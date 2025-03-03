/*	$NetBSD: bpf.c,v 1.249 2022/11/30 06:02:37 ozaki-r Exp $	*/

/*
 * Copyright (c) 1990, 1991, 1993
 *	The Regents of the University of California.  All rights reserved.
 *
 * This code is derived from the Stanford/CMU enet packet filter,
 * (net/enet.c) distributed as part of 4.3BSD, and code contributed
 * to Berkeley by Steven McCanne and Van Jacobson both of Lawrence
 * Berkeley Laboratory.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *	@(#)bpf.c	8.4 (Berkeley) 1/9/95
 * static char rcsid[] =
 * "Header: bpf.c,v 1.67 96/09/26 22:00:52 leres Exp ";
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD: bpf.c,v 1.249 2022/11/30 06:02:37 ozaki-r Exp $");

#if defined(_KERNEL_OPT)
#include "opt_bpf.h"
#include "sl.h"
#include "opt_net_mpsafe.h"
#endif

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/mbuf.h>
#include <sys/buf.h>
#include <sys/time.h>
#include <sys/proc.h>
#include <sys/ioctl.h>
#include <sys/conf.h>
#include <sys/vnode.h>
#include <sys/queue.h>
#include <sys/stat.h>
#include <sys/module.h>
#include <sys/atomic.h>
#include <sys/cpu.h>

#include <sys/file.h>
#include <sys/filedesc.h>
#include <sys/tty.h>
#include <sys/uio.h>

#include <sys/protosw.h>
#include <sys/socket.h>
#include <sys/errno.h>
#include <sys/kernel.h>
#include <sys/poll.h>
#include <sys/sysctl.h>
#include <sys/kauth.h>
#include <sys/syslog.h>
#include <sys/percpu.h>
#include <sys/pserialize.h>
#include <sys/lwp.h>
#include <sys/xcall.h>

#include <net/if.h>
#include <net/slip.h>

#include <net/bpf.h>
#include <net/bpfdesc.h>
#include <net/bpfjit.h>

#include <net/if_arc.h>
#include <net/if_ether.h>
#include <net/if_types.h>

#include <netinet/in.h>
#include <netinet/if_inarp.h>


#include <compat/sys/sockio.h>

#ifndef BPF_BUFSIZE
/*
 * 4096 is too small for FDDI frames. 8192 is too small for gigabit Ethernet
 * jumbos (circa 9k), ATM, or Intel gig/10gig ethernet jumbos (16k).
 */
# define BPF_BUFSIZE 32768
#endif

#define PRINET  26			/* interruptible */

/*
 * The default read buffer size, and limit for BIOCSBLEN, is sysctl'able.
 * XXX the default values should be computed dynamically based
 * on available memory size and available mbuf clusters.
 */
static int bpf_bufsize = BPF_BUFSIZE;
static int bpf_maxbufsize = BPF_DFLTBUFSIZE;	/* XXX set dynamically, see above */
static bool bpf_jit = false;

struct bpfjit_ops bpfjit_module_ops = {
	.bj_generate_code = NULL,
	.bj_free_code = NULL
};

/*
 * Global BPF statistics returned by net.bpf.stats sysctl.
 */
static struct percpu	*bpf_gstats_percpu; /* struct bpf_stat */

#define BPF_STATINC(id)					\
	{						\
		struct bpf_stat *__stats =		\
		    percpu_getref(bpf_gstats_percpu);	\
		__stats->bs_##id++;			\
		percpu_putref(bpf_gstats_percpu);	\
	}

/*
 * Locking notes:
 * - bpf_mtx (adaptive mutex) protects:
 *   - Gobal lists: bpf_iflist and bpf_dlist
 *   - struct bpf_if
 *   - bpf_close
 *   - bpf_psz (pserialize)
 * - struct bpf_d has two mutexes:
 *   - bd_buf_mtx (spin mutex) protects the buffers that can be accessed
 *     on packet tapping
 *   - bd_mtx (adaptive mutex) protects member variables other than the buffers
 * - Locking order: bpf_mtx => bpf_d#bd_mtx => bpf_d#bd_buf_mtx
 * - struct bpf_d obtained via fp->f_bpf in bpf_read and bpf_write is
 *   never freed because struct bpf_d is only freed in bpf_close and
 *   bpf_close never be called while executing bpf_read and bpf_write
 * - A filter that is assigned to bpf_d can be replaced with another filter
 *   while tapping packets, so it needs to be done atomically
 * - struct bpf_d is iterated on bpf_dlist with psz
 * - struct bpf_if is iterated on bpf_iflist with psz or psref
 */
/*
 * Use a mutex to avoid a race condition between gathering the stats/peers
 * and opening/closing the device.
 */
static kmutex_t bpf_mtx;

static struct psref_class	*bpf_psref_class __read_mostly;
static pserialize_t		bpf_psz;

static inline void
bpf_if_acquire(struct bpf_if *bp, struct psref *psref)
{

	psref_acquire(psref, &bp->bif_psref, bpf_psref_class);
}

static inline void
bpf_if_release(struct bpf_if *bp, struct psref *psref)
{

	psref_release(psref, &bp->bif_psref, bpf_psref_class);
}

/*
 *  bpf_iflist is the list of interfaces; each corresponds to an ifnet
 *  bpf_dtab holds the descriptors, indexed by minor device #
 */
static struct pslist_head bpf_iflist;
static struct pslist_head bpf_dlist;

/* Macros for bpf_d on bpf_dlist */
#define BPF_DLIST_WRITER_INSERT_HEAD(__d)				\
	PSLIST_WRITER_INSERT_HEAD(&bpf_dlist, (__d), bd_bpf_dlist_entry)
#define BPF_DLIST_READER_FOREACH(__d)					\
	PSLIST_READER_FOREACH((__d), &bpf_dlist, struct bpf_d,		\
	                      bd_bpf_dlist_entry)
#define BPF_DLIST_WRITER_FOREACH(__d)					\
	PSLIST_WRITER_FOREACH((__d), &bpf_dlist, struct bpf_d,		\
	                      bd_bpf_dlist_entry)
#define BPF_DLIST_ENTRY_INIT(__d)					\
	PSLIST_ENTRY_INIT((__d), bd_bpf_dlist_entry)
#define BPF_DLIST_WRITER_REMOVE(__d)					\
	PSLIST_WRITER_REMOVE((__d), bd_bpf_dlist_entry)
#define BPF_DLIST_ENTRY_DESTROY(__d)					\
	PSLIST_ENTRY_DESTROY((__d), bd_bpf_dlist_entry)

/* Macros for bpf_if on bpf_iflist */
#define BPF_IFLIST_WRITER_INSERT_HEAD(__bp)				\
	PSLIST_WRITER_INSERT_HEAD(&bpf_iflist, (__bp), bif_iflist_entry)
#define BPF_IFLIST_READER_FOREACH(__bp)					\
	PSLIST_READER_FOREACH((__bp), &bpf_iflist, struct bpf_if,	\
	                      bif_iflist_entry)
#define BPF_IFLIST_WRITER_FOREACH(__bp)					\
	PSLIST_WRITER_FOREACH((__bp), &bpf_iflist, struct bpf_if,	\
	                      bif_iflist_entry)
#define BPF_IFLIST_WRITER_REMOVE(__bp)					\
	PSLIST_WRITER_REMOVE((__bp), bif_iflist_entry)
#define BPF_IFLIST_ENTRY_INIT(__bp)					\
	PSLIST_ENTRY_INIT((__bp), bif_iflist_entry)
#define BPF_IFLIST_ENTRY_DESTROY(__bp)					\
	PSLIST_ENTRY_DESTROY((__bp), bif_iflist_entry)

/* Macros for bpf_d on bpf_if#bif_dlist_pslist */
#define BPFIF_DLIST_READER_FOREACH(__d, __bp)				\
	PSLIST_READER_FOREACH((__d), &(__bp)->bif_dlist_head, struct bpf_d, \
	                      bd_bif_dlist_entry)
#define BPFIF_DLIST_WRITER_INSERT_HEAD(__bp, __d)			\
	PSLIST_WRITER_INSERT_HEAD(&(__bp)->bif_dlist_head, (__d),	\
	                          bd_bif_dlist_entry)
#define BPFIF_DLIST_WRITER_REMOVE(__d)					\
	PSLIST_WRITER_REMOVE((__d), bd_bif_dlist_entry)
#define BPFIF_DLIST_ENTRY_INIT(__d)					\
	PSLIST_ENTRY_INIT((__d), bd_bif_dlist_entry)
#define	BPFIF_DLIST_READER_EMPTY(__bp)					\
	(PSLIST_READER_FIRST(&(__bp)->bif_dlist_head, struct bpf_d,	\
	                     bd_bif_dlist_entry) == NULL)
#define	BPFIF_DLIST_WRITER_EMPTY(__bp)					\
	(PSLIST_WRITER_FIRST(&(__bp)->bif_dlist_head, struct bpf_d,	\
	                     bd_bif_dlist_entry) == NULL)
#define BPFIF_DLIST_ENTRY_DESTROY(__d)					\
	PSLIST_ENTRY_DESTROY((__d), bd_bif_dlist_entry)

static int	bpf_allocbufs(struct bpf_d *);
static u_int	bpf_xfilter(struct bpf_filter **, void *, u_int, u_int);
static void	bpf_deliver(struct bpf_if *,
		            void *(*cpfn)(void *, const void *, size_t),
		            void *, u_int, u_int, const u_int);
static void	bpf_freed(struct bpf_d *);
static void	bpf_free_filter(struct bpf_filter *);
static void	bpf_ifname(struct ifnet *, struct ifreq *);
static void	*bpf_mcpy(void *, const void *, size_t);
static int	bpf_movein(struct ifnet *, struct uio *, int, uint64_t,
			        struct mbuf **, struct sockaddr *,
				struct bpf_filter **);
static void	bpf_attachd(struct bpf_d *, struct bpf_if *);
static void	bpf_detachd(struct bpf_d *);
static int	bpf_setif(struct bpf_d *, struct ifreq *);
static int	bpf_setf(struct bpf_d *, struct bpf_program *, u_long);
static void	bpf_timed_out(void *);
static inline void
		bpf_wakeup(struct bpf_d *);
static int	bpf_hdrlen(struct bpf_d *);
static void	catchpacket(struct bpf_d *, u_char *, u_int, u_int,
    void *(*)(void *, const void *, size_t), struct timespec *);
static void	reset_d(struct bpf_d *);
static int	bpf_getdltlist(struct bpf_d *, struct bpf_dltlist *);
static int	bpf_setdlt(struct bpf_d *, u_int);

static int	bpf_read(struct file *, off_t *, struct uio *, kauth_cred_t,
    int);
static int	bpf_write(struct file *, off_t *, struct uio *, kauth_cred_t,
    int);
static int	bpf_ioctl(struct file *, u_long, void *);
static int	bpf_poll(struct file *, int);
static int	bpf_stat(struct file *, struct stat *);
static int	bpf_close(struct file *);
static int	bpf_kqfilter(struct file *, struct knote *);

static const struct fileops bpf_fileops = {
	.fo_name = "bpf",
	.fo_read = bpf_read,
	.fo_write = bpf_write,
	.fo_ioctl = bpf_ioctl,
	.fo_fcntl = fnullop_fcntl,
	.fo_poll = bpf_poll,
	.fo_stat = bpf_stat,
	.fo_close = bpf_close,
	.fo_kqfilter = bpf_kqfilter,
	.fo_restart = fnullop_restart,
};

dev_type_open(bpfopen);

const struct cdevsw bpf_cdevsw = {
	.d_open = bpfopen,
	.d_close = noclose,
	.d_read = noread,
	.d_write = nowrite,
	.d_ioctl = noioctl,
	.d_stop = nostop,
	.d_tty = notty,
	.d_poll = nopoll,
	.d_mmap = nommap,
	.d_kqfilter = nokqfilter,
	.d_discard = nodiscard,
	.d_flag = D_OTHER | D_MPSAFE
};

bpfjit_func_t
bpf_jit_generate(bpf_ctx_t *bc, void *code, size_t size)
{
	struct bpfjit_ops *ops = &bpfjit_module_ops;
	bpfjit_func_t (*generate_code)(const bpf_ctx_t *,
	    const struct bpf_insn *, size_t);

	generate_code = atomic_load_acquire(&ops->bj_generate_code);
	if (generate_code != NULL) {
		return generate_code(bc, code, size);
	}
	return NULL;
}

void
bpf_jit_freecode(bpfjit_func_t jcode)
{
	KASSERT(bpfjit_module_ops.bj_free_code != NULL);
	bpfjit_module_ops.bj_free_code(jcode);
}

static int
bpf_movein(struct ifnet *ifp, struct uio *uio, int linktype, uint64_t mtu, struct mbuf **mp,
	   struct sockaddr *sockp, struct bpf_filter **wfilter)
{
	struct mbuf *m, *m0, *n;
	int error;
	size_t len;
	size_t hlen;
	size_t align;
	u_int slen;

	/*
	 * Build a sockaddr based on the data link layer type.
	 * We do this at this level because the ethernet header
	 * is copied directly into the data field of the sockaddr.
	 * In the case of SLIP, there is no header and the packet
	 * is forwarded as is.
	 * Also, we are careful to leave room at the front of the mbuf
	 * for the link level header.
	 */
	switch (linktype) {

	case DLT_SLIP:
		sockp->sa_family = AF_INET;
		hlen = 0;
		align = 0;
		break;

	case DLT_PPP:
		sockp->sa_family = AF_UNSPEC;
		hlen = 0;
		align = 0;
		break;

	case DLT_EN10MB:
		sockp->sa_family = AF_UNSPEC;
		/* XXX Would MAXLINKHDR be better? */
 		/* 6(dst)+6(src)+2(type) */
		hlen = sizeof(struct ether_header);
		align = 2;
		break;

	case DLT_ARCNET:
		sockp->sa_family = AF_UNSPEC;
		hlen = ARC_HDRLEN;
		align = 5;
		break;

	case DLT_FDDI:
		sockp->sa_family = AF_LINK;
		/* XXX 4(FORMAC)+6(dst)+6(src) */
		hlen = 16;
		align = 0;
		break;

	case DLT_ECONET:
		sockp->sa_family = AF_UNSPEC;
		hlen = 6;
		align = 2;
		break;

	case DLT_NULL:
		sockp->sa_family = AF_UNSPEC;
		if (ifp->if_type == IFT_LOOP) {
			/* Set here to apply the following validations */
			hlen = sizeof(uint32_t);
		} else
			hlen = 0;
		align = 0;
		break;

	default:
		return (EIO);
	}

	len = uio->uio_resid;
	/*
	 * If there aren't enough bytes for a link level header or the
	 * packet length exceeds the interface mtu, return an error.
	 */
	if (len - hlen > mtu)
		return (EMSGSIZE);

	m0 = m = m_gethdr(M_WAIT, MT_DATA);
	m_reset_rcvif(m);
	m->m_pkthdr.len = (int)(len - hlen);
	if (len + align > MHLEN) {
		m_clget(m, M_WAIT);
		if ((m->m_flags & M_EXT) == 0) {
			error = ENOBUFS;
			goto bad;
		}
	}

	/* Insure the data is properly aligned */
	if (align > 0)
		m->m_data += align;

	for (;;) {
		len = M_TRAILINGSPACE(m);
		if (len > uio->uio_resid)
			len = uio->uio_resid;
		error = uiomove(mtod(m, void *), len, uio);
		if (error)
			goto bad;
		m->m_len = len;

		if (uio->uio_resid == 0)
			break;

		n = m_get(M_WAIT, MT_DATA);
		m_clget(n, M_WAIT);	/* if fails, there is no problem */
		m->m_next = n;
		m = n;
	}

	slen = bpf_xfilter(wfilter, mtod(m, u_char *), len, len);
	if (slen == 0) {
		error = EPERM;
		goto bad;
	}

	if (hlen != 0) {
		if (linktype == DLT_NULL && ifp->if_type == IFT_LOOP) {
			uint32_t af;
			/* the link header indicates the address family */
			memcpy(&af, mtod(m0, void *), sizeof(af));
			sockp->sa_family = af;
		} else {
			/* move link level header in the top of mbuf to sa_data */
			memcpy(sockp->sa_data, mtod(m0, void *), hlen);
		}
		m0->m_data += hlen;
		m0->m_len -= hlen;
	}

	*mp = m0;
	return (0);

bad:
	m_freem(m0);
	return (error);
}

/*
 * Attach file to the bpf interface, i.e. make d listen on bp.
 */
static void
bpf_attachd(struct bpf_d *d, struct bpf_if *bp)
{
	struct bpf_event_tracker *t;

	KASSERT(mutex_owned(&bpf_mtx));
	KASSERT(mutex_owned(d->bd_mtx));
	/*
	 * Point d at bp, and add d to the interface's list of listeners.
	 * Finally, point the driver's bpf cookie at the interface so
	 * it will divert packets to bpf.
	 */
	d->bd_bif = bp;
	BPFIF_DLIST_WRITER_INSERT_HEAD(bp, d);

	*bp->bif_driverp = bp;

	SLIST_FOREACH(t, &bp->bif_trackers, bet_entries) {
		t->bet_notify(bp, bp->bif_ifp, bp->bif_dlt,
		    BPF_TRACK_EVENT_ATTACH);
	}
}

/*
 * Detach a file from its interface.
 */
static void
bpf_detachd(struct bpf_d *d)
{
	struct bpf_if *bp;
	struct bpf_event_tracker *t;

	KASSERT(mutex_owned(&bpf_mtx));
	KASSERT(mutex_owned(d->bd_mtx));

	bp = d->bd_bif;
	/*
	 * Check if this descriptor had requested promiscuous mode.
	 * If so, turn it off.
	 */
	if (d->bd_promisc) {
		int error __diagused;

		d->bd_promisc = 0;
		/*
		 * Take device out of promiscuous mode.  Since we were
		 * able to enter promiscuous mode, we should be able
		 * to turn it off.  But we can get an error if
		 * the interface was configured down, so only panic
		 * if we don't get an unexpected error.
		 */
		KERNEL_LOCK_UNLESS_NET_MPSAFE();
  		error = ifpromisc(bp->bif_ifp, 0);
		KERNEL_UNLOCK_UNLESS_NET_MPSAFE();
#ifdef DIAGNOSTIC
		if (error)
			printf("%s: ifpromisc failed: %d", __func__, error);
#endif
	}

	/* Remove d from the interface's descriptor list. */
	BPFIF_DLIST_WRITER_REMOVE(d);

	pserialize_perform(bpf_psz);

	if (BPFIF_DLIST_WRITER_EMPTY(bp)) {
		/*
		 * Let the driver know that there are no more listeners.
		 */
		*d->bd_bif->bif_driverp = NULL;
	}

	d->bd_bif = NULL;

	SLIST_FOREACH(t, &bp->bif_trackers, bet_entries) {
		t->bet_notify(bp, bp->bif_ifp, bp->bif_dlt,
		    BPF_TRACK_EVENT_DETACH);
	}
}

static void
bpf_init(void)
{

	mutex_init(&bpf_mtx, MUTEX_DEFAULT, IPL_NONE);
	bpf_psz = pserialize_create();
	bpf_psref_class = psref_class_create("bpf", IPL_SOFTNET);

	PSLIST_INIT(&bpf_iflist);
	PSLIST_INIT(&bpf_dlist);

	bpf_gstats_percpu = percpu_alloc(sizeof(struct bpf_stat));

	return;
}

/*
 * bpfilterattach() is called at boot time.  We don't need to do anything
 * here, since any initialization will happen as part of module init code.
 */
/* ARGSUSED */
void
bpfilterattach(int n)
{

}

/*
 * Open ethernet device. Clones.
 */
/* ARGSUSED */
int
bpfopen(dev_t dev, int flag, int mode, struct lwp *l)
{
	struct bpf_d *d;
	struct file *fp;
	int error, fd;

	/* falloc() will fill in the descriptor for us. */
	if ((error = fd_allocfile(&fp, &fd)) != 0)
		return error;

	d = kmem_zalloc(sizeof(*d), KM_SLEEP);
	d->bd_bufsize = bpf_bufsize;
	d->bd_direction = BPF_D_INOUT;
	d->bd_feedback = 0;
	d->bd_pid = l->l_proc->p_pid;
#ifdef _LP64
	if (curproc->p_flag & PK_32)
		d->bd_compat32 = 1;
#endif
	getnanotime(&d->bd_btime);
	d->bd_atime = d->bd_mtime = d->bd_btime;
	callout_init(&d->bd_callout, CALLOUT_MPSAFE);
	selinit(&d->bd_sel);
	d->bd_jitcode = NULL;
	d->bd_rfilter = NULL;
	d->bd_wfilter = NULL;
	d->bd_locked = 0;
	BPF_DLIST_ENTRY_INIT(d);
	BPFIF_DLIST_ENTRY_INIT(d);
	d->bd_mtx = mutex_obj_alloc(MUTEX_DEFAULT, IPL_SOFTNET);
	d->bd_buf_mtx = mutex_obj_alloc(MUTEX_DEFAULT, IPL_NET);
	cv_init(&d->bd_cv, "bpf");

	mutex_enter(&bpf_mtx);
	BPF_DLIST_WRITER_INSERT_HEAD(d);
	mutex_exit(&bpf_mtx);

	return fd_clone(fp, fd, flag, &bpf_fileops, d);
}

/*
 * Close the descriptor by detaching it from its interface,
 * deallocating its buffers, and marking it free.
 */
/* ARGSUSED */
static int
bpf_close(struct file *fp)
{
	struct bpf_d *d;

	mutex_enter(&bpf_mtx);

	if ((d = fp->f_bpf) == NULL) {
		mutex_exit(&bpf_mtx);
		return 0;
	}

	/*
	 * Refresh the PID associated with this bpf file.
	 */
	d->bd_pid = curproc->p_pid;

	mutex_enter(d->bd_mtx);
	if (d->bd_state == BPF_WAITING)
		callout_halt(&d->bd_callout, d->bd_mtx);
	d->bd_state = BPF_IDLE;
	if (d->bd_bif)
		bpf_detachd(d);
	mutex_exit(d->bd_mtx);

	BPF_DLIST_WRITER_REMOVE(d);

	pserialize_perform(bpf_psz);
	mutex_exit(&bpf_mtx);

	BPFIF_DLIST_ENTRY_DESTROY(d);
	BPF_DLIST_ENTRY_DESTROY(d);
	fp->f_bpf = NULL;
	bpf_freed(d);
	callout_destroy(&d->bd_callout);
	seldestroy(&d->bd_sel);
	mutex_obj_free(d->bd_mtx);
	mutex_obj_free(d->bd_buf_mtx);
	cv_destroy(&d->bd_cv);

	kmem_free(d, sizeof(*d));

	return (0);
}

/*
 * Rotate the packet buffers in descriptor d.  Move the store buffer
 * into the hold slot, and the free buffer into the store slot.
 * Zero the length of the new store buffer.
 */
#define ROTATE_BUFFERS(d) \
	(d)->bd_hbuf = (d)->bd_sbuf; \
	(d)->bd_hlen = (d)->bd_slen; \
	(d)->bd_sbuf = (d)->bd_fbuf; \
	(d)->bd_slen = 0; \
	(d)->bd_fbuf = NULL;
/*
 *  bpfread - read next chunk of packets from buffers
 */
static int
bpf_read(struct file *fp, off_t *offp, struct uio *uio,
    kauth_cred_t cred, int flags)
{
	struct bpf_d *d = fp->f_bpf;
	int timed_out;
	int error;

	/*
	 * Refresh the PID associated with this bpf file.
	 */
	d->bd_pid = curproc->p_pid;

	getnanotime(&d->bd_atime);
	/*
	 * Restrict application to use a buffer the same size as
	 * the kernel buffers.
	 */
	if (uio->uio_resid != d->bd_bufsize)
		return (EINVAL);

	mutex_enter(d->bd_mtx);
	if (d->bd_state == BPF_WAITING)
		callout_halt(&d->bd_callout, d->bd_mtx);
	timed_out = (d->bd_state == BPF_TIMED_OUT);
	d->bd_state = BPF_IDLE;
	mutex_exit(d->bd_mtx);
	/*
	 * If the hold buffer is empty, then do a timed sleep, which
	 * ends when the timeout expires or when enough packets
	 * have arrived to fill the store buffer.
	 */
	mutex_enter(d->bd_buf_mtx);
	while (d->bd_hbuf == NULL) {
		if (fp->f_flag & FNONBLOCK) {
			if (d->bd_slen == 0) {
				error = EWOULDBLOCK;
				goto out;
			}
			ROTATE_BUFFERS(d);
			break;
		}

		if ((d->bd_immediate || timed_out) && d->bd_slen != 0) {
			/*
			 * A packet(s) either arrived since the previous
			 * read or arrived while we were asleep.
			 * Rotate the buffers and return what's here.
			 */
			ROTATE_BUFFERS(d);
			break;
		}

		error = cv_timedwait_sig(&d->bd_cv, d->bd_buf_mtx, d->bd_rtout);

		if (error == EINTR || error == ERESTART)
			goto out;

		if (error == EWOULDBLOCK) {
			/*
			 * On a timeout, return what's in the buffer,
			 * which may be nothing.  If there is something
			 * in the store buffer, we can rotate the buffers.
			 */
			if (d->bd_hbuf)
				/*
				 * We filled up the buffer in between
				 * getting the timeout and arriving
				 * here, so we don't need to rotate.
				 */
				break;

			if (d->bd_slen == 0) {
				error = 0;
				goto out;
			}
			ROTATE_BUFFERS(d);
			break;
		}
		if (error != 0)
			goto out;
	}
	/*
	 * At this point, we know we have something in the hold slot.
	 */
	mutex_exit(d->bd_buf_mtx);

	/*
	 * Move data from hold buffer into user space.
	 * We know the entire buffer is transferred since
	 * we checked above that the read buffer is bpf_bufsize bytes.
	 */
	error = uiomove(d->bd_hbuf, d->bd_hlen, uio);

	mutex_enter(d->bd_buf_mtx);
	d->bd_fbuf = d->bd_hbuf;
	d->bd_hbuf = NULL;
	d->bd_hlen = 0;
out:
	mutex_exit(d->bd_buf_mtx);
	return (error);
}


/*
 * If there are processes sleeping on this descriptor, wake them up.
 */
static inline void
bpf_wakeup(struct bpf_d *d)
{

	mutex_enter(d->bd_buf_mtx);
	cv_broadcast(&d->bd_cv);
	mutex_exit(d->bd_buf_mtx);

	if (d->bd_async)
		fownsignal(d->bd_pgid, SIGIO, 0, 0, NULL);
	selnotify(&d->bd_sel, 0, 0);
}

static void
bpf_timed_out(void *arg)
{
	struct bpf_d *d = arg;

	mutex_enter(d->bd_mtx);
	if (d->bd_state == BPF_WAITING) {
		d->bd_state = BPF_TIMED_OUT;
		if (d->bd_slen != 0)
			bpf_wakeup(d);
	}
	mutex_exit(d->bd_mtx);
}


static int
bpf_write(struct file *fp, off_t *offp, struct uio *uio,
    kauth_cred_t cred, int flags)
{
	struct bpf_d *d = fp->f_bpf;
	struct bpf_if *bp;
	struct ifnet *ifp;
	struct mbuf *m, *mc;
	int error;
	static struct sockaddr_storage dst;
	struct psref psref;
	int bound;

	/*
	 * Refresh the PID associated with this bpf file.
	 */
	d->bd_pid = curproc->p_pid;

	m = NULL;	/* XXX gcc */

	bound = curlwp_bind();
	mutex_enter(d->bd_mtx);
	bp = d->bd_bif;
	if (bp == NULL) {
		mutex_exit(d->bd_mtx);
		error = ENXIO;
		goto out_bindx;
	}
	bpf_if_acquire(bp, &psref);
	mutex_exit(d->bd_mtx);

	getnanotime(&d->bd_mtime);

	ifp = bp->bif_ifp;
	if (if_is_deactivated(ifp)) {
		error = ENXIO;
		goto out;
	}

	if (uio->uio_resid == 0) {
		error = 0;
		goto out;
	}

	error = bpf_movein(ifp, uio, (int)bp->bif_dlt, ifp->if_mtu, &m,
		(struct sockaddr *) &dst, &d->bd_wfilter);
	if (error)
		goto out;

	if (m->m_pkthdr.len > ifp->if_mtu) {
		m_freem(m);
		error = EMSGSIZE;
		goto out;
	}

	if (d->bd_hdrcmplt)
		dst.ss_family = pseudo_AF_HDRCMPLT;

	if (d->bd_feedback) {
		mc = m_dup(m, 0, M_COPYALL, M_NOWAIT);
		if (mc != NULL)
			m_set_rcvif(mc, ifp);
		/* Set M_PROMISC for outgoing packets to be discarded. */
		if (1 /*d->bd_direction == BPF_D_INOUT*/)
			m->m_flags |= M_PROMISC;
	} else  
		mc = NULL;

	error = if_output_lock(ifp, ifp, m, (struct sockaddr *) &dst, NULL);

	if (mc != NULL) {
		if (error == 0) {
			int s = splsoftnet();
			KERNEL_LOCK_UNLESS_IFP_MPSAFE(ifp);
			ifp->_if_input(ifp, mc);
			KERNEL_UNLOCK_UNLESS_IFP_MPSAFE(ifp);
			splx(s);
		} else
			m_freem(mc);
	}
	/*
	 * The driver frees the mbuf.
	 */
out:
	bpf_if_release(bp, &psref);
out_bindx:
	curlwp_bindx(bound);
	return error;
}

/*
 * Reset a descriptor by flushing its packet buffer and clearing the
 * receive and drop counts.
 */
static void
reset_d(struct bpf_d *d)
{

	KASSERT(mutex_owned(d->bd_mtx));

	mutex_enter(d->bd_buf_mtx);
	if (d->bd_hbuf) {
		/* Free the hold buffer. */
		d->bd_fbuf = d->bd_hbuf;
		d->bd_hbuf = NULL;
	}
	d->bd_slen = 0;
	d->bd_hlen = 0;
	d->bd_rcount = 0;
	d->bd_dcount = 0;
	d->bd_ccount = 0;
	mutex_exit(d->bd_buf_mtx);
}

/*
 *  FIONREAD		Check for read packet available.
 *  BIOCGBLEN		Get buffer len [for read()].
 *  BIOCSETF		Set ethernet read filter.
 *  BIOCFLUSH		Flush read packet buffer.
 *  BIOCPROMISC		Put interface into promiscuous mode.
 *  BIOCGDLT		Get link layer type.
 *  BIOCGETIF		Get interface name.
 *  BIOCSETIF		Set interface.
 *  BIOCSRTIMEOUT	Set read timeout.
 *  BIOCGRTIMEOUT	Get read timeout.
 *  BIOCGSTATS		Get packet stats.
 *  BIOCIMMEDIATE	Set immediate mode.
 *  BIOCVERSION		Get filter language version.
 *  BIOCGHDRCMPLT	Get "header already complete" flag.
 *  BIOCSHDRCMPLT	Set "header already complete" flag.
 *  BIOCSFEEDBACK	Set packet feedback mode.
 *  BIOCGFEEDBACK	Get packet feedback mode.
 *  BIOCGDIRECTION	Get packet direction flag
 *  BIOCSDIRECTION	Set packet direction flag
 */
/* ARGSUSED */
static int
bpf_ioctl(struct file *fp, u_long cmd, void *addr)
{
	struct bpf_d *d = fp->f_bpf;
	int error = 0;

	/*
	 * Refresh the PID associated with this bpf file.
	 */
	d->bd_pid = curproc->p_pid;
#ifdef _LP64
	if (curproc->p_flag & PK_32)
		d->bd_compat32 = 1;
	else
		d->bd_compat32 = 0;
#endif

	mutex_enter(d->bd_mtx);
	if (d->bd_state == BPF_WAITING)
		callout_halt(&d->bd_callout, d->bd_mtx);
	d->bd_state = BPF_IDLE;
	mutex_exit(d->bd_mtx);

	if (d->bd_locked) {
		switch (cmd) {
		case BIOCGBLEN:		/* FALLTHROUGH */
		case BIOCFLUSH:		/* FALLTHROUGH */
		case BIOCGDLT:		/* FALLTHROUGH */
		case BIOCGDLTLIST:	/* FALLTHROUGH */
		case BIOCGETIF:		/* FALLTHROUGH */
		case BIOCGRTIMEOUT:	/* FALLTHROUGH */
		case BIOCGSTATS:	/* FALLTHROUGH */
		case BIOCVERSION:	/* FALLTHROUGH */
		case BIOCGHDRCMPLT:	/* FALLTHROUGH */
		case FIONREAD:		/* FALLTHROUGH */
		case BIOCLOCK:		/* FALLTHROUGH */
		case BIOCSRTIMEOUT:	/* FALLTHROUGH */
		case BIOCIMMEDIATE:	/* FALLTHROUGH */
		case TIOCGPGRP:
			break;
		default:
			return EPERM;
		}
	}

	switch (cmd) {

	default:
		error = EINVAL;
		break;

	/*
	 * Check for read packet available.
	 */
	case FIONREAD:
		{
			int n;

			mutex_enter(d->bd_buf_mtx);
			n = d->bd_slen;
			if (d->bd_hbuf)
				n += d->bd_hlen;
			mutex_exit(d->bd_buf_mtx);

			*(int *)addr = n;
			break;
		}

	/*
	 * Get buffer len [for read()].
	 */
	case BIOCGBLEN:
		*(u_int *)addr = d->bd_bufsize;
		break;

	/*
	 * Set buffer length.
	 */
	case BIOCSBLEN:
		/*
		 * Forbid to change the buffer length if buffers are already
		 * allocated.
		 */
		mutex_enter(d->bd_mtx);
		mutex_enter(d->bd_buf_mtx);
		if (d->bd_bif != NULL || d->bd_sbuf != NULL)
			error = EINVAL;
		else {
			u_int size = *(u_int *)addr;

			if (size > bpf_maxbufsize)
				*(u_int *)addr = size = bpf_maxbufsize;
			else if (size < BPF_MINBUFSIZE)
				*(u_int *)addr = size = BPF_MINBUFSIZE;
			d->bd_bufsize = size;
		}
		mutex_exit(d->bd_buf_mtx);
		mutex_exit(d->bd_mtx);
		break;

	/*
	 * Set link layer read filter.
	 */
	case BIOCSETF:		/* FALLTHROUGH */
	case BIOCSETWF:
		error = bpf_setf(d, addr, cmd);
		break;

	case BIOCLOCK:
		d->bd_locked = 1;
		break;

	/*
	 * Flush read packet buffer.
	 */
	case BIOCFLUSH:
		mutex_enter(d->bd_mtx);
		reset_d(d);
		mutex_exit(d->bd_mtx);
		break;

	/*
	 * Put interface into promiscuous mode.
	 */
	case BIOCPROMISC:
		mutex_enter(d->bd_mtx);
		if (d->bd_bif == NULL) {
			mutex_exit(d->bd_mtx);
			/*
			 * No interface attached yet.
			 */
			error = EINVAL;
			break;
		}
		if (d->bd_promisc == 0) {
			KERNEL_LOCK_UNLESS_NET_MPSAFE();
			error = ifpromisc(d->bd_bif->bif_ifp, 1);
			KERNEL_UNLOCK_UNLESS_NET_MPSAFE();
			if (error == 0)
				d->bd_promisc = 1;
		}
		mutex_exit(d->bd_mtx);
		break;

	/*
	 * Get device parameters.
	 */
	case BIOCGDLT:
		mutex_enter(d->bd_mtx);
		if (d->bd_bif == NULL)
			error = EINVAL;
		else
			*(u_int *)addr = d->bd_bif->bif_dlt;
		mutex_exit(d->bd_mtx);
		break;

	/*
	 * Get a list of supported device parameters.
	 */
	case BIOCGDLTLIST:
		mutex_enter(d->bd_mtx);
		if (d->bd_bif == NULL)
			error = EINVAL;
		else
			error = bpf_getdltlist(d, addr);
		mutex_exit(d->bd_mtx);
		break;

	/*
	 * Set device parameters.
	 */
	case BIOCSDLT:
		mutex_enter(&bpf_mtx);
		mutex_enter(d->bd_mtx);
		if (d->bd_bif == NULL)
			error = EINVAL;
		else
			error = bpf_setdlt(d, *(u_int *)addr);
		mutex_exit(d->bd_mtx);
		mutex_exit(&bpf_mtx);
		break;

	/*
	 * Set interface name.
	 */
#ifdef OBIOCGETIF
	case OBIOCGETIF:
#endif
	case BIOCGETIF:
		mutex_enter(d->bd_mtx);
		if (d->bd_bif == NULL)
			error = EINVAL;
		else
			bpf_ifname(d->bd_bif->bif_ifp, addr);
		mutex_exit(d->bd_mtx);
		break;

	/*
	 * Set interface.
	 */
#ifdef OBIOCSETIF
	case OBIOCSETIF:
#endif
	case BIOCSETIF:
		mutex_enter(&bpf_mtx);
		error = bpf_setif(d, addr);
		mutex_exit(&bpf_mtx);
		break;

	/*
	 * Set read timeout.
	 */
	case BIOCSRTIMEOUT:
		{
			struct timeval *tv = addr;

			/* Compute number of ticks. */
			if (tv->tv_sec < 0 ||
			    tv->tv_usec < 0 || tv->tv_usec >= 1000000) {
				error = EINVAL;
				break;
			} else if (tv->tv_sec > INT_MAX/hz - 1) {
				d->bd_rtout = INT_MAX;
			} else {
				d->bd_rtout = tv->tv_sec * hz
				    + tv->tv_usec / tick;
			}
			if ((d->bd_rtout == 0) && (tv->tv_usec != 0))
				d->bd_rtout = 1;
			break;
		}

#ifdef BIOCGORTIMEOUT
	/*
	 * Get read timeout.
	 */
	case BIOCGORTIMEOUT:
		{
			struct timeval50 *tv = addr;

			tv->tv_sec = d->bd_rtout / hz;
			tv->tv_usec = (d->bd_rtout % hz) * tick;
			break;
		}
#endif

#ifdef BIOCSORTIMEOUT
	/*
	 * Set read timeout.
	 */
	case BIOCSORTIMEOUT:
		{
			struct timeval50 *tv = addr;

			/* Compute number of ticks. */
			if (tv->tv_sec < 0 ||
			    tv->tv_usec < 0 || tv->tv_usec >= 1000000) {
				error = EINVAL;
				break;
			} else if (tv->tv_sec > INT_MAX/hz - 1) {
				d->bd_rtout = INT_MAX;
			} else {
				d->bd_rtout = tv->tv_sec * hz
				    + tv->tv_usec / tick;
			}
			if ((d->bd_rtout == 0) && (tv->tv_usec != 0))
				d->bd_rtout = 1;
			break;
		}
#endif

	/*
	 * Get read timeout.
	 */
	case BIOCGRTIMEOUT:
		{
			struct timeval *tv = addr;

			tv->tv_sec = d->bd_rtout / hz;
			tv->tv_usec = (d->bd_rtout % hz) * tick;
			break;
		}
	/*
	 * Get packet stats.
	 */
	case BIOCGSTATS:
		{
			struct bpf_stat *bs = addr;

			bs->bs_recv = d->bd_rcount;
			bs->bs_drop = d->bd_dcount;
			bs->bs_capt = d->bd_ccount;
			break;
		}

	case BIOCGSTATSOLD:
		{
			struct bpf_stat_old *bs = addr;

			bs->bs_recv = d->bd_rcount;
			bs->bs_drop = d->bd_dcount;
			break;
		}

	/*
	 * Set immediate mode.
	 */
	case BIOCIMMEDIATE:
		d->bd_immediate = *(u_int *)addr;
		break;

	case BIOCVERSION:
		{
			struct bpf_version *bv = addr;

			bv->bv_major = BPF_MAJOR_VERSION;
			bv->bv_minor = BPF_MINOR_VERSION;
			break;
		}

	case BIOCGHDRCMPLT:	/* get "header already complete" flag */
		*(u_int *)addr = d->bd_hdrcmplt;
		break;

	case BIOCSHDRCMPLT:	/* set "header already complete" flag */
		d->bd_hdrcmplt = *(u_int *)addr ? 1 : 0;
		break;

	/*
	 * Get packet direction flag
	 */
	case BIOCGDIRECTION:
		*(u_int *)addr = d->bd_direction;
		break;

	/*
	 * Set packet direction flag
	 */
	case BIOCSDIRECTION:
		{
			u_int	direction;

			direction = *(u_int *)addr;
			switch (direction) {
			case BPF_D_IN:
			case BPF_D_INOUT:
			case BPF_D_OUT:
				d->bd_direction = direction;
				break;
			default:
				error = EINVAL;
			}
		}
		break;

	/*
	 * Set "feed packets from bpf back to input" mode
	 */
	case BIOCSFEEDBACK:
		d->bd_feedback = *(u_int *)addr;
		break;

	/*
	 * Get "feed packets from bpf back to input" mode
	 */
	case BIOCGFEEDBACK:
		*(u_int *)addr = d->bd_feedback;
		break;

	case FIONBIO:		/* Non-blocking I/O */
		/*
		 * No need to do anything special as we use IO_NDELAY in
		 * bpfread() as an indication of whether or not to block
		 * the read.
		 */
		break;

	case FIOASYNC:		/* Send signal on receive packets */
		mutex_enter(d->bd_mtx);
		d->bd_async = *(int *)addr;
		mutex_exit(d->bd_mtx);
		break;

	case TIOCSPGRP:		/* Process or group to send signals to */
	case FIOSETOWN:
		error = fsetown(&d->bd_pgid, cmd, addr);
		break;

	case TIOCGPGRP:
	case FIOGETOWN:
		error = fgetown(d->bd_pgid, cmd, addr);
		break;
	}
	return (error);
}

/*
 * Set d's packet filter program to fp.  If this file already has a filter,
 * free it and replace it.  Returns EINVAL for bogus requests.
 */
static int
bpf_setf(struct bpf_d *d, struct bpf_program *fp, u_long cmd)
{
	struct bpf_insn *fcode;
	bpfjit_func_t jcode;
	size_t flen, size = 0;
	struct bpf_filter *oldf, *newf, **storef;

	jcode = NULL;
	flen = fp->bf_len;

	if ((fp->bf_insns == NULL && flen) || flen > BPF_MAXINSNS) {
		return EINVAL;
	}

	if (flen) {
		/*
		 * Allocate the buffer, copy the byte-code from
		 * userspace and validate it.
		 */
		size = flen * sizeof(*fp->bf_insns);
		fcode = kmem_alloc(size, KM_SLEEP);
		if (copyin(fp->bf_insns, fcode, size) != 0 ||
		    !bpf_validate(fcode, (int)flen)) {
			kmem_free(fcode, size);
			return EINVAL;
		}
		if (bpf_jit)
			jcode = bpf_jit_generate(NULL, fcode, flen);
	} else {
		fcode = NULL;
	}

	newf = kmem_alloc(sizeof(*newf), KM_SLEEP);
	newf->bf_insn = fcode;
	newf->bf_size = size;
	newf->bf_jitcode = jcode;
	if (cmd == BIOCSETF)
		d->bd_jitcode = jcode; /* XXX just for kvm(3) users */

	/* Need to hold bpf_mtx for pserialize_perform */
	mutex_enter(&bpf_mtx);
	mutex_enter(d->bd_mtx);
	if (cmd == BIOCSETWF) {
		oldf = d->bd_wfilter;
		storef = &d->bd_wfilter;
	} else {
		oldf = d->bd_rfilter;
		storef = &d->bd_rfilter;
	}
	atomic_store_release(storef, newf);
	reset_d(d);
	pserialize_perform(bpf_psz);
	mutex_exit(d->bd_mtx);
	mutex_exit(&bpf_mtx);

	if (oldf != NULL)
		bpf_free_filter(oldf);

	return 0;
}

/*
 * Detach a file from its current interface (if attached at all) and attach
 * to the interface indicated by the name stored in ifr.
 * Return an errno or 0.
 */
static int
bpf_setif(struct bpf_d *d, struct ifreq *ifr)
{
	struct bpf_if *bp;
	char *cp;
	int unit_seen, i, error;

	KASSERT(mutex_owned(&bpf_mtx));
	/*
	 * Make sure the provided name has a unit number, and default
	 * it to '0' if not specified.
	 * XXX This is ugly ... do this differently?
	 */
	unit_seen = 0;
	cp = ifr->ifr_name;
	cp[sizeof(ifr->ifr_name) - 1] = '\0';	/* sanity */
	while (*cp++)
		if (*cp >= '0' && *cp <= '9')
			unit_seen = 1;
	if (!unit_seen) {
		/* Make sure to leave room for the '\0'. */
		for (i = 0; i < (IFNAMSIZ - 1); ++i) {
			if ((ifr->ifr_name[i] >= 'a' &&
			     ifr->ifr_name[i] <= 'z') ||
			    (ifr->ifr_name[i] >= 'A' &&
			     ifr->ifr_name[i] <= 'Z'))
				continue;
			ifr->ifr_name[i] = '0';
		}
	}

	/*
	 * Look through attached interfaces for the named one.
	 */
	BPF_IFLIST_WRITER_FOREACH(bp) {
		struct ifnet *ifp = bp->bif_ifp;

		if (ifp == NULL ||
		    strcmp(ifp->if_xname, ifr->ifr_name) != 0)
			continue;
		/* skip additional entry */
		if (bp->bif_driverp != &ifp->if_bpf)
			continue;
		/*
		 * We found the requested interface.
		 * Allocate the packet buffers if we need to.
		 * If we're already attached to requested interface,
		 * just flush the buffer.
		 */
		/*
		 * bpf_allocbufs is called only here. bpf_mtx ensures that
		 * no race condition happen on d->bd_sbuf.
		 */
		if (d->bd_sbuf == NULL) {
			error = bpf_allocbufs(d);
			if (error != 0)
				return (error);
		}
		mutex_enter(d->bd_mtx);
		if (bp != d->bd_bif) {
			if (d->bd_bif) {
				/*
				 * Detach if attached to something else.
				 */
				bpf_detachd(d);
				BPFIF_DLIST_ENTRY_INIT(d);
			}

			bpf_attachd(d, bp);
		}
		reset_d(d);
		mutex_exit(d->bd_mtx);
		return (0);
	}
	/* Not found. */
	return (ENXIO);
}

/*
 * Copy the interface name to the ifreq.
 */
static void
bpf_ifname(struct ifnet *ifp, struct ifreq *ifr)
{
	memcpy(ifr->ifr_name, ifp->if_xname, IFNAMSIZ);
}

static int
bpf_stat(struct file *fp, struct stat *st)
{
	struct bpf_d *d = fp->f_bpf;

	(void)memset(st, 0, sizeof(*st));
	mutex_enter(d->bd_mtx);
	st->st_dev = makedev(cdevsw_lookup_major(&bpf_cdevsw), d->bd_pid);
	st->st_atimespec = d->bd_atime;
	st->st_mtimespec = d->bd_mtime;
	st->st_ctimespec = st->st_birthtimespec = d->bd_btime;
	st->st_uid = kauth_cred_geteuid(fp->f_cred);
	st->st_gid = kauth_cred_getegid(fp->f_cred);
	st->st_mode = S_IFCHR;
	mutex_exit(d->bd_mtx);
	return 0;
}

/*
 * Support for poll() system call
 *
 * Return true iff the specific operation will not block indefinitely - with
 * the assumption that it is safe to positively acknowledge a request for the
 * ability to write to the BPF device.
 * Otherwise, return false but make a note that a selnotify() must be done.
 */
static int
bpf_poll(struct file *fp, int events)
{
	struct bpf_d *d = fp->f_bpf;
	int revents;

	/*
	 * Refresh the PID associated with this bpf file.
	 */
	mutex_enter(&bpf_mtx);
	d->bd_pid = curproc->p_pid;

	revents = events & (POLLOUT | POLLWRNORM);
	if (events & (POLLIN | POLLRDNORM)) {
		/*
		 * An imitation of the FIONREAD ioctl code.
		 */
		mutex_enter(d->bd_mtx);
		if (d->bd_hlen != 0 ||
		    ((d->bd_immediate || d->bd_state == BPF_TIMED_OUT) &&
		     d->bd_slen != 0)) {
			revents |= events & (POLLIN | POLLRDNORM);
		} else {
			selrecord(curlwp, &d->bd_sel);
			/* Start the read timeout if necessary */
			if (d->bd_rtout > 0 && d->bd_state == BPF_IDLE) {
				callout_reset(&d->bd_callout, d->bd_rtout,
					      bpf_timed_out, d);
				d->bd_state = BPF_WAITING;
			}
		}
		mutex_exit(d->bd_mtx);
	}

	mutex_exit(&bpf_mtx);
	return (revents);
}

static void
filt_bpfrdetach(struct knote *kn)
{
	struct bpf_d *d = kn->kn_hook;

	mutex_enter(d->bd_buf_mtx);
	selremove_knote(&d->bd_sel, kn);
	mutex_exit(d->bd_buf_mtx);
}

static int
filt_bpfread(struct knote *kn, long hint)
{
	struct bpf_d *d = kn->kn_hook;
	int rv;

	/*
	 * Refresh the PID associated with this bpf file.
	 */
	d->bd_pid = curproc->p_pid;

	mutex_enter(d->bd_buf_mtx);
	kn->kn_data = d->bd_hlen;
	if (d->bd_immediate)
		kn->kn_data += d->bd_slen;
	rv = (kn->kn_data > 0);
	mutex_exit(d->bd_buf_mtx);
	return rv;
}

static const struct filterops bpfread_filtops = {
	.f_flags = FILTEROP_ISFD,
	.f_attach = NULL,
	.f_detach = filt_bpfrdetach,
	.f_event = filt_bpfread,
};

static int
bpf_kqfilter(struct file *fp, struct knote *kn)
{
	struct bpf_d *d = fp->f_bpf;

	switch (kn->kn_filter) {
	case EVFILT_READ:
		kn->kn_fop = &bpfread_filtops;
		break;

	default:
		return (EINVAL);
	}

	kn->kn_hook = d;

	mutex_enter(d->bd_buf_mtx);
	selrecord_knote(&d->bd_sel, kn);
	mutex_exit(d->bd_buf_mtx);

	return (0);
}

/*
 * Copy data from an mbuf chain into a buffer.  This code is derived
 * from m_copydata in sys/uipc_mbuf.c.
 */
static void *
bpf_mcpy(void *dst_arg, const void *src_arg, size_t len)
{
	const struct mbuf *m;
	u_int count;
	u_char *dst;

	m = src_arg;
	dst = dst_arg;
	while (len > 0) {
		if (m == NULL)
			panic("bpf_mcpy");
		count = uimin(m->m_len, len);
		memcpy(dst, mtod(m, const void *), count);
		m = m->m_next;
		dst += count;
		len -= count;
	}
	return dst_arg;
}

static inline u_int
bpf_xfilter(struct bpf_filter **filter, void *pkt, u_int pktlen, u_int buflen)
{
	struct bpf_filter *filt;
	uint32_t mem[BPF_MEMWORDS];
	bpf_args_t args = {
		.pkt = (const uint8_t *)pkt,
		.wirelen = pktlen,
		.buflen = buflen,
		.mem = mem,
		.arg = NULL
	};
	u_int slen;

	filt = atomic_load_consume(filter);
	if (filt == NULL) /* No filter means accept all. */
		return (u_int)-1;

	if (filt->bf_jitcode != NULL)
		slen = filt->bf_jitcode(NULL, &args);
	else
		slen = bpf_filter_ext(NULL, filt->bf_insn, &args);
	return slen;
}

/*
 * Dispatch a packet to all the listeners on interface bp.
 *
 * pkt       pointer to the packet, either a data buffer or an mbuf chain
 * buflen    buffer length, if pkt is a data buffer
 * cpfn      a function that can copy pkt into the listener's buffer
 * pktlen    length of the packet
 * direction BPF_D_IN or BPF_D_OUT
 */
static inline void
bpf_deliver(struct bpf_if *bp, void *(*cpfn)(void *, const void *, size_t),
    void *pkt, u_int pktlen, u_int buflen, const u_int direction)
{
	bool gottime = false;
	struct timespec ts;
	struct bpf_d *d;
	int s;
	u_int slen;

	KASSERT(!cpu_intr_p());

	/*
	 * Note that the IPL does not have to be raised at this point.
	 * The only problem that could arise here is that if two different
	 * interfaces shared any data.  This is not the case.
	 */
	s = pserialize_read_enter();
	BPFIF_DLIST_READER_FOREACH(d, bp) {
		if (direction == BPF_D_IN) {
			if (d->bd_direction == BPF_D_OUT)
				continue;
		} else { /* BPF_D_OUT */
			if (d->bd_direction == BPF_D_IN)
				continue;
		}

		atomic_inc_ulong(&d->bd_rcount);
		BPF_STATINC(recv);

		slen = bpf_xfilter(&d->bd_rfilter, pkt, pktlen, buflen);
		if (slen == 0)
			continue;

		if (!gottime) {
			gottime = true;
			nanotime(&ts);
		}
		/* Assume catchpacket doesn't sleep */
		catchpacket(d, pkt, pktlen, slen, cpfn, &ts);
	}
	pserialize_read_exit(s);
}

/*
 * Incoming linkage from device drivers, when the head of the packet is in
 * a buffer, and the tail is in an mbuf chain.
 */
static void
_bpf_mtap2(struct bpf_if *bp, void *data, u_int dlen, struct mbuf *m,
	u_int direction)
{
	u_int pktlen;
	struct mbuf mb;

	/* Skip outgoing duplicate packets. */
	if ((m->m_flags & M_PROMISC) != 0 && m->m_pkthdr.rcvif_index == 0) {
		m->m_flags &= ~M_PROMISC;
		return;
	}

	pktlen = m_length(m) + dlen;

	/*
	 * Craft on-stack mbuf suitable for passing to bpf_filter.
	 * Note that we cut corners here; we only setup what's
	 * absolutely needed--this mbuf should never go anywhere else.
	 */
	(void)memset(&mb, 0, sizeof(mb));
	mb.m_type = MT_DATA;
	mb.m_next = m;
	mb.m_data = data;
	mb.m_len = dlen;

	bpf_deliver(bp, bpf_mcpy, &mb, pktlen, 0, direction);
}

/*
 * Incoming linkage from device drivers, when packet is in an mbuf chain.
 */
static void
_bpf_mtap(struct bpf_if *bp, struct mbuf *m, u_int direction)
{
	void *(*cpfn)(void *, const void *, size_t);
	u_int pktlen, buflen;
	void *marg;

	/* Skip outgoing duplicate packets. */
	if ((m->m_flags & M_PROMISC) != 0 && m->m_pkthdr.rcvif_index == 0) {
		m->m_flags &= ~M_PROMISC;
		return;
	}

	pktlen = m_length(m);

	/* Skip zero-sized packets. */
	if (__predict_false(pktlen == 0)) {
		return;
	}

	if (pktlen == m->m_len) {
		cpfn = (void *)memcpy;
		marg = mtod(m, void *);
		buflen = pktlen;
		KASSERT(buflen != 0);
	} else {
		cpfn = bpf_mcpy;
		marg = m;
		buflen = 0;
	}

	bpf_deliver(bp, cpfn, marg, pktlen, buflen, direction);
}

/*
 * We need to prepend the address family as
 * a four byte field.  Cons up a dummy header
 * to pacify bpf.  This is safe because bpf
 * will only read from the mbuf (i.e., it won't
 * try to free it or keep a pointer a to it).
 */
static void
_bpf_mtap_af(struct bpf_if *bp, uint32_t af, struct mbuf *m, u_int direction)
{
	struct mbuf m0;

	m0.m_type = MT_DATA;
	m0.m_flags = 0;
	m0.m_next = m;
	m0.m_nextpkt = NULL;
	m0.m_owner = NULL;
	m0.m_len = 4;
	m0.m_data = (char *)&af;

	_bpf_mtap(bp, &m0, direction);
}

/*
 * Put the SLIP pseudo-"link header" in place.
 * Note this M_PREPEND() should never fail,
 * swince we know we always have enough space
 * in the input buffer.
 */
static void
_bpf_mtap_sl_in(struct bpf_if *bp, u_char *chdr, struct mbuf **m)
{
	u_char *hp;

	M_PREPEND(*m, SLIP_HDRLEN, M_DONTWAIT);
	if (*m == NULL)
		return;

	hp = mtod(*m, u_char *);
	hp[SLX_DIR] = SLIPDIR_IN;
	(void)memcpy(&hp[SLX_CHDR], chdr, CHDR_LEN);

	_bpf_mtap(bp, *m, BPF_D_IN);

	m_adj(*m, SLIP_HDRLEN);
}

/*
 * Put the SLIP pseudo-"link header" in
 * place.  The compressed header is now
 * at the beginning of the mbuf.
 */
static void
_bpf_mtap_sl_out(struct bpf_if *bp, u_char *chdr, struct mbuf *m)
{
	struct mbuf m0;
	u_char *hp;

	m0.m_type = MT_DATA;
	m0.m_flags = 0;
	m0.m_next = m;
	m0.m_nextpkt = NULL;
	m0.m_owner = NULL;
	m0.m_data = m0.m_dat;
	m0.m_len = SLIP_HDRLEN;

	hp = mtod(&m0, u_char *);

	hp[SLX_DIR] = SLIPDIR_OUT;
	(void)memcpy(&hp[SLX_CHDR], chdr, CHDR_LEN);

	_bpf_mtap(bp, &m0, BPF_D_OUT);
	m_freem(m);
}

static struct mbuf *
bpf_mbuf_enqueue(struct bpf_if *bp, struct mbuf *m)
{
	struct mbuf *dup;

	dup = m_dup(m, 0, M_COPYALL, M_NOWAIT);
	if (dup == NULL)
		return NULL;

	if (bp->bif_mbuf_tail != NULL) {
		bp->bif_mbuf_tail->m_nextpkt = dup;
	} else {
		bp->bif_mbuf_head = dup;
	}
	bp->bif_mbuf_tail = dup;
#ifdef BPF_MTAP_SOFTINT_DEBUG
	log(LOG_DEBUG, "%s: enqueued mbuf=%p to %s\n",
	    __func__, dup, bp->bif_ifp->if_xname);
#endif

	return dup;
}

static struct mbuf *
bpf_mbuf_dequeue(struct bpf_if *bp)
{
	struct mbuf *m;
	int s;

	/* XXX NOMPSAFE: assumed running on one CPU */
	s = splnet();
	m = bp->bif_mbuf_head;
	if (m != NULL) {
		bp->bif_mbuf_head = m->m_nextpkt;
		m->m_nextpkt = NULL;

		if (bp->bif_mbuf_head == NULL)
			bp->bif_mbuf_tail = NULL;
#ifdef BPF_MTAP_SOFTINT_DEBUG
		log(LOG_DEBUG, "%s: dequeued mbuf=%p from %s\n",
		    __func__, m, bp->bif_ifp->if_xname);
#endif
	}
	splx(s);

	return m;
}

static void
bpf_mtap_si(void *arg)
{
	struct bpf_if *bp = arg;
	struct mbuf *m;

	while ((m = bpf_mbuf_dequeue(bp)) != NULL) {
#ifdef BPF_MTAP_SOFTINT_DEBUG
		log(LOG_DEBUG, "%s: tapping mbuf=%p on %s\n",
		    __func__, m, bp->bif_ifp->if_xname);
#endif
		bpf_ops->bpf_mtap(bp, m, BPF_D_IN);
		m_freem(m);
	}
}

static void
_bpf_mtap_softint(struct ifnet *ifp, struct mbuf *m)
{
	struct bpf_if *bp = ifp->if_bpf;
	struct mbuf *dup;

	KASSERT(cpu_intr_p());

	/* To avoid extra invocations of the softint */
	if (BPFIF_DLIST_READER_EMPTY(bp))
		return;
	KASSERT(bp->bif_si != NULL);

	dup = bpf_mbuf_enqueue(bp, m);
	if (dup != NULL)
		softint_schedule(bp->bif_si);
}

static int
bpf_hdrlen(struct bpf_d *d)
{
	int hdrlen = d->bd_bif->bif_hdrlen;
	/*
	 * Compute the length of the bpf header.  This is not necessarily
	 * equal to SIZEOF_BPF_HDR because we want to insert spacing such
	 * that the network layer header begins on a longword boundary (for
	 * performance reasons and to alleviate alignment restrictions).
	 */
#ifdef _LP64
	if (d->bd_compat32)
		return (BPF_WORDALIGN32(hdrlen + SIZEOF_BPF_HDR32) - hdrlen);
	else
#endif
		return (BPF_WORDALIGN(hdrlen + SIZEOF_BPF_HDR) - hdrlen);
}

/*
 * Move the packet data from interface memory (pkt) into the
 * store buffer. Call the wakeup functions if it's time to wakeup
 * a listener (buffer full), "cpfn" is the routine called to do the
 * actual data transfer. memcpy is passed in to copy contiguous chunks,
 * while bpf_mcpy is passed in to copy mbuf chains.  In the latter case,
 * pkt is really an mbuf.
 */
static void
catchpacket(struct bpf_d *d, u_char *pkt, u_int pktlen, u_int snaplen,
    void *(*cpfn)(void *, const void *, size_t), struct timespec *ts)
{
	char *h;
	int totlen, curlen, caplen;
	int hdrlen = bpf_hdrlen(d);
	int do_wakeup = 0;

	atomic_inc_ulong(&d->bd_ccount);
	BPF_STATINC(capt);
	/*
	 * Figure out how many bytes to move.  If the packet is
	 * greater or equal to the snapshot length, transfer that
	 * much.  Otherwise, transfer the whole packet (unless
	 * we hit the buffer size limit).
	 */
	totlen = hdrlen + uimin(snaplen, pktlen);
	if (totlen > d->bd_bufsize)
		totlen = d->bd_bufsize;
	/*
	 * If we adjusted totlen to fit the bufsize, it could be that
	 * totlen is smaller than hdrlen because of the link layer header.
	 */
	caplen = totlen - hdrlen;
	if (caplen < 0)
		caplen = 0;

	mutex_enter(d->bd_buf_mtx);
	/*
	 * Round up the end of the previous packet to the next longword.
	 */
#ifdef _LP64
	if (d->bd_compat32)
		curlen = BPF_WORDALIGN32(d->bd_slen);
	else
#endif
		curlen = BPF_WORDALIGN(d->bd_slen);
	if (curlen + totlen > d->bd_bufsize) {
		/*
		 * This packet will overflow the storage buffer.
		 * Rotate the buffers if we can, then wakeup any
		 * pending reads.
		 */
		if (d->bd_fbuf == NULL) {
			mutex_exit(d->bd_buf_mtx);
			/*
			 * We haven't completed the previous read yet,
			 * so drop the packet.
			 */
			atomic_inc_ulong(&d->bd_dcount);
			BPF_STATINC(drop);
			return;
		}
		ROTATE_BUFFERS(d);
		do_wakeup = 1;
		curlen = 0;
	} else if (d->bd_immediate || d->bd_state == BPF_TIMED_OUT) {
		/*
		 * Immediate mode is set, or the read timeout has
		 * already expired during a select call.  A packet
		 * arrived, so the reader should be woken up.
		 */
		do_wakeup = 1;
	}

	/*
	 * Append the bpf header.
	 */
	h = (char *)d->bd_sbuf + curlen;
#ifdef _LP64
	if (d->bd_compat32) {
		struct bpf_hdr32 *hp32;

		hp32 = (struct bpf_hdr32 *)h;
		hp32->bh_tstamp.tv_sec = ts->tv_sec;
		hp32->bh_tstamp.tv_usec = ts->tv_nsec / 1000;
		hp32->bh_datalen = pktlen;
		hp32->bh_hdrlen = hdrlen;
		hp32->bh_caplen = caplen;
	} else
#endif
	{
		struct bpf_hdr *hp;

		hp = (struct bpf_hdr *)h;
		hp->bh_tstamp.tv_sec = ts->tv_sec;
		hp->bh_tstamp.tv_usec = ts->tv_nsec / 1000;
		hp->bh_datalen = pktlen;
		hp->bh_hdrlen = hdrlen;
		hp->bh_caplen = caplen;
	}

	/*
	 * Copy the packet data into the store buffer and update its length.
	 */
	(*cpfn)(h + hdrlen, pkt, caplen);
	d->bd_slen = curlen + totlen;
	mutex_exit(d->bd_buf_mtx);

	/*
	 * Call bpf_wakeup after bd_slen has been updated so that kevent(2)
	 * will cause filt_bpfread() to be called with it adjusted.
	 */
	if (do_wakeup)
		bpf_wakeup(d);
}

/*
 * Initialize all nonzero fields of a descriptor.
 */
static int
bpf_allocbufs(struct bpf_d *d)
{

	d->bd_fbuf = kmem_zalloc(d->bd_bufsize, KM_NOSLEEP);
	if (!d->bd_fbuf)
		return (ENOBUFS);
	d->bd_sbuf = kmem_zalloc(d->bd_bufsize, KM_NOSLEEP);
	if (!d->bd_sbuf) {
		kmem_free(d->bd_fbuf, d->bd_bufsize);
		return (ENOBUFS);
	}
	d->bd_slen = 0;
	d->bd_hlen = 0;
	return (0);
}

static void
bpf_free_filter(struct bpf_filter *filter)
{

	KASSERT(filter != NULL);

	if (filter->bf_insn != NULL)
		kmem_free(filter->bf_insn, filter->bf_size);
	if (filter->bf_jitcode != NULL)
		bpf_jit_freecode(filter->bf_jitcode);
	kmem_free(filter, sizeof(*filter));
}

/*
 * Free buffers currently in use by a descriptor.
 * Called on close.
 */
static void
bpf_freed(struct bpf_d *d)
{
	/*
	 * We don't need to lock out interrupts since this descriptor has
	 * been detached from its interface and it yet hasn't been marked
	 * free.
	 */
	if (d->bd_sbuf != NULL) {
		kmem_free(d->bd_sbuf, d->bd_bufsize);
		if (d->bd_hbuf != NULL)
			kmem_free(d->bd_hbuf, d->bd_bufsize);
		if (d->bd_fbuf != NULL)
			kmem_free(d->bd_fbuf, d->bd_bufsize);
	}
	if (d->bd_rfilter != NULL) {
		bpf_free_filter(d->bd_rfilter);
		d->bd_rfilter = NULL;
	}
	if (d->bd_wfilter != NULL) {
		bpf_free_filter(d->bd_wfilter);
		d->bd_wfilter = NULL;
	}
	d->bd_jitcode = NULL;
}

/*
 * Attach an interface to bpf.  dlt is the link layer type;
 * hdrlen is the fixed size of the link header for the specified dlt
 * (variable length headers not yet supported).
 */
static void
_bpfattach(struct ifnet *ifp, u_int dlt, u_int hdrlen, struct bpf_if **driverp)
{
	struct bpf_if *bp;

	bp = kmem_alloc(sizeof(*bp), KM_SLEEP);

	mutex_enter(&bpf_mtx);
	bp->bif_driverp = driverp;
	bp->bif_ifp = ifp;
	bp->bif_dlt = dlt;
	bp->bif_si = NULL;
	BPF_IFLIST_ENTRY_INIT(bp);
	PSLIST_INIT(&bp->bif_dlist_head);
	psref_target_init(&bp->bif_psref, bpf_psref_class);
	SLIST_INIT(&bp->bif_trackers);

	BPF_IFLIST_WRITER_INSERT_HEAD(bp);

	*bp->bif_driverp = NULL;

	bp->bif_hdrlen = hdrlen;
	mutex_exit(&bpf_mtx);
#if 0
	printf("bpf: %s attached with dlt %x\n", ifp->if_xname, dlt);
#endif
}

static void
_bpf_mtap_softint_init(struct ifnet *ifp)
{
	struct bpf_if *bp;

	mutex_enter(&bpf_mtx);
	BPF_IFLIST_WRITER_FOREACH(bp) {
		if (bp->bif_ifp != ifp)
			continue;

		bp->bif_mbuf_head = NULL;
		bp->bif_mbuf_tail = NULL;
		bp->bif_si = softint_establish(SOFTINT_NET, bpf_mtap_si, bp);
		if (bp->bif_si == NULL)
			panic("%s: softint_establish() failed", __func__);
		break;
	}
	mutex_exit(&bpf_mtx);

	if (bp == NULL)
		panic("%s: no bpf_if found for %s", __func__, ifp->if_xname);
}

/*
 * Remove an interface from bpf.
 */
static void
_bpfdetach(struct ifnet *ifp)
{
	struct bpf_if *bp;
	struct bpf_d *d;
	int s;

	mutex_enter(&bpf_mtx);
	/* Nuke the vnodes for any open instances */
  again_d:
	BPF_DLIST_WRITER_FOREACH(d) {
		mutex_enter(d->bd_mtx);
		if (d->bd_bif != NULL && d->bd_bif->bif_ifp == ifp) {
			/*
			 * Detach the descriptor from an interface now.
			 * It will be free'ed later by close routine.
			 */
			bpf_detachd(d);
			mutex_exit(d->bd_mtx);
			goto again_d;
		}
		mutex_exit(d->bd_mtx);
	}

  again:
	BPF_IFLIST_WRITER_FOREACH(bp) {
		if (bp->bif_ifp == ifp) {
			BPF_IFLIST_WRITER_REMOVE(bp);

			pserialize_perform(bpf_psz);
			psref_target_destroy(&bp->bif_psref, bpf_psref_class);

			while (!SLIST_EMPTY(&bp->bif_trackers)) {
				struct bpf_event_tracker *t =
				    SLIST_FIRST(&bp->bif_trackers);
				SLIST_REMOVE_HEAD(&bp->bif_trackers,
				    bet_entries);
				kmem_free(t, sizeof(*t));
			}

			BPF_IFLIST_ENTRY_DESTROY(bp);
			if (bp->bif_si != NULL) {
				/* XXX NOMPSAFE: assumed running on one CPU */
				s = splnet();
				while (bp->bif_mbuf_head != NULL) {
					struct mbuf *m = bp->bif_mbuf_head;
					bp->bif_mbuf_head = m->m_nextpkt;
					m_freem(m);
				}
				splx(s);
				softint_disestablish(bp->bif_si);
			}
			kmem_free(bp, sizeof(*bp));
			goto again;
		}
	}
	mutex_exit(&bpf_mtx);
}

/*
 * Change the data link type of a interface.
 */
static void
_bpf_change_type(struct ifnet *ifp, u_int dlt, u_int hdrlen)
{
	struct bpf_if *bp;

	mutex_enter(&bpf_mtx);
	BPF_IFLIST_WRITER_FOREACH(bp) {
		if (bp->bif_driverp == &ifp->if_bpf)
			break;
	}
	if (bp == NULL)
		panic("bpf_change_type");

	bp->bif_dlt = dlt;

	bp->bif_hdrlen = hdrlen;
	mutex_exit(&bpf_mtx);
}

/*
 * Get a list of available data link type of the interface.
 */
static int
bpf_getdltlist(struct bpf_d *d, struct bpf_dltlist *bfl)
{
	int n, error;
	struct ifnet *ifp;
	struct bpf_if *bp;
	int s, bound;

	KASSERT(mutex_owned(d->bd_mtx));

	ifp = d->bd_bif->bif_ifp;
	n = 0;
	error = 0;

	bound = curlwp_bind();
	s = pserialize_read_enter();
	BPF_IFLIST_READER_FOREACH(bp) {
		if (bp->bif_ifp != ifp)
			continue;
		if (bfl->bfl_list != NULL) {
			struct psref psref;

			if (n >= bfl->bfl_len) {
				pserialize_read_exit(s);
				return ENOMEM;
			}

			bpf_if_acquire(bp, &psref);
			pserialize_read_exit(s);

			error = copyout(&bp->bif_dlt,
			    bfl->bfl_list + n, sizeof(u_int));

			s = pserialize_read_enter();
			bpf_if_release(bp, &psref);
		}
		n++;
	}
	pserialize_read_exit(s);
	curlwp_bindx(bound);

	bfl->bfl_len = n;
	return error;
}

/*
 * Set the data link type of a BPF instance.
 */
static int
bpf_setdlt(struct bpf_d *d, u_int dlt)
{
	int error, opromisc;
	struct ifnet *ifp;
	struct bpf_if *bp;

	KASSERT(mutex_owned(&bpf_mtx));
	KASSERT(mutex_owned(d->bd_mtx));

	if (d->bd_bif->bif_dlt == dlt)
		return 0;
	ifp = d->bd_bif->bif_ifp;
	BPF_IFLIST_WRITER_FOREACH(bp) {
		if (bp->bif_ifp == ifp && bp->bif_dlt == dlt)
			break;
	}
	if (bp == NULL)
		return EINVAL;
	opromisc = d->bd_promisc;
	bpf_detachd(d);
	BPFIF_DLIST_ENTRY_INIT(d);
	bpf_attachd(d, bp);
	reset_d(d);
	if (opromisc) {
		KERNEL_LOCK_UNLESS_NET_MPSAFE();
		error = ifpromisc(bp->bif_ifp, 1);
		KERNEL_UNLOCK_UNLESS_NET_MPSAFE();
		if (error)
			printf("%s: bpf_setdlt: ifpromisc failed (%d)\n",
			    bp->bif_ifp->if_xname, error);
		else
			d->bd_promisc = 1;
	}
	return 0;
}

static int
sysctl_net_bpf_maxbufsize(SYSCTLFN_ARGS)
{
	int newsize, error;
	struct sysctlnode node;

	node = *rnode;
	node.sysctl_data = &newsize;
	newsize = bpf_maxbufsize;
	error = sysctl_lookup(SYSCTLFN_CALL(&node));
	if (error || newp == NULL)
		return (error);

	if (newsize < BPF_MINBUFSIZE || newsize > BPF_MAXBUFSIZE)
		return (EINVAL);

	bpf_maxbufsize = newsize;

	return (0);
}

#if defined(MODULAR) || defined(BPFJIT)
static int
sysctl_net_bpf_jit(SYSCTLFN_ARGS)
{
	bool newval;
	int error;
	struct sysctlnode node;

	node = *rnode;
	node.sysctl_data = &newval;
	newval = bpf_jit;
	error = sysctl_lookup(SYSCTLFN_CALL(&node));
	if (error != 0 || newp == NULL)
		return error;

	bpf_jit = newval;
	if (newval && bpfjit_module_ops.bj_generate_code == NULL) {
		printf("JIT compilation is postponed "
		    "until after bpfjit module is loaded\n");
	}

	return 0;
}
#endif

static int
sysctl_net_bpf_peers(SYSCTLFN_ARGS)
{
	int    error, elem_count;
	struct bpf_d	 *dp;
	struct bpf_d_ext  dpe;
	size_t len, needed, elem_size, out_size;
	char   *sp;

	if (namelen == 1 && name[0] == CTL_QUERY)
		return (sysctl_query(SYSCTLFN_CALL(rnode)));

	if (namelen != 2)
		return (EINVAL);

	/* BPF peers is privileged information. */
	error = kauth_authorize_network(l->l_cred, KAUTH_NETWORK_INTERFACE,
	    KAUTH_REQ_NETWORK_INTERFACE_GETPRIV, NULL, NULL, NULL);
	if (error)
		return (EPERM);

	len = (oldp != NULL) ? *oldlenp : 0;
	sp = oldp;
	elem_size = name[0];
	elem_count = name[1];
	out_size = MIN(sizeof(dpe), elem_size);
	needed = 0;

	if (elem_size < 1 || elem_count < 0)
		return (EINVAL);

	mutex_enter(&bpf_mtx);
	BPF_DLIST_WRITER_FOREACH(dp) {
		if (len >= elem_size && elem_count > 0) {
#define BPF_EXT(field)	dpe.bde_ ## field = dp->bd_ ## field
			BPF_EXT(bufsize);
			BPF_EXT(promisc);
			BPF_EXT(state);
			BPF_EXT(immediate);
			BPF_EXT(hdrcmplt);
			BPF_EXT(direction);
			BPF_EXT(pid);
			BPF_EXT(rcount);
			BPF_EXT(dcount);
			BPF_EXT(ccount);
#undef BPF_EXT
			mutex_enter(dp->bd_mtx);
			if (dp->bd_bif)
				(void)strlcpy(dpe.bde_ifname,
				    dp->bd_bif->bif_ifp->if_xname,
				    IFNAMSIZ - 1);
			else
				dpe.bde_ifname[0] = '\0';
			dpe.bde_locked = dp->bd_locked;
			mutex_exit(dp->bd_mtx);

			error = copyout(&dpe, sp, out_size);
			if (error)
				break;
			sp += elem_size;
			len -= elem_size;
		}
		needed += elem_size;
		if (elem_count > 0 && elem_count != INT_MAX)
			elem_count--;
	}
	mutex_exit(&bpf_mtx);

	*oldlenp = needed;

	return (error);
}

static void
bpf_stats(void *p, void *arg, struct cpu_info *ci __unused)
{
	struct bpf_stat *const stats = p;
	struct bpf_stat *sum = arg;

	int s = splnet();

	sum->bs_recv += stats->bs_recv;
	sum->bs_drop += stats->bs_drop;
	sum->bs_capt += stats->bs_capt;

	splx(s);
}

static int
bpf_sysctl_gstats_handler(SYSCTLFN_ARGS)
{
	struct sysctlnode node;
	int error;
	struct bpf_stat sum;

	memset(&sum, 0, sizeof(sum));
	node = *rnode;

	percpu_foreach_xcall(bpf_gstats_percpu, XC_HIGHPRI_IPL(IPL_SOFTNET),
	    bpf_stats, &sum);

	node.sysctl_data = &sum;
	node.sysctl_size = sizeof(sum);
	error = sysctl_lookup(SYSCTLFN_CALL(&node));
	if (error != 0 || newp == NULL)
		return error;

	return 0;
}

SYSCTL_SETUP(sysctl_net_bpf_setup, "bpf sysctls")
{
	const struct sysctlnode *node;

	node = NULL;
	sysctl_createv(clog, 0, NULL, &node,
		       CTLFLAG_PERMANENT,
		       CTLTYPE_NODE, "bpf",
		       SYSCTL_DESCR("BPF options"),
		       NULL, 0, NULL, 0,
		       CTL_NET, CTL_CREATE, CTL_EOL);
	if (node != NULL) {
#if defined(MODULAR) || defined(BPFJIT)
		sysctl_createv(clog, 0, NULL, NULL,
			CTLFLAG_PERMANENT|CTLFLAG_READWRITE,
			CTLTYPE_BOOL, "jit",
			SYSCTL_DESCR("Toggle Just-In-Time compilation"),
			sysctl_net_bpf_jit, 0, &bpf_jit, 0,
			CTL_NET, node->sysctl_num, CTL_CREATE, CTL_EOL);
#endif
		sysctl_createv(clog, 0, NULL, NULL,
			CTLFLAG_PERMANENT|CTLFLAG_READWRITE,
			CTLTYPE_INT, "maxbufsize",
			SYSCTL_DESCR("Maximum size for data capture buffer"),
			sysctl_net_bpf_maxbufsize, 0, &bpf_maxbufsize, 0,
			CTL_NET, node->sysctl_num, CTL_CREATE, CTL_EOL);
		sysctl_createv(clog, 0, NULL, NULL,
			CTLFLAG_PERMANENT,
			CTLTYPE_STRUCT, "stats",
			SYSCTL_DESCR("BPF stats"),
			bpf_sysctl_gstats_handler, 0, NULL, 0,
			CTL_NET, node->sysctl_num, CTL_CREATE, CTL_EOL);
		sysctl_createv(clog, 0, NULL, NULL,
			CTLFLAG_PERMANENT,
			CTLTYPE_STRUCT, "peers",
			SYSCTL_DESCR("BPF peers"),
			sysctl_net_bpf_peers, 0, NULL, 0,
			CTL_NET, node->sysctl_num, CTL_CREATE, CTL_EOL);
	}

}

static int
_bpf_register_track_event(struct bpf_if **driverp,
	    void (*_fun)(struct bpf_if *, struct ifnet *, int, int))
{
	struct bpf_if *bp;
	struct bpf_event_tracker *t;
	int ret = ENOENT;

	t = kmem_zalloc(sizeof(*t), KM_SLEEP);
	if (!t)
		return ENOMEM;
	t->bet_notify = _fun;

	mutex_enter(&bpf_mtx);
	BPF_IFLIST_WRITER_FOREACH(bp) {
		if (bp->bif_driverp != driverp)
			continue;
		SLIST_INSERT_HEAD(&bp->bif_trackers, t, bet_entries);
		ret = 0;
		break;
	}
	mutex_exit(&bpf_mtx);

	return ret;
}

static int
_bpf_deregister_track_event(struct bpf_if **driverp,
	    void (*_fun)(struct bpf_if *, struct ifnet *, int, int))
{
	struct bpf_if *bp;
	struct bpf_event_tracker *t = NULL;
	int ret = ENOENT;

	mutex_enter(&bpf_mtx);
	BPF_IFLIST_WRITER_FOREACH(bp) {
		if (bp->bif_driverp != driverp)
			continue;
		SLIST_FOREACH(t, &bp->bif_trackers, bet_entries) {
			if (t->bet_notify == _fun) {
				ret = 0;
				break;
			}
		}
		if (ret == 0)
			break;
	}
	if (ret == 0 && t && t->bet_notify == _fun) {
		SLIST_REMOVE(&bp->bif_trackers, t, bpf_event_tracker,
		    bet_entries);
	}
	mutex_exit(&bpf_mtx);
	if (ret == 0)
		kmem_free(t, sizeof(*t));
	return ret;
}

struct bpf_ops bpf_ops_kernel = {
	.bpf_attach =		_bpfattach,
	.bpf_detach =		_bpfdetach,
	.bpf_change_type =	_bpf_change_type,
	.bpf_register_track_event = _bpf_register_track_event,
	.bpf_deregister_track_event = _bpf_deregister_track_event,

	.bpf_mtap =		_bpf_mtap,
	.bpf_mtap2 =		_bpf_mtap2,
	.bpf_mtap_af =		_bpf_mtap_af,
	.bpf_mtap_sl_in =	_bpf_mtap_sl_in,
	.bpf_mtap_sl_out =	_bpf_mtap_sl_out,

	.bpf_mtap_softint =		_bpf_mtap_softint,
	.bpf_mtap_softint_init =	_bpf_mtap_softint_init,
};

MODULE(MODULE_CLASS_DRIVER, bpf, "bpf_filter");

static int
bpf_modcmd(modcmd_t cmd, void *arg)
{
#ifdef _MODULE
	devmajor_t bmajor, cmajor;
#endif
	int error = 0;

	switch (cmd) {
	case MODULE_CMD_INIT:
		bpf_init();
#ifdef _MODULE
		bmajor = cmajor = NODEVMAJOR;
		error = devsw_attach("bpf", NULL, &bmajor,
		    &bpf_cdevsw, &cmajor);
		if (error)
			break;
#endif

		bpf_ops_handover_enter(&bpf_ops_kernel);
		atomic_swap_ptr(&bpf_ops, &bpf_ops_kernel);
		bpf_ops_handover_exit();
		break;

	case MODULE_CMD_FINI:
		/*
		 * While there is no reference counting for bpf callers,
		 * unload could at least in theory be done similarly to 
		 * system call disestablishment.  This should even be
		 * a little simpler:
		 * 
		 * 1) replace op vector with stubs
		 * 2) post update to all cpus with xc
		 * 3) check that nobody is in bpf anymore
		 *    (it's doubtful we'd want something like l_sysent,
		 *     but we could do something like *signed* percpu
		 *     counters.  if the sum is 0, we're good).
		 * 4) if fail, unroll changes
		 *
		 * NOTE: change won't be atomic to the outside.  some
		 * packets may be not captured even if unload is
		 * not successful.  I think packet capture not working
		 * is a perfectly logical consequence of trying to
		 * disable packet capture.
		 */
		error = EOPNOTSUPP;
		break;

	default:
		error = ENOTTY;
		break;
	}

	return error;
}
