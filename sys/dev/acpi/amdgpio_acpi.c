/*	$NetBSD$	*/
/*
 * Copyright (c) 2016 Mark Kettenis
 * Copyright (c) 2019 James Hastings
 *
 * Permission to use, copy, modify, and distribute this software for 
any
 * purpose with or without fee is hereby granted, provided that the 
above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL 
WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE 
FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY 
DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN 
AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING 
OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/malloc.h>
#include <sys/device.h>
#include <sys/device_impl.h>
#include <sys/gpio.h>
#include <sys/kmem.h>

#include <dev/acpi/acpireg.h>
#include <dev/acpi/acpivar.h>
#include <dev/acpi/acpi_intr.h>
#include <dev/acpi/acpi_event.h>

#include <dev/gpio/gpiovar.h>

#define LR_GPIO_POLARITY	(3L << 1)
#define LR_GPIO_ACTHI		(0L << 1)
#define LR_GPIO_ACTLO		(1L << 1)
#define LR_GPIO_ACTBOTH		(2L << 1)
#define LR_GPIO_MODE		(1L << 0)
#define LR_GPIO_LEVEL		(0L << 0)

#define AMDGPIO_CONF_LEVEL		0x00000100
#define AMDGPIO_CONF_ACTLO		0x00000200
#define AMDGPIO_CONF_ACTBOTH		0x00000400
#define AMDGPIO_CONF_MASK		0x00000600
#define AMDGPIO_CONF_INT_EN		0x00000800
#define AMDGPIO_CONF_INT_MASK		0x00001000
#define AMDGPIO_CONF_RXSTATE		0x00010000
#define AMDGPIO_CONF_TXSTATE		0x00400000
#define AMDGPIO_CONF_TXSTATE_EN		0x00800000
#define AMDGPIO_CONF_INT_STS		0x10000000
#define AMDGPIO_IRQ_MASTER_EOI		0x20000000
#define AMDGPIO_IRQ_BITS		46
#define AMDGPIO_IRQ_PINS		4

#define AMDGPIO_IRQ_MASTER		0xfc
#define AMDGPIO_IRQ_STS			0x2f8

struct amdgpio_intrhand {
	int (*ih_func)(void *);
	void *ih_arg;
};

struct amdgpio_pincfg {
	/* Modeled after pchgpio but we only have one value to 
save/restore */
	uint32_t	pin_cfg;
};

struct amdgpio_softc {
	device_t sc_dev;
	struct acpi_softc *sc_acpi;
	ACPI_HANDLE sc_handle;

	bus_space_tag_t sc_memt;
	bus_space_handle_t sc_memh;
	bus_size_t sc_size;
	void *sc_ih;

	int sc_npins;
	int	sc_pmf;
	struct amdgpio_pincfg *sc_pin_cfg;
	struct amdgpio_intrhand *sc_pin_ih;

	struct gpio_chipset_tag	sc_gpio;
};

void amdgpio_acpi_register_event(void *, struct acpi_event *, ACPI_RESOURCE_GPIO *);
int	amdgpio_match(device_t, cfdata_t, void *);
void amdgpio_attach(device_t, device_t, void *);
int	amdgpio_activate(device_t, int);

CFATTACH_DECL_NEW(amdgpio_acpi, sizeof(struct amdgpio_softc), amdgpio_match, amdgpio_attach, NULL, NULL);

static const struct device_compatible_entry compat_data[] = {
	{ .compat = "AMDI0030" },
	{ .compat = "AMD0030" },
	DEVICE_COMPAT_EOL
};

int	amdgpio_read_pin(void *, int);
void amdgpio_write_pin(void *, int, int);
void * amdgpio_intr_establish(void *, int, int, int, int (*)(void *), void *);
void amdgpio_intr_enable(void *, int);
void amdgpio_intr_disable(void *, void *);
int	amdgpio_pin_intr(struct amdgpio_softc *, int);
int	amdgpio_intr(void *);
void amdgpio_save_pin(struct amdgpio_softc *, int pin);
bool amdgpio_save(device_t self, const pmf_qual_t *qual);
void amdgpio_restore_pin(struct amdgpio_softc *, int pin);
bool amdgpio_restore(device_t self, const pmf_qual_t *qual);

int
amdgpio_match(device_t parent, cfdata_t cf, void *aux)
{
	struct acpi_attach_args *aa = aux;

	return acpi_compatible_match(aa, compat_data);
}

void
amdgpio_attach(device_t parent, device_t self, void *aux)
{
	struct acpi_attach_args *aa = aux;
	struct amdgpio_softc * const sc = device_private(self);
	struct gpiobus_attach_args gba;
	struct acpi_resources res;
	struct acpi_mem *mem;
	struct acpi_irq *irq;
	ACPI_STATUS rv;
	ACPI_DEVICE_INFO *devinfo;
	int64_t uid;

	sc->sc_dev = self;
	sc->sc_acpi = (struct acpi_softc *)parent;
	sc->sc_memt = aa->aa_memt;

	sc->sc_handle = aa->aa_node->ad_handle;

	if (acpi_eval_integer(aa->aa_node->ad_handle, "_UID", &uid)) {
		printf(": can't find uid\n");
		return;
	}

	printf(" uid %ld", uid);

	switch (uid) {
	case 0:
		sc->sc_npins = 184;
		break;
	default:
		printf("\n");
		return;
	}

	rv = acpi_resource_parse(sc->sc_dev, aa->aa_node->ad_handle, "_CRS",
	    &res, &acpi_resource_parse_ops_default);
	if (ACPI_FAILURE(rv))
		return;

	rv = AcpiGetObjectInfo(sc->sc_handle, &devinfo);
	if (ACPI_FAILURE(rv)) {
                aprint_error_dev(sc->sc_dev, "AcpiGetObjectInfo failed\n");
                return;
        }
    aprint_normal_dev(sc->sc_dev, "_HID: %s, %x\n",
        devinfo->HardwareId.String, devinfo->Name);

	mem = acpi_res_mem(&res, 0);
	if (mem == NULL) {
		aprint_error_dev(self, "couldn't find mem resource\n");
		goto done;
	}

	irq = acpi_res_irq(&res, 0);
	if (irq == NULL) {
		aprint_error_dev(self, "couldn't find irq resource\n");
		goto done;
	}
	
	if (bus_space_map(aa->aa_memt, mem->ar_base, mem->ar_length,
	    0, &sc->sc_memh)) {
		aprint_error_dev(self, ": can't map registers\n");
		return;
	}

	sc->sc_pin_cfg = kmem_zalloc(sc->sc_npins * sizeof(sc->sc_pin_cfg[0]), KM_SLEEP);
	sc->sc_pin_ih = kmem_zalloc(sc->sc_npins * sizeof(sc->sc_pin_ih[0]), KM_SLEEP);

	sc->sc_ih = acpi_intr_establish(self, 
		(uint64_t)(uintptr_t)aa->aa_node->ad_handle,
	    IPL_BIO, false, amdgpio_intr, sc, sc->sc_dev->dv_xname);
	if (sc->sc_ih == NULL) {
		printf(": can't establish interrupt\n");
		goto unmap;
	}

	sc->sc_gpio.gp_cookie = sc;
	sc->sc_gpio.gp_pin_read = amdgpio_read_pin;
	sc->sc_gpio.gp_pin_write = amdgpio_write_pin;
	sc->sc_gpio.gp_intr_establish = amdgpio_intr_establish;
	//sc->sc_gpio.gp_intr_enable = amdgpio_intr_enable;
	sc->sc_gpio.gp_intr_disestablish = amdgpio_intr_disable;
	//sc->sc_node->gpio = &sc->sc_gpio;

	memset(&gba, 0, sizeof(gba));
	gba.gba_npins = sc->sc_npins;

	printf(", %d pins\n", sc->sc_npins);

#if NGPIO > 0
	config_found(sc->sc_dev, &gba, gpiobus_print, CFARGS_NONE);
#endif

	return;

unmap:
	kmem_free(sc->sc_pin_ih, sc->sc_npins * sizeof(*sc->sc_pin_ih));
	bus_space_unmap(sc->sc_memt, sc->sc_memh, sc->sc_size);
	sc->sc_size = 0;

done:
	acpi_resource_cleanup(&res);
	(void)pmf_device_register(self, amdgpio_save, amdgpio_restore);
	sc->sc_pmf = 1;

	kmem_free(sc->sc_pin_cfg, sc->sc_npins * sizeof(sc->sc_pin_cfg[0]));
	kmem_free(sc->sc_pin_ih, sc->sc_npins * sizeof(sc->sc_pin_ih[0]));

}

void
amdgpio_acpi_register_event(void *priv, struct acpi_event *ev, ACPI_RESOURCE_GPIO *gpio)
{
	return;
}

void
amdgpio_save_pin(struct amdgpio_softc *sc, int pin)
{
	sc->sc_pin_cfg[pin].pin_cfg = bus_space_read_4(sc->sc_memt, sc->sc_memh,
	    pin * 4);
}

bool
amdgpio_save(device_t self, const pmf_qual_t *qual)
{
	struct amdgpio_softc *sc = device_private(self);
	int pin;

	for (pin = 0; pin < sc->sc_npins; pin++)
		amdgpio_save_pin(sc, pin);

	return true;
}

void
amdgpio_restore_pin(struct amdgpio_softc *sc, int pin)
{
	if (!sc->sc_pin_ih[pin].ih_func)
		return;

	bus_space_write_4(sc->sc_memt, sc->sc_memh, pin * 4,
	    sc->sc_pin_cfg[pin].pin_cfg);
}

bool
amdgpio_restore(device_t self, const pmf_qual_t *qual)
{
	int pin;
	struct amdgpio_softc *sc = device_private(self);
	for (pin = 0; pin < sc->sc_npins; pin++)
		amdgpio_restore_pin(sc, pin);

	return true;
}

int
amdgpio_read_pin(void *cookie, int pin)
{
	struct amdgpio_softc *sc = cookie;
	uint32_t reg;

	reg = bus_space_read_4(sc->sc_memt, sc->sc_memh, pin * 4);

	return !!(reg & AMDGPIO_CONF_RXSTATE);
}

void
amdgpio_write_pin(void *cookie, int pin, int value)
{
	struct amdgpio_softc *sc = cookie;
	uint32_t reg;

	reg = bus_space_read_4(sc->sc_memt, sc->sc_memh, pin * 4);
	reg |= AMDGPIO_CONF_TXSTATE_EN;
	if (value)
		reg |= AMDGPIO_CONF_TXSTATE;
	else
		reg &= ~AMDGPIO_CONF_TXSTATE;
	bus_space_write_4(sc->sc_memt, sc->sc_memh, pin * 4, reg);
}

void *
amdgpio_intr_establish(void *cookie, int pin, int flags,
    int irqmode, int (*func)(void *), void *arg)
{
	struct amdgpio_softc *sc = cookie;
	uint32_t reg;

	KASSERT(pin >= 0 && pin != 63 && pin < sc->sc_npins);

	sc->sc_pin_ih[pin].ih_func = func;
	sc->sc_pin_ih[pin].ih_arg = arg;

	reg = bus_space_read_4(sc->sc_memt, sc->sc_memh, pin * 4);
	reg &= ~(AMDGPIO_CONF_MASK | AMDGPIO_CONF_LEVEL |
	    AMDGPIO_CONF_TXSTATE_EN);
	if ((flags & LR_GPIO_MODE) == 0)
		reg |= AMDGPIO_CONF_LEVEL;
	if ((flags & LR_GPIO_POLARITY) == LR_GPIO_ACTLO)
		reg |= AMDGPIO_CONF_ACTLO;
	if ((flags & LR_GPIO_POLARITY) == LR_GPIO_ACTBOTH)
		reg |= AMDGPIO_CONF_ACTBOTH;
	reg |= (AMDGPIO_CONF_INT_MASK | AMDGPIO_CONF_INT_EN);
	bus_space_write_4(sc->sc_memt, sc->sc_memh, pin * 4, reg);
	return NULL;
}

void
amdgpio_intr_disable(void *cookie, void *pin)
{
	struct amdgpio_softc *sc = cookie;
	uint32_t reg;
	int *pin_aux = pin;

	KASSERT(*pin_aux >= 0 && *pin_aux != 63 && *pin_aux < sc->sc_npins);

	reg = bus_space_read_4(sc->sc_memt, sc->sc_memh, *pin_aux * 4);
	reg &= ~(AMDGPIO_CONF_INT_MASK | AMDGPIO_CONF_INT_EN);
	bus_space_write_4(sc->sc_memt, sc->sc_memh, (int) *pin_aux * 4, reg);
}

int
amdgpio_pin_intr(struct amdgpio_softc *sc, int pin)
{
	uint32_t reg;
	int rc = 0;

	reg = bus_space_read_4(sc->sc_memt, sc->sc_memh, pin * 4);
	if (reg & AMDGPIO_CONF_INT_STS) {
		if (sc->sc_pin_ih[pin].ih_func) {
			
sc->sc_pin_ih[pin].ih_func(sc->sc_pin_ih[pin].ih_arg);

			/* Clear interrupt */
			reg = bus_space_read_4(sc->sc_memt, 
sc->sc_memh,
			    pin * 4);
			bus_space_write_4(sc->sc_memt, sc->sc_memh,
			    pin * 4, reg);
			rc = 1;
		} else {
			/* Mask unhandled interrupt */
			reg &= ~(AMDGPIO_CONF_INT_MASK | 
AMDGPIO_CONF_INT_EN);
			bus_space_write_4(sc->sc_memt, sc->sc_memh,
			    pin * 4, reg);
		}
	}

	return rc;
}

int
amdgpio_intr(void *arg)
{
	struct amdgpio_softc *sc = arg;
	uint64_t status;
	uint32_t reg;
	int rc = 0, pin = 0;
	int i, j;

	status = bus_space_read_4(sc->sc_memt, sc->sc_memh,
	    AMDGPIO_IRQ_STS + 4);
	status <<= 32;
	status |= bus_space_read_4(sc->sc_memt, sc->sc_memh,
	    AMDGPIO_IRQ_STS);

	/* One status bit for every four pins */
	for (i = 0; i < AMDGPIO_IRQ_BITS; i++, pin += 4) {
		if (status & (1ULL << i)) {
			for (j = 0; j < AMDGPIO_IRQ_PINS; j++) {
				if (amdgpio_pin_intr(sc, pin + j))
					rc = 1;
			}
		}
	}

	/* Signal end of interrupt */
	reg = bus_space_read_4(sc->sc_memt, sc->sc_memh,
	    AMDGPIO_IRQ_MASTER);
	reg |= AMDGPIO_IRQ_MASTER_EOI;
	bus_space_write_4(sc->sc_memt, sc->sc_memh,
	    AMDGPIO_IRQ_MASTER, reg);

	return rc;
}
