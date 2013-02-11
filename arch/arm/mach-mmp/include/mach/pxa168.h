#ifndef __ASM_MACH_PXA168_H
#define __ASM_MACH_PXA168_H

#include <mach/devices.h>
#include <mach/pxa168fb.h>
#include <plat/i2c.h>
#include <plat/ssp.h>
#include <plat/pxa27x_keypad.h>
#include <plat/pxa2xx_spi.h>
#include <plat/pxa_u2o.h>
#include <asm/mach/flash.h>
#include <mach/pxa168_eth.h>
#include <mach/pxa168_pcie.h>
#include <mach/mmc.h>
#include <mach/gpio_ir.h>
#include <mach/ov529.h>
#include <linux/card.h>


#define VDD_IO_3P3V	0
#define VDD_IO_1P8V	1
typedef enum {
	VDD_IO0,
	VDD_IO1,
	VDD_IO2,
	VDD_IO3,
	VDD_IO4,
} vdd_io_t;

extern void pxa168_set_vdd_iox(vdd_io_t, int);
extern void pxa168_mfp_set_fastio_drive(int);

static inline int pxa168_add_uart(int id)
{
	struct pxa_device_desc *d = NULL;

	switch (id) {
	case 1: d = &pxa168_device_uart1; break;
	case 2: d = &pxa168_device_uart2; break;
	case 3: d = &pxa168_device_uart3; break;
	/* special case for avengers lite mapping */
	case 4: d = &pxa168_device_uart1b; break;
	default:
		return -EINVAL;
	}

	return pxa_register_device(d, NULL, 0);
}

static inline int pxa168_add_msp(struct card_platform_data *data)
{
	return pxa_register_device(&pxa168_device_msp, data, sizeof(*data));
}

static inline int pxa168_add_cf(void)
{
	return pxa_register_device(&pxa168_device_cf, NULL, 0);
}

static inline int pxa168_add_twsi(int id, struct i2c_pxa_platform_data *data,
				  struct i2c_board_info *info, unsigned size)
{
	struct pxa_device_desc *d = NULL;
	int ret;

	switch (id) {
	case 0: d = &pxa168_device_twsi0; break;
	case 1: d = &pxa168_device_twsi1; break;
	default:
		return -EINVAL;
	}

	ret = i2c_register_board_info(id, info, size);
	if (ret)
		return ret;

	return pxa_register_device(d, data, sizeof(*data));
}

static inline int pxa168_add_fb(struct pxa168fb_mach_info *mi)
{
	return pxa_register_device(&pxa168_device_fb, mi, sizeof(*mi));
}

static inline int pxa168_add_fb_ovly(struct pxa168fb_mach_info *mi)
{
	return pxa_register_device(&pxa168_device_fb_ovly, mi, sizeof(*mi));
}

static inline int pxa168_add_cam(void)
{
	return pxa_register_device(&pxa168_device_camera, NULL, 0);
}

static inline int pxa168_add_ov529(struct ov529_platform_data *pd)
{
	return pxa_register_device(&pxa168_device_ov529, pd, sizeof(*pd));
}

static inline int pxa168_add_ssp(int id)
{
	struct pxa_device_desc *d = NULL;

	switch (id) {
	case 0: d = &pxa168_device_ssp0; break;
	case 1: d = &pxa168_device_ssp1; break;
	case 2: d = &pxa168_device_ssp2; break;
	case 3: d = &pxa168_device_ssp3; break;
	case 4: d = &pxa168_device_ssp4; break;
	default:
		return -EINVAL;
	}

	return pxa_register_device(d, NULL, 0);
}

static inline int pxa168_add_spi(int id, struct pxa2xx_spi_master *pdata)
{
	struct platform_device *pd;

	pd = platform_device_alloc("pxa2xx-spi", id);
	if (pd == NULL) {
		pr_err("pxa2xx-spi: failed to allocate device (id=%d)\n", id);
		return -ENOMEM;
	}

	platform_device_add_data(pd, pdata, sizeof(*pdata));

	return platform_device_add(pd);
}

static inline int pxa168_add_keypad(struct pxa27x_keypad_platform_data *data)
{
	return pxa_register_device(&pxa168_device_keypad, data, sizeof(*data));
}

static inline int pxa168_add_rtc(void *data)
{
	pxa910_device_rtc.dev.platform_data = data;
	return platform_device_register(&pxa910_device_rtc);
}

static inline int pxa168_add_nand(struct flash_platform_data *data)
{
	return pxa_register_device(&pxa168_device_nand, data, sizeof(*data));
}

static inline int pxa168_add_onenand(struct flash_platform_data *data)
{
	return pxa_register_device(&pxa168_device_onenand, data, sizeof(*data));
}

static inline int pxa168_add_u2o(void *data)
{
	pxa168_device_u2o.dev.platform_data = data;
	return platform_device_register(&pxa168_device_u2o);
}

static inline int pxa168_add_u2h(struct pxa_usb_plat_info *info)
{
	pxa168_device_u2h.dev.platform_data = info;
	return platform_device_register(&pxa168_device_u2h);
}

static inline int pxa168_add_u2ootg(struct pxa_usb_plat_info *info)
{
	pxa168_device_u2ootg.dev.platform_data = info;
	return platform_device_register(&pxa168_device_u2ootg);
}

static inline int pxa168_add_u2oehci(struct pxa_usb_plat_info *info)
{
	pxa168_device_u2oehci.dev.platform_data = info;
	return platform_device_register(&pxa168_device_u2oehci);
}

static inline int pxa168_add_mfu(struct pxa168_eth_platform_data *data)
{
#if defined(CONFIG_PXA168_ETH)
	return pxa_register_device(&pxa168_device_mfu, data, sizeof(*data));
#else
	return 0;
#endif
}

static inline int pxa168_add_pcie(struct pxa168_pcie_platform_data *data)
{
	return pxa_register_device(&pxa168_device_pcie, data, sizeof(*data));
}

static inline int pxa168_add_sdh(int id, struct pxasdh_platform_data *data)
{
	struct pxa_device_desc *d = NULL;

	switch (id) {
	case 0: d = &pxa168_device_sdh0; break;
	case 1: d = &pxa168_device_sdh1; break;
	case 2: d = &pxa168_device_sdh2; break;
	case 3: d = &pxa168_device_sdh3; break;
	default:
		return -EINVAL;
	}

	return pxa_register_device(d, data, sizeof(*data));
}

static inline void pxa168_add_freq(void)
{
        int ret;
        ret = platform_device_register(&pxa168_device_freq);
	if (ret)
		dev_err(&pxa168_device_freq.dev,
			"unable to register device: %d\n", ret);
}

static inline void pxa168_cir_init(void)
{
	int ret;
	ret = platform_device_register(&pxa168_device_cir);
	if (ret)
		dev_err(&pxa168_device_cir.dev,
			"unable to register device: %d\n", ret);
}

static inline void pxa168_add_icr(void)
{
	int ret;
	ret = pxa_register_device(&pxa168_device_icr, NULL, 0);
	if (ret)
		pr_err("unable to register ICR device: %d\n", ret);
}

static inline void pxa168_add_cs4344(void)
{
	int ret;
	ret = platform_device_register(&pxa168_device_cs4344);
	if (ret)
		dev_err(&pxa168_device_cs4344.dev,
			"unable to register CS4344 device: %d\n", ret);
}

#endif /* __ASM_MACH_PXA168_H */
