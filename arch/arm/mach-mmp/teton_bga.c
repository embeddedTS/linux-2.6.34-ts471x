/*
 *  linux/arch/arm/mach-mmp/teton_bga.c
 *
 *  Support for the Marvell PXA168-based Teton BGA Platform.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/smc91x.h>
#include <linux/i2c/pca953x.h>
#include <linux/card.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/delay.h>
#include <linux/usb.h>
#include <linux/usb/otg.h>
#include <plat/pxa_u2o.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/addr-map.h>
#include <mach/mfp-pxa168.h>
#include <mach/mfp-teton_bga.h>
#include <mach/pxa168.h>
#include <mach/gpio.h>
#include <mach/max8660.h>
#include <mach/pxa3xx_nand.h>
#include <mach/camera.h>
#include <mach/mfp.h>
#include <mach/pxa168_eth.h>

#include <plat/part_table.h>
#include <plat/generic.h>
#include <plat/pxa3xx_pmic.h>
#include <plat/pxa_u2o.h>
#include <plat/pxa3xx_otg.h>

#include "common.h"
#include <linux/mmc/sdhci.h>
#include <plat/pfn_cfg.h>

static unsigned long teton_bga_pin_config[] __initdata = {

	/* Data Flash Interface Nana */
	GPIO0_DFI_D15,
	GPIO1_DFI_D14,
	GPIO2_DFI_D13,
	GPIO3_DFI_D12,
	GPIO4_DFI_D11,
	GPIO5_DFI_D10,
	GPIO6_DFI_D9,
	GPIO7_DFI_D8,
	GPIO8_DFI_D7,
	GPIO9_DFI_D6,
	GPIO10_DFI_D5,
	GPIO11_DFI_D4,
	GPIO12_DFI_D3,
	GPIO13_DFI_D2,
	GPIO14_DFI_D1,
	GPIO15_DFI_D0,

#if defined(CONFIG_PXA168_CF)
	/* Compact Flash Controller */
	GPIO19_CF_nCE1,
	GPIO20_CF_nCE2,
	GPIO22_ND_CLE,
	GPIO23_CF_nALE,
	GPIO25_CF_nRESET,
	GPIO26_ND_RnB1,
	GPIO27_ND_RnB2,
	GPIO28_CF_RDY,
	GPIO29_CF_STSCH,
	GPIO30_CF_nREG,
	GPIO31_CF_nIOIS16,
#if defined(CONFIG_PXA168_CF_USE_GPIO_CARDDETECT)
	GPIO32_GPIO,	/* CF_nCD1 IRQ */
	GPIO33_GPIO,    /* CF_nCD2 IRQ */
#else
	GPIO32_CF_nCD1,
	GPIO33_CF_nCD2,
#endif
	GPIO34_SMC_nCS1,
	GPIO35_CF_INPACK,
	GPIO36_CF_nWAIT,
#endif
	/* LCD */
	GPIO56_LCD_FCLK_RD,
	GPIO57_LCD_LCLK_A0,
	GPIO58_LCD_PCLK_WR,
	GPIO59_LCD_DENA_BIAS,
	GPIO60_LCD_DD0,
	GPIO61_LCD_DD1,
	GPIO62_LCD_DD2,
	GPIO63_LCD_DD3,
	GPIO64_LCD_DD4,
	GPIO65_LCD_DD5,
	GPIO66_LCD_DD6,
	GPIO67_LCD_DD7,
	GPIO68_LCD_DD8,
	GPIO69_LCD_DD9,
	GPIO70_LCD_DD10,
	GPIO71_LCD_DD11,
	GPIO72_LCD_DD12,
	GPIO73_LCD_DD13,
	GPIO74_LCD_DD14,
	GPIO75_LCD_DD15,
	GPIO76_LCD_DD16,
	GPIO77_LCD_DD17,

	/* i2c bus */
	GPIO105_CI2C_SDA,
	GPIO106_CI2C_SCL,

	/* UART1 */
	GPIO107_UART1_TXD,
	/* CIR and UART1 on TetonBGA are multiplexed */
#if 0 /* disabled CIR due to UART_RX conflict */
	GPIO108_GPIO,
#else
	GPIO108_UART1_RXD,
#endif

	/* Keypad */
	GPIO110_KP_MKIN0,
	GPIO109_KP_MKIN1,
	GPIO111_KP_MKOUT7,
	GPIO112_KP_MKOUT6,

	/* SSP0 */
	GPIO113_I2S_MCLK,
	GPIO114_I2S_FRM,
	GPIO115_I2S_BCLK,
	GPIO116_I2S_TXD,

	/* USB OTG HPENA */
	GPIO85_GPIO,

	/* MFU */
	GPIO86_TX_CLK,
	GPIO87_TX_EN,
	GPIO88_TX_DQ3,
	GPIO89_TX_DQ2,
	GPIO90_TX_DQ1,
	GPIO91_TX_DQ0,
	GPIO92_MII_CRS,
	GPIO93_MII_COL,
	GPIO94_RX_CLK,
	GPIO95_RX_ER,
	GPIO96_RX_DQ3,
	GPIO97_RX_DQ2,
	GPIO98_RX_DQ1,
	GPIO99_RX_DQ0,
	GPIO100_MII_MDC,
	GPIO101_MII_MDIO,
	GPIO103_RX_DV,

	/* mspro detect */
	GPIO47_MSP_INS,

	/* MMC */
	GPIO27_MMC1_EN,
};

#define ENET_RESET_N (104)
#define ENET_COMA_N  (102)

static int pxa168_eth_init(void)
{
	if (gpio_request(ENET_RESET_N, "ENET_RESET_N")) {
		printk(KERN_ERR "Request GPIO failed,"
		       "gpio: %d \n", ENET_RESET_N);
		return -EIO;
	}

	if (gpio_request(ENET_COMA_N, "ENET_COMA_N")) {
		gpio_free(ENET_RESET_N);
		printk(KERN_ERR "Request GPIO failed,"
		       "gpio: %d\n", ENET_COMA_N);
		return -EIO;
	}

	/* reset Ethernet Phy */
	gpio_direction_output(ENET_RESET_N, 0);
	gpio_direction_output(ENET_COMA_N, 1);
	gpio_direction_output(ENET_RESET_N, 1);

	gpio_free(ENET_RESET_N);
	gpio_free(ENET_COMA_N);

	return 0;
}

static struct pxa168_eth_platform_data pxa168_eth_data = {
	.phy_addr = 0,		/* phy addr depends on boards */
	.force_phy_addr = 1,
	.init = pxa168_eth_init,
};

#if defined(CONFIG_PXA168_CF)

#define CF_PWEN (82)
static int __init teton_bga_init_CF(void)
{
	printk("Enabling CF power..\r\n");
	if (gpio_request(CF_PWEN, "CF PWEN")) {
		printk(KERN_ERR "Failed to enable CF Power\n");
		return -1;
	}

	/* set CF power enable (not) pin to low */
	gpio_direction_output(CF_PWEN, 0);
	gpio_free(CF_PWEN);

	return 0;

}

static struct resource pxa168_cf_resources[] = {
	[0] = {
	       .start = 0xD4285000,
	       .end = 0xD4285800,
	       .flags = IORESOURCE_MEM,
	       },

	[1] = {
	       .start = IRQ_PXA168_CF,
	       .end = IRQ_PXA168_CF,
	       .flags = IORESOURCE_IRQ,
	       },
	[2] = {
	       .start = IRQ_GPIO(32),
	       .end = IRQ_GPIO(32),
	       .flags = IORESOURCE_IRQ,
	       },
};

static struct platform_device pxa168_cf_device = {
	.name = "pxa168-cf",
	.id = -1,
	.resource = pxa168_cf_resources,
	.num_resources = ARRAY_SIZE(pxa168_cf_resources),
};

static void __init pxa168_cf_init(void)
{
	platform_device_register(&pxa168_cf_device);
}
#endif

#if defined(CONFIG_PXA168_MSP)
/* msp platform data */
static mfp_cfg_t mfp_cfg_msp[] = {
	GPIO42_MSP_BS,
	GPIO44_MSP_DAT1,
	GPIO45_MSP_DAT0,
	GPIO46_MSP_DAT2,
	GPIO48_MSP_DAT3,
	GPIO50_MSP_SCLK,
};

static void mspro_mfp_config(void)
{
	mfp_config(ARRAY_AND_SIZE(mfp_cfg_msp));
}

static struct card_platform_data msp_ops = {
	/* GPIO47 used as mspro detect pin */
	.pin_detect = MFP_PIN_GPIO47,
	.mfp_config = mspro_mfp_config,
};
#endif

static struct i2c_pxa_platform_data pwri2c_info __initdata = {
	.use_pio = 1,
};

#define LCD_VDD_EN (79)
#define LCD_BKL_EN (80)
static int __init teton_bga_init_BKL(void)
{
	printk(KERN_NOTICE "Enabling LCD Vdd & Backlight..\r\n");
	if (gpio_request(LCD_VDD_EN, "LCD VDD")) {
		printk(KERN_ERR "Failed to enable LCD Power\n");
		return -1;
	}
	if (gpio_request(LCD_BKL_EN, "LCD BKL")) {
		printk(KERN_ERR "Failed to enable backlight\n");
		gpio_free(LCD_VDD_EN);
		return -1;
	}

	/* set VDD and BKL to output and high */
	gpio_direction_output(LCD_VDD_EN, 1);
	msleep(1);
	gpio_direction_output(LCD_BKL_EN, 1);
	gpio_free(LCD_VDD_EN);
	gpio_free(LCD_BKL_EN);

	return 0;

}

static struct fb_videomode video_modes_aspen[] = {
	/* 10.1" WSVGA mode info */
	[0] = {
	       .pixclock = 18422,
	       .refresh = 60,
	       .xres = 1024,
	       .yres = 600,
	       .hsync_len = 381,
	       .left_margin = 0,
	       .right_margin = 0,	/* H_total=1405 */
	       .vsync_len = 50,
	       .upper_margin = 0,
	       .lower_margin = 0,	/* V_total=650 */
	       .sync = 2,
	       },
};

struct pxa168fb_mach_info teton_bga_lcd_info __initdata = {
	.id = "Base-aspen",
	.modes = video_modes_aspen,
	.num_modes = ARRAY_SIZE(video_modes_aspen),
	.pix_fmt = PIX_FMT_RGB565,
	.io_pin_allocation_mode = PIN_MODE_DUMB_18_GPIO,
	.dumb_mode = DUMB_MODE_RGB666,
	.active = 1,
	.spi_ctrl = -1,
	.spi_gpio_cs = -1,
	.spi_gpio_reset = -1,
	.gpio_output_data = 0x10,
	.gpio_output_mask = 0xff,
	.invert_pixclock = 0,
	.invert_vsync = 0,
	.invert_hsync = 0,
	.panel_rbswap = 0,
	.max_fb_size = 1024 * 600 * 4 * 2,
};

struct pxa168fb_mach_info teton_bga_lcd_ovly_info __initdata = {
	.id = "Ovly-aspen",
	.modes = video_modes_aspen,
	.num_modes = ARRAY_SIZE(video_modes_aspen),
	.pix_fmt = PIX_FMT_RGB565,
	.io_pin_allocation_mode = PIN_MODE_DUMB_18_GPIO,
	.dumb_mode = DUMB_MODE_RGB666,
	.active = 1,
	.spi_ctrl = -1,
	.spi_gpio_cs = -1,
	.spi_gpio_reset = -1,
	.gpio_output_data = 0x10,
	.gpio_output_mask = 0xff,
	.invert_pixclock = 0,
	.invert_vsync = 0,
	.invert_hsync = 0,
	.panel_rbswap = 0,
	.max_fb_size = 1024 * 600 * 4 * 2,
};

static struct i2c_board_info teton_bga_i2c_board_info[] = {
#if defined(CONFIG_TSC2007)
	{
	 .type = "tsc2007",
	 .addr = 0x48,		/* 0x90/0x91 */
	 .irq = IRQ_GPIO(86),	/* EXT_WAKEUP pin for interrupt */
	 },
#endif
	{
	 .type = "ds1337",
	 .addr = 0x68,
	 },

};

static unsigned int teton_bga_default_matrix_key_map[] = {
	KEY(0, 6, KEY_UP),	/* S4 */
	KEY(1, 6, KEY_LEFT),	/* S5 */
	KEY(0, 7, KEY_DOWN),	/* S7 */
	KEY(1, 7, KEY_RIGHT),	/* S8 */
};

static unsigned int teton_bga_android_matrix_key_map[] = {
	KEY(0, 6, KEY_BACK),	/* S4 */
	KEY(1, 6, KEY_MENU),	/* S5 */
	KEY(0, 7, KEY_ENTER),	/* S7 */
};

static struct pxa27x_keypad_platform_data teton_bga_default_keypad_info
    __initdata = {
	.matrix_key_rows = 8,
	.matrix_key_cols = 8,
	.matrix_key_map = teton_bga_default_matrix_key_map,
	.matrix_key_map_size = ARRAY_SIZE(teton_bga_default_matrix_key_map),
	.debounce_interval = 30,
};

static struct pxa27x_keypad_platform_data teton_bga_android_keypad_info
    __initdata = {
	.matrix_key_rows = 8,
	.matrix_key_cols = 8,
	.matrix_key_map = teton_bga_android_matrix_key_map,
	.matrix_key_map_size = ARRAY_SIZE(teton_bga_android_matrix_key_map),
	.debounce_interval = 30,
};

DECLARE_ANDROID_512M_V75_PARTITIONS(android_512m_v75_partitions);
DECLARE_512M_V75_PARTITIONS(generic_512m_v75_partitions);
static struct pxa3xx_nand_platform_data teton_bga_nand_info;
static void __init teton_bga_add_nand(void)
{
	if (is_android()) {
		teton_bga_nand_info.parts[0] = android_512m_v75_partitions;
		teton_bga_nand_info.nr_parts[0] =
		    ARRAY_SIZE(android_512m_v75_partitions);
	} else {
		teton_bga_nand_info.parts[0] = generic_512m_v75_partitions;
		teton_bga_nand_info.nr_parts[0] =
		    ARRAY_SIZE(generic_512m_v75_partitions);
	}

	teton_bga_nand_info.use_dma = 1;
	teton_bga_nand_info.enable_arbiter = 1;
	pxa168_add_nand((struct flash_platform_data *)&teton_bga_nand_info);
}

#if defined(CONFIG_MMC_PXA_SDH)
static struct pfn_cfg mmc1_pfn_cfg[] = {
        PFN_CFG(PIN_MMC_DAT7, GPIO37_MMC1_DAT7, GPIO37_GPIO),
        PFN_CFG(PIN_MMC_DAT6, GPIO38_MMC1_DAT6, GPIO38_GPIO),
        PFN_CFG(PIN_MMC_DAT5, GPIO54_MMC1_DAT5, GPIO54_GPIO),
        PFN_CFG(PIN_MMC_DAT4, GPIO48_MMC1_DAT4, GPIO48_GPIO),
        PFN_CFG(PIN_MMC_DAT3, GPIO51_MMC1_DAT3, GPIO51_GPIO),
        PFN_CFG(PIN_MMC_DAT2, GPIO52_MMC1_DAT2, GPIO52_GPIO),
        PFN_CFG(PIN_MMC_DAT1, GPIO40_MMC1_DAT1, GPIO40_GPIO),
        PFN_CFG(PIN_MMC_DAT0, GPIO41_MMC1_DAT0, GPIO41_GPIO),
        PFN_CFG(PIN_MMC_CMD, GPIO49_MMC1_CMD, GPIO49_GPIO),
        PFN_CFG(PIN_MMC_CLK, GPIO43_MMC1_CLK, GPIO43_GPIO),
        PFN_CFG(PIN_MMC_CD, GPIO53_MMC1_CD, GPIO53_GPIO),
        PFN_CFG(PIN_MMC_WP, GPIO46_MMC1_WP, GPIO46_GPIO),
        PFN_CFG(PIN_MMC_END, PFN_TERM, PFN_TERM),
};

static int sdh_mfp_config_mmc1(void)
{
	int ret = 0;
	if (!ret)
		pfn_config(mmc1_pfn_cfg, PFN_FN);
	return ret;
}

static struct pxasdh_platform_data teton_bga_sdh_platform_data_MMC1 = {
	.detect_delay = 20,
	.ocr_mask = MMC_VDD_29_30 | MMC_VDD_30_31,
	.mfp_config = sdh_mfp_config_mmc1,
	.pfn_table = mmc1_pfn_cfg,
};

#define MMC_PWEN (27)
static int __init teton_bga_init_mmc(void)
{
	printk(KERN_NOTICE "Enabling MMC Power..\r\n");
	if (gpio_request(MMC_PWEN, "MMC PWEN")) {
		printk(KERN_ERR "Failed to enable MMC Power\n");
		return -1;
	}

	/* set MMC power enable (not) pin to low */
	gpio_direction_output(MMC_PWEN, 0);
	gpio_free(MMC_PWEN);

	return 0;
}

#if defined(CONFIG_WLAN_8688_SDIO)
static struct pfn_cfg mmc2_pfn_cfg[] = {
	PFN_CFG(PIN_MMC_DAT7, PFN_UNDEF, PFN_UNDEF),
	PFN_CFG(PIN_MMC_DAT6, PFN_UNDEF, PFN_UNDEF),
	PFN_CFG(PIN_MMC_DAT5, PFN_UNDEF, PFN_UNDEF),
	PFN_CFG(PIN_MMC_DAT4, PFN_UNDEF, PFN_UNDEF),
	PFN_CFG(PIN_MMC_DAT3, GPIO122_MMC2_DAT3, GPIO122_GPIO),
	PFN_CFG(PIN_MMC_DAT2, GPIO121_MMC2_DAT2, GPIO121_GPIO),
	PFN_CFG(PIN_MMC_DAT1, GPIO120_MMC2_DAT1, GPIO120_GPIO),
	PFN_CFG(PIN_MMC_DAT0, GPIO119_MMC2_DAT0, GPIO119_GPIO),
	PFN_CFG(PIN_MMC_CMD, GPIO117_MMC2_CMD, GPIO117_GPIO),
	PFN_CFG(PIN_MMC_CLK, GPIO118_MMC2_CLK, GPIO118_GPIO),
	PFN_CFG(PIN_MMC_CD, PFN_UNDEF, PFN_UNDEF),
	PFN_CFG(PIN_MMC_WP, PFN_UNDEF, PFN_UNDEF),
	PFN_CFG(PIN_MMC_END, PFN_TERM, PFN_TERM),
};

#define RST_WIFI (81)
static int __init teton_bga_reset_wifi(void)
{
	printk(KERN_NOTICE "Resetting WIFI..\r\n");
	if (gpio_request(RST_WIFI, "RST WIFI")) {
		printk(KERN_ERR "Failed to Reset WiFi Module\n");
		gpio_free(RST_WIFI);
		return -1;
	}

	gpio_direction_output(RST_WIFI, 0);
	mdelay(500);
	gpio_direction_output(RST_WIFI, 1);
	gpio_free(RST_WIFI);

	return 0;
}

static struct pxasdh_platform_data teton_bga_sdh_platform_data_MMC2 = {
	.detect_delay = 20,
	.ocr_mask = MMC_VDD_29_30 | MMC_VDD_30_31,
	.quirks = SDHCI_QUIRK_BROKEN_CARD_DETECTION,
	.pfn_table = mmc2_pfn_cfg,
};

#endif
#endif

#ifdef CONFIG_USB_GADGET_PXA_U2O
static int teton_bga_u2o_vbus_status(unsigned base)
{
	int status = VBUS_HIGH;

#ifdef CONFIG_USB_OTG
	/* FIXME on teton_bga R0 boards otg stat1/stat2 could not
	 * reflect VBUS status yet, check with U2O itself instead
	 */
	if (u2o_get(base, U2xOTGSC) & U2xOTGSC_BSV)
		status = VBUS_HIGH;
	else
		status = VBUS_LOW;
#endif
	return status;
}

#define USB_HPENA (85)
static int teton_bga_u2o_vbus_set(int vbus_type)
{
	unsigned long flags;

	local_irq_save(flags);

	if (gpio_request(USB_HPENA, "USB OTG Host Power Enable")) {
		printk(KERN_ERR "%s USB_HPENA GPIO Request"
			" Failed\n", __func__);
		return -1;
	}

	switch (vbus_type) {
	case VBUS_SRP:
		gpio_direction_output(USB_HPENA, 1);
		udelay(10);
		gpio_direction_output(USB_HPENA, 0);
		break;
	case VBUS_HIGH:
		gpio_direction_output(USB_HPENA, 1);
		break;
	case VBUS_LOW:
		gpio_direction_output(USB_HPENA, 0);
		break;
	default:
		break;
	}
	gpio_free(USB_HPENA);

	local_irq_restore(flags);

	return 0;
}

static int teton_bga_otg_init(void)
{
	return 0;
}

static int teton_bga_u2o_vbus_set_ic(int function)
{
	printk(KERN_DEBUG "%s %d not implemented yet\n", __func__, function);
	return 0;
}

static struct otg_pmic_ops teton_bga_otg_ops = {
	.otg_vbus_init = teton_bga_otg_init,
	.otg_set_vbus = teton_bga_u2o_vbus_set,
	.otg_set_vbus_ic = teton_bga_u2o_vbus_set_ic,
	.otg_get_vbus_state = teton_bga_u2o_vbus_status,
};

struct otg_pmic_ops *init_teton_bga_otg_ops(void)
{
	return &teton_bga_otg_ops;
}

static struct pxa_usb_plat_info teton_bga_u2o_info = {
	.phy_init = pxa168_usb_phy_init,
	.vbus_set = teton_bga_u2o_vbus_set,
	.vbus_status = teton_bga_u2o_vbus_status,
	.init_pmic_ops = init_teton_bga_otg_ops,
	.is_otg = 1,
};
#endif

#ifdef CONFIG_USB_EHCI_PXA_U2H
/* USB 2.0 Host Controller */
static int teton_bga_u2h_vbus_set(int enable)
{
	return 0;
}

static struct pxa_usb_plat_info teton_bga_u2h_info = {
	.phy_init = pxa168_usb_phy_init,
	.vbus_set = teton_bga_u2h_vbus_set,
};
#endif

static void __init teton_bga_init(void)
{
	pxa168_mfp_set_fastio_drive(MFP_DS02X);
	pxa168_set_vdd_iox(VDD_IO0, VDD_IO_3P3V);
	pxa168_set_vdd_iox(VDD_IO1, VDD_IO_3P3V);
	pxa168_set_vdd_iox(VDD_IO2, VDD_IO_3P3V);
	pxa168_set_vdd_iox(VDD_IO3, VDD_IO_3P3V);
	pxa168_set_vdd_iox(VDD_IO4, VDD_IO_3P3V);
	mfp_config(ARRAY_AND_SIZE(teton_bga_pin_config));

	/* on-chip devices */

	pxa168_add_uart(1);
	teton_bga_add_nand();
	pxa168_add_ssp(0);

	pxa168_add_twsi(0, &pwri2c_info,
			ARRAY_AND_SIZE(teton_bga_i2c_board_info));

	if (is_android())
		pxa168_add_keypad(&teton_bga_android_keypad_info);
	else
		pxa168_add_keypad(&teton_bga_default_keypad_info);

#ifdef CONFIG_USB_GADGET_PXA_U2O
	pxa168_add_u2o(&teton_bga_u2o_info);
#endif

#ifdef CONFIG_USB_OTG
	pxa168_add_u2ootg(&teton_bga_u2o_info);
	pxa168_add_u2oehci(&teton_bga_u2o_info);
#endif

#ifdef CONFIG_USB_EHCI_PXA_U2H
	pxa168_add_u2h(&teton_bga_u2h_info);
#endif
	pxa168_add_mfu(&pxa168_eth_data);

#if defined(CONFIG_MMC_PXA_SDH)
	if (teton_bga_init_mmc() == 0)
		pxa168_add_sdh(0, &teton_bga_sdh_platform_data_MMC1);
#if defined(CONFIG_WLAN_8688_SDIO)
	if (teton_bga_reset_wifi() == 0)
		pxa168_add_sdh(1, &teton_bga_sdh_platform_data_MMC2);
#endif
#endif

#if 0 /* disabled CIR due to UART_RX conflict */
	pxa168_cir_init();	/*init the gpio */
#endif

#if defined(CONFIG_PXA168_MSP)
	pxa168_add_msp(&msp_ops);
#endif

#if defined(CONFIG_PXA168_CF)
	if (teton_bga_init_CF() == 0)
		pxa168_cf_init();
#endif
	pxa168_add_freq();

	if (teton_bga_init_BKL() == 0) {
		pxa168_add_fb(&teton_bga_lcd_info);
		pxa168_add_fb_ovly(&teton_bga_lcd_ovly_info);
	}
#if defined(CONFIG_PXA_ICR)
	pxa168_add_icr();
#endif

#if defined(CONFIG_SND_SOC_CS4344)
	pxa168_add_cs4344();
#endif
}

MACHINE_START(TETON_BGA, "PXA168-based Teton BGA Platform")
	.phys_io = APB_PHYS_BASE,
	.boot_params = 0x00000100,
	.io_pg_offst = (APB_VIRT_BASE >> 18) & 0xfffc,
	.map_io = pxa_map_io,
	.init_irq = pxa168_init_irq,
	.timer = &pxa168_timer,
	.init_machine = teton_bga_init,
MACHINE_END
