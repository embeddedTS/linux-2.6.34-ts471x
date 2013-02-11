/*
 *  linux/arch/arm/mach-mmp/edge.c
 *
 *  Support for the Marvell PXA168-based Edge 2.0 Development Platform.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/pwm.h>
#include <linux/pwm_backlight.h>
#include <linux/spi/spi.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/android_pmem.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/addr-map.h>
#include <mach/mfp-pxa168.h>
#include <mach/pxa168.h>
#include <mach/gpio.h>
#include <linux/gpio_keys.h>
#include <mach/irqs.h>
#include <plat/pxa3xx_pmic.h>
#include <plat/pxa_u2o.h>

#include <mach/pxa3xx_nand.h>
#include <plat/part_table.h>
#include <plat/generic.h>
#include <mach/mmc.h>
#include <mach/camera.h>
#include <mach/pxa168fb.h>
#include <mach/power_button.h>

#include "common.h"
#include <linux/delay.h>
#ifdef CONFIG_SD8XXX_RFKILL
#include <linux/sd8x_rfkill.h>
#endif
#include <linux/mmc/sdhci.h>

/* Edge 2.0 MFP configurations */
static unsigned long edge_pin_config[] __initdata = {
	/* DEBUG UART2 */
	MFP_CFG(GPIO88, AF3),
	MFP_CFG(GPIO89, AF3),

	/* MMC2_DET/WP */
	MFP_CFG(GPIO86, AF0),
	MFP_CFG(GPIO87, AF0),

	/* MMC2 DAT3-0/CLK/CMD */
	MFP_CFG(GPIO90, AF1),
	MFP_CFG(GPIO91,	AF1),
	MFP_CFG(GPIO92,	AF1),
	MFP_CFG(GPIO93,	AF1),
	MFP_CFG(GPIO94,	AF1),
	MFP_CFG(GPIO95,	AF1),

	/* SD_PWR_EN */
	MFP_CFG(GPIO105, AF0),

	/* JBALL, JBALL_DOWN - EXT_WAKEUP */
	MFP_CFG(GPIO96, AF0),  /* JBALL_PRESS */
	MFP_CFG(GPIO25, AF5),  /* JBALL_LEFT */
	MFP_CFG(GPIO104, AF0), /* JBALL_RIGHT */
	MFP_CFG(GPIO112, AF0), /* JBALL_UP */

	/* PEN DIGITIZER */
	MFP_CFG(GPIO97, AF0), /* PEN_DETECT ???*/
	MFP_CFG(GPIO100, AF0), /* PEN_SLEEP */
	MFP_CFG(GPIO109, AF0), /* PEN_RESETn */
	MFP_CFG_DRV(GPIO98, AF2, FAST), /* PEN_RXD, UART3_TXD */
	MFP_CFG_DRV(GPIO99, AF2, FAST), /* PEN_TXD, UART3_RXD */
	MFP_CFG_DRV(GPIO101, AF2, FAST), /* PEN_CTS, UART3_RTS */

	/* I2C */
	MFP_CFG_DRV(GPIO102, AF1, SLOW),
	MFP_CFG_DRV(GPIO103, AF1, SLOW),

	/* 3G Mini_PCIe HUAWEI EM660 Module */
	MFP_CFG(GPIO106, AF0), /* PERST# */
	MFP_CFG(GPIO43, AF0),  /* W_DISABLE# */

	/* SW_LCDn[0:3] */
	GPIO37_KP_DKIN0,
	GPIO38_KP_DKIN1,
	GPIO39_KP_DKIN2,
	GPIO42_KP_DKIN3,

	/* SW_VOL */
	MFP_CFG(GPIO40, AF0),	/* SW_VOL_UP# */
	MFP_CFG(GPIO41, AF0),	/* SW_VOL_DOWN# */

	/* SW_EPDn[0:3] */
	MFP_CFG(GPIO44, AF0),
	MFP_CFG(GPIO45, AF0),
	MFP_CFG(GPIO47, AF0),
	MFP_CFG(GPIO46, AF0),

	/* HALL sensor */
	MFP_CFG(GPIO48, AF0), /* UNIT_OPEN */

	/* EC power control */
	MFP_CFG(GPIO49, AF0), /* SD_REQ# */
	MFP_CFG(GPIO51, AF0), /* SD_GNT# */
	MFP_CFG(GPIO52, AF0), /* SLP_REQ# */
	MFP_CFG(GPIO53, AF0), /* STR_GNT# */
	MFP_CFG(GPIO54, AF0), /* VCORE_INT, gpio47 on ava ??? */

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

	MFP_CFG(GPIO34, AF0), /* LCD_PWR_EN */
	MFP_CFG(GPIO35, AF0), /* LCD_PWDN# */

	MFP_CFG(GPIO78, AF0), /* EPSON_RESET# */
	MFP_CFG(GPIO79, AF0), /* EPSON_IRQ */
	MFP_CFG(GPIO80, AF0), /* EPSON_DC, ??? GPIO22 */
	MFP_CFG(GPIO81, AF0), /* EPSON_RDY */

	/* USB host port power control */
	MFP_CFG(GPIO122, AF0), /* USBH_RST# */
	MFP_CFG(GPIO20, AF0), /* USB_CAM_EN# */
	MFP_CFG(GPIO82, AF0), /* USB_tp_P_EN# USB customer 2 */
	MFP_CFG(GPIO83, AF0), /* USB_kb_P_EN# USB customer 1 */

	/* backlight control */
	MFP_CFG(GPIO84, AF4), /* LCD_PWM */
	MFP_CFG(GPIO85, AF0), /* LCD_BL_PWR_EN */

	/* FLASH */
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
	GPIO16_ND_nCS0, /* MLC CS0# */
	GPIO26_ND_RnB1, /* MLC_NF_R_B0# */
	GPIO21_ND_ALE,  /* MLC NF_WE# */
	/* shared with EPSON panel */
	GPIO24_ND_nRE, /* AP_NF_RE#, EPSON_RDn */
	GPIO17_ND_nWE, /* AP_NF_WE#, EPSON_WDn */
	GPIO22_ND_CLE, /* AP_NF_CLE#, EPSON_DC ??? GPIO80 */

	/* EPSON panel */
	GPIO18_SMC_nCS1, /* EPSON_CS# */

	/* tsc2007 irq */
	MFP_CFG(GPIO19, AF5),

	/* RTC */
	MFP_CFG(GPIO23, AF5), /* RTC_INT# */

	/* not used, GPIO27_ND_RnB2 */
	MFP_CFG(GPIO27, AF1),

	/* SD_WIFI */
	MFP_CFG(GPIO28, AF1),
	MFP_CFG(GPIO29, AF1),
	MFP_CFG(GPIO30, AF1),
	MFP_CFG(GPIO31, AF1),
	MFP_CFG(GPIO32, AF1),
	MFP_CFG(GPIO33, AF1),

	/* WIFI */
	MFP_CFG(GPIO50, AF0),	/* SW_WIFI_ON */ /* could be used as gpio key tmply */
	MFP_CFG(GPIO55, AF0),	/* GP_PD_WLAN# */
	MFP_CFG(GPIO118, AF0),	/* AP_WPD_1# */
	MFP_CFG(GPIO119, AF0),	/* AP_WPD_2# */
	MFP_CFG(GPIO120, AF0),	/* RST_WIFI# */
	MFP_CFG(GPIO121, AF0),	/* WIFI_LED# */

	/* G-sensor BMA020 */
	MFP_CFG(GPIO36, AF0), /* G_INT */

	/* SSP2, SPI NOR */
	GPIO107_SPI_NOR_RXD,
	GPIO108_SPI_NOR_TXD,
	GPIO110_GPIO,
	GPIO111_SPI_NOR_CLK,

	/* SSP0, AUDIO */
	MFP_CFG(GPIO113, AF6),
	MFP_CFG(GPIO114, AF1),
	MFP_CFG(GPIO115, AF1),
	MFP_CFG(GPIO116, AF2),
	MFP_CFG(GPIO117, AF2),
};

/*
 * GPIO Keys
 */
static struct gpio_keys_button btn_button_table[] = {
	{
		.code			=	KEY_F4,
		.gpio			=	MFP_PIN_GPIO40,  /* SW1, SW_VOL_UP# */
		.active_low		=	1,		//0 for down 0, up 1; 1 for down 1, up 0
		.desc			=	"VUP button",
		.type			=	EV_KEY,
		//.wakeup			=
		.debounce_interval	=	10,		//10 msec jitter elimination
	},
	{
		.code			=	KEY_F5,
		.gpio			=	MFP_PIN_GPIO41, /* SW2, SW_VOL_DOWN# */
		.active_low		=	1,		//0 for down 0, up 1; 1 for down 1, up 0
		.desc			=	"VDN button",
		.type			=	EV_KEY,
		//.wakeup			=
		.debounce_interval	=	10,		//10 msec jitter elimination
	},
};

static struct gpio_keys_platform_data gpio_keys_data = {
	.buttons  = btn_button_table,
	.nbuttons = ARRAY_SIZE(btn_button_table),
};

static struct platform_device gpio_keys = {
	.name = "gpio-keys",
	.dev  = {
		.platform_data = &gpio_keys_data,
	},
	.id   = -1,
};

static void __init edge_gpio_keys_init(void)
{
	platform_device_register(&gpio_keys);
}

static struct fb_videomode video_modes[] = {
	/* innolux WVGA mode info */
	[0] = {
		.pixclock	= 22222,
		.refresh	= 60,
		.xres		= 1024,
		.yres		= 600,
		.hsync_len	= 176,
		.left_margin	= 0,
		.right_margin	= 0,
		.vsync_len	= 25,
		.upper_margin	= 0,
		.lower_margin	= 0,
		.sync		= FB_SYNC_VERT_HIGH_ACT|FB_SYNC_HOR_HIGH_ACT,
	},
};

static struct pxa168fb_mach_info edge_lcd_info __initdata = {
	.id			= "Base",
	.modes			= video_modes,
	.num_modes		= ARRAY_SIZE(video_modes),
	.pix_fmt		= PIX_FMT_RGB565,
	.io_pin_allocation_mode	= PIN_MODE_DUMB_18_GPIO,
	.dumb_mode		= DUMB_MODE_RGB666,
	.active			= 1,
	.panel_rbswap		= 0,
	.invert_pixclock	= 1,
	.max_fb_size		= (DEFAULT_FB_SIZE + 5 * 1024 * 1024),
};

struct pxa168fb_mach_info edge_lcd_ovly_info __initdata = {
        .id                     = "Ovly",
        .modes                  = video_modes,
        .num_modes              = ARRAY_SIZE(video_modes),
        .pix_fmt                = PIX_FMT_RGB565,
        .io_pin_allocation_mode = PIN_MODE_DUMB_18_GPIO,
        .dumb_mode              = DUMB_MODE_RGB666,
        .active                 = 1,
	.panel_rbswap		= 0,
	.max_fb_size		= 1024 * 768 *4,
};

static int __init edge_lcd_init(void)
{
	int lcd_pwdn, lcd_pwr_en;

	/* LCD_PWR_EN */
	lcd_pwr_en = MFP_PIN_GPIO34;
	if(gpio_request(lcd_pwr_en, "lcd_pwr_en")) {
		printk(KERN_INFO "gpio %d request failed\n", lcd_pwr_en);
		return -EIO;
	}
	gpio_direction_output(lcd_pwr_en, 1);
	gpio_free(lcd_pwr_en);

	/* LCD_PWDN# */
	lcd_pwdn = MFP_PIN_GPIO35;
	if(gpio_request(lcd_pwdn, "lcd_pwdn")) {
		printk(KERN_INFO "gpio %d request failed\n", lcd_pwdn);
		return -EIO;
	}
	gpio_direction_output(lcd_pwdn, 1);
	gpio_free(lcd_pwdn);

	/* register device */
	pxa168_add_fb(&edge_lcd_info);
	pxa168_add_fb_ovly(&edge_lcd_ovly_info);

	return 0;
}

static struct platform_pwm_backlight_data edge_backlight_data = {
	.pwm_id         = 0,
	.max_brightness = 255,
	.dft_brightness = 200,
	.pwm_period_ns  = 78770,
};

static struct platform_device edge_backlight_device = {
	.name           = "pwm-backlight",
	.dev            = {
		.parent = &pxa168_device_pwm0.dev,
		.platform_data = &edge_backlight_data,
	},
};

static int __init edge_backlight_register(void)
{
	int lcd_bl_pwr_en;
	int ret = platform_device_register(&edge_backlight_device);
	if (ret)
		printk(KERN_ERR "edge: failed to register backlight device: %d\n", ret);

	/* LCD_BL_PWR_EN */
	lcd_bl_pwr_en = MFP_PIN_GPIO85;
	if(gpio_request(lcd_bl_pwr_en, "lcd_bl_pwr_en")) {
		printk(KERN_INFO "gpio %d request failed\n", lcd_bl_pwr_en);
		return -EIO;
	}
	gpio_direction_output(lcd_bl_pwr_en, 1);
	gpio_free(lcd_bl_pwr_en);

	return 0;
}

static struct i2c_pxa_platform_data pwri2c_info __initdata = {
	.use_pio		= 1,
};

/* touch screen, rtc, audio codec  */
static struct i2c_board_info edge_i2c_board_info[] = {
#if defined(CONFIG_TSC2007)
       {
	       .type	= "tsc2007",
	       .addr	= 0x48,			/* 0x90/0x91 */
	       .irq	= IRQ_GPIO(mfp_to_gpio(MFP_PIN_GPIO19)),
       },
#endif
#if defined(CONFIG_RTC_DRV_DS1307)
	{
		.type		= "ds1337",
		.addr           = 0x68, /* 0xD0 */
	},
#endif
};

static struct i2c_board_info pwri2c_board_info[] =
{
#if defined(CONFIG_BMA020)
	{
		.type	="bma020",
		.addr	=0x38,
		.irq	=IRQ_GPIO(mfp_to_gpio(MFP_PIN_GPIO36)), /* G_INT */
	},
#endif
#if defined(CONFIG_MCU_PM)
	{
		.type	="power_mcu",
		.addr	=0x2C, /* why not 0x3c as it's 0x78 in schematic ??? */
	},
#endif
};

static struct pxa27x_keypad_platform_data edge_android_keypad_info \
						  __initdata = {

	.direct_key_map = { KEY_F9, KEY_F6, KEY_F7, KEY_F8 },
	.direct_key_num = 4,
	.debounce_interval	= 30,
};

#if defined(CONFIG_MMC_PXA_SDH)
static int edge_sdh_init(struct device *dev,
		     irq_handler_t detect_int, void *data)
{
	int sd_pwr_en;
	int ap_wpd_1n, ap_wpd_2n, rst_wifi;
	int err, cd_irq, gpio_cd;

	/* SD_PWR_EN */
	sd_pwr_en = MFP_PIN_GPIO105;
	if(gpio_request(sd_pwr_en, "sd_pwr_en")) {
		printk(KERN_INFO "gpio %d request failed\n", sd_pwr_en);
		return -EIO;
	}
	gpio_direction_output(sd_pwr_en, 1);/* should fix to 0 for 1.5Q */
	gpio_free(sd_pwr_en);

	/* V3P3_WLAN - AP_WPD_1#; V1P8_WLAN - AP_WPD_2# */
	ap_wpd_1n = mfp_to_gpio(MFP_PIN_GPIO118);
	ap_wpd_2n = mfp_to_gpio(MFP_PIN_GPIO119);

	if(gpio_request(ap_wpd_1n, "ap_wpd_1n")) {
		printk(KERN_INFO "gpio %d request failed\n", ap_wpd_1n);
		return -EIO;
	}
	if(gpio_request(ap_wpd_2n, "ap_wpd_2n")) {
		printk(KERN_INFO "gpio %d request failed\n", ap_wpd_2n);
		return -EIO;
	}

	gpio_direction_output(ap_wpd_1n, 1);
	gpio_direction_output(ap_wpd_2n, 1);
	gpio_free(ap_wpd_1n);
	gpio_free(ap_wpd_2n);

	/* RST_WIFI# to high */
	rst_wifi = mfp_to_gpio(MFP_PIN_GPIO120);
	if(gpio_request(rst_wifi, "rst_wifi")) {
		printk(KERN_INFO "gpio %d request failed\n", rst_wifi);
		return -EIO;
	}
	gpio_direction_output(rst_wifi, 1);
	gpio_free(rst_wifi);

	/* MMC2_DET */
	gpio_cd = mfp_to_gpio(MFP_PIN_GPIO86);
	cd_irq = gpio_to_irq(gpio_cd);

	/*
	 * setup GPIO for MMC controller
	 */
	err = gpio_request(gpio_cd, "mmc card detect");
	if (err)
		goto err_request_cd;
	gpio_direction_input(gpio_cd);

	err = request_irq(cd_irq, detect_int,
			  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			  "MMC card detect", data);
	if (err) {
		printk(KERN_ERR "%s: MMC/SD/SDIO: "
				"can't request card detect IRQ\n", __func__);
		goto err_request_irq;
	}

	return 0;

err_request_irq:
	gpio_free(gpio_cd);
err_request_cd:
	return err;
}

static struct pxasdh_platform_data edge_sdh_platform_data = {
	.detect_delay	= 20,
	.ocr_mask	= MMC_VDD_32_33 | MMC_VDD_33_34,
	.init		= edge_sdh_init,
	.quirks		= SDHCI_QUIRK_NO_BUSY_IRQ,
};

static struct pxasdh_platform_data edge_sdh3_platform_data = {
	.detect_delay	= 20,
	.ocr_mask	= MMC_VDD_32_33 | MMC_VDD_33_34,
};

static void __init edge_init_mmc(void)
{
	/* mmc2, mmc/sd socket */
	pxa168_add_sdh(1, &edge_sdh_platform_data);

	/* mmc4, wifi */
#ifdef CONFIG_SD8XXX_RFKILL
	add_sd8x_rfkill_device(mfp_to_gpio(MFP_PIN_GPIO55), 0,
			&edge_sdh3_platform_data.pmmc);
#endif
	pxa168_add_sdh(3, &edge_sdh3_platform_data);
}
#endif

#ifdef CONFIG_USB_GADGET_PXA_U2O
static struct pxa_usb_plat_info edge_u2o_info = {
	.phy_init	= pxa168_usb_phy_init,
};
#endif

#ifdef CONFIG_USB_EHCI_PXA_U2H
static int edge_u2h_plat_init (struct device *dev)
{
	int usbh_rst = mfp_to_gpio(MFP_PIN_GPIO122);

	/* USBH_RST# */
	if(gpio_request(usbh_rst, "usbh_rst")) {
		printk(KERN_INFO "gpio %d request failed\n", usbh_rst);
		return -EIO;
	}
	gpio_direction_output(usbh_rst, 1);
	gpio_free(usbh_rst);
	return 0;
}

static int edge_u2h_vbus_set (int enable)
{
	int usb_cam_en = mfp_to_gpio(MFP_PIN_GPIO20);
	int usb_tp_p_en = mfp_to_gpio(MFP_PIN_GPIO82);
	int usb_kb_p_en = mfp_to_gpio(MFP_PIN_GPIO83);

	/* USB_CAM_EN# */
	if(gpio_request(usb_cam_en, "usb_cam_en")) {
		printk(KERN_INFO "gpio %d request failed\n", usb_cam_en);
		return -EIO;
	}
	gpio_direction_output(usb_cam_en, 0);
	gpio_free(usb_cam_en);

	/* USB_tp_P_EN# */
	if(gpio_request(usb_tp_p_en, "usb_tp_p_en")) {
		printk(KERN_INFO "gpio %d request failed\n", usb_tp_p_en);
		return -EIO;
	}
	gpio_direction_output(usb_tp_p_en, 0);
	gpio_free(usb_tp_p_en);

	/* USB_kb_P_EN# */
	if(gpio_request(usb_kb_p_en, "usb_kb_p_en")) {
		printk(KERN_INFO "gpio %d request failed\n", usb_kb_p_en);
		return -EIO;
	}
	gpio_direction_output(usb_kb_p_en, 0);
	gpio_free(usb_kb_p_en);

	return 0;
}

static struct pxa_usb_plat_info edge_u2h_info = {
	.phy_init	= pxa168_usb_phy_init,
	.vbus_set	= edge_u2h_vbus_set,
	.plat_init	= edge_u2h_plat_init,
};
#endif

#if defined(CONFIG_MTD_M25P80)
static struct pxa2xx_spi_master pxa_ssp_master_info = {
	.num_chipselect	= 1,
};

static struct pxa2xx_spi_chip m25pxx_spi_info = {
	.tx_threshold = 1,
	.rx_threshold = 1,
	.timeout = 1000,
	.gpio_cs = 110
};

static struct spi_board_info __initdata spi_board_info[] = {
	{
		.modalias = "m25p80",
		.mode = SPI_MODE_0,
		.max_speed_hz = 260000,
		.bus_num = 2,
		.chip_select = 0,
		.platform_data = NULL,
		.controller_data = &m25pxx_spi_info,
		.irq = -1,
	},
};

static void __init edge_init_spi(void)
{
	pxa168_add_ssp(1);
	pxa168_add_spi(2, &pxa_ssp_master_info);
	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
}
#else
static inline void edge_init_spi(void) {}
#endif

struct platform_device edge_power_mcu = {
	.name		= "power_mcu",
};

struct platform_device edge_power_supply = {
	.name		= "battery",
};

static void ack_standby(void)
{
	gpio_direction_output(MFP_PIN_GPIO53, 1);
	mdelay(100);
	gpio_direction_output(MFP_PIN_GPIO53, 0);
}

static void ack_powerdwn(void)
{
	gpio_direction_output(MFP_PIN_GPIO51, 1);
	mdelay(100);
	gpio_direction_output(MFP_PIN_GPIO51, 0);
}

static int power_button_init(irq_handler_t pwrdwn_handler, irq_handler_t standby_handler)
{
	int ret;
	int irq;

	irq = gpio_to_irq(MFP_PIN_GPIO49);
	ret = request_irq(irq , pwrdwn_handler, \
				IRQF_SAMPLE_RANDOM | IRQF_TRIGGER_RISING , \
				"shutdown detect", NULL);
	if (ret) {
		printk(KERN_ERR "%s: can't request detect standby irq\n",
				__FUNCTION__);
		goto out;
	}

	irq = gpio_to_irq(MFP_PIN_GPIO52);
	ret = request_irq(irq, standby_handler , \
				IRQF_SAMPLE_RANDOM | IRQF_TRIGGER_RISING , \
				"standby detect", NULL);
	if (ret) {
		printk(KERN_ERR "%s: can't request detect shutdown irq\n",
				__FUNCTION__);
		goto out;
	}

	gpio_direction_input(MFP_PIN_GPIO49);
	gpio_direction_input(MFP_PIN_GPIO52);

out:
	return ret;
}

static struct power_button_platform_data power_button_data = {
	.init = power_button_init,
	.send_standby_ack = ack_standby,
	.send_powerdwn_ack = ack_powerdwn,
};

struct platform_device edge_power_button = {
	.name		= "power-button",
	.dev	= {
		.platform_data = &power_button_data,
	},
	.id = -1,
};



static struct platform_device *devices[] __initdata = {
	&pxa168_device_pwm0,
	&edge_power_mcu,
	&edge_power_supply,
	&edge_power_button,
};

DECLARE_AVENGERSLITE_MLC_4G_ANDROID_PARTITIONS(edge_mlc_4g_android_partitions);
DECLARE_AVENGERSLITE_MLC_1G_ANDROID_PARTITIONS(edge_mlc_1g_android_partitions);
DECLARE_AVENGERSLITE_MLC_4G_MAEMO_PARTITIONS(edge_mlc_4g_maemo_partitions);
DECLARE_AVENGERSLITE_MLC_1G_MAEMO_PARTITIONS(edge_mlc_1g_maemo_partitions);
static struct pxa3xx_nand_platform_data edge_nand_info;
static void __init edge_add_nand(void)
{
    unsigned long long mlc_size = 0;

        mlc_size = get_mlc_size();
        printk (KERN_INFO "@%s Get mlc size from CMDLINE:%d\n", __FUNCTION__, (int)mlc_size);

	if (is_android()) {
            switch (mlc_size)
            {
                case 1:  /* 1G */
                    edge_nand_info.parts[0] = edge_mlc_1g_android_partitions;
                    edge_nand_info.nr_parts[0] = ARRAY_SIZE(edge_mlc_1g_android_partitions);
                    break;
                case 4:  /* 4G */
                    edge_nand_info.parts[0] = edge_mlc_4g_android_partitions;
                    edge_nand_info.nr_parts[0] = ARRAY_SIZE(edge_mlc_4g_android_partitions);
                    break;
                default:
                    /* FIXME: do something or not */
                    edge_nand_info.parts[0] = edge_mlc_4g_android_partitions;
                    edge_nand_info.nr_parts[0] = ARRAY_SIZE(edge_mlc_4g_android_partitions);
                    break;
            }
	} else {
	        switch (mlc_size)
	        {
	            case 1:  /* 1G */
	                edge_nand_info.parts[0] = edge_mlc_1g_maemo_partitions;
	                edge_nand_info.nr_parts[0] = ARRAY_SIZE(edge_mlc_1g_maemo_partitions);
	                break;
	            case 4:  /* 4G */
	                edge_nand_info.parts[0] = edge_mlc_4g_maemo_partitions;
	                edge_nand_info.nr_parts[0] = ARRAY_SIZE(edge_mlc_4g_maemo_partitions);
	                break;
	            default:
	                /* FIXME: do something or not */
	                edge_nand_info.parts[0] = edge_mlc_4g_maemo_partitions;
	                edge_nand_info.nr_parts[0] = ARRAY_SIZE(edge_mlc_4g_maemo_partitions);
	                break;
	        }
			/* make mlc whole as a massstorage room */
		}

	edge_nand_info.use_dma = 1;
	edge_nand_info.RD_CNT_DEL = 0;
	edge_nand_info.enable_arbiter = 1;
	pxa168_add_nand((struct flash_platform_data *) &edge_nand_info);
}

static void gpio_ec_init(void)
{
	/* ??? no uart lines connected, how to control battery */
}

static void edge_power_off(void)
{
	pr_notice("notify EC to shutdown\n");
	gpio_direction_output(MFP_PIN_GPIO51, 1);
	gpio_direction_output(MFP_PIN_GPIO53, 1);
	mdelay(100);
	gpio_direction_output(MFP_PIN_GPIO51, 0);
	gpio_direction_output(MFP_PIN_GPIO53, 0);

	/* Spin to death... */
	while (1);
}

static void __init edge_init(void)
{
	pxa168_set_vdd_iox(VDD_IO0, VDD_IO_3P3V);
	pxa168_set_vdd_iox(VDD_IO1, VDD_IO_3P3V);
	pxa168_set_vdd_iox(VDD_IO2, VDD_IO_3P3V);
	pxa168_set_vdd_iox(VDD_IO3, VDD_IO_3P3V);
	pxa168_set_vdd_iox(VDD_IO4, VDD_IO_3P3V);
	pxa168_mfp_set_fastio_drive(MFP_DS02X);

	mfp_config(ARRAY_AND_SIZE(edge_pin_config));

	/* on-chip devices */
	pxa168_add_uart(2);
	edge_add_nand();
	pxa168_add_ssp(0);
	pxa168_add_twsi(0, &pwri2c_info, ARRAY_AND_SIZE(edge_i2c_board_info));
	pxa168_add_twsi(1, &pwri2c_info, ARRAY_AND_SIZE(pwri2c_board_info));

	pxa168_add_keypad(&edge_android_keypad_info);

#ifdef CONFIG_USB_GADGET_PXA_U2O
	pxa168_add_u2o(&edge_u2o_info);
#endif
#ifdef CONFIG_USB_EHCI_PXA_U2H
	pxa168_add_u2h(&edge_u2h_info);
#endif
#if defined(CONFIG_MMC_PXA_SDH)
	edge_init_mmc();
#endif
	platform_add_devices(devices, ARRAY_SIZE(devices));

	/* off-chip devices */
	edge_lcd_init();
	edge_backlight_register();

	pxa168_add_freq();
	edge_init_spi();

#if defined(CONFIG_KEYBOARD_GPIO)
	edge_gpio_keys_init();
#endif

#ifdef CONFIG_ANDROID_PMEM
	android_add_pmem("pmem", 0x01000000UL, 1, 0);
	android_add_pmem("pmem_adsp", 0x00400000UL, 0, 0);
#endif

	gpio_ec_init();

	pm_power_off = edge_power_off;
}

MACHINE_START(EDGE, "PXA168 Edge Development Platform")
	.phys_io        = APB_PHYS_BASE,
	.boot_params    = 0x00000100,
	.io_pg_offst    = (APB_VIRT_BASE >> 18) & 0xfffc,
	.map_io		= pxa_map_io,
	.init_irq       = pxa168_init_irq,
	.timer          = &pxa168_timer,
	.init_machine   = edge_init,
MACHINE_END
