/*
 *  linux/arch/arm/mach-mmp/avengers_lite.c
 *
 *  Support for the Marvell PXA168-based Avengers lite Development Platform.
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
#include <linux/spi/cmmb.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/android_pmem.h>
#include <linux/usb/android_composite.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/addr-map.h>
#include <mach/mfp-pxa168.h>
#include <mach/pxa168.h>
#include <mach/gpio.h>
#include <linux/gpio_keys.h>
#include <mach/irqs.h>
#include <mach/power_mcu.h>
#include <mach/rtc.h>
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
#include <plat/pfn_cfg.h>
#include <linux/mmc/sdhci.h>

static unsigned int AVLITE_BOARDID = 0;
unsigned int board_is_1p9(void)
{
#define BOARD_1P9 0x6

	return (BOARD_1P9 == AVLITE_BOARDID);
}
EXPORT_SYMBOL(board_is_1p9);

unsigned int board_is_2p5(void)
{
#define BOARD_2P5 0x4

	return (BOARD_2P5 == AVLITE_BOARDID);
}
EXPORT_SYMBOL(board_is_2p5);

static int sd_debug = 0;
static int __init SD_debug_setup(char *__unused)
{
	sd_debug = 1;
	return 1;
}
__setup("SD_debug", SD_debug_setup);

int SD_debug_enable(void)
{
	return sd_debug;
}
EXPORT_SYMBOL(SD_debug_enable);

/* Avengers lite MFP configurations */
static unsigned long avengers_lite_pin_config_V16F[] __initdata = {
	/***** UART_CAS ******/
	MFP_CFG(GPIO96, AF0),
	MFP_CFG(GPIO97, AF1),
	MFP_CFG(GPIO98, AF2),/* MFP_CFG(GPIO98, AF3), //FN2,3 */
	MFP_CFG(GPIO99, AF0),

	/***** BOARD_ID *****/
	MFP_CFG(GPIO100, AF0),
	MFP_CFG(GPIO101, AF0),
	MFP_CFG(GPIO20, AF0),

	/***** IIC ******/
	MFP_CFG(GPIO102, AF1),
	MFP_CFG(GPIO103, AF1),

	/***** Camera ******/
	MFP_CFG(GPIO37, AF4),
	MFP_CFG(GPIO38, AF4),
	MFP_CFG(GPIO39, AF4),
	MFP_CFG(GPIO40, AF4),
	MFP_CFG(GPIO41, AF4),
	MFP_CFG(GPIO42, AF4),
	MFP_CFG(GPIO44, AF4),
	MFP_CFG(GPIO45, AF4),
	MFP_CFG(GPIO46, AF4),
	MFP_CFG(GPIO48, AF4),
	MFP_CFG(GPIO50, AF4),
	MFP_CFG(GPIO54, AF4),
	MFP_CFG(GPIO55, AF4),

	/***** for avlite ******/
	MFP_CFG(GPIO19, AF5),
	MFP_CFG(GPIO23, AF5),
	MFP_CFG(GPIO25, AF5),

	MFP_CFG(GPIO43, AF0),
	MFP_CFG(GPIO47, AF0),
	MFP_CFG(GPIO49, AF0),
	MFP_CFG(GPIO51, AF0),
	MFP_CFG(GPIO52, AF0),
	MFP_CFG(GPIO53, AF0),

	MFP_CFG(GPIO110, AF0),

	MFP_CFG(GPIO104, AF0),
	MFP_CFG(GPIO105, AF0),
	MFP_CFG(GPIO106, AF0),
	MFP_CFG(GPIO122, AF0),

	/***** fast IO bank *****/
	/***** LCD *****/
	MFP_CFG(GPIO56, AF1),/* fast IO drive strength control */
	MFP_CFG(GPIO57, AF1),/* fast IO drive strength control */
	MFP_CFG(GPIO58, AF1),/* fast IO drive strength control */
	MFP_CFG(GPIO59, AF1),/* fast IO drive strength control   //attention */

	MFP_CFG(GPIO60, AF1),/* voltage control */
				/* VCC_IO1 - 3.3V,VCC_IO0 - 3.3V:DS01X */
				/* VCC_IO1 - 3.3V,VCC_IO0 - 1.8V:DS02X */
				/* VCC_IO1 - 1.8V,VCC_IO0 - 3.3V:DS03X */
				/*VCC_IO1 - 1.8V,VCC_IO0 - 1.8V:DS04X */
	MFP_CFG(GPIO61, AF1),/* voltage control */
				/* VCC_IO3 - 3.3V,VCC_IO2 - 3.3V:DS01X */
				/* VCC_IO3 - 3.3V,VCC_IO2 - 1.8V:DS02X */
				/* VCC_IO3 - 1.8V,VCC_IO2 - 3.3V:DS03X */
				/* VCC_IO3 - 1.8V,VCC_IO2 - 1.8V:DS04X */
	MFP_CFG(GPIO62, AF1),/* voltage control */
				/* VCC_IO4 - 3.3V:DS01X */
				/*VCC_IO4 - 1.8V:DS03X */

	MFP_CFG(GPIO63, AF1),
	MFP_CFG(GPIO64, AF1),
	MFP_CFG(GPIO65, AF1),
	MFP_CFG(GPIO66, AF1),
	MFP_CFG(GPIO67, AF1),
	MFP_CFG(GPIO68, AF1),
	MFP_CFG(GPIO69, AF1),
	MFP_CFG(GPIO70, AF1),
	MFP_CFG(GPIO71, AF1),
	MFP_CFG(GPIO72, AF1),
	MFP_CFG(GPIO73, AF1),
	MFP_CFG(GPIO74, AF1),
	MFP_CFG(GPIO75, AF1),
	MFP_CFG(GPIO76, AF1),
	MFP_CFG(GPIO77, AF1),
	MFP_CFG(GPIO78, AF1),
	MFP_CFG(GPIO79, AF1),
	MFP_CFG(GPIO80, AF1),
	MFP_CFG(GPIO81, AF1),
	MFP_CFG(GPIO82, AF1),
	MFP_CFG(GPIO83, AF1),

	/***** backlight control *****/
	MFP_CFG(GPIO34, AF0),
	MFP_CFG(GPIO35, AF0),
	MFP_CFG(GPIO36, AF0),
	MFP_CFG(GPIO84, AF4),
	MFP_CFG(GPIO85, AF0),

	/***** FLASH *****/
	MFP_CFG(GPIO0, AF5),
	MFP_CFG(GPIO1, AF5),
	MFP_CFG(GPIO2, AF5),
	MFP_CFG(GPIO3, AF5),
	MFP_CFG(GPIO4, AF5),
	MFP_CFG(GPIO5, AF5),
	MFP_CFG(GPIO6, AF5),
	MFP_CFG(GPIO7, AF5),

	MFP_CFG(GPIO10, AF0),
	MFP_CFG(GPIO11, AF0),
	MFP_CFG(GPIO12, AF0),
	MFP_CFG(GPIO13, AF0),
	MFP_CFG(GPIO14, AF0),
	MFP_CFG(GPIO15, AF0),
	MFP_CFG(GPIO16, AF1),
	MFP_CFG(GPIO17, AF0),
	MFP_CFG(GPIO18, AF1),

	MFP_CFG(GPIO21, AF0),
	MFP_CFG(GPIO22, AF0),
	MFP_CFG(GPIO24, AF0),
	MFP_CFG(GPIO26, AF1),
	MFP_CFG(GPIO27, AF1),

	/***** DEBUG_UART *****/
	MFP_CFG(GPIO88, AF2),/* MFP_CFG(GPIO107,AF1), */
	MFP_CFG(GPIO89, AF2),/* MFP_CFG(GPIO108,AF1), */

#if !defined(CONFIG_MTD_M25P80)
	MFP_CFG(GPIO109, AF0),
#else
	MFP_CFG(GPIO109, AF4),
#endif
	MFP_CFG(GPIO111, AF4),
	MFP_CFG(GPIO107, AF4),
	MFP_CFG(GPIO108, AF4),
	MFP_CFG(GPIO112, AF0),

	/***** IIS_AUDIO *****/
	MFP_CFG(GPIO113, AF6),
	MFP_CFG(GPIO114, AF1),
	MFP_CFG(GPIO115, AF1),
	MFP_CFG(GPIO116, AF2),
	MFP_CFG(GPIO117, AF2),

	/***** SSP_CMMB reserved *****/
	MFP_CFG(GPIO118, AF0),
	MFP_CFG(GPIO119, AF0),
	MFP_CFG(GPIO120, AF0),
	MFP_CFG(GPIO121, AF0),

	/***** FREE PIN *****/
	MFP_CFG(GPIO8, AF0),
	MFP_CFG(GPIO9, AF0),
};

static unsigned long avengers_lite_boardid_pin_config[]  = {
	/***** BOARD_ID *****/
	MFP_CFG(GPIO101, AF0),
	MFP_CFG(GPIO100, AF0),
	MFP_CFG(GPIO20, AF0),
};

/*
 * GPIO Keys
 */
static struct gpio_keys_button btn_button_table[] = {
	[0] = {
		.code			=	KEY_F1,
		.gpio			=	MFP_PIN_GPIO4,
		.active_low		=	1,		/* 0 for down 0, up 1; 1 for down 1, up 0 */
		.desc			=	"H_BTN button",
		.type			=	EV_KEY,
		/* .wakeup			= */
		.debounce_interval	=	10,		/* 10 msec jitter elimination */
	},
	[1] = {
		.code			=	KEY_F2,
		.gpio			=	MFP_PIN_GPIO3,
		.active_low		=	1,		/* 0 for down 0, up 1; 1 for down 1, up 0 */
		.desc			=	"O_BTN button",
		.type			=	EV_KEY,
		/* .wakeup			= */
		.debounce_interval	=	10,		/* 10 msec jitter elimination */
	},
	[2] = {
		.code			=	KEY_F3,
		.gpio			=	MFP_PIN_GPIO2,
		.active_low		=	1,		/* 0 for down 0, up 1; 1 for down 1, up 0 */
		.desc			=	"B_BTN button",
		.type			=	EV_KEY,
		/* .wakeup			= */
		.debounce_interval	=	10,		/* 10 msec jitter elimination */
	},
	[3] = {
		.code			=	KEY_F4,
		.gpio			=	MFP_PIN_GPIO5,
		.active_low		=	1,		/* 0 for down 0, up 1; 1 for down 1, up 0 */
		.desc			=	"S_BTN button",
		.type			=	EV_KEY,
		/* .wakeup			= */
		.debounce_interval	=	10,		/* 10 msec jitter elimination */
	},
	[4] = {
		.code			=	KEY_F5,
		.gpio			=	MFP_PIN_GPIO1,
		.active_low		=	1,		/* 0 for down 0, up 1; 1 for down 1, up 0 */
		.desc			=	"VUP button",
		.type			=	EV_KEY,
		/* .wakeup			= */
		.debounce_interval	=	10,		/* 10 msec jitter elimination */
	},
	[5] = {
		.code			=	KEY_F6,
		.gpio			=	MFP_PIN_GPIO0,
		.active_low		=	1,		/* 0 for down 0, up 1; 1 for down 1, up 0 */
		.desc			=	"VDN button",
		.type			=	EV_KEY,
		/* .wakeup			= */
		.debounce_interval	=	10,		/* 10 msec jitter elimination */
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

static void __init avengers_lite_gpio_keys_init(void)
{
	platform_device_register(&gpio_keys);
}

/*
 *
 */

#define LCD_BL_PWR_EN	(board_is_2p5()? MFP_PIN_GPIO25 : MFP_PIN_GPIO85)

static void avengers_lcd_power(struct pxa168fb_info *fbi,
	unsigned int spi_gpio_cs, unsigned int spi_gpio_reset, int on)
{
	int lcd_pwr_en;
	int lcd_bl_pwr_en;
	int lcd_lr, lcd_ud;

	lcd_pwr_en = MFP_PIN_GPIO34;
	if (gpio_request(lcd_pwr_en, "lcd_pwr_en")) {
		printk(KERN_INFO "gpio %d request failed\n",
						lcd_pwr_en);
		goto out;
	}

	lcd_bl_pwr_en = LCD_BL_PWR_EN;
	if (gpio_request(lcd_bl_pwr_en, "lcd_bl_pwr_en")) {
		printk(KERN_INFO "gpio %d request failed\n",
						lcd_bl_pwr_en);
		goto out1;
	}

	lcd_lr = MFP_PIN_GPIO35;
	if (gpio_request(lcd_lr, "lcd_lr")) {
		printk(KERN_INFO "gpio %d request failed\n", lcd_lr);
		goto out2;
	}

	lcd_ud = MFP_PIN_GPIO36;
	if (gpio_request(lcd_ud, "lcd_ud")) {
		printk(KERN_INFO "gpio %d request failed\n", lcd_ud);
		goto out3;
	}

	if (on) {
		/* re-config lcd_lr/ud pin to enable lcd */
		gpio_direction_output(lcd_pwr_en, 1);
		gpio_direction_output(lcd_bl_pwr_en, 1);
		gpio_direction_output(lcd_lr, 1);
		gpio_direction_output(lcd_ud, 0);
	} else {
		/* config lcd_lr/ud pin to gpio for power optimization */
		gpio_direction_output(lcd_pwr_en, 0);
		gpio_direction_output(lcd_bl_pwr_en, 0);
		gpio_direction_input(lcd_lr);
		gpio_direction_input(lcd_ud);
	}

	gpio_free(lcd_ud);
out3:
	gpio_free(lcd_lr);
out2:
	gpio_free(lcd_bl_pwr_en);
out1:
	gpio_free(lcd_pwr_en);
out:
	return;
}

static struct fb_videomode video_modes[] = {
	/* innolux WVGA mode info */
	[0] = {
		.pixclock	= 25000,
		.refresh	= 60,
		.xres		= 800,
		.yres		= 480,
		.hsync_len	= 1,
		.left_margin	= 45,
		.right_margin	= 210,
		.vsync_len	= 1,
		.upper_margin	= 22,
		.lower_margin	= 132,
		.sync		= 0,
	},
};
static struct pxa168fb_mach_info avengers_lite_lcd_info __initdata = {
	.id			= "Base",
	.modes			= video_modes,
	.num_modes		= ARRAY_SIZE(video_modes),
	.pix_fmt		= PIX_FMT_RGB565,
	.io_pin_allocation_mode	= PIN_MODE_DUMB_18_GPIO,
	.dumb_mode		= DUMB_MODE_RGB666,
	.active			= 1,
	.panel_rbswap		= 0,
	.max_fb_size		= (DEFAULT_FB_SIZE + 5 * 1024 * 1024),
	.pxa168fb_lcd_power	= avengers_lcd_power,
};

struct pxa168fb_mach_info avengers_lite_lcd_ovly_info __initdata = {
	.id			= "Ovly",
	.modes			= video_modes,
	.num_modes		= ARRAY_SIZE(video_modes),
	.pix_fmt		= PIX_FMT_RGB565,
	.io_pin_allocation_mode	= PIN_MODE_DUMB_18_GPIO,
	.dumb_mode		= DUMB_MODE_RGB666,
	.active			= 1,
	.panel_rbswap		= 0,
	.max_fb_size		= (DEFAULT_FB_SIZE),
};

static struct pxa168fb_mach_info v1p9_avengers_lite_lcd_info __initdata = {
	.id			= "Base",
	.modes			= video_modes,
	.num_modes		= ARRAY_SIZE(video_modes),
	.pix_fmt		= PIX_FMT_RGB565,
	.io_pin_allocation_mode	= PIN_MODE_DUMB_24,
	.dumb_mode		= DUMB_MODE_RGB888,
	.active			= 1,
	.panel_rbswap		= 0,
	.max_fb_size		= (DEFAULT_FB_SIZE + 5 * 1024 * 1024),
	.pxa168fb_lcd_power	= avengers_lcd_power,
};

#if defined(CONFIG_PXA168_CAMERA) && defined(CONFIG_VIDEO_OV7740)
/* sensor init */
static int sensor_power_onoff(int on, int sensor)
{

	int ov7740_pwr_down;

	ov7740_pwr_down = MFP_PIN_GPIO96;
	if (gpio_request(ov7740_pwr_down , "ov7740_pwr_down")) {
		printk(KERN_INFO "gpio %d request failed\n", ov7740_pwr_down);
		return -EIO;
	}

	if (on) {
		gpio_direction_output(ov7740_pwr_down, 0);
		printk(KERN_INFO"ov7740 power on\n");
	} else {
		gpio_direction_output(ov7740_pwr_down, 1);
		printk(KERN_INFO"ov7740 power off\n");
	}
	gpio_free(ov7740_pwr_down);

	return 0;
}


static struct sensor_platform_data ov7740_sensor_data = {
	.id = SENSOR_LOW,
	.power_on = sensor_power_onoff,
};

#endif
static void __init avengers_lite_lcd_init(void)
{
	int lcd_lr, lcd_ud;

	lcd_lr = MFP_PIN_GPIO35;
	lcd_ud = MFP_PIN_GPIO36;

	if (gpio_request(lcd_lr, "lcd_lr")) {
		printk(KERN_INFO "gpio %d request failed\n", lcd_lr);
		return;
	}
	gpio_direction_output(lcd_lr, 1);
	gpio_free(lcd_lr);

	if (gpio_request(lcd_ud, "lcd_ud")) {
		printk(KERN_INFO "gpio %d request failed\n", lcd_ud);
		return;
	}
	gpio_direction_output(lcd_ud, 0);
	gpio_free(lcd_ud);

	if(board_is_1p9())
		pxa168_add_fb(&v1p9_avengers_lite_lcd_info);
	else
		pxa168_add_fb(&avengers_lite_lcd_info);

	pxa168_add_fb_ovly(&avengers_lite_lcd_ovly_info);
}

static struct platform_pwm_backlight_data avengers_lite_backlight_data = {
	.pwm_id		= 0,
	.max_brightness	= 255,
	.dft_brightness	= 200,
	.pwm_period_ns	= 78770,/* 3921569, */
	/*
	.pwm_id         = 0,
	.max_brightness = 1023,
	.dft_brightness = 1023,
	.pwm_period_ns  = 78770,
	*/
};

static struct platform_device avengers_lite_backlight_device = {
	.name		= "pwm-backlight",
	.dev		= {
		.parent	= &pxa168_device_pwm0.dev,
		.platform_data = &avengers_lite_backlight_data,
	},
};

static void __init avengers_lite_backlight_register(void)
{
	int lcd_pwr_en;
	int lcd_bl_pwr_en;
	int ret = platform_device_register(&avengers_lite_backlight_device);
	if (ret)
		printk(KERN_ERR "avengers lite: failed to register backlight device: %d\n", ret);

	lcd_pwr_en = MFP_PIN_GPIO34;

	if (gpio_request(lcd_pwr_en, "lcd_pwr_en")) {
		printk(KERN_INFO "gpio %d request failed\n", lcd_pwr_en);
		return;
	}
	gpio_direction_output(lcd_pwr_en, 1);
	gpio_free(lcd_pwr_en);

	lcd_bl_pwr_en = LCD_BL_PWR_EN;

	if (gpio_request(lcd_bl_pwr_en, "lcd_bl_pwr_en")) {
		printk(KERN_INFO "gpio %d request failed\n", lcd_bl_pwr_en);
		return;
	}
	gpio_direction_output(lcd_bl_pwr_en, 1);
	gpio_free(lcd_bl_pwr_en);

	return;
}

struct platform_device avengers_lite_power_mcu = {
	.name		= "power_mcu",
};

struct platform_device avengers_lite_power_supply = {
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

struct platform_device avengers_lite_power_button = {
	.name	= "power-button",
	.dev	= {
		.platform_data = &power_button_data,
	},
	.id = -1,
};

struct platform_device avengers_lite_ca = {
	.name		= "pxa168-ca",
};

static struct i2c_pxa_platform_data pwri2c_info __initdata = {
	.use_pio		= 1,
};

static struct i2c_board_info avengers_lite_i2c_board_info[] = {
#if defined(CONFIG_TSC2007)
       {
		.type	= "tsc2007",
		.addr	= 0x48,			/* 0x90/0x91 */
		.irq	= IRQ_GPIO(mfp_to_gpio(MFP_PIN_GPIO19)),
       },
#endif
#if defined(CONFIG_PXA168_CAMERA) && defined(CONFIG_VIDEO_OV7740)
	{
		.type	= "ov7740",
		.addr	= 0x21,
		.platform_data	= &ov7740_sensor_data,
		/* .irq	= */
	},
#endif
};

static struct i2c_board_info pwri2c_board_info[] =
{

#if defined(CONFIG_BMA020)
	{
		.type	= "bma020",
		.addr	= 0x38,
		.irq	= IRQ_GPIO(mfp_to_gpio(MFP_PIN_GPIO109)),
	},
#endif
#if defined(CONFIG_MCU_PM)
	{
		.type	= "power_mcu",
		.addr	= 0x2C,
	},
#endif
};

#if defined(CONFIG_MMC_PXA_SDH)
/* sd/mmc port is connected to the external socket */
static struct pfn_cfg mmc1_pfn_cfg[] = {
	PFN_CFG(PIN_MMC_DAT7, PFN_UNDEF, PFN_UNDEF),
	PFN_CFG(PIN_MMC_DAT6, PFN_UNDEF, PFN_UNDEF),
	PFN_CFG(PIN_MMC_DAT5, PFN_UNDEF, PFN_UNDEF),
	PFN_CFG(PIN_MMC_DAT4, PFN_UNDEF, PFN_UNDEF),
	PFN_CFG(PIN_MMC_DAT3, MFP_CFG(GPIO90, AF1), MFP_CFG(GPIO90, AF0)),
	PFN_CFG(PIN_MMC_DAT2, MFP_CFG(GPIO91, AF1), MFP_CFG(GPIO91, AF0)),
	PFN_CFG(PIN_MMC_DAT1, MFP_CFG(GPIO92, AF1), MFP_CFG(GPIO92, AF0)),
	PFN_CFG(PIN_MMC_DAT0, MFP_CFG(GPIO93, AF1), MFP_CFG(GPIO93, AF0)),
	PFN_CFG(PIN_MMC_CMD, MFP_CFG(GPIO94, AF1), MFP_CFG(GPIO94, AF0)),
	PFN_CFG(PIN_MMC_CLK, MFP_CFG(GPIO95, AF1), MFP_CFG(GPIO95, AF0)),
	PFN_CFG(PIN_MMC_CD, PFN_UNDEF, PFN_UNDEF),
	PFN_CFG(PIN_MMC_WP, PFN_UNDEF, PFN_UNDEF),
	PFN_CFG(PIN_MMC_END, PFN_TERM, PFN_TERM),
};

/* avlite_mmc1_get_cd: return a value indicating card present */
/* return value:                                              */
/*    not 0: card present                                     */
/*        1: card not detected                                */
/* note:                                                      */
/*    on the avlite, mfp_86 is a gpio connected to the        */
/*    external sd/mmc socket's card detect pin.               */
/*    gpio86 will act as nCD:                                 */
/*           gpio86 is low when a card is present             */
/*           gpio86 is high when no card is detected          */
static int avlite_mmc1_get_cd_n(struct device *dev)
{
	unsigned gpio_cd = mfp_to_gpio(MFP_PIN_GPIO86);
	int rc;

	int status = gpio_get_value(gpio_cd);

	rc =  (status == 0) ? 0 : 1;	/* gpio86 low? return card present */

	return rc;
}

static struct pxasdh_platform_data avengers_lite_sdh_platform_data = {
	.detect_delay	= 20,
	.ocr_mask	= MMC_VDD_32_33 | MMC_VDD_33_34,
	.max_speed	= 24000000,
	.init		= pxa_mci_init,
	.exit		= pxa_mci_exit,
	.get_ro		= pxa_mci_ro,
	.pfn_table	= mmc1_pfn_cfg,
	.get_cd		= avlite_mmc1_get_cd_n,
};

/* sd/mmc port is connected to the wifi part: hard-wired, but not always on */
static struct pfn_cfg mmc4_pfn_cfg[] = {
	PFN_CFG(PIN_MMC_DAT7, PFN_UNDEF, PFN_UNDEF),
	PFN_CFG(PIN_MMC_DAT6, PFN_UNDEF, PFN_UNDEF),
	PFN_CFG(PIN_MMC_DAT5, PFN_UNDEF, PFN_UNDEF),
	PFN_CFG(PIN_MMC_DAT4, PFN_UNDEF, PFN_UNDEF),
	PFN_CFG(PIN_MMC_DAT3, MFP_CFG(GPIO33, AF1), MFP_CFG(GPIO33, AF5)),
	PFN_CFG(PIN_MMC_DAT2, MFP_CFG(GPIO32, AF1), MFP_CFG(GPIO32, AF5)),
	PFN_CFG(PIN_MMC_DAT1, MFP_CFG(GPIO31, AF1), MFP_CFG(GPIO31, AF5)),
	PFN_CFG(PIN_MMC_DAT0, MFP_CFG(GPIO30, AF1), MFP_CFG(GPIO30, AF5)),
	PFN_CFG(PIN_MMC_CMD, MFP_CFG(GPIO28, AF1), MFP_CFG(GPIO28, AF5)),
	PFN_CFG(PIN_MMC_CLK, MFP_CFG(GPIO29, AF1), MFP_CFG(GPIO29, AF5)),
	PFN_CFG(PIN_MMC_CD, PFN_UNDEF, PFN_UNDEF),
	PFN_CFG(PIN_MMC_WP, PFN_UNDEF, PFN_UNDEF),
	PFN_CFG(PIN_MMC_END, PFN_TERM, PFN_TERM),
};

/* avlite_mmc4_get_cd: return a value indicating card present */
/* return value:                                              */
/*    not 0: card present                                     */
/*        1: card not detected                                */
/* note:                                                      */
/*    on the avlite, this sd/mmc port is connected to the     */
/*    hard-wired wifi chip. for now, this routine will return */
/*    always present. a future enhancement could be to return */
/*    present only if power is supplied to the part.          */
static int avlite_mmc4_get_cd_n(struct device *dev)
{
	return 0;	/* 0 means card present */
}

static struct pxasdh_platform_data avengers_lite_sdh3_platform_data = {
	.detect_delay	= 20,
	.ocr_mask	= MMC_VDD_32_33 | MMC_VDD_33_34,
	.quirks         = SDHCI_QUIRK_BROKEN_ADMA,
	.pfn_table	= mmc4_pfn_cfg,
	.get_cd		= avlite_mmc4_get_cd_n,
};

static void __init avengers_lite_init_mmc(void)
{
	int uart_to_sd;

	/* switch SD card and SD debug board */
	uart_to_sd = MFP_PIN_GPIO23;
	if (gpio_request(uart_to_sd, "uart_to_sd")) {
		pr_warning("failed to request GPIO for uart_to_sd\n");
		return;
	}
	if (SD_debug_enable())
		gpio_direction_output(uart_to_sd, 1);
	else
		gpio_direction_output(uart_to_sd, 0);

	gpio_free(uart_to_sd);

	/* MMC card detect & write protect for controller 0 */
	pxa_mmc_slot[1].gpio_detect = 1;
	pxa_mmc_slot[1].gpio_cd  = mfp_to_gpio(MFP_PIN_GPIO86);
	pxa_mmc_slot[1].gpio_wp  = mfp_to_gpio(MFP_PIN_GPIO87);

	pxa168_add_sdh(1, &avengers_lite_sdh_platform_data);

#ifdef CONFIG_SD8XXX_RFKILL
	add_sd8x_rfkill_device(mfp_to_gpio(MFP_PIN_GPIO104), 0,
			&avengers_lite_sdh3_platform_data.pmmc);
#endif
	pxa168_add_sdh(3, &avengers_lite_sdh3_platform_data);
}
#endif

struct platform_device pxa168_device_rtc = {
	.name           = "ec-rtc",
	.id             = -1,
};

static void __init avengers_lite_init_rtc(void)
{
	platform_device_register(&pxa168_device_rtc);
}

#ifdef CONFIG_USB_GADGET_PXA_U2O
static struct pxa_usb_plat_info avengers_lite_u2o_info = {
	.phy_init	= pxa168_usb_phy_init,
};
#endif

#ifdef CONFIG_USB_EHCI_PXA_U2H
/* USB 2.0 Host Controller */
static struct u2h_work_t {
	struct device *dev;
	struct work_struct work;
} u2h_work;
static int gpio_u2h_oc = mfp_to_gpio(MFP_PIN_GPIO37);

int avengers_lite_is_u2h_oc(void)
{
	int ret;
	ret = gpio_get_value(gpio_u2h_oc);
	pr_debug("gpio_u2h_oc %x\n", ret);
	return !ret;
}

static void avengers_lite_u2h_oc_worker(struct work_struct *work)
{
	pr_debug("%s dev %p\n", __func__, u2h_work.dev);
	if (avengers_lite_is_u2h_oc())
		kobject_uevent(&u2h_work.dev->kobj, KOBJ_OFFLINE);
	else
		kobject_uevent(&u2h_work.dev->kobj, KOBJ_ONLINE);
}

static irqreturn_t avengers_lite_u2h_oc_handler(int irq, void *data)
{
	pr_debug("%s\n", __func__);
	schedule_work(&u2h_work.work);
	return IRQ_HANDLED;
}

static int avengers_lite_u2h_plat_init(struct device *dev)
{
	int ret;

	gpio_direction_input(gpio_u2h_oc);
	ret = request_irq(IRQ_GPIO(gpio_u2h_oc), avengers_lite_u2h_oc_handler,
		IRQF_DISABLED | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
		"u2h over current", NULL);
	if (ret) {
		printk("request u2h over current interrupt failed %d\n", ret);
		return -EIO;
	}

	INIT_WORK(&u2h_work.work, avengers_lite_u2h_oc_worker);
	u2h_work.dev = dev;

	return 0;
}

static int avengers_lite_u2h_vbus_set(int enable)
{
	int usb_p_en = mfp_to_gpio(MFP_PIN_GPIO106);

	if (gpio_request(usb_p_en, "usb_p_en")) {
		printk(KERN_INFO "gpio %d request failed\n", usb_p_en);
		return -EIO;
	}
	gpio_direction_output(usb_p_en, 0);
	gpio_free(usb_p_en);

	return 0;
}

static struct pxa_usb_plat_info avengers_lite_u2h_info = {
	.phy_init	= pxa168_usb_phy_init,
	.vbus_set	= avengers_lite_u2h_vbus_set,
	.plat_init	= avengers_lite_u2h_plat_init,
};
#endif

#if defined(CONFIG_SPI_PXA2XX) || defined(CONFIG_SPI_PXA2XX_MODULE)
#define GPIO_CMMB_CS MFP_PIN_GPIO112
#define GPIO_CMMB_IRQ MFP_PIN_GPIO25
#define GPIO_CMMB_POWER MFP_PIN_GPIO110

static struct pxa2xx_spi_master pxa_ssp_master_info = {
	.num_chipselect	= 1,
#if !defined(CONFIG_MTD_M25P80)
	.enable_dma = 1,
#endif
};

/*
 * spi_finish used by spi_read_bytes to control
 * GPIO_CMMB_CS to be pull down always
 * when read not finished.
 */
int spi_finish = 1;

static int cmmb_power_on(void)
{
	if (gpio_request(GPIO_CMMB_POWER, "cmmb if101 power")) {
		pr_warning("failed to request GPIO for CMMB POWER\n");
		return -EIO;
	}
	gpio_direction_output(GPIO_CMMB_POWER, 0);
	mdelay(100);
	gpio_direction_output(GPIO_CMMB_POWER, 1);
	gpio_free(GPIO_CMMB_POWER);
	mdelay(100);

	printk("CMMB module is power on\n");

	return 0;
}

static int cmmb_power_off(void)
{
	if (gpio_request(GPIO_CMMB_POWER, "cmmb if101 power")) {
		pr_warning("failed to request GPIO for CMMB POWER\n");
		return -EIO;
	}
	gpio_direction_output(GPIO_CMMB_POWER, 0);
	gpio_free(GPIO_CMMB_POWER);
	mdelay(100);

	printk("CMMB module is power off\n");

	return 0;
}

static struct cmmb_platform_data cmmb_info = {
	.power_on = cmmb_power_on,
	.power_off = cmmb_power_off,
};

static void cmmb_if101_cs(u32 cmd)
{
	if (!spi_finish && cmd == PXA2XX_CS_DEASSERT)
		return;
	gpio_set_value(GPIO_CMMB_CS, !(cmd == PXA2XX_CS_ASSERT));
}

static struct pxa2xx_spi_chip cmmb_if101_chip = {
	.rx_threshold	= 1,
	.tx_threshold	= 1,
	.cs_control	= cmmb_if101_cs,
};

/* bus_num must match id in pxa2xx_set_spi_info() call */
#if defined(CONFIG_CMMB_IF101)
static struct spi_board_info spi_board_info[] __initdata = {
	{
		.modalias	= "cmmb_if101",
		.platform_data	= &cmmb_info,
		.controller_data	= &cmmb_if101_chip,
		.irq		= gpio_to_irq(mfp_to_gpio(GPIO_CMMB_IRQ)),
		.max_speed_hz	= 1000000,
		.bus_num	= 2,
		.chip_select	= 0,
		.mode		= SPI_MODE_0,
	},
};
#elif defined(CONFIG_MTD_M25P80)
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
#endif

static void __init avengers_lite_init_spi(void)
{
#if defined(CONFIG_CMMB_IF101)
	int err;

	err = gpio_request(GPIO_CMMB_CS, "cmmb if101 cs");
	if (err) {
		pr_warning("failed to request GPIO for CMMB CS\n");
		return;
	}
	gpio_direction_output(GPIO_CMMB_CS, 1);

	err = gpio_request(GPIO_CMMB_IRQ, "cmmb if101 irq");
	if (err) {
		pr_warning("failed to request GPIO for CMMB IRQ\n");
		return;
	}
	gpio_direction_input(GPIO_CMMB_IRQ);
#endif
	pxa168_add_ssp(1);
	pxa168_add_spi(2, &pxa_ssp_master_info);
#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_CMMB_IF101)
	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
#endif
}
#else
static inline void avengers_lite_init_spi(void) {}
#endif

static struct platform_device *devices[] __initdata = {
	&pxa168_device_pwm0,
	&avengers_lite_power_mcu,
	&avengers_lite_power_supply,
	&avengers_lite_power_button,
};

DECLARE_AVENGERSLITE_MLC_4G_ANDROID_PARTITIONS(avengerslite_mlc_4g_android_partitions);
DECLARE_AVENGERSLITE_MLC_1G_ANDROID_PARTITIONS(avengerslite_mlc_1g_android_partitions);
DECLARE_AVENGERSLITE_MLC_4G_MAEMO_PARTITIONS(avengerslite_mlc_4g_maemo_partitions);
DECLARE_AVENGERSLITE_MLC_1G_MAEMO_PARTITIONS(avengerslite_mlc_1g_maemo_partitions);
DECLARE_AVENGERSLITE_SLC_PARTITIONS(avengerslite_slc_partitions);
static struct pxa3xx_nand_platform_data avengers_lite_nand_info;
static void __init avengers_lite_add_nand(void)
{
	unsigned long long mlc_size = 0;

	mlc_size = get_mlc_size();
	printk(KERN_INFO "@%s Get mlc size from CMDLINE:%d\n", __FUNCTION__, (int)mlc_size);

	if (is_android()) {
		if (board_is_2p5()) {
			avengers_lite_nand_info.parts[0] = avengerslite_mlc_1g_android_partitions;
			avengers_lite_nand_info.nr_parts[0] = ARRAY_SIZE(avengerslite_mlc_1g_android_partitions);
			goto done;
		} else {
			avengers_lite_nand_info.parts[0] = avengerslite_slc_partitions;
			avengers_lite_nand_info.nr_parts[0] = ARRAY_SIZE(avengerslite_slc_partitions);
		}

		switch (mlc_size) {
		case 1:  /* 1G */
			avengers_lite_nand_info.parts[1] = avengerslite_mlc_1g_android_partitions;
			avengers_lite_nand_info.nr_parts[1] = ARRAY_SIZE(avengerslite_mlc_1g_android_partitions);
			break;
		case 4:  /* 4G */
			avengers_lite_nand_info.parts[1] = avengerslite_mlc_4g_android_partitions;
			avengers_lite_nand_info.nr_parts[1] = ARRAY_SIZE(avengerslite_mlc_4g_android_partitions);
			break;
		default:
			/* FIXME: do something or not */
			avengers_lite_nand_info.parts[1] = avengerslite_mlc_4g_android_partitions;
			avengers_lite_nand_info.nr_parts[1] = ARRAY_SIZE(avengerslite_mlc_4g_android_partitions);
			break;
		}
	} else {
		if (board_is_2p5()) {
			avengers_lite_nand_info.parts[0] = avengerslite_mlc_1g_maemo_partitions;
			avengers_lite_nand_info.nr_parts[0] = ARRAY_SIZE(avengerslite_mlc_1g_maemo_partitions);
			goto done;
		} else {
			avengers_lite_nand_info.parts[0] = avengerslite_slc_partitions;
			avengers_lite_nand_info.nr_parts[0] = ARRAY_SIZE(avengerslite_slc_partitions);
		}

		switch (mlc_size) {
		case 1:  /* 1G */
			avengers_lite_nand_info.parts[1] = avengerslite_mlc_1g_maemo_partitions;
			avengers_lite_nand_info.nr_parts[1] = ARRAY_SIZE(avengerslite_mlc_1g_maemo_partitions);
			break;
		case 4:  /* 4G */
			avengers_lite_nand_info.parts[1] = avengerslite_mlc_4g_maemo_partitions;
			avengers_lite_nand_info.nr_parts[1] = ARRAY_SIZE(avengerslite_mlc_4g_maemo_partitions);
			break;
		default:
			/* FIXME: do something or not */
			avengers_lite_nand_info.parts[1] = avengerslite_mlc_4g_maemo_partitions;
			avengers_lite_nand_info.nr_parts[1] = ARRAY_SIZE(avengerslite_mlc_4g_maemo_partitions);
			break;
		}
		/* make mlc whole as a massstorage room */
	}

done:
	avengers_lite_nand_info.use_dma = 1;
	avengers_lite_nand_info.RD_CNT_DEL = 0;
	avengers_lite_nand_info.enable_arbiter = 1;
	pxa168_add_nand((struct flash_platform_data *)&avengers_lite_nand_info);
}

unsigned int mfp_read_boardid(unsigned long *cfgs, int num) {
	unsigned long id = 0;
	int i;

	for (i = 0; i < num; i++, cfgs++) {
		unsigned long c = *cfgs;
		int pin;
		int pin_level;

		pin = MFP_PIN(c);/* get pin no. */
		pin_level = 0;

		gpio_direction_input(pin);

		pin_level = gpio_get_value(pin) ? 1 : 0;
		id <<= 1;
		id |= pin_level;
	}
	return id;
}

static void gpio_ec_init(void)
{
	gpio_direction_input(MFP_PIN_GPIO89);
	gpio_direction_output(MFP_PIN_GPIO88, 0);
}

static void avenger_lite_power_off(void)
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

#ifdef CONFIG_USB_ANDROID

#define NUM_ANDROID_USB_FUNCTIONS 2
char *android_usb_functions[NUM_ANDROID_USB_FUNCTIONS]
	= {"usb_mass_storage", "adb"};

/* android usb adb device data */
static struct android_usb_platform_data android_data = {
	.product_id = 0x0c02,
	.vendor_id = 0x0bb4,
	.product_name = "Avengers_lite",
	.manufacturer_name = "Marvell",
	.num_products = 0,
	.products = NULL,
	.num_functions = NUM_ANDROID_USB_FUNCTIONS,
	.functions = android_usb_functions,
};

static struct platform_device android_usb = {
	.name = "android_usb",
	.dev  = {
		.platform_data = &android_data,
	},
};

/* Platform data for "usb_mass_storage" driver. */
static struct usb_mass_storage_platform_data usb_mass_storage_data = {
	/* number of LUNS */
	.nluns = 2
};

static struct platform_device usb_mass_storage = {
	.name = "usb_mass_storage",
	.dev  = {
		.platform_data = &usb_mass_storage_data,
	},
};

static void __init android_init(void)
{
	platform_device_register(&android_usb);
	platform_device_register(&usb_mass_storage);
}

#endif

static void avengers_sel_debug(void)
{
	int sel_debug;
	sel_debug = mfp_to_gpio(MFP_PIN_GPIO2);
	if (gpio_request(sel_debug, "uart debug select")) {
		printk(KERN_ERR "gpio %d request failed\n", sel_debug);
		return;
	}
	gpio_direction_input(sel_debug);
	if (!gpio_get_value(sel_debug))
		SD_debug_setup("used");
	gpio_free(sel_debug);
}

static void __init avengers_lite_init(void)
{
	pxa168_set_vdd_iox(VDD_IO0, VDD_IO_3P3V);
	pxa168_set_vdd_iox(VDD_IO1, VDD_IO_3P3V);
	pxa168_set_vdd_iox(VDD_IO2, VDD_IO_3P3V);
	pxa168_set_vdd_iox(VDD_IO3, VDD_IO_3P3V);
	pxa168_set_vdd_iox(VDD_IO4, VDD_IO_3P3V);
	pxa168_mfp_set_fastio_drive(MFP_DS02X);

	mfp_config(ARRAY_AND_SIZE(avengers_lite_boardid_pin_config));

	AVLITE_BOARDID = mfp_read_boardid(
			ARRAY_AND_SIZE(avengers_lite_boardid_pin_config));
	printk(KERN_INFO "BOARD ID=%d\n",AVLITE_BOARDID);

	mfp_config(ARRAY_AND_SIZE(avengers_lite_pin_config_V16F));

	/* check whether to enable SD-to-UART debug */
	avengers_sel_debug();

	/* on-chip devices */
	pxa168_add_uart(4);
	avengers_lite_add_nand();
	pxa168_add_ssp(0);
	pxa168_add_twsi(0, &pwri2c_info, ARRAY_AND_SIZE(avengers_lite_i2c_board_info));
	pxa168_add_twsi(1, &pwri2c_info, ARRAY_AND_SIZE(pwri2c_board_info));
#ifdef CONFIG_USB_GADGET_PXA_U2O
	pxa168_add_u2o(&avengers_lite_u2o_info);
#endif
#ifdef CONFIG_USB_EHCI_PXA_U2H
	pxa168_add_u2h(&avengers_lite_u2h_info);
#endif
#if defined(CONFIG_MMC_PXA_SDH)
	avengers_lite_init_mmc();
#endif

	pxa168_add_freq();

	platform_add_devices(devices, ARRAY_SIZE(devices));

	avengers_lite_init_rtc();
	/* off-chip devices */
	avengers_lite_lcd_init();
	avengers_lite_backlight_register();
	avengers_lite_init_spi();

#if defined(CONFIG_PXA168_CAMERA) && defined(CONFIG_VIDEO_OV7740)
	pxa168_add_cam();
#endif

#if defined(CONFIG_PXA_ICR)
	pxa168_add_icr();
#endif

#if defined(CONFIG_KEYBOARD_GPIO)
	avengers_lite_gpio_keys_init();
#endif

#ifdef CONFIG_ANDROID_PMEM
	android_add_pmem("pmem", 0x02000000UL, 1, 0);
	android_add_pmem("pmem_adsp", 0x00600000UL, 0, 0);
#endif

	if (is_android())
#ifdef CONFIG_USB_ANDROID
		android_init();
#endif

	gpio_ec_init();
    
	pm_power_off = avenger_lite_power_off;
}

MACHINE_START(AVENGERS_LITE, "PXA168 Avengers lite Development Platform")
	.phys_io        = APB_PHYS_BASE,
	.boot_params    = 0x00000100,
	.io_pg_offst    = (APB_VIRT_BASE >> 18) & 0xfffc,
	.map_io		= pxa_map_io,
	.init_irq       = pxa168_init_irq,
	.timer          = &pxa168_timer,
	.init_machine   = avengers_lite_init,
MACHINE_END
