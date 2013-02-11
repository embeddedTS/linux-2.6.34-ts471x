/*
 *  linux/arch/arm/mach-mmp/tavorevb.c
 *
 *  Support for the Marvell PXA910-based TavorEVB Development Platform.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/smc91x.h>
<<<<<<< HEAD:arch/arm/mach-mmp/tavorevb.c
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/usb/otg.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/flash.h>
#include <mach/addr-map.h>
#include <mach/mfp-pxa910.h>
#include <mach/pxa168.h>
#include <mach/pxa910.h>
#include <mach/gpio.h>
#include <mach/micco.h>
#include <mach/sanremo.h>
#include <mach/portofino.h>
#include <mach/mmc.h>
#include <mach/devices.h>
#include <mach/cputype.h>
#include <mach/camera.h>
#include <mach/pxa3xx_nand.h>

#include <plat/part_table.h>
#include <plat/generic.h>
#include <plat/pxa_u2o.h>
#include <plat/pxa3xx_otg.h>
#include "common.h"

/*
* tavorevb_keypad_type
* 0 - use tavorevb builtin keypad
* 1 - use zylonite keypad
* NOTE: default to 0 at the moment
*/
static int tavorevb_keypad_type = 1;

static int __init keypad_setup(char *type)
{
	tavorevb_keypad_type = simple_strtol(type, NULL, 0);
	return 1;
}
__setup("tavorevb_keypad_type", keypad_setup);

=======

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/addr-map.h>
#include <mach/mfp-pxa910.h>
#include <mach/pxa910.h>
#include <mach/gpio.h>

#include "common.h"

>>>>>>> e40152ee1e1c7a63f4777791863215e3faa37a86:arch/arm/mach-mmp/tavorevb.c
static unsigned long tavorevb_pin_config[] __initdata = {
	/* UART2 */
	GPIO47_UART2_RXD,
	GPIO48_UART2_TXD,

	/* SMC */
	SM_nCS0_nCS0,
	SM_ADV_SM_ADV,
	SM_SCLK_SM_SCLK,
	SM_SCLK_SM_SCLK,
	SM_BE0_SM_BE0,
	SM_BE1_SM_BE1,

<<<<<<< HEAD:arch/arm/mach-mmp/tavorevb.c
	/* IRDA */
	GPIO51_IRDA_SHDN,

	/* I2C */
	GPIO53_CI2C_SCL,
	GPIO54_CI2C_SDA,

	/* SSP1 (I2S) */
	GPIO24_SSP1_SDATA_IN,
	GPIO21_SSP1_BITCLK,
	GPIO20_SSP1_SYSCLK,
	GPIO22_SSP1_SYNC,
	GPIO23_SSP1_DATA_OUT,
	GPIO124_MN_CLK_OUT,
	GPIO123_CLK_REQ,

=======
>>>>>>> e40152ee1e1c7a63f4777791863215e3faa37a86:arch/arm/mach-mmp/tavorevb.c
	/* DFI */
	DF_IO0_ND_IO0,
	DF_IO1_ND_IO1,
	DF_IO2_ND_IO2,
	DF_IO3_ND_IO3,
	DF_IO4_ND_IO4,
	DF_IO5_ND_IO5,
	DF_IO6_ND_IO6,
	DF_IO7_ND_IO7,
	DF_IO8_ND_IO8,
	DF_IO9_ND_IO9,
	DF_IO10_ND_IO10,
	DF_IO11_ND_IO11,
	DF_IO12_ND_IO12,
	DF_IO13_ND_IO13,
	DF_IO14_ND_IO14,
	DF_IO15_ND_IO15,
	DF_nCS0_SM_nCS2_nCS0,
	DF_ALE_SM_WEn_ND_ALE,
	DF_CLE_SM_OEn_ND_CLE,
	DF_WEn_DF_WEn,
	DF_REn_DF_REn,
	DF_RDY0_DF_RDY0,
<<<<<<< HEAD:arch/arm/mach-mmp/tavorevb.c

	/*keypad*/
	GPIO00_KP_MKIN0,
	GPIO01_KP_MKOUT0,
	GPIO02_KP_MKIN1,
	GPIO03_KP_MKOUT1,
	GPIO04_KP_MKIN2,
	GPIO05_KP_MKOUT2,
	GPIO06_KP_MKIN3,
	GPIO07_KP_MKOUT3,
	GPIO08_KP_MKIN4,
	GPIO09_KP_MKOUT4,
	GPIO10_KP_MKIN5,
	GPIO11_KP_MKOUT5,
	GPIO12_KP_MKIN6,
	GPIO13_KP_MKOUT6,
	GPIO14_KP_MKIN7,
	GPIO15_KP_MKOUT7,
	GPIO16_KP_DKIN0,
	GPIO17_KP_DKIN1,
	GPIO18_KP_DKIN2,
	GPIO19_KP_DKIN3,

	/* LCD */
	GPIO81_LCD_FCLK,
	GPIO82_LCD_LCLK,
	GPIO83_LCD_PCLK,
	GPIO84_LCD_DENA,
	GPIO85_LCD_DD0,
	GPIO86_LCD_DD1,
	GPIO87_LCD_DD2,
	GPIO88_LCD_DD3,
	GPIO89_LCD_DD4,
	GPIO90_LCD_DD5,
	GPIO91_LCD_DD6,
	GPIO92_LCD_DD7,
	GPIO93_LCD_DD8,
	GPIO94_LCD_DD9,
	GPIO95_LCD_DD10,
	GPIO96_LCD_DD11,
	GPIO97_LCD_DD12,
	GPIO98_LCD_DD13,
	GPIO100_LCD_DD14,
	GPIO101_LCD_DD15,
	GPIO102_LCD_DD16,
	GPIO103_LCD_DD17,
	GPIO104_LCD_DD18,
	GPIO105_LCD_DD19,
	GPIO106_LCD_DD20,
	GPIO107_LCD_DD21,
	GPIO108_LCD_DD22,
	GPIO109_LCD_DD23,

	/*1wire*/
	GPIO106_1WIRE,

	/*CCIC/CAM*/
	GPIO67_CCIC_IN7,
	GPIO68_CCIC_IN6,
	GPIO69_CCIC_IN5,
	GPIO70_CCIC_IN4,
	GPIO71_CCIC_IN3,
	GPIO72_CCIC_IN2,
	GPIO73_CCIC_IN1,
	GPIO74_CCIC_IN0,
	GPIO75_CAM_HSYNC,
	GPIO76_CAM_VSYNC,
	GPIO77_CAM_MCLK,
	GPIO78_CAM_PCLK,

#if defined(CONFIG_MMC_PXA_SDH)
	/*MMC sdh*/
	MMC1_DAT7_MMC1_DAT7,
	MMC1_DAT6_MMC1_DAT6,
	MMC1_DAT5_MMC1_DAT5,
	MMC1_DAT4_MMC1_DAT4,
	MMC1_DAT3_MMC1_DAT3,
	MMC1_DAT2_MMC1_DAT2,
	MMC1_DAT1_MMC1_DAT1,
	MMC1_DAT0_MMC1_DAT0,
	MMC1_CMD_MMC1_CMD,
	MMC1_CLK_MMC1_CLK,
	MMC1_CD_MMC1_CD,
	MMC1_WP_MMC1_WP,
#endif
};

#if defined(CONFIG_PXA168_CAMERA)
/* sensor init */
static int sensor_power_onoff(int on, int sensor)
{
	/*
	 * sensor, 0, low resolution
	 * sensor, 1, high resolution
	 */

	unsigned int cam_hi_pwdn;
	unsigned int cam_lo_pwdn;

	if (cpu_is_pxa910()) {
		cam_hi_pwdn = mfp_to_gpio(MFP_PIN_GPIO80);
		cam_lo_pwdn = mfp_to_gpio(MFP_PIN_GPIO79);
	}

	if (gpio_request(cam_hi_pwdn, "CAM_EANBLE_HI_SENSOR")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", cam_hi_pwdn);
		return -EIO;
	}

	if (gpio_request(cam_lo_pwdn, "CAM_EANBLE_LO_SENSOR")){
		gpio_free(cam_hi_pwdn);
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d\n", cam_lo_pwdn);
		return -EIO;
	}

	if (on) {
		if(sensor){
			gpio_direction_output(cam_hi_pwdn, 0);
			gpio_direction_output(cam_lo_pwdn, 1);
		}else{
			gpio_direction_output(cam_lo_pwdn, 0);
			gpio_direction_output(cam_hi_pwdn, 1);
		}
	} else {
		if (sensor)
			gpio_direction_output(cam_hi_pwdn, 1);
		else
			gpio_direction_output(cam_lo_pwdn, 1);

	}
	gpio_free(cam_hi_pwdn);
	gpio_free(cam_lo_pwdn);

	return 0;
}

static struct sensor_platform_data ov7660_sensor_data = {
        .id = SENSOR_LOW,
        .power_on = sensor_power_onoff,
};
static struct sensor_platform_data ov3640_sensor_data = {
        .id = SENSOR_HIGH,
        .power_on = sensor_power_onoff,
};
#endif

=======
};

>>>>>>> e40152ee1e1c7a63f4777791863215e3faa37a86:arch/arm/mach-mmp/tavorevb.c
static struct smc91x_platdata tavorevb_smc91x_info = {
	.flags	= SMC91X_USE_16BIT | SMC91X_NOWAIT,
};

static struct resource smc91x_resources[] = {
	[0] = {
		.start	= SMC_CS1_PHYS_BASE + 0x300,
		.end	= SMC_CS1_PHYS_BASE + 0xfffff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= gpio_to_irq(80),
		.end	= gpio_to_irq(80),
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
	}
};

static struct platform_device smc91x_device = {
	.name		= "smc91x",
	.id		= 0,
	.dev		= {
		.platform_data = &tavorevb_smc91x_info,
	},
	.num_resources	= ARRAY_SIZE(smc91x_resources),
	.resource	= smc91x_resources,
};

<<<<<<< HEAD:arch/arm/mach-mmp/tavorevb.c
#if defined(CONFIG_MICCO) || defined(CONFIG_MICCO_MODULE)
static int micco_init_irq(void)
{
	return 0;
}

static int micco_ack_irq(void)
{
	return 0;
}

static void micco_init(void)
{
	u8 value;

	/* Mask interrupts that are not needed */
	micco_write(MICCO_IRQ_MASK_A, 0xFE);
	micco_write(MICCO_IRQ_MASK_B, 0xFF);
	micco_write(MICCO_IRQ_MASK_C, 0xFF);
	micco_write(MICCO_IRQ_MASK_D, 0xFF);

	/* avoid SRAM power off during sleep*/
	micco_write(0x10, 0x07);
	micco_write(0x11, 0xff);
	//micco_write(0x12, 0xbf);/*never enable LDO4: CP controls it via COMM_OVER3 reg (0x22)*/

	/* Enable the ONKEY power down functionality */
	micco_write(MICCO_SYSCTRL_B, 0x20);
	micco_write(MICCO_SYSCTRL_A, 0x60);

	/* IRQ is masked during the power-up sequence and will not be released
	 * until they have been read for the first time */
	micco_read(MICCO_EVENT_A, &value);
	micco_read(MICCO_EVENT_B, &value);
	micco_read(MICCO_EVENT_C, &value);
	micco_read(MICCO_EVENT_D, &value);
}

/* micco_power_module[] should be consistent with enum
 * in include/asm-arm/arch-pxa/pxa3xx_pmic.h */
static struct power_supply_module miccoB0_power_modules[] = {
	/* {command,		power_module}, */
	{VCC_CORE,		BUCK1},
	{VCC_SRAM,		LDO2},
	{VCC_MVT,		LDO1},
	{VCC_3V_APPS,		LDO3},
	{VCC_SDIO,		LDO14},
	{VCC_CAMERA_ANA,	LDO13},
	{VCC_USB,		LDO5},
	{VCC_LCD,		LDO3},
	{VCC_TSI,		0},
	{VCC_CAMERA_IO,		LDO3},
	{VCC_1P8V,		LDO4},
	{VCC_MEM,		BUCK3},
	{HDMI_TX,		0},
	{TECH_3V,		0},
	{TECH_1P8V,		0},
};

/* micco_power_module[] should be consistent with enum
 * in include/asm-arm/arch-pxa/pxa3xx_pmic.h */
static struct power_supply_module miccoEA_power_modules[] = {
	/* {command,		power_module}, */
	{VCC_CORE,		BUCK1},
	{VCC_SRAM,		BUCK2},
	{VCC_MVT,		LDO1},
	{VCC_3V_APPS,		LDO3},
	{VCC_SDIO,		LDO13},
	{VCC_CAMERA_ANA,	LDO3},
	{VCC_USB,		LDO5},
	{VCC_LCD,		LDO3},
	{VCC_TSI,		0},
	{VCC_CAMERA_IO,		LDO2},
	{VCC_1P8V,		LDO4},
	{VCC_MEM,		LDO2},
	{HDMI_TX,		0},
	{TECH_3V,		0},
	{TECH_1P8V,		0},
};

static struct power_chip micco_chips[] = {
	{0x10,	"miccoB0",	miccoB0_power_modules},
	{0x30,	"miccoEA",	miccoEA_power_modules},
	{0x31,	"miccoEB",	miccoEA_power_modules},
	{0,	NULL,		NULL},
};

static struct micco_platform_data micco_data = {
	.init_irq = micco_init_irq,
	.ack_irq = micco_ack_irq,
	.platform_init = micco_init,
	.power_chips = micco_chips,
};

#endif /* CONFIG_MICCO || CONFIG_MICCO_MODULE*/

static unsigned int zylonite_matrix_key_map[] = {
		/* KEY(row, col, key_code) */
		KEY(0, 0, KEY_A), KEY(0, 1, KEY_B), KEY(0, 2, KEY_C),
		KEY(0, 5, KEY_D), KEY(1, 0, KEY_E), KEY(1, 1, KEY_F),
		KEY(1, 2, KEY_G), KEY(1, 5, KEY_H), KEY(2, 0, KEY_I),
		KEY(2, 1, KEY_J), KEY(2, 2, KEY_K), KEY(2, 5, KEY_L),
		KEY(3, 0, KEY_M), KEY(3, 1, KEY_N), KEY(3, 2, KEY_O),
		KEY(3, 5, KEY_P), KEY(5, 0, KEY_Q), KEY(5, 1, KEY_R),
		KEY(5, 2, KEY_S), KEY(5, 5, KEY_T), KEY(6, 0, KEY_U),
		KEY(6, 1, KEY_V), KEY(6, 2, KEY_W), KEY(6, 5, KEY_X),
		KEY(7, 1, KEY_Y), KEY(7, 2, KEY_Z),

		KEY(4, 4, KEY_0), KEY(1, 3, KEY_1), KEY(4, 1, KEY_2),
		KEY(1, 4, KEY_3), KEY(2, 3, KEY_4), KEY(4, 2, KEY_5),
		KEY(2, 4, KEY_6), KEY(3, 3, KEY_7), KEY(4, 3, KEY_8),
		KEY(3, 4, KEY_9),

		KEY(4, 5, KEY_SPACE),
		KEY(5, 3, KEY_KPASTERISK),      /* * */
		KEY(5, 4, KEY_KPDOT),           /* #" */

		KEY(0, 7, KEY_UP),
		KEY(1, 7, KEY_DOWN),
		KEY(2, 7, KEY_LEFT),
		KEY(3, 7, KEY_RIGHT),
		KEY(2, 6, KEY_HOME),
		KEY(3, 6, KEY_END),
		KEY(6, 4, KEY_DELETE),
		KEY(6, 6, KEY_BACK),
		KEY(6, 3, KEY_CAPSLOCK),/* KEY_LEFTSHIFT), */

		KEY(4, 6, KEY_ENTER),   /* scroll push */
		KEY(5, 7, KEY_ENTER),   /* keypad action */

		KEY(0, 4, KEY_EMAIL),
		KEY(5, 6, KEY_SEND),
		KEY(4, 0, KEY_CALENDAR),
		KEY(7, 6, KEY_RECORD),
		KEY(6, 7, KEY_VOLUMEUP),
		KEY(7, 7, KEY_VOLUMEDOWN),

		KEY(0, 6, KEY_F22),     /* soft1 */
		KEY(1, 6, KEY_F23),     /* soft2 */

		KEY(0, 3, KEY_AUX),     /* contact */
};

static unsigned int tavorevb_matrix_key_map[] = {
		/* KEY(row, col, key_code) */
		KEY(0, 4, KEY_A), KEY(0, 5, KEY_B), KEY(0, 6, KEY_C),
		KEY(0, 7, KEY_D), KEY(1, 4, KEY_E), KEY(1, 5, KEY_F),
		KEY(1, 6, KEY_G), KEY(1, 7, KEY_H), KEY(2, 4, KEY_I),
		KEY(2, 5, KEY_J), KEY(2, 6, KEY_K), KEY(2, 7, KEY_L),
		KEY(3, 4, KEY_M), KEY(3, 5, KEY_N), KEY(3, 6, KEY_O),
		KEY(3, 7, KEY_P), KEY(7, 3, KEY_Q), KEY(4, 5, KEY_R),
		KEY(4, 6, KEY_S), KEY(4, 7, KEY_T), KEY(5, 4, KEY_U),
		KEY(5, 5, KEY_V), KEY(5, 6, KEY_W), KEY(5, 7, KEY_X),
		KEY(6, 4, KEY_Y), KEY(6, 5, KEY_Z),

		KEY(0, 3, KEY_0), KEY(2, 0, KEY_1), KEY(2, 1, KEY_2),
		KEY(2, 2, KEY_3), KEY(2, 3, KEY_4), KEY(1, 0, KEY_5),
		KEY(1, 1, KEY_6), KEY(1, 2, KEY_7),
		KEY(1, 3, KEY_8), KEY(0, 2, KEY_9),

		KEY(6, 6, KEY_SPACE),
		KEY(0, 0, KEY_KPASTERISK),      /* * */
		KEY(0, 1, KEY_KPDOT),           /* #" */

		KEY(4, 1, KEY_UP),
		KEY(4, 3, KEY_DOWN),
		KEY(4, 0, KEY_LEFT),
		KEY(4, 2, KEY_RIGHT),
		KEY(6, 0, KEY_HOME),
		KEY(3, 2, KEY_END),
		KEY(6, 1, KEY_DELETE),
		KEY(5, 2, KEY_BACK),
		KEY(6, 3, KEY_CAPSLOCK),/* KEY_LEFTSHIFT), */

		KEY(6, 2, KEY_ENTER),   /* keypad action */

		KEY(7, 2, KEY_EMAIL),
		KEY(3, 1, KEY_SEND),
		KEY(7, 1, KEY_CALENDAR),
		KEY(5, 3, KEY_RECORD),
		KEY(5, 0, KEY_VOLUMEUP),
		KEY(5, 1, KEY_VOLUMEDOWN),

		KEY(3, 0, KEY_F22),	/* soft1 */
		KEY(3, 3, KEY_F23),	/* soft2 */

		KEY(7, 0, KEY_AUX),     /* contact */
};

static struct pxa27x_keypad_platform_data tavorevb_keypad_info = {
	.matrix_key_rows        = 8,
	.matrix_key_cols        = 8,
	.matrix_key_map         = tavorevb_matrix_key_map,
	.matrix_key_map_size    = ARRAY_SIZE(tavorevb_matrix_key_map),
	.debounce_interval      = 30,
};
static struct pxa27x_keypad_platform_data zylonite_keypad_info = {
	.matrix_key_rows        = 8,
	.matrix_key_cols        = 8,
	.matrix_key_map         = zylonite_matrix_key_map,
	.matrix_key_map_size    = ARRAY_SIZE(zylonite_matrix_key_map),
	.debounce_interval      = 30,
};


static struct fb_videomode video_modes[] = {
        /* sharp_ls037 QVGA mode info */
        [0] = {
                .pixclock       = 158000,
                .refresh        = 60,
                .xres           = 240,
                .yres           = 320,
                .hsync_len      = 4,
                .left_margin    = 39,
                .right_margin   = 39,
                .vsync_len      = 1,
                .upper_margin   = 1,
                .lower_margin   = 2,
                .sync		= FB_SYNC_VERT_HIGH_ACT | FB_SYNC_HOR_HIGH_ACT,
        },
        /* sharp_ls037 VGA mode info */
        [1] = {
                .pixclock       = 39700,
                .refresh        = 60,
                .xres           = 480,
                .yres           = 640,
                .hsync_len      = 8,
                .left_margin    = 81,
                .right_margin   = 81,
                .vsync_len      = 1,
                .upper_margin   = 2,
                .lower_margin   = 7,
                .sync           = 0,
        },
};

#define LCD_SCLK (312000000UL)
static struct pxa168fb_mach_info pxa168_tavorevb_lcd_info = {
        .id                     = "GFX Layer",
        .sclk_clock             = LCD_SCLK,
        .num_modes              = ARRAY_SIZE(video_modes),
        .modes                  = video_modes,
        .pix_fmt                = PIX_FMT_RGB565,
        .io_pin_allocation_mode = PIN_MODE_DUMB_16_GPIO,
        .dumb_mode              = DUMB_MODE_RGB565,
        .panel_rgb_reverse_lanes= 0,
        .gpio_output_data       = 1,
        .gpio_output_mask       = 0xff,
        .invert_composite_blank = 0,
        .invert_pix_val_ena     = 0,
        .invert_pixclock        = 0,
        .invert_vsync           = 0,
        .invert_hsync           = 0,
        .panel_rbswap		= 1,
        .active                 = 1,
        .enable_lcd             = 1,
        .spi_gpio_cs            = -1,
        .spi_gpio_reset         = -1,
	.max_fb_size		= 1280 * 720 * 4,
};

static struct pxa168fb_mach_info pxa168_tavorevb_lcd_ovly_info = {
        .id                     = "Video Layer",
        .sclk_clock             = LCD_SCLK,
        .num_modes              = ARRAY_SIZE(video_modes),
        .modes                  = video_modes,
        .pix_fmt                = PIX_FMT_RGB565,
        .io_pin_allocation_mode = PIN_MODE_DUMB_16_GPIO,
        .dumb_mode              = DUMB_MODE_RGB565,
        .panel_rgb_reverse_lanes= 0,
        .gpio_output_data       = 1,
        .gpio_output_mask       = 0xff,
        .invert_composite_blank = 0,
        .invert_pix_val_ena     = 0,
        .invert_pixclock        = 0,
        .invert_vsync           = 0,
        .invert_hsync           = 0,
        .panel_rbswap		= 1,
        .active                 = 1,
        .enable_lcd             = 1,
        .spi_gpio_cs            = -1,
        .spi_gpio_reset         = -1,
	.max_fb_size		= 1280 * 720 * 4,
};

#if defined(CONFIG_SANREMO) || defined(CONFIG_SANREMO_MODULE)
static int sanremo_init_irq(void)
{
	return 0;
}

static int sanremo_ack_irq(void)
{
	return 0;
}

static void sanremo_init(void)
{
	sanremo_write(SANREMO_GPADC_MISC1, 0x0b); 
}

/* sanremo_power_module[] should be consistent with enum
 * in include/asm-arm/arch-pxa/pxa3xx_pmic.h */
static struct power_supply_module sanremo_power_modules[] = {
	/* {command,		power_module}, */
	{VCC_CORE,		SAN_BUCK1},
	{VCC_USB,		SAN_LDO5},
	{VCC_CAMERA_ANA,	SAN_LDO12},
	{VCC_SDIO,		SAN_LDO14},
};


static struct power_chip sanremo_chips[] = {
	{0x40,	"sanremo",	sanremo_power_modules},
	{0x41,	"sanremo",	sanremo_power_modules},
	{0,	NULL,		NULL},
};

static struct sanremo_platform_data sanremo_data = {
	.init_irq = sanremo_init_irq,
	.ack_irq = sanremo_ack_irq,
	.platform_init = sanremo_init,
	.power_chips = sanremo_chips,
};
#endif /* CONFIG_PXA3xx_SANREMO || CONFIG_PXA3xx_SANREMO_MODULE*/

static struct i2c_pxa_platform_data i2c_info __initdata = {
	.use_pio		= 1,
};

static struct i2c_board_info i2c_board_info[] =
{
#if defined(CONFIG_MICCO) || defined(CONFIG_MICCO_MODULE)
	{
		.type		= "micco",
		.addr		= 0x34,
		.platform_data	= &micco_data,
		.irq		= IRQ_PXA168_PMIC_INT,
	},
#endif
#if defined(CONFIG_SANREMO) || defined(CONFIG_SANREMO_MODULE)
	{
		.type		= "sanremo",
		.addr		= 0x30,
		.platform_data	= &sanremo_data ,
		.irq		= IRQ_PXA168_PMIC_INT,
	},
#endif
#if defined(CONFIG_PORTOFINO) || defined(CONFIG_PORTOFINO_MODULE)
	{
		.type		= "portofino",
		.addr		= 0x10,
	},
#endif
#if defined(CONFIG_PXA168_CAMERA)
        {
                .type           = "ov7660",
                .addr           = 0x21,
                .platform_data  = &ov7660_sensor_data,
        },
        {
                .type           = "ov3640",
                .addr           = 0x3C,
                .platform_data  = &ov3640_sensor_data,
        },
#endif
};

DECLARE_ANDROID_128M_V75_PARTITIONS(android_128m_v75_partitions);
DECLARE_128M_V75_PARTITIONS(generic_128m_v75_partitions);
static struct pxa3xx_nand_platform_data tavorevb_nand_info;
static void __init tavorevb_add_nand(void)
{
	if (is_android()) {
		tavorevb_nand_info.parts[0] = android_128m_v75_partitions;
		tavorevb_nand_info.nr_parts[0] = ARRAY_SIZE(android_128m_v75_partitions);
	} else {
		tavorevb_nand_info.parts[0] = generic_128m_v75_partitions;
		tavorevb_nand_info.nr_parts[0] = ARRAY_SIZE(generic_128m_v75_partitions);
	}

	tavorevb_nand_info.use_dma = 0;
	tavorevb_nand_info.enable_arbiter = 1;
	pxa168_add_nand(&tavorevb_nand_info);
}

static struct flash_platform_data tavorevb_onenand_info;
static void __init tavorevb_add_onenand(void)
{
	if (is_android()) {
		tavorevb_onenand_info.parts = android_128m_v75_partitions;
		tavorevb_onenand_info.nr_parts = ARRAY_SIZE(android_128m_v75_partitions);
	} else {
		tavorevb_onenand_info.parts = generic_128m_v75_partitions;
		tavorevb_onenand_info.nr_parts = ARRAY_SIZE(generic_128m_v75_partitions);
	}
	pxa168_add_onenand(&tavorevb_onenand_info);
}

#if defined(CONFIG_MMC_PXA_SDH)
static struct pxasdh_platform_data tavorevb_sdh_platform_data = {
	.detect_delay	= 20,
	.ocr_mask	= MMC_VDD_32_33|MMC_VDD_33_34,
};
#endif

#ifdef CONFIG_USB_GADGET_PXA_U2O
static int tavorevb_vbus_status(unsigned base)
{
	int status = VBUS_LOW;

	if (pxa3xx_pmic_is_vbus_assert()) {
		status = VBUS_HIGH;
	}
	return status;
}

static int tavorevb_vbus_detect(void *func, int enable)
{
	if (enable) {
		pmic_callback_register(PMIC_EVENT_USB, func);
		pxa3xx_pmic_set_pump(1);
	} else {
		pxa3xx_pmic_set_pump(0);
		pmic_callback_unregister(PMIC_EVENT_USB, func);
	}

	return 0;
}

static struct pxa_usb_plat_info tavorevb_u2o_info = {
	.phy_init	= pxa168_usb_phy_init,
	.vbus_status	= tavorevb_vbus_status,
	.vbus_detect	= tavorevb_vbus_detect,
	.is_otg		= 1,
};
#endif

static void __init tavorevb_init(void)
{

=======
static void __init tavorevb_init(void)
{
>>>>>>> e40152ee1e1c7a63f4777791863215e3faa37a86:arch/arm/mach-mmp/tavorevb.c
	mfp_config(ARRAY_AND_SIZE(tavorevb_pin_config));

	/* on-chip devices */
	pxa910_add_uart(1);
<<<<<<< HEAD:arch/arm/mach-mmp/tavorevb.c
	pxa168_add_twsi(0, &i2c_info, ARRAY_AND_SIZE(i2c_board_info));
	tavorevb_add_nand();
	tavorevb_add_onenand();
	pxa910_add_ire();
	pxa910_add_acipc();
	if (tavorevb_keypad_type == 1)
		pxa168_add_keypad(&zylonite_keypad_info);
	else
		pxa168_add_keypad(&tavorevb_keypad_info);
#if defined(CONFIG_MMC_PXA_SDH)
	pxa168_add_sdh(0, &tavorevb_sdh_platform_data);
#endif

	if(cpu_is_pxa910_Ax()){
		pxa910_add_fb(&pxa168_tavorevb_lcd_info);
		pxa910_add_fb_ovly(&pxa168_tavorevb_lcd_ovly_info);
	}else{
		pxa168_add_fb(&pxa168_tavorevb_lcd_info);
		pxa168_add_fb_ovly(&pxa168_tavorevb_lcd_ovly_info);
	}
#if defined(CONFIG_PXA168_CAMERA)
        pxa168_add_cam();
#endif
#ifdef CONFIG_USB_GADGET_PXA_U2O
	pxa168_add_u2o(&tavorevb_u2o_info);
#endif
#ifdef CONFIG_USB_OTG
	pxa168_add_u2ootg(&tavorevb_u2o_info);
	pxa168_add_u2oehci(&tavorevb_u2o_info);
#endif
	pxa910_add_ssp(1);
	pxa910_add_imm();
	pxa168_add_freq();
=======
>>>>>>> e40152ee1e1c7a63f4777791863215e3faa37a86:arch/arm/mach-mmp/tavorevb.c

	/* off-chip devices */
	platform_device_register(&smc91x_device);
}

MACHINE_START(TAVOREVB, "PXA910 Evaluation Board (aka TavorEVB)")
	.phys_io        = APB_PHYS_BASE,
	.boot_params    = 0x00000100,
	.io_pg_offst    = (APB_VIRT_BASE >> 18) & 0xfffc,
	.map_io		= pxa_map_io,
<<<<<<< HEAD:arch/arm/mach-mmp/tavorevb.c
	.init_irq       = pxa168_init_irq,
	.timer          = &pxa168_timer,
=======
	.init_irq       = pxa910_init_irq,
	.timer          = &pxa910_timer,
>>>>>>> e40152ee1e1c7a63f4777791863215e3faa37a86:arch/arm/mach-mmp/tavorevb.c
	.init_machine   = tavorevb_init,
MACHINE_END
