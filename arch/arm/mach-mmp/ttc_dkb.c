/*
 *  linux/arch/arm/mach-mmp/ttc_dkb.c
 *
 *  Support for the Marvell PXA910-based TTC_DKB Development Platform.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
<<<<<<< HEAD:arch/arm/mach-mmp/ttc_dkb.c
#include <linux/smc91x.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/delay.h>
#include <linux/pda_power.h>
#include <linux/switch.h>
#include <linux/i2c/pca9575.h>
#include <linux/i2c/pca953x.h>
#include <linux/i2c/si4703.h>
#include <linux/proc_fs.h>
#include <linux/sensor-input.h>

#include <asm/uaccess.h>
=======
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/onenand.h>

>>>>>>> e40152ee1e1c7a63f4777791863215e3faa37a86:arch/arm/mach-mmp/ttc_dkb.c
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/flash.h>
#include <mach/addr-map.h>
#include <mach/mfp-pxa910.h>
<<<<<<< HEAD:arch/arm/mach-mmp/ttc_dkb.c
#include <mach/pxa168.h>
#include <mach/pxa910.h>
#include <mach/cputype.h>
#include <mach/gpio.h>
#include <mach/sanremo.h>
#include <mach/portofino.h>
#include <mach/pxa3xx_nand.h>
#include <mach/resource.h>

#include <plat/part_table.h>
#include <plat/generic.h>
#include "common.h"
#include <mach/camera.h>

#include <plat/pxa_u2o.h>
#include <linux/usb/otg.h>
#ifdef CONFIG_SD8XXX_RFKILL
#include <linux/sd8x_rfkill.h>
#endif

extern int fake_suspend;

static int is_td_dkb = 0;
static int __init td_dkb_setup(char *__unused)
{
	is_td_dkb = 1;
	return 1;
}
__setup("td_dkb", td_dkb_setup);


#define ARRAY_AND_SIZE(x)       (x), ARRAY_SIZE(x)

#define GPIO_EXT0(x)		(NR_BUILTIN_GPIO + (x))
#define GPIO_EXT1(x)		(NR_BUILTIN_GPIO + 16 + (x))

static unsigned long ttc_dkb_pin_config[] __initdata = {
	/* GPS UART */
	GPIO43_UART1_RXD,
	GPIO44_UART1_TXD,

	/* GPS GPIO */
	GPIO15_GPIO15,
	GPIO16_GPIO16,
	GPIO45_GPIO45, /*share with TPO reset*/

=======
#include <mach/pxa910.h>

#include "common.h"

static unsigned long ttc_dkb_pin_config[] __initdata = {
>>>>>>> e40152ee1e1c7a63f4777791863215e3faa37a86:arch/arm/mach-mmp/ttc_dkb.c
	/* UART2 */
	GPIO47_UART2_RXD,
	GPIO48_UART2_TXD,

<<<<<<< HEAD:arch/arm/mach-mmp/ttc_dkb.c
	/* UART3/BT_UART */
	GPIO29_UART3_CTS,
	GPIO30_UART3_RTS,
	GPIO31_UART3_TXD,
	GPIO32_UART3_RXD,

	/* SMC */
	SM_nCS0_nCS0,
	SM_ADV_SM_ADV,
	SM_SCLK_SM_SCLK,
	SM_SCLK_SM_SCLK,
	SM_BE0_SM_BE0,
	SM_BE1_SM_BE1,

	/* IRDA */
	GPIO51_IRDA_SHDN,

	/* USB_ID */
	/*GPIO44_USB_ID,*/

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
>>>>>>> e40152ee1e1c7a63f4777791863215e3faa37a86:arch/arm/mach-mmp/ttc_dkb.c
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
<<<<<<< HEAD:arch/arm/mach-mmp/ttc_dkb.c

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
	GPIO104_LCD_SPIDOUT,
	GPIO105_LCD_SPIDIN,
	GPIO107_LCD_CS1,
	GPIO108_LCD_DCLK,
	GPIO106_LCD_RESET,	

	/*1wire*/
//	GPIO106_1WIRE,

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
	GPIO16_CAM_PWR_SUB,
	GPIO18_CAM_RESET_SUB,	
	GPIO15_CAM_PWR_MAIN,
	GPIO17_CAM_RESET_MAIN,
	GPIO49_CAM_AFEN,

	/*ethernet irq*/
	GPIO13_GPIO13,

#if defined(CONFIG_RADIO_SI4703) || defined(CONFIG_RADIO_SI4703_MODULE)
	GPIO19_FM_RDS_IRQ,
	//GPIO20_FM_RESET,
#endif 

#if defined(CONFIG_MMC_PXA_SDH)
	/*sdh MMC1*/
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

	/*sdh MMC2*/
	MMC2_DAT3_GPIO_37,
	MMC2_DAT2_GPIO_38,
	MMC2_DAT1_GPIO_39,
	MMC2_DAT0_GPIO_40,
	MMC2_CMD_GPIO_41,
	MMC2_CLK_GPIO_42,

	/*wlan_bt*/
	WLAN_PD_GPIO_14,
	WLAN_RESET_GPIO_20,
	WLAN_BT_RESET_GPIO_34,
	WLAN_MAC_WAKEUP_GPIO_35,
	WLAN_LHC_GPIO_36,
#endif
};

static struct smc91x_platdata ttc_dkb_smc91x_info = {
	.flags	= SMC91X_USE_16BIT | SMC91X_NOWAIT,
};

static struct resource smc91x_resources[] = {
	[0] = {
		.start	= SMC_CS1_PHYS_BASE + 0x300,
		.end	= SMC_CS1_PHYS_BASE + 0xfffff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= gpio_to_irq(13),
		.end	= gpio_to_irq(13),
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
	}
};

static struct platform_device smc91x_device = {
	.name		= "smc91x",
	.id		= 0,
	.dev		= {
		.platform_data = &ttc_dkb_smc91x_info,
	},
	.num_resources	= ARRAY_SIZE(smc91x_resources),
	.resource	= smc91x_resources,
};

#if defined(CONFIG_PXA168_CAMERA)
/* sensor init */
static int sensor_power_onoff(int on, int sensor)
{
	unsigned int cam_pwr;
	unsigned int cam_reset;
	unsigned int cam_afen;

	/* actually, no small power down pin needed */
	cam_pwr = sensor ? GPIO_EXT0(6):mfp_to_gpio(MFP_PIN_GPIO16);
	cam_reset = sensor ? GPIO_EXT0(4):GPIO_EXT0(14);
	cam_afen = mfp_to_gpio(MFP_PIN_GPIO49);

	if (gpio_request(cam_pwr, "CAM_PWR")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", cam_pwr);
		return -EIO;
	}
	if (gpio_request(cam_reset, "CAM_RESET")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", cam_reset);
		return -EIO;
	}
	if (gpio_request(cam_afen, "CAM_RESET")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", cam_afen);
		return -EIO;
	}

	if(on){
		gpio_direction_output(cam_afen, 1);
		msleep(1);
		gpio_direction_output(cam_pwr, 0);
		msleep(1);
		gpio_direction_output(cam_reset, 0);
		msleep(1);
		gpio_direction_output(cam_reset, 1);
		msleep(1);
		/* set MIPI_AVDD1P2V for MIPI IO */
		sanremo_write(SANREMO_LDO12, 0x8);
		msleep(1);
	}else{
		gpio_direction_output(cam_pwr, 1);
	}

	gpio_free(cam_pwr);
	gpio_free(cam_reset);
	gpio_free(cam_afen);
	return 0;
}
static struct sensor_platform_data ov7670_sensor_data = {
	.id = SENSOR_LOW,
	.power_on = sensor_power_onoff,
};

static struct sensor_platform_data ov3640_sensor_data = {
        .id = SENSOR_HIGH,
        .power_on = sensor_power_onoff,
};

/* sensor init over */
#endif

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
	u8 val;
	/* Mask interrupts that are not needed */
	sanremo_write(SANREMO_INTERRUPT_ENABLE1, 0x00);
	sanremo_write(SANREMO_INTERRUPT_ENABLE2, 0x00);
	sanremo_write(SANREMO_INTERRUPT_ENABLE3, 0x00);

	/* disable LDO5 turn on/off by LDO3_EN */
	sanremo_read(SANREMO_MISC2, &val);
	sanremo_write(SANREMO_MISC2, val | 0x80);

	/* enable LDO5 for AVDD_USB */
	sanremo_read(SANREMO_SUPPLIES_EN11, &val);
	sanremo_write(SANREMO_SUPPLIES_EN11, val | 0x80);

	/* Set AVDD_USB voltage as 3.3V */
	sanremo_write(SANREMO_LDO5, 0x0F);

	/* avoid SRAM power off during sleep*/
	sanremo_read(SANREMO_SUPPLIES_EN11, &val);
	printk("SANREMO_SUPPLIES_EN11 %x\n", val);
	sanremo_write(SANREMO_SUPPLIES_EN11, val & 0xBF);	/* never enable LDO4: CP controls it */
	sanremo_write(SANREMO_SUPPLIES_EN12, 0xFF);

	/* Enable the ONKEY power down functionality, power hold & watchdog disable */
	sanremo_write(SANREMO_WAKEUP, 0xA6);

	/* IRQ is masked during the power-up sequence and will not be released
	 * until they have been read for the first time */
	sanremo_write(SANREMO_INTERRUPT_STATUS1, 0x1F);
	sanremo_write(SANREMO_INTERRUPT_STATUS2, 0xFF);
	sanremo_write(SANREMO_INTERRUPT_STATUS3, 0xFF);

	sanremo_write(SANREMO_GPADC_MISC1, 0x0b);  /*enable GADC for CP and touch*/
	sanremo_write(SANREMO_TSI_PREBIAS_TIME, 0x06);  
	sanremo_write(SANREMO_PD_PREBIAS_TIME, 0x50);
	sanremo_write(SANREMO_RTC1, 0x40); /*Set RTC to use the external 32K frequency */
	sanremo_write(SANREMO_RTC_MISC2, 0x2); /* enable to set RTC to use external 32K frequency */
	sanremo_write(SANREMO_AUDIO_Side_Tone_1, 0x24); 
	sanremo_read(SANREMO_AUDIO_DAC_LO1_CTRL, &val); 
	sanremo_write(SANREMO_AUDIO_DAC_LO1_CTRL, val|0x60); 

	/*sleep mode setting*/
	sanremo_write(SANREMO_SLEEP_MODE1, 0x85);
	//sanremo_write(SANREMO_SLEEP_MODE2, 0xaa);
	//sanremo_write(SANREMO_SLEEP_MODE3, 0xa2);
	//sanremo_write(SANREMO_SLEEP_MODE4, 0x2a);
	sanremo_write(SANREMO_VBUCK1_SET_SLP, 0x24); //vbuck1 0.9v in sleep mode
	sanremo_write(SANREMO_VBUCK2_SET_SLP, 0x24);

	sanremo_write(SANREMO_LDO6, 0x1b); //set LDO6 to 2.65V for RF
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
	{0x40,	"sanremoA0",	sanremo_power_modules},
	{0x41,	"sanremoA1",	sanremo_power_modules},
	{0x48,	"sanremoB0",	sanremo_power_modules},
	{0,	NULL,		NULL},
};

static struct sanremo_touch_platform_data sanremo_touch_data = {
	.ts = {0, 3600, 0, 3800},
	.ts_revert = {-1, 0},
};

static struct sanremo_platform_data sanremo_data = {
	.init_irq = sanremo_init_irq,
	.ack_irq = sanremo_ack_irq,
	.platform_init = sanremo_init,
	.power_chips = sanremo_chips,
	.tsi = &sanremo_touch_data,
};
#endif /* CONFIG_PXA3xx_SANREMO || CONFIG_PXA3xx_SANREMO_MODULE*/

static struct i2c_pxa_platform_data i2c_info __initdata = {
	.use_pio		= 1,
	.get_ripc = get_RIPC,
	.release_ripc =  release_RIPC,
};

static struct pca953x_platform_data max7312_data[] = {
	[0] = {
		.gpio_base      = GPIO_EXT0(0),
	},
};

#if defined(CONFIG_GPIO_PCA9575)

/* GPIO expander PCA9575 */
static struct pca9575_platform_data pca9575_data[] = {
	[0] = {
		.gpio_base      = GPIO_EXT1(0),
	},
};
#endif

#if defined(CONFIG_RADIO_SI4703) || defined(CONFIG_RADIO_SI4703_MODULE)
int si4703_setup(struct i2c_client *client, void *context)
{
        int reset_pin = mfp_to_gpio(MFP_PIN_GPIO20);
        int irq_pin = mfp_to_gpio(MFP_PIN_GPIO19);

        gpio_request(reset_pin, "si4703 FM radio reset");
        gpio_request(irq_pin, "si4703 FM radio interrupt");

        /* clear GPIO96 edge detect */
      //  pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO19_FM_RDS_IRQ), MFP_EDGE_NONE);

        /* configure interrupt pin as input */
        gpio_direction_input(reset_pin);

        /* assert reset for 100 ms*/
        gpio_direction_output(reset_pin, 0);
        mdelay(100);

        /* deassert reset */
        gpio_set_value(reset_pin, 1);

        gpio_free(reset_pin);
        gpio_free(irq_pin);

        return 0;
}

static struct si4703_platform_data si4703_data = {
        .setup = si4703_setup,
};
#endif

#if defined(CONFIG_TOUCHSCREEN_TPO) || defined(CONFIG_TOUCHSCREEN_TOPCOM_MODULE)
static void __init top_touch_reset(void)
{
	int tpo_reset = mfp_to_gpio(MFP_PIN_GPIO46);

	if (gpio_request(tpo_reset, "tpo reset")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", tpo_reset);
		return -EIO;
	}
	gpio_direction_output(tpo_reset, 1);
	msleep(10);
        gpio_set_value(tpo_reset, 0);
	gpio_free(tpo_reset);
}
#endif

static struct i2c_board_info i2c_board_info[] =
{
#if defined(CONFIG_PORTOFINO) || defined(CONFIG_PORTOFINO_MODULE)
	{
		.type		= "portofino",
		.addr		= 0x11,
	},
#endif

#if defined(CONFIG_SANREMO) || defined(CONFIG_SANREMO_MODULE)
	{
		.type		= "sanremo",
		.addr		= 0x30,
		.platform_data	= &sanremo_data ,
		.irq		= IRQ_PXA168_PMIC_INT,
	},
	{
		.type		= "sanremo",
		.addr		= 0x34,
		.platform_data	= &sanremo_data ,
		.irq		= IRQ_PXA168_PMIC_INT,
	},
#endif
#if defined(CONFIG_PXA168_CAMERA)
        {
                .type           = "ov7670",
                .addr           = 0x21,
                .platform_data  = &ov7670_sensor_data,
        },
        {
                .type		= "ov3640",
                .addr           = 0x3C,
                .platform_data  = &ov3640_sensor_data,
        },
#endif
#if defined(CONFIG_TOUCHSCREEN_TPO) || defined(CONFIG_TOUCHSCREEN_TOPCOM_MODULE)
	{
		.type		= "tpo_touch",
		.addr		=  0x18,
		.irq		= gpio_to_irq(45),
	},
#endif
#if defined(CONFIG_RADIO_SI4703) || defined(CONFIG_RADIO_SI4703_MODULE)
        {
                .type           = "si4703",
                .addr           = 0x10,
                .platform_data  = &si4703_data,
                .irq            = IRQ_GPIO(mfp_to_gpio(MFP_PIN_GPIO19)),
        },
#endif

};

static struct i2c_board_info ttc_dkb_i2c_board_info[] =
{
	{
		.type           = "max7312",
		.addr           = 0x20,
		.irq            = IRQ_GPIO(80),
		.platform_data  = &max7312_data,
	},
};

static struct i2c_board_info td_dkb_i2c_board_info[] =
{
#if defined(CONFIG_GPIO_PCA9575)
	{
		.type           = "pca9575",
		.addr           = 0x20,
		.irq            = IRQ_GPIO(19),
		.platform_data  = &pca9575_data,
	},
#endif
};

static void (*spi_send)(struct pxa168fb_info *, void *, int , unsigned int );

#ifdef CONFIG_FB_PXA168
static u16 tpo_spi_cmdon[] = {
	0x0801,
	0x0800,
	0x0200,
	0x0304,
	0x040e,
	0x0903,
	0x0b18,
	0x0c53,
	0x0d01,
	0x0ee0,
	0x0f01,
	0x1058,
	0x201e,
	0x210a,
	0x220a,
	0x231e,
	0x2400,
	0x2532,
	0x2600,
	0x27ac,
	0x2904,
	0x2aa2,
	0x2b45,
	0x2c45,
	0x2d15,
	0x2e5a,
	0x2fff,
	0x306b,
	0x310d,
	0x3248,
	0x3382,
	0x34bd,
	0x35e7,
	0x3618,
	0x3794,
	0x3801,
	0x395d,
	0x3aae,
	0x3bff,
	0x07c9,		//auto power on
};

static u16 tpo_spi_cmdoff[] = {
	0x07d9,		//auto power off
};

static void tpo_lcd_power(struct pxa168fb_info *fbi, unsigned int spi_gpio_cs, unsigned int spi_gpio_reset, int on)
{
	int err = 0;
	/* turn on backlight on ttc dkb */
	if (machine_is_ttc_dkb()) {
		portofino_write(0x10,0x0e);
		portofino_write(0x15,0x02);
		portofino_write(0x16,0x10);
		portofino_write(0x00,0x0f);
		portofino_write(0x01,0x00);
		portofino_write(0x02,0x0f);
		portofino_write(0x03,0x29);
	}
	if (on) {
		if (spi_gpio_reset != -1) {
			err = gpio_request(spi_gpio_reset, "TPO_LCD_SPI_RESET");
			if (err) {
				printk("failed to request GPIO for TPO LCD RESET\n");
				return;
			}
			gpio_direction_output(spi_gpio_reset, 0);
			msleep(100);
			gpio_set_value(spi_gpio_reset, 1);
			msleep(100);
			gpio_free(spi_gpio_reset);
		}
		pxa168fb_spi_send(fbi, tpo_spi_cmdon, ARRAY_SIZE(tpo_spi_cmdon), spi_gpio_cs);
		spi_send(fbi, tpo_spi_cmdon, ARRAY_SIZE(tpo_spi_cmdon), spi_gpio_cs);
	} else 
		spi_send(fbi, tpo_spi_cmdoff, ARRAY_SIZE(tpo_spi_cmdoff), spi_gpio_cs);
}

static struct fb_videomode video_modes[] = {
	/* lpj032l001b HVGA mode info */
	[0] = {
		.pixclock       = 100000,
		.refresh        = 60,
		.xres           = 320,
		.yres           = 480,
		.hsync_len      = 10,
		.left_margin    = 15,
		.right_margin   = 10,
		.vsync_len      = 2,
		.upper_margin   = 4,
		.lower_margin   = 2,
		.sync           = 0,
	},
};

/* SPI Control Register. */
#define     CFG_SCLKCNT(div)                    (div<<24)  /* 0xFF~0x2 */
#define     CFG_RXBITS(rx)                      ((rx - 1)<<16)   /* 0x1F~0x1 */
#define     CFG_TXBITS(tx)                      ((tx - 1)<<8)    /* 0x1F~0x1, 0x1: 2bits ... 0x1F: 32bits */
#define     CFG_SPI_ENA(spi)                    (spi<<3)
#define     CFG_SPI_SEL(spi)                    (spi<<2)   /* 1: port1; 0: port0 */
#define     CFG_SPI_3W4WB(wire)                 (wire<<1)  /* 1: 3-wire; 0: 4-wire */

static struct pxa168fb_mach_info ttc_dkb_lcd_info __initdata = {
	.id                     = "Base",
	.modes                  = video_modes,
	.num_modes              = ARRAY_SIZE(video_modes),
	.pix_fmt                = PIX_FMT_RGB565,
	.io_pin_allocation_mode = PIN_MODE_DUMB_18_SPI,
	.dumb_mode              = DUMB_MODE_RGB666,
	.active                 = 1,
	.spi_ctrl               = CFG_SCLKCNT(16) | CFG_TXBITS(16) | CFG_SPI_SEL(1) | CFG_SPI_3W4WB(1) | CFG_SPI_ENA(1),
	.spi_gpio_cs            = -1,
	.spi_gpio_reset		= mfp_to_gpio(MFP_PIN_GPIO106),
	.panel_rbswap		= 1,
	.pxa168fb_lcd_power	= tpo_lcd_power,
	.invert_pixclock	= 1,
	.max_fb_size		= 1280 * 720 * 4,
};

static struct pxa168fb_mach_info ttc_dkb_lcd_ovly_info __initdata = {
	.id                     = "Ovly",
	.modes                  = video_modes,
	.num_modes              = ARRAY_SIZE(video_modes),
	.pix_fmt                = PIX_FMT_RGB565,
	.io_pin_allocation_mode = PIN_MODE_DUMB_18_SPI,
	.dumb_mode              = DUMB_MODE_RGB666,
	.active                 = 1,
        .spi_ctrl               = CFG_SCLKCNT(16) | CFG_TXBITS(16) | CFG_SPI_SEL(1) | CFG_SPI_3W4WB(1) | CFG_SPI_ENA(1),
        .spi_gpio_cs            = -1,
        .spi_gpio_reset         = mfp_to_gpio(MFP_PIN_GPIO106),
	.panel_rbswap		= 1,
        .pxa168fb_lcd_power     = tpo_lcd_power,
	.invert_pixclock	= 1,
	.max_fb_size		= 1280 * 720 * 4,
};

/* for oled panel */
static int oled_lcd = 0;
static int __init oled_lcd_setup(char *__unused)
{
	oled_lcd = 1;
	return 1;
}
__setup("oled_lcd", oled_lcd_setup);

inline int is_oled_lcd(void)
{
	return oled_lcd;
}

#define CMD1(x, x1)     (((x) << 8) | (x1))

static uint32_t oscillation[] = {
	CMD1(0x1E,0x00),
};

static uint32_t stand_by[] = {
	CMD1(0x1E,0x03),
};

static uint32_t power_on[] = {
	CMD1(0x12,0x11),
	CMD1(0x13,0x08),
	CMD1(0x14,0x25),
	CMD1(0x15,0x6A),
	CMD1(0x16,0x53),
	CMD1(0x17,0x06),
	CMD1(0x18,0x42),
	CMD1(0x19,0x51),
	CMD1(0x1B,0x21),	/* swap R&B for line connection for OLED panel */
	//        CMD1(0x1B,0x20),
};

static uint32_t boost_power_on[] = {
	CMD1(0x1A,0x01),
};

static uint32_t power_off[] = {
	CMD1(0x1A,0x00),
};

static uint32_t initialize[] = {
	CMD1(0x20,0x08),
	CMD1(0x21,0x16),
	CMD1(0x22,0x00),
	CMD1(0x23,0x28),
	CMD1(0x26,0x06),
	CMD1(0x24,0x00),
	CMD1(0x27,0x01),
	CMD1(0x25,0x01),
	CMD1(0x01,0x63),
	CMD1(0x02,0x46),
	CMD1(0x03,0x0F),
	CMD1(0x04,0x2C),
	CMD1(0x05,0x00),
	CMD1(0x06,0x00),
	CMD1(0x07,0x00),
	CMD1(0x08,0x00),
	CMD1(0x09,0x00),
	CMD1(0x0A,0x00),
	CMD1(0x0B,0x04),
	CMD1(0x0C,0x04),
	CMD1(0x0D,0x1E),
	CMD1(0x0E,0x1E),
	CMD1(0x0F,0x1E),
};
static uint32_t gamma_setting[] = {
	CMD1(0x2D,0x01),
	CMD1(0x30,0x19),
	CMD1(0x31,0x1A),
	CMD1(0x32,0x1F),
	CMD1(0x33,0x5E),
	CMD1(0x34,0x74),
	CMD1(0x35,0x70),
	CMD1(0x36,0x20),
	CMD1(0x37,0x1E),
	CMD1(0x38,0x1D),
	CMD1(0x39,0x13),
	CMD1(0x3A,0x11),
	CMD1(0x3B,0x0F),
	CMD1(0x3C,0x42),
	CMD1(0x3D,0x3C),
	CMD1(0x3E,0x36),
	CMD1(0x3F,0x49),
	CMD1(0x40,0x3F),
	CMD1(0x41,0x3A),
	CMD1(0x2D,0x00),
};
static uint32_t disp_on_ampon[]={
	CMD1(0x1C,0x01),
};
static uint32_t disp_on_elon[]={
	CMD1(0x1D,0x01),
};
static uint32_t disp_on_glson[]={
	CMD1(0x1D,0x03),
};
static uint32_t disp_on_elson[]={
	CMD1(0x1D,0x07),
};
static uint32_t disp_on_dispon[]={
	CMD1(0x1D,0x0F),
};

static uint32_t disp_off_dispon[]={
	CMD1(0x1D,0x07),
};
static uint32_t disp_off_elon[]={
	CMD1(0x1D,0x06),
};
static uint32_t disp_off_glson[]={
	CMD1(0x1D,0x00),
};
static uint32_t disp_off_ampon[]={
	CMD1(0x1C,0x00),
};

static void lcd_display_on(struct pxa168fb_info *fbi, unsigned int spi_gpio_cs)
{
	spi_send(fbi, disp_on_ampon, ARRAY_SIZE(disp_on_ampon), spi_gpio_cs);
	mdelay(10);
	spi_send(fbi, disp_on_elon, ARRAY_SIZE(disp_on_elon), spi_gpio_cs);
	mdelay(17);
	spi_send(fbi, disp_on_glson, ARRAY_SIZE(disp_on_glson), spi_gpio_cs);
	mdelay(17);
	spi_send(fbi, disp_on_elson, ARRAY_SIZE(disp_on_elson), spi_gpio_cs);
	mdelay(17);
	spi_send(fbi, disp_on_dispon, ARRAY_SIZE(disp_on_dispon), spi_gpio_cs);
	mdelay(17);
}
static void lcd_display_off(struct pxa168fb_info *fbi, unsigned int spi_gpio_cs)
{
	spi_send(fbi, disp_off_dispon, ARRAY_SIZE(disp_off_dispon), spi_gpio_cs);
	mdelay(17);
	spi_send(fbi, disp_off_elon, ARRAY_SIZE(disp_off_elon), spi_gpio_cs);
	mdelay(34);
	spi_send(fbi, disp_off_glson, ARRAY_SIZE(disp_off_glson), spi_gpio_cs);
	mdelay(17);
	spi_send(fbi, disp_off_ampon, ARRAY_SIZE(disp_off_ampon), spi_gpio_cs);
	mdelay(10);
}
static void benzina_lcd_power(struct pxa168fb_info *fbi, unsigned int spi_gpio_cs, unsigned int spi_gpio_reset, int on)
{
	int err = 0;
	if (on) {
		if (spi_gpio_reset != -1) {
			err = gpio_request(spi_gpio_reset, "TPO_LCD_SPI_RESET");
			if (err) {
				printk("failed to request GPIO for TPO LCD RESET\n");
				return;
			}
			msleep(10);
			printk("lcd power off ...\n");

			lcd_display_off(fbi, spi_gpio_cs);
			spi_send(fbi, power_off, ARRAY_SIZE(power_off), spi_gpio_cs);
			mdelay(20);
			spi_send(fbi, stand_by, ARRAY_SIZE(stand_by), spi_gpio_cs);
			mdelay(1000);

			printk("oled lcd power on ...\n");
			gpio_direction_output(spi_gpio_reset, 0);
			mdelay(100);
			gpio_direction_output(spi_gpio_reset, 1);
			mdelay(100);
			gpio_free(spi_gpio_reset);
		}

		spi_send(fbi, oscillation, ARRAY_SIZE(oscillation), spi_gpio_cs);
		spi_send(fbi, power_on, ARRAY_SIZE(power_on), spi_gpio_cs);
		spi_send(fbi, boost_power_on, ARRAY_SIZE(boost_power_on), spi_gpio_cs);
		mdelay(140);
		spi_send(fbi, initialize, ARRAY_SIZE(initialize), spi_gpio_cs);
		spi_send(fbi, gamma_setting, ARRAY_SIZE(gamma_setting), spi_gpio_cs);
		lcd_display_on(fbi, spi_gpio_cs);

	} else {
		printk("lcd power off ...\n");
		lcd_display_off(fbi, spi_gpio_cs);
		spi_send(fbi, power_off, ARRAY_SIZE(power_off), spi_gpio_cs);
		mdelay(10);
		spi_send(fbi, stand_by, ARRAY_SIZE(stand_by), spi_gpio_cs);
	}
}

static struct fb_videomode novatek_l1N304_modes[] = {
	[0] = {
		.pixclock               = 67833,
		.refresh        	= 60,
		.xres                   = 360,
		.yres                   = 640,
		.hsync_len              = 2,
		.left_margin            = 5,
		.right_margin           = 10,
		.vsync_len              = 2,
		.upper_margin           = 4,
		.lower_margin           = 3,
		.sync			= 0,
	},
};

static struct pxa168fb_mach_info benzina_lcd_info __initdata = {
	.id                     = "Base",
	.modes                  = novatek_l1N304_modes,
	.num_modes              = 1,
	.pix_fmt                = PIX_FMT_RGB565,
	.io_pin_allocation_mode = PIN_MODE_DUMB_18_SPI,
	.dumb_mode              = DUMB_MODE_RGB666,
	.active                 = 1,
	.invert_pix_val_ena	= 1,	/* DE pin - panel is active low while controll is active high */
	.spi_ctrl               = CFG_SCLKCNT(2) | CFG_TXBITS(17) | CFG_SPI_SEL(1) | CFG_SPI_3W4WB(1) | CFG_SPI_ENA(1),
	.spi_gpio_cs            = mfp_to_gpio(MFP_PIN_GPIO107),
	.spi_gpio_reset         = mfp_to_gpio(MFP_PIN_GPIO106),		/* GPIO106 is conflicted with 1WIRE */
	.pxa168fb_lcd_power     = benzina_lcd_power,
	.invert_pixclock        = 1,
};

static struct pxa168fb_mach_info benzina_lcd_ovly_info __initdata = {
	.id                     = "Ovly",
	.modes                  = novatek_l1N304_modes,
	.num_modes              = 1,
	.pix_fmt                = PIX_FMT_RGB565,
	.io_pin_allocation_mode = PIN_MODE_DUMB_18_SPI,
	.dumb_mode              = DUMB_MODE_RGB666,
	.active                 = 1,
};

#endif

static struct platform_device portofino_bl_device = {
	.name           = "portofino-bl",
	.id             = 0,
};

DECLARE_ANDROID_256M_V75_PARTITIONS(android_256m_v75_partitions);
DECLARE_256M_V75_PARTITIONS(generic_256m_v75_partitions);
static struct flash_platform_data ttc_dkb_onenand_info;
static struct pxa3xx_nand_platform_data ttc_dkb_nand_info;
static void __init ttc_dkb_add_flash(void)
{
	if (is_android()) {
		ttc_dkb_onenand_info.parts = android_256m_v75_partitions;
		ttc_dkb_onenand_info.nr_parts = ARRAY_SIZE(android_256m_v75_partitions);
		ttc_dkb_nand_info.parts[0] = android_256m_v75_partitions;
		ttc_dkb_nand_info.nr_parts[0] = ARRAY_SIZE(android_256m_v75_partitions);
	} else {
		ttc_dkb_onenand_info.parts = generic_256m_v75_partitions;
		ttc_dkb_onenand_info.nr_parts = ARRAY_SIZE(generic_256m_v75_partitions);
		ttc_dkb_nand_info.parts[0] = generic_256m_v75_partitions;
		ttc_dkb_nand_info.nr_parts[0] = ARRAY_SIZE(generic_256m_v75_partitions);
	}
	pxa168_add_onenand(&ttc_dkb_onenand_info);
	ttc_dkb_nand_info.use_dma = 0;
	ttc_dkb_nand_info.enable_arbiter = 1;
	pxa168_add_nand(&ttc_dkb_nand_info);
}

static unsigned int ttc_dkb_matrix_key_map[] = {
	KEY(0, 0, KEY_BACKSPACE),
	KEY(0, 1, KEY_END),
	KEY(0, 2, KEY_RIGHTCTRL),
	KEY(0, 3, KEY_0),
	KEY(0, 4, KEY_1),

	KEY(1, 0, KEY_MENU),
	KEY(1, 1, KEY_HOME),
	KEY(1, 2, KEY_SEND),
	KEY(1, 3, KEY_8),
	KEY(1, 4, KEY_9),

	KEY(2, 0, KEY_OK),
	KEY(2, 1, KEY_2),
	KEY(2, 2, KEY_3),
	KEY(2, 3, KEY_4),
	KEY(2, 4, KEY_5),

	KEY(3, 0, KEY_6),
	KEY(3, 1, KEY_VOLUMEUP),
	KEY(3, 2, KEY_7),
	KEY(3, 3, KEY_VOLUMEDOWN),
	KEY(3, 4, KEY_RECORD),

	KEY(4, 0, KEY_KPASTERISK),
	KEY(4, 1, KEY_KPDOT),
	KEY(4, 2, KEY_F2),
	KEY(4, 3, KEY_CAMERA),
	KEY(4, 4, KEY_CAMERA),

	KEY(6, 0, KEY_F1),
	KEY(6, 1, KEY_UP),
	KEY(6, 2, KEY_DOWN),
	KEY(6, 3, KEY_LEFT),
	KEY(6, 4, KEY_RIGHT),
};

static struct pxa27x_keypad_platform_data ttc_dkb_keypad_info __initdata = {
	.matrix_key_rows	= 7,
	.matrix_key_cols	= 5,
	.matrix_key_map		= ttc_dkb_matrix_key_map,
	.matrix_key_map_size	= ARRAY_SIZE(ttc_dkb_matrix_key_map),
	.debounce_interval	= 30,
};


#if defined(CONFIG_MMC_PXA_SDH)
static struct pxasdh_platform_data ttc_dkb_sdh_platform_data_0 = {
	.detect_delay	= 20,
	.ocr_mask	= MMC_VDD_27_28|MMC_VDD_28_29,
	.max_speed	= 12000000,
	//.sd_clock	= 1,
	//.sdclk_sel	= 0,
};

static struct pxasdh_platform_data ttc_dkb_sdh_platform_data_1 = {
	.detect_delay	= 20,
	.ocr_mask	= MMC_VDD_165_195,
};

static void __init ttc_dkb_init_mmc(void)
{
#ifdef CONFIG_SD8XXX_RFKILL
	int gpio_reset;
	int gpio_buck_en = mfp_to_gpio(WLAN_BT_RESET_GPIO_34);
	int gpio_lhc = mfp_to_gpio(WLAN_LHC_GPIO_36);

	if(cpu_is_pxa910_Ax()) {
		if (gpio_request(gpio_buck_en, "PG8211_BUCK2_EN2")) {
			printk(KERN_INFO "gpio %d request failed\n", gpio_buck_en);
			return -1;
		}

		if (gpio_request(gpio_lhc, "WLAN_LHC")) {
			printk(KERN_INFO "gpio %d request failed\n", gpio_lhc);
			return -1;
		}
		gpio_direction_output(gpio_buck_en, 1);
		gpio_direction_output(gpio_lhc, 1);
		gpio_free(gpio_buck_en);
		gpio_free(gpio_lhc);

		gpio_reset = mfp_to_gpio(WLAN_RESET_GPIO_20);
	} else {
		gpio_reset = mfp_to_gpio(WLAN_BT_RESET_GPIO_34);
	}

	add_sd8x_rfkill_device(mfp_to_gpio(WLAN_PD_GPIO_14), gpio_reset,
			&ttc_dkb_sdh_platform_data_1.pmmc);
#endif
	pxa168_add_sdh(0, &ttc_dkb_sdh_platform_data_0);
	pxa168_add_sdh(1, &ttc_dkb_sdh_platform_data_1);
}
#endif

#ifdef CONFIG_USB_GADGET_PXA_U2O
static int ttc_dkb_vbus_status(unsigned base)
{
	int status = VBUS_LOW;

	if (pxa3xx_pmic_is_vbus_assert()) {
		status = VBUS_HIGH;
	}
	return status;
}

static int ttc_dkb_vbus_detect(void *func, int enable)
{
	if (enable) {
		pmic_callback_register(PMIC_EVENT_USB, func);
		pxa3xx_pmic_set_pump(1);
	} else {
		/*don't disable pump for charging*/
		/*pxa3xx_pmic_set_pump(0); */
		pmic_callback_unregister(PMIC_EVENT_USB, func);
	}

	return 0;
}

static int ttc_dkb_usbid_detect(struct otg_transceiver *otg);
irqreturn_t ttc_dkb_usbid_handler(int irq, struct otg_transceiver *otg)
{
	if (ttc_dkb_usbid_detect(otg)) {
		/* USB_ID is HIGH, b-device */
		printk("%s id high\n", __func__);
	} else {
		/* USB_ID is LOW, a-device */
		printk("%s id low\n", __func__);
	}

	otg_interrupt(otg);
	return IRQ_HANDLED;
}

static int ttc_dkb_usbid_detect(struct otg_transceiver *otg)
{
	int gpio_usbid = mfp_to_gpio(MFP_PIN_GPIO44), ret;
	static int init_done;

	if (!init_done) {
		init_done = 1;
		gpio_direction_input(gpio_usbid);
		request_irq(IRQ_GPIO(gpio_usbid), ttc_dkb_usbid_handler,
			IRQF_DISABLED | IRQF_TRIGGER_RISING | 
			IRQF_TRIGGER_FALLING, "usbid", otg);
	}

	if (gpio_get_value(gpio_usbid))
		return 1;
	else 
		return 0;
}

static int ttc_dkb_usb_power(int enable)
{
	if (enable)
		pxa3xx_pmic_set_voltage(VCC_USB, 2900);
	else
		pxa3xx_pmic_set_voltage(VCC_USB, 0);
}

static struct pxa_usb_plat_info ttc_dkb_u2o_info = {
	.phy_init	= pxa168_usb_phy_init,
	.phy_deinit	= pxa168_usb_phy_deinit,
	.vbus_status	= ttc_dkb_vbus_status,
	.vbus_detect	= ttc_dkb_vbus_detect,
 	/* USB_ID connect with GPIO may cause chip damage
	.usbid_detect	= ttc_dkb_usbid_detect, */
 	/* ECN001 on DKB rev2.0 fix the usb power issue */
	.rely_on_vbus	= 1,
	/* workaround for IN endpoint hang issue if multiple queue */
	.in_single	= 1,
	/* FIXME touch LDO5 would cause system power from USB,
	 * which would casue system hang if unplug the cable
	.set_power	= ttc_dkb_usb_power, */
  	.is_otg		= 1,
};
#endif

static struct platform_device sensor_input_device = {
	.name = "sensor_input",
	.id   = -1,
};

static int ttc_dkb_is_ac_online(void)
{
	u8 tmp;

	return 0;
}

static int ttc_dkb_is_usb_online(void)
{
	u8 tmp;

	return sanremo_usb_connect();
}

static char *ttc_dkb_supplicants[] = {
	"battery",
};

static struct pda_power_pdata ttc_dkb_power_supply_info = {
	.is_ac_online   = ttc_dkb_is_ac_online,
	.is_usb_online   = ttc_dkb_is_usb_online,
	.supplied_to     = ttc_dkb_supplicants,
	.num_supplicants = ARRAY_SIZE(ttc_dkb_supplicants),
};

static struct resource ttc_dkb_power_supply_resources[] = {
	[0] = {
		.name  = "ac",
	},
	[1] = {
		.name  = "usb",
	},
};

static struct platform_device ttc_dkb_power_supply = {
	.name = "pda-power",
	.id   = -1,
	.dev  = {
	        .platform_data = &ttc_dkb_power_supply_info,
	},
	.resource      = ttc_dkb_power_supply_resources,
	.num_resources = ARRAY_SIZE(ttc_dkb_power_supply_resources),
};

static struct platform_device sanremo_battery = {
	.name = "sanremo_battery",
	.id   = -1,
};

static void __init ttc_dkb_init_power(void)
{
       platform_device_register(&ttc_dkb_power_supply);
       platform_device_register(&sanremo_battery);
}

static void (*headset_update_func)(int state);

static void sanremo_headset_interrupt(unsigned long event)
{
	if (headset_update_func)
		headset_update_func(sanremo_get_headset_state());
}

static int ttc_dkb_headset_detect(void *func, int enable)
{
	headset_update_func = func;
	return sanremo_enable_headset_detect(sanremo_headset_interrupt, enable);
}

static struct headset_switch_platform_data headset_switch_device_data = {
	.name = "h2w",
	.gpio = NULL,
	.name_on = NULL,
	.name_off = NULL,
	.state_on = NULL,
	.state_off = NULL,
	.enable_detect = ttc_dkb_headset_detect,
};

static struct platform_device headset_switch_device = {
	.name            = "headset",
	.id              = 0,
	.dev             = {
		.platform_data = &headset_switch_device_data,
	},
};

static void __init ttc_dkb_init_headset(void)
{
	platform_device_register(&headset_switch_device);
}

static int gps_power_on(void)
{
	int gps_ldo, gps_rst_n, gps_on;
	gps_ldo = GPIO_EXT1(8);
	if (gpio_request(gps_ldo, "gpio_gps_ldo")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", gps_ldo);
	}

	gps_on = GPIO_EXT1(10);
	if (gpio_request(gps_on, "gpio_gps_on")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", gps_on);
	}

	gps_rst_n = GPIO_EXT1(11);
	if (gpio_request(gps_rst_n, "gpio_gps_rst")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", gps_rst_n);
	}

	gpio_direction_output(gps_ldo, 0);
	gpio_direction_output(gps_rst_n, 0);
	gpio_direction_output(gps_on, 0);
	gpio_direction_output(gps_ldo, 1);
	gpio_direction_output(gps_rst_n, 1);
	mdelay(1);
	gpio_direction_output(gps_on, 1);

	gpio_free(gps_ldo);
	gpio_free(gps_on);
	gpio_free(gps_rst_n);
	printk(KERN_INFO "sirf gps chip (gsd3tw) powered on\n", __func__);
}

static int gps_power_off(void)
{
	int gps_ldo, gps_rst_n, gps_on;
	gps_ldo = GPIO_EXT1(8);
	if (gpio_request(gps_ldo, "gpio_gps_ldo")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", gps_ldo);
	}

	gps_on = GPIO_EXT1(10);
	if (gpio_request(gps_on, "gpio_gps_on")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", gps_on);
	}

	gps_rst_n = GPIO_EXT1(11);
	if (gpio_request(gps_rst_n, "gpio_gps_rst")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", gps_rst_n);
	}

	gpio_direction_output(gps_ldo, 0);
	gpio_direction_output(gps_rst_n, 0);
	gpio_direction_output(gps_on, 0);

	gpio_free(gps_ldo);
	gpio_free(gps_on);
	gpio_free(gps_rst_n);
	printk(KERN_INFO "sirf gps chip (gsd3tw) powered off\n", __func__);
}

static int gps_reset(int flag)
{
	int gps_rst_n;

	gps_rst_n = GPIO_EXT1(11);
	if (gpio_request(gps_rst_n, "gpio_gps_rst")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", gps_rst_n);
	}
	gpio_direction_output(gps_rst_n, flag);
	mdelay(1);

	gpio_free(gps_rst_n);
	printk(KERN_INFO "sirf gps chip (gsd3tw) reset\n", __func__);
}

static int gps_on_off(int flag)
{
	int gps_on;

	gps_on = GPIO_EXT1(10);
	if (gpio_request(gps_on, "gpio_gps_on")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", gps_on);
	}
	gpio_direction_output(gps_on, flag);
	mdelay(1);

	gpio_free(gps_on);
	printk(KERN_INFO "sirf gps chip (gsd3tw) offon\n", __func__);
}

#ifdef	CONFIG_PROC_FS
#define PROC_PRINT(fmt, args...) 	do {len += sprintf(page + len, fmt, ##args); } while(0)

static char sirf_status[4] = "off";
static ssize_t sirf_read_proc(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	int len = strlen(sirf_status);

	sprintf(page, "%s\n", sirf_status);
	return len + 1;
}

static ssize_t sirf_write_proc(struct file *filp,
		const char *buff, size_t len, loff_t *off)
{
	char messages[256];
	int flag, ret;
	char buffer[7];

	if (len > 256)
		len = 256;

	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	if (strncmp(messages, "off", 3) == 0) {
		strcpy(sirf_status, "off");
		gps_power_off();
	} else if (strncmp(messages, "on", 2) == 0) {
		strcpy(sirf_status, "on");
		gps_power_on();
	} else if (strncmp(messages, "reset", 5) == 0) {
		strcpy(sirf_status, messages);
		ret = sscanf(messages, "%s %d", buffer, &flag);
		if (ret == 2)
			gps_reset(flag);
	} else if (strncmp(messages, "sirfon", 5) == 0) {
		strcpy(sirf_status, messages);
		ret = sscanf(messages, "%s %d", buffer, &flag);
		if (ret == 2)
			gps_on_off(flag);
	} else {
		printk("usage: echo {on/off} > /proc/driver/sirf\n");
	}

	return len;
}

static void create_sirf_proc_file(void)
{
	struct proc_dir_entry *sirf_proc_file = 
		create_proc_entry("driver/sirf", 0644, NULL);

	if (sirf_proc_file) {
		sirf_proc_file->read_proc = sirf_read_proc;
		sirf_proc_file->write_proc = sirf_write_proc;
	} else 
		printk(KERN_INFO "proc file create failed!\n");
}
#endif

#if defined(CONFIG_SENSORS_LIS3LV02D_I2C) || defined(CONFIG_SENSORS_LIS3LV02D_I2C_MODULE)
static int lis3lv02d_direction_idx = 1;  
static struct platform_device g_sensor = {
	.name     = "lis3lv02d",
	.id           = -1,
	.dev = {
		.platform_data = &lis3lv02d_direction_idx,
	},
};
#endif

static void __init ttc_dkb_init(void)
{
	fake_suspend = 1; /*enable fake suspend*/

	/*dummy driver init*/
	platform_device_register(&sensor_input_device);

=======
};

static struct mtd_partition ttc_dkb_onenand_partitions[] = {
	{
		.name		= "bootloader",
		.offset		= 0,
		.size		= SZ_1M,
		.mask_flags	= MTD_WRITEABLE,
	}, {
		.name		= "reserved",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_128K,
		.mask_flags	= MTD_WRITEABLE,
	}, {
		.name		= "reserved",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_8M,
		.mask_flags	= MTD_WRITEABLE,
	}, {
		.name		= "kernel",
		.offset		= MTDPART_OFS_APPEND,
		.size		= (SZ_2M + SZ_1M),
		.mask_flags	= 0,
	}, {
		.name		= "filesystem",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_48M,
		.mask_flags	= 0,
	}
};

static struct onenand_platform_data ttc_dkb_onenand_info = {
	.parts		= ttc_dkb_onenand_partitions,
	.nr_parts	= ARRAY_SIZE(ttc_dkb_onenand_partitions),
};

static struct resource ttc_dkb_resource_onenand[] = {
	[0] = {
		.start	= SMC_CS0_PHYS_BASE,
		.end	= SMC_CS0_PHYS_BASE + SZ_1M,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device ttc_dkb_device_onenand = {
	.name		= "onenand-flash",
	.id		= -1,
	.resource	= ttc_dkb_resource_onenand,
	.num_resources	= ARRAY_SIZE(ttc_dkb_resource_onenand),
	.dev		= {
		.platform_data	= &ttc_dkb_onenand_info,
	},
};

static struct platform_device *ttc_dkb_devices[] = {
	&ttc_dkb_device_onenand,
};

static void __init ttc_dkb_init(void)
{
>>>>>>> e40152ee1e1c7a63f4777791863215e3faa37a86:arch/arm/mach-mmp/ttc_dkb.c
	mfp_config(ARRAY_AND_SIZE(ttc_dkb_pin_config));

	/* on-chip devices */
	pxa910_add_uart(1);
<<<<<<< HEAD:arch/arm/mach-mmp/ttc_dkb.c
	pxa910_add_uart(2);
	pxa910_add_uart(3);

#if defined(CONFIG_TOUCHSCREEN_TPO) || defined(CONFIG_TOUCHSCREEN_TOPCOM_MODULE)
	top_touch_reset();
#endif
	if(is_td_dkb)
		i2c_register_board_info(0, ARRAY_AND_SIZE(td_dkb_i2c_board_info));
	else
		i2c_register_board_info(0, ARRAY_AND_SIZE(ttc_dkb_i2c_board_info));
	pxa910_add_twsi(0, &i2c_info, ARRAY_AND_SIZE(i2c_board_info));

#ifdef CONFIG_USB_GADGET_PXA_U2O
	pxa168_add_u2o(&ttc_dkb_u2o_info);
#endif
#ifdef CONFIG_USB_OTG
	pxa168_add_u2ootg(&ttc_dkb_u2o_info);
	pxa168_add_u2oehci(&ttc_dkb_u2o_info);
#endif
	pxa910_add_acipc();
	pxa910_add_ire();
	pxa168_add_keypad(&ttc_dkb_keypad_info);
#ifdef CONFIG_FB_PXA910
	if(!cpu_is_pxa910_Ax()) {
		spi_send = pxa168fb_spi_send;
		if (is_oled_lcd()) {
			pxa168_add_fb(&benzina_lcd_info);
			pxa168_add_fb_ovly(&benzina_lcd_ovly_info);
		} else {
			pxa168_add_fb(&ttc_dkb_lcd_info);
			pxa168_add_fb_ovly(&ttc_dkb_lcd_ovly_info);
		}
	}else{
		spi_send = pxa910fb_spi_send;
		if (is_oled_lcd()) {
			pxa910_add_fb(&benzina_lcd_info);
			pxa910_add_fb_ovly(&benzina_lcd_ovly_info);
		} else {
			pxa910_add_fb(&ttc_dkb_lcd_info);
			pxa910_add_fb_ovly(&ttc_dkb_lcd_ovly_info);
		}
	}
#endif
	platform_device_register(&portofino_bl_device);
#if defined(CONFIG_PXA168_CAMERA)
	pxa168_add_cam();
#endif
	pxa910_add_ssp(1);
	pxa910_add_imm();
	pxa168_add_freq();

	ttc_dkb_add_flash();
#if defined(CONFIG_MMC_PXA_SDH)
	ttc_dkb_init_mmc();
#endif
	pxa910_add_rtc();
	/*power device*/
	ttc_dkb_init_power();

	/*headset device*/
	ttc_dkb_init_headset();

	/* off-chip devices */
	platform_device_register(&smc91x_device);

#if defined(CONFIG_SENSORS_LIS3LV02D_I2C) || defined(CONFIG_SENSORS_LIS3LV02D_I2C_MODULE)
	platform_device_register(&g_sensor);
#endif
	/* create proc for sirf control */
	create_sirf_proc_file();

	res_add_sanremo_vibrator();

	pm_power_off = sanremo_turn_off_power;
=======

	/* off-chip devices */
	platform_add_devices(ARRAY_AND_SIZE(ttc_dkb_devices));
>>>>>>> e40152ee1e1c7a63f4777791863215e3faa37a86:arch/arm/mach-mmp/ttc_dkb.c
}

MACHINE_START(TTC_DKB, "PXA910-based TTC_DKB Development Platform")
	.phys_io        = APB_PHYS_BASE,
	.boot_params    = 0x00000100,
	.io_pg_offst    = (APB_VIRT_BASE >> 18) & 0xfffc,
	.map_io		= pxa_map_io,
<<<<<<< HEAD:arch/arm/mach-mmp/ttc_dkb.c
	.init_irq       = pxa168_init_irq,
	.timer          = &pxa168_timer,
=======
	.init_irq       = pxa910_init_irq,
	.timer          = &pxa910_timer,
>>>>>>> e40152ee1e1c7a63f4777791863215e3faa37a86:arch/arm/mach-mmp/ttc_dkb.c
	.init_machine   = ttc_dkb_init,
MACHINE_END
