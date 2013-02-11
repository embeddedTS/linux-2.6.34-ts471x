/*
 *  linux/arch/arm/mach-mmp/aspenite.c
 *
 *  Support for the Marvell PXA168-based Aspenite and Zylonite2
 *  Development Platform.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
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
#include <linux/usb/otg.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/addr-map.h>
#include <mach/mfp-pxa168.h>
#include <mach/pxa168.h>
#include <mach/gpio.h>
#include <mach/max8660.h>
#include <mach/pxa3xx_nand.h>
#include <mach/camera.h>
#include <mach/pxa168_eth.h>
#include <mach/pxa168_pcie.h>
#include <mach/cputype.h>

#include <plat/part_table.h>
#include <plat/generic.h>
#include <plat/pxa3xx_pmic.h>
#include <plat/pxa_u2o.h>
#include <plat/pxa3xx_otg.h>

#if defined(CONFIG_SPI_PXA2XX)
#include <linux/spi/spi.h>
#include <plat/pxa2xx_spi.h>
#endif

#include "common.h"
#include <linux/mmc/sdhci.h>
#include <plat/pfn_cfg.h>

/*used by expander max7312, 16 pins gpio expander */
#define GPIO_EXT0(x)		(NR_BUILTIN_GPIO + (x))
#define GPIO_EXT1(x)		(NR_BUILTIN_GPIO + 16 + (x))
#define GPIO_EXT2(x)		(NR_BUILTIN_GPIO + 16 + 16 + (x))

#define CARD_EN GPIO_EXT1(0)
#define CAM_PWDN GPIO_EXT1(1)
#define TW9907_PWDN GPIO_EXT1(4)
#define TW9907_RST_N GPIO_EXT1(2)

static unsigned long aspenite_pin_config[] __initdata = {
	/* Data Flash Interface Nana or eMMC/eSD*/
#if defined(CONFIG_MMC3)
	GPIO16_SMC_nCS0_DIS,
#else
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
#else
	/* Static Memory Controller */
	GPIO18_SMC_nCS0,
	GPIO23_SMC_nLUA,
	GPIO25_SMC_nLLA,
	GPIO27_GPIO,	/* Ethernet IRQ */
	GPIO28_SMC_RDY,
	GPIO29_SMC_SCLK,
	GPIO34_SMC_nCS1,
	GPIO35_SMC_BE1,
	GPIO36_SMC_BE2,
#endif
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
	GPIO78_LCD_DD18,
	GPIO79_LCD_DD19,
	GPIO80_LCD_DD20,
	GPIO81_LCD_DD21,
	GPIO82_LCD_DD22,
	GPIO83_LCD_DD23,

	/* i2c bus */
	GPIO105_CI2C_SDA,
	GPIO106_CI2C_SCL,

#if !defined(CONFIG_MTD_M25P80)
	/* UART1 */
	GPIO107_UART1_RXD,
	GPIO108_UART1_TXD,

	/* Keypad */
	GPIO110_KP_MKIN0,
	GPIO109_KP_MKIN1,
	GPIO121_KP_MKIN4,
	GPIO111_KP_MKOUT7,
	GPIO112_KP_MKOUT6,
#else
	/* SSP1 */
	GPIO107_SPI_NOR_RXD,
	GPIO108_SPI_NOR_TXD,
	GPIO109_SPI_NOR_SYSCLK,
	GPIO110_GPIO,
	GPIO111_SPI_NOR_CLK,
#endif

	/* SSP0 */
	GPIO113_I2S_MCLK,
	GPIO114_I2S_FRM,
	GPIO115_I2S_BCLK,
	GPIO116_I2S_RXD,
	GPIO117_I2S_TXD,

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
	GPIO84_MSP_CD,

};

struct platform_device aspenite_device_battery = {
	.name		= "aspenite-battery",
	.id		= -1,
};

static inline void aspenite_add_battery(void)
{
	int ret;
        ret = platform_device_register(&aspenite_device_battery);
	if (ret)
		dev_err(&aspenite_device_battery.dev,
			"unable to register device: %d\n", ret);
}

static struct smc91x_platdata zylonite2_smc91x_info = {
	.flags	= SMC91X_USE_16BIT | SMC91X_NOWAIT,
};

static struct resource smc91x_resources[] = {
	[0] = {
		.start	= SMC_CS1_PHYS_BASE + 0x300,
		.end	= SMC_CS1_PHYS_BASE + 0xfffff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= gpio_to_irq(27),
		.end	= gpio_to_irq(27),
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
	}
};

static struct platform_device smc91x_device = {
	.name		= "smc91x",
	.id		= 0,
	.dev		= {
		.platform_data = &zylonite2_smc91x_info,
	},
	.num_resources	= ARRAY_SIZE(smc91x_resources),
	.resource	= smc91x_resources,
};

#define ENET_RESET_N GPIO_EXT1(9)
#define ENET_COMA_N GPIO_EXT1(10)

static int pxa168_eth_init(void) 
{
	if (cpu_is_pxa168_A0()) {
		printk(KERN_ERR "FE for pxa168 A0 not supported\n");
		return -EIO;
	}

	if (gpio_request(ENET_RESET_N, "ENET_RESET_N")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", ENET_RESET_N);
		return -EIO;
	}

	if (gpio_request(ENET_COMA_N, "ENET_COMA_N")){
		gpio_free(ENET_RESET_N);
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d\n", ENET_COMA_N);
		return -EIO;
	}

	gpio_direction_output(ENET_RESET_N, 1);
	gpio_direction_output(ENET_COMA_N, 1);

	gpio_free(ENET_RESET_N);
	gpio_free(ENET_COMA_N);

	return 0;
}

static struct pxa168_eth_platform_data pxa168_eth_data = {
	.phy_addr 	= 0, 	/* phy addr depends on boards */
	.force_phy_addr	= 1,
	.init		= pxa168_eth_init,
};

#if defined(CONFIG_PCI)



/* #define ASPENITE_REV5 */
#if !defined(ASPENITE_REV5)

#define PCIE_MINICARD_CD_N      GPIO_EXT2(0)
#define PCIE_1P5V_SHDN_N        GPIO_EXT2(1)
#define PCIE_3P3V_SHDN_N        GPIO_EXT2(2)
#define PCIE_MINICARD_PERST_N   GPIO_EXT2(3)
#define PCIE_MINICARD_WAKE_N    GPIO_EXT2(4)
#define PCIE_MINICARD_CLKREQ_N  GPIO_EXT2(5)
#define PCIE_REFCLK_OE          GPIO_EXT2(6)

#define PCIE_CARD_DECTECT       GPIO_EXT2(0)


static unsigned int pcie_card_inserted(void)
{
	int ret = 0;
	int res;

	res = gpio_get_value_cansleep(PCIE_CARD_DECTECT);
	if (!(res < 0)) {

		/*
		 * Check input reg (0x00) bit 0
		 *    0 = card is inserted
		 *    1 = card is not inserted
		 */
		ret = !res;
	}

	return ret;
}

int pxa168_gpio_pcie_init(void)
{
	/* Card inserted? */
	if (!pcie_card_inserted()) {
		printk(KERN_ERR "pcie: No card detected.\n");
		return -EIO;
	}

	if (gpio_request(PCIE_1P5V_SHDN_N, "PCIE_1P5V_SHDN_N")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", PCIE_1P5V_SHDN_N);
		return -EIO;
	}

	if (gpio_request(PCIE_3P3V_SHDN_N, "PCIE_3P3V_SHDN_N")) {
		gpio_free(PCIE_1P5V_SHDN_N);
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d\n", PCIE_3P3V_SHDN_N);
		return -EIO;
	}

	if (gpio_request(PCIE_REFCLK_OE, "PCIE_REFCLK_OE")) {
		gpio_free(PCIE_1P5V_SHDN_N);
		gpio_free(PCIE_3P3V_SHDN_N);
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d\n", PCIE_REFCLK_OE);
		return -EIO;
	}

	if (gpio_request(PCIE_MINICARD_PERST_N, "PCIE_MINICARD_PERST_N")) {
		gpio_free(PCIE_1P5V_SHDN_N);
		gpio_free(PCIE_3P3V_SHDN_N);
		gpio_free(PCIE_REFCLK_OE);
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d\n", PCIE_MINICARD_PERST_N);
		return -EIO;
	}

	if (gpio_direction_output(PCIE_MINICARD_PERST_N, 0))
		return -EIO;
	if (gpio_direction_output(PCIE_1P5V_SHDN_N, 1))
		return -EIO;
	if (gpio_direction_output(PCIE_1P5V_SHDN_N, 1))
		return -EIO;
	if (gpio_direction_output(PCIE_3P3V_SHDN_N, 1))
		return -EIO;
	/* wait for power supply to stabilize */
	mdelay(2);
	if (gpio_direction_output(PCIE_REFCLK_OE, 1))
		return -EIO;
	/* 4ms: PCIClock output to stabilize +
	 * 96ms: Tpvperl pr PCISIG Base1.0a design checklist
	 */
	mdelay(100);
	if (gpio_direction_output(PCIE_MINICARD_PERST_N, 1))
		return -EIO;

	gpio_free(PCIE_1P5V_SHDN_N);
	gpio_free(PCIE_3P3V_SHDN_N);
	gpio_free(PCIE_REFCLK_OE);
	gpio_free(PCIE_MINICARD_PERST_N);

	return 0;
}

#else /* Rev 5 */

#define PCIE_3P3V_SHDN_N   GPIO_EXT2(2)
#define PCIE_PRSNT2_N      GPIO_EXT2(3)
#define PCIE_WAKE_N        GPIO_EXT2(4)
#define PCIE_PWRGD         GPIO_EXT2(5)
#define PCIE_REFCLK_OE     GPIO_EXT2(6)

int pxa168_gpio_pcie_init(void)
{

	if (gpio_request(PCIE_3P3V_SHDN_N, "PCIE_3P3V_SHDN_N")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", PCIE_3P3V_SHDN_N);
		return -EIO;
	}

	if (gpio_request(PCIE_PWRGD, "PCIE_PWRGD")) {
		gpio_free(PCIE_3P3V_SHDN_N);
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", PCIE_PWRGD);
		return -EIO;
	}

	if (gpio_request(PCIE_REFCLK_OE, "PCIE_REFCLK_OE")) {
		gpio_free(PCIE_3P3V_SHDN_N);
		gpio_free(PCIE_PWRGD);
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d\n", PCIE_REFCLK_OE);
		return -EIO;
	}

	gpio_direction_output(PCIE_3P3V_SHDN_N, 1);
	mdelay(2);
	gpio_direction_output(PCIE_PWRGD, 1);
	mdelay(2);
	gpio_direction_output(PCIE_REFCLK_OE, 1);
	mdelay(100);

	gpio_free(PCIE_3P3V_SHDN_N);
	gpio_free(PCIE_PWRGD);
	gpio_free(PCIE_REFCLK_OE);

	return 0;
}
#endif

static struct pxa168_pcie_platform_data pxa168_pcie_data = {
	.init		= pxa168_gpio_pcie_init,
};
#endif

#if defined(CONFIG_PXA168_CF) && defined(CONFIG_PXA168_CF_USE_GPIO_CARDDETECT)
static struct resource pxa168_cf_resources[] = {
	[0] = {
		.start  = 0xD4285000,
		.end    = 0xD4285800,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start  = IRQ_PXA168_CF,
		.end    = IRQ_PXA168_CF,
		.flags  = IORESOURCE_IRQ,
	},
	[2] = {
		.start  = IRQ_GPIO(32),
		.end    = IRQ_GPIO(32),
		.flags  = IORESOURCE_IRQ,
	}
};

static struct platform_device pxa168_cf_device = {
	.name		= "pxa168-cf",
	.id		= -1,
	.resource	= pxa168_cf_resources,
	.num_resources	= ARRAY_SIZE(pxa168_cf_resources),
};

static void __init pxa168_cf_init(void)
{
	platform_device_register(&pxa168_cf_device);
}
#endif

/*
 * mfp is shared in card, cam and tw9907, only one is effective
 */
typedef enum{
	SW_CARD    = 0x01,
	SW_CAM_ON  = 0x02,
	SW_CAM_OFF = 0x03,
	SW_TW9907  = 0x04,
} SW_TYPE_T;

static int aspenite_pinmux_switch(SW_TYPE_T type)
{
	int ret = 0;

	if (gpio_request(CARD_EN, "CARD_EN")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", CARD_EN);
		return -EIO;
	}

	if (gpio_request(CAM_PWDN, "CAM_PWDN")) {
		gpio_free(CARD_EN);
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d\n", CAM_PWDN);
		return -EIO;
	}

	if (gpio_request(TW9907_PWDN, "TW9907_PWDN")) {
		gpio_free(CARD_EN);
		gpio_free(CAM_PWDN);
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d\n", TW9907_PWDN);
		return -EIO;
	}

	switch (type) {
	case SW_CARD:
		gpio_direction_output(CARD_EN, 1);
		gpio_direction_output(CAM_PWDN, 1);
		gpio_direction_output(TW9907_PWDN, 1);
		break;
	case SW_CAM_ON:
		gpio_direction_output(CARD_EN, 0);
		gpio_direction_output(CAM_PWDN, 0);
		gpio_direction_output(TW9907_PWDN, 1);
		break;
	case SW_CAM_OFF:
		gpio_direction_output(CARD_EN, 0);
		gpio_direction_output(CAM_PWDN, 1);
		gpio_direction_output(TW9907_PWDN, 1);
		break;
	case SW_TW9907:
		gpio_direction_output(CARD_EN, 0);
		gpio_direction_output(CAM_PWDN, 1);
		gpio_direction_output(TW9907_PWDN, 0);
		break;
	default:
		ret = -EIO;
		break;
	}

	gpio_free(CARD_EN);
	gpio_free(CAM_PWDN);
	gpio_free(TW9907_PWDN);

	return ret;
}

#if defined(CONFIG_PXA168_MSP)
/* msp platform data */
static mfp_cfg_t mfp_cfg_msp[]  = {
	GPIO40_MSP_DAT1,
	GPIO41_MSP_DAT0,
	GPIO43_MSP_DAT2,
	GPIO44_MSP_DAT3,
	GPIO42_MSP_BS,
	GPIO50_MSP_SCLK,
};

static int mspro_mfp_config(void)
{
	int ret = 0;

	ret = aspenite_pinmux_switch(SW_CARD);
	if (0 == ret)
		mfp_config(ARRAY_AND_SIZE(mfp_cfg_msp));

	return ret;
}

static struct card_platform_data msp_ops = {
	/* GPIO84 used as mspro detect pin */
	.pin_detect		= MFP_PIN_GPIO84,
	.mfp_config		= mspro_mfp_config,
};
#endif

#if defined(CONFIG_PXA168_CAMERA)
static mfp_cfg_t aspenite_cam_pins[] = {
	GPIO37_CAM_DAT7,
	GPIO38_CAM_DAT6,
	GPIO39_CAM_DAT5,
	GPIO40_CAM_DAT4,
	GPIO41_CAM_DAT3,
	GPIO42_CAM_DAT2,
	GPIO43_CAM_DAT1,
	GPIO44_CAM_DAT0,
	GPIO46_CAM_VSYNC,
	GPIO48_CAM_HSYNC,
	GPIO54_CAM_MCLK,
	GPIO55_CAM_PCLK,
};

/* sensor init */
static int sensor_power_onoff(int on, int id)
{
	/*
	 * on, 1, power on
	 * on, 0, power off
	 */
	int ret = 0;
	if(on){
		ret = aspenite_pinmux_switch(SW_CAM_ON);
	        if (0 == ret)
			mfp_config(ARRAY_AND_SIZE(aspenite_cam_pins));
	}else{
		ret = aspenite_pinmux_switch(SW_CAM_OFF);
	}
	return ret;
}

static struct sensor_platform_data ov7670_sensor_data = {
	.id = SENSOR_LOW,
	.power_on = sensor_power_onoff,
};

/* sensor init over */
#endif

static struct fb_videomode video_modes[] = {
	/* sharp_ls037 QVGA mode info */
	[0] = {
		.pixclock	= 158000,
		.refresh	= 60,
		.xres		= 240,
		.yres		= 320,
		.hsync_len	= 4,
		.left_margin	= 39,
		.right_margin	= 39,
		.vsync_len	= 1,
		.upper_margin	= 2,
		.lower_margin	= 3,
		.sync		= 0,
	},
	/* sharp_ls037 VGA mode info */
	[1] = {
		.pixclock	= 39700,
		.refresh	= 60,
		.xres		= 480,
		.yres		= 640,
		.hsync_len	= 8,
		.left_margin	= 81,
		.right_margin	= 81,
		.vsync_len	= 1,
		.upper_margin	= 2,
		.lower_margin	= 7,
		.sync		= 0,
	},
};

static struct pxa168fb_mach_info zylonite2_lcd_info __initdata = {
	.id			= "Base",
	.modes			= video_modes,
	.num_modes		= ARRAY_SIZE(video_modes),
	.pix_fmt		= PIX_FMT_RGB565,
	.io_pin_allocation_mode	= PIN_MODE_DUMB_16_GPIO,
	.dumb_mode		= DUMB_MODE_RGB565,
	.active			= 1,
	.panel_rbswap		= 1,
};

static struct i2c_pxa_platform_data pwri2c_info __initdata = {
	.use_pio		= 1,
};

static u16 tpo_spi_cmdon[] = {
	0x080F,
	0x0C5F,
	0x1017,
	0x1420,
	0x1808,
	0x1c20,
	0x2020,
	0x2420,
	0x2820,
	0x2c20,
	0x3020,
	0x3420,
	0x3810,
	0x3c10,
	0x4010,
	0x4415,
	0x48aa,
	0x4cff,
	0x5086,
	0x548d,
	0x58d4,
	0x5cfb,
	0x602e,
	0x645a,
	0x6889,
	0x6cfe,
	0x705a,
	0x749b,
	0x78c5,
	0x7cff,
	0x80f0,
	0x84f0,
	0x8808,
};

static u16 tpo_spi_cmdoff[] = {
	0x0c5e,		//standby
};

static void tpo_lcd_power(struct pxa168fb_info *fbi, unsigned int spi_gpio_cs, unsigned int spi_gpio_reset, int on)
{
	int err = 0;
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

		pxa168fb_spi_send(fbi, tpo_spi_cmdon,
				ARRAY_SIZE(tpo_spi_cmdon),
				spi_gpio_cs);

		/* Put backlight back to default brightness */
		pxa3xx_pmic_set_voltage(VCC_MISC2, 1800);

	} else {

		/* reset below not needed at this time because we
		 * actually remove power from the display
		 */

		/* put backlight to off to hide startup garbage */

		pxa3xx_pmic_set_voltage(VCC_MISC2, 3300);
	}
}

static struct fb_videomode video_modes_aspen[] = {
	/* lpj032l001b HVGA mode info */
	[0] = {
		.pixclock       = 30120,
		.refresh        = 60,
		.xres           = 800,
		.yres           = 480,
		.hsync_len      = 1,
		.left_margin    = 215,
		.right_margin   = 40,
		.vsync_len      = 1,
		.upper_margin   = 34,
		.lower_margin   = 10,
		.sync           = FB_SYNC_VERT_HIGH_ACT | FB_SYNC_HOR_HIGH_ACT,
	},

        [1] = {
                .pixclock       = 16129,
                .refresh        = 60,
                .xres           = 1024,
                .yres           = 768,
                .hsync_len      = 136,
                .left_margin    = 160,
                .right_margin   = 24,
                .vsync_len      = 6,
                .upper_margin   = 29,
                .lower_margin   = 3,
                .sync           = 0, 
        },

        [2] = {
                .pixclock       = 25641,
                .refresh        = 60,
                .xres           = 800,
                .yres           = 600,
                .hsync_len      = 128,
                .left_margin    = 88,
                .right_margin   = 40,
                .vsync_len      = 4,
                .upper_margin   = 23,
                .lower_margin   = 1,
                .sync           = 0, 
        },

        [3] = {
                .pixclock       = 111111,
                .refresh        = 60,
                .xres           = 480,
                .yres           = 272,
                .hsync_len      = 1,
                .left_margin    = 43,
                .right_margin   = 2,
                .vsync_len      = 1,
                .upper_margin   = 12,
                .lower_margin   = 2,
                .sync           = 0, 
        },

        [4] = {
                .pixclock       = 16129,
                .refresh        = 60,
                .xres           = 1280,
                .yres           = 720,
                .hsync_len      = 40,
                .left_margin    = 220,
                .right_margin   = 110,
                .vsync_len      = 5,
                .upper_margin   = 20,
                .lower_margin   = 5,
                .sync           = 0, 
        },

        [5] = {
                .pixclock       = 39722,
                .refresh        = 60,
                .xres           = 640,
                .yres           = 480,
                .hsync_len      = 96,
                .left_margin    = 40,
                .right_margin   = 8,
                .vsync_len      = 2,
                .upper_margin   = 25,
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

struct pxa168fb_mach_info aspenite_lcd_info __initdata = {
	.id                     = "Base-aspen",
	.modes                  = video_modes_aspen,
	.num_modes              = ARRAY_SIZE(video_modes_aspen),
	.pix_fmt                = PIX_FMT_RGB565,
	.io_pin_allocation_mode = PIN_MODE_DUMB_24,
	.dumb_mode              = DUMB_MODE_RGB888,
	.active                 = 1,
	.spi_ctrl		= CFG_SCLKCNT(2) | CFG_TXBITS(16) | CFG_SPI_SEL(1) | CFG_SPI_3W4WB(1) | CFG_SPI_ENA(1),
	.spi_gpio_cs		= GPIO_EXT1(14),
	.spi_gpio_reset         = -1,
	.panel_rbswap		= 1,
	.invert_pixclock	= 1,
	.pxa168fb_lcd_power     = tpo_lcd_power,
	.max_fb_size		= 1024 * 768 * 4 * 2,
};

struct pxa168fb_mach_info aspenite_lcd_ovly_info __initdata = {
        .id                     = "Ovly-aspen",
        .modes                  = video_modes_aspen,
        .num_modes              = ARRAY_SIZE(video_modes_aspen),
        .pix_fmt                = PIX_FMT_RGB565,
        .io_pin_allocation_mode = PIN_MODE_DUMB_24,
        .dumb_mode              = DUMB_MODE_RGB888,
        .active                 = 1,
	.panel_rbswap		= 1,
	.max_fb_size		= 1024 * 768 * 4 * 2,
};

#if defined(CONFIG_GPIO_PCA953X)
static int stamp_8688_wlan_poweron(void);

/* GPIO expander max7312 could reuse PCA953X */

static struct pca953x_platform_data max7312_data[] = {
	/* three max7312 in system */

	[0] = {
		.gpio_base      = GPIO_EXT0(0),
	},

	[1] = {
		.gpio_base      = GPIO_EXT1(0),
#if defined(CONFIG_WLAN_8688_SDIO)
		.poweron = stamp_8688_wlan_poweron,
#endif
	},
#if defined(CONFIG_PCI)
	/* GPIO expander #3 */
	[2] = {
		.gpio_base      = GPIO_EXT2(0),
		.invert         = 0,
	},
#endif
};

#endif

static struct i2c_board_info aspenite_i2c_board_info[] = {

#if defined(CONFIG_GPIO_PCA953X)
	{
		.type           = "max7312",
		.addr           = 0x10,                 /* 0x20/0x21 */
		.irq            = IRQ_GPIO(122),
		.platform_data  = &max7312_data[0],
	},

	{
		.type           = "max7312",
		.addr           = 0x20,                 /* 0x40/0x41 */
		.irq            = IRQ_GPIO(120),
		.platform_data  = &max7312_data[1],
	},

#endif
#if defined(CONFIG_PCI)
	{
		.type           = "max7312",
		.addr           = 0x28,  		/* 0x50/0x51 */
		.platform_data  = &max7312_data[2],
	},
#endif
#if defined(CONFIG_PXA168_CAMERA)
	{
		.type		= "ov7670",
		.addr           = 0x21,
		.platform_data  = &ov7670_sensor_data,
	},
#endif
#if defined(CONFIG_RTC_DRV_ISL1208)
	{
		.type		= "isl1208",
		.addr           = 0x6f,
	},
#endif
};

#if defined(CONFIG_MAX8660)
static void max8660_init(void)
{
        /* There was 0.2 voltage decline on V3 according to the spec setting, the real voltage is 1.1v;
	 * use hardware default value on aspenite to high voltage damage the board;
	 * pxa3xx_pmic_set_voltage(VCC_CORE, 1300); 
	 */
	pxa3xx_pmic_set_voltage(HDMI_1P2V, 1200);
	pxa3xx_pmic_set_voltage(VCC_MVT, 1800);
	pxa3xx_pmic_set_voltage(VCC_MISC1, 3000); 	/* SD voltage */
	pxa3xx_pmic_set_voltage(VCC_MISC2, 1800);	/* Backlight control */
}

/* max8660_power_module[] should be consistent with enum
 * in include/asm-arm/arch-pxa/pxa3xx_pmic.h 
 */
static struct power_supply_module max8660_power_modules[] = {
        /* {command,            power_module}, */
        {VCC_CORE,              MAX8660_V3},
        {HDMI_1P2V,             MAX8660_V4},
        {VCC_MVT,               MAX8660_V5},
        {VCC_MISC1,             MAX8660_V6},
        {VCC_MISC2,           	MAX8660_V7},
        {0,                     0},
};

static struct power_chip max8660_chips[] = {
        {MAX8660_ID,  "max8660",      max8660_power_modules},
        {0,     NULL,           NULL},
};

static struct max8660_platform_data max8660_data = {
        .platform_init = max8660_init,
        .power_chips = max8660_chips,
};
#endif

static struct i2c_board_info pwri2c_board_info[] =
{
#if defined(CONFIG_MAX8660)
	{
		.type	= "max8660",
		.addr	= 0x34,
		.platform_data = &max8660_data,
	},
#endif

#if defined(CONFIG_TSC2007)
       {
	       .type	= "tsc2007",
	       .addr	= 0x48,                                 /* 0x90/0x91 */
	       .irq	= IRQ_GPIO(GPIO_EXT0(7)),               /* IO7 of TSC2007 */
       },
#endif

};

static unsigned int aspenite_matrix_key_map[] = {
	KEY(0, 7, KEY_LEFT),
	KEY(4, 7, KEY_RIGHT),
	KEY(0, 6, KEY_HOME),
	KEY(4, 6, KEY_END),
	KEY(1, 7, KEY_ENTER),	/* keypad action */
	KEY(1, 6, KEY_SEND),
};

static unsigned int aspenite_android_matrix_key_map[] = {
	KEY(0, 6, KEY_UP),	/* SW 4 */
	KEY(0, 7, KEY_DOWN),	/* SW 5 */
	KEY(1, 6, KEY_LEFT),	/* SW 6 */
	KEY(1, 7, KEY_RIGHT),	/* SW 7 */
	KEY(4, 6, KEY_MENU),	/* SW 8 */
	KEY(4, 7, KEY_BACK),	/* SW 9 */
};

static struct pxa27x_keypad_platform_data aspenite_keypad_info __initdata = {
	.matrix_key_rows	= 8,
	.matrix_key_cols	= 8,
	.matrix_key_map		= aspenite_matrix_key_map,
	.matrix_key_map_size	= ARRAY_SIZE(aspenite_matrix_key_map),
	.debounce_interval	= 30,
};

static struct pxa27x_keypad_platform_data aspenite_android_keypad_info __initdata = {
	.matrix_key_rows	= 8,
	.matrix_key_cols	= 8,
	.matrix_key_map		= aspenite_android_matrix_key_map,
	.matrix_key_map_size	= ARRAY_SIZE(aspenite_android_matrix_key_map),
	.debounce_interval	= 30,
};

#if (defined(CONFIG_SPI_PXA2XX) || defined(CONFIG_SPI_PXA2XX_MODULE)) \
	&& defined(CONFIG_MTD_M25P80)

static struct pxa2xx_spi_master pxa_ssp_master_info = {
	.num_chipselect	= 1,
	.enable_dma = 1,
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

static void __init aspenite_init_spi(void)
{
	pxa168_add_ssp(1);
	pxa168_add_spi(2, &pxa_ssp_master_info);
	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
}
#else
static inline void aspenite_init_spi(void) {}
#endif

#if defined(CONFIG_SAMSUNG_32G_MLC_NAND)

DECLARE_ANDROID_32G_V75_PARTITIONS(android_32G_v75_partitions);
DECLARE_32G_V75_PARTITIONS(generic_32G_v75_partitions);

#elif defined(CONFIG_MICRON_32G_MLC_NAND)

DECLARE_ANDROID_32G_V75_PARTITIONS(android_32G_v75_partitions);
DECLARE_32G_V75_PARTITIONS(generic_32G_v75_partitions);

#elif defined(CONFIG_SAMSUNG_8G_MLC_NAND)

DECLARE_ANDROID_8G_V75_PARTITIONS(android_8G_v75_partitions);
DECLARE_8G_V75_PARTITIONS(generic_8G_v75_partitions);

#elif defined(CONFIG_SAMSUNG_512M_SLC_NAND)

DECLARE_64M_V75_PARTITIONS(generic_64M_v75_partitions);


#else

DECLARE_ANDROID_512M_V75_PARTITIONS(android_512m_v75_partitions);
DECLARE_512M_V75_PARTITIONS(generic_512m_v75_partitions);

#endif

static struct pxa3xx_nand_platform_data aspenite_nand_info;

#if defined(CONFIG_SAMSUNG_32G_MLC_NAND)

static void __init aspenite_add_nand(void)
{
        if (is_android()) {
                aspenite_nand_info.parts[0] = android_32G_v75_partitions;
                aspenite_nand_info.nr_parts[0] =
                        ARRAY_SIZE(android_32G_v75_partitions);
        } else {
                aspenite_nand_info.parts[0] = generic_32G_v75_partitions;
                aspenite_nand_info.nr_parts[0] =
                        ARRAY_SIZE(generic_32G_v75_partitions);
        }

        aspenite_nand_info.use_dma = 0;
        aspenite_nand_info.enable_arbiter = 1;
        pxa168_add_nand((struct flash_platform_data *) &aspenite_nand_info);
}

#elif defined(CONFIG_MICRON_32G_MLC_NAND)

static void __init aspenite_add_nand(void)
{
        if (is_android()) {
                aspenite_nand_info.parts[0] = android_32G_v75_partitions;
                aspenite_nand_info.nr_parts[0] =
                        ARRAY_SIZE(android_32G_v75_partitions);
        } else {
                aspenite_nand_info.parts[0] = generic_32G_v75_partitions;
                aspenite_nand_info.nr_parts[0] =
                        ARRAY_SIZE(generic_32G_v75_partitions);
        }

        aspenite_nand_info.use_dma = 0;
        aspenite_nand_info.enable_arbiter = 1;
        pxa168_add_nand((struct flash_platform_data *) &aspenite_nand_info);
}

#elif defined(CONFIG_SAMSUNG_8G_MLC_NAND)

static void __init aspenite_add_nand(void)
{
        if (is_android()) {
                aspenite_nand_info.parts[0] = android_8G_v75_partitions;
                aspenite_nand_info.nr_parts[0] =
                        ARRAY_SIZE(android_8G_v75_partitions);
        } else {
                aspenite_nand_info.parts[0] = generic_8G_v75_partitions;
                aspenite_nand_info.nr_parts[0] =
                        ARRAY_SIZE(generic_8G_v75_partitions);
        }

        aspenite_nand_info.use_dma = 0;
        aspenite_nand_info.enable_arbiter = 1;
        pxa168_add_nand((struct flash_platform_data *) &aspenite_nand_info);
}

#elif defined(CONFIG_SAMSUNG_512M_SLC_NAND)

static void __init aspenite_add_nand(void)
{
	aspenite_nand_info.parts[0] = generic_64M_v75_partitions;
	aspenite_nand_info.nr_parts[0] =
		ARRAY_SIZE(generic_64M_v75_partitions);


	aspenite_nand_info.use_dma = 0;
	aspenite_nand_info.enable_arbiter = 1;
	pxa168_add_nand((struct flash_platform_data *) &aspenite_nand_info);
}


#else

static void __init aspenite_add_nand(void)
{
        if (is_android()) {
                aspenite_nand_info.parts[0] = android_512m_v75_partitions;
                aspenite_nand_info.nr_parts[0] = ARRAY_SIZE(android_512m_v75_partitions);
        } else {
                aspenite_nand_info.parts[0] = generic_512m_v75_partitions;
                aspenite_nand_info.nr_parts[0] = ARRAY_SIZE(generic_512m_v75_partitions);
        }

        aspenite_nand_info.use_dma = 1;
        aspenite_nand_info.enable_arbiter = 1;
        pxa168_add_nand((struct flash_platform_data *) &aspenite_nand_info);
}

#endif

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

#if defined(CONFIG_WLAN_8688_SDIO)
	/* sdh MMC2, wlan*/
static struct pfn_cfg mmc2_pfn_cfg[] = {
	PFN_CFG(PIN_MMC_DAT7, PFN_UNDEF, PFN_UNDEF),
	PFN_CFG(PIN_MMC_DAT6, PFN_UNDEF, PFN_UNDEF),
	PFN_CFG(PIN_MMC_DAT5, PFN_UNDEF, PFN_UNDEF),
	PFN_CFG(PIN_MMC_DAT4, PFN_UNDEF, PFN_UNDEF),
	PFN_CFG(PIN_MMC_DAT3, GPIO90_MMC2_DAT3, GPIO90_GPIO),
	PFN_CFG(PIN_MMC_DAT2, GPIO91_MMC2_DAT2, GPIO91_GPIO),
	PFN_CFG(PIN_MMC_DAT1, GPIO92_MMC2_DAT1, GPIO92_GPIO),
	PFN_CFG(PIN_MMC_DAT0, GPIO93_MMC2_DAT0, GPIO93_GPIO),
	PFN_CFG(PIN_MMC_CMD, GPIO94_MMC2_CMD, GPIO94_GPIO),
	PFN_CFG(PIN_MMC_CLK, GPIO95_MMC2_CLK, GPIO95_GPIO),
	PFN_CFG(PIN_MMC_CD, PFN_UNDEF, PFN_UNDEF),
	PFN_CFG(PIN_MMC_WP, PFN_UNDEF, PFN_UNDEF),
	PFN_CFG(PIN_MMC_END, PFN_TERM, PFN_TERM),
};
#endif

#if defined(CONFIG_MMC3)
static struct pfn_cfg mmc3_pfn_cfg[] = {
	PFN_CFG(PIN_MMC_DAT7, GPIO0_MMC3_DAT7, GPIO0_GPIO),
	PFN_CFG(PIN_MMC_DAT6, GPIO1_MMC3_DAT6, GPIO1_GPIO),
	PFN_CFG(PIN_MMC_DAT5, GPIO2_MMC3_DAT5, GPIO2_GPIO),
	PFN_CFG(PIN_MMC_DAT4, GPIO3_MMC3_DAT4, GPIO3_GPIO),
	PFN_CFG(PIN_MMC_DAT3, GPIO4_MMC3_DAT3, GPIO4_GPIO),
	PFN_CFG(PIN_MMC_DAT2, GPIO5_MMC3_DAT2, GPIO5_GPIO),
	PFN_CFG(PIN_MMC_DAT1, GPIO6_MMC3_DAT1, GPIO6_GPIO),
	PFN_CFG(PIN_MMC_DAT0, GPIO7_MMC3_DAT0, GPIO7_GPIO),
	PFN_CFG(PIN_MMC_CLK, GPIO8_MMC3_CLK, GPIO8_GPIO),
	PFN_CFG(PIN_MMC_CMD, GPIO9_MMC3_CMD, GPIO9_GPIO),
	PFN_CFG(PIN_MMC_CD, PFN_UNDEF, PFN_UNDEF),
	PFN_CFG(PIN_MMC_WP, PFN_UNDEF, PFN_UNDEF),
	PFN_CFG(PIN_MMC_END, PFN_TERM, PFN_TERM),
};
#endif


static int sdh_mfp_config_mmc1(void)
{
	int ret = 0;

	ret = aspenite_pinmux_switch(SW_CARD);
	if (!ret)
		pfn_config(mmc1_pfn_cfg, PFN_FN);
	return ret;
}

static struct pxasdh_platform_data aspenite_sdh_platform_data_mmc1 = {
	.detect_delay	= 20,
	.ocr_mask	= MMC_VDD_29_30 | MMC_VDD_30_31,
	.mfp_config	= sdh_mfp_config_mmc1,
	.bus_width	= 4,
	.pfn_table	= mmc1_pfn_cfg,
};

#if defined(CONFIG_BT_HCIUART) && defined(CONFIG_WLAN_8688_SDIO)
static mfp_cfg_t aspenite_bt_uart_pins[] = {
	GPIO98_UART_SOUT,
	GPIO99_UART_SIN,
	GPIO100_UART_RTS,
	GPIO101_UART_CTS,
};

static void bt_uart_mfp_config(void)
{
	mfp_config(ARRAY_AND_SIZE(aspenite_bt_uart_pins));
	return;
}

static void __init aspenite_bt_init(void)
{
	bt_uart_mfp_config();
}

#endif

#if defined(CONFIG_WLAN_8688_SDIO)
static int stamp_8688_wlan_poweron(void)
{
	int gpio_power = 0;
	int gpio_reset = 0;
	int gpio_wake = 0;
	int gpio_h_wake = 0;

	gpio_power = GPIO_EXT1(5);
	gpio_reset = GPIO_EXT1(6);
	gpio_wake = GPIO_EXT1(8);
	gpio_h_wake = GPIO_EXT1(7);

	if (gpio_request(gpio_power, "8688 wlan power down")) {
		printk(KERN_INFO "gpio %d request failed\n", gpio_power);
		return -1;
	}

	if(gpio_request(gpio_reset, "8688 wlan reset")) {
		printk(KERN_INFO "gpio %d request failed\n", gpio_reset);
		gpio_free(gpio_power);
		return -1;
	}
	if(gpio_request(gpio_wake, "8688 wlan gpio_wake")) {
		printk(KERN_INFO "gpio %d request failed\n", gpio_wake);
		gpio_free(gpio_power);
		gpio_free(gpio_reset);
		return -1;
	}
	if(gpio_request(gpio_h_wake, "8688 wlan card gpio_h_wake")) {
		printk(KERN_INFO "gpio %d request failed\n", gpio_h_wake);
		gpio_free(gpio_power);
		gpio_free(gpio_reset);
		gpio_free(gpio_wake);
		return -1;
	}

	gpio_direction_output(gpio_power, 0);
	gpio_direction_output(gpio_reset, 0);
	gpio_direction_output(gpio_wake, 0);
	gpio_direction_input(gpio_h_wake);
	mdelay(500);
	gpio_direction_output(gpio_reset, 1);
	gpio_direction_output(gpio_power, 1);
	gpio_direction_output(gpio_wake, 1);

	gpio_free(gpio_power);
	gpio_free(gpio_reset);
	gpio_free(gpio_wake);
	gpio_free(gpio_h_wake);
	return 0;
}

static struct pxasdh_platform_data aspenite_sdh_platform_data_mmc2 = {
	.detect_delay	= 20,
	.ocr_mask	= MMC_VDD_29_30 | MMC_VDD_30_31,
	.bus_width	= 4,
	.quirks 	= SDHCI_QUIRK_BROKEN_CARD_DETECTION,
	.pfn_table	= mmc2_pfn_cfg,
};

#endif
#if defined(CONFIG_MMC3)
static struct pxasdh_platform_data aspenite_sdh_platform_data_mmc3 = {
	.detect_delay	= 20,
	.ocr_mask	= MMC_VDD_29_30 | MMC_VDD_30_31,
	.bus_width	= 8,
	.quirks 	= SDHCI_QUIRK_BROKEN_CARD_DETECTION,
	.pfn_table	= mmc3_pfn_cfg,
};
#endif
#endif

#ifdef CONFIG_USB_GADGET_PXA_U2O
static int gpio_usb_otg_pen = GPIO_EXT0(0);
static int gpio_usb_otg_stat1 = GPIO_EXT0(8);
static int gpio_usb_otg_stat2 = GPIO_EXT0(9);

static int aspenite_u2o_vbus_status(unsigned base)
{
	int otg_stat1, otg_stat2, otg_pen, status = VBUS_LOW;
	unsigned long flags;

	local_irq_save(flags);

#if 1 /* remove the workaroud here */
#ifdef CONFIG_USB_OTG
	/* FIXME on aspenite R0 boards otg stat1/stat2 could not
	 * reflect VBUS status yet, check with U2O itself instead
	 */
	mdelay(2);
	if (u2o_get(base, U2xOTGSC) & U2xOTGSC_BSV)
		status = VBUS_HIGH;
	else
		status = VBUS_LOW;

	return status;
#endif
#endif

	if (gpio_request(gpio_usb_otg_pen, "USB OTG Power Enable")) {
		printk(KERN_ERR "%s Max7312 USB_OTG_PEN GPIO Request"
			" Failed\n", __func__);
        	return -1;
	}

	if (gpio_request(gpio_usb_otg_stat1, "USB OTG VBUS stat1")) {
		printk(KERN_ERR "%s Max7312 USB_OTG_STAT1 GPIO Request"
			" Failed\n", __func__);
		return -1;
	}

	if (gpio_request(gpio_usb_otg_stat2, "USB OTG Power Enable")) {
		printk(KERN_ERR "%s Max7312 USB_OTG_STAT2 GPIO Request"
			" Failed\n", __func__);
		return -1;
	} 

	gpio_direction_input(gpio_usb_otg_pen);
	gpio_direction_input(gpio_usb_otg_stat1);
	gpio_direction_input(gpio_usb_otg_stat2);

	otg_pen = __gpio_get_value(gpio_usb_otg_pen);
	otg_stat1 = __gpio_get_value(gpio_usb_otg_stat1);
	otg_stat2 = __gpio_get_value(gpio_usb_otg_stat2);

	if (otg_pen) {
		status = VBUS_CHARGE;
		if (otg_stat1 && otg_stat2) {
			status |= VBUS_HIGH;
		}
	} else {
		/* workaroud for some aspenite rev1 board that stat1=1
		 * though vbus high, conflict with max3355 spec
		 * if (!otg_stat1 && !otg_stat2) */
		if (!otg_stat2) {
			status = VBUS_HIGH;
		}
	}

	printk(KERN_DEBUG "%s otg_pen %d stat1 %d stat2 %d status %d\n\n",
			__func__, otg_pen, otg_stat1, otg_stat2, status);
	gpio_free(gpio_usb_otg_pen);
	gpio_free(gpio_usb_otg_stat1);
	gpio_free(gpio_usb_otg_stat2);

	local_irq_restore(flags);
	return status;
}

static irqreturn_t aspenite_u2o_vbus_event(int irq, void *(func)(int))
{
	if (func)
		func(1);

	return IRQ_HANDLED;
}

static int aspenite_u2o_vbus_detect(void *func, int enable)
{
	int ret;

	if (enable) {
		/* FIXME needed for future SRP support
		 * request_irq(IRQ_GPIO(gpio_usb_otg_stat1),
			aspenite_u2o_vbus_event, IRQF_DISABLED |
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"otg_stat1", func);
		 */
		ret = request_irq(IRQ_GPIO(gpio_usb_otg_stat2),
			(irq_handler_t)aspenite_u2o_vbus_event, IRQF_DISABLED |
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"otg_stat2", func);
		if (ret)
			printk(KERN_INFO "request irq otgstat2 %d failed: %d\n",
					IRQ_GPIO(gpio_usb_otg_stat2), ret);
	} else {
		/* free_irq(IRQ_GPIO(gpio_usb_otg_stat1), NULL); */
		free_irq(IRQ_GPIO(gpio_usb_otg_stat2), func);
	}

	return 0;
}

static int aspenite_u2o_vbus_set(int vbus_type)
{
	unsigned long flags;

	local_irq_save(flags);

	if (gpio_request(gpio_usb_otg_pen, "USB OTG Power Enable")) {
		printk(KERN_ERR "%s Max7312 USB_OTG_PEN GPIO Request"
			" Failed\n", __func__);
		return -1;
	}

	switch (vbus_type) {
	case VBUS_SRP:
		gpio_direction_output(gpio_usb_otg_pen,1);
		udelay(10);
		gpio_direction_output(gpio_usb_otg_pen,0);
		break;
	case VBUS_HIGH:
		gpio_direction_output(gpio_usb_otg_pen,1);
		break;
	case VBUS_LOW:
 		gpio_direction_output(gpio_usb_otg_pen,0);
		break;
	default:
		break;
	}
	gpio_free(gpio_usb_otg_pen);

	local_irq_restore(flags);

	return 0;
}
static int aspenite_otg_init(void)
{
	int gpio_usb_otg_stat1 = 0, gpio_usb_otg_stat2 = 0;

	if (gpio_request(gpio_usb_otg_stat1, "USB OTG Host Status 1") &&
	    gpio_request(gpio_usb_otg_stat2, "USB OTG Host Status 2")) {
		printk(KERN_ERR "Max7312 USB OTG Status GPIO Request Failed\n");
		return -EAGAIN;
	}

	gpio_direction_input(gpio_usb_otg_stat1);
	gpio_direction_input(gpio_usb_otg_stat2);
	gpio_free(gpio_usb_otg_stat1);
	gpio_free(gpio_usb_otg_stat2);
	return 0;
}

static int aspenite_u2o_vbus_set_ic(int function)
{
	printk(KERN_DEBUG "%s %d not implemented yet\n", __func__, function);
	return 0;
}

static struct otg_pmic_ops aspenite_otg_ops = {
	.otg_vbus_init          = aspenite_otg_init,
	.otg_set_vbus           = aspenite_u2o_vbus_set,
	.otg_set_vbus_ic        = aspenite_u2o_vbus_set_ic,
	.otg_get_vbus_state     = aspenite_u2o_vbus_status,
};

struct otg_pmic_ops *init_aspenite_otg_ops(void)
{
	return &aspenite_otg_ops;
}

static struct pxa_usb_plat_info aspenite_u2o_info = {
	.phy_init	= pxa168_usb_phy_init,
	.phy_deinit	= pxa168_usb_phy_deinit,
	.vbus_set	= aspenite_u2o_vbus_set,
	.vbus_status	= aspenite_u2o_vbus_status,
	.vbus_detect    = aspenite_u2o_vbus_detect,
	.init_pmic_ops	= (void *)init_aspenite_otg_ops,
#ifdef CONFIG_USB_OTG
	.is_otg		= 1,
#else
	.clk_gating	= 1,
#endif
};
#endif

#ifdef CONFIG_USB_EHCI_PXA_U2H
/* USB 2.0 Host Controller */
static int aspenite_u2h_vbus_set (int enable)
{
	int gpio_u2h_vbus_on = GPIO_EXT0(4);
	int gpio_u2h_vbus_flt_n = GPIO_EXT0(14);

	if(gpio_request(gpio_u2h_vbus_on, "USB Host VBUS_ON")) {
		printk(KERN_ERR "Max7312 VBUS_ON GPIO Request Failed\n");
		return -EIO;
	}
	if(gpio_request(gpio_u2h_vbus_flt_n, "USB Host VBUS_FLT_N")) {
                printk(KERN_ERR "Max7312 VBUS_FLT_N GPIO Request Failed\n");
		return -EIO;
	}
	if (gpio_u2h_vbus_on && gpio_u2h_vbus_flt_n)
	{
		if (enable)
			gpio_direction_output(gpio_u2h_vbus_on, 1);
		else
			gpio_direction_output(gpio_u2h_vbus_on, 0);

		gpio_direction_input(gpio_u2h_vbus_flt_n);
		gpio_free(gpio_u2h_vbus_on);
		gpio_free(gpio_u2h_vbus_flt_n);
	}

	return 0;
}

static struct pxa_usb_plat_info aspenite_u2h_info = {
	.phy_init	= pxa168_usb_phy_init,
	.vbus_set	= aspenite_u2h_vbus_set,
};
#endif

static void __init aspenite_init(void)
{
	mfp_config(ARRAY_AND_SIZE(aspenite_pin_config));
        pxa168_set_vdd_iox(VDD_IO0, VDD_IO_3P3V);
	pxa168_set_vdd_iox(VDD_IO1, VDD_IO_3P3V);
	pxa168_set_vdd_iox(VDD_IO2, VDD_IO_3P3V);
	pxa168_set_vdd_iox(VDD_IO3, VDD_IO_3P3V);
	pxa168_set_vdd_iox(VDD_IO4, VDD_IO_3P3V);
	pxa168_mfp_set_fastio_drive(MFP_DS02X);

	/* on-chip devices */
	pxa168_add_uart(1);

	pxa168_add_freq();

#if defined(CONFIG_BT_HCIUART) && defined(CONFIG_WLAN_8688_SDIO)
	pxa168_add_uart(3);
#endif
	aspenite_add_nand();
	pxa168_add_ssp(0);
	pxa168_add_twsi(0, &pwri2c_info, ARRAY_AND_SIZE(aspenite_i2c_board_info));
	pxa168_add_twsi(1, &pwri2c_info, ARRAY_AND_SIZE(pwri2c_board_info));
	if (is_android())
		pxa168_add_keypad(&aspenite_android_keypad_info);
	else
		pxa168_add_keypad(&aspenite_keypad_info);

#ifdef CONFIG_USB_GADGET_PXA_U2O
 	pxa168_add_u2o(&aspenite_u2o_info);
#endif

#ifdef CONFIG_USB_OTG
	pxa168_add_u2ootg(&aspenite_u2o_info);
	pxa168_add_u2oehci(&aspenite_u2o_info);
#endif

#ifdef CONFIG_USB_EHCI_PXA_U2H
 	pxa168_add_u2h(&aspenite_u2h_info);
#endif
	pxa168_add_mfu(&pxa168_eth_data);
#ifdef CONFIG_PCI
	pxa168_add_pcie(&pxa168_pcie_data);
#endif
#if defined(CONFIG_MMC_PXA_SDH)
	pxa168_add_sdh(0, &aspenite_sdh_platform_data_mmc1);
#if defined(CONFIG_WLAN_8688_SDIO)
	pxa168_add_sdh(1, &aspenite_sdh_platform_data_mmc2);
#if defined(CONFIG_BT_HCIUART)
	aspenite_bt_init();
#endif
#endif
#if defined(CONFIG_MMC3)
	pxa168_add_sdh(2, &aspenite_sdh_platform_data_mmc3);
#endif
#endif
#if defined(CONFIG_CIR)
	pxa168_cir_init(); /*init the gpio */
#endif
#if defined(CONFIG_PXA168_MSP)
	pxa168_add_msp(&msp_ops);
#endif
#if defined(CONFIG_PXA168_CF)
#if defined(CONFIG_PXA168_CF_USE_GPIO_CARDDETECT)
	pxa168_cf_init();
#else
	pxa168_add_cf();	
#endif
#endif
	if (machine_is_aspenite()) {
		pxa168_add_fb(&aspenite_lcd_info);
		pxa168_add_fb_ovly(&aspenite_lcd_ovly_info);
	}
	/* off-chip devices */
	if (machine_is_zylonite2()) {
		pxa168_add_fb(&zylonite2_lcd_info);
		platform_device_register(&smc91x_device);
	}

	aspenite_init_spi();
#if defined(CONFIG_PXA168_CAMERA)
	pxa168_add_cam();
#endif
#if defined(CONFIG_PXA_ICR)
	pxa168_add_icr();
#endif

#if defined(CONFIG_BATTERY_ASPENITE)
	aspenite_add_battery();
#endif

}

MACHINE_START(ASPENITE, "PXA168 based Aspenite Development Platform")
	.phys_io        = APB_PHYS_BASE,
	.boot_params    = 0x00000100,
	.io_pg_offst    = (APB_VIRT_BASE >> 18) & 0xfffc,
	.map_io		= pxa_map_io,
	.init_irq       = pxa168_init_irq,
	.timer          = &pxa168_timer,
	.init_machine   = aspenite_init,
MACHINE_END

MACHINE_START(ZYLONITE2, "PXA168-based Zylonite2 Development Platform")
	.phys_io        = APB_PHYS_BASE,
	.boot_params    = 0x00000100,
	.io_pg_offst    = (APB_VIRT_BASE >> 18) & 0xfffc,
	.map_io		= pxa_map_io,
	.init_irq       = pxa168_init_irq,
	.timer          = &pxa168_timer,
	.init_machine   = aspenite_init,
MACHINE_END
