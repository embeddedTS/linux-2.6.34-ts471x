/*
 *  linux/arch/arm/mach-mmp/dkb_generic.c
 *
 *  Support for the Marvell PXA910-based Generic DKBDevelopment Platform.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/i2c/pca953x.h>
#include <linux/smc91x.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/delay.h>
#include <linux/pda_power.h>
#include <linux/switch.h>
#include <linux/proc_fs.h>

#include <asm/uaccess.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/flash.h>
#include <mach/cputype.h>
#include <mach/addr-map.h>
#include <mach/mfp-pxa910.h>
#include <mach/pxa168.h>
#include <mach/pxa168fb.h>
#include <mach/pxa910.h>
#include <mach/gpio.h>
#include <mach/sanremo.h>
#include <mach/portofino.h>
#include <mach/resource.h>

#include <plat/part_table.h>
#include <plat/generic.h>
#include "common.h"
#include <mach/camera.h>
#include <mach/pxa3xx_nand.h>

#include <plat/pxa_u2o.h>
#include <linux/usb/otg.h>
#ifdef CONFIG_SD8XXX_RFKILL
#include <linux/sd8x_rfkill.h>
#endif

#define ARRAY_AND_SIZE(x)       (x), ARRAY_SIZE(x) 
/*used by expander max7312, 16 pins gpio expander */
#define GPIO_EXT0(x)		(NR_BUILTIN_GPIO + (x))
#define GPIO_EXT1(x)		(NR_BUILTIN_GPIO + 16 + (x))
/*uart, smc, i2c are checked by bin.yang*/
static unsigned long dkb_generic_pin_config[] __initdata = {
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

	/* I2C */
	GPIO53_CI2C_SCL,
	GPIO54_CI2C_SDA,

	/* SSP1 (I2S) */
	GPIO24_SSP1_SDATA_IN,
	GPIO21_SSP1_BITCLK,
	/*GPIO20_SSP1_SYSCLK,*/
	GPIO22_SSP1_SYNC,
	GPIO23_SSP1_DATA_OUT,
	GPIO124_MN_CLK_OUT,
	GPIO123_CLK_REQ,

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

};

#ifdef CONFIG_SANREMO
static int dkb_generic_is_ac_online(void)
{
	u8 tmp;

	return 0;
}

static int dkb_generic_is_usb_online(void)
{
	return sanremo_usb_connect();
}

static char *dkb_generic_supplicants[] = {
	"battery",
};

static struct pda_power_pdata dkb_generic_power_supply_info = {
	.is_ac_online   = dkb_generic_is_ac_online,
	.is_usb_online   = dkb_generic_is_usb_online,
	.supplied_to     = dkb_generic_supplicants,
	.num_supplicants = ARRAY_SIZE(dkb_generic_supplicants),
};

static struct resource dkb_generic_power_supply_resources[] = {
	[0] = {
		.name  = "ac",
	},
	[1] = {
		.name  = "usb",
	},
};

static struct platform_device dkb_generic_power_supply = {
	.name = "pda-power",
	.id   = -1,
	.dev  = {
	        .platform_data = &dkb_generic_power_supply_info,
	},
	.resource      = dkb_generic_power_supply_resources,
	.num_resources = ARRAY_SIZE(dkb_generic_power_supply_resources),
};

static struct platform_device sanremo_battery = {
	.name = "sanremo_battery",
	.id   = -1,
};

static struct platform_device portofino_bl_device = {
	.name           = "portofino-bl",
	.id             = 0,
};

static void __init dkb_generic_init_power(void)
{
       platform_device_register(&dkb_generic_power_supply);
       platform_device_register(&sanremo_battery);
}
#else
static void __init dkb_generic_init_power(void) {}
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

	sanremo_write(SANREMO_LDO12, 0x66); /*enable ldo12 for touch*/
	sanremo_write(SANREMO_LDO3, 0x2d); /*enable ldo12 for touch*/
	sanremo_write(SANREMO_VBUCK2_SET, 0x24); /*enable i2c 1.8_2.8 voltage rail */
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

static struct sanremo_platform_data sanremo_data = {
	.init_irq = sanremo_init_irq,
	.ack_irq = sanremo_ack_irq,
	.platform_init = sanremo_init,
	.power_chips = sanremo_chips,
	.tsi = NULL,
};
#endif /* CONFIG_PXA3xx_SANREMO || CONFIG_PXA3xx_SANREMO_MODULE*/

static struct i2c_pxa_platform_data i2c_info = {
	.use_pio		= 0,
	.get_ripc		= get_RIPC,
	.release_ripc		= release_RIPC,
};

static struct i2c_pxa_platform_data pwri2c_info = {
	.use_pio		= 0,
};

#if defined(CONFIG_GPIO_PCA953X)

/* GPIO expander max7312 could reuse PCA953X */
static struct pca953x_platform_data max7312_data[] = {
	[0] = {
		.gpio_base      = GPIO_EXT0(0),
	},
};
#endif

static struct i2c_board_info dkb_generic_i2c_board_info[] =
{
#if defined(CONFIG_GPIO_PCA953X)
	{
		.type           = "max7312",
		.addr           = 0x20,
		.irq            = IRQ_GPIO(80),
		.platform_data  = &max7312_data,
	},
#endif
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
};

static struct i2c_board_info pxa920_pwri2c_board_info[] =
{
};

DECLARE_LAB_PARTITIONS(lab_partitions);
DECLARE_ANDROID_256M_V75_PARTITIONS(android_256m_v75_partitions);
DECLARE_256M_V75_PARTITIONS(generic_256m_v75_partitions);
static struct flash_platform_data dkb_generic_onenand_info;
static struct pxa3xx_nand_platform_data dkb_generic_nand_info;
static void __init dkb_generic_add_flash(void)
{
	if (is_lab()){
		dkb_generic_onenand_info.parts = lab_partitions;
		dkb_generic_onenand_info.nr_parts = ARRAY_SIZE(lab_partitions);
		dkb_generic_nand_info.parts[0] = lab_partitions;
		dkb_generic_nand_info.nr_parts[0] = ARRAY_SIZE(lab_partitions);
	}else if (is_android()) {
		dkb_generic_onenand_info.parts = android_256m_v75_partitions;
		dkb_generic_onenand_info.nr_parts = ARRAY_SIZE(android_256m_v75_partitions);
		dkb_generic_nand_info.parts[0] = android_256m_v75_partitions;
		dkb_generic_nand_info.nr_parts[0] = ARRAY_SIZE(android_256m_v75_partitions);
	} else {
		dkb_generic_onenand_info.parts = generic_256m_v75_partitions;
		dkb_generic_onenand_info.nr_parts = ARRAY_SIZE(generic_256m_v75_partitions);
		dkb_generic_nand_info.parts[0] = generic_256m_v75_partitions;
		dkb_generic_nand_info.nr_parts[0] = ARRAY_SIZE(generic_256m_v75_partitions);
	}
	pxa168_add_onenand(&dkb_generic_onenand_info);
	dkb_generic_nand_info.use_dma = 0;
	dkb_generic_nand_info.enable_arbiter = 1;
	pxa168_add_nand(&dkb_generic_nand_info);
}

#ifdef CONFIG_USB_GADGET_PXA_U2O
static int dkb_generic_vbus_status(unsigned base)
{
	int status = VBUS_LOW;

	if (pxa3xx_pmic_is_vbus_assert()) {
		status = VBUS_HIGH;
	}
	return status;
}

static int dkb_generic_vbus_detect(void *func, int enable)
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

static struct pxa_usb_plat_info dkb_generic_u2o_info = {
	.phy_init	= pxa168_usb_phy_init,
	.phy_deinit	= pxa168_usb_phy_deinit,
	.vbus_status	= dkb_generic_vbus_status,
	.vbus_detect	= dkb_generic_vbus_detect,
	.rely_on_vbus	= 1,
  	.is_otg		= 1,
};
#endif

static struct platform_device sensor_input_device = {
	.name = "sensor_input",
	.id   = -1,
};

static void (*headset_update_func)(int state);

static void sanremo_headset_interrupt(unsigned long event)
{
	if (headset_update_func)
		headset_update_func(sanremo_get_headset_state());
}

static int dkb_generic_headset_detect(void *func, int enable)
{
	headset_update_func = func;
	headset_update_func(1);
	return sanremo_enable_headset_detect(sanremo_headset_interrupt, enable);
}

static struct headset_switch_platform_data headset_switch_device_data = {
	.name = "h2w",
	.gpio = NULL,
	.name_on = NULL,
	.name_off = NULL,
	.state_on = NULL,
	.state_off = NULL,
	.enable_detect = dkb_generic_headset_detect,
};

static struct platform_device headset_switch_device = {
	.name            = "headset",
	.id              = 0,
	.dev             = {
		.platform_data = &headset_switch_device_data,
	},
};

static void __init dkb_generic_init_headset(void)
{
	platform_device_register(&headset_switch_device);
}

static void __init dkb_generic_init(void)
{
	/*dummy driver init*/
	platform_device_register(&sensor_input_device);

	mfp_config(ARRAY_AND_SIZE(dkb_generic_pin_config));

	/* on-chip devices */
	pxa910_add_uart(1);
	pxa910_add_uart(2);
	pxa910_add_uart(3);
	pxa910_add_twsi(0, &i2c_info, ARRAY_AND_SIZE(dkb_generic_i2c_board_info));
	pxa910_add_twsi(1, &pwri2c_info, ARRAY_AND_SIZE(dkb_generic_i2c_board_info));
#ifdef CONFIG_USB_GADGET_PXA_U2O
	pxa168_add_u2o(&dkb_generic_u2o_info);
#endif
#ifdef CONFIG_USB_OTG
	pxa168_add_u2ootg(&dkb_generic_u2o_info);
	pxa168_add_u2oehci(&dkb_generic_u2o_info);
#endif
	pxa910_add_acipc();
	pxa910_add_ire();
#if defined(CONFIG_PXA168_CAMERA)
	pxa168_add_cam();
#endif
	pxa910_add_ssp(1);
	pxa910_add_imm();
	pxa168_add_freq();

	pxa910_add_rtc();
	dkb_generic_add_flash();

#ifdef CONFIG_SANREMO
	platform_device_register(&portofino_bl_device);
#endif

	/*power device*/
	dkb_generic_init_power();

	/*headset device*/
	dkb_generic_init_headset();

	res_add_sanremo_vibrator();

	pm_power_off = sanremo_turn_off_power;
}

MACHINE_START(DKB_GENERIC, "PXA910-based Generic DKB Development Platform")
	.phys_io        = APB_PHYS_BASE,
	.boot_params    = 0x00000100,
	.io_pg_offst    = (APB_VIRT_BASE >> 18) & 0xfffc,
	.map_io		= pxa_map_io,
	.init_irq       = pxa168_init_irq,
	.timer          = &pxa168_timer,
	.init_machine   = dkb_generic_init,
MACHINE_END
