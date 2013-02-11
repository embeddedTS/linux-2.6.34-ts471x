/*
 * * PXA168 MAX8660 PMIC Management Routines
 * *
 * *
 * * Copyright (C) 2009, Marvell Corporation(bshen9@marvell.com).
 * *
 * * This software program is licensed subject to the GNU General Public License
 * * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 * */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/sysctl.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/suspend.h>

#include <asm/ioctl.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <plat/pxa3xx_pmic.h>
#include <mach/max8660.h>


extern int get_pm_state(void);
static struct pxa3xx_pmic_regs max8660_regs[MAX8660_REG_NUM];
static struct power_supply_module *max8660_power_module;
static struct i2c_client *g_client;


/* max8660 do not support read to the register */
int max8660_read(u8 reg, u8 *pval)
{
	/* Cache read of the max8660 register.*/
	*pval = max8660_regs[reg].data;

	return 0;
}

int max8660_write(u8 reg, u8 val)
{
	struct max8660_platform_data *pdata = g_client->dev.platform_data;
	int ret;
	int status;
	int map;

	if (g_client == NULL)	/* No global client pointer? */
		return -1;

	spin_lock(&pdata->lock);
	map = max8660_reg_map[reg];
	ret = i2c_smbus_write_byte_data(g_client, map, val);
	if (ret == 0) {
		max8660_regs[reg].data = val;
		status = 0;
	} else
		status = -EIO;
	spin_unlock(&pdata->lock);

	return status;
}

static int max8660_initchip(void)
{
	memset(&max8660_regs, 0,
		(sizeof(struct pxa3xx_pmic_regs) * MAX8660_REG_NUM));
	

	max8660_regs[MAX8660_OVER1].data = 0x0;
	max8660_regs[MAX8660_OVER2].data = 0x0;
	max8660_regs[MAX8660_VCC1].data = 0x0;
	max8660_regs[MAX8660_ADTV1].data = 0x1b;
	max8660_regs[MAX8660_ADTV2].data = 0x1b;
	max8660_regs[MAX8660_SDTV1].data = 0x1b;
	max8660_regs[MAX8660_SDTV2].data = 0x1b;
	max8660_regs[MAX8660_MDTV1].data = 0x4;
	max8660_regs[MAX8660_MDTV2].data = 0x4;
	max8660_regs[MAX8660_L12VCR].data = 0x0;
	max8660_regs[MAX8660_FPWM].data = 0x0;
	
	return 0;
}

static int get_power_supply_module(int cmd)
{
        int power_module, i;

	power_module = -EINVAL;

        if (cmd < VCC_CORE || cmd >= MAX_CMD) {
		printk(KERN_WARNING "input wrong command: %d\n", cmd);
		return -EINVAL;
	}


	for(i = 0; max8660_power_module[i].command != 0; i++ ){
		if(max8660_power_module[i].command == cmd){
			power_module = max8660_power_module[i].power_module;
			break;
		}
	}
					
	return power_module;
}

static int set_max8660_voltage(unsigned int cmd, unsigned int mv)
{
	unsigned char val;
	int status = 0;
	int power_module = get_power_supply_module(cmd);


	switch (power_module){
	case MAX8660_V1:
		break;

	case MAX8660_V2:
		break;

	case MAX8660_V3:
		if (mv == 0)
		{	/* Disable V3 output */
			max8660_read(MAX8660_OVER1, &val);
			val &= ~OVER1_EN3;
			status = max8660_write(MAX8660_OVER1, val);
			return status;
		}
		else if ((mv < MAX8660_V3_BASE) || (mv > MAX8660_V3_MAX))
			return -EINVAL;

		max8660_read(MAX8660_VCC1, &val);
		
		if( val & VCC1_AVS){
			status = max8660_write(MAX8660_ADTV2, (mv - MAX8660_V3_BASE)/MAX8660_V3_STEP);
		
		} else {
			status = max8660_write(MAX8660_ADTV1, (mv - MAX8660_V3_BASE)/MAX8660_V3_STEP);
		}
		
		val |= VCC1_AGO;
		status= max8660_write(MAX8660_VCC1, val);

		max8660_read(MAX8660_OVER1, &val);
		val |= OVER1_EN3;
		status = max8660_write(MAX8660_OVER1, val);
		
		break;

	case MAX8660_V4:
		if (mv == 0)
		{	/* Disable V4 output */
			status = max8660_read(MAX8660_OVER1, &val);
			val &= ~OVER1_EN4;
			status = max8660_write(MAX8660_OVER1, val);
			return status;
		}
		else if ((mv < MAX8660_V4_BASE) || (mv > MAX8660_V4_MAX))
			return -EINVAL;

		max8660_read(MAX8660_VCC1, &val);

		if( val & VCC1_SVS){
			status = max8660_write(MAX8660_SDTV2, (mv - MAX8660_V4_BASE)/MAX8660_V4_STEP);
		} else {
			status = max8660_write(MAX8660_SDTV1, (mv - MAX8660_V4_BASE)/MAX8660_V4_STEP);
		}

		val |= VCC1_SGO;
		status= max8660_write(MAX8660_VCC1, val);
		
		max8660_read(MAX8660_OVER1, &val);
		val |= OVER1_EN4;
		status = max8660_write(MAX8660_OVER1, val);

		break;

	case MAX8660_V5:
		if ((mv < MAX8660_V5_BASE) || (mv > MAX8660_V5_MAX))
			return -EINVAL;

		max8660_read(MAX8660_VCC1, &val);
		
		if( val & VCC1_MVS){
			status = max8660_write(MAX8660_MDTV2, (mv - MAX8660_V5_BASE)/MAX8660_V5_STEP);
		} else {
			status = max8660_write(MAX8660_MDTV1, (mv - MAX8660_V5_BASE)/MAX8660_V5_STEP);
		}

		val |= VCC1_MGO;
		status= max8660_write(MAX8660_VCC1, val);
		break;

	case MAX8660_V6:
		if (mv == 0)
		{	/* Disable V6 output */
			status = max8660_read(MAX8660_OVER2, &val);
			val &= ~OVER2_EN6;
			status = max8660_write(MAX8660_OVER2, val);
			return status;
		}
		else if ((mv < MAX8660_V6_BASE) || (mv > MAX8660_V6_MAX))
			return -EINVAL;
		
		max8660_read(MAX8660_L12VCR, &val);
		val = (val & 0xf0) | (mv-MAX8660_V6_BASE)/MAX8660_V6_STEP;
		status = max8660_write(MAX8660_L12VCR, val);

		max8660_read(MAX8660_OVER2, &val);
		val |= OVER2_EN6;
		status = max8660_write(MAX8660_OVER2, val);
		break;

	case MAX8660_V7:
		if (mv == 0)
		{	/* Disable V7 output */
			status = max8660_read(MAX8660_OVER2, &val);
			val &= ~OVER2_EN7;
			status = max8660_write(MAX8660_OVER2, val);
			return status;
		}
		else if ((mv < MAX8660_V7_BASE) || (mv > MAX8660_V7_MAX))
			return -EINVAL;
	
		max8660_read(MAX8660_L12VCR, &val);
		val = (val & 0xf) | (mv - MAX8660_V7_BASE)/MAX8660_V7_STEP << 4;
		status = max8660_write(MAX8660_L12VCR, val);
		
		max8660_read(MAX8660_OVER2, &val);
		val |= OVER2_EN7;
		status = max8660_write(MAX8660_OVER2, val);
		break;

	case MAX8660_V8:
		break;
	
	}

	return status;
}


static int get_max8660_voltage(unsigned int cmd, int *pmv)
{
	unsigned char val;
	int status = 0;
	int power_module = get_power_supply_module(cmd);

	*pmv = 0;

	switch (power_module){
	case MAX8660_V1:
		*pmv = MAX8660_V1_DEFAULT;
		break;

	case MAX8660_V2:
		*pmv = MAX8660_V2_DEFAULT;
		break;

	case MAX8660_V3:
		status = max8660_read(MAX8660_OVER1, &val);
		if( val & OVER1_EN3){
			status = max8660_read(MAX8660_VCC1,&val);
			if (val & VCC1_AGO)
			{
				if (val & VCC1_AVS)
				{
					status = max8660_read(MAX8660_ADTV2, &val);
					*pmv = MAX8660_V3_BASE + MAX8660_V3_STEP * val;
				} else {
					status = max8660_read(MAX8660_ADTV1, &val);
					*pmv = MAX8660_V3_BASE + MAX8660_V3_STEP * val;
				}
			} else {
				*pmv = MAX8660_V3_DEFAULT;
			}
		} else {
			/*V3 output disable */
			*pmv = 0;
		}
		break;

	case MAX8660_V4:
		status = max8660_read(MAX8660_OVER1, &val);
		if( val & OVER1_EN4){
			status = max8660_read(MAX8660_VCC1,&val);
			if (val & VCC1_SGO)
			{
				if (val & VCC1_SVS)
				{
					status = max8660_read(MAX8660_SDTV2, &val);
					*pmv = MAX8660_V4_BASE + MAX8660_V4_STEP * val;
				} else {
					status = max8660_read(MAX8660_SDTV1, &val);
					*pmv = MAX8660_V4_BASE + MAX8660_V4_STEP * val;
				}
			} else {
				*pmv = MAX8660_V4_DEFAULT;
			}
		} else {
			/*V4 output disable */
			*pmv = 0;
		}

		break;

	case MAX8660_V5:
		status = max8660_read(MAX8660_VCC1, &val);
		if (val & VCC1_MGO)
		{
			if (val & VCC1_MVS)
			{
				status = max8660_read(MAX8660_MDTV2, &val);
				*pmv = MAX8660_V5_BASE + MAX8660_V5_STEP * val;
			} else {
				status = max8660_read(MAX8660_MDTV1, &val);
				*pmv = MAX8660_V5_BASE + MAX8660_V5_STEP * val;
			}
		} else {
			*pmv = MAX8660_V5_DEFAULT;
		}

		break;

	case MAX8660_V6:
		status = max8660_read(MAX8660_OVER2, &val);
		if (val & OVER2_EN6)
		{
			status = max8660_read(MAX8660_L12VCR, &val);
			val &= 0xf;
			*pmv = MAX8660_V6_BASE + MAX8660_V6_STEP * val;
		} else {
			*pmv = 0;
		}
		break;

	case MAX8660_V7:
		status = max8660_read(MAX8660_OVER2, &val);
		if (val & OVER2_EN7)
		{
			status = max8660_read(MAX8660_L12VCR, &val);
			val &= 0xf0;
			val = val >> 4;
			*pmv = MAX8660_V6_BASE + MAX8660_V6_STEP * val;
		} else {
			*pmv = 0;
		}
		break;

	case MAX8660_V8:
		*pmv = MAX8660_V8_DEFAULT;
		break;

	}
	
	return status;
}


#ifdef	CONFIG_PM
static unsigned int vcc_misc1, vcc_misc2, hdmi_1p2v;

static int max8660_suspend(struct i2c_client *client, pm_message_t state)
{
	get_max8660_voltage(VCC_MISC1, &vcc_misc1);
	get_max8660_voltage(VCC_MISC2, &vcc_misc2);
	get_max8660_voltage(HDMI_1P2V, &hdmi_1p2v);
	set_max8660_voltage(VCC_MISC1, 0);
	set_max8660_voltage(VCC_MISC2, 0);
	set_max8660_voltage(HDMI_1P2V, 0);
	return 0;
}

static int max8660_resume(struct i2c_client *client)
{
	set_max8660_voltage(VCC_MISC1, vcc_misc1);
	set_max8660_voltage(VCC_MISC2, vcc_misc2);
	set_max8660_voltage(HDMI_1P2V, hdmi_1p2v);
	return 0;
}
#else
#define	max8660_suspend		NULL
#define	max8660_resume		NULL
#endif

static struct pmic_ops max8660_pmic_ops = {
	.get_voltage            = get_max8660_voltage,
	.set_voltage            = set_max8660_voltage,
};

static int __devinit max8660_probe(struct i2c_client *client,
		const struct i2c_device_id* id)
{
	struct max8660_platform_data *pdata;
	int ret;
	int i;

	g_client = client;

	ret = max8660_initchip();
	if (ret != 0)
		printk(KERN_WARNING "Initialize max8660 failed with ret 0x%x\n",
			ret);

	pdata = client->dev.platform_data;
	/* init spinlock */
	spin_lock_init(&pdata->lock);


	for (i = 0; pdata->power_chips[i].power_supply_modules != NULL; i++) {
		if (pdata->power_chips[i].chip_id == MAX8660_ID ){
			max8660_power_module = pdata->power_chips[i].power_supply_modules;
			break;
		}
        }

        if (pdata->power_chips[i].power_supply_modules == NULL)
		panic("This max8660 chip is not supported");

	pmic_set_ops(&max8660_pmic_ops);
	
	pdata->platform_init();
	
	return 0;
}

static int max8660_remove(struct i2c_client *client)
{
	pmic_set_ops(NULL);

	free_irq(client->irq, NULL);

	return 0;
}

static const struct i2c_device_id max8660_id[] = {
        { "max8660", 0x34 },
	{},
};

static struct i2c_driver max8660_driver = {
	.driver = {
		.name	= "max8660",
	},
	.probe		= max8660_probe,
	.remove		= max8660_remove,
	.suspend	= max8660_suspend,
	.resume		= max8660_resume,
	.id_table	= max8660_id,
};

static int __init max8660_init(void)
{
	return i2c_add_driver(&max8660_driver);
}

static void __exit max8660_exit(void)
{
	i2c_del_driver(&max8660_driver);
}

subsys_initcall(max8660_init);
module_exit(max8660_exit);

MODULE_DESCRIPTION("Max8660 PMIC Driver");
MODULE_LICENSE("GPL");

