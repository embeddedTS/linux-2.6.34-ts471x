/*
 * Driver for batteries with sanremo chips inside.
 *
 * Copyright © 2009 Bin Yang 
 *	       2007 Anton Vorontsov
 *	       2004-2007 Matt Reimer
 *	       2004 Szabolcs Gyurko
 *
 * Use consistent with the GNU GPL is permitted,
 * provided that this copyright notice is
 * preserved in its entirety in all copies and derived works.
 *
 * Author: 
 *	    Bin Yang <bin.yang@marvell.com>
 *	    June 2009
 *
 *	    Anton Vorontsov <cbou@mail.ru>
 *	    February 2007
 *
 *	    Matt Reimer <mreimer@vpop.net>
 *	    April 2004, 2005, 2007
 *
 *	    Szabolcs Gyurko <szabolcs.gyurko@tlt.hu>
 *	    September 2004
 */

#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <mach/sanremo.h>

static struct power_supply bat;

static int sanremo_battery_battery_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	u8 tmp;
	int value;

	switch (psp) {
		case POWER_SUPPLY_PROP_STATUS:
			sanremo_read(SANREMO_CHG_CTRL1, &tmp); 
			if(tmp & 0x3)
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
			else
				val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			break;
		case POWER_SUPPLY_PROP_HEALTH:
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
			break;
		case POWER_SUPPLY_PROP_PRESENT:
			val->intval = 1;
			break;
		case POWER_SUPPLY_PROP_CAPACITY:
			value = sanremo_get_vbat();
			if(value <= 3500)
				val->intval = 1;
			else if(value > 4000)
				val->intval = 100;
			else
				val->intval = 100 - (4000-value)/5;
			break;
		case POWER_SUPPLY_PROP_TECHNOLOGY:
			val->intval = 1;
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			sanremo_read(SANREMO_VBAT_MEAS2, &tmp);
			value = tmp&0xf;
			sanremo_read(SANREMO_VBAT_MEAS1, &tmp);
			value += tmp<<4;
			val->intval = (value * 1000); /*uV*/
			break;
		case POWER_SUPPLY_PROP_CURRENT_NOW:
			sanremo_read(SANREMO_IBAT_MEAS2, &tmp);
			value = tmp&0x3f;
			sanremo_read(SANREMO_IBAT_MEAS1, &tmp);
			value += tmp<<6;
			val->intval = value;/*uA*/
			break;
		case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
			val->intval = 1000000; /*uA*/
			break;
		case POWER_SUPPLY_PROP_CHARGE_FULL:
			val->intval = 1000000; /*uAh*/
			break;
		case POWER_SUPPLY_PROP_CHARGE_EMPTY:
			val->intval = 900000;
			break;
		case POWER_SUPPLY_PROP_CHARGE_NOW:
			val->intval = 500000;
			break;
		case POWER_SUPPLY_PROP_TEMP:
			val->intval = 250;
			break;
		case POWER_SUPPLY_PROP_ONLINE:
			val->intval = 1;
			break;
		default:
			return -EINVAL;
	}

	return 0;
}

static enum power_supply_property sanremo_battery_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_EMPTY,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_TEMP,
};

static int sanremo_battery_battery_probe(struct platform_device *pdev)
{
	int retval = 0;

	bat.name	   = "battery";
	bat.type	   = POWER_SUPPLY_TYPE_BATTERY;
	bat.properties     = sanremo_battery_battery_props;
	bat.num_properties = ARRAY_SIZE(sanremo_battery_battery_props);
	bat.get_property   = sanremo_battery_battery_get_property;

	retval = power_supply_register(&pdev->dev, &bat);
	if (retval)
		printk(KERN_ERR "failed to register battery\n");

	return retval;
}

static int sanremo_battery_battery_remove(struct platform_device *pdev)
{
	power_supply_unregister(&bat);
	return 0;
}

#ifdef CONFIG_PM

static int sanremo_battery_battery_suspend(struct platform_device *pdev,
				  pm_message_t state)
{
	return 0;
}

static int sanremo_battery_battery_resume(struct platform_device *pdev)
{
	return 0;
}

#else

#define sanremo_battery_battery_suspend NULL
#define sanremo_battery_battery_resume NULL

#endif /* CONFIG_PM */

MODULE_ALIAS("platform:sanremo_battery-battery");

static struct platform_driver sanremo_battery_battery_driver = {
	.driver = {
		.name = "sanremo_battery",
	},
	.probe	  = sanremo_battery_battery_probe,
	.remove   = sanremo_battery_battery_remove,
	.suspend  = sanremo_battery_battery_suspend,
	.resume	  = sanremo_battery_battery_resume,
};

static int __init sanremo_battery_battery_init(void)
{
	return platform_driver_register(&sanremo_battery_battery_driver);
}

static void __exit sanremo_battery_battery_exit(void)
{
	platform_driver_unregister(&sanremo_battery_battery_driver);
}

module_init(sanremo_battery_battery_init);
module_exit(sanremo_battery_battery_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Bin Yang <bin.yang@marvell.com");
MODULE_DESCRIPTION("sanremo_battery battery driver");
