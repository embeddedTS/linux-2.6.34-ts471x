/*
 * PXA168 Power supply driver
 *
 * Android requires the platform to report battery status to the
 * com.android.server.BatteryService through linux power_supply framework
 *
 * Copyright (C) 2009 Marvell International Ltd.
 * All rights reserved. 
 * Author: Mark Brown <markb@marvell.com>
 * Based on goldfish_battery.c driver by Mike Lockwood <lockwood@android.com>
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <asm/io.h>

#define	TRUE	1

#define FAKE_CAPACITY	100

struct pxa168_battery_data {
	struct power_supply battery;
	struct power_supply ac;
} pxa168_battery_data;

static int pxa168_ac_get_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = TRUE;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int pxa168_battery_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	int ret = 0;

	/* XXX Report fake information temporarily */
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = POWER_SUPPLY_STATUS_FULL;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = TRUE;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = FAKE_CAPACITY;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static enum power_supply_property pxa168_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
};

static enum power_supply_property pxa168_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int pxa168_battery_probe(struct platform_device *pdev)
{
	int ret = -1;

	pxa168_battery_data.battery.properties = pxa168_battery_props;
	pxa168_battery_data.battery.num_properties = ARRAY_SIZE(pxa168_battery_props);
	pxa168_battery_data.battery.get_property = pxa168_battery_get_property;
	pxa168_battery_data.battery.name = "battery";
	pxa168_battery_data.battery.type = POWER_SUPPLY_TYPE_BATTERY;

	pxa168_battery_data.ac.properties = pxa168_ac_props;
	pxa168_battery_data.ac.num_properties = ARRAY_SIZE(pxa168_ac_props);
	pxa168_battery_data.ac.get_property = pxa168_ac_get_property;
	pxa168_battery_data.ac.name = "ac";
	pxa168_battery_data.ac.type = POWER_SUPPLY_TYPE_MAINS;

	ret = power_supply_register(&pdev->dev, &pxa168_battery_data.ac);
	if (ret)
		goto err_ac_failed;

	ret = power_supply_register(&pdev->dev, &pxa168_battery_data.battery);
	if (ret)
		goto err_battery_failed;

	return 0;

err_battery_failed:
	power_supply_unregister(&pxa168_battery_data.ac);
err_ac_failed:
	return ret;
}

static int pxa168_battery_remove(struct platform_device *pdev)
{
	power_supply_unregister(&pxa168_battery_data.battery);
	power_supply_unregister(&pxa168_battery_data.ac);
	return 0;
}

static struct platform_driver pxa168_battery_device = {
	.probe = pxa168_battery_probe,
	.remove = pxa168_battery_remove,
	.driver = {
		   .name = "aspenite-battery"}
};

static int __init pxa168_battery_init(void)
{
	return platform_driver_register(&pxa168_battery_device);
}

static void __exit pxa168_battery_exit(void)
{
	platform_driver_unregister(&pxa168_battery_device);
}

module_init(pxa168_battery_init);
module_exit(pxa168_battery_exit);

MODULE_AUTHOR("Mark F. Brown <markb@marvell.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Battery driver for the PXA168");
