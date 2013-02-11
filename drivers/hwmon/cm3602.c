/*
 *  cm3602_asps.c - short distance proximity sensor with ambient light sensor driver
 *
 *  Copyright (C) 2009 Angela Wan<jwan@marvell.com>
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/dmi.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/kthread.h>
#include <linux/semaphore.h>
#include <linux/sensor-input.h>
#include <asm/atomic.h>
#include <mach/cm3602.h>
#include <linux/sensor-input.h>

static struct cm3602_platform_data *pd;

static int power_use;

static inline void cm3602_poweroff(void)
{
	power_use--;
	if (power_use == 0)
		pd->power(0);
}

static void cm3602_poweron(void)
{
	if (power_use == 0)
		pd->power(1);
	power_use++;
}

static void cm3602_report_ps(struct input_dev *idev)
{
	int pout;

	pout = pd->get_pout();
	input_report_abs(idev, ABS_DISTANCE, pout);
	input_sync(idev);
}

static void cm3602_report_as(struct input_dev *idev)
{
	int aout;

	aout = pd->get_aout();
	input_report_abs(idev, ABS_PRESSURE, aout);
	input_sync(idev);
}

static int __devinit cm3602_probe(struct platform_device *pdev)
{
	pd = pdev->dev.platform_data;
	printk("****************sensor CM3602**********\n");
	sensor_input_add(INPUT_PROXIMITY_SENSOR, "cm3602_ps", cm3602_report_ps, NULL, cm3602_poweron,
			cm3602_poweroff);
	sensor_input_add(INPUT_AMBIENT_SENSOR, "cm3602_as", cm3602_report_as, NULL, cm3602_poweron,
			cm3602_poweroff);

	return 0;
}

static int cm3602_remove(struct platform_device *pdev)
{
	sensor_input_del("cm3602");

	return 0;
}

static struct platform_driver cm3602_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "cm3602",
	},
	.probe = cm3602_probe,
	.remove = cm3602_remove,
};

static int __init cm3602_init(void)
{
	return platform_driver_register(&cm3602_driver);
}

static void __exit cm3602_exit(void)
{
	platform_device_unregister(&cm3602_driver);
}

MODULE_DESCRIPTION("CM3602 short distance proximity sensor with ambient light sensor driver");
MODULE_AUTHOR("Angela Wan <jwan@marvell.com>");
MODULE_LICENSE("GPL");

module_init(cm3602_init);
module_exit(cm3602_exit);

