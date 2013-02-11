/*
 *  linux/drivers/input/touchscreen/sanremo_touch.c
 *
 *  touch screen driver for TTC DKB Platform
 *
 *  Copyright (C) 2009, Marvell Corporation (bin.yang@Marvell.com)
 *  Author: Bin Yang <bin.yang@marvell.com> 
 * 				 Yael Sheli Chemla<yael.s.shemla@marvell.com> 
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/input.h>
#include <asm/uaccess.h>
#include <plat/pxa3xx_pmic.h>
#include <mach/sanremo.h>

struct sanremo_ts_data {
	int				use_count;
	struct timer_list 		*ts_timer;
};

static struct sanremo_ts_data *sanremo_td;
static struct input_dev *sanremo_ts_input_dev;
static unsigned *ts; 
static int *ts_revert;

void sanremo_ts_interrupt(unsigned long event)
{
	if (sanremo_td->use_count > 0) {
		sanremo_td->ts_timer->expires = jiffies+msecs_to_jiffies(100);
		add_timer(sanremo_td->ts_timer);
		sanremo_enable_pen_down_irq(0);
	}

	return;
}

extern int is_android(void);
static void sanremo_ts_timer_handler(unsigned long d)
{
	static u16 tem_x = 0xffff;
	static u16 tem_y = 0xffff;
	int pen_state;

	sanremo_tsi_readxy(&tem_x, &tem_y, &pen_state);
	if (pen_state) {
		if(ts_revert[0] == -1)
			tem_x = ts[1] - tem_x + ts[0];
		if(ts_revert[1] == -1)
			tem_y = ts[3] - tem_y + ts[2];

		input_report_abs(sanremo_ts_input_dev,
				ABS_X, tem_x);
		input_report_abs(sanremo_ts_input_dev,
				ABS_Y, tem_y);
		if (!is_android())
			input_report_abs(sanremo_ts_input_dev,
					ABS_PRESSURE, 255);
		if (is_android()) {
			input_report_key(sanremo_ts_input_dev,
					BTN_TOUCH, 1);
			input_sync(sanremo_ts_input_dev);
		}
		sanremo_td->ts_timer->expires = jiffies+msecs_to_jiffies(100);
		add_timer(sanremo_td->ts_timer);
	} else {
		/* Report a pen up event */
		if (!is_android()) {
			input_report_abs(sanremo_ts_input_dev,
					ABS_PRESSURE, 0);
			input_report_abs(sanremo_ts_input_dev,
					ABS_TOOL_WIDTH, 1);
		}
		if (is_android()) {
			input_report_key(sanremo_ts_input_dev,
					BTN_TOUCH, 0);
			input_sync(sanremo_ts_input_dev);
		}
		sanremo_enable_pen_down_irq(1);
	}

	return;
}

static int sanremo_ts_open(struct input_dev *idev)
{
	if (sanremo_td->use_count++ == 0) {
		sanremo_enable_pen_down_irq(1);
		sanremo_tsi_poweron();
		sanremo_tsi_enable_pen(1);
		sanremo_tsi_enable_tsi(1);
	} 
	return 0;
}

static void sanremo_ts_close(struct input_dev *idev)
{
	if(--sanremo_td->use_count == 0) {
		sanremo_enable_pen_down_irq(0);
		sanremo_tsi_poweroff();
		sanremo_tsi_enable_pen(0);
		sanremo_tsi_enable_tsi(0);
	}
}

static int sanremo_ts_probe(struct platform_device *pdev)
{
	int ret;
	u8 tmp;
	struct sanremo_touch_platform_data *pdata = pdev->dev.platform_data;

	if(sanremo_read(SANREMO_ID, &tmp) < 0)
		return -ENODEV;

	sanremo_td = kzalloc(sizeof(struct sanremo_ts_data), GFP_KERNEL);
	if (!sanremo_td) {
		ret = -ENOMEM;
		goto sanremo_ts_out;
	}

	ts = pdata->ts;
	ts_revert = pdata->ts_revert;

	sanremo_td->use_count = 0;
	sanremo_td->ts_timer = 
		(struct timer_list *)kmalloc(sizeof(struct timer_list), GFP_KERNEL);
	init_timer(sanremo_td->ts_timer);
	sanremo_td->ts_timer->function = sanremo_ts_timer_handler;
//	sanremo_td->ts_timer->data = sanremo_td;

	/* register input device */
	sanremo_ts_input_dev = input_allocate_device();
	if (sanremo_ts_input_dev == NULL) {
		printk(KERN_ERR "%s: failed to allocate input dev\n",
			__FUNCTION__);
		return -ENOMEM;
	}

	sanremo_ts_input_dev->name = "sanremo-ts";

	sanremo_ts_input_dev->phys = "sanremo-ts/input0";
	sanremo_ts_input_dev->dev.parent = &pdev->dev;

	sanremo_ts_input_dev->open = sanremo_ts_open;
	sanremo_ts_input_dev->close = sanremo_ts_close;

	__set_bit(EV_ABS, sanremo_ts_input_dev->evbit);
	__set_bit(ABS_X, sanremo_ts_input_dev->absbit);
	__set_bit(ABS_Y, sanremo_ts_input_dev->absbit);
	__set_bit(ABS_PRESSURE, sanremo_ts_input_dev->absbit);

	__set_bit(EV_SYN, sanremo_ts_input_dev->evbit);
	__set_bit(EV_KEY, sanremo_ts_input_dev->evbit);
	__set_bit(BTN_TOUCH, sanremo_ts_input_dev->keybit);

	input_set_abs_params(sanremo_ts_input_dev, ABS_X, ts[0], ts[1], 0, 0);
	input_set_abs_params(sanremo_ts_input_dev, ABS_Y, ts[2], ts[3], 0, 0);
	input_set_abs_params(sanremo_ts_input_dev, ABS_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(sanremo_ts_input_dev, ABS_TOOL_WIDTH, 0, 15, 0, 0);

	ret = input_register_device(sanremo_ts_input_dev);
	if (ret) {
		printk(KERN_ERR
			"%s: unabled to register input device, ret = %d\n",
			__FUNCTION__, ret);
		return ret;
	}

	/* The Littleton IRQ come from Sanremo. So we just implemate
	 * a callback IRQ function for Sanremo.
	 */
	ret = pmic_callback_register(PMIC_EVENT_TOUCH, sanremo_ts_interrupt);
	if (ret < 0)
		goto pmic_cb_out;

	return 0;

pmic_cb_out:
	input_unregister_device(sanremo_ts_input_dev);
sanremo_ts_out:
	kfree(sanremo_td);

	return ret;
}

static int sanremo_ts_remove(struct platform_device *pdev)
{
	pmic_callback_unregister(PMIC_EVENT_TOUCH, sanremo_ts_interrupt);
	input_unregister_device(sanremo_ts_input_dev);
	kfree(sanremo_td->ts_timer);
	kfree(sanremo_td);

	return 0;
}

#ifdef CONFIG_PM
static int sanremo_ts_resume(struct platform_device *pdev)
{
	sanremo_enable_pen_down_irq(1);
	sanremo_tsi_poweron();

	return 0;
}

static int sanremo_ts_suspend(struct platform_device *pdev, pm_message_t state)
{
	sanremo_tsi_poweroff();
	sanremo_enable_pen_down_irq(0);

	return 0;
}
#else
#define sanremo_ts_resume NULL
#define sanremo_ts_suspend NULL
#endif

static struct platform_driver sanremo_ts_driver = {
	.driver = {
		.name 	= "sanremo_touch",
	},
	.probe		= sanremo_ts_probe,
	.remove		= sanremo_ts_remove,
	.resume 	= sanremo_ts_resume,
	.suspend 	= sanremo_ts_suspend,
};

static int __init sanremo_ts_init(void)
{
	return platform_driver_register(&sanremo_ts_driver);
}

static void __exit sanremo_ts_exit(void)
{
	platform_driver_unregister(&sanremo_ts_driver);
}

module_init(sanremo_ts_init);
module_exit(sanremo_ts_exit);

MODULE_AUTHOR("Bin Yang <bin.yang@marvell.com>");
MODULE_DESCRIPTION("Sanremo touch screen driver");
MODULE_LICENSE("GPL");
