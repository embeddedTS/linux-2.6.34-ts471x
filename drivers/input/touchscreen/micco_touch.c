/*
 *  linux/drivers/input/touchscreen/micco_touch.c
 *
 *  touch screen driver for Littleton Platform
 *
 *  Copyright (C) 2006, Marvell Corporation (fengwei.yin@Marvell.com)
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
#include <asm/mach-types.h>
#include <plat/pxa3xx_pmic.h>
#include <mach/micco.h>

/* micco_ts_opt: 
 * 		=0, 1, 2, 3: 
 * 		sampling rate is higher while the opt value is higher.
 *		Touch timer block time is higher while the opt value is higher.
 */
static int micco_ts_opt = 3;
enum {
	TSI_EVENT_NONE = 0,
	TSI_EVENT_TOUCH = 1,
	TSI_EVENT_LIFT = 2
};

enum {
	TSI_PEN_UNKNOW = 0,
	TSI_PEN_DOWN = 1,
	TSI_PEN_UP = 2
};

struct micco_ts_data {
	spinlock_t			ts_lock;
	struct task_struct		*thread;
	int				suspended;
	int				pen_state;
	int				use_count;
#ifdef CONFIG_TOUCHSCREEN_MICCO_TIMER
	struct timer_list 		*ts_timer;
	int 				step;
#else
 	wait_queue_head_t		ts_wait_queue;
 	struct completion		thread_init;
 	struct completion		thread_exit;
#endif
};

static struct micco_ts_data *micco_td;
static struct input_dev *micco_ts_input_dev;

static int micco_ts_proc_write(struct file *file, const char __user *buffer,
			   unsigned long count, void *data)
{
	static char kbuf[1024];
	int arg;

	if (count >= 1024)
		return -EINVAL;
	if (copy_from_user(kbuf, buffer, count))
		return -EFAULT;

	sscanf(kbuf, "%d", &arg);
	if(arg <= 3)
		micco_ts_opt = arg;
	printk("micco ts opt = %d\n", micco_ts_opt);

	return count;
}

int micco_ts_read(u16 *x,  u16 *y, int *pen_state)
{
	int ret;

	ret = micco_tsi_readxy(x, y, *pen_state);
	return ret;
}

/*
 * The callback function for micco touch.
 * Just turn on the LCD backligtht when PEN DOWN on touch.
 * Leave all the touch X,Y read to a kernel thread.
 *
 * TODO:
 *	1. Move the backlight out of the touch driver.
 *	2. Make backlight align with kernel LCD backlight driver.
 *	3. Make user application handle the touch activity
 *	   turn on the LCD backlight.
 */
void micco_ts_interrupt(unsigned long event)
{
	if (micco_td->use_count > 0) {
#ifdef CONFIG_TOUCHSCREEN_MICCO_TIMER
		if (TSI_PEN_UP ==  micco_td->pen_state) {
			micco_td->pen_state = TSI_PEN_DOWN;
			micco_td->ts_timer->expires = jiffies;
			micco_td->step = 0;
			add_timer(micco_td->ts_timer);
		}
#else
 		wake_up_interruptible(&micco_td->ts_wait_queue);
#endif
	}

	return;
}
EXPORT_SYMBOL(micco_ts_interrupt);

extern int is_android(void);
#ifdef CONFIG_TOUCHSCREEN_MICCO_TIMER
static void micco_ts_timer_handler(unsigned long d)
{
	struct micco_ts_data *ts_data = (struct micco_ts_data *)d;
	static u16 tem_x = 0xffff;
	static u16 tem_y = 0xffff;
	int ret = 0, state;
	u8 val;
	unsigned long next;
	int cur_step;

	cur_step = ts_data->step; 
_start:
	switch (ts_data->step) {
		case 0:
			tem_x = 0xffff;
			tem_y = 0xffff;
			if (0 == micco_read(MICCO_STATUS_A, &val)) {
				/* pen down */
				if (val & 0x40) {
					if (TSI_PEN_UP ==  ts_data->pen_state) {
						ts_data->pen_state = TSI_PEN_DOWN;
						pr_debug("%s: touch pen down!", __func__);
					}

					/* Enable the auto measure of the TSI.
					 * This will disable pen down interupt automatically.
					 */
					micco_tsi_enable_tsi(1);
					ts_data->step = 1;
					if (micco_ts_opt >= 3) {
						mdelay(1);
					}else {
						next = 1;
						break;
					}
				} else {
					ts_data->step = 3;
				}
			}else {
				next = 1;	
				break;
			}
		case 1:
			micco_read(MICCO_EVENT_C, &val);
			if(val & 0x20) {
				ts_data->step = 2;
			}else {
				pr_debug("%s: micco c polling val = 0x%x",
						__func__, val);
			}
			if (micco_ts_opt >= 1) {
				if(ts_data->step == 1) {
					mdelay(1);
					goto _start;
				}
			}else{
				next = 1;
				break;
			}
		case 2:
			pr_debug("%s: micco c val = 0x%x when TSI ready",
					__func__, val);

			micco_ts_read(&tem_x, &tem_y, &state);
			pr_debug("%s: tem_x:0x%x; tem_y:0x%x; pen_state:0x%x",
					__func__, tem_x, tem_y, state);
		case 3:
			micco_tsi_enable_tsi(0);
			micco_td->step = 4;
			if (micco_ts_opt >= 2) {
				mdelay(1);
			}else {
				next = 1;
				break;
			}
		case 4:
			cur_step = 4;
			ret = micco_read(MICCO_STATUS_A, &val);
			pr_debug("%s: after tsi disable, before pen_down" \
					"enabled status a = 0x%x", __func__, val);
			if (val & 0x40) {
				/* Pen still down. The X, Y data is valid.
				 * Report it.
				 */
				if (tem_x > 1000) tem_x = 1000;
				if (tem_y > 1000) tem_y = 1000;

				if (machine_is_littleton())
					tem_x = 1000 - (tem_x & 0xfff);
				if (machine_is_tavorevb())
					tem_y = 1000 - (tem_y & 0xfff);
				input_report_abs(micco_ts_input_dev,
						ABS_X, tem_x & 0xfff);
				input_report_abs(micco_ts_input_dev,
						ABS_Y, tem_y & 0xfff);
				if (!is_android())
					input_report_abs(micco_ts_input_dev,
						ABS_PRESSURE, 255);
				if (is_android()) {
					input_report_key(micco_ts_input_dev,
							BTN_TOUCH, 1);
					input_sync(micco_ts_input_dev);
				}

				next = 1;
			} else {	/* Pen is up now */
				/* Report a pen up event */

				if (!is_android()) {
					input_report_abs(micco_ts_input_dev,
							ABS_PRESSURE, 0);
					input_report_abs(micco_ts_input_dev,
							ABS_TOOL_WIDTH, 1);
				}
				if (is_android()) {
					input_report_key(micco_ts_input_dev,
							BTN_TOUCH, 0);
					input_sync(micco_ts_input_dev);
				}
				pr_debug("%s: report pen up", __func__);
				micco_tsi_enable_pen(1);
				ts_data->pen_state = TSI_PEN_UP;
				next = 0xffffffff;
			}
			micco_td->step = 0;
			break;
		default:
			break;		
	}

	if(next != 0xffffffff) {
		ts_data->ts_timer->expires = jiffies+msecs_to_jiffies(next);
		ts_data->ts_timer->data = ts_data;
		add_timer(ts_data->ts_timer);
	}

	return;
}
#else
/*
 * The touchscreen sample reader thread
 */
static int micco_ts_thread(void *d)
{
	struct micco_ts_data *ts_data = d;
	u16 tem_x = 0xffff;
	u16 tem_y = 0xffff;
	int ret = 0, state;
	u8 val;

	DEFINE_WAIT(ts_wait);
	/* set up thread context */

	ts_data->thread = current;
	daemonize("micco_ts_thread");

	/* init is complete */
	complete(&ts_data->thread_init);

	/* touch reader loop */
	while (1) {
		/* if the pen state is up, sleep */
		if (TSI_PEN_UP == ts_data->pen_state) {
			prepare_to_wait(&ts_data->ts_wait_queue,
				&ts_wait, TASK_INTERRUPTIBLE);

			if (TSI_PEN_UP == ts_data->pen_state)
				schedule();

			finish_wait(&ts_data->ts_wait_queue, &ts_wait);
		}

		try_to_freeze();

		/* Now the pen state is down */
		ret = micco_read(MICCO_STATUS_A, &val);

		/* pen down */
		if (val & 0x40) {
			if (TSI_PEN_UP ==  ts_data->pen_state) {
				ts_data->pen_state = TSI_PEN_DOWN;
				pr_debug("%s: touch pen down!", __func__);
			}

			/* Enable the auto measure of the TSI.
			 * This will disable pen down interupt automatically.
			 */
			micco_tsi_enable_tsi(1);
			mdelay(1);
			micco_read(MICCO_EVENT_C, &val);
			while (!(val & 0x20)) {
				pr_debug("%s: micco c val = 0x%x",
					__func__, val);
				micco_read(MICCO_EVENT_C, &val);
				mdelay(1);
			}
			pr_debug("%s: micco c val = 0x%x when TSI ready",
					__func__, val);

			micco_ts_read(&tem_x, &tem_y, &state);
			pr_debug("%s: tem_x:0x%x; tem_y:0x%x; pen_state:0x%x",
					__func__, tem_x, tem_y, state);
		}

		micco_tsi_enable_tsi(0);
		/* After disable the auto tsi, need wait a while to read the
		 * correct pen_down state. Currently, just wait for 1 ms.
		 */

		mdelay(1);
		ret = micco_read(MICCO_STATUS_A, &val);
		pr_debug("%s: after tsi disable, before pen_down" \
				"enabled status a = 0x%x", __func__, val);

		if (val & 0x40) {
			/* Pen still down. The X, Y data is valid.
			 * Report it.
			 */
			if (tem_x > 1000) tem_x = 1000;
			if (tem_y > 1000) tem_y = 1000;
			if (tem_x < 0 ) tem_x = 0;
			if (tem_y < 0 ) tem_y = 0;	

			if (machine_is_littleton())
				tem_x = 1000 - (tem_x & 0xfff);
			if (machine_is_tavorevb())
				tem_y = 1000 - (tem_y & 0xfff);
			input_report_abs(micco_ts_input_dev,
					ABS_X, tem_x & 0xfff);
			input_report_abs(micco_ts_input_dev,
					ABS_Y, tem_y & 0xfff);
			input_report_abs(micco_ts_input_dev,
					ABS_PRESSURE, 255);
			if (is_android())
				input_report_key(micco_ts_input_dev,
						BTN_TOUCH, 1);
			msleep(8);

			/* don't enable the pen down interrupt because
			 * the pen down interrupt enabling for Micco will
			 * trigger an pen down interrupt. polling to get
			 * the next pen X, Y.
			 */
			continue;
		} else {	/* Pen is up now */
			/* Report a pen up event */
			input_report_abs(micco_ts_input_dev,
					ABS_PRESSURE, 0);
			input_report_abs(micco_ts_input_dev,
					ABS_TOOL_WIDTH, 1);
			if (is_android()) {
				input_report_key(micco_ts_input_dev,
						BTN_TOUCH, 0);
				input_sync(micco_ts_input_dev);
			}
			pr_debug("%s: report pen up", __func__);
			ts_data->pen_state = TSI_PEN_UP;
			micco_tsi_enable_pen(1);
		}

		if (!ts_data->thread)
			break;
	}

	/* Always enable the pen down detection before exit from thread */
	micco_tsi_enable_pen(1);
	complete_and_exit(&ts_data->thread_exit, 0);
	return 0;
}
#endif

static int micco_ts_open(struct input_dev *idev)
{
	int ret = 0;
	unsigned long flags;

	pr_debug("%s: enter", __func__);

	if (micco_td->suspended) {
		printk(KERN_INFO "touch has been suspended!\n");
		return -1;
	}

	spin_lock_irqsave(&micco_td->ts_lock, flags);
	if (micco_td->use_count++ == 0) {
		spin_unlock_irqrestore(&micco_td->ts_lock, flags);

#ifdef CONFIG_TOUCHSCREEN_MICCO_TIMER
		micco_td->ts_timer = 
			(struct timer_list *)kmalloc(sizeof(struct timer_list), GFP_KERNEL);
		init_timer(micco_td->ts_timer);
		micco_td->ts_timer->function = micco_ts_timer_handler;
		micco_td->ts_timer->data = micco_td;
#else
 		init_completion(&micco_td->thread_init);
 		ret = kernel_thread(micco_ts_thread, micco_td, 0);
 		if (ret < 0)
 			return ret;
 
 		wait_for_completion(&micco_td->thread_init);
#endif
	} else {
		spin_unlock_irqrestore(&micco_td->ts_lock, flags);
	}

	return 0;
}

/* Kill the touchscreen thread and stop the touch digitiser. */
static void micco_ts_close(struct input_dev *idev)
{
	unsigned long flags;

	pr_debug("%s: enter with use count = %d", __func__,
		micco_td->use_count);

	spin_lock_irqsave(&micco_td->ts_lock, flags);
	if (--micco_td->use_count == 0) {
		spin_unlock_irqrestore(&micco_td->ts_lock, flags);

#ifdef CONFIG_TOUCHSCREEN_MICCO_TIMER
		kfree(micco_td->ts_timer);
#else
		pr_debug("%s: kill thread", __func__);
		/* kill thread */
		if (micco_td->thread) {
			init_completion(&micco_td->thread_exit);
			micco_td->thread = NULL;
			wake_up_interruptible(&micco_td->ts_wait_queue);
			wait_for_completion(&micco_td->thread_exit);
		}
#endif
	} else {
		spin_unlock_irqrestore(&micco_td->ts_lock, flags);
	}
}

static int micco_ts_probe(struct platform_device *pdev)
{
	int ret;
	struct proc_dir_entry *micco_ts_proc_entry;	

	micco_td = kzalloc(sizeof(struct micco_ts_data), GFP_KERNEL);
	if (!micco_td) {
		ret = -ENOMEM;
		goto micco_ts_out;
	}
	spin_lock_init(&micco_td->ts_lock);
#ifndef CONFIG_TOUCHSCREEN_MICCO_TIMER
	init_waitqueue_head(&micco_td->ts_wait_queue);
#endif
	micco_td->pen_state = TSI_PEN_UP;
	micco_td->suspended = 0;
	micco_td->use_count = 0;

	/* register input device */
	micco_ts_input_dev = input_allocate_device();
	if (micco_ts_input_dev == NULL) {
		printk(KERN_ERR "%s: failed to allocate input dev\n",
			__FUNCTION__);
		return -ENOMEM;
	}

	micco_ts_input_dev->name = "micco-ts";

	micco_ts_input_dev->phys = "micco-ts/input0";
	micco_ts_input_dev->dev.parent = &pdev->dev;

	micco_ts_input_dev->open = micco_ts_open;
	micco_ts_input_dev->close = micco_ts_close;

	__set_bit(EV_ABS, micco_ts_input_dev->evbit);
	__set_bit(ABS_X, micco_ts_input_dev->absbit);
	__set_bit(ABS_Y, micco_ts_input_dev->absbit);
	__set_bit(ABS_PRESSURE, micco_ts_input_dev->absbit);

	__set_bit(EV_SYN, micco_ts_input_dev->evbit);
	__set_bit(EV_KEY, micco_ts_input_dev->evbit);
	__set_bit(BTN_TOUCH, micco_ts_input_dev->keybit);

	input_set_abs_params(micco_ts_input_dev, ABS_X, 0, 1000, 0, 0);
	input_set_abs_params(micco_ts_input_dev, ABS_Y, 0, 1000, 0, 0);
	input_set_abs_params(micco_ts_input_dev, ABS_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(micco_ts_input_dev, ABS_TOOL_WIDTH, 0, 15, 0, 0);

	ret = input_register_device(micco_ts_input_dev);
	if (ret) {
		printk(KERN_ERR
			"%s: unabled to register input device, ret = %d\n",
			__FUNCTION__, ret);
		return ret;
	}

	/* The Littleton IRQ come from Micco. So we just implemate
	 * a callback IRQ function for Micco.
	 */
	ret = pmic_callback_register(PMIC_EVENT_TOUCH, micco_ts_interrupt);
	if (ret < 0)
		goto pmic_cb_out;

	/* Enable the touch IRQ here for lcd backlight */
	micco_enable_pen_down_irq(1);
	micco_tsi_poweron();
	micco_ts_proc_entry = create_proc_entry("driver/micco_touch", 0, NULL);
	if (micco_ts_proc_entry) { 
		micco_ts_proc_entry->write_proc = micco_ts_proc_write;
	} 
	return 0;

pmic_cb_out:
	input_unregister_device(micco_ts_input_dev);
micco_ts_out:
	kfree(micco_td);

	return ret;
}

static int micco_ts_remove(struct platform_device *pdev)
{
	input_unregister_device(micco_ts_input_dev);

	return 0;
}

#ifdef CONFIG_PM
static int micco_ts_resume(struct platform_device *pdev)
{
	micco_enable_pen_down_irq(1);
	micco_tsi_poweron();
	micco_td->suspended = 0;

	return 0;
}

static int micco_ts_suspend(struct platform_device *pdev, pm_message_t state)
{
	micco_td->suspended = 1;
	micco_tsi_poweroff();
	micco_enable_pen_down_irq(0);

	return 0;
}
#else
#define micco_ts_resume NULL
#define micco_ts_suspend NULL
#endif

static struct platform_driver micco_ts_driver = {
	.driver = {
		.name 	= "micco-touch",
	},
	.probe		= micco_ts_probe,
	.remove		= micco_ts_remove,
	.resume 	= micco_ts_resume,
	.suspend 	= micco_ts_suspend,
};

static int __init micco_ts_init(void)
{
	return platform_driver_register(&micco_ts_driver);
}

static void __exit micco_ts_exit(void)
{
	/* We move these codes here because we want to detect the
	 * pen down event even when touch driver is not opened.
	 */
	micco_tsi_poweroff();
	micco_enable_pen_down_irq(0);
	pmic_callback_unregister(PMIC_EVENT_TOUCH, micco_ts_interrupt);

	platform_driver_unregister(&micco_ts_driver);
}

module_init(micco_ts_init);
module_exit(micco_ts_exit);

MODULE_AUTHOR("Yin, Fengwei <fengwei.yin@marvell.com>");
MODULE_DESCRIPTION("Littleton Platfrom touch screen driver");
MODULE_LICENSE("GPL");
