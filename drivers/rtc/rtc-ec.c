/*
 * Real Time Clock interface for StrongARM SA1x00 and XScale PXA2xx
 *
 * Copyright (c) 2000 Nils Faerber
 *
 * Based on rtc.c by Paul Gortmaker
 *
 * Original Driver by Nils Faerber <nils@kernelconcepts.de>
 *
 * Modifications from:
 *   CIH <cih@coventive.com>
 *   Nicolas Pitre <nico@cam.org>
 *   Andrew Christian <andrew.christian@hp.com>
 *
 * Converted to the RTC subsystem and Driver Model
 *   by Richard Purdie <rpurdie@rpsys.net>
 *
 * Support RTC based on EC PIC16F883
 *   by Danny Song <dsong4@marvell.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/rtc.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/string.h>
#include <linux/pm.h>
#include <linux/bitops.h>
#include <linux/i2c.h>


#include <mach/hardware.h>
#include <asm/irq.h>
#include <linux/io.h>

#include <mach/regs-rtc.h>
#include <mach/rtc.h>
#include <mach/power_mcu.h>

#define TIMER_FREQ		CLOCK_TICK_RATE



static void __iomem *rtc_base;
static int irq_1hz = -1, irq_alrm = -1;
static unsigned long rtc_freq = 1024;
static struct rtc_time rtc_alarm;
static DEFINE_SPINLOCK(ec_rtc_lock);


static int ec_rtc_open(struct device *dev)
{
	return 0;
}

static void ec_rtc_release(struct device *dev)
{

}


static int ec_rtc_ioctl(struct device *dev, unsigned int cmd,
		unsigned long arg)
{
	int		ret;
	unsigned long rtc_value;
	unsigned long *ptr_temp;
	void *pvalue;
	struct rtc_time *tm;

	switch (cmd) {
	case RTC_AIE_OFF:
		return 0;
	case RTC_AIE_ON:
		return 0;
	case RTC_UIE_OFF:
		return -EINVAL;
	case RTC_UIE_ON:
		return -EINVAL;
	case RTC_IRQP_READ:
		return put_user(rtc_freq, (unsigned long *)arg);
	case RTC_IRQP_SET:
		pvalue = (void *)arg;
		if (*(unsigned long *)pvalue < 1 ||
			*(unsigned long *)pvalue > TIMER_FREQ)
			return -EINVAL;
		rtc_freq = arg;
		return 0;
	case RTC_PIE_ON:		/*not support*/
		return -EINVAL;
	case RTC_PIE_OFF:		/*not support*/
		return -EINVAL;
	case RTC_ALM_READ:
		tm = (struct rtc_time *) arg;
		power_mcu_read_alarm((u32 *)&rtc_value);
		rtc_time_to_tm(rtc_value, tm);
		return 0;
		break;
	case RTC_ALM_SET:
		spin_lock_irq(&ec_rtc_lock);
		tm = (struct rtc_time *) arg;
		ret = rtc_tm_to_time(tm, &rtc_value);
		if (ret == 0)
			power_mcu_write_alarm((u32)rtc_value);

		spin_unlock_irq(&ec_rtc_lock);
		return ret;
		break;
	case RTC_RD_TIME:
		tm = (struct rtc_time *) arg;
		power_mcu_read_rtc((u32 *)&rtc_value);
		rtc_time_to_tm(rtc_value, tm);
		return 0;
		break;
	case RTC_SET_TIME:
		spin_lock_irq(&ec_rtc_lock);
		tm = (struct rtc_time *) arg;
		ret = rtc_tm_to_time(tm, &rtc_value);
		if (ret == 0)
			power_mcu_write_rtc((u32)rtc_value);
		spin_unlock_irq(&ec_rtc_lock);
		return ret;
		break;
	case RTC_EPOCH_READ:	/*not support*/
		ptr_temp = (unsigned long *) arg;
		*ptr_temp = 0;
		return 0;
		break;
	case RTC_EPOCH_SET:		/*not support*/
		return -EINVAL;
		break;
	}
	return -ENOIOCTLCMD;
}

static int ec_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	unsigned long rtc_value;

	power_mcu_read_rtc((u32 *)&rtc_value);
	rtc_time_to_tm(rtc_value, tm);
	return 0;
}

static int ec_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	unsigned long time;
	int ret;

	ret = rtc_tm_to_time(tm, &time);
	if (ret == 0)
		power_mcu_write_rtc(time);

	return ret;
}

static int ec_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	return 0;
}

static int ec_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	int ret;
	unsigned long time;
	struct pxa168_rtc_platform_data *rtcops;

	spin_lock_irq(&ec_rtc_lock);

	if (alrm->enabled) {
		ret = rtc_tm_to_time(&alrm->time, &time);
		if (ret == 0)
			power_mcu_write_alarm(time);
	} else
		power_mcu_write_alarm(0xFFFFFFFF);

	spin_unlock_irq(&ec_rtc_lock);

	return ret;
}

static int ec_rtc_proc(struct device *dev, struct seq_file *seq)
{
	return 0;
}

static const struct rtc_class_ops ec_rtc_ops = {
	.open = ec_rtc_open,
	.release = ec_rtc_release,
	.ioctl = ec_rtc_ioctl,
	.read_time = ec_rtc_read_time,
	.set_time = ec_rtc_set_time,
	.read_alarm = ec_rtc_read_alarm,
	.set_alarm = ec_rtc_set_alarm,
	.proc = ec_rtc_proc,
};

static int ec_rtc_probe(struct platform_device *pdev)
{
	struct rtc_device *rtc;

	rtc = rtc_device_register(pdev->name, &pdev->dev, &ec_rtc_ops,
				THIS_MODULE);

	if (IS_ERR(rtc))
		return PTR_ERR(rtc);

	platform_set_drvdata(pdev, rtc);

	return 0;
}

static int ec_rtc_remove(struct platform_device *pdev)
{
	struct rtc_device *rtc = platform_get_drvdata(pdev);

	if (rtc)
		rtc_device_unregister(rtc);
	return 0;
}

#ifdef CONFIG_PM
static int ec_rtc_suspend(struct platform_device *pdev,
					pm_message_t state)
{
	return 0;
}

static int ec_rtc_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define ec_rtc_suspend	NULL
#define ec_rtc_resume	NULL
#endif

static struct platform_driver ec_rtc_driver = {
	.probe		= ec_rtc_probe,
	.remove		= ec_rtc_remove,
	.suspend	= ec_rtc_suspend,
	.resume		= ec_rtc_resume,
	.driver		= {
		.name		= "ec-rtc",
	},
};

static int __init ec_rtc_init(void)
{
	return platform_driver_register(&ec_rtc_driver);
}

static void __exit ec_rtc_exit(void)
{
	platform_driver_unregister(&ec_rtc_driver);
}

module_init(ec_rtc_init);
module_exit(ec_rtc_exit);

MODULE_DESCRIPTION("EC Realtime Clock Driver (RTC)");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ec-rtc");
