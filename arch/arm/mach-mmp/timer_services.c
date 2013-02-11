/*
 * linux/arch/arm/mach-ttc/timer_services.c
 *
 * Support for the MMP Development Platform
 *
 * Copyright (C) 2008 Marvell International Ltd.
 *
 * 2009-08-1: Ofer Zaarur <ozaarur@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 *
 * the driver provides timer services in parallel to the OS timer,currently it
 * provices three timers and can be expend to six timer. each timer has its
 * own resolution and can be enable or disable independently.
 * the timer services driver can be set to different resolution that are not
 * related to the OS timer. with very high resolution that can be changes
 * accordingly to meet driver client requirements.in addition you can handle
 * the resolution in your driver and just access directly the counter, in this
 * case the driver will need to handle the translation for the required scale.
 *
 * for performance aspect you should make sure disable the timer when in not
 * been used to minimize the interrupt. please be advicve that using periodic
 * delay in your driver could introduce system delay.
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <mach/timer_services.h>
#include <linux/delay.h>





unsigned long resolution;
unsigned int jiff;
static int resolution_list[3] = { RESOLUTION_0, RESOLUTION_1, RESOLUTION_2};
static struct timer_dev timers[3];
static seqlock_t xtimer_lock[4]; /* 1-3 individual lock and 4 global lock*/

static irqreturn_t timer_services_interrupt(int irq, void *dev_id)
{
struct timer_dev *dev = (struct timer_dev *) dev_id;
struct timer_device *timer = dev->timer;

write_seqlock(&xtimer_lock[timer->counter_id]);
do {
	/* Clear timer interrupt   */
	writel((1 << timer->counter_id), timer->mmio_base + TMR_ICR1_OFFSET);
	/* Calculate the next matching value */
	resolution = readl(timer->mmio_base + TMR_T1_M0_OFFSET +
	(MATCH_REGISTER_OFFSET * timer->counter_id) +
	(4 * timer->counter_id));
	 /* Read current matching value */
	resolution += timer->resolution;
	/* NOTE: We do not disable the timer before updating
	   TMR_T1_M0. Ignore what the specs says... */
	writel(resolution, timer->mmio_base + TMR_T1_M0_OFFSET +
	(MATCH_REGISTER_OFFSET * timer->counter_id)  + 4 * timer->counter_id);
	++timer->jiff;

} while ((signed long)(resolution - readl(timer->mmio_base +
		TMR_CR1_OFFSET)) <= 8);
write_sequnlock(&xtimer_lock[timer->counter_id]);
return IRQ_HANDLED;
}

/**
 * check_id - verify the ID number
 */

int check_id(int id)
{
if ((id) > 2 || ((id) < 0)) {
	printk(KERN_ERR "Timer ID error, please check timer ID\n");
	return -EINVAL;
} else
	return 0;
}

/**
 * timer_enable - enable the Timer counter
 *
 * Turn on the Timer ID.
 * this function provides several modes to enable the timer:
 * Continuasly running mode the counter will run in free mode and will generate
 * interrupts accordingly.
 * on demand mode the function provides period parameter that will be send with
 * enable request,the timer will stop and will turn to disable mode after the
 * requester period.
 *
 * period are in msec.
 */

void timer_services_enable(int id, int period)
{
	unsigned int tmp;

if (check_id(id) > 0) {
	printk(KERN_ERR "Timer ID error, please check timer ID\n");
	return;
}

if (period >= -1) {
	write_seqlock(&xtimer_lock[id+1]);
	write_seqlock(&xtimer_lock[id]);
	tmp = readl(timers[id].timer->mmio_base +  TMR_IER1_OFFSET);
	writel(tmp | (1 << id), timers[id].timer->mmio_base +
						TMR_IER1_OFFSET);
	tmp = readl(timers[id].timer->mmio_base + TMR_CR1_OFFSET);
	tmp += timers[id].timer->resolution;
	writel(tmp, timers[id].timer->mmio_base +  TMR_T1_M0_OFFSET +
		(MATCH_REGISTER_OFFSET * timers[id].timer->counter_id) +
					4 * timers[id].timer->counter_id);
	write_sequnlock(&xtimer_lock[id+1]);
	write_sequnlock(&xtimer_lock[id]);
	if (period != -1) {
		mdelay(period);
		write_seqlock(&xtimer_lock[id+1]);
		write_seqlock(&xtimer_lock[id]);
		tmp = readl(timers[id].timer->mmio_base +
			TMR_IER1_OFFSET);
		writel(tmp & ~(1 << id), timers[id].timer->mmio_base +
				TMR_IER1_OFFSET);
		write_sequnlock(&xtimer_lock[id+1]);
		write_sequnlock(&xtimer_lock[id]);
		}
	}

else {
	printk(KERN_ERR "The period parameter for timer-%d \
	is out of range :( \n", id);
	}
}
EXPORT_SYMBOL(timer_services_enable);

/**
 * timer_disable - shut down the Timer ID
 *
 * Turn off the Timer counter, optionally powering it down.
 */

void timer_services_disable(int id)
{
	unsigned int tmp;

if (check_id(id) > 0) {
	printk(KERN_ERR "Timer ID error, please check timer ID\n");
	return;
}

write_seqlock(&xtimer_lock[id+1]);
tmp = readl(timers[id].timer->mmio_base +  TMR_IER1_OFFSET);
writel(tmp & ~(1 << id), timers[id].timer->mmio_base +  TMR_IER1_OFFSET);
write_sequnlock(&xtimer_lock[id+1]);

}
EXPORT_SYMBOL(timer_services_disable);

/**
 * timer_services_counter_read
 *
 * return the counter value.
 */

unsigned int timer_services_counter_read(int id)
{
	unsigned int ret;

if (check_id(id) > 0) {
	printk(KERN_ERR "Timer ID error, please check timer ID\n");
	return -EINVAL;
}

ret = readl(timers[id].timer->mmio_base + TMR_CR1_OFFSET);
return ret;
}
EXPORT_SYMBOL(timer_services_counter_read);

/**
 * timer_config - configure Timer counter settings
 * @speed: counter speed
 *
 */

int timer_config(struct timer_dev *dev, u32 period)
{
/* configuration option */
return 0;
}


/**
 * timer_init - setup the Timer counter
 *
 * initialise and claim resources for the Timer counter.
 *The driver uses timer0 and counter1 within three matches registers
 * Returns:
 *   %-ENODEV	if the Timer counter is unavailable
 *   %-EBUSY	if the resources are already in use
 *   %0		on success
 */

int timer_init(struct timer_dev *dev, int id, u32 resolution)
{
	struct timer_device *timer;
	unsigned long tmp;

	timer = timer_request(id, "TIMER");
	if (timer == NULL)
		return -ENODEV;

	if (check_id(id) > 0) {
		printk(KERN_ERR "Timer ID error, please check timer ID\n");
	return -EINVAL;
}
dev->timer = timer;
dev->counter = id;

	tmp = readl(timer->mmio_base + TMR_CMR_OFFSET);

	writel((tmp | 1<<1), timer->mmio_base + TMR_CMR_OFFSET); /* Setup
	conter to free running mode - UNDOCUMENTED */

	writel(0, timer->mmio_base +
	       TMR_PLVR1_OFFSET); /* Setup preload value */


	writel(0, timer->mmio_base +
	       TMR_PLCR1_OFFSET); /* Set to free running mode for preload
				     control */

	writel(0x7, timer->mmio_base +
	       TMR_ICR1_OFFSET); /* Clear any pending match status {0-2}
				    for timer 0 */

	writel(1, timer->mmio_base +
	       TMR_CVWR1_OFFSET); /* Cause the latch */

	tmp = readl(timer->mmio_base +
		    TMR_CVWR1_OFFSET);
	tmp += resolution;

	writel(tmp, timer->mmio_base +
	       TMR_T1_M0_OFFSET + (MATCH_REGISTER_OFFSET * timer->counter_id));
	       /* Prime the match register */

	/*
	 * Enable match irq for the timer services.
	 */

	writel(3, timer->mmio_base +
	       TMR_CER_OFFSET); /* Enable timer */

	return 0;

}
EXPORT_SYMBOL(timer_init);

/**
 * timer_exit - undo the effects of timer_init
 *
 * release and free resources for the Timer counter.
 */
void timer_exit(struct timer_dev *dev)
{
struct timer_device *timer = dev->timer;
timer_services_disable(timer->counter_id);
timer_free(timer);

}
EXPORT_SYMBOL(timer_exit);

static LIST_HEAD(timer_list);

struct timer_device *timer_request(int id, const char *label)
{

	struct timer_device *timer = NULL;

	if (check_id(id) > 0) {
		printk(KERN_ERR "Timer ID error, please check timer ID\n");
		return NULL;
	}

	write_seqlock(&xtimer_lock[id + 1]);

	list_for_each_entry(timer, &timer_list, node) {
		if (timer->counter_id == id && timer->use_count == 0) {
			timer->use_count++;
			timer->label = label;
			break;
		}
	}

	write_sequnlock(&xtimer_lock[id + 1]);
	if (timer->counter_id != id)
		return NULL;
	return timer;
}
EXPORT_SYMBOL(timer_request);

void timer_free(struct timer_device *timer)
{
if (check_id(timer->counter_id) > 0) {
	printk(KERN_ERR "Timer ID error, please check timer ID\n");
	return;
}

	write_seqlock(&xtimer_lock[timer->counter_id]);
	if (timer->use_count) {
		timer->use_count--;
		timer->label = NULL;
	} else
		dev_err(&timer->pdev->dev, "device already free\n");
	write_sequnlock(&xtimer_lock[timer->counter_id]);
}
EXPORT_SYMBOL(timer_free);


static int __devinit timer_probe(struct platform_device *pdev, int type)
{

	struct resource *res;
	struct timer_device *timer;
	int ret = 0;

	timer = kzalloc(sizeof(struct timer_device), GFP_KERNEL);
	if (timer == NULL) {
		dev_err(&pdev->dev, "failed to allocate memory");
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "no memory resource defined\n");
		ret = -ENODEV;
	}

	timer->mmio_base = ioremap_nocache(res->start, SZ_256);

	if (timer->mmio_base == NULL) {
		dev_err(&pdev->dev, "failed to ioremap() registers\n");
		ret = -ENODEV;
		goto err_free_mem;
	}

	timer->irq = platform_get_irq(pdev, 0);
	if (timer->irq < 0) {
		dev_err(&pdev->dev, "no IRQ resource defined\n");
		ret = -ENODEV;
		goto err_free_io;
	}

	ret = request_irq(timer->irq, timer_services_interrupt, 0 ,
		"Timer Services", timer);

	if (ret)
		goto err_free_io;

	if (check_id(pdev->id) > 0) {
		dev_err(&pdev->dev, "Timer ID error, please check timer ID\n");
		ret = -EINVAL;
		goto err_free_io;
	}

	timer->counter_id = pdev->id ;
	timers[timer->counter_id].timer = timer;
	timer->jiff = 0;
	timer->resolution = resolution_list[timer->counter_id];
	timer->use_count = 0;
	timer->type = type;

	write_seqlock(&xtimer_lock[timer->counter_id]);
	list_add(&timer->node, &timer_list);
	write_sequnlock(&xtimer_lock[timer->counter_id]);
	/* timer init*/
	timer_init((struct timer_dev *)timer, 0 , timer->resolution);
	platform_set_drvdata(pdev, timer);
	return 0;

err_free_io:
	iounmap(timer->mmio_base);
err_free_mem:
	release_mem_region(res->start, res->end - res->start + 1);

	return ret;
}

static int timer_remove(struct platform_device *pdev)
{
	struct resource *res;
	struct timer_device *timer;

	timer = platform_get_drvdata(pdev);
	if (timer == NULL)
		return -ENODEV;

	iounmap(timer->mmio_base);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, res->end - res->start + 1);

	if (check_id(pdev->id) > 0) {
		dev_err(&pdev->dev, "Timer ID error, please check timer ID\n");
		return -EINVAL;
	}

	write_seqlock(&xtimer_lock[timer->counter_id]);
	list_del(&timer->node);
	write_sequnlock(&xtimer_lock[timer->counter_id]);

	kfree(timer);
	return 0;
}

static int mmp_timer_probe(struct platform_device *pdev)
{
	return timer_probe(pdev, MMP_TIMER);
}


#ifdef CONFIG_PM
/*
 * Basic power management.
 */

static int timer_services_suspend(struct platform_device *pdev,
		pm_message_t state)
{
	struct timer_device *timer = platform_get_drvdata(pdev);

	timer_free(timer);

	return 0;
}

static int timer_services_resume(struct platform_device *pdev)
{
	struct timer_device *timer = platform_get_drvdata(pdev);

	timer_init((struct timer_dev *)timer, 0 , timer->resolution);
	return 0;
}
#else
#define timer_services_suspend	NULL
#define timer_services_resume	NULL
#endif
static struct platform_driver mmp_timer_services = {
	.driver		= {
	.name	= "mmp-timer-services",
	},
	.probe		= mmp_timer_probe,
	.remove		= timer_remove,
	.suspend	= timer_services_suspend,
	.resume		= timer_services_resume,
};

static int __init timer_services_init(void)
{
	int ret ;
	ret = platform_driver_register(&mmp_timer_services);

	if (ret) {
		printk(KERN_ERR "failed to register mmp_timer_driver");
		return ret;
	}

	return ret;
}

static void __exit timer_services_exit(void)
{
	platform_driver_unregister(&mmp_timer_services);
}

unsigned long get_jiff(int id)
{
return timers[id].timer->jiff;
}
EXPORT_SYMBOL(get_jiff);

module_init(timer_services_init);
module_exit(timer_services_exit);

MODULE_DESCRIPTION("Timer driver services");
MODULE_AUTHOR("Ofer Zaarur");
MODULE_LICENSE("GPL");
