/*
 * linux/arch/arm/mach-mmp/time.c
 *
 *   Support for clocksource and clockevents
 *
 * Copyright (C) 2008 Marvell International Ltd.
 * All rights reserved.
 *
 *   2008-04-11: Jason Chagas <Jason.chagas@marvell.com>
 *   2008-10-08: Bin Yang <bin.yang@marvell.com>
 *
 * The timers module actually includes three timers, each timer with upto
 * three match comparators. Timer #0 is used here in free-running mode as
 * the clock source, and match comparator #1 used as clock event device.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/clockchips.h>

#include <linux/io.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/cnt32_to_63.h>

#include <asm/mach/time.h>
#include <mach/addr-map.h>
#include <mach/regs-apbc.h>
#include <mach/regs-timers.h>
#include <mach/irqs.h>
#include <mach/cputype.h>

#define TIMERS_VIRT_BASE	TIMERS1_VIRT_BASE

#define MAX_DELTA		(0xfffffffe)
#define MIN_DELTA		(16)

#define TCR2NS_SCALE_FACTOR	10

static unsigned long tcr2ns_scale;

static void __init set_tcr2ns_scale(unsigned long tcr_rate)
{
	unsigned long long v = 1000000000ULL << TCR2NS_SCALE_FACTOR;
	do_div(v, tcr_rate);
	tcr2ns_scale = v;
	/*
	 * We want an even value to automatically clear the top bit
	 * returned by cnt32_to_63() without an additional run time
	 * instruction. So if the LSB is 1 then round it up.
	 */
	if (tcr2ns_scale & 1)
		tcr2ns_scale++;
}

/*
 * Note: the timer needs some delay to stablize the counter capture
 */
static inline uint32_t timer_read(void)
{
	volatile int delay = 2;
	unsigned long flags;
	uint32_t val = 0;

	local_irq_save(flags);

#ifdef CONFIG_PXA_32KTIMER
	/* 32k timer needs more delay due to brain-damaged hardware :-( */
	//if (CLOCK_TICK_RATE == 32768)
		//delay = 100;*/
	val = __raw_readl(TIMERS_VIRT_BASE + TMR_CR(0));
#else

	__raw_writel(1, TIMERS_VIRT_BASE + TMR_CVWR(0));

	while (delay--) {
		val = __raw_readl(TIMERS_VIRT_BASE + TMR_CVWR(0));
	}

	val = __raw_readl(TIMERS_VIRT_BASE + TMR_CVWR(0));
#endif
	local_irq_restore(flags);

	return val;
}

unsigned int read_timer(void)
{
	return (unsigned int)timer_read();
}

#if 0 /* FIX-ME: rzaman: figure out how to resolve this */
unsigned long read_persistent_clock(void)
{
	unsigned int ticks;
	unsigned long sec;
	ticks = read_timer();
	sec = (unsigned long)ticks >> 15;
	return sec;
}
#endif

unsigned long long sched_clock(void)
{
	unsigned long long v = cnt32_to_63(timer_read());
	return (v * tcr2ns_scale) >> TCR2NS_SCALE_FACTOR;
}

static irqreturn_t timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *c = dev_id;

	/* disable and clear pending interrupt status */
	__raw_writel(0x0, TIMERS_VIRT_BASE + TMR_IER(0));
	__raw_writel(0x1, TIMERS_VIRT_BASE + TMR_ICR(0));
	c->event_handler(c);
	return IRQ_HANDLED;
}

static int timer_set_next_event(unsigned long delta,
				struct clock_event_device *dev)
{
	unsigned long flags, next;

	local_irq_save(flags);

	/* clear pending interrupt status and enable */
	__raw_writel(0x01, TIMERS_VIRT_BASE + TMR_ICR(0));
	__raw_writel(0x01, TIMERS_VIRT_BASE + TMR_IER(0));

	next = timer_read() + delta;
	__raw_writel(next, TIMERS_VIRT_BASE + TMR_TN_MM(0, 0));

	local_irq_restore(flags);
	return 0;
}

static void timer_set_mode(enum clock_event_mode mode,
			   struct clock_event_device *dev)
{
	unsigned long flags;

	local_irq_save(flags);
	switch (mode) {
	case CLOCK_EVT_MODE_ONESHOT:
	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
		/* disable the matching interrupt */
		__raw_writel(0x00, TIMERS_VIRT_BASE + TMR_IER(0));
		break;
	case CLOCK_EVT_MODE_RESUME:
	case CLOCK_EVT_MODE_PERIODIC:
		break;
	}
	local_irq_restore(flags);
}

static struct clock_event_device ckevt = {
	.name		= "clockevent",
	.features	= CLOCK_EVT_FEAT_ONESHOT,
#ifdef CONFIG_PXA_32KTIMER
	.shift		= 32,
	.rating		= 150,
#else
	.shift		= 32,
	.rating		= 200,
#endif
	.set_next_event	= timer_set_next_event,
	.set_mode	= timer_set_mode,
};

static cycle_t clksrc_read(struct clocksource *cs)
{
	return timer_read();
}

static struct clocksource cksrc = {
	.name		= "clocksource",
#ifdef CONFIG_PXA_32KTIMER
	.shift		= 10,
	.rating		= 150,
#else
	.shift		= 20,
	.rating		= 200,
#endif
	.read		= clksrc_read,
	.mask		= CLOCKSOURCE_MASK(32),
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
};

static void __init timer_config(void)
{
	uint32_t ccr = __raw_readl(TIMERS_VIRT_BASE + TMR_CCR);
	uint32_t cer = __raw_readl(TIMERS_VIRT_BASE + TMR_CER);
	uint32_t cmr = __raw_readl(TIMERS_VIRT_BASE + TMR_CMR);

	__raw_writel(cer & ~0x1, TIMERS_VIRT_BASE + TMR_CER); /* disable */

	ccr &= (cpu_is_mmp2()) ? TMR_CCR_CS_0(0) : TMR_CCR_CS_0(3);
	__raw_writel(ccr, TIMERS_VIRT_BASE + TMR_CCR);

	/* free-running mode */
	__raw_writel(cmr | 0x01, TIMERS_VIRT_BASE + TMR_CMR);

	__raw_writel(0x0, TIMERS_VIRT_BASE + TMR_PLCR(0)); /* free-running */
	__raw_writel(0x7, TIMERS_VIRT_BASE + TMR_ICR(0));  /* clear status */
	__raw_writel(0x0, TIMERS_VIRT_BASE + TMR_IER(0));

	/* enable timer counter */
	__raw_writel(cer | 0x01, TIMERS_VIRT_BASE + TMR_CER);
}

static struct irqaction timer_irq = {
	.name		= "timer",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= timer_interrupt,
	.dev_id		= &ckevt,
};

static void __init timer_init(int irq)
{
	timer_config();

	set_tcr2ns_scale(CLOCK_TICK_RATE);

	ckevt.mult = div_sc(CLOCK_TICK_RATE, NSEC_PER_SEC, ckevt.shift);
	ckevt.max_delta_ns = clockevent_delta2ns(MAX_DELTA, &ckevt);
	ckevt.min_delta_ns = clockevent_delta2ns(MIN_DELTA, &ckevt);
	ckevt.cpumask = cpumask_of(0);

	cksrc.mult = clocksource_hz2mult(CLOCK_TICK_RATE, cksrc.shift);

	setup_irq(irq, &timer_irq);

	clocksource_register(&cksrc);
	clockevents_register_device(&ckevt);
}

static void __init pxa168_timer_init(void)
{
	uint32_t clk_rst;

	/* this is early, we have to initialize the CCU registers by
	 * ourselves instead of using clk_* API. Clock rate is defined
	 * by APBC_TIMERS_FNCLKSEL and enabled free-running
	 */
	__raw_writel(APBC_APBCLK | APBC_RST, APBC_PXA168_TIMERS);

#ifdef CONFIG_PXA_32KTIMER
	/* 32KHz, bus/functional clock enabled, release reset */
	clk_rst = APBC_APBCLK | APBC_FNCLK | APBC_FNCLKSEL(1);
#else
	/* 3.25MHz, bus/functional clock enabled, release reset */
	clk_rst = APBC_APBCLK | APBC_FNCLK | APBC_FNCLKSEL(3);
#endif
	__raw_writel(clk_rst, APBC_PXA168_TIMERS);

	timer_init(IRQ_PXA168_TIMER1);
}

#ifdef CONFIG_PM
struct tmr_regs {
	unsigned int		tmr_ccr;
	unsigned int		tmr_tn_mm[9];
	unsigned int		tmr_crn[3];
	unsigned int		tmr_srn[3];
	unsigned int		tmr_iern[3];
	unsigned int		tmr_plvrn[3];
	unsigned int		tmr_plcrn[3];
	unsigned int		tmr_wmer;
	unsigned int		tmr_wmr;
	unsigned int		tmr_wvr;
	unsigned int		tmr_wsr;
	unsigned int		tmr_icrn[3];
	unsigned int		tmr_wicr;
	unsigned int		tmr_cer;
	unsigned int		tmr_cmr;
	unsigned int		tmr_ilrn[3];
	unsigned int		tmr_wcr;
	unsigned int		tmr_wfar;
	unsigned int		tmr_wsar;
	unsigned int		tmr_cvwrn[3];
};

static struct tmr_regs tmr_saved_regs;
/* static struct tmr_regs tmr2_saved_regs; */

static void pxa168_tmr_save(struct tmr_regs *context, unsigned int tmr_base)
{
	unsigned int i, j, temp;

	context->tmr_ccr = __raw_readl(tmr_base + TMR_CCR);
	for (i = 0 ;i < 3;i++) {
		/* double read to avoid metastability */
		do {
			temp = __raw_readl(tmr_base + TMR_CR(i));
			context->tmr_crn[i] = __raw_readl(tmr_base + TMR_CR(i));
		} while (context->tmr_crn[i] != temp);
	}
	/* save time difference instead of match counter register itself */
	for (i = 0; i < 3; i++)
		for (j = 0; j < 3; j++)
			context->tmr_tn_mm[i*3+j] = __raw_readl(tmr_base + TMR_TN_MM(i,j))
				- context->tmr_crn[i];
	for (i = 0 ;i < 3;i++)
		context->tmr_iern[i] = __raw_readl(tmr_base + TMR_IER(i));
	for (i = 0 ;i < 3;i++)
		context->tmr_plvrn[i] = __raw_readl(tmr_base + TMR_PLVR(i));
	for (i = 0 ;i < 3;i++)
		context->tmr_plcrn[i] = __raw_readl(tmr_base + TMR_PLCR(i));
	for (i = 0 ;i < 3;i++)
		context->tmr_ilrn[i] = __raw_readl(tmr_base + TMR_ILR(i));
	context->tmr_cmr = __raw_readl(tmr_base + TMR_CMR);
	context->tmr_cer = __raw_readl(tmr_base + TMR_CER);

	/* the following should be done in a watchdog driver... */
	context->tmr_wmer = __raw_readl(tmr_base + TMR_WMER);
	do {
		temp = __raw_readl(tmr_base + TMR_WVR);
		context->tmr_wvr = __raw_readl(tmr_base + TMR_WVR);
	} while (context->tmr_wvr != temp);
	context->tmr_wmr = __raw_readl(tmr_base + TMR_WMR) - context->tmr_wvr;
	context->tmr_wicr = __raw_readl(tmr_base + TMR_WICR);
	context->tmr_wcr = __raw_readl(tmr_base + TMR_WCR);
}

static void pxa168_tmr_restore(struct tmr_regs *context, unsigned int tmr_base)
{
	unsigned int i, j, temp;

	__raw_writel(context->tmr_ccr, tmr_base + TMR_CCR);
	for (i = 0 ;i < 3;i++) {
		/* double read to avoid metastability */
		do {
			temp = __raw_readl(tmr_base + TMR_CR(i));
			context->tmr_crn[i] = __raw_readl(tmr_base + TMR_CR(i));
		} while (context->tmr_crn[i] != temp);
	}
	/* restore match counter register based on current counter and
	 * the time difference we saved before */
	for (i = 0; i < 3; i++)
		for (j = 0; j < 3; j++) {
			context->tmr_tn_mm[3*i+j] += context->tmr_crn[i];
			__raw_writel(context->tmr_tn_mm[3*i+j], tmr_base + TMR_TN_MM(i,j));
		}
	for (i = 0 ;i < 3;i++)
		__raw_writel(context->tmr_iern[i], tmr_base + TMR_IER(i));
	for (i = 0 ;i < 3;i++)
		__raw_writel(0x7, tmr_base + TMR_ICR(i));
	for (i = 0 ;i < 3;i++)
		__raw_writel(context->tmr_plvrn[i], tmr_base + TMR_PLVR(i));
	for (i = 0 ;i < 3;i++)
		__raw_writel(context->tmr_plcrn[i], tmr_base + TMR_PLCR(i));
	for (i = 0 ;i < 3;i++)
		__raw_writel(context->tmr_ilrn[i], tmr_base + TMR_ILR(i));
	__raw_writel(context->tmr_cmr, tmr_base + TMR_CMR);
	__raw_writel(context->tmr_cer, tmr_base + TMR_CER);

	/* the following should be done in a watchdog driver... */
	__raw_writel(0xbaba, tmr_base + TMR_WFAR);
	__raw_writel(0xeb10, tmr_base + TMR_WSAR);
	__raw_writel(context->tmr_wmr, tmr_base + TMR_WMR);
	__raw_writel(context->tmr_wicr, tmr_base + TMR_WICR);
	__raw_writel(context->tmr_wcr, tmr_base + TMR_WCR);
	__raw_writel(context->tmr_wmer, tmr_base + TMR_WMER);
}

static void pxa_timer_suspend(void)
{
	pxa168_tmr_save(&tmr_saved_regs, TIMERS1_VIRT_BASE);
	/* pxa168_tmr_save(&tmr2_saved_regs, TIMERS2_VIRT_BASE); */
}

static void pxa_timer_resume(void)
{
	pxa168_tmr_restore(&tmr_saved_regs, TIMERS1_VIRT_BASE);
	/* pxa168_tmr_restore(&tmr2_saved_regs, TIMERS2_VIRT_BASE); */
}
#else
#define pxa_timer_suspend NULL
#define pxa_timer_resume NULL
#endif

struct sys_timer pxa168_timer = {
	.init		= pxa168_timer_init,
	.suspend	= pxa_timer_suspend,
	.resume		= pxa_timer_resume,
};

static void __init mmp2_timer_init(void)
{
	unsigned long clk_rst;

	__raw_writel(APBC_APBCLK | APBC_RST, APBC_MMP2_TIMERS);

	/*
	 * enable bus/functional clock, enable 6.5MHz (divider 4),
	 * release reset
	 */
	clk_rst = APBC_APBCLK | APBC_FNCLK | APBC_FNCLKSEL(1);
	__raw_writel(clk_rst, APBC_MMP2_TIMERS);

	timer_init(IRQ_MMP2_TIMER1);
}

struct sys_timer mmp2_timer = {
	.init	= mmp2_timer_init,
};

