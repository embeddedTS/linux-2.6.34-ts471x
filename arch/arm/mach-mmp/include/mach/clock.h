/*
 *  linux/arch/arm/mach-mmp/clock.h
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#ifndef __MACH_CLOCK_H
#define __MACH_CLOCK_H

#include <asm/clkdev.h>

struct clkops {
	void			(*enable)(struct clk *);
	void			(*disable)(struct clk *);
	unsigned long		(*getrate)(struct clk *);
	int			(*setrate)(struct clk *, unsigned long);
};

struct clk {
	const struct clkops	*ops;
	void __iomem		*clk_rst; /* clock/reset register */
	int			fnclksel; /* functional clock select (APBC) */
	uint32_t		enable_val; /* register value to enable (APMU) */
	unsigned long		rate;
	int			enabled;
};

#endif /* __MACH_CLOCK_H */
