/*
 * linux/arch/arm/mach-mmp/include/mach/system.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_MACH_SYSTEM_H
#define __ASM_MACH_SYSTEM_H

#include <mach/regs-mpmu.h>
#include <mach/regs-timers.h>
#include <mach/cputype.h>

#define MPMU_APRR_WDTR	(1<<4)

static inline void arch_idle(void)
{
	cpu_do_idle();
}

static inline void arch_reset(char mode, const char *cmd)
{
	u32 reg;
	u32 watchdog_virt_base;

	if (cpu_is_pxa168())
		watchdog_virt_base = TIMERS1_VIRT_BASE;
	else if (cpu_is_pxa910())
		watchdog_virt_base = CP_TIMERS2_VIRT_BASE;
	else
		return ;

	/* negate hardware reset to the WDT after system reset */
	reg = readl(MPMU_APRR) | MPMU_APRR_WDTR;
	writel(reg, MPMU_APRR);

	/*disable functional WDT clock */
	writel(0x1, MPMU_WDTPCR);

	/* clear previous WDT status */
	writel(0xbaba, watchdog_virt_base + TMR_WFAR);
	writel(0xeb10, watchdog_virt_base + TMR_WSAR);
	writel(0, watchdog_virt_base + TMR_WSR);

	/* set match counter */
	writel(0xbaba, watchdog_virt_base + TMR_WFAR);
	writel(0xeb10, watchdog_virt_base + TMR_WSAR);
	writel(0xf, watchdog_virt_base + TMR_WMR);

	/* enable WDT reset */
	writel(0xbaba, watchdog_virt_base + TMR_WFAR);
	writel(0xeb10, watchdog_virt_base + TMR_WSAR);
	writel(0x3, watchdog_virt_base + TMR_WMER);

	/*enable functional WDT clock */
	writel(0x3, MPMU_WDTPCR);
}

#endif /* __ASM_MACH_SYSTEM_H */
