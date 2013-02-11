/*
 * arch/arm/mach-mmp/include/mach/uncompress.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/serial_reg.h>
#include <mach/addr-map.h>
#include <asm/mach-types.h>

#define UART1_BASE	(APB_PHYS_BASE + 0x36000)
#define UART2_BASE	(APB_PHYS_BASE + 0x17000)
#define UART3_BASE	(APB_PHYS_BASE + 0x18000)
#define UART4_BASE	(APB_PHYS_BASE + 0x26000)

static inline void putc(char c)
{
	volatile unsigned long *UART;

	if (machine_is_ipcam())
		UART = (unsigned long *)UART4_BASE;
	else if (machine_is_avengers_lite() || machine_is_edge())
		UART = (unsigned long *)UART3_BASE;
	else
		UART = (unsigned long *)UART2_BASE;

	/* UART enabled? */
	if (!(UART[UART_IER] & UART_IER_UUE))
		return;

	while (!(UART[UART_LSR] & UART_LSR_THRE))
		barrier();

	UART[UART_TX] = c;
}

/*
 * This does not append a newline
 */
static inline void flush(void)
{
}

/*
 * nothing to do
 */
#define arch_decomp_setup()
#define arch_decomp_wdog()
