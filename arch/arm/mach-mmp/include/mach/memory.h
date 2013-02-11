/*
 * linux/arch/arm/mach-mmp/include/mach/memory.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_MACH_MEMORY_H
#define __ASM_MACH_MEMORY_H

#define PHYS_OFFSET	UL(0x00000000)

#if defined(CONFIG_CPU_L2_CACHE)
#define NET_IP_ALIGN 64
#define ARCH_DMA_CACHE_ALIGNMENT 64
#endif

#endif /* __ASM_MACH_MEMORY_H */
