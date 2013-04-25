/*
 * linux/arch/arm/mach-mmp/include/mach/dma.h
 */

#ifndef __ASM_MACH_DMA_H
#define __ASM_MACH_DMA_H

#include <mach/addr-map.h>

#define DMAC_REGS_VIRT	(APB_VIRT_BASE + 0x00000)

extern int request_dma(unsigned int dmanr, const char *device_id);
extern void free_dma(unsigned int dmanr);

#include <plat/dma.h>
#endif /* __ASM_MACH_DMA_H */
