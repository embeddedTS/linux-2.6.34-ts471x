/*
 * linux/arch/arm/mach-mmp/include/mach/regs-apmu.h
 *
 *   Application Subsystem Power Management Unit
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_MACH_REGS_APMU_H
#define __ASM_MACH_REGS_APMU_H

#include <mach/addr-map.h>

#define APMU_VIRT_BASE	(AXI_VIRT_BASE + 0x82800)
#define APMU_REG(x)	(APMU_VIRT_BASE + (x))

/* Clock Reset Control */
#define APMU_IRE	APMU_REG(0x048)
#define APMU_LCD	APMU_REG(0x04c)
#define APMU_CCIC_RST	APMU_REG(0x050)
#define APMU_SDH0	APMU_REG(0x054)
#define APMU_SDH1	APMU_REG(0x058)
#define APMU_SDH2	APMU_REG(0x0e0)
#define APMU_SDH3	APMU_REG(0x0e4)
#define APMU_USB	APMU_REG(0x05c)
#define APMU_NAND	APMU_REG(0x060)
#define APMU_DMA	APMU_REG(0x064)
#define APMU_GEU	APMU_REG(0x068)
#define APMU_BUS	APMU_REG(0x06c)
#define APMU_GC		APMU_REG(0x0cc)
#define APMU_GC_PD	APMU_REG(0x0d0)
#define APMU_CF         APMU_REG(0x0f0)
#define APMU_WAKE_CLR	APMU_REG(0x07c)
#define APMU_ICR	APMU_REG(0x0f8)
#define APMU_MFU	APMU_REG(0x0fc)
#define APMU_CCIC_DBG	APMU_REG(0x088)
#define APMU_CCIC_GATE	APMU_REG(0x028)
#define APMU_SMC	APMU_REG(0x0d4)

#define APMU_PCR		APMU_REG(0x0000)
#define APMU_CCR		APMU_REG(0x0004)
#define APMU_CCSR		APMU_REG(0x000c)
#define APMU_FC_TIMER		APMU_REG(0x0010)
#define APMU_CP_IDLE_CFG	APMU_REG(0x0014)
#define APMU_IDLE_CFG		APMU_REG(0x0018)
#define APMU_LCD_CLK_RES_CTRL	APMU_REG(0x004c)
#define APMU_CCIC_CLK_RES_CTRL	APMU_REG(0x0050)
#define APMU_SDH0_CLK_RES_CTRL	APMU_REG(0x0054)
#define APMU_SDH1_CLK_RES_CTRL	APMU_REG(0x0058)
#define APMU_SDH2_CLK_RES_CTRL	APMU_REG(0x00e0)
#define APMU_SDH3_CLK_RES_CTRL	APMU_REG(0x00e4)
#define APMU_USB_CLK_RES_CTRL	APMU_REG(0x005c)
#define APMU_NFC_CLK_RES_CTRL	APMU_REG(0x0060)
#define APMU_DMA_CLK_RES_CTRL	APMU_REG(0x0064)
#define APMU_BUS_CLK_RES_CTRL	APMU_REG(0x006c)
#define APMU_WAKE_CLK		APMU_REG(0x007c)
#define APMU_PWR_STBL_TIMER	APMU_REG(0x0084)
#define APMU_SRAM_PWR_DWN	APMU_REG(0x008c)
#define APMU_CORE_STATUS	APMU_REG(0x0090)
#define APMU_RES_FRM_SLP_CLR	APMU_REG(0x0094)
#define APMU_IMR		APMU_REG(0x0098)
#define APMU_IRWC		APMU_REG(0x009c)
#define APMU_ISR		APMU_REG(0x00a0)
#define APMU_DX8_CLK_RES_CTRL	APMU_REG(0x00a4)
#define APMU_DTC_CLK_RES_CTRL	APMU_REG(0x00ac)
#define APMU_MC_HW_SLP_TYPE	APMU_REG(0x00b0)
#define APMU_MC_SLP_REQ		APMU_REG(0x00b4)
#define APMU_MC_SW_SLP_TYPE	APMU_REG(0x00c0)
#define APMU_PLL_SEL_STATUS	APMU_REG(0x00c4)
#define APMU_GC_CLK_RES_CTRL	APMU_REG(0x00cc)
#define APMU_SMC_CLK_RES_CTRL	APMU_REG(0x00d4)
#define APMU_XD_CLK_RES_CTRL	APMU_REG(0x00dc)
#define APMU_CF_CLK_RES_CTRL	APMU_REG(0x00f0)
#define APMU_MSP_CLK_RES_CTRL	APMU_REG(0x00f4)
#define APMU_CMU_CLK_RES_CTRL	APMU_REG(0x00f8)
#define APMU_MFU_CLK_RES_CTRL	APMU_REG(0x00fc)

#define APMU_FNCLK_EN	(1 << 4)
#define APMU_AXICLK_EN	(1 << 3)
#define APMU_FNRST_DIS	(1 << 1)
#define APMU_AXIRST_DIS	(1 << 0)

#define APMU_GC_156M		0x0
#define APMU_GC_312M		0x40
#define APMU_GC_PLL2		0x80
#define APMU_GC_PLL2_DIV2	0xc0
#define APMU_GC_624M		0xc0 /* added according to Aspen SW spec v2.8*/

#endif /* __ASM_MACH_REGS_APMU_H */
