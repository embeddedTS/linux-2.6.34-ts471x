/*
 *  linux/arch/arm/mach-mmp/pxa910.c
 *
 *  Code specific to PXA910
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/io.h>
#include <linux/delay.h>

#include <mach/addr-map.h>
#include <mach/regs-apbc.h>
#include <mach/regs-apmu.h>
#include <mach/regs-mpmu.h>
#include <mach/cputype.h>
#include <mach/irqs.h>
#include <mach/gpio.h>
#include <mach/dma.h>
#include <mach/mfp.h>
#include <plat/i2c.h>
#include <mach/pxa910-squ.h>

#include "common.h"
#include "clock.h"

#define MFPR_VIRT_BASE	(APB_VIRT_BASE + 0x1e000)
#define FAB_CTRL	(AXI_VIRT_BASE + 0x260)

static struct mfp_addr_map pxa910_mfp_addr_map[] __initdata =
{
	MFP_ADDR_X(GPIO0, GPIO54, 0xdc),
	MFP_ADDR_X(GPIO67, GPIO98, 0x1b8),
	MFP_ADDR_X(GPIO100, GPIO109, 0x238),

	MFP_ADDR(GPIO123, 0xcc),
	MFP_ADDR(GPIO124, 0xd0),

	MFP_ADDR(DF_IO0, 0x40),
	MFP_ADDR(DF_IO1, 0x3c),
	MFP_ADDR(DF_IO2, 0x38),
	MFP_ADDR(DF_IO3, 0x34),
	MFP_ADDR(DF_IO4, 0x30),
	MFP_ADDR(DF_IO5, 0x2c),
	MFP_ADDR(DF_IO6, 0x28),
	MFP_ADDR(DF_IO7, 0x24),
	MFP_ADDR(DF_IO8, 0x20),
	MFP_ADDR(DF_IO9, 0x1c),
	MFP_ADDR(DF_IO10, 0x18),
	MFP_ADDR(DF_IO11, 0x14),
	MFP_ADDR(DF_IO12, 0x10),
	MFP_ADDR(DF_IO13, 0xc),
	MFP_ADDR(DF_IO14, 0x8),
	MFP_ADDR(DF_IO15, 0x4),

	MFP_ADDR(DF_nCS0_SM_nCS2, 0x44),
	MFP_ADDR(DF_nCS1_SM_nCS3, 0x48),
	MFP_ADDR(SM_nCS0, 0x4c),
	MFP_ADDR(SM_nCS1, 0x50),
	MFP_ADDR(DF_WEn, 0x54),
	MFP_ADDR(DF_REn, 0x58),
	MFP_ADDR(DF_CLE_SM_OEn, 0x5c),
	MFP_ADDR(DF_ALE_SM_WEn, 0x60),
	MFP_ADDR(SM_SCLK, 0x64),
	MFP_ADDR(DF_RDY0, 0x68),
	MFP_ADDR(SM_BE0, 0x6c),
	MFP_ADDR(SM_BE1, 0x70),
	MFP_ADDR(SM_ADV, 0x74),
	MFP_ADDR(DF_RDY1, 0x78),
	MFP_ADDR(SM_ADVMUX, 0x7c),
	MFP_ADDR(SM_RDY, 0x80),

	MFP_ADDR_X(MMC1_DAT7, MMC1_WP, 0x84),

	MFP_ADDR_END,
};

struct gc_rate_table {
	unsigned long	rate;
	unsigned int	flag;
};

static struct gc_rate_table gc500_rates [] = {
	/* put highest rate at the top of the table */
	{
		.rate	=	400000000,
		.flag	=	APMU_GC_PLL2,
	},
	{
		.rate	=	312000000,
		.flag	=	APMU_GC_312M,
	},
	{
		.rate	=	200000000,
		.flag	=	APMU_GC_PLL2_DIV2,
	},
	{
		.rate	=	156000000,
		.flag	=	APMU_GC_156M,
	},
};

static void gc500_clk_enable(struct clk *clk)
{
	u32 tmp = __raw_readl(clk->clk_rst);

	//__raw_writel(0x1f0f0f, APMU_XD_CLK_RES_CTRL); /*it will hang on old stepping*/
	__raw_writel(tmp | 0x38, clk->clk_rst);
	__raw_writel(tmp | 0x238, clk->clk_rst);
	udelay(200); /* at least 200 us*/
	__raw_writel(tmp | 0x638, clk->clk_rst);
	__raw_writel(tmp | 0x63a, clk->clk_rst);
	udelay(100); /* at least 48 cycles)*/
	__raw_writel(tmp | 0x73f, clk->clk_rst);
	__raw_writel(0x1f, APMU_GC_PD);
}

static void gc500_clk_disable(struct clk *clk)
{
	u32 tmp = __raw_readl(clk->clk_rst);

	__raw_writel(tmp | 0x63f, clk->clk_rst);
	__raw_writel(tmp | 0x23f, clk->clk_rst);
	__raw_writel(tmp | 0x03f, clk->clk_rst);
}

static int gc_lookaround_rate(unsigned long target_rate, u32 *flag)
{
	int i;

	for (i=0; i<ARRAY_SIZE(gc500_rates); i++) {
		if (target_rate >= gc500_rates[i].rate)
			break;
	}
	if (i==ARRAY_SIZE(gc500_rates)) i--;
	*flag = gc500_rates[i].flag;
	return gc500_rates[i].rate;
}

static int gc500_clk_setrate(struct clk *clk, unsigned long target_rate)
{
	u32 tmp, flag;
	int rate = gc_lookaround_rate(target_rate, &flag);

	clk->rate = rate;
	__raw_writel(0xf, APMU_GC_PD);
	tmp = __raw_readl(clk->clk_rst);
	tmp &= ~0xc0;
	tmp |= flag;
	__raw_writel(tmp, clk->clk_rst);
	return 0;
}

static unsigned long gc500_clk_getrate(struct clk *clk)
{
	return clk->rate;
}

struct clkops gc500_clk_ops = {
	.enable		= gc500_clk_enable,
	.disable	= gc500_clk_disable,
	.setrate	= gc500_clk_setrate,
	.getrate	= gc500_clk_getrate,
};

static APBC_UART_CLK(uart1, PXA168_UART0, 14745600);
static APBC_UART_CLK(uart2, PXA168_UART1, 14745600);
static APBC_UART_CLK(pxa910_uart3, PXA910_UART2, 14745600);
static APBC_CLK(twsi0, PXA910_TWSI0, 1, 33000000);
static APBC_CLK(twsi1, PXA910_TWSI1, 1, 33000000);
static APBC_CLK(keypad, PXA168_KPC, 0, 32000);
static APBC_CLK(ssp1,  PXA910_SSP1,  0, 0);

static PSEUDO_CLK(iscclk, 0, 0, 0);	/* pseudo clock for imm */

static APMU_CLK(lcd, LCD, 0x003f, 312000000);	/* 312MHz, HCLK, CLK, AXICLK */
static APMU_CLK(nand, NAND, 0x1DB, 208000000);
static APMU_CLK_OPS(u2o, USB, 480000000, &u2o_clk_ops);	/* 480MHz, AXICLK */
static APMU_CLK(ire, IRE, 0x8, 0);
static APMU_CLK_OPS(gc, GC, 0, &gc500_clk_ops);
static APMU_CLK(sdh0, SDH0, 0x001b, 48000000);	/* 48MHz, CLK, AXICLK */
static APMU_CLK(sdh1, SDH1, 0x001b, 48000000);	/* 48MHz, CLK, AXICLK */
static APMU_CLK(sdh2, SDH2, 0x001b, 48000000);	/* 48MHz, CLK, AXICLK */
static APMU_CLK(sdh3, SDH3, 0x001b, 48000000);	/* 48MHz, CLK, AXICLK */
static APMU_CLK(ccic_rst, CCIC_RST, 0x0, 312000000);
static APMU_CLK(ccic_gate, CCIC_GATE, 0xfff, 0);

static struct clk_lookup pxa910_clkregs[] = {
	INIT_CLKREG(&clk_uart1, "pxa2xx-uart.0", NULL),
	INIT_CLKREG(&clk_uart2, "pxa2xx-uart.1", NULL),
	INIT_CLKREG(&clk_pxa910_uart3, "pxa2xx-uart.2", NULL),
	INIT_CLKREG(&clk_twsi0, "pxa2xx-i2c.0", NULL),
	INIT_CLKREG(&clk_twsi1, "pxa2xx-i2c.1", NULL),
	INIT_CLKREG(&clk_lcd, "pxa168-fb", "LCDCLK"),
	INIT_CLKREG(&clk_lcd, "pxa910-fb", NULL),
	INIT_CLKREG(&clk_nand, "pxa3xx-nand", "NANDCLK"),
	INIT_CLKREG(&clk_u2o, NULL, "U2OCLK"),
	INIT_CLKREG(&clk_keypad, "pxa27x-keypad", NULL),
	INIT_CLKREG(&clk_ire, "pxa910-ire", NULL),
	INIT_CLKREG(&clk_ssp1, "pxa168-ssp.1", NULL),
	INIT_CLKREG(&clk_gc, NULL, "GCCLK"),
	INIT_CLKREG(&clk_ssp1,  "pxa910-ssp.1", NULL),
	INIT_CLKREG(&clk_iscclk,  NULL, "ISCCLK"),
	INIT_CLKREG(&clk_sdh0, "pxa-sdh.0", "PXA-SDHCLK"),
	INIT_CLKREG(&clk_sdh1, "pxa-sdh.1", "PXA-SDHCLK"),
	INIT_CLKREG(&clk_sdh2, "pxa-sdh.2", "PXA-SDHCLK"),
	INIT_CLKREG(&clk_sdh3, "pxa-sdh.3", "PXA-SDHCLK"),
        INIT_CLKREG(&clk_ccic_rst, "pxa168-camera", "CCICRSTCLK"),
        INIT_CLKREG(&clk_ccic_gate, "pxa168-camera", "CCICGATECLK"),
};

void pxa910_set_pll2(void)
{
	u32 tmp = __raw_readl(MPMU_PLL2CR);

	/* FIXME default set to 400MHz */
	tmp &= ~((0x1f<<19) | (0x1ff<<10));
	tmp |= ((4<<19) | (61<<10));
	tmp |= (1<<9);

	__raw_writel(tmp, MPMU_PLL2CR);
}

/* ACIPC clock is initialized by CP, enable the clock by default
 * and this clock is always enabled.
 */
static void pxa910_init_acipc_clock(void)
{
	__raw_writel(0x3, APBC_PXA910_ACIPC);
}
static void pxa910_init_ripc_clock(void)
{
        __raw_writel(0x2, APBC_PXA910_RIPC);
}

static int __init pxa910_init(void)
{
	u32 tmp;

	if (cpu_is_pxa910()) {
		mfp_init_base(MFPR_VIRT_BASE);
		mfp_init_addr(pxa910_mfp_addr_map);
		pxa_init_dma(IRQ_PXA168_DMA_INT0, 32);
		pxa910_init_squ(2);

		/* FIXME set PLL2 here temporarily */
		pxa910_set_pll2();
		pxa910_init_acipc_clock();
		pxa910_init_ripc_clock();
		/* Enable AXI write request for gc500 */
		tmp = __raw_readl(FAB_CTRL);
        	__raw_writel(tmp | 0x8, FAB_CTRL);

		clks_register(ARRAY_AND_SIZE(pxa910_clkregs));
	}

	return 0;
}
postcore_initcall(pxa910_init);
