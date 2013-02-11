/*
 *  linux/arch/arm/mach-mmp/pxa168.c
 *
 *  Code specific to PXA168
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

#include <mach/pxa168.h>
#include <mach/addr-map.h>
#include <mach/regs-apbc.h>
#include <mach/regs-apmu.h>
#include <mach/cputype.h>
#include <mach/irqs.h>
#include <mach/gpio.h>
#include <mach/dma.h>
#include <mach/mfp-pxa168.h>
#include <plat/i2c.h>

#include "common.h"
#include "clock.h"

#define MFPR_VIRT_BASE	(APB_VIRT_BASE + 0x1e000)

static struct mfp_addr_map pxa168_mfp_addr_map[] __initdata =
{
	MFP_ADDR_X(GPIO0,   GPIO36,  0x04c),
	MFP_ADDR_X(GPIO37,  GPIO55,  0x000),
	MFP_ADDR_X(GPIO56,  GPIO123, 0x0e0),
	MFP_ADDR_X(GPIO124, GPIO127, 0x0f4),

	MFP_ADDR_END,
};

#define APMASK(i)	(GPIO_REGS_VIRT + BANK_OFF(i) + 0x09c)

static void __init pxa168_init_gpio(void)
{
	int i;

	/* enable GPIO unit clock */
	__raw_writel(APBC_APBCLK | APBC_FNCLK, APBC_PXA168_GPIO);

	/* unmask GPIO edge detection for all 4 banks - APMASKx */
	for (i = 0; i < 4; i++)
		__raw_writel(0xffffffff, APMASK(i));

	pxa_init_gpio(IRQ_PXA168_GPIOX, 0, 127, NULL);
}

void __init pxa168_init_irq(void)
{
	icu_init_irq();
	pxa168_init_gpio();
}

struct gc_rate_table {
	unsigned long	rate;
	unsigned int	flag;
};

static struct gc_rate_table gc300_rates [] = {
	/* put highest rate at the top of the table */
	{
		.rate	=	624000000,
		.flag	=	APMU_GC_624M,
	},
	{
		.rate	=	312000000,
		.flag	=	APMU_GC_312M,
	},
	{
		.rate	=	156000000,
		.flag	=	APMU_GC_156M,
	},
};

static int gc_lookaround_rate(int target_rate, u32 *flag)
{
	int i;

	for (i=0; i<ARRAY_SIZE(gc300_rates); i++) {
		if (target_rate >= gc300_rates[i].rate)
			break;
	}
	if (i==ARRAY_SIZE(gc300_rates)) i--;
	*flag = gc300_rates[i].flag;
	return gc300_rates[i].rate;
}

static void gc300_clk_enable(struct clk *clk)
{
	u32 tmp = __raw_readl(clk->clk_rst), flag;

	/* reset gc clock */
	__raw_writel(tmp & ~0x07, clk->clk_rst);
	tmp = __raw_readl(clk->clk_rst);
	udelay(1);

	/* select GC clock source */
	gc_lookaround_rate(clk->rate, &flag);
	tmp &= ~0xc0;
	tmp |= flag;
	__raw_writel(tmp, clk->clk_rst);

	/* enable GC CLK EN */
	__raw_writel(tmp | 0x10, clk->clk_rst);
	tmp = __raw_readl(clk->clk_rst);

	/* enable GC HCLK EN */
	__raw_writel(tmp | 0x08, clk->clk_rst);
	tmp = __raw_readl(clk->clk_rst);

	/* enable GC ACLK EN */
	__raw_writel(tmp | 0x20, clk->clk_rst);
	tmp = __raw_readl(clk->clk_rst);

	/* reset GC */
	__raw_writel(tmp & ~0x07, clk->clk_rst);
	tmp = __raw_readl(clk->clk_rst);

	/* pull GC out of reset */
	__raw_writel(tmp | 0x2, clk->clk_rst);
	tmp = __raw_readl(clk->clk_rst);

	/* delay 48 cycles */
	udelay(1);

	/* pull GC AXI/AHB out of reset */
	__raw_writel(tmp | 0x5, clk->clk_rst);
	tmp = __raw_readl(clk->clk_rst);
}

static void gc300_clk_disable(struct clk *clk)
{
	__raw_writel(0, clk->clk_rst);
}

static int gc300_clk_setrate(struct clk *clk, unsigned long target_rate)
{
	u32 flag;

	clk->rate = gc_lookaround_rate(target_rate, &flag);

	return 0;
}

static unsigned long gc300_clk_getrate(struct clk *clk)
{
	return clk->rate;
}

struct clkops gc300_clk_ops = {
	.enable		= gc300_clk_enable,
	.disable	= gc300_clk_disable,
	.setrate	= gc300_clk_setrate,
	.getrate	= gc300_clk_getrate,
};
static APBC_UART_CLK(uart1, PXA168_UART0, 14745600);
static APBC_UART_CLK(uart2, PXA168_UART1, 14745600);
static APBC_UART_CLK(uart3, PXA168_UART2, 14745600);
static APBC_CLK(twsi0, PXA168_TWSI0, 1, 33000000);
static APBC_CLK(twsi1, PXA168_TWSI1, 1, 33000000);
static APBC_CLK(ssp0,  PXA168_SSP0,  4, 0);
static APBC_CLK(ssp1,  PXA168_SSP1,  0, 6500000);
static APBC_CLK(keypad, PXA168_KPC, 0, 32000);
static APBC_PWM_CLK(pwm0, PXA168_PWM0, 0, 13000000);
static APBC_PWM_CLK(pwm1, PXA168_PWM1, 0, 13000000);
static APBC_PWM_CLK(pwm2, PXA168_PWM2, 0, 13000000);
static APBC_PWM_CLK(pwm3, PXA168_PWM3, 0, 13000000);

static APMU_CLK(nand, NAND, 0x19B, 156000000);
static APMU_CLK(lcd, LCD, 0x007f, 312000000);	/* 312MHz, HCLK, CLK, AXICLK */
static APMU_CLK(mfu, MFU, 0x9, 0);	
static APMU_CLK_OPS(u2o, USB, 480000000, &u2o_clk_ops);	/* 480MHz, AXICLK */
static APMU_CLK_OPS(u2h, USB, 480000000, &u2h_clk_ops);	/* 480MHz, AXICLK */
static APMU_CLK_OPS(gc, GC, 0, &gc300_clk_ops);
static APMU_CLK_OPS(sdh0, SDH0, 48000000, &sdh_clk_ops);	/* 48MHz, CLK, AXICLK */
static APMU_CLK_OPS(sdh1, SDH1, 48000000, &sdh_clk_ops);	/* 48MHz, CLK, AXICLK */
static APMU_CLK_OPS(sdh2, SDH2, 48000000, &sdh_clk_ops);	/* 48MHz, CLK, AXICLK */
static APMU_CLK_OPS(sdh3, SDH3, 48000000, &sdh_clk_ops);	/* 48MHz, CLK, AXICLK */
static APMU_CLK(ccic_rst, CCIC_RST, 0x0, 312000000);
static APMU_CLK(ccic_gate, CCIC_GATE, 0xfff, 0);
static APMU_CLK_OPS(cf, CF, 78000000, &cf_clk_ops);
static APMU_CLK(icr, ICR, 0x3f, 0);
static APMU_CLK(smc, SMC, 0x5b, 0);

static struct clk_lookup pxa168_clkregs[] = {
	INIT_CLKREG(&clk_uart1, "pxa2xx-uart.0", NULL),
	INIT_CLKREG(&clk_uart2, "pxa2xx-uart.1", NULL),
	INIT_CLKREG(&clk_uart3, "pxa2xx-uart.2", NULL),
	INIT_CLKREG(&clk_uart3, NULL, "UART3CLK"),	/* clk for pxa168-ca */
	INIT_CLKREG(&clk_nand, "pxa3xx-nand", "NANDCLK"),
	INIT_CLKREG(&clk_twsi0, "pxa2xx-i2c.0", NULL),
	INIT_CLKREG(&clk_twsi1, "pxa2xx-i2c.1", NULL),
	INIT_CLKREG(&clk_ssp0,  "pxa168-ssp.0", NULL),
	INIT_CLKREG(&clk_ssp1,  "pxa168-ssp.1", NULL),
	INIT_CLKREG(&clk_keypad, "pxa27x-keypad", NULL),
	INIT_CLKREG(&clk_pwm0, "pxa168-pwm.0", "PWMCLK"),
	INIT_CLKREG(&clk_pwm1, "pxa168-pwm.1", "PWMCLK"),
	INIT_CLKREG(&clk_pwm2, "pxa168-pwm.2", "PWMCLK"),
	INIT_CLKREG(&clk_pwm3, "pxa168-pwm.3", "PWMCLK"),

	INIT_CLKREG(&clk_lcd, NULL, "LCDCLK"),
	INIT_CLKREG(&clk_u2o, NULL, "U2OCLK"),
	INIT_CLKREG(&clk_u2h, NULL, "U2HCLK"),
	INIT_CLKREG(&clk_gc, NULL, "GCCLK"),
	INIT_CLKREG(&clk_mfu, "pxa168-mfu", "MFUCLK"),
	INIT_CLKREG(&clk_sdh0, "pxa-sdh.0", "PXA-SDHCLK"),
	INIT_CLKREG(&clk_sdh1, "pxa-sdh.1", "PXA-SDHCLK"),
	INIT_CLKREG(&clk_sdh2, "pxa-sdh.2", "PXA-SDHCLK"),
	INIT_CLKREG(&clk_sdh3, "pxa-sdh.3", "PXA-SDHCLK"),
	INIT_CLKREG(&clk_ccic_rst, "pxa168-camera", "CCICRSTCLK"),
	INIT_CLKREG(&clk_ccic_gate, "pxa168-camera", "CCICGATECLK"),
	INIT_CLKREG(&clk_cf, "pxa168-cf", "CFCLK"),
	INIT_CLKREG(&clk_icr, NULL, "ICRCLK"),
	INIT_CLKREG(&clk_smc, "pxa168-ov529", "SMCCLK"),
};

void pxa168_mfp_set_fastio_drive(int type)
{
	switch (type) {
	/* pxa168 mfpr drive strength for fast IO pins[56:85]:
	 * ZPR[1] = MFPR58[11]
	 * ZPR[2] = MFPR58[10]
	 * ZNR[1] = MFPR56[11]
	 * ZNR[2] = MFPR56[10]
	 */
	case MFP_DS01X:
		/* ZPR[2:1] = ZNR[2:1] = 0b00 */
		mfp_clr(56, 3<<10);
		mfp_clr(58, 3<<10);
		break;
	case MFP_DS02X:
		/* ZPR[2:1] = ZNR[2:1] = 0b01 */
		mfp_clr(56, 1<<10);
		mfp_set(56, 1<<11);
		mfp_clr(58, 1<<10);
		mfp_set(58, 1<<11);
		break;
	case MFP_DS03X:
		/* ZPR[2:1] = ZNR[2:1] = 0b10 */
		mfp_set(56, 1<<10);
		mfp_clr(56, 1<<11);
		mfp_set(58, 1<<10);
		mfp_clr(58, 1<<11);
		break;
	case MFP_DS04X:
		/* ZPR[2:1] = ZNR[2:1] = 0b11 */
		mfp_set(56, 3<<10);
		mfp_set(58, 3<<10);
		break;
	default:
		pr_err("drv type %d not supported\n", type);
		break;
	}

	pr_info("%s config changed to %x\n", __func__, type);
	return;
}

#define MFP_VDD_IO_SET(type, pin, bit)			\
        do {                                            \
		if (type == VDD_IO_3P3V)	\
			mfp_set(pin, 1<<(bit));	\
		else				\
			mfp_clr(pin, 1<<(bit));	\
	} while (0);

void pxa168_set_vdd_iox(vdd_io_t vdd_io, int type)
{
	switch (vdd_io) {
	case VDD_IO0:
		MFP_VDD_IO_SET(type, 60, 10);
		break;
	case VDD_IO1:
		MFP_VDD_IO_SET(type, 60, 11);
		break;
	case VDD_IO2:
		MFP_VDD_IO_SET(type, 61, 10);
		break;
	case VDD_IO3:
		MFP_VDD_IO_SET(type, 61, 11);
		break;
	case VDD_IO4:
		MFP_VDD_IO_SET(type, 62, 11);
		break;
	default:
		pr_err("non-valid VDD_IO %d\n", vdd_io);
	}
}
#if defined(CONFIG_TIMER_SERVICES_MMP)
/****************************************************************/
/* Timer services */

static struct resource mmp_resource_timer_services_0[] = {
	/* regbase */
	[0] = {
		.start  = 0xD4014000,
		.end    = 0xD4014000,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_PXA168_TIMER2,
		.end	= IRQ_PXA168_TIMER2,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device mmp_device_timer_services_0 = {
	.name = "mmp-timer-services",
	.id   = 0,
	.num_resources = ARRAY_SIZE(mmp_resource_timer_services_0),
	.resource      = mmp_resource_timer_services_0,
};
#endif

static int __init pxa168_init(void)
{
	if (cpu_is_pxa168()) {
		mfp_init_base(MFPR_VIRT_BASE);
		mfp_init_addr(pxa168_mfp_addr_map);
		pxa_init_dma(IRQ_PXA168_DMA_INT0, 32);
		clks_register(ARRAY_AND_SIZE(pxa168_clkregs));
#if defined(CONFIG_TIMER_SERVICES_MMP)
	platform_device_register(&mmp_device_timer_services_0);
#endif
	}

	return 0;
}
postcore_initcall(pxa168_init);
