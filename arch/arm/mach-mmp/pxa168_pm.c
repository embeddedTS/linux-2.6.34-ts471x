/*
 * PXA168 Power Management Routines
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 *
 * (C) Copyright 2009 Marvell International Ltd.
 * All Rights Reserved
 */

#undef DEBUG
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/vmalloc.h>
#include <linux/kobject.h>
#include <linux/suspend.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <asm/cacheflush.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/cputype.h>
#include <mach/mfp.h>
#include <mach/gpio.h>
#include <mach/pxa168.h>
#include <mach/pxa168_pm.h>
#include <mach/addr-map.h>
#include <mach/dma.h>
#include <mach/gpio.h>
#include <mach/pxa910-squ.h>
#include <mach/regs-apbc.h>
#include <mach/regs-apmu.h>
#include <mach/regs-icu.h>
#include <mach/regs-mpmu.h>
#include <mach/regs-ciu.h>
#include <mach/regs-smc.h>
#include <linux/wakelock.h>
#include <mach/irqs.h>
#include <mach/dvfm.h>
#include <mach/mspm_prof.h>
#include <mach/pxa168_dvfm.h>
#include <linux/slab.h>

/*
#define APB_PHYS_BASE	   0xd4000000
#define APB_VIRT_BASE	   0xfe000000
#define APB_PHYS_SIZE	   0x00200000

#define AXI_PHYS_BASE	   0xd4200000
#define AXI_VIRT_BASE	   0xfe200000
#define AXI_PHYS_SIZE	   0x00200000

#define DMAC_REGS_VIRT		(APB_VIRT_BASE + 0x00000)

#define TIMERS1_VIRT_BASE	(APB_VIRT_BASE + 0x14000)

#define APBC_VIRT_BASE		(APB_VIRT_BASE + 0x15000)

#define GPIO_REGS_VIRT		(APB_VIRT_BASE + 0x19000)

#define MPMU_VIRT_BASE		(APB_VIRT_BASE + 0x50000)

#define ICU_VIRT_BASE		(AXI_VIRT_BASE + 0x82000)

#define APMU_VIRT_BASE		(AXI_VIRT_BASE + 0x82800)

#define PXA910_SQU_REGS_VIRT	(AXI_VIRT_BASE + 0xA0000)

*/

#define APMASK(i)	(GPIO_REGS_VIRT + BANK_OFF(i) + 0x09c)

/*used by expander max7312, 16 pins gpio expander */
#define GPIO_EXT0(x)		(NR_BUILTIN_GPIO + (x))
#define GPIO_EXT1(x)		(NR_BUILTIN_GPIO + 16 + (x))


/* How long we will in sleep mode if duty cycle. */
unsigned int pm_sleeptime = 0;	/* In seconds. */
EXPORT_SYMBOL(pm_sleeptime);

int enable_deepidle = 0x2;		/* IDLE_D0 -- 0 */

static struct pxa168_pm_regs pxa168_pm_regs;

static unsigned long pm_state;


unsigned char __iomem *dmc_membase;
EXPORT_SYMBOL(dmc_membase);
unsigned char __iomem *sram_membase;
EXPORT_SYMBOL(sram_membase);

extern struct wake_lock main_wake_lock;
/*************************************************************************/
static void flush_cpu_cache(void)
{
	__cpuc_flush_kern_all();
	//__cpuc_flush_l2cache_all();
}

static void pxa168_intc_save(struct intc_regs *context)
{
	unsigned int i;

	for (i = 0 ;i < 64;i++)
		context->icu_int_conf[i] = __raw_readl(ICU_INT_CONF(i));
	context->icu_fiq_sel_int_num = __raw_readl(ICU_AP_FIQ_SEL_INT_NUM);
	context->icu_irq_sel_int_num = __raw_readl(ICU_AP_IRQ_SEL_INT_NUM);
	context->icu_gbl_irq_msk = __raw_readl(ICU_AP_GBL_IRQ_MSK);
	context->icu_dma_int_msk = __raw_readl(ICU_DMA_INT_MSK);
	context->icu_dma_int_status = __raw_readl(ICU_DMA_INT_STATUS);
	context->icu_int_status_0 = __raw_readl(ICU_INT_STATUS_0);
	context->icu_int_status_1 = __raw_readl(ICU_INT_STATUS_1);
	context->icu_ddr_arm_l2_int_msk = __raw_readl(ICU_DDR_ARM_L2_INT_MSK);
	context->icu_ddr_arm_l2_int_status = __raw_readl(ICU_DDR_ARM_L2_INT_STATUS);
}

static void pxa168_intc_restore(struct intc_regs *context)
{
	unsigned int i;
	for (i = 0 ;i < 64;i++)
		__raw_writel(context->icu_int_conf[i], ICU_INT_CONF(i));
	__raw_writel(context->icu_fiq_sel_int_num, ICU_AP_FIQ_SEL_INT_NUM);
	__raw_writel(context->icu_irq_sel_int_num, ICU_AP_IRQ_SEL_INT_NUM);
	__raw_writel(context->icu_gbl_irq_msk, ICU_AP_GBL_IRQ_MSK);
	__raw_writel(context->icu_dma_int_msk, ICU_DMA_INT_MSK);
	__raw_writel(context->icu_dma_int_status, ICU_DMA_INT_STATUS);
	__raw_writel(context->icu_int_status_0, ICU_INT_STATUS_0);
	__raw_writel(context->icu_int_status_1, ICU_INT_STATUS_1);
	__raw_writel(context->icu_ddr_arm_l2_int_msk, ICU_DDR_ARM_L2_INT_MSK);
	__raw_writel(context->icu_ddr_arm_l2_int_status, ICU_DDR_ARM_L2_INT_STATUS);
}

#define MFPR_VIRT_BASE  (APB_VIRT_BASE + 0x1e000)

static void pxa168_mfp_save(struct mfp_regs *context)
{
	unsigned int i;

	for (i = 0; i < MAX_MFP_PINS; i++)
		context->mfp[i] = __raw_readl(MFPR_VIRT_BASE + (i << 2));
}

static void pxa168_mfp_restore(struct mfp_regs *context)
{
	unsigned int i;

	for (i = 0; i < MAX_MFP_PINS; i++)
		__raw_writel(context->mfp[i], MFPR_VIRT_BASE + (i << 2));
}

#define GAPMASK_OFFSET     0x9C

static void pxa168_gpio_save(struct gpio_regs *context)
{
	unsigned int i;

	for (i = 0; i < 4; i++) {
		context->gpdr[i] = __raw_readl(GPIO_BANK(i) + GPDR_OFFSET);
		context->grer[i] = __raw_readl(GPIO_BANK(i) + GRER_OFFSET);
		context->gfer[i] = __raw_readl(GPIO_BANK(i) + GFER_OFFSET);
		context->gedr[i] = __raw_readl(GPIO_BANK(i) + GEDR_OFFSET);
		context->gapmask[i] = __raw_readl(GPIO_BANK(i) + GAPMASK_OFFSET);
	}
}

static void pxa168_gpio_restore(struct gpio_regs *context)
{
	unsigned int i;

	for (i = 0; i < 4; i++) {
		__raw_writel(context->gpdr[i], GPIO_BANK(i) + GPDR_OFFSET);
		__raw_writel(context->grer[i], GPIO_BANK(i) + GRER_OFFSET);
		__raw_writel(context->gfer[i], GPIO_BANK(i) + GFER_OFFSET);
		__raw_writel(context->gedr[i], GPIO_BANK(i) + GEDR_OFFSET);
		__raw_writel(context->gapmask[i], GPIO_BANK(i) + GAPMASK_OFFSET);
	}
}

static void pxa168_mpmu_save(struct mpmu_regs *context)
{
	context->fccr = __raw_readl(MPMU_FCCR);
	context->pocr = __raw_readl(MPMU_POCR);
	context->posr = __raw_readl(MPMU_POSR);
	context->succr = __raw_readl(MPMU_SUCCR);
	context->ohcr = __raw_readl(MPMU_OHCR);
	context->gpcr = __raw_readl(MPMU_GPCR);
	context->pll2cr = __raw_readl(MPMU_PLL2CR);
	context->sccr = __raw_readl(MPMU_SCCR);
	context->pll1_reg1 = __raw_readl(MPMU_PLL1_REG1);
	context->pll1_reg2 = __raw_readl(MPMU_PLL1_REG2);
	context->pll1_ssc = __raw_readl(MPMU_PLL1_SSC);
	context->pll2_reg1 = __raw_readl(MPMU_PLL2_REG1);
	context->pll2_reg2 = __raw_readl(MPMU_PLL2_REG2);
	context->pll2_ssc = __raw_readl(MPMU_PLL2_SSC);
	context->ts = __raw_readl(MPMU_TS);
	context->wdtpcr = __raw_readl(MPMU_WDTPCR);
	context->apcr = __raw_readl(MPMU_APCR);
	context->apsr = __raw_readl(MPMU_APSR);
	context->aprr = __raw_readl(MPMU_APRR);
	context->acgr = __raw_readl(MPMU_ACGR);
	context->arsr = __raw_readl(MPMU_ARSR);
	context->awucrs = __raw_readl(MPMU_AWUCRS);
	context->awucrm = __raw_readl(MPMU_AWUCRM);
}

static void pxa168_mpmu_restore(struct mpmu_regs *context)
{
	__raw_writel(context->fccr, MPMU_FCCR);
	__raw_writel(context->pocr, MPMU_POCR);
	__raw_writel(context->posr, MPMU_POSR);
	__raw_writel(context->succr, MPMU_SUCCR);
	__raw_writel(context->ohcr, MPMU_OHCR);
	__raw_writel(context->gpcr, MPMU_GPCR);
	__raw_writel(context->pll2cr, MPMU_PLL2CR);
	__raw_writel(context->sccr, MPMU_SCCR);
	__raw_writel(context->pll1_reg1, MPMU_PLL1_REG1);
	__raw_writel(context->pll1_reg2, MPMU_PLL1_REG2);
	__raw_writel(context->pll1_ssc, MPMU_PLL1_SSC);
	__raw_writel(context->pll2_reg1, MPMU_PLL2_REG1);
	__raw_writel(context->pll2_reg2, MPMU_PLL2_REG2);
	__raw_writel(context->pll2_ssc, MPMU_PLL2_SSC);
	__raw_writel(context->ts, MPMU_TS);
	__raw_writel(context->wdtpcr, MPMU_WDTPCR);
	__raw_writel(context->apcr, MPMU_APCR);
	__raw_writel(context->apsr, MPMU_APSR);
	__raw_writel(context->aprr, MPMU_APRR);
	__raw_writel(context->acgr, MPMU_ACGR);
	__raw_writel(context->arsr, MPMU_ARSR);
	__raw_writel(context->awucrs, MPMU_AWUCRS);
	__raw_writel(context->awucrm, MPMU_AWUCRM);
}

static void pxa168_apmu_save(struct apmu_regs *context)
{
	context->ccr = __raw_readl(APMU_CCR);
	context->ccsr = __raw_readl(APMU_CCSR);
	context->fc_timer = __raw_readl(APMU_FC_TIMER);
	context->idle_cfg = __raw_readl(APMU_IDLE_CFG);
	context->lcd_clk_res_ctrl = __raw_readl(APMU_LCD_CLK_RES_CTRL);
	context->ccic_clk_res_ctrl = __raw_readl(APMU_CCIC_CLK_RES_CTRL);
	context->sdh0_clk_res_ctrl = __raw_readl(APMU_SDH0_CLK_RES_CTRL);
	context->sdh1_clk_res_ctrl = __raw_readl(APMU_SDH1_CLK_RES_CTRL);
	context->sdh2_clk_res_ctrl = __raw_readl(APMU_SDH2_CLK_RES_CTRL);
	context->sdh3_clk_res_ctrl = __raw_readl(APMU_SDH3_CLK_RES_CTRL);
	context->usb_clk_res_ctrl = __raw_readl(APMU_USB_CLK_RES_CTRL);
	context->nfc_clk_res_ctrl = __raw_readl(APMU_NFC_CLK_RES_CTRL);
	context->dma_clk_res_ctrl = __raw_readl(APMU_DMA_CLK_RES_CTRL);
	context->bus_clk_res_ctrl = __raw_readl(APMU_BUS_CLK_RES_CTRL);
	context->wake_clk = __raw_readl(APMU_WAKE_CLK);
	context->core_status = __raw_readl(APMU_CORE_STATUS);
	context->res_frm_slp_clr = __raw_readl(APMU_RES_FRM_SLP_CLR);
	context->imr = __raw_readl(APMU_IMR);
	context->irwc = __raw_readl(APMU_IRWC);
	context->isr = __raw_readl(APMU_ISR);
	context->dtc_clk_res_ctrl = __raw_readl(APMU_DTC_CLK_RES_CTRL);
	context->mc_hw_slp_type = __raw_readl(APMU_MC_HW_SLP_TYPE);
	context->mc_slp_req = __raw_readl(APMU_MC_SLP_REQ);
	context->mc_sw_slp_type = __raw_readl(APMU_MC_SW_SLP_TYPE);
	context->pll_sel_status = __raw_readl(APMU_PLL_SEL_STATUS);
	context->gc_clk_res_ctrl = __raw_readl(APMU_GC_CLK_RES_CTRL);
	context->smc_clk_res_ctrl = __raw_readl(APMU_SMC_CLK_RES_CTRL);
	context->xd_clk_res_ctrl = __raw_readl(APMU_XD_CLK_RES_CTRL);
	context->cf_clk_res_ctrl = __raw_readl(APMU_CF_CLK_RES_CTRL);
	context->msp_clk_res_ctrl = __raw_readl(APMU_MSP_CLK_RES_CTRL);
	context->cmu_clk_res_ctrl = __raw_readl(APMU_CMU_CLK_RES_CTRL);
	context->mfu_clk_res_ctrl = __raw_readl(APMU_MFU_CLK_RES_CTRL);
}

static void pxa168_apmu_restore(struct apmu_regs *context)
{
	__raw_writel(context->ccr, APMU_CCR);
	__raw_writel(context->ccsr, APMU_CCSR);
	__raw_writel(context->fc_timer, APMU_FC_TIMER);
	__raw_writel(context->idle_cfg, APMU_IDLE_CFG);
	__raw_writel(context->lcd_clk_res_ctrl, APMU_LCD_CLK_RES_CTRL);
	__raw_writel(context->ccic_clk_res_ctrl, APMU_CCIC_CLK_RES_CTRL);
	__raw_writel(context->sdh0_clk_res_ctrl, APMU_SDH0_CLK_RES_CTRL);
	__raw_writel(context->sdh1_clk_res_ctrl, APMU_SDH1_CLK_RES_CTRL);
	__raw_writel(context->sdh2_clk_res_ctrl, APMU_SDH2_CLK_RES_CTRL);
	__raw_writel(context->sdh3_clk_res_ctrl, APMU_SDH3_CLK_RES_CTRL);
	__raw_writel(context->usb_clk_res_ctrl, APMU_USB_CLK_RES_CTRL);
	__raw_writel(context->nfc_clk_res_ctrl, APMU_NFC_CLK_RES_CTRL);
	__raw_writel(context->dma_clk_res_ctrl, APMU_DMA_CLK_RES_CTRL);
	__raw_writel(context->bus_clk_res_ctrl, APMU_BUS_CLK_RES_CTRL);
	__raw_writel(context->wake_clk, APMU_WAKE_CLK);
	__raw_writel(context->core_status, APMU_CORE_STATUS);
	__raw_writel(context->res_frm_slp_clr, APMU_RES_FRM_SLP_CLR);
	__raw_writel(context->imr, APMU_IMR);
	__raw_writel(context->irwc, APMU_IRWC);
	__raw_writel(context->isr, APMU_ISR);
	__raw_writel(context->dtc_clk_res_ctrl, APMU_DTC_CLK_RES_CTRL);
	__raw_writel(context->mc_hw_slp_type, APMU_MC_HW_SLP_TYPE);
	__raw_writel(context->mc_slp_req, APMU_MC_SLP_REQ);
	__raw_writel(context->mc_sw_slp_type, APMU_MC_SW_SLP_TYPE);
	__raw_writel(context->pll_sel_status, APMU_PLL_SEL_STATUS);
	__raw_writel(context->gc_clk_res_ctrl, APMU_GC_CLK_RES_CTRL);
	__raw_writel(context->smc_clk_res_ctrl, APMU_SMC_CLK_RES_CTRL);
	__raw_writel(context->xd_clk_res_ctrl, APMU_XD_CLK_RES_CTRL);
	__raw_writel(context->cf_clk_res_ctrl, APMU_CF_CLK_RES_CTRL);
	__raw_writel(context->msp_clk_res_ctrl, APMU_MSP_CLK_RES_CTRL);
	__raw_writel(context->cmu_clk_res_ctrl, APMU_CMU_CLK_RES_CTRL);
	__raw_writel(context->mfu_clk_res_ctrl, APMU_MFU_CLK_RES_CTRL);
}

static void pxa168_apbclk_save(struct apbclk_regs *context)
{
	context->uart0 = __raw_readl(APBC_PXA168_UART0);
	context->uart1 = __raw_readl(APBC_PXA168_UART1);
	context->gpio = __raw_readl(APBC_PXA168_GPIO);
	context->pwm0 = __raw_readl(APBC_PXA168_PWM0);
	context->pwm1 = __raw_readl(APBC_PXA168_PWM1);
	context->pwm2 = __raw_readl(APBC_PXA168_PWM2);
	context->pwm3 = __raw_readl(APBC_PXA168_PWM3);
	context->rtc = __raw_readl(APBC_PXA168_RTC);
	context->twsi0 = __raw_readl(APBC_PXA168_TWSI0);
	context->twsi1 = __raw_readl(APBC_PXA168_TWSI1);
	context->kpc = __raw_readl(APBC_PXA168_KPC);
	context->timers = __raw_readl(APBC_PXA168_TIMERS);
	context->aib = __raw_readl(APBC_PXA168_AIB);
	context->sw_jtag = __raw_readl(APBC_PXA168_SW_JTAG);
	context->timer1 = __raw_readl(APBC_PXA168_TIMER1);
	context->onewire = __raw_readl(APBC_PXA168_ONEWIRE);
	context->asfar = __raw_readl(APBC_PXA168_ASFAR);
	context->assar = __raw_readl(APBC_PXA168_ASSAR);
	context->uart2 = __raw_readl(APBC_PXA168_UART2);
	context->timer2 = __raw_readl(APBC_PXA168_TIMER2);
	context->ac97 = __raw_readl(APBC_PXA168_AC97);
	context->ssp0 = __raw_readl(APBC_PXA168_SSP0);
	context->ssp1 = __raw_readl(APBC_PXA168_SSP1);
	context->ssp2 = __raw_readl(APBC_PXA168_SSP2);
	context->ssp3 = __raw_readl(APBC_PXA168_SSP3);
	context->ssp4 = __raw_readl(APBC_PXA168_SSP4);
}

static void pxa168_apbclk_restore(struct apbclk_regs *context)
{
	__raw_writel(context->uart0, APBC_PXA168_UART0);
	__raw_writel(context->uart1, APBC_PXA168_UART1);
	__raw_writel(context->gpio, APBC_PXA168_GPIO);
	__raw_writel(context->pwm0, APBC_PXA168_PWM0);
	__raw_writel(context->pwm1, APBC_PXA168_PWM1);
	__raw_writel(context->pwm2, APBC_PXA168_PWM2);
	__raw_writel(context->pwm3, APBC_PXA168_PWM3);
	__raw_writel(context->rtc, APBC_PXA168_RTC);
	__raw_writel(context->twsi0, APBC_PXA168_TWSI0);
	__raw_writel(context->twsi1, APBC_PXA168_TWSI1);
	__raw_writel(context->kpc, APBC_PXA168_KPC);
	__raw_writel(context->timers, APBC_PXA168_TIMERS);
	__raw_writel(context->aib, APBC_PXA168_AIB);
	__raw_writel(context->sw_jtag, APBC_PXA168_SW_JTAG);
	__raw_writel(context->timer1, APBC_PXA168_TIMER1);
	__raw_writel(context->onewire, APBC_PXA168_ONEWIRE);
	__raw_writel(context->asfar, APBC_PXA168_ASFAR);
	__raw_writel(context->assar, APBC_PXA168_ASSAR);
	__raw_writel(context->uart2, APBC_PXA168_UART2);
	__raw_writel(context->timer2, APBC_PXA168_TIMER2);
	__raw_writel(context->ac97, APBC_PXA168_AC97);
	__raw_writel(context->ssp0, APBC_PXA168_SSP0);
	__raw_writel(context->ssp1, APBC_PXA168_SSP1);
	__raw_writel(context->ssp2, APBC_PXA168_SSP2);
	__raw_writel(context->ssp3, APBC_PXA168_SSP3);
	__raw_writel(context->ssp4, APBC_PXA168_SSP4);
}

static void pxa168_ciu_save(struct ciu_regs *context)
{
	context->chip_id = __raw_readl(CIU_CHIP_ID);
	context->cpu_conf = __raw_readl(CIU_CPU_CONF);
	context->cpu_sram_spd = __raw_readl(CIU_CPU_SRAM_SPD);
	context->cpu_l2c_sram_spd = __raw_readl(CIU_CPU_L2C_SRAM_SPD);
	context->mcb_conf = __raw_readl(CIU_MCB_CONF);
	context->sys_boot_cntrl = __raw_readl(CIU_SYS_BOOT_CNTRL);
	context->sw_branch_addr = __raw_readl(CIU_SW_BRANCH_ADDR);
	context->perf_count0_cntrl = __raw_readl(CIU_PERF_COUNT0_CNTRL);
	context->perf_count1_cntrl = __raw_readl(CIU_PERF_COUNT0_CNTRL);
	context->perf_count2_cntrl = __raw_readl(CIU_PERF_COUNT0_CNTRL);
	context->perf_count0 = __raw_readl(CIU_PERF_COUNT0);
	context->perf_count1 = __raw_readl(CIU_PERF_COUNT1);
	context->perf_count2 = __raw_readl(CIU_PERF_COUNT2);
	context->mc_conf = __raw_readl(CIU_MC_CONF);
	context->mcb_sram_spd = __raw_readl(CIU_MCB_SRAM_SPD);
	context->axi_sram_spd = __raw_readl(CIU_AXI_SRAM_SPD);
}

static void pxa168_ciu_restore(struct ciu_regs *context)
{
	__raw_writel(context->chip_id, CIU_CHIP_ID);
	__raw_writel(context->cpu_conf, CIU_CPU_CONF);
	__raw_writel(context->cpu_sram_spd, CIU_CPU_SRAM_SPD);
	__raw_writel(context->cpu_l2c_sram_spd, CIU_CPU_L2C_SRAM_SPD);
	__raw_writel(context->mcb_conf, CIU_MCB_CONF);
	__raw_writel(context->sys_boot_cntrl, CIU_SYS_BOOT_CNTRL);
	__raw_writel(context->sw_branch_addr, CIU_SW_BRANCH_ADDR);
	__raw_writel(context->perf_count0_cntrl, CIU_PERF_COUNT0_CNTRL);
	__raw_writel(context->perf_count1_cntrl, CIU_PERF_COUNT0_CNTRL);
	__raw_writel(context->perf_count2_cntrl, CIU_PERF_COUNT0_CNTRL);
	__raw_writel(context->perf_count0, CIU_PERF_COUNT0);
	__raw_writel(context->perf_count1, CIU_PERF_COUNT1);
	__raw_writel(context->perf_count2, CIU_PERF_COUNT2);
	__raw_writel(context->mc_conf, CIU_MC_CONF);
	__raw_writel(context->mcb_sram_spd, CIU_MCB_SRAM_SPD);
	__raw_writel(context->axi_sram_spd, CIU_AXI_SRAM_SPD);
}

static void pxa168_squ_save(struct squ_regs *context)
{
	context->ctrl_0 = __raw_readl(SQU_CTRL_0);
	context->ctrl_1 = __raw_readl(SQU_CTRL_1);
	context->ctrl_2 = __raw_readl(SQU_CTRL_2);
	context->fmbist_ctrl_0 = __raw_readl(SQU_FMBIST_CTRL_0);
	context->fmbist_ctrl_1 = __raw_readl(SQU_FMBIST_CTRL_1);
	context->perf_count_cntrl = __raw_readl(SQU_PERF_COUNT_CNTRL);
	context->cam_ent_bank0 = __raw_readl(SQU_CAM_ENT_BANK0);
	context->cam_ent_bank1 = __raw_readl(SQU_CAM_ENT_BANK1);
	context->cam_ent_bank2 = __raw_readl(SQU_CAM_ENT_BANK2);
	context->cam_ent_bank3 = __raw_readl(SQU_CAM_ENT_BANK3);
	context->cam_ent_bank4 = __raw_readl(SQU_CAM_ENT_BANK4);
	context->cam_ent_bank5 = __raw_readl(SQU_CAM_ENT_BANK5);
	context->chan_0_ctrl = __raw_readl(SQU_CHAN_0_CTRL);
	context->chan_1_ctrl = __raw_readl(SQU_CHAN_1_CTRL);
	context->chan_pri = __raw_readl(SQU_CHAN_PRI);
}

static void pxa168_squ_restore(struct squ_regs *context)
{
	__raw_writel(context->ctrl_0, SQU_CTRL_0);
	__raw_writel(context->ctrl_1, SQU_CTRL_1);
	__raw_writel(context->ctrl_2, SQU_CTRL_2);
	__raw_writel(context->fmbist_ctrl_0, SQU_FMBIST_CTRL_0);
	__raw_writel(context->fmbist_ctrl_1, SQU_FMBIST_CTRL_1);
	__raw_writel(context->perf_count_cntrl, SQU_PERF_COUNT_CNTRL);
	__raw_writel(context->cam_ent_bank0, SQU_CAM_ENT_BANK0);
	__raw_writel(context->cam_ent_bank1, SQU_CAM_ENT_BANK1);
	__raw_writel(context->cam_ent_bank2, SQU_CAM_ENT_BANK2);
	__raw_writel(context->cam_ent_bank3, SQU_CAM_ENT_BANK3);
	__raw_writel(context->cam_ent_bank4, SQU_CAM_ENT_BANK4);
	__raw_writel(context->cam_ent_bank5, SQU_CAM_ENT_BANK5);
	__raw_writel(context->chan_0_ctrl, SQU_CHAN_0_CTRL);
	__raw_writel(context->chan_1_ctrl, SQU_CHAN_1_CTRL);
	__raw_writel(context->chan_pri, SQU_CHAN_PRI);
}

#define AXIFAB_REGS_VIRT	(AXI_VIRT_BASE + 0x10000)

static void pxa168_axifab_save(struct axifab_regs *context)
{
	int i;

	context->timeout = __raw_readl(AXIFAB_REGS_VIRT + 0x220);
	context->timeout_status = __raw_readl(AXIFAB_REGS_VIRT + 0x240);
	for (i = 0; i < 4; i++) {
		context->port[4*i]   = __raw_readl(AXIFAB_REGS_VIRT + 0x1000 * i + 0x408);
		context->port[4*i+1] = __raw_readl(AXIFAB_REGS_VIRT + 0x1000 * i + 0x40c);
		context->port[4*i+2] = __raw_readl(AXIFAB_REGS_VIRT + 0x1000 * i + 0x428);
		context->port[4*i+3] = __raw_readl(AXIFAB_REGS_VIRT + 0x1000 * i + 0x42c);
	}
	for (i = 0; i < 3; i++) {
		context->port[16+2*i]   = __raw_readl(AXIFAB_REGS_VIRT + 0x20 * i + 0x448);
		context->port[16+2*i+1] = __raw_readl(AXIFAB_REGS_VIRT + 0x20 * i + 0x44c);
	}
}

static void pxa168_axifab_restore(struct axifab_regs *context)
{
	int i;

	__raw_writel(context->timeout, AXIFAB_REGS_VIRT + 0x220);
	__raw_writel(context->timeout_status, AXIFAB_REGS_VIRT + 0x240);
	for (i = 0; i < 4; i++) {
		__raw_writel(context->port[4*i],   AXIFAB_REGS_VIRT + 0x1000 * i + 0x408);
		__raw_writel(context->port[4*i+1], AXIFAB_REGS_VIRT + 0x1000 * i + 0x40c);
		__raw_writel(context->port[4*i+2], AXIFAB_REGS_VIRT + 0x1000 * i + 0x428);
		__raw_writel(context->port[4*i+3], AXIFAB_REGS_VIRT + 0x1000 * i + 0x42c);
	}
	for (i = 0; i < 3; i++) {
		__raw_writel(context->port[16+2*i],   AXIFAB_REGS_VIRT + 0x20 * i + 0x448);
		__raw_writel(context->port[16+2*i+1], AXIFAB_REGS_VIRT + 0x20 * i + 0x44c);
	}
}

static void pxa168_smc_save(struct smc_regs *context)
{
	context->msc0 = __raw_readl(SMC_MSC0);
	context->msc1 = __raw_readl(SMC_MSC1);
	context->sxcnfg0 = __raw_readl(SMC_SXCNFG0);
	context->sxcnfg1 = __raw_readl(SMC_SXCNFG1);
	context->memclkcfg = __raw_readl(SMC_MEMCLKCFG);
	context->csdficfg0 = __raw_readl(SMC_CSDFICFG0);
	context->csdficfg1 = __raw_readl(SMC_CSDFICFG1);
	context->clk_ret_del = __raw_readl(SMC_CLK_RET_DEL);
	context->adv_ret_del = __raw_readl(SMC_ADV_RET_DEL);
	context->csadrmap0 = __raw_readl(SMC_CSADRMAP0);
	context->csadrmap1 = __raw_readl(SMC_CSADRMAP1);
	context->we_ap0 = __raw_readl(SMC_WE_AP0);
	context->we_ap1 = __raw_readl(SMC_WE_AP1);
	context->oe_ap0 = __raw_readl(SMC_OE_AP0);
	context->oe_ap1 = __raw_readl(SMC_OE_AP1);
	context->adv_ap0 = __raw_readl(SMC_ADV_AP0);
	context->adv_ap1 = __raw_readl(SMC_ADV_AP1);
}

static void pxa168_smc_restore(struct smc_regs *context)
{
	__raw_writel(context->msc0, SMC_MSC0);
	__raw_writel(context->msc1, SMC_MSC1);
	__raw_writel(context->sxcnfg0, SMC_SXCNFG0);
	__raw_writel(context->sxcnfg1, SMC_SXCNFG1);
	__raw_writel(context->memclkcfg, SMC_MEMCLKCFG);
	__raw_writel(context->csdficfg0, SMC_CSDFICFG0);
	__raw_writel(context->csdficfg1, SMC_CSDFICFG1);
	__raw_writel(context->clk_ret_del, SMC_CLK_RET_DEL);
	__raw_writel(context->adv_ret_del, SMC_ADV_RET_DEL);
	__raw_writel(context->csadrmap0, SMC_CSADRMAP0);
	__raw_writel(context->csadrmap1, SMC_CSADRMAP1);
	__raw_writel(context->we_ap0, SMC_WE_AP0);
	__raw_writel(context->we_ap1, SMC_WE_AP1);
	__raw_writel(context->oe_ap0, SMC_OE_AP0);
	__raw_writel(context->oe_ap1, SMC_OE_AP1);
	__raw_writel(context->adv_ap0, SMC_ADV_AP0);
	__raw_writel(context->adv_ap1, SMC_ADV_AP1);
}

static void pxa168_sysbus_save(struct pxa168_pm_regs *context)
{
	pxa168_ciu_save(&(context->ciu));
	pxa168_squ_save(&(context->squ));
	pxa168_axifab_save(&(context->axifab));
	pxa168_smc_save(&(context->smc));
	pxa168_intc_save(&(context->intc));
	pxa168_apbclk_save(&(context->apbclk));
	pxa168_mfp_save(&(context->mfp));
	pxa168_gpio_save(&(context->gpio));
	pxa168_apmu_save(&(context->apmu));
	pxa168_mpmu_save(&(context->mpmu));
}

static void pxa168_sysbus_restore(struct pxa168_pm_regs *context)
{
	pxa168_ciu_restore(&(context->ciu));
	pxa168_squ_restore(&(context->squ));
	pxa168_axifab_restore(&(context->axifab));
	pxa168_mpmu_restore(&(context->mpmu));
	pxa168_apmu_restore(&(context->apmu));
	pxa168_intc_restore(&(context->intc));
	pxa168_apbclk_restore(&(context->apbclk));
	pxa168_smc_restore(&(context->smc));
	pxa168_mfp_restore(&(context->mfp));
	pxa168_gpio_restore(&(context->gpio));
}

/*
static int hibernate_gpio_init(void)
{
	gpio_direction_output(ex_gpio, 1);
	mdelay(100);
	return 0;
}*/

static int pxa168_pm_enter_sleep(struct pxa168_pm_regs *pm_regs)
{
	int i;

	pm_regs->data_pool = (unsigned char *)0xC0000000;

	pxa168_sysbus_save(pm_regs);

	/* should set:modeSaveFlags, areaAddress, flushFunc, psprAddress,
	 * extendedChecksumByteCount */
	pm_regs->pm_data.modeSaveFlags = 0x3f;	/* PM_MODE_SAVE_FLAG_SVC; */
	pm_regs->pm_data.flushFunc = flush_cpu_cache;
	pm_regs->pm_data.areaAddress = (unsigned int)&(pm_regs->pm_data);
	pm_regs->pm_data.extendedChecksumByteCount =
		sizeof(struct pxa168_pm_regs) - sizeof(struct pm_save_data);
	printk("ext size:%d, save size%d\n",
		pm_regs->pm_data.extendedChecksumByteCount,
		sizeof(struct pm_save_data));

	pm_regs->word0 = __raw_readl(pm_regs->data_pool);
	pm_regs->word1 = __raw_readl(pm_regs->data_pool + 4);
	pm_regs->word2 = __raw_readl(pm_regs->data_pool + 8);

	/* Write resume back address to SDRAM */
	__raw_writel(virt_to_phys(pxa168_cpu_resume), pm_regs->data_pool);
	__raw_writel(virt_to_phys(&(pm_regs->pm_data)), pm_regs->data_pool + 4);

	/* Write Hibernate mode indicator to SDRAM */
	__raw_writel(0x55AA55AA, pm_regs->data_pool + 8);

	pxa168_cpu_sleep((unsigned int)&(pm_regs->pm_data),
			virt_to_phys(&(pm_regs->pm_data)));

	/* come back */
	__raw_writel(pm_regs->word0, pm_regs->data_pool);
	__raw_writel(pm_regs->word1, pm_regs->data_pool + 4);
	__raw_writel(pm_regs->word2, pm_regs->data_pool + 8);

	pxa168_sysbus_restore(pm_regs);

	/* unmask GPIO edge detection for all 4 banks - APMASKx */
	for (i = 0; i < 4; i++)
		__raw_writel(0xffffffff, APMASK(i));

	//printk("*** made it back from sleep\n");
	
	return 0;
}



/*
 * Called after processes are frozen, but before we shut down devices.
 */
static int pxa168_pm_prepare(void)
{
	return 0;
}

/*
 * Called after devices are re-setup, but before processes are thawed.
 */
static void pxa168_pm_finish(void)
{
	pm_state = PM_SUSPEND_ON;
}

static int pxa168_pm_valid(suspend_state_t state)
{
	int ret = 1;

	if (state == PM_SUSPEND_MEM) {
		pm_state = PM_SUSPEND_MEM;
	} else if (state == PM_SUSPEND_STANDBY) {
		pm_state = PM_SUSPEND_STANDBY;
	} else {
		ret = 0;
	}
	return ret;
}

static ssize_t sleeptime_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%u\n", pm_sleeptime);
}

static ssize_t sleeptime_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t len)
{
	sscanf(buf, "%u", &pm_sleeptime);
	return len;
}

void pxa168_pm_enter_core_extidle(void)
{
	uint32_t icu_ap_gbl_irq_msk;

	/* step 1: set the wakeup source */
	/* It should be woke up by ICU interrupt */

	/* step 2: set the IDLE bit in the AP idle configuration register*/
	/*********************************************************************
	 * set:	DIS_MC_SW_REQ(21), MC_WAKE_EN(20), L2_RESETn(9), and IDLE(1)
	 * *******************************************************************/
	if(cpu_is_pxa168_A0()==0)
		__raw_writel(0x28000000, APMU_PCR );	/* ensure SETALWAYS bits = 1 */
	else
		__raw_writel(0x08000000, APMU_PCR );	/* ensure SETALWAYS bits = 1 */

	__raw_writel(0x00300302, APMU_IDLE_CFG);

	/* step 3: set the global interrupt mask in the ICU register to mask
	 * the SYNC IRQ to the core */
	icu_ap_gbl_irq_msk = __raw_readl(ICU_AP_GBL_IRQ_MSK);
	icu_ap_gbl_irq_msk |= ICU_AP_GBL_IRQ_MSK_IRQ_MSK | \
				ICU_AP_GBL_IRQ_MSK_FIQ_MSK;
	__raw_writel(icu_ap_gbl_irq_msk, ICU_AP_GBL_IRQ_MSK);

	/* step 4: set the AXISDD bit in the AP power control register to 1*/

	/* step 5: set the DDRCORSD and APBSD bits in the MPMU_APCR to 1 */

	/* step 6: set the SLPEN bit in the MPMU_APCR to 1 */

	/* step 7: program the memory controller hardware sleep type */
	/*********************************************************************
	 * set:	SetAlways(28),
	 *	SetAlways(25),
	 *	SetAlways(14)
	 *********************************************************************/
	__raw_writel(0x12004000, MPMU_APCR);

	pxa168_pm_swi();
}


void pxa168_pm_enter_sys_sleep(void)
{
	uint32_t icu_ap_gbl_irq_msk;
	unsigned int icu_int_conf[64];
	unsigned long flags;
	int i;

	for (i = 0; i < 64; i++) {
		icu_int_conf[i] = __raw_readl(ICU_INT_CONF(i));
		if (
			(IRQ_PXA168_KEYPAD != i) &&
			(IRQ_PXA168_TIMER1 != i) &&
			(IRQ_PXA168_TIMER2 != i) &&
			(IRQ_PXA168_TIMER3 != i)
		)
			__raw_writel(icu_int_conf[i]&0xfffffff8, \
					ICU_INT_CONF(i));
	}

	/* step 1: set the wakeup source */
	/*********************************************************************
	 * unmask:      Keypress(21,wk3),
	 *	      RTC_ALARM(17,wk4),
	 *	      AP1_TIMER3(10,wk4), AP1_TIMER2(9,wk4), AP1_TIMER1(8,wk4)
	 *	      WAKEUP4,3(4,3)
	 * note:	must mask WAKEUP5(5) and WAKEUP1(1)(for USB port)
	 *********************************************************************/
	__raw_writel(0x00220718, MPMU_AWUCRM);

	/* step 2: set the IDLE bit in the AP idle configuration register*/
	if(cpu_is_pxa168_A0()==0)
		__raw_writel(0x28000000, APMU_PCR);    /* force SETALWAYS=1 */
	else
		__raw_writel(0x08000000, APMU_PCR);    /* force SETALWAYS=1 */

	__raw_writel(0x300202, APMU_IDLE_CFG);

	/* step 3: set the global interrupt mask in the ICU register to mask
	 * the SYNC IRQ to the core */
	icu_ap_gbl_irq_msk = __raw_readl(ICU_AP_GBL_IRQ_MSK);
	icu_ap_gbl_irq_msk |= ICU_AP_GBL_IRQ_MSK_IRQ_MSK | \
			ICU_AP_GBL_IRQ_MSK_FIQ_MSK;
	__raw_writel(icu_ap_gbl_irq_msk, ICU_AP_GBL_IRQ_MSK);

	/* step 4: set the AXISDD bit in the AP power control register to 1*/

	/* step 5: set the DDRCORSD and APBSD bits in the MPMU_APCR to 1 */

	/* step 6: set the SLPEN bit in the MPMU_APCR to 1 */

	/* step 7: program the memory controller hardware sleep type */
	/*********************************************************************
	* set: AXISDD(31), SLPEN(29), SetAlways(28), DDRCORESD(27), APBSD(26)
	*      SetAlways(25), SLPWP0,1,2,5,6,7(23,22,21,17,16,15),
	*      SetAlways(14)
	*********************************************************************/
	local_fiq_disable();
	local_irq_save(flags);

	pxa168_trigger_lpm(0xbee3c000);

	local_irq_restore(flags);
	local_fiq_enable();

	/* retain the ext idle config as default */
	__raw_writel(0x300202, APMU_IDLE_CFG);
	__raw_writel(0x00000000, MPMU_APCR);

	for (i = 0; i < 64; i++) {
		if (
			(IRQ_PXA168_KEYPAD != i) &&
			(IRQ_PXA168_TIMER1 != i) &&
			(IRQ_PXA168_TIMER2 != i) &&
			(IRQ_PXA168_TIMER3 != i)
		)
			__raw_writel(icu_int_conf[i], ICU_INT_CONF(i));
	}
}

void pxa168_pm_enter_sys_sleep_edge(void)
{
	uint32_t icu_ap_gbl_irq_msk;
	unsigned int icu_int_conf[64];
	int i;
	for (i = 0; i < 64; i++) {
		icu_int_conf[i] = __raw_readl(ICU_INT_CONF(i));
		if (IRQ_PXA168_KEYPAD != i)
			__raw_writel(icu_int_conf[i]&0xfffffff8, \
					ICU_INT_CONF(i));
	}
	/* step 1: set the wakeup source */
	/*__raw_writel(0x002207dd, MPMU_AWUCRM);*/
	__raw_writel(0x200008, MPMU_AWUCRM);

	/* step 2: set the IDLE bit in the AP idle configuration register*/
	if(cpu_is_pxa168_A0()==0)
		__raw_writel(0x28000000, APMU_PCR );	/* ensure SETALWAYS bits = 1 */
	else
		__raw_writel(0x08000000, APMU_PCR );	/* ensure SETALWAYS bits = 1 */

	__raw_writel(0x300202, APMU_IDLE_CFG);

	/* step 3: set the global interrupt mask in the ICU register to mask
	 * the SYNC IRQ to the core */
	icu_ap_gbl_irq_msk = __raw_readl(ICU_AP_GBL_IRQ_MSK);
	icu_ap_gbl_irq_msk |= ICU_AP_GBL_IRQ_MSK_IRQ_MSK | \
			ICU_AP_GBL_IRQ_MSK_FIQ_MSK;
	__raw_writel(icu_ap_gbl_irq_msk, ICU_AP_GBL_IRQ_MSK);

	/* step 4: set the AXISDD bit in the AP power control register to 1*/

	/* step 5: set the DDRCORSD and APBSD bits in the MPMU_APCR to 1 */

	/* step 6: set the SLPEN bit in the MPMU_APCR to 1 */

	/* step 7: program the memory controller hardware sleep type */
	__raw_writel(0xac000000, MPMU_APCR);

	pxa168_pm_swi();
	/* retain the ext idle config as default */
	__raw_writel(0x300202, APMU_IDLE_CFG);
	__raw_writel(0x00000000, MPMU_APCR);
	for (i = 0; i < 64; i++) {
		if (IRQ_PXA168_KEYPAD != i)
			__raw_writel(icu_int_conf[i], ICU_INT_CONF(i));
	}

}



#define	pxa168_pm_enter_hibernate() do {} while (0)



void pxa168_pm_enter_lowpower_mode(int state)
{
	struct op_info *info = NULL;
	int op;
	op = dvfm_get_op(&info);
	switch (state) {
	case POWER_MODE_CORE_EXTIDLE:
		mspm_add_event(op, CPU_STATE_RUN);
		pxa168_pm_enter_core_extidle();
		mspm_add_event(op, CPU_STATE_IDLE);
		break;
	case POWER_MODE_SYS_SLEEP:
		mspm_add_event(op, CPU_STATE_RUN);
		pxa168_pm_enter_sys_sleep();
		mspm_add_event(op, CPU_STATE_IDLE);
		break;
	case POWER_MODE_HIBERNATE:
		pxa168_pm_enter_hibernate();
		break;
	}
}

static int pxa168_pm_enter(suspend_state_t state)
{
	int op;
	struct op_info *info = NULL;
	if (state == PM_SUSPEND_MEM)
		if (machine_is_edge()) {
			op = dvfm_get_op(&info);
			dvfm_request_op(0);
			pxa168_pm_enter_sys_sleep_edge();
			dvfm_request_op(op);
			return 0;
		} else {
			return pxa168_pm_enter_sleep(&pxa168_pm_regs);
		}
	else
		return -EINVAL;
}

/*
 * Set to PM_DISK_FIRMWARE so we can quickly veto suspend-to-disk.
 */
static struct platform_suspend_ops pxa168_pm_ops = {
	.valid		= pxa168_pm_valid,
	.prepare	= pxa168_pm_prepare,
	.enter		= pxa168_pm_enter,
	.finish		= pxa168_pm_finish,
};


static int tokenizer(char **tbuf, const char *userbuf, ssize_t n,
			char **tokptrs, int maxtoks)
{
	char *cp, *tok;
	char *whitespace = " \t\r\n";
	int ntoks = 0;
	cp = kmalloc(n + 1, GFP_KERNEL);
	if (!cp)
		return -ENOMEM;

	*tbuf = cp;
	memcpy(cp, userbuf, n);
	cp[n] = '\0';

	do {
		cp = cp + strspn(cp, whitespace);
		tok = strsep(&cp, whitespace);
		if ((*tok == '\0') || (ntoks == maxtoks))
			break;
		tokptrs[ntoks++] = tok;
	} while (cp);

	return ntoks;
}


static ssize_t deepidle_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	int len = 0;

	if (enable_deepidle & IDLE_CORE_EXTIDLE)
		len += sprintf(buf + len, "core_extidle, ");
	if (enable_deepidle & IDLE_SYS_SLEEP)
		len += sprintf(buf + len, "sys_sleep\n");
	len += sprintf(buf + len, "Command: echo [set|unset]\
		[core_extidle|sys_sleep] "
		"> deepidle\n");
	return len;
}
#define MAXTOKENS       80
static ssize_t deepidle_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t len)
{
	int error = 0;
	char *tbuf = NULL;
	char *token[MAXTOKENS];
	int ntoks = tokenizer(&tbuf, buf, len, (char **)&token, MAXTOKENS);

	if (ntoks <= 0) {
		error = ntoks;
		goto out;
	}

	if (strcmp(token[0], "set") == 0) {
		if (strcmp(token[1], "core_extidle") == 0)
			enable_deepidle |= IDLE_CORE_EXTIDLE;
		else if (strcmp(token[1], "sys_sleep") == 0)
			enable_deepidle |= IDLE_SYS_SLEEP;
		else
			error = -EINVAL;
	} else if (strcmp(token[0], "unset") == 0) {
		if (strcmp(token[1], "core_extidle") == 0)
			enable_deepidle &= ~IDLE_CORE_EXTIDLE;
		else if (strcmp(token[1], "sys_sleep") == 0)
			enable_deepidle &= ~IDLE_SYS_SLEEP;
		else
			error = -EINVAL;
	} else {
		if (strcmp(token[0], "0") == 0)
			enable_deepidle = IDLE_ACTIVE;
		else
			error = -EINVAL;
	}
out:
	kfree(tbuf);
	return error ? error : len;
}
static struct kobj_attribute sleeptime_attr = {
	.attr	= {
		.name = __stringify(sleeptime),
		.mode = 0644,
	},
	.show	= sleeptime_show,
	.store	= sleeptime_store,
};

void pxa168_pm_enter_sys_sleep_test(void)
{
	uint32_t icu_ap_gbl_irq_msk;
	unsigned long flags;

	/* step 1: set the wakeup source */
	/*********************************************************************
	 * unmask:	Keypress(21),
	 * 		RTC_ALARM(17), AP1_TIMER_3(10), AP1_TIMER2(9),
	 * 		AP1_TIMER1(8), WAKEUP4,3(4,3)
	 * note:	must mask WAKEUP5(5) and WAKEUP1(1)(for USB port)
	 *********************************************************************/
	__raw_writel(0x00200008, MPMU_AWUCRM);	/* key, wk3 */

	/* step 2: set the IDLE bit in the AP idle configuration register*/
	/*********************************************************************
	 * set:	DIS_MC_SW_REQ(21), MC_WAKE_EN(20), L2_RESETn(9), and IDLE(1)
	 * *******************************************************************/
	__raw_writel(0x00300302, APMU_IDLE_CFG); /* auto_mc_wk, cor_clk_off */

	/* step 3: set the global interrupt mask in the ICU register to mask
	 * the SYNC IRQ to the core */
	icu_ap_gbl_irq_msk = __raw_readl(ICU_AP_GBL_IRQ_MSK);
	icu_ap_gbl_irq_msk |= ICU_AP_GBL_IRQ_MSK_IRQ_MSK | \
			ICU_AP_GBL_IRQ_MSK_FIQ_MSK;
	__raw_writel(icu_ap_gbl_irq_msk, ICU_AP_GBL_IRQ_MSK);

	/* step 4: set the AXISDD bit in the AP power control register to 1*/

	/* step 5: set the DDRCORSD and APBSD bits in the MPMU_APCR to 1 */

	/* step 6: set the SLPEN bit in the MPMU_APCR to 1 */

	/* step 7: program the memory controller hardware sleep type */
	/*********************************************************************
	 * set:	AXISDD(31), SLPEN(29), SetAlways(28), DDRCORESD(27), APBSD(26)
	 *	SetAlways(25), SLPWP0,1,2,5,6,7(23,22,21,17,16,15),
	 *	SetAlways(14)
	 *********************************************************************/
	local_fiq_disable();
	local_irq_save(flags);

	pxa168_trigger_lpm(0xbee3c000); /*mpmu_apcr: splen,sd(axi,ddr,cor,apb)*/

	local_irq_restore(flags);
	local_fiq_enable();

}



void pxa168_pm_enter_lowpower_mode_test(int state)
{

	unsigned int icu_int_conf[64];
	int i;
	if (state == POWER_MODE_CORE_EXTIDLE) {
		for (i = 0; i < 64; i++) {
			icu_int_conf[i] = __raw_readl(ICU_INT_CONF(i));
			if (IRQ_PXA168_KEYPAD != i)
				__raw_writel(icu_int_conf[i]&0xfffffff8, \
						ICU_INT_CONF(i));
		}
	}
	switch (state) {
	case POWER_MODE_CORE_EXTIDLE:
		pxa168_pm_enter_core_extidle();
		break;
	case POWER_MODE_SYS_SLEEP:
		pxa168_pm_enter_sys_sleep_test();
		break;
	case POWER_MODE_HIBERNATE:
		pxa168_pm_enter_hibernate();
		break;
	}
	if (state == POWER_MODE_CORE_EXTIDLE) {
		for (i = 0; i < 64; i++) {
			if (IRQ_PXA168_KEYPAD != i)
				__raw_writel(icu_int_conf[i], ICU_INT_CONF(i));
		}
	}
}

static ssize_t lpidle_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t len)
{
	int error = 0;
	char *tbuf = NULL;
	char *token[MAXTOKENS];
	int ntoks = tokenizer(&tbuf, buf, len, (char **)&token, MAXTOKENS);

	if (ntoks <= 0) {
		error = ntoks;
		goto out;
	}
	if (strcmp(token[0], "core_extidle") == 0)
		pxa168_pm_enter_lowpower_mode_test(POWER_MODE_CORE_EXTIDLE);
	else if (strcmp(token[0], "sys_sleep") == 0)
		pxa168_pm_enter_lowpower_mode_test(POWER_MODE_SYS_SLEEP);
	else
		error = -EINVAL;
out:
	return error ? error : len;
}

static ssize_t lpidle_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	return 0;
}

static struct kobj_attribute lpidle_attr = {
	.attr	= {
		.name = __stringify(lpidle),
		.mode = 0644,
	},
	.show	= lpidle_show,
	.store	= lpidle_store,
};





static struct kobj_attribute  deepidle_attr = {
	.attr	= {
		.name = __stringify(deepidle),
		.mode = 0644,
	},
	.show	= deepidle_show,
	.store	= deepidle_store,
};
static struct attribute * g[] = {
	&sleeptime_attr.attr,
	&deepidle_attr.attr,
	&lpidle_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = g,
};


static int __init pxa168_pm_init(void)
{
	if (!cpu_is_pxa168())
		return -EIO;

	if (sysfs_create_group(power_kobj, &attr_group))
		return -1;

	dmc_membase = ioremap(0xb0000000, 0x00001000);
	sram_membase = ioremap(0xd1020000, 0x00020000);

/*	ex_gpio = GPIO_EXT0(1);
	if (gpio_request(ex_gpio, "EXP2_SYS_DIS_N")) {
		printk("EXP2_SYS_DIS_N Request Failed\n");
		return -EIO;
	}  currently not required */

	suspend_set_ops(&pxa168_pm_ops);
	return 0;
}

late_initcall(pxa168_pm_init);
