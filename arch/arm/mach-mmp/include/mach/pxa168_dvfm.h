/*
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 *
 * (C) Copyright 2007 Marvell International Ltd.
 * All Rights Reserved
 */

#ifndef PXA168_DVFM_H
#define PXA168_DVFM_H

#include <mach/dvfm.h>

#define OP_NAME_LEN		16


struct pxa168_md_opt {
	struct pxa168_opmode_md	*dvfm_settings;	/* reg vals for this mode */
	int			index;		/* legacy way to look up */
	int			power_mode;	/* active, idle, etc. */
	int			vcc_core;	/* core voltage, mV */
	int			pclk;		/* core clock */
	int			baclk;		/* bus interface clock */
	int			xpclk;		/* L2 cache interface clock */
	int			dclk;		/* DDR clock */
	int			aclk;		/* AXI fabric clock */
	int			aclk2;		/* AXI2 fabric clock */
	int			lpj;
	char			name[OP_NAME_LEN];
};

struct pxa168_opmode_md {
	char		name[OP_NAME_LEN];
	int		power_mode;
	unsigned int	corepll_sel;	/* 0=pll1 312, 1=pll1 624, 2=pll2 */
	unsigned int	axipll_sel;	/* 0=pll1 312, 1=pll1 624, 2=pll2 */
	unsigned int	aclk2_div;	/* axi2 clock */
	unsigned int	aclk_div;	/* axi clock */
	unsigned int	dclk_div;	/* ddr clock */
	unsigned int	xpclk_div;	/* xp clock */
	unsigned int	baclk_div;	/* ba clock */
	unsigned int	pclk_div;	/* processor clock */
	unsigned int	pll2_refdiv;
	unsigned int	pll2_fbdiv;
	unsigned int	pll2_reg1;
	unsigned int	vcc_core;
};


#define BIT_0 (1 << 0)
#define BIT_1 (1 << 1)
#define BIT_2 (1 << 2)
#define BIT_3 (1 << 3)
#define BIT_4 (1 << 4)
#define BIT_5 (1 << 5)
#define BIT_6 (1 << 6)
#define BIT_7 (1 << 7)
#define BIT_8 (1 << 8)
#define BIT_9 (1 << 9)
#define BIT_10 (1 << 10)
#define BIT_11 (1 << 11)
#define BIT_12 (1 << 12)
#define BIT_13 (1 << 13)
#define BIT_14 (1 << 14)
#define BIT_15 (1 << 15)
#define BIT_16 (1 << 16)
#define BIT_17 (1 << 17)
#define BIT_18 (1 << 18)
#define BIT_19 (1 << 19)
#define BIT_20 (1 << 20)
#define BIT_21 (1 << 21)
#define BIT_22 (1 << 22)
#define BIT_23 (1 << 23)
#define BIT_24 (1 << 24)
#define BIT_25 (1 << 25)
#define BIT_26 (1 << 26)
#define BIT_27 (1 << 27)
#define BIT_28 (1 << 28)
#define BIT_29 (1 << 29)
#define BIT_30 (1 << 30)
#define BIT_31 ((unsigned)1 << 31)

#define SHIFT0(Val)  (Val)
#define SHIFT1(Val)  ((Val) << 1)
#define SHIFT2(Val)  ((Val) << 2)
#define SHIFT3(Val)  ((Val) << 3)
#define SHIFT4(Val)  ((Val) << 4)
#define SHIFT5(Val)  ((Val) << 5)
#define SHIFT6(Val)  ((Val) << 6)
#define SHIFT7(Val)  ((Val) << 7)
#define SHIFT8(Val)  ((Val) << 8)
#define SHIFT9(Val)  ((Val) << 9)
#define SHIFT10(Val) ((Val) << 10)
#define SHIFT11(Val) ((Val) << 11)
#define SHIFT12(Val) ((Val) << 12)
#define SHIFT13(Val) ((Val) << 13)
#define SHIFT14(Val) ((Val) << 14)
#define SHIFT15(Val) ((Val) << 15)
#define SHIFT16(Val) ((Val) << 16)
#define SHIFT17(Val) ((Val) << 17)
#define SHIFT18(Val) ((Val) << 18)
#define SHIFT19(Val) ((Val) << 19)
#define SHIFT20(Val) ((Val) << 20)
#define SHIFT21(Val) ((Val) << 21)
#define SHIFT22(Val) ((Val) << 22)
#define SHIFT23(Val) ((Val) << 23)
#define SHIFT24(Val) ((Val) << 24)
#define SHIFT25(Val) ((Val) << 25)
#define SHIFT26(Val) ((Val) << 26)
#define SHIFT27(Val) ((Val) << 27)
#define SHIFT28(Val) ((Val) << 28)
#define SHIFT29(Val) ((Val) << 29)
#define SHIFT30(Val) ((Val) << 30)
#define SHIFT31(Val) ((Val) << 31)

/*
 * apmu registers and bits definition
 */
#define	APMU_CCR_OFF				0x0004
#define	APMU_CCSR_OFF				0x000C
#define	APMU_IDLE_CFG_OFF			0x0018
#define	APMU_IMR_OFF				0x0098
#define	APMU_ISR_OFF				0x00A0
#define	APMU_CCR_ACLK_DYN_FC			BIT_30
#define	APMU_CCR_DCLK_DYN_FC			BIT_29
#define	APMU_CCR_CORE_DYN_FC			BIT_28
#define	APMU_CCR_CORE_ALLOW_SPD_CHG		BIT_27
#define	APMU_CCR_BUS_FREQ_CHG_REQ		BIT_26
#define	APMU_CCR_DDR_FREQ_CHG_REQ		BIT_25
#define	APMU_CCR_FREQ_CHG_REQ			BIT_24
#define	APMU_CCR_BUS2_CLK_DIV_MSK		SHIFT18(0x7)
#define	APMU_CCR_BUS2_CLK_DIV_BASE		18
#define	APMU_CCR_BUS_CLK_DIV_MSK		SHIFT15(0x7)
#define	APMU_CCR_BUS_CLK_DIV_BASE		15
#define	APMU_CCR_DDR_CLK_DIV_MSK		SHIFT12(0x7)
#define	APMU_CCR_DDR_CLK_DIV_BASE		12
#define	APMU_CCR_XP_CLK_DIV_MSK			SHIFT9(0x7)
#define	APMU_CCR_XP_CLK_DIV_BASE		9
#define	APMU_CCR_BIU_CLK_DIV_MSK		SHIFT6(0x7)
#define	APMU_CCR_BIU_CLK_DIV_BASE		6
#define	APMU_CCR_CORE_CLK_DIV_MSK		SHIFT0(0x7)
#define	APMU_CCR_CORE_CLK_DIV_BASE		0
#define	APMU_CCSR_BUS2_CLK_DIV_MSK		SHIFT18(0x7)
#define	APMU_CCSR_BUS2_CLK_DIV_BASE		18
#define	APMU_CCSR_BUS_CLK_DIV_MSK		SHIFT15(0x7)
#define	APMU_CCSR_BUS_CLK_DIV_BASE		15
#define	APMU_CCSR_DDR_CLK_DIV_MSK		SHIFT12(0x7)
#define	APMU_CCSR_DDR_CLK_DIV_BASE		12
#define	APMU_CCSR_XP_CLK_DIV_MSK		SHIFT9(0x7)
#define	APMU_CCSR_XP_CLK_DIV_BASE		9
#define	APMU_CCSR_BIU_CLK_DIV_MSK		SHIFT6(0x7)
#define	APMU_CCSR_BIU_CLK_DIV_BASE		6
#define	APMU_CCSR_CORE_CLK_DIV_MSK		SHIFT0(0x7)
#define	APMU_CCSR_CORE_CLK_DIV_BASE		0
#define	APMU_IMR_FC_INTR_MASK			BIT_1
#define	APMU_ISR_FC_ISR				BIT_1

/*
 * mpmu registers and bits definition
 */
#define	MPMU_FCCR_OFF				0x0008
#define	MPMU_POSR_OFF				0x0010
#define	MPMU_PLL2CR_OFF				0x0034
#define	MPMU_ACGR_OFF				0x1024
#define	MPMU_PLL2_REG1_OFF			0x0060
#define	MPMU_PLL2_REG2_OFF			0x0064

#define	MPMU_FCCR_CORECLKSEL_MSK		SHIFT29(0x7)
#define	MPMU_FCCR_CORECLKSEL_BASE		29
#define	MPMU_FCCR_AXICLKSEL_MSK			SHIFT23(0x7)
#define	MPMU_FCCR_AXICLKSEL_BASE		23
#define	MPMU_FCCR_MFC				BIT_15
#define	MPMU_FCCR_PLL1CEN			BIT_14
#define	MPMU_FCCR_PLL1REFD_MSK			SHIFT9(0x1f)
#define	MPMU_FCCR_PLL1REFD_BASE			9
#define	MPMU_FCCR_PLL1FBD_MSK			SHIFT0(0x1ff)
#define	MPMU_FCCR_PLL1FBD_BASE			0

#define	MPMU_PLL2CR_FBDIV_MSK			SHIFT10(0x1ff)
#define	MPMU_PLL2CR_REFDIV_MSK			SHIFT19(0x1f)

#define	MPMU_PLL2_REG1_VCODIV_SEL_DIFF_MSK	SHIFT19(0xf)
#define	MPMU_PLL2_REG1_VCODIV_SEL_DIFF_BASE	19

#define	MPMU_POSR_PLL2REFD_MSK			SHIFT23(0x1f)
#define	MPMU_POSR_PLL2REFD_BASE			23
#define	MPMU_POSR_PLL2FBD_MSK			SHIFT14(0x1ff)
#define	MPMU_POSR_PLL2FBD_BASE			14
#define	MPMU_POSR_PLL1REFD_MSK			SHIFT9(0x1f)
#define	MPMU_POSR_PLL1REFD_BASE			9
#define	MPMU_POSR_PLL1FBD_MSK			SHIFT0(0x1ff)
#define	MPMU_POSR_PLL1FBD_BASE			0

struct pxa168_dvfm_info {
	uint32_t cpuid;
	unsigned char __iomem *pmum_base;
	unsigned char __iomem *pmua_base;
};

extern void pxa168_op_machine_to_human(struct pxa168_opmode_md  *opmode_md,\
		struct pxa168_md_opt *opmode_hu);

extern int pxa168_trigger_dfc(uint32_t apmu_ccr);
extern unsigned long  pxa168_dfc_get_lockcache_location(unsigned long *base, \
		unsigned int *size);

extern int pxa168_trigger_lpm(uint32_t mpmu_apcr);
extern unsigned long  pxa168_lpm_get_lockcache_location(unsigned long *base, \
		unsigned int *size);

extern void pxa168_get_current_opmode_md(struct pxa168_dvfm_info *driver_data,\
		struct pxa168_opmode_md *opmode_md);


#endif
