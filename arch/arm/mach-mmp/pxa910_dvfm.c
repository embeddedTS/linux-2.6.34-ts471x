/*
 * PXA910 DVFM Driver
 *
 * Copyright (C) 2008 Marvell Corporation
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 *
 * (C) Copyright 2007 Marvell International Ltd.
 * All Rights Reserved
 */

#undef DEBUG

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/sysdev.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/list.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/err.h>

#include <asm/io.h>

#include <mach/cputype.h>
#include <mach/hardware.h>
#include <mach/dvfm.h>
#include <mach/pxa168_pm.h>
#include <mach/pxa910_pm.h>
#include <mach/pxa910_dvfm.h>

#include <plat/pxa3xx_pmic.h>

#define CONFIG_PXA910_DVFM_ASYNC_MODE 1
#define CONFIG_PXA910_AP_ALONE_MODE 1

struct pxa910_dvfm_info {
	uint32_t cpuid;
	unsigned char __iomem *pmum_base;
	unsigned char __iomem *pmua_base;
};

static struct info_head pxa910_dvfm_op_list = {
	.list = LIST_HEAD_INIT(pxa910_dvfm_op_list.list),
	.lock = RW_LOCK_UNLOCKED,
};

/* the operating point preferred by policy maker or user */
static int preferred_op;

extern unsigned int cur_op;		/* current operating point */
extern unsigned int def_op;		/* default operating point */

static int dvfm_dev_id;

static struct pxa910_md_opt pxa910_op_array[] = {
	/* core 26MHz ddr 13MHz bus 26MHz */
	{
		.power_mode = POWER_MODE_ACTIVE,
		.vcc_core = 1400,
		.pclk = 26,
		.pdclk = 26,
		.baclk = 26,
		.xpclk = 26,
		.dclk = 13,
		.aclk = 26,
		.lpj = 26*(500000)/HZ,
		.name = "26MHz",
	},
	/* core 78MHz ddr 78MHz bus 78MHz */
	{
		.power_mode = POWER_MODE_ACTIVE,
		.vcc_core = 1400,
		.pclk = 78,
		.pdclk = 78,
		.baclk = 78,
		.xpclk = 78,
		.dclk = 78,
		.aclk = 78,
		.lpj = 78*(500000)/HZ,
		.name = "78MHz",
	},
	/* core 156MHz ddr 104MHz bus 104MHz */
	{
		.power_mode = POWER_MODE_ACTIVE,
		.vcc_core = 1400,
		.pclk = 156,
		.pdclk = 104,
		.baclk = 104,
		.xpclk = 104,
		.dclk = 104,
		.aclk = 104,
		.lpj = 156*500000/HZ,
		.name = "156MHz",
	},
	/* core 312MHz ddr 156MHz bus 156MHz */
	{
		.power_mode = POWER_MODE_ACTIVE,
		.vcc_core = 1400,
		.pclk = 312,
		.pdclk = 156,
		.baclk = 156,
		.xpclk = 156,
		.dclk = 156,
		.aclk = 156,
		.lpj = 312*500000/HZ,
		.name = "312MHz",
	},
	/* core 624MHz ddr 156MHz bus 156MHz */
	{
		.power_mode = POWER_MODE_ACTIVE,
		.vcc_core = 1400,
		.pclk = 624,
		.pdclk = 156,
		.baclk = 156,
		.xpclk = 312,
		.dclk = 156,
		.aclk = 156,
		.lpj = 624*500000/HZ,
		.name = "624MHz",
	},
	/* core internal idle */
	{
		.power_mode = POWER_MODE_CORE_INTIDLE,
		.vcc_core = 1400,
		.name = "core_intidle",
	},
	/* core external idle */
	{
		.power_mode = POWER_MODE_CORE_EXTIDLE,
		.vcc_core = 1400,
		.name = "core_extidle",
	},
	/* application subsystem idle */
	{
		.power_mode = POWER_MODE_APPS_IDLE,
		.vcc_core = 1400,
		.name = "apps_idle",
	},
	/* application subsystem sleep */
	{
		.power_mode = POWER_MODE_APPS_SLEEP,
		.vcc_core = 1400,
		.name = "apps_sleep",
	},
	/* system sleep */
	{
		.power_mode = POWER_MODE_SYS_SLEEP,
		.vcc_core = 1400,
		.name = "sys_sleep",
	},
};

struct proc_op_array {
	unsigned int cpuid;
	char *cpu_name;
	struct pxa910_md_opt *op_array;
	unsigned int nr_op;
};

#define ARRAY_AND_SIZE(x)	(x), ARRAY_SIZE(x)
static struct proc_op_array proc_op_arrays[] = {
	{0x8000, "PXA910", ARRAY_AND_SIZE(pxa910_op_array)},
};

/* #####################Debug Function######################## */
static int dump_op(void *driver_data, struct op_info *p, char *buf)
{
	int len, count, x, i, max, sum;
	struct pxa910_md_opt *q = (struct pxa910_md_opt *)p->op;

	if (q == NULL)
		len = sprintf(buf, "Can't dump the op info\n");
	else {
		/* calculate how much bits is set in device word */
		max = DVFM_MAX_CLIENT >> 5;
		for (i = 0, sum = 0; i < max; i++) {
			x = p->device[i];
			for (count = 0; x; x = x & (x - 1), count++);
			sum += count;
		}
		len = sprintf(buf, "OP:%d name:%s [%s, %d]\n",
				p->index, q->name, (sum)?"Disabled"
				:"Enabled", sum);
		len += sprintf(buf + len, "pclk:%d pdclk:%d baclk:%d xpclk:%d "
				"dclk:%d aclk:%d vcc_core:%d\n",
				q->pclk, q->pdclk, q->baclk, q->xpclk,
				q->dclk, q->aclk, q->vcc_core);
	}
	return len;
}

static int dump_op_list(void *driver_data, struct info_head *op_table)
{
	struct op_info *p = NULL;
	struct list_head *list = NULL;
	struct pxa910_dvfm_info *info = driver_data;
	char buf[256];

	if (!op_table || list_empty(&op_table->list)) {
		printk(KERN_WARNING "op list is null\n");
		return -EINVAL;
	}
	memset(buf, 0, 256);
	list_for_each(list, &op_table->list) {
		p = list_entry(list, struct op_info, list);
		dump_op(info, p, buf);
	}
	return 0;
}

/* Get current setting, and record it in fv_info structure
 */
static int capture_op_info(void *driver_data, struct pxa910_md_opt *fv_info)
{
	struct pxa910_dvfm_info *info = driver_data;
	int res = -EFAULT;
	uint32_t fccr, core2, pll_ap = 0, pll_da = 0, temp;

	if (fv_info) {
		memset(fv_info, 0, sizeof(struct pxa910_md_opt));
		temp = readl(info->pmua_base + CC_MOH_OFF);
		temp &= ~PMUA_CC_MOH_MOH_RD_ST_CLEAR;
		writel(temp, info->pmua_base + CC_MOH_OFF);
		core2 = readl(info->pmua_base + DM_CC_MOH_OFF);
		while (core2 & PMUA_DM_CC_MOH_SEA_RD_STATUS)
			core2 = readl(info->pmua_base + DM_CC_MOH_OFF);
		fccr = readl(info->pmum_base + FCCR_OFF);
		temp = (fccr & PMUM_FCCR_MOHCLKSEL_MSK) >> PMUM_FCCR_MOHCLKSEL_BASE;
		if (temp == 0x0)
			pll_ap = 312;
		else if (temp == 0x1)
			pll_ap = 624;
		else if (temp == 0x3)
			pll_ap = 26;
		temp = (fccr & PMUM_FCCR_AXICLKSEL_MSK) >> PMUM_FCCR_AXICLKSEL_BASE;
		if (temp == 0x0)
			pll_da = 312;
		else if (temp == 0x1)
			pll_da = 624;
		else if (temp == 0x3)
			pll_da = 26;
		fv_info->pclk = pll_ap/(((core2 & PMUA_DM_CC_MOH_CORE_CLK_DIV_MSK)
					>> PMUA_DM_CC_MOH_CORE_CLK_DIV_BASE) + 1);
		fv_info->pdclk = pll_ap/(((core2 & PMUA_DM_CC_MOH_BUS_MC_CLK_DIV_MSK)
					>> PMUA_DM_CC_MOH_BUS_MC_CLK_DIV_BASE) + 1);
		fv_info->baclk = pll_ap/(((core2 & PMUA_DM_CC_MOH_BIU_CLK_DIV_MSK)
					>> PMUA_DM_CC_MOH_BIU_CLK_DIV_BASE) + 1);
		fv_info->xpclk = pll_ap/(((core2 & PMUA_DM_CC_MOH_XP_CLK_DIV_MSK)
					>> PMUA_DM_CC_MOH_XP_CLK_DIV_BASE) + 1);
		fv_info->dclk = pll_da/(((core2 & PMUA_DM_CC_MOH_DDR_CLK_DIV_MSK)
					>> PMUA_DM_CC_MOH_DDR_CLK_DIV_BASE) + 1);
		fv_info->dclk /= 2;
		fv_info->aclk = pll_da/(((core2 & PMUA_DM_CC_MOH_BUS_CLK_DIV_MSK)
					>> PMUA_DM_CC_MOH_BUS_CLK_DIV_BASE) + 1);
		pxa3xx_pmic_get_voltage(VCC_CORE, &fv_info->vcc_core);
		temp |= PMUA_CC_MOH_MOH_RD_ST_CLEAR;
		writel(temp, info->pmua_base + CC_MOH_OFF);
		return 0;
	}
	return res;
}

static int get_op_num(void *driver_data, struct info_head *op_table)
{
	struct list_head *entry = NULL;
	int num = 0;

	if (!op_table)
		goto out;
	read_lock(&op_table->lock);
	if (list_empty(&op_table->list)) {
		read_unlock(&op_table->lock);
		goto out;
	}
	list_for_each(entry, &op_table->list) {
		num++;
	}
	read_unlock(&op_table->lock);
out:
	return num;
}

static char *get_op_name(void *driver_data, struct op_info *p)
{
	struct pxa910_md_opt *q = NULL;
	if (p == NULL)
		return NULL;
	q = (struct pxa910_md_opt *)p->op;
	return q->name;
}

static void update_voltage(void *driver_data, struct pxa910_md_opt *old,
	struct pxa910_md_opt *new)
{
	pxa3xx_pmic_set_voltage(VCC_CORE, new->vcc_core);
}

static void PMUcore2_fc_seq(void *driver_data, uint32_t cp_pdiv, uint32_t cp_bdiv,
	uint32_t cp_mdiv, uint32_t cp_xpdiv, uint32_t d_div, uint32_t a_div)
{
	struct pxa910_dvfm_info *info = driver_data;
	uint32_t cc_reg = 0, temp;
	uint32_t fccr, rdata_core1, rdata_core2;
	uint32_t pll_da = 0, pll_ap = 0, pll_cp = 0;

	temp = readl(info->pmua_base + CC_MOH_OFF);
	temp &= ~PMUA_CC_MOH_MOH_RD_ST_CLEAR;
	writel(temp, info->pmua_base + CC_MOH_OFF);

	/*Read dummy before sending the command*/
	rdata_core2 = readl(info->pmua_base + DM_CC_MOH_OFF);
	while (rdata_core2 & PMUA_DM_CC_MOH_SEA_RD_STATUS)
		rdata_core2 = readl(info->pmua_base + DM_CC_MOH_OFF);

	rdata_core1 = readl(info->pmua_base + DM_CC_SEA_OFF);

	if (cp_pdiv == 26)
		writel(0x61800000, info->pmum_base + FCCR_OFF);
	else
		writel(0x20800000, info->pmum_base + FCCR_OFF);

	fccr = readl(info->pmum_base + FCCR_OFF);

	temp = (fccr & PMUM_FCCR_SEAGCLKSEL_MSK) >> PMUM_FCCR_SEAGCLKSEL_BASE;
	if (temp == 0x0)
		pll_cp = 312;
	else if (temp == 0x1)
		pll_cp = 624;
	else if (temp == 0x3)
		pll_cp = 26;

	temp = (fccr & PMUM_FCCR_MOHCLKSEL_MSK) >> PMUM_FCCR_MOHCLKSEL_BASE;
	if (temp == 0x0)
		pll_ap = 312;
	else if (temp == 0x1)
		pll_ap = 624;
	else if (temp == 0x3)
		pll_ap = 26;

	temp = (fccr & PMUM_FCCR_AXICLKSEL_MSK) >> PMUM_FCCR_AXICLKSEL_BASE;
	if (temp == 0x0)
		pll_da = 312;
	else if (temp == 0x1)
		pll_da = 624;
	else if (temp == 0x3)
		pll_da = 26;

	/* pclk divider */
	if (cp_pdiv != 0)
		cc_reg |= (((pll_ap / cp_pdiv) - 1)
				<< PMUA_CC_MOH_CORE_CLK_DIV_BASE);

	/* pdclock divider */
	if (cp_mdiv != 0)
		cc_reg |= ((((pll_ap) / cp_bdiv) - 1)
				<< PMUA_CC_MOH_BIU_CLK_DIV_BASE);

	/* xp clock divider */
	if (cp_xpdiv != 0)
		cc_reg |= ((((pll_ap) / cp_xpdiv) - 1)
				<< PMUA_CC_MOH_XP_CLK_DIV_BASE);

	/* bus clock divider */
	if (cp_bdiv != 0)
		cc_reg |= ((((pll_ap) / cp_mdiv) - 1)
				<< PMUA_CC_MOH_BUS_MC_CLK_DIV_BASE);

	/* D clock divider */
	if (d_div != 0)
		cc_reg |= (((pll_da / d_div) - 1)
				<< PMUA_CC_MOH_DDR_CLK_DIV_BASE);

	/* A clock divider : fabric clock */
	if (a_div != 0)
		cc_reg |= (((pll_da / a_div) - 1)
				<< PMUA_CC_MOH_BUS_CLK_DIV_BASE);

#ifndef CONFIG_PXA910_DVFM_ASYNC_MODE
	{
	uint32_t temp_ap, temp_cp, temp_a, temp_d;
	/* Async 2 Check : pdclock2 - DDRClk */
	temp_ap = ((((cc_reg & PMUA_CC_MOH_BUS_MC_CLK_DIV_MSK)
				>> PMUA_CC_MOH_BUS_MC_CLK_DIV_BASE) + 1));
	temp_cp = (((cc_reg & PMUA_CC_MOH_DDR_CLK_DIV_MSK)
				>> PMUA_CC_MOH_DDR_CLK_DIV_BASE) + 1) ;
	/* ddr clk has an extra div by 2 */
	if ((pll_da/(temp_cp*2)) != (pll_ap/temp_ap))
		cc_reg |= PMUA_CC_MOH_ASYNC2;
	else
		cc_reg &= ~PMUA_CC_MOH_ASYNC2;

	/* Async 5 Check : baclk2-Aclk */
	temp_ap = ((((cc_reg & PMUA_CC_MOH_BIU_CLK_DIV_MSK)
			>>PMUA_CC_MOH_BIU_CLK_DIV_BASE) + 1));
	temp_cp = ((cc_reg & PMUA_CC_MOH_BUS_CLK_DIV_MSK)
			>> PMUA_CC_MOH_BUS_CLK_DIV_BASE) + 1;
	if ((pll_da/temp_cp) != (pll_ap/temp_ap))
		cc_reg |= PMUA_CC_MOH_ASYNC5;
	else
		cc_reg &= ~PMUA_CC_MOH_ASYNC5;

	/* async 3 Aclk - DDR Clock */
	temp_a = (((cc_reg & PMUA_CC_MOH_BUS_CLK_DIV_MSK)
			>>PMUA_CC_MOH_BUS_CLK_DIV_BASE) + 1);
	temp_d = (((cc_reg & PMUA_CC_MOH_DDR_CLK_DIV_MSK)
			>>PMUA_CC_MOH_DDR_CLK_DIV_BASE) + 1);
	if ((temp_d*2) != temp_a) {
		cc_reg |= PMUA_CC_MOH_ASYNC3;
		cc_reg |= PMUA_CC_MOH_ASYNC3_1;
	} else {
		cc_reg &= (~PMUA_CC_MOH_ASYNC3_1);
		cc_reg &= (~PMUA_CC_MOH_ASYNC3);
	}

	/* Async 1 Check : pdclock1 - DDRClk */
	temp_ap = ((((rdata_core1 & PMUA_CC_SEA_BUS_MC_CLK_DIV_MSK)
			>>PMUA_CC_SEA_BUS_MC_CLK_DIV_BASE) + 1));
	temp_cp = (((cc_reg & PMUA_CC_MOH_DDR_CLK_DIV_MSK)
			>>PMUA_CC_MOH_DDR_CLK_DIV_BASE) + 1);
	/* ddr clk has an extra div by 2 */
	if ((pll_da/(temp_cp*2)) != (pll_ap/temp_ap))
		cc_reg |= PMUA_CC_MOH_ASYNC1;
	else
		cc_reg &= ~PMUA_CC_MOH_ASYNC1;

	/* Async 4 Check : baclk1-Aclk */
	temp_ap = ((((rdata_core1 & PMUA_CC_SEA_BIU_CLK_DIV_MSK)
			>> PMUA_CC_SEA_BIU_CLK_DIV_BASE) + 1));
	temp_cp = ((cc_reg & PMUA_CC_SEA_BUS_CLK_DIV_MSK)
			>> PMUA_CC_SEA_BUS_CLK_DIV_BASE) + 1 ;
	if ((pll_da/temp_cp) != (pll_ap/temp_ap))
		cc_reg |= PMUA_CC_MOH_ASYNC4;
	else
		cc_reg &= ~PMUA_CC_MOH_ASYNC4;
	}
#else
	cc_reg |= (PMUA_CC_MOH_ASYNC4 | PMUA_CC_MOH_ASYNC1 |
		  PMUA_CC_MOH_ASYNC3 | PMUA_CC_MOH_ASYNC3_1 |
		  PMUA_CC_MOH_ASYNC5 | PMUA_CC_MOH_ASYNC2);
#endif

	writel(cc_reg, info->pmua_base + CC_MOH_OFF);
}

static void core2freqchgcmd(void *driver_data, int pclk, int dclk, int aclk)
{
	struct pxa910_dvfm_info *info = driver_data;
	volatile unsigned long freqchg,coremsk;

	coremsk = readl(info->pmua_base + MOH_IMR_OFF);
	coremsk |= (PMUA_MOH_IMR_MOH_FC_INTR_MASK);
	writel(coremsk, info->pmua_base + MOH_IMR_OFF);

	freqchg = readl(info->pmua_base + CC_MOH_OFF);

	freqchg &= ~(PMUA_CC_MOH_MOH_ALLOW_SPD_CHG |
			PMUA_CC_MOH_BUS_FREQ_CHG_REQ |
			PMUA_CC_MOH_DDR_FREQ_CHG_REQ |
			PMUA_CC_MOH_MOH_FREQ_CHG_REQ);

#ifdef CONFIG_PXA910_AP_ALONE_MODE
	writel(readl(info->pmua_base + CC_SEA_OFF) |
			PMUA_CC_SEA_SEA_ALLOW_SPD_CHG,
		info->pmua_base + CC_SEA_OFF);
#else
	if (!(readl(info->pmua_base + CC_SEA_OFF) &
			PMUA_CC_SEA_SEA_ALLOW_SPD_CHG))
		return;
#endif

	if ( pclk || dclk || aclk )
		freqchg |= (PMUA_CC_MOH_MOH_ALLOW_SPD_CHG);
	if ( aclk )
		freqchg |= (PMUA_CC_SEA_BUS_FREQ_CHG_REQ);
	if ( dclk )
		freqchg |= (PMUA_CC_SEA_DDR_FREQ_CHG_REQ);
	if ( pclk )
		freqchg |= (PMUA_CC_MOH_MOH_FREQ_CHG_REQ);

	writel(freqchg, info->pmua_base + CC_MOH_OFF);

	/* Check 4 the cmd 2 go thru */
	while (!(PMUA_MOH_ISR_MOH_FC_ISR &
			readl(info->pmua_base + MOH_ISR_OFF)))
		;

	/* Clear the PMU ISR */
	writel(0x0, info->pmua_base + MOH_ISR_OFF);

	freqchg |= PMUA_CC_MOH_MOH_RD_ST_CLEAR;

	/* Clear the bits */
	freqchg &= ~(	PMUA_CC_MOH_MOH_ALLOW_SPD_CHG |
			PMUA_CC_MOH_BUS_FREQ_CHG_REQ |
			PMUA_CC_MOH_DDR_FREQ_CHG_REQ |
			PMUA_CC_MOH_MOH_FREQ_CHG_REQ);

	/* clear the cmds bit */
	writel(freqchg, info->pmua_base + CC_MOH_OFF);
}

static int set_freq(void *driver_data, struct pxa910_md_opt *old,
		struct pxa910_md_opt *new)
{
	int core = 0, ddr = 0, bus = 0;

	if (new->pclk != old->pclk)
		core = 1;
	if (new->dclk != old->dclk)
		ddr = 1;
	if (new->aclk != old->aclk)
		bus = 1;
	if (!core && !ddr && !bus)
		return 0;
	if (new->vcc_core > old->vcc_core)
		update_voltage(driver_data, old, new);
	PMUcore2_fc_seq(driver_data, new->pclk, new->baclk, new->pdclk,
			new->xpclk, new->dclk * 2, new->aclk);
	core2freqchgcmd(driver_data, core, ddr, bus);
	if (new->vcc_core < old->vcc_core)
		update_voltage(driver_data, old, new);
	return 0;
}

static int update_freq(void *driver_data, struct dvfm_freqs *freqs)
{
	struct pxa910_dvfm_info *info = driver_data;
	struct pxa910_md_opt *old = NULL, *new = NULL;
	struct op_info *p = NULL;
	unsigned long flags;
	int found = 0, new_op = cur_op;

	write_lock_irqsave(&pxa910_dvfm_op_list.lock, flags);
	if (!list_empty(&pxa910_dvfm_op_list.list)) {
		list_for_each_entry(p, &pxa910_dvfm_op_list.list, list) {
			if (p->index == freqs->old) {
				found++;
				old = (struct pxa910_md_opt *)p->op;
			}
			if (p->index == freqs->new) {
				found++;
				new = (struct pxa910_md_opt *)p->op;
				new_op = p->index;
			}
			if (found == 2)
				break;
		}
	}
	write_unlock_irqrestore(&pxa910_dvfm_op_list.lock, flags);
	if (found != 2)
		return -EINVAL;

	set_freq(info, old, new);
	cur_op = new_op;
	loops_per_jiffy = new->lpj;
	return 0;
}

static void do_freq_notify(void *driver_data, struct dvfm_freqs *freqs)
{
	struct pxa910_dvfm_info *info = driver_data;

	dvfm_notifier_frequency(freqs, DVFM_FREQ_PRECHANGE);
	update_freq(info, freqs);
	dvfm_notifier_frequency(freqs, DVFM_FREQ_POSTCHANGE);
}

static void do_lowpower_notify(void *driver_data, struct dvfm_freqs *freqs,
	unsigned int state)
{
	dvfm_notifier_frequency(freqs, DVFM_FREQ_PRECHANGE);
	pxa910_pm_enter_lowpower_mode(state);
	dvfm_notifier_frequency(freqs, DVFM_FREQ_POSTCHANGE);
}

/* Check whether any client blocks the current operating point */
static int block_client(struct op_info *info)
{
	int i;
	unsigned int ret = 0;
	for (i = 0; i < (DVFM_MAX_CLIENT >> 5); i++)
		ret |= info->device[i];
	return (int)ret;
}

static int check_op(void *driver_data, struct dvfm_freqs *freqs,
		unsigned int new, unsigned int relation)
{
	struct op_info *p = NULL;
	int index, tmp_index = -1, found = 0;
	int first_op = 0;

	freqs->new = -1;
	if (!dvfm_find_op(new, &p)) {
		index = p->index;
	} else
		return -EINVAL;

	read_lock(&pxa910_dvfm_op_list.lock);
	if (relation == RELATION_LOW) {
		/* Set the lowest usable op that is higher than specifed one */
		/* Note: we assume bigger index number is more 'usable' */
		list_for_each_entry(p, &pxa910_dvfm_op_list.list, list) {
			if (!block_client(p) && (p->index >= index)) {
				if (tmp_index == -1 || (tmp_index >= p->index)) {
					if (first_op == 0)
						first_op = p->index;
					freqs->new = p->index;
					tmp_index = p->index;
					found = 1;
				}
				if (found && (new == p->index))
					break;
			}
		}
		if (found && (first_op == 1) && (new != p->index))
			freqs->new = first_op;
	} else if (relation == RELATION_HIGH) {
		/* Set the highest usable op that is lower than specified one */
		list_for_each_entry(p, &pxa910_dvfm_op_list.list, list) {
			if (!block_client(p) && (p->index <= index)) {
				if (tmp_index == -1 || tmp_index < p->index) {
					freqs->new = p->index;
					tmp_index = p->index;
				}
			}
		}
	} else if (relation == RELATION_STICK) {
		/* Set the specified frequency */
		list_for_each_entry(p, &pxa910_dvfm_op_list.list, list) {
			if (!block_client(p) && (p->index == new)) {
				freqs->new = p->index;
				break;
			}
		}
	}
	read_unlock(&pxa910_dvfm_op_list.lock);
	if (freqs->new == -1)
		return -EINVAL;
	return 0;
}

static int pxa910_set_op(void *driver_data, struct dvfm_freqs *freqs,
		unsigned int new, unsigned int relation)
{
	struct pxa910_dvfm_info *info = driver_data;
	struct pxa910_md_opt *md = NULL;
	struct op_info *p = NULL;
	unsigned long flags;
	int ret;

	local_fiq_disable();
	local_irq_save(flags);

	ret = dvfm_find_op(freqs->old, &p);
	if (ret)
		goto out;

	memcpy(&freqs->old_info, p, sizeof(struct op_info));
	ret = check_op(info, freqs, new, relation);
	if (ret)
		goto out;

	if (!dvfm_find_op(freqs->new, &p)) {
		memcpy(&(freqs->new_info), p, sizeof(struct op_info));
		/* If find old op and new op is same, skip it.
		 * At here, ret should be zero.
		 */
		if (freqs->old_info.index == freqs->new_info.index)
			goto out;
		md = (struct pxa910_md_opt *)p->op;
		switch (md->power_mode) {
		case POWER_MODE_ACTIVE:
			do_freq_notify(info, freqs);
			break;
		case POWER_MODE_CORE_INTIDLE:
		case POWER_MODE_CORE_EXTIDLE:
		case POWER_MODE_APPS_IDLE:
		case POWER_MODE_APPS_SLEEP:
		case POWER_MODE_SYS_SLEEP:
			do_lowpower_notify(info, freqs, md->power_mode);
			break;
		}
	}

	local_irq_restore(flags);
	local_fiq_enable();
	return 0;
out:
	local_irq_restore(flags);
	local_fiq_enable();
	return ret;
}

static int pxa910_request_op(void *driver_data, int index)
{
	struct dvfm_freqs freqs;
	struct op_info *info = NULL;
	struct pxa910_md_opt *md = NULL;
	int relation, ret;

	ret = dvfm_find_op(index, &info);
	if (ret)
		goto out;
	freqs.old = cur_op;
	freqs.new = index;
	md = (struct pxa910_md_opt *)(info->op);
	relation = RELATION_LOW;
	ret = pxa910_set_op(driver_data, &freqs, index, relation);
	if (!ret)
		preferred_op = index;
out:
	return ret;
}

/*
 * The machine operation of dvfm_enable
 */
static int pxa910_enable_dvfm(void *driver_data, int dev_id)
{
	struct pxa910_dvfm_info *info = driver_data;
	struct pxa910_md_opt *md = NULL;
	struct op_info *p = NULL;
	int i, num;
	num = get_op_num(info, &pxa910_dvfm_op_list);
	for (i = 0; i < num; i++) {
		if (!dvfm_find_op(i, &p)) {
			md = (struct pxa910_md_opt *)p->op;
			dvfm_enable_op(i, dev_id);
		}
	}
	return 0;
}

/*
 * The mach operation of dvfm_disable
 */
static int pxa910_disable_dvfm(void *driver_data, int dev_id)
{
	struct pxa910_dvfm_info *info = driver_data;
	struct pxa910_md_opt *md = NULL;
	struct op_info *p = NULL;
	int i, num;
	num = get_op_num(info, &pxa910_dvfm_op_list);
	for (i = 0; i < num; i++) {
		if (!dvfm_find_op(i, &p)) {
			md = (struct pxa910_md_opt *)p->op;
			dvfm_disable_op(i, dev_id);
		}
	}
	return 0;
}

static int pxa910_enable_op(void *driver_data, int index, int relation)
{
	/*
	 * Restore preferred_op. Because this op is sugguested by policy maker
	 * or user.
	 */
	return pxa910_request_op(driver_data, preferred_op);
}

static int pxa910_disable_op(void *driver_data, int index, int relation)
{
	struct dvfm_freqs freqs;
	if (cur_op == index) {
		freqs.old = index;
		freqs.new = -1;
		dvfm_set_op(&freqs, freqs.old, relation);
	}
	return 0;
}

static struct dvfm_driver pxa910_driver = {
	.count		= get_op_num,
	.set		= pxa910_set_op,
	.dump		= dump_op,
	.name		= get_op_name,
	.request_set	= pxa910_request_op,
	.enable_dvfm	= pxa910_enable_dvfm,
	.disable_dvfm	= pxa910_disable_dvfm,
	.enable_op	= pxa910_enable_op,
	.disable_op	= pxa910_disable_op,
};

/* Produce a operating point table */
static int op_init(struct pxa910_dvfm_info *driver_data, struct info_head *op_table)
{
	struct pxa910_dvfm_info *info = driver_data;
	struct pxa910_md_opt *md, *smd;
	unsigned long flags;
	int i, index;
	struct op_info *p = NULL, *q = NULL;
	struct proc_op_array *proc = NULL;

	proc = proc_op_arrays;
	printk("initializing op table for %s\n", proc->cpu_name);
	for (i = 0, index = 0; i < proc->nr_op; i++) {
		/* Set index of operating point used in idle */
		if (proc->op_array[i].power_mode != POWER_MODE_ACTIVE)
			set_idle_op(index, proc->op_array[i].power_mode);

		if (!(p = (struct op_info *)kzalloc(sizeof(struct op_info),
				GFP_KERNEL)))
			return -ENOMEM;
		if (!(p->op = (struct pxa910_md_opt *)kzalloc(sizeof(struct pxa910_md_opt),
				GFP_KERNEL)))
			return -ENOMEM;
		memcpy(p->op, &proc->op_array[i], sizeof(struct pxa910_md_opt));
		p->index = index++;
		list_add_tail(&(p->list), &(op_table->list));
	}

	if (!(p = (struct op_info *)kzalloc(sizeof(struct op_info),
				GFP_KERNEL)))
			return -ENOMEM;
	if (!(p->op = (struct pxa910_md_opt *)kzalloc(sizeof(struct pxa910_md_opt),
				GFP_KERNEL)))
			return -ENOMEM;
	md = (struct pxa910_md_opt *)p->op;
	if (capture_op_info(info, md)) {
		printk(KERN_WARNING "Failed to get current op setting\n");
	} else {
		def_op = 0x5a5a;	/* magic number */
		list_for_each_entry(q, &(op_table->list), list) {
			smd = (struct pxa910_md_opt *)q->op;
			if (md->pclk == smd->pclk && md->pdclk == smd->pdclk &&
			    md->baclk == smd->baclk && md->xpclk == smd->xpclk &&
			    md->dclk == smd->dclk && md->aclk == smd->aclk) {
				def_op = q->index;
				break;
			}
		}
	}
	md->power_mode = POWER_MODE_ACTIVE;
	md->lpj = loops_per_jiffy;
	sprintf(md->name, "BOOT OP");

	if (!(q = (struct op_info *)kzalloc(sizeof(struct op_info),
				GFP_KERNEL)))
			return -ENOMEM;
	if (!(q->op = (struct pxa910_md_opt *)kzalloc(sizeof(struct pxa910_md_opt),
				GFP_KERNEL)))
			return -ENOMEM;
	smd = (struct pxa910_md_opt *)q->op;
	memcpy(smd, md, sizeof(struct pxa910_md_opt));
	sprintf(smd->name, "CUSTOM OP");

	/* Add CUSTOM OP into op list */
	q->index = index++;
	list_add_tail(&q->list, &op_table->list);
	/* Add BOOT OP into op list */
	p->index = index++;
	preferred_op = p->index;
	list_add_tail(&p->list, &op_table->list);
	/* BOOT op */
	if (def_op == 0x5a5a) {
		cur_op = p->index;
		def_op = p->index;
	} else
		cur_op = def_op;
	pr_debug("%s, def_op:%d, cur_op:%d\n", __FUNCTION__, def_op, cur_op);

	op_nums = proc->nr_op + 2;	/* set the operating point number */

	printk("Current Operating Point is %d\n", cur_op);
	dump_op_list(info, op_table);

	return 0;
}

#ifdef CONFIG_PM
static int pxa910_freq_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int pxa910_freq_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define pxa910_freq_suspend    NULL
#define pxa910_freq_resume     NULL
#endif

static int pxa910_freq_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct pxa910_dvfm_info *info;

	if (!(info = kzalloc(sizeof(struct pxa910_dvfm_info), GFP_KERNEL)))
		goto err;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "pmum_regs");
	if (!res) goto err;
	info->pmum_base = ioremap(res->start, res->end - res->start + 1);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "pmua_regs");
	if (!res) goto err;
	info->pmua_base = ioremap(res->start, res->end - res->start + 1);

	pxa910_driver.priv = info;

	if (op_init(info, &pxa910_dvfm_op_list))
		goto err;

	if (dvfm_register("PXA910-DVFM", &dvfm_dev_id))
		goto err;
	return dvfm_register_driver(&pxa910_driver, &pxa910_dvfm_op_list);
err:
	printk("pxa910_dvfm init failed\n");
	return -EIO;
}

static int pxa910_freq_remove(struct platform_device *pdev)
{
	kfree(pxa910_driver.priv);
	dvfm_unregister("PXA910-DVFM", &dvfm_dev_id);
	return dvfm_unregister_driver(&pxa910_driver);
}

static struct platform_driver pxa910_freq_driver = {
	.driver = {
		.name	= "pxa168-freq",
	},
	.probe		= pxa910_freq_probe,
	.remove		= pxa910_freq_remove,
#ifdef CONFIG_PM
	.suspend	= pxa910_freq_suspend,
	.resume		= pxa910_freq_resume,
#endif
};

static int __init pxa910_freq_init(void)
{
	if (!cpu_is_pxa910())
		return -EIO;

	return platform_driver_register(&pxa910_freq_driver);
}

static void __exit pxa910_freq_exit(void)
{
	platform_driver_unregister(&pxa910_freq_driver);
}

module_init(pxa910_freq_init);
module_exit(pxa910_freq_exit);
