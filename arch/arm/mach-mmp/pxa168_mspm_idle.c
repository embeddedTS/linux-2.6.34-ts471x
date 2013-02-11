/*
 * PXA168 MSPM IDLE
 *
 * Copyright (c) 2003 Intel Corporation.
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 *
 * (C) Copyright 2008 Marvell International Ltd.
 * All Rights Reserved
 */

#undef DEBUG
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>

#include <asm/proc-fns.h>
#include <asm/ptrace.h>
#include <asm/mach/time.h>

#include <mach/cputype.h>
#include <mach/hardware.h>
#include <mach/dvfm.h>
#include <mach/mspm_prof.h>
#include <mach/pxa168_pm.h>
#include <mach/pxa168_dvfm.h>
#include <mach/regs-timers.h>

static int core_extidle_opidx = -1;
static int sys_sleep_opidx = -1;

#define LOWPOWER_CORE_EXTIDLE_THRE 0
#define LOWPOWER_SYS_SLEEP_THRE 1500
#define IDLE_STATS_NUM 1000

struct idle_stats {
	unsigned int	index;
	unsigned int	ticks;
};

struct mspm_idle_stats {
	struct idle_stats	stats[IDLE_STATS_NUM];
	unsigned int		stats_index;
	spinlock_t		lock;
};

static struct mspm_idle_stats mspm_stats = {
	.stats_index	= 0,
	.lock		= SPIN_LOCK_UNLOCKED,
};

static void (*orig_idle)(void);

/* static void mspm_cpu_idle(void)
{
	struct op_info *info = NULL;
	int op;

	op = dvfm_get_op(&info);
	mspm_add_event(op, CPU_STATE_RUN);
	cpu_do_idle();
	mspm_add_event(op, CPU_STATE_IDLE);
}*/

/* Collect statistic information before entering idle */
static void record_idle_stats(void)
{
	int i;

	spin_lock(&mspm_stats.lock);
	if (++mspm_stats.stats_index == IDLE_STATS_NUM)
		mspm_stats.stats_index = 0;
	i = mspm_stats.stats_index;
	memset(&mspm_stats.stats[i], 0, sizeof(struct idle_stats));
	mspm_stats.stats[i].index = i;
	mspm_stats.stats[i].ticks = read_timer();
	spin_unlock(&mspm_stats.lock);
}

/* Collect statistic information after exiting idle.
 */
static void update_idle_stats(void)
{
	int i;

	spin_lock(&mspm_stats.lock);
	i = mspm_stats.stats_index;
	mspm_stats.stats[i].ticks = read_timer()
		- mspm_stats.stats[i].ticks;
	spin_unlock(&mspm_stats.lock);
}

void set_idle_op(int idx, int mode)
{
	switch (mode) {
	case POWER_MODE_CORE_EXTIDLE:
		core_extidle_opidx = idx;
		break;
	case POWER_MODE_SYS_SLEEP:
		sys_sleep_opidx = idx;
		break;
	}
}


static int lpidle_is_valid(int enable, unsigned int msec,
			struct dvfm_freqs *freqs, int lp_idle)
{
	struct op_info *info = NULL;
	struct pxa168_md_opt *op;
	int prev_op;
	int ret;
	int threshold = 0;
	if ((freqs == NULL)
		|| (lp_idle == IDLE_CORE_EXTIDLE && core_extidle_opidx == -1)
		|| (lp_idle == IDLE_SYS_SLEEP && sys_sleep_opidx == -1))
		return 0;

	if (enable & lp_idle) {
		switch (lp_idle) {
		case IDLE_CORE_EXTIDLE:
			threshold = LOWPOWER_CORE_EXTIDLE_THRE;
			break;
		case IDLE_SYS_SLEEP:
			threshold = LOWPOWER_SYS_SLEEP_THRE;
			break;
		default:
			BUG();
			break;
		}

		/* Check dynamic tick flag && idle interval */
		if (msec < threshold)
			return 0;
		/* Check whether the specified low power mode is valid */
		ret = dvfm_get_op(&info);
		if (info == NULL)
			return 0;
		op = (struct pxa168_md_opt *)info->op;
		if (op == NULL)
			return 0;
		if ((ret >= 0) && (op->power_mode == POWER_MODE_ACTIVE)) {
			prev_op = ret;
			freqs->old = ret;
			freqs->new =
				lp_idle == IDLE_CORE_EXTIDLE ?
					core_extidle_opidx :
				lp_idle == IDLE_SYS_SLEEP ?
					sys_sleep_opidx : -1;
			if (freqs->new < 0)
				return 0;
			ret = dvfm_get_opinfo(freqs->new, &info);
			if ((ret >= 0) && find_first_bit(info->device, \
					DVFM_MAX_CLIENT) == DVFM_MAX_CLIENT) {
				return 1;
			}
		}
	}
	return 0;
}

/*
 * IDLE Thread
 */

unsigned int get_remain_slice(void)
{
	uint32_t val1 = 0;
	uint32_t val2 = 0;

	val1 = __raw_readl(TIMERS1_VIRT_BASE + TMR_TN_MM(0, 0));
	val2 = read_timer();
	return (val1-val2)/3250;
}


void mspm_do_idle(void)
{
	struct dvfm_freqs freqs;
	int ret = -EINVAL;
	unsigned int msec;
#ifdef CONFIG_MSPM_PXA168_STATS
	struct op_info *info = NULL;
	int op;
#endif
	local_irq_disable();
	if (!need_resched()) {
		msec = get_remain_slice();
		record_idle_stats();

		if (cpu_is_pxa168_A0() == 0) {
			if (lpidle_is_valid(enable_deepidle, msec, &freqs, \
				IDLE_SYS_SLEEP)) {
				ret = dvfm_set_op(&freqs, freqs.new,
					RELATION_STICK);
				if (ret == 0)
					goto out;
			}
		}
		if (enable_deepidle & IDLE_CORE_EXTIDLE) {
#ifdef CONFIG_MSPM_PXA168_STATS
			op = dvfm_get_op(&info);
			dvfm_add_event(op, CPU_STATE_RUN, op, CPU_STATE_IDLE);
			dvfm_add_timeslot(op, CPU_STATE_RUN);
#endif

			pxa168_pm_enter_lowpower_mode(POWER_MODE_CORE_EXTIDLE);

#ifdef CONFIG_MSPM_PXA168_STATS
			dvfm_add_event(op, CPU_STATE_IDLE, op, CPU_STATE_RUN);
			dvfm_add_timeslot(op, CPU_STATE_IDLE);
#endif
		}

out:
		update_idle_stats();
	}
	local_irq_enable();
}

static struct proc_dir_entry *entry_dir;
static struct proc_dir_entry *entry_stats;

static int stats_show(struct seq_file *s, void *v)
{
	struct idle_stats *p = NULL;
	int i, ui;
	unsigned long flags;

	spin_lock_irqsave(&mspm_stats.lock, flags);
	ui = mspm_stats.stats_index;
	for (i = 0; i < IDLE_STATS_NUM; i++) {
		p = &mspm_stats.stats[ui++];
		seq_printf(s, "index:%u idle_ticks:%u\n",
				p->index, p->ticks);
		if (ui == IDLE_STATS_NUM)
			ui = 0;
	}
	spin_unlock_irqrestore(&mspm_stats.lock, flags);
	return 0;
}

static int stats_seq_open(struct inode *inode, struct file *file)
{
	return single_open(file, &stats_show, NULL);
}

static struct file_operations stats_seq_ops = {
	.owner		= THIS_MODULE,
	.open		= stats_seq_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int mspm_proc_init(void)
{
	entry_dir = proc_mkdir("driver/mspm", NULL);
	if (entry_dir == NULL)
		return -ENOMEM;

	entry_stats = create_proc_entry("stats", 0, entry_dir);
	if (entry_stats)
		entry_stats->proc_fops = &stats_seq_ops;
	return 0;
}

static void mspm_proc_cleanup(void)
{
	remove_proc_entry("stats", entry_dir);
	remove_proc_entry("driver/mspm", NULL);
}

void mspm_idle_load(void)
{
	if (pm_idle != mspm_do_idle) {
		orig_idle = pm_idle;
		pm_idle = mspm_do_idle;
	}
}

void mspm_idle_clean(void)
{
	if (pm_idle == mspm_do_idle)
		pm_idle = orig_idle;
}

static int __init mspm_init(void)
{
	/*if (!cpu_is_pxa910())
		return -EFAULT;*/

	/* Create file in procfs */
	if (mspm_proc_init())
		return -EFAULT;

	mspm_prof_init();

	pr_info("Initialize PXA910 MSPM\n");

	return 0;
}

static void __exit mspm_exit(void)
{
	/* Remove procfs */
	mspm_proc_cleanup();

	mspm_prof_exit();

	pr_info("Quit PXA910 MSPM\n");
}

module_init(mspm_init);
module_exit(mspm_exit);

MODULE_DESCRIPTION("PXA910_MSPM");
MODULE_LICENSE("GPL");
