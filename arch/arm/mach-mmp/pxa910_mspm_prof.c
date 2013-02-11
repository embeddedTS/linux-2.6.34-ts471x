/*
 * PXA910 MSPM Profiler
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 *
 * (C) Copyright 2008 Marvell International Ltd.
 * All Rights Reserved
 */

/*
 * Behavior of profiler
 *
 * When sample window is finished, profiler calculates the mips in sample
 * window. System will be adjusted to suitable OP.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/tick.h>
#include <linux/timer.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>

#include <mach/hardware.h>
#include <mach/dvfm.h>
#include <mach/pxa168_pm.h>
#include <mach/pxa910_pm.h>
#include <mach/pxa910_dvfm.h>
#include <mach/mspm_prof.h>

struct mspm_op_stats {
	int		op;
	int		idle;
	unsigned int	timestamp;
	unsigned int 	jiffies;
};

struct mspm_mips {
	int	mips;
	int	h_thres;	/* high threshold */
	int	l_thres;	/* low threshold */
};

enum {
	DISABLE = 0,
	ENABLE,
};


static int mspm_ctrl = DISABLE, mspm_prof = DISABLE;

/* Store OP's MIPS in op_mips[]. The lowest frequency OP is the first entry */
static struct mspm_mips op_mips[MAX_OP_NUM];

/* Store duration time of all OP in op_duration[] */
static int op_duration[MAX_OP_NUM];

/* Store costed time in run_op_time[] & idle_op_time[] */
static int run_op_time[MAX_OP_NUM], idle_op_time[MAX_OP_NUM];

/*
 * Store the first timestamp of sample window in first_stats
 * Store the current timestamp of sample window in cur_stats
 */
static struct mspm_op_stats first_stats, cur_stats;

/* OP numbers used in IPM IDLE Profiler */
static int mspm_op_num;

static struct timer_list idle_prof_timer;
static int mspm_window = DEF_SAMPLE_WINDOW;
static int window_jif;

/* DVFM notifier and index */
static int mspm_prof_notifier_freq(struct notifier_block *nb,
				unsigned long val, void *data);
static struct notifier_block notifier_freq_block = {
	.notifier_call = mspm_prof_notifier_freq,
};

static int dvfm_dev_idx;

/*
 * Adjust to the most appropriate OP according to MIPS result of
 * sample window
 */
static int mspm_request_tune(int mips)
{
	int i;

	for (i = mspm_op_num - 1; i >= 0; i--) {
		if (mips >= (op_mips[i].l_thres *
			op_mips[i].mips / 100))
			break;
	}
	dvfm_request_op(i);
	return 0;
}

/*
 * Calculate the MIPS in sample window
 */
static int mspm_calc_mips(unsigned int first_time)
{
	int i, mips, curop;
	unsigned int time, sum_time = 0, sum = 0;
	struct op_info *info = NULL;

	curop = dvfm_get_op(&info);

	/* Store the last slot as RUN state */
	time = read_timer();
	run_op_time[cur_stats.op] += time - cur_stats.timestamp;
	cur_stats.timestamp = time;
	cur_stats.jiffies = jiffies;
	cur_stats.op = curop;
	cur_stats.idle = CPU_STATE_RUN;
	/* Calculate total time costed in sample window */
	for (i = 0; i < mspm_op_num; i++) {
		sum_time += run_op_time[i] + idle_op_time[i];
		sum += run_op_time[i] * op_mips[i].mips;
		op_duration[i] = run_op_time[i] + idle_op_time[i];
	}
	if (sum_time == 0) {
		/* CPU usage is 100% in current operating point */
		sum_time = time - first_time;
		sum = sum_time * op_mips[curop].mips;
		op_duration[curop] = sum_time;
	}

	/*
	 * Calculate MIPS in sample window
	 * Formula: run_op_time[i] / sum_time * op_mips[i].mips
	 */
	mips = sum / sum_time;
	return mspm_request_tune(mips);
}

/*
 * Record the OP index and RUN/IDLE state.
 */
int mspm_add_event(int op, int cpu_idle)
{
	unsigned int time;

	if (mspm_prof == ENABLE) {
		time = read_timer();
		/* sum the current sample window */
		if (cpu_idle == CPU_STATE_IDLE)
			idle_op_time[cur_stats.op] += time - \
					cur_stats.timestamp;
		else if (cpu_idle == CPU_STATE_RUN)
			run_op_time[cur_stats.op] += time - cur_stats.timestamp;
		/* update start point of current sample window */
		cur_stats.op = op;
		cur_stats.idle = cpu_idle;
		cur_stats.timestamp = time;
		cur_stats.jiffies = jiffies;
	}
	return 0;
}
EXPORT_SYMBOL(mspm_add_event);

/*
 * Prepare to do a new sample.
 * Clear the index in mspm_op_stats table.
 */
static int mspm_do_new_sample(void)
{
	/* clear previous sample window */
	memset(&run_op_time, 0, sizeof(int) * MAX_OP_NUM);
	memset(&idle_op_time, 0, sizeof(int) * MAX_OP_NUM);
	/* prepare for the new sample window */
	first_stats.op = cur_stats.op;
	first_stats.idle = cur_stats.idle;
	first_stats.timestamp = read_timer();
	first_stats.jiffies = jiffies;
	return 0;
}

/***************************************************************************
 * 			Idle Profiler
 ***************************************************************************/

static int launch_prof(int enable)
{
	struct op_info *info = NULL;

	/* early return if mspm is disabled */
	if (mspm_ctrl == DISABLE)
		return 0;

	if (enable) {
		window_jif = msecs_to_jiffies(mspm_window);
		/* start next sample window */
		cur_stats.op = dvfm_get_op(&info);
		cur_stats.idle = CPU_STATE_RUN;
		cur_stats.timestamp = read_timer();
		cur_stats.jiffies = jiffies;
		mspm_do_new_sample();
		mod_timer(&idle_prof_timer, jiffies + window_jif);
	} else {
		del_timer(&idle_prof_timer);
	}
	mspm_prof = enable;

	return 0;
}

/*
 * Handler of IDLE PROFILER
 */
static void idle_prof_handler(unsigned long data)
{
	mspm_calc_mips(first_stats.timestamp);
	/* start next sample window */
	mspm_do_new_sample();
	mod_timer(&idle_prof_timer, jiffies + window_jif);
}

/*
 * Pause idle profiler when system enter Low Power mode.
 * Continue it when system exit from Low Power mode.
 */
static int mspm_prof_notifier_freq(struct notifier_block *nb,
				unsigned long val, void *data)
{
	/* will implement this when low power mode is enabled
	 * for pxa910, right now just return 0
	 */
	return 0;
}

/* It's invoked by initialization code & sysfs interface */
static int launch_mspm(int enable)
{
	if (enable) {
		/* enable mspm control */
		mspm_idle_load();
		if (mspm_prof == ENABLE)
			printk(KERN_ALERT "prof is enabled before mspm\n");
		/* disable unused OP in MSPM */
		dvfm_disable_op_name("CUSTOM OP", dvfm_dev_idx);
		dvfm_disable_op_name("BOOT OP", dvfm_dev_idx);
		dvfm_disable_op_name("core_intidle", dvfm_dev_idx);
	} else {
		/* disable mspm control */
		/* check if profiler is launched. If it is, disable it */
		if (mspm_prof == ENABLE)
			launch_prof(DISABLE);
		mspm_idle_clean();
		/* enable unused OP in MSPM */
		dvfm_enable_op_name("CUSTOM OP", dvfm_dev_idx);
		dvfm_enable_op_name("BOOT OP", dvfm_dev_idx);
		dvfm_enable_op_name("core_intidle", dvfm_dev_idx);
	}
	mspm_ctrl = enable;
	return 0;
}

/************************************************************************
 *	 		sysfs interface					*
 ************************************************************************/

#define mspm_attr(_name)				\
static struct kobj_attribute _name##_attr = {		\
	.attr	= {					\
		.name = __stringify(_name),		\
		.mode = 0644,				\
	},						\
	.show	= _name##_show,				\
	.store	= _name##_store,			\
}

/*
 * Show whether MSPM is enabled
 */
static ssize_t mspm_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%u\n", mspm_ctrl);
}

/*
 * Configure MSPM
 * When MSPM is enabled, mspm idle is loaded.
 */
static ssize_t mspm_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t len)
{
	int data;
	sscanf(buf, "%u", &data);
	if (data == ENABLE || data == DISABLE)
		if (data != mspm_ctrl)
			launch_mspm(data);
	return len;
}
mspm_attr(mspm);

/*
 * Show whether MSPM profiler in kernel space is enabled
 */
static ssize_t prof_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%u\n", mspm_prof);
}

/*
 * Configure the MSPM profiler in kernel space
 * This interface can't control deepidle
 */
static ssize_t prof_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t len)
{
	int data;
	sscanf(buf, "%u", &data);
	if (data == ENABLE || data == DISABLE)
		if (data != mspm_prof)
			launch_prof(data);
	return len;
}
mspm_attr(prof);

/* Show the length of sample window */
static ssize_t window_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%ums\n", mspm_window);
}

static ssize_t window_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t len)
{
	sscanf(buf, "%u", &mspm_window);
	return len;
}
mspm_attr(window);

static struct attribute *g[] = {
	&mspm_attr.attr,
	&prof_attr.attr,
	&window_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.name	= "mspm",
	.attrs	= g,
};

/*
 * Init MIPS of all OP
 * Return OP numbers
 */
int __init mspm_init_mips(void)
{
	struct op_info *info = NULL;
	struct pxa910_md_opt *md_op = NULL;
	int i, ret;
	memset(&op_mips, 0, MAX_OP_NUM * sizeof(struct mspm_mips));
	mspm_op_num = dvfm_op_count();
	for (i = 0; i < mspm_op_num; i++) {
		ret = dvfm_get_opinfo(i, &info);
		if (ret)
			continue;
		md_op = (struct pxa910_md_opt *)info->op;
		op_mips[i].mips = md_op->pclk;
		if (op_mips[i].mips) {
			op_mips[i].h_thres = DEF_HIGH_THRESHOLD;
		} else {
			mspm_op_num = i;
			break;
		}
	}
	for (i = 0; i < mspm_op_num - 1; i++)
		op_mips[i + 1].l_thres = op_mips[i].h_thres * op_mips[i].mips
				/ op_mips[i + 1].mips;
	return mspm_op_num;
}

int __init mspm_prof_init(void)
{
	if (sysfs_create_group(power_kobj, &attr_group))
		return -EFAULT;

	/* It's used to trigger sample window.
	 * If system is idle, the timer could be deferred.
	 */
	init_timer_deferrable(&idle_prof_timer);
	idle_prof_timer.function = idle_prof_handler;
	idle_prof_timer.data = 0;

	mspm_op_num = mspm_init_mips();

	dvfm_register_notifier(&notifier_freq_block,
				DVFM_FREQUENCY_NOTIFIER);
	dvfm_register("MSPM PROF", &dvfm_dev_idx);

	/* turn on mspm and turn off prof by default */
	launch_mspm(ENABLE);
	launch_prof(DISABLE);

	return 0;
}

void __exit mspm_prof_exit(void)
{
	dvfm_unregister("MSPM PROF", &dvfm_dev_idx);
	dvfm_unregister_notifier(&notifier_freq_block,
				DVFM_FREQUENCY_NOTIFIER);
}
