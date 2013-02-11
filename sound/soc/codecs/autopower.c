#include <linux/version.h>
#include <linux/module.h>

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/proc_fs.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>

#include <linux/semaphore.h>
#include <mach/irqs.h>
#include <asm/system.h>
#include <linux/uaccess.h>
#include <mach/dma.h>
#include <asm/atomic.h>


#include <linux/version.h>

#include <linux/clk.h>

#include <mach/hardware.h>
#include <linux/jiffies.h>

#include <linux/time.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/spinlock.h>

#include "autopower.h"
#include "../codecs/wm8960.h"

#define TIMER_INTERVAL (5*HZ)
struct timer_list audio_timer;
int audio_suspend_status;
int audio_suspend_enable;
static int busy;

void timerfunc()
{
	if (audio_suspend_enable == 1) {
		if (audio_suspend_status == 0) {
			wm8960_powerdown(0);
			audio_suspend_status = 1;
		}
	}
}

void autopower_init()
{
	busy = 0;
	init_timer(&audio_timer);
	audio_timer.data = 0;
	audio_timer.expires = jiffies + TIMER_INTERVAL;
	audio_timer.function = timerfunc;
	add_timer(&audio_timer);
	audio_suspend_status = 0;
}

void autopower_enable()
{
	if (busy == 0) {
		del_timer(&audio_timer);
		if (audio_suspend_status == 1) {
			wm8960_powerup();
			audio_suspend_status = 0;
		}
		audio_suspend_enable = 0;
	}
	busy++;
}


void autopower_disable()
{
	busy--;
	if (busy == 0) {
		del_timer(&audio_timer);
		audio_timer.expires = jiffies + TIMER_INTERVAL;
		audio_timer.function = timerfunc;
		add_timer(&audio_timer);
		audio_suspend_enable = 1;
	}
}
