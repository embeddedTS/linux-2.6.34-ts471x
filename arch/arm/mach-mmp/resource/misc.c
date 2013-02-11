#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/smc91x.h>
#include <linux/delay.h>
#include <linux/pda_power.h>
#include <linux/switch.h>
#include <linux/proc_fs.h>
#include <linux/sensor-input.h>

#include <asm/uaccess.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/addr-map.h>
#include <mach/mfp-pxa910.h>
#include <mach/pxa168.h>
#include <mach/pxa910.h>
#include <mach/gpio.h>
#include <mach/sanremo.h>
#include <mach/portofino.h>
#include <mach/resource.h>

#include <plat/part_table.h>
#include <plat/generic.h>
#include "../common.h"

void res_add_sanremo_vibrator(void)
{
#if defined(CONFIG_SANREMO)
	static struct vibrator_data vibrator_device_data = {
		.max_v = 5,
		.min_v = 0,
		.set_vibrator = sanremo_set_vibrator,
	};
	static struct platform_device vibrator_device = {
		.name            = "vibrator",
		.id              = 0,
		.dev             = {
			.platform_data = &vibrator_device_data,
		},
	};

	platform_device_register(&vibrator_device);
#endif
}

