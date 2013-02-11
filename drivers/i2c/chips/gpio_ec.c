#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/string.h>
#include <mach/power_mcu.h>
#include <linux/delay.h>
#include <mach/gpio.h>
#include <mach/mfp.h>
#include <mach/mfp-pxa168.h>
#include <plat/mfp.h>

static DEFINE_MUTEX(gpio_ec_lock);

int set_bus_busy(void)
{
	if(!mutex_trylock(&gpio_ec_lock)) {
		/* try lock reutrns 1 if the mutex is acquired successfully, 
		 * and 0 on contention.
		 */
		return -1;
	}
	gpio_direction_output(MFP_PIN_GPIO88, 1);

	if ((gpio_get_value(MFP_PIN_GPIO89) >> 25) == 0) {
		return 0;
	}

	gpio_direction_output(MFP_PIN_GPIO88, 0);
	mutex_unlock(&gpio_ec_lock);
	return -1;

}

EXPORT_SYMBOL(set_bus_busy);

int set_bus_free(void)
{
	gpio_direction_output(MFP_PIN_GPIO88, 0);
	mutex_unlock(&gpio_ec_lock);

	return 0;

}

EXPORT_SYMBOL(set_bus_free);


