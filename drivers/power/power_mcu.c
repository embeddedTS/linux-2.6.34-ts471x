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
#include <mach/gpio_ec.h>
#include <linux/slab.h>

static struct i2c_client *g_client;
static struct class *power_mcu_class;

static ssize_t led_blink_show_status(struct device *dev,
		struct device_attribute *attr, char *buf) {
	return sprintf(buf, "%d\n", 0);
}

static ssize_t reset_show_status(struct device *dev,
		struct device_attribute *attr, char *buf) {
	return sprintf(buf, "%d\n", 0);
}

static ssize_t vcore_show_status(struct device *dev,
		struct device_attribute *attr, char *buf) {
	unsigned short vcore, adj;
	unsigned char flag;
	power_mcu_read(MCU_VCORE_VOLT, &vcore);
	power_mcu_read(MCU_VCORE_ADJ, &adj);
	power_mcu_read(MCU_VCORE_FLAG, &flag);
	return sprintf(buf, "level %d vcore 0x%x flag 0x%x\n", adj, vcore, flag);
}

static ssize_t ecversion_show_status(struct device *dev,
		struct device_attribute *attr, char *buf) {
	unsigned char version;
	power_mcu_read(MCU_VERSION, &version);
	return sprintf(buf, "%d\n", version);
}

static ssize_t led_blink_store_status(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count) {
	power_mcu_write_byte(MCU_LED_BLINK, 1);
	return 0;
}

static ssize_t reset_store_status(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count) {
	return 0;
}

static ssize_t vcore_store_status(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count) {
	unsigned char flag = 0;
	unsigned long vcore;
	int ret;

	strict_strtoul(buf, 0, &vcore);
	if (vcore > 3) {
		printk(KERN_WARNING "vcore is invalid %d\n", vcore);
		return 0;
	}

	/* vcore:
	 * 0	---- 1.0373V
	 * 1	---- 1.1049V
	 * 2	---- 1.1788V
	 * 3	---- 1.2457V
	 */
	power_mcu_write_byte(MCU_VCORE_ADJ, vcore);

	msleep(100);
	power_mcu_read(MCU_VCORE_FLAG, &flag);
	if (flag == 0)
		printk(KERN_WARNING "Vcore adjust failed\n");
	return count;
}

static struct device_attribute power_mcu_attributes[] = {
	__ATTR(led_blink, 0644, led_blink_show_status, led_blink_store_status),
	__ATTR(reset, 0644, reset_show_status, reset_store_status),
	__ATTR(vcore, 0644, vcore_show_status, vcore_store_status),
	__ATTR(ecversion, 0644, ecversion_show_status, NULL),
	__ATTR_NULL,
};

int power_mcu_read(unsigned char reg, void *pval)
{
	int ret, status = -EIO;

	if (g_client == NULL)
		return -EFAULT;

	if ((reg < MCU_OFFSET_WORD) && (reg >= MCU_OFFSET_BYTE)) {
		/*
		if (set_bus_busy() == -1)
			return -EINVAL;
		*/
		ret = i2c_smbus_read_byte_data(g_client, reg);
		/* set_bus_free(); */
		if (ret >= 0) {
			*((unsigned char *)pval) = (unsigned char)ret;
			status = 0;
		}
	} else if ((reg >= MCU_OFFSET_WORD) && (reg < MCU_END)) {
		/*
		if (set_bus_busy() == -1)
			return -EINVAL;
		*/
		ret = i2c_smbus_read_word_data(g_client, reg);
		/* set_bus_free(); */

		if (ret >= 0) {
			*((unsigned short *)pval) = (unsigned short)ret;
			status = 0;
		}
	} else
		return -EINVAL;

	return status;
}


EXPORT_SYMBOL(power_mcu_read);

int power_mcu_write_word(unsigned char reg, unsigned short val)
{
	unsigned char data;
	int ret;

	if (g_client == NULL)
		return -EFAULT;
	if (reg >= MCU_END)
		return -EINVAL;

	/*
	if (set_bus_busy() == -1)
		return -EINVAL;
	*/
	ret = i2c_smbus_write_word_data(g_client, reg, val);
	/* set_bus_free(); */

	return ret;
}

int power_mcu_write_byte(unsigned char reg, unsigned char val)
{
	unsigned char data;
	int ret;

	if (g_client == NULL)
		return -EFAULT;

	if (reg >= MCU_END)
		return -EINVAL;

	/*
	if (set_bus_busy() == -1)
		return -EINVAL;
	*/
	ret = i2c_smbus_write_byte_data(g_client, reg, val);
	/* set_bus_free(); */

	return ret;
}

EXPORT_SYMBOL(power_mcu_write_byte);
EXPORT_SYMBOL(power_mcu_write_word);

void power_mcu_write_rtc(u32 rtc)
{
	power_mcu_write_word(MCU_RTC_LSB, rtc);
	power_mcu_write_word(MCU_RTC_MSB , rtc >> 16);
}

void power_mcu_read_rtc(u32 *rtc)
{
	power_mcu_read(MCU_RTC_LSB, rtc);
	power_mcu_read(MCU_RTC_MSB, ((u8 *)rtc + 2));
}

EXPORT_SYMBOL(power_mcu_write_rtc);
EXPORT_SYMBOL(power_mcu_read_rtc);

void power_mcu_write_alarm(u32 rtc)
{
	power_mcu_write_word(MCU_ALARM_LSB , rtc);
	power_mcu_write_word(MCU_ALARM_MSB , rtc >> 16);
}

void power_mcu_read_alarm(u32 *rtc)
{
	power_mcu_read(MCU_ALARM_LSB, rtc);
	power_mcu_read(MCU_ALARM_MSB, ((u8 *)rtc + 2));
}


EXPORT_SYMBOL(power_mcu_write_alarm);
EXPORT_SYMBOL(power_mcu_read_alarm);

static int __devinit power_i2c_probe(struct i2c_client *client)
{
	int i;
	int nretry = 25;
	unsigned char version;
	g_client = client;

	for (i = 0; i < nretry; i++) {
		if (power_mcu_read(MCU_VERSION, &version) == 0)
			return 0;
	}
	printk("power_mcu: probe failed\n");
	g_client = NULL;
	return -1;
}

static int __devexit power_i2c_remove(struct i2c_client *client)
{
	return 0;
}

static struct i2c_device_id power_mcu_idtable[] =
{
	{"power_mcu", 0},
	{},
};

static struct i2c_driver power_mcu_driver = {
	.driver = {
		.name	= "power_mcu",
	},
	.id_table		= power_mcu_idtable,
	.probe			= &power_i2c_probe,
	.remove			= &power_i2c_remove,
};

static struct device *dev;

static int __init power_mcu_init(void)
{
	int ret;

	printk("power_mcu_init\n");
	if ((ret = i2c_add_driver(&power_mcu_driver))) {
		printk(KERN_ERR "power mcu Driver registration failed\n");
		return ret;
	}

	power_mcu_class = class_create(THIS_MODULE, "power_mcu");
	if (IS_ERR(power_mcu_class)) {
		printk(KERN_WARNING "Unable to create \
		power_mcu class;errno=%ld\n", PTR_ERR(power_mcu_class));
		return PTR_ERR(power_mcu_class);
	}

	power_mcu_class->dev_attrs = power_mcu_attributes;
	dev = kzalloc(sizeof(struct device), GFP_KERNEL);
	dev->class = power_mcu_class;
	dev->release = NULL;
	ret = device_register(dev);
	if (ret)
		printk("power_mcu error:%d\n", -EEXIST);
	return 0;
}

static void __exit power_mcu_exit(void)
{
	i2c_del_driver(&power_mcu_driver);
	class_destroy(power_mcu_class);
}

module_init(power_mcu_init);
module_exit(power_mcu_exit);


