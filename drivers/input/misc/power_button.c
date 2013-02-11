#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/reboot.h>
#include <linux/pm.h>
#include <linux/delay.h>
#include <mach/gpio.h>
#include <mach/mfp.h>
#include <mach/mfp-pxa168.h>
#include <plat/mfp.h>
#include <linux/input.h>
#include <mach/power_button.h>

#include <asm/mach-types.h>
#include <linux/wakelock.h>

enum PM_EVENT{
    PM_STANDBY ,
    PM_SHUTDOWN,
    PM_STANDBY_NOW ,
    PM_SHUTDOWN_NOW,
    PM_NORMAL,
 };

struct work_struct irq_work;
static struct power_button_platform_data *ops;
struct input_dev *input;
static struct wake_lock power_button_wakeup;

atomic_t event;

enum PM_STATUS{
    PM_INIT,
    PM_SHUTDOWN_WAITING,
};

enum PM_STATUS status_pm;

static struct class *power_button_class;

#define SLEEP_CODE 0x58
#define SHUTDOWN_CODE 0x57

void shutdown_ok(void)
{
	printk("shutdown now\n");
       kernel_power_off();
}

void standby_ok(void)
{
	printk("standby now\n");
	ops->send_standby_ack();
}

static ssize_t shutdown_show_status(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", 0);
}

static ssize_t standby_show_status(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", 0);
}

static ssize_t shutdown_store_status(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	printk("OS shutdown the system\n");
	atomic_set(&event, PM_SHUTDOWN_NOW);
	schedule_work(&irq_work);
	return 0;
}

static ssize_t standby_store_status(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	printk("OS make the systemto be standby\n");
	atomic_set(&event , PM_STANDBY_NOW);
	schedule_work(&irq_work);
	return 0;
}

static struct device_attribute bl_device_attributes[] = {
	__ATTR(pm_shutdown, 0644, shutdown_show_status, shutdown_store_status),
	__ATTR(pm_standby, 0644, standby_show_status,
		     standby_store_status),
	__ATTR_NULL,
};

static void power_button_irq_work(struct work_struct *work)
{
	int status;

	status = atomic_read(&event) ;
	switch (status) {
	case PM_SHUTDOWN_NOW:
		shutdown_ok();
		return;
	case PM_STANDBY_NOW:
		standby_ok();
		return;
	case PM_SHUTDOWN:
		/* ACK */
		printk("long press is detected\n");
		ops->send_powerdwn_ack();
		input_event(input, EV_KEY, SHUTDOWN_CODE, 1);
		input_sync(input);
		input_event(input, EV_KEY, SHUTDOWN_CODE, 0);
		input_sync(input);

		return;
	case PM_STANDBY:
		printk("short press is detected\n");
		input_event(input, EV_KEY, SLEEP_CODE, 1);
		input_sync(input);
		input_event(input, EV_KEY, SLEEP_CODE, 0);
		input_sync(input);

		return;
	default:
		return;
	}

}

static irqreturn_t standby_handler(int irq, void *dev_id)
{
	atomic_set(&event, PM_STANDBY);
	schedule_work(&irq_work);

	return IRQ_HANDLED;
}

static irqreturn_t shutdown_handler(int irq, void *dev_id)
{
	atomic_set(&event, PM_SHUTDOWN);
	schedule_work(&irq_work);

	return IRQ_HANDLED;
}

static int power_button_probe(struct platform_device *pdev)
{
	int ret = 0;

	ops = pdev->dev.platform_data;
	ops->init(shutdown_handler, standby_handler);

	INIT_WORK(&irq_work , power_button_irq_work);

	atomic_set(&event , PM_NORMAL);


	input = input_allocate_device();
	if (!input)
		goto out;


	input->name = "power-button";
	input->phys = "power-button/input0";
	input->dev.parent = &pdev->dev;

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;

	input_set_capability(input, EV_KEY , SLEEP_CODE);

	input_set_capability(input, EV_KEY , SHUTDOWN_CODE);

	ret = input_register_device(input);
	if (ret) {
		pr_err("power button: Unable to register input device, "
			"error: %d\n", ret);
		goto out;
	}

	status_pm = PM_INIT;

	printk("power button probe finished\n");

	return 0;
out:
	return ret;
}

static int power_button_suspend(struct platform_device *pdev)
{
	if (wake_lock_active(&power_button_wakeup))
		return -EBUSY;

	return 0;
}

static int power_button_resume(struct platform_device *pdev)
{
	printk(KERN_ERR "%s\n", __func__);
	input_event(input, EV_KEY, SLEEP_CODE, 1);
	input_sync(input);
	input_event(input, EV_KEY, SLEEP_CODE, 0);
	input_sync(input);
	wake_lock_timeout(&power_button_wakeup, HZ * 5);

	return 0;
}

static struct platform_driver power_button_driver = {
	.probe		= power_button_probe,
	.driver 	= {
		.name	= "power-button",
		.owner	= THIS_MODULE,
	},
	.suspend	= power_button_suspend,
	.resume		= power_button_resume,
};

static int __init power_button_init(void)
{
	int ret;

	power_button_class = class_create(THIS_MODULE, "power-button");
	if (IS_ERR(power_button_class)) {
		printk(KERN_WARNING "Unable to create power_button class; errno = %ld\n",
				PTR_ERR(power_button_class));
		return PTR_ERR(power_button_class);
	}

	power_button_class->dev_attrs = bl_device_attributes;

	ret = platform_driver_register(&power_button_driver);
	if (ret) {
		printk(KERN_ERR "power_button_driver register failure\n");
		return ret;
	}

	wake_lock_init(&power_button_wakeup, WAKE_LOCK_SUSPEND, "power_button");

	return 0;
}

static void __exit power_button_exit(void)
{
	input_unregister_device(input);
	platform_driver_unregister(&power_button_driver);
	class_destroy(power_button_class);
	wake_lock_destroy(&power_button_wakeup);
}


module_init(power_button_init);
module_exit(power_button_exit);

MODULE_LICENSE("GPL");


