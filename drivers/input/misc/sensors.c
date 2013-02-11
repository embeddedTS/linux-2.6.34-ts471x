/*
 *  sensors.c - sensor work as input device
 *
 *  Copyright (C) 2008 Jack Ren <jack.ren@marvell.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/kthread.h>
#include <linux/semaphore.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/freezer.h>
#include <linux/uaccess.h>
#include <linux/proc_fs.h>
#include <linux/list.h>
#include <linux/sensor-input.h>
#include <asm/atomic.h>

#define DEFAULT_POLLING_DELAY	0xffffffff
/* #define PEERFORMANCE */

DEFINE_MUTEX(sensor_input_lock);
static struct input_dev		*sensor_input_idev;     /* input device */
static int			sensor_input_usage;
static LIST_HEAD(sensors);

static int sensor_input_kthread(void *data)
{
	struct sensor_input_dev *sensor = (struct sensor_input_dev *)data;
	int delay = sensor->delay;
#ifdef PEERFORMANCE
#define TIMEOUT 4
	int count, last_count;
	unsigned long timeout = jiffies + HZ*4;
#endif

	//daemonize(sensor->name);
#ifdef PEERFORMANCE
	timeout = jiffies + HZ*TIMEOUT;
#endif
	while(!sensor->thread_exit) {
		wait_event_timeout(sensor->wait, (delay!=sensor->delay) || sensor->thread_exit, msecs_to_jiffies(delay));
#ifdef PEERFORMANCE
		count++;
		if (time_after(jiffies, timeout)) {
			printk("%s: %dms/packet\n", sensor->name, TIMEOUT*1000/(count - last_count));
			last_count = count;
			timeout = jiffies + HZ*TIMEOUT;
		}
#endif
		if(sensor->delay==0)
			sensor->delay = DEFAULT_POLLING_DELAY;
		delay = sensor->delay;
		if (mutex_trylock(&sensor_input_lock)) {
			if (sensor->on) {
				if (sensor->dev)
					sensor->report(sensor->dev);
			}
			mutex_unlock(&sensor_input_lock);
		}
	}
	if (sensor->exit) {
		sensor->exit(sensor->dev);
	}
	//complete_and_exit(&sensor->thread_exit_complete, 0);
	return 0;
}

static int sensor_input_open(struct input_dev *input)
{
	struct sensor_input_dev *sensor;
	struct sensor_input_dev *tmp;

	mutex_lock(&sensor_input_lock);

	sensor = input_get_drvdata(input);

	if (sensor == NULL) {
		if (sensor_input_usage == 0) {
#ifdef CONFIG_INPUT_MERGED_SENSORS
			list_for_each_entry_safe(sensor, tmp, &sensors, list) {
				sensor->poweron();
				sensor->on = 1;
				sensor->thread_exit = 0;
				sensor->count++;
				sensor->thread_task = kthread_create(sensor_input_kthread, sensor, sensor->name);
				wake_up_process(sensor->thread_task);
			}
#endif
		}
		sensor_input_usage++;
	} else {
		if (sensor->count == 0) {
			sensor->poweron();
			sensor->on = 1;
			sensor->thread_exit = 0;
			sensor->thread_task = kthread_create(sensor_input_kthread, sensor, sensor->name);
			wake_up_process(sensor->thread_task);
		}
		sensor->count++;
	}

	mutex_unlock(&sensor_input_lock);
	return 0;
}

static void sensor_input_close(struct input_dev *input)
{
	struct sensor_input_dev *sensor;
	struct sensor_input_dev *tmp;

	mutex_lock(&sensor_input_lock);

	sensor = (struct sensor_input_dev *)input_get_drvdata(input);

	if (sensor == NULL) {
		sensor_input_usage--;
		if (sensor_input_usage == 0) {
#ifdef CONFIG_INPUT_MERGED_SENSORS
			list_for_each_entry_safe(sensor, tmp, &sensors, list) {
				sensor->count--;
				if (sensor->count == 0) {
					sensor->poweroff();
					sensor->on = 0;
					//init_completion(&sensor->thread_exit_complete);
					sensor->thread_exit = 1;
					wake_up(&sensor->wait);
					//wait_for_completion_timeout(&sensor->thread_exit_complete);
					kthread_stop(sensor->thread_task);
				}
			}
#endif
		}
	} else {
		sensor->count--;
		if (sensor->count == 0) {
			sensor->poweroff();
			sensor->on = 0;
			//init_completion(&sensor->thread_exit_complete);
			sensor->thread_exit = 1;
			wake_up(&sensor->wait);
			//wait_for_completion(&sensor->thread_exit_complete);
			kthread_stop(sensor->thread_task);
		}
	}
	mutex_unlock(&sensor_input_lock);
}

/* Sysfs stuff */
static ssize_t sensor_input_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int len = 0;
	struct sensor_input_dev *sensor;
	struct sensor_input_dev *tmp;

	mutex_lock(&sensor_input_lock);
	list_for_each_entry_safe(sensor, tmp, &sensors, list)
		len += sprintf(buf+len, "%s\t%d\n", sensor->name, sensor->delay);
	mutex_unlock(&sensor_input_lock);

	return len;
}

static ssize_t sensor_input_store(struct device *dev, \
		struct device_attribute *attr, const char *buf, size_t count)
{
	char name[PAGE_SIZE];
	int delay;
	int ret;
	struct sensor_input_dev *sensor;
	struct sensor_input_dev *tmp;

	ret = sscanf(buf, "%s %d", name, &delay);
	if(ret == 2) {
		mutex_lock(&sensor_input_lock);
		list_for_each_entry_safe(sensor, tmp, &sensors, list) {
			if(!strcmp(name, sensor->name)) {
				sensor->delay = delay;
				wake_up(&sensor->wait);
				printk(KERN_INFO "set %s delay to %dms\n", name, delay);
			}
		}
		mutex_unlock(&sensor_input_lock);
	}

	return count;
}

#ifdef	CONFIG_PM
static int sensor_input_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct sensor_input_dev *sensor;
	struct sensor_input_dev *tmp;

	if(sensor_input_usage == 0)
		return 0;

	if(!mutex_trylock(&sensor_input_lock))
		return -EBUSY;

	list_for_each_entry_safe(sensor, tmp, &sensors, list) {
		sensor->poweroff();
		sensor->on = 0;
	}

	return 0;
}

static int sensor_input_resume(struct platform_device *pdev)
{
	struct sensor_input_dev *sensor;
	struct sensor_input_dev *tmp;

	if(sensor_input_usage == 0)
		return 0;

	list_for_each_entry_safe(sensor, tmp, &sensors, list) {
		sensor->poweron();
		sensor->on = 1;
	}
	mutex_unlock(&sensor_input_lock);

	return 0;
}
#else
#define	sensor_input_suspend		NULL
#define	sensor_input_resume		NULL
#endif

#ifdef	CONFIG_PROC_FS
static ssize_t sensor_input_read_proc(struct file *file,
		char *buffer, size_t length, loff_t *offset)
{
	int len = 0;
	struct sensor_input_dev *sensor;
	struct sensor_input_dev *tmp;

	printk("Sensor Input List: \n");
	mutex_lock(&sensor_input_lock);
	list_for_each_entry_safe(sensor, tmp, &sensors, list) {
		printk("\tname: %s\tpower:%d\n",
				sensor->name, sensor->on?1:0);
	}
	mutex_unlock(&sensor_input_lock);

	return 0;
}

static ssize_t sensor_input_write_proc(struct file *file,
		const char __user *buffer,  size_t count, loff_t *offset)
{
	char input[PAGE_SIZE];
	int power, i=0;
	char name[40];
	struct sensor_input_dev *sensor;
	struct sensor_input_dev *tmp;

	if (copy_from_user(input, buffer, PAGE_SIZE))
		return -EFAULT;

	input[PAGE_SIZE-1] = 0;
	i = sscanf(input, "%s %d", name, &power);

	if(i == 2) {
		mutex_lock(&sensor_input_lock);
		list_for_each_entry_safe(sensor, tmp, &sensors, list) {
			if (!strcmp(name, sensor->name)) {
				sensor->on = power;
				if (power) {
					sensor->thread_exit = 0;
					kernel_thread(sensor_input_kthread, sensor, 0);
					printk(KERN_INFO "name %s\tpower: %d\n", sensor->name, power);
				} else {
					//init_completion(&sensor->thread_exit_complete);
					sensor->thread_exit = 1;
					wake_up(&sensor->wait);
					//wait_for_completion(&sensor->thread_exit_complete);
					kthread_stop(sensor->thread_task);
				}
			}
		}
		mutex_unlock(&sensor_input_lock);
	}

	return count;
}

static struct file_operations sensor_input_proc_ops = {
	.read = sensor_input_read_proc,
	.write = sensor_input_write_proc,
};

static void create_sensor_input_proc_file(void)
{
	struct proc_dir_entry *sensor_input_proc_file =
		create_proc_entry("driver/sensor-input", 0644, NULL);

	if (sensor_input_proc_file){
		sensor_input_proc_file->owner = THIS_MODULE;
		sensor_input_proc_file->proc_fops = &sensor_input_proc_ops;
	}
	else
		printk(KERN_INFO "proc file create failed!\n");
}

extern struct proc_dir_entry proc_root;
static void remove_sensor_input_proc_file(void)
{
	remove_proc_entry("driver/sensor-input", &proc_root);
}
#endif

struct device_attribute dev_attr_sensor_input = {
	.attr = {
		.name = "sensors",
		.mode = 0664,
	},
	.show = sensor_input_show,
	.store = sensor_input_store,
};

static struct attribute *sensor_input_attributes[] = {
	&dev_attr_sensor_input.attr,
	NULL
};

static struct attribute_group sensor_input_attribute_group = {
	.attrs = sensor_input_attributes
};

static int __devinit sensor_input_probe(struct platform_device *pdev)
{
	int err;

	if (sensor_input_idev)
		return -EINVAL;

	sensor_input_idev = input_allocate_device();
	if (!sensor_input_idev)
		return -ENOMEM;

	sensor_input_idev->name       = "sensor-input";
	sensor_input_idev->phys       =  "sensor-input/input0";
	sensor_input_idev->open       = sensor_input_open;
	sensor_input_idev->close      = sensor_input_close;

#ifdef CONFIG_INPUT_MERGED_SENSORS
	/* used as orientation sensor */
	__set_bit(EV_ABS, sensor_input_idev->evbit);
	__set_bit(ABS_RX, sensor_input_idev->absbit);
	__set_bit(ABS_RY, sensor_input_idev->absbit);
	__set_bit(ABS_RZ, sensor_input_idev->absbit);

	/* used as  acceleration sensor */
	__set_bit(ABS_X, sensor_input_idev->absbit);
	__set_bit(ABS_Y, sensor_input_idev->absbit);
	__set_bit(ABS_Z, sensor_input_idev->absbit);

	/* used as raw data */
	__set_bit(ABS_HAT1X, sensor_input_idev->absbit);
	__set_bit(ABS_HAT2X, sensor_input_idev->absbit);
	__set_bit(ABS_HAT3X, sensor_input_idev->absbit);
	__set_bit(ABS_MISC, sensor_input_idev->absbit);

	__set_bit(ABS_PRESSURE, sensor_input_idev->absbit);
	__set_bit(ABS_DISTANCE, sensor_input_idev->absbit);

	/* used as keyboard */
	__set_bit(EV_KEY, sensor_input_idev->evbit);
	__set_bit(EV_REP, sensor_input_idev->evbit);
	__set_bit(KEY_SEND, sensor_input_idev->keybit);
	__set_bit(KEY_3, sensor_input_idev->keybit);
	__set_bit(KEY_RIGHTCTRL, sensor_input_idev->keybit);
	__set_bit(KEY_HOME, sensor_input_idev->keybit);

	__set_bit(EV_SYN, sensor_input_idev->evbit);
#endif
	err = input_register_device(sensor_input_idev);
	if (err) {
		printk(KERN_ERR "register input driver error\n");
		input_free_device(sensor_input_idev);
		sensor_input_idev = NULL;
		return err;
	}

	input_set_drvdata(sensor_input_idev, NULL);
	err = sysfs_create_group(&pdev->dev.kobj, &sensor_input_attribute_group);
	if (err != 0){
		printk(KERN_ERR "register sysfs error\n");
		return err;
	}

#ifdef	CONFIG_PROC_FS
	create_sensor_input_proc_file();
#endif
	return 0;
}

static int sensor_input_remove(struct platform_device *pdev)
{
	if(!list_empty(&sensors))
		return -EBUSY;
	input_unregister_device(sensor_input_idev);
	sensor_input_idev = NULL;
	sysfs_remove_group(&pdev->dev.kobj, &sensor_input_attribute_group);

#ifdef	CONFIG_PROC_FS
	remove_sensor_input_proc_file();
#endif
	return 0;
}

static struct platform_driver sensor_input_driver = {
        .probe          = sensor_input_probe,
        .remove         = sensor_input_remove,
	.suspend	= sensor_input_suspend,
	.resume		= sensor_input_resume,
	.driver		= {
	        .name	= "sensor_input",
		.owner	= THIS_MODULE,
	},
};

int __init sensor_input_init(void)
{
	return platform_driver_register(&sensor_input_driver);
}

void __exit sensor_input_exit(void)
{
	platform_driver_unregister(&sensor_input_driver);
}

int sensor_input_add(int type, char *name,
	void (*report)(struct input_dev	*),
	void (*exit)(struct input_dev	*),
	void (*poweron)(void),
	void (*poweroff)(void))
{
	struct sensor_input_dev *sensor =
		kzalloc(sizeof(struct sensor_input_dev), GFP_KERNEL);

	struct input_dev *sensor_input_idev_ext;

	sensor->dev = NULL;
	sensor->name = name;
	sensor->report = report;
	sensor->exit = exit;
	sensor->poweron = poweron;
	sensor->poweroff = poweroff;
	sensor->delay = DEFAULT_POLLING_DELAY;
	init_waitqueue_head(&sensor->wait);
	mutex_lock(&sensor_input_lock);
	list_add(&sensor->list, &sensors);
#ifdef CONFIG_INPUT_MERGED_SENSORS
	if (sensor_input_usage != 0) {
		sensor->poweron();
		sensor->on = 1;
		sensor->thread_exit = 0;
		kernel_thread(sensor_input_kthread, sensor, 0);
	}
#endif
	mutex_unlock(&sensor_input_lock);

#ifdef CONFIG_INPUT_MERGED_SENSORS
	sensor->dev = sensor_input_idev;
#else
	sensor_input_idev_ext = input_allocate_device();
	if (!sensor_input_idev)
		return -ENOMEM;

	sensor_input_idev_ext->phys       =  "sensor-input/input0";
	sensor_input_idev_ext->open       = sensor_input_open;
	sensor_input_idev_ext->close      = sensor_input_close;
	sensor_input_idev_ext->name       = name;

	if (type == INPUT_G_SENSOR) {
		/* used as  acceleration sensor */
		__set_bit(EV_ABS, sensor_input_idev_ext->evbit);
		__set_bit(ABS_X, sensor_input_idev_ext->absbit);
		__set_bit(ABS_Y, sensor_input_idev_ext->absbit);
		__set_bit(ABS_Z, sensor_input_idev_ext->absbit);

		input_set_abs_params(sensor_input_idev_ext, ABS_X, -100000, 100000, 0, 0);
		input_set_abs_params(sensor_input_idev_ext, ABS_Y, -100000, 100000, 0, 0);
		input_set_abs_params(sensor_input_idev_ext, ABS_Z, -100000, 100000, 0, 0);
		input_set_abs_params(sensor_input_idev_ext, ABS_PRESSURE, 0, 255, 0, 0);
		input_set_abs_params(sensor_input_idev_ext, ABS_TOOL_WIDTH, 0, 15, 0, 0);

		input_set_drvdata(sensor_input_idev_ext, sensor);
		int err = input_register_device(sensor_input_idev_ext);
		if (err) {
			printk(KERN_ERR "register input driver error\n");
			input_free_device(sensor_input_idev_ext);
			sensor_input_idev_ext = NULL;
			return err;
		}

	} else if (type == INPUT_GYRO_SENSOR) {
		/* used as orientation sensor */
		__set_bit(EV_ABS, sensor_input_idev_ext->evbit);

		__set_bit(ABS_RX, sensor_input_idev_ext->absbit);
		__set_bit(ABS_RY, sensor_input_idev_ext->absbit);
		__set_bit(ABS_RZ, sensor_input_idev_ext->absbit);

		/* used as keyboard */
		__set_bit(EV_KEY, sensor_input_idev_ext->evbit);
		__set_bit(EV_REP, sensor_input_idev_ext->evbit);
	    	__set_bit(KEY_SEND, sensor_input_idev_ext->keybit);
		__set_bit(KEY_3, sensor_input_idev_ext->keybit);
		__set_bit(KEY_RIGHTCTRL, sensor_input_idev_ext->keybit);
		__set_bit(KEY_HOME, sensor_input_idev_ext->keybit);

		__set_bit(EV_SYN, sensor_input_idev_ext->evbit);

		/* used as raw data */
		__set_bit(ABS_HAT1X, sensor_input_idev_ext->absbit);
		__set_bit(ABS_HAT2X, sensor_input_idev_ext->absbit);
		__set_bit(ABS_HAT3X, sensor_input_idev_ext->absbit);
		__set_bit(ABS_MISC, sensor_input_idev_ext->absbit);

		input_set_abs_params(sensor_input_idev_ext, ABS_RX, -100000, 100000, 0, 0);
		input_set_abs_params(sensor_input_idev_ext, ABS_RY, -100000, 100000, 0, 0);
		input_set_abs_params(sensor_input_idev_ext, ABS_RZ, -100000, 100000, 0, 0);

		input_set_drvdata(sensor_input_idev_ext, sensor);
		int err = input_register_device(sensor_input_idev_ext);
		if (err) {
			printk(KERN_ERR "register input driver error\n");
			input_free_device(sensor_input_idev_ext);
			sensor_input_idev_ext = NULL;
			return err;
		}

	} else if (type == INPUT_AMBIENT_SENSOR) {
		/*used as light sensor*/
		__set_bit(EV_ABS, sensor_input_idev_ext->evbit);
		__set_bit(ABS_PRESSURE, sensor_input_idev_ext->absbit);

		input_set_abs_params(sensor_input_idev_ext, ABS_PRESSURE, -100000, 100000000, 0, 0);

		input_set_drvdata(sensor_input_idev_ext, sensor);
		int err = input_register_device(sensor_input_idev_ext);
		if (err) {
			printk(KERN_ERR "register input driver error\n");
			input_free_device(sensor_input_idev_ext);
			sensor_input_idev_ext = NULL;
			return err;
		}

	} else if (type == INPUT_PROXIMITY_SENSOR) {
		/*used as light sensor*/
		__set_bit(EV_ABS, sensor_input_idev_ext->evbit);
		__set_bit(ABS_DISTANCE, sensor_input_idev_ext->absbit);

		input_set_abs_params(sensor_input_idev_ext, ABS_DISTANCE, 0, 1, 0, 0);

		input_set_drvdata(sensor_input_idev_ext, sensor);
		int err = input_register_device(sensor_input_idev_ext);
		if (err) {
			printk(KERN_ERR "register input driver error\n");
			input_free_device(sensor_input_idev_ext);
			sensor_input_idev_ext = NULL;
			return err;
		}

	} else {
		input_free_device(sensor_input_idev_ext);
		sensor_input_idev_ext = NULL;
		return -1;
	}
	sensor->dev = sensor_input_idev_ext;
#endif
	return 0;
}

void sensor_input_del(char *name)
{
	struct sensor_input_dev *sensor;
	struct sensor_input_dev *tmp;

	mutex_lock(&sensor_input_lock);
	list_for_each_entry_safe(sensor, tmp, &sensors, list) {
		if(!strcmp(name, sensor->name)) {
			if (sensor->count != 0)
				return;
			input_unregister_device(sensor->dev);
			list_del(&sensor->list);
			kfree(sensor);
			return;
		}
	}
	mutex_unlock(&sensor_input_lock);
	printk(KERN_ERR "Try to remove a unknow sensor: %s\n", name);
}

MODULE_DESCRIPTION("Sensor Input driver");
MODULE_AUTHOR("Bin Yang <bin.yang@marvell.com>");
MODULE_LICENSE("GPL");

module_init(sensor_input_init);
module_exit(sensor_input_exit);

