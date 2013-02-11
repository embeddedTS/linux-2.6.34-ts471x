/*
 *  linux/drivers/input/touchscreen/tsc2007.c
 *
 *  touch screen driver for tsc2007
 *
 *  Copyright (C) 2006, Marvell Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/freezer.h>
#include <linux/proc_fs.h>
#include <linux/clk.h>
#include <linux/i2c.h>
#include <mach/gpio.h>

#include <linux/sysctl.h>
#include <asm/system.h>

extern int ts_linear_scale(int *x, int *y, int swap_xy);

/* Use MAV filter */
#define TSC_CMD_SETUP 0xb0

/* Use 12-bit */
#define TSC_CMD_X 0xc0
#define TSC_CMD_PLATEX 0x80
#define TSC_CMD_Y 0xd0
#define TSC_CMD_PLATEY 0x90

#define TSC_X_MAX 4096
#define TSC_Y_MAX 4096
#define TSC_X_MIN 0
#define TSC_Y_MIN 0

/* delay time for compute x, y, computed as us */

#ifdef DEBUG
#define TS_DEBUG(fmt,args...) printk(KERN_DEBUG fmt, ##args )
#else
#define TS_DEBUG(fmt,args...)
#endif
static int x_min=TSC_X_MIN;
static int y_min=TSC_Y_MIN;
static int x_max=TSC_X_MAX;
static int y_max=TSC_Y_MAX;
static int invert = 0;
static int debounce_time  = 150;
static int init_debounce = true;
static int delay_time = 1;

enum tsc2007_status {
	PEN_UP,
	PEN_DOWN,
};

struct _tsc2007 {
	struct input_dev *dev;
	int x;		/* X sample values */
	int y;		/* Y sample values */

	int status;
	struct work_struct irq_work;
	struct i2c_client *client;
	unsigned long last_touch;
};
struct _tsc2007 *g_tsc2007;

/* update abs params when min and max coordinate values are set */
int tsc2007_proc_minmax(struct ctl_table *table, int write,
                     void __user *buffer, size_t *lenp, loff_t *ppos)
{
	struct _tsc2007 *tsc2007= g_tsc2007;
	struct input_dev *input = tsc2007->dev;

	/* update value */
	int ret = proc_dointvec(table, write, buffer, lenp, ppos);

	/* updated abs params */
	if (input) {
		TS_DEBUG(KERN_DEBUG "update x_min %d x_max %d"
			" y_min %d y_max %d\n", x_min, x_max,
			y_min, y_max); 
		input_set_abs_params(input, ABS_X, x_min, x_max, 0, 0);
		input_set_abs_params(input, ABS_Y, y_min, y_max, 0, 0);
	}
	return ret;
}

static ctl_table tsc2007_proc_table[] = {
	{
		.procname	= "x-max",
		.data		= &x_max,
		.maxlen		= sizeof(int),
		.mode		= 0666,
		.proc_handler	= &tsc2007_proc_minmax,
	},
	{
		.procname	= "y-max",
		.data		= &y_max,
		.maxlen		= sizeof(int),
		.mode		= 0666,
		.proc_handler	= &tsc2007_proc_minmax,
	},
	{
		.procname	= "x-min",
		.data		= &x_min,
		.maxlen		= sizeof(int),
		.mode		= 0666,
		.proc_handler	= &tsc2007_proc_minmax,
	},
	{
		.procname	= "y-min",
		.data		= &y_min,
		.maxlen		= sizeof(int),
		.mode		= 0666,
		.proc_handler	= &tsc2007_proc_minmax,
	},
	{
		.procname	= "invert_xy",
		.data		= &invert,
		.maxlen		= sizeof(int),
		.mode		= 0666,
		.proc_handler	= &proc_dointvec,
	},
	{
		.procname	= "debounce_time",
		.data		= &debounce_time,
		.maxlen		= sizeof(int),
		.mode		= 0666,
		.proc_handler	= &proc_dointvec,
	},
	{
		.procname	= "delay_time",
		.data		= &delay_time,
		.maxlen		= sizeof(int),
		.mode		= 0666,
		.proc_handler	= &proc_dointvec,
	},
	{ }
};

static ctl_table tsc2007_proc_root[] = {
	{
		.procname	= "ts_device",
		.mode		= 0555,
		.child		= tsc2007_proc_table,
	},
	{ }
};

static ctl_table tsc2007_proc_dev_root[] = {
	{
		.procname	= "dev",
		.mode		= 0555,
		.child		= tsc2007_proc_root,
	},
	{ }
};

static struct ctl_table_header *sysctl_header;

static int __init init_sysctl(void)
{
    sysctl_header = register_sysctl_table(tsc2007_proc_dev_root);
    return 0;
}

static void __exit cleanup_sysctl(void)
{
    unregister_sysctl_table(sysctl_header);
}

static int tsc2007_measure(struct i2c_client *client, int *x, int * y)
{
	u8 x_buf[2] = {0, 0};
	u8 y_buf[2] = {0, 0};

	i2c_smbus_write_byte(client, TSC_CMD_PLATEX);
	msleep_interruptible(delay_time);

	i2c_smbus_write_byte(client, TSC_CMD_X);
	i2c_master_recv(client, x_buf, 2);
	*x = (x_buf[0]<<4) | (x_buf[1] >>4);

	i2c_smbus_write_byte(client, TSC_CMD_PLATEY);
	msleep_interruptible(delay_time);

	i2c_smbus_write_byte(client, TSC_CMD_Y);
	i2c_master_recv(client, y_buf, 2);
	*y = (y_buf[0]<<4) | (y_buf[1] >>4);

	return 0;
}

static void tsc2007_irq_work(struct work_struct *work)
{
	struct _tsc2007 *tsc2007= g_tsc2007;
	struct i2c_client *client = tsc2007-> client;
	struct input_dev *input = tsc2007->dev;

	int x = -1, y = -1, is_valid = 0;
	int tmp_x = 0, tmp_y = 0;

	int gpio = irq_to_gpio(client->irq);


	/* Ignore if PEN_DOWN */
	if(PEN_UP == tsc2007->status){

		if (gpio_request(gpio, "tsc2007 touch detect")) {
			printk(KERN_ERR "Request GPIO failed, gpio: %X\n", gpio);
			return;
		}
		gpio_direction_input(gpio);	
		
		while(0 == gpio_get_value(gpio)){

                        if ((jiffies_to_msecs(
                                ((long)jiffies - (long)tsc2007->last_touch)) <
				 debounce_time &&
				tsc2007->status == PEN_DOWN) ||
				init_debounce)
                        {
				init_debounce = false;
                                tsc2007_measure(client, &tmp_x, &tmp_y);
                                TS_DEBUG(KERN_DEBUG
				"dropping pen touch %lu %lu (%u)\n",
                                jiffies, tsc2007->last_touch,
                                jiffies_to_msecs(
				(long)jiffies - (long)tsc2007->last_touch));
                                schedule();
				continue;
                        }


			/* continue report x, y */
			if (x > 0 && y > 0)
			{
				int ox=x, oy=y;
				ts_linear_scale(&x, &y, invert);
				TS_DEBUG(KERN_DEBUG "ox=%d oy=%d x=%d y=%d!\n", ox, oy, x, y);
				input_report_abs(input, ABS_X, x);
				input_report_abs(input, ABS_Y, y);
				input_report_abs(input, ABS_PRESSURE, 255);
				input_report_abs(input, ABS_TOOL_WIDTH, 1);
				input_report_key(input, BTN_TOUCH, 1);
				input_sync(input);
			}

			tsc2007->status = PEN_DOWN;
			tsc2007_measure(client, &x, &y);
			is_valid = 1;
			schedule();
		}

		if (is_valid)
		{
			/*consider PEN_UP */
			tsc2007->status = PEN_UP;
			input_report_abs(input, ABS_PRESSURE, 0);
			input_report_abs(input, ABS_TOOL_WIDTH, 1);
			input_report_key(input, BTN_TOUCH, 0);
			input_sync(input);
			tsc2007->last_touch = jiffies;
			TS_DEBUG(KERN_DEBUG "pen up!\n"); 
		}

		gpio_free(gpio);	
	}
}

static irqreturn_t tsc2007_interrupt(int irq, void *dev_id)
{	
	schedule_work(&g_tsc2007->irq_work);
	
	return IRQ_HANDLED;
}

static int __devinit tsc2007_probe(struct i2c_client *client, 
				const struct i2c_device_id *id)
{
	struct _tsc2007 *tsc2007;
	struct input_dev *input_dev;
	int ret;

	tsc2007 = kzalloc(sizeof(struct _tsc2007), GFP_KERNEL);
	input_dev = input_allocate_device();

	g_tsc2007 = tsc2007;

	if (!tsc2007 || !input_dev) {
		ret = -ENOMEM;
		goto fail1;
	}

	i2c_set_clientdata(client, tsc2007);

	tsc2007->dev = input_dev;

	input_dev->name = "tsc2007";
	input_dev->phys = "tsc2007/input0";

	//input_dev->id.bustype = BUS_HOST;
	input_dev->dev.parent = &client->dev;

	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(ABS_PRESSURE, input_dev->evbit);
	__set_bit(ABS_X, input_dev->evbit);
	__set_bit(ABS_Y, input_dev->evbit);

	input_set_abs_params(input_dev, ABS_X, x_min, x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, y_min, y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, 255, 0, 0);

	ret = request_irq(client->irq, tsc2007_interrupt, 
		IRQF_DISABLED | IRQF_TRIGGER_FALLING,
		 "tsc2007 irq", NULL);
	if (ret){
		printk(KERN_ERR "tsc2007 request irq failed\n");
		goto fail2;
	}

	ret = input_register_device(tsc2007->dev);
	if (ret){
		printk(KERN_ERR "tsc2007 register device fail\n");
		goto fail2;
	}

	/*init */
	tsc2007->status = PEN_UP;
	tsc2007->client = client;
	tsc2007->last_touch = jiffies;

	INIT_WORK(&tsc2007->irq_work, tsc2007_irq_work);

	/* init tsc2007 */
	i2c_smbus_write_byte(client, TSC_CMD_SETUP);

	return 0;

 fail2:
	free_irq(client->irq, client);
 fail1:
	i2c_set_clientdata(client, NULL);
	input_free_device(input_dev);
	kfree(tsc2007);
	return ret;
}

static int __devexit tsc2007_remove(struct i2c_client *client)
{
	struct _tsc2007 *tsc2007 = i2c_get_clientdata(client);

	if(client->irq)
		free_irq(client->irq, client);
	
	i2c_set_clientdata(client, NULL);
	input_unregister_device(tsc2007->dev);
	kfree(tsc2007);

	return 0;
}

static struct i2c_device_id tsc2007_idtable[] = { 
	{ "tsc2007", 0 }, 
	{ } 
}; 

MODULE_DEVICE_TABLE(i2c, tsc2007_idtable);

static struct i2c_driver tsc2007_driver = {
	.driver = {
		.name 	= "tsc2007",
	},
	.id_table       = tsc2007_idtable,
	.probe		= tsc2007_probe,
	.remove		= __devexit_p(tsc2007_remove),
};

static int __init tsc2007_ts_init(void)
{
	init_sysctl();
	return i2c_add_driver(&tsc2007_driver);	 
}

static void __exit tsc2007_ts_exit(void)
{
	cleanup_sysctl();
	i2c_del_driver(&tsc2007_driver);
}

module_init(tsc2007_ts_init);
module_exit(tsc2007_ts_exit);

MODULE_DESCRIPTION("tsc2007 touch screen driver");
MODULE_LICENSE("GPL");
