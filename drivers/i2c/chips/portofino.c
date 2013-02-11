/*
 * * Monahans Portofino PMIC Management Routines
 * *
 * * Copyright (C) 2009, Marvell Corporation(bin.yang@marvell.com).
 * *
 * * Author: Bin Yang <bin.yang@marvell.com> 
 * *				 Yael Sheli Chemla<yael.s.shemla@marvell.com> 
 * *
 * * This software program is licensed subject to the GNU General Public License
 * * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 * */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <plat/pxa3xx_pmic.h>
#include <mach/portofino.h>

#define PORTOFINO_REG_NUM		0x1C

static struct i2c_client *g_client;

int portofino_read(u8 reg, u8 *pval)
{
	int ret;
	int status;

	if (g_client == NULL)	/* No global client pointer? */
		return -1;
	ret = i2c_smbus_read_byte_data(g_client, reg);
	if (ret >= 0) {
		*pval = ret;
		status = 0;
	} else {
		status = -EIO;
	}

	return status;
}
EXPORT_SYMBOL(portofino_read);

int portofino_write(u8 reg, u8 val)
{
	int ret;
	int status;

	if (g_client == NULL)	/* No global client pointer? */
		return -1;
	ret = i2c_smbus_write_byte_data(g_client, reg, val);
	if (ret == 0)
		status = 0;
	else
		status = -EIO;

	return status;
}
EXPORT_SYMBOL(portofino_write);

static int portofino_initchip(void)
{
	return 0;
}

int portofino_set_vibrator(unsigned char value)
{
	return 0;
}
EXPORT_SYMBOL(portofino_set_vibrator);

#ifdef	CONFIG_PM
static int portofino_suspend(struct i2c_client *client, pm_message_t state)
{
	return 0;
}

static int portofino_resume(struct i2c_client *client)
{
	return 0;
}
#else
#define	portofino_suspend		NULL
#define	portofino_resume		NULL
#endif

#ifdef	CONFIG_PROC_FS
#define	PORTOFINO_PROC_FILE	"driver/portofino"
static struct proc_dir_entry *portofino_proc_file;
static int index;

static ssize_t portofino_proc_read(struct file *filp,
		char *buffer, size_t length, loff_t *offset)
{
	u8 reg_val;

	if ((index < 0) || (index > PORTOFINO_REG_NUM))
		return 0;

	portofino_read(index, &reg_val);
	printk(KERN_INFO "register 0x%x: 0x%x\n", index, reg_val);
	return 0;
}

static ssize_t portofino_proc_write(struct file *filp,
		const char *buff, size_t len, loff_t *off)
{
	u8 reg_val;
	char messages[256], vol[256];
	u32 l;
	static int init_led;

	if (len > 256)
		len = 256;

	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	if ('-' == messages[0]) {
		/* set the register index */
		memcpy(vol, messages+1, len-1);
		index = (int) simple_strtoul(vol, NULL, 16);
	}else if ('l' == messages[0]) {
		if(!init_led) {
			init_led = 1;
			portofino_read(0x7, &reg_val);
			portofino_write(0x7, reg_val|0x2);
			portofino_write(0xc, 0xff);
		}
		/* set the register index */
		memcpy(vol, messages+1, len-1);
		l = (int) simple_strtoul(vol, NULL, 16);
		printk(KERN_INFO "Set LED-%d to 0x%x\n", 
				(l&0xf00)>>8, (l&0x1f));
		if(l&0xff)
			portofino_write(0xd+((l&0xf00)>>8), (l&0x1f)+0x20);
		else
			portofino_write(0xd+((l&0xf00)>>8), 0x0);
	} else {
		/* set the register value */
		reg_val = (int)simple_strtoul(messages, NULL, 16);
		portofino_write(index, reg_val & 0xFF);
	}

	return len;
}

static struct file_operations portofino_proc_ops = {
	.read = portofino_proc_read,
	.write = portofino_proc_write,
};

static void create_portofino_proc_file(void)
{
	portofino_proc_file = create_proc_entry(PORTOFINO_PROC_FILE, 0644, NULL);
	if (portofino_proc_file) {
		portofino_proc_file->owner = THIS_MODULE;
		portofino_proc_file->proc_fops = &portofino_proc_ops;
	} else
		printk(KERN_INFO "proc file create failed!\n");
}

static void remove_portofino_proc_file(void)
{
	extern struct proc_dir_entry proc_root;
	remove_proc_entry(PORTOFINO_PROC_FILE, &proc_root);
}

#endif
static int __devinit portofino_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int ret;
	int chip_id;

	chip_id = i2c_smbus_read_byte_data(client, PORTOFINO_ID);
	if (chip_id < 0) {
		printk(KERN_WARNING "portofino unavailable!\n");
		return -ENXIO;
	} else {
		printk(KERN_INFO "portofino(chip id:0x%02x) detected.\n", chip_id);
	}

	g_client = client;

	ret = portofino_initchip();
	if (ret != 0)
		printk(KERN_WARNING "Initialize Portofino failed with ret 0x%x\n",
			ret);

#ifdef	CONFIG_PROC_FS
	create_portofino_proc_file();
#endif
	return 0;
}

static int portofino_remove(struct i2c_client *client)
{
#ifdef	CONFIG_PROC_FS
	remove_portofino_proc_file();
#endif
	return 0;
}

static const struct i2c_device_id portofino_id[] = {
	{ "portofino", 0 },
	{ }
};

static struct i2c_driver portofino_driver = {
	.driver = {
		.name	= "portofino",
	},
	.id_table 	= portofino_id,
	.probe		= portofino_probe,
	.remove		= portofino_remove,
	.suspend	= portofino_suspend,
	.resume		= portofino_resume,
};

static int __init portofino_init(void)
{
	return i2c_add_driver(&portofino_driver);
}

static void __exit portofino_exit(void)
{
	i2c_del_driver(&portofino_driver);
}

subsys_initcall(portofino_init);
module_exit(portofino_exit);

MODULE_DESCRIPTION("Portofino Driver");
MODULE_LICENSE("GPL");

