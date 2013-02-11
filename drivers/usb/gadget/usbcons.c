/*
 * USB Serial Console driver
 *
 * Copyright (C) 2001 - 2002 Greg Kroah-Hartman (greg@kroah.com)
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License version
 *	2 as published by the Free Software Foundation.
 *
 * Thanks to Randy Dunlap for the original version of this code.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/tty.h>
#include <linux/console.h>
#include <linux/usb.h>
#include <linux/proc_fs.h>

static int debug;

struct usbcons_info {
	int			magic;
	int			n_ports;
	struct portmaster	*ports;
	struct usb_gadget	*gadget;
};

static struct usbcons_info usbcons_info;
static struct console usbcons;

/*
 * ------------------------------------------------------------
 * USB Serial console driver
 *
 * Much of the code here is copied from drivers/char/serial.c
 * and implements a phony serial console in the same way that
 * serial.c does so that in case some software queries it,
 * it will get the same results.
 *
 * Things that are different from the way the serial port code
 * does things, is that we call the lower level usb-serial
 * driver code to initialize the device, and we set the initial
 * console speeds based on the command line arguments.
 * ------------------------------------------------------------
 */

static int stop_console = 1;
static int __init usbcons_setup(char *__unused)
{
	stop_console = 0;
	return 1;
}
__setup("usbcons", usbcons_setup);

#include "linux/netdevice.h"
static void usbcons_write_gs(struct tty_struct *tty,
			const char *buf, unsigned count)
{
	gs_write(tty, buf, count);
	while (gs_write_polling)
		usb_gadget_ioctl(usbcons_info.gadget, USB_GADGET_POLLING, 0);
}

static void usbcons_write_msg(struct tty_struct *tty,
			const char *buf, unsigned count)
{
	while (count) {
		unsigned int i;
		unsigned int lf;
		/* search for LF so we can insert CR if necessary */
		for (i = 0, lf = 0 ; i < count ; i++) {
			if (*(buf + i) == 10) {
				lf = 1;
				i++;
				break;
			}
		}
		/* pass on to the driver specific version of this function if
		   it is available */
		usbcons_write_gs(tty, buf, i);
		if (lf) {
			/* append CR after LF */
			unsigned char cr = 13;
			usbcons_write_gs(tty, &cr, 1);
		}
		buf += i;
		count -= i;
	}
}

void usbcons_write(struct console *co, const char *buf, unsigned count)
{
	struct portmaster *ports = usbcons_info.ports;
	struct gs_port	*port;
	unsigned long flags;
	int i;

	if (stop_console)
		return;
	local_irq_save(flags);
	for (i = 0; i < usbcons_info.n_ports; i++) {
		port = ports->port;
		if (port && port->port_tty && port->open_count) {
			usbcons_write_msg(port->port_tty, buf, count);

		}
		ports++;
	}
	local_irq_restore(flags);
}

static struct console usbcons = {
	.name =		"ttyGS0",
	.write =	usbcons_write,
	.flags =	CON_ENABLED,
	.index =	-1,
};

static int usbcons_proc_read(char *buffer, char **buffer_location,
	       off_t offset, int buffer_length, int *zero, void *ptr)
{
	return 0;
}
static int usbcons_proc_write(struct file *file, const char *buffer,
		unsigned long count, void *data)
{
	char kbuf[8];
	int index;

	if (count >= 8)
		return -EINVAL;
	if (copy_from_user(kbuf, buffer, count))
		return -EFAULT;
	index = (int)simple_strtoul(kbuf, NULL, 10);

	switch (index) {
	case 1:
		stop_console = 0;
		break;
	case 2:
		stop_console = 1;
		break;
	case 3:
		printk(KERN_INFO "usb serial console %s\n",
			stop_console ? "disabled" : "enabled");
		break;
	default:
		return -EINVAL;
	}

    return count;
}


static struct proc_dir_entry *usbcons_proc_file;
int usbcons_proc_init(void)
{
  usbcons_proc_file = create_proc_entry("usbcons" , 0666 , NULL);
  usbcons_proc_file->read_proc = usbcons_proc_read;
  usbcons_proc_file->write_proc = usbcons_proc_write;
  return 0;
}

void usbcons_deinit(void)
{
	unregister_console(&usbcons);
}

void usbcons_init(struct usb_gadget *g, struct portmaster *ports,
		unsigned n_ports)
{
	debug = 1;
	usbcons_info.gadget = g;
	usbcons_info.ports = ports;
	usbcons_info.n_ports = n_ports;

	/*
	 * Call register_console() if this is the first device plugged
	 * in.  If we call it earlier, then the callback to
	 * console_setup() will fail, as there is not a device seen by
	 * the USB subsystem yet.
	 */
	/*
	 * Register console.
	 * NOTES:
	 * console_setup() is called (back) immediately (from
	 * register_console). console_write() is called immediately
	 * from register_console iff CON_PRINTBUFFER is set in flags.
	 */
	dbg("registering the USB serial console.");
	printk(KERN_INFO "USB serial console: n_ports %d ports %p\n",
			n_ports, ports);
	register_console(&usbcons);
	usbcons_proc_init();
}

void usbcons_exit(void)
{
	unregister_console(&usbcons);
}

