/*
 *  linux/drivers/media/video/cmmb/if101.c - cmmb if101 driver
 *
 *  Based on linux/drivers/media/video/cafe_ccic.c
 *
 *  Copyright:	(C) Copyright 2008 Marvell International Ltd.
 *              Mingwei Wang <mwwang@marvell.com>
 *
 * A driver for the CMOS camera controller in the Marvell 88ALP01 "cafe"
 * multifunction chip.  Currently works with the Omnivision OV7670
 * sensor.
 *
 * The data sheet for this device can be found at:
 *    http://www.marvell.com/products/pcconn/88ALP01.jsp
 *
 * Copyright 2006 One Laptop Per Child Association, Inc.
 * Copyright 2006-7 Jonathan Corbet <corbet@lwn.net>
 *
 * Written by Jonathan Corbet, corbet@lwn.net.
 *
 * This file may be distributed under the terms of the GNU General
 * Public License, version 2.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/videodev2.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-chip-ident.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/list.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/kthread.h>

#include <linux/vmalloc.h>
#include <linux/platform_device.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/cacheflush.h>

#include <linux/clk.h>

#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/spi/spi.h>
#include <linux/spi/cmmb.h>
#include <linux/gpio.h>
/* #include <asm/arch/hardware.h> */
/* #include <asm/arch/littleton.h> */

#define DRIVER_NAME "cmmb_if101"

#define SPI_CID V4L2_CID_PRIVATE_BASE

/* assemble an ioctl command */
#define CMMB_IOCTL(addr, data)	(((addr) << 16) | (data))

/* disassemble an ioctl command */
#define CMMB_IOCTL_ADDR(val)	((val) >> 16)
#define CMMB_IOCTL_DATA(val)	((val) & 0xffff)

/*
 * Parameters.
 */
MODULE_AUTHOR("Mingwei Wang");
MODULE_DESCRIPTION("CMMB Demodulator IF101 driver");
MODULE_LICENSE("GPL v2");
MODULE_SUPPORTED_DEVICE("Video");

/*
 * Internal DMA buffer management.  Since the controller cannot do S/G I/O,
 * we must have physically contiguous buffers to bring frames into.
 * These parameters control how many buffers we use, whether we
 * allocate them at load time (better chance of success, but nails down
 * memory) or when somebody tries to use the camera (riskier), and,
 * for load-time allocation, how big they should be.
 *
 * The controller can cycle through three buffers.  We could use
 * more by flipping pointers around, but it probably makes little
 * sense.
 */

#define MAX_DMA_BUFS 3
static int alloc_bufs_at_read = 0;
module_param(alloc_bufs_at_read, bool, 0444);
MODULE_PARM_DESC(alloc_bufs_at_read,
		"Non-zero value causes DMA buffers to be allocated when the "
		"video capture device is read, rather than at module load "
		"time.  This saves memory, but decreases the chances of "
		"successfully getting those buffers.");

static int dma_buf_size = 65536;
module_param(dma_buf_size, uint, 0444);
MODULE_PARM_DESC(dma_buf_size,
		"The size of the allocated DMA buffers.  If actual operating "
		"parameters require larger buffers, an attempt to reallocate "
		"will be made.");

static int min_buffers = 1;
module_param(min_buffers, uint, 0644);
MODULE_PARM_DESC(min_buffers,
		"The minimum number of streaming I/O buffers we are willing "
		"to work with.");

static int max_buffers = 10;
module_param(max_buffers, uint, 0644);
MODULE_PARM_DESC(max_buffers,
		"The maximum number of streaming I/O buffers an application "
		"will be allowed to allocate.  These buffers are big and live "
		"in vmalloc space.");

/*
 * A description of one of our devices.
 * Locking: controlled by s_mutex.  Certain fields, however, require
 *	    the dev_lock spinlock; they are marked as such by comments.
 *	    dev_lock is also required for access to device registers.
 */
struct cmmb_dev {
	unsigned long flags;		/* Buffer status, mainly (dev_lock) */
	int users;			/* How many open FDs */
	struct file *owner;		/* Who has data access (v4l2) */

	/*
	 * Subsystem structures.
	 */
	int irq;
	struct video_device v4ldev;
	struct v4l2_buffer v4lbuf;

	struct list_head dev_list;	/* link to other devices */

	/* DMA buffers */
	int mapcount;
	unsigned int dma_buf_size;	/* allocated size */
	int order;	/* Internal buffer addresses */

	void *dma_bufs;	/* Internal buffer addresses */
	dma_addr_t dma_handles; /* Buffer bus addresses */

	/* Locks */
	struct mutex s_mutex; /* Access to this structure */
	spinlock_t dev_lock;  /* Access to device */

	/* Misc */
	wait_queue_head_t iowait;	/* Waiting on frame data */
	struct spi_device *spi;
};

/*
 * Low-level register I/O.
 */

/* ---------------------------------------------------------------------*/
/*
 * We keep a simple list of known devices to search at open time.
 */
static LIST_HEAD(cmmb_dev_list);
static DEFINE_MUTEX(cmmb_dev_list_lock);

static void cmmb_add_dev(struct cmmb_dev *cam)
{
	mutex_lock(&cmmb_dev_list_lock);
	list_add_tail(&cam->dev_list, &cmmb_dev_list);
	mutex_unlock(&cmmb_dev_list_lock);
}

static void cmmb_remove_dev(struct cmmb_dev *cam)
{
	mutex_lock(&cmmb_dev_list_lock);
	list_del(&cam->dev_list);
	mutex_unlock(&cmmb_dev_list_lock);
}

static struct cmmb_dev *cmmb_find_dev(int minor)
{
	struct cmmb_dev *cam;

	mutex_lock(&cmmb_dev_list_lock);
	list_for_each_entry(cam, &cmmb_dev_list, dev_list) {
		if (cam->v4ldev.minor == minor)
			goto done;
	}
	cam = NULL;
done:
	mutex_unlock(&cmmb_dev_list_lock);
	return cam;
}

static struct cmmb_dev *cmmb_find_by_spi(struct spi_device *spi)
{
	struct cmmb_dev *cam;

	mutex_lock(&cmmb_dev_list_lock);
	list_for_each_entry(cam, &cmmb_dev_list, dev_list) {
		if (cam->spi == spi)
			goto done;
	}
	cam = NULL;
done:
	mutex_unlock(&cmmb_dev_list_lock);
	return cam;
}

/* -------------------------------------------------------------------- */
/*
 * DMA buffer management.  These functions need s_mutex held.
 */

/* FIXME: this is inefficient as hell, since dma_alloc_coherent just
 * does a get_free_pages() call, and we waste a good chunk of an orderN
 * allocation.  Should try to allocate the whole set in one chunk.
 */
static int cmmb_alloc_dma_bufs(struct cmmb_dev *cam)
{
	cam->dma_buf_size = dma_buf_size;

	cam->order = get_order(cam->dma_buf_size);
	cam->dma_bufs = (unsigned long *)__get_free_pages(GFP_KERNEL, cam->order);
	if (cam->dma_bufs == NULL) {
		printk("Failed to allocate DMA buffer\n");
		return -ENOMEM;
	}
	cam->dma_handles = __pa(cam->dma_bufs);

	/* For debug, remove eventually */
	memset(cam->dma_bufs, 0xcc, cam->dma_buf_size);

	return 0;
}

static void cmmb_free_dma_bufs(struct cmmb_dev *cam)
{
	free_pages((unsigned long)cam->dma_bufs, cam->order);
	cam->dma_bufs = NULL;
}


/* ----------------------------------------------------------------------- */
/*
 * Here starts the V4L2 interface code.
 */

static int cmmb_vidioc_g_ctrl(struct file *filp, void *priv,
		struct v4l2_control *ctrl)
{
	struct cmmb_dev *cam = filp->private_data;
	int ret;
	unsigned char addr = CMMB_IOCTL_ADDR(ctrl->value);
	unsigned char data = CMMB_IOCTL_DATA(ctrl->value);

	mutex_lock(&cam->s_mutex);
	if (ctrl->id == SPI_CID) {
		ret = spi_read(cam->spi, (u8 *)&data, sizeof(data));
	} else {
		printk("This control ID doesn't support get\n");
		ret = -EINVAL;
	}
	ctrl->value = CMMB_IOCTL(addr, data);
	mutex_unlock(&cam->s_mutex);
	return ret;
}


static int cmmb_vidioc_s_ctrl(struct file *filp, void *priv,
		struct v4l2_control *ctrl)
{
	struct cmmb_dev *cam = filp->private_data;
	int ret;
	/* unsigned char addr = CMMB_IOCTL_ADDR(ctrl->value); */
	unsigned char data = CMMB_IOCTL_DATA(ctrl->value);

	mutex_lock(&cam->s_mutex);
	if (ctrl->id == SPI_CID) {
		ret = spi_write(cam->spi, (const u8 *)&data, sizeof(data));
	} else {
		printk("This control ID doesn't support set\n");
		ret = -EINVAL;
	}
	mutex_unlock(&cam->s_mutex);
	return ret;
}


extern int spi_finish;
static int spi_read_bytes(struct cmmb_dev *cam, unsigned char *buffer, int len)
{
	int i, j;

	spi_finish = 0;
	for (i = 0; (i + 4096) < len; i += 4096) {
		spi_read(cam->spi, buffer + i, 4096);
	}
	/*
	 * The last spi_read need set spi_finish = 1,
	 * such led GPIO_CMMB_CS to be pull up
	 */
	spi_finish = 1;
	j = len % 4096;
	if (j)
		spi_read(cam->spi, buffer + i, j);
	else
		spi_read(cam->spi, buffer + i, 4096);

	return 0;
}


static int cmmb_vidioc_g_ext_ctrl(struct file *filp, void *priv,
		struct v4l2_ext_controls *ctrl)
{
	struct cmmb_dev *cam = filp->private_data;
	int ret = 1;
	unsigned char *buffer;
	int len;

	if (ctrl->count != 2)
		return -EINVAL;

	buffer = (unsigned char *)ctrl->controls[0].value;
	len = ctrl->controls[1].value;

	mutex_lock(&cam->s_mutex);

	dmac_inv_range(buffer, buffer + len);

	spi_read_bytes(cam, cam->dma_bufs, len);

	mutex_unlock(&cam->s_mutex);

	return ret;
}


static int cmmb_vidioc_try_fmt_cap(struct file *filp, void *priv,
		struct v4l2_format *fmt)
{
	return 0;
}


static int cmmb_vidioc_querybuf(struct file *filp, void *priv,
		struct v4l2_buffer *buf)
{
	struct cmmb_dev *cam = filp->private_data;
	int ret = -EINVAL;

	mutex_lock(&cam->s_mutex);
	if (buf->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		goto out;
	if (buf->index < 0)
		goto out;

	cam->v4lbuf.length = dma_buf_size;
	cam->v4lbuf.index = 0;
	cam->v4lbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	cam->v4lbuf.field = V4L2_FIELD_NONE;
	cam->v4lbuf.memory = V4L2_MEMORY_MMAP;
	cam->v4lbuf.m.offset = 0;

	*buf = cam->v4lbuf;

	ret = 0;
out:
	mutex_unlock(&cam->s_mutex);
	return ret;
}


static void cmmb_v4l_vm_open(struct vm_area_struct *vma)
{
	struct cmmb_dev *cam = vma->vm_private_data;
	/*
	 * Locking: done under mmap_sem, so we don't need to
	 * go back to the camera lock here.
	 */
	cam->mapcount++;
}


static void cmmb_v4l_vm_close(struct vm_area_struct *vma)
{
	struct cmmb_dev *cam = vma->vm_private_data;

	mutex_lock(&cam->s_mutex);
	cam->mapcount--;
	/* Docs say we should stop I/O too... */
	if (cam->mapcount == 0)
		cam->v4lbuf.flags &= ~V4L2_BUF_FLAG_MAPPED;
	mutex_unlock(&cam->s_mutex);
}

static struct vm_operations_struct cmmb_v4l_vm_ops = {
	.open = cmmb_v4l_vm_open,
	.close = cmmb_v4l_vm_close
};


static int cmmb_v4l_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct cmmb_dev *cam = filp->private_data;
	int ret = -EINVAL;

	/* if (! (vma->vm_flags & VM_WRITE) || ! (vma->vm_flags & VM_SHARED)) */
	/* if (! (vma->vm_flags & VM_SHARED)) */
	/* 	return -EINVAL; */
	/*
	 * Find the buffer they are looking for.
	 */
	mutex_lock(&cam->s_mutex);

	ret = remap_pfn_range(vma, vma->vm_start, cam->dma_handles >> PAGE_SHIFT, dma_buf_size, pgprot_val(PAGE_SHARED));
	if (ret)
		goto out;
	vma->vm_flags |= VM_DONTEXPAND;
	vma->vm_private_data = cam;
	vma->vm_ops = &cmmb_v4l_vm_ops;
	cam->v4lbuf.flags |= V4L2_BUF_FLAG_MAPPED;
	cmmb_v4l_vm_open(vma);
	ret = 0;
out:
	mutex_unlock(&cam->s_mutex);
	return ret;
}



static int cmmb_v4l_open(struct file *filp)
{
	struct cmmb_dev *cam;
	struct cmmb_platform_data *pdata;

	cam = cmmb_find_dev(video_devdata(filp)->minor);
	if (cam == NULL)
		return -ENODEV;

	pdata = cam->spi->dev.platform_data;

	if (pdata->power_on)
		pdata->power_on();

	filp->private_data = cam;

	mutex_lock(&cam->s_mutex);
	(cam->users)++;
	mutex_unlock(&cam->s_mutex);
	return 0;
}


static int cmmb_v4l_release(struct file *filp)
{
	struct cmmb_dev *cam = filp->private_data;
	struct cmmb_platform_data *pdata;

	mutex_lock(&cam->s_mutex);
	(cam->users)--;
	if (cam->users == 0) {
		if (alloc_bufs_at_read)
			cmmb_free_dma_bufs(cam);
	}
	mutex_unlock(&cam->s_mutex);

	pdata = cam->spi->dev.platform_data;

	if (pdata->power_off)
		pdata->power_off();

	return 0;
}

static unsigned int cmmb_v4l_poll(struct file *filp,
		struct poll_table_struct *pt)
{
	struct cmmb_dev *cam = filp->private_data;
	int frame = 0;

	clear_bit(frame, &cam->flags);

	interruptible_sleep_on(&cam->iowait);

	return POLLIN | POLLRDNORM;
}

static void cmmb_v4l_dev_release(struct video_device *vd)
{
	struct cmmb_dev *cam = container_of(vd, struct cmmb_dev, v4ldev);

	kfree(cam);
}


/*
 * This template device holds all of those v4l2 methods; we
 * clone it for specific real devices.
 */

static const struct v4l2_file_operations cmmb_v4l_fops = {
	.owner = THIS_MODULE,
	.open = cmmb_v4l_open,
	.release = cmmb_v4l_release,
	.poll = cmmb_v4l_poll,
	.mmap = cmmb_v4l_mmap,
	.ioctl = video_ioctl2,
};

static const struct v4l2_ioctl_ops cmmb_v4l_ioctl_ops = {
	.vidioc_try_fmt_vid_cap	= cmmb_vidioc_try_fmt_cap,
	.vidioc_querybuf	= cmmb_vidioc_querybuf,
	.vidioc_g_ctrl		= cmmb_vidioc_g_ctrl,
	.vidioc_s_ctrl		= cmmb_vidioc_s_ctrl,
	.vidioc_g_ext_ctrls	= cmmb_vidioc_g_ext_ctrl,
};

static struct video_device cmmb_v4l_template = {
	.name = "cmmb-module",
	.minor = -1, /* Get one dynamically */
	.tvnorms = V4L2_STD_NTSC_M,
	/* .current_norm = V4L2_STD_NTSC_M, */  /* make mplayer happy */

	.fops = &cmmb_v4l_fops,
	.ioctl_ops = &cmmb_v4l_ioctl_ops,
	.release = cmmb_v4l_dev_release,
};


/* ---------------------------------------------------------------------- */
/*
 * Interrupt handler stuff
 */

static irqreturn_t cmmb_irq(int irq, void *data)
{
	struct cmmb_dev *cam = data;
	int frame = 0;

	spin_lock(&cam->dev_lock);
	/*
	 * Basic frame housekeeping.
	 */
	if (test_bit(frame, &cam->flags) && printk_ratelimit())
		printk("Frame overrun on %d, frames lost\n", frame);
	set_bit(frame, &cam->flags);

	wake_up_interruptible(&cam->iowait);

	spin_unlock(&cam->dev_lock);
	return IRQ_HANDLED;
}

static int __devinit cmmb_probe(struct spi_device *spi)
{
	struct cmmb_platform_data *pdata;
	struct cmmb_dev *cam;
	int ret;
	u8 byte;

	pdata = spi->dev.platform_data;
	if (!pdata || !pdata->power_on)
		return -ENODEV;
	pdata->power_on();

	/*
	 * bits_per_word cannot be configured in platform data
	 */
	spi->bits_per_word = 8;

	ret = spi_setup(spi);
	if (ret < 0)
		goto out;

#if 1
	/*
	 * SPI test
	 */
	byte = 0x01;
	spi_write(spi, (const u8 *)&byte, sizeof(byte));
	spi_read(spi, (u8 *)&byte, sizeof(byte));
	if (byte != 0x42) {
		printk(KERN_NOTICE "CMMB Demodulator can't be detected\n");
		ret = -EINVAL;
		goto out;
	}
#endif

	/*
	 * Start putting together one of our big cmmb device structures.
	 */
	ret = -ENOMEM;
	cam = kzalloc(sizeof(struct cmmb_dev), GFP_KERNEL);
	if (cam == NULL)
		goto out;

	cam->spi = spi;

	mutex_init(&cam->s_mutex);
	mutex_lock(&cam->s_mutex);

	spin_lock_init(&cam->dev_lock);
	INIT_LIST_HEAD(&cam->dev_list);
	init_waitqueue_head(&cam->iowait);

	ret = request_irq(spi->irq, cmmb_irq, IRQF_TRIGGER_FALLING,
			"CMMB Demodulator", cam);
	if (ret)
		goto out_free;

	/*
	 * Get the v4l2 setup done.
	 */
	cam->v4ldev = cmmb_v4l_template;
	cam->v4ldev.debug = 0;
	ret = video_register_device(&cam->v4ldev, VFL_TYPE_GRABBER, -1);
	if (ret)
		goto out_free;
	/*
	 * If so requested, try to get our DMA buffers now.
	 */
	if (!alloc_bufs_at_read) {
		if (cmmb_alloc_dma_bufs(cam))
			printk("Unable to alloc DMA buffers at load"
					" will try again later.");
	}

	mutex_unlock(&cam->s_mutex);
	cmmb_add_dev(cam);

	if (pdata->power_off)
		pdata->power_off();

	printk(KERN_NOTICE "CMMB Demodulator detected\n");

	return ret;

out_free:
	kfree(cam);
out:
	return ret;
}

static int cmmb_remove(struct spi_device *spi)
{
	struct cmmb_dev *cam = cmmb_find_by_spi(spi);

	if (cam == NULL) {
		printk(KERN_WARNING "cmmb_remove on unknown spi %p\n", spi);
		return -ENODEV;
	}
	mutex_lock(&cam->s_mutex);
	if (cam->users > 0)
		printk("Removing a device with users!\n");

	cmmb_remove_dev(cam);
/* No unlock - it no longer exists */
	return 0;
}

static struct spi_driver cmmb_driver = {
	.driver = {
		.name		= DRIVER_NAME,
		.owner		= THIS_MODULE,
	},
	.probe		= cmmb_probe,
	.remove		= __devexit_p(cmmb_remove),
};

/*
 * Module initialization
 */

static int __init cmmb_module_init(void)
{
	printk(KERN_NOTICE "CMMB Demodulator driver, at your service\n");

	return spi_register_driver(&cmmb_driver);
}

static void __exit cmmb_module_exit(void)
{
	spi_unregister_driver(&cmmb_driver);
}

module_init(cmmb_module_init);
module_exit(cmmb_module_exit);
