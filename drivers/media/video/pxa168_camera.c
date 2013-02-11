/*
 *  linux/drivers/media/video/pxa910_camera.c - PXA9XX MMCI driver
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
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/videodev2.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-common.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-device.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/list.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/jiffies.h>
#include <linux/slab.h>

#include <linux/vmalloc.h>
#include <linux/platform_device.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/cacheflush.h>

#include <linux/clk.h>
#include "pxa168_camera.h"
#include <mach/pxa168.h>

#include <mach/camera.h>
#include <mach/regs-apmu.h>
#include <asm/mach-types.h>
#include <mach/cputype.h>


#if defined(CONFIG_DVFM)
#include <mach/dvfm.h>
static int dvfm_dev_idx;
#endif

#define CCIC_VERSION 0x000001
/*
 * Parameters.
 */
MODULE_AUTHOR("Jonathan Corbet <corbet@lwn.net>");
MODULE_DESCRIPTION("Marvell 88ALP01 CMOS Camera Controller driver");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("Video");

/*
 * Internal DMA buffer management.  Since the controller cannot do S/G
 * I/O,
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

#define DMA_POOL 0
#define MAX_DMA_BUFS 3
/* max dma buffer can be access by user point*/
#define DMA_TO_USER_POINT     10

static int alloc_bufs_at_read = 0;
module_param(alloc_bufs_at_read, bool, 0444);
MODULE_PARM_DESC(alloc_bufs_at_read,
		"Non-zero value causes DMA buffers to be allocated when the "
		"video capture device is read, rather than at module load "
		"time.  This saves memory, but decreases the chances of "
		"successfully getting those buffers.");

static int n_dma_bufs = 3;	//3 frame buffers
module_param(n_dma_bufs, uint, 0644);
MODULE_PARM_DESC(n_dma_bufs,
		"The number of DMA buffers to allocate.  Can be either two "
		"(saves memory, makes timing tighter) or three.");

//static int dma_buf_size = 2048 * 1536 * 2;  /* Worst case */
static int dma_buf_size = VGA_WIDTH * VGA_HEIGHT * 2;  /* Worst case */
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

static int flip = 0;
module_param(flip, bool, 0444);
MODULE_PARM_DESC(flip,
		"If set, the sensor will be instructed to flip the image "
		"vertically.");


enum ccic_state {
	S_NOTREADY,	/* Not yet initialized */
	S_IDLE,		/* Just hanging around */
	S_FLAKED,	/* Some sort of problem */
	S_SINGLEREAD,	/* In read() */
	S_SPECREAD,   	/* Speculative read (for future read()) */
	S_STREAMING	/* Streaming data */
};

struct yuv_pointer_t {
	dma_addr_t y;
	dma_addr_t u;
	dma_addr_t v;
};

/*
 * Tracking of streaming I/O buffers.
 */
struct ccic_sio_buffer {
	struct list_head list;
	struct v4l2_buffer v4lbuf;
	char *buffer;   /* Where it lives in kernel space */
	int mapcount;
	struct ccic_camera *cam;
	struct vm_area_struct *svma;
	dma_addr_t dma_handles;
	struct yuv_pointer_t yuv_p;
};

/*
 * A description of one of our devices.
 * Locking: controlled by s_mutex.  Certain fields, however, require
 * 	    the dev_lock spinlock; they are marked as such by comments.
 *	    dev_lock is also required for access to device registers.
 */
struct ccic_camera
{
	enum ccic_state state;
	unsigned long flags;   		/* Buffer status, mainly (dev_lock) */
	int users;			/* How many open FDs */
	struct file *owner;		/* Who has data access (v4l2) */

	/*
	 * Subsystem structures.
	 */
	int irq;
	struct platform_device *pdev;
	struct video_device v4ldev;
	struct i2c_adapter i2c_adapter;
	struct i2c_client *sensor;
	struct i2c_client *sensors[SENSOR_MAX];
	unsigned int bus_type[SENSOR_MAX];	/* parrallel or MIPI */

	unsigned char __iomem *regs;
	struct list_head dev_list;	/* link to other devices */

	/* DMA buffers */
	unsigned int nbufs;		/* How many are alloc'd */
	int next_buf;			/* Next to consume (dev_lock) */
	unsigned int dma_buf_size;  	/* allocated size */
	int order[MAX_DMA_BUFS];	/* Internal buffer addresses */
	void *dma_bufs[MAX_DMA_BUFS];	/* Internal buffer addresses */
	dma_addr_t dma_handles[MAX_DMA_BUFS]; /* Buffer bus addresses */
	void *rubbish_buf_virt;
	dma_addr_t rubbish_buf_phy;
	void *dma_bufs_user_pt[DMA_TO_USER_POINT];
	unsigned int specframes;	/* Unconsumed spec frames (dev_lock) */
	unsigned int sequence;		/* Frame sequence number */
	/* Sequence for individual buffers */
	unsigned int buf_seq[MAX_DMA_BUFS];

	/* Streaming buffers */
	unsigned int n_sbufs;		/* How many we have */
	struct ccic_sio_buffer *sb_bufs; /* The array of housekeeping
								structs */
	struct list_head sb_avail;	/* Available for data (we own)
								(dev_lock) */
	struct list_head sb_full;	/* With data (user space owns)
								(dev_lock) */
	struct list_head sb_dma;	/* dma list (dev_lock) */
	struct tasklet_struct s_tasklet;

	/* Current operating parameters */
	u32 sensor_type;		/* Currently ov7670 only */
	struct v4l2_pix_format pix_format;

	/* Locks */
	struct mutex s_mutex; /* Access to this structure */
	spinlock_t dev_lock;  /* Access to device */

	/* Misc */
	wait_queue_head_t iowait;	/* Waiting on frame data */
#ifdef CONFIG_VIDEO_ADV_DEBUG
	struct dentry *dfs_regs;
	struct dentry *dfs_cam_regs;
#endif
	unsigned long io_type;
	unsigned int n_map_bufs;	/* How many buffer from user point*/

};

#define BUS_PARALLEL 0x01
#define BUS_MIPI 0x02
#define BUS_IS_PARALLEL(b) (b & BUS_PARALLEL)
#define BUS_IS_MIPI(b) (b & BUS_MIPI)

/*
 * Status flags.  Always manipulated with bit operations.
 */
#define CF_BUF0_VALID	 0	/* Buffers valid - first three */
#define CF_BUF1_VALID	 1
#define CF_BUF2_VALID	 2
#define CF_DMA_ACTIVE	 3	/* A frame is incoming */
#define CF_CONFIG_NEEDED 4	/* Must configure hardware */

static int detected_high = 0;
static int detected_low = 0;
static int sensor_selected = 1;
static int sof, jpeg_cnt, lines;

struct clk *rst_clk;
struct clk *pxa168_ccic_gate_clk;
spinlock_t reg_lock;
EXPORT_SYMBOL(pxa168_ccic_gate_clk);

static int ccic_enable_clk(struct ccic_camera *cam,
					struct sensor_platform_data *pdata);

/*
 * Start over with DMA buffers - dev_lock needed.
 */
static void ccic_reset_buffers(struct ccic_camera *cam)
{
	int i;

	cam->next_buf = -1;
	for (i = 0; i < cam->nbufs; i++)
		clear_bit(i, &cam->flags);
	cam->specframes = 0;
}

static inline int ccic_needs_config(struct ccic_camera *cam)
{
	return test_bit(CF_CONFIG_NEEDED, &cam->flags);
}

static void ccic_set_config_needed(struct ccic_camera *cam, int needed)
{
	if (needed)
		set_bit(CF_CONFIG_NEEDED, &cam->flags);
	else
		clear_bit(CF_CONFIG_NEEDED, &cam->flags);
}




/*
 * Debugging and related.
 */
#define cam_err(cam, fmt, arg...) \
	dev_err(&(cam)->pdev->dev, fmt, ##arg);
#define cam_warn(cam, fmt, arg...) \
	dev_warn(&(cam)->pdev->dev, fmt, ##arg);
#define cam_dbg(cam, fmt, arg...) \
	dev_dbg(&(cam)->pdev->dev, fmt, ##arg);


/* -------------------------------------------------------------------*/
/*
 * We keep a simple list of known devices to search at open time.
 */
static LIST_HEAD(ccic_dev_list);
static DEFINE_MUTEX(ccic_dev_list_lock);
static int __ccic_cam_cmd(struct ccic_camera *cam, int cmd, void *arg);
static int __ccic_cam_reset(struct ccic_camera *cam);

static void ccic_add_dev(struct ccic_camera *cam)
{
	mutex_lock(&ccic_dev_list_lock);
	list_add_tail(&cam->dev_list, &ccic_dev_list);
	mutex_unlock(&ccic_dev_list_lock);
}

static void ccic_remove_dev(struct ccic_camera *cam)
{
	mutex_lock(&ccic_dev_list_lock);
	list_del(&cam->dev_list);
	mutex_unlock(&ccic_dev_list_lock);
}

static struct ccic_camera *ccic_find_dev(int minor)
{
	struct ccic_camera *cam;

	mutex_lock(&ccic_dev_list_lock);
	list_for_each_entry(cam, &ccic_dev_list, dev_list) {
		if (cam->v4ldev.minor == minor)
			goto done;
	}
	cam = NULL;
  done:
	mutex_unlock(&ccic_dev_list_lock);
	return cam;
}


static struct ccic_camera *ccic_find_by_pdev(
						struct platform_device *pdev)
{
	struct ccic_camera *cam;

	mutex_lock(&ccic_dev_list_lock);
	list_for_each_entry(cam, &ccic_dev_list, dev_list) {
		if (cam->pdev == pdev)
			goto done;
	}
	cam = NULL;
  done:
	mutex_unlock(&ccic_dev_list_lock);
	return cam;
}


/* ----------------------------------------------------------------- */
/*
 * Device register I/O
 */
static inline void ccic_reg_write(struct ccic_camera *cam, unsigned int reg,
		unsigned int val)
{
	__raw_writel(val, cam->regs + reg);
}

static inline unsigned int ccic_reg_read(struct ccic_camera *cam,
		unsigned int reg)
{
	return __raw_readl(cam->regs + reg);
}


static inline void ccic_reg_write_mask(struct ccic_camera *cam,
			unsigned int reg, unsigned int val, unsigned int mask)
{
	unsigned int v = ccic_reg_read(cam, reg);

	v = (v & ~mask) | (val & mask);
	ccic_reg_write(cam, reg, v);
}

static inline void ccic_reg_clear_bit(struct ccic_camera *cam,
		unsigned int reg, unsigned int val)
{
	ccic_reg_write_mask(cam, reg, 0, val);
}

static inline void ccic_reg_set_bit(struct ccic_camera *cam,
		unsigned int reg, unsigned int val)
{
	ccic_reg_write_mask(cam, reg, val, val);
}
/* only for debug */
#ifdef DEBUG_DUMP
static int dump_register(struct ccic_camera *cam)
{
	unsigned int irqs;
	spin_lock(&cam->dev_lock);
	irqs = ccic_reg_read(cam, REG_IRQSTAT);
	printk(KERN_INFO"CCIC: REG_IRQSTAT is %x\n", irqs);
	irqs = ccic_reg_read(cam, REG_IRQSTATRAW);
	printk(KERN_INFO"CCIC: REG_IRQSTATRAW is %x\n", irqs);
	irqs = ccic_reg_read(cam, REG_IRQMASK);
	printk(KERN_INFO"CCIC: REG_IRQMASK is %x\n\n", irqs);
	irqs = ccic_reg_read(cam, REG_IMGPITCH);
	printk(KERN_INFO"CCIC: REG_IMGPITCH is %x\n", irqs);
	irqs = ccic_reg_read(cam, REG_IMGSIZE);
	printk(KERN_INFO"CCIC: REG_IMGSIZE is %x\n", irqs);
	irqs = ccic_reg_read(cam, REG_IMGOFFSET);
	printk(KERN_INFO"CCIC: REG_IMGOFFSET is %x\n\n", irqs);
	irqs = ccic_reg_read(cam, REG_CTRL0);
	printk(KERN_INFO"CCIC: REG_CTRL0 is %x\n", irqs);
	irqs = ccic_reg_read(cam, REG_CTRL1);
	printk(KERN_INFO"CCIC: REG_CTRL1 is %x\n", irqs);
	irqs = ccic_reg_read(cam, REG_CLKCTRL);
	printk(KERN_INFO"CCIC: REG_CLKCTRL is %x\n\n", irqs);
	irqs = ccic_reg_read(cam, REG_CSI2_DPHY3);
	printk(KERN_INFO"CCIC: REG_CSI2_DPHY3 is %x\n", irqs);
	irqs = ccic_reg_read(cam, REG_CSI2_DPHY5);
	printk(KERN_INFO"CCIC: REG_CSI2_DPHY5 is %x\n\n", irqs);
	irqs = ccic_reg_read(cam, REG_CSI2_DPHY6);
	printk(KERN_INFO"CCIC: REG_CSI2_DPHY6 is %x\n\n", irqs);
	irqs = ccic_reg_read(cam, REG_CSI2_CTRL0);
	printk(KERN_INFO"CCIC: REG_CSI2_CTRL0 is %x\n\n", irqs);
	irqs = ccic_reg_read(cam, REG_Y0BAR);
	printk(KERN_ERR"REG_Y0BAR 0x%08x\n", irqs);
	irqs = ccic_reg_read(cam, REG_Y0BAR + 4);
	printk(KERN_ERR"REG_Y0BAR 0x%08x\n", irqs);
	irqs = ccic_reg_read(cam, REG_Y0BAR + 8);
	printk(KERN_ERR"REG_Y0BAR 0x%08x\n", irqs);
	spin_unlock(&cam->dev_lock);

	return 0;
}
#endif

/*provided for sensor calling to enable clock at begining of probe*/
void ccic_set_clock(unsigned int reg, unsigned int val)
{
	struct ccic_camera *cam;
	cam = ccic_find_dev(0);
	ccic_reg_write(cam, reg, val);
}

unsigned int ccic_get_clock(unsigned int reg)
{
        struct ccic_camera *cam;
        cam = ccic_find_dev(0);
	return ccic_reg_read(cam, reg);
}
/*
1. APMU CCIC clock reset control
2. CCIC power on
3. enable MCLK
*/
void ccic_set_clock_mipi(void)
{
	unsigned long flags;

	clk_set_rate(rst_clk, 0x6abf);
	spin_lock_irqsave(&reg_lock, flags);
	//enable MIPI by setting bit 25/26 of apmu_debug register
	__raw_writel(0x06000000 | __raw_readl(APMU_CCIC_DBG),
								APMU_CCIC_DBG);
	spin_unlock_irqrestore(&reg_lock, flags);

	clk_enable(pxa168_ccic_gate_clk);

	/*
	* workaround for stress streamon/streamoff:
	* CCIC_CTRL_1[30] (DMAREQCTRL) should be set during capture
	* to avoid DMA deadly waiting in case of two SOF without EOF
	*  between them.
	*/
		/*CCIC power on. bit28 (PWRDNEN) must be cleared.*/
	ccic_set_clock(REG_CTRL1, 0x4a00003c);
	ccic_set_clock(REG_CLKCTRL, (0x2<<29 | 0xc));	/*enable MCLK*/
}

void ccic_set_clock_parallel(void)
{
	clk_set_rate(rst_clk, 0xbff);
	clk_enable(pxa168_ccic_gate_clk);
	if (machine_is_aspenite() || machine_is_ttc_dkb()) {
		/*CCIC power on. bit28 (PWRDNEN) must be cleared.*/
		ccic_set_clock(REG_CTRL1, 0x0c00003c);
		ccic_set_clock(REG_CLKCTRL, 0x40000004); /*enable MCLK */
		ccic_set_clock(0x1ec, 0x00004);	/*undocumented register???*/
	} else if (machine_is_avengers_lite() || machine_is_ipcam()) {
		clk_set_rate(rst_clk, 0x5b);
		clk_enable(pxa168_ccic_gate_clk);

		ccic_set_clock(REG_CTRL1, 0x4c00003c);
		ccic_set_clock(REG_CLKCTRL, 0x40000004);/* enable MCLK */
		ccic_set_clock(0x1ec, 0x00004);	/* undocumented register??? */
	} else {
                clk_set_rate(rst_clk, 0x5b);
		/*CCIC power on. bit28 (PWRDNEN) must be cleared.*/
		ccic_set_clock(REG_CTRL1, 0x4c00003c);
		ccic_set_clock(REG_CLKCTRL, 0x2);	/*enable MCLK */
	}
}
EXPORT_SYMBOL (ccic_set_clock_parallel);

void ccic_disable_clock(void)
{
	unsigned long flags;

	clk_set_rate(rst_clk, 0x6800);
	ccic_set_clock(REG_CLKCTRL, 0x0);
	clk_set_rate(pxa168_ccic_gate_clk, 0x0);
	ccic_set_clock(REG_CTRL1, 0x0);
	/* disable MIPI */
	spin_lock_irqsave(&reg_lock, flags);
	__raw_writel((~0x06000000) & __raw_readl(APMU_CCIC_DBG), APMU_CCIC_DBG);
	spin_unlock_irqrestore(&reg_lock, flags);
}
EXPORT_SYMBOL (ccic_disable_clock);

void ccic_disable_mclk(void)
{
	ccic_set_clock(REG_CLKCTRL, 0x0);	
}

void ccic_enable_vclk(void)
{
	ccic_set_clock(REG_CLKCTRL, 0x40000004);
}
EXPORT_SYMBOL(ccic_enable_vclk);

void ccic_enable_mclk(struct ccic_camera *cam)
{
	if (BUS_IS_MIPI(cam->bus_type[sensor_selected])){
		ccic_set_clock(REG_CLKCTRL, (0x2<<29 | 0xc));
	} else if (sensor_selected == SENSOR_LOW) {
		if (machine_is_avengers_lite() || machine_is_ipcam())
			ccic_set_clock(REG_CLKCTRL, 0x40000004);
		else
			ccic_set_clock(REG_CLKCTRL, 0x2);
	}
}

/* Somebody is on the bus */
static void ccic_ctlr_stop_dma(struct ccic_camera *cam);
static void ccic_ctlr_power_down(struct ccic_camera *cam);

int ccic_sensor_attach(struct i2c_client *client)
{
	struct ccic_camera *cam;
	int ret;
	struct v4l2_dbg_chip_ident chip;

	cam = ccic_find_dev(0);	/*TODO - only support one camera controller */
	if (cam == NULL){
		printk("didn't find camera device!\n");
		return -ENODEV;
	}
	/*
	 * Don't talk to chips we don't recognize.
	 */

        mutex_lock(&cam->s_mutex);
	cam->sensor = client;
        ret = __ccic_cam_reset(cam);
        if (ret)
                goto out;
	chip.match.type = V4L2_CHIP_MATCH_I2C_ADDR;
	chip.match.addr = cam->sensor->addr;
        chip.ident = V4L2_IDENT_NONE;
	ret = __ccic_cam_cmd(cam, VIDIOC_DBG_G_CHIP_IDENT, &chip);
        if (ret)
                goto out;
        cam->sensor_type = chip.ident;

	if ((cam->sensor_type == V4L2_IDENT_OV7660) ||
		(cam->sensor_type == V4L2_IDENT_OV7670) ||
		(cam->sensor_type == V4L2_IDENT_SIV120A)) {
		cam->sensors[SENSOR_LOW] = client;
		cam->bus_type[SENSOR_LOW] = BUS_PARALLEL;
		detected_low = 1;
		sensor_selected = SENSOR_LOW; /* Set default sensor */
	} else if (cam->sensor_type == V4L2_IDENT_OV3640 ||
				cam->sensor_type == V4L2_IDENT_CLI5001) {
                cam->sensors[SENSOR_HIGH] = client;
		cam->bus_type[SENSOR_HIGH] = BUS_PARALLEL;
                detected_high = 1;
		if (cam->sensor_type == V4L2_IDENT_OV3640)
			cam->bus_type[SENSOR_HIGH] = BUS_MIPI;
		sensor_selected = SENSOR_HIGH;	/* Set default sensor */
	} else if (cam->sensor_type == V4L2_IDENT_OV7740) {
		cam->sensors[SENSOR_HIGH] = client;
		detected_high = 1;
		sensor_selected = SENSOR_HIGH;	/* Set default sensor */
		printk(KERN_ERR "camera: ov7740 attached\n");
	} else {
		cam_err(cam, "Unsupported sensor type %d at addr 0x%x",
					cam->sensor_type, cam->sensor->addr);
                ret = -EINVAL;
                goto out;
	}
/* Get/set parameters? */
        ret = 0;
        cam->state = S_IDLE;
  out:
        ccic_ctlr_power_down(cam);
	cam->sensor = NULL;
        mutex_unlock(&cam->s_mutex);
        return ret;

}
EXPORT_SYMBOL (ccic_sensor_attach);

#if 0
static int ccic_sensor_detach(struct i2c_client *client)
{
	struct ccic_camera *cam;

	cam = ccic_find_dev(0);
	if (cam == NULL)
		return -ENODEV;

	if (cam->sensor == client) {
		ccic_ctlr_stop_dma(cam);
		ccic_ctlr_power_down(cam);
		cam_err(cam, "lost the sensor!\n");
		cam->sensor = NULL;  /* Bummer, no camera */
		cam->state = S_NOTREADY;
	}
	return 0;
}
#endif

/* ------------------------------------------------------------------- */
/*
 * Deal with the controller.
 */
static void ccic_switch_dma(struct ccic_camera *cam, int frame)
{
	struct ccic_sio_buffer *sbuf, *newsbuf;
	unsigned long flags;
	dma_addr_t phy;
	clear_bit(frame, &cam->flags);
	/*
	 * if the buffer is dequeued, try to fetch one buffer from avail list
	 * if the buffer is in avail list, move form avail list to full list,
	 *	dequeue the buffer at the same time to protect
	 */

	phy = ccic_reg_read(cam, REG_Y0BAR + (frame<<2));
	/* rubbish got data */
	if (cam->rubbish_buf_phy == phy) {
		/* the buffer is dequeued */
		if (list_empty(&cam->sb_avail))
			goto out;

		sbuf = list_entry(cam->sb_avail.next,
				struct ccic_sio_buffer, list);
		spin_lock_irqsave(&cam->dev_lock, flags);
		ccic_reg_write(cam, REG_Y0BAR + (frame<<2), sbuf->yuv_p.y);
		ccic_reg_write(cam, REG_U0BAR + (frame<<2), sbuf->yuv_p.u);
		ccic_reg_write(cam, REG_V0BAR + (frame<<2), sbuf->yuv_p.v);
		list_move_tail(&sbuf->list, &cam->sb_dma);
		spin_unlock_irqrestore(&cam->dev_lock, flags);

	} else {  /* got the real data */
		list_for_each_entry(sbuf, &cam->sb_dma, list) {
			if (phy == sbuf->dma_handles) { /*find the match sbuf*/
				/*no more dma buffer*/
				if (list_empty(&cam->sb_avail)) {
					spin_lock_irqsave(&cam->dev_lock,
									flags);
					printk(KERN_DEBUG "CCIC link to rubbish\
								buffer\n");
					ccic_reg_write(cam, REG_Y0BAR +
					(frame<<2), cam->rubbish_buf_phy);
					ccic_reg_write(cam, REG_U0BAR +
					(frame<<2), cam->rubbish_buf_phy);
					ccic_reg_write(cam, REG_V0BAR +
					(frame<<2), cam->rubbish_buf_phy);
					spin_unlock_irqrestore(&cam->dev_lock,
									flags);

				} else {
					newsbuf = list_entry(cam->sb_avail.next,
					struct ccic_sio_buffer, list);
					spin_lock_irqsave(&cam->dev_lock,
									flags);
					ccic_reg_write(cam, REG_Y0BAR +
						(frame<<2), newsbuf->yuv_p.y);
					ccic_reg_write(cam, REG_U0BAR +
						(frame<<2), newsbuf->yuv_p.u);
					ccic_reg_write(cam, REG_V0BAR +
						(frame<<2), newsbuf->yuv_p.v);
					list_move_tail(&newsbuf->list,
								&cam->sb_dma);
					spin_unlock_irqrestore(&cam->dev_lock,
									flags);

				}
				dma_sync_single_for_device(&cam->pdev->dev,
						sbuf->dma_handles,
						sbuf->v4lbuf.length,
						DMA_FROM_DEVICE);

				if ((cam->pix_format.pixelformat ==
					V4L2_PIX_FMT_JPEG) && ((((char *)
					cam->dma_bufs_user_pt[frame])[0]
									!= 0xff)
					|| (((char *)
						cam->dma_bufs_user_pt[frame])
								[1] != 0xd8))) {
					printk(KERN_ERR"%s: JPEG ERROR !!! \
					dropped this frame\n",	__func__);
					goto out;
				}

				sbuf->v4lbuf.sequence = cam->buf_seq[frame];
				sbuf->v4lbuf.bytesused =
						cam->pix_format.sizeimage;
				sbuf->v4lbuf.flags &= ~V4L2_BUF_FLAG_QUEUED;
				sbuf->v4lbuf.flags |= V4L2_BUF_FLAG_DONE;

				cam->next_buf = frame;
				list_move_tail(&sbuf->list, &cam->sb_full);
				if (!list_empty(&cam->sb_full)) {
					clear_bit(CF_DMA_ACTIVE, &cam->flags);
					wake_up(&cam->iowait);
				}

				goto out;
			}
		}

	}
out:
	return;
}

static int ccic_ctlr_dma_mmap(struct ccic_camera *cam)
{
	struct v4l2_pix_format *fmt = &cam->pix_format;

	ccic_reg_write(cam, REG_Y0BAR, cam->dma_handles[0]);
	ccic_reg_write(cam, REG_Y1BAR, cam->dma_handles[1]);

	if (fmt->pixelformat == V4L2_PIX_FMT_YUV422P) {
		ccic_reg_write(cam, REG_U0BAR, cam->dma_handles[0] +
							fmt->width*fmt->height);
		ccic_reg_write(cam, REG_V0BAR, cam->dma_handles[0] +
			fmt->width*fmt->height + fmt->width*fmt->height/2);
		ccic_reg_write(cam, REG_U1BAR, cam->dma_handles[1] +
							fmt->width*fmt->height);
		ccic_reg_write(cam, REG_V1BAR, cam->dma_handles[1] +
			fmt->width*fmt->height + fmt->width*fmt->height/2);
	}
        if (fmt->pixelformat == V4L2_PIX_FMT_YUV420) {
		ccic_reg_write(cam, REG_U0BAR, cam->dma_handles[0] +
							fmt->width*fmt->height);
		ccic_reg_write(cam, REG_V0BAR, cam->dma_handles[0] +
			fmt->width*fmt->height + fmt->width*fmt->height/4);
		ccic_reg_write(cam, REG_U1BAR, cam->dma_handles[1] +
							fmt->width*fmt->height);
		ccic_reg_write(cam, REG_V1BAR, cam->dma_handles[1] +
			fmt->width*fmt->height + fmt->width*fmt->height/4);
        }

	/*
	* Store the first two Y buffers (we aren't supporting
	* planar formats for now, so no UV bufs).  Then either
	* set the third if it exists, or tell the controller
	* to just use two.
	*/
	if (cam->nbufs > 2) {
		ccic_reg_write(cam, REG_Y2BAR, cam->dma_handles[2]);
		if (fmt->pixelformat == V4L2_PIX_FMT_YUV422P) {
			ccic_reg_write(cam, REG_U2BAR, cam->dma_handles[2] +
							fmt->width*fmt->height);
			ccic_reg_write(cam, REG_V2BAR, cam->dma_handles[2] +
			fmt->width*fmt->height + fmt->width*fmt->height/2);
		}
                if (fmt->pixelformat == V4L2_PIX_FMT_YUV420) {
			ccic_reg_write(cam, REG_U2BAR, cam->dma_handles[2] +
							fmt->width*fmt->height);
			ccic_reg_write(cam, REG_V2BAR, cam->dma_handles[2] +
			fmt->width*fmt->height + fmt->width*fmt->height/4);
                }

		ccic_reg_clear_bit(cam, REG_CTRL1, C1_TWOBUFS);
	}
	else
		ccic_reg_set_bit(cam, REG_CTRL1, C1_TWOBUFS);

	return 0;
}

static int ccic_ctlr_dma(struct ccic_camera *cam)
{
	int frame = 0;
	unsigned long flags;
	struct ccic_sio_buffer *sbuf;
	if (cam->n_map_bufs < 2) {
		printk(KERN_ERR "ccic at least 2 dma buffers\n");
		return -ENOMEM;
	}
	spin_lock_irqsave(&cam->dev_lock, flags);
	for (frame = 0; frame < cam->n_map_bufs; frame++) {
		/* suppose only 3 buffers queued */
		if (MAX_DMA_BUFS == frame)
			break;
		sbuf = cam->sb_bufs + frame;
		ccic_reg_write(cam, REG_Y0BAR + (frame<<2), sbuf->yuv_p.y);
		ccic_reg_write(cam, REG_U0BAR + (frame<<2), sbuf->yuv_p.u);
		ccic_reg_write(cam, REG_V0BAR + (frame<<2), sbuf->yuv_p.v);

		list_move_tail(&sbuf->list, &cam->sb_dma);
	}
	spin_unlock_irqrestore(&cam->dev_lock, flags);
	if (cam->nbufs > 2)
		ccic_reg_clear_bit(cam, REG_CTRL1, C1_TWOBUFS);
	else
		ccic_reg_set_bit(cam, REG_CTRL1, C1_TWOBUFS);

	return 0;
}

static void ccic_ctlr_image(struct ccic_camera *cam)
{
	int imgsz;
	struct v4l2_pix_format *fmt = &cam->pix_format;
	int widthy = 0, widthuv = 0;

	if (fmt->pixelformat == V4L2_PIX_FMT_YUV420)
		imgsz = ((fmt->height << IMGSZ_V_SHIFT) & IMGSZ_V_MASK) |
				(((fmt->bytesperline)*4/3) & IMGSZ_H_MASK);
	else
		imgsz = ((fmt->height << IMGSZ_V_SHIFT) & IMGSZ_V_MASK) |
					(fmt->bytesperline & IMGSZ_H_MASK);
	printk("%s: CCIC input image size is %x\n", __func__, imgsz);
		/* YPITCH just drops the last two bits */
	//ccic_reg_write_mask(cam, REG_IMGPITCH, fmt->bytesperline,
	//		IMGP_YP_MASK);
	switch (fmt->pixelformat) {
	case V4L2_PIX_FMT_YUYV:
            widthy = fmt->width*2;
            widthuv = fmt->width*2;
	    break;
	case V4L2_PIX_FMT_RGB565:
	    widthy = fmt->width*2;
	    widthuv = 0;
	    break;
	case V4L2_PIX_FMT_JPEG:
		if (BUS_IS_MIPI(cam->bus_type[sensor_selected])){
			widthy = 0;
			widthuv = 0;
			imgsz = 0x1fff3fff;
		}else{/* same as 422pack for parallel */
			widthy = fmt->width*2;
			widthuv = fmt->width*2;
		}
		break; 
	case V4L2_PIX_FMT_YUV422P:
                widthy = fmt->width;
                widthuv = fmt->width/2;
		break;
	case V4L2_PIX_FMT_YUV420:
		widthy = fmt->width;
		widthuv = fmt->width/2;
		break;
	default:
		break;
	}
	ccic_reg_write(cam, REG_IMGPITCH, widthuv << 16 | widthy);
	ccic_reg_write(cam, REG_IMGSIZE, imgsz);
	ccic_reg_write(cam, REG_IMGOFFSET, 0x0);
/*
	 * Tell the controller about the image format we are using.
	 */
	switch (cam->pix_format.pixelformat) {
	case V4L2_PIX_FMT_YUV422P:
	    ccic_reg_write_mask(cam, REG_CTRL0,
			/*the endianness of sensor output is UYVY(Y1CrY0Cb)*/
			    C0_DF_YUV|C0_YUV_PLANAR|C0_YUVE_YVYU,
			    C0_DF_MASK);
	    break;
        case V4L2_PIX_FMT_YUV420:
            ccic_reg_write_mask(cam, REG_CTRL0,
			/*the endianness of sensor output is UYVY(Y1CrY0Cb)*/
				C0_DF_YUV|C0_YUV_420PL|C0_YUVE_YVYU,
				C0_DF_MASK);
            break;

	case V4L2_PIX_FMT_YUYV:
	    ccic_reg_write_mask(cam, REG_CTRL0,
		/*the endianness of sensor output is UYVY(Y1CrY0Cb)*/
		C0_DF_YUV|C0_YUV_PACKED|C0_YUVE_YUYV,
		C0_DF_MASK);
	    break;
	case V4L2_PIX_FMT_JPEG:	/*C0_YUV_PACKED must be set for JPEG?!*/
		if (BUS_IS_MIPI(cam->bus_type[sensor_selected])) {
			ccic_reg_write_mask(cam, REG_CTRL0,
			/* Set CTRL0 as 0x10a8 for JPEG */
			C0_DF_RGB|C0_RGB_BGR|C0_RGB4_BGRX,
			C0_DF_MASK);
		} else	{
		    ccic_reg_write_mask(cam, REG_CTRL0,
				    C0_DF_YUV|C0_YUV_PACKED|C0_YUVE_YUYV,
				    C0_DF_MASK);
	    }
	    break;

	case V4L2_PIX_FMT_RGB444:
	    ccic_reg_write_mask(cam, REG_CTRL0,
			    C0_DF_RGB|C0_RGBF_444|C0_RGB4_XRGB,
			    C0_DF_MASK);
		/* Alpha value? */
	    break;

	case V4L2_PIX_FMT_RGB565:
	    ccic_reg_write_mask(cam, REG_CTRL0,
			    C0_DF_RGB|C0_RGBF_565|C0_RGB5_BGGR,
			    C0_DF_MASK);
	    break;

	default:
	    cam_err(cam, "Unknown format %x\n", cam->pix_format.pixelformat);
	    break;
	}
	/*
	 * Make sure it knows we want to use hsync/vsync.
	 */
	ccic_reg_write_mask(cam, REG_CTRL0, C0_SIF_HVSYNC, C0_SIFM_MASK);
	printk("%s:REG_CTRL0 = %x\n ", __func__, ccic_get_clock(REG_CTRL0));
}


/*
 * Configure the controller for operation; caller holds the
 * device mutex.
 */
static int ccic_ctlr_configure(struct ccic_camera *cam)
{
	unsigned long flags;
	int ret = 0;
	spin_lock_irqsave(&cam->dev_lock, flags);
	if (V4L2_MEMORY_USERPTR == cam->io_type)
		ret = ccic_ctlr_dma(cam);
	else if (V4L2_MEMORY_MMAP == cam->io_type)
			ret = ccic_ctlr_dma_mmap(cam);
	ccic_ctlr_image(cam);
	ccic_set_config_needed(cam, 0);
	spin_unlock_irqrestore(&cam->dev_lock, flags);
	return ret;
}

static void ccic_ctlr_irq_enable(struct ccic_camera *cam)
{
	/*
	 * Clear any pending interrupts, since we do not
	 * expect to have I/O active prior to enabling.
	 */
	ccic_reg_write(cam, REG_IRQSTAT, FRAMEIRQS);
	ccic_reg_set_bit(cam, REG_IRQMASK, FRAMEIRQS);
}

static void ccic_ctlr_irq_disable(struct ccic_camera *cam)
{
	ccic_reg_clear_bit(cam, REG_IRQMASK, FRAMEIRQS);
}

/*
 * Make the controller start grabbing images.  Everything must
 * be set up before doing this.
 */
static void ccic_ctlr_start(struct ccic_camera *cam)
{
	/* set_bit performs a read, so no other barrier should be
	   needed here */
	ccic_reg_set_bit(cam, REG_CTRL0, C0_ENABLE);
}

static void ccic_ctlr_stop(struct ccic_camera *cam)
{
	ccic_reg_clear_bit(cam, REG_CTRL0, C0_ENABLE);
}

void ccic_ctlr_init(struct ccic_camera *cam)
{
	unsigned long flags;

	spin_lock_irqsave(&cam->dev_lock, flags);
	/*
	 * Make sure it's not powered down.
	 */
	ccic_reg_clear_bit(cam, REG_CTRL1, C1_PWRDWN);
	/*
	 * Turn off the enable bit.  It sure should be off anyway,
	 * but it's good to be sure.
	 */
	ccic_reg_clear_bit(cam, REG_CTRL0, C0_ENABLE);
	/*
	 * Mask all interrupts.
	 */
	ccic_reg_write(cam, REG_IRQMASK, 0);
	/*
	 * Clock the sensor appropriately.  Controller clock should
	 * be 48MHz, sensor "typical" value is half that.
	 */
	spin_unlock_irqrestore(&cam->dev_lock, flags);
}
/*
 * Stop the controller, and don't return until we're really sure that no
 * further DMA is going on.
 */
static void ccic_ctlr_stop_dma(struct ccic_camera *cam)
{
	unsigned long flags;

	/*
	 * Theory: stop the camera controller (whether it is operating
	 * or not).  Delay briefly just in case we race with the SOF
	 * interrupt, then wait until no DMA is active.
	 */
	wait_event_timeout(cam->iowait,
			!test_bit(CF_DMA_ACTIVE, &cam->flags), HZ/5);
	if (test_bit(CF_DMA_ACTIVE, &cam->flags))
		cam_err(cam, "Timeout waiting for DMA to end\n");
		/* This would be bad news - what now? */
	spin_lock_irqsave(&cam->dev_lock, flags);

	ccic_ctlr_stop(cam);
	/*CSI2/DPHY need to be cleared, or no EOF will be received*/
	ccic_reg_write(cam, REG_CSI2_DPHY3, 0x0);
	ccic_reg_write(cam, REG_CSI2_DPHY6, 0x0);
	ccic_reg_write(cam, REG_CSI2_DPHY5, 0x0);
	ccic_reg_write(cam, REG_CSI2_CTRL0, 0x0);

	cam->state = S_IDLE;
	ccic_ctlr_irq_disable(cam);
	spin_unlock_irqrestore(&cam->dev_lock, flags);
}

/*
 * Power up and down.
 */
void ccic_ctlr_power_up(struct ccic_camera *cam)
{
	unsigned long flags;

	/*
	 * Part one of the sensor dance: turn the global
	 * GPIO signal on.
	 */
	printk("%s: ccic is power up!!!\n", __func__);
	spin_lock_irqsave(&cam->dev_lock, flags);
	ccic_reg_clear_bit(cam, REG_CTRL1, C1_PWRDWN);
	spin_unlock_irqrestore(&cam->dev_lock, flags);
}

static void ccic_ctlr_power_down(struct ccic_camera *cam)
{
	unsigned long flags;
	printk("%s: ccic is power downed!!!\n", __func__);
	spin_lock_irqsave(&cam->dev_lock, flags);
	ccic_reg_set_bit(cam, REG_CTRL1, C1_PWRDWN);
	spin_unlock_irqrestore(&cam->dev_lock, flags);
}

/* -------------------------------------------------------------------- */
/*
 * Communications with the sensor.
 */

static int __ccic_cam_cmd(struct ccic_camera *cam, int cmd, void *arg)
{
	struct i2c_client *sc = cam->sensor;
	int ret;

	if (sc == NULL || sc->driver == NULL || sc->driver->command == NULL)
		return -EINVAL;
	ret = sc->driver->command(sc, cmd, arg);
	if (ret == -EPERM) /* Unsupported command */
		return 0;
	return ret;
}

static int __ccic_cam_reset(struct ccic_camera *cam)
{
	int zero = 0;
	return __ccic_cam_cmd(cam, VIDIOC_INT_RESET, &zero);
}

/*
 * We have found the sensor on the i2c.  Let's try to have a
 * conversation.
 */

static int ccic_cam_configure(struct ccic_camera *cam)
{
	struct v4l2_format fmt;
	int ret = 0;

	if (cam->state != S_IDLE)
		return -EINVAL;
	fmt.fmt.pix = cam->pix_format;
	ret = __ccic_cam_cmd(cam, VIDIOC_S_FMT, &fmt);

	/*
	 * OV7670 does weird things if flip is set *before* format...
	 */
//	ret += ccic_cam_set_flip(cam);		//TODO - no need to flip...
	return ret;
}

/* -------------------------------------------------------------------- */
/*
 * DMA buffer management.  These functions need s_mutex held.
 */

/* FIXME: this is inefficient as hell, since dma_alloc_coherent just
 * does a get_free_pages() call, and we waste a good chunk of an orderN
 * allocation.  Should try to allocate the whole set in one chunk.
 */
static int ccic_alloc_dma_bufs(struct ccic_camera *cam, int loadtime)
{
	int i;

	ccic_set_config_needed(cam, 1);
	if (loadtime)
		cam->dma_buf_size = dma_buf_size;
	else
		cam->dma_buf_size = cam->pix_format.sizeimage;
	if (n_dma_bufs > MAX_DMA_BUFS)
		n_dma_bufs = MAX_DMA_BUFS;

	cam->nbufs = 0;
	for (i = 0; i < n_dma_bufs; i++) {
#if DMA_POOL
		cam->dma_bufs[i] = dma_alloc_coherent(&cam->pdev->dev,
				cam->dma_buf_size, cam->dma_handles + i,
				GFP_KERNEL);
#else
		cam->order[i] = get_order(cam->dma_buf_size);
		cam->dma_bufs[i] = (void *)__get_free_pages
						(GFP_KERNEL, cam->order[i]);
		cam->dma_handles[i] = __pa(cam->dma_bufs[i]);
#endif
		if (cam->dma_bufs[i] == NULL) {
			cam_warn(cam, "Failed to allocate DMA buffer\n");
			break;
		}
		/* For debug, remove eventually */
		memset(cam->dma_bufs[i], 0xcc, cam->dma_buf_size);
		(cam->nbufs)++;
	}

	switch (cam->nbufs) {
	case 1:
#if DMA_POOL
	    dma_free_coherent(&cam->pdev->dev, cam->dma_buf_size,
			    cam->dma_bufs[0], cam->dma_handles[0]);
#else
	    free_pages((unsigned long)cam->dma_bufs[0], cam->order[0]);
#endif
	    cam->nbufs = 0;
	case 0:
	    cam_err(cam, "Insufficient DMA buffers, cannot operate\n");
	    return -ENOMEM;

	case 2:
	    if (n_dma_bufs > 2)
		    cam_warn(cam, "Will limp along with only 2 buffers\n");
	    break;
	}
	return 0;
}

static void ccic_free_dma_bufs(struct ccic_camera *cam)
{
	int i;

	for (i = 0; i < cam->nbufs; i++) {
#if DMA_POOL
		dma_free_coherent(&cam->pdev->dev, cam->dma_buf_size,
				cam->dma_bufs[i], cam->dma_handles[i]);
#else
		if (cam->dma_bufs[i]) {
			free_pages((unsigned long)cam->dma_bufs[i],
							cam->order[i]);
#endif
			cam->dma_bufs[i] = NULL;
		}
	}
	cam->nbufs = 0;
}





/* ----------------------------------------------------------------------- */
/*
 * Here starts the V4L2 interface code.
 */

/*
 * Read an image from the device.
 */
static ssize_t ccic_deliver_buffer(struct ccic_camera *cam,
		char __user *buffer, size_t len, loff_t *pos)
{
	int bufno;
	unsigned long flags;

	spin_lock_irqsave(&cam->dev_lock, flags);
	if (cam->next_buf < 0) {
		cam_err(cam, "deliver_buffer: No next buffer\n");
		spin_unlock_irqrestore(&cam->dev_lock, flags);
		return -EIO;
	}
	bufno = cam->next_buf;
	clear_bit(bufno, &cam->flags);
	if (++(cam->next_buf) >= cam->nbufs)
		cam->next_buf = 0;
	if (! test_bit(cam->next_buf, &cam->flags))
		cam->next_buf = -1;
	cam->specframes = 0;
	spin_unlock_irqrestore(&cam->dev_lock, flags);

	if (len > cam->pix_format.sizeimage)
		len = cam->pix_format.sizeimage;
#if !DMA_POOL
	dma_sync_single_for_device(&cam->pdev->dev,
		cam->dma_handles[bufno],
		len, DMA_FROM_DEVICE);
#endif
	if (copy_to_user(buffer, cam->dma_bufs[bufno], len))
		return -EFAULT;
	(*pos) += len;
	return len;
}

/*
 * Get everything ready, and start grabbing frames.
 */
static int ccic_read_setup(struct ccic_camera *cam, enum ccic_state state)
{
	int ret;
	unsigned long flags;

	/*
	 * Configuration.  If we still don't have DMA buffers,
	 * make one last, desperate attempt.
	 */
	if (cam->nbufs == 0 && V4L2_MEMORY_MMAP == cam->io_type)
		if (ccic_alloc_dma_bufs(cam, 0))
			return -ENOMEM;

	if (ccic_needs_config(cam)) {
		ccic_cam_configure(cam);
		ret = ccic_ctlr_configure(cam);
		if (ret)
			return ret;
	}

	/*
	 * Turn it loose.
	 */
	spin_lock_irqsave(&cam->dev_lock, flags);
	ccic_reset_buffers(cam);
	ccic_ctlr_irq_enable(cam);
	cam->state = state;


	if (BUS_IS_MIPI(cam->bus_type[sensor_selected])){
		//TODO DPHY clock tunning
		ccic_reg_write(cam, REG_CSI2_DPHY6, 0x0a00);
		ccic_reg_write(cam, REG_CSI2_DPHY3, 0x0a06);
		ccic_reg_write(cam, REG_CSI2_DPHY5, 0x33);
		if ((cam->pix_format.pixelformat == V4L2_PIX_FMT_JPEG)
				&& (!cpu_is_pxa910_Ax())){
			ccic_reg_write(cam, REG_CSI2_DPHY6, 0x0802);
			ccic_reg_write(cam, REG_CSI2_DPHY3, 0x0804);
		}
		if(!cpu_is_pxa910_Ax())/* old stepping IC */
			ccic_reg_write(cam, REG_CSI2_DPHY5, 0x3c0);

		ccic_reg_write(cam, REG_CSI2_CTRL0, 0x43);
	} else {
		ccic_reg_write(cam, REG_CSI2_DPHY3, 0x0);
		ccic_reg_write(cam, REG_CSI2_DPHY6, 0x0);
		ccic_reg_write(cam, REG_CSI2_DPHY5, 0x0);
		ccic_reg_write(cam, REG_CSI2_CTRL0, 0x0);
	}
	

	ccic_ctlr_start(cam);
	spin_unlock_irqrestore(&cam->dev_lock, flags);
	__ccic_cam_cmd(cam, VIDIOC_STREAMON, NULL);
	return 0;
}


static ssize_t ccic_v4l_read(struct file *filp,
		char __user *buffer, size_t len, loff_t *pos)
{
	struct ccic_camera *cam = filp->private_data;
	int ret = 0;

	/*
	 * Perhaps we're in speculative read mode and already
	 * have data?
	 */
	mutex_lock(&cam->s_mutex);
	if (cam->state == S_SPECREAD) {
		if (cam->next_buf >= 0) {
			ret = ccic_deliver_buffer(cam, buffer, len, pos);
			if (ret != 0)
				goto out_unlock;
		}
	} else if (cam->state == S_FLAKED || cam->state == S_NOTREADY) {
		ret = -EIO;
		goto out_unlock;
	} else if (cam->state != S_IDLE) {
		ret = -EBUSY;
		goto out_unlock;
	}

	/*
	 * v4l2: multiple processes can open the device, but only
	 * one gets to grab data from it.
	 */
	if (cam->owner && cam->owner != filp) {
		ret = -EBUSY;
		goto out_unlock;
	}
	cam->owner = filp;

	/*
	 * Do setup if need be.
	 */
	if (cam->state != S_SPECREAD) {
		ret = ccic_read_setup(cam, S_SINGLEREAD);
		if (ret)
			goto out_unlock;
	}
	/*
	 * Wait for something to happen.  This should probably
	 * be interruptible (FIXME).
	 */
	wait_event_timeout(cam->iowait, cam->next_buf >= 0, HZ);
	if (cam->next_buf < 0) {
		cam_err(cam, "read() operation timed out\n");
		ccic_ctlr_stop_dma(cam);
		ret = -EIO;
		goto out_unlock;
	}
	/*
	 * Give them their data and we should be done.
	 */
	ret = ccic_deliver_buffer(cam, buffer, len, pos);

  out_unlock:
	mutex_unlock(&cam->s_mutex);
	return ret;
}

/*
 * Streaming I/O support.
 */

static int ccic_vidioc_streamon(struct file *filp, void *priv,
		enum v4l2_buf_type type)
{
	struct ccic_camera *cam = filp->private_data;
	int ret = -EINVAL;

	if((cam->pix_format.pixelformat == V4L2_PIX_FMT_JPEG) &&
			BUS_IS_PARALLEL(cam->bus_type[sensor_selected])){
		sof = 7;
		jpeg_cnt = 0;
	}
	if (type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		goto out;
	mutex_lock(&cam->s_mutex);
	if (cam->state != S_IDLE || cam->n_sbufs == 0)
		goto out_unlock;

	cam->sequence = 0;
//	ccic_enable_mclk();	//for power optimization
	ret = ccic_read_setup(cam, S_STREAMING);
  out_unlock:
	mutex_unlock(&cam->s_mutex);
  out:
	return ret;
}


static int ccic_vidioc_streamoff(struct file *filp, void *priv,
		enum v4l2_buf_type type)
{
	struct ccic_camera *cam = filp->private_data;
	int ret = -EINVAL;

	if (type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		goto out;
	mutex_lock(&cam->s_mutex);
	if (cam->state != S_STREAMING)
		goto out_unlock;

	__ccic_cam_cmd(cam, VIDIOC_STREAMOFF, NULL);
	ccic_ctlr_stop_dma(cam);
//	ccic_disable_mclk();	//for power optimization
	ret = 0;

  out_unlock:
	mutex_unlock(&cam->s_mutex);
  out:
	return ret;
}



static int ccic_setup_siobuf(struct ccic_camera *cam, int index)
{
	struct ccic_sio_buffer *buf = cam->sb_bufs + index;

	INIT_LIST_HEAD(&buf->list);
	buf->v4lbuf.length = PAGE_ALIGN(cam->pix_format.sizeimage);
	buf->buffer = vmalloc_user(buf->v4lbuf.length);
	if (buf->buffer == NULL)
		return -ENOMEM;
	buf->mapcount = 0;
	buf->cam = cam;

	buf->v4lbuf.index = index;
	buf->v4lbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf->v4lbuf.field = V4L2_FIELD_NONE;
	buf->v4lbuf.memory = V4L2_MEMORY_MMAP;
	/*
	 * Offset: must be 32-bit even on a 64-bit system.  videobuf-dma-sg
	 * just uses the length times the index, but the spec warns
	 * against doing just that - vma merging problems.  So we
	 * leave a gap between each pair of buffers.
	 */
	buf->v4lbuf.m.offset = 2*index*buf->v4lbuf.length;
	return 0;

}

static int ccic_prepare_buffer_node(struct ccic_camera *cam,
	struct ccic_sio_buffer *buf, unsigned long userptr,
					unsigned int size, unsigned int index)
{
	unsigned int vaddr = PAGE_ALIGN(userptr);
	struct v4l2_pix_format *fmt = &cam->pix_format;

	buf->dma_handles = va_to_pa(vaddr, size);
	if (!buf->dma_handles) {
		printk(KERN_INFO "mem is not contiguous\n");
		return -ENOMEM;
	}
	buf->buffer = ioremap(buf->dma_handles, PAGE_ALIGN(size));
	cam->dma_bufs_user_pt[index] = buf->buffer;

	memset(buf->buffer, 0, size);

	INIT_LIST_HEAD(&buf->list);
	buf->v4lbuf.length = PAGE_ALIGN(size);
	buf->mapcount = 0;

	buf->v4lbuf.index = index;
	buf->v4lbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf->v4lbuf.field = V4L2_FIELD_NONE;
	buf->v4lbuf.memory = V4L2_MEMORY_USERPTR;
	buf->v4lbuf.m.offset = 2*index*buf->v4lbuf.length;

	if (fmt->pixelformat == V4L2_PIX_FMT_YUV422P) {
		buf->yuv_p.y = buf->dma_handles;
		buf->yuv_p.u = buf->yuv_p.y + fmt->width*fmt->height;
		buf->yuv_p.v = buf->yuv_p.u + fmt->width*fmt->height / 2;
	} else if (fmt->pixelformat == V4L2_PIX_FMT_YUV420) {
		buf->yuv_p.y = buf->dma_handles;
		buf->yuv_p.u = buf->yuv_p.y + fmt->width*fmt->height;
		buf->yuv_p.v = buf->yuv_p.u + fmt->width*fmt->height / 4;
	} else {
		buf->yuv_p.y = buf->dma_handles;
		buf->yuv_p.u = 0;
		buf->yuv_p.v = 0;
	}
	return 0;
}

static void ccic_free_buffer_node(struct ccic_sio_buffer *sbuf)
{
	/*
	 * vunmap will do TLB flush for us.
	 * We map uncachable memory, so needn't cache invalid operation here.
	 */

	if (V4L2_MEMORY_USERPTR != sbuf->v4lbuf.memory)
		return;

	if (sbuf->buffer) {
		iounmap(sbuf->buffer);
		sbuf->buffer = NULL;
	}
}

static int ccic_free_sio_buffers(struct ccic_camera *cam)
{
	int i;

	/*
	 * If any buffers are mapped, we cannot free them at all.
	 */
	for (i = 0; i < cam->n_sbufs; i++) {
		if (cam->sb_bufs[i].mapcount > 0)
			return -EBUSY;
		ccic_free_buffer_node(&cam->sb_bufs[i]);
	}
	cam->n_map_bufs = 0;
	/*
	 * OK, let's do it.
	 */
	for (i = 0; i < cam->n_sbufs; i++) {
		if (V4L2_MEMORY_MMAP == cam->sb_bufs[i].v4lbuf.memory &&
						cam->sb_bufs[i].buffer){
			vfree(cam->sb_bufs[i].buffer);
			cam->sb_bufs[i].buffer = NULL;
		}
	}
	cam->n_sbufs = 0;
	if (cam->sb_bufs) {
		/*kfree(cam->sb_bufs); safe no need*/
		cam->sb_bufs = NULL;
	}
	INIT_LIST_HEAD(&cam->sb_avail);
	INIT_LIST_HEAD(&cam->sb_full);
	INIT_LIST_HEAD(&cam->sb_dma);
	return 0;
}



static int ccic_vidioc_reqbufs(struct file *filp, void *priv,
		struct v4l2_requestbuffers *req)
{
	struct ccic_camera *cam = filp->private_data;
	int ret = 0;  /* Silence warning */

	/*
	 * Make sure it's something we can do.  User pointers could be
	 * implemented without great pain, but that's not been done yet.
	 */
	if (req->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	if (req->memory == V4L2_MEMORY_USERPTR) {
		if (cam->state != S_IDLE ||
			(cam->owner && cam->owner != filp)) {
			ret = -EBUSY;
			return ret;
		}
		cam->io_type = V4L2_MEMORY_USERPTR;
		cam->owner = filp;
		/* we do not need kernel to alloc the DAM buffer */
		ccic_free_dma_bufs(cam);
		ret = ccic_free_sio_buffers(cam);
		return ret;
	}
	if (req->memory != V4L2_MEMORY_MMAP)
		return -EINVAL;
	/*
	 * If they ask for zero buffers, they really want us to stop streaming
	 * (if it's happening) and free everything.  Should we check owner?
	 */
	mutex_lock(&cam->s_mutex);
	if (req->count == 0) {
		if (cam->state == S_STREAMING)
			ccic_ctlr_stop_dma(cam);
		ret = ccic_free_sio_buffers (cam);
		goto out;
	}
	/*
	 * Device needs to be idle and working.  We *could* try to do the
	 * right thing in S_SPECREAD by shutting things down, but it
	 * probably doesn't matter.
	 */
	if (cam->state != S_IDLE || (cam->owner && cam->owner != filp)) {
		ret = -EBUSY;
		goto out;
	}
	cam->owner = filp;
	cam->io_type = V4L2_MEMORY_MMAP;

	if (req->count < min_buffers)
		req->count = min_buffers;
	else if (req->count > max_buffers)
		req->count = max_buffers;
	if (cam->n_sbufs > 0) {
		ret = ccic_free_sio_buffers(cam);
		if (ret)
			goto out;
	}

	cam->sb_bufs = kzalloc(req->count*sizeof(struct ccic_sio_buffer),
			GFP_KERNEL);
	if (cam->sb_bufs == NULL) {
		ret = -ENOMEM;
		goto out;
	}
	for (cam->n_sbufs = 0; cam->n_sbufs < req->count; (cam->n_sbufs++)) {
		ret = ccic_setup_siobuf(cam, cam->n_sbufs);
		if (ret)
			break;
	}

	if (cam->n_sbufs == 0)  /* no luck at all - ret already set */
		kfree(cam->sb_bufs);
	req->count = cam->n_sbufs;  /* In case of partial success */

  out:
	mutex_unlock(&cam->s_mutex);
	return ret;
}


static int ccic_vidioc_querybuf(struct file *filp, void *priv,
		struct v4l2_buffer *buf)
{
	struct ccic_camera *cam = filp->private_data;
	int ret = -EINVAL;

	mutex_lock(&cam->s_mutex);
	if (buf->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		goto out;
	if (buf->index < 0 || buf->index >= cam->n_sbufs)
		goto out;
	*buf = cam->sb_bufs[buf->index].v4lbuf;
	ret = 0;
  out:
	mutex_unlock(&cam->s_mutex);
	return ret;
}

static int ccic_vidioc_qbuf(struct file *filp, void *priv,
		struct v4l2_buffer *buf)
{
	struct ccic_camera *cam = filp->private_data;
	struct ccic_sio_buffer *sbuf;
	int ret = -EINVAL;
	unsigned long flags;

	mutex_lock(&cam->s_mutex);
	if (buf->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		goto out;

	if (NULL == cam->sb_bufs) {
		cam->sb_bufs =
			kzalloc(max_buffers * sizeof(struct ccic_sio_buffer),
				GFP_KERNEL);
		if (cam->sb_bufs == NULL) {
			ret = -ENOMEM;
			goto out;
		}

	}
	sbuf = cam->sb_bufs + buf->index;
	if (sbuf->v4lbuf.flags & V4L2_BUF_FLAG_QUEUED) {
		ret = 0; /* Already queued?? */
		goto out;
	}
	if (sbuf->v4lbuf.flags & V4L2_BUF_FLAG_DONE) {
		/* Spec doesn't say anything, seems appropriate tho */
		ret = -EBUSY;
		goto out;
	}

	if ((buf->memory == V4L2_MEMORY_USERPTR)
			&& (buf->index == cam->n_sbufs)) {
		if (buf->length < cam->pix_format.sizeimage) {
			printk(KERN_ERR "prepare buffer, size is not enough\n");
			goto out;
		}

		if (buf->index > max_buffers) {
			printk(KERN_ERR "Only %d buffers are supported\n",
								max_buffers);
			goto out;
		}

		if (ccic_prepare_buffer_node(cam, sbuf,
				buf->m.userptr, buf->length, buf->index)){
			ret = -EINVAL;
			goto out;
		}
		cam->n_map_bufs++;
		cam->n_sbufs++;
	} else {
		if (buf->index < 0 || buf->index >= cam->n_sbufs)
			goto out;
	}
	sbuf->v4lbuf.flags |= V4L2_BUF_FLAG_QUEUED;
	spin_lock_irqsave(&cam->dev_lock, flags);
	list_add(&sbuf->list, &cam->sb_avail);
	spin_unlock_irqrestore(&cam->dev_lock, flags);
	ret = 0;
	if (buf->memory == V4L2_MEMORY_MMAP) {
		flush_cache_range(sbuf->svma, (unsigned long)sbuf->buffer,
		(unsigned long)(sbuf->buffer + cam->pix_format.sizeimage));
	}
  out:
	mutex_unlock(&cam->s_mutex);
	return ret;
}

static int ccic_vidioc_dqbuf(struct file *filp, void *priv,
		struct v4l2_buffer *buf)
{
	struct ccic_camera *cam = filp->private_data;
	struct ccic_sio_buffer *sbuf;
	int ret = -EINVAL;
	unsigned long flags;

	mutex_lock(&cam->s_mutex);
	if (buf->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		goto out_unlock;
	if (cam->state != S_STREAMING)
		goto out_unlock;
	if (list_empty(&cam->sb_full) && filp->f_flags & O_NONBLOCK) {
		ret = -EAGAIN;
		goto out_unlock;
	}

	while (list_empty(&cam->sb_full) && cam->state == S_STREAMING) {
		mutex_unlock(&cam->s_mutex);
		if (wait_event_interruptible(cam->iowait,
						!list_empty(&cam->sb_full))) {
			ret = -ERESTARTSYS;
			goto out;
		}
		mutex_lock(&cam->s_mutex);
	}

	if (cam->state != S_STREAMING)
		ret = -EINTR;
	else {
		spin_lock_irqsave(&cam->dev_lock, flags);
		/* Should probably recheck !list_empty() here */
		sbuf = list_entry(cam->sb_full.next,
				struct ccic_sio_buffer, list);
		list_del_init(&sbuf->list);
		spin_unlock_irqrestore(&cam->dev_lock, flags);
		sbuf->v4lbuf.flags &= ~V4L2_BUF_FLAG_DONE;
		*buf = sbuf->v4lbuf;
		ret = 0;
	}

  out_unlock:
	mutex_unlock(&cam->s_mutex);
  out:
	return ret;
}



static void ccic_v4l_vm_open(struct vm_area_struct *vma)
{
	struct ccic_sio_buffer *sbuf = vma->vm_private_data;
	/*
	 * Locking: done under mmap_sem, so we don't need to
	 * go back to the camera lock here.
	 */
	sbuf->mapcount++;

	/* FIXME:
	 * Workaround only. The svma now could only be set
	 * by the first process opens the driver.
	 */
	if (!sbuf->svma)
		sbuf->svma = vma;
}


static void ccic_v4l_vm_close(struct vm_area_struct *vma)
{
	struct ccic_sio_buffer *sbuf = vma->vm_private_data;

	mutex_lock(&sbuf->cam->s_mutex);
	sbuf->mapcount--;
	/* Docs say we should stop I/O too... */
	if (sbuf->mapcount == 0) {
		sbuf->v4lbuf.flags &= ~V4L2_BUF_FLAG_MAPPED;
		sbuf->svma = 0;
	}
	mutex_unlock(&sbuf->cam->s_mutex);
}

static struct vm_operations_struct ccic_v4l_vm_ops = {
	.open = ccic_v4l_vm_open,
	.close = ccic_v4l_vm_close
};


static int ccic_v4l_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct ccic_camera *cam = filp->private_data;
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	int ret = -EINVAL;
	int i;
	struct ccic_sio_buffer *sbuf = NULL;

	//if (! (vma->vm_flags & VM_WRITE) || ! (vma->vm_flags & VM_SHARED))
	//if (! (vma->vm_flags & VM_SHARED))
	//	return -EINVAL;
	/*
	 * Find the buffer they are looking for.
	 */
	mutex_lock(&cam->s_mutex);
	for (i = 0; i < cam->n_sbufs; i++)
		if (cam->sb_bufs[i].v4lbuf.m.offset == offset) {
			sbuf = cam->sb_bufs + i;
			break;
		}
	if (sbuf == NULL)
		goto out;

	ret = remap_vmalloc_range(vma, sbuf->buffer, 0);
	if (ret)
		goto out;
	vma->vm_flags |= VM_DONTEXPAND;
	vma->vm_private_data = sbuf;
	vma->vm_ops = &ccic_v4l_vm_ops;
	sbuf->v4lbuf.flags |= V4L2_BUF_FLAG_MAPPED;
	ccic_v4l_vm_open(vma);
	ret = 0;
  out:
	mutex_unlock(&cam->s_mutex);
	return ret;
}



static int ccic_v4l_open(struct file *filp)
{
	struct video_device *vd = video_devdata(filp);
	struct ccic_camera *cam = container_of(vd, struct ccic_camera, v4ldev);
	int ret = 0;

	if (cam == NULL)
		return -ENODEV;
	/* sensor_selected should be 0 or 1 */

	if (2 == sensor_selected) {
		printk(KERN_ERR"no sensor detected !\n");
		ret = -ENODEV;
		return ret;
	}
#ifdef CONFIG_DVFM
	dvfm_disable_op_name("apps_idle", dvfm_dev_idx);
	dvfm_disable_op_name("apps_sleep", dvfm_dev_idx);
	dvfm_disable_op_name("sys_sleep", dvfm_dev_idx);
#endif
	filp->private_data = cam;

	mutex_lock(&cam->s_mutex);
	if (cam->users == 0) {
		ccic_ctlr_power_up(cam);
		ccic_set_config_needed(cam, 1);
	/* FIXME make sure this is complete */
	}

	cam->sensor = cam->sensors[sensor_selected];
	mutex_unlock(&cam->s_mutex);

	if (cam->users == 0) {
		if (cam->sensor) {
			printk(KERN_ERR "Enable cam clk\n");
			ret = ccic_enable_clk(cam,
						cam->sensor->dev.platform_data);

			((struct sensor_platform_data *)cam->sensor->
			dev.platform_data)->power_on(1, sensor_selected);

			__ccic_cam_reset(cam);

			mutex_lock(&cam->s_mutex);
			ret = __ccic_cam_cmd(cam, VIDIOC_S_INPUT,
							&sensor_selected);
			mutex_unlock(&cam->s_mutex);
		} else {
			printk(KERN_ERR "weird,couldn't find default sensor\n");
			ret = -ENODEV;
		}
	}

	mutex_lock(&cam->s_mutex);
	(cam->users)++;
	mutex_unlock(&cam->s_mutex);

	return ret;
}

static int ccic_v4l_release(struct file *filp)
{
	struct ccic_camera *cam = filp->private_data;
	struct sensor_platform_data *pdata;
	pdata = cam->sensor->dev.platform_data;

	mutex_lock(&cam->s_mutex);
	(cam->users)--;
	if (filp == cam->owner) {
		ccic_ctlr_stop_dma(cam);
		ccic_free_sio_buffers(cam);
		cam->owner = NULL;
	}
	if (cam->users == 0) {
		ccic_disable_mclk();
		pdata->power_on(0, sensor_selected);
/*		if (detected_high == 1)	 only 7660 can be powered off*/
/*			pdata->power_off(1);*/
		ccic_ctlr_power_down(cam);
		ccic_free_dma_bufs(cam);
	}
	mutex_unlock(&cam->s_mutex);
#ifdef CONFIG_DVFM
	dvfm_enable_op_name("apps_idle", dvfm_dev_idx);
	dvfm_enable_op_name("apps_sleep", dvfm_dev_idx);
	dvfm_enable_op_name("sys_sleep", dvfm_dev_idx);
#endif
	return 0;
}

int dump_register(struct ccic_camera *cam)
{
	unsigned int irqs;
	spin_lock(&cam->dev_lock);

        irqs = ccic_reg_read(cam, REG_IRQSTAT);
        printk("CCIC: REG_IRQSTAT is %x\n", irqs);
	irqs = ccic_reg_read(cam, REG_IRQSTATRAW);
	printk("CCIC: REG_IRQSTATRAW is %x\n", irqs);
        irqs = ccic_reg_read(cam, REG_IRQMASK);
        printk("CCIC: REG_IRQMASK is %x\n\n", irqs);

        irqs = ccic_reg_read(cam, REG_IMGPITCH);
	printk("CCIC: REG_IMGPITCH is %x\n", irqs);
        irqs = ccic_reg_read(cam, REG_IMGSIZE);
        printk("CCIC: REG_IMGSIZE is %x\n", irqs);
        irqs = ccic_reg_read(cam, REG_IMGOFFSET);
        printk("CCIC: REG_IMGOFFSET is %x\n\n", irqs);

        irqs = ccic_reg_read(cam, REG_CTRL0);
        printk("CCIC: REG_CTRL0 is %x\n", irqs);
        irqs = ccic_reg_read(cam, REG_CTRL1);
        printk("CCIC: REG_CTRL1 is %x\n", irqs);
        irqs = ccic_reg_read(cam, REG_CLKCTRL);
        printk("CCIC: REG_CLKCTRL is %x\n\n", irqs);

        irqs = ccic_reg_read(cam, REG_CSI2_DPHY3);
        printk("CCIC: REG_CSI2_DPHY3 is %x\n", irqs);
        irqs = ccic_reg_read(cam, REG_CSI2_DPHY5);
        printk("CCIC: REG_CSI2_DPHY5 is %x\n\n", irqs);
        irqs = ccic_reg_read(cam, REG_CSI2_DPHY6);
        printk("CCIC: REG_CSI2_DPHY6 is %x\n\n", irqs);
        irqs = ccic_reg_read(cam, REG_CSI2_CTRL0);
        printk("CCIC: REG_CSI2_CTRL0 is %x\n\n", irqs);
        spin_unlock(&cam->dev_lock);
	return 0;
}

static unsigned int ccic_v4l_poll(struct file *filp,
		struct poll_table_struct *pt)
{
	struct ccic_camera *cam = filp->private_data;

	poll_wait(filp, &cam->iowait, pt);
	if (cam->next_buf >= 0)
		return POLLIN | POLLRDNORM;
	return 0;
}



static int ccic_vidioc_queryctrl(struct file *filp, void *priv,
		struct v4l2_queryctrl *qc)
{
	struct ccic_camera *cam = filp->private_data;
	int ret;

	mutex_lock(&cam->s_mutex);
	ret = __ccic_cam_cmd(cam, VIDIOC_QUERYCTRL, qc);
	mutex_unlock(&cam->s_mutex);
	return ret;
}


static int ccic_vidioc_g_ctrl(struct file *filp, void *priv,
		struct v4l2_control *ctrl)
{
	struct ccic_camera *cam = filp->private_data;
	int ret;

	mutex_lock(&cam->s_mutex);
	ret = __ccic_cam_cmd(cam, VIDIOC_G_CTRL, ctrl);
	mutex_unlock(&cam->s_mutex);
	return ret;
}


static int ccic_vidioc_s_ctrl(struct file *filp, void *priv,
		struct v4l2_control *ctrl)
{
	struct ccic_camera *cam = filp->private_data;
	int ret;

	mutex_lock(&cam->s_mutex);
	ret = __ccic_cam_cmd(cam, VIDIOC_S_CTRL, ctrl);
	mutex_unlock(&cam->s_mutex);
	return ret;
}





static int ccic_vidioc_querycap(struct file *file, void *priv,
		struct v4l2_capability *cap)
{
	strcpy(cap->driver, "pxa168_camera");
	strcpy(cap->card, "pxa168_camera");
	cap->version = CCIC_VERSION;
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	return 0;
}


/*
 * The default format we use until somebody says otherwise.
 */

static struct v4l2_pix_format ccic_def_pix_format = {
	.width		= CIF_WIDTH,
	.height		= CIF_HEIGHT,
	.pixelformat	= V4L2_PIX_FMT_YUYV,
	.field		= V4L2_FIELD_NONE,
	.bytesperline	= CIF_WIDTH*2,
	.sizeimage	= CIF_WIDTH*VGA_HEIGHT*2,
};

/*
static struct v4l2_pix_format ccic_def_pix_format = {
	.width		= VGA_WIDTH,
	.height		= VGA_HEIGHT,
	.pixelformat	= V4L2_PIX_FMT_YUV420,
	.field		= V4L2_FIELD_NONE,
	.bytesperline	= 640 * 12 / 8, //VGA_WIDTH*2,
	.sizeimage	= 640 * 480 * 12 / 8, //VGA_WIDTH*VGA_HEIGHT*2,
};
*/

static int ccic_vidioc_enum_fmt_cap(struct file *filp,
		void *priv, struct v4l2_fmtdesc *fmt)
{
	struct ccic_camera *cam = priv;
	int ret;

	if (fmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	mutex_lock(&cam->s_mutex);
	ret = __ccic_cam_cmd(cam, VIDIOC_ENUM_FMT, fmt);
	mutex_unlock(&cam->s_mutex);
	return ret;
}


static int ccic_vidioc_try_fmt_cap (struct file *filp, void *priv,
		struct v4l2_format *fmt)
{
	struct ccic_camera *cam = priv;
	int ret;
	mutex_lock(&cam->s_mutex);
	ret = __ccic_cam_cmd(cam, VIDIOC_TRY_FMT, fmt);
	mutex_unlock(&cam->s_mutex);
	return ret;
}

static int ccic_vidioc_s_fmt_cap(struct file *filp, void *priv,
		struct v4l2_format *fmt)
{
	struct ccic_camera *cam = priv;
	int ret;
	/*
	 * Can't do anything if the device is not idle
	 * Also can't if there are streaming buffers in place.
	 */
	if (cam->state != S_IDLE) {
		return -EBUSY;
	}
	/*
	 * See if the formatting works in principle.
	 */
	ret = ccic_vidioc_try_fmt_cap(filp, priv, fmt);
	if (ret)
		return ret;
	/*
	 * Now we start to change things for real, so let's do it
	 * under lock.
	 */
	mutex_lock(&cam->s_mutex);
	cam->pix_format = fmt->fmt.pix;

	/*
	 * Make sure we have appropriate DMA buffers.
	 */
	ret = -ENOMEM;
	if (cam->nbufs > 0 && cam->dma_buf_size < cam->pix_format.sizeimage)
		ccic_free_dma_bufs(cam);
	if (cam->nbufs == 0) {
		if (ccic_alloc_dma_bufs(cam, 0))
			goto out;
	}
	/*
	 * It looks like this might work, so let's program the sensor.
	 */
	ccic_set_config_needed(cam, 1);
	ret = ccic_cam_configure(cam);
  out:
	mutex_unlock(&cam->s_mutex);
	return ret;
}

/*
 * Return our stored notion of how the camera is/should be configured.
 * The V4l2 spec wants us to be smarter, and actually get this from
 * the camera (and not mess with it at open time).  Someday.
 */
static int ccic_vidioc_g_fmt_cap(struct file *filp, void *priv,
		struct v4l2_format *f)
{
	struct ccic_camera *cam = priv;

	f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	f->fmt.pix = cam->pix_format;
	return 0;
}

/*
 * We only have one input - the sensor - so minimize the nonsense here.
 */
static int ccic_vidioc_enum_input(struct file *filp, void *priv,
		struct v4l2_input *input)
{
	if ((input->index == 0 && detected_low == 0) ||
			(input->index == 1 && detected_high == 0) ||
			(input->index != 0 && input->index != 1))
		return -EINVAL;

	input->type = V4L2_INPUT_TYPE_CAMERA;
	/* input->std = V4L2_STD_ALL; */ /* Not sure what should go here */
	strcpy(input->name, "Camera");
	return 0;
}

static int ccic_vidioc_g_input(struct file *filp, void *priv, unsigned int *i)
{
	*i = sensor_selected;
	return 0;
}

static int ccic_enable_clk(struct ccic_camera *cam,
					struct sensor_platform_data *pdata)
{
	/*TODO need to do below reset for stress test*/
	ccic_disable_clock();

	if (BUS_IS_MIPI(cam->bus_type[sensor_selected]))
		ccic_set_clock_mipi();
	else
		ccic_set_clock_parallel();

	ccic_enable_mclk(cam);
	if (pdata->platform_set)
		/*set specific platform clk*/
		pdata->platform_set(sensor_selected, rst_clk);
	return 0;
}

static int ccic_vidioc_s_input(struct file *filp, void *priv, unsigned int i)
{
	struct ccic_camera *cam = filp->private_data;
        int ret = 0;

	/* If the required sensor is the same as the current
	 * active one, return immediately.
	 */
	if (i == sensor_selected)
		return ret;
	if (((i == SENSOR_LOW) && (detected_low == 1)) ||
		((i == SENSOR_HIGH) && (detected_high == 1))) {
		/* switch off the previous sensor power */
		((struct sensor_platform_data *)
		cam->sensor->dev.platform_data)->power_on(0, sensor_selected);
		cam->sensor = cam->sensors[i];
		sensor_selected = i;
	} else {
		printk(KERN_ERR "requested sensor %d is NOT attached!\n", i);
		return -EINVAL;
	}

	ret = ccic_enable_clk(cam, cam->sensor->dev.platform_data);
	((struct sensor_platform_data *)
		cam->sensor->dev.platform_data)->power_on(1, sensor_selected);
	__ccic_cam_reset(cam);	

        mutex_lock(&cam->s_mutex);
        ret = __ccic_cam_cmd(cam, VIDIOC_S_INPUT, &sensor_selected);
        mutex_unlock(&cam->s_mutex);

	return ret;	
}

/* from vivi.c */
static int ccic_vidioc_s_std(struct file *filp, void *priv, v4l2_std_id *a)
{
	return 0;
}

/*
 * G/S_PARM.  Most of this is done by the sensor, but we are
 * the level which controls the number of read buffers.
 */
static int ccic_vidioc_g_parm(struct file *filp, void *priv,
		struct v4l2_streamparm *parms)
{
	struct ccic_camera *cam = priv;
	int ret;

	mutex_lock(&cam->s_mutex);
	ret = __ccic_cam_cmd(cam, VIDIOC_G_PARM, parms);
	mutex_unlock(&cam->s_mutex);
	parms->parm.capture.readbuffers = n_dma_bufs;
	return ret;
}

static int ccic_vidioc_s_parm(struct file *filp, void *priv,
		struct v4l2_streamparm *parms)
{
	struct ccic_camera *cam = priv;
	int ret;
	mutex_lock(&cam->s_mutex);
	ret = __ccic_cam_cmd(cam, VIDIOC_S_PARM, parms);
	mutex_unlock(&cam->s_mutex);
	parms->parm.capture.readbuffers = n_dma_bufs;
	return ret;
}

static int ccic_vidioc_cropcap(struct file *file, void *fh,
					struct v4l2_cropcap *a)
{
	struct ccic_camera *cam = fh;
	struct v4l2_cropcap *ccap = a;
	int ret;

	/*printk(KERN_ERR "ccic_vidioc_cropcap\n"); */
	mutex_lock(&cam->s_mutex);
	ret = __ccic_cam_cmd(cam, VIDIOC_CROPCAP, ccap);
	mutex_unlock(&cam->s_mutex);
	return ret;
}

static int ccic_vidioc_enum_framesizes(struct file *file, void *fh,
					 struct v4l2_frmsizeenum *fsize)
{
	struct ccic_camera *cam = fh;
	int ret;

	mutex_lock(&cam->s_mutex);
	ret = __ccic_cam_cmd(cam, VIDIOC_ENUM_FRAMESIZES, fsize);
	mutex_unlock(&cam->s_mutex);
	return ret;
}

static void ccic_v4l_dev_release(struct video_device *vd)
{
	struct ccic_camera *cam = container_of(vd, struct ccic_camera, v4ldev);

	kfree(cam);
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
/*for register access*/
static int ccic_vidioc_g_register(struct file *filp, void *priv,
		struct v4l2_dbg_register *reg)
{
	struct ccic_camera *cam = priv;
        int ret;
        mutex_lock(&cam->s_mutex);
	ret =  __ccic_cam_cmd(cam, VIDIOC_DBG_G_REGISTER, reg);
	mutex_unlock(&cam->s_mutex);
	return ret;
}

static int ccic_vidioc_s_register(struct file *filp, void *priv,
                struct v4l2_dbg_register *reg)
{
        struct ccic_camera *cam = priv;
        int ret;
        mutex_lock(&cam->s_mutex);
        ret =  __ccic_cam_cmd(cam, VIDIOC_DBG_S_REGISTER, reg);
        mutex_unlock(&cam->s_mutex);
        return ret;

}
#endif

static long ccic_v4l_ioctl(struct file *file, unsigned int cmd,
							unsigned long arg)
{
	struct video_device *vdev = video_devdata(file);
	struct ccic_camera *cam = container_of(vdev, struct ccic_camera,
									v4ldev);
	int ret;

	/* Handle some specific cmds */
	switch (cmd) {
	case VIDIOC_ENUM_FRAMESIZES:
	{
		ret = ccic_vidioc_enum_framesizes(file, (void *)cam,
					(struct v4l2_frmsizeenum *) arg);
		return ret;
	}
#ifdef CONFIG_VIDEO_ADV_DEBUG
	case VIDIOC_DBG_S_REGISTER:
	{
		ret = ccic_vidioc_s_register(file, (void *)cam,
					(struct v4l2_dbg_register *)arg);
		return ret;
	}
	case VIDIOC_DBG_G_REGISTER:
	{
		ret = ccic_vidioc_g_register(file, (void *)cam,
					(struct v4l2_dbg_register *)arg);
	return ret;
	}
#endif
	default:
		break;
	}
	/* Handle other ioctl cmds with standard interface */
	ret = video_ioctl2(file, cmd, arg);

	return ret;
}

/*
 * This template device holds all of those v4l2 methods; we
 * clone it for specific real devices.
 */

static const struct v4l2_file_operations ccic_v4l_fops = {
	.owner = THIS_MODULE,
	.open = ccic_v4l_open,
	.release = ccic_v4l_release,
	.read = ccic_v4l_read,
	.poll = ccic_v4l_poll,
	.mmap = ccic_v4l_mmap,
	.ioctl = ccic_v4l_ioctl,
};
/* upgrade changes from .25 to .28 */
struct v4l2_ioctl_ops ccic_ioctl_ops = {
        .vidioc_querycap        = ccic_vidioc_querycap,
        .vidioc_enum_fmt_vid_cap    = ccic_vidioc_enum_fmt_cap,
        .vidioc_try_fmt_vid_cap     = ccic_vidioc_try_fmt_cap,
        .vidioc_s_fmt_vid_cap       = ccic_vidioc_s_fmt_cap,
        .vidioc_g_fmt_vid_cap       = ccic_vidioc_g_fmt_cap,
        .vidioc_enum_input      = ccic_vidioc_enum_input,
        .vidioc_g_input         = ccic_vidioc_g_input,
        .vidioc_s_input         = ccic_vidioc_s_input,
        .vidioc_s_std           = ccic_vidioc_s_std,
        .vidioc_reqbufs         = ccic_vidioc_reqbufs,
        .vidioc_querybuf        = ccic_vidioc_querybuf,
        .vidioc_qbuf            = ccic_vidioc_qbuf,
        .vidioc_dqbuf           = ccic_vidioc_dqbuf,
        .vidioc_streamon        = ccic_vidioc_streamon,
        .vidioc_streamoff       = ccic_vidioc_streamoff,
        .vidioc_queryctrl       = ccic_vidioc_queryctrl,
        .vidioc_g_ctrl          = ccic_vidioc_g_ctrl,
        .vidioc_s_ctrl          = ccic_vidioc_s_ctrl,
        .vidioc_g_parm          = ccic_vidioc_g_parm,
        .vidioc_s_parm          = ccic_vidioc_s_parm,
	.vidioc_cropcap		= ccic_vidioc_cropcap,
#ifdef CONFIG_VIDEO_ADV_DEBUG
        .vidioc_g_register      = ccic_vidioc_g_register,
        .vidioc_s_register      = ccic_vidioc_s_register,
#endif
};

static struct video_device ccic_v4l_template = {
	.name = "pxa168-camera",
	.vfl_type = VFL_TYPE_GRABBER,
	
	.minor = -1, /* Get one dynamically */
	.tvnorms = V4L2_STD_NTSC_M,
	.current_norm = V4L2_STD_NTSC_M,  /* make mplayer happy */

	.fops = &ccic_v4l_fops,
	.release = ccic_v4l_dev_release,

	.ioctl_ops		= &ccic_ioctl_ops,
};

/* ---------------------------------------------------------------------- */
/*
 * Interrupt handler stuff
 */



static void ccic_frame_tasklet(unsigned long data)
{
	struct ccic_camera *cam = (struct ccic_camera *) data;
	int i;
	unsigned long flags;
	struct ccic_sio_buffer *sbuf;
	spin_lock_irqsave(&cam->dev_lock, flags);
	for (i = 0; i < cam->nbufs; i++) {
		int bufno = cam->next_buf;
		if (bufno < 0) {  /* "will never happen" */
			cam_err(cam, "No valid bufs in tasklet!\n");
			break;
		}
		if (++(cam->next_buf) >= cam->nbufs)
			cam->next_buf = 0;
		if (! test_bit(bufno, &cam->flags))
			continue;
		if (list_empty(&cam->sb_avail)) {
			/*drop current frame because of buffer unavailable*/
			clear_bit(bufno, &cam->flags);
			break;  /* Leave it valid, hope for better later */
		}
		clear_bit(bufno, &cam->flags);
#if 0
/*!DMA_POOL*/
		dma_sync_single_for_device(&cam->pdev->dev,
			cam->dma_handles[bufno],
			cam->pix_format.sizeimage,
			DMA_FROM_DEVICE);
#endif
		if ((cam->pix_format.pixelformat == V4L2_PIX_FMT_JPEG) &&
		((((char *)cam->dma_bufs[bufno])[0] != 0xff) ||
				(((char *)cam->dma_bufs[bufno])[1] != 0xd8))) {
			spin_unlock_irqrestore(&cam->dev_lock, flags);
			printk(KERN_INFO "%s: JPEG ERROR !!! dropped this \
							frame\n", __func__);
			return; /*drop current JPEG frame because of wrong \
								header      */
		}
	
		sbuf = list_entry(cam->sb_avail.next,
				struct ccic_sio_buffer, list);
		/*
		 * Drop the lock during the big copy.  This *should* be safe...
		 */
		spin_unlock_irqrestore(&cam->dev_lock, flags);
		memcpy(sbuf->buffer, cam->dma_bufs[bufno],
				cam->pix_format.sizeimage);
#if 0
/* TODO need to check !DMA_POOL*/
		dmac_inv_range(cam->dma_bufs[bufno],
			cam->dma_bufs[bufno] + cam->pix_format.sizeimage);
#endif
		if (cam->pix_format.pixelformat == V4L2_PIX_FMT_JPEG) {
			/* we can not get line data in parallel mode for new
			frame vsync
			 * resets line register */
			sbuf->v4lbuf.bytesused = cam->pix_format.bytesperline *
									lines;
		}else{
			sbuf->v4lbuf.bytesused = cam->pix_format.sizeimage;
		}
		sbuf->v4lbuf.sequence = cam->buf_seq[bufno];
		sbuf->v4lbuf.flags &= ~V4L2_BUF_FLAG_QUEUED;
		sbuf->v4lbuf.flags |= V4L2_BUF_FLAG_DONE;
		spin_lock_irqsave(&cam->dev_lock, flags);
		list_move_tail(&sbuf->list, &cam->sb_full);
		break;		/*just exit once current frame done */
	}
	if (! list_empty(&cam->sb_full))
		wake_up(&cam->iowait);
	spin_unlock_irqrestore(&cam->dev_lock, flags);
}



static void ccic_frame_complete(struct ccic_camera *cam, int frame)
{
	/*
	 * Basic frame housekeeping.
	 */
	if (test_bit(frame, &cam->flags) && printk_ratelimit())
		cam_err(cam, "Frame overrun on %d, frames lost\n", frame);
	set_bit(frame, &cam->flags);
	clear_bit(CF_DMA_ACTIVE, &cam->flags);
	if (cam->next_buf < 0)
		cam->next_buf = frame;
	cam->buf_seq[frame] = ++(cam->sequence);

	switch (cam->state) {
	/*
	 * If in single read mode, try going speculative.
	 */
	    case S_SINGLEREAD:
		cam->state = S_SPECREAD;
		cam->specframes = 0;
		wake_up(&cam->iowait);
		break;

	/*
	 * If we are already doing speculative reads, and nobody is
	 * reading them, just stop.
	 */
	    case S_SPECREAD:
		if (++(cam->specframes) >= cam->nbufs) {
			ccic_ctlr_stop(cam);
			ccic_ctlr_irq_disable(cam);
			cam->state = S_IDLE;
		}
		wake_up(&cam->iowait);
		break;
	/*
	 * For the streaming case, we defer the real work to the
	 * camera tasklet.
	 *
	 * FIXME: if the application is not consuming the buffers,
	 * we should eventually put things on hold and restart in
	 * vidioc_dqbuf().
	 */
	    case S_STREAMING:
		if (V4L2_MEMORY_MMAP == cam->io_type)
			tasklet_schedule(&cam->s_tasklet);
		else if (V4L2_MEMORY_USERPTR == cam->io_type)
			ccic_switch_dma(cam, frame);
		break;

	    default:
		cam_err(cam, "Frame interrupt in non-operational state\n");
		break;
	}
}




static void ccic_frame_irq(struct ccic_camera *cam, unsigned int irqs)
{
	unsigned int frame;

	/*
	 * Handle any frame completions.  There really should
	 * not be more than one of these, or we have fallen
	 * far behind.
	 */
	for (frame = 0; frame < MAX_DMA_BUFS; frame++) {
		if (irqs & (IRQ_EOF0 << frame))
			ccic_frame_complete(cam, frame);
	}
	/* We workaround parallel interface JPEG mode.
	 * We can not detect EOF with IRQ, for JPEG file size is variable.
	 * We set lines as large as that JPEG data never fullfill and use SOF as
	 * frame complete indication. */
	if((cam->pix_format.pixelformat == V4L2_PIX_FMT_JPEG) &&
			BUS_IS_PARALLEL(cam->bus_type[sensor_selected])) {
		for (frame = 0; frame < MAX_DMA_BUFS; frame++)
			/* collect data according to SOF flag */
			if(sof & (1 << frame)){
				void **dma_bufs_temp = NULL;
				if (V4L2_MEMORY_USERPTR == cam->io_type)
					dma_bufs_temp = cam->dma_bufs_user_pt;
				else
					dma_bufs_temp = cam->dma_bufs;

				if ((jpeg_cnt % 2) &&
					((((char *)dma_bufs_temp[frame])[0]
					!= 0xff) ||
				(((char *)dma_bufs_temp[frame])[1] != 0xd8))) {
					jpeg_cnt ++;
				}

				lines = ccic_reg_read(cam, REG_LNNUM);
				if(jpeg_cnt %2) /* drop even frames */
					ccic_frame_complete(cam, frame);
				sof &= ~(1 << frame); /*mark this buffer is \
								handled */
			/* mark new SOF for data later collection */
			if (irqs & (IRQ_SOF0 << frame)){
				sof |= 1 << frame;
				jpeg_cnt ++;
			}
		}
	}
	/*
	 * If a frame starts, note that we have DMA active.  This
	 * code assumes that we won't get multiple frame interrupts
	 * at once; may want to rethink that.
	 */
	if (irqs & (IRQ_SOF0 | IRQ_SOF1 | IRQ_SOF2))
		set_bit(CF_DMA_ACTIVE, &cam->flags);
}



//#define SOF_DEBUG
static irqreturn_t ccic_irq(int irq, void *data)
{
	struct ccic_camera *cam = data;
	unsigned int irqs;

	spin_lock(&cam->dev_lock);
#ifdef SOF_DEBUG
	unsigned int irqs_raw;
	irqs_raw = ccic_reg_read(cam, REG_IRQSTATRAW);
#endif
	irqs = ccic_reg_read(cam, REG_IRQSTAT);
	ccic_reg_write(cam, REG_IRQSTAT, irqs); /*clear irqs here*/
#ifdef SOF_DEBUG
	static unsigned int first = 0, second = 0;
	static int counter = 0;
	printk("%s: REG_IRQRAWSTAT = %x\n", __func__, irqs_raw);
	if (counter++ % 2)
		second = irqs;
	else 
		first = irqs;
	if (first == second) {
		printk(KERN_NOTICE "\n-- %s: TWO CONTINUOUS SOF/EOF OCCURED!!! \
							--\n\n", __func__);
	}
#endif
	if ((irqs & ALLIRQS) == 0) {
		spin_unlock(&cam->dev_lock);
		return IRQ_NONE;
	}
	if (irqs & FRAMEIRQS)
		ccic_frame_irq(cam, irqs);
	if (irqs & TWSIIRQS) {
		ccic_reg_write(cam, REG_IRQSTAT, TWSIIRQS);
	}
	spin_unlock(&cam->dev_lock);
	return IRQ_HANDLED;
}


/* -------------------------------------------------------------------------- */
#ifdef CONFIG_VIDEO_ADV_DEBUG
/*
 * Debugfs stuff.
 */

static char ccic_debug_buf[1024];
static struct dentry *ccic_dfs_root;

static void ccic_dfs_setup(void)
{
	ccic_dfs_root = debugfs_create_dir("pxa910_camera", NULL);
	if (IS_ERR(ccic_dfs_root)) {
		ccic_dfs_root = NULL;  /* Never mind */
		printk(KERN_NOTICE "pxa910_camera unable to set up debugfs\n");
	}
}

#if 0
static void ccic_dfs_shutdown(void)
{
	if (ccic_dfs_root)
		debugfs_remove(ccic_dfs_root);
}
#endif

static int ccic_dfs_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t ccic_dfs_read_regs(struct file *file,
		char __user *buf, size_t count, loff_t *ppos)
{
	struct ccic_camera *cam = file->private_data;
	char *s = ccic_debug_buf;
	int offset;

	for (offset = 0; offset < 0x44; offset += 4)
		s += sprintf(s, "%02x: %08x\n", offset,
				ccic_reg_read(cam, offset));
	for (offset = 0x88; offset <= 0x90; offset += 4)
		s += sprintf(s, "%02x: %08x\n", offset,
				ccic_reg_read(cam, offset));
	for (offset = 0xb4; offset <= 0xbc; offset += 4)
		s += sprintf(s, "%02x: %08x\n", offset,
				ccic_reg_read(cam, offset));
	for (offset = 0x3000; offset <= 0x300c; offset += 4)
		s += sprintf(s, "%04x: %08x\n", offset,
				ccic_reg_read(cam, offset));
	return simple_read_from_buffer(buf, count, ppos, ccic_debug_buf,
			s - ccic_debug_buf);
}

static const struct file_operations ccic_dfs_reg_ops = {
	.owner = THIS_MODULE,
	.read = ccic_dfs_read_regs,
	.open = ccic_dfs_open
};

static ssize_t ccic_dfs_read_cam(struct file *file,
		char __user *buf, size_t count, loff_t *ppos)
{
	struct ccic_camera *cam = file->private_data;
	char *s = ccic_debug_buf;
	int offset;
	struct v4l2_dbg_register reg;
	/*3640 has too many registers so only support 7660*/
	if ((!cam->sensor) || (sensor_selected != SENSOR_LOW))
		return -EINVAL;
	mutex_lock(&cam->s_mutex);
	for (offset = 0x0; offset < 0x8a; offset++)
	{
		reg.reg = offset;
		__ccic_cam_cmd(cam, VIDIOC_DBG_G_REGISTER, &reg);
		s += sprintf(s, "%02x: %02x\n", offset, (u8)reg.val);
	}
	mutex_unlock(&cam->s_mutex);
	return simple_read_from_buffer(buf, count, ppos, ccic_debug_buf,
			s - ccic_debug_buf);
}

static const struct file_operations ccic_dfs_cam_ops = {
	.owner = THIS_MODULE,
	.read = ccic_dfs_read_cam,
	.open = ccic_dfs_open
};



static void ccic_dfs_cam_setup(struct ccic_camera *cam)
{
	char fname[40];

	if (!ccic_dfs_root)
		return;
	sprintf(fname, "regs-%d", cam->v4ldev.minor);
	cam->dfs_regs = debugfs_create_file(fname, 0444, ccic_dfs_root,
			cam, &ccic_dfs_reg_ops);
	sprintf(fname, "cam-%d", cam->v4ldev.minor);
	cam->dfs_cam_regs = debugfs_create_file(fname, 0444, ccic_dfs_root,
			cam, &ccic_dfs_cam_ops);
}


static void ccic_dfs_cam_shutdown(struct ccic_camera *cam)
{
	if (! IS_ERR(cam->dfs_regs))
		debugfs_remove(cam->dfs_regs);
	if (! IS_ERR(cam->dfs_cam_regs))
		debugfs_remove(cam->dfs_cam_regs);
}

#else

#define ccic_dfs_setup()
#define ccic_dfs_shutdown()
#define ccic_dfs_cam_setup(cam)
#define ccic_dfs_cam_shutdown(cam)
#endif    /* CONFIG_VIDEO_ADV_DEBUG */




static int pxa910_camera_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret;
	struct ccic_camera *cam;
	/*
	 * Start putting together one of our big camera structures.
	 */
	ret = -ENOMEM;
        rst_clk = clk_get(&pdev->dev, "CCICRSTCLK");
        if (IS_ERR(rst_clk)) {
                dev_err(&pdev->dev, "unable to get CCICRSTCLK");
                return PTR_ERR(rst_clk);
        }

        pxa168_ccic_gate_clk = clk_get(&pdev->dev, "CCICGATECLK");
        if (IS_ERR(pxa168_ccic_gate_clk)) {
                dev_err(&pdev->dev, "unable to get CCICGATECLK");
                return PTR_ERR(pxa168_ccic_gate_clk);
        }

	cam = kzalloc(sizeof(struct ccic_camera), GFP_KERNEL);
	if (cam == NULL)
		goto out;
	memset(cam, 0x00, sizeof(struct ccic_camera));
	platform_set_drvdata(pdev, cam);
	mutex_init(&cam->s_mutex);
	mutex_lock(&cam->s_mutex);
	spin_lock_init(&cam->dev_lock);
	cam->state = S_NOTREADY;
	ccic_set_config_needed(cam, 1);
	init_waitqueue_head(&cam->iowait);
	cam->pdev = pdev;
	cam->pix_format = ccic_def_pix_format;
	INIT_LIST_HEAD(&cam->dev_list);
	INIT_LIST_HEAD(&cam->sb_avail);
	INIT_LIST_HEAD(&cam->sb_full);
	tasklet_init(&cam->s_tasklet, ccic_frame_tasklet, (unsigned long) cam);

	INIT_LIST_HEAD(&cam->sb_dma);
	/*cam_ctx->platform_ops = pdev->dev.platform_data;
	 * if (cam_ctx->platform_ops == NULL) {
	 *	printk("camera no platform data defined\n");
	 * return -ENODEV;
	 * } */

	cam->irq = platform_get_irq(pdev, 0);
	if (cam->irq < 0)
		return -ENXIO;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		printk("no IO memory resource defined\n");
		return -ENODEV;
	}

	ret = -EIO;
	cam->regs = ioremap(res->start, SZ_4K);
	if (! cam->regs) {
		printk(KERN_ERR "Unable to ioremap pxa910-camera regs\n");
		goto out_free;
	}
	ret = request_irq(cam->irq, ccic_irq, IRQF_SHARED, "pxa168-cam", cam);
	if (ret)
		goto out_iounmap;
	/*
	 * Initialize the controller and leave it powered up.  It will
	 * stay that way until the sensor driver shows up.
	 */
	ccic_ctlr_init(cam);
	ccic_ctlr_power_up(cam);
	/*
	 * Set up I2C/SMBUS communications.  We have to drop the mutex here
	 * because the sensor could attach in this call chain, leading to
	 * unsightly deadlocks.
	 */
	mutex_unlock(&cam->s_mutex);  /* attach can deadlock */
	/*
	 * Get the v4l2 setup done.
	 */
	mutex_lock(&cam->s_mutex);
	cam->v4ldev = ccic_v4l_template;
	cam->v4ldev.debug = 0;
/*	cam->v4ldev.debug = V4L2_DEBUG_IOCTL_ARG;*/
/*	cam->v4ldev.dev = &pdev->dev;*/
	cam->v4ldev.dev = pdev->dev;

	/*
	cam->v4ldev.v4l2_dev =
	*(struct v4l2_device *)kzalloc(sizeof(struct v4l2_device),
								GFP_ATOMIC);
	ret = v4l2_device_register(&pdev->dev, &cam->v4ldev.v4l2_dev);
	if (ret) {
		printk(KERN_ERR "ccic: v4l2_device_register_failed.\n");
		goto out_freeirq;
	}
	*/
	cam->v4ldev.parent = &pdev->dev;
	ret = video_register_device(&cam->v4ldev, VFL_TYPE_GRABBER, -1);
	if (ret)
		goto out_freeirq;
	/*
	 * If so requested, try to get our DMA buffers now.
	 */
	if (!alloc_bufs_at_read) {
		if (ccic_alloc_dma_bufs(cam, 1))
			cam_warn(cam, "Unable to alloc DMA buffers at load"
					" will try again later.");
	}

	/* allocate rubbish buffer */
	cam->rubbish_buf_virt = (void *)__get_free_pages(GFP_KERNEL,
						get_order(dma_buf_size));
	if (!cam->rubbish_buf_virt) {
		printk(KERN_ERR "Can't get memory for rubbish buffer\n");
		return -ENOMEM;
	} else {
		cam->rubbish_buf_phy = __pa(cam->rubbish_buf_virt);
	}

	ccic_dfs_setup();
	ccic_dfs_cam_setup(cam);
	mutex_unlock(&cam->s_mutex);
	ccic_add_dev(cam);
//	ccic_ctlr_power_down(cam);	//for power optimization
	return 0;

  out_freeirq:
	ccic_ctlr_power_down(cam);
	free_irq(cam->irq, cam);
  out_iounmap:
	iounmap(cam->regs);
  out_free:
	kfree(cam);
  out:
	return ret;
}


/*
 * Shut down an initialized device
 */
static void ccic_shutdown(struct ccic_camera *cam)
{
/* FIXME: Make sure we take care of everything here */
	ccic_dfs_cam_shutdown(cam);
	if (cam->n_sbufs > 0)
		/* What if they are still mapped?  Shouldn't be, but... */
		ccic_free_sio_buffers(cam);
	ccic_remove_dev(cam);
	ccic_ctlr_stop_dma(cam);
	ccic_ctlr_power_down(cam);
	ccic_free_dma_bufs(cam);
	free_irq(cam->irq, cam);
	iounmap(cam->regs);
	video_unregister_device(&cam->v4ldev);
	/* kfree(cam); done in v4l_release () */
}

static int pxa910_camera_remove(struct platform_device *pdev)
{
	struct ccic_camera *cam = ccic_find_by_pdev(pdev);

	if (cam == NULL) {
		printk(KERN_WARNING "remove on unknown pdev %p\n", pdev);
		return -ENODEV;
	}
	mutex_lock(&cam->s_mutex);
	if (cam->users > 0)
		cam_warn(cam, "Removing a device with users!\n");
	ccic_shutdown(cam);
	/* No unlock - it no longer exists */
	/* free rubbish buffer */
	if (cam->rubbish_buf_virt)
		free_pages((unsigned long)cam->rubbish_buf_virt,
						get_order(dma_buf_size));

	return 0;
}

#ifdef CONFIG_PM
/*
 * Basic power management.
 */
static int ccic_suspend(struct platform_device *dev, pm_message_t state)
{
	struct ccic_camera *cam = platform_get_drvdata(dev);
	struct sensor_platform_data *pdata;
	enum ccic_state cstate;

	cstate = cam->state; /* HACK - stop_dma sets to idle */
	pdata = cam->sensor->dev.platform_data;
	ccic_ctlr_stop_dma(cam);
	/* we just keep sensor in idle state in first state. \
	 * reset sensor need more configuration */
	/*pdata->power_on(0, sensor_selected);*/
	ccic_ctlr_power_down(cam);
	ccic_disable_clock();
	cam->state = cstate;
	return 0;
}


static int ccic_resume(struct platform_device *dev)
{
	struct ccic_camera *cam = platform_get_drvdata(dev);
	int ret = 0;

	ccic_ctlr_init(cam);
	ccic_ctlr_power_down(cam);

	mutex_lock(&cam->s_mutex);
	if (cam->users > 0) {
		ccic_ctlr_power_up(cam);
		ccic_enable_clk(cam, cam->sensor->dev.platform_data);
	}
	mutex_unlock(&cam->s_mutex);

	set_bit(CF_CONFIG_NEEDED, &cam->flags);
	if (cam->state == S_SPECREAD)
		cam->state = S_IDLE;  /* Don't bother restarting */
	else if (cam->state == S_SINGLEREAD || cam->state == S_STREAMING)
		ret = ccic_read_setup(cam, cam->state);
	return ret;
}

#endif  /* CONFIG_PM */

static struct platform_driver pxa910_camera_driver = {
	.driver = {
		.name = "pxa168-camera"
	},
	.probe 		= pxa910_camera_probe,
	.remove 	= pxa910_camera_remove,
#ifdef CONFIG_PM
	.suspend 	= ccic_suspend,
	.resume 	= ccic_resume,
#endif

};

static int __devinit pxa910_camera_init(void)
{
#ifdef CONFIG_DVFM
	dvfm_register("Camera", &dvfm_dev_idx);
#endif
	return platform_driver_register(&pxa910_camera_driver);
}

static void __exit pxa910_camera_exit(void)
{	
	platform_driver_unregister(&pxa910_camera_driver);
#ifdef CONFIG_DVFM
	dvfm_unregister("Camera", &dvfm_dev_idx);
#endif
}

module_init(pxa910_camera_init);
module_exit(pxa910_camera_exit);

