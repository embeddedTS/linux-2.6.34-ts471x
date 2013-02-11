/*
 *  linux/drivers/media/video/ov529_drv.c - ov529 camera controller driver
 *
 *  Based on linux/drivers/media/video/pxa168_camera.c
 *
 *  Copyright:	(C) Copyright 2009 Marvell International Ltd.
 *              Weili Xia <wlxia@marvell.com>
 *
 * A driver for the OV529 single chip camera controller from OmniVision
 * technology.  Currently works with the Omnivision OV7740 sensor.
 *
 * Copyright 2006 One Laptop Per Child Association, Inc.
 * Copyright 2006-7 Jonathan Corbet <corbet@lwn.net>
 *
 * Written by Weili Xia, wlxia@marvell.com.
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
#include <linux/vmalloc.h>
#include <linux/platform_device.h>

#include <linux/uaccess.h>
#include <linux/io.h>
#include <asm/cacheflush.h>

#include <linux/clk.h>
#include "ov529_hw_ops.h"
#include "ov529_drv.h"
#include <mach/pxa168.h>
#include <asm/mach-types.h>

#if defined(CONFIG_DVFM)
#include <mach/dvfm.h>
static int dvfm_dev_idx;
#endif

#define DRIVER_DESC		"ov529 Camera Controller Driver"
#define DRIVER_VERSION		"v0.0.1"

#define OV529_VER		0x01

/* #define OV529_DBG */
#undef OV529_DBG
#ifdef OV529_DBG
#define DRIVER_NAME	"ov529"
#define ov_dbg(f, msg...) \
	printk(KERN_ERR DRIVER_NAME " [%s()]: " f, __func__, ## msg)
#else
#define ov_dbg(f, msg...)
#endif

#ifdef SMC_USE_DMA
#define WORKQUEUE_DELAY 2
#else
#define WORKQUEUE_DELAY 3
#endif

#define MIN_BUFFER_NUM		1
#define MAX_BUFFER_NUM		3

struct ov529_frame ov529_frames[] =  {
	{
		.index		= 0,
		.width		= VGA_WIDTH,
		.height		= VGA_HEIGHT,
		.interval_type	= V4L2_FRMIVAL_TYPE_DISCRETE,
		.frameinterval	= 1,
	},
	{
		.index		= 1,
		.width		= CIF_WIDTH,
		.height		= CIF_HEIGHT,
		.interval_type	= V4L2_FRMIVAL_TYPE_DISCRETE,
		.frameinterval	= 1,
	},
};

static volatile int current_imagesize;
static volatile int last_imagesize;

/* work queue for transfering image from ov529 */
static struct workqueue_struct *workqueue;

static int ov529_recv_img(struct ov529_encoder *ov529, int index);

static int ov529_counter = 0;
/* delayed work */
static void ov529_iotransfer(struct work_struct *work)
{
	struct ov529_encoder *ov529 = container_of(work, struct ov529_encoder, iotransfer.work);
	struct delayed_work *iotransfer = &ov529->iotransfer;
	struct ov529_buffer *buffer;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&ov529->list_lock, flags);
	if (ov529->state != S_STREAMING)
		goto out;

	if (list_empty(&ov529->buf_avail))
		goto out;

	buffer = list_entry(ov529->buf_avail.next,
				struct ov529_buffer, list);

	ret = ov529_recv_img(ov529, buffer->v4lbuf.index);
	if (ret < 0) {
		buffer->v4lbuf.flags &= ~V4L2_BUF_FLAG_DONE;
		goto out;
	}
	ov_dbg("buffer %d filled up\n", buffer->v4lbuf.index);

	if (ov529_counter == 500)
		ov529_counter = 0;
	if (ov529_counter == 0)
		ov_dbg("500 frames\n");
	ov529_counter++;

	buffer->v4lbuf.flags &= ~V4L2_BUF_FLAG_QUEUED;
	list_move_tail(&buffer->list, &ov529->buf_full);
	wake_up(&ov529->iowait);

out:
	spin_unlock_irqrestore(&ov529->list_lock, flags);
	queue_delayed_work(workqueue, iotransfer, WORKQUEUE_DELAY);
}

/* configure ov529 */
static int ov529_configure(struct ov529_encoder *ov529)
{
	/*
	struct ov529_format *fmt = ov529->curr_fmt;
	struct ov529_frame *frame = fmt->curr_frame;
	*/
	return 0;
}

static int ov529_vidioc_querycap(struct file *file, void *priv,
		struct v4l2_capability *cap)
{
	strcpy(cap->driver, "ov529");
	strcpy(cap->card, "ov529");
	cap->version = OV529_VER;
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	return 0;
}

/*
 * We only have one input - the ov529 encoder
 */
static int ov529_vidioc_enum_input(struct file *filp, void *priv,
		struct v4l2_input *input)
{
	struct ov529_encoder *ov529 = video_drvdata(filp);

	if (input->index != ov529->curr_devid)
		return -EINVAL;

	/* ov529 should have the same intf as a camera */
	input->type = V4L2_INPUT_TYPE_CAMERA;
	/* input->std = V4L2_STD_ALL; */
	strcpy(input->name, "ov529");
	return 0;
}

static int ov529_vidioc_g_input(struct file *filp, void *priv, unsigned int *i)
{
	struct ov529_encoder *ov529 = video_drvdata(filp);

	*i = ov529->curr_devid;
	return 0;
}

static int ov529_vidioc_s_input(struct file *filp, void *priv, unsigned int i)
{
	struct ov529_encoder *ov529 = video_drvdata(filp);

	/* If the required sensor is the same as the current
	 * active one, return immediately.
	 */
	if (i == ov529->curr_devid)
		return 0;
	else
		return -ENODEV;
}

static int ov529_vidioc_enum_fmt_cap(struct file *filp,
		void *priv, struct v4l2_fmtdesc *fmt)
{
	struct ov529_encoder *ov529 = video_drvdata(filp);
	struct ov529_format *format;
	int ret = -EINVAL;
	if (fmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	mutex_lock(&ov529->s_mutex);

	if (fmt->index >= ov529->nformats)
		goto out;

	format = &ov529->fmt[fmt->index];

	fmt->flags = 0;
	fmt->flags |= V4L2_FMT_FLAG_COMPRESSED;
	strncpy(fmt->description, format->name,
			sizeof fmt->description);
	fmt->description[sizeof fmt->description - 1] = 0;
	fmt->pixelformat = format->fcc;
	ret = 0;
out:
	mutex_unlock(&ov529->s_mutex);
	return ret;
}

static int ov529_vidioc_try_fmt_cap (struct file *filp, void *priv,
		struct v4l2_format *fmt)
{
	struct v4l2_pix_format *pix = &fmt->fmt.pix;
	struct ov529_encoder *ov529 = video_drvdata(filp);
	struct ov529_format *format = NULL;
	struct ov529_frame *frame = NULL;
	int ret = 0;
	int i;

	mutex_lock(&ov529->s_mutex);

	/*
	 * find out whether this format is valid for ov529
	 */
	for (i = 0; i < ov529->nformats; i++) {
		if (ov529->fmt[i].fcc == pix->pixelformat) {
			format = &ov529->fmt[i];
			break;
		}
	}
	if (format == NULL) {
		ret = -EINVAL;
		goto out;
	}

	for (i = 0; i < format->nframes; i++) {
		if (format->frame[i].width <= pix->width &&
				format->frame[i].height <= pix->height) {
			frame = &format->frame[i];
			break;
		}
	}
	if (frame == NULL) {
		ret = -EINVAL;
		goto out;
	}

	pix->width	= frame->width;
	pix->height	= frame->height;
	pix->bytesperline	= frame->width * format->bpp / 8;
	pix->sizeimage	= 50 * 1024;
	pix->field	= V4L2_FIELD_NONE;

out:
	mutex_unlock(&ov529->s_mutex);
	printk(KERN_ERR "%s, returns %d\n", __func__, ret);
	return ret;
}

static int ov529_vidioc_s_fmt_cap(struct file *filp, void *priv,
					struct v4l2_format *fmt)
{
	struct ov529_encoder *ov529 = video_drvdata(filp);
	int ret;

	/* could not change format if ov529 is busy */
	if (ov529->state != S_IDLE)
		return -EBUSY;

	/* try the format */
	ret = ov529_vidioc_try_fmt_cap(filp, priv, fmt);
	if (ret)
		return ret;

	/* find out which format and frame
	 * should be curr_fmt and curr_frame.
	 */
	/*
	ov529->curr_fmt =
	ov529->curr_fmt->curr_frame =
	*/

	ov529_configure(ov529);

	/* mutex_lock(&cam->s_mutex); */

	return 0;
}

static int ov529_vidioc_g_fmt_cap(struct file *filp, void *priv,
					struct v4l2_format *fmt)
{
	struct ov529_encoder *ov529 = video_drvdata(filp);
	struct ov529_format *format = ov529->curr_fmt;
	struct ov529_frame *frame = format->curr_frame;
/*
	if (fmt->type != ov529->type)
		return -EINVAL;
*/
	if (format == NULL || frame == NULL)
		return -EINVAL;

	fmt->fmt.pix.pixelformat = format->fcc;
	fmt->fmt.pix.width = frame->width;
	fmt->fmt.pix.height = frame->height;
	fmt->fmt.pix.field = V4L2_FIELD_NONE;
	fmt->fmt.pix.sizeimage = 50 * 1024;
	fmt->fmt.pix.bytesperline = format->bpp * frame->width / 8;
	/* fmt->fmt.pix.sizeimage = frame->height * fmt->fmt.pix.bytesperline; */
	fmt->fmt.pix.colorspace = format->colorspace;
	fmt->fmt.pix.priv = 0;

	return 0;
}

static int ov529_setup_buf(struct ov529_encoder *ov529, int index)
{
	struct ov529_buffer *buf = ov529->buf + index;

	/* Hardcode length, from ov sample code */
	buf->v4lbuf.length = PAGE_ALIGN(50 * 1024);
	buf->buffer = vmalloc_user(buf->v4lbuf.length);
	if (buf->buffer == NULL)
		return -ENOMEM;

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
	buf->v4lbuf.m.offset = index * buf->v4lbuf.length;
	return 0;
}

static int ov529_free_buf(struct ov529_encoder *ov529)
{
	int i;

	for (i = 0; i < ov529->nbufs; i++)
		if (ov529->buf[i].vma_use_count > 0)
			return -EBUSY;

	/*
	 * Free the ov529_buffer data structures.
	 */
	for (i = 0; i < ov529->nbufs; i++)
		vfree(ov529->buf[i].buffer);
	ov529->nbufs = 0;
	kfree(ov529->buf);
	ov529->buf = NULL;

	/* re-init the buffer lists. */
	INIT_LIST_HEAD(&ov529->buf_avail);
	INIT_LIST_HEAD(&ov529->buf_full);

	return 0;
}

static int ov529_vidioc_reqbufs(struct file *filp, void *priv,
				struct v4l2_requestbuffers *req)
{
	struct ov529_encoder *ov529 = video_drvdata(filp);
	int ret = 0;

	if (req->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	if (req->memory != V4L2_MEMORY_MMAP)
		return -EINVAL;

	mutex_lock(&ov529->s_mutex);

	if (req->count < MIN_BUFFER_NUM)
		req->count = MIN_BUFFER_NUM;
	else if (req->count > MAX_BUFFER_NUM)
		req->count = MAX_BUFFER_NUM;

	if (ov529->nbufs > 0) {
		ret = ov529_free_buf(ov529);
		if (ret)
			goto out;
	}

	ov529->buf = kzalloc((req->count * sizeof(struct ov529_buffer)), GFP_KERNEL);
	if (ov529->buf == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	for (ov529->nbufs = 0; ov529->nbufs < req->count; ov529->nbufs++) {
		ret = ov529_setup_buf(ov529, ov529->nbufs);
		if (ret)
			goto out;
	}

out:
	mutex_unlock(&ov529->s_mutex);
	return ret;
}

static int ov529_vidioc_querybuf(struct file *filp, void *priv,
		struct v4l2_buffer *buf)
{
	struct ov529_encoder *ov529 = video_drvdata(filp);
	int ret = -EINVAL;

	mutex_lock(&ov529->s_mutex);
	if (buf->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		goto out;

	if (buf->index < 0 || buf->index >= ov529->nbufs)
		goto out;

	*buf = ov529->buf[buf->index].v4lbuf;
	ret = 0;
out:
	mutex_unlock(&ov529->s_mutex);
	return ret;
}

static int ov529_vidioc_qbuf(struct file *filp, void *priv,
		struct v4l2_buffer *buf)
{
	struct ov529_encoder *ov529 = video_drvdata(filp);
	struct ov529_buffer *buffer;
	int ret = -EINVAL;
	unsigned long flags;

	mutex_lock(&ov529->s_mutex);

	if (buf->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		goto out;
	if (buf->index < 0 || buf->index >= ov529->nbufs)
		goto out;
	buffer = ov529->buf + buf->index;
	if (buffer->v4lbuf.flags & V4L2_BUF_FLAG_QUEUED) {
		ret = 0; /* Already queued?? */
		goto out;
	}
	if (buffer->v4lbuf.flags & V4L2_BUF_FLAG_DONE) {
		/* Spec doesn't say anything, seems appropriate tho */
		ret = -EBUSY;
		goto out;
	}
	buffer->v4lbuf.flags |= V4L2_BUF_FLAG_QUEUED;

	spin_lock_irqsave(&ov529->list_lock, flags);
	list_add_tail(&buffer->list, &ov529->buf_avail);
	spin_unlock_irqrestore(&ov529->list_lock, flags);
	ov_dbg("buffer %d queued\n", buffer->v4lbuf.index);

	ret = 0;

out:
	mutex_unlock(&ov529->s_mutex);
	return ret;
}

#define IMAGE_BLOCK_SIZE	1024
static int ov529_recv_img(struct ov529_encoder *ov529, int index)
{
	struct ov529_buffer *curr_buf = ov529->buf + index;
	char *buf = curr_buf->buffer;
	int length = curr_buf->v4lbuf.length;
	int imagesize = 0;
	int drop_image = 0;

	/* FIXME:
	 * implement a timeout mechanism?
	 */
	/* ov_dbg("image ready\n"); */

	/* Set first two bytes to zero.
	 * These two bytes are used to verify whether a frame is correct.
	 */
	buf[0] = 0x0;
	buf[1] = 0x0;
	while (!drop_image) {
		/* ov_dbg("imagesize: %d\n", imagesize); */
		while (ov529->ops.data_ready());

		ov529->ops.recv_data((char *)(buf + imagesize), IMAGE_BLOCK_SIZE);
		if ((buf[0] == 0xFF) && (buf[1] == 0xD8)) {
			imagesize += IMAGE_BLOCK_SIZE;
			if (imagesize >= length) {
				ov_dbg("overflow\n");
				return -ENOMEM;
			}

			if (buf[imagesize - 2] == 0xFF && buf[imagesize - 1] == 0xD9) {
				current_imagesize = imagesize;
				/*
				 if((last_imagesize + 0x200) < current_imagesize) {
					 last_imagesize = current_imagesize;
					 drop_image = 1;
					 printk(KERN_ERR "frame Error 0\n");
				 } else if(current_imagesize < (last_imagesize - 0x200)) {
					 last_imagesize = current_imagesize;
					 drop_image = 1;
					printk(KERN_ERR "frame Error 1\n");
				}
				*/
				if (drop_image == 0) {
					last_imagesize = current_imagesize;
					ov_dbg("frame end, %d\r\n", imagesize);
					curr_buf->v4lbuf.bytesused = current_imagesize;
					return imagesize;
				}
			}
		} else {
			printk("receive image data error\n");
			drop_image = 1;
			return -EINVAL;
		}
	}

	return -EINVAL;
}

static int ov529_vidioc_dqbuf(struct file *filp, void *priv,
		struct v4l2_buffer *buf)
{
	struct ov529_encoder *ov529 = video_drvdata(filp);
	struct ov529_buffer *buffer;
	int ret = -EINVAL;
	unsigned long flags;

	mutex_lock(&ov529->s_mutex);
	if (buf->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		goto out;
	if (ov529->state != S_STREAMING)
		goto out;

	if (list_empty(&ov529->buf_full) && filp->f_flags & O_NONBLOCK) {
		ret = -EAGAIN;
		goto out;
	}

	while (list_empty(&ov529->buf_full) && ov529->state == S_STREAMING) {
		mutex_unlock(&ov529->s_mutex);
		if (wait_event_interruptible(ov529->iowait,
						!list_empty(&ov529->buf_full))) {
			ret = -ERESTARTSYS;
			goto out_unlocked;
		}
		mutex_lock(&ov529->s_mutex);
	}
	/*
	buffer = ov529->buf + buf->index;
	ret = ov529_recv_img(ov529);
	if (ret < 0) {
		buffer->v4lbuf.flags &= ~V4L2_BUF_FLAG_DONE;
		*buf = buffer->v4lbuf;
		goto out;
	}
	buffer->v4lbuf.flags &= ~V4L2_BUF_FLAG_QUEUED;
	*/
	spin_lock_irqsave(&ov529->list_lock, flags);
	buffer = list_entry(ov529->buf_full.next,
				struct ov529_buffer, list);
	list_del_init(&buffer->list);
	spin_unlock_irqrestore(&ov529->list_lock, flags);

	/* dequeued, clear V4L2_BUF_FLAG_DONE */
	buffer->v4lbuf.flags &= ~V4L2_BUF_FLAG_DONE;
	*buf = buffer->v4lbuf;
	ret = 0;
out:
	mutex_unlock(&ov529->s_mutex);
out_unlocked:
	return ret;
}

static int ov529_vidioc_streamon(struct file *filp, void *priv,
		enum v4l2_buf_type type)
{
	struct ov529_encoder *ov529 = video_drvdata(filp);
	int ret = -EINVAL;
	/*
	if((cam->pix_format.pixelformat == V4L2_PIX_FMT_JPEG) &&
			BUS_IS_PARALLEL(cam->bus_type[sensor_selected])){
		sof = 7;
		jpeg_cnt = 0;
	}
	*/
	if (type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		goto out;
	mutex_lock(&ov529->s_mutex);
	if (ov529->state != S_IDLE || ov529->nbufs == 0)
		goto out;

	ret = ov529->ops.streamon();
	if (ret == 0)
		ov529->state = S_STREAMING;

	queue_delayed_work(workqueue, &ov529->iotransfer, 10);
out:
	mutex_unlock(&ov529->s_mutex);
	return ret;
}

static int ov529_vidioc_streamoff(struct file *filp, void *priv,
		enum v4l2_buf_type type)
{
	struct ov529_encoder *ov529 = video_drvdata(filp);
	int ret = 0;

	mutex_lock(&ov529->s_mutex);
	ov529->state = S_IDLE;
	cancel_delayed_work(&ov529->iotransfer);
	mutex_unlock(&ov529->s_mutex);

	return ret;
}

static int ov529_vidioc_g_parm(struct file *filp, void *priv,
		struct v4l2_streamparm *parms)
{
	struct ov529_encoder *ov529 = video_drvdata(filp);

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	memset(parms, 0, sizeof *parms);

	parms->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	parms->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
	parms->parm.capture.capturemode = 0;
	parms->parm.capture.timeperframe.numerator = 1;
	parms->parm.capture.timeperframe.denominator = 25;
	parms->parm.capture.extendedmode = 0;
	parms->parm.capture.readbuffers = 0;
	printk(KERN_ERR "%s, %d\n", __func__, __LINE__);
	return 0;
}

static int ov529_vidioc_s_parm(struct file *filp, void *priv,
		struct v4l2_streamparm *parms)
{
	struct ov529_encoder *ov529 = video_drvdata(filp);

	parms->parm.capture.readbuffers = 3;

	return 0;
}

static long ov529_v4l_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct video_device *vdev = video_devdata(file);
	struct ov529_encoder *ov529 = video_drvdata(file);
	int ret = 0;

	/* v4l_printk_ioctl(cmd); */

	/* Handle some specific cmds */
	switch (cmd) {
	case VIDIOC_ENUM_FRAMESIZES:
	{
		struct v4l2_frmsizeenum *fsize = (struct v4l2_frmsizeenum *)arg;
		struct ov529_format *format = NULL;
		struct ov529_frame *frame;
		int i;

		/* Look for the given pixel format */
		for (i = 0; i < ov529->nformats; i++) {
			if (ov529->fmt[i].fcc ==
					fsize->pixel_format) {
				format = &ov529->fmt[i];
				break;
			}
		}
		if (format == NULL)
			return -EINVAL;

		if (fsize->index >= format->nframes)
			return -EINVAL;

		frame = &format->frame[fsize->index];
		fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
		fsize->discrete.width = frame->width;
		fsize->discrete.height = frame->height;

		return ret;
	}
	case VIDIOC_ENUM_FRAMEINTERVALS:
	{
		struct v4l2_frmivalenum *fival = (struct v4l2_frmivalenum *)arg;
		struct ov529_format *format = NULL;
		struct ov529_frame *frame = NULL;
		int i;

		for (i = 0; i < ov529->nformats; i++) {
			if (ov529->fmt[i].fcc == fival->pixel_format) {
				format = &ov529->fmt[i];
				break;
			}
		}
		if (format == NULL)
			return -EINVAL;

		for (i = 0; i < format->nframes; i++) {
			if (format->frame[i].width == fival->width &&
			    format->frame[i].height == fival->height) {
				frame = &format->frame[i];
				break;
			}
		}
		if (frame == NULL)
			return -EINVAL;

		/* FIXME:
		 * Hard code, currently, CCIC outputs around 19.5MHz clock,
		 * which results in 25 fps.
		 */
		if (fival->index >= frame->interval_type)
				return -EINVAL;

		fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
		fival->discrete.numerator = frame->frameinterval;
		fival->discrete.denominator = 25;

		return 0;
	}
	default:
		break;
	}

	/* Handle other ioctl cmds with standard interface */
	ret = video_ioctl2(file, cmd, arg);

	return ret;
}

static int ov529_v4l_open(struct file *filp)
{
	struct ov529_encoder *ov529 = video_drvdata(filp);
	int ret = 0;

	if (ov529 == NULL)
		return -EINVAL;

	mutex_lock(&ov529->s_mutex);
	/* only support one ov529 now */
	ov529->curr_devid = 0;
	/* send clock, power on reset */
	ret = ov529->ops.startup();
	ov529->state = S_IDLE;
	mutex_unlock(&ov529->s_mutex);
	return ret;
}

static int ov529_v4l_release(struct file *filp)
{
	struct ov529_encoder *ov529 = video_drvdata(filp);

	mutex_lock(&ov529->s_mutex);
	ov529_free_buf(ov529);
	ov529->ops.remove();
	mutex_unlock(&ov529->s_mutex);

	return 0;
}

/* FIXME: Use mmap.
 * read is not supported currently.
 */
static ssize_t ov529_v4l_read(struct file *filp,
		char __user *buffer, size_t len, loff_t *pos)
{
	struct ov529_encoder *ov529 = video_drvdata(filp);

	mutex_lock(&ov529->s_mutex);

	mutex_unlock(&ov529->s_mutex);
	return 0;
}

static unsigned int ov529_v4l_poll(struct file *filp,
		struct poll_table_struct *pt)
{
	struct ov529_encoder *ov529 = video_drvdata(filp);

	poll_wait(filp, &ov529->iowait, pt);
	if (!list_empty(&ov529->buf_full))
		return POLLIN | POLLRDNORM;;

	return 0;
}

static void ov529_vm_open(struct vm_area_struct *vma)
{
	struct ov529_buffer *buffer = vma->vm_private_data;
	buffer->vma_use_count++;
}

static void ov529_vm_close(struct vm_area_struct *vma)
{
	struct ov529_buffer *buffer = vma->vm_private_data;
	buffer->vma_use_count--;
}

static struct vm_operations_struct ov529_vm_ops = {
	.open = ov529_vm_open,
	.close = ov529_vm_close
};

static int ov529_v4l_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct ov529_encoder *ov529 = video_drvdata(filp);
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	struct ov529_buffer *vbuf = NULL;
	int ret = 0;
	int i;

	mutex_lock(&ov529->s_mutex);

	for (i = 0; i < ov529->nbufs; i++) {
		if (ov529->buf[i].v4lbuf.m.offset == offset) {
			vbuf = ov529->buf + i;
			break;
		}
	}

	if (vbuf == NULL) {
		ret = -EINVAL;
		goto out;
	}

	ret = remap_vmalloc_range(vma, vbuf->buffer, 0);
	if (ret)
		goto out;
	vma->vm_flags |= VM_DONTEXPAND;
	vma->vm_private_data = vbuf;
	vma->vm_ops = &ov529_vm_ops;
	vbuf->v4lbuf.flags |= V4L2_BUF_FLAG_MAPPED;

	ov529_vm_open(vma);
out:
	mutex_unlock(&ov529->s_mutex);

	return 0;
}

static const struct v4l2_file_operations ov529_v4l_fops = {
	.owner = THIS_MODULE,
	.open = ov529_v4l_open,
	.release = ov529_v4l_release,
	.read = ov529_v4l_read,
	.poll = ov529_v4l_poll,
	.mmap = ov529_v4l_mmap,
	.ioctl = ov529_v4l_ioctl,
};

struct v4l2_ioctl_ops ov529_ioctl_ops = {
        .vidioc_querycap	= ov529_vidioc_querycap,		/* done */
        .vidioc_enum_fmt_vid_cap    = ov529_vidioc_enum_fmt_cap,	/* done */
        .vidioc_try_fmt_vid_cap     = ov529_vidioc_try_fmt_cap,		/* half done */
        .vidioc_s_fmt_vid_cap       = ov529_vidioc_s_fmt_cap,		/* half done */
        .vidioc_g_fmt_vid_cap       = ov529_vidioc_g_fmt_cap,		/* done */
        .vidioc_enum_input	= ov529_vidioc_enum_input,		/* done */
        .vidioc_g_input		= ov529_vidioc_g_input,			/* done */
        .vidioc_s_input		= ov529_vidioc_s_input,			/* done */
        .vidioc_reqbufs		= ov529_vidioc_reqbufs,			/* not */
        .vidioc_querybuf	= ov529_vidioc_querybuf,
        .vidioc_qbuf		= ov529_vidioc_qbuf,
        .vidioc_dqbuf		= ov529_vidioc_dqbuf,

        .vidioc_streamon	= ov529_vidioc_streamon,
        .vidioc_streamoff	= ov529_vidioc_streamoff,
/*
        .vidioc_queryctrl	= ov529_vidioc_queryctrl,
        .vidioc_g_ctrl		= ov529_vidioc_g_ctrl,
        .vidioc_s_ctrl		= ov529_vidioc_s_ctrl,
*/
        .vidioc_g_parm		= ov529_vidioc_g_parm,
        .vidioc_s_parm		= ov529_vidioc_s_parm,
/*
	.vidioc_cropcap		= ov529_vidioc_cropcap,
#ifdef CONFIG_VIDEO_ADV_DEBUG
        .vidioc_g_register	= ov529_vidioc_g_register,
        .vidioc_s_register	= ov529_vidioc_s_register,
#endif
*/
};

static int ov529_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret;
	struct video_device *vdev;
	struct ov529_encoder *ov529;
	struct ov529_format *fmt;

	/* Initialize ov529_encoder data structure */
	if ((ov529 = kzalloc(sizeof(struct ov529_encoder), GFP_KERNEL)) == NULL)
		return -ENOMEM;
	ov529->ops = ov529_smc_ops;
	strncpy(ov529->name, pdev->name, sizeof vdev->name);
	mutex_init(&ov529->s_mutex);
	spin_lock_init(&ov529->list_lock);

	/* find platform resource */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		printk("no IO memory resource defined\n");
		return -ENODEV;
	}

	/* init the ov529 hw interface */
	ret = ov529->ops.init(pdev, res);
	if (ret < 0) {
		printk(KERN_ERR "%s, %d: err=%d\n", __func__, __LINE__, ret);
		return ret;
	}

	/* register video_device */
	vdev = video_device_alloc();
	if (vdev == NULL)
		return -ENOMEM;

	vdev->parent = &pdev->dev;
	vdev->minor = -1;
	vdev->fops = &ov529_v4l_fops;
	vdev->release = video_device_release;
	vdev->ioctl_ops = &ov529_ioctl_ops;
	strncpy(vdev->name, pdev->name, sizeof vdev->name);

	/* keep ov529_encoder in video_device */
	video_set_drvdata(vdev, ov529);

	if (video_register_device(vdev, VFL_TYPE_GRABBER, -1) < 0) {
		/* dev->video.vdev = NULL; */
		ov529->ops.remove();
		video_device_release(vdev);
		kfree(ov529);
		return -EINVAL;
	}
	ov529->v4ldev = vdev;

	/* construct ov529 output format structure.
	 * now, only mjpeg will be used.
	 */
	fmt = kzalloc(sizeof(struct ov529_format), GFP_KERNEL);
	if (fmt == NULL) {
		ov529->ops.remove();
		video_unregister_device(vdev);
		kfree(ov529);
		return -ENOMEM;
	}

	fmt->type = FMT_MJPEG;
	fmt->index = 0;
	fmt->bpp = 0;
	strncpy(fmt->name, "MJPEG", sizeof fmt->name);
	fmt->fcc = V4L2_PIX_FMT_JPEG;
	fmt->flags = V4L2_FMT_FLAG_COMPRESSED;

	fmt->nframes = (sizeof(ov529_frames) / sizeof((ov529_frames)[0]));
	fmt->frame = &ov529_frames[0];
	fmt->curr_frame = fmt->frame;

	ov529->nformats = 1;
	ov529->fmt = ov529->curr_fmt = fmt;

	/* Initialize work queue for ov529 data transfer */
	workqueue = create_singlethread_workqueue("kov529d");
	if (!workqueue)
		return -ENOMEM;
	INIT_DELAYED_WORK(&ov529->iotransfer, ov529_iotransfer);
	init_waitqueue_head(&ov529->iowait);

	/* Initialize lists for queueing buffer */
	INIT_LIST_HEAD(&ov529->buf_avail);
	INIT_LIST_HEAD(&ov529->buf_full);

	return 0;
}


static int ov529_remove(struct platform_device *pdev)
{
	/* video_unregister_device(&cam->v4ldev); */
	destroy_workqueue(workqueue);
	/* kfree(ov529); */
	/* ov529->ops.remove(); */
	return 0;
}

#ifdef CONFIG_PM
/*
 * Basic power management.
 * TODO: Currently not implemented.
 */
static int ov529_suspend(struct platform_device *dev, pm_message_t state)
{
	return 0;
}

static int ov529_resume(struct platform_device *dev)
{
	return 0;
}

#endif  /* CONFIG_PM */

static struct platform_driver ov529_driver = {
	.driver = {
		.name = "pxa168-ov529"
	},
	.probe 		= ov529_probe,
	.remove 	= ov529_remove,
#ifdef CONFIG_PM
	.suspend 	= ov529_suspend,
	.resume 	= ov529_resume,
#endif

};

static int __devinit ov529_init(void)
{
	/* set current and last image size to 0 */
	current_imagesize = last_imagesize = 0;

#ifdef CONFIG_DVFM
	dvfm_register("OV529", &dvfm_dev_idx);
#endif
	return platform_driver_register(&ov529_driver);
}

static void __exit ov529_exit(void)
{
	platform_driver_unregister(&ov529_driver);
#ifdef CONFIG_DVFM
	dvfm_unregister("OV529", &dvfm_dev_idx);
#endif
}

module_init(ov529_init);
module_exit(ov529_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);
