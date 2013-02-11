/*
 * Provide definitions for the ov529 camera controler driver.
 *
 * Copyright 2009 Marvell International Ltd.
 * 		Weili Xia <wlxia@marvell.com>
 *
 * This file may be distributed under the terms of the GNU General
 * Public License, version 2.
 */

#define VGA_WIDTH	640
#define VGA_HEIGHT	480
#define CIF_WIDTH	352
#define CIF_HEIGHT	288

enum ov529_fmt_type {
	FMT_UNDEFINED,
	FMT_MJPEG,
};

enum ov529_state {
	S_NOTREADY,	/* Not yet initialized */
	S_IDLE,		/* Just hanging around */
	S_FLAKED,	/* Some sort of problem */
	S_SINGLEREAD,	/* In read() */
	S_SPECREAD,   	/* Speculative read (for future read()) */
	S_STREAMING	/* Streaming data */
};

struct ov529_frame {
	char  index;
	int width;
	int height;
	char  interval_type;
	unsigned int frameinterval;
};


struct ov529_format {
	char type;
	char index;
	char bpp;
	char colorspace;
	unsigned int fcc;
	unsigned int flags;

	char name[32];

	unsigned int nframes;
	struct ov529_frame *frame;
	struct ov529_frame *curr_frame;
};

struct ov529_buffer {
	struct list_head list;
	struct v4l2_buffer v4lbuf;
	char *buffer;   /* Where it lives in kernel space */
	struct ov529_encoder *owner;
	struct vm_area_struct *svma;
	unsigned long vma_use_count;
};

struct ov529_encoder {
	enum ov529_state state;
	char name[32];

	struct ov529_hw_ops ops;

	struct platform_device *pdev;
	struct video_device *v4ldev;
	unsigned int curr_devid;

	unsigned int nformats;
	struct ov529_format *fmt;
	struct ov529_format *curr_fmt;

	unsigned int nbufs;		/* How many are alloc'd */
	int next_buf;			/* Next to consume (dev_lock) */
	struct ov529_buffer	*buf;	/* Only one buffer avail */
	struct list_head buf_avail;	/* Available for data (we own) (dev_lock) */
	struct list_head buf_full;	/* With data (user space owns) (dev_lock) */

	struct delayed_work	iotransfer;
	wait_queue_head_t	iowait;

	struct mutex s_mutex; /* Access to this structure */
	spinlock_t list_lock;  /* Access to device */
};
