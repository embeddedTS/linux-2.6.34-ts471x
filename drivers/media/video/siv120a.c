/*
 * A V4L2 driver for Seti SIV120A cameras.
 *
 * Copyright 2009 Marvell Inc.  Written
 * by Jun Nie with substantial information from Seti Inc.
 * This file may be distributed under the terms of the GNU General
 * Public License, version 2.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/videodev.h>
#include <media/v4l2-common.h>
#include <media/v4l2-chip-ident.h>
#include <linux/i2c.h>
#include <mach/hardware.h>
#include <mach/camera.h>

#include <linux/clk.h>
#include <mach/pxa168.h>
#include "pxa168_camera.h"

MODULE_AUTHOR("Jun Nie <njun@marvell.com>");
MODULE_DESCRIPTION("A low-level driver for Seti SIV120A sensors");
MODULE_LICENSE("GPL");


/*
 * Basic window sizes.  These probably belong somewhere more globally
 * useful.
 */
#define VGA_WIDTH	640
#define VGA_HEIGHT	480
#define QVGA_WIDTH	320
#define QVGA_HEIGHT	240
#define CIF_WIDTH	352
#define CIF_HEIGHT	288
#define QCIF_WIDTH	176
#define	QCIF_HEIGHT	144

/*
 * Our nominal (default) frame rate.
 */
#define SIV120A_FRAME_RATE 30

/*
 * The SIV120A sits on i2c with ID 0x66
 */
#define SIV120A_I2C_ADDR 0x66

/* Sensor Registers */
#define REG_CHIP_ID 0x1
#define REG_VERSION 0x2

/*
 * This matrix defines how the colors are generated, must be
 * tweaked to adjust hue and saturation.
 *
 * Order: v-red, v-green, v-blue, u-red, u-green, u-blue
 *
 * They are nine-bit signed quantities, with the sign bit
 * stored in 0x58.  Sign for v-red is bit 0, and up from there.
 */
#define	REG_CMATRIX_BASE 0x54
#define	REG_CMATRIX_END 0x6E

struct regval_list {
	unsigned char reg_num;
	unsigned char value;
};

#if 0
/*
 * This matrix defines how the colors are generated, must be
 * tweaked to adjust hue and saturation.
 *
 * Order: v-red, v-green, v-blue, u-red, u-green, u-blue
 *
 * They are nine-bit signed quantities, with the sign bit
 * stored in 0x58.  Sign for v-red is bit 0, and up from there.
 */


/*
 * Information we maintain about a known sensor.
 */
struct siv120a_format_struct;  /* coming later */
#endif
struct siv120a_info {
	struct siv120a_format_struct *fmt;  /* Current format */
	unsigned char sat;		/* Saturation value */
	int hue;			/* Hue value */
};

/*
 *
 * YUV422, modified from seti setting
 *
 */
static struct regval_list siv120a_init_reg[] = {
	{0x00, 0x00},
	{0x03, 0x55},
	{0x04, 0x00},
        {0x05, 0x06},  /* VGA  */
	{0x10, 0x01},
	{0x13, 0x1a},
	{0x17, 0x82},
	{0x42, 0x32},
	{0x43, 0x80},
	{0xff, 0xff},
};

static struct regval_list siv120a_fmt_yuv422[] = {

	//#AE Block: expose
	{0x00, 0x01},
	{0x10, 0x00},
	{0x11, 0x10},
	{0x40, 0x5f},

	//AWB Block: white balance
	{0x00, 0x02},
	{0x10, 0xd3},
	{0x11, 0x00},
	{0x15, 0xfe},
	{0x16, 0x80},
	{0x17, 0xea},
	{0x18, 0x80},
	{0x19, 0xb0},
	{0x1a, 0x68},
	{0x1b, 0xb0},
	{0x1c, 0x64},
	{0x1d, 0x90},
	{0x1e, 0x70},
	{0x20, 0xe8},
	{0x21, 0x20},
	{0x22, 0x9d},
	{0x23, 0x18},
	{0x25, 0x20},
	{0x26, 0x0f},
	{0x27, 0x0d},
	{0x28, 0x90},
	{0x29, 0xd0},
	{0x2a, 0x90},
	{0x30, 0x05},
	{0x46, 0x64},

	//RGB to YCbCr (CSC) no used
	{0x50, 0x33},
	{0x51, 0x20},
	{0x52, 0xe5},
	{0x53, 0xfb},
	{0x54, 0x13},
	{0x55, 0x26},
	{0x56, 0x07},
	{0x57, 0xf5},
	{0x58, 0xea},
	{0x59, 0x21},
	{0x63, 0xbd},
	{0x64, 0xc2},
	{0x65, 0xbd},
	{0x66, 0xc2},
	{0x67, 0xef},
	{0x68, 0xa0},
	{0x69, 0xef},
	{0x6a, 0xa0},

	//#### IDP
	{0x00, 0x03},
	{0x10, 0xEF},
	//{0x12, 0x3D}, // cmcs Selects YUV422 mode */
	{0x12, 0x1D}, // UYVY YUV422 mode */
	{0x11, 0x5d}, // sync timing
	{0x14, 0xf5},

	//#shading _0415
	{0x2c, 0x44},
	{0x2d, 0x22},
	{0X2e, 0x11},
	{0x32, 0xA8},

	//#Gamma
	{0x34, 0x00},
	{0x35, 0x08},
	{0x36, 0x11},
	{0x37, 0x25},
	{0x38, 0x45},
	{0x39, 0x5f},
	{0x3a, 0x74},
	{0x3b, 0x87},
	{0x3c, 0x97},
	{0x3d, 0xa5},
	{0x3e, 0xb2},
	{0x3f, 0xc9},
	{0x40, 0xdd},
	{0x41, 0xf0},
	{0x42, 0xf8},
	{0x43, 0xff},
	{0x44, 0xbb},
	{0x4b, 0x40},
	{0x4c, 0xa1},
	{0x4d, 0x08},
	{0x4e, 0xff},
	{0x4f, 0x05},
	{0x50, 0x80},
	{0x51, 0x40},
	{0x52, 0xff},
	{0x53, 0x13},

	{0x70, 0x20},
	{0x71, 0x30},
	{0x73, 0x06},
	{0x74, 0x06},
	{0x76, 0x40},
	{0x79, 0x20},
	{0x7a, 0x30},
	{0x7b, 0x06},
	{0x7c, 0x06},
	{0x7d, 0x20},
	{0x7f, 0x40},

	{0x86, 0x10},
	{0x87, 0x10},

	{0x92, 0x44},
	//# windows size
	{0xa0, 0x24},
	{0xa1, 0x00},
	{0xa2, 0x80},
	{0xa3, 0x00},
	{0xa4, 0xE0},

	//#Sensor On
	{0x00, 0x00},
	{0x03, 0x55},

	//#AE On  
	{0x00, 0x01},
	{0x10, 0x80},
	//[END]

	{0x00, 0x00},
	{0xff, 0xff},
};

/*
 * Low-level register I/O.
 */

static int siv120a_read(struct i2c_client *c, unsigned char reg,
		unsigned char *value)
{
	int ret;

	ret = i2c_smbus_read_byte_data(c, reg);
	if (ret >= 0)
		*value = (unsigned char) ret;
	return ret;
}

static int siv120a_write(struct i2c_client *c, unsigned char reg,
		unsigned char value)
{
	int ret = i2c_smbus_write_byte_data(c, reg, value);
	return ret;
}

/*
 * Write a list of register settings; ff/ff stops the process.
 */
static int siv120a_write_array(struct i2c_client *c, struct regval_list *vals)
{
	int i = 0;
	while (vals->reg_num != 0xff || vals->value != 0xff) {
		int ret = siv120a_write(c, vals->reg_num, vals->value);
		if (ret < 0)
			return ret;
		vals++;
		if (i == 0)
			mdelay(5);
		i++;
	}
	return 0;
}


static int siv120a_init(struct i2c_client *client)
{
	return siv120a_write_array(client, siv120a_init_reg);
}

static int siv120a_detect(struct i2c_client *client)
{
	unsigned char v;
	int ret;

	/*ret = siv120a_init(client);
	if (ret < 0)
		return ret;
		*/
	ret = siv120a_read(client, REG_CHIP_ID, &v);
	if (ret < 0)
		return ret;
	if (v != 0x12) /* chip id. */
		return -ENODEV;

	ret = siv120a_read(client, REG_VERSION, &v);
	printk("find seti camera chip version 0x%x\n", v);
	if (ret < 0)
		return ret;
	return 0;
}

#if 0
/*
 * Stuff that knows about the sensor.
 */
static void siv120a_reset(struct i2c_client *client)
{
	siv120a_write(client, REG_COM7, COM7_RESET);
	msleep(1);
}

#endif
/*
 * Store information about the video data format.  The color matrix
 * is deeply tied into the format, so keep the relevant values here.
 * The magic matrix nubmers come from OmniVision.
 */
static struct siv120a_format_struct {
	__u8 *desc;
	__u32 pixelformat;
	struct regval_list *regs;
	//int cmatrix[CMATRIX_LEN];
	int bpp;   /* bits per pixel */
} siv120a_formats[] = {
	/*{
		.desc		= "YUYV 4:2:2",
		.pixelformat	= V4L2_PIX_FMT_YUYV,
		.regs 		= siv120a_fmt_yuv422,
		.cmatrix	= { 128, -128, 0, -34, -94, 128 },
		.bpp		= 16,
	},
	{
		.desc		= "YUYV 4:2:2",
		.pixelformat	= V4L2_PIX_FMT_YUV422P,
		.regs 		= siv120a_fmt_yuv422,
		.cmatrix	= { 128, -128, 0, -34, -94, 128 },
		.bpp		= 16,
	},*/
	{
		.desc           = "YUYV 4:2:0",
		.pixelformat    = V4L2_PIX_FMT_YUV420,
		.regs           = siv120a_fmt_yuv422,
		//.cmatrix        = { 128, -128, 0, -34, -94, 128 },
		.bpp            = 12,
	},
	/*{
		.desc		= "RGB 565",
		.pixelformat	= V4L2_PIX_FMT_RGB565,
		.regs		= siv120a_rgb565_qvga,
		//.cmatrix	= { 179, -179, 0, -61, -176, 228 },
		.bpp		= 16,
	},
	{
		.desc		= "RGB 444",
		.pixelformat	= V4L2_PIX_FMT_RGB444,
		.regs		= siv120a_fmt_rgb444,
		.cmatrix	= { 179, -179, 0, -61, -176, 228 },
		.bpp		= 16,
	},
	{
		.desc		= "Raw RGB Bayer",
		.pixelformat	= V4L2_PIX_FMT_SBGGR8,
		.regs 		= siv120a_fmt_raw,
		.cmatrix	= { 0, 0, 0, 0, 0, 0 },
		.bpp		= 8,
	},*/
};
#define N_SIV120A_FMTS ARRAY_SIZE(siv120a_formats)

/*
 * Then there is the issue of window sizes.  Try to capture the info here.
 */

/*
 * QCIF mode is done (by OV) in a very strange way - it actually looks like
 * VGA with weird scaling options - they do *not* use the canned QCIF mode
 * which is allegedly provided by the sensor.  So here's the weird register
 * settings.
 */
static struct regval_list siv120a_qcif_regs[] = {
};

static struct siv120a_win_size {
	int	width;
	int	height;
	unsigned char com7_bit;
	int	hstart;		/* Start/stop values for the camera.  Note */
	int	hstop;		/* that they do not always make complete */
	int	vstart;		/* sense to humans, but evidently the sensor */
	int	vstop;		/* will do the right thing... */
	struct regval_list *regs; /* Regs to tweak */
	/* h/vref stuff */
} siv120a_win_sizes[] = {
	/* VGA */
	{
		.width		= VGA_WIDTH,
		.height		= VGA_HEIGHT,
		//.com7_bit	= COM7_FMT_VGA,
		.hstart		= 158,
		.hstop		=  14,
		.vstart		=  10,
		.vstop		= 490,
		.regs 		= NULL,
	},
	/* QVGA */
#if 0
	{
		.width		= QVGA_WIDTH,
		.height		= QVGA_HEIGHT,
		//.com7_bit	= COM7_FMT_QVGA,
		.hstart		= 164,
		.hstop		=  20,
		.vstart		=  14,
		.vstop		= 494,
		.regs 		= siv120a_qvga,
	},
	/* CIF */
	{
		.width		= CIF_WIDTH,
		.height		= CIF_HEIGHT,
		.com7_bit	= COM7_FMT_CIF,
		.hstart		= 170,		/* Empirically determined */
		.hstop		=  90,
		.vstart		=  14,
		.vstop		= 494,
		.regs 		= NULL,
	},
	/* QCIF */
	{
		.width		= QCIF_WIDTH,
		.height		= QCIF_HEIGHT,
		.com7_bit	= COM7_FMT_VGA, /* see comment above */
		.hstart		= 456,		/* Empirically determined */
		.hstop		=  24,
		.vstart		=  14,
		.vstop		= 494,
		.regs 		= siv120a_qcif_regs,
	},
#endif
};

#define N_WIN_SIZES (ARRAY_SIZE(siv120a_win_sizes))

#if 0
/*
 * Store a set of start/stop values into the camera.
 */
static int siv120a_set_hw(struct i2c_client *client, int hstart, int hstop,
		int vstart, int vstop)
{
	int ret;
	unsigned char v;
	/*
	 * Horizontal: 11 bits, top 8 live in hstart and hstop.  Bottom 3 of
	 * hstart are in href[2:0], bottom 3 of hstop in href[5:3].  There is
	 * a mystery "edge offset" value in the top two bits of href.
	 */
	ret =  siv120a_write(client, REG_HSTART, (hstart >> 3) & 0xff);
	ret += siv120a_write(client, REG_HSTOP, (hstop >> 3) & 0xff);
	ret += siv120a_read(client, REG_HREF, &v);
	v = (v & 0xc0) | ((hstop & 0x7) << 3) | (hstart & 0x7);
	msleep(10);
	ret += siv120a_write(client, REG_HREF, v);
	/*
	 * Vertical: similar arrangement, but only 10 bits.
	 */
	ret += siv120a_write(client, REG_VSTART, (vstart >> 2) & 0xff);
	ret += siv120a_write(client, REG_VSTOP, (vstop >> 2) & 0xff);
	ret += siv120a_read(client, REG_VREF, &v);
	v = (v & 0xf0) | ((vstop & 0x3) << 2) | (vstart & 0x3);
	msleep(10);
	ret += siv120a_write(client, REG_VREF, v);
	return ret;
}

#endif

static int siv120a_enum_fmt(struct i2c_client *c, struct v4l2_fmtdesc *fmt)
{
	struct siv120a_format_struct *ofmt;

	if (fmt->index >= N_SIV120A_FMTS)
		return -EINVAL;

	ofmt = siv120a_formats + fmt->index;
	fmt->flags = 0;
	strcpy(fmt->description, ofmt->desc);
	fmt->pixelformat = ofmt->pixelformat;
	return 0;
}

static int siv120a_try_fmt(struct i2c_client *c, struct v4l2_format *fmt,
		struct siv120a_format_struct **ret_fmt,
		struct siv120a_win_size **ret_wsize)
{
	int index;
	struct siv120a_win_size *wsize;
	struct v4l2_pix_format *pix = &fmt->fmt.pix;

	for (index = 0; index < N_SIV120A_FMTS; index++)
		if (siv120a_formats[index].pixelformat == pix->pixelformat)
			break;
	if (index >= N_SIV120A_FMTS)
		return -EINVAL;
	if (ret_fmt != NULL)
		*ret_fmt = siv120a_formats + index;
	/*
	 * Fields: the OV devices claim to be progressive.
	 */
	/*if (pix->field == V4L2_FIELD_ANY)
		pix->field = V4L2_FIELD_NONE;
	else if (pix->field != V4L2_FIELD_NONE)
		return -EINVAL;
		*/
	/*
	 * Round requested image size down to the nearest
	 * we support, but not below the smallest.
	 */
#if 0
	for (wsize = siv120a_win_sizes; wsize < siv120a_win_sizes + N_WIN_SIZES;
			wsize++)
		if (pix->width >= wsize->width && pix->height >= wsize->height)
			break;
	if (wsize >= siv120a_win_sizes + N_WIN_SIZES)
		wsize--;   /* Take the smallest one */
	if (ret_wsize != NULL)
		*ret_wsize = wsize;
	/*
	 * Note the size we'll actually handle.
	 */
	pix->width = wsize->width;
	pix->height = wsize->height;
#endif
	pix->bytesperline = pix->width*siv120a_formats[index].bpp/8;
	pix->sizeimage = pix->height*pix->bytesperline;
	return 0;
}

/*
 * Set a format.
 */
static int siv120a_s_fmt(struct i2c_client *c, struct v4l2_format *fmt)
{
	int ret;
	struct siv120a_format_struct *ovfmt;
	struct siv120a_win_size *wsize;

	ret = siv120a_try_fmt(c, fmt, &ovfmt, &wsize);
	if (ret)
		return ret;

	siv120a_write_array(c, ovfmt->regs);

	return ret;
}

/*
 * Implement G/S_PARM.  There is a "high quality" mode we could try
 * to do someday; for now, we just do the frame rate tweak.
 */
static int siv120a_g_parm(struct i2c_client *c, struct v4l2_streamparm *parms)
{
	struct v4l2_captureparm *cp = &parms->parm.capture;
	unsigned char clkrc;
	int ret;

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	/*ret = siv120a_read(c, REG_CLKRC, &clkrc);
	if (ret < 0)
		return ret;
	*/
	memset(cp, 0, sizeof(struct v4l2_captureparm));
	cp->capability = V4L2_CAP_TIMEPERFRAME;
	cp->timeperframe.numerator = 1;
	cp->timeperframe.denominator = SIV120A_FRAME_RATE;
	/*if ((clkrc & CLK_EXT) == 0 && (clkrc & CLK_SCALE) > 1)
		cp->timeperframe.denominator /= (clkrc & CLK_SCALE);
		*/
	return 0;
}

static int siv120a_s_parm(struct i2c_client *c, struct v4l2_streamparm *parms)
{
	struct v4l2_captureparm *cp = &parms->parm.capture;
	struct v4l2_fract *tpf = &cp->timeperframe;
	unsigned char clkrc;
	int ret, div;

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	if (cp->extendedmode != 0)
		return -EINVAL;
	/*
	 * CLKRC has a reserved bit, so let's preserve it.
	 */
	/*ret = siv120a_read(c, REG_CLKRC, &clkrc);
	if (ret < 0)
		return ret;
		*/
	if (tpf->numerator == 0 || tpf->denominator == 0)
		div = 1;  /* Reset to full rate */
	else
		div = (tpf->numerator*SIV120A_FRAME_RATE)/tpf->denominator;
	/*if (div == 0)
		div = 1;
	else if (div > CLK_SCALE)
		div = CLK_SCALE;
	clkrc = (clkrc & 0x80) | div;
	*/
	tpf->numerator = 1;
	tpf->denominator = SIV120A_FRAME_RATE/div;
	return 0;
	//return siv120a_write(c, REG_CLKRC, clkrc);
}

static int siv120a_s_input(struct i2c_client *c, int *id)
{
	return 0;
}
#if 0

/*
 * Code for dealing with controls.
 */





static int siv120a_store_cmatrix(struct i2c_client *client,
		int matrix[CMATRIX_LEN])
{
	int i, ret;
	unsigned char signbits;

	/*
	 * Weird crap seems to exist in the upper part of
	 * the sign bits register, so let's preserve it.
	 */
	ret = siv120a_read(client, REG_CMATRIX_SIGN, &signbits);
	signbits &= 0xc0;

	for (i = 0; i < CMATRIX_LEN; i++) {
		unsigned char raw;

		if (matrix[i] < 0) {
			signbits |= (1 << i);
			if (matrix[i] < -255)
				raw = 0xff;
			else
				raw = (-1 * matrix[i]) & 0xff;
		}
		else {
			if (matrix[i] > 255)
				raw = 0xff;
			else
				raw = matrix[i] & 0xff;
		}
		ret += siv120a_write(client, REG_CMATRIX_BASE + i, raw);
	}
	ret += siv120a_write(client, REG_CMATRIX_SIGN, signbits);
	return ret;
}


/*
 * Hue also requires messing with the color matrix.  It also requires
 * trig functions, which tend not to be well supported in the kernel.
 * So here is a simple table of sine values, 0-90 degrees, in steps
 * of five degrees.  Values are multiplied by 1000.
 *
 * The following naive approximate trig functions require an argument
 * carefully limited to -180 <= theta <= 180.
 */
#define SIN_STEP 5
static const int siv120a_sin_table[] = {
	0,	 87,   173,   258,   342,   422,
	499,	573,   642,   707,   766,   819,
	866,	906,   939,   965,   984,   996,
	1000
};

static int siv120a_sine(int theta)
{
	int chs = 1;
	int sine;

	if (theta < 0) {
		theta = -theta;
		chs = -1;
	}
	if (theta <= 90)
		sine = siv120a_sin_table[theta/SIN_STEP];
	else {
		theta -= 90;
		sine = 1000 - siv120a_sin_table[theta/SIN_STEP];
	}
	return sine*chs;
}

static int siv120a_cosine(int theta)
{
	theta = 90 - theta;
	if (theta > 180)
		theta -= 360;
	else if (theta < -180)
		theta += 360;
	return siv120a_sine(theta);
}




static void siv120a_calc_cmatrix(struct siv120a_info *info,
		int matrix[CMATRIX_LEN])
{
	int i;
	/*
	 * Apply the current saturation setting first.
	 */
	for (i = 0; i < CMATRIX_LEN; i++)
		matrix[i] = (info->fmt->cmatrix[i]*info->sat) >> 7;
	/*
	 * Then, if need be, rotate the hue value.
	 */
	if (info->hue != 0) {
		int sinth, costh, tmpmatrix[CMATRIX_LEN];

		memcpy(tmpmatrix, matrix, CMATRIX_LEN*sizeof(int));
		sinth = siv120a_sine(info->hue);
		costh = siv120a_cosine(info->hue);

		matrix[0] = (matrix[3]*sinth + matrix[0]*costh)/1000;
		matrix[1] = (matrix[4]*sinth + matrix[1]*costh)/1000;
		matrix[2] = (matrix[5]*sinth + matrix[2]*costh)/1000;
		matrix[3] = (matrix[3]*costh - matrix[0]*sinth)/1000;
		matrix[4] = (matrix[4]*costh - matrix[1]*sinth)/1000;
		matrix[5] = (matrix[5]*costh - matrix[2]*sinth)/1000;
	}
}



static int siv120a_t_sat(struct i2c_client *client, int value)
{
	struct siv120a_info *info = i2c_get_clientdata(client);
	int matrix[CMATRIX_LEN];
	int ret;

	info->sat = value;
	siv120a_calc_cmatrix(info, matrix);
	ret = siv120a_store_cmatrix(client, matrix);
	return ret;
}

static int siv120a_q_sat(struct i2c_client *client, __s32 *value)
{
	struct siv120a_info *info = i2c_get_clientdata(client);

	*value = info->sat;
	return 0;
}

static int siv120a_t_hue(struct i2c_client *client, int value)
{
	struct siv120a_info *info = i2c_get_clientdata(client);
	int matrix[CMATRIX_LEN];
	int ret;

	if (value < -180 || value > 180)
		return -EINVAL;
	info->hue = value;
	siv120a_calc_cmatrix(info, matrix);
	ret = siv120a_store_cmatrix(client, matrix);
	return ret;
}


static int siv120a_q_hue(struct i2c_client *client, __s32 *value)
{
	struct siv120a_info *info = i2c_get_clientdata(client);

	*value = info->hue;
	return 0;
}


/*
 * Some weird registers seem to store values in a sign/magnitude format!
 */
static unsigned char siv120a_sm_to_abs(unsigned char v)
{
	if ((v & 0x80) == 0)
		return v + 128;
	else
		return 128 - (v & 0x7f);
}


static unsigned char siv120a_abs_to_sm(unsigned char v)
{
	if (v > 127)
		return v & 0x7f;
	else
		return (128 - v) | 0x80;
}

static int siv120a_t_brightness(struct i2c_client *client, int value)
{
	unsigned char com8, v;
	int ret;

	siv120a_read(client, REG_COM8, &com8);
	com8 &= ~COM8_AEC;
	siv120a_write(client, REG_COM8, com8);
	v = siv120a_abs_to_sm(value);
	ret = siv120a_write(client, REG_BRIGHT, v);
	return ret;
}

static int siv120a_q_brightness(struct i2c_client *client, __s32 *value)
{
	unsigned char v;
	int ret = siv120a_read(client, REG_BRIGHT, &v);

	*value = siv120a_sm_to_abs(v);
	return ret;
}

static int siv120a_t_contrast(struct i2c_client *client, int value)
{
	return siv120a_write(client, REG_CONTRAS, (unsigned char) value);
}

static int siv120a_q_contrast(struct i2c_client *client, __s32 *value)
{
	unsigned char v;
	int ret = siv120a_read(client, REG_CONTRAS, &v);

	*value = v;
	return ret;
}

static int siv120a_q_hflip(struct i2c_client *client, __s32 *value)
{
	int ret;
	unsigned char v;

	ret = siv120a_read(client, REG_MVFP, &v);
	*value = (v & MVFP_MIRROR) == MVFP_MIRROR;
	return ret;
}


static int siv120a_t_hflip(struct i2c_client *client, int value)
{
	unsigned char v;
	int ret;

	ret = siv120a_read(client, REG_MVFP, &v);
	if (value)
		v |= MVFP_MIRROR;
	else
		v &= ~MVFP_MIRROR;
	msleep(10);  /* FIXME */
	ret += siv120a_write(client, REG_MVFP, v);
	return ret;
}



static int siv120a_q_vflip(struct i2c_client *client, __s32 *value)
{
	int ret;
	unsigned char v;

	ret = siv120a_read(client, REG_MVFP, &v);
	*value = (v & MVFP_FLIP) == MVFP_FLIP;
	return ret;
}


static int siv120a_t_vflip(struct i2c_client *client, int value)
{
	unsigned char v;
	int ret;

	ret = siv120a_read(client, REG_MVFP, &v);
	if (value)
		v |= MVFP_FLIP;
	else
		v &= ~MVFP_FLIP;
	msleep(10);  /* FIXME */
	ret += siv120a_write(client, REG_MVFP, v);
	return ret;
}


static struct siv120a_control {
	struct v4l2_queryctrl qc;
	int (*query)(struct i2c_client *c, __s32 *value);
	int (*tweak)(struct i2c_client *c, int value);
} siv120a_controls[] =
{
	{
		.qc = {
			.id = V4L2_CID_BRIGHTNESS,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Brightness",
			.minimum = 0,
			.maximum = 255,
			.step = 1,
			.default_value = 0x80,
			.flags = V4L2_CTRL_FLAG_SLIDER
		},
		.tweak = siv120a_t_brightness,
		.query = siv120a_q_brightness,
	},
	{
		.qc = {
			.id = V4L2_CID_CONTRAST,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Contrast",
			.minimum = 0,
			.maximum = 127,
			.step = 1,
			.default_value = 0x40,   /* XXX siv120a spec */
			.flags = V4L2_CTRL_FLAG_SLIDER
		},
		.tweak = siv120a_t_contrast,
		.query = siv120a_q_contrast,
	},
	{
		.qc = {
			.id = V4L2_CID_SATURATION,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Saturation",
			.minimum = 0,
			.maximum = 256,
			.step = 1,
			.default_value = 0x80,
			.flags = V4L2_CTRL_FLAG_SLIDER
		},
		.tweak = siv120a_t_sat,
		.query = siv120a_q_sat,
	},
	{
		.qc = {
			.id = V4L2_CID_HUE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "HUE",
			.minimum = -180,
			.maximum = 180,
			.step = 5,
			.default_value = 0,
			.flags = V4L2_CTRL_FLAG_SLIDER
		},
		.tweak = siv120a_t_hue,
		.query = siv120a_q_hue,
	},
	{
		.qc = {
			.id = V4L2_CID_VFLIP,
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.name = "Vertical flip",
			.minimum = 0,
			.maximum = 1,
			.step = 1,
			.default_value = 0,
		},
		.tweak = siv120a_t_vflip,
		.query = siv120a_q_vflip,
	},
	{
		.qc = {
			.id = V4L2_CID_HFLIP,
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.name = "Horizontal mirror",
			.minimum = 0,
			.maximum = 1,
			.step = 1,
			.default_value = 0,
		},
		.tweak = siv120a_t_hflip,
		.query = siv120a_q_hflip,
	},
};
#define N_CONTROLS (ARRAY_SIZE(siv120a_controls))

static struct siv120a_control *siv120a_find_control(__u32 id)
{
	int i;

	for (i = 0; i < N_CONTROLS; i++)
		if (siv120a_controls[i].qc.id == id)
			return siv120a_controls + i;
	return NULL;
}


static int siv120a_queryctrl(struct i2c_client *client,
		struct v4l2_queryctrl *qc)
{
	struct siv120a_control *ctrl = siv120a_find_control(qc->id);

	if (ctrl == NULL)
		return -EINVAL;
	*qc = ctrl->qc;
	return 0;
}

static int siv120a_g_ctrl(struct i2c_client *client, struct v4l2_control *ctrl)
{
	struct siv120a_control *octrl = siv120a_find_control(ctrl->id);
	int ret;

	if (octrl == NULL)
		return -EINVAL;
	ret = octrl->query(client, &ctrl->value);
	if (ret >= 0)
		return 0;
	return ret;
}

static int siv120a_s_ctrl(struct i2c_client *client, struct v4l2_control *ctrl)
{
	struct siv120a_control *octrl = siv120a_find_control(ctrl->id);
	int ret;

	if (octrl == NULL)
		return -EINVAL;
	ret =  octrl->tweak(client, ctrl->value);
	if (ret >= 0)
		return 0;
	return ret;
}


#endif

int ccic_sensor_attach(struct i2c_client *client);
extern struct clk *pxa168_ccic_gate_clk;

/*
 * Basic i2c stuff.
 */
static int __devinit siv120a_probe(struct i2c_client *client)
{
	int ret;
	struct siv120a_info *info;
	struct sensor_platform_data *pdata;
	pdata = client->dev.platform_data;

	/*
	 * Set up our info structure.
	 */
	clk_enable(pxa168_ccic_gate_clk);
	ccic_set_clock_parallel();	//clock must be enabled before power on.

	pdata->power_on(1, 0);

	info = kzalloc(sizeof (struct siv120a_info), GFP_KERNEL);
	if (! info) {
		ret = -ENOMEM;
		goto out_free;
	}
	info->fmt = &siv120a_formats[0];
	info->sat = 128;	/* Review this */
	i2c_set_clientdata(client, info);

	/*
	 * Make sure it's an siv120a
	 */
	ret = siv120a_detect(client);
	if (ret) {
		printk("%s: failed to detect siv120a!\n", __func__);
		goto out_free_info;
	}
	printk(KERN_NOTICE "siv120a sensor detected\n");

	ret = ccic_sensor_attach(client);
	if (ret)
		goto out_free_info;
	pdata->power_on(0, 0);	//for power optimization
	ccic_disable_clock();
	return 0;

out_free_info:
	kfree(info);
out_free:
	return ret;
}


static int siv120a_remove(struct i2c_client *client)
{
	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int siv120a_g_register(struct i2c_client *client, struct v4l2_dbg_register * reg)
{
	return siv120a_read(client, (unsigned char)reg->reg, (unsigned char *)&(reg->val));
}

static int siv120a_s_register(struct i2c_client *client, struct v4l2_dbg_register * reg)
{
	return siv120a_write(client, (unsigned char)reg->reg, (unsigned char)reg->val);
}
#endif

static int siv120a_command(struct i2c_client *client, unsigned int cmd,
		void *arg)
{
	switch (cmd) {
		case VIDIOC_DBG_G_CHIP_IDENT:
			return v4l2_chip_ident_i2c_client(client, arg, V4L2_IDENT_SIV120A, 0);

		case VIDIOC_INT_RESET:
			//siv120a_reset(client);
			return 0;

		case VIDIOC_INT_INIT:
			return siv120a_init(client);

		case VIDIOC_ENUM_FMT:
			return siv120a_enum_fmt(client, (struct v4l2_fmtdesc *) arg);
		case VIDIOC_TRY_FMT:
			return siv120a_try_fmt(client, (struct v4l2_format *) arg, NULL, NULL);
		case VIDIOC_S_FMT:
			return siv120a_s_fmt(client, (struct v4l2_format *) arg);
		case VIDIOC_QUERYCTRL:
			return 0;
			//return siv120a_queryctrl(client, (struct v4l2_queryctrl *) arg);
		case VIDIOC_S_CTRL:
			return 0;
			//return siv120a_s_ctrl(client, (struct v4l2_control *) arg);
		case VIDIOC_G_CTRL:
			return 0;
			//return siv120a_g_ctrl(client, (struct v4l2_control *) arg);
		case VIDIOC_S_PARM:
			return siv120a_s_parm(client, (struct v4l2_streamparm *) arg);
		case VIDIOC_G_PARM:
			return siv120a_g_parm(client, (struct v4l2_streamparm *) arg);
		case VIDIOC_S_INPUT:
			return siv120a_s_input(client, (int *) arg);
#ifdef CONFIG_VIDEO_ADV_DEBUG
		case VIDIOC_DBG_G_REGISTER:
			return siv120a_g_register(client, (struct v4l2_dbg_register *) arg);
		case VIDIOC_DBG_S_REGISTER:
			return siv120a_s_register(client, (struct v4l2_dbg_register *) arg);
#endif
		default:
			printk("no handled siv120 cmd %x\n", cmd);
			return 0;
	}
	return -EINVAL;
}

static struct i2c_device_id siv120a_idtable[] = {
	{ "siv120a", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, siv120a_idtable);

static struct i2c_driver siv120a_driver = {
	.driver = {
		.name	= "siv120a",
	},
	.id_table       = siv120a_idtable,
	.command	= siv120a_command,
	.probe		= siv120a_probe,
	.remove		= siv120a_remove,
};


/*
 * Module initialization
 */
static int __init siv120a_mod_init(void)
{
	printk(KERN_NOTICE "NiceSeti siv120a sensor driver, at your service\n");
	return i2c_add_driver(&siv120a_driver);
}

static void __exit siv120a_mod_exit(void)
{
	i2c_del_driver(&siv120a_driver);
}

late_initcall(siv120a_mod_init);
module_exit(siv120a_mod_exit);

