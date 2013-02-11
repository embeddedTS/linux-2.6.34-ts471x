/*
 * A V4L2 driver for OmniVision OV7740 cameras.
 *
 * Copyright 2006 One Laptop Per Child Association, Inc.  Written
 * by Jonathan Corbet with substantial inspiration from Mark
 * McClelland's ovcamchip code.
 *
 * Copyright 2006-7 Jonathan Corbet <corbet@lwn.net>
 *
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
#include <asm/mach-types.h>
#include "pxa168_camera.h"

MODULE_AUTHOR("Tony Teng <ylteng@marvell.com>");
MODULE_DESCRIPTION("pxa168 low-level OmniVision ov7740 sensors");
MODULE_LICENSE("GPL");


#define DEBUG_YUV422	//what will using OV7740_debug_YUV422_pad_regs
//to fix the U/V conponents
#undef DEBUG_YUV422

#define DEBUG_PRINTK
#undef DEBUG_PRINTK


/*
 * Our nominal (default) frame rate.
 */
#define OV7740_FRAME_RATE 30	//norminal frame rate, 30~60,
//depends on the input clk & clk reg config
//30fps if inputclk=24M, 0x11=0x01,0x55=def
//60fps if inputclk=24M, 0x11=def,0x55=def
/*
 * The 7740 sits on i2c with ID 0x21
 */
#define OV7740_I2C_ADDR 0x21 //0x42 for write, 0x43 for read

//*******************************************************************************//
//OV7740 register table
//*******************************************************************************//
/* Registers */
#define REG_GAIN	0x00	/* Gain lower 8 bits (rest in vref) */
#define REG_BGAIN	0x01	/* blue gain */
#define REG_RGAIN	0x02	/* red gain */
#define REG_GGAIN	0x03	/* green gain */
#define REG_REG04	0x04	/* analog setting, dont change*/
#define REG_BAVG	0x05	/* b channel average */
#define REG_GAVG	0x06	/* g channel average */
#define REG_RAVG	0x07	/* r channel average */

#define REG_REG0C	0x0C	/* filp enable */
#define REG0C_FLIP_MASK	0x80
#define REG0C_MIRROR_MASK	0x40

#define REG_REG0E	0x0E	/* blc line */
#define REG_HAEC	0x0F	/* auto exposure cntrl */
#define REG_AEC		0x10	/* auto exposure cntrl */

#define REG_CLK		0x11	/* Clock control */
#define REG_REG55	0x55	/* Clock PLL DIV/PreDiv */

#define REG_REG12	0x12

#define REG_REG13	0x13	/* auto/manual AGC, AEC, Write Balance*/
#define REG13_AEC_EN	0x01
#define REG13_AGC_EN	0x04

#define REG_REG14	0x14
#define REG_REG15	0x15
#define REG_REG16	0x16

#define REG_MIDH	0x1C	/* manufacture id byte */
#define REG_MIDL	0x1D	/* manufacture id byre */
#define REG_PIDH	0x0A	/* Product ID MSB */
#define REG_PIDL	0x0B	/* Product ID LSB */

#define REG_84		0x84	/* lots of stuff */
#define REG_REG38	0x38	/* sub-addr */

//sensor output size ctrl
#define REG_AHSTART	0x17	/* Horiz start high bits */
#define REG_AHSIZE	0x18
#define REG_AVSTART	0x19	/* Vert start high bits */
#define REG_AVSIZE	0x1A
#define REG_PSHFT	0x1b	/* Pixel delay after HREF */

//dsp output size ctrl
#define REG_HOUTSIZE	0x31
#define REG_VOUTSIZE	0x32
#define REG_HVSIZEOFF	0x33
#define REG_REG34	0x34	/* DSP output size H/V LSB*/

#define	REG12_RESET	  0x80	/* Register reset */

//contrast
#define	REG_YGAIN	0xE2	//ygain for contrast control

//brightness
#define	REG_YBRIGHT	  0xE3
#define	REG_SGNSET	  0xE4
#define	SGNSET_YBRIGHT_MASK	  0x08

/* saturation */
#define REG_USAT	0xDD
#define REG_VSAT	0xDE

/*
 * Information we maintain about a known sensor.
 */
struct ov7740_format_struct;  /* coming later */
struct ov7740_info {//current config
	struct ov7740_format_struct *fmt;  /* Current format */
	unsigned char sat;		/* Saturation value */
	int hue;			/* Hue value */
};

/*
 * The default register settings, as obtained from OmniVision.
 *
 * These settings give VGA YUYV.
 */
struct regval_list {
	unsigned char reg_num;
	unsigned char value;
};


/*
 * ACCORDING TO OV FAE ANDY, ALL THE FOLLOWING YUV FORMAT CONFIG IS *YUV422*
 * YUYV: Y0U0Y1V1,Y2U2Y3V3,...
 * HOW TO DEBUG YUV FORMAT & SOME IMPORTANT REG:
 * OX11 CLK DIV: SELECT THE OUTPUT PCLK & VSYNC;
 * VSYNC=60FPS WHEN SYSCLK=24M
 *
 * 0X13[0]=1,[3]=0, DEBUG *Y*
 * 0XDA[7],[4],[3], FIX *YUV*
 *
 */

/*
 * start of ov7740 register configuration
 */

// OV7740 YCbCr CIF (352x288)
// 30fps at 24MHz input clock, 16x gain ceiling
// Date: 4/28/2009
static struct regval_list OV7740_CIF_YUYV_30fps[] = {
	//reg addr, val
	{0x13, 0x00},

	{0x55, 0x40},//div
	{0x11, 0x01},//clk

	{0x12, 0x00},

	{0xd5, 0x10},//scale smth ctrl
	{0x0c, 0x12},/*YUV output,YUYV*/
	{0x0d, 0x34},//nc

	{0x17, 0x25},//AHSTART
	{0x18, 0xa0},//AHSIZE
	{0x19, 0x03},//AVSTART
	{0x1a, 0xf0},//AVSIZE
	{0x1b, 0x89},//PSHFT

	{0x22, 0x03},//nc
	{0x29, 0x17},
	{0x2b, 0xf8},
	{0x2c, 0x01},
	{0x31, 0x58},
	{0x32, 0x90},
	{0x33, 0xc4},
	//beginof nc
	{0x35, 0x05},
	{0x36, 0x3f},
	{0x04, 0x60},
	{0x27, 0x80},
	{0x3d, 0x0f},
	{0x3e, 0x82},
	{0x3f, 0x40},
	{0x40, 0x7f},
	{0x41, 0x6a},
	{0x42, 0x29},
	{0x44, 0xe5},
	{0x45, 0x41},
	{0x47, 0x42},
	{0x48, 0x00},
	{0x49, 0x61},
	{0x4a, 0xa1},
	{0x4b, 0x46},
	{0x4c, 0x18},
	{0x4d, 0x50},
	{0x4e, 0x13},
	//endof nc
	{0x64, 0x00},
	{0x67, 0x88},
	{0x68, 0x1a},

	{0x14, 0x38},
	{0x24, 0x3c},
	{0x25, 0x30},
	{0x26, 0x72},
	{0x50, 0x97},
	{0x51, 0x7e},
	{0x52, 0x00},
	{0x53, 0x00},
	{0x20, 0x00},
	{0x21, 0x23},
	{0x38, 0x14},
	{0xe9, 0x00},
	{0x56, 0x55},
	{0x57, 0xff},
	{0x58, 0xff},
	{0x59, 0xff},
	{0x5f, 0x04},
	{0xec, 0x00},
	{0x13, 0xff},

	{0x80, 0x7d},
	{0x81, 0x3f},
	{0x82, 0x3f},
	{0x83, 0x01},
	{0x38, 0x11},
	{0x84, 0x70},
	{0x85, 0x00},
	{0x86, 0x03},
	{0x87, 0x01},
	{0x88, 0x05},
	{0x89, 0x30},
	{0x8d, 0x30},
	{0x8f, 0x85},
	{0x93, 0x30},
	{0x95, 0x85},
	{0x99, 0x30},
	{0x9b, 0x85},

	{0x9c, 0x08},
	{0x9d, 0x12},
	{0x9e, 0x23},
	{0x9f, 0x45},
	{0xa0, 0x55},
	{0xa1, 0x64},
	{0xa2, 0x72},
	{0xa3, 0x7f},
	{0xa4, 0x8b},
	{0xa5, 0x95},
	{0xa6, 0xa7},
	{0xa7, 0xb5},
	{0xa8, 0xcb},
	{0xa9, 0xdd},
	{0xaa, 0xec},
	{0xab, 0x1a},

	{0xce, 0x78},
	{0xcf, 0x6e},
	{0xd0, 0x0a},
	{0xd1, 0x0c},
	{0xd2, 0x84},
	{0xd3, 0x90},
	{0xd4, 0x1e},

	{0x5a, 0x24},
	{0x5b, 0x1f},
	{0x5c, 0x88},
	{0x5d, 0x60},

	{0xac, 0x6e},
	{0xbe, 0xff},
	{0xbf, 0x00},

	//0x50/60Hz auto detection is XCLK dependent
	//0xthe following is based on XCLK = 24MHz
	{0x70, 0x00},
	{0x71, 0x34},
	{0x74, 0x28},
	{0x75, 0x98},
	{0x76, 0x00},
	{0x77, 0x08},
	{0x78, 0x01},
	{0x79, 0xc2},
	{0x7d, 0x02},
	{0x7a, 0x4e},
	{0x7b, 0x1f},
	{0xEC, 0x00}, //00/80 for manual/auto
	{0x7c, 0x0c},

	{0xFF, 0xFF},	/* END MARKER */
};

// OV7740 YCbCr QCIF (176x144)
// 30fps at 24MHz input clock, 16x gain ceiling
// Date: 4/28/2009
static struct regval_list OV7740_QCIF_YCbCr_30fps[] = {
	{0x13, 0x00},

	{0x55, 0x40},//div
	{0x11, 0x01},
	{0x12, 0x00},
	{0xd5, 0x10},
	{0x0c, 0x02},/*YUV output,YUYV*/
	{0x0d, 0x34},
	{0x17, 0x25},
	{0x18, 0xa0},
	{0x19, 0x03},
	{0x1a, 0xf0},
	{0x1b, 0x89},
	{0x22, 0x03},
	{0x29, 0x17},
	{0x2b, 0xf8},
	{0x2c, 0x01},
	{0x31, 0x2c},
	{0x32, 0x48},
	{0x33, 0xc4},
	{0x35, 0x05},
	{0x36, 0x3f},

	{0x04, 0x60},
	{0x27, 0x80},
	{0x3d, 0x0f},
	{0x3e, 0x82},
	{0x3f, 0x40},
	{0x40, 0x7f},
	{0x41, 0x6a},
	{0x42, 0x29},
	{0x44, 0xe5},
	{0x45, 0x41},
	{0x47, 0x42},
	{0x48, 0x00},
	{0x49, 0x61},
	{0x4a, 0xa1},
	{0x4b, 0x46},
	{0x4c, 0x18},
	{0x4d, 0x50},
	{0x4e, 0x13},
	{0x64, 0x00},
	{0x67, 0x88},
	{0x68, 0x1a},

	{0x14, 0x38},
	{0x24, 0x3c},
	{0x25, 0x30},
	{0x26, 0x72},
	{0x50, 0x97},
	{0x51, 0x7e},
	{0x52, 0x00},
	{0x53, 0x00},
	{0x20, 0x00},
	{0x21, 0x23},
	{0x38, 0x14},
	{0xe9, 0x00},
	{0x56, 0x55},
	{0x57, 0xff},
	{0x58, 0xff},
	{0x59, 0xff},
	{0x5f, 0x04},
	{0xec, 0x00},
	{0x13, 0xff},

	{0x80, 0x7d},
	{0x81, 0x3f},
	{0x82, 0x3f},
	{0x83, 0x01},
	{0x38, 0x11},
	{0x84, 0x70},
	{0x85, 0x00},
	{0x86, 0x03},
	{0x87, 0x01},
	{0x88, 0x05},
	{0x89, 0x30},
	{0x8d, 0x30},
	{0x8f, 0x85},
	{0x93, 0x30},
	{0x95, 0x85},
	{0x99, 0x30},
	{0x9b, 0x85},

	{0x9c, 0x08},
	{0x9d, 0x12},
	{0x9e, 0x23},
	{0x9f, 0x45},
	{0xa0, 0x55},
	{0xa1, 0x64},
	{0xa2, 0x72},
	{0xa3, 0x7f},
	{0xa4, 0x8b},
	{0xa5, 0x95},
	{0xa6, 0xa7},
	{0xa7, 0xb5},
	{0xa8, 0xcb},
	{0xa9, 0xdd},
	{0xaa, 0xec},
	{0xab, 0x1a},

	{0xce, 0x78},
	{0xcf, 0x6e},
	{0xd0, 0x0a},
	{0xd1, 0x0c},
	{0xd2, 0x84},
	{0xd3, 0x90},
	{0xd4, 0x1e},

	{0x5a, 0x24},
	{0x5b, 0x1f},
	{0x5c, 0x88},
	{0x5d, 0x60},

	{0xac, 0x6e},
	{0xbe, 0xff},
	{0xbf, 0x00},

	//50/60Hz auto detection is XCLK dependent
	//the following is based on XCLK = 24MHz
	{0x70, 0x00},
	{0x71, 0x34},
	{0x74, 0x28},
	{0x75, 0x98},
	{0x76, 0x00},
	{0x77, 0x08},
	{0x78, 0x01},
	{0x79, 0xc2},
	{0x7d, 0x02},
	{0x7a, 0x4e},
	{0x7b, 0x1f},
	{0xEC, 0x00},//00/80 for manual/auto 50_60Hz
	{0x7c, 0x0c},

	{0xFF, 0xFF},	/* END MARKER */
};

// OV7740 YCbCr QQVGA (160x120)
// 30fps at 24MHz input clock, 16x gain ceiling
// Date: 4/28/2009
static struct regval_list OV7740_QQVGA_YCbCr_30fps[] = {
	{0x13, 0x00},

	{0x55, 0x40},//div
	{0x11, 0x01},
	{0x12, 0x00},
	{0xd5, 0x10},
	{0x0c, 0x02},/*YUV output,YUYV*/
	{0x0d, 0x34},
	{0x17, 0x25},
	{0x18, 0xa0},
	{0x19, 0x03},
	{0x1a, 0xf0},
	{0x1b, 0x89},
	{0x22, 0x03},
	{0x29, 0x17},
	{0x2b, 0xf8},
	{0x2c, 0x01},
	{0x31, 0x28},
	{0x32, 0x3c},
	{0x33, 0xc4},
	{0x35, 0x05},
	{0x36, 0x3f},

	{0x04, 0x60},
	{0x27, 0x80},
	{0x3d, 0x0f},
	{0x3e, 0x82},
	{0x3f, 0x40},
	{0x40, 0x7f},
	{0x41, 0x6a},
	{0x42, 0x29},
	{0x44, 0xe5},
	{0x45, 0x41},
	{0x47, 0x42},
	{0x48, 0x00},
	{0x49, 0x61},
	{0x4a, 0xa1},
	{0x4b, 0x46},
	{0x4c, 0x18},
	{0x4d, 0x50},
	{0x4e, 0x13},
	{0x64, 0x00},
	{0x67, 0x88},
	{0x68, 0x1a},

	{0x14, 0x38},
	{0x24, 0x3c},
	{0x25, 0x30},
	{0x26, 0x72},
	{0x50, 0x97},
	{0x51, 0x7e},
	{0x52, 0x00},
	{0x53, 0x00},
	{0x20, 0x00},
	{0x21, 0x23},
	{0x38, 0x14},
	{0xe9, 0x00},
	{0x56, 0x55},
	{0x57, 0xff},
	{0x58, 0xff},
	{0x59, 0xff},
	{0x5f, 0x04},
	{0xec, 0x00},
	{0x13, 0xff},

	{0x80, 0x7d},
	{0x81, 0x3f},
	{0x82, 0x3f},
	{0x83, 0x01},
	{0x38, 0x11},
	{0x84, 0x70},
	{0x85, 0x00},
	{0x86, 0x03},
	{0x87, 0x01},
	{0x88, 0x05},
	{0x89, 0x30},
	{0x8d, 0x30},
	{0x8f, 0x85},
	{0x93, 0x30},
	{0x95, 0x85},
	{0x99, 0x30},
	{0x9b, 0x85},

	{0x9c, 0x08},
	{0x9d, 0x12},
	{0x9e, 0x23},
	{0x9f, 0x45},
	{0xa0, 0x55},
	{0xa1, 0x64},
	{0xa2, 0x72},
	{0xa3, 0x7f},
	{0xa4, 0x8b},
	{0xa5, 0x95},
	{0xa6, 0xa7},
	{0xa7, 0xb5},
	{0xa8, 0xcb},
	{0xa9, 0xdd},
	{0xaa, 0xec},
	{0xab, 0x1a},

	{0xce, 0x78},
	{0xcf, 0x6e},
	{0xd0, 0x0a},
	{0xd1, 0x0c},
	{0xd2, 0x84},
	{0xd3, 0x90},
	{0xd4, 0x1e},

	{0x5a, 0x24},
	{0x5b, 0x1f},
	{0x5c, 0x88},
	{0x5d, 0x60},

	{0xac, 0x6e},
	{0xbe, 0xff},
	{0xbf, 0x00},

	//50/60Hz auto detection is XCLK dependent
	//the following is based on XCLK = 24MHz
	{0x70, 0x00},
	{0x71, 0x34},
	{0x74, 0x28},
	{0x75, 0x98},
	{0x76, 0x00},
	{0x77, 0x08},
	{0x78, 0x01},
	{0x79, 0xc2},
	{0x7d, 0x02},
	{0x7a, 0x4e},
	{0x7b, 0x1f},
	{0xEC, 0x00},//00/80 for manual/auto 50_60Hz
	{0x7c, 0x0c},

	{0xFF, 0xFF},	/* END MARKER */
};

// OV7740 YCbCr QVGA (320x240)
// 30fps at 24MHz input clock, 16x gain ceiling
// Date: 4/28/2009
static struct regval_list OV7740_QVGA_YCbCr_30fps[] = {
	{0x13, 0x00},

	{0x55, 0x40},//div
	{0x11, 0x01},
	{0x12, 0x00},
	{0xd5, 0x10},
	{0x0c, 0x42},/*YUV output,YUYV, mirror*/
	{0x16, 0x11},/* For mirror mode change SP*/

	{0x0d, 0x34},
	{0x17, 0x25},
	{0x18, 0xa0},
	{0x19, 0x03},
	{0x1a, 0xf0},
	{0x1b, 0x89},
	{0x22, 0x03},
	{0x29, 0x17},
	{0x2b, 0xf8},
	{0x2c, 0x01},
	{0x31, 0x50},
	{0x32, 0x78},
	{0x33, 0xc4},
	{0x35, 0x05},
	{0x36, 0x3f},

	{0x04, 0x60},
	{0x27, 0x80},
	{0x3d, 0x0f},
	{0x3e, 0x82},
	{0x3f, 0x40},
	{0x40, 0x7f},
	{0x41, 0x6a},
	{0x42, 0x29},
	{0x44, 0xe5},
	{0x45, 0x41},
	{0x47, 0x42},
	{0x48, 0x00},
	{0x49, 0x61},
	{0x4a, 0xa1},
	{0x4b, 0x46},
	{0x4c, 0x18},
	{0x4d, 0x50},
	{0x4e, 0x13},
	{0x64, 0x00},
	{0x67, 0x88},
	{0x68, 0x1a},

	{0x14, 0x38},
	{0x24, 0x3c},
	{0x25, 0x30},
	{0x26, 0x72},
	{0x50, 0x97},
	{0x51, 0x7e},
	{0x52, 0x00},
	{0x53, 0x00},
	{0x20, 0x00},
	{0x21, 0x23},
	{0x38, 0x14},
	{0xe9, 0x00},
	{0x56, 0x55},
	{0x57, 0xff},
	{0x58, 0xff},
	{0x59, 0xff},
	{0x5f, 0x04},
	{0xec, 0x00},
	{0x13, 0xff},

	{0x80, 0x7d},
	{0x81, 0x3f},
	{0x82, 0x3f},
	{0x83, 0x01},
	{0x38, 0x11},
	{0x84, 0x70},
	{0x85, 0x00},
	{0x86, 0x03},
	{0x87, 0x01},
	{0x88, 0x05},
	{0x89, 0x30},
	{0x8d, 0x30},
	{0x8f, 0x85},
	{0x93, 0x30},
	{0x95, 0x85},
	{0x99, 0x30},
	{0x9b, 0x85},

	{0x9c, 0x08},
	{0x9d, 0x12},
	{0x9e, 0x23},
	{0x9f, 0x45},
	{0xa0, 0x55},
	{0xa1, 0x64},
	{0xa2, 0x72},
	{0xa3, 0x7f},
	{0xa4, 0x8b},
	{0xa5, 0x95},
	{0xa6, 0xa7},
	{0xa7, 0xb5},
	{0xa8, 0xcb},
	{0xa9, 0xdd},
	{0xaa, 0xec},
	{0xab, 0x1a},

	{0xce, 0x78},
	{0xcf, 0x6e},
	{0xd0, 0x0a},
	{0xd1, 0x0c},
	{0xd2, 0x84},
	{0xd3, 0x90},
	{0xd4, 0x1e},

	{0x5a, 0x24},
	{0x5b, 0x1f},
	{0x5c, 0x88},
	{0x5d, 0x60},

	{0xac, 0x6e},
	{0xbe, 0xff},
	{0xbf, 0x00},

	//50/60Hz auto detection is XCLK dependent
	//the following is based on XCLK = 24MHz
	{0x70, 0x00},
	{0x71, 0x34},
	{0x74, 0x28},
	{0x75, 0x98},
	{0x76, 0x00},
	{0x77, 0x08},
	{0x78, 0x01},
	{0x79, 0xc2},
	{0x7d, 0x02},
	{0x7a, 0x4e},
	{0x7b, 0x1f},
	{0xEC, 0x00},//00/80 for manual/auto
	{0x7c, 0x0c},

	{0xFF, 0xFF},	/* END MARKER */
};

// OV7740 YCbCr VGA (640x480) BT.656
// 30fps at 24MHz input clock, 16x gain ceiling
// Date: 4/28/2009
static struct regval_list OV7740_VGA_BT656_30fps[] = {
	{0x13, 0x00},

	{0x55, 0x40},//div
	{0x11, 0x01},
	{0x12, 0x20},
	{0xd5, 0x10},
	{0x0c, 0x02},/*YUV output,YUYV*/
	{0x0d, 0x34},
	{0x16, 0x10},
	{0x17, 0x25},
	{0x18, 0xa0},
	{0x19, 0x03},
	{0x1a, 0xf0},
	{0x1b, 0x89},
	{0x22, 0x03},
	{0x29, 0x17},
	{0x2b, 0xf8},
	{0x2c, 0x01},
	{0x31, 0xa0},
	{0x32, 0xf0},
	{0x33, 0xc0},
	{0x34, 0x04},
	{0x35, 0x05},
	{0x36, 0x3f},

	{0x04, 0x60},
	{0x27, 0x80},
	{0x3d, 0x0f},
	{0x3e, 0x82},
	{0x3f, 0x40},
	{0x40, 0x7f},
	{0x41, 0x6a},
	{0x42, 0x29},
	{0x44, 0xe5},
	{0x45, 0x41},
	{0x47, 0x42},
	{0x48, 0x00},
	{0x49, 0x61},
	{0x4a, 0xa1},
	{0x4b, 0x46},
	{0x4c, 0x18},
	{0x4d, 0x50},
	{0x4e, 0x13},
	{0x64, 0x00},
	{0x67, 0x88},
	{0x68, 0x1a},

	{0x14, 0x38},
	{0x24, 0x3c},
	{0x25, 0x30},
	{0x26, 0x72},
	{0x50, 0x97},
	{0x51, 0x7e},
	{0x52, 0x00},
	{0x53, 0x00},
	{0x20, 0x00},
	{0x21, 0x23},
	{0x38, 0x14},
	{0xe9, 0x00},
	{0x56, 0x55},
	{0x57, 0xff},
	{0x58, 0xff},
	{0x59, 0xff},
	{0x5f, 0x04},
	{0xec, 0x00},
	{0x13, 0xff},

	{0x80, 0x7d},
	{0x81, 0x3f},
	{0x82, 0x32},
	{0x83, 0x01},
	{0x38, 0x11},
	{0x84, 0x70},
	{0x85, 0x00},
	{0x86, 0x03},
	{0x87, 0x01},
	{0x88, 0x05},
	{0x89, 0x30},
	{0x8d, 0x30},
	{0x8f, 0x85},
	{0x93, 0x30},
	{0x95, 0x85},
	{0x99, 0x30},
	{0x9b, 0x85},

	{0x9c, 0x08},
	{0x9d, 0x12},
	{0x9e, 0x23},
	{0x9f, 0x45},
	{0xa0, 0x55},
	{0xa1, 0x64},
	{0xa2, 0x72},
	{0xa3, 0x7f},
	{0xa4, 0x8b},
	{0xa5, 0x95},
	{0xa6, 0xa7},
	{0xa7, 0xb5},
	{0xa8, 0xcb},
	{0xa9, 0xdd},
	{0xaa, 0xec},
	{0xab, 0x1a},

	{0xce, 0x78},
	{0xcf, 0x6e},
	{0xd0, 0x0a},
	{0xd1, 0x0c},
	{0xd2, 0x84},
	{0xd3, 0x90},
	{0xd4, 0x1e},

	{0x5a, 0x24},
	{0x5b, 0x1f},
	{0x5c, 0x88},
	{0x5d, 0x60},

	{0xac, 0x6e},
	{0xbe, 0xff},
	{0xbf, 0x00},

	//50/60Hz auto detection is XCLK dependent
	//the following is based on XCLK = 24MHz
	{0x70, 0x00},
	{0x71, 0x34},
	{0x74, 0x28},
	{0x75, 0x98},
	{0x76, 0x00},
	{0x77, 0x08},
	{0x78, 0x01},
	{0x79, 0xc2},
	{0x7d, 0x02},
	{0x7a, 0x4e},
	{0x7b, 0x1f},
	{0xEC, 0x00},//00/80 for manual/auto
	{0x7c, 0x0c},

	{0xFF, 0xFF},	/* END MARKER */
};

// OV7740 Raw8 VGA (640x480)
// 30fps at 24MHz input clock, 16x gain ceiling
// Date: 4/28/2009
static struct regval_list OV7740_VGA_Raw8_30fps[] = {
	{0x13, 0x00},

	{0x55, 0x40},//div
	{0x11, 0x01},

	{0x12, 0x01},
	{0xd5, 0x10},
	{0x0c, 0x02},/*YUV output,YUYV*/
	{0x0d, 0x34},
	{0x17, 0x25},
	{0x18, 0xa0},
	{0x19, 0x03},
	{0x1a, 0xf0},
	{0x1b, 0x8a},
	{0x22, 0x03},
	{0x29, 0x17},
	{0x2b, 0xf8},
	{0x2c, 0x01},
	{0x31, 0xa0},
	{0x32, 0xf0},
	{0x33, 0xf4},
	{0x3a, 0xb4},
	{0x36, 0x2f},

	{0x04, 0x60},
	{0x27, 0x80},
	{0x3d, 0x0f},
	{0x3e, 0x82},
	{0x3f, 0x40},
	{0x40, 0x7f},
	{0x41, 0x6a},
	{0x42, 0x29},
	{0x44, 0xe5},
	{0x45, 0x41},
	{0x47, 0x42},
	{0x48, 0x00},
	{0x49, 0x61},
	{0x4a, 0xa1},
	{0x4b, 0x46},
	{0x4c, 0x18},
	{0x4d, 0x50},
	{0x4e, 0x13},
	{0x64, 0x00},
	{0x67, 0x88},
	{0x68, 0x1a},

	{0x14, 0x38},
	{0x24, 0x3c},
	{0x25, 0x30},
	{0x26, 0x72},
	{0x50, 0x97},
	{0x51, 0x7e},
	{0x52, 0x00},
	{0x53, 0x00},
	{0x20, 0x00},
	{0x21, 0x23},
	{0x38, 0x14},
	{0xe9, 0x00},
	{0x56, 0x55},
	{0x57, 0xff},
	{0x58, 0xff},
	{0x59, 0xff},
	{0x5f, 0x04},
	{0xec, 0x00},
	{0x13, 0xff},

	{0x80, 0x7d},
	{0x81, 0x3f},
	{0x82, 0x32},
	{0x83, 0x05},
	{0x38, 0x11},
	{0x84, 0x70},
	{0x85, 0x00},
	{0x86, 0x03},
	{0x87, 0x01},
	{0x88, 0x05},
	{0x89, 0x30},
	{0x8d, 0x30},
	{0x8f, 0x85},
	{0x93, 0x30},
	{0x95, 0x85},
	{0x99, 0x30},
	{0x9b, 0x85},

	{0x9c, 0x08},
	{0x9d, 0x12},
	{0x9e, 0x23},
	{0x9f, 0x45},
	{0xa0, 0x55},
	{0xa1, 0x64},
	{0xa2, 0x72},
	{0xa3, 0x7f},
	{0xa4, 0x8b},
	{0xa5, 0x95},
	{0xa6, 0xa7},
	{0xa7, 0xb5},
	{0xa8, 0xcb},
	{0xa9, 0xdd},
	{0xaa, 0xec},
	{0xab, 0x1a},

	{0xce, 0x78},
	{0xcf, 0x6e},
	{0xd0, 0x0a},
	{0xd1, 0x0c},
	{0xd2, 0x84},
	{0xd3, 0x90},
	{0xd4, 0x1e},

	{0x5a, 0x24},
	{0x5b, 0x1f},
	{0x5c, 0x88},
	{0x5d, 0x60},

	{0xac, 0x6e},
	{0xbe, 0xff},
	{0xbf, 0x00},

	//50/60Hz auto detection is XCLK dependant
	//the following is based on XCLK = 24MHz
	{0x70, 0x00},
	{0x71, 0x34},
	{0x74, 0x28},
	{0x75, 0x98},
	{0x76, 0x00},
	{0x77, 0x08},
	{0x78, 0x01},
	{0x79, 0xc2},
	{0x7d, 0x02},
	{0x7a, 0x4e},
	{0x7b, 0x1f},
	{0xEC, 0x00},//00/80 for manual/auto

	{0x7c, 0x0c},


	{0xFF, 0xFF},	/* END MARKER */
};

// OV7740 Raw10 VGA (640x480)
// 30fps at 24MHz input clock, 16x gain ceiling
// Date: 4/28/2009
static struct regval_list OV7740_VGA_Raw10_30fps[] = {
	{0x13, 0x00},

	{0x55, 0x40},//div
	{0x11, 0x01},
	{0x12, 0x01},
	{0xd5, 0x10},
	{0x0c, 0x02},/*YUV output,YUYV*/
	{0x0d, 0x34},
	{0x17, 0x25},
	{0x18, 0xa0},
	{0x19, 0x03},
	{0x1a, 0xf0},
	{0x1b, 0x8a},
	{0x22, 0x03},
	{0x29, 0x17},
	{0x2b, 0xf8},
	{0x2c, 0x01},
	{0x31, 0xa0},
	{0x32, 0xf0},
	{0x33, 0xf4},
	{0x3a, 0xb4},
	{0x36, 0x2f},

	{0x04, 0x60},
	{0x27, 0x80},
	{0x3d, 0x0f},
	{0x3e, 0x82},
	{0x3f, 0x40},
	{0x40, 0x7f},
	{0x41, 0x6a},
	{0x42, 0x29},
	{0x44, 0xe5},
	{0x45, 0x41},
	{0x47, 0x42},
	{0x48, 0x00},
	{0x49, 0x61},
	{0x4a, 0xa1},
	{0x4b, 0x46},
	{0x4c, 0x18},
	{0x4d, 0x50},
	{0x4e, 0x13},
	{0x64, 0x00},
	{0x67, 0x88},
	{0x68, 0x1a},

	{0x14, 0x38},
	{0x24, 0x3c},
	{0x25, 0x30},
	{0x26, 0x72},
	{0x50, 0x97},
	{0x51, 0x7e},
	{0x52, 0x00},
	{0x53, 0x00},
	{0x20, 0x00},
	{0x21, 0x23},
	{0x38, 0x14},
	{0xe9, 0x00},
	{0x56, 0x55},
	{0x57, 0xff},
	{0x58, 0xff},
	{0x59, 0xff},
	{0x5f, 0x04},
	{0xec, 0x00},
	{0x13, 0xff},

	{0x80, 0x7d},
	{0x81, 0x3f},
	{0x82, 0x32},
	{0x83, 0x01},
	{0x38, 0x11},
	{0x84, 0x70},
	{0x85, 0x00},
	{0x86, 0x03},
	{0x87, 0x01},
	{0x88, 0x05},
	{0x89, 0x30},
	{0x8d, 0x30},
	{0x8f, 0x85},
	{0x93, 0x30},
	{0x95, 0x85},
	{0x99, 0x30},
	{0x9b, 0x85},

	{0x9c, 0x08},
	{0x9d, 0x12},
	{0x9e, 0x23},
	{0x9f, 0x45},
	{0xa0, 0x55},
	{0xa1, 0x64},
	{0xa2, 0x72},
	{0xa3, 0x7f},
	{0xa4, 0x8b},
	{0xa5, 0x95},
	{0xa6, 0xa7},
	{0xa7, 0xb5},
	{0xa8, 0xcb},
	{0xa9, 0xdd},
	{0xaa, 0xec},
	{0xab, 0x1a},

	{0xce, 0x78},
	{0xcf, 0x6e},
	{0xd0, 0x0a},
	{0xd1, 0x0c},
	{0xd2, 0x84},
	{0xd3, 0x90},
	{0xd4, 0x1e},

	{0x5a, 0x24},
	{0x5b, 0x1f},
	{0x5c, 0x88},
	{0x5d, 0x60},

	{0xac, 0x6e},
	{0xbe, 0xff},
	{0xbf, 0x00},

	//50/60Hz auto detection is XCLK dependant
	//the following is based on XCLK = 24MHz
	{0x70, 0x00},
	{0x71, 0x34},
	{0x74, 0x28},
	{0x75, 0x98},
	{0x76, 0x00},
	{0x77, 0x08},
	{0x78, 0x01},
	{0x79, 0xc2},
	{0x7d, 0x02},
	{0x7a, 0x4e},
	{0x7b, 0x1f},
	{0xEC, 0x00},//00/80 for manual/auto
	{0x7c, 0x0c},


	{0xFF, 0xFF},	/* END MARKER */
};

// OV7740 YCbCr VGA (640x480)
// 30fps at 24MHz input clock, 16x gain ceiling
// Date: 4/28/2009
static struct regval_list OV7740_VGA_YCbCr_30fps[] = {
	{0x13, 0x00},

	{0x55, 0x40},//div
	{0x11, 0x01},
	{0x12, 0x00},
	{0xd5, 0x10},
	{0x0c, 0x42},/*YUV output,YUYV, mirror*/
	{0x16, 0x11},/* For mirror mode change SP*/

	{0x0d, 0x34},
	{0x17, 0x25},
	{0x18, 0xa0},
	{0x19, 0x03},
	{0x1a, 0xf0},
	{0x1b, 0x89},
	{0x22, 0x03},
	{0x29, 0x17},
	{0x2b, 0xf8},
	{0x2c, 0x01},
	{0x31, 0xa0},
	{0x32, 0xf0},
	{0x33, 0xc4},
	{0x35, 0x05},
	{0x36, 0x3f},

	{0x04, 0x60},
	{0x27, 0x80},
	{0x3d, 0x0f},
	{0x3e, 0x82},
	{0x3f, 0x40},
	{0x40, 0x7f},
	{0x41, 0x6a},
	{0x42, 0x29},
	{0x44, 0xe5},
	{0x45, 0x41},
	{0x47, 0x42},
	{0x48, 0x00},
	{0x49, 0x61},
	{0x4a, 0xa1},
	{0x4b, 0x46},
	{0x4c, 0x18},
	{0x4d, 0x50},
	{0x4e, 0x13},
	{0x64, 0x00},
	{0x67, 0x88},
	{0x68, 0x1a},

	{0x14, 0x38},
	{0x24, 0x3c},
	{0x25, 0x30},
	{0x26, 0x72},
	{0x50, 0x97},
	{0x51, 0x7e},
	{0x52, 0x00},
	{0x53, 0x00},
	{0x20, 0x00},
	{0x21, 0x23},
	{0x38, 0x14},
	{0xe9, 0x00},
	{0x56, 0x55},
	{0x57, 0xff},
	{0x58, 0xff},
	{0x59, 0xff},
	{0x5f, 0x04},
	{0xec, 0x00},
	{0x13, 0xff},

	{0x80, 0x7d},
	{0x81, 0x3f},
	{0x82, 0x32},
	{0x83, 0x01},
	{0x38, 0x11},
	{0x84, 0x70},
	{0x85, 0x00},
	{0x86, 0x03},
	{0x87, 0x01},
	{0x88, 0x05},
	{0x89, 0x30},
	{0x8d, 0x30},
	{0x8f, 0x85},
	{0x93, 0x30},
	{0x95, 0x85},
	{0x99, 0x30},
	{0x9b, 0x85},

	{0x9c, 0x08},
	{0x9d, 0x12},
	{0x9e, 0x23},
	{0x9f, 0x45},
	{0xa0, 0x55},
	{0xa1, 0x64},
	{0xa2, 0x72},
	{0xa3, 0x7f},
	{0xa4, 0x8b},
	{0xa5, 0x95},
	{0xa6, 0xa7},
	{0xa7, 0xb5},
	{0xa8, 0xcb},
	{0xa9, 0xdd},
	{0xaa, 0xec},
	{0xab, 0x1a},

	{0xce, 0x78},
	{0xcf, 0x6e},
	{0xd0, 0x0a},
	{0xd1, 0x0c},
	{0xd2, 0x84},
	{0xd3, 0x90},
	{0xd4, 0x1e},

	{0x5a, 0x24},
	{0x5b, 0x1f},
	{0x5c, 0x88},
	{0x5d, 0x60},

	{0xac, 0x6e},
	{0xbe, 0xff},
	{0xbf, 0x00},

	//50/60Hz auto detection is XCLK dependent
	//the following is based on XCLK = 24MHz
	{0x70, 0x00},
	{0x71, 0x34},
	{0x74, 0x28},
	{0x75, 0x98},
	{0x76, 0x00},
	{0x77, 0x08},
	{0x78, 0x01},
	{0x79, 0xc2},
	{0x7d, 0x02},
	{0x7a, 0x4e},
	{0x7b, 0x1f},
	{0xEC, 0x00},//00/80 for manual/auto
	{0x7c, 0x0c},

	{0xFF, 0xFF},	/* END MARKER */
};

// OV7740 YCbCr VGA (640x480)
// 60fps at 24MHz input clock, 16x gain ceiling
// Date: 4/28/2009
static struct regval_list OV7740_VGA_YCbCr_60fps[] = {
	{0x13, 0x00},

	{0x55, 0x40},//div
	{0x11, 0x00},
	{0x12, 0x00},
	{0xd5, 0x10},
	{0x0c, 0x12},/*YUV output,YUYV*/
	{0x0d, 0x34},
	{0x17, 0x25},
	{0x18, 0xa0},
	{0x19, 0x03},
	{0x1a, 0xf0},
	{0x1b, 0x89},
	{0x22, 0x03},
	{0x29, 0x17},
	{0x2b, 0xf8},
	{0x2c, 0x01},
	{0x31, 0xa0},
	{0x32, 0xf0},
	{0x33, 0xc4},
	{0x35, 0x05},
	{0x36, 0x3f},

	{0x04, 0x60},
	{0x27, 0x80},
	{0x3d, 0x08},
	{0x3e, 0x82},
	{0x3f, 0x40},
	{0x40, 0x7f},
	{0x41, 0x6a},
	{0x42, 0x29},
	{0x44, 0xf5},
	{0x45, 0x41},
	{0x47, 0x42},
	{0x48, 0x00},
	{0x49, 0x61},
	{0x4a, 0xa1},
	{0x4b, 0x46},
	{0x4c, 0x18},
	{0x4d, 0x50},
	{0x4e, 0x13},
	{0x64, 0x00},
	{0x67, 0x88},
	{0x68, 0x1a},

	{0x14, 0x38},
	{0x24, 0x3c},
	{0x25, 0x30},
	{0x26, 0x72},
	{0x50, 0x2e},
	{0x51, 0xfc},
	{0x52, 0x10},
	{0x53, 0x00},
	{0x20, 0x00},
	{0x21, 0x01},
	{0x38, 0x14},
	{0xe9, 0x00},
	{0x56, 0x55},
	{0x57, 0xff},
	{0x58, 0xff},
	{0x59, 0xff},
	{0x5f, 0x04},
	{0xec, 0x00},
	{0x13, 0xff},

	{0x80, 0x7d},
	{0x81, 0x3f},
	{0x82, 0x32},
	{0x83, 0x01},
	{0x38, 0x11},
	{0x84, 0x70},
	{0x85, 0x00},
	{0x86, 0x03},
	{0x87, 0x01},
	{0x88, 0x05},
	{0x89, 0x30},
	{0x8d, 0x30},
	{0x8f, 0x85},
	{0x93, 0x30},
	{0x95, 0x85},
	{0x99, 0x30},
	{0x9b, 0x85},

	{0x9c, 0x08},
	{0x9d, 0x12},
	{0x9e, 0x23},
	{0x9f, 0x45},
	{0xa0, 0x55},
	{0xa1, 0x64},
	{0xa2, 0x72},
	{0xa3, 0x7f},
	{0xa4, 0x8b},
	{0xa5, 0x95},
	{0xa6, 0xa7},
	{0xa7, 0xb5},
	{0xa8, 0xcb},
	{0xa9, 0xdd},
	{0xaa, 0xec},
	{0xab, 0x1a},

	{0xce, 0x78},
	{0xcf, 0x6e},
	{0xd0, 0x0a},
	{0xd1, 0x0c},
	{0xd2, 0x84},
	{0xd3, 0x90},
	{0xd4, 0x1e},

	{0x5a, 0x24},
	{0x5b, 0x1f},
	{0x5c, 0x88},
	{0x5d, 0x60},

	{0xac, 0x6e},
	{0xbe, 0xff},
	{0xbf, 0x00},

	//50/60Hz auto detection is XCLK dependent
	//the following is based on XCLK = 24MHz
	{0x70, 0x00},
	{0x71, 0x34},
	{0x74, 0x28},
	{0x75, 0x98},
	{0x76, 0x00},
	{0x77, 0x08},
	{0x78, 0x01},
	{0x79, 0xc2},
	{0x7d, 0x02},
	{0x7a, 0x4e},
	{0x7b, 0x1f},
	{0xEC, 0x00},//00/80 for manual/auto 50_60Hz
	{0x7c, 0x0c},

	{0xFF, 0xFF},	/* END MARKER */
};

static struct regval_list OV7740_CIF_YCbCr_30fps[] = {
	//reg addr, val
	{0x13, 0x00},

	{0x55, 0x40},//div
	{0x11, 0x01},//clk

	{0x12, 0x00},

	{0xd5, 0x10},//scale smth ctrl
	{0x0c, 0x02},/*YUV output,YUYV*/
	{0x0d, 0x34},//nc

	{0x17, 0x25},//AHSTART
	{0x18, 0xa0},//AHSIZE
	{0x19, 0x03},//AVSTART
	{0x1a, 0xf0},//AVSIZE
	{0x1b, 0x89},//PSHFT

	{0x22, 0x03},//nc
	{0x29, 0x17},
	{0x2b, 0xf8},
	{0x2c, 0x01},
	{0x31, 0x58},
	{0x32, 0x90},
	{0x33, 0xc4},
	//beginof nc
	{0x35, 0x05},
	{0x36, 0x3f},
	{0x04, 0x60},
	{0x27, 0x80},
	{0x3d, 0x0f},
	{0x3e, 0x82},
	{0x3f, 0x40},
	{0x40, 0x7f},
	{0x41, 0x6a},
	{0x42, 0x29},
	{0x44, 0xe5},
	{0x45, 0x41},
	{0x47, 0x42},
	{0x48, 0x00},
	{0x49, 0x61},
	{0x4a, 0xa1},
	{0x4b, 0x46},
	{0x4c, 0x18},
	{0x4d, 0x50},
	{0x4e, 0x13},
	//endof nc
	{0x64, 0x00},
	{0x67, 0x88},
	{0x68, 0x1a},

	{0x14, 0x38},
	{0x24, 0x3c},
	{0x25, 0x30},
	{0x26, 0x72},
	{0x50, 0x97},
	{0x51, 0x7e},
	{0x52, 0x00},
	{0x53, 0x00},
	{0x20, 0x00},
	{0x21, 0x23},
	{0x38, 0x14},
	{0xe9, 0x00},
	{0x56, 0x55},
	{0x57, 0xff},
	{0x58, 0xff},
	{0x59, 0xff},
	{0x5f, 0x04},
	{0xec, 0x00},
	{0x13, 0xff},

	{0x80, 0x7d},
	{0x81, 0x3f},
	{0x82, 0x3f},
	{0x83, 0x01},
	{0x38, 0x11},
	{0x84, 0x70},
	{0x85, 0x00},
	{0x86, 0x03},
	{0x87, 0x01},
	{0x88, 0x05},
	{0x89, 0x30},
	{0x8d, 0x30},
	{0x8f, 0x85},
	{0x93, 0x30},
	{0x95, 0x85},
	{0x99, 0x30},
	{0x9b, 0x85},
	{0x9c, 0x08},
	{0x9d, 0x12},
	{0x9e, 0x23},
	{0x9f, 0x45},
	{0xa0, 0x55},
	{0xa1, 0x64},
	{0xa2, 0x72},
	{0xa3, 0x7f},
	{0xa4, 0x8b},
	{0xa5, 0x95},
	{0xa6, 0xa7},
	{0xa7, 0xb5},
	{0xa8, 0xcb},
	{0xa9, 0xdd},
	{0xaa, 0xec},
	{0xab, 0x1a},

	{0xce, 0x78},
	{0xcf, 0x6e},
	{0xd0, 0x0a},
	{0xd1, 0x0c},
	{0xd2, 0x84},
	{0xd3, 0x90},
	{0xd4, 0x1e},

	{0x5a, 0x24},
	{0x5b, 0x1f},
	{0x5c, 0x88},
	{0x5d, 0x60},

	{0xac, 0x6e},
	{0xbe, 0xff},
	{0xbf, 0x00},

	//0x50/60Hz auto detection is XCLK dependent
	//0xthe following is based on XCLK = 24MHz
	{0x70, 0x00},
	{0x71, 0x34},
	{0x74, 0x28},
	{0x75, 0x98},
	{0x76, 0x00},
	{0x77, 0x08},
	{0x78, 0x01},
	{0x79, 0xc2},
	{0x7d, 0x02},
	{0x7a, 0x4e},
	{0x7b, 0x1f},
	{0xEC, 0x00}, //00/80 for manual/auto
	{0x7c, 0x0c},

	{0xFF, 0xFF},	/* END MARKER */
};

/*
 * end of ov7740 register configuration::position_tag
 */


//default ov config, rgb 30fps
static struct regval_list OV7740_default_regs[] = {
	{ REG_REG12, REG12_RESET },

	{ REG_REG55, 0x40 },	/* Clock PLL DIV/PreDiv */
	{ REG_CLK, 0x01 },	//clk, 24MHz

	{ 0xff, 0xff },	/* END MARKER */
};

static struct regval_list OV7740_debug_regs[] = {
	{ REG_REG12, REG12_RESET },
	{ REG_REG13, 0x87 },	//auto AGC, AEC

	{ REG_REG55, 0x40 },	/* Clock PLL DIV/PreDiv */
	{ REG_CLK, 0x01 },	//clk, 24MHz

	//{ REG_REG12, 0x00 },	/* VGA_YUV */
	//{ REG_REG12, 0x01 },	/* VGA_RGBraw */

	//debug mode
	{ REG_REG38,  0x17 },	//debug color bar
	{ REG_84,  0x02 },	//0x02/0x00 ouput data are
	//colorbar/normal data
	{ 0xff, 0xff },	/* END MARKER */
};

//behind the other config reglist to generate corresponding color bar for debug
static struct regval_list OV7740_colorbar_pad_regs[] = {
	//donot reset
	{ REG_REG38,  0x17 },	//debug color bar
	{ REG_84,  0x02 },	//0x02/0x00 ouput data are
	//colorbar/normal data
	{ 0xff, 0xff },	/* END MARKER */
};

#ifdef DEBUG_YUV422
//behind the other config reglist to generate corresponding color bar for debug
static struct regval_list OV7740_debug_YUV422_pad_regs[] = {
	//donot reset

	{0x13,0x80},//disable AEC
	{0x0F,0xFF},
	{0x10,0xFF},//max exposure

	{0xE0,0xAB},//V
	{0xDF,0xCD},//U
	//{0xE0,0xFF},//V
	//{0xDF,0xFF},//U
	//{0x0c, 0x12},/*YUV output,YUYV*/

	{0xDA,0x18},//enable UV debug

	{ 0xff, 0xff },	/* END MARKER */
};
#endif

/* ov7740 reg setting for optimizing ov7740 on IPCAM */
static struct regval_list OV7740_ipcam_opti_regs[] = {
	{0x80, 0x7f},
	{0x8c, 0x01},
	{0x8a, 0x4f},
	{0x8b, 0xf7},
	{0x8d, 0x3a},
	{0x8f, 0xf3},
	{0x8e, 0xcb},

	{0x92, 0x01},
	{0x90, 0x52},
	{0x91, 0xf5},
	{0x93, 0x34},
	{0x95, 0x73},
	{0x94, 0x79},

	{0x98, 0x01},
	{0x96, 0x4f},
	{0x97, 0xf5},
	{0x99, 0x31},
	{0x9b, 0x73},
	{0x9a, 0x1e},

	/* PLL divider to find out around 22fps
	 * from 19.5 MHz mclk input */
	{0x14, 0x3a},
	{0x2b, 0x28},
	{0x2c, 0x02},

	{0xff, 0xff},	/* END MARKER */
};

#if 0
static int ov7740_read(struct i2c_client *c, u8 reg,u8 *value)
{
	i2c_master_send(c,&reg,1);//i2c_smbus_write_byte(g_client,reg_addr);
	return i2c_master_recv(c,value,1);
}

static int ov7740_write(struct i2c_client *c, u8 reg,u8 value)
{
	return i2c_smbus_write_byte_data(c,reg,value);
}
#endif

/*
 * Low-level register I/O.
 */
static int ov7740_read(struct i2c_client *c, unsigned char reg,
		unsigned char *value)
{
#ifdef DEBUG_PRINTK
	printk("##!!## %s called\n",__FUNCTION__);
#endif
	i2c_smbus_write_byte(c, reg);
	*value = i2c_smbus_read_byte(c);

	return 0;
}

static int ov7740_write(struct i2c_client *c, unsigned char reg,
		unsigned char value)
{
#ifdef DEBUG_PRINTK
	printk("##!!## %s called,reg=0x%x,val=0x%x\n",__FUNCTION__,reg,value);
#endif
	int ret = i2c_smbus_write_byte_data(c, reg, value);
	if (reg == REG_REG12 && (value & REG12_RESET))
		msleep(2);  /* Wait for reset to run */
#ifdef DEBUG_PRINTK
	unsigned char v=0;
	ov7740_read(c, reg, &v);
	printk("::0x%x,0x%x", reg, v);
#endif
	return ret;
}

/*
 * Write a list of register settings; ff/ff stops the process.
 */
static int ov7740_write_array(struct i2c_client *c, struct regval_list *vals)
{
#ifdef DEBUG_PRINTK
	printk("##!!## %s called\n",__FUNCTION__);
#endif
	int i = 0;
	while (vals->reg_num != 0xff || vals->value != 0xff) {
		int ret = ov7740_write(c, vals->reg_num, vals->value);
		if (ret < 0)
			return ret;
		vals++;
		//if (i == 0)
		if (vals->reg_num ==  REG_REG12)
			mdelay(5);//delay if reset
		i++;
	}
	return 0;
}

/*
 * Stuff that knows about the sensor.
 */
static int ov7740_reset(struct i2c_client *client)
{
#ifdef DEBUG_PRINTK
	printk("##!!## %s called\n",__FUNCTION__);
#endif
	int ret;
	ret = ov7740_write(client, REG_REG12, REG12_RESET);
	msleep(1);
	return ret;
}

static int ov7740_init(struct i2c_client *client)
{
	int ret;
	ret = ov7740_write_array(client, OV7740_CIF_YCbCr_30fps);
	//ov7740_write_array(client, OV7740_debug_regs);
	//ov7740_write_array(client, OV7740_VGA_YCbCr_60fps);

	/* ret += ov7740_write_array(client, OV7740_colorbar_pad_regs);//color bar */
	return ret;
}

static int ov7740_detect(struct i2c_client *client)
{
#ifdef DEBUG_PRINTK
	printk("##!!## %s called\n",__FUNCTION__);
#endif
	unsigned char v = 0;
	int ret;

	ret = ov7740_read(client, REG_MIDH, &v);
	if (ret < 0)
		return ret;
	if (v != 0x7f) /* OV manuf. id. */
		return -ENODEV;
	ret = ov7740_read(client, REG_MIDL, &v);
	if (ret < 0)
		return ret;
	if (v != 0xa2)
		return -ENODEV;
	/*
	 * OK, we know we have an OmniVision chip...but which one?
	 */
	ret = ov7740_read(client, REG_PIDH, &v);
	if (ret < 0)
		return ret;
	if (v != 0x77)
		return -ENODEV;
	ret = ov7740_read(client, REG_PIDL, &v);
	if (ret < 0)
		return ret;
	/* could be 0x42, 0x41 or 0x40 */
	if ((v != 0x42) && (v != 0x40) && (v != 0x41))
		return -ENODEV;
	return 0;
}

/*
 * Basic window sizes.  These probably belong somewhere more globally
 * useful.
 */
#define VGA_WIDTH	640
#define VGA_HEIGHT	480
#define CIF_WIDTH	352
#define CIF_HEIGHT	288
#define QVGA_WIDTH	320
#define QVGA_HEIGHT	240
#define QCIF_WIDTH	176
#define	QCIF_HEIGHT	144
#define QQVGA_WIDTH	160
#define	QQVGA_HEIGHT	120

//*******************************************************************************//
//*******************************************************************************//
/*
 * Store information about the video data format.  The color matrix
 * is deeply tied into the format, so keep the relevant values here.
 * The magic matrix nubmers come from OmniVision.
 *
 * *The SIZE should sort with LARGE->SMALL order*
 */
static struct ov7740_format_struct {
	__u8	*desc;
	__u32	pixelformat;
	int	width;
	int	height;
	int	fps;
	struct	regval_list *regs;
	int	bpp;   /* bits per pixel */
} ov7740_formats[] = {
	{
		.desc		= "OV7740 VGA YUV422 30fps",
		.pixelformat	= V4L2_PIX_FMT_YUV422P,
		.width		= VGA_WIDTH,
		.height		= VGA_HEIGHT,
		.fps		= 30,
		.regs 		= OV7740_VGA_YCbCr_30fps,
		.bpp		= 16,
	},
	{//high precision
		.desc		= "OV7740 VGA RAW10 30fps",
		.pixelformat	= V4L2_PIX_FMT_YUV422P,
		.width		= VGA_WIDTH,
		.height		= VGA_HEIGHT,
		.fps		= 30,
		.regs 		= OV7740_VGA_Raw10_30fps,
		.bpp		= 10,
	},
	{
		.desc		= "OV7740 CIF YUV422 30fps",
		.pixelformat	= V4L2_PIX_FMT_YUV422P,
		.width		= CIF_WIDTH,
		.height		= CIF_HEIGHT,
		.fps		= 30,
		.regs 		= OV7740_CIF_YCbCr_30fps,
		.bpp		= 16,
	},
	{
		.desc		= "OV7740 QVGA YUV422 30fps",
		.pixelformat	= V4L2_PIX_FMT_YUV422P,
		.width		= QVGA_WIDTH,
		.height		= QVGA_HEIGHT,
		.fps		= 30,
		.regs		= OV7740_QVGA_YCbCr_30fps,
		.bpp		= 16,
	},
	{
		.desc		= "OV7740 QCIF YUV422 30fps",
		.pixelformat	= V4L2_PIX_FMT_YUV422P,
		.width		= QCIF_WIDTH,
		.height		= QCIF_HEIGHT,
		.fps		= 30,
		.regs		= OV7740_QCIF_YCbCr_30fps,
		.bpp		= 16,
	},
	{
		.desc		= "OV7740 QQVGA YUV422 30fps",
		.pixelformat	= V4L2_PIX_FMT_YUV422P,
		.width		= QQVGA_WIDTH,
		.height		= QQVGA_HEIGHT,
		.fps		= 30,
		.regs		= OV7740_QQVGA_YCbCr_30fps,
		.bpp		= 16,
	},
	{
		.desc		=	"OV7740 CIF YUV422pack 30fps",
		.pixelformat	=	V4L2_PIX_FMT_YUYV,
		.width		=	CIF_WIDTH,
		.height		=	CIF_HEIGHT,
		.fps		=	30,
		.regs		=	OV7740_CIF_YUYV_30fps,
		.bpp		=	16,
	},
	{
		.desc           = "OV7740 VGA YUV420 30fps",
		.pixelformat    = V4L2_PIX_FMT_YUV420,
		.width		= VGA_WIDTH,
		.height		= VGA_HEIGHT,
		.fps		= 30,
		.regs		= OV7740_VGA_YCbCr_30fps,
		.bpp            = 12,
	},
	{
		.desc		= "OV7740 CIF YUV420 30fps",
		.pixelformat	= V4L2_PIX_FMT_YUV420,
		.width		= CIF_WIDTH,
		.height		= CIF_HEIGHT,
		.fps		= 30,
		.regs 		= OV7740_CIF_YCbCr_30fps,
		.bpp		= 12,
	},
	{
		.desc		= "OV7740 QVGA YUV420 30fps",
		.pixelformat	= V4L2_PIX_FMT_YUV420,
		.width		= QVGA_WIDTH,
		.height		= QVGA_HEIGHT,
		.fps		= 30,
		.regs 		= OV7740_QVGA_YCbCr_30fps,
		.bpp		= 12,
	},
	{
		.desc		= "OV7740 QCIF YUV420 30fps",
		.pixelformat	= V4L2_PIX_FMT_YUV420,
		.width		= QCIF_WIDTH,
		.height		= QCIF_HEIGHT,
		.fps		= 30,
		.regs 		= OV7740_QCIF_YCbCr_30fps,
		.bpp		= 12,
	},
	{
		.desc		= "OV7740 QQVGA YUV420 30fps",
		.pixelformat	= V4L2_PIX_FMT_YUV420,
		.width		= QQVGA_WIDTH,
		.height		= QQVGA_HEIGHT,
		.fps		= 30,
		.regs 		= OV7740_QQVGA_YCbCr_30fps,
		.bpp		= 12,
	},
	{
		.desc		= "OV7740 VGA RAW8 30fps",
		.pixelformat	= V4L2_PIX_FMT_RGB24,//FIXME
		.width		= VGA_WIDTH,
		.height		= VGA_HEIGHT,
		.fps		= 30,
		.regs		= OV7740_VGA_Raw8_30fps,
		.bpp		= 8,
	},
	{
		.desc		= "OV7740 VGA BT656 30fps",
		.pixelformat	= NULL,//FIXME:not support by current CCIC
		.width		= VGA_WIDTH,
		.height		= VGA_HEIGHT,
		.fps		= 30,
		.regs		= OV7740_VGA_BT656_30fps,
		//.bpp		= 16,//TODO
	},
};
#define N_OV7740_FMTS ARRAY_SIZE(ov7740_formats)	//total number of ov7740 format

#define V4L2_PIX_FMT_RGB24   v4l2_fourcc('R', 'G', 'B', '3') /* 24  RGB-8-8-8     */

static int ov7740_cropcap(struct i2c_client *c, struct v4l2_cropcap *ccap)
{
	struct ov7740_format_struct *ovfmt = ov7740_formats;

	ccap->bounds.left = 0;
	ccap->bounds.top = 0;
	ccap->bounds.width = ovfmt->width;
	ccap->bounds.height = ovfmt->height;

	ccap->defrect = ccap->bounds;

	ccap->pixelaspect.numerator = 1;
	ccap->pixelaspect.denominator = 1;
}

//select in ov cam foramt by index NUM
static int ov7740_enum_fmt(struct i2c_client *c, struct v4l2_fmtdesc *fmt)
{
	struct ov7740_format_struct *ofmt;

	if (fmt->index >= N_OV7740_FMTS)
		return -EINVAL;

	ofmt = ov7740_formats + fmt->index;
	fmt->flags = 0;
	strcpy(fmt->description, ofmt->desc);
	fmt->pixelformat = ofmt->pixelformat;
	return 0;
}

static int ov7740_g_fmt(struct i2c_client *client, struct v4l2_pix_format *pix)
{
	struct ov7740_format_struct *ofmt;
	int i;

	for (i = 0; i < N_OV7740_FMTS; i++){
		if (ov7740_formats[i].pixelformat == pix->pixelformat){
			if (pix->width >= ov7740_formats[i].width &&
					pix->height >= ov7740_formats[i].height){
				break;
			}
		}
	}

	if (i >= N_OV7740_FMTS)
		return -EINVAL;

	ofmt = ov7740_formats + i;

	pix->bytesperline = ofmt->width * ofmt->bpp / 8;
	pix->sizeimage = ofmt->height * pix->bytesperline;

	return 0;
}

static int ov7740_enum_framesizes(struct i2c_client *client, struct v4l2_frmsizeenum *fsize)
{
	struct ov7740_format_struct *ofmt = NULL;
	int i, index = -1;

	if (fsize->index >= N_OV7740_FMTS)
		return -EINVAL;

	for (i = 0; i < N_OV7740_FMTS; i++) {
		if (ov7740_formats[i].pixelformat ==
				fsize->pixel_format) {
			index++;
			if (index == fsize->index)
				break;
		}
	}

	if (index < fsize->index)
		return -EINVAL;

	if (i >= N_OV7740_FMTS)
		return -EINVAL;

	ofmt = ov7740_formats + i;
	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	/* fsize->pixel_format = ofmt->pixelformat; */
	fsize->discrete.width = ofmt->width;
	fsize->discrete.height = ofmt->height;

	return 0;
}

// select image FORMAT and PIXEL SIZE
static int ov7740_try_fmt(struct i2c_client *c, struct v4l2_format *fmt,
		struct ov7740_format_struct **ret_fmt)
{
	int index;
	struct v4l2_pix_format *pix = &fmt->fmt.pix;

	//find the required pixelformat in ov7740_formats[]
	//
	//Round requested image size down to the nearest
	//we support, but not below the smallest.
	for (index = 0; index < N_OV7740_FMTS; index++){
		if (ov7740_formats[index].pixelformat == pix->pixelformat){
			if (pix->width >= ov7740_formats[index].width &&
					pix->height >= ov7740_formats[index].height){
				break;
			}
		}
	}

	if (index >= N_OV7740_FMTS)
		return -EINVAL;
	//set the returnpixel format of ov7740 which satisfy v4l2
	if (ret_fmt != NULL)
		*ret_fmt = ov7740_formats + index;
	/*
	 * Fields: the OV devices claim to be progressive.
	 */
	/*
	if (pix->field == V4L2_FIELD_ANY)
		pix->field = V4L2_FIELD_NONE;
	else if (pix->field != V4L2_FIELD_NONE)
		return -EINVAL;
	*/

	/*
	 * Note the size we'll actually handle.
	 */
	pix->width	= ov7740_formats[index].width;
	pix->height	= ov7740_formats[index].height;
	pix->bytesperline	= ov7740_formats[index].width*ov7740_formats[index].bpp/8;//TODO:##!!##?
	pix->sizeimage	= ov7740_formats[index].height*pix->bytesperline;
	pix->field	= V4L2_FIELD_NONE;
	return 0;
}

/*
 * Set a format.
 */
static int ov7740_s_fmt(struct i2c_client *c, struct v4l2_format *fmt)
{
#ifdef DEBUG_PRINTK
	printk("##!!##%s called\n",__FUNCTION__);
#endif
	int ret;
	struct ov7740_format_struct *ovfmt;

	//select image FORMAT and PIXEL SIZE
	ret = ov7740_try_fmt(c, fmt, &ovfmt);
	if (ret)
		return ret;

	//initialize cam sensor according to selected format
	ov7740_write_array(c, ovfmt->regs);
	if (machine_is_ipcam())
		ov7740_write_array(c, OV7740_ipcam_opti_regs);

#ifdef DEBUG_YUV422
	ov7740_write_array(c, OV7740_colorbar_pad_regs);//color bar
	//ov7740_write_array(c,OV7740_debug_YUV422_pad_regs);
#endif

	return ret;
}

/*
 * Implement G/S_PARM.  There is a "high quality" mode we could try
 * to do someday; for now, we just do the frame rate tweak.
 */
static int ov7740_g_parm(struct i2c_client *c, struct v4l2_streamparm *parms)
{
	return 0;
}

//keep denom=require rate, num=1 or 0
static int ov7740_s_parm(struct i2c_client *c, struct v4l2_streamparm *parms)
{
	return 0;
}

static int ov7740_s_input(struct i2c_client *c, int *id)
{
	return 0;
}


#if 0
#define   CMATRIX_LEN 6

static int ov7740_store_cmatrix(struct i2c_client *client,
		int matrix[CMATRIX_LEN])
{
	int i, ret;
	unsigned char signbits;

	/*
	 * Weird crap seems to exist in the upper part of
	 * the sign bits register, so let's preserve it.
	 */
	ret = ov7740_read(client, REG_CMATRIX_SIGN, &signbits);
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
		ret += ov7740_write(client, REG_CMATRIX_BASE + i, raw);
	}
	ret += ov7740_write(client, REG_CMATRIX_SIGN, signbits);
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
static const int ov7740_sin_table[] = {
	0,	 87,   173,   258,   342,   422,
	499,	573,   642,   707,   766,   819,
	866,	906,   939,   965,   984,   996,
	1000
};

static int ov7740_sine(int theta)
{
	int chs = 1;
	int sine;

	if (theta < 0) {
		theta = -theta;
		chs = -1;
	}
	if (theta <= 90)
		sine = ov7740_sin_table[theta/SIN_STEP];
	else {
		theta -= 90;
		sine = 1000 - ov7740_sin_table[theta/SIN_STEP];
	}
	return sine*chs;
}

static int ov7740_cosine(int theta)
{
	theta = 90 - theta;
	if (theta > 180)
		theta -= 360;
	else if (theta < -180)
		theta += 360;
	return ov7740_sine(theta);
}




static void ov7740_calc_cmatrix(struct ov7740_info *info,
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
		sinth = ov7740_sine(info->hue);
		costh = ov7740_cosine(info->hue);

		matrix[0] = (matrix[3]*sinth + matrix[0]*costh)/1000;
		matrix[1] = (matrix[4]*sinth + matrix[1]*costh)/1000;
		matrix[2] = (matrix[5]*sinth + matrix[2]*costh)/1000;
		matrix[3] = (matrix[3]*costh - matrix[0]*sinth)/1000;
		matrix[4] = (matrix[4]*costh - matrix[1]*sinth)/1000;
		matrix[5] = (matrix[5]*costh - matrix[2]*sinth)/1000;
	}
}


static int ov7740_t_sat(struct i2c_client *client, int value)
{
	return 0;
	struct ov7740_info *info = i2c_get_clientdata(client);
	int matrix[CMATRIX_LEN];
	int ret;

	info->sat = value;
	ov7740_calc_cmatrix(info, matrix);
	ret = ov7740_store_cmatrix(client, matrix);
	return ret;
}

static int ov7740_q_sat(struct i2c_client *client, __s32 *value)
{
	return 0;
	struct ov7740_info *info = i2c_get_clientdata(client);

	*value = info->sat;
	return 0;
}

static int ov7740_t_hue(struct i2c_client *client, int value)
{
	return 0;
	struct ov7740_info *info = i2c_get_clientdata(client);
	int matrix[CMATRIX_LEN];
	int ret;

	if (value < -180 || value > 180)
		return -EINVAL;
	info->hue = value;
	ov7740_calc_cmatrix(info, matrix);
	ret = ov7740_store_cmatrix(client, matrix);
	return ret;
}


static int ov7740_q_hue(struct i2c_client *client, __s32 *value)
{
	return 0;
	struct ov7740_info *info = i2c_get_clientdata(client);

	*value = info->hue;
	return 0;
}
#endif

/* set saturation */
static int ov7740_t_saturation(struct i2c_client *client, int value)
{
	int ret = 0;

	/* REG_USAT and REG_VSAT should hold the same value; */
	ret = ov7740_write(client, REG_USAT, value);
	ret = ov7740_write(client, REG_VSAT, value);

	return ret;
}

/* get saturation */
static int ov7740_q_saturation(struct i2c_client *client, __s32 *value)
{
	unsigned char v;
	int ret;

	/* REG_USAT and REG_VSAT holds the same value */
	ret = ov7740_read(client, REG_USAT, &v);
	*value = v;
	return ret;
}

//set brightness
static int ov7740_t_brightness(struct i2c_client *client, int value)
{
	unsigned char v;
	int ret;
	//disable AEC/AGC
	ret = ov7740_read(client, REG_REG13, &v);
	v &= ~REG13_AEC_EN;
	ret = ov7740_write(client, REG_REG13, v);

	ret = ov7740_read(client, REG_REG13, &v);
	v &= ~REG13_AGC_EN;
	ret = ov7740_write(client, REG_REG13, v);

	//set brightness
	if(value >= 0){
		ret += ov7740_write(client, REG_YBRIGHT, (value & 0xFF));
		ov7740_read(client, REG_SGNSET, &v);
		v &= ~SGNSET_YBRIGHT_MASK;
		ret += ov7740_write(client, REG_SGNSET, v);
	} else{
		value = -value;
		ret += ov7740_write(client, REG_YBRIGHT, (value & 0xFF));
		ov7740_read(client, REG_SGNSET, &v);
		v |= SGNSET_YBRIGHT_MASK;
		ret += ov7740_write(client, REG_SGNSET, v);
	}

	return ret;
}

//get brightness
static int ov7740_q_brightness(struct i2c_client *client, __s32 *value)
{
	unsigned char v;
	unsigned char sgn;
	int ret;
	ret = ov7740_read(client, REG_YBRIGHT, &v);
	ret += ov7740_read(client, REG_SGNSET, &sgn);
	*value = v;//TODO
	if((sgn & SGNSET_YBRIGHT_MASK) ==  SGNSET_YBRIGHT_MASK)
		*value = -(*value);
	return ret;
}

//TODO:any other regs required?
static int ov7740_t_contrast(struct i2c_client *client, int value)
{
	return ov7740_write(client, REG_YGAIN, (unsigned char) value);
}

static int ov7740_q_contrast(struct i2c_client *client, __s32 *value)
{
	unsigned char v;
	int ret = ov7740_read(client, REG_YGAIN, &v);
	*value = v;
	return ret;
}

static int ov7740_q_hflip(struct i2c_client *client, __s32 *value)
{
	int ret;
	unsigned char v;

	ret = ov7740_read(client, REG_REG0C, &v);
	*value = (v & REG_REG0C) == REG0C_MIRROR_MASK;
	return ret;
}


static int ov7740_t_hflip(struct i2c_client *client, int value)
{
	unsigned char v;
	int ret;

	ret = ov7740_read(client, REG_REG0C, &v);
	if (value)
		v |= REG0C_MIRROR_MASK;
	else
		v &= ~REG0C_MIRROR_MASK;
	msleep(10);
	ret += ov7740_write(client, REG_REG0C, v);
	return ret;
}



static int ov7740_q_vflip(struct i2c_client *client, __s32 *value)
{
	int ret;
	unsigned char v;

	ret = ov7740_read(client, REG_REG0C, &v);
	*value = (v &  REG0C_MIRROR_MASK) == REG0C_MIRROR_MASK;
	return ret;
}


static int ov7740_t_vflip(struct i2c_client *client, int value)
{
	unsigned char v;
	int ret;

	ret = ov7740_read(client, REG_REG0C, &v);
	if (value)
		v |= REG0C_FLIP_MASK;
	else
		v &= ~REG0C_FLIP_MASK;
	msleep(10);
	ret += ov7740_write(client, REG_REG0C, v);
	return ret;
}

static struct ov7740_control {
	struct v4l2_queryctrl qc;
	int (*query)(struct i2c_client *c, __s32 *value);
	int (*tweak)(struct i2c_client *c, int value);
} ov7740_controls[] =
{
	{
		.qc = {
			.id = V4L2_CID_BRIGHTNESS,//the tag used to search inside ov ctrl[]
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Brightness",
			.minimum = -255,
			.maximum = 255,
			.step = 1,
			.default_value = 0x00,
			.flags = V4L2_CTRL_FLAG_SLIDER
		},
		.tweak = ov7740_t_brightness,//set quality
		.query = ov7740_q_brightness,//get quality
	},
	{
		.qc = {
			.id = V4L2_CID_CONTRAST,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Contrast",
			.minimum = 0,
			.maximum = 127,
			.step = 1,
			.default_value = 0x20,
			.flags = V4L2_CTRL_FLAG_SLIDER
		},
		.tweak = ov7740_t_contrast,
		.query = ov7740_q_contrast,
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
		.tweak = ov7740_t_saturation,
		.query = ov7740_q_saturation,
	},
#if 0
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
		.tweak = ov7740_t_hue,
		.query = ov7740_q_hue,
	},
#endif
	{//flip vertical
		.qc = {
			.id = V4L2_CID_VFLIP,
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.name = "Vertical flip",
			.minimum = 0,
			.maximum = 1,
			.step = 1,
			.default_value = 0,
		},
		.tweak = ov7740_t_vflip,
		.query = ov7740_q_vflip,
	},
	{//flip h
		.qc = {
			.id = V4L2_CID_HFLIP,
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.name = "Horizontal mirror",
			.minimum = 0,
			.maximum = 1,
			.step = 1,
			.default_value = 0,
		},
		.tweak = ov7740_t_hflip,
		.query = ov7740_q_hflip,
	},
};
#define N_CONTROLS (ARRAY_SIZE(ov7740_controls))

static struct ov7740_control *ov7740_find_control(__u32 id)
{
	int i;

	//find the ctrl property required by v4l2 by expression of id
	for (i = 0; i < N_CONTROLS; i++)
		if (ov7740_controls[i].qc.id == id)
			return ov7740_controls + i;
	return NULL;
}


//search in ov7740_control[] by id
static int ov7740_queryctrl(struct i2c_client *client,
		struct v4l2_queryctrl *qc)
{
	struct ov7740_control *ctrl = ov7740_find_control(qc->id);

	if (ctrl == NULL)
		return -EINVAL;
	*qc = ctrl->qc;
	return 0;
}

static int ov7740_g_ctrl(struct i2c_client *client, struct v4l2_control *ctrl)
{
	struct ov7740_control *octrl = ov7740_find_control(ctrl->id);
	int ret;

	if (octrl == NULL)
		return -EINVAL;
	ret = octrl->query(client, &ctrl->value);
	if (ret >= 0)
		return 0;
	return ret;
}

//find in ov ctrl[] by id and set sensor
static int ov7740_s_ctrl(struct i2c_client *client, struct v4l2_control *ctrl)
{
	struct ov7740_control *octrl = ov7740_find_control(ctrl->id);
	int ret;

	if (octrl == NULL)
		return -EINVAL;
	ret =  octrl->tweak(client, ctrl->value);//##!!## tag
	if (ret >= 0)
		return 0;
	return ret;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
//get ov register value directly, for debug usage
static int ov7740_g_register(struct i2c_client *client,
						struct v4l2_dbg_register *reg)
{
	return ov7740_read(client, (unsigned char)reg->reg, (unsigned char *)&(reg->val));
}

//set ov register value directly, for debug usage
static int ov7740_s_register(struct i2c_client *client,
						struct v4l2_dbg_register *reg)
{
	return ov7740_write(client, (unsigned char)reg->reg, (unsigned char)reg->val);
}
#endif

static int ov7740_command(struct i2c_client *client, unsigned int cmd,
		void *arg)
{
#ifdef DEBUG_PRINTK
	printk("##!!##%s called\n",__FUNCTION__);
#endif
	switch (cmd) {
	case VIDIOC_DBG_G_CHIP_IDENT:
		return v4l2_chip_ident_i2c_client(client, arg,
				V4L2_IDENT_OV7740, 0);

	case VIDIOC_INT_RESET:
		return ov7740_reset(client);

	case VIDIOC_CROPCAP:
		return ov7740_cropcap(client, (struct v4l2_cropcap *)arg);

	case VIDIOC_ENUM_FMT:
		return ov7740_enum_fmt(client, (struct v4l2_fmtdesc *) arg);

	case VIDIOC_G_FMT:
		return ov7740_g_fmt(client, (struct v4l2_pix_format *)arg);

	case VIDIOC_ENUM_FRAMESIZES:
		return ov7740_enum_framesizes(client, (struct v4l2_frmsizeenum *)arg);

	case VIDIOC_TRY_FMT:
		return ov7740_try_fmt(client, (struct v4l2_format *) arg, NULL);

	case VIDIOC_S_FMT:/*select and set nearest format [format & win size]*/
		return ov7740_s_fmt(client, (struct v4l2_format *) arg);

	case VIDIOC_QUERYCTRL:/*search in ov7740_control[] by id*/
		return ov7740_queryctrl(client, (struct v4l2_queryctrl *) arg);

	case VIDIOC_S_CTRL:/*set ov ctrl*/
		return ov7740_s_ctrl(client, (struct v4l2_control *) arg);

	case VIDIOC_G_CTRL:
		return ov7740_g_ctrl(client, (struct v4l2_control *) arg);

	case VIDIOC_S_PARM:/*not implemented*/
		return ov7740_s_parm(client, (struct v4l2_streamparm *) arg);

	case VIDIOC_G_PARM:/*not implemented*/
		return ov7740_g_parm(client, (struct v4l2_streamparm *) arg);

	case VIDIOC_S_INPUT:/*not implemented*/
		return ov7740_s_input(client, (int *) arg);

#ifdef CONFIG_VIDEO_ADV_DEBUG
	case VIDIOC_DBG_G_REGISTER:/*get register val for debug*/
		return ov7740_g_register(client, (struct v4l2_register *) arg);

	case VIDIOC_DBG_S_REGISTER:/*set register val for debug*/
		return ov7740_s_register(client, (struct v4l2_register *) arg);
#endif
	}
	return -EINVAL;
}

//**************************************************************************//
//**************************************************************************//

int ccic_sensor_attach(struct i2c_client *client);
//int ccic_sensor_detach(struct i2c_client *client);

/*
 * Basic i2c stuff.
 */
extern struct clk *pxa168_ccic_gate_clk;
static int __devinit ov7740_probe(struct i2c_client *client, const struct i2c_device_id *dev_id)
{
	int ret;
	struct ov7740_info *info;
	struct sensor_platform_data *pdata;
	pdata = client->dev.platform_data;

	/*
	 * ccic_set_clock_parallel();
	 * ccic_enable_mclk();
	 */
	clk_enable(pxa168_ccic_gate_clk);
	ccic_set_clock_parallel();/*clock must be enabled before power on.*/

	pdata->power_on(1, 0);
	info = kzalloc(sizeof (struct ov7740_info), GFP_KERNEL);
	if (! info) {
		ret = -ENOMEM;
		goto out_free;
	}
	info->fmt = &ov7740_formats[0];
	info->sat = 128;	/* Review this */
	i2c_set_clientdata(client, info);

	/*
	 * Make sure it's an ov7740
	 */

	ret = ov7740_detect(client);
	if (ret)
		goto out_free_info;
	printk(KERN_INFO"OmniVision ov7740 sensor detected\n");

#ifdef DEBUG_YUV422
	ov7740_init(client);
	//while(1);
#endif

	ccic_sensor_attach(client);

	/*ccic_disable_mclk();*/
	ccic_disable_clock();

	pdata->power_on(0, 0);	//for power optimization

	printk(KERN_NOTICE "OV7740 i2c-probe detected\n");
	return 0;

out_free_info:
	kfree(info);
out_free:
	return ret;
}


static int ov7740_remove(struct i2c_client *client)
{
	struct ov7740_info *info;
	struct sensor_platform_data *pdata;

	//info = client->dev.driver_data;
	info = i2c_get_clientdata(client);
	kfree(info);

	//ccic_sensor_detach(client);//TODO:why declared static

	/*ccic_disable_mclk();*/
	ccic_disable_clock();

	pdata = client->dev.platform_data;
	pdata->power_on(0, 0);	//for power optimization
	printk(KERN_NOTICE "OV7740 i2c-remove\n");
	return 0;
}

static struct i2c_device_id ov7740_idtable[] = {
	{"ov7740", 0 },
	{},
};

MODULE_DEVICE_TABLE(i2c, ov7740_idtable);

static struct i2c_driver ov7740_driver = {
	.driver = {
		.name	= "ov7740",
	},
	.id_table       = ov7740_idtable,
	.command	= ov7740_command,
	.probe		= ov7740_probe,
	.remove		= ov7740_remove,
};

/*
 * Module initialization
 */
static int __init ov7740_mod_init(void)
{
	printk(KERN_NOTICE"OmniVision ov7740 sensor driver, at your service\n");
	return i2c_add_driver(&ov7740_driver);
}

static void __exit ov7740_mod_exit(void)
{
	i2c_del_driver(&ov7740_driver);
}

late_initcall(ov7740_mod_init);
//module_init(ov7740_mod_init);
module_exit(ov7740_mod_exit);

