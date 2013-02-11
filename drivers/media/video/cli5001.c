/*
 * A V4L2 driver for OmniVision CLI5001 camera ISP.
 *
 * This file may be distributed under the terms of the GNU General
 * Public License, version 2.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <media/v4l2-common.h>
#include <media/v4l2-chip-ident.h>
#include <linux/i2c.h>
#include <linux/firmware.h>
#include <linux/clk.h>
#include "pxa168_camera.h"
#include "cli5001.h"
#include <mach/camera.h>


MODULE_AUTHOR("Jun Nie <njun@marvell.com>");
MODULE_DESCRIPTION("A driver for cli5001 camera ISP");
MODULE_LICENSE("GPL");

static struct i2c_client *c;

/*
 * Basic window sizes.  These probably belong somewhere more globally
 * useful.
 */
#define VGA_WIDTH	640
#define VGA_HEIGHT	480
#define QXGA_WIDTH	2048
#define QXGA_HEIGHT	1536

/*
 * Our nominal (default) frame rate.
 */
#define CLI5001_FRAME_RATE 30

/*
 * The cli5001 sits on i2c with ID 0x42
 */
#define CLI5001_I2C_ADDR 0x42

/* Registers */


/*
 * Information we maintain about a known sensor.
 */
struct cli5001_format_struct;  /* coming later */
struct cli5001_info {
	struct cli5001_format_struct *fmt;  /* Current format */
	unsigned char sat;		/* Saturation value */
	int hue;			/* Hue value */
};

/*
 * Low-level register I/O.
 */

static int cli5001_set_reg_bank(struct i2c_client *c, char txData)
{
	int ret;
	unsigned char buf[2];
	static char bank = 0;

	if(bank == txData)
		return 0;

	buf[0] = 0x01;
	buf[1] = bank = txData;
	struct i2c_msg msg[] = {
		{
			.addr = 0x41, /* cli5001 address high byte access I2C address */
			.flags = 0,
			.len = 2,
			.buf = &buf,
		},
	};

	ret = i2c_transfer(c->adapter, msg, 1);
	if (ret < 0) {
		printk(KERN_ERR "cli5001: set reg bank error %d\n", ret);
		return ret;
	}
	return 0;
}

static int cli5001_read(struct i2c_client *c,
		u16 reg)
{
	int ret;

	ret = cli5001_set_reg_bank(c, (u8)(reg >> 8));
	if(ret < 0)
		return ret;
	ret = i2c_smbus_write_byte(c, (u8)(reg & 0xff));
	if(ret < 0)
		return ret;
	return i2c_smbus_read_byte(c);
}

static int cli5001_write(struct i2c_client *c,
		u16 reg, char value)
{
	int ret;

	ret = cli5001_set_reg_bank(c, (u8)(reg >> 8));
	if(ret < 0){
		printk("set cli5001 bank fail!\n");
		return ret;
	}
	ret = i2c_smbus_write_byte_data(c, (u8)(reg & 0xff), value);
	if (ret < 0) {
		printk(KERN_ERR "cli5001_write error %d reg 0x%x val 0x%x\n",
				ret, reg, value);
		return ret;
	}
	return 0;
}

/* ISP vender macro */
#define CoreISP2_IIC_Read(v) cli5001_read(c, v)
#define	CoreISP2_IIC_Write(addr, v) cli5001_write(c, addr, v)
#define DebugMessage printk
#define WaitTime_us(a) msleep((a)/1000)
typedef unsigned long long	ClUint_64;
typedef unsigned int		ClUint_32;
typedef unsigned short		ClUint_16;
typedef unsigned char		ClUint_8;
typedef long				ClSint_64;
typedef int					ClSint_32;
typedef short				ClSint_16;
typedef char				ClSint_8;
typedef char				Cl_Char;
typedef ClUint_32			Cl_Handle;
typedef ClUint_8			Cl_Bool;

#define _ROTATION_NORMAL_                   1
#define _ROTATION_HORIZONTAL_               2
#define _ROTATION_VERTICAL_                 3
#define _ROTATION_HORIZONTAL_VERTICAL_      4

#define _QVGA_RES_                   1
#define _HVGA_RES_               2
#define _3M_RES_                 3
#define _XGA_RES_                 4

/*
 * Stuff that knows about the sensor.
 */
static void cli5001_reset(struct i2c_client *client)
{
}

static void CoreISP2_Send_Command( ClUint_8 Cmd_Type )
{
	int count = 100;

	CoreISP2_IIC_Write(0xebb0, Cmd_Type);

	while(CoreISP2_IIC_Read(0xebb0) != 0x00)
	{
		if(count < 0)
		{
			DebugMessage("isp2 command time out 0x%x  [0x%x]\n", Cmd_Type, CoreISP2_IIC_Read(0xebb0));
			return;
		}
		count--;
		WaitTime_us(100000);    // 100ms
	}
	DebugMessage("isp2 command finish: 0x%x \n", Cmd_Type);
}

void CoreISP2_Mode_Change(ClUint_8 res)
{
	CoreISP2_IIC_Write(0xe64a, _ROTATION_HORIZONTAL_VERTICAL_);

	switch(res){
		case _3M_RES_:
			CoreISP2_IIC_Write(0xe6a8, 0);  // 3M
			//CoreISP2_Send_Command(0xb0); //init
			CoreISP2_Send_Command(0xdb);  // capture command for 3M
			break;
		case _HVGA_RES_: //480x320

			//CoreISP2_Send_Command(0xb0); //init
			CoreISP2_Send_Command(0xd0); // preview command for HVGA
			break;

		case _QVGA_RES_: //320x240
			//CoreISP2_Send_Command(0xb0); // init cmd
			CoreISP2_Send_Command(0xd1); // preview command for QVGA
			break;
		case _XGA_RES_: //1024x768
			//CoreISP2_Send_Command(0xb0); // init cmd
			CoreISP2_Send_Command(0xd3); // preview command for 1024x768
			break;
		default:
			break;
	}
}

static void CoreISP2_FullAF(void)
{
	int count = 50;

	CoreISP2_Send_Command(0xF3); 

	while(CoreISP2_IIC_Read(0xebb1) != 0x24)   // 0x24 means finish
	{
		if(count < 0)
			return;
		count--;
		WaitTime_us(50000);    // 50ms
	}	  	
}

static void CoreISP2_WBmode(ClUint_8 wbmode)
{
	switch (wbmode)
	{
		case 0 : //auto
			CoreISP2_IIC_Write(0xeb98, 0x00);
			break;
		case 1 : //cloudy
			CoreISP2_IIC_Write(0xeb98, 0x01);
			break;
		case 2 : //sunnyl
			CoreISP2_IIC_Write(0xeb98, 0x02);
			break;
		case 3 : //fluorescent
			CoreISP2_IIC_Write(0xeb98, 0x03);
			break;
		case 4 : //incandescent
			CoreISP2_IIC_Write(0xeb98, 0x06);
			break;
		default :
			CoreISP2_IIC_Write(0xeb98, 0x00);
			break;
	
			}
}	

static void CoreISP2_Set_Exposure_mode(u8 Exmode)
{
	switch (Exmode)
	{
		case 0 : //  -2 level
			CoreISP2_IIC_Write(0xebb6, 0x01);
			CoreISP2_IIC_Write(0xebb6, 0xc6);
			break;
		case 1 : // -1 level
			CoreISP2_IIC_Write(0xebb6, 0x02);
			CoreISP2_IIC_Write(0xebb6, 0xc6);
			break;
		case 2 : // 0 level 
			CoreISP2_IIC_Write(0xebb6, 0x03);
			CoreISP2_IIC_Write(0xebb6, 0xc6);
			break;
		case 3 : // 1 level
			CoreISP2_IIC_Write(0xebb6, 0x04);
			CoreISP2_IIC_Write(0xebb6, 0xc6);
			break;
		case 4 : // 2 level
			CoreISP2_IIC_Write(0xebb6, 0x05);
			CoreISP2_IIC_Write(0xebb6, 0xc6);
			break;
		default :
			CoreISP2_IIC_Write(0xebb6, 0x03);
			CoreISP2_IIC_Write(0xebb6, 0xc6);
			break;
	}
}

static void CoreISP2_Set_Effect_mode(u8 Effectmode)
{
	switch (Effectmode)
	{
		case 0:  //Normal : ImgEffect 
			CoreISP2_IIC_Write(0xee20, 0x00);
			break;

		case 1:  //Warm : ImgEffectB 
			CoreISP2_IIC_Write(0xee20, 0x01);
			break;

		case 2:  //Cool : ImgEffectB 
			CoreISP2_IIC_Write(0xee20, 0x02);
			break;

		case 3:  //Fog : ImgEffectB 
			CoreISP2_IIC_Write(0xee20, 0x03);
			break;						

		case 4:  //Opp_Neg : ImgEffectB 
			CoreISP2_IIC_Write(0xee20, 0x04);
			break;					

		case 5:  //Emboss : ImgEffectB 
			CoreISP2_IIC_Write(0xee20, 0x40);
			break;					

		case 6:  //Sketh1 : ImgEffectB 
			CoreISP2_IIC_Write(0xee20, 0x80);
			break;					

		case 7:  //Sketh2 : ImgEffectB 
			CoreISP2_IIC_Write(0xee20, 0xc0);
			break;					

		case 8:  //Opp_Neg : ImgEffectB 
			CoreISP2_IIC_Write(0xee20, 0x04);
			break;					
			// YUV effect	
		case 9:  //Sol : ImgEffectC 
			CoreISP2_IIC_Write(0xee22, 0x01);
			break;

		case 10:  //Aqua : ImgEffectC 
			CoreISP2_IIC_Write(0xee22, 0x02);
			break;

		case 11:  //Pos : ImgEffectC 
			CoreISP2_IIC_Write(0xee22, 0x03);
			break;

		case 12:  //Green : ImgEffectC 
			CoreISP2_IIC_Write(0xee22, 0x04);
			break;						

		case 13:  //Vio : ImgEffectC 
			CoreISP2_IIC_Write(0xee22, 0x05);
			break;					

		case 14:  //Soft : ImgEffectC 
			CoreISP2_IIC_Write(0xee22, 0x06);
			break;					

		case 15:  //Bin : ImgEffectC 
			CoreISP2_IIC_Write(0xee22, 0x07);
			break;					

		case 16:  //Orange : ImgEffectC 
			CoreISP2_IIC_Write(0xee22, 0x08);
			break;								

	}
}

//float
/*
static ClUint_32 CoreISP2_Get_Version(void)
{
	ClUint_32 SW_Ver;
	ClUint_16 majorVer;
	ClUint_16 minorVer;
	ClUint_16 data1;
	ClUint_16 data0;

	CoreISP2_Send_Command(0x14);  // Jacky change this

	data1  = CoreISP2_IIC_Read(0xE628);
	data0  = CoreISP2_IIC_Read(0xE629);
	majorVer   = data1<<8 | data0;

	data1 = CoreISP2_IIC_Read(0xE62A);
	data0 = CoreISP2_IIC_Read(0xE62B);
	minorVer  = data1<<8 | data0;

	SW_Ver = majorVer <<16 | minorVer;

	DebugMessage(" ====Get ISP vision  %d [0x%x]====\n", SW_Ver, SW_Ver);
	return SW_Ver;

}

*/

static int cli5001_init(struct i2c_client *client)
{
	int ret, data_cnt, i;
	const struct firmware *fw;
	size_t Size;
	u8* pInitialData;
	u32 tempdiv,temprem, wData;
	static int firmware_in = 1;

	c = client;

	/* download MCU binary */
	if(firmware_in){
		firmware_in = 0;
		ret = request_firmware(&fw, "cli5001.bin", &c->dev);
		if (ret){
			printk("request ISP firmware fail\n");
			goto init;
		}

		pInitialData = fw->data;
		Size = fw->size;

		// -LSC Size Setting
		CoreISP2_IIC_Write(0xE2D2, LSC_ISP2_SIZE/256);
		CoreISP2_IIC_Write(0xE2D3, LSC_ISP2_SIZE%256);

		// -Download LSC file
		for(data_cnt=0; data_cnt<LSC_ISP2_SIZE; data_cnt++)
		{
			CoreISP2_IIC_Write(0xE2D4, LSC_INIT_TABLE[data_cnt]);
		}

		WaitTime_us(1000*10);

		wData = CoreISP2_IIC_Read(0xE001);
		wData = (wData&0xFD)|0x02;
		CoreISP2_IIC_Write(0xE001, wData); // once LSC Data downloading, enable LSF.	
		DebugMessage("LSC Download End \n");

		//code ram download 
		CoreISP2_IIC_Write(0xE070,0xFD);
		CoreISP2_IIC_Write(0xE0A2,0x04);        
		CoreISP2_IIC_Write(0xE0A3,0x00);
		CoreISP2_IIC_Write(0xE0A4,0x00);

		DebugMessage("Bin code size = %d\n",Size);

		tempdiv = Size/256;
		temprem = Size%256;
		CoreISP2_IIC_Write(0xE0A6, tempdiv);
		CoreISP2_IIC_Write(0xE0A7, temprem);
		WaitTime_us(1000*1);

		DebugMessage("Bin code download start \n");
		for(i=0; i<Size; i++)
		{
			CoreISP2_IIC_Write(0xE0A5, pInitialData[i]);
		}
		WaitTime_us(1000*50);
		DebugMessage("BIN Download End \n");
		release_firmware(fw);
	
		CoreISP2_IIC_Write(0xe060, 0x03);   //PLL OFF
		CoreISP2_IIC_Write(0xe062, 0x0c);  // Pll setting    0000 1100   P=3  S=0
		WaitTime_us(1000);    // 1ms
		CoreISP2_IIC_Write(0xe061, 0x19);  //Pll setting   M=25
		CoreISP2_IIC_Write(0xe060, 0x00);  // pll on
		// end
	
		// Sensor Reset
		CoreISP2_IIC_Write(0xe010, 0xe0);   //[6:6] S1_RST=0 Cis1IntA
		WaitTime_us(10000);    // 10ms  
		CoreISP2_IIC_Write(0xe010, 0xc0);   //[6:6] S1_RST=0 Cis1IntA
		WaitTime_us(10000);    // 10ms  
		CoreISP2_IIC_Write(0xe010, 0xe0);   //[6:6] S1_RST=1
	
		CoreISP2_IIC_Write(0xe011,0x20);        // Cis1IntB 
		CoreISP2_IIC_Write(0xe012,0x02);        // Cis1IntC
		CoreISP2_IIC_Write(0xe013,0x21);        // Cis1IntD
		CoreISP2_IIC_Write(0xe014,0x20);        // Cis1IntE 
	
	
		CoreISP2_IIC_Write(0xee16,0x26);        // JpgOutMode [7:4]
		CoreISP2_IIC_Write(0xee1a,0x01);        // Jmclk
		
		CoreISP2_IIC_Write(0xe64a, _ROTATION_HORIZONTAL_VERTICAL_); // _ROTATION_HORIZONTAL_VERTICAL_ == 4
	
		CoreISP2_IIC_Write(0xE070,0x05);  // Cis1IntA
		WaitTime_us(1000*5);
		CoreISP2_IIC_Write(0xE070,0x04);  // S1_RST=0
		WaitTime_us(1000*5);
	
		//CoreISP2_Send_Command(_QVGA_RES_);  // sensor initial  for 320x240
		CoreISP2_Send_Command(0xb0); //init
		CoreISP2_Mode_Change (_HVGA_RES_);
	
		DebugMessage("CoreISP2 init finish\n");
	
		//CoreISP2_Get_Version();
	
		DebugMessage("CoreISP2 Scl Window[%d  %d  %d %d] \n", CoreISP2_IIC_Read(0xee40)<<8|CoreISP2_IIC_Read(0xee41)
				, CoreISP2_IIC_Read(0xee42)<<8|CoreISP2_IIC_Read(0xee43)
				, CoreISP2_IIC_Read(0xee44)<<8|CoreISP2_IIC_Read(0xee45)
				, CoreISP2_IIC_Read(0xee46)<<8|CoreISP2_IIC_Read(0xee47)
			    );
	
	}
	return 0;
init:
	printk("cli5001 init fail, ret %d\n", ret);
	return ret;
}

#define REG_I2CDEVID 0xe030
static int cli5001_detect(struct i2c_client *c)
{
	unsigned char v;

	v = CoreISP2_IIC_Read(REG_I2CDEVID);

	if (v != 0x60) /* i2c device id. */
	{
		printk("i2c id error!\n");
		return -1;
	}
	/* sleep */
	CoreISP2_IIC_Write(0xE060, 0x3);
	return 0;
}

/*
 * Store information about the video data format.  The color matrix
 * is deeply tied into the format, so keep the relevant values here.
 * The magic matrix nubmers come from OmniVision.
 */
static struct cli5001_format_struct {
	__u8 *desc;
	__u32 pixelformat;
	//reg_array *regs_array;
	//int cmatrix[CMATRIX_LEN];
	int bpp;   /* Bytes per pixel */
} cli5001_formats[] = {
	{
		.desc		= "YUYV 4:2:0",
		.pixelformat	= V4L2_PIX_FMT_YUV420,
		//.regs_array 		= cli5001_fmt_yuv422,
		//.cmatrix	= { 128, -128, 0, -34, -94, 128 },
		.bpp		= 12,
	},
	{
		.desc		= "JPEG",
		.pixelformat	= V4L2_PIX_FMT_JPEG,
		//.regs_array 		= cli5001_fmt_yuv422,
		//.cmatrix	= { 128, -128, 0, -34, -94, 128 },
		.bpp		= 16,
	},
	{
		.desc		= "JPEG_YUV",
		.pixelformat	= V4L2_PIX_FMT_YUYV,
		//.regs_array 		= cli5001_fmt_yuv422,
		//.cmatrix	= { 128, -128, 0, -34, -94, 128 },
		.bpp		= 16,
	},
};
#define N_CLI5001_FMTS ARRAY_SIZE(cli5001_formats)

static struct cli5001_win_size {
	int	width;
	int	height;
} cli5001_win_sizes[] = {
	/* QXGA */
	{
		.width		= 2048,
		.height		= 1536,
	},
	/* XGA */
	{
		.width		= 1024,
		.height		= 768,
	},
	/* HVGA */
	{
		.width		= 480,
		.height		= 320,
	},
	/* QVGA */
	{
		.width		= 320,
		.height		= 240,
	},
};

#define N_WIN_SIZES (ARRAY_SIZE(cli5001_win_sizes))

static int cli5001_enum_fmt(struct i2c_client *c, struct v4l2_fmtdesc *fmt)
{
	struct cli5001_format_struct *ofmt;

	if (fmt->index >= N_CLI5001_FMTS)
		return -EINVAL;

	ofmt = cli5001_formats + fmt->index;
	fmt->flags = 0;
	strcpy(fmt->description, ofmt->desc);
	fmt->pixelformat = ofmt->pixelformat;
	return 0;
}


static int cli5001_try_fmt(struct i2c_client *c, struct v4l2_format *fmt,
		struct cli5001_format_struct **ret_fmt,
		struct cli5001_win_size **ret_wsize)
{
	int index;
	struct cli5001_win_size *wsize;
	struct v4l2_pix_format *pix = &fmt->fmt.pix;

	for (index = 0; index < N_CLI5001_FMTS; index++)
		if (cli5001_formats[index].pixelformat == pix->pixelformat)
			break;
	if (index >= N_CLI5001_FMTS) {
		/* default to first format */
		index = 0;
		pix->pixelformat = cli5001_formats[0].pixelformat;
	}
	if (ret_fmt != NULL)
		*ret_fmt = cli5001_formats + index;
	/*
	 * Fields: the OV devices claim to be progressive.
	 */
	pix->field = V4L2_FIELD_NONE;
#if 0
	/*
	 * Round requested image size down to the nearest
	 * we support, but not below the smallest.
	 */
	for (wsize = cli5001_win_sizes; wsize < cli5001_win_sizes + N_WIN_SIZES;
	     wsize++)
		if (pix->width >= wsize->width && pix->height >= wsize->height)
			break;
	if (wsize >= cli5001_win_sizes + N_WIN_SIZES)
		wsize--;   /* Take the smallest one */
	if (ret_wsize != NULL)
		*ret_wsize = wsize;
	/*
	 * Note the size we'll actually handle.
	 */
	pix->width = wsize->width;
	pix->height = wsize->height;
#endif
	pix->bytesperline = pix->width*cli5001_formats[index].bpp >> 3;
	pix->sizeimage = pix->height*pix->bytesperline;
	printk("cli5001 w %d h %d bpl %d sizeimage %d\n",
			pix->width, pix->height, pix->bytesperline, pix->sizeimage);
	return 0;
}

/*
 * Set a format.
 */
static int cli5001_s_fmt(struct i2c_client *c, struct v4l2_format *fmt)
{
	int ret;
	struct cli5001_format_struct *ovfmt;
	struct cli5001_win_size *wsize;
	struct cli5001_info *info = i2c_get_clientdata(c);
	unsigned char v;

	ret = cli5001_try_fmt(c, fmt, &ovfmt, &wsize);
	if (ret)
		return ret;

	switch(fmt->fmt.pix.width){
		case 320:
			ret = _QVGA_RES_;
			break;
		case 480:
			ret = _HVGA_RES_;
			break;
		case 1024:
			ret = _XGA_RES_;
			break;
		default:
			ret = _3M_RES_;
	}
	CoreISP2_Mode_Change (ret);

	if(fmt->fmt.pix.pixelformat == V4L2_PIX_FMT_YUV420){
		//CoreISP2_IIC_Write(0xe059, 0x80); // Enabled YUV420 format
		printk("cli5001 fmt YUV420\n");
	}else{
		CoreISP2_IIC_Write(0xe059, 0x83); // Enabled JPEG format
		printk("cli5001 fmt JPEG\n");
	}
	return 0;
}

/*
 * Implement G/S_PARM.  There is a "high quality" mode we could try
 * to do someday; for now, we just do the frame rate tweak.
 */
static int cli5001_g_parm(struct i2c_client *c, struct v4l2_streamparm *parms)
{
	struct v4l2_captureparm *cp = &parms->parm.capture;
	unsigned char clkrc;
	int ret;

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	memset(cp, 0, sizeof(struct v4l2_captureparm));
	cp->capability = V4L2_CAP_TIMEPERFRAME;
	cp->timeperframe.numerator = 1;
	cp->timeperframe.denominator = CLI5001_FRAME_RATE;
	return 0;
}

static int cli5001_s_parm(struct i2c_client *c, struct v4l2_streamparm *parms)
{
	struct v4l2_captureparm *cp = &parms->parm.capture;
	struct v4l2_fract *tpf = &cp->timeperframe;
	unsigned char clkrc;
	int ret, div;

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	if (cp->extendedmode != 0)
		return -EINVAL;
	if (tpf->numerator == 0 || tpf->denominator == 0)
		div = 1;  /* Reset to full rate */
	else
		div = (tpf->numerator*CLI5001_FRAME_RATE)/tpf->denominator;
	tpf->numerator = 1;
	tpf->denominator = CLI5001_FRAME_RATE/div;
	return 0;
}



/*
 * Code for dealing with controls.
 */

static int cli5001_t_contrast(int value)
{
	return 0;
}

static int cli5001_t_vflip(int value)
{
	return 0;
}

static struct cli5001_control {
	struct v4l2_queryctrl qc;
	void (*tweak)(int value);
} cli5001_controls[] =
{
	{
		.qc = {
			.id = V4L2_CID_FOCUS_AUTO,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Auto Focus",
			.flags = V4L2_CTRL_FLAG_SLIDER
		},
		.tweak = CoreISP2_FullAF,
	},
	{
		.qc = {
			.id = V4L2_CID_DO_WHITE_BALANCE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Do White Balance",
			.minimum = 0,
			.maximum = 4,
			.step = 1,
			.default_value = 0,
			.flags = V4L2_CTRL_FLAG_SLIDER
		},
		.tweak = CoreISP2_WBmode,
	},
	{
		.qc = {
			.id = V4L2_CID_EXPOSURE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Exposure",
			.minimum = 0,
			.maximum = 4,
			.step = 1,
			.default_value = 0,
			.flags = V4L2_CTRL_FLAG_SLIDER
		},
		.tweak = cli5001_t_contrast,
	},
	{
		.qc = {
			.id = V4L2_CID_HUE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "ColorEffect",
			.minimum = 0,
			.maximum = 16,
			.step = 1,
			.default_value = 0,
			.flags = V4L2_CTRL_FLAG_SLIDER
		},
		.tweak = cli5001_t_contrast,
	},
	{
		.qc = {
			.id = V4L2_CID_CONTRAST,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Contrast",
			.minimum = 0,
			.maximum = 127,
			.step = 1,
			.default_value = 0x40,   /* XXX cli5001 spec */
			.flags = V4L2_CTRL_FLAG_SLIDER
		},
		.tweak = cli5001_t_contrast,
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
		.tweak = cli5001_t_vflip,
	},
};
#define N_CONTROLS (ARRAY_SIZE(cli5001_controls))

static struct cli5001_control *cli5001_find_control(__u32 id)
{
	int i;

	for (i = 0; i < N_CONTROLS; i++)
		if (cli5001_controls[i].qc.id == id)
			return cli5001_controls + i;
	return NULL;
}


static int cli5001_queryctrl(struct i2c_client *client,
		struct v4l2_queryctrl *qc)
{
	struct cli5001_control *ctrl = cli5001_find_control(qc->id);

	if (ctrl == NULL)
		return -EINVAL;
	*qc = ctrl->qc;
	return 0;
}

static int cli5001_g_ctrl(struct i2c_client *client, struct v4l2_control *ctrl)
{
	/* we do not support get control, just set it */
	return 0;
}

static int cli5001_s_ctrl(struct i2c_client *client, struct v4l2_control *ctrl)
{
	struct cli5001_control *octrl = cli5001_find_control(ctrl->id);
	int ret = 0;
	if (octrl == NULL)
		return -EINVAL;
	octrl->tweak((u8)(ctrl->value));
	return ret;
}

/*
 * Basic i2c stuff.
 */

static int cli5001_s_input(struct i2c_client *c, int *id)
{
	return 0;
}

static int cli5001_streamon(struct i2c_client *c)
{
	//CoreISP2_IIC_Write(0xe060, 0x00);  // pll on
	return 0;
}

static int cli5001_streamoff(struct i2c_client *c)
{
	DebugMessage("cli5001_streamoff\n");
	//CoreISP2_IIC_Write(0xE060, 0x3);
	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int cli5001_g_register(struct i2c_client *client, struct v4l2_dbg_register * reg)
{
	reg->val = cli5001_read(client, (u16)reg->reg);
	return 0;
}

static int cli5001_s_register(struct i2c_client *client, struct v4l2_dbg_register * reg)
{
	return cli5001_write(client, (u16)reg->reg, (unsigned char)reg->val);
}
#endif

static int cli5001_command(struct i2c_client *client, unsigned int cmd,
		void *arg)
{
	int ret = -EINVAL;

	switch (cmd) {
	case VIDIOC_DBG_G_CHIP_IDENT:
		ret = v4l2_chip_ident_i2c_client(client, arg, V4L2_IDENT_CLI5001, 0);

	case VIDIOC_INT_RESET:
		cli5001_reset(client);
		ret = 0;
		break;

	case VIDIOC_INT_INIT:
		ret = cli5001_init(client);
		break;

	case VIDIOC_ENUM_FMT:
		ret = cli5001_enum_fmt(client, (struct v4l2_fmtdesc *) arg);
		break;
	case VIDIOC_TRY_FMT:
		ret = cli5001_try_fmt(client, (struct v4l2_format *) arg, NULL, NULL);
		break;
	case VIDIOC_S_FMT:
		ret = cli5001_s_fmt(client, (struct v4l2_format *) arg);
		break;
	case VIDIOC_QUERYCTRL:
		ret = cli5001_queryctrl(client, (struct v4l2_queryctrl *) arg);
		break;
	case VIDIOC_S_CTRL:
		ret = cli5001_s_ctrl(client, (struct v4l2_control *) arg);
		break;
	case VIDIOC_G_CTRL:
		ret = cli5001_g_ctrl(client, (struct v4l2_control *) arg);
		break;
	case VIDIOC_S_PARM:
		ret = cli5001_s_parm(client, (struct v4l2_streamparm *) arg);
		break;
	case VIDIOC_G_PARM:
		ret = cli5001_g_parm(client, (struct v4l2_streamparm *) arg);
		break;
	case VIDIOC_S_INPUT:
		ret = cli5001_s_input(client, (int *) arg);
		break;
	case VIDIOC_STREAMON:
		ret = cli5001_streamon(client);
		break;
	case VIDIOC_STREAMOFF:
		ret = cli5001_streamoff(client);
		break;
#ifdef CONFIG_VIDEO_ADV_DEBUG
	case VIDIOC_DBG_G_REGISTER:
		ret = cli5001_g_register(client, (struct v4l2_dbg_register *) arg);
		break;
	case VIDIOC_DBG_S_REGISTER:
		ret = cli5001_s_register(client, (struct v4l2_dbg_register *) arg);
		break;
#endif
	default:
		break;
	}
	return ret;
}

extern struct clk *pxa168_ccic_gate_clk;
int ccic_sensor_attach(struct i2c_client *client);
static int __devinit cli5001_probe(struct i2c_client *client)
{
	int ret;
	struct cli5001_info *info;
	struct sensor_platform_data *pdata;
	pdata = client->dev.platform_data;

	clk_enable(pxa168_ccic_gate_clk);
	ccic_set_clock_parallel();	//clock must be enabled before power on.

	pdata->power_on(1, 1);
	/*
	 * Set up our info structure.
	 */
	info = kzalloc(sizeof (struct cli5001_info), GFP_KERNEL);
	if (! info) {
		ret = -ENOMEM;
		goto out_free;
	}
	info->fmt = &cli5001_formats[1];
	info->sat = 128;	/* Review this */
	i2c_set_clientdata(client, info);
	/*
	 * Make sure it's an cli5001
	 */
	ret = cli5001_detect(client);
	if (ret)
		goto out_free_info;

	ccic_sensor_attach(client);
	//pdata->power_on(0, 1);
	ccic_disable_clock();
	printk("cli5001 detected\n");
	return 0;

out_free_info:
	kfree(info);
out_free:
	return ret;
}

static int cli5001_remove(struct i2c_client *client)
{
	return 0;	//TODO
}

static struct i2c_device_id cli5001_idtable[] = {
	{ "cli5001", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, cli5001_idtable);

static struct i2c_driver cli5001_driver = {
	.driver = {
		.name = "cli5001",
	},
	.id_table       = cli5001_idtable,
	.command	= cli5001_command,
	.probe		= cli5001_probe,
	.remove		= cli5001_remove,
};


/*
 * Module initialization
 */
static int __init cli5001_mod_init(void)
{
	printk(KERN_NOTICE "cli5001 camera ISP driver, at your service\n");
	return i2c_add_driver(&cli5001_driver);
}

static void __exit cli5001_mod_exit(void)
{
	i2c_del_driver(&cli5001_driver);
}

late_initcall(cli5001_mod_init);
module_exit(cli5001_mod_exit);
