/*
 * linux/drivers/video/pxa168fb_ovly.c -- Marvell PXA168 LCD Controller
 *
 * Copyright (C) Marvell Semiconductor Company.  All rights reserved.
 *
 * 2009-03-19   adapted from original version for PXA168
 * 		Green Wan <gwan@marvell.com>
 *		Kevin Liu <kliu5@marvell.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file COPYING in the main directory of this archive for
 * more details.
 */

/*
 * 1. Adapted from:  linux/drivers/video/skeletonfb.c
 * 2. Merged from: linux/drivers/video/dovefb.c (Lennert Buytenhek)
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/cpufreq.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/uaccess.h>
#include <linux/console.h>

//#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>

#include <mach/pxa168fb.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <asm/mach-types.h>
#include "pxa168fb.h"

#ifdef CONFIG_DVFM
#include <mach/dvfm.h>
static int dvfm_dev_idx;
static void set_dvfm_constraint(void)
{
	/* Disable Lowpower mode */
	dvfm_disable_op_name("apps_idle", dvfm_dev_idx);
	dvfm_disable_op_name("apps_sleep", dvfm_dev_idx);
	dvfm_disable_op_name("sys_sleep", dvfm_dev_idx);
}

static void unset_dvfm_constraint(void)
{
	/* Enable Lowpower mode */
	dvfm_enable_op_name("apps_idle", dvfm_dev_idx);
	dvfm_enable_op_name("apps_sleep", dvfm_dev_idx);
	dvfm_enable_op_name("sys_sleep", dvfm_dev_idx);
}
#else
static void set_dvfm_constraint(void) {}
static void unset_dvfm_constraint(void) {}
#endif

#define RESET_BUF	0x1
#define FREE_ENTRY	0x2

static int pxa168fb_set_par(struct fb_info *fi);
static void set_graphics_start(struct fb_info *fi, int xoffset, int yoffset);
static void set_dma_control0(struct pxa168fb_info *fbi);
static int wait_for_vsync(struct pxa168fb_info *fbi);
static int pxa168fb_pan_display(struct fb_var_screeninfo *var, struct fb_info *fi);

static int pxa168fb_switch_buff(struct fb_info *fi);
static int addFreeBuf(u8 **bufList, u8 *freeBuf);
static void clearFreeBuf(u8 **bufList, int iFlag);
static void clearFilterBuf(u8 *bufList[][3], int iFlag);
static void collectFreeBuf(u8 *filterList[][3], u8 **freeList);
static u8 *filterBufList[MAX_QUEUE_NUM][3];
static u8 *freeBufList[MAX_QUEUE_NUM];
static atomic_t global_op_count = ATOMIC_INIT(0);
static unsigned int max_fb_size = 0;
static unsigned int fb_size_from_cmd = 0;

/* Compatibility mode global switch .....
 *
 * This is a secret switch for user space programs that may want to 
 * select color spaces and set overlay position through the nonstd 
 * element of fb_var_screeninfo This is the way legacy PXA display
 * drivers were used. The mode reverts back to normal mode on driver release. 
 *
 * To turn on compatibility with older PXA, set the MSB of nonstd to 0xAA.
 */

static unsigned int COMPAT_MODE;



static struct _sViewPortInfo gViewPortInfo = {
	.srcWidth = 640,	/* video source size */
	.srcHeight = 480,
	.zoomXSize = 640,	/* size after zooming */
	.zoomYSize = 480,
};

static struct _sViewPortOffset gViewPortOffset = {
	.xOffset = 0,	/* position on screen */
	.yOffset = 0
};

#ifdef FB_PM_DEBUG
static unsigned int g_regs[1024];
static unsigned int g_regs1[1024];
static unsigned int pxa168fb_rw_all_regs(struct pxa168fb_info *fbi,
		unsigned int *regs, int is_read)
{
	u32 i;
	u32 reg;

	for (i = 0xC0; i <= 0x01C4; i += 4) {
		if (is_read) {
			reg = readl(fbi->reg_base + i);
			regs[i] = reg;
		} else {
			writel(regs[i], fbi->reg_base + i);
		}
	}

	return 0;
}
#endif

static void pxa168fb_do_tasklet(unsigned long data)
{
	struct fb_info *fi;

	fi = (struct fb_info *)data;
	pxa168fb_switch_buff(fi);
}

static struct fb_videomode *
find_best_mode(struct pxa168fb_info *fbi, struct fb_var_screeninfo *var)
{
	struct pxa168fb_mach_info *mi = fbi->dev->platform_data;
	struct fb_videomode *best_mode;
	int i;

        dev_dbg(fbi->fb_info->dev, "Enter %s\n", __FUNCTION__);
	best_mode = NULL;
	for (i = 0; i < mi->num_modes; i++) {
		struct fb_videomode *m = mi->modes + i;

		/*
		 * Check whether this mode is suitable.
		 */
		if (var->xres > m->xres)
			continue;
		if (var->yres > m->yres)
			continue;

		/*
		 * Check whether this mode is more suitable than
		 * the best mode so far.
		 */
		if (best_mode != NULL &&
		    (best_mode->xres < m->xres ||
		     best_mode->yres < m->yres ||
		     best_mode->pixclock > m->pixclock))
			continue;

		best_mode = m;
	}

	return best_mode;
}

static int determine_best_pix_fmt(struct fb_var_screeninfo *var)
{
	unsigned char pxa_format;

        /* compatibility switch: if var->nonstd MSB is 0xAA then skip to 
         * using the nonstd variable to select the color space.
         */
        if(COMPAT_MODE != 0x2625) {        

                /*
                 * Pseudocolor mode?
                 */
                if (var->bits_per_pixel == 8)
                        return PIX_FMT_PSEUDOCOLOR;
                /*
                 * Check for YUV422PACK.
                 */
                if (var->bits_per_pixel == 16 && var->red.length == 16 &&
                    var->green.length == 16 && var->blue.length == 16) {
                        if (var->red.offset >= var->blue.offset) {
                                if (var->red.offset == 4)
                                        return PIX_FMT_YUV422PACK;
                                else
                                        return PIX_FMT_YUYV422PACK;
                        } else
                                return PIX_FMT_YVU422PACK;
                }
                /*
                 * Check for YUV422PLANAR.
                 */
                if (var->bits_per_pixel == 16 && var->red.length == 8 &&
                    var->green.length == 4 && var->blue.length == 4) {
                        if (var->red.offset >= var->blue.offset)
                                return PIX_FMT_YUV422PLANAR;
                        else
                                return PIX_FMT_YVU422PLANAR;
                }

                /*
                 * Check for YUV420PLANAR.
                 */
                if (var->bits_per_pixel == 12 && var->red.length == 8 &&
                    var->green.length == 2 && var->blue.length == 2) {
                        if (var->red.offset >= var->blue.offset)
                                return PIX_FMT_YUV420PLANAR;
                        else
                                return PIX_FMT_YVU420PLANAR;
                }
                /*
                 * Check for 565/1555.
                 */
                if (var->bits_per_pixel == 16 && var->red.length <= 5 &&
                    var->green.length <= 6 && var->blue.length <= 5) {
                        if (var->transp.length == 0) {
                                if (var->red.offset >= var->blue.offset)
                                        return PIX_FMT_RGB565;
                                else
                                        return PIX_FMT_BGR565;
                        }

                        if (var->transp.length == 1 && var->green.length <= 5) {
                                if (var->red.offset >= var->blue.offset)
                                        return PIX_FMT_RGB1555;
                                else
                                        return PIX_FMT_BGR1555;
                        }

                        /* fall through */
                }

                /*
                 * Check for 888/A888.
                 */
                if (var->bits_per_pixel <= 32 && var->red.length <= 8 &&
                    var->green.length <= 8 && var->blue.length <= 8) {
                        if (var->bits_per_pixel == 24 && var->transp.length == 0) {
                                if (var->red.offset >= var->blue.offset)
                                        return PIX_FMT_RGB888PACK;
                                else
                                        return PIX_FMT_BGR888PACK;
                        }

			if (var->bits_per_pixel == 32 && var->transp.offset == 24) {
				if (var->red.offset >= var->blue.offset)
					return PIX_FMT_RGBA888;
				else
					return PIX_FMT_BGRA888;
			} else {
				if (var->transp.length == 8) {
					if (var->red.offset >= var->blue.offset)
						return PIX_FMT_RGB888UNPACK;
					else
						return PIX_FMT_BGR888UNPACK;
				} else
					return PIX_FMT_YUV422PACK_IRE_90_270;

			}
                        /* fall through */
                }
        } else {

                pxa_format = (var->nonstd >> 20) & 0xf;
        
                switch (pxa_format) {
                case 0:
                        return PIX_FMT_RGB565;
                        break;
                case 3:
                        return PIX_FMT_YUV422PLANAR;
                        break;
                case 4:
                        return PIX_FMT_YUV420PLANAR;
                        break;
                case 5:
                        return PIX_FMT_RGB1555;
                        break;
                case 6:
                        return PIX_FMT_RGB888PACK;
                        break;
                case 7:
                        return PIX_FMT_RGB888UNPACK;
                        break;
                case 8:
                        return PIX_FMT_RGBA888;
                        break;
                case 9:
                        return PIX_FMT_YUV422PACK;
                        break;

                default:
                        return -EINVAL;
                }
        }



	return -EINVAL;
}

static void set_pix_fmt(struct fb_var_screeninfo *var, int pix_fmt)
{
	switch (pix_fmt) {
	case PIX_FMT_RGB565:
		var->bits_per_pixel = 16;
		var->red.offset = 11;    var->red.length = 5;
		var->green.offset = 5;   var->green.length = 6;
		var->blue.offset = 0;    var->blue.length = 5;
		var->transp.offset = 0;  var->transp.length = 0;
                var->nonstd &= ~0xff0fffff;
		break;
	case PIX_FMT_BGR565:
		var->bits_per_pixel = 16;
		var->red.offset = 0;     var->red.length = 5;
		var->green.offset = 5;   var->green.length = 6;
		var->blue.offset = 11;   var->blue.length = 5;
		var->transp.offset = 0;  var->transp.length = 0;
                var->nonstd &= ~0xff0fffff;
		break;
	case PIX_FMT_RGB1555:
		var->bits_per_pixel = 16;
		var->red.offset = 10;    var->red.length = 5;
		var->green.offset = 5;   var->green.length = 5;
		var->blue.offset = 0;    var->blue.length = 5;
		var->transp.offset = 15; var->transp.length = 1;
                var->nonstd &= ~0xff0fffff;
                var->nonstd |= 5 << 20;
		break;
	case PIX_FMT_BGR1555:
		var->bits_per_pixel = 16;
		var->red.offset = 0;     var->red.length = 5;
		var->green.offset = 5;   var->green.length = 5;
		var->blue.offset = 10;   var->blue.length = 5;
		var->transp.offset = 15; var->transp.length = 1;
		var->nonstd &= ~0xff0fffff;
                var->nonstd |= 5 << 20;
                break;
	case PIX_FMT_RGB888PACK:
		var->bits_per_pixel = 24;
		var->red.offset = 16;    var->red.length = 8;
		var->green.offset = 8;   var->green.length = 8;
		var->blue.offset = 0;    var->blue.length = 8;
		var->transp.offset = 0;  var->transp.length = 0;
		var->nonstd &= ~0xff0fffff;
                var->nonstd |= 6 << 20;
                break;
	case PIX_FMT_BGR888PACK:
		var->bits_per_pixel = 24;
		var->red.offset = 0;     var->red.length = 8;
		var->green.offset = 8;   var->green.length = 8;
		var->blue.offset = 16;   var->blue.length = 8;
		var->transp.offset = 0;  var->transp.length = 0;
		var->nonstd &= ~0xff0fffff;
                var->nonstd |= 6 << 20;
                break;
	case PIX_FMT_RGB888UNPACK:
		var->bits_per_pixel = 32;
		var->red.offset = 16;    var->red.length = 8;
		var->green.offset = 8;   var->green.length = 8;
		var->blue.offset = 0;    var->blue.length = 8;
		var->transp.offset = 0;  var->transp.length = 8;
		var->nonstd &= ~0xff0fffff;
                var->nonstd |= 7 << 20;
                break;
	case PIX_FMT_BGR888UNPACK:
		var->bits_per_pixel = 32;
		var->red.offset = 0;     var->red.length = 8;
		var->green.offset = 8;   var->green.length = 8;
		var->blue.offset = 16;   var->blue.length = 8;
		var->transp.offset = 0;  var->transp.length = 8;
		var->nonstd &= ~0xff0fffff;
                var->nonstd |= 7 << 20;
                break;
	case PIX_FMT_RGBA888:
		var->bits_per_pixel = 32;
		var->red.offset = 16;    var->red.length = 8;
		var->green.offset = 8;   var->green.length = 8;
		var->blue.offset = 0;    var->blue.length = 8;
		var->transp.offset = 24; var->transp.length = 8;
		var->nonstd &= ~0xff0fffff;
                var->nonstd |= 8 << 20;
                break;
	case PIX_FMT_BGRA888:
		var->bits_per_pixel = 32;
		var->red.offset = 0;     var->red.length = 8;
		var->green.offset = 8;   var->green.length = 8;
		var->blue.offset = 16;   var->blue.length = 8;
		var->transp.offset = 24; var->transp.length = 8;
		var->nonstd &= ~0xff0fffff;
                var->nonstd |= 8 << 20;
                break;
        case PIX_FMT_YUYV422PACK:
                var->bits_per_pixel = 16;
                var->red.offset = 8;     var->red.length = 16;
                var->green.offset = 4;   var->green.length = 16;
                var->blue.offset = 0;   var->blue.length = 16;
                var->transp.offset = 0;  var->transp.length = 0;
                var->nonstd &= ~0xff0fffff;
                var->nonstd |= 9 << 20;
                break;
        case PIX_FMT_YVU422PACK:
                var->bits_per_pixel = 16;
                var->red.offset = 0;     var->red.length = 16;
                var->green.offset = 8;   var->green.length = 16;
                var->blue.offset = 12;   var->blue.length = 16;
                var->transp.offset = 0;  var->transp.length = 0;
                var->nonstd &= ~0xff0fffff;
                var->nonstd |= 9 << 20;
                break;
	case PIX_FMT_YUV422PLANAR:
		var->bits_per_pixel = 16;
		var->red.offset = 8;	 var->red.length = 8;
		var->green.offset = 4;   var->green.length = 4;
		var->blue.offset = 0;   var->blue.length = 4;
		var->transp.offset = 0;  var->transp.length = 0;
                var->nonstd &= ~0xff0fffff;
                var->nonstd |= 3 << 20;
                break;
	case PIX_FMT_YVU422PLANAR:
		var->bits_per_pixel = 16;
		var->red.offset = 0;	 var->red.length = 8;
		var->green.offset = 8;   var->green.length = 4;
		var->blue.offset = 12;   var->blue.length = 4;
		var->transp.offset = 0;  var->transp.length = 0;
		var->nonstd &= ~0xff0fffff;
                var->nonstd |= 3 << 20;
                break;
	case PIX_FMT_YUV420PLANAR:
		var->bits_per_pixel = 12;
		var->red.offset = 4;	 var->red.length = 8;
		var->green.offset = 2;   var->green.length = 2;
		var->blue.offset = 0;   var->blue.length = 2;
		var->transp.offset = 0;  var->transp.length = 0;
		var->nonstd &= ~0xff0fffff;
                var->nonstd |= 4 << 20;
                break;
	case PIX_FMT_YVU420PLANAR:
		var->bits_per_pixel = 12;
		var->red.offset = 0;	 var->red.length = 8;
		var->green.offset = 8;   var->green.length = 2;
		var->blue.offset = 10;   var->blue.length = 2;
		var->transp.offset = 0;  var->transp.length = 0;
		var->nonstd &= ~0xff0fffff;
                var->nonstd |= 4 << 20;
                break;

	case PIX_FMT_PSEUDOCOLOR:
		var->bits_per_pixel = 8;
		var->red.offset = 0;     var->red.length = 8;
		var->green.offset = 0;   var->green.length = 8;
		var->blue.offset = 0;    var->blue.length = 8;
		var->transp.offset = 0;  var->transp.length = 0;
		break;
	case PIX_FMT_YUV422PACK:
		var->bits_per_pixel = 16;
		var->red.offset = 4;     var->red.length = 16;
		var->green.offset = 12;   var->green.length = 16;
		var->blue.offset = 0;    var->blue.length = 16;
		var->transp.offset = 0;  var->transp.length = 0;
		var->nonstd &= ~0xff0fffff;
                var->nonstd |= 9 << 20;
                break;
	case PIX_FMT_YUV422PACK_IRE_90_270:	/* YUV422 Packed will be YUV444 Packed after IRE 90 and 270 degree rotation*/
                var->bits_per_pixel = 32;
                var->red.offset = 16;    var->red.length = 8;
                var->green.offset = 8;   var->green.length = 8;
                var->blue.offset = 0;    var->blue.length = 8;
                var->transp.offset = 0;  var->transp.length = 0;
                var->nonstd &= ~0xff0fffff;
                var->nonstd |= 7 << 20;
		break;
	default:
		return  -EINVAL;
	}
}

static int convert_pix_fmt(u32 vmode)
{
/*	printk(KERN_INFO "vmode=%d\n", vmode); */
	switch(vmode) {
	case FB_VMODE_YUV422PACKED:
		return PIX_FMT_YUV422PACK;
	case FB_VMODE_YUV422PACKED_SWAPUV:
		return PIX_FMT_YVU422PACK;
	case FB_VMODE_YUV422PLANAR:
		return PIX_FMT_YUV422PLANAR;
	case FB_VMODE_YUV422PLANAR_SWAPUV:
		return PIX_FMT_YVU422PLANAR;
	case FB_VMODE_YUV420PLANAR:
		return PIX_FMT_YUV420PLANAR;
	case FB_VMODE_YUV420PLANAR_SWAPUV:
		return PIX_FMT_YVU420PLANAR;
	case FB_VMODE_YUV422PACKED_SWAPYUorV:
		return PIX_FMT_YUYV422PACK;
	case FB_VMODE_YUV422PACKED_IRE_90_270:
		return PIX_FMT_YUV422PACK_IRE_90_270;
	case FB_VMODE_RGB565:
		return PIX_FMT_RGB565;
	case FB_VMODE_BGR565:
		return PIX_FMT_BGR565;
	case FB_VMODE_RGB1555:
		return PIX_FMT_RGB1555;
	case FB_VMODE_BGR1555:
		return PIX_FMT_BGR1555;
	case FB_VMODE_RGB888PACK:
		return PIX_FMT_RGB888PACK;
	case FB_VMODE_BGR888PACK:
		return PIX_FMT_BGR888PACK;
	case FB_VMODE_RGBA888:
		return PIX_FMT_RGBA888;
	case FB_VMODE_BGRA888:
		return PIX_FMT_BGRA888;
	case FB_VMODE_RGB888UNPACK:
	case FB_VMODE_BGR888UNPACK:
	case FB_VMODE_YUV422PLANAR_SWAPYUorV:
	case FB_VMODE_YUV420PLANAR_SWAPYUorV:
	default:
		return -1;
	}
}

static void pxa168_sync_colorkey_structures(struct pxa168fb_info *fbi, int direction)
{
        struct _sColorKeyNAlpha *colorkey = &fbi->ckey_alpha;
        struct pxa168_fb_chroma *chroma = &fbi->chroma;
        unsigned int temp;
        
        dev_dbg(fbi->fb_info->dev, "ENTER %s\n", __FUNCTION__);
        if (direction == FB_SYNC_COLORKEY_TO_CHROMA) {
                chroma->mode        = colorkey->mode;
                chroma->y_alpha     = (colorkey->Y_ColorAlpha) & 0xff;
                chroma->y           = (colorkey->Y_ColorAlpha >> 8) & 0xff;
                chroma->y1          = (colorkey->Y_ColorAlpha >> 16) & 0xff;
                chroma->y2          = (colorkey->Y_ColorAlpha >> 24) & 0xff;

                chroma->u_alpha     = (colorkey->U_ColorAlpha) & 0xff;
                chroma->u           = (colorkey->U_ColorAlpha >> 8) & 0xff;
                chroma->u1          = (colorkey->U_ColorAlpha >> 16) & 0xff;
                chroma->u2          = (colorkey->U_ColorAlpha >> 24) & 0xff;

                chroma->v_alpha     = (colorkey->V_ColorAlpha) & 0xff;
                chroma->v           = (colorkey->V_ColorAlpha >> 8) & 0xff;
                chroma->v1          = (colorkey->V_ColorAlpha >> 16) & 0xff;
                chroma->v2          = (colorkey->V_ColorAlpha >> 24) & 0xff;
        }


        if (direction == FB_SYNC_CHROMA_TO_COLORKEY) {
       
                colorkey->mode = chroma->mode;
                temp = chroma->y_alpha;
                temp |= chroma->y << 8;
                temp |= chroma->y1 << 16;
                temp |= chroma->y2 << 24;
                colorkey->Y_ColorAlpha = temp;

                temp = chroma->u_alpha;
                temp |= chroma->u << 8;
                temp |= chroma->u1 << 16;
                temp |= chroma->u2 << 24;
                colorkey->U_ColorAlpha = temp;

                temp = chroma->v_alpha;
                temp |= chroma->v << 8;
                temp |= chroma->v1 << 16;
                temp |= chroma->v2 << 24;
                colorkey->V_ColorAlpha = temp;

        }
}

static u32 pxa168fb_ovly_set_colorkeyalpha(struct pxa168fb_info *fbi)
{
	unsigned int rb;
	unsigned int temp;
	unsigned int x;
	struct _sColorKeyNAlpha *color_a = &fbi->ckey_alpha;

        dev_dbg(fbi->fb_info->dev, "Enter %s\n", __FUNCTION__);
	/* reset to 0x0 to disable color key. */
	if (fbi->id <= 0)
		x = readl(fbi->reg_base + LCD_SPU_DMA_CTRL1) & ~(CFG_COLOR_KEY_MASK | CFG_ALPHA_MODE_MASK | CFG_ALPHA_MASK);
	else
		x = readl(fbi->reg_base + LCD_TV_CTRL1) & ~(CFG_COLOR_KEY_MASK | CFG_ALPHA_MODE_MASK | CFG_ALPHA_MASK);

	/* switch to color key mode */
	switch (color_a->mode) {
	case FB_DISABLE_COLORKEY_MODE:
		/* do nothing */
		break;
	case FB_ENABLE_Y_COLORKEY_MODE:
		x |= CFG_COLOR_KEY_MODE(0x1);
		break;
	case FB_ENABLE_U_COLORKEY_MODE:
		x |= CFG_COLOR_KEY_MODE(0x2);
		break;
	case FB_ENABLE_V_COLORKEY_MODE:
		x |= CFG_COLOR_KEY_MODE(0x4);
		break;
	case FB_ENABLE_RGB_COLORKEY_MODE:
		x |= CFG_COLOR_KEY_MODE(0x3);

		/* check whether h/w turn on RB swap. */
		if (fbi->id <= 0)
			rb = readl(fbi->reg_base + LCD_SPU_DMA_CTRL0);
		else
			rb = readl(fbi->reg_base + LCD_TV_CTRL0);
		if (rb & CFG_DMA_SWAPRB_MASK) {
			/* exchange r b fields. */
			temp = color_a->Y_ColorAlpha;
			color_a->Y_ColorAlpha = color_a->V_ColorAlpha;
			color_a->V_ColorAlpha = temp;
		}

		break;
	case FB_ENABLE_R_COLORKEY_MODE:
		x |= CFG_COLOR_KEY_MODE(0x5);
		break;
	case FB_ENABLE_G_COLORKEY_MODE:
		x |= CFG_COLOR_KEY_MODE(0x6);
	break;
	case FB_ENABLE_B_COLORKEY_MODE:
		x |= CFG_COLOR_KEY_MODE(0x7);
		break;
	default:
		printk(KERN_INFO "unknown mode");
		return -1;
	}

	/* switch to alpha path selection */
	switch (color_a->alphapath) {
	case FB_VID_PATH_ALPHA:
		x |= CFG_ALPHA_MODE(0x0);
		break;
	case FB_GRA_PATH_ALPHA:
		x |= CFG_ALPHA_MODE(0x1);
		break;
	case FB_CONFIG_ALPHA:
		x |= CFG_ALPHA_MODE(0x2);
		break;
	default:
		printk(KERN_INFO "unknown alpha path");
		return -1;
	}

	/* configure alpha */
	x |= CFG_ALPHA((color_a->config & 0xff));
	if (fbi->id <= 0) {
		writel(x, fbi->reg_base + LCD_SPU_DMA_CTRL1);
		writel(color_a->Y_ColorAlpha, fbi->reg_base + LCD_SPU_COLORKEY_Y);
		writel(color_a->U_ColorAlpha, fbi->reg_base + LCD_SPU_COLORKEY_U);
		writel(color_a->V_ColorAlpha, fbi->reg_base + LCD_SPU_COLORKEY_V);
	} else {
		writel(x, fbi->reg_base + LCD_TV_CTRL1);
		writel(color_a->Y_ColorAlpha, fbi->reg_base + LCD_TV_COLORKEY_Y);
		writel(color_a->U_ColorAlpha, fbi->reg_base + LCD_TV_COLORKEY_U);
		writel(color_a->V_ColorAlpha, fbi->reg_base + LCD_TV_COLORKEY_V);
	}
	return 0;
}

static int check_surface(struct fb_info *fi,
			FBVideoMode new_mode,
			struct _sViewPortInfo *new_info,
			struct _sViewPortOffset *new_offset,
			struct _sVideoBufferAddr *new_addr)
{
	struct pxa168fb_info *fbi = (struct pxa168fb_info *)fi->par;
	struct fb_var_screeninfo *var = &fi->var;
	int changed = 0;
	
        dev_dbg(fi->dev, "Enter %s\n", __FUNCTION__);
        
        /*
	 * check mode
	 */
	if (new_mode >= 0 && fbi->surface.videoMode != new_mode) {
		fbi->surface.videoMode = new_mode;
		fbi->pix_fmt = convert_pix_fmt(new_mode);
		set_pix_fmt(var, fbi->pix_fmt);
		changed = 1;
	}
	/*
	 * check view port settings.
	 */
	if (new_info &&
            (fbi->surface.viewPortInfo.srcWidth != new_info->srcWidth ||
                 fbi->surface.viewPortInfo.srcHeight != new_info->srcHeight ||
                 fbi->surface.viewPortInfo.zoomXSize != new_info->zoomXSize ||
             fbi->surface.viewPortInfo.zoomYSize != new_info->zoomYSize ||
		fbi->surface.viewPortInfo.yPitch != new_info->yPitch ||
		fbi->surface.viewPortInfo.uPitch != new_info->uPitch ||
		fbi->surface.viewPortInfo.vPitch != new_info->vPitch)) {
                if (!(new_addr && new_addr->startAddr[0])) {
			if (((new_info->srcWidth * new_info->srcHeight * var->bits_per_pixel / 8) * 2) > max_fb_size) {
                                printk("%s: requested memory buffer size %d exceed the max limit %d!\n", __func__,
					(new_info->srcWidth * new_info->srcHeight * var->bits_per_pixel / 4), max_fb_size);
                                return changed;
                        }
                }
                var->xres_virtual = new_info->srcWidth;
                var->yres_virtual = new_info->srcHeight * 2;
                var->xres = new_info->srcWidth;
                var->yres = new_info->srcHeight;
                fbi->surface.viewPortInfo = *new_info;
                set_pix_fmt(var, fbi->pix_fmt);
                changed = 1;
        }

	/*
	 * Check offset
	 */
	if (new_offset &&
	    (fbi->surface.viewPortOffset.xOffset != new_offset->xOffset ||
	    fbi->surface.viewPortOffset.yOffset != new_offset->yOffset)) {
		fbi->surface.viewPortOffset.xOffset = new_offset->xOffset;
		fbi->surface.viewPortOffset.yOffset = new_offset->yOffset;
		changed = 1;
	}
	/*
	 * Check buffer address
	 */
	if (new_addr && new_addr->startAddr[0] &&
	    fbi->new_addr[0] != (unsigned long)new_addr->startAddr[0]) {
		fbi->new_addr[0] = (unsigned long)new_addr->startAddr[0];
		fbi->new_addr[1] = (unsigned long)new_addr->startAddr[1];
		fbi->new_addr[2] = (unsigned long)new_addr->startAddr[2];
		changed = 1;
	}

	return changed;
}

static void pxa168fb_clear_framebuffer(struct fb_info *fi)
{
        struct pxa168fb_info *fbi = fi->par;

        memset(fbi->fb_start, 0, fbi->fb_size);
}



#ifdef CONFIG_DYNAMIC_PRINTK_DEBUG
static void debug_identify_called_ioctl(struct fb_info *fi, int cmd, unsigned long arg)
{
        switch (cmd) {
        case FBIO_CLEAR_FRAMEBUFFER:
                dev_dbg(fi->dev," FBIO_CLEAR_FRAMEBUFFER\n");
                break;
        case FB_IOCTL_WAIT_VSYNC:
                dev_dbg(fi->dev," FB_IOCTL_WAIT_VSYNC\n");
                break;
        case FB_IOCTL_GET_VIEWPORT_INFO:
                dev_dbg(fi->dev," FB_IOCTL_GET_VIEWPORT_INFO with arg = %08x\n", (unsigned int)arg);
                break;
        case FB_IOCTL_SET_VIEWPORT_INFO:
                dev_dbg(fi->dev," FB_IOCTL_SET_VIEWPORT_INFO with arg = %08x\n", (unsigned int)arg);
                break;
        case FB_IOCTL_SET_VIDEO_MODE:
                dev_dbg(fi->dev," FB_IOCTL_SET_VIDEO_MODE with arg = %08x\n", (unsigned int)arg);
                break;
        case FB_IOCTL_GET_VIDEO_MODE:
                dev_dbg(fi->dev," FB_IOCTL_GET_VIDEO_MODE with arg = %08x\n", (unsigned int)arg);
                break;
        case FB_IOCTL_FLIP_VID_BUFFER:
                dev_dbg(fi->dev," FB_IOCTL_FLIP_VID_BUFFER with arg = %08x\n", (unsigned int)arg);
                break;
        case FB_IOCTL_GET_FREELIST:
                dev_dbg(fi->dev," FB_IOCTL_GET_FREELIST with arg = %08x\n", (unsigned int)arg);
                break;
        case FB_IOCTL_GET_BUFF_ADDR:
                dev_dbg(fi->dev," FB_IOCTL_GET_BUFF_ADDR with arg = %08x\n", (unsigned int)arg);
                break;
        case FB_IOCTL_SET_VID_OFFSET:
                dev_dbg(fi->dev," FB_IOCTL_SET_VID_OFFSET with arg = %08x\n", (unsigned int)arg);
                break;
        case FB_IOCTL_GET_VID_OFFSET:
                dev_dbg(fi->dev," FB_IOCTL_GET_VID_OFFSET with arg = %08x\n", (unsigned int)arg);
                break;
        case FB_IOCTL_SET_MEMORY_TOGGLE:
                dev_dbg(fi->dev," FB_IOCTL_SET_MEMORY_TOGGLE with arg = %08x\n",(unsigned int) arg);
                break;
        case FB_IOCTL_SET_COLORKEYnALPHA:
                dev_dbg(fi->dev," FB_IOCTL_SET_COLORKEYnALPHA with arg = %08x\n", (unsigned int)arg);
                break;
        case FBIOGET_CHROMAKEYS:
                dev_dbg(fi->dev," FBIOGET_CHROMAKEYS with arg = %08x\n", (unsigned int)arg);
                break;
        case FBIOPUT_CHROMAKEYS:
                dev_dbg(fi->dev," FBIOPUT_CHROMAKEYS with arg = %08x\n", (unsigned int)arg);
                break;
        case FB_IOCTL_GET_COLORKEYnALPHA:
                dev_dbg(fi->dev," FB_IOCTL_GET_COLORKEYnALPHA with arg = %08x\n", (unsigned int)arg);
                break;
        case FB_IOCTL_SWITCH_VID_OVLY:
                dev_dbg(fi->dev," FB_IOCTL_SWITCH_VID_OVLY with arg = %08x\n", (unsigned int)arg);
                break;
        case FB_IOCTL_SWITCH_GRA_OVLY:
                dev_dbg(fi->dev," FB_IOCTL_SWITCH_GRA_OVLY with arg = %08x\n", (unsigned int)arg);
                break;
        case FBIOPUT_SWAP_VIDEO_RED_BLUE:
                dev_dbg(fi->dev," FBIOPUT_SWAP_VIDEO_RED_BLUE with arg = %08x\n", (unsigned int)arg);
                break;
        case FBIOPUT_SWAP_VIDEO_U_V:
                dev_dbg(fi->dev," FBIOPUT_SWAP_VIDEO_U_V with arg = %08x\n", (unsigned int)arg);
                break;
        case FBIOPUT_SWAP_VIDEO_Y_UV:
                dev_dbg(fi->dev," FBIOPUT_SWAP_VIDEO_Y_UV with arg = %08x\n", (unsigned int)arg);
                break;
        case FBIOPUT_VIDEO_ALPHABLEND:
                dev_dbg(fi->dev," FBIOPUT_VIDEO_ALPHABLEND with arg = %08x\n", (unsigned int)arg);
                break;
        case FBIOPUT_GLOBAL_ALPHABLEND:
                dev_dbg(fi->dev," FBIOPUT_GLOBAL_ALPHABLEND with arg = %08x\n", (unsigned int)arg);
                break;
        case FBIOPUT_GRAPHIC_ALPHABLEND:
                dev_dbg(fi->dev," FBIOPUT_GRAPHIC_ALPHABLEND with arg = %08x\n", (unsigned int)arg);
                break;

        }
}
#endif

static int pxa168fb_ioctl(struct fb_info *fi, unsigned int cmd,
			unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct pxa168fb_info *fbi = (struct pxa168fb_info *)fi->par;
	u32 x;
	int vmode = 0;
	int gfx_on = 1;
	int vid_on = 1;
        int val;
        unsigned char param;
        int blendval;

#ifdef CONFIG_DYNAMIC_PRINTK_DEBUG
        debug_identify_called_ioctl(fi, cmd, arg);
#endif         

	switch (cmd) {
        case FBIO_CLEAR_FRAMEBUFFER:
                pxa168fb_clear_framebuffer(fi);
                return 0;
                break;
	case FB_IOCTL_WAIT_VSYNC:
		wait_for_vsync(fbi);
		break;
	case FB_IOCTL_GET_VIEWPORT_INFO:
		return copy_to_user(argp, &fbi->surface.viewPortInfo,
			sizeof(struct _sViewPortInfo)) ? -EFAULT : 0;
	case FB_IOCTL_SET_VIEWPORT_INFO:
		mutex_lock(&fbi->access_ok);
		if (copy_from_user(&gViewPortInfo, argp,
				sizeof(gViewPortInfo))) {
			mutex_unlock(&fbi->access_ok);
			return -EFAULT;
		}

		if (check_surface(fi, -1, &gViewPortInfo, 0, 0))
			pxa168fb_set_par(fi);

		mutex_unlock(&fbi->access_ok);
		break;
	case FB_IOCTL_SET_VIDEO_MODE:
		/*
		 * Get data from user space.
		 */
		if (copy_from_user(&vmode, argp, sizeof(vmode)))
			return -EFAULT;

		if (check_surface(fi, vmode, 0, 0, 0))
			pxa168fb_set_par(fi);
		break;
	case FB_IOCTL_GET_VIDEO_MODE:
		return copy_to_user(argp, &fbi->surface.videoMode,
			sizeof(u32)) ? -EFAULT : 0;
	case FB_IOCTL_FLIP_VID_BUFFER:
	{
		struct _sOvlySurface *surface = 0;
		u8 *start_addr[3], *input_data;
		u32 length;
		surface = kmalloc(sizeof(struct _sOvlySurface),
				GFP_KERNEL);

		/* Get user-mode data. */
		if (copy_from_user(surface, argp,
					sizeof(struct _sOvlySurface))) {
			kfree(surface);
			return -EFAULT;
		}

		length = surface->videoBufferAddr.length;
		start_addr[0] = surface->videoBufferAddr.startAddr[0];
		input_data = surface->videoBufferAddr.inputData;

		/*
		 * Has DMA addr?
		 */
		if (start_addr[0] &&
				(!input_data)) {
			if (0 != addFreeBuf(freeBufList, (u8 *)surface)) {
				printk(KERN_INFO "Error: addFreeBuf()\n");
				mutex_unlock(&fbi->access_ok);
				kfree(surface);
				return -EFAULT;
			}
			tasklet_schedule(&fbi->tasklet);
		} else {
			if (check_surface(fi, surface->videoMode,
						&surface->viewPortInfo,
						&surface->viewPortOffset,
						&surface->videoBufferAddr))
				pxa168fb_set_par(fi);

			/* copy buffer */
			if (input_data) {
				wait_for_vsync(fbi);
				/* if support hw DMA, replace this. */
				if (copy_from_user(fbi->fb_start,
							input_data, length))
					return -EFAULT;
				return 0;
			}

			/*
			 * if it has its own physical address,
			 * switch to this memory. don't support YUV planar format 
			 * with split YUV buffers. but below code seems have no 
			 * chancee to execute. - FIXME
			 */
			if (start_addr[0]) {
				if (fbi->mem_status)
					free_pages(
							(unsigned long)fbi->fb_start,
							get_order(fbi->fb_size));
				else
					dma_free_writecombine(fbi->dev,
							fbi->fb_size,
							fbi->fb_start,
							fbi->fb_start_dma);

				fbi->fb_start = __va(start_addr[0]);
				fbi->fb_size = length;
				fbi->fb_start_dma =
					(dma_addr_t)__pa(fbi->fb_start);
				fbi->mem_status = 1;
				fi->fix.smem_start = fbi->fb_start_dma;
				fi->fix.smem_len = fbi->fb_size;
				fi->screen_base = fbi->fb_start;
				fi->screen_size = fbi->fb_size;
			}

			kfree(surface);
		}

		mutex_unlock(&fbi->access_ok);
		return 0;
	}
	case FB_IOCTL_GET_FREELIST:
	{
		mutex_lock(&fbi->access_ok);

		/* Collect expired frame to list */
		collectFreeBuf(filterBufList, freeBufList);
        
		if (copy_to_user(argp, filterBufList,
					3*MAX_QUEUE_NUM*sizeof(u8 *))) {
			mutex_unlock(&fbi->access_ok);
			return -EFAULT;
		}

		clearFilterBuf(filterBufList, RESET_BUF);

		mutex_unlock(&fbi->access_ok);
		return 0;
	}
	case FB_IOCTL_GET_BUFF_ADDR:
	{
		return copy_to_user(argp, &fbi->surface.videoBufferAddr,
			sizeof(struct _sVideoBufferAddr)) ? -EFAULT : 0;
	}
	case FB_IOCTL_SET_VID_OFFSET:
		mutex_lock(&fbi->access_ok);
		if (copy_from_user(&gViewPortOffset, argp,
				sizeof(gViewPortOffset))) {
			mutex_unlock(&fbi->access_ok);
			return -EFAULT;
		}

		if (check_surface(fi, -1, 0, &gViewPortOffset, 0))
			pxa168fb_set_par(fi);
		mutex_unlock(&fbi->access_ok);
		break;
	case FB_IOCTL_GET_VID_OFFSET:
		return copy_to_user(argp, &fbi->surface.viewPortOffset,
			sizeof(struct _sViewPortOffset)) ? -EFAULT : 0;
	case FB_IOCTL_SET_MEMORY_TOGGLE:
		break;

	case FB_IOCTL_SET_COLORKEYnALPHA:
		if (copy_from_user(&fbi->ckey_alpha, argp,
		    sizeof(struct _sColorKeyNAlpha)))
			return -EFAULT;

		pxa168fb_ovly_set_colorkeyalpha(fbi);
		break;
        case FBIOGET_CHROMAKEYS:
                pxa168_sync_colorkey_structures(fbi, FB_SYNC_COLORKEY_TO_CHROMA);
                return copy_to_user(argp, &fbi->chroma, sizeof(struct pxa168_fb_chroma)) ? -EFAULT : 0;
        case FBIOPUT_CHROMAKEYS:
                if (copy_from_user(&fbi->chroma, argp, sizeof(struct pxa168_fb_chroma)))
                        return -EFAULT;
                pxa168_sync_colorkey_structures(fbi, FB_SYNC_CHROMA_TO_COLORKEY);
                pxa168fb_ovly_set_colorkeyalpha(fbi);
                return copy_to_user(argp, &fbi->chroma, sizeof(struct pxa168_fb_chroma)) ? -EFAULT : 0;

	case FB_IOCTL_GET_COLORKEYnALPHA:
		if (copy_to_user(argp, &fbi->ckey_alpha,
		    sizeof(struct _sColorKeyNAlpha)))
			return -EFAULT;
		break;
	case FB_IOCTL_SWITCH_VID_OVLY:
		if (copy_from_user(&vid_on, argp, sizeof(int)))
			return -EFAULT;
		if (fbi->id <= 0) {
			if (0 == vid_on) {
				x = readl(fbi->reg_base + LCD_SPU_DMA_CTRL0) &
					~CFG_DMA_ENA_MASK;
				writel(x, fbi->reg_base + LCD_SPU_DMA_CTRL0);
			} else {
				x = readl(fbi->reg_base + LCD_SPU_DMA_CTRL0) |
					CFG_DMA_ENA(0x1);
				writel(x, fbi->reg_base + LCD_SPU_DMA_CTRL0);
			}
		} else {
			if (0 == vid_on) {
				x = readl(fbi->reg_base + LCD_TV_CTRL0) &
					~CFG_DMA_ENA_MASK;
				writel(x, fbi->reg_base + LCD_TV_CTRL0);
			} else {
				x = readl(fbi->reg_base + LCD_TV_CTRL0) |
					CFG_DMA_ENA(0x1);
				writel(x, fbi->reg_base + LCD_TV_CTRL0);
			}
		}
		break;
        case FB_IOCTL_SWITCH_GRA_OVLY:
		if (copy_from_user(&gfx_on, argp, sizeof(int))){
			return -EFAULT;
		}
		if (fbi->id <= 0) {
			if (0 == gfx_on) {
				x = readl(fbi->reg_base + LCD_SPU_DMA_CTRL0) &
					~CFG_GRA_ENA_MASK;
				writel(x, fbi->reg_base + LCD_SPU_DMA_CTRL0);
			} else {
				x = readl(fbi->reg_base + LCD_SPU_DMA_CTRL0) |
					CFG_GRA_ENA(0x1);
				writel(x, fbi->reg_base + LCD_SPU_DMA_CTRL0);
			}
		} else {
			if (0 == gfx_on) {
				x = readl(fbi->reg_base + LCD_TV_CTRL0) &
					~CFG_GRA_ENA_MASK;
				writel(x, fbi->reg_base + LCD_TV_CTRL0);
			} else {
				x = readl(fbi->reg_base + LCD_TV_CTRL0) |
					CFG_GRA_ENA(0x1);
				writel(x, fbi->reg_base + LCD_TV_CTRL0);
			}
		}
		break;
        case FBIOPUT_SWAP_VIDEO_RED_BLUE:
                param = (arg & 0x1);
		if (fbi->id <= 0) {
			val = readl(fbi->reg_base + LCD_SPU_DMA_CTRL0);
			val &= ~CFG_DMA_SWAPRB_MASK;
			val |= CFG_DMA_SWAPRB(param);
			writel(val, fbi->reg_base + LCD_SPU_DMA_CTRL0);
		} else {
			val = readl(fbi->reg_base + LCD_TV_CTRL0);
			val &= ~CFG_DMA_SWAPRB_MASK;
			val |= CFG_DMA_SWAPRB(param);
			writel(val, fbi->reg_base + LCD_TV_CTRL0);
		}
                return 0;
                break;

        case FBIOPUT_SWAP_VIDEO_U_V:
                param = (arg & 0x1);
		if (fbi->id <= 0) {
			val = readl(fbi->reg_base + LCD_SPU_DMA_CTRL0);
			val &= ~CFG_DMA_SWAPUV_MASK;
			val |= CFG_DMA_SWAPUV(param);
			writel(val, fbi->reg_base + LCD_SPU_DMA_CTRL0);
		} else {
			val = readl(fbi->reg_base + LCD_TV_CTRL0);
			val &= ~CFG_DMA_SWAPUV_MASK;
			val |= CFG_DMA_SWAPUV(param);
			writel(val, fbi->reg_base + LCD_TV_CTRL0);
		}
                return 0;
                break;

        case FBIOPUT_SWAP_VIDEO_Y_UV:
                param = (arg & 0x1);
		if (fbi->id <= 0) {
			val = readl(fbi->reg_base + LCD_SPU_DMA_CTRL0);
			val &= ~CFG_DMA_SWAPYU_MASK;
			val |= CFG_DMA_SWAPYU(param);
			writel(val, fbi->reg_base + LCD_SPU_DMA_CTRL0);
		} else {
			val = readl(fbi->reg_base + LCD_TV_CTRL0);
			val &= ~CFG_DMA_SWAPYU_MASK;
			val |= CFG_DMA_SWAPYU(param);
			writel(val, fbi->reg_base + LCD_TV_CTRL0);
		}
                return 0;
                break;

        case FBIOPUT_VIDEO_ALPHABLEND:
                /*
                 *  This puts the blending control to the Video layer.
                 */
		if (fbi->id <= 0) {
			val = readl(fbi->reg_base + LCD_SPU_DMA_CTRL1);
			val &= ~CFG_ALPHA_MODE_MASK;
			val &= ~CFG_ALPHA_MASK;
			val |= CFG_ALPHA_MODE(0);
			val |= CFG_ALPHA(0xff);
			writel(val, fbi->reg_base + LCD_SPU_DMA_CTRL1);
		} else {
			val = readl(fbi->reg_base + LCD_TV_CTRL1);
			val &= ~CFG_ALPHA_MODE_MASK;
			val &= ~CFG_ALPHA_MASK;
			val |= CFG_ALPHA_MODE(0);
			val |= CFG_ALPHA(0xff);
			writel(val, fbi->reg_base + LCD_TV_CTRL1);
		}
                return 0;
                break;

        case FBIOPUT_GLOBAL_ALPHABLEND:
                /*
                 *  The userspace application can specify a byte value for the amount of global blend 
                 *  between the video layer and the graphic layer. 
                 *
                 *  The alpha blending is per the formula below: 
                 *  P = (V[P] * blendval/255) + (G[P] * (1 - blendval/255))
                 *
                 *     where: P = Pixel value, V = Video Layer, and G = Graphic Layer
                 */
		if (fbi->id <= 0) {
			blendval = (arg & 0xff);
			val = readl(fbi->reg_base + LCD_SPU_DMA_CTRL1);
			val &= ~CFG_ALPHA_MODE_MASK;
			val &= ~CFG_ALPHA_MASK;
			val |= CFG_ALPHA_MODE(2);
			val |= CFG_ALPHA(blendval);
			writel(val, fbi->reg_base + LCD_SPU_DMA_CTRL1);
		} else {
			blendval = (arg & 0xff);
			val = readl(fbi->reg_base + LCD_TV_CTRL1);
			val &= ~CFG_ALPHA_MODE_MASK;
			val &= ~CFG_ALPHA_MASK;
			val |= CFG_ALPHA_MODE(2);
			val |= CFG_ALPHA(blendval);
			writel(val, fbi->reg_base + LCD_TV_CTRL1);
		}
                return 0;
                break;

        case FBIOPUT_GRAPHIC_ALPHABLEND:
                /*
                 *  This puts the blending back to the default mode of allowing the 
                 *  graphic layer to do pixel level blending. 
                 */
		if (fbi->id <= 0) {
			val = readl(fbi->reg_base + LCD_SPU_DMA_CTRL1);
			val &= ~CFG_ALPHA_MODE_MASK;
			val &= ~CFG_ALPHA_MASK;
			val |= CFG_ALPHA_MODE(1);
			val |= CFG_ALPHA(0x0);
			writel(val, fbi->reg_base + LCD_SPU_DMA_CTRL1);
		} else {
			val = readl(fbi->reg_base + LCD_TV_CTRL1);
			val &= ~CFG_ALPHA_MODE_MASK;
			val &= ~CFG_ALPHA_MASK;
			val |= CFG_ALPHA_MODE(1);
			val |= CFG_ALPHA(0x0);
			writel(val, fbi->reg_base + LCD_TV_CTRL1);
		}
                return 0;
                break;


	default:
		break;
	}

	return 0;
}

static void collectFreeBuf(u8 *filterList[][3], u8 **freeList)
{
	int i = 0, j = 0, count = 0;
	struct _sOvlySurface *srf = 0;
	u8 *ptr;

	/* Find the latest frame */
    if(freeList) {        
        for (i = (MAX_QUEUE_NUM-1); i >= 0; i--) {
        	if (freeList[i]) {
                count = i;
        		break;
        	}
        }
    }
    
	if ((count < 1) || !filterList || !freeList)
		return;

	for (i = 0, j = 0; i < count; i++) {

		ptr = freeList[i];

		/* Check freeList's content. */
		if (ptr) {
			for (; j < MAX_QUEUE_NUM; j++) {
				if (!(filterList[j][0])) {
					/* get surface's ptr. */
					srf = (struct _sOvlySurface *)ptr;

					/*
					 * save ptrs which need
					 * to be freed.
					 */
					filterList[j][0] =	srf->videoBufferAddr.startAddr[0];
                    filterList[j][1] =  srf->videoBufferAddr.startAddr[1];
                    filterList[j][2] =  srf->videoBufferAddr.startAddr[2];
					break;
				}
			}

			if (j >= MAX_QUEUE_NUM)
				break;

			kfree(freeList[i]);
			freeList[i] = 0;
		}
		else
		{
			/* till content is null. */
			break;
		}
	}

	freeList[0] = freeList[count];
	freeList[count] = 0;
}

static int addFreeBuf(u8 **ppBufList, u8 *pFreeBuf)
{
	int i = 0 ;
	struct _sOvlySurface *srf0 = (struct _sOvlySurface *)(pFreeBuf);
	struct _sOvlySurface *srf1 = 0;

	/* Error checking */
	if (!srf0) return -1;

	for (; i < MAX_QUEUE_NUM; i++) {
		srf1 = (struct _sOvlySurface *)ppBufList[i];

		if (!srf1)
		{
			/* printk(KERN_INFO "Set pFreeBuf "
			   "into %d entry.\n", i);
			 */
			ppBufList[i] = pFreeBuf;
			return 0;
		}

		if (srf1->videoBufferAddr.startAddr[0] ==
				srf0->videoBufferAddr.startAddr[0]) {
			/* same address, free this request. */
			kfree(pFreeBuf);
			return 0;
		}
	}


	if (i >= MAX_QUEUE_NUM)
		printk(KERN_INFO "buffer overflow\n");

	return -3;
}

static void clearFilterBuf(u8 *ppBufList[][3], int iFlag)
{
        /* Check null pointer. */
        if (!ppBufList)
                return;
        if (RESET_BUF & iFlag)
                memset(ppBufList, 0, 3 * MAX_QUEUE_NUM * sizeof(u8 *));
}

static void clearFreeBuf(u8 **ppBufList, int iFlag)
{
	int i = 0;

	/* Check null pointer. */
	if (!ppBufList)
		return;

	/* free */
	if (FREE_ENTRY & iFlag) {
		for (i = 0; i < MAX_QUEUE_NUM; i++) {
			if (ppBufList && ppBufList[i])
				kfree(ppBufList[i]);
		}
	}

	if (RESET_BUF & iFlag)
		memset(ppBufList, 0, MAX_QUEUE_NUM * sizeof(u8 *));
}

static int pxa168fb_open(struct fb_info *fi, int user)
{
	u32 val;
        struct pxa168fb_mach_info *mi;
	struct pxa168fb_info *fbi = (struct pxa168fb_info *)fi->par;
	struct fb_var_screeninfo *var = &fi->var;

        dev_dbg(fi->dev, "Enter %s\n", __FUNCTION__);

	set_dvfm_constraint();

	atomic_inc(&global_op_count);
        mi = fbi->dev->platform_data;
	fbi->new_addr[0] = 0;
	fbi->new_addr[1] = 0;
	fbi->new_addr[2] = 0;
	fi->fix.smem_start = fbi->fb_start_dma;
	fi->screen_base = fbi->fb_start;
	fbi->surface.videoMode = -1;
	fbi->surface.viewPortInfo.srcWidth = var->xres;
	fbi->surface.viewPortInfo.srcHeight = var->yres;
	fbi->active = 1;
	set_pix_fmt(var, fbi->pix_fmt);
	pxa168fb_set_par(fi);

	if (mutex_is_locked(&fbi->access_ok))
		mutex_unlock(&fbi->access_ok);

	/* clear buffer list. */
	mutex_lock(&fbi->access_ok);
	clearFilterBuf(filterBufList, RESET_BUF);
	clearFreeBuf(freeBufList, RESET_BUF|FREE_ENTRY);
	mutex_unlock(&fbi->access_ok);

	/* Compatibility with older PXA behavior. Force Video DMA engine on during the OPEN. */
	if (fbi->id <= 0) {
		val = readl(fbi->reg_base + LCD_SPU_DMA_CTRL0);
		val |= CFG_DMA_ENA_MASK;
		writel(val, fbi->reg_base + LCD_SPU_DMA_CTRL0);
	} else {
		val = readl(fbi->reg_base + LCD_TV_CTRL0);
		val |= CFG_DMA_ENA_MASK;
		writel(val, fbi->reg_base + LCD_TV_CTRL0);
	}
        return 0;
}

static int pxa168fb_release(struct fb_info *fi, int user)
{
        u32 val;
        struct pxa168fb_info *fbi = (struct pxa168fb_info *)fi->par;
        struct fb_var_screeninfo *var = &fi->var;

        dev_dbg(fi->dev, "Enter %s\n", __FUNCTION__);
        /* clear buffer list. */
	mutex_lock(&fbi->access_ok);
	clearFilterBuf(filterBufList, RESET_BUF);
	clearFreeBuf(freeBufList, RESET_BUF|FREE_ENTRY);
	mutex_unlock(&fbi->access_ok);

        /* Compatibility with older PXA behavior. Force Video DMA engine off at RELEASE.*/

	if(atomic_dec_and_test(&global_op_count)) {
		if (fbi->id <= 0) {
			val = readl(fbi->reg_base + LCD_SPU_DMA_CTRL0);
			val &= ~(CFG_DMA_ENA_MASK);
			writel(val, fbi->reg_base + LCD_SPU_DMA_CTRL0);
		} else {
			val = readl(fbi->reg_base + LCD_TV_CTRL0);
			val &= ~(CFG_DMA_ENA_MASK);
			writel(val, fbi->reg_base + LCD_TV_CTRL0);
		}
	}
	fbi->active = 0;

        /* Turn off compatibility mode */

        var->nonstd &= ~0xff000000;
        COMPAT_MODE = 0;

	unset_dvfm_constraint();

	return 0;
}

static int pxa168fb_switch_buff(struct fb_info *fi)
{
	struct pxa168fb_info *fbi = (struct pxa168fb_info *)fi->par;
	int i = 0;
	struct _sOvlySurface *pOvlySurface = 0;

	/*
	 * Find the latest frame.
	 */
	for (i = (MAX_QUEUE_NUM-1); i >= 0; i--) {
		if (freeBufList[i]) {
			pOvlySurface = (struct _sOvlySurface *)freeBufList[i];
			break;
		}
	}

	/*
	 * Got new frame?
	 */
	if (pOvlySurface && (fbi->new_addr[0] !=
				(unsigned long)pOvlySurface->videoBufferAddr.startAddr[0])) {
		/*
		 * Update new surface.
		 */
		if (check_surface(fi, pOvlySurface->videoMode,
					&pOvlySurface->viewPortInfo,
					&pOvlySurface->viewPortOffset,
					&pOvlySurface->videoBufferAddr))
			pxa168fb_set_par(fi);
	} else {
		/*printk(KERN_INFO "********Oops: pOvlySurface"
		  " is NULL!!!!\n\n");
		 */
		return -1;
	}

	return 0;
}

static void set_mode(struct pxa168fb_info *fbi, struct fb_var_screeninfo *var,
		     struct fb_videomode *mode, int pix_fmt, int ystretch)
{
	dev_dbg(fbi->fb_info->dev, "Enter %s\n", __FUNCTION__);
        set_pix_fmt(var, pix_fmt);

	var->xres = mode->xres;
	var->yres = mode->yres;
	var->xres_virtual = max(var->xres, var->xres_virtual);
	if (ystretch)
		var->yres_virtual = var->yres*2;
	else
		var->yres_virtual = max(var->yres, var->yres_virtual);

	var->grayscale = 0;
	var->accel_flags = FB_ACCEL_NONE;
	var->pixclock = mode->pixclock;
	var->left_margin = mode->left_margin;
	var->right_margin = mode->right_margin;
	var->upper_margin = mode->upper_margin;
	var->lower_margin = mode->lower_margin;
	var->hsync_len = mode->hsync_len;
	var->vsync_len = mode->vsync_len;
	var->sync = mode->sync;
	var->vmode = FB_VMODE_NONINTERLACED;
	var->rotate = FB_ROTATE_UR;
}

static int pxa168fb_check_var(struct fb_var_screeninfo *var, struct fb_info *fi)
{
	int pix_fmt;

	dev_dbg(fi->dev, "Enter %s\n", __FUNCTION__);
        if (var->bits_per_pixel == 8) {
                printk("bits per pixel too small\n");
		return -EINVAL;
        }

        /* compatibility mode: if the MSB of var->nonstd is 0xAA then 
         * set xres_virtual and yres_virtual to xres and yres.
         */

        if((var->nonstd >> 24) == 0xAA) 
                COMPAT_MODE = 0x2625;

        if((var->nonstd >> 24) == 0x55)
                COMPAT_MODE = 0x0;

	/*
	 * Basic geometry sanity checks.
	 */

	if (var->xoffset + var->xres > var->xres_virtual) {
                printk("ERROR: xoffset + xres is greater than xres_virtual\n"); 
		return -EINVAL;
        }
	if (var->yoffset + var->yres > var->yres_virtual) {
		printk("ERROR: yoffset + yres is greater than yres_virtual\n");
                return -EINVAL;
        }

	if (var->xres + var->right_margin +
	    var->hsync_len + var->left_margin > 2048)
		return -EINVAL;
	if (var->yres + var->lower_margin +
	    var->vsync_len + var->upper_margin > 2048)
		return -EINVAL;

	/*
	 * Check size of framebuffer.
	 */
	if (var->xres_virtual * var->yres_virtual *
	    (var->bits_per_pixel >> 3) > max_fb_size)
		return -EINVAL;

	/*
	 * Select most suitable hardware pixel format.
	 */
	pix_fmt = determine_best_pix_fmt(var);
        dev_dbg(fi->dev, "%s determine_best_pix_fmt returned: %d\n", __FUNCTION__, pix_fmt);
	if (pix_fmt < 0)
		return pix_fmt;

	return 0;
}

static void set_dma_control0(struct pxa168fb_info *fbi)
{
	u32 x, x_bk;

	dev_dbg(fbi->fb_info->dev, "FB1: Enter %s\n", __FUNCTION__);
	/*
	 * Get reg's current value
	 */
	if (fbi->id <= 0)
		x_bk = x = readl(fbi->reg_base + LCD_SPU_DMA_CTRL0);
	else
		x_bk = x = readl(fbi->reg_base + LCD_TV_CTRL0);
	/*
	 * clear video layer's field
	 */
	x &= 0xef0fff01;

	/*
	 * If we are in a pseudo-color mode, we need to enable
	 * palette lookup.
	 */
	if (fbi->pix_fmt == PIX_FMT_PSEUDOCOLOR)
		x |= 0x10000000;

	x |= 1 << 6;	/* horizontal smooth filter */
	/*
	 * Configure hardware pixel format.
	 */
	x |= ((fbi->pix_fmt & ~0x1000) >> 1) << 20;

	/*
	 * color format in memory:
	 * PXA168/PXA910:
	 * PIX_FMT_YUV422PACK: UYVY(CbY0CrY1)
	 * PIX_FMT_YUV422PLANAR: YUV
	 * PIX_FMT_YUV420PLANAR: YUV
	 */
	if (((fbi->pix_fmt & ~0x1000) >> 1) == 5) {
		x |= 0x00000002;
		x |= (fbi->panel_rbswap) << 4;
		if ((fbi->pix_fmt & 0x1000))
			x |= 1 << 2;	//Y and U/V is swapped
		else
			x |= (fbi->pix_fmt & 1) << 3;
	} else if (fbi->pix_fmt >= 12) {	/* PIX_FMT_YUV422PACK_IRE_90_270 is here */
		x |= 0x00000002;
		x |= (fbi->pix_fmt & 1) << 3;
		x |= (fbi->panel_rbswap) << 4;
	} else {
		x |= (((fbi->pix_fmt & 1) ^ 1) ^ (fbi->panel_rbswap)) << 4;
	}
	if (x_bk != x) {
		if (fbi->id <= 0){
			writel(x, fbi->reg_base + LCD_SPU_DMA_CTRL0);
		}else{
			x &= ~(1<<8); /*FIX ME*/
			writel(x, fbi->reg_base + LCD_TV_CTRL0);
		}
	}
}

static void set_dma_control1(struct pxa168fb_info *fbi, int sync)
{
	u32 x, x_bk;

        dev_dbg(fbi->fb_info->dev, "FB1: Enter %s\n", __FUNCTION__);
	/*
	 * Get current settings.
	 */
	if (fbi->id <= 0)
		x_bk = x = readl(fbi->reg_base + LCD_SPU_DMA_CTRL1);
	else
		x_bk = x = readl(fbi->reg_base + LCD_TV_CTRL1);
	/*
	 * We trigger DMA on the falling edge of vsync if vsync is
	 * active low, or on the rising edge if vsync is active high.
	 */
	if (!(sync & FB_SYNC_VERT_HIGH_ACT))
		x |= 0x08000000;

	
	if (x_bk != x) {
		if (fbi->id <= 0)
			writel(x, fbi->reg_base + LCD_SPU_DMA_CTRL1);
		else
			writel(x, fbi->reg_base + LCD_TV_CTRL1);
	}
}

static int wait_for_vsync(struct pxa168fb_info *fbi)
{
	if (fbi) {
		wait_event_interruptible(fbi->w_intr_wq,
		atomic_read(&fbi->w_intr));
		atomic_set(&fbi->w_intr, 0);
		return 0;
	}

	return 0;
}

static void set_graphics_start(struct fb_info *fi, int xoffset, int yoffset)
{
	struct pxa168fb_info *fbi = fi->par;
	struct fb_var_screeninfo *var = &fi->var;
	int pixel_offset;
	unsigned long addr = 0;
        static int debugcount = 0;

	if(debugcount < 10)
                debugcount++;
        
        if (debugcount < 9)
                dev_dbg(fi->dev, "Enter %s\n", __FUNCTION__);

	if (!(fbi->new_addr[0])) {
		pixel_offset = (yoffset * var->xres_virtual) + xoffset;
		/* Set this at VSync interrupt time */
		addr = fbi->fb_start_dma + ((pixel_offset * var->bits_per_pixel) >> 3);
		if (fbi->id <= 0) {
			writel(addr, fbi->reg_base + LCD_SPU_DMA_START_ADDR_Y0);
			if (fbi->pix_fmt >= 12 && fbi->pix_fmt <= 15)
				addr += var->xres * var->yres;
			writel(addr, fbi->reg_base + LCD_SPU_DMA_START_ADDR_U0);
			if ((fbi->pix_fmt>>1) == 6)
				addr += var->xres * var->yres/2;
			else if ((fbi->pix_fmt>>1) == 7)
				addr += var->xres * var->yres/4;
			writel(addr, fbi->reg_base + LCD_SPU_DMA_START_ADDR_V0);
		} else {
			writel(addr, fbi->reg_base + LCD_TVD_START_ADDR_Y0);
			if (fbi->pix_fmt >= 12 && fbi->pix_fmt <= 15)
				addr += var->xres * var->yres;
			writel(addr, fbi->reg_base + LCD_TVD_START_ADDR_U0);
			if ((fbi->pix_fmt>>1) == 6)
				addr += var->xres * var->yres/2;
			else if ((fbi->pix_fmt>>1) == 7)
				addr += var->xres * var->yres/4;
			writel(addr, fbi->reg_base + LCD_TVD_START_ADDR_V0);
		}
	} else {
		if (fbi->id <= 0) {
			writel(fbi->new_addr[0], fbi->reg_base + LCD_SPU_DMA_START_ADDR_Y0);
			if (fbi->pix_fmt >= 12 && fbi->pix_fmt <= 15) {
				writel(fbi->new_addr[1], fbi->reg_base + LCD_SPU_DMA_START_ADDR_U0);
				writel(fbi->new_addr[2], fbi->reg_base + LCD_SPU_DMA_START_ADDR_V0);
			}
		} else {
			writel(fbi->new_addr[0], fbi->reg_base + LCD_TVD_START_ADDR_Y0);
			if (fbi->pix_fmt >= 12 && fbi->pix_fmt <= 15) {
				writel(fbi->new_addr[1], fbi->reg_base + LCD_TVD_START_ADDR_U0);
				writel(fbi->new_addr[2], fbi->reg_base + LCD_TVD_START_ADDR_V0);
			}
		}
	}
}

static int pxa168fb_set_par(struct fb_info *fi)
{
	struct pxa168fb_info *fbi = fi->par;
	struct fb_var_screeninfo *var = &fi->var;
	int pix_fmt;
	int xzoom, yzoom;
	int xpos = 0;
	int ypos = 0;

	dev_dbg(fi->dev,"FB1: Enter %s\n", __FUNCTION__);
	/*
	 * Determine which pixel format we're going to use.
	 */
	pix_fmt = determine_best_pix_fmt(&fi->var);
	dev_dbg(fi->dev,"determine_best_pix_fmt returned: %d\n", pix_fmt);
	if (pix_fmt < 0)
		return pix_fmt;
	fbi->pix_fmt = pix_fmt;

	dev_dbg(fi->dev, "pix_fmt=%d\n", pix_fmt);
	dev_dbg(fi->dev,"BPP = %d\n", var->bits_per_pixel);
	/*
	 * Set additional mode info.
	 */
	if (pix_fmt == PIX_FMT_PSEUDOCOLOR)
		fi->fix.visual = FB_VISUAL_PSEUDOCOLOR;
	else
		fi->fix.visual = FB_VISUAL_TRUECOLOR;
	fi->fix.line_length = var->xres_virtual * var->bits_per_pixel / 8;

	/*
	 * Configure global panel parameters.
	 */
	set_dma_control0(fbi);
	set_dma_control1(fbi, fi->var.sync);

	/*
	 * Configure graphics DMA parameters.
	 */
	set_graphics_start(fi, fi->var.xoffset, fi->var.yoffset);

	if (fbi->id <= 0) {
		if (((fbi->pix_fmt & ~0x1000) >> 1) < 6) 
			writel((fbi->surface.viewPortInfo.yPitch) ? 
				(fbi->surface.viewPortInfo.yPitch) :
				var->xres * var->bits_per_pixel >> 3,	
				fbi->reg_base + LCD_SPU_DMA_PITCH_YC);
		else {
			writel(fbi->surface.viewPortInfo.yPitch ?
				fbi->surface.viewPortInfo.yPitch : var->xres,
				fbi->reg_base + LCD_SPU_DMA_PITCH_YC);
			writel((fbi->surface.viewPortInfo.uPitch && fbi->surface.viewPortInfo.vPitch) ?
				((fbi->surface.viewPortInfo.vPitch << 16) |
				 (fbi->surface.viewPortInfo.uPitch)) :
				((var->xres >> 1) << 16 | (var->xres >> 1)),
				fbi->reg_base + LCD_SPU_DMA_PITCH_UV);
		}

		dev_dbg(fi->dev, "Executing Standard Mode\n");
		writel((var->yres << 16) | var->xres,
				fbi->reg_base + LCD_SPU_DMA_HPXL_VLN);

		yzoom = fbi->surface.viewPortInfo.zoomYSize;
		xzoom = fbi->surface.viewPortInfo.zoomXSize;

		writel((yzoom << 16) | xzoom,
				fbi->reg_base + LCD_SPU_DZM_HPXL_VLN);

		if(COMPAT_MODE != 0x2625) {
			/* update video position offset */
			dev_dbg(fi->dev, "Setting Surface offsets...\n");

			writel(CFG_DMA_OVSA_VLN(fbi->surface.viewPortOffset.yOffset)|
				fbi->surface.viewPortOffset.xOffset,
				fbi->reg_base + LCD_SPUT_DMA_OVSA_HPXL_VLN);
		}
		else {
			dev_dbg(fi->dev, "Executing 2625 compatibility mode\n");
			xpos = (var->nonstd & 0x3ff);
			ypos = (var->nonstd >> 10) & 0x3ff;
			writel(CFG_DMA_OVSA_VLN(ypos) | xpos, fbi->reg_base + LCD_SPUT_DMA_OVSA_HPXL_VLN);
			writel((var->height << 16) | var->width,
				fbi->reg_base + LCD_SPU_DZM_HPXL_VLN);
		}
	} else {
		if (((fbi->pix_fmt & ~0x1000) >> 1) < 6)
			writel((fbi->surface.viewPortInfo.yPitch) ?
				(fbi->surface.viewPortInfo.yPitch) :
				var->xres * var->bits_per_pixel >> 3,
				fbi->reg_base + LCD_TVD_PITCH_YC);
		else {
			writel(fbi->surface.viewPortInfo.yPitch ?
				fbi->surface.viewPortInfo.yPitch : var->xres,
				fbi->reg_base + LCD_TVD_PITCH_YC);
			writel((fbi->surface.viewPortInfo.uPitch && fbi->surface.viewPortInfo.vPitch) ?
				((fbi->surface.viewPortInfo.vPitch << 16) |
				 (fbi->surface.viewPortInfo.uPitch)) :
				((var->xres >> 1) << 16 | (var->xres >> 1)),
				fbi->reg_base + LCD_TVD_PITCH_UV);
		}

		dev_dbg(fi->dev, "Executing Standard Mode\n");
		writel((var->yres << 16) | var->xres,
				fbi->reg_base + LCD_TVD_HPXL_VLN);

		yzoom = fbi->surface.viewPortInfo.zoomYSize;
		xzoom = fbi->surface.viewPortInfo.zoomXSize;

		writel((yzoom << 16) | xzoom,
				fbi->reg_base + LCD_TVDZM_HPXL_VLN);

		if(COMPAT_MODE != 0x2625) {
			/* update video position offset */
			dev_dbg(fi->dev, "Setting Surface offsets...\n");

			writel(CFG_DMA_OVSA_VLN(fbi->surface.viewPortOffset.yOffset)|
					fbi->surface.viewPortOffset.xOffset,
					fbi->reg_base + LCD_TVD_OVSA_HPXL_VLN);
		}
		else {
			dev_dbg(fi->dev, "Executing 2625 compatibility mode\n");
			xpos = (var->nonstd & 0x3ff);
			ypos = (var->nonstd >> 10) & 0x3ff;
			writel(CFG_DMA_OVSA_VLN(ypos) | xpos, fbi->reg_base + LCD_TVD_OVSA_HPXL_VLN);
			writel((var->height << 16) | var->width,
					fbi->reg_base + LCD_TVDZM_HPXL_VLN);
		}
	}
        fi->fix.smem_len = var->xres_virtual * var->yres_virtual * var->bits_per_pixel/8;
        fi->screen_size = fi->fix.smem_len;

	return 0;
}

static unsigned int chan_to_field(unsigned int chan, struct fb_bitfield *bf)
{
	return ((chan & 0xffff) >> (16 - bf->length)) << bf->offset;
}

static u32 to_rgb(u16 red, u16 green, u16 blue)
{
	red >>= 8;
	green >>= 8;
	blue >>= 8;

	return (red << 16) | (green << 8) | blue;
}

static int
pxa168fb_setcolreg(unsigned int regno, unsigned int red, unsigned int green,
		 unsigned int blue, unsigned int trans, struct fb_info *fi)
{
	struct pxa168fb_info *fbi = fi->par;
	u32 val;

	if (fi->fix.visual == FB_VISUAL_TRUECOLOR && regno < 16) {
		val =  chan_to_field(red,   &fi->var.red);
		val |= chan_to_field(green, &fi->var.green);
		val |= chan_to_field(blue , &fi->var.blue);
		fbi->pseudo_palette[regno] = val;
	}

	if (fi->fix.visual == FB_VISUAL_PSEUDOCOLOR && regno < 256) {
		val = to_rgb(red, green, blue);
		writel(val, fbi->reg_base + LCD_SPU_SRAM_WRDAT);	/* FIXME */
		writel(0x8300 | regno, fbi->reg_base + LCD_SPU_SRAM_CTRL);
	}

	return 0;
}

static int pxa168fb_pan_display(struct fb_var_screeninfo *var,
				struct fb_info *fi)
{
	set_graphics_start(fi, var->xoffset, var->yoffset);

	return 0;
}

static int pxa168fb_fb_sync(struct fb_info *info)
{
	struct pxa168fb_info *fbi = (struct pxa168fb_info *)info->par;

	return wait_for_vsync(fbi);
}


/*
 *  pxa168fb_handle_irq(two lcd controllers)
 */
static irqreturn_t pxa168fb_handle_irq(int irq, void *dev_id)
{
	struct fb_info *fi = (struct fb_info *)dev_id;
	struct pxa168fb_info *fbi = (struct pxa168fb_info *)fi->par;
	u32     isr;

	isr = readl(fbi->reg_base+SPU_IRQ_ISR);

	if ((isr & DMA_FRAME_IRQ0_ENA_MASK)) {
		/* wake up queue. */
		atomic_set(&fbi->w_intr, 1);
		wake_up(&fbi->w_intr_wq);

		writel((~DMA_FRAME_IRQ0_ENA_MASK), fbi->reg_base + SPU_IRQ_ISR);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static irqreturn_t pxa168fb_tv_handle_irq(int irq, void *dev_id)
{
	struct fb_info *fi = (struct fb_info *)dev_id;
	struct pxa168fb_info *fbi = (struct pxa168fb_info *)fi->par;
	u32     isr;

	isr = readl(fbi->reg_base+SPU_IRQ_ISR);

	if ((isr & TV_DMA_FRAME_IRQ0_ENA_MASK)) {
		/* wake up queue. */
		atomic_set(&fbi->w_intr, 1);
		wake_up(&fbi->w_intr_wq);
                /* add a tasklet. */
                tasklet_schedule(&fbi->tasklet);

		writel((~TV_DMA_FRAME_IRQ0_ENA_MASK), fbi->reg_base + SPU_IRQ_ISR);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static struct fb_ops pxa168fb_ops = {
	.owner		= THIS_MODULE,
	.fb_open	= pxa168fb_open,
	.fb_release	= pxa168fb_release,

	.fb_check_var	= pxa168fb_check_var,
	.fb_set_par	= pxa168fb_set_par,
	.fb_setcolreg	= pxa168fb_setcolreg,
	.fb_pan_display	= pxa168fb_pan_display,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
	.fb_sync	= pxa168fb_fb_sync,
	.fb_ioctl	= pxa168fb_ioctl,
};

static int __init get_ovly_size(char *str)
{
	int n;
	if (!get_option(&str, &n))
		return 0;
	max_fb_size = n;
	fb_size_from_cmd = 1;
	return 1;
}
__setup("ovly_size=", get_ovly_size);

static int __init pxa168fb_probe(struct platform_device *pdev)
{
	struct pxa168fb_mach_info *mi;
	struct fb_info *fi;
	struct pxa168fb_info *fbi;
	struct resource *res;
	int ret;
        int temp;

	mi = pdev->dev.platform_data;
	if (mi == NULL)
		return -EINVAL;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL)
		return -EINVAL;

	fi = framebuffer_alloc(sizeof(struct pxa168fb_info), &pdev->dev);
	if (fi == NULL){
		printk("%s: no enough memory!\n", __func__);
		return -ENOMEM;
	}

	fbi = fi->par;
	platform_set_drvdata(pdev, fbi);
	fbi->fb_info = fi;
	fbi->dev = &pdev->dev;
	fbi->id = pdev->id;
	fbi->panel_rbswap = mi->panel_rbswap;
	fbi->is_blanked = 0;
	fbi->cursor_enabled = 0;
	fbi->debug = 0;
	fbi->mem_status = 0;
	init_waitqueue_head(&fbi->w_intr_wq);
	mutex_init(&fbi->access_ok);

	/* get LCD clock information. */
	fbi->clk = clk_get(&pdev->dev, NULL);

	/*
	 * Initialise static fb parameters.
	 */
	fi->flags = FBINFO_DEFAULT | FBINFO_PARTIAL_PAN_OK |
		    FBINFO_HWACCEL_XPAN | FBINFO_HWACCEL_YPAN;
	fi->node = -1;
	strcpy(fi->fix.id, mi->id);
	fi->fix.type = FB_TYPE_PACKED_PIXELS;
	fi->fix.type_aux = 0;
	fi->fix.xpanstep = 1;
	fi->fix.ypanstep = 1;
	fi->fix.ywrapstep = 0;
	fi->fix.mmio_start = res->start;
	fi->fix.mmio_len = res->end - res->start + 1;
	fi->fix.accel = FB_ACCEL_NONE;
	fi->fbops = &pxa168fb_ops;
	fi->pseudo_palette = fbi->pseudo_palette;

	/*
	 * Map LCD controller registers.
	 */
	fbi->reg_base = ioremap_nocache(res->start, res->end - res->start);
	if (fbi->reg_base == NULL) {
		printk("%s: no enough memory!\n", __func__);
		ret = -ENOMEM;
		goto failed;
	}

	/*
	 * Allocate framebuffer memory.
	 */
	if (!fb_size_from_cmd) {
		if (mi->max_fb_size)
			max_fb_size = mi->max_fb_size;
		else
			max_fb_size = DEFAULT_FB_SIZE;
	}
	max_fb_size = PAGE_ALIGN(max_fb_size);
	fbi->fb_size = max_fb_size;

	/*
	 * FIXME, It may fail to alloc DMA buffer from dma_alloc_xxx
	 */
	fbi->fb_start = dma_alloc_writecombine(fbi->dev, max_fb_size,
						&fbi->fb_start_dma,
						GFP_KERNEL);
	if (!fbi->fb_start || !fbi->fb_start_dma) {
		fbi->new_addr[0] = 0;
		fbi->new_addr[1] = 0;
		fbi->new_addr[2] = 0;
		fbi->mem_status = 1;
		fbi->fb_start = (void *)__get_free_pages(GFP_DMA | GFP_KERNEL,
					get_order(fbi->fb_size));
		fbi->fb_start_dma = (dma_addr_t)__virt_to_phys(fbi->fb_start);
	}

	if (fbi->fb_start == NULL) {
		printk("%s: no enough memory!\n", __func__);
		ret = -ENOMEM;
		goto failed;
	}
	printk("---------FBoverlay DMA buffer phy addr : %x\n",(unsigned int)fbi->fb_start_dma);

	memset(fbi->fb_start, 0, fbi->fb_size);
	fi->fix.smem_start = fbi->fb_start_dma;
	fi->fix.smem_len = fbi->fb_size;
	fi->screen_base = fbi->fb_start;
	fi->screen_size = fbi->fb_size;

#ifdef FB_PM_DEBUG
	pxa168fb_rw_all_regs(fbi, g_regs, 1);
#endif

	/*
	 * Fill in sane defaults.
	 */
	set_mode(fbi, &fi->var, mi->modes, mi->pix_fmt, 1);
	pxa168fb_set_par(fi);
	fbi->active = mi->active;

	if (fbi->id <= 0) {
		/*
		 * Configure default register values.
		 */
		writel(0x00000000, fbi->reg_base + LCD_SPU_DMA_START_ADDR_Y1);
		writel(0x00000000, fbi->reg_base + LCD_SPU_DMA_START_ADDR_U1);
		writel(0x00000000, fbi->reg_base + LCD_SPU_DMA_START_ADDR_V1);
		writel(0x00000000, fbi->reg_base + LCD_SPUT_DMA_OVSA_HPXL_VLN);

		/* Set this frame off by default */
		temp = readl(fbi->reg_base + LCD_SPU_DMA_CTRL0);
		temp &= ~(CFG_DMA_ENA_MASK);
		writel(temp, fbi->reg_base + LCD_SPU_DMA_CTRL0);
	} else {
		writel(0x00000000, fbi->reg_base + LCD_TVD_START_ADDR_Y1);
		writel(0x00000000, fbi->reg_base + LCD_TVD_START_ADDR_U1);
		writel(0x00000000, fbi->reg_base + LCD_TVD_START_ADDR_V1);
		writel(0x00000000, fbi->reg_base + LCD_TVD_OVSA_HPXL_VLN);

		temp = readl(fbi->reg_base + LCD_TV_CTRL0);
		temp &= ~(CFG_DMA_ENA_MASK);
		writel(temp, fbi->reg_base + LCD_TV_CTRL0);
	}

	/*
	 * Allocate color map.
	 */
	if (fb_alloc_cmap(&fi->cmap, 256, 0) < 0) {
		ret = -ENOMEM;
		goto failed;
	}

	/*
	 * Get IRQ number.
	 */
	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (res == NULL)
		return -EINVAL;

	/*
	 * Register irq handler.
	 */
	 if(pdev->id==0)
		ret = request_irq(res->start, pxa168fb_handle_irq, IRQF_SHARED,
					mi->id, fi);
	 else
	 	ret = request_irq(res->start, pxa168fb_tv_handle_irq, IRQF_SHARED,
					mi->id, fi);
	if (ret < 0)
	{
		/*
		 *
		 */
		dev_err(&pdev->dev, "Failed to register pxa168fb: %d\n", ret);
		ret = -ENXIO;
		goto failed;
	}

	/*
	 * Enable Video interrupt
	 */
	fbi->tasklet.next = NULL;
	fbi->tasklet.state = 0;
	atomic_set(&fbi->tasklet.count, 0);
	fbi->tasklet.func = pxa168fb_do_tasklet;
	fbi->tasklet.data = (unsigned long)fi;

	if (fbi->id <= 0){
		writel(readl(fbi->reg_base + SPU_IRQ_ENA) | DMA_FRAME_IRQ0_ENA(0x1),
				fbi->reg_base + SPU_IRQ_ENA);
		writel(0x203eff00, fbi->reg_base + LCD_TV_CTRL1);	/* FIXME */
	} else
                writel(readl(fbi->reg_base + SPU_IRQ_ENA) | TV_DMA_FRAME_IRQ0_ENA(0x1),
                        fbi->reg_base + SPU_IRQ_ENA);

	/*
	 * Register framebuffer.
	 */
	ret = register_framebuffer(fi);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register pxa168fb: %d\n", ret);
		ret = -ENXIO;
		goto failed;
	}

	printk(KERN_INFO "pxa910fb_ovly: frame buffer device was loaded"
		" to /dev/fb%d <%s>.\n", fi->node, fi->fix.id);

#ifdef CONFIG_DVFM
	dvfm_register("overlay1", &dvfm_dev_idx);
#endif
	return 0;

failed:
	platform_set_drvdata(pdev, NULL);
	fb_dealloc_cmap(&fi->cmap);
	if (fbi->fb_start != NULL) {
		if (fbi->mem_status)
			free_pages((unsigned long)fbi->fb_start,
				get_order(max_fb_size));
		else
			dma_free_writecombine(fbi->dev, max_fb_size,
				fbi->fb_start, fbi->fb_start_dma);
	}
	if (fbi->reg_base != NULL)
		iounmap(fbi->reg_base);
	kfree(fbi);
	return ret;
}

#ifdef CONFIG_PM
static int pxa168fb_vid_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	struct pxa168fb_info *fbi = platform_get_drvdata(pdev);
	struct fb_info *fi = fbi->fb_info;

#ifdef FB_PM_DEBUG
	pxa168fb_rw_all_regs(fbi, g_regs1, 1);
#endif

	if (mesg.event & PM_EVENT_SLEEP)
		fb_set_suspend(fi, 1);
	pdev->dev.power.power_state = mesg;

#ifdef FB_PM_DEBUG
	pxa168fb_rw_all_regs(fbi, g_regs, 0);
#endif

	return 0;
}

static int pxa168fb_vid_resume(struct platform_device *pdev)
{
	struct pxa168fb_info *fbi = platform_get_drvdata(pdev);
	struct fb_info *fi = fbi->fb_info;
	unsigned int temp;

	clk_enable(fbi->clk);
	if (pxa168fb_set_par(fi) != 0) {
		printk(KERN_INFO "pxa168fb_vid_resume(): Failed in "
				"pxa168fb_set_par().\n");
		return -1;
	}

	fb_set_suspend(fi, 0);

	if(fbi->active){
		if (fbi->id <= 0) {
			temp = readl(fbi->reg_base + SPU_IRQ_ENA);
			temp |= GRA_FRAME_IRQ0_ENA_MASK,
			writel(temp, fbi->reg_base + SPU_IRQ_ENA);

			temp = readl(fbi->reg_base + LCD_SPU_DMA_CTRL0);
			temp |= CFG_DMA_ENA_MASK | 0x1;
			writel(temp, fbi->reg_base + LCD_SPU_DMA_CTRL0);
		} else {
			temp = readl(fbi->reg_base + SPU_IRQ_ENA);
			temp |= TV_FRAME_IRQ0_ENA_MASK,
			writel(temp, fbi->reg_base + SPU_IRQ_ENA);

			temp = readl(fbi->reg_base + LCD_TV_CTRL0);
			temp |= CFG_DMA_ENA_MASK | 0x1;
			writel(temp, fbi->reg_base + LCD_TV_CTRL0);
		}
	}

#ifdef FB_PM_DEBUG
	{
		u32 i;
		u32 reg;

		for (i = 0xC0; i <= 0x01C4; i += 4) {
			reg = readl(fbi->reg_base + i);
			if (reg != g_regs1[i])
				printk("Register 0x%08x: 0x%08x - 0x%08x.\n",
						i, g_regs1[i], reg);
		}
	}
#endif

	clk_disable(fbi->clk);
	printk(KERN_INFO "pxa168fb_vid_resumed\n");

	return 0;
}
#endif


static struct platform_driver pxa168fb_driver = {
	.probe		= pxa168fb_probe,
/*	.remove		= pxa168fb_remove,		*/
#ifdef CONFIG_PM
	.suspend	= pxa168fb_vid_suspend,
	.resume		= pxa168fb_vid_resume,
#endif
	.driver		= {
		.name	= "pxa910fb_ovly",
		.owner	= THIS_MODULE,
	},
};

static int __devinit pxa168fb_init(void)
{
	return platform_driver_register(&pxa168fb_driver);
}

/*module_init(pxa168fb_init);*/
late_initcall(pxa168fb_init);

MODULE_DESCRIPTION("Framebuffer driver for PXA168");
MODULE_LICENSE("GPL");
