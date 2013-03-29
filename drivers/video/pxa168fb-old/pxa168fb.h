/*
 * linux/include/video/dovefbreg.h -- Marvell frame buffer for DOVE
 *
 *
 * Copyright (C) Marvell Semiconductor Company.  All rights reserved.
 *
 * Written by Green Wan <gwan@marvell.com>
 *
 * Adapted from:  linux/drivers/video/skeletonfb.c
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file COPYING in the main directory of this archive for
 * more details.
 *
 */
#ifndef _PXA168FB_H_
#define _PXA168FB_H_

/* ------------< LCD register >------------ */
/* TV patch register for MMP2 */
#define LCD_TVD_START_ADDR_Y0            (0x0000)       /* 32 bit       TV Video Frame0 Y Starting Address */
#define LCD_TVD_START_ADDR_U0            (0x0004)       /* 32 bit       TV Video Frame0 U Starting Address */
#define LCD_TVD_START_ADDR_V0            (0x0008)       /* 32 bit       TV Video Frame0 V Starting Address */
#define LCD_TVD_START_ADDR_C0            (0x000C)       /* 32 bit       TV Video Frame0 Command Starting Address */
#define LCD_TVD_START_ADDR_Y1            (0x0010)       /* 32 bit       TV Video Frame1 Y Starting Address Register*/
#define LCD_TVD_START_ADDR_U1            (0x0014)       /* 32 bit       TV Video Frame1 U Starting Address Register*/
#define LCD_TVD_START_ADDR_V1            (0x0018)       /* 32 bit       TV Video Frame1 V Starting Address Register*/
#define LCD_TVD_START_ADDR_C1            (0x001C)       /* 32 bit       TV Video Frame1 Command Starting Address Register*/
#define LCD_TVD_PITCH_YC                 (0x0020)       /* 32 bit       TV Video Y andC Line Length(Pitch)Register*/
#define LCD_TVD_PITCH_UV                 (0x0024)       /* 32 bit       TV Video U andV Line Length(Pitch)Register*/
#define LCD_TVD_OVSA_HPXL_VLN            (0x0028)       /* 32 bit     TV Video Starting Point on Screen Register*/
#define LCD_TVD_HPXL_VLN                 (0x002C)       /* 32 bit       TV Video Source Size Register*/
#define LCD_TVDZM_HPXL_VLN               (0x0030)/* 32 bit      TV Video Destination Size (After Zooming)Register*/
#define LCD_TVG_START_ADDR0              (0x0034)/* 32 bit      TV Graphic Frame 0 Starting Address Register*/
#define LCD_TVG_START_ADDR1              (0x0038)/* 32 bit      TV Graphic Frame 1 Starting Address Register*/
#define LCD_TVG_PITCH                    (0x003C)       /* 32 bit       TV Graphic Line Length(Pitch)Register*/
#define LCD_TVG_OVSA_HPXL_VLN            (0x0040)       /* 32 bit       TV Graphic Starting Point on Screen Register*/
#define LCD_TVG_HPXL_VLN                 (0x0044)       /* 32 bit       TV Graphic Source Size Register*/
#define LCD_TVGZM_HPXL_VLN               (0x0048)       /* 32 bit       TV Graphic Destination size (after Zooming)Register*/
#define LCD_TVC_OVSA_HPXL_VLN            (0x004C)/* 32 bit      TV Hardware Cursor Starting Point on screen Register*/
#define LCD_TVC_HPXL_VLN                 (0x0050)       /* 32 bit       TV Hardware Cursor Size Register */
#define LCD_TV_V_H_TOTAL                 (0x0054)       /* 32 bit       TV Total Screen Size Register*/
#define LCD_TV_V_H_ACTIVE                (0x0058)       /* 32 bit       TV Screen Active Size Register*/
#define LCD_TV_H_PORCH                   (0x005C)       /* 32 bit       TV Screen Horizontal Porch Register*/
#define LCD_TV_V_PORCH                   (0x0060)       /* 32 bit       TV Screen Vertical Porch Register*/
#define LCD_TV_BLANKCOLOR                (0x0064)       /* 32 bit       TV Screen Blank Color Register*/
#define LCD_TV_ALPHA_COLOR1              (0x0068)       /* 32 bit       TV Hardware Cursor Color1 Register*/
#define LCD_TV_ALPHA_COLOR2              (0x006C)       /* 32 bit       TV Hardware Cursor Color2 Register*/
#define LCD_TV_COLORKEY_Y                (0x0070)       /* 32 bit       TV Video Y Color Key Control*/
#define LCD_TV_COLORKEY_U                (0x0074)       /* 32 bit       TV Video U Color Key Control*/
#define LCD_TV_COLORKEY_V                (0x0078)       /* 32 bit       TV Video V Color Key Control*/
#define LCD_TV_SEPXLCNT                  (0x007C)       /* 32 bit       TV VSYNC PulsePixel Edge Control Register*/
#define LCD_TV_CTRL0                     (0x0080)       /* 32 bit       TV Path DMA Control 0*/
#define LCD_TV_CTRL1                     (0x0084)       /* 32 bit       TV Path DMA Control 1*/
#define LCD_TV_CONTRAST                  (0x0088)       /* 32 bit       TV Path Video Contrast*/
#define LCD_TV_SATURATION                (0x008C)       /* 32 bit       TV Path Video Saturation*/
#define LCD_TV_CBSH_HUE                  (0x0090)       /* 32 bit       TV Path Video Hue Adjust*/
#define LCD_TVIF_CTRL                    (0x0094)       /* 32 bit TV Path TVIF Control  Register */
#define LCD_TVIOPAD_CTRL                 (0x0098)       /* 32 bit TV Path I/O Pad Control*/
#define LCD_TCLK_DIV                     (0x009C)       /* 32 bit TV Path Cloc  Divider  */

/* Video Frame 0&1 start address registers */
#define	LCD_SPU_DMA_START_ADDR_Y0	0x00C0
#define	LCD_SPU_DMA_START_ADDR_U0	0x00C4
#define	LCD_SPU_DMA_START_ADDR_V0	0x00C8
#define LCD_CFG_DMA_START_ADDR_0	0x00CC /* Cmd address */
#define	LCD_SPU_DMA_START_ADDR_Y1	0x00D0
#define	LCD_SPU_DMA_START_ADDR_U1	0x00D4
#define	LCD_SPU_DMA_START_ADDR_V1	0x00D8
#define LCD_CFG_DMA_START_ADDR_1	0x00DC /* Cmd address */

/* YC & UV Pitch */
#define LCD_SPU_DMA_PITCH_YC		0x00E0
#define     SPU_DMA_PITCH_C(c)		(c<<16)
#define     SPU_DMA_PITCH_Y(y)		(y)
#define LCD_SPU_DMA_PITCH_UV		0x00E4
#define     SPU_DMA_PITCH_V(v)		(v<<16)
#define     SPU_DMA_PITCH_U(u)		(u)

/* Video Starting Point on Screen Register */
#define LCD_SPUT_DMA_OVSA_HPXL_VLN		0x00E8
#define     CFG_DMA_OVSA_VLN(y)			(y<<16) /* 0~0xfff */
#define     CFG_DMA_OVSA_HPXL(x)		(x)     /* 0~0xfff */

/* Video Size Register */
#define LCD_SPU_DMA_HPXL_VLN			0x00EC
#define     CFG_DMA_VLN(y)			(y<<16)
#define     CFG_DMA_HPXL(x)			(x)

/* Video Size After zooming Register */
#define LCD_SPU_DZM_HPXL_VLN			0x00F0
#define     CFG_DZM_VLN(y)			(y<<16)
#define     CFG_DZM_HPXL(x)			(x)

/* Graphic Frame 0&1 Starting Address Register */
#define LCD_CFG_GRA_START_ADDR0			0x00F4
#define LCD_CFG_GRA_START_ADDR1			0x00F8

/* Graphic Frame Pitch */
#define LCD_CFG_GRA_PITCH			0x00FC

/* Graphic Starting Point on Screen Register */
#define LCD_SPU_GRA_OVSA_HPXL_VLN		0x0100
#define     CFG_GRA_OVSA_VLN(y)			(y<<16)
#define     CFG_GRA_OVSA_HPXL(x)		(x)

/* Graphic Size Register */
#define LCD_SPU_GRA_HPXL_VLN			0x0104
#define     CFG_GRA_VLN(y)			(y<<16)
#define     CFG_GRA_HPXL(x)			(x)

/* Graphic Size after Zooming Register */
#define LCD_SPU_GZM_HPXL_VLN			0x0108
#define     CFG_GZM_VLN(y)			(y<<16)
#define     CFG_GZM_HPXL(x)			(x)

/* HW Cursor Starting Point on Screen Register */
#define LCD_SPU_HWC_OVSA_HPXL_VLN		0x010C
#define     CFG_HWC_OVSA_VLN(y)			(y<<16)
#define     CFG_HWC_OVSA_HPXL(x)		(x)

/* HW Cursor Size */
#define LCD_SPU_HWC_HPXL_VLN			0x0110
#define     CFG_HWC_VLN(y)			(y<<16)
#define     CFG_HWC_HPXL(x)			(x)

/* Total Screen Size Register */
#define LCD_SPUT_V_H_TOTAL			0x0114
#define     CFG_V_TOTAL(y)			((y)<<16)
#define     CFG_H_TOTAL(x)			(x)

/* Total Screen Active Size Register */
#define LCD_SPU_V_H_ACTIVE			0x0118
#define     CFG_V_ACTIVE(y)			((y)<<16)
#define     CFG_H_ACTIVE(x)			(x)

/* Screen H&V Porch Register */
#define LCD_SPU_H_PORCH				0x011C
#define     CFG_H_BACK_PORCH(b)			(b<<16)
#define     CFG_H_FRONT_PORCH(f)		(f)
#define LCD_SPU_V_PORCH				0x0120
#define     CFG_V_BACK_PORCH(b)			(b<<16)
#define     CFG_V_FRONT_PORCH(f)		(f)

/* Screen Blank Color Register */
#define LCD_SPU_BLANKCOLOR			0x0124
#define     CFG_BLANKCOLOR_MASK			0x00FFFFFF
#define     CFG_BLANKCOLOR_R_MASK		0x000000FF
#define     CFG_BLANKCOLOR_G_MASK		0x0000FF00
#define     CFG_BLANKCOLOR_B_MASK		0x00FF0000

/* HW Cursor Color 1&2 Register */
#define LCD_SPU_ALPHA_COLOR1			0x0128
#define     CFG_HWC_COLOR1			0x00FFFFFF
#define     CFG_HWC_COLOR1_R(red)		(red<<16)
#define     CFG_HWC_COLOR1_G(green)		(green<<8)
#define     CFG_HWC_COLOR1_B(blue)		(blue)
#define     CFG_HWC_COLOR1_R_MASK		0x000000FF
#define     CFG_HWC_COLOR1_G_MASK		0x0000FF00
#define     CFG_HWC_COLOR1_B_MASK		0x00FF0000
#define LCD_SPU_ALPHA_COLOR2			0x012C
#define     CFG_HWC_COLOR2			0x00FFFFFF
#define     CFG_HWC_COLOR2_R_MASK		0x000000FF
#define     CFG_HWC_COLOR2_G_MASK		0x0000FF00
#define     CFG_HWC_COLOR2_B_MASK		0x00FF0000

/* Video YUV Color Key Control */
#define LCD_SPU_COLORKEY_Y			0x0130
#define     CFG_CKEY_Y2(y2)			((y2)<<24)
#define     CFG_CKEY_Y2_MASK			0xFF000000
#define     CFG_CKEY_Y1(y1)			((y1)<<16)
#define     CFG_CKEY_Y1_MASK			0x00FF0000
#define     CFG_CKEY_Y(y)			((y)<<8)
#define     CFG_CKEY_Y_MASK			0x0000FF00
#define     CFG_ALPHA_Y(y)			(y)
#define     CFG_ALPHA_Y_MASK			0x000000FF
#define LCD_SPU_COLORKEY_U			0x0134
#define     CFG_CKEY_U2(u2)			((u2)<<24)
#define     CFG_CKEY_U2_MASK			0xFF000000
#define     CFG_CKEY_U1(u1)			((u1)<<16)
#define     CFG_CKEY_U1_MASK			0x00FF0000
#define     CFG_CKEY_U(u)			((u)<<8)
#define     CFG_CKEY_U_MASK			0x0000FF00
#define     CFG_ALPHA_U(u)			(u)
#define     CFG_ALPHA_U_MASK			0x000000FF
#define LCD_SPU_COLORKEY_V			0x0138
#define     CFG_CKEY_V2(v2)			((v2)<<24)
#define     CFG_CKEY_V2_MASK			0xFF000000
#define     CFG_CKEY_V1(v1)			((v1)<<16)
#define     CFG_CKEY_V1_MASK			0x00FF0000
#define     CFG_CKEY_V(v)			((v)<<8)
#define     CFG_CKEY_V_MASK			0x0000FF00
#define     CFG_ALPHA_V(v)			(v)
#define     CFG_ALPHA_V_MASK			0x000000FF

#define LCD_PN_SEPXLCNT				0x013c	//MMP2

/* SPI Read Data Register */
#define LCD_SPU_SPI_RXDATA			0x0140

/* Smart Panel Read Data Register */
#define LCD_SPU_ISA_RSDATA			0x0144
#define     ISA_RXDATA_16BIT_1_DATA_MASK	0x000000FF
#define     ISA_RXDATA_16BIT_2_DATA_MASK	0x0000FF00
#define     ISA_RXDATA_16BIT_3_DATA_MASK	0x00FF0000
#define     ISA_RXDATA_16BIT_4_DATA_MASK	0xFF000000
#define     ISA_RXDATA_32BIT_1_DATA_MASK	0x00FFFFFF

#define LCD_SPU_DBG_ISA                  (0x0148)       //TTC
#define LCD_SPU_DMAVLD_YC                (0x014C)
#define LCD_SPU_DMAVLD_UV                (0x0150)
#define LCD_SPU_DMAVLD_UVSPU_GRAVLD      (0x0154)

#define LCD_READ_IOPAD                   (0x0148)       //MMP2
#define LCD_DMAVLD_YC                    (0x014C)
#define LCD_DMAVLD_UV                    (0x0150)
#define LCD_TVGGRAVLD_HLEN               (0x0154)

/* HWC SRAM Read Data Register */
#define LCD_SPU_HWC_RDDAT			0x0158

/* Gamma Table SRAM Read Data Register */
#define LCD_SPU_GAMMA_RDDAT			0x015c
#define     CFG_GAMMA_RDDAT_MASK		0x000000FF

/* Palette Table SRAM Read Data Register */
#define LCD_SPU_PALETTE_RDDAT			0x0160
#define     CFG_PALETTE_RDDAT_MASK		0x00FFFFFF

#define LCD_SPU_DBG_DMATOP               (0x0164)       //TTC
#define LCD_SPU_DBG_GRATOP               (0x0168)
#define LCD_SPU_DBG_TXCTRL               (0x016C)
#define LCD_SPU_DBG_SLVTOP               (0x0170)
#define LCD_SPU_DBG_MUXTOP               (0x0174)

#define LCD_SLV_DBG                      (0x0164)       //MMP2
#define LCD_TVDVLD_YC                    (0x0168)
#define LCD_TVDVLD_UV                    (0x016C)
#define LCD_TVC_RDDAT                    (0x0170)
#define LCD_TV_GAMMA_RDDAT               (0x0174)

/* I/O Pads Input Read Only Register */
#define LCD_SPU_IOPAD_IN			0x0178
#define     CFG_IOPAD_IN_MASK			0x0FFFFFFF

#define LCD_TV_PALETTE_RDDAT             (0x0178)	//MMP2

/* Reserved Read Only Registers */
#define LCD_CFG_RDREG5F				0x017C
#define     IRE_FRAME_CNT_MASK			0x000000C0
#define     IPE_FRAME_CNT_MASK			0x00000030
#define     GRA_FRAME_CNT_MASK			0x0000000C  /* Graphic */
#define     DMA_FRAME_CNT_MASK			0x00000003  /* Video */

#define LCD_FRAME_CNT                    (0x017C)	//MMP2

/* SPI Control Register. */
#define LCD_SPU_SPI_CTRL			0x0180
#define     CFG_SCLKCNT(div)			(div<<24)  /* 0xFF~0x2 */
#define     CFG_SCLKCNT_MASK			0xFF000000
#define     CFG_RXBITS(rx)			((rx - 1)<<16)   /* 0x1F~0x1 */
#define     CFG_RXBITS_MASK			0x00FF0000
#define     CFG_TXBITS(tx)			((tx - 1)<<8)    /* 0x1F~0x1 */
#define     CFG_TXBITS_MASK			0x0000FF00
#define     CFG_CLKINV(clk)			(clk<<7)
#define     CFG_CLKINV_MASK			0x00000080
#define     CFG_KEEPXFER(transfer)		(transfer<<6)
#define     CFG_KEEPXFER_MASK			0x00000040
#define     CFG_RXBITSTO0(rx)			(rx<<5)
#define     CFG_RXBITSTO0_MASK			0x00000020
#define     CFG_TXBITSTO0(tx)			(tx<<4)
#define     CFG_TXBITSTO0_MASK			0x00000010
#define     CFG_SPI_ENA(spi)			(spi<<3)
#define     CFG_SPI_ENA_MASK			0x00000008
#define     CFG_SPI_SEL(spi)			(spi<<2)
#define     CFG_SPI_SEL_MASK			0x00000004
#define     CFG_SPI_3W4WB(wire)			(wire<<1)
#define     CFG_SPI_3W4WB_MASK			0x00000002
#define     CFG_SPI_START(start)		(start)
#define     CFG_SPI_START_MASK			0x00000001

/* SPI Tx Data Register */
#define LCD_SPU_SPI_TXDATA			0x0184

/*
   1. Smart Pannel 8-bit Bus Control Register.
   2. AHB Slave Path Data Port Register
*/
#define LCD_SPU_SMPN_CTRL			0x0188

/* DMA Control 0 Register */
#define LCD_SPU_DMA_CTRL0			0x0190
#define     CFG_NOBLENDING(nb)			(nb<<31)
#define     CFG_NOBLENDING_MASK			0x80000000
#define     CFG_GAMMA_ENA(gn)			(gn<<30)
#define     CFG_GAMMA_ENA_MASK			0x40000000
#define     CFG_CBSH_ENA(cn)			(cn<<29)
#define     CFG_CBSH_ENA_MASK			0x20000000
#define     CFG_PALETTE_ENA(pn)			(pn<<28)
#define     CFG_PALETTE_ENA_MASK		0x10000000
#define     CFG_ARBFAST_ENA(an)			(an<<27)
#define     CFG_ARBFAST_ENA_MASK		0x08000000
#define     CFG_HWC_1BITMOD(mode)		(mode<<26)
#define     CFG_HWC_1BITMOD_MASK		0x04000000
#define     CFG_HWC_1BITENA(mn)			(mn<<25)
#define     CFG_HWC_1BITENA_MASK		0x02000000
#define     CFG_HWC_ENA(cn)		        (cn<<24)
#define     CFG_HWC_ENA_MASK			0x01000000
#define     CFG_DMAFORMAT(dmaformat)		(dmaformat<<20)
#define     CFG_DMAFORMAT_MASK			0x00F00000
#define     CFG_GRAFORMAT(graformat)		(graformat<<16)
#define     CFG_GRAFORMAT_MASK			0x000F0000
/* for graphic part */
#define     CFG_GRA_FTOGGLE(toggle)		(toggle<<15)
#define     CFG_GRA_FTOGGLE_MASK		0x00008000
#define     CFG_GRA_HSMOOTH(smooth)		(smooth<<14)
#define     CFG_GRA_HSMOOTH_MASK		0x00004000
#define     CFG_GRA_TSTMODE(test)		(test<<13)
#define     CFG_GRA_TSTMODE_MASK		0x00002000
#define     CFG_GRA_SWAPRB(swap)		(swap<<12)
#define     CFG_GRA_SWAPRB_MASK			0x00001000
#define     CFG_GRA_SWAPUV(swap)		(swap<<11)
#define     CFG_GRA_SWAPUV_MASK			0x00000800
#define     CFG_GRA_SWAPYU(swap)		(swap<<10)
#define     CFG_GRA_SWAPYU_MASK			0x00000400
#define     CFG_YUV2RGB_GRA(cvrt)		(cvrt<<9)
#define     CFG_YUV2RGB_GRA_MASK		0x00000200
#define     CFG_GRA_ENA(gra)			(gra<<8)
#define     CFG_GRA_ENA_MASK			0x00000100
/* for video part */
#define     CFG_DMA_FTOGGLE(toggle)		(toggle<<7)
#define     CFG_DMA_FTOGGLE_MASK		0x00000080
#define     CFG_DMA_HSMOOTH(smooth)		(smooth<<6)
#define     CFG_DMA_HSMOOTH_MASK		0x00000040
#define     CFG_DMA_TSTMODE(test)		(test<<5)
#define     CFG_DMA_TSTMODE_MASK		0x00000020
#define     CFG_DMA_SWAPRB(swap)		(swap<<4)
#define     CFG_DMA_SWAPRB_MASK			0x00000010
#define     CFG_DMA_SWAPUV(swap)		(swap<<3)
#define     CFG_DMA_SWAPUV_MASK			0x00000008
#define     CFG_DMA_SWAPYU(swap)		(swap<<2)
#define     CFG_DMA_SWAPYU_MASK			0x00000004
#define     CFG_DMA_SWAP_MASK			0x0000001C
#define     CFG_YUV2RGB_DMA(cvrt)		(cvrt<<1)
#define     CFG_YUV2RGB_DMA_MASK		0x00000002
#define     CFG_DMA_ENA(video)			(video)
#define     CFG_DMA_ENA_MASK			0x00000001

/* DMA Control 1 Register */
#define LCD_SPU_DMA_CTRL1			0x0194
#define     CFG_FRAME_TRIG(trig)		(trig<<31)
#define     CFG_FRAME_TRIG_MASK			0x80000000
#define     CFG_VSYNC_TRIG(trig)		(trig<<28)
#define     CFG_VSYNC_TRIG_MASK			0x70000000
#define     CFG_VSYNC_INV(inv)			(inv<<27)
#define     CFG_VSYNC_INV_MASK			0x08000000
#define     CFG_COLOR_KEY_MODE(cmode)		(cmode<<24)
#define     CFG_COLOR_KEY_MASK			0x07000000
#define     CFG_CARRY(carry)			(carry<<23)
#define     CFG_CARRY_MASK			0x00800000
#define     CFG_LNBUF_ENA(lnbuf)		(lnbuf<<22)
#define     CFG_LNBUF_ENA_MASK			0x00400000
#define     CFG_GATED_ENA(gated)		(gated<<21)
#define     CFG_GATED_ENA_MASK			0x00200000
#define     CFG_PWRDN_ENA(power)		(power<<20)
#define     CFG_PWRDN_ENA_MASK			0x00100000
#define     CFG_DSCALE(dscale)			(dscale<<18)
#define     CFG_DSCALE_MASK			0x000C0000
#define     CFG_ALPHA_MODE(amode)		(amode<<16)
#define     CFG_ALPHA_MODE_MASK			0x00030000
#define     CFG_ALPHA(alpha)			(alpha<<8)
#define     CFG_ALPHA_MASK			0x0000FF00
#define     CFG_PXLCMD(pxlcmd)			(pxlcmd)
#define     CFG_PXLCMD_MASK			0x000000FF

/* SRAM Control Register */
#define LCD_SPU_SRAM_CTRL			0x0198
#define     CFG_SRAM_INIT_WR_RD(mode)		(mode<<14)
#define     CFG_SRAM_INIT_WR_RD_MASK		0x0000C000
#define     CFG_SRAM_ADDR_LCDID(id)		(id<<8)
#define     CFG_SRAM_ADDR_LCDID_MASK		0x00000F00
#define     CFG_SRAM_ADDR(addr)			(addr)
#define     CFG_SRAM_ADDR_MASK			0x000000FF

/* SRAM Write Data Register */
#define LCD_SPU_SRAM_WRDAT			0x019C

/* SRAM RTC/WTC Control Register */
#define LCD_SPU_SRAM_PARA0			0x01A0

/* SRAM Power Down Control Register */
#define LCD_SPU_SRAM_PARA1			0x01A4
#define     CFG_CSB_256x32(hwc)			(hwc<<15)	/* HWC */
#define     CFG_CSB_256x32_MASK			0x00008000
#define     CFG_CSB_256x24(palette)		(palette<<14)	/* Palette */
#define     CFG_CSB_256x24_MASK			0x00004000
#define     CFG_CSB_256x8(gamma)		(gamma<<13)	/* Gamma */
#define     CFG_CSB_256x8_MASK			0x00002000
#define     CFG_PDWN256x32(pdwn)		(pdwn<<7)	/* HWC */
#define     CFG_PDWN256x32_MASK			0x00000080
#define     CFG_PDWN256x24(pdwn)		(pdwn<<6)	/* Palette */
#define     CFG_PDWN256x24_MASK			0x00000040
#define     CFG_PDWN256x8(pdwn)			(pdwn<<5)	/* Gamma */
#define     CFG_PDWN256x8_MASK			0x00000020
#define     CFG_PDWN32x32(pdwn)			(pdwn<<3)
#define     CFG_PDWN32x32_MASK			0x00000008
#define     CFG_PDWN16x66(pdwn)			(pdwn<<2)
#define     CFG_PDWN16x66_MASK			0x00000004
#define     CFG_PDWN32x66(pdwn)			(pdwn<<1)
#define     CFG_PDWN32x66_MASK			0x00000002
#define     CFG_PDWN64x66(pdwn)			(pdwn)
#define     CFG_PDWN64x66_MASK			0x00000001

/* Smart or Dumb Panel Clock Divider */
#define LCD_CFG_SCLK_DIV			0x01A8
#define     SCLK_SOURCE_SELECT(src)		(src<<31)
#define     SCLK_SOURCE_SELECT_MASK		0x80000000
#define     CLK_FRACDIV(frac)			(frac<<16)
#define     CLK_FRACDIV_MASK			0x0FFF0000
#define     CLK_INT_DIV(div)			(div)
#define     CLK_INT_DIV_MASK			0x0000FFFF

/* Video Contrast Register */
#define LCD_SPU_CONTRAST			0x01AC
#define     CFG_BRIGHTNESS(bright)		(bright<<16)
#define     CFG_BRIGHTNESS_MASK			0xFFFF0000
#define     CFG_CONTRAST(contrast)		(contrast)
#define     CFG_CONTRAST_MASK			0x0000FFFF

/* Video Saturation Register */
#define LCD_SPU_SATURATION			0x01B0
#define     CFG_C_MULTS(mult)			(mult<<16)
#define     CFG_C_MULTS_MASK			0xFFFF0000
#define     CFG_SATURATION(sat)			(sat)
#define     CFG_SATURATION_MASK			0x0000FFFF

/* Video Hue Adjust Register */
#define LCD_SPU_CBSH_HUE			0x01B4
#define     CFG_SIN0(sin0)			(sin0<<16)
#define     CFG_SIN0_MASK			0xFFFF0000
#define     CFG_COS0(con0)			(con0)
#define     CFG_COS0_MASK			0x0000FFFF

/* Dump LCD Panel Control Register */
#define LCD_SPU_DUMB_CTRL			0x01B8
#define     CFG_DUMBMODE(mode)			(mode<<28)
#define     CFG_DUMBMODE_MASK			0xF0000000
#define     CFG_LCDGPIO_O(data)			(data<<20)
#define     CFG_LCDGPIO_O_MASK			0x0FF00000
#define     CFG_LCDGPIO_ENA(gpio)		(gpio<<12)
#define     CFG_LCDGPIO_ENA_MASK		0x000FF000
#define     CFG_BIAS_OUT(bias)			(bias<<8)
#define     CFG_BIAS_OUT_MASK			0x00000100
#define     CFG_REVERSE_RGB(rRGB)		(rRGB<<7)
#define     CFG_REVERSE_RGB_MASK		0x00000080
#define     CFG_INV_COMPBLANK(blank)		(blank<<6)
#define     CFG_INV_COMPBLANK_MASK		0x00000040
#define     CFG_INV_COMPSYNC(sync)		(sync<<5)
#define     CFG_INV_COMPSYNC_MASK		0x00000020
#define     CFG_INV_HENA(hena)			(hena<<4)
#define     CFG_INV_HENA_MASK			0x00000010
#define     CFG_INV_VSYNC(vsync)		(vsync<<3)
#define     CFG_INV_VSYNC_MASK			0x00000008
#define     CFG_INV_HSYNC(hsync)		(hsync<<2)
#define     CFG_INV_HSYNC_MASK			0x00000004
#define     CFG_INV_PCLK(pclk)			(pclk<<1)
#define     CFG_INV_PCLK_MASK			0x00000002
#define     CFG_DUMB_ENA(dumb)			(dumb)
#define     CFG_DUMB_ENA_MASK			0x00000001

/* LCD I/O Pads Control Register */
#define SPU_IOPAD_CONTROL			0x01BC
#define     CFG_GRA_VM_ENA(vm)			(vm<<15)        /* gfx */
#define     CFG_GRA_VM_ENA_MASK			0x00008000
#define     CFG_DMA_VM_ENA(vm)			(vm<<13)	/* video */
#define     CFG_DMA_VM_ENA_MASK			0x00002000
#define     CFG_CMD_VM_ENA(vm)			(vm<<13)
#define     CFG_CMD_VM_ENA_MASK			0x00000800
#define     CFG_CSC(csc)			(csc<<8)	/* csc */
#define     CFG_CSC_MASK			0x00000300
#define     CFG_AXICTRL(axi)			(axi<<4)
#define     CFG_AXICTRL_MASK			0x000000F0
#define     CFG_IOPADMODE(iopad)		(iopad)
#define     CFG_IOPADMODE_MASK			0x0000000F

/* LCD Interrupt Control Register */
#define SPU_IRQ_ENA				0x01C0
#define     DMA_FRAME_IRQ0_ENA(irq)		(irq<<31)
#define     DMA_FRAME_IRQ0_ENA_MASK		0x80000000
#define     DMA_FRAME_IRQ1_ENA(irq)		(irq<<30)
#define     DMA_FRAME_IRQ1_ENA_MASK		0x40000000
#define     DMA_FF_UNDERFLOW_ENA(ff)		(ff<<29)
#define     DMA_FF_UNDERFLOW_ENA_MASK		0x20000000
#define     GRA_FRAME_IRQ0_ENA(irq)		(irq<<27)
#define     GRA_FRAME_IRQ0_ENA_MASK		0x08000000
#define     GRA_FRAME_IRQ1_ENA(irq)		(irq<<26)
#define     GRA_FRAME_IRQ1_ENA_MASK		0x04000000
#define     GRA_FF_UNDERFLOW_ENA(ff)		(ff<<25)
#define     GRA_FF_UNDERFLOW_ENA_MASK		0x02000000
#define     VSYNC_IRQ_ENA(vsync_irq)		(vsync_irq<<23)
#define     VSYNC_IRQ_ENA_MASK			0x00800000
#define     DUMB_FRAMEDONE_ENA(fdone)		(fdone<<22)
#define     DUMB_FRAMEDONE_ENA_MASK		0x00400000
#define     TWC_FRAMEDONE_ENA(fdone)		(fdone<<21)
#define     TWC_FRAMEDONE_ENA_MASK		0x00200000
#define     HWC_FRAMEDONE_ENA(fdone)		(fdone<<20)
#define     HWC_FRAMEDONE_ENA_MASK		0x00100000
#define     SLV_IRQ_ENA(irq)			(irq<<19)
#define     SLV_IRQ_ENA_MASK			0x00080000
#define     SPI_IRQ_ENA(irq)			(irq<<18)
#define     SPI_IRQ_ENA_MASK			0x00040000
#define     PWRDN_IRQ_ENA(irq)			(irq<<17)
#define     PWRDN_IRQ_ENA_MASK			0x00020000
#define     ERR_IRQ_ENA(irq)			(irq<<16)
#define     ERR_IRQ_ENA_MASK			0x00010000
#define     CLEAN_SPU_IRQ_ISR(irq)		(irq)
#define     CLEAN_SPU_IRQ_ISR_MASK		0x0000FFFF
#define     TV_DMA_FRAME_IRQ0_ENA(irq)		(irq<<15)
#define     TV_DMA_FRAME_IRQ0_ENA_MASK		0x00008000
#define     TVSYNC_IRQ_ENA(irq)			(irq<<12)
#define	    TVSYNC_IRQ_ENA_MASK			0x00001000
#define     TV_FRAME_IRQ0_ENA(irq)		(irq<<11)
#define     TV_FRAME_IRQ0_ENA_MASK		0x00000800

/* LCD Interrupt Status Register */
#define SPU_IRQ_ISR				0x01C4
#define     DMA_FRAME_IRQ0(irq)			(irq<<31)
#define     DMA_FRAME_IRQ0_MASK			0x80000000
#define     DMA_FRAME_IRQ1(irq)			(irq<<30)
#define     DMA_FRAME_IRQ1_MASK			0x40000000
#define     DMA_FF_UNDERFLOW(ff)		(ff<<29)
#define     DMA_FF_UNDERFLOW_MASK		0x20000000
#define     GRA_FRAME_IRQ0(irq)			(irq<<27)
#define     GRA_FRAME_IRQ0_MASK			0x08000000
#define     GRA_FRAME_IRQ1(irq)			(irq<<26)
#define     GRA_FRAME_IRQ1_MASK			0x04000000
#define     GRA_FF_UNDERFLOW(ff)		(ff<<25)
#define     GRA_FF_UNDERFLOW_MASK		0x02000000
#define     VSYNC_IRQ(vsync_irq)		(vsync_irq<<23)
#define     VSYNC_IRQ_MASK			0x00800000
#define     DUMB_FRAMEDONE(fdone)		(fdone<<22)
#define     DUMB_FRAMEDONE_MASK			0x00400000
#define     TWC_FRAMEDONE(fdone)		(fdone<<21)
#define     TWC_FRAMEDONE_MASK			0x00200000
#define     HWC_FRAMEDONE(fdone)		(fdone<<20)
#define     HWC_FRAMEDONE_MASK			0x00100000
#define     SLV_IRQ(irq)			(irq<<19)
#define     SLV_IRQ_MASK			0x00080000
#define     SPI_IRQ(irq)			(irq<<18)
#define     SPI_IRQ_MASK			0x00040000
#define     PWRDN_IRQ(irq)			(irq<<17)
#define     PWRDN_IRQ_MASK			0x00020000
#define     ERR_IRQ(irq)			(irq<<16)
#define     ERR_IRQ_MASK			0x00010000
#define     TVSYNC_IRQ(irq)            		(irq<<12)
#define     TVSYNC_IRQ_MASK                   	0x00001000
#define     TV_FRAME_IRQ0(irq)			(irq<<11)
#define     TV_FRAME_IRQ0_MASK			0x00000800

/* LCD FIFO Depth register */

#define LCD_FIFO_DEPTH                          0x01c8
#define     VIDEO_FIFO(fi)                      (fi << 0)
#define     VIDEO_FIFO_MASK                     0x00000003
#define     GRAPHIC_FIFO(fi)                    (fi << 2)
#define     GRAPHIC_FIFO_MASK                   0x0000000c

/* read-only */
#define     DMA_FRAME_IRQ0_LEVEL_MASK		0x00008000
#define     DMA_FRAME_IRQ1_LEVEL_MASK		0x00004000
#define     DMA_FRAME_CNT_ISR_MASK		0x00003000
#define     GRA_FRAME_IRQ0_LEVEL_MASK		0x00000800
#define     GRA_FRAME_IRQ1_LEVEL_MASK		0x00000400
#define     GRA_FRAME_CNT_ISR_MASK		0x00000300
#define     VSYNC_IRQ_LEVEL_MASK		0x00000080
#define     DUMB_FRAMEDONE_LEVEL_MASK		0x00000040
#define     TWC_FRAMEDONE_LEVEL_MASK		0x00000020
#define     HWC_FRAMEDONE_LEVEL_MASK		0x00000010
#define     SLV_FF_EMPTY_MASK			0x00000008
#define     DMA_FF_ALLEMPTY_MASK		0x00000004
#define     GRA_FF_ALLEMPTY_MASK		0x00000002
#define     PWRDN_IRQ_LEVEL_MASK		0x00000001

#define SPU_IRQ_RSR                      (0x01C8)       /* 32 bit LCD Interrupt Reset Status*/
#define LCD_GRA_CUTHPXL                  (0x01CC)/* 32 bit Panel Path Graphic Partial Display Horizontal Control Register*/
#define LCD_GRA_CUTVLN                   (0x01D0) /* 32 bit Panel Path Graphic Partial Display Vertical Control Register*/
#define LCD_TVG_CUTHPXL                  (0x01D4) /* 32 bit TV Path Graphic Partial Display     Horizontal Control Register*/
#define LCD_TVG_CUTVLN                   (0x01D8)/* 32 bit TV Path Graphic Partial Display Vertical Control Register*/
#define LCD_TOP_CTRL                     (0x01DC) /* 32 bit LCD Global Control Register*/
#define LCD_SQULN1_CTRL                  (0x01E0)       /* 32 bit LCD SQU Line Buffer Control Register 1*/
#define LCD_SQULN2_CTRL                  (0x01E4)       /* 32 bit LCD SQU Line Buffer Control Register 2*/
#define LCD_AFA_ALL2ONE                  (0x01E8)       /* 32 bit LCD Mixed Overlay Control Register */

/*
 * defined Video Memory Color format for DMA control 0 register
 * DMA0 bit[23:20]
 */
#define VMODE_RGB565		0x0
#define VMODE_RGB1555		0x1
#define VMODE_RGB888PACKED	0x2
#define VMODE_RGB888UNPACKED	0x3
#define VMODE_RGBA888		0x4
#define VMODE_YUV422PACKED	0x5
#define VMODE_YUV422PLANAR	0x6
#define VMODE_YUV420PLANAR	0x7
#define VMODE_SMPNCMD		0x8
#define VMODE_PALETTE4BIT	0x9
#define VMODE_PALETTE8BIT	0xa
#define VMODE_RESERVED		0xb

/*
 * defined Graphic Memory Color format for DMA control 0 register
 * DMA0 bit[19:16]
 */
#define GMODE_RGB565		0x0
#define GMODE_RGB1555		0x1
#define GMODE_RGB888PACKED	0x2
#define GMODE_RGB888UNPACKED	0x3
#define GMODE_RGBA888		0x4
#define GMODE_YUV422PACKED	0x5
#define GMODE_YUV422PLANAR	0x6
#define GMODE_YUV420PLANAR	0x7
#define GMODE_SMPNCMD		0x8
#define GMODE_PALETTE4BIT	0x9
#define GMODE_PALETTE8BIT	0xa
#define GMODE_RESERVED		0xb

/*
 * define for DMA control 1 register
 */
#define DMA1_FRAME_TRIG		31 /* bit location */
#define DMA1_VSYNC_MODE		28
#define DMA1_VSYNC_INV		27
#define DMA1_CKEY		24
#define DMA1_CARRY		23
#define DMA1_LNBUF_ENA		22
#define DMA1_GATED_ENA		21
#define DMA1_PWRDN_ENA		20
#define DMA1_DSCALE		18
#define DMA1_ALPHA_MODE		16
#define DMA1_ALPHA		08
#define DMA1_PXLCMD		00

/*
 * defined for Configure Dumb Mode
 * DUMB LCD Panel bit[31:28]
 */
#define DUMB16_RGB565_0		0x0
#define DUMB16_RGB565_1		0x1
#define DUMB18_RGB666_0		0x2
#define DUMB18_RGB666_1		0x3
#define DUMB12_RGB444_0		0x4
#define DUMB12_RGB444_1		0x5
#define DUMB24_RGB888_0		0x6
#define DUMB_BLANK		0x7

/*
 * defined for Configure I/O Pin Allocation Mode
 * LCD LCD I/O Pads control register bit[3:0]
 */
#define IOPAD_DUMB24                0x0
#define IOPAD_DUMB18SPI             0x1
#define IOPAD_DUMB18GPIO            0x2
#define IOPAD_DUMB16SPI             0x3
#define IOPAD_DUMB16GPIO            0x4
#define IOPAD_DUMB12                0x5
#define IOPAD_SMART18SPI            0x6
#define IOPAD_SMART16SPI            0x7
#define IOPAD_SMART8BOTH            0x8

/*
 * defined Dumb Panel Clock Divider register
 * SCLK_Source bit[31]
 */
#define AXI_BUS_SEL                 0x80000000 /* 0: PLL clock select*/
#define CCD_CLK_SEL                 0x40000000
#define DCON_CLK_SEL                0x20000000
#define ENA_CLK_INT_DIV             CONFIG_FB_DOVE_CLCD_SCLK_DIV
#define IDLE_CLK_INT_DIV            0x1      /* idle Integer Divider */
#define DIS_CLK_INT_DIV             0x0      /* Disable Integer Divider */

/* SRAM ID */
#define SRAMID_gamma_yr             0x0
#define SRAMID_gamma_ug             0x1
#define SRAMID_gamma_vb             0x2
#define SRAMID_palette              0x3
#define SRAMID_hwc                  0xf

/* SRAM INIT Read/Write */
#define SRAMID_INIT_READ			0x0
#define SRAMID_INIT_WRITE			0x2
#define SRAMID_INIT_DEFAULT			0x3

/*
 * defined VSYNC selection mode for DMA control 1 register
 * DMA1 bit[30:28]
 */
#define VMODE_SMPN                  0x0
#define VMODE_SMPNIRQ               0x1
#define VMODE_DUMB                  0x2
#define VMODE_IPE                   0x3
#define VMODE_IRE                   0x4

/*
 * defined Configure Alpha and Alpha mode for DMA control 1 register
 * DMA1 bit[15:08](alpha) / bit[17:16](alpha mode)
 */
/* ALPHA mode */
#define MODE_ALPHA_DMA              0x0
#define MODE_ALPHA_GRA              0x1
#define MODE_ALPHA_CFG              0x2

/* alpha value */
#define ALPHA_NOGRAPHIC		0xFF      /* all video, no graphic */
#define ALPHA_NOVIDEO		0x00      /* all graphic, no video */
#define ALPHA_GRAPHnVIDEO	0x0F      /* Selects graphic & video */

/*
 * defined Pixel Command for DMA control 1 register
 * DMA1 bit[07:00]
 */
#define PIXEL_CMD                  0x81

/* DSI */
#define DSI1_REGS_PHYSICAL_BASE                0xD420B800      /* DSI1 - 4 Lane Controller base */
#define DSI2_REGS_PHYSICAL_BASE                0xD420BA00      /* DSI2 - 3 Lane Controller base */

//       DSI Controller offsets

#define DSI_CTRL_0      0x000   /* DSI control register 0 */
#define DSI_CTRL_1      0x004   /* DSI control register 1 */
#define DSI_CPU_CMD_0   0x020   /* DSI CPU packet command register 0 */
#define DSI_CPU_CMD_1   0x024   /* DSU CPU Packet Command Register 1 */
#define DSI_CPU_CMD_3    0x02C   /* DSU CPU Packet Command Register 3 */
#define DSI_CPU_WDAT_0   0x030   /* DSI CUP */
#define DSI_RX_PKT_HDR_0 0x064   /* Rx Packet Header - data from slave device */
#define DSI_PHY_CTRL_2   0x088   /* DSI DPHI Control Register 2 */
#define DSI_PHY_CTRL_3   0x08C   /* DPHY Control Register 3 */
#define DSI_PHY_RCOMP_0         0x0B0   /* DPHY Rcomp Control Register */
#define DSI_PHY_TIME_0   0x0C0   /* DPHY Timing Control Register 0 */
#define DSI_PHY_TIME_1   0x0C4   /* DPHY Timing Control Register 1 */
#define DSI_PHY_TIME_2   0x0C8   /* DPHY Timing Control Register 2 */
#define DSI_PHY_TIME_3   0x0CC   /* DPHY Timing Control Register 3 */
#define DSI_PHY_TIME_4   0x0D0   /* DPHY Timing Control Register 4 */
#define DSI_PHY_TIME_5   0x0D4   /* DPHY Timing Control Register 5 */
#define DSI_LCD1_CTRL_0  0x100   /* DSI Active Panel 1 Control register 0 */
#define DSI_LCD1_CTRL_1  0x104   /* DSI Active Panel 1 Control register 1 */

#define DSI_LCD1_TIMING_0        0x110   /* Timing register 0 */
#define DSI_LCD1_TIMING_1        0x114   /* Timing register 1 */
#define DSI_LCD1_TIMING_2        0x118   /* Timing register 2 */
#define DSI_LCD1_TIMING_3        0x11C   /* Timing register 3 */
#define DSI_LCD1_WC_0            0x120   /* Word Count register 0 */
#define DSI_LCD1_WC_1            0x124   /* Word Count register 1 */
#define DSI_LCD1_WC_2		 0x128	 /* Word Count register 2 */

#define DSI_CTRL_0_CFG_SOFT_RST		0x80000000
#define DSI_CTRL_0_CFG_SOFT_RST_REG	0x40000000
#endif 	/* __PXA168FB_H__ */
