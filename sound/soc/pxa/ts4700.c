/*
 * ts4700.c  --  SoC audio for embeddedTS' ts47xx board
 *
 * At the time of writing, TS baseboards had either an SGTL5000 or WM8750
 * codec, or an SII9022 HDMI Transmitter.  The latter isn't a codec, of
 * course, but it does require a digital audio stream just like a codec does.
 * The SII9022 is initialised elsewhere (see drivers/video/hdmi_sii9022.c).
 *
 * Portions Copyright 2005 Wolfson Microelectronics PLC.
 * Author: Liam Girdwood
 *         liam.girdwood@wolfsonmicro.com or linux@wolfsonmicro.com
 *
 *  TTC FPGA audio amplifier code taken from arch/arm/mach-pxa/mainstone.c
 *  Copyright:	MontaVista Software Inc.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  Revision history
 *
 *    25th May 2010   Initial version, created by modifying existing code
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/soc-dai.h>

#include <asm/mach-types.h>
#include <asm/io.h>

#include <asm/uaccess.h>
#include <plat/regs-ssp.h>
#include <plat/ssp.h>
#include <mach/addr-map.h>
#include "pxa2xx-i2s.h"
#include "pxa-ssp.h"
#include <mach/clock.h>
#include <mach/regs-apbc.h>

enum {
      CODEC_NONE,
      CODEC_WM8750,
      CODEC_SGTL5000,
      CODEC_SII9022,
      CODEC_LAST = CODEC_SII9022
};

static int usecodec;

module_param(usecodec, int, CODEC_NONE);
MODULE_PARM_DESC(usecodec, "1=WM8750, 2=SGTL5000, 3=SII9022");


#include "../codecs/wm8750.h"  
#include "../codecs/sgtl5000.h"
#include "../codecs/sii9022.h"
#include "pxa3xx-pcm.h"
#include "pxa3xx-ssp.h"

//#define TS4700_CODEC_DEBUG

#ifdef TS4700_CODEC_DEBUG
#define dbg(format, arg...) \
	printk(KERN_INFO format "\n" , ## arg)
#else	
	#define dbg(...)
#endif


/**
 *    ASYSDR is the Audio SYSCLK Dithering Divider Register.
 *    ASSPDR is the SSPx_CLK Dithering Divider Register.
 *
 *    See Marvell pxa168 documentation for details.
 */

#define ASYSDR   (* ((volatile unsigned long *) (APB_VIRT_BASE + 0x51050)))
#define ASSPDR   (* ((volatile unsigned long *) (APB_VIRT_BASE + 0x51054)))
#define MPMU_ACGR      (* ((volatile unsigned long *) (APB_VIRT_BASE + 0x51024)))

/** The SSP1_CLK_RST reg is in the APB2 Peripheral Clock Control block */
#define SSP1_CLK_RST   (* ((volatile unsigned long *) (APBC_VIRT_BASE + 0x81C)))


struct _ssp_conf {
	unsigned int main_clk;
	unsigned int sys_num;
	unsigned int sys_den;
	unsigned int bit_clk;
	unsigned int ssp_num;
	unsigned int ssp_den;
	unsigned int freq_out;
};


/**  
   When SSP1_CLK_RST[FNCLKSEL] = 4, The bitrate of SSPxCLK is programmable, 
   by writing to the ASYSDR and ASSPDR registers, according to the 
   following formula...
   
   ASYSCLK = 312MHz / (sys_num / sys_den)
   
   SSPx_CLK = ASYSCLK / (ssp_num / ssp_den)
   
   The sample-rate is the bitrate divided by the bits per sample.
   
   ASYSCLK appears on the MSYNC pin
             
*/

static struct _ssp_conf ssp_conf[] = {   
	/*main_clk, sys_num, sys_den, bit_clk, ssp_num, ssp_den, freq_out*/
	{49152000,  0x659,    0x100,  3072000, 0x100,   0x10,	48000},
	{45151000,  0x6E9,    0x100,  2822000, 0x100,   0x10,	44100},
	{49152000,  0x659,    0x100,  2048000, 0x180,   0x10,	32000},
	{49152000,  0x659,    0x100,  1536000, 0x200,   0x10,	24000},
	{45151000,  0x6E9,    0x100,  1411000, 0x200,   0x10,	22050},
	{49152000,  0x659,    0x100,  1024000, 0x300,   0x10,	16000},
	{45151000,  0x6E9,    0x100,   706500, 0x400,   0x10,	11025},
	{49152000,  0x659,    0x100,   512000, 0x600,   0x10,	8000},
	{49152000,  0x659,    0x100,  6144000, 0x100,   0x20,	96000},
};

static int ssp_conf_lookup(unsigned int rate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ssp_conf); i++) {
		if (ssp_conf[i].freq_out == rate)		
			return i;		
	}
	return -EINVAL;
}


static void ssp_set_clock(unsigned int ceoff)
{
	unsigned long asysdr,asspdr;
			
	asysdr = (ssp_conf[ceoff].sys_num << 16) | ssp_conf[ceoff].sys_den;
	ASYSDR = asysdr;
	asspdr = (ssp_conf[ceoff].ssp_num << 16) | ssp_conf[ceoff].ssp_den;
	ASSPDR = asspdr;
		
}

/* ts4700 audio subsystem */
static struct snd_soc_device ts4700_snd_devdata;
static struct platform_device *ts4700_snd_device;
extern int tsGetBaseBoard(void);


static const struct snd_soc_dapm_route audio_map_wm8750[] = {
	{ "Headphone Jack", NULL, "LOUT1" },
	{ "Headphone Jack", NULL, "ROUT1" },
	{ "Internal Speaker", NULL, "LOUT2" },
	{ "Internal Speaker", NULL, "ROUT2" },
};

static const struct snd_soc_dapm_widget ts4700_dapm_widgets[] = {
   SND_SOC_DAPM_HP("Headphone Jack", NULL),
   SND_SOC_DAPM_SPK("Internal Speaker", NULL),
   SND_SOC_DAPM_LINE("Line In", NULL),
};


static int ts4700_hw_params(struct snd_pcm_substream *substream,
			  struct snd_pcm_hw_params *params)
{
   static int confTablePatched = 0;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	int ret = 0;	
	u32 dai_format;
	unsigned int clk = 0;
	unsigned int codec_sysclk = 0;
	unsigned int codec_lrclk = 0;
	unsigned int rate = params_rate(params);

	if (usecodec == CODEC_WM8750) {
	   int i;
	   codec_sysclk = WM8750_SYSCLK;
	   codec_lrclk = 0;
	   switch (params_rate(params)) {
   	  case 8000:
   	  case 16000:
   	  case 48000:
   	  case 96000:
   	     clk = 12288000;
   	     break;
   	  case 11025:
   	  case 22050:
   	  case 44100:
   	     clk = 11289600;	      
   	     break;
   	}   	
   	if (! confTablePatched) {
   	   for (i = 0; i < ARRAY_SIZE(ssp_conf); i++)
   	      ssp_conf[i].ssp_den <<= 1;
   	   confTablePatched = 1;
   	}
		   	
   } else if (usecodec == CODEC_SGTL5000) {
      codec_sysclk = SGTL5000_SYSCLK;
      codec_lrclk = SGTL5000_LRCLK;
      clk = 12500000;
   } else {
      codec_sysclk = SII9022_SYSCLK;
       codec_lrclk = SII9022_LRCLK;
      clk = 12500000;       
   }
	
	/* The clock value here is used by the codec driver to program the
	 * codec hardware.
	 */	 
	/* set the codec system clock for DAC and ADC */
		
	dbg("%s rate: %d, clk: %d\n", __func__, params_rate(params), clk);
	
	ret = snd_soc_dai_set_sysclk(codec_dai, codec_sysclk, clk,
				     SND_SOC_CLOCK_IN);
		
	if (ret < 0)
		return ret;

	SSP1_CLK_RST = 0x43;      /* Select "Programmable SSPx_CLK" for SSP1 */	
	MPMU_ACGR |= (1 << 19);   /* Enable APMU_ASYSCLK in Clock Gating Register */
		
	snd_soc_dai_set_sysclk(codec_dai, codec_lrclk, rate, 0);
			
	codec_dai->ops->hw_params(substream, params, codec_dai);	
	
   if (usecodec != CODEC_SGTL5000)
      dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_NB_NF;
   else
      dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBM_CFM | SND_SOC_DAIFMT_NB_NF;

   /* set codec DAI configuration */
   ret = snd_soc_dai_set_fmt(codec_dai, dai_format);
   if (ret < 0)
      return ret;

   /* set cpu DAI configuration */
   ret = snd_soc_dai_set_fmt(cpu_dai, dai_format);
   if (ret < 0)
      return ret;

	return 0;
}

static int ts4700_codec_startup(struct snd_pcm_substream *substream)
{
  struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *machine = rtd->dai;
	struct snd_soc_dai *cpu_dai = machine->cpu_dai;
	struct ssp_device *ssp = cpu_dai->private_data;
	struct snd_soc_dai *codec_dai = machine->codec_dai;
	unsigned int format;

	dbg("%s\n", __func__);
	
	cpu_dai->playback.channels_min = 2;
	cpu_dai->playback.channels_max = 2;
	 	
	if (usecodec == CODEC_WM8750) {
	   /* WM8750 operates in slave mode, with the cpu providing the clock */
	   format =  SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_NB_NF;	
	   __raw_writel(0xE1c0003F, ssp->mmio_base + SSCR0);
	   __raw_writel(0x10F01DC0, ssp->mmio_base + SSCR1);  
	   __raw_writel(0x00F01DC0, ssp->mmio_base + SSCR1);	
	   __raw_writel(0x42200000, ssp->mmio_base + SSPSP);
	} else if (usecodec == CODEC_SGTL5000) {
	
	   /**
	      sgtl5000 clocks, when in slave mode, must be synchronous
	      to SYS_MCLK.  We can't do that, because SYS_MCLK is a
	      12.5MHz signal provided by the FPGA on the TS-4700.
	      So, we need the codec to be master, and the SoC to be slave.
	    */
	 
	 /* I2S fmt, codec is bit clk & frm master, normal bit clock + frame */ 	
	 format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBM_CFM | SND_SOC_DAIFMT_NB_NF;
   
	} else if (usecodec == CODEC_SII9022) {
	
	   /* I2S fmt, codec is bit clk & frm slave, normal bit clock + frame */
	   format =  SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_NB_NF;
	   __raw_writel(0xE1D0003F, ssp->mmio_base + SSCR0);
	   __raw_writel(0x10F01DC0, ssp->mmio_base + SSCR1);  
	   __raw_writel(0x00F01DC0, ssp->mmio_base + SSCR1);	
	   __raw_writel(0x42200000, ssp->mmio_base + SSPSP);

	} else return -1;
	
   __raw_writel(0x3, ssp->mmio_base + SSTSA);
	__raw_writel(0x3, ssp->mmio_base + SSRSA);

	codec_dai->ops->set_fmt(codec_dai, format);

   return 0;
}


static int ts4700_codec_prepare(struct snd_pcm_substream *substream)
{
   struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *machine = rtd->dai;
	struct snd_soc_dai *cpu_dai = machine->cpu_dai;
	struct ssp_device *ssp = cpu_dai->private_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned long rate = runtime->rate;
	int ceoff;

	dbg("%s\n", __func__);
	
	cpu_dai->playback.channels_min = 2;
   cpu_dai->playback.channels_max = 2;

	if (usecodec == CODEC_WM8750) {
	   __raw_writel(0xE1C0003F, ssp->mmio_base + SSCR0);
	   __raw_writel(0x00F01DC0, ssp->mmio_base + SSCR1);	
	   __raw_writel(0x40200004, ssp->mmio_base + SSPSP);
	
	} else if (usecodec == CODEC_SII9022) {
		   
	   __raw_writel(0xE1D0003F, ssp->mmio_base + SSCR0);
	   __raw_writel(0x10F01DC0, ssp->mmio_base + SSCR1);  
	   __raw_writel(0x00F01DC0, ssp->mmio_base + SSCR1);	
	   __raw_writel(0x42200000, ssp->mmio_base + SSPSP);
	} 

   __raw_writel(0x3, ssp->mmio_base + SSTSA);
	__raw_writel(0x3, ssp->mmio_base + SSRSA);
			
	ceoff = ssp_conf_lookup(rate);
	if (ceoff >= 0 )
	{
		ssp_set_clock(ceoff);
	} else {
		printk(KERN_ERR "Unsupported audio sample rate\n");
	}

	/* we have to set the value of FRDC in SSCR0 here because the dai doesn't do it */
	__raw_writel(__raw_readl(ssp->mmio_base + SSCR0) | 0x01000000, 
	   ssp->mmio_base + SSCR0);

   return 0;
}

static void ts4700_codec_shutdown(struct snd_pcm_substream *substream)
{
   dbg("%s\n", __func__);   
}

static struct snd_soc_ops ts4700_ops = {
	.hw_params  = ts4700_hw_params,
   .startup    = ts4700_codec_startup,
	.prepare    = ts4700_codec_prepare,
	.shutdown   = ts4700_codec_shutdown,
};

static int ts4700_wm8750_init(struct snd_soc_codec *codec)
{
	int err;

	dbg("%s\n", __func__);
			
	/* These endpoints are not being used. */
	snd_soc_dapm_nc_pin(codec, "LINPUT2");
	snd_soc_dapm_nc_pin(codec, "RINPUT2");
	snd_soc_dapm_nc_pin(codec, "LINPUT3");
	snd_soc_dapm_nc_pin(codec, "RINPUT3");
	snd_soc_dapm_nc_pin(codec, "OUT3");

	/* Add ts4700 specific widgets */
	err = snd_soc_dapm_new_controls(codec, ts4700_dapm_widgets,
					ARRAY_SIZE(ts4700_dapm_widgets));
	if (err) {
		printk(KERN_ERR "%s: failed to add widgets (%d)\n",
		       __func__, err);
		return err;
	}

	snd_soc_dapm_add_routes(codec, audio_map_wm8750, ARRAY_SIZE(audio_map_wm8750));
	snd_soc_dapm_sync(codec);

	return 0;
}



static int ts4700_sgtl5000_init(struct snd_soc_codec *codec)
{
   dbg("%s\n", __func__);   
	
	
   return 0;
}


static int ts4700_sii9022_init(struct snd_soc_codec *codec)
{
   dbg("%s\n", __func__);
	
   return 0;
}

static struct snd_soc_dai_link ts4700_dai;


/* ts4700 audio machine driver */
static struct snd_soc_card snd_soc_machine_ts4700 = {
	.name		= "ts4700",
	.platform	= &pxa3xx_soc_platform,
	.dai_link	= &ts4700_dai,
	.num_links	= 1,
};

/* ts4700 audio private data */

static struct wm8750_setup_data ts4700_wm8750_setup = {  
   .i2c_address = 0x1a,
	.i2c_bus = 1,
   .spi = 0
};

static struct sgtl5000_platform_data sgtl5000_platform_setup = {
   .i2c_address = 0x0a,
   .i2c_bus = 1,
   .spi = 0
};

static struct sii9022_setup_data sii9022_platform_setup = {
   .clock_enable = NULL
};



static int __init ts4700_init(void)
{
	int ret;	
	int baseBoard = tsGetBaseBoard();

	printk("ts4700 SoC Audio support ");
	
	if (usecodec == CODEC_NONE) {
	
	   printk("on Baseboard: ");
	   switch(baseBoard) {
   	  case 1:  printk("TS-8395\n"); usecodec = CODEC_WM8750; break;
   	  case 5:  printk("TS-8400\n"); usecodec = CODEC_WM8750; break;
   	   
   	  case 2:  printk("TS-8390\n"); usecodec = CODEC_SGTL5000; break;   	
   	  case 10: printk("TS-8900\n"); usecodec = CODEC_SGTL5000; break;
   	  case 15: printk("TS-8380\n"); usecodec = CODEC_SGTL5000; break;      
   	  case 17: printk("TS-8920\n"); usecodec = CODEC_SGTL5000; break;
   	   
   	  case 11: printk("TS-8290\n"); usecodec = CODEC_SII9022; break;
   	   
   	  default: printk("Unknown\n");  return -EINVAL;
	}
	
	} else {
	   
	   printk("\nBaseboard ID overridden by usecodec parameter\n");
	   if (usecodec <= 0 || usecodec > CODEC_LAST) {
	       printk("Invalid usecodec parameter (only 1,2,3 allowed)\n");
	       return -EINVAL;
	   }
	}
	
	switch(usecodec) {
	case CODEC_WM8750:
	   ts4700_snd_devdata.codec_dev	   = &soc_codec_dev_wm8750;
	   ts4700_snd_devdata.codec_data	= &ts4700_wm8750_setup;
	   ts4700_dai.name = "wm8750";
	   ts4700_dai.stream_name  = "WM8750";  
	   ts4700_dai.codec_dai = &wm8750_dai; 
	   ts4700_dai.init      = 	ts4700_wm8750_init;	   	   	   
	   break;
	case CODEC_SGTL5000:
	   ts4700_snd_devdata.codec_dev = &soc_codec_dev_sgtl5000;
	   ts4700_snd_devdata.codec_data = &sgtl5000_platform_setup;	   
	   ts4700_dai.name = "sgtl5000-i2c";
	   ts4700_dai.stream_name   = "SGTL5000"; 
	   ts4700_dai.codec_dai = &sgtl5000_dai;
	   ts4700_dai.init      = ts4700_sgtl5000_init; 
	   break;
	case CODEC_SII9022:	   
	   ts4700_snd_devdata.codec_dev = &soc_codec_dev_sii9022;
	   ts4700_snd_devdata.codec_data = &sii9022_platform_setup;
	   ts4700_dai.name =  "sii9022-i2c";
	   ts4700_dai.stream_name   = "SII9022"; 
	   ts4700_dai.codec_dai = &sii9022_dai;
	   ts4700_dai.init      = ts4700_sii9022_init;
	   break;
	}

   ts4700_snd_devdata.card		= &snd_soc_machine_ts4700;
   
	ts4700_dai.cpu_dai   = &pxa3xx_ssp_dai[0];
	ts4700_dai.ops		= &ts4700_ops;
	   
	ts4700_snd_device = platform_device_alloc("soc-audio", -1);
	if (!ts4700_snd_device)
		return -ENOMEM;
	
	platform_set_drvdata(ts4700_snd_device, &ts4700_snd_devdata);
	ts4700_snd_devdata.dev = &ts4700_snd_device->dev;

	ret = platform_device_add(ts4700_snd_device);
	
	if (ret) {
	   printk("Failed to add ts4700_snd_device\n");
	   platform_device_put(ts4700_snd_device);
	   ts4700_snd_device = NULL;
	}

	return ret;
}

static void __exit ts4700_exit(void)
{
   dbg("%s\n", __func__);

   if (ts4700_snd_device)
      platform_device_unregister(ts4700_snd_device);
}


module_init(ts4700_init);
module_exit(ts4700_exit);

/* Module information */
MODULE_AUTHOR("Paul Shen, bshen9@marvell.com, www.marvell.com");
MODULE_LICENSE("GPL");
