/*
 * ts4700.c  --  SoC audio for Technologic Systems' ts4700 board
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



#define SND_SOC_WM8750  0

#if (SND_SOC_WM8750)
#include "../codecs/wm8750.h"
#define CODEC_SYSCLK WM8750_SYSCLK
#else
#if defined(CONFIG_SND_SOC_SGTL5000) || defined(CONFIG_SND_SOC_SGTL5000_MODULE)
#include "../codecs/sgtl5000.h"
#define CODEC_SYSCLK  SGTL5000_SYSCLK
#define CODEC_LRCLK   SGTL5000_LRCLK
#else
#include "../codecs/sii9022.h"
#define CODEC_SYSCLK  SII9022_SYSCLK
#define CODEC_LRCLK   SII9022_LRCLK
#endif
#endif
#include "pxa3xx-pcm.h"
#include "pxa3xx-ssp.h"

//#define TS4700_CODEC_DEBUG

#ifdef TS4700_CODEC_DEBUG
#define dbg(format, arg...) \
	printk(KERN_INFO format "\n" , ## arg)
#else
	#define dbg(format, arg...) do {} while (0)
#endif


struct _ssp_conf {
	unsigned int main_clk;
	unsigned int sys_num;
	unsigned int sys_den;
	unsigned int bit_clk;
	unsigned int ssp_num;
	unsigned int ssp_den;
	unsigned int freq_out;
};

static const struct _ssp_conf ssp_conf[] = {
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
		{
			dbg("ssp_conf_lookup: %d\n", i);
			return i;
		}
	}
	return -EINVAL;
}

/*
 *    ASYSDR is the Audio SYSCLK Dithering Divider Register.
 *    ASSPDR is the SSPx_CLK Dithering Divider Register.
 *
 *    See Marvell documentation for details.
 */

#define ASYSDR   (* ((volatile unsigned long *) (APB_VIRT_BASE + 0x51050)))
#define ASSPDR   (* ((volatile unsigned long *) (APB_VIRT_BASE + 0x51054)))
#define MPMU_ACGR      (* ((volatile unsigned long *) (APB_VIRT_BASE + 0x51024)))

#define SSP1_CLK_RST   (* ((volatile unsigned long *) (APBC_VIRT_BASE + 0x81C)))


static void ssp_set_clock(unsigned int ceoff)
{
	unsigned long asysdr,asspdr;
			
	asysdr = (ssp_conf[ceoff].sys_num << 16) | ssp_conf[ceoff].sys_den;
	ASYSDR = asysdr;
	asspdr = (ssp_conf[ceoff].ssp_num << 16) | ssp_conf[ceoff].ssp_den;
	ASSPDR = asspdr;
		
}

#if (SND_SOC_WM8750)
static const struct snd_soc_dapm_route audio_map[] = {
	{ "Headphone Jack", NULL, "LOUT1" },
	{ "Headphone Jack", NULL, "ROUT1" },
	{ "Internal Speaker", NULL, "LOUT2" },
	{ "Internal Speaker", NULL, "ROUT2" },
	{ "LINPUT1", NULL, "Line Input" },
	{ "RINPUT1", NULL, "Line Input" },
};

static const struct snd_soc_dapm_widget ts4700_dapm_widgets[] = {
   SND_SOC_DAPM_HP("Headphone Jack", NULL),
   SND_SOC_DAPM_SPK("Internal Speaker", NULL),
   SND_SOC_DAPM_LINE("Line In", NULL),
};
#else
#if (0)
static const struct snd_soc_dapm_route audio_map[] = {
   {"MIC_IN", NULL, "Mic Jack"},
   {"LINE_IN", NULL, "Line In Jack"},
   {"Headphone Jack", NULL, "HP_OUT"},
   {"Ext Spk", NULL, "LINE_OUT"},
};

static const struct snd_soc_dapm_widget ts4700_dapm_widgets[] = {
   SND_SOC_DAPM_MIC("Mic Jack", NULL),
   SND_SOC_DAPM_LINE("Line In Jack", NULL),
   SND_SOC_DAPM_SPK("Ext Spk", NULL),
   SND_SOC_DAPM_HP("Headphone Jack", NULL),
};
#endif
#endif


static int ts4700_hw_params(struct snd_pcm_substream *substream,
			  struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	int ret = 0;
	u32 dai_format;
	unsigned int rate = params_rate(params);


	/* The clock value here is used by the codec driver to program the
	 * codec hardware.
	 */
	/* set the codec system clock for DAC and ADC */
	ret = snd_soc_dai_set_sysclk(codec_dai, CODEC_SYSCLK, 12500000,
				     SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;
	
	SSP1_CLK_RST = 0x43;      /* Select "Programmable SSPx_CLK" for SSP1 */	
	MPMU_ACGR |= (1 << 19);   /* Enable APMU_ASYSCLK in Clock Gating Register */
		
	snd_soc_dai_set_sysclk(codec_dai, CODEC_LRCLK, rate, 0);

	/* I2S fmt, codec bit clk & frm master, normal bit clock + frame */

#if defined(CONFIG_SND_SOC_SII9022) || defined(CONFIG_SND_SOC_SII9022_MODULE)   
	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_NB_NF;
#else
	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBM_CFM | SND_SOC_DAIFMT_NB_NF;
#endif

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

static int ts4700_wm8750_startup(struct snd_pcm_substream *substream)
{
  struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *machine = rtd->dai;
	struct snd_soc_dai *cpu_dai = machine->cpu_dai;
	struct ssp_device *ssp = cpu_dai->private_data;
	struct snd_soc_dai *codec_dai = machine->codec_dai;
	unsigned int format;
		
	cpu_dai->playback.channels_min = 2;
	cpu_dai->playback.channels_max = 2;
	 	
#if defined(CONFIG_SND_SOC_SII9022) || defined(CONFIG_SND_SOC_SII9022_MODULE)
   /* I2S fmt, codec is bit clk & frm slave, normal bit clock + frame */
	format =  SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_NB_NF;
   __raw_writel(0xE1D0003F, ssp->mmio_base + SSCR0);
	__raw_writel(0x10F01DC0, ssp->mmio_base + SSCR1);  
	__raw_writel(0x00F01DC0, ssp->mmio_base + SSCR1);	
	__raw_writel(0x42200000, ssp->mmio_base + SSPSP);
#else
   /* sgtl5000 clocks, when in slave mode, must be synchronous
	 * to SYS_MCLK.  We can't do that, because SYS_MCLK is a
	 * 12.5MHz signal provided by the FPGA on the TS-4700.
	 * So, we need the codec to be master, and the SoC to be slave.
	 */
	 
	/* I2S fmt, codec is bit clk & frm master, normal bit clock + frame */ 	
   format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBM_CFM | SND_SOC_DAIFMT_NB_NF;
#endif

   __raw_writel(0x3, ssp->mmio_base + SSTSA);
	__raw_writel(0x3, ssp->mmio_base + SSRSA);

	codec_dai->ops->set_fmt(codec_dai, format);

   return 0;
}

static int ts4700_wm8750_prepare(struct snd_pcm_substream *substream)
{
   struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *machine = rtd->dai;
	struct snd_soc_dai *cpu_dai = machine->cpu_dai;
	struct ssp_device *ssp = cpu_dai->private_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned long rate = runtime->rate;
	int ceoff;

	cpu_dai->playback.channels_min = 2;
   cpu_dai->playback.channels_max = 2;

#if defined(CONFIG_SND_SOC_SII9022) || defined(CONFIG_SND_SOC_SII9022_MODULE)   
   __raw_writel(0xE1D0003F, ssp->mmio_base + SSCR0);
	__raw_writel(0x10F01DC0, ssp->mmio_base + SSCR1);  
	__raw_writel(0x00F01DC0, ssp->mmio_base + SSCR1);	
	__raw_writel(0x42200000, ssp->mmio_base + SSPSP);
#endif

   __raw_writel(0x3, ssp->mmio_base + SSTSA);
	__raw_writel(0x3, ssp->mmio_base + SSRSA);

	ceoff = ssp_conf_lookup(rate);
	if (ceoff >= 0 )
	{
		ssp_set_clock(ceoff);
	} else {
		printk(KERN_ERR "Wrong audio sample rate\n");
	}


	/* we have to set the value of FRDC in SSCR0 here because the dai doesn't do it */
	__raw_writel(__raw_readl(ssp->mmio_base + SSCR0) | 0x01000000, 
	   ssp->mmio_base + SSCR0);

   return 0;
}

static void ts4700_wm8750_shutdown(struct snd_pcm_substream *substream)
{
   
}

static struct snd_soc_ops ts4700_ops = {
	.hw_params	= ts4700_hw_params,
   .startup = ts4700_wm8750_startup,
	.prepare = ts4700_wm8750_prepare,
	.shutdown = ts4700_wm8750_shutdown,

};

static int ts4700_wm8750_init(struct snd_soc_codec *codec)
{
	int err;
	struct clk *ssp_clk;


#if (SND_SOC_WM8750)
	/* These endpoints are not being used. */
	snd_soc_dapm_nc_pin(codec, "LINPUT2");
	snd_soc_dapm_nc_pin(codec, "RINPUT2");
	snd_soc_dapm_nc_pin(codec, "LINPUT3");
	snd_soc_dapm_nc_pin(codec, "RINPUT3");
	snd_soc_dapm_nc_pin(codec, "OUT3");
	snd_soc_dapm_nc_pin(codec, "MONO");

	/* Add ts4700 specific widgets */
	err = snd_soc_dapm_new_controls(codec, ts4700_dapm_widgets,
					ARRAY_SIZE(ts4700_dapm_widgets));
	if (err) {
		printk(KERN_ERR "%s: failed to add widgets (%d)\n",
		       __func__, err);
		return err;
	}

	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));
	snd_soc_dapm_sync(codec);
#endif

	return 0;
}


static struct snd_soc_dai_link ts4700_dai = {

#if (SND_SOC_WM8750)
   .name    = "wm8750",
   .stream_name   = "WM8750",

	.cpu_dai   = &pxa3xx_ssp_dai[0],
	.codec_dai = &wm8750_dai,
	.init      = ts4700_wm8750_init,
#else
#if defined(CONFIG_SND_SOC_SGTL5000) || defined(CONFIG_SND_SOC_SGTL5000_MODULE)
	.name      = "sgtl5000-i2c",
   .stream_name   = "SGTL5000",

    .cpu_dai   = &pxa3xx_ssp_dai[0],
	.codec_dai	= &sgtl5000_dai,
	.init      = ts4700_wm8750_init,      // TBD: fix this
#else
	.name      = "sii9022-i2c",
   .stream_name   = "SII9022",

   .cpu_dai   = &pxa3xx_ssp_dai[0],
	.codec_dai	= &sii9022_dai,
	.init      = ts4700_wm8750_init,      // TBD: fix this
#endif
#endif

	.ops		= &ts4700_ops,
};

/* ts4700 audio machine driver */
static struct snd_soc_card snd_soc_machine_ts4700 = {
	.name		= "ts4700",
	.platform	= &pxa3xx_soc_platform,
	.dai_link	= &ts4700_dai,
	.num_links	= 1,
};

/* ts4700 audio private data */
#if (SND_SOC_WM8750)
static struct wm8750_setup_data ts4700_wm8750_setup = {
   .i2c_address = 0x1a,
	.i2c_bus = 1,
   .spi = 0
};
#else
#if defined(CONFIG_SND_SOC_SGTL5000) || defined(CONFIG_SND_SOC_SGTL5000_MODULE)
static struct sgtl5000_platform_data sgtl5000_platform_setup = {
   .i2c_address = 0x0a,
   .i2c_bus = 1,
   .spi = 0
};
#else
static struct sii9022_setup_data sii9022_platform_setup = {
   .clock_enable = NULL
};
#endif
#endif

/* ts4700 audio subsystem */
static struct snd_soc_device ts4700_snd_devdata = {
	.card		= &snd_soc_machine_ts4700,
#if (SND_SOC_WM8750)
	.codec_dev	= &soc_codec_dev_wm8750,
	.codec_data	= &ts4700_wm8750_setup,
#else
#if defined(CONFIG_SND_SOC_SGTL5000) || defined(CONFIG_SND_SOC_SGTL5000_MODULE)
	.codec_dev = &soc_codec_dev_sgtl5000,	
	.codec_data = &sgtl5000_platform_setup,
#else
   .codec_dev = &soc_codec_dev_sii9022,	
	.codec_data = &sii9022_platform_setup,   
#endif	
#endif
};


static struct platform_device *ts4700_snd_device;

static int __init ts4700_init(void)
{
	int ret;

	printk("TS4700 SoC Audio support\n");


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
   if (ts4700_snd_device)
      platform_device_unregister(ts4700_snd_device);
}


module_init(ts4700_init);
module_exit(ts4700_exit);

/* Module information */
MODULE_AUTHOR("Paul Shen, bshen9@marvell.com, www.marvell.com");
MODULE_LICENSE("GPL");
