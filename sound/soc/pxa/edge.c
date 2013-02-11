/*
 * edge.c  --  SoC audio for edge
 *
 * Copyright 2005 Wolfson Microelectronics PLC.
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
 *    30th Oct 2005   Initial version.
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

#include <asm/mach-types.h>
#include <asm/io.h>

#include <asm/uaccess.h>
#include <plat/regs-ssp.h>
#include <plat/ssp.h>
#include <mach/addr-map.h>


#include "../codecs/wm8978.h"
#include "pxa3xx-pcm.h"
#include "pxa3xx-ssp.h"

#undef ASPENITE_WM8978_DEBUG
//#define ZYLONITEII_SSP_SYSFS

#ifdef ASPENITE_WM8978_DEBUG
#define dbg(format, arg...) \
	printk(KERN_INFO format "\n" , ## arg)
#else
	#define dbg(format, arg...) do {} while (0)
#endif


struct wm8978_setup_data {
	int spi;
	int i2c_bus;
	unsigned short i2c_address;
};

static struct snd_soc_card edge;

struct _ssp_conf {
	unsigned int main_clk;
	unsigned int sys_num;
	unsigned int sys_den;
	unsigned int bit_clk;
	unsigned int ssp_num;
	unsigned int ssp_den;
	unsigned int freq_out;
};
/*
 * 312M * 0x40 / 0x659 = 12288000
 * 12288000 * 0x20 / 0x100 = 1536000
 * 1536000 / 32 = 48000
*/
static const struct _ssp_conf ssp_conf[] = {
	/*main_clk, sys_num, sys_den, bit_clk, ssp_num, ssp_den, freq_out*/
	{12288000,  0x659,    0x40,  1536000, 0x100,   0x20,	48000},
	{11287750,  0x6E9,    0x40,  1411200, 0x100,   0x20,	44100},
	{12288000,  0x659,    0x40,  1024000, 0x180,   0x20,	32000},
	{12288000,  0x659,    0x40,  768000, 0x200,   0x20,	24000},
	{11287750,  0x6E9,    0x40,  705600, 0x200,   0x20,	22050},
	{12288000,  0x659,    0x40,  512000, 0x300,   0x20,	16000},
	{11287750,  0x6E9,    0x40,   352800, 0x400,   0x20,	11025},
	{12288000,  0x659,    0x40,   256000, 0x600,   0x20,	8000},
	{12288000,  0x659,    0x40,  3072000, 0x100,   0x40,	96000},
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


#define ASYSDR   (* ((volatile unsigned long *) (APB_VIRT_BASE + 0x51050)))
#define ASSPDR   (* ((volatile unsigned long *) (APB_VIRT_BASE + 0x51054)))
static void ssp_set_clock(unsigned int ceoff)
{
	int asysdr,asspdr;
	asysdr = ssp_conf[ceoff].sys_num << 16 | ssp_conf[ceoff].sys_den;
	ASYSDR = asysdr;
	asspdr = ssp_conf[ceoff].ssp_num << 16 | ssp_conf[ceoff].ssp_den;
	ASSPDR = asspdr;
}


static int edge_wm8978_init(struct snd_soc_codec *codec)
{
	dbg(KERN_INFO "edge_wm8978_init\n");
	return 0;
}

static int edge_wm8978_hifi_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *machine = rtd->dai;
	struct snd_soc_dai *cpu_dai = machine->cpu_dai;
	struct ssp_device *ssp = cpu_dai->private_data;
	struct snd_soc_dai *codec_dai = machine->codec_dai;
	unsigned int format;

	cpu_dai->playback.channels_min = 2;
	cpu_dai->playback.channels_max = 2;
	cpu_dai->capture.channels_min = 2;
	cpu_dai->capture.channels_max = 2;
        cpu_dai->playback.formats = SNDRV_PCM_FMTBIT_S16_LE;
        cpu_dai->capture.formats = SNDRV_PCM_FMTBIT_S16_LE;

	__raw_writel(0x41d0003f, ssp->mmio_base + SSCR0);
	__raw_writel(0x00B01DC0, ssp->mmio_base + SSCR1);
	__raw_writel(0x02100004, ssp->mmio_base + SSPSP);

	format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_NB_NF;
	codec_dai->ops->set_fmt(codec_dai, format);

	return 0;
}

static int edge_wm8978_hifi_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *machine = rtd->dai;
	struct snd_soc_dai *cpu_dai = machine->cpu_dai;
	struct ssp_device *ssp = cpu_dai->private_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned long rate = runtime->rate;
	int ceoff;
	dbg("edge_wm8978_hifi_prepare\n");

	__raw_writel(0x41d0003f, ssp->mmio_base + SSCR0);
	__raw_writel(0x00B01DC0, ssp->mmio_base + SSCR1);
	__raw_writel(0x02100004, ssp->mmio_base + SSPSP);

	ceoff = ssp_conf_lookup(rate);
	if (ceoff >= 0 )
	{
		ssp_set_clock(ceoff);
	} else {
		printk(KERN_ERR "Wrong audio sample rate\n");
	}


	return 0;
}

static void edge_wm8978_hifi_shutdown(struct snd_pcm_substream *substream)
{
	dbg("edge_wm8978_hifi_shutdown\n");
}

#define CODEC_PROVIDES_PLL_CLOCK
#ifdef CODEC_PROVIDES_PLL_CLOCK

static int edge_wm8978_hifi_hw_params(struct snd_pcm_substream *substream,
					 struct snd_pcm_hw_params *hw_params)
{
	int ret;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	unsigned rate = 256 * hw_params->rate_num / hw_params->rate_den;
	unsigned mclk = ((rate % 8000) == 0) ? 12288000 : 11289600;

	ret = snd_soc_dai_set_pll(codec_dai, 0, mclk, rate);

	if (ret < 0) {
		printk(KERN_ERR "can't set codec system clock. ret = %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_clkdiv(codec_dai, WM8978_MCLKSEL, WM8978_MCLK_PLL); /* enable the pll */
	if (ret < 0) {
		printk(KERN_ERR "can't set codec clksel. ret = %d\n", ret);
		return ret;
	}
	return 0;
}

#else
static int edge_wm8978_hifi_hw_params(struct snd_pcm_substream *substream,
					 struct snd_pcm_hw_params *hw_params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	int ret;
	unsigned int mclk_div = 0;

	/*
	 * WM8978 SYSCLK = 256 sample rate,
	 * SYSCLK = MCLK / mclk_div
	 * MCLK = 12288000(12K serious), 11287750 (11k serious)
	 * 48000 * 256 = 12288000, div = 1
	 */
	switch (params_rate(hw_params)) {
	case 48000:
		mclk_div = WM8978_MCLK_DIV_1;
		break;

	case 32000:
		mclk_div = WM8978_MCLK_DIV_1_5;
		break;

	case 44100:
		mclk_div = WM8978_MCLK_DIV_1;
		break;

	case 22050:
		mclk_div = WM8978_MCLK_DIV_2;
		break;

	case 16000:
		mclk_div = WM8978_MCLK_DIV_3;
		break;

	case 11025:
		mclk_div = WM8978_MCLK_DIV_4;
		break;

	case 8000:
		mclk_div = WM8978_MCLK_DIV_6;
		break;

	default:
		printk(KERN_ERR "Wrong audio sample rate\n");
		return -EINVAL;
	}

	ret = snd_soc_dai_set_clkdiv(codec_dai, WM8978_MCLKDIV, mclk_div);
	if (ret < 0) {
		pr_warning("playpaq_wm8978: Failed to set CODEC MCLKDIV (%d)\n", ret);
		return ret;
	}

	return 0;
}

#endif

/* machine stream operations */
static struct snd_soc_ops edge_wm8978_machine_ops[] = {
{
	.startup = edge_wm8978_hifi_startup,
	.prepare = edge_wm8978_hifi_prepare,
	.shutdown = edge_wm8978_hifi_shutdown,
	.hw_params = edge_wm8978_hifi_hw_params,
},
};

static struct snd_soc_dai_link edge_dai[] = {
{
	.name = "WM8978",
	.stream_name = "WM8978 HiFi",
	.cpu_dai = &pxa3xx_ssp_dai[0],
	.codec_dai = &wm8978_dai,
	.ops = &edge_wm8978_machine_ops[0],
	.init = edge_wm8978_init,
},
};

static struct snd_soc_card edge = {
	.name = "ASPENITE WM8978",
	.platform = &pxa3xx_soc_platform,
	.dai_link = edge_dai,
	.num_links = ARRAY_SIZE(edge_dai),
};

static struct wm8978_setup_data wm8978_setup = {
	.i2c_address = 0x1a,
	.i2c_bus = 0,
};

static struct snd_soc_device edge_snd_devdata = {
	.card = &edge,
	.codec_dev = &soc_codec_dev_wm8978,
	.codec_data = &wm8978_setup,
};

static struct platform_device *edge_snd_device;

static int __init edge_init(void)
{
	int ret;

	dbg(KERN_INFO "edge_init\n");
	if (!machine_is_edge())
		return -ENODEV;

	edge_snd_device = platform_device_alloc("soc-audio", -1);
	if (!edge_snd_device)
		return -ENOMEM;

	platform_set_drvdata(edge_snd_device, &edge_snd_devdata);
	edge_snd_devdata.dev = &edge_snd_device->dev;
	ret = platform_device_add(edge_snd_device);
	if (ret)
		platform_device_put(edge_snd_device);

	struct i2c_board_info info;
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	struct wm8978_setup_data *setup = (struct wm8978_setup_data *)&wm8978_setup;
	memset(&info, 0, sizeof(struct i2c_board_info));
	info.addr = setup->i2c_address;
	strlcpy(info.type, "wm8978", I2C_NAME_SIZE);
	adapter = i2c_get_adapter(setup->i2c_bus);
	if (!adapter) {
		printk(KERN_ERR "can't get i2c adapter %d\n",setup->i2c_bus);
		return -ENODEV;
	}

	client = i2c_new_device(adapter, &info);
	i2c_put_adapter(adapter);
	if (!client) {
		printk(KERN_ERR "can't add i2c device at 0x%x\n",(unsigned int)info.addr);
		return -ENODEV;
	}

	return ret;
}

static void __exit edge_exit(void)
{
	platform_device_unregister(edge_snd_device);
}

module_init(edge_init);
module_exit(edge_exit);

/* Module information */
MODULE_AUTHOR("Paul Shen, bshen9@marvell.com, www.marvell.com");
