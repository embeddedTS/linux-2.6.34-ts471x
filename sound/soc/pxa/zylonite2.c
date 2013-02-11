/*
 * zyloniteii.c  --  SoC audio for zyloniteii
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
#include <sound/driver.h>
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


#include "../codecs/wm8753.h"
#include "pxa3xx-pcm.h"
#include "pxa3xx-ssp.h"

#undef ZYLONITEII_WM8753_DEBUG
//#define ZYLONITEII_SSP_SYSFS

#ifdef ZYLONITEII_WM8753_DEBUG
#define dbg(format, arg...) \
	printk(KERN_INFO format "\n" , ## arg)
#else
	#define dbg(format, arg...) do {} while (0)
#endif


static struct snd_soc_machine zyloniteii;

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


#define ASYSDR   (* ((volatile unsigned long *) (APB_VIRT_BASE + 0x51050)))
#define ASSPDR   (* ((volatile unsigned long *) (APB_VIRT_BASE + 0x51054)))
static int ssp_set_clock(unsigned int ceoff)
{
	int asysdr,asspdr;
	asysdr = ssp_conf[ceoff].sys_num << 16 | ssp_conf[ceoff].sys_den;
	ASYSDR = asysdr;
	asspdr = ssp_conf[ceoff].ssp_num << 16 | ssp_conf[ceoff].ssp_den;
	ASSPDR = asspdr;
}


static int zyloniteii_wm8753_init(struct snd_soc_codec *codec)
{
	dbg(KERN_INFO "zyloniteii_wm8753_init\n");
	return 0;
}

static int zyloniteii_wm8753_hifi_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *machine = rtd->dai;
	struct snd_soc_dai *cpu_dai = machine->cpu_dai;
	struct ssp_device *ssp = cpu_dai->private_data;
	struct snd_soc_dai *codec_dai = machine->codec_dai;
	unsigned int format;
	dbg("zyloniteii_wm8753_startup\n");

	cpu_dai->playback.channels_min = 2;
	cpu_dai->playback.channels_max = 2;


	__raw_writel(0xE1C0003F, ssp->mmio_base + SSCR0);
	__raw_writel(0x00701DC0, ssp->mmio_base + SSCR1);
	__raw_writel(0x3, ssp->mmio_base + SSTSA);
	__raw_writel(0x3, ssp->mmio_base + SSRSA);
	__raw_writel(0x40200004, ssp->mmio_base + SSPSP);


	format = SND_SOC_DAIFMT_LEFT_J | SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_NB_NF;
	codec_dai->dai_ops.set_fmt(codec_dai, format);

	return 0;
}

static int zyloniteii_wm8753_hifi_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *machine = rtd->dai;
	struct snd_soc_dai *cpu_dai = machine->cpu_dai;
	struct ssp_device *ssp = cpu_dai->private_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned long rate = runtime->rate;
	int ceoff;
	dbg("zyloniteii_wm8753_hifi_prepare\n");
	
	__raw_writel(0xE1C0003F, ssp->mmio_base + SSCR0);
	__raw_writel(0x00701DC0, ssp->mmio_base + SSCR1);
	__raw_writel(0x3, ssp->mmio_base + SSTSA);
	__raw_writel(0x3, ssp->mmio_base + SSRSA);
	__raw_writel(0x40200004, ssp->mmio_base + SSPSP);
	
	ceoff = ssp_conf_lookup(rate);
	if (ceoff >= 0 )
	{
		ssp_set_clock(ceoff);
	} else {
		printk(KERN_ERR "Wrong audio sample rate\n");
	}


	return 0;
}

static int zyloniteii_wm8753_hifi_shutdown(struct snd_pcm_substream *substream)
{
	dbg("zyloniteii_wm8753_hifi_shutdown\n");
	return 0;
}

static int zyloniteii_wm8753_hifi_hw_params(struct snd_pcm_substream *substream)
{

	return 0;
}

static int zyloniteii_wm8753_voice_startup(struct snd_pcm_substream *substream)
{
	dbg("zyloniteii_wm8753_voice_startup\n");
	return 0;
}


static int zyloniteii_wm8753_voice_prepare(struct snd_pcm_substream *substream)
{
	dbg("zyloniteii_wm8753_voice_prepare\n");
	return 0;
}

static int zyloniteii_wm8753_voice_shutdown(struct snd_pcm_substream *substream)
{
	dbg("zyloniteii_wm8753_voice_shutdown\n");
	return 0;
}

/* machine stream operations */
static struct snd_soc_ops zyloniteii_wm8753_machine_ops[] = {
{
	.startup = zyloniteii_wm8753_hifi_startup,
	.prepare = zyloniteii_wm8753_hifi_prepare,
	.shutdown = zyloniteii_wm8753_hifi_shutdown,
	.hw_params = zyloniteii_wm8753_hifi_hw_params,
},
{
	.startup = zyloniteii_wm8753_voice_startup,
	.prepare = zyloniteii_wm8753_voice_prepare,
	.shutdown = zyloniteii_wm8753_voice_shutdown,
},
};

static struct snd_soc_dai_link zyloniteii_dai[] = {
{
	.name = "WM8753",
	.stream_name = "WM8753 HiFi",
	.cpu_dai = &pxa3xx_ssp_dai[0],
	.codec_dai = &wm8753_dai[WM8753_DAI_HIFI],
	.ops = &zyloniteii_wm8753_machine_ops[0],
	.init = zyloniteii_wm8753_init,
},
{
	.name = "WM8753",
	.stream_name = "WM8753 Voice",
	.cpu_dai = &pxa3xx_ssp_dai[1],
	.codec_dai = &wm8753_dai[WM8753_DAI_VOICE],
	.ops = &zyloniteii_wm8753_machine_ops[1],
	.init = zyloniteii_wm8753_init,
},
};

static struct snd_soc_machine zyloniteii = {
	.name = "ZYLONITEII WM8753",
	.dai_link = zyloniteii_dai,
	.num_links = ARRAY_SIZE(zyloniteii_dai),
};

static struct wm8753_setup_data wm8753_setup = {
	.i2c_address = 0x1b,
	.i2c_bus = 1,
};

static struct snd_soc_device zyloniteii_snd_devdata = {
	.machine = &zyloniteii,
	.platform = &pxa3xx_soc_platform,
	.codec_dev = &soc_codec_dev_wm8753,
	.codec_data = &wm8753_setup,
};

static struct platform_device *zyloniteii_snd_device;

#if defined(ZYLONITEII_SSP_SYSFS)
static ssize_t asysdr_config_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	printk("asysdr_config_show\n");
	printk("ASYSDR:\t0x%x\n", ASYSDR);
	return 0;
}

static ssize_t asysdr_config_store(struct device *dev,struct device_attribute *attr,
		const char *buf, size_t count)
{
	int value;
	printk("asysdr_config_store: please use hexadecimal\n");
	
	value = simple_strtoul(buf, NULL, 16);
	printk("value = %x\n", value);
	
	ASYSDR = value;
	return value;
}
static ssize_t asspdr_config_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	printk("asspdr_config_show\n");
	printk("ASSPDR:\t0x%x\n", ASSPDR);
	return 0;
}
static ssize_t asspdr_config_store(struct device *dev,struct device_attribute *attr,
		const char *buf, size_t count)
{
	int value;
	printk("asysdr_config_store: please use hexadecimal\n");
	
	value = simple_strtoul(buf, NULL, 16);
	printk("value = %x\n", value);
	
	ASSPDR = value;
	return value;
}

static DEVICE_ATTR(asspdr_config, 0755, asspdr_config_show, asspdr_config_store);
static DEVICE_ATTR(asysdr_config, 0755, asysdr_config_show, asysdr_config_store);
#endif
static int __init zyloniteii_init(void)
{
	int ret;
	int err;

	dbg(KERN_INFO "zyloniteii_init\n");
	if (!machine_is_zylonite2())
		return -ENODEV;
	
	zyloniteii_snd_device = platform_device_alloc("soc-audio", -1);
	if (!zyloniteii_snd_device)
		return -ENOMEM;

	platform_set_drvdata(zyloniteii_snd_device, &zyloniteii_snd_devdata);
	zyloniteii_snd_devdata.dev = &zyloniteii_snd_device->dev;
	ret = platform_device_add(zyloniteii_snd_device);
	if (ret)
		platform_device_put(zyloniteii_snd_device);

#if defined(ZYLONITEII_SSP_SYSFS)
	printk("create sysfs node for ssp configuration\n");
	err = device_create_file(&zyloniteii_snd_device->dev, &dev_attr_asspdr_config);
	if (err)
	{
		printk(KERN_WARNING "asoc: failed to add asysdr sysfs entries\n");
	}

	err = device_create_file(&zyloniteii_snd_device->dev, &dev_attr_asysdr_config);
	if (err)
	{
		printk(KERN_WARNING "asoc: failed to add asspdr sysfs entries\n");
	}
#endif	
	return ret;
}

static void __exit zyloniteii_exit(void)
{
	platform_device_unregister(zyloniteii_snd_device);
}

module_init(zyloniteii_init);
module_exit(zyloniteii_exit);

/* Module information */
MODULE_AUTHOR("Paul Shen, bshen9@marvell.com, www.marvell.com");
