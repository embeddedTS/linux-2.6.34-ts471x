/*
 * aspenite.c  --  SoC audio for aspenite
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


#include "../codecs/wm8753.h"
#include "pxa3xx-pcm.h"
#include "pxa3xx-ssp.h"

#undef ASPENITE_WM8753_DEBUG
//#define ZYLONITEII_SSP_SYSFS

#ifdef ASPENITE_WM8753_DEBUG
#define dbg(format, arg...) \
	printk(KERN_INFO format "\n" , ## arg)
#else
	#define dbg(format, arg...) do {} while (0)
#endif

struct wm8753_setup_data {
	int spi;
	int i2c_bus;
	unsigned short i2c_address;
};

static struct snd_soc_card aspenite;

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
	{12288000,  0x659,    0x41,  3072000, 0x100,   0x40,   48000},
	{11289600,  0x6E9,    0x40,  2822400, 0x100,   0x40,   44100},
	{12288000,  0x659,    0x40,  2048000, 0x180,   0x40,   32000},
	{12288000,  0x659,    0x40,  1536000, 0x200,   0x40,   24000},
	{11289600,  0x6E9,    0x40,  1411200, 0x200,   0x40,   22050},
	{12288000,  0x659,    0x40,  1024000, 0x300,   0x40,   16000},
	{11289600,  0x6E9,    0x40,   705600, 0x400,   0x40,   11025},
	{12288000,  0x659,    0x40,   512000, 0x600,   0x40,   8000},
	{12288000,  0x659,    0x40,  6144000, 0x100,   0x80,   96000},
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


static int aspenite_wm8753_init(struct snd_soc_codec *codec)
{
	dbg(KERN_INFO "aspenite_wm8753_init\n");
	return 0;
}

static int aspenite_wm8753_hifi_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *machine = rtd->dai;
	struct snd_soc_dai *cpu_dai = machine->cpu_dai;
	struct ssp_device *ssp = cpu_dai->private_data;
	struct snd_soc_dai *codec_dai = machine->codec_dai;
	unsigned int format;
	dbg("aspenite_wm8753_startup\n");

	cpu_dai->playback.channels_min = 2;
	cpu_dai->playback.channels_max = 2;


	__raw_writel(0xE1C0003F, ssp->mmio_base + SSCR0);
	__raw_writel(0x00F01DC0, ssp->mmio_base + SSCR1);
	__raw_writel(0x3, ssp->mmio_base + SSTSA);
	__raw_writel(0x3, ssp->mmio_base + SSRSA);
	__raw_writel(0x40200004, ssp->mmio_base + SSPSP);


	format = SND_SOC_DAIFMT_LEFT_J | SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_NB_NF;
	codec_dai->ops->set_fmt(codec_dai, format);

	return 0;
}

static int aspenite_wm8753_hifi_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *machine = rtd->dai;
	struct snd_soc_dai *cpu_dai = machine->cpu_dai;
	struct ssp_device *ssp = cpu_dai->private_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned long rate = runtime->rate;
	int ceoff;
	dbg("aspenite_wm8753_hifi_prepare\n");
	
	__raw_writel(0xE1C0003F, ssp->mmio_base + SSCR0);
	__raw_writel(0x00F01DC0, ssp->mmio_base + SSCR1);
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

static void aspenite_wm8753_hifi_shutdown(struct snd_pcm_substream *substream)
{
	dbg("aspenite_wm8753_hifi_shutdown\n");
}

static int aspenite_wm8753_hifi_hw_params(
				struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *hw_params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	unsigned int clk = 0;
	int ret = 0;

	switch (params_rate(hw_params)) {
	case 8000:
	case 16000:
	case 24000:
	case 32000:
	case 48000:
	case 96000:
		clk = 12288000;
		break;
	case 11025:
	case 22050:
	case 44100:
	case 88200:
		clk = 11289600;
		break;
	}

	/* set the codec system clock for DAC and ADC */
	ret = snd_soc_dai_set_sysclk(codec_dai, WM8753_MCLK, clk,
			SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "Error Setting CODEC system MCLK\n");
	}
	return ret;
}

static int aspenite_wm8753_voice_startup(struct snd_pcm_substream *substream)
{
	dbg("aspenite_wm8753_voice_startup\n");
	return 0;
}


static int aspenite_wm8753_voice_prepare(struct snd_pcm_substream *substream)
{
	dbg("aspenite_wm8753_voice_prepare\n");
	return 0;
}

static void aspenite_wm8753_voice_shutdown(struct snd_pcm_substream *substream)
{
	dbg("aspenite_wm8753_voice_shutdown\n");
}

/* machine stream operations */
static struct snd_soc_ops aspenite_wm8753_machine_ops[] = {
{
	.startup = aspenite_wm8753_hifi_startup,
	.prepare = aspenite_wm8753_hifi_prepare,
	.shutdown = aspenite_wm8753_hifi_shutdown,
	.hw_params = aspenite_wm8753_hifi_hw_params,
},
{
	.startup = aspenite_wm8753_voice_startup,
	.prepare = aspenite_wm8753_voice_prepare,
	.shutdown = aspenite_wm8753_voice_shutdown,
},
};

static struct wm8753_setup_data wm8753_setup = {
	.i2c_address = 0x1b,
	.i2c_bus = 1,
};

static struct snd_soc_dai_link aspenite_dai[] = {
{
	.name = "WM8753",
	.stream_name = "WM8753 HiFi",
	.cpu_dai = &pxa3xx_ssp_dai[0],
	.codec_dai = &wm8753_dai[WM8753_DAI_HIFI],
	.ops = &aspenite_wm8753_machine_ops[0],
	.init = aspenite_wm8753_init,
},
{
	.name = "WM8753",
	.stream_name = "WM8753 Voice",
	.cpu_dai = &pxa3xx_ssp_dai[1],
	.codec_dai = &wm8753_dai[WM8753_DAI_VOICE],
	.ops = &aspenite_wm8753_machine_ops[1],
	.init = aspenite_wm8753_init,
},
};

static struct snd_soc_card aspenite = {
	.name = "ASPENITE WM8753",
	.platform = &pxa3xx_soc_platform,
	.dai_link = aspenite_dai,
	.num_links = ARRAY_SIZE(aspenite_dai),
};

static struct snd_soc_device aspenite_snd_devdata = {
	.card = &aspenite,
	.codec_dev = &soc_codec_dev_wm8753,
	.codec_data = &wm8753_setup,
};

static struct platform_device *aspenite_snd_device;

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
static int __init aspenite_init(void)
{
	int ret;
#if defined(ZYLONITEII_SSP_SYSFS)
	int err;
#endif
	struct i2c_board_info info;
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	struct wm8753_setup_data *setup = NULL;

	dbg(KERN_INFO "aspenite_init\n");
	if (!machine_is_aspenite())
		return -ENODEV;
	
	aspenite_snd_device = platform_device_alloc("soc-audio", -1);
	if (!aspenite_snd_device)
		return -ENOMEM;

	platform_set_drvdata(aspenite_snd_device, &aspenite_snd_devdata);
	aspenite_snd_devdata.dev = &aspenite_snd_device->dev;
	ret = platform_device_add(aspenite_snd_device);
	if (ret)
		platform_device_put(aspenite_snd_device);

#if defined(ZYLONITEII_SSP_SYSFS)
	printk("create sysfs node for ssp configuration\n");
	err = device_create_file(&aspenite_snd_device->dev, &dev_attr_asspdr_config);
	if (err)
	{
		printk(KERN_WARNING "asoc: failed to add asysdr sysfs entries\n");
	}

	err = device_create_file(&aspenite_snd_device->dev, &dev_attr_asysdr_config);
	if (err)
	{
		printk(KERN_WARNING "asoc: failed to add asspdr sysfs entries\n");
	}
#endif	

	setup = (struct wm8753_setup_data *)&wm8753_setup;

	memset(&info, 0, sizeof(struct i2c_board_info));
	info.addr = setup->i2c_address;
	strlcpy(info.type, "wm8753", I2C_NAME_SIZE);
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

static void __exit aspenite_exit(void)
{
	platform_device_unregister(aspenite_snd_device);
}

module_init(aspenite_init);
module_exit(aspenite_exit);

/* Module information */
MODULE_AUTHOR("Paul Shen, bshen9@marvell.com, www.marvell.com");
