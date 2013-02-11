/*
 * avlite.c  --  SoC audio for avlite
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

#include <linux/delay.h>
#include "../codecs/wm8960.h"
#include "pxa3xx-pcm.h"
#include "pxa3xx-ssp.h"
#include "../codecs/autopower.h"

#define AVLITE_WM8960_DEBUG
/* #define AVLITE_SSP_SYSFS */

#ifdef AVLITE_ALC5621_DEBUG
#define dbg(format, arg...) \
	printk(KERN_INFO format "\n" , ## arg)
#else
	#define dbg(format, arg...) do {} while (0)
#endif


static struct snd_soc_card avlite;

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
	{49152000,  0x659,    0x40,  3072000, 0x40,   0x10,     48000},
	{45151000,  0x6E9,    0x40,  2822000, 0x40,   0x10,     44100},
	{49152000,  0x659,    0x40,  2048000, 0x60,   0x10,     32000},
	{49152000,  0x659,    0x40,  1536000, 0x80,   0x10,     24000},
	{45151000,  0x6E9,    0x40,  1411000, 0x80,   0x10,     22050},
	{49152000,  0x659,    0x40,  1024000, 0xc0,   0x10,     16000},
	{45151000,  0x6E9,    0x40,   706500, 0x100,   0x10,     11025},
	{49152000,  0x659,    0x40,   512000, 0x180,   0x10,     8000},
	{49152000,  0x659,    0x40,  6144000, 0x40,   0x20,     96000},
};

struct wm8960_setup_data {
	unsigned short i2c_address;
	int i2c_bus;
};

static int ssp_conf_lookup(unsigned int rate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ssp_conf); i++) {
		if (ssp_conf[i].freq_out == rate) {
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
	int asysdr, asspdr;
	asysdr = ssp_conf[ceoff].sys_num << 16 | ssp_conf[ceoff].sys_den;
	ASYSDR = asysdr;
	asspdr = ssp_conf[ceoff].ssp_num << 16 | ssp_conf[ceoff].ssp_den;
	ASSPDR = asspdr;
}

static const struct snd_soc_dapm_widget avlite_dapm_widgets[] = {
	SND_SOC_DAPM_MIC("Onboard Microphone", NULL),
};

static const struct snd_soc_dapm_route audio_map[] = {
	/* Onboard Microphone */
	{ "LINPUT2", NULL, "MICB" },
	{ "MICB", NULL, "Onboard Microphone" },
};

static int avlite_wm8960_init(struct snd_soc_codec *codec)
{
	/* init ssp clock should be done
	 * before powering up codec.
	 */
	printk(KERN_NOTICE "***** INIT *****\n");
	ssp_set_clock(0);

	snd_soc_dapm_new_controls(codec, avlite_dapm_widgets,
				ARRAY_SIZE(avlite_dapm_widgets));

	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	/* setting for HP autodetection avlite specific */
	snd_soc_write(codec, WM8960_ADDCTL2 , 0x40);
	snd_soc_write(codec, WM8960_ADDCTL4 , 0x8);
	snd_soc_write(codec, WM8960_ADDCTL1 , 0x3);
	return 0;
}

static int avlite_wm8960_hifi_startup(struct snd_pcm_substream *substream)
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

	__raw_writel(0xE1C0003F, ssp->mmio_base + SSCR0);
	__raw_writel(0x00701DC0, ssp->mmio_base + SSCR1);
	__raw_writel(0x3, ssp->mmio_base + SSTSA);
	__raw_writel(0x3, ssp->mmio_base + SSRSA);
	__raw_writel(0x40200004, ssp->mmio_base + SSPSP);

	cpu_dai->playback.formats = SNDRV_PCM_FMTBIT_S16_LE;
	cpu_dai->capture.formats = SNDRV_PCM_FMTBIT_S16_LE;

	format = SND_SOC_DAIFMT_LEFT_J | SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_NB_NF;
	codec_dai->ops->set_fmt(codec_dai, format);

	return 0;
}

static int avlite_wm8960_hifi_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *machine = rtd->dai;
	struct snd_soc_dai *cpu_dai = machine->cpu_dai;
	struct ssp_device *ssp = cpu_dai->private_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned long rate = runtime->rate;
	int ceoff;

#if 0
	autopower_enable();
#endif

	__raw_writel(0xE1C0003F, ssp->mmio_base + SSCR0);
	__raw_writel(0x00701DC0, ssp->mmio_base + SSCR1);
	__raw_writel(0x3, ssp->mmio_base + SSTSA);
	__raw_writel(0x3, ssp->mmio_base + SSRSA);
	__raw_writel(0x40200004, ssp->mmio_base + SSPSP);

	ceoff = ssp_conf_lookup(rate);
	if (ceoff >= 0)
		ssp_set_clock(ceoff);
	else
		printk(KERN_ERR "Wrong audio sample rate\n");

	return 0;
}

static void avlite_wm8960_hifi_shutdown(struct snd_pcm_substream *substream)
{
}

static int avlite_wm8960_hifi_hw_params(struct snd_pcm_substream *substream)
{

	return 0;
}

static int avlite_wm8960_voice_startup(struct snd_pcm_substream *substream)
{
	return 0;
}


static int avlite_wm8960_voice_prepare(struct snd_pcm_substream *substream)
{
	return 0;
}

static void avlite_wm8960_voice_shutdown(struct snd_pcm_substream *substream)
{
}

/* machine stream operations */
static struct snd_soc_ops avlite_wm8960_machine_ops[] = {
{
	.startup = avlite_wm8960_hifi_startup,
	.prepare = avlite_wm8960_hifi_prepare,
	.shutdown = avlite_wm8960_hifi_shutdown,
	.hw_params = avlite_wm8960_hifi_hw_params,
},
{
	.startup = avlite_wm8960_voice_startup,
	.prepare = avlite_wm8960_voice_prepare,
	.shutdown = avlite_wm8960_voice_shutdown,
},
};

static struct snd_soc_dai_link avlite_dai[] = {
{
	.name = "WM8960",
	.stream_name = "WM8960 HiFi",
	.cpu_dai = &pxa3xx_ssp_dai[0],
	.codec_dai = &wm8960_dai,
	.ops = &avlite_wm8960_machine_ops[0],
	.init = avlite_wm8960_init,
},
};

static struct snd_soc_card avlite = {
	.name = "AVLITE WM8960",
	.platform = &pxa3xx_soc_platform,
	.dai_link = avlite_dai,
	.num_links = ARRAY_SIZE(avlite_dai),
};

static struct wm8960_setup_data wm8960_setup = {
	.i2c_address = 0x1a,
	.i2c_bus = 0,
};

static struct snd_soc_device avlite_snd_devdata = {
	.card = &avlite,
	.codec_dev = &soc_codec_dev_wm8960,
	.codec_data = &wm8960_setup,
};

static struct platform_device *avlite_snd_device;

#if defined(AVLITE_SSP_SYSFS)
static ssize_t asysdr_config_show(struct device *dev, struct device_attribute *attr, char *buf)
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

static ssize_t asspdr_config_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk("asspdr_config_show\n");

	printk("ASSPDR:\t0x%x\n", ASSPDR);

	return 0;
}

static ssize_t asspdr_config_store(struct device *dev, struct device_attribute *attr,
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

/* shut down codec chip */
#if 0
static void avlite_audio_shutdown(struct device *dev)
{
	printk(KERN_ERR "wm8960 power down\n");
	wm8960_powerdown(1);
}
#endif

static int __init avlite_init(void)
{
	int ret;
	int err;

	if (!machine_is_avengers_lite())
		return -ENODEV;

	/* ssp_set_clock(0); */
	avlite_snd_device = platform_device_alloc("soc-audio", -1);

	if (!avlite_snd_device)
		return -ENOMEM;

	platform_set_drvdata(avlite_snd_device, &avlite_snd_devdata);

	avlite_snd_devdata.dev = &avlite_snd_device->dev;
	ret = platform_device_add(avlite_snd_device);
	if (ret)
		platform_device_put(avlite_snd_device);

	/* device_driver's shutdown callback will
	 * be invoked automatically while system
	 * powers down
	 */
#warning AVlite codec powerdown code removed
#if 0
	avlite_snd_device->dev.driver->shutdown = &avlite_audio_shutdown;
#endif

#if defined(AVLITE_SSP_SYSFS)
	printk("create sysfs node for ssp configuration\n");
	err = device_create_file(&avlite_snd_device->dev, &dev_attr_asspdr_config);
	if (err) {
		printk(KERN_WARNING "asoc: failed to add asysdr sysfs entries\n");
	}
	err = device_create_file(&avlite_snd_device->dev, &dev_attr_asysdr_config);
	if (err) {
		printk(KERN_WARNING "asoc: failed to add asspdr sysfs entries\n");
	}
#endif

	struct i2c_board_info info;
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	struct wm8960_setup_data *setup = (struct wm8960_setup_data *)&wm8960_setup;
	memset(&info, 0, sizeof(struct i2c_board_info));
	info.addr = setup->i2c_address;
	strlcpy(info.type, "wm8960", I2C_NAME_SIZE);
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


static void __exit avlite_exit(void)
{
	platform_device_unregister(avlite_snd_device);
}


module_init(avlite_init);
module_exit(avlite_exit);

/* Module information */

