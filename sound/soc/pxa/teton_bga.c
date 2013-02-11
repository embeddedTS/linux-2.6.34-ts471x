/*
 * sound/soc/pxa/teton.c 
 *
 * Teton  --  SoC audio for Teton (Aspen based SoC)
 * 
 * Author:     Mark F. Brown <markb@marvell.com>
 * Created:    March 29th, 2009
 * Copyright:  (C) Copyright 2009 Marvell International Ltd.
 * 
 * 2009-03-29  ported from Marvell AE Aspenite Platform Code (aspen-cs4344.c)
 * 2010-02-10  ported to 2.6.29
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
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

#include "../codecs/cs4344.h"
#include "pxa3xx-pcm.h"
#include "pxa3xx-ssp.h"

#ifdef ASPEN_CS4344_DEBUG
#define dbg(format, arg...) \
	printk(KERN_INFO format "\n" , ## arg)
#else
	#define dbg(format, arg...) do {} while (0)
#endif

static struct snd_soc_card aspen_teton;

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
        {49152000,  0x659,    0x100,  3072000, 0x100,   0x10,     48000},
        {45151000,  0x6E9,    0x100,  2822000, 0x100,   0x10,     44100},
        {49152000,  0x659,    0x100,  2048000, 0x180,   0x10,     32000},
        {49152000,  0x659,    0x100,  1536000, 0x200,   0x10,     24000},
        {5644800,   0xDD,     0x3,    1411000, 0xC,     0x2,      22050},
        {4096000,   0x131,    0x3,    1024000, 0xC,     0x2,      16000},
        {2822400,   0x1BA,    0x3,    706500,  0xC,     0x2,      11025},
        {2048000,   0x261,    0x3,    512000,  0xC,     0x2,      8000},
        {49152000,  0x659,    0x100,  6144000, 0x100,   0x20,     96000},
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
	BUG_ON (ceoff >= ARRAY_SIZE(ssp_conf));
        asysdr = ssp_conf[ceoff].sys_num << 16 | ssp_conf[ceoff].sys_den;
        ASYSDR = asysdr;
        asspdr = ssp_conf[ceoff].ssp_num << 16 | ssp_conf[ceoff].ssp_den;
        ASSPDR = asspdr;
}


static int aspen_cs4344_init(struct snd_soc_codec *codec)
{
	dbg("aspen_cs4344_init\n");
	return 1;
}

static int teton_cs4344_startup(struct snd_pcm_substream *substream)
{
        struct snd_soc_pcm_runtime *rtd = substream->private_data;
        struct snd_soc_dai_link *machine = rtd->dai;
        struct snd_soc_dai *cpu_dai = machine->cpu_dai;
        struct ssp_device *ssp = cpu_dai->private_data;
        struct snd_soc_dai *codec_dai = machine->codec_dai;

	unsigned int format;
	dbg("teton_cs4344_startup\n");

	cpu_dai->playback.channels_min = 2;
        cpu_dai->playback.channels_max = 2;

        __raw_writel(0xe140003f, ssp->mmio_base + SSCR0); /* disable port */
        __raw_writel(0x6001c2, ssp->mmio_base + SSCR1);
        __raw_writel(0x3, ssp->mmio_base + SSTSA);
        __raw_writel(0x0, ssp->mmio_base + SSRSA);
        __raw_writel(0x31a00084, ssp->mmio_base + SSPSP);
//        __raw_writel(0x40200004, ssp->mmio_base + SSPSP);
        __raw_writel(0x0, ssp->mmio_base + SSACD);

        format = SND_SOC_DAIFMT_LEFT_J | SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_NB_NF;
        codec_dai->ops->set_fmt(codec_dai, format);

	return 0;
}

static int teton_cs4344_prepare(struct snd_pcm_substream *substream)
{
        struct snd_soc_pcm_runtime *rtd = substream->private_data;
        struct snd_soc_dai_link *machine = rtd->dai;
        struct snd_soc_dai *cpu_dai = machine->cpu_dai;
        struct ssp_device *ssp = cpu_dai->private_data;
        struct snd_pcm_runtime *runtime = substream->runtime;
        unsigned long rate = runtime->rate;
        int ceoff;

	dbg("teton_cs4344_prepare\r\n");

	__raw_writel(0xe140003f, ssp->mmio_base + SSCR0); /* disable port */
        __raw_writel(0x6001c2, ssp->mmio_base + SSCR1);
        __raw_writel(0x3, ssp->mmio_base + SSTSA);
        __raw_writel(0x0, ssp->mmio_base + SSRSA);

	if (rate >= 24000) /* Teton BGA <22Khz sampling fix */
		__raw_writel(0x31a00084, ssp->mmio_base + SSPSP);
	else
		__raw_writel(0x2000004, ssp->mmio_base + SSPSP);

        __raw_writel(0x0, ssp->mmio_base + SSACD);

        ceoff = ssp_conf_lookup(rate);
        if (ceoff >= 0 )
        {
                ssp_set_clock(ceoff);
        } else {
                printk(KERN_ERR "Wrong audio sample rate\n");
        }

	return 0;
}

static int teton_cs4344_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *hw_params)
{
	dbg("teton_cs4344_hw_params\r\n");
        return 0;
}

static void teton_cs4344_shutdown(struct snd_pcm_substream *substream)
{
	dbg("teton_cs4344_shutdown\r\n");
}

/* machine stream operations */
static struct snd_soc_ops aspen_cs4344_machine_ops = {
	
	.startup = teton_cs4344_startup,
	.prepare = teton_cs4344_prepare,
	.shutdown = teton_cs4344_shutdown,
	.hw_params = teton_cs4344_hw_params,
};

static struct snd_soc_dai_link aspen_teton_dai = {
	.name = "CS4344",
	.stream_name = "CS4344 HiFi",
	.cpu_dai =  &pxa3xx_ssp_dai[0],
	.codec_dai = &cs4344_dai,
	.ops = &aspen_cs4344_machine_ops,
	.init = aspen_cs4344_init,
};

static struct snd_soc_card aspen_teton = {
	.name = "CS4344",
	.platform = &pxa3xx_soc_platform,
	.dai_link = &aspen_teton_dai,
	.num_links = 1,
};

static struct snd_soc_device aspen_teton_snd_devdata = {
	.card = &aspen_teton,
	.codec_dev = &soc_codec_dev_cs4344,
	.codec_data = NULL,
};

static struct platform_device *aspen_teton_snd_device;

static int __init teton_cs4344_init(void)
{
	int ret;

	dbg(KERN_INFO "teton_cs4344_init\n");
	if (!machine_is_teton_bga() && !machine_is_teton())
	{
		return -ENODEV;
	}

	aspen_teton_snd_device = platform_device_alloc("soc-audio", -1);

	if (!aspen_teton_snd_device)
		return -ENOMEM;

	platform_set_drvdata(aspen_teton_snd_device, &aspen_teton_snd_devdata);
	aspen_teton_snd_devdata.dev = &aspen_teton_snd_device->dev;
	ret = platform_device_add(aspen_teton_snd_device);

	dbg(KERN_INFO "plat dev = %p\n", &aspen_teton_snd_devdata);

	if (ret)
	{
		platform_device_put(aspen_teton_snd_device);
	}
	dbg("%s: ret=%d\n", __FUNCTION__, ret);
	return ret;
}

static void __exit teton_cs4344_exit(void)
{
	platform_device_unregister(aspen_teton_snd_device);
}

module_init(teton_cs4344_init);
module_exit(teton_cs4344_exit);

/* Module information */
MODULE_AUTHOR("Mark Brown");
MODULE_DESCRIPTION("Teton CS4344 SOC Audio");
MODULE_LICENSE("GPL");
