/*
 * linux/sound/soc/pxa/tavorevb.c
 *
 * Copyright (C) 2007 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
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

#include "pxa3xx-ssp.h"
#include "pxa910-squ.h"
#include "pxa910-squ.h"



extern struct snd_soc_dai micco_dai[2];
extern struct snd_soc_codec_device soc_codec_dev_micco;



static int tavorevb_audio_init(struct snd_soc_codec *codec);

static int tavorevb_hifi_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *machine = rtd->dai;
	struct snd_soc_dai *cpu_dai = machine->cpu_dai;

	cpu_dai->playback.channels_min = 2;
	cpu_dai->playback.channels_max = 2;
	cpu_dai->playback.formats = SNDRV_PCM_FMTBIT_S16_LE;
	cpu_dai->playback.rates = 0xFFFFFFFF;

	return 0;
}

#define ISCCRx(x)   (* ((volatile unsigned long *) (APB_VIRT_BASE + 0x50040 + ((x)<< 2) )))
#define PMUM_VRCR   (* ((volatile unsigned long *) (APB_VIRT_BASE + 0x50018)))
static int tavorevb_hifi_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *machine = rtd->dai;
	struct snd_soc_dai *cpu_dai = machine->cpu_dai;
	struct ssp_device *ssp = cpu_dai->private_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned long rate = runtime->rate;
	u32 isccr_val, denom, nom, bitclk_div_468;

	bitclk_div_468 = 3;
	nom = 0x130b;
	denom = ((rate/1000)*0x130b*64*8)/156000;
	
	isccr_val = 0xe0000000 | (bitclk_div_468 << 27 ) |(denom << 15) | nom ;
	ISCCRx(1) = isccr_val;
	
	__raw_writel(0xe140003f, ssp->mmio_base + SSCR0);		
	__raw_writel(0x004001c2, ssp->mmio_base + SSCR1);
	__raw_writel(0x3, ssp->mmio_base + SSTSA);
	__raw_writel(0x31a00084, ssp->mmio_base + SSPSP);

	return 0;
}

static void tavorevb_hifi_shutdown(struct snd_pcm_substream *substream)
{
}



static struct snd_soc_ops tavorevb_machine_ops[] = {
{
	.startup = tavorevb_hifi_startup,
	.prepare = tavorevb_hifi_prepare,
	.shutdown = tavorevb_hifi_shutdown,
},
};


/* tavorevb digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link tavorevb_dai[] = {
{
	.name = "PXA910 I2S",
	.stream_name = "I2S HiFi",
	.cpu_dai = &pxa3xx_ssp_dai[1],
	.codec_dai = &micco_dai[0],
	.ops = &tavorevb_machine_ops[0],
	.init = tavorevb_audio_init,
},
};

/* tavorevb audio machine driver */
static struct snd_soc_machine tavorevb_snd_soc_machine = {
	.name = "tavorevb",
	.dai_link = tavorevb_dai,
	.num_links = ARRAY_SIZE(tavorevb_dai),
};

/* tavorevb audio subsystem */
static struct snd_soc_device tavorevb_snd_devdata = {
	.machine = &tavorevb_snd_soc_machine,
	.platform = &pxa910_soc_platform,
	.codec_dev = &soc_codec_dev_micco,
};

static struct platform_device *tavorevb_snd_device;

/*
 * Logic for a Micco as connected on a tavorevb Device
 */
static int tavorevb_audio_init(struct snd_soc_codec *codec)
{
	PMUM_VRCR = 0xff;
	return 0;
}

static int __init tavorevb_init(void)
{
	int ret;

	if (!machine_is_tavorevb())
		return -ENODEV;

	tavorevb_snd_device = platform_device_alloc("soc-audio", -1);
	if (!tavorevb_snd_device)
		return -ENOMEM;

	platform_set_drvdata(tavorevb_snd_device, &tavorevb_snd_devdata);
	tavorevb_snd_devdata.dev = &tavorevb_snd_device->dev;
	ret = platform_device_add(tavorevb_snd_device);

	if (ret)
		platform_device_put(tavorevb_snd_device);

	return ret;
}

static void __exit tavorevb_exit(void)
{
	platform_device_unregister(tavorevb_snd_device);
}

module_init(tavorevb_init);
module_exit(tavorevb_exit);

/* Module information */
MODULE_AUTHOR("bshen9@marvell.com");
MODULE_DESCRIPTION("ALSA SoC TAVOREVB");
MODULE_LICENSE("GPL");

