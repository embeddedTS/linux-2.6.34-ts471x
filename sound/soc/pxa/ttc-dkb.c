/*
 * linux/sound/soc/pxa/ttc_dkb.c
 *
 * Copyright (C) 2007 Marvell International Ltd.
 * Author: Bin Yang <bin.yang@marvell.com> 
 * 			 Yael Sheli Chemla<yael.s.shemla@marvell.com> 
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
#include <linux/delay.h>
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

extern struct snd_soc_dai sanremo_audio_dai[2];
extern struct snd_soc_codec_device soc_codec_dev_sanremo_audio;


#define SANREMO_AUDIO_HIFI_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
		                SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |\
				SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000)

static int ttc_sanremo_init(struct snd_soc_codec *codec);

static int ttc_sanremo_hifi_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *machine = rtd->dai;
	struct snd_soc_dai *cpu_dai = machine->cpu_dai;

	//set_dvfm_constraint();

	cpu_dai->playback.channels_min = 2;
	cpu_dai->playback.channels_max = 2;
	cpu_dai->capture.channels_min = 2;
	cpu_dai->capture.channels_max = 2;
	cpu_dai->playback.formats = SNDRV_PCM_FMTBIT_S16_LE;
	cpu_dai->capture.formats = SNDRV_PCM_FMTBIT_S16_LE;

	cpu_dai->playback.rates = SNDRV_PCM_RATE_48000;
	cpu_dai->capture.rates = SNDRV_PCM_RATE_48000;/*sanremo has sample rate switching issue, workaround for fixed sample rate*/

	return 0;
}

#define ISCCRx(x)   (* ((volatile unsigned long *) (APB_VIRT_BASE + 0x50040 + ((x)<< 2) )))
#define VCXO_REQ_MFP   (* ((volatile unsigned long *) (APB_VIRT_BASE + 0x1e0d4)))
#define PMUM_VRCR   (* ((volatile unsigned long *) (APB_VIRT_BASE + 0x50018)))
static int ttc_sanremo_hifi_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *machine = rtd->dai;
	struct snd_soc_dai *cpu_dai = machine->cpu_dai;
	struct ssp_device *ssp = cpu_dai->private_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned long rate = runtime->rate;

	__raw_writel(0x41D0003F, ssp->mmio_base + SSCR0);
	__raw_writel(0x03B01DC0, ssp->mmio_base + SSCR1);
	__raw_writel(0x02100004, ssp->mmio_base + SSPSP);
	__raw_writel(0x3, ssp->mmio_base + SSTSA);
	__raw_writel(0x3, ssp->mmio_base + SSRSA);

	return 0;
}

static void ttc_sanremo_hifi_shutdown(struct snd_pcm_substream *substream)
{
	//unset_dvfm_constraint();
}



static struct snd_soc_ops ttc_sanremo_machine_ops[] = {
{
	.startup = ttc_sanremo_hifi_startup,
	.prepare = ttc_sanremo_hifi_prepare,
	.shutdown = ttc_sanremo_hifi_shutdown,
},
};


/* tavorevb digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link ttc_sanremo_dai[] = {
{
	.name = "PXA910 I2S",
	.stream_name = "I2S HiFi",
	.cpu_dai = &pxa3xx_ssp_dai[1],
	.codec_dai = &sanremo_audio_dai[0],
	.ops = &ttc_sanremo_machine_ops[0],
	.init = ttc_sanremo_init,
},
};

/* tavorevb audio machine driver */
static struct snd_soc_card ttc_sanremo_snd_soc_machine = {
	.name = "ttc-dkb",
	.platform = &pxa910_soc_platform,
	.dai_link = ttc_sanremo_dai,
	.num_links = ARRAY_SIZE(ttc_sanremo_dai),
};

/* tavorevb audio subsystem */
static struct snd_soc_device ttc_sanremo_snd_devdata = {
	.card = &ttc_sanremo_snd_soc_machine,
	.codec_dev = &soc_codec_dev_sanremo_audio,
};

static struct platform_device *ttc_sanremo_snd_device;

/*
 * Logic for a Micco as connected on a tavorevb Device
 */
static int ttc_sanremo_init(struct snd_soc_codec *codec)
{
	VCXO_REQ_MFP = 0xd0c0;
	PMUM_VRCR = 0x1;
	return 0;
}

static int __init ttc_dkb_init(void)
{
	int ret;

	if (!(machine_is_ttc_dkb() || machine_is_dkb_generic()))
		return -ENODEV;

	ttc_sanremo_snd_device = platform_device_alloc("soc-audio", -1);
	if (!ttc_sanremo_snd_device)
		return -ENOMEM;

	platform_set_drvdata(ttc_sanremo_snd_device, &ttc_sanremo_snd_devdata);
	ttc_sanremo_snd_devdata.dev = &ttc_sanremo_snd_device->dev;
	ret = platform_device_add(ttc_sanremo_snd_device);

	if (ret)
		platform_device_put(ttc_sanremo_snd_device);

	return ret;
}

static void __exit ttc_dkb_exit(void)
{
	platform_device_unregister(ttc_sanremo_snd_device);
}

module_init(ttc_dkb_init);
module_exit(ttc_dkb_exit);

/* Module information */
MODULE_AUTHOR("bshen9@marvell.com");
MODULE_DESCRIPTION("ALSA SoC TTC DKB");
MODULE_LICENSE("GPL");

