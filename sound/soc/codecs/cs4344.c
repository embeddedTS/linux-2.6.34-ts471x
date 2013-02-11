/*
 * sound/soc/codecs/cs4344.c
 * 
 * CS4344  --  SoC audio driver
 * 
 * Author:     Mark F. Brown <markb@marvell.com>
 * Created:    March 29th, 2009
 * Copyright:  (C) Copyright 2009 Marvell International Ltd.
 *
 * 2009-03-29  ported from Marvell AE Aspenite Platform Code (cs4344.c)
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <linux/slab.h>

#define AUDIO_NAME "CS4344"

#ifdef CS4344_DEBUG
#define dbg(format, arg...) \
	printk(KERN_DEBUG AUDIO_NAME ": " format "\n" , ## arg)
#else
#define dbg(format, arg...) do {} while (0)
#endif
#define err(format, arg...) \
	printk(KERN_ERR AUDIO_NAME ": " format "\n" , ## arg)
#define info(format, arg...) \
	printk(KERN_INFO AUDIO_NAME ": " format "\n" , ## arg)
#define warn(format, arg...) \
	printk(KERN_WARNING AUDIO_NAME ": " format "\n" , ## arg)

/* There are no software controls for DAC so they need to be faked */

#define CS4344_DUMMY_CTRL     0x00    /* DAC Channel Dummy Control */

static struct snd_soc_codec *cs4344_codec = NULL;

struct snd_kcontrol_new cs4344_snd_controls[] = {
        SOC_SINGLE("Control",
                CS4344_DUMMY_CTRL, 0, 0xFF, 1)
};

static unsigned int cs4344_read_reg(struct snd_soc_codec *codec, unsigned int reg)
{
	return 0;
}

static int cs4344_write_reg(struct snd_soc_codec *codec, unsigned int reg, unsigned int value) 
{
	return 0;
}

static int cs4344_pcm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	return 0;
}
static int cs4344_mute(struct snd_soc_dai *dai, int mute)
{
	return 0;
}
static int cs4344_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	return 0;
}
static int cs4344_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	return 0;
}
#define CS4344_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
		SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_44100 | \
		SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000)

#define CS4344_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
	SNDRV_PCM_FMTBIT_S24_LE)

static struct snd_soc_dai_ops cs4344_dai_ops = {
	.hw_params = cs4344_pcm_hw_params,
	.set_fmt = cs4344_set_dai_fmt,
};

struct snd_soc_dai cs4344_dai = {
	.name = "CS4344",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = CS4344_RATES,
		.formats = CS4344_FORMATS,
	},
	.ops = &cs4344_dai_ops,
};
EXPORT_SYMBOL_GPL(cs4344_dai);

static int cs4344_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int cs4344_resume(struct platform_device *pdev)
{
	return 0;
}
static int cs4344_soc_probe(struct platform_device *pdev)
{
        struct snd_soc_device *socdev = platform_get_drvdata(pdev);
        struct snd_soc_codec *codec = cs4344_codec;
	int ret = 0;
	int i = 0;

	printk ("****** started cs4344 ******\n");
	BUG_ON(cs4344_codec == NULL);
	socdev->card->codec = cs4344_codec;
	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		printk(KERN_ERR "cs4344: failed to create pcms\n");
		goto pcm_err;
	}


        for (i = 0; i < ARRAY_SIZE(cs4344_snd_controls); i++) {
                struct snd_kcontrol *kctrl =
                snd_soc_cnew(&cs4344_snd_controls[i], codec, NULL);

                ret = snd_ctl_add(codec->card, kctrl);
                if (ret < 0)
		{
			printk (KERN_ERR "cs4344: failed to add control\n");
                        goto card_err;
		}
        }

	return ret;

card_err:
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
pcm_err:
	return ret;
}

static int cs4344_probe(struct platform_device *pdev)
{
	struct snd_soc_codec *codec = NULL;
	int ret = -1;
	unsigned char *cs4344_dummy_reg = NULL;

	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (codec == NULL)
		return -ENOMEM;

	codec->dev = &pdev->dev;
	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	codec->name = "cs4344";
	codec->owner = THIS_MODULE;
	codec->read = cs4344_read_reg;
	codec->write = cs4344_write_reg;
	codec->dai = &cs4344_dai;
	
	/* allocate a fake register cache */
	cs4344_dummy_reg = kmalloc(PAGE_SIZE, GFP_KERNEL);

	if (!cs4344_dummy_reg)
	{
		dev_err(codec->dev, "cs4344: failed to allocate register cache");
		goto error;
	}

        codec->reg_cache_size = PAGE_SIZE;
        codec->reg_cache = cs4344_dummy_reg;

	cs4344_codec = codec;

	ret = snd_soc_register_codec(codec);

	if (ret != 0) {
		dev_err(codec->dev, "Failed to register codec: %d\n", ret);
		goto err_codec;
	}

	ret = snd_soc_register_dai(&cs4344_dai);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to register DAIs: %d\n", ret);
		goto err_dai;
	}

	return ret;

err_dai:

err_codec:

error:
	BUG(); //TODO ADD
	return ret;
}

static int cs4344_remove(struct platform_device *pdev)
{
	/* can't turn off device */
	return 0;
}

static int cs4344_soc_remove(struct platform_device *pdev)
{
        struct snd_soc_device *socdev = platform_get_drvdata(pdev);

	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
	return 0;
}

struct snd_soc_codec_device soc_codec_dev_cs4344 = {
	.probe = 	cs4344_soc_probe,
	.remove = 	cs4344_soc_remove,
	.suspend = 	cs4344_suspend,
	.resume =	cs4344_resume,
};

static struct platform_driver cs4344_device = {
	.probe          = cs4344_probe,
	.remove         = cs4344_remove, 
	.driver = {
	.name = "cs4344"
	}
};

static int __init cs4344_mod_init(void)
{
        return platform_driver_register(&cs4344_device);
}

static void __exit cs4344_exit(void)
{
        platform_driver_unregister(&cs4344_device);
}

module_init(cs4344_mod_init);
module_exit(cs4344_exit);

EXPORT_SYMBOL_GPL(soc_codec_dev_cs4344);

MODULE_DESCRIPTION("ASoC CS4344 driver");
MODULE_AUTHOR("Mark F. Brown");
MODULE_LICENSE("GPL");
