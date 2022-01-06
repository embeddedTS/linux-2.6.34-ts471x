/*
 * sii9022.c  --  SII9022 ALSA SoC Audio driver
 *
 * The SII9022 isn't a codec.  It's an HDMI transmitter.  Still, we
 * want to send it a data stream just as we would to a real codec.
 *
 * TODO:  Need to make it talk to the SII9022 so that rates can be set, etc  
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
  
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <mach/hardware.h>

#include "sii9022.h"

struct sii9022_priv {
	int sysclk;
	int master;
	int vol_left;
	int vol_right;
	int fmt;
	int lrclk;
	int capture_channels;
	int playback_active;
	int capture_active;
	int clock_on;		/* clock enable status */
	int need_clk_for_access; /* need clock on because doing access */
	int need_clk_for_bias;   /* need clock on due to bias level */
	int (*clock_enable) (int enable);
	struct snd_pcm_substream *master_substream;
	struct snd_pcm_substream *slave_substream;
};

static struct snd_soc_device *sii9022_sndSocDevice;

static int sii9022_set_bias_level(struct snd_soc_codec *codec,
				   enum snd_soc_bias_level level);

#define SII9022_MAX_CACHED_REG SII9022_CHIP_SHORT_CTRL
static u16 sii9022_regs[(SII9022_MAX_CACHED_REG >> 1) + 1];

static unsigned int null_read(struct snd_soc_codec *codec,
					    unsigned int reg)
{
   return 0;  
}

static int null_write(struct snd_soc_codec *codec, unsigned int reg,
			  unsigned int value)
{
   return 0;  
}


#ifdef DEBUG
static void dump_reg(struct snd_soc_codec *codec)
{
	int i, reg;
	printk(KERN_DEBUG "dump begin\n");
	for (i = 0; i < 21; i++) {
		reg = sii9022_read(codec, all_reg[i]);
		printk(KERN_DEBUG "d r %04x, v %04x\n", all_reg[i], reg);
	}
	printk(KERN_DEBUG "dump end\n");
}
#else
#define dump_reg(x) 
#endif


static const struct snd_soc_dapm_widget sii9022_dapm_widgets[] = {
	SND_SOC_DAPM_DAC("DAC", "Playback", SND_SOC_NOPM, 0, 0),
};


static int sii9022_add_widgets(struct snd_soc_codec *codec)
{
	snd_soc_dapm_new_controls(codec, sii9022_dapm_widgets,
				  ARRAY_SIZE(sii9022_dapm_widgets));

	snd_soc_dapm_new_widgets(codec);
	return 0;
}

static int dac_info_volsw(struct snd_kcontrol *kcontrol,
			  struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 2;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 0xfc - 0x3c;
	return 0;
}

static int dac_get_volsw(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
   /* TODO:  Do something with the volume levels! */
   if (sii9022_sndSocDevice) {
      struct sii9022_priv *sii9022 = sii9022_sndSocDevice->card->codec->private_data;
      ucontrol->value.integer.value[0] = sii9022->vol_left;
      ucontrol->value.integer.value[1] = sii9022->vol_right;
      return 0;
   } else
      
      return -1;
}

static int dac_put_volsw(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol)
{
   /* TODO:  Do something with the volume levels! */
   if (sii9022_sndSocDevice) {
      struct sii9022_priv *sii9022 = sii9022_sndSocDevice->card->codec->private_data;
      sii9022->vol_left = ucontrol->value.integer.value[0];
      sii9022->vol_right = ucontrol->value.integer.value[1]; 
   
      return 0;
   } else
      return -1;
}


static const struct snd_kcontrol_new sii9022_snd_controls[] = {

	{.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	 .name = "Playback Volume",
	 .access = SNDRV_CTL_ELEM_ACCESS_READWRITE |
	 SNDRV_CTL_ELEM_ACCESS_VOLATILE,
	 .info = dac_info_volsw,
	 .get = dac_get_volsw,
	 .put = dac_put_volsw,
	 },
};

static int sii9022_add_controls(struct snd_soc_codec *codec)
{
	int err, i;

	for (i = 0; i < ARRAY_SIZE(sii9022_snd_controls); i++) {
		err = snd_ctl_add(codec->card,
				  snd_soc_cnew(&sii9022_snd_controls[i],
					       codec, NULL));
		if (err < 0)
			return err;
	}

	return 0;
}


static int sii9022_digital_mute(struct snd_soc_dai *codec_dai, int mute)
{	
	return 0;
}

static int sii9022_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{

	return 0;
}

static int sii9022_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				   int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct sii9022_priv *sii9022 = codec->private_data;
	
	switch (clk_id) {
	case SII9022_SYSCLK:
		sii9022->sysclk = freq;
		break;
	case SII9022_LRCLK:
		sii9022->lrclk = freq;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int sii9022_pcm_prepare(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{

	return 0;
}

static int sii9022_pcm_startup(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *sndSocDevice = rtd->socdev;
	struct snd_soc_codec *codec = sndSocDevice->card->codec;
	struct sii9022_priv *sii9022 = codec->private_data;
	struct snd_pcm_runtime *master_runtime;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		sii9022->playback_active++;
	else
		sii9022->capture_active++;

	
	/* The DAI has shared clocks so if we already have a playback or
	 * capture going then constrain this substream to match it.
	 */
	if (sii9022->master_substream) {
		master_runtime = sii9022->master_substream->runtime;

		if (master_runtime->rate != 0) {
			pr_debug("Constraining to %dHz\n",
				 master_runtime->rate);
			snd_pcm_hw_constraint_minmax(substream->runtime,
						     SNDRV_PCM_HW_PARAM_RATE,
						     master_runtime->rate,
						     master_runtime->rate);
		}

		if (master_runtime->sample_bits != 0) {
			pr_debug("Constraining to %d bits\n",
				 master_runtime->sample_bits);
			snd_pcm_hw_constraint_minmax(substream->runtime,
						     SNDRV_PCM_HW_PARAM_SAMPLE_BITS,
						     master_runtime->sample_bits,
						     master_runtime->sample_bits);
		}

		sii9022->slave_substream = substream;
	} else
		sii9022->master_substream = substream;
	
	return 0;
}

static void sii9022_pcm_shutdown(struct snd_pcm_substream *substream,
				  struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *sndSocDevice = rtd->socdev;
	struct snd_soc_codec *codec = sndSocDevice->card->codec;
	struct sii9022_priv *sii9022 = codec->private_data;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		sii9022->playback_active--;
	else
		sii9022->capture_active--;

	if (sii9022->master_substream == substream)
		sii9022->master_substream = sii9022->slave_substream;

	sii9022->slave_substream = NULL;

}

/*
 * Set PCM DAI bit size and sample rate.
 * input: params_rate, params_fmt
 */
static int sii9022_pcm_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *params,
				  struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *sndSocDevice = rtd->socdev;
	struct snd_soc_codec *codec = sndSocDevice->card->codec;
	struct sii9022_priv *sii9022 = codec->private_data;
	int channels = params_channels(params);

	pr_debug("%s channels=%d\n", __func__, channels);

	if (!sii9022->sysclk) {
		pr_err("%s: set sysclk first!\n", __func__);
		return -EFAULT;
	}

	if (substream == sii9022->slave_substream) {
		pr_debug("Ignoring hw_params for slave substream\n");
		return 0;
	}

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		sii9022->capture_channels = channels;

	return 0;
}

static int sii9022_set_bias_level(struct snd_soc_codec *codec,
				   enum snd_soc_bias_level level)
{
	return 0;
}

#define SII9022_RATES (SNDRV_PCM_RATE_8000 |\
		      SNDRV_PCM_RATE_11025 |\
		      SNDRV_PCM_RATE_16000 |\
		      SNDRV_PCM_RATE_22050 |\
		      SNDRV_PCM_RATE_32000 |\
		      SNDRV_PCM_RATE_44100 |\
		      SNDRV_PCM_RATE_48000 |\
		      SNDRV_PCM_RATE_96000)

#define SII9022_FORMATS (SNDRV_PCM_FMTBIT_S16_LE |\
			SNDRV_PCM_FMTBIT_S20_3LE |\
			SNDRV_PCM_FMTBIT_S24_LE)

struct snd_soc_dai_ops sii9022_ops = {
	.prepare = sii9022_pcm_prepare,
	.startup = sii9022_pcm_startup,
	.shutdown = sii9022_pcm_shutdown,
	.hw_params = sii9022_pcm_hw_params,
	.digital_mute = sii9022_digital_mute,
	.set_fmt = sii9022_set_dai_fmt,
	.set_sysclk = sii9022_set_dai_sysclk
};

struct snd_soc_dai sii9022_dai = {
	.name = "SII9022",
	.playback = {
		     .stream_name = "Playback",
		     .channels_min = 2,
		     .channels_max = 2,
		     .rates = SII9022_RATES,
		     .formats = SII9022_FORMATS,
		     },		    
	.ops = &sii9022_ops,
};
EXPORT_SYMBOL_GPL(sii9022_dai);

/*
 * Delayed work that turns off the audio clock after a delay.
 */
static void sii9022_work(struct work_struct *work)
{
	struct snd_soc_codec *codec =
		container_of(work, struct snd_soc_codec, delayed_work.work);
	struct sii9022_priv *sii9022 = codec->private_data;

	if (!sii9022->need_clk_for_access &&
	    !sii9022->need_clk_for_bias &&
	    sii9022->clock_on) {
		sii9022->clock_enable(0);
		sii9022->clock_on = 0;
	}
}

static int sii9022_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct snd_soc_device *sndSocDevice = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = sndSocDevice->card->codec;

	sii9022_set_bias_level(codec, SND_SOC_BIAS_OFF);

	return 0;
}

static int sii9022_resume(struct platform_device *pdev)
{
	/* TODO */
	return 0;
}



static __devinit int sii9022_do_init(struct snd_soc_device *socdev) 
{   
   int ret;
	struct snd_soc_codec *codec = socdev->card->codec;   	
	struct snd_soc_device *sndSocDevice = sii9022_sndSocDevice;
	
	codec->name = "SII9022";
	codec->owner = THIS_MODULE;
	codec->read = null_read; 
	codec->write = null_write; 
	codec->bias_level = SND_SOC_BIAS_OFF;
	codec->set_bias_level = sii9022_set_bias_level;
	codec->dai = &sii9022_dai;
	codec->num_dai = 1;
	codec->reg_cache_size = sizeof(sii9022_regs);
	codec->reg_cache_step = 2;
	codec->reg_cache = (void *)&sii9022_regs;

		/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		printk("failed to create pcms\n");
		return ret;
	}
	
	sii9022_add_controls(codec);
	sii9022_add_widgets(codec);
	sii9022_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
		
	return 0;
	
}

/*
 * initialise the SII9022 driver
 * register the mixer and dsp interfaces with the kernel
 */
static int sii9022_probe(struct platform_device *pdev)
{
	struct snd_soc_device *sndSocDevice = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = sndSocDevice->card->codec;		
	struct sii9022_priv *sii9022;
	struct sii9022_setup_data *setup = sndSocDevice->codec_data;
		
	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (codec == NULL)
		return -ENOMEM;

	sii9022 = kzalloc(sizeof(struct sii9022_priv), GFP_KERNEL);
	if (sii9022 == NULL) {
		kfree(codec);
		return -ENOMEM;
	}
	
	codec->private_data = sii9022;
	sndSocDevice->card->codec = codec;   
		
	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);   
	sii9022_sndSocDevice = sndSocDevice;

	if ((setup != NULL) && (setup->clock_enable != NULL)) {
		sii9022->clock_enable = setup->clock_enable;
		sii9022->need_clk_for_bias = 1;
		INIT_DELAYED_WORK(&codec->delayed_work, sii9022_work);
	}
	
	sii9022_do_init(sndSocDevice);
	
	return 0;
}

/*
 * This function forces any delayed work to be queued and run.
 */
static int run_delayed_work(struct delayed_work *dwork)
{
	int ret;

	/* cancel any work waiting to be queued. */
	ret = cancel_delayed_work(dwork);

	/* if there was any work waiting then we run it now and
	 * wait for it's completion */
	if (ret) {
		schedule_delayed_work(dwork, 0);
		flush_scheduled_work();
	}
	return ret;
}

/* power down chip */
static int sii9022_remove(struct platform_device *pdev)
{
	struct snd_soc_device *sndSocDevice = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = sndSocDevice->card->codec;

	if (codec->control_data)
		sii9022_set_bias_level(codec, SND_SOC_BIAS_OFF);
	run_delayed_work(&codec->delayed_work);
	snd_soc_free_pcms(sndSocDevice);
	snd_soc_dapm_free(sndSocDevice);
	
	kfree(codec->private_data);
	kfree(codec);
	
	return 0;
}

struct snd_soc_codec_device soc_codec_dev_sii9022 = {
	.probe = sii9022_probe,
	.remove = sii9022_remove,
	.suspend = sii9022_suspend,
	.resume = sii9022_resume,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_sii9022);


static int __init sii9022_modinit(void)
{
   return snd_soc_register_dai(&sii9022_dai);
}
module_init(sii9022_modinit);

static void __exit sii9022_exit(void)
{
   snd_soc_unregister_dai(&sii9022_dai);
}
module_exit(sii9022_exit);


MODULE_DESCRIPTION("ASoC SII9022 driver");
MODULE_AUTHOR("embeddedTS, Inc.");
MODULE_LICENSE("GPL");
