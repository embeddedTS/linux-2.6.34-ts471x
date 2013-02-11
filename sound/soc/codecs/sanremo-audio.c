/*
 * linux/sound/soc/codecs/sanremo_audio.c
 * Base on linux/sound/soc/codecs/wm8753.c
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
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <mach/sanremo.h>
#include "sanremo-audio.h"
#include <mach/regs-mpmu.h>
#include <asm/io.h>

#define SANREMO_SOC_PROC
#define AUDIO_NAME "sanremo audio codec"
#define SANREMO_AUDIO_VERSION "0.2"
/* debug */
#define SANREMO_DEBUG 0
#if SANREMO_DEBUG
#define dbg(format, arg...) printk(KERN_INFO "sanremo-audio: " format "\n", ## arg)
#else
#define dbg(format, arg...)
#endif

#define ARRAY_AND_SIZE(x)	x, ARRAY_SIZE(x)

/* codec private data */
struct sanremo_audio_priv {
        unsigned int sysclk;
        unsigned int pcmclk;
};

/*here is the sanremo audio default value, audio server will reset the value to it*/
static const u8 sanremo_audio_regs[] = {
	0x00, 0x00, 0x00, 0x00,	/*0x00 ~ 0x03*/
	0x00, 0x00, 0x00, 0x00, /*0x04 ~ 0x07*/
	0x00, 0x00, 0x00, 0x00, /*0x08 ~ 0x0b*/
	0x08, 0x00, 0x40, 0x00, /*0x0c ~ 0x0f*/
	0x00, 0x00, 0x00, 0x00,	/*0x10 ~ 0x13*/
	0x00, 0x3f, 0x3f, 0x3f,	/*0x14 ~ 0x17*/
	0x3f, 0x3f, 0x3f, 0x00,	/*0x18 ~ 0x1b*/
	0x00, 0x00, 0x00, 0x00,	/*0x1c ~ 0x1f*/
	0x00, 0x00, 0x00, 0x00,	/*0x20 ~ 0x23*/
	0x00, 0x00, 0x00, 0x00,	/*0x24 ~ 0x27*/
	0x00, 0x00, 0x00, 0x00,	/*0x28 ~ 0x2b*/
	0x00, 0x00, 0x00, 0x00,	/*0x2c ~ 0x2f*/
	0x00, 0x00, 0x00, 0x00,	/*0x30 ~ 0x33*/
	0x00, 0x00, 0x00, 0x00,	/*0x34 ~ 0x37*/
	0x00, 0x00, 0x00, 0x00,	/*0x38 ~ 0x3b*/
	0x00, 0x00, 0x00, 0x00, 0x00,		  /*0x42 ~ 0x46*/
};

static const struct snd_kcontrol_new levante_direct_access[] = {
	/* Audio Register */
	SOC_SINGLE("LEVANTE_PCM_INTERFACE1", PCM_INTERFACE1, 0, 0xff, 0),              /* 0xB0 */
	SOC_SINGLE("LEVANTE_PCM_INTERFACE2", PCM_INTERFACE2, 0, 0xff, 0),              /* 0xB1 */
	SOC_SINGLE("LEVANTE_PCM_INTERFACE3", PCM_INTERFACE3, 0, 0xff, 0),              /* 0xB2 */
	SOC_SINGLE("LEVANTE_PCM_RATE", PCM_RATE, 0, 0xff, 0),                          /* 0xB3 */
	SOC_SINGLE("LEVANTE_ECHO_CANCEL_PATH", ECHO_CANCEL_PATH, 0, 0xff, 0),          /* 0xB4 */
	SOC_SINGLE("LEVANTE_SIDETONE_GAIN1", SIDETONE_GAIN1, 0, 0xff, 0),              /* 0xB5 */
	SOC_SINGLE("LEVANTE_SIDETONE_GAIN2", SIDETONE_GAIN2, 0, 0xff, 0),              /* 0xB6 */
	SOC_SINGLE("LEVANTE_SIDETONE_GAIN3", SIDETONE_GAIN3, 0, 0xff, 0),              /* 0xB7 */
	SOC_SINGLE("LEVANTE_ADC_OFFSET1", ADC_OFFSET1, 0, 0xff, 0),                    /* 0xB8 */
	SOC_SINGLE("LEVANTE_ADC_OFFSET2", ADC_OFFSET2, 0, 0xff, 0),                    /* 0xB9 */
	SOC_SINGLE("LEVANTE_DMIC_DELAY", DMIC_DELAY, 0, 0xff, 0),                      /* 0xBA */
	SOC_SINGLE("LEVANTE_I2S_INTERFACE1", I2S_INTERFACE_1, 0, 0xff, 0),              /* 0xBB */
	SOC_SINGLE("LEVANTE_I2S_INTERFACE2", I2S_INTERFACE_2, 0, 0xff, 0),              /* 0xBC */
	SOC_SINGLE("LEVANTE_I2S_INTERFACE3", I2S_INTERFACE_3, 0, 0xff, 0),              /* 0xBD */
	SOC_SINGLE("LEVANTE_I2S_INTERFACE4", I2S_INTERFACE_4, 0, 0xff, 0),              /* 0xBE */
	SOC_SINGLE("LEVANTE_EQUALIZER_N0_1", EQUALIZER_N0_1, 0, 0xff, 0),              /* 0xBF */
	SOC_SINGLE("LEVANTE_EQUALIZER_N0_2", EQUALIZER_N0_2, 0, 0xff, 0),              /* 0xC0 */
	SOC_SINGLE("LEVANTE_EQUALIZER_N1_1", EQUALIZER_N1_1, 0, 0xff, 0),              /* 0xC1 */
	SOC_SINGLE("LEVANTE_EQUALIZER_N1_2", EQUALIZER_N1_2, 0, 0xff, 0),              /* 0xC2 */
	SOC_SINGLE("LEVANTE_EQUALIZER_D1_1", EQUALIZER_N1_1, 0, 0xff, 0),              /* 0xC3 */
	SOC_SINGLE("LEVANTE_EQUALIZER_D1_2", EQUALIZER_N1_2, 0, 0xff, 0),              /* 0xC4 */
	SOC_SINGLE("LEVANTE_SIDE_TONE1", SIDE_TONE_1, 0, 0xff, 0),                      /* 0xC5 */
	SOC_SINGLE("LEVANTE_SIDE_TONE2", SIDE_TONE_2, 0, 0xff, 0),                      /* 0xC6 */
	SOC_SINGLE("LEVANTE_LEFT_GAIN1", LEFT_GAIN1, 0, 0xff, 0),                      /* 0xC7 */
	SOC_SINGLE("LEVANTE_LEFT_GAIN2", LEFT_GAIN2, 0, 0xff, 0),                      /* 0xC8 */
	SOC_SINGLE("LEVANTE_RIGHT_GAIN1", RIGHT_GAIN1, 0, 0xff, 0),                    /* 0xC9 */
	SOC_SINGLE("LEVANTE_RIGHT_GAIN2", RIGHT_GAIN2, 0, 0xff, 0),                    /* 0xCA */
	SOC_SINGLE("LEVANTE_DAC_OFFSET", DAC_OFFSET, 0, 0xff, 0),                      /* 0xCB */
	SOC_SINGLE("LEVANTE_OFFSET_LEFT1", OFFSET_LEFT1, 0, 0xff, 0),                  /* 0xCC */
	SOC_SINGLE("LEVANTE_OFFSET_LEFT2", OFFSET_LEFT2, 0, 0xff, 0),                  /* 0xCD */
	SOC_SINGLE("LEVANTE_OFFSET_RIGHT1", OFFSET_RIGHT1, 0, 0xff, 0),                /* 0xCE */
	SOC_SINGLE("LEVANTE_OFFSET_RIGHT2", OFFSET_RIGHT2, 0, 0xff, 0),                /* 0xCF */
	SOC_SINGLE("LEVANTE_ADC_ANALOG_PROGRAM1", ADC_ANALOG_PROGRAM1, 0, 0xff, 0),    /* 0xD0 */
	SOC_SINGLE("LEVANTE_ADC_ANALOG_PROGRAM2", ADC_ANALOG_PROGRAM2, 0, 0xff, 0),    /* 0xD1 */
	SOC_SINGLE("LEVANTE_ADC_ANALOG_PROGRAM3", ADC_ANALOG_PROGRAM3, 0, 0xff, 0),    /* 0xD2 */
	SOC_SINGLE("LEVANTE_ADC_ANALOG_PROGRAM4", ADC_ANALOG_PROGRAM4, 0, 0xff, 0),    /* 0xD3 */
	SOC_SINGLE("LEVANTE_ANALOG_TO_ANALOG", ANALOG_TO_ANALOG, 0, 0xff, 0),          /* 0xD4 */
	SOC_SINGLE("LEVANTE_HS1_CTRL", HS1_CTRL, 0, 0xff, 0),                          /* 0xD5 */
	SOC_SINGLE("LEVANTE_HS2_CTRL", HS2_CTRL, 0, 0xff, 0),                          /* 0xD6 */
	SOC_SINGLE("LEVANTE_LO1_CTRL", LO1_CTRL, 0, 0xff, 0),                          /* 0xD7 */
	SOC_SINGLE("LEVANTE_LO2_CTRL", LO2_CTRL, 0, 0xff, 0),                          /* 0xD8 */
	SOC_SINGLE("LEVANTE_EAR_SPKR_CTRL1", EAR_SPKR_CTRL1, 0, 0xff, 0),              /* 0xD9 */
	SOC_SINGLE("LEVANTE_EAR_SPKR_CTRL2", EAR_SPKR_CTRL2, 0, 0xff, 0),              /* 0xDA */
	SOC_SINGLE("LEVANTE_AUDIO_SUPPLIES1", AUDIO_SUPPLIES1, 0, 0xff, 0),            /* 0xDB */
	SOC_SINGLE("LEVANTE_AUDIO_SUPPLIES2", AUDIO_SUPPLIES2, 0, 0xff, 0),            /* 0xDC */
	SOC_SINGLE("LEVANTE_ADC_ENABLES1", ADC_ENABLES1, 0, 0xff, 0),                  /* 0xDD */
	SOC_SINGLE("LEVANTE_ADC_ENABLES2", ADC_ENABLES2, 0, 0xff, 0),                  /* 0xDE */
	SOC_SINGLE("LEVANTE_DAC_ENABLES1", DAC_ENABLES1, 0, 0xff, 0),                  /* 0xDF */
	SOC_SINGLE("LEVANTE_DAC_DUMMY", DUMMY, 0, 0xff, 0),                            /* 0xE0 */
	SOC_SINGLE("LEVANTE_DAC_ENABLES2", DAC_ENABLES2, 0, 0xff, 0),                  /* 0xE1 */
	SOC_SINGLE("LEVANTE_AUDIO_CAL1", AUDIO_CAL1, 0, 0xff, 0),                      /* 0xE2 */
	SOC_SINGLE("LEVANTE_AUDIO_CAL2", AUDIO_CAL2, 0, 0xff, 0),                      /* 0xE3 */
	SOC_SINGLE("LEVANTE_AUDIO_CAL3", AUDIO_CAL3, 0, 0xff, 0),                      /* 0xE4 */
	SOC_SINGLE("LEVANTE_AUDIO_CAL4", AUDIO_CAL4, 0, 0xff, 0),                      /* 0xE5 */
	SOC_SINGLE("LEVANTE_AUDIO_CAL5", AUDIO_CAL5, 0, 0xff, 0),                      /* 0xE6 */
	SOC_SINGLE("LEVANTE_ANALOG_INPUT_SEL1", ANALOG_INPUT_SEL1, 0, 0xff, 0),        /* 0xE7 */
	SOC_SINGLE("LEVANTE_ANALOG_INPUT_SEL2", ANALOG_INPUT_SEL2, 0, 0xff, 0),        /* 0xE8 */
	SOC_SINGLE("LEVANTE_MIC_BUTTON_DETECTION", MIC_BUTTON_DETECTION, 0, 0xff, 0),  /* 0xE9 */
	SOC_SINGLE("LEVANTE_HEADSET_DETECTION", HEADSET_DETECTION, 0, 0xff, 0),        /* 0xEA */
	SOC_SINGLE("LEVANTE_SHORTS", SHORTS, 0, 0xff, 0),                              /* 0xEB */
	/* MIsc Register for audio clock configuration */
	SOC_SINGLE("LEVANTE_MISC2", MISC2, 0, 0xff, 0),                                /* 0x42 */
	SOC_SINGLE("LEVANTE_PLL_CTRL1", PLL_CTRL1, 0, 0xff, 0),                        /* 0x43 */
	SOC_SINGLE("LEVANTE_PLL_FRAC1", PLL_FRAC1, 0, 0xff, 0),                        /* 0x44 */
	SOC_SINGLE("LEVANTE_PLL_FRAC2", PLL_FRAC2, 0, 0xff, 0),                        /* 0x45 */
	SOC_SINGLE("LEVANTE_PLL_FRAC3", PLL_FRAC3, 0, 0xff, 0),                        /* 0x46 */
};

#define LEVANTE_ID_V  (66)

static int levante_id_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	int ret;
	unsigned char value;

	ret = sanremo_read(0x00, &value);
	if (ret != 0)
		return -1;

	ucontrol->value.integer.value[0] = value;
	return 0;
}


static const struct snd_kcontrol_new levante_id_read =
	SOC_SINGLE_EXT("LEVANTE_ID", LEVANTE_ID_V, 0, 0xff, 0, levante_id_get, NULL);


static int levante_add_direct_access(struct snd_soc_codec *codec)
{
	int err, i;
	for (i = 0; i < ARRAY_SIZE(levante_direct_access); i++) {
		err = snd_ctl_add(codec->card,
			snd_soc_cnew(&levante_direct_access[i], codec, NULL));
		if (err < 0)
			return err;
	}
	return 0;
}

static int levante_id_access(struct snd_soc_codec *codec)
{
	int err;
	printk(KERN_ERR "levante_id_access\n");
	err = snd_ctl_add(codec->card, snd_soc_cnew(&levante_id_read, codec, NULL));
	if (err < 0)
		return err;

	return 0;
}


/*
 * read sanremo audio register cache
 */
static unsigned int sanremo_audio_read(struct snd_soc_codec *codec, unsigned int reg)
{
	u8 *cache = codec->reg_cache;

	if (reg > (ARRAY_SIZE(sanremo_audio_regs)))
		return -EIO;

	return cache[reg];
}


/*
 * write to the sanremo audio register space
 */
static int sanremo_audio_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int value)
{
	int offset;
	u8 *cache = codec->reg_cache;
	if (reg <= SANREMO_AUDIO_END)
		offset = SANREMO_AUDIO_OFFSET;
	else if (reg < ARRAY_SIZE(sanremo_audio_regs))
		offset = SANREMO_AUDIO_MISC_OFFSET - SANREMO_AUDIO_END - 1;
	else
		return -EIO;
	sanremo_write(reg + offset, value);
	cache[reg] = value;
			

	return 0;
}


/*
 * Sanremo audio codec reset
 */
static int sanremo_audio_reset(struct snd_soc_codec *codec)
{
	unsigned int reg;

	for (reg = 0; reg < ARRAY_SIZE(sanremo_audio_regs); reg++) {
		if (reg != 0x2c)
			sanremo_audio_write(codec, reg, sanremo_audio_regs[reg]);
	}

	sanremo_write(0xda, 0x05);

	return 0;
}


/*
 * Sanremo audio codec startup config 
 */
static int sanremo_audio_startup_config(struct snd_soc_codec *codec)
{
	sanremo_audio_write(codec, 0x3c, 0x38);
	sanremo_audio_write(codec, 0x2c, 0x0b);
	sanremo_audio_write(codec, 0x0e, 0x80);
	sanremo_audio_write(codec, 0x39, 0x27);
	sanremo_audio_write(codec, 0x3d, 0x18);
	sanremo_audio_write(codec, 0x3e, 0x12);
	sanremo_audio_write(codec, 0x3f, 0x83);
	sanremo_audio_write(codec, 0x2a, 0x01);
	sanremo_audio_write(codec, 0x0c, 0x11);
	sanremo_audio_write(codec, 0x23, 0x20);
	sanremo_audio_write(codec, 0x31, 0x38);
	sanremo_audio_write(codec, 0x2a, 0x05);

	return 0;
}


/*
 * Sanremo audio codec speaker and mic1 config 
 */
static int sanremo_audio_speaker_mic1_config(struct snd_soc_codec *codec)
{
	sanremo_audio_write(codec, 0x0e, 0xc8);
	sanremo_audio_write(codec, 0x17, 0x04);
	sanremo_audio_write(codec, 0x18, 0x04);
	sanremo_audio_write(codec, 0x19, 0x04);
	sanremo_audio_write(codec, 0x1a, 0x04);
	sanremo_audio_write(codec, 0x24, 0x40);
	sanremo_audio_write(codec, 0x29, 0x07);
	sanremo_audio_write(codec, 0x2f, 0x20);
	sanremo_audio_write(codec, 0x31, 0x39);
	sanremo_audio_write(codec, 0x38, 0x02);
	sanremo_audio_write(codec, 0x2a, 0x01);
	sanremo_audio_write(codec, 0x2a, 0x05);

	sanremo_audio_write(codec, 0x02, 0x10);
	sanremo_audio_write(codec, 0x0d, 0x10);
	sanremo_audio_write(codec, 0x20, 0x3c);
	sanremo_audio_write(codec, 0x21, 0xc7);
	sanremo_audio_write(codec, 0x23, 0x23);
	sanremo_audio_write(codec, 0x2d, 0x05);
	sanremo_audio_write(codec, 0x2e, 0x24);
	sanremo_audio_write(codec, 0x2a, 0x01);
	sanremo_audio_write(codec, 0x2a, 0x05);

	sanremo_audio_write(codec, 0x2e, 0x34);
	sanremo_audio_write(codec, 0x2a, 0x01);
	sanremo_audio_write(codec, 0x2a, 0x05);

	return 0;
}


/*
 * sanremo audio controls
 */
static int sanremo_audio_get_dai(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

static int sanremo_audio_set_dai(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

/* 
 * add non dapm controls 
 */
static int sanremo_audio_add_controls(struct snd_soc_codec *codec)
{
	levante_add_direct_access(codec);
	levante_id_access(codec);
	return 0;
}


/*
 * DAMP controls
 */


/* 
 * add dapm controls
 */
static int sanremo_audio_add_widgets(struct snd_soc_codec *codec)
{
	return 0;
}



static int sanremo_audio_set_dai_pll(struct snd_soc_dai *codec_dai,
		int pll_id, unsigned int freq_in, unsigned int freq_out)
{
	return 0;
}

static int sanremo_audio_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	return 0;
}

/* set PCM DAI configuration */
static int sanremo_aduio_pcm_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	return 0;
}

static int sanremo_audio_pcm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	return 0;
}

/* set HIFI DAI configuration */
static int sanremo_aduio_hifi_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	return 0;
}

static int sanremo_audio_hifi_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;

	int rate, value;
	rate =  params_rate(params);
	value = sanremo_audio_read(codec, 0x0e);
	value &= 0xf0; 
	switch(rate){
		case 48000:
			value |= 0x08;
			break;
		case 44100:
			value |= 0x07;
			break;
		case 32000:
			value |= 0x06;
			break;
		case 24000:
			value |= 0x05;
			break;
		case 22050:
			value |= 0x04;
			break;
		case 16000:
			value |= 0x03;
			break;
		case 12000:
			value |= 0x02;
			break;
		case 11025:
			value |= 0x01;
			break;
		case 8000:
			value |= 0x00;
			break;
		default:
			printk(KERN_ERR "unsupported rate\n");
			return -EINVAL;
			break;
	}
	/*sanremo_audio_write(codec, 0x0e, value); sanremo has sample rate switching issue*/

	return 0;
}

static int sanremo_audio_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct pxa910_runtime_data *prtd = substream->runtime->private_data;
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
//		sanremo_write(0xdc, 0xb);
	//	__raw_writel(0x11, MPMU_VRCR);
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
//		sanremo_write(0xdc, 0x3);
	//	__raw_writel(0x0, MPMU_VRCR);
		break;

	case SNDRV_PCM_TRIGGER_RESUME:
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		break;
	}
	return 0;
}

static int sanremo_audio_mute(struct snd_soc_dai *dai, int mute)
{
	return 0;
}

static int sanremo_audio_set_bias_level(struct snd_soc_codec *codec,  enum snd_soc_bias_level level)
{
	switch (level) {
	        case SND_SOC_BIAS_ON: /* full On */
			break;

		case SND_SOC_BIAS_PREPARE: /* partial On */
			break;

		case SND_SOC_BIAS_STANDBY: /* partial On */
			break;

		case SND_SOC_BIAS_OFF: /* Off, without power */
			break;
	}

	codec->bias_level = level;

	return 0;
}

#define SANREMO_AUDIO_HIFI_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
		                SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |\ 
				SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000)

#define SANREMO_AUDIO_HIFI_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE)


static struct snd_soc_dai_ops sanremo_dai_ops = {
	.hw_params = sanremo_audio_hifi_hw_params,
	.digital_mute = sanremo_audio_mute,
	.set_fmt = sanremo_aduio_hifi_set_dai_fmt,
	.set_clkdiv = NULL,
	.set_pll = NULL,
	.set_sysclk = NULL,
	.trigger = sanremo_audio_trigger,
};

/*
 * HIFI DAI
 */
struct snd_soc_dai sanremo_audio_dai[]={
/* DAI HIFI mode*/
	{
		.name = "sanremo audio HiFi",
		.id = 1,
		.playback = {
			.stream_name = "HiFi Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SANREMO_AUDIO_HIFI_RATES,
			.formats = SANREMO_AUDIO_HIFI_FORMATS,
		},
		.capture = {
			.stream_name = "HiFi Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SANREMO_AUDIO_HIFI_RATES,
			.formats = SANREMO_AUDIO_HIFI_FORMATS,
		},
		.ops = &sanremo_dai_ops,
	},

	{
		.name = "sanremo audio pcm",
		.id = 1,
		.playback = {
			.stream_name = "Pcm Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SANREMO_AUDIO_HIFI_RATES,
			.formats = SANREMO_AUDIO_HIFI_FORMATS,
		},
		.capture = {
			.stream_name = "Pcm Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SANREMO_AUDIO_HIFI_RATES,
			.formats = SANREMO_AUDIO_HIFI_FORMATS,
		},
		.ops = &sanremo_dai_ops,
	},
};

static void sanremo_audio_work(struct work_struct *work)
{
	struct snd_soc_codec *codec =
			container_of(work, struct snd_soc_codec, delayed_work.work);
	sanremo_audio_set_bias_level(codec, codec->bias_level);
}


static int sanremo_audio_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}


static int sanremo_audio_resume(struct platform_device *pdev)
{
	return 0;
}


static int sanremo_audio_init(struct snd_soc_device *socdev)
{
	struct snd_soc_codec *codec = socdev->card->codec;
	int reg, ret = 0;

	codec->name = "sanremo audio";
	codec->owner = THIS_MODULE;
	codec->read = sanremo_audio_read;
	codec->write = sanremo_audio_write;
	codec->set_bias_level = sanremo_audio_set_bias_level;
	codec->dai = sanremo_audio_dai;
	codec->num_dai = ARRAY_SIZE(sanremo_audio_dai);
	codec->reg_cache_size = sizeof(sanremo_audio_regs);
	codec->reg_cache = kmemdup(sanremo_audio_regs, sizeof(sanremo_audio_regs), GFP_KERNEL);

	if(codec->reg_cache == NULL)
		return -ENOMEM;

	printk(KERN_ERR "sanremo_audio_init :power & pll init\n" );

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		printk(KERN_ERR "sanremo audio: failed to create pcms\n");
		goto pcm_err;
	}

	codec->bias_level = SND_SOC_BIAS_STANDBY;
	schedule_delayed_work(&codec->delayed_work,msecs_to_jiffies(2));

	sanremo_audio_reset(codec);

	sanremo_audio_add_controls(codec);
	sanremo_audio_add_widgets(codec);
	
	ret = snd_soc_init_card(socdev);
	if (ret < 0)
	{
		printk(KERN_ERR "sanremo audio: failed to register card\n");
		goto card_err;
	}

	printk(KERN_ERR "sanremo_audio_init :enable speaker and mic1\n" );
	sanremo_audio_startup_config(codec);
	sanremo_audio_speaker_mic1_config(codec);
	return ret;

card_err:
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
pcm_err:
	kfree(codec->reg_cache);
	return ret;
}

static struct snd_soc_device *sanremo_audio_socdev;


static int sanremo_audio_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct _setup_data *setup;
	struct snd_soc_codec *codec;
	struct sanremo_audio_priv *sanremo_audio;
	int ret = 0;

	printk(KERN_INFO "Sanremo Audio Codec %s", SANREMO_AUDIO_VERSION);

	setup = socdev->codec_data;
	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (codec == NULL)
		return -ENOMEM;

	sanremo_audio = kzalloc(sizeof(struct sanremo_audio_priv), GFP_KERNEL);
	if (sanremo_audio == NULL) {
		kfree(codec);
		return -ENOMEM;
	}


	codec->private_data = sanremo_audio;
	socdev->card->codec = codec;
	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);
	sanremo_audio_socdev = socdev;
	INIT_DELAYED_WORK(&codec->delayed_work, sanremo_audio_work);

	sanremo_audio_init(socdev);

	return ret;
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
static int sanremo_audio_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	if (codec->control_data)
		sanremo_audio_set_bias_level(codec, SND_SOC_BIAS_OFF);

	run_delayed_work(&codec->delayed_work);
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
	
	kfree(codec->private_data);
	kfree(codec);
	return 0;
}

struct snd_soc_codec_device soc_codec_dev_sanremo_audio = {
	.probe = 	sanremo_audio_probe,
	.remove = 	sanremo_audio_remove,
	.suspend = 	sanremo_audio_suspend,
	.resume =	sanremo_audio_resume,
};

EXPORT_SYMBOL_GPL(soc_codec_dev_sanremo_audio);

static int __init sanremo_audio_modinit(void)
{
	return snd_soc_register_dais(ARRAY_AND_SIZE(sanremo_audio_dai));
}
module_init(sanremo_audio_modinit);

static void __exit sanremo_audio_exit(void)
{
	snd_soc_unregister_dais(ARRAY_AND_SIZE(sanremo_audio_dai));
}
module_exit(sanremo_audio_exit);

MODULE_DESCRIPTION("ASoC Sanremo audio driver");
MODULE_AUTHOR("bshen9@marvell.com");
MODULE_LICENSE("GPL");


