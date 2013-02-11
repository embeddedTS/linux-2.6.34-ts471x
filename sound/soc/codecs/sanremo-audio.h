/*
 * sanremo-audio.h  --  audio driver for sanremo audio
 *
 * Copyright (C) 2007 Marvell International Ltd.
 * Author: Paul Shen <bshen9@marvell.com>
 *         Bin Yang <bin.yang@marvell.com> 
 * 				 Yael Sheli Chemla<yael.s.shemla@marvell.com> 
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#ifndef __SANREMO_AUDIO_H
#define __SANREMO_AUDIO_H

/* sanremo audio register space */
#define SANREMO_AUDIO_OFFSET    0xB0

#define PCM_INTERFACE1	      0x00
#define PCM_INTERFACE2	      0x01
#define PCM_INTERFACE3	      0x02
#define PCM_RATE	      0x03
#define	ECHO_CANCEL_PATH      0x04
#define SIDETONE_GAIN1	      0x05
#define SIDETONE_GAIN2	      0x06
#define SIDETONE_GAIN3	      0x07
#define ADC_OFFSET1	      0x08
#define ADC_OFFSET2	      0x09
#define DMIC_DELAY	      0x0a
#define I2S_INTERFACE_1	      0x0b
#define I2S_INTERFACE_2	      0x0c
#define I2S_INTERFACE_3	      0x0d
#define I2S_INTERFACE_4	      0x0e
#define EQUALIZER_N0_1	      0x0f
#define EQUALIZER_N0_2	      0x10
#define EQUALIZER_N1_1	      0x11
#define EQUALIZER_N1_2	      0x12
#define EQUALIZER_D1_1	      0x13
#define EQUALIZER_D1_2	      0x14
#define SIDE_TONE_1	      0x15
#define SIDE_TONE_2	      0x16
#define LEFT_GAIN1	      0x17
#define LEFT_GAIN2	      0x18
#define RIGHT_GAIN1	      0x19
#define RIGHT_GAIN2	      0x1a
#define DAC_OFFSET	      0x1b
#define OFFSET_LEFT1	      0x1c
#define OFFSET_LEFT2	      0x1d
#define OFFSET_RIGHT1	      0x1e
#define OFFSET_RIGHT2	      0x1f
#define ADC_ANALOG_PROGRAM1   0x20
#define ADC_ANALOG_PROGRAM2   0x21
#define ADC_ANALOG_PROGRAM3   0x22
#define ADC_ANALOG_PROGRAM4   0x23
#define ANALOG_TO_ANALOG      0x24
#define HS1_CTRL	      0x25
#define HS2_CTRL	      0x26
#define LO1_CTRL	      0x27
#define LO2_CTRL	      0x28
#define EAR_SPKR_CTRL1	      0x29
#define EAR_SPKR_CTRL2	      0x2a
#define AUDIO_SUPPLIES1	      0x2b
#define AUDIO_SUPPLIES2	      0x2c
#define ADC_ENABLES1	      0x2d
#define ADC_ENABLES2	      0x2e
#define DAC_ENABLES1	      0x2f
#define DUMMY		      0x30

#define DAC_ENABLES2	      0x31
#define AUDIO_CAL1	      0x32
#define AUDIO_CAL2	      0x33
#define AUDIO_CAL3	      0x34
#define AUDIO_CAL4	      0x35
#define AUDIO_CAL5	      0x36
#define ANALOG_INPUT_SEL1     0x37
#define ANALOG_INPUT_SEL2     0x38
#define MIC_BUTTON_DETECTION  0x39
#define HEADSET_DETECTION     0x3a
#define SHORTS		      0x3b

#define SANREMO_AUDIO_END    0x3b
/* sanremo audio clk related register */
#define SANREMO_AUDIO_MISC_OFFSET 0x42
#define MISC2		      (SANREMO_AUDIO_END + 0x1)
#define PLL_CTRL1	      (SANREMO_AUDIO_END + 0x2)
#define PLL_FRAC1	      (SANREMO_AUDIO_END + 0x3)
#define PLL_FRAC2	      (SANREMO_AUDIO_END + 0x4)
#define PLL_FRAC3	      (SANREMO_AUDIO_END + 0x5)

extern struct snd_soc_dai sanremo_audio_dai[2];
extern struct snd_soc_codec_device soc_codec_dev_sanremo_audio;

#endif
