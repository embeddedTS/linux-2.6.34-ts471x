/*
 * sii9022.h  --  SII9022 ALSA SoC Audio interface
 *
 * The SII9022 isn't a codec.  It's an HDMI transmitter.  Still, we
 * want to send it a data stream just as we would to a real codec.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _SII9022_H
#define _SII9022_H

#include <linux/i2c.h>

extern struct snd_soc_dai sii9022_dai;
extern struct snd_soc_codec_device soc_codec_dev_sii9022;


#define SII9022_CHIP_ID			0x0000
#define SII9022_CHIP_DIG_POWER			0x0002
#define SII9022_CHIP_CLK_CTRL			0x0004
#define SII9022_CHIP_I2S_CTRL			0x0006
#define SII9022_CHIP_SSS_CTRL			0x000a
#define SII9022_CHIP_ADCDAC_CTRL		0x000e
#define SII9022_CHIP_DAC_VOL			0x0010
#define SII9022_CHIP_PAD_STRENGTH		0x0014
#define SII9022_CHIP_ANA_ADC_CTRL		0x0020
#define SII9022_CHIP_ANA_HP_CTRL		0x0022
#define SII9022_CHIP_ANA_CTRL			0x0024
#define SII9022_CHIP_LINREG_CTRL		0x0026
#define SII9022_CHIP_REF_CTRL			0x0028
#define SII9022_CHIP_MIC_CTRL			0x002a
#define SII9022_CHIP_LINE_OUT_CTRL		0x002c
#define SII9022_CHIP_LINE_OUT_VOL		0x002e
#define SII9022_CHIP_ANA_POWER			0x0030
#define SII9022_CHIP_PLL_CTRL			0x0032
#define SII9022_CHIP_CLK_TOP_CTRL		0x0034
#define SII9022_CHIP_ANA_STATUS		0x0036
#define SII9022_CHIP_SHORT_CTRL		0x003c
#define SII9022_CHIP_ANA_TEST2			0x003a

#define SII9022_SYSCLK		0x00
#define SII9022_LRCLK		0x01

struct sii9022_setup_data {
	int (*clock_enable) (int enable);
};

#endif
