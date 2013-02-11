/*
 * Monahans Micco PMIC Management Routines
 *
 *
 * Copyright (C) 2006, Marvell Corporation(fengwei.yin@marvell.com).
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/suspend.h>
#include <asm/uaccess.h>
#include <plat/pxa3xx_pmic.h>
#include <mach/micco.h>
#include <mach/cputype.h>

#define	DEBUG

#define MICCO_REG_NUM		(0xC0)

/* FIXME: until pxa3xx_pm.c is merged, this function points to nothing
 * and returns PM_SUSPEND_MEM by default to fool micco_suspend/resume
 */
#if 0
extern int get_pm_state(void);
#else
static inline int get_pm_state(void) { return PM_SUSPEND_MEM; }
#endif

static struct pxa3xx_pmic_regs micco_regs[MICCO_REG_NUM];
static struct power_supply_module *micco_power_module;
static unsigned int event;

/* Make sure that Power I2C has been initialized before invoke this function */
extern int pxa_i2c_set_speed(int speed);
extern int pxa_i2c_get_speed(void);
int micco_codec_disable_output(int type);

/* Unique ID allocation */
static struct i2c_client *g_client;

int micco_read(u8 reg, u8 *pval)
{
	struct micco_platform_data *pdata = g_client->dev.platform_data;
	int ret;
	int status;

	if (g_client == NULL)	/* No global client pointer? */
		return -1;

	/* Cache read of the Micco register. Disabled temporary */
#if	0
	if (micco_regs[reg].hit) {
		*pval = micco_regs[reg].data;
		return 0;
	}
#endif

	spin_lock(&pdata->lock);
	disable_irq(g_client->irq);
	ret = i2c_smbus_read_byte_data(g_client, reg);
	enable_irq(g_client->irq);
	if (ret >= 0) {
		*pval = ret;
		micco_regs[reg].hit = ~micco_regs[reg].mask;
		micco_regs[reg].data = ret;
		status = 0;
	} else
		status = -EIO;
	spin_unlock(&pdata->lock);

	return status;
}

int micco_write(u8 reg, u8 val)
{
	struct micco_platform_data *pdata = g_client->dev.platform_data;
	int ret;
	int status;

	if (g_client == NULL)	/* No global client pointer? */
		return -1;

	spin_lock(&pdata->lock);
	disable_irq(g_client->irq);
	ret = i2c_smbus_write_byte_data(g_client, reg, val);
	enable_irq(g_client->irq);
	if (ret == 0) {
		micco_regs[reg].hit = ~micco_regs[reg].mask;
		micco_regs[reg].data = val;
		status = 0;
	} else
		status = -EIO;
	spin_unlock(&pdata->lock);

	return status;
}

static int micco_initchip(void)
{
	int i;

	memset(&micco_regs, 0,
		(sizeof(struct pxa3xx_pmic_regs) * MICCO_REG_NUM));
	/* TODO: Mask all micco registers uncacheable now.
	 * We can do some optimization here later.
	 */
	for (i = 0; i < MICCO_REG_NUM; i++)
		micco_regs[i].mask = 1;

	return micco_write(MICCO_SYSCTRL_A, 0xE8);
}

/* Micco TSI functions */
int micco_enable_pen_down_irq(int enable)
{
	int ret;
	u8 val;

	if (enable) {
		/* enable pen down IRQ */
		ret = micco_read(MICCO_IRQ_MASK_C, &val);
		val &= ~0x10;
		ret = micco_write(MICCO_IRQ_MASK_C, val);
	} else {
		/* disable pen down IRQ */
		ret = micco_read(MICCO_IRQ_MASK_C, &val);
		if (!(val & IRQ_MASK_C_PEN_DOWN)) {
			val |= 0x10;
			ret = micco_write(MICCO_IRQ_MASK_C, val);
		}
	}
	return ret;
}

int micco_tsi_poweron(void)
{
	int status;
	u8 val;

	val = 0x10;
	status = micco_write(MICCO_ADC_MAN_CONTROL, val);

	status = micco_read(MICCO_ADC_AUTO_CONTROL_2, &val);
	if (!(val & MICCO_ADC_AUTO_2_PENDET_EN)) {
		val |= MICCO_ADC_AUTO_2_PENDET_EN;
		status = micco_write(MICCO_ADC_AUTO_CONTROL_2, val);
	}

	/* TSI_DEABY: 3 slot. TSI_SKIP: 3 slot */
	val = 0x1B;
	status = micco_write(MICCO_TSI_CONTROL_1, val);

	val = 0x00;
	status = micco_write(MICCO_TSI_CONTROL_2, val);
	if (status)
		return -EIO;

	return 0;
}

int micco_tsi_poweroff(void)
{
	int status;
	u8 val;

	status = micco_read(MICCO_ADC_AUTO_CONTROL_2, &val);
	if (status)
		return -EIO;

	val &= ~(MICCO_ADC_AUTO_2_TSI_EN | MICCO_ADC_AUTO_2_PENDET_EN);
	status = micco_write(MICCO_ADC_AUTO_CONTROL_2, val);

	if (status)
		return -EIO;

	return 0;
}

int micco_tsi_readxy(u16 *x, u16 *y, int pen_state)
{
	int status;
	u8 val;
	u16 mx, my, lxy;

	status = micco_read(MICCO_TSI_X_MSB, &val);
	if (status)
		return -EIO;
	mx = val;

	status = micco_read(MICCO_TSI_Y_MSB, &val);
	if (status)
		return -EIO;
	my = val;

	status = micco_read(MICCO_TSI_XY_MSB, &val);
	if (status)
		return -EIO;
	lxy = val;

	*x = ((mx << 2) & 0x3fc) + (lxy & 0x03);
	*y = ((my << 2) & 0x3fc) + ((lxy & 0x0c) >> 2);

	return 0;
}

int micco_tsi_enable_pen(int pen_en)
{
	int status;
	u8 val;

	status = micco_read(MICCO_ADC_AUTO_CONTROL_2, &val);
	if (status)
		return -EIO;

	if (pen_en)
		val |= MICCO_ADC_AUTO_2_PENDET_EN;
	else
		val &= ~MICCO_ADC_AUTO_2_PENDET_EN;

	status = micco_write(MICCO_ADC_AUTO_CONTROL_2, val);
	if (status)
		return -EIO;

	return 0;
}

int micco_tsi_enable_tsi(int tsi_en)
{
	int status;
	u8 val;

	status = micco_read(MICCO_ADC_AUTO_CONTROL_2, &val);
	if (status)
		return -EIO;

	if (tsi_en)
		val |= MICCO_ADC_AUTO_2_TSI_EN;
	else
		val &= ~MICCO_ADC_AUTO_2_TSI_EN;

	status = micco_write(MICCO_ADC_AUTO_CONTROL_2, val);
	if (status)
		return -EIO;

	return 0;
}
/* Micco TSI functions end */

/* Micco Audio functions */
static volatile u8 micco_audio_regs[MICCO_AUDIO_REGS_NUM];

int micco_codec_read(u8 reg, u8 *val)
{
	return micco_read(MICCO_AUDIO_REG_BASE + reg, val);
}

int micco_codec_write(u8 reg, u8 val)
{
	return micco_write(MICCO_AUDIO_REG_BASE + reg, val);
}

void micco_read_codec(void)
{
	int i;
	u8 val;

	for (i = 0; i < MICCO_AUDIO_REGS_NUM; i++) {
		micco_codec_read(i, &val);
		micco_audio_regs[i] = val;
	}
}

void micco_dump_codec(void)
{
	int i;

	micco_read_codec();

	for (i = 0; i < MICCO_AUDIO_REGS_NUM; i++) {
		printk(KERN_ALERT "%s: Micco_audio_reg[%d] = 0x%x\n",
			__func__, i, micco_audio_regs[i]);
	}
}
EXPORT_SYMBOL(micco_dump_codec);

int micco_audio_init(void)
{
	int i;

	/* The default setting for Micco */
	micco_audio_regs[MICCO_MUX_MONO] = 0x00; /* MONO */
	micco_audio_regs[MICCO_MUX_BEAR] = 0x00; /* BEAR */
	micco_audio_regs[MICCO_MUX_LINE_OUT] = 0x00; /* LINE OUT */
	micco_audio_regs[MICCO_MUX_STEREO_CH1] =
		0x00;	/* STEREO_CH1 */
	micco_audio_regs[MICCO_MUX_STEREO_CH2] =
		0x00;	/* STEREO_CH2 */

	micco_audio_regs[MICCO_AUDIO_LINE_AMP] =
		MICCO_AUDIO_LINE_AMP_EN;	/* Enable the Line AMP */

	/* Gain for both channel controlled separately. Enable Setero */
	micco_audio_regs[MICCO_STEREO_AMPLITUDE_CH1] =
		MICCO_STEREO_GAIN_SEPARATE | MICCO_STEREO_EN;

	/* Soft startup of the Stereo amplifiers */
	micco_audio_regs[MICCO_STEREO_AMPLITUDE_CH2] = 0x0;

	micco_audio_regs[MICCO_HIFI_DAC_CONTROL] =
		MICCO_HIFI_DAC_ON;

	micco_audio_regs[MICCO_MONO_VOL] =
		MICCO_MONO_EN | 0xa;
	micco_audio_regs[MICCO_BEAR_VOL] =
		MICCO_BEAR_EN | 0xa;

	/* Micco as I2S slave. Use I2S MSB normal mode */
	micco_audio_regs[MICCO_I2S_CONTROL] =
		MICCO_I2S_MSB_JU_MODE;

	micco_audio_regs[MICCO_TX_PGA] =
		0x0c;	/* 0 dB */
	micco_audio_regs[MICCO_MIC_PGA] =
		MICCO_MIC_PGA_EXT_EN | MICCO_MIC_PGA_INT_EN |
		MICCO_MIC_PGA_SELMIC_2 | MICCO_MIC_PGA_AMP_EN |
		0x7;	/* 30 dB*/

	micco_audio_regs[MICCO_TX_PGA_MUX] =
		0xFF;

	micco_audio_regs[MICCO_VCODEC_ADC_CONTROL] =
		MICCO_VCODEC_ADC_ON_EN | 0x08;
	/* PCM_SDI normal operation, PCM_SDO enabled */
	micco_audio_regs[MICCO_VCODEC_VDAC_CONTROL] =
		MICCO_VDAC_ON | MICCO_VDAC_HPF_BYPASS;

	micco_audio_regs[MICCO_SIDETONE] =
		MICCO_SIDETONE_EN | MICCO_SIDETONE_GAIN_STEREO | 0x08;

	/* Enable AUX1,2. AUX1, 2 gain 0dB */
	micco_audio_regs[MICCO_PGA_AUX1_2] =
		MICCO_PGA_AUX1_EN | MICCO_PGA_AUX2_EN;

	/* Enable AUX3. AUX3 gain 0 dB */
	micco_audio_regs[MICCO_PGA_AUX3] =
		MICCO_PGA_AUX3_EN;

	/* DAC1, 2, 3 gain 0dB */
	micco_audio_regs[MICCO_PGA_DACS] =
		0x00;

	/*Soft start for MONO, BEAR LINE and STEREO is 61.5ms */
	micco_audio_regs[MICCO_SOFT_START_RAMP] =
		0x00;

	for (i = 0; i < MICCO_AUDIO_REGS_NUM; i++)
		micco_codec_write(i, micco_audio_regs[i]);

	return 0;
}
EXPORT_SYMBOL(micco_audio_init);

/* FIXME: The Stereo have left and right channel. Need add it later */
int micco_codec_enable_output(int type)
{
	switch (type) {
	case CODEC_BEAR:
		if (!(micco_audio_regs[MICCO_BEAR_VOL] & MICCO_BEAR_EN)) {
			micco_audio_regs[MICCO_BEAR_VOL] |=  MICCO_BEAR_EN;
			micco_codec_write(MICCO_BEAR_VOL,
				 micco_audio_regs[MICCO_BEAR_VOL]);
		}
		break;

	case CODEC_MONO:
		if (!(micco_audio_regs[MICCO_MONO_VOL] & MICCO_MONO_EN)) {
			micco_audio_regs[MICCO_MONO_VOL] |= MICCO_MONO_EN;
			micco_codec_write(MICCO_MONO_VOL,
				micco_audio_regs[MICCO_MONO_VOL]);
		}
		break;

	case CODEC_STEREO:
		if (!(micco_audio_regs[MICCO_STEREO_AMPLITUDE_CH1] &
				MICCO_STEREO_EN)){
			micco_audio_regs[MICCO_STEREO_AMPLITUDE_CH1] |=
				MICCO_STEREO_EN;
			micco_codec_write(MICCO_STEREO_AMPLITUDE_CH1,
				micco_audio_regs[MICCO_STEREO_AMPLITUDE_CH1]);
		}
		break;

	case CODEC_LINE_OUT:
		if (!(micco_audio_regs[MICCO_AUDIO_LINE_AMP] &
				MICCO_AUDIO_LINE_AMP_EN)) {
			micco_audio_regs[MICCO_AUDIO_LINE_AMP] |=
				MICCO_AUDIO_LINE_AMP_EN;
			micco_codec_write(MICCO_AUDIO_LINE_AMP,
				micco_audio_regs[MICCO_AUDIO_LINE_AMP]);
		}
		break;

	case CODEC_ADC:
		if (!(micco_audio_regs[MICCO_VCODEC_ADC_CONTROL] &
				MICCO_VCODEC_ADC_ON_EN)) {
			micco_audio_regs[MICCO_VCODEC_ADC_CONTROL] |=
				MICCO_VCODEC_ADC_ON_EN;
			micco_codec_write(MICCO_VCODEC_ADC_CONTROL,
				micco_audio_regs[MICCO_VCODEC_ADC_CONTROL]);
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(micco_codec_enable_output);

int micco_codec_disable_output(int type)
{
	switch (type) {
	case CODEC_BEAR:
		if (micco_audio_regs[MICCO_BEAR_VOL] & MICCO_BEAR_EN) {
			micco_audio_regs[MICCO_BEAR_VOL] &=  ~MICCO_BEAR_EN;
			micco_codec_write(MICCO_BEAR_VOL,
				 micco_audio_regs[MICCO_BEAR_VOL]);
		}
		break;

	case CODEC_MONO:
		if (micco_audio_regs[MICCO_MONO_VOL] & MICCO_MONO_EN) {
			micco_audio_regs[MICCO_MONO_VOL] &= ~MICCO_MONO_EN;
			micco_codec_write(MICCO_MONO_VOL,
				micco_audio_regs[MICCO_MONO_VOL]);
		}
		break;

	case CODEC_STEREO:
		if (micco_audio_regs[MICCO_STEREO_AMPLITUDE_CH1] &
				 MICCO_STEREO_EN){
			micco_audio_regs[MICCO_STEREO_AMPLITUDE_CH1] &=
				~MICCO_STEREO_EN;
			micco_codec_write(MICCO_STEREO_AMPLITUDE_CH1,
				micco_audio_regs[MICCO_STEREO_AMPLITUDE_CH1]);
		}
		break;

	case CODEC_LINE_OUT:
		if (micco_audio_regs[MICCO_AUDIO_LINE_AMP] &
				MICCO_AUDIO_LINE_AMP_EN) {
			micco_audio_regs[MICCO_AUDIO_LINE_AMP] &=
				~MICCO_AUDIO_LINE_AMP_EN;
			micco_codec_write(MICCO_AUDIO_LINE_AMP,
				micco_audio_regs[MICCO_AUDIO_LINE_AMP]);
		}
		break;

	case CODEC_ADC:
		if (micco_audio_regs[MICCO_VCODEC_ADC_CONTROL] &
				MICCO_VCODEC_ADC_ON_EN) {
			micco_audio_regs[MICCO_VCODEC_ADC_CONTROL] &=
				~MICCO_VCODEC_ADC_ON_EN;
			micco_codec_write(MICCO_VCODEC_ADC_CONTROL,
				micco_audio_regs[MICCO_VCODEC_ADC_CONTROL]);
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/* We don't check the paramater. The caller need make sure
 * that the vol is in legal range.
 */
int micco_codec_set_output_vol(int type, int vol)
{
	switch (type) {
	case CODEC_BEAR:
		micco_audio_regs[MICCO_BEAR_VOL] = (~0x3F &
			micco_audio_regs[MICCO_BEAR_VOL]) | vol;
		micco_codec_write(MICCO_BEAR_VOL,
			micco_audio_regs[MICCO_BEAR_VOL]);
		break;

	case CODEC_MONO:
		micco_audio_regs[MICCO_MONO_VOL] = (~0x3F &
			micco_audio_regs[MICCO_MONO_VOL]) | vol;
		micco_codec_write(MICCO_MONO_VOL,
			micco_audio_regs[MICCO_MONO_VOL]);

	case CODEC_STEREO:
		micco_audio_regs[MICCO_STEREO_AMPLITUDE_CH1] =
			(~0x3F & micco_audio_regs[MICCO_STEREO_AMPLITUDE_CH1]) |
			vol;
		micco_codec_write(MICCO_STEREO_AMPLITUDE_CH1,
			micco_audio_regs[MICCO_STEREO_AMPLITUDE_CH1]);
		break;

	case CODEC_LINE_OUT:
		micco_audio_regs[MICCO_AUDIO_LINE_AMP] =
			(~0x0F & micco_audio_regs[MICCO_AUDIO_LINE_AMP]) |
			vol;
		micco_codec_write(MICCO_AUDIO_LINE_AMP,
			micco_audio_regs[MICCO_AUDIO_LINE_AMP]);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(micco_codec_set_output_vol);


int micco_codec_enable_input(int type)
{
	switch (type) {
	case CODEC_AUX1:
		if (!(micco_audio_regs[MICCO_PGA_AUX1_2] & MICCO_PGA_AUX1_EN)) {
			micco_audio_regs[MICCO_PGA_AUX1_2] |= MICCO_PGA_AUX1_EN;
			micco_codec_write(MICCO_PGA_AUX1_2,
				micco_audio_regs[MICCO_PGA_AUX1_2]);
		}
		break;

	case CODEC_AUX2:
		if (!(micco_audio_regs[MICCO_PGA_AUX1_2] & MICCO_PGA_AUX2_EN)) {
			micco_audio_regs[MICCO_PGA_AUX1_2] |= MICCO_PGA_AUX2_EN;
			micco_codec_write(MICCO_PGA_AUX1_2,
				micco_audio_regs[MICCO_PGA_AUX1_2]);
		}
		break;

	case CODEC_AUX3:
		if (!(micco_audio_regs[MICCO_PGA_AUX3] & MICCO_PGA_AUX3_EN)) {
			micco_audio_regs[MICCO_PGA_AUX3] |= MICCO_PGA_AUX3_EN;
			micco_codec_write(MICCO_PGA_AUX3,
				micco_audio_regs[MICCO_PGA_AUX3]);
		}
		break;

	case CODEC_MIC1:
		micco_audio_regs[MICCO_MIC_PGA] &= ~MICCO_MIC_PGA_SELMIC_2;
		micco_audio_regs[MICCO_MIC_PGA] =
		       micco_audio_regs[MICCO_MIC_PGA] |
		       MICCO_MIC_PGA_EXT_EN | MICCO_MIC_PGA_INT_EN |
		       MICCO_MIC_PGA_AMP_EN;
		micco_codec_write(MICCO_MIC_PGA,
			micco_audio_regs[MICCO_MIC_PGA]);
		break;

	case CODEC_MIC2:
		micco_audio_regs[MICCO_MIC_PGA] |= MICCO_MIC_PGA_SELMIC_2;
		micco_audio_regs[MICCO_MIC_PGA] =
		       micco_audio_regs[MICCO_MIC_PGA] |
		       MICCO_MIC_PGA_EXT_EN | MICCO_MIC_PGA_INT_EN |
		       MICCO_MIC_PGA_AMP_EN;
		micco_codec_write(MICCO_MIC_PGA,
			micco_audio_regs[MICCO_MIC_PGA]);
		break;

	case CODEC_PCM:
		if (!(micco_audio_regs[MICCO_VCODEC_VDAC_CONTROL] &
				MICCO_VDAC_ON)) {
			micco_audio_regs[MICCO_VCODEC_VDAC_CONTROL] |=
				MICCO_VDAC_ON;
			micco_audio_regs[MICCO_VCODEC_VDAC_CONTROL] &=
				~MICCO_VDAC_HPF_MUTE;
			micco_codec_write(MICCO_VCODEC_VDAC_CONTROL,
				micco_audio_regs[MICCO_VCODEC_VDAC_CONTROL]);
		}
		break;

	case CODEC_HIFI:
		if (!(micco_audio_regs[MICCO_HIFI_DAC_CONTROL] &
				MICCO_HIFI_DAC_ON)) {
			micco_audio_regs[MICCO_HIFI_DAC_CONTROL] |=
				MICCO_HIFI_DAC_ON;
			micco_codec_write(MICCO_HIFI_DAC_CONTROL,
				micco_audio_regs[MICCO_HIFI_DAC_CONTROL]);
		}
		break;

	default:
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(micco_codec_enable_input);

int micco_codec_disable_input(int type)
{
	switch (type) {
	case CODEC_AUX1:
		if (micco_audio_regs[MICCO_PGA_AUX1_2] & MICCO_PGA_AUX1_EN) {
			micco_audio_regs[MICCO_PGA_AUX1_2] &=
				~MICCO_PGA_AUX1_EN;
			micco_codec_write(MICCO_PGA_AUX1_2,
				micco_audio_regs[MICCO_PGA_AUX1_2]);
		}
		break;

	case CODEC_AUX2:
		if (micco_audio_regs[MICCO_PGA_AUX1_2] & MICCO_PGA_AUX2_EN) {
			micco_audio_regs[MICCO_PGA_AUX1_2] &=
				~MICCO_PGA_AUX2_EN;
			micco_codec_write(MICCO_PGA_AUX1_2,
				micco_audio_regs[MICCO_PGA_AUX1_2]);
		}
		break;

	case CODEC_AUX3:
		if (micco_audio_regs[MICCO_PGA_AUX3] & MICCO_PGA_AUX3_EN) {
			micco_audio_regs[MICCO_PGA_AUX3] &= ~MICCO_PGA_AUX3_EN;
			micco_codec_write(MICCO_PGA_AUX3,
				micco_audio_regs[MICCO_PGA_AUX3]);
		}
		break;

	case CODEC_MIC1:
		micco_audio_regs[MICCO_MIC_PGA] =
			micco_audio_regs[MICCO_MIC_PGA] &
			~(MICCO_MIC_PGA_EXT_EN | MICCO_MIC_PGA_INT_EN |
				MICCO_MIC_PGA_AMP_EN);
		micco_codec_write(MICCO_MIC_PGA,
				micco_audio_regs[MICCO_MIC_PGA]);
		break;

	case CODEC_MIC2:
		micco_audio_regs[MICCO_MIC_PGA] =
			micco_audio_regs[MICCO_MIC_PGA] &
			~(MICCO_MIC_PGA_EXT_EN | MICCO_MIC_PGA_INT_EN |
				MICCO_MIC_PGA_AMP_EN);
		micco_codec_write(MICCO_MIC_PGA,
				micco_audio_regs[MICCO_MIC_PGA]);
		break;

	case CODEC_PCM:
		if (micco_audio_regs[MICCO_VCODEC_VDAC_CONTROL] &
				MICCO_VDAC_ON) {
			micco_audio_regs[MICCO_VCODEC_VDAC_CONTROL] &=
				~MICCO_VDAC_ON;
			micco_codec_write(MICCO_VCODEC_VDAC_CONTROL,
				micco_audio_regs[MICCO_VCODEC_VDAC_CONTROL]);
		}
		break;

	case CODEC_HIFI:
		if (micco_audio_regs[MICCO_HIFI_DAC_CONTROL] &
				MICCO_HIFI_DAC_ON) {
			micco_audio_regs[MICCO_HIFI_DAC_CONTROL] &=
				~MICCO_HIFI_DAC_ON;
			micco_codec_write(MICCO_HIFI_DAC_CONTROL,
				micco_audio_regs[MICCO_HIFI_DAC_CONTROL]);
		}
		break;

	default:
		return -EINVAL;
	}
	return 0;
}

/* We don't check the paramater. The caller need make sure
 * that the vol is in legal range.
 */
int micco_codec_set_input_gain(int type, int gain)
{
	switch (type) {
	case CODEC_AUX1:
		micco_audio_regs[MICCO_PGA_AUX1_2] = ((~0x03 &
			micco_audio_regs[MICCO_PGA_AUX1_2]) | gain);
		micco_codec_write(MICCO_PGA_AUX1_2,
			micco_audio_regs[MICCO_PGA_AUX1_2]);
		break;

	case CODEC_AUX2:
		micco_audio_regs[MICCO_PGA_AUX1_2] = ((~0x30 &
			micco_audio_regs[MICCO_PGA_AUX1_2]) | (gain << 4));
		micco_codec_write(MICCO_PGA_AUX1_2,
			micco_audio_regs[MICCO_PGA_AUX1_2]);
		break;

	case CODEC_AUX3:
		micco_audio_regs[MICCO_PGA_AUX3] = ((~0x03 &
			micco_audio_regs[MICCO_PGA_AUX3]) | gain);
		micco_codec_write(MICCO_PGA_AUX3,
			micco_audio_regs[MICCO_PGA_AUX3]);
		break;

	case CODEC_MIC1:
	case CODEC_MIC2:
		micco_audio_regs[MICCO_MIC_PGA] = ((~0x07 &
			micco_audio_regs[MICCO_MIC_PGA]) | gain);
		micco_codec_write(MICCO_MIC_PGA,
			micco_audio_regs[MICCO_MIC_PGA]);
		break;

	case CODEC_PCM:	/* Need check whether HIFI and PCM support input? */
	case CODEC_HIFI:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

int micco_codec_set_sample_rate(int port, int rate)
{
	switch (port) {
	case MICCO_VOICE_PORT:
		switch (rate) {
		case 8000:
			micco_audio_regs[MICCO_VCODEC_ADC_CONTROL] =
				(~(0x03 << 3) &
				 micco_audio_regs[MICCO_VCODEC_ADC_CONTROL]);
			break;

		case 16000:
			micco_audio_regs[MICCO_VCODEC_ADC_CONTROL] =
				(~(0x03 << 3) &
				 micco_audio_regs[MICCO_VCODEC_ADC_CONTROL]) |
				(0x01 << 3);
			break;

		case 32000:
			micco_audio_regs[MICCO_VCODEC_ADC_CONTROL] |=
				(0x3 << 3);
			break;
		default:
			return -EINVAL;
		}
		micco_codec_write(MICCO_VCODEC_ADC_CONTROL,
			micco_audio_regs[MICCO_VCODEC_ADC_CONTROL]);
		break;

	case MICCO_HIFI_PORT:
		switch (rate) {
		case 8000:
			micco_audio_regs[MICCO_I2S_CONTROL] &= 0xF0;
			break;
		case 11025:
			micco_audio_regs[MICCO_I2S_CONTROL] &= 0xF0;
			micco_audio_regs[MICCO_I2S_CONTROL] |= 0x01;
			break;
		case 12000:
			micco_audio_regs[MICCO_I2S_CONTROL] &= 0xF0;
			micco_audio_regs[MICCO_I2S_CONTROL] |= 0x02;
			break;
		case 16000:
			micco_audio_regs[MICCO_I2S_CONTROL] &= 0xF0;
			micco_audio_regs[MICCO_I2S_CONTROL] |= 0x03;
			break;
		case 22050:
			micco_audio_regs[MICCO_I2S_CONTROL] &= 0xF0;
			micco_audio_regs[MICCO_I2S_CONTROL] |= 0x04;
			break;
		case 24000:
			micco_audio_regs[MICCO_I2S_CONTROL] &= 0xF0;
			micco_audio_regs[MICCO_I2S_CONTROL] |= 0x05;
			break;
		case 32000:
			micco_audio_regs[MICCO_I2S_CONTROL] &= 0xF0;
			micco_audio_regs[MICCO_I2S_CONTROL] |= 0x06;
			break;
		case 44100:
			micco_audio_regs[MICCO_I2S_CONTROL] &= 0xF0;
			micco_audio_regs[MICCO_I2S_CONTROL] |= 0x07;
			break;
		case 48000:
			micco_audio_regs[MICCO_I2S_CONTROL] &= 0xF0;
			micco_audio_regs[MICCO_I2S_CONTROL] |= 0x0F;
			break;
		default:
			return -EINVAL;
		}
		micco_codec_write(MICCO_I2S_CONTROL,
			micco_audio_regs[MICCO_I2S_CONTROL]);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(micco_codec_set_sample_rate);

/* micco_set_vibrator parameter:
 *    SWITCH mode, repetition frequency controled by VIBRA_PWM<6:0>
 * 	00h : OFF
 * 	>= 54h: 100% alwayes on (code 84 decimal)
 */
int micco_set_vibrator(unsigned char value)
{
	micco_write(MICCO_VIBRA_CONTROL, value & 0x3F);
	return 0;
}
EXPORT_SYMBOL(micco_set_vibrator);

int micco_sidetone_enable(void)
{
	if (!(micco_audio_regs[MICCO_SIDETONE] & MICCO_SIDETONE_EN)) {
		micco_audio_regs[MICCO_SIDETONE] |= MICCO_SIDETONE_EN;
		micco_codec_write(MICCO_SIDETONE,
			micco_audio_regs[MICCO_SIDETONE]);
	}

	return 0;
}

int micco_sidetone_disable(void)
{
	if (micco_audio_regs[MICCO_SIDETONE] & MICCO_SIDETONE_EN) {
		micco_audio_regs[MICCO_SIDETONE] &= ~MICCO_SIDETONE_EN;
		micco_codec_write(MICCO_SIDETONE,
			micco_audio_regs[MICCO_SIDETONE]);
	}

	return 0;
}

/* The Micco route rule:
 * 1. DAC3 can't be route to TX_PGA.
 * 2. MIC1 and MIC2 can't be enabled at the same time.
 */
int micco_check_route(u16 *routemap)
{
/* TODO */
	return 0;
}

int micco_set_route(u16 *rmap, u16 *current_rmap)
{
	int ret;

	ret = micco_check_route(current_rmap);
	if (ret)
		return ret;

	return 0;
}
/* Micco Audio primitive end here */

/* Initialization functions for system */
/* LDO12 enabled for IO */
void micco_enable_LDO12(void)
{
	u8 val;

	micco_read(MICCO_LDO1312, &val);
	val = (0x0F & val);
	micco_write(MICCO_LDO1312, val);

	return;
}

/* USB Device/OTG functions */
static int micco_set_pump(int enable)
{
	int status = 0;
	u8 val;
	unsigned long flags;
	static int set_pump_count;

	local_irq_save(flags);
	if (enable) {
		if (set_pump_count++ != 0)
			goto out;
		status = micco_read(MICCO_MISC, &val);
		if (status)
			goto out;
		val |= MICCO_MISC_VBUS_COMPS_EN;
		status = micco_write(MICCO_MISC, val);
		if (status)
			goto out;

		/* FIXME: Littleton didn't connect EXTON as cable detection
		 * signal. We use USB_DEV detection in event B as cable
		 * detection.
		 */
		/* TODO: This is a platform related code. Need split
		 * to platform related code.
		 */
		status = micco_read(MICCO_IRQ_MASK_B, &val);
		if (status)
			goto out;
		val &= ~IRQ_MASK_B_USB_DEV;
		status = micco_write(MICCO_IRQ_MASK_B, val);
		if (status)
			goto out;
	} else {
		if (set_pump_count == 0 || set_pump_count-- != 0)
			goto out;

		status = micco_read(MICCO_IRQ_MASK_B, &val);
		if (status)
			goto out;
		val |= (IRQ_MASK_B_SESSION_VALID_1_8 | IRQ_MASK_B_VBUS_VALID_3_8
			| IRQ_MASK_B_VBUS_VALID_4_55);
		status = micco_write(MICCO_IRQ_MASK_B, val);
		if (status)
			goto out;

		status = micco_read(MICCO_MISC, &val);
		if (status)
			goto out;
		val &= ~MICCO_MISC_VBUS_COMPS_EN;
		status = micco_write(MICCO_MISC, val);
		if (status)
			goto out;

		status = micco_read(MICCO_IRQ_MASK_B, &val);
		if (status)
			goto out;
		val |= IRQ_MASK_B_USB_DEV;
		status = micco_write(MICCO_IRQ_MASK_B, val);
		if (status)
			goto out;
	}
out:
	local_irq_restore(flags);
	return status;
}

/* 1. enable: 1, srp: 0, 100ma mode
 * 2. enable: 1, srp: 1, 10ma mode
 *
 * Micco before e: 1:s: 1, must e:0, s:1.
 */
static int micco_set_vbus_supply(int enable, int srp)
{
	int ret;
	u8 val, tmp;

	ret = micco_read(MICCO_MISC, &val);
	if (ret) {
		printk(KERN_ERR "I2C operation error 0x%x\n", ret);
		return ret;
	}

	if (enable) {
		/* When enable the USB pump, need check the OTGCP_IOVER. */
		micco_read(MICCO_IRQ_MASK_B, &tmp);
		tmp &= ~IRQ_MASK_B_OTGCP_IOVER;
		micco_write(MICCO_IRQ_MASK_B, tmp);

		val |= MICCO_MISC_USBCP_EN;
		if (srp) {
			val |= MICCO_MISC_USBSR_EN;
		} else {
			val &= ~MICCO_MISC_USBSR_EN;
		}
	} else {
		/* When disable the USB pump, needn't check the OTGCP_IOVER. */
		micco_read(MICCO_IRQ_MASK_B, &tmp);
		tmp |= IRQ_MASK_B_OTGCP_IOVER;
		micco_write(MICCO_IRQ_MASK_B, tmp);

		val &= ~(MICCO_MISC_USBCP_EN | MICCO_MISC_USBSR_EN);
	}
	ret = micco_write(MICCO_MISC, val);
	if (ret)
		printk(KERN_ERR "I2C operation error 0x%x\n", ret);

	return ret;
}

static int micco_set_usbotg_a_mask(void)
{
	int ret;
	u8 val;

	ret = micco_read(MICCO_IRQ_MASK_B, &val);
	if (ret) {
		printk(KERN_ERR "I2C operation error 0x%x\n", ret);
		return ret;
	}

	/* Enable the interrupt for VUBS > 4.4V and Session valid
	 * which A device care about.
	 *
	 * NOTESSSSSSSSS: We care about the SRP detection signal (0.8v ~ 2.0v)
	 * on Micco. Which map to SESSION_VALID_1_8.
	 */
	val |= (IRQ_MASK_B_VBUS_VALID_3_8 | IRQ_MASK_B_SRP_READY_0_6);
	val &= ~(IRQ_MASK_B_VBUS_VALID_4_55 | IRQ_MASK_B_SESSION_VALID_1_8);

	ret = micco_write(MICCO_IRQ_MASK_B, val);
	return ret;
}

static int micco_set_usbotg_b_mask(void)
{
	int ret;
	u8 val;

	ret = micco_read(MICCO_IRQ_MASK_B, &val);
	if (ret) {
		printk(KERN_ERR "I2C operation error 0x%x\n", ret);
		return ret;
	}

	/* Mask all USB VBUS interrupt for B device */
	val |= (IRQ_MASK_B_VBUS_VALID_3_8 | IRQ_MASK_B_SESSION_VALID_1_8 | \
		IRQ_MASK_B_VBUS_VALID_4_55 | IRQ_MASK_B_SRP_READY_0_6);

	ret = micco_write(MICCO_IRQ_MASK_B, val);

	return ret;
}

static int is_micco_vbus_assert(void)
{
	int ret;
	u8 val, level = MICCO_STATUS_B_VBUS_V_4P4;

	ret = micco_read(MICCO_STATUS_B, &val);
	if (ret) {
		printk(KERN_ERR "I2C operation error 0x%x\n", ret);
		return ret;
	}
	
	/* USBDEV interrupt is not so reliable for vbus assert check,
 	 * replace with 4.4V check 
 	 *
	if (val & MICCO_STATUS_B_USBDEV) */
	if (cpu_is_pxa910())
		level = MICCO_STATUS_B_VBUS_V_3P8;
		
	if (val & level)
		return 1;
	else
		return 0;
}

#ifdef	CONFIG_LITTLETON_CHARGER
#define	CHARGER_HOOK
#else
#undef	CHARGER_HOOK
#endif

static unsigned int micco_event_change(void)
{
	unsigned int ret = 0;
	u8 val, mask;

	micco_read(MICCO_EVENT_A, &val);
	if (val & MICCO_EA_CHDET)
		ret |= PMIC_EVENT_CHDET;

	if (val & MICCO_EA_REV_IOVER)
		ret |= PMIC_EVENT_REV_IOVER;

	if (val & MICCO_EA_IOVER)
		ret |= PMIC_EVENT_IOVER;

	if (val & MICCO_EA_TBAT)
		ret |= PMIC_EVENT_TBAT;

	if (val & MICCO_EA_VBATMON)
		ret |= PMIC_EVENT_VBATMON;

	micco_read(MICCO_EVENT_B, &val);
	if (val & MICCO_EB_USB_DEV)
		ret |= PMIC_EVENT_VBUS;

	if (val & (MICCO_EB_VBUS_4P55|MICCO_EB_VBUS_3P8))
		ret |= PMIC_EVENT_VBUS;

	if (val & MICCO_EB_SESSION_1P8) {
		micco_read(MICCO_IRQ_MASK_B, &mask);
		if (!(mask & IRQ_MASK_B_SESSION_VALID_1_8))
			ret |= PMIC_EVENT_VBUS;
	}

	if (val & MICCO_EB_OTGCP_IOVER)
		ret |= PMIC_EVENT_OTGCP_IOVER;

	micco_read(MICCO_EVENT_C, &val);
	if (val & MICCO_EC_PEN_DOWN)
		ret |= PMIC_EVENT_TOUCH;

	micco_read(MICCO_EVENT_D, &val);
	if (val & MICCO_ED_HEADSET)
	{
		ret |= PMIC_EVENT_HSDETECT;
	}

	if (val & MICCO_ED_HOOKSWITCH)
	{
		ret |= PMIC_EVENT_HOOKSWITCH;
	}
	return ret;
}

static int is_micco_avbusvld(void)
{
	u8 val;

	micco_read(MICCO_IRQ_MASK_B, &val);
	if (val & IRQ_MASK_B_VBUS_VALID_4_55)
		return 0;

	micco_read(MICCO_STATUS_B, &val);
	if (val & MICCO_STATUS_B_VBUS_V_4P4)
		return 1;
	else
		return 0;
}

static int is_micco_asessvld(void)
{
	u8 val;

	micco_read(MICCO_IRQ_MASK_B, &val);
	if (val & IRQ_MASK_B_SESSION_VALID_1_8)
		return 0;

	micco_read(MICCO_STATUS_B, &val);
	if (val & MICCO_STATUS_B_SESS_V_1P8)
		return 1;
	else
		return 0;
}

static int is_micco_bsessvld(void)
{
	u8 val;

	micco_read(MICCO_IRQ_MASK_B, &val);
	if (val & IRQ_MASK_B_VBUS_VALID_3_8)
		return 0;

	micco_read(MICCO_STATUS_B, &val);
	if (val & MICCO_STATUS_B_VBUS_V_3P8)
		return 1;
	else
		return 0;
}

static int is_micco_srp_ready(void)
{
	u8 val;

	micco_read(MICCO_IRQ_MASK_B, &val);
	if (val & IRQ_MASK_B_SRP_READY_0_6)
		return 0;

	/* FIXME: When cable detached, the SESS Valid
	 * of STATUA B can not change to 0 immediately.
	 * That will cause potential problems.
	 */
	micco_read(MICCO_STATUS_B, &val);
	if (val & MICCO_STATUS_B_SRP_READY)
		return 1;
	else
		return 0;
}
static int get_power_supply_module(int cmd)
{
	int command, power_module;

	if (cmd < VCC_CORE || cmd >= MAX_CMD) {
		printk(KERN_WARNING "input wrong command: %d\n", cmd);
		return -EINVAL;
	}
	command = micco_power_module[cmd - VCC_CORE].command;
	if (command != cmd) {
		printk(KERN_WARNING "micco_power_module[] is error: %d\n", cmd);
		return -EINVAL;
	}
	power_module = micco_power_module[cmd - VCC_CORE].power_module;
	if (power_module == 0) {
		printk(KERN_WARNING "the command not supported: %d\n", cmd);
		return -EINVAL;
	}

	return power_module;
}

/* USB Device/OTG functions end here*/
static int get_micco_voltage(int cmd, int *pmv)
{
	int power_module, status = 0;
	u8 val;

	*pmv = 0;

	power_module = get_power_supply_module(cmd);

	start_calc_time();
	switch (power_module) {
	case BUCK1:
		status = micco_read(MICCO_BUCK1_DVC2, &val);
		if (status)
			return status;

		val &= 0x1f;
		*pmv = val * MICCO_VBUCK1_STEP + MICCO_VBUCK1_BASE;
		break;
	case BUCK2: /* TODO */
		*pmv = 1800;
		break;
	case BUCK3: /* TODO */
		*pmv = 1800;
		break;
	case LDO1:
		status = micco_read(MICCO_LDO1_MDTV1, &val);
		if (status)
			return status;

		val &= 0x0f;
		*pmv = val * MICCO_VLDO1_STEP + MICCO_VLDO1_BASE;
		break;
	case LDO2:
		status = micco_read(MICCO_SRAM_DVC2, &val);
		if (status)
			return status;

		val = val & 0x1f;
		*pmv = val * MICCO_VSRAM_STEP + MICCO_VSRAM_BASE;
		break;
	case LDO3:
		status = micco_read(MICCO_LDO643, &val);
		if (status)
			return status;

		val &= 0x0f;
		*pmv = val * MICCO_VLDO3_STEP + MICCO_VLDO3_BASE;
		break;
	case LDO4:
		status = micco_read(MICCO_LDO643, &val);
		if (status)
			return status;

		if (val & 0x10)
			*pmv = 2900;
		else
			*pmv = 1800;
		break;
	case LDO5: /* TODO */
		*pmv = 3100;
		break;
	case LDO6:
		status = micco_read(MICCO_LDO643, &val);
		if (status)
			return status;

		val = (val & 0xE0) >> 5;
		*pmv = val * MICCO_VLDO6_STEP + MICCO_VLDO6_BASE;
		break;
	case LDO7: /* TODO */
		printk(KERN_WARNING "power supply module TODO: %d\n", cmd);
		return -EINVAL;
	case LDO8: /* TODO */
		printk(KERN_WARNING "power supply module TODO: %d\n", cmd);
		return -EINVAL;
	case LDO9:
		status = micco_read(MICCO_LDO987, &val);
		if (status)
			return status;

		val = (val & 0xE0) >> 5;
		*pmv = val * MICCO_VLDO9_STEP + MICCO_VLDO9_BASE;
		break;
	case LDO10:
		status = micco_read(MICCO_LDO987, &val);
		if (status)
			return status;

		val &= 0x07;
		*pmv = val * MICCO_VLDO10_STEP + MICCO_VLDO10_BASE;
		break;
	case LDO11:
		status = micco_read(MICCO_LDO1110, &val);
		if (status)
			return status;

		val = (val & 0xf0) >> 4;
		*pmv = val * MICCO_VLDO11_STEP + MICCO_VLDO11_BASE;
		break;
	case LDO12:
		status = micco_read(MICCO_LDO1312, &val);
		if (status)
			return status;

		val &= 0x07;
		if (val & 0x08)
			*pmv = val * MICCO_VLDO12_STEP + MICCO_VLDO12_BASE1;
		else
			*pmv = val * MICCO_VLDO12_STEP + MICCO_VLDO12_BASE;
		break;
	case LDO13:
		status = micco_read(MICCO_LDO1312, &val);
		if (status)
			return status;

		val = (val & 0xf0) >> 4;
		*pmv = val * MICCO_VLDO13_STEP + MICCO_VLDO13_BASE;
		break;
	case LDO14:
		status = micco_read(MICCO_LDO1514, &val);
		if (status)
			return status;

		val &= 0x0f;
		*pmv = val * MICCO_VLDO14_STEP + MICCO_VLDO14_BASE;
		break;
	case LDO15:
		status = micco_read(MICCO_LDO1514, &val);
		if (status)
			return status;

		val = (val & 0xF0) >> 4;
		*pmv = val * MICCO_VLDO15_STEP + MICCO_VLDO15_BASE;
		break;
	case USB_OTG: /* TODO */
	case LDO_GPADC: /* TODO */
	case LDO_AUDIO: /* TODO */
	case LDO_PMCORE: /* TODO */
	case LDO_BBAT: /* TODO */
		printk(KERN_WARNING "power supply module TODO: %d\n", cmd);
		return -EINVAL;
	default:
		printk(KERN_WARNING "input wrong command: %d\n", cmd);
		return -EINVAL;
	}
	end_calc_time();
	return status;
}

static int set_micco_voltage(int cmd, int mv)
{
	int power_module, status = 0;
	u8 val;

	power_module = get_power_supply_module(cmd);

	start_calc_time();
	switch (power_module) {
	case BUCK1:
		if ((mv < MICCO_VBUCK1_BASE) || (mv > MICCO_VBUCK1_MAX))
			return -EINVAL;
		status = micco_read(MICCO_BUCK1_DVC2, &val);
		if (status)
			return status;

		val &= 0xe0;
		val = val | ((mv - MICCO_VBUCK1_BASE) / MICCO_VBUCK1_STEP);
		status = micco_write(MICCO_BUCK1_DVC2, val);
		/* Update the voltage output. Otherwise, voltage won't change */
		status = micco_write(MICCO_VCC1, 0x33);
		break;
	case BUCK2: /* TODO */
		return 0;
	case BUCK3: /* TODO */
		return 0;
	case LDO1:
		if ((mv < MICCO_VLDO1_BASE) || (mv > MICCO_VLDO1_MAX))
			return -EINVAL;
		status = micco_read(MICCO_LDO1_MDTV1, &val);
		if (status)
			return status;

		val &= 0xf0;
		val = val | ((mv - MICCO_VLDO1_BASE) / MICCO_VLDO1_STEP);
		status = micco_write(MICCO_LDO1_MDTV1, val);
		break;
	case LDO2:
		if ((mv < MICCO_VSRAM_BASE) || mv > (MICCO_VSRAM_MAX))
			return -EINVAL;
		status = micco_read(MICCO_SRAM_DVC2, &val);
		if (status)
			return status;

		val &= 0xe0;
		val = val | ((mv - MICCO_VSRAM_BASE) / MICCO_VSRAM_STEP);
		status = micco_write(MICCO_SRAM_DVC2, val);
		/* Update the voltage output. Otherwise, voltage won't change */
		status = micco_write(MICCO_VCC1, 0x33);
		break;
	case LDO3:
		if ((mv < MICCO_VLDO3_BASE) || (mv > MICCO_VLDO3_MAX))
			return -EINVAL;

		status = micco_read(MICCO_LDO643, &val);
		if (status)
			return status;

		val &= 0xf0;
		val = val | ((mv - MICCO_VLDO3_BASE) / MICCO_VLDO3_STEP);
		status = micco_write(MICCO_LDO643, val);
		break;
	case LDO4:
		if ((mv != 2900) && (mv != 1800))
			return -EINVAL;
		status = micco_read(MICCO_LDO643, &val);
		if (2900 == mv)
			val |= 0x10;
		else
			val &= ~0x10;
		status = micco_write(MICCO_LDO643, val);
		break;
	case LDO5: /* TODO */
		return 0;
	case LDO6:
		if ((mv < MICCO_VLDO6_BASE) || (mv > MICCO_VLDO6_MAX))
			return -EINVAL;

		status = micco_read(MICCO_LDO643, &val);
		if (status)
			return status;

		val &= 0x1f;
		val |= ((mv - MICCO_VLDO6_BASE) / MICCO_VLDO6_STEP) << 5;
		status = micco_write(MICCO_LDO643, val);
		break;
	case LDO7:
		printk(KERN_WARNING "power supply module TODO: %d\n", cmd);
		return -EINVAL;
	case LDO8:
		printk(KERN_WARNING "power supply module TODO: %d\n", cmd);
		return -EINVAL;
	case LDO9:
		if ((mv < MICCO_VLDO9_BASE) || (mv > MICCO_VLDO9_MAX))
			return -EINVAL;
		status = micco_read(MICCO_LDO987, &val);
		if (status)
			return status;

		val &= 0x1f;
		val |= ((mv - MICCO_VLDO9_BASE) / MICCO_VLDO9_STEP) << 5;
		status = micco_write(MICCO_LDO987, val);
		break;
	case LDO10:
		if ((mv < MICCO_VLDO10_BASE) || (mv > MICCO_VLDO10_MAX))
			return -EINVAL;

		status = micco_read(MICCO_LDO1110, &val);
		if (status)
			return status;

		val &= 0xf8;
		val = val | ((mv - MICCO_VLDO10_BASE) / MICCO_VLDO10_STEP);
		status = micco_write(MICCO_LDO1110, val);
		break;
	case LDO11:
		if ((mv < MICCO_VLDO11_BASE) || (mv > MICCO_VLDO11_MAX))
			return -EINVAL;

		status = micco_read(MICCO_LDO1110, &val);
		if (status)
			return status;

		val &= 0x0f;
		val |= ((mv - MICCO_VLDO11_BASE) / MICCO_VLDO11_STEP) << 4;
		status = micco_write(MICCO_LDO1110, val);
		break;
	case LDO12:
		if ((mv < MICCO_VLDO12_BASE) || (mv > MICCO_VLDO12_MAX))
			return -EINVAL;


		status = micco_read(MICCO_LDO1312, &val);
		if (status)
			return status;

		val &= 0xf8;
		if (mv > MICCO_VLDO12_BASE1) {
			val = val | ((mv - MICCO_VLDO12_BASE1) /	\
				MICCO_VLDO12_STEP);
			val |= 0x08;
		} else {
			val = val | ((mv - MICCO_VLDO12_BASE) /		\
				MICCO_VLDO12_STEP);
			val &= ~0x08;
		}
		status = micco_write(MICCO_LDO1312, val);
		break;
	case LDO13:
		if ((mv < MICCO_VLDO13_BASE) || (mv > MICCO_VLDO13_MAX))
			return -EINVAL;

		status = micco_read(MICCO_LDO1312, &val);
		if (status)
			return status;

		val &= 0x0f;
		val |= ((mv - MICCO_VLDO13_BASE) / MICCO_VLDO13_STEP) << 4;
		status = micco_write(MICCO_LDO1312, val);
		break;
	case LDO14:
		if ((mv < MICCO_VLDO14_BASE) || (mv > MICCO_VLDO14_MAX))
			return -EINVAL;
		status = micco_read(MICCO_LDO1514, &val);
		if (status)
			return status;

		val &= 0xf0;
		val = val | ((mv - MICCO_VLDO14_BASE) / MICCO_VLDO14_STEP);
		status = micco_write(MICCO_LDO1514, val);
		break;
	case LDO15:
		if ((mv < MICCO_VLDO15_BASE) || (mv > MICCO_VLDO15_MAX))
			return -EINVAL;

		status = micco_read(MICCO_LDO1514, &val);
		if (status)
			return status;

		val &= 0x0f;
		val |= ((mv - MICCO_VLDO15_BASE) / MICCO_VLDO15_STEP) << 4;
		status = micco_write(MICCO_LDO1514, val);
		break;
	case USB_OTG: /* TODO */
	case LDO_GPADC: /* TODO */
	case LDO_AUDIO: /* TODO */
	case LDO_PMCORE: /* TODO */
	case LDO_BBAT: /* TODO */
		printk(KERN_WARNING "power supply module TODO: %d\n", cmd);
		return -EINVAL;
	default:
		printk(KERN_INFO "error command\n");
		return -EINVAL;
	}

	if (status != 0)
		return status;

	end_calc_time();
	return status;
}

static void micco_worker(struct work_struct *work)
{
	u8 val;

	if(!cpu_is_pxa910())
		event = micco_event_change();
	pmic_event_handle(event);

	/* We don't put these codes to USB specific code because
	 * we need handle it even when no USB callback registered.
	 */
	if (event & PMIC_EVENT_OTGCP_IOVER) {
		/* According to Micco spec, when OTGCP_IOVER happen,
		 * Need clean the USBPCP_EN in MISC. and then set
		 * it again.
		 */
		micco_read(MICCO_MISC, &val);
		val &= ~MICCO_MISC_USBCP_EN;
		micco_write(MICCO_MISC, val);
		val |= MICCO_MISC_USBCP_EN;
		micco_write(MICCO_MISC, val);
	}
}

/*
 * Micco interrupt service routine.
 * In the ISR we need to check the Status bits in Micco and according
 * to those bits to check which kind of IRQ had happened.
 */
static irqreturn_t micco_irq_handler(int irq, void *dev_id)
{
	struct i2c_client *client = dev_id;
	struct micco_platform_data *pdata = client->dev.platform_data;

	/* clear the irq */
	pdata->ack_irq();
	/* pxa910: clear the level triggered interrupt in the handler */
	if(cpu_is_pxa910())
		event = micco_event_change();

	schedule_work(&pdata->work);

	return IRQ_HANDLED;
}

#ifdef	CONFIG_PM
/*
 * Suspend the micco interface.
 * Add register save/restore here.
 */
static int micco_suspend(struct i2c_client *client, pm_message_t state)
{
	unsigned char val;

	disable_irq(client->irq);
	if ((get_pm_state() == PM_SUSPEND_MEM)) {
		micco_read(MICCO_OVER1, &val);
		micco_regs[MICCO_OVER1].data = val;
		micco_read(MICCO_APP_OVER2, &val);
		micco_regs[MICCO_APP_OVER2].data = val;
		micco_read(MICCO_APP_OVER3, &val);
		micco_regs[MICCO_APP_OVER3].data = val;
		micco_read(MICCO_BUCK_SLEEP, &val);
		micco_regs[MICCO_BUCK_SLEEP].data = val;
		micco_read(MICCO_SYSCTRL_A, &val);
		micco_regs[MICCO_SYSCTRL_A].data = val;

		micco_write(MICCO_BUCK_SLEEP, 0x6D);
		micco_write(MICCO_SYSCTRL_A, 0xff);

/*		micco_write(MICCO_OVER1, 0x04);
		micco_write(MICCO_APP_OVER2, 0x30);
		micco_write(MICCO_APP_OVER3, 0x7c);
*/
	}
	return 0;
}

/*
 * Resume the Micco interface.
 */
static int micco_resume(struct i2c_client *client)
{
	int i;

	for (i = 0; i < MICCO_REG_NUM; i++)
		micco_regs[i].hit = 0;

	enable_irq(client->irq);
	micco_irq_handler(client->irq, client);
	if ((get_pm_state() == PM_SUSPEND_MEM)) {
		micco_write(MICCO_OVER1, micco_regs[MICCO_OVER1].data);
		micco_write(MICCO_APP_OVER2, micco_regs[MICCO_APP_OVER2].data);
		micco_write(MICCO_APP_OVER3, micco_regs[MICCO_APP_OVER3].data);
		micco_write(MICCO_BUCK_SLEEP, micco_regs[MICCO_BUCK_SLEEP].data);
		micco_write(MICCO_SYSCTRL_A, micco_regs[MICCO_SYSCTRL_A].data);
	}
	return 0;
}

#else				/*  */
#define	micco_suspend		NULL
#define	micco_resume		NULL
#endif				/*  */

static struct pmic_ops micco_pmic_ops = {
	.get_voltage		= get_micco_voltage,
	.set_voltage		= set_micco_voltage,

	.is_vbus_assert		= is_micco_vbus_assert,
	.is_avbusvld		= is_micco_avbusvld,
	.is_asessvld		= is_micco_asessvld,
	.is_bsessvld		= is_micco_bsessvld,
	.is_srp_ready		= is_micco_srp_ready,

	.set_pump		= micco_set_pump,
	.set_vbus_supply	= micco_set_vbus_supply,
	.set_usbotg_a_mask	= micco_set_usbotg_a_mask,
	.set_usbotg_b_mask	= micco_set_usbotg_b_mask,
};

#ifdef	CONFIG_PROC_FS
#define	MICCO_PROC_FILE	"driver/micco"
static struct proc_dir_entry *micco_proc_file;
static int index;

static ssize_t micco_proc_read(struct file *filp,
		char *buffer, size_t length, loff_t *offset)
{
	u8 reg_val;

	if ((index < 0) || (index > MICCO_REG_NUM))
		return 0;

	micco_read(index, &reg_val);
	printk(KERN_INFO "register 0x%x: 0x%x\n", index, reg_val);
	return 0;
}

#define VIBRA_OFF_VALUE	0
#define VIBRA_ON_VALUE	0x30	/* you can choose the value of bit6 ~ bit1
				  to change the freq of the vibrator */
static ssize_t micco_proc_write(struct file *filp,
		const char *buff, size_t len, loff_t *off)
{
	u8 reg_val;
	char messages[256], vol[256];

	if (len > 256)
		len = 256;

	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	if (strncmp(messages, "viboff", 6) == 0)
		micco_set_vibrator(VIBRA_OFF_VALUE);
	else if (strncmp(messages, "vibon", 5) == 0)
		micco_set_vibrator(VIBRA_ON_VALUE);
	else if (strncmp(messages, "vib", 3) == 0){
		unsigned int val = 0;
		sscanf(messages, "vib%x", &val);
		micco_set_vibrator(val);
	}else if ('-' == messages[0]) {
		/* set the register index */
		memcpy(vol, messages+1, len-1);
		index = (int) simple_strtoul(vol, NULL, 16);
	} else {
		/* set the register value */
		reg_val = (int)simple_strtoul(messages, NULL, 16);
		micco_write(index, reg_val & 0xFF);
	}

	return len;
}

static struct file_operations micco_proc_ops = {
	.read = micco_proc_read,
	.write = micco_proc_write,
};

static void create_micco_proc_file(void)
{
	micco_proc_file = create_proc_entry(MICCO_PROC_FILE, 0644, NULL);
	if (micco_proc_file) {
		micco_proc_file->owner = THIS_MODULE;
		micco_proc_file->proc_fops = &micco_proc_ops;
	} else
		printk(KERN_INFO "proc file create failed!\n");
}

static void remove_micco_proc_file(void)
{
	extern struct proc_dir_entry proc_root;
	remove_proc_entry(MICCO_PROC_FILE, &proc_root);
}

#endif

static int __devinit micco_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct micco_platform_data *pdata;
	int ret;
	int chip_id;
	int i;

	chip_id = i2c_smbus_read_byte_data(client, MICCO_CHIP_ID);
	if (chip_id < 0) {
		printk(KERN_WARNING "micco unavailable!\n");
		g_client = NULL;
		return -ENXIO;
	} else {
		printk(KERN_INFO "micco(chip id:0x%02x) detected.\n", chip_id);
	}

	g_client = client;

	ret = micco_initchip();
	if (ret != 0)
		printk(KERN_WARNING "Initialize Micco failed with ret 0x%x\n",
			ret);

	pdata = client->dev.platform_data;
	/* init spinlock */
	spin_lock_init(&pdata->lock);
	/* init workqueue */
	INIT_WORK(&pdata->work, micco_worker);
	/* init irq */
	pdata->init_irq();
	/* FIXME: define trigger type in platform data later */
	if (cpu_is_pxa910()) {
		ret = request_irq(client->irq, micco_irq_handler, IRQF_DISABLED,
			"Micco", client);
	} else {
		ret = request_irq(client->irq, micco_irq_handler, IRQF_TRIGGER_FALLING,
			"Micco", client);
	}
	if (ret) {
		printk(KERN_WARNING "Request IRQ for Micco failed, return:%d\n",
			ret);
		return ret;
	}

	pdata->platform_init();

	for (i = 0; pdata->power_chips[i].power_supply_modules != NULL; i++) {
		if (pdata->power_chips[i].chip_id == chip_id) {
			micco_power_module = \
				pdata->power_chips[i].power_supply_modules;
			break;
		}
	}
	if (pdata->power_chips[i].power_supply_modules == NULL)
		panic("This Micco chip is not supported");

#ifdef	CONFIG_PROC_FS
	create_micco_proc_file();
#endif

	pmic_set_ops(&micco_pmic_ops);

#ifdef CONFIG_TOUCHSCREEN_MICCO
	{
	struct platform_device *pdev;
	pdev = platform_device_alloc("micco-touch", (int)id->driver_data);
//	pdev->dev.platform_data = subdev->platform_data;
	platform_device_add(pdev);
	}
#endif

	return 0;
}

static int micco_remove(struct i2c_client *client)
{
	pmic_set_ops(NULL);

#ifdef	CONFIG_PROC_FS
	remove_micco_proc_file();
#endif

	free_irq(client->irq, NULL);

	return 0;
}

static const struct i2c_device_id micco_id[] = {
	{ "micco", 0 },
	{ }
};

static struct i2c_driver micco_driver = {
	.driver = {
		.name	= "micco",
	},
	.id_table 	= micco_id,
	.probe		= micco_probe,
	.remove		= micco_remove,
	.suspend	= micco_suspend,
	.resume		= micco_resume,
};

static int __init micco_init(void)
{
	return i2c_add_driver(&micco_driver);
}

static void __exit micco_exit(void)
{
	flush_scheduled_work();
	i2c_del_driver(&micco_driver);
}

subsys_initcall(micco_init);
module_exit(micco_exit);

MODULE_DESCRIPTION("Micco Driver");
MODULE_LICENSE("GPL");

