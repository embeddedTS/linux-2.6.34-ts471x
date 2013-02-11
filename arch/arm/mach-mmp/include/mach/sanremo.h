/*
 * Copyright (C) 2008, Marvell Corporation.
 * Author: Bin Yang <bin.yang@marvell.com> 
 * 			 Yael Sheli Chemla<yael.s.shemla@marvell.com> 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __MACH_SANREMO_H
#define __MACH_SANREMO_H
#include <plat/pxa3xx_pmic.h>

#define SANREMO_ADDRESS		0x30 /*88PM8607,  (codenamed 'SanRemo')*/

#define SANREMO_ID		0x00
#define SANREMO_PDOWN_STATUS	0x01
#define SANREMO_PDOWN_STATUS_B0	0x0e
#define SANREMO_OVER_TEMP_EVENT		(1 << 0)
#define SANREMO_SW_PWDOWN_EVENT		(1 << 2)
#define SANREMO_SYSEN_EVENT		(1 << 3)
#define SANREMO_WD_EVENT		(1 << 4)
#define SANREMO_LONG_ONKEY_EVENT	(1 << 5)
#define SANREMO_PWR_HOLD_EVENT		(1 << 6)
#define SANREMO_RTC_UNDER_VOLTAGE	(1 << 7)

#define SANREMO_STATUS	 	0x02
#define SANREMO_STATUS_B0	0x01
#define SANREMO_ONKEY_STATUS		(1 << 0)
#define SANREMO_EXTON_STATUS		(1 << 1) 
#define SANREMO_CHG_STATUS		(1 << 2)
#define SANREMO_BAT_STATUS		(1 << 3)
#define SANREMO_PEN_STATUS		(1 << 4)
#define SANREMO_HEADSET_STATUS		(1 << 5)
#define SANREMO_HOOK_STATUS		(1 << 6)
#define SANREMO_MICIN_STATUS		(1 << 7)

#define SANREMO_INTERRUPT_STATUS1	0x03
#define SANREMO_ONKEY_INT		(1 << 0)
#define SANREMO_EXTON_INT		(1 << 1)
#define SANREMO_CHG_INT			(1 << 2)
#define SANREMO_BAT_INT			(1 << 3)
#define SANREMO_RTC_INT			(1 << 4)

#define SANREMO_INTERRUPT_STATUS2	0x04
#define SANREMO_VBAT_INT		(1 << 0)
#define SANREMO_VCHG_INT		(1 << 1)
#define SANREMO_VSYS_INT		(1 << 2)
#define SANREMO_TINT_INT		(1 << 3)
#define SANREMO_GPADC0_INT		(1 << 4)
#define SANREMO_GPADC1_INT		(1 << 5)
#define SANREMO_GPADC2_INT		(1 << 6)
#define SANREMO_GPADC3_INT		(1 << 7)

#define SANREMO_INTERRUPT_STATUS3	0x05
#define SANREMO_PEN_INT			(1 << 1)
#define SANREMO_HEADSET_INT		(1 << 2)
#define SANREMO_HOOK_INT		(1 << 3)
#define SANREMO_MICIN_INT		(1 << 4)
#define SANREMO_CHG_FAIL_INT		(1 << 5)
#define SANREMO_CHG_DONE_INT		(1 << 6)
#define SANREMO_CHG_IOVER_INT		(1 << 7)

#define SANREMO_INTERRUPT_ENABLE1		0x06
#define SANREMO_ONKEY_MASK		(1 << 0)
#define SANREMO_EXTON_MASK		(1 << 1)
#define SANREMO_CHG_MASK		(1 << 2)
#define SANREMO_BAT_MASK		(1 << 3)
#define SANREMO_RTC_MASK		(1 << 4)

#define SANREMO_INTERRUPT_ENABLE2		0x07
#define SANREMO_VBAT_MASK		(1 << 0)
#define SANREMO_VCHG_MASK		(1 << 1)
#define SANREMO_VSYS_MASK		(1 << 2)
#define SANREMO_TINT_MASK		(1 << 3)
#define SANREMO_GPADC0_MASK		(1 << 4)
#define SANREMO_GPADC1_MASK		(1 << 5)
#define SANREMO_GPADC2_MASK		(1 << 6)
#define SANREMO_GPADC3_MASK		(1 << 7)

#define SANREMO_INTERRUPT_ENABLE3		0x08
#define SANREMO_PEN_MASK		(1 << 1)
#define SANREMO_HEADSET_MASK		(1 << 2)
#define SANREMO_HOOK_MASK		(1 << 3)
#define SANREMO_MICIN_MASK		(1 << 4)
#define SANREMO_CHG_FAIL_MASK		(1 << 5)
#define SANREMO_CHG_DONE_MASK		(1 << 6)
#define SANREMO_CHG_IOVER_MASK		(1 << 7)

#define SANREMO_RESET_OUT	0x09
#define SANREMO_DEBOUNCE		0x0a
#define SANREMO_WAKEUP	0x0b
#define SANREMO_LDO1	0x10
#define SANREMO_LDO2	0x11
#define SANREMO_LDO3	0x12
#define SANREMO_LDO4	0x13
#define SANREMO_LDO5	0x14
#define SANREMO_LDO6	0x15
#define SANREMO_LDO7	0x16
#define SANREMO_LDO8	0x17
#define SANREMO_LDO9	0x18
#define SANREMO_LDO10	0x19
#define SANREMO_LDO12	0x1a
#define SANREMO_LDO14	0x1b
#define SANREMO_SLEEP_MODE1	0x1c
#define SANREMO_SLEEP_MODE2	0x1d
#define SANREMO_SLEEP_MODE3	0x1e
#define SANREMO_SLEEP_MODE4	0x1f
#define SANREMO_GO	0x20
#define SANREMO_VBUCK1_SET_SLP	0x21
#define SANREMO_VBUCK2_SET_SLP	0x22
#define SANREMO_VBUCK3_SET_SLP	0x23
#define SANREMO_VBUCK1_SET	0x24
#define SANREMO_VBUCK2_SET	0x25
#define SANREMO_VBUCK3_SET	0x26
#define SANREMO_BUCK_CONTROLS	0x27
#define SANREMO_VIBRA_SET	0x28
#define SANREMO_VIBRA_PWM	0x29
#define SANREMO_REF_GROUP	0x2a
#define SANREMO_SUPPLIES_EN11	0x2b
#define SANREMO_SUPPLIES_EN12	0x2c
#define SANREMO_GROUP1	0x2d
#define SANREMO_GROUP2	0x2e
#define SANREMO_GROUP3	0x2f
#define SANREMO_GROUP4	0x30
#define SANREMO_GROUP5	0x31
#define SANREMO_GROUP6	0x32
#define SANREMO_SUPPLIES_EN21	0x33
#define SANREMO_SUPPLIES_EN22	0x34

#define SANREMO_AUDIO_MIC_BUTTON_DETECTION_B0		0x37
#define SANREMO_AUDIO_HEADSET_DETECTION_BO		0x38
#define SANREMO_MISC1	0x40
#define SANREMO_MISC1_GPIO1_DIR   (1 << 3)
#define SANREMO_MISC1_GPIO1_VAL   (1 << 4)
#define SANREMO_MISC1_GPIO2_DIR   (1 << 5)
#define SANREMO_MISC1_GPIO2_VAL   (1 << 6)

#define SANREMO_MCLK	0x41
#define SANREMO_MISC2	0x42
#define SANREMO_PLL_CTRL1	0x43
#define SANREMO_PLL_FRAC1	0x44
#define SANREMO_PLL_FRAC2	0x45
#define SANREMO_PLL_FRAC3	0x46
#define SANREMO_PLL_CTRL2	0x47
#define SANREMO_CHG_CTRL1	0x48
#define SANREMO_CHG_CTRL2	0x49
#define SANREMO_CHG_CTRL3	0x4a
#define SANREMO_CHG_CTRL4	0x4b
#define SANREMO_CHG_CTRL5	0x4c
#define SANREMO_CHG_CTRL6	0x4d
#define SANREMO_CHG_CTRL7	0x4e
#define SANREMO_GP_BIAS1	0x4f

#define SANREMO_MEAS_ENABLE1	0x50
#define SANREMO_MEAS_EN1_VBAT           (1 << 0)
#define SANREMO_MEAS_EN1_VCHG           (1 << 1)
#define SANREMO_MEAS_EN1_VSYS           (1 << 2)
#define SANREMO_MEAS_EN1_TINT           (1 << 3)
#define SANREMO_MEAS_EN1_RFTMP          (1 << 4)
#define SANREMO_MEAS_EN1_TBAT           (1 << 5)
#define SANREMO_MEAS_EN1_GPADC2        	(1 << 6)
#define SANREMO_MEAS_EN1_GPADC3         (1 << 7)

#define SANREMO_MEAS_ENABLE2	0x51

#define SANREMO_MEAS_ENABLE3	0x52
#define SANREMO_MEAS_EN3_IBAT           (1 << 0)
#define SANREMO_MEAS_EN3_IBAT_COMP 	    (1 << 1)
#define SANREMO_MEASOFFTIME1_BAT_DET_EN_B0	(1<<1)
#define SANREMO_MEAS_EN3_COULOMB_COUNTER (1 << 2)
#define SANREMO_MEAS_EN3_PENDET         (1 << 3)
#define SANREMO_MEAS_EN3_TSIX			(1 << 4)
#define SANREMO_MEAS_EN3_TSIY           (1 << 5)
#define SANREMO_MEAS_EN3_TSIZ1        	(1 << 6)
#define SANREMO_MEAS_EN3_TSIZ2         	(1 << 7)

#define SANREMO_MEAS_OFF_TIME1	0x53
#define SANREMO_MEASOFFTIME1_BAT_DET_EN 		(1 << 0)
#define SANREMO_MEASOFFTIME1_MEAS_OFF_TIME1  	(1 << 1)
#define SANREMO_MEASOFFTIME1_MEAS_OFF_TIME1_MASK  	(0x3F << 1)
#define SANREMO_MEASOFFTIME1_MEAS_OFF_TIME1_BIT  	(1)

#define SANREMO_MEAS_OFF_TIME2	0x54
#define SANREMO_TSI_PREBIAS_TIME	0x55
#define SANREMO_PD_PREBIAS_TIME	0x56

#define SANREMO_GPADC_MISC1	0x57
#define SANREMO_GPADC_MISC1_GPFSM_EN    (1 << 0)
#define SANREMO_GPPADC_GP_PREBIAS_TIME  (01 << 1)
#define SANREMO_GPADC_MISC1_MASK  (SANREMO_GPADC_MISC1_GPFSM_EN | SANREMO_GPPADC_GP_PREBIAS_TIME)

#define SANREMO_SW_CAL	0x58
#define SANREMO_GPADC_MISC2	0x59
#define SANREMO_GP_BIAS2	0x5a
#define SANREMO_GP_BIAS3		//TBD
#define SANREMO_VBAT_LOW_TH	0x5b
#define SANREMO_VCHG_LOW_TH	0x5c
#define SANREMO_VSYS_LOW_TH	0x5d
#define SANREMO_TINT_LOW_TH	0x5e
#define SANREMO_TBAT_LOW_TH	0x5f
#define SANREMO_GPADC1_LOW_TH	0x60
#define SANREMO_GPADC2_LOW_TH	0x61
#define SANREMO_GPADC3_LOW_TH	0x62
#define SANREMO_VBAT_UPP_TH	0x63
#define SANREMO_VCHG_UPP_TH	0x64
#define SANREMO_VSYS_UPP_TH	0x65
#define SANREMO_TINT_UPP_TH	0x66
#define SANREMO_TBAT_UPP_TH	0x67
#define SANREMO_GPADC1_UPP_TH	0x68
#define SANREMO_GPADC2_UPP_TH	0x69
#define SANREMO_GPADC3_UPP_TH	0x6a
#define SANREMO_IBAT_MEAS1	0x6b
#define SANREMO_IBAT_MEAS2	0x6c
#define SANREMO_VBAT_MEAS1	0x6d
#define SANREMO_VBAT_MEAS2	0x6e
#define SANREMO_VCHG_MEAS1	0x6f
#define SANREMO_VCHG_MEAS2	0x70
#define SANREMO_VSYS_MEAS1	0x71
#define SANREMO_VSYS_MEAS2	0x72
#define SANREMO_TINT_MEAS1	0x73
#define SANREMO_TINT_MEAS2	0x74
#define SANREMO_GPADC0_MEAS1	0x75
#define SANREMO_GPADC0_MEAS2	0x76
#define SANREMO_TBAT_MEAS1	0x77
#define SANREMO_TBAT_MEAS2	0x78
#define SANREMO_GPADC2_MEAS1	0x79
#define SANREMO_GPADC2_MEAS2	0x7a
#define SANREMO_GPADC3_MEAS1	0x7b
#define SANREMO_GPADC3_MEAS2	0x7c
#define SANREMO_TSIX_MEAS1	0x8d
#define SANREMO_TSIX_MEAS2	0x8e
#define SANREMO_TSIY_MEAS1	0x8f
#define SANREMO_TSIY_MEAS2	0x90
#define SANREMO_TSIZ1_MEAS1	0x91
#define SANREMO_TSIZ1_MEAS2	0x92
#define SANREMO_TSIZ2_MEAS1	0x93
#define SANREMO_TSIZ2_MEAS2	0x94
#define SANREMO_CCNT1	0x95
#define SANREMO_CCNT2	0x96
#define SANREMO_VBAT_AVG	0x97
#define SANREMO_VCHG_AVG	0x98
#define SANREMO_VSYS_AVG	0x99
#define SANREMO_VBAT_MIN	0x9a
#define SANREMO_VCHG_MIN	0x9b
#define SANREMO_VSYS_MIN	0x9c
#define SANREMO_VBAT_MAX	0x9d
#define SANREMO_VCHG_MAX	0x9e
#define SANREMO_VSYS_MAX	0x9f
#define SANREMO_RTC1	0xa0
#define SANREMO_RTC_COUNTER1	0xa1
#define SANREMO_RTC_COUNTER2	0xa2
#define SANREMO_RTC_COUNTER3	0xa3
#define SANREMO_RTC_COUNTER4	0xa4
#define SANREMO_RTC_EXPIRE1	0xa5
#define SANREMO_RTC_EXPIRE2	0xa6
#define SANREMO_RTC_EXPIRE3	0xa7
#define SANREMO_RTC_EXPIRE4	0xa8
#define SANREMO_RTC_TRIM_INT1	0xa9
#define SANREMO_RTC_TRIM_INT2	0xaa
#define SANREMO_RTC_TRIM_FRAC1	0xab
#define SANREMO_RTC_TRIM_FRAC2	0xac
#define SANREMO_RTC_MISC1	0xad
#define SANREMO_RTC_MISC2	0xae
#define SANREMO_RTC_MISC3	0xaf

/*Audio*/
#define SANREMO_AUDIO_REG_BASE 0xb0
#define SANREMO_AUDIO_REG_LEN  0x3b
#define SANREMO_AUDIO_PCM_INTERFACE_1	0x00
#define SANREMO_AUDIO_PCM_INTERFACE_2		0x01
#define SANREMO_AUDIO_PCM_INTERFACE_3		0x02
#define SANREMO_AUDIO_ADC_PCM		0x03
#define SANREMO_AUDIO_ADC_1		0x04
#define SANREMO_AUDIO_ADC_2		0x05
#define SANREMO_AUDIO_ADC_3		0x06
#define SANREMO_AUDIO_ADC_4		0x07
#define SANREMO_AUDIO_ADC_5		0x08
#define SANREMO_AUDIO_ADC_6		0x09
#define SANREMO_AUDIO_ADC_7		0x0a
#define SANREMO_AUDIO_I2S_INTERFACE_1		0x0b
#define SANREMO_AUDIO_I2S_INTERFACE_2		0x0c
#define SANREMO_AUDIO_I2S_INTERFACE_3		0x0d
#define SANREMO_AUDIO_Equalizer_N0_1		0x0e
#define SANREMO_AUDIO_Equalizer_N0_2		0x0f
#define SANREMO_AUDIO_Equalizer_N1_1		0x11
#define SANREMO_AUDIO_Equalizer_N1_2		0x12
#define SANREMO_AUDIO_Equalizer_D1_1		0x13
#define SANREMO_AUDIO_Equalizer_D1_2		0x14
#define SANREMO_AUDIO_Side_Tone_1		0x15
#define SANREMO_AUDIO_Side_Tone_2		0x16
#define SANREMO_AUDIO_Left_Gain1		0x17
#define SANREMO_AUDIO_Left_Gain2		0x18
#define SANREMO_AUDIO_Right_Gain1		0x19
#define SANREMO_AUDIO_Right_Gain2		0x1a
#define SANREMO_AUDIO_DWA_OFFSET		0x1b
#define SANREMO_AUDIO_OFFSET_LEFT1		0x1c
#define SANREMO_AUDIO_OFFSET_LEFT2		0x1d
#define SANREMO_AUDIO_OFFSET_RIGHT1		0x1e
#define SANREMO_AUDIO_OFFSET_RIGHT2		0x1f
#define SANREMO_AUDIO_ADC_ANALOG_PROGRAM1		0x20
#define SANREMO_AUDIO_ADC_ANALOG_PROGRAM2		0x21
#define SANREMO_AUDIO_ADC_ANALOG_PROGRAM3		0x22
#define SANREMO_AUDIO_ADC_ANALOG_PROGRAM4		0x23
#define SANREMO_AUDIO_A2A_PATH_PROGRAMMING		0x24
#define SANREMO_AUDIO_DAC_HS1_CTRL		0x25
#define SANREMO_AUDIO_DAC_HS2_CTRL		0x26
#define SANREMO_AUDIO_DAC_LO1_CTRL		0x27
#define SANREMO_AUDIO_DAC_LO2_CTRL		0x28
#define SANREMO_AUDIO_DAC_EAR_SPKRPHNE_GAIN		0x29
#define SANREMO_AUDIO_MISC_AUDIO		0x2a
#define SANREMO_AUDIO_AUDIO_SUPPLIES1		0x2b
#define SANREMO_AUDIO_AUDIO_SUPPLIES2		0x2c
#define SANREMO_AUDIO_ADC_ANALOG_ENABLES		0x2d
#define SANREMO_AUDIO_ADC_DIGITAL_ENABLES		0x2e
#define SANREMO_AUDIO_DAC_ANALOG_ENABLES	0x2f
#define SANREMO_AUDIO_DAC_DIGITAL_ENABLES		0x31
#define SANREMO_AUDIO_AUDIO_CAL1		0x32
#define SANREMO_AUDIO_AUDIO_CAL2		0x33
#define SANREMO_AUDIO_AUDIO_CAL3		0x34
#define SANREMO_AUDIO_AUDIO_CAL4		0x35
#define SANREMO_AUDIO_AUDIO_CAL5		0x36
#define SANREMO_AUDIO_ANALOG_INPUT_SEL1		0x37
#define SANREMO_AUDIO_ANALOG_INPUT_SEL2		0x38
#define SANREMO_AUDIO_MIC_BUTTON_DETECTION		0x39
#define SANREMO_AUDIO_HEADSET_DETECTION		0x3a
#define SANREMO_AUDIO_SHORTS		0x3b

#define SANREMO_VBUCK1_CNT(x)		((x < 0) ? -1 :			\
					((x < 800) ? (x / 25 + 0x20) :	\
					((x < 1525) ? ((x - 800) / 25)	\
					: -1)))
#define SANREMO_VBUCK1_MV(x)		((x < 0) ? -1 :			\
					((x < 0x1C) ? (x * 25 + 800) :	\
					((x < 0x20) ? 1500 :		\
					((x < 0x40) ? ((x - 0x20) * 25)	\
					: -1))))

#define SANREMO_VBUCK2_CNT(x)		((x < 0) ? -1 :			\
					((x < 3050) ? (x / 50) : -1))

#define SANREMO_VBUCK2_MV(x)		((x < 0) ? -1 :			\
					((x < 0x3C) ? (x * 50) :	\
					((x < 0x40) ? 3000 : -1)))

#define SANREMO_VBUCK3_CNT(x)		((x < 0) ? -1 :			\
					((x < 1525) ? (x / 25) : -1))

#define SANREMO_VBUCK3_MV(x)		((x < 0) ? -1 :			\
					((x < 0x3C) ? (x * 25) :	\
					((x < 0x40) ? 1500 : -1)))

#define SANREMO_LDO1_CNT(x)		((x == 1800) ? 0 :		\
					((x == 1200) ? 1 :		\
					((x == 2800) ? 2 : -1)))

#define SANREMO_LDO1_MV(x)		((x == 0) ? 1800 :		\
					((x == 1) ? 1200 :		\
					((x == 2) ? 2800 : -1)))

#define SANREMO_LDO1_SLEEP_CNT(x)	((x == 1800) ? 0 :		\
					((x == 1200) ? 1 : -1))

#define SANREMO_LDO1_SLEEP_MV(x)	((x == 0) ? 1800 :		\
					((x == 1) ? 1200 : -1))

#define SANREMO_LDO2_CNT(x)		((x < 1800) ? -1 :		\
					((x < 1950) ? ((x - 1800) / 50)	\
					: ((x < 2700) ? -1 :		\
					((x < 2950) ? ((x - 2550) / 50)	\
					: -1))))

#define SANREMO_LDO2_MV(x)		((x < 0) ? -1 :			\
					((x < 3) ? (x * 50 + 1800) :	\
					((x < 8) ? (x * 50 + 2550) : -1)))

#define SANREMO_LDO5_CNT(x)		((x < 2900) ? -1 :		\
					((x < 3200) ? ((x - 2900) / 100)\
					: ((x == 3300) ? 3 : -1)))

#define SANREMO_LDO5_MV(x)		((x < 0) ? -1 :			\
					((x < 3) ? (x * 100 + 2900) :	\
					((x == 3) ? 3300 : -1)))

#define SANREMO_LDO5_SLEEP_CNT(x)	((x == 2900) ? 0 : -1)

#define SANREMO_LDO5_SLEEP_MV(x)	((x == 0) ? 2900 : -1)

#define SANREMO_LDO6_CNT(x)		((x < 1800) ? -1 :		\
					((x < 1950) ? ((x - 1800) / 50)	\
					: ((x < 2600) ? -1 :		\
					((x < 2850) ? ((x - 2450) / 50)	\
					: -1))))

#define SANREMO_LDO6_MV(x)		((x < 0) ? -1 :			\
					((x < 3) ? (x * 50 + 1800) :	\
					((x < 8) ? (x * 50 + 2450) : -1)))

#define SANREMO_LDO10_CNT(x)		((x == 1200) ? 8 :		\
					((x < 1800) ? -1 :		\
					((x < 1950) ? ((x - 1800) / 50)	\
					: ((x < 2700) ? -1		\
					: ((x < 2950) ? ((x - 2550) / 50)\
					: -1)))))

#define SANREMO_LDO10_MV(x)		((x < 0) ? -1 :			\
					((x < 3) ? (x * 50 + 1800) :	\
					((x < 8) ? (x * 50 + 2550) : 1200)))

#define SANREMO_LDO12_CNT(x)		((x == 1200) ? 8 :		\
					((x < 1800) ? -1 :		\
					((x < 2000) ? ((x - 1800) / 100) :\
					((x < 2700) ? -1 :		\
					((x < 3200) ? ((x - 2500) / 100) :\
					((x == 3300) ? 7 :		\
					-1))))))

#define SANREMO_LDO12_MV(x)		((x < 0) ? -1 :			\
					((x < 2) ? (x * 100 + 1800) :	\
					((x < 7) ? (x * 100 + 2500) :	\
					((x == 7) ? 3300 : 1200))))

enum {
	SAN_BUCK1 = 1,
	SAN_BUCK2,
	SAN_BUCK3,
	SAN_LDO1,
	SAN_LDO2,
	SAN_LDO3,
	SAN_LDO4,
	SAN_LDO5,
	SAN_LDO6,
	SAN_LDO7,
	SAN_LDO8,
	SAN_LDO9,
	SAN_LDO10,
	SAN_LDO12,
	SAN_LDO14,
	SAN_BUCK1_SLEEP,
	SAN_BUCK2_SLEEP,
	SAN_BUCK3_SLEEP,
	SAN_LDO1_SLEEP,
	SAN_LDO2_SLEEP,
	SAN_LDO3_SLEEP,
	SAN_LDO4_SLEEP,
	SAN_LDO5_SLEEP,
	SAN_LDO6_SLEEP,
	SAN_LDO7_SLEEP,
	SAN_LDO8_SLEEP,
	SAN_LDO9_SLEEP,
	SAN_LDO10_SLEEP,
	SAN_LDO12_SLEEP,
	SAN_LDO14_SLEEP,
	SAN_LDO_MAX,
};

#define SANREMO_REG(x)			((x < SAN_BUCK1) ? -1 :		\
					((x < SAN_LDO1) ? ((x - SAN_BUCK1) \
					+ 0x24) : ((x < SAN_BUCK1_SLEEP) ? \
					(x -SAN_LDO1 +0x10) :	\
					((x < SAN_LDO1_SLEEP) \
					? (x - SAN_BUCK1_SLEEP + 0x21) :\
					((x <= SAN_LDO14_SLEEP) ?	\
					(x - SAN_LDO1_SLEEP + 0x10) : -1)))))

struct sanremo_touch_platform_data {
	unsigned int		ts[4];
	int			ts_revert[2];
};

struct sanremo_platform_data {
	int		(*init_irq)(void);
	int		(*ack_irq)(void);
	void		(*platform_init)(void);
	spinlock_t	lock;
	struct work_struct	work;
	struct power_chip	*power_chips;
	struct sanremo_touch_platform_data *tsi;
};

/* General */
int sanremo_read(u8 reg, u8 *val);
int sanremo_write(u8 reg, u8 val);

/* TSI */
int sanremo_enable_pen_down_irq(int enable);
int sanremo_tsi_poweron(void);
int sanremo_tsi_poweroff(void);
int sanremo_tsi_readxy(u16 *x, u16 *y, int *pen_state);
int sanremo_tsi_enable_pen(int pen_en);
int sanremo_tsi_enable_tsi(int tsi_en);
int sanremo_enable_mic_irq(int enable);
int sanremo_enable_headset_irq(int enable);
int sanremo_tsi_enable_mic(int mic_en);
int sanremo_tsi_enable_headset(int headset_en);

int sanremo_get_chip_id(void);
int sanremo_enable_headset_detect(void (*func)(unsigned long ), int enable);
int sanremo_get_headset_state(void);
/* For Audio */
int sanremo_codec_read(u8 reg, u8 *val);
int sanremo_codec_write(u8 reg, u8 val);

int sanremo_set_vibrator(unsigned char value);

void sanremo_turn_off_power(void);
int sanremo_get_vbat(void);
int sanremo_usb_connect(void);

#endif

