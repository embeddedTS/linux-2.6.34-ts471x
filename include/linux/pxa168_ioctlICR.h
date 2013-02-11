/*
 *  linux/drivers/icr/pxa168_ioctlICR.h
 *
 *  Copyright:	(C) Copyright 2009 Marvell International Ltd.
 *
 *  Author: Alan Guenther <>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 *
 */

#ifndef __PXA168_IOCTLICR_H
#define __PXA168_IOCTLICR_H

/* Structures */
typedef enum tagIcrHwFormat {
        ICR_HWFORMAT_RGB888,
        ICR_HWFORMAT_XRGB8888,
} ICR_FORMAT;

typedef struct ICRbufferDesc {
        void* addr;               //the virtual address of the source buffer
        unsigned short width;     //horizontal pixel number of source buffer.
        unsigned short height;    //vertical line number of source buffer
        unsigned int format;      //the format of source buffer, could be ICR_PACKED or //ICR_UNPACKED
} Buffer_Desc;

typedef struct
{
        Buffer_Desc buf;
} ICR_SRC_BUFFER;

typedef struct {
        Buffer_Desc buf;
        unsigned short video_start_x;     //The position in the destination buffer where
        unsigned short video_start_y;     //the ICR will write output data
        unsigned short video_width;       //horizontal pixel number of video rect
        unsigned short video_height;      // vertical line number of video rect
} ICR_DST_BUFFER;

#define ICRioctlGetBase 0x10
#define ICRioctlSetBase 0x40
#define ICR_GET_CODE 0x82
#define ICR_SET_CODE 0x82
#define ICRtransform     0
#define ICR_IOCTL_transform       _IOWR(ICR_GET_CODE, ICRtransform, int)
#define ICRwait          1
#define ICR_IOCTL_wait            _IOWR(ICR_GET_CODE, ICRwait, int)
#define ICRtransformDBwait     2
#define ICR_IOCTL_transformDBwait _IOWR(ICR_GET_CODE, ICRtransformDBwait, int)
#define ICR_IOCTL_tagged_get _IOR(ICR_GET_CODE, 0xf, int)
#define ICR_IOCTL_tagged_set _IOW(ICR_GET_CODE, 0xe, int)

#define GET_ICR_REG_FIRST (ICRioctlGetBase + 0)
#define ICR_IOCTL_GET_SRAM_WTC_RTC_REG _IOW(ICR_GET_CODE, (ICRioctlGetBase + 0), int)
#define ICR_IOCTL_GET_DBL_BUF_TRG_REG _IOW(ICR_GET_CODE, (ICRioctlGetBase + 1), int)
#define ICR_IOCTL_GET_DMA_GCR_REG _IOR(ICR_GET_CODE, (ICRioctlGetBase + 2), int)
#define ICR_IOCTL_GET_SVF0_SIZ_REG _IOW(ICR_GET_CODE, (ICRioctlGetBase + 3), int)
#define ICR_IOCTL_GET_SVF1_SIZ_REG _IOW(ICR_GET_CODE, (ICRioctlGetBase + 4), int)
#define ICR_IOCTL_GET_DF0DISP_SIZE_REG _IOW(ICR_GET_CODE, (ICRioctlGetBase + 5), int)
#define ICR_IOCTL_GET_DF1DISP_SIZE_REG _IOW(ICR_GET_CODE, (ICRioctlGetBase + 6), int)
#define ICR_IOCTL_GET_DF0DISP_SPSCR_REG _IOW(ICR_GET_CODE, (ICRioctlGetBase + 7), int)
#define ICR_IOCTL_GET_DF1DISP_SPSCR_REG _IOW(ICR_GET_CODE, (ICRioctlGetBase + 8), int)
#define ICR_IOCTL_GET_DF0_VID_SIZ_REG _IOW(ICR_GET_CODE, (ICRioctlGetBase + 9), int)
#define ICR_IOCTL_GET_DF1_VID_SIZ_REG _IOW(ICR_GET_CODE, (ICRioctlGetBase + 10), int)
#define ICR_IOCTL_GET_SVF0RGB_STRT_ADDR_REG _IOR(ICR_GET_CODE, (ICRioctlGetBase + 11), int)
#define ICR_IOCTL_GET_SVF1RGB_STRT_ADDR_REG _IOR(ICR_GET_CODE, (ICRioctlGetBase + 12), int)
#define ICR_IOCTL_GET_DF0DISP_STRT_ADDR_REG _IOR(ICR_GET_CODE, (ICRioctlGetBase + 13), int)
#define ICR_IOCTL_GET_DF1DISP_STRT_ADDR_REG _IOR(ICR_GET_CODE, (ICRioctlGetBase + 14), int)
#define ICR_IOCTL_GET_RX_DMA_CTRL0_REG _IOR(ICR_GET_CODE, (ICRioctlGetBase + 15), int)
#define ICR_IOCTL_GET_RX_DMA_CTRL1_REG _IOR(ICR_GET_CODE, (ICRioctlGetBase + 16), int)
#define ICR_IOCTL_GET_TX_DMA_CTRL0_REG _IOR(ICR_GET_CODE, (ICRioctlGetBase + 17), int)
#define ICR_IOCTL_GET_TX_DMA_CTRL1_REG _IOR(ICR_GET_CODE, (ICRioctlGetBase + 18), int)
#define ICR_IOCTL_GET_INTR_MASK_REG _IOR(ICR_GET_CODE, (ICRioctlGetBase + 19), int)
#define ICR_IOCTL_GET_INTR_CLR_SEL_REG _IOR(ICR_GET_CODE, (ICRioctlGetBase + 20), int)
#define ICR_IOCTL_GET_INTR_STAT_MASK_REG _IOR(ICR_GET_CODE, (ICRioctlGetBase + 21), int)
#define ICR_IOCTL_GET_INTR_STATUS_REG _IOR(ICR_GET_CODE, (ICRioctlGetBase + 22), int)
#define GET_ICR_REG_LAST (ICRioctlGetBase + 22)

#define SET_ICR_REG_FIRST (ICRioctlSetBase + 0)
#define ICR_IOCTL_SET_SRAM_WTC_RTC_REG _IOW(ICR_SET_CODE, (ICRioctlSetBase + 0), int)
#define ICR_IOCTL_SET_DBL_BUF_TRG_REG _IOW(ICR_SET_CODE, (ICRioctlSetBase + 1), int)
#define ICR_IOCTL_SET_DMA_GCR_REG _IOR(ICR_SET_CODE, (ICRioctlSetBase + 2), int)
#define SET_ICR_REG_LAST (ICRioctlSetBase + 2)


#define ICR_V1_ICSC_M_C0_L	0
#define ICR_V1_ICSC_M_C0_H	1
#define ICR_V1_ICSC_M_C1_L	2
#define ICR_V1_ICSC_M_C1_H	3
#define ICR_V1_ICSC_M_C2_L	4
#define ICR_V1_ICSC_M_C2_H	5
#define ICR_V1_ICSC_M_C3_L	6
#define ICR_V1_ICSC_M_C3_H	7
#define ICR_V1_ICSC_M_C4_L	8
#define ICR_V1_ICSC_M_C4_H	9
#define ICR_V1_ICSC_M_C5_L	10
#define ICR_V1_ICSC_M_C5_H	11
#define ICR_V1_ICSC_M_C6_L	12
#define ICR_V1_ICSC_M_C6_H	13
#define ICR_V1_ICSC_M_C7_L	14
#define ICR_V1_ICSC_M_C7_H	15
#define ICR_V1_ICSC_M_C8_L	16
#define ICR_V1_ICSC_M_C8_H	17
#define ICR_V1_ICSC_M_O1_0	18
#define ICR_V1_ICSC_M_O1_1	19
#define ICR_V1_ICSC_M_O1_2	20
#define ICR_V1_ICSC_M_O2_0	21
#define ICR_V1_ICSC_M_O2_1	22
#define ICR_V1_ICSC_M_O2_2	23
#define ICR_V1_ICSC_M_O3_0	24
#define ICR_V1_ICSC_M_O3_1	25
#define ICR_V1_ICSC_M_O3_2	26
#define ICR_V1_ICSC_P_C0_L	27
#define ICR_V1_ICSC_P_C0_H	28
#define ICR_V1_ICSC_P_C1_L	29
#define ICR_V1_ICSC_P_C1_H	30
#define ICR_V1_ICSC_P_C2_L	31
#define ICR_V1_ICSC_P_C2_H	32
#define ICR_V1_ICSC_P_C3_L	33
#define ICR_V1_ICSC_P_C3_H	34
#define ICR_V1_ICSC_P_C4_L	35
#define ICR_V1_ICSC_P_C4_H	36
#define ICR_V1_ICSC_P_C5_L	37
#define ICR_V1_ICSC_P_C5_H	38
#define ICR_V1_ICSC_P_C6_L	39
#define ICR_V1_ICSC_P_C6_H	40
#define ICR_V1_ICSC_P_C7_L	41
#define ICR_V1_ICSC_P_C7_H	42
#define ICR_V1_ICSC_P_C8_L	43
#define ICR_V1_ICSC_P_C8_H	44
#define ICR_V1_ICSC_P_O1_0	45
#define ICR_V1_ICSC_P_O1_1	46
#define ICR_V1_ICSC_P_O1_2	47
#define ICR_V1_ICSC_P_O2_0	48
#define ICR_V1_ICSC_P_O2_1	49
#define ICR_V1_ICSC_P_O2_2	50
#define ICR_V1_ICSC_P_O3_0	51
#define ICR_V1_ICSC_P_O3_1	52
#define ICR_V1_ICSC_P_O3_2	53
#define ICR_V1_FTDC_M_EN	54
#define ICR_V1_FTDC_P_EN	55
#define ICR_V1_FTDC_INLOW_L	56
#define ICR_V1_FTDC_INLOW_H	57
#define ICR_V1_FTDC_INHIGH_L	58
#define ICR_V1_FTDC_INHIGH_H	59
#define ICR_V1_FTDC_OUTLOW_L	60
#define ICR_V1_FTDC_OUTLOW_H	61
#define ICR_V1_FTDC_OUTHIGH_L	62
#define ICR_V1_FTDC_OUTHIGH_H	63
#define ICR_V1_FTDC_YLOW	64
#define ICR_V1_FTDC_YHIGH	65
#define ICR_V1_FTDC_CH1	66
#define ICR_V1_FTDC_CH2_L	67
#define ICR_V1_FTDC_CH2_H	68
#define ICR_V1_FTDC_CH3_L	69
#define ICR_V1_FTDC_CH3_H	70
#define ICR_V1_FTDC_1_C00	71
#define ICR_V1_FTDC_2_C00	72
#define ICR_V1_FTDC_3_C00	73
#define ICR_V1_FTDC_4_C00	74
#define ICR_V1_FTDC_5_C00	75
#define ICR_V1_FTDC_6_C00	76
#define ICR_V1_FTDC_1_C01	77
#define ICR_V1_FTDC_2_C01	78
#define ICR_V1_FTDC_3_C01	79
#define ICR_V1_FTDC_4_C01	80
#define ICR_V1_FTDC_5_C01	81
#define ICR_V1_FTDC_6_C01	82
#define ICR_V1_FTDC_1_C11	83
#define ICR_V1_FTDC_2_C11	84
#define ICR_V1_FTDC_3_C11	85
#define ICR_V1_FTDC_4_C11	86
#define ICR_V1_FTDC_5_C11	87
#define ICR_V1_FTDC_6_C11	88
#define ICR_V1_FTDC_1_C10	89
#define ICR_V1_FTDC_2_C10	90
#define ICR_V1_FTDC_3_C10	91
#define ICR_V1_FTDC_4_C10	92
#define ICR_V1_FTDC_5_C10	93
#define ICR_V1_FTDC_6_C10	94
#define ICR_V1_FTDC_1_OFF00	95
#define ICR_V1_FTDC_2_OFF00	96
#define ICR_V1_FTDC_3_OFF00	97
#define ICR_V1_FTDC_4_OFF00	98
#define ICR_V1_FTDC_5_OFF00	99
#define ICR_V1_FTDC_6_OFF00	100
#define ICR_V1_FTDC_1_OFF10	101
#define ICR_V1_FTDC_2_OFF10	102
#define ICR_V1_FTDC_3_OFF10	103
#define ICR_V1_FTDC_4_OFF10	104
#define ICR_V1_FTDC_5_OFF10	105
#define ICR_V1_FTDC_6_OFF10	106
#define ICR_V1_HS_M_EN	107
#define ICR_V1_HS_M_AX1_L	108
#define ICR_V1_HS_M_AX1_H	109
#define ICR_V1_HS_M_AX2_L	110
#define ICR_V1_HS_M_AX2_H	111
#define ICR_V1_HS_M_AX3_L	112
#define ICR_V1_HS_M_AX3_H	113
#define ICR_V1_HS_M_AX4_L	114
#define ICR_V1_HS_M_AX4_H	115
#define ICR_V1_HS_M_AX5_L	116
#define ICR_V1_HS_M_AX5_H	117
#define ICR_V1_HS_M_AX6_L	118
#define ICR_V1_HS_M_AX6_H	119
#define ICR_V1_HS_M_AX7_L	120
#define ICR_V1_HS_M_AX7_H	121
#define ICR_V1_HS_M_AX8_L	122
#define ICR_V1_HS_M_AX8_H	123
#define ICR_V1_HS_M_AX9_L	124
#define ICR_V1_HS_M_AX9_H	125
#define ICR_V1_HS_M_AX10_L	126
#define ICR_V1_HS_M_AX10_H	127
#define ICR_V1_HS_M_AX11_L	128
#define ICR_V1_HS_M_AX11_H	129
#define ICR_V1_HS_M_AX12_L	130
#define ICR_V1_HS_M_AX12_H	131
#define ICR_V1_HS_M_AX13_L	132
#define ICR_V1_HS_M_AX13_H	133
#define ICR_V1_HS_M_AX14_L	134
#define ICR_V1_HS_M_AX14_H	135
#define ICR_V1_HS_M_H1	136
#define ICR_V1_HS_M_H2	137
#define ICR_V1_HS_M_H3	138
#define ICR_V1_HS_M_H4	139
#define ICR_V1_HS_M_H5	140
#define ICR_V1_HS_M_H6	141
#define ICR_V1_HS_M_H7	142
#define ICR_V1_HS_M_H8	143
#define ICR_V1_HS_M_H9	144
#define ICR_V1_HS_M_H10	145
#define ICR_V1_HS_M_H11	146
#define ICR_V1_HS_M_H12	147
#define ICR_V1_HS_M_H13	148
#define ICR_V1_HS_M_H14	149
#define ICR_V1_HS_M_S1	150
#define ICR_V1_HS_M_S2	151
#define ICR_V1_HS_M_S3	152
#define ICR_V1_HS_M_S4	153
#define ICR_V1_HS_M_S5	154
#define ICR_V1_HS_M_S6	155
#define ICR_V1_HS_M_S7	156
#define ICR_V1_HS_M_S8	157
#define ICR_V1_HS_M_S9	158
#define ICR_V1_HS_M_S10	159
#define ICR_V1_HS_M_S11	160
#define ICR_V1_HS_M_S12	161
#define ICR_V1_HS_M_S13	162
#define ICR_V1_HS_M_S14	163
#define ICR_V1_HS_M_GL	164
#define ICR_V1_HS_M_MAXSAT_RGB_Y_L	165
#define ICR_V1_HS_M_MAXSAT_RGB_Y_H	166
#define ICR_V1_HS_M_MAXSAT_RCR_L	167
#define ICR_V1_HS_M_MAXSAT_RCR_H	168
#define ICR_V1_HS_M_MAXSAT_RCB_L	169
#define ICR_V1_HS_M_MAXSAT_RCB_H	170
#define ICR_V1_HS_M_MAXSAT_GCR_L	171
#define ICR_V1_HS_M_MAXSAT_GCR_H	172
#define ICR_V1_HS_M_MAXSAT_GCB_L	173
#define ICR_V1_HS_M_MAXSAT_GCB_H	174
#define ICR_V1_HS_M_MAXSAT_BCR_L	175
#define ICR_V1_HS_M_MAXSAT_BCR_H	176
#define ICR_V1_HS_M_MAXSAT_BCB_L	177
#define ICR_V1_HS_M_MAXSAT_BCB_H	178
#define ICR_V1_HS_M_ROFF_L	179
#define ICR_V1_HS_M_ROFF_H	180
#define ICR_V1_HS_M_GOFF_L	181
#define ICR_V1_HS_M_GOFF_H	182
#define ICR_V1_HS_M_BOFF_L	183
#define ICR_V1_HS_M_BOFF_H	184
#define ICR_V1_HS_P_EN	185
#define ICR_V1_HS_P_AX1_L	186
#define ICR_V1_HS_P_AX1_H	187
#define ICR_V1_HS_P_AX2_L	188
#define ICR_V1_HS_P_AX2_H	189
#define ICR_V1_HS_P_AX3_L	190
#define ICR_V1_HS_P_AX3_H	191
#define ICR_V1_HS_P_AX4_L	192
#define ICR_V1_HS_P_AX4_H	193
#define ICR_V1_HS_P_AX5_L	194
#define ICR_V1_HS_P_AX5_H	195
#define ICR_V1_HS_P_AX6_L	196
#define ICR_V1_HS_P_AX6_H	197
#define ICR_V1_HS_P_AX7_L	198
#define ICR_V1_HS_P_AX7_H	199
#define ICR_V1_HS_P_AX8_L	200
#define ICR_V1_HS_P_AX8_H	201
#define ICR_V1_HS_P_AX9_L	202
#define ICR_V1_HS_P_AX9_H	203
#define ICR_V1_HS_P_AX10_L	204
#define ICR_V1_HS_P_AX10_H	205
#define ICR_V1_HS_P_AX11_L	206
#define ICR_V1_HS_P_AX11_H	207
#define ICR_V1_HS_P_AX12_L	208
#define ICR_V1_HS_P_AX12_H	209
#define ICR_V1_HS_P_AX13_L	210
#define ICR_V1_HS_P_AX13_H	211
#define ICR_V1_HS_P_AX14_L	212
#define ICR_V1_HS_P_AX14_H	213
#define ICR_V1_HS_P_H1	214
#define ICR_V1_HS_P_H2	215
#define ICR_V1_HS_P_H3	216
#define ICR_V1_HS_P_H4	217
#define ICR_V1_HS_P_H5	218
#define ICR_V1_HS_P_H6	219
#define ICR_V1_HS_P_H7	220
#define ICR_V1_HS_P_H8	221
#define ICR_V1_HS_P_H9	222
#define ICR_V1_HS_P_H10	223
#define ICR_V1_HS_P_H11	224
#define ICR_V1_HS_P_H12	225
#define ICR_V1_HS_P_H13	226
#define ICR_V1_HS_P_H14	227
#define ICR_V1_HS_P_S1	228
#define ICR_V1_HS_P_S2	229
#define ICR_V1_HS_P_S3	230
#define ICR_V1_HS_P_S4	231
#define ICR_V1_HS_P_S5	232
#define ICR_V1_HS_P_S6	233
#define ICR_V1_HS_P_S7	234
#define ICR_V1_HS_P_S8	235
#define ICR_V1_HS_P_S9	236
#define ICR_V1_HS_P_S10	237
#define ICR_V1_HS_P_S11	238
#define ICR_V1_HS_P_S12	239
#define ICR_V1_HS_P_S13	240
#define ICR_V1_HS_P_S14	241
#define ICR_V1_HS_P_GL	242
#define ICR_V1_HS_P_MAXSAT_RGB_Y_L	243
#define ICR_V1_HS_P_MAXSAT_RGB_Y_H	244
#define ICR_V1_HS_P_MAXSAT_RCR_L	245
#define ICR_V1_HS_P_MAXSAT_RCR_H	246
#define ICR_V1_HS_P_MAXSAT_RCB_L	247
#define ICR_V1_HS_P_MAXSAT_RCB_H	248
#define ICR_V1_HS_P_MAXSAT_GCR_L	249
#define ICR_V1_HS_P_MAXSAT_GCR_H	250
#define ICR_V1_HS_P_MAXSAT_GCB_L	251
#define ICR_V1_HS_P_MAXSAT_GCB_H	252
#define ICR_V1_HS_P_MAXSAT_BCR_L	253
#define ICR_V1_HS_P_MAXSAT_BCR_H	254
#define ICR_V1_HS_P_MAXSAT_BCB_L	255
#define ICR_V1_HS_P_MAXSAT_BCB_H	256
#define ICR_V1_HS_P_ROFF_L	257
#define ICR_V1_HS_P_ROFF_H	258
#define ICR_V1_HS_P_GOFF_L	259
#define ICR_V1_HS_P_GOFF_H	260
#define ICR_V1_HS_P_BOFF_L	261
#define ICR_V1_HS_P_BOFF_H	262
#define ICR_V1_GCSC_M_C0_L	263
#define ICR_V1_GCSC_M_C0_H	264
#define ICR_V1_GCSC_M_C1_L	265
#define ICR_V1_GCSC_M_C1_H	266
#define ICR_V1_GCSC_M_C2_L	267
#define ICR_V1_GCSC_M_C2_H	268
#define ICR_V1_GCSC_M_C3_L	269
#define ICR_V1_GCSC_M_C3_H	270
#define ICR_V1_GCSC_M_C4_L	271
#define ICR_V1_GCSC_M_C4_H	272
#define ICR_V1_GCSC_M_C5_L	273
#define ICR_V1_GCSC_M_C5_H	274
#define ICR_V1_GCSC_M_C6_L	275
#define ICR_V1_GCSC_M_C6_H	276
#define ICR_V1_GCSC_M_C7_L	277
#define ICR_V1_GCSC_M_C7_H	278
#define ICR_V1_GCSC_M_C8_L	279
#define ICR_V1_GCSC_M_C8_H	280
#define ICR_V1_GCSC_M_O1_0	281
#define ICR_V1_GCSC_M_O1_1	282
#define ICR_V1_GCSC_M_O1_2	283
#define ICR_V1_GCSC_M_O2_0	284
#define ICR_V1_GCSC_M_O2_1	285
#define ICR_V1_GCSC_M_O2_2	286
#define ICR_V1_GCSC_M_O3_0	287
#define ICR_V1_GCSC_M_O3_1	288
#define ICR_V1_GCSC_M_O3_2	289
#define ICR_V1_GCSC_P_C0_L	290
#define ICR_V1_GCSC_P_C0_H	291
#define ICR_V1_GCSC_P_C1_L	292
#define ICR_V1_GCSC_P_C1_H	293
#define ICR_V1_GCSC_P_C2_L	294
#define ICR_V1_GCSC_P_C2_H	295
#define ICR_V1_GCSC_P_C3_L	296
#define ICR_V1_GCSC_P_C3_H	297
#define ICR_V1_GCSC_P_C4_L	298
#define ICR_V1_GCSC_P_C4_H	299
#define ICR_V1_GCSC_P_C5_L	300
#define ICR_V1_GCSC_P_C5_H	301
#define ICR_V1_GCSC_P_C6_L	302
#define ICR_V1_GCSC_P_C6_H	303
#define ICR_V1_GCSC_P_C7_L	304
#define ICR_V1_GCSC_P_C7_H	305
#define ICR_V1_GCSC_P_C8_L	306
#define ICR_V1_GCSC_P_C8_H	307
#define ICR_V1_GCSC_P_O1_0	308
#define ICR_V1_GCSC_P_O1_1	309
#define ICR_V1_GCSC_P_O1_2	310
#define ICR_V1_GCSC_P_O2_0	311
#define ICR_V1_GCSC_P_O2_1	312
#define ICR_V1_GCSC_P_O2_2	313
#define ICR_V1_GCSC_P_O3_0	314
#define ICR_V1_GCSC_P_O3_1	315
#define ICR_V1_GCSC_P_O3_2	316
#define ICR_V1_CPCB_PIXVAL_M_EN	317
#define ICR_V1_CPCB_PIXVAL_P_EN	318
#define ICR_V1_IGNORE -1
#endif /* __PXA168_IOCTLICR_H */

