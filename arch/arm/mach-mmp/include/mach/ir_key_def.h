/*
 * linux/include/asm-arm/arch-ttc/ir_key_def.h
 *
 * Support for the ASPEN /Zyloniteii /Teton development Platforms
 *
 * Copyright (C) 2008 Marvell International Ltd.
 *
 * 2008-010-12: Ofer Zaarur <ofer.zaarur@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
/*
 * IR key definition 
 */
typedef enum {
	MV_IR_KEY_NULL = 0,

	/* digital keys */
	MV_IR_KEY_DIGIT_1,
	MV_IR_KEY_DIGIT_16 = 2,
	MV_IR_KEY_DIGIT_17,
	MV_IR_KEY_DIGIT_18,
	MV_IR_KEY_DIGIT_19,
	MV_IR_KEY_DIGIT_20,
	MV_IR_KEY_DIGIT_21,
	MV_IR_KEY_DIGIT_22,
	MV_IR_KEY_DIGIT_23,
	MV_IR_KEY_DIGIT_24,
	MV_IR_KEY_DIGIT_26,

	MV_IR_KEY_DIGIT_28 = 30,
	MV_IR_KEY_DIGIT_29 = 48,
	MV_IR_KEY_DIGIT_30 = 46,
	MV_IR_KEY_DIGIT_31 = 32,
	MV_IR_KEY_DIGIT_32 = 18,
	MV_IR_KEY_DIGIT_33 = 33,

	MV_IR_KEY_DIGIT_2 = 34,
	MV_IR_KEY_DIGIT_3,
	MV_IR_KEY_DIGIT_4,
	MV_IR_KEY_DIGIT_5,
	MV_IR_KEY_DIGIT_6,
	MV_IR_KEY_DIGIT_7,

	MV_IR_KEY_DIGIT_8 = 103,
	MV_IR_KEY_DIGIT_9,
	MV_IR_KEY_DIGIT_10 = 105,
	MV_IR_KEY_DIGIT_11 = 28,
	MV_IR_KEY_DIGIT_12 = 106,
	MV_IR_KEY_DIGIT_13,
	MV_IR_KEY_DIGIT_14 = 108,
	MV_IR_KEY_DIGIT_15,


	MV_IR_KEY_DIGIT_25,
	MV_IR_KEY_DIGIT_27,

} MV_IR_KEY_CODE_t;

#define MV_IR_HOLDKEY_FLAG		(1 << 31)
#define MV_IR_HOLDKEY_KEY(key)	(key | MV_IR_HOLDKEY_FLAG)
#define MV_IR_HOLDKEY2KEY(key)	(key & (~MV_IR_HOLDKEY_FLAG))

