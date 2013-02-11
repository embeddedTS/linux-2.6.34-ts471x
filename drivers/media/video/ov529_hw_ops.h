/*
 * Provide definitions for SMC interface operations of the ov529
 * camera controler driver.
 *
 * Copyright 2009 Marvell International Ltd.
 * 		Weili Xia <wlxia@marvell.com>
 *
 * This file may be distributed under the terms of the GNU General
 * Public License, version 2.
 */

#ifndef __OV529_HW_INTF_H__
#define __OV529_HW_INTF_H__

#include <mach/ov529.h>

/* #define SMC_USE_DMA */
#undef SMC_USE_DMA

enum ov529_cmd_id {
	INVALID,
	INITIAL,
	SET_REG = 3,
	GET_PIC,
	SNAPSHOT,
	SAVEDATA,
	RESET = 8,
	PWR_OFF,
	DATA,
	GET_REG,
	DOWNLOAD_PROG,
	SYNC,
	ACK,
	NACK,
	SEL_IMG_QUALITY,
	SET_LIGHT_COND,
	DIG_ZOOM,
	SET_LIGHT_FREQ
};

enum ov529_pic_type {
	SNAP_PIC = 1,
	PREV_PIC,
	SERIAL_FLASH_PIC,
	COMPRESSION_PREV_PIC = 5,
	PLAYBACK_PIC,
};

struct ov529_cmd {
	char ff1;
	char ff2;
	char ff3;
	char id;
	char p1;
	char p2;
	char p3;
	char p4;
};

struct ov529_hw_ops {
	char name[32];
	int (*init)(struct platform_device *pdev, struct resource *res);
	int (*remove)(void);
	int (*startup)(void);
	int (*data_ready)(void);
	int (*streamon)(void);
	int (*recv_data)(char *buf, int size);
	int (*send_cmd)(int id, int p1, int p2, int p3, int p4);
};

extern struct ov529_hw_ops ov529_smc_ops;

#endif
