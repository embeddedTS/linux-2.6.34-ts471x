/*
 * include/linux/card.h
 *
 * Card platform data
 *
 * Copyright (C) 2008 Marvell International Ltd.
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */


#ifndef __LINUX_CARD_H__
#define __LINUX_CARD_H__

struct card_platform_data {
	int pin_detect;
	int (*mfp_config)(void);
	/* to be continue */
};


#endif

