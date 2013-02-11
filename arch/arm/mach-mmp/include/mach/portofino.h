/*
 * Copyright (C) 2008, Marvell Corporation.
 * Author: Bin Yang <bin.yang@marvell.com> 
 * 	    Yael Sheli Chemla<yael.s.shemla@marvell.com> 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __MACH_PORTOFINO_H
#define __MACH_PORTOFINO_H
#include <linux/i2c.h>

#define PORTOFINO_ADDRESS	0x11 /*88PM8606,  (codenamed 'Portofino')*/

#define PORTOFINO_ID		0x17
#define PORTOFINO_PREREG1	0x10

/* General */
int portofino_read(u8 reg, u8 *val);
int portofino_write(u8 reg, u8 val);

#endif

