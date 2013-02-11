/*
 * include/asm-arm/arch-pxa/max8660.h
 *
 * Copyright (C) 2009, Marvell Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _MAX8660_H_
#define _MAX8660_H_
#include <linux/i2c.h>
#include <linux/i2c-pxa.h>

#define MAX8660ETL			1
#undef  MAX8660AETL

#undef  V1_SET1_IN			
#define V1_SET1_UNCONNECTED		1
#undef  V1_SET1_GROUND	

#undef	V2_SET2_IN			
#undef  V2_SET2_UNCONNECTED
#define V2_SET2_GROUND			1	


#define MAX8660_ID			0x34
#define MAX8660_REG_NUM			0xb

static const int max8660_reg_map[MAX8660_REG_NUM]={
        0x10, 0x12, 0x20,
        0x23, 0x24, 0x29,
        0x2A, 0x32, 0x33,
        0x39, 0x80
};


/* definition for MAX8660 Register remapping */
#define MAX8660_OVER1			0	    /* the real address should be 0x10 */
#define MAX8660_OVER2			1	    /* the real address should be 0x12 */
#define MAX8660_VCC1			2	    /* the real address should be 0x20 */
#define MAX8660_ADTV1			3	    /* the real address should be 0x23 */
#define MAX8660_ADTV2			4	    /* the real address should be 0x24 */
#define MAX8660_SDTV1			5	    /* the real address should be 0x29 */
#define MAX8660_SDTV2			6	    /* the real address should be 0x2A */
#define MAX8660_MDTV1			7	    /* the real address should be 0x32 */
#define MAX8660_MDTV2			8	    /* the real address should be 0x33 */
#define MAX8660_L12VCR			9	    /* the real address should be 0x39 */
#define MAX8660_FPWM			0xa	    /* the real address should be 0x80 */


#define OVER1_EN3			1<<0
#define OVER1_EN4			1<<2
#define OVER2_EN6			1<<1
#define OVER2_EN7			1<<2

#define VCC1_AGO			1<<0
#define VCC1_AVS			1<<1
#define VCC1_SGO			1<<4
#define VCC1_SVS			1<<5
#define VCC1_MGO			1<<6
#define VCC1_MVS			1<<7

#define FPWM_FPWM1			1<<0
#define FPWM_FPWM2			1<<1
#define FPWM_FPWM3			1<<2
#define FPWM_FPWM4			1<<3
#define FPWM_ARD3			1<<6
#define FPWM_ADR4			1<<7

enum {
	MAX8660_V1 = 1,
	MAX8660_V2,
	MAX8660_V3,
	MAX8660_V4,
	MAX8660_V5,
	MAX8660_V6,
	MAX8660_V7,
	MAX8660_V8,
};

#if defined(MAX8660ETL)

#if defined(V1_SET1_IN)
#define	MAX8660_V1_DEFAULT		3300
#elif defined(V1_SET1_UNCONNECTED)
#define	MAX8660_V1_DEFAULT		3000
#elif defined(V1_SET1_GROUND)
#define	MAX8660_V1_DEFAULT		2850
#endif

#if defined(V2_SET2_IN)
#define	MAX8660_V2_DEFAULT		3300
#elif defined(V2_SET2_UNCONNECTED)
#define	MAX8660_V2_DEFAULT		2500
#elif defined(V2_SET2_GROUND)
#define	MAX8660_V2_DEFAULT		1800
#endif

#elif defined(MAX8660AETL)

#if defined(V1_SET1_IN)
#define	MAX8660_V1_DEFAULT		2500
#elif defined(V1_SET1_UNCONNECTED)
#define	MAX8660_V1_DEFAULT		2000
#elif defined(V1_SET1_GROUND)
#define	MAX8660_V1_DEFAULT		1800
#endif

#if defined(V2_SET2_IN)
#define	MAX8660_V2_DEFAULT		2500
#elif defined(V2_SET2_UNCONNECTED)
#define	MAX8660_V2_DEFAULT		2000
#elif defined(V2_SET2_GROUND)
#define	MAX8660_V2_DEFAULT		1800
#endif

#endif


#define	MAX8660_V3_BASE			725
#define	MAX8660_V3_STEP			25
#define	MAX8660_V3_MAX			1800
#define MAX8660_V3_DEFAULT		1400

#define	MAX8660_V4_BASE			725
#define	MAX8660_V4_STEP			25
#define	MAX8660_V4_MAX			1800
#define MAX8660_V4_DEFAULT		1400

#define	MAX8660_V5_BASE			1700
#define	MAX8660_V5_STEP			25
#define	MAX8660_V5_MAX			2000
#define MAX8660_V5_DEFAULT		1800

#define	MAX8660_V6_BASE			1800
#define	MAX8660_V6_STEP			100
#define	MAX8660_V6_MAX			3300
#define MAX8660_V6_DEFAULT		1800

#define	MAX8660_V7_BASE			1800
#define	MAX8660_V7_STEP			100
#define	MAX8660_V7_MAX			3300
#define MAX8660_V7_DEFAULT		1800

#define MAX8660_V8_DEFAULT		3300

struct max8660_platform_data {
	int             (*init_irq)(void);
	int             (*ack_irq)(void);
	void            (*platform_init)(void);
	spinlock_t      lock;
	struct work_struct      work;
	struct power_chip       *power_chips;
};


#endif

