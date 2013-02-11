/*
 * linux/arch/arm/mach-ttc/timer_services.c
 *
 * Support for the MMP Development Platform
 *
 * Copyright (C) 2008 Marvell International Ltd.
 *
 * 2009-08-1: Ofer Zaarur <ozaarur@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 */
#ifndef TIMER_SERV_H
#define TIMER_SERV_H 1

#define RESOLUTION_0  	(CLOCK_TICK_RATE / 1500 + 1/2)  /* Define resolution */
#define RESOLUTION_1	0
#define RESOLUTION_2	0
#define COUNTER_0 0
#define COUNTER_1 1
#define COUNTER_2 2

#define MATCH_REGISTER_OFFSET sizeof(int) /* HW dependent */

struct timer_device {
	struct platform_device *pdev;
	struct list_head	node;
	void __iomem	*mmio_base;
	const char	*label;
	int		counter_id;
	int		type;
	int		use_count;
	int		irq;
	unsigned int 	jiff;
	unsigned long	resolution;
};


struct timer_dev {
	struct timer_device *timer;
	u32 counter;
	u32 resolution;
	int irq;

};

enum cpu_timer_type {
	TIMER_UNDEFINED = 0,
	MMP_TIMER,
};


struct timer_device *timer_request(int counter, const char *label);
void timer_free(struct timer_device *timer);
void timer_services_disable(int id);
void timer_services_enable(int id, int period);
unsigned int timer_services_counter_read(int id);
unsigned long get_jiff(int id);


/*mapping the MMP timer registers */
#define PHYS_IO_START		0xD4000000   /* Physical Address of IO */
#define AP_APB_BASE_OFFSET	0x00010000   /* AP ABP base address offset */
/* Timer definitions */

#define TMR_IO_BASE_OFFSET	0x00014000   /* Timers IO Base Offset */
#define TMR2_IO_BASE_OFFSET	0x00016000   /* Timer2 IO Base Offset */
#define TMR_CCR_OFFSET		0x0000	     /* Timer Clock Control Register */
#define TMR_T0_M0_OFFSET	0x0004	     /* Timer Map Registers */
#define TMR_T0_M1_OFFSET	0x0008
#define TMR_T0_M2_OFFSET	0x000C
#define TMR_T1_M0_OFFSET	0x0010
#define TMR_T1_M1_OFFSET	0x0014
#define TMR_T1_M2_OFFSET	0x0018
#define TMR_T2_M0_OFFSET	0x001C
#define TMR_T2_M1_OFFSET	0x0020
#define TMR_T2_M2_OFFSET	0x0024
#define TMR_CR0_OFFSET		0x0028	   /* Timer Control Registers */
#define TMR_CR1_OFFSET		0x002C
#define TMR_CR2_OFFSET		0x0030
#define TMR_SR0_OFFSET		0x0034	   /* Timer Status Registers */
#define TMR_SR1_OFFSET		0x0038
#define TMR_SR2_OFFSET		0x003C
#define TMR_IER0_OFFSET	0x0040	   /* Timer Interrupt Enable Reg*/
#define TMR_IER1_OFFSET	0x0044
#define TMR_IER2_OFFSET	0x0048
#define TMR_PLVR0_OFFSET	0x004C	   /* Timer Preload Value Registers */
#define TMR_PLVR1_OFFSET	0x0050
#define TMR_PLVR2_OFFSET	0x0054
#define TMR_PLCR0_OFFSET	0x0058	   /* Timer Preload Control Registers*/
#define TMR_PLCR1_OFFSET	0x005C
#define TMR_PLCR2_OFFSET	0x0060
#define TMR_WMER_OFFSET	0x0064	   /* Timer Watchdog Match Enable reg*/
#define TMR_WMR_OFFSET		0x0068	   /* Timer Watchdog Match Register */
#define TMR_WVR_OFFSET		0x006C	   /* Timer Watchdog Value Register */
#define TMR_WSR_OFFSET		0x0070	   /* Timer Watchdog Status Register */
#define TMR_ICR0_OFFSET	0x0074	   /* Timer Interrupt Clear Registers*/
#define TMR_ICR1_OFFSET	0x0078
#define TMR_ICR2_OFFSET	0x007C
#define TMR_WICR_OFFSET	0x0080	   /*Timer Watchdog Interrupt Clr Reg*/
#define TMR_CER_OFFSET		0x0084	   /* Timer Count Enable Register */
#define TMR_CMR_OFFSET		0x0088	   /* Timer Count Mode Register */
#define TMR_ILR0_OFFSET	0x008C	   /*Timer Interrupt Length Registers*/
#define TMR_ILR1_OFFSET	0x0090
#define TMR_ILR2_OFFSET	0x0094
#define TMR_WCR_OFFSET		0x0098	/*Watchdog Counter Reset Register*/
#define TMR_WFAR_OFFSET	0x009C 	/*Watchdog First Access Register*/
#define TMR_WSAR_OFFSET	0x00A0	/*Watchdog Second Access Register*/
#define TMR_CVWR0_OFFSET 	0x00A4 	/*Counters Value Write 4 Read Request*/
#define TMR_CVWR1_OFFSET	0x00A8
#define TMR_CVWR2_OFFSET	0x00AC

#endif
