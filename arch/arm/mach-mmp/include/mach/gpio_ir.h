/*
 * linux/include/asm-arm/arch-ttc/gpio_ir.h
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

#define uart_putc(x) 	/*printk("x") */

#include <linux/list.h>

/*
 * Ugly macro :-)
 */
#define DISABLE_IRINT()
#define ENABLE_IRINT()

#define IR_IDLE_LENGTH	0x7fffffff
#define CUSTOM_CODE 	0x916E
#define CODE_SIZE	16
	 
/*
 * General Purpose I/O
 */
#define GAPMASK0	(0x9C)  /* BW GPIO unmask Register GPIO<31:0> */
#define GCPMASK0	(0xA8)  /* BW GPIO mask Register GPIO<31:0> */

/* More handy macros.  The argument is a literal GPIO number. */
 
/*
 * Infrared pin definition
 */
#define IR_PIN		ir_pin /* GPIO pin number */
#define IR_SHIFT	(IR_PIN % (sizeof(int) * 8)) /* Shift6*/
#define GPIO_BIT(x)	(1 << ((x) & 0x1f))

typedef struct {
	unsigned int ir_encode;
	unsigned int ir_key;
} ir_key_table_t;

struct cir_device {
	struct platform_device *pdev;
	struct input_dev *input_dev;
	void __iomem	*mmio_base;
	unsigned long	phys_base;
	int		irq;
};

struct cir_dev {
	struct cir_device *cir;
	int irq;
};

extern unsigned int ir_key;
extern unsigned int ir_key_recvflag;

void readout_ircbuffer(unsigned long nodata);
int infrared_int_handler(unsigned int data);
int cir_init(struct cir_dev *dev);
void cir_exit(struct cir_dev *dev);
