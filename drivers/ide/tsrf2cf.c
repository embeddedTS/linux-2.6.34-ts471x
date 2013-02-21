/* linux/drivers/ide/tsrf2.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * 2007-01-03:mos:modified for TS-7800 with TS-RF2-CF daughter card
 * 2013-01-07:mlf:updated for TS-4700 with module arguments for others
*/
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/ide.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <asm/mach-types.h>
#include <asm/io.h>
#include <asm/irq.h>

/*
TS-RF2 CF memory map:
BASE + 0x0 - ID register, must read 0x8E
BASE + 0x6 - CF aux reg #0
BASE + 0x7 - CF aux reg #1
BASE + 0x8-0xf - CF IDE regs #0 - #7
*/

static char aio8[16];
static char aio16[16];
static char airq[16];

#define TS4700_ISA8_BUS 	0x81008000
#define TS4700_ISA16_BUS 	0x80008000
#define IRQ_MARVELL_ISA 	62+5

#define RF2_IRQ6		0x40
#define RF2_IRQ7		0x80

#define TSRF2_IDENTIFIER 	0x8E
#define TSRF2_ID 		0x0
#define TSRF2_AUX1		0x6
#define TSRF2_AUX2		0x7
#define TSRF2_DATA		0x8
#define TSRF2_IDEBASE		0x9

static const uint isa_offsets[] = {
	0x100,
	0x110,
	0x200,
	0x210,
};

static const struct ide_tp_ops tsrf2_tp_ops = {
	.exec_command   = ide_exec_command,
	.read_status    = ide_read_status,
	.read_altstatus = ide_read_altstatus,
	.write_devctl   = ide_write_devctl,

	.dev_select     = ide_dev_select,
	.tf_load        = ide_tf_load,
	.tf_read        = ide_tf_read,

	.input_data     = ide_input_data,
	.output_data    = ide_output_data,

};

static const struct ide_port_info tsrf2_ide_port_info = {
	.name				= "TS-RF2-CF",
	.tp_ops				= &tsrf2_tp_ops,
	.chipset        		= ide_generic,
	.irq_flags			= IRQF_SHARED,
	.host_flags         		= IDE_HFLAG_MMIO |
					      IDE_HFLAG_NO_DMA |
					      IDE_HFLAG_SINGLE |
					      IDE_HFLAG_NON_BOOTABLE |
					      IDE_HFLAG_UNMASK_IRQS |
					      IDE_HFLAG_NO_IO_32BIT,
};

static int tsrf2_register(unsigned long addr8, unsigned long addr16, unsigned long offset, int irq)
{
	volatile unsigned long isa8, isa16;
	struct ide_host *host;
	struct ide_hw hw, *hws[] = { &hw };
	
	memset(&hw, 0, sizeof(hw));

	isa8 = (unsigned long) ioremap(addr8 + offset, 32);
	isa16 = (unsigned long) ioremap(addr16 + offset, 32);

	hw.io_ports.data_addr 	= isa16 + TSRF2_DATA;
	hw.io_ports.error_addr 	= isa8 + TSRF2_IDEBASE;
	hw.io_ports.nsect_addr 	= isa8 + TSRF2_IDEBASE + 1;
	hw.io_ports.lbal_addr 	= isa8 + TSRF2_IDEBASE + 2;
	hw.io_ports.lbam_addr	= isa8 + TSRF2_IDEBASE + 3;
	hw.io_ports.lbah_addr	= isa8 + TSRF2_IDEBASE + 4;
	hw.io_ports.device_addr = isa8 + TSRF2_IDEBASE + 5;
	hw.io_ports.status_addr = isa8 + TSRF2_IDEBASE + 6;
	hw.io_ports.ctl_addr 	= isa8 + TSRF2_AUX1;
	hw.irq = irq;

	host = ide_host_alloc(&tsrf2_ide_port_info, hws, 1);
	host->irq_handler = ide_intr;
	if (!host) {
		printk(KERN_ERR "TS-RF2-CF: failed to allocate ide host\n");
		return -ENOMEM;
	}

	ide_host_register(host, &tsrf2_ide_port_info, hws);

	return 0;
}

// return IRQ if found, or 0 otherwise
int tsrf2_detect(int offset, unsigned long addr8, unsigned long addr16, unsigned int irq) {
	volatile unsigned char *sh;
	int j=0;

	sh = ((volatile unsigned char *) ioremap(addr8, 32));
	if (sh == (volatile unsigned char *)addr8) {
		printk(KERN_ERR "TS-RF2-CF: cannot map 8-bit I/O\n");
		return 0;
	}
	sh += offset;
	if (sh[0] == TSRF2_IDENTIFIER) {
		// found board
		sh[2] = 0xE9; // make sure unused UART is off
		if (sh[1] & RF2_IRQ7) {
			j = irq + 2;
		} else if (sh[1] & RF2_IRQ6) {
			j = irq + 1;
		} else {
			printk(KERN_ERR "TS-RF2-CF: Found device at 0x%X, but no IRQ (skipping)\n", offset);
		}
	}
	iounmap(sh);
	return j;
}

static void __init tsrf2_autodetect(unsigned long addr8, unsigned long addr16, unsigned int irq)
{
	int i;

	for(i = 0; i < 4; i++) {
		int j;
		j = tsrf2_detect(isa_offsets[i], addr8, addr16, irq);
		if(j != 0) {
			volatile unsigned char *sh;
			// If there is nothing on the MUXBUS it will return the last value successfully read
			// which will cause this to think there are TS-RF2-CFs everywhere after the first one.
			// Invoke a read to 0x2 because it will never be 0x8e in a usable condition anyway
			sh = ((volatile unsigned char *) ioremap(addr8, 0xf));
			sh += isa_offsets[i];
			sh[0x2];
			__iounmap(sh);

			printk(KERN_INFO "TS-RF2-CF: Detected at 0x%X, IRQ %d\n", isa_offsets[i], j);
			tsrf2_register(addr8, addr16, isa_offsets[i], j);
		}
	}
}

static int __init tsrf2_init(void)
{
	// Assumes the TS-4700, but module arguments will be used instead if set
	unsigned long addr8 = TS4700_ISA8_BUS;
	unsigned long addr16 = TS4700_ISA16_BUS;
	int irq = IRQ_MARVELL_ISA;  // points to IRQ5

	if (strlen(aio8))
		addr8 = simple_strtoul(aio8, NULL, 0);

	if (strlen(aio16))
		addr16 = simple_strtoul(aio16, NULL, 0);

	if (strlen(airq))
		irq = simple_strtoul(airq, NULL, 0);

	tsrf2_autodetect(addr8, addr16, irq);

	return 0;
}

module_init(tsrf2_init);

module_param_string(irq, airq, sizeof(airq), 0644); // Point to IRQ 5
module_param_string(io8, aio8, sizeof(aio8), 0644); // ISA 8-bit bus at addr 0x0
module_param_string(io16, aio16, sizeof(aio16), 0644); // ISA 16-bit bus at addr 0x0

MODULE_AUTHOR("Mark Featherston <mark@embeddedarm.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("TS-RF2-CF IDE driver");
