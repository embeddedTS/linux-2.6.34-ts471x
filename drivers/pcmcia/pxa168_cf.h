/*
 * linux/drivers/pcmcia/aspen_cf.h
 *
 * Author:	Alex Kaluzhny <akaluzhn@marvell.com>
 * Created:	May 1, 2008
 * Copyright (C) 2006 Marvell International Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef __LINUX_ASPEN_CF_H
#define __LINUX_ASPEN_CF_H

#include <linux/types.h>

/* CF Registers */
#define CF_BASE_ADDR	0xD4285000

#define CF_MEM_BASE_OFFSET 0x800 
#define CF_ATTR_BASE_OFFSET 0xC00
#define CF_IO_BASE_OFFSET 0x30

/* CFI Status Register, all bits read only (RO) */
#define CFI_SR_OFFSET			(0x0000)
#define CFI_SR_STATUS_CHANGE		(0x1<<0)
#define CFI_SR_BINARY_AUDIO_OUT		(0x1<<1)
#define CFI_SR_CARD_DETECT_1		(0x1<<2)
#define CFI_SR_CARD_DETECT_2		(0x1<<3)
#define CFI_SR_INPUT_ACK		(0x1<<4)
#define CFI_SR_CARD_READY		(0x1<<5)
#define CFI_SR_BUS_CYCLE_WAIT		(0x1<<6)
#define CFI_SR_16BIT_IO_PORT		(0x1<<7)

/* CF IRQ Register */
#define CF_IRQ_OFFSET			(0x0004)
#define CF_IRQ_CARD_DETECT		(0x1<<0)
#define CF_IRQ_STATUS_CHANGE		(0x1<<1)
#define CF_IRQ_MEMO_MODE_READY		(0x1<<2)
#define CF_IRQ_IO_MODE_IREQ		(0x1<<3)
#define CF_IRQ_TRUEIDE_MODE_IREQ 	(0x1<<8)
#define CF_IRQ_PIO_TRANS_ERR_IRQ 	(0x1<<9)
#define CF_IRQ_BUFF_AVAL_IRQ		(0x1<<10)
#define CF_IRQ_TRANS_DONE_IRQ		(0x1<<11)
#define CF_IRQ_WAKEUP			(0x1<<12)

/* CF Interrupt Enable Register */
#define CF_INTEN_OFFSET			(0x0008)
#define CF_INTEN_CARD_DETECT		(0x1<<0)
#define CF_INTEN_STATUS_CHANGE		(0x1<<1)
#define CF_INTEN_MEM_MODE_READY		(0x1<<2)
#define CF_INTEN_IO_MODE_IREQ		(0x1<<3)
#define CF_INTEN_TRUEIDE_MODE_IREQ	(0x1<<8)
#define CF_INTEN_PIO_TRANS_ERR_IRQ	(0x1<<9)
#define CF_INTEN_BUFF_AVAL_IRQ		(0x1<<10)
#define CF_INTEN_TRANS_DONE_IRQ		(0x1<<11)
#define CF_INTEN_WAKEUP_EN		(0x1<<12)		

/* CF Operation Mode Register */
#define CF_OPMODE_OFFSET		(0x000C)
#define CF_OPMODE_CARD_MODE_MASK	(0x3<<0)
#define CF_OPMODE_CARD_TYPE		(0x1<<2)
#define CF_OPMODE_CARD_RESET		(0x1<<3)
#define CF_OPMODE_CFHOST_ENABLE		(0x1<<4)
#define CF_OPMODE_TRISTATE		(0x1<<5)
#define CF_OPMODE_ULTADMA_ENABLE	(0x1<<8)
#define CF_OPMODE_TRUEIDE_MW_DMA_ENABLE	(0x1<<9)
#define CF_OPMODE_SLAVE_DMA_ENABLE	(0x1<<10)
#define CF_OPMODE_DRQ_BLOCK_SIZE	(0x3<<11)
#define CF_OPMODE_AHB_STRICT_BURST	(0x1<<13)
#define CF_OPMODE_SLAVE_DMA_BURST_LENGTH (0xFF<<16)

/* CF Interface Clock Configuration Register */
#define CFI_CLOCK_CONFIG_OFFSET		(0x0010)
#define CFI_CLOCK_CONFIG_MASK		(0xF)
#define CFI_CLOCK_RATIO_MASK		(0x7<<4)

/* CF Timing Mode Configuration Register */
#define CF_TMCFG_OFFSET			(0x0014)
#define CF_TMCFG_MEM_TRANS_MASK		(0x3<<0)
#define CF_TMCFG_IO_TRANS_MASK		(0x3<<2)
#define CF_TMCFG_TRUEIDE_PIO_TRANS_MASK	(0x7<<4)
#define CF_TMCFG_TRUEIDE_MW_DMA_TRANS_MASK (0x7<<7)
#define CF_TMCFG_ULTRA_DMA_TRANS_MASK	(0x7<<10)

/* CF Transfer Address Register */
#define CF_TADDR_OFFSET			(0x0018)
#define CF_TADDR_TRANS_ADDR_MASK	(0x7FF)

/* CF Transfer Control Register */
#define CF_TCNTR_OFFSET			(0x001C)
#define CF_TCNTR_COUNT_MASK		(0x3FFFF)
#define CF_TCNTR_DISABLE_ADDR_INC	(0x1<<24)
#define CF_TCNTR_TRANS_SIZE_WORD	(0x1<<25)
#define CF_TCNTR_COMMON_ATTR_MEM 	(0x1<<26)
#define CF_TCNTR_MEM_IO_TRANS		(0x1<<27)
#define CF_TCNTR_DMA_TRANS_MODE		(0x1<<28)
#define CF_TCNTR_SLAVE_DMA_TRANS	(0x1<<29)
#define CF_TCNTR_TRANS_DIR_WRITE	(0x1<<30)
#define CF_TCNTR_MEM_TRANS_START	(0x1<<31)

/* CF Write Data Port Register */
#define CF_WDPR_OFFSET			(0x0024)

/* CF Read Data Port Register */
#define CF_RDPR_OFFSET			(0x0028)

/* CF Write Data Port Register */
#define CF_EXT_WDPR_OFFSET              (0x0200)

/* CF Read Data Port Register */
#define CF_EXT_RDPR_OFFSET              (0x0400)

/* CF ATA Data Port Register */
#define CF_ATADPR_OFFSET		(0x0030)
#define CF_ATADPR_MASK			(0xFFFF)

/* CF ATA Error/Features Register */
#define CF_ATAERR_OFFSET		(0x0034)
#define CF_ATAERR_MASK			(0xFF)

/* CF ATA Sector Count Register */
#define CF_ATASCOUNTR_OFFSET		(0x0038)
#define CF_ATASCOUNTR_MASK		(0xFF)

/* CF ATA Sector Number Register */
#define CF_ATASNR_OFFSET		(0x003C)
#define CF_ATASNR_MASK			(0xFF)

/* CF ATA Cylinder Low Register */
#define CF_ATACLR_OFFSET		(0x0040)
#define CF_ATACLR_MASK			(0xFF)

/* CF ATA Cylinder High Register */
#define CF_ATACHR_OFFSET		(0x0044)
#define CF_ATACHR_MASK			(0xFF)

/* CF ATA Select Card/Head Register */
#define CF_ATASCARDR_OFFSET		(0x0048)
#define CF_ATASCARDR_MASK		(0xFF)

/* CF ATA Status/Command Register */
#define CF_ATASTAT_OFFSET		(0x004C)
#define CF_ATASTAT_MASK			(0xFF)

/* CF ATA Alt Status/Device Control Register */
#define CF_ATAALTSTAT_OFFSET		(0x0050)
#define CF_ATAALTSTAT_MASK		(0xFF)


enum cf_op_card_mode {
	PC_CARD_MEM = 0,
	PC_CARD_IO =  0x1,
	CARD_TRUE_IDE = 0x2,
};

enum cf_op_card_type {
	COMPACT_FLASH = 0,
	CF_PLUS =       0x1,
};

enum cf_op_drq_block_size {
	DRQ_BLOCK_SIZE_512 =  0,
	DRQ_BLOCK_SIZE_1024 = 0x1,
	DRQ_BLOCK_SIZE_2048 = 0x2,
	DRQ_BLOCK_SIZE_4096 = 0x3,
};

enum cf_clock_config {
	CLOCK_CONFIG_100M = 0,
	CLOCK_CONFIG_75M = 0x1,
	CLOCK_CONFIG_66M = 0x2,
	CLOCK_CONFIG_50M = 0x3,
	CLOCK_CONFIG_40M = 0x4,
	CLOCK_CONFIG_33M = 0x5,
	CLOCK_CONFIG_25M = 0x6,
};

enum cf_clock_ratio {
	CLOCK_SAME_SOURCE = 0,
	CLOCK_RATIO_1 = 0x1,
	CLOCK_RATIO_2 = 0x2,
	CLOCK_RATIO_3 = 0x3,
	CLOCK_RATIO_4 = 0x4,
	CLOCK_RATIO_5 = 0x5,
	CLOCK_RATIO_6 = 0x6,
	CLOCK_RATIO_7 = 0x7,
};

enum cf_timing_mode_mem_trans {
	MEM_TRANS_250NS = 0,
	MEM_TRANS_120NS = 0x1,
	MEM_TRANS_100NS = 0x2,
	MEM_TRANS_80NS =  0x3,
};

enum cf_timing_mode_io_trans {
	IO_TRANS_250NS = 0,
	IO_TRANS_120NS = 0x1,
	IO_TRANS_100NS = 0x2,
	IO_TRANS_80NS =  0x3,
};

enum cf_timing_true_ide_mode {
	TRUE_IDE_MODE_0 = 0,
	TRUE_IDE_MODE_1 = 0x1,
	TRUE_IDE_MODE_2 = 0x2,
	TRUE_IDE_MODE_3 = 0x3,
	TRUE_IDE_MODE_4 = 0x4,
	TRUE_IDE_MODE_5 = 0x5,
	TRUE_IDE_MODE_6 = 0x6,
};	



#endif /* __LINUX_ASPEN_CF_H */
