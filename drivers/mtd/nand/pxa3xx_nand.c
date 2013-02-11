/*
 * drivers/mtd/nand/pxa3xx_nand.c
 *
 * Copyright © 2005 Intel Corporation
 * Copyright © 2006 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifdef   __KERNEL__

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/slab.h>

#include <mach/dma.h>
#include <mach/pxa3xx_nand.h>
#include <mach/nand_supported.h>
#include <mach/pxa3xx_bbm.h>
#endif

#define NAND_STOP_DELAY		(100)
#undef PXA3XX_NAND_DEBUG
#ifdef PXA3XX_NAND_DEBUG
#ifdef PXA3XX_NAND_DEBUG_MORE_CONTROL
static int debug_nand = 0;
#define DBG_NAND(x)	do {if (debug_nand) {x;}}while(0)
#else
#define DBG_NAND(x)	do{x;}while(0)
#endif
#else
#define DBG_NAND(x)
#endif
static int no_error_dump = 0;

#if defined(CONFIG_DVFM)
#include <mach/dvfm.h>
static int dvfm_dev_idx;

static void set_dvfm_constraint(void)
{
	/* Disable Low power mode */
	dvfm_disable_op_name("apps_idle", dvfm_dev_idx);
	dvfm_disable_op_name("apps_sleep", dvfm_dev_idx);
	dvfm_disable_op_name("sys_sleep", dvfm_dev_idx);
}

static void unset_dvfm_constraint(void)
{
	/* Enable Low power mode */
	dvfm_enable_op_name("apps_idle", dvfm_dev_idx);
	dvfm_enable_op_name("apps_sleep", dvfm_dev_idx);
	dvfm_enable_op_name("sys_sleep", dvfm_dev_idx);
}

#else
static void set_dvfm_constraint(void) {}
static void unset_dvfm_constraint(void) {}
#endif

#ifdef CONFIG_MTD_CMDLINE_PARTS
static const char *part_probes[] = { "cmdlinepart", NULL };
static const char *mtd_names[] = {"pxa3xx_nand-0", "pxa3xx_nand-1", NULL};
#endif

/* convert nano-seconds to nand flash controller clock cycles */
#define ns2cycle(ns, clk)	(int)(((ns) * (clk / 1000000) / 1000) + 1)
#define cycle2ns(cycle, clk)	(cycle * 1000 / (clk / 1000000))
#define	CHIP_DELAY_TIMEOUT	(500)
#define BCH_THRESHOLD 		(8)
#define PAGE_CHUNK_SIZE		(2048)
#define OOB_CHUNK_SIZE		(64)

/* registers and bit definitions */
#define NDCR			(0x00) /* Control register */
#define NDTR0CS0		(0x04) /* Timing Parameter 0 for CS0 */
#define NDTR1CS0		(0x0C) /* Timing Parameter 1 for CS0 */
#define NDSR			(0x14) /* Status Register */
#define NDPCR			(0x18) /* Page Count Register */
#define NDBBR0			(0x1C) /* Bad Block Register 0 */
#define NDBBR1			(0x20) /* Bad Block Register 1 */
#define NDREDEL			(0x24) /* Read Enable Return Delay Register */
#define NDECCCTRL		(0x28) /* ECC Control Register */
#define NDBZCNT			(0x2C) /* Timer for NDRnB0 and NDRnB1 */
#define NDMUTEX			(0x30) /* Mutex Lock Register */
#define NDCMDMAT0		(0x34) /* Partition Command Match Register 0 */
#define NDCMDMAT1		(0x38) /* Partition Command Match Register 1 */
#define NDCMDMAT2		(0x3C) /* Partition Command Match Register 2 */
#define NDDB			(0x40) /* Data Buffer */
#define NDCB0			(0x48) /* Command Buffer0 */
#define NDCB1			(0x4C) /* Command Buffer1 */
#define NDCB2			(0x50) /* Command Buffer2 */
#define NDCB3			(0x50) /* Command Buffer3 */
#define NDARBCR			(0x5C) /* DFI Arbitration Control Register */
#define NDPTXCS0		(0x60) /* Partition Region Control Register 0 */
#define NDPTXCS1		(0x64) /* Partition Region Control Register 1 */
#define NDPTXCS2		(0x68) /* Partition Region Control Register 2 */
#define NDPTXCS3		(0x6C) /* Partition Region Control Register 3 */
#define NDPTXCS4		(0x70) /* Partition Region Control Register 4 */
#define NDPTXCS5		(0x74) /* Partition Region Control Register 5 */
#define NDPTXCS6		(0x78) /* Partition Region Control Register 6 */
#define NDPTXCS7		(0x7C) /* Partition Region Control Register 7 */

/* NDCR Register */
#define NDCR_SPARE_EN		(0x1 << 31)
#define NDCR_ECC_EN		(0x1 << 30)
#define NDCR_DMA_EN		(0x1 << 29)
#define NDCR_ND_RUN		(0x1 << 28)
#define NDCR_DWIDTH_C		(0x1 << 27)
#define NDCR_DWIDTH_M		(0x1 << 26)
#define NDCR_PAGE_SZ_MASK	(0x3 << 24)
#define NDCR_PAGE_SZ(x)		(((x) << 24) & NDCR_PAGE_SZ_MASK)
#define NDCR_SEQ_DIS		(0x1 << 23)
#define NDCR_ND_STOP		(0x1 << 22)
#define NDCR_FORCE_CSX		(0x1 << 21)
#define NDCR_CLR_PG_CNT		(0x1 << 20)
#define NDCR_STOP_ON_UNCOR	(0x1 << 19)
#define NDCR_RD_ID_CNT_MASK	(0x7 << 16)
#define NDCR_RD_ID_CNT(x)	(((x) << 16) & NDCR_RD_ID_CNT_MASK)

#define NDCR_RA_START		(0x1 << 15)
#define NDCR_PG_PER_BLK_MASK	(0x3 << 13)
#define NDCR_PG_PER_BLK(x)	(((x) << 13) & NDCR_PG_PER_BLK_MASK)
#define NDCR_ND_ARB_EN		(0x1 << 12)
#define NDCR_RDYM		(0x1 << 11)
#define NDCR_CS0_PAGEDM		(0x1 << 10)
#define NDCR_CS1_PAGEDM		(0x1 << 9)
#define NDCR_CS0_CMDDM		(0x1 << 8)
#define NDCR_CS1_CMDDM		(0x1 << 7)
#define NDCR_CS0_BBDM		(0x1 << 6)
#define NDCR_CS1_BBDM		(0x1 << 5)
#define NDCR_UNCERRM		(0x1 << 4)
#define NDCR_CORERRM		(0x1 << 3)
#define NDCR_WRDREQM		(0x1 << 2)
#define NDCR_RDDREQM		(0x1 << 1)
#define NDCR_WRCMDREQM		(0x1)
#define NDCR_INT_MASK		(0xFFF)

/* Data Controller Timing Paramter x Register For CSx */
#define NDTR0_tADL(c)		(min_t(uint32_t, (c), 31) << 27)
#define NDTR0_SELCNTR		(0x1 << 26)
#define NDTR0_RD_CNT_DEL_MASK	(0xF << 22)
#define NDTR0_RD_CNT_DEL(x)	(((x) << 22) & NDTR0_RD_CNT_DEL_MASK)
#define NDTR0_tCH(c)		(min_t(uint32_t, (c), 7) << 19)
#define NDTR0_tCS(c)		(min_t(uint32_t, (c), 7) << 16)
#define NDTR0_tWH(c)		(min_t(uint32_t, (c), 7) << 11)
#define NDTR0_tWP(c)		(min_t(uint32_t, (c), 7) << 8)
#define NDTR0_sel_NRE_EDGE	(0x1 << 7)
#define NDTR0_ETRP		(0x1 << 6)
#define NDTR0_tRH(c)		(min_t(uint32_t, (c), 7) << 3)
#define NDTR0_tRP(c)		(min_t(uint32_t, (c), 7) << 0)

#define NDTR1_tR(c)		(min_t(uint32_t, (c), 65535) << 16)
#define NDTR1_WAIT_MODE		(0x1 << 15)
#define NDTR1_PRESCALE		(0x1 << 14)
#define NDTR1_tRHW(c)		(min_t(uint32_t, (c), 3) << 8)
#define NDTR1_tWHR(c)		(min_t(uint32_t, (c), 15) << 4)
#define NDTR1_tAR(c)		(min_t(uint32_t, (c), 15) << 0)

/* NDSR Register */
#define NDSR_ERR_CNT_MASK	(0x1F << 16)
#define NDSR_ERR_CNT(x)		(((x) << 16) & NDSR_ERR_CNT_MASK)
#define NDSR_TRUSTVIO		(0x1 << 15)
#define NDSR_MASK		(0xFFFF)
#define NDSR_RDY		(0x1 << 12)		
#define NDSR_FLASH_RDY		(0x1 << 11)
#define NDSR_CS0_PAGED		(0x1 << 10)
#define NDSR_CS1_PAGED		(0x1 << 9)
#define NDSR_CS0_CMDD		(0x1 << 8)
#define NDSR_CS1_CMDD		(0x1 << 7)
#define NDSR_CS0_BBD		(0x1 << 6)
#define NDSR_CS1_BBD		(0x1 << 5)
#define NDSR_UNCERR		(0x1 << 4)
#define NDSR_CORERR		(0x1 << 3)
#define NDSR_WRDREQ		(0x1 << 2)
#define NDSR_RDDREQ		(0x1 << 1)
#define NDSR_WRCMDREQ		(0x1)

/* NDPCR Register */
#define NDPCR_PG_CNT_1_MASK	(0xFF << 16)
#define NDPCR_PG_CNT_1(x)	(((x) << 16) & NDPCR_PG_CNT_1_MASK)
#define NDPCR_PG_CNT_0_MASK	(0xFF)
#define NDPCR_PG_CNT_0(x)	((x) & NDPCR_PG_CNT_0_MASK)

/* READ Enable Return Delay Register */
#define NDREDEL_ND_DIN_SEL	(0x1 << 25)
#define NDREDEL_ND_DATA_D_MASK	(0x3 << 8)
#define NDREDEL_ND_DATA_DLY(x)	(((x) << 8) & NDREDEL_ND_DATA_D_MASK)
#define NDREDEL_ND_RECLK_D_MASK	(0xF << 4)
#define NDREDEL_ND_RECLK_DLY(x)	(((x) << 4) & NDREDEL_ND_RECLK_D_MASK)
#define NDREDEL_ND_RE_D_MASK	(0xF)
#define NDREDEL_ND_RE_DLY(x)	((x) & NDREDEL_ND_RE_D_MASK)

/* ECC Control Register */
#define NDECCCTRL_ECC_SPARE_MSK	(0xFF << 7)
#define NDECCCTRL_ECC_SPARE(x)	(((x) << 7) & NDECCCTRL_ECC_SPARE_MSK)
#define NDECCCTRL_ECC_THR_MSK	(0x3F << 1)
#define NDECCCTRL_ECC_THRESH(x)	(((x) << 1) & NDECCCTRL_ECC_THR_MSK)
#define NDECCCTRL_BCH_EN	(0x1)

/* Timer for ND_RnBx */
#define NDBZCNT_MASK		(0xFFFF)
#define NDBZCNT_ND_RNB_CNT1(x)	(((x & NDBZCNT_MASK) << 16)
#define NDBZCNT_ND_RNB_CNT0(x)	(x & NDBZCNT_MASK)

		/* NAND Controller MUTEX Lock Register */
#define NDMUTEX_MUTEX		(0x1)

		/* Partition Command Match Registers */
#define NDCMDMAT_VALIDCNT_MASK	(0x3)
#define NDCMDMAT_CMD_MASK	(0xFF)
#define NDCMDMAT_VALIDCNT	((x & NDCMDMAT_VALIDCNT_MASK) << 30)
#define NDCMDMAT_NAKEDDIS2	(0x1 << 29)
#define NDCMDMAT_ROWADD2	(0x1 << 28)
#define NDCMDMAT_CMD2(x)	((x & NDCMDMAT) << 20)
#define NDCMDMAT_NAKEDDIS1	(0x1 << 29)
#define NDCMDMAT_ROWADD1	(0x1 << 28)
#define NDCMDMAT_CMD1(x)	((x & NDCMDMAT) << 20)
#define NDCMDMAT_NAKEDDIS0	(0x1 << 29)
#define NDCMDMAT_ROWADD0	(0x1 << 28)
#define NDCMDMAT_CMD0(x)	((x & NDCMDMAT) << 20)

		/* NAND Controller Command Buffers */
#define NDCB0_CMD_XTYPE_MASK	(0x7 << 29)
#define NDCB0_CMD_XTYPE(x)	(((x) << 29) & NDCB0_CMD_XTYPE_MASK)
#define NDCB0_LEN_OVRD		(0x1 << 28)
#define NDCB0_RDY_BYP		(0x1 << 27)
#define NDCB0_ST_ROW_EN		(0x1 << 26)
#define NDCB0_AUTO_RS		(0x1 << 25)
#define NDCB0_CSEL		(0x1 << 24)
#define NDCB0_CMD_TYPE_MASK	(0x7 << 21)
#define NDCB0_CMD_TYPE(x)	(((x) << 21) & NDCB0_CMD_TYPE_MASK)
#define NDCB0_NC		(0x1 << 20)
#define NDCB0_DBC		(0x1 << 19)
#define NDCB0_ADDR_CYC_MASK	(0x7 << 16)
#define NDCB0_ADDR_CYC(x)	(((x) << 16) & NDCB0_ADDR_CYC_MASK)
#define NDCB0_CMD2_MASK		(0xff << 8)
#define NDCB0_CMD1_MASK		(0xff)

#define NDCB_MASK		(0xFF)
#define NDCB1_ADDR4(x)		((x & NDCB_MASK) << 24)
#define NDCB1_ADDR3(x)		((x & NDCB_MASK) << 16)
#define NDCB1_ADDR2(x)		((x & NDCB_MASK) << 8)
#define NDCB1_ADDR1(x)		(x & NDCB_MASK)

#define NDCB2_ST_MASK(x)	((x & NDCB_MASK) << 24)
#define NDCB2_ST_CMD(x)		((x & NDCB_MASK) << 16)
#define NDCB2_PAGE_COUNT(x)	((x & NDCB_MASK) << 8)
#define NDCB2_ADDR5(x)		(x & NDCB_MASK)

#define NDCB3_ADDR7(x)		((x & NDCB_MASK) << 24)
#define NDCB3_ADDR6(x)		((x & NDCB_MASK) << 16)
#define NDCB3_NDLENCNT_MASK	(0xFFFF)
#define NDCB3_NDLENCNT(x)	(x & NDCB3_NDLENCNT_MASK)

/* DFI Arbitration Control Register */
#define NDARBCR_MASK		(0xFFFF)
#define NDARBCR_ARB_CNT(x)	(x & NDARBCR_MASK)

/* Partition Region Control Registers for CSx */
#define NDPTXCS_VALID		(0x1 << 31)
#define NDPTXCS_LOCK		(0x1 << 30)
#define NDPTXCS_TRUSTED		(0x1 << 29)
#define NDPTXCS_BLOCKADD_MASK	(0xFFFFFF)
#define NDPTXCS_BLOCKADD(x)	((x) & NDPTXCS_BLOCKADD_MASK)

/* dma-able I/O address for the NAND data and commands */
#define NDCB0_DMA_ADDR		(0xd4283048)
#define NDDB_DMA_ADDR		(0xd4283040)

/* macros for registers read/write */
#define nand_writel(nand, off, val)	\
	__raw_writel((val), (nand)->mmio_base + (off))

#define nand_readl(nand, off)		\
	__raw_readl((nand)->mmio_base + (off))

enum {
	ERR_NONE	= 0,
	ERR_DMABUSERR	= 1,
	ERR_SENDCMD	= (1 << 1),
	ERR_DBERR	= (1 << 2),
	ERR_BBERR	= (1 << 3),
	ERR_CORERR	= (1 << 4),
	ERR_TRUSTVIO	= (1 << 5),
};

enum {
	STATE_CMD_WAIT_DONE	= 1,
	STATE_DATA_PROCESSING	= (1 << 1),
	STATE_DATA_DONE		= (1 << 2),
	STATE_PAGE_DONE		= (1 << 3),
	STATE_CMD_DONE		= (1 << 4),
	STATE_READY		= (1 << 5),
	STATE_CMD_PREPARED	= (1 << 6),
};

static struct nand_ecclayout hw_smallpage_ecclayout = {
	.eccbytes = 6,
	.eccpos = {8, 9, 10, 11, 12, 13 },
	.oobfree = { {2, 6} }
};

static struct nand_ecclayout hw_largepage_ecclayout = {
	.eccbytes = 24,
	.eccpos = {
		40, 41, 42, 43, 44, 45, 46, 47,
		48, 49, 50, 51, 52, 53, 54, 55,
		56, 57, 58, 59, 60, 61, 62, 63},
	.oobfree = { {2, 38} }
};

struct pxa3xx_nand_info {
	struct nand_chip	nand_chip;
	struct pxa3xx_nand	*nand_data;
	const struct pxa3xx_nand_flash *flash_info;

	size_t			data_size;	/* data size in FIFO */
	unsigned char		*data_buff;
	unsigned char		*oob_buff;
	unsigned int 		buf_start;
	unsigned int		buf_count;

	/* dma related */
	dma_addr_t 		data_buff_phys;
	dma_addr_t 		data_desc_addr;
	struct pxa_dma_desc	*data_desc;

	uint16_t		chip_select;
	uint16_t		data_column;
	uint16_t		oob_column;

	/* command poll */
	uint32_t		current_cmd_seqs;
	uint32_t		total_cmds;
	uint32_t		need_additional_addressing;
	uint32_t		need_wait_ready;
	uint32_t		ndcb0[CMD_POOL_SIZE];
	uint32_t		ndcb1;
	uint32_t		ndcb2;
	uint32_t		ndcb3[CMD_POOL_SIZE];

	uint32_t		reg_ndcr;
	uint32_t		timing0;
	uint32_t		timing1;
	uint32_t		col_addr_cycles;
	uint32_t		row_addr_cycles;

	/* calculated from pxa3xx_nand_flash data */
	size_t			oob_size;
	size_t			read_id_bytes;

	/* use HW ECC ? */
	/* 0:off, 1:Hammin ECC  2: BCH ECC */
	uint16_t		use_ecc;
};

struct pxa3xx_nand {
	struct clk		*clk;
	void __iomem		*mmio_base;
	struct nand_hw_control	controller;

	/* 2 chipselects supported for the moment */
	int			chip_select;
	int			enable_arbiter;
	int			RD_CNT_DEL;
	int			wait_mode;
	struct mtd_info		*mtd[NUM_CHIP_SELECT];

	/* relate to the command */
	unsigned int		state;
	unsigned int		command;
	unsigned int		is_write;
	unsigned int		is_ready;
	uint16_t		use_ecc;
	unsigned int		bad_count;
	unsigned int		errcode;
	struct completion 	cmd_complete;

	/* DMA information */
	int			use_dma;
	int			drcmr_dat;
	int			drcmr_cmd;
	int 			data_dma_ch;
	size_t			data_buff_size;
};

static inline int is_buf_blank(uint8_t *buf, size_t len)
{
	for (; len > 0; len--)
		if (*buf++ != 0xff)
			return 0;
	return 1;
}

static void disable_int(struct pxa3xx_nand *nand, uint32_t int_mask)
{
	uint32_t ndcr;

	ndcr = nand_readl(nand, NDCR);
	nand_writel(nand, NDCR, ndcr | int_mask);
}

static void nand_error_dump(struct pxa3xx_nand *nand)
{
	struct mtd_info *mtd = nand->mtd[nand->chip_select];
	struct pxa3xx_nand_info *info = mtd->priv;
	int i;

	if (no_error_dump)
		return;

	printk(KERN_ERR "NAND controller state wrong!!!\n");
	printk(KERN_ERR "state %x, current seqs %d, errcode %x, bad count %d\n",
			nand->state, info->current_cmd_seqs,
			nand->errcode, nand->bad_count);
	printk(KERN_ERR "Totally %d command for sending\n",
			info->total_cmds);
	for (i = 0; i < info->total_cmds; i ++)
		printk(KERN_ERR "NDCB0:%d: %x; NDCB3:%d, %x\n",
				i, info->ndcb0[i], i, info->ndcb3[i]);
	printk(KERN_ERR "NDCB1: %x; NDCB2 %x\n", info->ndcb1, info->ndcb2);

	printk(KERN_ERR "\nRegister DUMPing ##############\n");
	printk(KERN_ERR "NDCR %x\n"
			"NDSR %x\n"
			"NDCB0 %x\n"
			"NDCB1 %x\n"
			"NDCB2 %x\n"
			"NDTR0CS0 %x\n"
			"NDTR1CS0 %x\n"
			"NDBBR0 %x\n"
			"NDBBR1 %x\n"
			"NDREDEL %x\n"
			"NDECCCTRL %x\n"
			"NDBZCNT %x\n\n",
			nand_readl(nand, NDCR),
			nand_readl(nand, NDSR),
			nand_readl(nand, NDCB0),
			nand_readl(nand, NDCB1),
			nand_readl(nand, NDCB2),
			nand_readl(nand, NDTR0CS0),
			nand_readl(nand, NDTR1CS0),
			nand_readl(nand, NDBBR0),
			nand_readl(nand, NDBBR1),
			nand_readl(nand, NDREDEL),
			nand_readl(nand, NDECCCTRL),
			nand_readl(nand, NDBZCNT));
}

/*
 * This function shows the real timing when NAND controller
 * send signal to the NAND chip.
 */
static void show_real_timing(uint32_t ndtr0, uint32_t ndtr1, unsigned long nand_clk)
{
	uint32_t rtADL, rtCH, rtCS, rtWH, rtWP, rtRH, rtRP;
	uint32_t rtR, rtRHW, rtWHR, rtAR, tmp;

	rtCH = ((ndtr0 >> 19) & 0x7) + 1;
	rtCS = ((ndtr0 >> 16) & 0x7) + 1;
	rtWH = ((ndtr0 >> 11) & 0x7) + 1;
	rtWP = ((ndtr0 >> 8) & 0x7) + 1;
	rtADL= (ndtr0 >> 27) & 0x1f;
	rtRH = ((ndtr0 >> 3) & 0x7) + 1;
	rtRP = (ndtr0 & NDTR0_ETRP) ? ((0x8 | (ndtr0 & 0x7)) + 1)
			: ((ndtr0 & 0x7) + 1);
	rtRHW = (ndtr1 >> 8) & 0x3;
	rtWHR = (ndtr1 >> 4) & 0xf;
	rtAR = ndtr1 & 0xf;

	if (rtADL != 0)
		rtADL -= 3 + rtWP;
	rtR = (ndtr1 >> 16) & 0xffff;
	if (ndtr1 & NDTR1_PRESCALE)
		rtR *= 16;

	rtR += rtCH + 2;
	switch(rtRHW) {
	case 0:
		rtRHW = 0;
		break;
	case 1:
		rtRHW = 16;
		break;
	case 2:
		rtRHW = 32;
		break;
	case 3:
		rtRHW = 48;
		break;
	}

	/*
	 * TWHR delay=max(tAR, max(0, tWHR-max(tWH, tCH)))
	 * TAR delay=max(tAR, max(0, tWHR-max(tWH, tCH))) + 2
	 */
	if (rtWH > rtCH)
		tmp = rtWH - 1;
	else
		tmp = rtCH - 1;
	if (rtWHR < tmp)
		rtWHR = rtAR;
	else {
		if (rtAR > (rtWHR - tmp))
			rtWHR = rtAR;
		else
			rtWHR = rtWHR - tmp;
	}
	rtAR = rtWHR + 2;
	printk("Shows real timing(ns):\n");
	if (ndtr0 & NDTR0_SELCNTR)
		printk("NDTR0 SELCNTR is set\n");
	else
		printk("NDTR0 SELCNTR is not set\n");
	if (ndtr0 & NDTR0_RD_CNT_DEL_MASK)
		printk("Read Strobe delay is %d\n",
				(ndtr0 & NDTR0_RD_CNT_DEL_MASK) >> 22);
	else
		printk("No Read Stobe delay\n");
	if (ndtr0 & NDTR0_sel_NRE_EDGE)
		printk("Controller is using rising edge to detect RE\n");
	else
		printk("Controller is using falling edge to detect RE\n");

	if (ndtr1 & NDTR1_WAIT_MODE)
		printk("NDTR1 wait mode is set\n");
	else
		printk("NDTR1 wait mode is not set\n");

	printk("TADL is %ld TCH is %ld TCS is %ld TWH is %ld TWP is %ld TRH is %ld "
		"TRP is %ld TR is %ld TRHW is %ld TWHR is %ld TAR is %ld\n",
		cycle2ns(rtADL, nand_clk), cycle2ns(rtCH, nand_clk),
		cycle2ns(rtCS, nand_clk), cycle2ns(rtWH, nand_clk),
		cycle2ns(rtWP, nand_clk), cycle2ns(rtRH, nand_clk),
		cycle2ns(rtRP, nand_clk), cycle2ns(rtR, nand_clk),
		cycle2ns(rtRHW, nand_clk), cycle2ns(rtWHR, nand_clk),
		cycle2ns(rtAR, nand_clk));
}

static void pxa3xx_nand_set_timing(struct pxa3xx_nand_info *info,
		const struct pxa3xx_nand_timing *t, int show_timing)
{
	struct pxa3xx_nand *nand = info->nand_data;
	unsigned long nand_clk = clk_get_rate(nand->clk);
	uint32_t ndtr0, ndtr1, tRP, tR, tRHW, tADL;

	if (!info->timing0 && !info->timing1) {
		ndtr0 = ndtr1 = 0;
		tRP = ns2cycle(t->tRP, nand_clk);
		tRP = (tRP > 0xf) ? 0xf : tRP;
		if (tRP > 0x7) {
			ndtr0 |= NDTR0_ETRP;
			tRP -= 0x7;
		}
		tR = ns2cycle(t->tR, nand_clk);
		if (tR > 0xffff) {
			ndtr1 |= NDTR1_PRESCALE;
			tR /= 16;
		}
		if (t->tRHW > 0) {
			tRHW = ns2cycle(t->tRHW, nand_clk);
			if (tRHW < 16)
				tRHW = 1;
			else {
				if (tRHW < 32)
					tRHW = 2;
				else
					tRHW = 3;
			}
		}
		else
			tRHW = 0;
		tADL = (t->tADL > 0) ? ns2cycle(t->tADL, nand_clk) : 0;

		if (nand->RD_CNT_DEL > 0)
			ndtr0 |= NDTR0_SELCNTR
				| (NDTR0_RD_CNT_DEL(nand->RD_CNT_DEL - 1));

		ndtr0 |= NDTR0_tADL(tADL)
			 | NDTR0_tCH(ns2cycle(t->tCH, nand_clk))
			 | NDTR0_tCS(ns2cycle(t->tCS, nand_clk))
			 | NDTR0_tWH(ns2cycle(t->tWH, nand_clk))
			 | NDTR0_tWP(ns2cycle(t->tWP, nand_clk))
			 | NDTR0_tRH(ns2cycle(t->tRH, nand_clk))
			 | NDTR0_tRP(tRP)
			 | NDTR0_SELCNTR;

		if (nand->wait_mode)
			ndtr1 |= NDTR1_WAIT_MODE;

		ndtr1 |= NDTR1_tR(tR)
			 | NDTR1_tRHW(tRHW)
			 | NDTR1_tWHR(ns2cycle(t->tWHR, nand_clk))
			 | NDTR1_tAR(ns2cycle(t->tAR, nand_clk));

		info->timing0 = ndtr0;
		info->timing1 = ndtr1;
		if (show_timing)
			show_real_timing(ndtr0, ndtr1, nand_clk);
	}

	nand_writel(nand, NDTR0CS0, info->timing0);
	nand_writel(nand, NDTR1CS0, info->timing1);
	nand_writel(nand, NDREDEL, 0x0);
}

static void pxa3xx_set_datasize(struct pxa3xx_nand_info *info, int oob_enable)
{
	const struct pxa3xx_nand_flash *flash_info = info->flash_info;

	if (likely(flash_info->page_size >= PAGE_CHUNK_SIZE)) {
		info->data_size = 2048;
		if (!oob_enable) {
			info->oob_size = 0;
			return;
		}

		switch (info->use_ecc) {
			case ECC_HAMMIN:
				info->oob_size = 40;
				break;
			case ECC_BCH:
				info->oob_size = 32;
				break;
			default:
				info->oob_size = 64;
				break;
		}
	}
	else {
		info->data_size = 512;
		if (!oob_enable) {
			info->oob_size = 0;
			return;
		}

		switch (info->use_ecc) {
			case ECC_HAMMIN:
				info->oob_size = 8;
				break;
			case ECC_BCH:
				printk("Don't support BCH on small"
					       " page device!!!\n");
				break;
			default:
				info->oob_size = 16;
				break;
		}
	}
}

/* NOTE: it is a must to set ND_RUN firstly, then write 
 * command buffer, otherwise, it does not work
 */
static void pxa3xx_nand_start(struct pxa3xx_nand_info *info)
{
	uint32_t ndcr, ndeccctrl;
	struct pxa3xx_nand *nand = info->nand_data;

	ndcr = info->reg_ndcr;
	ndeccctrl = 0;

	switch (info->use_ecc) {
	case ECC_BCH:
		ndeccctrl |= NDECCCTRL_BCH_EN;
		ndeccctrl |= NDECCCTRL_ECC_THRESH(BCH_THRESHOLD);
	case ECC_HAMMIN:
		ndcr |= NDCR_ECC_EN;
		break;
	default:
		break;
	}

	ndcr |= nand->use_dma ? NDCR_DMA_EN : NDCR_STOP_ON_UNCOR;
	ndcr |= NDCR_ND_RUN;

	DBG_NAND(printk("@@@ndcr set: %x, ndeccctrl set %x\n",
				ndcr, ndeccctrl));
	/* clear status bits and run */
	nand_writel(nand, NDCR, 0);
	nand_writel(nand, NDECCCTRL, ndeccctrl);
	nand_writel(nand, NDSR, NDSR_MASK);
	nand_writel(nand, NDCR, ndcr);
}

static void pxa3xx_nand_stop(struct pxa3xx_nand* nand)
{
	uint32_t ndcr;
	int timeout = NAND_STOP_DELAY;

	/* wait RUN bit in NDCR become 0 */
	do {
		/* clear status bits */
		nand_writel(nand, NDSR, NDSR_MASK);
		ndcr = nand_readl(nand, NDCR);
		udelay(1);
	} while ((ndcr & NDCR_ND_RUN) && (timeout -- > 0));

	if (timeout <= 0) {
		if (!no_error_dump)
			printk(KERN_ERR "NAND controller unable to stop,"
				"please reconfigure your timing!!!\n");
		nand_error_dump(nand);
		ndcr &= ~(NDCR_ND_RUN);
		nand_writel(nand, NDCR, ndcr);
	}
}

static void start_data_dma(struct pxa3xx_nand *nand, int dir_out, int cmd_seqs)
{
	struct mtd_info *mtd = nand->mtd[nand->chip_select];
	struct pxa3xx_nand_info *info = mtd->priv;
	struct pxa_dma_desc *desc = info->data_desc;
	int dma_len = ALIGN(info->ndcb3[cmd_seqs] + info->oob_size, 32);

	desc->ddadr = DDADR_STOP;
	desc->dcmd = DCMD_ENDIRQEN | DCMD_WIDTH4
		     | DCMD_BURST32 | dma_len;

	if (dir_out) {
		desc->dsadr = info->data_buff_phys + info->data_column;
		desc->dtadr = NDDB_DMA_ADDR;

		desc->dcmd |= DCMD_INCSRCADDR
			      | DCMD_FLOWTRG;
	} else {
		desc->dtadr = info->data_buff_phys + info->data_column;
		desc->dsadr = NDDB_DMA_ADDR;

		desc->dcmd |= DCMD_INCTRGADDR
			      | DCMD_FLOWSRC;
	}

	DBG_NAND(printk("DMA START:DMA dcmd %x, dsadr %x, dtadr %x, len %x\n",
				desc->dcmd, desc->dsadr, desc->dtadr, dma_len));
	DRCMR(nand->drcmr_dat) = DRCMR_MAPVLD | nand->data_dma_ch;
	DDADR(nand->data_dma_ch) = info->data_desc_addr;
	DCSR(nand->data_dma_ch) |= DCSR_RUN;

	return;
}

static void handle_data_pio(struct pxa3xx_nand *nand, int cmd_seqs)
{
	void *mmio_base = nand->mmio_base;
	struct mtd_info *mtd = nand->mtd[nand->chip_select];
	struct pxa3xx_nand_info *info = mtd->priv;

	DBG_NAND(printk("data col %x, size %x, oob col %x, size %x\n",
				info->data_column, info->ndcb3[cmd_seqs],
				info->oob_column, info->oob_size));
	if (nand->is_write) {
		if (info->oob_size > 0) {
			/* write data part */
			__raw_writesl(mmio_base + NDDB,				\
					info->data_buff	+ info->data_column,	\
					info->ndcb3[cmd_seqs] >> 2);

			/* write oob part */
			__raw_writesl(mmio_base + NDDB,				\
					info->oob_buff + info->oob_column,	\
					info->oob_size >> 2);
		}
		else
			__raw_writesl(mmio_base + NDDB,				\
					info->data_buff	+ info->data_column,	\
					info->ndcb3[cmd_seqs] >> 2);
	}
	else {
		if (info->oob_size > 0) {
			/* read data part */
			__raw_readsl(mmio_base + NDDB,				\
					info->data_buff	+ info->data_column,	\
					info->ndcb3[cmd_seqs] >> 2);

			/* read oob part */
			__raw_readsl(mmio_base + NDDB,				\
					info->oob_buff + info->oob_column,	\
					info->oob_size >> 2);
		}
		else
			__raw_readsl(mmio_base + NDDB,				\
					info->data_buff	+ info->data_column,	\
					info->ndcb3[cmd_seqs] >> 2);

	}

	info->data_column += info->ndcb3[cmd_seqs];
	info->oob_column += info->oob_size;
}

static void pxa3xx_nand_data_dma_irq(int channel, void *data)
{
	struct pxa3xx_nand *nand= data;
	struct mtd_info *mtd = nand->mtd[nand->chip_select];
	struct pxa3xx_nand_info *info = mtd->priv;
	struct pxa3xx_nand_flash *flash_info = info->flash_info;
	uint32_t dcsr, ndcr;
	int i, tmp;

	dcsr = DCSR(channel);
	DCSR(channel) = dcsr;

	DBG_NAND(printk("DMA IRQ: dcsr %x\n", dcsr));
	if (dcsr & DCSR_BUSERR) {
		nand->errcode |= ERR_DMABUSERR;
	}

	/*
	 * Here we need a workaround as flash layout is not our need
	 * Should notice this is only applied to 4K page size NAND
	 * In the first step, copy the first oob to the last part
	 * then kick the second chunk data next to the first data.
	 * After this, the two oob chunk would be at wrong order,
	 * that is also why we need to swap those two parts.
	 */
	if (flash_info->page_size > PAGE_CHUNK_SIZE && info->oob_size > 0) {
		if (info->data_column == 0)
			memcpy(info->oob_buff + info->oob_size, info->data_buff
				+ info->oob_size, info->oob_size);
		else
			for (i = 0; i < info->oob_size; i ++) {
				tmp = info->oob_buff[i];
				info->oob_buff[i] = info->oob_buff[info->oob_size + i];
				info->oob_buff[info->oob_size + i] = tmp;
			}
	}

	info->data_column += info->ndcb3[info->current_cmd_seqs - 1];
	ndcr = nand_readl(nand, NDCR);
	ndcr &= ~NDCR_INT_MASK;
	nand_writel(nand, NDCR, ndcr);
	nand_writel(nand, NDSR, NDSR_WRDREQ | NDSR_RDDREQ);
}

static irqreturn_t pxa3xx_nand_irq(int irq, void *devid)
{
	struct pxa3xx_nand *nand = devid;
	struct pxa3xx_nand_info *info;
	struct mtd_info *mtd;
	unsigned int status;
	int chip_select, cmd_done, ready, page_done, badblock_detect;
	int cmd_seqs, ndcb1, ndcb2, ndcr, is_completed = 0;

	chip_select 	= nand->chip_select;
	ready		= (chip_select) ? NDSR_RDY : NDSR_FLASH_RDY;
	cmd_done	= (chip_select) ? NDSR_CS1_CMDD : NDSR_CS0_CMDD;
	page_done	= (chip_select) ? NDSR_CS1_PAGED : NDSR_CS0_PAGED;
	badblock_detect	= (chip_select) ? NDSR_CS1_BBD : NDSR_CS0_BBD;
	mtd		= nand->mtd[chip_select];
	info		= (struct pxa3xx_nand_info *)(mtd->priv);
	cmd_seqs	= info->current_cmd_seqs;

	status = nand_readl(nand, NDSR);
	nand->bad_count = (status & NDSR_ERR_CNT_MASK) >> 16;
	DBG_NAND(if (status != 0)
		printk("\t\tcmd seqs %d, status %x\n", cmd_seqs, status));
	if (no_error_dump)
		goto ERR_IRQ_EXIT;

	if (status & NDSR_TRUSTVIO)
		nand->errcode |= ERR_TRUSTVIO;

	if (status & NDSR_CORERR)
		nand->errcode |= ERR_CORERR;

	if (status & NDSR_UNCERR)
		nand->errcode |= ERR_DBERR;

	if (status & badblock_detect)
		nand->errcode |= ERR_BBERR;

	if ((status & NDSR_WRDREQ) || (status & NDSR_RDDREQ)) {

		nand->state |= STATE_DATA_PROCESSING;
		/* whether use dma to transfer data */
		if (nand->use_dma) {
			ndcr = nand_readl(nand, NDCR);
			ndcr |= NDCR_INT_MASK;
			nand_writel(nand, NDCR, ndcr);
			start_data_dma(nand, nand->is_write, cmd_seqs - 1);
			goto NORMAL_IRQ_EXIT;
		}
		else 
			handle_data_pio(nand, cmd_seqs - 1);

		nand->state |= STATE_DATA_DONE;
	}

	if (status & cmd_done) {
		nand->state |= STATE_CMD_DONE;

		/* complete the command cycle when all command
		 * done, and don't wait for ready signal
		 */
		if ((cmd_seqs == info->total_cmds)	\
				&& !(cmd_seqs == info->need_wait_ready)) {

			is_completed = 1;
		}
	}

	if (status & ready) {
		nand->state |= STATE_READY;
		/* 
		 * wait for the ready signal, 
		 * then leavl the command cycle
		 */
		if ((cmd_seqs == info->total_cmds) \
				&& (cmd_seqs == info->need_wait_ready)) {

			is_completed = 1;
		}

		nand->is_ready = 1;
	}

	if (status & page_done)
		nand->state |= STATE_PAGE_DONE;

	if (nand->errcode != ERR_NONE && !nand->use_dma) {
		is_completed = 1;
		goto ERR_IRQ_EXIT;
	}

	if (status & NDSR_WRCMDREQ) {
		nand_writel(nand, NDSR, NDSR_WRCMDREQ);
		status &= ~NDSR_WRCMDREQ;
		if (cmd_seqs < info->total_cmds) {

			info->current_cmd_seqs ++;
			if (cmd_seqs == 0) {
				ndcb1 = info->ndcb1;
				ndcb2 = info->ndcb2;
			}
			else {
				ndcb1 = 0;
				ndcb2 = 0;
			}

			nand->state = STATE_CMD_WAIT_DONE;
			nand_writel(nand, NDCB0, info->ndcb0[cmd_seqs]);
			nand_writel(nand, NDCB0, ndcb1);
			nand_writel(nand, NDCB0, ndcb2);
			if (info->need_additional_addressing) {
				nand_writel(nand, NDCB0, info->ndcb3[cmd_seqs]);
				DBG_NAND(printk("\tndcb0 %x ndcb1 %x, "
							"ndcb2 %x, ndcb3 %x\n",
							info->ndcb0[cmd_seqs],
							ndcb1, ndcb2, info->ndcb3[cmd_seqs]));
			}
			else {
				DBG_NAND(printk("\tndcb0 %x ndcb1 %x ndcb2 %x\n",
							info->ndcb0[cmd_seqs],
							ndcb1, ndcb2));
			}
		}
		else
			is_completed = 1;

	}

ERR_IRQ_EXIT:
	/* clear NDSR to let the controller exit the IRQ */
	nand_writel(nand, NDSR, status);
	if (is_completed)
		complete(&nand->cmd_complete);

NORMAL_IRQ_EXIT:
	return IRQ_HANDLED;
}

static int pxa3xx_nand_dev_ready(struct mtd_info *mtd)
{
	struct pxa3xx_nand_info *info = mtd->priv;
	struct pxa3xx_nand *nand = info->nand_data;
	int ready_mask = (nand->chip_select)
			 ? NDSR_RDY : NDSR_FLASH_RDY;

	return (nand_readl(nand, NDSR) & ready_mask) ? 1 : 0;
}

static int prepare_command_pool(struct pxa3xx_nand *nand, int command,
		uint16_t column, int page_addr)
{
	uint16_t cmd;
	int addr_cycle, exec_cmd, ndcb0, ndcb3 = 0, i, chunks = 0, ecc_strength;
	struct mtd_info *mtd = nand->mtd[nand->chip_select];
	struct pxa3xx_nand_info *info = mtd->priv;
	struct nand_chip *chip = mtd->priv;
	const struct pxa3xx_nand_flash *flash_info = info->flash_info;

	ndcb0 = (nand->chip_select) ? NDCB0_CSEL : 0;
	addr_cycle = 0;
	exec_cmd = 1;

	/* reset data and oob column point to handle data */
	info->data_column	= 0;
	info->oob_column	= 0;
	info->buf_start		= 0;
	info->buf_count		= 0;
	info->current_cmd_seqs	= 0;
	info->need_wait_ready	= -1;
	info->oob_size		= 0;
	info->use_ecc		= ECC_NONE;

	nand->state		= 0;
	nand->is_write		= 0;
	nand->is_ready		= 0;
	nand->errcode		= ERR_NONE;
	nand->bad_count		= 0;
	nand->command		= command;

	switch (command) {
	case NAND_CMD_READ0:
	case NAND_CMD_PAGEPROG:
		if (chip->ecc.mode == NAND_ECC_HW)
			info->use_ecc = flash_info->ecc_type;

	case NAND_CMD_READOOB:
		ecc_strength = (command == NAND_CMD_READOOB) ? 1
					: flash_info->ecc_strength;
		if (ecc_strength > 1) {
			info->reg_ndcr &= ~NDCR_SPARE_EN;
			ndcb0 |= NDCB0_LEN_OVRD;
		}
		else
			info->reg_ndcr |= NDCR_SPARE_EN;

		pxa3xx_set_datasize(info, info->reg_ndcr & NDCR_SPARE_EN);
		chunks = flash_info->page_size / info->data_size;
		chunks = chunks * ecc_strength;
		ndcb3 = info->data_size / ecc_strength;
		break;
	case NAND_CMD_SEQIN:
		exec_cmd = 0;
		break;
	default:
		info->ndcb1 = 0;
		info->ndcb2 = 0;
		break;
	}

	/* clear the command buffer */
	for (i = 0; i < CMD_POOL_SIZE; i ++)
		info->ndcb0[i] = ndcb0;
	addr_cycle = NDCB0_ADDR_CYC(info->row_addr_cycles
			+ info->col_addr_cycles);

	if ((ndcb0 & NDCB0_LEN_OVRD)
		|| ((info->row_addr_cycles + info->col_addr_cycles) > 5)) {
		info->need_additional_addressing = 1;
	}
	else
		info->need_additional_addressing = 0;

	switch (command) {
	case NAND_CMD_READOOB:
	case NAND_CMD_READ0:

		cmd  = flash_info->cmdset->read1;

		if (command == NAND_CMD_READOOB) {
			if (!(info->reg_ndcr & NDCR_SPARE_EN))
				return 0;

			info->buf_start = mtd->writesize + column;
			for (i = 1; i <= chunks; i ++)
				info->ndcb3[i] = info->data_size;
		}
		else {
			info->ndcb3[0] = info->data_size;
			for (i = 1; i <= chunks; i ++)
				info->ndcb3[i] = ndcb3;

			info->buf_start = column;
		}

		if (unlikely(flash_info->page_size < PAGE_CHUNK_SIZE)) {
			info->total_cmds = 1;

			info->ndcb0[0] |= NDCB0_CMD_TYPE(0)
					| addr_cycle
					| cmd;
		}
		else {
			info->total_cmds = chunks + 1;

			info->ndcb0[0] |= NDCB0_CMD_XTYPE(0x6)
					| NDCB0_CMD_TYPE(0)
					| NDCB0_DBC
					| NDCB0_NC
					| addr_cycle
					| cmd;

			info->ndcb0[1] |= NDCB0_CMD_XTYPE(0x5)
					| NDCB0_NC
					| addr_cycle;

			for (i = 2; i <= chunks; i ++)
				info->ndcb0[i] = info->ndcb0[1];

			info->ndcb0[chunks] &= ~NDCB0_NC;
		}

	case NAND_CMD_SEQIN:
		/* small page addr setting */
		if (unlikely(flash_info->page_size < PAGE_CHUNK_SIZE)) {
			info->ndcb1 = ((page_addr & 0xFFFFFF) << 8)
				      | (column & 0xFF);

			info->ndcb2 = 0;
		}
		else {
			info->ndcb1 = ((page_addr & 0xFFFF) << 16)
				      | (column & 0xFFFF);

			if (page_addr & 0xFF0000)
				info->ndcb2 = (page_addr & 0xFF0000) >> 16;
			else
				info->ndcb2 = 0;
		}

		info->buf_count = mtd->writesize + mtd->oobsize;
		memset(info->data_buff, 0xFF, info->buf_count);

		break;

	case NAND_CMD_PAGEPROG:
		if (is_buf_blank(info->data_buff,
					(mtd->writesize + mtd->oobsize))) {
			exec_cmd = 0;
			break;
		}

		cmd = flash_info->cmdset->program;

		nand->is_write = 1;
		info->need_wait_ready = chunks + 1;

		if (unlikely(flash_info->page_size < PAGE_CHUNK_SIZE)) {
			info->total_cmds = 1;
			info->ndcb0[0] |= NDCB0_CMD_TYPE(0x1)
					| NDCB0_AUTO_RS
					| NDCB0_ST_ROW_EN
					| NDCB0_DBC
					| cmd
					| addr_cycle;
		}
		else {
			info->total_cmds = chunks + 1;
			info->ndcb0[0] |= NDCB0_CMD_XTYPE(0x4)
					| NDCB0_CMD_TYPE(0x1)
					| NDCB0_NC
					| NDCB0_AUTO_RS
					| (cmd & NDCB0_CMD1_MASK)
					| addr_cycle;

			for (i = 1; i < chunks; i ++)
				info->ndcb0[i] |= NDCB0_CMD_XTYPE(0x5)
						| NDCB0_NC
						| NDCB0_AUTO_RS
						| NDCB0_CMD_TYPE(0x1)
						| addr_cycle;

			info->ndcb0[chunks] |= NDCB0_CMD_XTYPE(0x3)
						| NDCB0_CMD_TYPE(0x1)
						| NDCB0_ST_ROW_EN
						| NDCB0_DBC
						| (cmd & NDCB0_CMD2_MASK)
						| NDCB0_CMD1_MASK
						| addr_cycle;
		}

		if (ndcb3) {
			for (i = 0; i < chunks; i ++)
				info->ndcb3[i] = ndcb3;
			info->ndcb3[i] = info->data_size;
		}

		break;

	case NAND_CMD_READID:
		cmd = flash_info->cmdset->read_id;
		nand->use_dma = 0;
		info->total_cmds = 1;
		info->buf_count = info->read_id_bytes;

		info->ndcb0[0] |= NDCB0_CMD_TYPE(3)
				  | NDCB0_ADDR_CYC(1)
				  | cmd;
		info->ndcb3[0] = 8;

		break;

	case NAND_CMD_STATUS:
		cmd = flash_info->cmdset->read_status;
		nand->use_dma = 0;
		info->total_cmds = 1;
		info->buf_count = 1;
		info->ndcb0[0] |= NDCB0_CMD_TYPE(4)
				  | NDCB0_ADDR_CYC(1)
				  | cmd;
		info->ndcb3[0] = 8;

		break;

	case NAND_CMD_ERASE1:
		cmd = flash_info->cmdset->erase;
		info->total_cmds = 1;
		info->ndcb0[0] |= NDCB0_CMD_TYPE(2)
			       | NDCB0_AUTO_RS
			       | NDCB0_ADDR_CYC(3)
			       | NDCB0_DBC
			       | cmd;
		info->ndcb1 = page_addr;
		info->ndcb2 = 0;

		break;
	case NAND_CMD_RESET:
		cmd = flash_info->cmdset->reset;
		info->total_cmds = 1;
		info->ndcb0[0] |= NDCB0_CMD_TYPE(5)
			       | cmd;

		break;

	case NAND_CMD_ERASE2:
		exec_cmd = 0;
		break;

	default:
		exec_cmd = 0;
		printk(KERN_ERR "non-supported command.\n");
		break;
	}

	nand->use_ecc = info->use_ecc;
	return exec_cmd;
}

static void pxa3xx_nand_cmdfunc(struct mtd_info *mtd, unsigned command,
		int column, int page_addr)
{
	struct pxa3xx_nand_info *info = mtd->priv;
	struct pxa3xx_nand *nand = info->nand_data;
	const struct pxa3xx_nand_flash *flash_info = info->flash_info;
	int ret, exec_cmd, use_dma;
	loff_t addr;
#ifdef CONFIG_PXA3XX_BBM
	struct pxa3xx_bbm *pxa3xx_bbm = mtd->bbm;

	DBG_NAND(printk("command %x, page %x, ", command, page_addr););
	if (pxa3xx_bbm && (command == NAND_CMD_READOOB
			|| command == NAND_CMD_READ0
			|| command == NAND_CMD_SEQIN
			|| command == NAND_CMD_ERASE1)) {

		addr = (loff_t)page_addr << mtd->writesize_shift;
		addr = pxa3xx_bbm->search(mtd, addr);
		page_addr = addr >> mtd->writesize_shift;
	}
	DBG_NAND(printk("post page %x\n", page_addr));
#else
	DBG_NAND(printk("command %x, page %x\n", command, page_addr););
#endif

	set_dvfm_constraint();

	/* reset timing */
	if (nand->chip_select != info->chip_select) {
		pxa3xx_nand_set_timing(info, flash_info->timing, 0);
		nand->chip_select = info->chip_select;
	}

	/* if this is a x16 device ,then convert the input 
	 * "byte" address into a "word" address appropriate
	 * for indexing a word-oriented device
	 */
	if (flash_info->flash_width == 16)
		column /= 2;

	use_dma = nand->use_dma;
	exec_cmd = prepare_command_pool(nand, command, column, page_addr);
	if (exec_cmd) {
		/* prepare for the first command */
		init_completion(&nand->cmd_complete);

		nand->state |= STATE_CMD_PREPARED;
		pxa3xx_nand_start(info);

		ret = wait_for_completion_timeout(&nand->cmd_complete,
				CHIP_DELAY_TIMEOUT);
		if (!ret) {
			printk(KERN_ERR "Wait time out!!!\n");
			nand_error_dump(nand);
			nand->errcode |= ERR_SENDCMD;
		}

		/* Stop State Machine for next command cycle */
		pxa3xx_nand_stop(nand);
		disable_int(nand, NDCR_INT_MASK);
		nand->state &= ~STATE_CMD_PREPARED;
	}

	nand->use_dma = use_dma;
	unset_dvfm_constraint();
}

static uint8_t pxa3xx_nand_read_byte(struct mtd_info *mtd)
{
	struct pxa3xx_nand_info *info = mtd->priv;
	char retval = 0xFF;

	if (info->buf_start < info->buf_count)
		/* Has just send a new command? */
		retval = info->data_buff[info->buf_start++];

	return retval;
}

static u16 pxa3xx_nand_read_word(struct mtd_info *mtd)
{
	struct pxa3xx_nand_info *info = mtd->priv;
	u16 retval = 0xFFFF;

	if (!(info->buf_start & 0x01)				    \
			&& info->buf_start < info->buf_count) {

		retval = *((u16 *)(info->data_buff+info->buf_start));
		info->buf_start += 2;
	}
	return retval;
}

static void pxa3xx_nand_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	struct pxa3xx_nand_info *info = mtd->priv;
	int real_len = min_t(size_t, len, info->buf_count - info->buf_start);

	memcpy(buf, info->data_buff + info->buf_start, real_len);
	info->buf_start += real_len;
}

static void pxa3xx_nand_write_buf(struct mtd_info *mtd,
		const uint8_t *buf, int len)
{
	struct pxa3xx_nand_info *info = mtd->priv;
	int real_len = min_t(size_t, len, info->buf_count - info->buf_start);

	memcpy(info->data_buff + info->buf_start, buf, real_len);
	info->buf_start += real_len;
}

static int pxa3xx_nand_verify_buf(struct mtd_info *mtd,
		const uint8_t *buf, int len)
{
	return 0;
}

static void pxa3xx_nand_select_chip(struct mtd_info *mtd, int chip)
{
	return;
}

/* Error handling expose to MTD level */
static int pxa3xx_nand_waitfunc(struct mtd_info *mtd, struct nand_chip *this)
{
	struct pxa3xx_nand_info *info = mtd->priv;
	struct pxa3xx_nand *nand = info->nand_data;

	if (nand->errcode & ERR_TRUSTVIO) {
		printk(KERN_ERR "Trust violation!!!\n");
		nand_error_dump(nand);
		return NAND_STATUS_FAIL;
	}

	if (nand->errcode & (ERR_BBERR | ERR_SENDCMD))
		return NAND_STATUS_FAIL;
	else
		return 0;
}

static int pxa3xx_nand_read_page_hwecc(struct mtd_info *mtd,
			struct nand_chip *chip,	uint8_t *buf)
{
	struct pxa3xx_nand_info *info = mtd->priv;
	struct pxa3xx_nand *nand = info->nand_data;

	chip->read_buf(mtd, buf, mtd->writesize);
	chip->read_buf(mtd, chip->oob_poi, mtd->oobsize);

	if (nand->errcode & ERR_CORERR) {
		DBG_NAND(printk("###correctable error detected\n"););
		switch (nand->use_ecc) {
		case ECC_BCH:
			if (nand->bad_count > BCH_THRESHOLD)
				mtd->ecc_stats.corrected +=
					(nand->bad_count - BCH_THRESHOLD);
			break;

		case ECC_HAMMIN:
			mtd->ecc_stats.corrected ++;
			break;

		case ECC_NONE:
		default:
			break;
		}
	}
	else if (nand->errcode & ERR_DBERR) {
		int buf_blank;

		buf_blank = is_buf_blank(buf, mtd->writesize);

		if (!buf_blank) {
			DBG_NAND(printk("###uncorrectable error!!!\n"));
			mtd->ecc_stats.failed++;
		}
	}

	return 0;
}

static void pxa3xx_nand_write_page_hwecc(struct mtd_info *mtd,
			struct nand_chip *chip, const uint8_t *buf)
{
	chip->write_buf(mtd, buf, mtd->writesize);
	chip->write_buf(mtd, chip->oob_poi, mtd->oobsize);
}

static int pxa3xx_nand_config_flash(struct pxa3xx_nand_info *info,
		const struct pxa3xx_nand_flash *f, int show_timing)
{
	/* enable all interrupts */
	uint32_t ndcr = 0;
	struct pxa3xx_nand *nand = info->nand_data;

	/* calculate flash information */
	info->oob_buff = info->data_buff + f->page_size;
	info->read_id_bytes = (f->page_size >= 2048) ? 4 : 2;

	/* calculate addressing information */
	info->col_addr_cycles = (f->page_size >= 2048) ? 2 : 1;

	if (f->num_blocks * f->page_per_block > 65536)
		info->row_addr_cycles = 3;
	else
		info->row_addr_cycles = 2;

	ndcr |= (nand->enable_arbiter) ? NDCR_ND_ARB_EN : 0;
	ndcr |= (info->col_addr_cycles == 2) ? NDCR_RA_START : 0;
	ndcr |= (f->flash_width == 16) ? NDCR_DWIDTH_M : 0;
	ndcr |= (f->dfc_width == 16) ? NDCR_DWIDTH_C : 0;

	switch (f->page_per_block) {
		case 32:
			ndcr |= NDCR_PG_PER_BLK(0x0);
			break;
		case 128:
			ndcr |= NDCR_PG_PER_BLK(0x1);
			break;
		case 256:
			ndcr |= NDCR_PG_PER_BLK(0x3);
			break;
		case 64:
		default:
			ndcr |= NDCR_PG_PER_BLK(0x2);
			break;
	}

	switch (f->page_size) {
		case 512:
			ndcr |= NDCR_PAGE_SZ(0x0);
			break;
		case 2048:
		default:
			ndcr |= NDCR_PAGE_SZ(0x1);
			ndcr |= NDCR_FORCE_CSX;
			break;

	}

	ndcr |= NDCR_RD_ID_CNT(info->read_id_bytes);
	info->reg_ndcr = ndcr;

	info->timing0 = info->timing1 = 0;
	pxa3xx_nand_set_timing(info, f->timing, show_timing);
	info->flash_info = f;
	return 0;
}

static void pxa3xx_erase_cmd(struct mtd_info *mtd, int page)
{
	struct nand_chip *chip = mtd->priv;
	/* Send commands to erase a block */
	chip->cmdfunc(mtd, NAND_CMD_ERASE1, -1, page);
}

static int pxa3xx_nand_sensing(struct pxa3xx_nand_info *info, int cs)
{
	struct pxa3xx_nand *nand = info->nand_data;
	const struct pxa3xx_nand_flash *f = &nand_common;
	struct mtd_info *mtd = nand->mtd[cs];

	pxa3xx_nand_config_flash(info, f, 0);
	pxa3xx_nand_cmdfunc(mtd, NAND_CMD_RESET, 0, 0);

	if (nand->is_ready)
		return 1;
	else
		return 0;
}

static int pxa3xx_nand_scan_ident(struct mtd_info *mtd, int maxchips)
{
	struct pxa3xx_nand_info *info = mtd->priv;
	struct pxa3xx_nand *nand = info->nand_data;
	struct pxa3xx_nand_flash *f;
	struct nand_chip *chip;
	uint32_t id = -1;
	int i, ret, chip_select;

	f = builtin_flash_types[0];
	chip_select = info->chip_select;
	chip = mtd->priv;
	ret = pxa3xx_nand_sensing(info, chip_select);
	if (!ret) {
		kfree (mtd);
		nand->mtd[chip_select] = NULL;
		printk(KERN_INFO "There is no nand chip on cs %d!\n", chip_select);

		return -EINVAL;
	}

	pxa3xx_nand_cmdfunc(mtd, NAND_CMD_READID, 0, 0);

	id = *((uint16_t *)(info->data_buff));

	if (id != 0)
		printk(KERN_INFO "Detect a flash id %x, cs %x\n", id, chip_select);
	else {
		kfree(mtd);
		nand->mtd[chip_select] = NULL;
		printk(KERN_WARNING "Read out ID 0, potential timing set wrong!!\n");

		return -EINVAL;
	}

	for (i = 1; i < ARRAY_SIZE(builtin_flash_types); i++) {

		f = builtin_flash_types[i];

		/* find the chip in default list */
		if (f->chip_id == (id & f->chip_id_mask)) {
			pxa3xx_nand_config_flash(info, f, 1);
			chip->cellinfo = info->data_buff[2];
			mtd->writesize = f->page_size;
			mtd->writesize_shift = ffs(mtd->writesize) - 1;
			mtd->writesize_mask = (1 << mtd->writesize_shift) - 1;
			mtd->oobsize = mtd->writesize / 32;
			mtd->erasesize = f->page_size * f->page_per_block;
			mtd->erasesize_shift = ffs(mtd->erasesize) - 1;
			mtd->erasesize_mask = (1 << mtd->erasesize_shift) - 1;

			mtd->name = f->name;
			break;
		}
	}

	if (i == ARRAY_SIZE(builtin_flash_types)) {
		kfree(mtd);
		nand->mtd[chip_select] = NULL;
		printk(KERN_ERR "ERROR!! flash not defined!!!\n");

		return -EINVAL;
	}

	chip->ecc.mode		= NAND_ECC_HW;
	chip->ecc.size		= f->page_size;
	chip->ecc.read_page	= pxa3xx_nand_read_page_hwecc;
	chip->ecc.write_page	= pxa3xx_nand_write_page_hwecc;

	if (f->page_size == 2048)
		chip->ecc.layout = &hw_largepage_ecclayout;
	else
		chip->ecc.layout = &hw_smallpage_ecclayout;

	chip->chipsize 		= (uint64_t)f->num_blocks 	* \
				  f->page_per_block 		* \
				  f->page_size;

	chip->chip_shift 	= ffs(chip->chipsize) - 1;
	mtd->size 		= chip->chipsize;

	/* Calculate the address shift from the page size */
	chip->page_shift = ffs(mtd->writesize) - 1;
	chip->pagemask = mtd_div_by_ws(chip->chipsize, mtd) - 1;

	chip->numchips		= 1;
	chip->chip_delay	= 25;
	chip->bbt_erase_shift = chip->phys_erase_shift = ffs(mtd->erasesize) - 1;

	/* Set the bad block position */
	chip->badblockpos = mtd->writesize > 512 ?
		NAND_LARGE_BADBLOCK_POS : NAND_SMALL_BADBLOCK_POS;

	/*
	 * Set chip as a default. Board drivers can override it,
	 * if necessary
	 */
	chip->options = (f->flash_width == 16) ? NAND_BUSWIDTH_16: 0;
	chip->options |= NAND_NO_AUTOINCR;
	chip->options |= NAND_NO_READRDY;
	chip->options |= NAND_USE_FLASH_BBT;
	chip->options |= BBT_RELOCATION_IFBAD;

	return 0;
}

/* the max buff size should be large than 
 * the largest size of page of NAND flash
 * that currently controller support
 */
#define MAX_BUFF_SIZE	((PAGE_CHUNK_SIZE + OOB_CHUNK_SIZE) * 2) + sizeof(struct pxa_dma_desc)

static struct pxa3xx_nand *alloc_nand_resource(struct platform_device *pdev,
						int use_dma)
{
	struct pxa3xx_nand_info		 *info;
	struct pxa3xx_nand 		 *nand;
	struct mtd_info 		 *mtd;
	struct resource 		 *r;
	int data_desc_offset = MAX_BUFF_SIZE - sizeof(struct pxa_dma_desc);
	int ret, irq, i, chip_select;

	nand = kzalloc(sizeof(struct pxa3xx_nand), GFP_KERNEL);
	if (!nand) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		return NULL;
	}

	platform_set_drvdata(pdev, nand);
	nand->clk = clk_get(&pdev->dev, "NANDCLK");
	if (IS_ERR(nand->clk)) {
		dev_err(&pdev->dev, "failed to get nand clock\n");
		goto fail_end;
	}
	clk_enable(nand->clk);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (r == NULL) {
		dev_err(&pdev->dev, "no IO memory resource defined\n");
		goto fail_put_clk;
	}

	r = request_mem_region(r->start, resource_size(r), pdev->name);
	if (r == NULL) {
		dev_err(&pdev->dev, "failed to request memory resource\n");
		goto fail_put_clk;
	}

	nand->mmio_base = ioremap(r->start, r->end - r->start + 1);
	if (nand->mmio_base == NULL) {
		dev_err(&pdev->dev, "ioremap() failed\n");
		goto fail_free_res;
	}

	/* disable all irq before structure initialized */
	disable_int(nand, NDCR_INT_MASK);
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "no IRQ resource defined\n");
		goto fail_free_res;
	}

	ret = request_irq(IRQ_PXA168_NAND, pxa3xx_nand_irq, IRQF_DISABLED,
			pdev->name, nand);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to request IRQ\n");
		goto fail_free_res;
	}

	for (chip_select = 0; chip_select < NUM_CHIP_SELECT; chip_select ++) {
		mtd = kzalloc(sizeof(struct mtd_info)		    \
				+ sizeof(struct pxa3xx_nand_info),  \
				GFP_KERNEL);

		if (!mtd) {
			dev_err(&pdev->dev, "failed to allocate memory\n");
			break;
		}

		info = (struct pxa3xx_nand_info *)(&mtd[1]);
		info->chip_select = chip_select;
		info->nand_data = nand;
		mtd->priv = info;
		mtd->owner = THIS_MODULE;
		nand->mtd[chip_select] = mtd;

		if (use_dma == 0) {

			info->data_buff = kmalloc(MAX_BUFF_SIZE, GFP_KERNEL);
			if (info->data_buff == NULL) {
				break;
			}
		}
		else {
			info->data_buff = dma_alloc_coherent(&pdev->dev,    \
					MAX_BUFF_SIZE,			    \
					&info->data_buff_phys, 		    \
					GFP_KERNEL);

			if (info->data_buff == NULL) {
				dev_err(&pdev->dev, "failed to allocate dma \
						buffer\n");

				break;
			}

			info->data_desc = (void *)info->data_buff   \
					  + data_desc_offset;
			r = platform_get_resource(pdev, IORESOURCE_DMA, 0);
			if (r == NULL) {
				dev_err(&pdev->dev, "no resource defined    \
						for data DMA\n");

				goto fail_free_buf;
			}
			nand->drcmr_dat = r->start;

			r = platform_get_resource(pdev, IORESOURCE_DMA, 1);
			if (r == NULL) {
				dev_err(&pdev->dev, "no resource defined    \
						for command DMA\n");

				goto fail_free_buf;
			}
			nand->drcmr_cmd = r->start;
			info->data_desc_addr = info->data_buff_phys	    \
					       + data_desc_offset;

			nand->data_buff_size = MAX_BUFF_SIZE;
			nand->data_dma_ch = pxa_request_dma("nand-data",    \
					DMA_PRIO_LOW,			    \
					pxa3xx_nand_data_dma_irq, nand);

			if (nand->data_dma_ch < 0) {
				dev_err(&pdev->dev, "failed to request data dma\n");
				goto fail_free_dma;
			}
		}
	}

	return nand;
fail_free_dma:
	if (use_dma)
		pxa_free_dma(nand->data_dma_ch);
fail_free_buf:
	for (i = 0; i < NUM_CHIP_SELECT; i ++) {
		mtd = nand->mtd[i];
		info = mtd->priv;

		if (info->data_buff) {
			if (use_dma)
				dma_free_coherent(&pdev->dev, 		\
						nand->data_buff_size,	\
						info->data_buff, 	\
						info->data_buff_phys);
			else
				kfree(info->data_buff);
		}

		if (mtd)
			kfree(mtd);
	}

	iounmap(nand->mmio_base);
	free_irq(irq, nand);
fail_free_res:
	release_mem_region(r->start, resource_size(r));
fail_put_clk:
	clk_disable(nand->clk);
	clk_put(nand->clk);
fail_end:
	kfree(nand);
	return NULL;
}

static void pxa3xx_nand_init_mtd(struct mtd_info *mtd)
{
	struct pxa3xx_nand_info *info = mtd->priv;
	struct pxa3xx_nand *nand = info->nand_data;
	struct nand_chip *this = &info->nand_chip;

	this->controller	= &nand->controller;
	this->scan_ident	= pxa3xx_nand_scan_ident;
	this->waitfunc		= pxa3xx_nand_waitfunc;
	this->select_chip	= pxa3xx_nand_select_chip;
	this->dev_ready		= pxa3xx_nand_dev_ready;
	this->cmdfunc		= pxa3xx_nand_cmdfunc;
	this->read_word		= pxa3xx_nand_read_word;
	this->read_byte		= pxa3xx_nand_read_byte;
	this->read_buf		= pxa3xx_nand_read_buf;
	this->write_buf		= pxa3xx_nand_write_buf;
	this->verify_buf	= pxa3xx_nand_verify_buf;
	this->erase_cmd		= pxa3xx_erase_cmd;
	this->write_page	= NULL;
	this->bbt		= NULL;
#ifdef CONFIG_PXA3XX_BBM
	this->scan_bbt		= pxa3xx_scan_bbt;
	this->update_bbt	= pxa3xx_update_bbt;
	this->block_markbad	= pxa3xx_block_markbad;
	this->block_bad		= pxa3xx_block_bad;
#else
	this->scan_bbt		= NULL;
	this->update_bbt	= NULL;
	this->block_markbad	= NULL;
	this->block_bad		= NULL;
#endif
}

static int pxa3xx_nand_probe(struct platform_device *pdev)
{
	struct pxa3xx_nand_platform_data *pdata;
	struct pxa3xx_nand 		 *nand;
	struct mtd_info 		 *mtd;
	int    i, ret = -1;
#ifdef CONFIG_MTD_PARTITIONS
	struct mtd_partition *partitions = NULL, *parts = NULL;
	int num_part = 0;
#endif

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev, "no platform data defined\n");
		return -ENODEV;
	}

	nand = alloc_nand_resource(pdev, pdata->use_dma);
	if (!nand)
		return -ENODEV;

	nand->enable_arbiter 	= pdata->enable_arbiter;
	nand->use_dma 		= pdata->use_dma;
	nand->RD_CNT_DEL	= pdata->RD_CNT_DEL;
	spin_lock_init(&nand->controller.lock);
	init_waitqueue_head(&nand->controller.wq);

	for (i = 0; i < NUM_CHIP_SELECT; i ++) {
		mtd = nand->mtd[i];
		pxa3xx_nand_init_mtd(mtd);
		if (nand_scan(mtd, 1))
			continue;

#ifdef CONFIG_MTD_PARTITIONS
#ifdef CONFIG_MTD_CMDLINE_PARTS
		mtd->name = mtd_names[i];
		num_part = parse_mtd_partitions(mtd, part_probes, &partitions, 0);
#endif
		if (num_part <= 0) {
			num_part = pdata->nr_parts[i];
			partitions = pdata->parts[i];
		}

		if (partitions && num_part > 0) {
#ifdef CONFIG_PXA3XX_BBM
			struct pxa3xx_bbm *pxa3xx_bbm = mtd->bbm;
			parts = pxa3xx_bbm->check_partition(mtd, partitions, &num_part);
			if (!parts)
				return -EINVAL;
#else
			parts = partitions;
#endif

			ret = add_mtd_partitions(mtd, parts, num_part);
#ifdef CONFIG_PXA3XX_BBM
			kfree(parts);
#endif
		}
		else
			ret = add_mtd_device(mtd);
#else
		ret = add_mtd_device(mtd);
#endif
	}

	return ret;
}

static int pxa3xx_nand_remove(struct platform_device *pdev)
{
	struct pxa3xx_nand *nand = platform_get_drvdata(pdev);
	struct mtd_info *mtd;
	struct pxa3xx_nand_info *info;
	struct resource *r;
	int i;
#ifdef CONFIG_PXA3XX_BBM
	struct pxa3xx_bbm *pxa3xx_bbm;
#endif

	pxa3xx_nand_stop(nand);
	platform_set_drvdata(pdev, NULL);
	free_irq(IRQ_PXA168_NAND, nand);
	if (nand->use_dma)
		pxa_free_dma(nand->data_dma_ch);

	iounmap(nand->mmio_base);
	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(r->start, resource_size(r));

	clk_disable(nand->clk);
	clk_put(nand->clk);

	for (i = 0; i < NUM_CHIP_SELECT; i ++) {
		mtd = nand->mtd[i];
		if (!mtd)
			continue;
		info = mtd->priv;
#ifdef CONFIG_PXA3XX_BBM
		pxa3xx_bbm = mtd->bbm;
		pxa3xx_bbm->uninit(mtd);
#endif
		if (nand->use_dma) {
			dma_free_writecombine(&pdev->dev, nand->data_buff_size,
					info->data_buff, info->data_buff_phys);
		} else
			kfree(info->data_buff);
		del_mtd_device(mtd);
		del_mtd_partitions(mtd);
		kfree(mtd);
	}

	return 0;
}

#ifdef CONFIG_PM
static unsigned int ndtr0cs0, ndtr1cs0;

static int pxa3xx_nand_suspend(struct platform_device *pdev,
		pm_message_t state)
{
	struct pxa3xx_nand *nand = platform_get_drvdata(pdev);
	struct mtd_info *mtd = nand->mtd[nand->chip_select];
	int ret = 0;

	if (nand->state & STATE_CMD_PREPARED) {
		dev_err(&pdev->dev, "driver busy, state = %d\n", nand->state);
		return -EAGAIN;
	}

	if (mtd)
		ret = mtd->suspend(mtd);

	ndtr0cs0 = nand_readl(nand, NDTR0CS0);
	ndtr1cs0 = nand_readl(nand, NDTR1CS0);

	return ret;
}

static int pxa3xx_nand_resume(struct platform_device *pdev)
{
	struct pxa3xx_nand *nand = platform_get_drvdata(pdev);
	struct mtd_info *mtd = nand->mtd[nand->chip_select];
	struct pxa3xx_nand_info *info = mtd->priv;

	nand_writel(nand, NDTR0CS0, ndtr0cs0);
	nand_writel(nand, NDTR1CS0, ndtr1cs0);
	nand_writel(nand, NDREDEL, 0x0);

	no_error_dump = 1;
	pxa3xx_nand_start(info);
	pxa3xx_nand_stop(nand);
	no_error_dump = 0;
	if (mtd)
		mtd->resume(mtd);
	return 0;
}
#else
#define pxa3xx_nand_suspend	NULL
#define pxa3xx_nand_resume	NULL
#endif

static struct platform_driver pxa3xx_nand_driver = {
	.driver = {
		.name	= "pxa3xx-nand",
	},
	.probe		= pxa3xx_nand_probe,
	.remove		= pxa3xx_nand_remove,
	.suspend	= pxa3xx_nand_suspend,
	.resume		= pxa3xx_nand_resume,
};

static int __init pxa3xx_nand_init(void)
{
#if defined(CONFIG_DVFM)
	dvfm_register("NAND", &dvfm_dev_idx);
#endif
	return platform_driver_register(&pxa3xx_nand_driver);
}
module_init(pxa3xx_nand_init);

static void __exit pxa3xx_nand_exit(void)
{
#if defined(CONFIG_DVFM)
	dvfm_unregister("NAND", &dvfm_dev_idx);
#endif
	platform_driver_unregister(&pxa3xx_nand_driver);
}
module_exit(pxa3xx_nand_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("PXA3xx NAND controller driver");
