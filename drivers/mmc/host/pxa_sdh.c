/*
 *  linux/drivers/mmc/host/pxa_sdh.c - PXAxxx SD Host driver
 *
 *  Copyright (C) 2008-2009 Marvell International Ltd.
 *                Mingwei Wang <mwwang@marvell.com>
 *                Kevin Wang <kevin.wang@marvell.com>
 *
 *  Based on linux/drivers/mmc/host/pxa.c - PXA MMCI driver
 *
 *  Copyright (C) 2003 Russell King, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>

#include <mach/mmc.h>
#include <asm/mach-types.h>

#include "pxa_sdh.h"

#ifdef CONFIG_DVFM
#include <mach/dvfm.h>
static struct dvfm_lock dvfm_lock = {
	.lock		= SPIN_LOCK_UNLOCKED,
	.count		= 0,
	.dev_idx	= -1,
};

static void set_dvfm_constraint(void)
{
	spin_lock_irqsave(&dvfm_lock.lock, dvfm_lock.flags);
	if (dvfm_lock.count++ == 0) {
		/* Disable Low power mode */
		dvfm_disable_op_name("apps_idle", dvfm_lock.dev_idx);
		dvfm_disable_op_name("apps_sleep", dvfm_lock.dev_idx);
		dvfm_disable_op_name("sys_sleep", dvfm_lock.dev_idx);
	} else
		dvfm_lock.count--;
	spin_unlock_irqrestore(&dvfm_lock.lock, dvfm_lock.flags);
}

static void unset_dvfm_constraint(void)
{
	spin_lock_irqsave(&dvfm_lock.lock, dvfm_lock.flags);
	if (dvfm_lock.count == 0) {
		spin_unlock_irqrestore(&dvfm_lock.lock, dvfm_lock.flags);
		return;
	}
	if (--dvfm_lock.count == 0) {
		/* Enable Low power mode */
		dvfm_enable_op_name("apps_idle", dvfm_lock.dev_idx);
		dvfm_enable_op_name("apps_sleep", dvfm_lock.dev_idx);
		dvfm_enable_op_name("sys_sleep", dvfm_lock.dev_idx);
	} else
		dvfm_lock.count++;
	spin_unlock_irqrestore(&dvfm_lock.lock, dvfm_lock.flags);
}
#else
static void set_dvfm_constraint(void) {}
static void unset_dvfm_constraint(void) {}
#endif

#define DRIVER_NAME	"pxa-sdh"

#define ADMA_DESC_TBL_SIZE		PAGE_SIZE
#define CLKRT_OFF			(~0)
#define GET_REG(host, reg) 		readw(host->base + reg)
#define SET_REG(host, val, reg)		writew(val, host->base + reg)
#define DATA_DIRECTION_READ(data) 	(data->flags & MMC_DATA_READ)
#define SET_REG_BIT(host, bit_mask, reg) \
		SET_REG(host, \
			GET_REG(host, reg) | (bit_mask), reg)
#define CLEAR_REG_BIT(host, bit_mask, reg) \
		SET_REG(host, \
			GET_REG(host, reg) & ~(bit_mask), reg)
#define SET_REG_BITS(host, bits_pos, bits_mask, val, reg) \
		SET_REG(host, \
			GET_REG(host, reg) & ~(bits_mask << bits_pos), reg); \
		SET_REG(host, \
			GET_REG(host, reg) | (val << bits_pos), reg)
#define GET_REG_BITS(host, bit_pos, bits_mask, reg) \
		((GET_REG(host, reg) >> bit_pos) & bits_mask)
#define SG_NEED_PIO(sg)  (sg_dma_len(sg) < 64)

#if 0
#define dev_dbg(dev, format, arg...) \
	dev_info(dev , format , ## arg)
#endif

struct adma_desc_table {
	u16 entry_attr;
	u16 entry_length;
	u32 entry_address;
};

struct pxa_sdh_host {
	struct mmc_host		*mmc;
	spinlock_t		lock;
	struct resource		*res;
	void __iomem		*base;
	struct clk		*clk;
	u32		clkrate;
	int		irq;
	u32		clkrt;
	u32		segment;
	u32		power_mode;
	struct pxasdh_platform_data *pdata;

	struct mmc_request	*mrq;
	struct mmc_command	*cmd;

	struct adma_desc_table	*desc_tbl_addr;
	dma_addr_t		desc_tbl_dma_addr;
	unsigned int 		sg_index;
	u32			data_len;
	unsigned int		pio_sg_pos;
	unsigned int 		dma_sg_len;
};

#ifdef CONFIG_MMC_DEBUG
static void __attribute__((unused)) dump_registers(struct pxa_sdh_host *host)
{
	unsigned int val;
	int offset;

	for (offset = 0; offset < 0x60; offset += 4) {
		if (offset == 0x20)
			continue;
		val = readl(host->base + offset);
		printk(KERN_ERR "%08x: %08x\n", (unsigned int)host->base + offset, val);
	}
	for (offset = 0xE0; offset < 0xF0; offset += 4) {
		val = readl(host->base + offset);
		printk(KERN_ERR "%08x: %08x\n", (unsigned int)host->base + offset, val);
	}
	val = readl(host->base + 0xFC);
	printk(KERN_ERR "%08x: %08x\n", (unsigned int)host->base + 0xFC, val);
}
#endif

static int pxa_sdh_wait_reset(struct pxa_sdh_host *host)
{
	u32 timeout = 1000;
	u16 val;

	do {
		val = GET_REG(host, SD_TO_CTRL_SW_RST);
		if (!(val & (SW_RST_DAT | SW_RST_CMD | SW_RST_ALL)))
			break;
		udelay(1);
	} while (timeout--);
	if (timeout)
		return 0;

	dev_err(mmc_dev(host->mmc), "Fatal: Wait RESET timeout.\n");

	return 1;
}

static int pxa_sdh_get_cd(struct mmc_host *mmc)
{
#if 1
	return 1;
#else
	/*not stable on TTC EVB*/
	struct pxa_sdh_host *host = mmc_priv(mmc);
	u32 timeout = 1000;

	if (pxa_sdh_wait_reset(host))
		return 0;

	do {
		if (GET_REG(host, SD_PRESENT_STAT_2) & CARD_STABLE)
			break;
	} while (timeout--);
	if (!timeout) {
		dev_err(mmc_dev(mmc), "Card State is not stable!\n");
	} else if (GET_REG(host, SD_PRESENT_STAT_2) & CARD_DETECTED) {
		dev_dbg(mmc_dev(mmc), "Card is detected!\n");
		return 1;
	}

	dev_dbg(mmc_dev(mmc), "Card is not detected!\n");
	return 0;
#endif
}

static void pxa_sdh_stop_clock(struct pxa_sdh_host *host)
{
	CLEAR_REG_BIT(host, EXT_CLK_EN, SD_CLOCK_CNTL);
}

static void pxa_sdh_start_clock(struct pxa_sdh_host *host)
{
	u32 timeout = 1000;

	SET_REG_BIT(host, INT_CLK_EN, SD_CLOCK_CNTL);
	do {
		if (GET_REG(host, SD_CLOCK_CNTL) & INT_CLK_STABLE)
			break;
		udelay(1);
	} while (timeout--);
	if (!timeout)
		dev_err(mmc_dev(host->mmc), "unable to start clock\n");
	
	SET_REG_BITS(host, SD_FREQ_SEL_OFFSET, SD_FREQ_SEL_MASK,
		host->clkrt, SD_CLOCK_CNTL);

	/* set as maximum value for data line timeout*/
	SET_REG_BITS(host, DAT_TO_VAL_OFFSET, DAT_TO_MASK,
		(DAT_TO_MASK - 1), SD_TO_CTRL_SW_RST);

	SET_REG_BIT(host, EXT_CLK_EN, SD_CLOCK_CNTL);
}

static void pxa_sdh_enable_irq(struct pxa_sdh_host *host, u32 mask)
{
	unsigned long flags;
	u16 nor_mask, nor_int_mask, err_mask, nor_val, err_val;

	spin_lock_irqsave(&host->lock, flags);
	nor_mask = (mask & 0xffff) & ~SD_NOR_I_STAT_RVD_MASK;
	nor_int_mask = (mask & 0xffff) & ~SD_NOR_INT_EN_RVD_MASK;
	err_mask = mask >> 16;
	nor_val = GET_REG(host, SD_NOR_I_STAT_EN);
	err_val = GET_REG(host, SD_ERR_I_STAT_EN);

	SET_REG(host, nor_mask, SD_NOR_I_STAT);
	SET_REG(host, err_mask, SD_ERR_I_STAT);
	SET_REG(host, nor_val | nor_int_mask, SD_NOR_INT_EN);
	SET_REG(host, nor_val | nor_int_mask, SD_NOR_I_STAT_EN);
	SET_REG(host, err_val | err_mask, SD_ERR_INT_EN);
	SET_REG(host, err_val | err_mask, SD_ERR_I_STAT_EN);
	spin_unlock_irqrestore(&host->lock, flags);
}

static void pxa_sdh_disable_irq(struct pxa_sdh_host *host, u32 mask)
{
	unsigned long flags;
	u16 nor_mask, nor_int_mask, err_mask, nor_val, err_val;

	spin_lock_irqsave(&host->lock, flags);
	nor_mask = (mask & 0xffff) & ~SD_NOR_I_STAT_RVD_MASK;
	nor_int_mask = (mask & 0xffff) & ~SD_NOR_INT_EN_RVD_MASK;
	err_mask = mask >> 16;
	nor_val = GET_REG(host, SD_NOR_I_STAT_EN);
	err_val = GET_REG(host, SD_ERR_I_STAT_EN);
	
	SET_REG(host, nor_mask, SD_NOR_I_STAT);
	SET_REG(host, err_mask, SD_ERR_I_STAT);
	SET_REG(host, nor_val & ~nor_int_mask, SD_NOR_INT_EN);
	SET_REG(host, nor_val & ~nor_int_mask, SD_NOR_I_STAT_EN);
	SET_REG(host, err_val & ~err_mask, SD_ERR_INT_EN);
	SET_REG(host, err_val & ~err_mask, SD_ERR_I_STAT_EN);
	spin_unlock_irqrestore(&host->lock, flags);
}

static void pxa_sdh_setup_sdma(struct pxa_sdh_host *host, struct scatterlist *sg, 
		unsigned int dir_read)
{
	dma_map_sg(mmc_dev(host->mmc), sg, 1, 
		dir_read ? DMA_FROM_DEVICE : DMA_TO_DEVICE);

	SET_REG(host, sg_dma_address(sg) & 0xffff, SD_SYS_ADDR_LOW);
	SET_REG(host, sg_dma_address(sg) >> 16, SD_SYS_ADDR_HIGH);
}

static u32 pxa_sdh_setup_adma2(struct pxa_sdh_host *host, struct scatterlist *sg,
		unsigned int dir_read, u32 sg_len)
{
	u32 data_len = 0;
	unsigned int i = 0;

	BUG_ON(sg_len != dma_map_sg(mmc_dev(host->mmc), sg, sg_len, 
		dir_read ? DMA_FROM_DEVICE : DMA_TO_DEVICE));

	for (i = 0; i < sg_len; i++) {
		host->desc_tbl_addr[i].entry_address = sg_dma_address(&sg[i]);
		host->desc_tbl_addr[i].entry_length = sg_dma_len(&sg[i]);
		host->desc_tbl_addr[i].entry_attr = 0x0021;
		data_len += sg_dma_len(&sg[i]);
		BUG_ON(sg_dma_len(&sg[i]) > (1 << 16));
	}

	/*The Last Link, Set End bit*/
	host->desc_tbl_addr[sg_len - 1].entry_attr |= 0x2;

	SET_REG(host, host->desc_tbl_dma_addr & 0xffff, SD_ADMA_SADDR_1);
	SET_REG(host, host->desc_tbl_dma_addr >> 16, SD_ADMA_SADDR_2);

	return data_len;
}

static void pxa_sdh_setup_data(struct pxa_sdh_host *host)
{
	struct mmc_data *data = host->mrq->data;
	unsigned int dir_read =  DATA_DIRECTION_READ(data);
	struct scatterlist *sg; 
	u16 blk_size = data->blksz & BLOCK_SIZE_MASK;
	u32 data_len = 0;

	sg = &data->sg[host->sg_index];
	host->dma_sg_len = host->pio_sg_pos = 0;

	if (0 && SG_NEED_PIO(sg)) {
		void *sg_va = sg_virt(sg);
		data_len = sg_dma_len(sg);

		dev_dbg(mmc_dev(host->mmc),
			"Using PIO(H%sC): addr=0x%x, len=0x%x, sg_index=%d/%d\n",
			dir_read ? "<-": "->", (unsigned int)sg_va, sg_dma_len(sg),
			host->sg_index + 1, data->sg_len);
		pxa_sdh_disable_irq(host, DMA_INT);
		if (dir_read) {
			pxa_sdh_enable_irq(host, RX_RDY);
		 } else {
			u16 i = 0;
			pxa_sdh_enable_irq(host, TX_RDY);
			for (; i < blk_size; i += sizeof(u32)) {
				writel(*(u32*)(sg_va + i), host->base + SD_BUF_DPORT_0);
			}
		}
	} else {
		u16 dma_sel = DMA_SEL_SDMA;
		for (host->dma_sg_len++; host->sg_index + host->dma_sg_len < data->sg_len;) {
			if (SG_NEED_PIO(&sg[host->sg_index + host->dma_sg_len])) {
				break;
			}
			host->dma_sg_len++; 
		}
		if (host->dma_sg_len == 1) {
			pxa_sdh_setup_sdma(host, sg, dir_read);
			data_len = sg_dma_len(sg); 
			dev_dbg(mmc_dev(host->mmc),
				"Using SDMA(H%sC): addr=0x%x, len=0x%x, sg_index=%d/%d\n",
				dir_read ? "<-": "->", (unsigned int)sg_virt(sg), sg_dma_len(sg),
				host->sg_index + 1, data->sg_len);
		} else {
			data_len = pxa_sdh_setup_adma2(host, sg, dir_read, host->dma_sg_len);
			dma_sel = DMA_SEL_ADMA2_32;
			dev_dbg(mmc_dev(host->mmc),
				"Using ADMA2(H%sC): saddr=0x%x, len=0x%x, sg_index=%d-%d/%d\n",
				dir_read ? "<-": "->", (unsigned int)sg_virt(sg), (unsigned int)data_len,
				host->sg_index + 1, host->sg_index + host->dma_sg_len, data->sg_len);
		}
		SET_REG_BITS(host, DMA_SEL_OFFSET, DMA_SEL_MASK, dma_sel, SD_HOST_CTRL);
		pxa_sdh_enable_irq(host, DMA_INT);
		pxa_sdh_disable_irq(host, RX_RDY | TX_RDY);
		
		/* if 8-byte aligned set DMA burst size as 64 for better performance */
		/*SET_REG_BIT(host, DMA_BURST_SIZE, SD_CLK_BURST_SET);
		do {
			if (sg_dma_len(&sg[host->sg_index + i]) % 8)
				CLEAR_REG_BIT(host, DMA_BURST_SIZE, SD_CLK_BURST_SET);
				break;
			i++;
		} while (i < host->dma_sg_len);*/
	}
	host->data_len = data_len;
}

static void pxa_sdh_start_cmd(struct pxa_sdh_host *host, struct mmc_command *cmd)
{
	u16 resp = 0;
	u16 xfrmd_val = 0;
	u16 cmd_val = 0;
	u16 val, mask;

	BUG_ON(host->cmd != NULL);
	host->cmd = cmd;

	/*Set Response Type*/
	switch (mmc_resp_type(cmd)) {
	case MMC_RSP_NONE:
		break;

	case MMC_RSP_R1: /* r1, r5, r6, r7 */
		resp = CMD_RESP_48BIT;
		cmd_val |= CMD_CRC_CHK_EN | CMD_IDX_CHK_EN;
		break;

	case MMC_RSP_R2: /* r2 */
		resp = CMD_RESP_136BIT;
		cmd_val |= CMD_CRC_CHK_EN;
		break;

	case MMC_RSP_R3: /* r3, r4*/
		resp = CMD_RESP_48BIT;
		break;

	case MMC_RSP_R1B: /* r1b */
		resp = CMD_RESP_48BITB;
		cmd_val |= CMD_CRC_CHK_EN | CMD_IDX_CHK_EN;
		break;

	default:
		break;
	}

	/*Set Transfer mode regarding to data flag*/
	if (cmd->data)
	{
		cmd_val |= DATA_PRESENT;
		xfrmd_val |= BLK_CNT_EN;
		if (cmd->data->blocks > 1)
			xfrmd_val |= MULTI_BLK_SEL;
		if (host->dma_sg_len)
			xfrmd_val |= DMA_EN;

		if (DATA_DIRECTION_READ(cmd->data))
			xfrmd_val |= TO_HOST_DIR;
		else
			xfrmd_val &= ~TO_HOST_DIR;
	}

	//if (cmd->opcode == 12)
	//	cmd_val |= host, CMD_TYPE_OFFSET, CMD_TYPE_MASK, CMD_TYPE_ABORT, SD_COMMAND);
	SET_REG(host, cmd->arg & 0xffff, SD_ARG_LOW);
	SET_REG(host, cmd->arg >> 16, SD_ARG_HIGH);
	SET_REG(host, xfrmd_val, SD_TRANS_MODE);
	cmd_val |= cmd->opcode << CMD_IDX_OFFSET | resp << RESP_TYPE_OFFSET;
	dev_dbg(mmc_dev(host->mmc), "Starting CMD%d with ARGUMENT 0x%x\n", cmd->opcode, cmd->arg);

	if (!host->mmc->card || !mmc_card_sdio(host->mmc->card)) {
		val = GET_REG(host, SD_PRESENT_STAT_2);
		mask = CMD_LINE_LEVEL_MASK | DATA_LINE_LEVEL_MASK;
		if ((val & mask) != mask)
			dev_dbg(mmc_dev(host->mmc), "WARN: CMD/DATA pins are not all high, PRE_STAT=0x%04x\n",
				GET_REG(host, SD_PRESENT_STAT_2));
	}

	/* FIXME workaround for DPF-532, host needs to delay
	 * when programming sdio controller (very bad) */
	if (!host->mmc->card)
		udelay(10);

	SET_REG(host, cmd_val, SD_COMMAND);
}

static void pxa_sdh_finish_request(struct pxa_sdh_host *host,
	struct mmc_request *mrq)
{
	pxa_sdh_disable_irq(host, ~(CARD_INT | CARD_INS	| CARD_REM));

	if (host->mrq->data && host->mrq->data->error)
		SET_REG_BIT(host, SW_RST_DAT, SD_TO_CTRL_SW_RST);

	dev_dbg(mmc_dev(host->mmc), "Finishing CMD%d(%s)\n", mrq->cmd->opcode,
		(mrq->cmd->error || (mrq->data && mrq->data->error)) ? "failed" : "done");

	host->mrq = NULL;
	host->cmd = NULL;
	host->sg_index = 0;
	host->pio_sg_pos = 0;
	host->dma_sg_len = 0;

	mmc_request_done(host->mmc, mrq);
	unset_dvfm_constraint();
}

static int pxa_sdh_cmd_done(struct pxa_sdh_host *host)
{
	struct mmc_command *cmd = host->cmd;
	u32 resp[8];

	BUG_ON(!cmd);
	host->cmd = NULL;

	/* get cmd response */
	resp[0] = GET_REG(host, SD_RESP_0);
	resp[1] = GET_REG(host, SD_RESP_1);
	resp[2] = GET_REG(host, SD_RESP_2);
	resp[3] = GET_REG(host, SD_RESP_3);
	resp[4] = GET_REG(host, SD_RESP_4);
	resp[5] = GET_REG(host, SD_RESP_5);
	resp[6] = GET_REG(host, SD_RESP_6);
	resp[7] = readb(host->base + SD_RESP_7);

	if (cmd->flags & MMC_RSP_136) {
		cmd->resp[0] = resp[5] >> 8 | resp[6] << 8 | resp[7] << 24;
		cmd->resp[1] = resp[3] >> 8 | resp[4] << 8 | resp[5] << 24;
		cmd->resp[2] = resp[1] >> 8 | resp[2] << 8 | resp[3] << 24;
		cmd->resp[3] = resp[0] << 8 | resp[1] << 24;
	} else {
		cmd->resp[0] = resp[1] << 16 | resp[0];
		cmd->resp[1] = resp[3] << 16 | resp[2];
		cmd->resp[2] = resp[5] << 16 | resp[4];
		cmd->resp[3] = resp[7] << 16 | resp[6];
	}

	if (cmd->error || !host->mrq->data || (host->mrq->cmd != cmd)) {
		pxa_sdh_finish_request(host, host->mrq);
	}

	return 1;
}

static void pxa_sdh_pio_data_done(struct pxa_sdh_host *host)
{
	struct mmc_data *data = host->mrq->data;
	u16 blk_size = data->blksz; 
	struct scatterlist *sg = &data->sg[host->sg_index];
	char *sg_va = (char*)sg_virt(sg) + host->pio_sg_pos;
	u16 i = 0;

	BUG_ON (host->pio_sg_pos + blk_size > sg_dma_len(sg));

	if (DATA_DIRECTION_READ(data)) {
		for (i = 0; i < blk_size; i += sizeof(u32)) {
			*(u32*)(sg_va + i) = readl(host->base + SD_BUF_DPORT_0);
		}
	} else {
		if (host->pio_sg_pos < sg_dma_len(sg)) {
			for (i = 0; i < blk_size; i += sizeof(u32)) {
				writel(*(u32*)(sg_va + i), host->base + SD_BUF_DPORT_0);
			}
		}
	}

	host->pio_sg_pos += blk_size;
	data->bytes_xfered += blk_size;

	if (host->pio_sg_pos >= sg_dma_len(sg)) {
		host->sg_index++;
		if (host->sg_index < data->sg_len)
			pxa_sdh_setup_data(host);
	}
}

static void pxa_sdh_dma_data_done(struct pxa_sdh_host *host)
{
	struct mmc_data *data = host->mrq->data;

	host->sg_index += host->dma_sg_len;
	data->bytes_xfered += host->data_len;
	if (host->sg_index < data->sg_len)
		pxa_sdh_setup_data(host);
}

static void pxa_sdh_data_done(struct pxa_sdh_host *host)
{
	struct mmc_data *data = host->mrq->data;
	 
	if (!host->mrq || !host->mrq->data) //ignore unexpected XFER_COMP interrupt
		return;

	if (host->dma_sg_len)
		dma_unmap_sg(mmc_dev(host->mmc), &data->sg[host->sg_index], host->dma_sg_len, 
		     DATA_DIRECTION_READ(host->mrq->data) ? DMA_FROM_DEVICE : DMA_TO_DEVICE);
	
	if (!data->error) {
		if (host->sg_index  < data->sg_len) {
			if (!host->dma_sg_len) {
				if (host->pio_sg_pos < sg_dma_len(&data->sg[host->sg_index])){
					BUG_ON (!DATA_DIRECTION_READ(data));
					pxa_sdh_pio_data_done(host);
				}
				BUG_ON (host->pio_sg_pos != sg_dma_len(&data->sg[host->sg_index - 1]));
			} else {
				pxa_sdh_dma_data_done(host);
			}
		}

		BUG_ON (host->sg_index != data->sg_len);
		BUG_ON (data->bytes_xfered != data->blocks * data->blksz);
	}

	/*
	 * If there was an error on any block, we mark all
	 * data blocks as being in error.
	 */
	if (data->error)
		data->bytes_xfered = 0;

	if (host->mrq->stop)
		pxa_sdh_start_cmd(host, host->mrq->stop);
	else
		pxa_sdh_finish_request(host, host->mrq);
}

static inline int query_irq(u16 *irqs, u16 mask)
{
	u16 re = *irqs & mask;
	*irqs &= ~mask;
	return re;
}

static int pxa_sdh_err_int_handler(struct pxa_sdh_host *host,  u16 err_stat)
{
	u16 err_stat_bk = err_stat;
	BUG_ON(err_stat & CPL_TO_ERR); /* CE-ATA mode only, not support yet*/
	if (query_irq(&err_stat, AXI_RESP_ERR)) {
		dev_err(mmc_dev(host->mmc), 
			"AXI Bus Response Error, non-reconverable!\n");
		host->cmd->error = -ENOTRECOVERABLE;
		pxa_sdh_finish_request(host, host->mrq);
		goto out;
	}
	BUG_ON(err_stat & SPI_ERR); /* SPI mode only, not support yet*/
	if (query_irq(&err_stat, ADMA_ERR)) {
		dev_err(mmc_dev(host->mmc), 
			"ADMA Error(status: 0x%x)!\n",
			GET_REG(host, SD_ADMA_ERR_STAT));
		host->mrq->data->error = -EIO;
		pxa_sdh_data_done(host);
		goto out;
	}
	BUG_ON(err_stat & AUTO_CMD12_ERR); /* Auto CMD12 not used yet*/
	BUG_ON(err_stat & CUR_LIMIT_ERR); /* Not support by host Controller*/
	if (query_irq(&err_stat, SD_ERR_INT_CMD_ERR_MASK)) {
		dev_dbg(mmc_dev(host->mmc), 
			"CMD Line Error(status: 0x%04x)!\n", err_stat_bk);
		SET_REG(host, \
			GET_REG(host, SD_TO_CTRL_SW_RST) | SW_RST_CMD, \
			SD_TO_CTRL_SW_RST);
		if (err_stat & CMD_TO_ERR)
			host->cmd->error = -ETIMEDOUT;
		else
			host->cmd->error = -EIO;
		pxa_sdh_cmd_done(host);
		goto out;
	}
	if (query_irq(&err_stat, SD_ERR_INT_DATA_ERR_MASK)) {
		dev_err(mmc_dev(host->mmc), 
			"DATA Line Error(status: 0x%04x)!\n", err_stat_bk);	
		SET_REG(host, \
			GET_REG(host, SD_TO_CTRL_SW_RST) | SW_RST_DAT, \
			SD_TO_CTRL_SW_RST);
		if (err_stat & CRC_STATUS_ERR) {
			BUG_ON(!host->mrq->data || (DATA_DIRECTION_READ(host->mrq->data)));
		}
		if (err_stat & DATA_TO_ERR)
			host->mrq->data->error = -ETIMEDOUT;
		else
			host->mrq->data->error = -EIO;
		pxa_sdh_data_done(host);
		goto out;
	}

out:
	BUG_ON(err_stat);
	return 1;
}

static irqreturn_t pxa_sdh_irq(int irq, void *devid)
{
	struct pxa_sdh_host *host = devid;
	u16 nor_stat, err_stat;
	
	nor_stat = GET_REG(host, SD_NOR_I_STAT);
	if (nor_stat == 0 || nor_stat == 0xFFFF)
		return IRQ_NONE;

	SET_REG(host, nor_stat & ~ERR_INT & ~CARD_INT, SD_NOR_I_STAT);
	err_stat = GET_REG(host, SD_ERR_I_STAT);
	SET_REG(host, err_stat, SD_ERR_I_STAT);

	dev_dbg(mmc_dev(host->mmc), 
		"Card Interrupt, nor_stat: 0x%x, err_stat: 0x%x\n", nor_stat, err_stat);

	if (query_irq(&nor_stat, CARD_INT)) {
		dev_dbg(mmc_dev(host->mmc), "CARD_INT Detected!\n");
		mmc_signal_sdio_irq(host->mmc);
	}
	if (query_irq(&nor_stat, CARD_REM)) {
		if (host->pdata->mfp_config)
			host->pdata->mfp_config();
		mmc_detect_change(host->mmc, host->pdata->detect_delay);
	}
	if (query_irq(&nor_stat, CARD_INS)) {
		if (host->pdata->mfp_config)
			host->pdata->mfp_config();
		mmc_detect_change(host->mmc, host->pdata->detect_delay);
	}

	if (!nor_stat)
		goto out;

	BUG_ON(!host->mrq);

	if (query_irq(&nor_stat, ERR_INT)) {
		BUG_ON(!err_stat);
		if (query_irq(&err_stat, CRC_STATUS_ERR)) {
			/*a strange error, have no idea when happens but seems like DATA-related.*/
			dev_dbg(mmc_dev(host->mmc), 
				"CRC Status Error detected, strange!\n");
			SET_REG_BIT(host, SW_RST_DAT | SW_RST_CMD, SD_TO_CTRL_SW_RST);
			if (host->mrq->data)
				host->mrq->data->error = -ECANCELED;
			else
				BUG_ON(1);
			BUG_ON(nor_stat != XFER_COMP);
			nor_stat = 0;
			pxa_sdh_finish_request(host, host->mrq);
			goto out;
		} else {
			nor_stat = 0;
			pxa_sdh_err_int_handler(host, err_stat);
			goto out;
		}
	}

	BUG_ON(err_stat);

	if (query_irq(&nor_stat, CMD_COMP)) {
		pxa_sdh_cmd_done(host);
	}
	if ((nor_stat & TX_RDY) || (nor_stat & RX_RDY)) {
		BUG_ON((nor_stat & (RX_RDY | TX_RDY)) == (RX_RDY | TX_RDY));
		nor_stat &= ~(RX_RDY | TX_RDY);
		pxa_sdh_pio_data_done(host);
	}
	if (query_irq(&nor_stat, DMA_INT)) {
		pxa_sdh_dma_data_done(host);
	}
	if (query_irq(&nor_stat, XFER_COMP)) {
		pxa_sdh_data_done(host);
	}

out:
	BUG_ON(nor_stat);

	return IRQ_RETVAL(1);
}

static void pxa_sdh_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct pxa_sdh_host *host = mmc_priv(mmc);
	u16 val;
	u32 timeout = 1000, i;

	BUG_ON(host->mrq != NULL);

	/* release this constraint when request is finished */
	set_dvfm_constraint();

	if (!pxa_sdh_get_cd(mmc)) {
		mrq->cmd->error = -ENODEV;
		pxa_sdh_finish_request(host, mrq);
		return;
	}
	if (pxa_sdh_wait_reset(host)) {
		mrq->cmd->error = -EBUSY;
		pxa_sdh_finish_request(host, mrq);
		return;
	}

	do {
		val = GET_REG(host, SD_PRESENT_STAT_1);
		if (!(val & CMD_INHBT_DAT || val & CMD_INHBT_CMD))
			break;
		udelay(1);
	} while (timeout--);
	if (!timeout) {
		dev_err(mmc_dev(mmc), "In busy, unable to start the request.\n");
		mrq->cmd->error = -EBUSY;
		pxa_sdh_finish_request(host, mrq);
		return;
	}

	pxa_sdh_enable_irq(host, ~(u32)CARD_INT & ~(u32)BLK_GAP_EVNT &
			~(u32)TX_RDY & ~(u32)RX_RDY & ~(u32)DMA_INT);
	host->mrq = mrq;

	if (mrq->data) {
		struct scatterlist *sg;
		unsigned int data_len = 0;
		if (mrq->data->flags & MMC_DATA_STREAM) {
			dev_err(mmc_dev(mmc), "No stream commands support\n");
			goto inval_out;
		}
		for (i = 0; i < mrq->data->sg_len; i++) {
			sg = &mrq->data->sg[i];
			if ((unsigned int)sg_virt(sg) % 4) {
				dev_err(mmc_dev(host->mmc),
					"sg_addr 0x%x(sg_len=%x) is not 4-byte alined, unsupported!\n",
					(unsigned int)sg_virt(sg), sg_dma_len(sg));
				goto inval_out;
			}	
			if (sg_dma_len(sg) % 4) {
				dev_err(mmc_dev(host->mmc),
					"sg_len 0x%x is not 4-byte alined, unsupported!\n",
					sg_dma_len(sg));
				goto inval_out;
			}
			if (sg_dma_len(sg) % mrq->data->blksz) {
				dev_err(mmc_dev(host->mmc),
					"sg_len 0x%x is not multiple of blk_size 0x%x, unsupported!\n",
					sg_dma_len(sg), mrq->data->blksz);
				goto inval_out;
			} else {
				data_len += sg_dma_len(sg);
			}
		}
		if (data_len / mrq->data->blksz != mrq->data->blocks) {
			dev_err(mmc_dev(host->mmc),
				"data_len(0x%x) != blk_size(0x%x) * blk_cnt(0x%x), unsupported!\n",
				data_len, mrq->data->blksz, mrq->data->blocks);
			goto inval_out;
		}

		dev_dbg(mmc_dev(host->mmc), "setup data, blk_sz=%d, blk_cnt=0x%x\n",
			mrq->data->blksz, mrq->data->blocks);
		SET_REG(host, ((u16)HOST_DMA_BDRY_MASK << HOST_DMA_BDRY_OFFSET) | mrq->data->blksz,
			SD_BLOCK_SIZE);
		SET_REG(host, mrq->data->blocks, SD_BLOCK_COUNT);

		BUG_ON (host->sg_index || host->pio_sg_pos || host->dma_sg_len);  
		pxa_sdh_setup_data(host);
	}

	pxa_sdh_start_cmd(host, mrq->cmd);
	return;

inval_out:
	mrq->cmd->error = mrq->data->error = -EINVAL;
	pxa_sdh_finish_request(host, host->mrq);
	return;
}

static int pxa_sdh_get_ro(struct mmc_host *mmc)
{
#if 1
	return 0;
#else	
	/*not stable on TTC EVB*/
	struct pxa_sdh_host *host = mmc_priv(mmc);

	if (!pxa_sdh_wait_reset(host) && (GET_REG(host, SD_PRESENT_STAT_2) & CARD_PROT))
		return 1;

	return 0;
#endif
}

static void pxa_sdh_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct pxa_sdh_host *host = mmc_priv(mmc);

	if (pxa_sdh_wait_reset(host))
		return;

	set_dvfm_constraint();
	if (ios->clock) {
		unsigned long rate = host->clkrate;
		unsigned int clk = rate / ios->clock;
		unsigned int shift;

		if (host->clkrt == CLKRT_OFF)
			clk_enable(host->clk);

		BUG_ON((ios->clock > mmc->f_max) || (ios->clock < mmc->f_min));
		if (ios->clock >= host->clkrate) {
			host->clkrt = 0x00;
		} else {
			shift = fls(clk);
			if (rate / clk > ios->clock)
				shift++;
			host->clkrt = 1 << (shift - 2);
		}

		dev_dbg(mmc_dev(mmc),"set clkrt = %08x\n", host->clkrt);
		pxa_sdh_stop_clock(host);
		pxa_sdh_start_clock(host);

		if((host->clkrt == 0 && host->clkrate > 25000000) 
			|| (host->clkrt && (host->clkrate/(host->clkrt*2)) > 25000000)) {
			//this bit should not be set, or sd8688 cannot pass iperf stress
			//SET_REG_BIT(host, HI_SPEED_EN, SD_HOST_CTRL);
			dev_dbg(mmc_dev(mmc), "set as HIGH_SPEED.\n");
		} else
			CLEAR_REG_BIT(host, HI_SPEED_EN, SD_HOST_CTRL);

	} else {
		pxa_sdh_stop_clock(host);
		if (host->clkrt != CLKRT_OFF) {
			host->clkrt = CLKRT_OFF;
			clk_disable(host->clk);
		}
	}

	SET_REG_BITS(host, SDCLK_SEL_OFFSET, SDCLK_SEL_MASK, SDCLK_SEL_INIT_VAL, SD_CLK_BURST_SET);

	if (host->power_mode != ios->power_mode) {
		host->power_mode = ios->power_mode;

		if (host->pdata && host->pdata->setpower)
			host->pdata->setpower(mmc_dev(mmc), ios->vdd);

		if (ios->power_mode == MMC_POWER_ON) {
			SET_REG_BITS(host, SD_BUS_VLT_OFFSET, SD_BUS_VLT_MASK,
				SD_BUS_VLT_18V, SD_HOST_CTRL);
			SET_REG_BIT(host, SD_BUS_POWER, SD_HOST_CTRL);
		}
	}

	if (ios->bus_width == MMC_BUS_WIDTH_8) {
		SET_REG_BIT(host, MMC_CARD, SD_CE_ATA_2);
		SET_REG_BIT(host, DATA_WIDTH_8BIT, SD_CE_ATA_2);
			dev_dbg(mmc_dev(mmc), "set as 8_BIT_MODE.\n");
	} else {
			CLEAR_REG_BIT(host, MMC_CARD, SD_CE_ATA_2);
		CLEAR_REG_BIT(host, DATA_WIDTH_8BIT, SD_CE_ATA_2);
		if (ios->bus_width == MMC_BUS_WIDTH_4) {
			SET_REG_BIT(host, DATA_WIDTH_4BIT, SD_HOST_CTRL);
				dev_dbg(mmc_dev(mmc), "set as 4_BIT_MODE.\n");
		} else {
			CLEAR_REG_BIT(host, DATA_WIDTH_4BIT, SD_HOST_CTRL);
		}
	}
	unset_dvfm_constraint();
}

static void pxa_sdh_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	struct pxa_sdh_host *host = mmc_priv(mmc);

	if (pxa_sdh_wait_reset(host))
		return;

	if (enable) {
		pxa_sdh_enable_irq(host, CARD_INT);
		SET_REG_BIT(host, DIS_PAD_SD_CLK_GATE, SD_FIFO_PARAM); 
	} else {
		pxa_sdh_disable_irq(host, CARD_INT);
		CLEAR_REG_BIT(host, DIS_PAD_SD_CLK_GATE, SD_FIFO_PARAM); 
	}
}

static const struct mmc_host_ops pxa_sdh_ops = {
	.request		= pxa_sdh_request,
	.get_ro			= pxa_sdh_get_ro,
	.set_ios		= pxa_sdh_set_ios,
	.get_cd			= pxa_sdh_get_cd,
	.enable_sdio_irq	= pxa_sdh_enable_sdio_irq,
};

static irqreturn_t pxa_sdh_detect_irq(int irq, void *devid)
{
	struct pxa_sdh_host *host = mmc_priv(devid);
	
	mmc_detect_change(devid, host->pdata->detect_delay);
	return IRQ_HANDLED;
}

static int pxa_sdh_probe(struct platform_device *pdev)
{
	struct mmc_host *mmc;
	struct pxa_sdh_host *host = NULL;
	struct resource *r;
	int ret, irq;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);
	if (!r || irq < 0)
		return -ENXIO;
	r = request_mem_region(r->start, SZ_256, DRIVER_NAME);
	if (!r)
		return -EBUSY;

	mmc = mmc_alloc_host(sizeof(struct pxa_sdh_host), &pdev->dev);
	if (!mmc) {
		ret = -ENOMEM;
		goto out;
	}

	mmc->ops = &pxa_sdh_ops;

	mmc->max_phys_segs = ADMA_DESC_TBL_SIZE / sizeof(struct adma_desc_table);
	mmc->max_hw_segs = ADMA_DESC_TBL_SIZE / sizeof(struct adma_desc_table);

	/*Determined by the 16-bit Length Field of ADMA2 Descriptor Table line*/
	mmc->max_seg_size = (1 << 16); 

	mmc->max_blk_size = BLOCK_SIZE_MAX;

	/*Set as the maximum value of 16-bit SD_BLOCK_COUNT Register*/
	mmc->max_blk_count = (1 << 16) -1;

	mmc->max_req_size = min(mmc->max_blk_size * mmc->max_blk_count,
			mmc->max_seg_size * mmc->max_phys_segs);

	host = mmc_priv(mmc);
	host->mmc = mmc;
	host->pdata = pdev->dev.platform_data;
	host->clkrt = CLKRT_OFF;

	host->clk = clk_get(&pdev->dev, "PXA-SDHCLK");
	if (IS_ERR(host->clk)) {
		ret = PTR_ERR(host->clk);
		host->clk = NULL;
		goto out;
	}
	clk_enable(host->clk);

	host->clkrate = clk_get_rate(host->clk);
	/*
	 * Calculate minimum clock rate, rounding up.
	 */
	mmc->f_min = (host->clkrate + SD_FREQ_SEL_MASK) / (SD_FREQ_SEL_MASK + 1);
	mmc->f_max = host->clkrate;
	if (host->pdata->max_speed)
		mmc->f_max = host->pdata->max_speed;

	mmc->ocr_avail = 0xffffffff; /*host->pdata ?
			 host->pdata->ocr_mask :
			 MMC_VDD_32_33|MMC_VDD_33_34;*/

	mmc->caps = MMC_CAP_SDIO_IRQ |
			MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED;
	if (host->pdata->bus_width == 8)
		mmc->caps |= MMC_CAP_8_BIT_DATA | MMC_CAP_4_BIT_DATA;
	else if ((host->pdata->bus_width == 4) || !host->pdata->bus_width) //default as 4-bit bus
		mmc->caps |= MMC_CAP_4_BIT_DATA;
	else
		dev_info(mmc_dev(mmc), "works as 1-bit mode\n");

	host->desc_tbl_addr = dma_alloc_coherent(mmc_dev(host->mmc), ADMA_DESC_TBL_SIZE, 
			&host->desc_tbl_dma_addr, GFP_KERNEL);
	if (!host->desc_tbl_addr) {
		ret = -ENOMEM;
		goto out;
	}

	spin_lock_init(&host->lock);
	host->res = r;
	host->irq = irq;

	host->base = ioremap(r->start, SZ_256);
	if (!host->base) {
		ret = -ENOMEM;
		goto out;
	}

	pxa_sdh_stop_clock(host);
	pxa_sdh_disable_irq(host, 0xffffffff);
	ret = request_irq(host->irq, pxa_sdh_irq, IRQF_SHARED, DRIVER_NAME, host);
	if (ret)
		goto out;

	platform_set_drvdata(pdev, mmc);

	if (host->pdata && host->pdata->init)
		host->pdata->init(&pdev->dev, pxa_sdh_detect_irq, mmc);

	mmc_add_host(mmc);	
	pxa_sdh_enable_irq(host, CARD_INS | CARD_REM);

	if (host->pdata->mfp_config)
		host->pdata->mfp_config();

	//SET_REG_BIT(host, DIS_PAD_SD_CLK_GATE, SD_FIFO_PARAM); 
	return 0;

 out:
	if (host) {
		if (host->base)
			iounmap(host->base);
		if (host->clk)
			clk_put(host->clk);
	}
	if (host->desc_tbl_addr)
		dma_free_coherent(mmc_dev(host->mmc), PAGE_SIZE, 
			host->desc_tbl_addr, host->desc_tbl_dma_addr);
	if (mmc)
		mmc_free_host(mmc);
	release_mem_region(r->start, SZ_256);
	return ret;
}

static int pxa_sdh_remove(struct platform_device *pdev)
{
	struct mmc_host *mmc = platform_get_drvdata(pdev);
	struct pxa_sdh_host *host;

	platform_set_drvdata(pdev, NULL);

	if (mmc) {
		host = mmc_priv(mmc);

		if (host->pdata && host->pdata->exit)
			host->pdata->exit(&pdev->dev, mmc);

		mmc_remove_host(mmc);

		pxa_sdh_stop_clock(host);
		pxa_sdh_disable_irq(host, 0xffffffff);

		free_irq(host->irq, host);
		iounmap(host->base);
		dma_free_coherent(mmc_dev(host->mmc), PAGE_SIZE, 
			host->desc_tbl_addr, host->desc_tbl_dma_addr);

		clk_put(host->clk);

		release_mem_region(host->res->start, SZ_256);

		mmc_free_host(mmc);
	}
	return 0;
}

#ifdef CONFIG_PM
static int pxa_sdh_suspend(struct platform_device *dev, pm_message_t state)
{
	struct mmc_host *mmc = platform_get_drvdata(dev);
	int ret = 0;

	if (mmc)
		ret = mmc_suspend_host(mmc, state);

	return ret;
}

static int pxa_sdh_resume(struct platform_device *dev)
{
	struct mmc_host *mmc = platform_get_drvdata(dev);
	int ret = 0;

	if (mmc)
		ret = mmc_resume_host(mmc);

	return ret;
}
#else
#define pxa_sdh_suspend	NULL
#define pxa_sdh_resume	NULL
#endif

static struct platform_driver pxa_sdh_driver = {
	.probe		= pxa_sdh_probe,
	.remove		= pxa_sdh_remove,
	.suspend	= pxa_sdh_suspend,
	.resume		= pxa_sdh_resume,
	.driver		= {
		.name	= DRIVER_NAME,
	},
};

static int __init pxa_sdh_init(void)
{
#ifdef CONFIG_DVFM
	dvfm_register("MMC", &(dvfm_lock.dev_idx));
#endif
	return platform_driver_register(&pxa_sdh_driver);
}

static void __exit pxa_sdh_exit(void)
{
	platform_driver_unregister(&pxa_sdh_driver);
#ifdef CONFIG_DVFM
	dvfm_unregister("MMC", &(dvfm_lock.dev_idx));
#endif
}

module_init(pxa_sdh_init);
module_exit(pxa_sdh_exit);

MODULE_DESCRIPTION("PXA SD Host Controller(MM4) Driver");
MODULE_LICENSE("GPL");
