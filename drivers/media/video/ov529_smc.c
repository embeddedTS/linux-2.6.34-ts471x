/*
 *  linux/drivers/media/video/ov529_smc.c - ov529 SMC interface driver.
 *
 *  Copyright:	(C) Copyright 2009 Marvell International Ltd.
 *              Weili Xia <wlxia@marvell.com>
 *
 * An abstraction layer which describes how SMC interface controls ov529.
 *
 * Written by Weili Xia, wlxia@marvell.com.
 *
 * This file may be distributed under the terms of the GNU General
 * Public License, version 2.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/list.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/jiffies.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <linux/clk.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <mach/ov529.h>
#include <asm/mach-types.h>
#include <plat/dma.h>

#include "ov529_hw_ops.h"
#include "pxa168_camera.h"

static struct ov529_platform_data *ov529_platform_ops;
static unsigned char __iomem *smc_addr;
static unsigned long smc_phy_addr;
struct clk *smc_clk;

#ifdef SMC_USE_DMA
static int smc_dma;
static void *rxdma_addr;
static dma_addr_t rxdma_addr_phys;
static wait_queue_head_t dma_wait;
static int dma_finish_flags;

/* smc dma operation */
static void pxa_smc_transmit_dma_start(int count)
{
	if (!(DCSR(smc_dma) & DCSR_STOPSTATE))
		return;

	DALGN |= 1 << smc_dma;

	DCSR(smc_dma)  = DCSR_NODESC;
	DSADR(smc_dma) = smc_phy_addr;
	DTADR(smc_dma) = rxdma_addr_phys;
	DCMD(smc_dma) = DCMD_INCTRGADDR | DCMD_ENDIRQEN | count;
	DCSR(smc_dma) |= DCSR_RUN;
}

static void pxa_smc_dma_irq(int channel, void *data)
{
	volatile unsigned long dcsr;

	DCSR(channel) &= ~DCSR_RUN;
	dcsr = DCSR(channel);
	/* printk(KERN_ERR "%s, %d", __func__, __LINE__); */
	if (dcsr & DCSR_BUSERR) {
		DCSR(channel) |= DCSR_BUSERR;
		printk(KERN_ERR "%s(): DMA channel bus error\n", __func__);
		dma_finish_flags = 2;
	}

	if ((dcsr & DCSR_ENDINTR) || (dcsr & DCSR_STOPSTATE)) {
		if (dcsr & DCSR_ENDINTR)
			DCSR(channel) |= DCSR_ENDINTR;
		if (dcsr & DCSR_STOPSTATE)
			DCSR(channel) &= ~DCSR_STOPSTATE;
		dma_finish_flags = 1;
	}
	/* printk(KERN_ERR "wake up dma_wait\n"); */
	wake_up(&dma_wait);

	return;
}
#endif

static void ov529_prepare_cmd(struct ov529_cmd *cmd)
{
	if (cmd) {
		memset(cmd, 0x0, sizeof(*cmd));
		cmd->ff1 = cmd->ff2 = cmd->ff3 = 0xff;
	}
}

static int ov529_smc_send_cmd(int id, int p1, int p2, int p3, int p4)
{
	struct ov529_cmd cmd, resp;
	int i, j;

	ov529_prepare_cmd(&cmd);
	cmd.id = id;
	cmd.p1 = p1;
	cmd.p2 = p2;
	cmd.p3 = p3;
	cmd.p4 = p4;
	memset(&resp, 0x0, sizeof(resp));

	for (i = 0; i < 8; i++) {
		*smc_addr = *((unsigned char *)&cmd + i);
	}
	msleep(3);
	for (i = 0; i < 8; i++) {
		*((unsigned char *)&resp + i) = *smc_addr;
	}

	printk(KERN_DEBUG "%s, %d: resp.id=%d, resp.p1=%d\n", __func__, __LINE__, resp.id, resp.p1);
	if ((resp.id == ACK) && (resp.p1 == cmd.id)) {
		printk(KERN_ERR "cmd 0x%X ack\n", resp.p1);
		return 0;
	} else {
		if (resp.id == NACK) {
			printk(KERN_ERR "NAK counter: %d\n", resp.p2);
			printk(KERN_ERR "Error number: 0x%X\n", resp.p3);
		}
		return -EINVAL;
	}
}

static int ov529_smc_data_ready(void)
{
	return ov529_platform_ops->get_sel_uart();
}

static int ov529_smc_streamon(void)
{
	int ret = 0;
	/*
	 * Send cmd GET PICTURE to ov529.
	 */
	/* ov529_platform_ops->turnon_sensor(); */

	ov529_platform_ops->set_pcd(PCD_CMD);
	ret = ov529_smc_send_cmd(GET_PIC, COMPRESSION_PREV_PIC, 0, 0, 0);
	ov529_platform_ops->set_pcd(PCD_DATA);
	msleep(1000);
	return ret;
}

static int ov529_smc_recv_img(char *buf, int size)
{
	int i;
	int ret = 0;
#ifdef SMC_USE_DMA
	char *dma_buf = (char *)rxdma_addr;
#endif

	ov529_platform_ops->set_rts(0);

#ifndef SMC_USE_DMA
	for (i = 0; i < size; i++) {
		*(buf + i) = *smc_addr;
	}
#else
	//printk(KERN_ERR "%s, %d\n", __func__, __LINE__);
	/* kick off dma transfer */
	pxa_smc_transmit_dma_start(size);
	/* wait on dma finish */
	dma_finish_flags = 0;
	if (wait_event_interruptible(dma_wait, dma_finish_flags)) {
		ret = -ERESTARTSYS;
		goto out;
	}
	memcpy(buf, dma_buf, size);
	printk(KERN_ERR "0x%X, 0x%X, 0x%X, 0x%X\n", buf[0], buf[1], buf[2], buf[3]);
	printk(KERN_ERR "0x%X, 0x%X, 0x%X, 0x%X\n", buf[8], buf[9], buf[10], buf[11]);
	if (dma_finish_flags != 1)
		ret = -EINVAL;
#endif
out:
	ov529_platform_ops->set_rts(1);
	return ret;
}

static int ov529_smc_startup(void)
{
	int ret = 0;

	/* switch the clock freq of smc from 31.2 MHz to 62.4 MHz */
	clk_enable(smc_clk);

	ov529_platform_ops->set_ptype(1);
	msleep(10);

	/* FIXME: is this necessary */
	ccic_set_clock_parallel();
	ccic_enable_vclk();

	/* After power on reset, it will take ov529 more than
	 * one second to finish initialization*/
	/* ov529_platform_ops->turnon_sensor(); */
	msleep(10);
	ov529_platform_ops->power_on(1);
	msleep(1500);
	ov529_platform_ops->set_pcd(PCD_CMD);

	/* send SYNC and INITIAL commands to ov529 */
	ret = ov529_smc_send_cmd(SYNC, 0, 0, 0, 0);
	if (ret < 0) {
		printk(KERN_ERR "%s, %d\n", __func__, __LINE__);
		goto err;
	}
	msleep(100);

	ret = ov529_smc_send_cmd(INITIAL, 0, 0x87, 0, 0x07);

err:
	return ret;
}

static int ov529_smc_init(struct platform_device *pdev, struct resource *res)
{
	struct ov529_platform_data *data = pdev->dev.platform_data;

	if (data == NULL)
		return -EINVAL;

	ov529_platform_ops = data;
	printk(KERN_ERR "ov529 intf's name: %s\n", ov529_platform_ops->name);
	if (ov529_platform_ops->init() < 0)
		return -EIO;

	ov529_platform_ops->power_on(1);
	msleep(10);
	/* Put ov529 to reset */
	ov529_platform_ops->power_on(0);
	/* msleep(300); */

	/* record the physical address for dma */
	smc_phy_addr = res->start;
	smc_addr = ioremap_nocache(res->start, 0x1);
	if (!smc_addr) {
		printk(KERN_ERR "Unable to ioremap ov529 io mem\n");
		ov529_platform_ops->release();
		return -EINVAL;
	}

	/* Tune smc freq to 62.4 MHz */
	smc_clk = clk_get(&pdev->dev, "SMCCLK");
	if (IS_ERR(smc_clk)) {
		dev_err(&pdev->dev, "unable to get SMCCLK");
		return PTR_ERR(smc_clk);
	}

#ifdef SMC_USE_DMA
	init_waitqueue_head(&dma_wait);

	smc_dma = pxa_request_dma("pxa-smc", DMA_PRIO_HIGH, pxa_smc_dma_irq, NULL);
	if (smc_dma < 0) {
		printk(KERN_ERR "error requesting smc dma\n");
		return -EBUSY;
	}
	printk(KERN_ERR "smc dma channel %d\n", smc_dma);

	rxdma_addr = dma_alloc_coherent(NULL, 4096, &rxdma_addr_phys, GFP_KERNEL);
	if (!rxdma_addr) {
		printk(KERN_ERR "dma_alloc_coherent failed\n");
		pxa_free_dma(smc_dma);
		return -EINVAL;
	}
#endif

	return 0;
}

static int ov529_smc_remove(void)
{
	/* power off ov529 */
	ov529_platform_ops->power_on(0);

	/* unmap remapped address */
	/* iounmap((void *)smc_addr); */
	/* release gpio */
	/* ov529_platform_ops->release(); */

	return 0;
}

/*
 * SMC specific functions for
 * ov529 hardware level operations.
 */
struct ov529_hw_ops ov529_smc_ops = {
	.name = "smc",
	.init = ov529_smc_init,
	.remove = ov529_smc_remove,
	.startup = ov529_smc_startup,
	.data_ready = ov529_smc_data_ready,
	.streamon = ov529_smc_streamon,
	.recv_data = ov529_smc_recv_img,
	.send_cmd = ov529_smc_send_cmd,
};






