/*
 * pxa168_cf.c -- Aspen CompactFlash Host controller driver
 * Copyright (c) 2008 Marvell International Ltd. (kvedere@marvell.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/hardirq.h>
#include <linux/jiffies.h>
#include <linux/slab.h>

#include <pcmcia/ss.h>

#include <asm/io.h>
#include <asm/sizes.h>
#include <mach/irqs.h>
#include <mach/regs-apmu.h>
#include <mach/gpio.h>
#include "pxa168_cf.h"

static const char driver_name[] = "pxa168-cf";
struct device *dev;

#define BUFFER_AVAIL_TIMEOUT (2*HZ/10)
#define TRANSFER_DONE_TIMEOUT (2*HZ/10)
#define CARD_READY_TIMEOUT (2*HZ/10)
#define CONFIG_CF_LOCK 1

#define GPIO_CF_nCD1 32
#define GPIO_CF_nCD2 33

struct pxa168_cf_socket {
	struct pcmcia_socket	socket;
	struct resource		*res;
	struct clk		*clk;
	unsigned		present:1;
	unsigned		active:1;
	struct platform_device	*pdev;
	unsigned long		phys_baseaddr;
	void __iomem		*base;
	u_int			irq;
	spinlock_t		lock;
};

static void __iomem *cf_base;
static spinlock_t *cf_lock;
static struct completion transfer_done, buffer_avail, card_ready;
volatile int transf_done, buf_avail, card_rdy;

static inline int is_card_ready(void __iomem *cf_base);

static int pxa168_cf_present(struct pxa168_cf_socket *cf)
{
	int ret;
#ifndef CONFIG_PXA168_CF_USE_GPIO_CARDDETECT
	u32 cfi_reg;
	cfi_reg = readl(cf->base + CFI_SR_OFFSET);
	dev_dbg(dev,"Card Present = 0x%x 0x%x\n",cfi_reg, 
		~((cfi_reg & CFI_SR_CARD_DETECT_1) >> 2));
	return !(cfi_reg & CFI_SR_CARD_DETECT_1);
#else
	if (gpio_request(GPIO_CF_nCD1, "CF_nCD1")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", GPIO_CF_nCD1);
		return -EIO;
	}
	if (gpio_request(GPIO_CF_nCD2, "CF_nCD2")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", GPIO_CF_nCD2);
		return -EIO;
	}
	if (gpio_get_value(GPIO_CF_nCD1) && gpio_get_value(GPIO_CF_nCD2))
		ret = 0;
	else
		ret = 1;
	gpio_free(GPIO_CF_nCD1);	
	gpio_free(GPIO_CF_nCD2);
	return ret;	
#endif
}

#if 0 /* unused */
static int pxa168_cf_ready(struct pxa168_cf_socket *cf)
{
	u32 cfi_reg;	
	cfi_reg = readl(cf->base + CFI_SR_OFFSET);
	return ((cfi_reg & CFI_SR_CARD_READY) >> 5);
}
#endif

/* Card Ready Interrupt doesnot occur for some
 * reason. (May be routine called with interrupts
 * disabled). Using polling mode to getaround this
 * This routine will be called only once after card is 
 * inserted. So this shouldn't be bad.
 */

static int pxa168_cf_reset(struct pxa168_cf_socket *cf)
{
	u32 opmode, ret;
	
	opmode = readl(cf->base + CF_OPMODE_OFFSET);
	opmode = opmode | CF_OPMODE_CARD_RESET;
	writel(opmode, cf->base + CF_OPMODE_OFFSET);
	udelay(5);
	opmode = opmode & ~(CF_OPMODE_CARD_RESET);
	writel(opmode, cf->base + CF_OPMODE_OFFSET);
	ret = is_card_ready(cf->base);
	if(!ret) {
		printk(KERN_ERR "%s: Card Ready timeout\n",__func__);
		return 0;
	}
	return 1;
}	

/* When the process is in SOFT_IRQ Context interrupts seem
 * to be disabled in some cases. So we use polling to monitor 
 * if each of the events have occured
 */
static inline int is_buffer_avail(void __iomem *cf_base)
{
	int ret=1, count=10000;
	while(!(readl(cf_base + CF_IRQ_OFFSET) & CF_IRQ_BUFF_AVAL_IRQ) && count--);
	if(!count)
		ret = 0;
	writel(CF_IRQ_BUFF_AVAL_IRQ, cf_base + CF_IRQ_OFFSET);
	return ret;
}

static inline int is_transfer_done(void __iomem *cf_base)
{
	int ret=1, count = 10000;
	while(!(readl(cf_base + CF_IRQ_OFFSET) & CF_IRQ_TRANS_DONE_IRQ) && count--);
	if(!count)
		ret = 0;
	writel(CF_IRQ_TRANS_DONE_IRQ, cf_base + CF_IRQ_OFFSET);
	return ret;
}

static inline int is_card_ready(void __iomem *cf_base)
{
	int ret=1, count = 10000;
	while(!(readl(cf_base + CF_IRQ_OFFSET) & CF_IRQ_MEMO_MODE_READY) && count--);
	if(!count)
		ret = 0;
	writel(CF_IRQ_MEMO_MODE_READY, cf_base + CF_IRQ_OFFSET);
	return ret;
}

/**************** pccard_operations.init ***********************************/

static int pxa168_cf_ss_init(struct pcmcia_socket *s)
{
	return 0;
}

static int pxa168_cf_get_status(struct pcmcia_socket *sock, u_int *sp)
{
	struct pxa168_cf_socket	*cf;
        cf = container_of(sock, struct pxa168_cf_socket, socket);

	if (!sp)
		return -EINVAL;
	if (cf->present) {
		*sp = SS_DETECT | SS_3VCARD | SS_POWERON;
		
		if (cf->active)
			*sp |= SS_READY;
	} else
		*sp = 0;
	return 0;
}

static int pxa168_cf_set_socket(struct pcmcia_socket *sock, 
		struct socket_state_t *s)
{
	struct pxa168_cf_socket   *cf;

        cf = container_of(sock, struct pxa168_cf_socket, socket);
	dev_dbg(dev,"%s: Vcc= %d\n",driver_name, s->Vcc);

	switch (s->Vcc) {
	case 0:
	case 33:
		break;
	default:
		return -EINVAL;
	}

	if (s->flags & SS_RESET)
	{
		cf->active = pxa168_cf_reset(cf);
		/* If Card Reset fails, issue warning
 		 * and set cf->active to 1 to proceed 
 		 * further with card initialization.
 		 */
		printk("%s:CF Card Reset %s\n",__func__,
			cf->active?"Successfull":"Failed");
		cf->active = 1;  
	}

	dev_dbg(dev,"%s: Vcc %d, io_irq %d, flags %04x csc %04x\n",
		driver_name, s->Vcc, s->io_irq, s->flags, s->csc_mask);

	return 0;
}

#ifdef CONFIG_PM
static int pxa168_cf_ss_suspend(struct pcmcia_socket *s)
{
	dev_dbg(dev,"%s: %s\n", driver_name, __FUNCTION__);
	return pxa168_cf_set_socket(s, &dead_socket);
}
#else
#define pxa168_cf_ss_suspend NULL
#endif

/* PXA168 Platform doesnot have an IO-Map for the CF IO
 * Memory Space. We use the CF Host Controller to read/write
 * to the IO Memory Space
 */
static int pxa168_cf_set_io_map(struct pcmcia_socket *s, 
		struct pccard_io_map *io)
{
	struct pxa168_cf_socket	*cf;

	cf = container_of(s, struct pxa168_cf_socket, socket);
	io->flags &= (MAP_ACTIVE | MAP_16BIT | MAP_AUTOSZ);
	io->start = cf->socket.io_offset;
	io->stop = io->start + SZ_2K - CF_IO_BASE_OFFSET - 1;
	return 0;
}

/* PXA168 Platform doesnot support Memory-Mapping the CF Common/
 * Attrib. Memory Space. We use the CF Host Controller to read/write
 * to the these Memory Spaces
 */
static int pxa168_cf_set_mem_map(struct pcmcia_socket *s, 
		struct pccard_mem_map *map)
{
	struct pxa168_cf_socket	*cf;

	cf = container_of(s, struct pxa168_cf_socket, socket);
	map->static_start = (unsigned long) cf->base;
        return 0;
}

static struct pccard_operations pxa168_cf_ops = {
	.init			= pxa168_cf_ss_init,
	.suspend		= pxa168_cf_ss_suspend,
	.get_status		= pxa168_cf_get_status,
	.set_socket		= pxa168_cf_set_socket,
	.set_io_map		= pxa168_cf_set_io_map,
	.set_mem_map		= pxa168_cf_set_mem_map,
};

/*---- PXA168 CF Memory Read/Write Routines -----*/

void pxa168_cf_mem_writeb(u8 attr_mem_transfer, u8 val, u32 addr)
{
	u32 tmp, trans_ctrl, intr_en, ret, retries=5;

#ifdef CONFIG_CF_LOCK
	spin_lock_bh(cf_lock);
#endif
	attr_mem_transfer = attr_mem_transfer?1:0;
	if (!in_softirq())
		INIT_COMPLETION(transfer_done);
	else {
		dev_dbg(dev,"%s: In SoftIRQ Context\n",__func__);
		intr_en = readl(cf_base + CF_INTEN_OFFSET);
		intr_en = intr_en & ~(CF_INTEN_TRANS_DONE_IRQ | 
			CF_INTEN_BUFF_AVAL_IRQ);
	}	
	/* Program Transfer Address Register with Common/Attrib. Mem Addr. */
retry_again:
	writel((addr & 0x7FF), cf_base + CF_TADDR_OFFSET);
	trans_ctrl = ((CF_TCNTR_MEM_TRANS_START) |
		      (CF_TCNTR_TRANS_DIR_WRITE) |
		      (CF_TCNTR_COMMON_ATTR_MEM &(attr_mem_transfer<<26)) |
		      (CF_TCNTR_DISABLE_ADDR_INC) |
		      (CF_TCNTR_COUNT_MASK & 0x1));
	writel(trans_ctrl, cf_base + CF_TCNTR_OFFSET);
	/* Write value to Host Controller WriteFIFO */	
	writel((u32) val, cf_base + CF_WDPR_OFFSET);
	dev_dbg(dev,"%s: Before Transfer: intr_en = 0x%x addr = 0x%x \
		trans_ctrl_reg=0x%x val=0x%x\n",__func__, 
		readl(cf_base + CF_INTEN_OFFSET), addr, 
		readl(cf_base+CF_TCNTR_OFFSET),val);
	if (in_softirq()) {
		ret = is_transfer_done(cf_base);
	} else {
		ret = wait_for_completion_timeout(&transfer_done,
			TRANSFER_DONE_TIMEOUT);
	}
	if(!ret) {
		if(retries--) {
			writel(trans_ctrl & ~(CF_TCNTR_MEM_TRANS_START), 
				cf_base + CF_TCNTR_OFFSET);
			goto retry_again;
		}
		printk(KERN_WARNING "%s: Transfer Done timeout\n",__func__);
		if (!in_softirq())
			complete(&transfer_done);
	}
	/* Disable Transfer Start Bit Explicitly */
	tmp = readl(cf_base + CF_TCNTR_OFFSET);
	writel(tmp & ~(CF_TCNTR_MEM_TRANS_START), cf_base + CF_TCNTR_OFFSET);
	if (in_softirq()) {
		intr_en = readl(cf_base + CF_INTEN_OFFSET);
		intr_en = intr_en | CF_INTEN_TRANS_DONE_IRQ | 
			CF_INTEN_BUFF_AVAL_IRQ;
	}
#ifdef CONFIG_CF_LOCK
	spin_unlock_bh(cf_lock);
#endif
	dev_dbg(dev,"%s: After Transfer: intr_en = 0x%x addr = 0x%x \
		trans_ctrl_reg=0x%x\n",__func__, 
		readl(cf_base + CF_INTEN_OFFSET), addr, 
		readl(cf_base + CF_TCNTR_OFFSET));
}

u8 pxa168_cf_mem_readb(u8 attr_mem_transfer, u32 addr)
{
	u32 val, tmp, trans_ctrl, intr_en, ret, retries=5;

#ifdef CONFIG_CF_LOCK
	spin_lock_bh(cf_lock);
#endif
	attr_mem_transfer = attr_mem_transfer?1:0;
	if (!in_softirq()) {
		INIT_COMPLETION(transfer_done);
		INIT_COMPLETION(buffer_avail);
	} else { 
		dev_dbg(dev,"%s: In SoftIRQ Context\n",__func__);
		intr_en = readl(cf_base + CF_INTEN_OFFSET);
		intr_en = intr_en & ~(CF_INTEN_TRANS_DONE_IRQ | 
			CF_INTEN_BUFF_AVAL_IRQ);
	}
retry_again:
        writel((addr & 0x7FF), cf_base + CF_TADDR_OFFSET);
	/* Program Transfer Address Register with Common/Attrib. Mem Addr. */
	trans_ctrl = ((CF_TCNTR_MEM_TRANS_START) |
		      (CF_TCNTR_COMMON_ATTR_MEM &(attr_mem_transfer<<26)) |
		      (CF_TCNTR_DISABLE_ADDR_INC) |
		      (CF_TCNTR_COUNT_MASK & 0x1));
	writel(trans_ctrl, cf_base + CF_TCNTR_OFFSET);
	dev_dbg(dev,"%s: Before Transfer: intr_en = 0x%x addr = 0x%x \
		trans_ctrl_reg=0x%x\n",__func__, 
		readl(cf_base + CF_INTEN_OFFSET), 
		addr, readl(cf_base+CF_TCNTR_OFFSET));
	if (in_softirq()) {
		ret = is_buffer_avail(cf_base);
	} else {
		ret = wait_for_completion_timeout(&buffer_avail, 
			BUFFER_AVAIL_TIMEOUT);
	}
	if(!ret) {
		if(retries--) {
        		val = readl(cf_base + CF_RDPR_OFFSET);
			writel(trans_ctrl & ~(CF_TCNTR_MEM_TRANS_START), 
				cf_base + CF_TCNTR_OFFSET);
			goto retry_again;
		}
		printk(KERN_WARNING "%s: Buffer Available timeout\n",
			__func__);
		if (!in_softirq())
			complete(&buffer_avail);
		val = 0x0;
	} else {
		/* Read value from Host Controller ReadFIFO */
        	val = readl(cf_base + CF_RDPR_OFFSET);
	}
	if (in_softirq()) {
		ret = is_transfer_done(cf_base);
	} else {
		ret = wait_for_completion_timeout(&transfer_done, TRANSFER_DONE_TIMEOUT);
	}
	if(!ret) {
                dev_dbg(dev,"%s: Transfer Done timeout\n",__func__);
		if (!in_softirq())
                	complete(&transfer_done);
	}
	tmp = readl(cf_base + CF_TCNTR_OFFSET);
	writel(tmp & ~(CF_TCNTR_MEM_TRANS_START), cf_base + CF_TCNTR_OFFSET);
	dev_dbg(dev,"%s: After Transfer: intr_en = 0x%x addr = 0x%x\
		trans_ctrl_reg=0x%x val=0x%x\n",__func__, 
		readl(cf_base + CF_INTEN_OFFSET), addr, 
		readl(cf_base + CF_TCNTR_OFFSET),val);
	if (in_softirq()) { 
		intr_en = readl(cf_base + CF_INTEN_OFFSET);
		intr_en = intr_en | CF_INTEN_TRANS_DONE_IRQ | 
			CF_INTEN_BUFF_AVAL_IRQ;
	}
#ifdef CONFIG_CF_LOCK
	spin_unlock_bh(cf_lock);
#endif
	return (u8) val & 0xFF;
}

void pxa168_cf_mem_writew(u8 attr_mem_transfer, u16 val, u32 addr)
{
	u32 tmp, trans_ctrl, intr_en, ret, retries=5;

#ifdef CONFIG_CF_LOCK
	spin_lock_bh(cf_lock);
#endif
	attr_mem_transfer = attr_mem_transfer?1:0;
	if (!in_softirq())
		INIT_COMPLETION(transfer_done);
	else {
		dev_dbg(dev,"%s: In SoftIRQ Context\n",__func__);
		intr_en = readl(cf_base + CF_INTEN_OFFSET);
		intr_en = intr_en & ~(CF_INTEN_TRANS_DONE_IRQ | 
			CF_INTEN_BUFF_AVAL_IRQ);
	}
retry_again:
	/* Program Transfer Address Register with Common/Attrib. Mem Addr. */
	writel((addr & 0x7FF), cf_base + CF_TADDR_OFFSET);
	trans_ctrl = ((CF_TCNTR_MEM_TRANS_START) |
		      (CF_TCNTR_TRANS_DIR_WRITE) |
		      (CF_TCNTR_COMMON_ATTR_MEM &(attr_mem_transfer<<26)) |
		      (CF_TCNTR_TRANS_SIZE_WORD) |
		      (CF_TCNTR_DISABLE_ADDR_INC)|
		      (CF_TCNTR_COUNT_MASK & 0x2));
	writel(trans_ctrl, cf_base + CF_TCNTR_OFFSET);
	/* Write value to Host Controller WriteFIFO */	
	writel((u32) val, cf_base + CF_WDPR_OFFSET);
	dev_dbg(dev,"%s: Before Transfer: intr_en = 0x%x addr = 0x%x\
		trans_ctrl_reg=0x%x val=0x%x\n",__func__, 
		readl(cf_base + CF_INTEN_OFFSET), addr, 
		readl(cf_base+CF_TCNTR_OFFSET),val);
	if (in_softirq()) {
		ret = is_transfer_done(cf_base);
	} else {
		ret = wait_for_completion_timeout(&transfer_done,
			TRANSFER_DONE_TIMEOUT);
	}
	if(!ret) {
		if(retries--) {
			writel(trans_ctrl & ~(CF_TCNTR_MEM_TRANS_START), 
				cf_base + CF_TCNTR_OFFSET);
			goto retry_again;
		}
		printk(KERN_WARNING "%s: Transfer Done timeout\n",__func__);
		if (!in_softirq())
			complete(&transfer_done);
	}
	/* Disable Transfer Start Bit */
	tmp = readl(cf_base + CF_TCNTR_OFFSET);
	writel(tmp & ~(CF_TCNTR_MEM_TRANS_START), cf_base + CF_TCNTR_OFFSET);
	dev_dbg(dev,"%s: After Transfer: intr_en = 0x%x addr = 0x%x\
		trans_ctrl_reg=0x%x\n",__func__, 
		readl(cf_base + CF_INTEN_OFFSET), addr,
		readl(cf_base+CF_TCNTR_OFFSET));
	if (in_softirq()) {
		intr_en = readl(cf_base + CF_INTEN_OFFSET);
		intr_en = intr_en | CF_INTEN_TRANS_DONE_IRQ | 
			CF_INTEN_BUFF_AVAL_IRQ;
	}
#ifdef CONFIG_CF_LOCK
	spin_unlock_bh(cf_lock);
#endif
}

u32 pxa168_cf_mem_readw(u8 attr_mem_transfer, u32 addr)
{
	u32 val, tmp, trans_ctrl, intr_en, ret, retries=5;

#ifdef CONFIG_CF_LOCK
	spin_lock_bh(cf_lock);
#endif
	attr_mem_transfer = attr_mem_transfer?1:0;
	if (!in_softirq()) {
		INIT_COMPLETION(transfer_done);
		INIT_COMPLETION(buffer_avail);
	} else {
		dev_dbg(dev,"%s: In SoftIRQ Context\n",__func__);
		intr_en = readl(cf_base + CF_INTEN_OFFSET);
		intr_en = intr_en & ~(CF_INTEN_TRANS_DONE_IRQ | 
			CF_INTEN_BUFF_AVAL_IRQ);
	}
retry_again:
        writel((addr & 0x7FF), cf_base + CF_TADDR_OFFSET);
	/* Program Transfer Address Register with Common/Attrib. Mem Addr. */
	trans_ctrl = ((CF_TCNTR_MEM_TRANS_START) |
		      (CF_TCNTR_COMMON_ATTR_MEM &(attr_mem_transfer<<26)) |
		      (CF_TCNTR_TRANS_SIZE_WORD) |
		      (CF_TCNTR_DISABLE_ADDR_INC) |
		      (CF_TCNTR_COUNT_MASK & 0x2));
	writel(trans_ctrl, cf_base + CF_TCNTR_OFFSET);
	dev_dbg(dev,"%s: Before Transfer: intr_en = 0x%x addr = 0x%x\
		trans_ctrl_reg=0x%x\n",__func__, 
		readl(cf_base + CF_INTEN_OFFSET), addr, 
		readl(cf_base+CF_TCNTR_OFFSET));
	if (in_softirq()) {
		ret = is_buffer_avail(cf_base);
	} else {
		ret = wait_for_completion_timeout(&buffer_avail, 
			BUFFER_AVAIL_TIMEOUT);
	}
	if(!ret) {
		if(retries--) {
        		val = readl(cf_base + CF_RDPR_OFFSET);
			writel(trans_ctrl & ~(CF_TCNTR_MEM_TRANS_START), 
				cf_base + CF_TCNTR_OFFSET);
			goto retry_again;
		}
		printk(KERN_WARNING "%s: Buffer Available timeout\n",
			__func__);
		if (!in_softirq())
			complete(&buffer_avail);
		val = 0x0;
	} else {
		/* Read value from Host Controller ReadFIFO */
        	val = readl(cf_base + CF_RDPR_OFFSET);
	}
	if (in_softirq()) {
		ret = is_transfer_done(cf_base);
	} else {
		ret = wait_for_completion_timeout(&transfer_done, 
			TRANSFER_DONE_TIMEOUT);
	}
	if(!ret) {
                dev_dbg(dev,"%s: Transfer Done timeout\n",__func__);
		if (!in_softirq())
                	complete(&transfer_done);
	}
	/* Disable Transfer Start Bit */
	tmp = readl(cf_base + CF_TCNTR_OFFSET);
	writel(tmp & ~(CF_TCNTR_MEM_TRANS_START), cf_base + CF_TCNTR_OFFSET);
	dev_dbg(dev,"%s: After Transfer: intr_en = 0x%x addr = 0x%x \
		trans_ctrl_reg=0x%x val=0x%x\n",__func__, 
		readl(cf_base + CF_INTEN_OFFSET), addr, 
		readl(cf_base+CF_TCNTR_OFFSET),val);
	if (in_softirq()) {
		intr_en = readl(cf_base + CF_INTEN_OFFSET);
		intr_en = intr_en | CF_INTEN_TRANS_DONE_IRQ | 
			CF_INTEN_BUFF_AVAL_IRQ;
	}
#ifdef CONFIG_CF_LOCK
	spin_unlock_bh(cf_lock);
#endif
	return val & 0xFFFF;
}

void pxa168_cf_mem_write(u8 attr_mem_transfer, u8* buf, u32 addr, u32 words)
{
	u32 tmp, trans_ctrl, val, intr_en, ret;
	u32 buf_len = words*2; /* Length of buffer in bytes */
	u32 len, trans_len;
	u32 ext_wp_offset;

	if(words<1) {
		printk(KERN_WARNING "%s: Invalid word count\n",__func__);
		return;
	}
	dev_dbg(dev,"%s: addr=0x%x words=0x%x\n",__func__, addr, words);
#ifdef CONFIG_CF_LOCK
	spin_lock_bh(cf_lock);
#endif
	attr_mem_transfer = attr_mem_transfer?1:0;
	if (in_softirq()) {
		dev_dbg(dev,"%s: In SoftIRQ Context\n",__func__);
		intr_en = readl(cf_base + CF_INTEN_OFFSET);
		intr_en = intr_en & ~(CF_INTEN_TRANS_DONE_IRQ | 
			CF_INTEN_BUFF_AVAL_IRQ);
	}
	writel((addr & 0x7FF), cf_base + CF_TADDR_OFFSET);
	while(buf_len)
	{
		if (!in_softirq())
			INIT_COMPLETION(transfer_done);
		len = (buf_len > 0x20000) ? 0x20000 : buf_len;
		trans_ctrl = ((CF_TCNTR_MEM_TRANS_START) |
			      (CF_TCNTR_TRANS_DIR_WRITE) |
		      (CF_TCNTR_COMMON_ATTR_MEM &(attr_mem_transfer<<26)) |
			      (CF_TCNTR_TRANS_SIZE_WORD) |
			      (CF_TCNTR_DISABLE_ADDR_INC)|
			      (CF_TCNTR_COUNT_MASK & len));
		writel(trans_ctrl, cf_base + CF_TCNTR_OFFSET);
		buf_len = buf_len - len;
		dev_dbg(dev,"%s: len=%d trans_ctrl=0x%x",__func__,len,trans_ctrl);
		do {
			if (!in_softirq())
				INIT_COMPLETION(buffer_avail);
			trans_len = (len>512)?512:len;
			ext_wp_offset=CF_EXT_WDPR_OFFSET;
			len = len - trans_len;
			while(trans_len) {
				if(trans_len==2) {
					val = ((buf[1]<<8)|(buf[0]));
					buf+= 2;
					trans_len = 0;
				} else {
					val = ((buf[3]<<24)|(buf[2]<<16)|
						(buf[1]<<8)|(buf[0]));
					buf+= 4;
					trans_len = trans_len - 4;
				}
				writel((u32) val, cf_base + ext_wp_offset);
				ext_wp_offset+=4;
			}
			if (!len)
				break;
			if (in_softirq()) {
				ret = is_buffer_avail(cf_base);
			} else {
				ret = wait_for_completion_timeout(&buffer_avail,BUFFER_AVAIL_TIMEOUT);
			}
			if(!ret) {
				printk(KERN_WARNING "%s: Buffer Available timeout\n",
					__func__);
				if (!in_softirq()) {
					dev_dbg(dev,"Process Context\n");
					complete(&buffer_avail);
				}
			}
		} while (len);
		if (in_softirq()) {
			ret = is_transfer_done(cf_base);
		} else {
			ret = wait_for_completion_timeout(&transfer_done, 
				TRANSFER_DONE_TIMEOUT);
		}
		if(!ret) {
			if(buf_len)
        	        	printk(KERN_WARNING "%s: Transfer Done timeout\n",__func__);
			if (!in_softirq()) {
				dev_dbg(dev,"Process Context\n");
                		complete(&transfer_done);
			}
		}
	}
	tmp = readl(cf_base + CF_TCNTR_OFFSET);
	/* Disable Transfer Start Bit explicitly */
	writel(tmp & ~(CF_TCNTR_MEM_TRANS_START), cf_base + CF_TCNTR_OFFSET);
	if (in_softirq()) {
		intr_en = readl(cf_base + CF_INTEN_OFFSET);
		intr_en = intr_en | CF_INTEN_TRANS_DONE_IRQ | 
			CF_INTEN_BUFF_AVAL_IRQ;
	}
#ifdef CONFIG_CF_LOCK
	spin_unlock_bh(cf_lock);
#endif
}

void pxa168_cf_mem_read(u8 attr_mem_transfer, u8* buf, u32 addr, u32 words)
{
	u32 tmp, val, trans_ctrl, intr_en, ret;
	u32 buf_len = words*2; /* Length of buffer in bytes */
	u32 len, trans_len;
	u32 ext_rp_offset;

	if(words<1) {
		printk(KERN_WARNING "%s: Invalid word count\n",__func__);
		return;
	}
	dev_dbg(dev,"%s: addr=0x%x words=0x%x\n",__func__, addr, words);
#ifdef CONFIG_CF_LOCK
	spin_lock_bh(cf_lock);
#endif
	attr_mem_transfer = attr_mem_transfer?1:0;
	if (in_softirq()) {
		dev_dbg(dev,"%s: In SoftIRQ Context\n",__func__);
		intr_en = readl(cf_base + CF_INTEN_OFFSET);
		intr_en = intr_en & ~(CF_INTEN_TRANS_DONE_IRQ | 
			CF_INTEN_BUFF_AVAL_IRQ);
	}
	writel((addr & 0x7FF), cf_base + CF_TADDR_OFFSET);
	if (!in_softirq())
		INIT_COMPLETION(buffer_avail);
	while(buf_len)
	{
		if (!in_softirq())
			INIT_COMPLETION(transfer_done);
		len = (buf_len > 0x20000) ? 0x20000 : buf_len;
		trans_ctrl = ((CF_TCNTR_MEM_TRANS_START) |
		      (CF_TCNTR_COMMON_ATTR_MEM &(attr_mem_transfer<<26)) |
			      (CF_TCNTR_TRANS_SIZE_WORD) |
			      (CF_TCNTR_DISABLE_ADDR_INC)|
			      (CF_TCNTR_COUNT_MASK & len));
		writel(trans_ctrl, cf_base + CF_TCNTR_OFFSET);
		buf_len = buf_len - len;
		dev_dbg(dev,"%s: len=%d trans_ctrl=0x%x\n",__func__,
			len,trans_ctrl);
		do {
			if (in_softirq()) {
				ret = is_buffer_avail(cf_base);
			} else {
				ret = wait_for_completion_timeout(&buffer_avail,BUFFER_AVAIL_TIMEOUT);
			}
			if(!ret) {
				printk(KERN_WARNING "%s: Buffer Available timeout\n",
					__func__);
				if (!in_softirq()) {
					dev_dbg(dev,"Process Context\n");
					complete(&buffer_avail);
				} 
			}
			if (!in_softirq())
				INIT_COMPLETION(buffer_avail);
			trans_len = (len>512)?512:len;
			ext_rp_offset=CF_EXT_RDPR_OFFSET;
			len = len - trans_len;
			while(trans_len) {
				val = readl(cf_base + ext_rp_offset);
				if(trans_len==2) {
					buf[0] = val & 0xFF;
					buf[1] = (val>>8) & 0xFF;
					buf+= 2;
					trans_len = 0;
				} else {
					buf[0] = val & 0xFF;
					buf[1] = (val>>8) & 0xFF;
					buf[2] = (val>>16) & 0xFF;
					buf[3] = (val>>24) & 0xFF;
					buf+= 4;
					trans_len = trans_len - 4;
				}
				ext_rp_offset+=4;
			}
			dev_dbg(dev,"%s: bytes_left(len)=%d\n",__func__,len);
		} while (len);
		if (in_softirq()) {
			ret = is_transfer_done(cf_base);
		} else {
			ret = wait_for_completion_timeout(&transfer_done,
				TRANSFER_DONE_TIMEOUT);
		}
		if(!ret) {
			if(buf_len)
				printk(KERN_WARNING "%s: Transfer Done timeout\n",__func__);
			if (!in_softirq()) {
				dev_dbg(dev,"Process Context\n");
                		complete(&transfer_done);
			}
		}
		dev_dbg(dev,"%s: bytes_left(buf_len)=%d\n",__func__,buf_len);
	}
	tmp = readl(cf_base + CF_TCNTR_OFFSET);
	/* Disable Transfer Start Bit explicitly */
	writel(tmp & ~(CF_TCNTR_MEM_TRANS_START), cf_base + CF_TCNTR_OFFSET);
	if (in_softirq()) {
		intr_en = readl(cf_base + CF_INTEN_OFFSET);
		intr_en = intr_en | CF_INTEN_TRANS_DONE_IRQ | 
				CF_INTEN_BUFF_AVAL_IRQ;
	}
#ifdef CONFIG_CF_LOCK
	spin_unlock_bh(cf_lock);
#endif
}

void pxa168_cf_enable_trueide_mode(void)
{
	u32 opmode, intr_en;
	opmode = readl(cf_base + CF_OPMODE_OFFSET);
	opmode = opmode | (CF_OPMODE_CARD_MODE_MASK && CARD_TRUE_IDE);
	writel(opmode, cf_base + CF_OPMODE_OFFSET);
	udelay(100);
	intr_en = readl(cf_base + CF_INTEN_OFFSET);
	intr_en = intr_en | CF_INTEN_TRUEIDE_MODE_IREQ;
	writel(intr_en, cf_base + CF_INTEN_OFFSET);
	udelay(100);
}

static irqreturn_t pxa168_cf_wakeup_irq(int irq, void *_cf)
{
	return IRQ_HANDLED;
}

static irqreturn_t pxa168_cf_irq(int irq, void *_cf)
{
#ifndef CONFIG_PXA168_CF_USE_GPIO_CARDDETECT
	int present;
#endif
	unsigned int irq_reg, intr_en;
	struct pxa168_cf_socket *cf = _cf;

	writel(0x40, APMU_WAKE_CLR); /* Clear CF Wakeup */
	irq_reg = readl(cf->base + CF_IRQ_OFFSET);
	intr_en = readl(cf->base + CF_INTEN_OFFSET);

#ifndef CONFIG_PXA168_CF_USE_GPIO_CARDDETECT
	if ((irq_reg & CF_IRQ_CARD_DETECT) && 
		(intr_en & CF_INTEN_CARD_DETECT)) { 
		dev_dbg(dev,"Card Detect Interrupt occured\n");
	
		/* Need to Clear Card Detect Interrupt Enable bit  
 		 * to clear Card Detect interrupt in IRQ Register
 		 */
		intr_en = intr_en & ~CF_INTEN_CARD_DETECT;
		writel(intr_en, cf->base + CF_INTEN_OFFSET);
		/* Clear Card Detect IRQ */
		writel(CF_IRQ_CARD_DETECT, cf->base + CF_IRQ_OFFSET);
		irq_reg = readl(cf->base + CF_IRQ_OFFSET);
		/* Re-enable Card Detect IRQ */
		intr_en = readl(cf->base + CF_INTEN_OFFSET);
                intr_en = intr_en | CF_INTEN_CARD_DETECT;
		writel(intr_en, cf->base + CF_INTEN_OFFSET);
		/* Kick off pccardd operations */
		present = pxa168_cf_present(cf);
		if (present != cf->present) {
			cf->present = present;
			dev_dbg(dev,"%s: card %s\n", driver_name,
				present ? "inserted" : "removed");
			pcmcia_parse_events(&cf->socket, SS_DETECT);
		}
	}
#endif
	if ((irq_reg & CF_IRQ_MEMO_MODE_READY) && 
		(intr_en & CF_INTEN_MEM_MODE_READY)) { 
		dev_dbg(dev,"Memory Mode Ready Interrupt occured\n");
		writel(CF_IRQ_MEMO_MODE_READY, cf->base + CF_IRQ_OFFSET);
		irq_reg = readl(cf->base + CF_IRQ_OFFSET);
		intr_en = intr_en & ~CF_INTEN_MEM_MODE_READY;
		writel(intr_en, cf->base + CF_INTEN_OFFSET);
		complete(&card_ready);
	}
	if ((irq_reg & CF_IRQ_BUFF_AVAL_IRQ) && 
		(intr_en & CF_INTEN_BUFF_AVAL_IRQ)) { 
		dev_dbg(dev,"Buffer Available Interrupt occured\n");
		writel(CF_IRQ_BUFF_AVAL_IRQ, cf->base+CF_IRQ_OFFSET);
		irq_reg = readl(cf->base + CF_IRQ_OFFSET);
		complete(&buffer_avail);
	}
	if ((irq_reg & CF_IRQ_TRANS_DONE_IRQ) && 
		(intr_en & CF_INTEN_TRANS_DONE_IRQ)) { 
		dev_dbg(dev,"Transfer Done Interrupt occured\n");
		writel(CF_IRQ_TRANS_DONE_IRQ, cf->base+CF_IRQ_OFFSET);
		irq_reg = readl(cf->base + CF_IRQ_OFFSET);
		complete(&transfer_done);
	}
	if ((irq_reg & CF_IRQ_PIO_TRANS_ERR_IRQ) &&
		(intr_en & CF_INTEN_PIO_TRANS_ERR_IRQ)) {
		printk(KERN_ERR "PIO Transfer Error!\n");
		writel(CF_IRQ_PIO_TRANS_ERR_IRQ, cf->base+CF_IRQ_OFFSET);
		irq_reg = readl(cf->base + CF_IRQ_OFFSET);
	}
	return IRQ_HANDLED;
}

#ifdef CONFIG_PXA168_CF_USE_GPIO_CARDDETECT
/* CF Host Controller Card Detect Interrupt doesnot occur.
 * As a temporary workaround we are configuring the CF
 * Card Detect Pin as GPIO and using it to trigger interrupt
 * during a Falling/Rising Edge
 */
static irqreturn_t pxa168_cf_carddetect_irq(int irq, void *_cf)
{
	struct pxa168_cf_socket	*cf = _cf;
	int present;
	/* Kick off pccardd operations */
	present = pxa168_cf_present(cf);
	if (present != cf->present) {
		cf->present = present;
		printk("%s: CF card %s\n", driver_name,
			present ? "inserted" : "removed");
		pcmcia_parse_events(&cf->socket, SS_DETECT);
	}
	return IRQ_HANDLED;
}
#endif

static void pxa168_cfhost_init(struct pxa168_cf_socket *cf)
{
	int 			opmode;
	u32			intr_en, clk_cfg_reg;
	/* Clear IRQ/Inten/Opmode Registers*/
	writel(0x1FFF, cf->base + CF_IRQ_OFFSET);
	writel(0x0, cf->base + CF_INTEN_OFFSET);
	writel(0x0, cf->base + CF_OPMODE_OFFSET);
	udelay(10);

	/* Enable the CFHost Controller*/
	opmode = readl(cf->base + CF_OPMODE_OFFSET);
	opmode = opmode | CF_OPMODE_CFHOST_ENABLE;
	writel(opmode, cf->base + CF_OPMODE_OFFSET);

	/* Program Clock Configuration Register */
	clk_cfg_reg = readl(cf->base + CFI_CLOCK_CONFIG_OFFSET);
	clk_cfg_reg = (CFI_CLOCK_CONFIG_MASK & CLOCK_CONFIG_75M) |
		(CFI_CLOCK_RATIO_MASK & (CLOCK_RATIO_1 << 4));
	writel(clk_cfg_reg, cf->base + CFI_CLOCK_CONFIG_OFFSET);

	/* Set Interrupt Enable Register*/
	intr_en = readl(cf->base + CF_INTEN_OFFSET);
	intr_en = intr_en | CF_INTEN_CARD_DETECT |
			    CF_INTEN_MEM_MODE_READY |
			    CF_INTEN_TRANS_DONE_IRQ |
			    CF_INTEN_PIO_TRANS_ERR_IRQ |
			    CF_INTEN_BUFF_AVAL_IRQ;
	writel(intr_en, cf->base + CF_INTEN_OFFSET);
}

static int __init pxa168_cf_probe(struct platform_device *pdev)
{
	struct pxa168_cf_socket	*cf;
	int			irq, cd_irq;
	int			status;
	struct resource		*res;
	struct clk *clk = NULL;

	dev = &pdev->dev;
	res = platform_get_resource(pdev,IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);
	cd_irq = platform_get_irq(pdev, 1); 
	dev_dbg(dev,"%s: res->start: 0x%8x irq:%d",__func__,res->start,irq);
	if (!res || irq < 0 || cd_irq < 0)
	{
		dev_err(&pdev->dev, "Unable to get resource/irq\n");
                return -ENXIO;
	}
	cf = kzalloc(sizeof *cf, GFP_KERNEL);
	if (!cf)
	{
		dev_err(&pdev->dev, "Unable to allocate CF Memory\n");
		return -ENOMEM;
	}
	cf->pdev = pdev;
	cf->res  = res;
	cf->phys_baseaddr = res->start;
	platform_set_drvdata(pdev, cf);

	res = request_mem_region(res->start, SZ_2K, driver_name);
        if (!res) {
		dev_err(&pdev->dev, "Unable to request CF Memory\n");
                status = -EBUSY;
		goto fail0;
	}
	cf->base = ioremap(res->start,SZ_2K);
	printk("cf->base:0x%8x\n",(unsigned int) cf->base);
        if (!cf->base) {
                status = -ENOMEM;
                goto fail1;
        }
	cf_base = cf->base;

	 /* Enable CKEN_CF */
        clk = clk_get(&pdev->dev, "CFCLK");
        if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "failed to get cf clock\n");
                return PTR_ERR(clk);
	}
        clk_enable(clk);
	cf->clk = clk;

	status = request_irq(irq, pxa168_cf_irq, IRQF_SHARED | IRQF_DISABLED, 
		driver_name, cf);
	if (status < 0)
	{
		dev_err(&pdev->dev, "Unable to request CF irq\n");
		goto fail2;
	}
	status = request_irq(IRQ_PXA168_CF_WAKEUP, pxa168_cf_wakeup_irq, 
		IRQF_SHARED, driver_name, cf);

	if (status < 0)
	{
		dev_err(&pdev->dev, "Unable to request CF Wakeup irq\n");
		goto fail2;
	}

	status = request_irq(cd_irq, pxa168_cf_carddetect_irq,
		IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, driver_name, cf);
	
	if (status < 0)
	{
		dev_err(&pdev->dev, "Unable to request CF CardDetect GPIO\
			irq\n");
		goto fail2;
	}

	init_completion(&transfer_done);
	init_completion(&buffer_avail);
	init_completion(&card_ready);
	
	transf_done = buf_avail = card_rdy = 1;
	
	cf->irq = irq;
	/* FIXME - Revisit Socket Initialization */
	cf->socket.pci_irq = irq; 
	/* pcmcia layer only remaps "real" memory not iospace */
	cf->socket.io_offset = (u32) cf->base + CF_IO_BASE_OFFSET;
	if (!cf->socket.io_offset) {
		status = -ENXIO;
		goto fail3;
	}
	cf->socket.owner = THIS_MODULE;
	cf->socket.dev.parent = &pdev->dev;
	cf->socket.ops = &pxa168_cf_ops;
	cf->socket.resource_ops = &pccard_static_ops;
	cf->socket.features = SS_CAP_PCCARD | SS_CAP_STATIC_MAP
				| SS_CAP_MEM_ALIGN;
	cf->socket.map_size = SZ_1K;
	cf->socket.io[0].res = res;
	spin_lock_init(&cf->lock);
	cf_lock = &cf->lock;

	dev_dbg(dev,"%s:Registering PCMCIA Socket\n",__func__);
	status = pcmcia_register_socket(&cf->socket);
	if (status < 0)
	{
		dev_err(&pdev->dev, "Unable to Register Socket\n");
		goto fail4;
	}
	cf->active = 0;
	cf->present = 0;

	pxa168_cfhost_init(cf);

	/* Kick off pccardd operations if card detected during boot */
	cf->present = pxa168_cf_present(cf);
	printk("%s:%sCF card detected\n", driver_name,
		cf->present ? "" : "No ");
	if (cf->present)
		pcmcia_parse_events(&cf->socket, SS_DETECT);

	return 0;

fail4:	
	if (cf->socket.io_offset)
		iounmap((void __iomem *) cf->socket.io_offset);
fail3:  
	free_irq(irq, cf);
fail2:
	clk_disable(cf->clk);
	if (cf->base)
		iounmap(cf->base);
fail1:
	release_resource(cf->res);
fail0:
	kfree(cf);
	return status;
}

static int __exit pxa168_cf_remove(struct platform_device *pdev)
{
	struct pxa168_cf_socket *cf = platform_get_drvdata(pdev);

	cf->active = 0;
	pcmcia_unregister_socket(&cf->socket);
	iounmap(cf->base);
	iounmap((void __iomem *) cf->socket.io_offset);
	clk_disable(cf->clk);
	release_resource(cf->res);
	free_irq(cf->irq, cf);
	kfree(cf);
	return 0;
}

#ifdef CONFIG_PM
static int pxa168_cf_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	return 0;
}

static int pxa168_cf_resume(struct platform_device *pdev)
{
	struct pxa168_cf_socket *cf;
	cf = platform_get_drvdata(pdev);
	clk_enable(cf->clk);
	pxa168_cfhost_init(cf);
	init_completion(&transfer_done);
	init_completion(&buffer_avail);
	init_completion(&card_ready);
	transf_done = buf_avail = card_rdy = 1;
	return 0;
}
#else
#define pxa168_cf_suspend  NULL
#define pxa168_cf_resume   NULL
#endif


static struct platform_driver pxa168_cf_driver = {
	.driver = {
		.name	= (char *) driver_name,
	},
	.remove		= __exit_p(pxa168_cf_remove),
	.suspend	= pxa168_cf_suspend,
	.resume		= pxa168_cf_resume,
};

static int __init pxa168_cf_init(void)
{
	return platform_driver_probe(&pxa168_cf_driver, pxa168_cf_probe);
}

static void __exit pxa168_cf_exit(void)
{
	platform_driver_unregister(&pxa168_cf_driver);
}

module_init(pxa168_cf_init);
module_exit(pxa168_cf_exit);

MODULE_DESCRIPTION("Aspen CF Interface Driver");
MODULE_LICENSE("GPL");

EXPORT_SYMBOL_GPL(pxa168_cf_mem_readb);
EXPORT_SYMBOL_GPL(pxa168_cf_mem_writeb);
EXPORT_SYMBOL_GPL(pxa168_cf_mem_readw);
EXPORT_SYMBOL_GPL(pxa168_cf_mem_writew);
EXPORT_SYMBOL_GPL(pxa168_cf_mem_read);
EXPORT_SYMBOL_GPL(pxa168_cf_mem_write);
