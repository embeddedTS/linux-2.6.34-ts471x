/*
 *  linux/drivers/icr/pxa168_icr.c
 *
 *  Copyright:	(C) Copyright 2009 Marvell International Ltd.
 *
 *  Author: Alan Guenther <>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/highmem.h>
#include <linux/cdev.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <linux/slab.h>
#include <linux/sched.h>

#include <mach/hardware.h>
#include <linux/pxa168_ioctlICR.h>

#include "pxa168_icr.h"
#include "pxa168_ICRioctlTable.h"

#define ICR_TIMEOUT (20*HZ)
#define ICRaddr(offset) ((int)(icr->mmio_base + (offset)))

#define WRITEREG(value,offset) __raw_writel((value),ICRaddr(offset))

#define READREG(offset) __raw_readl(ICRaddr(offset))

#define PRINTKREG(a, ZZZ) DEBUG("%s %s = 0x%x\n", a, #ZZZ, READREG((ZZZ)))

#define ORREG(reg, value) { u32 temp = READREG((reg));\
				temp |= value;\
				WRITEREG(temp, (reg));\
			  }

/* ICR need to allocate a very large contiguous area */
#define ENABLE_ALL_INTERUPTS 0xfff
#define CLEAR_ALL_INTERUPTS (~0xfff)

#define ICR_NAME "icr"
#define PFX ICR_NAME ": "
#define pr_init(fmt, args...) ({ static const __initconst char __fmt[] = fmt; printk(__fmt, ## args); })

#if 0
#define DEBUG(args...)	  {printk(args); }
#else
#define DEBUG(args...)
#endif

static int icr_count = 1;
static dev_t icr_dev;

DECLARE_WAIT_QUEUE_HEAD(dma_wait);

static struct cdev *icr_cdev;

struct dma_slots {
	u32 inAddr;
	u32 inSize;
	u32 outAddr;
	u32 outSize;
	u16 inInUse;
	u16 outInUse;
	u32 waitToken;
};

#define SLOTMAX 2
static struct dma_slots DMAslot[SLOTMAX];

static int DMAnextslotInR = 0;
static int DMAnextslotInW = 0;

#define DMA_READ 0
#define DMA_WRITE 1
#define DMA_READWRITE 2
#define DMAdirection int
#define DMANoSlot(slot) (slot < 0)

struct pxa168_icr {
	int initialized;	/* non-zero if ICR in use */
	int interrupt_status;
	int sem;
	int lock;
	void *dev_id;
	int icr_dma_transfer_size;
	char *mmio_base;
	char *clockReg;
	int waitToken;
	int irq;
	struct clk *clk;
};

static int icr_svf_siz_reg[SLOTMAX] = {
	ICR_SVF0_SIZ_REG,
	ICR_SVF1_SIZ_REG
};

static int icr_df_disp_size_reg[SLOTMAX] = {
	ICR_DF0DISP_SIZE_REG,
	ICR_DF1DISP_SIZE_REG
};

static int icr_rx_dma_ctrl_reg[SLOTMAX] = {
	ICR_RX_DMA_CTRL0_REG,
	ICR_RX_DMA_CTRL1_REG
};

static int icr_tx_dma_ctrl_reg[SLOTMAX] = {
	ICR_TX_DMA_CTRL0_REG,
	ICR_TX_DMA_CTRL1_REG
};

static int icr_svf_rgb_strt_addr_reg[SLOTMAX] = {
	ICR_SVF0RGB_STRT_ADDR_REG,
	ICR_SVF1RGB_STRT_ADDR_REG
};

static int icr_df_disp_strt_addr_reg[SLOTMAX] = {
	ICR_DF0DISP_STRT_ADDR_REG,
	ICR_DF1DISP_STRT_ADDR_REG
};

static int icr_df_disp_spscr_reg[SLOTMAX] = {
	ICR_DF0DISP_SPSCR_REG,
	ICR_DF1DISP_SPSCR_REG
};

static int icr_df_vid_siz_reg[SLOTMAX] = {
	ICR_DF0_VID_SIZ_REG,
	ICR_DF1_VID_SIZ_REG
};

static char *icr_dev_name = "icr";

static struct pxa168_icr *my_pxa168_icr;

/* dma_slots access methods */
static void setAvailable(int slot, int rw)
{
	switch(rw) {
		case DMA_READ:
			DMAslot[slot].inInUse = 0;
			break;
		case DMA_WRITE:
			DMAslot[slot].outInUse = 0;
			break;
		case DMA_READWRITE:
			DMAslot[slot].inInUse = 0;
			DMAslot[slot].outInUse = 0;
			break;
		default:
			printk("Fatal error in setAvailable\n");
	}
}

static u32 getKaddr(struct mm_struct *mm, u32 Vaddr)
{
	u32 ret = 0UL;
	pgd_t *pgd;
	pud_t *pud;
	pmd_t *pmd;
	pte_t *pte;
	struct page *page;

	pgd = pgd_offset(mm, Vaddr);
	if (!pgd_none(*pgd)) {
		pud = pud_offset(pgd, Vaddr);
		if (!pud_none(*pud)) {
			pmd = pmd_offset(pud, Vaddr);
			if (!pmd_none(*pmd)) {
				pte = pte_offset_map(pmd, Vaddr);
				if (!pte_none(*pte) && pte_present(*pte)) {
					page = pte_page(*pte);
					if(page) {
						ret = page_to_phys(page);
						ret |= (Vaddr & (PAGE_SIZE-1));
					}
				}
			}
		}
	}
	return ret;
}

static irqreturn_t pxa168_icr_irq_handler(int irq, void *dev_id)
{
	struct pxa168_icr *icr = dev_id;
	irqreturn_t ret = IRQ_NONE;
	u32 icr_interrupt_status;
	u32 wake = 0;
	u32 temp;

	icr_interrupt_status = READREG(ICR_INTR_STATUS_REG);
	DEBUG("**** interupt:  ICR_INTR_STATUS_REG = 0x%x\n",
		icr_interrupt_status);
	WRITEREG(~icr_interrupt_status, ICR_INTR_STATUS_REG);
	icr->interrupt_status |= icr_interrupt_status;
	if (ICR_INTR_STATUS_ERRORS(icr_interrupt_status))
		DEBUG("errors:  ICR_INTR_STATUS_REG = 0x%x\n",
			icr_interrupt_status);
	if (ICR_INTR_STATUS_DMA0_DONE(icr->interrupt_status)) {
		DEBUG("Process DMA0 interrupt\n");
		temp = READREG(ICR_RX_DMA_CTRL0_REG);
		if (temp & 1)
			DEBUG("RX_DMA0_REG %d bad\n", temp);
		temp = READREG(ICR_TX_DMA_CTRL0_REG);
		if (temp & 1)
			DEBUG("TX_DMA0_REG %d bad\n", temp);
		ICR_INTR_STATUS_DMA0_CLEAR(icr->interrupt_status);
		dma_unmap_single(NULL, DMAslot[0].inAddr,
				 DMAslot[0].inSize, DMA_FROM_DEVICE);
		dma_unmap_single(NULL, DMAslot[0].outAddr,
				 DMAslot[0].outSize, DMA_TO_DEVICE);
		setAvailable(0, DMA_READWRITE);
		wake = 1;
	}
	if (ICR_INTR_STATUS_DMA1_DONE(icr->interrupt_status)) {
		DEBUG("Process DMA1 interrupt\n");
		temp = READREG(ICR_RX_DMA_CTRL1_REG);
		if (temp & 1)
			DEBUG("RX_DMA1_REG %d bad\n", temp);
		temp = READREG(ICR_TX_DMA_CTRL1_REG);
		if (temp & 1)
			DEBUG("TX_DMA1_REG %d bad\n", temp);
		ICR_INTR_STATUS_DMA1_CLEAR(icr->interrupt_status);
		dma_unmap_single(NULL, DMAslot[1].inAddr,
				 DMAslot[1].inSize, DMA_FROM_DEVICE);
		dma_unmap_single(NULL, DMAslot[1].outAddr,
				 DMAslot[1].outSize, DMA_TO_DEVICE);
		setAvailable(1, DMA_READWRITE);
		wake = 1;
	}
	if (wake) {
		DEBUG("wake up\n");
		wake_up_interruptible(&dma_wait);
	}
	if (icr_interrupt_status != 0) {
		DEBUG("Interrupt handled\n");
		ret = IRQ_HANDLED;
	}
	return ret;
}

/*
 * the ICR works with DMA.  If a DMA slot is available,
 * return it, otherwise return -1
 */
static int icr_get_slot(DMAdirection dir)
{
	int slot = -1;
	if ((dir == DMA_WRITE) && (DMAslot[DMAnextslotInW].outInUse == 0))
		slot = DMAnextslotInW;
	if ((dir == DMA_READ) && (DMAslot[DMAnextslotInR].inInUse == 0))
		slot = DMAnextslotInR;
	if ((dir == DMA_READWRITE) && (DMAslot[DMAnextslotInR].inInUse == 0)
	    && (DMAslot[DMAnextslotInW].outInUse == 0)) {
		slot = DMAnextslotInW;
		/* fatal error? */
		if (DMAnextslotInW != DMAnextslotInR)
			slot = -1;
	}
	return slot;
}

/*
 * advance to the next slot
 */
static void icr_update_slot(int slot, DMAdirection dir)
{
	if ((dir == DMA_WRITE) && (DMAnextslotInW == slot)) {
		DMAslot[slot].outInUse = 1;
		DMAnextslotInW ^= 1;
		DEBUG("DMAnextslotInW: %d\n", DMAnextslotInW);
	}
	if ((dir == DMA_READ) && (DMAnextslotInR == slot)) {
		DMAslot[slot].inInUse = 1;
		DMAnextslotInR ^= 1;
		DEBUG("DMAnextslotInR: %d\n", DMAnextslotInR);
	}
	if ((dir == DMA_READWRITE) && (DMAnextslotInR == slot)
	    && (DMAnextslotInW == slot)) {
		DMAslot[slot].inInUse = 1;
		DMAslot[slot].outInUse = 1;
		DMAnextslotInW ^= 1;
		DMAnextslotInR ^= 1;
		DEBUG("DMAnextslotInRW: %d\n", DMAnextslotInR);
	}
	DEBUG("update slot error: %d\n", DMAnextslotInR);
}

static int pxa168_icr_ioctl(struct inode *inode, struct file *file,
			u32 cmd, unsigned long arg1)
{
	struct pxa168_icr *icr = (struct pxa168_icr *)file->private_data;
	int blocking = !(file->f_flags & O_NONBLOCK);
	int slot;
	int type = _IOC_TYPE(cmd);
	int number = _IOC_NR(cmd);

	if (cmd == ICR_IOCTL_transform || cmd == ICR_IOCTL_transformDBwait) {
		int psrc_buf;	/* ICR_SRC_BUFFER * */
		int pdst_buf;	/* ICR_DST_BUFFER * */
		int pwaitToken;	/* int * */
		ICR_SRC_BUFFER src_buf;
		ICR_DST_BUFFER dst_buf;
		u32 srcPaddr;
		u32 dstPaddr;
		int temp;
		int sSize;
		int dSize;
		struct mm_struct *mm = current->mm;
		int new_format;

		/* get user space pointer to input and output addresses */
		if (get_user(psrc_buf, (int *)arg1))
			return -EFAULT;

		if (get_user(pdst_buf, (int *)(arg1 + 4)))
			return -EFAULT;

		if (get_user(pwaitToken, (int *)(arg1 + 8)))
			return -EFAULT;

		temp = copy_from_user((char *)&src_buf, __user (char *)psrc_buf, sizeof(src_buf));
		if (temp != 0)
			return -EFAULT;

		temp = copy_from_user((char *)&dst_buf, __user (char *)pdst_buf, sizeof(dst_buf));
		if (temp != 0)
			return -EFAULT;

		/* require FULL SCREEN mode */
		if (src_buf.buf.height != dst_buf.buf.height ||
			src_buf.buf.width != dst_buf.buf.width ||
			src_buf.buf.height != dst_buf.video_height ||
			src_buf.buf.width != dst_buf.video_width ||
			dst_buf.video_start_y != 0 ||
			dst_buf.video_start_x != 0 )
			return -EINVAL;

		sSize = src_buf.buf.height * src_buf.buf.width;
		dSize = dst_buf.buf.height * dst_buf.buf.width;
		if (sSize == 0 || dSize == 0)
			return -EINVAL;
		if (!access_ok(VERIFY_READ, (unsigned int)src_buf.buf.addr, sSize))
			return -EFAULT;
		if (!access_ok(VERIFY_WRITE, (unsigned int)dst_buf.buf.addr, dSize))
			return -EFAULT;

		/* convert user space addresses to kernel physical addresses */
		spin_lock(&mm->page_table_lock);
		srcPaddr = getKaddr(mm, (unsigned int)src_buf.buf.addr);
		dstPaddr = getKaddr(mm, (unsigned int)dst_buf.buf.addr);
		spin_unlock(&mm->page_table_lock);

		if (srcPaddr == -1 || dstPaddr == -1)
			return -EFAULT;

		/* wait for a DMA slot to be available */
		do {
			slot = icr_get_slot(DMA_READWRITE);
			/* if no slot and blocking, wait for a slot */
			if (DMANoSlot(slot)) {
				if (!blocking)
					return -EAGAIN;
				else {
					int timeout = interruptible_sleep_on_timeout(&dma_wait,ICR_TIMEOUT);
					if(timeout == 0)
						return -EIO;
				}
			}
			if (signal_pending(current))
				return -EINTR;
		} while (DMANoSlot(slot));

		DEBUG
			("setup and start transform: DMA %d source VA 0x%x(0x%x)   dest 0x%x(0x%x)\n",
			slot, (unsigned int)src_buf.buf.addr, srcPaddr,
			(unsigned int)dst_buf.buf.addr, dstPaddr);

		if (pwaitToken != 0) {
			DMAslot[slot].waitToken  = icr->waitToken++;
			put_user(DMAslot[slot].waitToken, (__u32 __user *) pwaitToken);
			DEBUG("slot %d - waitToken 0x%x\n",slot, DMAslot[slot].waitToken );
		}
		DMAslot[slot].inAddr = srcPaddr;
		DMAslot[slot].outAddr = dstPaddr;
		DMAslot[slot].inSize = sSize;
		DMAslot[slot].outSize = dSize;

		WRITEREG(((src_buf.buf.height << 16) | src_buf.buf.width),
			 icr_svf_siz_reg[slot]);
		WRITEREG((dst_buf.buf.height << 16 | dst_buf.buf.width),
			 icr_df_disp_size_reg[slot]);
		WRITEREG(((dst_buf.video_start_y << 16) | dst_buf.video_start_x),
			 icr_df_disp_spscr_reg[slot]);
		WRITEREG(((dst_buf.video_height << 16)| dst_buf.video_width),
			 icr_df_vid_siz_reg[slot]);

		DEBUG("ICR_DF%dDISP_SIZE_REG = 0x%x   dSize = %d\n",
			slot, temp, DMAslot[slot].outSize);

		WRITEREG(5, ICR_DBL_BUF_TRG_REG);

		new_format = (src_buf.buf.format == ICR_HWFORMAT_XRGB8888)
			? ICR_GCR_SRC_FORMAT : 0;
		new_format |= (dst_buf.buf.format == ICR_HWFORMAT_XRGB8888)
			? ICR_GCR_DST_FORMAT : 0;

		temp = READREG(ICR_DMA_GCR_REG);
		if ((temp & ICR_DMA_FORMAT_MASK) != new_format) {
			int i;
			/* if any DMA channel is active,
			 * do NOT change the format or garbage will result
			 */
			for (i = 0; i < SLOTMAX; i++) {
				if (((DMAslot[i].inInUse == 1)
					|| (DMAslot[i].outInUse == 1)))
					return -EAGAIN;
			}
			temp &= ~ICR_DMA_FORMAT_MASK;
			temp |= new_format;
			DEBUG("Changing DMA format -0x%x to 0x%x\n", (temp & ICR_DMA_FORMAT_MASK), new_format);
			WRITEREG(temp, ICR_DMA_GCR_REG);
		}

		icr_update_slot (slot,DMA_READWRITE);

		PRINTKREG("initial TX DMA channel reg", icr_tx_dma_ctrl_reg[slot]);
		PRINTKREG("initial RX DMA channel reg", icr_rx_dma_ctrl_reg[slot]);
		WRITEREG(srcPaddr, icr_svf_rgb_strt_addr_reg[slot]);
		WRITEREG(dstPaddr, icr_df_disp_strt_addr_reg[slot]);
		PRINTKREG("input addr:", icr_svf_rgb_strt_addr_reg[slot]);
		PRINTKREG("output addr:", icr_df_disp_strt_addr_reg[slot]);

		ORREG(icr_tx_dma_ctrl_reg[slot], 1);
		PRINTKREG("after setting", icr_tx_dma_ctrl_reg[slot]);

		ORREG(icr_rx_dma_ctrl_reg[slot], 1);
		PRINTKREG("after setting", icr_rx_dma_ctrl_reg[slot]);


		/* DMA is running on this channel now */
		DEBUG("DMA%d setup\n", slot);

		if (blocking) {
			/* if this is a blocking IO,
			 *then wait for completion here
			 */
			int timeout = interruptible_sleep_on_timeout(&dma_wait,ICR_TIMEOUT);
			if (timeout == 0)
				return -EIO;
			if (signal_pending(current))
				return -EINTR;
		}
		if (cmd != ICR_IOCTL_transformDBwait)
			return 0;
	}
	if (cmd == ICR_IOCTL_wait || cmd == ICR_IOCTL_transformDBwait) {
		int wait;
		int waitToken;
		int i;
		int ret = 0;
		if (cmd == ICR_IOCTL_wait) {
			waitToken = arg1;
		} else {
			if (get_user(waitToken, (int*)(arg1 + 12)))
				return -EFAULT;
		}

		DEBUG("ICRwait - 0x%x\n", waitToken);
		do {
			DEBUG("wait loop\n");
			wait = -1;
			for (i = 0; i < SLOTMAX; i++) {
				if (((DMAslot[i].inInUse == 1)
				     || (DMAslot[i].outInUse == 1))
				    && (DMAslot[i].waitToken == waitToken))
					wait = i;
			}

			if (wait >= 0) {
				DEBUG("Waiting for 0x%x\n", wait);
				/* last chance check for sleep */
				if ((DMAslot[wait].inInUse == 1)
				    || (DMAslot[wait].outInUse == 1)) {
					int timeout = interruptible_sleep_on_timeout(&dma_wait,ICR_TIMEOUT);
					if(timeout == 0) {
						ret = -EIO;
						break;
					}
				}
			}
			if (signal_pending(current)) {
				ret = -EINTR;
				break;
			}
		} while (wait >= 0);

		return ret;
	}

	if (cmd == ICR_IOCTL_tagged_set) {
		int tags;
		int data;
		u32 datalen;
		u32 copied_data;
		u32 copied_tags;
		u8 *local_data;
		u16 *local_tags;
		int i;
		int retval = 0;

		/* get user space pointer to input and output addresses */
		if (get_user(tags, (int *)arg1))
			return -EFAULT;

		if (get_user(data, (int *)(arg1 + 4)))
			return -EFAULT;

		if (get_user(datalen, (int *)(arg1 + 8)))
			return -EFAULT;

		if ((datalen > sizeof(ICRtagTable)/2) || datalen == 0)
			return -EINVAL;

		local_data = (u8 *)kmalloc(datalen*sizeof(u8), GFP_KERNEL);
		if (local_data == NULL)
			return -EFAULT;
		copied_data = copy_from_user(local_data, __user (u8 *)data, datalen);
		local_tags = (u16 *)kmalloc(datalen*sizeof(u16), GFP_KERNEL);
		if (local_tags == NULL)
			return -EFAULT;
		copied_tags = copy_from_user(local_tags, __user (u16 *)tags, datalen*sizeof(u16));
		if(copied_data == 0 && copied_tags == 0) {
			for(i=0;i<datalen;i++) {
				u16 j = local_tags[i];
				if (j != (u16)ICR_V1_IGNORE ) {
					int regaddr = ICRtagTable[j];
					WRITEREG(local_data[i], regaddr);
				}
			}
			retval = 0;
		} else
			retval = -EINVAL;

		kfree(local_data);
		kfree(local_tags);
		return retval;

	}
	if (cmd == ICR_IOCTL_tagged_get) {
		int tags;
		int data;
		u32 datalen;
		u32 copied_data;
		u32 copied_tags;
		u8 *local_data;
		u16 *local_tags;
		int i;
		int retval = 0;

		/* get user space pointer to input and output addresses */
		if (get_user(tags, (int *)arg1))
			return -EFAULT;

		if (get_user(data, (int *)(arg1 + 4)))
			return -EFAULT;

		if (get_user(datalen, (int *)(arg1 + 8)))
			return -EFAULT;

		if ((datalen > 1024) || datalen == 0)
			return -EINVAL;	/*value out of range */

		local_data = (u8 *)kmalloc(datalen*sizeof(u8), GFP_KERNEL);
		local_tags = (u16 *)kmalloc(datalen*sizeof(u16), GFP_KERNEL);
		copied_tags = copy_from_user(local_tags, __user (u16 *)tags, datalen*sizeof(u16));
		if(copied_tags == 0) {
			for(i=0;i<datalen;i++) {
				u16 j = local_tags[i];
				if (j != (u16)ICR_V1_IGNORE ) {
					int regaddr = ICRtagTable[j];
					local_data[i] = READREG(regaddr);
				} else {
					local_data[i] = 0;
				}
			}
			copied_data = copy_to_user(__user (u8 *)data, local_data, datalen);
			retval = (copied_data != 0) ? -EINVAL : 0;
		} else
			retval = -EINVAL;

		kfree(local_data);
		kfree(local_tags);
		return retval;

	}
	if (type == ICR_SET_CODE && (number >= SET_ICR_REG_FIRST)
	    && (number <= SET_ICR_REG_LAST)) {
		int i = number - SET_ICR_REG_FIRST;
		int reg = ICRioctlTable[i];

		/* if this is a dma transfer size, remember it */
		if (ICRioctlTable[i] >= 0x30 && ICRioctlTable[i] <= 0x3c)
			icr->icr_dma_transfer_size = arg1;
		WRITEREG(arg1, reg);
		return 0;
	}
	if (type == ICR_GET_CODE && (number >= GET_ICR_REG_FIRST)
	    && (number <= GET_ICR_REG_LAST)) {
		int i = number - GET_ICR_REG_FIRST;
		int reg = (int)ICRioctlTable[i];
		u32 val;

		val = READREG(reg);

		put_user(val, (__u32 __user *) arg1);
		return 0;
	}

	return -ENOIOCTLCMD;

}

static int pxa168_icr_open(struct inode *inode, struct file *file)
{
	struct pxa168_icr *icr = my_pxa168_icr;

	DEBUG("ICR open\n");

	if (icr->initialized++)
		return -EBUSY;

	WRITEREG(ICR_GCR_FULL_RESET, ICR_DMA_GCR_REG);
	WRITEREG(ICR_GCR_FULL_ENABLE, ICR_DMA_GCR_REG);

	file->private_data = icr;

	return nonseekable_open(inode, file);
}

static int pxa168_icr_release(struct inode *inode, struct file *file)
{
	struct pxa168_icr *icr = (struct pxa168_icr *)file->private_data;

	DEBUG("ICR Release");

	if (icr)
		icr->initialized = 0;

	return 0;
}

#define res_size(res)	((res)->end - (res)->start + 1)

static const struct file_operations icr_fops = {
	.owner = THIS_MODULE,
	.ioctl = pxa168_icr_ioctl,
	.open = pxa168_icr_open,
	.release = pxa168_icr_release,
};

static void pxa168_icr_initdev(struct pxa168_icr *icr)
{
	clk_enable(icr->clk);

	icr->icr_dma_transfer_size = ICR_DMA_TRANSFER_64; /*_32, _64, or _128*/
	WRITEREG(ICR_GCR_FULL_RESET, ICR_DMA_GCR_REG);

	WRITEREG(icr->icr_dma_transfer_size, ICR_RX_DMA_CTRL0_REG);
	WRITEREG(icr->icr_dma_transfer_size, ICR_RX_DMA_CTRL1_REG);
	WRITEREG(icr->icr_dma_transfer_size, ICR_TX_DMA_CTRL0_REG);
	WRITEREG(icr->icr_dma_transfer_size, ICR_TX_DMA_CTRL1_REG);

	WRITEREG(ICR_GCR_FULL_ENABLE, ICR_DMA_GCR_REG);
	PRINTKREG("", ICR_DMA_GCR_REG);

	WRITEREG(1, ICR_DBL_BUF_TRG_REG);

	/* set interrupts clear on write */
	WRITEREG(0, ICR_INTR_CLR_SEL_REG);

	/* then clear them by writing */
	WRITEREG(CLEAR_ALL_INTERUPTS, ICR_INTR_STATUS_REG);

	/* do interrupt in all cases */
	WRITEREG(ENABLE_ALL_INTERUPTS, ICR_INTR_MASK_REG);

	WRITEREG(ENABLE_ALL_INTERUPTS, ICR_INTR_STAT_MASK_REG);

	DMAnextslotInR = 0;
	DMAnextslotInW = 0;
	setAvailable(0, DMA_READWRITE);
	setAvailable(1, DMA_READWRITE);
}

static int __devinit pxa168_icr_probe(struct platform_device *pdev)
{
	struct pxa168_icr *icr;
	struct resource *res;
	int error;

	DEBUG("ICR probe");
	my_pxa168_icr = icr = kzalloc(sizeof(struct pxa168_icr), GFP_KERNEL);
	if (icr == NULL) {
		dev_err(&pdev->dev, "failed to allocate driver data\n");
		return -ENOMEM;
	}

	icr->irq = platform_get_irq(pdev, 0);
	if (icr->irq < 0) {
		dev_err(&pdev->dev, "failed to get icr irq\n");
		error = -ENXIO;
		goto failed_free;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "failed to get I/O memory\n");
		error = -ENXIO;
		goto failed_free;
	}

	res = request_mem_region(res->start, res_size(res), pdev->name);
	if (res == NULL) {
		dev_err(&pdev->dev, "failed to request I/O memory\n");
		error = -EBUSY;
		goto failed_free;
	}

	icr->mmio_base = ioremap(res->start, res_size(res));
	printk("mmio_base = 0x%x,  res->start = 0x%x,  res_sizer = 0x%x\n",
		  (unsigned int)icr->mmio_base, res->start, res_size(res) );
	if (icr->mmio_base == NULL) {
		dev_err(&pdev->dev, "failed to remap I/O memory\n");
		error = -ENXIO;
		goto failed_free_mem;
	}

	icr->clk = clk_get(&pdev->dev, "ICRCLK");
	if (IS_ERR(icr->clk)) {
		dev_err(&pdev->dev, "failed to get icr clock\n");
		error = PTR_ERR(icr->clk);
		goto failed_free_io;
	}

	platform_set_drvdata(pdev, icr);

	DEBUG("request ICR irq - %d\n",icr->irq);
	error = request_irq(icr->irq, pxa168_icr_irq_handler, IRQF_DISABLED,
			    pdev->name, icr);
	if (error) {
		dev_err(&pdev->dev, "failed to request IRQ\n");
		goto failed_free_dev;
	}

	device_init_wakeup(&pdev->dev, 1);

	if (alloc_chrdev_region(&icr_dev, 0, icr_count, icr_dev_name)) {
		DEBUG(KERN_INFO "Init ICR alloc failed\n");
		error = -ENODEV;
		//		goto error_exit;
	}

	icr_cdev = cdev_alloc();

	if (!icr_cdev) {
		error = -ENOMEM;
		DEBUG(KERN_INFO "Init ICR nomem2\n");
		goto error_dev_unregister;
	}

	icr_cdev->ops = &icr_fops;
	icr_cdev->owner = THIS_MODULE;

	if (cdev_add(icr_cdev, icr_dev, icr_dev)) {
		DEBUG(KERN_INFO "Init ICR nodev2\n");
		error = -ENODEV;
		goto error_free_cdev;
	}

	pxa168_icr_initdev(icr);

	init_waitqueue_head(&dma_wait);

	DEBUG(KERN_INFO "*********Init ICR good\n");

	return 0;

error_free_cdev:
	cdev_del(icr_cdev);

error_dev_unregister:
	unregister_chrdev_region(icr_dev, icr_count);

failed_free_dev:
	platform_set_drvdata(pdev, NULL);
failed_free_io:
	iounmap(icr->mmio_base);
	//failed_put_clk:
	clk_put(icr->clk);
failed_free_mem:
	release_mem_region(res->start, res_size(res));
failed_free:
	kfree(icr);
	return error;
}

static int __devexit pxa168_icr_remove(struct platform_device *pdev)
{
	struct pxa168_icr *icr = platform_get_drvdata(pdev);

	DEBUG("****** ICR remove ****");
	free_irq(icr->irq, pdev);

	clk_disable(icr->clk);
	clk_put(icr->clk);

	iounmap(icr->mmio_base);

	//	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	//	release_mem_region(res->start, res_size(res));

	platform_set_drvdata(pdev, NULL);
	kfree(icr);

	DEBUG(KERN_INFO "Exit ICR\n");

	return 0;
}

#ifdef CONFIG_PM
static int pxa168_icr_suspend(struct platform_device *pdev, pm_message_t msg)
{

	DEBUG(KERN_INFO "ICR suspend complete\n");

	return 0;
}

static int pxa168_icr_resume(struct platform_device *pdev)
{
	struct pxa168_icr *icr = platform_get_drvdata(pdev);

	pxa168_icr_initdev(icr);

	DEBUG(KERN_INFO "ICR resume complete\n");

	return 0;
}
#else
#define pxa168_icr_suspend NULL
#define pxa168_icr_resume NULL
#endif /* CONFIG_PM */

static struct platform_driver pxa168_icr_driver = {
	.probe		= pxa168_icr_probe,
	.remove		= __devexit_p(pxa168_icr_remove),
	.suspend	= pxa168_icr_suspend,
	.resume		= pxa168_icr_resume,
	.driver		= {
		.name	= "pxa168-icr",
		.owner	= THIS_MODULE,
	},
};

static int __init pxa168_icr_init(void)
{
	int ret;
	DEBUG("ICR init");
	ret = platform_driver_register(&pxa168_icr_driver);
	if (ret) {
		pr_init(KERN_ERR PFX "unable to register driver\n");
		DEBUG("icr error - 0x%x\n", ret);
		return ret;
	}
	return 0;
}

static void __exit pxa168_icr_exit(void)
{
	DEBUG("ICR exit");
	platform_driver_unregister(&pxa168_icr_driver);
}

module_init(pxa168_icr_init);
module_exit(pxa168_icr_exit);

MODULE_DESCRIPTION("ICR driver for PXA168");
MODULE_AUTHOR("Alan Guenther");
MODULE_LICENSE("GPL");
