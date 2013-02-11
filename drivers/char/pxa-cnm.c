/*
 *  pxa-cnm main driver for chip&media codad8x codec
 *
 *  Copyright (C) 2009, Marvell International Ltd.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/device.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/uio.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/fb.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/miscdevice.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/page.h>
#include <asm/mman.h>
#include <asm/pgtable.h>
#include <asm/uaccess.h>
#include <asm/cacheflush.h>

#include <mach/regs-apmu.h>
#include <mach/cputype.h>



/*FPGA addr*/
//#define CNM_MEM_START		0x80000000 

/*TTC EVB*/
#define CNM_MEM_START		0xD420D000
#define CNM_MEM_SIZE		0x010000
#define MAX_NUM_INSTANCE	4

/*Power On*/
////////////////////////////////////////////////////////////
/////
//  // DX8_CLK_RES definition:
    // bit[1:0]: {func_reset, axi_reset}
    // bit[4:3]: {func_clk_en, axi_clk_en}
    // bit[10:6]:  dx8_clk_div
    // bit[18:16]: {pwr_on2, pwr_on1, isob}
    // bit[19]: hw_mode
#define DX8_AXI_RST   0x1
#define DX8_FUNC_RST  0x2
#define DX8_RST_OFF       0x3
#define DX8_FUNC_CLK  0x10
#define DX8_AXI_CLK   0x8
#define DX8_CLK_DIV   0x40
#define DX8_CLK_SEL_208   0x20
#define DX8_CLK_EN    0x58
#define DX8_HW_MODE   0x80000
#define DX8_ON2       0x40000
#define DX8_ON1       0x20000
#define DX8_PWR_ON    0x60000
#define DX8_ISO       0x10000

#if 0
#define PMUA_PWR_STAT  0xd42828F0
#define  PMUA_DX8_CLK_RES  0xd42828A4

#define PMU_GC_CLK_RES_CTRL     0xD42828CC
//Dx8 clk_reset ctrl
//#define ADDR_VPRO_CLK_RES_CTRL  0xD42828A4
#define PMU_DX8_CLK_RES_CTRL    0xD42828A4
#define PMUA_PWR_TIMER      0xd42828DC
#define PMUA_PWR_CTRL 0xd42828D8

//#define APMU_CF         	APMU_REG(0x0f0)
//#define APMU_DX8_CLK_RES_CTRL	APMU_REG(0x00a4)
//#define APMU_XD_CLK_RES_CTRL	APMU_REG(0x00dc)
#endif


struct vpu_info {
	unsigned int	off;
	void		*value;
};

enum {
	VPU_READREG = 1,
	VPU_WRITEREG,
	VPU_WAITFORTIMEOUT,
};

struct pxa_cnm {
	struct clk              *clk;
	void __iomem		*mmio_base;
	int			irq;
	int			vpu_downloaded;
	int			current_act;
	int			ins_num;
	struct mutex		lock;
};

static struct pxa_cnm pxa_cnm;
void VpuWriteReg(unsigned int off, unsigned int value)
{
	__raw_writel((value), pxa_cnm.mmio_base + (off));
}

unsigned int VpuReadReg(unsigned int off)
{
	return __raw_readl(pxa_cnm.mmio_base + (off));
}

static int pxa_cnm_open(struct inode *inode, struct file *file)
{
	int ret = 0;

	lock_kernel();
	if (pxa_cnm.ins_num < MAX_NUM_INSTANCE)
		pxa_cnm.ins_num ++;
	else {
		printk("Current CNM cannot accept more instance!!\n");
		ret = -EACCES;
		goto out;
	}

    	printk("Current CNM Get %d instance!!\n", pxa_cnm.ins_num);
	file->private_data = &pxa_cnm;
out:
	unlock_kernel();
	return ret;
}

static int pxa_cnm_close(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	mutex_lock(&pxa_cnm.lock);
	pxa_cnm.ins_num --;
	mutex_unlock(&pxa_cnm.lock);

	return 0;
}

static int pxa_cnm_ioctl
(struct inode *inode, struct file *file, u_int cmd, u_long arg)
{
	void __user *argp = (void __user *)arg;
	struct vpu_info vpu_info;
	unsigned int value;

	if (copy_from_user(&vpu_info, argp, sizeof(struct vpu_info)))
		return -EFAULT;

	switch (cmd) {
	case VPU_READREG:
	{
		value = VpuReadReg(vpu_info.off);
		if (copy_to_user(vpu_info.value, &value, sizeof(unsigned int)))
			return -EFAULT;
		break;
	}
	case VPU_WRITEREG:
	{
		if (copy_from_user(&value, vpu_info.value, sizeof(unsigned int)))
			return -EFAULT;

		VpuWriteReg(vpu_info.off, value);
		break;
	}
	case VPU_WAITFORTIMEOUT:
	{
		break;
	}
	default:
	{
		return -EFAULT;
	}
	}

	return 0;
}

static struct file_operations pxa_cnm_fops = {
	.owner          = THIS_MODULE,
	.open           = pxa_cnm_open,
	.release        = pxa_cnm_close,
	.ioctl          = pxa_cnm_ioctl,
};

static struct miscdevice pxa_cnm_miscdev = {
	.minor  = MISC_DYNAMIC_MINOR,
	.name   = "cnm",
	.fops   = &pxa_cnm_fops,
	.this_device = NULL,
};

static irqreturn_t pxa_cnm_irq (int irq, void *devid)
{
	return IRQ_HANDLED;
}

static int pxa_cnm_clock(int enable)
{
	if(enable == 1){
		/*Enable*/
		/*Power On*/
		printk("pxa_cnm_clock enable\n");

		/*DX8_CLK check*/
		printk("Dx8 Read ...APMU_DX8_CLK_RES_CTRL = 0x%.8x\n", __raw_readl(APMU_DX8_CLK_RES_CTRL));		

		// DX8_CLK_RES definition:
		// bit[1:0]: {func_reset, axi_reset}
		// bit[4:3]: {func_clk_en, axi_clk_en}
		// bit[10:6]:  dx8_clk_div
		// bit[18:16]: {pwr_on2, pwr_on1, isob}
		// bit[19]: hw_mode
		// set power timer@vctcxo (HW mode only)
    		// SW turn-on GC/Dx8 power, and enable clock
     		//  BU_REG_WRITE(PMUA_PWR_TIMER,0x0f0f0f); // on1, on2, off timer are 0x0F.
		__raw_writel(0x0f0f0f, APMU_XD_CLK_RES_CTRL);
        	// SW turn-on GC/Dx8 power, and enable clock
  		// BU_REG_WRITE(PMUA_DX8_CLK_RES, DX8_CLK_DIV | DX8_FUNC_CLK | DX8_AXI_CLK);  // clock enable
  		// Enable Bus Clock and Function clocks
		__raw_writel(DX8_CLK_DIV | DX8_FUNC_CLK | DX8_AXI_CLK, APMU_DX8_CLK_RES_CTRL);
  		// BU_REG_WRITE(PMUA_DX8_CLK_RES, DX8_CLK_EN | DX8_ON1);  // on1
  		// BU_REG_WRITE(PMUA_DX8_CLK_RES,  DX8_CLK_EN | DX8_ON1 | DX8_ON2);  // on2, 200us latency required
  		// Enable power_on1 wait for at least 200us , eanble power_on2
		__raw_writel(DX8_CLK_EN | DX8_ON1, APMU_DX8_CLK_RES_CTRL);

		//printk("Dx8 Power On1, wait 20\n");
		schedule_timeout(20);

		__raw_writel(DX8_CLK_EN | DX8_ON1 | DX8_ON2, APMU_DX8_CLK_RES_CTRL);

		//printk("Dx8 Power On2, wait 20\n");
		schedule_timeout(20);

		//printk("Dx8 Reset ...APMU_DX8_CLK_RES_CTRL = 0x%.8x\n", __raw_readl(APMU_DX8_CLK_RES_CTRL));

		// SW release reset
		// BU_REG_WRITE(PMUA_DX8_CLK_RES, DX8_CLK_EN | DX8_PWR_ON | DX8_FUNC_RST);   // resetPin_ first
		// BU_REG_WRITE(PMUA_DX8_CLK_RES, DX8_CLK_EN | DX8_PWR_ON | DX8_FUNC_RST | DX8_AXI_RST);   // areset/hreset 48 cycles needed
		__raw_writel(DX8_CLK_EN | DX8_PWR_ON | DX8_FUNC_RST, APMU_DX8_CLK_RES_CTRL);
		__raw_writel(DX8_CLK_EN | DX8_PWR_ON | DX8_FUNC_RST | DX8_AXI_RST, APMU_DX8_CLK_RES_CTRL);

        	// SW enable FW
		// BU_REG_WRITE(PMUA_DX8_CLK_RES, DX8_CLK_EN | DX8_PWR_ON | DX8_RST_OFF | DX8_ISO);   // GC is fully functional now
		__raw_writel(DX8_CLK_EN | DX8_PWR_ON | DX8_RST_OFF | DX8_ISO , APMU_DX8_CLK_RES_CTRL);
		//BU_REG_READ(PMUA_DX8_CLK_RES) != (DX8_CLK_EN | DX8_PWR_ON | DX8_RST_OFF | DX8_ISO
        	if( __raw_readl(APMU_DX8_CLK_RES_CTRL) != (DX8_CLK_EN | DX8_PWR_ON | DX8_RST_OFF | DX8_ISO ) ) {
		/*Enable 208M AXI bridget*/
		//__raw_writel(DX8_CLK_EN | DX8_PWR_ON | DX8_RST_OFF | DX8_ISO | DX8_CLK_SEL_208, APMU_DX8_CLK_RES_CTRL);
        	//if( __raw_readl(APMU_DX8_CLK_RES_CTRL) != (DX8_CLK_EN | DX8_PWR_ON | DX8_RST_OFF | DX8_ISO | DX8_CLK_SEL_208) ) {
        		printk("---------PMUA_DX8_CLK_RES error!--------\n");  // reg read test
			return -1;
		}else{
			printk("Dx8 Reset ...APMU_DX8_CLK_RES_CTRL = 0x%.8x\n", __raw_readl(APMU_DX8_CLK_RES_CTRL));
		}

		/*Test*/
#if 0
	        {
			unsigned int retval = 0;
			printk("test register 0x120, init 0x%x\n", VpuReadReg(0x120));

			VpuWriteReg(0x120, 0x12345678);
			retval = VpuReadReg(0x120);
			printk("ret value 0x%x\n", retval);
	
		}
#endif

	}else{
  // *PMUA_DX8_CLK_RES = DX8_CLK_EN | DX8_PWR_ON | DX8_RST_OFF;  // shutdown FW
  // *PMUA_DX8_CLK_RES = DX8_CLK_EN | DX8_ON1 | DX8_RST_OFF;   // on2 off
  //*PMUA_DX8_CLK_RES = DX8_CLK_EN | DX8_RST_OFF ;   // on1 off, GC power down now.
  //if(*PMUA_DX8_CLK_RES != (DX8_CLK_EN | DX8_RST_OFF) ) {
  //      *ARM_MSG = 0xf0f0; *ARM_MSG = SIM_END;  // reg read test
  //}
		/*Disable*/
		printk("pxa_cnm_clock disable\n");
		__raw_writel(DX8_CLK_EN | DX8_PWR_ON | DX8_RST_OFF , APMU_DX8_CLK_RES_CTRL);
		__raw_writel(DX8_CLK_EN | DX8_ON1 | DX8_RST_OFF , APMU_DX8_CLK_RES_CTRL);
		__raw_writel(DX8_CLK_EN | DX8_RST_OFF  , APMU_DX8_CLK_RES_CTRL);
        	if( __raw_readl(APMU_DX8_CLK_RES_CTRL) != (DX8_CLK_EN | DX8_RST_OFF ) ) {
			printk("error in close\n");
			return -1;
		}
	}
	return 0;
}

static int __init pxa_cnm_probe(struct platform_device *pdev)
{
	struct resource *r;
	struct pxa_cnm *cnm = &pxa_cnm;
	int irq, ret;

	platform_set_drvdata(pdev, cnm);
#if 0
	cnm->clk = clk_get(&pdev->dev, "CNMCLK");
	if (IS_ERR(cnm->clk)) {
		dev_err(&pdev->dev, "failed to get cnm clock\n");
		goto out;
	}
	clk_enable(cnm->clk);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "no IRQ resource defined\n");
		goto fail_put_clk;
	}

	pxa_cnm.irq = irq;
	ret = request_irq(irq, pxa_cnm_irq, 0, "cnm", NULL);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to request IRQ\n");
		goto fail_free_irq;
	}

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (r == NULL)
		goto fail_free_irq;
#endif

	r = request_mem_region(CNM_MEM_START, CNM_MEM_SIZE, "cnm");
	if (r == NULL) {
		dev_err(&pdev->dev, "failed to request memory resource\n");
		goto fail_free_irq;
	}

	cnm->mmio_base = ioremap(CNM_MEM_START, CNM_MEM_SIZE);
	if(cnm->mmio_base == NULL)
		goto fail_free_res;

	ret = misc_register(&pxa_cnm_miscdev);
	if (ret < 0) {
		dev_err(&pdev->dev,
				"unable to register device node /dev/cnm\n");
		goto fail_free_res;
	}

	mutex_init(&pxa_cnm.lock);

	/*Probe*/
	printk(" CnM Probe\n");
	/*CLock Enable*/

	if(pxa_cnm_clock(1) != 0){
		goto fail_free_res;
	};

	return 0;

fail_free_res:
	release_mem_region(r->start, r->end - r->start + 1);
fail_free_irq:
	free_irq(irq, &pxa_cnm);
fail_put_clk:
	clk_disable(pxa_cnm.clk);
	clk_put(pxa_cnm.clk);
out:
	return -ENODEV;
}

static int pxa_cnm_remove(struct platform_device *pdev)
{
	misc_deregister(&pxa_cnm_miscdev);
	//free_irq(pxa_cnm.irq, &pxa_cnm);
	printk("CnM Remove\n");

	return 0;
}

#define pxa_cnm_suspend      NULL
#define pxa_cnm_resume       NULL

static struct resource cnm_resource[] = {
	[0] = {
		.start	= CNM_MEM_START,
		.end	= CNM_MEM_START + CNM_MEM_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_driver cnm_driver = {
	.driver         = {
		.name   = "pxa-cnm",
	},
	.probe          = pxa_cnm_probe,
	.remove         = pxa_cnm_remove,
	.suspend        = pxa_cnm_suspend,
	.resume         = pxa_cnm_resume,
};

static struct platform_device cnm_device = {
	.name           = "pxa-cnm",
	.id             = 0,
	.dev            = {
	},
	.num_resources  = ARRAY_SIZE(cnm_resource),
	.resource       = cnm_resource,
};

static int __init pxa_cnm_init(void)
{
	int ret;
//#if defined(CONFIG_PXA3xx_DVFM)
//	dvfm_register("MVED", &dvfm_lock.dev_idx);
//	dvfm_register_notifier(&notifier_freq_block,
//			DVFM_FREQUENCY_NOTIFIER);
//#endif
	if(!cpu_is_pxa910_Ax())
		return -1;
	ret = platform_driver_register(&cnm_driver);
	platform_device_register(&cnm_device);
	return ret;
}

static void __exit pxa_cnm_exit(void)
{
//#if defined(CONFIG_PXA3xx_DVFM)
//	dvfm_unregister_notifier(&notifier_freq_block,
//			DVFM_FREQUENCY_NOTIFIER);
//	dvfm_unregister("CNM", &dvfm_lock.dev_idx);
//#endif
	platform_driver_unregister(&cnm_driver);
}

module_init(pxa_cnm_init);
module_exit(pxa_cnm_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Lei Wen (leiwen@marvell.com)");
MODULE_DESCRIPTION("Chip and Media CodaDx8 codec Driver");
