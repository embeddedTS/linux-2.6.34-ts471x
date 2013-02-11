/*
 * linux/arch/arm/mach-mmp/devices.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>

#include <mach/devices.h>
#include <mach/cputype.h>
#include <asm/delay.h>
#include <asm/irq.h>

#include <plat/pxa_u2o.h>

int __init pxa_register_device(struct pxa_device_desc *desc,
				void *data, size_t size)
{
	struct platform_device *pdev;
	struct resource res[2 + MAX_RESOURCE_DMA];
	int i, ret = 0, nres = 0;

	if (desc == NULL)
		return -EINVAL;

	pdev = platform_device_alloc(desc->drv_name, desc->id);
	if (pdev == NULL)
		return -ENOMEM;

	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);

	memset(res, 0, sizeof(res));

	if (desc->start != -1ul && desc->size > 0) {
		res[nres].start	= desc->start;
		res[nres].end	= desc->start + desc->size - 1;
		res[nres].flags	= IORESOURCE_MEM;
		nres++;
	}

	if (desc->irq != NO_IRQ) {
		res[nres].start	= desc->irq;
		res[nres].end	= desc->irq;
		res[nres].flags	= IORESOURCE_IRQ;
		nres++;
	}

	for (i = 0; i < MAX_RESOURCE_DMA; i++, nres++) {
		if (desc->dma[i] == 0)
			break;

		res[nres].start	= desc->dma[i];
		res[nres].end	= desc->dma[i];
		res[nres].flags	= IORESOURCE_DMA;
	}

	ret = platform_device_add_resources(pdev, res, nres);
	if (ret) {
		platform_device_put(pdev);
		return ret;
	}

	if (data && size) {
		ret = platform_device_add_data(pdev, data, size);
		if (ret) {
			platform_device_put(pdev);
			return ret;
		}
	}

	return platform_device_add(pdev);
}

PXA168_DEVICE(uart1, "pxa2xx-uart", 0, UART1, 0xd4017000, 0x30, 19, 20);
PXA168_DEVICE(uart2, "pxa2xx-uart", 1, UART2, 0xd4018000, 0x30, 21, 22);
/* special case for avengers to id = 0 (ttyS0) */
#warning mapping uart2 to uart1 for avengers
PXA168_DEVICE(uart1b, "pxa2xx-uart", 0, UART2, 0xd4018000, 0x30, 21, 22);
PXA168_DEVICE(uart3, "pxa2xx-uart", 2, UART3, 0xd4026000, 0x30, 23, 24);
PXA168_DEVICE(twsi0, "pxa2xx-i2c", 0, TWSI0, 0xd4011000, 0x28);
PXA168_DEVICE(twsi1, "pxa2xx-i2c", 1, TWSI1, 0xd4025000, 0x28);
PXA168_DEVICE(ssp0, "pxa168-ssp", 0, SSP0, 0xd401b000, 0x90, 52, 53);
PXA168_DEVICE(ssp1, "pxa168-ssp", 1, SSP1, 0xd401c000, 0x90, 54, 55);
PXA168_DEVICE(ssp2, "pxa168-ssp", 2, SSP2, 0xd401f000, 0x90, 56, 57);
PXA168_DEVICE(ssp3, "pxa168-ssp", 3, SSP3, 0xd4020000, 0x90, 58, 59);
PXA168_DEVICE(ssp4, "pxa168-ssp", 4, SSP4, 0xd4021000, 0x90, 60, 61);
PXA168_DEVICE(fb, "pxa168-fb", -1, LCD, 0xd420b000, 0x1c8);
PXA168_DEVICE(fb_ovly, "pxa168fb_ovly", -1, LCD, 0xd420b000, 0x1c8);
PXA168_DEVICE(keypad, "pxa27x-keypad", -1, KEYPAD, 0xd4012000, 0x4c);
PXA168_DEVICE(nand, "pxa3xx-nand", -1, NAND, 0xD4283000, 0x200, 97, 99);
PXA168_DEVICE(onenand, "onenand", -1, NONE, 0x80000000, 0x100000);
PXA168_DEVICE_M(sdh0, "pxa-sdh", 0, MMC, 0xd4280000, 0x100);
PXA168_DEVICE_M(sdh1, "pxa-sdh", 1, MMC, 0xd4281000, 0x100);
PXA168_DEVICE(sdh2, "pxa-sdh", 2, MMC2, 0xd427e000, 0x100);
PXA168_DEVICE(sdh3, "pxa-sdh", 3, MMC2, 0xd427f000, 0x100);
PXA168_DEVICE(cf, "pxa168-cf", -1, CF, 0xd4285000, 0x800);
PXA168_DEVICE(mfu, "pxa168-mfu", -1, MFU, 0xc0800000, 0x0FFF);
PXA168_DEVICE(pcie, "pxa168-pcie", -1, PCIE_CORE, 0xd1200000, 0x0FFF);
PXA168_DEVICE(camera, "pxa168-camera", -1, CI, 0xd420a000, 0xfff);
PXA168_DEVICE(ov529, "pxa168-ov529", -1, NONE, SMC_CS0_PHYS_BASE, 0x100);
PXA168_DEVICE(msp, "pxa168-msp", -1, MSP, 0xd4286000, 0x0FFF);
PXA168_DEVICE(icr, "pxa168-icr", -1, ICR, 0xC0802000, 0x1000);

/*PXA910 Specific*/
PXA910_DEVICE(uart1, "pxa2xx-uart", 0, UART1, 0xd4017000, 0x30, 21, 22);
PXA910_DEVICE(uart2, "pxa2xx-uart", 1, UART2, 0xd4018000, 0x30, 23, 24);
PXA910_DEVICE(uart3, "pxa2xx-uart", 2, UART3, 0xd4036000, 0x30, 4, 5);
PXA910_DEVICE(ire, "pxa910-ire", -1, IRE, 0xd420C000, 0x90);
PXA910_DEVICE(ssp0, "pxa168-ssp", 0, SSP0, 0xd401b000, 0x90, 52, 53);
PXA910_DEVICE(ssp1, "pxa168-ssp", 1, SSP1, 0xd42a0c00, 0x90,  1, 2);
PXA910_DEVICE(ssp2, "pxa168-ssp", 2, SSP2, 0xd401C000, 0x90, 60, 61);
PXA910_DEVICE(twsi0, "pxa2xx-i2c", 0, TWSI0, 0xd4011000, 0x28);
PXA910_DEVICE(twsi1, "pxa2xx-i2c", 1, TWSI1, 0xd4037000, 0x28);
PXA910_DEVICE(fb, "pxa910-fb", -1, LCD, 0xd420b000, 0x1ec);
PXA910_DEVICE(fb_ovly, "pxa910fb_ovly", -1, LCD, 0xd420b000, 0x1ec);
#ifdef CONFIG_CPU_PXA910
static struct resource pxa910_resource_acipc[] = {
	[0] = {
		.start  = 0xD401D000,
		.end    = 0xD401D0ff,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start  = IRQ_PXA910_IPC_AP_DATAACK,
		.end    = IRQ_PXA910_IPC_AP_DATAACK,
		.flags  = IORESOURCE_IRQ,
		.name   = "IPC_AP_DATAACK",
	},
	[2] = {
		.start  = IRQ_PXA910_IPC_AP_SET_CMD,
		.end    = IRQ_PXA910_IPC_AP_SET_CMD,
		.flags  = IORESOURCE_IRQ,
		.name   = "IPC_AP_SET_CMD",
	},
	[3] = {
		.start  = IRQ_PXA910_IPC_AP_SET_MSG,
		.end    = IRQ_PXA910_IPC_AP_SET_MSG,
		.flags  = IORESOURCE_IRQ,
		.name   = "IPC_AP_SET_MSG",
	},
};

struct platform_device pxa910_device_acipc = {
       .name           = "pxa930-acipc",
       .id             = -1,
       .resource       = pxa910_resource_acipc,
       .num_resources  = ARRAY_SIZE(pxa910_resource_acipc),
};
#endif

#if defined (CONFIG_USB) || defined (CONFIG_USB_GADGET)

/*****************************************************************************
 * The registers read/write routines
 *****************************************************************************/

unsigned u2o_get(unsigned base, unsigned offset)
{
	return readl(base + offset);
}

void u2o_set(unsigned base, unsigned offset, unsigned value)
{
	volatile unsigned int reg;

	reg = readl(base + offset);
	reg |= value;
	writel(reg, base + offset);
	__raw_readl(base + offset);

}

void u2o_clear(unsigned base, unsigned offset, unsigned value)
{
	volatile unsigned int reg;

	reg = readl(base + offset);
	reg &= ~value;
	writel(reg, base + offset);
	__raw_readl(base + offset);
}

void u2o_write(unsigned base, unsigned offset, unsigned value)
{
	writel(value, base + offset);
	__raw_readl(base + offset);

}

/********************************************************************
 * USB 2.0 OTG controller
 */
int pxa168_usb_phy_init(unsigned base)
{	
	static int init_done;
	int count;

	if (init_done) {
		printk(KERN_DEBUG "re-init phy\n\n");
		/* return; */
	}

	/* enable the pull up */
	if (cpu_is_pxa910_z0()) {
		if (cpu_is_pxa910()) {
			u32 U2H_UTMI_CTRL = 
				(u32)ioremap_nocache(0xc0000004, 4);
			writel(1<<20, U2H_UTMI_CTRL);
		}
	}

	/* Initialize the USB PHY power */
	if (cpu_is_pxa910()) {
		u2o_set(base, UTMI_CTRL, (1<<UTMI_CTRL_INPKT_DELAY_SOF_SHIFT)
			| (1<<UTMI_CTRL_PU_REF_SHIFT));
	}

	u2o_set(base, UTMI_CTRL, 1<<UTMI_CTRL_PLL_PWR_UP_SHIFT);
	u2o_set(base, UTMI_CTRL, 1<<UTMI_CTRL_PWR_UP_SHIFT);
	
	/* UTMI_PLL settings */
	u2o_clear(base, UTMI_PLL, UTMI_PLL_PLLVDD18_MASK
		| UTMI_PLL_PLLVDD12_MASK | UTMI_PLL_PLLCALI12_MASK
		| UTMI_PLL_FBDIV_MASK | UTMI_PLL_REFDIV_MASK
		| UTMI_PLL_ICP_MASK | UTMI_PLL_KVCO_MASK);

	u2o_set(base, UTMI_PLL, 0xee<<UTMI_PLL_FBDIV_SHIFT
		| 0xb<<UTMI_PLL_REFDIV_SHIFT | 3<<UTMI_PLL_PLLVDD18_SHIFT
		| 3<<UTMI_PLL_PLLVDD12_SHIFT | 3<<UTMI_PLL_PLLCALI12_SHIFT
		| 2<<UTMI_PLL_ICP_SHIFT | 3<<UTMI_PLL_KVCO_SHIFT);

	/* UTMI_TX */
	u2o_clear(base, UTMI_TX, UTMI_TX_TXVDD12_MASK
		| UTMI_TX_CK60_PHSEL_MASK | UTMI_TX_IMPCAL_VTH_MASK);
	u2o_set(base, UTMI_TX, 3<<UTMI_TX_TXVDD12_SHIFT
		| 4<<UTMI_TX_CK60_PHSEL_SHIFT | 5<<UTMI_TX_IMPCAL_VTH_SHIFT);

	/* UTMI_RX */
	u2o_clear(base, UTMI_RX, UTMI_RX_SQ_THRESH_MASK
		| UTMI_REG_SQ_LENGTH_MASK);
	if (cpu_is_pxa168())
		u2o_set(base, UTMI_RX, 7<<UTMI_RX_SQ_THRESH_SHIFT
			| 2<<UTMI_REG_SQ_LENGTH_SHIFT);
	else
		u2o_set(base, UTMI_RX, 0xa<<UTMI_RX_SQ_THRESH_SHIFT
			| 2<<UTMI_REG_SQ_LENGTH_SHIFT);

	/* UTMI_IVREF */
	if (cpu_is_pxa168())
		/* fixing Microsoft Altair board interface with NEC hub issue -
		 * Set UTMI_IVREF from 0x4a3 to 0x4bf */
		u2o_write(base, UTMI_IVREF, 0x4bf);

	/* calibrate */
	count = 10000;
	while(((u2o_get(base, UTMI_PLL) & PLL_READY)==0) && count--);
	if (count <= 0) printk("%s %d: calibrate timeout, UTMI_PLL %x\n", 
		__func__, __LINE__, u2o_get(base, UTMI_PLL));

	/* toggle VCOCAL_START bit of UTMI_PLL */
	udelay(200);
	u2o_set(base, UTMI_PLL, VCOCAL_START);
	udelay(40);
	u2o_clear(base, UTMI_PLL, VCOCAL_START);

	/* toggle REG_RCAL_START bit of UTMI_TX */
	udelay(200);
	u2o_set(base, UTMI_TX, REG_RCAL_START);
	udelay(40);
	u2o_clear(base, UTMI_TX, REG_RCAL_START);
	udelay(200);

	/* make sure phy is ready */
	count = 1000;
	while(((u2o_get(base, UTMI_PLL) & PLL_READY)==0) && count--);
	if (count <= 0) printk("%s %d: calibrate timeout, UTMI_PLL %x\n", 
		__func__, __LINE__, u2o_get(base, UTMI_PLL));

	if (cpu_is_pxa168()) {
		u2o_set(base, UTMI_RESERVE, 1<<5);
		u2o_write(base, UTMI_OTG_ADDON, 1);  /* Turn on UTMI PHY OTG extension */
	}

	init_done = 1;
	return 0;
}

int pxa168_usb_phy_deinit(unsigned base)
{
	if (cpu_is_pxa168())
		u2o_clear(base, UTMI_OTG_ADDON, UTMI_OTG_ADDON_OTG_ON);

	u2o_clear(base, UTMI_CTRL, UTMI_CTRL_RXBUF_PDWN);
	u2o_clear(base, UTMI_CTRL, UTMI_CTRL_TXBUF_PDWN);
	u2o_clear(base, UTMI_CTRL, UTMI_CTRL_USB_CLK_EN);
	u2o_clear(base, UTMI_CTRL, 1<<UTMI_CTRL_PWR_UP_SHIFT);
	u2o_clear(base, UTMI_CTRL, 1<<UTMI_CTRL_PLL_PWR_UP_SHIFT);

	return 0;
}

static u64 u2o_dma_mask = ~(u32)0;
struct resource pxa168_u2o_resources[] = {
	/* regbase */
	[0] = {
		.start	= PXA168_U2O_REGBASE,
		.end	= PXA168_U2O_REGBASE + USB_REG_RANGE,
		.flags	= IORESOURCE_MEM,
		.name	= "u2o",
	},
	/* phybase */
	[1] = {
		.start	= PXA168_U2O_PHYBASE,
		.end	= PXA168_U2O_PHYBASE + USB_PHY_RANGE,
		.flags	= IORESOURCE_MEM,
		.name	= "u2ophy",
	},
	[2] = {
		.start	= IRQ_PXA168_USB1,
		.end	= IRQ_PXA168_USB1,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device pxa168_device_u2o = {
	.name		= "pxa-u2o",
	.id		= -1,
	.resource	= pxa168_u2o_resources,
	.num_resources	= ARRAY_SIZE(pxa168_u2o_resources),
	.dev		=  {
		.dma_mask	= &u2o_dma_mask,
		.coherent_dma_mask = 0xffffffff,
	}
};
#endif

/********************************************************************
 * USB 2.0 Dedicated Host controller
 */
static u64 ehci_hcd_pxa_dmamask = DMA_BIT_MASK(32);
static void ehci_hcd_pxa_device_release(struct device *dev)
{
        /* Keep this function empty. */
}

#ifdef CONFIG_USB_EHCI_PXA_U2H
static struct resource pxa168_u2h_resources[] = {
	/* regbase */
	[0] = {
		.start	= PXA168_U2H_REGBASE,
		.end	= PXA168_U2H_REGBASE + USB_REG_RANGE,
		.flags	= IORESOURCE_MEM,
		.name	= "u2h",
	},
	/* phybase */
	[1] = {
		.start	= PXA168_U2H_PHYBASE,
		.end	= PXA168_U2H_PHYBASE + USB_PHY_RANGE,
		.flags	= IORESOURCE_MEM,
		.name	= "u2hphy",
	},
	[2] = {
		.start	= IRQ_PXA168_USB2,
		.end	= IRQ_PXA168_USB2,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device pxa168_device_u2h = {
        .name = "pxau2h-ehci",
        .id   = -1,
        .dev  = {
                .dma_mask = &ehci_hcd_pxa_dmamask,
                .coherent_dma_mask = DMA_BIT_MASK(32),
                .release = ehci_hcd_pxa_device_release,
        },

        .num_resources = ARRAY_SIZE(pxa168_u2h_resources),
        .resource      = pxa168_u2h_resources,
};
#endif

#if defined(CONFIG_USB_PXA_U2O) && defined(CONFIG_USB_OTG)
struct resource pxa168_u2ootg_resources[] = {
	/* regbase */
	[0] = {
		.start	= PXA168_U2O_REGBASE,
		.end	= PXA168_U2O_REGBASE + USB_REG_RANGE,
		.flags	= IORESOURCE_MEM,
		.name	= "u2o",
	},
	/* phybase */
	[1] = {
		.start	= PXA168_U2O_PHYBASE,
		.end	= PXA168_U2O_PHYBASE + USB_PHY_RANGE,
		.flags	= IORESOURCE_MEM,
		.name	= "u2ophy",
	},
	[2] = {
		.start	= IRQ_PXA168_USB1,
		.end	= IRQ_PXA168_USB1,
		.flags	= IORESOURCE_IRQ,
	},
};

struct resource pxa168_u2oehci_resources[] = {
	/* regbase */
	[0] = {
		.start	= PXA168_U2O_REGBASE,
		.end	= PXA168_U2O_REGBASE + USB_REG_RANGE,
		.flags	= IORESOURCE_MEM,
		.name	= "u2o",
	},
	/* phybase */
	[1] = {
		.start	= PXA168_U2O_PHYBASE,
		.end	= PXA168_U2O_PHYBASE + USB_PHY_RANGE,
		.flags	= IORESOURCE_MEM,
		.name	= "u2ophy",
	},
	[2] = {
		.start	= IRQ_PXA168_USB1,
		.end	= IRQ_PXA168_USB1,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device pxa168_device_u2ootg = {
	.name		= "pxa-otg",
	.id		= -1,
	.dev  = {
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},

	.num_resources	= ARRAY_SIZE(pxa168_u2ootg_resources),
	.resource      = pxa168_u2ootg_resources,
};

struct platform_device pxa168_device_u2oehci = {
	.name = "pxau2o-ehci",
	.id = -1,
	.dev		= {
		.dma_mask = &ehci_hcd_pxa_dmamask,
		.coherent_dma_mask = 0xffffffff,
		.release = ehci_hcd_pxa_device_release,
	},

	.num_resources = ARRAY_SIZE(pxa168_u2o_resources),
	.resource      = pxa168_u2oehci_resources,
};

#endif

#ifdef CONFIG_CPU_PXA910
static struct resource pxa910_resource_imm[] = {
        [0] = {
	                        .name   = "phy_sram",
	                        .start  = 0xd1000000 + SZ_64K ,
	                        .end    = 0xd1000000 + SZ_128K - 1,
	                        .flags  = IORESOURCE_MEM,
	                },
        [1] = {
	                        .name   = "imm_sram",
	                        .start  = 0xd1000000 + SZ_64K,
	                        .end    = 0xd1000000 + SZ_128K - 1,
	                        .flags  = IORESOURCE_MEM,
	                },
};

struct platform_device pxa910_device_imm = {
        .name           = "pxa3xx-imm",
        .id             = -1,
        .num_resources  = ARRAY_SIZE(pxa910_resource_imm),
	.resource       = pxa910_resource_imm,
};

static struct resource pxa168_resource_pwm1[] = {
	[0] = {
		.start  = 0xD401A400,
		.end    = 0xD401A40B,
		.flags  = IORESOURCE_MEM,
	},
};

struct platform_device pxa168_device_pwm1 = {
	.name           = "pxa168-pwm",
	.id             = 1,
	.resource       = pxa168_resource_pwm1,
	.num_resources  = ARRAY_SIZE(pxa168_resource_pwm1),
};

static struct resource pxa168_resource_pwm2[] = {
	[0] = {
		.start  = 0xD401A800,
		.end    = 0xD401A80B,
		.flags  = IORESOURCE_MEM,
	},
};

struct platform_device pxa168_device_pwm2 = {
	.name           = "pxa168-pwm",
	.id             = 2,
	.resource       = pxa168_resource_pwm2,
	.num_resources  = ARRAY_SIZE(pxa168_resource_pwm2),
};

static struct resource pxa168_resource_pwm3[] = {
	[0] = {
		.start  = 0xD401AC00,
		.end    = 0xD401AC0B,
		.flags  = IORESOURCE_MEM,
	},
};

struct platform_device pxa168_device_pwm3 = {
	.name           = "pxa168-pwm",
	.id             = 3,
	.resource       = pxa168_resource_pwm3,
	.num_resources  = ARRAY_SIZE(pxa168_resource_pwm3),
};
#endif

static struct resource pxa168_resource_pwm0[] = {
	[0] = {
		.start  = 0xD401A000,
		.end    = 0xD401A00B,
		.flags  = IORESOURCE_MEM,
	},
};

struct platform_device pxa168_device_pwm0 = {
	.name           = "pxa168-pwm",
	.id             = 0,
	.resource       = pxa168_resource_pwm0,
	.num_resources  = ARRAY_SIZE(pxa168_resource_pwm0),
};

static struct resource pxa168_resource_freq[] = {
	[0] = {
		.name   = "pmum_regs",
		.start	= 0xd4050000,
		.end	= 0xd4051050,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.name   = "pmua_regs",
		.start	= 0xd4282800,
		.end	= 0xd4282900,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device pxa168_device_freq = {
	.name		= "pxa168-freq",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(pxa168_resource_freq),
	.resource	= pxa168_resource_freq,
};

static struct resource pxa910_resource_rtc[] = {
	[0] = {
		.start  = 0xd4010000,
		.end    = 0xD40100ff,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start  = IRQ_PXA168_RTC_INT,
		.end    = IRQ_PXA168_RTC_INT,
		.flags  = IORESOURCE_IRQ,
		.name   = "RTC_1HZ",
	},

	[2] = {
		.start  = IRQ_PXA168_RTC_ALARM,
		.end    = IRQ_PXA168_RTC_ALARM,
		.flags  = IORESOURCE_IRQ,
		.name   = "RTC_ALARM",
	},

};

struct platform_device pxa910_device_rtc = {
       .name           = "mmp-rtc",
       .id             = -1,
       .resource       = pxa910_resource_rtc,
       .num_resources  = ARRAY_SIZE(pxa910_resource_rtc),
};


struct platform_device pxa168_device_cs4344 = {
	.name           = "cs4344",
	.id             = -1,
};

#if defined(CONFIG_CIR)
struct resource pxa168_resource_cir[] = {
        [0] = {
                .start  = 0xD4019100,
                .end    = 0xD4019100,
                .flags  = IORESOURCE_MEM,
        },

        [1] = {
                .start  = IRQ_GPIO(102),
                .end    = IRQ_GPIO(102),
                .flags  = IORESOURCE_IRQ,
                },
};

struct platform_device pxa168_device_cir = {
        .name           = "aspenite-cir",
        .resource       = pxa168_resource_cir,
        .num_resources  = ARRAY_SIZE(pxa168_resource_cir),
};
#endif
