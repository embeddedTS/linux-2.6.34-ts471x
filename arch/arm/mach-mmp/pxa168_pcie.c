/*
 * arch/arm/mach-mmp/pxa168_pcie.c
 *
 * PCIe functions for Marvell PXA168 SoC
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/mbus.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <asm/mach/pci.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>
#include <asm/delay.h>
#include <mach/pxa168_pcie.h>
#include <mach/irqs.h>
#include <mach/regs-pcie.h>
#include "common.h"

/* #define PCIE_DEBUG_CODE */
#if defined(PCIE_DEBUG_CODE)
#define PCIE_DEBUG(fmt, arg...)			        \
	do {						\
		printk(KERN_ERR				\
			"[pcie debug: %s] " fmt ,	\
			__func__ , ##arg);		\
	} while (0)
#else
#define PCIE_DEBUG(fmt, arg...)		do { } while (0)
#endif

/*
 * PCIe unit register offsets.
 */
#define PCIE_CONF_BUS(b)	(((b) & 0xff) << 24)
#define PCIE_CONF_DEV(d)	(((d) & 0x1f) << 19)
#define PCIE_CONF_FUNC(f)	(((f) & 0x7) << 16)
#define PCIE_CONF_REG(r)	(((r) & 0xfff))
#define CONFIG_MEM		1
#define IO_MEM			0

/* Size in Bytes */
#define SIZE_8			1
#define SIZE_16			2
#define SIZE_32			4

static atomic_t ch = ATOMIC_INIT(-1);

#define NUM_PCIE_PORTS 1
#define NUM_PCIE_CHANNELS 4
#define MAX_WAIT_LOOP 2000
#define TIME_DELAY 2

#define LINKUP_TIMEOUT 100
#define LINKUP_DELAY 100


extern int pxa168_gpio_pcie_init(void);

struct pxa168_pcie_channel_desc {
	void __iomem *dma_pcie_ctrl_ch;
	void __iomem *pio_addr_ls_ch;
	void __iomem *pio_addr_ms_ch;
	void __iomem *dma_mode_ch;
	void __iomem *pio_wr_strb_ch;
	void __iomem *pio_start_ch;
	void __iomem *pio_rd_data_ch;
	void __iomem *pio_wr_data_ch;
};

static struct pxa168_pcie_channel_desc pcie_channels[] = {
	{PCIE_REG(PCIE_DMA_PCIE_CTRL_CH0),	/* Channel 0 */
	 PCIE_REG(PCIE_PIO_ADDR_LS_CH0),
	 PCIE_REG(PCIE_PIO_ADDR_MS_CH0),
	 PCIE_REG(PCIE_DMA_MODE_CH0),
	 PCIE_REG(PCIE_PIO_WR_STRB_CH0),
	 PCIE_REG(PCIE_PIO_START_CH0),
	 PCIE_REG(PCIE_PIO_RD_DATA_CH0),
	 PCIE_REG(PCIE_PIO_WR_DATA_CH0)},
	{PCIE_REG(PCIE_DMA_PCIE_CTRL_CH1),	/* Channel 1 */
	 PCIE_REG(PCIE_PIO_ADDR_LS_CH1),
	 PCIE_REG(PCIE_PIO_ADDR_MS_CH1),
	 PCIE_REG(PCIE_DMA_MODE_CH1),
	 PCIE_REG(PCIE_PIO_WR_STRB_CH1),
	 PCIE_REG(PCIE_PIO_START_CH1),
	 PCIE_REG(PCIE_PIO_RD_DATA_CH1),
	 PCIE_REG(PCIE_PIO_WR_DATA_CH1)},
	{PCIE_REG(PCIE_DMA_PCIE_CTRL_CH2),	/* Channel 2 */
	 PCIE_REG(PCIE_PIO_ADDR_LS_CH2),
	 PCIE_REG(PCIE_PIO_ADDR_MS_CH2),
	 PCIE_REG(PCIE_DMA_MODE_CH2),
	 PCIE_REG(PCIE_PIO_WR_STRB_CH2),
	 PCIE_REG(PCIE_PIO_START_CH2),
	 PCIE_REG(PCIE_PIO_RD_DATA_CH2),
	 PCIE_REG(PCIE_PIO_WR_DATA_CH2)},
	{PCIE_REG(PCIE_DMA_PCIE_CTRL_CH3),	/* Channel 3 */
	 PCIE_REG(PCIE_PIO_ADDR_LS_CH3),
	 PCIE_REG(PCIE_PIO_ADDR_MS_CH3),
	 PCIE_REG(PCIE_DMA_MODE_CH3),
	 PCIE_REG(PCIE_PIO_WR_STRB_CH3),
	 PCIE_REG(PCIE_PIO_START_CH3),
	 PCIE_REG(PCIE_PIO_RD_DATA_CH3),
	 PCIE_REG(PCIE_PIO_WR_DATA_CH3)},
};

static spinlock_t channel_lock[NUM_PCIE_CHANNELS];


static u32
pxa168_pcie_read_reg(void __iomem * base, u8 is_conf_read, u32 addr,
		     u8 size)
{
	u32 val;
	int i;
	u8 data_strobe;
	u32 aligned_addr;
	unsigned long flags = 0;
	unsigned int channel;

	aligned_addr = addr & ~0x3;	/* Align to DWORD */

	switch (size) {
	case SIZE_8:
		data_strobe = (0x1 << (addr & 0x3));
		break;
	case SIZE_16:
		data_strobe = (0x3 << (addr & 0x3));
		break;
	default:
		data_strobe = 0xf;	/* Size 32 */
		break;
	}
	/* Next channel */
	for (i = 0; i < NUM_PCIE_CHANNELS; i++) {
		channel = atomic_inc_return(&ch) % NUM_PCIE_CHANNELS;
		/* If lock is available or all channel locks were taken  */
		if ((!spin_trylock(&channel_lock[channel])) ||
		    (i >= NUM_PCIE_CHANNELS)) {
			/* then get the lock for a channel */
			spin_lock_irqsave(&channel_lock[channel], flags);
			break;
		}
	}

	/* Configure Channel DMA PCI-E Control Register with
	 * Read Type (Configuration or Memory Read)*/
	if (is_conf_read)
		__raw_writel(((1 << PCIE_DMA_PCIE_CTRL_PCIETD_OFFSET) |
			      (PCIE_DMA_PCIE_CTRL_TLP_CFG_TYPE0_RW &
			       0x1F)),
			     pcie_channels[channel].dma_pcie_ctrl_ch);
	else
		__raw_writel(((1 << PCIE_DMA_PCIE_CTRL_PCIETD_OFFSET) |
			      (PCIE_DMA_PCIE_CTRL_TLP_MEM_RW & 0x1F)),
			     pcie_channels[channel].dma_pcie_ctrl_ch);

	/* Configure Channel PIO Address Register's */
	__raw_writel(aligned_addr, pcie_channels[channel].pio_addr_ls_ch);
	if (!is_conf_read)
		__raw_writel(0x0, pcie_channels[channel].pio_addr_ms_ch);

	/* Configure Channel DMA Mode register to Read/Non-chain Mode */
	__raw_writel((PCIE_DMA_MODE_NONCHAINED |
		      PCIE_DMA_MODE_PCIE_READ),
		     pcie_channels[channel].dma_mode_ch);

	/* Configure Channel PIO Write Data Strobes */
	__raw_writel(data_strobe, pcie_channels[channel].pio_wr_strb_ch);

	/* Start Channel Data Transfer */
	__raw_writel(PCIE_DMA_START_TRANSFER,
		     pcie_channels[channel].pio_start_ch);

	/* Wait for Start Bit to clear which indicates transfer
	 * completion
	 */
	for (i = 0; i < MAX_WAIT_LOOP; i++) {
		if (!__raw_readl(pcie_channels[channel].pio_start_ch))
			break;
		udelay(TIME_DELAY);
	}

	if (likely(i < MAX_WAIT_LOOP)) {
		val = __raw_readl(pcie_channels[channel].pio_rd_data_ch);
		spin_unlock_irqrestore(&channel_lock[channel], flags);

		switch (size) {
		case SIZE_8:
			val = ((val >> (8 * (addr & 0x3))) & 0xff);
			break;
		case SIZE_16:
			val = ((val >> (8 * (addr & 0x3))) & 0xffff);
			break;
		default:
			break;
		}
	} else {
		spin_unlock_irqrestore(&channel_lock[channel], flags);
		PCIE_DEBUG("Error: PIO register read timeout\n");
		val = -1;
	}

	return val;
}

static void pxa168_pcie_write_reg(void __iomem * base, u8 is_conf_write,
				  u32 addr, u8 size, u32 val)
{
	int i;

	u8 data_strobe;
	u32 aligned_addr;
	u32 aligned_val = val;
	unsigned long flags = 0;
	unsigned int channel;

	aligned_addr = addr & ~0x3;	/* Align to DWORD */

	switch (size) {
	case SIZE_8:
		data_strobe = (0x1 << (addr & 0x3));
		/* Align val according to data strobe */
		aligned_val = ((val & 0xff) << (8 * (addr & 0x3)));
		break;
	case SIZE_16:
		data_strobe = (0x3 << (addr & 0x3));
		/* Align val according to data strobe */
		aligned_val = ((val & 0xffff) << (8 * (addr & 0x3)));
		break;
	default:
		data_strobe = 0xf;	/* Size 32 */
		break;
	}

	/* Next channel */
	for (i = 0; i < NUM_PCIE_CHANNELS; i++) {
		channel = atomic_inc_return(&ch) % NUM_PCIE_CHANNELS;
		/* If lock is available or all channel locks were taken  */
		if ((!spin_trylock(&channel_lock[channel])) ||
		    (i >= NUM_PCIE_CHANNELS)) {
			/* then get the lock for a channel */
			spin_lock_irqsave(&channel_lock[channel], flags);
			break;
		}
	}

	/* Configure Channel DMA PCI-E Control Register with
	 * Write Type (Configuration or Memory Write)*/
	if (is_conf_write)
		__raw_writel(((1 << PCIE_DMA_PCIE_CTRL_PCIETD_OFFSET) |
			      (PCIE_DMA_PCIE_CTRL_TLP_CFG_TYPE0_RW &
			       0x1F)),
			     pcie_channels[channel].dma_pcie_ctrl_ch);
	else
		__raw_writel(((1 << PCIE_DMA_PCIE_CTRL_PCIETD_OFFSET) |
			      (PCIE_DMA_PCIE_CTRL_TLP_MEM_RW & 0x1F)),
			     pcie_channels[channel].dma_pcie_ctrl_ch);

	/* Configure Channel PIO Address Register's */
	__raw_writel(aligned_addr, pcie_channels[channel].pio_addr_ls_ch);

	if (!is_conf_write)
		writel(0x0, pcie_channels[channel].pio_addr_ms_ch);

	/* Configure Channel DMA Mode register to Write/Non-chain Mode */
	__raw_writel((PCIE_DMA_MODE_NONCHAINED |
		      PCIE_DMA_MODE_PCIE_WRITE),
		     pcie_channels[channel].dma_mode_ch);

	/* Configure Channel PIO Write Data Strobes */
	__raw_writel(data_strobe, pcie_channels[channel].pio_wr_strb_ch);

	/* Write data to Channel PIO Write Data Register */
	__raw_writel(aligned_val, pcie_channels[channel].pio_wr_data_ch);

	/* Start Channel Data Transfer */
	__raw_writel(PCIE_DMA_START_TRANSFER,
		     pcie_channels[channel].pio_start_ch);

	/* Wait for Start Bit to clear which indicates transfer
	 * completion
	 */
	for (i = 0; i < MAX_WAIT_LOOP; i++) {
		if (!__raw_readl(pcie_channels[channel].pio_start_ch))
			break;
		udelay(TIME_DELAY);
	}

	if (i >= MAX_WAIT_LOOP)
		PCIE_DEBUG("PIO register write timeout\n");

	spin_unlock_irqrestore(&channel_lock[channel], flags);
}

u8 pxa168_pcie_read8(u32 addr)
{
	return (u8) pxa168_pcie_read_reg(0, IO_MEM, addr, SIZE_8);
}

u16 pxa168_pcie_read16(u32 addr)
{
	return (u16) pxa168_pcie_read_reg(0, IO_MEM, addr, SIZE_16);
}

u32 pxa168_pcie_read32(u32 addr)
{
	return pxa168_pcie_read_reg(0, IO_MEM, addr, SIZE_32);
}

void pxa168_pcie_write8(u8 val, u32 addr)
{
	pxa168_pcie_write_reg(0, IO_MEM, addr, SIZE_8, (u32) val);
}

void pxa168_pcie_write16(u16 val, u32 addr)
{
	pxa168_pcie_write_reg(0, IO_MEM, addr, SIZE_16, (u32) val);
}

void pxa168_pcie_write32(u32 val, u32 addr)
{
	pxa168_pcie_write_reg(0, IO_MEM, addr, SIZE_32, val);
}

static void pxa168_pcie_clk_enable(void)
{

	/* Enable Clock to PCIe Controller */
	__raw_writel(0xFF, AXI_VIRT_BASE + 0x82900);
	mdelay(1);
}

static void pxa168_pcie_enable_link(void)
{
	unsigned int temp;

	pxa168_pcie_clk_enable();

	/*
	 * Enable ANALOG_CTRL register in PCI-E PHY. This
	 * should supposedly be the only register among the PCI-E
	 * PHY registers based on DE's. Set bit 7:6 to 0x01 (0.9 ns)
	 */
	temp = __raw_readl(PCIE_REG(PCIE_PHY_ANALOG_CTRL));
	temp &= ~0x000000f0;
	temp |= 0x50;
	__raw_writel(temp, PCIE_REG(PCIE_PHY_ANALOG_CTRL));	/* 0.9ns */

	/* Advertise as Gen 1 at Link Capability Register */
	__raw_writel((__raw_readl(PCIE_REG(PCIE_CFG_PCIE_CAP + 0xC)) &
		      0xFFFFFFF0) | 0x1, PCIE_REG(PCIE_CFG_PCIE_CAP + 0xC));

	/*
	 * Magic Register settings to init PCI-E Link
	 * (based on some test scripts)
	 */

	/* Program RC memory limit and base */
	__raw_writel(0x90008000, PCIE_STD_PCI_CFG_REG(0x20));
	/* Program RC prefechable memory limit and base */
	__raw_writel(0xB000A000, PCIE_STD_PCI_CFG_REG(0x24));

	/* Sets the default link number to zero since we have just one port */
	__raw_writel((__raw_readl(PCIE_REG(0x3708)) & 0xFFFFFF00),
		     PCIE_REG(0x3708));
	/* Enable fast training */
	__raw_writel((__raw_readl(PCIE_REG(0x3710)) | 0x00000080),
		     PCIE_REG(0x3710));
	/* Enable ECRC checking and generation */
	__raw_writel((__raw_readl(PCIE_REG(0x3118)) | 0x00000140),
		     PCIE_REG(0x3118));

	/* Disable L0s - Power Management */
	__raw_writel(0x00000000, PCIE_CFG_PCIE_CAP_REG(0x10));

	/* Set device into active mode - D0 state  */
	__raw_writel((__raw_readl(PCIE_CFG_PM_CAP_REG(0x04)) & 0xFFFFFFFC),
		     PCIE_CFG_PM_CAP_REG(0x04));

	/* Enable the memory space and bus master on Control Register */
	__raw_writel(0x6, PCIE_STD_PCI_CFG_REG(0x04));

	/* Bus number register */
	__raw_writel(0x00010100, PCIE_STD_PCI_CFG_REG(0x18));

	/* Enable LTSSM */
	__raw_writel(0x1, PCIE_REG(PCIE_CTRL0));
}

int pxa168_pcie_link_up(void)
{
	unsigned int count = 0;
	unsigned int ret = 1;

	while (!(__raw_readl(PCIE_REG(PCIE_ISR0)) & 0xc0000000)) {
		udelay(LINKUP_DELAY);
		count++;
		if (count == LINKUP_TIMEOUT) {
			ret = 0;	/* Failed to get link  */
			break;
		}
	}
	if (count == LINKUP_TIMEOUT)
		PCIE_DEBUG("Link up timeout at %dus\n",
			   (LINKUP_TIMEOUT * LINKUP_DELAY));

	if (count && (count < LINKUP_TIMEOUT))
		PCIE_DEBUG("Link up time = %dus\n", (count * LINKUP_DELAY));

	return ret;
}

static void pxa168_pcie_set_local_bus_nr(int nr)
{
	/* Write Configuration Space Type1 Header (Offset 0x18)
	 * through DBI */
	__raw_writeb((u8) (nr & 0xff), PCIE_REG(0x3018));

	return;
}

static int pxa168_pcie_rd_conf(void __iomem *base, struct pci_bus *bus,
			u32 devfn, int where, int size, u32 * val)
{
	u32 conf_addr;
	u32 reg_val;

	if (devfn > 15)
		return PCIBIOS_FUNC_NOT_SUPPORTED;

	conf_addr = PCIE_CONF_BUS(bus->number) |
	    PCIE_CONF_DEV(PCI_SLOT(devfn)) |
	    PCIE_CONF_FUNC(PCI_FUNC(devfn)) | PCIE_CONF_REG(where);

	reg_val = pxa168_pcie_read_reg(base, CONFIG_MEM, conf_addr, 4);

	switch (size) {
	case SIZE_8:
		reg_val = (reg_val >> (8 * (where & 3))) & 0xff;
		break;
	case SIZE_16:
		reg_val = (reg_val >> (8 * (where & 3))) & 0xffff;
		break;
	default:
		break;
	}

	*val = reg_val;

	return PCIBIOS_SUCCESSFUL;
}

static int pxa168_pcie_wr_conf(void __iomem *base, struct pci_bus *bus,
			u32 devfn, int where, int size, u32 val)
{
	int ret = PCIBIOS_SUCCESSFUL;
	u32 conf_addr;

	if (devfn > 15)
		return PCIBIOS_FUNC_NOT_SUPPORTED;

	conf_addr = PCIE_CONF_BUS(bus->number) |
	    PCIE_CONF_DEV(PCI_SLOT(devfn)) |
	    PCIE_CONF_FUNC(PCI_FUNC(devfn)) | PCIE_CONF_REG(where);

	pxa168_pcie_write_reg(base, CONFIG_MEM, conf_addr, size, val);

	return ret;
}

static int __init pxa168_pcie_setup(int nr, struct pci_sys_data *sys)
{
	if (nr >= NUM_PCIE_PORTS)
		return 0;

	pxa168_pcie_set_local_bus_nr(nr);

	return 1;
}

static int pcie_rd_conf(struct pci_bus *bus, u32 devfn, int where,
			int size, u32 * val)
{
	int ret;

	if (!pxa168_pcie_link_up()) {
		*val = 0xffffffff;
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	ret = pxa168_pcie_rd_conf(0, bus, devfn, where, size, val);

	return ret;
}

static int pcie_wr_conf(struct pci_bus *bus, u32 devfn,
			int where, int size, u32 val)
{
	int ret;

	if (!pxa168_pcie_link_up()) {
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	ret = pxa168_pcie_wr_conf(0, bus, devfn, where, size, val);

	return ret;
}

static struct pci_ops pcie_ops = {
	.read = pcie_rd_conf,
	.write = pcie_wr_conf,
};

static void __devinit rc_pci_fixup(struct pci_dev *dev)
{
	/*
	 * Prevent enumeration of root complex.
	 */
	if (dev->bus->parent == NULL && dev->devfn == 0) {
		int i;

		for (i = 0; i < DEVICE_COUNT_RESOURCE; i++) {
			dev->resource[i].start = 0;
			dev->resource[i].end = 0;
			dev->resource[i].flags = 0;
		}
	}
}

DECLARE_PCI_FIXUP_HEADER(PCI_VENDOR_ID_MARVELL, PCI_ANY_ID, rc_pci_fixup);

static struct pci_bus __init *pxa168_pcie_scan_bus(int nr, struct pci_sys_data
						   *sys)
{
	struct pci_bus *bus;

	if (nr < NUM_PCIE_PORTS) {
		bus = pci_scan_bus(sys->busnr, &pcie_ops, sys);
	} else {
		bus = NULL;
		BUG();
	}

	return bus;
}

static int pxa168_pcie_map_irq(struct pci_dev *dev, u8 slot, u8 pin)
{
	int irq = -1;

	if (slot == 1) {
		/* Unmask the Legacy Interrupts */
		__raw_writel(0x3, PCIE_REG(PCIE_PCI_LEGACY_ISRM0));
		/* Enable the Interrupt Pin Register in RC Config Space */
		__raw_writeb(0x1, PCIE_REG(0x303D));
		irq = IRQ_PXA168_PCIE_CORE;
	}

	return irq;
}

static struct hw_pci pxa168_pci __initdata = {
	.nr_controllers = 1,
	.swizzle = pci_std_swizzle,
	.setup = pxa168_pcie_setup,
	.scan = pxa168_pcie_scan_bus,
	.map_irq = pxa168_pcie_map_irq,
};

static void __init pcie_channel_data_init(void)
{
	unsigned int i;

	for (i = 0; i < NUM_PCIE_CHANNELS; i++)
		spin_lock_init(&(channel_lock[i]));
}

static void __init add_pcie_port(void)
{
	pxa168_pcie_enable_link();

	pcie_channel_data_init();
}


int pxa168_pcie_probe(struct platform_device *pdev)
{
	struct pxa168_pcie_platform_data *pdata =
		(struct pxa168_pcie_platform_data *)pdev->dev.platform_data;

	if (pdata->init == NULL)
		return 0;

	if (pdata->init() < 0) {
		printk(KERN_ERR "pcie: GPIO initialization failed.\n");
		return 1;
	}

	add_pcie_port();
	pci_common_init(&pxa168_pci);

	return 0;
}

#if defined(CONFIG_PM)
static int pxa168_pcie_suspend(struct platform_device *pdev, pm_message_t msg)
{
	return 0;
}

static int pxa168_pcie_resume(struct platform_device *pdev)
{
	/* TODO: Get this from platform data instead */
/*	if (pxa168_gpio_pcie_init() < 0) {
		printk(KERN_ERR "pcie: GPIO initialization failed.\n");
		return 1;
	}
*/
	pxa168_pcie_probe(pdev);
	pxa168_pcie_clk_enable();
	pxa168_pcie_enable_link();
	pxa168_pcie_map_irq(NULL, 1, 0);

	return 0;
}
#endif /* CONFIG_PM */

static struct platform_driver pxa168_pcie_driver = {
	.probe		= pxa168_pcie_probe,
#if defined(CONFIG_PM)
	.suspend	= pxa168_pcie_suspend,
	.resume		= pxa168_pcie_resume,
#endif /* CONFIG_PM */
	.driver		= {
		.name	= "pxa168-pcie",
		.owner	= THIS_MODULE,
	},
};

static int __init pxa168_pcie_init(void)
{
	int ret;

	/* TODO: Get this from platform data instead */
/*	Commented the below line as it was specific to aspenite and creats
	issue if the same kernel is used for AVLite. Using .probe instead. */
/*	if (pxa168_gpio_pcie_init() < 0) {
		printk(KERN_ERR "pcie: GPIO initialization failed.\n");
		return 1;
	}
*/
	ret = platform_driver_register(&pxa168_pcie_driver);
	if (ret) {
		printk(KERN_ERR "pcie: unable to register driver\n");
		return 1;
	}

/*	add_pcie_port();
	pci_common_init(&pxa168_pci);*/
	return 0;
}

device_initcall(pxa168_pcie_init);
