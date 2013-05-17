#ifndef ASMARM_ARCH_MMC_H
#define ASMARM_ARCH_MMC_H

#include <linux/mmc/host.h>
#include <linux/interrupt.h>

struct device;
struct mmc_host;


#define SD_RESP_6	0x1c		/* Command Response 6 */
#define SD_RESP_7	0x1e		/* Command Response 7 */

#define SD_FIFO_PARAM	0xE0
#define DIS_PAD_SD_CLK_GATE_BIT	(1 << 10) /* Turn on/off Dynamic SD Clock Gating */

#define SD_CLOCK_AND_BURST_SIZE_SETUP	0xE6
#define SDCLK_DELAY_MASK	0xF
#define SDCLK_SEL_MASK		0x3
#define SDCLK_DELAY_SHIFT	10
#define SDCLK_SEL_SHIFT		8

#define DRIVER_NAME	"pxa-sdh"
#define MAX_MMC_SLOTS	8

/* SD spec says 74 clocks few more is okay */
#define INIT_CLOCKS	80

struct sdhci_mmc_slot {
	struct sdhci_mmc_chip	*chip;
	struct sdhci_host	*host;
	struct clk	*clk;
	u32	clkrate;
	u32	f_max;
	u8	width;
	u8	eightBitEnabled;
	u8	clockEnabled;
};

struct sdhci_mmc_chip {
	struct platform_device	*pdev;
	struct resource	*res;
	struct sdhci_mmc_fixes	*fixes;
	unsigned int	quirks;
	int	num_slots;	/* Slots on controller */
	struct sdhci_mmc_slot	*slots[MAX_MMC_SLOTS];
	struct pxasdh_platform_data *pdata;
	unsigned int	mrvl_quirks;
};

struct sdhci_mmc_fixes {
	unsigned int	quirks;
	int	(*probe)(struct sdhci_mmc_chip*);
	int	(*probe_slot)(struct sdhci_mmc_slot*);
	void	(*remove_slot)(struct sdhci_mmc_slot*, int);
	int	(*suspend)(struct sdhci_mmc_chip*, pm_message_t);
	int	(*resume)(struct sdhci_mmc_chip*);
};

struct pxasdh_platform_data {
	unsigned int ocr_mask;			/* available voltages */
	unsigned long detect_delay;		/* delay in jiffies before detecting cards after interrupt */
	int (*init)(struct device *, irq_handler_t , void *);
	int (*get_ro)(struct device *);
	void (*setpower)(struct device *, unsigned int);
	void (*exit)(struct device *, void *);
	int (*mfp_config)(void);
	int (*mfp_unconfig)(void);
	struct pfn_cfg *pfn_table;
	int (*get_cd)(struct device *);
	unsigned int bus_width;
	unsigned int max_speed;

	/* SD_CLOCK_AND_BURST_SIZE_SETUP register */
	unsigned int sd_clock;			/* 1 for need to tuning sd clock */
	unsigned int sdclk_sel;
	unsigned int sdclk_delay;

	unsigned int quirks;
	unsigned int mrvl_quirks;

#ifdef CONFIG_SD8XXX_RFKILL
	/*for sd8688-rfkill device*/
	struct mmc_host **pmmc;
#endif
};

struct platform_mmc_slot {
	int gpio_detect; /* 0 for controller detect, 1 for gpio detect */
	int no_wp; /* 0 for with write protect, 1 for without write protect */
	int gpio_cd;
	int gpio_wp;
};

extern struct platform_mmc_slot pxa_mmc_slot[];
extern int pxa_mci_ro(struct device *dev);
extern int pxa_mci_init(struct device *dev, irq_handler_t pxa_detect_int,void *data);
extern void pxa_mci_exit(struct device *dev, void *data);
extern int pxa_mci_get_cd(struct device *dev);

/* Disable Free Running Clocks for SDIO */
#define MRVL_QUIRK_SDIO_ENABLE_DYN_CLOCK_GATING			(1<<0)

/* pin enum */
enum {
	PIN_MMC_CMD,
	PIN_MMC_CLK,
	PIN_MMC_WP,
	PIN_MMC_CD,
	PIN_MMC_DAT0,
	PIN_MMC_DAT1,
	PIN_MMC_DAT2,
	PIN_MMC_DAT3,
	PIN_MMC_DAT4,
	PIN_MMC_DAT5,
	PIN_MMC_DAT6,
	PIN_MMC_DAT7,
	PIN_MMC_END
};
#endif
