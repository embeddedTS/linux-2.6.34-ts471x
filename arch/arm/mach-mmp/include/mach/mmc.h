#ifndef ASMARM_ARCH_MMC_H
#define ASMARM_ARCH_MMC_H

#include <linux/mmc/host.h>
#include <linux/interrupt.h>

struct device;
struct mmc_host;

struct pxasdh_platform_data {
	unsigned int ocr_mask;			/* available voltages */
	unsigned long detect_delay;		/* delay in jiffies before detecting cards after interrupt */
	int (*init)(struct device *, irq_handler_t , void *);
	int (*get_ro)(struct device *);
	void (*setpower)(struct device *, unsigned int);
	void (*exit)(struct device *, void *);
	int (*mfp_config)(void);
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
