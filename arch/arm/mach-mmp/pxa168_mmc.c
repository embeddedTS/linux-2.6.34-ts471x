#if defined(CONFIG_MMC_PXA_SDH) || defined(CONFIG_MMC_PXA_SDH_MODULE)
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sdhci.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>

#include <mach/mmc.h>
#include <mach/cputype.h>
#include <plat/pfn_cfg.h>
#include <linux/gpio.h>


void pxa_sdh_startclk(struct mmc_host *mmc)
{
	unsigned int status_en_save;
	unsigned int interrupt_en_save;
	struct sdhci_host *host = mmc_priv(mmc);
	struct sdhci_mmc_slot *slot = sdhci_priv(host);
	struct sdhci_mmc_chip *chip = slot->chip;
	struct pfn_cfg *cfg = chip->pdata->pfn_table;

	if (cfg == NULL)
		return;

	/* configure CMD pin as GPIO */
	mfp_config(pfn_lookup(cfg, PFN_GPIO, PIN_MMC_CMD), 1);

	disable_irq(host->irq);

	/* Send CMD0 and wait */
	status_en_save = sdhci_readl(host, SDHCI_INT_ENABLE);
	interrupt_en_save = sdhci_readl(host, SDHCI_SIGNAL_ENABLE);
	sdhci_writel(host, 0, SDHCI_SIGNAL_ENABLE);
	sdhci_writel(host, SDHCI_INT_RESPONSE, SDHCI_INT_ENABLE);
	sdhci_writel(host, 0, SDHCI_ARGUMENT);
	sdhci_writew(host, 0, SDHCI_TRANSFER_MODE);
	sdhci_writew(host, 0, SDHCI_COMMAND);

	while (!(sdhci_readl(host, SDHCI_INT_STATUS) & SDHCI_INT_RESPONSE));

	sdhci_writel(host, sdhci_readl(host, SDHCI_INT_STATUS),
		SDHCI_INT_STATUS);

	/* restore CMD pin */
	mfp_config(pfn_lookup(cfg, PFN_FN, PIN_MMC_CMD), 1);

	sdhci_writel(host, status_en_save, SDHCI_INT_ENABLE);
	sdhci_writel(host, interrupt_en_save, SDHCI_SIGNAL_ENABLE);

	enable_irq(host->irq);
}
EXPORT_SYMBOL_GPL(pxa_sdh_startclk);
#endif
