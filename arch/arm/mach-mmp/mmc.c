#include <linux/platform_device.h>
#include <mach/mmc.h>
#include <mach/gpio.h>

#define MAX_SLOTS       4
struct platform_mmc_slot pxa_mmc_slot[MAX_SLOTS];

int pxa_mci_ro(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);

	return gpio_get_value(pxa_mmc_slot[pdev->id].gpio_wp);
}

int pxa_mci_init(struct device *dev,
			     irq_handler_t pxa_detect_int,
			     void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	int err, cd_irq, gpio_cd, gpio_wp;

	if (!pxa_mmc_slot[pdev->id].gpio_detect)
		return 0;

	cd_irq = gpio_to_irq(pxa_mmc_slot[pdev->id].gpio_cd);
	gpio_cd = pxa_mmc_slot[pdev->id].gpio_cd;
	gpio_wp = -1;

	/*
	 * setup GPIO for MMC controller
	 */
	err = gpio_request(gpio_cd, "mmc card detect");
	if (err)
		goto err_request_cd;
	gpio_direction_input(gpio_cd);

	if (!pxa_mmc_slot[pdev->id].no_wp) {
		gpio_wp = pxa_mmc_slot[pdev->id].gpio_wp;
		err = gpio_request(gpio_wp, "mmc write protect");
		if (err)
			goto err_request_wp;
		gpio_direction_input(gpio_wp);
	}

	err = request_irq(cd_irq, pxa_detect_int,
			  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			  "MMC card detect", data);
	if (err) {
		printk(KERN_ERR "%s: MMC/SD/SDIO: "
				"can't request card detect IRQ\n", __func__);
		goto err_request_irq;
	}

	return 0;

err_request_irq:
	if (!pxa_mmc_slot[pdev->id].no_wp && gpio_wp != -1)
		gpio_free(gpio_wp);
err_request_wp:
	gpio_free(gpio_cd);
err_request_cd:
	return err;
}

void pxa_mci_exit(struct device *dev, void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	int cd_irq, gpio_cd, gpio_wp;

	if (!pxa_mmc_slot[pdev->id].gpio_detect)
		return;

	cd_irq = gpio_to_irq(pxa_mmc_slot[pdev->id].gpio_cd);
	gpio_cd = pxa_mmc_slot[pdev->id].gpio_cd;
	gpio_wp = pxa_mmc_slot[pdev->id].gpio_wp;

	free_irq(cd_irq, data);
	gpio_free(gpio_cd);
	if (!pxa_mmc_slot[pdev->id].no_wp) {
		gpio_free(gpio_wp);
	}
}

int pxa_mci_get_cd(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);

	return gpio_get_value(pxa_mmc_slot[pdev->id].gpio_cd);
}
