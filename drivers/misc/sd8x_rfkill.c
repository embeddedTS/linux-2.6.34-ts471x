/*
 * rfkill power contorl for Marvell sd8xxx wlan/bt
 *
 * Copyright (C) 2009 Marvell, Inc.
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
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/err.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/sd8x_rfkill.h>
#include <linux/slab.h>
#include <mach/gpio.h>
#include <mach/mmc.h>
#define SD8X_DEV_NAME "sd8x-rfkill"

struct sd8x_rfkill_data {
	enum rfkill_type type;
	bool blocked;
	struct sd8x_rfkill_platform_data *pdata;
};
#define SD8X_DATA_SIZE (RFKILL_TYPE_BLUETOOTH + 1)
static struct sd8x_rfkill_data *local_sd8x_data[SD8X_DATA_SIZE];

int add_sd8x_rfkill_device(int gpio_power_down, int gpio_reset,
	struct mmc_host ***pmmc)
{
        int ret;
	struct platform_device *pdev = NULL;
	struct sd8x_rfkill_platform_data *pdata = NULL;

	pdata = kzalloc(sizeof(struct sd8x_rfkill_platform_data), GFP_KERNEL);
	if (!pdata) {
		printk(KERN_CRIT "no memory\n");
		goto err_out;
	}
	pdata->gpio_power_down = gpio_power_down;
	pdata->gpio_reset = gpio_reset;

	pdev = kzalloc(sizeof(struct platform_device), GFP_KERNEL);
	if (!pdev) {
		printk(KERN_CRIT "no memory\n");
		goto err_out;
	}
	pdev->name = SD8X_DEV_NAME;
	pdev->id = -1,
	pdev->dev.platform_data = pdata;

        ret = platform_device_register(pdev);
	if (ret) {
		dev_err(&pdev->dev,
			"unable to register device: %d\n", ret);
		goto err_out;
	}
	*pmmc = &(pdata->mmc);
	return 0;

err_out:
	if (pdata)
		kfree(pdata);
	if (pdev)
		kfree(pdev);
	pr_debug("%s: error\n", __func__);
	return -1;
}
EXPORT_SYMBOL(add_sd8x_rfkill_device);

static int sd8x_power_on(struct sd8x_rfkill_platform_data *pdata, int on)
{
	int gpio_power_down = pdata->gpio_power_down;
	int gpio_reset = pdata->gpio_reset;

	pr_debug("%s: on=%d\n", __FUNCTION__, on);
	if (gpio_request(gpio_power_down, "sd8xxx power down")) {
		printk(KERN_INFO "gpio %d request failed\n", gpio_power_down);
		return -1;
	}

	if(gpio_reset && gpio_request(gpio_reset, "sd8xxx reset")) {
		printk(KERN_INFO "gpio %d request failed\n", gpio_reset);
		gpio_free(gpio_power_down);
		return -1;
	}

	if (on) {
		gpio_direction_output(gpio_power_down, 1);
		if(gpio_reset) {
			gpio_direction_output(gpio_reset, 0);
			msleep(50);
			gpio_direction_output(gpio_reset, 1);
		}
	} else {
		gpio_direction_output(gpio_power_down, 0);
		if(gpio_reset)
			gpio_direction_output(gpio_reset, 0);
	}

	if (!pdata->mmc)
		printk(KERN_DEBUG "rfkill is not linked with mmc_host\n");
	else {
		int wait = 100;
		mmc_detect_change(pdata->mmc, msecs_to_jiffies(10));

#ifdef CONFIG_RFKILL_WAIT
		while (--wait) {
			if ((on && pdata->mmc->card) ||
				(!on && !pdata->mmc->card))
				break;
			msleep(100);
		}
		if (!wait)
			printk(KERN_INFO "rfkill fails to wait right bus re-scan result in 10 seconds\n");
#endif
	}

	gpio_free(gpio_power_down);
	if(gpio_reset)
		gpio_free(gpio_reset);
	return 0;
}

static int sd8x_set_block(void *data, bool blocked)
{
	bool pre_blocked, another_blocked;
	int ret = 0;
	struct sd8x_rfkill_data *sd8x_data =
		(struct sd8x_rfkill_data *)data;

	if (!sd8x_data->pdata->wlan_rfkill && !sd8x_data->pdata->bt_rfkill) {
		ret = sd8x_power_on(sd8x_data->pdata, 0);
		return 0;
	}

	if (!sd8x_data->pdata->wlan_rfkill || !sd8x_data->pdata->bt_rfkill) {
		BUG_ON(!blocked);
		sd8x_data->blocked = blocked;
		return 0;
	}

	pre_blocked = sd8x_data->blocked;

	if (sd8x_data->type == RFKILL_TYPE_WLAN) {
		another_blocked =
			local_sd8x_data[RFKILL_TYPE_BLUETOOTH]->blocked;
	} else if (sd8x_data->type == RFKILL_TYPE_BLUETOOTH) {
		another_blocked = local_sd8x_data[RFKILL_TYPE_WLAN]->blocked;
	} else {
		return 0;
	}

	pr_debug("%s: try to set block state of type(%d) as %d, pre_blocked=%d,"
		" another_blocked=%d\n", __func__, sd8x_data->type, blocked,
		pre_blocked, another_blocked);
	if (!blocked && pre_blocked && another_blocked) {
		ret = sd8x_power_on(sd8x_data->pdata, 1);
	} else if (blocked && !pre_blocked && another_blocked) {
		ret = sd8x_power_on(sd8x_data->pdata, 0);
	}
	sd8x_data->blocked = blocked;

	return ret;
}

static struct rfkill * sd8x_rfkill_register(struct device *parent,
	enum rfkill_type type, char *name,
	struct sd8x_rfkill_platform_data *pdata)
{
	int err;
	struct rfkill *dev = NULL;
	struct rfkill_ops *ops = NULL;
	struct sd8x_rfkill_data *data = NULL;

	ops = kzalloc(sizeof(struct rfkill_ops), GFP_KERNEL);
	if (!ops)
		goto err_out;
	ops->set_block = sd8x_set_block;

	data = kzalloc(sizeof(struct sd8x_rfkill_data), GFP_KERNEL);
	if (!data)
		goto err_out;
	data->type = type;
	data->blocked = true;
	data->pdata = pdata;
	local_sd8x_data[type] = data;

	dev = rfkill_alloc(name, parent, type, ops, data);
	if (!dev)
		goto err_out;

	err = rfkill_register(dev);
	if (err)
		goto err_out;

	return dev;

err_out:
	if (ops)
		kfree(ops);
	if (data)
		kfree(data);
	if (dev)
		rfkill_destroy(dev);
	return 0;
}

static void sd8x_rfkill_free(struct sd8x_rfkill_platform_data *pdata)
{
	int i;

	if (pdata->wlan_rfkill) {
		rfkill_unregister(pdata->wlan_rfkill);
		rfkill_destroy(pdata->wlan_rfkill);
	}
	if (pdata->bt_rfkill) {
		rfkill_unregister(pdata->bt_rfkill);
		rfkill_destroy(pdata->bt_rfkill);
	}

	for (i = 0; i < SD8X_DATA_SIZE && local_sd8x_data[i]; i++) {
		kfree(local_sd8x_data[i]);
	}
}

static int sd8x_rfkill_probe(struct platform_device *pdev)
{
	struct rfkill *rfkill = NULL;
	struct sd8x_rfkill_platform_data *pdata =
		pdev->dev.platform_data;

	rfkill = sd8x_rfkill_register(&pdev->dev,
		RFKILL_TYPE_WLAN, "sd8xxx-wlan", pdata);
	if (IS_ERR(rfkill))
		goto err_out;
	pdata->wlan_rfkill = rfkill;

	rfkill = sd8x_rfkill_register(&pdev->dev,
		RFKILL_TYPE_BLUETOOTH, "sd8xxx-bluetooth", pdata);
	if (IS_ERR(rfkill))
		goto err_out;
	pdata->bt_rfkill = rfkill;

	return 0;

err_out:
	sd8x_rfkill_free(pdata);
	return -1;
}

static int sd8x_rfkill_remove(struct platform_device *pdev)
{
	struct sd8x_rfkill_platform_data *pdata =
		pdev->dev.platform_data;

	sd8x_rfkill_free(pdata);

	return 0;
}

static int sd8x_rfkill_suspend(struct platform_device *pdev, pm_message_t pm_state)
{
	struct sd8x_rfkill_platform_data *pdata =
		pdev->dev.platform_data;

	/* For WIFI module is totally powered off, we need marked state
	 * as blocked so as to trigger re-download firmware
	 * when rfkill framework restore original active status */
	if (PM_EVENT_SUSPEND == pm_state.event) {
		if (local_sd8x_data[RFKILL_TYPE_WLAN]->blocked == false)
			local_sd8x_data[RFKILL_TYPE_WLAN]->blocked = true;
		if (local_sd8x_data[RFKILL_TYPE_BLUETOOTH]->blocked == false)
			local_sd8x_data[RFKILL_TYPE_BLUETOOTH]->blocked = true;
	}

	return 0;
}

static int sd8x_rfkill_resume(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver sd8x_rfkill_platform_driver = {
	.probe = sd8x_rfkill_probe,
	.remove = sd8x_rfkill_remove,
	.driver = {
		.name = SD8X_DEV_NAME,
		.owner = THIS_MODULE,
	},
	.suspend = sd8x_rfkill_suspend,
	.resume = sd8x_rfkill_resume,
};

static int __init sd8x_rfkill_init(void)
{
	return platform_driver_register(&sd8x_rfkill_platform_driver);
}

static void __exit sd8x_rfkill_exit(void)
{
	platform_driver_unregister(&sd8x_rfkill_platform_driver);
}

module_init(sd8x_rfkill_init);
module_exit(sd8x_rfkill_exit);

MODULE_ALIAS("platform:sd8x_rfkill");
MODULE_DESCRIPTION("sd8x_rfkill");
MODULE_AUTHOR("Marvell");
MODULE_LICENSE("GPL");
