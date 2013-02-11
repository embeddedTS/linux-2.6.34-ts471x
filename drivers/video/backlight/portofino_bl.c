/*
 * Backlight driver for Marvell Semiconductor Portofio(PM8606)
 *
 * Copyright (C) 2009 Marvell International Ltd.
 * 	<njun@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <mach/portofino.h>


#define REG_WLED_DUTY(id) (0x2 + ((id) << 1))

#define REG_WLED_CTRL(id) (0x3 + ((id) << 1))
#define PORTOFINO_WLED_FULL_DUTY	1
#define PORTOFINO_WLED_CURRENT_MASK	0x3e
#define PORTOFINO_WLED_CURRENT(x)	(((x) << 1) & PORTOFINO_WLED_CURRENT_MASK)
#define PORTOFINO_WLED_CURRENT_GET(x)	(((x) >> 1) & 0x1f)

#define PORTOFINO_MAX_BRIGHTNESS	0x1f

struct portofino_backlight_data {
	struct device *portofino_dev;
	int id;
};

static int portofino_backlight_set(struct backlight_device *bl, int brightness)
{
	struct portofino_backlight_data *data = bl_get_data(bl);
	int ret = 0;
	u8 val;

	ret = portofino_read(REG_WLED_CTRL(data->id), &val);
	val &= ~PORTOFINO_WLED_CURRENT_MASK;
	val |= (u8) PORTOFINO_WLED_CURRENT(brightness);
	ret = portofino_write(REG_WLED_CTRL(data->id), val);
	if (ret){
		bl->props.brightness = 0;
		printk("Failed to update backlight!\n");
		return ret;
	}

	return 0;
}

static int portofino_backlight_update_status(struct backlight_device *bl)
{
	int brightness = bl->props.brightness;

	if (bl->props.power != FB_BLANK_UNBLANK)
		brightness = 0;

	if (bl->props.fb_blank != FB_BLANK_UNBLANK)
		brightness = 0;

	return portofino_backlight_set(bl, brightness);
}

static int portofino_backlight_get_brightness(struct backlight_device *bl)
{
	struct portofino_backlight_data *data = bl_get_data(bl);
	int ret;
	u8 val;
	ret = portofino_read(REG_WLED_CTRL(data->id), &val);
	if(ret){
		printk("Failed to get backlight power! set as 0\n");
		return 0;
	}
	return (int)PORTOFINO_WLED_CURRENT_GET(val);
}

static struct backlight_ops portofino_backlight_ops = {
	.update_status	= portofino_backlight_update_status,
	.get_brightness	= portofino_backlight_get_brightness,
};

static int __init portofino_backlight_probe(struct platform_device *pdev)
{
	struct portofino_backlight_data *data;
	struct backlight_device *bl;
	int max_brightness, ret = -EINVAL;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (data == NULL)
		return -ENOMEM;

	max_brightness = PORTOFINO_MAX_BRIGHTNESS;

	if(pdev->id < 0 || pdev->id > 2)
		goto probe;
	data->id = pdev->id;
	data->portofino_dev = pdev->dev.parent;

	bl = backlight_device_register(pdev->name, data->portofino_dev,
			data, &portofino_backlight_ops);
	if (IS_ERR(bl)) {
		ret = PTR_ERR(bl);
		goto probe;
	}

	bl->props.max_brightness = max_brightness;
	bl->props.brightness = max_brightness >> 1;

	platform_set_drvdata(pdev, bl);
	backlight_update_status(bl);
	dev_notice(&pdev->dev, "backlight detected at WLED id %d\n", data->id);
	return 0;

probe:
	dev_err(&pdev->dev, "failed to register backlight\n");
	kfree(data);
	return ret;
}

static int portofino_backlight_remove(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct portofino_backlight_data *data = bl_get_data(bl);

	backlight_device_unregister(bl);
	kfree(data);
	return 0;
}

#ifdef CONFIG_PM
static int portofino_backlight_suspend(struct platform_device *pdev,
				 pm_message_t state)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);
	return portofino_backlight_set(bl, 0);
}

static int portofino_backlight_resume(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);

	backlight_update_status(bl);
	return 0;
}
#else
#define portofino_backlight_suspend	NULL
#define portofino_backlight_resume		NULL
#endif

static struct platform_driver portofino_backlight_driver = {
	.driver		= {
		.name	= "portofino-bl",
		.owner	= THIS_MODULE,
	},
	.probe		= portofino_backlight_probe,
	.remove		= portofino_backlight_remove,
	.suspend	= portofino_backlight_suspend,
	.resume		= portofino_backlight_resume,
};

static int __init portofino_backlight_init(void)
{
	return platform_driver_register(&portofino_backlight_driver);
}
module_init(portofino_backlight_init);

static void __exit portofino_backlight_exit(void)
{
	platform_driver_unregister(&portofino_backlight_driver);
}
module_exit(portofino_backlight_exit);

MODULE_DESCRIPTION("Backlight Driver for Marvell Semiconductor Portofino PM8606");
MODULE_AUTHOR("Jun Nie <njun@marvell.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:portofino-backlight");
