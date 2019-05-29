/*
 * xp3 GPIO's driver for various purposes
 * Copyright (C) 2016-2019 Borqs software solutions.
 * Author: Anilkumar <anilkumar.ml@borqs.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/module.h>
#include <linux/pinctrl/consumer.h>
#include <linux/sysfs.h>
#include <linux/device.h>
#include <linux/kobject.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/hw_ver.h>

struct kobject *xp3_gpio_kobj;

struct xp3_gpio_device {
	struct xp3_gpio_platform_data *pdata;
};

static struct pinctrl *p;
static struct pinctrl_state *def;
static struct pinctrl_state *sus;
struct xp3_gpio_device xp3_gpio_dev;

struct xp3_gpio_platform_data {
	unsigned int hwid1_gpio;
	unsigned int hwid2_gpio;
};

static struct xp3_gpio_platform_data *
	xp3_parse_dt(struct device *dev)
{
	struct device_node *node;
	struct xp3_gpio_platform_data *pdata;

	node = dev->of_node;

	if (!node)
		goto err_out;

	pdata = kzalloc(sizeof(*pdata) , GFP_KERNEL);
	if (!pdata)
		goto err_out;

	pdata->hwid1_gpio = of_get_named_gpio(node, "hw-id1_gpio", 0);
	if (pdata->hwid1_gpio < 0)
		dev_err(dev, "Error in reading hw-id1_gpio property in dtsi");

	pdata->hwid2_gpio = of_get_named_gpio(node, "hw-id2_gpio", 0);
	if (pdata->hwid2_gpio < 0)
		dev_err(dev, "Error in reading hw-id2_gpio property in dtsi");

	pr_info("Parsing dtsi successful\n");
	return pdata;
err_out:
	return ERR_PTR(-1);
}

unsigned int hw_version(void)
{
	unsigned int hw_version = 0;
	int error = 0;

	error = pinctrl_select_state(p, def);
	if (error < 0)
		pr_crit("pinctrl select active state failed\n");

	msleep(20);
	hw_version = (gpio_get_value(xp3_gpio_dev.pdata->hwid1_gpio) << 1) |
		(gpio_get_value(xp3_gpio_dev.pdata->hwid2_gpio));

	error = pinctrl_select_state(p, sus);
	if (error < 0)
		pr_crit("pinctrl select suspend state failed\n");

	return  hw_version;
}
EXPORT_SYMBOL(hw_version);

static ssize_t get_hw_version(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	unsigned int hw_version = 0;
	int error = 0;

	error = pinctrl_select_state(p, def);
	if (error < 0)
		dev_err(dev, "pinctrl select active state failed\n");

	msleep(20);
	pr_info("hwid1_gpio :%d  value: %d hwid2_gpio :%d value %d\n",
			xp3_gpio_dev.pdata->hwid1_gpio,
			gpio_get_value(xp3_gpio_dev.pdata->hwid1_gpio),
			xp3_gpio_dev.pdata->hwid2_gpio,
			gpio_get_value(xp3_gpio_dev.pdata->hwid2_gpio));
	hw_version = (gpio_get_value(xp3_gpio_dev.pdata->hwid1_gpio) << 1) |
		(gpio_get_value(xp3_gpio_dev.pdata->hwid2_gpio));

	error = pinctrl_select_state(p, sus);
	if (error < 0)
		dev_err(dev, "pinctrl select suspend state failed\n");

	return scnprintf(buf, PAGE_SIZE, "%d\n", hw_version);
}

static DEVICE_ATTR(hw_version, 0664, (void *)get_hw_version, NULL);

static struct attribute *dev_attrs_hw_version[] = {
	&dev_attr_hw_version.attr,
	NULL,
};

static struct attribute_group dev_attr_group_hw_version = {
	.attrs = dev_attrs_hw_version,
};

static int xp3_gpio_probe(struct platform_device *pdev)
{
	int error = 0;
	struct device *dev = &pdev->dev;
	struct xp3_gpio_platform_data *pdata = dev_get_platdata(dev);

	if (!pdata) {
		pdata = xp3_parse_dt(dev);
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);
	}
	xp3_gpio_dev.pdata = pdata;

	p = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(p))
		dev_err(&pdev->dev, "pin control handle failed\n");

	if (gpio_is_valid(xp3_gpio_dev.pdata->hwid1_gpio)) {
		error = gpio_request(xp3_gpio_dev.pdata->hwid1_gpio,
					"xp3-gpio1");
		if (error)
			dev_err(&pdev->dev, "GPIO_REQUEST failed\n");

		error = gpio_direction_input(xp3_gpio_dev.pdata->hwid1_gpio);
		if (error)
			dev_err(&pdev->dev, "Setting GPIO direction input failed\n");
	}
	if (gpio_is_valid(xp3_gpio_dev.pdata->hwid2_gpio)) {
		error = gpio_request(xp3_gpio_dev.pdata->hwid2_gpio,
					"xp3-gpio2");
		if (error)
			dev_err(&pdev->dev, "GPIO_REQUEST failed\n");

		error = gpio_direction_input(xp3_gpio_dev.pdata->hwid2_gpio);
		if (error)
			dev_err(&pdev->dev, "Setting GPIO direction input failed\n");
	}
	def = pinctrl_lookup_state(p, "active");
	if (IS_ERR_OR_NULL(def))
		dev_err(&pdev->dev, "pinctrl lookup active state failed\n");

	sus = pinctrl_lookup_state(p, "suspend");
	if (IS_ERR_OR_NULL(sus))
		dev_err(&pdev->dev, "pinctrl lookup suspend state failed\n");

	xp3_gpio_kobj = kobject_create_and_add("xp3_gpios", kernel_kobj);
	if (!xp3_gpio_kobj) {
		dev_err(&pdev->dev, "xp3 gpio kobject create problem\n");
		return -ENOMEM;
	}

	error = sysfs_create_group(xp3_gpio_kobj, &dev_attr_group_hw_version);
	if (error < 0) {
		dev_err(&pdev->dev, "device create problem\n");
		goto destroy_kobj;
	}

	pr_info("Probe successful\n");
	return 0;

destroy_kobj:
	kobject_put(xp3_gpio_kobj);

	return error;
}

static int xp3_gpio_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	sysfs_remove_group(&dev->kobj, &dev_attr_group_hw_version);
	kobject_put(xp3_gpio_kobj);
	return 0;
}

static const struct of_device_id of_gpio_leds_match[] = {
	{ .compatible = "xp3-gpios", },
	{},
};

static struct platform_driver xp3_gpio_driver = {
	.probe          = xp3_gpio_probe,
	.remove         = xp3_gpio_remove,
	.driver         = {
		.name   = "xp3-gpio",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(of_gpio_leds_match),
	},
};

static int __init xp3_gpio_init(void)
{
	return platform_driver_register(&xp3_gpio_driver);
}

static void __exit xp3_gpio_exit(void)
{
	platform_driver_unregister(&xp3_gpio_driver);
}

postcore_initcall(xp3_gpio_init);
module_exit(xp3_gpio_exit);

module_platform_driver(xp3_gpio_driver);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Anilkumar <anilkumar.ml@borqs.com>");
