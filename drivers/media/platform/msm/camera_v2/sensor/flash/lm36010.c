/* Copyright (c) 2017-2019, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/module.h>
#include <linux/export.h>
#include "msm_led_flash.h"

#define FLASH_NAME "qcom,led-flash1"

#undef CDBG
#define CDBG(fmt, args...) pr_debug(fmt, ##args)

static struct msm_led_flash_ctrl_t fctrl;
static struct i2c_driver lm36010_i2c_driver;

static struct msm_camera_i2c_reg_array lm36010_init_array[] = {
	{0x01, 0x20},
	{0x02, 0x15},
	{0x03, 0x00},
	{0x04, 0x00},
};

static struct msm_camera_i2c_reg_array lm36010_off_array[] = {
	{0x01, 0x20},
};

static struct msm_camera_i2c_reg_array lm36010_release_array[] = {
	{0x01, 0x20},
};

static struct msm_camera_i2c_reg_array lm36010_low_array[] = {
	{0x01, 0x22},
/*	{0x02, 0x15},
	{0x03, 0x00},
	{0x04, 0x00},
*/
};

static struct msm_camera_i2c_reg_array lm36010_high_array[] = {
	{0x01, 0x23},
/*	{0x02, 0x15},
	{0x03, 0x15},
	{0x04, 0x15},
*/
};

static void __exit msm_flash_lm36010_i2c_remove(void)
{
	i2c_del_driver(&lm36010_i2c_driver);
}

static const struct of_device_id lm36010_trigger_dt_match[] = {
	{.compatible = "qcom,led-flash1", .data = &fctrl},
	{}
};

MODULE_DEVICE_TABLE(of, lm36010_trigger_dt_match);

static const struct i2c_device_id flash_i2c_id[] = {
	{"qcom,led-flash1", (kernel_ulong_t)&fctrl},
	{ }
};

static const struct i2c_device_id lm36010_i2c_id[] = {
	{FLASH_NAME, (kernel_ulong_t)&fctrl},
	{ }
};

static int msm_flash_lm36010_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	if (!id) {
		pr_err("msm_flash_lm36010_i2c_probe: id is NULL");
		id = lm36010_i2c_id;
	}

	return msm_flash_i2c_probe(client, id);
}

static struct i2c_driver lm36010_i2c_driver = {
	.id_table = lm36010_i2c_id,
	.probe  = msm_flash_lm36010_i2c_probe,
	.remove = __exit_p(msm_flash_lm36010_i2c_remove),
	.driver = {
		.name = FLASH_NAME,
		.owner = THIS_MODULE,
		.of_match_table = lm36010_trigger_dt_match,
	},
};

static int msm_flash_lm36010_platform_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;

	match = of_match_device(lm36010_trigger_dt_match, &pdev->dev);

	if (!match)
		return -EFAULT;
	return msm_flash_probe(pdev, match->data);
}

static struct platform_driver lm36010_platform_driver = {
	.probe = msm_flash_lm36010_platform_probe,
	.driver = {
		.name = "qcom,led-flash1",
		.owner = THIS_MODULE,
		.of_match_table = lm36010_trigger_dt_match,
	},
};

static int __init msm_flash_lm36010_init_module(void)
{
	int32_t rc = 0;

	rc = platform_driver_register(&lm36010_platform_driver);

	if (fctrl.pdev != NULL && rc == 0)
		pr_err("lm36010 platform_driver_register success");
	else if (rc != 0)
		pr_err("lm36010 platform_driver_register failed");
	else {
		rc = i2c_add_driver(&lm36010_i2c_driver);
		if (!rc)
			pr_err("lm36010 i2c_add_driver success");
	}
	return rc;
}

static void __exit msm_flash_lm36010_exit_module(void)
{
	if (fctrl.pdev)
		platform_driver_unregister(&lm36010_platform_driver);
	else
		i2c_del_driver(&lm36010_i2c_driver);
}

static struct msm_camera_i2c_client lm36010_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};

static struct msm_camera_i2c_reg_setting lm36010_init_setting = {
	.reg_setting = lm36010_init_array,
	.size = ARRAY_SIZE(lm36010_init_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm36010_off_setting = {
	.reg_setting = lm36010_off_array,
	.size = ARRAY_SIZE(lm36010_off_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm36010_release_setting = {
	.reg_setting = lm36010_release_array,
	.size = ARRAY_SIZE(lm36010_release_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm36010_low_setting = {
	.reg_setting = lm36010_low_array,
	.size = ARRAY_SIZE(lm36010_low_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm36010_high_setting = {
	.reg_setting = lm36010_high_array,
	.size = ARRAY_SIZE(lm36010_high_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_led_flash_reg_t lm36010_regs = {
	.init_setting = &lm36010_init_setting,
	.off_setting = &lm36010_off_setting,
	.low_setting = &lm36010_low_setting,
	.high_setting = &lm36010_high_setting,
	.release_setting = &lm36010_release_setting,
};

static struct msm_flash_fn_t lm36010_func_tbl = {
	.flash_get_subdev_id = msm_led_i2c_trigger_get_subdev_id,
	.flash_led_config = msm_led_i2c_trigger_config,
	.flash_led_init = msm_flash_led_init,
	.flash_led_release = msm_flash_led_release,
	.flash_led_off = msm_flash_led_off,
	.flash_led_low = msm_flash_led_low,
	.flash_led_high = msm_flash_led_high,
};

static struct msm_led_flash_ctrl_t fctrl = {
	.flash_i2c_client = &lm36010_i2c_client,
	.reg_setting = &lm36010_regs,
	.func_tbl = &lm36010_func_tbl,
};

/*subsys_initcall(msm_flash_i2c_add_driver);*/
module_init(msm_flash_lm36010_init_module);
module_exit(msm_flash_lm36010_exit_module);
MODULE_DESCRIPTION("lm36010 FLASH");
MODULE_LICENSE("GPL v2");
