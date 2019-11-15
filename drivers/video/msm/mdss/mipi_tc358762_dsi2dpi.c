/*
 * Toshiba DSI to DPI Interface
 *
 * Copyright (c) 2007-2014, The Linux Foundation. All rights reserved.
 * Copyright (C) 2007 Google Incorporated
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include "mipi_tc358762_dsi2dpi.h"
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <asm/uaccess.h>
#include <asm/mach-types.h>
#include <asm/setup.h>

#define TC_L2_MIN_UV			1200000
#define TC_L2_MAX_UV			1200000
#define TC_L6_MIN_UV			1800000
#define TC_L6_MAX_UV			1800000
#define TC_L17_MIN_UV			2850000
#define TC_L17_MAX_UV			2850000

static struct tc358762_platform_data *pdata;
static struct tc358762_data *data;
/*clock enable function*/
static int tc358762_clock_select(void);
/*clock disable function*/
static int tc358762_clock_deselect(void);

static struct {
	u16 reg;
	u32 data;
} tc358762_init_seq[] = {
#if 0
/* For 16 bit panel interface */
{0x47c,   0x00000000},
{0x210 ,  0x00000003},
{0x164 , 0x00000004},
{0x168 ,  0x00000004},
{0x144 ,  0x00000000 },
{0x148 ,  0x00000000 },
{0x0114,  0x00000002 },
{0x0450,  0x00000001 },
{0x0454,  0x00000122 },
{0x0484,  0x00000000 },
{0x0480,  0x0000001F },
{0x0400,  0x00000000},
{0x0410, 0x00000007 },
{0x0418, 0x0000003C},
{0x0440, 0x00000100},
{0x0464,  0x00000205},
{0x0104,  0x00000001},
{0x0204,  0x00000001},
{0x0468,   0x00000004},
{0x0470,   0x50300000},
{0x047C,   0x00000080},
{0x047C,   0x00000000},
{0x0414,   0x00000005},
#else
/* For 18 bit panel interface */
{0x47c,   0x00000000},
{0x210 ,  0x00000003 },
{0x164 , 0x00000005},
{0x168 ,  0x00000005},
{0x144 ,  0x00000000 },
{0x148 ,  0x00000000 },
{0x0114,  0x00000003 },
{0x0450,  0x00000001 },
{0x0454,  0x00000122 },
{0x0484,  0x00000000 },
{0x0480,  0x0000001F },
{0x0400,  0x00000000},
{0x0410, 0x00000007 },
{0x0418, 0x0000003C},
{0x0440, 0x00000100},
{0x0464,  0x00000405},
{0x0104,  0x00000001},
{0x0204,  0x00000001},
{0x0468,   0x00000004},
{0x0470,   0x38260000},
{0x047C,   0x00000080},
{0x047C,   0x00000000},
{0x0414,   0x00000006},

#endif
};
#if 0
static struct {
	u16 reg;
	u32 data;
} tc358762_denit_seq[] = {
	{0x47c,   0x00000080},
};
#endif
/*
	Routine to enable clock.
	this routine can be extended to select from multiple
	sources based on clk_src_name.
*/
static int tc358762_clock_select(void)
{
	int r = 0;

	data->bb_clk2 = clk_get(&data->client->dev, "bb_clk2");

	if (data->bb_clk2 == NULL) {
		printk(KERN_ERR "clk_get bb_clk2 failed");
		goto err_clk;
	}

	if (data->clk_run == false) {
		printk(KERN_ERR "clk_prepare_enable bb_clk2 calling");
		r = clk_prepare_enable(data->bb_clk2);
	}

	if (r) {
		printk(KERN_ERR "clk_prepare_enable bb_clk2\
				failed ret = %d", r);
		goto err_clk;
	}

	data->clk_run = true;

	return r;

err_clk:
	r = -1;
	return r;
}
/*
	Routine to disable clocks
*/
static int tc358762_clock_deselect(void)
{
	int r = -1;

	if (data->bb_clk2 != NULL) {
		if (data->clk_run == true) {
			printk(KERN_ERR "clk_disable_unprepare bb_clk2");
			clk_disable_unprepare(data->bb_clk2);
			data->clk_run = false;
		}
		return 0;
	}
	return r;
}

static int tc358762_write_register(struct i2c_client *client, u16 reg,
		u32 value)
{
	int r;
	u8 tx_data[] = {
		/* NOTE: Register address big-endian, data little-endian. */
		(reg >> 8) & 0xff,
		reg & 0xff,
		value & 0xff,
		(value >> 8) & 0xff,
		(value >> 16) & 0xff,
		(value >> 24) & 0xff,
	};
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = tx_data,
			.len = ARRAY_SIZE(tx_data),
		},
	};
	r = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (r < 0) {
		printk(KERN_ERR "tc358762 write failed");
		return r;
	}
	return 0;
}
static int tc358762_write_init_config(void)
{
	int i, r;
	u16 reg;
	u32 value;

	pr_err("\n%s:%d", __func__, __LINE__);
	for (i = 0; i < ARRAY_SIZE(tc358762_init_seq); ++i) {
		reg = tc358762_init_seq[i].reg;
		value = tc358762_init_seq[i].data;
		if (reg == DELAY_INIT_SEQ) {
			mdelay(value);
			continue;
		}
		r = tc358762_write_register(data->client, reg, value);
		if (r) {
			pr_err("failed to write initial config"
					" (write) %d\n", i);
			return r;
		}
	}
	pr_err("\n%s:%d Successfully wrote the init config", __func__,\
			__LINE__);
	return 0;
}
#if 0
static int tc358762_write_deinit_config(void)
{
	int i, r;
	u16 reg;
	u32 value;

	pr_err("\n%s:%d", __func__, __LINE__);
	for (i = 0; i < ARRAY_SIZE(tc358762_denit_seq); ++i) {
		reg = tc358762_denit_seq[i].reg;
		value = tc358762_denit_seq[i].data;
		if (reg == DELAY_INIT_SEQ) {
			mdelay(value);
			continue;
		}
		r = tc358762_write_register(data->client, reg, value);
		if (r) {
			pr_err("failed to write initial config"
					" (write) %d\n", i);
			return r;
		}
	}
	pr_err("\n%s:%d Successfully wrote the deinit config", __func__,\
		__LINE__);
	return 0;
}
#endif
static int tc358762_power_on(bool on)
{
	int rc;

	if (!on)
		goto power_off;

	pr_err("\n%s:%d", __func__, __LINE__);

	rc = regulator_enable(data->vdd_l17);
	if (rc) {
		dev_err(&data->client->dev,
				"Regulator vdd_l17 enable failed rc=%d\n", rc);
		return rc;
	} else
		pr_err("\n %s vdds_l17:%d", __func__,
			regulator_get_voltage(data->vdda_l2));

	pr_err("\n%s:%d Successfully enabled the reg voltages",\
			__func__, __LINE__);

	return rc;

power_off:

	rc = regulator_disable(data->vdd_l17);
	if (rc)
		dev_err(&data->client->dev,
				"Regulator vdd_l17 disable failed rc=%d\n", rc);
	return rc;
}

static int tc358762_power_init(void)
{
	int rc;

	pr_err("\n%s:%d", __func__, __LINE__);
	data->vdda_l2 = regulator_get(&data->client->dev, "vdda_l2");
	if (IS_ERR(data->vdda_l2)) {
		rc = PTR_ERR(data->vdda_l2);
		dev_err(&data->client->dev,
				"Regulator get failed vdda_l2 rc=%d\n", rc);
		return rc;
	}
	if (regulator_count_voltages(data->vdda_l2) > 0) {
		rc = regulator_set_voltage(data->vdda_l2, TC_L2_MIN_UV,
				TC_L2_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator set_vtg failed vdda_l2 rc=%d\n", rc);
			goto release_vdda_l2;
		}
	}
	rc = regulator_enable(data->vdda_l2);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vdda_l2 enable failed rc=%d\n", rc);
		return rc;
	} else
		pr_err("\n %s vdds_l2:%d", __func__,\
			regulator_get_voltage(data->vdda_l2));

	data->vdd_l17 = regulator_get(&data->client->dev, "vdd_l17");

	if (IS_ERR(data->vdd_l17)) {
		rc = PTR_ERR(data->vdd_l17);
		dev_err(&data->client->dev,
				"Regulator get failed vdd_l17 rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(data->vdd_l17) > 0) {
		rc = regulator_set_voltage(data->vdd_l17, TC_L17_MIN_UV,
				TC_L17_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator set_vtg failed vdd_l17 rc=%d\n", rc);
			goto release_vdd_l17;
		}
	}
	rc = regulator_enable(data->vdd_l17);
	if (rc) {
		dev_err(&data->client->dev,
				"Regulator vdd_l17 enable failed rc=%d\n", rc);
		return rc;
	} else
		pr_err("\n %s vdds_l17:%d", __func__,
				regulator_get_voltage(data->vdda_l2));
	data->vdds_l6 = regulator_get(&data->client->dev, "vdds_l6");
	if (IS_ERR(data->vdds_l6)) {
		rc = PTR_ERR(data->vdds_l6);
		dev_err(&data->client->dev,
				"Regulator get failed vdds_l6 rc=%d\n", rc);
		return rc;
	}
	if (regulator_count_voltages(data->vdds_l6) > 0) {
		rc = regulator_set_voltage(data->vdds_l6, TC_L6_MIN_UV,
				TC_L6_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator set_vtg failed vdds_l6 rc=%d\n", rc);
			goto release_vdds_l6;
		}
	}
	rc = regulator_enable(data->vdds_l6);
	if (rc) {
		dev_err(&data->client->dev,
				"Regulator vdds_l6 enable failed rc=%d\n", rc);
		return rc;
	} else
		pr_err("\n %s vdds_l6:%d", __func__,\
			regulator_get_voltage(data->vdds_l6));
	pr_err("\n%s:%d successfully parsed the reg voltages", __func__,
			__LINE__);
	return rc;

release_vdds_l6:
	regulator_put(data->vdds_l6);
release_vdda_l2:
	regulator_put(data->vdda_l2);
release_vdd_l17:
	regulator_put(data->vdd_l17);
	return rc;
}

static int toshiba_parse_dt(struct device *dev,
		struct tc358762_platform_data *pdata)
{
	int rc, ret;
	struct device_node *np = dev->of_node;

	pr_err("\n%s:%d", __func__, __LINE__);
	pdata->name = "toshiba";
	rc = of_property_read_string(np, "toshiba,name", &pdata->name);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read name\n");
		return rc;
	}
	pdata->i2c_pull_up = of_property_read_bool(np,
			"toshiba,i2c-pull-up");
	pdata->splash_disabled = of_property_read_bool(np,
			"toshiba,splash-disabled");
	pdata->reset_gpio = of_get_named_gpio_flags(np, "toshiba,reset-gpio",
			0, &pdata->reset_gpio_flags);
	if (pdata->reset_gpio < 0)
		return pdata->reset_gpio;
	ret = gpio_direction_output(pdata->reset_gpio, 2);
	gpio_set_value(pdata->reset_gpio, 1);
	if (ret) {
		printk("%s: reset_gpio %d output dir failed\n",
				__func__, pdata->reset_gpio);
		return pdata->reset_gpio;
	}
	pr_err("\n%s:%d", __func__, __LINE__);

	return 0;
}
void tc358762_reset(void)
{
	pr_debug("\n%s:%d", __func__, __LINE__);
	gpio_set_value(pdata->reset_gpio, 0);
	udelay(100);
	gpio_set_value(pdata->reset_gpio, 1);
}

void tc358762_suspend(void)
{
	pr_debug("\n%s:%d", __func__, __LINE__);
	tc358762_clock_deselect();
	mdelay(5);
	tc358762_power_on(false);
}

void tc358762_resume(void)
{
	pr_debug("\n%s:%d", __func__, __LINE__);
	tc358762_power_on(true);
	if (!tc358762_clock_select()) {
		mdelay(5);
		gpio_set_value(pdata->reset_gpio, 0);
		mdelay(20);
		gpio_set_value(pdata->reset_gpio, 1);
		tc358762_write_init_config();
	}
}

static int tc358762_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int err = 0;

	pr_err("\n%s:%d", __func__, __LINE__);

	if (client->dev.of_node) {
		pr_debug("\n%s:%d platform str allocation and dt parsing",\
			__func__, __LINE__);
		pdata = devm_kzalloc(&client->dev,
				sizeof(struct tc358762_platform_data),
				GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "%s: Failed to allocate memory\n",
				__func__);
			return -ENOMEM;
		}

		err = toshiba_parse_dt(&client->dev, pdata);
		if (err) {
			dev_err(&client->dev, "DT parsing failed\n");
			return err;
		}
	} else
		pdata = client->dev.platform_data;
	if (!pdata) {
		dev_err(&client->dev, "Invalid pdata\n");
		return -EINVAL;
	}

	data = kzalloc(sizeof(struct tc358762_data), GFP_KERNEL);
	if (!data) {
		printk(KERN_ERR "%s: Failed to allocate memory\n", __func__);
		err = -ENOMEM;
		goto err_free_mem;
	}
	data->client = client;
	i2c_set_clientdata(client, data);
	err = tc358762_power_init();
	if (err)
		return err;
	tc358762_clock_select();
	if  (pdata->splash_disabled) {
		pr_err("\n%s:%d", __func__, __LINE__);
		mdelay(5);
		gpio_set_value(pdata->reset_gpio, 0);
		mdelay(20);
		gpio_set_value(pdata->reset_gpio, 1);
		tc358762_write_init_config();
	}
	pr_err("\n %s:%d tc358762 probe succcedded splash_disabled = %d",\
		__func__, __LINE__, pdata->splash_disabled);

	return 0;
err_free_mem:
	data = NULL;
	kfree(data);
	return err;
}

static int toshiba_i2c_remove(struct i2c_client *client)
{
	struct tc358762_data *data = i2c_get_clientdata(client);

	i2c_set_clientdata(client, NULL);
	data = NULL;
	kfree(data);
	return 0;
}

static const struct i2c_device_id toshiba_i2c_idtable[] = {
	{"tc358762", 0},
	{ }
};

MODULE_DEVICE_TABLE(i2c, toshiba_i2c_idtable);
static struct of_device_id toshiba_match_table[] = {
	{ .compatible = "toshiba,tc358762",},
	{ },
};

static struct i2c_driver toshiba_i2c_driver = {
	.driver = {
		.name = "tc358762",
		.owner = THIS_MODULE,
		.of_match_table = toshiba_match_table,
	},
	.id_table = toshiba_i2c_idtable,
	.probe = tc358762_i2c_probe,
	.remove = toshiba_i2c_remove,
};

static int __init tc358762_init(void)
{
	int r = 0;

	r = i2c_add_driver(&toshiba_i2c_driver);
	if (r < 0) {
		printk(KERN_WARNING "tc358762 i2c driver registration\
			failed\n");
		return r;
	}
	return 0;
}

static void __exit tc358762_exit(void)
{
	i2c_del_driver(&toshiba_i2c_driver);
}


module_init(tc358762_init);
module_exit(tc358762_exit);
MODULE_AUTHOR("BORQS");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("TC358762 DSI-2-DPI Bridge Driver");
