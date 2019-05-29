/*!
 * @section LICENSE
 * (C) Copyright 2011~2015 Bosch Sensortec GmbH All Rights Reserved
 *
 * This software program is licensed subject to the GNU General
 * Public License (GPL).Version 2,June 1991,
 * available at http://www.fsf.org/copyleft/gpl.html
 *
 * @filename bmp280_i2c.c
 * @date     "Thu Nov 6 15:03:52 2014 +0800"
 * @id       "c0aa5f9"
 *
 * @brief
 * This file implements module function, which adds
 * the driver to I2C core.
*/

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include "bmp280_core.h"

static struct i2c_client *bmp_i2c_client;

/*! @defgroup bmp280_i2c_src
 *  @brief bmp280 i2c driver module
 @{*/
/*! maximum retry times during i2c transfer */
#define BMP_MAX_RETRY_I2C_XFER 10
/*! wait time after i2c transfer error occurred */
#define BMP_I2C_WRITE_DELAY_TIME 1

struct regulator_map {
	struct regulator        *regulator;
	int                     min_uv;
	int                     max_uv;
	char                    *supply;
};

static struct regulator_map power_config[] = {
	{.supply = "vdd", .min_uv = 1800000, .max_uv = 1800000, },
};


#ifdef BMP_USE_BASIC_I2C_FUNC
/*!
 * @brief define i2c write function
 *
 * @param client the pointer of i2c client
 * @param reg_addr register address
 * @param data the pointer of data buffer
 * @param len block size need to write
 *
 * @return zero success, non-zero failed
 * @retval zero success
 * @retval non-zero failed
*/
static s8 bmp_i2c_read(struct i2c_client *client, u8 reg_addr,
		u8 *data, u8 len)
{
	int retry;

	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = 1,
		 .buf = &reg_addr,
		},

		{
		 .addr = client->addr,
		 .flags = I2C_M_RD,
		 .len = len,
		 .buf = data,
		 },
	};

	for (retry = 0; retry < BMP_MAX_RETRY_I2C_XFER; retry++) {
		if (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) > 0)
			break;

		mdelay(BMP_I2C_WRITE_DELAY_TIME);
	}

	if (BMP_MAX_RETRY_I2C_XFER <= retry) {
		PERR("I2C xfer error");
		return -EIO;
	}

	return 0;
}

/*!
 * @brief define i2c write function
 *
 * @param client the pointer of i2c client
 * @param reg_addr register address
 * @param data the pointer of data buffer
 * @param len block size need to write
 *
 * @return zero success, non-zero failed
 * @retval zero success
 * @retval non-zero failed
*/
static s8 bmp_i2c_write(struct i2c_client *client, u8 reg_addr,
		u8 *data, u8 len)
{
	u8 buffer[2];
	int retry;
	struct i2c_msg msg = {
		 .addr = client->addr,
		 .flags = 0,
		 .len = 2,
		 .buf = buffer,
	};

	while (0 != len--) {
		msg.buf = buffer;
		msg.buf[0] = reg_addr;
		msg.buf[1] = *data;
		for (retry = 0; retry < BMP_MAX_RETRY_I2C_XFER; retry++) {
			if (i2c_transfer(client->adapter, &msg, 1) > 0)
				break;
			mdelay(BMP_I2C_WRITE_DELAY_TIME);
		}
		if (BMP_MAX_RETRY_I2C_XFER <= retry) {
			PERR("I2C xfer error");
			return -EIO;
		}
		reg_addr++;
		data++;
	}

	return 0;
}
#endif/*BMP_USE_BASIC_I2C_FUNC*/

/*!
 * @brief define i2c block write function
 *
 * @param dev_addr sensor i2c address
 * @param reg_addr register address
 * @param data the pointer of data buffer
 * @param len block size need to write
 *
 * @return zero success, non-zero failed
 * @retval zero success
 * @retval non-zero failed
*/
static s8 bmp_i2c_write_block(u8 dev_addr, u8 reg_addr, u8 *data, u8 len)
{
	s8 err = 0;

	if (NULL == bmp_i2c_client)
		return -EPERM;

#ifdef BMP_USE_BASIC_I2C_FUNC
	err = bmp_i2c_write(bmp_i2c_client, reg_addr, data, len);
#else
	err = i2c_smbus_write_i2c_block_data(bmp_i2c_client, \
			reg_addr, len, data);
#endif
	if (err < 0)
		return err;

	return 0;
}

/*!
 * @brief define i2c block read function
 *
 * @param dev_addr sensor i2c address
 * @param reg_addr register address
 * @param data the pointer of data buffer
 * @param len block size need to read
 *
 * @return zero success, non-zero failed
 * @retval zero success
 * @retval non-zero failed
*/
static s8 bmp_i2c_read_block(u8 dev_addr, u8 reg_addr, u8 *data, u8 len)
{
	s8 err = 0;

	if (NULL == bmp_i2c_client)
		return -EPERM;

#ifdef BMP_USE_BASIC_I2C_FUNC
	err = bmp_i2c_read(bmp_i2c_client, reg_addr, data, len);
#else
	err = i2c_smbus_read_i2c_block_data(bmp_i2c_client, \
			reg_addr, len, data);
#endif
	if (err < 0)
		return err;

	return 0;
}

/*!
 * @brief i2c bus operation
*/
static const struct bmp_bus_ops bmp_i2c_bus_ops = {
	/**< i2c block write pointer */
	.bus_write  = bmp_i2c_write_block,
	/**< i2c block read pointer */
	.bus_read   = bmp_i2c_read_block
};

static int sensor_power_init(struct device *dev, struct regulator_map *map,
	int size)
{
	int rc;
	int i;

	for (i = 0; i < size; i++) {
		map[i].regulator = devm_regulator_get(dev, map[i].supply);
		if (IS_ERR(map[i].regulator)) {
			rc = PTR_ERR(map[i].regulator);
			dev_err(dev, "Regualtor get failed vdd rc=%d\n", rc);
			goto exit;
		}
		if (regulator_count_voltages(map[i].regulator) > 0) {
			rc = regulator_set_voltage(map[i].regulator,
				map[i].min_uv, map[i].max_uv);
			if (rc) {
				dev_err(dev, "Regulator set failed vdd rc=%d\n",
					rc);
				goto exit;
			}
		}
	}

	return 0;

exit:
	/* Regulator not set correctly */
	for (i = i - 1; i >= 0; i--) {
		if (regulator_count_voltages(map[i].regulator))
			regulator_set_voltage(map[i].regulator, 0,
				map[i].max_uv);
	}

	return rc;
}

int sensor_power_config(struct device *dev, bool enable)
{
	int i;
	int rc = 0;

	pr_err("%s enable = %d\n", __func__, enable);
	if (enable) {
		for (i = 0; i < ARRAY_SIZE(power_config); i++) {
			rc = regulator_enable(power_config[i].regulator);
			if (rc) {
				dev_err(dev, "enable %s failed.\n",
					power_config[i].supply);
				rc = -EINVAL;
				goto exit_enable;
			}
		}
	} else {
		for (i = 0; i < ARRAY_SIZE(power_config); i++) {
			rc = regulator_disable(power_config[i].regulator);
			if (rc) {
				dev_err(dev, "disable %s failed.\n",
					power_config[i].supply);
				rc = -EINVAL;
				goto exit_disable;
			}
		}
	}

	return 0;

exit_enable:
	for (i = i - 1; i >= 0; i--)
		regulator_disable(power_config[i].regulator);

	return rc;

exit_disable:
	for (i = i - 1; i >= 0; i--)
		if (regulator_enable(power_config[i].regulator))
			dev_err(dev, "enable %s failed\n", power_config[i].supply);

	return rc;
}
EXPORT_SYMBOL(sensor_power_config);


/*!
 * @brief BMP probe function via i2c bus
 *
 * @param client the pointer of i2c client
 * @param id the pointer of i2c device id
 *
 * @return zero success, non-zero failed
 * @retval zero success
 * @retval non-zero failed
*/
static int bmp_i2c_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int res = 0;
	struct bmp_data_bus data_bus = {
		.bops = &bmp_i2c_bus_ops,
		.client = client
	};

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		PERR("i2c_check_functionality error!");
		return -EIO;
	}

	if (NULL == bmp_i2c_client)
		bmp_i2c_client = client;
	else{
		PERR("This driver does not support multiple clients!\n");
		return -EINVAL;
	}

	res = sensor_power_init(&client->dev, power_config,
					ARRAY_SIZE(power_config));
	if (res) {
		dev_err(&client->dev, "Power init Failed\n");
		goto err_power_init;
	}

	res = sensor_power_config(&client->dev, true);
	if (res)
		dev_err(&client->dev, "power Enable failed\n");

	if (bmp_probe(&client->dev, &data_bus) != 0) {
		dev_err(&client->dev, "Driver Probe Failed\n");
		sensor_power_config(&client->dev, false);
		res = -EINVAL;
		goto err_power_init;
	}
	sensor_power_config(&client->dev, false);

	return 0;
err_power_init:
	return res;
}

/*!
 * @brief shutdown bmp device in i2c driver
 *
 * @param client the pointer of i2c client
 *
 * @return no return value
*/
static void bmp_i2c_shutdown(struct i2c_client *client)
{
#ifdef CONFIG_PM
	bmp_disable(&client->dev);
#endif
}

/*!
 * @brief remove bmp i2c client
 *
 * @param client the pointer of i2c client
 *
 * @return zero
 * @retval zero
*/
static int bmp_i2c_remove(struct i2c_client *client)
{
	return bmp_remove(&client->dev);
}

#ifdef CONFIG_PM
/*!
 * @brief suspend bmp device in i2c driver
 *
 * @param dev the pointer of device
 *
 * @return zero
 * @retval zero
*/
static int bmp_i2c_suspend(struct device *dev)
{
	int res = 0;
	struct bmp_client_data *data = dev_get_drvdata(dev);

	bmp_disable(dev);
	if (data->enable) {
		res = sensor_power_config(dev, false);
		if (res) {
			pr_err("power Enable failed\n");
			res = -EINVAL;
		}
	}
	return res;
}

/*!
 * @brief resume bmp device in i2c driver
 *
 * @param dev the pointer of device
 *
 * @return zero
 * @retval zero
*/
static int bmp_i2c_resume(struct device *dev)
{
	int res = 0;
	struct bmp_client_data *data = dev_get_drvdata(dev);

	bmp_enable(dev);
	if (data->enable) {
		res = sensor_power_config(dev, true);
		if (res) {
			pr_err("power Enable failed\n");
			res = -EINVAL;
		}
	}
	return res;
}

/*!
 * @brief register i2c device power manager hooks
*/
static const struct dev_pm_ops bmp_i2c_pm_ops = {
	/**< device suspend */
	.suspend = bmp_i2c_suspend,
	/**< device resume */
	.resume = bmp_i2c_resume
};
#endif

/*!
 * @brief register i2c device id
*/
static const struct i2c_device_id bmp_id[] = {
	{ BMP_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, bmp_id);

static struct of_device_id bmp_match_table[] = {
	{ .compatible = "bosch,bmp280", },
	{ },
};

/*!
 * @brief register i2c driver hooks
*/
static struct i2c_driver bmp_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name  = BMP_NAME,
		.of_match_table = bmp_match_table,
#ifdef CONFIG_PM
		.pm    = &bmp_i2c_pm_ops,
#endif
	},
	.id_table   = bmp_id,
	.probe      = bmp_i2c_probe,
	.shutdown   = bmp_i2c_shutdown,
	.remove     = bmp_i2c_remove
};

/*!
 * @brief initialize bmp i2c module
 *
 * @return zero success, non-zero failed
 * @retval zero success
 * @retval non-zero failed
*/
static int __init bmp_i2c_init(void)
{
	return i2c_add_driver(&bmp_i2c_driver);
}

/*!
 * @brief remove bmp i2c module
 *
 * @return no return value
*/
static void __exit bmp_i2c_exit(void)
{
	i2c_del_driver(&bmp_i2c_driver);
}


MODULE_DESCRIPTION("BMP280 I2C DRIVER");
MODULE_LICENSE("GPL v2");

module_init(bmp_i2c_init);
module_exit(bmp_i2c_exit);
/*@}*/
