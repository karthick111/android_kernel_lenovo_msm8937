/*
 * SPL-06001 driver Pressure Sensor Driver for
 * various purposes
 * Copyright (C) 2016 Borqs software solutions.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/sensors.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/kthread.h>
#include "spl06.h"

/**
 * struct spl06_sensor -
 * @baro_dev - Pointer to the input device
 * @client - Pointer to the I2C client
 * @regmap - Register map of the device
 * @work - Work item used to off load the enable/disable of the vibration
 * @regulator - Pointer to the regulator for the IC
 * @magnitude - Magnitude of the vibration event
 **/
struct spl06_sensor {
	struct input_dev *baro_dev;
	struct device *dev;
	struct sensors_classdev baro_cdev;
	struct i2c_client *client;
	struct regmap *regmap;
	int baro_wkp_flag;
	struct work_struct work;
	struct regulator *regulator;
	unsigned int  baro_poll_ms;
	struct task_struct *baro_task;
	wait_queue_head_t       baro_wq;
	struct hrtimer baro_timer;
};

static struct sensors_classdev spl06_baro_cdev = {
	.name = "spl06-baro",
	.vendor = "GoerTek",
	.version = 1,
	.handle = SENSORS_PRESSURE_HANDLE,
	.type = SENSOR_TYPE_PRESSURE,
	.max_range = "156.8",   /* m/s^2 */
	.min_delay = SPL06_BARO_MIN_POLL_INTERVAL_MS * 1000,
	.max_delay = SPL06_BARO_MAX_POLL_INTERVAL_MS,
	.delay_msec = SPL06_BARO_DEFAULT_POLL_INTERVAL_MS,
};

static int i2c_read_register(struct i2c_client *client, char start_addr,
	u8 *buffer, int length)
{
	/*
	 * Annoying we can't make this const because the i2c layer doesn't
	 * declare input buffers const.
	 */
	struct i2c_msg msg[2] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &start_addr,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = buffer,
		},
	};

	return i2c_transfer(client->adapter, msg, 2);
}

static int i2c_write_register(struct i2c_client *client, int addr, int data)
{
	/*
	 * Annoying we can't make this const because the i2c layer doesn't
	 * declare input buffers const.
	 */

	return i2c_smbus_write_byte_data(client, addr, data);
}


enum hrtimer_restart baro_timer_handle(struct hrtimer *hrtimer)
{
	ktime_t ktime;
	struct spl06_sensor *sensor =  container_of(hrtimer, struct spl06_sensor, baro_timer);

	sensor->baro_wkp_flag = 1;
	wake_up_interruptible(&sensor->baro_wq);
	ktime = ktime_set(0, sensor->baro_poll_ms * NSEC_PER_MSEC);
	hrtimer_start(&sensor->baro_timer, ktime, HRTIMER_MODE_REL);
	return  HRTIMER_NORESTART;
}

static int baro_poll_thread(void *data)
{
	u8 reg_read = 0;
	int pressure = 0;
	ktime_t ts;
	struct spl06_sensor *sensor = data;

	while (1) {
		wait_event_interruptible(sensor->baro_wq,
				((sensor->baro_wkp_flag != 0) ||
					kthread_should_stop()));
		sensor->baro_wkp_flag = 0;
		if (kthread_should_stop())
			break;

		ts = ktime_get_boottime();
		i2c_read_register(sensor->client, PRS_B2, &reg_read, 1);
		pressure = reg_read << 16;
		i2c_read_register(sensor->client, PRS_B1, &reg_read, 1);
		pressure = pressure | (reg_read << 8);
		i2c_read_register(sensor->client, PRS_B0, &reg_read, 1);
		pressure = pressure | reg_read;
		/*Input Events*/
		input_report_abs(sensor->baro_dev, ABS_PRESSURE, pressure);
		input_event(sensor->baro_dev, EV_SYN, SYN_TIME_SEC,
				ktime_to_timespec(ts).tv_sec);
		input_event(sensor->baro_dev, EV_SYN, SYN_TIME_NSEC,
				ktime_to_timespec(ts).tv_nsec);
		input_sync(sensor->baro_dev);
	}
	return 0;
}

static int spl06_init(struct i2c_client *client)
{

	u8 chip_id = 0;

	pr_info("spl06 sensor Initftn\r\n");
	if (i2c_read_register(client, CHIPID, &chip_id, 1) < 0) {
		pr_err("I2C read failed\r\n");
		return -EINVAL;
	}
	pr_info("spl06 Chip_ID:%x\n", chip_id);
	return 0;
}

static int spl06_baro_set_poll_delay(struct spl06_sensor *sensor,
		unsigned long delay)
{
	int ret = 0;
	ktime_t ktime;

	if (sensor->baro_poll_ms == delay)
		return ret;

	hrtimer_try_to_cancel(&sensor->baro_timer);
	if (delay < SPL06_BARO_MIN_POLL_INTERVAL_MS)
		delay = SPL06_BARO_MIN_POLL_INTERVAL_MS;
	if (delay > SPL06_BARO_MAX_POLL_INTERVAL_MS)
		delay = SPL06_BARO_MAX_POLL_INTERVAL_MS;
	if (0 == delay)
		delay = SPL06_BARO_DEFAULT_POLL_INTERVAL_MS;
	sensor->baro_poll_ms = delay;
	ktime = ktime_set(0, sensor->baro_poll_ms * NSEC_PER_MSEC);
	hrtimer_start(&sensor->baro_timer, ktime, HRTIMER_MODE_REL);
	return ret;
}

static int spl06_baro_cdev_enable(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct spl06_sensor *baro = container_of(sensors_cdev,
			struct spl06_sensor, baro_cdev);
	int err = 0;
	ktime_t ktime;

	if (enable) {
		if (baro->regulator != NULL) {
			err = regulator_enable(baro->regulator);
			/*Instilisation of Sensor*/
			i2c_write_register(baro->client, SET_PRES, READ_PRES);
			pr_info("regulator enabled\r\n");
			ktime = ktime_set(0, baro->baro_poll_ms * NSEC_PER_MSEC);
			hrtimer_start(&baro->baro_timer, ktime, HRTIMER_MODE_REL);
		}
	} else {
		err = hrtimer_try_to_cancel(&baro->baro_timer);
		/*Disabling of Sensor*/
		i2c_write_register(baro->client, SET_PRES, READ_PRES_DIS);
		if (baro->regulator != NULL) {
			err = regulator_disable(baro->regulator);
			pr_info("regulator disabled\r\n");
		}
	}
	return err;
}

static int spl06_baro_cdev_poll_delay(struct sensors_classdev *sensors_cdev,
		unsigned int delay_ms)
{
	struct spl06_sensor *baro = container_of(sensors_cdev,
			struct spl06_sensor, baro_cdev);

	return spl06_baro_set_poll_delay(baro, delay_ms);
}

static int spl06_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct spl06_sensor *baro;
	int error = 0x00;

	pr_info("Pressure Sensor spl06  Probe\r\n");
	baro = devm_kzalloc(&client->dev, sizeof(*baro), GFP_KERNEL);
	if (!baro)
		return -ENOMEM;

	baro->regulator = regulator_get(&client->dev, "vdd");
	if (IS_ERR(baro->regulator)) {
		error = PTR_ERR(baro->regulator);
		dev_err(&client->dev,
				"unable to get regulator, error: %d\n", error);
		return error;
	}

	error = regulator_enable(baro->regulator);
	if (error)
		dev_err(&client->dev, "regulator enable failed\n");

	baro->baro_dev = devm_input_allocate_device(&client->dev);
	if (!baro->baro_dev) {
		dev_err(&client->dev, "Failed to allocate input device\n");
		return -ENOMEM;
	}

	/*Event Generation*/
	baro->baro_dev->name = "spl06";
	baro->baro_dev->id.bustype = BUS_I2C;
	baro->baro_poll_ms = SPL06_BARO_DEFAULT_POLL_INTERVAL_MS;
	baro->baro_dev->dev.parent = client->dev.parent;
	set_bit(EV_ABS, baro->baro_dev->evbit);
	input_set_abs_params(baro->baro_dev, ABS_PRESSURE, 0, 65535, 0, 0);
	input_set_drvdata(baro->baro_dev, baro);
	error = input_register_device(baro->baro_dev);
	if (error)
		pr_err("baro event file failed!\r\n");

	baro->client = client;
	i2c_set_clientdata(client, baro);

	/*Chip -ID*/
	error = spl06_init(client);
	if (error) {
		dev_err(&client->dev, "no SPL06 sensor\n");
		return -ENODEV;
	}

	/*hr timer*/
	hrtimer_init(&baro->baro_timer, CLOCK_BOOTTIME, HRTIMER_MODE_REL);
	baro->baro_timer.function = baro_timer_handle;

	/*polling thread*/
	init_waitqueue_head(&baro->baro_wq);
	baro->baro_wkp_flag = 0;
	baro->baro_task = kthread_run(baro_poll_thread, baro, "sns_baro");

	/*input in sys/class/sensors*/
	baro->baro_cdev = spl06_baro_cdev;
	baro->baro_cdev.sensors_enable = spl06_baro_cdev_enable;
	baro->baro_cdev.sensors_poll_delay = spl06_baro_cdev_poll_delay;
	error = sensors_classdev_register(&client->dev,
			&baro->baro_cdev);
	if (error)
		pr_err("create baro class device file failed!\r\n");

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int spl06_suspend(struct device *dev)
{
	struct spl06_sensor *baro = dev_get_drvdata(dev);
	int ret = 0;

	ret = regulator_disable(baro->regulator);
	if (ret)
		pr_err("%s: regulator disable failed\n", __func__);
	return ret;
}

static int spl06_resume(struct device *dev)
{
	struct spl06_sensor *baro = dev_get_drvdata(dev);
	int ret = 0;

	ret = regulator_enable(baro->regulator);
	if (ret)
		pr_err("%s: regulator enable failed\n", __func__);
	return ret;
}
#endif

static SIMPLE_DEV_PM_OPS(spl06_pm_ops, spl06_suspend, spl06_resume);

static const struct i2c_device_id spl06_id[] = {
	{ "spl06", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, spl06_id);

#ifdef CONFIG_OF
static const struct of_device_id spl06_of_match[] = {
	{ .compatible = "qcom,spl06", },
	{ }
};
MODULE_DEVICE_TABLE(of, spl06_of_match);
#endif

static struct i2c_driver spl06_driver = {
	.probe		= spl06_probe,
	.driver		= {
		.name	= "spl06",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(spl06_of_match),
		.pm	= &spl06_pm_ops,
	},
	.id_table = spl06_id,
};
module_i2c_driver(spl06_driver);
MODULE_ALIAS("platform:spl06-pressure");
MODULE_DESCRIPTION("SPL06 pressuer driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("ayyappadas.ps@borqs.com>");
