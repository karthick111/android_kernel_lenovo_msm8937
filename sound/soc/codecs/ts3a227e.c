#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/slab.h>

#include "ts3a227e.h"

/* TS3A227E registers */
#define TS3A227E_REG_DEVICE_ID		0x00
#define TS3A227E_REG_INTERRUPT		0x01
#define TS3A227E_REG_KP_INTERRUPT	0x02
#define TS3A227E_REG_INTERRUPT_DISABLE	0x03
#define TS3A227E_REG_SETTING_1		0x04
#define TS3A227E_REG_SETTING_2		0x05
#define TS3A227E_REG_SETTING_3		0x06
#define TS3A227E_REG_SWITCH_CONTROL_1	0x07
#define TS3A227E_REG_SWITCH_CONTROL_2	0x08
#define TS3A227E_REG_SWITCH_STATUS_1	0x09
#define TS3A227E_REG_SWITCH_STATUS_2	0x0a
#define TS3A227E_REG_ACCESSORY_STATUS	0x0b
#define TS3A227E_REG_ADC_OUTPUT		0x0c
#define TS3A227E_REG_KP_THRESHOLD_1	0x0d
#define TS3A227E_REG_KP_THRESHOLD_2	0x0e
#define TS3A227E_REG_KP_THRESHOLD_3	0x0f
#define TS3A227E_REGISTERS          0x10

/* TS3A227E_REG_INTERRUPT 0x01 */
#define INS_REM_EVENT 0x01
#define DETECTION_COMPLETE_EVENT 0x02

/* TS3A227E_REG_INTERRUPT_DISABLE 0x03 */
#define INS_REM_INT_DISABLE 0x01
#define DETECTION_COMPLETE_INT_DISABLE 0x02
#define ADC_COMPLETE_INT_DISABLE 0x04
#define INTB_DISABLE 0x08

/* TS3A227E_REG_SETTING_1 0x04 */
#define FM_SUPPORT            0x08
#define DET_TRIGGER           0x10
#define AUTO_DET_ENABLE       0x20
#define MANUAL_SWITCH_CONTROL 0x40

/* TS3A227E_REG_SETTING_2 0x05 */
#define KP_ENABLE 0x04

/* TS3A227E_REG_SETTING_3 0x06 */
#define MICBIAS_SETTING_SFT (3)
#define MICBIAS_SETTING_MASK (0x7 << MICBIAS_SETTING_SFT)

/* TS3A227E_REG_SWITCH_STATUS_1 0x07 */
#define SW_S1            0x1
#define SW_S2            0x2
#define SW_RING2_DFET    0x4
#define SW_SLEEVE_DFET   0x8
#define SW_RING2_GNDFET    0x10
#define SW_SLEEVE_GNDFET   0x20

/* TS3A227E_REG_SWITCH_STATUS_2	0x0a */
#define SW_S3GR  0x1
#define SW_S3GS  0x2
#define SW_S3PR  0x4
#define SW_S3PS  0x8

/* TS3A227E_REG_ACCESSORY_STATUS 0x0b */
#define DET_3POLE             0x01
#define DET_4POLE_OMTP        0x02
#define DET_4POLE_STD         0x04

#define DETECT_COMPLETE_TIMEOUT_MS    800

struct ts3a227e {
	struct device *dev;
    struct i2c_client *i2c;
    bool hp_switch; /* only keep state from user space */
    bool manual_switch;
    bool fm_support;
    bool key_press_support;
    bool is_omtp;
    int manual_mode;
	int irq;
    struct completion detect_compl;
	struct pinctrl *pincrl;
	struct pinctrl_state *pin_state_on;
	struct pinctrl_state *pin_state_off;
};

static struct ts3a227e *ts3a_chip = NULL;
/* 3 pole */
const u8 ts3a227e_reg_3pole[TS3A227E_REGISTERS][2] = {
    {TS3A227E_REG_SWITCH_CONTROL_1, SW_S1|SW_RING2_DFET|SW_SLEEVE_DFET|SW_RING2_GNDFET|SW_SLEEVE_GNDFET},
    {TS3A227E_REG_SWITCH_CONTROL_2, SW_S3GS|SW_S3GR},
};

/* 3 pole FM*/
const u8 ts3a227e_reg_3pole_fm[TS3A227E_REGISTERS][2] = {
    {TS3A227E_REG_SWITCH_CONTROL_1, SW_S1|SW_RING2_GNDFET},
    {TS3A227E_REG_SWITCH_CONTROL_2, SW_S3GR},
};

/* 4 pole standard */
const u8 ts3a227e_reg_std[TS3A227E_REGISTERS][2] = {
    {TS3A227E_REG_SWITCH_CONTROL_1, SW_RING2_DFET|SW_RING2_GNDFET},
    {TS3A227E_REG_SWITCH_CONTROL_2, SW_S3PS|SW_S3GR},
};

/* 4 pole standard FM*/
const u8 ts3a227e_reg_std_fm[TS3A227E_REGISTERS][2] = {
    {TS3A227E_REG_SWITCH_CONTROL_1, SW_RING2_GNDFET},
    {TS3A227E_REG_SWITCH_CONTROL_2, SW_S3PS|SW_S3GR},
};

/* 4 pole OMTP */
const u8 ts3a227e_reg_omtp[TS3A227E_REGISTERS][2] = {
    {TS3A227E_REG_SWITCH_CONTROL_1, SW_SLEEVE_DFET|SW_SLEEVE_GNDFET},
    {TS3A227E_REG_SWITCH_CONTROL_2, SW_S3PR|SW_S3GS},
};

/* 4 pole OMTP FM*/
const u8 ts3a227e_reg_omtp_fm[TS3A227E_REGISTERS][2] = {
    {TS3A227E_REG_SWITCH_CONTROL_1, SW_SLEEVE_GNDFET},
    {TS3A227E_REG_SWITCH_CONTROL_2, SW_S3PR|SW_S3GS},
};

static int  ts3a227e_auto_detect(void);

static int ts3a227e_write_reg(struct i2c_client *client, int reg, int value)
{
    int ret;

    //pr_debug("%s: reg:0x%x, value:0x%x\n", __func__, reg, value);
    ret = i2c_smbus_write_byte_data(client, reg, value);
    if (ret < 0)
        dev_err(&client->dev, "%s: reg:%d  ret:%d\n", __func__, reg, ret);
    return ret;
}

static int ts3a227e_read_reg(struct i2c_client *client, int reg)
{
    int ret;

    ret = i2c_smbus_read_byte_data(client, reg);
    if (ret < 0) {
    	dev_err(&client->dev, "%s: reg:%d ret:%d\n", __func__, reg, ret);
    }
    // pr_debug("%s: reg:0x%x, value:0x%x\n", __func__, reg, ret);
    return ret;
}

static int ts3a227e_update_bits(struct i2c_client *client, unsigned int reg, unsigned int mask,
    unsigned int val)
{
    unsigned int orig, tmp;

    orig = ts3a227e_read_reg(client, reg);
    tmp = orig & ~mask;
    tmp |= val & mask;
    ts3a227e_write_reg(client, reg, tmp);
    return 0;
}

/* ts3a227e_dbg_regs
 *
 * Debug attribute to attach to parent device to show core registers
*/
static ssize_t ts3a227e_regs_show(struct device *dev,
			      struct device_attribute *attr, char *buff)
{
	struct ts3a227e *ts3a227e = dev_get_drvdata(dev);
	unsigned int reg;
	char *ptr = buff;
	int ret;

	for (reg = 0x00; reg <= 0x0f; reg++) {
		ret = sprintf(ptr, "%02x = %02x\n",
			      reg, ts3a227e_read_reg(ts3a227e->i2c, reg));
		ptr += ret;
	}

	return ptr - buff;
}

static ssize_t ts3a227e_regs_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct ts3a227e *ts3a227e = dev_get_drvdata(dev);
	char *start = (char *)buf;
	unsigned long reg, value;
    int ret;

	while (*start == ' ')
		start++;
	reg = simple_strtoul(start, &start, 16);
	while (*start == ' ')
		start++;
	ret = kstrtoul(start, 16, &value);
	if (ret) {
        printk("%s: kstrtoul ret:%d\n", __func__, ret);
		return ret;
    }
    ts3a227e_write_reg(ts3a227e->i2c, reg, value);
	return size;
}

static ssize_t ts3a227e_switch_show(struct device *dev,
			      struct device_attribute *attr, char *buff)
{
	struct ts3a227e *ts3a227e = dev_get_drvdata(dev);
	int value;
	char *ptr = buff;

	value = ts3a227e_read_reg(ts3a227e->i2c, TS3A227E_REG_SWITCH_STATUS_1);
	ptr += sprintf(ptr, "Switch 1: %s\n", (value & SW_S1 ) ? "on" : "off");
	ptr += sprintf(ptr, "Switch 2: %s\n", (value & SW_S2 ) ? "on" : "off");
	ptr += sprintf(ptr, "RING2 DFET: %s\n", (value & SW_RING2_DFET ) ? "on" : "off");
	ptr += sprintf(ptr, "SLEEVE DFET: %s\n", (value & SW_SLEEVE_DFET ) ? "on" : "off");
	ptr += sprintf(ptr, "RING2 GNDFET: %s\n", (value & SW_RING2_GNDFET ) ? "on" : "off");
	ptr += sprintf(ptr, "SLEEVE GNDFET: %s\n", (value & SW_SLEEVE_GNDFET ) ? "on" : "off");

	value = ts3a227e_read_reg(ts3a227e->i2c, TS3A227E_REG_SWITCH_STATUS_2);
	ptr += sprintf(ptr, "S3GR: %s\n", (value & SW_S3GR ) ? "on" : "off");
	ptr += sprintf(ptr, "S3GS: %s\n", (value & SW_S3GS ) ? "on" : "off");
	ptr += sprintf(ptr, "S3PR: %s\n", (value & SW_S3PR ) ? "on" : "off");
	ptr += sprintf(ptr, "S3PS: %s\n", (value & SW_S3PS ) ? "on" : "off");

	return ptr - buff;
}

static ssize_t ts3a227e_fm_show(struct device *dev,
			      struct device_attribute *attr, char *buff)
{
	struct ts3a227e *ts3a227e = dev_get_drvdata(dev)	;

	sprintf(buff, "%d\n", ts3a227e->fm_support);
	return strlen(buff);
}

static ssize_t ts3a227e_fm_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct ts3a227e *ts3a227e = dev_get_drvdata(dev);
	unsigned long value;
	int ret;

	ret = kstrtoul(buf, 10, &value);
	if (ret)
		return ret;
	if (value)
		ts3a227e_update_bits(ts3a227e->i2c, TS3A227E_REG_SETTING_1, FM_SUPPORT, FM_SUPPORT);
	else
		ts3a227e_update_bits(ts3a227e->i2c, TS3A227E_REG_SETTING_1, FM_SUPPORT, 0);
	ts3a227e->fm_support = value ? 1:0;
	return size;
}

static int ts3a227e_pinctrl_init(struct device *dev)
{
	const char *pin_on_names = "pmx_hp_switch_on";
	const char *pin_off_names = "pmx_hp_switch_off";
	struct ts3a227e *ts3a227e = dev_get_drvdata(dev);

	ts3a227e->pincrl = devm_pinctrl_get(dev);
	if (IS_ERR(ts3a227e->pincrl)) {
		pr_err("%s: Unable to get pinctrl handle\n",
				__func__);
		return -EINVAL;
	}

	ts3a227e->pin_state_on = pinctrl_lookup_state(ts3a227e->pincrl, pin_on_names);
	if (IS_ERR(ts3a227e->pin_state_on))
		pr_err("%s: Unable to get pinctrl handle for %s\n",
			__func__, pin_on_names);

	ts3a227e->pin_state_off = pinctrl_lookup_state(ts3a227e->pincrl, pin_off_names);
	if (IS_ERR(ts3a227e->pin_state_off))
		pr_err("%s: Unable to get pinctrl handle for %s\n",
			__func__, pin_off_names);

	return 0;
}

int ts3a227e_switch(int mode)
{
	bool fm_support = false;
	int i;

	pr_info("%s: mode: %d\n", __func__, mode);
    if (!ts3a_chip) {
        pr_warn("%s: ts3a227e not been initialzed\n", __func__);
        return 0;
    }

	fm_support = ts3a_chip->fm_support;
	switch(mode) {
	case TS3A227E_SWITCH_AUTO:
		mode = ts3a227e_auto_detect();
		return mode;
    case TS3A227E_SWITCH_3POLE:
        if (fm_support)
            for (i = 0; i < ARRAY_SIZE(ts3a227e_reg_3pole_fm); i++)
                ts3a227e_write_reg(ts3a_chip->i2c, ts3a227e_reg_3pole_fm[i][0], ts3a227e_reg_3pole_fm[i][1]);
        else
            for (i = 0; i < ARRAY_SIZE(ts3a227e_reg_3pole); i++)
                ts3a227e_write_reg(ts3a_chip->i2c, ts3a227e_reg_3pole[i][0], ts3a227e_reg_3pole[i][1]);
        break;
    case TS3A227E_SWITCH_4POLE_STD:
        if (fm_support)
            for (i = 0; i < ARRAY_SIZE(ts3a227e_reg_std_fm); i++)
                ts3a227e_write_reg(ts3a_chip->i2c, ts3a227e_reg_std_fm[i][0], ts3a227e_reg_std_fm[i][1]);
        else
            for (i = 0; i < ARRAY_SIZE(ts3a227e_reg_std); i++)
                ts3a227e_write_reg(ts3a_chip->i2c, ts3a227e_reg_std[i][0], ts3a227e_reg_std[i][1]);
        break;
    case TS3A227E_SWITCH_4POLE_OMTP:
        if (fm_support)
            for (i = 0; i < ARRAY_SIZE(ts3a227e_reg_omtp_fm); i++)
                ts3a227e_write_reg(ts3a_chip->i2c, ts3a227e_reg_omtp_fm[i][0], ts3a227e_reg_omtp_fm[i][1]);
        else
            for (i = 0; i < ARRAY_SIZE(ts3a227e_reg_omtp); i++)
                ts3a227e_write_reg(ts3a_chip->i2c, ts3a227e_reg_omtp[i][0], ts3a227e_reg_omtp[i][1]);
        break;
    default:
        pr_warn("%s: invalid mode: %d\n", __func__, mode);
		break;
	}

    ts3a227e_update_bits(ts3a_chip->i2c, TS3A227E_REG_SETTING_1,
        MANUAL_SWITCH_CONTROL, MANUAL_SWITCH_CONTROL);
	return mode;
}

bool ts3a227e_next_switch(void)
{
    int i;

    if (ts3a_chip) {
        ts3a_chip->is_omtp = !ts3a_chip->is_omtp;
        if (ts3a_chip->is_omtp) {
            printk("%s:  OMTP\n", __func__);
            for (i = 0; i < ARRAY_SIZE(ts3a227e_reg_omtp); i++)
                ts3a227e_write_reg(ts3a_chip->i2c, ts3a227e_reg_omtp[i][0], ts3a227e_reg_omtp[i][1]);
            return false;
        } else {
            printk("%s:  standard\n", __func__);
            for (i = 0; i < ARRAY_SIZE(ts3a227e_reg_std); i++)
                ts3a227e_write_reg(ts3a_chip->i2c, ts3a227e_reg_std[i][0], ts3a227e_reg_std[i][1]);
            return true;
        }     
    }
    return true;
}

static int  ts3a227e_auto_detect(void)
{
    int value;
    unsigned long rc;

    pr_debug("%s:\n", __func__);
    if (!ts3a_chip)
        return -1;

	reinit_completion(&ts3a_chip->detect_compl);
    ts3a227e_update_bits(ts3a_chip->i2c, TS3A227E_REG_SETTING_1,
        MANUAL_SWITCH_CONTROL|DET_TRIGGER, DET_TRIGGER);

    pr_debug("%s: wait for interrupt\n", __func__);
	rc = wait_for_completion_timeout(&ts3a_chip->detect_compl,
			msecs_to_jiffies(DETECT_COMPLETE_TIMEOUT_MS));
    if (!rc) {
        pr_err("ts3a227e_auto_detect time out\n");
        return -1;
    } else {
        value = ts3a227e_read_reg(ts3a_chip->i2c, TS3A227E_REG_ACCESSORY_STATUS);
        pr_info("%s: Detction Results: 0x%x\n", __func__, value);
        if (value & DET_3POLE)
			return TS3A227E_SWITCH_3POLE;
		if (value & DET_4POLE_OMTP)
			return TS3A227E_SWITCH_4POLE_OMTP;
		if (value & DET_4POLE_STD)
			return TS3A227E_SWITCH_4POLE_STD;
    }
    return TS3A227E_SWITCH_UKNOWN;
}

int karate_hp_switch_set(bool enable, int user)
{
    pr_info("%s: enable: %d\n", __func__, enable);
    if (!ts3a_chip)
        return -1;

	if (user)
	    ts3a_chip->hp_switch = enable;
	if (enable)
		pinctrl_select_state(ts3a_chip->pincrl, ts3a_chip->pin_state_on);
	else
		pinctrl_select_state(ts3a_chip->pincrl, ts3a_chip->pin_state_off);
    return 0;
}

bool karate_hp_switch_get(void)
{
    if (!ts3a_chip)
        return false;
    return ts3a_chip->hp_switch;
}
 
static DEVICE_ATTR(register, S_IRUGO | S_IWUSR | S_IWGRP,
    ts3a227e_regs_show, ts3a227e_regs_store);

static DEVICE_ATTR(switch, S_IRUGO,
    ts3a227e_switch_show, NULL);

static DEVICE_ATTR(fm_support, S_IRUGO | S_IWUSR | S_IWGRP,
    ts3a227e_fm_show, ts3a227e_fm_store);

static irqreturn_t ts3a227e_interrupt(int irq, void *data)
{
    struct ts3a227e *ts3a227e = (struct ts3a227e *)data;
    unsigned int int_reg, kp_int_reg;

    int_reg = ts3a227e_read_reg(ts3a227e->i2c, TS3A227E_REG_INTERRUPT);
    pr_debug("%s: int_reg:0x%x\n", __func__, int_reg);

	if (int_reg & DETECTION_COMPLETE_EVENT)
        complete(&ts3a227e->detect_compl);

	/* Report any key events. */
	kp_int_reg = ts3a227e_read_reg(ts3a227e->i2c, TS3A227E_REG_KP_INTERRUPT);
    pr_debug("%s: kp_int_reg:0x%x\n", __func__, kp_int_reg);

   return IRQ_HANDLED;
}

static int ts3a227e_i2c_probe(struct i2c_client *i2c,
		      const struct i2c_device_id *id)
{
	struct ts3a227e *ts3a227e;
	struct device *dev = &i2c->dev;
	u32 micbias;
    int device_id;
    int value;
    int ret, i;

    pr_debug("ts3a227e_i2c_probe\n");
	ts3a227e = devm_kzalloc(&i2c->dev, sizeof(*ts3a227e), GFP_KERNEL);
	if (ts3a227e == NULL)
		return -ENOMEM;

	i2c_set_clientdata(i2c, ts3a227e);
    ts3a227e->i2c = i2c;
	ts3a227e->dev = dev;
	ts3a227e->irq = i2c->irq;

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&i2c->dev, "%s: no support for i2c read/write byte data\n", __func__);
		return -EIO;
	}

    device_id = ts3a227e_read_reg(i2c, TS3A227E_REG_DEVICE_ID);
    printk("%s: device_id 0x%x\n", __func__, device_id);

	ts3a227e_pinctrl_init(&i2c->dev);

	ts3a227e->hp_switch = false;
	karate_hp_switch_set(ts3a227e->hp_switch, 0);
    ts3a227e->fm_support = of_property_read_bool(i2c->dev.of_node,"ti,fm_support");
    ts3a227e->manual_switch = of_property_read_bool(i2c->dev.of_node,"ti,manual_switch");
    ts3a227e->key_press_support = of_property_read_bool(i2c->dev.of_node,"ti,key_support");
    pr_info("%s: fm_support:%d, manual switch:%d, key support:%d\n", __func__,
        ts3a227e->fm_support, ts3a227e->manual_switch, ts3a227e->key_press_support);
    ret = of_property_read_u32(i2c->dev.of_node, "ti,micbias", &micbias);
    if (!ret) {
		ts3a227e_update_bits(i2c, TS3A227E_REG_SETTING_3,
			MICBIAS_SETTING_MASK,
			(micbias & 0x07) << MICBIAS_SETTING_SFT);
    }

    value = 0;
    if (ts3a227e->fm_support)
        value |= FM_SUPPORT;
    else
        value &= ~FM_SUPPORT;
    if (ts3a227e->manual_switch)
        value |= MANUAL_SWITCH_CONTROL;
    else
        value &= ~MANUAL_SWITCH_CONTROL;
    ts3a227e_update_bits(i2c, TS3A227E_REG_SETTING_1,
        FM_SUPPORT | MANUAL_SWITCH_CONTROL|AUTO_DET_ENABLE, value);

	init_completion(&ts3a227e->detect_compl);

    if (ts3a227e->key_press_support)
        ts3a227e_update_bits(i2c, TS3A227E_REG_SETTING_2, KP_ENABLE, KP_ENABLE);

    ts3a227e_update_bits(i2c, TS3A227E_REG_INTERRUPT_DISABLE,
                          INTB_DISABLE | ADC_COMPLETE_INT_DISABLE | DETECTION_COMPLETE_INT_DISABLE,
                          ADC_COMPLETE_INT_DISABLE);

    /* if manual control, default to switch standard type*/
    ts3a227e->is_omtp = false;
    if (ts3a227e->manual_switch) {
        for (i = 0; i < ARRAY_SIZE(ts3a227e_reg_std); i++)
            ts3a227e_write_reg(i2c, ts3a227e_reg_std[i][0], ts3a227e_reg_std[i][1]);
    }

   ret = devm_request_threaded_irq(dev, i2c->irq, NULL, ts3a227e_interrupt,
                                   IRQF_TRIGGER_LOW | IRQF_ONESHOT,
                                   "TS3A227E", ts3a227e);
    if (ret) {
        dev_err(dev, "Cannot request irq %d (%d)\n", i2c->irq, ret);
        return ret;
    }
	ret = device_create_file(&i2c->dev, &dev_attr_register);
	if (ret)
		dev_err(&i2c->dev, "failed to create register file\n");
	ret = device_create_file(&i2c->dev, &dev_attr_switch);
	if (ret)
		dev_err(&i2c->dev, "failed to create switch file\n");
	ret = device_create_file(&i2c->dev, &dev_attr_fm_support);
	if (ret)
		dev_err(&i2c->dev, "failed to create fm support file\n");

    ts3a_chip = ts3a227e;
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int ts3a227e_suspend(struct device *dev)
{
	struct ts3a227e *ts3a227e = dev_get_drvdata(dev);

	dev_dbg(ts3a227e->dev, "suspend disable irq\n");
	disable_irq(ts3a227e->irq);

	return 0;
}

static int ts3a227e_resume(struct device *dev)
{
	struct ts3a227e *ts3a227e = dev_get_drvdata(dev);

	dev_dbg(ts3a227e->dev, "resume enable irq\n");
	enable_irq(ts3a227e->irq);

	return 0;
}
#endif

static const struct dev_pm_ops ts3a227e_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(ts3a227e_suspend, ts3a227e_resume)
};

static const struct i2c_device_id ts3a227e_i2c_ids[] = {
	{ "ts3a227e", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ts3a227e_i2c_ids);

static const struct of_device_id ts3a227e_of_match[] = {
	{ .compatible = "ti,ts3a227e", },
	{ }
};
MODULE_DEVICE_TABLE(of, ts3a227e_of_match);

static struct i2c_driver ts3a227e_driver = {
	.driver = {
		.name = "ts3a227e",
		.pm = &ts3a227e_pm,
		.of_match_table = of_match_ptr(ts3a227e_of_match),
	},
	.probe = ts3a227e_i2c_probe,
	.id_table = ts3a227e_i2c_ids,
};
module_i2c_driver(ts3a227e_driver);

MODULE_DESCRIPTION("ASoC ts3a227e driver");
MODULE_AUTHOR("Jonathan Huang<huangys@lenovo.com>");
MODULE_LICENSE("GPL v2");
