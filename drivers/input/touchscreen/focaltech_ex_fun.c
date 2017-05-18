/*
 *
 * FocalTech fts TouchScreen driver.
 *
 * Copyright (c) 2010-2015, Focaltech Ltd. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

 /*******************************************************************************
*
* File Name: Focaltech_ex_fun.c
*
* Author: Xu YongFeng
*
* Created: 2015-01-29
*
* Modify by mshl on 2015-10-26
*
* Abstract:
*
* Reference:
*
*******************************************************************************/

/*******************************************************************************
* 1.Included header files
*******************************************************************************/
#include "focaltech_core.h"
#include "mcap_test_lib.h"

/*******************************************************************************
* Private constant and macro definitions using #define
*******************************************************************************/
/*create apk debug channel*/
#define PROC_UPGRADE			0
#define PROC_READ_REGISTER		1
#define PROC_WRITE_REGISTER	2
#define PROC_AUTOCLB			4
#define PROC_UPGRADE_INFO		5
#define PROC_WRITE_DATA		6
#define PROC_READ_DATA			7
#define PROC_SET_TEST_FLAG				8
#define PROC_NAME	"ftxxxx-debug"

#define WRITE_BUF_SIZE		512
#define READ_BUF_SIZE		512

#define FTS_DOUBLE_CLICK_BIT	(4)
#define FTS_DOUBLE_CLICK_MASK	(1 << FTS_DOUBLE_CLICK_BIT)

/*******************************************************************************
* Private enumerations, structures and unions using typedef
*******************************************************************************/


/*******************************************************************************
* Static variables
*******************************************************************************/
static unsigned char proc_operate_mode = PROC_UPGRADE;
static struct proc_dir_entry *fts_proc_entry;
#if LENOVO_DOUBLE_CLICK
int enable_double_click;
#endif
/*******************************************************************************
* Global variable or extern global variabls/functions
*******************************************************************************/
#if GTP_ESD_PROTECT
int apk_debug_flag = 0;
#endif
/*******************************************************************************
* Static function prototypes
*******************************************************************************/

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0))
/*interface of write proc*/
/************************************************************************
*   Name: fts_debug_write
*  Brief:interface of write proc
* Input: file point, data buf, data len, no use
* Output: no
* Return: data len
***********************************************************************/
static ssize_t fts_debug_write(struct file *filp, const char __user *buff, size_t count, loff_t *ppos)
{
	unsigned char writebuf[WRITE_BUF_SIZE];
	int buflen = count;
	int writelen = 0;
	int ret = 0;

	if (copy_from_user(&writebuf, buff, buflen)) {
		dev_err(&fts_i2c_client->dev, "%s:copy from user error\n", __func__);
		return -EFAULT;
	}
	proc_operate_mode = writebuf[0];

	switch (proc_operate_mode) {
	case PROC_UPGRADE:
		{
			char upgrade_file_path[128];
			memset(upgrade_file_path, 0, sizeof(upgrade_file_path));
			sprintf(upgrade_file_path, "%s", writebuf + 1);
			upgrade_file_path[buflen-1] = '\0';
			FTS_DBG("%s\n", upgrade_file_path);
			disable_irq(fts_i2c_client->irq);
			#if GTP_ESD_PROTECT
			apk_debug_flag = 1;
			#endif

			ret = fts_ctpm_fw_upgrade_with_app_file(fts_i2c_client, upgrade_file_path);
			#if GTP_ESD_PROTECT
			apk_debug_flag = 0;
			#endif
			enable_irq(fts_i2c_client->irq);
			if (ret < 0) {
				dev_err(&fts_i2c_client->dev, "%s:upgrade failed.\n", __func__);
				return ret;
			}
		}
		break;
	case PROC_READ_REGISTER:
		writelen = 1;
		ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
		if (ret < 0) {
			dev_err(&fts_i2c_client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_WRITE_REGISTER:
		writelen = 2;
		ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
		if (ret < 0) {
			dev_err(&fts_i2c_client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_AUTOCLB:
		FTS_DBG("%s: autoclb\n", __func__);
		fts_ctpm_auto_clb(fts_i2c_client);
		break;
	case PROC_READ_DATA:
	case PROC_WRITE_DATA:
		writelen = count - 1;
		ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
		if (ret < 0) {
			dev_err(&fts_i2c_client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	default:
		break;
	}


	return count;
}

/*interface of read proc*/
/************************************************************************
*   Name: fts_debug_read
*  Brief:interface of read proc
* Input: point to the data, no use, no use, read len, no use, no use
* Output: page point to data
* Return: read char number
***********************************************************************/
static ssize_t fts_debug_read(struct file *filp, char __user *buff, size_t count, loff_t *ppos)
{
	int ret = 0;
	int num_read_chars = 0;
	int readlen = 0;
	u8 regvalue = 0x00, regaddr = 0x00;
	unsigned char buf[READ_BUF_SIZE];

	switch (proc_operate_mode) {
	case PROC_UPGRADE:
		//after calling fts_debug_write to upgrade
		regaddr = 0xA6;
		ret = fts_read_reg(fts_i2c_client, regaddr, &regvalue);
		if (ret < 0)
			num_read_chars = sprintf(buf, "%s", "get fw version failed.\n");
		else
			num_read_chars = sprintf(buf, "current fw version:0x%02x\n", regvalue);
		break;
	case PROC_READ_REGISTER:
		readlen = 1;
		ret = fts_i2c_read(fts_i2c_client, NULL, 0, buf, readlen);
		if (ret < 0) {
			dev_err(&fts_i2c_client->dev, "%s:read iic error\n", __func__);
			return ret;
		}
		num_read_chars = 1;
		break;
	case PROC_READ_DATA:
		readlen = count;
		ret = fts_i2c_read(fts_i2c_client, NULL, 0, buf, readlen);
		if (ret < 0) {
			dev_err(&fts_i2c_client->dev, "%s:read iic error\n", __func__);
			return ret;
		}

		num_read_chars = readlen;
		break;
	case PROC_WRITE_DATA:
		break;
	default:
		break;
	}

	if (copy_to_user(buff, buf, num_read_chars)) {
		dev_err(&fts_i2c_client->dev, "%s:copy to user error\n", __func__);
		return -EFAULT;
	}

	return num_read_chars;
}
static const struct file_operations fts_proc_fops = {
		.owner = THIS_MODULE,
		.read = fts_debug_read,
		.write = fts_debug_write,
};
#else
/*interface of write proc*/
/************************************************************************
*   Name: fts_debug_write
*  Brief:interface of write proc
* Input: file point, data buf, data len, no use
* Output: no
* Return: data len
***********************************************************************/
static int fts_debug_write(struct file *filp,
	const char __user *buff, unsigned long len, void *data)
{
	unsigned char writebuf[WRITE_BUF_SIZE];
	int buflen = len;
	int writelen = 0;
	int ret = 0;

	if (copy_from_user(&writebuf, buff, buflen)) {
		dev_err(&fts_i2c_client->dev, "%s:copy from user error\n", __func__);
		return -EFAULT;
	}
	proc_operate_mode = writebuf[0];

	switch (proc_operate_mode) {

	case PROC_UPGRADE:
		{
			char upgrade_file_path[128];
			memset(upgrade_file_path, 0, sizeof(upgrade_file_path));
			sprintf(upgrade_file_path, "%s", writebuf + 1);
			upgrade_file_path[buflen-1] = '\0';
			FTS_DBG("%s\n", upgrade_file_path);
			disable_irq(fts_i2c_client->irq);
			#if GTP_ESD_PROTECT
				apk_debug_flag = 1;
			#endif
			ret = fts_ctpm_fw_upgrade_with_app_file(fts_i2c_client, upgrade_file_path);
			#if GTP_ESD_PROTECT
				apk_debug_flag = 0;
			#endif
			enable_irq(fts_i2c_client->irq);
			if (ret < 0) {
				dev_err(&fts_i2c_client->dev, "%s:upgrade failed.\n", __func__);
				return ret;
			}
		}
		break;
	case PROC_READ_REGISTER:
		writelen = 1;
		ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
		if (ret < 0) {
			dev_err(&fts_i2c_client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_WRITE_REGISTER:
		writelen = 2;
		ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
		if (ret < 0) {
			dev_err(&fts_i2c_client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_AUTOCLB:
		FTS_DBG("%s: autoclb\n", __func__);
		fts_ctpm_auto_clb(fts_i2c_client);
		break;
	case PROC_READ_DATA:
	case PROC_WRITE_DATA:
		writelen = len - 1;
		ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
		if (ret < 0) {
			dev_err(&fts_i2c_client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	default:
		break;
	}

	return len;
}

/*interface of read proc*/
/************************************************************************
*   Name: fts_debug_read
*  Brief:interface of read proc
* Input: point to the data, no use, no use, read len, no use, no use
* Output: page point to data
* Return: read char number
***********************************************************************/
static int fts_debug_read( char *page, char **start,
	off_t off, int count, int *eof, void *data )
{
	int ret = 0;
	unsigned char buf[READ_BUF_SIZE];
	int num_read_chars = 0;
	int readlen = 0;
	u8 regvalue = 0x00, regaddr = 0x00;

	switch (proc_operate_mode) {
	case PROC_UPGRADE:
		//after calling fts_debug_write to upgrade
		regaddr = 0xA6;
		ret = fts_read_reg(fts_i2c_client, regaddr, &regvalue);
		if (ret < 0)
			num_read_chars = sprintf(buf, "%s", "get fw version failed.\n");
		else
			num_read_chars = sprintf(buf, "current fw version:0x%02x\n", regvalue);
		break;
	case PROC_READ_REGISTER:
		readlen = 1;
		ret = fts_i2c_read(fts_i2c_client, NULL, 0, buf, readlen);
		if (ret < 0) {
			dev_err(&fts_i2c_client->dev, "%s:read iic error\n", __func__);
			return ret;
		}
		num_read_chars = 1;
		break;
	case PROC_READ_DATA:
		readlen = count;
		ret = fts_i2c_read(fts_i2c_client, NULL, 0, buf, readlen);
		if (ret < 0) {
			dev_err(&fts_i2c_client->dev, "%s:read iic error\n", __func__);
			return ret;
		}

		num_read_chars = readlen;
		break;
	case PROC_WRITE_DATA:
		break;
	default:
		break;
	}

	memcpy(page, buf, num_read_chars);
	return num_read_chars;
}
#endif
/************************************************************************
* Name: fts_create_apk_debug_channel
* Brief:  create apk debug channel
* Input: i2c info
* Output: no
* Return: success =0
***********************************************************************/
int fts_create_apk_debug_channel(struct i2c_client * client)
{
	#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0))
		fts_proc_entry = proc_create(PROC_NAME, 0777, NULL, &fts_proc_fops);
	#else
		fts_proc_entry = create_proc_entry(PROC_NAME, 0777, NULL);
	#endif
	if (NULL == fts_proc_entry)
	{
		dev_err(&client->dev, "Couldn't create proc entry!\n");

		return -ENOMEM;
	}
	else
	{
		dev_info(&client->dev, "Create proc entry success!\n");

		#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0))
			fts_proc_entry->write_proc = fts_debug_write;
			fts_proc_entry->read_proc = fts_debug_read;
		#endif
	}
	return 0;
}
/************************************************************************
* Name: fts_release_apk_debug_channel
* Brief:  release apk debug channel
* Input: no
* Output: no
* Return: no
***********************************************************************/
void fts_release_apk_debug_channel(void)
{

	if (fts_proc_entry)
		#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0))
			proc_remove(fts_proc_entry);
		#else
			remove_proc_entry(NULL, fts_proc_entry);
		#endif
}
#if LENOVO_DOUBLE_CLICK
/***********************************************************************
*fts_wake_gesture_show
************************************************************************/
static ssize_t fts_wake_gesture_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 readreg;

	mutex_lock(&fts_input_dev->mutex);

	if (fts_read_reg(fts_i2c_client, 0xd1, &readreg) < 0) {
		dev_err(dev, "fail to disable wake gesture\n");
	}

	mutex_unlock(&fts_input_dev->mutex);
	return snprintf(buf, 128, "0x%02X\n", readreg);
}

static ssize_t fts_wake_gesture_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned long value;
	retval = kstrtoul(buf, 16, &value);
	if (retval)
		return retval;

	dev_info(dev, "fts_wake_gesture_store%ld\n",value);
	mutex_lock(&fts_input_dev->mutex);
	if (value) {
		u8 readreg;
		if (fts_read_reg(fts_i2c_client, 0xd1, &readreg) < 0) {
			dev_err(dev, "fail to enable wake gesture\n");
			goto FTS_WAKE_GESTURE_STORE_OUT;
		}
		readreg |= FTS_DOUBLE_CLICK_MASK;
		if (fts_write_reg(fts_i2c_client, 0xd1, readreg) < 0) {
			dev_err(dev, "fail to enable wake gesture\n");
			goto FTS_WAKE_GESTURE_STORE_OUT;
		}
		enable_double_click = 1;
	}
	else {
		u8 readreg;
		if (fts_read_reg(fts_i2c_client, 0xd1, &readreg) < 0) {
			dev_err(dev, "fail to disable wake gesture\n");
			goto FTS_WAKE_GESTURE_STORE_OUT;
		}
		readreg &= ~FTS_DOUBLE_CLICK_MASK;
		if (fts_write_reg(fts_i2c_client, 0xd1, readreg) < 0) {
			dev_err(dev, "fail to disable wake gesture\n");
			goto FTS_WAKE_GESTURE_STORE_OUT;
		}
		enable_double_click = 0;
	}

FTS_WAKE_GESTURE_STORE_OUT:
	mutex_unlock(&fts_input_dev->mutex);
	return count;
}
#endif
/************************************************************************
* Name: fts_tpfwver_show
* Brief:  show tp fw vwersion
* Input: device, device attribute, char buf
* Output: no
* Return: char number
***********************************************************************/
static ssize_t fts_tpfwver_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t num_read_chars = 0;
	u8 fwver = 0;
	mutex_lock(&fts_input_dev->mutex);
	if (fts_read_reg(fts_i2c_client, FTS_REG_FW_VER, &fwver) < 0)
		return -1;

	if (fwver == 255)
		num_read_chars = snprintf(buf, 128,"get tp fw version fail!\n");
	else
	{
		num_read_chars = snprintf(buf, 128, "%02X\n", fwver);
	}

	mutex_unlock(&fts_input_dev->mutex);

	return num_read_chars;
}
/************************************************************************
* Name: fts_tpfwver_store
* Brief:  no
* Input: device, device attribute, char buf, char count
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_tpfwver_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	/* place holder for future use */
	return -EPERM;
}
/************************************************************************
* Name: fts_tpdriver_version_show
* Brief:  show tp fw vwersion
* Input: device, device attribute, char buf
* Output: no
* Return: char number
***********************************************************************/
static ssize_t fts_tpdriver_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t num_read_chars = 0;

	mutex_lock(&fts_input_dev->mutex);

	num_read_chars = snprintf(buf, 128,"%s \n", FTS_DRIVER_INFO);

	mutex_unlock(&fts_input_dev->mutex);

	return num_read_chars;
}
/************************************************************************
* Name: fts_tpdriver_version_store
* Brief:  no
* Input: device, device attribute, char buf, char count
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_tpdriver_version_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	/* place holder for future use */
	return -EPERM;
}
/************************************************************************
* Name: fts_tprwreg_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_tprwreg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* place holder for future use */
	return -EPERM;
}
/************************************************************************
* Name: fts_tprwreg_store
* Brief:  read/write register
* Input: device, device attribute, char buf, char count
* Output: print register value
* Return: char count
***********************************************************************/
static ssize_t fts_tprwreg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	ssize_t num_read_chars = 0;
	int retval;
	long unsigned int wmreg=0;
	u8 regaddr=0xff,regvalue=0xff;
	u8 valbuf[5]={0};

	memset(valbuf, 0, sizeof(valbuf));
	mutex_lock(&fts_input_dev->mutex);
	num_read_chars = count - 1;
	if (num_read_chars != 2)
	{
		if (num_read_chars != 4)
		{
			dev_err(dev, "please input 2 or 4 character\n");
			goto error_return;
		}
	}
	memcpy(valbuf, buf, num_read_chars);
	retval = kstrtoul(valbuf, 16, &wmreg);
	if (0 != retval)
	{
		dev_err(dev, "%s() - ERROR: Could not convert the given input to a number. The given input was: \"%s\"\n", __FUNCTION__, buf);
		goto error_return;
	}
	if (2 == num_read_chars)
	{
		/*read register*/
		regaddr = wmreg;
		printk("[focal][test](0x%02x)\n", regaddr);
		if (fts_read_reg(client, regaddr, &regvalue) < 0)
			printk("[Focal] %s : Could not read the register(0x%02x)\n", __func__, regaddr);
		else
			printk("[Focal] %s : the register(0x%02x) is 0x%02x\n", __func__, regaddr, regvalue);
	}
	else
	{
		regaddr = wmreg>>8;
		regvalue = wmreg;
		if (fts_write_reg(client, regaddr, regvalue)<0)
			dev_err(dev, "[Focal] %s : Could not write the register(0x%02x)\n", __func__, regaddr);
		else
			dev_dbg(dev, "[Focal] %s : Write 0x%02x into register(0x%02x) successful\n", __func__, regvalue, regaddr);
	}
	error_return:
	mutex_unlock(&fts_input_dev->mutex);

	return count;
}
/************************************************************************
* Name: fts_fwupdate_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_fwupdate_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* place holder for future use */
	return -EPERM;
}

/************************************************************************
* Name: fts_fwupdate_store
* Brief:  upgrade from *.i
* Input: device, device attribute, char buf, char count
* Output: no
* Return: char count
***********************************************************************/
static ssize_t fts_fwupdate_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u8 uc_host_fm_ver;
	int i_ret;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	mutex_lock(&fts_input_dev->mutex);

	disable_irq(client->irq);
	#if GTP_ESD_PROTECT
		apk_debug_flag = 1;
	#endif

	i_ret = fts_ctpm_fw_upgrade_with_i_file(client);
	if (i_ret == 0)
	{
		msleep(300);
		uc_host_fm_ver = fts_ctpm_get_i_file_ver();
		dev_dbg(dev, "%s [FTS] upgrade to new version 0x%x\n", __func__, uc_host_fm_ver);
	}
	else
	{
		dev_err(dev, "%s ERROR:[FTS] upgrade failed ret=%d.\n", __func__, i_ret);
	}

	#if GTP_ESD_PROTECT
		apk_debug_flag = 0;
	#endif
	enable_irq(client->irq);
	mutex_unlock(&fts_input_dev->mutex);

	return count;
}
/************************************************************************
* Name: fts_fwupgradeapp_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_fwupgradeapp_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* place holder for future use */
	return -EPERM;
}

/************************************************************************
* Name: fts_fwupgradeapp_store
* Brief:  upgrade from app.bin
* Input: device, device attribute, char buf, char count
* Output: no
* Return: char count
***********************************************************************/
static ssize_t fts_fwupgradeapp_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	char fwname[128];
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	memset(fwname, 0, sizeof(fwname));
	sprintf(fwname, "%s", buf);
	fwname[count-1] = '\0';

	mutex_lock(&fts_input_dev->mutex);

	disable_irq(client->irq);
	#if GTP_ESD_PROTECT
				apk_debug_flag = 1;
			#endif
	fts_ctpm_fw_upgrade_with_app_file(client, fwname);
	#if GTP_ESD_PROTECT
				apk_debug_flag = 0;
			#endif
	enable_irq(client->irq);

	mutex_unlock(&fts_input_dev->mutex);
	return count;
}
/************************************************************************
* Name: fts_ftsgetprojectcode_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_getprojectcode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return -EPERM;
}
/************************************************************************
* Name: fts_ftsgetprojectcode_store
* Brief:  no
* Input: device, device attribute, char buf, char count
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_getprojectcode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	/* place holder for future use */
	return -EPERM;
}

#define FT5X0X_INI_FILEPATH "/system/etc/"
#define FT5X0X_SAVE_PATH "/data/user/"
static int focal_i2c_Read(unsigned char *writebuf, int writelen, unsigned char *readbuf, int readlen)
{
	int ret;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
				.addr = fts_i2c_client->addr,
				.flags = 0,
				.len = writelen,
				.buf = writebuf,
			},
			{
				.addr = fts_i2c_client->addr,
				.flags = I2C_M_RD,
				.len = readlen,
				.buf = readbuf,
			},
		};
		ret = i2c_transfer(fts_i2c_client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&fts_i2c_client->dev, "f%s: i2c read error.\n", __func__);
	} else {
		struct i2c_msg msgs[] = {
			{
				.addr = fts_i2c_client->addr,
				.flags = I2C_M_RD,
				.len = readlen,
				.buf = readbuf,
			},
		};
		ret = i2c_transfer(fts_i2c_client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&fts_i2c_client->dev, "%s:i2c read error.\n", __func__);
	}
	return ret;
}

static int focal_i2c_Write(unsigned char *writebuf, int writelen)
{
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr = fts_i2c_client->addr,
			.flags = 0,
			.len = writelen,
			.buf = writebuf,
		},
	};

	ret = i2c_transfer(fts_i2c_client->adapter, msg, 1);
	if (ret < 0)
		dev_err(&fts_i2c_client->dev, "%s i2c write error.\n", __func__);

	return ret;
}

static int ft5x0x_GetInISize(char *config_name)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize = 0;
	char filepath[128];
	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "%s%s", FT5X0X_INI_FILEPATH, config_name);
	if (NULL == pfile)
		pfile = filp_open(filepath, O_RDONLY, 0);
	if (IS_ERR(pfile)) {
		pr_err("error occured while opening file %s.\n", filepath);
		return -EIO;
	}
	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	filp_close(pfile, NULL);
	return fsize;
}

static int ft5x0x_ReadInIData(char *config_name, char *config_buf)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize;
	char filepath[128];
	loff_t pos;
	mm_segment_t old_fs;
	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "%s%s", FT5X0X_INI_FILEPATH, config_name);
	if (NULL == pfile)
		pfile = filp_open(filepath, O_RDONLY, 0);
	if (IS_ERR(pfile)) {
		pr_err("error occured while opening file %s.\n", filepath);
		return -EIO;
	}
	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_read(pfile, config_buf, fsize, &pos);
	filp_close(pfile, NULL);
	set_fs(old_fs);
	return 0;
}

static int ft5x0x_get_testparam_from_ini(char *config_name)
{
	char *filedata = NULL;
	int inisize = ft5x0x_GetInISize(config_name);
	pr_info("inisize = %d \n ", inisize);
	if (inisize <= 0) {
		pr_err("%s ERROR:Get firmware size failed\n",__func__);
		return -EIO;
	}
	filedata = kmalloc(inisize + 1, GFP_ATOMIC);
	if (ft5x0x_ReadInIData(config_name, filedata)) {
		pr_err("%s() - ERROR: request_firmware failed\n",__func__);
		kfree(filedata);
		return -EIO;
	} else {
		pr_info("ft5x0x_ReadInIData successful\n");
	}
	set_param_data(filedata);
	return 0;
}

static int ft5x0x_save_test_data(char *file_name, char *data_buf, int iLen)
{
	struct file *pfile = NULL;

	char filepath[128];
	loff_t pos;
	mm_segment_t old_fs;

	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "%s%s", FT5X0X_SAVE_PATH, file_name);
	if (NULL == pfile)
		pfile = filp_open(filepath, O_CREAT|O_RDWR, 0);
	if (IS_ERR(pfile)) {
		pr_err("error occured while opening file %s.\n", filepath);
		return -EIO;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_write(pfile, data_buf, iLen, &pos);
	filp_close(pfile, NULL);
	set_fs(old_fs);

	return 0;
}

static int test_rst;
#define FTS_TEST_NOT_SET_INI_FILE	(0)
#define FTS_TEST_FAILED		(-1)
#define FTS_TEST_SUCCESS		(1)

static ssize_t ft5x0x_ftsmcaptest_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t num_read_chars = 0;

	mutex_lock(&fts_input_dev->mutex);

	if (FTS_TEST_NOT_SET_INI_FILE == test_rst)
		num_read_chars = snprintf(buf, 128,"not set ini file\n");
	else if (FTS_TEST_FAILED == test_rst)
		num_read_chars = snprintf(buf, 128,"failed\n");
	else if (FTS_TEST_SUCCESS == test_rst)
		num_read_chars = snprintf(buf, 128,"success\n");
	else
		num_read_chars = snprintf(buf, 128,"para wrong\n");

	mutex_unlock(&fts_input_dev->mutex);

	return num_read_chars;
}

static ssize_t ft5x0x_ftsmcaptest_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	char cfgname[128];
	int testdatalen = 0;
	char *testdata;

	testdata = kmalloc(1024*80, GFP_ATOMIC);
	if (testdata == NULL) {
		dev_err(dev, "tp test kmalloc err\n");
		test_rst = FTS_TEST_NOT_SET_INI_FILE;
		return -1;
	}
	memset(cfgname, 0, sizeof(cfgname));
	sprintf(cfgname, "%s", buf);
	cfgname[count-1] = '\0';

	mutex_lock(&fts_input_dev->mutex);
	init_i2c_write_func(focal_i2c_Write);
	init_i2c_read_func(focal_i2c_Read);
	if(ft5x0x_get_testparam_from_ini(cfgname) <0) {
		dev_err(dev, "get testparam from ini failure\n");
		test_rst = FTS_TEST_NOT_SET_INI_FILE;
	}
	else {
		if(true == start_test_tp()) {
			dev_info(dev, "tp test pass\n");
			test_rst = FTS_TEST_SUCCESS;
		} else {
			dev_info(dev, "tp test failure\n");
			test_rst = FTS_TEST_FAILED;
		}
		testdatalen = get_test_data(testdata);
		dev_info(dev, "tp test data len = %d\n", testdatalen);
		ft5x0x_save_test_data("fts_testdata.csv", testdata, testdatalen);
		free_test_param_data();
	}
	mutex_unlock(&fts_input_dev->mutex);
	kfree(testdata);

	return count;
}

static ssize_t ft5x0x_ftsdumpinfo_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", fts_wq_data->ts_info);
}

/****************************************/
/* sysfs */
/*get the fw version
*example:cat ftstpfwver
*/
static DEVICE_ATTR(ftstpfwver, S_IRUGO|S_IWUSR, fts_tpfwver_show, fts_tpfwver_store);

static DEVICE_ATTR(ftstpdriverver, S_IRUGO|S_IWUSR, fts_tpdriver_version_show, fts_tpdriver_version_store);
/*upgrade from *.i
*example: echo 1 > ftsfwupdate
*/
static DEVICE_ATTR(ftsfwupdate, S_IRUGO|S_IWUSR, fts_fwupdate_show, fts_fwupdate_store);
/*read and write register
*read example: echo 88 > ftstprwreg ---read register 0x88
*write example:echo 8807 > ftstprwreg ---write 0x07 into register 0x88
*
*note:the number of input must be 2 or 4.if it not enough,please fill in the 0.
*/
static DEVICE_ATTR(ftstprwreg, S_IRUGO|S_IWUSR, fts_tprwreg_show, fts_tprwreg_store);
/*upgrade from app.bin
*example:echo "*_app.bin" > ftsfwupgradeapp
*/
static DEVICE_ATTR(ftsfwupgradeapp, S_IRUGO|S_IWUSR, fts_fwupgradeapp_show, fts_fwupgradeapp_store);
static DEVICE_ATTR(ftsgetprojectcode, S_IRUGO|S_IWUSR, fts_getprojectcode_show, fts_getprojectcode_store);
static DEVICE_ATTR(ftsmcaptest, S_IRUGO|S_IWUSR|S_IWGRP, ft5x0x_ftsmcaptest_show, ft5x0x_ftsmcaptest_store);
#if  LENOVO_DOUBLE_CLICK
static DEVICE_ATTR(wake_gesture, S_IRUGO|S_IWUSR|S_IWGRP, fts_wake_gesture_show, fts_wake_gesture_store);
#endif
static DEVICE_ATTR(ftsdumpinfo, S_IRUGO, ft5x0x_ftsdumpinfo_show, NULL);

/*add your attr in here*/
static struct attribute *fts_attributes[] = {
	&dev_attr_ftstpfwver.attr,
	&dev_attr_ftstpdriverver.attr,
	&dev_attr_ftsfwupdate.attr,
	&dev_attr_ftstprwreg.attr,
	&dev_attr_ftsfwupgradeapp.attr,
	&dev_attr_ftsgetprojectcode.attr,
	&dev_attr_ftsmcaptest.attr,
	&dev_attr_ftsdumpinfo.attr,
#if  LENOVO_DOUBLE_CLICK
	&dev_attr_wake_gesture.attr,
#endif
	NULL
};

static struct attribute_group fts_attribute_group = {
	.attrs = fts_attributes
};

/************************************************************************
* Name: fts_create_sysfs
* Brief:  create sysfs for debug
* Input: i2c info
* Output: no
* Return: success =0
***********************************************************************/
int fts_create_sysfs(struct i2c_client * client)
{
	int err;

	err = sysfs_create_group(&client->dev.kobj, &fts_attribute_group);
	if (0 != err)
	{
		dev_err(&client->dev, "%s() - ERROR: sysfs_create_group() failed.\n", __func__);
		sysfs_remove_group(&client->dev.kobj, &fts_attribute_group);
		return -EIO;
	}
	else
	{
		pr_info("fts:%s() - sysfs_create_group() succeeded.\n",__func__);
	}
	return err;
}
/************************************************************************
* Name: fts_remove_sysfs
* Brief:  remove sys
* Input: i2c info
* Output: no
* Return: no
***********************************************************************/
int fts_remove_sysfs(struct i2c_client * client)
{
	sysfs_remove_group(&client->dev.kobj, &fts_attribute_group);
	return 0;
}
