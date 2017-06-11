/*! \file sx9310.c
 * \brief  SX9310 Driver
 *
 * Driver for the SX9310 
 * Copyright (c) 2011 Semtech Corp
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
//#define DEBUG

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/of_gpio.h>
#include <linux/switch.h>

#include <linux/input/smtc/misc/sx86xx.h> /* main struct, interrupt,init,pointers */
#include <linux/input/smtc/misc/sx9310_i2c_reg.h>
#include <linux/input/smtc/misc/sx9310_platform_data.h>  /* platform data */

#define IDLE 0
#define ACTIVE 1
#define DRIVER_NAME "sx9310"
#define MAX_WRITE_ARRAY_SIZE 32

//static u8 get_default_value_of_reg(psx86XX_t this, u8 reg);
static void set_default_value_of_reg(psx86XX_t this, u8 reg, u8 val);

/*! \struct sx9310
 * Specialized struct containing input event data, platform data, and
 * last cap state read if needed.
 */
typedef struct sx9310
{
	pbuttonInformation_t pbuttonInformation;
	psx9310_platform_data_t hw; /* specific platform data settings */
	struct switch_dev sdev;
	int state;
} sx9310_t, *psx9310_t;

#ifdef CONFIG_OF
psx9310_platform_data_t g_pdata = NULL;
#endif

/*! \fn static int write_register(psx86XX_t this, u8 address, u8 value)
 * \brief Sends a write register to the device
 * \param this Pointer to main parent struct 
 * \param address 8-bit register address
 * \param value   8-bit register value to write to address
 * \return Value from i2c_master_send
 */
static int write_register(psx86XX_t this, u8 address, u8 value)
{
	struct i2c_client *i2c = 0;
	char buffer[2];
	int returnValue = 0;
	buffer[0] = address;
	buffer[1] = value;
	returnValue = -ENOMEM;
	if (this && this->bus) {
		i2c = this->bus;

		returnValue = i2c_master_send(i2c,buffer,2);
		dev_dbg(&i2c->dev,"write_register Address: 0x%x Value: 0x%x Return: %d\n",
				address,value,returnValue);
	}
	return returnValue;
}

/*! \fn static int read_register(psx86XX_t this, u8 address, u8 *value) 
 * \brief Reads a register's value from the device
 * \param this Pointer to main parent struct 
 * \param address 8-Bit address to read from
 * \param value Pointer to 8-bit value to save register value to 
 * \return Value from i2c_smbus_read_byte_data if < 0. else 0
 */
static int read_register(psx86XX_t this, u8 address, u8 *value)
{
	struct i2c_client *i2c = 0;
	s32 returnValue = 0;
	if (this && value && this->bus) {
		i2c = this->bus;
		returnValue = i2c_smbus_read_byte_data(i2c,address);
		dev_dbg(&i2c->dev, "read_register Address: 0x%x Return: 0x%x\n",address,returnValue);
		if (returnValue >= 0) {
			*value = returnValue;
			return 0;
		} else {
			return returnValue;
		}
	}
	return -ENOMEM;
}

#if 0
/*! \brief Sends a write register range to the device
 * \param this Pointer to main parent struct 
 * \param reg 8-bit register address (base address)
 * \param data pointer to 8-bit register values
 * \param size size of the data pointer
 * \return Value from i2c_master_send
 */
static int write_registerEx(psx86XX_t this, unsigned char reg,
		unsigned char *data, int size)
{
	struct i2c_client *i2c = 0;
	u8 tx[MAX_WRITE_ARRAY_SIZE];
	int ret = 0;

	if (this && (i2c = this->bus) && data && (size <= MAX_WRITE_ARRAY_SIZE))
	{
		dev_dbg(this->pdev, "inside write_registerEx()\n");
		tx[0] = reg;
		dev_dbg(this->pdev, "going to call i2c_master_send(0x%p, 0x%x ",
				(void *)i2c,tx[0]);
		for (ret = 0; ret < size; ret++)
		{
			tx[ret+1] = data[ret];
			dev_dbg(this->pdev, "0x%x, ",tx[ret+1]);
		}
		dev_dbg(this->pdev, "\n");

		ret = i2c_master_send(i2c, tx, size+1 );
		if (ret < 0)
			dev_err(this->pdev, "I2C write error\n");
	}
	dev_dbg(this->pdev, "leaving write_registerEx()\n");


	return ret;
}

/*! \brief Reads a group of registers from the device
 * \param this Pointer to main parent struct 
 * \param reg 8-Bit address to read from (base address)
 * \param data Pointer to 8-bit value array to save registers to 
 * \param size size of array
 * \return Value from i2c_smbus_read_byte_data if < 0. else 0
 */
static int read_registerEx(psx86XX_t this, unsigned char reg,
		unsigned char *data, int size)
{
	struct i2c_client *i2c = 0;
	int ret = 0;
	u8 tx[] = {
		reg
	};
	if (this && (i2c = this->bus) && data && (size <= MAX_WRITE_ARRAY_SIZE))
	{
		dev_dbg(this->pdev, "inside read_registerEx()\n");
		dev_dbg(this->pdev,
				"going to call i2c_master_send(0x%p,0x%p,1) Reg: 0x%x\n",
				(void *)i2c,(void *)tx,tx[0]);
		ret = i2c_master_send(i2c,tx,1);
		if (ret >= 0) {
			dev_dbg(this->pdev, "going to call i2c_master_recv(0x%p,0x%p,%x)\n",
					(void *)i2c,(void *)data,size);
			ret = i2c_master_recv(i2c, data, size);
		}
	}
	if (unlikely(ret < 0))
		dev_err(this->pdev, "I2C read error\n");
	dev_dbg(this->pdev, "leaving read_registerEx()\n");
	return ret;
}
#endif

/*********************************************************************/
/*! \brief Perform a manual offset calibration
 * \param this Pointer to main parent struct 
 * \return Value return value from the write register
 */
static int manual_offset_calibration(psx86XX_t this)
{
	s32 returnValue = 0;
	dev_info( this->pdev, "Performing manual_offset_calibration()\n");
	returnValue = write_register(this,SX9310_IRQSTAT_REG,0xFF);
	return returnValue;
}
/*! \brief sysfs show function for manual calibration which currently just
 * returns register value.
 */
static ssize_t manual_offset_calibration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u8 reg_value = 0;
	psx86XX_t this = dev_get_drvdata(dev);

	dev_dbg(this->pdev, "Reading IRQSTAT_REG\n");
	read_register(this,SX9310_IRQSTAT_REG,&reg_value);
	return sprintf(buf, "%d\n", reg_value);
}

/*! \brief sysfs store function for manual calibration
 */
static ssize_t manual_offset_calibration_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	psx86XX_t this = dev_get_drvdata(dev);
	unsigned long val;
	if (kstrtoul(buf, 10, &val))
		return -EINVAL;
	if (val) {
		dev_info( this->pdev, "Performing manual_offset_calibration()\n");
		manual_offset_calibration(this);
	}
	return count;
}

static ssize_t reg_dump_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u8 reg_value = 0;
	psx86XX_t this = dev_get_drvdata(dev);
	char * p = buf;

	if (this->read_flag) {
		this->read_flag = 0;
		read_register(this, this->read_reg, &reg_value);
		p += sprintf(p, "%02x\n", reg_value);
		return (p-buf);
	}

	read_register(this, SX9310_IRQ_ENABLE_REG, &reg_value);
	p += sprintf(p, "ENABLE(0x%02x)=0x%02x\n", SX9310_IRQ_ENABLE_REG, reg_value);

	read_register(this, SX9310_IRQFUNC_REG, &reg_value);
	p += sprintf(p, "IRQFUNC(0x%02x)=0x%02x\n", SX9310_IRQFUNC_REG, reg_value);

	read_register(this, SX9310_CPS_CTRL0_REG, &reg_value);
	p += sprintf(p, "CTRL0(0x%02x)=0x%02x\n", SX9310_CPS_CTRL0_REG, reg_value);

	read_register(this, SX9310_CPS_CTRL1_REG, &reg_value);
	p += sprintf(p, "CTRL1(0x%02x)=0x%02x\n", SX9310_CPS_CTRL1_REG, reg_value);

	read_register(this, SX9310_CPS_CTRL2_REG, &reg_value);
	p += sprintf(p, "CTRL2(0x%02x)=0x%02x\n", SX9310_CPS_CTRL2_REG, reg_value);

	read_register(this, SX9310_CPS_CTRL3_REG, &reg_value);
	p += sprintf(p, "CTRL3(0x%02x)=0x%02x\n", SX9310_CPS_CTRL3_REG, reg_value);

	read_register(this, SX9310_CPS_CTRL4_REG, &reg_value);
	p += sprintf(p, "CTRL4(0x%02x)=0x%02x\n", SX9310_CPS_CTRL4_REG, reg_value);

	read_register(this, SX9310_CPS_CTRL5_REG, &reg_value);
	p += sprintf(p, "CTRL5(0x%02x)=0x%02x\n", SX9310_CPS_CTRL5_REG, reg_value);

	read_register(this, SX9310_CPS_CTRL6_REG, &reg_value);
	p += sprintf(p, "CTRL6(0x%02x)=0x%02x\n", SX9310_CPS_CTRL6_REG, reg_value);

	read_register(this, SX9310_CPS_CTRL7_REG, &reg_value);
	p += sprintf(p, "CTRL7(0x%02x)=0x%02x\n", SX9310_CPS_CTRL7_REG, reg_value);

	read_register(this, SX9310_CPS_CTRL8_REG, &reg_value);
	p += sprintf(p, "CTRL8(0x%02x)=0x%02x\n", SX9310_CPS_CTRL8_REG, reg_value);

	read_register(this, SX9310_CPS_CTRL9_REG, &reg_value);
	p += sprintf(p, "CTRL9(0x%02x)=0x%02x\n", SX9310_CPS_CTRL9_REG, reg_value);

	read_register(this, SX9310_CPS_CTRL10_REG, &reg_value);
	p += sprintf(p, "CTRL10(0x%02x)=0x%02x\n", SX9310_CPS_CTRL10_REG, reg_value);

	read_register(this, SX9310_CPS_CTRL11_REG, &reg_value);
	p += sprintf(p, "CTRL11(0x%02x)=0x%02x\n", SX9310_CPS_CTRL11_REG, reg_value);

	read_register(this, SX9310_CPS_CTRL12_REG, &reg_value);
	p += sprintf(p, "CTRL12(0x%02x)=0x%02x\n", SX9310_CPS_CTRL12_REG, reg_value);

	read_register(this, SX9310_CPS_CTRL13_REG, &reg_value);
	p += sprintf(p, "CTRL13(0x%02x)=0x%02x\n", SX9310_CPS_CTRL13_REG, reg_value);

	read_register(this, SX9310_CPS_CTRL14_REG, &reg_value);
	p += sprintf(p, "CTRL14(0x%02x)=0x%02x\n", SX9310_CPS_CTRL14_REG, reg_value);

	read_register(this, SX9310_CPS_CTRL15_REG, &reg_value);
	p += sprintf(p, "CTRL15(0x%02x)=0x%02x\n", SX9310_CPS_CTRL15_REG, reg_value);

	read_register(this, SX9310_CPS_CTRL16_REG, &reg_value);
	p += sprintf(p, "CTRL16(0x%02x)=0x%02x\n", SX9310_CPS_CTRL16_REG, reg_value);

	read_register(this, SX9310_CPS_CTRL17_REG, &reg_value);
	p += sprintf(p, "CTRL17(0x%02x)=0x%02x\n", SX9310_CPS_CTRL17_REG, reg_value);

	read_register(this, SX9310_CPS_CTRL18_REG, &reg_value);
	p += sprintf(p, "CTRL18(0x%02x)=0x%02x\n", SX9310_CPS_CTRL18_REG, reg_value);

	read_register(this, SX9310_CPS_CTRL19_REG, &reg_value);
	p += sprintf(p, "CTRL19(0x%02x)=0x%02x\n", SX9310_CPS_CTRL19_REG, reg_value);

	read_register(this, SX9310_SAR_CTRL0_REG, &reg_value);
	p += sprintf(p, "SCTRL0(0x%02x)=0x%02x\n", SX9310_SAR_CTRL0_REG, reg_value);

	read_register(this, SX9310_SAR_CTRL1_REG, &reg_value);
	p += sprintf(p, "SCTRL1(0x%02x)=0x%02x\n", SX9310_SAR_CTRL1_REG, reg_value);

	read_register(this, SX9310_SAR_CTRL2_REG, &reg_value);
	p += sprintf(p, "SCTRL2(0x%02x)=0x%02x\n", SX9310_SAR_CTRL2_REG, reg_value);

	reg_value = gpio_get_value(this->nirq_gpio);
	p += sprintf(p, "NIRQ=%d\n", reg_value);

	return (p-buf);
}

static ssize_t reg_dump_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	psx86XX_t this = dev_get_drvdata(dev);
	unsigned int val,reg,opt;

	if (sscanf(buf, "%x,%x,%x", &reg, &val, &opt) == 3) {
		this->read_reg = *((u8*)&reg);
		this->read_flag = 1;
	} else if (sscanf(buf, "%x,%x", &reg, &val) == 2) {
		dev_info( this->pdev, "%s,reg = 0x%02x, val = 0x%02x\n", __FUNCTION__, *(u8*)&reg, *(u8*)&val);
		write_register(this, *((u8*)&reg), *((u8*)&val));
		set_default_value_of_reg(this, *((u8*)&reg), *((u8*)&val));
		if (reg == 0x10 && (val&0xf) == 0x00) {
			msleep(50);
			switch_set_state(&(((psx9310_t)(this->pDevice))->sdev), 0);
		}
	}

	return count;
}

static ssize_t touch_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u8 reg_value = 0;
	psx86XX_t this = dev_get_drvdata(dev);

	read_register(this, SX9310_STAT0_REG, &reg_value);
	dev_dbg(this->pdev, "Reading SX9310_STAT0_REG = %d\n", reg_value);

	reg_value &= 0x0f;

	return sprintf(buf, "0x%02x\n", reg_value);
}

static DEVICE_ATTR(calibrate, 0660, manual_offset_calibration_show,
		manual_offset_calibration_store);
static DEVICE_ATTR(reg, 0660, reg_dump_show, reg_dump_store);
static DEVICE_ATTR(status, 0440, touch_status_show, NULL);
static struct attribute *sx9310_attributes[] = {
	&dev_attr_calibrate.attr,
	&dev_attr_reg.attr,
	&dev_attr_status.attr,
	NULL,
};
static struct attribute_group sx9310_attr_group = {
	.attrs = sx9310_attributes,
};
/*********************************************************************/





/*! \fn static int read_regStat(psx86XX_t this)
 * \brief Shortcut to read what caused interrupt.
 * \details This is to keep the drivers a unified
 * function that will read whatever register(s) 
 * provide information on why the interrupt was caused.
 * \param this Pointer to main parent struct 
 * \return If successful, Value of bit(s) that cause interrupt, else 0
 */
static int read_regStat(psx86XX_t this)
{
	u8 data = 0;
	if (this) {
		if (read_register(this,SX9310_IRQSTAT_REG,&data) == 0)
			return (data & 0x00FF);
	}
	return 0;
}

static void read_rawData(psx86XX_t this)
{
	u8 msb=0, lsb=0;
	if(this){
		write_register(this,SX9310_CPSRD,1);//here to check the CS1, also can read other channel
		msleep(100);
		read_register(this,SX9310_USEMSB,&msb);
		read_register(this,SX9310_USELSB,&lsb);
		dev_info(this->pdev, "sx9310 cs1 raw data USEFUL msb = 0x%x, lsb = 0x%x\n",msb,lsb);
		read_register(this,SX9310_AVGMSB,&msb);
		read_register(this,SX9310_AVGLSB,&lsb);
		dev_info(this->pdev, "sx9310 cs1 raw data AVERAGE msb = 0x%x, lsb = 0x%x\n",msb,lsb);
		read_register(this,SX9310_DIFFMSB,&msb);
		read_register(this,SX9310_DIFFLSB,&lsb);
		dev_info(this->pdev, "sx9310 cs1 raw data DIFF msb = 0x%x, lsb = 0x%x\n",msb,lsb);
		read_register(this,SX9310_OFFSETMSB,&msb);
		read_register(this,SX9310_OFFSETLSB,&lsb);
		dev_info(this->pdev, "sx9310 cs1 raw data OFFSET msb = 0x%x, lsb = 0x%x\n",msb,lsb);

		write_register(this,SX9310_CPSRD,2);//here to check the CS1, also can read other channel
		msleep(100);
		read_register(this,SX9310_USEMSB,&msb);
		read_register(this,SX9310_USELSB,&lsb);
		dev_info(this->pdev, "sx9310 cs2 raw data USEFUL msb = 0x%x, lsb = 0x%x\n",msb,lsb);
		read_register(this,SX9310_AVGMSB,&msb);
		read_register(this,SX9310_AVGLSB,&lsb);
		dev_info(this->pdev, "sx9310 cs2 raw data AVERAGE msb = 0x%x, lsb = 0x%x\n",msb,lsb);
		read_register(this,SX9310_DIFFMSB,&msb);
		read_register(this,SX9310_DIFFLSB,&lsb);
		dev_info(this->pdev, "sx9310 cs2 raw data DIFF msb = 0x%x, lsb = 0x%x\n",msb,lsb);
		read_register(this,SX9310_OFFSETMSB,&msb);
		read_register(this,SX9310_OFFSETLSB,&lsb);
		dev_info(this->pdev, "sx9310 cs2 raw data OFFSET msb = 0x%x, lsb = 0x%x\n",msb,lsb);
	}
}

/*! \brief  Initialize I2C config from platform data
 * \param this Pointer to main parent struct 
 */
static void hw_init(psx86XX_t this)
{
	psx9310_t pDevice = 0;
	psx9310_platform_data_t pdata = 0;
	int i = 0;
	/* configure device */
	dev_dbg(this->pdev, "Going to Setup I2C Registers\n");
	if (this && (pDevice = this->pDevice) && (pdata = pDevice->hw))
	{
		while ( i < pdata->i2c_reg_num) {
			/* Write all registers/values contained in i2c_reg */
			dev_dbg(this->pdev, "Going to Write Reg: 0x%x Value: 0x%x\n",
					pdata->pi2c_reg[i].reg,pdata->pi2c_reg[i].val);
			//      msleep(3);        
			write_register(this, pdata->pi2c_reg[i].reg,pdata->pi2c_reg[i].val);
			i++;
		}
	} else {
		dev_err(this->pdev, "ERROR! platform data 0x%p\n",pDevice->hw);
	}
}
/*********************************************************************/




/*! \fn static int initialize(psx86XX_t this)
 * \brief Performs all initialization needed to configure the device
 * \param this Pointer to main parent struct 
 * \return Last used command's return value (negative if error)
 */
static int initialize(psx86XX_t this)
{
	if (this) {
		/* prepare reset by disabling any irq handling */
		this->irq_disabled = 1;
		disable_irq(this->irq);
		/* perform a reset */
		write_register(this,SX9310_SOFTRESET_REG,SX9310_SOFTRESET);
		/* wait until the reset has finished by monitoring NIRQ */
		dev_dbg(this->pdev, "Sent Software Reset. Waiting until device is back from reset to continue.\n");
		/* just sleep for awhile instead of using a loop with reading irq status */
		msleep(300);
		//    while(this->get_nirq_low && this->get_nirq_low()) { read_regStat(this); }
		dev_dbg(this->pdev, "Device is back from the reset, continuing. NIRQ = %d\n",this->get_nirq_low());
		hw_init(this);
		msleep(100); /* make sure everything is running */
		manual_offset_calibration(this);

		/* re-enable interrupt handling */
		enable_irq(this->irq);
		this->irq_disabled = 0;

		/* make sure no interrupts are pending since enabling irq will only
		 * work on next falling edge */
		read_regStat(this);
		dev_dbg(this->pdev, "Exiting initialize(). NIRQ = %d\n",this->get_nirq_low());
		return 0;
	}
	return -ENOMEM;
}

#if 0
static u8 get_default_value_of_reg(psx86XX_t this, u8 reg)
{
	psx9310_t pDevice = 0;
	psx9310_platform_data_t pdata = 0;
	int i = 0;
	u8 val = 0;

	if (this && (pDevice = this->pDevice) && (pdata = pDevice->hw)) {
		while ( i < pdata->i2c_reg_num) {
			if (reg == pdata->pi2c_reg[i].reg)
				return pdata->pi2c_reg[i].val;
			write_register(this, pdata->pi2c_reg[i].reg,pdata->pi2c_reg[i].val);
			i++;
		}
	} else {
		dev_err(this->pdev, "ERROR! platform data 0x%p\n",pDevice->hw);
	}

	read_register(this, reg, &val);
	return val;
}
#endif

static void set_default_value_of_reg(psx86XX_t this, u8 reg, u8 val)
{
	psx9310_t pDevice = 0;
	psx9310_platform_data_t pdata = 0;
	int i = 0;

	if (this && (pDevice = this->pDevice) && (pdata = pDevice->hw)) {
		while ( i < pdata->i2c_reg_num) {
			if (reg == pdata->pi2c_reg[i].reg) {
				pdata->pi2c_reg[i].val = val;
				break;
			}
			i++;
		}
	} else {
		dev_err(this->pdev, "ERROR! platform data 0x%p\n",pDevice->hw);
	}
}

static ssize_t switch_cap_print_state(struct switch_dev *sdev, char *buf)
{
	const char *state;

	if (switch_get_state(sdev))
		state = "1";
	else
		state = "0";

	if (state)
		return sprintf(buf, "%s\n", state);

	return 0;
}


/*! 
 * \brief Handle what to do when a touch occurs
 * \param this Pointer to main parent struct 
 */
static void touchProcess(psx86XX_t this)
{
	int counter = 0;
	u8 i = 0;
	u8 stat1 = 0;
	int numberOfButtons = 0;
	psx9310_t pDevice = NULL;
	struct _buttonInfo *buttons = NULL;
	struct input_dev *input = NULL;
	int state = IDLE;

	struct _buttonInfo *pCurrentButton  = NULL;


	if (this && (pDevice = this->pDevice))
	{
		dev_dbg(this->pdev, "Inside touchProcess()\n");
		read_register(this, SX9310_STAT0_REG, &i);
		read_register(this, SX9310_STAT1_REG, &stat1);
		dev_info(this->pdev, "Read Reg[0x%02x] = 0x%02x\n", SX9310_STAT0_REG, i);
		dev_info(this->pdev, "Read Reg[0x%02x] = 0x%02x\n", SX9310_STAT1_REG, stat1);

		buttons = pDevice->pbuttonInformation->buttons;
		input = pDevice->pbuttonInformation->input;
		numberOfButtons = pDevice->pbuttonInformation->buttonSize;

		if (unlikely( (buttons==NULL) || (input==NULL) )) {
			dev_err(this->pdev, "ERROR!! buttons or input NULL!!!\n");
			return;
		}

		for (counter = 0; counter < numberOfButtons; counter++) {
			pCurrentButton = &buttons[counter];
			if (pCurrentButton==NULL) {
				dev_err(this->pdev,"ERROR!! current button at index: %d NULL!!!\n",
						counter);
				return; // ERRORR!!!!
			}
			switch (pCurrentButton->state) {
				case IDLE: /* Button is not being touched! */
					if (((i & pCurrentButton->mask) == pCurrentButton->mask)) {
						/* User pressed button */
						dev_info(this->pdev, "cap button %d touched\n", counter);
						//input_report_key(input, pCurrentButton->keycode, 1);
						pCurrentButton->state = ACTIVE;
						state = ACTIVE;
					} else {
						dev_dbg(this->pdev, "Button %d already released.\n",counter);
					}
					break;
				case ACTIVE: /* Button is being touched! */ 
					if (((i & pCurrentButton->mask) != pCurrentButton->mask)) {
						/* User released button */
						dev_info(this->pdev, "cap button %d released\n",counter);
						//input_report_key(input, pCurrentButton->keycode, 0);
						pCurrentButton->state = IDLE;
					} else {
						dev_dbg(this->pdev, "Button %d still touched.\n",counter);
					}
					break;
				default: /* Shouldn't be here, device only allowed ACTIVE or IDLE */
					break;
			};
		}
		//input_sync(input);
		if(pDevice->state != state) {
			switch_set_state(&pDevice->sdev, state);
			pDevice->state = state;
		}

		dev_dbg(this->pdev, "Leaving touchProcess()\n");
	}
}

static int sx9310_check_id(psx86XX_t this)
{
	u8 id;
	if(read_register(this, SX9310_WHOAMI, &id)) {
		dev_err( this->pdev, "sx9310 check id fail\n");
		return -1;
	}
	if(id !=0x01) {
		dev_err( this->pdev, "sx9310 id not match:0x%02x\n", id);
		return -2;
	}

	dev_info( this->pdev, "sx9310 check id ok\n");
	return 0;
}

#ifdef CONFIG_OF
static int sx9310_pinctrl_init(struct i2c_client *client, psx9310_platform_data_t pdata)
{
	int retval;

	/* Get pinctrl if target uses pinctrl */
	pdata->cap_pinctrl = devm_pinctrl_get(&(client->dev));
	if (IS_ERR_OR_NULL(pdata->cap_pinctrl)) {
		retval = PTR_ERR(pdata->cap_pinctrl);
		dev_dbg(&client->dev,
				"Target does not use pinctrl %d\n", retval);
		goto err_pinctrl_get;
	}

	pdata->pinctrl_state_active =
		pinctrl_lookup_state(pdata->cap_pinctrl, "cap_active");
	if (IS_ERR_OR_NULL(pdata->pinctrl_state_active)) {
		retval = PTR_ERR(pdata->pinctrl_state_active);
		dev_err(&client->dev,
				"Can not lookup %s pinstate %d\n",
				"cap_active", retval);
		goto err_pinctrl_lookup;
	}

	pdata->pinctrl_state_suspend =
		pinctrl_lookup_state(pdata->cap_pinctrl, "cap_suspend");
	if (IS_ERR_OR_NULL(pdata->pinctrl_state_suspend)) {
		retval = PTR_ERR(pdata->pinctrl_state_suspend);
		dev_err(&client->dev,
				"Can not lookup %s pinstate %d\n",
				"cap_suspend", retval);
		goto err_pinctrl_lookup;
	}

	return 0;

err_pinctrl_lookup:
	devm_pinctrl_put(pdata->cap_pinctrl);
err_pinctrl_get:
	pdata->cap_pinctrl = NULL;
	return retval;
}

#if 0
static int sx9310_load_init_reg_table(struct device *dev, psx9310_platform_data_t pdata))
{
	struct addr_set *qdss_addr_set;
	struct platform_device *pdev = res->pdev;
	struct device_node *np = dev->of_node;
	int i, count;
	int rc = 0;

	if (!of_find_property(np, "semtech,init-reg", NULL)) {
		/* qcom,qdss-presets is an optional property. It likely won't be
		 * present if we don't have any register settings to program */
		dprintk(VIDC_DBG, "qcom,qdss-presets not found\n");
		return rc;
	}

	qdss_addr_set = &res->qdss_addr_set;
	count = get_u32_array_num_elements(np, "semtech,init-reg");
	count /= sizeof(*qdss_addr_set->addr_tbl) / sizeof(u32);

	if (!qdss_addr_set->count) {
		dprintk(VIDC_DBG, "no elements in qdss reg set\n");
		return rc;
	}

	qdss_addr_set->addr_tbl = devm_kzalloc(&pdev->dev,
			qdss_addr_set->count * sizeof(*qdss_addr_set->addr_tbl),
			GFP_KERNEL);
	if (!qdss_addr_set->addr_tbl) {
		dprintk(VIDC_ERR, "%s Failed to alloc register table\n",
				__func__);
		rc = -ENOMEM;
		goto err_qdss_addr_tbl;
	}

	rc = of_property_read_u32_array(np, "semtech,init-reg",
			(u32 *)qdss_addr_set->addr_tbl, qdss_addr_set->count * 2);
	if (rc) {
		dprintk(VIDC_ERR, "Failed to read qdss address table\n");
		msm_vidc_free_qdss_addr_table(res);
		rc = -EINVAL;
		goto err_qdss_addr_tbl;
	}

	for (i = 0; i < qdss_addr_set->count; i++) {
		dprintk(VIDC_DBG, "qdss addr = %x, value = %x\n",
				qdss_addr_set->addr_tbl[i].start,
				qdss_addr_set->addr_tbl[i].size);
	}
err_qdss_addr_tbl:
	return rc;
}
#endif

static int sx9310_parse_dt(struct device *dev, psx9310_platform_data_t pdata)
{
	struct device_node *np = dev->of_node;

	pdata->irq_gpio = of_get_named_gpio_flags(np, "semtech,irq-gpio", 0, &pdata->irq_gpio_flags);
	if (pdata->irq_gpio < 0)
		dev_err(dev, "Unable to get irq_gpio\n");
	dev_dbg(dev, "irq_gpio:%d\n", pdata->irq_gpio);
	return 0;
}

static int sx9310_get_nirq_state(void)
{
	return g_pdata?(!gpio_get_value(g_pdata->irq_gpio)):-1;
}

static struct _buttonInfo psmtcButtons[] = {
	{
		.keycode = KEY_0,
		.mask = SX9310_TCHCMPSTAT_TCHSTAT0_FLAG,
	},
	{
		.keycode = KEY_1,
		.mask = SX9310_TCHCMPSTAT_TCHSTAT1_FLAG,
	},
	{
		.keycode = KEY_2,
		.mask = SX9310_TCHCMPSTAT_TCHSTAT2_FLAG,
	},
	{
		.keycode = KEY_3,//KEY_COMB,
		.mask = SX9310_TCHCMPSTAT_TCHSTAT3_FLAG,
	},
};

static struct _totalButtonInformation smtcButtonInformation = {
	.buttons = psmtcButtons,
	.buttonSize = ARRAY_SIZE(psmtcButtons),
};

static struct smtc_reg_data sx9310_i2c_reg_setup[] = {
	{
		.reg = SX9310_IRQ_ENABLE_REG,
		.val = 0x70,
	},
	{
		.reg = SX9310_IRQFUNC_REG,
		.val = 0x00,
	},
	{
		.reg = SX9310_CPS_CTRL1_REG,
		.val = 0x00,
	},
	{
		.reg = SX9310_CPS_CTRL2_REG,
		.val = 0x04,
	},
	{
		.reg = SX9310_CPS_CTRL3_REG,
		.val = 0x0c,
	},
	{
		.reg = SX9310_CPS_CTRL4_REG,
		.val = 0x0D,
	},
	{
		.reg = SX9310_CPS_CTRL5_REG,
		.val = 0x81,
	},
	{
		.reg = SX9310_CPS_CTRL6_REG,
		.val = 0x20,
	},
	{
		.reg = SX9310_CPS_CTRL7_REG,
		.val = 0x4C,
	},
	{
		.reg = SX9310_CPS_CTRL8_REG,
		.val = 0x9e,
	},
	{
		.reg = SX9310_CPS_CTRL9_REG,
		.val = 0x7D,
	},
	{
		.reg = SX9310_CPS_CTRL10_REG,
		.val = 0x1a,
	},
	/*
	   {
	   .reg = SX9310_CPS_CTRL11_REG,
	   .val = 0x00,
	   },
	   {
	   .reg = SX9310_CPS_CTRL12_REG,
	   .val = 0x00,
	   },
	   {
	   .reg = SX9310_CPS_CTRL13_REG,
	   .val = 0x00,
	   },
	   {
	   .reg = SX9310_CPS_CTRL14_REG,
	   .val = 0x00,
	   },
	   {
	   .reg = SX9310_CPS_CTRL15_REG,
	   .val = 0x00,
	   },
	   {
	   .reg = SX9310_CPS_CTRL16_REG,
	   .val = 0x00,
	   },
	   {
	   .reg = SX9310_CPS_CTRL17_REG,
	   .val = 0x00,
	   },
	   {
	   .reg = SX9310_CPS_CTRL18_REG,
	   .val = 0x00,
	   },
	   {
	   .reg = SX9310_CPS_CTRL19_REG,
	   .val = 0x00,
	   },*/
	{
		.reg = SX9310_CPS_CTRL0_REG,
			.val = 0x51,
	},
};

#else
static int sx9310_parse_dt(struct device *dev, psx9310_platform_data_t pdata)
{
	return -ENODEV;
}
#endif

/*! \fn static int sx9310_probe(struct i2c_client *client, const struct i2c_device_id *id)
 * \brief Probe function
 * \param client pointer to i2c_client
 * \param id pointer to i2c_device_id
 * \return Whether probe was successful
 */
static int sx9310_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int i = 0,err = 0;
	psx86XX_t this = 0;
	psx9310_t pDevice = 0;
	psx9310_platform_data_t pplatData = 0;
	struct input_dev *input = NULL;

	dev_info(&client->dev, "sx9310_probe()\n");

#ifdef CONFIG_OF
	if (client->dev.of_node) {
		pplatData = devm_kzalloc(&client->dev,
				sizeof(sx9310_platform_data_t), GFP_KERNEL);
		if (!pplatData) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		err = sx9310_parse_dt(&client->dev, pplatData);
		if (err) {
			dev_err(&client->dev, "DT parsing failed\n");
			return -EINVAL;
		}

		err = gpio_request(pplatData->irq_gpio, "cap irq");
		if (err) {
			dev_err(&client->dev, "Request GPIO Fail %d\n", err);
		}

		pplatData->pbuttonInformation = &smtcButtonInformation;
		pplatData->get_is_nirq_low = sx9310_get_nirq_state;
		pplatData->pi2c_reg = sx9310_i2c_reg_setup;
		pplatData->i2c_reg_num = ARRAY_SIZE(sx9310_i2c_reg_setup);
		g_pdata = pplatData;

		sx9310_pinctrl_init(client, pplatData);
		if (pplatData->cap_pinctrl) {
			err = pinctrl_select_state(pplatData->cap_pinctrl, pplatData->pinctrl_state_active);
			if (err < 0)
				dev_err(&client->dev, "Cannot get active pinctrl state\n");
		}
	} else
#else
		pplatData = client->dev.platform_data;
#endif

	if (!pplatData) {
		dev_err(&client->dev, "platform data is required!\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_READ_WORD_DATA))
		return -EIO;

	this = kzalloc(sizeof(sx86XX_t), GFP_KERNEL); /* create memory for main struct */
	dev_dbg(&client->dev, "\t Initialized Main Memory: 0x%p\n",this);

	if (this)
	{
		/* In case we need to reinitialize data 
		 * (e.q. if suspend reset device) */
		this->init = initialize;
		/* shortcut to read status of interrupt */
		this->refreshStatus = read_regStat;
		/* pointer to function from platform data to get pendown 
		 * (1->NIRQ=0, 0->NIRQ=1) */
		this->get_nirq_low = pplatData->get_is_nirq_low;
		this->nirq_gpio = pplatData->irq_gpio;
		/* save irq in case we need to reference it */
		this->irq = client->irq;
		/* do we need to create an irq timer after interrupt ? */
		this->useIrqTimer = 0;

		/* Setup function to call on corresponding reg irq source bit */
		if (MAX_NUM_STATUS_BITS>= 8)
		{
			this->statusFunc[0] = 0; /* TXEN_STAT */
			this->statusFunc[1] = 0; /* UNUSED */
			this->statusFunc[2] = 0; /* UNUSED */
			this->statusFunc[3] = read_rawData; /* CONV_STAT */
			this->statusFunc[4] = 0; /* COMP_STAT */
			this->statusFunc[5] = touchProcess; /* RELEASE_STAT */
			this->statusFunc[6] = touchProcess; /* TOUCH_STAT  */
			this->statusFunc[7] = 0; /* RESET_STAT */
		}

		/* setup i2c communication */
		this->bus = client;
		i2c_set_clientdata(client, this);

		/* record device struct */
		this->pdev = &client->dev;

		if(sx9310_check_id(this)) {
			dev_err(this->pdev, "sx9310 probe failed\n");
			kfree(this);
			return -EINVAL;
		}

		/* create memory for device specific struct */
		this->pDevice = pDevice = kzalloc(sizeof(sx9310_t), GFP_KERNEL);
		dev_dbg(&client->dev, "\t Initialized Device Specific Memory: 0x%p\n",pDevice);

		if (pDevice)
		{
			/* for accessing items in user data (e.g. calibrate) */
			if (sysfs_create_group(&client->dev.kobj, &sx9310_attr_group))
				dev_err(&client->dev, "\t create attr for sx9310 failed\n");

			/* Check if we hava a platform initialization function to call*/
			if (pplatData->init_platform_hw)
				pplatData->init_platform_hw();

			/* Add Pointer to main platform data struct */
			pDevice->hw = pplatData;

			/* Initialize the button information initialized with keycodes */
			pDevice->pbuttonInformation = pplatData->pbuttonInformation;

			/* Create the input device */
			input = input_allocate_device();
			if (!input) {
				return -ENOMEM;
			}

			/* Set all the keycodes */
			__set_bit(EV_KEY, input->evbit);
			for (i = 0; i < pDevice->pbuttonInformation->buttonSize; i++) {
				__set_bit(pDevice->pbuttonInformation->buttons[i].keycode, 
						input->keybit);
				pDevice->pbuttonInformation->buttons[i].state = IDLE;
			}
			/* save the input pointer and finish initialization */
			pDevice->pbuttonInformation->input = input;
			input->name = "SX9310 Cap Touch";
			input->id.bustype = BUS_I2C;
			//      input->id.product = sx863x->product;
			//      input->id.version = sx863x->version;
			if(input_register_device(input))
				return -ENOMEM;

			//pDevice->pbuttonInformation->sdev = kzalloc(sizeof(struct hall_switch_data), GFP_KERNEL);
			//if (!pDevice->pbuttonInformation->sdev)
			//	return -ENOMEM;

			pDevice->sdev.name = "capsensor";
			pDevice->sdev.print_state = switch_cap_print_state;

			if(switch_dev_register(&pDevice->sdev))
				return -ENOMEM;
			pDevice->state = IDLE;
		}
		sx86XX_init(this);
		this->read_flag = 0;
		this->read_reg = SX9310_CPS_CTRL0_REG;

		return  0;
	}
	return -1;
}

/*! \fn static int sx9310_remove(struct i2c_client *client)
 * \brief Called when device is to be removed
 * \param client Pointer to i2c_client struct
 * \return Value from sx86XX_remove()
 */
static int sx9310_remove(struct i2c_client *client)
{
	psx9310_platform_data_t pplatData =0;
	psx9310_t pDevice = 0;
	psx86XX_t this = i2c_get_clientdata(client);
	if (this && (pDevice = this->pDevice))
	{
		input_unregister_device(pDevice->pbuttonInformation->input);

		sysfs_remove_group(&client->dev.kobj, &sx9310_attr_group);
		pplatData = client->dev.platform_data;
		if (pplatData && pplatData->exit_platform_hw)
			pplatData->exit_platform_hw();
		kfree(this->pDevice);
	}
	return sx86XX_remove(this);
}

/*====================================================*/
#if defined(USE_KERNEL_SUSPEND)
/***** Kernel Suspend *****/
static int sx9310_suspend(struct i2c_client *client)
{
	psx86XX_t this = i2c_get_clientdata(client);
	sx86XX_suspend(this);
	read_regStat(this);
	return 0;
}
/***** Kernel Resume *****/
static int sx9310_resume(struct i2c_client *client)
{
	psx86XX_t this = i2c_get_clientdata(client);
	sx86XX_resume(this);
	return 0;
}
#endif

#if CONFIG_PM
static int sx9310_suspend(struct device *dev)
{
	struct i2c_client *client = (struct i2c_client *)to_i2c_client(dev);
	psx86XX_t this = (psx86XX_t)i2c_get_clientdata(client);

	write_register(this,SX9310_CPS_CTRL0_REG,0x50);
	sx86XX_suspend(this);
	dev_info(this->pdev, "sx9310 disable IRQ.\n");
	read_regStat(this);
	return 0;
}

static void sx9310_resume(struct device *dev)
{
	struct i2c_client *client = (struct i2c_client *)to_i2c_client(dev);
	psx86XX_t this = (psx86XX_t)i2c_get_clientdata(client);

	sx86XX_resume(this);

	return;
}

static const struct dev_pm_ops sx9310_pm = {
	.prepare = sx9310_suspend,
	.complete = sx9310_resume,
};
#endif

static void sx9310_shutdown(struct i2c_client *client)
{
	psx86XX_t this = (psx86XX_t)i2c_get_clientdata(client);
	write_register(this,SX9310_CPS_CTRL0_REG,0x50);
}

/*====================================================*/
static struct i2c_device_id sx9310_idtable[] = {
	{ DRIVER_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sx9310_idtable);

#ifdef CONFIG_OF
static struct of_device_id sx9310_match_table[] = {
	{ .compatible = "semtech,cap",},
	{ },
};
#else
#define sx9310_match_table NULL
#endif

static struct i2c_driver sx9310_driver = {
	.driver = {
		.owner  = THIS_MODULE,
		.name   = DRIVER_NAME,
		.of_match_table = sx9310_match_table,
#if CONFIG_PM
		.pm	= &sx9310_pm,
#endif
	},
	.id_table = sx9310_idtable,
	.probe	  = sx9310_probe,
	.remove	  = sx9310_remove,
#if defined(USE_KERNEL_SUSPEND)
	.suspend  = sx9310_suspend,
	.resume   = sx9310_resume,
#endif
	.shutdown = sx9310_shutdown,
};
static int __init sx9310_init(void)
{
	return i2c_add_driver(&sx9310_driver);
}
static void __exit sx9310_exit(void)
{
	i2c_del_driver(&sx9310_driver);
}

module_init(sx9310_init);
module_exit(sx9310_exit);

MODULE_AUTHOR("Semtech Corp. (http://www.semtech.com/)");
MODULE_DESCRIPTION("SX9310 Capacitive Touch Controller Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");
