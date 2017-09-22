/*
** =============================================================================
** Copyright (c) 2016  Texas Instruments Inc.
**
** This program is free software; you can redistribute it and/or modify it under
** the terms of the GNU General Public License as published by the Free Software
** Foundation; version 2.
**
** This program is distributed in the hope that it will be useful, but WITHOUT
** ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
** FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License along with
** this program; if not, write to the Free Software Foundation, Inc., 51 Franklin
** Street, Fifth Floor, Boston, MA 02110-1301, USA.
**
** File:
**     tas2555-core.c
**
** Description:
**     TAS2555 common functions for Android Linux
**
** =============================================================================
*/

#define DEBUG
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h>
#include <linux/regmap.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/fcntl.h>
#include <asm/uaccess.h>

#include "tas2555.h"
#include "tas2555-core.h"

#define TAS2555_CAL_NAME    "/persist/tas2555_cal.bin"

//set default PLL CLKIN to GPI2 (MCLK) = 0x00
#define TAS2555_DEFAULT_PLL_CLKIN 0x00

static void tas2555_load_calibration(struct tas2555_priv *pTAS2555,
	char *pFileName);
static void tas2555_load_data(struct tas2555_priv *pTAS2555, TData * pData,
	unsigned int nType);
static void tas2555_load_block(struct tas2555_priv *pTAS2555, TBlock * pBlock);
static void tas2555_load_configuration(struct tas2555_priv *pTAS2555,
	unsigned int nConfiguration, bool bLoadSame);

#define TAS2555_UDELAY 0xFFFFFFFE

#define FW_ERR_HEADER -1
#define FW_ERR_SIZE -2

#define TAS2555_BLOCK_PLL			0x00
#define TAS2555_BLOCK_BASE_MAIN		0x01
#define TAS2555_BLOCK_CONF_COEFF	0x03
#define TAS2555_BLOCK_CONF_PRE		0x04
#define TAS2555_BLOCK_CONF_POST		0x05
#define TAS2555_BLOCK_CONF_POST_POWER	0x06
#define TAS2555_BLOCK_CONF_CAL		0x0A

static unsigned int p_tas2555_default_data[] = {
	TAS2555_ASI1_DAC_FORMAT_REG, 0x10,	//ASI1 DAC word length = 24 bits

	TAS2555_PLL_CLKIN_REG, TAS2555_DEFAULT_PLL_CLKIN,	//PLL_CLKIN = GPIO1 (BCLK)
	TAS2555_MAIN_CLKIN_REG, 0x0F,	//NDIV_MUX_CLKIN = PLL_CLK
	TAS2555_PLL_P_VAL_REG, 0x01,	//PLL P = 1
//  TAS2555_PLL_J_VAL_REG,      0x10, //PLL J = 16
	TAS2555_PLL_J_VAL_REG, 0x30,	//PLL J = 48 -> PLL_CLK = 1.536MHz * 48 = 73.728MHz
	TAS2555_PLL_D_VAL_MSB_REG, 0x00,	//PLL D = 0
	TAS2555_PLL_D_VAL_LSB_REG, 0x00,
	TAS2555_PLL_N_VAL_REG, 0x03,	//PLL N = 3 -> NDIV_CLK = 24.576MHz
	TAS2555_DAC_MADC_VAL_REG, 0x08,	//MDAC = 8
	TAS2555_CLK_MISC_REG, 0x20,	//DSP CLK = PLL out
//  TAS2555_ISENSE_DIV_REG,     0x40, //Isense div and MADC final divider configure auto
	TAS2555_ISENSE_DIV_REG, 0x00,	//Isense div and MADC final divider configure auto
//  TAS2555_RAMP_CLK_DIV_LSB_REG,   0x20, //ramp_clk divider = 32 so that 12.288MHz/32 = 384KHz
	TAS2555_RAMP_CLK_DIV_LSB_REG, 0x40,	//ramp_clk divider = 64 so that 24.576MHz/64 = 384KHz
	TAS2555_DSP_MODE_SELECT_REG, 0x22,	//DSP ROM mode 2, default coeffs

//  TAS2555_SPK_CTRL_REG,       0x74, //DAC channel gain
	TAS2555_SPK_CTRL_REG, 0x7C,	//DAC channel gain
//  TAS2555_POWER_CTRL2_REG,    0xA3, //power up
//  TAS2555_POWER_CTRL1_REG,    0xF8, //power up
//  TAS2555_MUTE_REG,       0x00, //unmute
//  TAS2555_SOFT_MUTE_REG,      0x00, //soft unmute
//  TAS2555_CLK_ERR_CTRL,       0x09, //enable clock error detection on PLL
	0xFFFFFFFF, 0xFFFFFFFF
};

#define TAS2555_STARTUP_DATA_PLL_CLKIN_INDEX 3
static unsigned int p_tas2555_startup_data[] = {
	TAS2555_CLK_ERR_CTRL, 0x00,	//disable clock error detection on PLL
	TAS2555_PLL_CLKIN_REG, TAS2555_DEFAULT_PLL_CLKIN,
	TAS2555_POWER_CTRL2_REG, 0xA0,	//Class-D, Boost power up
	TAS2555_POWER_CTRL2_REG, 0xA3,	//Class-D, Boost, IV sense power up
	TAS2555_POWER_CTRL1_REG, 0xF8,	//PLL, DSP, clock dividers power up
	TAS2555_UDELAY, 2000,		//delay
	0xFFFFFFFF, 0xFFFFFFFF
};

static unsigned int p_tas2555_unmute_data[] = {
	TAS2555_MUTE_REG, 0x00,		//unmute
	TAS2555_SOFT_MUTE_REG, 0x00,	//soft unmute
	0xFFFFFFFF, 0xFFFFFFFF
};

static unsigned int p_tas2555_shutdown_data[] = {
	TAS2555_MUTE_REG, 0x03,		//mute
	TAS2555_PLL_CLKIN_REG, 0x0F,	//PLL clock input = osc
	TAS2555_POWER_CTRL1_REG, 0x60,	//DSP power down
	TAS2555_UDELAY, 0xFF,		//delay
	TAS2555_POWER_CTRL2_REG, 0x00,	//Class-D, Boost power down
	TAS2555_POWER_CTRL1_REG, 0x00,	//all power down
	0xFFFFFFFF, 0xFFFFFFFF
};

#if 0
static unsigned int p_tas2555_shutdown_clk_err[] = {
	TAS2555_CLK_ERR_CTRL, 0x09,	//enable clock error detection on PLL
	0xFFFFFFFF, 0xFFFFFFFF
};
#endif

static unsigned int p_tas2555_mute_DSP_down_data[] = {
	TAS2555_MUTE_REG, 0x03,		//mute
	TAS2555_PLL_CLKIN_REG, 0x0F,	//PLL clock input = osc
	TAS2555_POWER_CTRL1_REG, 0x60,	//DSP power down
	TAS2555_UDELAY, 0xFF,		//delay
	0xFFFFFFFF, 0xFFFFFFFF
};

static int tas2555_dev_load_data(struct tas2555_priv *pTAS2555,
	unsigned int *pData)
{
	int ret = 0;
	unsigned int n = 0;
	unsigned int nRegister;
	unsigned int nData;

	do {
		nRegister = pData[n * 2];
		nData = pData[n * 2 + 1];
		if (nRegister == TAS2555_UDELAY)
			udelay(nData);
		else if (nRegister != 0xFFFFFFFF){
			ret = pTAS2555->write(pTAS2555, nRegister, nData);
			if(ret < 0) {
				dev_err(pTAS2555->dev, "Reg Write err %d\n", ret);
				break;
			}
		}
		n++;
	} while (nRegister != 0xFFFFFFFF);

	return ret;
}

int tas2555_load_default(struct tas2555_priv *pTAS2555)
{
	return tas2555_dev_load_data(pTAS2555, p_tas2555_default_data);
}

void tas2555_enable(struct tas2555_priv *pTAS2555, bool bEnable)
{
	dev_dbg(pTAS2555->dev, "Enable: %d\n", bEnable);
	if (bEnable) {
		if (!pTAS2555->mbPowerUp) {
			TConfiguration *pConfiguration;

			if (!pTAS2555->mbCalibrationLoaded) {
				tas2555_load_calibration(pTAS2555, TAS2555_CAL_NAME);
				pTAS2555->mbCalibrationLoaded = true;
			}
			dev_dbg(pTAS2555->dev, "Enable: load startup sequence\n");
			tas2555_dev_load_data(pTAS2555, p_tas2555_startup_data);
			if (pTAS2555->mpFirmware->mpConfigurations) {
				pConfiguration = &(pTAS2555->mpFirmware->mpConfigurations[pTAS2555->mnCurrentConfiguration]);
				tas2555_load_data(pTAS2555, &(pConfiguration->mData),
					TAS2555_BLOCK_CONF_POST_POWER);
				if (pTAS2555->mbLoadConfigurationPostPowerUp) {
					dev_dbg(pTAS2555->dev,	"Enable: load configuration: %s, %s\n",
						pConfiguration->mpName, pConfiguration->mpDescription);
					tas2555_load_data(pTAS2555, &(pConfiguration->mData),
						TAS2555_BLOCK_CONF_COEFF);
					pTAS2555->mbLoadConfigurationPostPowerUp = false;
					if (pTAS2555->mpCalFirmware->mnCalibrations) {
						dev_dbg(pTAS2555->dev, "Enable: load calibration\n");
						tas2555_load_block(pTAS2555, &(pTAS2555->mpCalFirmware->mpCalibrations[pTAS2555->mnCurrentCalibration].mBlock));
					}
				}
			}
			dev_dbg(pTAS2555->dev, "Enable: load unmute sequence\n");
			tas2555_dev_load_data(pTAS2555, p_tas2555_unmute_data);
			pTAS2555->mbPowerUp = true;
		}
	} else {
		if (pTAS2555->mbPowerUp) {
			dev_dbg(pTAS2555->dev, "Enable: load shutdown sequence\n");
			tas2555_dev_load_data(pTAS2555, p_tas2555_shutdown_data);
			//tas2555_dev_load_data(pTAS2555, p_tas2555_shutdown_clk_err);
			pTAS2555->mbPowerUp = false;
		}
	}
}

int tas2555_set_sampling_rate(struct tas2555_priv *pTAS2555, unsigned int nSamplingRate)
{
	TConfiguration *pConfiguration;
	unsigned int nConfiguration;

	dev_dbg(pTAS2555->dev, "tas2555_setup_clocks: nSamplingRate = %d [Hz]\n",
		nSamplingRate);

	if ((!pTAS2555->mpFirmware->mpPrograms) ||
		(!pTAS2555->mpFirmware->mpConfigurations)) {
		dev_err(pTAS2555->dev, "Firmware not loaded\n");
		return -EINVAL;
	}

	pConfiguration = &(pTAS2555->mpFirmware->mpConfigurations[pTAS2555->mnCurrentConfiguration]);
	if (pConfiguration->mnSamplingRate == nSamplingRate) {
		dev_info(pTAS2555->dev, "Sampling rate for current configuration matches: %d\n",
			nSamplingRate);
		return 0;
	}

	for (nConfiguration = 0;
		nConfiguration < pTAS2555->mpFirmware->mnConfigurations;
		nConfiguration++) {
		pConfiguration =
			&(pTAS2555->mpFirmware->mpConfigurations[nConfiguration]);
		if ((pConfiguration->mnSamplingRate == nSamplingRate)
			&&(pConfiguration->mnProgram == pTAS2555->mnCurrentProgram)){
			dev_info(pTAS2555->dev,
				"Found configuration: %s, with compatible sampling rate %d\n",
				pConfiguration->mpName, nSamplingRate);
			tas2555_load_configuration(pTAS2555, nConfiguration, false);
			return 0;
		}
	}

	dev_err(pTAS2555->dev, "Cannot find a configuration that supports sampling rate: %d\n",
		nSamplingRate);

	return -EINVAL;
}

static void fw_print_header(struct tas2555_priv *pTAS2555, TFirmware * pFirmware)
{
	dev_info(pTAS2555->dev, "  FW Size       = %d", pFirmware->mnFWSize);
	dev_info(pTAS2555->dev, "  Checksum      = 0x%04X", pFirmware->mnChecksum);
	dev_info(pTAS2555->dev, "  PPC Version   = 0x%04X", pFirmware->mnPPCVersion);
	dev_info(pTAS2555->dev, "  FW  Version   = 0x%04X", pFirmware->mnFWVersion);
	dev_info(pTAS2555->dev, "  Timestamp     = %d", pFirmware->mnTimeStamp);
	dev_info(pTAS2555->dev, "  DDC Name      = %s", pFirmware->mpDDCName);
	dev_info(pTAS2555->dev, "  Description   = %s", pFirmware->mpDescription);
}

inline unsigned int fw_convert_number(unsigned char *pData)
{
	return pData[3] + (pData[2] << 8) + (pData[1] << 16) + (pData[0] << 24);
}

static int fw_parse_header(struct tas2555_priv *pTAS2555,
	TFirmware * pFirmware, unsigned char *pData,
	unsigned int nSize)
{
	unsigned char *pDataStart = pData;
	unsigned int n;
	unsigned char pMagicNumber[] = { 0x35, 0x35, 0x35, 0x32 };
	if (nSize < 102) {
		dev_err(pTAS2555->dev, "Firmware: Header too short");
		return -1;
	}

	if (memcmp(pData, pMagicNumber, 4)) {
		dev_err(pTAS2555->dev, "Firmware: Magic number doesn't match");
		return -1;
	}

	pData += 4;

	pFirmware->mnFWSize = fw_convert_number(pData);
	pData += 4;

	pFirmware->mnChecksum = fw_convert_number(pData);
	pData += 4;

	pFirmware->mnPPCVersion = fw_convert_number(pData);
	pData += 4;

	pFirmware->mnFWVersion = fw_convert_number(pData);
	pData += 4;

	pFirmware->mnTimeStamp = fw_convert_number(pData);
	pData += 4;

	memcpy(pFirmware->mpDDCName, pData, 64);
	pData += 64;

	n = strlen(pData);
	pFirmware->mpDescription = kmemdup(pData, n + 1, GFP_KERNEL);
	pData += n + 1;

	if ((pData - pDataStart) >= nSize) {
		dev_err(pTAS2555->dev, "Firmware: Header too short after DDC description");
		return -1;
	}

	pFirmware->mnDeviceFamily = fw_convert_number(pData);
	pData += 4;

	pFirmware->mnDevice = fw_convert_number(pData);
	pData += 4;

	fw_print_header(pTAS2555, pFirmware);

	return pData - pDataStart;
}

static int fw_parse_block_data(TBlock * pBlock, unsigned char *pData)
{
	unsigned char *pDataStart = pData;
	unsigned int n;

	pBlock->mnType = fw_convert_number(pData);
	pData += 4;

	pBlock->mnCommands = fw_convert_number(pData);
	pData += 4;

	n = pBlock->mnCommands * 4;
	pBlock->mpData = kmemdup(pData, n, GFP_KERNEL);
	pData += n;

	return pData - pDataStart;
}

static int fw_parse_data(TData * pImageData, unsigned char *pData)
{
	unsigned char *pDataStart = pData;
	unsigned int nBlock;
	unsigned int n;

	memcpy(pImageData->mpName, pData, 64);
	pData += 64;

	n = strlen(pData);
	pImageData->mpDescription = kmemdup(pData, n + 1, GFP_KERNEL);
	pData += n + 1;

	pImageData->mnBlocks = (pData[0] << 8) + pData[1];
	pData += 2;

	pImageData->mpBlocks =
		kmalloc(sizeof(TBlock) * pImageData->mnBlocks, GFP_KERNEL);

	for (nBlock = 0; nBlock < pImageData->mnBlocks; nBlock++) {
		n = fw_parse_block_data(&(pImageData->mpBlocks[nBlock]), pData);
		pData += n;
	}

	return pData - pDataStart;
}

static int fw_parse_pll_data(TFirmware * pFirmware, unsigned char *pData)
{
	unsigned char *pDataStart = pData;
	unsigned int n;
	unsigned int nPLL;
	TPLL *pPLL;

	pFirmware->mnPLLs = (pData[0] << 8) + pData[1];
	pData += 2;

	pFirmware->mpPLLs = kmalloc(sizeof(TPLL) * pFirmware->mnPLLs, GFP_KERNEL);
	for (nPLL = 0; nPLL < pFirmware->mnPLLs; nPLL++) {
		pPLL = &(pFirmware->mpPLLs[nPLL]);

		memcpy(pPLL->mpName, pData, 64);
		pData += 64;

		n = strlen(pData);
		pPLL->mpDescription = kmemdup(pData, n + 1, GFP_KERNEL);
		pData += n + 1;

		n = fw_parse_block_data(&(pPLL->mBlock), pData);
		pData += n;
	}

	return pData - pDataStart;
}

static int fw_parse_program_data(TFirmware * pFirmware, unsigned char *pData)
{
	unsigned char *pDataStart = pData;
	unsigned int n;
	unsigned int nProgram;
	TProgram *pProgram;

	pFirmware->mnPrograms = (pData[0] << 8) + pData[1];
	pData += 2;

	pFirmware->mpPrograms =
		kmalloc(sizeof(TProgram) * pFirmware->mnPrograms, GFP_KERNEL);
	for (nProgram = 0; nProgram < pFirmware->mnPrograms; nProgram++) {
		pProgram = &(pFirmware->mpPrograms[nProgram]);
		memcpy(pProgram->mpName, pData, 64);
		pData += 64;

		n = strlen(pData);
		pProgram->mpDescription = kmemdup(pData, n + 1, GFP_KERNEL);
		pData += n + 1;

		n = fw_parse_data(&(pProgram->mData), pData);
		pData += n;
	}

	return pData - pDataStart;
}

static int fw_parse_configuration_data(TFirmware * pFirmware,
	unsigned char *pData)
{
	unsigned char *pDataStart = pData;
	unsigned int n;
	unsigned int nConfiguration;
	TConfiguration *pConfiguration;

	pFirmware->mnConfigurations = (pData[0] << 8) + pData[1];
	pData += 2;

	pFirmware->mpConfigurations =
		kmalloc(sizeof(TConfiguration) * pFirmware->mnConfigurations,
		GFP_KERNEL);
	for (nConfiguration = 0; nConfiguration < pFirmware->mnConfigurations;
		nConfiguration++) {
		pConfiguration = &(pFirmware->mpConfigurations[nConfiguration]);
		memcpy(pConfiguration->mpName, pData, 64);
		pData += 64;

		n = strlen(pData);
		pConfiguration->mpDescription = kmemdup(pData, n + 1, GFP_KERNEL);
		pData += n + 1;

		pConfiguration->mnProgram = pData[0];
		pData++;

		pConfiguration->mnPLL = pData[0];
		pData++;

		pConfiguration->mnSamplingRate = fw_convert_number(pData);
		pData += 4;

		n = fw_parse_data(&(pConfiguration->mData), pData);
		pData += n;
	}

	return pData - pDataStart;
}

int fw_parse_calibration_data(TFirmware * pFirmware, unsigned char *pData)
{
	unsigned char *pDataStart = pData;
	unsigned int n;
	unsigned int nCalibration;
	TCalibration *pCalibration;

	pFirmware->mnCalibrations = (pData[0] << 8) + pData[1];
	pData += 2;

	pFirmware->mpCalibrations =
		kmalloc(sizeof(TCalibration) * pFirmware->mnCalibrations, GFP_KERNEL);
	for (nCalibration = 0;
		nCalibration < pFirmware->mnCalibrations;
		nCalibration++) {
		pCalibration = &(pFirmware->mpCalibrations[nCalibration]);
		memcpy(pCalibration->mpName, pData, 64);
		pData += 64;

		n = strlen(pData);
		pCalibration->mpDescription = kmemdup(pData, n + 1, GFP_KERNEL);
		pData += n + 1;

		pCalibration->mnProgram = pData[0];
		pData++;

		pCalibration->mnConfiguration = pData[0];
		pData++;

		n = fw_parse_block_data(&(pCalibration->mBlock), pData);
		pData += n;
	}

	return pData - pDataStart;
}

static int fw_parse(struct tas2555_priv *pTAS2555,
	TFirmware * pFirmware,
	unsigned char *pData,
	unsigned int nSize)
{
	int nPosition = 0;

	nPosition = fw_parse_header(pTAS2555, pFirmware, pData, nSize);
	if (nPosition < 0) {
		dev_err(pTAS2555->dev, "Firmware: Wrong Header");
		return FW_ERR_HEADER;
	}

	if (nPosition >= nSize) {
		dev_err(pTAS2555->dev, "Firmware: Too short");
		return FW_ERR_SIZE;
	}

	pData += nPosition;
	nSize -= nPosition;
	nPosition = 0;

	nPosition = fw_parse_pll_data(pFirmware, pData);

	pData += nPosition;
	nSize -= nPosition;
	nPosition = 0;

	nPosition = fw_parse_program_data(pFirmware, pData);

	pData += nPosition;
	nSize -= nPosition;
	nPosition = 0;

	nPosition = fw_parse_configuration_data(pFirmware, pData);

	pData += nPosition;
	nSize -= nPosition;
	nPosition = 0;

	if (nSize > 64)
		nPosition = fw_parse_calibration_data(pFirmware, pData);

	return 0;
}

static void tas2555_load_block(struct tas2555_priv *pTAS2555, TBlock * pBlock)
{
	unsigned int nCommand = 0;
	unsigned char nBook;
	unsigned char nPage;
	unsigned char nOffset;
	unsigned char nData;
	unsigned int nLength;
	unsigned char *pData = pBlock->mpData;

	dev_dbg(pTAS2555->dev, "TAS2555 load block: Type = %d, commands = %d\n",
		pBlock->mnType, pBlock->mnCommands);
	while (nCommand < pBlock->mnCommands) {
		pData = pBlock->mpData + nCommand * 4;

		nBook = pData[0];
		nPage = pData[1];
		nOffset = pData[2];
		nData = pData[3];

		nCommand++;

		if (nOffset <= 0x7F){
			pTAS2555->write(pTAS2555, TAS2555_REG(nBook, nPage, nOffset),
				nData);
		}else if (nOffset == 0x81) {
			unsigned int nSleep = (nBook << 8) + nPage;
			msleep(nSleep);
		}else if (nOffset == 0x85) {
			pData += 4;
			nLength = (nBook << 8) + nPage;
			nBook = pData[0];
			nPage = pData[1];
			nOffset = pData[2];
			if (nLength > 1)
				pTAS2555->bulk_write(pTAS2555, TAS2555_REG(nBook, nPage,
						nOffset), pData + 3, nLength);
			else
				pTAS2555->write(pTAS2555, TAS2555_REG(nBook, nPage, nOffset),
					pData[3]);

			nCommand++;
			if (nLength >= 2)
				nCommand += ((nLength - 2) / 4) + 1;
		}
	}
}

static void tas2555_load_data(struct tas2555_priv *pTAS2555, TData * pData,
	unsigned int nType)
{
	unsigned int nBlock;
	TBlock *pBlock;

	dev_dbg(pTAS2555->dev,
		"TAS2555 load data: %s, Blocks = %d, Block Type = %d\n", pData->mpName,
		pData->mnBlocks, nType);

	for (nBlock = 0; nBlock < pData->mnBlocks; nBlock++) {
		pBlock = &(pData->mpBlocks[nBlock]);
		if (pBlock->mnType == nType)
			tas2555_load_block(pTAS2555, pBlock);
	}
}

static void tas2555_load_configuration(struct tas2555_priv *pTAS2555,
	unsigned int nConfiguration, bool bLoadSame)
{
	TConfiguration *pCurrentConfiguration;
	TConfiguration *pNewConfiguration;
	TPLL *pNewPLL;

	dev_dbg(pTAS2555->dev, "tas2555_load_configuration: %d\n", nConfiguration);

	if ((!pTAS2555->mpFirmware->mpPrograms) ||
		(!pTAS2555->mpFirmware->mpConfigurations)) {
		dev_err(pTAS2555->dev, "Firmware not loaded\n");
		return;
	}

	if (nConfiguration >= pTAS2555->mpFirmware->mnConfigurations) {
		dev_err(pTAS2555->dev, "Configuration %d doesn't exist\n",
			nConfiguration);
		return;
	}

	if ((nConfiguration == pTAS2555->mnCurrentConfiguration) && (!bLoadSame)) {
		dev_info(pTAS2555->dev, "Configuration %d is already loaded\n",
			nConfiguration);
		return;
	}

	pCurrentConfiguration =
		&(pTAS2555->mpFirmware->mpConfigurations[pTAS2555->mnCurrentConfiguration]);
	pNewConfiguration =
		&(pTAS2555->mpFirmware->mpConfigurations[nConfiguration]);

	if (pNewConfiguration->mnProgram != pCurrentConfiguration->mnProgram) {
		dev_err(pTAS2555->dev,
			"Configuration %d, %s doesn't share the same program as current %d\n",
			nConfiguration, pNewConfiguration->mpName, pCurrentConfiguration->mnProgram);
		return;
	}

	if (pNewConfiguration->mnPLL >= pTAS2555->mpFirmware->mnPLLs) {
		dev_err(pTAS2555->dev,
			"Configuration %d, %s doesn't have a valid PLL index %d\n",
			nConfiguration, pNewConfiguration->mpName, pNewConfiguration->mnPLL);
		return;
	}

	pNewPLL = &(pTAS2555->mpFirmware->mpPLLs[pNewConfiguration->mnPLL]);

	if (pTAS2555->mbPowerUp) {
		if (pNewConfiguration->mnPLL != pCurrentConfiguration->mnPLL) {
			dev_dbg(pTAS2555->dev,
				"TAS2555 is powered up -> mute and power down DSP before loading new configuration\n");
			//tas2555_dev_load_data(pTAS2555, p_tas2555_mute_DSP_down_data);
			tas2555_dev_load_data(pTAS2555, p_tas2555_shutdown_data);

			dev_dbg(pTAS2555->dev,
				"load post block from current configuration: %s, before loading new configuration: %s\n",
				pCurrentConfiguration->mpName, pNewConfiguration->mpName);
			tas2555_load_data(pTAS2555, &(pCurrentConfiguration->mData),
				TAS2555_BLOCK_CONF_POST);
			dev_dbg(pTAS2555->dev, "TAS2555: load new PLL: %s, block data\n",
				pNewPLL->mpName);
			tas2555_load_block(pTAS2555, &(pNewPLL->mBlock));
			pTAS2555->mnCurrentSampleRate = pNewConfiguration->mnSamplingRate;
			dev_dbg(pTAS2555->dev,
				"load new configuration: %s, pre block data\n",
				pNewConfiguration->mpName);
			tas2555_load_data(pTAS2555, &(pNewConfiguration->mData),
				TAS2555_BLOCK_CONF_PRE);
			dev_dbg(pTAS2555->dev, "TAS2555: power up TAS2555\n");
			tas2555_dev_load_data(pTAS2555, p_tas2555_startup_data);
			dev_dbg(pTAS2555->dev,
				"TAS2555: load new configuration: %s, post power up block data\n",
				pNewConfiguration->mpName);
			tas2555_load_data(pTAS2555, &(pNewConfiguration->mData),
				TAS2555_BLOCK_CONF_POST_POWER);
			dev_dbg(pTAS2555->dev,
				"TAS2555: load new configuration: %s, coeff block data\n",
				pNewConfiguration->mpName);
			tas2555_load_data(pTAS2555, &(pNewConfiguration->mData),
				TAS2555_BLOCK_CONF_COEFF);
			dev_dbg(pTAS2555->dev, "TAS2555: unmute TAS2555\n");
			tas2555_dev_load_data(pTAS2555, p_tas2555_unmute_data);
		} else {
			dev_dbg(pTAS2555->dev,
				"TAS2555 is powered up, no change in PLL: load new configuration: %s, coeff block data\n",
				pNewConfiguration->mpName);
			tas2555_load_data(pTAS2555, &(pNewConfiguration->mData),
				TAS2555_BLOCK_CONF_COEFF);
		}
		pTAS2555->mbLoadConfigurationPostPowerUp = false;
	} else {
		dev_dbg(pTAS2555->dev,
			"TAS2555 was powered down -> set flag to load configuration data when OS powers up the TAS2555 the next time\n");
		if (pNewConfiguration->mnPLL != pCurrentConfiguration->mnPLL) {
			dev_dbg(pTAS2555->dev,
				"load post block from current configuration: %s, before loading new configuration: %s\n",
				pCurrentConfiguration->mpName, pNewConfiguration->mpName);
			tas2555_load_data(pTAS2555, &(pCurrentConfiguration->mData),
				TAS2555_BLOCK_CONF_POST);
			dev_dbg(pTAS2555->dev, "TAS2555: load new PLL: %s, block data\n",
				pNewPLL->mpName);
			tas2555_load_block(pTAS2555, &(pNewPLL->mBlock));
			pTAS2555->mnCurrentSampleRate = pNewConfiguration->mnSamplingRate;
			dev_dbg(pTAS2555->dev,
				"load new configuration: %s, pre block data\n",
				pNewConfiguration->mpName);
			tas2555_load_data(pTAS2555, &(pNewConfiguration->mData),
				TAS2555_BLOCK_CONF_PRE);
		}
		pTAS2555->mbLoadConfigurationPostPowerUp = true;
	}

	pTAS2555->mnCurrentConfiguration = nConfiguration;
}

int tas2555_set_config(struct tas2555_priv *pTAS2555, int config)
{
	TConfiguration *pConfiguration;
	TProgram *pProgram;
	unsigned int nProgram = pTAS2555->mnCurrentProgram;
	unsigned int nConfiguration = config;

	if ((!pTAS2555->mpFirmware->mpPrograms) ||
		(!pTAS2555->mpFirmware->mpConfigurations)) {
		dev_err(pTAS2555->dev, "Firmware not loaded\n");
		return -1;
	}

	if (nConfiguration >= pTAS2555->mpFirmware->mnConfigurations) {
		dev_err(pTAS2555->dev, "Configuration %d doesn't exist\n",
			nConfiguration);
		return -1;
	}

	pConfiguration = &(pTAS2555->mpFirmware->mpConfigurations[nConfiguration]);
	pProgram = &(pTAS2555->mpFirmware->mpPrograms[nProgram]);

	if (nProgram != pConfiguration->mnProgram) {
		dev_err(pTAS2555->dev,
			"Configuration %d, %s with Program %d isn't compatible with existing Program %d, %s\n",
			nConfiguration, pConfiguration->mpName, pConfiguration->mnProgram,
			nProgram, pProgram->mpName);
		return -1;
	}

	tas2555_load_configuration(pTAS2555, nConfiguration, false);

	return 0;
}

void tas2555_clear_firmware(TFirmware *pFirmware)
{
	unsigned int n, nn;
	if (!pFirmware) return;
	if (pFirmware->mpDescription) kfree(pFirmware->mpDescription);

	for (n = 0; n < pFirmware->mnPLLs; n++)
	{
		kfree(pFirmware->mpPLLs[n].mpDescription);
		kfree(pFirmware->mpPLLs[n].mBlock.mpData);
	}
	kfree(pFirmware->mpPLLs);

	for (n = 0; n < pFirmware->mnPrograms; n++)
	{
		kfree(pFirmware->mpPrograms[n].mpDescription);
		kfree(pFirmware->mpPrograms[n].mData.mpDescription);
		for (nn = 0; nn < pFirmware->mpPrograms[n].mData.mnBlocks; nn++)
			kfree(pFirmware->mpPrograms[n].mData.mpBlocks[nn].mpData);
		kfree(pFirmware->mpPrograms[n].mData.mpBlocks);
	}
	kfree(pFirmware->mpPrograms);

	for (n = 0; n < pFirmware->mnConfigurations; n++)
	{
		kfree(pFirmware->mpConfigurations[n].mpDescription);
		kfree(pFirmware->mpConfigurations[n].mData.mpDescription);
		for (nn = 0; nn < pFirmware->mpConfigurations[n].mData.mnBlocks; nn++)
			kfree(pFirmware->mpConfigurations[n].mData.mpBlocks[nn].mpData);
		kfree(pFirmware->mpConfigurations[n].mData.mpBlocks);
	}
	kfree(pFirmware->mpConfigurations);

	for (n = 0; n < pFirmware->mnCalibrations; n++)
	{
		kfree(pFirmware->mpCalibrations[n].mpDescription);
		kfree(pFirmware->mpCalibrations[n].mBlock.mpData);
	}
	kfree(pFirmware->mpCalibrations);

	memset(pFirmware, 0x00, sizeof(TFirmware));
}

static void tas2555_load_calibration(struct tas2555_priv *pTAS2555,
	char *pFileName)
{
	int nResult;
	int nFile;
	mm_segment_t fs;
	unsigned char pBuffer[512];
	int nSize = 0;

	dev_dbg(pTAS2555->dev, "%s:\n", __func__);

	fs = get_fs();
	set_fs(KERNEL_DS);
	nFile = sys_open(pFileName, O_RDONLY, 0);

	dev_info(pTAS2555->dev, "TAS2555 calibration file = %s, handle = %d\n",
		pFileName, nFile);

	if (nFile >= 0) {
		nSize = sys_read(nFile, pBuffer, 512);
		sys_close(nFile);
	} else {
		dev_err(pTAS2555->dev, "TAS2555 cannot open calibration file: %s\n",
			pFileName);
	}

	set_fs(fs);

	if (!nSize)
		return;

	tas2555_clear_firmware(pTAS2555->mpCalFirmware);

	dev_info(pTAS2555->dev, "TAS2555 calibration file size = %d\n", nSize);
	nResult = fw_parse(pTAS2555, pTAS2555->mpCalFirmware, pBuffer, nSize);

	if (nResult) {
		dev_err(pTAS2555->dev, "TAS2555 calibration file is corrupt\n");
		return;
	}

	dev_info(pTAS2555->dev, "TAS2555 calibration: %d calibrations\n",
		pTAS2555->mpCalFirmware->mnCalibrations);
}

void tas2555_fw_ready(const struct firmware *pFW, void *pContext)
{
	struct tas2555_priv *pTAS2555 = (struct tas2555_priv *) pContext;
	int nResult;
	unsigned int nProgram = 0;
	unsigned int nSampleRate = 0;

	dev_info(pTAS2555->dev, "%s:\n", __func__);

	if (unlikely(!pFW) || unlikely(!pFW->data)) {
		dev_err(pTAS2555->dev, "%s firmware is not loaded.\n",
			TAS2555_FW_NAME);
		return;
	}

	if (pTAS2555->mpFirmware->mpConfigurations){
		nProgram = pTAS2555->mnCurrentProgram;
		nSampleRate = pTAS2555->mnCurrentSampleRate;
		dev_dbg(pTAS2555->dev, "clear current firmware\n");
		tas2555_clear_firmware(pTAS2555->mpFirmware);
	}

	nResult = fw_parse(pTAS2555, pTAS2555->mpFirmware,
		(unsigned char *) (pFW->data),	pFW->size);

	release_firmware(pFW);

	if (nResult) {
		dev_err(pTAS2555->dev, "firmware is corrupt\n");
		return;
	}

	if (!pTAS2555->mpFirmware->mnPrograms) {
		dev_err(pTAS2555->dev, "firmware contains no programs\n");
		return;
	}

	if (!pTAS2555->mpFirmware->mnConfigurations) {
		dev_err(pTAS2555->dev,
			"firmware contains no configurations\n");
		return;
	}

	if(nProgram >= pTAS2555->mpFirmware->mnPrograms){
		dev_info(pTAS2555->dev,
			"no previous program, set to default\n");
		nProgram = 0;
	}

	pTAS2555->mnCurrentSampleRate = nSampleRate;

	tas2555_set_program(pTAS2555, nProgram);
}

int tas2555_set_program(struct tas2555_priv *pTAS2555,
	unsigned int nProgram)
{
	TPLL *pPLL;
	TConfiguration *pConfiguration;
	unsigned int nConfiguration = 0;
	unsigned int nSampleRate = 0;
	unsigned int Value = 0;
	bool bFound = false;
	int nResult = -1;

	if ((!pTAS2555->mpFirmware->mpPrograms) ||
		(!pTAS2555->mpFirmware->mpConfigurations)) {
		dev_err(pTAS2555->dev, "Firmware not loaded\n");
		return -1;
	}

	if (nProgram >= pTAS2555->mpFirmware->mnPrograms) {
		dev_err(pTAS2555->dev, "TAS2555: Program %d doesn't exist\n",
			nConfiguration);
		return -1;
	}

	nConfiguration = 0;
	nSampleRate = pTAS2555->mnCurrentSampleRate;

	while (!bFound
		&& (nConfiguration < pTAS2555->mpFirmware->mnConfigurations)) {
		if (pTAS2555->mpFirmware->mpConfigurations[nConfiguration].mnProgram
			== nProgram){
			if(nSampleRate == 0){
				bFound = true;
				dev_info(pTAS2555->dev, "find default configuration %d\n", nConfiguration);
			}else if(nSampleRate
				== pTAS2555->mpFirmware->mpConfigurations[nConfiguration].mnSamplingRate){
				bFound = true;
				dev_info(pTAS2555->dev, "find matching configuration %d\n", nConfiguration);
			}else{
				nConfiguration++;
			}
		}else{
			nConfiguration++;
		}
	}

	if (!bFound) {
		dev_err(pTAS2555->dev,
			"Program %d, no valid configuration found for sample rate %d, ignore\n",
			nProgram, nSampleRate);
		return -1;
	}

	pTAS2555->mnCurrentProgram = nProgram;

	tas2555_dev_load_data(pTAS2555, p_tas2555_mute_DSP_down_data);
	pTAS2555->write(pTAS2555, TAS2555_SW_RESET_REG, 0x01);

	udelay(1000);
	pTAS2555->mnCurrentBook = 0;
	pTAS2555->mnCurrentPage = 0;

	dev_info(pTAS2555->dev, "load program %d\n", nProgram);
	tas2555_load_data(pTAS2555,
		&(pTAS2555->mpFirmware->mpPrograms[nProgram].mData),
		TAS2555_BLOCK_BASE_MAIN);
//    pTAS2555->write(pTAS2555, TAS2555_CURRENT_LIMIT_REG,0x00);
	pTAS2555->mnCurrentConfiguration = nConfiguration;

	pConfiguration =
		&(pTAS2555->mpFirmware->mpConfigurations[nConfiguration]);
	pPLL = &(pTAS2555->mpFirmware->mpPLLs[pConfiguration->mnPLL]);
	dev_dbg(pTAS2555->dev,
		"TAS2555 load PLL: %s block for Configuration %s\n",
		pPLL->mpName, pConfiguration->mpName);

	tas2555_load_block(pTAS2555, &(pPLL->mBlock));
	pTAS2555->mnCurrentSampleRate = pConfiguration->mnSamplingRate;
	dev_dbg(pTAS2555->dev,
		"load configuration %s conefficient pre block\n",
		pConfiguration->mpName);
	tas2555_load_data(pTAS2555, &(pConfiguration->mData), TAS2555_BLOCK_CONF_PRE);

	nResult = pTAS2555->read(pTAS2555, TAS2555_CRC_CHECKSUM_REG, &Value);
	dev_info(pTAS2555->dev, "uCDSP Checksum: 0x%02x\n", Value);
	nResult = pTAS2555->read(pTAS2555, TAS2555_PLL_CLKIN_REG, &Value);
	dev_info(pTAS2555->dev, "TAS2555 PLL_CLKIN = 0x%02X\n", Value);
	p_tas2555_startup_data[TAS2555_STARTUP_DATA_PLL_CLKIN_INDEX] = Value;

	if (pTAS2555->mbPowerUp){
		dev_dbg(pTAS2555->dev, "device powered up, load startup\n");
		tas2555_dev_load_data(pTAS2555, p_tas2555_startup_data);
		dev_dbg(pTAS2555->dev,
			"device powered up, load configuration %s post power block\n",
			pConfiguration->mpName);
		tas2555_load_data(pTAS2555, &(pConfiguration->mData),
			TAS2555_BLOCK_CONF_POST_POWER);
	}

	tas2555_load_configuration(pTAS2555, nConfiguration, true);
	if (pTAS2555->mbPowerUp){
		dev_dbg(pTAS2555->dev,
			"device powered up, load unmute\n");
		tas2555_dev_load_data(pTAS2555, p_tas2555_unmute_data);
	}

	return 0;
}

int tas2555_set_calibration(struct tas2555_priv *pTAS2555,
	unsigned int nCalibration)
{
	if ((!pTAS2555->mpFirmware->mpPrograms) || (!pTAS2555->mpFirmware->mpConfigurations))
	{
		dev_err(pTAS2555->dev, "Firmware not loaded\n\r");
		return -1;
	}

	if (nCalibration == 0x00FF)
	{
		dev_info(pTAS2555->dev, "load new calibration file %s\n", TAS2555_CAL_NAME);
		tas2555_load_calibration(pTAS2555, TAS2555_CAL_NAME);
		nCalibration = 0;
	}

	if (nCalibration >= pTAS2555->mpFirmware->mnCalibrations) {
		dev_err(pTAS2555->dev,
			"Calibration %d doesn't exist\n", nCalibration);
		return -1;
	}

	pTAS2555->mnCurrentCalibration = nCalibration;
	tas2555_load_block(pTAS2555,
		&(pTAS2555->mpCalFirmware->mpCalibrations[pTAS2555->mnCurrentCalibration].mBlock));

	return 0;
}

MODULE_AUTHOR("Texas Instruments Inc.");
MODULE_DESCRIPTION("TAS2555 common functions for Android Linux");
MODULE_LICENSE("GPLv2");
