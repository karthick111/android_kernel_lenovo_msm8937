/*
 * Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
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
#ifndef MIPI_DSI2DPI_H
#define MIPI_DSI2DPI_H
#include <linux/spi/spi.h>

struct tc358762_platform_data {
	const char *name;
	u32 reset_gpio;
	bool    i2c_pull_up;
	bool    splash_disabled;
	u32 reset_gpio_flags;
};

struct tc358762_data {
	struct i2c_client *client;
	const struct tc358762_platform_data *pdata;
	struct regulator *vdds_l6;
	struct regulator *vdda_l2;
	struct regulator *vdd_l17;
	const char *clk_src_name;
	struct clk *bb_clk2;
	bool clk_run;
};

#define DELAY_INIT_SEQ 0x0000

/* DSI D-PHY Layer Registers */
#define D0W_DPHYCONTTX		0x0004 /* Data Lane 0 DPHY TX */
#define CLW_DPHYCONTRX		0x0020 /* Clock Lane DPHY RX */
#define D0W_DPHYCONTRX		0x0024 /* Data Lane 0 DPHY Rx */
#define D1W_DPHYCONTRX		0x0028 /* Data Lane 1 DPHY Rx */
#define COM_DPHYCONTRX		0x0038 /* DPHY Rx Common */
#define MON_DPHYRX		0x003C /* DPHY Rx Monitor */
#define CLW_CNTRL		0x0040 /* Clock Lane */
#define D0W_CNTRL		0x0044 /* Data Lane 0 */
#define D1W_CNTRL		0x0048 /* Data Lane 1 */
#define DFTMODE_CNTRL		0x0054 /* DFT Mode */

/* DSI PPI Layer Registers */
#define PPI_STARTPPI		0x0104 /* Start control bit */
#define PPI_BUSYPPI		0x0108 /* Busy bit */
#define PPI_LINEINITCNT	0x0110 /* Line Initialization */
#define PPI_LPTXTIMECNT	0x0114 /* LPTX timing signal */
#define PPI_CLS_ATMR		0x0140 /* Analog timer fcn */
#define PPI_D0S_ATMR		0x0144 /* Analog timer fcn Lane 0 */
#define PPI_D1S_ATMR		0x0148 /* Analog timer fcn Lane 1 */
#define PPI_D0S_CLRSIPOCOUNT	0x0164 /* Assertion timer Lane 0 */
#define PPI_D1S_CLRSIPOCOUNT	0x0168 /* Assertion timer Lane 1 */
#define CLS_PRE		0x0180 /* PHY IO cntr */
#define D0S_PRE		0x0184 /* PHY IO cntr */
#define D1S_PRE		0x0188 /* PHY IO cntr */
#define CLS_PREP		0x01A0 /* PHY IO cntr */
#define D0S_PREP		0x01A4 /* PHY IO cntr */
#define D1S_PREP		0x01A8 /* PHY IO cntr */
#define CLS_ZERO		0x01C0 /* PHY IO cntr */
#define D0S_ZERO		0x01C4 /* PHY IO cntr */
#define D1S_ZERO		0x01C8 /* PHY IO cntr */
#define PPI_CLRFLG		0x01E0 /* PRE cntrs */
#define PPI_CLRSIPO		0x01E4 /* Clear SIPO */
#define PPI_HSTimeout		0x01F0 /* HS RX timeout */
#define PPI_HSTimeoutEnable	0x01F4 /* Enable HS Rx Timeout */

/* DSI Protocol Layer Registers */
#define DSI_STARTDSI		0x0204 /* DSI TX start bit */
#define DSI_BUSYDSI		0x0208 /* DSI busy bit */
#define DSI_LANEENABLE		0x0210 /* Lane enable */
#define DSI_LANESTATUS0	0x0214 /* HS Rx mode */
#define DSI_LANESTATUS1	0x0218 /* ULPS or STOP state */
#define DSI_INTSTATUS		0x0220 /* Interrupt status */
#define DSI_INTMASK		0x0224 /* Interrupt mask */
#define DSI_INTCLR		0x0228 /* Interrupt clear */
#define DSI_LPTXTO		0x0230 /* LP Tx Timeout Counter */
#define DSI_MODE		0x0260 /* DSI Mode, hardwired to 1 */
#define DSI_PAYLOAD0		0x0268 /* LPTX Payload Data 0 */
#define DSI_PAYLOAD1		0x026C /* LPTX Payload Data 1 */
#define DSI_SHORTPKTDAT	0x0270 /* LP Tx Header */
#define DSI_SHORTPKTREQ	0x0274 /* LP Tx Request Status */
#define DSI_BTASTAT		0x0278 /* BTA Status */
#define DSI_BTACLR		0x027C /* BTA Clear */

/* DSI General Registers */
#define DSIERRCNT		0x0300 /* DSI Error Count */
#define DSISIGMOD		0x0304 /* DSI Mode */
/* DSI Application Layer Registers */
#define APLCTRL		0x0400 /* Application Layer Cntrl */
#define APLSTAT		0x0404 /* Application Layer Status */
#define APLERR			0x0408 /* Application Layer Error */
#define PWRMOD			0x040C /* Power Mode (DBI Only) */
#define RDPKTLN		0x0410 /* Packet length */
#define PXLFMT			0x0414 /* RGB Pixel Format */
#define MEMWRCMD		0x0418 /* Memory Write Command (CPU Mode) */

/* LCDC/DPI Host Registers */
#define LCDCTRL_PORT		0x0420 /* LCDC Control, RGB Output Polarity */
#define LCDCTRL_DPI_ENABLE	0x0100 /* Enable DPI interface */
#define LCDCTRL_RGB666_PACKED	0x0000 /* RGB666 18 bits per pixel */
#define LCDCTRL_RGB666_LOOSE	0x0010 /* RGB666 loosely packed 18 bits
						per pixel */
#define LCDCTRL_RGB888		0x0050 /* RGB888 24 bits per pixel */
#define LCDCTRL_VTGEN_ON	0x0002 /* Enable DSI VTGEN */
#define LCDCTRL_MSF		0x0001 /* Enable Magic Square FRC */
#define PORT_DCLK_INVERTED	0x0010 /* Invert Video clock polarity */
#define PORT_VSYNC_INVERTED	0x0008 /* Assert "1" during V-sync */
#define PORT_DE_INVERTED	0x0004 /* Assert "0" when valid data
						 is output */
#define PORT_HSYNC_INVERTED	0x0002 /* Assert "1" during H-sync */
#define HSR_HBPR		0x0424 /* Horiz. Sync Rate, Horiz. Back Porch */
#define HDISPR_HFPR		0x0428 /* Horiz. Display Size, Horiz.
						Front Porch */
#define VSR_VBPR		0x042C /* Vert. Sync Low Pulse Width, Vert.
						Back Porch */
#define VDISPR_VFPR		0x0430 /* Vert. Display Size, Vert.
						Front Porch */
#define VFUEN			0x0434 /* Video Frame update enable */

/* System Controller Registers */
#define SYSSTAT		0x0460 /* System Status */
#define SYSCTRL		0x0464 /* System Control */
#define SYSPLL1		0x0468 /* System PLL Control 1 */
#define SYSPLL2		0x046C /* System PLL Control 2 */
#define SYSPLL3		0x0470 /* System PLL Control 3 */
#define SYSPMCTRL		0x047C /* System Power Management Control */

/* GPIO Registers */
#define GPIOC			0x0480 /* GPIO Control */
#define GPIOO			0x0484 /* GPIO Output */
#define GPIOI			0x0488 /* GPIO Input */

/* Chip Revision Registers */
#define IDREG			0x04A0 /* Chip and Revision ID */
/* Debug Registers */
#define DEBUG00		0x04C0 /* Debug */
/* Command Queue */
#define WCMDQUE		0x0500 /* Write Command Queue */
#define RCMDQUE		0x0504 /* Read Command Queue */

/* DSI DCS commands */
#define DCS_READ_NUM_ERRORS	0x05
#define DCS_READ_POWER_MODE	0x0a
#define DCS_READ_MADCTL	0x0b
#define DCS_READ_PIXEL_FORMAT	0x0c
#define DCS_RDDSDR		0x0f
#define DCS_SLEEP_IN		0x10
#define DCS_SLEEP_OUT		0x11
#define DCS_DISPLAY_OFF	0x28
#define DCS_DISPLAY_ON		0x29
#define DCS_COLUMN_ADDR	0x2a
#define DCS_PAGE_ADDR		0x2b
#define DCS_MEMORY_WRITE	0x2c
#define DCS_TEAR_OFF		0x34
#define DCS_TEAR_ON		0x35
#define DCS_MEM_ACC_CTRL	0x36
#define DCS_PIXEL_FORMAT	0x3a
#define DCS_BRIGHTNESS		0x51
#define DCS_CTRL_DISPLAY	0x53
#define DCS_WRITE_CABC		0x55
#define DCS_READ_CABC		0x56
#define DCS_GET_ID1		0xda
#define DCS_GET_ID2		0xdb
#define DCS_GET_ID3		0xdc

extern void tc358762_suspend(void);
extern void tc358762_reset(void);
extern void tc358762_resume(void);

#endif /* MIPI_DSI2DPI_H */

