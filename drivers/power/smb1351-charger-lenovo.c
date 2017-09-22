/* Copyright (c) 2015-2016 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "SMB1351 %s: " fmt, __func__
//#define DEBUG

//#define pr_fmt(fmt) "SMB1351 %s: %s: " fmt, dev_name(chip->dev), __func__

#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/of.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/pinctrl/consumer.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/wakelock.h>
#include <linux/reboot.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/delay.h>
#include <linux/bitops.h>
#include <linux/msm_bcl.h>

/*===============feature operation==================*/
#define PSY_PARALLEL_NAME "ext-charger-dual"
//#define PSY_PARALLEL_NAME "usb-parallel"
#define EXT_CHARGER_POWER_SUPPLY
#define MMI_TEST
#define CHG_HEATBEAT_WORK
#define APP_CONTROL
#define LENOVO_OTG_USB_SHORT

#define CHARGER_CHECK_JIFFIES		(HZ * 2)
#define VBUS_DIV_FACTOR			7
/*==============feature operation end===============*/

#ifdef DEBUG
#ifdef pr_debug
#undef pr_debug
#define pr_debug(fmt, ...) \
	printk(KERN_INFO pr_fmt(fmt), ##__VA_ARGS__)
#endif

#ifdef pr_info
#undef pr_info
#define pr_info(fmt, ...) \
	printk(KERN_INFO pr_fmt(fmt), ##__VA_ARGS__)
#endif
#endif

#define SMB1351_SHUTDOWN_BATTERY_VOLTGE 3400000

/* Mask/Bit helpers */
#define _SMB1351_MASK(BITS, POS) \
	((unsigned char)(((1 << (BITS)) - 1) << (POS)))
#define SMB1351_MASK(LEFT_BIT_POS, RIGHT_BIT_POS) \
		_SMB1351_MASK((LEFT_BIT_POS) - (RIGHT_BIT_POS) + 1, \
				(RIGHT_BIT_POS))

/* Configuration registers */
#define CHG_CURRENT_CTRL_REG			0x0
#define FAST_CHG_CURRENT_MASK			SMB1351_MASK(7, 4)
#define AC_INPUT_CURRENT_LIMIT_MASK		SMB1351_MASK(3, 0)

#define CHG_OTH_CURRENT_CTRL_REG		0x1
#define PRECHG_CURRENT_MASK			SMB1351_MASK(7, 5)
#define ITERM_MASK				SMB1351_MASK(4, 2)
#define USB_2_3_MODE_SEL_BIT			BIT(1)
#define USB_2_3_MODE_SEL_BY_I2C			0
#define USB_2_3_MODE_SEL_BY_PIN			0x2
#define USB_5_1_CMD_POLARITY_BIT		BIT(0)
#define USB_CMD_POLARITY_500_1_100_0		0
#define USB_CMD_POLARITY_500_0_100_1		0x1

#define VARIOUS_FUNC_REG			0x2
#define SUSPEND_MODE_CTRL_BIT			BIT(7)
#define SUSPEND_MODE_CTRL_BY_PIN		0
#define SUSPEND_MODE_CTRL_BY_I2C		0x80
#define BATT_TO_SYS_POWER_CTRL_BIT		BIT(6)
#define MAX_SYS_VOLTAGE				BIT(5)
#define AICL_EN_BIT				BIT(4)
#define AICL_DET_TH_BIT				BIT(3)
#define APSD_EN_BIT				BIT(2)
#define BATT_OV_BIT				BIT(1)
#define VCHG_FUNC_BIT				BIT(0)

#define VFLOAT_REG				0x3
#define PRECHG_TO_FAST_VOLTAGE_CFG_MASK		SMB1351_MASK(7, 6)
#define VFLOAT_MASK				SMB1351_MASK(5, 0)

#define CHG_CTRL_REG				0x4
#define AUTO_RECHG_BIT				BIT(7)
#define AUTO_RECHG_ENABLE			0
#define AUTO_RECHG_DISABLE			0x80
#define ITERM_EN_BIT				BIT(6)
#define ITERM_ENABLE				0
#define ITERM_DISABLE				0x40
#define MAPPED_AC_INPUT_CURRENT_LIMIT_MASK	SMB1351_MASK(5, 4)
#define AUTO_RECHG_TH_BIT			BIT(3)
#define AUTO_RECHG_TH_50MV			0
#define AUTO_RECHG_TH_100MV			0x8
#define AFCV_MASK				SMB1351_MASK(2, 0)

#define CHG_STAT_TIMERS_CTRL_REG		0x5
#define STAT_OUTPUT_POLARITY_BIT		BIT(7)
#define STAT_OUTPUT_ACTIVE_HIGH			BIT(7)
#define STAT_OUTPUT_ACTIVE_LOW			0
#define STAT_OUTPUT_MODE_BIT			BIT(6)
#define STAT_OUTPUT_CTRL_BIT			BIT(5)
#define OTH_CHG_IL_BIT				BIT(4)
#define COMPLETE_CHG_TIMEOUT_MASK		SMB1351_MASK(3, 2)
#define PRECHG_TIMEOUT_MASK			SMB1351_MASK(1, 0)

#define CHG_PIN_EN_CTRL_REG			0x6
#define LED_BLINK_FUNC_BIT			BIT(7)
#define EN_PIN_CTRL_MASK			SMB1351_MASK(6, 5)
#define EN_BY_I2C_0_DISABLE			0
#define EN_BY_I2C_0_ENABLE			0x20
#define EN_BY_PIN_HIGH_ENABLE			0x40
#define EN_BY_PIN_LOW_ENABLE			0x60
#define USBCS_CTRL_BIT				BIT(4)
#define USBCS_CTRL_BY_I2C			0
#define USBCS_CTRL_BY_PIN			0x10
#define USBCS_INPUT_STATE_BIT			BIT(3)
#define CHG_ERR_BIT				BIT(2)
#define APSD_DONE_BIT				BIT(1)
#define USB_FAIL_BIT				BIT(0)

#define THERM_A_CTRL_REG			0x7
#define MIN_SYS_VOLTAGE_MASK			SMB1351_MASK(7, 6)
#define LOAD_BATT_10MA_FVC_BIT			BIT(5)
#define THERM_MONITOR_BIT			BIT(4)
#define THERM_MONITOR_EN			0
#define SOFT_COLD_TEMP_LIMIT_MASK		SMB1351_MASK(3, 2)
#define SOFT_HOT_TEMP_LIMIT_MASK		SMB1351_MASK(1, 0)

#define WDOG_SAFETY_TIMER_CTRL_REG		0x8
#define AICL_FAIL_OPTION_BIT			BIT(7)
#define AICL_FAIL_TO_SUSPEND			0
#define AICL_FAIL_TO_150_MA			0x80
#define WDOG_TIMEOUT_MASK			SMB1351_MASK(6, 5)
#define WDOG_IRQ_SAFETY_TIMER_MASK		SMB1351_MASK(4, 3)
#define WDOG_IRQ_SAFETY_TIMER_EN_BIT		BIT(2)
#define WDOG_OPTION_BIT				BIT(1)
#define WDOG_TIMER_EN_BIT			BIT(0)

#define OTG_USBIN_AICL_CTRL_REG			0x9
#define OTG_ID_PIN_CTRL_MASK			SMB1351_MASK(7, 6)
#define OTG_PIN_POLARITY_BIT			BIT(5)
#define DCIN_IC_GLITCH_FILTER_HV_ADAPTER_MASK	SMB1351_MASK(4, 3)
#define DCIN_IC_GLITCH_FILTER_LV_ADAPTER_BIT	BIT(2)
#define USBIN_AICL_CFG1_BIT			BIT(1)
#define USBIN_AICL_CFG0_BIT			BIT(0)

#define OTG_TLIM_CTRL_REG			0xA
#define SWITCH_FREQ_MASK			SMB1351_MASK(7, 6)
#define SWITCH_FREQ_SHIFT			6
#define THERM_LOOP_TEMP_SEL_MASK		SMB1351_MASK(5, 4)
#define OTG_OC_LIMIT_MASK			SMB1351_MASK(3, 2)
#define OTG_BATT_UVLO_TH_MASK			SMB1351_MASK(1, 0)

#define HARD_SOFT_LIMIT_CELL_TEMP_REG		0xB
#define HARD_LIMIT_COLD_TEMP_ALARM_TRIP_MASK	SMB1351_MASK(7, 6)
#define HARD_LIMIT_HOT_TEMP_ALARM_TRIP_MASK	SMB1351_MASK(5, 4)
#define SOFT_LIMIT_COLD_TEMP_ALARM_TRIP_MASK	SMB1351_MASK(3, 2)
#define SOFT_LIMIT_HOT_TEMP_ALARM_TRIP_MASK	SMB1351_MASK(1, 0)

#define FAULT_INT_CFG_REG				0xC
#define HOT_COLD_HARD_LIMIT_BIT			BIT(7)
#define HOT_COLD_SOFT_LIMIT_BIT			BIT(6)
#define BATT_UVLO_IN_OTG_BIT			BIT(5)
#define OTG_OC_BIT				BIT(4)
#define INPUT_OVLO_BIT				BIT(3)
#define INPUT_UVLO_BIT				BIT(2)
#define AICL_DONE_FAIL_BIT			BIT(1)
#define INTERNAL_OVER_TEMP_BIT			BIT(0)

#define STATUS_INT_CFG_REG				0xD
#define CHG_OR_PRECHG_TIMEOUT_BIT		BIT(7)
#define RID_CHANGE_BIT				BIT(6)
#define BATT_OVP_BIT				BIT(5)
#define FAST_TERM_TAPER_RECHG_INHIBIT_BIT	BIT(4)
#define WDOG_TIMER_BIT				BIT(3)
#define POK_BIT					BIT(2)
#define BATT_MISSING_BIT			BIT(1)
#define BATT_LOW_BIT				BIT(0)

#define VARIOUS_FUNC_2_REG			0xE
#define CHG_HOLD_OFF_TIMER_AFTER_PLUGIN_BIT	BIT(7)
#define CHG_INHIBIT_BIT				BIT(6)
#define FAST_CHG_CC_IN_BATT_SOFT_LIMIT_MODE_BIT	BIT(5)
#define FVCL_IN_BATT_SOFT_LIMIT_MODE_MASK	SMB1351_MASK(4, 3)
#define HARD_TEMP_LIMIT_BEHAVIOR_BIT		BIT(2)
#define PRECHG_TO_FASTCHG_BIT			BIT(1)
#define STAT_PIN_CONFIG_BIT			BIT(0)

#define FLEXCHARGER_REG				0x10
#define AFVC_IRQ_BIT				BIT(7)
#define CHG_CONFIG_MASK				SMB1351_MASK(6, 4)
#define LOW_BATT_VOLTAGE_DET_TH_MASK		SMB1351_MASK(3, 0)

#define VARIOUS_FUNC_3_REG			0x11
#define SAFETY_TIMER_EN_MASK			SMB1351_MASK(7, 6)
#define BLOCK_SUSPEND_DURING_VBATT_LOW_BIT	BIT(5)
#define TIMEOUT_SEL_FOR_APSD_BIT		BIT(4)
#define SDP_SUSPEND_BIT				BIT(3)
#define QC_2P1_AUTO_INCREMENT_MODE_BIT		BIT(2)
#define QC_2P1_AUTH_ALGO_BIT			BIT(1)
#define DCD_EN_BIT				BIT(0)

#define HVDCP_BATT_MISSING_CTRL_REG		0x12
#define HVDCP_ADAPTER_SEL_MASK			SMB1351_MASK(7, 6)
#define HVDCP_EN_BIT				BIT(5)
#define HVDCP_AUTO_INCREMENT_LIMIT_BIT		BIT(4)
#define BATT_MISSING_ON_INPUT_PLUGIN_BIT	BIT(3)
#define BATT_MISSING_2P6S_POLLER_BIT		BIT(2)
#define BATT_MISSING_ALGO_BIT			BIT(1)
#define BATT_MISSING_THERM_PIN_SOURCE_BIT	BIT(0)

#define PON_OPTIONS_REG				0x13
#define SYSOK_INOK_POLARITY_BIT			BIT(7)
#define SYSOK_INOK_POLARITY_INVERT		BIT(7)
#define SYSOK_OPTIONS_MASK			SMB1351_MASK(6, 4)
#define SYSOK_INOK_OPTION1				0x00
#define INPUT_MISSING_POLLER_CONFIG_BIT		BIT(3)
#define VBATT_LOW_DISABLED_OR_RESET_STATE_BIT	BIT(2)
#define QC_2P1_AUTH_ALGO_IRQ_EN_BIT		BIT(0)

#define OTG_MODE_POWER_OPTIONS_REG		0x14
#define ADAPTER_CONFIG_MASK			SMB1351_MASK(7, 6)
#define MAP_HVDCP_BIT				BIT(5)
#define SDP_LOW_BATT_FORCE_USB5_OVER_USB1_BIT	BIT(4)
#define OTG_HICCUP_MODE_BIT			BIT(2)
#define INPUT_CURRENT_LIMIT_MASK		SMB1351_MASK(1, 0)

#define CHARGER_I2C_CTRL_REG			0x15
#define FULLON_MODE_EN_BIT			BIT(7)
#define I2C_HS_MODE_EN_BIT			BIT(6)
#define SYSON_LDO_OUTPUT_SEL_BIT		BIT(5)
#define VBATT_TRACKING_VOLTAGE_DIFF_BIT		BIT(4)
#define DISABLE_AFVC_WHEN_ENTER_TAPER_BIT	BIT(3)
#define VCHG_IINV_BIT				BIT(2)
#define AFVC_OVERRIDE_BIT			BIT(1)
#define SYSOK_PIN_CONFIG_BIT			BIT(0)

#define VERSION_REG				0x2E
#define VERSION_MASK				BIT(1)

/* Command registers */
#define CMD_I2C_REG				0x30
#define CMD_RELOAD_BIT				BIT(7)
#define CMD_BQ_CFG_ACCESS_BIT			BIT(6)

#define CMD_INPUT_LIMIT_REG			0x31
#define CMD_OVERRIDE_BIT			BIT(7)
#define CMD_SUSPEND_MODE_BIT			BIT(6)
#define CMD_INPUT_CURRENT_MODE_BIT		BIT(3)
#define CMD_INPUT_CURRENT_MODE_APSD		0
#define CMD_INPUT_CURRENT_MODE_CMD		0x08
#define CMD_USB_2_3_SEL_BIT			BIT(2)
#define CMD_USB_2_MODE				0
#define CMD_USB_3_MODE				0x4
#define CMD_USB_1_5_AC_CTRL_MASK		SMB1351_MASK(1, 0)
#define CMD_USB_100_MODE			0
#define CMD_USB_500_MODE			0x2
#define CMD_USB_HC_MODE				0x1

#define CMD_CHG_REG				0x32
#define CMD_DISABLE_THERM_MONITOR_BIT		BIT(4)
#define CMD_TURN_OFF_STAT_PIN_BIT		BIT(3)
#define CMD_PRE_TO_FAST_EN_BIT			BIT(2)
#define CMD_CHG_EN_BIT				BIT(1)
#define CMD_CHG_DISABLE				0
#define CMD_CHG_ENABLE				0x2
#define CMD_OTG_EN_BIT				BIT(0)

#define CMD_DEAD_BATT_REG			0x33
#define CMD_STOP_DEAD_BATT_TIMER_MASK		SMB1351_MASK(7, 0)

#define CMD_HVDCP_REG				0x34
#define CMD_APSD_RE_RUN_BIT			BIT(7)
#define CMD_FORCE_HVDCP_2P0_BIT			BIT(5)
#define CMD_HVDCP_MODE_MASK			SMB1351_MASK(5, 0)

/* Status registers */
#define STATUS_0_REG				0x36
#define STATUS_AICL_DONE_BIT				BIT(7)
#define STATUS_INPUT_MODE_MASK			SMB1351_MASK(6, 5)
#define STATUS_INPUT_SUSPEND			BIT(4)
#define STATUS_INPUT_CURRENT_LIMIT_MASK		SMB1351_MASK(3, 0)

#define STATUS_1_REG				0x37
#define STATUS_INPUT_RANGE_MASK			SMB1351_MASK(7, 4)
#define STATUS_INPUT_RANGE_12V			BIT(7)
#define STATUS_INPUT_RANGE_5V_9V		BIT(6)
#define STATUS_INPUT_RANGE_9V			BIT(5)
#define STATUS_INPUT_RANGE_5V			BIT(4)
#define STATUS_INPUT_USB_BIT			BIT(0)

#define STATUS_2_REG				0x38
#define STATUS_FAST_CHG_BIT			BIT(7)
#define STATUS_HARD_LIMIT_BIT			BIT(6)
#define STATUS_FLOAT_VOLTAGE_MASK		SMB1351_MASK(5, 0)

#define STATUS_3_REG				0x39
#define STATUS_CHG_BIT				BIT(7)
#define STATUS_PRECHG_CURRENT_MASK		SMB1351_MASK(6, 4)
#define STATUS_FAST_CHG_CURRENT_MASK		SMB1351_MASK(3, 0)

#define STATUS_4_REG				0x3A
#define STATUS_OTG_BIT				BIT(7)
#define STATUS_AFVC_BIT				BIT(6)
#define STATUS_DONE_BIT				BIT(5)
#define STATUS_BATT_LESS_THAN_2V_BIT		BIT(4)
#define STATUS_HOLD_OFF_BIT			BIT(3)
#define STATUS_CHG_MASK				SMB1351_MASK(2, 1)
#define STATUS_NO_CHARGING			0
#define STATUS_FAST_CHARGING			0x4
#define STATUS_PRE_CHARGING			0x2
#define STATUS_TAPER_CHARGING			0x6
#define STATUS_CHG_EN_STATUS_BIT		BIT(0)

#define STATUS_5_REG				0x3B
#define STATUS_SOURCE_DETECTED_MASK		SMB1351_MASK(7, 0)
#define STATUS_PORT_CDP				0x80
#define STATUS_PORT_DCP				0x40
#define STATUS_PORT_OTHER			0x20
#define STATUS_PORT_SDP				0x10
#define STATUS_PORT_ACA_A			0x8
#define STATUS_PORT_ACA_B			0x4
#define STATUS_PORT_ACA_C			0x2
#define STATUS_PORT_ACA_DOCK			0x1

#define STATUS_6_REG				0x3C
#define STATUS_DCD_TIMEOUT_BIT			BIT(7)
#define STATUS_DCD_GOOD_DG_BIT			BIT(6)
#define STATUS_OCD_GOOD_DG_BIT			BIT(5)
#define STATUS_RID_ABD_DG_BIT			BIT(4)
#define STATUS_RID_FLOAT_STATE_MACHINE_BIT	BIT(3)
#define STATUS_RID_A_STATE_MACHINE_BIT		BIT(2)
#define STATUS_RID_B_STATE_MACHINE_BIT		BIT(1)
#define STATUS_RID_C_STATE_MACHINE_BIT		BIT(0)

#define STATUS_7_REG				0x3D
#define STATUS_HVDCP_MASK			SMB1351_MASK(7, 0)
#define HVDCP_SEL_5V				BIT(1)
#define HVDCP_SEL_9V				BIT(2)
#define HVDCP_SEL_12V				BIT(3)

#define STATUS_8_REG				0x3E
#define STATUS_USNIN_HV_INPUT_SEL_BIT		BIT(5)
#define STATUS_USBIN_LV_UNDER_INPUT_SEL_BIT	BIT(4)
#define STATUS_USBIN_LV_INPUT_SEL_BIT		BIT(3)

/* Revision register */
#define CHG_REVISION_REG			0x3F
#define GUI_REVISION_MASK			SMB1351_MASK(7, 4)
#define DEVICE_REVISION_MASK			SMB1351_MASK(3, 0)

/* IRQ status registers */
#define IRQ_A_REG				0x40
#define IRQ_HOT_HARD_BIT			BIT(6)
#define IRQ_COLD_HARD_BIT			BIT(4)
#define IRQ_HOT_SOFT_BIT			BIT(2)
#define IRQ_COLD_SOFT_BIT			BIT(0)

#define IRQ_B_REG				0x41
#define IRQ_BATT_TERMINAL_REMOVED_BIT		BIT(6)
#define IRQ_BATT_MISSING_BIT			BIT(4)
#define IRQ_LOW_BATT_VOLTAGE_BIT		BIT(2)
#define IRQ_INTERNAL_TEMP_LIMIT_BIT		BIT(0)

#define IRQ_C_REG				0x42
#define IRQ_PRE_TO_FAST_VOLTAGE_BIT		BIT(6)
#define IRQ_RECHG_BIT				BIT(4)
#define IRQ_TAPER_BIT				BIT(2)
#define IRQ_TERM_BIT				BIT(0)

#define IRQ_D_REG				0x43
#define IRQ_BATT_OV_BIT				BIT(6)
#define IRQ_CHG_ERROR_BIT			BIT(4)
#define IRQ_CHG_TIMEOUT_BIT			BIT(2)
#define IRQ_PRECHG_TIMEOUT_BIT			BIT(0)

#define IRQ_E_REG				0x44
#define IRQ_USBIN_OV_BIT			BIT(6)
#define IRQ_USBIN_UV_BIT			BIT(4)
#define IRQ_AFVC_BIT				BIT(2)
#define IRQ_POWER_OK_BIT			BIT(0)

#define IRQ_F_REG				0x45
#define IRQ_OTG_OVER_CURRENT_BIT		BIT(6)
#define IRQ_OTG_FAIL_BIT			BIT(4)
#define IRQ_RID_BIT				BIT(2)
#define IRQ_OTG_OC_RETRY_BIT			BIT(0)

#define IRQ_G_REG				0x46
#define IRQ_SOURCE_DET_BIT			BIT(6)
#define IRQ_AICL_DONE_BIT			BIT(4)
#define IRQ_AICL_FAIL_BIT			BIT(2)
#define IRQ_CHG_INHIBIT_BIT			BIT(0)

#define IRQ_H_REG				0x47
#define IRQ_IC_LIMIT_STATUS_BIT			BIT(5)
#define IRQ_HVDCP_2P1_STATUS_BIT		BIT(4)
#define IRQ_HVDCP_AUTH_DONE_BIT			BIT(2)
#define IRQ_WDOG_TIMEOUT_BIT			BIT(0)

/* constants */
#define USB2_MIN_CURRENT_MA			100
#define USB2_MAX_CURRENT_MA			500
#define USB3_MIN_CURRENT_MA			150
#define USB3_MAX_CURRENT_MA			900
#define SMB1351_IRQ_REG_COUNT			8
#define SMB1351_CHG_PRE_MIN_MA			100
#define SMB1351_CHG_FAST_MIN_MA			1000
#define SMB1351_CHG_FAST_MAX_MA			5000
#define SMB1351_CHG_PRE_SHIFT			5
#define SMB1351_CHG_FAST_SHIFT			4
#define DEFAULT_BATT_CAPACITY			50
#define DEFAULT_BATT_TEMP			250
#define DEFAULT_BATT_VOLTAGE			3800
#define DEFAULT_VBUS_VOLTAGE			4000
#define SUSPEND_CURRENT_MA			2

#define CHG_ITERM_200MA				0x0
#define CHG_ITERM_300MA				0x04
#define CHG_ITERM_400MA				0x08
#define CHG_ITERM_500MA				0x0C
#define CHG_ITERM_600MA				0x10
#define CHG_ITERM_700MA				0x14

#define ADC_TM_WARM_COOL_THR_ENABLE		ADC_TM_HIGH_LOW_THR_ENABLE

enum reason {
	USER	= BIT(0),
	THERMAL = BIT(1),
	CURRENT = BIT(2),
	SOC	= BIT(3),
	PARALLEL = BIT(4),
	ITERM	= BIT(5),
};

static char *pm_batt_supplied_to[] = {
	"bms",
};

struct smb1351_regulator {
	struct regulator_desc	rdesc;
	struct regulator_dev	*rdev;
};

enum chip_version {
	SMB_UNKNOWN = 0,
	SMB1350,
	SMB1351,
	SMB_MAX_TYPE,
};

static const char *smb1351_version_str[SMB_MAX_TYPE] = {
	[SMB_UNKNOWN] = "Unknown",
	[SMB1350] = "SMB1350",
	[SMB1351] = "SMB1351",
};

enum workaround_flags {
	CHG_FORCE_ESR_WA	= BIT(0),
	CHG_FG_NOTIFY_JEITA	= BIT(1),
	CHG_ITERM_CHECK_SOC	= BIT(2),
	CHG_SOC_BASED_RESUME	= BIT(3),
};

enum wakeup_src {
	I2C_ACCESS,
	HVDCP_DETECT,
	REMOVAL_DETECT,
	RERUN_APSD,
	PARALLEL_CHARGE,
	PARALLEL_TAPER,
	FORCE_ESR_PULSE,
	ITERM_CHECK_SOC,
	WAKEUP_SRC_MAX,
};
#define WAKEUP_SRC_MASK GENMASK(WAKEUP_SRC_MAX, 0)

struct smb1351_wakeup_source {
	struct wakeup_source source;
	unsigned long enabled_bitmap;
	spinlock_t ws_lock;
};

/* parallel primary charger */
struct parallel_main_cfg {
	struct power_supply	*psy;
	struct mutex		lock;

	bool			avail;
	bool			slave_detected;
	int			min_current_thr_ma;
	int			min_9v_current_thr_ma;
	int			min_12v_current_thr_ma;
	int			allowed_lowering_ma;
	int			main_chg_fcc_percent;
	int			main_chg_icl_percent;
	int			total_icl_ma;
	int			slave_icl_ma;

	struct delayed_work	parallel_work;
};

struct smb1351_charger {
	struct i2c_client	*client;
	struct device		*dev;

	bool			recharge_disabled;
	int			recharge_mv;
	bool			iterm_disabled;
	int			iterm_ma;
	int			vfloat_mv;
	int			switch_freq;
	int			chg_present;
	int			fake_battery_soc;
	int			resume_soc;
	bool			chg_autonomous_mode;
	bool			disable_apsd;
	bool			using_pmic_therm;
	bool			jeita_supported;
	bool			fg_notify_jeita;
	bool			battery_missing;
	const char		*bms_psy_name;
	bool			resume_completed;
	bool			irq_waiting;
	struct delayed_work	chg_remove_work;
	struct delayed_work	hvdcp_det_work;
	struct delayed_work	rerun_apsd_work;
	struct delayed_work	init_fg_work;
	struct delayed_work	iterm_check_soc_work;
	struct smb1351_wakeup_source	smb1351_ws;
	struct delayed_work charger_check_work;

	/* status tracking */
	bool			batt_full;
	bool			batt_hot;
	bool			batt_cold;
	bool			batt_warm;
	bool			batt_cool;

	int			battchg_disabled_status;
	int			usb_suspended_status;
	int			target_fastchg_current_max_ma;
	int			fastchg_current_max_ma;
	int			main_fcc_ma_before_esr;
	int			slave_fcc_ma_before_esr;
	int			workaround_flags;

	int			parallel_pin_polarity_setting;
	bool			is_slave;
	bool			use_external_fg;
	bool			parallel_charger_present;
	bool			bms_controlled_charging;
	bool			apsd_rerun;
	bool			usbin_ov;
	int			chg_remove_work_scheduled;
	bool			force_hvdcp_2p0;
	bool			check_parallel;
	bool			in_esr_pulse;
	enum chip_version	version;
	u32			wa_flags;
	int			pre_soc;

	/* psy */
	struct power_supply	*usb_psy;
	int			usb_psy_ma;
	enum power_supply_type	usb_psy_type;
	struct power_supply	*bms_psy;
	struct power_supply	batt_psy;
	struct power_supply	parallel_psy;
#ifdef EXT_CHARGER_POWER_SUPPLY
	struct power_supply	ext_charger_psy;
	bool temp_debug_flag;
#endif
#ifdef CHG_HEATBEAT_WORK
	struct delayed_work	heatbeat_work;
#endif

	bool handshake_state;
	bool fast_charger_is_engineermode;

	struct smb1351_regulator	otg_vreg;
	struct mutex		irq_complete;
	struct mutex		fcc_lock;
	struct mutex		usb_suspend_status_lock;
	struct wake_lock charging_lock;

	struct dentry		*debug_root;
	u32			peek_poke_address;

	/* adc_tm parameters */
	struct qpnp_vadc_chip	*vadc_dev;
	struct qpnp_adc_tm_chip	*adc_tm_dev;
	struct qpnp_adc_tm_btm_param	adc_param;

	/* jeita parameters */
	int			batt_hot_decidegc;
	int			batt_cold_decidegc;
	int			batt_warm_decidegc;
	int			batt_cool_decidegc;
	int			batt_missing_decidegc;
	unsigned int		batt_warm_ma;
	unsigned int		batt_warm_mv;
	unsigned int		batt_cool_ma;
	unsigned int		batt_cool_mv;

	/* pinctrl parameters */
	const char		*pinctrl_state_name;
	struct pinctrl		*smb_pinctrl;

	/* parallel primary */
	struct parallel_main_cfg	parallel;

	struct regulator *vdd;
	int chg_en_gpio;
	int chg_led_gpio;
	int batt_temp;

	struct qpnp_vadc_chip *chg_pm_vadc_dev;
	struct qpnp_vadc_chip *chg_pmi_vadc_dev;
	bool is_factory_mode;
	bool is_factory_cable;
	bool is_factory_need_write;
	bool is_in_esr_state;
	bool is_hv_fast_charging;

	struct notifier_block	mmi_reboot;

#ifdef LENOVO_OTG_USB_SHORT
	int otg_usb_short_gpio;
	bool otg_usb_short_state;
	bool is_otg_on;
#endif
};

struct smb_irq_info {
	const char		*name;
	int (*smb_irq)(struct smb1351_charger *chip, u8 rt_stat);
	int			high;
	int			low;
};

struct irq_handler_info {
	u8			stat_reg;
	u8			val;
	u8			prev_val;
	struct smb_irq_info	irq_info[4];
};

/* USB input charge current */
static int usb_chg_current[] = {
	500, 685, 1000, 1100, 1200, 1300, 1500, 1600,
	1700, 1800, 2000, 2200, 2500, 3000,
};

static int fast_chg_current[] = {
	1000, 1200, 1400, 1600, 1800, 2000, 2200,
	2400, 2600, 2800, 3000, 3400, 3600, 3800,
	4000, 4640,
};

static int pre_chg_current[] = {
	200, 300, 400, 500, 600, 700,
};

struct battery_status {
	bool			batt_hot;
	bool			batt_warm;
	bool			batt_cool;
	bool			batt_cold;
	bool			batt_present;
};

enum {
	BATT_HOT = 0,
	BATT_WARM,
	BATT_NORMAL,
	BATT_COOL,
	BATT_COLD,
	BATT_MISSING,
	BATT_STATUS_MAX,
};

static struct battery_status batt_s[] = {
	[BATT_HOT] = {1, 0, 0, 0, 1},
	[BATT_WARM] = {0, 1, 0, 0, 1},
	[BATT_NORMAL] = {0, 0, 0, 0, 1},
	[BATT_COOL] = {0, 0, 1, 0, 1},
	[BATT_COLD] = {0, 0, 0, 1, 1},
	[BATT_MISSING] = {0, 0, 0, 1, 0},
};

static int smb1351_led_operation(struct smb1351_charger *chip, int en)
{
	pr_info("set led stat %d\n", en);

	gpio_set_value(chip->chg_led_gpio, en);
	return 0;
}

static int smb1351_led_state(struct smb1351_charger *chip)
{
	return gpio_get_value(chip->chg_led_gpio);
}

static void smb1351_stay_awake(struct smb1351_wakeup_source *source,
					enum wakeup_src wk_src)
{
	unsigned long flags;

	spin_lock_irqsave(&source->ws_lock, flags);

	if (!__test_and_set_bit(wk_src, &source->enabled_bitmap))
		__pm_stay_awake(&source->source);

	spin_unlock_irqrestore(&source->ws_lock, flags);
}

static void smb1351_relax(struct smb1351_wakeup_source *source,
					enum wakeup_src wk_src)
{
	unsigned long flags;

	spin_lock_irqsave(&source->ws_lock, flags);
	if (__test_and_clear_bit(wk_src, &source->enabled_bitmap) &&
		!(source->enabled_bitmap & WAKEUP_SRC_MASK)) {
		__pm_relax(&source->source);
	}
	spin_unlock_irqrestore(&source->ws_lock, flags);

}

static void smb1351_wakeup_src_init(struct smb1351_charger *chip)
{
	spin_lock_init(&chip->smb1351_ws.ws_lock);
	wakeup_source_init(&chip->smb1351_ws.source, "smb1351");
}

static int smb1351_read_reg(struct smb1351_charger *chip, int reg, u8 *val)
{
	s32 ret;

	smb1351_stay_awake(&chip->smb1351_ws, I2C_ACCESS);
	ret = i2c_smbus_read_byte_data(chip->client, reg);
	if (ret < 0) {
		pr_err("%s i2c read fail: can't read from %02x: %d\n", dev_name(chip->dev), reg, ret);
		smb1351_relax(&chip->smb1351_ws, I2C_ACCESS);
		return ret;
	} else {
		*val = ret;
	}
	smb1351_relax(&chip->smb1351_ws, I2C_ACCESS);
//	pr_debug("%s Reading 0x%02x=0x%02x\n", dev_name(chip->dev), reg, *val);
	return 0;
}

static int smb1351_write_reg(struct smb1351_charger *chip, int reg, u8 val)
{
	s32 ret;

	smb1351_stay_awake(&chip->smb1351_ws, I2C_ACCESS);
	ret = i2c_smbus_write_byte_data(chip->client, reg, val);
	if (ret < 0) {
		pr_err("%s, i2c write fail: can't write %02x to %02x: %d\n",
			dev_name(chip->dev), val, reg, ret);
		smb1351_relax(&chip->smb1351_ws, I2C_ACCESS);
		return ret;
	}
	smb1351_relax(&chip->smb1351_ws, I2C_ACCESS);
//	pr_debug("%s Writing 0x%02x=0x%02x\n", dev_name(chip->dev), reg, val);
	return 0;
}

static int smb1351_masked_write(struct smb1351_charger *chip, int reg,
							u8 mask, u8 val)
{
	s32 rc;
	u8 temp;

	rc = smb1351_read_reg(chip, reg, &temp);
	if (rc) {
		pr_err("read failed: reg=%03X, rc=%d\n", reg, rc);
		return rc;
	}
	temp &= ~mask;
	temp |= val & mask;
	rc = smb1351_write_reg(chip, reg, temp);
	if (rc) {
		pr_err("write failed: reg=%03X, rc=%d\n", reg, rc);
		return rc;
	}
	return 0;
}

static int smb1351_enable_volatile_writes(struct smb1351_charger *chip)
{
	int rc;

	rc = smb1351_masked_write(chip, CMD_I2C_REG, CMD_BQ_CFG_ACCESS_BIT,
							CMD_BQ_CFG_ACCESS_BIT);
	if (rc)
		pr_err("Couldn't write CMD_BQ_CFG_ACCESS_BIT rc=%d\n", rc);

	return rc;
}

static int smb1351_usb_suspend(struct smb1351_charger *chip, int reason,
					bool suspend)
{
	int rc = 0;
	int suspended;

	mutex_lock(&chip->usb_suspend_status_lock);
	suspended = chip->usb_suspended_status;

	pr_debug("%s reason = %d requested_suspend = %d suspended_status = %d\n",
						dev_name(chip->dev), reason, suspend, suspended);

	if (suspend == false)
		suspended &= ~reason;
	else
		suspended |= reason;

	pr_debug("new suspended_status = %d\n", suspended);

	rc = smb1351_masked_write(chip, CMD_INPUT_LIMIT_REG,
				CMD_SUSPEND_MODE_BIT,
				suspended ? CMD_SUSPEND_MODE_BIT : 0);
	if (rc)
		pr_err("Couldn't suspend rc = %d\n", rc);
	else
		chip->usb_suspended_status = suspended;

	mutex_unlock(&chip->usb_suspend_status_lock);
	return rc;
}

static int smb1351_battchg_disable(struct smb1351_charger *chip,
					int reason, int disable)
{
	int rc = 0;
	int disabled;

	if (chip->chg_autonomous_mode) {
		pr_debug("Charger in autonomous mode\n");
		return 0;
	}

	disabled = chip->battchg_disabled_status;

	pr_debug("%s reason = %d requested_disable = %d disabled_status = %d\n",
						dev_name(chip->dev), reason, disable, disabled);
	if (disable == true)
		disabled |= reason;
	else
		disabled &= ~reason;

	pr_debug("new disabled_status = %d\n", disabled);

	rc = smb1351_masked_write(chip, CMD_CHG_REG, CMD_CHG_EN_BIT,
					disabled ? 0 : CMD_CHG_ENABLE);
	if (rc)
		pr_err("Couldn't %s charging rc=%d\n",
					disable ? "disable" : "enable", rc);
	else
		chip->battchg_disabled_status = disabled;

	return rc;
}

static int smb1351_parallel_charger_disable_slave(struct smb1351_charger *chip);
static int smb1351_set_hv_handshake_current(struct smb1351_charger *chip,
						int current_ma)
{
	int rc = 0;

	pr_debug("handshake current_ma = %d\n", current_ma);
	rc = smb1351_masked_write(chip, CMD_INPUT_LIMIT_REG, 0x2, (current_ma>100)?0x2:0x0);
	if (rc) {
		pr_err("Set 0x31 1 failed, rc=%d\n", rc);
		return rc;
	}

	return rc;
}

#ifdef MMI_TEST

#define CHG_SHOW_MAX_SIZE 50

static int factory_kill_disable;

module_param(factory_kill_disable, int, 0644);

/*motorola mmi test use*/
static ssize_t force_chg_fail_clear_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	unsigned long r;
	unsigned long mode;

	r = kstrtoul(buf, 0, &mode);
	if (r) {
		pr_err("Invalid chg fail mode value = %lu\n", mode);
		return -EINVAL;
	}

	/* do nothing for SMBCHG */
	r = 0;

	return r ? r : count;
}

static ssize_t force_chg_fail_clear_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	/* do nothing for SMBCHG */
	return scnprintf(buf, CHG_SHOW_MAX_SIZE, "0\n");
}

static DEVICE_ATTR(force_chg_fail_clear, 0664,
		force_chg_fail_clear_show,
		force_chg_fail_clear_store);
#endif

#define INPUT_MODE_HC		0x00
#define INPUT_MODE_USB100	(0x01 << 5)
#define INPUT_MODE_USB500	(0x02 << 5)
static int smb1351_get_usb_chg_current(struct smb1351_charger *chip,
							int *icl_ma)
{
	int rc, i;
	bool is_usb3;
	u8 icl_status, usb_mode;

	rc = smb1351_read_reg(chip, STATUS_0_REG, &icl_status);
	if (rc) {
		pr_err("read STATUS_0 failed, rc=%d\n", rc);
		return rc;
	}
	if (icl_status & STATUS_INPUT_SUSPEND) {
		pr_err("%s:USB suspended!\n", dev_name(chip->dev));
		*icl_ma = 0;
		return rc;
	}
	rc = smb1351_read_reg(chip, CMD_INPUT_LIMIT_REG, &usb_mode);
	if (rc) {
		pr_err("read CMD_IL failed, rc=%d\n", rc);
		return rc;
	}

	is_usb3 = !!(usb_mode & CMD_USB_2_3_SEL_BIT);
	switch (icl_status & STATUS_INPUT_MODE_MASK) {
	case INPUT_MODE_USB100:
		if (is_usb3)
			*icl_ma = USB3_MIN_CURRENT_MA;
		else
			*icl_ma = USB2_MIN_CURRENT_MA;
		break;
	case INPUT_MODE_USB500:
		if (is_usb3)
			*icl_ma = USB3_MAX_CURRENT_MA;
		else
			*icl_ma = USB2_MAX_CURRENT_MA;
		break;
	case INPUT_MODE_HC:
		i = icl_status & STATUS_INPUT_CURRENT_LIMIT_MASK;
		if (i >= ARRAY_SIZE(usb_chg_current))
			i = ARRAY_SIZE(usb_chg_current) - 1;
		*icl_ma = usb_chg_current[i];
		break;
	default:
		break;
	}
	pr_debug("%s:USB ICL status: %d\n", dev_name(chip->dev), *icl_ma);

	return rc;
}

static int smb1351_set_usb_chg_current(struct smb1351_charger *chip,
						int current_ma)
{
	int i, rc = 0, icl_result_ma = 0;
	u8 reg = 0, mask = 0;

	pr_debug("%s:USB current_ma = %d\n", dev_name(chip->dev), current_ma);

	if (chip->chg_autonomous_mode) {
		pr_debug("Charger in autonomous mode\n");
		return 0;
	}

#ifdef APP_CONTROL
	if (!chip->is_slave) {
		if ((current_ma < 500)&&(current_ma >= 100)) {
			smb1351_set_hv_handshake_current(chip, current_ma);
			return 0;
		} else if(chip->handshake_state) {
			pr_info ("in handshake state, do not set usb in (%d)\n", current_ma);
			return 0;
		}
	}
#endif


	/* set suspend bit when urrent_ma <= 2 */
	if (current_ma <= SUSPEND_CURRENT_MA) {
		if(chip->is_slave) {
			smb1351_usb_suspend(chip, CURRENT, true);
			pr_err("USB suspend\n");
		}
		return 0;
	}

	if (current_ma > SUSPEND_CURRENT_MA &&
			current_ma < USB2_MIN_CURRENT_MA)
		current_ma = USB2_MIN_CURRENT_MA;

	if (current_ma == USB2_MIN_CURRENT_MA) {
		/* USB 2.0 - 100mA */
		reg = CMD_USB_2_MODE | CMD_USB_100_MODE;
	} else if (current_ma == USB3_MIN_CURRENT_MA) {
		/* USB 3.0 - 150mA */
		reg = CMD_USB_3_MODE | CMD_USB_100_MODE;
	} else if (current_ma == USB2_MAX_CURRENT_MA) {
		/* USB 2.0 - 500mA */
		reg = CMD_USB_2_MODE | CMD_USB_500_MODE;
	} else if (current_ma == USB3_MAX_CURRENT_MA) {
		/* USB 3.0 - 900mA */
		reg = CMD_USB_3_MODE | CMD_USB_500_MODE;
	} else if (current_ma > USB2_MAX_CURRENT_MA) {
		/* HC mode  - if none of the above */
		reg = CMD_USB_HC_MODE;
		rc = smb1351_get_usb_chg_current(chip, &icl_result_ma);
		if (rc) {
			pr_err("Get ICL result failed, rc=%d\n", rc);
			return rc;
		}

		for (i = ARRAY_SIZE(usb_chg_current) - 1; i >= 0; i--) {
			if (usb_chg_current[i] <= current_ma)
				break;
		}

		if (i < 0)
			i = 0;

		rc = smb1351_masked_write(chip, CHG_CURRENT_CTRL_REG,
						AC_INPUT_CURRENT_LIMIT_MASK, i);
		if (rc) {
			pr_err("Couldn't set input mA rc=%d\n", rc);
			return rc;
		}
		current_ma = usb_chg_current[i];
	}

	/* control input current mode by command */
	reg |= CMD_INPUT_CURRENT_MODE_CMD;
	mask = CMD_INPUT_CURRENT_MODE_BIT | CMD_USB_2_3_SEL_BIT |
		CMD_USB_1_5_AC_CTRL_MASK;
	rc = smb1351_masked_write(chip, CMD_INPUT_LIMIT_REG, mask, reg);
	if (rc) {
		pr_err("Couldn't set charging mode rc = %d\n", rc);
		return rc;
	}

	/* unset the suspend bit here */
	smb1351_usb_suspend(chip, CURRENT, false);
	/*
	 * AICL could be rerun if set a larger ICL
	 * and force a USB500 then USBAC mode.
	 */
	if ((icl_result_ma < current_ma) && (reg & CMD_USB_HC_MODE)) {
		rc = smb1351_masked_write(chip, CMD_INPUT_LIMIT_REG, mask,
				CMD_USB_2_MODE | CMD_USB_500_MODE |
				CMD_INPUT_CURRENT_MODE_CMD);
		if (rc) {
			pr_err("Set USB500 for AICL rerun failed, rc=%d\n", rc);
			return rc;
		}
		rc = smb1351_masked_write(chip, CMD_INPUT_LIMIT_REG, mask, reg);
		if (rc) {
			pr_err("Set USBAC for AICL rerun failed, rc=%d\n", rc);
			return rc;
		}
	}

	return rc;
}

static int smb1351_fastchg_current_set(struct smb1351_charger *chip,
					unsigned int fastchg_current)
{
	int i, rc;
	bool is_pre_chg = false;

	mutex_lock(&chip->fcc_lock);
	if (fastchg_current < SMB1351_CHG_PRE_MIN_MA)
		fastchg_current = SMB1351_CHG_PRE_MIN_MA;

	if (fastchg_current > SMB1351_CHG_FAST_MAX_MA)
		fastchg_current = SMB1351_CHG_FAST_MAX_MA;

	pr_debug("set fastchg current mA=%d\n", fastchg_current);

	/*
	 * fast chg current could not support less than 1000mA
	 * use pre chg to instead for the parallel charging
	 */
	if (fastchg_current < SMB1351_CHG_FAST_MIN_MA) {
		is_pre_chg = true;
		pr_debug("is_pre_chg true, current is %d\n", fastchg_current);
	}

	if (is_pre_chg) {
		/* set prechg current */
		for (i = ARRAY_SIZE(pre_chg_current) - 1; i >= 0; i--) {
			if (pre_chg_current[i] <= fastchg_current)
				break;
		}
		if (i < 0)
			i = 0;
		chip->fastchg_current_max_ma = pre_chg_current[i];
		pr_debug("prechg setting %02x\n", i);

		i = i << SMB1351_CHG_PRE_SHIFT;

		rc = smb1351_masked_write(chip, CHG_OTH_CURRENT_CTRL_REG,
				PRECHG_CURRENT_MASK, i);
		if (rc)
			pr_err("Couldn't write CHG_OTH_CURRENT_CTRL_REG rc=%d\n",
									rc);

		rc = smb1351_masked_write(chip, VARIOUS_FUNC_2_REG,
				PRECHG_TO_FASTCHG_BIT, PRECHG_TO_FASTCHG_BIT);
		if (rc)
			pr_err("Write VARIOUS_FUNC_2_REG failed, rc=%d\n", rc);
	} else {
		if (chip->version == SMB_UNKNOWN) {
			rc = -EINVAL;
			goto done;
		}

		/* SMB1350 supports FCC upto 2600 mA */
		if (chip->version == SMB1350 && fastchg_current > 2600)
			fastchg_current = 2600;

		/* set fastchg current */
		for (i = ARRAY_SIZE(fast_chg_current) - 1; i >= 0; i--) {
			if (fast_chg_current[i] <= fastchg_current)
				break;
		}
		if (i < 0)
			i = 0;
		chip->fastchg_current_max_ma = fast_chg_current[i];

		i = i << SMB1351_CHG_FAST_SHIFT;
		pr_debug("fastchg limit=%d setting %02x\n",
					chip->fastchg_current_max_ma, i);

		/* make sure pre chg mode is disabled */
		rc = smb1351_masked_write(chip, VARIOUS_FUNC_2_REG,
					PRECHG_TO_FASTCHG_BIT, 0);
		if (rc)
			pr_err("Couldn't write VARIOUS_FUNC_2_REG rc=%d\n", rc);

		rc = smb1351_masked_write(chip, CHG_CURRENT_CTRL_REG,
					FAST_CHG_CURRENT_MASK, i);
		if (rc)
			pr_err("Write CURRENT_CTRL_REG failed, rc=%d\n", rc);
	}

done:
	mutex_unlock(&chip->fcc_lock);
	return rc;
}

static int smb1351_fastchg_hv_current_set(struct smb1351_charger *chip,
					unsigned int fastchg_current)
{
	int rc;

	pr_info("set hv fastchg current mA=%d-target fast max %d\n", fastchg_current, chip->target_fastchg_current_max_ma);

	/*
	 * fast chg current could not support less than 1000mA
	 * use pre chg to instead for the parallel charging
	 */

#ifdef APP_CONTROL
	if(fastchg_current>2000) {
		pr_info("current(%d)>2000, using parallel charger\n", fastchg_current);
		chip->target_fastchg_current_max_ma = fastchg_current;
		rc = 0;

		schedule_delayed_work(&chip->parallel.parallel_work, (HZ*1));
	}else {
		smb1351_parallel_charger_disable_slave(chip);
		smb1351_fastchg_current_set(chip, fastchg_current);
	}
#else
	rc = 0;
#endif

	return rc;
}

#define charger_enabled(chip, sleep_time)  {smb1351_set_hv_handshake_current(chip,500);msleep(sleep_time);}
#define charger_disabled(chip, sleep_time) {smb1351_set_hv_handshake_current(chip,100);msleep(sleep_time);}
int smb1351_hv_handshake(struct smb1351_charger *chip)
{
	int i;
	int ret = 0;
	int rc = 0;

	pr_info("%s\n", __func__);

	smb1351_fastchg_current_set(chip, 1000);
	smb1351_set_usb_chg_current(chip, 500);

	//mutex_lock(&chip->handshake_lock);
	for(i=0;i<3;i++) {
		charger_enabled(chip, 1000);
		charger_disabled(chip, 100);
		charger_enabled(chip, 100);
		charger_disabled(chip, 100);

		charger_enabled(chip, 100);
		charger_disabled(chip, 100);
		charger_enabled(chip, 300);
		charger_disabled(chip, 100);
		charger_enabled(chip, 300);
		charger_disabled(chip, 100);
		charger_enabled(chip, 300);
		charger_disabled(chip, 100);
		charger_enabled(chip, 500);
		charger_disabled(chip, 150);
		//charger_enabled(chip, 500);
		//msleep(100);
	}
	//mutex_unlock(&chip->handshake_lock);

	rc = smb1351_masked_write(chip, CMD_INPUT_LIMIT_REG, BIT(0), CMD_USB_HC_MODE);
	if (rc) {
		pr_err("Set 0x31 0 failed, rc=%d\n", rc);
		return rc;
	}

	return ret;
}

enum TEMP_TYPE {
	BOARD_TEMP = 0,
	CHARGE_TEMP,
};

static int smb1351_get_board_temp_voltage(struct smb1351_charger *chip, enum TEMP_TYPE type)
{
	int rc = 0;
	struct qpnp_vadc_result results;
	int chan;

	if((chip == NULL)||(chip->chg_pm_vadc_dev == NULL))
		return -1;

	switch (type) {
		case BOARD_TEMP:
			chan = P_MUX4_1_1;
			break;
		case CHARGE_TEMP:
			chan = P_MUX2_1_1;
			break;
		default:
			pr_err("get ad type error %d\n", type);
			return -2;
	}

	rc = qpnp_vadc_read(chip->chg_pm_vadc_dev, chan, &results);
	if (rc) {
		pr_err("Unable to read tboard rc=%d\n", rc);
		return -1;
	}

	//pr_debug("get tboard ret2=%lld(%d)\n", results.physical, (int)(results.physical));

	return (int)results.physical;
}

static int smb1351_get_board_temp(struct smb1351_charger *chip, enum TEMP_TYPE type)
{
	int i;
	const int cnt_num = 6;
	int sum, avr;
	int max_val, min_val;
	int ret;

	sum = 0;
	//get
	for(i=0;i<cnt_num;i++) {
		ret = smb1351_get_board_temp_voltage(chip, type);
		if (ret<0) {
			pr_err("Unable to read batt temperature rc=%d\n", ret);
			return -1;
		}

		//pr_debug(" cnt %d  vol = %d\n",  i,ret);
		if(i==0) {
			max_val = ret;
			min_val = ret;
		} else {
			if(ret>max_val)
				max_val = ret;
			else if(ret<min_val)
				min_val = ret;
		}
		sum += ret;

		msleep(1);
	}

	//sum
	sum = sum - min_val - max_val;
	avr = sum /(cnt_num-2);

	return avr;
}

static int smb1351_get_vbus_voltage(struct smb1351_charger *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;

	if((chip == NULL)||(chip->chg_pmi_vadc_dev == NULL))
		return -1;

	rc = qpnp_vadc_read(chip->chg_pmi_vadc_dev, P_MUX1_1_1, &results);
	if (rc) {
		pr_err("Unable to read tboard rc=%d\n", rc);
		return -1;
	}

	//pr_debug("get vbus ret2=%lld(%d)\n", results.physical, (int)(results.physical));

	return ((int)results.physical) * VBUS_DIV_FACTOR / 3;
}

#define MIN_FLOAT_MV		3500
#define MAX_FLOAT_MV		4500
#define VFLOAT_STEP_MV		20
static int smb1351_float_voltage_set(struct smb1351_charger *chip,
								int vfloat_mv)
{
	u8 temp;

	if ((vfloat_mv < MIN_FLOAT_MV) || (vfloat_mv > MAX_FLOAT_MV)) {
		pr_err("bad float voltage mv =%d asked to set\n", vfloat_mv);
		return -EINVAL;
	}

	temp = (vfloat_mv - MIN_FLOAT_MV) / VFLOAT_STEP_MV;

	return smb1351_masked_write(chip, VFLOAT_REG, VFLOAT_MASK, temp);
}

static int smb1351_chg_set_appropriate_battery_current(
				struct smb1351_charger *chip)
{
	int rc;
	unsigned int current_max = chip->target_fastchg_current_max_ma;

	if (chip->batt_cool)
		current_max = min(current_max, chip->batt_cool_ma);
	if (chip->batt_warm)
		current_max = min(current_max, chip->batt_warm_ma);

	pr_debug("setting %dmA", current_max);

	rc = smb1351_fastchg_current_set(chip, current_max);
	if (rc)
		pr_err("Couldn't set charging current rc = %d\n", rc);

	return rc;
}

static int smb1351_chg_set_appropriate_vfloat(struct smb1351_charger *chip)
{
	int rc;
	unsigned int vfloat = chip->vfloat_mv;

	if (chip->batt_cool)
		vfloat = min(vfloat, chip->batt_cool_mv);
	if (chip->batt_warm)
		vfloat = min(vfloat, chip->batt_warm_mv);

	pr_debug("setting %dmV\n", vfloat);

	rc = smb1351_float_voltage_set(chip, vfloat);
	if (rc)
		pr_err("Couldn't set float voltage rc = %d\n", rc);

	return rc;
}

static int smb1351_iterm_set(struct smb1351_charger *chip, int iterm_ma)
{
	int rc;
	u8 reg;

	if (iterm_ma <= 200)
		reg = CHG_ITERM_200MA;
	else if (iterm_ma <= 300)
		reg = CHG_ITERM_300MA;
	else if (iterm_ma <= 400)
		reg = CHG_ITERM_400MA;
	else if (iterm_ma <= 500)
		reg = CHG_ITERM_500MA;
	else if (iterm_ma <= 600)
		reg = CHG_ITERM_600MA;
	else
		reg = CHG_ITERM_700MA;

	rc = smb1351_masked_write(chip, CHG_OTH_CURRENT_CTRL_REG,
				ITERM_MASK, reg);
	if (rc) {
		pr_err("Couldn't set iterm rc = %d\n", rc);
		return rc;
	}
	/* enable the iterm */
	rc = smb1351_masked_write(chip, CHG_CTRL_REG,
				ITERM_EN_BIT, ITERM_ENABLE);
	if (rc) {
		pr_err("Couldn't enable iterm rc = %d\n", rc);
		return rc;
	}
	return 0;
}

#define N_TYPE_BIT	8
static int smb1351_get_usb_supply_type(struct smb1351_charger *chip,
				enum power_supply_type *type)
{
	int rc, index;
	u8 reg;
	static const char * const usb_ps_type_str[] = {
		"ACA_DOCK",
		"ACA_C",
		"ACA_B",
		"ACA_A",
		"SDP",
		"OTHER",
		"DCP",
		"CDP",
	};

	rc = smb1351_read_reg(chip, STATUS_5_REG, &reg);
	if (rc) {
		pr_err("Couldn't read STATUS_5 rc = %d\n", rc);
		return rc;
	}

	switch (reg) {
	case STATUS_PORT_ACA_DOCK:
	case STATUS_PORT_ACA_C:
	case STATUS_PORT_ACA_B:
	case STATUS_PORT_ACA_A:
		*type = POWER_SUPPLY_TYPE_USB_ACA;
		break;
	case STATUS_PORT_CDP:
		*type = POWER_SUPPLY_TYPE_USB_CDP;
		break;
	case STATUS_PORT_DCP:
		*type = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	case STATUS_PORT_SDP:
		*type = POWER_SUPPLY_TYPE_USB;
		break;
	case STATUS_PORT_OTHER:
		*type = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	default:
		*type = POWER_SUPPLY_TYPE_UNKNOWN;
		break;
	}

	if (chip->is_factory_mode)
		*type = POWER_SUPPLY_TYPE_USB;

	if (*type == POWER_SUPPLY_TYPE_UNKNOWN) {
		pr_err("Can't get correct power supply type!");
	} else {
		index = find_first_bit((unsigned long *)&reg, N_TYPE_BIT);
		pr_debug("STATUS_5_REG(0x3B)=%x, USB type = %s\n",
				reg, usb_ps_type_str[index]);
	}

	return rc;
}

static int smb1351_set_bms_property(struct smb1351_charger *chip,
		enum power_supply_property prop, int value)
{
	int rc;
	union power_supply_propval propval = {0, };

	if (chip->bms_psy_name && !chip->bms_psy)
		chip->bms_psy =
			power_supply_get_by_name((char *)chip->bms_psy_name);

	if (IS_ERR_OR_NULL(chip->bms_psy)) {
		pr_debug("BMS(fg) power supply not present\n");
		return -ENODEV;
	}

	propval.intval = value;
	rc = chip->bms_psy->set_property(chip->bms_psy, prop, &propval);
	if (rc) {
		pr_err("Set BMS property %d failed, rc=%d\n", prop, rc);
		return rc;
	}

	return rc;
}

static int smb1351_get_bms_property(struct smb1351_charger *chip,
		enum power_supply_property prop, int *val)
{
	int rc;
	union power_supply_propval propval = {0, };

	if (chip->bms_psy_name && !chip->bms_psy)
		chip->bms_psy =
			power_supply_get_by_name((char *)chip->bms_psy_name);

	if (IS_ERR_OR_NULL(chip->bms_psy)) {
		pr_debug("BMS(fg) power supply not present\n");
		return -ENODEV;
	}

	rc = chip->bms_psy->get_property(chip->bms_psy, prop, &propval);
	if (rc) {
		pr_err("Set BMS property %d failed, rc=%d\n", prop, rc);
		return rc;
	}
	*val = propval.intval;

	return rc;
}

int smb1351_set_battery_property(struct smb1351_charger *chip,
		enum power_supply_property prop, int value)
{
	int rc;
	union power_supply_propval propval = {0, };

	propval.intval = value;
	rc = chip->batt_psy.set_property(&chip->batt_psy, prop, &propval);
	if (rc) {
		pr_err("Set batter property %d failed, rc=%d\n", prop, rc);
		return rc;
	}

	return rc;
}

static int smb1351_set_ext_charger_property(struct smb1351_charger *chip,
		enum power_supply_property prop, int value)
{
	int rc;
	union power_supply_propval propval = {0, };

	propval.intval = value;
	rc = chip->ext_charger_psy.set_property(&chip->ext_charger_psy, prop, &propval);
	if (rc) {
		pr_err("Set ext-charger property %d failed, rc=%d\n", prop, rc);
		return rc;
	}

	return rc;
}

static int smb1351_chg_otg_regulator_enable(struct regulator_dev *rdev)
{
	int rc = 0;
	struct smb1351_charger *chip = rdev_get_drvdata(rdev);

#ifdef LENOVO_OTG_USB_SHORT
	chip->otg_usb_short_state = false;
	chip->is_otg_on = true;
#endif

	rc = smb1351_masked_write(chip, OTG_TLIM_CTRL_REG, OTG_OC_LIMIT_MASK,
							0x00);
	if (rc)
		pr_err("Couldn't set OTG_OC_LIMIT rc=%d\n", rc);

	rc = smb1351_masked_write(chip, CMD_CHG_REG, CMD_OTG_EN_BIT,
							CMD_OTG_EN_BIT);
	if (rc)
		pr_err("Couldn't enable  OTG mode rc=%d\n", rc);
	return rc;
}

static int smb1351_chg_otg_regulator_disable(struct regulator_dev *rdev)
{
	int rc = 0;
	struct smb1351_charger *chip = rdev_get_drvdata(rdev);

#ifdef LENOVO_OTG_USB_SHORT
	{
		int pre_state = chip->otg_usb_short_state;

		gpio_set_value(chip->otg_usb_short_gpio, 0);
		chip->otg_usb_short_state = false;
		chip->is_otg_on = false;
		pr_debug("smb1351_otg_short_config = %d", chip->otg_usb_short_state);
		if (pre_state) {
			pr_debug("update otg usb short state\n");
			power_supply_changed(&chip->batt_psy);
		}
	}
#endif

	rc = smb1351_masked_write(chip, CMD_CHG_REG, CMD_OTG_EN_BIT, 0);
	if (rc)
		pr_err("Couldn't disable OTG mode rc=%d\n", rc);

	return rc;
}

static int smb1351_chg_otg_regulator_is_enable(struct regulator_dev *rdev)
{
	int rc = 0;
	u8 reg = 0;
	struct smb1351_charger *chip = rdev_get_drvdata(rdev);

	rc = smb1351_read_reg(chip, CMD_CHG_REG, &reg);
	if (rc) {
		pr_err("Couldn't read OTG enable bit rc=%d\n", rc);
		return rc;
	}

	return (reg & CMD_OTG_EN_BIT) ? 1 : 0;
}

struct regulator_ops smb1351_chg_otg_reg_ops = {
	.enable		= smb1351_chg_otg_regulator_enable,
	.disable	= smb1351_chg_otg_regulator_disable,
	.is_enabled	= smb1351_chg_otg_regulator_is_enable,
};

static int smb1351_regulator_init(struct smb1351_charger *chip)
{
	int rc = 0;
	struct regulator_init_data *init_data;
	struct regulator_config cfg = {};

	init_data = of_get_regulator_init_data(chip->dev, chip->dev->of_node);
	if (!init_data) {
		pr_err("Unable to allocate memory\n");
		return -ENOMEM;
	}

	if (init_data->constraints.name) {
		chip->otg_vreg.rdesc.owner = THIS_MODULE;
		chip->otg_vreg.rdesc.type = REGULATOR_VOLTAGE;
		chip->otg_vreg.rdesc.ops = &smb1351_chg_otg_reg_ops;
		chip->otg_vreg.rdesc.name = init_data->constraints.name;

		cfg.dev = chip->dev;
		cfg.init_data = init_data;
		cfg.driver_data = chip;
		cfg.of_node = chip->dev->of_node;

		init_data->constraints.valid_ops_mask
			|= REGULATOR_CHANGE_STATUS;

		chip->otg_vreg.rdev = regulator_register(
						&chip->otg_vreg.rdesc, &cfg);
		if (IS_ERR(chip->otg_vreg.rdev)) {
			rc = PTR_ERR(chip->otg_vreg.rdev);
			chip->otg_vreg.rdev = NULL;
			if (rc != -EPROBE_DEFER)
				pr_err("OTG reg failed, rc=%d\n", rc);
		}
	}

	return rc;
}

static int smb_chip_get_version(struct smb1351_charger *chip)
{
	u8 ver;
	int rc = 0;

	if (chip->version == SMB_UNKNOWN) {
		rc = smb1351_read_reg(chip, VERSION_REG, &ver);
		if (rc) {
			pr_err("Couldn't read version rc=%d\n", rc);
			return rc;
		}

		/* If bit 1 is set, it is SMB1350 */
		if (ver & VERSION_MASK)
			chip->version = SMB1350;
		else
			chip->version = SMB1351;
	}

	return rc;
}

static int smb1351_disable_hw_jieta(struct smb1351_charger *chip, bool disable)
{
	int rc = 0;

	rc = smb1351_masked_write(chip, THERM_A_CTRL_REG,
			THERM_MONITOR_BIT, disable?THERM_MONITOR_BIT:THERM_MONITOR_EN);
	if (rc) {
		pr_err("Couldn't set THERM_A_CTRL_REG rc=%d\n",	rc);
		return rc;
	}

	return rc;
}

#define INIT_FG_RETRY_MS	1000
static int smb1351_hw_init(struct smb1351_charger *chip)
{
	int rc = 0;
	u8 reg = 0, mask = 0;

	/* configure smb_pinctrl to enable irqs */
	if (chip->pinctrl_state_name) {
		chip->smb_pinctrl = pinctrl_get_select(chip->dev,
						chip->pinctrl_state_name);
		if (IS_ERR(chip->smb_pinctrl)) {
			pr_err("Could not get/set %s pinctrl state rc = %ld\n",
						chip->pinctrl_state_name,
						PTR_ERR(chip->smb_pinctrl));
			return PTR_ERR(chip->smb_pinctrl);
		}
	}

	/*
	 * If the charger is pre-configured for autonomous operation,
	 * do not apply additional settings
	 */
	if (chip->chg_autonomous_mode) {
		pr_debug("Charger configured for autonomous mode\n");
		return 0;
	}

	rc = smb_chip_get_version(chip);
	if (rc) {
		pr_err("Couldn't get version rc = %d\n", rc);
		return rc;
	}

	rc = smb1351_enable_volatile_writes(chip);
	if (rc) {
		pr_err("Couldn't configure volatile writes rc=%d\n", rc);
		return rc;
	}

	/* setup battery missing source */
	reg = BATT_MISSING_THERM_PIN_SOURCE_BIT;
	mask = BATT_MISSING_THERM_PIN_SOURCE_BIT;
	rc = smb1351_masked_write(chip, HVDCP_BATT_MISSING_CTRL_REG,
								mask, reg);
	if (rc) {
		pr_err("Couldn't set HVDCP_BATT_MISSING_CTRL_REG rc=%d\n", rc);
		return rc;
	}

	/* disable hvdcp */
	mask = HVDCP_EN_BIT;
	rc = smb1351_masked_write(chip, HVDCP_BATT_MISSING_CTRL_REG,
								mask, 0);
	if (rc) {
		pr_err("Couldn't set HVDCP_BATT_MISSING_CTRL_REG rc=%d\n", rc);
		return rc;
	}

	/* setup defaults for CHG_PIN_EN_CTRL_REG */
	reg = EN_BY_I2C_0_DISABLE | USBCS_CTRL_BY_I2C | CHG_ERR_BIT |
		APSD_DONE_BIT | LED_BLINK_FUNC_BIT;
	mask = EN_PIN_CTRL_MASK | USBCS_CTRL_BIT | CHG_ERR_BIT |
		APSD_DONE_BIT | LED_BLINK_FUNC_BIT;
	rc = smb1351_masked_write(chip, CHG_PIN_EN_CTRL_REG, mask, reg);
	if (rc) {
		pr_err("Couldn't set CHG_PIN_EN_CTRL_REG rc=%d\n", rc);
		return rc;
	}
	/* setup USB 2.0/3.0 detection and USB 500/100 command polarity */
	reg = USB_2_3_MODE_SEL_BY_I2C | USB_CMD_POLARITY_500_1_100_0;
	mask = USB_2_3_MODE_SEL_BIT | USB_5_1_CMD_POLARITY_BIT;
	rc = smb1351_masked_write(chip, CHG_OTH_CURRENT_CTRL_REG, mask, reg);
	if (rc) {
		pr_err("Couldn't set CHG_OTH_CURRENT_CTRL_REG rc=%d\n", rc);
		return rc;
	}
	/* setup USB suspend, AICL and APSD  */
	reg = SUSPEND_MODE_CTRL_BY_I2C | AICL_EN_BIT;
	if (!chip->disable_apsd)
		reg |= APSD_EN_BIT;
	mask = SUSPEND_MODE_CTRL_BIT | AICL_EN_BIT | APSD_EN_BIT;
	rc = smb1351_masked_write(chip, VARIOUS_FUNC_REG, mask, reg);
	if (rc) {
		pr_err("Couldn't set VARIOUS_FUNC_REG rc=%d\n",	rc);
		return rc;
	}
	/* Fault and Status IRQ configuration */
	reg = HOT_COLD_HARD_LIMIT_BIT | HOT_COLD_SOFT_LIMIT_BIT | OTG_OC_BIT
		| INPUT_OVLO_BIT | INPUT_UVLO_BIT | AICL_DONE_FAIL_BIT;
	rc = smb1351_write_reg(chip, FAULT_INT_CFG_REG, reg);
	if (rc) {
		pr_err("Couldn't set FAULT_INT_CFG_REG rc=%d\n", rc);
		return rc;
	}
	reg = CHG_OR_PRECHG_TIMEOUT_BIT | RID_CHANGE_BIT | BATT_OVP_BIT |
		FAST_TERM_TAPER_RECHG_INHIBIT_BIT | BATT_MISSING_BIT |
		BATT_LOW_BIT;
	rc = smb1351_write_reg(chip, STATUS_INT_CFG_REG, reg);
	if (rc) {
		pr_err("Couldn't set STATUS_INT_CFG_REG rc=%d\n", rc);
		return rc;
	}
	/* setup THERM Monitor */
	if (!chip->using_pmic_therm) {
		rc = smb1351_masked_write(chip, THERM_A_CTRL_REG,
			THERM_MONITOR_BIT, THERM_MONITOR_EN);
		if (rc) {
			pr_err("Couldn't set THERM_A_CTRL_REG rc=%d\n",	rc);
			return rc;
		}
	}

	/*disable hw jieta anyway*/
	rc = smb1351_masked_write(chip, THERM_A_CTRL_REG,
		THERM_MONITOR_BIT, THERM_MONITOR_BIT);
	if (rc) {
		pr_err("Couldn't set THERM_A_CTRL_REG rc=%d\n",	rc);
		return rc;
	}

	/* set the fast charge current limit */
	rc = smb1351_fastchg_current_set(chip,
			chip->target_fastchg_current_max_ma);
	if (rc) {
		pr_err("Couldn't set fastchg current rc=%d\n", rc);
		return rc;
	}

	/* set the float voltage */
	if (chip->vfloat_mv != -EINVAL) {
		rc = smb1351_float_voltage_set(chip, chip->vfloat_mv);
		if (rc) {
			pr_err("Couldn't set float voltage rc = %d\n", rc);
			return rc;
		}
	}

	/* set iterm */
	if (chip->iterm_ma != -EINVAL) {
		if (chip->iterm_disabled) {
			pr_err("Error: Both iterm_disabled and iterm_ma set\n");
			return -EINVAL;
		} else {
			rc = smb1351_iterm_set(chip, chip->iterm_ma);
			if (rc) {
				pr_err("Couldn't set iterm rc = %d\n", rc);
				return rc;
			}
		}
	} else  if (chip->iterm_disabled) {
		rc = smb1351_masked_write(chip, CHG_CTRL_REG,
					ITERM_EN_BIT, ITERM_DISABLE);
		if (rc) {
			pr_err("Couldn't set iterm rc = %d\n", rc);
			return rc;
		}
	}

	/* set recharge-threshold */
	if (chip->recharge_mv != -EINVAL) {
		if (chip->recharge_disabled) {
			pr_err("Error: Both recharge_disabled and recharge_mv set\n");
			return -EINVAL;
		} else {
			reg = AUTO_RECHG_ENABLE;
			if (chip->recharge_mv > 50)
				reg |= AUTO_RECHG_TH_100MV;
			else
				reg |= AUTO_RECHG_TH_50MV;

			rc = smb1351_masked_write(chip, CHG_CTRL_REG,
					AUTO_RECHG_BIT |
					AUTO_RECHG_TH_BIT, reg);
			if (rc) {
				pr_err("Couldn't set rechg-cfg rc = %d\n", rc);
				return rc;
			}
		}
	} else if (chip->recharge_disabled ||
			(chip->wa_flags & CHG_SOC_BASED_RESUME)) {
		rc = smb1351_masked_write(chip, CHG_CTRL_REG,
				AUTO_RECHG_BIT,
				AUTO_RECHG_DISABLE);
		if (rc) {
			pr_err("Couldn't disable auto-rechg rc = %d\n", rc);
			return rc;
		}
	}

	/* Update switching frequency based on device tree entry */
	if (chip->switch_freq != -EINVAL) {
		pr_debug("set switch_freq\n");
		rc = smb1351_masked_write(chip, OTG_TLIM_CTRL_REG,
				SWITCH_FREQ_MASK,
				(chip->switch_freq << SWITCH_FREQ_SHIFT));
		if (rc) {
			pr_err("Set switching frequency failed, r=%d\n", rc);
			return rc;
		}
	}

	/* set STAT pin behavior: active low, indicate charging status */
	rc = smb1351_masked_write(chip, CHG_STAT_TIMERS_CTRL_REG,
			STAT_OUTPUT_POLARITY_BIT | STAT_OUTPUT_MODE_BIT
			| STAT_OUTPUT_CTRL_BIT, 0);
	if (rc) {
		pr_err("Set STAT pin polarity failed, r=%d\n", rc);
		return rc;
	}

	if (chip->parallel.avail) {
		rc = smb1351_masked_write(chip, PON_OPTIONS_REG,
			SYSOK_INOK_POLARITY_BIT | SYSOK_OPTIONS_MASK,
			SYSOK_INOK_POLARITY_INVERT | SYSOK_INOK_OPTION1);
		if (rc) {
			pr_err("Program PON option failed, rc=%d\n", rc);
			return rc;
		}
	}

	if (chip->use_external_fg) {
		pr_info("using externel fg\n");
		rc = smb1351_set_bms_property(chip,
			POWER_SUPPLY_PROP_IGNORE_FALSE_NEGATIVE_ISENSE, 0);
		if (rc) {
			pr_debug("set IGNORE_FALSE_NEGATIVE_ISENSE failed, rc=%d\n",
								rc);
			/* start a delay worker to do it again if failed */
			schedule_delayed_work(&chip->init_fg_work,
				msecs_to_jiffies(INIT_FG_RETRY_MS));
		} else if (chip->fg_notify_jeita) {
			rc = smb1351_set_bms_property(chip,
				POWER_SUPPLY_PROP_ENABLE_JEITA_DETECTION, 1);
			if (rc) {
				pr_err("Set ENABLE_JEITA_DETECTION failed, rc=%d\n",
								rc);
				return rc;
			}
		}
	}

	if (chip->is_factory_mode) {
		pr_info("set chg as factory mode\n");

//		smb1351_set_usb_chg_current(chip, 3000);
//		smb1351_masked_write(chip, CMD_INPUT_LIMIT_REG, BIT(0), CMD_USB_HC_MODE);
		rc = smb1351_battchg_disable(chip, USER, 1);
		if (rc)
			pr_err("Couldn't %s charging rc = %d\n",
				chip->usb_suspended_status ? "disable" : "enable", rc);
	} else {
		/* enable/disable charging by suspending usb */
		rc = smb1351_usb_suspend(chip, USER, chip->usb_suspended_status);
		if (rc) {
			pr_err("Unable to %s USB input. rc=%d\n",
				chip->usb_suspended_status ? "suspend" : "enable", rc);
			return rc;
		}

		rc = smb1351_battchg_disable(chip, USER, chip->usb_suspended_status);
		if (rc)
			pr_err("Couldn't %s charging rc = %d\n",
				chip->usb_suspended_status ? "disable" : "enable", rc);

		rc = smb1351_masked_write(chip, FLEXCHARGER_REG, 0xff, 0x47);
		if (rc)
			pr_err("FLEXCHARGER_REG rc = %d\n",
				rc);
		rc = smb1351_masked_write(chip, OTG_MODE_POWER_OPTIONS_REG, 0xff, 0x9c);
		if (rc)
			pr_err("OTG_MODE_POWER_OPTIONS_REG rc = %d\n",
				rc);
		rc = smb1351_masked_write(chip, CHG_CTRL_REG, AFCV_MASK, 0);
		if (rc)
			pr_err("CHG_CTRL_REG rc = %d\n",
				rc);
		rc = smb1351_masked_write(chip, WDOG_SAFETY_TIMER_CTRL_REG, AICL_FAIL_OPTION_BIT, AICL_FAIL_OPTION_BIT);
		if (rc)
			pr_err("WDOG_SAFETY_TIMER_CTRL_REG rc = %d\n",
				rc);
	}

	return rc;
}

static enum power_supply_property smb1351_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_INPUT_VOLTAGE_REGULATION,
	POWER_SUPPLY_PROP_HI_POWER,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
};

static int smb1351_get_prop_batt_status(struct smb1351_charger *chip)
{
	int rc;
	u8 reg = 0;

	if (chip->batt_full)
		return POWER_SUPPLY_STATUS_FULL;

	rc = smb1351_read_reg(chip, STATUS_4_REG, &reg);
	if (rc) {
		pr_err("Couldn't read STATUS_4 rc = %d\n", rc);
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}

	pr_debug("STATUS_4_REG(0x3A)=%x\n", reg);

	if (reg & STATUS_HOLD_OFF_BIT)
		return POWER_SUPPLY_STATUS_NOT_CHARGING;

	if (reg & STATUS_CHG_MASK)
		return POWER_SUPPLY_STATUS_CHARGING;

	return POWER_SUPPLY_STATUS_DISCHARGING;
}

static int smb1351_get_prop_batt_present(struct smb1351_charger *chip)
{
	return !chip->battery_missing;
}

static int smb1351_get_prop_batt_capacity(struct smb1351_charger *chip)
{
	int rc = 0, soc = 0;

	if (chip->fake_battery_soc >= 0)
		return chip->fake_battery_soc;

	rc = smb1351_get_bms_property(chip, POWER_SUPPLY_PROP_CAPACITY, &soc);
	if (rc) {
		pr_debug("Get capacity from BMS failed, rc=%d\n", rc);
		return DEFAULT_BATT_CAPACITY;
	}
	pr_debug("SoC = %d\n", soc);

	return soc;
}

static int smb1351_get_prop_batt_voltage(struct smb1351_charger *chip)
{
	union power_supply_propval ret = {0, };

	if (chip->fake_battery_soc >= 0)
		return chip->fake_battery_soc;

	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_VOLTAGE_NOW, &ret);
		return ret.intval;
	}
	pr_debug("return DEFAULT_BATT_VOLTAGE\n");
	return DEFAULT_BATT_VOLTAGE;
}

static int smb1351_get_prop_batt_temp(struct smb1351_charger *chip)
{
	int rc = 0, temp = 0;
	struct qpnp_vadc_result results;

	rc = smb1351_get_bms_property(chip, POWER_SUPPLY_PROP_TEMP, &temp);
	if (rc) {
		temp = DEFAULT_BATT_TEMP;
		pr_debug("Get tempertory from BMS failed, rc=%d\n", rc);
	}
	if (chip->vadc_dev) {
		rc = qpnp_vadc_read(chip->vadc_dev,
				LR_MUX1_BATT_THERM, &results);
		if (rc)
			pr_debug("Unable to read adc batt temp rc=%d\n", rc);
		else
			temp = (int)results.physical;
	}
	pr_debug("temperature: %d\n", temp);

	if ( chip->temp_debug_flag)
		temp = chip->batt_temp;

	return temp;
}

static int smb1351_get_prop_charge_type(struct smb1351_charger *chip)
{
	int rc;
	u8 reg = 0;

	rc = smb1351_read_reg(chip, STATUS_4_REG, &reg);
	if (rc) {
		pr_err("Couldn't read STATUS_4 rc = %d\n", rc);
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	}

	pr_debug("STATUS_4_REG(0x3A)=%x\n", reg);

	reg &= STATUS_CHG_MASK;

	if (reg == STATUS_FAST_CHARGING)
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	else if (reg == STATUS_TAPER_CHARGING)
		return POWER_SUPPLY_CHARGE_TYPE_TAPER;
	else if (reg == STATUS_PRE_CHARGING)
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
	else
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
}

static int smb1351_get_prop_batt_health(struct smb1351_charger *chip)
{
	union power_supply_propval ret = {0, };

	if (chip->batt_hot)
		ret.intval = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (chip->batt_cold)
		ret.intval = POWER_SUPPLY_HEALTH_COLD;
	else if (chip->batt_warm) {
		if (chip->batt_full) {
			ret.intval = POWER_SUPPLY_HEALTH_OVERHEAT;
		} else {
			ret.intval = POWER_SUPPLY_HEALTH_GOOD;
		}
		pr_info("batt_full %d ret %d\n", chip->batt_full, ret.intval);
	} else if (chip->batt_cool)
		ret.intval = POWER_SUPPLY_HEALTH_GOOD;
	else
		ret.intval = POWER_SUPPLY_HEALTH_GOOD;

	return ret.intval;
}

static int smb1351_get_usbid_status(struct smb1351_charger *chip, bool *gnd)
{
	int rc;
	u8 regval;

	rc = smb1351_read_reg(chip, STATUS_6_REG, &regval);
	if (rc) {
		pr_err("Read STATUS_6 failed, rc=%d\n", rc);
		return rc;
	}

	*gnd = !regval;

	return rc;
}

static struct power_supply *smb1351_get_parallel_slave(
			struct smb1351_charger *chip)
{
	if (!chip->parallel.avail)
		return NULL;
	if (chip->parallel.psy)
		return chip->parallel.psy;

	chip->parallel.psy = power_supply_get_by_name(PSY_PARALLEL_NAME);
	if (!chip->parallel.psy)
		pr_debug("parallel slave not found\n");

	return chip->parallel.psy;
}
/*
 * Get the mininum parallel charging current threshold according to
 * the voltage provided on the charger adapter
 */
static int smb1351_get_min_parallel_current_ma(struct smb1351_charger *chip)
{
	int rc;
	u8 value;

	rc = smb1351_read_reg(chip, STATUS_1_REG, &value);
	if (rc) {
		pr_err("read STATUS_7_REG failed, rc = %d\n", rc);
		return rc;
	}
	switch (value & STATUS_INPUT_RANGE_MASK) {
	case STATUS_INPUT_RANGE_5V:
	case STATUS_INPUT_RANGE_5V_9V:
		rc = chip->parallel.min_current_thr_ma;
		break;
	case STATUS_INPUT_RANGE_9V:
		rc = chip->parallel.min_9v_current_thr_ma;
		break;
	case STATUS_INPUT_RANGE_12V:
		rc = chip->parallel.min_12v_current_thr_ma;
		break;
	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int smb1351_parallel_charger_disable_slave(
			struct smb1351_charger *chip)
{
	int rc;
	struct power_supply *parallel_psy = smb1351_get_parallel_slave(chip);

	pr_debug("Disable parallel slave!\n");

	chip->parallel.total_icl_ma = 0;
	chip->parallel.slave_icl_ma = 0;

	/* Suspend USB and clear PRESENT to slave charger */
	power_supply_set_current_limit(parallel_psy, SUSPEND_CURRENT_MA * 1000);
	power_supply_set_present(parallel_psy, false);

	/* Restore main charger FCC && ICL as the standalone */
	rc = smb1351_chg_set_appropriate_battery_current(chip);
	if (rc) {
		pr_err("restore FCC to %d failed\n",
				chip->target_fastchg_current_max_ma);
		return rc;
	}
	rc = smb1351_set_usb_chg_current(chip, chip->usb_psy_ma);
	if (rc) {
		pr_err("restore ICL to %d failed\n", chip->usb_psy_ma);
		return rc;
	}
	pr_debug("Restore ICL %dmA to the main charger\n", chip->usb_psy_ma);

	return rc;
}

/*
 * Enable slave, and split the total input current to main charger
 * and the slave according to the percent.
 */
static int smb1351_parallel_charger_enable_slave(
			struct smb1351_charger *chip,
			int total_icl_ma)
{
	struct power_supply *parallel_psy = smb1351_get_parallel_slave(chip);
	union power_supply_propval pval = {0, };
	int rc, slave_icl_ma, actual_slave_icl_ma, main_icl_ma;
	int slave_fcc_ma, actual_slave_fcc_ma, main_fcc_ma;

	if (!parallel_psy || !chip->parallel.slave_detected)
		return 0;

	if (chip->handshake_state) {
		pr_err("in handshake state. Do not enable\n");
		return 0;
	}

	pr_debug("Enable parallel slave!\n");
	/*
	 * Set slave's float voltage a little high main charger
	 * to avoid slave trigger taper before the main charger
	 */
	rc = power_supply_set_voltage_limit(parallel_psy, chip->vfloat_mv + 50);
	if (rc) {
		pr_err("Set slave VFLT failed, rc=%d\n", rc);
		return rc;
	}

	/* split the ICL for main and slave charger */
	slave_icl_ma = total_icl_ma *
		(100 - chip->parallel.main_chg_icl_percent) / 100;

	/* set the allotted ICL for slave */
	rc = power_supply_set_present(parallel_psy, true);
	if (rc) {
		pr_err("Set slave present failed, rc=%d\n", rc);
		return rc;
	}

	rc = power_supply_set_current_limit(parallel_psy, slave_icl_ma * 1000);
	if (rc) {
		pr_err("Set slave ICL to %dmA failed, rc=%d\n",
						slave_icl_ma, rc);
		return rc;
	}

	chip->parallel.slave_icl_ma = slave_icl_ma;

	/* get the ICL real set in HW */
	parallel_psy->get_property(parallel_psy,
			POWER_SUPPLY_PROP_CURRENT_MAX, &pval);
	actual_slave_icl_ma = pval.intval / 1000;
	pr_debug("parallel slave ICL, program %d, set %d\n",
			slave_icl_ma, actual_slave_icl_ma);

	/* set the allotted ICL for main charger */
	main_icl_ma = max(total_icl_ma - actual_slave_icl_ma, 0);

	rc = smb1351_set_usb_chg_current(chip, main_icl_ma);
	if (rc) {
		pr_err("set main charger ICL %d failed, rc=%d\n",
						main_icl_ma, rc);
		return rc;
	}
	pr_debug("ICL split, total %d: main %d, slave %d\n",
			total_icl_ma, main_icl_ma, actual_slave_icl_ma);
	/* split the FCC for main and slave charger */
	slave_fcc_ma = chip->target_fastchg_current_max_ma *
		(100 - chip->parallel.main_chg_fcc_percent) / 100;

	/* set the allotted FCC to slave charger */
	pval.intval = slave_fcc_ma * 1000;
	parallel_psy->set_property(parallel_psy,
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &pval);
	parallel_psy->get_property(parallel_psy,
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &pval);
	actual_slave_fcc_ma = pval.intval / 1000;

	/* set the allotted FCC to main charger */
	main_fcc_ma = chip->target_fastchg_current_max_ma
				- actual_slave_fcc_ma;

	rc = smb1351_fastchg_current_set(chip, main_fcc_ma);
	if (rc) {
		pr_err("Set main charger FCC %d failed, rc=%d\n",
						main_fcc_ma, rc);
		return rc;
	}
	pr_debug("FCC split, total %d: main %d, slave %d\n",
			chip->target_fastchg_current_max_ma,
			main_fcc_ma, actual_slave_fcc_ma);
	return rc;
}

/*
 * Check conditions if it's able to enable the slave for parallel
 * charging, and also caculate the max current could be drawn from
 * the charger adapter.
 */
static bool smb1351_attempt_enable_parallel_slave(
			struct smb1351_charger *chip,
			int *ret_total_icl_ma)
{
	struct power_supply *parallel_psy = smb1351_get_parallel_slave(chip);
	union power_supply_propval pval = {0, };
	enum power_supply_type type;
	int min_icl_thr_ma, main_current_icl_ma;
	int slave_current_icl_ma, total_icl_ma;
	int rc;

	pr_debug("Check if it is OK to enable slave!\n");
	if (!parallel_psy || !chip->parallel.slave_detected)
		return false;
	/*
	 *  Report false to disable parallel charging if NOT in fastchg mode,
	 *  only when battery is connected or parallel slave hasn't been
	 *  enabled.
	 */
	if (smb1351_get_prop_charge_type(chip) != POWER_SUPPLY_CHARGE_TYPE_FAST
		&& (smb1351_get_prop_batt_present(chip)
			|| chip->parallel.slave_icl_ma == 0)) {
		pr_info("Not in fastchg mode, disable parallel!\n");
		return false;
	}
	/* If battery health NOT good */
	if (smb1351_get_prop_batt_health(chip) != POWER_SUPPLY_HEALTH_GOOD) {
		pr_info("JEITA active, disable parallel!\n");
		return false;
	}
	/* Only start parallel charger in DCP/HVDCP */
	rc = smb1351_get_usb_supply_type(chip, &type);
	if (rc) {
		pr_err("Get USB power supply type failed, rc=%d\n", rc);
		return false;
	} else if (type == POWER_SUPPLY_TYPE_USB_CDP ||
			type == POWER_SUPPLY_TYPE_USB) {
		pr_debug("USB type %d, disable parallel!\n", type);
		return false;
	}

	min_icl_thr_ma = smb1351_get_min_parallel_current_ma(chip);
	if (min_icl_thr_ma <= 0) {
		pr_info("Get min ICL threshold failed, disable parallel!\n");
		return false;
	}

	rc = smb1351_get_usb_chg_current(chip, &main_current_icl_ma);
	if (rc) {
		pr_info("Get main charger ICL failed, skipping!\n");
		return false;
	}
	pr_debug("main charger ICL = %d\n", main_current_icl_ma);
	/*
	 * If parallel is not enabled, and main charger ICL is less than min
	 * parallel ICL threshold, disable parallel.
	 * Record the total ICL at 1st place.
	 */
	if (chip->parallel.total_icl_ma == 0) {
		if (main_current_icl_ma < min_icl_thr_ma) {
			pr_info("Current ICL %d lower than threshold %d, skipping!\n",
					main_current_icl_ma, min_icl_thr_ma);
			return false;
		}
		chip->parallel.total_icl_ma = main_current_icl_ma;
		pr_debug("Not in parallel charging previously, total_icl = %d\n",
							main_current_icl_ma);
	}

	/* Check if parallel slave already draw some current */
	parallel_psy->get_property(parallel_psy,
			POWER_SUPPLY_PROP_CURRENT_MAX, &pval);
	slave_current_icl_ma = pval.intval / 1000;
	if (slave_current_icl_ma == SUSPEND_CURRENT_MA)
		slave_current_icl_ma = 0;

	pr_debug("slave charger ICL = %d\n", slave_current_icl_ma);
	total_icl_ma = min(main_current_icl_ma + slave_current_icl_ma,
						chip->usb_psy_ma);
	pr_debug("calculated total ICL = %d\n", total_icl_ma);
	/*
	 * If calculated total ICL is below than allowed_lowering,
	 * disable parallel
	 */
	if (total_icl_ma <
		chip->parallel.total_icl_ma -
		chip->parallel.allowed_lowering_ma) {
		pr_info("total ICL reduced to below the threshold: %d < (%d-%d)\n",
				total_icl_ma, chip->parallel.total_icl_ma,
				chip->parallel.allowed_lowering_ma);
		return false;
	}

	*ret_total_icl_ma = total_icl_ma;

	pr_debug("Final total ICL for parallel charging = %d\n", total_icl_ma);
	return true;
}

#define MIN_SLAVER_FCC_MA			500
#define SLAVE_CHG_FCC_REDUCTION_PERCENTAGE		75
#define SLAVER_FCC_REDUCTION_MAX_TRIES		3
static int smb1351_parallel_charger_taper_attempt(
			struct smb1351_charger *chip)
{
	struct power_supply *parallel_psy = smb1351_get_parallel_slave(chip);
	union power_supply_propval pval = {0, };
	int rc, slave_fcc_ma, tries = 0;
	u8 value;

	if (!parallel_psy || !chip->parallel.slave_detected)
		return 0;

	smb1351_stay_awake(&chip->smb1351_ws, PARALLEL_TAPER);

try_again:
	mutex_lock(&chip->parallel.lock);
	if (chip->parallel.slave_icl_ma == 0) {
		pr_debug("Parallel slave not enabled, skipping\n");
		rc = 0;
		goto done;
	}
	rc = parallel_psy->get_property(parallel_psy,
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &pval);
	if (rc) {
		pr_err("Get slave FCC failed, rc=%d\n", rc);
		goto done;
	}

	tries += 1;
	slave_fcc_ma = pval.intval / 1000;

	if (slave_fcc_ma < MIN_SLAVER_FCC_MA ||
			tries > SLAVER_FCC_REDUCTION_MAX_TRIES) {
		pr_debug("reduce slave FCC can't exist taper, disable slave\n");
		smb1351_parallel_charger_disable_slave(chip);
		goto done;
	}

	pval.intval = slave_fcc_ma * SLAVE_CHG_FCC_REDUCTION_PERCENTAGE / 100;
	pval.intval *= 1000;
	rc = parallel_psy->set_property(parallel_psy,
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &pval);
	if (rc) {
		pr_err("Set slave FCC %duA failed, rc=%d\n", pval.intval, rc);
		goto done;
	}

	mutex_unlock(&chip->parallel.lock);
	/*
	 * Sleep 100ms to make sure the charger has a chance to
	 * go back to fast charging state.
	 */
	msleep(100);

	mutex_lock(&chip->parallel.lock);
	rc = smb1351_read_reg(chip, IRQ_C_REG, &value);
	if (rc) {
		pr_err("Read IRQ_C failed, rc=%d\n", rc);
		goto done;
	}

	if (value & IRQ_TAPER_BIT) {
		mutex_unlock(&chip->parallel.lock);
		pr_debug("recheck: %d\n", tries);
		goto try_again;
	}
done:
	mutex_unlock(&chip->parallel.lock);
	smb1351_relax(&chip->smb1351_ws, PARALLEL_TAPER);
	return rc;
}

static void smb1351_parallel_work(struct work_struct *work)
{
	struct smb1351_charger *chip = container_of(work,
		struct smb1351_charger, parallel.parallel_work.work);

	int rc, pre_icl_ma, icl_ma, total_icl_ma = 0;

	/* Get ICL twice with 500ms delays to check if ICl settled */
	rc = smb1351_get_usb_chg_current(chip, &pre_icl_ma);
	if (rc) {
		pr_err("Get ICL result failed, rc = %d\n", rc);
		return;
	}

	msleep(500);

	rc = smb1351_get_usb_chg_current(chip, &icl_ma);
	if (rc) {
		pr_err("Get ICL result failed, rc = %d\n", rc);
		return;
	}

	if ((pre_icl_ma > 0) && (pre_icl_ma == icl_ma)) {
		pr_debug("AICL stable at %d\n", icl_ma);
	} else {
		pr_debug("AICL changed %d -> %d, double check\n",
				pre_icl_ma, icl_ma);
		goto recheck;
	}
	mutex_lock(&chip->parallel.lock);
	if (smb1351_attempt_enable_parallel_slave(chip, &total_icl_ma)) {
		pr_debug("Enable parallel slave, total_icl = %d!\n",
							total_icl_ma);
		smb1351_parallel_charger_enable_slave(chip, total_icl_ma);
	} else {
		if (chip->parallel.slave_icl_ma != 0) {
			pr_debug("Disable parallel slave!\n");
			smb1351_parallel_charger_disable_slave(chip);
		}
	}
	mutex_unlock(&chip->parallel.lock);
	smb1351_relax(&chip->smb1351_ws, PARALLEL_CHARGE);

	return;
recheck:
	pr_debug("reschedule!\n");
	schedule_delayed_work(&chip->parallel.parallel_work, 0);
}

static void smb1351_parallel_check_start(struct smb1351_charger *chip)
{
	struct power_supply *parallel_psy = smb1351_get_parallel_slave(chip);

	if (!parallel_psy || !chip->parallel.slave_detected)
		return;

	cancel_delayed_work_sync(&chip->parallel.parallel_work);
	smb1351_stay_awake(&chip->smb1351_ws, PARALLEL_CHARGE);
	schedule_delayed_work(&chip->parallel.parallel_work, 0);
	pr_debug("parallel work scheduled\n");
}

static void smb1351_init_fg_work(struct work_struct *work)
{
	int rc;
	struct smb1351_charger *chip = container_of(work,
		struct smb1351_charger, init_fg_work.work);

	rc = smb1351_set_bms_property(chip,
		POWER_SUPPLY_PROP_IGNORE_FALSE_NEGATIVE_ISENSE, 0);
	if (rc) {
		pr_debug("Disable isense patch failed, rc=%d\n", rc);
		goto recheck;
	}

	if (chip->fg_notify_jeita) {
		rc = smb1351_set_bms_property(chip,
			POWER_SUPPLY_PROP_ENABLE_JEITA_DETECTION, 1);
		if (rc) {
			pr_debug("Enable FG JEITA IRQ failed, rc=%d\n", rc);
			goto recheck;
		}
	}
	return;
recheck:
	/* start a delay worker to do it again if failed */
	schedule_delayed_work(&chip->init_fg_work,
			msecs_to_jiffies(INIT_FG_RETRY_MS));
}

static void smb1351_handle_jeita_from_fg(struct smb1351_charger *chip,
						int health)
{
	int rc;
#ifndef APP_CONTROL
	bool disable;
#endif

	if (health <= POWER_SUPPLY_HEALTH_UNKNOWN
		|| health > POWER_SUPPLY_HEALTH_COOL) {
		pr_err("health state not valid: %d\n", health);
		return;
	}

	switch (health) {
	case POWER_SUPPLY_HEALTH_GOOD:
		chip->batt_hot = false;
		chip->batt_cold = false;
		chip->batt_warm = false;
		chip->batt_cool = false;
		break;
	case POWER_SUPPLY_HEALTH_OVERHEAT:
		chip->batt_hot = true;
		chip->batt_cold = false;
		chip->batt_warm = false;
		chip->batt_cool = false;
		break;
	case POWER_SUPPLY_HEALTH_COLD:
		chip->batt_hot = false;
		chip->batt_cold = true;
		chip->batt_cool = false;
		chip->batt_warm = false;
		break;
	case POWER_SUPPLY_HEALTH_WARM:
		chip->batt_hot = false;
		chip->batt_cold = false;
		chip->batt_warm = true;
		chip->batt_cool = false;
		break;
	case POWER_SUPPLY_HEALTH_COOL:
		chip->batt_hot = false;
		chip->batt_cold = false;
		chip->batt_warm = false;
		chip->batt_cool = true;
		break;
	default:
		pr_debug("health: %d, not a JEITA state\n", health);
		break;
	}
	pr_debug("hot: %d, cold: %d, warm = %d, cool = %d\n",
			chip->batt_hot, chip->batt_cold,
			chip->batt_warm, chip->batt_cool);

#ifndef APP_CONTROL
	disable = (chip->batt_hot || chip->batt_cold) ? true : false;
	rc = smb1351_battchg_disable(chip, THERMAL, disable);
	if (rc) {
		pr_err("%s charging for THERMAL failed, rc=%d\n",
				disable ? "Disable" : "Enable", rc);
		return;
	}

	rc = smb1351_chg_set_appropriate_battery_current(chip);
	if (rc) {
		pr_err("Set battery current failed\n");
		return;
	}
#endif

	rc = smb1351_chg_set_appropriate_vfloat(chip);
	if (rc) {
		pr_err("Set float voltage failed\n");
		return;
	}

	smb1351_parallel_check_start(chip);

	if (chip->use_external_fg) {
		smb1351_set_bms_property(chip, POWER_SUPPLY_PROP_STATUS,
			smb1351_get_prop_batt_status(chip));
		smb1351_set_bms_property(chip, POWER_SUPPLY_PROP_HEALTH,
			smb1351_get_prop_batt_health(chip));
	}

	pr_debug("end!\n");
}

static void smb1351_handle_jeita_lenovo(struct smb1351_charger *chip,
						int health)
{
	int rc;
#ifndef APP_CONTROL
	bool disable;
#endif

	if (health <= POWER_SUPPLY_HEALTH_UNKNOWN
		|| health > POWER_SUPPLY_HEALTH_COOL) {
		pr_err("health state not valid: %d\n", health);
		return;
	}

	switch (health) {
	case POWER_SUPPLY_HEALTH_GOOD:
		chip->batt_hot = false;
		chip->batt_cold = false;
		chip->batt_warm = false;
		chip->batt_cool = false;
		break;
	case POWER_SUPPLY_HEALTH_OVERHEAT:
		chip->batt_hot = true;
		chip->batt_cold = false;
		chip->batt_warm = false;
		chip->batt_cool = false;
		break;
	case POWER_SUPPLY_HEALTH_COLD:
		chip->batt_hot = false;
		chip->batt_cold = true;
		chip->batt_cool = false;
		chip->batt_warm = false;
		break;
	case POWER_SUPPLY_HEALTH_WARM:
		chip->batt_hot = false;
		chip->batt_cold = false;
		chip->batt_warm = true;
		chip->batt_cool = false;
		break;
	case POWER_SUPPLY_HEALTH_COOL:
		chip->batt_hot = false;
		chip->batt_cold = false;
		chip->batt_warm = false;
		chip->batt_cool = true;
		break;
	default:
		pr_debug("health: %d, not a JEITA state\n", health);
		break;
	}
	pr_debug("hot: %d, cold: %d, warm = %d, cool = %d\n",
			chip->batt_hot, chip->batt_cold,
			chip->batt_warm, chip->batt_cool);

#ifndef APP_CONTROL
	disable = (chip->batt_hot || chip->batt_cold) ? true : false;
	rc = smb1351_battchg_disable(chip, THERMAL, disable);
	if (rc) {
		pr_err("%s charging for THERMAL failed, rc=%d\n",
				disable ? "Disable" : "Enable", rc);
		return;
	}

	rc = smb1351_chg_set_appropriate_battery_current(chip);
	if (rc) {
		pr_err("Set battery current failed\n");
		return;
	}
#endif

	rc = smb1351_chg_set_appropriate_vfloat(chip);
	if (rc) {
		pr_err("Set float voltage failed\n");
		return;
	}

	if (chip->use_external_fg) {
		smb1351_set_bms_property(chip, POWER_SUPPLY_PROP_HEALTH,
			smb1351_get_prop_batt_health(chip));
	}

	pr_debug("end!\n");
}

#define DEFAULT_CHARGE_FULL_DESIGN	5000000
static int get_prop_charge_full_design(struct smb1351_charger *chip)
{
	int uah, rc;

	rc = smb1351_get_bms_property(chip,
			POWER_SUPPLY_PROP_CHARGE_FULL, &uah);
	if (rc) {
		pr_err("Couldn't get full design rc = %d\n", rc);
		uah = DEFAULT_CHARGE_FULL_DESIGN;
	}

	if (uah == 0)
		uah = DEFAULT_CHARGE_FULL_DESIGN;

	return uah;
}

static int ibat_ocp_threshold_ua = 4500000;
module_param(ibat_ocp_threshold_ua, int, 0644);

#define UCONV			1000000LL
#define MCONV			1000LL
#define FLASH_V_THRESHOLD	3000000
#define FLASH_VDIP_MARGIN	100000
#define VPH_FLASH_VDIP		(FLASH_V_THRESHOLD + FLASH_VDIP_MARGIN)
#define BUCK_EFFICIENCY		800LL
#define BATT_PARASITIC_R_UOHM	100000
#define DEFAULT_VLED_MAX_UV	3500000
static int smb1351_calc_max_flash_current(struct smb1351_charger *chip)
{
	int ocv_uv, esr_uohm, rbatt_uohm, ibat_now, rc;
	int64_t ibat_flash_ua, avail_flash_ua, avail_flash_power_fw;
	int64_t ibat_safe_ua, vin_flash_uv, vph_flash_uv;

	rc = smb1351_get_bms_property(chip, POWER_SUPPLY_PROP_VOLTAGE_OCV,
							&ocv_uv);
	if (rc) {
		pr_err("Get OCV from BMS psy failed, rc=%d\n", rc);
		return 0;
	}

	rc = smb1351_get_bms_property(chip, POWER_SUPPLY_PROP_RESISTANCE,
							&esr_uohm);
	if (rc) {
		pr_err("Get ESR from BMS psy failed, rc=%d\n", rc);
		return 0;
	}

	rc = msm_bcl_read(BCL_PARAM_CURRENT, &ibat_now);
	if (rc) {
		pr_err("BCL current read failed: %d\n", rc);
		return 0;
	}

	rbatt_uohm = esr_uohm + BATT_PARASITIC_R_UOHM;

	ibat_safe_ua = div_s64((ocv_uv - VPH_FLASH_VDIP) * UCONV,
				rbatt_uohm);

	if (ibat_safe_ua <= ibat_ocp_threshold_ua) {
		/*
		 * If the calculated current is below the OCP threshold, then
		 * use it as the possible flash current.
		 */
		ibat_flash_ua = ibat_safe_ua - ibat_now;
		vph_flash_uv = VPH_FLASH_VDIP;
	} else {
		/*
		 * If the calculated current is above the OCP threshold, then
		 * use the ocp threshold instead.
		 *
		 * Any higher current will be tripping the battery OCP.
		 */
		ibat_flash_ua = ibat_ocp_threshold_ua - ibat_now;
		vph_flash_uv = ocv_uv - div64_s64((int64_t)rbatt_uohm
				* ibat_ocp_threshold_ua, UCONV);
	}
	/* Calculate the input voltage of the flash module. */
	vin_flash_uv = max((DEFAULT_VLED_MAX_UV + 500000LL),
				div64_s64((vph_flash_uv * 1200), 1000));
	/* Calculate the available power for the flash module. */
	avail_flash_power_fw = BUCK_EFFICIENCY * vph_flash_uv * ibat_flash_ua;
	/*
	 * Calculate the available amount of current the flash module can draw
	 * before collapsing the battery. (available power/ flash input voltage)
	 */
	avail_flash_ua = div64_s64(avail_flash_power_fw, vin_flash_uv * MCONV);
	pr_debug("avail_iflash=%lld, ocv=%d, ibat=%d, rbatt=%d\n",
		avail_flash_ua, ocv_uv, ibat_now, rbatt_uohm);

	return (int)avail_flash_ua;
}

static int smb1351_batt_property_is_writeable(struct power_supply *psy,
					enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_CAPACITY:
		return 1;
	default:
		break;
	}
	return 0;
}

static int smb1351_battery_set_property(struct power_supply *psy,
					enum power_supply_property prop,
					const union power_supply_propval *val)
{
	int rc;
	struct smb1351_charger *chip = container_of(psy,
				struct smb1351_charger, batt_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		if (!chip->bms_controlled_charging)
			return -EINVAL;
		switch (val->intval) {
		case POWER_SUPPLY_STATUS_FULL:
			rc = smb1351_battchg_disable(chip, SOC, true);
			if (rc) {
				pr_err("Couldn't disable charging  rc = %d\n",
									rc);
			} else {
				chip->batt_full = true;
				pr_debug("status = FULL, batt_full = %d\n",
							chip->batt_full);
			}
			break;
		case POWER_SUPPLY_STATUS_DISCHARGING:
			chip->batt_full = false;
			power_supply_changed(&chip->batt_psy);
			pr_debug("status = DISCHARGING, batt_full = %d\n",
							chip->batt_full);
			break;
		case POWER_SUPPLY_STATUS_CHARGING:
			rc = smb1351_battchg_disable(chip, SOC, false);
			if (rc) {
				pr_err("Couldn't enable charging rc = %d\n",
									rc);
			} else {
				chip->batt_full = false;
				pr_debug("status = CHARGING, batt_full = %d\n",
							chip->batt_full);
			}
			break;
		default:
			return -EINVAL;
		}
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		smb1351_usb_suspend(chip, USER, !val->intval);
		break;
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
		if (chip->is_factory_mode) {
			pr_err("factorymode need to disable chg, do not exc(%d)\n", val->intval);
			break;
		}
		smb1351_battchg_disable(chip, USER, !val->intval);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		chip->fake_battery_soc = val->intval;
		power_supply_changed(&chip->batt_psy);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		if (chip->fg_notify_jeita)
			smb1351_handle_jeita_from_fg(chip, val->intval);
		break;
	/* placeholder for flash w/a */
	case POWER_SUPPLY_PROP_FLASH_ACTIVE:
	case POWER_SUPPLY_PROP_FLASH_TRIGGER:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int smb1351_battery_get_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       union power_supply_propval *val)
{
	struct smb1351_charger *chip = container_of(psy,
				struct smb1351_charger, batt_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = smb1351_get_prop_batt_status(chip);
		if ((val->intval == POWER_SUPPLY_STATUS_FULL) && (chip->batt_warm))
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = smb1351_get_prop_batt_present(chip);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = smb1351_get_prop_batt_capacity(chip);
		if (val->intval < 1) {
			int vol = smb1351_get_prop_batt_voltage(chip);
			if (vol >= SMB1351_SHUTDOWN_BATTERY_VOLTGE)
				val->intval = 1;
		}
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = smb1351_get_prop_batt_voltage(chip);
		if (chip->is_factory_mode) {
			static int version = 0;
			union power_supply_propval pval = {0,};
			struct power_supply *parallel_psy = smb1351_get_parallel_slave(chip);

			pr_debug("in factory mode. check slave charger\n");
			if (!version) {
				parallel_psy->get_property(parallel_psy,
					POWER_SUPPLY_PROP_SERIAL_NUMBER, &pval);
				version = pval.intval;
				if (!version) {
					pr_err("in factory mode. slave revision is null. Show error in vbus voltage *-1\n");
					val->intval *= (-1);
				}
			}
		}
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = !chip->usb_suspended_status;
		break;
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
		val->intval = !chip->battchg_disabled_status;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = smb1351_get_prop_charge_type(chip);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = smb1351_get_prop_batt_health(chip);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = smb1351_get_prop_batt_temp(chip);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = "smb1351";
		break;
	case POWER_SUPPLY_PROP_FLASH_CURRENT_MAX:
		val->intval = smb1351_calc_max_flash_current(chip);
		break;
	case POWER_SUPPLY_PROP_FLASH_ACTIVE: /* placeholder for flash w/a */
	case POWER_SUPPLY_PROP_FLASH_TRIGGER:
		val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_REGULATION:
		val->intval = smb1351_get_vbus_voltage(chip);
		if (val->intval < 0)
			val->intval = DEFAULT_VBUS_VOLTAGE;
		break;
	case POWER_SUPPLY_PROP_HI_POWER:
		val->intval= chip->is_hv_fast_charging?
					1:0;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = get_prop_charge_full_design(chip);
		val->intval = DEFAULT_CHARGE_FULL_DESIGN;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = DEFAULT_CHARGE_FULL_DESIGN;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static enum power_supply_property smb1351_parallel_properties[] = {
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMITED,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_FLASH_CURRENT_MAX,
	POWER_SUPPLY_PROP_FLASH_ACTIVE,
	POWER_SUPPLY_PROP_FLASH_TRIGGER,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
};

static int smb1351_parallel_set_chg_present(struct smb1351_charger *chip,
						int present)
{
	int rc;
	u8 reg, mask = 0;

	pr_debug("set slave present = %d\n", present);
	if (present == chip->parallel_charger_present) {
		pr_debug("present %d -> %d, skipping\n",
				chip->parallel_charger_present, present);
		return 0;
	}

	chip->parallel_charger_present = present;

	if (present) {
		/* Check if SMB1351 is present */
		rc = smb1351_read_reg(chip, CHG_REVISION_REG, &reg);
		if (rc) {
			pr_debug("Failed to detect smb1351-parallel-charger, may be absent\n");
			return -ENODEV;
		}

		rc = smb_chip_get_version(chip);
		if (rc) {
			pr_err("Couldn't get version rc = %d\n", rc);
			return rc;
		}

		rc = smb1351_enable_volatile_writes(chip);
		if (rc) {
			pr_err("Couldn't configure for volatile rc = %d\n", rc);
			return rc;
		}

		/* set the float voltage */
		if (chip->vfloat_mv != -EINVAL) {
			rc = smb1351_float_voltage_set(chip, chip->vfloat_mv);
			if (rc) {
				pr_err("Couldn't set float voltage rc = %d\n",
									rc);
				return rc;
			}
		}

		/* set recharge-threshold and enable auto recharge */
		if (chip->recharge_mv != -EINVAL) {
			reg = AUTO_RECHG_ENABLE;
			if (chip->recharge_mv > 50)
				reg |= AUTO_RECHG_TH_100MV;
			else
				reg |= AUTO_RECHG_TH_50MV;

			rc = smb1351_masked_write(chip, CHG_CTRL_REG,
					AUTO_RECHG_BIT |
					AUTO_RECHG_TH_BIT, reg);
			if (rc) {
				pr_err("Couldn't set rechg-cfg rc = %d\n", rc);
				return rc;
			}
		}

		/* set chg en by pin active low  */
		reg = chip->parallel_pin_polarity_setting | USBCS_CTRL_BY_I2C;
		rc = smb1351_masked_write(chip, CHG_PIN_EN_CTRL_REG,
					EN_PIN_CTRL_MASK | USBCS_CTRL_BIT, reg);
		if (rc) {
			pr_err("Couldn't set en pin rc=%d\n", rc);
			return rc;
		}

		/* control USB suspend via command bits */
		rc = smb1351_masked_write(chip, VARIOUS_FUNC_REG,
						SUSPEND_MODE_CTRL_BIT,
						SUSPEND_MODE_CTRL_BY_I2C);
		if (rc) {
			pr_err("Couldn't set USB suspend rc=%d\n", rc);
			return rc;
		}
		/*
		 * setup USB 2.0/3.0 detection and USB 500/100
		 * command polarity
		 */
		reg = USB_2_3_MODE_SEL_BY_I2C | USB_CMD_POLARITY_500_1_100_0;
		mask = USB_2_3_MODE_SEL_BIT | USB_5_1_CMD_POLARITY_BIT;
		rc = smb1351_masked_write(chip,
				CHG_OTH_CURRENT_CTRL_REG, mask, reg);
		if (rc) {
			pr_err("Couldn't set CHG_OTH_CURRENT_CTRL_REG rc=%d\n",
					rc);
			return rc;
		}

		/* set fast charging current limit */
		chip->target_fastchg_current_max_ma = SMB1351_CHG_FAST_MIN_MA;
		rc = smb1351_fastchg_current_set(chip,
					chip->target_fastchg_current_max_ma);
		if (rc) {
			pr_err("Couldn't set fastchg current rc=%d\n", rc);
			return rc;
		}
		/*
		 * Suspend USB input (CURRENT reason) to avoid slave start
		 * charging before any SW logic been run. USB input will be
		 * un-suspended (CURRENT reason) after allotted ICL being set.
		 */
		chip->usb_psy_ma = SUSPEND_CURRENT_MA;
		rc = smb1351_usb_suspend(chip, CURRENT, true);
		if (rc) {
			pr_err("Suspend USB (CURRENT) failed, rc=%d\n", rc);
			return rc;
		}

		rc = smb1351_usb_suspend(chip, PARALLEL, false);
		if (rc) {
			pr_err("Suspend USB (PARALLEL) failed, rc=%d\n", rc);
			return rc;
		}

		schedule_delayed_work(&chip->heatbeat_work, HZ * 1);
	} else {
		rc = smb1351_usb_suspend(chip, PARALLEL, true);
		if (rc) {
			pr_debug("Suspend USB (PARALLEL) failed, rc=%d\n", rc);
			return rc;
		}

		cancel_delayed_work_sync(&chip->heatbeat_work);
	}

	return 0;
}

static bool smb1351_is_input_current_limited(struct smb1351_charger *chip)
{
	int rc;
	u8 reg;

	rc = smb1351_read_reg(chip, IRQ_H_REG, &reg);
	if (rc) {
		pr_err("Failed to read IRQ_H_REG for ICL status: %d\n", rc);
		return false;
	}

	return !!(reg & IRQ_IC_LIMIT_STATUS_BIT);
}

static int smb1351_parallel_set_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       const union power_supply_propval *val)
{
	int rc = 0;
	struct smb1351_charger *chip = container_of(psy,
				struct smb1351_charger, parallel_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		/*
		 *CHG EN is controlled by pin in the parallel charging.
		 *Use suspend if disable charging by command.
		 */
		if (chip->parallel_charger_present) {
			rc = smb1351_usb_suspend(chip, USER, !val->intval);
			if (rc)
				pr_err("%suspend charger failed\n",
						val->intval ? "Un-s" : "S");
		}
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		rc = smb1351_parallel_set_chg_present(chip, val->intval);
		if (rc)
			pr_err("Set charger %spresent failed\n",
					val->intval ? "" : "un-");
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		if (chip->parallel_charger_present) {
			chip->target_fastchg_current_max_ma =
						val->intval / 1000;
			rc = smb1351_fastchg_current_set(chip,
					chip->target_fastchg_current_max_ma);
		}
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		if (chip->parallel_charger_present) {
			rc = smb1351_set_usb_chg_current(chip,
					val->intval / 1000);
		}
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		if (chip->parallel_charger_present &&
			(chip->vfloat_mv != val->intval)) {
			rc = smb1351_float_voltage_set(chip, val->intval);
			if (!rc)
				chip->vfloat_mv = val->intval;
		} else {
			chip->vfloat_mv = val->intval;
		}
		break;
	default:
		return -EINVAL;
	}
	return rc;
}

static int smb1351_parallel_is_writeable(struct power_supply *psy,
				       enum power_supply_property prop)
{
	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		return 1;
	default:
		return 0;
	}
}

static int smb1351_parallel_get_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       union power_supply_propval *val)
{
	struct smb1351_charger *chip = container_of(psy,
				struct smb1351_charger, parallel_psy);
	int rc, icl;

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = !chip->usb_suspended_status;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		if (chip->parallel_charger_present) {
			rc = smb1351_get_usb_chg_current(chip, &icl);
			if (rc) {
				pr_err("Get ICL result failed, rc=%d\n", rc);
				return rc;
			}
			if (icl > 0)
				val->intval = icl * 1000;
			else
				val->intval = 0;
		} else {
			val->intval = 0;
		}
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = chip->vfloat_mv;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = chip->parallel_charger_present;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		if (chip->parallel_charger_present)
			val->intval = chip->fastchg_current_max_ma * 1000;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		if (chip->parallel_charger_present)
			val->intval = smb1351_get_prop_batt_status(chip);
		else
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMITED:
		if (chip->parallel_charger_present)
			val->intval =
				smb1351_is_input_current_limited(chip) ? 1 : 0;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_SERIAL_NUMBER:
		smb_chip_get_version(chip);
		pr_debug("smb1351 chip revision is %d\n", chip->version);
		val->intval = chip->version;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static void smb1351_chg_ctrl_in_jeita(struct smb1351_charger *chip)
{
	union power_supply_propval ret = {0, };
	int rc;

	/* enable the iterm to prevent the reverse boost */
	if (chip->iterm_disabled) {
		if (chip->batt_cool || chip->batt_warm) {
			rc = smb1351_iterm_set(chip, 100);
			pr_debug("set the iterm due to JEITA\n");
		} else {
			rc = smb1351_masked_write(chip, CHG_CTRL_REG,
						ITERM_EN_BIT, ITERM_DISABLE);
			pr_debug("disable the iterm when exits warm/cool\n");
		}
		if (rc) {
			pr_err("Couldn't set iterm rc = %d\n", rc);
			return;
		}
	}
	/*
	* When JEITA back to normal, the charging maybe disabled due to
	* the current termination. So re-enable the charging if the soc
	* is less than 100 in the normal mode. A 200ms delay is requred
	* before the disabe and enable operation.
	*/
	if (chip->bms_psy) {
		rc = chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_CAPACITY, &ret);
		if (rc) {
			pr_err("Couldn't read the bms capacity rc = %d\n",
									rc);
			return;
		}
		if (!chip->batt_cool && !chip->batt_warm
				&& !chip->batt_cold && !chip->batt_hot
				&& ret.intval < 100) {
			rc = smb1351_battchg_disable(chip, THERMAL, true);
			if (rc) {
				pr_err("Couldn't disable charging rc = %d\n",
									rc);
				return;
			}
			/* delay for resetting the charging */
			msleep(200);
			rc = smb1351_battchg_disable(chip, THERMAL, false);
			if (rc) {
				pr_err("Couldn't enable charging rc = %d\n",
									rc);
				return;
			} else {
				chip->batt_full = false;
				pr_debug("re-enable charging, batt_full = %d\n",
							chip->batt_full);
			}
			pr_debug("batt psy changed\n");
			power_supply_changed(&chip->batt_psy);
		}
	}
}

static void smb1351_reset(struct smb1351_charger *chip) {
	int rc;

	rc = smb1351_masked_write(chip, CMD_I2C_REG,
				CMD_RELOAD_BIT, CMD_RELOAD_BIT);
	if (rc) {
		pr_err("%s, Couldn't reset charger = %d\n", dev_name(chip->dev), rc);
	} else {
		pr_info("%s, reset charger\n", dev_name(chip->dev));
	}
}

#define HYSTERESIS_DECIDEGC 20
static void smb1351_chg_adc_notification(enum qpnp_tm_state state, void *ctx)
{
	struct smb1351_charger *chip = ctx;
	struct battery_status *cur;
	int temp;

	if (state >= ADC_TM_STATE_NUM) {
		pr_err("invalid state parameter %d\n", state);
		return;
	}

	temp = smb1351_get_prop_batt_temp(chip);

	pr_debug("temp = %d state = %s\n", temp,
				state == ADC_TM_WARM_STATE ? "hot" : "cold");

	/* reset the adc status request */
	chip->adc_param.state_request = ADC_TM_WARM_COOL_THR_ENABLE;

	/* temp from low to high */
	if (state == ADC_TM_WARM_STATE) {
		/* WARM -> HOT */
		if (temp >= chip->batt_hot_decidegc) {
			cur = &batt_s[BATT_HOT];
			chip->adc_param.low_temp =
				chip->batt_hot_decidegc - HYSTERESIS_DECIDEGC;
			chip->adc_param.state_request =	ADC_TM_COOL_THR_ENABLE;
		/* NORMAL -> WARM */
		} else if (temp >= chip->batt_warm_decidegc &&
					chip->jeita_supported) {
			cur = &batt_s[BATT_WARM];
			chip->adc_param.low_temp =
				chip->batt_warm_decidegc - HYSTERESIS_DECIDEGC;
			chip->adc_param.high_temp = chip->batt_hot_decidegc;
		/* COOL -> NORMAL */
		} else if (temp >= chip->batt_cool_decidegc &&
					chip->jeita_supported) {
			cur = &batt_s[BATT_NORMAL];
			chip->adc_param.low_temp =
				chip->batt_cool_decidegc - HYSTERESIS_DECIDEGC;
			chip->adc_param.high_temp = chip->batt_warm_decidegc;
		/* COLD -> COOL */
		} else if (temp >= chip->batt_cold_decidegc) {
			cur = &batt_s[BATT_COOL];
			chip->adc_param.low_temp =
				chip->batt_cold_decidegc - HYSTERESIS_DECIDEGC;
			if (chip->jeita_supported)
				chip->adc_param.high_temp =
						chip->batt_cool_decidegc;
			else
				chip->adc_param.high_temp =
						chip->batt_hot_decidegc;
		/* MISSING -> COLD */
		} else if (temp >= chip->batt_missing_decidegc) {
			cur = &batt_s[BATT_COLD];
			chip->adc_param.high_temp = chip->batt_cold_decidegc;
			chip->adc_param.low_temp = chip->batt_missing_decidegc
							- HYSTERESIS_DECIDEGC;

		}
	/* temp from high to low */
	} else {
		/* COLD -> MISSING */
		if (temp <= chip->batt_missing_decidegc) {
			cur = &batt_s[BATT_MISSING];
			chip->adc_param.high_temp = chip->batt_missing_decidegc
							+ HYSTERESIS_DECIDEGC;
			chip->adc_param.state_request = ADC_TM_WARM_THR_ENABLE;
		/* COOL -> COLD */
		} else if (temp <= chip->batt_cold_decidegc) {
			cur = &batt_s[BATT_COLD];
			chip->adc_param.high_temp =
				chip->batt_cold_decidegc + HYSTERESIS_DECIDEGC;
			/* add low_temp to enable batt present check */
			chip->adc_param.low_temp = chip->batt_missing_decidegc;
		/* NORMAL -> COOL */
		} else if (temp <= chip->batt_cool_decidegc &&
					chip->jeita_supported) {
			cur = &batt_s[BATT_COOL];
			chip->adc_param.high_temp =
				chip->batt_cool_decidegc + HYSTERESIS_DECIDEGC;
			chip->adc_param.low_temp = chip->batt_cold_decidegc;
		/* WARM -> NORMAL */
		} else if (temp <= chip->batt_warm_decidegc &&
					chip->jeita_supported) {
			cur = &batt_s[BATT_NORMAL];
			chip->adc_param.high_temp =
				chip->batt_warm_decidegc + HYSTERESIS_DECIDEGC;
			chip->adc_param.low_temp = chip->batt_cool_decidegc;
		/* HOT -> WARM */
		} else if (temp <= chip->batt_hot_decidegc) {
			cur = &batt_s[BATT_WARM];
			if (chip->jeita_supported)
				chip->adc_param.low_temp =
					chip->batt_warm_decidegc;
			else
				chip->adc_param.low_temp =
					chip->batt_cold_decidegc;
			chip->adc_param.high_temp =
				chip->batt_hot_decidegc + HYSTERESIS_DECIDEGC;
		}
	}

	if (cur->batt_present)
		chip->battery_missing = false;
	else
		chip->battery_missing = true;

	if (cur->batt_hot ^ chip->batt_hot ||
			cur->batt_cold ^ chip->batt_cold) {
		chip->batt_hot = cur->batt_hot;
		chip->batt_cold = cur->batt_cold;
		/* stop charging explicitly since we use PMIC thermal pin*/
		if (cur->batt_hot || cur->batt_cold ||
							chip->battery_missing)
			smb1351_battchg_disable(chip, THERMAL, 1);
		else
			smb1351_battchg_disable(chip, THERMAL, 0);
	}

	if ((chip->batt_warm ^ cur->batt_warm ||
				chip->batt_cool ^ cur->batt_cool)
						&& chip->jeita_supported) {
		chip->batt_warm = cur->batt_warm;
		chip->batt_cool = cur->batt_cool;
		smb1351_chg_set_appropriate_battery_current(chip);
		smb1351_chg_set_appropriate_vfloat(chip);
		smb1351_chg_ctrl_in_jeita(chip);
	}

	pr_debug("hot %d, cold %d, warm %d, cool %d, soft jeita supported %d, missing %d, low = %d deciDegC, high = %d deciDegC\n",
		chip->batt_hot, chip->batt_cold, chip->batt_warm,
		chip->batt_cool, chip->jeita_supported,
		chip->battery_missing, chip->adc_param.low_temp,
		chip->adc_param.high_temp);
	if (qpnp_adc_tm_channel_measure(chip->adc_tm_dev, &chip->adc_param))
		pr_err("request ADC error\n");
}

static void smb1351_rerun_apsd_work(struct work_struct *work)
{
	int rc;
	struct smb1351_charger *chip = container_of(work,
						struct smb1351_charger,
						rerun_apsd_work.work);
	enum power_supply_type type = POWER_SUPPLY_TYPE_UNKNOWN;

	pr_debug("Rerunning APSD\n");

	rc = smb1351_get_usb_supply_type(chip, &type);
	if ((type == POWER_SUPPLY_TYPE_USB) ||(type == POWER_SUPPLY_TYPE_USB_CDP)
		 ||(type == POWER_SUPPLY_TYPE_UNKNOWN)) {
	chip->apsd_rerun = true;
	rc = smb1351_masked_write(chip, CMD_HVDCP_REG, CMD_APSD_RE_RUN_BIT,
						CMD_APSD_RE_RUN_BIT);
	if (rc)
		pr_err("Couldn't re-run APSD algo, rc = %d\n", rc);
	}

	smb1351_relax(&chip->smb1351_ws, RERUN_APSD);
}

static void smb1351_hvdcp_det_work(struct work_struct *work)
{
	int rc;
	u8 reg;
	bool is_hvdcp;
	struct smb1351_charger *chip = container_of(work,
						struct smb1351_charger,
						hvdcp_det_work.work);

	rc = smb1351_read_reg(chip, STATUS_7_REG, &reg);
	if (rc) {
		pr_err("Couldn't read STATUS_7_REG rc == %d\n", rc);
		goto end;
	}
	pr_debug("STATUS_7_REG = 0x%02X\n", reg);

	is_hvdcp = !!(reg & (HVDCP_SEL_5V | HVDCP_SEL_9V | HVDCP_SEL_12V));
	if (is_hvdcp) {
		pr_debug("HVDCP detected; notifying USB PSY\n");
		power_supply_set_supply_type(chip->usb_psy,
			POWER_SUPPLY_TYPE_USB_HVDCP);
	}
end:
	smb1351_relax(&chip->smb1351_ws, HVDCP_DETECT);
}

#define HVDCP_NOTIFY_MS 2500
static int smb1351_apsd_complete_handler(struct smb1351_charger *chip,
						u8 status)
{
	int rc = 0;
	enum power_supply_type type = POWER_SUPPLY_TYPE_UNKNOWN;
	struct power_supply *parallel_psy = smb1351_get_parallel_slave(chip);

	/*
	 * If apsd is disabled, charger detection is done by
	 * USB phy driver.
	 */
	if (chip->disable_apsd || chip->usbin_ov) {
		pr_debug("APSD %s, status = %d\n",
			chip->disable_apsd ? "disabled" : "enabled", !!status);
		pr_debug("USBIN ov, status = %d\n", chip->usbin_ov);
		return 0;
	}

	if (status) {
		chip->chg_present = true;
		rc = smb1351_get_usb_supply_type(chip, &type);
		if (rc) {
			pr_err("Get USB power supply type failed, rc=%d\n", rc);
			return rc;
		}
		pr_debug("APSD complete. USB type detected=%d chg_present=%d\n",
						type, chip->chg_present);
		/*
		 * If defined force hvdcp 2p0 property,
		 * we force to hvdcp 2p0 in the APSD handler.
		 */
		if (chip->force_hvdcp_2p0) {
			pr_debug("Force set to HVDCP 2.0 mode\n");
			smb1351_masked_write(chip, VARIOUS_FUNC_3_REG,
						QC_2P1_AUTH_ALGO_BIT, 0);
			smb1351_masked_write(chip, CMD_HVDCP_REG,
						CMD_FORCE_HVDCP_2P0_BIT,
						CMD_FORCE_HVDCP_2P0_BIT);
			type = POWER_SUPPLY_TYPE_USB_HVDCP;
		} else if (type == POWER_SUPPLY_TYPE_USB_DCP) {
			pr_debug("schedule hvdcp detection worker\n");
			smb1351_stay_awake(&chip->smb1351_ws, HVDCP_DETECT);
			if (!wake_lock_active(&chip->charging_lock))
				wake_lock(&chip->charging_lock);

			schedule_delayed_work(&chip->hvdcp_det_work,
					msecs_to_jiffies(HVDCP_NOTIFY_MS));
		}

		power_supply_set_supply_type(chip->usb_psy, type);
		/*
		 * SMB is now done sampling the D+/D- lines,
		 * indicate USB driver
		 */
		pr_debug("updating usb_psy present=%d\n", chip->chg_present);
		power_supply_set_present(chip->usb_psy, chip->chg_present);
		power_supply_set_online(chip->usb_psy, chip->chg_present);
		chip->apsd_rerun = false;
		/* set parallel slave PRESENT */
		if (parallel_psy) {
			pr_debug("set parallel charger present!\n");
			rc = power_supply_set_present(parallel_psy, true);
			if (rc)
				pr_debug("parallel slave absent!\n");
			else
				chip->parallel.slave_detected = true;
		}

		schedule_delayed_work(&chip->heatbeat_work, (HZ * 1));
	}

	if(chip->is_factory_mode) {
		schedule_delayed_work(&chip->charger_check_work, CHARGER_CHECK_JIFFIES);
	}

	return rc;
}

/*
 * As source detect interrupt is not triggered on the falling edge,
 * we need to schedule a work for checking source detect status after
 * charger UV interrupt fired.
 */
#define FIRST_CHECK_DELAY_MS	100
#define SECOND_CHECK_DELAY_MS	1000
#define RERUN_CHG_REMOVE_DELAY_MS	1500
static void smb1351_chg_remove_work(struct work_struct *work)
{
	int rc, vbus_uv = 0;
	u8 reg;
	struct smb1351_charger *chip = container_of(work,
				struct smb1351_charger, chg_remove_work.work);
	struct power_supply *parallel_psy = smb1351_get_parallel_slave(chip);

	if(chip->chg_present == false) {
		pr_debug("chg already removed\n");
		goto end;
	}

	rc = smb1351_read_reg(chip, IRQ_G_REG, &reg);
	if (rc) {
		pr_err("Couldn't read IRQ_G_REG rc = %d\n", rc);
	}

	vbus_uv = smb1351_get_vbus_voltage(chip);

	if (((!(reg & IRQ_SOURCE_DET_BIT) ) && (rc == 0))
		||((chip->chg_remove_work_scheduled >= 5) && (vbus_uv < 2000000) && (vbus_uv > 0))) {
		pr_debug("chg removed, vbus_uv = %d\n",vbus_uv);
		chip->chg_present = false;
		chip->batt_full = false;
		smb1351_battchg_disable(chip, SOC, false);
		smb1351_set_bms_property(chip, POWER_SUPPLY_PROP_STATUS,
				smb1351_get_prop_batt_status(chip));
		/* clear parallel slave PRESENT */
		if (parallel_psy && chip->parallel.slave_detected) {
			pr_debug("set parallel charger un-present!\n");
			power_supply_set_present(parallel_psy, false);
		}
		power_supply_set_supply_type(chip->usb_psy,
						POWER_SUPPLY_TYPE_UNKNOWN);
		power_supply_set_present(chip->usb_psy,
						chip->chg_present);
		power_supply_set_online(chip->usb_psy,
						chip->chg_present);

		pr_debug("Set usb psy dp=r dm=r\n");
		power_supply_set_dp_dm(chip->usb_psy,
				POWER_SUPPLY_DP_DM_DPR_DMR);
		chip->apsd_rerun = false;

		if (wake_lock_active(&chip->charging_lock))
			wake_unlock(&chip->charging_lock);

		cancel_delayed_work_sync(&chip->heatbeat_work);
		chip->is_hv_fast_charging = false;

		if(chip->is_factory_mode) {
			schedule_delayed_work(&chip->charger_check_work, CHARGER_CHECK_JIFFIES);
		}
	} else if (chip->chg_remove_work_scheduled < 5) {
		chip->chg_remove_work_scheduled ++;
		pr_debug("chg_remove_work_scheduled = %d\n",chip->chg_remove_work_scheduled);
		goto reschedule;
	} else {
		pr_debug("charger is present\n");
	}
end:
	chip->chg_remove_work_scheduled = 0;
	smb1351_relax(&chip->smb1351_ws, REMOVAL_DETECT);
	return;

reschedule:
	pr_debug("reschedule after 1s\n");
	schedule_delayed_work(&chip->chg_remove_work,
				msecs_to_jiffies(SECOND_CHECK_DELAY_MS));
}

static void smb1351_charger_check_worker(struct work_struct *work)
{
	struct smb1351_charger *chip = container_of(work, struct smb1351_charger, charger_check_work.work);
	union power_supply_propval charger_type = {0, };

	if (!chip->is_factory_cable) {
		chip->usb_psy->get_property(chip->usb_psy,
			POWER_SUPPLY_PROP_TYPE, &charger_type);
		if((charger_type.intval==POWER_SUPPLY_TYPE_USB)&&(chip->is_factory_mode))
			chip->is_factory_cable = true;
		else
			chip->is_factory_cable = false;

		pr_info("CHG %s, charger type %d is_factorymode %d is_cable %d\n", __func__,
			charger_type.intval, chip->is_factory_mode, chip->is_factory_cable);
	} else {
		if (chip->chg_present)
			return;

#ifdef MMI_TEST
		//if (!factory_kill_disable && !reboot_in_progress()) {
		if (!factory_kill_disable ) {
#else
		if (1) {
#endif
			pr_info("CHG - Factory Cable removed, power-off\n");
			kernel_power_off();
		} else
			pr_info("CHG - Factory cable removed - kill disabled\n");

		chip->is_factory_cable = false;
	}
}

static int smb1351_usbin_uv_handler(struct smb1351_charger *chip, u8 status)
{
	int chg_remove_delay = FIRST_CHECK_DELAY_MS;
	struct power_supply *parallel_psy = smb1351_get_parallel_slave(chip);
	int rc;

	pr_debug("enter, status = %d\n", !!status);
	/* use this to detect USB insertion only if !apsd */
	if (chip->disable_apsd) {
		/*
		 * If APSD is disabled, src det interrupt won't trigger.
		 * Hence use usbin_uv for removal and insertion notification
		 */
		if (status == 0) {
			chip->chg_present = true;
			pr_debug("updating usb_psy present=%d\n",
						chip->chg_present);
			power_supply_set_supply_type(chip->usb_psy,
						POWER_SUPPLY_TYPE_USB);
			power_supply_set_present(chip->usb_psy,
						chip->chg_present);
			/* set parallel slave PRESENT */
			if (parallel_psy) {
				rc = power_supply_set_present(
						parallel_psy, true);
				if (rc)
					pr_debug("parallel slave absent!\n");
				else
					chip->parallel.slave_detected = true;
			}
		} else {
			chip->chg_present = false;
			/* set parallel slave UN-PRESENT */
			if (parallel_psy && chip->parallel.slave_detected)
				power_supply_set_present(parallel_psy, false);
			power_supply_set_supply_type(chip->usb_psy,
						POWER_SUPPLY_TYPE_UNKNOWN);
			power_supply_set_present(chip->usb_psy,
						chip->chg_present);
			pr_debug("updating usb_psy present=%d\n",
							chip->chg_present);
		}
		return 0;
	}

	if (status) {
		cancel_delayed_work_sync(&chip->hvdcp_det_work);
		if (chip->wa_flags & CHG_ITERM_CHECK_SOC)
			cancel_delayed_work_sync(&chip->iterm_check_soc_work);
		smb1351_relax(&chip->smb1351_ws, REMOVAL_DETECT);
		pr_debug("schedule charger remove worker\n");
		if (chip->apsd_rerun)
			chg_remove_delay = RERUN_CHG_REMOVE_DELAY_MS;
		smb1351_stay_awake(&chip->smb1351_ws, REMOVAL_DETECT);
		schedule_delayed_work(&chip->chg_remove_work,
					msecs_to_jiffies(chg_remove_delay));
	} else {
		cancel_delayed_work_sync(&chip->chg_remove_work);
		smb1351_relax(&chip->smb1351_ws, REMOVAL_DETECT);
		pr_debug("Set usb psy dp=f dm=f\n");
		power_supply_set_dp_dm(chip->usb_psy,
					POWER_SUPPLY_DP_DM_DPF_DMF);
	}

	pr_info("chip->chg_present = %d\n", chip->chg_present);

	return 0;
}

static int smb1351_usbin_ov_handler(struct smb1351_charger *chip, u8 status)
{
	struct power_supply *parallel_psy = smb1351_get_parallel_slave(chip);
	int health;
	int rc;
	u8 reg;

	pr_debug("enter, status = %d\n", !!status);
	rc = smb1351_read_reg(chip, IRQ_E_REG, &reg);
	if (rc)
		pr_err("Couldn't read IRQ_E rc = %d\n", rc);

	if (status != 0) {
		chip->chg_present = false;
		chip->usbin_ov = true;
		/* set parallel slave UN-PRESENT */
		if (parallel_psy && chip->parallel.slave_detected)
			power_supply_set_present(parallel_psy, false);
		power_supply_set_supply_type(chip->usb_psy,
						POWER_SUPPLY_TYPE_UNKNOWN);
		power_supply_set_present(chip->usb_psy, chip->chg_present);
	} else {
		chip->usbin_ov = false;
		if (reg & IRQ_USBIN_UV_BIT)
			pr_debug("Charger unplugged from OV\n");
		else
			smb1351_apsd_complete_handler(chip, 1);
	}

	if (chip->usb_psy) {
		health = status ? POWER_SUPPLY_HEALTH_OVERVOLTAGE
					: POWER_SUPPLY_HEALTH_GOOD;
		power_supply_set_health_state(chip->usb_psy, health);
		pr_debug("chip ov status is %d\n", health);
	}
	pr_debug("chip->chg_present = %d\n", chip->chg_present);

	return 0;
}

static int smb1351_usbid_handler(struct smb1351_charger *chip, u8 status)
{
	bool usbid_gnd;
	int rc;

	if (!!status) {
		rc = smb1351_get_usbid_status(chip, &usbid_gnd);
		if (rc) {
			pr_err("Get usbid failed!");
			return 0;
		}
		pr_debug("usbid grounded: %d\n", usbid_gnd);
		if (chip->usb_psy)
			power_supply_set_usb_otg(chip->usb_psy, usbid_gnd);
	}
	if (chip->use_external_fg)
		smb1351_set_bms_property(chip, POWER_SUPPLY_PROP_STATUS,
			smb1351_get_prop_batt_status(chip));

	return 0;
}

static int smb1351_fast_chg_handler(struct smb1351_charger *chip, u8 status)
{
	pr_debug("enter, status = 0x%02x\n", status);
	chip->check_parallel = true;
	if (chip->use_external_fg)
		smb1351_set_bms_property(chip, POWER_SUPPLY_PROP_STATUS,
				smb1351_get_prop_batt_status(chip));
	return 0;
}

static int smb1351_taper_handler(struct smb1351_charger *chip, u8 status)
{
	pr_debug("enter, status = 0x%02x\n", status);
	if (!!status)
		smb1351_parallel_charger_taper_attempt(chip);

	if (chip->use_external_fg)
		smb1351_set_bms_property(chip, POWER_SUPPLY_PROP_STATUS,
				smb1351_get_prop_batt_status(chip));
	return 0;
}

static int smb1351_recharge_handler(struct smb1351_charger *chip, u8 status)
{
	pr_debug("enter, status = 0x%02x\n", status);
//	if (!chip->bms_controlled_charging)
//		chip->batt_full = !status;
	chip->check_parallel = true;

	if (chip->use_external_fg)
		smb1351_set_bms_property(chip, POWER_SUPPLY_PROP_STATUS,
				smb1351_get_prop_batt_status(chip));
	return 0;
}

#define FULL_SOC		100
#define ITERM_CHECK_MS		10000
#define ITERM_RECHECK_COUNT	3
static void smb1351_iterm_check_soc_work(struct work_struct *work)
{
	int rc, current_ua, soc;
	struct smb1351_charger *chip = container_of(work,
			struct smb1351_charger, iterm_check_soc_work.work);

	if (!chip->chg_present) {
		pr_debug("Plugged out, ignore!\n");
		goto done;
	}

	rc = smb1351_set_bms_property(chip,
			POWER_SUPPLY_PROP_UPDATE_NOW, 1);
	if (rc) {
		pr_err("Update bms status failed, rc=%d\n", rc);
		goto recheck;
	}

	rc = smb1351_get_bms_property(chip,
			POWER_SUPPLY_PROP_CURRENT_NOW, &current_ua);
	if (rc) {
		pr_err("Get bms current now failed, rc=%d\n", rc);
		goto recheck;
	}

	pr_debug("current: %d ua!", current_ua);
	if (current_ua >= 0) {
		pr_debug("Terminated? large current consumption?");
		goto recheck;
	}

	if (-1 * current_ua / 1000 < chip->iterm_ma) {
		pr_debug("Charging current is less than iterm, terminate!");
		chip->batt_full = true;
		goto done;
	}

	rc = smb1351_get_bms_property(chip, POWER_SUPPLY_PROP_CAPACITY, &soc);
	if (rc) {
		pr_err("Get bms capacity failed, rc=%d\n", rc);
		goto recheck;
	}

	if (soc == FULL_SOC) {
		pr_debug("FG has terminated, terminate charger!\n");
		chip->batt_full = true;
		goto done;
	} else {
		goto recheck;
	}
done:
	if (chip->batt_full) {
		smb1351_set_bms_property(chip, POWER_SUPPLY_PROP_STATUS,
				smb1351_get_prop_batt_status(chip));
		if (chip->wa_flags & CHG_SOC_BASED_RESUME) {
			rc = smb1351_battchg_disable(chip, SOC, true);
			if (rc) {
				pr_err("Disable charger for SOC failed, rc=%d\n",
						rc);
				goto recheck;
			}
		}
	}

	rc = smb1351_masked_write(chip, CHG_CTRL_REG,
			ITERM_EN_BIT, ITERM_ENABLE);
	if (rc) {
		pr_err("Re-enable iterm failed, rc=%d\n", rc);
		goto recheck;
	}

	chip->iterm_disabled = false;
	smb1351_relax(&chip->smb1351_ws, ITERM_CHECK_SOC);

	return;

recheck:
	schedule_delayed_work(&chip->iterm_check_soc_work,
			msecs_to_jiffies(ITERM_CHECK_MS));
}

static int smb1351_chg_term_handler(struct smb1351_charger *chip, u8 status)
{
	int rc, soc;

	pr_debug("enter, status = 0x%02x\n", status);
	if (!chip->bms_controlled_charging &&
			!(chip->wa_flags & CHG_ITERM_CHECK_SOC))
		chip->batt_full = !!status;

	/* Only check parallel when falling */
	if (!status)
		chip->check_parallel = true;

	if (!(chip->wa_flags & CHG_ITERM_CHECK_SOC))
		return 0;

	if (!!status && !chip->batt_full) {
		rc = smb1351_get_bms_property(chip,
				POWER_SUPPLY_PROP_CAPACITY, &soc);
		if (rc) {
			pr_err("Get capacity failed, rc=%d\n", rc);
			return rc;
		}
		if (soc != FULL_SOC) {
			pr_debug("FG haven't reported FULL, soc=%d\n", soc);
			/*
			 * disable termination, restart charging,
			 * hold a wakelock and start a timer for
			 * checking SOC until the SOC reach to
			 * FULL_SOC.
			 */
			rc = smb1351_masked_write(chip, CHG_CTRL_REG,
					ITERM_EN_BIT, ITERM_DISABLE);
			if (rc) {
				pr_err("disable iterm failed, rc=%d", rc);
				return rc;
			}
			chip->iterm_disabled = true;
			/* toggle charging disble bit for re-charge */
			rc = smb1351_battchg_disable(chip, ITERM, true);
			if (rc) {
				pr_err("Disable charger for ITERM failed, rc=%d\n",
								rc);
				return rc;
			}
			rc = smb1351_battchg_disable(chip, ITERM, false);
			if (rc) {
				pr_err("Enable charger for ITERM failed, rc=%d\n",
								rc);
				return rc;
			}
			smb1351_stay_awake(&chip->smb1351_ws, ITERM_CHECK_SOC);
			schedule_delayed_work(&chip->iterm_check_soc_work, 0);
		} else {
			pr_debug("Terminated, FG report full\n");
			chip->batt_full = true;
		}
	}

	return 0;
}

static int smb1351_safety_timeout_handler(struct smb1351_charger *chip,
						u8 status)
{
	pr_debug("enter, status = 0x%02x\n", status);
	chip->check_parallel = true;
	if (chip->use_external_fg)
		smb1351_set_bms_property(chip, POWER_SUPPLY_PROP_STATUS,
				smb1351_get_prop_batt_status(chip));
	return 0;
}

static int smb1351_chg_error_handler(struct smb1351_charger *chip,
						u8 status)
{
	pr_debug("enter, status = 0x%02x\n", status);
	chip->check_parallel = true;
	if (chip->use_external_fg)
		smb1351_set_bms_property(chip, POWER_SUPPLY_PROP_STATUS,
				smb1351_get_prop_batt_status(chip));
	return 0;
}

static int smb1351_aicl_done_handler(struct smb1351_charger *chip, u8 status)
{
	pr_debug("enter, status = 0x%02x\n", status);
	chip->check_parallel = true;
	return 0;
}

static int smb1351_hot_hard_handler(struct smb1351_charger *chip, u8 status)
{
	pr_debug("enter, status = 0x%02x\n", status);
	chip->batt_hot = !!status;
	chip->check_parallel = true;
	return 0;
}
static int smb1351_cold_hard_handler(struct smb1351_charger *chip, u8 status)
{
	pr_debug("enter, status = 0x%02x\n", status);
	chip->batt_cold = !!status;
	chip->check_parallel = true;
	return 0;
}
static int smb1351_hot_soft_handler(struct smb1351_charger *chip, u8 status)
{
	pr_debug("enter, status = 0x%02x\n", status);
	chip->batt_warm = !!status;
	chip->check_parallel = true;
	return 0;
}
static int smb1351_cold_soft_handler(struct smb1351_charger *chip, u8 status)
{
	pr_debug("enter, status = 0x%02x\n", status);
	chip->batt_cool = !!status;
	chip->check_parallel = true;
	return 0;
}

static int smb1351_battery_missing_handler(struct smb1351_charger *chip,
						u8 status)
{
	pr_debug("enter, status = 0x%02x\n", status);
	if (status)
		chip->battery_missing = true;
	else
		chip->battery_missing = false;

	if (chip->use_external_fg)
		smb1351_set_bms_property(chip, POWER_SUPPLY_PROP_STATUS,
				smb1351_get_prop_batt_status(chip));
	return 0;
}

static struct irq_handler_info handlers[] = {
	[0] = {
		.stat_reg	= IRQ_A_REG,
		.val		= 0,
		.prev_val	= 0,
		.irq_info	= {
			{	.name	 = "cold_soft",
				.smb_irq = smb1351_cold_soft_handler,
			},
			{	.name	 = "hot_soft",
				.smb_irq = smb1351_hot_soft_handler,
			},
			{	.name	 = "cold_hard",
				.smb_irq = smb1351_cold_hard_handler,
			},
			{	.name	 = "hot_hard",
				.smb_irq = smb1351_hot_hard_handler,
			},
		},
	},
	[1] = {
		.stat_reg	= IRQ_B_REG,
		.val		= 0,
		.prev_val	= 0,
		.irq_info	= {
			{	.name	 = "internal_temp_limit",
			},
			{	.name	 = "vbatt_low",
			},
			{	.name	 = "battery_missing",
				.smb_irq = smb1351_battery_missing_handler,
			},
			{	.name	 = "batt_therm_removed",
			},
		},
	},
	[2] = {
		.stat_reg	= IRQ_C_REG,
		.val		= 0,
		.prev_val	= 0,
		.irq_info	= {
			{	.name	 = "chg_term",
				.smb_irq = smb1351_chg_term_handler,
			},
			{	.name	 = "taper",
				.smb_irq = smb1351_taper_handler,
			},
			{	.name	 = "recharge",
				.smb_irq = smb1351_recharge_handler,
			},
			{	.name	 = "fast_chg",
				.smb_irq = smb1351_fast_chg_handler,
			},
		},
	},
	[3] = {
		.stat_reg	= IRQ_D_REG,
		.val		= 0,
		.prev_val	= 0,
		.irq_info	= {
			{	.name	 = "prechg_timeout",
			},
			{	.name	 = "safety_timeout",
				.smb_irq = smb1351_safety_timeout_handler,
			},
			{	.name	 = "chg_error",
				.smb_irq = smb1351_chg_error_handler,
			},
			{	.name	 = "batt_ov",
			},
		},
	},
	[4] = {
		.stat_reg	= IRQ_E_REG,
		.val		= 0,
		.prev_val	= 0,
		.irq_info	= {
			{	.name	 = "power_ok",
			},
			{	.name	 = "afvc",
			},
			{	.name	 = "usbin_uv",
				.smb_irq = smb1351_usbin_uv_handler,
			},
			{	.name	 = "usbin_ov",
				.smb_irq = smb1351_usbin_ov_handler,
			},
		},
	},
	[5] = {
		.stat_reg	= IRQ_F_REG,
		.val		= 0,
		.prev_val	= 0,
		.irq_info	= {
			{	.name	 = "otg_oc_retry",
			},
			{	.name	 = "rid",
				.smb_irq = smb1351_usbid_handler,
			},
			{	.name	 = "otg_fail",
			},
			{	.name	 = "otg_oc",
			},
		},
	},
	[6] = {
		.stat_reg	= IRQ_G_REG,
		.val		= 0,
		.prev_val	= 0,
		.irq_info	= {
			{	.name	 = "chg_inhibit",
			},
			{	.name	 = "aicl_fail",
			},
			{	.name	 = "aicl_done",
				.smb_irq = smb1351_aicl_done_handler,
			},
			{	.name	 = "apsd_complete",
				.smb_irq = smb1351_apsd_complete_handler,
			},
		},
	},
	[7] = {
		.stat_reg	= IRQ_H_REG,
		.val		= 0,
		.prev_val	= 0,
		.irq_info	= {
			{	.name	 = "wdog_timeout",
			},
			{	.name	 = "hvdcp_auth_done",
			},
		},
	},
};

#define IRQ_LATCHED_MASK	0x02
#define IRQ_STATUS_MASK		0x01
#define BITS_PER_IRQ		2
static irqreturn_t smb1351_chg_stat_handler(int irq, void *dev_id)
{
	struct smb1351_charger *chip = dev_id;
	int i, j;
	u8 triggered;
	u8 changed;
	u8 rt_stat, prev_rt_stat;
	int rc;
	int handler_count = 0;

	pr_debug("%s IRQ triggered\n", __func__);

	mutex_lock(&chip->irq_complete);

	chip->irq_waiting = true;
	if (!chip->resume_completed) {
		pr_debug("IRQ triggered before device-resume\n");
		disable_irq_nosync(irq);
		mutex_unlock(&chip->irq_complete);
		return IRQ_HANDLED;
	}
	chip->irq_waiting = false;

	for (i = 0; i < ARRAY_SIZE(handlers); i++) {
		rc = smb1351_read_reg(chip, handlers[i].stat_reg,
						&handlers[i].val);
		if (rc) {
			pr_err("Couldn't read %d rc = %d\n",
					handlers[i].stat_reg, rc);
			continue;
		}

		for (j = 0; j < ARRAY_SIZE(handlers[i].irq_info); j++) {
			triggered = handlers[i].val
			       & (IRQ_LATCHED_MASK << (j * BITS_PER_IRQ));
			rt_stat = handlers[i].val
				& (IRQ_STATUS_MASK << (j * BITS_PER_IRQ));
			prev_rt_stat = handlers[i].prev_val
				& (IRQ_STATUS_MASK << (j * BITS_PER_IRQ));
			changed = prev_rt_stat ^ rt_stat;

			if (triggered || changed)
				rt_stat ? handlers[i].irq_info[j].high++ :
						handlers[i].irq_info[j].low++;

			if ((triggered || changed)
				&& handlers[i].irq_info[j].smb_irq != NULL) {
				handler_count++;
				rc = handlers[i].irq_info[j].smb_irq(chip,
								rt_stat);
				if (rc)
					pr_err("Couldn't handle %d irq for reg 0x%02x rc = %d\n",
						j, handlers[i].stat_reg, rc);
			}
		}
		handlers[i].prev_val = handlers[i].val;
	}
	if (chip->check_parallel) {
		smb1351_parallel_check_start(chip);
		chip->check_parallel = false;
	}
	pr_debug("handler count = %d\n", handler_count);
	if (handler_count) {
		pr_debug("batt psy changed\n");
		power_supply_changed(&chip->batt_psy);
	}

	mutex_unlock(&chip->irq_complete);

	return IRQ_HANDLED;
}

#define DEFAULT_SDP_MA		500
#define DEFAULT_CDP_MA		1500
//#define DEFAULT_DCP_MA		1800
//#define DEFAULT_HVDCP_MA	1800
#define DEFAULT_DCP_MA 2000
#define DEFAULT_HVDCP_MA 2000
#define DEFAULT_HVDCP3_MA	3000
static int smb1351_update_usb_supply_icl(struct smb1351_charger *chip)
{
	int rc, type, icl;
	union power_supply_propval pval = {0, };

	rc = chip->usb_psy->get_property(chip->usb_psy,
			POWER_SUPPLY_PROP_TYPE, &pval);
	if (rc) {
		pr_err("Get USB supply type failed, rc=%d\n", rc);
		return rc;
	}
	type = pval.intval;
	chip->usb_psy_type = type;
	rc = chip->usb_psy->get_property(chip->usb_psy,
			POWER_SUPPLY_PROP_CURRENT_MAX, &pval);
	pr_debug("pro type = %d, icl = %d\n", type, icl);
	if (!rc)
	icl = pval.intval /1000;
	else
		return rc;
	/*
	 * Only update ICL when DCP/HVDCP/HVDCP3 being detected
	 * For other types such as SDP/DCP, keep the ICL set in
	 * USB PHY to avoid violate USB spec.
	 */
	switch (type) {
	case POWER_SUPPLY_TYPE_USB_DCP:
		icl = DEFAULT_DCP_MA;
		break;
	case POWER_SUPPLY_TYPE_USB_HVDCP:
		icl = DEFAULT_HVDCP_MA;
		break;
	case POWER_SUPPLY_TYPE_USB_HVDCP_3:
		icl = DEFAULT_HVDCP3_MA;
		break;
	default:
		break;
	}
	chip->usb_psy_ma = icl;
	pr_info("type = %d, icl = %d\n", type, icl);

	return rc;
}

#define ESR_PULSE_DELTA_MA	200
static int smb1351_force_esr_pulse_en(struct smb1351_charger *chip, bool en)
{
	int rc = 0, current_in_esr = 0, fg_current_now = 0;
	int main_fcc_in_esr = 0, slave_fcc_in_esr = 0;
	int actual_slave_fcc_in_esr = 0;
	union power_supply_propval pval = {0,};
	struct power_supply *parallel_psy = smb1351_get_parallel_slave(chip);

	if (chip->in_esr_pulse == en) {
		pr_err("ESR pulse is already %s\n",
			en ? "enabled" : "disabled");
		return 0;
	}

	if (en) {
		rc = smb1351_get_bms_property(chip,
				POWER_SUPPLY_PROP_CURRENT_NOW,
				&fg_current_now);
		if (rc) {
			pr_debug("Get battery current from BMS(fg) failed, rc=%d\n",
								rc);
			return rc;
		}
		pr_debug("ibatt from FG reading: %duA\n", fg_current_now);
		fg_current_now = abs(fg_current_now);
		fg_current_now /= 1000;
		chip->main_fcc_ma_before_esr = chip->fastchg_current_max_ma;
		current_in_esr = max(chip->iterm_ma + ESR_PULSE_DELTA_MA,
				fg_current_now - ESR_PULSE_DELTA_MA);
		pr_debug("Force battery current to %d in ESR pulse\n",
							current_in_esr);
	}

	mutex_lock(&chip->parallel.lock);
	/*
	 * If parallel charging is not enabled, set the current_in_esr
	 * to main charger to achieve the ESR pulse
	 */
	if (!parallel_psy || !chip->parallel.slave_detected
			|| chip->parallel.slave_icl_ma == 0) {
		rc = smb1351_fastchg_current_set(chip,
				en ? current_in_esr :
				chip->main_fcc_ma_before_esr);
		if (!rc)
			chip->in_esr_pulse = en;
		else
			pr_err("set FCC for ESR failed, rc=%d\n", rc);

		goto unlock;
	}

	/*
	 * If parallel charging is enabled, split current_in_esr and
	 * set it to main and slave charger accordingly
	 */
		if (en) {
			rc = parallel_psy->get_property(parallel_psy,
				POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &pval);
			if (rc) {
				pr_err("get slave fcc failed, rc=%d\n", rc);
				goto unlock;
			}
			chip->slave_fcc_ma_before_esr = pval.intval / 1000;
			slave_fcc_in_esr = current_in_esr *
				(100 - chip->parallel.main_chg_fcc_percent) / 100;
			pval.intval = slave_fcc_in_esr * 1000;
			rc = parallel_psy->set_property(parallel_psy,
				POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &pval);
			if (rc) {
				pr_err("set slave fcc for ESR failed, rc=%d\n", rc);
				goto unlock;
			}
			rc = parallel_psy->get_property(parallel_psy,
				POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &pval);
			if (rc) {
				pr_err("get slave fcc for ESR failed, rc=%d\n", rc);
				goto unlock;
			}
			actual_slave_fcc_in_esr = pval.intval / 1000;
			pr_debug("slave_fcc_in_esr = %dmA, actual_slave_fcc_in_esr = %dmA\n",
				slave_fcc_in_esr, actual_slave_fcc_in_esr);
			main_fcc_in_esr = current_in_esr - actual_slave_fcc_in_esr;
			rc = smb1351_fastchg_current_set(chip, main_fcc_in_esr);
			if (rc) {
			pr_err("Set main fcc for ESR failed, rc=%d\n",
								rc);
				goto unlock;
			}
			pr_debug("main_fcc_in_esr = %dmA\n", main_fcc_in_esr);
		} else {
			pval.intval = chip->slave_fcc_ma_before_esr * 1000;
			rc = parallel_psy->set_property(parallel_psy,
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
				&pval);
			if (rc) {
				pr_err("restore fcc for slave failed, rc=%d\n", rc);
				goto unlock;
			}
		rc = smb1351_fastchg_current_set(chip,
				chip->main_fcc_ma_before_esr);
			if (rc) {
				pr_err("restore fcc for main failed, rc=%d\n", rc);
				goto unlock;
			}
			pr_debug("restore, main_fcc = %dmA, slave_fcc = %dmA\n",
					chip->main_fcc_ma_before_esr,
					chip->slave_fcc_ma_before_esr);
		}


	chip->in_esr_pulse = en;
unlock:
	mutex_unlock(&chip->parallel.lock);

	return rc;
}

#define ESR_PULSE_TIME_MS	1500
static void smb1351_force_esr_for_fg(struct smb1351_charger *chip)
{
	int rc = 0, esr_count = 0;

	if (!chip->chg_present) {
		pr_debug("No force ESR pulse when no charging\n");
		return;
	}

	pr_debug("start!\n");
	chip->is_in_esr_state = true;

	rc = smb1351_set_bms_property(chip, POWER_SUPPLY_PROP_UPDATE_NOW, 1);
	if (rc) {
		pr_debug("Update BMS(fg) status failed, rc=%d\n", rc);
		chip->is_in_esr_state = false;
		return;
	}
	rc = smb1351_get_bms_property(chip, POWER_SUPPLY_PROP_ESR_COUNT,
					&esr_count);
	if (rc) {
		pr_debug("Get ESR count from BMS(fg) failed, rc=%d\n", rc);
		chip->is_in_esr_state = false;
		return;
	}
	if (esr_count != 0) {
		pr_debug("ESR count is not zero: %d, skipping\n",
					esr_count);
		chip->is_in_esr_state = false;
		return;
	}
	smb1351_stay_awake(&chip->smb1351_ws, FORCE_ESR_PULSE);
	rc = smb1351_force_esr_pulse_en(chip, true);
	if (rc) {
		pr_err("Force ESR pulse enable failed, rc=%d\n", rc);
		chip->is_in_esr_state = false;
		goto end;
	}
	msleep(ESR_PULSE_TIME_MS);
	rc = smb1351_force_esr_pulse_en(chip, false);
	if (rc) {
		pr_err("Force ESR pulse disable failed, rc=%d\n", rc);
		chip->is_in_esr_state = false;
		goto end;
	}

end:
	smb1351_relax(&chip->smb1351_ws, FORCE_ESR_PULSE);

	chip->is_in_esr_state = false;
	pr_debug("end!\n");
}

static void battery_soc_changed(struct smb1351_charger *chip)
{
	int rc, soc;

	soc = smb1351_get_prop_batt_capacity(chip);
	if (chip->pre_soc == soc)
		return;

	if (chip->wa_flags & CHG_FORCE_ESR_WA)
		smb1351_force_esr_for_fg(chip);

	if ((chip->wa_flags & CHG_SOC_BASED_RESUME)
		&& chip->batt_full && (soc <= chip->resume_soc)) {
		rc = smb1351_battchg_disable(chip, SOC, false);
		if (rc) {
			pr_err("Enable charge for SOC failed, rc=%d\n",
						rc);
			return;
		}
		chip->batt_full = false;
		smb1351_set_bms_property(chip, POWER_SUPPLY_PROP_STATUS,
				smb1351_get_prop_batt_status(chip));
	}

	chip->pre_soc = soc;
}

static void smb1351_external_power_changed(struct power_supply *psy)
{
	struct smb1351_charger *chip = container_of(psy,
				struct smb1351_charger, batt_psy);
	union power_supply_propval prop = {0,};
	int rc, online = 0;
	struct power_supply *parallel_psy = smb1351_get_parallel_slave(chip);
	int slave_present;


	battery_soc_changed(chip);


	parallel_psy->get_property(parallel_psy, POWER_SUPPLY_PROP_PRESENT, &prop);
	slave_present = prop.intval;
	/* Don't update ICL if slave is enabled */
	pr_debug("slave_detected = %d, slave_present = %d, slave_icl_ma = %d\n",
			chip->parallel.slave_detected, slave_present, chip->parallel.slave_icl_ma);
	if (chip->parallel.slave_detected && slave_present
			&& chip->parallel.slave_icl_ma != 0)
		return;

	rc = chip->usb_psy->get_property(chip->usb_psy,
				POWER_SUPPLY_PROP_ONLINE, &prop);
	if (rc) {
		pr_err("Couldn't read USB online property, rc=%d\n", rc);
		return;
	}

	online = prop.intval;
	rc = smb1351_update_usb_supply_icl(chip);
	if (rc) {
		pr_err("Update USB supply ICL failed!\n");
		return;
	}

	pr_debug("online = %d, current_limit = %d\n", online, chip->usb_psy_ma);

	smb1351_enable_volatile_writes(chip);
	if (chip->is_factory_mode)
		pr_info("in factory mode. do not set usbin\n");
	else
		smb1351_set_usb_chg_current(chip, chip->usb_psy_ma);

	pr_debug("updating batt psy\n");
}

#define LAST_CNFG_REG	0x16
static int show_cnfg_regs(struct seq_file *m, void *data)
{
	struct smb1351_charger *chip = m->private;
	int rc;
	u8 reg;
	u8 addr;

	for (addr = 0; addr <= LAST_CNFG_REG; addr++) {
		rc = smb1351_read_reg(chip, addr, &reg);
		if (!rc)
			seq_printf(m, "0x%02x = 0x%02x\n", addr, reg);
	}

	return 0;
}

static int cnfg_debugfs_open(struct inode *inode, struct file *file)
{
	struct smb1351_charger *chip = inode->i_private;

	return single_open(file, show_cnfg_regs, chip);
}

static const struct file_operations cnfg_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= cnfg_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#define FIRST_CMD_REG	0x30
#define LAST_CMD_REG	0x34
static int show_cmd_regs(struct seq_file *m, void *data)
{
	struct smb1351_charger *chip = m->private;
	int rc;
	u8 reg;
	u8 addr;

	for (addr = FIRST_CMD_REG; addr <= LAST_CMD_REG; addr++) {
		rc = smb1351_read_reg(chip, addr, &reg);
		if (!rc)
			seq_printf(m, "0x%02x = 0x%02x\n", addr, reg);
	}

	return 0;
}

static int cmd_debugfs_open(struct inode *inode, struct file *file)
{
	struct smb1351_charger *chip = inode->i_private;

	return single_open(file, show_cmd_regs, chip);
}

static const struct file_operations cmd_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= cmd_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#define FIRST_STATUS_REG	0x36
#define LAST_STATUS_REG		0x3F
static int show_status_regs(struct seq_file *m, void *data)
{
	struct smb1351_charger *chip = m->private;
	int rc;
	u8 reg;
	u8 addr;

	for (addr = FIRST_STATUS_REG; addr <= LAST_STATUS_REG; addr++) {
		rc = smb1351_read_reg(chip, addr, &reg);
		if (!rc)
			seq_printf(m, "0x%02x = 0x%02x\n", addr, reg);
	}

	return 0;
}

static int status_debugfs_open(struct inode *inode, struct file *file)
{
	struct smb1351_charger *chip = inode->i_private;

	return single_open(file, show_status_regs, chip);
}

static const struct file_operations status_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= status_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int show_irq_count(struct seq_file *m, void *data)
{
	int i, j, total = 0;

	for (i = 0; i < ARRAY_SIZE(handlers); i++)
		for (j = 0; j < 4; j++) {
			seq_printf(m, "%s=%d\t(high=%d low=%d)\n",
						handlers[i].irq_info[j].name,
						handlers[i].irq_info[j].high
						+ handlers[i].irq_info[j].low,
						handlers[i].irq_info[j].high,
						handlers[i].irq_info[j].low);
			total += (handlers[i].irq_info[j].high
					+ handlers[i].irq_info[j].low);
		}

	seq_printf(m, "\n\tTotal = %d\n", total);

	return 0;
}

static int irq_count_debugfs_open(struct inode *inode, struct file *file)
{
	struct smb1351_charger *chip = inode->i_private;

	return single_open(file, show_irq_count, chip);
}

static const struct file_operations irq_count_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= irq_count_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int get_reg(void *data, u64 *val)
{
	struct smb1351_charger *chip = data;
	int rc;
	u8 temp;

	rc = smb1351_read_reg(chip, chip->peek_poke_address, &temp);
	if (rc) {
		pr_err("Couldn't read reg %x rc = %d\n",
			chip->peek_poke_address, rc);
		return -EAGAIN;
	}
	*val = temp;
	return 0;
}

static int set_reg(void *data, u64 val)
{
	struct smb1351_charger *chip = data;
	int rc;
	u8 temp;

	temp = (u8) val;
	rc = smb1351_write_reg(chip, chip->peek_poke_address, temp);
	if (rc) {
		pr_err("Couldn't write 0x%02x to 0x%02x rc= %d\n",
			temp, chip->peek_poke_address, rc);
		return -EAGAIN;
	}
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(poke_poke_debug_ops, get_reg, set_reg, "0x%02llx\n");

static int force_irq_set(void *data, u64 val)
{
	struct smb1351_charger *chip = data;

	smb1351_chg_stat_handler(chip->client->irq, data);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(force_irq_ops, NULL, force_irq_set, "0x%02llx\n");

#ifdef DEBUG
static void dump_regs(struct smb1351_charger *chip)
{
	int rc;
	u8 reg;
	u8 addr;

	pr_debug("%s %s\n", dev_name(chip->dev), __func__);

	for (addr = 0; addr <= LAST_CNFG_REG; addr++) {
		rc = smb1351_read_reg(chip, addr, &reg);
		if (rc)
			pr_err("Couldn't read 0x%02x rc = %d\n", addr, rc);
		else
			pr_debug("0x%02x = 0x%02x\n", addr, reg);
	}

	for (addr = FIRST_STATUS_REG; addr <= LAST_STATUS_REG; addr++) {
		rc = smb1351_read_reg(chip, addr, &reg);
		if (rc)
			pr_err("Couldn't read 0x%02x rc = %d\n", addr, rc);
		else
			pr_debug("0x%02x = 0x%02x\n", addr, reg);
	}

	for (addr = FIRST_CMD_REG; addr <= LAST_CMD_REG; addr++) {
		rc = smb1351_read_reg(chip, addr, &reg);
		if (rc)
			pr_err("Couldn't read 0x%02x rc = %d\n", addr, rc);
		else
			pr_debug("0x%02x = 0x%02x\n", addr, reg);
	}
}
#else
static void dump_regs(struct smb1351_charger *chip)
{
}
#endif

#define MAIN_CHG_DEFAULT_FCC_PERCENT	50
#define MAIN_CHG_DEFAULT_ICL_PERCENT	50
static int smb1351_parse_parallel_charger_dt(struct smb1351_charger *chip)
{
	int rc;
	struct device_node *node = chip->dev->of_node;

	rc = of_property_read_u32(node, "qcom,parallel-usb-min-current-ma",
					&chip->parallel.min_current_thr_ma);
	if (rc && rc != -EINVAL)
		goto bail_out;

	rc = of_property_read_u32(node, "qcom,parallel-usb-9v-min-current-ma",
					&chip->parallel.min_9v_current_thr_ma);
	if (rc && rc != -EINVAL)
		goto bail_out;

	rc = of_property_read_u32(node, "qcom,parallel-usb-12v-min-current-ma",
				&chip->parallel.min_12v_current_thr_ma);
	if (rc && rc != -EINVAL)
		goto bail_out;

	rc = of_property_read_u32(node, "qcom,parallel-allowed-lowering-ma",
				&chip->parallel.allowed_lowering_ma);
	if (rc && rc != -EINVAL)
		goto bail_out;

	rc = of_property_read_u32(node, "qcom,parallel-main-chg-fcc-percent",
					&chip->parallel.main_chg_fcc_percent);
	if (rc < 0)
		chip->parallel.main_chg_fcc_percent =
			MAIN_CHG_DEFAULT_FCC_PERCENT;

	rc = of_property_read_u32(node, "qcom,parallel-main-chg-icl-percent",
					&chip->parallel.main_chg_icl_percent);
	if (rc < 0)
		chip->parallel.main_chg_icl_percent =
			MAIN_CHG_DEFAULT_ICL_PERCENT;

	chip->parallel.avail = true;
	pr_debug("parallel main charger settings: min_curr = %d, min_9v_curr = %d, min_12v_curr = %d, allowed_lowering = %d, fcc_percent = %d, icl_percent = %d\n",
				chip->parallel.min_current_thr_ma,
				chip->parallel.min_9v_current_thr_ma,
				chip->parallel.min_12v_current_thr_ma,
				chip->parallel.allowed_lowering_ma,
				chip->parallel.main_chg_fcc_percent,
				chip->parallel.main_chg_icl_percent);

	return 0;

bail_out:
	chip->parallel.min_current_thr_ma = -EINVAL;
	chip->parallel.min_9v_current_thr_ma = -EINVAL;
	chip->parallel.min_12v_current_thr_ma = -EINVAL;
	chip->parallel.allowed_lowering_ma = -EINVAL;
	pr_debug("parallel charger not support!\n");

	return 0;
}

static int smb1351_parse_dt(struct smb1351_charger *chip)
{
	int rc;
	struct device_node *node = chip->dev->of_node;

	if (!node) {
		pr_err("device tree info. missing\n");
		return -EINVAL;
	}

	chip->chg_en_gpio = of_get_named_gpio_flags(node, "qcom,en-gpio", 0, 0);
	chip->chg_led_gpio = of_get_named_gpio_flags(node, "qcom,led-gpio", 0, 0);
#ifdef LENOVO_OTG_USB_SHORT
	chip->otg_usb_short_gpio = of_get_named_gpio_flags(node, "qcom,otg-short-gpio", 0, 0);
#endif

	chip->usb_suspended_status = of_property_read_bool(node,
					"qcom,charging-disabled");

	chip->chg_autonomous_mode = of_property_read_bool(node,
					"qcom,chg-autonomous-mode");

	chip->disable_apsd = of_property_read_bool(node, "qcom,disable-apsd");

	chip->using_pmic_therm = of_property_read_bool(node,
						"qcom,using-pmic-therm");
	chip->bms_controlled_charging  = of_property_read_bool(node,
					"qcom,bms-controlled-charging");
	chip->force_hvdcp_2p0 = of_property_read_bool(node,
					"qcom,force-hvdcp-2p0");

	rc = of_property_read_string(node, "qcom,bms-psy-name",
						&chip->bms_psy_name);
	if (rc)
		chip->bms_psy_name = NULL;
	chip->resume_soc = -EINVAL;
	if (of_property_read_bool(node, "qcom,use-external-fg")) {
		chip->use_external_fg = true;
		chip->wa_flags |= CHG_FORCE_ESR_WA | CHG_FG_NOTIFY_JEITA
				| CHG_ITERM_CHECK_SOC;
		rc = of_property_read_u32(node, "qcom,resume-soc",
					&chip->resume_soc);
		if (!rc && chip->resume_soc > 0)
			chip->wa_flags |= CHG_SOC_BASED_RESUME;
	}

	rc = of_property_read_u32(node, "qcom,fastchg-current-max-ma",
					&chip->target_fastchg_current_max_ma);
	if (rc)
		chip->target_fastchg_current_max_ma = SMB1351_CHG_FAST_MAX_MA;

	chip->iterm_disabled = of_property_read_bool(node,
					"qcom,iterm-disabled");

	rc = of_property_read_u32(node, "qcom,iterm-ma", &chip->iterm_ma);
	if (rc)
		chip->iterm_ma = -EINVAL;

	rc = of_property_read_u32(node, "qcom,float-voltage-mv",
						&chip->vfloat_mv);
	if (rc)
		chip->vfloat_mv = -EINVAL;

	rc = of_property_read_u32(node, "qcom,recharge-mv",
						&chip->recharge_mv);
	if (rc)
		chip->recharge_mv = -EINVAL;

	chip->recharge_disabled = of_property_read_bool(node,
					"qcom,recharge-disabled");

	rc = of_property_read_u32(node, "qcom,switch-freq",
						&chip->switch_freq);
	if (rc < 0)
		chip->switch_freq = -EINVAL;
	/* thermal and jeita support */
	rc = of_property_read_u32(node, "qcom,batt-cold-decidegc",
						&chip->batt_cold_decidegc);
	if (rc < 0)
		chip->batt_cold_decidegc = -EINVAL;

	rc = of_property_read_u32(node, "qcom,batt-hot-decidegc",
						&chip->batt_hot_decidegc);
	if (rc < 0)
		chip->batt_hot_decidegc = -EINVAL;

	rc = of_property_read_u32(node, "qcom,batt-warm-decidegc",
						&chip->batt_warm_decidegc);
	if (rc < 0)
		chip->batt_warm_decidegc = -EINVAL;

	rc = of_property_read_u32(node, "qcom,batt-cool-decidegc",
						&chip->batt_cool_decidegc);
	if (rc < 0)
		chip->batt_cool_decidegc = -EINVAL;

	if ((chip->batt_warm_decidegc != -EINVAL
			&& chip->batt_cool_decidegc != -EINVAL)
			|| (chip->wa_flags & CHG_FG_NOTIFY_JEITA)) {
		rc = of_property_read_u32(node, "qcom,batt-cool-mv",
						&chip->batt_cool_mv);

		rc |= of_property_read_u32(node, "qcom,batt-warm-mv",
						&chip->batt_warm_mv);

		rc |= of_property_read_u32(node, "qcom,batt-cool-ma",
						&chip->batt_cool_ma);

		rc |= of_property_read_u32(node, "qcom,batt-warm-ma",
						&chip->batt_warm_ma);
		if (rc)
			chip->jeita_supported = false;
		else if (chip->wa_flags & CHG_FG_NOTIFY_JEITA)
			chip->fg_notify_jeita = true;
		else
			chip->jeita_supported = true;
	}

	pr_debug("wa_flag = %d, fg_notify_jeita = %d, jeita_supported = %d\n",
			chip->wa_flags, chip->fg_notify_jeita, chip->jeita_supported);

	rc = of_property_read_u32(node, "qcom,batt-missing-decidegc",
						&chip->batt_missing_decidegc);

	chip->pinctrl_state_name = of_get_property(node, "pinctrl-names", NULL);

	smb1351_parse_parallel_charger_dt(chip);

	return 0;
}

#define RERUN_APSD_DELAY_MS	1000
static int smb1351_determine_initial_state(struct smb1351_charger *chip)
{
	int rc;
	u8 reg = 0;

	/*
	 * It is okay to read the interrupt status here since
	 * interrupts aren't requested. Reading interrupt status
	 * clears the interrupt so be careful to read interrupt
	 * status only in interrupt handling code
	 */

	rc = smb1351_read_reg(chip, IRQ_B_REG, &reg);
	if (rc) {
		pr_err("Couldn't read IRQ_B rc = %d\n", rc);
		goto fail_init_status;
	}

	chip->battery_missing = (reg & IRQ_BATT_MISSING_BIT) ? true : false;

	rc = smb1351_read_reg(chip, IRQ_C_REG, &reg);
	if (rc) {
		pr_err("Couldn't read IRQ_C rc = %d\n", rc);
		goto fail_init_status;
	}
	chip->batt_full = (reg & IRQ_TERM_BIT) ? true : false;

	rc = smb1351_read_reg(chip, IRQ_A_REG, &reg);
	if (rc) {
		pr_err("Couldn't read irq A rc = %d\n", rc);
		return rc;
	}

	if (reg & IRQ_HOT_HARD_BIT)
		chip->batt_hot = true;
	if (reg & IRQ_COLD_HARD_BIT)
		chip->batt_cold = true;
	if (reg & IRQ_HOT_SOFT_BIT)
		chip->batt_warm = true;
	if (reg & IRQ_COLD_SOFT_BIT)
		chip->batt_cool = true;

	rc = smb1351_read_reg(chip, IRQ_E_REG, &reg);
	if (rc) {
		pr_err("Couldn't read IRQ_E rc = %d\n", rc);
		goto fail_init_status;
	}

	if (reg & IRQ_USBIN_UV_BIT) {
		smb1351_usbin_uv_handler(chip, 1);
	} else {
		if (!chip->is_factory_mode) {
			smb1351_stay_awake(&chip->smb1351_ws, RERUN_APSD);
			schedule_delayed_work(&chip->rerun_apsd_work,
				 msecs_to_jiffies(RERUN_APSD_DELAY_MS));
		}
	}

	rc = smb1351_read_reg(chip, IRQ_G_REG, &reg);
	if (rc) {
		pr_err("Couldn't read IRQ_G rc = %d\n", rc);
		goto fail_init_status;
	}

	if (reg & IRQ_SOURCE_DET_BIT)
		smb1351_apsd_complete_handler(chip, 1);

	return 0;

fail_init_status:
	pr_err("Couldn't determine initial status\n");
	return rc;
}

static int is_parallel_slave(struct i2c_client *client)
{
	struct device_node *node = client->dev.of_node;

	return of_property_read_bool(node, "qcom,parallel-charger");
}

static int create_debugfs_entries(struct smb1351_charger *chip)
{
	struct dentry *ent;
	char str[16];

	snprintf(str, ARRAY_SIZE(str), "smb1351_%s",
			chip->is_slave ? "slave" : "main");
	chip->debug_root = debugfs_create_dir(str, NULL);
	if (!chip->debug_root) {
		pr_err("Couldn't create debug dir\n");
	} else {
		ent = debugfs_create_file("config_registers", S_IFREG | S_IRUGO,
					chip->debug_root, chip,
					&cnfg_debugfs_ops);
		if (!ent)
			pr_err("Couldn't create cnfg debug file\n");

		ent = debugfs_create_file("status_registers", S_IFREG | S_IRUGO,
					chip->debug_root, chip,
					&status_debugfs_ops);
		if (!ent)
			pr_err("Couldn't create status debug file\n");

		ent = debugfs_create_file("cmd_registers", S_IFREG | S_IRUGO,
					chip->debug_root, chip,
					&cmd_debugfs_ops);
		if (!ent)
			pr_err("Couldn't create cmd debug file\n");

		ent = debugfs_create_x32("address", S_IFREG | S_IWUSR | S_IRUGO,
					chip->debug_root,
					&(chip->peek_poke_address));
		if (!ent)
			pr_err("Couldn't create address debug file\n");

		ent = debugfs_create_file("data", S_IFREG | S_IWUSR | S_IRUGO,
					chip->debug_root, chip,
					&poke_poke_debug_ops);
		if (!ent)
			pr_err("Couldn't create data debug file\n");
		if (!chip->is_slave) {
			ent = debugfs_create_file("force_irq",
					S_IFREG | S_IWUSR | S_IRUGO,
					chip->debug_root, chip,
					&force_irq_ops);
			if (!ent)
				pr_err("Couldn't create data debug file\n");

			ent = debugfs_create_file("irq_count",
					S_IFREG | S_IRUGO,
					chip->debug_root, chip,
					&irq_count_debugfs_ops);
			if (!ent)
				pr_err("Couldn't create count debug file\n");
		}
	}
	return 0;
}

static int smb1351_regulator_get(struct smb1351_charger *chip)
{
	int ret;

	pr_info("%s\n", __func__);

	chip->vdd = regulator_get(&chip->client->dev, "vdd");
	if (IS_ERR_OR_NULL(chip->vdd)) {
		pr_err("%s: fail to get 1.8v LDO\n", __func__);
		return -3;
	}

	if (regulator_count_voltages(chip->vdd) > 0) {
		ret = regulator_set_voltage(chip->vdd, 1800000, 1800000);
		if (ret) {
			pr_err("%s: regulator set_vtg vdd_reg failed rc=%d\n", __func__, ret);
			regulator_put(chip->vdd);
			return -4;
		}
	}

	ret = regulator_enable(chip->vdd);
	if (ret) {
		pr_err("%s: Regulator vdd_reg enable failed rc=%d\n", __func__, ret);
	}

    return 0;
}

static bool smb1351_charger_is_factory_mode(void)
{
	struct device_node *np = of_find_node_by_path("/chosen");
	bool factory = false;

	if (np)
		factory = of_property_read_bool(np, "mmi,factory-cable");

	of_node_put(np);

	return factory;
}

static bool smb1351_is_usb_present(struct smb1351_charger *chip)
{
	int rc;
	u8 reg;

	rc = smb1351_read_reg(chip, STATUS_1_REG, &reg);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read rt status rc = %d\n", rc);
		return false;
	}
	reg &= 0xf0;

	return reg?true:false;
}

static int smb1351_reboot(struct notifier_block *nb,
			 unsigned long event, void *unused)
{
	struct smb1351_charger *chip =
			container_of(nb, struct smb1351_charger, mmi_reboot);

	dev_dbg(chip->dev, "SMB Reboot\n");
	if (!chip) {
		dev_warn(chip->dev, "called before chip valid!\n");
		return NOTIFY_DONE;
	}

	if (chip->is_factory_mode) {
		dev_info(chip->dev, "SMB Reboot\n");

		switch (event) {
		case SYS_POWER_OFF:
			/* Disable Charging */
			smb1351_battchg_disable(chip, USER, true);

			/* Suspend USB and DC */
			smb1351_usb_suspend(chip, USER, true);

			while (smb1351_is_usb_present(chip))
				msleep(100);
			dev_warn(chip->dev, "VBUS UV wait 1 sec!\n");
			/* Delay 1 sec to allow more VBUS decay */
			msleep(1000);
			break;
		default:
			break;
		}
	}
	return NOTIFY_DONE;
}

static int smb1351_jeita_check_lenovo(struct smb1351_charger *chip, int temp)
{
	int health;

	pr_info("temp %d cool %d warm %d\n", temp, chip->batt_cool_decidegc, chip->batt_warm_decidegc);

	if (temp <= chip->batt_cold_decidegc)
		health = POWER_SUPPLY_HEALTH_COLD;
	else if (temp < chip->batt_cool_decidegc)
		health = POWER_SUPPLY_HEALTH_COOL;
	else if (temp < chip->batt_warm_decidegc)
		health = POWER_SUPPLY_HEALTH_GOOD;
	else if (temp < chip->batt_hot_decidegc)
		health = POWER_SUPPLY_HEALTH_WARM;
	else
		health = POWER_SUPPLY_HEALTH_OVERHEAT;

	return health;
}

#ifdef CHG_HEATBEAT_WORK
static void smb1351_heatbeat_work(struct work_struct *work)
{
	struct smb1351_charger *chip = container_of(work, struct smb1351_charger, heatbeat_work.work);
	unsigned char state;
	int vbus_uv =  0, temp_bat = 0, temp_b = 0, temp_c = 0, soc = 0, vol = 0, cur = 0;
	static int pre_health = -1;
	int health;

	smb1351_read_reg(chip, STATUS_4_REG, &state);
	if (!chip->is_slave) {
		vbus_uv = smb1351_get_vbus_voltage(chip);
		temp_b = smb1351_get_board_temp(chip, BOARD_TEMP);
		temp_c = smb1351_get_board_temp(chip, CHARGE_TEMP);
		if (vbus_uv > 5600000)
			chip->is_hv_fast_charging = true;
		else
			chip->is_hv_fast_charging = false;
		temp_bat = smb1351_get_prop_batt_temp(chip);
		smb1351_get_bms_property(chip, POWER_SUPPLY_PROP_CAPACITY, &soc);
		smb1351_get_bms_property(chip, POWER_SUPPLY_PROP_VOLTAGE_NOW, &vol);
		smb1351_get_bms_property(chip, POWER_SUPPLY_PROP_CURRENT_NOW, &cur);
		pr_info(" %s, chg state 0x%x, soc %d, vbat %d, cur %d, vbus %d, battemp %d, btemp %d, ctemp %d\n",
				dev_name(chip->dev), state, soc, vol, cur, vbus_uv, temp_bat, temp_b, temp_c);

		health = smb1351_jeita_check_lenovo(chip, temp_bat);
		if (health != pre_health) {
			pr_info("jeita change. bat heal %d pre %d\n", health, pre_health);
			smb1351_set_ext_charger_property(chip, POWER_SUPPLY_PROP_HEALTH, health);
			pre_health = health;
		}

		if((vbus_uv < 2000000) && (vbus_uv > 0) && (chip->chg_present == true)) {
			pr_info("%s, chg remove but not do remove work\n",dev_name(chip->dev));
			smb1351_usbin_uv_handler(chip, 1);
		}
	} else
		pr_info(" %s, chg state 0x%x\n", dev_name(chip->dev), state);

	schedule_delayed_work(&chip->heatbeat_work, (HZ * 10));
}
#endif

#ifdef EXT_CHARGER_POWER_SUPPLY
static char *ext_charger_chip_name = "ext-charger";
static struct smb1351_charger *smb1351_charger_chip = NULL;

static enum power_supply_property ext_charger_power_supply_props[] = {
	POWER_SUPPLY_PROP_CHARGE_TYPE,
//	POWER_SUPPLY_PROP_ONLINE,
//	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_TEMP,
//	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_CHARGE_ENABLED,
	POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
//	POWER_SUPPLY_PROP_FLASH_ACTIVE,
//	POWER_SUPPLY_PROP_USB_OTG,
	POWER_SUPPLY_PROP_HEALTH,
};

static int ext_charger_power_supply_get_property(struct power_supply *psy,
					     enum power_supply_property psp,
					     union power_supply_propval *val)
{
	struct smb1351_charger *chip = container_of(psy, struct smb1351_charger, ext_charger_psy);
	int value;

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = 25;//smb1351_get_board_temp(chip, PA1_TEMP);//board ntc temp
		break;
	case POWER_SUPPLY_PROP_CHARGE_ENABLED:
		val->intval = !chip->usb_suspended_status;
		break;
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
		val->intval = !chip->battchg_disabled_status;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
		smb1351_get_usb_chg_current(chip, &value);
		val->intval = value;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = chip->fastchg_current_max_ma;
		break;
	default:
		pr_err("%s unsupported psy %d\n", __func__, psp);
		return -EINVAL;
	}

	return 0;
}

static int ext_charger_power_supply_set_property(struct power_supply *psy,
					     enum power_supply_property psp,
					     const union power_supply_propval *val)
{
	struct smb1351_charger *chip = container_of(psy, struct smb1351_charger, ext_charger_psy);
	int value;

	switch (psp) {
		//just for charging temp protect function debug
		case POWER_SUPPLY_PROP_TEMP:
			pr_err("%s debug charging temp, intval = %d\n", __func__, val->intval);
			if ((val->intval<-200) ||(val->intval>600)) {
				chip->temp_debug_flag = false;
			} else {
				chip->temp_debug_flag = true;
				value = val->intval;
				chip->batt_temp = val->intval;
			}
			break;
		case POWER_SUPPLY_PROP_CHARGE_ENABLED:
			chip->is_factory_need_write = true;
			smb1351_usb_suspend(chip, USER, !val->intval);
			chip->is_factory_need_write = false;
			break;
		case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
			chip->is_factory_need_write = true;
			smb1351_battchg_disable(chip, USER, !val->intval);
			chip->is_factory_need_write = false;
			break;
		case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
			value = val->intval;
			chip->is_factory_need_write = true;
			if (!value)
				smb1351_usb_suspend(chip, USER, 1);
			else {
				if (chip->usb_suspended_status)
					smb1351_usb_suspend(chip, USER, 0);

				smb1351_set_usb_chg_current(chip, value);
			}
			chip->is_factory_need_write = false;
			break;
		case POWER_SUPPLY_PROP_CURRENT_MAX:
			value = val->intval;
			chip->is_factory_need_write = true;
			if (!value)
				smb1351_battchg_disable(chip, USER, 1);
			else {
				if (!chip->battchg_disabled_status)
					smb1351_battchg_disable(chip, USER, 0);

				smb1351_fastchg_hv_current_set(chip, value);
			}
			chip->is_factory_need_write = false;
			break;
		case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
			value = val->intval;
			chip->is_factory_need_write = true;
			if (!value)
				smb1351_battchg_disable(chip, USER, 1);
			else {
				if (chip->battchg_disabled_status)
					smb1351_battchg_disable(chip, USER, 0);

				smb1351_fastchg_hv_current_set(chip, value);
			}
			chip->is_factory_need_write = false;
			break;
		case POWER_SUPPLY_PROP_HEALTH:
			if (!chip->is_factory_mode) {
				smb1351_handle_jeita_lenovo(chip, val->intval);
				power_supply_changed(&chip->batt_psy);
			}
			break;
		default:
			pr_err("%s not support power_supply property cmd\n", __func__);
			return -EINVAL;
	}
	return 0;
}

static int ext_charger_power_supply_property_is_writeable(struct power_supply *psy,
					enum power_supply_property psp)
{
	switch (psp) {
		case POWER_SUPPLY_PROP_CHARGE_ENABLED:
		case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
		case POWER_SUPPLY_PROP_CURRENT_MAX:
		case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
		case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		case POWER_SUPPLY_PROP_HEALTH:
			return 1;
			break;
		default:
			break;
	}

	return 0;
}

static int ext_charger_power_supply_init(struct smb1351_charger *chip)
{
	int ret;

	chip->ext_charger_psy.name = ext_charger_chip_name;
	chip->ext_charger_psy.type = POWER_SUPPLY_TYPE_BMS;
	chip->ext_charger_psy.properties = ext_charger_power_supply_props;
	chip->ext_charger_psy.num_properties = ARRAY_SIZE(ext_charger_power_supply_props);
	chip->ext_charger_psy.get_property = ext_charger_power_supply_get_property;
	chip->ext_charger_psy.set_property = ext_charger_power_supply_set_property;
	chip->ext_charger_psy.property_is_writeable = ext_charger_power_supply_property_is_writeable;

	ret = power_supply_register(&chip->client->dev, &chip->ext_charger_psy);
	if (ret)
	{
		pr_err("psy register err %d\n", ret);
		return ret;
	}

	return 0;
}


static void ext_charger_power_supply_exit(struct smb1351_charger *chip)
{
	//cancel_delayed_work_sync(&chip->work);
	power_supply_unregister(&chip->ext_charger_psy);

}

static ssize_t led_opt_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	int cmd;
	int value = -1;
	struct smb1351_charger *chip = smb1351_charger_chip;

	if (!chip) {
		pr_err("no valid chip value, return\n");
		return ret;
	}

	sscanf(buf, "%x", &cmd);
	value = smb1351_led_operation(chip, cmd?1:0);

	return ret;
}


static ssize_t  led_opt_get(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct smb1351_charger *chip = smb1351_charger_chip;
	int val;
	int ret = 0;

	if (!chip) {
		pr_err("no valid chip value, return\n");
		return ret;
	}

	val = smb1351_led_state(chip);

	pr_info("led state  is %d\n", val);
	ret = sprintf(buf, "%d\n", val);

	return ret;
}

static DEVICE_ATTR(led_opt, S_IRUGO|S_IWUSR, led_opt_get, led_opt_set);

static ssize_t fast_charger_need_change_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	int cmd;
	s32 rc;
	struct smb1351_charger *chip = smb1351_charger_chip;

	if (!chip) {
		pr_err("no valid chip value, return\n");
		return ret;
	}

	while (chip->is_in_esr_state) {
		msleep(500);
		pr_info("in esr state. wait to handshake prepare\n");
	}

	sscanf(buf, "%x", &cmd);
	chip->handshake_state = cmd;
	if (chip->handshake_state) {
		pr_info("handshake prepare\n");

		smb1351_parallel_charger_disable_slave(chip);
		rc = smb1351_masked_write(chip, CMD_INPUT_LIMIT_REG, 1, 0);
		if (rc) {
			pr_err("Set 0x31 0 failed, rc=%d\n", rc);
			return rc;
		}

		chip->handshake_state = true;
	} else {
		pr_info("handshake finish\n");

		rc = smb1351_masked_write(chip, CMD_INPUT_LIMIT_REG, BIT(0), CMD_USB_HC_MODE);
		if (rc) {
			pr_err("Set 0x31 0 failed, rc=%d\n", rc);
			return rc;
		}

		chip->handshake_state = false;

		rc = smb1351_get_vbus_voltage(chip);
		if (rc > 6000000)
			chip->is_hv_fast_charging = true;
		else
			chip->is_hv_fast_charging = false;
	}

	return ret;
}


static ssize_t  fast_charger_need_change_get(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct smb1351_charger *chip = smb1351_charger_chip;
	int val;
	int ret = 0;

	if (!chip) {
		pr_err("no valid chip value, return\n");
		return ret;
	}

	val = chip->handshake_state;

	pr_info("handshake_state  is %d\n", val);
	ret = sprintf(buf, "%d\n", val);

	return ret;
}

static DEVICE_ATTR(fast_charger_need_change, S_IRUGO|S_IWUSR, fast_charger_need_change_get, fast_charger_need_change_set);

static ssize_t fast_charger_is_engineermode_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	int cmd;
	struct smb1351_charger *chip = smb1351_charger_chip;

	sscanf(buf, "%x", &cmd);
	chip->fast_charger_is_engineermode = cmd;

	pr_info("fast charge engineer ret %d\n", chip->fast_charger_is_engineermode);

	return ret;
}

ssize_t  fast_charger_is_engineermode_get(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	int ret;
	struct smb1351_charger *chip = smb1351_charger_chip;

	ret = sprintf(buf, "%d\n", chip->fast_charger_is_engineermode);

	return ret;
}

static DEVICE_ATTR(fast_charger_is_engineermode, S_IRUGO|S_IWUSR, fast_charger_is_engineermode_get,fast_charger_is_engineermode_set);

#ifdef LENOVO_OTG_USB_SHORT
static int smb1351_turn_otg_vbus(struct smb1351_charger *chip, bool votg_on)
{
	int rc = 0;

	if (votg_on) {
		rc = smb1351_masked_write(chip, OTG_TLIM_CTRL_REG, OTG_OC_LIMIT_MASK,
								0x00);
		if (rc)
			pr_err("Couldn't set OTG_OC_LIMIT rc=%d\n", rc);

		rc = smb1351_masked_write(chip, CMD_CHG_REG, CMD_OTG_EN_BIT,
								CMD_OTG_EN_BIT);
		if (rc)
			pr_err("Couldn't enable  OTG mode rc=%d\n", rc);
	} else {
		rc = smb1351_masked_write(chip, CMD_CHG_REG, CMD_OTG_EN_BIT, 0);
		if (rc)
			pr_err("Couldn't disable OTG mode rc=%d\n", rc);
	}

	return rc;
}

int smb1351_otg_short_config(struct smb1351_charger *chip, int en)
{
	int value;

	if (en==1) {
		if (gpio_is_valid(chip->otg_usb_short_gpio)) {
			chip->otg_usb_short_state = true;

			smb1351_turn_otg_vbus(chip, 0);
			msleep(1000);
			gpio_set_value(chip->otg_usb_short_gpio, 1);
			msleep(1000);
			smb1351_turn_otg_vbus(chip, 1);

			if (!chip->otg_usb_short_state) {
				pr_info("find otg cable out, turn off vbus\n");
				gpio_set_value(chip->otg_usb_short_gpio, 0);
				smb1351_turn_otg_vbus(chip, 0);
			}
		}else
			chip->otg_usb_short_state = false;
	} else {
		if (gpio_is_valid(chip->otg_usb_short_gpio)) {
			smb1351_turn_otg_vbus(chip, 0);
			gpio_set_value(chip->otg_usb_short_gpio, 0);
			msleep(2000);
			smb1351_turn_otg_vbus(chip, 1);

			if (!chip->otg_usb_short_state) {
				pr_info("find otg cable out, turn off vbus\n");
				gpio_set_value(chip->otg_usb_short_gpio, 0);
				smb1351_turn_otg_vbus(chip, 0);
			}

			chip->otg_usb_short_state = false;
		} else
			chip->otg_usb_short_state = false;
	}

	value = gpio_get_value(chip->otg_usb_short_gpio);

	pr_info("state  is %d, en %d, ret %d\n",chip->otg_usb_short_state, en, value);

	return value;
}

static ssize_t otg_usb_short_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	int cmd;
	int value = -1;
	struct smb1351_charger *chip = smb1351_charger_chip;

	sscanf(buf, "%x", &cmd);

	if(chip->otg_usb_short_state==cmd)
	{
		pr_err("%s new cmd is as same as the old value(%d)\n", __func__, chip->otg_usb_short_state);
		return ret;
	}

	if(!chip->is_otg_on)
	{
		pr_err("%s otg is off, ignore usb short config\n", __func__);
		return ret;
	}

	value = smb1351_otg_short_config(chip, cmd);

	pr_info("otg usb short state  is %d, cmd %d, ret %d\n",chip->otg_usb_short_state, cmd, value);

	return ret;
}


ssize_t  otg_usb_short_get(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	int ret;
	struct smb1351_charger *chip = smb1351_charger_chip;

	pr_info("otg usb short state  is %d\n",chip->otg_usb_short_state);
	ret = sprintf(buf, "%d\n", chip->otg_usb_short_state);

	return ret;
}

static DEVICE_ATTR(otg_usb_short, S_IRUGO|S_IWUSR,otg_usb_short_get,otg_usb_short_set);
#endif

static struct attribute *fs_attrs[] = {
	&dev_attr_led_opt.attr,
	&dev_attr_fast_charger_need_change.attr,
#ifdef LENOVO_OTG_USB_SHORT
	&dev_attr_otg_usb_short.attr,
#endif
	&dev_attr_fast_charger_is_engineermode.attr,
	NULL,
};

static struct attribute_group fs_attr_group = {
	.attrs = fs_attrs,
};
#endif
static int smb1351_main_charger_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int rc;
	struct smb1351_charger *chip;
	struct power_supply *usb_psy;
	u8 reg = 0;

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		pr_debug("USB psy not found; deferring probe\n");
		return -EPROBE_DEFER;
	}

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		pr_err("Couldn't allocate memory\n");
		return -ENOMEM;
	}

	chip->client = client;
	chip->dev = &client->dev;
	chip->usb_psy = usb_psy;
	chip->fake_battery_soc = -EINVAL;
	INIT_DELAYED_WORK(&chip->chg_remove_work, smb1351_chg_remove_work);
	INIT_DELAYED_WORK(&chip->hvdcp_det_work, smb1351_hvdcp_det_work);
	INIT_DELAYED_WORK(&chip->rerun_apsd_work, smb1351_rerun_apsd_work);
	smb1351_wakeup_src_init(chip);

	smb1351_regulator_get(chip);

	/* probe the device to check if its actually connected */
	rc = smb1351_read_reg(chip, CHG_REVISION_REG, &reg);
	if (rc) {
		pr_err("Failed to detect smb1351, device may be absent\n");
		rc = -ENODEV;
		goto trash_ws;
	}
	pr_debug("smb1351 chip revision is %d\n", reg);

	rc = smb1351_parse_dt(chip);
	if (rc) {
		pr_err("Couldn't parse DT nodes rc=%d\n", rc);
		goto trash_ws;
	}

	/* using vadc and adc_tm for implementing pmic therm */
	if (chip->using_pmic_therm) {
		chip->vadc_dev = qpnp_get_vadc(chip->dev, "chg");
		if (IS_ERR(chip->vadc_dev)) {
			rc = PTR_ERR(chip->vadc_dev);
			if (rc != -EPROBE_DEFER)
				pr_err("vadc property missing\n");
			goto trash_ws;
		}
		chip->adc_tm_dev = qpnp_get_adc_tm(chip->dev, "chg");
		if (IS_ERR(chip->adc_tm_dev)) {
			rc = PTR_ERR(chip->adc_tm_dev);
			if (rc != -EPROBE_DEFER)
				pr_err("adc_tm property missing\n");
			goto trash_ws;
		}
	}

	i2c_set_clientdata(client, chip);
	chip->is_in_esr_state = false;
	chip->is_hv_fast_charging = false;
	chip->fast_charger_is_engineermode = false;

	chip->batt_psy.name		= "battery";
	chip->batt_psy.type		= POWER_SUPPLY_TYPE_BATTERY;
	chip->batt_psy.get_property	= smb1351_battery_get_property;
	chip->batt_psy.set_property	= smb1351_battery_set_property;
	chip->batt_psy.property_is_writeable =
					smb1351_batt_property_is_writeable;
	chip->batt_psy.properties	= smb1351_battery_properties;
	chip->batt_psy.num_properties	=
				ARRAY_SIZE(smb1351_battery_properties);
	chip->batt_psy.external_power_changed =
					smb1351_external_power_changed;
	chip->batt_psy.supplied_to	= pm_batt_supplied_to;
	chip->batt_psy.num_supplicants	= ARRAY_SIZE(pm_batt_supplied_to);

	chip->resume_completed = true;
	mutex_init(&chip->irq_complete);
	mutex_init(&chip->fcc_lock);
	mutex_init(&chip->parallel.lock);
	mutex_init(&chip->usb_suspend_status_lock);

	rc = power_supply_register(chip->dev, &chip->batt_psy);
	if (rc) {
		pr_err("Couldn't register batt psy rc=%d\n", rc);
		goto destroy_mutex;
	}

	INIT_DELAYED_WORK(&chip->parallel.parallel_work, smb1351_parallel_work);
	INIT_DELAYED_WORK(&chip->init_fg_work, smb1351_init_fg_work);
	INIT_DELAYED_WORK(&chip->iterm_check_soc_work,
					smb1351_iterm_check_soc_work);
#ifdef CHG_HEATBEAT_WORK
	INIT_DELAYED_WORK(&chip->heatbeat_work, smb1351_heatbeat_work);
#endif

	chip->is_slave = false;
	chip->is_factory_cable = false;
	chip->is_factory_need_write = false;
	chip->is_factory_mode = smb1351_charger_is_factory_mode();
	if (chip->is_factory_mode) {
		dev_warn(&client->dev, "Entering Factory Mode\n");
		rc = device_create_file(chip->dev, &dev_attr_force_chg_fail_clear);
		if (rc) {
			pr_err("couldn't create force_chg_fail_clear\n");
		}

		chip->mmi_reboot.notifier_call = smb1351_reboot;
		chip->mmi_reboot.next = NULL;
		chip->mmi_reboot.priority = 1;
		rc = register_reboot_notifier(&chip->mmi_reboot);
		if (rc)
			dev_err(chip->dev, "register for reboot failed\n");

		INIT_DELAYED_WORK(&chip->charger_check_work, smb1351_charger_check_worker);
	}

	dump_regs(chip);

	rc = smb1351_regulator_init(chip);
	if (rc) {
		pr_err("Couldn't initialize smb1351 ragulator rc=%d\n", rc);
		goto fail_smb1351_regulator_init;
	}

	rc = smb1351_hw_init(chip);
	if (rc) {
		pr_err("Couldn't intialize hardware rc=%d\n", rc);
		goto fail_smb1351_hw_init;
	}

	rc = smb1351_determine_initial_state(chip);
	if (rc) {
		pr_err("Couldn't determine initial state rc=%d\n", rc);
		goto fail_smb1351_hw_init;
	}

	chip->chg_pm_vadc_dev = NULL;
	chip->chg_pm_vadc_dev = qpnp_get_vadc(chip->dev, "chg-pm");
	if (IS_ERR(chip->chg_pm_vadc_dev)) {
		pr_err("get chg_pm_vadc_dev fail\n");
		rc = PTR_ERR(chip->chg_pm_vadc_dev);
		if (rc != -EPROBE_DEFER)
			pr_err("Couldn't get chg_pm_vadc_dev rc=%d\n",
					rc);
		goto fail_smb1351_hw_init;
	}

	chip->chg_pmi_vadc_dev = NULL;
	chip->chg_pmi_vadc_dev = qpnp_get_vadc(chip->dev, "chg-pmi");
	if (IS_ERR(chip->chg_pmi_vadc_dev)) {
		pr_err("get chg_pmi_vadc_dev fail\n");
		rc = PTR_ERR(chip->chg_pmi_vadc_dev);
		if (rc != -EPROBE_DEFER)
			pr_err("Couldn't get chg_pmi_vadc_dev rc=%d\n",
					rc);
		goto fail_smb1351_hw_init;
	}

#ifdef EXT_CHARGER_POWER_SUPPLY
	smb1351_charger_chip = chip;

	ext_charger_power_supply_init(chip);
	rc = sysfs_create_group(&chip->ext_charger_psy.dev->kobj, &fs_attr_group);
	if (rc) {
		pr_err("Couldn't register fs group=%d\n", rc);
		goto fail_smb1351_hw_init;
	}
#endif

/*	if (gpio_is_valid(chip->chg_en_gpio)) {
		gpio_request(chip->chg_en_gpio, "chg_en_gpio");
		gpio_direction_output(chip->chg_en_gpio,0);
		gpio_set_value(chip->chg_en_gpio,0);
	}
	else
		pr_err("%s chg_en_gpio is invalid\n", __func__);*/

	if (gpio_is_valid(chip->chg_led_gpio)) {
		gpio_request(chip->chg_led_gpio, "chg_led_gpio");
		gpio_direction_output(chip->chg_led_gpio,0);
		//gpio_set_value(chip->chg_en_gpio,0);
	}
	else
		pr_err("%s chg_led_gpio is invalid\n", __func__);

#ifdef LENOVO_OTG_USB_SHORT
	rc = gpio_request_one(chip->otg_usb_short_gpio, GPIOF_DIR_OUT, "otg_usb_short");
	if (rc)
		pr_err("%s otg_usb_short_gpio is invalid\n", __func__);
	else
		gpio_set_value(chip->otg_usb_short_gpio, 0);
#endif

	/* STAT irq configuration */
	if (client->irq) {
		rc = devm_request_threaded_irq(&client->dev, client->irq, NULL,
				smb1351_chg_stat_handler, IRQF_ONESHOT,
				"smb1351_chg_stat_irq", chip);
		if (rc) {
			pr_err("Failed STAT irq=%d request rc = %d\n",
				client->irq, rc);
			goto fail_smb1351_hw_init;
		}
		enable_irq_wake(client->irq);
	}

	if (chip->using_pmic_therm) {
		if (!chip->jeita_supported) {
			/* add hot/cold temperature monitor */
			chip->adc_param.low_temp = chip->batt_cold_decidegc;
			chip->adc_param.high_temp = chip->batt_hot_decidegc;
		} else {
			chip->adc_param.low_temp = chip->batt_cool_decidegc;
			chip->adc_param.high_temp = chip->batt_warm_decidegc;
		}
		chip->adc_param.timer_interval = ADC_MEAS2_INTERVAL_1S;
		chip->adc_param.state_request = ADC_TM_WARM_COOL_THR_ENABLE;
		chip->adc_param.btm_ctx = chip;
		chip->adc_param.threshold_notification =
				smb1351_chg_adc_notification;
		chip->adc_param.channel = LR_MUX1_BATT_THERM;

		rc = qpnp_adc_tm_channel_measure(chip->adc_tm_dev,
							&chip->adc_param);
		if (rc) {
			pr_err("requesting ADC error %d\n", rc);
			goto fail_smb1351_hw_init;
		}
	}

	create_debugfs_entries(chip);

	dump_regs(chip);

	wake_lock_init(&chip->charging_lock, WAKE_LOCK_SUSPEND, "charger_handshake_wakelock");

	pr_info("smb1351 successfully probed. charger=%d, batt=%d version=%s\n",
			chip->chg_present,
			smb1351_get_prop_batt_present(chip),
			smb1351_version_str[chip->version]);
	return 0;
fail_smb1351_hw_init:
	regulator_unregister(chip->otg_vreg.rdev);
fail_smb1351_regulator_init:
	power_supply_unregister(&chip->batt_psy);
destroy_mutex:
	ext_charger_power_supply_exit(chip);
	mutex_destroy(&chip->irq_complete);
	mutex_destroy(&chip->fcc_lock);
	mutex_destroy(&chip->parallel.lock);
	mutex_destroy(&chip->usb_suspend_status_lock);
trash_ws:
	wakeup_source_trash(&chip->smb1351_ws.source);

	return rc;
}

static int smb1351_parallel_slave_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int rc;
	struct smb1351_charger *chip;
	struct device_node *node = client->dev.of_node;

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		pr_err("Couldn't allocate memory\n");
		return -ENOMEM;
	}

	chip->client = client;
	chip->dev = &client->dev;
	chip->is_slave = true;

	chip->usb_suspended_status = of_property_read_bool(node,
					"qcom,charging-disabled");
	rc = of_property_read_u32(node, "qcom,float-voltage-mv",
						&chip->vfloat_mv);
	if (rc)
		chip->vfloat_mv = -EINVAL;
	rc = of_property_read_u32(node, "qcom,recharge-mv",
						&chip->recharge_mv);
	if (rc)
		chip->recharge_mv = -EINVAL;

	rc = of_property_read_u32(node, "qcom,parallel-en-pin-polarity",
					&chip->parallel_pin_polarity_setting);
	if (rc)
		chip->parallel_pin_polarity_setting = EN_BY_PIN_LOW_ENABLE;
	else
		chip->parallel_pin_polarity_setting =
				chip->parallel_pin_polarity_setting ?
				EN_BY_PIN_HIGH_ENABLE : EN_BY_PIN_LOW_ENABLE;

	i2c_set_clientdata(client, chip);

	chip->parallel_psy.name		= PSY_PARALLEL_NAME;//"usb-parallel";
	chip->parallel_psy.type		= POWER_SUPPLY_TYPE_USB_PARALLEL;
	chip->parallel_psy.get_property	= smb1351_parallel_get_property;
	chip->parallel_psy.set_property	= smb1351_parallel_set_property;
	chip->parallel_psy.properties	= smb1351_parallel_properties;
	chip->parallel_psy.property_is_writeable
				= smb1351_parallel_is_writeable;
	chip->parallel_psy.num_properties
				= ARRAY_SIZE(smb1351_parallel_properties);

	mutex_init(&chip->fcc_lock);
	mutex_init(&chip->irq_complete);
	mutex_init(&chip->usb_suspend_status_lock);
	smb1351_wakeup_src_init(chip);
#ifdef CHG_HEATBEAT_WORK
	INIT_DELAYED_WORK(&chip->heatbeat_work, smb1351_heatbeat_work);
#endif

	rc = power_supply_register(chip->dev, &chip->parallel_psy);
	if (rc) {
		pr_err("Couldn't register parallel psy rc=%d\n", rc);
		goto fail_register_psy;
	}

	chip->resume_completed = true;
	create_debugfs_entries(chip);

	smb1351_regulator_get(chip);
	smb1351_enable_volatile_writes(chip);
	smb1351_disable_hw_jieta(chip, 1);

	pr_info("smb1351 parallel successfully probed.\n");

	return 0;

fail_register_psy:
	wakeup_source_trash(&chip->smb1351_ws.source);
	mutex_destroy(&chip->irq_complete);
	mutex_destroy(&chip->fcc_lock);
	mutex_destroy(&chip->usb_suspend_status_lock);
	return rc;
}

static int smb1351_charger_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	if (is_parallel_slave(client))
		return smb1351_parallel_slave_probe(client, id);
	else
		return smb1351_main_charger_probe(client, id);
}

static int smb1351_charger_remove(struct i2c_client *client)
{
	struct smb1351_charger *chip = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&chip->chg_remove_work);
	power_supply_unregister(&chip->batt_psy);
#ifdef EXT_CHARGER_POWER_SUPPLY
	if (!chip->is_slave)
		ext_charger_power_supply_exit(chip);
#endif

	wakeup_source_trash(&chip->smb1351_ws.source);
	mutex_destroy(&chip->irq_complete);
	mutex_destroy(&chip->fcc_lock);
	mutex_destroy(&chip->usb_suspend_status_lock);
	if (is_parallel_slave(client))
		mutex_destroy(&chip->parallel.lock);
	debugfs_remove_recursive(chip->debug_root);
	return 0;
}

static void smb1351_shutdown(struct i2c_client *client)
{
	struct smb1351_charger *chip = i2c_get_clientdata(client);

	pr_info("%s:smb1351_shutdown \r\n", dev_name(chip->dev));

	if (chip->is_hv_fast_charging) {
		pr_info("%s: reset hv charger \r\n", dev_name(chip->dev));
		smb1351_set_hv_handshake_current(chip, 100);
		msleep(250);
	}
	smb1351_reset(chip);

	pr_info("smb1351_shutdown complete\r\n");
}

static int smb1351_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct smb1351_charger *chip = i2c_get_clientdata(client);

	/* no suspend resume activities for parallel charger */
	if (chip->is_slave)
		return 0;

	mutex_lock(&chip->irq_complete);
	chip->resume_completed = false;
	mutex_unlock(&chip->irq_complete);

	return 0;
}

static int smb1351_suspend_noirq(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct smb1351_charger *chip = i2c_get_clientdata(client);

	/* no suspend resume activities for parallel charger */
	if (chip->is_slave)
		return 0;

	if (chip->irq_waiting) {
		pr_err_ratelimited("Aborting suspend, an interrupt was detected while suspending\n");
		return -EBUSY;
	}
	return 0;
}

static int smb1351_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct smb1351_charger *chip = i2c_get_clientdata(client);

	/* no suspend resume activities for parallel charger */
	if (chip->is_slave)
		return 0;

	mutex_lock(&chip->irq_complete);
	chip->resume_completed = true;
	if (chip->irq_waiting) {
		mutex_unlock(&chip->irq_complete);
		smb1351_chg_stat_handler(client->irq, chip);
		enable_irq(client->irq);
	} else {
		mutex_unlock(&chip->irq_complete);
	}
	return 0;
}

static const struct dev_pm_ops smb1351_pm_ops = {
	.suspend	= smb1351_suspend,
	.suspend_noirq	= smb1351_suspend_noirq,
	.resume		= smb1351_resume,
};

static struct of_device_id smb1351_match_table[] = {
	{ .compatible = "qcom,smb1351-charger",},
	{ },
};

static const struct i2c_device_id smb1351_charger_id[] = {
	{"smb1351-charger", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, smb1351_charger_id);

static struct i2c_driver smb1351_charger_driver = {
	.driver		= {
		.name		= "smb1351-charger",
		.owner		= THIS_MODULE,
		.of_match_table	= smb1351_match_table,
		.pm		= &smb1351_pm_ops,
	},
	.probe		= smb1351_charger_probe,
	.remove		= smb1351_charger_remove,
	.shutdown	= smb1351_shutdown,
	.id_table	= smb1351_charger_id,
};

module_i2c_driver(smb1351_charger_driver);

MODULE_DESCRIPTION("smb1351 Charger");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:smb1351-charger");
