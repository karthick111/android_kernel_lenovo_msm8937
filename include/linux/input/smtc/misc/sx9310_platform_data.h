/*
 * include/linux/input/sx9310_platform_data.h
 *
 * SX9310 Platform Data
 * 2 cap differential  
 *
 * Copyright 2012 Semtech Corp.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef _SX9310_PLATFORM_DATA_H_
#define _SX9310_PLATFORM_DATA_H_
#define DIFFERENTIAL



struct smtc_reg_data {
	unsigned char reg;
	unsigned char val;
};
typedef struct smtc_reg_data smtc_reg_data_t;
typedef struct smtc_reg_data *psmtc_reg_data_t;


struct _buttonInfo {
	/*! The Key to send to the input */
	int keycode;
	/*! Mask to look for on Touch Status */
	int mask;
	/*! Current state of button. */
	int state;
};

struct _totalButtonInformation {
	struct _buttonInfo *buttons;
	int buttonSize;
	struct input_dev *input;
};

typedef struct _totalButtonInformation buttonInformation_t;
typedef struct _totalButtonInformation *pbuttonInformation_t;


struct sx9310_platform_data {
	int i2c_reg_num;
	struct smtc_reg_data *pi2c_reg;

	pbuttonInformation_t pbuttonInformation;

	int (*get_is_nirq_low)(void);

	int     (*init_platform_hw)(void);
	void    (*exit_platform_hw)(void);

#ifdef CONFIG_OF
	unsigned int irq_gpio;
	unsigned int irq_gpio_flags;
	struct pinctrl *cap_pinctrl;
	struct pinctrl_state *pinctrl_state_active;
	struct pinctrl_state *pinctrl_state_suspend;
#endif

};
typedef struct sx9310_platform_data sx9310_platform_data_t;
typedef struct sx9310_platform_data *psx9310_platform_data_t;

#endif
