/* 
 * linux/include/linux/mc3190.h
 *
 * Global definitions for Symbol MC3190 hardware
 * 
 * Author:	Mark Kennard <markkennard4@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef INCLUDE_LINUX_MC3190_H
#define INCLUDE_LINUX_MC3190_H

#include <linux/input.h>

#define EXT_GPIO(x)		(128 + (x))

struct mc3190_bl_platform_data {
	int		pwm_id;		/* Linux PWM channel number: 1 */
	unsigned long	pwm_period_ns;	/* full-scale PWM period, ns: 23077 */
	int		max_brightness;	/* matches WinCE PWMPeriod: 99 */
	int		dft_brightness;	/* your chosen default, e.g. 60 */
};

struct mc3190_bl_data {
	struct pwm_device	*pwm;
	struct mc3190_bl_platform_data *pdata;
	int			powered;	/* rail currently enabled? */
};

/*
 * CPLD Definitions
 */

#define MC3190_CPLD_BASE            0x14020000
#define MC3190_CPLD_SZ              0x80

#define MC3190_CPLD_REG_VERSION     0x00
#define MC3190_CPLD_REG_BL			0x1c
#define MC3190_CPLD_REG_USB_STATUS	0x24
#define MC3190_CPLD_REG_AUDIO		0x28
#define MC3190_CPLD_REG_LCD			0x74

#define MC3190_CPLD_LCD_LCD_BIT_0   (1 << 0)
#define MC3190_CPLD_LCD_LCD_BIT_1   (1 << 1)
#define MC3190_CPLD_LCD_READY_BIT   (1 << 2)
#define MC3190_CPLD_LCD_BL_BIT      (1 << 3)
#define MC3190_CPLD_LCD_LCD_RB_BIT  (1 << 4)

#define MC3190_CPLD_USB_CONNECTED_BIT	(1 << 5)

#define MC3190_CPLD_AUDIOAMP_BIT	(1 << 1)


extern u16 mc3190_cpld_read(unsigned int reg);
extern void mc3190_cpld_regon(u16 val, unsigned int reg);
extern void mc3190_cpld_regoff(u16 val, unsigned int reg);
extern void mc3190_cpld_rmw(u16 ormask, u16 andmask, unsigned int reg);

/*
 * AVR Microcontroller MFD Device
 */

#define PWRMICRO_TOUCH_UP		false
#define PWRMICRO_TOUCH_DOWN		true

#define MC3190_TAG_TOUCH_DATA       0x03
#define MC3190_TAG_SYS_ACK          0x04
#define MC3190_TAG_LOW_BATT_WAKE    0x06 			// FIXME: TBC Function and data
#define MC3190_TAG_BATT_CAP_EVENT   0x07			// FIXME: TBC Function and data
#define MC3190_TAG_DRIVER_ID        0x0B			// FIXME: TBC Function and data
#define MC3190_TAG_PWR_EVENT_TBC    0x10			// FIXME: TBC Function and data
	#define MC3190_TAG_PWR_BATTMV			0x05
	#define MC3190_TAG_PWR_EVENT_TBC_SUB06	0x06	// FIXME: TBC Function and data // Calibration for mV maybe?
	#define MC3190_TAG_PWR_BATTPERCENT		0x07	// Battery percent, and a flag in upper byte (07 on charge, 00 off charge?)
	#define MC3190_TAG_PWR_BATTTEMPERATURE	0x08	// FIXME: TBC Function and data // maybe temperature
	#define MC3190_TAG_PWR_EVENT_TBC_SUB09	0x09	// FIXME: TBC Function and data // 
	#define MC3190_TAG_PWR_EVENT_TBC_SUB14	0x14	// FIXME: TBC Function and data
	
#define MC3190_TAG_PWR_EVENT        0x11			// FIXME: TBC Function and data
#define MC3190_TAG_READ_EEPROM      0x14			// FIXME: TBC Function and data
#define MC3190_TAG_VER              0x20
#define MC3190_TAG_USERVER          0x21
#define MC3190_TAG_HEALTH_BYTE      0x23			// FIXME: TBC Function and data
#define MC3190_TAG_CMD_ERR          0x3F			// FIXME: TBC Function and data


struct avr_register {
	u8 data[3];
	bool is_valid_register;
	bool needs_update;
	bool always_needs_update;
	u8 needed_initiator;
	u8 alternate_initiator;
};

struct mc3190_touch;

struct mc3190_pwrmicro {
	struct device *dev;
	void __iomem *regs;
	int irq;

	spinlock_t lock;
	u32 last_rx_word;
	bool have_rx_word;
	struct work_struct rx_work;

	struct delayed_work ready_work;
	bool ready_for_gated_tags;

	bool have_last_cmd;
	u8   last_cmd_tag;
	u32  last_cmd_payload;
	u32 last_sent_word;

	bool have_reply;
	u32  last_reply;

	struct mc3190_touch *touch_dev;
	struct power_supply *battery_psy;

	bool in_state_machine;
	u8   state_machine;

	bool have_outstanding_request;
	u8   outstanding_request_tag;
	u32  outstanding_request_word;

	struct avr_register register_data[MC3190_TAG_CMD_ERR + 1];
	struct avr_register cmd10_subdata[MC3190_TAG_PWR_EVENT_TBC_SUB14 + 1];

	u8 versionMajor;
	u8 versionMinor;
	u16 versionUser;

};

void mc3190_set_touch_dev(struct mc3190_pwrmicro *core, struct mc3190_touch *touch);
void mc3190_touch_report(struct mc3190_touch *touch, u16 x, u16 y, bool touchstate);

void mc3190_set_battery_psy(struct mc3190_pwrmicro *priv, struct power_supply *psy);
int mc3190_get_battery_voltage(struct mc3190_pwrmicro *priv);
int mc3190_get_battery_capacity(struct mc3190_pwrmicro *priv);
int mc3190_get_battery_status(struct mc3190_pwrmicro *priv);
bool mc3190_is_battery_present(struct mc3190_pwrmicro *priv);

#endif // INCLUDE_LINUX_MC3190_H