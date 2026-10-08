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
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/completion.h>
#include <linux/workqueue.h>

#define EXT_GPIO(x)		(128 + (x))

struct mc3190_bl_platform_data {
	int	pwm_id;
	unsigned long pwm_period_ns;
	int	max_brightness;
	int	dft_brightness;
};

struct mc3190_bl_data {
	struct pwm_device *pwm;
	struct mc3190_bl_platform_data *pdata;
	int	powered;
};

/*
 * Various hardware functions
 */

extern void mc3190_panel_set_power(int on);
extern void mc3190_bl_set_power(int on);

/*
 * Board specific IRQs/Virtual IRQs
 */

#define IRQ_MC3190_BTUART   (IRQ_BOARD_START + 0)

/*
 * CPLD Definitions
 */

#define MC3190_CPLD_BASE            0x14020000
#define MC3190_CPLD_SZ              0x80

#define MC3190_CPLD_REG_VERSION     0x00
#define MC3190_CPLD_REG_ISR			0x08
#define MC3190_CPLD_REG_BL			0x1c
#define MC3190_CPLD_REG_USB_STATUS	0x24
#define MC3190_CPLD_REG_AUDIO		0x28
#define MC3190_CPLD_REG_WIFI_PWR	0x40
#define MC3190_CPLD_REG_BT_1		0x44
#define MC3190_CPLD_REG_BT_2		0x64
#define MC3190_CPLD_REG_WIFI_CTRL	0x6C
#define MC3190_CPLD_REG_LCD			0x74

#define MC3190_CPLD_LCD_LCD_BIT_0   	(1 << 0)
#define MC3190_CPLD_LCD_LCD_BIT_1   	(1 << 1)
#define MC3190_CPLD_LCD_EN_BIT      	(1 << 0)
#define MC3190_CPLD_LCD_READY_BIT   	(1 << 2)
#define MC3190_CPLD_LCD_BL_BIT      	(1 << 3)

#define MC3190_CPLD_WIFI_PWR_BIT	(1 << 0)
#define MC3190_CPLD_WIFI_READY_BIT	(1 << 5)
#define MC3190_CPLD_WIFI_RESET_BIT	(1 << 1)

#define MC3190_CPLD_BT_POWER_BIT		(1 << 0)
#define MC3190_CPLD_BT_UART_BIT			(1 << 6)

#define MC3190_CPLD_USB_CONNECTED_BIT	(1 << 5)

#define MC3190_CPLD_AUDIOAMP_BIT		(1 << 1)

#define MC3190_CPLD_ISR_BTUART_BIT		(1 << 17)
#define MC3190_CPLD_ISR_MASK			(MC3190_CPLD_ISR_BTUART_BIT) 	// Add other handled IRQs later


extern u16 mc3190_cpld_read(unsigned int reg);
extern void mc3190_cpld_regon(u16 val, unsigned int reg);
extern void mc3190_cpld_regoff(u16 val, unsigned int reg);
extern void mc3190_cpld_rmw(u16 ormask, u16 andmask, unsigned int reg);
extern int mc3190_cpld_wait_bit(unsigned int reg, u16 bit, bool set, unsigned int timeout_ms);
extern int mc3190_cpld_regon_wait(u16 val, unsigned int reg, u16 wait_bit, unsigned int timeout_ms);
extern int mc3190_cpld_regoff_wait(u16 val, unsigned int reg, u16 wait_bit, unsigned int timeout_ms);

/*
 * AVR Microcontroller MFD Device
 */

#define PWRMICRO_TOUCH_UP		false
#define PWRMICRO_TOUCH_DOWN		true

#define MC3190_TAG_TOUCH_WAKE		0x02			// Handles GetTchPnlSetup, SetTchPnlSetup, GetWakeupTimeout, GetTouchState, SetTouchState, SetWakeupTimeout // FIXME: Not implemented
#define MC3190_TAG_TOUCH_DATA       0x03
#define MC3190_TAG_SYS_ACK          0x04
#define MC3190_TAG_SET_LOWBATTWAKE	0x05 			// FIXME: Not implemented
#define MC3190_TAG_LOW_BATT_WAKE    0x06 			// FIXME: Not implemented
#define MC3190_TAG_BATT_CAP_EVENT   0x07
#define MC3190_TAG_SET_BATTRESERVE	0x08			// FIXME: Not implemented
#define MC3190_TAG_REBOOTCMD		0x09			// FIXME: Not implemented
#define MC3190_TAG_GET_WAKE_CAUSE	0x0A			// FIXME: Not implemented
#define MC3190_TAG_DRIVER_ID        0x0B			// FIXME: TBC Function and data
#define MC3190_TAG_SUSPEND_WAKE		0x0C			// FIXME: Not implemented
#define MC3190_TAG_SETREBOOTTIMES	0x0E			// FIXME: Not implemented
#define MC3190_TAG_GETREBOOTTIMES	0x0F			// FIXME: Not implemented
#define MC3190_TAG_PWR_EVENT_TBC    0x10			// FIXME: TBC Function and data
	#define MC3190_TAG_PWR_BATTID_2			0x01	// 2nd Stage read // FIXME: Not implemented
	#define MC3190_TAG_PWR_BATTID_1			0x04	// 1st Stage read // FIXME: Not implemented
	#define MC3190_TAG_PWR_BATTMV			0x05
	#define MC3190_TAG_PWR_BATTCURRENT		0x06	// FIXME: AVR responds to this request with a 0x1005 response.
	#define MC3190_TAG_PWR_BATTPERCENT		0x07
	#define MC3190_TAG_PWR_BATTTEMPERATURE	0x08
	#define MC3190_TAG_PWR_BACKUPBATTVOLT	0x09
	#define MC3190_TAG_PWR_AGGREGATECHRG_2	0x0A	// 2nd Stage read // FIXME: Not implemented
	#define MC3190_TAG_PWR_AGGREGATECHRG_1	0x0B	// 1st Stage read // FIXME: Not implemented
	#define MC3190_TAG_PWR_BATTRATEDCAP		0x0D
	#define MC3190_TAG_PWR_BATTMFGDATE_1	0x0E	// 1st Stage read // FIXME: Not implemented
	#define MC3190_TAG_PWR_BATTMFGDATE_2	0x0F	// 2nd Stage read // FIXME: Not implemented
	#define MC3190_TAG_PWR_BATTPARTNUM_2	0x10	// 2nd Stage read // FIXME: Not implemented
	#define MC3190_TAG_PWR_BATTPARTNUM_1	0x11	// 1st Stage read // FIXME: Not implemented
	#define MC3190_TAG_PWR_BATTINSTVOLT		0x12	// FIXME: Not implemented
	#define MC3190_TAG_PWR_BATTINSTCURRENT	0x13
	#define MC3190_TAG_PWR_BATTTYPE			0x14	// FIXME: Not implemented

	#define MC3190_TAG_PWR_MAXTAGS			MC3190_TAG_PWR_BATTTYPE
	
#define MC3190_TAG_PWR_EVENT        0x11			// Charge status (0x8000) and battery level (0xff = unknown, 0x64 = max)
#define MC3190_TAG_OPEN_EEPROM		0x12			// Has second stage tag of 0x12010000 for open, 0x12020000 for close // FIXME: Not implemented
#define MC3190_TAG_READ_EEPROM      0x14			// FIXME: Not implemented
#define MC3190_TAG_SETTOUCHCFG2		0x15			// FIXME: Not implemented
#define MC3190_TAG_SETLED1			0x16			// FIXME: Not implemented
#define MC3190_TAG_SETLED2			0x17			// FIXME: Not implemented
#define MC3190_TAG_IEEE1725LIMITS   0x19			// FIXME: Not implemented
#define MC3190_TAG_IEEE1725DISABLE	0x1A			// FIXME: Not implemented
#define MC3190_TAG_SETBATTFIRSTUSE	0x1B			// FIXME: Not implemented
#define MC3190_TAG_SETHEALTHBYTE	0x1C			// FIXME: Not implemented
#define MC3190_TAG_SETUSBCHRGLIMIT	0x1D			// FIXME: Not implemented
#define MC3190_TAG_GETUSBCHRGLIMIT	0x1E			// FIXME: Not implemented
#define MC3190_TAG_SENDGIFTBATTCMD	0x1F			// FIXME: Not implemented
#define MC3190_TAG_VER              0x20			// GetTermType
#define MC3190_TAG_USERVER          0x21
#define MC3190_TAG_BATTFIRSTUSEDATE	0x22			// FIXME: Not implemented
#define MC3190_TAG_HEALTH_BYTE      0x23			// FIXME: Not implemented
#define MC3190_TAG_SETUSBCHRGSTATE	0x24			// FIXME: Not implemented
#define MC3190_TAG_GETFLASHBYTE		0x2A			// FIXME: Not implemented
#define MC3190_TAG_CMD_ERR          0x3F			// FIXME: TBC Function and data


/* Version 0x20 is GetTermType, 0x11 is the power event; both are plain reads. */
#define MC3190_DRIVER_ID_WORD		0x0B010000	// WinCE SetDriverID. Also (re)opens the AVR's TX gate.

struct avr_register {
	u8 data[3];
	bool is_valid_register;
	bool needs_update;
	bool oneshot_read;
	u8 needed_initiator;
	u8 alternate_initiator;
};

/* Diagnostic counters, exposed through the pm_status sysfs attribute. */
struct mc3190_pm_stats {
	u32 frames;		/* words received from the AVR */
	u32 tag0_frames;	/* dummy frames */
	u32 retry_frames;	/* AVR re-sent a word (bit 7 set) */
	u32 rx_dropped;		/* RX ring was full */
	u32 ror;		/* SSP receive overrun */
	u32 bce;		/* SSP bit count error */
	u32 tur;		/* SSP transmit underrun */
	u32 resyncs;
	u32 recoveries;
	u32 handshake_ok;
	u32 handshake_fail;
	u32 req_sent;
	u32 req_retries;
	u32 req_abandoned;	/* timed out MC3190_REQ_MAX_RETRIES times */
	u32 cmd_errors;		/* AVR answered 0x3F */
	u32 bad_subtag;		/* 0x10 reply with out-of-range sub tag */
	u32 pings;
	u32 ping_fail;
	u32 stage_fail;		/* couldn't write SSDR (BSY stuck / FIFO full) */
	u32 stuck_touch;
	u32 avr_resets;
};

#define MC3190_RXQ_SIZE		16

struct mc3190_touch;

struct mc3190_pwrmicro {
	struct device *dev;
	void __iomem *regs;
	int irq;

	/* Locking:
	 *  lock          - SSP registers and the RX ring. Taken from hard IRQ.
	 *  ack_lock      - ACK GPIO pulse sequencing. Taken from hard IRQ.
	 *  proto_lock    - everything below the "protocol state" comment.
	 *  recover_lock  - serialises resync / handshake / health checks.
	 */
	spinlock_t lock;
	spinlock_t ack_lock;
	struct mutex proto_lock;
	struct mutex recover_lock;

	/* RX ring: hard IRQ drains the SSP FIFO into it, the IRQ thread dispatches it */
	u32 rxq[MC3190_RXQ_SIZE];
	unsigned int rxq_head;
	unsigned int rxq_tail;

	struct workqueue_struct *wq;
	struct delayed_work ready_work;
	struct delayed_work watch_work;
	struct work_struct cmd_work;
	unsigned long cmd_flags;
	bool shutting_down;

	struct mc3190_touch *touch_dev;
	struct power_supply *battery_psy;

	/* ---- protocol state (proto_lock) ---- */
	bool ready_for_gated_tags;
	bool handshaking;		/* suppress request chaining while we run the handshake */

	bool have_last_cmd;
	u8   last_cmd_tag;
	u32  last_cmd_payload;
	u32  last_sent_word;

	bool in_state_machine;
	u8   state_machine;

	bool have_outstanding_request;
	u8   outstanding_request_tag;
	u32  outstanding_request_word;
	unsigned long outstanding_deadline;
	u8   outstanding_retries;	/* watchdog re-sends */
	u8   outstanding_stages;	/* re-stages on incoming frames */
	bool sent_pending_frame;	/* we staged a word and haven't seen a frame since */
	unsigned int consecutive_failures;

	/* synchronous requests (handshake / ping) */
	struct completion sync_done;
	bool sync_active;
	int  sync_result;

	/* link health */
	unsigned long last_frame_jiffies;
	unsigned long last_touch_jiffies;
	bool ssp_error;			/* set from IRQ on ROR/BCE or when staging fails */
	bool link_down;
	unsigned long next_recover;
	unsigned int recover_failures;

	struct mc3190_pm_stats stats;

	/* Battery percentage smoothing (see mc3190-battery.c), protected by soc_lock */
	spinlock_t soc_lock;
	bool soc_have_real;			/* AVR has reported a real percentage at least once */
	u8 soc_last_real;			/* last real percentage the AVR reported */
	unsigned long soc_last_real_jiffies;	/* when it reported it */
	int soc_reported;			/* what we currently show while the AVR sends 0xFF */
	unsigned long soc_last_drop_jiffies;

	struct avr_register register_data[MC3190_TAG_CMD_ERR + 1];
	struct avr_register cmd10_subdata[MC3190_TAG_PWR_MAXTAGS + 1];

	u8 versionMajor;
	u8 versionMinor;
	u16 versionUser;

};

void mc3190_set_touch_dev(struct mc3190_pwrmicro *core, struct mc3190_touch *touch);
void mc3190_touch_report(struct mc3190_touch *touch, u16 x, u16 y, bool touchstate);

void mc3190_set_battery_psy(struct mc3190_pwrmicro *priv, struct power_supply *psy);
int mc3190_get_battery_current(struct mc3190_pwrmicro *priv);
int mc3190_get_battery_voltage(struct mc3190_pwrmicro *priv);
int mc3190_get_battery_capacity(struct mc3190_pwrmicro *priv);
int mc3190_get_battery_status(struct mc3190_pwrmicro *priv);
int mc3190_get_battery_temperature(struct mc3190_pwrmicro *priv);
int mc3190_get_battery_rated_capacity(struct mc3190_pwrmicro *priv);
int mc3190_get_backup_battery_voltage(struct mc3190_pwrmicro *priv);
bool mc3190_is_battery_present(struct mc3190_pwrmicro *priv);

#endif // INCLUDE_LINUX_MC3190_H