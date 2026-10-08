/*
 * mc3190-micro.c -- MFD Driver for Symbol MC3190 PwrMicro
 * 
 * By Mark Kennard <markkennard4@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/completion.h>
#include <linux/jiffies.h>
#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/mfd/core.h>
#include <linux/power_supply.h>

#include <linux/mc3190.h>

#ifdef CONFIG_MFD_MC3190_PM_DEBUG
#define DEBUG
#endif // CONFIG_MFD_MC3190_PM_DEBUG

#define DRV_NAME "mc3190-pwrmicro"

#define SSCR0				0x00
#define SSCR1				0x04
#define SSSR				0x08
#define SSDR				0x10

#define SSCR0_DSS_32BIT		0xF
#define SSCR0_SSE			(1 << 7)
#define SSCR0_EDSS			(1 << 20)
#define SSCR0_TIM			(1 << 23)

#define SSCR1_RIE			(1 << 0)
#define SSCR1_RWOT			(1 << 23)
#define SSCR1_SFRMDIR		(1 << 24)
#define SSCR1_SCLKDIR		(1 << 25)
#define SSCR1_SCFR			(1 << 28)

#define SSSR_TNF			(1 << 2)
#define SSSR_RNE			(1 << 3)
#define SSSR_BSY			(1 << 4)
#define SSSR_ROR			(1 << 7)
#define SSSR_TUR			(1 << 21)
#define SSSR_CSS			(1 << 22)
#define SSSR_BCE			(1 << 23)
#define SSSR_W1C_MASK		0x00BC0080

#define PWRMICRO_ACK_OUT_GPIO 17
#define PWRMICRO_RESET_GPIO   16

#ifdef CONFIG_MFD_MC3190_PM_DEBUG
#define printk_pmdbg(dev, format, arg...)		\
	dev_printk(KERN_INFO , dev , format , ## arg)
#else
#define printk_pmdbg(dev, format, arg...)		\
	({ if (0) dev_printk(KERN_DEBUG, dev, format, ##arg); 0; })
#endif // CONFIG_MFD_MC3190_PM_DEBUG

#define MC3190_RETRY_BIT   0x80
#define MC3190_TAG_MASK    0x7F

#define MC3190_READY_DELAY_MS   300

#define PWRMICRO_STATE_TOUCH	1
#define PWRMICRO_STATE_BATTERY	2

#define MC3190_REQ_TIMEOUT_MS		300
#define MC3190_REQ_MAX_RETRIES		3
#define MC3190_REQ_MAX_STAGES		4
#define MC3190_FAILS_BEFORE_RECOVER	3
#define MC3190_WATCH_PERIOD_MS		500
#define MC3190_SILENCE_MS			12000
#define MC3190_TOUCH_STUCK_MS		1000
#define MC3190_BACKOFF_MAX_MS		30000

#define MC3190_CMD_RECOVER	0
#define MC3190_CMD_AVR_RESET	1

static int pm_handshake = 1;
module_param(pm_handshake, int, 0644);
MODULE_PARM_DESC(pm_handshake, "Run the WinCE bring-up handshake (version, DriverID, power event) at probe and after every recovery");

static int pm_watchdog = 1;
module_param(pm_watchdog, int, 0644);
MODULE_PARM_DESC(pm_watchdog, "Run the request-timeout / silence / SSP-error watchdog and auto-recovery");

static int reset_active_high = 1;
module_param(reset_active_high, int, 0444);
MODULE_PARM_DESC(reset_active_high, "GPIO16 level that asserts the AVR reset. WinCE idles it low, so high is assumed");

static int reset_pulse_ms = 20;
module_param(reset_pulse_ms, int, 0644);
MODULE_PARM_DESC(reset_pulse_ms, "Length of the AVR reset pulse");

static int avr_boot_ms = 1000;
module_param(avr_boot_ms, int, 0644);
MODULE_PARM_DESC(avr_boot_ms, "Time to wait for the AVR to boot after a reset pulse");

static int reset_after_failures = 0;
module_param(reset_after_failures, int, 0644);
MODULE_PARM_DESC(reset_after_failures, "Pulse the AVR reset after this many failed recoveries in a row (0 = never)");

static void pwrmicro_init_tags(struct mc3190_pwrmicro *priv) {
	priv->register_data[MC3190_TAG_PWR_EVENT_TBC].is_valid_register = true;
	priv->register_data[MC3190_TAG_PWR_EVENT_TBC].needed_initiator = MC3190_TAG_PWR_EVENT;
	priv->register_data[MC3190_TAG_PWR_EVENT_TBC].alternate_initiator = MC3190_TAG_PWR_EVENT_TBC;

	priv->cmd10_subdata[MC3190_TAG_PWR_BATTMV].is_valid_register = true;
	priv->cmd10_subdata[MC3190_TAG_PWR_BATTPERCENT].is_valid_register = true;
	priv->cmd10_subdata[MC3190_TAG_PWR_BATTTEMPERATURE].is_valid_register = true;
	priv->cmd10_subdata[MC3190_TAG_PWR_BACKUPBATTVOLT].is_valid_register = true;
	priv->cmd10_subdata[MC3190_TAG_PWR_BATTRATEDCAP].is_valid_register = true;
	priv->cmd10_subdata[MC3190_TAG_PWR_BATTRATEDCAP].oneshot_read = true;
	priv->cmd10_subdata[MC3190_TAG_PWR_BATTINSTCURRENT].is_valid_register = true;
	priv->cmd10_subdata[MC3190_TAG_PWR_BATTTYPE].is_valid_register = true;

	priv->register_data[MC3190_TAG_PWR_EVENT].is_valid_register = true;
	priv->register_data[MC3190_TAG_PWR_EVENT].needed_initiator = MC3190_TAG_BATT_CAP_EVENT;

	priv->register_data[MC3190_TAG_VER].is_valid_register = true;
	priv->register_data[MC3190_TAG_VER].needed_initiator = MC3190_TAG_PWR_EVENT;

	priv->register_data[MC3190_TAG_USERVER].is_valid_register = true;
	priv->register_data[MC3190_TAG_USERVER].needed_initiator = MC3190_TAG_VER;
	priv->register_data[MC3190_TAG_USERVER].oneshot_read = true;
	priv->register_data[MC3190_TAG_USERVER].needs_update = true;

	priv->register_data[MC3190_TAG_CMD_ERR].is_valid_register = false;
}

/* ------------------------------------------------------------------ */
/* ACK GPIO (GPIO17)                                                   */
/* ------------------------------------------------------------------ */

static void pwrmicro_ack_set(struct mc3190_pwrmicro *priv, int level)
{
	unsigned long flags;

	spin_lock_irqsave(&priv->ack_lock, flags);
	gpio_set_value(PWRMICRO_ACK_OUT_GPIO, level);
	spin_unlock_irqrestore(&priv->ack_lock, flags);
}

/*
 * The whole pulse runs under ack_lock with interrupts off, so the RX
 * interrupt (which raises ACK) can't land in the middle of it and leave the
 * final line state order-dependent.
 */
static void pwrmicro_send_ack(struct mc3190_pwrmicro *priv, bool awaiting_packet)
{
	unsigned long flags;

	spin_lock_irqsave(&priv->ack_lock, flags);
	gpio_set_value(PWRMICRO_ACK_OUT_GPIO, 0);	// Quick pulse of ACK_OUT as an acknowledge of the packet just received
	udelay(3);
	gpio_set_value(PWRMICRO_ACK_OUT_GPIO, 1);
	if (awaiting_packet) {
		udelay(100);
		gpio_set_value(PWRMICRO_ACK_OUT_GPIO, 0);	// Long pulse of ACK_OUT to tell AVR it can send us another packet
	}
	spin_unlock_irqrestore(&priv->ack_lock, flags);
}

/* ------------------------------------------------------------------ */
/* SSP helpers                                                         */
/* ------------------------------------------------------------------ */

/*
 * Port of WinCE Ssp4_ResetAndResync(). Leaves SSDR=0 staged and ACK low.
 * Safe to call from process context at any time; the RX ring is flushed.
 */
static int mc3190_ssp_resync(struct mc3190_pwrmicro *priv)
{
	unsigned long flags;
	int i, ret = 0;
	u32 sscr0 = SSCR0_EDSS | SSCR0_TIM | SSCR0_DSS_32BIT;			/* 0x90000F */
	u32 sscr1 = SSCR1_SCFR | SSCR1_SCLKDIR | SSCR1_SFRMDIR | SSCR1_RWOT;	/* 0x13800000 */

	pwrmicro_ack_set(priv, 1);

	spin_lock_irqsave(&priv->lock, flags);

	for (i = 0; i < 64 && (readl(priv->regs + SSSR) & SSSR_RNE); i++)
		(void)readl(priv->regs + SSDR);

	writel(SSSR_W1C_MASK, priv->regs + SSSR);

	writel(readl(priv->regs + SSCR0) & ~SSCR0_SSE, priv->regs + SSCR0);
	udelay(10);
	writel(sscr0, priv->regs + SSCR0);
	writel(sscr1, priv->regs + SSCR1);
	writel(sscr0 | SSCR0_SSE, priv->regs + SSCR0);

	/* WinCE spins forever here; we give up after ~1 ms */
	for (i = 0; i < 1000 && (readl(priv->regs + SSSR) & SSSR_CSS); i++)
		udelay(1);
	if (i == 1000)
		ret = -ETIMEDOUT;

	writel(0, priv->regs + SSDR);
	writel(SSSR_W1C_MASK, priv->regs + SSSR);
	writel(sscr1 | SSCR1_RIE, priv->regs + SSCR1);

	priv->rxq_head = 0;
	priv->rxq_tail = 0;
	priv->ssp_error = false;

	spin_unlock_irqrestore(&priv->lock, flags);

	pwrmicro_ack_set(priv, 0);

	priv->stats.resyncs++;
	if (ret)
		dev_warn(priv->dev, "SSP CSS did not clear after resync\n");
	return ret;
}

/*
 * Write one word into the SSP TX FIFO (WinCE Ssp4_SendCommand32 minus the
 * ACK drop, which the caller does).
 */
static int mc3190_stage_word(struct mc3190_pwrmicro *priv, u32 word)
{
	unsigned long flags;
	int i;

	for (i = 0; i < 1000; i++) {	/* up to 10 ms, as WinCE */
		if (!(readl(priv->regs + SSSR) & SSSR_BSY))
			break;
		udelay(10);
	}
	if (i == 1000) {
		priv->stats.stage_fail++;
		priv->ssp_error = true;
		return -EBUSY;
	}

	spin_lock_irqsave(&priv->lock, flags);
	if (!(readl(priv->regs + SSSR) & SSSR_TNF)) {
		spin_unlock_irqrestore(&priv->lock, flags);
		priv->stats.stage_fail++;
		priv->ssp_error = true;
		return -ENOSPC;
	}
	writel(word, priv->regs + SSDR);
	spin_unlock_irqrestore(&priv->lock, flags);

	priv->last_sent_word = word;
	return 0;
}

/* ------------------------------------------------------------------ */
/* Request tracking (proto_lock held for all of these)                 */
/* ------------------------------------------------------------------ */

static void mc3190_req_begin_locked(struct mc3190_pwrmicro *priv, u32 word)
{
	priv->have_outstanding_request = true;
	priv->outstanding_request_tag = (word >> 24) & MC3190_TAG_MASK;
	priv->outstanding_request_word = word;
	priv->outstanding_retries = 0;
	priv->outstanding_stages = 0;
	priv->outstanding_deadline = jiffies + msecs_to_jiffies(MC3190_REQ_TIMEOUT_MS);
}

/* A reply matches by tag, and for tag 0x10 also by sub tag (as WinCE does) */
static bool mc3190_response_matches(u32 req, u32 rx)
{
	u8 rtag = (req >> 24) & MC3190_TAG_MASK;
	u8 tag = (rx >> 24) & MC3190_TAG_MASK;

	if (rtag != tag)
		return false;
	if (tag == MC3190_TAG_PWR_EVENT_TBC)
		return ((req >> 16) & 0xFF) == ((rx >> 16) & 0xFF);
	return true;
}

/* Stop asking for whatever this request was for, so the chain moves on */
static void mc3190_mark_request_done(struct mc3190_pwrmicro *priv, u32 word)
{
	u8 tag = (word >> 24) & MC3190_TAG_MASK;

	if (tag == MC3190_TAG_PWR_EVENT_TBC) {
		u8 sub = (word >> 16) & 0xFF;

		if (sub <= MC3190_TAG_PWR_MAXTAGS)
			priv->cmd10_subdata[sub].needs_update = false;
	} else if (tag < ARRAY_SIZE(priv->register_data)) {
		priv->register_data[tag].needs_update = false;
	}
}

/* Stage the outstanding request and drop ACK so the AVR clocks a frame for it */
static void mc3190_send_outstanding(struct mc3190_pwrmicro *priv)
{
	if (mc3190_stage_word(priv, priv->outstanding_request_word)) {
		pwrmicro_send_ack(priv, false);
		return;
	}
	priv->outstanding_stages++;
	priv->sent_pending_frame = true;
	priv->stats.req_sent++;
	priv->outstanding_deadline = jiffies + msecs_to_jiffies(MC3190_REQ_TIMEOUT_MS);
	pwrmicro_send_ack(priv, true);
}

/* After handling a frame: push the next request if one is pending, else plain ACK */
static void mc3190_ack_or_send(struct mc3190_pwrmicro *priv)
{
	if (priv->have_outstanding_request &&
	    priv->outstanding_stages < MC3190_REQ_MAX_STAGES) {
		mc3190_send_outstanding(priv);
		return;
	}
	pwrmicro_send_ack(priv, false);
}

static void mc3190_request_information(struct mc3190_pwrmicro *priv, u8 rx_tag)
{
	int i = 0;
	for (i = 0; i < MC3190_TAG_CMD_ERR; i++) {
		if (priv->register_data[i].is_valid_register && (priv->register_data[i].needs_update) &&
				(priv->register_data[i].needed_initiator == rx_tag || priv->register_data[i].alternate_initiator == rx_tag)) {
			if (i == MC3190_TAG_PWR_EVENT_TBC) {
				int j = 0;
				for (j = 0; j <= MC3190_TAG_PWR_MAXTAGS; j++) {
					if (priv->cmd10_subdata[j].is_valid_register && priv->cmd10_subdata[j].needs_update) {
						mc3190_req_begin_locked(priv, (i << 24) | (j << 16));
						printk_pmdbg(priv->dev, "cmd10 tag 0x%02x needs update: outstanding_request_word=0x%08x\n",
											j, priv->outstanding_request_word);
						return;
					}
				}
				priv->register_data[i].needs_update = false;
				continue;

			}
			mc3190_req_begin_locked(priv, i << 24);
			printk_pmdbg(priv->dev, "tag 0x%02x needs update: outstanding_request_word=0x%08x\n",
								i, priv->outstanding_request_word);
			return;
		}
	}
}

/* Chain the next information request, unless one is in flight or we're handshaking */
static void mc3190_chain(struct mc3190_pwrmicro *priv, u8 tag)
{
	if (priv->handshaking || priv->have_outstanding_request)
		return;
	mc3190_request_information(priv, tag);
}

/* Give up on the outstanding request (error reply or retries exhausted) */
static void mc3190_req_abandon_locked(struct mc3190_pwrmicro *priv, int err)
{
	u8 tag = priv->outstanding_request_tag;

	mc3190_mark_request_done(priv, priv->outstanding_request_word);
	priv->have_outstanding_request = false;
	if (priv->sync_active) {
		priv->sync_result = err;
		complete(&priv->sync_done);
	}
	mc3190_chain(priv, tag);
}

static void mc3190_store_register_data(struct mc3190_pwrmicro *priv, u8 reg, u32 payload) {
	if (reg >= ARRAY_SIZE(priv->register_data))
		return;

	if (reg != MC3190_TAG_PWR_EVENT_TBC) {
		priv->register_data[reg].data[0] = payload >> 16 & 0xFF;
		priv->register_data[reg].data[1] = payload >> 8 & 0xFF;
		priv->register_data[reg].data[2] = payload & 0xFF;
		priv->register_data[reg].needs_update = false;
	} else {
		u8 subcmd = (payload >> 16) & 0xFF;

		/* cmd10_subdata[] only has MC3190_TAG_PWR_MAXTAGS + 1 entries */
		if (subcmd > MC3190_TAG_PWR_MAXTAGS) {
			priv->stats.bad_subtag++;
			if (printk_ratelimit())
				dev_warn(priv->dev, "cmd 0x10 reply with bad sub tag 0x%02x (payload 0x%06x)\n",
					 subcmd, payload);
			return;
		}
		priv->cmd10_subdata[subcmd].data[0] = payload >> 8 & 0xFF;
		priv->cmd10_subdata[subcmd].data[1] = payload & 0xFF;
		priv->cmd10_subdata[subcmd].needs_update = false;

		/* Remember the last real charge level (0xFF = AVR doesn't know, e.g. on charge) */
		if (subcmd == MC3190_TAG_PWR_BATTPERCENT && (payload & 0xFF) <= 100) {
			unsigned long flags;

			spin_lock_irqsave(&priv->soc_lock, flags);
			priv->soc_last_real = payload & 0xFF;
			priv->soc_last_real_jiffies = jiffies;
			priv->soc_have_real = true;
			spin_unlock_irqrestore(&priv->soc_lock, flags);
		}
	}

}

static void pwrmicro_reset_always_needs_updates(struct mc3190_pwrmicro *priv) {
	int i = 0;
	for (i = 0; i < MC3190_TAG_CMD_ERR; i++) {
		if (priv->register_data[i].is_valid_register && !priv->register_data[i].oneshot_read) {
			priv->register_data[i].needs_update = true;
		}
	}

	for (i = 0; i <= MC3190_TAG_PWR_MAXTAGS; i++) {
		if (priv->cmd10_subdata[i].is_valid_register && !priv->cmd10_subdata[i].oneshot_read) {
			priv->cmd10_subdata[i].needs_update = true;
		}
	}
}

static void mc3190_release_touch_locked(struct mc3190_pwrmicro *priv)
{
#ifdef CONFIG_TOUCHSCREEN_MC3190
	if (priv->in_state_machine && priv->state_machine == PWRMICRO_STATE_TOUCH &&
	    priv->touch_dev)
		mc3190_touch_report(priv->touch_dev, 0, 0, PWRMICRO_TOUCH_UP);
#endif // CONFIG_TOUCHSCREEN_MC3190
	priv->in_state_machine = false;
	priv->state_machine = 0;
}

/* ------------------------------------------------------------------ */
/* Frame dispatch (IRQ thread, proto_lock held)                        */
/* ------------------------------------------------------------------ */

static void mc3190_dispatch_locked(struct mc3190_pwrmicro *priv, u32 rx_word)
{
	u8 raw_tag = (rx_word >> 24) & 0xFF;
	bool is_retry = raw_tag & MC3190_RETRY_BIT;
	u8 tag = raw_tag & MC3190_TAG_MASK;
	u32 payload = rx_word & 0xFFFFFF;
	bool is_same_as_last = priv->have_last_cmd &&
			        tag == priv->last_cmd_tag &&
			        payload == priv->last_cmd_payload;
	bool was_pending = priv->sent_pending_frame;

	priv->sent_pending_frame = false;
	priv->last_frame_jiffies = jiffies;
	priv->link_down = false;
	priv->stats.frames++;

	if (priv->have_outstanding_request &&
	    mc3190_response_matches(priv->outstanding_request_word, rx_word)) {
		printk_pmdbg(priv->dev, "outstanding request 0x%02x answered: raw_tag=0x%02x payload=0x%06x\n",
			 tag, raw_tag, payload);
		priv->have_outstanding_request = false;
		priv->consecutive_failures = 0;
		priv->recover_failures = 0;
		if (priv->sync_active) {
			priv->sync_result = 0;
			complete(&priv->sync_done);
		}
	}

	/*
	 * Dummy frame. If we had a word staged, it rode this frame (ACK was
	 * low when the AVR started it), so don't stage it again: that is what
	 * produced duplicate commands and duplicate replies.
	 */
	if (tag == 0x00) {
		priv->stats.tag0_frames++;
		if (priv->have_outstanding_request && was_pending)
			pwrmicro_send_ack(priv, false);
		else
			mc3190_ack_or_send(priv);
		return;
	}

	if (is_retry && is_same_as_last) {
		priv->stats.retry_frames++;
		mc3190_ack_or_send(priv);
		return;
	}

	priv->have_last_cmd = true;
	priv->last_cmd_tag = tag;
	priv->last_cmd_payload = payload;

	switch (tag) {
	case MC3190_TAG_TOUCH_DATA: {
		u16 y = rx_word & 0xFFF;
		u16 x = (((rx_word >> 0xc) & 0xff0) | (rx_word & 0xf000)) >> 4;


		printk_pmdbg(priv->dev, "touch data: raw=0x%08x x=%u y=%u\n", rx_word, x, y);
#ifdef CONFIG_TOUCHSCREEN_MC3190
		if (priv->touch_dev)
			mc3190_touch_report(priv->touch_dev, x, y, PWRMICRO_TOUCH_DOWN);
#endif // CONFIG_TOUCHSCREEN_MC3190

		priv->last_touch_jiffies = jiffies;
		priv->in_state_machine = true;
		priv->state_machine = PWRMICRO_STATE_TOUCH;
		pwrmicro_send_ack(priv, false);
		break;
	}

	case MC3190_TAG_BATT_CAP_EVENT:
		if (!priv->ready_for_gated_tags) {
			printk_pmdbg(priv->dev, "tag 0x07 received but not ready yet - ignoring\n");
			pwrmicro_send_ack(priv, false);
			break;
		}

		pwrmicro_reset_always_needs_updates(priv);
		printk_pmdbg(priv->dev, "Battery Capacity Event (rx_word=0x%08x)\n", rx_word);
		priv->in_state_machine = true;
		priv->state_machine = PWRMICRO_STATE_BATTERY;

		mc3190_store_register_data(priv, tag, payload);
#ifdef CONFIG_MC3190_BATTERY
		if (priv->battery_psy)
			power_supply_changed(priv->battery_psy);
#endif // CONFIG_MC3190_BATTERY

		mc3190_chain(priv, tag);
		mc3190_ack_or_send(priv);
		break;


	case MC3190_TAG_PWR_EVENT_TBC:
	case MC3190_TAG_PWR_EVENT:
		printk_pmdbg(priv->dev, "Power Event (rx_word=0x%08x)\n", rx_word);

		mc3190_store_register_data(priv, tag, payload);
#ifdef CONFIG_MC3190_BATTERY
		if (priv->battery_psy)
			power_supply_changed(priv->battery_psy);
#endif // CONFIG_MC3190_BATTERY

		mc3190_chain(priv, tag);
		mc3190_ack_or_send(priv);
		break;

	case MC3190_TAG_VER:
		printk_pmdbg(priv->dev, "Version Event (rx_word=0x%08x)\n", rx_word);

		priv->versionMajor = (payload >> 16);
		priv->versionMinor = (payload >> 8);

		mc3190_store_register_data(priv, tag, payload);
		mc3190_chain(priv, tag);
		mc3190_ack_or_send(priv);
		break;

	case MC3190_TAG_USERVER:
		printk_pmdbg(priv->dev, "User Version Event (rx_word=0x%08x)\n", rx_word);

		priv->versionUser = (payload >> 8);

		if (priv->versionMajor != 0 && priv->versionMinor != 0) {
			dev_info(priv->dev, "PwrMicro Firmware Version: %d.%d.%d\n", priv->versionMajor, priv->versionMinor, priv->versionUser);
		}

		mc3190_store_register_data(priv, tag, payload);
		mc3190_chain(priv, tag);
		mc3190_ack_or_send(priv);
		break;

	case MC3190_TAG_DRIVER_ID:
		/* Reply to our 0x0B010000: the AVR's TX gate is open */
		printk_pmdbg(priv->dev, "DriverID reply (rx_word=0x%08x)\n", rx_word);
		mc3190_ack_or_send(priv);
		break;

	case MC3190_TAG_SYS_ACK:
		printk_pmdbg(priv->dev, "ACK Tag (rx_word=0x%08x)\n", rx_word);
		if (priv->in_state_machine) {
			switch (priv->state_machine) {
			case PWRMICRO_STATE_TOUCH:
#ifdef CONFIG_TOUCHSCREEN_MC3190
				if (priv->touch_dev)
					mc3190_touch_report(priv->touch_dev, 0, 0, PWRMICRO_TOUCH_UP);
#endif // CONFIG_TOUCHSCREEN_MC3190
				break;
			default:
				break;
			}
			priv->in_state_machine = false;
			priv->state_machine = 0;
		}
		mc3190_ack_or_send(priv);
		break;

	case MC3190_TAG_CMD_ERR:
		priv->stats.cmd_errors++;
		if (printk_ratelimit())
			dev_err(priv->dev, "PwrMicro returned error 0x%08x for last tx (0x%08x)\n", rx_word, priv->last_sent_word);
		/*
		 * Don't keep re-sending a request the AVR rejects (that was a
		 * livelock) and always ACK so the AVR isn't left waiting.
		 */
		if (priv->have_outstanding_request)
			mc3190_req_abandon_locked(priv, -EIO);
		mc3190_ack_or_send(priv);
		break;

	default:
		if (printk_ratelimit())
			dev_warn(priv->dev, "Unhandled AVR Tag: 0x%02x (rx_word=0x%08x)\n", tag, rx_word);
		pwrmicro_send_ack(priv, false);
		break;
	}
}

static void mc3190_dispatch(struct mc3190_pwrmicro *priv, u32 rx_word)
{
	mutex_lock(&priv->proto_lock);
	mc3190_dispatch_locked(priv, rx_word);
	mutex_unlock(&priv->proto_lock);
}

/* ------------------------------------------------------------------ */
/* Interrupt handling                                                  */
/* ------------------------------------------------------------------ */

/*
 * Hard IRQ: raise ACK, drain the whole RX FIFO into the ring and clear the
 * sticky error flags. Dispatch happens in the (realtime) IRQ thread, not
 * keventd, and no word can be overwritten any more.
 */
static irqreturn_t mc3190_ssp4_irq(int irq, void *dev_id)
{
	struct mc3190_pwrmicro *priv = dev_id;
	unsigned long flags;
	u32 status, clr;
	int n = 0;
	bool wake = false;

	status = readl(priv->regs + SSSR);
	if (!(status & (SSSR_RNE | SSSR_ROR | SSSR_BCE | SSSR_TUR)))
		return IRQ_NONE;

	pwrmicro_ack_set(priv, 1);	/* Bring ACK GPIO high after receiving any packets */

	spin_lock_irqsave(&priv->lock, flags);
	while ((readl(priv->regs + SSSR) & SSSR_RNE) && n++ < 32) {
		u32 w = readl(priv->regs + SSDR);
		unsigned int next = (priv->rxq_head + 1) % MC3190_RXQ_SIZE;

		if (next == priv->rxq_tail) {
			priv->stats.rx_dropped++;
		} else {
			priv->rxq[priv->rxq_head] = w;
			priv->rxq_head = next;
			wake = true;
		}
	}
	status |= readl(priv->regs + SSSR);
	clr = status & (SSSR_ROR | SSSR_BCE | SSSR_TUR);
	if (clr)
		writel(clr, priv->regs + SSSR);
	spin_unlock_irqrestore(&priv->lock, flags);

	if (clr & SSSR_TUR)
		priv->stats.tur++;
	if (clr & SSSR_ROR) {
		priv->stats.ror++;
		priv->ssp_error = true;
	}
	if (clr & SSSR_BCE) {
		priv->stats.bce++;
		priv->ssp_error = true;
	}
	if ((clr & (SSSR_ROR | SSSR_BCE)) && printk_ratelimit())
		dev_warn(priv->dev, "SSP error flags 0x%08x (cleared)\n", clr);

	return wake ? IRQ_WAKE_THREAD : IRQ_HANDLED;
}

static irqreturn_t mc3190_ssp4_irq_thread(int irq, void *dev_id)
{
	struct mc3190_pwrmicro *priv = dev_id;
	unsigned long flags;
	u32 word = 0;
	bool have;

	for (;;) {
		spin_lock_irqsave(&priv->lock, flags);
		have = priv->rxq_tail != priv->rxq_head;
		if (have) {
			word = priv->rxq[priv->rxq_tail];
			priv->rxq_tail = (priv->rxq_tail + 1) % MC3190_RXQ_SIZE;
		}
		spin_unlock_irqrestore(&priv->lock, flags);

		if (!have)
			break;
		mc3190_dispatch(priv, word);
	}
	return IRQ_HANDLED;
}

/* ------------------------------------------------------------------ */
/* Synchronous requests, handshake, recovery                           */
/* ------------------------------------------------------------------ */

/* Send a request from process context and wait for its reply (WinCE SpiGet) */
static int mc3190_sync_request(struct mc3190_pwrmicro *priv, u32 word,
			       int attempts, unsigned int timeout_ms)
{
	int i, ret = -ETIMEDOUT;

	for (i = 0; i < attempts; i++) {
		mutex_lock(&priv->proto_lock);
		INIT_COMPLETION(priv->sync_done);
		priv->sync_active = true;
		priv->sync_result = -ETIMEDOUT;
		mc3190_req_begin_locked(priv, word);
		mc3190_send_outstanding(priv);
		mutex_unlock(&priv->proto_lock);

		if (wait_for_completion_timeout(&priv->sync_done,
						msecs_to_jiffies(timeout_ms)))
			ret = priv->sync_result;
		else
			ret = -ETIMEDOUT;

		mutex_lock(&priv->proto_lock);
		priv->sync_active = false;
		if (ret == -ETIMEDOUT)
			priv->have_outstanding_request = false;
		mutex_unlock(&priv->proto_lock);

		if (ret == 0)
			break;
		msleep(100);	/* WinCE SpiGet sleeps 100 ms between attempts */
	}
	return ret;
}

/* WinCE SysPwrReadyThread: version, DriverID, power event */
static int mc3190_handshake(struct mc3190_pwrmicro *priv)
{
	int ver, id;

	ver = mc3190_sync_request(priv, (u32)MC3190_TAG_VER << 24, 3, 300);
	/* Old firmware doesn't reply to DriverID, so a timeout here isn't fatal */
	id = mc3190_sync_request(priv, MC3190_DRIVER_ID_WORD, 2, 300);
	if (ver && id) {
		priv->stats.handshake_fail++;
		dev_warn(priv->dev, "handshake failed (version %d, DriverID %d)\n", ver, id);
		return -ETIMEDOUT;
	}
	mc3190_sync_request(priv, (u32)MC3190_TAG_PWR_EVENT << 24, 2, 300);

	priv->stats.handshake_ok++;
	dev_info(priv->dev, "handshake done (version %s, DriverID %s)\n",
		 ver ? "no reply" : "ok", id ? "no reply" : "ok");
	return 0;
}

/*
 * WinCE EnableTouch(): command 0x01 with enable (0x80), channel mask 0xF,
 * min touch points 3 (SPIMinTouchPoints default) and average size 0xF
 * (SPITouchDataAvgSize default) in byte 1, then 0x0E10 in the low 16 bits.
 * The AVR does not reply to it, and an AVR reset clears the enable flag, so
 * it has to be (re)sent after every bring-up. It's idempotent, so send it
 * twice in case the first rides a frame the AVR discards.
 */
#define MC3190_TOUCH_ENABLE_WORD	0x01BF0E10

static void mc3190_enable_touch(struct mc3190_pwrmicro *priv)
{
	int i;

	for (i = 0; i < 2; i++) {
		mutex_lock(&priv->proto_lock);
		if (!mc3190_stage_word(priv, MC3190_TOUCH_ENABLE_WORD)) {
			priv->sent_pending_frame = true;
			pwrmicro_send_ack(priv, true);
		}
		mutex_unlock(&priv->proto_lock);
		msleep(50);
	}
	dev_info(priv->dev, "touch enable sent (0x%08x)\n", MC3190_TOUCH_ENABLE_WORD);
}

/*
 * Called with recover_lock held, SSP freshly resynced and the ready delay
 * elapsed.
 */
static int mc3190_bringup(struct mc3190_pwrmicro *priv)
{
	int ret = 0;

	mutex_lock(&priv->proto_lock);
	priv->ready_for_gated_tags = true;
	priv->handshaking = pm_handshake ? true : false;
	mutex_unlock(&priv->proto_lock);

	if (pm_handshake)
		ret = mc3190_handshake(priv);

	mc3190_enable_touch(priv);

	mutex_lock(&priv->proto_lock);
	priv->handshaking = false;
	if (!ret) {
		/* Start the battery refresh chain, as a 0x07 event would */
		pwrmicro_reset_always_needs_updates(priv);
		if (!priv->have_outstanding_request) {
			mc3190_request_information(priv, MC3190_TAG_PWR_EVENT);
			if (priv->have_outstanding_request)
				mc3190_send_outstanding(priv);
		}
	}
	mutex_unlock(&priv->proto_lock);
	return ret;
}

static void mc3190_avr_reset(struct mc3190_pwrmicro *priv)
{
	int active = reset_active_high ? 1 : 0;

	dev_warn(priv->dev, "pulsing AVR reset (GPIO%d, active %s)\n",
		 PWRMICRO_RESET_GPIO, active ? "high" : "low");
	gpio_set_value(PWRMICRO_RESET_GPIO, active);
	msleep(reset_pulse_ms);
	gpio_set_value(PWRMICRO_RESET_GPIO, !active);
	msleep(avr_boot_ms);
	priv->stats.avr_resets++;
}

/* recover_lock held */
static void mc3190_recover_locked(struct mc3190_pwrmicro *priv, const char *why, bool avr_reset)
{
	unsigned int backoff_ms;
	int ret;

	dev_warn(priv->dev, "recovering PwrMicro link (%s)\n", why);
	priv->stats.recoveries++;

	if (reset_after_failures > 0 &&
	    priv->recover_failures >= (unsigned int)reset_after_failures) {
		avr_reset = true;
		priv->recover_failures = 0;
	}

	mutex_lock(&priv->proto_lock);
	priv->ready_for_gated_tags = false;
	priv->handshaking = true;
	priv->have_outstanding_request = false;
	priv->have_last_cmd = false;
	priv->sent_pending_frame = false;
	priv->consecutive_failures = 0;
	mc3190_release_touch_locked(priv);
	mutex_unlock(&priv->proto_lock);

	if (avr_reset)
		mc3190_avr_reset(priv);

	mc3190_ssp_resync(priv);
	msleep(MC3190_READY_DELAY_MS);

	ret = mc3190_bringup(priv);
	if (ret) {
		priv->recover_failures++;
		priv->link_down = true;
		backoff_ms = 1000U << min(priv->recover_failures, 5U);
		if (backoff_ms > MC3190_BACKOFF_MAX_MS)
			backoff_ms = MC3190_BACKOFF_MAX_MS;
		priv->next_recover = jiffies + msecs_to_jiffies(backoff_ms);
		dev_warn(priv->dev, "recovery failed (%u in a row), retrying in %u ms\n",
			 priv->recover_failures, backoff_ms);
	} else {
		priv->recover_failures = 0;
		priv->link_down = false;
		priv->last_frame_jiffies = jiffies;
		dev_info(priv->dev, "PwrMicro link recovered\n");
	}
}

/*
 * Liveness check: a version read always gets a reply, even with the AVR's TX
 * gate closed. If it answers we also resend DriverID, which reopens the gate
 * if something (PC5/PD2 glitch, SPI re-enable) closed it. recover_lock held.
 */
static int mc3190_ping(struct mc3190_pwrmicro *priv)
{
	int ret;

	priv->stats.pings++;
	ret = mc3190_sync_request(priv, (u32)MC3190_TAG_VER << 24, 3, 300);
	if (ret) {
		priv->stats.ping_fail++;
		return ret;
	}
	mc3190_sync_request(priv, MC3190_DRIVER_ID_WORD, 1, 300);
	return 0;
}

/* ------------------------------------------------------------------ */
/* Work items                                                          */
/* ------------------------------------------------------------------ */

static void mc3190_ready_work_fn(struct work_struct *work)
{
	struct mc3190_pwrmicro *priv = container_of(to_delayed_work(work),
						     struct mc3190_pwrmicro, ready_work);
	int ret;

	mutex_lock(&priv->recover_lock);
	ret = mc3190_bringup(priv);
	if (ret) {
		priv->recover_failures++;
		priv->link_down = true;
		priv->next_recover = jiffies + msecs_to_jiffies(2000);
	} else {
		priv->last_frame_jiffies = jiffies;
	}
	mutex_unlock(&priv->recover_lock);
}

static void mc3190_watch_work_fn(struct work_struct *work)
{
	struct mc3190_pwrmicro *priv = container_of(to_delayed_work(work),
						     struct mc3190_pwrmicro, watch_work);
	const char *why = NULL;

	if (!mutex_trylock(&priv->recover_lock))
		goto rearm;	/* a recovery or manual command is running */

	mutex_lock(&priv->proto_lock);

	/* 1. a lost pen-up must not leave the touchscreen pressed forever */
	if (priv->in_state_machine && priv->state_machine == PWRMICRO_STATE_TOUCH &&
	    time_after(jiffies, priv->last_touch_jiffies + msecs_to_jiffies(MC3190_TOUCH_STUCK_MS))) {
		priv->stats.stuck_touch++;
		mc3190_release_touch_locked(priv);
	}

	/* 2. request timeouts: re-send, then give up so the chain can't wedge */
	if (priv->have_outstanding_request && !priv->sync_active &&
	    time_after(jiffies, priv->outstanding_deadline)) {
		if (priv->outstanding_retries < MC3190_REQ_MAX_RETRIES) {
			priv->outstanding_retries++;
			priv->outstanding_stages = 0;
			priv->stats.req_retries++;
			mc3190_send_outstanding(priv);
		} else {
			dev_dbg(priv->dev, "request 0x%08x abandoned\n", priv->outstanding_request_word);
			priv->stats.req_abandoned++;
			priv->consecutive_failures++;
			mc3190_req_abandon_locked(priv, -ETIMEDOUT);
			if (priv->have_outstanding_request)
				mc3190_send_outstanding(priv);
		}
	}
	if (priv->consecutive_failures >= MC3190_FAILS_BEFORE_RECOVER)
		why = "repeated request timeouts";
	mutex_unlock(&priv->proto_lock);

	/* 3. SSP error flags seen by the IRQ handler: make sure we can still talk */
	if (!why && priv->ssp_error) {
		priv->ssp_error = false;
		if (mc3190_ping(priv))
			why = "SSP error and no reply";
	}

	/* 4. nothing at all from the AVR for a long time */
	if (!why && !priv->link_down &&
	    time_after(jiffies, priv->last_frame_jiffies + msecs_to_jiffies(MC3190_SILENCE_MS))) {
		if (mc3190_ping(priv))
			why = "AVR silent and not answering";
		else
			priv->last_frame_jiffies = jiffies;
	}

	/* 5. a previous recovery failed: retry with backoff */
	if (!why && priv->link_down && time_after(jiffies, priv->next_recover))
		why = "retrying after failed recovery";

	if (why)
		mc3190_recover_locked(priv, why, false);

	mutex_unlock(&priv->recover_lock);

rearm:
	if (!priv->shutting_down)
		queue_delayed_work(priv->wq, &priv->watch_work,
				   msecs_to_jiffies(MC3190_WATCH_PERIOD_MS));
}

static void mc3190_cmd_work_fn(struct work_struct *work)
{
	struct mc3190_pwrmicro *priv = container_of(work, struct mc3190_pwrmicro, cmd_work);
	bool avr = test_and_clear_bit(MC3190_CMD_AVR_RESET, &priv->cmd_flags);
	bool rec = test_and_clear_bit(MC3190_CMD_RECOVER, &priv->cmd_flags);

	if (!avr && !rec)
		return;

	mutex_lock(&priv->recover_lock);
	mc3190_recover_locked(priv, avr ? "manual AVR reset" : "manual request", avr);
	mutex_unlock(&priv->recover_lock);
}

/* ------------------------------------------------------------------ */
/* sysfs                                                               */
/* ------------------------------------------------------------------ */

static ssize_t pm_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mc3190_pwrmicro *priv = dev_get_drvdata(dev);
	struct mc3190_pm_stats *s = &priv->stats;
	u32 sssr = readl(priv->regs + SSSR);

	return snprintf(buf, PAGE_SIZE,
		"sscr0=0x%08x sscr1=0x%08x sssr=0x%08x ack_gpio=%d irq=%d\n"
		"link_down=%d ready=%d outstanding=%d (0x%08x, retries %u) consecutive_failures=%u\n"
		"last_frame_age_ms=%u version=%u.%u.%u\n"
		"frames=%u tag0=%u retry=%u rx_dropped=%u ror=%u bce=%u tur=%u\n"
		"resyncs=%u recoveries=%u handshake_ok=%u handshake_fail=%u avr_resets=%u\n"
		"req_sent=%u req_retries=%u req_abandoned=%u cmd_errors=%u bad_subtag=%u\n"
		"pings=%u ping_fail=%u stage_fail=%u stuck_touch=%u\n",
		readl(priv->regs + SSCR0), readl(priv->regs + SSCR1), sssr,
		gpio_get_value(PWRMICRO_ACK_OUT_GPIO), priv->irq,
		priv->link_down, priv->ready_for_gated_tags,
		priv->have_outstanding_request, priv->outstanding_request_word,
		priv->outstanding_retries, priv->consecutive_failures,
		jiffies_to_msecs(jiffies - priv->last_frame_jiffies),
		priv->versionMajor, priv->versionMinor, priv->versionUser,
		s->frames, s->tag0_frames, s->retry_frames, s->rx_dropped, s->ror, s->bce, s->tur,
		s->resyncs, s->recoveries, s->handshake_ok, s->handshake_fail, s->avr_resets,
		s->req_sent, s->req_retries, s->req_abandoned, s->cmd_errors, s->bad_subtag,
		s->pings, s->ping_fail, s->stage_fail, s->stuck_touch);
}
static DEVICE_ATTR(pm_status, 0444, pm_status_show, NULL);

/* echo 1 > pm_recover : SSP resync + handshake (recovery test from userspace) */
static ssize_t pm_recover_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct mc3190_pwrmicro *priv = dev_get_drvdata(dev);

	set_bit(MC3190_CMD_RECOVER, &priv->cmd_flags);
	queue_work(priv->wq, &priv->cmd_work);
	return count;
}
static DEVICE_ATTR(pm_recover, 0200, NULL, pm_recover_store);

/* echo 1 > pm_avr_reset : pulse GPIO16, then resync + handshake */
static ssize_t pm_avr_reset_store(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct mc3190_pwrmicro *priv = dev_get_drvdata(dev);

	set_bit(MC3190_CMD_AVR_RESET, &priv->cmd_flags);
	queue_work(priv->wq, &priv->cmd_work);
	return count;
}
static DEVICE_ATTR(pm_avr_reset, 0200, NULL, pm_avr_reset_store);

void mc3190_set_touch_dev(struct mc3190_pwrmicro *priv, struct mc3190_touch *touch)
{
	/* proto_lock: the IRQ thread uses touch_dev under it */
	mutex_lock(&priv->proto_lock);
	priv->touch_dev = touch;
	mutex_unlock(&priv->proto_lock);
}
EXPORT_SYMBOL_GPL(mc3190_set_touch_dev);

/* ------------------------------------------------------------------ */
/* Platform driver                                                     */
/* ------------------------------------------------------------------ */

static int mc3190_pwrmicro_probe(struct platform_device *pdev)
{
	struct mc3190_pwrmicro *priv;
	struct mfd_cell mc3190_cells[] = {
		{	.name = "mc3190-touch", },
		{	.name = "mc3190-battery", },
	};

	struct resource *res;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	priv->dev = &pdev->dev;
	spin_lock_init(&priv->lock);
	spin_lock_init(&priv->ack_lock);
	spin_lock_init(&priv->soc_lock);
	mutex_init(&priv->proto_lock);
	mutex_init(&priv->recover_lock);
	init_completion(&priv->sync_done);
	INIT_DELAYED_WORK(&priv->ready_work, mc3190_ready_work_fn);
	INIT_DELAYED_WORK(&priv->watch_work, mc3190_watch_work_fn);
	INIT_WORK(&priv->cmd_work, mc3190_cmd_work_fn);
	priv->last_frame_jiffies = jiffies;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	priv->regs = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!priv->regs)
		return -ENOMEM;

	priv->irq = platform_get_irq(pdev, 0);
	if (priv->irq < 0)
		return priv->irq;

	/* ACK idles high (WinCE SPI_Init) */
	ret = gpio_request_one(PWRMICRO_ACK_OUT_GPIO, GPIOF_OUT_INIT_HIGH,
				"pwrmicro-ack-out");
	if (ret) {
		dev_err(&pdev->dev, "failed to request ACK_OUT GPIO%d: %d\n",
			PWRMICRO_ACK_OUT_GPIO, ret);
		return ret;
	}

	/*
	 * GPIO16: WinCE drives it low once in SPI_Init and never touches it
	 * again. Until now this driver left it however the bootloader did.
	 */
	ret = gpio_request_one(PWRMICRO_RESET_GPIO,
			       reset_active_high ? GPIOF_OUT_INIT_LOW : GPIOF_OUT_INIT_HIGH,
			       "pwrmicro-reset");
	if (ret) {
		dev_err(&pdev->dev, "failed to request RESET GPIO%d: %d\n",
			PWRMICRO_RESET_GPIO, ret);
		goto err_ack;
	}

	priv->wq = create_singlethread_workqueue("mc3190-pm");
	if (!priv->wq) {
		ret = -ENOMEM;
		goto err_reset;
	}

	pwrmicro_init_tags(priv);
	platform_set_drvdata(pdev, priv);

	writel(0, priv->regs + SSCR0);	/* SSP off, RIE off until the IRQ is in place */

	ret = request_threaded_irq(priv->irq, mc3190_ssp4_irq, mc3190_ssp4_irq_thread,
				   0, DRV_NAME, priv);
	if (ret) {
		dev_err(&pdev->dev, "failed to request IRQ%d: %d\n", priv->irq, ret);
		goto err_wq;
	}

	/* Same configuration WinCE uses, leaves SSDR=0 staged and ACK low */
	mc3190_ssp_resync(priv);

	if (device_create_file(&pdev->dev, &dev_attr_pm_status) ||
	    device_create_file(&pdev->dev, &dev_attr_pm_recover) ||
	    device_create_file(&pdev->dev, &dev_attr_pm_avr_reset))
		dev_warn(&pdev->dev, "failed to create sysfs attributes\n");

	/* After the ready delay: handshake, then start the battery chain */
	queue_delayed_work(priv->wq, &priv->ready_work, msecs_to_jiffies(MC3190_READY_DELAY_MS));
	if (pm_watchdog)
		queue_delayed_work(priv->wq, &priv->watch_work,
				   msecs_to_jiffies(MC3190_READY_DELAY_MS + 2000));

	dev_info(&pdev->dev, "MC3190 PwrMicro driver active (IRQ%d)\n", priv->irq); // FIXME: get PM version from device

	ret = mfd_add_devices(&pdev->dev, -1, mc3190_cells, ARRAY_SIZE(mc3190_cells), NULL, 0);
	if (ret) {
		dev_err(&pdev->dev, "failed to add MFD devices: %d\n", ret);
		goto err_work;
	}

	return 0;

err_work:
	priv->shutting_down = true;
	cancel_delayed_work_sync(&priv->watch_work);
	cancel_delayed_work_sync(&priv->ready_work);
	cancel_work_sync(&priv->cmd_work);
	device_remove_file(&pdev->dev, &dev_attr_pm_status);
	device_remove_file(&pdev->dev, &dev_attr_pm_recover);
	device_remove_file(&pdev->dev, &dev_attr_pm_avr_reset);
	writel(0, priv->regs + SSCR0);
	free_irq(priv->irq, priv);
err_wq:
	destroy_workqueue(priv->wq);
err_reset:
	gpio_free(PWRMICRO_RESET_GPIO);
err_ack:
	gpio_free(PWRMICRO_ACK_OUT_GPIO);
	return ret;
}

static int mc3190_pwrmicro_remove(struct platform_device *pdev)
{
	struct mc3190_pwrmicro *priv = platform_get_drvdata(pdev);

	mfd_remove_devices(&pdev->dev);

	priv->shutting_down = true;
	cancel_delayed_work_sync(&priv->watch_work);
	cancel_delayed_work_sync(&priv->ready_work);
	cancel_work_sync(&priv->cmd_work);

	device_remove_file(&pdev->dev, &dev_attr_pm_status);
	device_remove_file(&pdev->dev, &dev_attr_pm_recover);
	device_remove_file(&pdev->dev, &dev_attr_pm_avr_reset);

	writel(0, priv->regs + SSCR0);
	free_irq(priv->irq, priv);
	destroy_workqueue(priv->wq);

	gpio_free(PWRMICRO_RESET_GPIO);
	gpio_free(PWRMICRO_ACK_OUT_GPIO);

	return 0;
}

/*
 * WinCE redoes the whole bring-up on every resume (SPI_PowerUp ->
 * PowerOnEvent -> PwrOnHndlr / SysPwrReadyThread); the SSP state and the
 * AVR's TX gate can't be assumed to survive.
 */
static int mc3190_pwrmicro_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct mc3190_pwrmicro *priv = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&priv->watch_work);
	mutex_lock(&priv->recover_lock);
	mutex_lock(&priv->proto_lock);
	priv->ready_for_gated_tags = false;
	priv->have_outstanding_request = false;
	mutex_unlock(&priv->proto_lock);
	mutex_unlock(&priv->recover_lock);
	return 0;
}

static int mc3190_pwrmicro_resume(struct platform_device *pdev)
{
	struct mc3190_pwrmicro *priv = platform_get_drvdata(pdev);

	set_bit(MC3190_CMD_RECOVER, &priv->cmd_flags);
	queue_work(priv->wq, &priv->cmd_work);
	if (pm_watchdog)
		queue_delayed_work(priv->wq, &priv->watch_work,
				   msecs_to_jiffies(MC3190_WATCH_PERIOD_MS));
	return 0;
}

static struct platform_driver mc3190_pwrmicro_driver = {
	.driver = {
		.name  = DRV_NAME,
		.owner = THIS_MODULE,
	},
	.probe  = mc3190_pwrmicro_probe,
	.remove = mc3190_pwrmicro_remove,
	.suspend = mc3190_pwrmicro_suspend,
	.resume  = mc3190_pwrmicro_resume,
};

static int __init mc3190_pwrmicro_init(void)
{
	return platform_driver_register(&mc3190_pwrmicro_driver);
}
module_init(mc3190_pwrmicro_init);

static void __exit mc3190_pwrmicro_exit(void)
{
	platform_driver_unregister(&mc3190_pwrmicro_driver);
}
module_exit(mc3190_pwrmicro_exit);

MODULE_AUTHOR("Mark Kennard <markkennard4@gmail.com>");
MODULE_DESCRIPTION("MFD Driver for MC3190 AVR PwrMicro");
MODULE_LICENSE("GPL");