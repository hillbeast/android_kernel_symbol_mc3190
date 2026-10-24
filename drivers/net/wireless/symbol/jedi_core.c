/*
 * jedi_core.c -- Driver for Symbol "Jedi" WiFi module in MC3190 (transport, netdev, probe)
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

#include <linux/mmc/host.h>
#include "jedi_priv.h"

#define JEDI_VENDOR_ID  0x0013
#define JEDI_DEVICE_ID  0xc684

bool jedi_debug;
module_param_named(debug, jedi_debug, bool, 0644);
MODULE_PARM_DESC(debug, "Enable verbose Jedi driver logging (default off)");

bool jedi_reg_test;
module_param_named(reg_test, jedi_reg_test, bool, 0444);
MODULE_PARM_DESC(reg_test, "Experiment: send a guessed 2.4 GHz regulatory row (R15)");

char *jedi_country;
module_param_named(country, jedi_country, charp, 0444);

/* R18.7: after init, flip the band mask and restart the MLME to force a rebuild. */
bool jedi_rebuild;
module_param_named(rebuild, jedi_rebuild, bool, 0444);
MODULE_PARM_DESC(rebuild, "Re-send band mask + empty SSID after init (R18.7)");

/* R18.6: look for firmware debug text via HOST_INT 0x8000/0x4000 + reg 0x50. */
/*
 * Timing experiment: debug=1 logs every frame to the (slow) serial console,
 * which spaces commands out by tens of ms. debug=0 sends them ~1 ms apart.
 * The default of 20 ms gives debug=0 the same spacing; jedi.cmd_delay=0 removes it.
 */
unsigned int jedi_cmd_delay = 20;
module_param_named(cmd_delay, jedi_cmd_delay, uint, 0644);
MODULE_PARM_DESC(cmd_delay, "Extra ms to wait after every command (default 20, 0 = none)");

/*
 * Experiment knobs for the association work. All writable at run time under
 * /sys/module/jedi/parameters, so ideas from RE can be tried without a
 * rebuild. Defaults reproduce the current behaviour.
 */
unsigned int jedi_suite_override;	/* 0 = derive from SIOCSIWAUTH */
module_param_named(suite, jedi_suite_override, uint, 0644);
MODULE_PARM_DESC(suite, "Force the 0x1200000b suite code (0 = automatic)");

unsigned int jedi_join_mode = 3;
module_param_named(join_mode, jedi_join_mode, uint, 0644);
MODULE_PARM_DESC(join_mode, "3 = WinCE sequence (default), 0 = BSSID then SSID, 1 = SSID only, 2 = SSID then BSSID");
bool jedi_dot11d = true;
module_param_named(dot11d, jedi_dot11d, bool, 0644);
MODULE_PARM_DESC(dot11d, "802.11d: let the firmware adopt the country seen in beacons (default on, as WinCE). 0 keeps jedi.country fixed");
unsigned int jedi_join_settle_ms = 500;
module_param_named(join_settle_ms, jedi_join_settle_ms, uint, 0644);
MODULE_PARM_DESC(join_settle_ms, "join_mode 3: pause after the disconnect SSID and after the regulatory refresh (WinCE: ~3300; 0 and 250 fail, 500 works; default 500)");

bool jedi_send_privacy;
module_param_named(privacy, jedi_send_privacy, bool, 0644);
MODULE_PARM_DESC(privacy, "Send 0x12000001/0x12000002 with the security suite (RE Q4)");

unsigned int jedi_data_trace = 32;
module_param_named(data_trace, jedi_data_trace, uint, 0644);
MODULE_PARM_DESC(data_trace, "Log the first N data frames in each direction (default 32, 0 = off)");
bool jedi_power_ctl;
module_param_named(power_ctl, jedi_power_ctl, bool, 0644);
MODULE_PARM_DESC(power_ctl, "Power the module off when jedi.ko is unloaded (Wi-Fi off) and on again when it is loaded (default off; needs the board hooks)");
unsigned int jedi_recover_max = 3;
module_param_named(recover, jedi_recover_max, uint, 0644);
MODULE_PARM_DESC(recover, "Board-level power cycles to try when the card stops answering (0 = off, default 3)");
bool jedi_ps_kick;
module_param_named(ps_kick, jedi_ps_kick, bool, 0644);
MODULE_PARM_DESC(ps_kick, "Write 0x38=2 after each TX like WinCE (lets the chip sleep; a TX to a sleeping chip is refused with -EIO). Default off");
bool jedi_link_fallback;
module_param_named(link_fallback, jedi_link_fallback, bool, 0644);
MODULE_PARM_DESC(link_fallback, "Treat MLME state 5 as link up if no link trap arrives (default off)");
unsigned int jedi_ptk_delay_ms = 30;
module_param_named(ptk_delay_ms, jedi_ptk_delay_ms, uint, 0644);
MODULE_PARM_DESC(ptk_delay_ms, "Wait this long after EAPOL message 4 before installing the pairwise key");
int jedi_wmm_mode = -1;
module_param_named(wmm, jedi_wmm_mode, int, 0644);
MODULE_PARM_DESC(wmm, "0x13000108: -1 follow the AP's WMM IE (default), 0 or 1 force");
bool jedi_send_suite = true;
module_param_named(send_suite, jedi_send_suite, bool, 0644);
MODULE_PARM_DESC(send_suite, "Send 0x1200000b (suite) for secured joins (WinCE did not in one capture)");
bool jedi_dummy_keys;
module_param_named(dummy_keys, jedi_dummy_keys, bool, 0644);
MODULE_PARM_DESC(dummy_keys, "Send WinCE's four type-1 0x12000008 broadcast records before each join");
unsigned int jedi_key_remove_mode = 1;
module_param_named(key_remove, jedi_key_remove_mode, uint, 0644);
MODULE_PARM_DESC(key_remove, "Key removal: 1 = 0x1200000a record with type byte 8 (default), 0 = overwrite with a zero key");
unsigned int jedi_rsc_mode = 1;
module_param_named(rsc_mode, jedi_rsc_mode, uint, 0644);
MODULE_PARM_DESC(rsc_mode, "Key receive counter: 0 = as supplied (LE), 1 = zero (default), 2 = byte-reversed");
unsigned int jedi_country_wait_ms = 4000;
module_param_named(country_wait, jedi_country_wait_ms, uint, 0644);
MODULE_PARM_DESC(country_wait, "Max ms to wait for the regulatory-change trap (0x1f000003) after a join's commit (0 = fixed delay only)");
bool jedi_key_dump = true;
module_param_named(key_dump, jedi_key_dump, bool, 0644);
MODULE_PARM_DESC(key_dump, "Log the raw 64-byte key records (INCLUDES KEY MATERIAL; test networks only)");
bool jedi_tx_gate = true;
module_param_named(tx_gate, jedi_tx_gate, bool, 0644);
MODULE_PARM_DESC(tx_gate, "Drop non-EAPOL frames until the WPA port is authorised (default on)");
bool jedi_bssid_lock;
module_param_named(bssid_lock, jedi_bssid_lock, bool, 0644);
MODULE_PARM_DESC(bssid_lock, "Send the supplicant's BSSID to the firmware (default: zero = any AP, as WinCE does)");
unsigned int jedi_port_oid = 0x150007e4;
module_param_named(port_oid, jedi_port_oid, uint, 0644);
MODULE_PARM_DESC(port_oid, "802.1X port-authorise OID (WinCE: 0x150007e4 = BSSID after PTK and GTK; 0 = none)");

unsigned int jedi_join_poll_ms = 8000;
module_param_named(join_poll_ms, jedi_join_poll_ms, uint, 0644);
MODULE_PARM_DESC(join_poll_ms, "How long to poll 0x10000003 after a join (default 8000)");

/*
 * First-TX failure investigation (init failed with -EIO on the first data
 * burst after SET 0x10, then the card stopped answering).
 */
unsigned int jedi_tx_gap_us;
module_param_named(tx_gap_us, jedi_tx_gap_us, uint, 0644);
MODULE_PARM_DESC(tx_gap_us, "Delay between writing the TX length and the data burst, us (default 0)");

unsigned int jedi_tx_retry = 3;
module_param_named(tx_retry, jedi_tx_retry, uint, 0644);
MODULE_PARM_DESC(tx_retry, "Retries when the card refuses a data burst (default 3)");

bool jedi_tx_trace = true;
module_param_named(tx_trace, jedi_tx_trace, bool, 0644);
MODULE_PARM_DESC(tx_trace, "Log device registers before the first 12 TX frames (default on)");

unsigned int jedi_init_retries = 3;
module_param_named(init_retries, jedi_init_retries, uint, 0644);
MODULE_PARM_DESC(init_retries, "Chip reset + firmware reload attempts if init fails (default 3)");

bool jedi_uart;
module_param_named(uart, jedi_uart, bool, 0644);
MODULE_PARM_DESC(uart, "Poll HOST_INT and drain firmware UART text (R18.6)");
MODULE_PARM_DESC(country, "Country code (2-4 chars) sent with SET 0x1f000002");


static void jedi_dump_frame(struct sdio_func *func, const char *dir,
			    const u8 *buf, u32 len)
{
	if (!jedi_debug)
		return;
	dev_info(&func->dev, "jedi: %s frame, %u bytes\n", dir, len);
	print_hex_dump(KERN_INFO, "jedi: ", DUMP_PREFIX_OFFSET, 16, 1,
		       buf, min_t(u32, len, 64), false);
}

/*
 * R8.x (WinCE trace findings): power-save handshake that follows most TX
 * frames. Read CCCR 0x05 (function 0, bus power-save/sleep state); if it
 * reads 0 the device is not already awake/ready, so nudge it by writing
 * CTRL (0x38) = 0x02 twice. Our driver was missing this entirely. Best
 * effort only: errors here are not fatal to the TX itself.
 */
static void jedi_tx_powersave_kick(struct jedi_private *priv)
{
	struct sdio_func *func = priv->func;
	int err;
	u8 v;

	v = sdio_f0_readb(func, 0x05, &err);
	if (err || v != 0)
		return;
	jedi_reg_write(func, JEDI_REG_CTRL, 1, 0x02);
	jedi_reg_write(func, JEDI_REG_CTRL, 1, 0x02);
}

static void jedi_rx(struct jedi_private *priv);
static void jedi_tx_snapshot(struct jedi_private *priv, const char *tag, bool err);

/* After port authorise: sample the chip's status registers once a second. */
static void jedi_diag_work(struct work_struct *work)
{
	struct jedi_private *priv = container_of(work, struct jedi_private,
						 diag_work.work);
	struct sdio_func *func = priv->func;
	char tag[64];

	if (priv->diag_n == 4)
		priv->diag_dump = true;
	sdio_claim_host(func);
	snprintf(tag, sizeof(tag), "post-auth #%d irq %lu rx-irq %lu rx-len0 %lu",
		 priv->diag_n, priv->irq_cnt, priv->irq_rx_cnt,
		 priv->rx_zero_cnt);
	jedi_tx_snapshot(priv, tag, false);
	sdio_release_host(func);
	if (++priv->diag_n < 8)
		schedule_delayed_work(&priv->diag_work, HZ);
}

void jedi_diag_start(struct jedi_private *priv)
{
	cancel_delayed_work_sync(&priv->diag_work);
	priv->diag_n = 0;
	priv->diag_dump = true;
	schedule_delayed_work(&priv->diag_work, HZ / 4);
}

/*
 * debug=0 log: "TX burst failed ... -5" is mmc_io_rw_extended returning -EIO,
 * which means the CARD set the error flag in its R5 response (it refused the
 * write), not a bus glitch; every later burst then timed out (-110). The data
 * port is shared with RX, so a pending received frame (a trap, the 1 s
 * statistics trap) is a candidate. Drain RX before every TX, host claimed.
 */
static void jedi_tx_drain_rx(struct jedi_private *priv)
{
	int err, n;
	u8 st;

	for (n = 0; n < 8; n++) {
		st = sdio_readb(priv->func, JEDI_REG_STATUS, &err);
		if (err || !(st & JEDI_STATUS_RX))
			return;
		jedi_rx(priv);
	}
}

/* Snapshot of the device registers; @err selects the log level. */
static void jedi_tx_snapshot(struct jedi_private *priv, const char *tag, bool err)
{
	struct sdio_func *func = priv->func;
	u32 hi = 0, txl = 0, rxl = 0, dev = 0;
	int e1, e2, e3, e4, e5, e6;
	char msg[160];
	u8 st, f0;

	st = sdio_readb(func, JEDI_REG_STATUS, &e1);
	e2 = jedi_reg_read(func, JEDI_REG_HOST_INT, 2, &hi);
	e3 = jedi_reg_read(func, JEDI_REG_TX_LEN, 2, &txl);
	e4 = jedi_reg_read(func, JEDI_REG_RX_LEN, 2, &rxl);
	e5 = jedi_reg_read(func, JEDI_REG_DEVSTAT, 4, &dev);
	f0 = sdio_f0_readb(func, 0x05, &e6);
	snprintf(msg, sizeof(msg),
		 "jedi: %s: status=0x%02x HOST_INT=0x%04x 0x50=0x%04x rx_len=%u "
		 "DEVSTAT=0x%08x F0[5]=0x%02x (errs %d %d %d %d %d %d)",
		 tag, st, hi, txl, rxl, dev, f0, e1, e2, e3, e4, e5, e6);
	if (err)
		dev_err(&func->dev, "%s\n", msg);
	else
		dev_info(&func->dev, "%s\n", msg);
}

/* One attempt at sending a frame. Takes the host itself (R6.2):
 *   1. read 0x50; send only if 0x4000 is set (a failed read counts as ready)
 *   2. write 0x50 = len | 0xc000 (bits 14+15, per the WinCE trace findings)
 *   3. write the frame to the data port in bursts of at most 512 bytes
 *   4. power-save handshake (jedi_tx_powersave_kick)
 */
static int jedi_tx_try(struct jedi_private *priv, const u8 *frame, u32 len,
		       bool trace)
{
	struct sdio_func *func = priv->func;
	u32 sent = 0, v = 0;
	unsigned int i;
	bool body_retried = false;
	int ret = 0;

	sdio_claim_host(func);
	jedi_tx_drain_rx(priv);

	for (i = 0; i < JEDI_TX_READY_TRIES; i++) {
		/* Idle reads 0x4000. A latched length (bit 15 set, e.g. 0xc05c
		 * after a failed burst) means the card still waits for that
		 * frame's body: not ready. */
		if (jedi_reg_read(func, JEDI_REG_TX_LEN, 2, &v) ||
		    (v & (JEDI_TX_READY | 0x8000)) == JEDI_TX_READY)
			break;
		sdio_release_host(func);
		msleep(1);
		sdio_claim_host(func);
		jedi_tx_drain_rx(priv);
	}
	if (i == JEDI_TX_READY_TRIES) {
		dev_err(&func->dev, "jedi: TX not ready (0x50=0x%04x)\n", v);
		ret = -EBUSY;
		goto out;
	}
	if (trace)
		jedi_tx_snapshot(priv, "pre-TX", false);

	ret = jedi_reg_write(func, JEDI_REG_TX_LEN, 2, len | JEDI_TX_LEN_FLAGS);
	if (ret)
		goto out;
	if (jedi_tx_gap_us) {
		if (jedi_tx_gap_us >= 1000)
			msleep(jedi_tx_gap_us / 1000);
		else
			udelay(jedi_tx_gap_us);
	}

	while (sent < len) {
		u32 n = min_t(u32, len - sent, JEDI_BURST);

		memcpy(priv->bounce, frame + sent, n);
		ret = sdio_writesb(func, 0x0, priv->bounce, n);
		if (ret) {
			dev_err(&func->dev,
				"jedi: TX burst failed at %u/%u: %d\n",
				sent, len, ret);
			jedi_tx_snapshot(priv, "after TX failure", true);
			/*
			 * The length register still holds len|0xc000, so the
			 * card is waiting for this frame's body. Send the body
			 * again WITHOUT rewriting the length (a second length
			 * write on top of the first is what wedged the card).
			 */
			if (ret == -EIO && !body_retried) {
				u32 lat = 0;

				body_retried = true;
				msleep(2);
				if (!jedi_reg_read(func, JEDI_REG_TX_LEN, 2, &lat) &&
				    (lat & 0x8000)) {
					dev_warn(&func->dev,
						 "jedi: length still latched (0x%04x), resending body only\n",
						 lat);
					continue;
				}
			}
			goto out;
		}
		sent += n;
	}
	if (jedi_ps_kick)
		jedi_tx_powersave_kick(priv);
out:
	sdio_release_host(func);
	return ret;
}

/* Send one frame. -EIO means the card itself refused the burst; try again
 * after a short wait (the first attempt's state is logged above). */
/*
 * Recovery: a wedged card (CMD53 timeouts, firmware download refused) is not
 * cured by the chip reset in jedi_fw.c; WinCE power-cycles the module through
 * the CPLD. The board code owns that sequence (mc3190_wifi_power_cycle(), a
 * weak symbol so the module still loads on other boards). After the cycle the
 * MMC core is told to look again: its detect step no longer gets an answer
 * from the (reset) card, removes it, powers the slot and enumerates it afresh,
 * which unbinds and re-probes this driver. It runs from the global workqueue
 * because this driver instance is torn down underneath it.
 */
static struct mmc_host *jedi_recover_host;
static unsigned int jedi_recoveries;
static unsigned long jedi_last_recover;

static void jedi_recover_fn(struct work_struct *w)
{
	pr_warn("jedi: recovery: power-cycling the module\n");
	if (mc3190_wifi_power_cycle)
		mc3190_wifi_power_cycle();
	if (jedi_recover_host)
		mmc_detect_change(jedi_recover_host, msecs_to_jiffies(500));
}
static DECLARE_WORK(jedi_recover_work, jedi_recover_fn);

bool jedi_request_recovery(struct sdio_func *func, const char *why)
{
	if (!jedi_recover_max)
		return false;
	if (!mc3190_wifi_power_cycle) {
		dev_err(&func->dev,
			"jedi: %s, but this kernel has no board power-cycle hook\n",
			why);
		return false;
	}
	/* a long quiet spell counts as a fresh start */
	if (time_after(jiffies, jedi_last_recover + 300 * HZ))
		jedi_recoveries = 0;
	if (jedi_recoveries >= jedi_recover_max) {
		dev_err(&func->dev,
			"jedi: %s; %u power cycles already tried, giving up\n",
			why, jedi_recoveries);
		return false;
	}
	jedi_recover_host = func->card->host;
	if (schedule_work(&jedi_recover_work)) {
		jedi_recoveries++;
		jedi_last_recover = jiffies;
		dev_err(&func->dev,
			"jedi: %s: power-cycling the module (%u/%u)\n", why,
			jedi_recoveries, jedi_recover_max);
	}
	return true;
}

static int jedi_tx_frame(struct jedi_private *priv, const u8 *frame, u32 len)
{
	unsigned int attempt;
	bool trace;
	int ret;

	if (!len)
		return -EINVAL;

	jedi_dump_frame(priv->func, "tx", frame, len);

	for (attempt = 0; ; attempt++) {
		trace = jedi_tx_trace && priv->tx_traced < 12;
		if (trace)
			priv->tx_traced++;
		ret = jedi_tx_try(priv, frame, len, trace);
		if (ret != -EIO || attempt >= jedi_tx_retry)
			break;
		dev_warn(&priv->func->dev,
			 "jedi: card refused the TX burst, retry %u/%u\n",
			 attempt + 1, jedi_tx_retry);
		msleep(10 << attempt);
	}
	if (ret == -ETIMEDOUT || ret == -EBUSY) {
		/* the card no longer takes data at all */
		if (++priv->tx_stalls >= 2)
			jedi_request_recovery(priv->func, "TX stalled");
	} else if (!ret) {
		priv->tx_stalls = 0;
	}
	return ret;
}

/*
 * Send a PIMFOR command and wait for the reply matched by OID (R5.1, R5.3).
 * For GET, @plen is the expected reply size and no payload is sent.
 * @reply (optional) receives up to @reply_max bytes; *@reply_len gets the
 * actual size.
 */
int jedi_cmd(struct jedi_private *priv, u8 op, u32 oid,
		    const void *payload, u32 plen,
		    void *reply, u32 reply_max, u32 *reply_len)
{
	struct jedi_pending pend;
	/* WinCE sends a GET with a zeroed buffer of the reply size after the
	 * header (capture: GET 0x1f000002 is 26 + 260 bytes). The firmware
	 * fills the buffer in place, so fields it doesn't write read back as
	 * whatever the buffer held. */
	u32 flen = JEDI_FRAME_HDR + plen;
	u8 *f;
	int ret;

	f = kzalloc(flen, GFP_KERNEL);
	if (!f)
		return -ENOMEM;

	/* WinCE puts the adapter MAC in both address fields (R5.1); they stay
	 * zero only until the MAC has been read (R6.6). */
	if (priv->ndev && !is_zero_ether_addr(priv->ndev->dev_addr)) {
		memcpy(f, priv->ndev->dev_addr, ETH_ALEN);
		memcpy(f + 6, priv->ndev->dev_addr, ETH_ALEN);
	}
	f[12] = JEDI_ETH_P_PIMFOR >> 8;
	f[13] = JEDI_ETH_P_PIMFOR & 0xff;
	f[14] = JEDI_PIMFOR_VERSION;
	f[15] = op;
	put_unaligned_be32(oid, f + 16);
	f[20] = 0;	/* device id */
	f[21] = 0;	/* flags: big-endian, not APPLIC_ORIGIN */
	put_unaligned_be32(plen, f + 22);
	if (op == JEDI_OP_SET && plen)
		memcpy(f + JEDI_FRAME_HDR, payload, plen);

	mutex_lock(&priv->cmd_lock);

	memset(&pend, 0, sizeof(pend));
	pend.oid = oid;
	pend.buf = reply;
	pend.buf_len = reply ? reply_max : 0;
	init_completion(&pend.done);
	spin_lock_bh(&priv->pend_lock);
	priv->pend = &pend;
	spin_unlock_bh(&priv->pend_lock);

	ret = jedi_tx_frame(priv, f, flen);
	if (!ret) {
		if (!wait_for_completion_timeout(&pend.done,
				msecs_to_jiffies(JEDI_CMD_TIMEOUT_MS))) {
			dev_err(&priv->func->dev,
				"jedi: command op %u oid 0x%08x timed out\n",
				op, oid);
			ret = -ETIMEDOUT;
		} else {
			ret = pend.status;
		}
	}

	spin_lock_bh(&priv->pend_lock);
	priv->pend = NULL;
	spin_unlock_bh(&priv->pend_lock);
	if (jedi_cmd_delay)
		msleep(jedi_cmd_delay);
	mutex_unlock(&priv->cmd_lock);

	if (reply_len)
		*reply_len = pend.reply_len;
	kfree(f);
	return ret;
}

/* Log the first few data frames so the first DHCP exchange shows whether the
 * "plain Ethernet on the data port" assumption holds. Counts both directions
 * together; the budget is per module load. */
void jedi_trace_data(struct jedi_private *priv, const char *dir,
		     const u8 *buf, u32 len)
{
	if (priv->data_traced >= jedi_data_trace)
		return;
	priv->data_traced++;
	dev_info(&priv->func->dev,
		 "jedi: data %s %u bytes, dst %pM src %pM type 0x%04x\n", dir,
		 len, buf, buf + 6, len >= 14 ? (buf[12] << 8) | buf[13] : 0);
	print_hex_dump_bytes("jedi: data ", DUMP_PREFIX_OFFSET, buf,
			     min_t(u32, len, 64));
}

/*
 * A normal data frame. The firmware hands us plain Ethernet frames (R6.3).
 * Called from the SDIO IRQ thread, which is process context.
 */
static void jedi_rx_data(struct jedi_private *priv, const u8 *buf, u32 len)
{
	struct net_device *ndev = priv->ndev;
	struct sk_buff *skb;

	if (!netif_running(ndev))
		return;
	jedi_trace_data(priv, "rx", buf, len);

	skb = dev_alloc_skb(len + NET_IP_ALIGN);
	if (!skb) {
		ndev->stats.rx_dropped++;
		return;
	}
	skb_reserve(skb, NET_IP_ALIGN);
	memcpy(skb_put(skb, len), buf, len);
	skb->protocol = eth_type_trans(skb, ndev);
	skb->ip_summed = CHECKSUM_NONE;

	ndev->stats.rx_packets++;
	ndev->stats.rx_bytes += len;

	/*
	 * The firmware sends EAPOL message 1 before the link-up trap, and the
	 * supplicant drops EAPOL it receives before it has seen the AP event
	 * (the AP then waits 3 s to retry). Hold it until the event is out.
	 */
	if (skb->protocol == htons(ETH_P_PAE)) {
		unsigned long fl;
		bool hold;

		spin_lock_irqsave(&priv->eapol_lock, fl);
		hold = !priv->eapol_ok;
		if (hold) {
			if (priv->eapol_hold)
				dev_kfree_skb_any(priv->eapol_hold);
			priv->eapol_hold = skb;
		}
		spin_unlock_irqrestore(&priv->eapol_lock, fl);
		if (hold)
			return;
	}
	netif_rx_ni(skb);
}

/* ok: deliver a held EAPOL frame and let later ones through; else drop. */
void jedi_eapol_release(struct jedi_private *priv, bool ok)
{
	struct sk_buff *skb;
	unsigned long fl;

	spin_lock_irqsave(&priv->eapol_lock, fl);
	priv->eapol_ok = ok;
	skb = priv->eapol_hold;
	priv->eapol_hold = NULL;
	spin_unlock_irqrestore(&priv->eapol_lock, fl);
	if (!skb)
		return;
	if (ok)
		netif_rx_ni(skb);
	else
		dev_kfree_skb_any(skb);
}


/* A complete frame has been read from the data port. */
void jedi_rx_frame(struct jedi_private *priv, const u8 *buf, u32 len)
{
	struct sdio_func *func = priv->func;
	struct jedi_pending *p;
	u32 oid, plen;
	u8 op;

	jedi_dump_frame(func, "rx", buf, len);

	if (len < 14)
		return;			/* WinCE discards these too */

	if (((buf[12] << 8) | buf[13]) != JEDI_ETH_P_PIMFOR) {
		jedi_rx_data(priv, buf, len);
		return;
	}
	if (len < JEDI_FRAME_HDR || buf[14] != JEDI_PIMFOR_VERSION) {
		dev_warn(&func->dev, "jedi: malformed management frame (%u bytes)\n",
			 len);
		return;
	}

	op = buf[15];
	oid = get_unaligned_be32(buf + 16);
	plen = get_unaligned_be32(buf + 22);
	if (plen > len - JEDI_FRAME_HDR)
		plen = len - JEDI_FRAME_HDR;

	if (op == JEDI_OP_TRAP) {
		jedi_handle_trap(priv, oid, buf + JEDI_FRAME_HDR, plen);
		return;
	}
	if (op != JEDI_OP_RESPONSE && op != JEDI_OP_ERROR) {
		dev_warn(&func->dev, "jedi: unexpected PIMFOR op %u oid 0x%08x\n",
			 op, oid);
		return;
	}

	spin_lock(&priv->pend_lock);
	p = priv->pend;
	if (p && p->oid == oid) {
		if (op == JEDI_OP_ERROR)
			p->status = -EIO;
		p->reply_len = min(plen, p->buf_len);
		if (p->reply_len)
			memcpy(p->buf, buf + JEDI_FRAME_HDR, p->reply_len);
		complete(&p->done);
		spin_unlock(&priv->pend_lock);
		return;
	}
	spin_unlock(&priv->pend_lock);
	dev_warn(&func->dev, "jedi: unmatched %s oid 0x%08x, %u bytes\n",
		 op == JEDI_OP_ERROR ? "error" : "response", oid, plen);
}

/*
 * RX (R6.3): status bit 0 means a frame is waiting; its length is the 16-bit
 * register at 0x1c. Read it from the data port, at most 512 bytes a time.
 */
static void jedi_rx(struct jedi_private *priv)
{
	struct sdio_func *func = priv->func;
	u32 len = 0, got = 0;

	if (jedi_reg_read(func, JEDI_REG_RX_LEN, 2, &len))
		return;
	if (!len) {
		priv->rx_zero_cnt++;
		return;
	}

	while (got < len) {
		u32 n = min_t(u32, len - got, JEDI_BURST);

		sdio_readsb(func, priv->bounce, 0x0, n);
		if (got < JEDI_RX_MAX)
			memcpy(priv->rx_buf + got, priv->bounce,
			       min_t(u32, n, JEDI_RX_MAX - got));
		got += n;
	}
	if (len > JEDI_RX_MAX) {
		dev_warn(&func->dev, "jedi: oversized RX frame (%u), dropped\n",
			 len);
		return;
	}
	jedi_rx_frame(priv, priv->rx_buf, len);
}

/*
 * SDIO function interrupt handler (R2.4, R6.1). Called by the MMC core with
 * the host claimed. Only claimed AFTER the firmware has booted.
 *
 * Status byte 0x04: bit 2 = HOST_INT pending (read it, write it back to
 * 0x48 to ack); bit 0 = a received frame is waiting.
 */
static void jedi_sdio_irq(struct sdio_func *func)
{
	struct jedi_private *priv = sdio_get_drvdata(func);
	u32 hostint = 0;
	u8 st;
	int ret;

	if (!priv)
		return;

	st = sdio_readb(func, JEDI_REG_STATUS, &ret);
	if (ret)
		return;

	if (st & JEDI_STATUS_HOST_INT) {
		ret = jedi_reg_read(func, JEDI_REG_HOST_INT, 2, &hostint);
		if (!ret)
			jedi_reg_write(func, JEDI_REG_HOST_INT_ACK, 2, hostint);
		jedi_dbg(func, "IRQ status=0x%02x HOST_INT=0x%04x\n", st, hostint);
		/* 0x0040 TX free / 0x0080 awake need no action yet: TX polls. */
	}

	priv->irq_cnt++;
	if (st & JEDI_STATUS_RX) {
		priv->irq_rx_cnt++;
		jedi_rx(priv);
	}
}

/* ---- network device ---- */

static void jedi_tx_work(struct work_struct *work)
{
	struct jedi_private *priv =
		container_of(work, struct jedi_private, tx_work);
	struct net_device *ndev = priv->ndev;
	struct sk_buff *skb;

	while ((skb = skb_dequeue(&priv->tx_q)) != NULL) {
		unsigned int len = skb->len;

		jedi_trace_data(priv, "tx", skb->data, len);
		if (jedi_tx_frame(priv, skb->data, len)) {
			ndev->stats.tx_errors++;
			ndev->stats.tx_dropped++;
		} else {
			ndev->stats.tx_packets++;
			ndev->stats.tx_bytes += len;
		}
		dev_kfree_skb(skb);

		if (netif_queue_stopped(ndev) &&
		    skb_queue_len(&priv->tx_q) < JEDI_TXQ_LOW)
			netif_wake_queue(ndev);
	}
}

static netdev_tx_t jedi_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct jedi_private *priv = netdev_priv(ndev);

	/* A zero length frame must never be sent (R6.2). */
	if (skb->len < ETH_HLEN) {
		ndev->stats.tx_dropped++;
		dev_kfree_skb(skb);
		return NETDEV_TX_OK;
	}

	if (jedi_tx_gate && priv->port_closed &&
	    skb->protocol != htons(ETH_P_PAE)) {
		ndev->stats.tx_dropped++;
		dev_kfree_skb(skb);
		return NETDEV_TX_OK;
	}

	skb_queue_tail(&priv->tx_q, skb);
	if (skb_queue_len(&priv->tx_q) >= JEDI_TXQ_MAX) {
		netif_stop_queue(ndev);
		/* the worker may have drained the queue meanwhile */
		if (skb_queue_len(&priv->tx_q) < JEDI_TXQ_LOW)
			netif_wake_queue(ndev);
	}
	queue_work(priv->wq, &priv->tx_work);
	return NETDEV_TX_OK;
}

static int jedi_open(struct net_device *ndev)
{
	/*
	 * Firmware is already running (it boots at probe). The carrier stays
	 * off until the firmware reports an association; the event that says
	 * so is not known yet (glossary R5.5, 0x18xxxxxx traps).
	 */
	netif_carrier_off(ndev);
	netif_start_queue(ndev);
	return 0;
}

static int jedi_stop(struct net_device *ndev)
{
	struct jedi_private *priv = netdev_priv(ndev);

	netif_stop_queue(ndev);
	cancel_work_sync(&priv->tx_work);
	cancel_delayed_work_sync(&priv->scan_work);
	cancel_work_sync(&priv->link_work);
	cancel_work_sync(&priv->clearkeys_work);
	cancel_delayed_work_sync(&priv->diag_work);
	cancel_delayed_work_sync(&priv->join_work);
	skb_queue_purge(&priv->tx_q);
	jedi_eapol_release(priv, false);
	priv->wx.connected = false;
	priv->wx.scan_pending = false;
	netif_carrier_off(ndev);
	return 0;
}

static void jedi_tx_timeout(struct net_device *ndev)
{
	struct jedi_private *priv = netdev_priv(ndev);

	dev_warn(&priv->func->dev, "jedi: TX timeout\n");
	ndev->stats.tx_errors++;
	queue_work(priv->wq, &priv->tx_work);
	netif_wake_queue(ndev);
}

/*
 * Bring-up aid: /sys/bus/sdio/devices/<dev>/cmd
 *   echo "set 18000004 ffff"   > cmd     (OID in hex, payload as hex bytes)
 *   echo "get 17000021 4"      > cmd     (OID in hex, expected size in decimal)
 *   cat cmd                              (status and reply/error payload)
 * Payload bytes are sent as given (little-endian values: "0b000000" = 11).
 */
static ssize_t jedi_cmd_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct sdio_func *func = dev_to_sdio_func(dev);
	struct jedi_private *priv = sdio_get_drvdata(func);

	if (!priv || !priv->cmd_out)
		return sprintf(buf, "no result\n");
	return snprintf(buf, PAGE_SIZE, "%s", priv->cmd_out);
}

static ssize_t jedi_cmd_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct sdio_func *func = dev_to_sdio_func(dev);
	struct jedi_private *priv = sdio_get_drvdata(func);
	char op[8], *out;
	u32 oid, got = 0, size = 0;
	u8 *pl = NULL, *rep = NULL;
	int n = 0, ret, i, o;
	char hex[1100] = "";

	if (!priv)
		return -ENODEV;
	if (sscanf(buf, "%7s %x %n", op, &oid, &n) < 2)
		return -EINVAL;
	if (!priv->cmd_out) {
		priv->cmd_out = kzalloc(PAGE_SIZE, GFP_KERNEL);
		if (!priv->cmd_out)
			return -ENOMEM;
	}
	out = priv->cmd_out;
	rep = kmalloc(1024, GFP_KERNEL);
	if (!rep)
		return -ENOMEM;

	if (!strcmp(op, "set")) {
		size_t hl;
		int k;

		sscanf(buf + n, "%1099s", hex);
		hl = strlen(hex);
		if (hl & 1) {
			kfree(rep);
			return -EINVAL;
		}
		pl = kmalloc(hl / 2 + 1, GFP_KERNEL);
		if (!pl) {
			kfree(rep);
			return -ENOMEM;
		}
		for (k = 0; k < hl / 2; k++) {
			unsigned int v;
			char t[3] = { hex[2 * k], hex[2 * k + 1], 0 };

			if (sscanf(t, "%x", &v) != 1) {
				kfree(pl);
				kfree(rep);
				return -EINVAL;
			}
			pl[k] = v;
		}
		ret = jedi_cmd(priv, JEDI_OP_SET, oid, pl, hl / 2, rep, 1024, &got);
		kfree(pl);
	} else if (!strcmp(op, "get")) {
		if (sscanf(buf + n, "%u", &size) != 1 || size > 1024) {
			kfree(rep);
			return -EINVAL;
		}
		ret = jedi_cmd(priv, JEDI_OP_GET, oid, NULL, size, rep, 1024, &got);
	} else {
		kfree(rep);
		return -EINVAL;
	}

	o = snprintf(out, PAGE_SIZE, "%s 0x%08x: status %d, %u bytes:", op, oid,
		     ret, got);
	for (i = 0; i < got && o < PAGE_SIZE - 4; i++)
		o += snprintf(out + o, PAGE_SIZE - o, " %02x", rep[i]);
	snprintf(out + o, PAGE_SIZE - o, "\n");
	dev_info(&func->dev, "jedi: %s", out);
	kfree(rep);
	return count;
}

static DEVICE_ATTR(cmd, 0600, jedi_cmd_show, jedi_cmd_store);

static const struct net_device_ops jedi_netdev_ops = {
	.ndo_open		= jedi_open,
	.ndo_stop		= jedi_stop,
	.ndo_start_xmit		= jedi_xmit,
	.ndo_tx_timeout		= jedi_tx_timeout,
	.ndo_change_mtu		= eth_change_mtu,
	.ndo_validate_addr	= eth_validate_addr,
};

static const struct sdio_device_id jedi_ids[] = {
	{ SDIO_DEVICE(JEDI_VENDOR_ID, JEDI_DEVICE_ID) },
	{ /* Sentinel */ },
};
MODULE_DEVICE_TABLE(sdio, jedi_ids);

/*
 * R2.1 discriminating test. With the module powered and idle, CMD52 reads
 * of 0x40 and 0x41 are separate bytes. A fixed-address CMD53 read of 2
 * bytes at 0x40 should return byte 0x40 twice. Idle values may all be 0,
 * so this is most informative once the chip has been reset (see the
 * "after chip reset" register dump in jedi_fw.c).
 * Must be called with the host claimed.
 */
static void jedi_regtest(struct sdio_func *func)
{
	u8 b40, b41, pair[2] = { 0, 0 };
	u32 devstat = 0, intena = 0;
	int e40, e41, e54, e44;

	b40 = sdio_readb(func, JEDI_REG_HOST_INT, &e40);
	b41 = sdio_readb(func, JEDI_REG_HOST_INT + 1, &e41);
	sdio_readsb(func, pair, JEDI_REG_HOST_INT, 2);
	e54 = jedi_reg_read(func, JEDI_REG_DEVSTAT, 4, &devstat);
	e44 = jedi_reg_read(func, JEDI_REG_INT_ENABLE, 2, &intena);

	jedi_dbg(func,
		 "regtest: CMD52 0x40=0x%02x (err %d) 0x41=0x%02x (err %d); "
		 "CMD53 fixed readsb(0x40,2)=%02x %02x\n",
		 b40, e40, b41, e41, pair[0], pair[1]);
	jedi_dbg(func,
		 "regtest: DEVSTAT=0x%08x (err %d) INT_ENABLE=0x%04x (err %d)\n",
		 devstat, e54, intena, e44);
}

static int jedi_probe(struct sdio_func *func, const struct sdio_device_id *id)
{
	struct net_device *ndev;
	struct jedi_private *priv;
	int ret;

	jedi_dbg(func, "probing SDIO device Vendor: 0x%04x, Device: 0x%04x "
		 "(Func %d)\n", func->vendor, func->device, func->num);

	/* Allocate the network device; Android expects the name wlan0. */
	ndev = alloc_netdev(sizeof(struct jedi_private), "wlan%d", ether_setup);
	if (!ndev)
		return -ENOMEM;
	SET_NETDEV_DEV(ndev, &func->dev);
	ndev->netdev_ops = &jedi_netdev_ops;
	ndev->watchdog_timeo = JEDI_TX_TIMEOUT;
	ndev->wireless_handlers = &jedi_wext_handler_def;

	priv = netdev_priv(ndev);
	priv->func = func;
	priv->ndev = ndev;
	priv->irq_claimed = false;
	mutex_init(&priv->cmd_lock);
	mutex_init(&priv->wx.lock);
	spin_lock_init(&priv->pend_lock);
	skb_queue_head_init(&priv->tx_q);
	INIT_WORK(&priv->tx_work, jedi_tx_work);
	INIT_DELAYED_WORK(&priv->scan_work, jedi_scan_work);
	init_completion(&priv->country_done);
	spin_lock_init(&priv->eapol_lock);
	INIT_WORK(&priv->link_work, jedi_link_work);
	INIT_WORK(&priv->clearkeys_work, jedi_clearkeys_work);
	INIT_DELAYED_WORK(&priv->diag_work, jedi_diag_work);
	INIT_DELAYED_WORK(&priv->join_work, jedi_join_work);
	priv->wq = create_singlethread_workqueue("jedi_tx");
	priv->bounce = kmalloc(JEDI_BURST, GFP_KERNEL);
	priv->rx_buf = kmalloc(JEDI_RX_MAX, GFP_KERNEL);
	if (!priv->wq || !priv->bounce || !priv->rx_buf) {
		ret = -ENOMEM;
		goto err_free_dev;
	}
	sdio_set_drvdata(func, priv);

	/* Claim host for configuration */
	sdio_claim_host(func);

	ret = sdio_enable_func(func);
	if (ret) {
		dev_err(&func->dev,
			"jedi: failed to enable SDIO function: %d\n",
			ret);
		sdio_release_host(func);
		goto err_free_dev;
	}

	/* WinCE sets block size 0x200 */
	ret = sdio_set_block_size(func, 512);
	if (ret) {
		dev_warn(&func->dev,
			 "jedi: unable to set block size to 512 (%d), "
			 "trying 64\n", ret);
		sdio_set_block_size(func, 64);
	}

	if (jedi_debug)
		jedi_regtest(func);

	sdio_release_host(func);

	/*
	 * Reset the chip into its boot ROM, upload the image, wait for FW
	 * Init, then run the init job. The card has been seen to refuse the
	 * first data burst of the init job and then stop answering; if that
	 * happens, reset the chip and start again (jedi.init_retries).
	 * jedi_fw_download() claims the host itself.
	 */
	{
		unsigned int attempt;

		for (attempt = 0; ; attempt++) {
			u8 mac[ETH_ALEN] = { 0 };
			u32 got = 0;

			ret = jedi_fw_download(func);
			if (ret) {
				dev_err(&func->dev,
					"jedi: firmware download failed: %d\n",
					ret);
				/* A missing image will not appear by waiting;
				 * anything else: let the chip settle and run the
				 * whole reset + download again. */
				if (ret == -ENOENT)
					goto err_disable_func;
				if (attempt >= jedi_init_retries) {
					if (ret == -ETIMEDOUT)
						jedi_request_recovery(func,
							"firmware download keeps failing");
					goto err_disable_func;
				}
				dev_warn(&func->dev,
					 "jedi: retrying in 1 s (%u/%u)\n",
					 attempt + 1, jedi_init_retries);
				msleep(1000);
				continue;
			}

			/* Firmware is running: now it is safe to take its
			 * interrupt. */
			sdio_claim_host(func);
			ret = sdio_claim_irq(func, jedi_sdio_irq);
			sdio_release_host(func);
			if (ret) {
				dev_err(&func->dev,
					"jedi: failed to claim SDIO IRQ: %d\n",
					ret);
				goto err_disable_func;
			}
			priv->irq_claimed = true;
			priv->hw_failed = false;
			priv->tx_traced = 0;

			/* First command (R6.6): ask for the MAC address. */
			ret = jedi_cmd(priv, JEDI_OP_GET, JEDI_OID_MAC, NULL,
				       ETH_ALEN, mac, sizeof(mac), &got);
			if (ret || got != ETH_ALEN) {
				dev_err(&func->dev,
					"jedi: GET MAC failed: %d (%u bytes), "
					"using a random address\n", ret, got);
				if (is_zero_ether_addr(ndev->dev_addr))
					random_ether_addr(ndev->dev_addr);
				priv->hw_failed = true;
			} else {
				memcpy(ndev->dev_addr, mac, ETH_ALEN);
				dev_info(&func->dev, "jedi: MAC address %pM\n",
					 mac);
				jedi_hw_init(priv);
			}
			if (!priv->hw_failed)
				break;
			if (attempt >= jedi_init_retries) {
				dev_err(&func->dev,
					"jedi: init failed %u times, giving up "
					"(the interface is left unconfigured)\n",
					attempt + 1);
				jedi_request_recovery(func, "init keeps failing");
				break;
			}
			dev_warn(&func->dev,
				 "jedi: init failed, resetting the chip "
				 "(retry %u/%u)\n", attempt + 1, jedi_init_retries);
			sdio_claim_host(func);
			sdio_release_irq(func);
			sdio_release_host(func);
			priv->irq_claimed = false;
		}
	}

	ret = register_netdev(ndev);
	if (ret) {
		dev_err(&func->dev, "jedi: register_netdev failed: %d\n", ret);
		goto err_release_irq;
	}
	dev_info(&func->dev, "jedi: registered %s, MAC %pM\n",
		 ndev->name, ndev->dev_addr);
	if (device_create_file(&func->dev, &dev_attr_cmd))
		dev_warn(&func->dev, "jedi: cannot create cmd attribute\n");

	return 0;

err_release_irq:
	sdio_claim_host(func);
	sdio_release_irq(func);
	sdio_release_host(func);
	priv->irq_claimed = false;
err_disable_func:
	sdio_claim_host(func);
	sdio_disable_func(func);
	sdio_release_host(func);
err_free_dev:
	sdio_set_drvdata(func, NULL);
	if (priv->wq)
		destroy_workqueue(priv->wq);
	kfree(priv->bounce);
	kfree(priv->rx_buf);
	free_netdev(ndev);
	return ret;
}

static void jedi_remove(struct sdio_func *func)
{
	struct jedi_private *priv = sdio_get_drvdata(func);

	if (priv) {
		struct net_device *ndev = priv->ndev;

		/* unregister first: ndo_stop stops the TX worker */
		device_remove_file(&func->dev, &dev_attr_cmd);
		unregister_netdev(ndev);
		cancel_delayed_work_sync(&priv->scan_work);
			cancel_work_sync(&priv->link_work);
		cancel_work_sync(&priv->clearkeys_work);
		cancel_delayed_work_sync(&priv->diag_work);
		cancel_delayed_work_sync(&priv->join_work);
		/* leave the radio off: drop any association, SET 0x0b = 1 */
		jedi_hw_shutdown(priv);
		kfree(priv->cmd_out);

		sdio_claim_host(func);
		if (priv->irq_claimed)
			sdio_release_irq(func);
		sdio_release_host(func);
		priv->irq_claimed = false;
		/* Wi-Fi off: stop the firmware (boot ROM idle), keep the card */
		if (jedi_fw_halt(func))
			dev_warn(&func->dev, "jedi: could not park the chip\n");
		sdio_claim_host(func);
		sdio_disable_func(func);
		sdio_release_host(func);

		sdio_set_drvdata(func, NULL);
		destroy_workqueue(priv->wq);
		kfree(priv->bounce);
		kfree(priv->rx_buf);
		free_netdev(ndev);	/* frees priv too */
	}
	pr_info("jedi: Removed\n");
}

static struct sdio_driver jedi_driver = {
	.name     = "jedi",
	.id_table = jedi_ids,
	.probe    = jedi_probe,
	.remove   = jedi_remove,
};

static int __init jedi_init(void)
{
	/* Wi-Fi on: if a previous unload cut the power, restore it and let the
	 * board re-enumerate the card before the driver registers. */
	if (jedi_power_ctl && mc3190_wifi_power_on)
		mc3190_wifi_power_on();
	return sdio_register_driver(&jedi_driver);
}

static void __exit jedi_exit(void)
{
	cancel_work_sync(&jedi_recover_work);
	sdio_unregister_driver(&jedi_driver);
	/* Wi-Fi off: the radio was disabled by jedi_remove(); now remove power
	 * as well. The MMC core is not told: the next power-on re-detects. */
	if (jedi_power_ctl && mc3190_wifi_power_off)
		mc3190_wifi_power_off();
}

module_init(jedi_init);
module_exit(jedi_exit);

MODULE_AUTHOR("Mark Kennard <markkennard4@gmail.com>");
MODULE_DESCRIPTION("Symbol Jedi SDIO Wi-Fi Driver");
MODULE_LICENSE("GPL");