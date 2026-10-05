/*
 * jedi.c -- Driver for Symbol "Jedi" WiFi module in MC3190
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
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/completion.h>
#include <linux/skbuff.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <asm/byteorder.h>
#include <asm/unaligned.h>
#include <linux/etherdevice.h>
#include <linux/if_arp.h>
#include <linux/wireless.h>
#include <net/iw_handler.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>

#include "jedi.h"

#define JEDI_VENDOR_ID  0x0013
#define JEDI_DEVICE_ID  0xc684

bool jedi_debug;
module_param_named(debug, jedi_debug, bool, 0644);
MODULE_PARM_DESC(debug, "Enable verbose Jedi driver logging (default off)");

/* ---- PIMFOR management protocol (glossary R5/R6) ---- */
#define JEDI_ETH_P_PIMFOR	0x8828
#define JEDI_PIMFOR_HDR		12		/* version..length */
#define JEDI_FRAME_HDR		(14 + JEDI_PIMFOR_HDR)	/* 26 */
#define JEDI_PIMFOR_VERSION	1
#define JEDI_OP_GET		0
#define JEDI_OP_SET		1
#define JEDI_OP_RESPONSE	2
#define JEDI_OP_ERROR		3
#define JEDI_OP_TRAP		4

#define JEDI_OID_MAC		0x00000000	/* GET, 6 bytes */

#define JEDI_REG_RX_LEN		0x1c		/* R16: received frame length */
#define JEDI_REG_TX_LEN		0x50		/* W16: len | 0x4000; R: ready */
#define JEDI_TX_READY		0x4000
#define JEDI_STATUS_RX		0x01		/* status bit 0: RX pending */
#define JEDI_HOST_INT_TX_FREE	0x0040		/* [INF] TX buffer free */

#define JEDI_BURST		512
#define JEDI_RX_MAX		2400
#define JEDI_CMD_TIMEOUT_MS	4000
#define JEDI_TX_READY_TRIES	200

struct jedi_pending {
	u32 oid;
	u8 *buf;
	u32 buf_len;
	u32 reply_len;
	int status;
	struct completion done;
};

/*
 * Wireless extensions state (WE-22), written for Android 4.0 wpa_supplicant_8
 * (VER_0_8_X, driver WEXT). See the handler table further down.
 */
#define JEDI_MAX_BSS		32
#define JEDI_MAX_IE		256
#define JEDI_SSID_MAX		32

struct jedi_bss {
	u8 bssid[ETH_ALEN];
	u8 ssid[JEDI_SSID_MAX];
	u8 ssid_len;
	u16 freq;			/* MHz */
	s8 level;			/* dBm */
	u16 caps;			/* 802.11 capability info */
	u8 ie_len;
	u8 ie[JEDI_MAX_IE];		/* concatenated WPA/RSN/etc IEs */
};

struct jedi_wext {
	struct mutex lock;
	u8 ssid[JEDI_SSID_MAX];
	u8 ssid_len;
	u8 bssid[ETH_ALEN];		/* requested BSSID */
	bool bssid_set;
	u8 cur_bssid[ETH_ALEN];		/* associated BSSID */
	bool connected;
	bool scan_pending;		/* SIOCGIWSCAN returns -EAGAIN */
	u32 auth[IW_AUTH_MFP + 1];	/* SIOCSIWAUTH values */
	u8 gen_ie[JEDI_MAX_IE];		/* SIOCSIWGENIE (WPA/RSN IE) */
	u8 gen_ie_len;
	struct jedi_bss bss[JEDI_MAX_BSS];
	int nbss;
	struct iw_statistics stats;
};

struct jedi_private {
	struct sdio_func *func;
	struct net_device *ndev;
	bool irq_claimed;

	struct mutex cmd_lock;		/* one command in flight */
	spinlock_t pend_lock;
	struct jedi_pending *pend;

	u8 *bounce;			/* DMA-safe 512 byte bus buffer */
	u8 *rx_buf;			/* assembled received frame */

	/* Data TX: ndo_start_xmit cannot sleep, SDIO access does. */
	struct sk_buff_head tx_q;
	struct workqueue_struct *wq;
	struct work_struct tx_work;

	struct jedi_wext wx;
	struct delayed_work scan_work;
	struct work_struct link_work;	/* GET BSSID after a connect trap */
	char *cmd_out;		/* sysfs 'cmd' result */
	int scan_tries;
};

/* OIDs (glossary R5, R7) */
#define JEDI_OID_RADIO_STATE	0x0000000b
#define JEDI_OID_INACTIVE_STATE	0x0000000c
#define JEDI_OID_FRAG_THR	0x13000001
#define JEDI_OID_RTS_THR	0x13000000
#define JEDI_OID_PHYCAP		0x17000021
#define JEDI_OID_SCAN_DONE_TRAP	0x1400000a
#define JEDI_OID_MIC_FAIL_TRAP	0x150007e1
#define JEDI_OID_WEP_DEFKEY	0x12000003	/* default (TX) key index */
#define JEDI_OID_WEP_SLOT0	0x12000004	/* + index, 20 bytes */
#define JEDI_OID_STA_KEY	0x12000008	/* 44 bytes */
#define JEDI_OID_STA_KEY_RSC	0x1200000a	/* 64 bytes */
#define JEDI_OID_SECURITY	0x1200000b	/* suite code */
#define JEDI_OID_SEC_MODE	0x12000012	/* driver security mode */
#define JEDI_OID_BAND_PREF	0x14000003
#define JEDI_OID_WWR_MODE	0x1f000005
#define JEDI_OID_CISCO_SCAN_DIS	0x13000109
#define JEDI_OID_QOS_NULL_TAG	0x1300010a
#define JEDI_OID_TXPOWER	0x1700000f
#define JEDI_OID_DSFREQ		0x17000011
#define JEDI_OID_SUPP_FREQS	0x17000012
#define JEDI_OID_DISABLE_QOS	0x1b00000f
#define JEDI_OID_PREAMBLE	0x17000009
#define JEDI_OID_ANT_DIVERSITY	0x17000006
#define JEDI_OID_BT_OPTIONS	0xa6000001
#define JEDI_OID_CCX_PARAMS	0xa0000008
#define JEDI_OID_BSS_TYPE	0x10000000
#define JEDI_OID_BSSID		0x10000001
#define JEDI_OID_SSID		0x10000002
#define JEDI_OID_LINKSTATE	0x00000001
#define JEDI_OID_DEAUTH_TRAP	0x18000000
#define JEDI_OID_AUTH_TRAP	0x18000001
#define JEDI_OID_DISASSOC_TRAP	0x18000002
#define JEDI_OID_ASSOC_TRAP	0x18000003
#define JEDI_OID_SCAN		0x18000004
#define JEDI_OID_REASSOC_TRAP	0x1800000b
#define JEDI_OID_BSSLIST	0x1c000044
#define JEDI_OID_BAND_MASK	0x1f000000
#define JEDI_OID_MAX_SBSPECS	0x1f00000f

#define JEDI_BSS_INFRA		1
#define JEDI_BAND_BOTH		3
#define JEDI_SSID_WIRE		0x24
#define JEDI_BSS_REQ_LEN	0x42	/* SET body for the BSS list */
#define JEDI_BSS_MAX		0x242	/* one BSS list entry */
#define JEDI_BSS_HDR		0x40	/* fixed part, IEs follow */
#define JEDI_SCAN_LEN		12	/* scan parameter block */
#define JEDI_SCAN_RETRIES	3
#define JEDI_SCAN_RETRY_MS	2500
#define JEDI_SCAN_WAIT_MS	3000	/* guess: time for a full scan */

#define JEDI_TXQ_MAX		64
#define JEDI_TXQ_LOW		16
#define JEDI_TX_TIMEOUT		(5 * HZ)

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
 * Send one frame (R6.2). Takes the host itself.
 *   1. read 0x50; send only if 0x4000 is set (a failed read counts as ready)
 *   2. write 0x50 = len | 0x4000
 *   3. write the frame to the data port in bursts of at most 512 bytes
 */
static int jedi_tx_frame(struct jedi_private *priv, const u8 *frame, u32 len)
{
	struct sdio_func *func = priv->func;
	u32 sent = 0, v = 0;
	unsigned int i;
	int ret = 0;

	if (!len)
		return -EINVAL;

	jedi_dump_frame(func, "tx", frame, len);

	sdio_claim_host(func);

	for (i = 0; i < JEDI_TX_READY_TRIES; i++) {
		if (jedi_reg_read(func, JEDI_REG_TX_LEN, 2, &v) ||
		    (v & JEDI_TX_READY))
			break;
		sdio_release_host(func);
		msleep(1);
		sdio_claim_host(func);
	}
	if (i == JEDI_TX_READY_TRIES) {
		dev_err(&func->dev, "jedi: TX not ready (0x50=0x%04x)\n", v);
		ret = -EBUSY;
		goto out;
	}

	ret = jedi_reg_write(func, JEDI_REG_TX_LEN, 2, len | JEDI_TX_READY);
	if (ret)
		goto out;

	while (sent < len) {
		u32 n = min_t(u32, len - sent, JEDI_BURST);

		memcpy(priv->bounce, frame + sent, n);
		ret = sdio_writesb(func, 0x0, priv->bounce, n);
		if (ret) {
			dev_err(&func->dev,
				"jedi: TX burst failed at %u/%u: %d\n",
				sent, len, ret);
			goto out;
		}
		sent += n;
	}
out:
	sdio_release_host(func);
	return ret;
}

/*
 * Send a PIMFOR command and wait for the reply matched by OID (R5.1, R5.3).
 * For GET, @plen is the expected reply size and no payload is sent.
 * @reply (optional) receives up to @reply_max bytes; *@reply_len gets the
 * actual size.
 */
static int jedi_cmd(struct jedi_private *priv, u8 op, u32 oid,
		    const void *payload, u32 plen,
		    void *reply, u32 reply_max, u32 *reply_len)
{
	struct jedi_pending pend;
	u32 flen = JEDI_FRAME_HDR + (op == JEDI_OP_SET ? plen : 0);
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
	mutex_unlock(&priv->cmd_lock);

	if (reply_len)
		*reply_len = pend.reply_len;
	kfree(f);
	return ret;
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
	netif_rx_ni(skb);
}

static void jedi_handle_trap(struct jedi_private *priv, u32 oid,
			     const u8 *p, u32 plen);
static void jedi_wext_link_event(struct jedi_private *priv, const u8 *bssid);
static void jedi_link_work(struct work_struct *work);

/* A complete frame has been read from the data port. */
static void jedi_rx_frame(struct jedi_private *priv, const u8 *buf, u32 len)
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

	if (jedi_reg_read(func, JEDI_REG_RX_LEN, 2, &len) || !len)
		return;

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

	if (st & JEDI_STATUS_RX)
		jedi_rx(priv);
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
	skb_queue_purge(&priv->tx_q);
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

/* ------------------------------------------------------------------ */
/* Wireless extensions                                                 */
/* ------------------------------------------------------------------ */

/*
 * Event entry points: call these from jedi_rx_frame() when the firmware
 * traps are understood.
 */

/* Scan finished: userspace then reads results with SIOCGIWSCAN. */
static void jedi_wext_scan_done(struct jedi_private *priv)
{
	union iwreq_data wrqu;

	memset(&wrqu, 0, sizeof(wrqu));
	wireless_send_event(priv->ndev, SIOCGIWSCAN, &wrqu, NULL);
}

/* Association up (bssid != NULL) or down (bssid == NULL). */
static void jedi_wext_link_event(struct jedi_private *priv, const u8 *bssid)
{
	struct jedi_wext *wx = &priv->wx;
	union iwreq_data wrqu;

	memset(&wrqu, 0, sizeof(wrqu));
	wrqu.ap_addr.sa_family = ARPHRD_ETHER;

	mutex_lock(&wx->lock);
	if (bssid) {
		memcpy(wx->cur_bssid, bssid, ETH_ALEN);
		memcpy(wrqu.ap_addr.sa_data, bssid, ETH_ALEN);
		wx->connected = true;
	} else {
		memset(wx->cur_bssid, 0, ETH_ALEN);
		wx->connected = false;
	}
	mutex_unlock(&wx->lock);

	if (bssid)
		netif_carrier_on(priv->ndev);
	else
		netif_carrier_off(priv->ndev);
	wireless_send_event(priv->ndev, SIOCGIWAP, &wrqu, NULL);
}

static void __maybe_unused jedi_wext_bss_clear(struct jedi_private *priv)
{
	mutex_lock(&priv->wx.lock);
	priv->wx.nbss = 0;
	mutex_unlock(&priv->wx.lock);
}

/* Add or update a scan result. freq in MHz, level in dBm. */
static int __maybe_unused jedi_wext_bss_add(struct jedi_private *priv,
		const u8 *bssid, const u8 *ssid, u8 ssid_len, u16 freq,
		s8 level, u16 caps, const u8 *ie, u8 ie_len)
{
	struct jedi_wext *wx = &priv->wx;
	struct jedi_bss *b = NULL;
	int i, ret = 0;

	if (ssid_len > JEDI_SSID_MAX)
		ssid_len = JEDI_SSID_MAX;

	mutex_lock(&wx->lock);
	for (i = 0; i < wx->nbss; i++)
		if (!memcmp(wx->bss[i].bssid, bssid, ETH_ALEN)) {
			b = &wx->bss[i];
			break;
		}
	if (!b) {
		if (wx->nbss >= JEDI_MAX_BSS) {
			ret = -ENOSPC;
			goto out;
		}
		b = &wx->bss[wx->nbss++];
	}
	memcpy(b->bssid, bssid, ETH_ALEN);
	memcpy(b->ssid, ssid, ssid_len);
	b->ssid_len = ssid_len;
	b->freq = freq;
	b->level = level;
	b->caps = caps;
	b->ie_len = ie_len;
	if (ie_len)
		memcpy(b->ie, ie, ie_len);
out:
	mutex_unlock(&wx->lock);
	return ret;
}

/*
 * Hardware hooks (glossary R5.5, R7.1, R7.5, R7.6).
 */

/* Payload values are little-endian (the firmware is ARM LE; R7.3 trap). */
static int jedi_set_u32(struct jedi_private *priv, u32 oid, u32 val)
{
	u8 b[4];

	put_unaligned_le32(val, b);
	return jedi_cmd(priv, JEDI_OP_SET, oid, b, sizeof(b), NULL, 0, NULL);
}

/* Empty SSID (length 0, 36 zero bytes): "any network" and starts the MLME. */
static int jedi_hw_set_ssid_empty(struct jedi_private *priv)
{
	u8 b[JEDI_SSID_WIRE] = { 0 };
	int ret = jedi_cmd(priv, JEDI_OP_SET, JEDI_OID_SSID, b, sizeof(b), NULL,
			   0, NULL);

	if (ret)
		dev_warn(&priv->func->dev, "jedi: init SET empty SSID failed: %d\n",
			 ret);
	return ret;
}

static int jedi_init_set(struct jedi_private *priv, u32 oid, u32 val)
{
	int ret = jedi_set_u32(priv, oid, val);

	if (ret)
		dev_warn(&priv->func->dev,
			 "jedi: init SET 0x%08x = %u failed: %d\n", oid, val, ret);
	return ret;
}

static int jedi_init_get(struct jedi_private *priv, u32 oid, u32 size,
			 u8 *buf, u32 *got)
{
	int ret = jedi_cmd(priv, JEDI_OP_GET, oid, NULL, size, buf, size, got);

	if (ret)
		dev_warn(&priv->func->dev,
			 "jedi: init GET 0x%08x failed: %d\n", oid, ret);
	return ret;
}

/*
 * Start-up sequence as WinCE sends it (glossary R7.6, R7.6b, R8.4): the
 * init job (R6-A) then the configuration job (R6-B), using the driver's
 * built-in defaults. Steps WinCE skips with this registry are not sent.
 * Failures only warn. The final empty-SSID SET matters most: the firmware
 * starts its MLME task on it (R7.6c), and a scan is rejected before that.
 */
static void jedi_hw_init(struct jedi_private *priv)
{
	u8 b[258];
	u32 got = 0, phycap = 3;

	/* --- R6-A init job --- */
	if (!jedi_init_get(priv, JEDI_OID_RADIO_STATE, 4, b, &got) && got == 4)
		jedi_dbg(priv->func, "0x0b = 0x%x\n", get_unaligned_le32(b));
	if (!jedi_init_get(priv, JEDI_OID_INACTIVE_STATE, 4, b, &got) && got == 4)
		jedi_dbg(priv->func, "0x0c = 0x%x\n", get_unaligned_le32(b));

	jedi_init_set(priv, JEDI_OID_FRAG_THR, 2346);
	jedi_init_set(priv, JEDI_OID_RTS_THR, 2347);
	if (!jedi_init_get(priv, JEDI_OID_CISCO_SCAN_DIS, 4, b, &got) && got == 4)
		jedi_dbg(priv->func, "0x13000109 = 0x%x\n",
			 get_unaligned_le32(b));
	jedi_init_set(priv, JEDI_OID_CISCO_SCAN_DIS, 1);	/* step 11 */
	jedi_init_set(priv, JEDI_OID_TXPOWER, 0);		/* step 12 */

	if (!jedi_init_get(priv, JEDI_OID_PHYCAP, 4, b, &got) && got == 4)
		phycap = get_unaligned_le32(b);
	jedi_dbg(priv->func, "phy capability 0x%x\n", phycap);
	if ((phycap & 3) == 3)					/* step 17 */
		jedi_init_set(priv, JEDI_OID_BAND_MASK, 3);
	else if (phycap & 3)
		jedi_init_set(priv, JEDI_OID_BAND_MASK, phycap & 3);

	jedi_init_set(priv, JEDI_OID_BAND_PREF, 0);		/* step 20 */
	jedi_init_set(priv, JEDI_OID_WWR_MODE, 3);		/* step 22 */
	jedi_init_set(priv, JEDI_OID_DSFREQ, 2462000);		/* step 23 */
	jedi_init_set(priv, JEDI_OID_QOS_NULL_TAG, 7);		/* step 26 */

	/* --- R6-B configuration job --- */
	jedi_init_set(priv, JEDI_OID_BSS_TYPE, JEDI_BSS_INFRA);
	jedi_init_set(priv, JEDI_OID_DISABLE_QOS, 0);
	jedi_init_set(priv, JEDI_OID_PREAMBLE, 2);
	jedi_init_set(priv, JEDI_OID_ANT_DIVERSITY, 1);
	jedi_init_set(priv, JEDI_OID_BT_OPTIONS, 0);
	jedi_set_u32(priv, JEDI_OID_SEC_MODE, 0);
	jedi_hw_set_ssid_empty(priv);				/* step 25 */
	jedi_init_set(priv, JEDI_OID_CCX_PARAMS, 5);

	/*
	 * R10: the band-mask SET may flag a channel-table rebuild that the
	 * MLME task only performs when it next runs. Give it time, then log
	 * the MLME state (0x10000003) and the channel list (0x1400000d).
	 */
	msleep(2000);
	if (!jedi_init_get(priv, 0x10000003, 4, b, &got) && got == 4)
		jedi_dbg(priv->func, "0x10000003 (MLME state) = 0x%x\n",
			 get_unaligned_le32(b));
	if (!jedi_init_get(priv, 0x1400000d, 64, b, &got))
		print_hex_dump_bytes("jedi: chanlist ", DUMP_PREFIX_OFFSET, b,
				     min_t(u32, got, 32));

	if (!jedi_init_get(priv, JEDI_OID_SUPP_FREQS, 258, b, &got)) {
		jedi_dbg(priv->func, "supported frequencies: %u bytes\n", got);
		if (jedi_debug)
			print_hex_dump_bytes("jedi: freqs ", DUMP_PREFIX_OFFSET,
					     b, min_t(u32, got, 64));
	}
	if (!jedi_init_get(priv, JEDI_OID_MAX_SBSPECS, 8, b, &got) && got == 8)
		jedi_dbg(priv->func, "max allowed: 2.4GHz 0x%08x 5GHz 0x%08x\n",
			 get_unaligned_le32(b), get_unaligned_le32(b + 4));
}

/*
 * Scan results (R7.1): loop of SET 0x1c000044 (0x42 bytes) then GET
 * 0x1c000044 (0x242 bytes); each GET returns one BSS entry, a zero-length
 * GET ends the list.
 *
 * TODO(RE): the real 0x42-byte SET block lives in JEDI10.dll's data
 * section. Zeros are sent until RE supplies it; the log will show whether
 * the firmware accepts that.
 */
static void jedi_parse_bss(struct jedi_private *priv, const u8 *e, u32 len)
{
	u8 ssid_len, ch;
	u16 caps, ie_len, freq;

	if (len < JEDI_BSS_HDR)
		return;
	ssid_len = e[0x0e];
	if (ssid_len > JEDI_SSID_MAX)
		ssid_len = JEDI_SSID_MAX;
	ch = e[0x33];
	caps = get_unaligned_le16(e + 0x36);
	ie_len = get_unaligned_le16(e + 0x3e);
	if (ie_len > len - JEDI_BSS_HDR)
		ie_len = len - JEDI_BSS_HDR;
	if (ie_len > JEDI_MAX_IE - 1)
		ie_len = JEDI_MAX_IE - 1;

	if (ch >= 1 && ch <= 13)
		freq = 2407 + 5 * ch;
	else if (ch == 14)
		freq = 2484;
	else
		freq = 5000 + 5 * ch;

	jedi_wext_bss_add(priv, e, e + 0x0f, ssid_len, freq, (s8)e[0x0d], caps,
			  e + JEDI_BSS_HDR, ie_len);
}

static void jedi_scan_work(struct work_struct *work)
{
	struct jedi_private *priv = container_of(work, struct jedi_private,
						 scan_work.work);
	u8 req[JEDI_BSS_REQ_LEN] = { 0 };
	u8 *e;
	int i, ret;
	u32 got;

	e = kmalloc(JEDI_BSS_MAX, GFP_KERNEL);
	if (!e) {
		mutex_lock(&priv->wx.lock);
		priv->wx.scan_pending = false;
		mutex_unlock(&priv->wx.lock);
		jedi_wext_scan_done(priv);
		return;
	}

	for (i = 0; i < JEDI_MAX_BSS; i++) {
		ret = jedi_cmd(priv, JEDI_OP_SET, JEDI_OID_BSSLIST, req,
			       sizeof(req), NULL, 0, NULL);
		if (ret)
			jedi_dbg(priv->func, "BSS list SET failed: %d\n", ret);

		got = 0;
		ret = jedi_cmd(priv, JEDI_OP_GET, JEDI_OID_BSSLIST, NULL,
			       JEDI_BSS_MAX, e, JEDI_BSS_MAX, &got);
		if (ret) {
			dev_warn(&priv->func->dev,
				 "jedi: BSS list GET failed: %d (entry %d)\n",
				 ret, i);
			break;
		}
		if (!got)
			break;
		if (jedi_debug && i == 0)
			print_hex_dump_bytes("jedi: bss0 ", DUMP_PREFIX_OFFSET,
					     e, min_t(u32, got, 128));
		jedi_parse_bss(priv, e, got);
	}
	jedi_dbg(priv->func, "scan: %d entries read, %d cached\n", i,
		 priv->wx.nbss);
	kfree(e);
	if (!priv->wx.nbss && priv->scan_tries < JEDI_SCAN_RETRIES) {
		/* Nothing yet: the scan may still be running. Look again. */
		priv->scan_tries++;
		schedule_delayed_work(&priv->scan_work,
				      msecs_to_jiffies(JEDI_SCAN_RETRY_MS));
		return;
	}
	mutex_lock(&priv->wx.lock);
	priv->wx.scan_pending = false;
	mutex_unlock(&priv->wx.lock);
	jedi_wext_scan_done(priv);
}

/*
 * Scan trigger (R8.1): SET 0x18000004 with exactly 2 bytes = signed channel
 * count, 0xffff = all. (A 12-byte payload only stores the dwell-time
 * parameters, which already have good defaults, so it is not written.)
 * The firmware returns success without scanning if a scan or join is
 * already running; results are read from the BSS list either way.
 */
static int jedi_hw_scan(struct jedi_private *priv, const u8 *ssid, u8 ssid_len)
{
	static const u8 all[2] = { 0xff, 0xff };
	int ret;

	/* TODO(RE): directed (single-SSID) scan; plain scan for now. */
	jedi_wext_bss_clear(priv);

	ret = jedi_cmd(priv, JEDI_OP_SET, JEDI_OID_SCAN, all, sizeof(all),
		       NULL, 0, NULL);
	if (ret) {
		dev_warn(&priv->func->dev, "jedi: scan SET failed: %d\n", ret);
		return ret;
	}
	mutex_lock(&priv->wx.lock);
	priv->wx.scan_pending = true;
	priv->scan_tries = 0;
	mutex_unlock(&priv->wx.lock);
	schedule_delayed_work(&priv->scan_work,
			      msecs_to_jiffies(JEDI_SCAN_WAIT_MS));
	return 0;
}

/* SSID: byte 0 = length, then the SSID, zero padded to 0x24 (R5.5). */
static int jedi_hw_set_ssid(struct jedi_private *priv, const u8 *ssid, u8 len)
{
	u8 b[JEDI_SSID_WIRE] = { 0 };

	if (len > JEDI_SSID_MAX)
		return -E2BIG;
	b[0] = len;
	memcpy(b + 1, ssid, len);
	jedi_dbg(priv->func, "set SSID len %u\n", len);
	return jedi_cmd(priv, JEDI_OP_SET, JEDI_OID_SSID, b, sizeof(b), NULL, 0,
			NULL);
}

/* BSSID: 6 bytes, all 0xff converted to zero (any). The firmware joins by
 * itself once SSID/BSSID are set (R7.5). */
static int jedi_hw_set_bssid(struct jedi_private *priv, const u8 *bssid)
{
	u8 b[ETH_ALEN];

	if (is_broadcast_ether_addr(bssid))
		memset(b, 0, ETH_ALEN);
	else
		memcpy(b, bssid, ETH_ALEN);
	jedi_dbg(priv->func, "set BSSID %pM\n", b);
	return jedi_cmd(priv, JEDI_OP_SET, JEDI_OID_BSSID, b, sizeof(b), NULL, 0,
			NULL);
}

/*
 * Security suite (R9.3): derived from the SIOCSIWAUTH state. The firmware
 * builds the association IEs from this code; the host runs the handshake.
 */
static u32 jedi_suite_code(struct jedi_private *priv, bool *wpa)
{
	struct jedi_wext *wx = &priv->wx;
	u32 ver = wx->auth[IW_AUTH_WPA_VERSION];
	u32 km = wx->auth[IW_AUTH_KEY_MGMT];
	bool psk = km & IW_AUTH_KEY_MGMT_PSK;

	*wpa = false;
	if (ver & IW_AUTH_WPA_VERSION_WPA2) {
		*wpa = true;
		return psk ? 9 : 8;
	}
	if (ver & IW_AUTH_WPA_VERSION_WPA) {
		*wpa = true;
		return psk ? 5 : 4;
	}
	if (wx->auth[IW_AUTH_80211_AUTH_ALG] == IW_AUTH_ALG_SHARED_KEY)
		return 2;
	return 1;
}

static void jedi_hw_set_security(struct jedi_private *priv)
{
	bool wpa;
	u32 code = jedi_suite_code(priv, &wpa);

	jedi_dbg(priv->func, "security suite %u wpa %d\n", code, wpa);
	jedi_set_u32(priv, JEDI_OID_SECURITY, code);
	jedi_set_u32(priv, JEDI_OID_SEC_MODE, wpa ? 1 : 0);
}

/* R9.3: WinCE sets the suite to open, then an empty SSID. */
static int jedi_hw_disassociate(struct jedi_private *priv)
{
	jedi_dbg(priv->func, "disassociate\n");
	jedi_set_u32(priv, JEDI_OID_SECURITY, 1);
	return jedi_hw_set_ssid_empty(priv);
}

/* After a connect trap the BSSID is read with GET 0x10000001 (R9.1). */
static void jedi_link_work(struct work_struct *work)
{
	struct jedi_private *priv = container_of(work, struct jedi_private,
						 link_work);
	u8 b[16];
	u32 rlen = 0;
	int ret;

	ret = jedi_cmd(priv, JEDI_OP_GET, JEDI_OID_BSSID, NULL, 0, b, sizeof(b),
		       &rlen);
	if (ret || rlen < ETH_ALEN) {
		dev_warn(&priv->func->dev, "jedi: GET BSSID failed (%d, %u)\n",
			 ret, rlen);
		memset(b, 0, ETH_ALEN);
		jedi_wext_link_event(priv, b);	/* still mark link up */
		return;
	}
	jedi_wext_link_event(priv, b);
}

/*
 * Traps (PIMFOR op 4), called from the SDIO IRQ thread: must not issue
 * commands (jedi_cmd waits for this very thread). Use workers.
 */
static void jedi_handle_trap(struct jedi_private *priv, u32 oid,
			     const u8 *p, u32 plen)
{
	struct sdio_func *func = priv->func;

	switch (oid) {
	case JEDI_OID_LINKSTATE: {
		/* R9.1: word0 != 0 connected (rate code), 0 disconnected,
		 * word1 = disconnect reason, +0x18 = rate * 1000. */
		u32 w0, w1, rate;

		if (plen < 8)
			break;
		w0 = get_unaligned_le32(p);
		w1 = get_unaligned_le32(p + 4);
		rate = plen >= 0x1c ? get_unaligned_le32(p + 0x18) : 0;
		dev_info(&func->dev,
			 "jedi: link %s (w0 %u, reason %u, rate %u kbps)\n",
			 w0 ? "UP" : "DOWN", w0, w1, rate);
		if (jedi_debug)
			print_hex_dump_bytes("jedi: linkstate ",
					     DUMP_PREFIX_OFFSET, p,
					     min_t(u32, plen, 0x30));
		if (w0)
			schedule_work(&priv->link_work);
		else
			jedi_wext_link_event(priv, NULL);
		break;
	}
	case JEDI_OID_SCAN_DONE_TRAP:
		jedi_dbg(func, "scan complete trap (%u)\n",
			 plen >= 4 ? get_unaligned_le32(p) : 0);
		if (plen >= 4 && get_unaligned_le32(p) == 1 &&
		    priv->wx.scan_pending) {
			cancel_delayed_work(&priv->scan_work);
			schedule_delayed_work(&priv->scan_work, 0);
		}
		break;
	case JEDI_OID_INACTIVE_STATE:
		jedi_dbg(func, "radio/inactive trap, %u bytes\n", plen);
		break;
	case JEDI_OID_MIC_FAIL_TRAP:
		/* TODO: report IWEVMICHAELMICFAILURE to the supplicant */
		dev_warn(&func->dev, "jedi: Michael MIC failure, AP %pM\n", p);
		break;
	case JEDI_OID_DEAUTH_TRAP:
	case JEDI_OID_DISASSOC_TRAP:
		jedi_dbg(func, "deauth/disassoc trap oid 0x%08x, reason %u\n", oid,
			 plen >= 0x12 ? get_unaligned_le16(p + 0x10) : 0);
		jedi_wext_link_event(priv, NULL);
		break;
	case JEDI_OID_AUTH_TRAP:
	case JEDI_OID_ASSOC_TRAP:
	case JEDI_OID_REASSOC_TRAP:
		jedi_dbg(func, "mlme trap oid 0x%08x, %u bytes\n", oid, plen);
		break;
	default:
		/* 0x16000101 stats and 0x1f000003 can be ignored (R9.5) */
		jedi_dbg(func, "trap oid 0x%08x, %u bytes\n", oid, plen);
		break;
	}
}

/* ---- handlers ---- */

static int jedi_wx_giwname(struct net_device *dev, struct iw_request_info *info,
			   union iwreq_data *wrqu, char *extra)
{
	strcpy(wrqu->name, "IEEE 802.11bg");
	return 0;
}

static int jedi_wx_siwmode(struct net_device *dev, struct iw_request_info *info,
			   union iwreq_data *wrqu, char *extra)
{
	/* Station (infrastructure) mode only for now. */
	if (wrqu->mode != IW_MODE_INFRA && wrqu->mode != IW_MODE_AUTO)
		return -EOPNOTSUPP;
	return 0;
}

static int jedi_wx_giwmode(struct net_device *dev, struct iw_request_info *info,
			   union iwreq_data *wrqu, char *extra)
{
	wrqu->mode = IW_MODE_INFRA;
	return 0;
}

static const u32 jedi_bitrates[] = {
	1000000, 2000000, 5500000, 11000000, 6000000, 9000000, 12000000,
	18000000, 24000000, 36000000, 48000000, 54000000,
};

static int jedi_wx_giwrange(struct net_device *dev, struct iw_request_info *info,
			    union iwreq_data *wrqu, char *extra)
{
	struct iw_range *range = (struct iw_range *)extra;
	int i;

	wrqu->data.length = sizeof(*range);
	memset(range, 0, sizeof(*range));

	/* wpa_supplicant ignores everything if this is below 18. */
	range->we_version_compiled = WIRELESS_EXT;
	range->we_version_source = 22;

	range->throughput = 20 * 1000 * 1000 / 8;

	/* Signal levels are dBm (supplicant reads max_qual.level as max_level). */
	range->max_qual.qual = 100;
	range->max_qual.level = 0;
	range->max_qual.noise = 0;
	range->max_qual.updated = IW_QUAL_ALL_UPDATED | IW_QUAL_DBM;
	range->avg_qual.qual = 50;
	range->avg_qual.level = 256 - 60;
	range->avg_qual.noise = 0;
	range->avg_qual.updated = IW_QUAL_ALL_UPDATED | IW_QUAL_DBM;

	/* 2.4 GHz channels 1-14 */
	for (i = 0; i < 14; i++) {
		range->freq[i].i = i + 1;
		range->freq[i].m = (i == 13) ? 2484 : 2407 + 5 * (i + 1);
		range->freq[i].e = 6;
	}
	range->num_channels = 14;
	range->num_frequency = 14;

	for (i = 0; i < ARRAY_SIZE(jedi_bitrates); i++)
		range->bitrate[i] = jedi_bitrates[i];
	range->num_bitrates = ARRAY_SIZE(jedi_bitrates);

	range->encoding_size[0] = 5;
	range->encoding_size[1] = 13;
	range->num_encoding_sizes = 2;
	range->max_encoding_tokens = 4;

	/*
	 * Note: IW_ENC_CAPA_4WAY_HANDSHAKE is deliberately NOT set. It tells
	 * wpa_supplicant to hand the PSK to the driver and let it do the 4-way
	 * handshake. Leave it off until RE says the firmware does that.
	 */
	range->enc_capa = IW_ENC_CAPA_WPA | IW_ENC_CAPA_WPA2 |
			  IW_ENC_CAPA_CIPHER_TKIP | IW_ENC_CAPA_CIPHER_CCMP;

	IW_EVENT_CAPA_SET_KERNEL(range->event_capa);
	IW_EVENT_CAPA_SET(range->event_capa, SIOCGIWAP);
	IW_EVENT_CAPA_SET(range->event_capa, SIOCGIWSCAN);

	return 0;
}

static int jedi_wx_siwessid(struct net_device *dev, struct iw_request_info *info,
			    union iwreq_data *wrqu, char *extra)
{
	struct jedi_private *priv = netdev_priv(dev);
	struct jedi_wext *wx = &priv->wx;
	u8 len = wrqu->essid.length;

	if (len > JEDI_SSID_MAX)
		return -E2BIG;

	mutex_lock(&wx->lock);
	/* flags == 0 means "any SSID" */
	wx->ssid_len = wrqu->essid.flags ? len : 0;
	memcpy(wx->ssid, extra, wx->ssid_len);
	mutex_unlock(&wx->lock);

	if (wx->ssid_len)
		jedi_hw_set_security(priv);
	return jedi_hw_set_ssid(priv, (const u8 *)extra, wx->ssid_len);
}

static int jedi_wx_giwessid(struct net_device *dev, struct iw_request_info *info,
			    union iwreq_data *wrqu, char *extra)
{
	struct jedi_private *priv = netdev_priv(dev);
	struct jedi_wext *wx = &priv->wx;

	mutex_lock(&wx->lock);
	if (wx->connected) {
		wrqu->essid.length = wx->ssid_len;
		wrqu->essid.flags = 1;
		memcpy(extra, wx->ssid, wx->ssid_len);
	} else {
		wrqu->essid.length = 0;
		wrqu->essid.flags = 0;
	}
	mutex_unlock(&wx->lock);
	return 0;
}

static int jedi_wx_siwap(struct net_device *dev, struct iw_request_info *info,
			 union iwreq_data *wrqu, char *extra)
{
	struct jedi_private *priv = netdev_priv(dev);
	struct jedi_wext *wx = &priv->wx;
	const u8 *a = (const u8 *)wrqu->ap_addr.sa_data;
	static const u8 zero[ETH_ALEN];

	if (wrqu->ap_addr.sa_family != ARPHRD_ETHER)
		return -EINVAL;

	mutex_lock(&wx->lock);
	if (!memcmp(a, zero, ETH_ALEN) || is_broadcast_ether_addr(a)) {
		wx->bssid_set = false;
		memset(wx->bssid, 0, ETH_ALEN);
	} else {
		wx->bssid_set = true;
		memcpy(wx->bssid, a, ETH_ALEN);
	}
	mutex_unlock(&wx->lock);

	/*
	 * wpa_supplicant calls SIOCSIWESSID then SIOCSIWAP last when
	 * associating, and SIOCSIWAP(0) + random SSID to disconnect.
	 * TODO(RE): decide where the actual join is triggered.
	 */
	return jedi_hw_set_bssid(priv, a);
}

static int jedi_wx_giwap(struct net_device *dev, struct iw_request_info *info,
			 union iwreq_data *wrqu, char *extra)
{
	struct jedi_private *priv = netdev_priv(dev);
	struct jedi_wext *wx = &priv->wx;

	wrqu->ap_addr.sa_family = ARPHRD_ETHER;
	mutex_lock(&wx->lock);
	if (wx->connected)
		memcpy(wrqu->ap_addr.sa_data, wx->cur_bssid, ETH_ALEN);
	else
		memset(wrqu->ap_addr.sa_data, 0, ETH_ALEN);
	mutex_unlock(&wx->lock);
	return 0;
}

static int jedi_wx_siwscan(struct net_device *dev, struct iw_request_info *info,
			   union iwreq_data *wrqu, char *extra)
{
	struct jedi_private *priv = netdev_priv(dev);
	const u8 *ssid = NULL;
	u8 ssid_len = 0;

	if (wrqu->data.length == sizeof(struct iw_scan_req) &&
	    (wrqu->data.flags & IW_SCAN_THIS_ESSID)) {
		struct iw_scan_req *req = (struct iw_scan_req *)extra;

		if (req->essid_len <= JEDI_SSID_MAX) {
			ssid = req->essid;
			ssid_len = req->essid_len;
		}
	}
	return jedi_hw_scan(priv, ssid, ssid_len);
}

static int jedi_wx_giwscan(struct net_device *dev, struct iw_request_info *info,
			   union iwreq_data *wrqu, char *extra)
{
	struct jedi_private *priv = netdev_priv(dev);
	struct jedi_wext *wx = &priv->wx;
	char *ev = extra, *end = extra + wrqu->data.length;
	struct iw_event iwe;
	int i, ret = 0;

	mutex_lock(&wx->lock);
	/* Results are not ready yet: userspace (iwlist) waits and retries. */
	if (wx->scan_pending) {
		mutex_unlock(&wx->lock);
		return -EAGAIN;
	}
	for (i = 0; i < wx->nbss; i++) {
		struct jedi_bss *b = &wx->bss[i];

		/* generous margin for the fixed-size events of one entry */
		if (end - ev < 200 + b->ssid_len + b->ie_len) {
			ret = -E2BIG;
			break;
		}

		memset(&iwe, 0, sizeof(iwe));
		iwe.cmd = SIOCGIWAP;
		iwe.u.ap_addr.sa_family = ARPHRD_ETHER;
		memcpy(iwe.u.ap_addr.sa_data, b->bssid, ETH_ALEN);
		ev = iwe_stream_add_event(info, ev, end, &iwe, IW_EV_ADDR_LEN);

		memset(&iwe, 0, sizeof(iwe));
		iwe.cmd = SIOCGIWESSID;
		iwe.u.data.length = b->ssid_len;
		iwe.u.data.flags = 1;
		ev = iwe_stream_add_point(info, ev, end, &iwe, (char *)b->ssid);

		memset(&iwe, 0, sizeof(iwe));
		iwe.cmd = SIOCGIWMODE;
		iwe.u.mode = (b->caps & 0x0002) ? IW_MODE_ADHOC : IW_MODE_MASTER;
		ev = iwe_stream_add_event(info, ev, end, &iwe, IW_EV_UINT_LEN);

		memset(&iwe, 0, sizeof(iwe));
		iwe.cmd = SIOCGIWFREQ;
		iwe.u.freq.m = b->freq;
		iwe.u.freq.e = 6;
		ev = iwe_stream_add_event(info, ev, end, &iwe, IW_EV_FREQ_LEN);

		memset(&iwe, 0, sizeof(iwe));
		iwe.cmd = IWEVQUAL;
		iwe.u.qual.level = (u8)b->level;
		iwe.u.qual.updated = IW_QUAL_QUAL_INVALID | IW_QUAL_NOISE_INVALID |
				     IW_QUAL_DBM;
		ev = iwe_stream_add_event(info, ev, end, &iwe, IW_EV_QUAL_LEN);

		memset(&iwe, 0, sizeof(iwe));
		iwe.cmd = SIOCGIWENCODE;
		iwe.u.data.flags = (b->caps & 0x0010) ?
			(IW_ENCODE_ENABLED | IW_ENCODE_NOKEY) : IW_ENCODE_DISABLED;
		ev = iwe_stream_add_point(info, ev, end, &iwe, (char *)b->ssid);

		if (b->ie_len) {
			memset(&iwe, 0, sizeof(iwe));
			iwe.cmd = IWEVGENIE;
			iwe.u.data.length = b->ie_len;
			ev = iwe_stream_add_point(info, ev, end, &iwe, (char *)b->ie);
		}
	}
	mutex_unlock(&wx->lock);

	wrqu->data.length = ev - extra;
	wrqu->data.flags = 0;
	return ret;
}

static int jedi_wx_siwauth(struct net_device *dev, struct iw_request_info *info,
			   union iwreq_data *wrqu, char *extra)
{
	struct jedi_private *priv = netdev_priv(dev);
	int idx = wrqu->param.flags & IW_AUTH_INDEX;

	switch (idx) {
	case IW_AUTH_WPA_VERSION:
	case IW_AUTH_CIPHER_PAIRWISE:
	case IW_AUTH_CIPHER_GROUP:
	case IW_AUTH_KEY_MGMT:
	case IW_AUTH_TKIP_COUNTERMEASURES:
	case IW_AUTH_DROP_UNENCRYPTED:
	case IW_AUTH_80211_AUTH_ALG:
	case IW_AUTH_WPA_ENABLED:
	case IW_AUTH_RX_UNENCRYPTED_EAPOL:
	case IW_AUTH_ROAMING_CONTROL:
	case IW_AUTH_PRIVACY_INVOKED:
		priv->wx.auth[idx] = wrqu->param.value;
		/* TODO(RE): push to firmware where it matters (cipher, auth alg) */
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static int jedi_wx_giwauth(struct net_device *dev, struct iw_request_info *info,
			   union iwreq_data *wrqu, char *extra)
{
	struct jedi_private *priv = netdev_priv(dev);
	int idx = wrqu->param.flags & IW_AUTH_INDEX;

	if (idx > IW_AUTH_MFP)
		return -EOPNOTSUPP;
	wrqu->param.value = priv->wx.auth[idx];
	return 0;
}

static int jedi_wx_siwgenie(struct net_device *dev, struct iw_request_info *info,
			    union iwreq_data *wrqu, char *extra)
{
	struct jedi_private *priv = netdev_priv(dev);
	struct jedi_wext *wx = &priv->wx;

	if (wrqu->data.length > JEDI_MAX_IE)
		return -E2BIG;
	mutex_lock(&wx->lock);
	wx->gen_ie_len = wrqu->data.length;
	if (wx->gen_ie_len)
		memcpy(wx->gen_ie, extra, wx->gen_ie_len);
	mutex_unlock(&wx->lock);
	return 0;
}

static int jedi_wx_giwgenie(struct net_device *dev, struct iw_request_info *info,
			    union iwreq_data *wrqu, char *extra)
{
	struct jedi_private *priv = netdev_priv(dev);
	struct jedi_wext *wx = &priv->wx;
	int ret = 0;

	mutex_lock(&wx->lock);
	if (wrqu->data.length < wx->gen_ie_len) {
		ret = -E2BIG;
	} else {
		wrqu->data.length = wx->gen_ie_len;
		memcpy(extra, wx->gen_ie, wx->gen_ie_len);
	}
	mutex_unlock(&wx->lock);
	return ret;
}

/*
 * Keys. TODO(RE): station key OIDs 0x12000008 (0x2c bytes) / 0x1200000a
 * (0x40 bytes), default WEP keys (0x14 bytes). Only "disable" (key clear,
 * which wpa_supplicant does at startup) is accepted until then.
 */
static int jedi_wx_siwencode(struct net_device *dev, struct iw_request_info *info,
			     union iwreq_data *wrqu, char *extra)
{
	if (wrqu->encoding.flags & IW_ENCODE_DISABLED)
		return 0;
	return -EOPNOTSUPP;
}

static int jedi_wx_giwencode(struct net_device *dev, struct iw_request_info *info,
			     union iwreq_data *wrqu, char *extra)
{
	wrqu->encoding.length = 0;
	wrqu->encoding.flags = IW_ENCODE_DISABLED;
	return 0;
}

static int jedi_install_wep(struct jedi_private *priv, int idx,
			    const u8 *key, int len, bool tx)
{
	u8 b[20] = { 0 };
	int ret;

	if (idx < 0 || idx > 3 || (len != 5 && len != 13))
		return -EINVAL;
	b[0] = 0;		/* type: WEP */
	b[1] = len;
	memcpy(b + 2, key, len);
	ret = jedi_cmd(priv, JEDI_OP_SET, JEDI_OID_WEP_SLOT0 + idx, b,
		       sizeof(b), NULL, 0, NULL);
	if (!ret && tx)
		ret = jedi_set_u32(priv, JEDI_OID_WEP_DEFKEY, idx);
	return ret;
}

/*
 * TKIP/CCMP key (R9.3). TODO(RE): cipher type byte values and the u16 at
 * offset 6 are guesses (TKIP 1, CCMP 2, u16 0); verify on a real association.
 */
static int jedi_install_sta_key(struct jedi_private *priv, const u8 *mac,
				int idx, u8 type, const u8 *key, int len,
				bool tx, const u8 *rsc)
{
	u8 b[64] = { 0 };

	if (len > 32)
		return -EINVAL;
	memcpy(b, mac, ETH_ALEN);
	/* b[6..7]: unknown u16 */
	put_unaligned_le16(idx, b + 8);
	b[10] = type;
	b[11] = len;
	memcpy(b + 12, key, len);

	if (rsc) {
		u8 *r = b + 44;

		memcpy(r, mac, ETH_ALEN);
		r[8] = idx;
		r[9] = tx;
		memcpy(r + 10, rsc, 6);
		return jedi_cmd(priv, JEDI_OP_SET, JEDI_OID_STA_KEY_RSC, b, 64,
				NULL, 0, NULL);
	}
	return jedi_cmd(priv, JEDI_OP_SET, JEDI_OID_STA_KEY, b, 44, NULL, 0,
			NULL);
}

static int jedi_wx_siwencodeext(struct net_device *dev,
				struct iw_request_info *info,
				union iwreq_data *wrqu, char *extra)
{
	struct jedi_private *priv = netdev_priv(dev);
	struct iw_encode_ext *ext = (struct iw_encode_ext *)extra;
	int idx = (wrqu->encoding.flags & IW_ENCODE_INDEX);
	int keylen = (int)wrqu->encoding.length - (int)sizeof(*ext);
	bool tx = ext->ext_flags & IW_ENCODE_EXT_SET_TX_KEY;
	bool group = ext->ext_flags & IW_ENCODE_EXT_GROUP_KEY;
	const u8 *mac;
	u8 type;

	idx = idx ? idx - 1 : 0;

	if ((wrqu->encoding.flags & IW_ENCODE_DISABLED) ||
	    ext->alg == IW_ENCODE_ALG_NONE)
		return 0;	/* TODO(RE): no key-remove command known */
	if (keylen <= 0)
		return -EINVAL;

	jedi_dbg(priv->func, "set key: alg %u idx %d len %d ext_flags 0x%x\n",
		 ext->alg, idx, keylen, ext->ext_flags);

	switch (ext->alg) {
	case IW_ENCODE_ALG_WEP:
		return jedi_install_wep(priv, idx, ext->key, keylen, tx);
	case IW_ENCODE_ALG_TKIP:
		type = 1;	/* TODO(RE) */
		break;
	case IW_ENCODE_ALG_CCMP:
		type = 2;	/* TODO(RE) */
		break;
	default:
		return -EOPNOTSUPP;
	}

	/* Group keys are keyed by the BSSID, pairwise by the peer address. */
	mac = (group || is_zero_ether_addr((u8 *)ext->addr.sa_data) ||
	       is_broadcast_ether_addr((u8 *)ext->addr.sa_data)) ?
	      priv->wx.cur_bssid : (u8 *)ext->addr.sa_data;
	return jedi_install_sta_key(priv, mac, idx, type, ext->key, keylen, tx,
		(ext->ext_flags & IW_ENCODE_EXT_RX_SEQ_VALID) ? ext->rx_seq : NULL);
}

static int jedi_wx_siwmlme(struct net_device *dev, struct iw_request_info *info,
			   union iwreq_data *wrqu, char *extra)
{
	struct jedi_private *priv = netdev_priv(dev);
	struct iw_mlme *mlme = (struct iw_mlme *)extra;

	switch (mlme->cmd) {
	case IW_MLME_DEAUTH:
	case IW_MLME_DISASSOC:
		return jedi_hw_disassociate(priv);
	default:
		return -EOPNOTSUPP;
	}
}

static int jedi_wx_siwpmksa(struct net_device *dev, struct iw_request_info *info,
			    union iwreq_data *wrqu, char *extra)
{
	struct iw_pmksa *pmksa = (struct iw_pmksa *)extra;

	/* Supplicant flushes the PMKSA cache at startup. */
	if (pmksa->cmd == IW_PMKSA_FLUSH)
		return 0;
	return -EOPNOTSUPP;
}

static struct iw_statistics *jedi_wx_get_stats(struct net_device *dev)
{
	struct jedi_private *priv = netdev_priv(dev);

	priv->wx.stats.qual.updated = IW_QUAL_ALL_INVALID;
	return &priv->wx.stats;
}

static const iw_handler jedi_wx_handlers[] = {
	IW_HANDLER(SIOCGIWNAME,		jedi_wx_giwname),
	IW_HANDLER(SIOCSIWMODE,		jedi_wx_siwmode),
	IW_HANDLER(SIOCGIWMODE,		jedi_wx_giwmode),
	IW_HANDLER(SIOCGIWRANGE,	jedi_wx_giwrange),
	IW_HANDLER(SIOCSIWAP,		jedi_wx_siwap),
	IW_HANDLER(SIOCGIWAP,		jedi_wx_giwap),
	IW_HANDLER(SIOCSIWMLME,		jedi_wx_siwmlme),
	IW_HANDLER(SIOCSIWSCAN,		jedi_wx_siwscan),
	IW_HANDLER(SIOCGIWSCAN,		jedi_wx_giwscan),
	IW_HANDLER(SIOCSIWESSID,	jedi_wx_siwessid),
	IW_HANDLER(SIOCGIWESSID,	jedi_wx_giwessid),
	IW_HANDLER(SIOCSIWENCODE,	jedi_wx_siwencode),
	IW_HANDLER(SIOCGIWENCODE,	jedi_wx_giwencode),
	IW_HANDLER(SIOCSIWGENIE,	jedi_wx_siwgenie),
	IW_HANDLER(SIOCGIWGENIE,	jedi_wx_giwgenie),
	IW_HANDLER(SIOCSIWAUTH,		jedi_wx_siwauth),
	IW_HANDLER(SIOCGIWAUTH,		jedi_wx_giwauth),
	IW_HANDLER(SIOCSIWENCODEEXT,	jedi_wx_siwencodeext),
	IW_HANDLER(SIOCSIWPMKSA,	jedi_wx_siwpmksa),
};

static const struct iw_handler_def jedi_wext_handler_def = {
	.standard		= jedi_wx_handlers,
	.num_standard		= ARRAY_SIZE(jedi_wx_handlers),
	.get_wireless_stats	= jedi_wx_get_stats,
};

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
	INIT_WORK(&priv->link_work, jedi_link_work);
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
			"symbol_jedi: failed to enable SDIO function: %d\n",
			ret);
		sdio_release_host(func);
		goto err_free_dev;
	}

	/* WinCE sets block size 0x200 */
	ret = sdio_set_block_size(func, 512);
	if (ret) {
		dev_warn(&func->dev,
			 "symbol_jedi: unable to set block size to 512 (%d), "
			 "trying 64\n", ret);
		sdio_set_block_size(func, 64);
	}

	if (jedi_debug)
		jedi_regtest(func);

	sdio_release_host(func);

	/*
	 * Reset the chip into its boot ROM, upload the image, and wait for
	 * FW Init. jedi_fw_download() claims the host itself.
	 */
	ret = jedi_fw_download(func);
	if (ret) {
		dev_err(&func->dev,
			"symbol_jedi: firmware download failed: %d\n", ret);
		goto err_disable_func;
	}

	/* Firmware is running: now it is safe to take its interrupt. */
	sdio_claim_host(func);
	ret = sdio_claim_irq(func, jedi_sdio_irq);
	sdio_release_host(func);
	if (ret) {
		dev_err(&func->dev,
			"symbol_jedi: failed to claim SDIO IRQ: %d\n", ret);
		goto err_disable_func;
	}
	priv->irq_claimed = true;

	/* First command (R6.6): ask the firmware for the MAC address. */
	{
		u8 mac[ETH_ALEN] = { 0 };
		u32 got = 0;

		ret = jedi_cmd(priv, JEDI_OP_GET, JEDI_OID_MAC, NULL, ETH_ALEN,
			       mac, sizeof(mac), &got);
		if (ret || got != ETH_ALEN) {
			dev_err(&func->dev,
				"jedi: GET MAC failed: %d (%u bytes), using a "
				"random address\n", ret, got);
			random_ether_addr(ndev->dev_addr);
		} else {
			memcpy(ndev->dev_addr, mac, ETH_ALEN);
			dev_info(&func->dev, "jedi: MAC address %pM\n", mac);
		}
	}

	jedi_hw_init(priv);

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
		kfree(priv->cmd_out);

		sdio_claim_host(func);
		if (priv->irq_claimed)
			sdio_release_irq(func);
		sdio_disable_func(func);
		sdio_release_host(func);

		sdio_set_drvdata(func, NULL);
		destroy_workqueue(priv->wq);
		kfree(priv->bounce);
		kfree(priv->rx_buf);
		free_netdev(ndev);	/* frees priv too */
	}
	pr_info("symbol_jedi: Removed\n");
}

static struct sdio_driver jedi_driver = {
	.name     = "symbol_jedi",
	.id_table = jedi_ids,
	.probe    = jedi_probe,
	.remove   = jedi_remove,
};

static int __init jedi_init(void)
{
	return sdio_register_driver(&jedi_driver);
}

static void __exit jedi_exit(void)
{
	sdio_unregister_driver(&jedi_driver);
}

module_init(jedi_init);
module_exit(jedi_exit);

MODULE_AUTHOR("Mark Kennard <markkennard4@gmail.com>");
MODULE_DESCRIPTION("Symbol Jedi SDIO Wi-Fi Driver");
MODULE_LICENSE("GPL");