/*
 * jedi_priv.h -- Private definitions shared by jedi_core.c, jedi_hw.c and jedi_wext.c
 *
 * By Mark Kennard <markkennard4@gmail.com>
 *
 * GPL v2, see jedi_core.c.
 */

#ifndef SYMBOL_JEDI_PRIV_H
#define SYMBOL_JEDI_PRIV_H

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
#include <linux/firmware.h>
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
#define JEDI_REG_TX_LEN		0x50		/* W16: len | 0xc000; R: ready */
#define JEDI_TX_READY		0x4000		/* ready bit when reading 0x50 */
/* WinCE trace findings: the length write always has bits 14 AND 15 set
 * (0xc000), not just bit 14. Our old 0x4000-only write was a bug. */
#define JEDI_TX_LEN_FLAGS	0xc000
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
#define JEDI_MAX_IE		256	/* SIOCSIWGENIE */
#define JEDI_MAX_BSS_IE		768	/* scan result IEs: busy beacons exceed 255 */
#define JEDI_SSID_MAX		32

struct jedi_bss {
	u8 bssid[ETH_ALEN];
	u8 ssid[JEDI_SSID_MAX];
	u8 ssid_len;
	u16 freq;			/* MHz */
	s8 level;			/* dBm */
	u16 caps;			/* 802.11 capability info */
	bool wmm;			/* advertises a WMM vendor IE */
	u16 ie_len;
	u8 ie[JEDI_MAX_BSS_IE];		/* concatenated WPA/RSN/etc IEs */
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
	u32 rate_kbps;			/* from the link-state trap */
	u32 rts, frag;			/* last values written */
	u8 req_ie[JEDI_MAX_IE];		/* assoc request IEs (0x18000003 trap) */
	u8 req_ie_len;
	u8 resp_ie[JEDI_MAX_IE];	/* assoc response IEs */
	u8 resp_ie_len;
	bool ptk_set, gtk_set;		/* keys installed since the join */
	u32 suite_sent;			/* last 0x1200000b value (0 = unknown) */
	struct {			/* last installed key per index, for removal */
		bool valid;
		u8 type, len;
		u8 mac[ETH_ALEN];
	} key[4];
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
	spinlock_t eapol_lock;		/* eapol_hold / eapol_ok */
	struct sk_buff *eapol_hold;	/* AP message 1 that beat the AP event */
	struct completion country_done;	/* 0x1f000003 trap seen */
	struct delayed_work diag_work;
	int diag_n;
	bool diag_dump;		/* dump the next 0x16000101 trap in full */
	unsigned long irq_cnt, irq_rx_cnt, rx_zero_cnt;
	bool port_closed;		/* WPA join: only EAPOL may be sent */
	bool eapol_ok;			/* supplicant has been told we are associated */
	struct work_struct clearkeys_work; /* wipe stale keys after a link loss */
	struct delayed_work join_work;	/* BSSID, then SSID (starts the join) */
	char *cmd_out;		/* sysfs 'cmd' result */
	bool stat_valid;		/* last 0x16000101 counters */
	u32 stat_a, stat_b;
	bool hw_failed;
	unsigned int tx_stalls;		/* consecutive TX timeouts */
	unsigned int data_traced;	/* data frames logged so far */
	unsigned long join_jiffies;	/* when the last join was started */			/* init step failed: reset and retry */
	unsigned int tx_traced;		/* TX frames logged so far */
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
#define JEDI_OID_PORT_AUTH	0x150007e4	/* 8 bytes: BSSID + 2 zero; after PTK+GTK */
#define JEDI_OID_SEC_MODE	0x12000012	/* driver security mode */
#define JEDI_OID_COUNTRY_LIST	0x1f000001
#define JEDI_OID_COUNTRY	0x1f000002
#define JEDI_OID_DOT11D_MODE	0x1f000004
#define JEDI_OID_REG_BEGIN	0x00000010
#define JEDI_OID_REG_TABLE	0x1f00000e
#define JEDI_OID_REG_DONE	0x02000000
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
#define JEDI_SCAN_WAIT_MS	4000	/* WinCE waits ~4 s (trace findings 9) */

#define JEDI_TXQ_MAX		64
#define JEDI_TXQ_LOW		16
#define JEDI_TX_TIMEOUT		(5 * HZ)

/* Module parameters (defined in jedi_core.c) */
extern bool jedi_reg_test;
extern char *jedi_country;
extern bool jedi_rebuild;
extern unsigned int jedi_cmd_delay;
extern unsigned int jedi_suite_override;
extern unsigned int jedi_join_mode;
extern unsigned int jedi_join_settle_ms;
extern bool jedi_dot11d;
extern bool jedi_send_privacy;
extern unsigned int jedi_port_oid;
extern bool jedi_bssid_lock;
extern bool jedi_tx_gate;
extern bool jedi_key_dump;
extern unsigned int jedi_country_wait_ms;
extern unsigned int jedi_rsc_mode;
extern unsigned int jedi_key_remove_mode;
int jedi_key_remove(struct jedi_private *priv, const u8 *mac, int idx);
extern int jedi_wmm_mode;
extern bool jedi_send_suite, jedi_dummy_keys;
extern unsigned int jedi_data_trace;
extern bool jedi_link_fallback;
extern bool jedi_ps_kick;
extern unsigned int jedi_recover_max;
bool jedi_request_recovery(struct sdio_func *func, const char *why);
/* Board hook (arch/arm/mach-pxa/mc3190.c); weak so the module still loads
 * where it does not exist. */
extern void mc3190_wifi_power_cycle(void) __attribute__((weak));
extern void mc3190_wifi_power_on(void) __attribute__((weak));
extern void mc3190_wifi_power_off(void) __attribute__((weak));
void jedi_trace_data(struct jedi_private *priv, const char *dir,
		     const u8 *buf, u32 len);
extern unsigned int jedi_join_poll_ms;
extern bool jedi_uart;
extern unsigned int jedi_tx_gap_us;
extern unsigned int jedi_tx_retry;
extern bool jedi_tx_trace;
extern unsigned int jedi_init_retries;


/* jedi_core.c: transport, netdev, probe */
int jedi_cmd(struct jedi_private *priv, u8 op, u32 oid, const void *payload, u32 plen, void *reply, u32 reply_max, u32 *reply_len);
void jedi_rx_frame(struct jedi_private *priv, const u8 *buf, u32 len);

/* jedi_hw.c: firmware operations, join, traps */
void jedi_handle_trap(struct jedi_private *priv, u32 oid, const u8 *p, u32 plen);
int jedi_hw_disassociate(struct jedi_private *priv);
void jedi_hw_init(struct jedi_private *priv);
void jedi_hw_shutdown(struct jedi_private *priv);
void jedi_hw_port_authorize(struct jedi_private *priv, bool authorized);
void jedi_diag_start(struct jedi_private *priv);
int jedi_hw_scan(struct jedi_private *priv, const u8 *ssid, u8 ssid_len);
int jedi_hw_set_bssid(struct jedi_private *priv, const u8 *bssid);
int jedi_hw_set_ssid(struct jedi_private *priv, const u8 *ssid, u8 len);
void jedi_join_work(struct work_struct *work);
void jedi_link_work(struct work_struct *work);
void jedi_clearkeys_work(struct work_struct *work);
void jedi_eapol_release(struct jedi_private *priv, bool ok);
extern unsigned int jedi_ptk_delay_ms;
void jedi_scan_work(struct work_struct *work);
int jedi_set_u32(struct jedi_private *priv, u32 oid, u32 val);

/* jedi_wext.c: wireless extensions */
bool jedi_bss_wmm(struct jedi_private *priv, const u8 *ssid, u8 ssid_len);
void jedi_bss_set_wmm(struct jedi_private *priv, const u8 *bssid, bool wmm);
int jedi_wext_bss_add(struct jedi_private *priv, const u8 *bssid, const u8 *ssid, u8 ssid_len, u16 freq, s8 level, u16 caps, const u8 *ie, u16 ie_len);
void jedi_wext_bss_clear(struct jedi_private *priv);
void jedi_wext_link_event(struct jedi_private *priv, const u8 *bssid);
void jedi_wext_mic_failure(struct jedi_private *priv, const u8 *src);
void jedi_wext_scan_done(struct jedi_private *priv);
extern const struct iw_handler_def jedi_wext_handler_def;

#endif /* SYMBOL_JEDI_PRIV_H */