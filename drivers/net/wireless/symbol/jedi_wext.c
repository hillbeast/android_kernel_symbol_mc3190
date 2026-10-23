/*
 * jedi_wext.c -- Jedi wireless extensions handlers
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

#include "jedi_priv.h"


/* ------------------------------------------------------------------ */
/* Wireless extensions                                                 */
/* ------------------------------------------------------------------ */

/*
 * Event entry points: call these from jedi_rx_frame() when the firmware
 * traps are understood.
 */

/* Scan finished: userspace then reads results with SIOCGIWSCAN. */
void jedi_wext_scan_done(struct jedi_private *priv)
{
	union iwreq_data wrqu;

	memset(&wrqu, 0, sizeof(wrqu));
	wireless_send_event(priv->ndev, SIOCGIWSCAN, &wrqu, NULL);
}

/* Association up (bssid != NULL) or down (bssid == NULL). */
void jedi_wext_link_event(struct jedi_private *priv, const u8 *bssid)
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

/*
 * Tell the supplicant about a TKIP MIC failure. The trap does not say whether
 * the frame was unicast or group, or which key index; report pairwise/key 0
 * (the supplicant counts failures either way). TODO: refine once known.
 */
void jedi_wext_mic_failure(struct jedi_private *priv, const u8 *src)
{
	struct iw_michaelmicfailure ev;
	union iwreq_data wrqu;

	memset(&ev, 0, sizeof(ev));
	ev.flags = IW_MICFAILURE_PAIRWISE;
	ev.src_addr.sa_family = ARPHRD_ETHER;
	if (src)
		memcpy(ev.src_addr.sa_data, src, ETH_ALEN);
	memset(&wrqu, 0, sizeof(wrqu));
	wrqu.data.length = sizeof(ev);
	wireless_send_event(priv->ndev, IWEVMICHAELMICFAILURE, &wrqu,
			    (char *)&ev);
}

void jedi_wext_bss_clear(struct jedi_private *priv)
{
	mutex_lock(&priv->wx.lock);
	priv->wx.nbss = 0;
	mutex_unlock(&priv->wx.lock);
}

/* Add or update a scan result. freq in MHz, level in dBm. */
int jedi_wext_bss_add(struct jedi_private *priv,
		const u8 *bssid, const u8 *ssid, u8 ssid_len, u16 freq,
		s8 level, u16 caps, const u8 *ie, u16 ie_len)
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

	if (wx->ssid_len) {
		/* the join itself happens in jedi_join_work */
		schedule_delayed_work(&priv->join_work,
				      msecs_to_jiffies(150));
		return 0;
	}
	cancel_delayed_work_sync(&priv->join_work);
	return jedi_hw_set_ssid(priv, (const u8 *)extra, 0);
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
	 * wpa_supplicant calls SIOCSIWESSID then SIOCSIWAP when associating.
	 * A real BSSID is sent by jedi_join_work (before the SSID); clearing it
	 * is sent straight away.
	 */
	if (!wx->bssid_set)
		return jedi_hw_set_bssid(priv, zero);
	if (wx->ssid_len)
		schedule_delayed_work(&priv->join_work, msecs_to_jiffies(150));
	return 0;
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
 * TKIP/CCMP key (R9.3, R12.6). Type byte: WEP 0, TKIP 1, CCMP 5. The u16 at
 * offset 6 (from ctx+0x1c8 in WinCE) is not decoded; 0 is sent.
 */
#define JEDI_KEY_TKIP	1
#define JEDI_KEY_CCMP	5
static int jedi_install_sta_key(struct jedi_private *priv, const u8 *mac,
				int idx, u8 type, const u8 *key, int len,
				bool tx, const u8 *rsc)
{
	u8 b[64] = { 0 };

	if (len > 32)
		return -EINVAL;
	if (idx >= 0 && idx < 4 && type != 0) {
		priv->wx.key[idx].valid = true;
		priv->wx.key[idx].type = type;
		priv->wx.key[idx].len = len;
		memcpy(priv->wx.key[idx].mac, mac, ETH_ALEN);
	}
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
	u8 tkbuf[32];
	int ret;

	idx = idx ? idx - 1 : 0;

	if ((wrqu->encoding.flags & IW_ENCODE_DISABLED) ||
	    ext->alg == IW_ENCODE_ALG_NONE) {
		/* R12.7: no remove command decoded; overwrite with a zero key
		 * of the same type and length (stand-in). */
		struct jedi_wext *wx = &priv->wx;

		if (idx < 4 && wx->key[idx].valid) {
			u8 zero[32] = { 0 };
			u8 mac[ETH_ALEN];

			int ret;

			memcpy(mac, wx->key[idx].mac, ETH_ALEN);
			ret = jedi_install_sta_key(priv, mac, idx,
						   wx->key[idx].type, zero,
						   wx->key[idx].len, false,
						   NULL);
			wx->key[idx].valid = false;
			return ret;
		}
		return 0;
	}
	if (keylen <= 0 || keylen > 32)
		return -EINVAL;

	jedi_dbg(priv->func, "set key: alg %u idx %d len %d ext_flags 0x%x\n",
		 ext->alg, idx, keylen, ext->ext_flags);

	switch (ext->alg) {
	case IW_ENCODE_ALG_WEP:
		return jedi_install_wep(priv, idx, ext->key, keylen, tx);
	case IW_ENCODE_ALG_TKIP:
		type = JEDI_KEY_TKIP;
		break;
	case IW_ENCODE_ALG_CCMP:
		type = JEDI_KEY_CCMP;
		break;
	default:
		return -EOPNOTSUPP;
	}

	/*
	 * R12.6: for a supplicant WinCE swaps the two 8-byte Michael MIC keys
	 * of a 32-byte TKIP key before sending (the firmware expects the
	 * opposite order from NDIS/WEXT). TODO: verify with a real handshake.
	 */
	if (type == JEDI_KEY_TKIP && keylen == 32) {
		u8 sw[32];

		memcpy(sw, ext->key, 16);
		memcpy(sw + 16, ext->key + 24, 8);
		memcpy(sw + 24, ext->key + 16, 8);
		memcpy(tkbuf, sw, 32);
	} else {
		memcpy(tkbuf, ext->key, keylen);
	}

	/* Group keys are keyed by the BSSID, pairwise by the peer address. */
	mac = (group || is_zero_ether_addr((u8 *)ext->addr.sa_data) ||
	       is_broadcast_ether_addr((u8 *)ext->addr.sa_data)) ?
	      priv->wx.cur_bssid : (u8 *)ext->addr.sa_data;
	ret = jedi_install_sta_key(priv, mac, idx, type, tkbuf, keylen, tx,
		(ext->ext_flags & IW_ENCODE_EXT_RX_SEQ_VALID) ? ext->rx_seq : NULL);
	if (!ret && !group)
		jedi_hw_port_authorize(priv, true);	/* TODO(RE Q4) */
	return ret;
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

/*
 * wpa_supplicant (WEXT) calls SIOCSIWFREQ first in every association and
 * counts "not supported" as a failed association request. The firmware picks
 * the channel itself from the SSID/BSSID, so accept and ignore it.
 */
static int jedi_wx_siwfreq(struct net_device *dev, struct iw_request_info *info,
			   union iwreq_data *wrqu, char *extra)
{
	return 0;
}

static int jedi_wx_giwfreq(struct net_device *dev, struct iw_request_info *info,
			   union iwreq_data *wrqu, char *extra)
{
	struct jedi_private *priv = netdev_priv(dev);
	struct jedi_wext *wx = &priv->wx;
	int i;

	memset(&wrqu->freq, 0, sizeof(wrqu->freq));
	mutex_lock(&wx->lock);
	if (wx->connected) {
		for (i = 0; i < wx->nbss; i++) {
			if (!memcmp(wx->bss[i].bssid, wx->cur_bssid, ETH_ALEN)) {
				wrqu->freq.m = wx->bss[i].freq;
				wrqu->freq.e = 6;
				break;
			}
		}
	}
	mutex_unlock(&wx->lock);
	return 0;
}

static int jedi_wx_giwrate(struct net_device *dev, struct iw_request_info *info,
			   union iwreq_data *wrqu, char *extra)
{
	struct jedi_private *priv = netdev_priv(dev);

	memset(&wrqu->bitrate, 0, sizeof(wrqu->bitrate));
	mutex_lock(&priv->wx.lock);
	wrqu->bitrate.value = priv->wx.rate_kbps * 1000;
	mutex_unlock(&priv->wx.lock);
	wrqu->bitrate.fixed = 0;
	return 0;
}

static int jedi_wx_giwsens(struct net_device *dev, struct iw_request_info *info,
			   union iwreq_data *wrqu, char *extra)
{
	memset(&wrqu->sens, 0, sizeof(wrqu->sens));
	wrqu->sens.disabled = 1;
	return 0;
}

/* Power save is not wired up yet; report it off, accept and ignore sets. */
static int jedi_wx_siwpower(struct net_device *dev, struct iw_request_info *info,
			    union iwreq_data *wrqu, char *extra)
{
	return 0;
}

static int jedi_wx_giwpower(struct net_device *dev, struct iw_request_info *info,
			    union iwreq_data *wrqu, char *extra)
{
	memset(&wrqu->power, 0, sizeof(wrqu->power));
	wrqu->power.disabled = 1;
	return 0;
}

static int jedi_wx_giwtxpow(struct net_device *dev, struct iw_request_info *info,
			    union iwreq_data *wrqu, char *extra)
{
	memset(&wrqu->txpower, 0, sizeof(wrqu->txpower));
	wrqu->txpower.disabled = 1;	/* TODO: GET the TX power OID */
	return 0;
}

static int jedi_wx_siwrts(struct net_device *dev, struct iw_request_info *info,
			  union iwreq_data *wrqu, char *extra)
{
	struct jedi_private *priv = netdev_priv(dev);
	u32 v = wrqu->rts.disabled ? 2347 : wrqu->rts.value;
	int ret;

	if (v > 2347)
		return -EINVAL;
	ret = jedi_set_u32(priv, JEDI_OID_RTS_THR, v);
	if (!ret)
		priv->wx.rts = v;
	return ret;
}

static int jedi_wx_giwrts(struct net_device *dev, struct iw_request_info *info,
			  union iwreq_data *wrqu, char *extra)
{
	struct jedi_private *priv = netdev_priv(dev);

	wrqu->rts.value = priv->wx.rts ? priv->wx.rts : 2347;
	wrqu->rts.disabled = wrqu->rts.value >= 2347;
	wrqu->rts.fixed = 1;
	return 0;
}

static int jedi_wx_siwfrag(struct net_device *dev, struct iw_request_info *info,
			   union iwreq_data *wrqu, char *extra)
{
	struct jedi_private *priv = netdev_priv(dev);
	u32 v = wrqu->frag.disabled ? 2346 : wrqu->frag.value;
	int ret;

	if (v < 256 || v > 2346)
		return -EINVAL;
	ret = jedi_set_u32(priv, JEDI_OID_FRAG_THR, v);
	if (!ret)
		priv->wx.frag = v;
	return ret;
}

static int jedi_wx_giwfrag(struct net_device *dev, struct iw_request_info *info,
			   union iwreq_data *wrqu, char *extra)
{
	struct jedi_private *priv = netdev_priv(dev);

	wrqu->frag.value = priv->wx.frag ? priv->wx.frag : 2346;
	wrqu->frag.disabled = wrqu->frag.value >= 2346;
	wrqu->frag.fixed = 1;
	return 0;
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
	IW_HANDLER(SIOCSIWFREQ,		jedi_wx_siwfreq),
	IW_HANDLER(SIOCGIWFREQ,		jedi_wx_giwfreq),
	IW_HANDLER(SIOCGIWRATE,		jedi_wx_giwrate),
	IW_HANDLER(SIOCGIWSENS,		jedi_wx_giwsens),
	IW_HANDLER(SIOCSIWPOWER,	jedi_wx_siwpower),
	IW_HANDLER(SIOCGIWPOWER,	jedi_wx_giwpower),
	IW_HANDLER(SIOCGIWTXPOW,	jedi_wx_giwtxpow),
	IW_HANDLER(SIOCSIWRTS,		jedi_wx_siwrts),
	IW_HANDLER(SIOCGIWRTS,		jedi_wx_giwrts),
	IW_HANDLER(SIOCSIWFRAG,		jedi_wx_siwfrag),
	IW_HANDLER(SIOCGIWFRAG,		jedi_wx_giwfrag),
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

const struct iw_handler_def jedi_wext_handler_def = {
	.standard		= jedi_wx_handlers,
	.num_standard		= ARRAY_SIZE(jedi_wx_handlers),
	.get_wireless_stats	= jedi_wx_get_stats,
};