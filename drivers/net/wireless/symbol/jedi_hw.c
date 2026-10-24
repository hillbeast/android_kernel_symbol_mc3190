/*
 * jedi_hw.c -- Jedi firmware operations: init, regulatory, scan, join, traps
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


/*
 * Hardware hooks (glossary R5.5, R7.1, R7.5, R7.6).
 */

/* Payload values are little-endian (the firmware is ARM LE; R7.3 trap). */
int jedi_set_u32(struct jedi_private *priv, u32 oid, u32 val)
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

	if (ret && oid != JEDI_OID_CISCO_SCAN_DIS)	/* fails under WinCE too */
		dev_warn(&priv->func->dev,
			 "jedi: init SET 0x%08x = %u failed: %d\n", oid, val, ret);
	return ret;
}

static int jedi_init_get(struct jedi_private *priv, u32 oid, u32 size,
			 u8 *buf, u32 *got)
{
	int ret = jedi_cmd(priv, JEDI_OP_GET, oid, NULL, size, buf, size, got);

	if (ret && oid != JEDI_OID_CISCO_SCAN_DIS)	/* fails under WinCE too */
		dev_warn(&priv->func->dev,
			 "jedi: init GET 0x%08x failed: %d\n", oid, ret);
	return ret;
}

/*
 * Regulatory tables (R15.1). The firmware's tables are empty in the image;
 * the host must download them with SET 0x1f00000e BEFORE the MLME starts
 * (the SET fails once the empty-SSID SET has run). Source:
 *   /lib/firmware/jedi_reg.bin: a sequence of { u16 LE length, blob }, each
 *   blob forwarded unchanged (WinCE stores up to 11 blobs of <= 0x400 bytes).
 * Without that file, module parameter reg_test=1 sends one guessed row.
 *
 * After the blobs, WinCE sends SET 0x02000000. The WinCE trace findings
 * (dump-wifienabled.bin) captured the real payload and the real reply:
 *   TX: 01 01 00 00 00 0d 00 02 00 00 00 04 00 00 00 00
 *   RX: 01 02 00 00 00 0d 00 02 00 00 00 04 00 00 00 00  (RESP, ~207ms later)
 * Our old payload (01 00 00 ... all zero after the first byte) was wrong
 * and got an error frame back instead. Byte 1 (0x01 -> 0x02 in the reply)
 * looks like an ack/sequence style field the firmware flips; send WinCE's
 * exact bytes rather than guessing at the meaning of the rest.
 */
/* One regulatory blob. A failure here means the card has stopped answering
 * (retrying the same blob never helped), so flag it: probe resets the chip
 * and runs the init job again. */
static int jedi_reg_send(struct jedi_private *priv, const u8 *p, u32 len)
{
	int ret = jedi_cmd(priv, JEDI_OP_SET, JEDI_OID_REG_TABLE, p, len,
			   NULL, 0, NULL);

	if (ret) {
		dev_warn(&priv->func->dev,
			 "jedi: regulatory blob failed: %d\n", ret);
		priv->hw_failed = true;
	}
	return ret;
}

/* Exact bytes WinCE sends for the commit (see the comment above). */
static void jedi_hw_reg_commit(struct jedi_private *priv)
{
	static const u8 blk[16] = {
		0x01, 0x01, 0x00, 0x00, 0x00, 0x0d, 0x00, 0x02,
		0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00,
	};
	int ret = jedi_cmd(priv, JEDI_OP_SET, JEDI_OID_REG_DONE, blk,
			   sizeof(blk), NULL, 0, NULL);

	jedi_dbg(priv->func, "SET 0x02000000: %d\n", ret);
}

static int jedi_hw_regulatory(struct jedi_private *priv)
{
	struct sdio_func *func = priv->func;
	struct jedi_blob fwb, *fw = &fwb;
	int sent = 0, ret;

	/* R16.2: begin download. Clears the MLME lock and the tables-supplied
	 * flag, stops the MLME; SET 0x1f00000e fails without it once locked. */
	ret = jedi_set_u32(priv, JEDI_OID_REG_BEGIN, 0);
	dev_info(&func->dev, "jedi: SET 0x10 = 0 (regulatory begin): %d\n", ret);
	/* Its handler resets state and stops the MLME; with debug=0 the next
	 * SET arrives ~1 ms later, so let it settle first. */
	msleep(20);

	if (!jedi_blob_get(func, "jedi_reg.bin", fw, true)) {
		const u8 *p = fw->data, *end = fw->data + fw->size;
		bool jreg = fw->size >= 8 && !memcmp(p, "JREG", 4);
		unsigned int n_blobs = 0, hdr = 2;

		/* Preferred: RE's container: 'JREG' | u16 version(1) |
		 * u16 n_blobs | n x { u32 LE length, payload }. The eight
		 * WinCE blobs go out verbatim (jedi_reg_wince_full.jreg).
		 * Legacy: a bare sequence of { u16 LE length, blob }. */
		if (jreg) {
			if (get_unaligned_le16(p + 4) != 1) {
				dev_warn(&func->dev,
					 "jedi: jedi_reg.bin: unknown version %u\n",
					 get_unaligned_le16(p + 4));
				jreg = false;
				goto legacy;
			}
			n_blobs = get_unaligned_le16(p + 6);
			p += 8;
			hdr = 4;
			dev_info(&func->dev, "jedi: jedi_reg.bin: JREG, %u blobs\n",
				 n_blobs);
		}
legacy:
		while (end - p >= hdr && (!jreg || sent < n_blobs)) {
			u32 len = jreg ? get_unaligned_le32(p) :
					 get_unaligned_le16(p);

			p += hdr;
			if (!len || len > 0x400 || len > end - p) {
				dev_warn(&func->dev,
					 "jedi: jedi_reg.bin: bad blob length %u\n",
					 len);
				priv->hw_failed = true;
				break;
			}
			ret = jedi_reg_send(priv, p, len);
			dev_info(&func->dev,
				 "jedi: regulatory blob %d (%u bytes): %d\n",
				 sent, len, ret);
			sent++;
			p += len;
			if (ret)
				break;
		}
		if (jreg && sent != n_blobs)
			dev_warn(&func->dev,
				 "jedi: sent %d of %u regulatory blobs\n",
				 sent, n_blobs);
		jedi_blob_put(fw);
	} else if (jedi_reg_test) {
		/* RE's minimal NZ table (R17): one record, table 0 (2.4 GHz),
		 * row 0: "NZ", channels 1..13, base power 18, class 0x30
		 * (ETSI-like). Not validated on hardware. */
		static const u8 rec[16] = {
			0x00, 0x00,		/* row index */
			0x01,			/* record count */
			0x00,			/* table 0 = 2.4 GHz */
			'N', 'Z', 0x20, 0x00,	/* [0..3] country */
			0x01, 0x0d,		/* [4] first channel, [5] count 13 */
			0x12, 0x00,		/* [6] base power, [7] back-off */
			0x00, 0x00, 0x30, 0x00,	/* [8],[9],[10] class,[11] */
		};

		ret = jedi_reg_send(priv, rec, sizeof(rec));
		dev_info(&func->dev, "jedi: regulatory TEST row: %d\n", ret);
		if (!ret)
			sent = 1;
	} else {
		dev_info(&func->dev,
			 "jedi: no jedi_reg.bin, regulatory tables stay empty\n");
	}

	if (sent)
		jedi_hw_reg_commit(priv);
	return sent;
}

/*
 * WinCE trace findings 5: after the table and the config job, WinCE
 * re-asserts the band/11d settings and commits again, and only then does
 * GET 0x1f000002 return "NZ " plus the 13 frequencies:
 *   SET 0x1f000000 = 1 (twice), 0x1f000004 = 0, 0x1f000004 = 1,
 *   0x1f000005 = 3, SET 0x02000000.
 */
static void jedi_hw_reg_finish(struct jedi_private *priv)
{
	jedi_init_set(priv, JEDI_OID_BAND_MASK, 1);
	jedi_init_set(priv, JEDI_OID_BAND_MASK, 1);
	jedi_init_set(priv, JEDI_OID_DOT11D_MODE, 0);
	jedi_init_set(priv, JEDI_OID_DOT11D_MODE, jedi_dot11d);
	jedi_init_set(priv, JEDI_OID_WWR_MODE, 3);
	jedi_hw_reg_commit(priv);
}

/* Country string (R15.2): 2 chars -> {c1,c2,0x20,0}, 3 chars -> {c1,c2,c3,0} */
static void jedi_hw_country(struct jedi_private *priv)
{
	u8 b[4] = { 0 };
	size_t n;
	int ret;

	if (!jedi_country || !*jedi_country)
		return;
	n = strlen(jedi_country);
	if (n < 2 || n > 4)
		return;
	if (n == 2) {
		b[0] = jedi_country[0];
		b[1] = jedi_country[1];
		b[2] = 0x20;
	} else {
		memcpy(b, jedi_country, 3);
	}
	ret = jedi_cmd(priv, JEDI_OP_SET, JEDI_OID_COUNTRY, b, 4, NULL, 0, NULL);
	dev_info(&priv->func->dev, "jedi: country %s: %d\n", jedi_country, ret);
}

/* GET 0x1f000002: current country, then the 4-byte entries it accepts. */
static void jedi_hw_log_countries(struct jedi_private *priv)
{
	u8 *b = kzalloc(0x104, GFP_KERNEL);
	u32 got = 0;

	if (!b)
		return;
	if (!jedi_cmd(priv, JEDI_OP_GET, JEDI_OID_COUNTRY, NULL, 0x104, b, 0x104,
		      &got))
		dev_info(&priv->func->dev,
			 "jedi: country '%.4s', %u bytes (%u accepted)\n", b, got,
			 got > 4 ? (got - 4) / 4 : 0);
	if (jedi_debug && got)
		print_hex_dump_bytes("jedi: countries ", DUMP_PREFIX_OFFSET, b,
				     min_t(u32, got, 64));
	kfree(b);
}


/*
 * R18.6 experiment: the firmware's debug log may be bridged through register
 * 0x50 with HOST_INT 0x8000 = data ready. Poll HOST_INT, log it whenever any
 * bit other than the known TX-free/awake ones is set, and drain text.
 * @tag names the init step so the log shows when text appeared.
 */
static void jedi_uart_poll(struct jedi_private *priv, const char *tag)
{
	struct sdio_func *func = priv->func;
	char line[128];
	int n = 0, i, ret;
	u32 hi = 0;

	if (!jedi_uart)
		return;
	sdio_claim_host(func);
	ret = jedi_reg_read(func, JEDI_REG_HOST_INT, 2, &hi);
	if (ret) {
		sdio_release_host(func);
		return;
	}
	dev_info(&func->dev, "jedi: uart[%s] HOST_INT=0x%04x\n", tag, hi);
	for (i = 0; (hi & 0x8000) && i < 4096; i++) {
		u8 c = sdio_readb(func, JEDI_REG_TX_LEN, &ret);

		if (ret)
			break;
		jedi_reg_write(func, JEDI_REG_HOST_INT_ACK, 2, 0x8000);
		if (c == '\n' || n == sizeof(line) - 1) {
			line[n] = 0;
			dev_info(&func->dev, "jedi: fwlog: %s\n", line);
			n = 0;
			if (c == '\n')
				goto next;
		}
		if (c >= 0x20 && c < 0x7f)
			line[n++] = c;
next:
		hi = 0;
		if (jedi_reg_read(func, JEDI_REG_HOST_INT, 2, &hi))
			break;
	}
	if (n) {
		line[n] = 0;
		dev_info(&func->dev, "jedi: fwlog: %s\n", line);
	}
	sdio_release_host(func);
}

/*
 * Start-up sequence as WinCE sends it (glossary R7.6, R7.6b, R8.4): the
 * init job (R6-A) then the configuration job (R6-B), using the driver's
 * built-in defaults. Steps WinCE skips with this registry are not sent.
 * Failures only warn. The final empty-SSID SET matters most: the firmware
 * starts its MLME task on it (R7.6c), and a scan is rejected before that.
 */
void jedi_hw_init(struct jedi_private *priv)
{
	u8 b[258];
	u32 got = 0, phycap = 3;
	int reg_sent;

	/* --- R6-A init job --- */
	if (!jedi_init_get(priv, JEDI_OID_RADIO_STATE, 4, b, &got) && got == 4)
		jedi_dbg(priv->func, "0x0b = 0x%x\n", get_unaligned_le32(b));
	if (!jedi_init_get(priv, JEDI_OID_INACTIVE_STATE, 4, b, &got) && got == 4)
		jedi_dbg(priv->func, "0x0c = 0x%x\n", get_unaligned_le32(b));
	/* WinCE trace findings 3: SET 0x0b = 0 (not 1) after the two GETs. */
	jedi_init_set(priv, JEDI_OID_RADIO_STATE, 0);

	jedi_uart_poll(priv, "boot");
	reg_sent = jedi_hw_regulatory(priv);		/* R15: before the MLME */
	if (priv->hw_failed)
		return;		/* probe resets the chip and retries */
	jedi_uart_poll(priv, "after-table");

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

	jedi_init_set(priv, JEDI_OID_DOT11D_MODE, 1);		/* step 18, 11dMode */
	jedi_hw_country(priv);
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
	jedi_hw_log_countries(priv);
	jedi_uart_poll(priv, "before-mlme");
	/* WinCE trace findings 4: the config job's SSID is "-NoProfileApplied-",
	 * not an empty one. */
	jedi_hw_set_ssid(priv, (const u8 *)"-NoProfileApplied-", 18);
	jedi_uart_poll(priv, "after-mlme");
	jedi_init_set(priv, JEDI_OID_CCX_PARAMS, 5);
	if (reg_sent)
		jedi_hw_reg_finish(priv);	/* findings 5, then country GET */

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
				     min_t(u32, got, 64));
	jedi_hw_log_countries(priv);	/* after the MLME rebuilt the tables */
	jedi_uart_poll(priv, "after-wait");

	if (jedi_rebuild) {
		/* R18.7: change the band mask so the next type 1/2/4 request
		 * copies it into the band selector, then restart the MLME. */
		/*
		 * The manual 'try' runs that produced "NZ " all repeated the whole
		 * sequence with the MLME already running: SET 0x10 (stops the
		 * MLME, clears the lock and the tables flag), table, band mask,
		 * 802.11d, empty SSID. Do the same here.
		 */
		jedi_hw_regulatory(priv);
		jedi_init_set(priv, JEDI_OID_BAND_MASK, 1);
		jedi_init_set(priv, JEDI_OID_BAND_MASK, 3);
		jedi_init_set(priv, JEDI_OID_DOT11D_MODE, 1);
		jedi_hw_set_ssid_empty(priv);
		msleep(1000);
		jedi_uart_poll(priv, "after-rebuild");
		if (!jedi_init_get(priv, 0x1400000d, 256, b, &got)) {
			dev_info(&priv->func->dev, "jedi: chanlist2: %u bytes\n", got);
			print_hex_dump_bytes("jedi: chanlist2 ", DUMP_PREFIX_OFFSET,
					     b, min_t(u32, got, 256));
		}
		jedi_hw_log_countries(priv);
	}

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
/*
 * Round-2 log analysis: the area at +0x40 (length from +0x3e) is the raw
 * beacon BODY, not just IEs: timestamp(8), beacon interval(2), capability(2)
 * (e.g. 21 04 = ESS|short preamble|short slot), then the IEs (SSID IE first).
 * The old guessed offsets for channel (+0x33) and capabilities (+0x36) were
 * wrong (reported channel 2 for an AP whose DS Parameter IE says 11). The
 * channel is taken from the DS Parameter IE (id 3) or the HT Operation IE
 * (id 61), falling back to +0x33.
 */
static u8 jedi_ie_channel(const u8 *ie, u32 len)
{
	u32 p = 0;
	u8 ht = 0;

	while (p + 2 <= len) {
		u8 id = ie[p], l = ie[p + 1];

		if (p + 2 + l > len)
			break;
		if (id == 3 && l >= 1)
			return ie[p + 2];
		if (id == 61 && l >= 1)
			ht = ie[p + 2];
		p += 2 + l;
	}
	return ht;
}

/* Length of @ie trimmed back to a whole-IE boundary within @max bytes. */
static u32 jedi_ie_trim(const u8 *ie, u32 len, u32 max)
{
	u32 p = 0;

	while (p + 2 <= len && p + 2 + ie[p + 1] <= max && p + 2 + ie[p + 1] <= len)
		p += 2 + ie[p + 1];
	return p;
}

static void jedi_parse_bss(struct jedi_private *priv, const u8 *e, u32 len)
{
	u8 ssid_len, ch;
	u16 caps = 0, body_len, freq;
	const u8 *ies = NULL;
	u32 ies_len = 0;

	if (len < JEDI_BSS_HDR)
		return;
	ssid_len = e[0x0e];
	if (ssid_len > JEDI_SSID_MAX)
		ssid_len = JEDI_SSID_MAX;
	/* Hidden networks come back as length 1 with a NUL byte (or a run of
	 * NULs): report them as an empty SSID. */
	{
		u8 k;

		for (k = 0; k < ssid_len && !e[0x0f + k]; k++)
			;
		if (k == ssid_len)
			ssid_len = 0;
	}
	ch = e[0x33];
	body_len = get_unaligned_le16(e + 0x3e);
	if (body_len > len - JEDI_BSS_HDR)
		body_len = len - JEDI_BSS_HDR;
	if (body_len >= 12) {
		caps = get_unaligned_le16(e + JEDI_BSS_HDR + 10);
		ies = e + JEDI_BSS_HDR + 12;
		ies_len = jedi_ie_trim(ies, body_len - 12, JEDI_MAX_BSS_IE);
		if (jedi_ie_channel(ies, body_len - 12))
			ch = jedi_ie_channel(ies, body_len - 12);
	}

	if (ch >= 1 && ch <= 13)
		freq = 2407 + 5 * ch;
	else if (ch == 14)
		freq = 2484;
	else
		freq = 5000 + 5 * ch;

	jedi_wext_bss_add(priv, e, e + 0x0f, ssid_len, freq, (s8)e[0x0d], caps,
			  ies, ies_len);
	if (body_len >= 12) {
		/* WMM: vendor IE 00:50:f2 type 2 (WinCE: 0x13000108 follows it) */
		u32 p = 0, n = body_len - 12;
		bool wmm = false;

		while (p + 2 <= n && p + 2 + ies[p + 1] <= n) {
			if (ies[p] == 221 && ies[p + 1] >= 4 &&
			    !memcmp(ies + p + 2, "\x00\x50\xf2\x02", 4))
				wmm = true;
			p += 2 + ies[p + 1];
		}
		jedi_bss_set_wmm(priv, e, wmm);
	}
}

void jedi_scan_work(struct work_struct *work)
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

	/*
	 * Round-2 log: a SET before EVERY GET resets the list cursor, so the
	 * firmware returned the first BSS 32 times. WinCE sends ONE SET, then
	 * GETs until the list is exhausted, then a final SET (trace findings,
	 * 18.314 to 18.328 s).
	 */
	ret = jedi_cmd(priv, JEDI_OP_SET, JEDI_OID_BSSLIST, req, sizeof(req),
		       NULL, 0, NULL);
	if (ret)
		jedi_dbg(priv->func, "BSS list SET failed: %d\n", ret);

	for (i = 0; i < JEDI_MAX_BSS; i++) {
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
	/* WinCE ends the loop with another SET (resets the cursor). */
	jedi_cmd(priv, JEDI_OP_SET, JEDI_OID_BSSLIST, req, sizeof(req), NULL, 0,
		 NULL);
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
int jedi_hw_scan(struct jedi_private *priv, const u8 *ssid, u8 ssid_len)
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
int jedi_hw_set_ssid(struct jedi_private *priv, const u8 *ssid, u8 len)
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
int jedi_hw_set_bssid(struct jedi_private *priv, const u8 *bssid)
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

/* TODO(RE Q4): WEP-enable pair 0x12000001 (privacy invoked) and 0x12000002
 * (exclude unencrypted). Off unless jedi.privacy=1. */
static void jedi_hw_set_privacy(struct jedi_private *priv, bool on)
{
	if (!jedi_send_privacy)
		return;
	jedi_dbg(priv->func, "privacy %d\n", on);
	jedi_set_u32(priv, 0x12000001, on);
	jedi_set_u32(priv, 0x12000002, on);
}

/*
 * 802.1X port authorise (WPA2 capture): after the 4-way handshake WinCE sends
 * PTK, GTK (0x1200000a) and then SET 0x150007e4 = BSSID + 2 zero bytes.
 * Nothing is known about a "close" command; a disconnect is a bogus SSID.
 */
void jedi_hw_port_authorize(struct jedi_private *priv, bool authorized)
{
	u8 b[8] = { 0 };

	if (!authorized) {
		priv->wx.ptk_set = priv->wx.gtk_set = false;
		return;
	}
	priv->port_closed = false;
	jedi_diag_start(priv);
	if (!jedi_port_oid)
		return;
	mutex_lock(&priv->wx.lock);
	memcpy(b, priv->wx.cur_bssid, ETH_ALEN);
	mutex_unlock(&priv->wx.lock);
	dev_info(&priv->func->dev, "jedi: port authorise 0x%08x bssid %pM\n",
		 jedi_port_oid, b);
	jedi_cmd(priv, JEDI_OP_SET, jedi_port_oid, b, sizeof(b), NULL, 0, NULL);
}

static void jedi_hw_set_security(struct jedi_private *priv)
{
	bool wpa;
	u32 code = jedi_suite_code(priv, &wpa);

	if (jedi_suite_override)
		code = jedi_suite_override;
	jedi_dbg(priv->func, "security suite %u wpa %d\n", code, wpa);
	jedi_set_u32(priv, JEDI_OID_SECURITY, code);
	jedi_set_u32(priv, JEDI_OID_SEC_MODE, wpa ? 1 : 0);
	jedi_hw_set_privacy(priv, code != 1);
}

/* R9.3: WinCE sets the suite to open, then an empty SSID. */
int jedi_hw_disassociate(struct jedi_private *priv)
{
	jedi_dbg(priv->func, "disassociate\n");
	jedi_hw_port_authorize(priv, false);
	jedi_set_u32(priv, JEDI_OID_SECURITY, 1);
	priv->wx.suite_sent = 1;
	return jedi_hw_set_ssid_empty(priv);
}

/*
 * The firmware encrypts every data frame, EAPOL included, once a pairwise
 * key is installed. After a link loss (it re-associates on its own) the old
 * key would encrypt message 2 of the next handshake, so wipe the key table
 * (SET 0x12000008, type byte 8, as WinCE does at start).
 */
void jedi_clearkeys_work(struct work_struct *work)
{
	struct jedi_private *priv = container_of(work, struct jedi_private,
						 clearkeys_work);
	u8 key[44] = { 0 };

	int i;

	key[10] = 8;
	priv->wx.ptk_set = priv->wx.gtk_set = false;
	if (jedi_key_remove_mode) {
		for (i = 0; i < 4; i++) {
			u8 mac[ETH_ALEN];

			if (!priv->wx.key[i].valid)
				continue;
			memcpy(mac, priv->wx.key[i].mac, ETH_ALEN);
			jedi_key_remove(priv, mac, i);
			priv->wx.key[i].valid = false;
		}
	}
	jedi_cmd(priv, JEDI_OP_SET, JEDI_OID_STA_KEY, key, sizeof(key), NULL, 0,
		 NULL);
}

/* After a connect trap the BSSID is read with GET 0x10000001 (R9.1). */
void jedi_link_work(struct work_struct *work)
{
	struct jedi_private *priv = container_of(work, struct jedi_private,
						 link_work);
	u8 b[16];
	u32 rlen = 0;
	int ret;

	ret = jedi_cmd(priv, JEDI_OP_GET, JEDI_OID_BSSID, NULL, ETH_ALEN, b, ETH_ALEN,
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
 * WinCE's connect sequence, taken from the open-network capture
 * (dump-connecttoopennetwork.bin, join to "TesterPoops"). WinCE does NOT send
 * the suite OIDs (0x1200000b / 0x12000012) for an open network. Every join
 * is this, with these gaps:
 *   1. SSID = 32 bytes 01 02 .. 1f 20 (a name nothing uses: this is how WinCE
 *      cancels a join / disassociates), BSSID = 0
 *   2. ~3.3 s later: 0x1f000000=1, 0x1f000004=0, 0x1f000005=3,
 *      0x1f000004=1, 0x1f000005=3, commit (SET 0x02000000)
 *   3. ~3.3 s later: GET 0x1f000002 (260 zeroed bytes: the country)
 *   4. SET 0x12000008 = 44 bytes, byte +10 = 8 (clear the key table)
 *   5. 0x17000100=0, 0x14000000=2, 0x12000004..7 (20 bytes each, WinCE sends
 *      stack junk here, zeros are used), 0x10000000=1, 0x150007e0=0,
 *      0x13000108=0, 0x12000001=0, 0x12000002=0
 *   6. SSID, BSSID (zero = any), 0x13000102 = 6 zeros
 * The firmware answers about 4 s later with traps 0x18000001 (auth),
 * 0x18000003 x2 (assoc request/response), then 0x00000001 (link up).
 */
static void jedi_hw_join_prepare(struct jedi_private *priv, u32 code, bool wpa,
				 bool wmm)
{
	static const u8 bogus[32] = {
		1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18,
		19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32,
	};
	u8 zero6[ETH_ALEN] = { 0 }, key[44] = { 0 }, slot[20] = { 0 };
	u8 country[260] = { 0 };
	u32 got = 0;
	int i;

	jedi_hw_set_ssid(priv, bogus, sizeof(bogus));
	jedi_hw_set_bssid(priv, zero6);
	msleep(jedi_join_settle_ms);

	jedi_init_set(priv, JEDI_OID_BAND_MASK, 1);
	jedi_init_set(priv, JEDI_OID_DOT11D_MODE, 0);
	jedi_init_set(priv, JEDI_OID_WWR_MODE, 3);
	jedi_init_set(priv, JEDI_OID_DOT11D_MODE, jedi_dot11d);
	jedi_init_set(priv, JEDI_OID_WWR_MODE, 3);
	/*
	 * The firmware applies the regulatory commit asynchronously and
	 * announces the result with trap 0x1f000003. Joins that raced it read
	 * back an empty or odd country and then never associated, so wait for
	 * the trap (and retry the commit once if it never comes).
	 */
	for (i = 0; i < 2; i++) {
		INIT_COMPLETION(priv->country_done);
		jedi_hw_reg_commit(priv);
		jedi_hw_country(priv);	/* no-op unless jedi.country is set */
		if (!jedi_country_wait_ms)
			break;
		if (wait_for_completion_timeout(&priv->country_done,
				msecs_to_jiffies(jedi_country_wait_ms))) {
			dev_info(&priv->func->dev, "jedi: regulatory trap seen (try %d)\n", i);
			break;
		}
		dev_info(&priv->func->dev, "jedi: no regulatory trap after %u ms (try %d)\n",
			 jedi_country_wait_ms, i);
	}
	msleep(jedi_join_settle_ms);

	if (!jedi_cmd(priv, JEDI_OP_GET, JEDI_OID_COUNTRY, NULL, sizeof(country),
		      country, sizeof(country), &got) && got >= 3)
		dev_info(&priv->func->dev, "jedi: country now \"%.3s\"\n",
			 country);

	key[10] = 8;
	jedi_cmd(priv, JEDI_OP_SET, 0x12000008, key, sizeof(key), NULL, 0, NULL);
	jedi_init_set(priv, 0x17000100, 0);
	jedi_init_set(priv, 0x14000000, 2);
	for (i = 4; i <= 7; i++)
		jedi_cmd(priv, JEDI_OP_SET, 0x12000000 + i, slot, sizeof(slot),
			 NULL, 0, NULL);
	if (jedi_dummy_keys) {
		/* WinCE: ff:ff:ff:ff:ff:ff, idx 0..3, type byte 1 (stack junk after) */
		for (i = 0; i < 4; i++) {
			memset(key, 0, sizeof(key));
			memset(key, 0xff, ETH_ALEN);
			key[8] = i;
			key[10] = 1;
			jedi_cmd(priv, JEDI_OP_SET, 0x12000008, key, sizeof(key),
				 NULL, 0, NULL);
		}
	}
	jedi_init_set(priv, JEDI_OID_BSS_TYPE, JEDI_BSS_INFRA);
	/* WPA2 capture: the suite goes here, after 0x10000000, and sticks
	 * in the firmware (WinCE sent it once). Open joins send nothing
	 * unless an earlier join changed it. */
	if (jedi_wmm_mode >= 0)
		wmm = jedi_wmm_mode;
	dev_info(&priv->func->dev, "jedi: join setup: suite %u%s wpa %d wmm %d dummy_keys %d\n",
		 code, jedi_send_suite ? "" : " (not sent)", wpa, wmm, jedi_dummy_keys);
	if (jedi_send_suite && (code != 1 || (priv->wx.suite_sent && priv->wx.suite_sent != 1))) {
		jedi_set_u32(priv, JEDI_OID_SECURITY, code);
		priv->wx.suite_sent = code;
	}
	jedi_init_set(priv, 0x150007e0, 0);
	jedi_init_set(priv, 0x13000108, wmm ? 1 : 0);
	jedi_set_u32(priv, 0x12000001, wpa ? 3 : 0);
	jedi_set_u32(priv, 0x12000002, wpa ? 1 : 0);
}

/*
 * Join. wpa_supplicant sets the SSID and then the BSSID, but the firmware
 * starts joining as soon as the SSID is written, so the target BSSID has to
 * be in place first. This runs 150 ms after the ioctls, when both are known:
 * suite, BSSID, SSID (last), then 0x13000102 = 0 as WinCE does after an
 * SSID change (R5.5). Afterwards the MLME state (0x10000003) is polled so the
 * log shows how far the join gets.
 */
void jedi_join_work(struct work_struct *work)
{
	struct jedi_private *priv = container_of(work, struct jedi_private,
						 join_work.work);
	struct jedi_wext *wx = &priv->wx;
	u8 ssid[JEDI_SSID_MAX], bssid[ETH_ALEN] = { 0 }, zero6[6] = { 0 };
	u8 len, b[16];
	u32 rlen, st, last = 0xffffffff;
	int i;

	mutex_lock(&wx->lock);
	len = wx->ssid_len;
	memcpy(ssid, wx->ssid, len);
	if (wx->bssid_set)
		memcpy(bssid, wx->bssid, ETH_ALEN);
	mutex_unlock(&wx->lock);
	if (!len)
		return;

	priv->join_jiffies = jiffies;
	priv->data_traced = 0;		/* fresh trace budget for this join */
	dev_info(&priv->func->dev, "jedi: joining \"%.*s\" bssid %pM (%s)\n",
		 len, ssid, bssid, wx->bssid_set ? "locked" : "any");
	if (jedi_join_mode == 3) {
		bool wpa;
		u32 code = jedi_suite_code(priv, &wpa);

		if (jedi_suite_override)
			code = jedi_suite_override;
		priv->wx.ptk_set = priv->wx.gtk_set = false;
		priv->wx.req_ie_len = priv->wx.resp_ie_len = 0;
		priv->port_closed = wpa;
		jedi_hw_join_prepare(priv, code, wpa, jedi_bss_wmm(priv, ssid, len));
	} else {
		priv->port_closed = false;
		jedi_hw_set_security(priv);
	}
	switch (jedi_join_mode) {
	case 3:			/* WinCE: SSID, then BSSID (zero = any) */
		if (!jedi_bssid_lock)
			memset(bssid, 0, ETH_ALEN);
		jedi_hw_set_ssid(priv, ssid, len);
		jedi_hw_set_bssid(priv, bssid);
		break;
	case 1:			/* SSID only */
		jedi_hw_set_ssid(priv, ssid, len);
		break;
	case 2:			/* SSID, then BSSID */
		jedi_hw_set_ssid(priv, ssid, len);
		jedi_hw_set_bssid(priv, bssid);
		break;
	default:		/* BSSID, then SSID (starts the join) */
		jedi_hw_set_bssid(priv, bssid);
		jedi_hw_set_ssid(priv, ssid, len);
		break;
	}
	jedi_cmd(priv, JEDI_OP_SET, 0x13000102, zero6, sizeof(zero6), NULL, 0,
		 NULL);

	for (i = 0; i < jedi_join_poll_ms / 250; i++) {
		msleep(250);
		rlen = 0;
		if (jedi_cmd(priv, JEDI_OP_GET, 0x10000003, NULL, 4, b, 4,
			     &rlen) || rlen < 4)
			continue;
		st = get_unaligned_le32(b);
		if (st != last) {
			dev_info(&priv->func->dev,
				 "jedi: join state %u after %d ms\n", st,
				 (i + 1) * 250);
			last = st;
		}
		if (st == 5)
			break;
	}
	/* Some firmware builds may report the join in the MLME state before
	 * (or instead of) the link trap. Off by default: state 5 is a guess. */
	if (jedi_link_fallback && last == 5) {
		mutex_lock(&wx->lock);
		last = wx->connected;
		mutex_unlock(&wx->lock);
		if (!last) {
			dev_warn(&priv->func->dev,
				 "jedi: state 5 but no link trap, forcing link up\n");
			schedule_work(&priv->link_work);
		}
	}
}

/*
 * Traps (PIMFOR op 4), called from the SDIO IRQ thread: must not issue
 * commands (jedi_cmd waits for this very thread). Use workers.
 */
void jedi_handle_trap(struct jedi_private *priv, u32 oid,
			     const u8 *p, u32 plen)
{
	struct sdio_func *func = priv->func;

	switch (oid) {
	case JEDI_OID_LINKSTATE: {
		/* Capture-confirmed: word0 != 0 connected (rate code in 500 kbps
		 * units, 0x6c = 54 Mbps), 0 disconnected, word1 = disconnect
		 * reason, +0x14 = 1, +0x18 = channel frequency in kHz (0x24e168 =
		 * 2417 MHz), +0x1c = beacon interval. */
		u32 w0, w1, rate;

		if (plen < 8)
			break;
		w0 = get_unaligned_le32(p);
		w1 = get_unaligned_le32(p + 4);
		rate = plen >= 0x1c ? get_unaligned_le32(p + 0x18) / 1000 : 0;
		dev_info(&func->dev,
			 "jedi: link %s (rate code %u, reason %u, %u MHz) "
			 "%u ms after join\n",
			 w0 ? "UP" : "DOWN", w0, w1, rate,
			 jiffies_to_msecs(jiffies - priv->join_jiffies));
		/* the layout is not fully known: always show it */
		print_hex_dump_bytes("jedi: linkstate ",
					     DUMP_PREFIX_OFFSET, p,
					     min_t(u32, plen, 0x30));
		if (w0) {
			schedule_work(&priv->link_work);
		} else {
			jedi_wext_link_event(priv, NULL);
			schedule_work(&priv->clearkeys_work);
		}
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
		dev_warn(&func->dev, "jedi: Michael MIC failure, AP %pM\n", p);
		jedi_wext_mic_failure(priv, plen >= 6 ? p : NULL);
		break;
	case JEDI_OID_DEAUTH_TRAP:
	case JEDI_OID_DISASSOC_TRAP:
		dev_info(&func->dev, "jedi: deauth/disassoc trap 0x%08x, reason %u\n",
			 oid, plen >= 0x12 ? get_unaligned_le16(p + 0x10) : 0);
		print_hex_dump_bytes("jedi: mlme ", DUMP_PREFIX_OFFSET, p,
				     min_t(u32, plen, 0x40));
		jedi_wext_link_event(priv, NULL);
		schedule_work(&priv->clearkeys_work);
		break;
	case JEDI_OID_ASSOC_TRAP:
		/* WPA2 capture: 20-byte header (BSSID, ..., u16 body length at
		 * +18), then the 802.11 body. Request (cap, listen, IEs) is
		 * the first trap, response (cap, status, AID, IEs) the second;
		 * wpa_supplicant needs the request IEs for message 2. */
		if (plen >= 24) {
			u32 blen = get_unaligned_le16(p + 18);
			const u8 *body = p + 20;
			bool resp = priv->wx.req_ie_len != 0;
			u32 skip = resp ? 6 : 4;

			if (blen > plen - 20)
				blen = plen - 20;
			if (blen >= skip && blen - skip <= JEDI_MAX_IE) {
				mutex_lock(&priv->wx.lock);
				if (!resp) {
					memcpy(priv->wx.req_ie, body + skip,
					       blen - skip);
					priv->wx.req_ie_len = blen - skip;
				} else {
					memcpy(priv->wx.resp_ie, body + skip,
					       blen - skip);
					priv->wx.resp_ie_len = blen - skip;
				}
				mutex_unlock(&priv->wx.lock);
			}
		}
		/* fall through */
	case JEDI_OID_AUTH_TRAP:
	case JEDI_OID_REASSOC_TRAP:
		/* RE Q5: payload layouts undecoded; show them whatever debug is */
		dev_info(&func->dev, "jedi: mlme trap 0x%08x, %u bytes\n", oid, plen);
		print_hex_dump_bytes("jedi: mlme ", DUMP_PREFIX_OFFSET, p,
				     min_t(u32, plen, 0x40));
		break;
	default:
		if (oid == 0x1f000003)
			complete(&priv->country_done);
		if (oid == 0x16000101) {
			/* Statistics. Two counters at payload +0x10/+0x14 rise by
			 * 1 per SSID SET and 2 per scan (RE Q5); log changes. */
			u32 a, b;

			if (priv->diag_dump) {
				u32 off;
				char line[80];

				priv->diag_dump = false;
				dev_info(&func->dev, "jedi: stats trap %u bytes (non-zero rows)\n", plen);
				for (off = 0; off < plen; off += 16) {
					u32 n = min_t(u32, 16, plen - off), k;

					for (k = 0; k < n && !p[off + k]; k++)
						;
					if (k == n)
						continue;
					hex_dump_to_buffer(p + off, n, 16, 1, line,
							   sizeof(line), false);
					dev_info(&func->dev, "jedi: stats +%04x: %s\n", off, line);
				}
			}
			if (plen < 0x18)
				break;
			a = get_unaligned_le32(p + 0x10);
			b = get_unaligned_le32(p + 0x14);
			if (!priv->stat_valid || a != priv->stat_a ||
			    b != priv->stat_b)
				jedi_dbg(func, "stats counters %u %u\n", a, b);
			priv->stat_valid = true;
			priv->stat_a = a;
			priv->stat_b = b;
			break;
		}
		/* anything else unexpected is shown regardless of debug */
		dev_info(&func->dev, "jedi: trap 0x%08x, %u bytes\n", oid, plen);
		print_hex_dump_bytes("jedi: trap ", DUMP_PREFIX_OFFSET, p,
				     min_t(u32, plen, 0x40));
		break;
	}
}

/* Radio off before unload (WinCE's disable is SET 0x0b = 1, R10.0). */
void jedi_hw_shutdown(struct jedi_private *priv)
{
	jedi_hw_disassociate(priv);
	jedi_set_u32(priv, JEDI_OID_RADIO_STATE, 1);
}