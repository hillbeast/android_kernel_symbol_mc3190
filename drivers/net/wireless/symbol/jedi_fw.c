/*
 * jedi_fw.c -- Firmware Downloader for Symbol "Jedi" WiFi module in MC3190
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

/*
 * Protocol reverse-engineered from JEDI10.dll (WinCE miniport),
 * specifically FUN_1004e6d0 ("DownloadFirmware") and its subordinates.
 * See mc3190_jedi_glossary.md, REVISION 2 (R2.x references below).
 *
 * Firmware container format (JEDI10_dll.bin, raw / unstripped):
 *
 *   repeating, back to back, until all declared bytes consumed:
 *     u32 address   (LE; bit 31 set = "monitor path" chunk)
 *     u32 length    (LE, payload byte count)
 *     u8  payload[length]
 *
 * REGISTER ACCESS (R2.1):
 *   All control registers (0x04..0x54) are accessed with CMD52, one byte
 *   at a time, at ASCENDING addresses, little-endian. Every status bit
 *   the protocol tests (0x2000/0x4000/0x8000 in HOST_INT, the RESET and
 *   RAMBOOT bits in DEVSTAT) lives in a HIGH byte, which a fixed-address
 *   CMD53 (sdio_readsb/writesb) can never reach.
 *   Only the data port (function 1, address 0x0) is CMD53 fixed-address.
 *
 * SEQUENCE:
 *   1. Chip reset into boot ROM (R2.3 D)       jedi_chip_reset()
 *   2. Chunk 0, monitor path (R2.3 E)          jedi_monitor_write()
 *   3. Remaining chunks, bulk data port        jedi_download_chunk()
 *   4. Unmask interrupts, wait for FW Init     jedi_wait_fw_boot() (R2.4)
 *
 * Register map (see jedi.h for the defines).
 */

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/jiffies.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/sdio_func.h>
#include <linux/module.h>
#include <linux/slab.h>

#include "jedi.h"

#define JEDI_FW_NAME		"jedi_fw.bin"

/* Chunk address flag: bit 31 set = use monitor path */
#define JEDI_CHUNK_MONITOR_FLAG	0x80000000UL

/* Maximum burst size for the bulk CMD53 data port (9-bit count field). */
#define JEDI_MAX_BURST		512

/* WinCE hardcodes both the target address and the byte count as ASCII
 * literals. Address 0x1ffe0000 is the boot ROM trampoline area; count
 * 0x9d4 (2516) is a literal in WinCE, not derived from the chunk. */
#define JEDI_MONITOR_CMD	"<1ffe0000 9d4\r"

/* Timeouts */
#define JEDI_BULK_TIMEOUT_MS		1000
#define JEDI_MON_DONE_TIMEOUT_MS	5000	/* WinCE: 65535 tries, a few s */
#define JEDI_BOOT_TIMEOUT_MS		10000	/* WinCE watchdog: 10 s (R2.4) */
#define JEDI_RESET_WAIT_POLLS		100     /* R2.3 D.2: ~100 reads */
#define JEDI_FLUSH_MAX			1000

/* Per-byte monitor handshake poll. WinCE reads once; we poll briefly. */
#define JEDI_MON_POLL_US	20
#define JEDI_MON_POLL_MAX	20

struct jedi_fw_chunk {
	__le32 address;
	__le32 length;
	u8 payload[];
} __packed;

/* ------------------------------------------------------------------ */
/* Register layer: CMD52, ascending addresses, little-endian (R2.1)    */
/* ------------------------------------------------------------------ */

int jedi_reg_read(struct sdio_func *func, u32 addr, unsigned int n, u32 *val)
{
	u32 v = 0;
	unsigned int i;
	int ret;

	for (i = 0; i < n; i++) {
		u8 b = sdio_readb(func, addr + i, &ret);

		if (ret)
			return ret;
		v |= (u32)b << (8 * i);
	}
	*val = v;
	return 0;
}

int jedi_reg_write(struct sdio_func *func, u32 addr, unsigned int n, u32 val)
{
	unsigned int i;
	int ret = 0;

	for (i = 0; i < n; i++) {
		sdio_writeb(func, (val >> (8 * i)) & 0xff, addr + i, &ret);
		if (ret)
			return ret;
	}
	return 0;
}

static int jedi_host_int(struct sdio_func *func, u16 *status)
{
	u32 v;
	int ret;

	ret = jedi_reg_read(func, JEDI_REG_HOST_INT, 2, &v);
	if (!ret)
		*status = v;
	return ret;
}

static int jedi_ack(struct sdio_func *func, u32 val)
{
	return jedi_reg_write(func, JEDI_REG_HOST_INT_ACK, 4, val);
}

static int jedi_ctrl(struct sdio_func *func, u32 val)
{
	return jedi_reg_write(func, JEDI_REG_CTRL, 4, val);
}

static int jedi_monitor_writeb(struct sdio_func *func, u8 val)
{
	int ret;

	sdio_writeb(func, val, JEDI_REG_MONITOR_FIFO, &ret);
	return ret;
}

/* DEVSTAT read-modify-write: v = (v & ~clear) | set. */
static int jedi_devstat_rmw(struct sdio_func *func, u32 set, u32 clear,
			    u32 *after)
{
	u32 v;
	int ret;

	ret = jedi_reg_read(func, JEDI_REG_DEVSTAT, 4, &v);
	if (ret)
		return ret;
	v = (v & ~clear) | set;
	ret = jedi_reg_write(func, JEDI_REG_DEVSTAT, 4, v);
	if (!ret && after)
		*after = v;
	return ret;
}

/*
 * Poll HOST_INT until any bit in @mask is set. Returns 0 (hit),
 * -ETIMEDOUT, or a bus error. @last receives the last value read.
 * Spins briefly first, then sleeps.
 */
static int jedi_wait_host_int(struct sdio_func *func, u16 mask,
			      unsigned int timeout_ms, u16 *last)
{
	unsigned long deadline = jiffies + msecs_to_jiffies(timeout_ms);
	unsigned int i = 0;
	u16 status = 0;
	int ret;

	do {
		ret = jedi_host_int(func, &status);
		if (ret)
			return ret;
		*last = status;
		if (status & mask)
			return 0;
		if (i++ < 100)
			udelay(20);
		else
			msleep(1);
	} while (time_before(jiffies, deadline));

	return -ETIMEDOUT;
}

/* Snapshot of every register the protocol touches, for error paths. */
static void jedi_dump_regs(struct sdio_func *func, const char *where)
{
	u32 hostint = 0, devstat = 0;
	u8 r04, r4c, r50;
	int e04, e40, e54, e4c, e50;

	r04 = sdio_readb(func, JEDI_REG_STATUS, &e04);
	e40 = jedi_reg_read(func, JEDI_REG_HOST_INT, 2, &hostint);
	e54 = jedi_reg_read(func, JEDI_REG_DEVSTAT, 4, &devstat);
	r4c = sdio_readb(func, JEDI_REG_MONITOR_FIFO, &e4c);
	r50 = sdio_readb(func, JEDI_REG_TX_READY, &e50);

	dev_info(&func->dev,
		 "jedi_fw: [%s] 0x04=%02x HOST_INT=%04x DEVSTAT=%08x "
		 "0x4c=%02x 0x50=%02x (err %d/%d/%d/%d/%d)\n",
		 where, r04, hostint, devstat, r4c, r50,
		 e04, e40, e54, e4c, e50);
}

/* ------------------------------------------------------------------ */
/* Chip reset into boot ROM (R2.3 D, FUN_1004e2e0)                     */
/* ------------------------------------------------------------------ */

/*
 * Mask host interrupts, wait for DEVSTAT bit 0x8000 to clear, then pulse
 * RESET with RAMBOOT cleared so the chip comes up from its boot ROM
 * (which is what makes the ROM monitor listen). There is no "ROM ready"
 * signal: the host causes it (R2.0 #2).
 */
static int jedi_chip_reset(struct sdio_func *func)
{
	u32 ds = 0;
	unsigned int i;
	u8 b;
	int ret;

	/* D.1 */
	ret = jedi_reg_write(func, JEDI_REG_INT_ENABLE, 2, 0);
	if (ret) {
		dev_err(&func->dev, "jedi_fw: reset: INT_ENABLE write failed: %d\n",
			ret);
		return ret;
	}

	/* D.2 */
	for (i = 0; i < JEDI_RESET_WAIT_POLLS; i++) {
		ret = jedi_reg_read(func, JEDI_REG_DEVSTAT, 4, &ds);
		if (ret)
			return ret;
		if (!(ds & JEDI_DEVSTAT_WAIT))
			break;
		udelay(100);
	}
	if (ds & JEDI_DEVSTAT_WAIT)
		dev_warn(&func->dev,
			 "jedi_fw: reset: DEVSTAT 0x8000 still set after %u "
			 "polls (DEVSTAT=0x%08x), continuing\n", i, ds);

	/* D.3: clear RESET | RAMBOOT */
	ret = jedi_devstat_rmw(func, 0,
			       JEDI_DEVSTAT_RESET | JEDI_DEVSTAT_RAMBOOT, &ds);
	if (ret)
		return ret;
	/* D.4: assert RESET */
	ret = jedi_devstat_rmw(func, JEDI_DEVSTAT_RESET, 0, &ds);
	if (ret)
		return ret;
	/* D.5: release RESET (RAMBOOT clear => boot ROM) */
	ret = jedi_devstat_rmw(func, 0, JEDI_DEVSTAT_RESET, &ds);
	if (ret)
		return ret;

	/* D.6 */
	msleep(50);

	/* D.7 */
	b = sdio_readb(func, JEDI_REG_STATUS, &ret);
	if (ret)
		return ret;

	jedi_dbg(func, "chip reset done: DEVSTAT=0x%08x, 0x04=0x%02x\n", ds, b);
	return 0;
}

/* ------------------------------------------------------------------ */
/* Bulk path                                                           */
/* ------------------------------------------------------------------ */

/*
 * Send one block stream (R4): the chunk body only, which carries its own
 * 12-byte {dest_or_last, len, entry} header. The container's 8-byte
 * (addr, len) header is never sent. The body goes out in 512-byte byte-mode
 * CMD53 bursts, the last one shorter, through a kmalloc'd bounce buffer
 * (the PXA320 MMC DMA cannot take request_firmware()'s vmalloc memory).
 * The data port is function 1, address 0, fixed-address.
 *
 * The stub raises HOST_INT 0x2000 once per accepted block, and it must be
 * acked before the next block is sent. The last block raises 0x0001 (FW
 * Init) instead, which jedi_wait_fw_boot() picks up.
 */
static int jedi_download_chunk(struct sdio_func *func,
			       const u8 *data, u32 len,
			       unsigned int chunk_num, bool last)
{
	u8 *bounce;
	u32 sent = 0;
	u16 status = 0;
	int ret;

	bounce = kmalloc(JEDI_MAX_BURST, GFP_KERNEL);
	if (!bounce)
		return -ENOMEM;

	while (sent < len) {
		u32 burst = min_t(u32, len - sent, JEDI_MAX_BURST);

		memcpy(bounce, data + sent, burst);
		ret = sdio_writesb(func, 0x0, bounce, burst);
		if (ret) {
			dev_err(&func->dev,
				"jedi_fw: chunk %u burst write failed at "
				"offset %u/%u: %d\n", chunk_num, sent, len, ret);
			goto out;
		}
		sent += burst;
	}

	if (last) {
		jedi_dbg(func, "chunk %u sent (last block, FW Init follows)\n",
			 chunk_num);
		ret = 0;
		goto out;
	}

	ret = jedi_wait_host_int(func, JEDI_HOST_INT_BULK_OK,
				 3 * JEDI_BULK_TIMEOUT_MS, &status);
	if (ret == -ETIMEDOUT) {
		dev_err(&func->dev,
			"jedi_fw: chunk %u: block not accepted (HOST_INT=0x%04x)\n",
			chunk_num, status);
		jedi_dump_regs(func, "block timeout");
		goto out;
	}
	if (ret)
		goto out;

	ret = jedi_ack(func, JEDI_HOST_INT_BULK_OK);
	if (!ret)
		jedi_dbg(func, "chunk %u: %u bytes accepted\n", chunk_num, len);
out:
	kfree(bounce);
	return ret;
}

/* ------------------------------------------------------------------ */
/* Monitor path (R2.3 E)                                               */
/* ------------------------------------------------------------------ */

/* Per-byte acks seen/missed, for the debug log only. */
static struct {
	unsigned int cmd_hit, cmd_miss, data_hit, data_miss;
} jedi_mon_stats;

/*
 * Poll HOST_INT briefly for any bit in @mask. On a hit, ack with
 * 0xffffffff. Returns 1 on hit, 0 on miss (not an error: per R2.3 WinCE
 * does not act on missing per-byte acks), or a negative bus error.
 */
static int jedi_poll_mask_ack(struct sdio_func *func, u16 mask)
{
	unsigned int i;
	u16 status;
	int ret;

	for (i = 0; i < JEDI_MON_POLL_MAX; i++) {
		ret = jedi_host_int(func, &status);
		if (ret)
			return ret;
		if (status & mask) {
			ret = jedi_ack(func, 0xffffffff);
			return ret ? ret : 1;
		}
		udelay(JEDI_MON_POLL_US);
	}
	return 0;
}

/* FUN_1004e3f4: if HOST_INT & 0x4000, write 0xffffffff to 0x48. */
static int jedi_monitor_clear_pending(struct sdio_func *func)
{
	u16 status;
	int ret;

	ret = jedi_host_int(func, &status);
	if (ret)
		return ret;
	if (status & JEDI_HOST_INT_BYTE_OK)
		ret = jedi_ack(func, 0xffffffff);
	return ret;
}

/*
 * FUN_1004e448: loop { CTRL = 0x4000; read HOST_INT; if 0x8000 set, ack }
 * until 0x8000 clears. Each time the chip raises 0x8000 it has a character
 * for us at 0x50 (the monitor's output register); read it before acking.
 * The monitor's text is only logged when debugging. A timeout is not fatal.
 */
static int jedi_monitor_flush(struct sdio_func *func, const char *where)
{
	char text[33];
	unsigned int n = 0, i;
	u16 status = 0;
	int ret, err;

	for (i = 0; i < JEDI_FLUSH_MAX; i++) {
		ret = jedi_ctrl(func, JEDI_CTRL_FLUSH);
		if (ret)
			return ret;
		ret = jedi_host_int(func, &status);
		if (ret)
			return ret;
		if (!(status & JEDI_HOST_INT_TX_BUSY))
			break;
		{
			u8 b = sdio_readb(func, JEDI_REG_TX_READY, &err);

			if (!err && n < sizeof(text) - 1)
				text[n++] = (b >= 0x20 && b < 0x7f) ? b : '.';
		}
		ret = jedi_ack(func, 0xffffffff);
		if (ret)
			return ret;
	}
	text[n] = '\0';
	jedi_dbg(func, "monitor reply [%s]: \"%s\"\n", where, text);

	if (i == JEDI_FLUSH_MAX)
		dev_warn(&func->dev,
			 "jedi_fw: flush: TX_BUSY still set after %u "
			 "iterations (HOST_INT=0x%04x), continuing\n",
			 i, status);
	return 0;
}

/*
 * FUN_1004e61c -> FUN_1004e4b8: command-string byte.
 *   write byte to 0x4c
 *   CTRL=0x8000; wait HOST_INT&0x4000; ack
 *   CTRL=0x4000; wait HOST_INT&0x8000; ack
 *   read 0x50 (the monitor's echo)
 * Misses are advisory.
 */
static int jedi_monitor_send_cmd_byte(struct sdio_func *func, u8 byte)
{
	int ret, err;

	ret = jedi_monitor_writeb(func, byte);
	if (ret)
		return ret;

	ret = jedi_ctrl(func, JEDI_CTRL_BYTE_STROBE);
	if (ret)
		return ret;
	ret = jedi_poll_mask_ack(func, JEDI_HOST_INT_BYTE_OK);
	if (ret < 0)
		return ret;
	if (ret)
		jedi_mon_stats.cmd_hit++;
	else
		jedi_mon_stats.cmd_miss++;

	ret = jedi_ctrl(func, JEDI_CTRL_FLUSH);
	if (ret)
		return ret;
	ret = jedi_poll_mask_ack(func, JEDI_HOST_INT_TX_BUSY);
	if (ret < 0)
		return ret;

	(void)sdio_readb(func, JEDI_REG_TX_READY, &err);
	return err;
}

/*
 * FUN_1004e678 -> FUN_1004e584: payload byte.
 *   write byte to 0x4c; CTRL=0x8000; check HOST_INT&0x4000; ack
 * WinCE discards the result of this handshake (R2.3 correction), so a
 * miss is only counted, never fatal.
 */
static int jedi_monitor_send_data_byte(struct sdio_func *func, u8 byte)
{
	int ret;

	ret = jedi_monitor_writeb(func, byte);
	if (ret)
		return ret;

	ret = jedi_ctrl(func, JEDI_CTRL_BYTE_STROBE);
	if (ret)
		return ret;

	ret = jedi_poll_mask_ack(func, JEDI_HOST_INT_BYTE_OK);
	if (ret < 0)
		return ret;

	if (ret)
		jedi_mon_stats.data_hit++;
	else
		jedi_mon_stats.data_miss++;
	return 0;
}

/*
 * jedi_monitor_write() -- R2.3 E, FUN_1004e6d0 flagged-chunk branch.
 *
 *  E.1  clear pending
 *  E.2  32-bit unlock 0x12345678 to 0x4c..0x4f
 *  E.3  clear pending
 *  E.4  ASCII command, byte by byte (light handshake, advisory)
 *  E.5  flush
 *  E.6  payload, byte by byte (advisory acks)
 *  E.7  "1234"
 *  E.8  flush
 *  E.9  DEVSTAT |= 0x30000000; DEVSTAT &= ~0x10000000
 *  E.10 wait HOST_INT&0x2000, ack 0x2000  <-- the ONLY fatal check
 *
 * Note the command claims 0x9d4 bytes but, like WinCE, we send only the
 * chunk's own 2488 bytes plus "1234" and then reset into RAM boot: the
 * monitor never completes the command and that is fine. Padding to the
 * claimed length does not help.
 *
 * Must be preceded by jedi_chip_reset() so the boot ROM is running.
 */
static int jedi_monitor_write(struct sdio_func *func,
			      const u8 *data, u32 len)
{
	static const char cmd[] = JEDI_MONITOR_CMD;
	static const char tag[] = "1234";
	u32 ds = 0;
	u16 status = 0;
	unsigned int i;
	int ret;

	memset(&jedi_mon_stats, 0, sizeof(jedi_mon_stats));

	/* E.1 */
	ret = jedi_monitor_clear_pending(func);
	if (ret)
		return ret;

	/* E.2 */
	ret = jedi_reg_write(func, JEDI_REG_MONITOR_FIFO, 4, 0x12345678);
	if (ret) {
		dev_err(&func->dev,
			"jedi_fw: monitor unlock write failed: %d\n", ret);
		return ret;
	}

	/* E.3 */
	ret = jedi_monitor_clear_pending(func);
	if (ret)
		return ret;

	/* E.4 */
	for (i = 0; cmd[i] != '\0'; i++) {
		ret = jedi_monitor_send_cmd_byte(func, cmd[i]);
		if (ret) {
			dev_err(&func->dev,
				"jedi_fw: command byte %u failed: %d\n", i, ret);
			return ret;
		}
	}
	jedi_dbg(func, "monitor command: %u/%u bytes acked\n",
		 jedi_mon_stats.cmd_hit,
		 jedi_mon_stats.cmd_hit + jedi_mon_stats.cmd_miss);

	/* E.5 */
	ret = jedi_monitor_flush(func, "after cmd");
	if (ret)
		return ret;

	/* E.6 */
	for (i = 0; i < len; i++) {
		ret = jedi_monitor_send_data_byte(func, data[i]);
		if (ret) {
			dev_err(&func->dev,
				"jedi_fw: monitor payload bus error at "
				"byte %u/%u: %d\n", i, len, ret);
			jedi_dump_regs(func, "payload failure");
			return ret;
		}
	}
	jedi_dbg(func, "monitor payload: %u acked, %u missed (advisory)\n",
		 jedi_mon_stats.data_hit, jedi_mon_stats.data_miss);

	/* E.7 */
	for (i = 0; tag[i] != '\0'; i++) {
		ret = jedi_monitor_send_data_byte(func, tag[i]);
		if (ret) {
			dev_err(&func->dev,
				"jedi_fw: monitor trailing tag failed: %d\n", ret);
			return ret;
		}
	}

	/* E.8 */
	ret = jedi_monitor_flush(func, "after payload");
	if (ret)
		return ret;

	/* E.9: leaves RAMBOOT (0x20000000) set, RESET released. */
	ret = jedi_devstat_rmw(func, JEDI_DEVSTAT_RESET | JEDI_DEVSTAT_RAMBOOT,
			       0, &ds);
	if (ret)
		return ret;
	ret = jedi_devstat_rmw(func, 0, JEDI_DEVSTAT_RESET, &ds);
	if (ret)
		return ret;

	/* E.10: the only fatal check in the monitor path. */
	ret = jedi_wait_host_int(func, JEDI_HOST_INT_BULK_OK,
				 JEDI_MON_DONE_TIMEOUT_MS, &status);
	if (ret == -ETIMEDOUT) {
		dev_err(&func->dev,
			"jedi_fw: monitor image NOT accepted: HOST_INT 0x2000 "
			"never set (last HOST_INT=0x%04x)\n", status);
		jedi_dump_regs(func, "monitor timeout");
		return ret;
	}
	if (ret)
		return ret;

	ret = jedi_ack(func, JEDI_HOST_INT_BULK_OK);
	if (ret)
		return ret;

	jedi_dbg(func, "monitor image accepted (HOST_INT=0x%04x)\n", status);
	return 0;
}

/* ------------------------------------------------------------------ */
/* Boot confirmation (R2.4)                                            */
/* ------------------------------------------------------------------ */

/*
 * After the last chunk WinCE writes INT_ENABLE = 0xFFFF and waits up to
 * 10 s for HOST_INT bit 0x0001 ("FW Init"). We poll instead of using the
 * SDIO IRQ, so no interrupt handler needs to exist yet.
 */
static int jedi_wait_fw_boot(struct sdio_func *func)
{
	unsigned long deadline = jiffies + msecs_to_jiffies(JEDI_BOOT_TIMEOUT_MS);
	u16 status = 0;
	int ret;

	ret = jedi_reg_write(func, JEDI_REG_INT_ENABLE, 2, 0xffff);
	if (ret) {
		dev_err(&func->dev, "jedi_fw: INT_ENABLE write failed: %d\n",
			ret);
		return ret;
	}

	do {
		ret = jedi_host_int(func, &status);
		if (ret)
			return ret;

		if (status & JEDI_HOST_INT_FW_INIT) {
			ret = jedi_reg_write(func, JEDI_REG_HOST_INT_ACK, 2,
					     status);
			dev_info(&func->dev, "jedi_fw: firmware booted\n");
			return ret;
		}
		msleep(10);
	} while (time_before(jiffies, deadline));

	dev_err(&func->dev,
		"jedi_fw: no FW Init within %u ms (last HOST_INT=0x%04x)\n",
		JEDI_BOOT_TIMEOUT_MS, status);
	jedi_dump_regs(func, "boot timeout");
	return -ETIMEDOUT;
}

/* ------------------------------------------------------------------ */
/* Main entry point                                                    */
/* ------------------------------------------------------------------ */

/*
 * jedi_fw_download() - reset the chip into its boot ROM, push the full
 * firmware image, and wait for the firmware to report FW Init.
 *
 * Use the raw chunked container JEDI10_dll.bin (headers intact) placed
 * at /lib/firmware/jedi_fw.bin. Do NOT use cx50222_firmware.bin.
 *
 * The caller must NOT hold the SDIO host; function 1 must be enabled and
 * the block size set. Returns 0 only once the firmware has booted. The
 * SDIO function IRQ should be claimed AFTER this returns.
 */
int jedi_fw_download(struct sdio_func *func)
{
	const struct firmware *fw;
	const u8 *pos, *end;
	unsigned int chunk_num = 0;
	int ret;

	ret = request_firmware(&fw, JEDI_FW_NAME, &func->dev);
	if (ret) {
		dev_err(&func->dev,
			"jedi_fw: request_firmware(%s) failed: %d\n",
			JEDI_FW_NAME, ret);
		return ret;
	}

	pos = fw->data;
	end = fw->data + fw->size;

	dev_info(&func->dev, "jedi_fw: downloading firmware (%zu bytes)\n",
		 fw->size);
	jedi_dbg(func, "func1 max_blksize=%u cur_blksize=%u\n",
		 func->max_blksize, func->cur_blksize);

	sdio_claim_host(func);

	ret = jedi_chip_reset(func);
	if (ret) {
		dev_err(&func->dev, "jedi_fw: chip reset failed: %d\n", ret);
		goto out;
	}

	while (pos < end) {
		const struct jedi_fw_chunk *chunk;
		u32 addr, len;
		bool monitor;

		if ((size_t)(end - pos) < sizeof(*chunk)) {
			dev_err(&func->dev,
				"jedi_fw: truncated header at file offset %td\n",
				pos - fw->data);
			ret = -EINVAL;
			goto out;
		}

		chunk = (const struct jedi_fw_chunk *)pos;
		addr  = le32_to_cpu(chunk->address);
		len   = le32_to_cpu(chunk->length);
		monitor = !!(addr & JEDI_CHUNK_MONITOR_FLAG);

		/* Compared this way round so a huge len cannot wrap. */
		if (len > (size_t)(end - pos) - sizeof(*chunk)) {
			dev_err(&func->dev,
				"jedi_fw: chunk %u claims %u bytes but only "
				"%zu remain\n",
				chunk_num, len,
				(size_t)(end - pos) - sizeof(*chunk));
			ret = -EINVAL;
			goto out;
		}

		jedi_dbg(func, "chunk %u: addr=0x%08x len=%u%s\n",
			 chunk_num, addr, len, monitor ? " [monitor path]" : "");

		if (monitor)
			ret = jedi_monitor_write(func, chunk->payload, len);
		else
			ret = jedi_download_chunk(func, chunk->payload, len,
						  chunk_num,
						  (size_t)(end - pos) ==
						  sizeof(*chunk) + len);
		if (ret)
			goto out;

		pos += sizeof(*chunk) + len;
		chunk_num++;
	}

	jedi_dbg(func, "download complete (%u chunks, %zu bytes)\n",
		 chunk_num, fw->size);

	ret = jedi_wait_fw_boot(func);

out:
	sdio_release_host(func);
	release_firmware(fw);
	return ret;
}
EXPORT_SYMBOL_GPL(jedi_fw_download);