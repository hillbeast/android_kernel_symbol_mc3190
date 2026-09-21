// SPDX-License-Identifier: GPL-2.0
/*
 * jedi_fw.c -- Firmware download for the Symbol "Jedi" (Conexant CX50222)
 * SDIO WiFi module, MC3190.
 *
 * Protocol fully reverse-engineered from JEDI10.dll (WinCE miniport),
 * specifically FUN_1004e6d0 ("DownloadFirmware") and its subordinates
 * FUN_1004e61c, FUN_1004e678, FUN_1004e3f4, FUN_1004e448, FUN_1004e4b8,
 * FUN_1004e584.
 *
 * Firmware container format (JEDI10_dll.bin, raw / unstripped):
 *
 *   repeating, back to back, until all declared bytes consumed:
 *     u32 address   (LE; bit 31 set = "monitor path" chunk)
 *     u32 length    (LE, payload byte count)
 *     u8  payload[length]
 *
 * Two distinct write paths exist:
 *
 * BULK PATH (bit 31 of address clear):
 *   CMD53 byte-mode writes to SDIO Func 1 addr 0x0 in <=512 byte bursts.
 *   After each burst: poll REG_HOST_INT (0x40) for bit 0x2000, then ack
 *   via REG_HOST_INT_ACK (0x48).
 *
 * MONITOR PATH (bit 31 of address set, only chunk 0 in practice):
 *   Uses a byte-at-a-time ASCII ROM monitor protocol over register 0x4c.
 *   Fully confirmed against WinCE disassembly. See jedi_monitor_write().
 *
 * Register map (confirmed):
 *   0x38 W  Control/strobe: 0x8000="byte sent, sample it"; 0x4000="flush"
 *   0x40 R  REG_HOST_INT: 0x2000=bulk-block-ok; 0x4000=byte-tx-ok;
 *                          0x8000=TX-buffer-draining
 *   0x48 W  REG_HOST_INT_ACK: write 0x2000 to ack bulk block;
 *                              write 0xffffffff to clear all pending
 *   0x4c W  Monitor byte FIFO (one byte per write, also used for the
 *           32-bit unlock magic 0x12345678 as an initial wider write)
 *   0x54 R/W Strobe/trigger used at end of monitor chunk
 */

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/sdio_func.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#define JEDI_FW_NAME		"jedi_fw.bin"

/* REG_HOST_INT (0x40) bit flags */
#define JEDI_HOST_INT_BULK_OK	0x2000	/* bulk block accepted */
#define JEDI_HOST_INT_BYTE_OK	0x4000	/* monitor byte accepted */
#define JEDI_HOST_INT_TX_BUSY	0x8000	/* TX buffer still draining */

/* Register addresses */
#define JEDI_REG_CTRL		0x38
#define JEDI_REG_HOST_INT	0x40
#define JEDI_REG_HOST_INT_ACK	0x48
#define JEDI_REG_MONITOR_FIFO	0x4c
#define JEDI_REG_TRIGGER	0x54

/* CTRL register strobe values */
#define JEDI_CTRL_BYTE_STROBE	0x8000
#define JEDI_CTRL_FLUSH		0x4000

/* Chunk address flag: bit 31 set = use monitor path */
#define JEDI_CHUNK_MONITOR_FLAG	0x80000000UL

/* Maximum burst size for bulk CMD53 path.
 * Matches the 0x1ff (511) mask used throughout JEDI10.dll — the CMD53
 * 9-bit byte-count field gives a natural 512-byte ceiling. */
#define JEDI_MAX_BURST		512

/* Bulk-path block handshake timeout. WinCE busy-polls up to 0xffff times
 * with a 1us stall (~65ms worst case). We use udelay() to match — 2.6.35
 * does not have usleep_range(). Total budget matches WinCE with margin. */
#define JEDI_BULK_POLL_US	1
#define JEDI_BULK_POLL_MAX	0xffff

struct jedi_fw_chunk {
	__le32 address;
	__le32 length;
	u8 payload[];
} __packed;

/* Diagnostic module param: skip the monitor-path chunk instead of
 * aborting, to allow bulk-path transport testing before the chip boots.
 * The chip will NOT produce a working firmware load with this set. */
static bool jedi_fw_skip_monitor;
module_param(jedi_fw_skip_monitor, bool, 0644);
MODULE_PARM_DESC(jedi_fw_skip_monitor,
	"DIAGNOSTIC: skip monitor-path chunk; chip will not boot. "
	"Transport test use only.");

/* ------------------------------------------------------------------ */
/* Low-level register helpers                                          */
/* ------------------------------------------------------------------ */

static int jedi_reg_read(struct sdio_func *func, u32 addr, u32 *val)
{
	int ret;
	*val = sdio_readl(func, addr, &ret);
	return ret;
}

static int jedi_reg_write(struct sdio_func *func, u32 addr, u32 val)
{
	int ret;
	sdio_writel(func, val, addr, &ret);
	return ret;
}

/* ------------------------------------------------------------------ */
/* Bulk path                                                           */
/* ------------------------------------------------------------------ */

/*
 * jedi_wait_block_accepted() - poll REG_HOST_INT for bit 0x2000 and
 * acknowledge. Matches DownloadFirmware's inner poll loop exactly,
 * using udelay() as required for 2.6.35.
 */
static int jedi_wait_block_accepted(struct sdio_func *func)
{
	u32 status;
	unsigned int i;
	int ret;

	for (i = 0; i < JEDI_BULK_POLL_MAX; i++) {
		ret = jedi_reg_read(func, JEDI_REG_HOST_INT, &status);
		if (ret) {
			dev_err(&func->dev,
				"jedi_fw: REG_HOST_INT read failed: %d\n", ret);
			return ret;
		}

		if (status & JEDI_HOST_INT_BULK_OK) {
			ret = jedi_reg_write(func, JEDI_REG_HOST_INT_ACK,
					     JEDI_HOST_INT_BULK_OK);
			if (ret)
				dev_err(&func->dev,
					"jedi_fw: bulk ack write failed: %d\n",
					ret);
			return ret;
		}

		udelay(JEDI_BULK_POLL_US);
	}

	dev_err(&func->dev,
		"jedi_fw: timed out waiting for bulk block acceptance "
		"(REG_HOST_INT=0x%x)\n", status);
	return -ETIMEDOUT;
}

/*
 * jedi_download_chunk() - send one bulk-path firmware chunk.
 *
 * Uses a kmalloc'd bounce buffer: sdio_writesb() calls into the PXA320
 * MMC DMA engine which requires a physically contiguous buffer;
 * request_firmware() data is vmalloc-backed and will oops in
 * dma_cache_maint_page otherwise (confirmed on first hardware run,
 * 2.6.35/PXA320).
 */
static int jedi_download_chunk(struct sdio_func *func,
			       const u8 *data, u32 len,
			       unsigned int chunk_num)
{
	u8 *bounce;
	u32 sent = 0;
	unsigned int burst_num = 0;
	int ret;

	bounce = kmalloc(JEDI_MAX_BURST, GFP_KERNEL);
	if (!bounce)
		return -ENOMEM;

	while (sent < len) {
		u32 burst = min_t(u32, len - sent, JEDI_MAX_BURST);

		memcpy(bounce, data + sent, burst);

		dev_dbg(&func->dev,
			"jedi_fw: chunk %u burst %u: %u bytes (offset %u/%u)\n",
			chunk_num, burst_num, burst, sent, len);

		ret = sdio_writesb(func, 0x0, bounce, burst);
		if (ret) {
			dev_err(&func->dev,
				"jedi_fw: chunk %u burst write failed at "
				"offset %u/%u: %d\n",
				chunk_num, sent, len, ret);
			goto out;
		}

		ret = jedi_wait_block_accepted(func);
		if (ret) {
			dev_err(&func->dev,
				"jedi_fw: chunk %u block handshake failed at "
				"offset %u/%u\n", chunk_num, sent, len);
			goto out;
		}

		sent += burst;
		burst_num++;
	}

	dev_info(&func->dev,
		 "jedi_fw: chunk %u complete: %u bytes in %u bursts\n",
		 chunk_num, len, burst_num);
	ret = 0;

out:
	kfree(bounce);
	return ret;
}

/* ------------------------------------------------------------------ */
/* Monitor path                                                        */
/* ------------------------------------------------------------------ */

/*
 * jedi_monitor_clear_pending() - equivalent to FUN_1004e3f4.
 * If REG_HOST_INT bit 0x4000 is already set, clear it before
 * starting a monitor transaction.
 */
static int jedi_monitor_clear_pending(struct sdio_func *func)
{
	u32 status;
	int ret;

	ret = jedi_reg_read(func, JEDI_REG_HOST_INT, &status);
	if (ret)
		return ret;

	if (status & JEDI_HOST_INT_BYTE_OK)
		ret = jedi_reg_write(func, JEDI_REG_HOST_INT_ACK, 0xffffffff);

	return ret;
}

/*
 * jedi_monitor_flush() - equivalent to FUN_1004e448.
 * Triggers a TX-flush and waits until the TX buffer drains.
 * Used between the ASCII command string and the payload, and after
 * the payload.
 */
static int jedi_monitor_flush(struct sdio_func *func)
{
	u32 status;
	int ret;

	do {
		ret = jedi_reg_write(func, JEDI_REG_CTRL, JEDI_CTRL_FLUSH);
		if (ret)
			return ret;

		ret = jedi_reg_read(func, JEDI_REG_HOST_INT, &status);
		if (ret)
			return ret;

		if (status & JEDI_HOST_INT_TX_BUSY) {
			ret = jedi_reg_write(func, JEDI_REG_HOST_INT_ACK,
					     0xffffffff);
			if (ret)
				return ret;
		}
	} while (status & JEDI_HOST_INT_TX_BUSY);

	return 0;
}

/*
 * jedi_monitor_send_cmd_byte() - equivalent to FUN_1004e61c.
 * Sends one byte of the ASCII command string to the monitor FIFO.
 * Non-critical path (ROM monitor buffers command bytes) — fires two
 * strobe/check sequences but does not abort on a missed ack.
 */
static int jedi_monitor_send_cmd_byte(struct sdio_func *func, u8 byte)
{
	u32 status;
	int ret;

	ret = jedi_reg_write(func, JEDI_REG_MONITOR_FIFO, byte);
	if (ret)
		return ret;

	/* First strobe/check — 0x8000 to 0x38, check 0x4000 in 0x40 */
	ret = jedi_reg_write(func, JEDI_REG_CTRL, JEDI_CTRL_BYTE_STROBE);
	if (ret)
		return ret;
	ret = jedi_reg_read(func, JEDI_REG_HOST_INT, &status);
	if (ret)
		return ret;
	if (status & JEDI_HOST_INT_BYTE_OK)
		jedi_reg_write(func, JEDI_REG_HOST_INT_ACK, 0xffffffff);
	/* missed ack on cmd bytes is non-fatal — fall through */

	/* Second strobe/check — 0x4000 to 0x38, check 0x8000 in 0x40 */
	ret = jedi_reg_write(func, JEDI_REG_CTRL, JEDI_CTRL_FLUSH);
	if (ret)
		return ret;
	ret = jedi_reg_read(func, JEDI_REG_HOST_INT, &status);
	if (ret)
		return ret;
	if (status & JEDI_HOST_INT_TX_BUSY)
		jedi_reg_write(func, JEDI_REG_HOST_INT_ACK, 0xffffffff);

	return 0;
}

/*
 * jedi_monitor_send_data_byte() - equivalent to FUN_1004e678.
 * Sends one payload byte to the monitor FIFO.
 * Strict path — returns an error if the byte-ok ack is not received,
 * matching WinCE's behaviour for the payload (every byte matters).
 */
static int jedi_monitor_send_data_byte(struct sdio_func *func, u8 byte)
{
	u32 status;
	int ret;

	ret = jedi_reg_write(func, JEDI_REG_MONITOR_FIFO, byte);
	if (ret)
		return ret;

	ret = jedi_reg_write(func, JEDI_REG_CTRL, JEDI_CTRL_BYTE_STROBE);
	if (ret)
		return ret;

	ret = jedi_reg_read(func, JEDI_REG_HOST_INT, &status);
	if (ret)
		return ret;

	if (!(status & JEDI_HOST_INT_BYTE_OK)) {
		dev_err(&func->dev,
			"jedi_fw: monitor payload byte 0x%02x not accepted "
			"(HOST_INT=0x%x)\n", byte, status);
		return -EIO;
	}

	return jedi_reg_write(func, JEDI_REG_HOST_INT_ACK, 0xffffffff);
}

/*
 * jedi_monitor_write() - send one monitor-path chunk to the chip's
 * ROM monitor. Equivalent to the flagged-address branch in
 * FUN_1004e6d0 (DownloadFirmware).
 *
 * The boot ROM monitor accepts a simple binary-upload command:
 *   "<XXXXXXXX YYY\r"  where XXXXXXXX is the hex target address and
 *                      YYY is the hex byte count
 * followed by the raw payload bytes and a trailing "1234" tag.
 *
 * All communication goes through register 0x4c (monitor FIFO), one
 * byte at a time, with strobing via register 0x38 and handshaking via
 * REG_HOST_INT (0x40)/REG_HOST_INT_ACK (0x48).
 */
static int jedi_monitor_write(struct sdio_func *func,
			      u32 target_addr, const u8 *data, u32 len)
{
	/* Monitor command string fields, hard-coded per WinCE:
	 *   address = 0x1ffe0000 (boot ROM trampoline location)
	 *   count   = len in hex
	 * WinCE hard-codes these as literal ASCII arrays in FUN_1004e6d0. */
	char cmd[32];
	u32 status;
	u32 i;
	int ret;

	dev_info(&func->dev,
		 "jedi_fw: monitor path: writing %u bytes to 0x%08x\n",
		 len, target_addr);

	/* Step 1: clear any pending interrupt state */
	ret = jedi_monitor_clear_pending(func);
	if (ret)
		return ret;

	/* Step 2: send the 32-bit unlock magic to register 0x4c.
	 * This transitions the boot ROM from idle to monitor mode. */
	ret = jedi_reg_write(func, JEDI_REG_MONITOR_FIFO, 0x12345678);
	if (ret) {
		dev_err(&func->dev,
			"jedi_fw: monitor unlock write failed: %d\n", ret);
		return ret;
	}

	/* Step 3: clear pending again after unlock */
	ret = jedi_monitor_clear_pending(func);
	if (ret)
		return ret;

	/* Step 4: send ASCII command string byte by byte.
	 * Format: "<XXXXXXXX YYY\r" matching WinCE's literal string.
	 * WinCE uses the hardcoded address 0x1ffe0000 and derives the
	 * count from the chunk length field. */
	snprintf(cmd, sizeof(cmd), "<%08x %x\r", target_addr, len);
	dev_dbg(&func->dev, "jedi_fw: monitor command: %s", cmd);

	for (i = 0; cmd[i] != '\0'; i++) {
		ret = jedi_monitor_send_cmd_byte(func, cmd[i]);
		if (ret)
			return ret;
	}

	/* Step 5: flush — signals end of command line */
	ret = jedi_monitor_flush(func);
	if (ret)
		return ret;

	/* Step 6: send payload bytes one at a time */
	for (i = 0; i < len; i++) {
		ret = jedi_monitor_send_data_byte(func, data[i]);
		if (ret) {
			dev_err(&func->dev,
				"jedi_fw: monitor payload failed at byte %u/%u\n",
				i, len);
			return ret;
		}
	}

	/* Step 7: send trailing "1234" tag (ASCII, byte-by-byte) */
	ret  = jedi_monitor_send_data_byte(func, '1');
	ret |= jedi_monitor_send_data_byte(func, '2');
	ret |= jedi_monitor_send_data_byte(func, '3');
	ret |= jedi_monitor_send_data_byte(func, '4');
	if (ret) {
		dev_err(&func->dev, "jedi_fw: monitor trailing tag failed\n");
		return ret;
	}

	/* Step 8: final flush */
	ret = jedi_monitor_flush(func);
	if (ret)
		return ret;

	/* Step 9: register 0x54 toggle: read, OR 0x30000000, write;
	 * AND 0xefffffff, write; read.
	 * Signals to the boot ROM that the upload is complete. */
	ret = jedi_reg_read(func, JEDI_REG_TRIGGER, &status);
	if (ret)
		return ret;
	status |= 0x30000000;
	ret = jedi_reg_write(func, JEDI_REG_TRIGGER, status);
	if (ret)
		return ret;
	status &= 0xefffffff;
	ret = jedi_reg_write(func, JEDI_REG_TRIGGER, status);
	if (ret)
		return ret;
	ret = jedi_reg_read(func, JEDI_REG_TRIGGER, &status);

	if (!ret)
		dev_info(&func->dev, "jedi_fw: monitor write complete\n");

	return ret;
}

/* ------------------------------------------------------------------ */
/* debugfs monitor interface                                           */
/* ------------------------------------------------------------------ */

#ifdef CONFIG_DEBUG_FS

/*
 * The debugfs interface exposes the ROM monitor as a simple write-only
 * file: /sys/kernel/debug/jedi_fw/monitor_write
 *
 * Write a binary blob to this file to send it via jedi_monitor_write()
 * at the hardcoded boot-ROM address (0x1ffe0000). This is useful for
 * diagnostics before firmware boots — once firmware is running the
 * monitor is no longer available.
 *
 * Usage (from shell, after modprobe with chip in pre-boot state):
 *   cat vector_table.bin > /sys/kernel/debug/jedi_fw/monitor_write
 */

static struct sdio_func *jedi_debug_func;
static struct dentry *jedi_debugfs_dir;

static ssize_t jedi_dbg_monitor_write(struct file *file,
				      const char __user *ubuf,
				      size_t count, loff_t *pos)
{
	u8 *buf;
	int ret;

	if (!jedi_debug_func)
		return -ENODEV;

	if (count == 0 || count > 0x10000)
		return -EINVAL;

	buf = memdup_user(ubuf, count);
	if (IS_ERR(buf))
		return PTR_ERR(buf);

	sdio_claim_host(jedi_debug_func);
	ret = jedi_monitor_write(jedi_debug_func, 0x1ffe0000, buf, count);
	sdio_release_host(jedi_debug_func);

	kfree(buf);
	return ret ? ret : count;
}

static const struct file_operations jedi_dbg_monitor_fops = {
	.owner	= THIS_MODULE,
	.write	= jedi_dbg_monitor_write,
	.llseek	= no_llseek,
};

void jedi_debugfs_init(struct sdio_func *func)
{
	jedi_debug_func = func;
	jedi_debugfs_dir = debugfs_create_dir("jedi_fw", NULL);
	if (!IS_ERR_OR_NULL(jedi_debugfs_dir))
		debugfs_create_file("monitor_write", 0200, jedi_debugfs_dir,
				    NULL, &jedi_dbg_monitor_fops);
}
EXPORT_SYMBOL_GPL(jedi_debugfs_init);

void jedi_debugfs_remove(void)
{
	debugfs_remove_recursive(jedi_debugfs_dir);
	jedi_debugfs_dir = NULL;
	jedi_debug_func = NULL;
}
EXPORT_SYMBOL_GPL(jedi_debugfs_remove);

#else /* !CONFIG_DEBUG_FS */

void jedi_debugfs_init(struct sdio_func *func) {}
void jedi_debugfs_remove(void) {}

#endif /* CONFIG_DEBUG_FS */

/* ------------------------------------------------------------------ */
/* Main entry point                                                    */
/* ------------------------------------------------------------------ */

/*
 * jedi_fw_download() - parse and push the full firmware image.
 *
 * Use the raw chunked container JEDI10_dll.bin (headers intact) placed
 * at /lib/firmware/jedi_fw.bin. Do NOT use cx50222_firmware.bin — that
 * is the stripped/flattened Ghidra-analysis image with headers removed.
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

	dev_info(&func->dev, "jedi_fw: starting download, %zu bytes\n",
		 fw->size);

	sdio_claim_host(func);

	while (pos < end) {
		const struct jedi_fw_chunk *chunk;
		u32 addr, len;
		bool monitor;

		if (end - pos < (ptrdiff_t)sizeof(*chunk)) {
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

		if (end - pos < (ptrdiff_t)(sizeof(*chunk) + len)) {
			dev_err(&func->dev,
				"jedi_fw: chunk %u claims %u bytes but only "
				"%td remain\n",
				chunk_num, len,
				end - pos - (ptrdiff_t)sizeof(*chunk));
			ret = -EINVAL;
			goto out;
		}

		dev_info(&func->dev,
			 "jedi_fw: chunk %u: addr=0x%08x len=%u%s\n",
			 chunk_num, addr, len,
			 monitor ? " [monitor path]" : "");

		if (monitor) {
			if (jedi_fw_skip_monitor) {
				dev_warn(&func->dev,
					 "jedi_fw: SKIPPING monitor chunk "
					 "%u — diagnostic mode, chip will "
					 "NOT boot\n", chunk_num);
				ret = 0;
			} else {
				ret = jedi_monitor_write(func,
					0x1ffe0000,	chunk->payload, len);
			}
		} else {
			ret = jedi_download_chunk(func, chunk->payload,
						  len, chunk_num);
		}

		if (ret)
			goto out;

		pos += sizeof(*chunk) + len;
		chunk_num++;
	}

	dev_info(&func->dev,
		 "jedi_fw: download complete (%u chunks, %zu bytes)\n",
		 chunk_num, fw->size);
	ret = 0;

out:
	sdio_release_host(func);
	release_firmware(fw);
	return ret;
}
EXPORT_SYMBOL_GPL(jedi_fw_download);