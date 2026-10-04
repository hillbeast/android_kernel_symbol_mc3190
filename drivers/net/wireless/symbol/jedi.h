/*
 * jedi.h -- Header for Symbol "Jedi" WiFi module in MC3190
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

#ifndef SYMBOL_JEDI_H
#define SYMBOL_JEDI_H

#include <linux/types.h>
#include <linux/mmc/sdio_func.h>

/*
 * Control registers, SDIO function 1. Accessed with CMD52, one byte at a
 * time at ascending addresses, little-endian (glossary R2.1). Only the
 * data port (function 1, address 0x0) is CMD53 fixed-address.
 */
#define JEDI_REG_STATUS		    0x04	/* R8: bit 2 = HOST_INT pending */
#define JEDI_REG_CTRL		    0x38	/* W32: strobe 0x8000 / flush 0x4000 */
#define JEDI_REG_HOST_INT	    0x40	/* R16: device-to-host status */
#define JEDI_REG_INT_ENABLE	    0x44	/* W16: host interrupt mask */
#define JEDI_REG_HOST_INT_ACK	0x48	/* W32: ack (0x2000 / 0xffffffff) */
#define JEDI_REG_MONITOR_FIFO	0x4c	/* W8: monitor byte; W32 unlock */
#define JEDI_REG_TX_READY	    0x50	/* R: read after each cmd byte */
#define JEDI_REG_DEVSTAT	    0x54	/* R/W32: reset/boot control */

/* JEDI_REG_STATUS bits */
#define JEDI_STATUS_HOST_INT	0x04

/* JEDI_REG_HOST_INT bits (byte 0x41 holds 0x2000/0x4000/0x8000) */
#define JEDI_HOST_INT_FW_INIT	0x0001	/* firmware booted (R2.4) */
#define JEDI_HOST_INT_BULK_OK	0x2000	/* bulk block / monitor image accepted */
#define JEDI_HOST_INT_BYTE_OK	0x4000	/* monitor byte accepted */
#define JEDI_HOST_INT_TX_BUSY	0x8000	/* TX buffer still draining */

/* JEDI_REG_CTRL strobe values */
#define JEDI_CTRL_BYTE_STROBE	0x8000
#define JEDI_CTRL_FLUSH		    0x4000

/*
 * JEDI_REG_DEVSTAT bits. Roles are [INF] by analogy with p54pci.h
 * (ISL38XX_CTRL_STAT_*): RESET pulse with RAMBOOT clear boots the ROM,
 * with RAMBOOT set runs the uploaded RAM image.
 */
#define JEDI_DEVSTAT_WAIT	    0x00008000	/* polled to clear before reset */
#define JEDI_DEVSTAT_RESET	    0x10000000
#define JEDI_DEVSTAT_RAMBOOT	0x20000000

/* Verbose logging: boot the kernel with symbol.debug=1, or at runtime
 * echo 1 > /sys/module/symbol/parameters/debug */
extern bool jedi_debug;

#define jedi_dbg(func, fmt, ...)					\
	do {								\
		if (jedi_debug)						\
			dev_info(&(func)->dev, "jedi: " fmt, ##__VA_ARGS__); \
	} while (0)

/* CMD52 register helpers, defined in jedi_fw.c. n = 1..4 bytes. */
int jedi_reg_read(struct sdio_func *func, u32 addr, unsigned int n, u32 *val);
int jedi_reg_write(struct sdio_func *func, u32 addr, unsigned int n, u32 val);

/* Reset the chip into its boot ROM, upload firmware, wait for FW Init. */
extern int jedi_fw_download(struct sdio_func *func);

#endif // SYMBOL_JEDI_H