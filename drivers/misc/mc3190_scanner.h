/*
 * mc3190_scanner.h -- user/kernel interface of /dev/scanner1d
 *
 * One read() returns exactly ONE sweep record:
 *
 *     struct mc3190_scan_header   (20 bytes)
 *     __u16 widths[pulse_count]   (run lengths, in 1/tick_hz seconds)
 *
 * widths[0] is the first run and has the DBP level 'initial_level'; levels
 * then alternate.  (Which level is "bar" depends on the engine polarity; the
 * userspace decoder tries both.)  A record is produced for every SOS toggle,
 * i.e. every sweep direction, so alternate records are mirror images.
 */
#ifndef MC3190_SCANNER_H
#define MC3190_SCANNER_H

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
typedef uint16_t __u16;
typedef uint32_t __u32;
#endif

#define MC3190_SCAN_F_TRUNCATED          0x00000001U /* >MAX_RUNS runs, tail dropped */
#define MC3190_SCAN_F_QUEUE_OVERFLOW     0x00000002U /* earlier record(s) lost */
#define MC3190_SCAN_F_INITIAL_LEVEL_BAD  0x00000004U
#define MC3190_SCAN_F_RUN_CLAMPED        0x00000008U /* a run was > 65535 ticks */
#define MC3190_SCAN_F_DMA_OVERRUN        0x00000010U /* CI FIFO/DMA error seen */
#define MC3190_SCAN_F_GPIO_MODE          0x00000020U /* ticks are us, GPIO fallback */

#define MC3190_SCAN_MAX_RUNS   1024

struct mc3190_scan_header {
	__u32 sequence;
	__u32 pulse_count;
	__u32 flags;
	__u32 initial_level;
	__u32 tick_hz;
};

#define MC3190_SCAN_MAX_RECORD \
	(sizeof(struct mc3190_scan_header) + MC3190_SCAN_MAX_RUNS * sizeof(__u16))

#endif