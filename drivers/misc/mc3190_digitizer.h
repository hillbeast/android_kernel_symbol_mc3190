/*
 * mc3190_digitizer.h -- sample stream -> per-sweep run-length records
 *
 * Header-only, no kernel dependencies except u8/u16/u32 so the very same code
 * is built into the kernel driver and into the host-side test harness.
 *
 * Input: one byte per CI pixel clock.  Two bits matter (defaults, from the
 * WinCE CamScanPDD registry fallbacks):  SosMask 0x80 (CIF_D7), DbpMask 0x01
 * (CIF_D0).  SOS toggles once per sweep; DBP is the digitised bar pattern.
 *
 * DBP glitch filter: a level change is accepted only if the new level persists
 * for min_run samples; shorter excursions are folded into the current run.
 * SOS is debounced the same way (sos_min samples).
 */
#ifndef MC3190_DIGITIZER_H
#define MC3190_DIGITIZER_H

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#include <stddef.h>
typedef uint8_t u8; typedef uint16_t u16; typedef uint32_t u32;
#endif
#include "mc3190_scanner.h"

struct mc3190_dig_sweep {
	u32 initial_level;
	u32 count;
	u32 flags;
	u16 w[MC3190_SCAN_MAX_RUNS];
};

struct mc3190_dig {
	/* config */
	u8  sos_mask, dbp_mask;
	u32 min_run;		/* DBP debounce, samples (>=1) */
	u32 sos_min;		/* SOS debounce, samples (>=1) */
	u32 min_runs_emit;	/* drop sweeps with fewer runs (empty sweeps) */
	void (*emit)(void *ctx, struct mc3190_dig_sweep *sw);
	void *ctx;

	/* state */
	u8  last_key;
	u8  sos_init, in_sweep;
	u8  sos_level, dbp_level;
	u32 sos_pend, pend, run;
	u32 sweeps_seen, sweeps_emitted;
	struct mc3190_dig_sweep sw;
};

static inline void mc3190_dig_reset(struct mc3190_dig *d)
{
	d->last_key = 0;
	d->sos_init = d->in_sweep = 0;
	d->sos_level = d->dbp_level = 0;
	d->sos_pend = d->pend = d->run = 0;
	d->sw.count = 0;
	d->sw.flags = 0;
}

static inline void mc3190_dig_push(struct mc3190_dig *d, u32 run)
{
	if (!run)
		return;
	if (run > 0xffff) {
		run = 0xffff;
		d->sw.flags |= MC3190_SCAN_F_RUN_CLAMPED;
	}
	if (d->sw.count < MC3190_SCAN_MAX_RUNS)
		d->sw.w[d->sw.count++] = (u16)run;
	else
		d->sw.flags |= MC3190_SCAN_F_TRUNCATED;
}

static inline void mc3190_dig_end_sweep(struct mc3190_dig *d)
{
	if (!d->in_sweep)
		return;
	mc3190_dig_push(d, d->run + d->pend);
	d->run = d->pend = 0;
	d->sweeps_seen++;
	if (d->sw.count >= d->min_runs_emit) {
		d->sweeps_emitted++;
		d->emit(d->ctx, &d->sw);
	}
	d->in_sweep = 0;
}

static inline void mc3190_dig_begin_sweep(struct mc3190_dig *d, u8 dbp)
{
	d->sw.count = 0;
	d->sw.flags = 0;
	d->sw.initial_level = dbp;
	d->dbp_level = dbp;
	d->run = d->pend = 0;
	d->in_sweep = 1;
}

static inline void mc3190_dig_feed(struct mc3190_dig *d, const u8 *buf, u32 len)
{
	const u8 keymask = d->sos_mask | d->dbp_mask;
	u32 i;

	for (i = 0; i < len; i++) {
		u8 b = buf[i], key = b & keymask;
		u8 sl, dl;

		/* fast path: nothing changed and nothing pending */
		if (key == d->last_key && !d->pend && !d->sos_pend && d->sos_init) {
			d->run += d->in_sweep;
			continue;
		}
		d->last_key = key;
		sl = !!(b & d->sos_mask);
		dl = !!(b & d->dbp_mask);

		if (!d->sos_init) {
			d->sos_init = 1;
			d->sos_level = sl;
			d->sos_pend = 0;
		} else if (sl != d->sos_level) {
			if (++d->sos_pend >= d->sos_min) {
				d->sos_level = sl;
				d->sos_pend = 0;
				mc3190_dig_end_sweep(d);
				mc3190_dig_begin_sweep(d, dl);
			}
		} else {
			d->sos_pend = 0;
		}

		if (!d->in_sweep)
			continue;

		if (dl == d->dbp_level) {
			d->run += d->pend + 1;	/* pending excursion was a glitch */
			d->pend = 0;
		} else if (++d->pend >= d->min_run) {
			mc3190_dig_push(d, d->run);
			d->dbp_level = dl;
			d->run = d->pend;
			d->pend = 0;
		}
	}
}

#endif
