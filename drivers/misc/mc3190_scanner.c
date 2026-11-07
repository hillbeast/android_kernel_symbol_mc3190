/*
 * mc3190_scanner.c -- Symbol SE-950 1D laser scanner on the MC3190 (PXA320)
 *
 * Capture method (reverse engineered from CamScanPDD1D.dll, WinCE):
 *
 *   The scan engine's digitised bar pattern (DBP) and start-of-scan (SOS)
 *   outputs are wired to the PXA320 Quick Capture Interface data pins
 *   (CIF_D0 = GPIO49 = DBP, CIF_D7 = GPIO56 = SOS).  The CI is run as a
 *   free-running 8-bit "logic analyser": MCLK (104 MHz / (2*(DIV+1)), default
 *   ~2.74 MHz) clocks one byte per tick into a DMA ring.  Sample timing is
 *   therefore exact and costs no interrupts per edge, which is the whole point:
 *   at ~17 m/s spot speed a 5 mil element lasts only ~7 us.
 *
 *   A kernel thread follows the DMA write pointer, runs the byte stream
 *   through the digitiser (mc3190_digitizer.h: SOS/DBP debounce + run-length
 *   encoding, one record per sweep) and queues records for /dev/scanner1d.
 *   Decoding is done in userspace (scand).
 *
 *   mode=1 selects the old GPIO-interrupt method (timestamps in us); it is
 *   kept only as a bring-up fallback.
 *
 * The CI register values/sequences marked [WinCE] are copied literally from
 * the vendor driver; registers 0x044/0x0b0/0x0c0/0x200/0x240/0x248 are
 * PXA3xx extensions whose names are inferred (see MC3190-scanner-notes.md).
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/ktime.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <asm/clkdev.h>	/* 2.6.35: asm/, not linux/ (moved in 2.6.37) */
#include <linux/io.h>
#include <linux/err.h>
#include <linux/mc3190.h>
#include "mc3190_scanner.h"
#include "mc3190_digitizer.h"

/* ---- module parameters ---- */
static int mode;			/* 0 = CI + DMA, 1 = GPIO interrupts */
module_param(mode, int, 0444);
MODULE_PARM_DESC(mode, "0 = Quick Capture Interface + DMA (default), 1 = GPIO IRQ fallback");
static int mclk_div;			/* 0: platform data, else WinCE default 18 */
module_param(mclk_div, int, 0444);
static int ring_kb = 256;
module_param(ring_kb, int, 0444);
MODULE_PARM_DESC(ring_kb, "DMA ring size in KiB (multiple of 8 bytes, max 1024)");
static int min_run = 3;
module_param(min_run, int, 0644);
MODULE_PARM_DESC(min_run, "DBP glitch filter, samples (CI) / us (GPIO)");
static int sos_min = 2;
module_param(sos_min, int, 0644);
static int min_runs = 8;
module_param(min_runs, int, 0644);
MODULE_PARM_DESC(min_runs, "drop sweeps with fewer runs than this");
static int poll_ms = 2;
module_param(poll_ms, int, 0644);
static unsigned int ci_clk_hz;
module_param(ci_clk_hz, uint, 0444);
MODULE_PARM_DESC(ci_clk_hz, "override CI clock rate if clk_get_rate() returns 0 (default 104 MHz)");
static int debug;
module_param(debug, int, 0644);

/* ---- Quick Capture Interface ---- */
#define CI_PHYS_BASE	0x50000000UL
#define CI_REGS_SIZE	0x280
#define CIBR0_PHYS	(CI_PHYS_BASE + 0x28)	/* FIFO0: DMA source */

#define CICR0		0x00
#define CICR1		0x04
#define CICR2		0x08
#define CICR3		0x0c
#define CICR4		0x10
#define CISR		0x14
#define CITOR		0x1c
#define CI_X044		0x044	/* [WinCE] cleared in init */
#define CI_X0B0		0x0b0	/* [WinCE] =8 at init, |=8,|=1 at start */
#define CI_X0C0		0x0c0	/* [WinCE] second status reg, cleared like CISR */
#define CI_DMA_STAT	0x200	/* [WinCE] bit0 = error, write 1 to ack */
#define CI_DMA_DDADR	0x240	/* [WinCE] descriptor address */
#define CI_DMA_DTADR	0x248	/* [WinCE] live target (write) address */

#define CICR0_INIT	0x01001e5fUL	/* SIM=1, masks */
#define CICR0_RUN	0x30000000UL	/* ENB | SL_CAP_EN */
#define CICR1_INIT	0x84ff8002UL	/* 8-bit data, PPL=2559 */
#define CICR4_BITS	0x00480000UL	/* MCLK_EN | PCP */
#define CISR_ACKMASK	0xc0008778UL
#define CI_X0C0_ACKMASK	0x11e3c40fUL
#define CISR_CQD	0x40		/* quick-disable complete */

#define DMA_DCMD_BASE	0x40400000UL	/* INCTRGADDR | irq bit; | len */
#define DMA_LEN_MASK	0x1ffff8UL
#define DESC_AREA	32		/* descriptor area in front of the ring */

#define DEF_MCLK_DIV	18		/* WinCE: 104 MHz*(1/(2*19)) = 2.74 MHz */
#define DEF_SCAN_RATE	104		/* sweeps/s; only used for LPF sizing */

#define QSIZE		8
#define GUARD		32		/* stay this far behind the DMA pointer */
#define BOUNCE		4096

struct sc_record {
	struct mc3190_scan_header h;
	u16 w[MC3190_SCAN_MAX_RUNS];
};

struct mc3190_scanner {
	struct mc3190_scanner_platform_data *pdata;
	struct device *dev;
	struct miscdevice miscdev;

	spinlock_t lock;		/* queue + GPIO-mode state, irq safe */
	struct mutex read_lock;
	wait_queue_head_t wq;
	int opened;
	int active;

	struct sc_record q[QSIZE];
	unsigned int q_head, q_tail, q_count;
	u32 seq, pending_flags;

	/* CI mode */
	void __iomem *regs;
	struct resource *res;
	struct clk *clk;
	void *ring_cpu;			/* descriptor area + ring */
	dma_addr_t ring_dma;
	u8 *data;
	dma_addr_t data_dma;
	u32 ring_bytes, rpos, sample_hz, mdiv, polls;
	struct task_struct *thread;
	u8 *bounce;
	struct mc3190_dig *dig;

	/* GPIO mode */
	int irq_sos, irq_data;
	ktime_t last_edge;
	int have_sos;
	struct mc3190_dig_sweep gcap;
};

/* ------------------------------------------------------------------ */
/* record queue                                                        */
/* ------------------------------------------------------------------ */
/* caller holds sc->lock; returns 1 if queued */
static int sc_queue_push_locked(struct mc3190_scanner *sc, struct mc3190_dig_sweep *sw,
				u32 tick_hz, u32 extra_flags)
{
	struct sc_record *rec;

	if (sc->q_count >= QSIZE) {
		sc->pending_flags |= MC3190_SCAN_F_QUEUE_OVERFLOW;
		return 0;
	}
	rec = &sc->q[sc->q_tail];
	rec->h.sequence = sc->seq++;
	rec->h.pulse_count = sw->count;
	rec->h.flags = sw->flags | extra_flags | sc->pending_flags;
	rec->h.initial_level = sw->initial_level;
	rec->h.tick_hz = tick_hz;
	sc->pending_flags = 0;
	memcpy(rec->w, sw->w, sw->count * sizeof(u16));
	sc->q_tail = (sc->q_tail + 1) % QSIZE;
	sc->q_count++;
	return 1;
}

static void sc_queue_push(struct mc3190_scanner *sc, struct mc3190_dig_sweep *sw,
			  u32 tick_hz, u32 extra_flags)
{
	unsigned long f;
	int queued;

	spin_lock_irqsave(&sc->lock, f);
	queued = sc_queue_push_locked(sc, sw, tick_hz, extra_flags);
	spin_unlock_irqrestore(&sc->lock, f);
	if (queued)
		wake_up_interruptible(&sc->wq);
}

static void sc_dig_emit(void *ctx, struct mc3190_dig_sweep *sw)
{
	struct mc3190_scanner *sc = ctx;
	sc_queue_push(sc, sw, sc->sample_hz, 0);
}

static void sc_queue_flush(struct mc3190_scanner *sc)
{
	unsigned long f;
	spin_lock_irqsave(&sc->lock, f);
	sc->q_head = sc->q_tail = sc->q_count = 0;
	sc->pending_flags = 0;
	spin_unlock_irqrestore(&sc->lock, f);
}

/* ------------------------------------------------------------------ */
/* laser / motor                                                       */
/* ------------------------------------------------------------------ */
static void sc_laser(struct mc3190_scanner *sc, int on)
{
	struct mc3190_scanner_platform_data *pd = sc->pdata;

	if (on) {
		gpio_set_value(pd->gpio_motor, !pd->motor_active_low);
		mc3190_cpld_regon(MC3190_CPLD_REG_LASER_EN_1, MC3190_CPLD_REG_LASER_1);
		mc3190_cpld_regon(MC3190_CPLD_REG_LASER_EN_2, MC3190_CPLD_REG_LASER_2);
	} else {
		mc3190_cpld_regoff(MC3190_CPLD_REG_LASER_EN_2, MC3190_CPLD_REG_LASER_2);
		mc3190_cpld_regoff(MC3190_CPLD_REG_LASER_EN_1, MC3190_CPLD_REG_LASER_1);
		gpio_set_value(pd->gpio_motor, pd->motor_active_low);
	}
}

/* ------------------------------------------------------------------ */
/* CI mode                                                             */
/* ------------------------------------------------------------------ */
static inline u32 ci_r(struct mc3190_scanner *sc, unsigned int off)
{
	return readl(sc->regs + off);
}
static inline void ci_w(struct mc3190_scanner *sc, unsigned int off, u32 v)
{
	writel(v, sc->regs + off);
}

/* [WinCE] FUN_10006f64: program the interface; capture not yet enabled */
static void ci_hw_init(struct mc3190_scanner *sc)
{
	u32 lpf, sweep, u;

	/* LPF sizing exactly like the vendor driver (lines of 2560 samples) */
	sweep = (sc->sample_hz / DEF_SCAN_RATE + 4) & ~3U;
	u = (sweep + 31) & ~31U;
	lpf = (u < 0x9ff0) ? 16 : u / 2559;
	if (lpf > 0x800)
		lpf = 0x800;

	ci_w(sc, CI_X0B0, 8);
	ci_w(sc, CICR0, CICR0_INIT);
	ci_w(sc, CICR1, CICR1_INIT);
	ci_w(sc, CICR2, 0);
	ci_w(sc, CICR3, (lpf - 1) & 0x7ff);
	ci_w(sc, CICR4, (sc->mdiv & 0xff) | CICR4_BITS);
	ci_w(sc, CITOR, 0);
	ci_w(sc, CI_X0B0, 0);
	ci_w(sc, CI_X044, 0);
}

static void ci_ack(struct mc3190_scanner *sc)
{
	ci_w(sc, CISR, ci_r(sc, CISR) & CISR_ACKMASK);
	ci_w(sc, CI_X0C0, ci_r(sc, CI_X0C0) & CI_X0C0_ACKMASK);
}

/* [WinCE] FUN_10006ad4: arm DMA and enable capture */
static void ci_start(struct mc3190_scanner *sc)
{
	ci_ack(sc);
	ci_w(sc, CI_DMA_STAT, 0);
	ci_w(sc, CI_DMA_DDADR, (u32)sc->ring_dma);	/* descriptor chain (loops) */
	ci_w(sc, CI_X0B0, ci_r(sc, CI_X0B0) | 8);
	ci_w(sc, CI_X0B0, ci_r(sc, CI_X0B0) | 1);
	memset(sc->data, 0, sc->ring_bytes);
	sc->rpos = 0;
	wmb();
	ci_w(sc, CICR0, ci_r(sc, CICR0) | CICR0_RUN);
}

/* [WinCE] FUN_10006b90: disable, wait for quick-disable done (max ~50 polls) */
static void ci_stop(struct mc3190_scanner *sc)
{
	int i;

	ci_w(sc, CICR0, ci_r(sc, CICR0) & ~CICR0_RUN);
	for (i = 0; i < 50; i++) {
		if (ci_r(sc, CISR) & CISR_CQD)
			return;
		udelay(20);
	}
	dev_warn(sc->dev, "CI quick-disable did not complete\n");
}

/* one descriptor, 16 bytes, loops to itself ([WinCE] FUN_10006850 layout) */
static void ci_build_ring(struct mc3190_scanner *sc)
{
	u32 *d = sc->ring_cpu;

	d[0] = (u32)sc->ring_dma;			/* DDADR: next = self */
	d[1] = CIBR0_PHYS;				/* DSADR: CI FIFO0 */
	d[2] = (u32)sc->data_dma;			/* DTADR */
	d[3] = (u32)((sc->ring_bytes & DMA_LEN_MASK) | DMA_DCMD_BASE);
	wmb();
}

static void ci_poll_once(struct mc3190_scanner *sc)
{
	u32 stat = ci_r(sc, CI_DMA_STAT) & 1;
	u32 w, avail;

	if (debug && !(sc->polls++ & 0xff))	/* bring-up trace: is DTADR moving? */
		dev_info(sc->dev, "CI: DTADR=%08x (ring %08x) CISR=%08x X0C0=%08x STAT=%08x CICR0=%08x\n",
			 ci_r(sc, CI_DMA_DTADR), (u32)sc->data_dma, ci_r(sc, CISR),
			 ci_r(sc, CI_X0C0), ci_r(sc, CI_DMA_STAT), ci_r(sc, CICR0));
	ci_ack(sc);
	if (stat) {	/* [WinCE] error or overrun: restart the capture */
		unsigned long f;
		dev_dbg(sc->dev, "DMA error flag, restarting capture\n");
		ci_w(sc, CI_DMA_STAT, ci_r(sc, CI_DMA_STAT) | 1);
		ci_stop(sc);
		ci_start(sc);
		mc3190_dig_reset(sc->dig);
		spin_lock_irqsave(&sc->lock, f);
		sc->pending_flags |= MC3190_SCAN_F_DMA_OVERRUN;
		spin_unlock_irqrestore(&sc->lock, f);
		return;
	}

	w = ci_r(sc, CI_DMA_DTADR) - (u32)sc->data_dma;
	if (w >= sc->ring_bytes) {
		if (w != sc->ring_bytes)
			return;		/* garbage, try next time */
		w = 0;
	}
	/* stay GUARD bytes behind so we never read bytes still in the FIFO */
	w = (w + sc->ring_bytes - GUARD) % sc->ring_bytes;
	avail = (w + sc->ring_bytes - sc->rpos) % sc->ring_bytes;

	while (avail) {
		u32 n = min(avail, sc->ring_bytes - sc->rpos);

		n = min(n, (u32)BOUNCE);

		memcpy(sc->bounce, sc->data + sc->rpos, n);	/* uncached -> cached */
		mc3190_dig_feed(sc->dig, sc->bounce, n);
		sc->rpos = (sc->rpos + n) % sc->ring_bytes;
		avail -= n;
	}
}

static int ci_thread(void *arg)
{
	struct mc3190_scanner *sc = arg;
	struct sched_param sp = { .sched_priority = 20 };
	long j = msecs_to_jiffies(poll_ms);

	sched_setscheduler(current, SCHED_FIFO, &sp);
	if (j < 1)
		j = 1;
	while (!kthread_should_stop()) {
		ci_poll_once(sc);
		set_current_state(TASK_INTERRUPTIBLE);
		if (!kthread_should_stop())
			schedule_timeout(j);
		__set_current_state(TASK_RUNNING);
	}
	return 0;
}

static int ci_session_start(struct mc3190_scanner *sc)
{
	int ret = clk_enable(sc->clk);

	if (ret)
		return ret;
	sc->dig->min_run = min_run > 0 ? min_run : 1;
	sc->dig->sos_min = sos_min > 0 ? sos_min : 1;
	sc->dig->min_runs_emit = min_runs;
	mc3190_dig_reset(sc->dig);
	ci_hw_init(sc);
	ci_build_ring(sc);
	ci_start(sc);
	sc->thread = kthread_run(ci_thread, sc, "scanner1d");
	if (IS_ERR(sc->thread)) {
		ret = PTR_ERR(sc->thread);
		sc->thread = NULL;
		ci_stop(sc);
		clk_disable(sc->clk);
		return ret;
	}
	return 0;
}

static void ci_session_stop(struct mc3190_scanner *sc)
{
	if (sc->thread) {
		kthread_stop(sc->thread);
		sc->thread = NULL;
	}
	ci_stop(sc);
	clk_disable(sc->clk);
}

static int ci_probe_hw(struct mc3190_scanner *sc, struct platform_device *pdev)
{
	struct mc3190_scanner_platform_data *pd = sc->pdata;
	unsigned long rate;
	u32 ring;
	int ret;

	if (ring_kb < 16 || ring_kb > 1024) {
		dev_err(&pdev->dev, "ring_kb out of range\n");
		return -EINVAL;
	}
	ring = ring_kb * 1024;
	sc->mdiv = mclk_div ? (u32)mclk_div : (pd->mclk_div ? pd->mclk_div : DEF_MCLK_DIV);

	sc->res = request_mem_region(CI_PHYS_BASE, CI_REGS_SIZE, "mc3190-scanner-ci");
	if (!sc->res) {
		dev_err(&pdev->dev, "CI register window busy (is the PXA camera driver enabled?)\n");
		return -EBUSY;
	}
	sc->regs = ioremap(CI_PHYS_BASE, CI_REGS_SIZE);
	if (!sc->regs) {
		ret = -ENOMEM;
		goto err_res;
	}

	sc->clk = clk_get_sys("pxa27x-camera.0", NULL);	/* check pxa3xx.c clkregs */
	if (IS_ERR(sc->clk))
		sc->clk = clk_get(NULL, "CAMCLK");
	if (IS_ERR(sc->clk)) {
		dev_err(&pdev->dev, "no CI clock found, see pxa3xx_clkregs[]\n");
		ret = PTR_ERR(sc->clk);
		goto err_map;
	}
	rate = ci_clk_hz ? ci_clk_hz : clk_get_rate(sc->clk);
	if (!rate)
		rate = 104000000;
	sc->sample_hz = rate / (2 * (sc->mdiv + 1));

	if (!pdev->dev.coherent_dma_mask)
		pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	sc->ring_bytes = ring;
	sc->ring_cpu = dma_alloc_coherent(&pdev->dev, ring + DESC_AREA, &sc->ring_dma, GFP_KERNEL);
	if (!sc->ring_cpu) {
		ret = -ENOMEM;
		goto err_clk;
	}
	sc->data = (u8 *)sc->ring_cpu + DESC_AREA;
	sc->data_dma = sc->ring_dma + DESC_AREA;
	sc->bounce = kmalloc(BOUNCE, GFP_KERNEL);
	sc->dig = kzalloc(sizeof(*sc->dig), GFP_KERNEL);
	if (!sc->bounce || !sc->dig) {
		ret = -ENOMEM;
		goto err_dma;
	}
	sc->dig->sos_mask = pd->sos_mask ? pd->sos_mask : 0x80;	/* [WinCE] SosMask  */
	sc->dig->dbp_mask = pd->dbp_mask ? pd->dbp_mask : 0x01;	/* [WinCE] DbpMask  */
	sc->dig->emit = sc_dig_emit;
	sc->dig->ctx = sc;

	dev_info(&pdev->dev, "CI capture: MCLK div %u -> %u Hz sample clock, ring %u KiB\n",
		 sc->mdiv, sc->sample_hz, ring / 1024);
	return 0;

err_dma:
	kfree(sc->bounce);
	kfree(sc->dig);
	dma_free_coherent(&pdev->dev, ring + DESC_AREA, sc->ring_cpu, sc->ring_dma);
err_clk:
	clk_put(sc->clk);
err_map:
	iounmap(sc->regs);
err_res:
	release_mem_region(CI_PHYS_BASE, CI_REGS_SIZE);
	return ret;
}

static void ci_remove_hw(struct mc3190_scanner *sc, struct platform_device *pdev)
{
	kfree(sc->bounce);
	kfree(sc->dig);
	dma_free_coherent(&pdev->dev, sc->ring_bytes + DESC_AREA, sc->ring_cpu, sc->ring_dma);
	clk_put(sc->clk);
	iounmap(sc->regs);
	release_mem_region(CI_PHYS_BASE, CI_REGS_SIZE);
}

/* ------------------------------------------------------------------ */
/* GPIO fallback (mode=1): per-edge timestamps, microsecond ticks      */
/* ------------------------------------------------------------------ */
static irqreturn_t sc_gpio_sos_isr(int irq, void *id)
{
	struct mc3190_scanner *sc = id;
	unsigned long f;
	int lvl = gpio_get_value(sc->pdata->gpio_data);
	int queued = 0;

	spin_lock_irqsave(&sc->lock, f);
	if (sc->active) {
		if (sc->have_sos && sc->gcap.count >= (u32)min_runs)
			queued = sc_queue_push_locked(sc, &sc->gcap, 1000000,
						      MC3190_SCAN_F_GPIO_MODE);
		sc->gcap.count = 0;
		sc->gcap.flags = 0;
		sc->gcap.initial_level = lvl > 0;
		if (lvl < 0)
			sc->gcap.flags |= MC3190_SCAN_F_INITIAL_LEVEL_BAD;
		sc->last_edge = ktime_get();
		sc->have_sos = 1;
	}
	spin_unlock_irqrestore(&sc->lock, f);
	if (queued)
		wake_up_interruptible(&sc->wq);
	return IRQ_HANDLED;
}

static irqreturn_t sc_gpio_data_isr(int irq, void *id)
{
	struct mc3190_scanner *sc = id;
	unsigned long f;

	spin_lock_irqsave(&sc->lock, f);
	if (sc->active && sc->have_sos) {
		ktime_t now = ktime_get();
		s64 d = ktime_us_delta(now, sc->last_edge);

		sc->last_edge = now;
		if (d <= 0)
			d = 1;
		if (d > 0xffff) {
			d = 0xffff;
			sc->gcap.flags |= MC3190_SCAN_F_RUN_CLAMPED;
		}
		if (sc->gcap.count < MC3190_SCAN_MAX_RUNS)
			sc->gcap.w[sc->gcap.count++] = (u16)d;
		else
			sc->gcap.flags |= MC3190_SCAN_F_TRUNCATED;
	}
	spin_unlock_irqrestore(&sc->lock, f);
	return IRQ_HANDLED;
}

/* ------------------------------------------------------------------ */
/* file operations                                                     */
/* ------------------------------------------------------------------ */
static int sc_open(struct inode *inode, struct file *file)
{
	struct miscdevice *m = file->private_data;
	struct mc3190_scanner *sc = container_of(m, struct mc3190_scanner, miscdev);
	unsigned long f;
	int ret = 0;

	spin_lock_irqsave(&sc->lock, f);
	if (sc->opened) {
		spin_unlock_irqrestore(&sc->lock, f);
		return -EBUSY;
	}
	sc->opened = 1;
	spin_unlock_irqrestore(&sc->lock, f);

	file->private_data = sc;
	sc_queue_flush(sc);
	sc->seq = 0;

	sc_laser(sc, 1);
	if (mode == 0) {
		ret = ci_session_start(sc);
	} else {
		spin_lock_irqsave(&sc->lock, f);
		sc->have_sos = 0;
		sc->active = 1;
		spin_unlock_irqrestore(&sc->lock, f);
	}
	if (ret) {
		sc_laser(sc, 0);
		sc->opened = 0;
	}
	return ret;
}

static int sc_release(struct inode *inode, struct file *file)
{
	struct mc3190_scanner *sc = file->private_data;
	unsigned long f;

	if (mode == 0) {
		ci_session_stop(sc);
	} else {
		spin_lock_irqsave(&sc->lock, f);
		sc->active = 0;
		spin_unlock_irqrestore(&sc->lock, f);
		synchronize_irq(sc->irq_sos);
		synchronize_irq(sc->irq_data);
	}
	sc_laser(sc, 0);
	sc_queue_flush(sc);
	sc->opened = 0;
	return 0;
}

static ssize_t sc_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	struct mc3190_scanner *sc = file->private_data;
	struct sc_record *rec;
	unsigned long f;
	size_t size;
	int ret;

	ret = mutex_lock_interruptible(&sc->read_lock);
	if (ret)
		return ret;

	while (!sc->q_count) {
		if (file->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			goto out;
		}
		ret = wait_event_interruptible(sc->wq, sc->q_count > 0);
		if (ret)
			goto out;
	}

	rec = &sc->q[sc->q_head];	/* only readers pop, and we hold read_lock */
	size = sizeof(rec->h) + rec->h.pulse_count * sizeof(u16);
	if (count < size) {
		ret = -EINVAL;
		goto out;
	}
	if (copy_to_user(buf, &rec->h, sizeof(rec->h)) ||
	    copy_to_user(buf + sizeof(rec->h), rec->w, rec->h.pulse_count * sizeof(u16))) {
		ret = -EFAULT;
		goto out;
	}
	spin_lock_irqsave(&sc->lock, f);
	sc->q_head = (sc->q_head + 1) % QSIZE;
	sc->q_count--;
	spin_unlock_irqrestore(&sc->lock, f);
	ret = size;
out:
	mutex_unlock(&sc->read_lock);
	return ret;
}

static unsigned int sc_poll(struct file *file, poll_table *wait)
{
	struct mc3190_scanner *sc = file->private_data;

	poll_wait(file, &sc->wq, wait);
	return sc->q_count ? (POLLIN | POLLRDNORM) : 0;
}

static const struct file_operations sc_fops = {
	.owner	 = THIS_MODULE,
	.open	 = sc_open,
	.release = sc_release,
	.read	 = sc_read,
	.poll	 = sc_poll,
};

/* ------------------------------------------------------------------ */
/* platform driver                                                     */
/* ------------------------------------------------------------------ */
static int __devinit mc3190_scanner_probe(struct platform_device *pdev)
{
	struct mc3190_scanner_platform_data *pd = pdev->dev.platform_data;
	struct mc3190_scanner *sc;
	int ret;

	if (!pd)
		return -EINVAL;
	sc = kzalloc(sizeof(*sc), GFP_KERNEL);
	if (!sc)
		return -ENOMEM;
	sc->pdata = pd;
	sc->dev = &pdev->dev;
	spin_lock_init(&sc->lock);
	mutex_init(&sc->read_lock);
	init_waitqueue_head(&sc->wq);

	ret = gpio_request_one(pd->gpio_motor,
			       pd->motor_active_low ? GPIOF_OUT_INIT_HIGH : GPIOF_OUT_INIT_LOW,
			       "scanner_motor");
	if (ret)
		goto err_free;

	if (mode == 0) {
		ret = ci_probe_hw(sc, pdev);
		if (ret)
			goto err_motor;
	} else {
		sc->sample_hz = 1000000;
		ret = gpio_request_one(pd->gpio_sos, GPIOF_IN, "scanner_sos");
		if (ret)
			goto err_motor;
		ret = gpio_request_one(pd->gpio_data, GPIOF_IN, "scanner_data");
		if (ret)
			goto err_gsos;
		sc->irq_sos = gpio_to_irq(pd->gpio_sos);
		sc->irq_data = gpio_to_irq(pd->gpio_data);
		/* SOS toggles once per sweep: take BOTH edges (old code: falling only) */
		ret = request_irq(sc->irq_sos, sc_gpio_sos_isr,
				  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "scanner_sos", sc);
		if (ret)
			goto err_gdata;
		ret = request_irq(sc->irq_data, sc_gpio_data_isr,
				  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "scanner_data", sc);
		if (ret)
			goto err_gsos_irq;
	}

	sc->miscdev.minor = MISC_DYNAMIC_MINOR;
	sc->miscdev.name = "scanner1d";
	sc->miscdev.fops = &sc_fops;
	ret = misc_register(&sc->miscdev);
	if (ret)
		goto err_hw;

	platform_set_drvdata(pdev, sc);
	dev_info(&pdev->dev, "MC3190 1D scanner ready (%s mode)\n", mode ? "GPIO" : "CI/DMA");
	return 0;

err_hw:
	if (mode == 0) {
		ci_remove_hw(sc, pdev);
		goto err_motor;
	}
	free_irq(sc->irq_data, sc);
err_gsos_irq:
	free_irq(sc->irq_sos, sc);
err_gdata:
	gpio_free(pd->gpio_data);
err_gsos:
	gpio_free(pd->gpio_sos);
err_motor:
	gpio_free(pd->gpio_motor);
err_free:
	kfree(sc);
	return ret;
}

static int __devexit mc3190_scanner_remove(struct platform_device *pdev)
{
	struct mc3190_scanner *sc = platform_get_drvdata(pdev);
	struct mc3190_scanner_platform_data *pd = sc->pdata;

	misc_deregister(&sc->miscdev);
	if (mode == 0) {
		ci_remove_hw(sc, pdev);
	} else {
		free_irq(sc->irq_data, sc);
		free_irq(sc->irq_sos, sc);
		gpio_free(pd->gpio_data);
		gpio_free(pd->gpio_sos);
	}
	gpio_free(pd->gpio_motor);
	kfree(sc);
	return 0;
}

/* refuse to sleep while a scan session is running (laser + DMA active) */
static int mc3190_scanner_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct mc3190_scanner *sc = platform_get_drvdata(pdev);

	return sc->opened ? -EBUSY : 0;
}

static struct platform_driver mc3190_scanner_driver = {
	.probe	 = mc3190_scanner_probe,
	.remove	 = __devexit_p(mc3190_scanner_remove),
	.suspend = mc3190_scanner_suspend,
	.driver	 = {
		.name  = "mc3190-scanner",
		.owner = THIS_MODULE,
	},
};

static int __init mc3190_scanner_init(void)
{
	return platform_driver_register(&mc3190_scanner_driver);
}

static void __exit mc3190_scanner_exit(void)
{
	platform_driver_unregister(&mc3190_scanner_driver);
}

module_init(mc3190_scanner_init);
module_exit(mc3190_scanner_exit);
MODULE_AUTHOR("Mark Kennard");
MODULE_DESCRIPTION("Symbol MC3190 1D Laser Scanner Driver (CI/DMA capture)");
MODULE_LICENSE("GPL v2");