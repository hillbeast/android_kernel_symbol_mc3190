/*
 *  pxa3xx-gcu.c - Linux kernel module for PXA3xx graphics controllers
 *
 *  This driver needs a DirectFB counterpart in user space, communication
 *  is handled via mmap()ed memory areas and an ioctl.
 *
 *  Copyright (c) 2009 Daniel Mack <daniel@caiaq.de>
 *  Copyright (c) 2009 Janine Kropp <nin@directfb.org>
 *  Copyright (c) 2009 Denis Oliver Kropp <dok@directfb.org>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/*
 * WARNING: This controller is attached to System Bus 2 of the PXA which
 * needs its arbiter to be enabled explicitly (CKENB & 1<<9).
 * There is currently no way to do this from Linux, so you need to teach
 * your bootloader for now.
 */

#include <linux/module.h>
#include <linux/version.h>

#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/ioctl.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/mutex.h>

#include <asm/cacheflush.h>
#include <asm/outercache.h>

#include "pxa3xx-gcu.h"

#define DRV_NAME	"pxa3xx-gcu"
#define MISCDEV_MINOR	197

#define REG_GCCR	0x00
#define GCCR_SYNC_CLR	(1 << 9)
#define GCCR_BP_RST	(1 << 8)
#define GCCR_ABORT	(1 << 6)
#define GCCR_STOP	(1 << 4)

#define REG_GCISCR	0x04
#define REG_GCIECR	0x08
#define REG_GCRBBR	0x20
#define REG_GCRBLR	0x24
#define REG_GCRBHR	0x28
#define REG_GCRBTR	0x2C
#define REG_GCRBEXHR	0x30

#define IE_EOB		(1 << 0)
#define IE_EEOB		(1 << 5)
#define IE_ALL		0xff

#define SHARED_SIZE	PAGE_ALIGN(sizeof(struct pxa3xx_gcu_shared))

/* #define PXA3XX_GCU_DEBUG */
/* #define PXA3XX_GCU_DEBUG_TIMER */

#ifdef PXA3XX_GCU_DEBUG
#define QDUMP(msg)					\
	do {						\
		QPRINT(priv, KERN_DEBUG, msg);		\
	} while (0)
#else
#define QDUMP(msg)	do {} while (0)
#endif

#define QERROR(msg)					\
	do {						\
		QPRINT(priv, KERN_ERR, msg);		\
	} while (0)

struct pxa3xx_gcu_batch {
	struct pxa3xx_gcu_batch *next;
	u32			*ptr;
	dma_addr_t		 phys;
	unsigned long		 length;
};

/* A physically contiguous buffer that can be mapped into user space. */
struct pxa3xx_gcu_buf {
	void			*kaddr;			/* NULL: slot is free */
	phys_addr_t		 phys;
	size_t			 size;
	struct file		*owner;			/* the open file that allocated it */
	struct mm_struct	*mm;		/* the process that mapped it */
	unsigned long		 uaddr;		/* start of that user mapping */
	int			 map_count;
	u32			 flags;				/* PXA3XX_GCU_BUF_* */
};

struct pxa3xx_gcu_priv {
	struct device		 *dev;
	struct mutex		  buf_lock;	/* protects bufs[] */
	struct pxa3xx_gcu_buf	  bufs[PXA3XX_GCU_MAX_BUFS];
	void __iomem		 *mmio_base;
	struct clk		 *clk;
	struct pxa3xx_gcu_shared *shared;
	dma_addr_t		  shared_phys;
	struct resource		 *resource_mem;
	struct miscdevice	  misc_dev;
	struct file_operations	  misc_fops;
	wait_queue_head_t	  wait_idle;
	wait_queue_head_t	  wait_free;
	spinlock_t		  spinlock;
	struct timeval 		  base_time;

	struct pxa3xx_gcu_batch *free;

	struct pxa3xx_gcu_batch *ready;
	struct pxa3xx_gcu_batch *ready_last;
	struct pxa3xx_gcu_batch *running;
};

static inline unsigned long
gc_readl(struct pxa3xx_gcu_priv *priv, unsigned int off)
{
	return __raw_readl(priv->mmio_base + off);
}

static inline void
gc_writel(struct pxa3xx_gcu_priv *priv, unsigned int off, unsigned long val)
{
	__raw_writel(val, priv->mmio_base + off);
}

#define QPRINT(priv, level, msg)					\
	do {								\
		struct timeval tv;					\
		struct pxa3xx_gcu_shared *shared = priv->shared;	\
		u32 base = gc_readl(priv, REG_GCRBBR);			\
									\
		do_gettimeofday(&tv);					\
									\
		printk(level "%ld.%03ld.%03ld - %-17s: %-21s (%s, "	\
			"STATUS "					\
			"0x%02lx, B 0x%08lx [%ld], E %5ld, H %5ld, "	\
			"T %5ld)\n",					\
			tv.tv_sec - priv->base_time.tv_sec,		\
			tv.tv_usec / 1000, tv.tv_usec % 1000,		\
			__func__, msg,					\
			shared->hw_running ? "running" : "   idle",	\
			gc_readl(priv, REG_GCISCR),			\
			gc_readl(priv, REG_GCRBBR),			\
			gc_readl(priv, REG_GCRBLR),			\
			(gc_readl(priv, REG_GCRBEXHR) - base) / 4,	\
			(gc_readl(priv, REG_GCRBHR) - base) / 4,	\
			(gc_readl(priv, REG_GCRBTR) - base) / 4);	\
	} while (0)

static void
pxa3xx_gcu_reset(struct pxa3xx_gcu_priv *priv)
{
	QDUMP("RESET");

	/* disable interrupts */
	gc_writel(priv, REG_GCIECR, 0);

	/* reset hardware */
	gc_writel(priv, REG_GCCR, GCCR_ABORT);
	gc_writel(priv, REG_GCCR, 0);

	memset(priv->shared, 0, SHARED_SIZE);
	priv->shared->buffer_phys = priv->shared_phys;
	priv->shared->magic = PXA3XX_GCU_SHARED_MAGIC;

	do_gettimeofday(&priv->base_time);

	/* set up the ring buffer pointers */
	gc_writel(priv, REG_GCRBLR, 0);
	gc_writel(priv, REG_GCRBBR, priv->shared_phys);
	gc_writel(priv, REG_GCRBTR, priv->shared_phys);

	/* enable all IRQs except EOB */
	gc_writel(priv, REG_GCIECR, IE_ALL & ~IE_EOB);
}

static void
dump_whole_state(struct pxa3xx_gcu_priv *priv)
{
	struct pxa3xx_gcu_shared *sh = priv->shared;
	u32 base = gc_readl(priv, REG_GCRBBR);

	QDUMP("DUMP");

	printk(KERN_DEBUG "== PXA3XX-GCU DUMP ==\n"
		"%s, STATUS 0x%02lx, B 0x%08lx [%ld], E %5ld, H %5ld, T %5ld\n",
		sh->hw_running ? "running" : "idle   ",
		gc_readl(priv, REG_GCISCR),
		gc_readl(priv, REG_GCRBBR),
		gc_readl(priv, REG_GCRBLR),
		(gc_readl(priv, REG_GCRBEXHR) - base) / 4,
		(gc_readl(priv, REG_GCRBHR) - base) / 4,
		(gc_readl(priv, REG_GCRBTR) - base) / 4);
}

static void
flush_running(struct pxa3xx_gcu_priv *priv)
{
	struct pxa3xx_gcu_batch *running = priv->running;
	struct pxa3xx_gcu_batch *next;

	while (running) {
		next = running->next;
		running->next = priv->free;
		priv->free = running;
		running = next;
	}

	priv->running = NULL;
}

static void
run_ready(struct pxa3xx_gcu_priv *priv)
{
	unsigned int num = 0;
	struct pxa3xx_gcu_shared *shared = priv->shared;
	struct pxa3xx_gcu_batch	*ready = priv->ready;

	QDUMP("Start");

	BUG_ON(!ready);

	shared->buffer[num++] = 0x05000000;

	while (ready) {
		shared->buffer[num++] = 0x00000001;
		shared->buffer[num++] = ready->phys;
		ready = ready->next;
	}

	shared->buffer[num++] = 0x05000000;
	priv->running = priv->ready;
	priv->ready = priv->ready_last = NULL;
	gc_writel(priv, REG_GCRBLR, 0);
	shared->hw_running = 1;

	/* ring base address */
	gc_writel(priv, REG_GCRBBR, shared->buffer_phys);

	/* ring tail address */
	gc_writel(priv, REG_GCRBTR, shared->buffer_phys + num * 4);

	/* ring length */
	gc_writel(priv, REG_GCRBLR, ((num + 63) & ~63) * 4);
}

static irqreturn_t
pxa3xx_gcu_handle_irq(int irq, void *ctx)
{
	struct pxa3xx_gcu_priv *priv = ctx;
	struct pxa3xx_gcu_shared *shared = priv->shared;
	u32 status = gc_readl(priv, REG_GCISCR) & IE_ALL;

	QDUMP("-Interrupt");

	if (!status)
		return IRQ_NONE;

	spin_lock(&priv->spinlock);
	shared->num_interrupts++;

	if (status & IE_EEOB) {
		QDUMP(" [EEOB]");

		flush_running(priv);
		wake_up_all(&priv->wait_free);

		if (priv->ready) {
			run_ready(priv);
		} else {
			/* There is no more data prepared by the userspace.
			 * Set hw_running = 0 and wait for the next userspace
			 * kick-off */
			shared->num_idle++;
			shared->hw_running = 0;

			QDUMP(" '-> Idle.");

			/* set ring buffer length to zero */
			gc_writel(priv, REG_GCRBLR, 0);

			wake_up_all(&priv->wait_idle);
		}

		shared->num_done++;
	} else {
		QERROR(" [???]");
		dump_whole_state(priv);
	}

	/* Clear the interrupt */
	gc_writel(priv, REG_GCISCR, status);
	spin_unlock(&priv->spinlock);

	return IRQ_HANDLED;
}

static int
pxa3xx_gcu_wait_idle(struct pxa3xx_gcu_priv *priv)
{
	int ret = 0;

	QDUMP("Waiting for idle...");

	/* Does not need to be atomic. There's a lock in user space,
	 * but anyhow, this is just for statistics. */
	priv->shared->num_wait_idle++;

	while (priv->shared->hw_running) {
		int num = priv->shared->num_interrupts;
		u32 rbexhr = gc_readl(priv, REG_GCRBEXHR);

		ret = wait_event_interruptible_timeout(priv->wait_idle,
					!priv->shared->hw_running, HZ*4);

		if (ret < 0)
			break;

		if (ret > 0)
			continue;

		if (gc_readl(priv, REG_GCRBEXHR) == rbexhr &&
		    priv->shared->num_interrupts == num) {
			QERROR("TIMEOUT");
			ret = -ETIMEDOUT;
			break;
		}
	}

	QDUMP("done");

	return ret;
}

static int
pxa3xx_gcu_wait_free(struct pxa3xx_gcu_priv *priv)
{
	int ret = 0;

	QDUMP("Waiting for free...");

	/* Does not need to be atomic. There's a lock in user space,
	 * but anyhow, this is just for statistics. */
	priv->shared->num_wait_free++;

	while (!priv->free) {
		u32 rbexhr = gc_readl(priv, REG_GCRBEXHR);

		ret = wait_event_interruptible_timeout(priv->wait_free,
						       priv->free, HZ*4);

		if (ret < 0)
			break;

		if (ret > 0)
			continue;

		if (gc_readl(priv, REG_GCRBEXHR) == rbexhr) {
			QERROR("TIMEOUT");
			ret = -ETIMEDOUT;
			break;
		}
	}

	QDUMP("done");

	return ret;
}

static int pxa3xx_gcu_quiesce(struct pxa3xx_gcu_priv *priv)
{
	if (!priv->shared->hw_running)
		return 0;

	if (!wait_event_timeout(priv->wait_idle,
				!priv->shared->hw_running, HZ * 4))
		return -ETIMEDOUT;

	return 0;
}

static struct pxa3xx_gcu_buf *
buf_lookup(struct pxa3xx_gcu_priv *priv, struct file *filp, u32 id)
{
	struct pxa3xx_gcu_buf *buf;

	if (id >= PXA3XX_GCU_MAX_BUFS)
		return NULL;

	buf = &priv->bufs[id];
	if (!buf->kaddr || buf->owner != filp)
		return NULL;

	return buf;
}

/* priv->buf_lock must be held. The caller has made sure the GCU is idle. */
static void
buf_release(struct pxa3xx_gcu_buf *buf)
{
	free_pages_exact(buf->kaddr, buf->size);
	memset(buf, 0, sizeof(*buf));
}

static int
pxa3xx_gcu_buf_alloc(struct pxa3xx_gcu_priv *priv, struct file *filp,
		     struct pxa3xx_gcu_buf_req __user *ureq)
{
	struct pxa3xx_gcu_buf_req req;
	struct pxa3xx_gcu_buf *buf = NULL;
	dma_addr_t dma;
	size_t size;
	int i, ret = 0;

	if (copy_from_user(&req, ureq, sizeof(req)))
		return -EFAULT;

	if (!req.size || req.size > PXA3XX_GCU_BUF_MAX_SIZE)
		return -EINVAL;

	if (req.flags & ~PXA3XX_GCU_BUF_WRITETHROUGH)
		return -EINVAL;

	size = PAGE_ALIGN(req.size);

	mutex_lock(&priv->buf_lock);

	for (i = 0; i < PXA3XX_GCU_MAX_BUFS; i++) {
		if (!priv->bufs[i].kaddr) {
			buf = &priv->bufs[i];
			break;
		}
	}
	if (!buf) {
		ret = -ENOSPC;
		goto out;
	}

	/* zeroed, so nothing stale is handed to user space */
	buf->kaddr = alloc_pages_exact(size, GFP_KERNEL | __GFP_ZERO);
	if (!buf->kaddr) {
		ret = -ENOMEM;
		goto out;
	}

	/* Write the zeroes out of the kernel's cached alias, so the later
	 * cached user mapping (a different virtual address; the caches are
	 * virtually indexed) can never see stale kernel-side lines. */
	dma = dma_map_single(priv->dev, buf->kaddr, size, DMA_TO_DEVICE);
	if (dma_mapping_error(priv->dev, dma)) {
		free_pages_exact(buf->kaddr, size);
		buf->kaddr = NULL;
		ret = -EIO;
		goto out;
	}
	dma_unmap_single(priv->dev, dma, size, DMA_TO_DEVICE);

	buf->phys = virt_to_phys(buf->kaddr);
	buf->size = size;
	buf->owner = filp;
	buf->mm = NULL;
	buf->uaddr = 0;
	buf->map_count = 0;
	buf->flags = req.flags;

	req.id = i;
	req.phys = buf->phys;
	req.mmap_offset = PXA3XX_GCU_BUF_MMAP_BASE +
			  i * PXA3XX_GCU_BUF_MMAP_STRIDE;
	req.alloc_size = size;

	if (copy_to_user(ureq, &req, sizeof(req))) {
		buf_release(buf);
		ret = -EFAULT;
	}
out:
	mutex_unlock(&priv->buf_lock);
	return ret;
}

static int
pxa3xx_gcu_buf_free(struct pxa3xx_gcu_priv *priv, struct file *filp,
		    u32 __user *uid)
{
	struct pxa3xx_gcu_buf *buf;
	u32 id;
	int ret = 0;

	if (get_user(id, uid))
		return -EFAULT;

	mutex_lock(&priv->buf_lock);

	buf = buf_lookup(priv, filp, id);
	if (!buf)
		ret = -EINVAL;
	else if (buf->map_count)
		ret = -EBUSY;			/* munmap() it first */
	else if (pxa3xx_gcu_quiesce(priv))
		ret = -ETIMEDOUT;		/* GCU might still read it */
	else
		buf_release(buf);

	mutex_unlock(&priv->buf_lock);
	return ret;
}

/*
 * Make the CPU caches and RAM agree for a range of a buffer: write back
 * dirty lines (before the GCU reads what the CPU wrote) and drop cached
 * lines (before the CPU reads what the GCU wrote). The L1 caches are
 * virtually indexed, so the maintenance has to be done through the *user*
 * mapping; the kernel's own alias of the buffer is never touched.
 */
static int
pxa3xx_gcu_buf_sync(struct pxa3xx_gcu_priv *priv, struct file *filp,
		    struct pxa3xx_gcu_sync_req __user *ureq)
{
	struct pxa3xx_gcu_sync_req req;
	struct pxa3xx_gcu_buf *buf;
	struct vm_area_struct *vma;
	unsigned long start, end;
	int ret = 0;

	if (copy_from_user(&req, ureq, sizeof(req)))
		return -EFAULT;

	mutex_lock(&priv->buf_lock);

	buf = buf_lookup(priv, filp, req.id);
	if (!buf || !buf->map_count || buf->mm != current->mm) {
		ret = -EINVAL;
		goto out;
	}
	if (req.offset > buf->size || req.len > buf->size - req.offset) {
		ret = -EINVAL;
		goto out;
	}
	if (!req.len)
		goto out;

	if (req.flags == PXA3XX_GCU_SYNC_DRAIN) {
		if (!(buf->flags & PXA3XX_GCU_BUF_WRITETHROUGH)) {
			ret = -EINVAL;		/* needs a write-through buffer */
			goto out;
		}
		mb();				/* drain the CPU write buffer */
		outer_clean_range(buf->phys + req.offset,
				  buf->phys + req.offset + req.len);
		goto out;
	}

	start = buf->uaddr + req.offset;
	end = start + req.len;

	down_read(&current->mm->mmap_sem);
	vma = find_vma(current->mm, start);
	if (vma && vma->vm_start <= start && end <= vma->vm_end &&
	    vma->vm_private_data == buf) {
		if (req.flags == PXA3XX_GCU_SYNC_TO_DEVICE)
			/* write dirty lines back, keep them cached */
			flush_cache_user_range(start, end);
		else
			/* write back and drop the lines */
			flush_cache_range(vma, start, end);
	} else {
		ret = -EFAULT;
	}
	up_read(&current->mm->mmap_sem);

	if (!ret) {
		if (req.flags == PXA3XX_GCU_SYNC_TO_DEVICE)
			outer_clean_range(buf->phys + req.offset,
					  buf->phys + req.offset + req.len);
		else
			outer_flush_range(buf->phys + req.offset,
					  buf->phys + req.offset + req.len);
	}
out:
	mutex_unlock(&priv->buf_lock);
	return ret;
}

static void
buf_vm_close(struct vm_area_struct *vma)
{
	struct pxa3xx_gcu_buf *buf = vma->vm_private_data;

	/* buf_lock is not needed to clear these; the owner file cannot be
	 * released (and the buffer freed) before this mapping is gone. */
	buf->map_count = 0;
	buf->uaddr = 0;
	buf->mm = NULL;
}

static const struct vm_operations_struct buf_vm_ops = {
	.close = buf_vm_close,
};

static int
pxa3xx_gcu_buf_mmap(struct pxa3xx_gcu_priv *priv, struct file *filp,
		    struct vm_area_struct *vma)
{
	unsigned long rel = (vma->vm_pgoff << PAGE_SHIFT) -
			    PXA3XX_GCU_BUF_MMAP_BASE;
	unsigned long size = vma->vm_end - vma->vm_start;
	struct pxa3xx_gcu_buf *buf;
	int ret;

	if (rel % PXA3XX_GCU_BUF_MMAP_STRIDE)
		return -EINVAL;

	mutex_lock(&priv->buf_lock);

	buf = buf_lookup(priv, filp, rel / PXA3XX_GCU_BUF_MMAP_STRIDE);
	if (!buf) {
		ret = -EINVAL;
	} else if (size > buf->size) {
		ret = -EINVAL;
	} else if (buf->map_count) {
		ret = -EBUSY;			/* one mapping at a time */
	} else {
		/* normal cached (write-back) mapping, unless write-through
		 * was requested at allocation */
		if (buf->flags & PXA3XX_GCU_BUF_WRITETHROUGH)
			vma->vm_page_prot = __pgprot_modify(vma->vm_page_prot,
					L_PTE_MT_MASK, L_PTE_MT_WRITETHROUGH);
		ret = remap_pfn_range(vma, vma->vm_start,
				      buf->phys >> PAGE_SHIFT, size,
				      vma->vm_page_prot);
		if (!ret) {
			vma->vm_flags |= VM_DONTCOPY | VM_DONTEXPAND;
			vma->vm_private_data = buf;
			vma->vm_ops = &buf_vm_ops;
			buf->mm = vma->vm_mm;
			buf->uaddr = vma->vm_start;
			buf->map_count = 1;
		}
	}

	mutex_unlock(&priv->buf_lock);
	return ret;
}

/* The device node was closed: give back everything this file allocated. */
static int
pxa3xx_gcu_misc_release(struct inode *inode, struct file *filp)
{
	struct pxa3xx_gcu_priv *priv =
		container_of(filp->f_op, struct pxa3xx_gcu_priv, misc_fops);
	int i, owns = 0, not_idle;

	mutex_lock(&priv->buf_lock);

	for (i = 0; i < PXA3XX_GCU_MAX_BUFS; i++)
		if (priv->bufs[i].kaddr && priv->bufs[i].owner == filp)
			owns = 1;

	if (!owns) {
		/* the common case: do not stall close() waiting for the GCU */
		mutex_unlock(&priv->buf_lock);
		return 0;
	}

	not_idle = pxa3xx_gcu_quiesce(priv);

	for (i = 0; i < PXA3XX_GCU_MAX_BUFS; i++) {
		struct pxa3xx_gcu_buf *buf = &priv->bufs[i];

		if (!buf->kaddr || buf->owner != filp)
			continue;

		if (not_idle || buf->map_count) {
			/* GCU hung or mapping still alive: leaking the memory
			 * is safer than letting it be reused under the GCU */
			dev_warn(priv->dev, "leaking contiguous buffer %d\n", i);
			buf->owner = NULL;
			continue;
		}
		buf_release(buf);
	}

	mutex_unlock(&priv->buf_lock);
	return 0;
}

/* Misc device layer */

static ssize_t
pxa3xx_gcu_misc_write(struct file *filp, const char *buff,
		      size_t count, loff_t *offp)
{
	int ret;
	unsigned long flags;
	struct pxa3xx_gcu_batch	*buffer;
	struct pxa3xx_gcu_priv *priv =
		container_of(filp->f_op, struct pxa3xx_gcu_priv, misc_fops);

	int words = count / 4;

	/* Does not need to be atomic. There's a lock in user space,
	 * but anyhow, this is just for statistics. */
	priv->shared->num_writes++;

	priv->shared->num_words += words;

	/* Last word reserved for batch buffer end command */
	if (words >= PXA3XX_GCU_BATCH_WORDS)
		return -E2BIG;

	/* Wait for a free buffer */
	if (!priv->free) {
		ret = pxa3xx_gcu_wait_free(priv);
		if (ret < 0)
			return ret;
	}

	/*
	 * Get buffer from free list
	 */
	spin_lock_irqsave(&priv->spinlock, flags);

	buffer = priv->free;
	priv->free = buffer->next;

	spin_unlock_irqrestore(&priv->spinlock, flags);


	/* Copy data from user into buffer */
	ret = copy_from_user(buffer->ptr, buff, words * 4);
	if (ret) {
		spin_lock_irqsave(&priv->spinlock, flags);
		buffer->next = priv->free;
		priv->free = buffer;
		spin_unlock_irqrestore(&priv->spinlock, flags);
		return -EFAULT;
	}

	buffer->length = words;

	/* Append batch buffer end command */
	buffer->ptr[words] = 0x01000000;

	/*
	 * Add buffer to ready list
	 */
	spin_lock_irqsave(&priv->spinlock, flags);

	buffer->next = NULL;

	if (priv->ready) {
		BUG_ON(priv->ready_last == NULL);

		priv->ready_last->next = buffer;
	} else
		priv->ready = buffer;

	priv->ready_last = buffer;

	if (!priv->shared->hw_running)
		run_ready(priv);

	spin_unlock_irqrestore(&priv->spinlock, flags);

	return words * 4;
}


static long
pxa3xx_gcu_misc_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	unsigned long flags;
	struct pxa3xx_gcu_priv *priv =
		container_of(filp->f_op, struct pxa3xx_gcu_priv, misc_fops);

	switch (cmd) {
	case PXA3XX_GCU_IOCTL_RESET:
		spin_lock_irqsave(&priv->spinlock, flags);
		pxa3xx_gcu_reset(priv);
		spin_unlock_irqrestore(&priv->spinlock, flags);
		return 0;

	case PXA3XX_GCU_IOCTL_WAIT_IDLE:
		return pxa3xx_gcu_wait_idle(priv);

	case PXA3XX_GCU_IOCTL_ALLOC_BUF:
		return pxa3xx_gcu_buf_alloc(priv, filp,
				(struct pxa3xx_gcu_buf_req __user *) arg);

	case PXA3XX_GCU_IOCTL_FREE_BUF:
		return pxa3xx_gcu_buf_free(priv, filp, (u32 __user *) arg);

	case PXA3XX_GCU_IOCTL_SYNC_BUF:
		return pxa3xx_gcu_buf_sync(priv, filp,
				(struct pxa3xx_gcu_sync_req __user *) arg);
	}

	return -ENOSYS;
}

static int
pxa3xx_gcu_misc_mmap(struct file *filp, struct vm_area_struct *vma)
{
	unsigned int size = vma->vm_end - vma->vm_start;
	struct pxa3xx_gcu_priv *priv =
		container_of(filp->f_op, struct pxa3xx_gcu_priv, misc_fops);

	if (vma->vm_pgoff >= (PXA3XX_GCU_BUF_MMAP_BASE >> PAGE_SHIFT))
		return pxa3xx_gcu_buf_mmap(priv, filp, vma);

	switch (vma->vm_pgoff) {
	case 0:
		/* hand out the shared data area */
		if (size != SHARED_SIZE)
			return -EINVAL;

		return dma_mmap_coherent(NULL, vma,
			priv->shared, priv->shared_phys, size);

	case SHARED_SIZE >> PAGE_SHIFT:
		/* hand out the MMIO base for direct register access
		 * from userspace */
		if (size != resource_size(priv->resource_mem))
			return -EINVAL;

		vma->vm_flags |= VM_IO;
		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

		return io_remap_pfn_range(vma, vma->vm_start,
				priv->resource_mem->start >> PAGE_SHIFT,
				size, vma->vm_page_prot);
	}

	return -EINVAL;
}


#ifdef PXA3XX_GCU_DEBUG_TIMER
static struct timer_list pxa3xx_gcu_debug_timer;

static void pxa3xx_gcu_debug_timedout(unsigned long ptr)
{
	struct pxa3xx_gcu_priv *priv = (struct pxa3xx_gcu_priv *) ptr;

	QERROR("Timer DUMP");

	/* init the timer structure */
	init_timer(&pxa3xx_gcu_debug_timer);
	pxa3xx_gcu_debug_timer.function = pxa3xx_gcu_debug_timedout;
	pxa3xx_gcu_debug_timer.data = ptr;
	pxa3xx_gcu_debug_timer.expires = jiffies + 5*HZ; /* one second */

	add_timer(&pxa3xx_gcu_debug_timer);
}

static void pxa3xx_gcu_init_debug_timer(void)
{
	pxa3xx_gcu_debug_timedout((unsigned long) &pxa3xx_gcu_debug_timer);
}
#else
static inline void pxa3xx_gcu_init_debug_timer(void) {}
#endif

static int
add_buffer(struct platform_device *dev,
	   struct pxa3xx_gcu_priv *priv)
{
	struct pxa3xx_gcu_batch *buffer;

	buffer = kzalloc(sizeof(struct pxa3xx_gcu_batch), GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	buffer->ptr = dma_alloc_coherent(&dev->dev, PXA3XX_GCU_BATCH_WORDS * 4,
					 &buffer->phys, GFP_KERNEL);
	if (!buffer->ptr) {
		kfree(buffer);
		return -ENOMEM;
	}

	buffer->next = priv->free;

	priv->free = buffer;

	return 0;
}

static void
free_buffers(struct platform_device *dev,
	     struct pxa3xx_gcu_priv *priv)
{
	struct pxa3xx_gcu_batch *next, *buffer = priv->free;

	while (buffer) {
		next = buffer->next;

		dma_free_coherent(&dev->dev, PXA3XX_GCU_BATCH_WORDS * 4,
				  buffer->ptr, buffer->phys);

		kfree(buffer);

		buffer = next;
	}

	priv->free = NULL;
}

static int __devinit
pxa3xx_gcu_probe(struct platform_device *dev)
{
	int i, ret, irq;
	struct resource *r;
	struct pxa3xx_gcu_priv *priv;

	priv = kzalloc(sizeof(struct pxa3xx_gcu_priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = &dev->dev;
	mutex_init(&priv->buf_lock);

	for (i = 0; i < 8; i++) {
		ret = add_buffer(dev, priv);
		if (ret) {
			dev_err(&dev->dev, "failed to allocate DMA memory\n");
			goto err_free_priv;
		}
	}

	init_waitqueue_head(&priv->wait_idle);
	init_waitqueue_head(&priv->wait_free);
	spin_lock_init(&priv->spinlock);

	/* we allocate the misc device structure as part of our own allocation,
	 * so we can get a pointer to our priv structure later on with
	 * container_of(). This isn't really necessary as we have a fixed minor
	 * number anyway, but this is to avoid statics. */

	priv->misc_fops.owner	= THIS_MODULE;
	priv->misc_fops.write	= pxa3xx_gcu_misc_write;
	priv->misc_fops.unlocked_ioctl = pxa3xx_gcu_misc_ioctl;
	priv->misc_fops.mmap	= pxa3xx_gcu_misc_mmap;
	priv->misc_fops.release	= pxa3xx_gcu_misc_release;

	priv->misc_dev.minor	= MISCDEV_MINOR,
	priv->misc_dev.name	= DRV_NAME,
	priv->misc_dev.fops	= &priv->misc_fops,

	/* register misc device */
	ret = misc_register(&priv->misc_dev);
	if (ret < 0) {
		dev_err(&dev->dev, "misc_register() for minor %d failed\n",
			MISCDEV_MINOR);
		goto err_free_priv;
	}

	/* handle IO resources */
	r = platform_get_resource(dev, IORESOURCE_MEM, 0);
	if (r == NULL) {
		dev_err(&dev->dev, "no I/O memory resource defined\n");
		ret = -ENODEV;
		goto err_misc_deregister;
	}

	if (!request_mem_region(r->start, resource_size(r), dev->name)) {
		dev_err(&dev->dev, "failed to request I/O memory\n");
		ret = -EBUSY;
		goto err_misc_deregister;
	}

	priv->mmio_base = ioremap_nocache(r->start, resource_size(r));
	if (!priv->mmio_base) {
		dev_err(&dev->dev, "failed to map I/O memory\n");
		ret = -EBUSY;
		goto err_free_mem_region;
	}

	/* allocate dma memory */
	priv->shared = dma_alloc_coherent(&dev->dev, SHARED_SIZE,
					  &priv->shared_phys, GFP_KERNEL);

	if (!priv->shared) {
		dev_err(&dev->dev, "failed to allocate DMA memory\n");
		ret = -ENOMEM;
		goto err_free_io;
	}

	/* enable the clock */
	priv->clk = clk_get(&dev->dev, NULL);
	if (IS_ERR(priv->clk)) {
		dev_err(&dev->dev, "failed to get clock\n");
		ret = -ENODEV;
		goto err_free_dma;
	}

	ret = clk_enable(priv->clk);
	if (ret < 0) {
		dev_err(&dev->dev, "failed to enable clock\n");
		goto err_put_clk;
	}

	/* request the IRQ */
	irq = platform_get_irq(dev, 0);
	if (irq < 0) {
		dev_err(&dev->dev, "no IRQ defined\n");
		ret = -ENODEV;
		goto err_put_clk;
	}

	ret = request_irq(irq, pxa3xx_gcu_handle_irq,
			  IRQF_DISABLED, DRV_NAME, priv);
	if (ret) {
		dev_err(&dev->dev, "request_irq failed\n");
		ret = -EBUSY;
		goto err_put_clk;
	}

	platform_set_drvdata(dev, priv);
	priv->resource_mem = r;
	pxa3xx_gcu_reset(priv);
	pxa3xx_gcu_init_debug_timer();

	dev_info(&dev->dev, "registered @0x%p, DMA 0x%p (%d bytes), IRQ %d, "
			"up to %d contiguous user buffers of %d bytes\n",
			(void *) r->start, (void *) priv->shared_phys,
			SHARED_SIZE, irq, PXA3XX_GCU_MAX_BUFS,
			PXA3XX_GCU_BUF_MAX_SIZE);
	return 0;

err_put_clk:
	clk_disable(priv->clk);
	clk_put(priv->clk);

err_free_dma:
	dma_free_coherent(&dev->dev, SHARED_SIZE,
			priv->shared, priv->shared_phys);

err_free_io:
	iounmap(priv->mmio_base);

err_free_mem_region:
	release_mem_region(r->start, resource_size(r));

err_misc_deregister:
	misc_deregister(&priv->misc_dev);

err_free_priv:
	platform_set_drvdata(dev, NULL);
	free_buffers(dev, priv);
	kfree(priv);
	return ret;
}

static int __devexit
pxa3xx_gcu_remove(struct platform_device *dev)
{
	struct pxa3xx_gcu_priv *priv = platform_get_drvdata(dev);
	struct resource *r = priv->resource_mem;

	pxa3xx_gcu_wait_idle(priv);

	misc_deregister(&priv->misc_dev);

	/* normally empty: every file release frees its buffers */
	{
		int i;

		mutex_lock(&priv->buf_lock);
		for (i = 0; i < PXA3XX_GCU_MAX_BUFS; i++)
			if (priv->bufs[i].kaddr)
				buf_release(&priv->bufs[i]);
		mutex_unlock(&priv->buf_lock);
	}
	dma_free_coherent(&dev->dev, SHARED_SIZE,
			priv->shared, priv->shared_phys);
	iounmap(priv->mmio_base);
	release_mem_region(r->start, resource_size(r));
	platform_set_drvdata(dev, NULL);
	clk_disable(priv->clk);
	free_buffers(dev, priv);
	kfree(priv);

	return 0;
}

static struct platform_driver pxa3xx_gcu_driver = {
	.probe	  = pxa3xx_gcu_probe,
	.remove	 = __devexit_p(pxa3xx_gcu_remove),
	.driver	 = {
		.owner  = THIS_MODULE,
		.name   = DRV_NAME,
	},
};

static int __init
pxa3xx_gcu_init(void)
{
	return platform_driver_register(&pxa3xx_gcu_driver);
}

static void __exit
pxa3xx_gcu_exit(void)
{
	platform_driver_unregister(&pxa3xx_gcu_driver);
}

module_init(pxa3xx_gcu_init);
module_exit(pxa3xx_gcu_exit);

MODULE_DESCRIPTION("PXA3xx graphics controller unit driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(MISCDEV_MINOR);
MODULE_AUTHOR("Janine Kropp <nin@directfb.org>, "
		"Denis Oliver Kropp <dok@directfb.org>, "
		"Daniel Mack <daniel@caiaq.de>");
