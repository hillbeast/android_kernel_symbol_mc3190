#ifndef __PXA3XX_GCU_H__
#define __PXA3XX_GCU_H__

#include <linux/types.h>

/* Number of 32bit words in display list (ring buffer). */
#define PXA3XX_GCU_BUFFER_WORDS  ((256 * 1024 - 256) / 4)

/* To be increased when breaking the ABI */
#define PXA3XX_GCU_SHARED_MAGIC  0x30000001

#define PXA3XX_GCU_BATCH_WORDS   8192

struct pxa3xx_gcu_shared {
	u32            buffer[PXA3XX_GCU_BUFFER_WORDS];

	bool           hw_running;

	unsigned long  buffer_phys;

	unsigned int   num_words;
	unsigned int   num_writes;
	unsigned int   num_done;
	unsigned int   num_interrupts;
	unsigned int   num_wait_idle;
	unsigned int   num_wait_free;
	unsigned int   num_idle;

	u32            magic;
};

/* Initialization and synchronization.
 * Hardware is started upon write(). */
#define PXA3XX_GCU_IOCTL_RESET		_IO('G', 0)
#define PXA3XX_GCU_IOCTL_WAIT_IDLE	_IO('G', 2)

/* Physically contiguous, CPU-cached buffers. */

#define PXA3XX_GCU_MAX_BUFS		4
#define PXA3XX_GCU_BUF_MAX_SIZE		(1024 * 1024)

/* mmap() offset of buffer <id> is BASE + id * STRIDE (also returned by the
 * ALLOC ioctl, so user space need not compute it). */
#define PXA3XX_GCU_BUF_MMAP_BASE	0x01000000UL
#define PXA3XX_GCU_BUF_MMAP_STRIDE	0x00100000UL

/* ALLOC_BUF flags */
#define PXA3XX_GCU_BUF_WRITETHROUGH	1	/* map the buffer write-through */

struct pxa3xx_gcu_buf_req {
	__u32 size;			/* in:  requested size in bytes */
	__u32 id;			/* out: buffer id */
	__u32 phys;			/* out: physical address (use in GCU commands) */
	__u32 mmap_offset;	/* out: offset to pass to mmap() */
	__u32 alloc_size;	/* out: size actually allocated (page multiple) */
	__u32 flags;		/* in:  PXA3XX_GCU_BUF_* */
};

#define PXA3XX_GCU_SYNC_TO_DEVICE	1
#define PXA3XX_GCU_SYNC_FROM_DEVICE	2
#define PXA3XX_GCU_SYNC_DRAIN		4

struct pxa3xx_gcu_sync_req {
	__u32 id;
	__u32 offset;		/* byte offset into the buffer */
	__u32 len;			/* number of bytes */
	__u32 flags;		/* PXA3XX_GCU_SYNC_*; 0 = full clean + invalidate */
};

#define PXA3XX_GCU_IOCTL_ALLOC_BUF	_IOWR('G', 3, struct pxa3xx_gcu_buf_req)
#define PXA3XX_GCU_IOCTL_FREE_BUF	_IOW('G', 4, __u32)
#define PXA3XX_GCU_IOCTL_SYNC_BUF	_IOW('G', 5, struct pxa3xx_gcu_sync_req)

#endif /* __PXA3XX_GCU_H__ */

