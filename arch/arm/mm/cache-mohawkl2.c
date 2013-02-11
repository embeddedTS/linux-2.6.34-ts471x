/*
 * arch/arm/mm/cache-mohawkl2.c
 *
 *  Copyright (C) 2010 Marvell International Ltd.
 *  All rights reserved.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file COPYING in the main directory of this archive for
 * more details.
 */

/* L2 cache maintenance operations */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/memory.h>

#define CACHE_LINE_SIZE	32

enum cache_op {
	CLEAN = 0,
	INVALIDATE = 1,
	FLUSH = 2,
};

/* clean single mva entry */
static inline void mohawk_l2_clean_mva(unsigned long addr)
{
	__asm__("mcr p15, 1, %0, c7, c11, 1" : : "r" (addr));
}

/* clean all l2 entries */
static inline void mohawk_l2_clean_all(void)
{
	__asm__("mcr p15, 1, %0, c7, c11, 0" : : "r" (0));
}

/* invalidate single mva entry */
static inline void mohawk_l2_inv_mva(unsigned long addr)
{
	__asm__("mcr p15, 1, %0, c7, c7, 1" : : "r" (addr));
}

/* invalidate all l2 entries */
static inline void mohawk_l2_inv_all(void)
{
	__asm__("mcr p15, 1, %0, c7, c7, 0" : : "r" (0));
}

/* flush: clean + invalidate single mva entry */
static inline void mohawk_l2_flush_mva(unsigned long addr)
{
	__asm__("mcr p15, 1, %0, c7, c15, 1" : : "r" (addr));
}

static inline void drain_write_buffer(void)
{
	__asm__("mcr p15, 0, %0, c7, c10, 4" : : "r" (0));
}

static inline void cache_l2_op(enum cache_op op, unsigned long start,
				unsigned long end)
{
	unsigned long vaddr;

	/* perform cache maintenance on all cache lines */
	if (unlikely(start == 0 && end == -1ul)) {
		switch (op) {
		case INVALIDATE:
			mohawk_l2_inv_all();
			break;
		case CLEAN:
			mohawk_l2_clean_all();
			break;
		case FLUSH:
			mohawk_l2_clean_all();
			mohawk_l2_inv_all();
			break;
		default:
			BUG();
		}

		drain_write_buffer();
		return;
	}

	/* align pa on cache line */
	start = start & ~(CACHE_LINE_SIZE-1);

	while (start < end) {
		/* convert pa to va XXX highmem not supported XXX */
		vaddr = __phys_to_virt(start);

		switch (op) {
		case INVALIDATE:
			mohawk_l2_inv_mva(vaddr);
			break;
		case CLEAN:
			mohawk_l2_clean_mva(vaddr);
			break;
		case FLUSH:
			mohawk_l2_flush_mva(vaddr);
			break;
		default:
			BUG();
		}
		start += CACHE_LINE_SIZE;
	}

	drain_write_buffer();
}

static void mohawk_l2_inv_range(unsigned long start, unsigned long end)
{
	cache_l2_op(INVALIDATE, start, end);
}

static void mohawk_l2_clean_range(unsigned long start, unsigned long end)
{
	cache_l2_op(CLEAN, start, end);
}

static void mohawk_l2_flush_range(unsigned long start, unsigned long end)
{
	cache_l2_op(FLUSH, start, end);
}

static int __init mohawk_l2_init(void)
{
	pr_info("Mohawk L2 cache enabled.\n");
	outer_cache.inv_range = mohawk_l2_inv_range;
	outer_cache.clean_range = mohawk_l2_clean_range;
	outer_cache.flush_range = mohawk_l2_flush_range;

	return 0;
}

core_initcall(mohawk_l2_init);

/* HIGHMEM is not supported currently */
#ifdef CONFIG_HIGHMEM
#error HIGHMEM is not supported
#endif
