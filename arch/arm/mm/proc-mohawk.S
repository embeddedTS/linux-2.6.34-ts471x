/*
 *  linux/arch/arm/mm/proc-mohawk.S: MMU functions for ARM926EJ-S
 *
 *  Copyright (C) 1999-2001 ARM Limited
 *  Copyright (C) 2000 Deep Blue Solutions Ltd.
 *  hacked for non-paged-MM by Hyok S. Choi, 2003.
 *  Copyright (C) 2008 Marvell International - L2 cache support by
 *                         Jason Chagas <jason.chagas@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *
 * These are the low level assembler for performing cache and TLB
 * functions on the mohawk.
 *
 *  CONFIG_CPU_ARM926_CPU_IDLE -> nohlt
 */

#include <linux/linkage.h>
#include <linux/init.h>
#include <asm/assembler.h>
#include <asm/hwcap.h>
#include <asm/pgtable-hwdef.h>
#include <asm/pgtable.h>
#include <asm/page.h>
#include <asm/ptrace.h>
#include "proc-macros.S"

/*
 * This is the maximum size of an area which will be invalidated
 * using the single invalidate entry instructions.  Anything larger
 * than this, and we go for the whole cache.
 *
 * This value should be chosen such that we choose the cheapest
 * alternative.
 */
#define CACHE_DLIMIT	16384

/*
 * the cache line size of the I and D cache
 */
#define CACHE_DLINESIZE	32

#if !defined(CONFIG_CPU_PXA168_B0)
#define BAD_CACHE_LINE_OFFSET	(PAGE_SZ - CACHE_DLINESIZE)
#endif

/*
 * Run with L2 enabled.
 */
#undef TTC_Z0_SUPPORT
/* #define L2_CACHE_PREFETCH_DISABLE */

#define MOHAWK_CPU_CONF	0xd4282c08

	.text
/*
 * cpu_mohawk_proc_init()
 */

ENTRY(cpu_mohawk_proc_init)
	mov	pc, lr

/*
 * cpu_mohawk_proc_fin()
 */
ENTRY(cpu_mohawk_proc_fin)
	stmfd	sp!, {lr}
	mov	ip, #PSR_F_BIT | PSR_I_BIT | SVC_MODE
	msr	cpsr_c, ip
	bl	mohawk_flush_kern_cache_all
	mrc	p15, 0, r0, c1, c0, 0		@ ctrl register
	bic	r0, r0, #0x1000			@ ...i............
	bic	r0, r0, #0x000e			@ ............wca.
	mcr	p15, 0, r0, c1, c0, 0		@ disable caches
	ldmfd	sp!, {pc}

/*
 * cpu_mohawk_reset(loc)
 *
 * Perform a soft reset of the system.  Put the CPU into the
 * same state as it would be if it had been reset, and branch
 * to what would be the reset vector.
 *
 * loc: location to jump to for soft reset
 */
	.align	5
ENTRY(cpu_mohawk_reset)
	mov	ip, #0
	mcr	p15, 0, ip, c7, c7, 0		@ invalidate I,D caches
	mcr	p15, 0, ip, c7, c10, 4		@ drain WB
#ifdef CONFIG_MMU
	mcr	p15, 0, ip, c8, c7, 0		@ invalidate I & D TLBs
#endif
	ldr	ip, =0x5087a
	mcr	p15, 0, ip, c1, c0, 0		@ ctrl register
	mrc	p15, 0, ip, c1, c0, 0		@ ctrl register
	mov	pc, r0

/*
 * cpu_mohawk_do_idle()
 *
 * Called with IRQs disabled
 */
	.align	10
ENTRY(cpu_mohawk_do_idle)
	mov	r0, #0
#ifdef CONFIG_ENABLE_COREIDLE
	mcr	p15, 0, r0, c7, c0, 4		@ Wait for interrupt
#endif
	mov	pc, lr

/*
 *	flush_user_cache_all()
 *
 *	Clean and invalidate all cache entries in a particular
 *	address space.
 */
ENTRY(mohawk_flush_user_cache_all)
	/* FALLTHROUGH */

/*
 *	flush_kern_cache_all()
 *
 *	Clean and invalidate the entire cache.
 */
ENTRY(mohawk_flush_kern_cache_all)
	mov	r2, #VM_EXEC
	mov	ip, #0
__flush_whole_cache:
#ifdef CONFIG_CPU_DCACHE_WRITETHROUGH
	mcr	p15, 0, ip, c7, c6, 0		@ invalidate D cache
#else
1:	mrc	p15, 0, r15, c7, c14, 3 	@ test,clean,invalidate
	bne	1b
#endif
	tst	r2, #VM_EXEC
	mcrne	p15, 0, ip, c7, c5, 0		@ invalidate I cache
	mcrne	p15, 0, ip, c7, c10, 4		@ drain WB
	mov	pc, lr

/*
 *	flush_user_cache_range(start, end, flags)
 *
 *	Clean and invalidate a range of cache entries in the
 *	specified address range.
 *
 *	- start	- start address (inclusive)
 *	- end	- end address (exclusive)
 *	- flags	- vm_flags describing address space
 */
ENTRY(mohawk_flush_user_cache_range)
	mov	ip, #0
	sub	r3, r1, r0			@ calculate total size
	cmp	r3, #CACHE_DLIMIT
	bgt	__flush_whole_cache
1:	tst	r2, #VM_EXEC
#ifdef CONFIG_CPU_DCACHE_WRITETHROUGH
	mcr	p15, 0, r0, c7, c6, 1		@ invalidate D entry
	mcrne	p15, 0, r0, c7, c5, 1		@ invalidate I entry
	add	r0, r0, #CACHE_DLINESIZE
	mcr	p15, 0, r0, c7, c6, 1		@ invalidate D entry
	mcrne	p15, 0, r0, c7, c5, 1		@ invalidate I entry
	add	r0, r0, #CACHE_DLINESIZE
#else
	mcr	p15, 0, r0, c7, c14, 1		@ clean and invalidate D entry
	mcrne	p15, 0, r0, c7, c5, 1		@ invalidate I entry
	add	r0, r0, #CACHE_DLINESIZE
	mcr	p15, 0, r0, c7, c14, 1		@ clean and invalidate D entry
	mcrne	p15, 0, r0, c7, c5, 1		@ invalidate I entry
	add	r0, r0, #CACHE_DLINESIZE
#endif
	cmp	r0, r1
	blo	1b
	tst	r2, #VM_EXEC
	mcrne	p15, 0, ip, c7, c10, 4		@ drain WB
	mov	pc, lr

/*
 *	coherent_kern_range(start, end)
 *
 *	Ensure coherency between the Icache and the Dcache in the
 *	region described by start, end.  If you have non-snooping
 *	Harvard caches, you need to implement this function.
 *
 *	- start	- virtual start address
 *	- end	- virtual end address
 */
ENTRY(mohawk_coherent_kern_range)
	/* FALLTHROUGH */

/*
 *	coherent_user_range(start, end)
 *
 *	Ensure coherency between the Icache and the Dcache in the
 *	region described by start, end.  If you have non-snooping
 *	Harvard caches, you need to implement this function.
 *
 *	- start	- virtual start address
 *	- end	- virtual end address
 */
ENTRY(mohawk_coherent_user_range)
	bic	r0, r0, #CACHE_DLINESIZE - 1
#if !defined(CONFIG_CPU_PXA168_B0)
#if defined(CONFIG_CPU_L2_CACHE)
	mov	r2,	r0
#endif
#endif
1:	mcr	p15, 0, r0, c7, c10, 1		@ clean D entry
	mcr	p15, 0, r0, c7, c5, 1		@ invalidate I entry
	add	r0, r0, #CACHE_DLINESIZE
	cmp	r0, r1
	blo	1b
	mcr	p15, 0, r0, c7, c10, 4		@ drain WB
#if !defined(CONFIG_CPU_PXA168_B0)
#if defined(CONFIG_CPU_L2_CACHE)
	mov		r0, r2
	add		r0, #BAD_CACHE_LINE_OFFSET
1:	cmp		r0, r1
	bgt		2f
	mcr 	p15, 1, r0, c7, c11, 1		@ l2 clean single entry
	add		r0, #PAGE_SZ
	b		1b
2:
#endif
	mcr	p15, 0, r0, c7, c10, 4		@ drain WB
#endif
	mov	pc, lr

/*
 *	flush_kern_dcache_area(void *addr, size_t size)
 *
 *	Ensure no D cache aliasing occurs, either with itself or
 *	the I cache
 *
 *	- addr	- kernel address
 *	- size	- region size
 */
ENTRY(mohawk_flush_kern_dcache_area)
	add	r1, r0, r1
1:	mcr	p15, 0, r0, c7, c14, 1		@ clean+invalidate D entry
	add	r0, r0, #CACHE_DLINESIZE
	cmp	r0, r1
	blo	1b
	mov	r0, #0
	mcr	p15, 0, r0, c7, c5, 0		@ invalidate I cache
	mcr	p15, 0, r0, c7, c10, 4		@ drain WB
	mov	pc, lr

/*
 *	flush_kern_dcache_page(void *page)
 *
 *	Ensure no D cache aliasing occurs, either with itself or
 *	the I cache
 *
 *	- addr	- page aligned address
 */
ENTRY(mohawk_flush_kern_dcache_page)
	add	r1, r0, #PAGE_SZ
1:	mcr	p15, 0, r0, c7, c14, 1		@ clean+invalidate D entry
	add	r0, r0, #CACHE_DLINESIZE
	cmp	r0, r1
	blo	1b
#if !defined(CONFIG_CPU_PXA168_B0)
#if defined(CONFIG_CPU_L2_CACHE)
	sub		r1, r1,	#CACHE_DLINESIZE
	mcr		p15, 1, r1, c7, c15, 1		@ L2C clean+invalidate entry
#endif
#endif
	mov	r0, #0
	mcr	p15, 0, r0, c7, c5, 0		@ invalidate I cache
	mcr	p15, 0, r0, c7, c10, 4		@ drain WB
	mov	pc, lr

/*
 *	dma_inv_range(start, end)
 *
 *	Invalidate (discard) the specified virtual address range.
 *	May not write back any entries.  If 'start' or 'end'
 *	are not cache line aligned, those lines must be written
 *	back.
 *
 *	- start	- virtual start address
 *	- end	- virtual end address
 *
 * (same as v4wb)
 */
ENTRY(mohawk_dma_inv_range)
#ifndef CONFIG_CPU_DCACHE_WRITETHROUGH
	tst	r0, #CACHE_DLINESIZE - 1
	mcrne	p15, 0, r0, c7, c10, 1		@ clean D entry
	mcrne	p15, 1, r0, c7, c11, 1		@ clean L2 entry
	tst	r1, #CACHE_DLINESIZE - 1
	mcrne	p15, 0, r1, c7, c10, 1		@ clean D entry
	mcrne	p15, 1, r1, c7, c11, 1		@ clean L2 entry
#endif
	bic	r0, r0, #CACHE_DLINESIZE - 1
1:	mcr	p15, 0, r0, c7, c6, 1		@ invalidate D entry
	mcr	p15, 1, r0, c7, c7, 1		@ invalidate L2 entry
	add	r0, r0, #CACHE_DLINESIZE
	cmp	r0, r1
	blo	1b
#if defined(CONFIG_CPU_L2_CACHE) && !defined(L2_CACHE_PREFETCH_DISABLE)
	tst	r0, #CACHE_DLINESIZE		@ odd last cacheline?
	mcrne	p15, 0, r0, c7, c14, 1		@ clean+invalidate D entry
	mcrne	p15, 1, r0, c7, c15, 1		@ L2C clean+invalidate entry
#endif
	mcr	p15, 0, r0, c7, c10, 4		@ drain WB
	mov	pc, lr

/*
 *	dma_clean_range(start, end)
 *
 *	Clean the specified virtual address range.
 *
 *	- start	- virtual start address
 *	- end	- virtual end address
 *
 * (same as v4wb)
 */
ENTRY(mohawk_dma_clean_range)
#ifndef CONFIG_CPU_DCACHE_WRITETHROUGH
	bic	r0, r0, #CACHE_DLINESIZE - 1
1:	mcr	p15, 0, r0, c7, c10, 1		@ clean D entry
	mcr	p15, 1, r0, c7, c11, 1		@ clean L2 entry
	add	r0, r0, #CACHE_DLINESIZE
	cmp	r0, r1
	blo	1b
#if defined(CONFIG_CPU_L2_CACHE) && !defined(L2_CACHE_PREFETCH_DISABLE)
	tst	r0, #CACHE_DLINESIZE		@ odd last cacheline?
	mcrne	p15, 0, r0, c7, c10, 1		@ clean D entry
	mcrne	p15, 1, r0, c7, c11, 1		@ L2C clean entry
#endif
#endif
	mcr	p15, 0, r0, c7, c10, 4		@ drain WB
	mov	pc, lr

/*
 *	dma_flush_range(start, end)
 *
 *	Clean and invalidate the specified virtual address range.
 *
 *	- start	- virtual start address
 *	- end	- virtual end address
 */
ENTRY(mohawk_dma_flush_range)
	bic	r0, r0, #CACHE_DLINESIZE - 1
1:
#ifndef CONFIG_CPU_DCACHE_WRITETHROUGH
	mcr	p15, 0, r0, c7, c14, 1		@ clean+invalidate D entry
	mcr	p15, 1, r0, c7, c15, 1		@ clean+invalidate L2 entry
#else
	mcr	p15, 0, r0, c7, c10, 1		@ clean D entry
	mcr	p15, 1, r0, c7, c11, 1		@ clean L2 entry
#endif
	add	r0, r0, #CACHE_DLINESIZE
	cmp	r0, r1
	blo	1b
#if defined(CONFIG_CPU_L2_CACHE) && !defined(L2_CACHE_PREFETCH_DISABLE)
	tst	r0, #CACHE_DLINESIZE		@ odd last cacheline?
	mcrne	p15, 0, r0, c7, c14, 1		@ clean+invalidate D entry
	mcrne	p15, 1, r0, c7, c15, 1		@ L2C clean+invalidate entry
#endif
	mcr	p15, 0, r0, c7, c10, 4		@ drain WB
	mov	pc, lr

/*
 *	dma_map_area(start, size, dir)
 *	- start	- kernel virtual start address
 *	- size	- size of region
 *	- dir	- DMA direction
 */
ENTRY(mohawk_dma_map_area)
	add	r1, r1, r0
	cmp	r2, #DMA_TO_DEVICE
	beq	mohawk_dma_clean_range
	bcs	mohawk_dma_inv_range
	b	mohawk_dma_flush_range
ENDPROC(mohawk_dma_map_area)

/*
 *	dma_unmap_area(start, size, dir)
 *	- start	- kernel virtual start address
 *	- size	- size of region
 *	- dir	- DMA direction
 */
ENTRY(mohawk_dma_unmap_area)
	mov	pc, lr
ENDPROC(mohawk_dma_unmap_area)

ENTRY(mohawk_cache_fns)
	.long	mohawk_flush_kern_cache_all
	.long	mohawk_flush_user_cache_all
	.long	mohawk_flush_user_cache_range
	.long	mohawk_coherent_kern_range
	.long	mohawk_coherent_user_range
	.long	mohawk_flush_kern_dcache_area
	.long	mohawk_dma_map_area
	.long	mohawk_dma_unmap_area
	.long	mohawk_dma_flush_range

ENTRY(cpu_mohawk_dcache_clean_area)
#ifndef CONFIG_CPU_DCACHE_WRITETHROUGH
1:	mcr	p15, 0, r0, c7, c10, 1		@ clean D entry
	add	r0, r0, #CACHE_DLINESIZE
	subs	r1, r1, #CACHE_DLINESIZE
	bhi	1b
#endif
	mcr	p15, 0, r0, c7, c10, 4		@ drain WB
	mov	pc, lr

/* =============================== PageTable ============================== */

/*
 * cpu_mohawk_switch_mm(pgd)
 *
 * Set the translation base pointer to be as described by pgd.
 *
 * pgd: new page tables
 */
	.align	5
ENTRY(cpu_mohawk_switch_mm)
#ifdef CONFIG_MMU
	mov	ip, #0
#ifdef CONFIG_CPU_DCACHE_WRITETHROUGH
	mcr	p15, 0, ip, c7, c6, 0		@ invalidate D cache
#else
@ && 'Clean & Invalidate whole DCache'
1:	mrc	p15, 0, r15, c7, c14, 3 	@ test,clean,invalidate
	bne	1b
#endif
	mcr	p15, 0, ip, c7, c5, 0		@ invalidate I cache
	mcr	p15, 0, ip, c7, c10, 4		@ drain WB

#ifdef CONFIG_CPU_DCACHE_WRITETHROUGH
#if defined(CONFIG_CPU_L2_CACHE)
	orr	r0, r0, #0x10			@ L2 - Outer write-through
#endif
#else
#if defined(CONFIG_CPU_L2_CACHE)
	orr	r0, r0, #0x18			@ L2 - Outer write-back
#endif
#endif

	mcr	p15, 0, r0, c2, c0, 0		@ load page table pointer
	mcr	p15, 0, ip, c8, c7, 0		@ invalidate I & D TLBs
#endif
	mov	pc, lr

/*
 * cpu_mohawk_set_pte_ext(ptep, pte, ext)
 *
 * Set a PTE and flush it out
 */
	.align	5
ENTRY(cpu_mohawk_set_pte_ext)
#ifdef CONFIG_MMU
	str	r1, [r0], #-2048		@ linux version

	eor	r1, r1, #L_PTE_PRESENT | L_PTE_YOUNG | L_PTE_WRITE | L_PTE_DIRTY

	bic	r2, r1, #PTE_SMALL_AP_MASK
	bic	r2, r2, #PTE_TYPE_MASK
	orr	r2, r2, #PTE_TYPE_SMALL

	tst	r1, #L_PTE_USER			@ User?
	orrne	r2, r2, #PTE_SMALL_AP_URO_SRW

	tst	r1, #L_PTE_WRITE | L_PTE_DIRTY	@ Write and Dirty?
	orreq	r2, r2, #PTE_SMALL_AP_UNO_SRW

	tst	r1, #L_PTE_PRESENT | L_PTE_YOUNG	@ Present and Young?
	movne	r2, #0

#ifdef CONFIG_CPU_DCACHE_WRITETHROUGH
	eor	r3, r2, #0x0a			@ C & small page?
	tst	r3, #0x0b
	biceq	r2, r2, #4
#endif
	str	r2, [r0]			@ hardware version
	mov	r0, r0
#ifndef CONFIG_CPU_DCACHE_WRITETHROUGH
	mcr	p15, 0, r0, c7, c10, 1		@ clean D entry
#endif
	mcr	p15, 0, r0, c7, c10, 4		@ drain WB
#endif
	mov	pc, lr

	__INIT

	.type	__mohawk_setup, #function
__mohawk_setup:
	mov	r0, #0
	mcr	p15, 0, r0, c7, c7		@ invalidate I,D caches on v4
	mcr	p15, 0, r0, c7, c10, 4		@ drain write buffer on v4
#ifdef CONFIG_MMU
	mcr	p15, 0, r0, c8, c7		@ invalidate I,D TLBs on v4
#if defined(CONFIG_CPU_L2_CACHE)
	mcr	p15, 1, r0, c7, c7, 0		@ invalidate L2 cache
#endif
#endif


#ifdef CONFIG_CPU_DCACHE_WRITETHROUGH
	mov	r0, #4				@ disable write-back on caches explicitly
	mcr	p15, 7, r0, c15, c0, 0
#if defined(CONFIG_CPU_L2_CACHE)
	orr	r4, r4, #0x10			@ L2 - Outer write through
#endif
#else
#if defined(CONFIG_CPU_L2_CACHE)
	orr	r4, r4, #0x18			@ L2 - Outer write back
#endif
#endif 

#if defined(CONFIG_CPU_L2_CACHE) && defined(L2_CACHE_PREFETCH_DISABLE)
	mrc	p15, 1, r0, c15, c1, 0		@ get extra features register
	orr	r0, r0, #(1 << 24)		@ L2 prefetch disable
	mcr	p15, 1, r0, c15, c1, 0		@ set extra features register
#endif

	adr	r5, mohawk_crval
	ldmia	r5, {r5, r6}
	mrc	p15, 0, r0, c1, c0		@ get control register v4
	bic	r0, r0, r5
	orr	r0, r0, r6
	orr	r0, r0, #0x800			@ BPU enable
#ifdef CONFIG_CPU_CACHE_ROUND_ROBIN
	orr	r0, r0, #0x4000			@ .1.. .... .... ....
#endif
#if defined(CONFIG_CPU_L2_CACHE)
#ifdef TTC_Z0_SUPPORT
	ldr	r6, =0xd4282c00
	ldr	r5, [r6]
	mov 	r5, r5, lsl #8
	mov	r5, r5, lsr #28
	cmp	r5, #0xa			@ check TTC A0 stepping or not
	orrne	r0, r0, #(1 << 26)		@ L2 enable - xscale/PJ41/PJ4

	@ Must also pre-enable L2 cache _before_ L1 is enabled
	mrcne	p15, 0, r5, c1, c0, 0		@ read control reg
	orrne	r5, r5, #(1 << 26)		@ L2 enable - PJ1
	mcrne	p15, 0, r5, c1, c0, 0		@ write control reg
#else
	orr	r0, r0, #(1 << 26)		@ L2 enable - xscale/PJ41/PJ4

	@ Must also pre-enable L2 cache _before_ L1 is enabled
	mrc	p15, 0, r5, c1, c0, 0		@ read control reg
	orr	r5, r5, #(1 << 26)		@ L2 enable - PJ1
	mcr	p15, 0, r5, c1, c0, 0		@ write control reg
#endif
#endif
	mrc	p15, 1, r5, c15, c1, 0		@ get extra features register
	orr	r5, r5, #(1<<29)		@ streaming enable
	orr     r5, r5, #0x700                  @ 8-wait cycles write coalesce
	mcr     p15, 1, r5, c15, c1, 0          @ set extra features register

	@ Must disable write buffer merge due to PJ1 bug in afifo_merge logic
	ldr	r6, =MOHAWK_CPU_CONF
	ldr	r5, [r6]
	orr	r5, r5, #(1<<5)			@ disable write buffer merge
	str	r5, [r6]

	mov	pc, lr
	.size	__mohawk_setup, . - __mohawk_setup

	/*
	 *  R
	 * .RVI ZFRS BLDP WCAM
	 * .011 0001 ..11 0101
	 * 
	 */
	.type	mohawk_crval, #object
mohawk_crval:
	crval	clear=0x00007f3f, mmuset=0x00003135, ucset=0x00001134

	__INITDATA

/*
 * Purpose : Function pointers used to access above functions - all calls
 *	     come through these
 */
	.type	mohawk_processor_functions, #object
mohawk_processor_functions:
	.word	v5t_early_abort
	.word	cpu_mohawk_proc_init
	.word	cpu_mohawk_proc_fin
	.word	cpu_mohawk_reset
	.word	cpu_mohawk_do_idle
	.word	cpu_mohawk_dcache_clean_area
	.word	cpu_mohawk_switch_mm
	.word	cpu_mohawk_set_pte_ext
	.size	mohawk_processor_functions, . - mohawk_processor_functions

	.section ".rodata"

	.type	cpu_arch_name, #object
cpu_arch_name:
	.asciz	"armv5tej"
	.size	cpu_arch_name, . - cpu_arch_name

	.type	cpu_elf_name, #object
cpu_elf_name:
	.asciz	"v5"
	.size	cpu_elf_name, . - cpu_elf_name

	.type	cpu_mohawk_name, #object
cpu_mohawk_name:
	.asciz	"Marvell Mohawk"
	.size	cpu_mohawk_name, . - cpu_mohawk_name

	.align

	.section ".proc.info.init", #alloc, #execinstr

#ifdef CONFIG_CPU_MOHAWK_OLD_ID
	.type	__mohawk_old_id_proc_info,#object
	__mohawk_old_id_proc_info:
	.long	0x41159260
	.long	0xfffffff0
	.long   PMD_TYPE_SECT | \
		PMD_SECT_BUFFERABLE | \
		PMD_SECT_CACHEABLE | \
		PMD_BIT4 | \
		PMD_SECT_AP_WRITE | \
		PMD_SECT_AP_READ
		.long   PMD_TYPE_SECT | \
			PMD_BIT4 | \
			PMD_SECT_AP_WRITE | \
			PMD_SECT_AP_READ
			b	__mohawk_setup
			.long	cpu_arch_name
			.long	cpu_elf_name
			.long	HWCAP_SWP|HWCAP_HALF|HWCAP_THUMB|HWCAP_FAST_MULT|HWCAP_EDSP
			.long	cpu_mohawk_name
			.long	mohawk_processor_functions
			.long	v4wbi_tlb_fns
			.long	v4wb_user_fns
			.long	mohawk_cache_fns
			.size	__mohawk_old_id_proc_info, . - __mohawk_old_id_proc_info
#endif

	.type	__mohawk_proc_info,#object
__mohawk_proc_info:
	.long	0x56158000			@ Marvell PXA168
	.long	0xfffff000
	.long   PMD_TYPE_SECT | \
		PMD_SECT_BUFFERABLE | \
		PMD_SECT_CACHEABLE | \
		PMD_BIT4 | \
		PMD_SECT_AP_WRITE | \
		PMD_SECT_AP_READ
	.long   PMD_TYPE_SECT | \
		PMD_BIT4 | \
		PMD_SECT_AP_WRITE | \
		PMD_SECT_AP_READ
	b	__mohawk_setup
	.long	cpu_arch_name
	.long	cpu_elf_name
	.long	HWCAP_SWP|HWCAP_HALF|HWCAP_THUMB|HWCAP_FAST_MULT|HWCAP_EDSP|HWCAP_JAVA
	.long	cpu_mohawk_name
	.long	mohawk_processor_functions
	.long	v4wbi_tlb_fns
	.long	v4wb_user_fns
	.long	mohawk_cache_fns
	.size	__mohawk_proc_info, . - __mohawk_proc_info

