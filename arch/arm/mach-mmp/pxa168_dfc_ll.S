/*
 * Low-level PXA168 hibernate mode support
 *
 * Copyright (C) 2009, Marvell Corporation.
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 */
#include <linux/linkage.h>
#include <asm/assembler.h>
#include <mach/hardware.h>
#include <mach/pxa168_pm.h>
#include <mach/regs-mpmu.h>
/* pxa168_trigger_dfc()
 * Entry point for trigger the DFC
 *
 */
	.text
	.align 5

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ r0: apmu_ccr value
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@


@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@      trigger dfc
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
#define DDR_SR_BY_APMU
#undef DDR_SR_BY_DMC

#define RUN_DFC_IN_CACHE
#undef RUN_DFC_IN_SRAM

ENTRY(pxa168_dfc_get_lockcache_location)

	stmfd	sp!, {r3 - r12, lr}

	ldr	r2, =lock_cache_start
	str	r2, [r0]
	ldr	r3, =lock_cache_end
	sub	r3, r3, r2
	str	r3, [r1]

	mov r0, #0
        ldmfd   sp!, {r3 - r12, pc}

ENTRY(pxa168_trigger_dfc)

	stmfd	sp!, {r3 - r12, lr}
	mov r11, r0

#ifdef RUN_DFC_IN_SRAM
	@ Step 1
	@ map L2 cache to SQU as SRAM
	mcr	p15, 1, r0, c7, c11, 0	/* clean L2 cache */
	mcr	p15, 1, r0, c7, c7, 0	/* invalidate L2 cache */
	mrc	p15, 0, r0, c1, c0, 0
	bic	r0, r0, #(1 << 26)
	mcr	p15, 0, r0, c1, c0, 0	/* disable L2 cache */

	ldr	r0, =0xfe282c08		/* test L2 IDLE bit */
	ldr	r0, [r0]
100:	tst	r0, #(1 << 16)
	beq	100b

	ldr	r0, =0xfe2a0030		/* enable SQU bank3 */
	ldr	r1, [r0]
	orr	r1, #0x1
	str	r1, [r0]

	ldr	r0, =0xfe282c08		/* L2 cache to SQU */
	ldr	r1, [r0]
	orr	r1, #0x10
	str	r1, [r0]

	@ Step 2
	@ copy the DFC code to ISRAM
	ldr	r0, =sram_membase
	ldr	r0, [r0]
	ldr	r3, =dfc_start
	ldr	r4, =dfc_end
	add	r4, r4, #0x100

dfc_rel_ram:
	ldmia	r3!, {r5 - r10}
	stmia	r0!, {r5 - r10}
	cmp	r3, r4
	ble	dfc_rel_ram

	ldr	r0, =sram_membase
	ldr	r0, [r0]

	mov	pc, r0

#endif


#ifdef RUN_DFC_IN_CACHE
	@ Step 1
	@ lock Data&instruction into cache
@	ldr	r0, =lock_cache_start
@	ldr	r1, =lock_cache_end
@	sub	r1, r1, r0
@	bl	remap_to_uncacheable

	ldr	lr, =dfc_start

	ldr	r0, =dfc_lock_cache_code_base
	ldr	r0, [r0]

	bx	r0

lock_cache_start:
	nop
	nop

	@locking I cache, using direct-mapped I-cache procedure.
	mrc	p15, 0, r3, c9, c0, 1
	orr	r3, r3, #0xF
	mcr	p15, 0, r3, c9, c0, 1

	@locking D cache, using 4 way mapped procedure.
	mrc	p15, 0, r3, c9, c0, 0
	mov	r2, r3
	bic	r3, r3, #0x8
	orr	r3, r3, #0x7
	mcr	p15, 0, r3, c9, c0, 0

	ldr	r0, =dfc_start
	ldr	r1, =dfc_end

	bic	r0, r0, #0x7F
lock_one_cacheline:
	mcr	p15, 0, r0, c7, c5, 1	@I cache line invalidate by MVA.
	mcr	p15, 0, r0, c7, c13, 1	@force a I cacheline line fill.
	mcr	p15, 0, r0, c7, c6, 1	@invalidate D-Cache line by MVA.
	pld	[r0]

	add	r0, r0, #0x20
	cmp	r0, r1
	blo	lock_one_cacheline

	bic	r3, r3, #0xF
	mcr	p15, 0, r3, c9, c0, 1

	mov	r3, r2
	orr	r3, r3, #0x8
	mcr	p15, 0, r3, c9, c0, 0

	mov	pc, lr
#endif



lock_cache_end:
dfc_start:
	b	1f
	.align	5
1:

	@ put some address into TLB
	ldr	r1, =0xFE282800	/* lock APUM register address into TLB */
	mcr	p15, 0, r1, c8, c7, 1
	mrc	p15, 0, r0, c10, c0, 0
	orr	r0, r0, #1
	mcr	p15, 0, r0, c10, c0, 0
	ldr	r1, [r1]
	mrc	p15, 0, r0, c10, c0, 0
	bic	r0, r0, #1
	mcr	p15, 0, r0, c10, c0, 0

	ldr	r1, =0xFE050000	/* lock MPMU register addresses into TLB */
	mcr	p15, 0, r1, c8, c7, 1
	mrc	p15, 0, r0, c10, c0, 0
	orr	r0, r0, #1
	mcr	p15, 0, r0, c10, c0, 0
	ldr	r1, [r1]
	mrc	p15, 0, r0, c10, c0, 0
	bic	r0, r0, #1
	mcr	p15, 0, r0, c10, c0, 0


	ldr	r1, =dmc_membase /* lock MPMU register addresses into TLB */
	ldr	r1, [r1]
	mcr	p15, 0, r1, c8, c7, 1
	mrc	p15, 0, r0, c10, c0, 0
	orr	r0, r0, #1
	mcr	p15, 0, r0, c10, c0, 0
	ldr	r1, [r1]
	mrc	p15, 0, r0, c10, c0, 0
	bic	r0, r0, #1
	mcr	p15, 0, r0, c10, c0, 0

	@ Step 3
	@ put the DDR into self refresh
#ifdef DDR_SR_BY_APMU
	@ put DDR into self-refresh by SW_SLP_TYPE
	ldr r1, =0xFE282800
	mov r0, #0
	str r0, [r1, #0xC0]	/* write SW_SLP_TYPE to 0, self-refresh */
	ldr r0, =0x1
	str r0, [r1, #0xB4]	/* write SLP_REQ to 1 */

sr_ack_check:
	ldr r0, [r1, #0xB4]
	tst r0, #2
	beq sr_ack_check
#endif

#ifdef DDR_SR_BY_DMC
	@ block ddr data request
	ldr	r1, =dmc_membase
	ldr	r1, [r1]	/* DMC base: 0xB000_0000 */
	mov	r0, #1
	str	r0, [r1, #0x07e0]

	@ ddr self refresh
	ldr	r0, [r1, #0x0120]
	orr	r0, #0x40
	str	r0, [r1, #0x0120]
#endif

	@ Step 4
	@ trigger the DFC by apmu_ccr
trigger_freq_chg:
	ldr r0, =0xFE282804	/* APMU_CCR 0xD4282804 */
	str r11, [r0]

	ldr r1, =0xFE2828A0	/* APMU_ISR 0xD42828A0 */

dfc_done_check:
	ldr r0, [r1]
	tst r0, #2		/* PMUA_ISR_FC_ISR */
	beq dfc_done_check

	mov r0, #0		/* clean the ISR */
	str r0, [r1]

	@wait for some cycle
	nop
	nop
	nop
	nop
	nop
	nop
	nop

	@ Step 6
	@ re-initialize the DDR

	@DMC base: 0xB000_0000
	ldr	r1, =dmc_membase
	ldr	r1, [r1]

	@ set PHY_SYNC_EN
	@ resynchronize the new PHY clock
	ldr r0, =0x80000000
	str r0, [r1, #0x240]

#ifdef DDR_SR_BY_DMC
	@get out of self refresh
	ldr r0, =0x80
	str r0, [r1, #0x120]
#endif

	@ set DLL_RESET bit
	ldr r0, =0x40
	str r0, [r1, #0x80]

	@send MRS command
	ldr r0, =0x03000100
	str r0, [r1, #0x120]

	@clean DLL_RESET bit
	ldr r0, =0x0
	str r0, [r1, #0x80]

	@set DLL_RESET_TIMER to 8
	ldr r0, [r1, #0x230]
	and r0, r0, #0x0FFFFFFF
	orr r0, r0, #0x80000000
	str r0, [r1, #0x230]

	@ set PHY_DLL_RESET
	ldr r0, =0x20000000
	str r0, [r1, #0x240]

	@write bit 27 of PHY_CNTRL_14 and wait at least
	ldr r0, =0x08000000
	str r0, [r1, #0x240]

	@512 dlck cycles at least to update DLL.
	ldr     r0, =512
wait_512dclks:
	subs    r0, r0, #2
	bne     wait_512dclks

	@write bit 27 of PHY_CNTRL_14 again to restore,
	@since this bit is a write '1' to toggle bit.
	ldr r0, =0x08000000
	str r0, [r1, #0x240]

#ifdef DDR_SR_BY_APMU
	@ Make DDR out of self-refresh
	@ write 0 to SLP_REQ
	ldr r2, =0xFE282800
	ldr r0, =0x0
	str r0, [r2, #0xB4]
#endif


#ifdef DDR_SR_BY_DMC
	@unblocking the DDR access request
	mov	r0, #0
	str	r0, [r1, #0x07e0]
#endif



	mov	r0, #2560
loop2:
	subs	r0, r0, #1
	bne	loop2



	ldr	r1, =ddr_restart_add
	mov	pc, r1
	.ltorg		/* ensure the data pool here */

ddr_restart_add:
#ifdef RUN_DFC_IN_SRAM
	@ disable L2 cache to SQU mapping
	ldr	r1, =0xfe282c08
	ldr	r2, [r1]
	bic	r2, r2, #0x10
	str	r2, [r1]
	@ disable SQU bank3
	ldr	r1, =0xfe2a0030
	ldr	r2, [r1]
	bic	r2, r2, #0x1
	str	r2, [r1]

	mrc	p15, 0, r0, c1, c0, 0
	orr	r0, r0, #(1 << 26)
	mcr	p15, 0, r0, c1, c0, 0	/* enable L2 cache */

	@ invalidate I, D caches & BTB
	mcr	p15, 0, ip, c7, c7, 0
	@ Drain Write (& Fill) Buffer
	mcr	p15, 0, ip, c7, c10, 4
	@ Prefetch Flush
	mcr	p15, 0, ip, c7, c5, 4
	@ invalidate I, D TLBs
	mcr	p15, 0, ip, c8, c7, 0
	@ invalidate L2 cache
	mcr	p15, 1, ip, c7, c7, 0
#endif

#ifdef RUN_DFC_IN_CACHE
	@unlock DCache way 3
	mrc	p15, 0, r3, c9, c0, 0
	bic	r3, r3, #0x8
	mcr	p15, 0, r3, c9, c0, 0

	@invalidate all the I-Cache and flush BPU
	mcr	p15, 0, r0, c7, c5, 0
#endif


	mov r0, #0
        ldmfd   sp!, {r3 - r12, pc}

dfc_end:
	nop
	nop

