/*
 *  linux/arch/arm/mach-mmp/common.c
 *
 *  Code common to PXA168 processor lines
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/kallsyms.h>
#include <linux/sched.h>
#include <linux/interrupt.h>

#include <asm/page.h>
#include <asm/mach/map.h>
#include <asm/cacheflush.h>
#include <mach/addr-map.h>
#include <mach/regs-pcie.h>

#include <asm/cacheflush.h>
#include "common.h"

static struct map_desc standard_io_desc[] __initdata = {
	{
		.pfn		= __phys_to_pfn(APB_PHYS_BASE),
		.virtual	= APB_VIRT_BASE,
		.length		= APB_PHYS_SIZE,
		.type		= MT_DEVICE,
	}, {
		.pfn		= __phys_to_pfn(AXI_PHYS_BASE),
		.virtual	= AXI_VIRT_BASE,
		.length		= AXI_PHYS_SIZE,
		.type		= MT_DEVICE,
	}, {
		.pfn		= __phys_to_pfn(PXA168_PCIE_PHYS_BASE),
		.virtual	= PXA168_PCIE_VIRT_BASE,
		.length		= PXA168_PCIE_SIZE,
		.type		= MT_DEVICE,
	},
};

void __init pxa_map_io(void)
{
	iotable_init(standard_io_desc, ARRAY_SIZE(standard_io_desc));
}

void release_RIPC(void)
{
	RIPC0_STATUS = 1;
}
void get_RIPC(void)
{
	volatile unsigned long status ;

	status = RIPC0_STATUS;
	while(status!=0) {
		if (!in_atomic() && !irqs_disabled())
			schedule();
		else
			cpu_relax();
		status = RIPC0_STATUS;
	}
}

/*EXPORT_SYMBOL(dmac_flush_range);
EXPORT_SYMBOL(dmac_clean_range);*/
EXPORT_SYMBOL(kallsyms_lookup_name);
static unsigned long uva_to_pa(unsigned long addr, struct page **page)
{
	unsigned long ret = 0UL;
	pgd_t *pgd;
	pud_t *pud;
	pmd_t *pmd;
	pte_t *pte;

	pgd = pgd_offset(current->mm, addr);
	if (!pgd_none(*pgd)) {
		pud = pud_offset(pgd, addr);
		if (!pud_none(*pud)) {
			pmd = pmd_offset(pud, addr);
			if (!pmd_none(*pmd)) {
				pte = pte_offset_map(pmd, addr);
				if (!pte_none(*pte) && pte_present(*pte)) {
					(*page) = pte_page(*pte);
					ret = page_to_phys(*page);
					ret |= (addr & (PAGE_SIZE-1));
				}
			}
		}
	}
	return ret;
}
EXPORT_SYMBOL(uva_to_pa);

unsigned long va_to_pa(unsigned long user_addr, unsigned int size)
{
	unsigned long  paddr, paddr_tmp;
	unsigned long  size_tmp = 0;
	struct page *page = NULL;
	int page_num = PAGE_ALIGN(size) / PAGE_SIZE;
	unsigned int vaddr = PAGE_ALIGN(user_addr);
	int i = 0;

	if (!vaddr)
		return 0;

	paddr = uva_to_pa(vaddr, &page);

	for (i = 0; i < page_num; i++) {
		paddr_tmp = uva_to_pa(vaddr, &page);
		if ((paddr_tmp - paddr) != size_tmp)
			return 0;
		vaddr += PAGE_SIZE;
		size_tmp += PAGE_SIZE;
	}
	return paddr;
}
EXPORT_SYMBOL(va_to_pa);
