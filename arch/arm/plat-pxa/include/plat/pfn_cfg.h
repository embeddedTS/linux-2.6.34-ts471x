/*
 *  arch/arm/plat-pxa/include/plat/pfn_cfg.h
 *
 *  Pin Configuration Abstraction
 *
 *  Copyright (C) 2010 Marvell International Ltd.
 *
 *  2010-06-03: Mark F. Brown <markb@marvell.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#ifndef __ASM_ARCH_PFN_CFG_H
#define __ASM_ARCH_PFN_CFG_H

#include <plat/mfp.h>

/**
 * This describes an entry in the pin table
 * entry can be a function (fn) or gpio
 */

struct pfn_cfg {
	mfp_cfg_t fn;
	mfp_cfg_t gpio;
};

/* Table entry helper macro */
#define PFN_CFG(index, fn, gpio)	\
	[index] = { fn, gpio }

/* Describes sentinel values for table searches */
#define PFN_TERM	0xffffffff

/* This describes an undefined table entry */
#define PFN_UNDEF	0x0

enum pfn_type {
	PFN_FN,
	PFN_GPIO,
};

/**
 * pfn_lookup - lookup pin setting from table by index number
 * @table:	table of functions corresponding gpio settings
 * @type:	function or gpio
 * @index:	index to the table
 */
static inline mfp_cfg_t *pfn_lookup(struct pfn_cfg *table, enum pfn_type type,
		int index)
{
	return (type == PFN_FN) ? &table[index].fn : &table[index].gpio;
}

/**
 * pfn_config - set all pin settings in a table to function or gpio
 * @table:	table of functions corresponding gpio settings
 * @type:	function or gpio
 */
static inline void pfn_config(struct pfn_cfg *table, enum pfn_type type)
{
	struct pfn_cfg *cur = table;
	for (; cur->fn != PFN_TERM && cur->gpio != PFN_TERM; cur++) {
		if (cur->fn != PFN_UNDEF && cur->gpio != PFN_UNDEF)
			mfp_config((type == PFN_FN) ? &cur->fn : &cur->gpio, 1);
	}
}

#endif /* _ASM_ARCH_PFN_CFG_H */
