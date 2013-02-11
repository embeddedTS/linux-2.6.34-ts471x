/*
 *  pmem.c
 *
 *  Buffer Management Module
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.

 *(C) Copyright 2009 Marvell International Ltd.
 * All Rights Reserved
 */

//#define DEBUG
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/miscdevice.h>

#define PMEM_NUM_MAX	8
static char *pmem_name[PMEM_NUM_MAX];
static unsigned int pmem_size[PMEM_NUM_MAX];
static int pmem_cache[PMEM_NUM_MAX], pmem_no_allocator[PMEM_NUM_MAX];
static int pmem_num;

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Bin Yang <bin.yang@marvell.com>");
MODULE_DESCRIPTION("PMEM Module");
MODULE_PARM_DESC(pmem_name, "pmem device name");
MODULE_PARM_DESC(pmem_size, "pmem memory size");
MODULE_PARM_DESC(pmem_cache, "pmem memory cacheable attribute");
MODULE_PARM_DESC(pmem_no_allocator, "pmem does not allocate");
module_param_array(pmem_name, charp, &pmem_num, 0);
module_param_array(pmem_size, uint, NULL, 0);
module_param_array(pmem_cache, int, NULL, 0);
module_param_array(pmem_no_allocator, int, NULL, 0);
/*usage sample:
 * insmod pmem.ko pmem_name="pmem","pmem_dsp" pmem_size=0x800000,0x500000 pmem_
 * no_allocator=1,0 pmem_cache=0,1
 *
*/

void android_add_pmem(char *name, size_t size, int no_allocator, int cached);

static int __init pmem_init(void)
{
	int i; 
	pr_info("PMEM: create %d pmem devices\n", pmem_num);

	for(i=0; i<pmem_num; i++) {
		pr_info("PMEM: create <%s>, size: 0x%x, no_allocator: %d, cacheable: %d \n",
				pmem_name[i], pmem_size[i], pmem_no_allocator[i], pmem_cache[i]);
		android_add_pmem(pmem_name[i], pmem_size[i], pmem_no_allocator[i], pmem_cache[i]);
	}
	return 0;
}

static void __exit pmem_exit(void)
{
	pr_info("PMEM remove is not supported\n");
}

module_init(pmem_init);
module_exit(pmem_exit);

