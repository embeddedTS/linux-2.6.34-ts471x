/*
 *  linux/arch/arm/plat-pxa/generic.c
 *
 *  Code to PXA processor lines
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mm.h>
#include <linux/proc_fs.h>
#include <linux/version.h>
#include <asm/page.h>
#include <asm/setup.h>

#include <plat/generic.h>

char default_pxa_cmdline[COMMAND_LINE_SIZE] = "unknow";  
static int __init parse_tag_pxa(const struct tag *tag)
{
	strlcpy(default_pxa_cmdline, tag->u.cmdline.cmdline, COMMAND_LINE_SIZE);
	printk(KERN_INFO "pxa cmdline:%s\n", default_pxa_cmdline);
	return 0;
}
__tagtable('pxa', parse_tag_pxa);

static ssize_t pxa_cmdline_read_proc(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	int len = strlen(default_pxa_cmdline);

	sprintf(page, "%s\n", default_pxa_cmdline);
	return len + 1;
}
static void __init create_pxa_cmdline_proc_file(void)
{
	struct proc_dir_entry *pxa_cmdline_proc_file = 
		create_proc_entry("pxa_cmdline", 0644, NULL);

	if (pxa_cmdline_proc_file) 
		pxa_cmdline_proc_file->read_proc = pxa_cmdline_read_proc;
	else 
		printk(KERN_INFO "pxa_cmdline proc file create failed!\n");
}
module_init(create_pxa_cmdline_proc_file);

static int android_project = 0;
static int __init android_setup(char *__unused)
{
	android_project = 1;
	return 1;
}
__setup("android", android_setup);

int is_android(void)
{
	return android_project;
}
EXPORT_SYMBOL(is_android);

static int lab_kernel = 0;
static int __init lab_setup(char *__unused)
{
	lab_kernel = 1;
	return 1;
}
__setup("lab", lab_setup);

int is_lab(void)
{
	return lab_kernel;
}
EXPORT_SYMBOL(is_lab);

static unsigned long long g_mlc_size = 1; /*FIXME: default is 4G ? */
static int __init mlc_size_setup(char *str)
{
	char *endchar;
	if (NULL == str) {
		return 1; /* FIXME*/
	}
	str++; /*skip = */

	/* g_mlc_size = memparse(str, &endchar); */
	g_mlc_size = simple_strtoull(str, &endchar, 0);

	return 1;
}
__setup("pxastorage", mlc_size_setup);

int get_mlc_size(void)
{
	return g_mlc_size;
}
EXPORT_SYMBOL(get_mlc_size);

#ifdef CONFIG_ANDROID_PMEM
#include <linux/dma-mapping.h>
#include <linux/android_pmem.h>
void android_add_pmem(char *name, size_t size, int no_allocator, int cached)
{
	struct platform_device *android_pmem_device;
	struct android_pmem_platform_data *android_pmem_pdata;
	struct page *page;
	unsigned long addr, tmp;
	static int id;
	unsigned long paddr = 0;

	android_pmem_device = kzalloc(sizeof(struct platform_device), GFP_KERNEL);
	if(android_pmem_device == NULL)
		return ;

	android_pmem_pdata = kzalloc(sizeof(struct android_pmem_platform_data), GFP_KERNEL);
	if(android_pmem_pdata == NULL) {
		kfree(android_pmem_device);
		return ;
	}
	
	page = alloc_pages(GFP_KERNEL, get_order(size));
	if (page == NULL)
		return ;

	addr = (unsigned long)page_address(page);
	paddr = virt_to_phys((void *)addr);
	tmp = size;
	#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
	dma_map_single(NULL, (void *)addr, size, DMA_FROM_DEVICE);
	#else
	dma_cache_maint((void *)addr, size, DMA_FROM_DEVICE);
	#endif
	while(tmp > 0) {
		SetPageReserved(virt_to_page(addr));
		addr += PAGE_SIZE;
		tmp -= PAGE_SIZE;
	}
	android_pmem_pdata->name = name;
	android_pmem_pdata->start = paddr;
	android_pmem_pdata->size = size;
	android_pmem_pdata->no_allocator = no_allocator ;
	android_pmem_pdata->cached = cached;

	android_pmem_device->name = "android_pmem";
	android_pmem_device->id = id++;
	android_pmem_device->dev.platform_data = android_pmem_pdata;

	platform_device_register(android_pmem_device);
}
EXPORT_SYMBOL(android_add_pmem);
#endif

