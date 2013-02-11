#ifndef __PLAT_PXA_GENERIC_H_
#define __PLAT_PXA_GENERIC_H_

int is_android(void);
int get_mlc_size(void);
int is_lab(void);

#endif
void android_add_pmem(char *name, size_t size, int no_allocator, int cached);

