#ifndef	__PLAT_PART_TABLE_H
#define	__PLAT_PART_TABLE_H

#define DECLARE_LAB_PARTITIONS(partitions) \
	static struct mtd_partition partitions[] = { \
		[0] = { \
			.name        = "lab", \
			.offset      = 0, \
			.size        = MTDPART_SIZ_FULL,     /* massstorage*/ \
		}, \
	}

#define DECLARE_ANDROID_128M_V75_PARTITIONS(partitions) \
	static struct mtd_partition partitions[] = { \
		[0] = { \
			.name        = "Bootloader", \
			.offset      = 0, \
			.size        = 0x100000, \
			.mask_flags  = MTD_WRITEABLE, /* force read-only */ \
		}, \
		[1] = { \
			.name        = "NVM", \
			.offset      = 0x100000, \
			.size        = 0x020000, \
		}, \
		[2] = { \
			.name        = "Arbel and Greyback Image", \
			.offset      = 0x120000,  \
			.size        = 0x800000, \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		},	 \
		[3] = { \
			.name        = "Kernel", \
			.offset      = 0x920000, \
			.size        = 0x300000, \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		}, \
		[4] = { \
			.name        = "system", \
			.offset      = 0x0c20000, \
			.size        = 0x4000000,     /* mount 64M fs */ \
		}, \
		[5] = { \
			.name        = "userdata", \
			.offset      = 0x4c20000, \
			.size        = 0x20E0000,     /* mount 32.875M */ \
		}, \
		[6] = { \
			.name        = "filesystem", \
			.offset      = 0x6d00000, \
			.size        = 0x1000000,     /* mount 16M fs */ \
		}, \
		[7] = { \
			.name        = "BBT", \
			.offset      = 0x7d00000, \
			.size        = 0x0080000, \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		}, \
		[8] = { \
			.name        = "reserved_for_bbm", \
			.offset      = 0x7d80000, \
			.size        = MTDPART_SIZ_FULL,      \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		}, \
	/* NOTES: We reserve some blocks for PXA3xx BBM at the end of NAND.*/ \
		/* And the max relocation blocks is not same on different platform. */\
		/* Please take care it when define the partition table.*/ \
	}

#define DECLARE_128M_V75_PARTITIONS(partitions) \
	static struct mtd_partition partitions[] = { \
		[0] = { \
			.name        = "Bootloader", \
			.offset      = 0, \
			.size        = 0x100000, \
			.mask_flags  = MTD_WRITEABLE, /* force read-only */ \
		}, \
		[1] = { \
			.name        = "NVM", \
			.offset      = 0x100000, \
			.size        = 0x020000, \
		}, \
		[2] = { \
			.name        = "Arbel and Greyback Image", \
			.offset      = 0x120000,  \
			.size        = 0x800000, \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		},	 \
		[3] = { \
			.name        = "Kernel", \
			.offset      = 0x920000, \
			.size        = 0x300000, \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		}, \
		[4] = { \
			.name        = "Filesystem", \
			.offset      = 0x0c20000, \
			.size        = 0x3000000,     /* only mount 48M fs */ \
		}, \
		[5] = { \
			.name        = "MassStorage", \
			.offset      = 0x3c20000, \
			.size        = 0x40e0000,		/* 64.875M */ \
		}, \
		[6] = { \
			.name        = "BBT", \
			.offset      = 0x7d00000, \
			.size        = 0x0080000, \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		}, \
		[7] = { \
			.name        = "reserved_for_bbm", \
			.offset      = 0x7d80000, \
			.size        = MTDPART_SIZ_FULL,      \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		}, \
		/* NOTES: We reserve some blocks for PXA3xx BBM at the end of NAND.*/ \
		/* And the max relocation blocks is not same on different platform. */ \
		/* Please take care it when define the partition table.*/ \
	}


#define DECLARE_64M_V75_PARTITIONS(partitions) \
	static struct mtd_partition partitions[] = { \
		[0] = { \
			.name	     = "Bootloader", \
			.offset	     = 0, \
			.size	     = 0x4000, \
			.mask_flags  = MTD_WRITEABLE, /* force read-only */ \
		}, \
		[1] = { \
			.name	     = "Spare", \
			.offset	     = 0x4000, \
			.size	     = 0x4000, \
		}, \
		[2] = { \
			.name	     = "Spare", \
			.offset	     = 0x8000,	\
			.size	     = 0x400000, \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		},	 \
		[3] = { \
			.name	     = "Kernel", \
			.offset	     = 0x100000, \
			.size	     = 0x400000, \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		}, \
		[4] = { \
			.name	     = "Filesystem", \
			.offset	     = 0x0500000, \
			.size	     = 0x3000000,     /* only mount 48M fs */ \
		}, \
		[5] = { \
			.name	     = "MassStorage", \
			.offset	     = 0x3500000, \
			.size	     = 0x9c0000,		/* 9 Mb */ \
		}, \
		[6] = { \
			.name	     = "BBT", \
			.offset	     = 0x3ec0000, \
			.size	     = MTDPART_SIZ_FULL, \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		}, \
		/* NOTES: We reserve some blocks for BBM at the end of NAND.*/ \
		/* And the max relocation blocks changes per platform. */ \
		/* Please take care it when define the partition table.*/ \
	}


#define DECLARE_512M_V75_PARTITIONS(partitions) \
static struct mtd_partition partitions[] = { \
		[0] = { \
			.name        = "Bootloader", \
			.offset      = 0x00040000, \
			.size        = 0x000b0000, \
			.mask_flags  = MTD_WRITEABLE, /* force read-only */ \
		}, \
		[1] = { \
			.name        = "Reserve", \
			.offset      = 0x00100000, \
			.size        = 0x00020000, \
		}, \
		[2] = { \
			.name        = "Reserve", \
			.offset      = 0x00120000, \
			.size        = 0x00800000, \
		}, \
		[3] = { \
			.name        = "Kernel", \
			.offset      = 0x00920000, \
			.size        = 0x00300000, \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		}, \
		[4] = { \
			.name        = "Filesystem", \
			.offset      = 0x00c20000, \
			.size        = 0x10000000, /* only mount 256M fs */ \
		}, \
		[5] = { \
			.name        = "MassStorage", \
			.offset      = 0x10c20000, \
			.size        = 0x0e920000,	/* 233M */ \
		}, \
		[6] = { \
			.name        = "BBT", \
			.offset      = 0x1f540000, \
			.size        = 0x00080000, \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		}, \
		[7] = { \
			.name        = "reserved_for_bbm", \
			.offset      = 0x1f5c0000, \
			.size        = MTDPART_SIZ_FULL,      \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		}, \
		/* NOTES: We reserve some blocks for PXA3xx */ \
		/*   BBM at the end of NAND.*/ \
		/* And the max relocation blocks is not */ \
		/*   same on different platform. */ \
		/* Please take care it when define the partition table.*/ \
	}



#define DECLARE_ANDROID_512M_V75_PARTITIONS(partitions) \
static struct mtd_partition partitions[] = { \
		[0] = { \
			.name        = "Bootloader", \
			.offset      = 0x00040000, \
			.size        = 0x000b0000, \
			.mask_flags  = MTD_WRITEABLE, /* force read-only */ \
		}, \
		[1] = { \
			.name        = "Reserve", \
			.offset      = 0x00100000, \
			.size        = 0x00020000, \
		}, \
		[2] = { \
			.name        = "Reserve", \
			.offset      = 0x00120000, \
			.size        = 0x00800000, \
		}, \
		[3] = { \
			.name        = "Kernel", \
			.offset      = 0x00920000, \
			.size        = 0x00300000, \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		}, \
		[4] = { \
			.name        = "system", \
			.offset      = 0x00c20000, \
			.size        = 0x10000000, /* only mount 256M fs */ \
		}, \
		[5] = { \
			.name        = "userdata", \
			.offset      = 0x10c20000, \
			.size        = 0x0e920000,	/* 233.125M */ \
		}, \
		[6] = { \
			.name        = "BBT", \
			.offset      = 0x1f540000,     /* 81 Blocks reserved*/ \
			.size        = 0x00080000, \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		}, \
		[7] = { \
			.name        = "reserved_for_bbm", \
			.offset      = 0x1f5c0000, \
			.size        = MTDPART_SIZ_FULL,      \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		}, \
	}

#define DECLARE_ANDROID_256M_V75_PARTITIONS(partitions) \
	static struct mtd_partition partitions[] = { \
		[0] = { \
			.name        = "init", \
			.offset      = 0xc0000, \
			.size        = 0x40000, \
			.mask_flags  = MTD_WRITEABLE, /* force read-only */ \
		}, \
		[1] = { \
			.name        = "NVM", \
			.offset      = 0x100000, \
			.size        = 0x020000, \
		}, \
		[2] = { \
			.name        = "Arbel and Greyback Image", \
			.offset      = 0x120000,  \
			.size        = 0x800000, \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		},	 \
		[3] = { \
			.name        = "Kernel", \
			.offset      = 0x920000, \
			.size        = 0x300000, \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		}, \
		[4] = { \
			.name        = "system", \
			.offset      = 0x0c20000, \
			.size        = 0x7000000,     /* mount 112M fs */ \
		}, \
		[5] = { \
			.name        = "userdata", \
			.offset      = 0x7c20000, \
			.size        = 0x7000000,     /* mount 112M */ \
		}, \
		[6] = { \
			.name        = "filesystem", \
			.offset      = 0xec20000, \
			.size        = 0xEE0000,     /* mount 14.875M fs */ \
		}, \
		[7] = { \
			.name        = "reserved_for_bbm", \
			.offset      = 0xfb00000, \
			.size        = MTDPART_SIZ_FULL,      \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		}, \
	}

#define DECLARE_256M_V75_PARTITIONS(partitions) \
	static struct mtd_partition partitions[] = { \
		[0] = { \
			.name        = "Bootloader", \
			.offset      = 0, \
			.size        = 0x100000, \
			.mask_flags  = MTD_WRITEABLE, /* force read-only */ \
		}, \
		[1] = { \
			.name        = "NVM", \
			.offset      = 0x100000, \
			.size        = 0x020000, \
		}, \
		[2] = { \
			.name        = "Arbel and Greyback Image", \
			.offset      = 0x120000,  \
			.size        = 0x800000, \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		},	 \
		[3] = { \
			.name        = "Kernel", \
			.offset      = 0x920000, \
			.size        = 0x300000, \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		}, \
		[4] = { \
			.name        = "Filesystem", \
			.offset      = 0x0c20000, \
			.size        = 0x3000000,     /* only mount 48M fs */ \
		}, \
		[5] = { \
			.name        = "MassStorage", \
			.offset      = 0x3c20000, \
			.size        = 0x40e0000,		/* 64.875M */ \
		}, \
		[6] = { \
			.name        = "BBT", \
			.offset      = 0x7d00000, \
			.size        = 0x0080000, \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		}, \
		[7] = { \
			.name        = "SWAP", \
			.offset      = 0x8000000, \
			.size        = 0x6000000,   \
		}, \
		[8] = { \
			.name        = "reserved_for_bbm", \
			.offset      = 0xe000000, \
			.size        = MTDPART_SIZ_FULL,      \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		}, \
	}


/* 32G partition table is for 32Gbit parts */

#define DECLARE_32G_V75_PARTITIONS(partitions) \
static struct mtd_partition partitions[] = { \
		[0] = { \
			.name	     = "Bootloader", \
			.offset	     = 0x00100000, \
			.size	     = 0x00100000, \
			.mask_flags  = MTD_WRITEABLE, /* force read-only */ \
		}, \
		[1] = { \
			.name	     = "Reserve", \
			.offset	     = 0x00200000, \
			.size	     = 0x00100000, \
		}, \
		[2] = { \
			.name	     = "Reserve", \
			.offset	     = 0x00300000, \
			.size	     = 0x00700000, \
		}, \
		[3] = { \
			.name	     = "Kernel", \
			.offset	     = 0x00a00000, \
			.size	     = 0x00400000, \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		}, \
		[4] = { \
			.name	     = "Filesystem", \
			.offset	     = 0x00e00000, \
			.size	     = 0x10000000, /* only mount 256M fs */ \
		}, \
		[5] = { \
			.name	     = "MassStorage", \
			.offset	     = 0x10e00000, \
			.size	     = 0xe9200000,	/* 3.911G */ \
		}, \
		[6] = { \
			.name	     = "BBT", \
			.offset	     = 0xfa000000,	 /* 80 blocks  */ \
			.size	     = MTDPART_SIZ_FULL, \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		}, \
		/* NOTES: We reserve some blocks for			\
		 *  PXA3xx BBM at the end of NAND.			\
		 * And the max relocation blocks is not same		\
		 * on different platform.				\
		 * Please take care it when define the partition table.*/\
	}

#define DECLARE_ANDROID_32G_V75_PARTITIONS(partitions) \
static struct mtd_partition partitions[] = { \
		[0] = { \
			.name	     = "Bootloader", \
			.offset	     = 0x00100000, \
			.size	     = 0x00100000, \
			.mask_flags  = MTD_WRITEABLE, /* force read-only */ \
		}, \
		[1] = { \
			.name	     = "Reserve", \
			.offset	     = 0x00100000, \
			.size	     = 0x00100000, \
		}, \
		[2] = { \
			.name	     = "Reserve", \
			.offset	     = 0x00200000, \
			.size	     = 0x00700000, \
		}, \
		[3] = { \
			.name	     = "Kernel", \
			.offset	     = 0x00a00000, \
			.size	     = 0x00400000, \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		}, \
		[4] = { \
			.name	     = "system", \
			.offset	     = 0x00e00000, \
			.size	     = 0x10000000, /* only mount 256M fs */ \
		}, \
		[5] = { \
			.name	     = "userdata", \
			.offset	     = 0x10e00000, \
			.size	     = 0xe9200000,	/* 3.911G */ \
		}, \
		[6] = { \
			.name	     = "BBT",	\
			.offset	     = 0xf1000000,	\
			.size	     = MTDPART_SIZ_FULL,		   \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		}, \
		/* Please take care it when define the partition table.*/ \
}




/* 8G partition table is for 8Gbit parts */

#define DECLARE_8G_V75_PARTITIONS(partitions) \
static struct mtd_partition partitions[] = { \
		[0] = { \
			.name	     = "Bootloader", \
			.offset	     = 0x00100000, \
			.size	     = 0x00100000, \
			.mask_flags  = MTD_WRITEABLE, /* force read-only */ \
		}, \
		[1] = { \
			.name	     = "Reserve", \
			.offset	     = 0x00200000, \
			.size	     = 0x00100000, \
		}, \
		[2] = { \
			.name	     = "Reserve", \
			.offset	     = 0x00300000, \
			.size	     = 0x00700000, \
		}, \
		[3] = { \
			.name	     = "Kernel", \
			.offset	     = 0x00a00000, \
			.size	     = 0x00400000, \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		}, \
		[4] = { \
			.name	     = "Filesystem", \
			.offset	     = 0x00e00000, \
			.size	     = 0x10000000, /* only mount 256M fs */ \
		}, \
		[5] = { \
			.name	     = "MassStorage", \
			.offset	     = 0x10e00000, \
			.size	     = 0x29200000,	/* 658M */ \
		}, \
		[6] = { \
			.name	     = "BBT", \
			.offset	     = 0x3a000000,	 /* 80 blocks */ \
			.size	     = MTDPART_SIZ_FULL, \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		}, \
		/* NOTES: We reserve some blocks for			\
 *		    *  PXA3xx BBM at the end of NAND.			   \
 *		    * And the max relocation blocks is not same		   \
 *		    * on different platform.				   \
 *		    * Please take care it when define the partition table.*/\
}

#define DECLARE_ANDROID_8G_V75_PARTITIONS(partitions) \
static struct mtd_partition partitions[] = { \
		[0] = { \
			.name	     = "Bootloader", \
			.offset	     = 0x00100000, \
			.size	     = 0x00100000, \
			.mask_flags  = MTD_WRITEABLE, /* force read-only */ \
		}, \
		[1] = { \
			.name	     = "Reserve", \
			.offset	     = 0x00100000, \
			.size	     = 0x00100000, \
		}, \
		[2] = { \
			.name	     = "Reserve", \
			.offset	     = 0x00200000, \
			.size	     = 0x00700000, \
		}, \
		[3] = { \
			.name	     = "Kernel", \
			.offset	     = 0x00a00000, \
			.size	     = 0x00400000, \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		}, \
		[4] = { \
			.name	     = "system", \
			.offset	     = 0x00e00000, \
			.size	     = 0x10000000, /* only mount 256M fs */ \
		}, \
		[5] = { \
			.name	     = "userdata", \
			.offset	     = 0x10e00000, \
			.size	     = 0x29200000,	/* 3.911G */ \
		}, \
		[6] = { \
			.name	     = "BBT",	\
			.offset	     = 0x3a000000,	\
			.size	     = MTDPART_SIZ_FULL,		   \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		}, \
		/* Please take care it when define the partition table.*/ \
}




#define DECLARE_AVENGERSLITE_SLC_PARTITIONS(partitions)     \
	static struct mtd_partition partitions[] = { \
		[0] = { \
			.name        = "Bootloader", \
			.offset      = 0, \
			.size        = 0x100000, \
		}, \
		[1] = { \
			.name        = "MassStorage0", \
			.offset      = 0x100000, \
			.size        = MTDPART_SIZ_FULL, /*use rest of flash*/ \
		}, \
	}

#define DECLARE_AVENGERSLITE_MLC_4G_ANDROID_PARTITIONS(partitions) \
	static struct mtd_partition partitions[] = { \
		[0] = { \
			.name        = "resereved", \
			.offset      = 0, \
			.size        = 0xA00000,     /* res for relo table */ \
			.mask_flags  = MTD_WRITEABLE, /* force read-only */ \
		}, \
		[1] = { \
			.name        = "Kernel",        \
			.offset      = 0xA00000,         \
			.size        = 0x380000,        \
			.mask_flags  = MTD_WRITEABLE, /* force read-only */ \
		}, \
		[2] = { \
			.name        = "Kernel_recovery", \
			.offset      = 0xD80000, \
			.size        = 0x400000,     /* massstorage*/ \
			.mask_flags  = MTD_WRITEABLE, /* force read-only */ \
		}, \
		[3] = { \
			.name        = "System", \
			.offset      = 0x1180000, \
			.size        = 0xEE80000,     /* massstorage*/ \
		}, \
		[4] = { \
			.name        = "Userdata", \
			.offset      = 0x10000000, \
			.size        = 0x70000000,     /* massstorage*/ \
		}, \
		[5] = { \
			.name        = "MassStorage1", \
			.offset      = 0x80000000, \
			.size        = MTDPART_SIZ_FULL, /*use rest of flash*/ \
		}, \
	}

#define DECLARE_AVENGERSLITE_MLC_1G_ANDROID_PARTITIONS(partitions) \
	static struct mtd_partition partitions[] = { \
		[0] = { \
			.name        = "resereved", \
			.offset      = 0, \
			.size        = 0xA00000,     /* res for relo table */ \
			.mask_flags  = MTD_WRITEABLE, /* force read-only */ \
		}, \
		[1] = { \
			.name        = "Kernel",        \
			.offset      = 0xA00000,         \
			.size        = 0x380000,        \
			.mask_flags  = MTD_WRITEABLE, /* force read-only */ \
		}, \
		[2] = { \
			.name        = "Kernel_recovery", \
			.offset      = 0xD80000, \
			.size        = 0x400000,     /* massstorage*/ \
			.mask_flags  = MTD_WRITEABLE, /* force read-only */ \
		}, \
		[3] = { \
			.name        = "System", \
			.offset      = 0x1180000, \
			.size        = 0x7680000,     /* massstorage*/ \
		}, \
		[4] = { \
			.name        = "Userdata", \
			.offset      = 0x8800000, \
			.size        = 0x22c00000,     /* massstorage*/ \
		}, \
		[5] = { \
			.name        = "MassStorage1", \
			.offset      = 0x2B400000, \
			.size        = MTDPART_SIZ_FULL, /*use rest of flash*/ \
		}, \
	}

#define DECLARE_AVENGERSLITE_MLC_4G_MAEMO_PARTITIONS(partitions) \
	static struct mtd_partition partitions[] = { \
		[0] = { \
			.name        = "resereved", \
			.offset      = 0, \
			.size        = 0xA00000,     /* res for relo table */ \
			.mask_flags  = MTD_WRITEABLE, /* force read-only */ \
		}, \
		[1] = { \
			.name        = "Kernel",        \
			.offset      = 0xA00000,         \
			.size        = 0x380000,        \
			.mask_flags  = MTD_WRITEABLE, /* force read-only */ \
		}, \
		[2] = { \
			.name        = "Kernel_recovery", \
			.offset      = 0xD80000, \
			.size        = 0x400000,     /* massstorage*/ \
			.mask_flags  = MTD_WRITEABLE, /* force read-only */ \
		}, \
		[3] = { \
			.name        = "Filesystem", \
			.offset      = 0x1180000, \
			.size        = 0x3EE80000,     /* massstorage*/ \
		}, \
		[4] = { \
			.name        = "Userdata", \
			.offset      = 0x40000000, \
			.size        = 0x40000000,     /* massstorage*/ \
		}, \
		[5] = { \
			.name        = "MassStorage1", \
			.offset      = 0x80000000, \
			.size        = MTDPART_SIZ_FULL,      \
		}, \
	}

#define DECLARE_AVENGERSLITE_MLC_1G_MAEMO_PARTITIONS(partitions) \
	static struct mtd_partition partitions[] = { \
		[0] = { \
			.name        = "resereved", \
			.offset      = 0, \
			.size        = 0xA00000,     /* res for relo table */ \
			.mask_flags  = MTD_WRITEABLE, /* force read-only */ \
		}, \
		[1] = { \
			.name        = "Kernel",        \
			.offset      = 0xA00000,         \
			.size        = 0x380000,        \
			.mask_flags  = MTD_WRITEABLE, /* force read-only */ \
		}, \
		[2] = { \
			.name        = "Kernel_recovery", \
			.offset      = 0xD80000, \
			.size        = 0x400000,     /* massstorage*/ \
			.mask_flags  = MTD_WRITEABLE, /* force read-only */ \
		}, \
		[3] = { \
			.name        = "Filesystem", \
			.offset      = 0x1180000, \
			.size        = 0x1AE80000,     /* massstorage*/ \
		}, \
		[4] = { \
			.name        = "Userdata", \
			.offset      = 0x1c000000, \
			.size        = MTDPART_SIZ_FULL,     /* massstorage*/ \
		},\
	}

#define DECLARE_SPI_PARTITIONS(partitions) \
	static struct mtd_partition partitions[] = { \
		[0] = { \
			.name        = "Bootloader", \
			.offset      = 0, \
			.size        = 0x100000, \
		}, \
		[1] = { \
			.name        = "MassStorage0", \
			.offset      = 0x100000, \
			.size        = MTDPART_SIZ_FULL,  /* rest of flash*/ \
		}, \
	}

#endif /*__PLAT_PART_TABLE_H*/

