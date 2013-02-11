#ifndef __ASM_ARCH_PXA3XX_NAND_H
#define __ASM_ARCH_PXA3XX_NAND_H

#ifdef   __KERNEL__
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#endif

#define NUM_CHIP_SELECT		2
#define CMD_POOL_SIZE		5

/* error code and state */
enum ecc_type {
	ECC_NONE = 0,
	ECC_HAMMIN,
	ECC_BCH,
};

struct pxa3xx_nand_timing {
	unsigned int		tADL; /* Adress to Write Data delay */
	unsigned int		tCH;  /* Enable signal hold time */
	unsigned int		tCS;  /* Enable signal setup time */
	unsigned int		tWH;  /* ND_nWE high duration */
	unsigned int		tWP;  /* ND_nWE pulse time */
	unsigned int		tRH;  /* ND_nRE high duration */
	unsigned int		tRP;  /* ND_nRE pulse width */
	unsigned int		tR;   /* ND_nWE high to ND_nRE low for read */
	unsigned int		tRHW; /* delay for next command issue */
	unsigned int		tWHR; /* ND_nWE high to ND_nRE low for status read */
	unsigned int		tAR;  /* ND_ALE low to ND_nRE low delay */
};

struct pxa3xx_nand_cmdset {
	uint16_t        	read1;
	uint16_t        	read2;
	uint16_t        	program;
	uint16_t        	read_status;
	uint16_t        	read_id;
	uint16_t        	erase;
	uint16_t        	reset;
	uint16_t        	lock;
	uint16_t       		unlock;
	uint16_t        	lock_status;
};

struct pxa3xx_nand_flash {
	const struct pxa3xx_nand_timing *timing; /* NAND Flash timing */
	const struct pxa3xx_nand_cmdset *cmdset;
	const char name[18];

	uint32_t 		page_per_block;	/* Pages per block (PG_PER_BLK) */
	uint32_t 		page_size;	/* Page size in bytes (PAGE_SZ) */
	uint32_t 		flash_width;	/* Width of Flash memory (DWIDTH_M) */
	uint32_t 		dfc_width;	/* Width of flash controller(DWIDTH_C) */
	uint32_t 		num_blocks;	/* Number of physical blocks in Flash */
	uint32_t 		chip_id;
	uint32_t		chip_id_mask;
	uint32_t		ecc_type;	/* 0 for Hamming, 1 for BCH */
	/*
	 * how many times you want to apply ecc in one page,
	 * Note if you give this value more than 1, the page
	 * would be divided into several chunks to execute
	 */
	uint32_t                ecc_strength;
};

struct pxa3xx_nand_platform_data {

	/* the data flash bus is shared between the Static Memory
	 * Controller and the Data Flash Controller,  the arbiter
	 * controls the ownership of the bus
	 */
	int			enable_arbiter;
	int			use_dma;	/* use DMA ? */
	int			RD_CNT_DEL;

	struct mtd_partition    *parts[NUM_CHIP_SELECT];
	unsigned int            nr_parts[NUM_CHIP_SELECT];
};

extern void pxa3xx_set_nand_info(struct pxa3xx_nand_platform_data *info);
#endif /* __ASM_ARCH_PXA3XX_NAND_H */
