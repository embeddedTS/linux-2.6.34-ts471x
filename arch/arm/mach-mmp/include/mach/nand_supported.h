#ifndef __MACH_NAND_SUPPORTED_H__
#define	__MACH_NAND_SUPPORTED_H__

static struct pxa3xx_nand_cmdset smallpage_cmdset = {
	.read1          = 0x0000,
	.read2          = 0x0050,
	.program        = 0x1080,
	.read_status    = 0x0070,
	.read_id        = 0x0090,
	.erase          = 0xD060,
	.reset          = 0x00FF,
	.lock           = 0x002A,
	.unlock         = 0x2423,
	.lock_status    = 0x007A,
};

static struct pxa3xx_nand_cmdset largepage_cmdset = {
	.read1          = 0x3000,
	.read2          = 0x0050,
	.program        = 0x1080,
	.read_status    = 0x0070,
	.read_id        = 0x0090,
	.erase          = 0xD060,
	.reset          = 0x00FF,
	.lock           = 0x002A,
	.unlock         = 0x2423,
	.lock_status    = 0x007A,
};

static struct pxa3xx_nand_timing common_timing = {
	.tCH	= 40,
	.tCS	= 80,
	.tWH	= 60,
	.tWP	= 100,
	.tRH	= 80,
	.tRP	= 100,
	.tR	= 90000,
	.tWHR	= 400,
	.tAR	= 40,
};

static struct pxa3xx_nand_timing samsung512MbX8_timing = {
	.tCH	= 10,
	.tCS	= 15,
	.tWH	= 20,
	.tWP	= 40,
	.tRH	= 30,
	.tRP	= 40,
	.tR	= 15000,
	.tWHR	= 110,
	.tAR	= 10,
};

static struct pxa3xx_nand_timing samsung512MbX16_timing = {
	.tCH	= 10,
	.tCS	= 10,
	.tWH	= 20,
	.tWP	= 40,
	.tRH	= 30,
	.tRP	= 40,
	.tR	= 11123,
	.tWHR	= 110,
	.tAR	= 10,
};

static struct pxa3xx_nand_timing samsung2GbX8_timing = {
	.tCH = 10,
	.tCS = 35,
	.tWH = 15,
	.tWP = 25,
	.tRH = 20,
	.tRP = 25,
	.tR = 25000,
	.tWHR = 60,
	.tAR = 10,
};

static struct pxa3xx_nand_timing samsung2GbX8_timing_fast = {
	.tCH    = 5,
	.tCS    = 10,
	.tWH    = 10,
	.tWP    = 0,
	.tRH    = 10,
	.tRP    = 12,
	.tR     = 25000,
	.tWHR   = 60,
	.tAR    = 10,
};

static struct pxa3xx_nand_timing samsung8GbX8_timing = {
	.tADL = 160,
	.tCH = 5,
	.tCS = 20,
	.tWH = 10,
	.tWP = 12,
	.tRH = 10,
	.tRP = 12,
	.tR = 60000,
	.tRHW = 20,
	.tWHR = 80,
	.tAR = 10,
};

static struct pxa3xx_nand_timing samsung8GbX8_timing_fast = {
	.tADL	= 160,
	.tCH    = 5,
	.tCS    = 10,
	.tWH    = 10,
	.tWP    = 0,
	.tRH    = 10,
	.tRP    = 12,
	.tR     = 25000,
	.tRHW	= 20,
	.tWHR	= 80,
	.tAR    = 10,
};

static struct pxa3xx_nand_timing samsung32GbX8_timing = {
	.tADL = 200,
	.tCH = 10,
	.tCS = 15,
	.tWH = 10,
	.tWP = 12,
	.tRH = 10,
	.tRP = 8,
	.tR = 60000,
	.tRHW = 20,
	.tWHR = 75,
	.tAR = 10,
};

static struct pxa3xx_nand_timing micron32GbX8_timing = {
	.tADL = 200,
	.tCH = 10,
        .tCS = 35,
        .tWH = 30,
        .tWP = 50,
        .tRH = 15,
        .tRP = 50,
        .tR = 50000,
	.tRHW = 20,
        .tWHR = 120,
        .tAR = 25,
};

static struct pxa3xx_nand_timing micron_timing = {
	.tADL   = 200,
	.tCH	= 10,
	.tCS	= 25,
	.tWH	= 15,
	.tWP	= 25,
	.tRH	= 15,
	.tRP	= 25,
	.tR	= 25000,
	.tRHW   = 20,
	.tWHR	= 60,
	.tAR	= 10,
};

static struct pxa3xx_nand_timing stm2GbX16_timing = {
	.tADL = 200,
	.tCH = 10,
	.tCS = 35,
	.tWH = 15,
	.tWP = 25,
	.tRH = 15,
	.tRP = 25,
	.tR = 25000,
	.tRHW = 20,
	.tWHR = 60,
	.tAR = 10,
};

static struct pxa3xx_nand_flash hynix4GbX16 = {
	.timing		= &micron_timing,
	.cmdset         = &largepage_cmdset,
	.name		= "Hynix 4Gibx16",
	.page_per_block	= 64,
	.page_size	= 2048,
	.flash_width	= 16,
	.dfc_width	= 16,
	.num_blocks	= 4096,
	.chip_id	= 0xbcad,
	.chip_id_mask   = 0xffff,
	.ecc_type	= ECC_HAMMIN,
	.ecc_strength   = 1,
};

struct pxa3xx_nand_flash nand_common = {
	.timing		= &common_timing,
	.cmdset         = &largepage_cmdset,
	.page_per_block	= 64,
	.page_size	= 2048,
	.flash_width	= 8,
	.dfc_width	= 8,
	.ecc_type	= ECC_HAMMIN,
	.ecc_strength   = 1,
};

static struct pxa3xx_nand_flash samsung512MbX8 = {
	.timing		= &samsung512MbX8_timing,
	.cmdset		= &smallpage_cmdset,
	.name		= "Samsung 512Mibx8",
	.page_per_block	= 32,
	.page_size	= 512,
	.flash_width	= 8,
	.dfc_width	= 8,
	.num_blocks	= 4096,
	.chip_id	= 0x76ec,
	.chip_id_mask   = 0xffff,
	.ecc_type	= ECC_HAMMIN,
	.ecc_strength   = 1,
};

static struct pxa3xx_nand_flash samsung512MbX16 = {
	.timing		= &samsung512MbX16_timing,
	.cmdset		= &smallpage_cmdset,
	.name		= "Samsung 512Mibx16",
	.page_per_block	= 32,
	.page_size	= 512,
	.flash_width	= 16,
	.dfc_width	= 16,
	.num_blocks	= 4096,
	.chip_id	= 0x46ec,
	.chip_id_mask   = 0xffff,
	.ecc_type	= ECC_HAMMIN,
	.ecc_strength   = 1,
};

static struct pxa3xx_nand_flash samsung2GbX8 = {
	.timing		= &samsung2GbX8_timing,
	.cmdset         = &largepage_cmdset,
	.name		= "Samsung 2Gibx8",
	.page_per_block	= 64,
	.page_size	= 2048,
	.flash_width	= 8,
	.dfc_width	= 8,
	.num_blocks	= 2048,
	.chip_id	= 0xdaec,
	.chip_id_mask   = 0xffff,
	.ecc_type	= ECC_HAMMIN,
	.ecc_strength   = 1,
};

static struct pxa3xx_nand_flash samsung8GbX8 = {
	.timing		= &samsung8GbX8_timing,
	.cmdset         = &largepage_cmdset,
	.name		= "Samsung 8Gibx8",
	.page_per_block	= 128,
	.page_size	= 2048,
	.flash_width	= 8,
	.dfc_width	= 8,
	.num_blocks	= 4096,
	.chip_id	= 0xd3ec,
	.chip_id_mask   = 0xffff,
	.ecc_type	= ECC_BCH,
	.ecc_strength   = 1,
};

static struct pxa3xx_nand_flash samsung32GbX8 = {
	.timing		= &samsung32GbX8_timing,
	.cmdset         = &largepage_cmdset,
	.name		= "Samsung 32Gibx8",
	.page_per_block	= 128,
	.page_size	= 4096,
	.flash_width	= 8,
	.dfc_width	= 8,
	.num_blocks	= 8192,
	.chip_id	= 0xd7ec,
	.chip_id_mask   = 0xffff,
	.ecc_type	= ECC_BCH,
	.ecc_strength   = 1,
};

static struct pxa3xx_nand_flash micron1GbX8 = {
	.timing		= &micron_timing,
	.cmdset         = &largepage_cmdset,
	.name		= "Micron 1Gibx8",
	.page_per_block	= 64,
	.page_size	= 2048,
	.flash_width	= 8,
	.dfc_width	= 8,
	.num_blocks	= 1024,
	.chip_id	= 0xa12c,
	.chip_id_mask   = 0xffff,
	.ecc_type	= ECC_HAMMIN,
	.ecc_strength   = 1,
};

static struct pxa3xx_nand_flash micron1GbX16 = {
	.timing		= &micron_timing,
	.cmdset         = &largepage_cmdset,
	.name		= "Micron 1Gibx16",
	.page_per_block	= 64,
	.page_size	= 2048,
	.flash_width	= 16,
	.dfc_width	= 16,
	.num_blocks	= 1024,
	.chip_id	= 0xb12c,
	.chip_id_mask   = 0xffff,
	.ecc_type	= ECC_HAMMIN,
	.ecc_strength   = 1,
};

static struct pxa3xx_nand_flash micron2GbX16 = {
	.timing		= &micron_timing,
	.cmdset         = &largepage_cmdset,
	.name		= "Micron 2Gibx16",
	.page_per_block	= 64,
	.page_size	= 2048,
	.flash_width	= 16,
	.dfc_width	= 16,
	.num_blocks	= 2048,
	.chip_id	= 0xbaec,
	.chip_id_mask   = 0xffff,
	.ecc_type	= ECC_HAMMIN,
	.ecc_strength   = 1,
};

static struct pxa3xx_nand_flash micron4GbX8 = {
	.timing		= &micron_timing,
	.cmdset         = &largepage_cmdset,
	.name		= "Micron 4Gibx8",
	.page_per_block	= 64,
	.page_size	= 2048,
	.flash_width	= 8,
	.dfc_width	= 8,
	.num_blocks	= 4096,
	.chip_id	= 0xdc2c,
	.chip_id_mask   = 0xffff,
	.ecc_type	= ECC_HAMMIN,
	.ecc_strength   = 1,
};

static struct pxa3xx_nand_flash micron2GbX16_ba2c = {
	.timing		= &micron_timing,
	.cmdset         = &largepage_cmdset,
	.name		= "Micron 2Gibx16",
	.page_per_block	= 64,
	.page_size	= 2048,
	.flash_width	= 16,
	.dfc_width	= 16,
	.num_blocks	= 2048,
	.chip_id	= 0xba2c,
	.chip_id_mask   = 0xffff,
	.ecc_type	= ECC_HAMMIN,
	.ecc_strength   = 1,
};

static struct pxa3xx_nand_flash micron32GbX8 = {
        .timing         = &micron32GbX8_timing,
        .cmdset         = &largepage_cmdset,
        .name           = "Micron 32Gibx8",
        .page_per_block = 256,
        .page_size      = 4096,
        .flash_width    = 8,
        .dfc_width      = 8,
        .num_blocks     = 4096,
        .chip_id        = 0x682c,
	.chip_id_mask   = 0xffff,
        .ecc_type       = ECC_BCH,
	.ecc_strength   = 1,
};


static struct pxa3xx_nand_flash stm2GbX16 = {
	.timing 	= &stm2GbX16_timing,
	.cmdset         = &largepage_cmdset,
	.name		= "Stm 2Gibx16",
	.page_per_block = 64,
	.page_size 	= 2048,
	.flash_width 	= 16,
	.dfc_width 	= 16,
	.num_blocks 	= 2048,
	.chip_id 	= 0xba20,
	.chip_id_mask   = 0xffff,
	.ecc_type	= ECC_HAMMIN,
	.ecc_strength   = 1,
};

static struct pxa3xx_nand_flash *builtin_flash_types[] = {
	&nand_common,
	&samsung512MbX8,
	&samsung512MbX16,
	&samsung2GbX8,
	&samsung8GbX8,
	&samsung32GbX8,
	&micron1GbX8,
	&micron4GbX8,
	&micron1GbX16,
	&micron2GbX16,
	&micron2GbX16_ba2c,
	&micron32GbX8,
	&stm2GbX16,
	&hynix4GbX16,
};

#endif
