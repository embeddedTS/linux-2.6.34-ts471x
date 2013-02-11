#ifndef __ASM_MACH_CPUTYPE_H
#define __ASM_MACH_CPUTYPE_H

#include <asm/io.h>
#include <asm/cputype.h>
#include <mach/addr-map.h>

#define CHIP_ID		(AXI_VIRT_BASE + 0x82c00)

/*
 *  CPU   Stepping   OLD_ID       CPU_ID      CHIP_ID
 *
 * PXA168    A0    0x41159263   0x56158400   0x00A0A333
 * PXA910    Y0    0x41159262   0x56158000   0x00F0C910
 * MMP2	     Z0			0x560f5811
 */

#ifndef CONFIG_CPU_MOHAWK_OLD_ID

#ifdef CONFIG_CPU_PXA168
#  define __cpu_is_pxa168(id, cid)	\
	({ unsigned int _id = ((id) >> 8) & 0xff; \
	 unsigned int _cid = (cid) & 0xfff; \
	 _id == 0x84 && _cid != 0x910; })
#else
#  define __cpu_is_pxa168(id, cid)	(0)
#endif

#ifdef CONFIG_CPU_PXA910
#  define __cpu_is_pxa910(id, cid)	\
	({ unsigned int _id = ((id) >> 8) & 0xff; \
	 unsigned int _cid = (cid) & 0xfff; \
	 (_id == 0x84 || _id == 0x80) && (_cid == 0x910 || _cid == 0x920); })
#else
#  define __cpu_is_pxa910(id, cid)	(0)
#endif

#else

#ifdef CONFIG_CPU_PXA168
#  define __cpu_is_pxa168(id, cid)	\
	({ unsigned int _id = (id) & 0xffff; _id == 0x9263; })
#else
#  define __cpu_is_pxa168(id, cid)	(0)
#endif

#ifdef CONFIG_CPU_PXA910
#  define __cpu_is_pxa910(id, cid)	\
	({ unsigned int _id = (id) & 0xffff; _id == 0x9262; })
#else
#  define __cpu_is_pxa910(id, cid)	(0)
#endif

#endif /* CONFIG_CPU_MOHAWK_OLD_ID */

#ifdef CONFIG_CPU_MMP2
#  define __cpu_is_mmp2(id)	\
	({ unsigned int _id = ((id) >> 8) & 0xff; _id == 0x58; })
#else
#  define __cpu_is_mmp2(id)	(0)
#endif

#define cpu_is_pxa168()		({ __cpu_is_pxa168(read_cpuid_id(), __raw_readl(CHIP_ID)); })
#define cpu_is_pxa910()		({ __cpu_is_pxa910(read_cpuid_id(), __raw_readl(CHIP_ID)); })
#define cpu_is_mmp2()		({ __cpu_is_mmp2(read_cpuid_id()); })

static inline int cpu_is_pxa910_z0(void)
{
	unsigned int chip_id = __raw_readl(CHIP_ID);
	if (cpu_is_pxa910() && ((chip_id & 0x00f00000) == 0x00a00000))
		return 1;
	else
		return 0;
}

static inline int cpu_is_pxa910_Ax(void)
{
	unsigned int chip_id = __raw_readl(CHIP_ID);
	if (cpu_is_pxa910() && ((chip_id & 0x00ff0000) >= 0x00f10000))
		return 1;
	else
		return 0;
}

static inline int cpu_is_pxa168_A0(void)
{
	unsigned int chip_id = __raw_readl(CHIP_ID);
	if (cpu_is_pxa168() && ((chip_id & 0x00f0ffff) == 0x00a0a168))
		return 1;
	else
		return 0;
}

static inline int cpu_is_pxa920_z2(void)
{
	unsigned int chip_id = __raw_readl(CHIP_ID);
	if (cpu_is_pxa910() && ((chip_id & 0x00fff000) == 0x0070c000))
		return 1;
	else
		return 0;
}


#endif /* __ASM_MACH_CPUTYPE_H */
