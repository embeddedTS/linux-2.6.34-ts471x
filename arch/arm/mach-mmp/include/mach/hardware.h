#ifndef __ASM_MACH_HARDWARE_H
#define __ASM_MACH_HARDWARE_H

#define cpu_is_pxa3xx() 0
#define pcibios_assign_all_busses()	1
#define PCIBIOS_MIN_IO			0x1000
#define PCIBIOS_MIN_MEM			0x01000000

/* The GC300 has the lower phys address on PXA168. We will assume
 * that any phy address below it belongs to PCIe.
 */
#define PXA168_PCIE_MAX_MEM (0xc0400000-1)

#endif /* __ASM_MACH_HARDWARE_H */
