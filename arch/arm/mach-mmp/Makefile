#
# Makefile for Marvell's PXA168 processors line
#

obj-y				+= common.o clock.o devices.o time.o resource/

# SoC support
obj-$(CONFIG_CPU_PXA168)	+= pxa168.o irq-pxa168.o
obj-$(CONFIG_CPU_PXA910)	+= pxa910.o pxa910-squ.o irq-pxa168.o
obj-$(CONFIG_CPU_MMP2)		+= mmp2.o irq-mmp2.o

# board support
obj-$(CONFIG_MACH_ASPENITE)	+= aspenite.o
obj-$(CONFIG_MACH_ZYLONITE2)	+= aspenite.o
obj-$(CONFIG_MACH_TETON_BGA)    += teton_bga.o
obj-$(CONFIG_MACH_IPCAM)	+= ipcam.o
obj-$(CONFIG_MACH_DKB_GENERIC)	+= dkb_generic.o
obj-$(CONFIG_MACH_AVENGERS_LITE)+= avengers_lite.o
obj-$(CONFIG_MACH_EDGE)		+= edge.o
obj-$(CONFIG_MACH_TTC_DKB)	+= ttc_dkb.o
obj-$(CONFIG_MACH_TAVOREVB)	+= tavorevb.o
obj-$(CONFIG_DVFM)		+= dvfm.o
obj-$(CONFIG_PXA910_DVFM_STATS) += dvfm_stats.o
obj-$(CONFIG_DVFM_PXA168)	+= pxa168_dvfm.o pxa168_dfc_ll.o pxa168_lpm_ll.o
obj-$(CONFIG_MSPM_PXA168_STATS)	+= pxa168_dvfm_stats.o
obj-$(CONFIG_DVFM_PXA910)	+= pxa910_dvfm.o
obj-$(CONFIG_MSPM_PXA168)	+= pxa168_mspm_idle.o pxa168_mspm_prof.o
obj-$(CONFIG_MSPM_PXA910)	+= pxa910_mspm_idle.o pxa910_mspm_prof.o
obj-$(CONFIG_PM_PXA168)		+= pxa168_pm.o pxa168_pm_ll.o
obj-$(CONFIG_PM_PXA910)		+= pxa910_pm.o pxa910_pm_ll.o
obj-$(CONFIG_TIMER_SERVICES_MMP)+= timer_services.o
obj-$(CONFIG_MMC)		+= mmc.o
obj-$(CONFIG_PCI)		+= pxa168_pcie.o
obj-$(CONFIG_MACH_FLINT)	+= flint.o
obj-$(CONFIG_MACH_MARVELL_JASPER) += jasper.o
obj-$(CONFIG_MACH_TS47XX)	+= ts4700.o ts4700_mux_irq.o

