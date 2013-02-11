#
# USB OTG controller driver
#

obj-$(CONFIG_USB_PXA3XX_U2D)	+= pxa310_otg.o
obj-$(CONFIG_USB_PXA3XX_U2D)	+= pxa930_otg.o
obj-$(CONFIG_USB_OTG_PXA3XX)	+= pxa3xx_otg_pmic.o
obj-$(CONFIG_USB_OTG_PXA3XX)	+= pxa3xx_otg.o
obj-$(CONFIG_USB_PXA_U2O)	+= pxa_u2o.o
ifeq ($(CONFIG_USB_PXA3XX_UDC),y)
obj-$(CONFIG_USB_OTG_PXA3XX)	+= pxa300_otg.o
endif
# infrastructure
obj-$(CONFIG_USB_OTG_UTILS)	+= otg.o

# transceiver drivers
obj-$(CONFIG_USB_GPIO_VBUS)	+= gpio_vbus.o
obj-$(CONFIG_ISP1301_OMAP)	+= isp1301_omap.o
obj-$(CONFIG_TWL4030_USB)	+= twl4030-usb.o
obj-$(CONFIG_NOP_USB_XCEIV)	+= nop-usb-xceiv.o
obj-$(CONFIG_USB_ULPI)		+= ulpi.o

ccflags-$(CONFIG_USB_DEBUG)	+= -DDEBUG
ccflags-$(CONFIG_USB_GADGET_DEBUG) += -DDEBUG

