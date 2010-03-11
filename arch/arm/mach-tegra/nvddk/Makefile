ccflags-y += -DNV_IS_AVP=0
ccflags-y += -DNV_OAL=0
ccflags-y += -DNV_USE_FUSE_CLOCK_ENABLE=0
ifeq ($(CONFIG_MACH_TEGRA_GENERIC_DEBUG),y)
ccflags-y += -DNV_DEBUG=1
else
ccflags-y += -DNV_DEBUG=0
endif

ccflags-y += -Iarch/arm/mach-tegra/include/hwinc
ccflags-y += -Iarch/arm/mach-tegra/nvddk/

ifneq ($(filter y,$(CONFIG_USB_TEGRA) $(CONFIG_USB_TEGRA_HCD)),)
obj-y				+= nvddk_usbphy.o
obj-y				+= nvddk_usbphy_ap16.o
obj-y				+= nvddk_usbphy_ap20.o
endif

obj-$(CONFIG_KEYBOARD_TEGRA)	+= nvddk_kbc.o
obj-$(CONFIG_MTD_NAND_TEGRA)	+= nvddk_nand.o

obj-$(CONFIG_MMC_TEGRA_SDIO)	+= nvddk_sdio.o
obj-$(CONFIG_TEGRA_SNOR)	+= nvsnor_controller.o
