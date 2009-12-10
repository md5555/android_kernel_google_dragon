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

obj-y += nvddk_usbphy.o
obj-y += nvddk_usbphy_ap16.o
obj-y += nvddk_usbphy_ap20.o

