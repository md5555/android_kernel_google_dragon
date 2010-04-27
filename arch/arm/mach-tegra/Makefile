ccflags-y += -DNV_IS_AVP=0
ccflags-y += -DNV_OAL=0
ccflags-y += -DNV_USE_FUSE_CLOCK_ENABLE=0
ifeq ($(CONFIG_MACH_TEGRA_GENERIC_DEBUG),y)
ccflags-y += -DNV_DEBUG=1
else
ccflags-y += -DNV_DEBUG=0
endif

# Mandatory components
#obj-y					+= clock.o
#obj-y					+= io.o
#obj-y					+= irq.o
#obj-y					+= gpio.o
obj-y					+= timer.o
#obj-y					+= tegra_sysmap.o

# Dynamic voltage and frequency scaling support
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)		+= idle-t2.o

# NvOs / NvRm-based kernel implementation
#obj-$(CONFIG_MACH_TEGRA_GENERIC)	+= nvrm/
#obj-$(CONFIG_MACH_TEGRA_GENERIC)	+= nvreftrack/
#obj-$(CONFIG_MACH_TEGRA_GENERIC)	+= nvos/
#obj-$(CONFIG_MACH_TEGRA_GENERIC)	+= nvodm/
#obj-$(CONFIG_MACH_TEGRA_GENERIC)	+= odm_kit/
#obj-$(CONFIG_MACH_TEGRA_GENERIC)	+= nvos_user.o
#obj-$(CONFIG_MACH_TEGRA_GENERIC)	+= nvrm_user.o
#obj-$(CONFIG_MACH_TEGRA_GENERIC)	+= init_common.o
obj-$(CONFIG_MACH_TEGRA_GENERIC)	+= board_nvodm.o
#obj-$(CONFIG_MACH_TEGRA_GENERIC)	+= odm_to_plat.o
