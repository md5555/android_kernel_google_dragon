ccflags-y += -DNV_IS_AVP=0
ccflags-y += -DNV_OAL=0
ccflags-y += -DNV_USE_FUSE_CLOCK_ENABLE=0
ifeq ($(CONFIG_MACH_TEGRA_GENERIC_DEBUG),y)
ccflags-y += -DNV_DEBUG=1
else
ccflags-y += -DNV_DEBUG=0
endif

# Mandatory components
obj-y					+= clock.o
obj-y					+= io.o
obj-y					+= irq.o
obj-y					+= irq_dma.o
obj-y					+= gpio.o
obj-y					+= timer.o
obj-y					+= tegra_sysmap.o

# IOVMM support
obj-$(CONFIG_TEGRA_IOVMM_GART)		+= iovmm-gart.o
obj-$(CONFIG_TEGRA_IOVMM)		+= iovmm.o

# Tegra suspend operation
obj-$(CONFIG_PM)		+= suspend_ops.o

# Export symbols used by loadable modules
obj-y					+= tegra_exports.o

# Linux CPU frequency scaling interface
obj-$(CONFIG_CPU_FREQ)			+= cpufreq.o

# Dynamic voltage and frequency scaling support
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)		+= idle-t2.o
obj-$(CONFIG_ARCH_TEGRA_1x_SOC)		+= idle-t1.o

# SMP
obj-$(CONFIG_SMP)			+= headsmp.o
obj-$(CONFIG_SMP)			+= platsmp.o

obj-$(CONFIG_LOCAL_TIMERS)		+= localtimer.o

# CPU hotplug support
ifeq ($(CONFIG_HOTPLUG_CPU),y)
obj-y					+= hotplug.o
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)		+= power-t2.o
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)		+= power-context-t2.o
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)		+= power-lp.o
endif

# System DMA
obj-$(CONFIG_TEGRA_SYSTEM_DMA)		+= dma.o

# NvOs / NvRm-based kernel implementation
obj-$(CONFIG_MACH_TEGRA_GENERIC)	+= nvddk/
# NvOs / NvRm-based kernel implementation
obj-$(CONFIG_MACH_TEGRA_GENERIC)	+= nvrm/
obj-$(CONFIG_MACH_TEGRA_GENERIC)	+= nvreftrack/
obj-$(CONFIG_MACH_TEGRA_GENERIC)	+= nvos/
obj-$(CONFIG_MACH_TEGRA_GENERIC)	+= nvodm/
obj-$(CONFIG_MACH_TEGRA_GENERIC)	+= odm_kit/
obj-$(CONFIG_MACH_TEGRA_GENERIC)	+= nvos_user.o
obj-$(CONFIG_MACH_TEGRA_GENERIC)	+= nvrm_user.o
obj-$(CONFIG_MACH_TEGRA_GENERIC)	+= init_common.o
obj-$(CONFIG_MACH_TEGRA_GENERIC)	+= board_nvodm.o

# Misc drivers
obj-$(CONFIG_TEGRA_ODM_RFKILL)		+= tegra_rfkill_odm.o
obj-$(CONFIG_TEGRA_ODM_VIBRATE)		+= tegra_vibrate.o

# NvEc
obj-$(CONFIG_TEGRA_NVEC)			+= nvec/
obj-$(CONFIG_TEGRA_NVEC_USER)		+= nvec_user.o

# PCIe support
obj-$(CONFIG_TEGRA_PCI)				+= pci.o
obj-$(CONFIG_TEGRA_PCI)				+= pci-enum.o

