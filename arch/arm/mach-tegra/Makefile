# Mandatory components
obj-y					+= clock.o
obj-y					+= io.o
obj-y					+= irq.o
obj-y					+= irq_dma.o

# Linux CPU frequency scaling interface
obj-$(CONFIG_CPU_FREQ)			+= cpufreq.o

# SMP
obj-$(CONFIG_SMP)			+= headsmp.o
obj-$(CONFIG_SMP)			+= platsmp.o

# CPU hotplug support
ifeq ($(CONFIG_HOTPLUG_CPU),y)
obj-y					+= hotplug.o
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)		+= power-t2.o
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)		+= power-lp.o
endif

# System DMA
obj-$(CONFIG_TEGRA_SYSTEM_DMA)		+= dma.o

# NvOs / NvRm-based kernel implementation
obj-$(CONFIG_MACH_TEGRA_GENERIC)	+= nvddk/
# NvOs / NvRm-based kernel implementation
obj-$(CONFIG_MACH_TEGRA_GENERIC)	+= nvrm/

# Misc drivers
obj-$(CONFIG_TEGRA_ODM_RFKILL)		+= tegra_rfkill_odm.o
obj-$(CONFIG_TEGRA_ODM_VIBRATE)		+= tegra_vibrate.o

# NvEc
obj-$(CONFIG_TEGRA_NVEC)		+= nvec/
obj-$(CONFIG_TEGRA_NVEC_USER)		+= nvec_user.o
