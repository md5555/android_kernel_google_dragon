# Mandatory components
obj-y					+= clock.o
obj-y					+= io.o
obj-y					+= irq.o
obj-y					+= irq_dma.o

# Linux CPU frequency scaling interface
obj-$(CONFIG_CPU_FREQ)			+= cpufreq.o

# SMP support
obj-$(CONFIG_SMP)			+= headsmp.o

# System DMA
obj-$(CONFIG_TEGRA_SYSTEM_DMA)		+= dma.o

# NvOs / NvRm-based kernel implementation
obj-$(CONFIG_MACH_TEGRA_GENERIC)	+= nvddk/
# NvOs / NvRm-based kernel implementation
obj-$(CONFIG_MACH_TEGRA_GENERIC)	+= nvrm/

# Misc drivers
obj-$(CONFIG_TEGRA_ODM_RFKILL)		+= tegra_rfkill_odm.o