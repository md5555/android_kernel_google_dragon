#
# Tegra thermal specific Makefile
#

# Tegra soc thermal drivers
obj-$(CONFIG_TEGRA_SOCTHERM)	+= tegra_soctherm.o tegra_soctherm_fuse.o
obj-$(CONFIG_TEGRA124_SOCTHERM)	+= tegra124_soctherm.o
obj-$(CONFIG_TEGRA132_SOCTHERM)	+= tegra132_soctherm.o
obj-$(CONFIG_TEGRA210_SOCTHERM)	+= tegra210_soctherm.o
obj-$(CONFIG_TEGRA_THERM_EST)   += tegra_therm_est.o
obj-$(CONFIG_TEGRA_DRAM_THERMAL)	+= tegra_dram_therm.o

# Tegra thermally-reactive drivers
obj-$(CONFIG_TEGRA_DVFS)	+= tegra_core_action.o
obj-$(CONFIG_TEGRA_CPU_EDP)	+= tegra_cpu_edp_action.o
obj-$(CONFIG_TEGRA_DFLL_THERMAL_REACTION)	+= tegra_dfll_action.o
