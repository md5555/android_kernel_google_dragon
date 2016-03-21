dtb-$(CONFIG_ARCH_TEGRA) += tegra132-norrin.dtb
dtb-$(CONFIG_ARCH_TEGRA) += tegra210-foster.dtb
dtb-$(CONFIG_ARCH_TEGRA) += tegra210-smaug-p0.dtb
dtb-$(CONFIG_ARCH_TEGRA) += tegra210-smaug-p1.dtb
dtb-$(CONFIG_ARCH_TEGRA) += tegra210-smaug-p2.dtb

DTS_CPP_FLAGS :=
ifdef CONFIG_KASAN
DTS_CPP_FLAGS += -DCONFIG_KASAN=1
endif
CPP += $(DTS_CPP_FLAGS)

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb
