dtb-$(CONFIG_ARCH_MEDIATEK) += mt8173-evb.dtb
dtb-$(CONFIG_ARCH_THUNDER) += thunder-88xx.dtb
dtb-$(CONFIG_ARCH_VEXPRESS) += rtsm_ve-aemv8a.dtb foundation-v8.dtb
dtb-$(CONFIG_ARCH_XGENE) += apm-mustang.dtb
dtb-$(CONFIG_ARCH_TEGRA) += tegra132-norrin.dtb
dtb-$(CONFIG_ARCH_TEGRA) += tegra210-foster.dtb

DTB_NAMES := $(subst $\",,$(CONFIG_BUILD_ARM64_APPENDED_DTB_IMAGE_NAMES))
ifneq ($(DTB_NAMES),)
DTB_LIST := $(addsuffix .dtb,$(DTB_NAMES))
else
DTB_LIST := $(dtb-y)
endif

always		:= $(DTB_LIST)
subdir-y	:= $(dts-dirs)
clean-files	:= dts/*.dtb *.dtb
