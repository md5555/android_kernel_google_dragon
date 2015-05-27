dts-dirs += apm
dts-dirs += arm
dts-dirs += cavium
dts-dirs += mediatek
dts-dirs += tegra

DTB_NAMES := $(subst $\",,$(CONFIG_BUILD_ARM64_APPENDED_DTB_IMAGE_NAMES))
ifneq ($(DTB_NAMES),)
DTB_LIST := $(addsuffix .dtb,$(DTB_NAMES))
else
DTB_LIST := $(dtb-y)
endif

always		:= $(DTB_LIST)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb
