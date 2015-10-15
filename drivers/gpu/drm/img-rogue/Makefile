img_basedir := drivers/gpu/drm/img-rogue
include $(img_basedir)/config_kernel.mk

obj-$(CONFIG_DRM_POWERVR_ROGUE) += pvrsrvkm.o

ccflags-y += \
 -include config_kernel.h \
 -Iinclude/drm \
 -I$(img_basedir) \
 -I$(img_basedir)/include \
 -I$(img_basedir)/km \
 -D__linux__

ccflags-y += -I$(img_basedir)/mt8173

include $(img_basedir)/pvrsrvkm.mk

