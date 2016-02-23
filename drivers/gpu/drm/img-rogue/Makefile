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

include $(img_basedir)/pvrsrvkm.mk
include $(img_basedir)/mt8173/Makefile

obj-$(CONFIG_DRM_POWERVR_ROGUE_PDUMP) += dbgdrv.o

dbgdrv-y += \
 dbgdriv.o \
 dbgdriv_handle.o \
 ioctl.o \
 hostfunc.o \
 main.o
