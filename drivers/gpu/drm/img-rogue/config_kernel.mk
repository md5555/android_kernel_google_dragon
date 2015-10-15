override CACHEFLUSH_TYPE := CACHEFLUSH_GENERIC
override PVRSRV_MODNAME := pvrsrvkm
override PVR_BUILD_DIR := mt8173_linux
override PVR_HANDLE_BACKEND := idr
override PVR_SYSTEM := mt8173
override SUPPORT_BUFFER_SYNC := 1
override SUPPORT_COMPUTE := 1
override SUPPORT_DMABUF := 1
override SUPPORT_DRM := 1
override SUPPORT_ES32 := 1
override SUPPORT_SECURE_EXPORT := 1
override SUPPORT_SERVER_SYNC := 1
override SUPPORT_TLA := 1
ifeq ($(CONFIG_DRM_POWERVR_ROGUE_DEBUG),y)
override BUILD := debug
override PVR_BUILD_TYPE := debug
override SUPPORT_PAGE_FAULT_DEBUG := 1
else
override BUILD := release
override PVR_BUILD_TYPE := release
endif
ifeq ($(CONFIG_DRM_POWERVR_ROGUE_KERNEL_SRVINIT),y)
override RGX_FW_FILENAME := rgx.fw
override SUPPORT_KERNEL_SRVINIT := 1
endif
ifeq ($(CONFIG_DRM_POWERVR_ROGUE_DVFS),y)
override PVR_DVFS := 1
endif
