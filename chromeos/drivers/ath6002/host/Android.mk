#Android makefile to build kernel as a part of Android Build

ifneq ($(TARGET_SIMULATOR),true)

LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)

export  ATH_SRC_BASE=$(LOCAL_PATH)
export  ATH_BUILD_TYPE=QUALCOMM_ARM_NATIVEMMC
export  ATH_BUS_TYPE=sdio
export  ATH_OS_SUB_TYPE=linux_2_6
export  ATH_LINUXPATH=$(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ
export  ATH_ARCH_CPU_TYPE=arm
export  ATH_BUS_SUBTYPE=linux_sdio
export  ATH_ANDROID_ENV=yes
export  ATH_EEPROM_FILE_USED=no
export  ATH_SOFTMAC_USED=no
 
ATH_HIF_TYPE:=sdio
ATH_SRC_BASE:= ../external/athwlan/olca/host

mod_cleanup := $(TARGET_OUT_INTERMEDIATES)/external/athwlan/olca/dummy

$(mod_cleanup) :
	rm $(TARGET_OUT_INTERMEDIATES)/external/athwlan/olca -rf
    
mod_file := $(TARGET_OUT)/wifi/ar6000.ko
$(mod_file) : $(mod_cleanup)
	$(MAKE) ARCH=arm CROSS_COMPILE=arm-eabi- -C $(ATH_LINUXPATH) ATH_HIF_TYPE=$(ATH_HIF_TYPE) SUBDIRS=$(ATH_SRC_BASE)/os/linux modules
	$(ACP) $(TARGET_OUT_INTERMEDIATES)/external/athwlan/olca/host/os/linux/ar6000.ko $(TARGET_OUT)/wifi/

ALL_PREBUILT+=$(mod_file)

include $(CLEAR_VARS)
LOCAL_MODULE := athwlan.bin.z77
LOCAL_MODULE_TAGS := user
LOCAL_MODULE_CLASS := ETC
LOCAL_MODULE_PATH := $(TARGET_OUT)/wifi
LOCAL_SRC_FILES := ../target/AR6002/hw2.0/bin/$(LOCAL_MODULE)
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_MODULE := data.patch.hw2_0.bin
LOCAL_MODULE_TAGS := user
LOCAL_MODULE_CLASS := ETC
LOCAL_MODULE_PATH := $(TARGET_OUT)/wifi
LOCAL_SRC_FILES := ../target/AR6002/hw2.0/bin/$(LOCAL_MODULE)
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_MODULE := eeprom.data
LOCAL_MODULE_TAGS := user
LOCAL_MODULE_CLASS := ETC
LOCAL_MODULE_PATH := $(TARGET_OUT)/wifi
LOCAL_SRC_FILES := ../target/AR6002/hw2.0/bin/$(LOCAL_MODULE)
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_MODULE := eeprom.bin
LOCAL_MODULE_TAGS := user
LOCAL_MODULE_CLASS := ETC
LOCAL_MODULE_PATH := $(TARGET_OUT)/wifi
LOCAL_SRC_FILES := ../target/AR6002/hw2.0/bin/$(LOCAL_MODULE)
include $(BUILD_PREBUILT)

endif
