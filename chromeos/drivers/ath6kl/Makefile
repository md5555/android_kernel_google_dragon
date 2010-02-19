#------------------------------------------------------------------------------
# <copyright file="makefile" company="Atheros">
#    Copyright (c) 2005-2010 Atheros Corporation.  All rights reserved.
# 
# $ATH_LICENSE_HOSTSDK0_C$
#------------------------------------------------------------------------------
#==============================================================================
# Author(s): ="Atheros"
#==============================================================================

ccflags-y += -I$(obj)/include
ccflags-y += -I$(obj)/wlan/include
ccflags-y += -I$(obj)/os/linux/include
ccflags-y += -I$(obj)/os
ccflags-y += -I$(obj)/bmi/include

ifeq ($(CONFIG_AR6003_HW10),y)
ccflags-y += -I$(obj)/include/AR6002/hw4.0
ccflags-y += -DAR6003
ccflags-y += -DHW10
endif

ifeq ($(CONFIG_AR6003_HW20),y)
ccflags-y += -I$(obj)/include/AR6002/hw4.0
ccflags-y += -DAR6003
ccflags-y += -DHW20
endif

ifeq ($(CONFIG_AR600x_SD31_XXX),y)
ccflags-y += -DAR600x_SD31_XXX
endif

ifeq ($(CONFIG_AR600x_WB31_XXX),y)
ccflags-y += -DAR600x_WB31_XXX
endif

ifeq ($(CONFIG_AR600x_SD32_XXX),y)
ccflags-y += -DAR600x_SD32_XXX
endif

ifeq ($(CONFIG_AR600x_CUSTOM_XXX),y)
ccflags-y += -DAR600x_CUSTOM_XXX
endif

ifeq ($(CONFIG_ATH6KL_HCI_BRIDGE),y)
ccflags-y += -DATH_AR6K_ENABLE_GMBOX
ccflags-y += -DHCI_TRANSPORT_SDIO
ath6kl-y += htc2/AR6000/ar6k_gmbox.o
ath6kl-y += htc2/AR6000/ar6k_gmbox_hciuart.o
ath6kl-y += miscdrv/ar3kconfig.o
ath6kl-y += miscdrv/ar3kps/ar3kpsconfig.o
ath6kl-y += miscdrv/ar3kps/ar3kpsparser.o
endif

ifeq ($(CONFIG_ATH6KL_CFG80211),y)
ccflags-y += -DCONFIG_CFG80211
ath6kl-y += os/linux/cfg80211.o
endif

ifeq ($(CONFIG_ATH6KL_HTC_RAW_INTERFACE),y)
ccflags-y += -DHTC_RAW_INTERFACE
endif

ifeq ($(CONFIG_ATH6KL_DEBUG),y)
ccflags-y += -DDEBUG
endif

ccflags-y += -DLINUX -DKERNEL_2_6
ccflags-y += -DTCMD
ccflags-y += -DSEND_EVENT_TO_APP
ccflags-y += -DUSER_KEYS
ccflags-y += -DNO_SYNC_FLUSH
ccflags-y += -DHTC_EP_STAT_PROFILING
ccflags-y += -DATH_AR6K_11N_SUPPORT
ccflags-y += -DWAPI_ENABLE
ccflags-y += -DCHECKSUM_OFFLOAD
ccflags-y += -DWLAN_HEADERS
ccflags-y += -DINIT_MODE_DRV_ENABLED

obj-$(CONFIG_ATH6K_LEGACY) := ath6kl.o
ath6kl-y += htc2/AR6000/ar6k.o
ath6kl-y += htc2/AR6000/ar6k_events.o
ath6kl-y += htc2/htc_send.o
ath6kl-y += htc2/htc_recv.o
ath6kl-y += htc2/htc_services.o
ath6kl-y += htc2/htc.o
ath6kl-y += bmi/src/bmi.o
ath6kl-y += os/linux/ar6000_drv.o
ath6kl-y += os/linux/ar6000_raw_if.o
ath6kl-y += os/linux/netbuf.o
ath6kl-y += os/linux/wireless_ext.o
ath6kl-y += os/linux/ioctl.o
ath6kl-y += os/linux/hci_bridge.o
ath6kl-y += miscdrv/common_drv.o
ath6kl-y += miscdrv/credit_dist.o
ath6kl-y += wmi/wmi.o
ath6kl-y += reorder/rcv_aggr.o
ath6kl-y += wlan/src/wlan_node.o
ath6kl-y += wlan/src/wlan_recv_beacon.o
ath6kl-y += wlan/src/wlan_utils.o			 

# ATH_HIF_TYPE := sdio
ccflags-y += -I$(obj)/hif/sdio/linux_sdio/include
ccflags-y += -DSDIO
ath6kl-y += hif/sdio/linux_sdio/src/hif.o
ath6kl-y += hif/sdio/linux_sdio/src/hif_scatter.o
