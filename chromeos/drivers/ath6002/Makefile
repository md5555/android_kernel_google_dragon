#######################################################################################
#  AR6K Kernel Module makefile.
#######################################################################################

ATH_SRC_BASE := $(obj)

REV ?= 2
EXTRA_CFLAGS += -I$(ATH_SRC_BASE)/host/include
EXTRA_CFLAGS += -I$(ATH_SRC_BASE)/include
# TBDXXX....handle include/AR6002
EXTRA_CFLAGS += -I$(ATH_SRC_BASE)/include/AR6001
EXTRA_CFLAGS += -I$(ATH_SRC_BASE)/host/wlan/include
EXTRA_CFLAGS += -I$(ATH_SRC_BASE)/host/os/linux/include
EXTRA_CFLAGS += -I$(ATH_SRC_BASE)/host/os/
EXTRA_CFLAGS += -I$(ATH_SRC_BASE)/host/bmi/include
EXTRA_CFLAGS += -I$(ATH_SRC_BASE)/host/hif/sdio/linux_sdio/include
EXTRA_CFLAGS += -DLINUX -DKERNEL_2_6 -DDEBUG -D__KERNEL__ \
                -DTCMD -DSEND_EVENT_TO_APP -DUSER_KEYS \
                -DNO_SYNC_FLUSH #\
                -DMULTIPLE_FRAMES_PER_INTERRUPT -DAR6000REV$(REV) \
                -DBLOCK_TX_PATH_FLAG \

ifeq ($(CONFIG_ATH6002_BUILD_ANDROID),y)
  EXTRA_CFLAGS += -DANDROID_ENV
endif
ifeq ($(CONFIG_ATH6002_FAKE_EEPROM_USED),y)
  EXTRA_CFLAGS += -DFAKE_EEPROM_USED
endif

ifeq ($(CONFIG_ATH6002_HTC_RAW_INT_ENV),y)
  EXTRA_CFLAGS += -DHTC_RAW_INTERFACE
endif

ar6000-objs += host/htc2/AR6000/ar6k.o			\
               host/htc2/AR6000/ar6k_events.o 		\
               host/htc2/htc_send.o       		\
               host/htc2/htc_recv.o       		\
               host/htc2/htc_services.o   		\
               host/htc2/htc.o     		  	\
               host/bmi/src/bmi.o            		\
               host/os/linux/ar6000_drv.o       	\
               host/os/linux/ar6000_raw_if.o		\
               host/os/linux/netbuf.o			\
               host/os/linux/wireless_ext.o		\
               host/os/linux/ioctl.o			\
               host/os/linux/engine.o			\
               host/miscdrv/common_drv.o		\
               host/miscdrv/credit_dist.o		\
               host/wmi/wmi.o                       	\
               host/wlan/src/wlan_node.o            	\
               host/wlan/src/wlan_recv_beacon.o     	\
               host/wlan/src/wlan_utils.o           	\
               host/hif/sdio/linux_sdio/src/hif.o
ifeq ($(CONFIG_ATH6002_BUILD_ANDROID),y)
  ar6000-objs += host/os/linux/eeprom.o
endif

obj-$(CONFIG_ATH6002) += ar6000.o
