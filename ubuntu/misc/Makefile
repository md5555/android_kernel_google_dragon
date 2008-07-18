#
# Makefile for Ubuntu additional drivers
#

obj-$(CONFIG_USB_APPLEIR)	+= appleir.o
obj-$(CONFIG_BLK_DEV_DM_BBR)	+= dm-bbr.o
obj-$(CONFIG_INPUT_ACERHK)	+= acerhk.o
obj-$(CONFIG_LMPCM_USB)		+= lmpcm_usb.o
obj-$(CONFIG_TP_SMAPI)		+= tp_smapi.o
obj-$(CONFIG_TP_SMAPI_EC)	+= thinkpad_ec.o
obj-$(CONFIG_FSAM7400)		+= fsam7400.o

obj-m				+= media/ wireless/

CFLAGS_dm-bbr.o			+= -I$(srctree)/drivers/md
