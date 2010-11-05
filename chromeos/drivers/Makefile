#
# Makefile for Chromeos Supplied Third-Party device drivers
#

obj-$(CONFIG_ATH6K_LEGACY)	+= ath6kl/
obj-$(CONFIG_R8103)	+= rtl8103/r8101-1.014.00/src/
obj-$(CONFIG_MWIFIEX)	+= mwifiex/
obj-$(CONFIG_GOBI_QCSERIAL) += gobi/QCSerial2k/
obj-$(CONFIG_GOBI_USBNET)   += gobi/QCUSBNet2k/
obj-$(CONFIG_TUSBD_USB_OVER_IP) += usb-server/modules/src/tusbd2.6.31/

# This is a stupid trick to get kbuild to create chromeos/built-in.o
obj- += foo.o
