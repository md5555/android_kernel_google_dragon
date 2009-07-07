#
# Makefile for the Linux kernel ubuntu supplied third-party device drivers.
#

obj-$(CONFIG_BLK_DEV_DRBD)	+= drbd/
#obj-$(CONFIG_SCSI_ISCSITARGET)	+= iscsitarget/
#obj-$(CONFIG_AUFS)		+= aufs/
obj-$(CONFIG_DM_RAID45)		+= dm-raid4-5/
#obj-$(CONFIG_DM_LOOP)		+= dm-loop/
obj-$(CONFIG_NDISWRAPPER)	+= ndiswrapper/
obj-$(CONFIG_BLK_DEV_COMPCACHE)	+= compcache/
obj-$(CONFIG_LIRC_DEV)          += lirc/
obj-$(CONFIG_TLSUP)		+= tlsup/
#obj-$(CONFIG_LENOVO_SL_LAPTOP)	+= lenovo-sl-laptop/

obj-m				+= misc/ rfkill/

# This is a stupid trick to get kbuild to create ubuntu/built-in.o
obj- += foo.o
