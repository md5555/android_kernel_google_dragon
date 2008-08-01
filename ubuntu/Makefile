#
# Makefile for the Linux kernel ubuntu supplied third-party device drivers.
#

obj-$(CONFIG_BLK_DEV_DRBD)	+= drbd/

obj-$(CONFIG_SCSI_ISCSITARGET)	+= iscsitarget/

obj-$(CONFIG_SQUASHFS)		+= squashfs/

obj-$(CONFIG_AUFS)		+= aufs/

obj-$(CONFIG_ATL2)		+= atl2/

obj-$(CONFIG_NET_ET131X)	+= et131x/

obj-$(CONFIG_DM_RAID45)		+= dm-raid4-5/

obj-$(CONFIG_DM_LOOP)		+= dm-loop/

obj-$(CONFIG_NDISWRAPPER)	+= ndiswrapper/

obj-$(CONFIG_TLSF)		+= compcache/

obj-m				+= misc/

# This is a stupid trick to get kbuild to create ubuntu/built-in.o
obj- += foo.o
