# AUFS Makefile for the Linux 2.6.25 and later
# $Id: Makefile,v 1.8 2008/08/04 00:32:21 sfjro Exp $

# the environment variables are not inherited since 2.6.23
ifdef AUFS_EXTRA_CFLAGS
ccflags-y += ${AUFS_EXTRA_CFLAGS}
endif

########################################

ifdef CONFIG_AUFS_RR_SQUASHFS
# cf. squashfs3.2-r2 and sqlzma patch.
ccflags-y += -DSQUASHFS_MAGIC=0x73717368
ccflags-y += -DSQUASHFS_MAGIC_SWAP=0x68737173
ccflags-y += -DSQUASHFS_MAGIC_LZMA=0x71736873
ccflags-y += -DSQUASHFS_MAGIC_LZMA_SWAP=0x73687371
endif

# defined in ${srctree}/fs/fuse/inode.c
ccflags-$(CONFIG_AUFS_WORKAROUND_FUSE) += -DFUSE_SUPER_MAGIC=0x65735546

# defined in ${srctree}/fs/xfs/xfs_sb.h
# tristate
ifdef CONFIG_XFS_FS
ccflags-y += -DXFS_SB_MAGIC=0x58465342
endif

# defined in ${srctree}/mm/shmem.c in linux-2.6.26 and earlier
# tristate
ifdef CONFIG_TMPFS
ifeq ($(shell test ${SUBLEVEL} -le 26 && echo t),t)
ccflags-y += -DTMPFS_MAGIC=0x01021994
endif
endif

# defined in ${srctree}fs/sysfs/mount.c
# bool
ccflags-$(CONFIG_SYSFS) += -DSYSFS_MAGIC=0x62656572

ifndef EXTRAVERSION
EXTRAVERSION = $(shell echo ${KERNELVERSION} | cut -f3- -d. | cut -f2- -d-)
endif
# for -mm tree, support the latest version only
ifneq ($(strip $(shell echo ${EXTRAVERSION} | fgrep -- mm)),)
ccflags-y += -DCONFIG_AUFS_UNIONFS22_PATCH -DCONFIG_AUFS_UNIONFS23_PATCH
endif

-include $(dir $(lastword ${MAKEFILE_LIST}))priv.mk
#$(warning ${ccflags-y})

########################################

obj-$(CONFIG_AUFS) += aufs.o
aufs-y := module.o super.o sbinfo.o branch.o xino.o sysaufs.o opts.o \
	wkq.o vfsub.o dcsub.o \
	cpup.o whout.o plink.o wbr_policy.o \
	dentry.o dinfo.o \
	file.o f_op.o finfo.o \
	dir.o vdir.o \
	inode.o i_op.o i_op_add.o i_op_del.o i_op_ren.o iinfo.o \
	misc.o

aufs-$(CONFIG_SYSFS) += sysfs.o
aufs-$(CONFIG_AUFS_BR_NFS) += br_nfs.o
aufs-$(CONFIG_AUFS_BR_XFS) += br_xfs.o
aufs-$(CONFIG_AUFS_WORKAROUND_FUSE) += br_fuse.o
aufs-$(CONFIG_AUFS_DLGT) += dlgt.o
aufs-$(CONFIG_AUFS_HINOTIFY) += hinotify.o
aufs-$(CONFIG_AUFS_HIN_OR_DLGT) += hin_or_dlgt.o
aufs-$(CONFIG_AUFS_HIN_OR_FUSE) += hin_or_fuse.o
aufs-$(CONFIG_AUFS_EXPORT) += export.o
aufs-$(CONFIG_AUFS_ROBR) += robr.o
# reserved for future use
#aufs-$(CONFIG_AUFS_XATTR) += xattr.o
aufs-$(CONFIG_AUFS_DEBUG) += debug.o
aufs-$(CONFIG_AUFS_MAGIC_SYSRQ) += sysrq.o
