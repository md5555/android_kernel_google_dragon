UNIONFS_VERSION  = 1.4
SUP_MAJOR= 2
SUP_MINOR= 6
SUP_PATCH= 18

EXTRA_CFLAGS+=-DUNIONFS_VERSION=\"${UNIONFS_VERSION}\" -DSUP_MAJOR=${SUP_MAJOR} -DSUP_MINOR=${SUP_MINOR} -DSUP_PATCH=${SUP_PATCH} -DUNIONFS_UNSUPPORTED

# This will enable full debugging support
# EXTRA_CFLAGS+=-DUNIONFS_DEBUG

obj-$(CONFIG_FS_UNIONFS) += unionfs.o

unionfs-objs := subr.o dentry.o file.o inode.o main.o super.o \
	stale_inode.o branchman.o xattr.o rdstate.o copyup.o  \
	dirhelper.o rename.o unlink.o lookup.o persistent_inode.o \
	commonfops.o dirfops.o print.o sioq.o
