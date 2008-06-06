#
# Makefile for the Linux kernel device drivers.
#

obj-$(CONFIG_SCSI_ISCSITARGET)		+= iscsi_trgt.o
iscsi_trgt-objs	:= tio.o iscsi.o nthread.o wthread.o config.o digest.o \
			conn.o session.o target.o volume.o iotype.o \
			file-io.o null-io.o target_disk.o event.o param.o \
			block-io.o
