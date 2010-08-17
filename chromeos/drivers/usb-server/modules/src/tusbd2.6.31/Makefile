DEBUG ?= n
USE_EHCI_FIX ?= y
VHCI ?= n
STUB ?= y

obj-m := tusbd.o
tusbd-objs := driver.o usbdcdev.o minor_control.o minor_device.o minor_vhci.o usb_device.o vhci_device.o fifo.o utils.o waitable_queue.o bind_unbind.o urb_chain.o

KERNELDIR ?= /usr/src/linux
PWD:=$(shell pwd)

EXTRA_CFLAGS += -DHCD_H_PATH=\"$(KERNELDIR)/drivers/usb/core/hcd.h\"

ifeq ($(DEBUG),y)
EXTRA_CFLAGS += -D_USBD_DEBUG_BUILD_
endif

ifeq ($(VHCI),y)
EXTRA_CFLAGS += -D_USBD_ENABLE_VHCI_
endif

ifeq ($(STUB),y)
EXTRA_CFLAGS += -D_USBD_ENABLE_STUB_
endif

ifeq ($(USE_EHCI_FIX),y)
EXTRA_CFLAGS += -D_USBD_USE_EHCI_FIX_
endif

default: 
	make -C $(KERNELDIR) M=$(PWD) modules
#	cp tusbd.ko ..
#	make clean
#	mv ../tusbd.ko ./

clean:
	make -C $(KERNELDIR) M=$(PWD) clean
