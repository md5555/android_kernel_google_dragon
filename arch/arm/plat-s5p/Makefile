# arch/arm/plat-s5p/Makefile
#
# Copyright (c) 2009 Samsung Electronics Co., Ltd.
# 		http://www.samsung.com/
#
# Licensed under GPLv2

obj-y				:=
obj-m				:=
obj-n				:= dummy.o
obj-				:=

# Core files

obj-y				+= dev-uart.o
obj-y				+= cpu.o
obj-y				+= clock.o
obj-y				+= irq.o
obj-y				+= bootmem.o
obj-y				+= setup-i2c0.o
obj-$(CONFIG_SYSTIMER_S5P)	+= systimer-s5p.o

obj-$(CONFIG_PM)		+= pm.o
obj-$(CONFIG_PM)		+= irq-pm.o
obj-$(CONFIG_PM)		+= sleep.o
