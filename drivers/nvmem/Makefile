#
# Makefile for nvmem drivers.
#

obj-$(CONFIG_NVMEM)		+= nvmem_core.o
nvmem_core-y			:= core.o

# Devices
obj-$(CONFIG_MTK_EFUSE)		+= nvmem_mtk-efuse.o
nvmem_mtk-efuse-y		:= mtk-efuse.o
obj-$(CONFIG_QCOM_QFPROM)	+= nvmem_qfprom.o
nvmem_qfprom-y			:= qfprom.o
obj-$(CONFIG_NVMEM_SUNXI_SID)	+= nvmem_sunxi_sid.o
nvmem_sunxi_sid-y		:= sunxi_sid.o
