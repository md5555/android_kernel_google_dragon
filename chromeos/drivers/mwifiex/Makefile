#
# Makefile for Marvell WiFi driver.
#

ccflags-y += -Ichromeos/drivers/mwifiex/mlan
ccflags-y += -DLINUX
ccflags-y += -DFPNUM='"64"'
ccflags-y += -DDEBUG_LEVEL1
ccflags-y += -DPROC_DEBUG
ccflags-y += -DREASSOCIATION
ccflags-y += -DMFG_CMD_SUPPORT
ccflags-y += -DSDIO_MULTI_PORT_TX_AGGR
ccflags-y += -DSDIO_MULTI_PORT_RX_AGGR

mwifiex-y += mlan/mlan_module.o
mwifiex-y += mlan/mlan_shim.o
mwifiex-y += mlan/mlan_init.o
mwifiex-y += mlan/mlan_cfp.o
mwifiex-y += mlan/mlan_cmdevt.o
mwifiex-y += mlan/mlan_misc.o
mwifiex-y += mlan/mlan_txrx.o
mwifiex-y += mlan/mlan_wmm.o
mwifiex-y += mlan/mlan_11n.o
mwifiex-y += mlan/mlan_11n_aggr.o
mwifiex-y += mlan/mlan_11n_rxreorder.o
mwifiex-y += mlan/mlan_scan.o
mwifiex-y += mlan/mlan_join.o
mwifiex-y += mlan/mlan_sta_ioctl.o
mwifiex-y += mlan/mlan_sta_cmd.o
mwifiex-y += mlan/mlan_sta_cmdresp.o
mwifiex-y += mlan/mlan_sta_event.o
mwifiex-y += mlan/mlan_sta_tx.o
mwifiex-y += mlan/mlan_sta_rx.o
mwifiex-y += mlan/mlan_11h.o
mwifiex-y += mlan/mlan_meas.o
mwifiex-y += mlan/mlan_11d.o
mwifiex-y += mlan/mlan_sdio.o

obj-$(CONFIG_MWIFIEX) += mwifiex.o

mwifiex_sdio-y += mlinux/moal_main.o
mwifiex_sdio-y += mlinux/moal_shim.o
mwifiex_sdio-y += mlinux/moal_priv.o
mwifiex_sdio-y += mlinux/moal_wext.o
mwifiex_sdio-y += mlinux/moal_ioctl.o
mwifiex_sdio-y += mlinux/moal_debug.o
mwifiex_sdio-y += mlinux/moal_sdio_mmc.o
mwifiex_sdio-$(CONFIG_PROC_FS) += mlinux/moal_proc.o

obj-$(CONFIG_MWIFIEX_SDIO) += mwifiex_sdio.o
