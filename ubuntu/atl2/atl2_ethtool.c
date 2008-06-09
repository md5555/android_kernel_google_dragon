/* atl2_ethtool.c -- atl2 ethtool support
 *
 * Copyright(c) 2007 Atheros Corporation. All rights reserved.
 * Copyright(c) 2006 xiong huang <xiong.huang@atheros.com>
 * Copyright(c) 2007 Chris Snook <csnook@redhat.com>
 *
 * Derived from Intel e1000 driver
 * Copyright(c) 1999 - 2005 Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 59
 * Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#include <linux/bitops.h>
#include <linux/ethtool.h>
#include <linux/mii.h>
#include <linux/netdevice.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>

#include "atl2.h"
#include "atl2_hw.h"

extern char atl2_driver_name[];
extern char atl2_driver_version[];

extern int atl2_up(struct atl2_adapter *adapter);
extern void atl2_down(struct atl2_adapter *adapter);
extern void atl2_reinit_locked(struct atl2_adapter *adapter);
extern s32 atl2_reset_hw(struct atl2_hw *hw);

static int
atl2_get_settings(struct net_device *netdev, struct ethtool_cmd *ecmd)
{
	struct atl2_adapter *adapter = netdev_priv(netdev);
	struct atl2_hw *hw = &adapter->hw;

	ecmd->supported = (SUPPORTED_10baseT_Half |
		SUPPORTED_10baseT_Full |
		SUPPORTED_100baseT_Half |
		SUPPORTED_100baseT_Full |
		SUPPORTED_Autoneg |
		SUPPORTED_TP);
	ecmd->advertising = ADVERTISED_TP;

	ecmd->advertising |= ADVERTISED_Autoneg;
	ecmd->advertising |= hw->autoneg_advertised;

	ecmd->port = PORT_TP;
	ecmd->phy_address = 0;
	ecmd->transceiver = XCVR_INTERNAL;

	if (adapter->link_speed != SPEED_0) {
		ecmd->speed = adapter->link_speed;
		if (adapter->link_duplex == FULL_DUPLEX)
			ecmd->duplex = DUPLEX_FULL;
		else
			ecmd->duplex = DUPLEX_HALF;
	} else {
		ecmd->speed = -1;
		ecmd->duplex = -1;
	}

	ecmd->autoneg = AUTONEG_ENABLE;
	return 0;
}

static int
atl2_set_settings(struct net_device *netdev, struct ethtool_cmd *ecmd)
{
	struct atl2_adapter *adapter = netdev_priv(netdev);
	struct atl2_hw *hw = &adapter->hw;

	while (test_and_set_bit(__ATL2_RESETTING, &adapter->flags))
		msleep(1);

	if (ecmd->autoneg == AUTONEG_ENABLE) {
#define MY_ADV_MASK	(ADVERTISE_10_HALF| \
					 ADVERTISE_10_FULL| \
					 ADVERTISE_100_HALF| \
					 ADVERTISE_100_FULL)

		if ((ecmd->advertising&MY_ADV_MASK) == MY_ADV_MASK) {
			hw->MediaType = MEDIA_TYPE_AUTO_SENSOR;
			hw->autoneg_advertised =  MY_ADV_MASK;
		} else if ((ecmd->advertising&MY_ADV_MASK) == ADVERTISE_100_FULL) {
			hw->MediaType = MEDIA_TYPE_100M_FULL;
			hw->autoneg_advertised = ADVERTISE_100_FULL;
		} else if ((ecmd->advertising&MY_ADV_MASK) == ADVERTISE_100_HALF) {
			hw->MediaType = MEDIA_TYPE_100M_HALF;
			hw->autoneg_advertised = ADVERTISE_100_HALF;
		} else if ((ecmd->advertising&MY_ADV_MASK) == ADVERTISE_10_FULL) {
			hw->MediaType = MEDIA_TYPE_10M_FULL;
			hw->autoneg_advertised = ADVERTISE_10_FULL;
		}  else if ((ecmd->advertising&MY_ADV_MASK) == ADVERTISE_10_HALF) {
			hw->MediaType = MEDIA_TYPE_10M_HALF;
			hw->autoneg_advertised = ADVERTISE_10_HALF;
		} else {
			clear_bit(__ATL2_RESETTING, &adapter->flags);
			return -EINVAL;
		}
		ecmd->advertising = hw->autoneg_advertised |
			ADVERTISED_TP | ADVERTISED_Autoneg;
	} else {
		clear_bit(__ATL2_RESETTING, &adapter->flags);
		return -EINVAL;
	}

	/* reset the link */
	if (netif_running(adapter->netdev)) {
		atl2_down(adapter);
		atl2_up(adapter);
	} else
		atl2_reset_hw(&adapter->hw);

	clear_bit(__ATL2_RESETTING, &adapter->flags);
	return 0;
}

static u32
atl2_get_tx_csum(struct net_device *netdev)
{
	return (netdev->features & NETIF_F_HW_CSUM) != 0;
}

static u32
atl2_get_msglevel(struct net_device *netdev)
{
	return 0;
}

/*
 * It's sane for this to be empty, but we might want to take advantage of this.
 */
static void
atl2_set_msglevel(struct net_device *netdev, u32 data)
{
}

static int
atl2_get_regs_len(struct net_device *netdev)
{
#define ATL2_REGS_LEN 42
	return ATL2_REGS_LEN * sizeof(u32);
}

static void
atl2_get_regs(struct net_device *netdev, struct ethtool_regs *regs, void *p)
{
	struct atl2_adapter *adapter = netdev_priv(netdev);
	struct atl2_hw *hw = &adapter->hw;
	u32 *regs_buff = p;
	u16 phy_data;

	memset(p, 0, ATL2_REGS_LEN * sizeof(u32));

	regs->version = (1 << 24) | (hw->revision_id << 16) | hw->device_id;

	regs_buff[0]  = ATL2_READ_REG(hw, REG_VPD_CAP);
	regs_buff[1]  = ATL2_READ_REG(hw, REG_SPI_FLASH_CTRL);
	regs_buff[2]  = ATL2_READ_REG(hw, REG_SPI_FLASH_CONFIG);
	regs_buff[3]  = ATL2_READ_REG(hw, REG_TWSI_CTRL);
	regs_buff[4]  = ATL2_READ_REG(hw, REG_PCIE_DEV_MISC_CTRL);
	regs_buff[5]  = ATL2_READ_REG(hw, REG_MASTER_CTRL);
	regs_buff[6]  = ATL2_READ_REG(hw, REG_MANUAL_TIMER_INIT);
	regs_buff[7]  = ATL2_READ_REG(hw, REG_IRQ_MODU_TIMER_INIT);
	regs_buff[8]  = ATL2_READ_REG(hw, REG_PHY_ENABLE);
	regs_buff[9]  = ATL2_READ_REG(hw, REG_CMBDISDMA_TIMER);
	regs_buff[10] = ATL2_READ_REG(hw, REG_IDLE_STATUS);
	regs_buff[11] = ATL2_READ_REG(hw, REG_MDIO_CTRL);
	regs_buff[12] = ATL2_READ_REG(hw, REG_SERDES_LOCK);
	regs_buff[13] = ATL2_READ_REG(hw, REG_MAC_CTRL);
	regs_buff[14] = ATL2_READ_REG(hw, REG_MAC_IPG_IFG);
	regs_buff[15] = ATL2_READ_REG(hw, REG_MAC_STA_ADDR);
	regs_buff[16] = ATL2_READ_REG(hw, REG_MAC_STA_ADDR+4);
	regs_buff[17] = ATL2_READ_REG(hw, REG_RX_HASH_TABLE);
	regs_buff[18] = ATL2_READ_REG(hw, REG_RX_HASH_TABLE+4);
	regs_buff[19] = ATL2_READ_REG(hw, REG_MAC_HALF_DUPLX_CTRL);
	regs_buff[20] = ATL2_READ_REG(hw, REG_MTU);
	regs_buff[21] = ATL2_READ_REG(hw, REG_WOL_CTRL);
	regs_buff[22] = ATL2_READ_REG(hw, REG_SRAM_TXRAM_END);
	regs_buff[23] = ATL2_READ_REG(hw, REG_DESC_BASE_ADDR_HI);
	regs_buff[24] = ATL2_READ_REG(hw, REG_TXD_BASE_ADDR_LO);
	regs_buff[25] = ATL2_READ_REG(hw, REG_TXD_MEM_SIZE);	
	regs_buff[26] = ATL2_READ_REG(hw, REG_TXS_BASE_ADDR_LO);
	regs_buff[27] = ATL2_READ_REG(hw, REG_TXS_MEM_SIZE);
	regs_buff[28] = ATL2_READ_REG(hw, REG_RXD_BASE_ADDR_LO);
	regs_buff[29] = ATL2_READ_REG(hw, REG_RXD_BUF_NUM);
	regs_buff[30] = ATL2_READ_REG(hw, REG_DMAR);
	regs_buff[31] = ATL2_READ_REG(hw, REG_TX_CUT_THRESH);
	regs_buff[32] = ATL2_READ_REG(hw, REG_DMAW);
	regs_buff[33] = ATL2_READ_REG(hw, REG_PAUSE_ON_TH);
	regs_buff[34] = ATL2_READ_REG(hw, REG_PAUSE_OFF_TH);
	regs_buff[35] = ATL2_READ_REG(hw, REG_MB_TXD_WR_IDX);
	regs_buff[36] = ATL2_READ_REG(hw, REG_MB_RXD_RD_IDX);	
	regs_buff[38] = ATL2_READ_REG(hw, REG_ISR);	
	regs_buff[39] = ATL2_READ_REG(hw, REG_IMR);	

	atl2_read_phy_reg(hw, MII_BMCR, &phy_data);
	regs_buff[40] = (u32)phy_data;
	atl2_read_phy_reg(hw, MII_BMSR, &phy_data);
	regs_buff[41] = (u32)phy_data;
}

static int
atl2_get_eeprom_len(struct net_device *netdev)
{
	struct atl2_adapter *adapter = netdev_priv(netdev);

	if (!atl2_check_eeprom_exist(&adapter->hw)) {
		return 512;
	} else
		return 0;
}

static int
atl2_get_eeprom(struct net_device *netdev, struct ethtool_eeprom *eeprom, u8 *bytes)
{
	struct atl2_adapter *adapter = netdev_priv(netdev);
	struct atl2_hw *hw = &adapter->hw;
	u32 *eeprom_buff;
	int first_dword, last_dword;
	int ret_val = 0;
	int i;

	if (eeprom->len == 0)
		return -EINVAL;

	if (atl2_check_eeprom_exist(hw)) {
		return -EINVAL;
	}

	eeprom->magic = hw->vendor_id | (hw->device_id << 16);

	first_dword = eeprom->offset >> 2;
	last_dword = (eeprom->offset + eeprom->len - 1) >> 2;

	eeprom_buff = kmalloc(sizeof(u32) * (last_dword - first_dword + 1), GFP_KERNEL);
	if (!eeprom_buff)
		return -ENOMEM;

	for (i=first_dword; i < last_dword; i++) {
		if (!atl2_read_eeprom(hw, i*4, &(eeprom_buff[i-first_dword])))
			return -EIO;
	}

	memcpy(bytes, (u8 *)eeprom_buff + (eeprom->offset & 3),
		eeprom->len);
	kfree(eeprom_buff);

	return ret_val;
}

static int
atl2_set_eeprom(struct net_device *netdev, struct ethtool_eeprom *eeprom, u8 *bytes)
{
	struct atl2_adapter *adapter = netdev_priv(netdev);
	struct atl2_hw *hw = &adapter->hw;
	u32 *eeprom_buff;
	u32 *ptr;
	int max_len, first_dword, last_dword, ret_val = 0;
	int i;

	if (eeprom->len == 0)
		return -EOPNOTSUPP;

	if (eeprom->magic != (hw->vendor_id | (hw->device_id << 16)))
		return -EFAULT;

	max_len = 512;

	first_dword = eeprom->offset >> 2;
	last_dword = (eeprom->offset + eeprom->len - 1) >> 2;
	eeprom_buff = kmalloc(max_len, GFP_KERNEL);
	if (!eeprom_buff)
		return -ENOMEM;

	ptr = (u32 *)eeprom_buff;

	if (eeprom->offset & 3) {
		/* need read/modify/write of first changed EEPROM word */
		/* only the second byte of the word is being modified */
		if (!atl2_read_eeprom(hw, first_dword*4, &(eeprom_buff[0])))
			return -EIO;
		ptr++;
	}
	if (((eeprom->offset + eeprom->len) & 3) ) {
		/* need read/modify/write of last changed EEPROM word */
		/* only the first byte of the word is being modified */
		
		if (!atl2_read_eeprom(hw, last_dword*4, &(eeprom_buff[last_dword - first_dword])))
			return -EIO;
	}

	/* Device's eeprom is always little-endian, word addressable */
	memcpy(ptr, bytes, eeprom->len);

	for (i = 0; i < last_dword - first_dword + 1; i++) {
		if (!atl2_write_eeprom(hw, ((first_dword+i)*4), eeprom_buff[i]))
			return -EIO;
	}

	kfree(eeprom_buff);
	return ret_val;
}

static void
atl2_get_drvinfo(struct net_device *netdev, struct ethtool_drvinfo *drvinfo)
{
	struct atl2_adapter *adapter = netdev_priv(netdev);

	strncpy(drvinfo->driver,  atl2_driver_name, 32);
	strncpy(drvinfo->version, atl2_driver_version, 32);
	strncpy(drvinfo->fw_version, "L2", 32);
	strncpy(drvinfo->bus_info, pci_name(adapter->pdev), 32);
	drvinfo->n_stats = 0;
	drvinfo->testinfo_len = 0;
	drvinfo->regdump_len = atl2_get_regs_len(netdev);
	drvinfo->eedump_len = atl2_get_eeprom_len(netdev);
}

static void
atl2_get_wol(struct net_device *netdev, struct ethtool_wolinfo *wol)
{
	struct atl2_adapter *adapter = netdev_priv(netdev);

	wol->supported = WAKE_MAGIC;
	wol->wolopts = 0;

	if (adapter->wol & ATL2_WUFC_EX)
		wol->wolopts |= WAKE_UCAST;
	if (adapter->wol & ATL2_WUFC_MC)
		wol->wolopts |= WAKE_MCAST;
	if (adapter->wol & ATL2_WUFC_BC)
		wol->wolopts |= WAKE_BCAST;
	if (adapter->wol & ATL2_WUFC_MAG)
		wol->wolopts |= WAKE_MAGIC;
	if (adapter->wol & ATL2_WUFC_LNKC)
		wol->wolopts |= WAKE_PHY;
}

static int
atl2_set_wol(struct net_device *netdev, struct ethtool_wolinfo *wol)
{
	struct atl2_adapter *adapter = netdev_priv(netdev);

	if (wol->wolopts & (WAKE_ARP | WAKE_MAGICSECURE))
		return -EOPNOTSUPP;

	if (wol->wolopts & (WAKE_MCAST|WAKE_BCAST|WAKE_MCAST))
		return -EOPNOTSUPP;

	/* these settings will always override what we currently have */
	adapter->wol = 0;

	if (wol->wolopts & WAKE_MAGIC)
		adapter->wol |= ATL2_WUFC_MAG;
	if (wol->wolopts & WAKE_PHY)
		adapter->wol |= ATL2_WUFC_LNKC;

	return 0;
}

static int
atl2_nway_reset(struct net_device *netdev)
{
	struct atl2_adapter *adapter = netdev_priv(netdev);
	if (netif_running(netdev))
		atl2_reinit_locked(adapter);
	return 0;
}

static struct ethtool_ops atl2_ethtool_ops = {
	.get_settings		= atl2_get_settings,
	.set_settings		= atl2_set_settings,
	.get_drvinfo		= atl2_get_drvinfo,
	.get_regs_len		= atl2_get_regs_len,
	.get_regs		= atl2_get_regs,
	.get_wol		= atl2_get_wol,
	.set_wol		= atl2_set_wol,
	.get_msglevel		= atl2_get_msglevel,
	.set_msglevel		= atl2_set_msglevel,
	.nway_reset		= atl2_nway_reset,
	.get_link		= ethtool_op_get_link,
	.get_eeprom_len		= atl2_get_eeprom_len,
	.get_eeprom		= atl2_get_eeprom,
	.set_eeprom		= atl2_set_eeprom,
	.get_tx_csum		= atl2_get_tx_csum,
	.get_sg			= ethtool_op_get_sg,
	.set_sg			= ethtool_op_set_sg,
#ifdef NETIF_F_TSO
	.get_tso		= ethtool_op_get_tso,
#endif
#if 0 //FIXME: not implemented?
//#ifdef ETHTOOL_GPERMADDR
	.get_perm_addr		= ethtool_op_get_perm_addr,
#endif
};

void
atl2_set_ethtool_ops(struct net_device *netdev)
{
	SET_ETHTOOL_OPS(netdev, &atl2_ethtool_ops);
}
