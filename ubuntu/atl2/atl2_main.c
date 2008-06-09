/* atl2_main.c -- atl2 driver main functions
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

#include <asm/atomic.h>
#include <linux/dma-mapping.h>
#include <linux/etherdevice.h>
#include <linux/hardirq.h>
#include <linux/if_vlan.h>
#include <linux/in.h>
#include <linux/interrupt.h>
#include <linux/ip.h>
#include <linux/irqflags.h>
#include <linux/irqreturn.h>
#include <linux/mii.h>
#include <linux/net.h>
#include <linux/netdevice.h>
#include <linux/pci.h>
#include <linux/pci_ids.h>
#include <linux/pm.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/tcp.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#include "atl2.h"

#define ATL2_DRV_VERSION "2.0.4"

char atl2_driver_name[] = "atl2";
static const char atl2_driver_string[] = "Atheros(R) L2 Ethernet Driver";
static char atl2_copyright[] = "Copyright (c) 2007 Atheros Corporation.";
char atl2_driver_version[] = ATL2_DRV_VERSION;

MODULE_AUTHOR("Atheros Corporation <xiong.huang@atheros.com>, Chris Snook <csnook@redhat.com>");
MODULE_DESCRIPTION("Atheros Fast Ethernet Network Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(ATL2_DRV_VERSION);

/* FIXME: remove this after merging, goes in pci_ids.h */
#ifndef PCI_DEVICE_ID_ATTANSIC_L2
#define PCI_DEVICE_ID_ATTANSIC_L2	0x2048
#endif

/*
 * atl2_pci_tbl - PCI Device ID Table
 */
static struct pci_device_id atl2_pci_tbl[] = {
	{PCI_DEVICE(PCI_VENDOR_ID_ATTANSIC, PCI_DEVICE_ID_ATTANSIC_L2)},
	/* required last entry */
	{0,}
};
MODULE_DEVICE_TABLE(pci, atl2_pci_tbl);

extern void atl2_set_ethtool_ops(struct net_device *netdev);
#ifdef ETHTOOL_OPS_COMPAT
extern int ethtool_ioctl(struct ifreq *ifr);
#endif

#define COPYBREAK_DEFAULT 256
static unsigned int copybreak __read_mostly = COPYBREAK_DEFAULT;
module_param(copybreak, uint, 0644);
MODULE_PARM_DESC(copybreak, "Maximum size of packet that is copied to a new buffer on receive");

extern void atl2_check_options(struct atl2_adapter *adapter);
#ifdef SIOCDEVPRIVATE
extern int atl2_priv_ioctl(struct net_device* netdev, struct ifreq* ifr);
#endif

/**
 * atl2_sw_init - Initialize general software structures (struct atl2_adapter)
 * @adapter: board private structure to initialize
 *
 * atl2_sw_init initializes the Adapter private data structure.
 * Fields are initialized based on PCI device information and
 * OS network device settings (MTU size).
 **/
static int __devinit
atl2_sw_init(struct atl2_adapter *adapter)
{
	struct atl2_hw *hw = &adapter->hw;
	struct pci_dev *pdev = adapter->pdev;

	/* PCI config space info */
	hw->vendor_id = pdev->vendor;
	hw->device_id = pdev->device;
	hw->subsystem_vendor_id = pdev->subsystem_vendor;
	hw->subsystem_id = pdev->subsystem_device;

	pci_read_config_byte(pdev, PCI_REVISION_ID, &hw->revision_id);
	pci_read_config_word(pdev, PCI_COMMAND, &hw->pci_cmd_word);

	adapter->wol = 0;
	adapter->ict = 50000;  // 100ms
	adapter->link_speed = SPEED_0;   // hardware init
	adapter->link_duplex = FULL_DUPLEX;

	hw->phy_configured = false;
	hw->preamble_len = 7;
	hw->ipgt = 0x60;
	hw->min_ifg = 0x50;
	hw->ipgr1 = 0x40;
	hw->ipgr2 = 0x60;
	hw->retry_buf = 2;
	hw->max_retry = 0xf;
	hw->lcol = 0x37;
	hw->jam_ipg = 7;
	hw->fc_rxd_hi = 0;
	hw->fc_rxd_lo = 0;
	hw->max_frame_size = adapter->netdev->mtu;

	spin_lock_init(&adapter->stats_lock);
	spin_lock_init(&adapter->tx_lock);

	set_bit(__ATL2_DOWN, &adapter->flags);

	return 0;
}

/**
 * atl2_set_multi - Multicast and Promiscuous mode set
 * @netdev: network interface device structure
 *
 * The set_multi entry point is called whenever the multicast address
 * list or the network interface flags are updated.  This routine is
 * responsible for configuring the hardware for proper multicast,
 * promiscuous mode, and all-multi behavior.
 **/
static void
atl2_set_multi(struct net_device *netdev)
{
	struct atl2_adapter *adapter = netdev_priv(netdev);
	struct atl2_hw *hw = &adapter->hw;
	struct dev_mc_list *mc_ptr;
	u32 rctl;
	u32 hash_value;

	/* Check for Promiscuous and All Multicast modes */
	rctl = ATL2_READ_REG(hw, REG_MAC_CTRL);

	if(netdev->flags & IFF_PROMISC) {
		rctl |= MAC_CTRL_PROMIS_EN;
	} else if(netdev->flags & IFF_ALLMULTI) {
		rctl |= MAC_CTRL_MC_ALL_EN;
		rctl &= ~MAC_CTRL_PROMIS_EN;
	} else {
		rctl &= ~(MAC_CTRL_PROMIS_EN | MAC_CTRL_MC_ALL_EN);
	}

	ATL2_WRITE_REG(hw, REG_MAC_CTRL, rctl);

	/* clear the old settings from the multicast hash table */
	ATL2_WRITE_REG(hw, REG_RX_HASH_TABLE, 0);
	ATL2_WRITE_REG_ARRAY(hw, REG_RX_HASH_TABLE, 1, 0);

	/* comoute mc addresses' hash value ,and put it into hash table */
	for(mc_ptr = netdev->mc_list; mc_ptr; mc_ptr = mc_ptr->next) {
		hash_value = atl2_hash_mc_addr(hw, mc_ptr->dmi_addr);
		atl2_hash_set(hw, hash_value);
	}
}

static void
init_ring_ptrs(struct atl2_adapter *adapter)
{
	// Read / Write Ptr Initialize:
	adapter->txd_write_ptr = 0;
	atomic_set(&adapter->txd_read_ptr, 0);

	adapter->rxd_read_ptr = 0;
	adapter->rxd_write_ptr = 0;

	atomic_set(&adapter->txs_write_ptr, 0);
	adapter->txs_next_clear = 0;
}

/**
 * atl2_configure - Configure Transmit&Receive Unit after Reset
 * @adapter: board private structure
 *
 * Configure the Tx /Rx unit of the MAC after a reset.
 **/
static int
atl2_configure(struct atl2_adapter *adapter)
{
	struct atl2_hw * hw = &adapter->hw;
	u32 value;

	// clear interrupt status
	ATL2_WRITE_REG(&adapter->hw, REG_ISR, 0xffffffff);

	// set MAC Address
	value = (((u32)hw->mac_addr[2]) << 24) |
		(((u32)hw->mac_addr[3]) << 16) |
		(((u32)hw->mac_addr[4]) << 8 ) |
		(((u32)hw->mac_addr[5])      ) ;
	ATL2_WRITE_REG(hw, REG_MAC_STA_ADDR, value);
	value = (((u32)hw->mac_addr[0]) << 8 ) |
		(((u32)hw->mac_addr[1])      ) ;
	ATL2_WRITE_REG(hw, (REG_MAC_STA_ADDR+4), value);

	// HI base address
	ATL2_WRITE_REG(hw, REG_DESC_BASE_ADDR_HI,
		(u32)((adapter->ring_dma & 0xffffffff00000000ULL) >> 32));

	// LO base address
	ATL2_WRITE_REG(hw, REG_TXD_BASE_ADDR_LO,
		(u32)(adapter->txd_dma & 0x00000000ffffffffULL));
	ATL2_WRITE_REG(hw, REG_TXS_BASE_ADDR_LO,
		(u32)(adapter->txs_dma & 0x00000000ffffffffULL));
	ATL2_WRITE_REG(hw, REG_RXD_BASE_ADDR_LO,
			(u32)(adapter->rxd_dma & 0x00000000ffffffffULL));

	// element count
	ATL2_WRITE_REGW(hw, REG_TXD_MEM_SIZE, (u16)(adapter->txd_ring_size/4));
	ATL2_WRITE_REGW(hw, REG_TXS_MEM_SIZE, (u16)adapter->txs_ring_size);
	ATL2_WRITE_REGW(hw, REG_RXD_BUF_NUM,  (u16)adapter->rxd_ring_size);

	/* config Internal SRAM */
/*
    ATL2_WRITE_REGW(hw, REG_SRAM_TXRAM_END, sram_tx_end);
    ATL2_WRITE_REGW(hw, REG_SRAM_TXRAM_END, sram_rx_end);
*/

	/* config IPG/IFG */
	value = (((u32)hw->ipgt & MAC_IPG_IFG_IPGT_MASK) << MAC_IPG_IFG_IPGT_SHIFT) |
		(((u32)hw->min_ifg & MAC_IPG_IFG_MIFG_MASK) << MAC_IPG_IFG_MIFG_SHIFT) |
		(((u32)hw->ipgr1 & MAC_IPG_IFG_IPGR1_MASK) << MAC_IPG_IFG_IPGR1_SHIFT)|
		(((u32)hw->ipgr2 & MAC_IPG_IFG_IPGR2_MASK) << MAC_IPG_IFG_IPGR2_SHIFT);
	ATL2_WRITE_REG(hw, REG_MAC_IPG_IFG, value);

	/* config  Half-Duplex Control */
	value = ((u32)hw->lcol & MAC_HALF_DUPLX_CTRL_LCOL_MASK) |
		(((u32)hw->max_retry & MAC_HALF_DUPLX_CTRL_RETRY_MASK) <<
		MAC_HALF_DUPLX_CTRL_RETRY_SHIFT) |
		MAC_HALF_DUPLX_CTRL_EXC_DEF_EN |
		(0xa << MAC_HALF_DUPLX_CTRL_ABEBT_SHIFT) |
		(((u32)hw->jam_ipg & MAC_HALF_DUPLX_CTRL_JAMIPG_MASK) <<
		MAC_HALF_DUPLX_CTRL_JAMIPG_SHIFT);
	ATL2_WRITE_REG(hw, REG_MAC_HALF_DUPLX_CTRL, value);

	/* set Interrupt Moderator Timer */
	ATL2_WRITE_REGW(hw, REG_IRQ_MODU_TIMER_INIT, adapter->imt);
	ATL2_WRITE_REG(hw, REG_MASTER_CTRL, MASTER_CTRL_ITIMER_EN);

	/* set Interrupt Clear Timer */
	ATL2_WRITE_REGW(hw, REG_CMBDISDMA_TIMER, adapter->ict);

	/* set MTU */
	ATL2_WRITE_REG(hw, REG_MTU, adapter->netdev->mtu +
		ENET_HEADER_SIZE + VLAN_SIZE + ETHERNET_FCS_SIZE);

	/* 1590 */
	ATL2_WRITE_REG(hw, REG_TX_CUT_THRESH, 0x177);

	/* flow control */
	ATL2_WRITE_REGW(hw, REG_PAUSE_ON_TH, hw->fc_rxd_hi);
	ATL2_WRITE_REGW(hw, REG_PAUSE_OFF_TH, hw->fc_rxd_lo);

	/* Init mailbox */
	ATL2_WRITE_REGW(hw, REG_MB_TXD_WR_IDX, (u16)adapter->txd_write_ptr);
	ATL2_WRITE_REGW(hw, REG_MB_RXD_RD_IDX, (u16)adapter->rxd_read_ptr);

	/* enable DMA read/write */
	ATL2_WRITE_REGB(hw, REG_DMAR, DMAR_EN);
	ATL2_WRITE_REGB(hw, REG_DMAW, DMAW_EN);

	value = ATL2_READ_REG(&adapter->hw, REG_ISR);
	if ((value & ISR_PHY_LINKDOWN) != 0) {
		value = 1; // config failed
	} else {
		value = 0;
	}

	// clear all interrupt status
	ATL2_WRITE_REG(&adapter->hw, REG_ISR, 0x3fffffff);
	ATL2_WRITE_REG(&adapter->hw, REG_ISR, 0);
	return value;
}

/**
 * atl2_setup_ring_resources - allocate Tx / RX descriptor resources
 * @adapter: board private structure
 *
 * Return 0 on success, negative on failure
 **/
static s32
atl2_setup_ring_resources(struct atl2_adapter *adapter)
{
	struct pci_dev *pdev = adapter->pdev;
	int size;
	u8 offset = 0;

	/* real ring DMA buffer */
	adapter->ring_size = size =
		adapter->txd_ring_size * 1 + 7 +        // dword align
		adapter->txs_ring_size * 4 + 7 +        // dword align
		adapter->rxd_ring_size * 1536 + 127;    // 128bytes align
	
	adapter->ring_vir_addr = pci_alloc_consistent(pdev, size, &adapter->ring_dma);
	if (!adapter->ring_vir_addr) {
		return -ENOMEM;
	}
#if 0
	if (adapter->pci_using_64) {
		// test whether HIDWORD dma buffer is not cross boundary
		if (    ((adapter->ring_dma       &0xffffffff00000000ULL)>>32)
			!= (((adapter->ring_dma+size)&0xffffffff00000000ULL)>>32) ) {
			pci_free_consistent(pdev, adapter->ring_size, adapter->ring_vir_addr, adapter->ring_dma);
		DEBUGOUT("memory allocated cross 32bit boundary !");
		return -ENOMEM;
		}
	}
#endif
	memset(adapter->ring_vir_addr, 0, adapter->ring_size);

	// Init TXD Ring
	adapter->txd_dma = adapter->ring_dma ;
	offset = (adapter->txd_dma & 0x7) ? (8 - (adapter->txd_dma & 0x7)) : 0;
	adapter->txd_dma += offset;
	adapter->txd_ring = (tx_pkt_header_t*) (adapter->ring_vir_addr + offset);
	
	// Init TXS Ring
	adapter->txs_dma = adapter->txd_dma + adapter->txd_ring_size;
	offset = (adapter->txs_dma & 0x7) ? (8- (adapter->txs_dma & 0x7)) : 0;
	adapter->txs_dma += offset;
	adapter->txs_ring = (tx_pkt_status_t*)
		(((u8*)adapter->txd_ring) + (adapter->txd_ring_size+offset));

	// Init RXD Ring
	adapter->rxd_dma = adapter->txs_dma + adapter->txs_ring_size*4;
	offset = (adapter->rxd_dma & 127) ? (128 - (adapter->rxd_dma & 127)) : 0;
	if (offset > 7) {
		offset -= 8;
	} else {
		offset += (128 - 8);
	}
	adapter->rxd_dma += offset;
	adapter->rxd_ring = (rx_desc_t*) (((u8*)adapter->txs_ring) +
		(adapter->txs_ring_size*4 + offset));

// Read / Write Ptr Initialize:
//      init_ring_ptrs(adapter);

	return ATL2_SUCCESS;
}

/**
 * atl2_irq_enable - Enable default interrupt generation settings
 * @adapter: board private structure
 **/
static inline void
atl2_irq_enable(struct atl2_adapter *adapter)
{
	ATL2_WRITE_REG(&adapter->hw, REG_IMR, IMR_NORMAL_MASK);
	ATL2_WRITE_FLUSH(&adapter->hw);
}

/**
 * atl2_irq_disable - Mask off interrupt generation on the NIC
 * @adapter: board private structure
 **/
static inline void
atl2_irq_disable(struct atl2_adapter *adapter)
{
    ATL2_WRITE_REG(&adapter->hw, REG_IMR, 0);
    ATL2_WRITE_FLUSH(&adapter->hw);
    synchronize_irq(adapter->pdev->irq);
}

#ifdef NETIF_F_HW_VLAN_TX
static void
atl2_vlan_rx_register(struct net_device *netdev, struct vlan_group *grp)
{
	struct atl2_adapter *adapter = netdev_priv(netdev);
	u32 ctrl;

	atl2_irq_disable(adapter);
	adapter->vlgrp = grp;

	if(grp) {
		/* enable VLAN tag insert/strip */
		ctrl = ATL2_READ_REG(&adapter->hw, REG_MAC_CTRL);
		ctrl |= MAC_CTRL_RMV_VLAN;
		ATL2_WRITE_REG(&adapter->hw, REG_MAC_CTRL, ctrl);
	} else {
		/* disable VLAN tag insert/strip */
		ctrl = ATL2_READ_REG(&adapter->hw, REG_MAC_CTRL);
		ctrl &= ~MAC_CTRL_RMV_VLAN;
		ATL2_WRITE_REG(&adapter->hw, REG_MAC_CTRL, ctrl);
	}

	atl2_irq_enable(adapter);
}

static void
atl2_restore_vlan(struct atl2_adapter *adapter)
{
	atl2_vlan_rx_register(adapter->netdev, adapter->vlgrp);
}
#endif

static void
atl2_intr_rx(struct atl2_adapter* adapter)
{
	struct net_device *netdev = adapter->netdev;
	rx_desc_t* rxd;
	struct sk_buff* skb;

	do {
		rxd = adapter->rxd_ring+adapter->rxd_write_ptr;
		if (!rxd->status.update)
			break; // end of tx

		// clear this flag at once
		rxd->status.update = 0;

		if (rxd->status.ok && rxd->status.pkt_size >= 60) {
			int rx_size = (int)(rxd->status.pkt_size - 4);
			// alloc new buffer
			skb = netdev_alloc_skb(netdev, rx_size + NET_IP_ALIGN);
			if (NULL == skb) {
				printk(KERN_WARNING"%s: Memory squeeze, deferring packet.\n",
					netdev->name);
				/* We should check that some rx space is free.
				 * If not, free one and mark stats->rx_dropped++. */
				adapter->net_stats.rx_dropped++;
				break;
			}
/*           FIXME: ???
	    if (rx_size > 1400) {
                int s,c;
		c = 0;
		printk("rx_size= %d\n", rx_size);
		for (s=0; s < 800; s++) {
		    if (0 == c) {
		        printk("%04x ", s);
		    }
		    printk("%02x ", rxd->packet[s]);
		    if (++c == 16) {
			c = 0;
			printk("\n");
		    }
		}
		printk(KERN_WARNING"\n");
	    }
*/
			skb_reserve(skb, NET_IP_ALIGN);
			skb->dev = netdev;
/* gone in 2.6.23, just use memcpy? -- CHS
			eth_copy_and_sum(skb, rxd->packet, rx_size, 0); */
			memcpy(skb->data, rxd->packet, rx_size); /* totally untested -- CHS */
			skb_put(skb, rx_size);
	    /* FIXME ???
            memcpy(skb_put(skb, rx_size), 
                    rxd->packet,
                    rx_size);
            */
			skb->protocol = eth_type_trans(skb, netdev);
#ifdef NETIF_F_HW_VLAN_TX
			if(adapter->vlgrp && (rxd->status.vlan)) {
				u16 vlan_tag = (rxd->status.vtag>>4) |
					((rxd->status.vtag&7) << 13) |
					((rxd->status.vtag&8) << 9);
				vlan_hwaccel_rx(skb, adapter->vlgrp, vlan_tag);
			} else
#endif
			netif_rx(skb);
			adapter->net_stats.rx_bytes += rx_size;
			adapter->net_stats.rx_packets++;
			netdev->last_rx = jiffies;
		} else {
			adapter->net_stats.rx_errors++;

			if (rxd->status.ok && rxd->status.pkt_size <= 60) {
				adapter->net_stats.rx_length_errors++;
			}
			if (rxd->status.mcast)  adapter->net_stats.multicast++;
			if (rxd->status.crc)    adapter->net_stats.rx_crc_errors++;
			if (rxd->status.align)  adapter->net_stats.rx_frame_errors++;
		}

		// advance write ptr
		if (++adapter->rxd_write_ptr == adapter->rxd_ring_size)
			adapter->rxd_write_ptr = 0;
	} while (1);

	// update mailbox ?
	adapter->rxd_read_ptr = adapter->rxd_write_ptr;
	ATL2_WRITE_REGW(&adapter->hw, REG_MB_RXD_RD_IDX, adapter->rxd_read_ptr);
}

static void
atl2_intr_tx(struct atl2_adapter* adapter)
{
	u32 txd_read_ptr;
	u32 txs_write_ptr;
	tx_pkt_status_t* txs;
	tx_pkt_header_t* txph;
	int free_hole = 0;

	do {
		txs_write_ptr = (u32) atomic_read(&adapter->txs_write_ptr);
		txs = adapter->txs_ring + txs_write_ptr;
		if (!txs->update)
			break; // tx stop here

		free_hole = 1;
		txs->update = 0;

		if (++txs_write_ptr == adapter->txs_ring_size)
			txs_write_ptr = 0;
		atomic_set(&adapter->txs_write_ptr, (int)txs_write_ptr);

		txd_read_ptr = (u32) atomic_read(&adapter->txd_read_ptr);
		txph = (tx_pkt_header_t*) (((u8*)adapter->txd_ring) + txd_read_ptr);

		if ( txph->pkt_size != txs->pkt_size) {
			tx_pkt_status_t* old_txs = txs;
			printk(KERN_WARNING
				"%s: txs packet size do not coinsist with txd"
				" txd_:0x%08x, txs_:0x%08x!\n",
				adapter->netdev->name,
				*(u32*)txph, *(u32*)txs);
			printk(KERN_WARNING
				"txd read ptr: 0x%x\n",
				txd_read_ptr);
			txs = adapter->txs_ring + txs_write_ptr;
			printk(KERN_WARNING
				"txs-behind:0x%08x\n",
				*(u32*)txs);
			if (txs_write_ptr < 2) {
				txs = adapter->txs_ring +
					(adapter->txs_ring_size +
					txs_write_ptr - 2);
			} else {
				txs = adapter->txs_ring + (txs_write_ptr - 2);
			}
			printk(KERN_WARNING
				"txs-before:0x%08x\n",
				*(u32*)txs);
			txs = old_txs;
		}

		txd_read_ptr += (((u32)(txph->pkt_size)+7)& ~3);//4for TPH
		if (txd_read_ptr >= adapter->txd_ring_size)
			txd_read_ptr -= adapter->txd_ring_size;

		atomic_set(&adapter->txd_read_ptr, (int)txd_read_ptr);

		// tx statistics:
		if (txs->ok)
			adapter->net_stats.tx_packets++;
		else
			adapter->net_stats.tx_errors++;

		if (txs->defer)         adapter->net_stats.collisions++;
		if (txs->abort_col)     adapter->net_stats.tx_aborted_errors++;
		if (txs->late_col)      adapter->net_stats.tx_window_errors++;
		if (txs->underun)       adapter->net_stats.tx_fifo_errors++;
	} while (1);

	if (free_hole) {
		if(netif_queue_stopped(adapter->netdev) &&
			netif_carrier_ok(adapter->netdev))
			netif_wake_queue(adapter->netdev);
	}
}

static void
atl2_check_for_link(struct atl2_adapter* adapter)
{
	struct net_device *netdev = adapter->netdev;
	u16 phy_data = 0;

	spin_lock(&adapter->stats_lock);
	atl2_read_phy_reg(&adapter->hw, MII_BMSR, &phy_data);
	atl2_read_phy_reg(&adapter->hw, MII_BMSR, &phy_data);
	spin_unlock(&adapter->stats_lock);

	// notify upper layer link down ASAP
	if (!(phy_data & BMSR_LSTATUS)) { // Link Down
		if (netif_carrier_ok(netdev)) { // old link state: Up
		printk(KERN_INFO "%s: %s NIC Link is Down\n",
			atl2_driver_name, netdev->name);
		adapter->link_speed = SPEED_0;
		netif_carrier_off(netdev);
		netif_stop_queue(netdev);
		}
	}
	schedule_work(&adapter->link_chg_task);
}

static inline void
atl2_clear_phy_int(struct atl2_adapter *adapter)
{
	u16 phy_data;
	spin_lock(&adapter->stats_lock);
	atl2_read_phy_reg(&adapter->hw, 19, &phy_data);
	spin_unlock(&adapter->stats_lock);
}

/**
 * atl2_intr - Interrupt Handler
 * @irq: interrupt number
 * @data: pointer to a network interface device structure
 * @pt_regs: CPU registers structure
 **/
static irqreturn_t
atl2_intr(int irq, void *data)
{
	struct atl2_adapter *adapter = netdev_priv(data);
	struct atl2_hw *hw = &adapter->hw;
	u32 status;

	status = ATL2_READ_REG(hw, REG_ISR);
	if (0 == status)
		return IRQ_NONE;

	// link event
	if (status & ISR_PHY) {
		atl2_clear_phy_int(adapter);
	}

	// clear ISR status, and Enable CMB DMA/Disable Interrupt
	ATL2_WRITE_REG(hw, REG_ISR, status | ISR_DIS_INT);

	// FIXME: if PCIe link is down, how did we read the register? -- CHS
	// check if PCIE PHY Link down
	if (status & ISR_PHY_LINKDOWN) {
		if(netif_running(adapter->netdev)) { // reset MAC
			ATL2_WRITE_REG(hw, REG_ISR, 0);
			ATL2_WRITE_REG(hw, REG_IMR, 0);
			ATL2_WRITE_FLUSH(hw);
			schedule_work(&adapter->reset_task);
			return IRQ_HANDLED;
		}
	}

	// check if DMA read/write error ?
	if (status & (ISR_DMAR_TO_RST | ISR_DMAW_TO_RST))
	{
		//ATL2_WRITE_REG(&adapter->hw, REG_MASTER_CTRL, MASTER_CTRL_SOFT_RST);
		ATL2_WRITE_REG(hw, REG_ISR, 0);
		ATL2_WRITE_REG(hw, REG_IMR, 0);
		ATL2_WRITE_FLUSH(hw);
		schedule_work(&adapter->reset_task);
		return IRQ_HANDLED;
	}

	// link event
	if (status & (ISR_PHY | ISR_MANUAL))
	{
		adapter->net_stats.tx_carrier_errors++;
		atl2_check_for_link(adapter);
	}

	// transmit event
	if (status & ISR_TX_EVENT) {
		atl2_intr_tx(adapter);
	}

	// rx exception
	if (status & ISR_RX_EVENT) {
		atl2_intr_rx(adapter);
	}

	// re-enable Interrupt
	ATL2_WRITE_REG(&adapter->hw, REG_ISR, 0);
	return IRQ_HANDLED;
}

static int
atl2_request_irq(struct atl2_adapter *adapter)
{
	struct net_device *netdev = adapter->netdev;
	int flags, err = 0;

	flags = IRQF_SHARED;
#ifdef CONFIG_PCI_MSI
	adapter->have_msi = true;
	if ((err = pci_enable_msi(adapter->pdev)))
		adapter->have_msi = false;

	if (adapter->have_msi)
		flags &= ~IRQF_SHARED;
#endif

	return request_irq(adapter->pdev->irq, &atl2_intr, flags, netdev->name, netdev);
}

/**
 * atl2_free_ring_resources - Free Tx / RX descriptor Resources
 * @adapter: board private structure
 *
 * Free all transmit software resources
 **/
static void
atl2_free_ring_resources(struct atl2_adapter *adapter)
{
	struct pci_dev *pdev = adapter->pdev;
	pci_free_consistent(pdev, adapter->ring_size, adapter->ring_vir_addr, adapter->ring_dma);
}

/**
 * atl2_open - Called when a network interface is made active
 * @netdev: network interface device structure
 *
 * Returns 0 on success, negative value on failure
 *
 * The open entry point is called when a network interface is made
 * active by the system (IFF_UP).  At this point all resources needed
 * for transmit and receive operations are allocated, the interrupt
 * handler is registered with the OS, the watchdog timer is started,
 * and the stack is notified that the interface is ready.
 **/
static int
atl2_open(struct net_device *netdev)
{
	struct atl2_adapter *adapter = netdev_priv(netdev);
	int err;
	u32 val;

	/* disallow open during test */
	if (test_bit(__ATL2_TESTING, &adapter->flags))
		return -EBUSY;

	/* allocate transmit descriptors */
	if((err = atl2_setup_ring_resources(adapter)))
		return err;

	if((err = atl2_init_hw(&adapter->hw))) {
		err = -EIO;
		goto err_init_hw;
	}

	/* hardware has been reset, we need to reload some things */
	atl2_set_multi(netdev);
	init_ring_ptrs(adapter);

#ifdef NETIF_F_HW_VLAN_TX
	atl2_restore_vlan(adapter);
#endif

	if (atl2_configure(adapter)) {
		err = -EIO;
		goto err_config;
	}

	if ((err = atl2_request_irq(adapter)))
		goto err_req_irq;

	clear_bit(__ATL2_DOWN, &adapter->flags);

	mod_timer(&adapter->watchdog_timer, jiffies + 4*HZ);

	val = ATL2_READ_REG(&adapter->hw, REG_MASTER_CTRL);
	ATL2_WRITE_REG(&adapter->hw, REG_MASTER_CTRL, val | MASTER_CTRL_MANUAL_INT);

	atl2_irq_enable(adapter);

	return 0;

err_init_hw:
err_req_irq:
err_config:
	atl2_free_ring_resources(adapter);
	atl2_reset_hw(&adapter->hw);

	return err;
}

void
atl2_down(struct atl2_adapter *adapter)
{
	struct net_device *netdev = adapter->netdev;

	/* signal that we're down so the interrupt handler does not
	* reschedule our watchdog timer */
	set_bit(__ATL2_DOWN, &adapter->flags);

#ifdef NETIF_F_LLTX
	netif_stop_queue(netdev);
#else
	netif_tx_disable(netdev);
#endif

	/* reset MAC to disable all RX/TX */
	atl2_reset_hw(&adapter->hw);
	msleep(1);

	atl2_irq_disable(adapter);
		
	del_timer_sync(&adapter->watchdog_timer);
	del_timer_sync(&adapter->phy_config_timer);
	clear_bit(0, &adapter->cfg_phy);

	netif_carrier_off(netdev);
	adapter->link_speed = SPEED_0;
	adapter->link_duplex = -1;

//    atl2_reset(adapter);
}

static void
atl2_free_irq(struct atl2_adapter *adapter)
{
	struct net_device *netdev = adapter->netdev;

	free_irq(adapter->pdev->irq, netdev);

#ifdef CONFIG_PCI_MSI
	if (adapter->have_msi)
		pci_disable_msi(adapter->pdev);
#endif
}

/**
 * atl2_close - Disables a network interface
 * @netdev: network interface device structure
 *
 * Returns 0, this is not allowed to fail
 *
 * The close entry point is called when an interface is de-activated
 * by the OS.  The hardware is still under the drivers control, but
 * needs to be disabled.  A global MAC reset is issued to stop the
 * hardware, and all transmit and receive resources are freed.
 **/
static int
atl2_close(struct net_device *netdev)
{
	struct atl2_adapter *adapter = netdev_priv(netdev);

	WARN_ON(test_bit(__ATL2_RESETTING, &adapter->flags));

	atl2_down(adapter);
	atl2_free_irq(adapter);
	atl2_free_ring_resources(adapter);

	return 0;
}

static inline int
TxsFreeUnit(struct atl2_adapter *adapter)
{
	u32 txs_write_ptr = (u32) atomic_read(&adapter->txs_write_ptr);

	return (adapter->txs_next_clear >= txs_write_ptr) ?
		(int) (adapter->txs_ring_size - adapter->txs_next_clear +
		txs_write_ptr - 1) :
		(int) (txs_write_ptr - adapter->txs_next_clear - 1);
}

static inline int
TxdFreeBytes(struct atl2_adapter *adapter)
{
	u32 txd_read_ptr = (u32)atomic_read(&adapter->txd_read_ptr);

	return (adapter->txd_write_ptr >= txd_read_ptr) ?
		(int) (adapter->txd_ring_size - adapter->txd_write_ptr +
		txd_read_ptr - 1):
		(int) (txd_read_ptr - adapter->txd_write_ptr - 1);
}

static int
atl2_xmit_frame(struct sk_buff *skb, struct net_device *netdev)
{
	struct atl2_adapter *adapter = netdev_priv(netdev);
	unsigned long flags;
	tx_pkt_header_t* txph;
	u32 offset, copy_len;
	int txs_unused;
	int txbuf_unused;

	if (test_bit(__ATL2_DOWN, &adapter->flags)) {
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	if (unlikely(skb->len <= 0)) {
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

#ifdef NETIF_F_LLTX
	local_irq_save(flags);
	if (!spin_trylock(&adapter->tx_lock)) {
		/* Collision - tell upper layer to requeue */
		local_irq_restore(flags);
		return NETDEV_TX_LOCKED;
	}
#else
	spin_lock_irqsave(&adapter->tx_lock, flags);
#endif
	txs_unused = TxsFreeUnit(adapter);
	txbuf_unused = TxdFreeBytes(adapter);

	if (txs_unused < 1 || skb->len > txbuf_unused) {
		// no enough resource
		netif_stop_queue(netdev);
		spin_unlock_irqrestore(&adapter->tx_lock, flags);
		return NETDEV_TX_BUSY;
	}

	offset = adapter->txd_write_ptr;

	txph = (tx_pkt_header_t*) (((u8*)adapter->txd_ring)+offset);

	*(u32*)txph = 0;
	txph->pkt_size = skb->len;

	offset += 4;
	if (offset >= adapter->txd_ring_size)
		offset -= adapter->txd_ring_size;
	copy_len = adapter->txd_ring_size - offset;
	if (copy_len >= skb->len) {
		memcpy(((u8*)adapter->txd_ring)+offset, skb->data, skb->len);
		offset += ((u32)(skb->len+3)&~3);
	} else {
		memcpy(((u8*)adapter->txd_ring)+offset, skb->data, copy_len);
		memcpy((u8*)adapter->txd_ring, skb->data+copy_len, skb->len-copy_len);
		offset = ((u32)(skb->len-copy_len+3)&~3);
	}
#ifdef NETIF_F_HW_VLAN_TX
	if (adapter->vlgrp && vlan_tx_tag_present(skb)) {
		u16 vlan_tag = vlan_tx_tag_get(skb);
		vlan_tag = (vlan_tag << 4) |
			(vlan_tag >> 13) |
			((vlan_tag >>9) & 0x8);
		txph->ins_vlan = 1;
		txph->vlan = vlan_tag;
	}
#endif
	if (offset >= adapter->txd_ring_size)
		offset -= adapter->txd_ring_size;
	adapter->txd_write_ptr = offset;

	// clear txs before send
	adapter->txs_ring[adapter->txs_next_clear].update = 0;
	if (++adapter->txs_next_clear == adapter->txs_ring_size)
		adapter->txs_next_clear = 0;

	ATL2_WRITE_REGW(&adapter->hw, REG_MB_TXD_WR_IDX, (adapter->txd_write_ptr >> 2));

	spin_unlock_irqrestore(&adapter->tx_lock, flags);

	netdev->trans_start = jiffies;
	dev_kfree_skb_any(skb);
	return NETDEV_TX_OK;
}

/**
 * atl2_get_stats - Get System Network Statistics
 * @netdev: network interface device structure
 *
 * Returns the address of the device statistics structure.
 * The statistics are actually updated from the timer callback.
 **/
static struct net_device_stats *
atl2_get_stats(struct net_device *netdev)
{
	struct atl2_adapter *adapter = netdev_priv(netdev);

	return &adapter->net_stats;
}

/**
 * atl2_change_mtu - Change the Maximum Transfer Unit
 * @netdev: network interface device structure
 * @new_mtu: new value for maximum frame size
 *
 * Returns 0 on success, negative on failure
 **/
static int
atl2_change_mtu(struct net_device *netdev, int new_mtu)
{
	struct atl2_adapter *adapter = netdev_priv(netdev);
	struct atl2_hw *hw = &adapter->hw;

	if ((new_mtu < 40) || (new_mtu > (ETH_DATA_LEN + VLAN_SIZE)))
		return -EINVAL;

	/* set MTU */
	if (hw->max_frame_size != new_mtu) {
//		while (test_and_set_bit(__ATL2_RESETTING, &adapter->flags))
//			msleep(1);
		netdev->mtu = new_mtu;

		ATL2_WRITE_REG(hw, REG_MTU,
			new_mtu + ENET_HEADER_SIZE + VLAN_SIZE + ETHERNET_FCS_SIZE);
//		clear_bit(__ATL2_RESETTING, &adapter->flags);
	}

	return 0;
}

/**
 * atl2_set_mac - Change the Ethernet Address of the NIC
 * @netdev: network interface device structure
 * @p: pointer to an address structure
 *
 * Returns 0 on success, negative on failure
 **/
static int
atl2_set_mac(struct net_device *netdev, void *p)
{
	struct atl2_adapter *adapter = netdev_priv(netdev);
	struct sockaddr *addr = p;

	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	if (netif_running(netdev))
		return -EBUSY;

	memcpy(netdev->dev_addr, addr->sa_data, netdev->addr_len);
	memcpy(adapter->hw.mac_addr, addr->sa_data, netdev->addr_len);

	atl2_set_mac_addr(&adapter->hw);

	return 0;
}

/**
 * atl2_mii_ioctl -
 * @netdev:
 * @ifreq:
 * @cmd:
 **/
static int
atl2_mii_ioctl(struct net_device *netdev, struct ifreq *ifr, int cmd)
{
	struct atl2_adapter *adapter = netdev_priv(netdev);
	struct mii_ioctl_data *data = if_mii(ifr);
	unsigned long flags;

	switch (cmd) {
		case SIOCGMIIPHY:
			data->phy_id = 0;
			break;
		case SIOCGMIIREG:
			if (!capable(CAP_NET_ADMIN))
				return -EPERM;
			spin_lock_irqsave(&adapter->stats_lock, flags);
			if (atl2_read_phy_reg(&adapter->hw, data->reg_num & 0x1F, &data->val_out)) {
				spin_unlock_irqrestore(&adapter->stats_lock, flags);
				return -EIO;
			}
			spin_unlock_irqrestore(&adapter->stats_lock, flags);
			break;
		case SIOCSMIIREG:
			if (!capable(CAP_NET_ADMIN))
				return -EPERM;
			if (data->reg_num & ~(0x1F))
				return -EFAULT;
			spin_lock_irqsave(&adapter->stats_lock, flags);
			if (atl2_write_phy_reg(&adapter->hw, data->reg_num, data->val_in)) {
				spin_unlock_irqrestore(&adapter->stats_lock, flags);
				return -EIO;
			}
			spin_unlock_irqrestore(&adapter->stats_lock, flags);
			break;
		default:
			return -EOPNOTSUPP;
	}
	return ATL2_SUCCESS;
}

/**
 * atl2_ioctl -
 * @netdev:
 * @ifreq:
 * @cmd:
 **/
static int
atl2_ioctl(struct net_device *netdev, struct ifreq *ifr, int cmd)
{
	switch (cmd) {
		case SIOCGMIIPHY:
		case SIOCGMIIREG:
		case SIOCSMIIREG:
			return atl2_mii_ioctl(netdev, ifr, cmd);
#ifdef ETHTOOL_OPS_COMPAT
		case SIOCETHTOOL:
			return ethtool_ioctl(ifr);
#endif
		default:
			return -EOPNOTSUPP;
	}
}

/**
 * atl2_tx_timeout - Respond to a Tx Hang
 * @netdev: network interface device structure
 **/
static void
atl2_tx_timeout(struct net_device *netdev)
{
	struct atl2_adapter *adapter = netdev_priv(netdev);

	/* Do the reset outside of interrupt context */
	schedule_work(&adapter->reset_task);
}

/**
 * atl2_watchdog - Timer Call-back
 * @data: pointer to netdev cast into an unsigned long
 **/
static void
atl2_watchdog(unsigned long data)
{
	struct atl2_adapter *adapter = (struct atl2_adapter *) data;
	u32 drop_rxd, drop_rxs;
	unsigned long flags;

	if (!test_bit(__ATL2_DOWN, &adapter->flags)) {
		spin_lock_irqsave(&adapter->stats_lock, flags);
		drop_rxd = ATL2_READ_REG(&adapter->hw, REG_STS_RXD_OV);
		drop_rxs = ATL2_READ_REG(&adapter->hw, REG_STS_RXS_OV);
		adapter->net_stats.rx_over_errors += (drop_rxd+drop_rxs);
		spin_unlock_irqrestore(&adapter->stats_lock, flags);

		/* Reset the timer */
		mod_timer(&adapter->watchdog_timer, jiffies + 4 * HZ);
	}
}

/**
 * atl2_phy_config - Timer Call-back
 * @data: pointer to netdev cast into an unsigned long
 **/
static void
atl2_phy_config(unsigned long data)
{
	struct atl2_adapter *adapter = (struct atl2_adapter *) data;
	struct atl2_hw *hw = &adapter->hw;
	unsigned long flags;

	spin_lock_irqsave(&adapter->stats_lock, flags);
	atl2_write_phy_reg(hw, MII_ADVERTISE, hw->mii_autoneg_adv_reg);
	atl2_write_phy_reg(hw, MII_BMCR, MII_CR_RESET|MII_CR_AUTO_NEG_EN|MII_CR_RESTART_AUTO_NEG);
	spin_unlock_irqrestore(&adapter->stats_lock, flags);
	clear_bit(0, &adapter->cfg_phy);
}

int
atl2_up(struct atl2_adapter *adapter)
{
	struct net_device *netdev = adapter->netdev;
	int err = 0;
	u32 val;

	/* hardware has been reset, we need to reload some things */

	err = atl2_init_hw(&adapter->hw);
	if (err) {
		err = -EIO;
		return err;
	}

	atl2_set_multi(netdev);
	init_ring_ptrs(adapter);

#ifdef NETIF_F_HW_VLAN_TX
	atl2_restore_vlan(adapter);
#endif

	if (atl2_configure(adapter)) {
		err = -EIO;
		goto err_up;
	}

	clear_bit(__ATL2_DOWN, &adapter->flags);

	val = ATL2_READ_REG(&adapter->hw, REG_MASTER_CTRL);
	ATL2_WRITE_REG(&adapter->hw, REG_MASTER_CTRL, val | MASTER_CTRL_MANUAL_INT);

	atl2_irq_enable(adapter);

err_up:
	return err;
}

void
atl2_reinit_locked(struct atl2_adapter *adapter)
{
	WARN_ON(in_interrupt());
	while (test_and_set_bit(__ATL2_RESETTING, &adapter->flags))
		msleep(1);
	atl2_down(adapter);
	atl2_up(adapter);
	clear_bit(__ATL2_RESETTING, &adapter->flags);
}

static void
atl2_reset_task(struct work_struct *work)
{
	struct atl2_adapter *adapter;
	adapter = container_of(work, struct atl2_adapter, reset_task);

	atl2_reinit_locked(adapter);
}

static inline void
atl2_setup_mac_ctrl(struct atl2_adapter *adapter)
{
	u32 value;
	struct atl2_hw* hw = &adapter->hw;
	struct net_device* netdev = adapter->netdev;

	/* Config MAC CTRL Register */
	value = MAC_CTRL_TX_EN | MAC_CTRL_RX_EN | MAC_CTRL_MACLP_CLK_PHY;

	// duplex
	if (FULL_DUPLEX == adapter->link_duplex)
		value |= MAC_CTRL_DUPLX;

	// flow control
	value |= (MAC_CTRL_TX_FLOW | MAC_CTRL_RX_FLOW);

	// PAD & CRC
	value |= (MAC_CTRL_ADD_CRC | MAC_CTRL_PAD);

	// preamble length
	value |= (((u32)adapter->hw.preamble_len & MAC_CTRL_PRMLEN_MASK) <<
		MAC_CTRL_PRMLEN_SHIFT);
	
	// vlan
	if (adapter->vlgrp)
		value |= MAC_CTRL_RMV_VLAN;

	// filter mode
	value |= MAC_CTRL_BC_EN;
	if (netdev->flags & IFF_PROMISC)
		value |= MAC_CTRL_PROMIS_EN;
	else if (netdev->flags & IFF_ALLMULTI)
		value |= MAC_CTRL_MC_ALL_EN;

	// half retry buffer
	value |= (((u32)(adapter->hw.retry_buf & MAC_CTRL_HALF_LEFT_BUF_MASK)) <<
		MAC_CTRL_HALF_LEFT_BUF_SHIFT);

	ATL2_WRITE_REG(hw, REG_MAC_CTRL, value);
}

static int
atl2_check_link(struct atl2_adapter *adapter)
{
	struct atl2_hw *hw = &adapter->hw;
	struct net_device * netdev = adapter->netdev;
	int ret_val;
	u16 speed, duplex, phy_data;
	int reconfig = 0;

	// MII_BMSR must read twise
	atl2_read_phy_reg(hw, MII_BMSR, &phy_data);
	atl2_read_phy_reg(hw, MII_BMSR, &phy_data);
	if (!(phy_data&BMSR_LSTATUS)) { // link down
		if (netif_carrier_ok(netdev)) { // old link state: Up
			u32 value;
			//disable rx
			value = ATL2_READ_REG(hw, REG_MAC_CTRL);
			value &= ~MAC_CTRL_RX_EN;
			ATL2_WRITE_REG(hw, REG_MAC_CTRL, value);
			adapter->link_speed = SPEED_0;
			netif_carrier_off(netdev);
			netif_stop_queue(netdev);
		}
		return ATL2_SUCCESS;
	}

	// Link Up
	ret_val = atl2_get_speed_and_duplex(hw, &speed, &duplex);
	if (ret_val)
		return ret_val;
	switch( hw->MediaType ) {
		case MEDIA_TYPE_100M_FULL:
			if (speed  != SPEED_100 || duplex != FULL_DUPLEX)
				reconfig = 1;
			break;
		case MEDIA_TYPE_100M_HALF:
			if (speed  != SPEED_100 || duplex != HALF_DUPLEX)
				reconfig = 1;
			break;
		case MEDIA_TYPE_10M_FULL:
			if (speed != SPEED_10 || duplex != FULL_DUPLEX)
				reconfig = 1;
			break;
		case MEDIA_TYPE_10M_HALF:
			if (speed  != SPEED_10 || duplex != HALF_DUPLEX)
				reconfig = 1;
			break;
	}
	// link result is our setting
	if (0 == reconfig) {
		if (adapter->link_speed != speed || adapter->link_duplex != duplex ) {
			adapter->link_speed = speed;
			adapter->link_duplex = duplex;
			atl2_setup_mac_ctrl(adapter); 
			printk(KERN_INFO "%s: %s NIC Link is Up<%d Mbps %s>\n",
				atl2_driver_name, netdev->name,
				adapter->link_speed,
				adapter->link_duplex == FULL_DUPLEX ?
					"Full Duplex" : "Half Duplex");
		}

		if (!netif_carrier_ok(netdev)) { // Link down -> Up
			netif_carrier_on(netdev);
			netif_wake_queue(netdev);
		}
		return ATL2_SUCCESS;
	}

	// change orignal link status
	if (netif_carrier_ok(netdev)) {
		u32 value;
		// disable rx
		value = ATL2_READ_REG(hw, REG_MAC_CTRL);
		value &= ~MAC_CTRL_RX_EN;
		ATL2_WRITE_REG(hw, REG_MAC_CTRL, value);

		adapter->link_speed = SPEED_0;
		netif_carrier_off(netdev);
		netif_stop_queue(netdev);
	}

	// auto-neg, insert timer to re-config phy (if interval smaller than 5 seconds, something strange)
	if (!test_bit(__ATL2_DOWN, &adapter->flags)) {
		if (!test_and_set_bit(0, &adapter->cfg_phy)) {
			mod_timer(&adapter->phy_config_timer, jiffies + 5 * HZ);
		}
	}

	return ATL2_SUCCESS;
}

/**
 * atl2_link_chg_task - deal with link change event Out of interrupt context
 * @netdev: network interface device structure
 **/
static void
atl2_link_chg_task(struct work_struct *work)
{
	struct atl2_adapter *adapter;
	unsigned long flags;

	adapter = container_of(work, struct atl2_adapter, link_chg_task);

	spin_lock_irqsave(&adapter->stats_lock, flags);
	atl2_check_link(adapter);
	spin_unlock_irqrestore(&adapter->stats_lock, flags);
}

static void
atl2_setup_pcicmd(struct pci_dev *pdev)
{
	u16 cmd;

	pci_read_config_word(pdev, PCI_COMMAND, &cmd);
	
	if (cmd & PCI_COMMAND_INTX_DISABLE)
		cmd &= ~PCI_COMMAND_INTX_DISABLE;
	if (cmd & PCI_COMMAND_IO)
		cmd &= ~PCI_COMMAND_IO;
	if (0 == (cmd & PCI_COMMAND_MEMORY))
		cmd |= PCI_COMMAND_MEMORY;
	if (0 == (cmd & PCI_COMMAND_MASTER))
		cmd |= PCI_COMMAND_MASTER;
	pci_write_config_word(pdev, PCI_COMMAND, cmd);
	
	/*
	 * some motherboards BIOS(PXE/EFI) driver may set PME
	 * while they transfer control to OS (Windows/Linux)
	 * so we should clear this bit before NIC work normally
	 */
	pci_write_config_dword(pdev, REG_PM_CTRLSTAT, 0);
}

/**
 * atl2_probe - Device Initialization Routine
 * @pdev: PCI device information struct
 * @ent: entry in atl2_pci_tbl
 *
 * Returns 0 on success, negative on failure
 *
 * atl2_probe initializes an adapter identified by a pci_dev structure.
 * The OS initialization, configuring of the adapter private structure,
 * and a hardware reset occur.
 **/
static int __devinit
atl2_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	struct net_device *netdev;
	struct atl2_adapter *adapter;
	static int cards_found = 0;
	unsigned long mmio_start;
	int mmio_len;
	int err;

	if((err = pci_enable_device(pdev)))
		return err;

	/*
	 * atl2 is a shared-high-32-bit device, so we're stuck with 32-bit DMA
	 * until the kernel has the proper infrastructure to support 64-bit DMA
	 * on these devices.
	 */
	if ((err = pci_set_dma_mask(pdev, DMA_32BIT_MASK)) &&
		(err = pci_set_consistent_dma_mask(pdev, DMA_32BIT_MASK))) {
		printk(KERN_ERR "atl2: No usable DMA configuration, aborting\n");
		goto err_dma;
	}

	// Mark all PCI regions associated with PCI device
	// pdev as being reserved by owner atl2_driver_name
	if((err = pci_request_regions(pdev, atl2_driver_name)))
		goto err_pci_reg;

	// Enables bus-mastering on the device and calls
	// pcibios_set_master to do the needed arch specific settings
	pci_set_master(pdev);

	err = -ENOMEM;
	netdev = alloc_etherdev(sizeof(struct atl2_adapter));
	if(!netdev)
		goto err_alloc_etherdev;

	SET_NETDEV_DEV(netdev, &pdev->dev);

	pci_set_drvdata(pdev, netdev);
	adapter = netdev_priv(netdev);
	adapter->netdev = netdev;
	adapter->pdev = pdev;
	adapter->hw.back = adapter;

	mmio_start = pci_resource_start(pdev, 0x0);
	mmio_len = pci_resource_len(pdev, 0x0);

	adapter->hw.mem_rang = (u32)mmio_len;
	adapter->hw.hw_addr = ioremap(mmio_start, mmio_len);
	if(!adapter->hw.hw_addr) {
		err = -EIO;
		goto err_ioremap;
	}

	atl2_setup_pcicmd(pdev);

	netdev->open = &atl2_open;
	netdev->stop = &atl2_close;
	netdev->hard_start_xmit = &atl2_xmit_frame;
	netdev->get_stats = &atl2_get_stats;
	netdev->set_multicast_list = &atl2_set_multi;
	netdev->set_mac_address = &atl2_set_mac;
	netdev->change_mtu = &atl2_change_mtu;
	netdev->do_ioctl = &atl2_ioctl;
	atl2_set_ethtool_ops(netdev);

#ifdef HAVE_TX_TIMEOUT
	netdev->tx_timeout = &atl2_tx_timeout;
	netdev->watchdog_timeo = 5 * HZ; //FIXME -- CHS
#endif
#ifdef NETIF_F_HW_VLAN_TX
	netdev->vlan_rx_register = atl2_vlan_rx_register;
#endif
	strncpy(netdev->name, pci_name(pdev), sizeof(netdev->name) - 1);

	netdev->mem_start = mmio_start;
	netdev->mem_end = mmio_start + mmio_len;
	//netdev->base_addr = adapter->io_base;
	adapter->bd_number = cards_found;
	adapter->pci_using_64 = false;

	/* setup the private structure */

	if((err = atl2_sw_init(adapter)))
		goto err_sw_init;

	err = -EIO;

#ifdef NETIF_F_HW_VLAN_TX
	netdev->features |= (NETIF_F_HW_VLAN_TX | NETIF_F_HW_VLAN_RX );
#endif

#ifdef NETIF_F_LLTX
	netdev->features |= NETIF_F_LLTX;
#endif

	/* Init PHY as early as possible due to power saving issue  */
	atl2_phy_init(&adapter->hw);

	/* reset the controller to
	* put the device in a known good starting state */

	if (atl2_reset_hw(&adapter->hw)) {
		err = -EIO;
		goto err_reset;
	}

	/* copy the MAC address out of the EEPROM */
	atl2_read_mac_addr(&adapter->hw);
	memcpy(netdev->dev_addr, adapter->hw.mac_addr, netdev->addr_len);
//FIXME: do we still need this?
#ifdef ETHTOOL_GPERMADDR
	memcpy(netdev->perm_addr, adapter->hw.mac_addr, netdev->addr_len);

	if (!is_valid_ether_addr(netdev->perm_addr)) {
#else
	if (!is_valid_ether_addr(netdev->dev_addr)) {
#endif
		err = -EIO;
		goto err_eeprom;
	}

	atl2_check_options(adapter);

	init_timer(&adapter->watchdog_timer);
	adapter->watchdog_timer.function = &atl2_watchdog;
	adapter->watchdog_timer.data = (unsigned long) adapter;

	init_timer(&adapter->phy_config_timer);
	adapter->phy_config_timer.function = &atl2_phy_config;
	adapter->phy_config_timer.data = (unsigned long) adapter;
	
	INIT_WORK(&adapter->reset_task, atl2_reset_task);
	INIT_WORK(&adapter->link_chg_task, atl2_link_chg_task);

	strcpy(netdev->name, "eth%d"); // ??
	if((err = register_netdev(netdev)))
		goto err_register;

	/* assume we have no link for now */
	netif_carrier_off(netdev);
	netif_stop_queue(netdev);

	cards_found++;

	return 0;

//err_init_hw:
err_reset:
err_register:
err_sw_init:
err_eeprom:
	iounmap(adapter->hw.hw_addr);
err_ioremap:
	free_netdev(netdev);
err_alloc_etherdev:
	pci_release_regions(pdev);
err_pci_reg:
err_dma:
	pci_disable_device(pdev);
	return err;
}

/**
 * atl2_remove - Device Removal Routine
 * @pdev: PCI device information struct
 *
 * atl2_remove is called by the PCI subsystem to alert the driver
 * that it should release a PCI device.  The could be caused by a
 * Hot-Plug event, or because the driver is going to be removed from
 * memory.
 **/
/* FIXME: write the original MAC address back in case it was changed from a
 * BIOS-set value, as in atl1 -- CHS */
static void __devexit
atl2_remove(struct pci_dev *pdev)
{
	struct net_device *netdev = pci_get_drvdata(pdev);
	struct atl2_adapter *adapter = netdev_priv(netdev);

	/* flush_scheduled work may reschedule our watchdog task, so
	 * explicitly disable watchdog tasks from being rescheduled  */
	set_bit(__ATL2_DOWN, &adapter->flags);

	del_timer_sync(&adapter->watchdog_timer);
	del_timer_sync(&adapter->phy_config_timer);

	flush_scheduled_work();

	unregister_netdev(netdev);

	atl2_force_ps(&adapter->hw);

	iounmap(adapter->hw.hw_addr);
	pci_release_regions(pdev);

	free_netdev(netdev);

	pci_disable_device(pdev);
}

static int
atl2_suspend(struct pci_dev *pdev, pm_message_t state)
{
	struct net_device *netdev = pci_get_drvdata(pdev);
	struct atl2_adapter *adapter = netdev_priv(netdev);
	struct atl2_hw * hw = &adapter->hw;
	u16 speed, duplex;
	u32 ctrl = 0;
	u32 wufc = adapter->wol;

#ifdef CONFIG_PM
    int retval = 0;
#endif

	netif_device_detach(netdev);

	if (netif_running(netdev)) {
		WARN_ON(test_bit(__ATL2_RESETTING, &adapter->flags));
		atl2_down(adapter);
	}

#ifdef CONFIG_PM
	retval = pci_save_state(pdev);
	if (retval)
		return retval;
#endif

	atl2_read_phy_reg(hw, MII_BMSR, (u16*)&ctrl);
	atl2_read_phy_reg(hw, MII_BMSR, (u16*)&ctrl);
	if(ctrl & BMSR_LSTATUS)
		wufc &= ~ATL2_WUFC_LNKC;

	if (0 != (ctrl & BMSR_LSTATUS) && 0 != wufc) {
		u32 ret_val;
		/* get current link speed & duplex */
		ret_val = atl2_get_speed_and_duplex(hw, &speed, &duplex);
		if (ret_val) {
			printk(KERN_DEBUG "%s: get speed&duplex error while suspend\n", atl2_driver_name);
			goto wol_dis;
		}

		ctrl = 0;

		/* turn on magic packet wol */
		if (wufc & ATL2_WUFC_MAG)
			ctrl |= (WOL_MAGIC_EN | WOL_MAGIC_PME_EN);

		/* ignore Link Chg event when Link is up */
		ATL2_WRITE_REG(hw, REG_WOL_CTRL, ctrl);

		/* Config MAC CTRL Register */
		ctrl = MAC_CTRL_RX_EN | MAC_CTRL_MACLP_CLK_PHY;
		if (FULL_DUPLEX == adapter->link_duplex)
			ctrl |= MAC_CTRL_DUPLX;
		ctrl |= (MAC_CTRL_ADD_CRC | MAC_CTRL_PAD);
		ctrl |= (((u32)adapter->hw.preamble_len &
			MAC_CTRL_PRMLEN_MASK) << MAC_CTRL_PRMLEN_SHIFT);
		ctrl |= (((u32)(adapter->hw.retry_buf &
			MAC_CTRL_HALF_LEFT_BUF_MASK)) <<
			MAC_CTRL_HALF_LEFT_BUF_SHIFT);
		if (wufc & ATL2_WUFC_MAG) {
			/* magic packet maybe Broadcast&multicast&Unicast frame */
			ctrl |= MAC_CTRL_BC_EN;
		}

		ATL2_WRITE_REG(hw, REG_MAC_CTRL, ctrl);

		/* pcie patch */
		ctrl = ATL2_READ_REG(hw, REG_PCIE_PHYMISC);
		ctrl |= PCIE_PHYMISC_FORCE_RCV_DET;
		ATL2_WRITE_REG(hw, REG_PCIE_PHYMISC, ctrl);
		ctrl = ATL2_READ_REG(hw, REG_PCIE_DLL_TX_CTRL1);
		ctrl |= PCIE_DLL_TX_CTRL1_SEL_NOR_CLK;
		ATL2_WRITE_REG(hw, REG_PCIE_DLL_TX_CTRL1, ctrl);

		pci_enable_wake(pdev, pci_choose_state(pdev, state), 1);
		goto suspend_exit;
	}

	if (0 == (ctrl&BMSR_LSTATUS) && 0 != (wufc&ATL2_WUFC_LNKC)) {
		/* link is down, so only LINK CHG WOL event enable */
		ctrl |= (WOL_LINK_CHG_EN | WOL_LINK_CHG_PME_EN);
		ATL2_WRITE_REG(hw, REG_WOL_CTRL, ctrl);
		ATL2_WRITE_REG(hw, REG_MAC_CTRL, 0);

		/* pcie patch */
		ctrl = ATL2_READ_REG(hw, REG_PCIE_PHYMISC);
		ctrl |= PCIE_PHYMISC_FORCE_RCV_DET;
		ATL2_WRITE_REG(hw, REG_PCIE_PHYMISC, ctrl);
		ctrl = ATL2_READ_REG(hw, REG_PCIE_DLL_TX_CTRL1);
		ctrl |= PCIE_DLL_TX_CTRL1_SEL_NOR_CLK;
		ATL2_WRITE_REG(hw, REG_PCIE_DLL_TX_CTRL1, ctrl);

		hw->phy_configured = false; /* re-init PHY when resume */

		pci_enable_wake(pdev, pci_choose_state(pdev, state), 1);

		goto suspend_exit;
	}

wol_dis:
	/* WOL disabled */
	ATL2_WRITE_REG(hw, REG_WOL_CTRL, 0);

	/* pcie patch */
	ctrl = ATL2_READ_REG(hw, REG_PCIE_PHYMISC);
	ctrl |= PCIE_PHYMISC_FORCE_RCV_DET;
	ATL2_WRITE_REG(hw, REG_PCIE_PHYMISC, ctrl);
	ctrl = ATL2_READ_REG(hw, REG_PCIE_DLL_TX_CTRL1);
	ctrl |= PCIE_DLL_TX_CTRL1_SEL_NOR_CLK;
	ATL2_WRITE_REG(hw, REG_PCIE_DLL_TX_CTRL1, ctrl);

	atl2_force_ps(hw);
	hw->phy_configured = false; /* re-init PHY when resume */

	pci_enable_wake(pdev, pci_choose_state(pdev, state), 0);

suspend_exit:
	if (netif_running(netdev))
		atl2_free_irq(adapter);

	pci_disable_device(pdev);

	pci_set_power_state(pdev, pci_choose_state(pdev, state));

	return 0;
}

#ifdef CONFIG_PM
static int
atl2_resume(struct pci_dev *pdev)
{
	struct net_device *netdev = pci_get_drvdata(pdev);
	struct atl2_adapter *adapter = netdev_priv(netdev);
	u32 err;

	pci_set_power_state(pdev, PCI_D0);
	pci_restore_state(pdev);
	
	if ((err = pci_enable_device(pdev))) {
		printk(KERN_ERR "atl2: Cannot enable PCI device from suspend\n");
		return err;
	}

	pci_set_master(pdev);

	ATL2_READ_REG(&adapter->hw, REG_WOL_CTRL); /* clear WOL status */

	pci_enable_wake(pdev, PCI_D3hot, 0);
	pci_enable_wake(pdev, PCI_D3cold, 0);

	ATL2_WRITE_REG(&adapter->hw, REG_WOL_CTRL, 0);
	
	if (netif_running(netdev) && (err = atl2_request_irq(adapter)))
		return err;

	atl2_reset_hw(&adapter->hw);

	if(netif_running(netdev))
		atl2_up(adapter);

	netif_device_attach(netdev);

	return 0;
}
#endif

static void
atl2_shutdown(struct pci_dev *pdev)
{
	atl2_suspend(pdev, PMSG_SUSPEND);
}

static struct pci_driver atl2_driver = {
	.name     = atl2_driver_name,
	.id_table = atl2_pci_tbl,
	.probe    = atl2_probe,
	.remove   = __devexit_p(atl2_remove),
	/* Power Managment Hooks */
	.suspend  = atl2_suspend,
#ifdef CONFIG_PM
	.resume   = atl2_resume,
#endif
	.shutdown = atl2_shutdown,
};

/**
 * atl2_init_module - Driver Registration Routine
 *
 * atl2_init_module is the first routine called when the driver is
 * loaded. All it does is register with the PCI subsystem.
 **/
static int __init
atl2_init_module(void)
{
	int ret;
	printk(KERN_INFO "%s - version %s\n", atl2_driver_string, atl2_driver_version);
	printk(KERN_INFO "%s\n", atl2_copyright);

	ret = pci_register_driver(&atl2_driver);
	if (copybreak != COPYBREAK_DEFAULT) {
		if (copybreak == 0)
			printk(KERN_INFO "atl2: copybreak disabled\n");
		else
			printk(KERN_INFO "atl2: copybreak enabled for packets <= %u bytes\n", copybreak);
	}
	return ret;
}
module_init(atl2_init_module);

/**
 * atl2_exit_module - Driver Exit Cleanup Routine
 *
 * atl2_exit_module is called just before the driver is removed
 * from memory.
 **/
static void __exit
atl2_exit_module(void)
{
	pci_unregister_driver(&atl2_driver);
}
module_exit(atl2_exit_module);

void
atl2_read_pci_cfg(struct atl2_hw *hw, u32 reg, u16 *value)
{
	struct atl2_adapter *adapter = hw->back;
	pci_read_config_word(adapter->pdev, reg, value);
}

void
atl2_write_pci_cfg(struct atl2_hw *hw, u32 reg, u16 *value)
{
	struct atl2_adapter *adapter = hw->back;
	pci_write_config_word(adapter->pdev, reg, *value);
}
