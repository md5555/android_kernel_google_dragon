/* atl2_hw.h -- atl2 hardware definitions
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
 *
 * Some of these defines are unused for various reasons.  Some describe
 * hardware features we don't yet use.  Some are specific to the cousin atl1
 * hardware, which we may merge this driver with in the future.  Please
 * remember this is a surrogate for hardware specs, and don't unnecessarily
 * abuse the content or formatting. -- CHS
 */

#ifndef _ATL2_HW_H_
#define _ATL2_HW_H_

#include "atl2_osdep.h"

struct atl2_adapter;
struct atl2_hw;

/* function prototype */
s32 atl2_reset_hw(struct atl2_hw *hw);
s32 atl2_read_mac_addr(struct atl2_hw *hw);
s32 atl2_init_hw(struct atl2_hw *hw);
s32 atl2_get_speed_and_duplex(struct atl2_hw *hw, u16 *speed, u16 *duplex);
u32 atl2_auto_get_fc(struct atl2_adapter *adapter, u16 duplex);
u32 atl2_hash_mc_addr(struct atl2_hw *hw, u8 *mc_addr);
void atl2_hash_set(struct atl2_hw *hw, u32 hash_value);
s32 atl2_read_phy_reg(struct atl2_hw *hw, u16 reg_addr, u16 *phy_data);
s32 atl2_write_phy_reg(struct atl2_hw *hw, u32 reg_addr, u16 phy_data);
void atl2_read_pci_cfg(struct atl2_hw *hw, u32 reg, u16 *value);
void atl2_write_pci_cfg(struct atl2_hw *hw, u32 reg, u16 *value);
s32 atl2_validate_mdi_setting(struct atl2_hw *hw);
void atl2_set_mac_addr(struct atl2_hw *hw);
bool atl2_read_eeprom(struct atl2_hw *hw, u32 Offset, u32 *pValue);
bool atl2_write_eeprom(struct atl2_hw *hw, u32 offset, u32 value);
s32 atl2_phy_init(struct atl2_hw *hw);
int atl2_check_eeprom_exist(struct atl2_hw *hw);
void atl2_force_ps(struct atl2_hw *hw);

/* register definition */
#define REG_PM_CTRLSTAT	0x44

#define REG_PCIE_CAP_LIST	0x58

#define REG_VPD_CAP			0x6C
#define     VPD_CAP_ID_MASK		0xff
#define     VPD_CAP_ID_SHIFT		0
#define     VPD_CAP_NEXT_PTR_MASK	0xFF
#define     VPD_CAP_NEXT_PTR_SHIFT	8
#define     VPD_CAP_VPD_ADDR_MASK	0x7FFF
#define     VPD_CAP_VPD_ADDR_SHIFT	16
#define     VPD_CAP_VPD_FLAG		0x80000000

#define REG_VPD_DATA	0x70

#define REG_SPI_FLASH_CTRL			0x200
#define     SPI_FLASH_CTRL_STS_NON_RDY		0x1
#define     SPI_FLASH_CTRL_STS_WEN		0x2
#define     SPI_FLASH_CTRL_STS_WPEN		0x80
#define     SPI_FLASH_CTRL_DEV_STS_MASK		0xFF
#define     SPI_FLASH_CTRL_DEV_STS_SHIFT	0
#define     SPI_FLASH_CTRL_INS_MASK		0x7
#define     SPI_FLASH_CTRL_INS_SHIFT		8
#define     SPI_FLASH_CTRL_START		0x800
#define     SPI_FLASH_CTRL_EN_VPD		0x2000
#define     SPI_FLASH_CTRL_LDSTART		0x8000
#define     SPI_FLASH_CTRL_CS_HI_MASK		0x3
#define     SPI_FLASH_CTRL_CS_HI_SHIFT		16
#define     SPI_FLASH_CTRL_CS_HOLD_MASK		0x3
#define     SPI_FLASH_CTRL_CS_HOLD_SHIFT	18
#define     SPI_FLASH_CTRL_CLK_LO_MASK		0x3
#define     SPI_FLASH_CTRL_CLK_LO_SHIFT		20
#define     SPI_FLASH_CTRL_CLK_HI_MASK		0x3
#define     SPI_FLASH_CTRL_CLK_HI_SHIFT		22
#define     SPI_FLASH_CTRL_CS_SETUP_MASK	0x3
#define     SPI_FLASH_CTRL_CS_SETUP_SHIFT	24
#define     SPI_FLASH_CTRL_EROM_PGSZ_MASK	0x3
#define     SPI_FLASH_CTRL_EROM_PGSZ_SHIFT	26
#define     SPI_FLASH_CTRL_WAIT_READY		0x10000000

#define REG_SPI_ADDR	0x204

#define REG_SPI_DATA	0x208

#define REG_SPI_FLASH_CONFIG			0x20C
#define     SPI_FLASH_CONFIG_LD_ADDR_MASK	0xFFFFFF
#define     SPI_FLASH_CONFIG_LD_ADDR_SHIFT	0
#define     SPI_FLASH_CONFIG_VPD_ADDR_MASK	0x3
#define     SPI_FLASH_CONFIG_VPD_ADDR_SHIFT	24
#define     SPI_FLASH_CONFIG_LD_EXIST		0x4000000

#define REG_SPI_FLASH_OP_PROGRAM	0x210
#define REG_SPI_FLASH_OP_SC_ERASE	0x211
#define REG_SPI_FLASH_OP_CHIP_ERASE	0x212
#define REG_SPI_FLASH_OP_RDID		0x213
#define REG_SPI_FLASH_OP_WREN		0x214
#define REG_SPI_FLASH_OP_RDSR		0x215
#define REG_SPI_FLASH_OP_WRSR		0x216
#define REG_SPI_FLASH_OP_READ		0x217

#define REG_TWSI_CTRL				0x218
#define     TWSI_CTRL_LD_OFFSET_MASK		0xFF
#define     TWSI_CTRL_LD_OFFSET_SHIFT		0
#define     TWSI_CTRL_LD_SLV_ADDR_MASK		0x7
#define     TWSI_CTRL_LD_SLV_ADDR_SHIFT		8
#define     TWSI_CTRL_SW_LDSTART		0x800
#define     TWSI_CTRL_HW_LDSTART		0x1000
#define     TWSI_CTRL_SMB_SLV_ADDR_MASK		0x0x7F
#define     TWSI_CTRL_SMB_SLV_ADDR_SHIFT	15
#define     TWSI_CTRL_LD_EXIST			0x400000
#define     TWSI_CTRL_READ_FREQ_SEL_MASK	0x3
#define     TWSI_CTRL_READ_FREQ_SEL_SHIFT	23
#define     TWSI_CTRL_FREQ_SEL_100K		0
#define     TWSI_CTRL_FREQ_SEL_200K		1
#define     TWSI_CTRL_FREQ_SEL_300K		2
#define     TWSI_CTRL_FREQ_SEL_400K		3
#define     TWSI_CTRL_SMB_SLV_ADDR
#define     TWSI_CTRL_WRITE_FREQ_SEL_MASK	0x3
#define     TWSI_CTRL_WRITE_FREQ_SEL_SHIFT	24

#define REG_PCIE_DEV_MISC_CTRL			0x21C
#define     PCIE_DEV_MISC_CTRL_EXT_PIPE		0x2
#define     PCIE_DEV_MISC_CTRL_RETRY_BUFDIS	0x1
#define     PCIE_DEV_MISC_CTRL_SPIROM_EXIST	0x4
#define     PCIE_DEV_MISC_CTRL_SERDES_ENDIAN	0x8
#define     PCIE_DEV_MISC_CTRL_SERDES_SEL_DIN	0x10

#define REG_PCIE_PHYMISC		0x1000
#define     PCIE_PHYMISC_FORCE_RCV_DET	0x4

#define REG_PCIE_DLL_TX_CTRL1			0x1104
#define     PCIE_DLL_TX_CTRL1_SEL_NOR_CLK	0x0400
#define     PCIE_DLL_TX_CTRL1_DEF		0x0568

#define REG_LTSSM_TEST_MODE	0x12FC
#define     LTSSM_TEST_MODE_DEF	0x6500

/* Master Control Register */
#define REG_MASTER_CTRL			0x1400
#define     MASTER_CTRL_SOFT_RST	0x1
#define     MASTER_CTRL_MTIMER_EN	0x2
#define     MASTER_CTRL_ITIMER_EN	0x4
#define     MASTER_CTRL_MANUAL_INT	0x8
#define     MASTER_CTRL_REV_NUM_SHIFT	16
#define     MASTER_CTRL_REV_NUM_MASK	0xff
#define     MASTER_CTRL_DEV_ID_SHIFT	24
#define     MASTER_CTRL_DEV_ID_MASK	0xff

/* Timer Initial Value Register */
#define REG_MANUAL_TIMER_INIT	0x1404

/* IRQ ModeratorTimer Initial Value Register */
#define REG_IRQ_MODU_TIMER_INIT	0x1408

#define REG_PHY_ENABLE		0x140C
// IRQ Anti-Lost Timer Initial Value Register
//#define REG_IRQ_CLR_TIMER	0x140c	// Maximum allowance for software to clear the interrupt.
// IRQ Anti-Lost Timer Initial Value Register
#define REG_CMBDISDMA_TIMER	0x140E

/* Block IDLE Status Register */
#define REG_IDLE_STATUS		0x1410
#define     IDLE_STATUS_RXMAC	1	/* 1: RXMAC state machine is in non-IDLE state. 0: RXMAC is idling */
#define     IDLE_STATUS_TXMAC	2	/* 1: TXMAC state machine is in non-IDLE state. 0: TXMAC is idling */
#define     IDLE_STATUS_DMAR	8	/* 1: DMAR state machine is in non-IDLE state.  0: DMAR is idling */
#define     IDLE_STATUS_DMAW	4	/* 1: DMAW state machine is in non-IDLE state.  0: DMAW is idling */

/* MDIO Control Register */
#define REG_MDIO_CTRL		0x1414
#define     MDIO_DATA_MASK	0xffff		/* On MDIO write, the 16-bit control data to write to PHY MII management register */
#define     MDIO_DATA_SHIFT	0		/* On MDIO read, the 16-bit status data that was read from the PHY MII management register. */
#define     MDIO_REG_ADDR_MASK	0x1f		/* MDIO register address */
#define     MDIO_REG_ADDR_SHIFT	16
#define     MDIO_RW		0x200000	/* 1: read, 0: write */
#define     MDIO_SUP_PREAMBLE	0x400000	/* Suppress preamble */
#define     MDIO_START		0x800000	/* Write 1 to initiate the MDIO master. And this bit is self cleared after one cycle. */
#define     MDIO_CLK_SEL_SHIFT	24
#define     MDIO_CLK_25_4	0
#define     MDIO_CLK_25_6	2
#define     MDIO_CLK_25_8	3
#define     MDIO_CLK_25_10	4
#define     MDIO_CLK_25_14	5
#define     MDIO_CLK_25_20	6
#define     MDIO_CLK_25_28	7
#define     MDIO_BUSY		0x8000000
#define MDIO_WAIT_TIMES		10

/* SerDes Lock Detect Control and Status Register */
#define REG_SERDES_LOCK			0x1424
#define     SERDES_LOCK_DETECT		1	/* 1: SerDes lock detected. This signal comes from Analog SerDes. */
#define     SERDES_LOCK_DETECT_EN	2	/* 1: Enable SerDes Lock detect function. */

/* MAC Control Register */
#define REG_MAC_CTRL				0x1480
#define     MAC_CTRL_TX_EN			1		/* 1: Transmit Enable */
#define     MAC_CTRL_RX_EN			2		/* 1: Receive Enable */
#define     MAC_CTRL_TX_FLOW			4		/* 1: Transmit Flow Control Enable */
#define     MAC_CTRL_RX_FLOW			8		/* 1: Receive Flow Control Enable */
#define     MAC_CTRL_LOOPBACK			0x10		/* 1: Loop back at G/MII Interface */
#define     MAC_CTRL_DUPLX			0x20		/* 1: Full-duplex mode  0: Half-duplex mode */
#define     MAC_CTRL_ADD_CRC			0x40		/* 1: Instruct MAC to attach CRC on all egress Ethernet frames */
#define     MAC_CTRL_PAD			0x80		/* 1: Instruct MAC to pad short frames to 60-bytes, and then attach CRC. This bit has higher priority over CRC_EN */
#define     MAC_CTRL_PRMLEN_SHIFT		10		/* Preamble length, it's 0x07 by standard */
#define     MAC_CTRL_PRMLEN_MASK		0xf
#define     MAC_CTRL_RMV_VLAN			0x4000		/* 1: to remove VLAN Tag automatically from all receive packets */
#define     MAC_CTRL_PROMIS_EN			0x8000		/* 1: Promiscuous Mode Enable */
#define     MAC_CTRL_DBG_TX_BKPRESURE		0x100000	/* 1: transmit maximum backoff (half-duplex test bit) */
#define     MAC_CTRL_MC_ALL_EN			0x2000000	/* 1: upload all multicast frame without error to system */
#define     MAC_CTRL_BC_EN			0x4000000	/* 1: upload all broadcast frame without error to system */
#define     MAC_CTRL_MACLP_CLK_PHY		0x8000000	/* 1: MAC-LoopBack clock from phy, 0:from sys_25M */
#define     MAC_CTRL_HALF_LEFT_BUF_SHIFT	28
#define     MAC_CTRL_HALF_LEFT_BUF_MASK		0xF		/* When half-duplex mode, should hold some bytes for mac retry . (8*4bytes unit) */

/* MAC IPG/IFG Control Register */
#define REG_MAC_IPG_IFG			0x1484
#define     MAC_IPG_IFG_IPGT_SHIFT	0	/* Desired back to back inter-packet gap. The default is 96-bit time. */
#define     MAC_IPG_IFG_IPGT_MASK	0x7f
#define     MAC_IPG_IFG_MIFG_SHIFT	8	/* Minimum number of IFG to enforce in between RX frames. */
#define     MAC_IPG_IFG_MIFG_MASK	0xff	/* Frame gap below such IFP is dropped. */
#define     MAC_IPG_IFG_IPGR1_SHIFT	16	/* 64bit Carrier-Sense window */
#define     MAC_IPG_IFG_IPGR1_MASK	0x7f
#define     MAC_IPG_IFG_IPGR2_SHIFT	24	/* 96-bit IPG window */
#define     MAC_IPG_IFG_IPGR2_MASK	0x7f

/* MAC STATION ADDRESS */
#define REG_MAC_STA_ADDR	0x1488

/* Hash table for multicast address */
#define REG_RX_HASH_TABLE	0x1490

/* MAC Half-Duplex Control Register */
#define REG_MAC_HALF_DUPLX_CTRL			0x1498
#define     MAC_HALF_DUPLX_CTRL_LCOL_SHIFT	0	/* Collision Window. */
#define     MAC_HALF_DUPLX_CTRL_LCOL_MASK	0x3ff
#define     MAC_HALF_DUPLX_CTRL_RETRY_SHIFT	12	/* Retransmission maximum, afterwards the packet will be discarded. */
#define     MAC_HALF_DUPLX_CTRL_RETRY_MASK	0xf
#define     MAC_HALF_DUPLX_CTRL_EXC_DEF_EN	0x10000	/* 1: Allow the transmission of a packet which has been excessively deferred */
#define     MAC_HALF_DUPLX_CTRL_NO_BACK_C	0x20000	/* 1: No back-off on collision, immediately start the retransmission. */
#define     MAC_HALF_DUPLX_CTRL_NO_BACK_P	0x40000	/* 1: No back-off on backpressure, immediately start the transmission after back pressure */
#define     MAC_HALF_DUPLX_CTRL_ABEBE		0x80000	/* 1: Alternative Binary Exponential Back-off Enabled */
#define     MAC_HALF_DUPLX_CTRL_ABEBT_SHIFT	20	/* Maximum binary exponential number. */
#define     MAC_HALF_DUPLX_CTRL_ABEBT_MASK	0xf
#define     MAC_HALF_DUPLX_CTRL_JAMIPG_SHIFT	24	/* IPG to start JAM for collision based flow control in half-duplex  */
#define     MAC_HALF_DUPLX_CTRL_JAMIPG_MASK	0xf	/* mode. In unit of 8-bit time. */

/* Maximum Frame Length Control Register */
#define REG_MTU	0x149c

/* Wake-On-Lan control register */
#define REG_WOL_CTRL		0x14a0
#define     WOL_PATTERN_EN	0x00000001
#define     WOL_PATTERN_PME_EN	0x00000002
#define     WOL_MAGIC_EN	0x00000004
#define     WOL_MAGIC_PME_EN	0x00000008
#define     WOL_LINK_CHG_EN	0x00000010
#define     WOL_LINK_CHG_PME_EN	0x00000020
#define     WOL_PATTERN_ST	0x00000100
#define     WOL_MAGIC_ST	0x00000200
#define     WOL_LINKCHG_ST	0x00000400
#define     WOL_PT0_EN		0x00010000
#define     WOL_PT1_EN		0x00020000
#define     WOL_PT2_EN		0x00040000
#define     WOL_PT3_EN		0x00080000
#define     WOL_PT4_EN		0x00100000
#define     WOL_PT0_MATCH	0x01000000
#define     WOL_PT1_MATCH	0x02000000
#define     WOL_PT2_MATCH	0x04000000
#define     WOL_PT3_MATCH	0x08000000
#define     WOL_PT4_MATCH	0x10000000

/* Internal SRAM Partition Register */
#define REG_SRAM_TXRAM_END	0x1500	/* Internal tail address of TXRAM default: 2byte*1024 */
#define REG_SRAM_RXRAM_END	0x1502	/* Internal tail address of RXRAM default: 2byte*1024 */

/*
#define REG_SRAM_TCPH_PATH_ADDR     (REG_SRAM_RFD_ADDR+48)
#define     SRAM_TCPH_ADDR_MASK             0x0fff
#define     SRAM_TCPH_ADDR_SHIFT            0
#define     SRAM_PATH_ADDR_MASK             0x0fff
#define     SRAM_PATH_ADDR_SHIFT            16
*/

/* Descriptor Control register */
#define REG_DESC_BASE_ADDR_HI	0x1540
#define REG_TXD_BASE_ADDR_LO	0x1544	/* The base address of the Transmit Data Memory low 32-bit(dword align) */
#define REG_TXD_MEM_SIZE	0x1548	/* Transmit Data Memory size(by double word , max 256KB) */
#define REG_TXS_BASE_ADDR_LO	0x154C	/* The base address of the Transmit Status Memory low 32-bit(dword word align) */
#define REG_TXS_MEM_SIZE	0x1550	/* double word unit, max 4*2047 bytes. */
#define REG_RXD_BASE_ADDR_LO	0x1554	/* The base address of the Transmit Status Memory low 32-bit(unit 8 bytes) */
#define REG_RXD_BUF_NUM		0x1558	/* Receive Data & Status Memory buffer number (unit 1536bytes, max 1536*2047) */

/* DMAR Control Register */
#define REG_DMAR	0x1580
#define     DMAR_EN	0x1	/* 1: Enable DMAR */

/* TX Cur-Through (early tx threshold) Control Register */
#define REG_TX_CUT_THRESH	0x1590	/* TxMac begin transmit packet threshold(unit word) */

/* DMAW Control Register */
#define REG_DMAW	0x15A0
#define     DMAW_EN	0x1

/* Flow control register */
#define REG_PAUSE_ON_TH		0x15A8	/* RXD high watermark of overflow threshold configuration register */
#define REG_PAUSE_OFF_TH	0x15AA	/* RXD lower watermark of overflow threshold configuration register */

/* Mailbox Register */
#define REG_MB_TXD_WR_IDX	0x15f0	/* double word align */
#define REG_MB_RXD_RD_IDX	0x15F4	/* RXD Read index (unit: 1536byets) */

/* Interrupt Status Register */
#define REG_ISR			0x1600
#define     ISR_TIMER		1		/* Interrupt when Timer is counted down to zero */
#define     ISR_MANUAL		2		/* Software manual interrupt, for debug. Set when SW_MAN_INT_EN is set in Table 51 Selene Master Control Register (Offset 0x1400). */
#define     ISR_RXF_OV		4		/* RXF overflow interrupt */
#define     ISR_TXF_UR		8		/* TXF underrun interrupt */
#define     ISR_TXS_OV		0x10		/* Internal transmit status buffer full interrupt */
#define     ISR_RXS_OV		0x20		/* Internal receive status buffer ful interrupt */
#define     ISR_LINK_CHG	0x40		/* Link Status Change Interrupt */
#define     ISR_HOST_TXD_UR	0x80
#define     ISR_HOST_RXD_OV	0x100		/* Host rx data memory full , one pulse */
//#define     ISR_HOST_TXS_OV	0x200		/* Host tx status memory full , one pulse */
#define     ISR_DMAR_TO_RST	0x200		/* DMAR op timeout interrupt. SW should do Reset */
#define     ISR_DMAW_TO_RST	0x400
#define     ISR_PHY		0x800		/* phy interrupt */
#define     ISR_TS_UPDATE	0x10000		/* interrupt after new tx pkt status written to host */
#define     ISR_RS_UPDATE	0x20000		/* interrupt ater new rx pkt status written to host. */
#define     ISR_TX_EARLY	0x40000		/* interrupt when txmac begin transmit one packet */
#define     ISR_UR_DETECTED	0x1000000
#define     ISR_FERR_DETECTED	0x2000000
#define     ISR_NFERR_DETECTED	0x4000000
#define     ISR_CERR_DETECTED	0x8000000
#define     ISR_PHY_LINKDOWN	0x10000000
#define     ISR_DIS_INT		0x80000000

#define ISR_TX_EVENT (ISR_TXF_UR|ISR_TXS_OV|ISR_HOST_TXD_UR|ISR_TS_UPDATE|ISR_TX_EARLY)
#define ISR_RX_EVENT (ISR_RXF_OV|ISR_RXS_OV|ISR_HOST_RXD_OV|ISR_RS_UPDATE)

/* Interrupt Mask Register */
#define REG_IMR	0x1604

#define IMR_NORMAL_MASK		(\
	/*ISR_LINK_CHG		|*/\
	ISR_MANUAL		|\
	ISR_DMAR_TO_RST		|\
	ISR_DMAW_TO_RST		|\
	ISR_PHY			|\
	ISR_PHY_LINKDOWN	|\
	ISR_TS_UPDATE		|\
	ISR_RS_UPDATE		)

/* Receive MAC Statistics Registers */
#define REG_STS_RX_PAUSE	0x1700	/* The number of Pause packet received */
#define REG_STS_RXD_OV		0x1704	/* The number of frame dropped due to occurrence of RX FIFO overflow */
#define REG_STS_RXS_OV		0x1708	/* The number of frame dropped due to occerrence of RX Status Buffer Overflow */
#define REG_STS_RX_FILTER	0x170C	/* The number of packet dropped due to address filtering */

/* MII definitions */

/* PHY Common Register */
#define MII_AT001_CR	0x09
#define MII_AT001_SR	0x0A
#define MII_AT001_ESR	0x0F
#define MII_AT001_PSCR	0x10
#define MII_AT001_PSSR	0x11
#define MII_SMARTSPEED	0x14
#define MII_DBG_ADDR	0x1D
#define MII_DBG_DATA	0x1E

/* PHY Control Register */
#define MII_CR_SPEED_SELECT_MSB	0x0040	/* bits 6,13: 10=1000, 01=100, 00=10 */
#define MII_CR_COLL_TEST_ENABLE	0x0080	/* Collision test enable */
#define MII_CR_FULL_DUPLEX	0x0100	/* FDX =1, half duplex =0 */
#define MII_CR_RESTART_AUTO_NEG	0x0200	/* Restart auto negotiation */
#define MII_CR_ISOLATE		0x0400	/* Isolate PHY from MII */
#define MII_CR_POWER_DOWN	0x0800	/* Power down */
#define MII_CR_AUTO_NEG_EN	0x1000	/* Auto Neg Enable */
#define MII_CR_SPEED_SELECT_LSB	0x2000	/* bits 6,13: 10=1000, 01=100, 00=10 */
#define MII_CR_LOOPBACK		0x4000	/* 0 = normal, 1 = loopback */
#define MII_CR_RESET		0x8000	/* 0 = normal, 1 = PHY reset */
#define MII_CR_SPEED_MASK	0x2040
#define MII_CR_SPEED_1000	0x0040
#define MII_CR_SPEED_100	0x2000
#define MII_CR_SPEED_10		0x0000

/* PHY Status Register */
#define MII_SR_EXTENDED_CAPS		0x0001	/* Extended register capabilities */
#define MII_SR_JABBER_DETECT		0x0002	/* Jabber Detected */
#define MII_SR_LINK_STATUS		0x0004	/* Link Status 1 = link */
#define MII_SR_AUTONEG_CAPS		0x0008	/* Auto Neg Capable */
#define MII_SR_REMOTE_FAULT		0x0010	/* Remote Fault Detect */
#define MII_SR_AUTONEG_COMPLETE		0x0020	/* Auto Neg Complete */
#define MII_SR_PREAMBLE_SUPPRESS	0x0040	/* Preamble may be suppressed */
#define MII_SR_EXTENDED_STATUS		0x0100	/* Ext. status info in Reg 0x0F */
#define MII_SR_100T2_HD_CAPS		0x0200	/* 100T2 Half Duplex Capable */
#define MII_SR_100T2_FD_CAPS		0x0400	/* 100T2 Full Duplex Capable */
#define MII_SR_10T_HD_CAPS		0x0800	/* 10T   Half Duplex Capable */
#define MII_SR_10T_FD_CAPS		0x1000	/* 10T   Full Duplex Capable */
#define MII_SR_100X_HD_CAPS		0x2000	/* 100X  Half Duplex Capable */
#define MII_SR_100X_FD_CAPS		0x4000	/* 100X  Full Duplex Capable */
#define MII_SR_100T4_CAPS		0x8000	/* 100T4 Capable */

/* Link partner ability register. */
#define MII_LPA_SLCT		0x001f	/* Same as advertise selector */
#define MII_LPA_10HALF		0x0020	/* Can do 10mbps half-duplex  */
#define MII_LPA_10FULL		0x0040	/* Can do 10mbps full-duplex  */
#define MII_LPA_100HALF		0x0080	/* Can do 100mbps half-duplex */
#define MII_LPA_100FULL		0x0100	/* Can do 100mbps full-duplex */
#define MII_LPA_100BASE4	0x0200	/* 100BASE-T4 */
#define MII_LPA_PAUSE		0x0400	/* PAUSE */
#define MII_LPA_ASYPAUSE	0x0800	/* Asymmetrical PAUSE */
#define MII_LPA_RFAULT		0x2000	/* Link partner faulted */
#define MII_LPA_LPACK		0x4000	/* Link partner acked us */
#define MII_LPA_NPAGE		0x8000	/* Next page bit */

/* Autoneg Advertisement Register */
#define MII_AR_SELECTOR_FIELD	0x0001	/* indicates IEEE 802.3 CSMA/CD */
#define MII_AR_10T_HD_CAPS	0x0020	/* 10T   Half Duplex Capable */
#define MII_AR_10T_FD_CAPS	0x0040	/* 10T   Full Duplex Capable */
#define MII_AR_100TX_HD_CAPS	0x0080	/* 100TX Half Duplex Capable */
#define MII_AR_100TX_FD_CAPS	0x0100	/* 100TX Full Duplex Capable */
#define MII_AR_100T4_CAPS	0x0200	/* 100T4 Capable */
#define MII_AR_PAUSE		0x0400	/* Pause operation desired */
#define MII_AR_ASM_DIR		0x0800	/* Asymmetric Pause Direction bit */
#define MII_AR_REMOTE_FAULT	0x2000	/* Remote Fault detected */
#define MII_AR_NEXT_PAGE	0x8000	/* Next Page ability supported */
#define MII_AR_SPEED_MASK	0x01E0
#define MII_AR_DEFAULT_CAP_MASK	0x0DE0

/* 1000BASE-T Control Register */
#define MII_AT001_CR_1000T_HD_CAPS		0x0100	/* Advertise 1000T HD capability */
#define MII_AT001_CR_1000T_FD_CAPS		0x0200	/* Advertise 1000T FD capability  */
#define MII_AT001_CR_1000T_REPEATER_DTE		0x0400	/* 1=Repeater/switch device port, 0=DTE device */
#define MII_AT001_CR_1000T_MS_VALUE		0x0800	/* 1=Configure PHY as Master, 0=Configure PHY as Slave */
#define MII_AT001_CR_1000T_MS_ENABLE		0x1000	/* 1=Master/Slave manual config value, 0=Automatic Master/Slave config */
#define MII_AT001_CR_1000T_TEST_MODE_NORMAL	0x0000	/* Normal Operation */
#define MII_AT001_CR_1000T_TEST_MODE_1		0x2000	/* Transmit Waveform test */
#define MII_AT001_CR_1000T_TEST_MODE_2		0x4000	/* Master Transmit Jitter test */
#define MII_AT001_CR_1000T_TEST_MODE_3		0x6000	/* Slave Transmit Jitter test */
#define MII_AT001_CR_1000T_TEST_MODE_4		0x8000	/* Transmitter Distortion test */
#define MII_AT001_CR_1000T_SPEED_MASK		0x0300
#define MII_AT001_CR_1000T_DEFAULT_CAP_MASK	0x0300

/* 1000BASE-T Status Register */
#define MII_AT001_SR_1000T_LP_HD_CAPS			0x0400	/* LP is 1000T HD capable */
#define MII_AT001_SR_1000T_LP_FD_CAPS			0x0800	/* LP is 1000T FD capable */
#define MII_AT001_SR_1000T_REMOTE_RX_STATUS		0x1000	/* Remote receiver OK */
#define MII_AT001_SR_1000T_LOCAL_RX_STATUS		0x2000	/* Local receiver OK */
#define MII_AT001_SR_1000T_MS_CONFIG_RES		0x4000	/* 1=Local TX is Master, 0=Slave */
#define MII_AT001_SR_1000T_MS_CONFIG_FAULT		0x8000	/* Master/Slave config fault */
#define MII_AT001_SR_1000T_REMOTE_RX_STATUS_SHIFT	12
#define MII_AT001_SR_1000T_LOCAL_RX_STATUS_SHIFT	13

/* Extended Status Register */
#define MII_AT001_ESR_1000T_HD_CAPS	0x1000	/* 1000T HD capable */
#define MII_AT001_ESR_1000T_FD_CAPS	0x2000	/* 1000T FD capable */
#define MII_AT001_ESR_1000X_HD_CAPS	0x4000	/* 1000X HD capable */
#define MII_AT001_ESR_1000X_FD_CAPS	0x8000	/* 1000X FD capable */

/* AT001 PHY Specific Control Register */
#define MII_AT001_PSCR_JABBER_DISABLE			0x0001	/* 1=Jabber Function disabled */
#define MII_AT001_PSCR_POLARITY_REVERSAL		0x0002	/* 1=Polarity Reversal enabled */
#define MII_AT001_PSCR_SQE_TEST				0x0004	/* 1=SQE Test enabled *//
#define MII_AT001_PSCR_MAC_POWERDOWN			0x0008
#define MII_AT001_PSCR_CLK125_DISABLE			0x0010	/* 1=CLK125 low, 0=CLK125 toggling */
#define MII_AT001_PSCR_MDI_MANUAL_MODE			0x0000	/* MDI Crossover Mode bits 6:5, Manual MDI configuration */
#define MII_AT001_PSCR_MDIX_MANUAL_MODE			0x0020	/* Manual MDIX configuration */
#define MII_AT001_PSCR_AUTO_X_1000T			0x0040	/* 1000BASE-T: Auto crossover, 100BASE-TX/10BASE-T: MDI Mode */
#define MII_AT001_PSCR_AUTO_X_MODE			0x0060	/* Auto crossover enabled all speeds. */
#define MII_AT001_PSCR_10BT_EXT_DIST_ENABLE		0x0080	/* 1=Enable Extended 10BASE-T distance (Lower 10BASE-T RX Threshold), 0=Normal 10BASE-T RX Threshold */
#define MII_AT001_PSCR_MII_5BIT_ENABLE			0x0100	/* 1=5-Bit interface in 100BASE-TX, 0=MII interface in 100BASE-TX */
#define MII_AT001_PSCR_SCRAMBLER_DISABLE		0x0200	/* 1=Scrambler disable */
#define MII_AT001_PSCR_FORCE_LINK_GOOD			0x0400	/* 1=Force link good */
#define MII_AT001_PSCR_ASSERT_CRS_ON_TX			0x0800	/* 1=Assert CRS on Transmit */
#define MII_AT001_PSCR_POLARITY_REVERSAL_SHIFT		1
#define MII_AT001_PSCR_AUTO_X_MODE_SHIFT		5
#define MII_AT001_PSCR_10BT_EXT_DIST_ENABLE_SHIFT	7

/* AT001 PHY Specific Status Register */
#define MII_AT001_PSSR_SPD_DPLX_RESOLVED	0x0800	/* 1=Speed & Duplex resolved */
#define MII_AT001_PSSR_DPLX			0x2000	/* 1=Duplex 0=Half Duplex */
#define MII_AT001_PSSR_SPEED			0xC000	/* Speed, bits 14:15 */
#define MII_AT001_PSSR_10MBS			0x0000	/* 00=10Mbs */
#define MII_AT001_PSSR_100MBS			0x4000	/* 01=100Mbs */
#define MII_AT001_PSSR_1000MBS			0x8000	/* 10=1000Mbs */

/* PCI Command Register Bit Definitions */
#define PCI_REG_COMMAND		0x04
#define CMD_IO_SPACE		0x0001
#define CMD_MEMORY_SPACE	0x0002
#define CMD_BUS_MASTER		0x0004

/* Wake Up Filter Control */
#define ATL2_WUFC_LNKC	0x00000001	/* Link Status Change Wakeup Enable */
#define ATL2_WUFC_MAG	0x00000002	/* Magic Packet Wakeup Enable */
#define ATL2_WUFC_EX	0x00000004	/* Directed Exact Wakeup Enable */
#define ATL2_WUFC_MC	0x00000008	/* Multicast Wakeup Enable */
#define ATL2_WUFC_BC	0x00000010	/* Broadcast Wakeup Enable */

/* Error Codes */
#define ATL2_SUCCESS		0
#define ATL2_ERR_EEPROM		1
#define ATL2_ERR_PHY		2
#define ATL2_ERR_CONFIG		3
#define ATL2_ERR_PARAM		4
#define ATL2_ERR_MAC_TYPE	5
#define ATL2_ERR_PHY_TYPE	6
#define ATL2_ERR_PHY_SPEED	7
#define ATL2_ERR_PHY_RES	8

#define SPEED_0		0xffff
#define SPEED_10	10
#define SPEED_100	100
#define HALF_DUPLEX	1
#define FULL_DUPLEX	2

#define MEDIA_TYPE_AUTO_SENSOR	0
#define MEDIA_TYPE_100M_FULL	1
#define MEDIA_TYPE_100M_HALF	2
#define MEDIA_TYPE_10M_FULL	3
#define MEDIA_TYPE_10M_HALF	4

#define ADVERTISE_10_HALF	0x0001
#define ADVERTISE_10_FULL	0x0002
#define ADVERTISE_100_HALF	0x0004
#define ADVERTISE_100_FULL	0x0008
#define ADVERTISE_1000_HALF	0x0010	/* Not used, just FYI */
#define ADVERTISE_1000_FULL	0x0020

#define AUTONEG_ADVERTISE_SPEED_DEFAULT	0x000F	/* Everything */
#define AUTONEG_ADVERTISE_10_100_ALL	0x000F	/* All 10/100 speeds*/
#define AUTONEG_ADVERTISE_10_ALL	0x0003	/* 10Mbps Full & Half speeds*/

/* The size (in bytes) of a ethernet packet */
#define ENET_HEADER_SIZE		14
#define MAXIMUM_ETHERNET_FRAME_SIZE	1518	/* with FCS */
#define MINIMUM_ETHERNET_FRAME_SIZE	64	/* with FCS */
#define ETHERNET_FCS_SIZE		4
#define MAX_JUMBO_FRAME_SIZE		0x2000
#define VLAN_SIZE                                               4

#define PHY_AUTO_NEG_TIME	45	/* 4.5 Seconds */
#define PHY_FORCE_TIME		20	/* 2.0 Seconds */

/* For checksumming , the sum of all words in the EEPROM should equal 0xBABA */
#define EEPROM_SUM		0xBABA
#define NODE_ADDRESS_SIZE	6

typedef struct _tx_pkt_header {
	unsigned    pkt_size        : 11;
	unsigned                    : 4;    // reserved
	unsigned    ins_vlan        : 1;    // txmac should insert vlan
	unsigned short vlan             ;   // vlan tag
} tx_pkt_header_t;
/* FIXME: replace above bitfields with MASK/SHIFT defines below */
#define TX_PKT_HEADER_SIZE_MASK		0x7FF
#define TX_PKT_HEADER_SIZE_SHIFT	0
#define TX_PKT_HEADER_INS_VLAN_MASK	0x1
#define TX_PKT_HEADER_INS_VLAN_SHIFT	15
#define TX_PKT_HEADER_VLAN_TAG_MASK	0xFFFF
#define TX_PKT_HEADER_VLAN_TAG_SHIFT	16

typedef struct _tx_pkt_status {
	unsigned    pkt_size        : 11;
	unsigned                    : 5;    // reserved
	unsigned    ok              : 1;    // current packet is transmitted ok without error
	unsigned    bcast           : 1;    // current packet is broadcast
	unsigned    mcast           : 1;    // current packet is multicast
	unsigned    pause           : 1;    // transmiited a pause frame
	unsigned    ctrl            : 1;
	unsigned    defer           : 1;    // current packet is xmitted with defer.
	unsigned    exc_defer       : 1;
	unsigned    single_col      : 1;
	unsigned    multi_col       : 1;
	unsigned    late_col        : 1;
	unsigned    abort_col       : 1;
	unsigned    underun         : 1;    // current packet is abort due to txram underrun.
	unsigned                    : 3;    // reserved
	unsigned    update          : 1;    // always 1'b1 in tx_status_buf.
} tx_pkt_status_t;
/* FIXME: replace above bitfields with MASK/SHIFT defines below */
#define TX_PKT_STATUS_SIZE_MASK		0x7FF
#define TX_PKT_STATUS_SIZE_SHIFT	0
#define TX_PKT_STATUS_OK_MASK		0x1
#define TX_PKT_STATUS_OK_SHIFT		16
#define TX_PKT_STATUS_BCAST_MASK	0x1
#define TX_PKT_STATUS_BCAST_SHIFT	17
#define TX_PKT_STATUS_MCAST_MASK	0x1
#define TX_PKT_STATUS_MCAST_SHIFT	18
#define TX_PKT_STATUS_PAUSE_MASK	0x1
#define TX_PKT_STATUS_PAUSE_SHIFT	19
#define TX_PKT_STATUS_CTRL_MASK		0x1
#define TX_PKT_STATUS_CTRL_SHIFT	20
#define TX_PKT_STATUS_DEFER_MASK	0x1
#define TX_PKT_STATUS_DEFER_SHIFT	21
#define TX_PKT_STATUS_EXC_DEFER_MASK	0x1
#define TX_PKT_STATUS_EXC_DEFER_SHIFT	22
#define TX_PKT_STATUS_SINGLE_COL_MASK	0x1
#define TX_PKT_STATUS_SINGLE_COL_SHIFT	23
#define TX_PKT_STATUS_MULTI_COL_MASK	0x1
#define TX_PKT_STATUS_MULTI_COL_SHIFT	24
#define TX_PKT_STATUS_LATE_COL_MASK	0x1
#define TX_PKT_STATUS_LATE_COL_SHIFT	25
#define TX_PKT_STATUS_ABORT_COL_MASK	0x1
#define TX_PKT_STATUS_ABORT_COL_SHIFT	26
#define TX_PKT_STATUS_UNDERRUN_MASK	0x1
#define TX_PKT_STATUS_UNDERRUN_SHIFT	27
#define TX_PKT_STATUS_UPDATE_MASK	0x1
#define TX_PKT_STATUS_UPDATE_SHIFT	31

typedef struct _rx_pkt_status {
	unsigned    pkt_size        : 11;   // packet size, max 2047bytes
	unsigned                    : 5;    // reserved
	unsigned    ok              : 1;    // current packet is received ok without error.
	unsigned    bcast           : 1;    // current packet is broadcast.
	unsigned    mcast           : 1;    // current packet is multicast.
	unsigned    pause           : 1;
	unsigned    ctrl            : 1;
	unsigned    crc             : 1;    // received a packet with crc error.
	unsigned    code            : 1;    // received a packet with code error.
	unsigned    runt            : 1;    // received a packet less than 64bytes with good crc
	unsigned    frag            : 1;    // ....................................with bad  crc
	unsigned    trunc           : 1;    // current frame is cutted due to rxram full.
	unsigned    align           : 1;    // this packet is alignment error.
	unsigned    vlan            : 1;    // this packet has vlan
	unsigned                    : 3;    // reserved
	unsigned    update          : 1;
	unsigned short vtag         ;       // vlan tag
	unsigned                    : 16;
} rx_pkt_status_t;
/* FIXME: replace above bitfields with MASK/SHIFT defines below */
#define RX_PKT_STATUS_SIZE_MASK		0x7FF
#define RX_PKT_STATUS_SIZE_SHIFT	0
#define RX_PKT_STATUS_OK_MASK		0x1
#define RX_PKT_STATUS_OK_SHIFT		16
#define RX_PKT_STATUS_BCAST_MASK	0x1
#define RX_PKT_STATUS_BCAST_SHIFT	17
#define RX_PKT_STATUS_MCAST_MASK	0x1
#define RX_PKT_STATUS_MCAST_SHIFT	18
#define RX_PKT_STATUS_PAUSE_MASK	0x1
#define RX_PKT_STATUS_PAUSE_SHIFT	19
#define RX_PKT_STATUS_CTRL_MASK		0x1
#define RX_PKT_STATUS_CTRL_SHIFT	20
#define RX_PKT_STATUS_CRC_MASK		0x1
#define RX_PKT_STATUS_CRC_SHIFT		21
#define RX_PKT_STATUS_CODE_MASK		0x1
#define RX_PKT_STATUS_CODE_SHIFT	22
#define RX_PKT_STATUS_RUNT_MASK		0x1
#define RX_PKT_STATUS_RUNT_SHIFT	23
#define RX_PKT_STATUS_FRAG_MASK		0x1
#define RX_PKT_STATUS_FRAG_SHIFT	24
#define RX_PKT_STATUS_TRUNK_MASK	0x1
#define RX_PKT_STATUS_TRUNK_SHIFT	25
#define RX_PKT_STATUS_ALIGN_MASK	0x1
#define RX_PKT_STATUS_ALIGN_SHIFT	26
#define RX_PKT_STATUS_VLAN_MASK		0x1
#define RX_PKT_STATUS_VLAN_SHIFT	27
#define RX_PKT_STATUS_UPDATE_MASK	0x1
#define RX_PKT_STATUS_UPDATE_SHIFT	31
#define RX_PKT_STATUS_VLAN_TAG_MASK	0xFFFF
#define RX_PKT_STATUS_VLAN_TAG_SHIFT	32

typedef struct _rx_desc {
	rx_pkt_status_t   status;
	unsigned char     packet[1536-sizeof(rx_pkt_status_t)];
} rx_desc_t;

typedef enum {
	atl2_10_half = 0,
	atl2_10_full = 1,
	atl2_100_half = 2,
	atl2_100_full = 3
} atl2_speed_duplex_type;

struct atl2_spi_flash_dev {
	const char *manu_name;	/* manufacturer id */
	/* op-code */
	u8 cmdWRSR;
	u8 cmdREAD;
	u8 cmdPROGRAM;
	u8 cmdWREN;
	u8 cmdWRDI;
	u8 cmdRDSR;
	u8 cmdRDID;
	u8 cmdSECTOR_ERASE;
	u8 cmdCHIP_ERASE;
};

/* Structure containing variables used by the shared code (atl2_hw.c) */
struct atl2_hw {
	u8 *hw_addr;
	void *back;
		
	u8 preamble_len;
	u8 max_retry;          // Retransmission maximum , afterwards the packet will be discarded.
	u8 jam_ipg;            // IPG to start JAM for collision based flow control in half-duplex mode. In unit of 8-bit time.
	u8 ipgt;               // Desired back to back inter-packet gap. The default is 96-bit time.
	u8 min_ifg;            // Minimum number of IFG to enforce in between RX frames. Frame gap below such IFP is dropped.
	u8 ipgr1;              // 64bit Carrier-Sense window
	u8 ipgr2;              // 96-bit IPG window
	u8 retry_buf;          // When half-duplex mode, should hold some bytes for mac retry . (8*4bytes unit)
				
	u16 fc_rxd_hi;
	u16 fc_rxd_lo;
	u16 lcol;              // Collision Window
	u16 max_frame_size;
	
	u16 MediaType;
	u16 autoneg_advertised;
	u16 pci_cmd_word;

	u16 mii_autoneg_adv_reg;
	
	u32 mem_rang;
	u32 txcw;
	u32 mc_filter_type;
	u32 num_mc_addrs;
	u32 collision_delta;
	u32 tx_packet_delta;
	u16 phy_spd_default;
	
	u16 device_id;
	u16 vendor_id;
	u16 subsystem_id;
	u16 subsystem_vendor_id;
	u8 revision_id;

	// spi flash
	u8 flash_vendor;
	
	u8 dma_fairness;
	u8 mac_addr[NODE_ADDRESS_SIZE];
	u8 perm_mac_addr[NODE_ADDRESS_SIZE];

	//    bool phy_preamble_sup;
	bool phy_configured;
};

#endif /* _ATL2_HW_H_ */
