/* atl2_hw.c -- atl2 hardware control functions
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

#include <asm/processor.h>
#include <linux/crc32.h>
#include <linux/etherdevice.h>
#include <linux/mii.h>
#include <linux/string.h>

#include "atl2.h"
#include "atl2_hw.h"

#define LBYTESWAP( a )  ( ( ( (a) & 0x00ff00ff ) << 8 ) | ( ( (a) & 0xff00ff00 ) >> 8 ) )
#define LONGSWAP( a )   ( ( LBYTESWAP( a ) << 16 ) | ( LBYTESWAP( a ) >> 16 ) )
#define SHORTSWAP( a )  ( ( (a) << 8 ) | ( (a) >> 8 ) )

/*
 * Reset the transmit and receive units; mask and clear all interrupts.
 *
 * hw - Struct containing variables accessed by shared code
 * return : ATL2_SUCCESS  or  idle status (if error)
 */
s32
atl2_reset_hw(struct atl2_hw *hw)
{
	u32 icr;
	u16 pci_cfg_cmd_word;
	int i;

	/* Workaround for PCI problem when BIOS sets MMRBC incorrectly. */
	atl2_read_pci_cfg(hw, PCI_REG_COMMAND, &pci_cfg_cmd_word);
	if ((pci_cfg_cmd_word &
		(CMD_IO_SPACE|CMD_MEMORY_SPACE|CMD_BUS_MASTER)) !=
		(CMD_IO_SPACE|CMD_MEMORY_SPACE|CMD_BUS_MASTER)) {
		pci_cfg_cmd_word |=
			(CMD_IO_SPACE|CMD_MEMORY_SPACE|CMD_BUS_MASTER);
		atl2_write_pci_cfg(hw, PCI_REG_COMMAND, &pci_cfg_cmd_word);
	}

	/* Clear Interrupt mask to stop board from generating
	 * interrupts & Clear any pending interrupt events
	 */
//    ATL2_WRITE_REG(hw, REG_IMR, 0);
//    ATL2_WRITE_REG(hw, REG_ISR, 0xffffffff);

	/* Issue Soft Reset to the MAC.  This will reset the chip's
	 * transmit, receive, DMA.  It will not effect
	 * the current PCI configuration.  The global reset bit is self-
	 * clearing, and should clear within a microsecond.
	 */
	ATL2_WRITE_REG(hw, REG_MASTER_CTRL, MASTER_CTRL_SOFT_RST);
	wmb();
	msec_delay(1); // delay about 1ms

	/* Wait at least 10ms for All module to be Idle */
	for (i=0; i < 10; i++) {
		icr = ATL2_READ_REG(hw, REG_IDLE_STATUS);
		if (!icr)
			break;
		msec_delay(1); // delay 1 ms
		cpu_relax();
	}

	if (icr)
		return icr;

	return ATL2_SUCCESS;
}

#define CUSTOM_SPI_CS_SETUP        2
#define CUSTOM_SPI_CLK_HI          2
#define CUSTOM_SPI_CLK_LO          2
#define CUSTOM_SPI_CS_HOLD         2
#define CUSTOM_SPI_CS_HI           3

static struct atl2_spi_flash_dev flash_table[] =
{
/*  manu_name   WRSR    READ    PROGRAM WREN    WRDI    RDSR    RDID    SECTOR_ERASE CHIP_ERASE */
    {"Atmel",   0x0,    0x03,   0x02,   0x06,   0x04,   0x05,   0x15,   0x52,   0x62 },
    {"SST",     0x01,   0x03,   0x02,   0x06,   0x04,   0x05,   0x90,   0x20,   0x60 },
    {"ST",      0x01,   0x03,   0x02,   0x06,   0x04,   0x05,   0xAB,   0xD8,   0xC7 },
};

static bool
atl2_spi_read(struct atl2_hw* hw, u32 addr, u32* buf)
{
	int i;
	u32 value;

	ATL2_WRITE_REG(hw, REG_SPI_DATA, 0);
	ATL2_WRITE_REG(hw, REG_SPI_ADDR, addr);

	value = SPI_FLASH_CTRL_WAIT_READY |
		(CUSTOM_SPI_CS_SETUP & SPI_FLASH_CTRL_CS_SETUP_MASK) << SPI_FLASH_CTRL_CS_SETUP_SHIFT |
		(CUSTOM_SPI_CLK_HI & SPI_FLASH_CTRL_CLK_HI_MASK) << SPI_FLASH_CTRL_CLK_HI_SHIFT |
		(CUSTOM_SPI_CLK_LO & SPI_FLASH_CTRL_CLK_LO_MASK) << SPI_FLASH_CTRL_CLK_LO_SHIFT |
		(CUSTOM_SPI_CS_HOLD & SPI_FLASH_CTRL_CS_HOLD_MASK) << SPI_FLASH_CTRL_CS_HOLD_SHIFT |
		(CUSTOM_SPI_CS_HI & SPI_FLASH_CTRL_CS_HI_MASK) << SPI_FLASH_CTRL_CS_HI_SHIFT |
		(0x1 & SPI_FLASH_CTRL_INS_MASK) << SPI_FLASH_CTRL_INS_SHIFT;

	ATL2_WRITE_REG(hw, REG_SPI_FLASH_CTRL, value);

	value |= SPI_FLASH_CTRL_START;

	ATL2_WRITE_REG(hw, REG_SPI_FLASH_CTRL, value);

	for (i = 0; i < 10; i++)
	{
		msec_delay(1); // 1ms
		value = ATL2_READ_REG(hw, REG_SPI_FLASH_CTRL);
		if (!(value & SPI_FLASH_CTRL_START))
			break;
	}

	if (value & SPI_FLASH_CTRL_START)
		return false;

	*buf = ATL2_READ_REG(hw, REG_SPI_DATA);

	return true;
}

/*
 * get_permanent_address
 * return 0 if get valid mac address,
 */
static int
get_permanent_address(struct atl2_hw *hw)
{
	u32 Addr[2];
	u32 i, Control;
	u16 Register;
	u8  EthAddr[NODE_ADDRESS_SIZE];
	bool KeyValid;
	
	if (is_valid_ether_addr(hw->perm_mac_addr))
		return 0;

	Addr[0] = 0;
	Addr[1] = 0;

	if (!atl2_check_eeprom_exist(hw)) { /* eeprom exists */
		Register = 0;
		KeyValid = false;

		/* Read out all EEPROM content */
		i = 0;
		while (1) {
			if (atl2_read_eeprom(hw, i + 0x100, &Control)) {
				if (KeyValid) {
					if (Register == REG_MAC_STA_ADDR)
						Addr[0] = Control;
					else if (Register == (REG_MAC_STA_ADDR + 4))
						Addr[1] = Control;
					KeyValid = false;
				} else if ((Control & 0xff) == 0x5A) {
					KeyValid = true;
					Register = (u16) (Control >> 16);
				} else {
					break; /* assume data end while encount an invalid KEYWORD */
				}
			} else {
				break; /* read error */
			}
			i += 4;
		}

		*(u32*) &EthAddr[2] = LONGSWAP(Addr[0]);
		*(u16*) &EthAddr[0] = SHORTSWAP(*(u16*)&Addr[1]);

		if (is_valid_ether_addr(EthAddr)) {
			memcpy(hw->perm_mac_addr, EthAddr, NODE_ADDRESS_SIZE);
			return 0;
		}
		return 1;
	}

	// see if SPI FLAHS exist ?
	Addr[0] = 0;
	Addr[1] = 0;
	Register = 0;
	KeyValid = false;
	i = 0;
	while (1) {
		if (atl2_spi_read(hw, i + 0x1f000, &Control)) {
			if (KeyValid) {
				if (Register == REG_MAC_STA_ADDR)
					Addr[0] = Control;
				else if (Register == (REG_MAC_STA_ADDR + 4))
					Addr[1] = Control;
				KeyValid = false;
			} else if ((Control & 0xff) == 0x5A) {
				KeyValid = true;
				Register = (u16) (Control >> 16);
			} else {
				break; /* data end */
			}
		} else {
			break; /* read error */
		}
		i += 4;
	}

	*(u32*) &EthAddr[2] = LONGSWAP(Addr[0]);
	*(u16*) &EthAddr[0] = SHORTSWAP(*(u16*)&Addr[1]);
	if (is_valid_ether_addr(EthAddr)) {
		memcpy(hw->perm_mac_addr, EthAddr, NODE_ADDRESS_SIZE);
		return 0;
	}
	/* maybe MAC-address is from BIOS */
	Addr[0] = ATL2_READ_REG(hw,REG_MAC_STA_ADDR);
	Addr[1] = ATL2_READ_REG(hw,REG_MAC_STA_ADDR+4);
	*(u32*) &EthAddr[2] = LONGSWAP(Addr[0]);
	*(u16*) &EthAddr[0] = SHORTSWAP(*(u16*)&Addr[1]);

	if (is_valid_ether_addr(EthAddr)) {
		memcpy(hw->perm_mac_addr, EthAddr, NODE_ADDRESS_SIZE);
		return 0;
	}

	return 1;
}

/*
 * Reads the adapter's MAC address from the EEPROM 
 *
 * hw - Struct containing variables accessed by shared code
 */
s32
atl2_read_mac_addr(struct atl2_hw *hw)
{
	u16 i;

	if (get_permanent_address(hw)) {
		// for test
		hw->perm_mac_addr[0] = 0x00;
		hw->perm_mac_addr[1] = 0x13;
		hw->perm_mac_addr[2] = 0x74;
		hw->perm_mac_addr[3] = 0x00;
		hw->perm_mac_addr[4] = 0x5c;
		hw->perm_mac_addr[5] = 0x38;
	}
	
	for(i = 0; i < NODE_ADDRESS_SIZE; i++)
		hw->mac_addr[i] = hw->perm_mac_addr[i];

	return ATL2_SUCCESS;
}

/*
 * Hashes an address to determine its location in the multicast table
 *
 * hw - Struct containing variables accessed by shared code
 * mc_addr - the multicast address to hash
 *
 * atl2_hash_mc_addr
 *  purpose
 *      set hash value for a multicast address
 *      hash calcu processing :
 *          1. calcu 32bit CRC for multicast address
 *          2. reverse crc with MSB to LSB
 */
u32
atl2_hash_mc_addr(struct atl2_hw *hw, u8 *mc_addr)
{
	u32 crc32, value=0;
	int i;

	crc32 = ether_crc_le(6, mc_addr);

	for (i=0; i<32; i++)
		value |= (((crc32>>i)&1)<<(31-i));

	return value;
}

/*
 * Sets the bit in the multicast table corresponding to the hash value.
 *
 * hw - Struct containing variables accessed by shared code
 * hash_value - Multicast address hash value
 */
void
atl2_hash_set(struct atl2_hw *hw, u32 hash_value)
{
	u32 hash_bit, hash_reg;
	u32 mta;

	/* The HASH Table  is a register array of 2 32-bit registers.
	 * It is treated like an array of 64 bits.  We want to set
	 * bit BitArray[hash_value]. So we figure out what register
	 * the bit is in, read it, OR in the new bit, then write
	 * back the new value.  The register is determined by the
	 * upper 7 bits of the hash value and the bit within that
	 * register are determined by the lower 5 bits of the value.
	 */
	hash_reg = (hash_value >> 31) & 0x1;
	hash_bit = (hash_value >> 26) & 0x1F;

	mta = ATL2_READ_REG_ARRAY(hw, REG_RX_HASH_TABLE, hash_reg);

	mta |= (1 << hash_bit);

	ATL2_WRITE_REG_ARRAY(hw, REG_RX_HASH_TABLE, hash_reg, mta);
}

/*
 * atl2_init_pcie - init PCIE module
 */
static void
atl2_init_pcie(struct atl2_hw *hw)
{
    u32 value;
    value = LTSSM_TEST_MODE_DEF;
    ATL2_WRITE_REG(hw, REG_LTSSM_TEST_MODE, value);

    value = PCIE_DLL_TX_CTRL1_DEF;
    ATL2_WRITE_REG(hw, REG_PCIE_DLL_TX_CTRL1, value);
}

void
atl2_init_flash_opcode(struct atl2_hw *hw)
{
	if (hw->flash_vendor >= ARRAY_SIZE(flash_table)) {
		hw->flash_vendor = 0; // ATMEL
	}
	// Init OP table
	ATL2_WRITE_REGB(hw, REG_SPI_FLASH_OP_PROGRAM,     flash_table[hw->flash_vendor].cmdPROGRAM);
	ATL2_WRITE_REGB(hw, REG_SPI_FLASH_OP_SC_ERASE,    flash_table[hw->flash_vendor].cmdSECTOR_ERASE);
	ATL2_WRITE_REGB(hw, REG_SPI_FLASH_OP_CHIP_ERASE,  flash_table[hw->flash_vendor].cmdCHIP_ERASE);
	ATL2_WRITE_REGB(hw, REG_SPI_FLASH_OP_RDID,        flash_table[hw->flash_vendor].cmdRDID);
	ATL2_WRITE_REGB(hw, REG_SPI_FLASH_OP_WREN,        flash_table[hw->flash_vendor].cmdWREN);
	ATL2_WRITE_REGB(hw, REG_SPI_FLASH_OP_RDSR,        flash_table[hw->flash_vendor].cmdRDSR);
	ATL2_WRITE_REGB(hw, REG_SPI_FLASH_OP_WRSR,        flash_table[hw->flash_vendor].cmdWRSR);
	ATL2_WRITE_REGB(hw, REG_SPI_FLASH_OP_READ,        flash_table[hw->flash_vendor].cmdREAD);
}

/********************************************************************
* Performs basic configuration of the adapter.
*
* hw - Struct containing variables accessed by shared code
* Assumes that the controller has previously been reset and is in a
* post-reset uninitialized state. Initializes multicast table, 
* and  Calls routines to setup link
* Leaves the transmit and receive units disabled and uninitialized.
********************************************************************/
s32
atl2_init_hw(struct atl2_hw *hw)
{
	u32 ret_val = 0;

	atl2_init_pcie(hw);

	/* Zero out the Multicast HASH table */
	/* clear the old settings from the multicast hash table */
	ATL2_WRITE_REG(hw, REG_RX_HASH_TABLE, 0);
	ATL2_WRITE_REG_ARRAY(hw, REG_RX_HASH_TABLE, 1, 0);

	atl2_init_flash_opcode(hw);

	ret_val = atl2_phy_init(hw);

	return ret_val;
}

/*
 * Detects the current speed and duplex settings of the hardware.
 *
 * hw - Struct containing variables accessed by shared code
 * speed - Speed of the connection
 * duplex - Duplex setting of the connection
 */
s32
atl2_get_speed_and_duplex(struct atl2_hw *hw, u16 *speed, u16 *duplex)
{
	s32 ret_val;
	u16 phy_data;

	// ; --- Read   PHY Specific Status Register (17)
	ret_val = atl2_read_phy_reg(hw, MII_AT001_PSSR, &phy_data);
	if (ret_val)
		return ret_val;
	
	if (!(phy_data & MII_AT001_PSSR_SPD_DPLX_RESOLVED))
		return ATL2_ERR_PHY_RES;
	
	switch(phy_data & MII_AT001_PSSR_SPEED) {
		case MII_AT001_PSSR_100MBS:
			*speed = SPEED_100;
			break;
		case MII_AT001_PSSR_10MBS:
			*speed = SPEED_10;
			break;
		default:
			return ATL2_ERR_PHY_SPEED;
			break;
	}
	
	if (phy_data & MII_AT001_PSSR_DPLX) {
		*duplex = FULL_DUPLEX;
	} else {
		*duplex = HALF_DUPLEX;
	}

	return ATL2_SUCCESS;
}

/*
 * Reads the value from a PHY register
 * hw - Struct containing variables accessed by shared code
 * reg_addr - address of the PHY register to read
 */
s32
atl2_read_phy_reg(struct atl2_hw *hw, u16 reg_addr, u16 *phy_data)
{
	u32 val;
	int i;

	val = ((u32)(reg_addr & MDIO_REG_ADDR_MASK)) << MDIO_REG_ADDR_SHIFT |
		MDIO_START |
		MDIO_SUP_PREAMBLE |
		MDIO_RW |
		MDIO_CLK_25_4 << MDIO_CLK_SEL_SHIFT;
	ATL2_WRITE_REG(hw, REG_MDIO_CTRL, val);

	wmb();

	for (i=0; i<MDIO_WAIT_TIMES; i++) {
		usec_delay(2);
		val = ATL2_READ_REG(hw, REG_MDIO_CTRL);
		if (!(val & (MDIO_START | MDIO_BUSY))) {
			break;
		}
		wmb();
	}
	if (!(val & (MDIO_START | MDIO_BUSY))) {
		*phy_data = (u16)val;
		return ATL2_SUCCESS;
	}

	return ATL2_ERR_PHY;
}

/*
 * Writes a value to a PHY register
 * hw - Struct containing variables accessed by shared code
 * reg_addr - address of the PHY register to write
 * data - data to write to the PHY
 */
s32
atl2_write_phy_reg(struct atl2_hw *hw, u32 reg_addr, u16 phy_data)
{
	int i;
	u32 val;

	val = ((u32)(phy_data & MDIO_DATA_MASK)) << MDIO_DATA_SHIFT |
		(reg_addr & MDIO_REG_ADDR_MASK) << MDIO_REG_ADDR_SHIFT |
		MDIO_SUP_PREAMBLE |
		MDIO_START |
		MDIO_CLK_25_4 << MDIO_CLK_SEL_SHIFT;
	ATL2_WRITE_REG(hw, REG_MDIO_CTRL, val);

	wmb();

	for (i=0; i<MDIO_WAIT_TIMES; i++) {
		usec_delay(2);
		val = ATL2_READ_REG(hw, REG_MDIO_CTRL);
		if (!(val & (MDIO_START | MDIO_BUSY))) {
			break;
		}
		wmb();
	}

	if (!(val & (MDIO_START | MDIO_BUSY)))
		return ATL2_SUCCESS;

	return ATL2_ERR_PHY;
}

/*
 * Configures PHY autoneg and flow control advertisement settings
 *
 * hw - Struct containing variables accessed by shared code
 */
static s32
atl2_phy_setup_autoneg_adv(struct atl2_hw *hw)
{
	s32 ret_val;
	s16 mii_autoneg_adv_reg;

	/* Read the MII Auto-Neg Advertisement Register (Address 4). */
	mii_autoneg_adv_reg = MII_AR_DEFAULT_CAP_MASK;
	
	/* Need to parse autoneg_advertised  and set up
	 * the appropriate PHY registers.  First we will parse for
	 * autoneg_advertised software override.  Since we can advertise
	 * a plethora of combinations, we need to check each bit
	 * individually.
	 */
	
	/* First we clear all the 10/100 mb speed bits in the Auto-Neg
	 * Advertisement Register (Address 4) and the 1000 mb speed bits in
	 * the  1000Base-T Control Register (Address 9).
	 */
	mii_autoneg_adv_reg &= ~MII_AR_SPEED_MASK;
	
	/* Need to parse MediaType and setup the
	 * appropriate PHY registers.
	 */
	switch (hw->MediaType) {
		case MEDIA_TYPE_AUTO_SENSOR:
			mii_autoneg_adv_reg |=
				(MII_AR_10T_HD_CAPS |
				MII_AR_10T_FD_CAPS |
				MII_AR_100TX_HD_CAPS |
				MII_AR_100TX_FD_CAPS);
			hw->autoneg_advertised =
				ADVERTISE_10_HALF |
				ADVERTISE_10_FULL |
				ADVERTISE_100_HALF |
				ADVERTISE_100_FULL;
			break;
		case MEDIA_TYPE_100M_FULL:
			mii_autoneg_adv_reg |= MII_AR_100TX_FD_CAPS;
			hw->autoneg_advertised = ADVERTISE_100_FULL;
			break;
		case MEDIA_TYPE_100M_HALF:
			mii_autoneg_adv_reg |= MII_AR_100TX_HD_CAPS;
			hw->autoneg_advertised = ADVERTISE_100_HALF;
			break;
		case MEDIA_TYPE_10M_FULL:
			mii_autoneg_adv_reg |= MII_AR_10T_FD_CAPS;
			hw->autoneg_advertised = ADVERTISE_10_FULL;
			break;
		default:
			mii_autoneg_adv_reg |= MII_AR_10T_HD_CAPS;
			hw->autoneg_advertised = ADVERTISE_10_HALF;
			break;
	}

	/* flow control fixed to enable all */
	mii_autoneg_adv_reg |= (MII_AR_ASM_DIR | MII_AR_PAUSE);

	hw->mii_autoneg_adv_reg = mii_autoneg_adv_reg;

	ret_val = atl2_write_phy_reg(hw, MII_ADVERTISE, mii_autoneg_adv_reg);

	if(ret_val)
		return ret_val;

	return ATL2_SUCCESS;
}

/*
 * Resets the PHY and make all config validate
 *
 * hw - Struct containing variables accessed by shared code
 *
 * Sets bit 15 and 12 of the MII Control regiser (for F001 bug)
 */
static s32
atl2_phy_commit(struct atl2_hw *hw)
{
	s32 ret_val;
	u16 phy_data;

/* FIXME: use or remove -- CHS
    if (hw->MediaType == MEDIA_TYPE_AUTO_SENSOR) {
    	phy_data = MII_CR_RESET | MII_CR_AUTO_NEG_EN;
    } else {
        switch (hw->MediaType)
    	{
    	case MEDIA_TYPE_100M_FULL:
    		phy_data = MII_CR_FULL_DUPLEX|MII_CR_SPEED_100|MII_CR_RESET;
    		break;
    	case MEDIA_TYPE_100M_HALF:
    		phy_data = MII_CR_SPEED_100|MII_CR_RESET;
    		break;
    	case MEDIA_TYPE_10M_FULL:
    		phy_data = MII_CR_FULL_DUPLEX|MII_CR_SPEED_10|MII_CR_RESET;
    		break;
    	default: // MEDIA_TYPE_10M_HALF:
    		phy_data = MII_CR_SPEED_10|MII_CR_RESET;
    		break;
    	}
    }
*/
	phy_data = MII_CR_RESET | MII_CR_AUTO_NEG_EN | MII_CR_RESTART_AUTO_NEG;
	ret_val = atl2_write_phy_reg(hw, MII_BMCR, phy_data);
	if (ret_val) { // bug fixed
		u32 val;
		int i;
		/* pcie serdes link may be down ! */
		for (i=0; i < 25; i++) {
			msec_delay(1);
			val = ATL2_READ_REG(hw, REG_MDIO_CTRL);
			if (!(val & (MDIO_START | MDIO_BUSY)))
				break;
		}

		if (0 != (val & (MDIO_START | MDIO_BUSY))) {
			printk(KERN_ERR "atl2: PCIe link down for at least 25ms !\n");
			return ret_val;
		}
	}
	return ATL2_SUCCESS;
}

s32
atl2_phy_init(struct atl2_hw *hw)
{
	s32 ret_val;
	u16 phy_val;

	if (hw->phy_configured)
		return 0;

	/* Enable PHY */
	ATL2_WRITE_REGW(hw, REG_PHY_ENABLE, 1);
	ATL2_WRITE_FLUSH(hw);
	msec_delay(1);

	/* check if the PHY is in powersaving mode */
	atl2_write_phy_reg(hw, MII_DBG_ADDR, 0);
	atl2_read_phy_reg(hw, MII_DBG_DATA, &phy_val);

	/* 024E / 124E 0r 0274 / 1274 ? */
	if (phy_val & 0x1000) {
		phy_val &= ~0x1000;
		atl2_write_phy_reg(hw, MII_DBG_DATA, phy_val);
	}

	msec_delay(1);

	
	/*Enable PHY LinkChange Interrupt */
	ret_val = atl2_write_phy_reg(hw, 18, 0xC00);
	if (ret_val)
		return ret_val;

	/* setup AutoNeg parameters */
	ret_val = atl2_phy_setup_autoneg_adv(hw);
        if(ret_val)
		return ret_val;

	/* SW.Reset & En-Auto-Neg to restart Auto-Neg */
	ret_val = atl2_phy_commit(hw);
	if (ret_val)
		return ret_val;

	hw->phy_configured = true;

	return ret_val;
}

void
atl2_set_mac_addr(struct atl2_hw *hw)
{
	u32 value;
	// 00-0B-6A-F6-00-DC
	// 0:  6AF600DC   1: 000B
	//  low dword
	value = (((u32)hw->mac_addr[2]) << 24) |
		(((u32)hw->mac_addr[3]) << 16) |
		(((u32)hw->mac_addr[4]) << 8 ) |
		(((u32)hw->mac_addr[5])      ) ;
	ATL2_WRITE_REG_ARRAY(hw, REG_MAC_STA_ADDR, 0, value);
	// hight dword
	value = (((u32)hw->mac_addr[0]) << 8 ) |
		(((u32)hw->mac_addr[1])      ) ;
	ATL2_WRITE_REG_ARRAY(hw, REG_MAC_STA_ADDR, 1, value);
}

/*
 * check_eeprom_exist
 * return 0 if eeprom exist
 */
int
atl2_check_eeprom_exist(struct atl2_hw *hw)
{
	u32 value;

	value = ATL2_READ_REG(hw, REG_SPI_FLASH_CTRL);
	if (value & SPI_FLASH_CTRL_EN_VPD) {
		value &= ~SPI_FLASH_CTRL_EN_VPD;
		ATL2_WRITE_REG(hw, REG_SPI_FLASH_CTRL, value);
	}
	value = ATL2_READ_REGW(hw, REG_PCIE_CAP_LIST);
	return ((value & 0xFF00) == 0x6C00) ? 0 : 1;
}

// FIXME: This doesn't look right. -- CHS
bool
atl2_write_eeprom(struct atl2_hw *hw, u32 offset, u32 value)
{
	return true;
}

bool
atl2_read_eeprom(struct atl2_hw *hw, u32 Offset, u32 *pValue)
{
	int i;
	u32    Control;

	if (Offset & 0x3)
		return false; /* address do not align */

	ATL2_WRITE_REG(hw, REG_VPD_DATA, 0);
	Control = (Offset & VPD_CAP_VPD_ADDR_MASK) << VPD_CAP_VPD_ADDR_SHIFT;
	ATL2_WRITE_REG(hw, REG_VPD_CAP, Control);
	
	for (i = 0; i < 10; i++) {
		msec_delay(2);
		Control = ATL2_READ_REG(hw, REG_VPD_CAP);
		if (Control & VPD_CAP_VPD_FLAG)
			break;
	}

	if (Control & VPD_CAP_VPD_FLAG) {
		*pValue = ATL2_READ_REG(hw, REG_VPD_DATA);
		return true;
	}
	return false; /* timeout */
}

void
atl2_force_ps(struct atl2_hw *hw)
{
	u16 phy_val;

	atl2_write_phy_reg(hw, MII_DBG_ADDR, 0);
	atl2_read_phy_reg(hw, MII_DBG_DATA, &phy_val);
	atl2_write_phy_reg(hw, MII_DBG_DATA, phy_val | 0x1000);

	atl2_write_phy_reg(hw, MII_DBG_ADDR, 2);
	atl2_write_phy_reg(hw, MII_DBG_DATA, 0x3000);
	atl2_write_phy_reg(hw, MII_DBG_ADDR, 3);
	atl2_write_phy_reg(hw, MII_DBG_DATA, 0);
}
