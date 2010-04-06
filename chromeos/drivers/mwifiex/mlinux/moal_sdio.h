/** @file moal_sdio.h
  *
  * @brief This file contains definitions for SDIO interface.
  * driver. 
  *
  * Copyright (C) 2008-2009, Marvell International Ltd. 
  * 
  * This software file (the "File") is distributed by Marvell International 
  * Ltd. under the terms of the GNU General Public License Version 2, June 1991 
  * (the "License").  You may use, redistribute and/or modify this File in 
  * accordance with the terms and conditions of the License, a copy of which 
  * is available by writing to the Free Software Foundation, Inc.,
  * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or on the
  * worldwide web at http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt.
  *
  * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE 
  * IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE 
  * ARE EXPRESSLY DISCLAIMED.  The License provides additional details about 
  * this warranty disclaimer.
  *
  */
/****************************************************
Change log:
****************************************************/

#ifndef	_MOAL_SDIO_H
#define	_MOAL_SDIO_H

#include        <linux/mmc/sdio.h>
#include        <linux/mmc/sdio_ids.h>
#include        <linux/mmc/sdio_func.h>
#include        <linux/mmc/card.h>

#include "moal_main.h"

#ifndef BLOCK_MODE
/** Block mode */
#define BLOCK_MODE	1
#endif

#ifndef BYTE_MODE
/** Byte Mode */
#define BYTE_MODE	0
#endif

#ifndef FIXED_ADDRESS
/** Fixed address mode */
#define FIXED_ADDRESS	0
#endif

/** Default firmware name */

#define DEFAULT_FW_NAME "mrvl/sd8787.bin"

#ifndef DEFAULT_FW_NAME
#define DEFAULT_FW_NAME ""
#endif

/********************************************************
		Global Functions
********************************************************/

/** Function to write register */
mlan_status woal_write_reg(moal_handle * handle, t_u32 reg, t_u32 data);
/** Function to read register */
mlan_status woal_read_reg(moal_handle * handle, t_u32 reg, t_u32 * data);
/** Function to write data to IO memory */
mlan_status woal_write_data_sync(moal_handle * handle, mlan_buffer * pmbuf,
                                 t_u32 port, t_u32 timeout);
/** Function to read data from IO memory */
mlan_status woal_read_data_sync(moal_handle * handle, mlan_buffer * pmbuf,
                                t_u32 port, t_u32 timeout);

/** Register to bus driver function */
mlan_status woal_bus_register(void);
/** Unregister from bus driver function */
void woal_bus_unregister(void);

/** Register device function */
mlan_status woal_register_dev(moal_handle * handle);
/** Unregister device function */
void woal_unregister_dev(moal_handle * handle);

int woal_sdio_set_bus_clock(moal_handle * handle, t_u8 option);
/** Structure: SDIO MMC card */
struct sdio_mmc_card
{
        /** sdio_func structure pointer */
    struct sdio_func *func;
        /** moal_handle structure pointer */
    moal_handle *handle;
};

#endif /* _MOAL_SDIO_H */
