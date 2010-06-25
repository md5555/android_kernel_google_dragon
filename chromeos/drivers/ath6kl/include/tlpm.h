//------------------------------------------------------------------------------
// <copyright file="wlan_dset.h" company="Atheros">
//    Copyright (c) 2010 Atheros Corporation.  All rights reserved.
// 
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License version 2 as
// published by the Free Software Foundation;
//
// Software distributed under the License is distributed on an "AS
// IS" basis, WITHOUT WARRANTY OF ANY KIND, either express or
// implied. See the License for the specific language governing
// rights and limitations under the License.
//
//
//------------------------------------------------------------------------------
//==============================================================================
// Author(s): ="Atheros"
//==============================================================================

#ifndef __TLPM_H__
#define __TLPM_H__

/* idle timeout in 16-bit value as in HOST_INTEREST hi_hci_uart_pwr_mgmt_params */
#define TLPM_DEFAULT_IDLE_TIMEOUT_MS         1000
/* hex in LSB and MSB for HCI command */
#define TLPM_DEFAULT_IDLE_TIMEOUT_LSB        0xE8
#define TLPM_DEFAULT_IDLE_TIMEOUT_MSB        0x3

/* wakeup timeout in 8-bit value as in HOST_INTEREST hi_hci_uart_pwr_mgmt_params */
#define TLPM_DEFAULT_WAKEUP_TIMEOUT_MS       10

/* default UART FC polarity is low */
#define TLPM_DEFAULT_UART_FC_POLARITY        0

#endif
