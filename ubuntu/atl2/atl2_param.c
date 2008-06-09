/* atl2_param.c -- atl2 parameter processing
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

#include <linux/netdevice.h>
#include "atl2.h"

/* This is the only thing that needs to be changed to adjust the
 * maximum number of ports that the driver can manage.
 */
#define ATL2_MAX_NIC 4

#define OPTION_UNSET    -1
#define OPTION_DISABLED 0
#define OPTION_ENABLED  1

/* All parameters are treated the same, as an integer array of values.
 * This macro just reduces the need to repeat the same declaration code
 * over and over (plus this helps to avoid typo bugs).
 */
#define ATL2_PARAM_INIT { [0 ... ATL2_MAX_NIC] = OPTION_UNSET }
#ifndef module_param_array
/* Module Parameters are always initialized to -1, so that the driver
 * can tell the difference between no user specified value or the
 * user asking for the default value.
 * The true default values are loaded in when atl2_check_options is called.
 *
 * This is a GCC extension to ANSI C.
 * See the item "Labeled Elements in Initializers" in the section
 * "Extensions to the C Language Family" of the GCC documentation.
 */

#define ATL2_PARAM(X, desc) \
    static const int __devinitdata X[ATL2_MAX_NIC + 1] = ATL2_PARAM_INIT; \
    MODULE_PARM(X, "1-" __MODULE_STRING(ATL2_MAX_NIC) "i"); \
    MODULE_PARM_DESC(X, desc);
#else
#define ATL2_PARAM(X, desc) \
    static int __devinitdata X[ATL2_MAX_NIC+1] = ATL2_PARAM_INIT; \
    static int num_##X = 0; \
    module_param_array_named(X, X, int, &num_##X, 0); \
    MODULE_PARM_DESC(X, desc);
#endif

/* Transmit Memory Size
 *
 * Valid Range: 64-2048
 *
 * Default Value: 128
 */
#define ATL2_MIN_TX_MEMSIZE		4	// 4KB
#define ATL2_MAX_TX_MEMSIZE		64	// 64KB
#define ATL2_DEFAULT_TX_MEMSIZE		8	// 8KB
ATL2_PARAM(TxMemSize, "Bytes of Transmit Memory");

/* Receive Memory Block Count
 *
 * Valid Range: 16-512
 *
 * Default Value: 128
 */
#define ATL2_MIN_RXD_COUNT		16
#define ATL2_MAX_RXD_COUNT		512
#define ATL2_DEFAULT_RXD_COUNT		64
ATL2_PARAM(RxMemBlock, "Number of receive memory block");

/* User Specified MediaType Override
 *
 * Valid Range: 0-5
 *  - 0    - auto-negotiate at all supported speeds
 *  - 1    - only link at 1000Mbps Full Duplex
 *  - 2    - only link at 100Mbps Full Duplex
 *  - 3    - only link at 100Mbps Half Duplex
 *  - 4    - only link at 10Mbps Full Duplex
 *  - 5    - only link at 10Mbps Half Duplex
 * Default Value: 0
 */
ATL2_PARAM(MediaType, "MediaType Select");

/* Interrupt Moderate Timer in units of 2 us
 *
 * Valid Range: 10-65535
 *
 * Default Value: 45000(90ms)
 */
#define INT_MOD_DEFAULT_CNT	100 // 200us
#define INT_MOD_MAX_CNT		65000
#define INT_MOD_MIN_CNT		50
ATL2_PARAM(IntModTimer, "Interrupt Moderator Timer");

/* FlashVendor
 * Valid Range: 0-2
 * 0 - Atmel
 * 1 - SST
 * 2 - ST
 */
ATL2_PARAM(FlashVendor, "SPI Flash Vendor");

#define AUTONEG_ADV_DEFAULT	0x2F
#define AUTONEG_ADV_MASK	0x2F
#define FLOW_CONTROL_DEFAULT	FLOW_CONTROL_FULL

#define FLASH_VENDOR_DEFAULT	0
#define FLASH_VENDOR_MIN	0
#define FLASH_VENDOR_MAX	2

struct atl2_option {
	enum { enable_option, range_option, list_option } type;
	char *name;
	char *err;
	int  def;
	union {
		struct { /* range_option info */
			int min;
			int max;
		} r;
		struct { /* list_option info */
			int nr;
			struct atl2_opt_list { int i; char *str; } *p;
		} l;
	} arg;
};

static int __devinit
atl2_validate_option(int *value, struct atl2_option *opt)
{
	int i;
	struct atl2_opt_list *ent;

	if(*value == OPTION_UNSET) {
		*value = opt->def;
		return 0;
	}

	switch (opt->type) {
		case enable_option:
			switch (*value) {
				case OPTION_ENABLED:
					printk(KERN_INFO "%s Enabled\n", opt->name);
					return 0;
					break;
				case OPTION_DISABLED:
					printk(KERN_INFO "%s Disabled\n", opt->name);
					return 0;
					break;
			}
			break;
		case range_option:
			if(*value >= opt->arg.r.min && *value <= opt->arg.r.max) {
				printk(KERN_INFO "%s set to %i\n", opt->name, *value);
				return 0;
			}
			break;
		case list_option:
			for(i = 0; i < opt->arg.l.nr; i++) {
				ent = &opt->arg.l.p[i];
				if(*value == ent->i) {
					if(ent->str[0] != '\0')
						printk(KERN_INFO "%s\n", ent->str);
				return 0;
				}
			}
			break;
		default:
			BUG();
	}

	printk(KERN_INFO "Invalid %s specified (%i) %s\n",
		opt->name, *value, opt->err);
	*value = opt->def;
	return -1;
}

/**
 * atl2_check_options - Range Checking for Command Line Parameters
 * @adapter: board private structure
 *
 * This routine checks all command line parameters for valid user
 * input.  If an invalid value is given, or if no user specified
 * value exists, a default value is used.  The final value is stored
 * in a variable in the adapter structure.
 **/
void __devinit
atl2_check_options(struct atl2_adapter *adapter)
{
	int val;
	struct atl2_option opt;
	int bd = adapter->bd_number;
	if(bd >= ATL2_MAX_NIC) {
		printk(KERN_NOTICE "Warning: no configuration for board #%i\n", bd);
		printk(KERN_NOTICE "Using defaults for all values\n");
#ifndef module_param_array
		bd = ATL2_MAX_NIC;
#endif
	}

	/* Bytes of Transmit Memory */
	opt.type = range_option;
	opt.name = "Bytes of Transmit Memory";
	opt.err = "using default of " __MODULE_STRING(ATL2_DEFAULT_TX_MEMSIZE);
	opt.def = ATL2_DEFAULT_TX_MEMSIZE;
	opt.arg.r.min = ATL2_MIN_TX_MEMSIZE;
	opt.arg.r.max = ATL2_MAX_TX_MEMSIZE;
#ifdef module_param_array
	if(num_TxMemSize > bd) {
#endif
		val = TxMemSize[bd];
		atl2_validate_option(&val, &opt);
		adapter->txd_ring_size = ((u32) val) * 1024;
#ifdef module_param_array
        } else {
		adapter->txd_ring_size = ((u32)opt.def) * 1024;
	}
#endif
	// txs ring size:
	adapter->txs_ring_size = adapter->txd_ring_size / 128;
	if (adapter->txs_ring_size > 160)
		adapter->txs_ring_size = 160;

	/* Receive Memory Block Count */
	opt.type = range_option;
	opt.name = "Number of receive memory block";
	opt.err = "using default of " __MODULE_STRING(ATL2_DEFAULT_RXD_COUNT);
	opt.def = ATL2_DEFAULT_RXD_COUNT;
	opt.arg.r.min = ATL2_MIN_RXD_COUNT;
	opt.arg.r.max = ATL2_MAX_RXD_COUNT;
#ifdef module_param_array
	if(num_RxMemBlock > bd) {
#endif
		val = RxMemBlock[bd];
		atl2_validate_option(&val, &opt);
		adapter->rxd_ring_size = (u32)val; //((u16)val)&~1; // even number
#ifdef module_param_array
	} else {
		adapter->rxd_ring_size = (u32)opt.def;
	}
#endif
	// init RXD Flow control value
	adapter->hw.fc_rxd_hi = (adapter->rxd_ring_size/8)*7;
	adapter->hw.fc_rxd_lo = (ATL2_MIN_RXD_COUNT/8) > (adapter->rxd_ring_size/12) ?
		(ATL2_MIN_RXD_COUNT/8) : (adapter->rxd_ring_size/12);

	/* Interrupt Moderate Timer */
	opt.type = range_option;
	opt.name = "Interrupt Moderate Timer";
	opt.err = "using default of " __MODULE_STRING(INT_MOD_DEFAULT_CNT);
	opt.def = INT_MOD_DEFAULT_CNT;
	opt.arg.r.min = INT_MOD_MIN_CNT;
	opt.arg.r.max = INT_MOD_MAX_CNT;
#ifdef module_param_array
	if(num_IntModTimer > bd) {
#endif
		val = IntModTimer[bd];
		atl2_validate_option(&val, &opt);
		adapter->imt = (u16) val;
#ifdef module_param_array
	} else {
		adapter->imt = (u16)(opt.def);
	}
#endif
	/* Flash Vendor */
	opt.type = range_option;
	opt.name = "SPI Flash Vendor";
	opt.err = "using default of " __MODULE_STRING(FLASH_VENDOR_DEFAULT);
	opt.def = FLASH_VENDOR_DEFAULT;
	opt.arg.r.min = FLASH_VENDOR_MIN;
	opt.arg.r.max = FLASH_VENDOR_MAX;
#ifdef module_param_array
	if(num_FlashVendor > bd) {
#endif
		val = FlashVendor[bd];
		atl2_validate_option(&val, &opt);
		adapter->hw.flash_vendor = (u8) val;
#ifdef module_param_array
	} else {
		adapter->hw.flash_vendor = (u8)(opt.def);
	}
#endif
	/* MediaType */
	opt.type = range_option;
	opt.name = "Speed/Duplex Selection";
	opt.err = "using default of " __MODULE_STRING(MEDIA_TYPE_AUTO_SENSOR);
	opt.def = MEDIA_TYPE_AUTO_SENSOR;
	opt.arg.r.min = MEDIA_TYPE_AUTO_SENSOR;
	opt.arg.r.max = MEDIA_TYPE_10M_HALF;
#ifdef module_param_array
	if(num_MediaType > bd) {
#endif
		val = MediaType[bd];
		atl2_validate_option(&val, &opt);
		adapter->hw.MediaType = (u16) val;
#ifdef module_param_array
	} else {
		adapter->hw.MediaType = (u16)(opt.def);
	}
#endif
}
