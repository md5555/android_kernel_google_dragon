/* atl2_osdep.h -- atl2 compat cruft
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

#ifndef _ATL2_OSDEP_H_
#define _ATL2_OSDEP_H_

#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/if_ether.h>

#define usec_delay(x) udelay(x)
#ifndef msec_delay
#define msec_delay(x) do { \
	if(in_interrupt()) BUG(); \
	else msleep(x); \
	} while (0)

/* Some workarounds require millisecond delays and are run during interrupt
 * context.  Most notably, when establishing link, the phy may need tweaking
 * but cannot process phy register reads/writes faster than millisecond
 * intervals...and we establish link due to a "link status change" interrupt.
 */
#define msec_delay_irq(x) mdelay(x)
#endif

#define PCI_COMMAND_REGISTER	PCI_COMMAND
#define CMD_MEM_WRT_INVALIDATE	PCI_COMMAND_INVALIDATE
#define ETH_ADDR_LEN		ETH_ALEN

#define ATL2_WRITE_REG(a, reg, value) (writel((value), ((a)->hw_addr + reg)))

#define ATL2_WRITE_FLUSH(a) (readl((a)->hw_addr))

#define ATL2_READ_REG(a, reg) (readl((a)->hw_addr + reg))

#define ATL2_WRITE_REGB(a, reg, value) (writeb((value), ((a)->hw_addr + reg)))

#define ATL2_READ_REGB(a, reg) (readb((a)->hw_addr + reg))

#define ATL2_WRITE_REGW(a, reg, value) (writew((value), ((a)->hw_addr + reg)))

#define ATL2_READ_REGW(a, reg) (readw((a)->hw_addr + reg))

#define ATL2_WRITE_REG_ARRAY(a, reg, offset, value) \
	(writel((value), (((a)->hw_addr + reg) + ((offset) << 2))))

#define ATL2_READ_REG_ARRAY(a, reg, offset) \
	(readl(((a)->hw_addr + reg) + ((offset) << 2)))

#endif /* _ATL2_OSDEP_H_ */
