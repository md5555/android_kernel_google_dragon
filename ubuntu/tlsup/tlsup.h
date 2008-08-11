/* Toshiba Laptop Support -- tlsup.h
 *
 * General top level definitions etc for T.L.S.
 *
 * Copyright 2008 Daniel Silverstone <dsilvers@digital-scurf.org>
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; specifically version 2
 * of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301 USA.
 *
 *
 *  Credits:
 *	Jonathan A. Buzzard - Toshiba HCI info, and critical tips on reverse
 *		engineering the Windows drivers
 *	Rob Miller - TV out and hotkeys help
 *      John Belmonte - The toshiba_acpi driver maintainance for some time.
 */

#ifndef TLSUP_H
#define TLSUP_H

#define MODULE_LOGPREFIX "tlsup: "

#define tlsup_info(X...) printk(KERN_INFO MODULE_LOGPREFIX X)
#define tlsup_err(X...) printk(KERN_ERR MODULE_LOGPREFIX X)
#define tlsup_notice(X...) printk(KERN_NOTICE MODULE_LOGPREFIX X)

#include "tlsup_acpi.h"
#include "tlsup_backlight.h"
#include "tlsup_hotkeys.h"
#include "tlsup_radios.h"

#endif /* TLSUP_H */
