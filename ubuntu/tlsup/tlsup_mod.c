/* Toshiba Laptop Support -- tlsup_mod.c
 *
 * Aggregation and module structural support for T.L.S.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>

#include "tlsup.h"

MODULE_AUTHOR("Daniel Silverstone");
MODULE_DESCRIPTION("Toshiba Laptop Support Driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(TLSUP_VERSION);

/* The following ACPI devices tend to be exported by toshiba laptops */
MODULE_ALIAS("acpi:TOS6200");
MODULE_ALIAS("acpi:TOS6207");
MODULE_ALIAS("acpi:TOS6208");

static void toshiba_laptop_support_exit(void)
{
        tlsup_deregister_radios();
        tlsup_deregister_hotkeys();
        tlsup_deregister_backlight();
}

static int __init toshiba_laptop_support_init(void)
{
        int ret;
        
        if ((ret = tlsup_acpi_detect()) < 0)
                return ret;
        
        tlsup_info("Toshiba Laptop Support version %s initialising...\n",
                   TLSUP_VERSION);
        
        tlsup_acpi_report();
        
        if ((ret = tlsup_register_backlight()) < 0)
                return ret;
        
        if ((ret = tlsup_register_hotkeys()) < 0) {
                tlsup_deregister_backlight();
                return ret;
        }
        
        if ((ret = tlsup_register_radios()) < 0) {
                tlsup_deregister_hotkeys();
                tlsup_deregister_backlight();
                return ret;
        }
        
        tlsup_info("Toshiba Laptop Support version %s loaded.\n",
                   TLSUP_VERSION);
        
        return 0;
}

module_init(toshiba_laptop_support_init);
module_exit(toshiba_laptop_support_exit);
