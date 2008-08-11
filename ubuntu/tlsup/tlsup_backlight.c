/* Toshiba Laptop Support -- tlsup_backlight.c
 *
 * LCD Backlight support for T.L.S.
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
#include <linux/backlight.h>
#include <linux/err.h>

#include "tlsup.h"

/* HCI registers */
#define HCI_REG_LCD_BRIGHTNESS		0x002a

/* Value manipulation */
#define HCI_LCD_BRIGHTNESS_BITS		3
#define HCI_LCD_BRIGHTNESS_SHIFT	(16-HCI_LCD_BRIGHTNESS_BITS)
#define HCI_LCD_BRIGHTNESS_LEVELS	(1 << HCI_LCD_BRIGHTNESS_BITS)


/* Backlight driver */
static struct backlight_device *tlsup_backlight_device = NULL;

static int tlsup_read_brightness(struct backlight_device *bd)
{
	u32 hci_result;
	u32 value;

        if (tlsup_hci_read_1_reg(HCI_REG_LCD_BRIGHTNESS, &value, &hci_result))
                return -EFAULT;
        
        if (hci_result != HCI_SUCCESS)
                return -EFAULT;
        
        return (value >> HCI_LCD_BRIGHTNESS_SHIFT);
}

static int tlsup_set_brightness(struct backlight_device *bd)
{
        u32 hci_result;
        u32 value = bd->props.brightness << HCI_LCD_BRIGHTNESS_SHIFT;
        
        if (tlsup_hci_write_1_reg(HCI_REG_LCD_BRIGHTNESS, value, &hci_result))
                return -EFAULT;
        
        if (hci_result != HCI_SUCCESS)
                return -EFAULT;
        
        return 0;
}

static struct backlight_ops tlsup_backlight_data = {
        .get_brightness = tlsup_read_brightness,
        .update_status = tlsup_set_brightness,
};

/* Registration functions */

int tlsup_register_backlight(void)
{
        u32 hci_result, ignored;
        if (tlsup_hci_read_1_reg(HCI_REG_LCD_BRIGHTNESS, &ignored, &hci_result))
                return -EFAULT;
        if (hci_result != HCI_SUCCESS)
                return 0;
        /* Present and results are sane, register the backlight driver. */
        tlsup_backlight_device = backlight_device_register("toshiba",
                                                           NULL,
                                                           NULL,
                                                           &tlsup_backlight_data);
        
        if (IS_ERR(tlsup_backlight_device)) {
                int ret = PTR_ERR(tlsup_backlight_device);
                tlsup_err("Could not register backlight device\n");
                tlsup_backlight_device = NULL;
                return ret;
        }
        
	tlsup_backlight_device->props.max_brightness = HCI_LCD_BRIGHTNESS_LEVELS - 1;
        
        return 1;
}

void tlsup_deregister_backlight(void)
{
        if (tlsup_backlight_device)
		backlight_device_unregister(tlsup_backlight_device);
}
