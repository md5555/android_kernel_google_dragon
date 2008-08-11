/* Toshiba Laptop Support -- tlsup_hotkeys.c
 *
 * Hotkeys input support for T.L.S.
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
#include <linux/err.h>
#include <linux/input-polldev.h>

#include "tlsup.h"

/* HCI registers */
#define HCI_SYSTEM_EVENT		0x0016

/* Value manipulation */

/* Module stuff */
static int hotkeys_checks_per_second = 4;
module_param(hotkeys_checks_per_second, uint, 0400);
MODULE_PARM_DESC(hotkeys_checks_per_second, "The number of times per second that the kernel will poll the HCI for hotkey events.");

/* Keycode system */

struct key_entry {
        u8 fncode;
        u16 keycode;
};

static struct key_entry tlsup_hotkey_map[] = {
        {2, KEY_FN_1},
        {0, 0}
};
        
static struct key_entry *tlsup_get_entry_by_scancode(int code)
{
        struct key_entry *key;
        
        for (key = tlsup_hotkey_map; key->fncode != 0; key++) {
                if (key->fncode == code)
                        return key;
        }
        
        return NULL;
}

static struct key_entry *tlsup_get_entry_by_keycode(int code)
{
        struct key_entry *key;
        
        for (key = tlsup_hotkey_map; key->fncode != 0; key++) {
                if (key->keycode == code)
                        return key;
        }
        
        return NULL;
}

static int tlsup_get_keycode(struct input_dev *dev, int scancode, int *keycode)
{
        struct key_entry *key = tlsup_get_entry_by_scancode(scancode);
        
        if (key != NULL) {
                *keycode = key->keycode;
                return 0;
        }
        
        return -EINVAL;
}

static int tlsup_set_keycode(struct input_dev *dev, int scancode, int keycode)
{
        struct key_entry *key = tlsup_get_entry_by_scancode(scancode);
        int old_keycode;
        
        if (keycode < 0 || keycode > KEY_MAX)
                return -EINVAL;
        
        if (key != NULL) {
                old_keycode = key->keycode;
                key->keycode = keycode;
                set_bit(keycode, dev->keybit);
                if (tlsup_get_entry_by_keycode(old_keycode) == NULL)
                        clear_bit(old_keycode, dev->keybit);
                return 0;
        }
        
        return -EINVAL;
}

/* Hotkey system */

static struct input_polled_dev *tlsup_polled_dev = NULL;

static int tlsup_read_hotkey_event(void)
{
        u32 value, hci_result;
        if (tlsup_hci_read_1_reg(HCI_SYSTEM_EVENT, &value, &hci_result))
                return -EFAULT;
        
        if (hci_result == HCI_EMPTY) {
                /* better luck next time */
                return 0;
        } else if (hci_result == HCI_NOT_SUPPORTED) {
                /* This is a workaround for an unresolved issue on
                 * some machines where system events sporadically
                 * become disabled. */
                tlsup_hci_write_1_reg(HCI_SYSTEM_EVENT, 1, &hci_result);
                tlsup_info("Had to re-enable hotkeys.\n");
                return 0;
        } else if (hci_result == HCI_SUCCESS) {
                /* Process key */
                if (value == 0x0100) {
                        /* Fn+<nothing> */
                        return 0;
                } else {
                        return value;
                }
        } else {
                /* Unknown result */
                return -EFAULT;
        }
        /* Not reached */
        BUG();
}

static void tlsup_polled_dev_poll(struct input_polled_dev *dev)
{
        int scancode = tlsup_read_hotkey_event();
        struct key_entry *key;
        
        if (scancode <= 0)
                return;
        
        key = tlsup_get_entry_by_scancode(scancode & 0x7F);
        
        if (key != NULL) {
                input_report_key(dev->input, key->keycode,
                                 !(scancode & 0x80));
                input_sync(dev->input);
        }
        
}

/* Registration functions */

int tlsup_register_hotkeys(void)
{
        int ret;
        struct input_dev *input;
        struct key_entry *key;
        
        /* Swallow what's there already */
        while ((ret = tlsup_read_hotkey_event()) > 0);
        
        if (ret < 0)
                return -EFAULT;
        
        
        
        /* Present and results are sane, register the backlight driver. */
        tlsup_polled_dev = input_allocate_polled_device();
        if (tlsup_polled_dev == NULL) {
                tlsup_err("Could not allocate polled device for hotkeys\n");
                return -ENOMEM;
        }
        
        tlsup_polled_dev->private = kmalloc(sizeof(tlsup_hotkey_map), GFP_KERNEL);
        
        tlsup_polled_dev->poll = tlsup_polled_dev_poll;
        tlsup_polled_dev->poll_interval = 1000 / hotkeys_checks_per_second;
        
        /* id name phys bits */
        
        input = tlsup_polled_dev->input;
        
        input->id.bustype = BUS_HOST;
        input->name = "tlsup: Toshiba Fn Hotkeys";
        input->phys = "toshiba/hotkeys0";
        
        input->keycode = tlsup_polled_dev->private;
        input->keycodemax = ARRAY_SIZE(tlsup_hotkey_map);
        input->keycodesize = sizeof(unsigned short);
        
        input->getkeycode = tlsup_get_keycode;
        input->setkeycode = tlsup_set_keycode;

        set_bit(EV_KEY, input->evbit);
        for (key = tlsup_hotkey_map; key->fncode != 0; key++)
                set_bit(key->keycode, input->keybit);
        clear_bit(KEY_RESERVED, input->keybit);
        
        if ((ret = input_register_polled_device(tlsup_polled_dev)) < 0) {
                input_free_polled_device(tlsup_polled_dev);
                tlsup_err("Could not register polled device for hotkeys\n");
                tlsup_polled_dev = NULL;
                return ret;
        }
        
        tlsup_info("Hotkeys polled every %dms.\n", 1000 / hotkeys_checks_per_second);
        
        return 1;
}

void tlsup_deregister_hotkeys(void)
{
        if (tlsup_polled_dev != NULL) {
                input_unregister_polled_device(tlsup_polled_dev);
                input_free_polled_device(tlsup_polled_dev);
                tlsup_polled_dev = NULL;
        }
}
