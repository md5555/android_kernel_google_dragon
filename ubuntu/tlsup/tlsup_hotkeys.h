/* Toshiba Laptop Support -- tlsup_hotkeys.h
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

#ifndef TLSUP_HOTKEYS_H
#define TLSUP_HOTKEYS_H

/* Look for the hotkey support in the HCI and if found, register with
 * input-polldev to get an input device to provide events to userland
 * with.
 *
 * Returns -errno on error, 0 on success (but no registration) or 1 if
 * an input device was registered.
 */
extern int tlsup_register_hotkeys(void);

/* Remove the registration of the hotkeys input device. */
extern void tlsup_deregister_hotkeys(void);

#endif /* TLSUP_BACKLIGHT_H */
