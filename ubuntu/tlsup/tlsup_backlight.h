/* Toshiba Laptop Support -- tlsup_backlight.h
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

#ifndef TLSUP_BACKLIGHT_H
#define TLSUP_BACKLIGHT_H

/* Look for the LCD backlight support and if present, register a backlight
 * class device to manage it.
 *
 * Returns -errno on error, 0 on success (but no registration) or 1 if
 * a backlight was registered.
 */
extern int tlsup_register_backlight(void);

/* Remove the registration of the backlight. */
extern void tlsup_deregister_backlight(void);

#endif /* TLSUP_BACKLIGHT_H */
