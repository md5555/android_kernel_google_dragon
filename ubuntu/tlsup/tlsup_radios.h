/* Toshiba Laptop Support -- tlsup_radios.h
 *
 * Radio kill-switch support for T.L.S.
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

#ifndef TLSUP_RADIOS_H
#define TLSUP_RADIOS_H

/* Look for the radio kill switch support and if present, register
 * rfkill devices to manage them. This supports wifi, bluetooth and
 * hsdpa modem.
 *
 * Returns -errno on error, 0 on success (but no registration) or the
 * number of rfkills registered.
 */
extern int tlsup_register_radios(void);

/* Remove the registration of the radios. */
extern void tlsup_deregister_radios(void);

#endif /* TLSUP_RADIOS_H */
