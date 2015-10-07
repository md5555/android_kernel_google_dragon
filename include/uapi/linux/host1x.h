/*
 * Copyright (c) 2015, NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#ifndef _HOST1X_USER_H_
#define _HOST1X_USER_H_

enum host1x_clock_index {
	HOST1X_CLOCK_INDEX_CBUS,
	HOST1X_CLOCK_INDEX_EMC,
	HOST1X_CLOCK_INDEX_MAX,
};

enum host1x_user_constraint_type {
	HOST1X_USER_CONSTRAINT_TYPE_UNINITIALIZED,
	HOST1X_USER_CONSTRAINT_TYPE_HZ,
	HOST1X_USER_CONSTRAINT_TYPE_BW_KBPS,
	HOST1X_USER_CONSTRAINT_TYPE_PIXELRATE,
};

#endif
