/*
 * Copyright (c) 2013-2015, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __AD5823_H__
#define __AD5823_H__

#include <linux/ioctl.h>  /* For IOCTL macros */

#define AD5823_IOCTL_GET_CONFIG		_IOR('o', 1, struct ad5823_config)
#define AD5823_IOCTL_SET_POSITION	_IOW('o', 2, u32)
#define AD5823_IOCTL_SET_CAL_DATA	_IOW('o', 3, struct ad5823_cal_data)
#define AD5823_IOCTL_SET_CONFIG		_IOW('o', 4, struct nv_focuser_config)
#define AD5823_IOCTL_GET_FEATURES	_IOR('o', 5, u32)
#define AD5823_IOCTL_SET_FEATURES	_IOW('o', 6, u32)

#endif  /* __AD5820_H__ */

