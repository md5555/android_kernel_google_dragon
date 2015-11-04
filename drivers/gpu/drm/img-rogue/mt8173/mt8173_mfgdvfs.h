/*
* Copyright (c) 2014 MediaTek Inc.
* Author: Chiawen Lee <chiawen.lee@mediatek.com>
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*/
#ifndef MT8173_MFGDVFS_H
#define MT8173_MFGDVFS_H

/* us, (DA9212 externel buck) I2C command delay 100us, Buck 10mv/us */
#define GPU_DVFS_VOLT_SETTLE_TIME(uv_old, uv_new) \
	(((((uv_old) > (uv_new)) ? \
	((uv_old)-(uv_new)) : ((uv_new)-(uv_old))) / 1000 + 9) / 10 + 100)

#define GPU_VOLT_TO_EXTBUCK_MAXVAL(volt) (volt + 9999)

#endif /* MT8173_MFGDVFS_H */
