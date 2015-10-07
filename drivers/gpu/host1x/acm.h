/*
 * Copyright (C) 2015, NVIDIA Corporation
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

#ifndef HOST1X_ACM_H
#define HOST1X_ACM_H

#include <linux/host1x.h>

int host1x_module_add_user(struct host1x_client *client,
			   struct host1x_user *userctx);

void host1x_module_remove_user(struct host1x_client *client,
			       struct host1x_user *userctx);

#endif
