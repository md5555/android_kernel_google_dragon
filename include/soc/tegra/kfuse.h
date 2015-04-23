/*
 * Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#ifndef SOC_TEGRA_KFUSE_H
#define SOC_TEGRA_KFUSE_H

struct tegra_kfuse;

struct tegra_kfuse *tegra_kfuse_find_by_of_node(struct device_node *np);
ssize_t tegra_kfuse_read(struct tegra_kfuse *kfuse, void *buffer, size_t size);

#endif
