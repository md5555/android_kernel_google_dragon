/*
 * arch/arm/mach-tegra/include/mach/dma.h
 *
 * Copyright (c) 2008-2009, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef __MACH_TEGRA_DMA_H

#if defined(CONFIG_TEGRA_SYSTEM_DMA)

struct tegra_dma_req;

enum tegra_dma_mode {
	TEGRA_DMA_SHARED = 1,
	TEGRA_DMA_MODE_CONTINOUS = 2,
	TEGRA_DMA_MODE_ONESHOT = 4,
};

struct tegra_dma_req {
	struct list_head list;
	unsigned int modid;
	int instace;

	void (*complete)(struct tegra_dma_req *req, int bytes_trasferred,
		int err);
	/* 1 to copy to memory.
	 * 0 to copy from the memory to device FIFO */
	int to_memory;

	unsigned long source_addr;
	unsigned long dest_addr;
	unsigned long dest_wrap;
	unsigned long source_wrap;

	unsigned int size;

	/* Client specific data */
	void *data;
};

void tegra_dma_stop(int channel);
void tegra_dma_is_enabled(int channel);
void tegra_dma_start(int channel);

int tegra_dma_enqueue_req(int channel, struct tegra_dma_req *req);

/*  Returns 1 if there are DMA is empty.
 */
int tegra_dma_is_empty(int channel);
void tegra_dma_flush(int channel);

int tegra_dma_allocate_channel(int mode);
void tegra_dma_free_channel(int channel);

int __init tegra_dma_init(void);

#endif

#endif
