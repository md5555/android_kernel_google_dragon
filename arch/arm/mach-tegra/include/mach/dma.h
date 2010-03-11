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

enum tegra_dma_req_error {
	TEGRA_DMA_REQ_SUCCESS = 0,
	TEGRA_DMA_REQ_ERROR_ABOTRED,
};

enum tegra_dma_req_buff_status {
	TEGRA_DMA_REQ_BUF_STATUS_EMPTY,
	TEGRA_DMA_REQ_BUF_STATUS_HALF_FULL,
	TEGRA_DMA_REQ_BUF_STATUS_FULL,
};

struct tegra_dma_req {
	struct list_head list;
	unsigned int modid;
	int instance;

	/* Called when the req is complete and from the DMA ISR context.
	 * When this is called the req structure is no longer queued by
	 * the DMA channel.
	 *
	 * State of the DMA depends on the number of req it has. If there are
	 * no DMA requests queued up, then it will STOP the DMA. It there are
	 * more requests in the DMA, then it will queue the next request.
	 */
	void (*complete)(struct tegra_dma_req *req, int err);

	/*  This is a called from the DMA ISR context when the DMA is still in
	 *  progress and is actively filling same buffer.
	 *
	 *  In case of continous mode receive, this threshold is 1/2 the buffer
	 *  size. In other cases, this will not even be called as there is no
	 *  hardware support for it.
	 *
	 * In the case of continous mode receive, if there is next req already
	 * queued, DMA programs the HW to use that req when this req is
	 * completed. If there is no "next req" queued, then DMA ISR doesn't do
	 * anything before calling this callback.
	 *
	 *	This is mainly used by the cases, where the clients has queued
	 *	only one req and want to get some sort of DMA threshold
	 *	callback to program the next buffer.
	 *
	 */
	void (*threshold)(struct tegra_dma_req *req, int err);

	/* 1 to copy to memory.
	 * 0 to copy from the memory to device FIFO */
	int to_memory;

	void *virt_addr;

	unsigned long source_addr;
	unsigned long dest_addr;
	unsigned long dest_wrap;
	unsigned long source_wrap;

	unsigned int size;

	/* Updated by the DMA driver on the conpletion of the request. */
	int bytes_transferred;
	int status;

	/* DMA completion tracking information */
	int buffer_status;

	/* Client specific data */
	void *data;
};

int tegra_dma_enqueue_req(int channel, struct tegra_dma_req *req);
int tegra_dma_dequeue_req(int channel, struct tegra_dma_req *req);
void tegra_dma_dequeue(int channel);
void tegra_dma_flush(int channel);

bool tegra_dma_is_req_inflight(int channel, struct tegra_dma_req *req);
bool tegra_dma_is_empty(int channel);

int tegra_dma_allocate_channel(int mode);
void tegra_dma_free_channel(int channel);

int __init tegra_dma_init(void);

#endif

#endif
