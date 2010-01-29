/*
 * arch/arm/mach-tegra/dma.c
 *
 * System DMA driver for NVIDIA Tegra SoCs
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

#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <mach/dma.h>
#include <mach/irqs.h>
#include <linux/irq.h>

#include "nvos.h"
#include "mach/nvrm_linux.h"
#include "nvassert.h"
#include "nvrm_drf.h"
#include "nvrm_dma.h"
#include "ap20/arapbdmachan.h"
#include "nvrm_interrupt.h"

#define NV_DMA_MAX_TRASFER_SIZE 0x10000

const unsigned int ahb_addr_wrap_table[8] =
	{0, 32, 64, 128, 256, 512,1024, 2048};

const unsigned int apb_addr_wrap_table[8] =
	{0, 1, 2, 4, 8, 16, 32, 64};

const u8 uart_selector_values[] = {
	APBDMACHAN_CHANNEL_0_CSR_0_REQ_SEL_UART_A,
	APBDMACHAN_CHANNEL_0_CSR_0_REQ_SEL_UART_B,
	APBDMACHAN_CHANNEL_0_CSR_0_REQ_SEL_UART_C,
	APBDMACHAN_CHANNEL_0_CSR_0_REQ_SEL_UART_D,
	APBDMACHAN_CHANNEL_0_CSR_0_REQ_SEL_UART_E,
};

const u8 i2s_selector_values[] = {
	APBDMACHAN_CHANNEL_0_CSR_0_REQ_SEL_I2S_1,
	APBDMACHAN_CHANNEL_0_CSR_0_REQ_SEL_I2S_2,
};

const u8 slink_selector_values[] = {
	APBDMACHAN_CHANNEL_0_CSR_0_REQ_SEL_SL2B1,
	APBDMACHAN_CHANNEL_0_CSR_0_REQ_SEL_SL2B2,
	APBDMACHAN_CHANNEL_0_CSR_0_REQ_SEL_SL2B3,
};

const u8 i2c_selector_values[] = {
	APBDMACHAN_CHANNEL_0_CSR_0_REQ_SEL_I2C,
	APBDMACHAN_CHANNEL_0_CSR_0_REQ_SEL_I2C2,
	APBDMACHAN_CHANNEL_0_CSR_0_REQ_SEL_I2C3,
};

struct tegra_dma_channel {
	struct list_head	list;
	int			id;
	spinlock_t		lock;
	char			*name;
	volatile void  __iomem	*addr;
	unsigned long		phys_addr;
	int			mode;

	/* Register shadow */
	unsigned long		csr;
	unsigned long		ahb_seq;
	unsigned long		ahb_ptr;
	unsigned long		apb_seq;
	unsigned long		apb_ptr;
};

#define  NV_DMA_MAX_CHANNELS  32

static DECLARE_BITMAP(channel_usage, NV_DMA_MAX_CHANNELS);
static struct tegra_dma_channel dma_channels[NV_DMA_MAX_CHANNELS];

static void tegra_dma_update_hw(struct tegra_dma_channel *ch,
	struct tegra_dma_req *req);
static void tegra_dma_update_hw_partial(struct tegra_dma_channel *ch,
	struct tegra_dma_req *req);
static void tegra_dma_init_hw(struct tegra_dma_channel *ch);
static void tegra_dma_stop(struct tegra_dma_channel *ch);

void tegra_dma_flush(int channel)
{
}
EXPORT_SYMBOL(tegra_dma_flush);

void tegra_dma_dequeue(int channel)
{
	struct tegra_dma_channel *ch = &dma_channels[channel];
	struct tegra_dma_req *req;
	req = list_entry(ch->list.next, typeof(*req), list);

	tegra_dma_dequeue_req(channel, req);
	return;
}

void tegra_dma_stop(struct tegra_dma_channel *ch)
{
	unsigned int csr;
	unsigned int status;

	csr = ch->csr;
	csr = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, CSR, IE_EOC,
		DISABLE, csr);
	writel(csr, ch->addr + APBDMACHAN_CHANNEL_0_CSR_0);

	csr = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, CSR, ENB, DISABLE, csr);
	writel(csr, ch->addr + APBDMACHAN_CHANNEL_0_CSR_0);

	status = readl(ch->addr + APBDMACHAN_CHANNEL_0_STA_0);
	if (status & NV_DRF_DEF(APBDMACHAN_CHANNEL_0, STA, ISE_EOC, INTR))
		writel(status, ch->addr + APBDMACHAN_CHANNEL_0_STA_0);
}

int tegra_dma_dequeue_req(int channel, struct tegra_dma_req *_req)
{
	struct tegra_dma_channel *ch = &dma_channels[channel];
	unsigned int csr;
	unsigned int status;
	struct tegra_dma_req *req = NULL;
	int found = 0;
	unsigned long irq_flags;
	int to_transfer;
	int req_transfer_count;

	spin_lock_irqsave(&ch->lock, irq_flags);
	list_for_each_entry (req, &ch->list, list) {
		if (req == _req) {
			list_del(&req->list);
			found = 1;
			break;
		}
	}
	if (found==0) {
		spin_unlock_irqrestore(&ch->lock, irq_flags);
		return 0;
	}

	/* STOP the DMA and get the transfer count.
	 * Getting the transfer count is tricky.
	 *  - Change the source selector to invalid to stop the DMA from
	 *    FIFO to memory.
	 *  - Read the status register to know the number of pending
	 *    bytes to be transfered.
	 *  - Finally stop or program the DMA to the next buffer in the
	 *    list.
	 */
	csr = ch->csr;
	csr = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, CSR,
		REQ_SEL, NA31, csr);
	/* Set the enable as that is not shadowed */
	csr = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, CSR,
		ENB, ENABLE, csr);
	writel(csr, ch->addr + APBDMACHAN_CHANNEL_0_CSR_0);

	/* Get the transfer count */
	status = readl(ch->addr + APBDMACHAN_CHANNEL_0_STA_0);
	to_transfer = NV_DRF_VAL(APBDMACHAN_CHANNEL_0, STA,
		COUNT, status);
	req_transfer_count = NV_DRF_VAL(APBDMACHAN_CHANNEL_0,
		CSR, WCOUNT, ch->csr);

	req->bytes_transferred = req_transfer_count - to_transfer;
	req->bytes_transferred *= 4;
	/* In continous transfer mode, DMA only tracks the count of the
	 * half DMA buffer. So, if the DMA already finished half the DMA
	 * then add the half buffer to the completed count.
	 *
	 *	FIXME: There can be a race here. What if the req to
	 *	dequue happens at the same time as the DMA just moved to
	 *	the new buffer and SW didn't yet received the interrupt?
	 */
	if (ch->mode & TEGRA_DMA_MODE_CONTINOUS)
		if (req->buffer_status == TEGRA_DMA_REQ_BUF_STATUS_HALF_FULL) {
			req->bytes_transferred += 4 * req_transfer_count;
		}

	tegra_dma_stop(ch);
	if (!list_empty(&ch->list)) {
		/* if the list is not empty, queue the next request */
		struct tegra_dma_req *next_req;
		next_req = list_entry(ch->list.next,
			typeof(*next_req), list);
		tegra_dma_update_hw(ch, next_req);
	}
	req->status = -TEGRA_DMA_REQ_ERROR_ABOTRED;

	spin_unlock_irqrestore(&ch->lock, irq_flags);

	/* Callback should be called without any lock */
	req->complete(req, req->status);
	return 0;
}
EXPORT_SYMBOL(tegra_dma_dequeue_req);

bool tegra_dma_is_empty(int channel)
{
	unsigned long irq_flags;
	struct tegra_dma_channel *ch = &dma_channels[channel];
	bool is_empty;

	spin_lock_irqsave(&ch->lock, irq_flags);
	if (list_empty(&ch->list))
		is_empty = true;
	else
		is_empty = false;
	spin_unlock_irqrestore(&ch->lock, irq_flags);
	return is_empty;
}
EXPORT_SYMBOL(tegra_dma_is_empty);

bool tegra_dma_is_req_inflight(int channel, struct tegra_dma_req *_req)
{
	unsigned long irq_flags;
	struct tegra_dma_channel *ch = &dma_channels[channel];
	struct tegra_dma_req *req;

	spin_lock_irqsave(&ch->lock, irq_flags);
	list_for_each_entry (req, &ch->list, list) {
		if (req == _req) {
			spin_unlock_irqrestore(&ch->lock, irq_flags);
			return true;
		}
	}
	spin_unlock_irqrestore(&ch->lock, irq_flags);
	return false;
}
EXPORT_SYMBOL(tegra_dma_is_req_inflight);

int tegra_dma_enqueue_req(int channel, struct tegra_dma_req *req)
{
	unsigned long irq_flags;
	struct tegra_dma_channel *ch = &dma_channels[channel];
	int start_dma = 0;

	if (req->size > NV_DMA_MAX_TRASFER_SIZE ||
		req->source_addr & 0x3 || req->dest_addr & 0x3) {
		printk(KERN_ERR "Invalid DMA request for channel %d\n", ch->id);
		return -EINVAL;
	}

	spin_lock_irqsave(&ch->lock, irq_flags);

	req->bytes_transferred = 0;
	req->status = 0;
	req->buffer_status = 0;
	if (list_empty(&ch->list))
		start_dma = 1;

	list_add_tail(&req->list, &ch->list);

	if (start_dma) {
		tegra_dma_update_hw(ch, req);
	}
	spin_unlock_irqrestore(&ch->lock, irq_flags);

	return 0;
}
EXPORT_SYMBOL(tegra_dma_enqueue_req);

int tegra_dma_allocate_channel(int mode)
{
	int channel;
	struct tegra_dma_channel *ch;

	/* first channel is the shared channel */
	if (mode & TEGRA_DMA_SHARED) {
		channel = TEGRA_SYSTEM_DMA_CH_MIN;
	}
	else {
		channel = find_first_zero_bit(channel_usage,
			ARRAY_SIZE(dma_channels));
		if (channel >= ARRAY_SIZE(dma_channels))
			return -ENODEV;
	}
	__set_bit(channel, channel_usage);
	ch = &dma_channels[channel];
	ch->mode = mode;
	return channel;
}
EXPORT_SYMBOL(tegra_dma_allocate_channel);

void tegra_dma_free_channel(int channel)
{
	struct tegra_dma_channel *ch;

	if (channel < TEGRA_SYSTEM_DMA_CH_MIN ||
	    channel >= TEGRA_SYSTEM_DMA_CH_MAX)
		return;

	ch = &dma_channels[channel];
	if (ch->mode & TEGRA_DMA_SHARED)
		return;

	__clear_bit(channel, channel_usage);
}
EXPORT_SYMBOL(tegra_dma_free_channel);

static int tegra_dma_set_name(int i, const char *fmt, ...)
{
	va_list vargs;
	int ret = 0;

	va_start(vargs, fmt);
	dma_channels[i].name = kvasprintf(GFP_KERNEL, fmt, vargs);
	if (!dma_channels[i].name)
		ret = -ENOMEM;
	va_end(vargs);

	return ret;
}

static void tegra_dma_update_hw_partial(struct tegra_dma_channel *ch,
	struct tegra_dma_req *req)
{
	if (req->to_memory) {
		ch->apb_ptr = req->source_addr;
		ch->ahb_ptr = req->dest_addr;
	} else {
		ch->apb_ptr = req->dest_addr;
		ch->ahb_ptr = req->source_addr;
	}
	writel(ch->apb_ptr, ch->addr + APBDMACHAN_CHANNEL_0_APB_PTR_0);
	writel(ch->ahb_ptr, ch->addr + APBDMACHAN_CHANNEL_0_AHB_PTR_0);
	return;
}

static void tegra_dma_update_hw(struct tegra_dma_channel *ch,
	struct tegra_dma_req *req)
{
	int ahb_addr_wrap, apb_addr_wrap;
	int index;
	unsigned long csr;

	switch (req->modid)
	{
	case NvRmModuleID_Uart:
		BUG_ON(req->instance >= 5);
		ch->csr = NV_FLD_SET_DRF_NUM(APBDMACHAN_CHANNEL_0, CSR,
			REQ_SEL, uart_selector_values[req->instance], ch->csr);
		ch->csr = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0,
			CSR, FLOW, ENABLE, ch->csr);
		ch->apb_seq = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0,
			APB_SEQ, APB_BUS_WIDTH, BUS_WIDTH_8, ch->apb_seq);
		ch->ahb_seq = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0,
			AHB_SEQ, AHB_BURST, DMA_BURST_1WORDS, ch->ahb_seq);
		break;
	case NvRmModuleID_I2c:
		BUG_ON(req->instance >= 3);
		ch->csr = NV_FLD_SET_DRF_NUM(APBDMACHAN_CHANNEL_0, CSR,
			REQ_SEL, i2c_selector_values[req->instance], ch->csr);
		ch->csr = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0,
			CSR, FLOW, ENABLE, ch->csr);
		ch->apb_seq = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0,
			APB_SEQ, APB_BUS_WIDTH, BUS_WIDTH_32, ch->apb_seq);
		ch->ahb_seq = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0,
			AHB_SEQ, AHB_BURST, DMA_BURST_1WORDS, ch->ahb_seq);
		break;
	case NvRmModuleID_Mipi:
	case NvRmModuleID_Spi:
	case NvRmModuleID_Slink:
	case NvRmModuleID_Spdif:
	case NvRmModuleID_I2s:
	default:
		/* FIXME add support for other modules */
		BUG();
		break;
	}

	/* One shot mode is always single buffered,
	 * continuous mode is always double buffered
	 * */
	if (ch->mode & TEGRA_DMA_MODE_ONESHOT) {
		ch->csr = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0,
			CSR, ONCE, SINGLE_BLOCK, ch->csr);
		ch->ahb_seq = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0,
			AHB_SEQ, DBL_BUF, RELOAD_FOR_1X_BLOCKS,
			ch->ahb_seq);
		ch->csr = NV_FLD_SET_DRF_NUM(APBDMACHAN_CHANNEL_0, CSR, WCOUNT,
			((req->size>>2)-1), ch->csr);
	} else {
		ch->csr = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0,
			CSR, ONCE, MULTIPLE_BLOCK, ch->csr);
		ch->ahb_seq = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0,
			AHB_SEQ, DBL_BUF, RELOAD_FOR_2X_BLOCKS, ch->ahb_seq);

		/* In double buffered mode, we set the size to half the
		 * requested size and interrupt when half the buffer
		 * is full */
		ch->csr = NV_FLD_SET_DRF_NUM(APBDMACHAN_CHANNEL_0, CSR, WCOUNT,
			((req->size>>3)-1), ch->csr);
	}

	if (req->to_memory) {
		ch->csr = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0,
			CSR, DIR, AHB_WRITE, ch->csr);
		ch->apb_ptr = req->source_addr;
		ch->ahb_ptr = req->dest_addr;

		apb_addr_wrap = req->source_wrap;
		ahb_addr_wrap = req->dest_wrap;

	} else {
		ch->csr = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0,
			CSR, DIR, AHB_READ, ch->csr);
		ch->apb_ptr = req->dest_addr;
		ch->ahb_ptr = req->source_addr;

		apb_addr_wrap = req->dest_wrap;
		ahb_addr_wrap = req->source_wrap;
	}

	apb_addr_wrap >>= 2;
	ahb_addr_wrap >>= 2;

	/* set address wrap for APB size */
	index = 0;
	do  {
		if (apb_addr_wrap_table[index] == apb_addr_wrap)
			break;
		index++;
	} while (index < ARRAY_SIZE(apb_addr_wrap_table));
	BUG_ON(index == ARRAY_SIZE(apb_addr_wrap_table));
	ch->apb_seq = NV_FLD_SET_DRF_NUM(APBDMACHAN_CHANNEL_0,
		APB_SEQ, APB_ADDR_WRAP, index, ch->apb_seq);

	/* set address wrap for AHB size */
	index = 0;
	do  {
		if (ahb_addr_wrap_table[index] == ahb_addr_wrap)
			break;
		index++;
	} while (index < ARRAY_SIZE(ahb_addr_wrap_table));
	BUG_ON(index == ARRAY_SIZE(ahb_addr_wrap_table));
	ch->ahb_seq = NV_FLD_SET_DRF_NUM(APBDMACHAN_CHANNEL_0,
		AHB_SEQ, WRAP, index, ch->ahb_seq);

	ch->csr = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, CSR, IE_EOC,
		ENABLE, ch->csr);

	/* update hw registers with the shadow */
	writel(ch->csr, ch->addr + APBDMACHAN_CHANNEL_0_CSR_0);
	writel(ch->apb_seq, ch->addr + APBDMACHAN_CHANNEL_0_APB_SEQ_0);
	writel(ch->apb_ptr, ch->addr + APBDMACHAN_CHANNEL_0_APB_PTR_0);
	writel(ch->ahb_seq, ch->addr + APBDMACHAN_CHANNEL_0_AHB_SEQ_0);
	writel(ch->ahb_ptr, ch->addr + APBDMACHAN_CHANNEL_0_AHB_PTR_0);

	csr = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, CSR, ENB,
		ENABLE, ch->csr);
	writel(csr, ch->addr + APBDMACHAN_CHANNEL_0_CSR_0);
}

static void tegra_dma_init_hw(struct tegra_dma_channel *ch)
{
	ch->csr = NV_RESETVAL(APBDMACHAN_CHANNEL_0, CSR);
	ch->ahb_seq = NV_RESETVAL(APBDMACHAN_CHANNEL_0, AHB_SEQ);
	ch->apb_seq = NV_RESETVAL(APBDMACHAN_CHANNEL_0,APB_SEQ);

	/* One shot with an interrupt to CPU after transfer */
	ch->csr = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, CSR,
		ONCE, SINGLE_BLOCK, ch->csr);
	ch->csr = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0, CSR,
		IE_EOC, ENABLE, ch->csr);
	ch->ahb_seq = NV_FLD_SET_DRF_DEF(APBDMACHAN_CHANNEL_0,
		AHB_SEQ, INTR_ENB, CPU, ch->ahb_seq);
}

static void handle_oneshot_dma(struct tegra_dma_channel *ch)
{
	static struct tegra_dma_req *req;

	spin_lock(&ch->lock);
	if (list_empty(&ch->list)) {
		spin_unlock(&ch->lock);
		return;
	}

	req = list_entry(ch->list.next, typeof(*req), list);
	if (req) {
		int bytes_transferred;

		bytes_transferred = NV_DRF_VAL(APBDMACHAN_CHANNEL_0, CSR,
			WCOUNT, ch->csr);

		bytes_transferred += 1;
		bytes_transferred <<= 2;

		list_del(&req->list);
		req->bytes_transferred = bytes_transferred;
		req->status = TEGRA_DMA_REQ_SUCCESS;

		spin_unlock(&ch->lock);
		/* Callback should be called without any lock */
		req->complete(req, TEGRA_DMA_REQ_SUCCESS);
		spin_lock(&ch->lock);
	}

	if (!list_empty(&ch->list)) {
		req = list_entry(ch->list.next, typeof(*req), list);
		tegra_dma_update_hw(ch, req);
	}
	spin_unlock(&ch->lock);
}

static void handle_continuous_dma(struct tegra_dma_channel *ch)
{
	static struct tegra_dma_req *req;

	spin_lock(&ch->lock);
	if (list_empty(&ch->list)) {
		spin_unlock(&ch->lock);
		return;
	}

	req = list_entry(ch->list.next, typeof(*req), list);
	if (req) {
		if (req->buffer_status == TEGRA_DMA_REQ_BUF_STATUS_EMPTY) {
			/* Load the next request into the hardware, if available
			 * */
			if (!list_is_last(&req->list, &ch->list)) {
				struct tegra_dma_req *next_req;

				printk("Queue the next request\n");
				next_req = list_entry(req->list.next,
					typeof(*next_req), list);
				tegra_dma_update_hw_partial(ch, next_req);
			}
			req->buffer_status = TEGRA_DMA_REQ_BUF_STATUS_HALF_FULL;
			/* DMA lock is NOT held when callbak is called */
			spin_unlock(&ch->lock);
			req->threshold(req, TEGRA_DMA_REQ_SUCCESS);
			return;

		} else if (req->buffer_status ==
			TEGRA_DMA_REQ_BUF_STATUS_HALF_FULL) {
			/* Callback when the buffer is completely full (i.e on
			 * the second  interrupt */
			int bytes_transferred;

			bytes_transferred = NV_DRF_VAL(APBDMACHAN_CHANNEL_0,
				CSR, WCOUNT, ch->csr);
			bytes_transferred += 1;
			bytes_transferred <<= 3;

			req->buffer_status = TEGRA_DMA_REQ_BUF_STATUS_FULL;
			req->bytes_transferred = bytes_transferred;
			req->status = TEGRA_DMA_REQ_SUCCESS;
			list_del(&req->list);

			/* DMA lock is NOT held when callbak is called */
			spin_unlock(&ch->lock);
			req->complete(req, TEGRA_DMA_REQ_SUCCESS);
			return;

		} else {
			BUG();
		}
	}
	spin_unlock(&ch->lock);
}

static irqreturn_t dma_isr(int irq, void *id)
{
	int channel = (int)id;
	struct tegra_dma_channel *ch = &dma_channels[channel];
	unsigned long status;

	status = readl(ch->addr + APBDMACHAN_CHANNEL_0_STA_0);
	if (status & NV_DRF_DEF(APBDMACHAN_CHANNEL_0, STA, ISE_EOC, INTR))
		writel(status, ch->addr + APBDMACHAN_CHANNEL_0_STA_0);
	else {
		printk("Got a supurious ISR for DMA channel %d\n", channel);
		return IRQ_HANDLED;
	}

	if (ch->mode & TEGRA_DMA_MODE_ONESHOT)
		handle_oneshot_dma(ch);
	else
		handle_continuous_dma(ch);


	return IRQ_HANDLED;
}

int __init tegra_dma_init(void)
{
	int ret = 0;
	unsigned int size;
	unsigned long modid;
	int i;
	unsigned int irq;
	int max_channels;

	max_channels = NvRmModuleGetNumInstances(s_hRmGlobal,
		NvRmPrivModuleID_ApbDmaChannel);
	BUG_ON(max_channels > NV_DMA_MAX_CHANNELS);

	/* Reserve all the channels we are not supposed to touch */
	for (i=0; i<TEGRA_SYSTEM_DMA_CH_MIN; i++)
		__set_bit(i, channel_usage);

	for (i=TEGRA_SYSTEM_DMA_CH_MAX; i<ARRAY_SIZE(dma_channels); i++)
		__set_bit(i, channel_usage);

	for (i=TEGRA_SYSTEM_DMA_CH_MIN; i<TEGRA_SYSTEM_DMA_CH_MAX; i++) {
		struct tegra_dma_channel *ch = &dma_channels[i];

		ch->id = i;
		ret = tegra_dma_set_name(i, "dma_channel_%d\n", i);
		if (ret)
			goto fail;

		modid = NVRM_MODULE_ID(NvRmPrivModuleID_ApbDmaChannel, i);
		NvRmModuleGetBaseAddress(s_hRmGlobal, modid,
			&ch->phys_addr, &size);
		ch->addr = IO_ADDRESS(ch->phys_addr);
		if (!ch->addr) {
			ret = -ENOMEM;
			goto fail;
		}
		spin_lock_init(&ch->lock);
		INIT_LIST_HEAD(&ch->list);
		tegra_dma_init_hw(ch);
	}

	for (i=TEGRA_SYSTEM_DMA_CH_MIN; i<TEGRA_SYSTEM_DMA_CH_MAX; i++) {
		irq = NvRmGetIrqForLogicalInterrupt(s_hRmGlobal,
			NvRmPrivModuleID_ApbDma, i);
		printk("Irq value = %d\n", irq);
		BUG_ON(irq >= NR_IRQS);
		ret = request_irq(irq, dma_isr, 0, dma_channels[i].name,
			(void *)i);
		if (ret) {
			printk("Failed to register IRQ for DMA %d\n", i);
			goto fail;
		}
		set_irq_flags(irq, IRQF_VALID);
	}
	return ret;
fail:
	/* FIXME cleanup */
	return ret;
}
