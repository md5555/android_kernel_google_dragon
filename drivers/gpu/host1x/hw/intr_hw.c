/*
 * Tegra host1x Interrupt Management
 *
 * Copyright (C) 2010 Google, Inc.
 * Copyright (c) 2010-2013, NVIDIA Corporation.
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

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>

#include "../intr.h"
#include "../dev.h"

/*
 * Sync point threshold interrupt service function
 * Handles sync point threshold triggers, in interrupt context
 */
static void host1x_intr_syncpt_handle(struct host1x_syncpt *syncpt)
{
	unsigned int id = syncpt->id;
	struct host1x *host = syncpt->host;

	host1x_sync_writel(host, BIT_MASK(id),
		HOST1X_SYNC_SYNCPT_THRESH_INT_DISABLE(BIT_WORD(id)));
	host1x_sync_writel(host, BIT_MASK(id),
		HOST1X_SYNC_SYNCPT_THRESH_CPU0_INT_STATUS(BIT_WORD(id)));

	queue_work(host->intr_wq, &syncpt->intr.work);
}

static irqreturn_t syncpt_thresh_isr(int irq, void *dev_id)
{
	struct host1x *host = dev_id;
	unsigned long reg;
	int i, id;

	for (i = 0; i < DIV_ROUND_UP(host->info->nb_pts, 32); i++) {
		reg = host1x_sync_readl(host,
			HOST1X_SYNC_SYNCPT_THRESH_CPU0_INT_STATUS(i));
		for_each_set_bit(id, &reg, BITS_PER_LONG) {
			struct host1x_syncpt *syncpt =
				host->syncpt + (i * BITS_PER_LONG + id);
			host1x_intr_syncpt_handle(syncpt);
		}
	}

	return IRQ_HANDLED;
}

static void _host1x_intr_disable_all_syncpt_intrs(struct host1x *host)
{
	u32 i;

	for (i = 0; i < DIV_ROUND_UP(host->info->nb_pts, 32); ++i) {
		host1x_sync_writel(host, 0xffffffffu,
			HOST1X_SYNC_SYNCPT_THRESH_INT_DISABLE(i));
		host1x_sync_writel(host, 0xffffffffu,
			HOST1X_SYNC_SYNCPT_THRESH_CPU0_INT_STATUS(i));
	}
}

static int _host1x_intr_init_host_sync(struct host1x *host, u32 cpm,
	void (*syncpt_thresh_work)(struct work_struct *))
{
	int i, err;

	host1x_hw_intr_disable_all_syncpt_intrs(host);

	for (i = 0; i < host->info->nb_pts; i++)
		INIT_WORK(&host->syncpt[i].intr.work, syncpt_thresh_work);

	err = devm_request_irq(host->dev, host->intr_syncpt_irq,
			       syncpt_thresh_isr, IRQF_SHARED,
			       "host1x_syncpt", host);
	if (IS_ERR_VALUE(err)) {
		WARN_ON(1);
		return err;
	}

	/* disable the ip_busy_timeout. this prevents write drops */
	host1x_sync_writel(host, 0, HOST1X_SYNC_IP_BUSY_TIMEOUT);

	/*
	 * increase the auto-ack timout to the maximum value. 2d will hang
	 * otherwise on Tegra2.
	 */
	host1x_sync_writel(host, 0xff, HOST1X_SYNC_CTXSW_TIMEOUT_CFG);

	/* update host clocks per usec */
	host1x_sync_writel(host, cpm, HOST1X_SYNC_USEC_CLK);

	return 0;
}

static void _host1x_intr_set_syncpt_threshold(struct host1x *host,
	u32 id, u32 thresh)
{
	host1x_sync_writel(host, thresh, HOST1X_SYNC_SYNCPT_INT_THRESH(id));
}

static void _host1x_intr_enable_syncpt_intr(struct host1x *host, u32 id)
{
	host1x_sync_writel(host, BIT_MASK(id),
		HOST1X_SYNC_SYNCPT_THRESH_INT_ENABLE_CPU0(BIT_WORD(id)));
}

static void _host1x_intr_disable_syncpt_intr(struct host1x *host, u32 id)
{
	host1x_sync_writel(host, BIT_MASK(id),
		HOST1X_SYNC_SYNCPT_THRESH_INT_DISABLE(BIT_WORD(id)));
	host1x_sync_writel(host, BIT_MASK(id),
		HOST1X_SYNC_SYNCPT_THRESH_CPU0_INT_STATUS(BIT_WORD(id)));
}

static int _host1x_free_syncpt_irq(struct host1x *host)
{
	devm_free_irq(host->dev, host->intr_syncpt_irq, host);
	flush_workqueue(host->intr_wq);
	return 0;
}

/**
 * Host general interrupt service function
 * Handles read / write failures
 */
static irqreturn_t general_isr(int irq, void *dev_id)
{
	struct host1x *host = dev_id;
	unsigned long stat, intstat;
	u32 ext_stat, addr;
	int i;

	intstat = host1x_sync_readl(host, HOST1X_SYNC_INTSTATUS);
	host->intstatus = intstat;

	/* Handle host1x interrupt in ISR */
	stat = host1x_sync_readl(host, HOST1X_SYNC_HINTSTATUS);
	ext_stat = host1x_sync_readl(host, HOST1X_SYNC_HINTSTATUS_EXT);

	for_each_set_bit(i, &stat, HOST1X_NUM_HOST_ISR) {
		if (host->host_isr[i])
			host->host_isr[i](stat, host->host_isr_priv[i]);
	}

	if (HOST1X_SYNC_HINTSTATUS_EXT_IP_READ_INT_V(ext_stat)) {
		addr = host1x_sync_readl(host,
					 HOST1X_SYNC_IP_READ_TIMEOUT_ADDR);
		pr_err("Host read timeout at address %x\n", addr);
	}

	if (HOST1X_SYNC_HINTSTATUS_EXT_IP_WRITE_INT_V(ext_stat)) {
		addr = host1x_sync_readl(host,
					 HOST1X_SYNC_IP_WRITE_TIMEOUT_ADDR);
		pr_err("Host write timeout at address %x\n", addr);
	}

	host1x_sync_writel(host, ext_stat, HOST1X_SYNC_HINTSTATUS_EXT);
	host1x_sync_writel(host, stat, HOST1X_SYNC_HINTSTATUS);

	host1x_sync_writel(host, intstat, HOST1X_SYNC_INTSTATUS);
	return IRQ_HANDLED;
}

static int _host1x_intr_request_host_general_irq(struct host1x *host)
{
	int err;
	u32 val;

	/* master disable for general (not syncpt) host interrupts */
	host1x_sync_writel(host, 0, HOST1X_SYNC_INTMASK);

	/* clear status & extstatus */
	host1x_sync_writel(host, 0xfffffffful, HOST1X_SYNC_HINTSTATUS_EXT);
	host1x_sync_writel(host, 0xfffffffful, HOST1X_SYNC_HINTSTATUS);

	err = devm_request_irq(host->dev, host->intr_general_irq,
			       general_isr, 0,
			       "host1x_general", host);
	if (IS_ERR_VALUE(err)) {
		WARN_ON(1);
		return err;
	}

	/* enable extra interrupt sources IP_READ_INT and IP_WRITE_INT */
	host1x_sync_writel(host, BIT(30) | BIT(31), HOST1X_SYNC_HINTMASK_EXT);

	/* enable extra interrupt sources */
	val = host1x_sync_readl(host, HOST1X_SYNC_HINTMASK);
	val |= BIT(31);
	host1x_sync_writel(host, val, HOST1X_SYNC_HINTMASK);

	/* enable host module interrupt to CPU0 */
	host1x_sync_writel(host, BIT(0), HOST1X_SYNC_INTC0MASK);

	/* master enable for general (not syncpt) host interrupts */
	host1x_sync_writel(host, BIT(0), HOST1X_SYNC_INTMASK);

	return err;
}

static void _host1x_intr_enable_host_irq(struct host1x *host, int irq)
{
	long val;

	val = host1x_sync_readl(host, HOST1X_SYNC_HINTMASK);
	val |= BIT(irq);
	host1x_sync_writel(host, val, HOST1X_SYNC_HINTMASK);
}

static void _host1x_intr_disable_host_irq(struct host1x *host, int irq)
{
	long val;

	val = host1x_sync_readl(host, HOST1X_SYNC_HINTMASK);
	val &= ~BIT(irq);
	host1x_sync_writel(host, val, HOST1X_SYNC_HINTMASK);
}

static void _host1x_intr_free_host_general_irq(struct host1x *host)
{
	/* master disable for general (not syncpt) host interrupts */
	host1x_sync_writel(host, 0, HOST1X_SYNC_INTMASK);

	devm_free_irq(host->dev, host->intr_general_irq, host);
}

static const struct host1x_intr_ops host1x_intr_ops = {
	/* Syncpt interrupts */
	.init_host_sync = _host1x_intr_init_host_sync,
	.set_syncpt_threshold = _host1x_intr_set_syncpt_threshold,
	.enable_syncpt_intr = _host1x_intr_enable_syncpt_intr,
	.disable_syncpt_intr = _host1x_intr_disable_syncpt_intr,
	.disable_all_syncpt_intrs = _host1x_intr_disable_all_syncpt_intrs,
	.free_syncpt_irq = _host1x_free_syncpt_irq,

	/* Host general interrupts */
	.request_host_general_irq = _host1x_intr_request_host_general_irq,
	.enable_host_irq = _host1x_intr_enable_host_irq,
	.disable_host_irq = _host1x_intr_disable_host_irq,
	.free_host_general_irq = _host1x_intr_free_host_general_irq,
};
