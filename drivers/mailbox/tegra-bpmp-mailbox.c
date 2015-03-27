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
 */

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mailbox_controller.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <soc/tegra/bpmp.h>
#include <soc/tegra/irq.h>

#include "mailbox.h"

/* Semaphore Registers */
#define RES_SEMA_SHRD_SMP_STATUS	0x0
#define RES_SEMA_SHRD_SMP_SET		0x4
#define RES_SEMA_SHRD_SMP_CLEAR		0x8

/* Atomics Registers */
#define ATOMICS_AP0_TRIGGER		0x0
#define ATOMICS_AP0_RESULT(id)		(0xc00 + (id) * 4)

#define TRIGGER_ID_SHIFT	16
#define TRIGGER_CMD_GET		4

/*
 * How the token bits are interpreted
 *
 * SL_SIGL (b00): slave ch in signaled state
 * SL_QUED (b01): slave ch is in queue
 * MA_FREE (b10): master ch is free
 * MA_ACKD (b11): master ch is acked
 */
#define SL_SIGL(ch)	(0x0 << ((ch) * 2))
#define SL_QUED(ch)	(0x1 << ((ch) * 2))
#define MA_FREE(ch)	(0x2 << ((ch) * 2))
#define MA_ACKD(ch)	(0x3 << ((ch) * 2))

#define CH_MASK(ch)	(0x3 << ((ch) * 2))

/*
 * Ideally, the slave should only set bits while the
 * master do only clear them. But there is an exception -
 * see bpmp_ack_master()
 *
 * While moving from SL_QUED to MA_FREE (DO_ACK not set) so that
 * the channel won't be in ACKD state forever.
 */
#define FREE_CH(ch)	(0x1 << ((ch) * 2))

#define CHANNEL_TIMEOUT		USEC_PER_SEC

struct channel_data {
	struct mb_data *data;
};

struct tegra_bpmp_mbox {
	struct mbox_controller mbox;
	void __iomem *base;
	void __iomem *atomics_base;
	int rx_irqs[BPMP_NR_INBOX_CHAN];
	int tx_irq;
	spinlock_t lock;
	struct channel_data channel_area[BPMP_NR_CHANNEL];
	struct notifier_block cpu_nb;
};

static u32 tegra_bpmp_token(struct tegra_bpmp_mbox *mbox)
{
	return readl(mbox->base + RES_SEMA_SHRD_SMP_STATUS);
}

static void tegra_bpmp_token_set(struct tegra_bpmp_mbox *mbox, u32 val)
{
	writel(val, mbox->base + RES_SEMA_SHRD_SMP_SET);
}

static void tegra_bpmp_token_clear(struct tegra_bpmp_mbox *mbox, u32 val)
{
	writel(val, mbox->base + RES_SEMA_SHRD_SMP_CLEAR);
}

static u32 tegra_bpmp_chan_status(struct mbox_chan *chan)
{
	struct tegra_bpmp_mbox *mbox = dev_get_drvdata(chan->mbox->dev);
	int ch = chan - mbox->mbox.chans;

	return tegra_bpmp_token(mbox) & CH_MASK(ch);
}

static bool tegra_bpmp_master_is_acked(struct mbox_chan *chan)
{
	struct tegra_bpmp_mbox *mbox = dev_get_drvdata(chan->mbox->dev);
	int ch = chan - mbox->mbox.chans;

	return tegra_bpmp_chan_status(chan) == MA_ACKD(ch);
}

static bool tegra_bpmp_master_is_free(struct mbox_chan *chan)
{
	struct tegra_bpmp_mbox *mbox = dev_get_drvdata(chan->mbox->dev);
	int ch = chan - mbox->mbox.chans;

	return tegra_bpmp_chan_status(chan) == MA_FREE(ch);
}

static bool tegra_bpmp_slave_signalled(struct mbox_chan *chan)
{
	struct tegra_bpmp_mbox *mbox = dev_get_drvdata(chan->mbox->dev);
	int ch = chan - mbox->mbox.chans;

	return tegra_bpmp_chan_status(chan) == SL_SIGL(ch);
}

static void tegra_bpmp_free_master(struct mbox_chan *chan)
{
	struct tegra_bpmp_mbox *mbox = dev_get_drvdata(chan->mbox->dev);
	int ch = chan - mbox->mbox.chans;

	tegra_bpmp_token_clear(mbox, FREE_CH(ch));
}

static void tegra_bpmp_ack_master(struct mbox_chan *chan, int flags)
{
	struct tegra_bpmp_mbox *mbox = dev_get_drvdata(chan->mbox->dev);
	int ch = chan - mbox->mbox.chans;

	tegra_bpmp_token_set(mbox, MA_ACKD(ch));

	if (!(flags & BPMP_MSG_DO_ACK))
		tegra_bpmp_free_master(chan);
}

static void tegra_bpmp_signal_slave(struct mbox_chan *chan)
{
	struct tegra_bpmp_mbox *mbox = dev_get_drvdata(chan->mbox->dev);
	int ch = chan - mbox->mbox.chans;

	tegra_bpmp_token_clear(mbox, CH_MASK(ch));
}

static int tegra_bpmp_wait_ack(struct mbox_chan *chan)
{
	ktime_t start = ktime_get();

	do {
		if (tegra_bpmp_master_is_acked(chan))
			return 0;
		udelay(100);
	} while (ktime_us_delta(ktime_get(), start) < CHANNEL_TIMEOUT);

	return -ETIMEDOUT;
}

static int tegra_bpmp_wait_free(struct mbox_chan *chan)
{
	ktime_t start = ktime_get();

	do {
		if (tegra_bpmp_master_is_free(chan))
			return 0;
		udelay(100);
	} while (ktime_us_delta(ktime_get(), start) < CHANNEL_TIMEOUT);

	return -ETIMEDOUT;
}

static void tegra_bpmp_ring_doorbell(int irq)
{
	tegra_legacy_irq_trigger(irq);
}

static struct mb_data *tegra_bpmp_chan_data(struct mbox_chan *chan)
{
	struct tegra_bpmp_mbox *mbox = dev_get_drvdata(chan->mbox->dev);
	int ch = chan - mbox->mbox.chans;

	return mbox->channel_area[ch].data;
}

static int tegra_bpmp_mbox_send_data(struct mbox_chan *chan, void *data)
{
	struct tegra_bpmp_mbox *mbox = dev_get_drvdata(chan->mbox->dev);
	struct tegra_bpmp_mbox_msg *msg = data;
	int ret;

	if (msg->type & BPMP_MSG_TYPE_REQ) {
		struct mb_data *d = tegra_bpmp_chan_data(chan);

		ret = tegra_bpmp_wait_free(chan);
		if (ret < 0)
			return ret;

		d->code = msg->code;
		d->flags = msg->flags;
		if (msg->data)
			memcpy(d->data, msg->data, msg->size);

		tegra_bpmp_signal_slave(chan);
		tegra_bpmp_ring_doorbell(mbox->tx_irq);

		if (msg->type & BPMP_MSG_TYPE_ATOMIC) {
			ret = tegra_bpmp_wait_ack(chan);
			if (ret >= 0) {
				mbox_chan_received_data(chan, (void *)d);
				tegra_bpmp_free_master(chan);
			}
		}

		return ret;
	} else {
		struct mb_data *d = tegra_bpmp_chan_data(chan);

		d->code = msg->code;
		memcpy(d->data, msg->data, msg->size);
		tegra_bpmp_ack_master(chan, d->flags);

		if (d->flags & BPMP_MSG_RING_DOORBELL)
			tegra_bpmp_ring_doorbell(mbox->tx_irq);

		return 0;
	}
}

static int tegra_bpmp_channel_area_attach(struct mbox_chan *chan)
{
	struct tegra_bpmp_mbox *mbox = dev_get_drvdata(chan->mbox->dev);
	int ch = chan - mbox->mbox.chans;
	u32 addr;
	void *p;

	writel(ch << TRIGGER_ID_SHIFT | TRIGGER_CMD_GET,
	       mbox->atomics_base + ATOMICS_AP0_TRIGGER);
	addr = readl(mbox->atomics_base + ATOMICS_AP0_RESULT(ch));
	if (!addr)
		return -EFAULT;

	p = ioremap(addr, sizeof(struct mb_data));
	if (!p)
		return -EFAULT;

	mbox->channel_area[ch].data = p;
	return 0;
}

static void tegra_bpmp_channel_area_detach(struct mbox_chan *chan)
{
	struct tegra_bpmp_mbox *mbox = dev_get_drvdata(chan->mbox->dev);
	int ch = chan - mbox->mbox.chans;

	iounmap(mbox->channel_area[ch].data);
	mbox->channel_area[ch].data = NULL;
}

static int tegra_bpmp_mbox_startup(struct mbox_chan *chan)
{
	int ret = -ENODEV;

	/* BPMP firmware shall set token to MA_FREE for handshake */
	if (tegra_bpmp_master_is_free(chan))
		ret = tegra_bpmp_channel_area_attach(chan);

	return ret;
}

static void tegra_bpmp_mbox_shutdown(struct mbox_chan *chan)
{
	/* set token to SL_SIGL for handshake */
	/*
	 * FIXME: when does BPMP firmware set the token back?
	 * when BPMP firmware resumes?
	 */
	tegra_bpmp_signal_slave(chan);
	tegra_bpmp_channel_area_detach(chan);
}

static bool tegra_bpmp_mbox_last_tx_done(struct mbox_chan *chan)
{
	return true;
}

static const struct mbox_chan_ops tegra_bpmp_mbox_chan_ops = {
	.send_data = tegra_bpmp_mbox_send_data,
	.startup = tegra_bpmp_mbox_startup,
	.shutdown = tegra_bpmp_mbox_shutdown,
	.last_tx_done = tegra_bpmp_mbox_last_tx_done,
};

static void tegra_bpmp_irq_set_affinity(struct tegra_bpmp_mbox *mbox, int cpu)
{
	int r;

	r = irq_set_affinity(mbox->rx_irqs[cpu], cpumask_of(cpu));
	WARN_ON(r);
}

static void tegra_bpmp_irq_clear_affinity(struct tegra_bpmp_mbox *mbox, int cpu)
{
	int new, r;

	new = cpumask_any_but(cpu_online_mask, cpu);
	r = irq_set_affinity(mbox->rx_irqs[cpu], cpumask_of(new));
	WARN_ON(r);
}

static int tegra_bpmp_cpu_notify(struct notifier_block *nb,
				 unsigned long action, void *hcpu)
{
	struct tegra_bpmp_mbox *mbox =
		container_of(nb, struct tegra_bpmp_mbox, cpu_nb);
	int cpu = (long)hcpu;

	switch (action) {
	case CPU_DOWN_PREPARE:
	case CPU_DOWN_PREPARE_FROZEN:
		tegra_bpmp_irq_clear_affinity(mbox, cpu);
		break;
	case CPU_ONLINE:
	case CPU_ONLINE_FROZEN:
		tegra_bpmp_irq_set_affinity(mbox, cpu);
		break;
	}

	return NOTIFY_OK;
}

static irqreturn_t tegra_bpmp_mbox_irq(int irq, void *p)
{
	struct mbox_chan *chan = (struct mbox_chan *)p;
	struct tegra_bpmp_mbox *mbox = dev_get_drvdata(chan->mbox->dev);
	int i;

	/* for RX channels */
	if (tegra_bpmp_slave_signalled(chan)) {
		struct mb_data *d = tegra_bpmp_chan_data(chan);
		mbox_chan_received_data(chan, (void *)d);
	}

	spin_lock(&mbox->lock);

	for (i = 0; i < BPMP_NR_THREAD_CHAN; i++) {
		struct mbox_chan *mchan =
				&mbox->mbox.chans[i + THREAD_CHAN_OFFSET];

		if (tegra_bpmp_master_is_acked(mchan)) {
			struct mb_data *d = tegra_bpmp_chan_data(mchan);
			mbox_chan_received_data(mchan, (void *)d);
			tegra_bpmp_free_master(mchan);

			/*
			 * FIXME:
			 * thread channel always has
			 * TEGRA_BPMP_RING_DOORBELL set
			 * */
			if (d->flags & BPMP_MSG_RING_DOORBELL)
				mbox_chan_txdone(mchan, 0);
		}
	}

	spin_unlock(&mbox->lock);

	return IRQ_HANDLED;
}

static const struct of_device_id tegra_bpmp_mbox_of_match[] = {
	{ .compatible = "nvidia,tegra210-bpmp-mbox" },
	{ },
};
MODULE_DEVICE_TABLE(of, tegra_bpmp_mbox_of_match);

static int tegra_bpmp_mbox_probe(struct platform_device *pdev)
{
	struct tegra_bpmp_mbox *mbox;
	struct resource *res;
	int i, irq, ret;

	mbox = devm_kzalloc(&pdev->dev, sizeof(*mbox), GFP_KERNEL);
	if (!mbox)
		return -ENOMEM;

	platform_set_drvdata(pdev, mbox);
	spin_lock_init(&mbox->lock);

	mbox->mbox.dev = &pdev->dev;
	mbox->mbox.chans = devm_kcalloc(&pdev->dev, BPMP_NR_CHANNEL,
					sizeof(*mbox->mbox.chans), GFP_KERNEL);
	if (!mbox->mbox.chans)
		return -ENOMEM;

	mbox->mbox.num_chans = BPMP_NR_CHANNEL;
	mbox->mbox.ops = &tegra_bpmp_mbox_chan_ops;

	/* make use of TXDONE_BY_ACK */
	mbox->mbox.txdone_irq = false;
	mbox->mbox.txdone_poll = false;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	mbox->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(mbox->base))
		return PTR_ERR(mbox->base);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	mbox->atomics_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(mbox->atomics_base))
		return PTR_ERR(mbox->atomics_base);

	/* setup inbox channel interrupts */
	for (i = 0; i < BPMP_NR_INBOX_CHAN; i++) {
		struct mbox_chan *chan =
				&mbox->mbox.chans[i + INBOX_CHAN_OFFSET];

		irq = platform_get_irq(pdev, i);
		if (irq < 0)
			return irq;

		ret = devm_request_irq(&pdev->dev, irq, tegra_bpmp_mbox_irq, 0,
				       dev_name(&pdev->dev), chan);
		if (ret < 0)
			return ret;

		mbox->rx_irqs[i] = irq;
	}

	/* setup tx irq */
	irq = platform_get_irq(pdev, i);
	if (irq < 0) {
		dev_err(&pdev->dev, "missing tx irq\n");
		return irq;
	}

	mbox->tx_irq = irq;

	mbox->cpu_nb.notifier_call = tegra_bpmp_cpu_notify;
	ret = register_cpu_notifier(&mbox->cpu_nb);
	if (ret)
		return ret;

	for_each_present_cpu(i)
		tegra_bpmp_irq_set_affinity(mbox, i);

	/* for thread channels TX ACK is done via IRQ */
	for (i = 0; i < BPMP_NR_THREAD_CHAN; ++i) {
		struct mbox_chan *chan =
				&mbox->mbox.chans[i + THREAD_CHAN_OFFSET];
		chan->txdone_method = TXDONE_BY_IRQ;
	}

	ret = mbox_controller_register(&mbox->mbox);
	if (ret < 0)
		dev_err(&pdev->dev, "failed to register mailbox: %d\n", ret);

	return ret;
}

static int tegra_bpmp_mbox_remove(struct platform_device *pdev)
{
	struct tegra_bpmp_mbox *mbox = platform_get_drvdata(pdev);
	int i;

	unregister_cpu_notifier(&mbox->cpu_nb);
	for (i = 0; i < BPMP_NR_INBOX_CHAN; i++) {
		struct mbox_chan *chan =
			&mbox->mbox.chans[i + INBOX_CHAN_OFFSET];

		synchronize_irq(mbox->rx_irqs[i]);
		devm_free_irq(&pdev->dev, mbox->rx_irqs[i], chan);
	}
	mbox_controller_unregister(&mbox->mbox);

	return 0;
}

static struct platform_driver tegra_bpmp_mbox_driver = {
	.probe	= tegra_bpmp_mbox_probe,
	.remove	= tegra_bpmp_mbox_remove,
	.driver	= {
		.name = "tegra-bpmp-mbox",
		.of_match_table = tegra_bpmp_mbox_of_match,
	},
};
module_platform_driver(tegra_bpmp_mbox_driver);

MODULE_DESCRIPTION("NVIDIA Tegra BPMP-lite mailbox driver");
MODULE_LICENSE("GPL v2");
