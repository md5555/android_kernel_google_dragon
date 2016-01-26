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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/firmware.h>
#include <linux/reset.h>
#include <linux/mailbox_client.h>

#include <soc/tegra/bpmp.h>
#include <soc/tegra/flowctrl.h>

#define FWTAG_SIZE		32
#define SHARED_MEMORY_SIZE	512
#define THREAD_CHAN_TIMEOUT	USEC_PER_SEC

#define __MRQ_ATTRS		0xff000000
#define __MRQ_INDEX(id)		((id) & ~__MRQ_ATTRS)

#define BPMP_CONNECT_RETRY_MAX	50
#define BPMP_FIRMWARE_NAME	"nvidia/tegra210/bpmp.bin"

typedef void (*bpmp_mrq_handler)(int mrq, void *data, int ch);

struct fw_header {
	u32 magic;
	u32 version;
	u32 chip_id;
	u32 mem_size;
	u32 reset_offset;
	u8 reserved2[124];
	u8 md5sum[32];
	u8 common_head[40];
	u8 platform_head[40];
} __packed;

struct tegra_bpmp {
	void __iomem *reset_vec;
	struct reset_control *cop_rst;
	struct device *dev;
	u32 bootaddr;
	bool initialized;
	phys_addr_t loadfw_phys;
	void *loadfw_virt;
	spinlock_t shared_memory_lock;
};

static struct tegra_bpmp *bpmp;

struct mrq_handler_data {
	bpmp_mrq_handler handler;
	void *data;
};

struct module_mrq {
	struct list_head link;
	u32 base;
	bpmp_mrq_handler handler;
	void *data;
};

static LIST_HEAD(module_mrq_list);
static struct mrq_handler_data mrq_handlers[NR_MRQS];
static DEFINE_SPINLOCK(mrq_lock);

struct tegra_bpmp_mbox_client {
	struct mbox_client client;
	struct mbox_chan *mbox_chan;
	struct mb_data *rx_req;
	struct mb_data *tx_resp;
	bool in_use; /* only used for thread channels */
};

struct tegra_bpmp_mbox_client mbox_clients[BPMP_NR_CHANNEL];
static bool connected;
static DEFINE_SPINLOCK(lock);
static void *shared_virt;
static u32 shared_phys;
static bool bpmp_suspended;

static void bpmp_handle_mail(int mrq, int ch);

static struct tegra_bpmp_mbox_client *acquire_outbox_client(void)
{
	return &mbox_clients[smp_processor_id()];
}

static struct tegra_bpmp_mbox_client *acquire_thread_client(void)
{
	struct tegra_bpmp_mbox_client *found = NULL;
	struct tegra_bpmp_mbox_client *cl;
	unsigned long flags;
	int i;

	spin_lock_irqsave(&lock, flags);

	for (i = 0; i < BPMP_NR_THREAD_CHAN; ++i) {
		cl = &mbox_clients[i + THREAD_CHAN_OFFSET];

		if (!cl->in_use) {
			cl->in_use = true;
			found = cl;
			break;
		}
	}

	spin_unlock_irqrestore(&lock, flags);
	return found;
}

static void free_thread_client(struct tegra_bpmp_mbox_client *cl)
{
	unsigned long flags;

	spin_lock_irqsave(&lock, flags);
	cl->in_use = false;
	spin_unlock_irqrestore(&lock, flags);
}

static int tegra_bpmp_read_response(struct tegra_bpmp_mbox_client *cl,
				    void *data, int sz)
{
	struct mb_data *resp = cl->tx_resp;

	if (data)
		memcpy(data, resp->data, sz);

	return resp->code;
}

int tegra_bpmp_send(int mrq, void *data, int sz)
{
	struct tegra_bpmp_mbox_client *cl;
	struct tegra_bpmp_mbox_msg msg;
	int ret;

	if (!connected)
		return -ENODEV;

	preempt_disable();
	cl = acquire_outbox_client();

	msg.type = BPMP_MSG_TYPE_REQ;
	msg.code = mrq;
	msg.flags = 0;	/* no DO_ACK, no RING_DOORBELL */
	msg.data = data;
	msg.size = sz;
	ret = mbox_send_message(cl->mbox_chan, &msg);
	mbox_client_txdone(cl->mbox_chan, 0);

	preempt_enable();
	return ret;
}
EXPORT_SYMBOL(tegra_bpmp_send);

/* NOTE: should be called with local irqs disabled */
int tegra_bpmp_send_receive_atomic(int mrq, void *ob_data, int ob_sz,
				   void *ib_data, int ib_sz)
{
	struct tegra_bpmp_mbox_client *cl;
	struct tegra_bpmp_mbox_msg msg;
	int ret;

	if (!connected)
		return -ENODEV;

	cl = acquire_outbox_client();

	msg.type = BPMP_MSG_TYPE_ATOMIC | BPMP_MSG_TYPE_REQ;
	msg.code = mrq;
	msg.flags = BPMP_MSG_DO_ACK;
	msg.data = ob_data;
	msg.size = ob_sz;

	ret = mbox_send_message(cl->mbox_chan, &msg);
	if (ret < 0)
		return ret;
	mbox_client_txdone(cl->mbox_chan, 0);

	return tegra_bpmp_read_response(cl, ib_data, ib_sz);
}
EXPORT_SYMBOL(tegra_bpmp_send_receive_atomic);

int tegra_bpmp_send_receive(int mrq, void *ob_data, int ob_sz,
			    void *ib_data, int ib_sz)
{
	struct tegra_bpmp_mbox_client *cl;
	struct tegra_bpmp_mbox_msg msg;
	int ret;

	if (!connected)
		return -ENODEV;

	cl = acquire_thread_client();
	if (!cl)
		return -EBUSY;

	msg.type = BPMP_MSG_TYPE_REQ;
	msg.code = mrq;
	msg.flags = BPMP_MSG_DO_ACK | BPMP_MSG_RING_DOORBELL;
	msg.data = ob_data;
	msg.size = ob_sz;

	ret = mbox_send_message(cl->mbox_chan, &msg);
	if (ret < 0)
		return ret;

	ret = tegra_bpmp_read_response(cl, ib_data, ib_sz);
	free_thread_client(cl);

	return ret;
}
EXPORT_SYMBOL(tegra_bpmp_send_receive);

/* NOTE: this function may be called in irq context */
static u32 tegra_bpmp_mail_readl(int ch, int offset)
{
	struct tegra_bpmp_mbox_client *cl = &mbox_clients[ch];

	u32 *data = (u32 *)(cl->rx_req + offset);
	return *data;
}

/* NOTE: this function may be called in irq context */
static void tegra_bpmp_mail_return(int ch, int code, int v)
{
	struct tegra_bpmp_mbox_client *cl = &mbox_clients[ch];
	struct tegra_bpmp_mbox_msg msg;

	msg.type = BPMP_MSG_TYPE_RESP;
	msg.data = &v;
	msg.size = sizeof(v);
	mbox_send_message(cl->mbox_chan, &msg);
	mbox_client_txdone(cl->mbox_chan, 0);
}

static void tegra_bpmp_handle_request(struct mbox_client *client, void *data)
{
	struct tegra_bpmp_mbox_client *cl =
		container_of(client, struct tegra_bpmp_mbox_client, client);
	struct mb_data *req = data;
	int ch = cl - mbox_clients;

	cl->rx_req = req;
	bpmp_handle_mail(req->code, ch);
}

static void tegra_bpmp_receive_response(struct mbox_client *client, void *data)
{
	struct tegra_bpmp_mbox_client *cl =
		container_of(client, struct tegra_bpmp_mbox_client, client);

	cl->tx_resp = (struct mb_data *)data;
}

static int __tegra_bpmp_connect(void)
{
	int err, i;

	for (i = 0; i < BPMP_NR_CHANNEL; ++i) {
		struct tegra_bpmp_mbox_client *cl = &mbox_clients[i];

		cl->mbox_chan = mbox_request_channel(&cl->client, i);
		if (IS_ERR(cl->mbox_chan)) {
			err = PTR_ERR(cl->mbox_chan);
			dev_err(cl->client.dev,
				"failed to request mailbox: %d\n", err);
			return err;
		}
	}

	return 0;
}

static int tegra_bpmp_connect(void)
{
	int retry = BPMP_CONNECT_RETRY_MAX;

	while (retry--) {
		msleep(100);
		if (!__tegra_bpmp_connect())
			return 0;
	}

	return -ETIMEDOUT;
}

static void tegra_bpmp_detach(void)
{
	int i;

	for (i = 0; i < BPMP_NR_CHANNEL; ++i) {
		struct tegra_bpmp_mbox_client *cl = &mbox_clients[i];

		mbox_free_channel(cl->mbox_chan);
	}
}

static int tegra_bpmp_ping(struct device *dev)
{
	unsigned long flags;
	ktime_t tm;
	int challenge = 1;
	int reply;
	int err;

	local_irq_save(flags);
	tm = ktime_get();
	err = tegra_bpmp_send_receive_atomic(MRQ_PING,
					     &challenge, sizeof(challenge),
					     &reply, sizeof(reply));
	tm = ktime_sub(ktime_get(), tm);
	local_irq_restore(flags);

	if (!err)
		dev_info(dev, "ping OK: challenge=%d reply=%d timeo=%lld\n",
			 challenge, reply, (long long)ktime_to_us(tm));
	return err;
}

static int tegra_bpmp_get_fwtag(void *fwtag)
{
	unsigned long flags;
	int err;

	spin_lock_irqsave(&bpmp->shared_memory_lock, flags);
	err = tegra_bpmp_send_receive_atomic(MRQ_QUERY_TAG, &shared_phys,
					     sizeof(shared_phys), NULL, 0);
	if (!err)
		memcpy(fwtag, shared_virt, FWTAG_SIZE);
	spin_unlock_irqrestore(&bpmp->shared_memory_lock, flags);

	return err;
}

static int tegra_bpmp_check_fwtag(struct device *dev)
{
	char fwtag[FWTAG_SIZE + 1] = { 0 };
	int err;

	err = tegra_bpmp_get_fwtag(fwtag);
	if (!err)
		dev_info(dev, "fwtag: %s\n", fwtag);

	return err;
}

static int tegra_bpmp_request_mrq(int mrq, bpmp_mrq_handler handler, void *data)
{
	unsigned long flags;
	int idx;
	int ret = 0;

	idx = __MRQ_INDEX(mrq);
	if (idx >= NR_MRQS || !handler)
		return -EINVAL;

	spin_lock_irqsave(&mrq_lock, flags);

	if (mrq_handlers[idx].handler) {
		ret = -EEXIST;
		goto out;
	}

	mrq_handlers[idx].handler = handler;
	mrq_handlers[idx].data = data;

out:
	spin_unlock_irqrestore(&mrq_lock, flags);
	return ret;
}

static struct module_mrq *bpmp_find_module_mrq(u32 module_base)
{
	struct module_mrq *item;

	list_for_each_entry(item, &module_mrq_list, link) {
		if (item->base == module_base)
			return item;
	}

	return NULL;
}

static void bpmp_mrq_module_mail(int code, void *data, int ch)
{
	struct module_mrq *item;
	u32 base;

	base = tegra_bpmp_mail_readl(ch, 0);
	item = bpmp_find_module_mrq(base);
	if (item)
		item->handler(code, item->data, ch);
	else
		tegra_bpmp_mail_return(ch, -ENODEV, 0);
}

static void bpmp_mrq_ping(int code, void *data, int ch)
{
	int challenge, reply;

	challenge = tegra_bpmp_mail_readl(ch, 0);
	reply = challenge  << (smp_processor_id() + 1);
	tegra_bpmp_mail_return(ch, 0, reply);
}

static void bpmp_handle_mail(int mrq, int ch)
{
	struct mrq_handler_data *h;
	int idx;

	idx = __MRQ_INDEX(mrq);
	if (idx >= NR_MRQS) {
		tegra_bpmp_mail_return(ch, -EINVAL, 0);
		return;
	}

	spin_lock(&mrq_lock);

	h = mrq_handlers + idx;
	if (!h->handler) {
		spin_unlock(&mrq_lock);
		tegra_bpmp_mail_return(ch, -ENODEV, 0);
		return;
	}

	h->handler(mrq, h->data, ch);
	spin_unlock(&mrq_lock);
}

static int tegra_bpmp_mailman_setup(void)
{
	int ret;

	ret = tegra_bpmp_request_mrq(MRQ_PING, bpmp_mrq_ping, NULL);
	if (ret)
		return ret;

	return tegra_bpmp_request_mrq(MRQ_MODULE_MAIL,
				      bpmp_mrq_module_mail, NULL);
}

static int tegra_bpmp_mailman_init(struct device *dev)
{
	struct tegra_bpmp_mbox_client *cl;
	dma_addr_t phys;
	int ret;
	int i;

	/* setup channels */
	for (i = 0; i < BPMP_NR_CHANNEL; i++) {
		cl = &mbox_clients[i];
		cl->client.dev = dev;
		cl->client.tx_block = false;
		cl->client.rx_callback = tegra_bpmp_receive_response;
	}

	/* for each thread channel */
	for (i = 0; i < BPMP_NR_THREAD_CHAN; ++i) {
		cl = &mbox_clients[i + THREAD_CHAN_OFFSET];
		cl->client.tx_block = true;
		cl->client.tx_tout = THREAD_CHAN_TIMEOUT;
	}

	/* for each inbox channel */
	for (i = 0; i < BPMP_NR_INBOX_CHAN; ++i) {
		cl = &mbox_clients[i + INBOX_CHAN_OFFSET];
		cl->client.rx_callback = tegra_bpmp_handle_request;
	}

	ret = tegra_bpmp_connect();
	if (ret) {
		dev_err(dev, "connect failed: %d\n", ret);
		goto out;
	}
	connected = true;

	/* alloc shared mem */
	shared_virt = dma_alloc_coherent(dev, SHARED_MEMORY_SIZE,
					 &phys, GFP_KERNEL);
	if (!shared_virt) {
		ret = -ENOMEM;
		goto out;
	}
	shared_phys = phys;

	ret = tegra_bpmp_mailman_setup();
	if (ret) {
		dev_err(dev, "mailman setup failed: %d\n", ret);
		goto out;
	}

	ret = tegra_bpmp_ping(dev);
	if (ret) {
		dev_err(dev, "ping failed: %d\n", ret);
		goto out;
	}

	ret = tegra_bpmp_check_fwtag(dev);
	if (ret) {
		dev_err(dev, "unable to get firmware tag: %d\n", ret);
		goto out;
	}

out:
	return ret;
}

static void tegra_bpmp_start(struct tegra_bpmp *bpmp)
{
	flowctrl_cop_halt();
	writel(bpmp->bootaddr, bpmp->reset_vec);

	reset_control_assert(bpmp->cop_rst);
	udelay(2);
	reset_control_deassert(bpmp->cop_rst);

	flowctrl_cop_unhalt();
}

static int tegra_bpmp_load_fw(struct tegra_bpmp *bpmp, const void *data,
			      int size)
{
	struct device *dev = bpmp->dev;
	struct fw_header *h;
	const int sz = sizeof(h->md5sum);
	char fwtag[sz + 1];

	bpmp->loadfw_virt = dma_alloc_coherent(dev, SZ_256K,
					       &bpmp->loadfw_phys, GFP_KERNEL);
	if (!bpmp->loadfw_virt || !bpmp->loadfw_phys) {
		dev_err(dev, "out of memory\n");
		return -ENOMEM;
	}
	dev_info(dev, "loadfw_phys: 0x%llx\n", bpmp->loadfw_phys);
	dev_info(dev, "loadfw_virt: 0x%p\n", bpmp->loadfw_virt);

	h = (struct fw_header *)data;
	bpmp->bootaddr = bpmp->loadfw_phys + h->reset_offset;

	memcpy(fwtag, h->md5sum, sz);
	fwtag[sz] = 0;

	dev_info(dev, "magic     : %x\n", h->magic);
	dev_info(dev, "version   : %x\n", h->version);
	dev_info(dev, "chip      : %x\n", h->chip_id);
	dev_info(dev, "memsize   : %u bytes\n", h->mem_size);
	dev_info(dev, "reset off : %x\n", h->reset_offset);
	dev_info(dev, "reset addr: %x\n", bpmp->bootaddr);
	dev_info(dev, "fwtag     : %s\n", fwtag);

	if (size > h->mem_size || h->mem_size > SZ_256K) {
		dev_err(dev, "firmware too big\n");
		return -EFAULT;
	}

	memcpy(bpmp->loadfw_virt, data, size);
	memset(bpmp->loadfw_virt + size, 0, SZ_256K - size);

	return 0;
}

static int tegra_bpmp_request_fw(struct tegra_bpmp *bpmp)
{
	struct device *dev = bpmp->dev;
	const struct firmware *fw;
	int ret;

	ret = request_firmware(&fw, BPMP_FIRMWARE_NAME, dev);
	if (ret < 0) {
		dev_err(dev, "firmware not ready\n");
		goto out;
	}

	ret = tegra_bpmp_load_fw(bpmp, fw->data, fw->size);
	if (ret < 0) {
		dev_err(dev, "loading firmware failed\n");
		goto out;
	}

	tegra_bpmp_start(bpmp);
	dev_info(dev, "firmware started\n");

	/* initialize only after firmware is loaded */
	tegra_bpmp_mailman_init(dev);
out:
	release_firmware(fw);
	return ret;
}

static int tegra_bpmp_tolerate_idle(int cpu, int ccxtl, int scxtl)
{
	s32 data[3];

	data[0] = cpu_to_le32(cpu);
	data[1] = cpu_to_le32(ccxtl);
	data[2] = cpu_to_le32(scxtl);

	return tegra_bpmp_send(MRQ_TOLERATE_IDLE, data, sizeof(data));
}

static int tegra_cpu_notify(struct notifier_block *nb, unsigned long action,
                 void *hcpu)
{
	int cpu = (long)hcpu;

	if (bpmp_suspended)
		goto out;

	switch (action) {
	case CPU_POST_DEAD:
	case CPU_DEAD_FROZEN:
		tegra_bpmp_tolerate_idle(cpu, TEGRA_BPMP_PM_CC7,
					 TEGRA_BPMP_PM_SC7);
		break;
	case CPU_UP_PREPARE:
	case CPU_UP_PREPARE_FROZEN:
		tegra_bpmp_tolerate_idle(cpu, TEGRA_BPMP_PM_CC1,
					 TEGRA_BPMP_PM_SC1);
		break;
	}

out:
	return NOTIFY_OK;
}

static struct notifier_block tegra_cpu_nb = {
	.notifier_call = tegra_cpu_notify
};

static int __maybe_unused tegra_bpmp_enable_suspend(int mode, int flags, bool scx_enable)
{
	s32 mb[] = { cpu_to_le32(mode), cpu_to_le32(flags) };
	s32 val = cpu_to_le32(scx_enable);

	tegra_bpmp_send(MRQ_SCX_ENABLE, &val, sizeof(val));
	tegra_bpmp_send(MRQ_ENABLE_SUSPEND, &mb, sizeof(mb));
	return 0;
}

static int __maybe_unused tegra_bpmp_suspend(struct device *dev)
{
	bpmp_suspended = true;
	tegra_bpmp_enable_suspend(TEGRA_BPMP_PM_SC7, 0, true);
	return 0;
}

static int __maybe_unused tegra_bpmp_resume(struct device *dev)
{
	s32 val = 0;

	tegra_bpmp_send(MRQ_SCX_ENABLE, &val, sizeof(val));
	bpmp_suspended = false;
	return 0;
}
SIMPLE_DEV_PM_OPS(tegra_bpmp_pm, tegra_bpmp_suspend, tegra_bpmp_resume);

static const struct of_device_id tegra_bpmp_mailman_of_match[] = {
	{ .compatible = "nvidia,tegra210-bpmp-mailman", },
	{ }
};

static int tegra_bpmp_mailman_probe(struct platform_device *pdev)
{
	struct device_node *mbox_np;
	struct platform_device *mbox_dev;
	struct resource *res;

	mbox_np = of_parse_phandle(pdev->dev.of_node, "nvidia,bpmp-mbox", 0);
	if (!mbox_np) {
		dev_err(&pdev->dev, "Error parsing BPMP-MBOX phandle.\n");
		return -EINVAL;
	}

	mbox_dev = of_find_device_by_node(mbox_np);
	if (!mbox_dev) {
		dev_err(&pdev->dev, "Error finding BPMP-MBOX device.\n");
		return -EINVAL;
	}

	if (!mbox_dev->dev.driver) {
		dev_err(&pdev->dev, "Error BPMP-MBOX driver not ready.\n");
		return -EPROBE_DEFER;
	}

	bpmp = devm_kzalloc(&pdev->dev, sizeof(*bpmp), GFP_KERNEL);
	if (!bpmp)
		return -ENOMEM;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					   "cop-reset-vec");
	bpmp->reset_vec = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(bpmp->reset_vec))
		return PTR_ERR(bpmp->reset_vec);

	bpmp->cop_rst = devm_reset_control_get(&pdev->dev, "cop");
	if (IS_ERR(bpmp->cop_rst))
		return PTR_ERR(bpmp->cop_rst);

	bpmp->dev = &pdev->dev;
	spin_lock_init(&bpmp->shared_memory_lock);

	if (tegra_bpmp_request_fw(bpmp) < 0)
		return -EINVAL;

	register_cpu_notifier(&tegra_cpu_nb);

	return 0;
}

static int tegra_bpmp_mailman_remove(struct platform_device *pdev)
{
	unregister_cpu_notifier(&tegra_cpu_nb);
	if (connected)
		tegra_bpmp_detach();

	return 0;
}

static struct platform_driver tegra_bpmp_mailman_driver = {
	.driver = {
		.name = "tegra-bpmp-mailman",
		.of_match_table = tegra_bpmp_mailman_of_match,
		.pm = &tegra_bpmp_pm,
	},
	.probe = tegra_bpmp_mailman_probe,
	.remove = tegra_bpmp_mailman_remove,
};
module_platform_driver(tegra_bpmp_mailman_driver);

MODULE_DESCRIPTION("NVIDIA Tegra BPMP-lite mailman driver");
MODULE_LICENSE("GPL v2");
