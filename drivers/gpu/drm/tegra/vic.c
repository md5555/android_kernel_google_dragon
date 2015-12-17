/*
 * Copyright (c) 2014-2015, NVIDIA Corporation.
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

#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/clk.h>
#include <linux/host1x.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <soc/tegra/pmc.h>
#include <soc/tegra/tegra_emc.h>
#include <linux/iommu.h>

#include "drm.h"
#include "falcon.h"

#define VIC_MAX_CLK_RATE				627200000

#define NV_PVIC_THI_CLK_OVERRIDE			0x00000e00
#define NV_PVIC_THI_SLCG_OVERRIDE_LOW_A			0x0000008c
#define NV_PVIC_THI_SLCG_OVERRIDE_HIGH_A		0x00000088
#define NV_PVIC_FALCON_CGCTL				0x000010a0
#define NV_PVIC_TFBIF_MCCIF_FIFOCTRL			0x00001604
#define NV_PVIC_FALCON_CG2				0x00001134
#define NV_PVIC_FALCON_CG1_SLCG				0x0000117c

#define NVA0B6_VIDEO_COMPOSITOR_SET_APPLICATION_ID	0x00000200
#define NVA0B6_VIDEO_COMPOSITOR_SET_FCE_UCODE_SIZE	0x0000071C
#define NVA0B6_VIDEO_COMPOSITOR_SET_FCE_UCODE_OFFSET	0x0000072C

/* Used in the vic firmware binary to indicate that there is no FCE. */
#define FALCON_FW_FCE_ABSENT			0xa5a5a5a5

struct fce {
	u32 data_offset;
	u32 size;
};

struct vic_ucode_fce_header_v1 {
	u32 fce_ucode_offset;
	u32 fce_ucode_buffer_size;
	u32 fce_ucode_size;
};

struct vic_config {
	const char *ucode_name;
	u32 class_id;
	int powergate_id;
	const struct host1x_client_clock *clocks;
};

struct vic {
	struct tegra_drm_client client;

	struct falcon falcon;

	struct host1x_channel *channel;
	struct device *dev;
	struct clk *clk;
	struct clk *cbus_clk;
	struct clk *emc_clk;
	struct reset_control *rst;
	struct notifier_block slcg_notifier;

	struct fce fce;
	struct vic_ucode_fce_header_v1 *fce_header;

	struct iommu_domain *domain;

	/* Platform configuration */
	struct vic_config *config;
};

static inline struct vic *to_vic(struct tegra_drm_client *client)
{
	return container_of(client, struct vic, client);
}

static inline void vic_writel(struct vic *vic, u32 v, u32 r)
{
	writel(v, vic->falcon.regs + r);
}

static int vic_slcg_handler(struct notifier_block *nb,
	unsigned long unused0, void *unused1)
{
	struct vic *vic = container_of(nb, struct vic, slcg_notifier);

	/* Disable slcg, wait a while and re-enable it */
	vic_writel(vic, 0xffffffff, NV_PVIC_THI_CLK_OVERRIDE);
	vic_writel(vic, 0x000000ff, NV_PVIC_THI_SLCG_OVERRIDE_HIGH_A);
	vic_writel(vic, 0xffffffff, NV_PVIC_THI_SLCG_OVERRIDE_LOW_A);
	vic_writel(vic, 0x00000001, NV_PVIC_FALCON_CGCTL);
	vic_writel(vic, 0x00000000, NV_PVIC_TFBIF_MCCIF_FIFOCTRL);
	vic_writel(vic, 0x00000003, NV_PVIC_FALCON_CG2);
	vic_writel(vic, 0x00000003, NV_PVIC_FALCON_CG1_SLCG);

	udelay(1);

	vic_writel(vic, 0x00000000, NV_PVIC_THI_CLK_OVERRIDE);
	vic_writel(vic, 0x00000000, NV_PVIC_THI_SLCG_OVERRIDE_HIGH_A);
	vic_writel(vic, 0x00000000, NV_PVIC_THI_SLCG_OVERRIDE_LOW_A);
	vic_writel(vic, 0x00000000, NV_PVIC_FALCON_CGCTL);
	vic_writel(vic, 0x00000000, NV_PVIC_TFBIF_MCCIF_FIFOCTRL);
	vic_writel(vic, 0x00000000, NV_PVIC_FALCON_CG2);
	vic_writel(vic, 0x00000000, NV_PVIC_FALCON_CG1_SLCG);

	return NOTIFY_OK;
}

static int vic_power_off(struct device *dev)
{
	struct vic *vic = dev_get_drvdata(dev);
	int err;

	err = falcon_power_off(&vic->falcon);
	if (err)
		return err;

	err = reset_control_assert(vic->rst);
	if (err)
		return err;

	clk_disable_unprepare(vic->clk);
	host1x_module_disable_clocks(&vic->client.base);

	return tegra_pmc_powergate(vic->config->powergate_id);
}

static int vic_power_on(struct device *dev)
{
	struct vic *vic = dev_get_drvdata(dev);
	int err;

	err = falcon_power_on(&vic->falcon);
	if (err)
		return err;

	err = host1x_module_enable_clocks(&vic->client.base);
	if (err)
		goto err_host1x_clk;

	err = tegra_pmc_unpowergate(vic->config->powergate_id);
	if (err)
		goto err_unpowergate;

	err = clk_prepare_enable(vic->clk);
	if (err)
		goto err_vic_clk;

	return 0;

err_vic_clk:
	tegra_pmc_powergate(vic->config->powergate_id);
err_unpowergate:
	host1x_module_disable_clocks(&vic->client.base);
err_host1x_clk:
	falcon_power_off(&vic->falcon);
	return err;
}

static void vic_finalize_poweron(struct vic *vic)
{
	falcon_execute_method(&vic->falcon,
			      NVA0B6_VIDEO_COMPOSITOR_SET_APPLICATION_ID,
			      1);

	falcon_execute_method(&vic->falcon,
			      NVA0B6_VIDEO_COMPOSITOR_SET_FCE_UCODE_SIZE,
			      vic->fce.size);

	falcon_execute_method(&vic->falcon,
			      NVA0B6_VIDEO_COMPOSITOR_SET_FCE_UCODE_OFFSET,
			      (vic->falcon.ucode_paddr +
			       vic->fce.data_offset) >> 8);
}

static void vic_reset(struct device *dev)
{
	struct vic *vic = dev_get_drvdata(dev);

	vic_power_off(dev);
	vic_power_on(dev);
	falcon_boot(&vic->falcon, vic->config->ucode_name);

	vic_finalize_poweron(vic);
}

static int vic_init(struct host1x_client *client)
{
	struct tegra_drm_client *drm = host1x_to_drm_client(client);
	struct drm_device *dev = dev_get_drvdata(client->parent);
	struct tegra_drm *tegra = dev->dev_private;
	struct vic *vic = to_vic(drm);
	int err;

	if (tegra->domain) {
		err = iommu_attach_device(tegra->domain, client->dev);
		if (err < 0) {
			dev_err(client->dev, "failed to attach to domain: %d\n",
				err);
			return err;
		}

		vic->domain = tegra->domain;
	}

	vic->channel = host1x_channel_request(client);
	if (!vic->channel) {
		err = -ENOMEM;
		goto error_iommu_detach_device;
	}

	client->syncpts[0] = host1x_syncpt_request(client->dev, 0);
	if (!client->syncpts[0]) {
		err = -ENOMEM;
		goto error_host1x_channel_free;
	}

	err = tegra_drm_register_client(dev->dev_private, drm);
	if (err)
		goto error_host1x_syncpt_free;

	return 0;

error_host1x_syncpt_free:
	host1x_syncpt_free(client->syncpts[0]);
error_host1x_channel_free:
	host1x_channel_free(vic->channel);
error_iommu_detach_device:
	if (vic->domain) {
		iommu_detach_device(vic->domain, vic->dev);
		vic->domain = NULL;
	}
	return err;
}

static int vic_exit(struct host1x_client *client)
{
	struct tegra_drm_client *drm = host1x_to_drm_client(client);
	struct drm_device *dev = dev_get_drvdata(client->parent);
	struct vic *vic = to_vic(drm);
	int err;

	err = tegra_drm_unregister_client(dev->dev_private, drm);

	host1x_syncpt_free(client->syncpts[0]);
	host1x_channel_free(vic->channel);

	if (vic->domain) {
		iommu_detach_device(vic->domain, vic->dev);
		vic->domain = NULL;
	}

	return err;
}

static int vic_get_clk_rate(struct host1x_client *client, u64 *data, u32 type)
{
	struct tegra_drm_client *drm = host1x_to_drm_client(client);
	struct vic *vic = to_vic(drm);

	switch (type) {
	case DRM_TEGRA_REQ_TYPE_CLK_HZ:
		*data = clk_get_rate(vic->cbus_clk);
		break;
	case DRM_TEGRA_REQ_TYPE_BW_KBPS:
		*data = clk_get_rate(vic->emc_clk);
		break;
	default:
		dev_err(vic->dev, "Unknown Clock request type\n");
		return -EINVAL;
	}
	return 0;
}

static int vic_set_clk_rate(struct host1x_client *client, u64 data, u32 type)
{
	struct tegra_drm_client *drm = host1x_to_drm_client(client);
	struct vic *vic = to_vic(drm);

	switch (type) {
	case DRM_TEGRA_REQ_TYPE_CLK_HZ:
		if (data > VIC_MAX_CLK_RATE)
			data = VIC_MAX_CLK_RATE;
		return clk_set_rate(vic->cbus_clk, data);
	case DRM_TEGRA_REQ_TYPE_BW_KBPS:
		return clk_set_rate(vic->emc_clk,
				tegra_emc_bw_to_freq_req(data) * 1000);
	default:
		dev_err(vic->dev, "Unknown Clock request type\n");
		return -EINVAL;
	}
}

static const struct host1x_client_ops vic_client_ops = {
	.init = vic_init,
	.exit = vic_exit,
	.get_clk_rate = vic_get_clk_rate,
	.set_clk_rate = vic_set_clk_rate,
};

static int vic_open_channel(struct tegra_drm_client *client,
			    struct tegra_drm_context *context)
{
	struct vic *vic = to_vic(client);
	int err;
	struct falcon_ucode_v1 ucode;

	pm_runtime_get_sync(vic->dev);

	err = falcon_boot(&vic->falcon, vic->config->ucode_name);
	if (err)
		return err;

	ucode.bin_header = (struct falcon_ucode_bin_header_v1 *)vic->falcon.ucode_vaddr;
	if (ucode.bin_header->fce_bin_header_offset == FALCON_FW_FCE_ABSENT )
		return -EINVAL;

	vic->fce_header = (struct vic_ucode_fce_header_v1 *)
			  (((void *)vic->falcon.ucode_vaddr) +
			  ucode.bin_header->fce_bin_header_offset);
	vic->fce.size = vic->fce_header->fce_ucode_size;
	vic->fce.data_offset =
		ucode.bin_header->fce_bin_data_offset;

	vic_finalize_poweron(vic);

	context->channel = host1x_channel_get(vic->channel, &context->user);
	if (!context->channel)
		return -ENOMEM;

	return 0;
}

static void vic_close_channel(struct tegra_drm_context *context)
{
	struct vic *vic = to_vic(context->client);

	if (!context->channel)
		return;

	host1x_channel_put(context->channel, &context->user);
	context->channel = NULL;

	pm_runtime_mark_last_busy(vic->dev);
	pm_runtime_put_autosuspend(vic->dev);
}

static int vic_is_addr_reg(struct device *dev, u32 class, u32 offset)
{
	/**
	 * Falcon communicates addresses and commands through the same
	 * method/data registers
	 */

	return 0;
}

static const struct tegra_drm_client_ops vic_ops = {
	.open_channel = vic_open_channel,
	.close_channel = vic_close_channel,
	.is_addr_reg = vic_is_addr_reg,
	.reset = vic_reset,
	.submit = tegra_drm_submit,
};

static void *vic_falcon_alloc(struct falcon *falcon, size_t size,
			      dma_addr_t *paddr)
{
	struct vic *vic = container_of(falcon, struct vic, falcon);
	struct tegra_drm_client *drm = &vic->client;
	struct drm_device *dev = dev_get_drvdata(drm->base.parent);
	struct tegra_bo *bo;

	bo = tegra_bo_create(dev, size, 0);
	bo->vaddr = vmap(bo->pages, bo->num_pages, VM_MAP,
			 pgprot_writecombine(PAGE_KERNEL));
	if (!bo->vaddr) {
		dev_err(vic->dev, "dma memory allocation failed");
		return NULL;
	}

	*paddr = bo->paddr;

	return bo->vaddr;
}

static void vic_falcon_free(struct falcon *falcon, size_t size,
			    dma_addr_t paddr, void *vaddr)
{
	struct vic *vic = container_of(falcon, struct vic, falcon);
	DEFINE_DMA_ATTRS(attrs);

	dma_free_attrs(vic->dev, size, vaddr, paddr, &attrs);
}

static const struct falcon_ops vic_falcon_ops = {
	.alloc = vic_falcon_alloc,
	.free = vic_falcon_free,
};

static const struct host1x_client_clock vic_t210_clocks[] = {
	{
		.clk_name = "vic_cbus",
		.default_rate = 448000000,
		.valid_constraint_types =
			BIT(HOST1X_USER_CONSTRAINT_TYPE_HZ),
	}, {
		.clk_name = "emc",
		.default_rate = 204000000,
		.valid_constraint_types =
			BIT(HOST1X_USER_CONSTRAINT_TYPE_HZ) |
			BIT(HOST1X_USER_CONSTRAINT_TYPE_BW_KBPS),
	},
};

static const struct vic_config vic_vic_t210_config = {
	.ucode_name = "nvidia/tegra210/vic.bin",
	.class_id = HOST1X_CLASS_VIC,
	.powergate_id = TEGRA_POWERGATE_VIC,
	.clocks = vic_t210_clocks,
};

static const struct of_device_id vic_match[] = {
	{ .compatible = "nvidia,tegra210-vic",
		.data = &vic_vic_t210_config },
	{ },
};

static int vic_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct vic_config *vic_config;
	struct device *dev = &pdev->dev;
	struct host1x_syncpt **syncpts;
	struct resource *regs;
	struct vic *vic;
	int err;

	if (!dev->of_node) {
		dev_err(&pdev->dev, "no dt node\n");
		return -EINVAL;
	}

	match = of_match_device(vic_match, dev);
	if (match)
		vic_config = (struct vic_config *)match->data;
	else
		return -ENXIO;

	vic = devm_kzalloc(dev, sizeof(*vic), GFP_KERNEL);
	if (!vic)
		return -ENOMEM;

	syncpts = devm_kzalloc(dev, sizeof(*syncpts), GFP_KERNEL);
	if (!syncpts)
		return -ENOMEM;

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	vic->falcon.regs = devm_ioremap_resource(&pdev->dev, regs);
	if (IS_ERR(vic->falcon.regs))
		return PTR_ERR(vic->falcon.regs);

	vic->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(vic->clk)) {
		dev_err(&pdev->dev, "cannot get clock\n");
		return PTR_ERR(vic->clk);
	}
	vic->cbus_clk = devm_clk_get(dev, "vic_cbus");
	if (IS_ERR(vic->cbus_clk)) {
		dev_err(&pdev->dev, "cannot get vic cbus clock\n");
		return PTR_ERR(vic->cbus_clk);
	}
	vic->emc_clk = devm_clk_get(dev, "emc");
	if (IS_ERR(vic->emc_clk)) {
		dev_err(&pdev->dev, "cannot get emc clock\n");
		return PTR_ERR(vic->emc_clk);
	}
	vic->rst = devm_reset_control_get(&pdev->dev, "vic");
	if (IS_ERR(vic->rst)) {
		dev_err(&pdev->dev, "cannot get reset\n");
		return PTR_ERR(vic->rst);
	}

	vic->falcon.dev = dev;
	vic->falcon.ops = &vic_falcon_ops;

	INIT_LIST_HEAD(&vic->client.base.list);
	vic->client.base.ops = &vic_client_ops;
	vic->client.base.dev = dev;
	vic->client.base.class = vic_config->class_id;
	vic->client.base.syncpts = syncpts;
	vic->client.base.num_syncpts = 1;
	memcpy(&vic->client.base.clocks, vic_config->clocks,
	       sizeof(vic_t210_clocks));
	vic->dev = dev;
	vic->config = vic_config;

	INIT_LIST_HEAD(&vic->client.list);
	vic->client.ops = &vic_ops;

	err = falcon_init(&vic->falcon);
	if (err) {
		dev_err(dev, "failed initializing falcon helper\n");
		return err;
	}

	vic->slcg_notifier.notifier_call = vic_slcg_handler;
	tegra_slcg_register_notifier(vic->config->powergate_id,
		&vic->slcg_notifier);

	platform_set_drvdata(pdev, vic);

	err = host1x_client_register(&vic->client.base);
	if (err < 0) {
		dev_err(dev, "failed to register host1x client: %d\n", err);
		goto error_slcg;
	}

	err = vic_power_on(&pdev->dev);
	if (err) {
		dev_err(dev, "cannot turn on the device\n");
		goto error_client_unregister;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_set_autosuspend_delay(dev, 5000);

	dev_info(&pdev->dev, "initialized");

	return 0;

error_client_unregister:
	host1x_client_unregister(&vic->client.base);
error_slcg:
	tegra_slcg_unregister_notifier(vic->config->powergate_id,
				       &vic->slcg_notifier);
	falcon_exit(&vic->falcon);
	return err;
}

static int vic_remove(struct platform_device *pdev)
{
	struct vic *vic = platform_get_drvdata(pdev);
	int err;

	vic_power_off(&pdev->dev);

	err = host1x_client_unregister(&vic->client.base);
	if (err < 0)
		dev_err(&pdev->dev, "failed to unregister host1x client: %d\n",
			err);

	tegra_slcg_unregister_notifier(vic->config->powergate_id,
		&vic->slcg_notifier);

	falcon_exit(&vic->falcon);

	return err;
}

static int __maybe_unused vic_runtime_suspend(struct device *dev)
{
	dev_info(dev, "%s\n", __func__);
	return vic_power_off(dev);
}

static int __maybe_unused vic_runtime_resume(struct device *dev)
{
	dev_info(dev, "%s\n", __func__);
	return vic_power_on(dev);
}

static int __maybe_unused vic_suspend(struct device *dev)
{
	dev_info(dev, "%s\n", __func__);

	if (pm_runtime_status_suspended(dev))
		return 0;

	return vic_power_off(dev);
}

static int __maybe_unused vic_resume(struct device *dev)
{
	struct vic *vic = dev_get_drvdata(dev);
	int err;

	dev_info(dev, "%s\n", __func__);

	if (pm_runtime_status_suspended(dev))
		return 0;

	err = vic_power_on(dev);
	if (err) {
		dev_err(dev, "failed to power on during resume\n");
		return err;
	}

	return falcon_boot(&vic->falcon, vic->config->ucode_name);
}

static const struct dev_pm_ops vic_pm_ops = {
	SET_RUNTIME_PM_OPS(vic_runtime_suspend, vic_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(vic_suspend, vic_resume)
};

struct platform_driver tegra_vic_driver = {
	.driver = {
		.name = "tegra-vic",
		.of_match_table = vic_match,
		.pm = &vic_pm_ops,
	},
	.probe = vic_probe,
	.remove = vic_remove,
};
