/*
 * Copyright (c) 2015, NVIDIA Corporation.
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

#define NVENC_MAX_CLK_RATE	716800000

struct nvenc_config {
	const char *ucode_name;
	u32 class_id;
	int powergate_id;
	const struct host1x_client_clock *clocks;
};

struct nvenc {
	struct falcon falcon;

	struct tegra_drm_client client;

	struct host1x_channel *channel;
	struct device *dev;
	struct clk *clk;
	struct clk *cbus_clk;
	struct clk *emc_clk;
	struct reset_control *rst;

	struct iommu_domain *domain;

	/* Platform configuration */
	struct nvenc_config *config;
};

static inline struct nvenc *to_nvenc(struct tegra_drm_client *client)
{
	return container_of(client, struct nvenc, client);
}

static int nvenc_power_off(struct device *dev)
{
	struct nvenc *nvenc = dev_get_drvdata(dev);
	int err;

	err = falcon_power_off(&nvenc->falcon);
	if (err)
		return err;

	err = reset_control_assert(nvenc->rst);
	if (err)
		return err;

	clk_disable_unprepare(nvenc->clk);
	host1x_module_disable_clocks(&nvenc->client.base);

	return tegra_pmc_powergate(nvenc->config->powergate_id);
}

static int nvenc_power_on(struct device *dev)
{
	struct nvenc *nvenc = dev_get_drvdata(dev);
	int err;

	err = falcon_power_on(&nvenc->falcon);
	if (err)
		return err;

	err = host1x_module_enable_clocks(&nvenc->client.base);
	if (err)
		goto err_host1x_clk;

	err = tegra_pmc_unpowergate(nvenc->config->powergate_id);
	if (err)
		goto err_unpowergate;

	err = clk_prepare_enable(nvenc->clk);
	if (err)
		goto err_nvenc_clk;
	return err;

err_nvenc_clk:
	tegra_pmc_powergate(nvenc->config->powergate_id);
err_unpowergate:
	host1x_module_disable_clocks(&nvenc->client.base);
err_host1x_clk:
	falcon_power_off(&nvenc->falcon);
	return err;
}

static void nvenc_reset(struct device *dev)
{
	struct nvenc *nvenc = dev_get_drvdata(dev);

	nvenc_power_off(dev);
	nvenc_power_on(dev);
	falcon_boot(&nvenc->falcon, nvenc->config->ucode_name);
}

static int nvenc_init(struct host1x_client *client)
{
	struct tegra_drm_client *drm = host1x_to_drm_client(client);
	struct drm_device *dev = dev_get_drvdata(client->parent);
	struct tegra_drm *tegra = dev->dev_private;
	struct nvenc *nvenc = to_nvenc(drm);
	int err;

	if (tegra->domain) {
		err = iommu_attach_device(tegra->domain, client->dev);
		if (err < 0) {
			dev_err(client->dev, "failed to attach to domain: %d\n",
				err);
			return err;
		}

		nvenc->domain = tegra->domain;
	}

	nvenc->channel = host1x_channel_request(client);
	if (!nvenc->channel) {
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
	host1x_channel_free(nvenc->channel);
error_iommu_detach_device:
	if (nvenc->domain) {
		iommu_detach_device(nvenc->domain, nvenc->dev);
		nvenc->domain = NULL;
	}
	return err;
}

static int nvenc_exit(struct host1x_client *client)
{
	struct tegra_drm_client *drm = host1x_to_drm_client(client);
	struct drm_device *dev = dev_get_drvdata(client->parent);
	struct nvenc *nvenc = to_nvenc(drm);
	int err;

	err = tegra_drm_unregister_client(dev->dev_private, drm);

	host1x_syncpt_free(client->syncpts[0]);
	host1x_channel_free(nvenc->channel);

	if (nvenc->domain) {
		iommu_detach_device(nvenc->domain, nvenc->dev);
		nvenc->domain = NULL;
	}

	return err;
}

static int nvenc_get_clk_rate(struct host1x_client *client, u64 *data,
					u32 type)
{
	struct tegra_drm_client *drm = host1x_to_drm_client(client);
	struct nvenc *nvenc = to_nvenc(drm);

	switch (type) {
	case DRM_TEGRA_REQ_TYPE_CLK_HZ:
		*data = clk_get_rate(nvenc->cbus_clk);
		break;
	case DRM_TEGRA_REQ_TYPE_BW_KBPS:
		*data = clk_get_rate(nvenc->emc_clk);
		break;
	default:
		dev_err(nvenc->dev, "Unknown Clock request type\n");
		return -EINVAL;
	}
	return 0;
}

static int nvenc_set_clk_rate(struct host1x_client *client, u64 data,
					u32 type)
{
	struct tegra_drm_client *drm = host1x_to_drm_client(client);
	struct nvenc *nvenc = to_nvenc(drm);

	switch (type) {
	case DRM_TEGRA_REQ_TYPE_CLK_HZ:
		if (data > NVENC_MAX_CLK_RATE)
			data = NVENC_MAX_CLK_RATE;
		return clk_set_rate(nvenc->cbus_clk, data);
	case DRM_TEGRA_REQ_TYPE_BW_KBPS:
		return clk_set_rate(nvenc->emc_clk,
				tegra_emc_bw_to_freq_req(data) * 1000);
	default:
		dev_err(nvenc->dev, "Unknown Clock request type\n");
		return -EINVAL;
	}
}

static const struct host1x_client_ops nvenc_client_ops = {
	.init = nvenc_init,
	.exit = nvenc_exit,
	.get_clk_rate = nvenc_get_clk_rate,
	.set_clk_rate = nvenc_set_clk_rate,
};

static int nvenc_open_channel(struct tegra_drm_client *client,
			      struct tegra_drm_context *context)
{
	struct nvenc *nvenc = to_nvenc(client);
	int err;

	err = host1x_module_busy(&client->base);
	if (err)
		return err;

	err = falcon_boot(&nvenc->falcon, nvenc->config->ucode_name);
	if (err)
		goto done;

	context->channel = host1x_channel_get(nvenc->channel, &context->user);
	if (!context->channel)
		err = -ENOMEM;

done:
	host1x_module_idle(&client->base);

	return err;
}

static void nvenc_close_channel(struct tegra_drm_context *context)
{
	if (!context->channel)
		return;

	host1x_channel_put(context->channel, &context->user);
	context->channel = NULL;
}

static int nvenc_is_addr_reg(struct device *dev, u32 class, u32 offset)
{
	/**
	 * Falcon communicates addresses and commands through the same
	 * method/data registers
	 */

	return 0;
}

static const struct tegra_drm_client_ops nvenc_ops = {
	.open_channel = nvenc_open_channel,
	.close_channel = nvenc_close_channel,
	.is_addr_reg = nvenc_is_addr_reg,
	.reset = nvenc_reset,
	.submit = tegra_drm_submit,
};

static void *nvenc_falcon_alloc(struct falcon *falcon, size_t size,
				dma_addr_t *paddr)
{
	struct nvenc *nvenc = (struct nvenc *)falcon;
	struct tegra_drm_client *drm = &nvenc->client;
	struct drm_device *dev = dev_get_drvdata(drm->base.parent);
	struct tegra_bo *bo;

	bo = tegra_bo_create(dev, size, 0);
	bo->vaddr = vmap(bo->pages, bo->num_pages, VM_MAP,
			 pgprot_writecombine(PAGE_KERNEL));
	if (!bo->vaddr) {
		dev_err(nvenc->dev, "dma memory allocation failed");
		return NULL;
	}

	*paddr = bo->paddr;

	return bo->vaddr;
}

static void nvenc_falcon_free(struct falcon *falcon, size_t size,
			      dma_addr_t paddr, void *vaddr)
{
	struct nvenc *nvenc = (struct nvenc *)falcon;
	DEFINE_DMA_ATTRS(attrs);

	dma_free_attrs(nvenc->dev, size, vaddr, paddr, &attrs);
}

static const struct falcon_ops nvenc_falcon_ops = {
	.alloc = nvenc_falcon_alloc,
	.free = nvenc_falcon_free,
};

static const struct host1x_client_clock nvenc_t210_clocks[] = {
	{
		.clk_name = "nvenc_cbus",
		.default_rate = 192000000,
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

static const struct nvenc_config nvenc_nvenc_t210_config = {
	.ucode_name = "nvidia/tegra210/nvenc.bin",
	.class_id = HOST1X_CLASS_NVENC,
	.powergate_id = TEGRA_POWERGATE_MPE,
	.clocks = nvenc_t210_clocks,
};

static const struct of_device_id nvenc_match[] = {
	{ .compatible = "nvidia,tegra210-nvenc",
		.data = &nvenc_nvenc_t210_config },
	{ },
};

static int nvenc_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct nvenc_config *nvenc_config;
	struct device *dev = &pdev->dev;
	struct host1x_syncpt **syncpts;
	struct resource *regs;
	struct nvenc *nvenc;
	int err;

	if (!dev->of_node) {
		dev_err(&pdev->dev, "no dt node\n");
		return -EINVAL;
	}

	match = of_match_device(nvenc_match, dev);
	if (match)
		nvenc_config = (struct nvenc_config *)match->data;
	else
		return -ENXIO;

	nvenc = devm_kzalloc(dev, sizeof(*nvenc), GFP_KERNEL);
	if (!nvenc)
		return -ENOMEM;

	syncpts = devm_kzalloc(dev, sizeof(*syncpts), GFP_KERNEL);
	if (!syncpts)
		return -ENOMEM;

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	nvenc->falcon.regs = devm_ioremap_resource(&pdev->dev, regs);
	if (IS_ERR(nvenc->falcon.regs))
		return PTR_ERR(nvenc->falcon.regs);

	nvenc->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(nvenc->clk)) {
		dev_err(&pdev->dev, "cannot get clock\n");
		return PTR_ERR(nvenc->clk);
	}
	nvenc->cbus_clk = devm_clk_get(dev, "nvenc_cbus");
	if (IS_ERR(nvenc->cbus_clk)) {
		dev_err(dev, "cannot get nvenc cbus clock\n");
		return PTR_ERR(nvenc->cbus_clk);
	}
	nvenc->emc_clk = devm_clk_get(dev, "emc");
	if (IS_ERR(nvenc->emc_clk)) {
		dev_err(dev, "cannot get emc clock\n");
		return PTR_ERR(nvenc->emc_clk);
	}
	nvenc->rst = devm_reset_control_get(&pdev->dev, "nvenc");
	if (IS_ERR(nvenc->rst)) {
		dev_err(&pdev->dev, "cannot get reset\n");
		return PTR_ERR(nvenc->rst);
	}

	nvenc->falcon.dev = dev;
	nvenc->falcon.ops = &nvenc_falcon_ops;

	INIT_LIST_HEAD(&nvenc->client.base.list);
	nvenc->client.base.ops = &nvenc_client_ops;
	nvenc->client.base.dev = dev;
	nvenc->client.base.class = nvenc_config->class_id;
	nvenc->client.base.syncpts = syncpts;
	nvenc->client.base.num_syncpts = 1;
	memcpy(&nvenc->client.base.clocks, nvenc_config->clocks,
	       sizeof(nvenc_t210_clocks));
	nvenc->dev = dev;
	nvenc->config = nvenc_config;

	INIT_LIST_HEAD(&nvenc->client.list);
	nvenc->client.ops = &nvenc_ops;

	err = falcon_init(&nvenc->falcon);
	if (err) {
		dev_err(dev, "failed initializing falcon helper\n");
		return err;
	}

	platform_set_drvdata(pdev, nvenc);

	err = host1x_client_register(&nvenc->client.base);
	if (err < 0) {
		dev_err(dev, "failed to register host1x client: %d\n", err);
		goto error_falcon_exit;
	}

	err = nvenc_power_on(&pdev->dev);
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
	host1x_client_unregister(&nvenc->client.base);
error_falcon_exit:
	falcon_exit(&nvenc->falcon);
	return err;
}

static int nvenc_remove(struct platform_device *pdev)
{
	struct nvenc *nvenc = platform_get_drvdata(pdev);
	int err;

	nvenc_power_off(&pdev->dev);

	err = host1x_client_unregister(&nvenc->client.base);
	if (err < 0)
		dev_err(&pdev->dev, "failed to unregister host1x client: %d\n",
			err);

	falcon_exit(&nvenc->falcon);

	return err;
}

static int __maybe_unused nvenc_runtime_suspend(struct device *dev)
{
	dev_info(dev, "%s\n", __func__);
	return nvenc_power_off(dev);
}

static int __maybe_unused nvenc_runtime_resume(struct device *dev)
{
	struct nvenc *nvenc = dev_get_drvdata(dev);
	int err;

	dev_info(dev, "%s\n", __func__);

	err = nvenc_power_on(dev);
	if (err)
		return err;

	return falcon_boot(&nvenc->falcon, nvenc->config->ucode_name);
}

static int __maybe_unused nvenc_suspend(struct device *dev)
{
	dev_info(dev, "%s\n", __func__);

	if (pm_runtime_status_suspended(dev))
		return 0;

	return nvenc_power_off(dev);
}

static int __maybe_unused nvenc_resume(struct device *dev)
{
	struct nvenc *nvenc = dev_get_drvdata(dev);
	int err;

	dev_info(dev, "%s\n", __func__);

	if (pm_runtime_status_suspended(dev))
		return 0;

	err = nvenc_power_on(dev);
	if (err) {
		dev_err(dev, "failed to power on during resume\n");
		return err;
	}

	return falcon_boot(&nvenc->falcon, nvenc->config->ucode_name);
}

static const struct dev_pm_ops nvenc_pm_ops = {
	SET_RUNTIME_PM_OPS(nvenc_runtime_suspend, nvenc_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(nvenc_suspend, nvenc_resume)
};

struct platform_driver tegra_nvenc_driver = {
	.driver = {
		.name = "tegra-nvenc",
		.of_match_table = nvenc_match,
		.pm = &nvenc_pm_ops,
	},
	.probe = nvenc_probe,
	.remove = nvenc_remove,
};
