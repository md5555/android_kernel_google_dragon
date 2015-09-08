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
#include <linux/iommu.h>

#include "drm.h"
#include "falcon.h"

struct nvjpg_config {
	const char *ucode_name;
	u32 class_id;
	int powergate_id;
};

struct nvjpg {
	struct falcon falcon;

	struct tegra_drm_client client;

	struct host1x_channel *channel;
	struct device *dev;
	struct clk *clk;
	struct reset_control *rst;

	struct iommu_domain *domain;

	/* Platform configuration */
	struct nvjpg_config *config;
};

static inline struct nvjpg *to_nvjpg(struct tegra_drm_client *client)
{
	return container_of(client, struct nvjpg, client);
}

static int nvjpg_power_off(struct device *dev)
{
	struct nvjpg *nvjpg = dev_get_drvdata(dev);
	int err;

	err = falcon_power_off(&nvjpg->falcon);
	if (err)
		return err;

	err = reset_control_assert(nvjpg->rst);
	if (err)
		return err;

	clk_disable_unprepare(nvjpg->clk);

	return tegra_powergate_power_off(nvjpg->config->powergate_id);
}

static int nvjpg_power_on(struct device *dev)
{
	struct nvjpg *nvjpg = dev_get_drvdata(dev);
	int err;

	err = falcon_power_on(&nvjpg->falcon);
	if (err)
		return err;

	err = tegra_powergate_sequence_power_up(nvjpg->config->powergate_id);
	if (err)
		return err;

	err = clk_prepare_enable(nvjpg->clk);
	if (err)
		tegra_powergate_power_off(nvjpg->config->powergate_id);

	return err;
}

static void nvjpg_reset(struct device *dev)
{
	struct nvjpg *nvjpg = dev_get_drvdata(dev);

	nvjpg_power_off(dev);
	nvjpg_power_on(dev);
	falcon_boot(&nvjpg->falcon, nvjpg->config->ucode_name);
}

static int nvjpg_init(struct host1x_client *client)
{
	struct tegra_drm_client *drm = host1x_to_drm_client(client);
	struct drm_device *dev = dev_get_drvdata(client->parent);
	struct tegra_drm *tegra = dev->dev_private;
	struct nvjpg *nvjpg = to_nvjpg(drm);
	int err;

	if (tegra->domain) {
		err = iommu_attach_device(tegra->domain, client->dev);
		if (err < 0) {
			dev_err(client->dev, "failed to attach to domain: %d\n",
				err);
			return err;
		}

		nvjpg->domain = tegra->domain;
	}

	nvjpg->channel = host1x_channel_request(client->dev);
	if (!nvjpg->channel) {
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
	host1x_channel_free(nvjpg->channel);
error_iommu_detach_device:
	if (nvjpg->domain) {
		iommu_detach_device(nvjpg->domain, nvjpg->dev);
		nvjpg->domain = NULL;
	}
	return err;
}

static int nvjpg_exit(struct host1x_client *client)
{
	struct tegra_drm_client *drm = host1x_to_drm_client(client);
	struct drm_device *dev = dev_get_drvdata(client->parent);
	struct nvjpg *nvjpg = to_nvjpg(drm);
	int err;

	err = tegra_drm_unregister_client(dev->dev_private, drm);

	host1x_syncpt_free(client->syncpts[0]);
	host1x_channel_free(nvjpg->channel);

	if (nvjpg->domain) {
		iommu_detach_device(nvjpg->domain, nvjpg->dev);
		nvjpg->domain = NULL;
	}

	return err;
}

static const struct host1x_client_ops nvjpg_client_ops = {
	.init = nvjpg_init,
	.exit = nvjpg_exit,
};

static int nvjpg_open_channel(struct tegra_drm_client *client,
			      struct tegra_drm_context *context)
{
	struct nvjpg *nvjpg = to_nvjpg(client);
	int err;

	pm_runtime_get_sync(nvjpg->dev);

	err = falcon_boot(&nvjpg->falcon, nvjpg->config->ucode_name);
	if (err)
		return err;

	context->channel = host1x_channel_get(nvjpg->channel);
	if (!context->channel)
		return -ENOMEM;

	return 0;
}

static void nvjpg_close_channel(struct tegra_drm_context *context)
{
	struct nvjpg *nvjpg = to_nvjpg(context->client);

	if (!context->channel)
		return;

	host1x_channel_put(context->channel);
	context->channel = NULL;

	pm_runtime_mark_last_busy(nvjpg->dev);
	pm_runtime_put_autosuspend(nvjpg->dev);
}

static int nvjpg_is_addr_reg(struct device *dev, u32 class, u32 offset)
{
	/**
	 * Falcon communicates addresses and commands through the same
	 * method/data registers
	 */

	return 0;
}

static const struct tegra_drm_client_ops nvjpg_ops = {
	.open_channel = nvjpg_open_channel,
	.close_channel = nvjpg_close_channel,
	.is_addr_reg = nvjpg_is_addr_reg,
	.reset = nvjpg_reset,
	.submit = tegra_drm_submit,
};

static void *nvjpg_falcon_alloc(struct falcon *falcon, size_t size,
				dma_addr_t *paddr)
{
	struct nvjpg *nvjpg = (struct nvjpg *)falcon;
	struct tegra_drm_client *drm = &nvjpg->client;
	struct drm_device *dev = dev_get_drvdata(drm->base.parent);
	struct tegra_bo *bo;

	bo = tegra_bo_create(dev, size, 0);
	bo->vaddr = vmap(bo->pages, bo->num_pages, VM_MAP,
			 pgprot_writecombine(PAGE_KERNEL));
	if (!bo->vaddr) {
		dev_err(nvjpg->dev, "dma memory allocation failed");
		return NULL;
	}

	*paddr = bo->paddr;

	return bo->vaddr;
}

static void nvjpg_falcon_free(struct falcon *falcon, size_t size,
			      dma_addr_t paddr, void *vaddr)
{
	struct nvjpg *nvjpg = (struct nvjpg *)falcon;
	DEFINE_DMA_ATTRS(attrs);

	dma_free_attrs(nvjpg->dev, size, vaddr, paddr, &attrs);
}

static const struct falcon_ops nvjpg_falcon_ops = {
	.alloc = nvjpg_falcon_alloc,
	.free = nvjpg_falcon_free,
};

static const struct nvjpg_config nvjpg_nvjpg_t210_config = {
	.ucode_name = "nvidia/tegra210/nvjpg.bin",
	.class_id = HOST1X_CLASS_NVJPG,
	.powergate_id = TEGRA_POWERGATE_NVJPG,
};

static const struct of_device_id nvjpg_match[] = {
	{ .compatible = "nvidia,tegra210-nvjpg",
		.data = &nvjpg_nvjpg_t210_config },
	{ },
};

static int nvjpg_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct nvjpg_config *nvjpg_config;
	struct device *dev = &pdev->dev;
	struct host1x_syncpt **syncpts;
	struct resource *regs;
	struct nvjpg *nvjpg;
	int err;

	if (!dev->of_node) {
		dev_err(&pdev->dev, "no dt node\n");
		return -EINVAL;
	}

	match = of_match_device(nvjpg_match, dev);
	if (match)
		nvjpg_config = (struct nvjpg_config *)match->data;
	else
		return -ENXIO;

	nvjpg = devm_kzalloc(dev, sizeof(*nvjpg), GFP_KERNEL);
	if (!nvjpg)
		return -ENOMEM;

	syncpts = devm_kzalloc(dev, sizeof(*syncpts), GFP_KERNEL);
	if (!syncpts)
		return -ENOMEM;

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	nvjpg->falcon.regs = devm_ioremap_resource(&pdev->dev, regs);
	if (IS_ERR(nvjpg->falcon.regs))
		return PTR_ERR(nvjpg->falcon.regs);

	nvjpg->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(nvjpg->clk)) {
		dev_err(&pdev->dev, "cannot get clock\n");
		return PTR_ERR(nvjpg->clk);
	}
	nvjpg->rst = devm_reset_control_get(&pdev->dev, "nvjpg");
	if (IS_ERR(nvjpg->rst)) {
		dev_err(&pdev->dev, "cannot get reset\n");
		return PTR_ERR(nvjpg->rst);
	}

	nvjpg->falcon.dev = dev;
	nvjpg->falcon.ops = &nvjpg_falcon_ops;

	INIT_LIST_HEAD(&nvjpg->client.base.list);
	nvjpg->client.base.ops = &nvjpg_client_ops;
	nvjpg->client.base.dev = dev;
	nvjpg->client.base.class = nvjpg_config->class_id;
	nvjpg->client.base.syncpts = syncpts;
	nvjpg->client.base.num_syncpts = 1;
	nvjpg->dev = dev;
	nvjpg->config = nvjpg_config;

	INIT_LIST_HEAD(&nvjpg->client.list);
	nvjpg->client.ops = &nvjpg_ops;

	err = falcon_init(&nvjpg->falcon);
	if (err) {
		dev_err(dev, "failed initializing falcon helper\n");
		return err;
	}

	platform_set_drvdata(pdev, nvjpg);

	err = nvjpg_power_on(&pdev->dev);
	if (err) {
		dev_err(dev, "cannot turn on the device\n");
		goto error_falcon_exit;
	}

	err = host1x_client_register(&nvjpg->client.base);
	if (err < 0) {
		dev_err(dev, "failed to register host1x client: %d\n", err);
		goto error_nvjpg_power_off;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_set_autosuspend_delay(dev, 5000);

	dev_info(&pdev->dev, "initialized");

	return 0;

error_nvjpg_power_off:
	nvjpg_power_off(&pdev->dev);
error_falcon_exit:
	falcon_exit(&nvjpg->falcon);
	return err;
}

static int nvjpg_remove(struct platform_device *pdev)
{
	struct nvjpg *nvjpg = platform_get_drvdata(pdev);
	int err;

	err = host1x_client_unregister(&nvjpg->client.base);
	if (err < 0)
		dev_err(&pdev->dev, "failed to unregister host1x client: %d\n",
			err);

	nvjpg_power_off(&pdev->dev);

	falcon_exit(&nvjpg->falcon);

	return err;
}

static int __maybe_unused nvjpg_runtime_suspend(struct device *dev)
{
	dev_info(dev, "%s\n", __func__);
	return nvjpg_power_off(dev);
}

static int __maybe_unused nvjpg_runtime_resume(struct device *dev)
{
	dev_info(dev, "%s\n", __func__);
	return nvjpg_power_on(dev);
}

static int __maybe_unused nvjpg_suspend(struct device *dev)
{
	dev_info(dev, "%s\n", __func__);

	if (pm_runtime_status_suspended(dev))
		return 0;

	return nvjpg_power_off(dev);
}

static int __maybe_unused nvjpg_resume(struct device *dev)
{
	struct nvjpg *nvjpg = dev_get_drvdata(dev);
	int err;

	dev_info(dev, "%s\n", __func__);

	if (pm_runtime_status_suspended(dev))
		return 0;

	err = nvjpg_power_on(dev);
	if (err) {
		dev_err(dev, "failed to power on during resume\n");
		return err;
	}

	return falcon_boot(&nvjpg->falcon, nvjpg->config->ucode_name);
}

static const struct dev_pm_ops nvjpg_pm_ops = {
	SET_RUNTIME_PM_OPS(nvjpg_runtime_suspend, nvjpg_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(nvjpg_suspend, nvjpg_resume)
};

struct platform_driver tegra_nvjpg_driver = {
	.driver = {
		.name = "tegra-nvjpg",
		.of_match_table = nvjpg_match,
		.pm = &nvjpg_pm_ops,
	},
	.probe = nvjpg_probe,
	.remove = nvjpg_remove,
};
