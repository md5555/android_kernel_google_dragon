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

#define NVDEC_MAX_CLK_RATE	716800000

struct nvdec_config {
	const char *ucode_name;
	u32 class_id;
	int powergate_id;
};

struct nvdec {
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
	struct nvdec_config *config;
};

static inline struct nvdec *to_nvdec(struct tegra_drm_client *client)
{
	return container_of(client, struct nvdec, client);
}

static int nvdec_power_off(struct device *dev)
{
	struct nvdec *nvdec = dev_get_drvdata(dev);
	int err;

	err = falcon_power_off(&nvdec->falcon);
	if (err)
		return err;

	err = reset_control_assert(nvdec->rst);
	if (err)
		return err;

	clk_disable_unprepare(nvdec->clk);
	clk_disable_unprepare(nvdec->cbus_clk);
	err = tegra_powergate_power_off(nvdec->config->powergate_id);
	if (err)
		return err;

	clk_disable_unprepare(nvdec->emc_clk);
	err = tegra_powergate_power_off(TEGRA_POWERGATE_NVJPG);
	return err;
}

static int nvdec_power_on(struct device *dev)
{
	struct nvdec *nvdec = dev_get_drvdata(dev);
	int err;

	err = falcon_power_on(&nvdec->falcon);
	if (err)
		return err;

	err = clk_prepare_enable(nvdec->emc_clk);
	if (err)
		goto err_emc_clk;

	/* NVDEC needs NVJPG to be powered up first */
	err = tegra_powergate_sequence_power_up(TEGRA_POWERGATE_NVJPG);
	if (err)
		goto err_powergate_nvjpg;

	err = tegra_powergate_sequence_power_up(nvdec->config->powergate_id);
	if (err)
		goto err_powergate_nvdec;

	err = clk_prepare_enable(nvdec->clk);
	if (err)
		goto err_nvdec_clk;

	err = clk_prepare_enable(nvdec->cbus_clk);
	if (err)
		goto err_nvdec_cbus_clk;
	return 0;

err_nvdec_cbus_clk:
	clk_disable_unprepare(nvdec->clk);
err_nvdec_clk:
	tegra_powergate_power_off(nvdec->config->powergate_id);
err_powergate_nvdec:
	tegra_powergate_power_off(TEGRA_POWERGATE_NVJPG);
err_powergate_nvjpg:
	clk_disable_unprepare(nvdec->emc_clk);
err_emc_clk:
	falcon_power_off(&nvdec->falcon);
	return err;
}

static void nvdec_reset(struct device *dev)
{
	struct nvdec *nvdec = dev_get_drvdata(dev);

	nvdec_power_off(dev);
	nvdec_power_on(dev);
	falcon_boot(&nvdec->falcon, nvdec->config->ucode_name);
}

static int nvdec_init(struct host1x_client *client)
{
	struct tegra_drm_client *drm = host1x_to_drm_client(client);
	struct drm_device *dev = dev_get_drvdata(client->parent);
	struct tegra_drm *tegra = dev->dev_private;
	struct nvdec *nvdec = to_nvdec(drm);
	int err;

	if (tegra->domain) {
		err = iommu_attach_device(tegra->domain, client->dev);
		if (err < 0) {
			dev_err(client->dev, "failed to attach to domain: %d\n",
				err);
			return err;
		}

		nvdec->domain = tegra->domain;
	}

	nvdec->channel = host1x_channel_request(client->dev);
	if (!nvdec->channel) {
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
	host1x_channel_free(nvdec->channel);
error_iommu_detach_device:
	if (nvdec->domain) {
		iommu_detach_device(nvdec->domain, nvdec->dev);
		nvdec->domain = NULL;
	}
	return err;
}

static int nvdec_exit(struct host1x_client *client)
{
	struct tegra_drm_client *drm = host1x_to_drm_client(client);
	struct drm_device *dev = dev_get_drvdata(client->parent);
	struct nvdec *nvdec = to_nvdec(drm);
	int err;

	err = tegra_drm_unregister_client(dev->dev_private, drm);

	host1x_syncpt_free(client->syncpts[0]);
	host1x_channel_free(nvdec->channel);

	if (nvdec->domain) {
		iommu_detach_device(nvdec->domain, nvdec->dev);
		nvdec->domain = NULL;
	}

	return err;
}

static int nvdec_get_clk_rate(struct host1x_client *client, u64 *data,
					u32 type)
{
	struct tegra_drm_client *drm = host1x_to_drm_client(client);
	struct nvdec *nvdec = to_nvdec(drm);

	switch (type) {
		case DRM_TEGRA_REQ_TYPE_CLK_KHZ:
			*data = clk_get_rate(nvdec->cbus_clk);
			break;
		case DRM_TEGRA_REQ_TYPE_BW_KBPS:
			*data = clk_get_rate(nvdec->emc_clk);
			break;
		default:
			dev_err(nvdec->dev, "Unknown Clock request type\n");
			return -EINVAL;
	}
	return 0;
}

static int nvdec_set_clk_rate(struct host1x_client *client, u64 data,
					u32 type)
{
	struct tegra_drm_client *drm = host1x_to_drm_client(client);
	struct nvdec *nvdec = to_nvdec(drm);

	switch (type) {
		case DRM_TEGRA_REQ_TYPE_CLK_KHZ:
			if (data > NVDEC_MAX_CLK_RATE)
				data = NVDEC_MAX_CLK_RATE;
			return clk_set_rate(nvdec->cbus_clk, data);
		case DRM_TEGRA_REQ_TYPE_BW_KBPS:
			return clk_set_rate(nvdec->emc_clk, data);
		default:
			dev_err(nvdec->dev, "Unknown Clock request type\n");
			return -EINVAL;
	}
}

static const struct host1x_client_ops nvdec_client_ops = {
	.init = nvdec_init,
	.exit = nvdec_exit,
	.get_clk_rate = nvdec_get_clk_rate,
	.set_clk_rate = nvdec_set_clk_rate,
};

static int nvdec_open_channel(struct tegra_drm_client *client,
			      struct tegra_drm_context *context)
{
	struct nvdec *nvdec = to_nvdec(client);
	int err;

	pm_runtime_get_sync(nvdec->dev);

	err = falcon_boot(&nvdec->falcon, nvdec->config->ucode_name);
	if (err)
		return err;

	context->channel = host1x_channel_get(nvdec->channel);
	if (!context->channel)
		return -ENOMEM;

	return 0;
}

static void nvdec_close_channel(struct tegra_drm_context *context)
{
	struct nvdec *nvdec = to_nvdec(context->client);

	if (!context->channel)
		return;

	host1x_channel_put(context->channel);
	context->channel = NULL;

	pm_runtime_mark_last_busy(nvdec->dev);
	pm_runtime_put_autosuspend(nvdec->dev);
}

static int nvdec_is_addr_reg(struct device *dev, u32 class, u32 offset)
{
	/**
	 * Falcon communicates addresses and commands through the same
	 * method/data registers
	 */

	return 0;
}

static const struct tegra_drm_client_ops nvdec_ops = {
	.open_channel = nvdec_open_channel,
	.close_channel = nvdec_close_channel,
	.is_addr_reg = nvdec_is_addr_reg,
	.reset = nvdec_reset,
	.submit = tegra_drm_submit,
};

static void *nvdec_falcon_alloc(struct falcon *falcon, size_t size,
				dma_addr_t *paddr)
{
	struct nvdec *nvdec = (struct nvdec *)falcon;
	struct tegra_drm_client *drm = &nvdec->client;
	struct drm_device *dev = dev_get_drvdata(drm->base.parent);
	struct tegra_bo *bo;

	bo = tegra_bo_create(dev, size, 0);
	bo->vaddr = vmap(bo->pages, bo->num_pages, VM_MAP,
			 pgprot_writecombine(PAGE_KERNEL));
	if (!bo->vaddr) {
		dev_err(nvdec->dev, "dma memory allocation failed");
		return NULL;
	}

	*paddr = bo->paddr;

	return bo->vaddr;
}

static void nvdec_falcon_free(struct falcon *falcon, size_t size,
			      dma_addr_t paddr, void *vaddr)
{
	struct nvdec *nvdec = (struct nvdec *)falcon;
	DEFINE_DMA_ATTRS(attrs);

	dma_free_attrs(nvdec->dev, size, vaddr, paddr, &attrs);
}

static const struct falcon_ops nvdec_falcon_ops = {
	.alloc = nvdec_falcon_alloc,
	.free = nvdec_falcon_free,
};

static const struct nvdec_config nvdec_t210_config = {
	.ucode_name = "nvidia/tegra210/nvdec_ns.bin",
	.class_id = HOST1X_CLASS_NVDEC,
	.powergate_id = TEGRA_POWERGATE_NVDEC,
};

static struct of_device_id nvdec_match[] = {
	{ .compatible = "nvidia,tegra210-nvdec",
		.data = &nvdec_t210_config },
	{ },
};

static int nvdec_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct nvdec_config *nvdec_config;
	struct device *dev = &pdev->dev;
	struct host1x_syncpt **syncpts;
	struct resource *regs;
	struct nvdec *nvdec;
	int err;

	if (!dev->of_node) {
		dev_err(&pdev->dev, "no dt node\n");
		return -EINVAL;
	}

	match = of_match_device(nvdec_match, dev);
	if (match)
		nvdec_config = (struct nvdec_config *)match->data;
	else
		return -ENXIO;

	nvdec = devm_kzalloc(dev, sizeof(*nvdec), GFP_KERNEL);
	if (!nvdec)
		return -ENOMEM;

	syncpts = devm_kzalloc(dev, sizeof(*syncpts), GFP_KERNEL);
	if (!syncpts)
		return -ENOMEM;

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	nvdec->falcon.regs = devm_ioremap_resource(&pdev->dev, regs);
	if (IS_ERR(nvdec->falcon.regs))
		return PTR_ERR(nvdec->falcon.regs);

	nvdec->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(nvdec->clk)) {
		dev_err(dev, "cannot get nvdec clock\n");
		return PTR_ERR(nvdec->clk);
	}
	nvdec->cbus_clk = devm_clk_get(dev, "nvdec_cbus");
	if (IS_ERR(nvdec->cbus_clk)) {
		dev_err(dev, "cannot get nvdec cbus clock\n");
		return PTR_ERR(nvdec->cbus_clk);
	}
	nvdec->emc_clk = devm_clk_get(dev, "emc");
	if (IS_ERR(nvdec->emc_clk)) {
		dev_err(dev, "cannot get emc clock\n");
		return PTR_ERR(nvdec->emc_clk);
	}
	nvdec->rst = devm_reset_control_get(&pdev->dev, "nvdec");
	if (IS_ERR(nvdec->rst)) {
		dev_err(dev, "cannot get reset\n");
		return PTR_ERR(nvdec->rst);
	}

	nvdec->falcon.dev = dev;
	nvdec->falcon.ops = &nvdec_falcon_ops;

	INIT_LIST_HEAD(&nvdec->client.base.list);
	nvdec->client.base.ops = &nvdec_client_ops;
	nvdec->client.base.dev = dev;
	nvdec->client.base.class = nvdec_config->class_id;
	nvdec->client.base.syncpts = syncpts;
	nvdec->client.base.num_syncpts = 1;
	nvdec->dev = dev;
	nvdec->config = nvdec_config;

	INIT_LIST_HEAD(&nvdec->client.list);
	nvdec->client.ops = &nvdec_ops;

	err = falcon_init(&nvdec->falcon);
	if (err) {
		dev_err(dev, "failed initializing falcon helper\n");
		return err;
	}

	platform_set_drvdata(pdev, nvdec);

	err = nvdec_power_on(&pdev->dev);
	if (err) {
		dev_err(dev, "cannot turn on the device\n");
		goto error_falcon_exit;
	}

	err = host1x_client_register(&nvdec->client.base);
	if (err < 0) {
		dev_err(dev, "failed to register host1x client: %d\n", err);
		goto error_nvdec_power_off;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_set_autosuspend_delay(dev, 5000);

	dev_info(&pdev->dev, "initialized");

	return 0;

error_nvdec_power_off:
	nvdec_power_off(&pdev->dev);
error_falcon_exit:
	falcon_exit(&nvdec->falcon);
	return err;
}

static int nvdec_remove(struct platform_device *pdev)
{
	struct nvdec *nvdec = platform_get_drvdata(pdev);
	int err;

	err = host1x_client_unregister(&nvdec->client.base);
	if (err < 0)
		dev_err(&pdev->dev, "failed to unregister host1x client: %d\n",
			err);

	nvdec_power_off(&pdev->dev);

	falcon_exit(&nvdec->falcon);

	return err;
}

static int __maybe_unused nvdec_runtime_suspend(struct device *dev)
{
	dev_info(dev, "%s\n", __func__);
	return nvdec_power_off(dev);
}

static int __maybe_unused nvdec_runtime_resume(struct device *dev)
{
	dev_info(dev, "%s\n", __func__);
	return nvdec_power_on(dev);
}

static int __maybe_unused nvdec_suspend(struct device *dev)
{
	dev_info(dev, "%s\n", __func__);

	if (pm_runtime_status_suspended(dev))
		return 0;

	return nvdec_power_off(dev);
}

static int __maybe_unused nvdec_resume(struct device *dev)
{
	struct nvdec *nvdec = dev_get_drvdata(dev);
	int err;

	dev_info(dev, "%s\n", __func__);

	if (pm_runtime_status_suspended(dev))
		return 0;

	err = nvdec_power_on(dev);
	if (err) {
		dev_err(dev, "failed to power on during resume\n");
		return err;
	}

	return falcon_boot(&nvdec->falcon, nvdec->config->ucode_name);
}

static const struct dev_pm_ops nvdec_pm_ops = {
	SET_RUNTIME_PM_OPS(nvdec_runtime_suspend, nvdec_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(nvdec_suspend, nvdec_resume)
};

struct platform_driver tegra_nvdec_driver = {
	.driver = {
		.name = "tegra-nvdec",
		.of_match_table = nvdec_match,
		.pm = &nvdec_pm_ops,
	},
	.probe = nvdec_probe,
	.remove = nvdec_remove,
};
