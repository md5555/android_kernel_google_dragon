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
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/clk.h>
#include <linux/host1x.h>
#include <linux/module.h>
#include <linux/ote_protocol.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <soc/tegra/pmc.h>
#include <linux/iommu.h>

#include "drm.h"
#include "falcon.h"

#define NVDEC_MAX_CLK_RATE	716800000

#define MC_SECURITY_CARVEOUT1_BOM_LO		0xc0c
#define MC_SECURITY_CARVEOUT1_BOM_HI		0xc10
#define MC_SECURITY_CARVEOUT1_SIZE_128KB	0xc14
#define WPR_SIZE (128 * 1024)

struct nvdec_bl_shared_data {
	u32 ls_fw_start_addr;
	u32 ls_fw_size;
	u32 wpr_addr;
	u32 wpr_size;
};

struct nvdec_config {
	const char *ucode_name;
	const char *ucode_name_bl;
	const char *ucode_name_ls;
	u32 class_id;
	int powergate_id;
	const struct host1x_client_clock *clocks;
};

struct nvdec {
	struct falcon falcon_bl;
	struct falcon falcon_ls;

	struct tegra_drm_client client;

	struct host1x_channel *channel;
	struct device *dev;
	struct clk *clk;
	struct clk *cbus_clk;
	struct clk *emc_clk;
	struct clk *fuse;
	struct clk *kfuse;
	struct reset_control *rst;

	struct iommu_domain *domain;

	void __iomem *mc_base;

	/* Platform configuration */
	struct nvdec_config *config;
	struct mutex lock;
	/* Flag to indicate both the ucodes are setup correctly */
	bool ucode_setup;
};

static inline struct nvdec *to_nvdec(struct tegra_drm_client *client)
{
	return container_of(client, struct nvdec, client);
}

static int nvdec_power_off(struct device *dev)
{
	struct nvdec *nvdec = dev_get_drvdata(dev);
	int err;

	err = falcon_power_off(&nvdec->falcon_bl);
	if (err)
		return err;

	if (IS_ENABLED(CONFIG_NVDEC_BOOTLOADER)) {
		err = falcon_power_off(&nvdec->falcon_ls);
		if (err)
			return err;
	}

	err = reset_control_assert(nvdec->rst);
	if (err)
		return err;

	clk_disable_unprepare(nvdec->clk);
	clk_disable_unprepare(nvdec->kfuse);
	clk_disable_unprepare(nvdec->fuse);
	err = tegra_pmc_powergate(nvdec->config->powergate_id);
	if (err)
		return err;

	host1x_module_disable_clocks(&nvdec->client.base);

	return err;
}

static int nvdec_power_on(struct device *dev)
{
	struct nvdec *nvdec = dev_get_drvdata(dev);
	int err;

	err = falcon_power_on(&nvdec->falcon_bl);
	if (err)
		return err;

	if (IS_ENABLED(CONFIG_NVDEC_BOOTLOADER)) {
		err = falcon_power_on(&nvdec->falcon_ls);
		if (err)
			goto err_falcon_power_on;
	}

	err = host1x_module_enable_clocks(&nvdec->client.base);
	if (err)
		goto err_host1x_clk;

	err = tegra_pmc_unpowergate(nvdec->config->powergate_id);
	if (err)
		goto err_unpowergate;

	err = clk_prepare_enable(nvdec->clk);
	if (err)
		goto err_nvdec_clk;

	err = clk_prepare_enable(nvdec->fuse);
	if (err)
		goto err_nvdec_fuse_clk;

	err = clk_prepare_enable(nvdec->kfuse);
	if (err)
		goto err_nvdec_kfuse_clk;

	if (IS_ENABLED(CONFIG_TRUSTED_LITTLE_KERNEL))
		te_restore_keyslots();

	return 0;

err_nvdec_kfuse_clk:
	clk_disable_unprepare(nvdec->fuse);
err_nvdec_fuse_clk:
	clk_disable_unprepare(nvdec->clk);
err_nvdec_clk:
	tegra_pmc_powergate(nvdec->config->powergate_id);
err_unpowergate:
	host1x_module_disable_clocks(&nvdec->client.base);
err_host1x_clk:
	if (IS_ENABLED(CONFIG_NVDEC_BOOTLOADER))
		falcon_power_off(&nvdec->falcon_ls);
err_falcon_power_on:
	falcon_power_off(&nvdec->falcon_bl);
	return err;
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

	nvdec->channel = host1x_channel_request(client);
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

	mutex_init(&nvdec->lock);
	nvdec->ucode_setup = false;

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
		case DRM_TEGRA_REQ_TYPE_CLK_HZ:
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
		case DRM_TEGRA_REQ_TYPE_CLK_HZ:
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

static void nvdec_wpr_setup(struct nvdec *nvdec)
{
	size_t fb_data_offset;
	u32 wpr_addr_lo, wpr_addr_hi;
	struct nvdec_bl_shared_data shared_data;

	fb_data_offset = (nvdec->falcon_bl.os.bin_data_offset +
			  nvdec->falcon_bl.os.data_offset) / sizeof(u32);

	shared_data.ls_fw_start_addr = nvdec->falcon_ls.ucode_paddr >> 8;
	shared_data.ls_fw_size = nvdec->falcon_ls.ucode_size;

	wpr_addr_lo = readl(nvdec->mc_base + MC_SECURITY_CARVEOUT1_BOM_LO);
	wpr_addr_hi = readl(nvdec->mc_base + MC_SECURITY_CARVEOUT1_BOM_HI);
	shared_data.wpr_addr = ((wpr_addr_hi << 24) & 0xff000000)
		| ((wpr_addr_lo  >> 8) & 0xffffff);

	shared_data.wpr_size = readl(nvdec->mc_base +
				     MC_SECURITY_CARVEOUT1_SIZE_128KB);
	shared_data.wpr_size *= WPR_SIZE;

	memcpy(nvdec->falcon_bl.ucode_vaddr + fb_data_offset,
	       &shared_data, sizeof(shared_data));
}

static int nvdec_read_ucode(struct nvdec *nvdec)
{
	int err = 0;

	mutex_lock(&nvdec->lock);
	if (nvdec->ucode_setup)
		goto done;

	if (IS_ENABLED(CONFIG_NVDEC_BOOTLOADER)) {
		err = falcon_read_ucode(&nvdec->falcon_bl,
					nvdec->config->ucode_name_bl);
		if (err)
			goto done;

		err = falcon_read_ucode(&nvdec->falcon_ls,
					nvdec->config->ucode_name_ls);
		if (err)
			goto done;

		nvdec_wpr_setup(nvdec);
	} else {
		err = falcon_read_ucode(&nvdec->falcon_bl,
					nvdec->config->ucode_name);
		if (err)
			goto done;
	}
	nvdec->ucode_setup = true;

done:
	mutex_unlock(&nvdec->lock);
	return err;
}

static int nvdec_boot(struct nvdec *nvdec)
{
	int err;
	int retry_count = 3;

	err = falcon_boot(&nvdec->falcon_bl, NULL);
	if (!err)
		return err;

	/* Something wrong, attempt a poweroff-poweron cycle */
	dev_info(nvdec->dev, "falcon_boot failed, retrying ");
	while (retry_count--) {
		err = nvdec_power_off(nvdec->dev);
		if (err) {
			dev_err(nvdec->dev, "failed to power off during nvdec_boot");
			continue;
		}
		err = nvdec_power_on(nvdec->dev);
		if (err) {
			dev_err(nvdec->dev, "failed to power on during nvdec_boot");
			continue;
		}
		err = nvdec_read_ucode(nvdec);
		if (err) {
			dev_err(nvdec->dev, "failed to read ucode during nvdec_boot");
			continue;
		}
		err = falcon_boot(&nvdec->falcon_bl, NULL);
		if (!err)
			return err;
	}
	return err;
}

static int nvdec_open_channel(struct tegra_drm_client *client,
			      struct tegra_drm_context *context)
{
	struct nvdec *nvdec = to_nvdec(client);
	int err;

	err = host1x_module_busy(&client->base);
	if (err)
		return err;

	err = nvdec_read_ucode(nvdec);
	if (err)
		goto done;

	err = nvdec_boot(nvdec);
	if (err)
		goto done;

	context->channel = host1x_channel_get(nvdec->channel, &context->user);
	if (!context->channel)
		err = -ENOMEM;

done:
	host1x_module_idle(&client->base);

	return err;
}

static void nvdec_close_channel(struct tegra_drm_context *context)
{
	if (!context->channel)
		return;

	host1x_channel_put(context->channel, &context->user);
	context->channel = NULL;
}

static int nvdec_is_addr_reg(struct device *dev, u32 class, u32 offset)
{
	/**
	 * Falcon communicates addresses and commands through the same
	 * method/data registers
	 */

	return 0;
}

static void nvdec_reset(struct device *dev)
{
	int err;
	struct nvdec *nvdec = dev_get_drvdata(dev);

	err = nvdec_power_off(dev);
	if (err) {
		dev_err(dev, "failed to power off during reset\n");
		return;
	}

	err = nvdec_power_on(dev);
	if (err) {
		dev_err(dev, "failed to power on during reset\n");
		return;
	}

	err = nvdec_read_ucode(nvdec);
	if (err) {
		dev_err(dev, "failed to read ucode during reset\n");
		return;
	}

	nvdec_boot(nvdec);
}

static const struct tegra_drm_client_ops nvdec_ops = {
	.open_channel = nvdec_open_channel,
	.close_channel = nvdec_close_channel,
	.is_addr_reg = nvdec_is_addr_reg,
	.reset = nvdec_reset,
	.submit = tegra_drm_submit,
};

static void *__nvdec_falcon_alloc(struct nvdec *nvdec, size_t size,
				  dma_addr_t *paddr)
{
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

static void *nvdec_falcon_bl_alloc(struct falcon *falcon, size_t size,
				   dma_addr_t *paddr)
{
	struct nvdec *nvdec = container_of(falcon, struct nvdec, falcon_bl);

	return __nvdec_falcon_alloc(nvdec, size, paddr);
}

static void nvdec_falcon_bl_free(struct falcon *falcon, size_t size,
				 dma_addr_t paddr, void *vaddr)
{
	struct nvdec *nvdec = container_of(falcon, struct nvdec, falcon_bl);
	DEFINE_DMA_ATTRS(attrs);

	dma_free_attrs(nvdec->dev, size, vaddr, paddr, &attrs);
}

static void *nvdec_falcon_ls_alloc(struct falcon *falcon, size_t size,
				   dma_addr_t *paddr)
{
	struct nvdec *nvdec = container_of(falcon, struct nvdec, falcon_ls);

	return __nvdec_falcon_alloc(nvdec, size, paddr);
}

static void nvdec_falcon_ls_free(struct falcon *falcon, size_t size,
				 dma_addr_t paddr, void *vaddr)
{
	struct nvdec *nvdec = container_of(falcon, struct nvdec, falcon_ls);
	DEFINE_DMA_ATTRS(attrs);

	dma_free_attrs(nvdec->dev, size, vaddr, paddr, &attrs);
}

static const struct falcon_ops nvdec_falcon_bl_ops = {
	.alloc = nvdec_falcon_bl_alloc,
	.free = nvdec_falcon_bl_free,
};

static const struct falcon_ops nvdec_falcon_ls_ops = {
	.alloc = nvdec_falcon_ls_alloc,
	.free = nvdec_falcon_ls_free,
};

static const struct host1x_client_clock nvdec_t210_clocks[] = {
	{
		.clk_name = "nvdec_cbus",
		.default_rate = 192000000,
		.valid_constraint_types =
			BIT(HOST1X_USER_CONSTRAINT_TYPE_HZ),
	}, {
		.clk_name = "emc",
		.default_rate = 102000000,
		.valid_constraint_types =
			BIT(HOST1X_USER_CONSTRAINT_TYPE_HZ) |
			BIT(HOST1X_USER_CONSTRAINT_TYPE_BW_KBPS),
	},
};

static const struct nvdec_config nvdec_t210_config = {
	.ucode_name = "nvidia/tegra210/nvdec_ns.bin",
	.ucode_name_bl = "nvidia/tegra210/nvdec_bl_prod.bin",
	.ucode_name_ls = "nvidia/tegra210/nvdec_prod.bin",
	.class_id = HOST1X_CLASS_NVDEC,
	.powergate_id = TEGRA_POWERGATE_NVDEC,
	.clocks = nvdec_t210_clocks,
};

static struct of_device_id nvdec_match[] = {
	{ .compatible = "nvidia,tegra210-nvdec",
		.data = &nvdec_t210_config },
	{ },
};

static const struct of_device_id mc_match[] = {
	{ .compatible = "nvidia,tegra210-mc" },
	{},
};

static int nvdec_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct nvdec_config *nvdec_config;
	struct device *dev = &pdev->dev;
	struct host1x_syncpt **syncpts;
	struct resource *regs;
	struct device_node *node;
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
	nvdec->falcon_bl.regs = devm_ioremap_resource(&pdev->dev, regs);
	if (IS_ERR(nvdec->falcon_bl.regs))
		return PTR_ERR(nvdec->falcon_bl.regs);

	node = of_find_matching_node(NULL, mc_match);
	if (!node)
		return -ENODEV;

	nvdec->mc_base = of_iomap(node, 0);
	if (!nvdec->mc_base)
		return -ENOMEM;

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

	nvdec->kfuse = devm_clk_get(dev, "kfuse");
	if (IS_ERR(nvdec->kfuse)) {
		dev_err(dev, "cannot get kfuse clock\n");
		return PTR_ERR(nvdec->kfuse);
	}

	nvdec->fuse = devm_clk_get(dev, "fuse");
	if (IS_ERR(nvdec->fuse)) {
		dev_err(dev, "cannot get fuse clock\n");
		return PTR_ERR(nvdec->fuse);
	}

	nvdec->rst = devm_reset_control_get(&pdev->dev, "nvdec");
	if (IS_ERR(nvdec->rst)) {
		dev_err(dev, "cannot get reset\n");
		return PTR_ERR(nvdec->rst);
	}

	nvdec->falcon_bl.dev = dev;
	nvdec->falcon_bl.ops = &nvdec_falcon_bl_ops;

	if (IS_ENABLED(CONFIG_NVDEC_BOOTLOADER)) {
		nvdec->falcon_ls.dev = dev;
		nvdec->falcon_ls.ops = &nvdec_falcon_ls_ops;
	}

	INIT_LIST_HEAD(&nvdec->client.base.list);
	nvdec->client.base.ops = &nvdec_client_ops;
	nvdec->client.base.dev = dev;
	nvdec->client.base.class = nvdec_config->class_id;
	nvdec->client.base.syncpts = syncpts;
	nvdec->client.base.num_syncpts = 1;
	memcpy(&nvdec->client.base.clocks, nvdec_config->clocks,
	       sizeof(nvdec_t210_clocks));
	nvdec->dev = dev;
	nvdec->config = nvdec_config;

	INIT_LIST_HEAD(&nvdec->client.list);
	nvdec->client.ops = &nvdec_ops;

	err = falcon_init(&nvdec->falcon_bl);
	if (err) {
		dev_err(dev, "failed initializing falcon BL helper\n");
		return err;
	}

	if (IS_ENABLED(CONFIG_NVDEC_BOOTLOADER)) {
		err = falcon_init(&nvdec->falcon_ls);
		if (err) {
			dev_err(dev, "failed initializing falcon ls helper\n");
			goto error_falcon_exit_bl;
		}
	}

	platform_set_drvdata(pdev, nvdec);

	err = host1x_client_register(&nvdec->client.base);
	if (err < 0) {
		dev_err(dev, "failed to register host1x client: %d\n", err);
		goto error_falcon_exit;
	}

	err = nvdec_power_on(&pdev->dev);
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
	host1x_client_unregister(&nvdec->client.base);
error_falcon_exit:
	if (IS_ENABLED(CONFIG_NVDEC_BOOTLOADER))
		falcon_exit(&nvdec->falcon_ls);
error_falcon_exit_bl:
	falcon_exit(&nvdec->falcon_bl);

	return err;
}

static int nvdec_remove(struct platform_device *pdev)
{
	struct nvdec *nvdec = platform_get_drvdata(pdev);
	int err;

	nvdec_power_off(&pdev->dev);

	err = host1x_client_unregister(&nvdec->client.base);
	if (err < 0)
		dev_err(&pdev->dev, "failed to unregister host1x client: %d\n",
			err);

	if (IS_ENABLED(CONFIG_NVDEC_BOOTLOADER))
		falcon_exit(&nvdec->falcon_ls);

	falcon_exit(&nvdec->falcon_bl);

	nvdec->ucode_setup = false;

	return err;
}

static int __maybe_unused nvdec_runtime_suspend(struct device *dev)
{
	dev_info(dev, "%s\n", __func__);
	return nvdec_power_off(dev);
}

static int __maybe_unused nvdec_runtime_resume(struct device *dev)
{
	struct nvdec *nvdec = dev_get_drvdata(dev);
	int err;

	dev_info(dev, "%s\n", __func__);

	err = nvdec_power_on(dev);
	if (err)
		return err;

	err = nvdec_read_ucode(nvdec);
	if (err)
		return err;

	return nvdec_boot(nvdec);
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

	err = nvdec_read_ucode(nvdec);
	if (err)
		return err;

	return nvdec_boot(nvdec);
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
