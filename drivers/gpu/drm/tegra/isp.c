/*
 * Copyright (c) 2012-2015, NVIDIA Corporation.
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
#include <linux/iommu.h>
#include <soc/tegra/pmc.h>

#include "drm.h"

#define ISP_NUM_SYNCPTS 4

struct isp_config {
	u32 class_id;
	int powergate_id;
};

struct isp {
	struct tegra_drm_client client;

	void __iomem *regs;

	struct host1x_channel *channel;
	struct device *dev;
	struct clk *clk;
	struct reset_control *rst;

	struct iommu_domain *domain;

	/* Platform configuration */
	const struct isp_config *config;
};

static inline struct isp *to_isp(struct tegra_drm_client *client)
{
	return container_of(client, struct isp, client);
}

static int isp_power_off(struct device *dev)
{
	struct isp *isp = dev_get_drvdata(dev);
	int err;

	err = reset_control_assert(isp->rst);
	if (err)
		return err;

	clk_disable_unprepare(isp->clk);

	return tegra_pmc_powergate(isp->config->powergate_id);
}

static int isp_power_on(struct device *dev)
{
	int ret;
	struct isp *isp = dev_get_drvdata(dev);

	ret = tegra_pmc_unpowergate(isp->config->powergate_id);
	if (ret)
		return ret;

	ret = clk_prepare_enable(isp->clk);
	if (ret < 0) {
		dev_err(dev, "failed to enable clock\n");
		tegra_pmc_powergate(isp->config->powergate_id);
	}

	return ret;
}

static void isp_reset(struct device *dev)
{
	isp_power_off(dev);
	isp_power_on(dev);
}

static int isp_init(struct host1x_client *client)
{
	struct tegra_drm_client *drm = host1x_to_drm_client(client);
	struct drm_device *dev = dev_get_drvdata(client->parent);
	struct tegra_drm *tegra = dev->dev_private;
	unsigned long flags = HOST1X_SYNCPT_HAS_BASE;
	struct isp *isp = to_isp(drm);
	unsigned int i;
	int err;

	if (tegra->domain) {
		err = iommu_attach_device(tegra->domain, client->dev);
		if (err < 0) {
			dev_err(client->dev, "failed to attach to domain: %d\n",
				err);
			return err;
		}

		isp->domain = tegra->domain;
	}

	isp->channel = host1x_channel_request(client->dev);
	if (!isp->channel) {
		err = -ENOMEM;
		goto error_iommu_detach_device;
	}

	for (i = 0; i < ISP_NUM_SYNCPTS; i++) {
		client->syncpts[i] = host1x_syncpt_request(client->dev, flags);
		if (!client->syncpts[i]) {
			err = -ENOMEM;
			goto error_host1x_syncpt_free;
		}
	}

	err = tegra_drm_register_client(dev->dev_private, drm);
	if (err)
		goto error_host1x_syncpt_free;

	return 0;

error_host1x_syncpt_free:
	for (i = 0; i < ISP_NUM_SYNCPTS; i++)
		if (client->syncpts[i]) {
			host1x_syncpt_free(client->syncpts[i]);
			client->syncpts[i] = NULL;
		}
	host1x_channel_free(isp->channel);
error_iommu_detach_device:
	if (isp->domain) {
		iommu_detach_device(isp->domain, isp->dev);
		isp->domain = NULL;
	}
	return err;
}

static int isp_exit(struct host1x_client *client)
{
	struct tegra_drm_client *drm = host1x_to_drm_client(client);
	struct drm_device *dev = dev_get_drvdata(client->parent);
	struct isp *isp = to_isp(drm);
	unsigned int i;
	int err;

	err = tegra_drm_unregister_client(dev->dev_private, drm);

	for (i = 0; i < ISP_NUM_SYNCPTS; i++)
		if (client->syncpts[i]) {
			host1x_syncpt_free(client->syncpts[i]);
			client->syncpts[i] = NULL;
		}

	host1x_channel_free(isp->channel);

	if (isp->domain) {
		iommu_detach_device(isp->domain, isp->dev);
		isp->domain = NULL;
	}

	return 0;
}

static const struct host1x_client_ops isp_client_ops = {
	.init = isp_init,
	.exit = isp_exit,
};

static int isp_open_channel(struct tegra_drm_client *client,
			     struct tegra_drm_context *context)
{
	struct isp *isp = to_isp(client);

	pm_runtime_get_sync(isp->dev);

	context->channel = host1x_channel_get(isp->channel);
	if (!context->channel)
		return -ENOMEM;

	return 0;
}

static void isp_close_channel(struct tegra_drm_context *context)
{
	struct isp *isp = to_isp(context->client);

	if (!context->channel)
		return;

	host1x_channel_put(context->channel);
	context->channel = NULL;

	pm_runtime_mark_last_busy(isp->dev);
	pm_runtime_put_autosuspend(isp->dev);
}

static int isp_is_addr_reg(struct device *dev, u32 class, u32 offset)
{
	return 0;
}

static const struct tegra_drm_client_ops isp_ops = {
	.open_channel = isp_open_channel,
	.close_channel = isp_close_channel,
	.is_addr_reg = isp_is_addr_reg,
	.submit = tegra_drm_submit,
	.reset = isp_reset,
};

static const struct isp_config ispa_t210_config = {
	.class_id = HOST1X_CLASS_ISPA,
	.powergate_id = TEGRA_POWERGATE_VENC,
};

static const struct isp_config ispb_t210_config = {
	.class_id = HOST1X_CLASS_ISPB,
	.powergate_id = TEGRA_POWERGATE_VE2,
};

static const struct of_device_id ispa_match[] = {
	{ .compatible = "nvidia,tegra210-isp", .data = &ispa_t210_config },
	{ },
};

static const struct of_device_id ispb_match[] = {
	{ .compatible = "nvidia,tegra210-isp", .data = &ispb_t210_config },
	{ },
};

static int isp_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	const struct isp_config *isp_config;
	struct device *dev = &pdev->dev;
	struct host1x_syncpt **syncpts;
	struct resource *regs;
	struct isp *isp;
	int alias_id;
	int err;

	alias_id = of_alias_get_id(pdev->dev.of_node, "isp");
	if (alias_id == 0)
		match = of_match_node(ispa_match, pdev->dev.of_node);
	else if (alias_id == 1)
		match = of_match_node(ispb_match, pdev->dev.of_node);
	else {
		dev_err(&pdev->dev, "bad isp alias id %d\n", alias_id);
		return -EINVAL;
	}
	if (match)
		isp_config = match->data;
	else
		return -ENXIO;

	isp = devm_kzalloc(dev, sizeof(*isp), GFP_KERNEL);
	if (!isp)
		return -ENOMEM;

	syncpts = devm_kzalloc(dev, ISP_NUM_SYNCPTS * sizeof(*syncpts),
			       GFP_KERNEL);
	if (!syncpts)
		return -ENOMEM;

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	isp->regs = devm_ioremap_resource(&pdev->dev, regs);
	if (IS_ERR(isp->regs))
		return PTR_ERR(isp->regs);

	isp->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(isp->clk))
		return PTR_ERR(isp->clk);

	isp->rst = devm_reset_control_get(&pdev->dev, "isp");
	if (IS_ERR(isp->rst)) {
		dev_err(&pdev->dev, "cannot get reset\n");
		return PTR_ERR(isp->rst);
	}

	INIT_LIST_HEAD(&isp->client.base.list);
	isp->client.base.ops = &isp_client_ops;
	isp->client.base.dev = dev;
	isp->client.base.class = isp_config->class_id;
	isp->client.base.syncpts = syncpts;
	isp->client.base.num_syncpts = ISP_NUM_SYNCPTS;
	isp->dev = dev;
	isp->config = isp_config;

	INIT_LIST_HEAD(&isp->client.list);
	isp->client.ops = &isp_ops;

	platform_set_drvdata(pdev, isp);

	err = isp_power_on(&pdev->dev);
	if (err) {
		dev_err(dev, "cannot power on device\n");
		return err;
	}

	err = host1x_client_register(&isp->client.base);
	if (err < 0) {
		dev_err(dev, "failed to register host1x client: %d\n", err);
		goto error_isp_power_off;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_set_autosuspend_delay(dev, 5000);

	dev_info(&pdev->dev, "initialized");

	return 0;

error_isp_power_off:
	isp_power_off(&pdev->dev);
	return err;
}

static int isp_remove(struct platform_device *pdev)
{
	struct isp *isp = platform_get_drvdata(pdev);
	int err;

	err = host1x_client_unregister(&isp->client.base);
	if (err < 0)
		dev_err(&pdev->dev, "failed to unregister host1x client: %d\n",
			err);

	isp_power_off(&pdev->dev);

	return err;
}

static int __maybe_unused isp_runtime_suspend(struct device *dev)
{
	dev_info(dev, "%s\n", __func__);
	return isp_power_off(dev);
}

static int __maybe_unused isp_runtime_resume(struct device *dev)
{
	dev_info(dev, "%s\n", __func__);
	return isp_power_on(dev);
}

static const struct dev_pm_ops isp_pm_ops = {
	SET_RUNTIME_PM_OPS(isp_runtime_suspend, isp_runtime_resume, NULL)
};

struct platform_driver tegra_isp_driver = {
	.driver = {
		.name = "tegra-isp",
		.of_match_table = ispa_match,
		.pm = &isp_pm_ops,
	},
	.probe = isp_probe,
	.remove = isp_remove,
};
