/*
 * Copyright (c) 2013-2015, NVIDIA CORPORATION. All rights reserved.
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

#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/clk/tegra.h>
#include <linux/host1x.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/iommu.h>
#include <linux/regulator/consumer.h>
#include <linux/resource.h>
#include <soc/tegra/pmc.h>

#include "drm.h"
#include "vi.h"

#define VI_NUM_SYNCPTS	6
#define WATCHDOG_INT 0x20

struct vi_config {
	u32 class_id;
	int powergate_id;
};

struct vi {
	struct tegra_drm_client client;

	void __iomem *regs;

	struct host1x_channel *channel;
	struct device *dev;
	struct clk *clk;
	struct clk *plld_clk;
	struct clk *cilab;
	struct clk *cilcd;
	struct clk *cile;
	struct clk *csi;
	struct clk *csus;
	struct clk *sclk;

	struct reset_control *rst;
	struct regulator *reg;

	struct notifier_block slcg_notifier;

	struct iommu_domain *domain;

	int vi_irq;
	struct workqueue_struct *vi_workqueue;
	struct work_struct mfi_cb_work;

	/* Platform configuration */
	struct vi_config *config;
};

static void (*mfi_callback)(void *);
static void *mfi_callback_arg;
static DEFINE_MUTEX(vi_mfi_lock);

struct wd_reg {
	u32 err_status_addr;
	u32 wd_ctrl_addr;
};

static const struct wd_reg wd_reg_table[] = {
	{VI_CSI_0_ERROR_STATUS, VI_CSI_0_WD_CTRL},
	{VI_CSI_1_ERROR_STATUS, VI_CSI_1_WD_CTRL},
	{VI_CSI_2_ERROR_STATUS, VI_CSI_2_WD_CTRL},
	{VI_CSI_3_ERROR_STATUS, VI_CSI_3_WD_CTRL},
	{VI_CSI_4_ERROR_STATUS, VI_CSI_4_WD_CTRL},
	{VI_CSI_5_ERROR_STATUS, VI_CSI_5_WD_CTRL},
};

static const u32 mask_reg_table[] = {
	VI_CFG_INTERRUPT_MASK_0,
	VI_CSI_0_ERROR_INT_MASK_0,
	VI_CSI_1_ERROR_INT_MASK_0,
	VI_CSI_2_ERROR_INT_MASK_0,
	VI_CSI_3_ERROR_INT_MASK_0,
	VI_CSI_4_ERROR_INT_MASK_0,
	VI_CSI_5_ERROR_INT_MASK_0,
	VI_CSI_0_WD_CTRL,
	VI_CSI_1_WD_CTRL,
	VI_CSI_2_WD_CTRL,
	VI_CSI_3_WD_CTRL,
	VI_CSI_4_WD_CTRL,
	VI_CSI_5_WD_CTRL,
};

static const u32 status_reg_table[] = {
	VI_CSI_0_ERROR_STATUS,
	VI_CSI_1_ERROR_STATUS,
	VI_CSI_2_ERROR_STATUS,
	VI_CSI_3_ERROR_STATUS,
	VI_CSI_4_ERROR_STATUS,
	VI_CSI_5_ERROR_STATUS,
	VI_CFG_INTERRUPT_STATUS_0,
};

static struct vi *to_vi(struct tegra_drm_client *client)
{
	return container_of(client, struct vi, client);
}

static u32 vi_readl(struct vi *vi, u32 r)
{
	return readl(vi->regs + r);
}

static inline void vi_writel(struct vi *vi, u32 v, u32 r)
{
	writel(v, vi->regs + r);
}

static void clear_state(struct vi *tegra_vi, int addr)
{
	u32 val;

	val = vi_readl(tegra_vi, addr);
	vi_writel(tegra_vi, val, addr);
}

static void mask_interrupts(struct vi *tegra_vi)
{
	int i;

	/* Mask all VI interrupts */
	for (i = 0; i < ARRAY_SIZE(mask_reg_table); i++)
		vi_writel(tegra_vi, 0, mask_reg_table[i]);
}

static void clear_interrupts(struct vi *tegra_vi)
{
	int i;

	/* Clear all VI interrupt state registers */
	for (i = 0; i < ARRAY_SIZE(status_reg_table); i++)
		clear_state(tegra_vi, status_reg_table[i]);
}

static int vi_enable_irq(struct vi *tegra_vi)
{
	mask_interrupts(tegra_vi);
	clear_interrupts(tegra_vi);
	enable_irq(tegra_vi->vi_irq);

	return 0;
}

static int vi_disable_irq(struct vi *tegra_vi)
{
	disable_irq(tegra_vi->vi_irq);
	mask_interrupts(tegra_vi);
	clear_interrupts(tegra_vi);

	return 0;
}

static irqreturn_t vi_checkwd(struct vi *tegra_vi, int stream)
{
	int val;

	if (stream >= NUM_VI_CHANS)
		return IRQ_NONE;

	val = vi_readl(tegra_vi, wd_reg_table[stream].err_status_addr);
	if (val & WATCHDOG_INT) {
		vi_writel(tegra_vi, 0, wd_reg_table[stream].wd_ctrl_addr);
		vi_writel(tegra_vi, WATCHDOG_INT,
			wd_reg_table[stream].err_status_addr);
		queue_work(tegra_vi->vi_workqueue, &tegra_vi->mfi_cb_work);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static irqreturn_t vi_isr(int irq, void *dev_id)
{
	struct vi *tegra_vi = (struct vi *)dev_id;
	int i;
	irqreturn_t result = IRQ_NONE;

	dev_dbg(tegra_vi->dev, "%s: ++", __func__);

	for (i = 0; i < NUM_VI_CHANS; i++)
		if (IRQ_HANDLED == vi_checkwd(tegra_vi, i))
			result = IRQ_HANDLED;

	clear_interrupts(tegra_vi);

	return result;
}

static int vi_intr_init(struct vi *tegra_vi)
{
	struct platform_device *ndev = container_of(tegra_vi->dev,
					struct platform_device, dev);
	int ret;

	dev_dbg(tegra_vi->dev, "%s: ++", __func__);

	tegra_vi->vi_irq = platform_get_irq(ndev, 0);
	if (tegra_vi->vi_irq < 0) {
		dev_err(tegra_vi->dev, "missing vi irq\n");
		return tegra_vi->vi_irq;
	}

	ret = request_irq(tegra_vi->vi_irq, vi_isr, 0,
			dev_name(tegra_vi->dev), tegra_vi);
	if (ret) {
		dev_err(tegra_vi->dev, "failed to get vi irq\n");
		return -EBUSY;
	}

	disable_irq(tegra_vi->vi_irq);

	return 0;
}

static int vi_intr_free(struct vi *tegra_vi)
{
	dev_dbg(tegra_vi->dev, "%s: ++", __func__);

	free_irq(tegra_vi->vi_irq, tegra_vi);

	return 0;
}

int tegra_vi_register_mfi_cb(callback cb, void *cb_arg)
{
	mutex_lock(&vi_mfi_lock);
	if (mfi_callback || mfi_callback_arg) {
		pr_err("cb already registered\n");
		mutex_unlock(&vi_mfi_lock);
		return -EEXIST;
	}

	mfi_callback = cb;
	mfi_callback_arg = cb_arg;
	mutex_unlock(&vi_mfi_lock);

	return 0;
}
EXPORT_SYMBOL(tegra_vi_register_mfi_cb);

int tegra_vi_unregister_mfi_cb(void)
{
	mutex_lock(&vi_mfi_lock);
	mfi_callback = NULL;
	mfi_callback_arg = NULL;
	mutex_unlock(&vi_mfi_lock);

	return 0;
}
EXPORT_SYMBOL(tegra_vi_unregister_mfi_cb);

static void vi_mfi_worker(struct work_struct *vi_work)
{
	mutex_lock(&vi_mfi_lock);
	if (mfi_callback == NULL) {
		pr_err("NULL MFI callback\n");
		mutex_unlock(&vi_mfi_lock);
		return;
	}

	mfi_callback(mfi_callback_arg);
	mutex_unlock(&vi_mfi_lock);
}

static int vi_slcg_handler(struct notifier_block *nb,
	unsigned long unused0, void *unused1)
{
	struct vi *vi = container_of(nb, struct vi, slcg_notifier);
	int ret;

	tegra210_csi_source_from_plld();
	ret = clk_prepare_enable(vi->plld_clk);
	if (ret) {
		tegra210_csi_source_from_brick();
		dev_err(vi->dev, "can't enable pll_d");
		return ret;
	}

	udelay(1);

	clk_disable_unprepare(vi->plld_clk);
	tegra210_csi_source_from_brick();

	return NOTIFY_OK;
}

static int vi_power_off(struct device *dev)
{
	struct vi *vi = dev_get_drvdata(dev);
	int err;

	err = vi_disable_irq(vi);
	if (err) {
		dev_err(dev, "%s: vi_disable_irq failed\n", __func__);
		return err;
	}

	err = reset_control_assert(vi->rst);
	if (err)
		return err;

	err = regulator_disable(vi->reg);
	if (err) {
		reset_control_deassert(vi->rst);
		dev_err(dev, "disable csi regulator failed");
		return err;
	}

	clk_disable_unprepare(vi->clk);
	clk_disable_unprepare(vi->sclk);
	clk_disable_unprepare(vi->csus);
	clk_disable_unprepare(vi->csi);
	clk_disable_unprepare(vi->cile);
	clk_disable_unprepare(vi->cilcd);
	clk_disable_unprepare(vi->cilab);

	return tegra_pmc_powergate(vi->config->powergate_id);
}

static int vi_power_on(struct device *dev)
{
	struct vi *vi = dev_get_drvdata(dev);
	int err;

	err = tegra_pmc_unpowergate(vi->config->powergate_id);
	if (err)
		return err;

	err = clk_prepare_enable(vi->cilab);
	if (err) {
		dev_err(dev, "enable cilab failed.\n");
		goto enable_cilab_failed;
	}

	err = clk_prepare_enable(vi->cilcd);
	if (err) {
		dev_err(dev, "enable cilcd failed.\n");
		goto enable_cilcd_failed;
	}

	err = clk_prepare_enable(vi->cile);
	if (err) {
		dev_err(dev, "enable cile failed.\n");
		goto enable_cile_failed;
	}

	err = clk_prepare_enable(vi->csi);
	if (err) {
		dev_err(dev, "enable csi failed.\n");
		goto enable_csi_failed;
	}

	err = clk_prepare_enable(vi->csus);
	if (err) {
		dev_err(dev, "enable cus failed.\n");
		goto enable_csus_failed;
	}

	err = clk_prepare_enable(vi->sclk);
	if (err) {
		dev_err(dev, "enable sclk failed.\n");
		goto enable_sclk_failed;
	}

	err = clk_prepare_enable(vi->clk);
	if (err) {
		dev_err(dev, "enable vi clk failed.\n");
		goto enable_vi_failed;
	}

	err = regulator_enable(vi->reg);
	if (err) {
		dev_err(dev, "enable csi regulator failed.\n");
		goto enable_reg_failed;
	}

	vi_writel(vi, T12_CG_2ND_LEVEL_EN, T12_VI_CFG_CG_CTRL);

	err = vi_enable_irq(vi);
	if (err) {
		dev_err(dev, "%s: vi_enable_irq failed\n", __func__);
		return err;
	}

	return 0;
enable_reg_failed:
	clk_disable_unprepare(vi->clk);
enable_vi_failed:
	clk_disable_unprepare(vi->sclk);
enable_sclk_failed:
	clk_disable_unprepare(vi->csus);
enable_csus_failed:
	clk_disable_unprepare(vi->csi);
enable_csi_failed:
	clk_disable_unprepare(vi->cile);
enable_cile_failed:
	clk_disable_unprepare(vi->cilcd);
enable_cilcd_failed:
	clk_disable_unprepare(vi->cilab);
enable_cilab_failed:
	tegra_pmc_powergate(vi->config->powergate_id);

	return err;

}

static void vi_reset(struct device *dev)
{
	struct vi *vi = dev_get_drvdata(dev);

	vi_power_off(dev);
	vi_power_on(dev);

	/* Reset sensor A */
	vi_writel(vi, 0x00000000, T12_VI_CSI_0_CSI_IMAGE_DT);

	vi_writel(vi, T12_CSI_CSICIL_SW_SENSOR_A_RESET_SENSOR_A_RESET,
		  T12_CSI_CSICIL_SW_SENSOR_A_RESET);

	vi_writel(vi, T12_CSI_CSI_SW_SENSOR_A_RESET_SENSOR_A_RESET,
		  T12_CSI_CSI_SW_SENSOR_A_RESET);

	vi_writel(vi, T12_VI_CSI_0_SW_RESET_SHADOW_RESET |
		      T12_VI_CSI_0_SW_RESET_SENSORCTL_RESET |
		      T12_VI_CSI_0_SW_RESET_PF_RESET |
		      T12_VI_CSI_0_SW_RESET_MCINTF_RESET |
		      T12_VI_CSI_0_SW_RESET_ISPINTF_RESET,
		  T12_VI_CSI_0_SW_RESET);

	/* Reset sensor B */
	vi_writel(vi, 0x00000000, T12_VI_CSI_1_CSI_IMAGE_DT);
	vi_writel(vi, T12_CSI_CSICIL_SW_SENSOR_B_RESET_SENSOR_B_RESET,
		  T12_CSI_CSICIL_SW_SENSOR_B_RESET);

	vi_writel(vi, T12_CSI_CSI_SW_SENSOR_B_RESET_SENSOR_B_RESET,
		  T12_CSI_CSI_SW_SENSOR_B_RESET);

	vi_writel(vi, T12_VI_CSI_1_SW_RESET_SHADOW_RESET |
		      T12_VI_CSI_1_SW_RESET_SENSORCTL_RESET |
		      T12_VI_CSI_1_SW_RESET_PF_RESET |
		      T12_VI_CSI_1_SW_RESET_MCINTF_RESET |
		      T12_VI_CSI_1_SW_RESET_ISPINTF_RESET,
		  T12_VI_CSI_1_SW_RESET);

	udelay(10);

	vi_writel(vi, 0x00000000, T12_CSI_CSI_SW_SENSOR_A_RESET);
	vi_writel(vi, 0x00000000, T12_CSI_CSICIL_SW_SENSOR_A_RESET);
	vi_writel(vi, 0x00000000, T12_CSI_CSI_SW_SENSOR_B_RESET);
	vi_writel(vi, 0x00000000, T12_CSI_CSICIL_SW_SENSOR_B_RESET);
	vi_writel(vi, 0x00000000, T12_VI_CSI_0_SW_RESET);
	vi_writel(vi, 0x00000000, T12_VI_CSI_1_SW_RESET);
}

static int vi_init(struct host1x_client *client)
{
	struct tegra_drm_client *drm = host1x_to_drm_client(client);
	struct drm_device *dev = dev_get_drvdata(client->parent);
	struct tegra_drm *tegra = dev->dev_private;
	unsigned long flags = HOST1X_SYNCPT_HAS_BASE;
	struct vi *vi = to_vi(drm);
	unsigned int i;
	int err;

	if (tegra->domain) {
		err = iommu_attach_device(tegra->domain, client->dev);
		if (err < 0) {
			dev_err(client->dev, "failed to attach to domain: %d\n",
				err);
			return err;
		}

		vi->domain = tegra->domain;
	}

	vi->channel = host1x_channel_request(client);
	if (!vi->channel) {
		err = -ENOMEM;
		goto error_iommu_detach_device;
	}

	for (i = 0; i < VI_NUM_SYNCPTS; i++) {
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
	for (i = 0; i < VI_NUM_SYNCPTS; i++)
		if (client->syncpts[i]) {
			host1x_syncpt_free(client->syncpts[i]);
			client->syncpts[i] = NULL;
		}
	host1x_channel_free(vi->channel);
error_iommu_detach_device:
	if (vi->domain) {
		iommu_detach_device(vi->domain, vi->dev);
		vi->domain = NULL;
	}
	return err;
}

static int vi_exit(struct host1x_client *client)
{
	struct tegra_drm_client *drm = host1x_to_drm_client(client);
	struct drm_device *dev = dev_get_drvdata(client->parent);
	struct vi *vi = to_vi(drm);
	unsigned int i;
	int err;

	err = tegra_drm_unregister_client(dev->dev_private, drm);

	for (i = 0; i < VI_NUM_SYNCPTS; i++)
		if (client->syncpts[i]) {
			host1x_syncpt_free(client->syncpts[i]);
			client->syncpts[i] = NULL;
		}

	host1x_channel_free(vi->channel);

	if (vi->domain) {
		iommu_detach_device(vi->domain, vi->dev);
		vi->domain = NULL;
	}

	return 0;
}

static const struct host1x_client_ops vi_client_ops = {
	.init = vi_init,
	.exit = vi_exit,
};

static int vi_open_channel(struct tegra_drm_client *client,
			   struct tegra_drm_context *context)
{
	struct vi *vi = to_vi(client);

	context->channel = host1x_channel_get(vi->channel, &context->user);
	if (!context->channel)
		return -ENOMEM;

	return 0;
}

static void vi_close_channel(struct tegra_drm_context *context)
{
	if (!context->channel)
		return;

	host1x_channel_put(context->channel, &context->user);
	context->channel = NULL;
}

static int vi_is_addr_reg(struct device *dev, u32 class, u32 offset)
{
	return 0;
}

static const struct tegra_drm_client_ops vi_ops = {
	.open_channel = vi_open_channel,
	.close_channel = vi_close_channel,
	.is_addr_reg = vi_is_addr_reg,
	.submit = tegra_drm_submit,
	.reset = vi_reset,
};

static const struct vi_config vi_t210_config = {
	.class_id = HOST1X_CLASS_VI,
	.powergate_id = TEGRA_POWERGATE_VENC,
};

static const struct of_device_id vi_match[] = {
	{ .compatible = "nvidia,tegra210-vi", .data = &vi_t210_config },
	{ },
};

static int vi_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct vi_config *vi_config;
	struct device *dev = &pdev->dev;
	struct host1x_syncpt **syncpts;
	struct resource *regs;
	struct vi *vi;
	int err;

	if (!dev->of_node) {
		dev_err(&pdev->dev, "no dt node\n");
		return -EINVAL;
	}

	match = of_match_node(vi_match, pdev->dev.of_node);
	if (match)
		vi_config = (struct vi_config *)match->data;
	else
		return -ENXIO;

	vi = devm_kzalloc(dev, sizeof(*vi), GFP_KERNEL);
	if (!vi)
		return -ENOMEM;

	syncpts = devm_kzalloc(dev, VI_NUM_SYNCPTS * sizeof(*syncpts),
			       GFP_KERNEL);
	if (!syncpts)
		return -ENOMEM;

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	vi->regs = devm_ioremap_resource(&pdev->dev, regs);
	if (IS_ERR(vi->regs))
		return PTR_ERR(vi->regs);

	vi->rst = devm_reset_control_get(&pdev->dev, "vi");
	if (IS_ERR(vi->rst)) {
		dev_err(&pdev->dev, "cannot get reset\n");
		return PTR_ERR(vi->rst);
	}

	vi->cilab = devm_clk_get(dev, "cilab");
	if (IS_ERR(vi->cilab))
		return PTR_ERR(vi->cilab);

	vi->cilcd = devm_clk_get(dev, "cilcd");
	if (IS_ERR(vi->cilcd))
		return PTR_ERR(vi->cilcd);

	vi->cile = devm_clk_get(dev, "cile");
	if (IS_ERR(vi->cile))
		return PTR_ERR(vi->cile);

	vi->csi = devm_clk_get(dev, "csi");
	if (IS_ERR(vi->csi))
		return PTR_ERR(vi->csi);

	vi->csus = devm_clk_get(dev, "csus");
	if (IS_ERR(vi->csus))
		return PTR_ERR(vi->csus);

	vi->sclk = devm_clk_get(dev, "sclk");
	if (IS_ERR(vi->sclk))
		return PTR_ERR(vi->sclk);

	vi->clk = devm_clk_get(dev, "vi");
	if (IS_ERR(vi->clk))
		return PTR_ERR(vi->clk);

	vi->plld_clk = devm_clk_get(dev, "pll_d");
	if (IS_ERR(vi->plld_clk))
		return PTR_ERR(vi->plld_clk);

	vi->reg = devm_regulator_get(dev, "avdd-dsi-csi");
	if (IS_ERR(vi->reg)) {
		err = PTR_ERR(vi->reg);
		return err;
	}

	INIT_LIST_HEAD(&vi->client.base.list);
	vi->client.base.ops = &vi_client_ops;
	vi->client.base.dev = dev;
	vi->client.base.class = vi_config->class_id;
	vi->client.base.syncpts = syncpts;
	vi->client.base.num_syncpts = VI_NUM_SYNCPTS;
	vi->dev = dev;
	vi->config = vi_config;

	INIT_LIST_HEAD(&vi->client.list);
	vi->client.ops = &vi_ops;

	vi->slcg_notifier.notifier_call = vi_slcg_handler;
	tegra_slcg_register_notifier(vi->config->powergate_id,
		&vi->slcg_notifier);

	/* create vi workqueue */
	vi->vi_workqueue = alloc_workqueue("vi_workqueue",
					WQ_HIGHPRI | WQ_UNBOUND, 1);
	if (!vi->vi_workqueue) {
		dev_err(dev, "Failed to allocated vi_workqueue");
		err = -ENOMEM;
		goto error_slcg;
	}

	/* Init mfi callback work */
	INIT_WORK(&vi->mfi_cb_work, vi_mfi_worker);

	/* Init VI IRQs */
	err = vi_intr_init(vi);
	if (err < 0)
		goto error_workqueue;

	platform_set_drvdata(pdev, vi);

	err = host1x_client_register(&vi->client.base);
	if (err < 0) {
		dev_err(dev, "failed to register host1x client: %d\n", err);
		goto error_intr_init;
	}

	err = vi_power_on(&pdev->dev);
	if (err) {
		dev_err(dev, "cannot power on device\n");
		goto error_client_unregister;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_set_autosuspend_delay(dev, 5000);

	dev_info(&pdev->dev, "initialized");

	return 0;

error_client_unregister:
	host1x_client_unregister(&vi->client.base);
error_intr_init:
	vi_intr_free(vi);
error_workqueue:
	destroy_workqueue(vi->vi_workqueue);
error_slcg:
	tegra_slcg_unregister_notifier(vi->config->powergate_id,
				&vi->slcg_notifier);
	return err;
}

static int vi_remove(struct platform_device *pdev)
{
	struct vi *vi = platform_get_drvdata(pdev);
	int err;

	vi_power_off(&pdev->dev);

	err = host1x_client_unregister(&vi->client.base);
	if (err < 0)
		dev_err(&pdev->dev, "failed to unregister host1x client: %d\n",
			err);

	vi_intr_free(vi);
	destroy_workqueue(vi->vi_workqueue);

	tegra_slcg_unregister_notifier(vi->config->powergate_id,
		&vi->slcg_notifier);

	return err;
}

static int __maybe_unused vi_runtime_suspend(struct device *dev)
{
	dev_info(dev, "%s\n", __func__);
	return vi_power_off(dev);
}

static int __maybe_unused vi_runtime_resume(struct device *dev)
{
	dev_info(dev, "%s\n", __func__);
	return vi_power_on(dev);
}

static const struct dev_pm_ops vi_pm_ops = {
	SET_RUNTIME_PM_OPS(vi_runtime_suspend, vi_runtime_resume, NULL)
};

struct platform_driver tegra_vi_driver = {
	.driver = {
		.name = "tegra-vi",
		.of_match_table = vi_match,
		.pm = &vi_pm_ops,
	},
	.probe = vi_probe,
	.remove = vi_remove,
};
