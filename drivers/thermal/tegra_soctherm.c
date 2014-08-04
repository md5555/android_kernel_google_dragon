/*
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author:
 *	Mikko Perttunen <mperttunen@nvidia.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/thermal.h>

#include <soc/tegra/fuse.h>

#include <dt-bindings/thermal/tegra124-soctherm.h>

#include "tegra_soctherm.h"

#define SENSOR_CONFIG0				0
#define SENSOR_CONFIG0_STOP			BIT(0)
#define SENSOR_CONFIG0_TALL_SHIFT		8
#define SENSOR_CONFIG0_TCALC_OVER		BIT(4)
#define SENSOR_CONFIG0_OVER			BIT(3)
#define SENSOR_CONFIG0_CPTR_OVER		BIT(2)

#define SENSOR_CONFIG1				4
#define SENSOR_CONFIG1_TSAMPLE_SHIFT		0
#define SENSOR_CONFIG1_TIDDQ_EN_SHIFT		15
#define SENSOR_CONFIG1_TEN_COUNT_SHIFT		24
#define SENSOR_CONFIG1_TEMP_ENABLE		BIT(31)

#define SENSOR_CONFIG2				8
#define SENSOR_CONFIG2_THERMA_SHIFT		16
#define SENSOR_CONFIG2_THERMB_SHIFT		0

#define		THERMCTL_LEVEL0_GROUP_EN	BIT(8)
#define		THERMCTL_LEVEL0_GROUP_DN_THRESH_SHIFT 9
#define		THERMCTL_LEVEL0_GROUP_UP_THRESH_SHIFT 17

#define THERMCTL_INTR_STATUS			0x84
#define THERMCTL_INTR_EN			0x88
#define		TH_INTR_UP_DOWN_LVL0_MASK	0x3

#define SENSOR_PDIV				0x1c0
#define SENSOR_HOTSPOT_OFF			0x1c4

#define SENSOR_TEMP_MASK			0xffff
#define READBACK_VALUE_MASK			0xff00
#define READBACK_VALUE_SHIFT			8
#define READBACK_ADD_HALF			BIT(7)
#define READBACK_NEGATE				BIT(1)

#define FUSE_TSENSOR8_CALIB			0x180
#define FUSE_SPARE_REALIGNMENT_REG_0		0x1fc

static const int min_low_temp = -127000;
static const int max_high_temp = 127000;

struct tegra_soctherm {
	struct platform_device *pdev;
	struct reset_control *reset;
	struct clk *clock_tsensor;
	struct clk *clock_soctherm;
	void __iomem *regs;

	struct thermal_zone_device *thermctl_tzs[4];
	struct tegra_tsensor_group *sensor_groups;
};

struct tegra_thermctl_zone {
	struct tegra_soctherm *tegra;
	struct tegra_tsensor_group *sensor_group;
	struct thermal_zone_device *tz;
};

/**
 * soctherm_writel() - writes a value to a SOC_THERM register
 * @ts: pointer to a struct tegra_soctherm
 * @v: the value to write
 * @reg: the register offset
 *
 * Writes @v to @reg.  No return value.
 */
static void soctherm_writel(struct tegra_soctherm *ts, u32 v, u16 reg)
{
	writel(v, ts->regs + reg);
}

/**
 * soctherm_readl() - reads specified register from SOC_THERM IP block
 * @ts: pointer to a struct tegra_soctherm
 * @reg: register address to be read
 *
 * Return: the value of the register
 */
static u32 soctherm_readl(struct tegra_soctherm *ts, u16 reg)
{
	return readl(ts->regs + reg);
}

/**
 * soctherm_barrier() - ensure previous writes to SOC_THERM have completed
 * @ts: pointer to a struct tegra_soctherm
 *
 * Ensures that any previous writes to the SOC_THERM IP block have reached
 * the IP block before continuing.
 */
static void soctherm_barrier(struct tegra_soctherm *ts)
{
	soctherm_readl(ts, THERMCTL_LEVEL0_GROUP_CPU);
}

static int enable_tsensor(struct tegra_soctherm *tegra,
			  struct tegra_tsensor *sensor,
			  struct tsensor_shared_calibration *shared)
{
	unsigned int val;
	u32 calib;
	int err;

	err = tegra_soctherm_calculate_tsensor_calibration(sensor,
							   sensor->group,
							   *shared, &calib);
	if (err)
		return err;

	val = sensor->config->tall << SENSOR_CONFIG0_TALL_SHIFT;
	soctherm_writel(tegra, val, sensor->base + SENSOR_CONFIG0);

	val  = (sensor->config->tsample - 1) << SENSOR_CONFIG1_TSAMPLE_SHIFT;
	val |= sensor->config->tiddq_en << SENSOR_CONFIG1_TIDDQ_EN_SHIFT;
	val |= sensor->config->ten_count << SENSOR_CONFIG1_TEN_COUNT_SHIFT;
	val |= SENSOR_CONFIG1_TEMP_ENABLE;
	soctherm_writel(tegra, val, sensor->base + SENSOR_CONFIG1);

	soctherm_writel(tegra, calib, sensor->base + SENSOR_CONFIG2);

	return 0;
}

/*
 * Translate from soctherm readback format to millicelsius.
 * The soctherm readback format in bits is as follows:
 *   TTTTTTTT H______N
 * where T's contain the temperature in Celsius,
 * H denotes an addition of 0.5 Celsius and N denotes negation
 * of the final value.
 */
static long translate_temp(u16 val)
{
	long t;

	t = ((val & READBACK_VALUE_MASK) >> READBACK_VALUE_SHIFT) * 1000;
	if (val & READBACK_ADD_HALF)
		t += 500;
	if (val & READBACK_NEGATE)
		t *= -1;

	return t;
}

static int tegra_thermctl_get_temp(void *data, long *out_temp)
{
	struct tegra_thermctl_zone *zone = data;
	u32 val;

	val = soctherm_readl(zone->tegra,
			     zone->sensor_group->sensor_temp_offset);
	val &= zone->sensor_group->sensor_temp_mask;
	val >>= ffs(zone->sensor_group->sensor_temp_mask) - 1;

	*out_temp = translate_temp(val);

	return 0;
}

static int tegra_thermctl_set_trips(void *data, long low, long high)
{
	struct tegra_thermctl_zone *zone = data;
	u32 val;

	low /= 1000;
	high /= 1000;

	low = clamp_val(low, -127, 127);
	high = clamp_val(high, -127, 127);

	val = ((u8) low) << THERMCTL_LEVEL0_GROUP_DN_THRESH_SHIFT;
	val |= ((u8) high) << THERMCTL_LEVEL0_GROUP_UP_THRESH_SHIFT;
	val |= THERMCTL_LEVEL0_GROUP_EN;

	soctherm_writel(zone->tegra, val,
	       zone->sensor_group->thermctl_level0_offset);

	return 0;
}

static irqreturn_t soctherm_isr(int irq, void *dev_id)
{
	struct tegra_thermctl_zone *zone = dev_id;
	u32 val;
	u32 intr_mask = TH_INTR_UP_DOWN_LVL0_MASK <<
			zone->sensor_group->thermctl_isr_shift;

	val = soctherm_readl(zone->tegra, THERMCTL_INTR_STATUS);

	if ((val & intr_mask) == 0)
		return IRQ_NONE;

	soctherm_writel(zone->tegra, val & intr_mask, THERMCTL_INTR_STATUS);

	return IRQ_WAKE_THREAD;
}

static irqreturn_t soctherm_isr_thread(int irq, void *dev_id)
{
	struct tegra_thermctl_zone *zone = dev_id;

	thermal_zone_device_update(zone->tz);

	return IRQ_HANDLED;
}

static const struct thermal_zone_of_device_ops tegra_of_thermal_ops = {
	.get_temp = tegra_thermctl_get_temp,
	.set_trips = tegra_thermctl_set_trips,
};

int tegra_soctherm_probe(struct platform_device *pdev,
		struct tegra_tsensor_configuration *tegra_tsensor_configs,
		struct tegra_tsensor *tsensors,
		struct tegra_tsensor_group **tegra_tsensor_groups,
		u8 nominal_calib_cp, u8 nominal_calib_ft)
{
	struct tegra_soctherm *tegra;
	struct thermal_zone_device *tz;
	struct tegra_tsensor_group *ttg;
	struct tsensor_shared_calibration shared_calib;
	struct resource *res;
	unsigned int i;
	int err, irq;
	u32 v;

	tegra = devm_kzalloc(&pdev->dev, sizeof(*tegra), GFP_KERNEL);
	if (!tegra)
		return -ENOMEM;

	tegra->sensor_groups = *tegra_tsensor_groups;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	tegra->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(tegra->regs))
		return PTR_ERR(tegra->regs);

	tegra->reset = devm_reset_control_get(&pdev->dev, "soctherm");
	if (IS_ERR(tegra->reset)) {
		dev_err(&pdev->dev, "can't get soctherm reset\n");
		return PTR_ERR(tegra->reset);
	}

	tegra->clock_tsensor = devm_clk_get(&pdev->dev, "tsensor");
	if (IS_ERR(tegra->clock_tsensor)) {
		dev_err(&pdev->dev, "can't get tsensor clock\n");
		return PTR_ERR(tegra->clock_tsensor);
	}

	tegra->clock_soctherm = devm_clk_get(&pdev->dev, "soctherm");
	if (IS_ERR(tegra->clock_soctherm)) {
		dev_err(&pdev->dev, "can't get soctherm clock\n");
		return PTR_ERR(tegra->clock_soctherm);
	}

	irq = platform_get_irq(pdev, 0);
	if (irq <= 0) {
		dev_err(&pdev->dev, "can't get interrupt\n");
		return -EINVAL;
	}

	reset_control_assert(tegra->reset);

	err = clk_prepare_enable(tegra->clock_soctherm);
	if (err)
		return err;

	err = clk_prepare_enable(tegra->clock_tsensor);
	if (err) {
		clk_disable_unprepare(tegra->clock_soctherm);
		return err;
	}

	reset_control_deassert(tegra->reset);

	/* Initialize raw sensors */

	err = tegra_soctherm_calculate_shared_calibration(&shared_calib,
							  nominal_calib_cp,
							  nominal_calib_ft);
	if (err)
		goto disable_clocks;

	for (i = 0; tsensors[i].name; ++i) {
		err = enable_tsensor(tegra, tsensors + i, &shared_calib);
		if (err)
			goto disable_clocks;
	}

	/* Initialize thermctl sensors */

	for (i = 0; tegra_tsensor_groups[i]; ++i) {
		struct tegra_thermctl_zone *zone =
			devm_kzalloc(&pdev->dev, sizeof(*zone), GFP_KERNEL);
		if (!zone) {
			err = -ENOMEM;
			goto unregister_tzs;
		}

		ttg = tegra_tsensor_groups[i];

		v = soctherm_readl(tegra, SENSOR_PDIV);
		v &= ~ttg->pdiv_mask;
		v |= ttg->pdiv << (ffs(ttg->pdiv_mask) - 1);
		soctherm_writel(tegra, v, SENSOR_PDIV);

		v = soctherm_readl(tegra, SENSOR_HOTSPOT_OFF);
		v &= ~ttg->pllx_hotspot_mask;
		v |= (ttg->pllx_hotspot_diff / 1000) <<
			(ffs(ttg->pllx_hotspot_mask) - 1);
		soctherm_writel(tegra, v, SENSOR_HOTSPOT_OFF);

		zone->sensor_group = tegra_tsensor_groups[i];
		zone->tegra = tegra;

		tz = thermal_zone_of_sensor_register(&pdev->dev, i, zone,
						     &tegra_of_thermal_ops);
		if (IS_ERR(tz)) {
			err = PTR_ERR(tz);
			dev_err(&pdev->dev, "failed to register sensor: %d\n",
				err);
			goto unregister_tzs;
		}

		zone->tz = tz;
		tegra->thermctl_tzs[i] = tz;

		err = devm_request_threaded_irq(&pdev->dev, irq, soctherm_isr,
						soctherm_isr_thread,
						IRQF_SHARED,
						dev_name(&pdev->dev),
						zone);
		if (err) {
			dev_err(&pdev->dev, "unable to register isr: %d\n",
				err);
			goto unregister_tzs;
		}

		soctherm_writel(tegra,
				TH_INTR_UP_DOWN_LVL0_MASK <<
					zone->sensor_group->thermctl_isr_shift,
				THERMCTL_INTR_EN);
	}

	return 0;

unregister_tzs:
	while (i--)
		thermal_zone_of_sensor_unregister(&pdev->dev,
						  tegra->thermctl_tzs[i]);

disable_clocks:
	clk_disable_unprepare(tegra->clock_tsensor);
	clk_disable_unprepare(tegra->clock_soctherm);

	return err;
}

int tegra_soctherm_remove(struct platform_device *pdev)
{
	struct tegra_soctherm *tegra = platform_get_drvdata(pdev);
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(tegra->thermctl_tzs); ++i)
		thermal_zone_of_sensor_unregister(&pdev->dev,
						  tegra->thermctl_tzs[i]);

	clk_disable_unprepare(tegra->clock_tsensor);
	clk_disable_unprepare(tegra->clock_soctherm);

	return 0;
}

MODULE_AUTHOR("Mikko Perttunen <mperttunen@nvidia.com>");
MODULE_DESCRIPTION("NVIDIA Tegra SOCTHERM thermal management driver");
MODULE_LICENSE("GPL v2");
