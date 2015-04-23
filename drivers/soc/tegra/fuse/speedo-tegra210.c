/*
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/bug.h>

#include <soc/tegra/fuse.h>

#include "fuse.h"

#define PROCESS_CORNERS	2
#define CORE_PROCESS_CORNERS	3

#define FUSE_CPU_SPEEDO_0	0x14
#define FUSE_CPU_SPEEDO_1	0x2c
#define FUSE_CPU_SPEEDO_2	0x30
#define FUSE_SOC_SPEEDO_0	0x34
#define FUSE_SOC_SPEEDO_1	0x38
#define FUSE_SOC_SPEEDO_2	0x3c
#define FUSE_CPU_IDDQ		0x18
#define FUSE_SOC_IDDQ		0x40
#define FUSE_GPU_IDDQ		0x128
#define FUSE_FT_REV		0x28

#define SPEEDO_VERSION_CTRL_BIT0	0x2
#define SPEEDO_VERSION_CTRL_BIT1	0x3
#define SPEEDO_VERSION_CTRL_BIT2	0x4

#define TEGRA210_CPU_SPEEDO 2100
#define TEGRA210_GPU_SPEEDO 2100
#define TEGRA210_GPU_SPEEDO_OFFS 75

enum {
	THRESHOLD_INDEX_0,
	THRESHOLD_INDEX_1,
	THRESHOLD_INDEX_COUNT,
};

static const u32 __initconst cpu_process_speedos[][PROCESS_CORNERS] = {
	{2119,	UINT_MAX},
	{2119,	UINT_MAX},
};

static const u32 __initconst gpu_process_speedos[][PROCESS_CORNERS] = {
	{UINT_MAX,	UINT_MAX},
	{UINT_MAX,	UINT_MAX},
};

static const u32 __initconst core_process_speedos[][CORE_PROCESS_CORNERS] = {
	{1950,	2100,	UINT_MAX},
	{1950,	2100,	UINT_MAX},
};

static int __init tegra210_get_speedo_rev(void)
{
	return tegra30_spare_fuse(SPEEDO_VERSION_CTRL_BIT0) |
		(tegra30_spare_fuse(SPEEDO_VERSION_CTRL_BIT1) << 0x1) |
		(tegra30_spare_fuse(SPEEDO_VERSION_CTRL_BIT2) << 0x2);
}

static void __init rev_sku_to_speedo_ids(struct tegra_sku_info *sku_info,
					 int *threshold, int speedo_rev)
{
	int sku = sku_info->sku_id;
	int rev = sku_info->revision;

	/* Assign to default */
	sku_info->cpu_speedo_id = 0;
	sku_info->soc_speedo_id = 0;
	sku_info->gpu_speedo_id = 0;
	*threshold = THRESHOLD_INDEX_0;

	switch (sku) {
	case 0x01: /* Eng sku */
	case 0x13:
		if (rev == TEGRA_REVISION_A02) {
			sku_info->cpu_speedo_id = 1;
			sku_info->gpu_speedo_id = 2;
			break;
		}
		/* fall through for a01 */
	case 0x00: /* Eng sku */
	case 0x07:
	case 0x17:
	case 0x27:
		sku_info->gpu_speedo_id = 1;
		break;
	case 0x83:
		if (rev == TEGRA_REVISION_A02) {
			sku_info->cpu_speedo_id = 1;
			sku_info->gpu_speedo_id = 4;
			break;
		}
		/* fall through for a01 */
	case 0x87:
		sku_info->gpu_speedo_id = 3;
		break;
	default:
		pr_warn("Tegra210 Unknown SKU %d\n", sku);
		break;
	}

	/* Overwrite GPU speedo selection for speedo revision 0, 1 */
	if (speedo_rev < 2)
		sku_info->gpu_speedo_id = 0;
}

void __init tegra210_init_speedo_data(struct tegra_sku_info *sku_info)
{
	int i, threshold, soc_speedo_0_value;
	int cpu_iddq_value, gpu_iddq_value, soc_iddq_value;
	int speedo_rev;

	BUILD_BUG_ON(ARRAY_SIZE(cpu_process_speedos) !=
			THRESHOLD_INDEX_COUNT);
	BUILD_BUG_ON(ARRAY_SIZE(gpu_process_speedos) !=
			THRESHOLD_INDEX_COUNT);
	BUILD_BUG_ON(ARRAY_SIZE(core_process_speedos) !=
			THRESHOLD_INDEX_COUNT);

	sku_info->cpu_speedo_value = tegra30_fuse_readl(FUSE_CPU_SPEEDO_0);
	/* GPU Speedo is stored in CPU_SPEEDO_2 */
	sku_info->gpu_speedo_value = tegra30_fuse_readl(FUSE_CPU_SPEEDO_2);

	soc_speedo_0_value = tegra30_fuse_readl(FUSE_SOC_SPEEDO_0);

	cpu_iddq_value = tegra30_fuse_readl(FUSE_CPU_IDDQ) * 4;
	soc_iddq_value = tegra30_fuse_readl(FUSE_SOC_IDDQ) * 4;
	gpu_iddq_value = tegra30_fuse_readl(FUSE_GPU_IDDQ) * 5;

	speedo_rev = tegra210_get_speedo_rev();
	/*
	 * - If speedo revision is 3 or above, apply fused CPU, GPU speedo
	 *   values as is.
	 * - If speedo revision is 2, convert fused CPU, GPU speedo values
	 *   using linear equation with fixed coefficients specified for each
	 *   domain.
	 * - If speedo revision is 0 or 1, ignore fused speedo values for CPU,
	 *   apply constant speedo values specified for this domain; for GPU
	 *   apply fused speedo value with specified fixed offset.
	 */
	if (speedo_rev == 2) {
		sku_info->cpu_speedo_value =
			(-1938 + (1095 * sku_info->cpu_speedo_value / 100)) / 10;
		sku_info->gpu_speedo_value =
			(-1662 + (1082 * sku_info->gpu_speedo_value / 100)) / 10;
	} else if (speedo_rev <= 1) {
		sku_info->cpu_speedo_value = TEGRA210_CPU_SPEEDO;
		sku_info->gpu_speedo_value = sku_info->gpu_speedo_value -
					TEGRA210_GPU_SPEEDO_OFFS;
	}

	if (sku_info->cpu_speedo_value == 0 ||
		sku_info->gpu_speedo_value == 0) {
		WARN(1, "Tegra Warning: Speedo value not fused.\n");
		return;
	}

	rev_sku_to_speedo_ids(sku_info, &threshold, speedo_rev);

	sku_info->cpu_iddq_value = tegra30_fuse_readl(FUSE_CPU_IDDQ) * 4;

	for (i = 0; i < PROCESS_CORNERS; i++)
		if (sku_info->gpu_speedo_value <
			gpu_process_speedos[threshold][i])
			break;
	sku_info->gpu_process_id = i;

	for (i = 0; i < PROCESS_CORNERS; i++)
		if (sku_info->cpu_speedo_value <
			cpu_process_speedos[threshold][i])
				break;
	sku_info->cpu_process_id = i;

	for (i = 0; i < PROCESS_CORNERS; i++)
		if (soc_speedo_0_value <
			core_process_speedos[threshold][i])
			break;
	sku_info->core_process_id = i;

	pr_debug("Tegra CPU Speedo ID=%d, Speedo Value=%d\n",
		 sku_info->cpu_speedo_id, sku_info->cpu_speedo_value);
	pr_debug("Tegra GPU Speedo ID=%d, Speedo Value=%d\n",
		 sku_info->gpu_speedo_id, sku_info->gpu_speedo_value);
}
