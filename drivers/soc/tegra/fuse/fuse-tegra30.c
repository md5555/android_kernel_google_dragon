/*
 * Copyright (c) 2013-2014, NVIDIA CORPORATION.  All rights reserved.
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
 *
 */

#include <linux/device.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/random.h>
#include <linux/regulator/consumer.h>

#include <soc/tegra/fuse.h>
#include <soc/tegra/pmc.h>

#include "fuse.h"

#define FUSE_CTRL		0x00
#define FUSE_CTRL_PD		BIT(26)
#define FUSE_REG_ADDR		0x04
#define FUSE_REG_READ		0x08
#define FUSE_REG_WRITE		0x0c
#define FUSE_TIME_PGM2		0x1c
#define FUSE_PRIV2INTFC_START	0x20
#define FUSE_DIS_PGM		0x2c
#define FUSE_WRITE_ACCESS	0x30
#define FUSE_PWR_GOOD_SW	0x34

#define FUSE_BEGIN		0x100

/* Tegra30 and later */
#define FUSE_VENDOR_CODE	0x100
#define FUSE_FAB_CODE		0x104
#define FUSE_LOT_CODE_0		0x108
#define FUSE_LOT_CODE_1		0x10c
#define FUSE_WAFER_ID		0x110
#define FUSE_X_COORDINATE	0x114
#define FUSE_Y_COORDINATE	0x118
#define FUSE_OPT_OPS_RESERVED	0x120

#define OPT_VENDOR_CODE_MASK	0xf
#define OPT_FAB_CODE_MASK	0x3f
#define OPT_LOT_CODE_1_MASK	0xfffffff
#define OPT_WAFER_ID_MASK	0x3f
#define OPT_X_COORDINATE_MASK	0x1ff
#define OPT_Y_COORDINATE_MASK	0x1ff
#define OPT_OPS_RESERVED_MASK	0x3f
#define ECID_ECID0_0_RSVD1_MASK 0x3F
#define ECID_ECID0_0_Y_MASK	0x1ff
#define ECID_ECID0_0_Y_RANGE	6
#define ECID_ECID0_0_X_MASK	0x1ff
#define ECID_ECID0_0_X_RANGE	15
#define ECID_ECID0_0_WAFER_MASK	0x3f
#define ECID_ECID0_0_WAFER_RANGE	24
#define ECID_ECID0_0_LOT1_MASK	0x3
#define ECID_ECID0_0_LOT1_RANGE	30
#define ECID_ECID1_0_LOT1_MASK	0x3ffffff
#define ECID_ECID1_0_LOT0_MASK	0x3f
#define ECID_ECID1_0_LOT0_RANGE	26
#define ECID_ECID2_0_LOT0_MASK	0x3ffffff
#define ECID_ECID2_0_FAB_MASK	0x3f
#define ECID_ECID2_0_FAB_RANGE	26
#define ECID_ECID3_0_VENDOR_MASK	0xf

#define FUSE_UID_SIZE		16

#define FUSE_HAS_REVISION_INFO	BIT(0)

#define FUSE_READ		0x1
#define FUSE_WRITE		0x2
#define FUSE_SENSE		0x3
#define FUSE_CMD_MASK		0x3

#define STATE_IDLE		(0x4 << 16)
#define SENSE_DONE		(0x1 << 30)

#define FUSETIME_PGM2_TWIDTH_PGM_MASK	0xffff

#define PRIV2INTFC_START_DATA		BIT(0)
#define PRIV2INTFC_SKIP_RAMREPAIR	BIT(1)

enum speedo_idx {
	SPEEDO_TEGRA30 = 0,
	SPEEDO_TEGRA114,
	SPEEDO_TEGRA124,
	SPEEDO_TEGRA210,
};

struct tegra_fuse_info {
	int			size;
	int			spare_bit;
	enum speedo_idx		speedo_idx;
	bool			need_sense_done;
	bool			ext_regulator;
	bool			power_down_mode;
	struct tegra_fuse_data	*fuse_array;
	int			fuse_array_size;
};

struct tegra_fuse_location {
	int fuse_addr;
	int start_bit;
	int end_bit;
};

struct tegra_fuse_data {
	int regular_addr;
	struct tegra_fuse_location loc_high;
	struct tegra_fuse_location loc_low;
	int bits_num;
};

static struct tegra_fuse_data tegra30_fuse_array[] = {
	{0x0a0, { 0,  0,  0}, { 0, 23, 23},  1},
	{0x0b8, { 0,  0,  0}, { 0, 24, 24},  1},
	{0x0bc, {26,  0,  5}, {24, 22, 31}, 16},
	{0x0c0, { 0,  0,  0}, {26,  6, 13},  8},
};

static struct tegra_fuse_data tegra114_fuse_array[] = {
	{0x0a0, { 0,  0,  0}, { 0,  7,  7},  1},
	{0x0b8, { 0,  0,  0}, { 0,  8,  8},  1},
	{0x0bc, { 0,  0,  0}, {46,  7, 22}, 16},
	{0x0c0, { 0,  0,  0}, {46, 23, 30},  8},
	{0x168, { 0,  0,  0}, {90, 22, 22},  1},
};

static struct tegra_fuse_data tegra124_fuse_array[] = {
	{0x0a0, { 0,  0,  0}, { 0, 11, 11},  1},
	{0x0b8, { 0,  0,  0}, { 0, 12, 12},  1},
	{0x0bc, { 0,  0,  0}, {44, 12, 27}, 16},
	{0x0c0, {46,  0,  3}, {44, 28, 31},  8},
	{0x0c8, {48,  0,  4}, {46,  5, 31}, 32},
	{0x0cc, {50,  0,  4}, {48,  5, 31}, 32},
	{0x0d0, {52,  0,  4}, {50,  5, 31}, 32},
	{0x0d4, {54,  0,  4}, {52,  5, 31}, 32},
	{0x0d8, {56,  0,  4}, {54,  5, 31}, 32},
	{0x0e0, {60,  0,  4}, {58,  5, 31}, 32},
	{0x0e4, {62,  0,  4}, {60,  5, 31}, 32},
	{0x168, { 0,  0,  0}, {90,  9,  9},  1},
};

static struct tegra_fuse_data tegra210_fuse_array[] = {
	{0x008, { 0,  0,  0}, { 0,  6,  9},  4}, /* odm_lock */
	{0x064, {14,  0,  5}, {12,  6, 31}, 32}, /* public_key0 */
	{0x068, {16,  0,  5}, {14,  6, 31}, 32}, /* public_key1 */
	{0x06c, {18,  0,  5}, {16,  6, 31}, 32}, /* public_key2 */
	{0x070, {20,  0,  5}, {18,  6, 31}, 32}, /* public_key3 */
	{0x074, {22,  0,  5}, {20,  6, 31}, 32}, /* public_key4 */
	{0x078, {24,  0,  5}, {22,  6, 31}, 32}, /* public_key5 */
	{0x07c, {26,  0,  5}, {24,  6, 31}, 32}, /* public_key6 */
	{0x080, {28,  0,  5}, {26,  6, 31}, 32}, /* public_key7 */
	{0x0a0, { 0,  0,  0}, { 0, 11, 11},  1}, /* security_mode */
	{0x0a4, {36,  0, 19}, {34, 20, 31}, 32}, /* private_key0 */
	{0x0a8, {38,  0, 19}, {36, 20, 31}, 32}, /* private_key1 */
	{0x0ac, {40,  0, 19}, {38, 20, 31}, 32}, /* private_key2 */
	{0x0b0, {42,  0, 19}, {40, 20, 31}, 32}, /* private_key3 */
	{0x0b4, {44,  0, 19}, {42, 20, 31}, 32}, /* private_key4 */
	{0x0b8, { 0,  0,  0}, { 0, 12, 12},  1}, /* arm_jtag_dis */
	{0x0bc, {46,  0,  3}, {44, 20, 31}, 16}, /* boot_device_info */
	{0x0c0, { 0,  0,  0}, {46,  4, 15}, 12}, /* reserved_sw */
	{0x0c8, {48,  0, 16}, {46, 17, 31}, 32}, /* reserved_odm0 */
	{0x0cc, {50,  0, 16}, {48, 17, 31}, 32}, /* reserved_odm1 */
	{0x0d0, {52,  0, 16}, {50, 17, 31}, 32}, /* reserved_odm2 */
	{0x0d4, {54,  0, 16}, {52, 17, 31}, 32}, /* reserved_odm3 */
	{0x0d8, {56,  0, 16}, {54, 17, 31}, 32}, /* reserved_odm4 */
	{0x0dc, {58,  0, 16}, {56, 17, 31}, 32}, /* reserved_odm5 */
	{0x0e0, {60,  0, 16}, {58, 17, 31}, 32}, /* reserved_odm6 */
	{0x0e4, {62,  0, 16}, {60, 17, 31}, 32}, /* reserved_odm7 */
	{0x124, { 0,  0,  0}, {72,  4,  5},  2}, /* sata_calib */
	{0x168, { 0,  0,  0}, {82,  7,  7},  1}, /* pkc_disable */
	{0x19c, {86,  0,  3}, {84, 20, 31}, 16}, /* odm_info */
	{0x1e8, { 0,  0,  0}, {90, 24, 27},  4}, /* secure_provision_index */
	{0x1ec, { 0,  0,  0}, {90, 28, 29},  2}, /* secure_provision_info */
};

static void __iomem *fuse_base;
static struct clk *fuse_clk;
static struct tegra_fuse_info *fuse_info;
static struct regulator *vpp_reg;
static struct platform_device *fuse_pdev;
static u32 pgm_cycles;

static inline void wait_for_idle(void)
{
	u32 reg;

	do {
		udelay(1);
		reg = readl_relaxed(fuse_base + FUSE_CTRL);
	} while ((reg & (0x1F << 16)) != STATE_IDLE);
}

static inline void wait_for_sense_done(void)
{
	u32 reg;

	ndelay(400);

	writel_relaxed(PRIV2INTFC_START_DATA | PRIV2INTFC_SKIP_RAMREPAIR,
			fuse_base + FUSE_PRIV2INTFC_START);

	ndelay(400);

	do {
		udelay(1);
		reg = readl_relaxed(fuse_base + FUSE_CTRL);
	} while ((reg & BIT(30)) != SENSE_DONE ||
			(reg & (0x1F << 16)) != STATE_IDLE);
}

static u32 fuse_cmd_read(int addr)
{
	u32 reg;

	wait_for_idle();

	writel_relaxed(addr, fuse_base + FUSE_REG_ADDR);

	reg = readl_relaxed(fuse_base + FUSE_CTRL);
	reg &= ~FUSE_CMD_MASK;
	reg |= FUSE_READ;
	writel_relaxed(reg, fuse_base + FUSE_CTRL);

	wait_for_idle();

	return readl_relaxed(fuse_base + FUSE_REG_READ);
}

/* Must be called by fuse_set_value() */
static void fuse_cmd_write(int addr, u32 value)
{
	u32 reg;

	wait_for_idle();

	writel_relaxed(addr, fuse_base + FUSE_REG_ADDR);
	writel_relaxed(value, fuse_base + FUSE_REG_WRITE);

	reg = readl_relaxed(fuse_base + FUSE_CTRL);
	reg &= ~FUSE_CMD_MASK;
	reg |= FUSE_WRITE;
	writel_relaxed(reg, fuse_base + FUSE_CTRL);

	wait_for_idle();
}

static u32 fuse_get_value(int index)
{
	u32 low_bits, high_bits, val;
	struct tegra_fuse_location *low, *high;
	int bits_num[2] = {0, 0};

	if (index >= fuse_info->fuse_array_size ||
			!fuse_info->fuse_array[index].bits_num)
		return ~0;

	low = &fuse_info->fuse_array[index].loc_low;
	high = &fuse_info->fuse_array[index].loc_high;

	bits_num[0] = low->end_bit + 1 - low->start_bit;

	low_bits = fuse_cmd_read(low->fuse_addr);
	low_bits >>= low->start_bit;
	if (bits_num[0] < 32)
		low_bits &= BIT(bits_num[0]) - 1;

	high_bits = 0;
	if (fuse_info->fuse_array[index].bits_num - bits_num[0] != 0) {
		bits_num[1] = high->end_bit + 1 - high->start_bit;

		high_bits = fuse_cmd_read(high->fuse_addr);
		high_bits >>= high->start_bit;

		if (bits_num[1] < 32)
			high_bits &= BIT(bits_num[1]) - 1;
	}

	val = (high_bits << bits_num[0]) | low_bits;

	return val;
}

/* Must be called by tegra30_fuse_program() */
static void fuse_set_value(int index, u32 value)
{
	u32 low_bits, high_bits;
	struct tegra_fuse_location *low, *high;
	int bits_num[2] = {0, 0};

	if (index >= fuse_info->fuse_array_size ||
			!fuse_info->fuse_array[index].bits_num)
		return;

	low = &fuse_info->fuse_array[index].loc_low;
	high = &fuse_info->fuse_array[index].loc_high;

	bits_num[0] = low->end_bit + 1 - low->start_bit;
	if (fuse_info->fuse_array[index].bits_num - bits_num[0] != 0)
		bits_num[1] = high->end_bit + 1 - high->start_bit;

	if (bits_num[0] < 32)
		low_bits = value & (BIT(bits_num[0]) - 1);
	else
		low_bits = value;
	low_bits <<= low->start_bit;

	if (low_bits) {
		fuse_cmd_write(low->fuse_addr, low_bits);
		/* also write to redundant fuse */
		fuse_cmd_write(low->fuse_addr + 1, low_bits);
	}

	if (bits_num[1]) {
		high_bits = value >> bits_num[0];
		high_bits <<= high->start_bit;
		if (high_bits) {
			fuse_cmd_write(high->fuse_addr, high_bits);
			/* also write to redundant fuse */
			fuse_cmd_write(high->fuse_addr + 1, high_bits);
		}
	}
}

u32 tegra30_fuse_readl(const unsigned int offset)
{
	u32 val;

	/*
	 * early in the boot, the fuse clock will be enabled by
	 * tegra_init_fuse()
	 */

	if (fuse_clk)
		clk_prepare_enable(fuse_clk);

	val = readl_relaxed(fuse_base + FUSE_BEGIN + offset);

	if (fuse_clk)
		clk_disable_unprepare(fuse_clk);

	return val;
}

static void tegra30_fuse_sense(void)
{
	u32 reg;

	wait_for_idle();

	reg = readl_relaxed(fuse_base + FUSE_CTRL);
	reg &= ~FUSE_CMD_MASK;
	reg |= FUSE_SENSE;
	writel_relaxed(reg, fuse_base + FUSE_CTRL);

	wait_for_idle();
}

static bool regulator_is_available(void)
{
	/* First time initialization */
	if (!vpp_reg) {
		/* Get power supply for fuse programming */
		vpp_reg = devm_regulator_get(&fuse_pdev->dev, "vdd");
		if (IS_ERR(vpp_reg))
			return false;
	} else if (IS_ERR(vpp_reg))
		return false;

	return true;
}

/*
 * Make the programming function as static and only allowed to be called
 * from fuse-tegra.c by sysfs call.
 */
static u32 tegra30_fuse_program(const unsigned int offset, const char *buf,
	u32 size)
{
	int ret;
	u32 val;
	int index = 0;
	int result = 0;
	int i, j;

	if (fuse_info->ext_regulator && !regulator_is_available())
		return -EPERM;

	clk_prepare_enable(fuse_clk);

	/*
	 * Confirm fuse option write access hasn't already been permanenttly
	 * disabled
	 */
	val = readl_relaxed(fuse_base + FUSE_DIS_PGM);
	if (val)
		goto err_pgm_disabled;

	/* Enable software writes to fuse registers */
	writel_relaxed(0x0, fuse_base + FUSE_WRITE_ACCESS);

	/* Set the fuse strobe programming width */
	if (pgm_cycles)
		writel_relaxed(pgm_cycles, fuse_base + FUSE_TIME_PGM2);

	/* Turn on 1.8V power supply */
	if (fuse_info->ext_regulator) {
		ret = regulator_enable(vpp_reg);
		if (ret) {
			WARN(1, "failed to enable vpp_fuse power supply\n");
			goto err_reg;
		}
	} else {
		ret = tegra_fuse_ps18_latch_set();
		if (ret) {
			WARN(1, "failed to set ps18 latch");
			goto err_reg;
		}
	}

	/* Enable power */
	writel_relaxed(0x1, fuse_base + FUSE_PWR_GOOD_SW);
	udelay(1);

	for (i = 0; i < size; ) {
		int addr = offset + i;
		int remainder = addr % 4;
		u32 data = 0;
		int k;

		for (j = remainder; j < 4; j++) {
			data |= buf[index++] << (j * 8);
			if (index >= size)
				break;
		}

		/*
		 * Only set the bits that should be burned, not any bits that
		 * have alreardy been burned.
		 */
		for (k = 0; k < fuse_info->fuse_array_size; k++) {
			if (round_down(addr, 4) ==
					fuse_info->fuse_array[k].regular_addr) {
				val = fuse_get_value(k);
				break;
			}
		}

		if (k == fuse_info->fuse_array_size)
			break;

		data &= ~val;
		if (data)
			fuse_set_value(k, data);

		i += 4 - remainder;
	}

	result = size;

	/* Disable power */
	writel_relaxed(0x0, fuse_base + FUSE_PWR_GOOD_SW);
	udelay(1);

	if (fuse_info->ext_regulator)
		regulator_disable(vpp_reg);
	else
		tegra_fuse_ps18_latch_clear();

	tegra30_fuse_sense();

	if (fuse_info->need_sense_done)
		wait_for_sense_done();

err_reg:

	/* Disable software writes to fuse registers */
	writel_relaxed(0x1, fuse_base + FUSE_WRITE_ACCESS);

err_pgm_disabled:

	clk_disable_unprepare(fuse_clk);

	return result;
}

static void tegra30_fuse_get_uid(uint32_t id_size, void *serial_no)
{
	uint32_t	uid[4];
	uint32_t	vendor;
	uint32_t	fab;
	uint32_t	wafer;
	uint32_t	lot0;
	uint32_t	lot1;
	uint32_t	x;
	uint32_t	y;
	uint32_t	rsvd1;
	uint32_t	reg;

	reg = tegra30_fuse_readl(FUSE_VENDOR_CODE);
	vendor = reg & OPT_VENDOR_CODE_MASK;

	reg = tegra30_fuse_readl(FUSE_FAB_CODE);
	fab = reg & OPT_FAB_CODE_MASK;

	lot0 = tegra30_fuse_readl(FUSE_LOT_CODE_0);

	reg = tegra30_fuse_readl(FUSE_LOT_CODE_1);
	lot1 = reg & OPT_LOT_CODE_1_MASK;

	reg = tegra30_fuse_readl(FUSE_WAFER_ID);
	wafer = reg & OPT_WAFER_ID_MASK;

	reg = tegra30_fuse_readl(FUSE_X_COORDINATE);
	x = reg & OPT_X_COORDINATE_MASK;

	reg = tegra30_fuse_readl(FUSE_Y_COORDINATE);
	y = reg & OPT_Y_COORDINATE_MASK;

	reg = tegra30_fuse_readl(FUSE_OPT_OPS_RESERVED);
	rsvd1 = reg & OPT_OPS_RESERVED_MASK;

	reg = 0;
	reg |= rsvd1 & ECID_ECID0_0_RSVD1_MASK;
	reg |= (y & ECID_ECID0_0_Y_MASK) << ECID_ECID0_0_Y_RANGE;
	reg |= (x & ECID_ECID0_0_X_MASK) << ECID_ECID0_0_X_RANGE;
	reg |= (wafer & ECID_ECID0_0_WAFER_MASK) << ECID_ECID0_0_WAFER_RANGE;
	reg |= (lot1 & ECID_ECID0_0_LOT1_MASK) << ECID_ECID0_0_LOT1_RANGE;
	uid[0] = reg;

	lot1 >>= 2;

	reg = 0;
	reg |= lot1 & ECID_ECID1_0_LOT1_MASK;
	reg |= (lot0 & ECID_ECID1_0_LOT0_MASK) << ECID_ECID1_0_LOT0_RANGE;
	uid[1] = reg;

	lot0 >>= 6;

	reg = 0;
	reg |= lot0 & ECID_ECID2_0_LOT0_MASK;
	reg |= (fab & ECID_ECID2_0_FAB_MASK) << ECID_ECID2_0_FAB_RANGE;
	uid[2] = reg;

	reg = 0;
	reg |= vendor & ECID_ECID3_0_VENDOR_MASK;
	uid[3] = reg;

	memcpy(serial_no, &uid, id_size);
}

static ssize_t tegra30_fuse_uid_show(struct device *child,
		struct device_attribute *attr, char *buf)
{
	tegra30_fuse_get_uid(FUSE_UID_SIZE, buf);
	return FUSE_UID_SIZE;
}

static struct device_attribute tegra30_fuse_uid = {
	.attr = { .name = "uid", .mode = S_IRUGO, },
	.show = tegra30_fuse_uid_show,
};

static int tegra30_fuse_create_uid_sysfs(struct device *dev)
{
	return device_create_file(dev, &tegra30_fuse_uid);
}

static struct tegra_fuse_info tegra30_info = {
	.size			= 0x2a4,
	.spare_bit		= 0x144,
	.speedo_idx		= SPEEDO_TEGRA30,
	.need_sense_done	= true,
	.ext_regulator		= true,
	.fuse_array		= tegra30_fuse_array,
	.fuse_array_size	= ARRAY_SIZE(tegra30_fuse_array),
};

static struct tegra_fuse_info tegra114_info = {
	.size			= 0x2a0,
	.speedo_idx		= SPEEDO_TEGRA114,
	.ext_regulator		= true,
	.fuse_array		= tegra114_fuse_array,
	.fuse_array_size	= ARRAY_SIZE(tegra114_fuse_array),
};

static struct tegra_fuse_info tegra124_info = {
	.size			= 0x300,
	.speedo_idx		= SPEEDO_TEGRA124,
	.ext_regulator		= true,
	.fuse_array		= tegra124_fuse_array,
	.fuse_array_size	= ARRAY_SIZE(tegra124_fuse_array),
};

static struct tegra_fuse_info tegra210_info = {
	.size			= 0x300,
	.spare_bit		= 0x280,
	.speedo_idx		= SPEEDO_TEGRA210,
	.need_sense_done	= true,
	.power_down_mode	= true,
	.fuse_array		= tegra210_fuse_array,
	.fuse_array_size	= ARRAY_SIZE(tegra210_fuse_array),
};

static const struct of_device_id tegra30_fuse_of_match[] = {
	{ .compatible = "nvidia,tegra30-efuse", .data = &tegra30_info },
	{ .compatible = "nvidia,tegra114-efuse", .data = &tegra114_info },
	{ .compatible = "nvidia,tegra124-efuse", .data = &tegra124_info },
	{ .compatible = "nvidia,tegra210-efuse", .data = &tegra210_info },
	{},
};

static int tegra30_fuse_probe(struct platform_device *pdev)
{
	const struct of_device_id *of_dev_id;
	struct clk *osc_clk;
	u32 reg;

	of_dev_id = of_match_device(tegra30_fuse_of_match, &pdev->dev);
	if (!of_dev_id)
		return -ENODEV;

	fuse_clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(fuse_clk)) {
		dev_err(&pdev->dev, "missing clock");
		return PTR_ERR(fuse_clk);
	}

	osc_clk = devm_clk_get(&pdev->dev, "clk_m");
	if (IS_ERR(osc_clk)) {
		/* Should be impossible to see this */
		dev_err(&pdev->dev, "failed to get clk_m");
		return PTR_ERR(osc_clk);
	}

	/*
	 * The strobe programming pulse is 12us based on the osc clock
	 * frequency
	 */
	pgm_cycles = DIV_ROUND_UP(clk_get_rate(osc_clk) * 12, 1000 * 1000);
	pgm_cycles &= FUSETIME_PGM2_TWIDTH_PGM_MASK;
	dev_dbg(&pdev->dev, "pgm_cycles is %d\n", pgm_cycles);

	if (fuse_info->power_down_mode) {
		/* Disable power down mode if enabled */
		reg = readl_relaxed(fuse_base + FUSE_CTRL);
		if (FUSE_CTRL_PD & reg) {
			reg &= ~FUSE_CTRL_PD;
			writel_relaxed(reg, fuse_base + FUSE_CTRL);
		}
	}

	platform_set_drvdata(pdev, NULL);

	fuse_pdev = pdev;

	if (tegra_fuse_create_sysfs(&pdev->dev, fuse_info->size,
				    tegra30_fuse_readl, tegra30_fuse_program))
		return -ENODEV;

	if (tegra30_fuse_create_uid_sysfs(&pdev->dev))
		return -ENODEV;

	dev_dbg(&pdev->dev, "loaded\n");

	return 0;
}

static struct platform_driver tegra30_fuse_driver = {
	.probe = tegra30_fuse_probe,
	.driver = {
		.name = "tegra_fuse",
		.owner = THIS_MODULE,
		.of_match_table = tegra30_fuse_of_match,
	}
};

static int __init tegra30_fuse_init(void)
{
	return platform_driver_register(&tegra30_fuse_driver);
}
postcore_initcall(tegra30_fuse_init);

/* Early boot code. This code is called before the devices are created */

typedef void (*speedo_f)(struct tegra_sku_info *sku_info);

static speedo_f __initdata speedo_tbl[] = {
	[SPEEDO_TEGRA30]	= tegra30_init_speedo_data,
	[SPEEDO_TEGRA114]	= tegra114_init_speedo_data,
	[SPEEDO_TEGRA124]	= tegra124_init_speedo_data,
	[SPEEDO_TEGRA210]	= tegra210_init_speedo_data,
};

static void __init tegra30_fuse_add_randomness(void)
{
	u32 randomness[12];

	randomness[0] = tegra_sku_info.sku_id;
	randomness[1] = tegra_read_straps();
	randomness[2] = tegra_read_chipid();
	randomness[3] = tegra_sku_info.cpu_process_id << 16;
	randomness[3] |= tegra_sku_info.core_process_id;
	randomness[4] = tegra_sku_info.cpu_speedo_id << 16;
	randomness[4] |= tegra_sku_info.soc_speedo_id;
	randomness[5] = tegra30_fuse_readl(FUSE_VENDOR_CODE);
	randomness[6] = tegra30_fuse_readl(FUSE_FAB_CODE);
	randomness[7] = tegra30_fuse_readl(FUSE_LOT_CODE_0);
	randomness[8] = tegra30_fuse_readl(FUSE_LOT_CODE_1);
	randomness[9] = tegra30_fuse_readl(FUSE_WAFER_ID);
	randomness[10] = tegra30_fuse_readl(FUSE_X_COORDINATE);
	randomness[11] = tegra30_fuse_readl(FUSE_Y_COORDINATE);

	add_device_randomness(randomness, sizeof(randomness));
}

static void __init legacy_fuse_init(void)
{
	switch (tegra_get_chip_id()) {
	case TEGRA30:
		fuse_info = &tegra30_info;
		break;
	case TEGRA114:
		fuse_info = &tegra114_info;
		break;
	case TEGRA124:
		fuse_info = &tegra124_info;
		break;
	case TEGRA210:
		fuse_info = &tegra210_info;
		break;
	default:
		return;
	}

	fuse_base = ioremap(TEGRA_FUSE_BASE, TEGRA_FUSE_SIZE);
}

bool __init tegra30_spare_fuse(int spare_bit)
{
	u32 offset = fuse_info->spare_bit + spare_bit * 4;

	return tegra30_fuse_readl(offset) & 1;
}

void __init tegra30_init_fuse_early(void)
{
	struct device_node *np;
	const struct of_device_id *of_match;

	np = of_find_matching_node_and_match(NULL, tegra30_fuse_of_match,
						&of_match);
	if (np) {
		fuse_base = of_iomap(np, 0);
		fuse_info = (struct tegra_fuse_info *)of_match->data;
	} else
		legacy_fuse_init();

	if (!fuse_base) {
		pr_warn("fuse DT node missing and unknown chip id: 0x%02x\n",
			tegra_get_chip_id());
		return;
	}

	tegra_init_revision();
	speedo_tbl[fuse_info->speedo_idx](&tegra_sku_info);
	tegra30_fuse_add_randomness();
}
