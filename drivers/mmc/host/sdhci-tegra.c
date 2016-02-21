/*
 * Copyright (C) 2010 Google, Inc.
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

#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/slot-gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/pm_runtime.h>

#include "sdhci-pltfm.h"

/* Tegra SDHOST controller vendor register definitions */
#define SDHCI_TEGRA_VENDOR_CLK_CTRL		0x100
#define SDHCI_CLK_CTRL_SDMMC_CLK		0x1
#define SDHCI_CLK_CTRL_INPUT_IO_CLK		0x2
#define SDHCI_CLK_CTRL_SPI_MODE_CLKEN_OVERRIDE	0x4
#define SDHCI_CLK_CTRL_PADPIPE_CLKEN_OVERRIDE	0x8
#define SDHCI_CLK_CTRL_TAP_VAL_SHIFT		16
#define SDHCI_CLK_CTRL_TAP_VAL_MASK		0xff
#define SDHCI_CLK_CTRL_TRIM_VAL_SHIFT		24
#define SDHCI_CLK_CTRL_TRIM_VAL_MASK		0x1f

#define SDHCI_VNDR_SYS_SW_CTRL			0x104
#define SDHCI_VNDR_SYS_SW_CTRL_WR_CRC_TMCLK	0x40000000

#define SDHCI_TEGRA_VENDOR_MISC_CTRL		0x120
#define SDHCI_MISC_CTRL_INFINITE_ERASE_TIMEOUT	0x1
#define SDHCI_MISC_CTRL_ENABLE_SDR104		0x8
#define SDHCI_MISC_CTRL_ENABLE_SDR50		0x10
#define SDHCI_MISC_CTRL_ENABLE_SDHCI_SPEC_300	0x20
#define SDHCI_MISC_CTRL_ENABLE_DDR50		0x200
#define SDHCI_MISC_CTRL_EN_EXT_LOOPBACK		0x20000

#define SDMMC_VNDR_IO_TRIM_CTRL			0x1ac
#define SDMMC_VNDR_IO_TRIM_CTRL_SEL_VREG	0x4

#define SDHCI_VNDR_TUN_CTRL0			0x1c0
#define SDHCI_VNDR_TUN_CTRL0_MUL_M_MASK		0x7f
#define SDHCI_VNDR_TUN_CTRL0_MUL_M_SHIFT	6
#define SDHCI_VNDR_TUN_CTRL0_MUL_M_VAL		0x1
#define SDHCI_VNDR_TUN_CTRL0_RETUNE_REQ_EN	0x8000000
#define SDHCI_VNDR_TUN_CTRL0_TUN_ITERATIONS_MASK	0x7
#define SDHCI_VNDR_TUN_CTRL0_TUN_ITERATIONS_SHIFT	13
#define SDHCI_VNDR_TUN_CTRL0_TUN_ITERATIONS_64		1
#define SDHCI_VNDR_TUN_CTRL0_TUN_ITERATIONS_128		2
#define SDHCI_VNDR_TUN_CTRL0_TUN_ITERATIONS_256		4
#define SDHCI_VNDR_TUN_CTRL0_TAP_VAL_UPDATED_BY_HW	0x20000

#define SDHCI_VNDR_TUN_CTRL1			0x1c4
#define SDHCI_VNDR_TUN_CTRL1_TUN_STEP_SIZE_MASK	0x77

#define SDHCI_PAD_CTRL				0x1e0
#define SDHCI_PAD_CTRL_VREF_SEL_MASK		0xf
#define SDHCI_PAD_CTRL_PAD_E_INPUT_OR_PWRD	0x80000000

#define SDHCI_PAD_CTRL_VREF_BDSDMEM_3V3		0x1
#define SDHCI_PAD_CTRL_VREF_BDSDMEM_1V8		0x2
#define SDHCI_PAD_CTRL_VREF_DEFAULT		0x7

#define SDHCI_AUTO_CAL_CONFIG			0x1e4
#define SDHCI_AUTO_CAL_CONFIG_START		0x80000000
#define SDHCI_AUTO_CAL_CONFIG_ENABLE		0x20000000
#define SDHCI_AUTO_CAL_CONFIG_PD_OFFSET_SHIFT	8
#define SDHCI_AUTO_CAL_CONFIG_PU_OFFSET_SHIFT	0

#define SDHCI_AUTO_CAL_STATUS			0x1ec
#define SDHCI_AUTO_CAL_STATUS_ACTIVE		0x80000000

#define SDHCI_IO_SPARE				0x1f0
#define SDHCI_IO_SPARE_BIT_19			0x80000

#define NVQUIRK_FORCE_SDHCI_SPEC_200	BIT(0)
#define NVQUIRK_ENABLE_BLOCK_GAP_DET	BIT(1)
#define NVQUIRK_ENABLE_SDHCI_SPEC_300	BIT(2)
#define NVQUIRK_DISABLE_SDR50		BIT(3)
#define NVQUIRK_DISABLE_SDR104		BIT(4)
#define NVQUIRK_DISABLE_DDR50		BIT(5)
#define NVQUIRK_ADD_AUTOCAL_DELAY	BIT(6)
#define NVQUIRK_SET_PAD_E_INPUT_OR_PWRD BIT(7)
#define NVQUIRK_SELECT_TRIMMER		BIT(8)
#define NVQUIRK_ENABLE_PADPIPE_CLKEN	BIT(9)
#define NVQUIRK_DISABLE_SPI_MODE_CLKEN	BIT(10)
#define NVQUIRK_EN_FEEDBACK_CLK		BIT(11)
#define NVQUIRK_USE_HW_TUNING		BIT(12)
#define NVQUIRK_TMCLK_WR_CRC_TIMEOUT	BIT(13)
#define NVQUIRK_DISABLE_EXT_LOOPBACK	BIT(14)
#define NVQUIRK_IO_SPARE_BIT		BIT(15)
#define NVQUIRK_INFINITE_ERASE_TIMEOUT	BIT(16)

#define TEGRA_SDHCI_AUTOSUSPEND_DELAY	1500

struct sdhci_tegra_soc_data {
	const struct sdhci_pltfm_data *pdata;
	u32 nvquirks;
};

struct sdhci_tegra {
	const struct sdhci_tegra_soc_data *soc_data;
	struct gpio_desc *power_gpio;
	u32 pu_1v8_offset;
	u32 pd_1v8_offset;
	u32 pu_3v3_offset;
	u32 pd_3v3_offset;
	bool use_bdsdmem_pads;
	u32 trim_delay;
	u32 tap_delay;
};

static u16 sdhci_tegra_readw(struct sdhci_host *host, int reg)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	const struct sdhci_tegra_soc_data *soc_data = tegra_host->soc_data;

	if (unlikely((soc_data->nvquirks & NVQUIRK_FORCE_SDHCI_SPEC_200) &&
			(reg == SDHCI_HOST_VERSION))) {
		/* Erratum: Version register is invalid in HW. */
		return SDHCI_SPEC_200;
	}

	return readw(host->ioaddr + reg);
}

static void sdhci_tegra_writew(struct sdhci_host *host, u16 val, int reg)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);

	switch (reg) {
	case SDHCI_TRANSFER_MODE:
		/*
		 * Postpone this write, we must do it together with a
		 * command write that is down below.
		 */
		pltfm_host->xfer_mode_shadow = val;
		return;
	case SDHCI_COMMAND:
		writel((val << 16) | pltfm_host->xfer_mode_shadow,
			host->ioaddr + SDHCI_TRANSFER_MODE);
		return;
	}

	writew(val, host->ioaddr + reg);
}

static void sdhci_tegra_writel(struct sdhci_host *host, u32 val, int reg)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	const struct sdhci_tegra_soc_data *soc_data = tegra_host->soc_data;

	/* Seems like we're getting spurious timeout and crc errors, so
	 * disable signalling of them. In case of real errors software
	 * timers should take care of eventually detecting them.
	 */
	if (unlikely(reg == SDHCI_SIGNAL_ENABLE))
		val &= ~(SDHCI_INT_TIMEOUT|SDHCI_INT_CRC);

	writel(val, host->ioaddr + reg);

	if (unlikely((soc_data->nvquirks & NVQUIRK_ENABLE_BLOCK_GAP_DET) &&
			(reg == SDHCI_INT_ENABLE))) {
		/* Erratum: Must enable block gap interrupt detection */
		u8 gap_ctrl = readb(host->ioaddr + SDHCI_BLOCK_GAP_CONTROL);
		if (val & SDHCI_INT_CARD_INT)
			gap_ctrl |= 0x8;
		else
			gap_ctrl &= ~0x8;
		writeb(gap_ctrl, host->ioaddr + SDHCI_BLOCK_GAP_CONTROL);
	}
}

static unsigned int sdhci_tegra_get_ro(struct sdhci_host *host)
{
	return mmc_gpio_get_ro(host->mmc);
}

static void sdhci_tegra_set_trim_sel_vreg(struct sdhci_host *host, bool enable)
{
	unsigned int wait_usecs;
	u32 misc_ctrl, val;

	val = sdhci_readl(host, SDHCI_TEGRA_VENDOR_CLK_CTRL);
	val &= ~SDHCI_CLK_CTRL_SDMMC_CLK;
	sdhci_writel(host, val, SDHCI_TEGRA_VENDOR_CLK_CTRL);

	misc_ctrl = sdhci_readl(host, SDMMC_VNDR_IO_TRIM_CTRL);
	if (enable) {
		misc_ctrl &= ~SDMMC_VNDR_IO_TRIM_CTRL_SEL_VREG;
		wait_usecs = 3;
	} else {
		misc_ctrl |= SDMMC_VNDR_IO_TRIM_CTRL_SEL_VREG;
		wait_usecs = 1;
	}
	sdhci_writel(host, misc_ctrl, SDMMC_VNDR_IO_TRIM_CTRL);
	udelay(wait_usecs);

	val = sdhci_readl(host, SDHCI_TEGRA_VENDOR_CLK_CTRL);
	val |= SDHCI_CLK_CTRL_SDMMC_CLK;
	sdhci_writel(host, val, SDHCI_TEGRA_VENDOR_CLK_CTRL);

	sdhci_reset(host, SDHCI_RESET_CMD | SDHCI_RESET_DATA);
}

static int sdhci_tegra_clk_enable(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	u32 val;
	int ret;

	ret = clk_prepare_enable(pltfm_host->clk);
	if (ret < 0)
		return ret;

	val = sdhci_readl(host, SDHCI_TEGRA_VENDOR_CLK_CTRL);
	val |= SDHCI_CLK_CTRL_SDMMC_CLK;
	sdhci_writel(host, val, SDHCI_TEGRA_VENDOR_CLK_CTRL);

	return 0;
}

static void sdhci_tegra_clk_disable(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	u32 val;

	val = sdhci_readl(host, SDHCI_TEGRA_VENDOR_CLK_CTRL);
	val &= ~SDHCI_CLK_CTRL_SDMMC_CLK;
	sdhci_writel(host, val, SDHCI_TEGRA_VENDOR_CLK_CTRL);

	clk_disable_unprepare(pltfm_host->clk);
}

static int sdhci_tegra_get_max_tuning_iterations(struct sdhci_host *sdhci)
{
	u16 hw_tuning_iterations;
	u32 vendor_ctrl;
	int ret;

	switch (sdhci->mmc->ios.timing) {
	case MMC_TIMING_UHS_SDR50:
		hw_tuning_iterations = SDHCI_VNDR_TUN_CTRL0_TUN_ITERATIONS_256;
		ret = 256;
		break;
	case MMC_TIMING_UHS_SDR104:
	case MMC_TIMING_MMC_HS200:
	case MMC_TIMING_MMC_HS400:
	default:
		hw_tuning_iterations = SDHCI_VNDR_TUN_CTRL0_TUN_ITERATIONS_128;
		ret = 128;
		break;
	}

	vendor_ctrl = sdhci_readl(sdhci, SDHCI_VNDR_TUN_CTRL0);
	vendor_ctrl &= ~(SDHCI_VNDR_TUN_CTRL0_TUN_ITERATIONS_MASK <<
			 SDHCI_VNDR_TUN_CTRL0_TUN_ITERATIONS_SHIFT);
	vendor_ctrl |= (hw_tuning_iterations <<
			SDHCI_VNDR_TUN_CTRL0_TUN_ITERATIONS_SHIFT);
	sdhci_writel(sdhci, vendor_ctrl, SDHCI_VNDR_TUN_CTRL0);

	return ret;
}

static int sdhci_tegra_do_calibration(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	const struct sdhci_tegra_soc_data *soc_data = tegra_host->soc_data;
	u32 val, pu_offset, pd_offset, vref_sel;
	unsigned long timeout;

	vref_sel = SDHCI_PAD_CTRL_VREF_DEFAULT;
	switch (host->mmc->ios.signal_voltage) {
	case MMC_SIGNAL_VOLTAGE_330:
		pu_offset = tegra_host->pu_3v3_offset;
		pd_offset = tegra_host->pd_3v3_offset;
		if (tegra_host->use_bdsdmem_pads)
			vref_sel = SDHCI_PAD_CTRL_VREF_BDSDMEM_3V3;
		break;
	case MMC_SIGNAL_VOLTAGE_180:
		pu_offset = tegra_host->pu_1v8_offset;
		pd_offset = tegra_host->pd_1v8_offset;
		if (tegra_host->use_bdsdmem_pads)
			vref_sel = SDHCI_PAD_CTRL_VREF_BDSDMEM_1V8;
		break;
	default:
		return -EINVAL;
	}

	if (soc_data->nvquirks & NVQUIRK_SELECT_TRIMMER)
		sdhci_tegra_set_trim_sel_vreg(host, true);

	val = sdhci_readl(host, SDHCI_PAD_CTRL);
	val &= ~SDHCI_PAD_CTRL_VREF_SEL_MASK;
	if (soc_data->nvquirks & NVQUIRK_SET_PAD_E_INPUT_OR_PWRD)
		val |= SDHCI_PAD_CTRL_PAD_E_INPUT_OR_PWRD;
	val |= vref_sel;
	sdhci_writel(host, val, SDHCI_PAD_CTRL);

	if (soc_data->nvquirks & NVQUIRK_ADD_AUTOCAL_DELAY)
		udelay(1);

	/* Enable auto calibration */
	val = sdhci_readl(host, SDHCI_AUTO_CAL_CONFIG);
	val |= SDHCI_AUTO_CAL_CONFIG_ENABLE;
	val |= SDHCI_AUTO_CAL_CONFIG_START;
	if (pu_offset) {
		val &= ~(0x7f << SDHCI_AUTO_CAL_CONFIG_PU_OFFSET_SHIFT);
		val |= pu_offset << SDHCI_AUTO_CAL_CONFIG_PU_OFFSET_SHIFT;
	}
	if (pd_offset) {
		val &= ~(0x7f << SDHCI_AUTO_CAL_CONFIG_PD_OFFSET_SHIFT);
		val |= pd_offset << SDHCI_AUTO_CAL_CONFIG_PD_OFFSET_SHIFT;
	}
	sdhci_writel(host, val, SDHCI_AUTO_CAL_CONFIG);

	/* Wait until the calibration is done */
	timeout = 10;
	while (sdhci_readl(host, SDHCI_AUTO_CAL_STATUS) &
	       SDHCI_AUTO_CAL_STATUS_ACTIVE) {
		if (timeout == 0)
			return -ETIMEDOUT;

		timeout--;
		mdelay(1);
	}

	if (soc_data->nvquirks & NVQUIRK_ADD_AUTOCAL_DELAY)
		udelay(1);

	if (soc_data->nvquirks & NVQUIRK_SET_PAD_E_INPUT_OR_PWRD) {
		val = sdhci_readl(host, SDHCI_PAD_CTRL);
		val &= ~SDHCI_PAD_CTRL_PAD_E_INPUT_OR_PWRD;
		sdhci_writel(host, val, SDHCI_PAD_CTRL);
	}

	return 0;
}

static void sdhci_tegra_set_tap_delay(struct sdhci_host *host,
				      unsigned int uhs)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	bool clk_on;
	u32 val;

	switch (uhs) {
	case MMC_TIMING_UHS_SDR104:
	case MMC_TIMING_MMC_HS200:
	case MMC_TIMING_MMC_HS400:
		/* Tap value set during tuning procedure. */
		return;
	case MMC_TIMING_UHS_SDR50:
		if (host->flags & SDHCI_SDR50_NEEDS_TUNING)
			return;
	default:
		break;
	}

	clk_on = !!(sdhci_readw(host, SDHCI_CLOCK_CONTROL) &
		    SDHCI_CLOCK_CARD_EN);
	if ((host->quirks2 & SDHCI_QUIRK2_TUNING_CLOCK_OFF) && clk_on) {
		val = sdhci_readw(host, SDHCI_CLOCK_CONTROL);
		val &= ~SDHCI_CLOCK_CARD_EN;
		sdhci_writew(host, val, SDHCI_CLOCK_CONTROL);
	}

	if (tegra_host->soc_data->nvquirks & NVQUIRK_USE_HW_TUNING) {
		val = sdhci_readl(host, SDHCI_VNDR_TUN_CTRL0);
		val &= ~SDHCI_VNDR_TUN_CTRL0_TAP_VAL_UPDATED_BY_HW;
		sdhci_writel(host, val, SDHCI_VNDR_TUN_CTRL0);
	}

	val = sdhci_readl(host, SDHCI_TEGRA_VENDOR_CLK_CTRL);
	val &= ~(SDHCI_CLK_CTRL_TAP_VAL_MASK << SDHCI_CLK_CTRL_TAP_VAL_SHIFT);
	val |= tegra_host->tap_delay << SDHCI_CLK_CTRL_TAP_VAL_SHIFT;
	sdhci_writel(host, val, SDHCI_TEGRA_VENDOR_CLK_CTRL);

	if (tegra_host->soc_data->nvquirks & NVQUIRK_USE_HW_TUNING) {
		val = sdhci_readl(host, SDHCI_VNDR_TUN_CTRL0);
		val |= SDHCI_VNDR_TUN_CTRL0_TAP_VAL_UPDATED_BY_HW;
		sdhci_writel(host, val, SDHCI_VNDR_TUN_CTRL0);
	}

	if (host->quirks2 & SDHCI_QUIRK2_TUNING_CLOCK_OFF) {
		udelay(1);
		sdhci_reset(host, SDHCI_RESET_CMD | SDHCI_RESET_DATA);
		if (clk_on) {
			val = sdhci_readw(host, SDHCI_CLOCK_CONTROL);
			val &= ~SDHCI_CLOCK_CARD_EN;
			sdhci_writew(host, val, SDHCI_CLOCK_CONTROL);
		}
	}
}

static void sdhci_tegra_set_uhs_signaling(struct sdhci_host *host,
					  unsigned int uhs)
{
	int ret;

	ret = sdhci_tegra_do_calibration(host);
	if (ret < 0) {
		dev_err(mmc_dev(host->mmc), "Auto calibration failed: %d\n",
			ret);
	}
	sdhci_set_uhs_signaling(host, uhs);
	sdhci_tegra_set_tap_delay(host, uhs);
}

static void sdhci_tegra_reset(struct sdhci_host *host, u8 mask)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	const struct sdhci_tegra_soc_data *soc_data = tegra_host->soc_data;
	u32 misc_ctrl, vendor_ctrl;

	sdhci_reset(host, mask);

	if (!(mask & SDHCI_RESET_ALL))
		return;

	if (soc_data->nvquirks & NVQUIRK_SELECT_TRIMMER)
		sdhci_tegra_set_trim_sel_vreg(host, true);

	misc_ctrl = sdhci_readw(host, SDHCI_TEGRA_VENDOR_MISC_CTRL);
	/* Erratum: Enable SDHCI spec v3.00 support */
	if (soc_data->nvquirks & NVQUIRK_ENABLE_SDHCI_SPEC_300)
		misc_ctrl |= SDHCI_MISC_CTRL_ENABLE_SDHCI_SPEC_300;
	/* Don't advertise UHS modes which aren't supported yet */
	if (soc_data->nvquirks & NVQUIRK_DISABLE_SDR50)
		misc_ctrl &= ~SDHCI_MISC_CTRL_ENABLE_SDR50;
	if (soc_data->nvquirks & NVQUIRK_DISABLE_DDR50)
		misc_ctrl &= ~SDHCI_MISC_CTRL_ENABLE_DDR50;
	if (soc_data->nvquirks & NVQUIRK_DISABLE_SDR104)
		misc_ctrl &= ~SDHCI_MISC_CTRL_ENABLE_SDR104;
	if (soc_data->nvquirks & NVQUIRK_INFINITE_ERASE_TIMEOUT)
		misc_ctrl |= SDHCI_MISC_CTRL_INFINITE_ERASE_TIMEOUT;
	if (soc_data->nvquirks & NVQUIRK_DISABLE_EXT_LOOPBACK)
		misc_ctrl &= ~SDHCI_MISC_CTRL_EN_EXT_LOOPBACK;
	sdhci_writel(host, misc_ctrl, SDHCI_TEGRA_VENDOR_MISC_CTRL);

	vendor_ctrl = sdhci_readl(host, SDHCI_TEGRA_VENDOR_CLK_CTRL);
	if (soc_data->nvquirks & NVQUIRK_ENABLE_PADPIPE_CLKEN)
		vendor_ctrl &= ~SDHCI_CLK_CTRL_PADPIPE_CLKEN_OVERRIDE;
	if (soc_data->nvquirks & NVQUIRK_DISABLE_SPI_MODE_CLKEN)
		vendor_ctrl &= ~SDHCI_CLK_CTRL_SPI_MODE_CLKEN_OVERRIDE;
	if (soc_data->nvquirks & NVQUIRK_EN_FEEDBACK_CLK)
		vendor_ctrl &= ~SDHCI_CLK_CTRL_INPUT_IO_CLK;
	else
		vendor_ctrl |= SDHCI_CLK_CTRL_INPUT_IO_CLK;
	sdhci_writel(host, vendor_ctrl, SDHCI_TEGRA_VENDOR_CLK_CTRL);

	if (soc_data->nvquirks & NVQUIRK_IO_SPARE_BIT) {
		misc_ctrl = sdhci_readl(host, SDHCI_IO_SPARE);
		misc_ctrl |= SDHCI_IO_SPARE_BIT_19;
		sdhci_writel(host, misc_ctrl, SDHCI_IO_SPARE);
	}

	if (soc_data->nvquirks & NVQUIRK_USE_HW_TUNING) {
		vendor_ctrl = sdhci_readl(host, SDHCI_VNDR_TUN_CTRL0);
		vendor_ctrl &= ~(SDHCI_VNDR_TUN_CTRL0_MUL_M_MASK <<
				SDHCI_VNDR_TUN_CTRL0_MUL_M_SHIFT);
		vendor_ctrl |= SDHCI_VNDR_TUN_CTRL0_MUL_M_VAL <<
				SDHCI_VNDR_TUN_CTRL0_MUL_M_SHIFT;
		vendor_ctrl |= SDHCI_VNDR_TUN_CTRL0_RETUNE_REQ_EN;
		sdhci_writel(host, vendor_ctrl, SDHCI_VNDR_TUN_CTRL0);

		vendor_ctrl = sdhci_readl(host, SDHCI_VNDR_TUN_CTRL1);
		vendor_ctrl &= ~SDHCI_VNDR_TUN_CTRL1_TUN_STEP_SIZE_MASK;
		sdhci_writel(host, vendor_ctrl, SDHCI_VNDR_TUN_CTRL1);
	}

	vendor_ctrl = sdhci_readl(host, SDHCI_TEGRA_VENDOR_CLK_CTRL);
	vendor_ctrl &= ~(SDHCI_CLK_CTRL_TRIM_VAL_MASK <<
			SDHCI_CLK_CTRL_TRIM_VAL_SHIFT);
	vendor_ctrl |= tegra_host->trim_delay << SDHCI_CLK_CTRL_TRIM_VAL_SHIFT;
	sdhci_writel(host, vendor_ctrl, SDHCI_TEGRA_VENDOR_CLK_CTRL);

	/* Use timeout clk data timeout counter for generating wr crc status */
	if (soc_data->nvquirks & NVQUIRK_TMCLK_WR_CRC_TIMEOUT) {
		vendor_ctrl = sdhci_readl(host, SDHCI_VNDR_SYS_SW_CTRL);
		vendor_ctrl |= SDHCI_VNDR_SYS_SW_CTRL_WR_CRC_TMCLK;
		sdhci_writel(host, vendor_ctrl, SDHCI_VNDR_SYS_SW_CTRL);
	}
}

static void sdhci_tegra_set_bus_width(struct sdhci_host *host, int bus_width)
{
	u32 ctrl;

	ctrl = sdhci_readb(host, SDHCI_HOST_CONTROL);
	if ((host->mmc->caps & MMC_CAP_8_BIT_DATA) &&
	    (bus_width == MMC_BUS_WIDTH_8)) {
		ctrl &= ~SDHCI_CTRL_4BITBUS;
		ctrl |= SDHCI_CTRL_8BITBUS;
	} else {
		ctrl &= ~SDHCI_CTRL_8BITBUS;
		if (bus_width == MMC_BUS_WIDTH_4)
			ctrl |= SDHCI_CTRL_4BITBUS;
		else
			ctrl &= ~SDHCI_CTRL_4BITBUS;
	}
	sdhci_writeb(host, ctrl, SDHCI_HOST_CONTROL);
}

static int sdhci_tegra_enable_dma(struct sdhci_host *host)
{
	struct device *dev = host->mmc->parent;
	int err;

	if (host->flags & SDHCI_USE_64_BIT_DMA) {
		if (host->quirks2 & SDHCI_QUIRK2_BROKEN_64_BIT_DMA) {
			host->flags &= ~SDHCI_USE_64_BIT_DMA;
			err = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
		} else {
			err = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64));
		}
	} else {
		err = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	}

	return err;
}

static const struct sdhci_ops sdhci_tegra_ops = {
	.get_ro     = sdhci_tegra_get_ro,
	.read_w     = sdhci_tegra_readw,
	.write_l    = sdhci_tegra_writel,
	.set_clock  = sdhci_set_clock,
	.set_bus_width = sdhci_tegra_set_bus_width,
	.reset      = sdhci_tegra_reset,
	.set_uhs_signaling = sdhci_tegra_set_uhs_signaling,
	.get_max_clock = sdhci_pltfm_clk_get_max_clock,
	.enable_dma = sdhci_tegra_enable_dma,
	.get_max_tuning_iterations = sdhci_tegra_get_max_tuning_iterations,
};

static const struct sdhci_pltfm_data sdhci_tegra20_pdata = {
	.quirks = SDHCI_QUIRK_BROKEN_TIMEOUT_VAL |
		  SDHCI_QUIRK_SINGLE_POWER_WRITE |
		  SDHCI_QUIRK_NO_HISPD_BIT |
		  SDHCI_QUIRK_BROKEN_ADMA_ZEROLEN_DESC |
		  SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN,
	.quirks2 = SDHCI_QUIRK2_HOST_OFF_CARD_ON,
	.ops  = &sdhci_tegra_ops,
};

static struct sdhci_tegra_soc_data soc_data_tegra20 = {
	.pdata = &sdhci_tegra20_pdata,
	.nvquirks = NVQUIRK_FORCE_SDHCI_SPEC_200 |
		    NVQUIRK_ENABLE_BLOCK_GAP_DET,
};

static const struct sdhci_pltfm_data sdhci_tegra30_pdata = {
	.quirks = SDHCI_QUIRK_BROKEN_TIMEOUT_VAL |
		  SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK |
		  SDHCI_QUIRK_SINGLE_POWER_WRITE |
		  SDHCI_QUIRK_NO_HISPD_BIT |
		  SDHCI_QUIRK_BROKEN_ADMA_ZEROLEN_DESC |
		  SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN,
	.quirks2 = SDHCI_QUIRK2_HOST_OFF_CARD_ON,
	.ops  = &sdhci_tegra_ops,
};

static struct sdhci_tegra_soc_data soc_data_tegra30 = {
	.pdata = &sdhci_tegra30_pdata,
	.nvquirks = NVQUIRK_ENABLE_SDHCI_SPEC_300 |
		    NVQUIRK_DISABLE_SDR50 |
		    NVQUIRK_DISABLE_SDR104,
};

static const struct sdhci_ops sdhci_tegra114_ops = {
	.get_ro     = sdhci_tegra_get_ro,
	.read_w     = sdhci_tegra_readw,
	.write_w    = sdhci_tegra_writew,
	.write_l    = sdhci_tegra_writel,
	.set_clock  = sdhci_set_clock,
	.set_bus_width = sdhci_tegra_set_bus_width,
	.reset      = sdhci_tegra_reset,
	.set_uhs_signaling = sdhci_set_uhs_signaling,
	.get_max_clock = sdhci_pltfm_clk_get_max_clock,
};

static const struct sdhci_pltfm_data sdhci_tegra114_pdata = {
	.quirks = SDHCI_QUIRK_BROKEN_TIMEOUT_VAL |
		  SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK |
		  SDHCI_QUIRK_SINGLE_POWER_WRITE |
		  SDHCI_QUIRK_NO_HISPD_BIT |
		  SDHCI_QUIRK_BROKEN_ADMA_ZEROLEN_DESC |
		  SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN,
	.quirks2 = SDHCI_QUIRK2_HOST_OFF_CARD_ON,
	.ops  = &sdhci_tegra114_ops,
};

static struct sdhci_tegra_soc_data soc_data_tegra114 = {
	.pdata = &sdhci_tegra114_pdata,
	.nvquirks = NVQUIRK_DISABLE_SDR50 |
		    NVQUIRK_DISABLE_DDR50 |
		    NVQUIRK_DISABLE_SDR104,
};

static const struct sdhci_pltfm_data sdhci_tegra132_pdata = {
	.quirks = SDHCI_QUIRK_BROKEN_TIMEOUT_VAL |
		  SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK |
		  SDHCI_QUIRK_SINGLE_POWER_WRITE |
		  SDHCI_QUIRK_NO_HISPD_BIT |
		  SDHCI_QUIRK_BROKEN_ADMA_ZEROLEN_DESC |
		  SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN,
	.quirks2 = SDHCI_QUIRK2_BROKEN_64_BIT_DMA |
		   SDHCI_QUIRK2_HOST_OFF_CARD_ON,
	.ops  = &sdhci_tegra_ops,
};

static struct sdhci_tegra_soc_data soc_data_tegra132 = {
	.pdata = &sdhci_tegra132_pdata,
	.nvquirks = NVQUIRK_DISABLE_SDR50 |
		    NVQUIRK_DISABLE_DDR50 |
		    NVQUIRK_DISABLE_SDR104,
};

static const struct sdhci_pltfm_data sdhci_tegra210_pdata = {
	.quirks = SDHCI_QUIRK_BROKEN_TIMEOUT_VAL |
		  SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK |
		  SDHCI_QUIRK_SINGLE_POWER_WRITE |
		  SDHCI_QUIRK_NO_HISPD_BIT |
		  SDHCI_QUIRK_BROKEN_ADMA_ZEROLEN_DESC |
		  SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN,
	.quirks2 = SDHCI_QUIRK2_BROKEN_64_BIT_DMA |
		   SDHCI_QUIRK2_HOST_OFF_CARD_ON |
		   SDHCI_QUIRK2_TUNING_CLOCK_OFF |
		   SDHCI_QUIRK2_RESET_ON_TUNE_TIMEOUT,
	.ops  = &sdhci_tegra_ops,
};

static struct sdhci_tegra_soc_data soc_data_tegra210 = {
	.pdata = &sdhci_tegra210_pdata,
	.nvquirks = NVQUIRK_ADD_AUTOCAL_DELAY |
		    NVQUIRK_SET_PAD_E_INPUT_OR_PWRD |
		    NVQUIRK_SELECT_TRIMMER |
		    NVQUIRK_DISABLE_SPI_MODE_CLKEN |
		    NVQUIRK_EN_FEEDBACK_CLK |
		    NVQUIRK_USE_HW_TUNING |
		    NVQUIRK_TMCLK_WR_CRC_TIMEOUT |
		    NVQUIRK_DISABLE_EXT_LOOPBACK |
		    NVQUIRK_IO_SPARE_BIT |
		    NVQUIRK_INFINITE_ERASE_TIMEOUT,
};

static const struct of_device_id sdhci_tegra_dt_match[] = {
	{ .compatible = "nvidia,tegra210-sdhci", .data = &soc_data_tegra210 },
	{ .compatible = "nvidia,tegra132-sdhci", .data = &soc_data_tegra132 },
	{ .compatible = "nvidia,tegra124-sdhci", .data = &soc_data_tegra114 },
	{ .compatible = "nvidia,tegra114-sdhci", .data = &soc_data_tegra114 },
	{ .compatible = "nvidia,tegra30-sdhci", .data = &soc_data_tegra30 },
	{ .compatible = "nvidia,tegra20-sdhci", .data = &soc_data_tegra20 },
	{}
};
MODULE_DEVICE_TABLE(of, sdhci_tegra_dt_match);

static int sdhci_tegra_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;

	of_property_read_u32(np, "nvidia,pu-1v8-offset",
			     &tegra_host->pu_1v8_offset);
	of_property_read_u32(np, "nvidia,pd-1v8-offset",
			     &tegra_host->pd_1v8_offset);
	of_property_read_u32(np, "nvidia,pu-3v3-offset",
			     &tegra_host->pu_3v3_offset);
	of_property_read_u32(np, "nvidia,pd-3v3-offset",
			     &tegra_host->pd_3v3_offset);
	tegra_host->use_bdsdmem_pads =
		of_property_read_bool(np, "nvidia,use-bdsdmem-pads");

	of_property_read_u32(np, "nvidia,trim-delay",
			     &tegra_host->trim_delay);
	of_property_read_u32(np, "nvidia,tap-delay",
			     &tegra_host->tap_delay);

	return mmc_of_parse(host->mmc);
}

static int sdhci_tegra_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	const struct sdhci_tegra_soc_data *soc_data;
	struct sdhci_host *host;
	struct sdhci_pltfm_host *pltfm_host;
	struct sdhci_tegra *tegra_host;
	struct clk *clk;
	int rc;

	match = of_match_device(sdhci_tegra_dt_match, &pdev->dev);
	if (!match)
		return -EINVAL;
	soc_data = match->data;

	host = sdhci_pltfm_init(pdev, soc_data->pdata, 0);
	if (IS_ERR(host))
		return PTR_ERR(host);
	pltfm_host = sdhci_priv(host);

	tegra_host = devm_kzalloc(&pdev->dev, sizeof(*tegra_host), GFP_KERNEL);
	if (!tegra_host) {
		dev_err(mmc_dev(host->mmc), "failed to allocate tegra_host\n");
		rc = -ENOMEM;
		goto err_alloc_tegra_host;
	}
	tegra_host->soc_data = soc_data;
	pltfm_host->priv = tegra_host;

	rc = sdhci_tegra_parse_dt(&pdev->dev);
	if (rc)
		goto err_parse_dt;

	tegra_host->power_gpio = devm_gpiod_get_optional(&pdev->dev, "power",
							 GPIOD_OUT_HIGH);
	if (IS_ERR(tegra_host->power_gpio)) {
		rc = PTR_ERR(tegra_host->power_gpio);
		goto err_power_req;
	}

	clk = devm_clk_get(mmc_dev(host->mmc), NULL);
	if (IS_ERR(clk)) {
		dev_err(mmc_dev(host->mmc), "clk err\n");
		rc = PTR_ERR(clk);
		goto err_clk_get;
	}
	pltfm_host->clk = clk;
	sdhci_tegra_clk_enable(host);

	host->mmc->caps |= MMC_CAP_WAIT_WHILE_BUSY;

	rc = sdhci_add_host(host);
	if (rc)
		goto err_add_host;

	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	pm_runtime_set_autosuspend_delay(&pdev->dev,
					 TEGRA_SDHCI_AUTOSUSPEND_DELAY);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_suspend_ignore_children(&pdev->dev, true);

	return 0;

err_add_host:
	sdhci_tegra_clk_disable(host);
err_clk_get:
err_power_req:
err_parse_dt:
err_alloc_tegra_host:
	sdhci_pltfm_free(pdev);
	return rc;
}

static int sdhci_tegra_remove(struct platform_device *pdev)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);
	int dead = (readl(host->ioaddr + SDHCI_INT_STATUS) == 0xffffffff);

	pm_runtime_get_sync(&pdev->dev);
	sdhci_remove_host(host, dead);
	pm_runtime_disable(&pdev->dev);

	sdhci_tegra_clk_disable(host);

	sdhci_pltfm_free(pdev);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int tegra_sdhci_suspend(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	int ret;

	pm_runtime_get_sync(dev);

	ret = sdhci_suspend_host(host);
	if (!mmc_card_keep_power(host->mmc))
		if (tegra_host->power_gpio)
			gpiod_direction_output(tegra_host->power_gpio, 0);

	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);

	return ret;
}

static int tegra_sdhci_resume(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_tegra *tegra_host = pltfm_host->priv;
	int ret;

	pm_runtime_get_sync(dev);

	if (!mmc_card_keep_power(host->mmc))
		if (tegra_host->power_gpio)
			gpiod_direction_output(tegra_host->power_gpio, 1);
	ret = sdhci_resume_host(host);

	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);

	return ret;
}
#endif

#ifdef CONFIG_PM_RUNTIME
static int tegra_sdhci_runtime_suspend(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	int ret;

	ret = sdhci_runtime_suspend_host(host);
	sdhci_tegra_clk_disable(host);

	return ret;
}

static int tegra_sdhci_runtime_resume(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	int ret;

	sdhci_tegra_clk_enable(host);
	ret = sdhci_runtime_resume_host(host);

	return ret;
}
#endif

static const struct dev_pm_ops sdhci_tegra_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(tegra_sdhci_suspend, tegra_sdhci_resume)
	SET_RUNTIME_PM_OPS(tegra_sdhci_runtime_suspend,
			   tegra_sdhci_runtime_resume, NULL)
};

static struct platform_driver sdhci_tegra_driver = {
	.driver		= {
		.name	= "sdhci-tegra",
		.of_match_table = sdhci_tegra_dt_match,
		.pm	= &sdhci_tegra_pm_ops,
	},
	.probe		= sdhci_tegra_probe,
	.remove		= sdhci_tegra_remove,
};

module_platform_driver(sdhci_tegra_driver);

MODULE_DESCRIPTION("SDHCI driver for Tegra");
MODULE_AUTHOR("Google, Inc.");
MODULE_LICENSE("GPL v2");
