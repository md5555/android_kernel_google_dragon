/*
 * drivers/i2c/busses/i2c-tegra.c
 *
 * Copyright (C) 2010 Google, Inc.
 * Author: Colin Cross <ccross@android.com>
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/completion.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>

#include <soc/tegra/pmc.h>

#include <asm/unaligned.h>

#define TEGRA_I2C_TIMEOUT (msecs_to_jiffies(1000))
#define BYTES_PER_FIFO_WORD 4

#define VI_I2C_REG_SHIFT			2
#define VI_I2C_REG_OFFSET			0xc00

#define I2C_CNFG				0x000
#define I2C_CNFG_DEBOUNCE_CNT_SHIFT		12
#define I2C_CNFG_PACKET_MODE_EN			(1<<10)
#define I2C_CNFG_NEW_MASTER_FSM			(1<<11)
#define I2C_CNFG_MULTI_MASTER_MODE		(1<<17)
#define I2C_STATUS				0x01C
#define I2C_SL_CNFG				0x020
#define I2C_SL_CNFG_NACK			(1<<1)
#define I2C_SL_CNFG_NEWSL			(1<<2)
#define I2C_SL_ADDR1				0x02c
#define I2C_SL_ADDR2				0x030
#define I2C_TLOW_SEXT				0x034
#define I2C_TX_FIFO				0x050
#define I2C_RX_FIFO				0x054
#define I2C_PACKET_TRANSFER_STATUS		0x058
#define I2C_FIFO_CONTROL			0x05c
#define I2C_FIFO_CONTROL_TX_FLUSH		(1<<1)
#define I2C_FIFO_CONTROL_RX_FLUSH		(1<<0)
#define I2C_FIFO_CONTROL_TX_TRIG_SHIFT		5
#define I2C_FIFO_CONTROL_RX_TRIG_SHIFT		2
#define I2C_FIFO_STATUS				0x060
#define I2C_FIFO_STATUS_TX_MASK			0xF0
#define I2C_FIFO_STATUS_TX_SHIFT		4
#define I2C_FIFO_STATUS_RX_MASK			0x0F
#define I2C_FIFO_STATUS_RX_SHIFT		0
#define I2C_INT_MASK				0x064
#define I2C_INT_STATUS				0x068
#define I2C_INT_BUS_CLEAR_DONE			(1<<11)
#define I2C_INT_PACKET_XFER_COMPLETE		(1<<7)
#define I2C_INT_ALL_PACKETS_XFER_COMPLETE	(1<<6)
#define I2C_INT_TX_FIFO_OVERFLOW		(1<<5)
#define I2C_INT_RX_FIFO_UNDERFLOW		(1<<4)
#define I2C_INT_NO_ACK				(1<<3)
#define I2C_INT_ARBITRATION_LOST		(1<<2)
#define I2C_INT_TX_FIFO_DATA_REQ		(1<<1)
#define I2C_INT_RX_FIFO_DATA_REQ		(1<<0)
#define I2C_CLK_DIVISOR				0x06c
#define I2C_CLK_DIVISOR_STD_FAST_MODE_SHIFT	16
#define I2C_CLK_MULTIPLIER_STD_FAST_MODE	8
#define I2C_BUS_CLEAR_CNFG			0x084
#define I2C_INTERFACE_TIMING_0			0x094
#define I2C_TLOW				2
#define I2C_TLOW_SHIFT				0
#define I2C_THIGH				4
#define I2C_THIGH_SHIFT				8
#define I2C_INTERFACE_TIMING_1			0x098
#define I2C_HS_INTERFACE_TIMING_0		0x09C
#define I2C_HS_INTERFACE_TIMING_1		0x0A0

#define DVC_CTRL_REG1				0x000
#define DVC_CTRL_REG1_INTR_EN			(1<<10)
#define DVC_CTRL_REG2				0x004
#define DVC_CTRL_REG3				0x008
#define DVC_CTRL_REG3_SW_PROG			(1<<26)
#define DVC_CTRL_REG3_I2C_DONE_INTR_EN		(1<<30)
#define DVC_STATUS				0x00c
#define DVC_STATUS_I2C_DONE_INTR		(1<<30)

#define I2C_ERR_NONE				0x00
#define I2C_ERR_NO_ACK				0x01
#define I2C_ERR_ARBITRATION_LOST		0x02
#define I2C_ERR_UNKNOWN_INTERRUPT		0x04

#define PACKET_HEADER0_HEADER_SIZE_SHIFT	28
#define PACKET_HEADER0_PACKET_ID_SHIFT		16
#define PACKET_HEADER0_CONT_ID_SHIFT		12
#define PACKET_HEADER0_CONT_ID_MASK		0xF
#define PACKET_HEADER0_PROTOCOL_I2C		(1<<4)

#define I2C_HEADER_HIGHSPEED_MODE		(1<<22)
#define I2C_HEADER_CONT_ON_NAK			(1<<21)
#define I2C_HEADER_SEND_START_BYTE		(1<<20)
#define I2C_HEADER_READ				(1<<19)
#define I2C_HEADER_10BIT_ADDR			(1<<18)
#define I2C_HEADER_IE_ENABLE			(1<<17)
#define I2C_HEADER_REPEAT_START			(1<<16)
#define I2C_HEADER_CONTINUE_XFER		(1<<15)
#define I2C_HEADER_MASTER_ADDR_SHIFT		12
#define I2C_HEADER_SLAVE_ADDR_SHIFT		1

#define I2C_BUS_CLEAR_CNFG			0x084
#define I2C_BC_SCLK_THRESHOLD			(9 << 16)
#define I2C_BC_STOP_COND			(1 << 2)
#define I2C_BC_TERMINATE			(1 << 1)
#define I2C_BC_ENABLE				(1 << 0)

#define I2C_BUS_CLEAR_STATUS			0x088
#define I2C_BC_STATUS				(1 << 0)

#define I2C_CONFIG_LOAD				0x08C
#define I2C_MSTR_CONFIG_LOAD			(1 << 0)
#define I2C_SLV_CONFIG_LOAD			(1 << 1)
#define I2C_TIMEOUT_CONFIG_LOAD			(1 << 2)

#define I2C_CLKEN_OVERRIDE			0x090
#define I2C_MST_CORE_CLKEN_OVR			(1 << 0)

#define DPAUX_HYBRID_PADCTL			0x124
#define DPAUX_MODE_I2C				(1 << 0)
#define DPAUX_I2C_SCL_INPUT_RCV			(1 << 14)
#define DPAUX_I2C_SDA_INPUT_RCV			(1 << 15)

#define DPAUX_HYBRID_SPARE			0x134

#define MAX_BUSCLEAR_CLOCK			(9 * 8 + 1)

/*
 * msg_end_type: The bus control which need to be send at end of transfer.
 * @MSG_END_STOP: Send stop pulse at end of transfer.
 * @MSG_END_REPEAT_START: Send repeat start at end of transfer.
 * @MSG_END_CONTINUE: The following on message is coming and so do not send
 *		stop or repeat start.
 */
enum msg_end_type {
	MSG_END_STOP,
	MSG_END_REPEAT_START,
	MSG_END_CONTINUE,
};

/**
 * struct tegra_i2c_hw_feature : Different HW support on Tegra
 * @has_continue_xfer_support: Continue transfer supports.
 * @has_per_pkt_xfer_complete_irq: Has enable/disable capability for transfer
 *		complete interrupt per packet basis.
 * @has_single_clk_source: The i2c controller has single clock source. Tegra30
 *		and earlier Socs has two clock sources i.e. div-clk and
 *		fast-clk.
 * @has_config_load_reg: Has the config load register to load the new
 *		configuration.
 * @has_regulator: Controller requires extrnal regulator to be powered on
 *		during transfers.
 * @has_powergate: Controller is located inside a powergate partition.
 * @has_hw_arb_support: Controller has hardware support for arbitration lost
 * @is_vi: Identifies the VI i2c controller, has a different register layout,
 *		and needs more clocks.
 * @is_dpaux: Identifies the DPAUX i2c controller, has separate pad control
 *		setings, and is in a different power domain.
 * @powergate: Powergate partition ID, if applicable.
 * @clk_divisor_hs_mode: Clock divisor in HS mode.
 * @clk_divisor_std_fast_mode: Clock divisor in standard/fast mode. It is
 *		applicable if there is no fast clock source i.e. single clock
 *		source.
 * @has_multi_master_mode: Controller supports single/multi master mode. In
 *		single_master mode, no arbitration checks happen.
 * @has_slcg_override_reg: Controller has sclg override register used for
 *		overriding clk enable for multimaster busses.
 * @rx_fifo_trig: Receive FIFO trigger level
 */

struct tegra_i2c_hw_feature {
	bool has_continue_xfer_support;
	bool has_per_pkt_xfer_complete_irq;
	bool has_single_clk_source;
	bool has_config_load_reg;
	bool has_regulator;
	bool has_powergate;
	bool has_hw_arb_support;
	bool is_vi;
	bool is_dpaux;
	int powergate_id;
	int clk_divisor_hs_mode;
	int clk_divisor_std_fast_mode;
	u16 clk_divisor_fast_plus_mode;
	bool has_multi_master_mode;
	bool has_slcg_override_reg;
	u32 rx_fifo_trig;
};

/**
 * struct tegra_i2c_dev	- per device i2c context
 * @dev: device reference for power management
 * @hw: Tegra i2c hw feature.
 * @adapter: core i2c layer adapter information
 * @div_clk: clock reference for div clock of i2c controller.
 * @fast_clk: clock reference for fast clock of i2c controller.
 * @base: ioremapped registers cookie
 * @cont_id: i2c controller id, used for for packet header
 * @irq: irq number of transfer complete interrupt
 * @is_dvc: identifies the DVC i2c controller, has a different register layout
 * @msg_complete: transfer completion notifier
 * @msg_err: error code for completed message
 * @msg_buf: pointer to current message data
 * @msg_buf_remaining: size of unsent data in the message buffer
 * @msg_read: identifies read transfers
 * @bus_clk_rate: current i2c bus clock rate
 * @is_suspended: prevents i2c controller accesses after suspend is called
 * @is_multimaster_mode: this bus is configured with multiple masters present
 * @sda_gpio: serial data line gpio number
 * @scl_gpio: serial clock line gpio number
 */
struct tegra_i2c_dev {
	struct device *dev;
	const struct tegra_i2c_hw_feature *hw;
	struct i2c_adapter adapter;
	struct clk *div_clk;
	struct clk *fast_clk;
	struct clk *slow_clk;
	bool needs_cl_dvfs_clock;
	struct clk *dvfs_ref_clk;
	struct clk *dvfs_soc_clk;
	struct clk *host1x_clk;
	struct clk *sor_clk;
	struct clk *dpaux_clk;
	struct regulator *reg;
	struct reset_control *rst;
	void __iomem *base;
	void __iomem *dpaux_base;
	int cont_id;
	int irq;
	bool irq_disabled;
	int is_dvc;
	struct completion msg_complete;
	int msg_err;
	u8 *msg_buf;
	size_t msg_buf_remaining;
	int msg_read;
	u32 bus_clk_rate;
	u16 clk_divisor_non_hs_mode;
	bool is_suspended;
	bool is_multimaster_mode;
	int sda_gpio;
	int scl_gpio;
	spinlock_t xfer_lock;
};

static void dvc_writel(struct tegra_i2c_dev *i2c_dev, u32 val, unsigned long reg)
{
	writel(val, i2c_dev->base + reg);
}

static u32 dvc_readl(struct tegra_i2c_dev *i2c_dev, unsigned long reg)
{
	return readl(i2c_dev->base + reg);
}

/*
 * i2c_writel and i2c_readl will offset the register if necessary to talk
 * to the I2C block inside the DVC block
 */
static unsigned long tegra_i2c_reg_addr(struct tegra_i2c_dev *i2c_dev,
	unsigned long reg)
{
	if (i2c_dev->is_dvc)
		reg += (reg >= I2C_TX_FIFO) ? 0x10 : 0x40;
	else if (i2c_dev->hw->is_vi)
		reg = VI_I2C_REG_OFFSET + (reg << VI_I2C_REG_SHIFT);
	return reg;
}

static void i2c_writel(struct tegra_i2c_dev *i2c_dev, u32 val,
	unsigned long reg)
{
	writel(val, i2c_dev->base + tegra_i2c_reg_addr(i2c_dev, reg));

	/* Read back register to make sure that register writes completed */
	if (reg != I2C_TX_FIFO)
		readl(i2c_dev->base + tegra_i2c_reg_addr(i2c_dev, reg));
}

static u32 i2c_readl(struct tegra_i2c_dev *i2c_dev, unsigned long reg)
{
	return readl(i2c_dev->base + tegra_i2c_reg_addr(i2c_dev, reg));
}

static void i2c_writesl(struct tegra_i2c_dev *i2c_dev, void *data,
	unsigned long reg, int len)
{
	u32 *p = data;
	while (len--) {
		writel(*p, i2c_dev->base + tegra_i2c_reg_addr(i2c_dev, reg));
		p++;
	}
}

static void i2c_readsl(struct tegra_i2c_dev *i2c_dev, void *data,
	unsigned long reg, int len)
{
	u32 *p = data;
	while (len--) {
		*p = readl(i2c_dev->base + tegra_i2c_reg_addr(i2c_dev, reg));
		p++;
	}
}

static void tegra_i2c_mask_irq(struct tegra_i2c_dev *i2c_dev, u32 mask)
{
	u32 int_mask = i2c_readl(i2c_dev, I2C_INT_MASK);
	int_mask &= ~mask;
	i2c_writel(i2c_dev, int_mask, I2C_INT_MASK);
}

static void tegra_i2c_unmask_irq(struct tegra_i2c_dev *i2c_dev, u32 mask)
{
	u32 int_mask = i2c_readl(i2c_dev, I2C_INT_MASK);
	int_mask |= mask;
	i2c_writel(i2c_dev, int_mask, I2C_INT_MASK);
}

static int tegra_i2c_flush_fifos(struct tegra_i2c_dev *i2c_dev)
{
	unsigned long timeout = jiffies + HZ;
	u32 val = i2c_readl(i2c_dev, I2C_FIFO_CONTROL);
	val |= I2C_FIFO_CONTROL_TX_FLUSH | I2C_FIFO_CONTROL_RX_FLUSH;
	i2c_writel(i2c_dev, val, I2C_FIFO_CONTROL);

	while (i2c_readl(i2c_dev, I2C_FIFO_CONTROL) &
		(I2C_FIFO_CONTROL_TX_FLUSH | I2C_FIFO_CONTROL_RX_FLUSH)) {
		if (time_after(jiffies, timeout)) {
			dev_warn(i2c_dev->dev, "timeout waiting for fifo flush\n");
			return -ETIMEDOUT;
		}
		msleep(1);
	}
	return 0;
}

static int tegra_i2c_empty_rx_fifo(struct tegra_i2c_dev *i2c_dev)
{
	u32 val;
	int rx_fifo_avail;
	u8 *buf = i2c_dev->msg_buf;
	size_t buf_remaining = i2c_dev->msg_buf_remaining;
	int words_to_transfer;

	val = i2c_readl(i2c_dev, I2C_FIFO_STATUS);
	rx_fifo_avail = (val & I2C_FIFO_STATUS_RX_MASK) >>
		I2C_FIFO_STATUS_RX_SHIFT;

	/* Rounds down to not include partial word at the end of buf */
	words_to_transfer = buf_remaining / BYTES_PER_FIFO_WORD;
	if (words_to_transfer > rx_fifo_avail)
		words_to_transfer = rx_fifo_avail;

	i2c_readsl(i2c_dev, buf, I2C_RX_FIFO, words_to_transfer);

	buf += words_to_transfer * BYTES_PER_FIFO_WORD;
	buf_remaining -= words_to_transfer * BYTES_PER_FIFO_WORD;
	rx_fifo_avail -= words_to_transfer;

	/*
	 * If there is a partial word at the end of buf, handle it manually to
	 * prevent overwriting past the end of buf
	 */
	if (rx_fifo_avail > 0 && buf_remaining > 0) {
		BUG_ON(buf_remaining > 3);
		val = i2c_readl(i2c_dev, I2C_RX_FIFO);
		memcpy(buf, &val, buf_remaining);
		buf_remaining = 0;
		rx_fifo_avail--;
	}

	BUG_ON(rx_fifo_avail > 0 && buf_remaining > 0);
	i2c_dev->msg_buf_remaining = buf_remaining;
	i2c_dev->msg_buf = buf;
	return 0;
}

static int tegra_i2c_fill_tx_fifo(struct tegra_i2c_dev *i2c_dev)
{
	u32 val;
	int tx_fifo_avail;
	u8 *buf = i2c_dev->msg_buf;
	size_t buf_remaining = i2c_dev->msg_buf_remaining;
	int words_to_transfer;

	val = i2c_readl(i2c_dev, I2C_FIFO_STATUS);
	tx_fifo_avail = (val & I2C_FIFO_STATUS_TX_MASK) >>
		I2C_FIFO_STATUS_TX_SHIFT;

	/* Rounds down to not include partial word at the end of buf */
	words_to_transfer = buf_remaining / BYTES_PER_FIFO_WORD;

	/* It's very common to have < 4 bytes, so optimize that case. */
	if (words_to_transfer) {
		if (words_to_transfer > tx_fifo_avail)
			words_to_transfer = tx_fifo_avail;

		/*
		 * Update state before writing to FIFO.  If this casues us
		 * to finish writing all bytes (AKA buf_remaining goes to 0) we
		 * have a potential for an interrupt (PACKET_XFER_COMPLETE is
		 * not maskable).  We need to make sure that the isr sees
		 * buf_remaining as 0 and doesn't call us back re-entrantly.
		 */
		buf_remaining -= words_to_transfer * BYTES_PER_FIFO_WORD;
		tx_fifo_avail -= words_to_transfer;
		i2c_dev->msg_buf_remaining = buf_remaining;
		i2c_dev->msg_buf = buf +
			words_to_transfer * BYTES_PER_FIFO_WORD;
		barrier();

		i2c_writesl(i2c_dev, buf, I2C_TX_FIFO, words_to_transfer);

		buf += words_to_transfer * BYTES_PER_FIFO_WORD;
	}

	/*
	 * If there is a partial word at the end of buf, handle it manually to
	 * prevent reading past the end of buf, which could cross a page
	 * boundary and fault.
	 */
	if (tx_fifo_avail > 0 && buf_remaining > 0) {
		BUG_ON(buf_remaining > 3);
		memcpy(&val, buf, buf_remaining);

		/* Again update before writing to FIFO to make sure isr sees. */
		i2c_dev->msg_buf_remaining = 0;
		i2c_dev->msg_buf = NULL;
		barrier();

		i2c_writel(i2c_dev, val, I2C_TX_FIFO);
	}

	return 0;
}

/*
 * One of the Tegra I2C blocks is inside the DVC (Digital Voltage Controller)
 * block.  This block is identical to the rest of the I2C blocks, except that
 * it only supports master mode, it has registers moved around, and it needs
 * some extra init to get it into I2C mode.  The register moves are handled
 * by i2c_readl and i2c_writel
 */
static void tegra_dvc_init(struct tegra_i2c_dev *i2c_dev)
{
	u32 val = 0;
	val = dvc_readl(i2c_dev, DVC_CTRL_REG3);
	val |= DVC_CTRL_REG3_SW_PROG;
	val |= DVC_CTRL_REG3_I2C_DONE_INTR_EN;
	dvc_writel(i2c_dev, val, DVC_CTRL_REG3);

	val = dvc_readl(i2c_dev, DVC_CTRL_REG1);
	val |= DVC_CTRL_REG1_INTR_EN;
	dvc_writel(i2c_dev, val, DVC_CTRL_REG1);
}

/*
 * One of the Tegra I2C blocks is inside the VI (Video Interface?)
 * block.  This block is identical to the rest of the I2C blocks, except that
 * it only supports master mode, it has registers moved around, and it needs
 * some extra init to get it into I2C mode.  The register moves are handled
 * by i2c_readl and i2c_writel.
 */
static void tegra_vi_init(struct tegra_i2c_dev *i2c_dev)
{
	i2c_writel(i2c_dev, (I2C_TLOW << I2C_TLOW_SHIFT) |
		(I2C_THIGH << I2C_THIGH_SHIFT), I2C_INTERFACE_TIMING_0);
	i2c_writel(i2c_dev, 0x04070404, I2C_INTERFACE_TIMING_1);
	i2c_writel(i2c_dev, 0x308, I2C_HS_INTERFACE_TIMING_0);
	i2c_writel(i2c_dev, 0x0B0B0B, I2C_HS_INTERFACE_TIMING_1);
	i2c_writel(i2c_dev, 0x90004, I2C_BUS_CLEAR_CNFG);
	i2c_writel(i2c_dev, 0x0, I2C_TLOW_SEXT);
}

/*
 * The I2C controller used for DPAUX lives in the SOR partition and has
 * additional pad control and power control registers which must be
 * programmed.
 */
static void tegra_dpaux_init(struct tegra_i2c_dev *i2c_dev)
{
	u32 val;

	val = DPAUX_MODE_I2C | DPAUX_I2C_SCL_INPUT_RCV |
		DPAUX_I2C_SDA_INPUT_RCV;
	writel(val, i2c_dev->dpaux_base + DPAUX_HYBRID_PADCTL);
	writel(0x0, i2c_dev->dpaux_base + DPAUX_HYBRID_SPARE);
}

static inline int tegra_i2c_clock_enable(struct tegra_i2c_dev *i2c_dev)
{
	int ret;
	if (!i2c_dev->hw->has_single_clk_source) {
		ret = clk_enable(i2c_dev->fast_clk);
		if (ret < 0) {
			dev_err(i2c_dev->dev,
				"Enabling fast clk failed, err %d\n", ret);
			return ret;
		}
	}
	ret = clk_enable(i2c_dev->div_clk);
	if (ret < 0) {
		dev_err(i2c_dev->dev,
			"Enabling div clk failed, err %d\n", ret);
		goto err_fast_disable;
	}
	if (i2c_dev->hw->is_vi) {
		ret = clk_prepare_enable(i2c_dev->slow_clk);
		if (ret < 0) {
			dev_err(i2c_dev->dev,
				"Enabling slow clk failed, err %d\n", ret);
			goto err_div_disable;
		}

		ret = clk_prepare_enable(i2c_dev->host1x_clk);
		if (ret < 0) {
			dev_err(i2c_dev->dev,
				"Enabling host1x clk failed, err %d\n", ret);
			goto err_slow_disable;
		}
	}
	if (i2c_dev->hw->is_dpaux) {
		ret = clk_prepare_enable(i2c_dev->dpaux_clk);
		if (ret < 0) {
			dev_err(i2c_dev->dev,
				"Enabling dpaux clk failed, err %d\n", ret);
			goto err_div_disable;
		}

		ret = clk_prepare_enable(i2c_dev->sor_clk);
		if (ret < 0) {
			dev_err(i2c_dev->dev,
				"Enabling sor clk failed, err %d\n", ret);
			goto err_dpaux_disable;
		}
	}

	if (i2c_dev->needs_cl_dvfs_clock) {
		ret = clk_enable(i2c_dev->dvfs_soc_clk);
		if (ret < 0) {
			dev_err(i2c_dev->dev,
				"Error in enabling dvfs soc clock %d\n", ret);
			goto err_dpaux_disable;
		}
		ret = clk_enable(i2c_dev->dvfs_ref_clk);
		if (ret < 0) {
			dev_err(i2c_dev->dev,
				"Error in enabling dvfs ref clock %d\n", ret);
			goto err_ref_clk;
		}
	}

	return 0;

err_ref_clk:
	if (i2c_dev->needs_cl_dvfs_clock)
		clk_disable(i2c_dev->dvfs_soc_clk);

err_dpaux_disable:
	if (i2c_dev->hw->is_dpaux)
		clk_disable_unprepare(i2c_dev->dpaux_clk);
err_slow_disable:
	if (i2c_dev->hw->is_vi)
		clk_disable_unprepare(i2c_dev->slow_clk);
err_div_disable:
	clk_disable(i2c_dev->div_clk);
err_fast_disable:
	if (!i2c_dev->hw->has_single_clk_source)
		clk_disable(i2c_dev->fast_clk);

	return ret;
}

static inline void tegra_i2c_clock_disable(struct tegra_i2c_dev *i2c_dev)
{
	if (i2c_dev->hw->is_dpaux) {
		clk_disable_unprepare(i2c_dev->sor_clk);
		clk_disable_unprepare(i2c_dev->dpaux_clk);
	}
	if (i2c_dev->hw->is_vi) {
		clk_disable_unprepare(i2c_dev->host1x_clk);
		clk_disable_unprepare(i2c_dev->slow_clk);
	}
	clk_disable(i2c_dev->div_clk);
	if (!i2c_dev->hw->has_single_clk_source)
		clk_disable(i2c_dev->fast_clk);
	if (i2c_dev->needs_cl_dvfs_clock) {
		clk_disable(i2c_dev->dvfs_soc_clk);
		clk_disable(i2c_dev->dvfs_ref_clk);
	}
}

static int tegra_i2c_wait_for_config_load(struct tegra_i2c_dev *i2c_dev)
{
	unsigned long timeout;

	if (i2c_dev->hw->has_config_load_reg) {
		i2c_writel(i2c_dev, I2C_MSTR_CONFIG_LOAD, I2C_CONFIG_LOAD);
		timeout = jiffies + msecs_to_jiffies(1000);
		while (i2c_readl(i2c_dev, I2C_CONFIG_LOAD) != 0) {
			if (time_after(jiffies, timeout)) {
				dev_warn(i2c_dev->dev,
					"timeout waiting for config load\n");
				return -ETIMEDOUT;
			}
			msleep(1);
		}
	}

	return 0;
}

static int tegra_i2c_init(struct tegra_i2c_dev *i2c_dev)
{
	u32 val;
	int err = 0;
	u32 clk_divisor;

	err = tegra_i2c_clock_enable(i2c_dev);
	if (err < 0) {
		dev_err(i2c_dev->dev, "Clock enable failed %d\n", err);
		return err;
	}

	reset_control_assert(i2c_dev->rst);
	udelay(2);
	reset_control_deassert(i2c_dev->rst);

	if (i2c_dev->is_dvc)
		tegra_dvc_init(i2c_dev);

	if (i2c_dev->hw->is_dpaux)
		tegra_dpaux_init(i2c_dev);

	val = I2C_CNFG_NEW_MASTER_FSM | I2C_CNFG_PACKET_MODE_EN |
		(0x2 << I2C_CNFG_DEBOUNCE_CNT_SHIFT);

	if (i2c_dev->hw->has_multi_master_mode)
		val |= I2C_CNFG_MULTI_MASTER_MODE;

	i2c_writel(i2c_dev, val, I2C_CNFG);
	i2c_writel(i2c_dev, 0, I2C_INT_MASK);

	if (i2c_dev->hw->is_vi)
		tegra_vi_init(i2c_dev);

	/* Make sure clock divisor programmed correctly */
	clk_divisor = i2c_dev->hw->clk_divisor_hs_mode;
	clk_divisor |= i2c_dev->clk_divisor_non_hs_mode <<
					I2C_CLK_DIVISOR_STD_FAST_MODE_SHIFT;
	i2c_writel(i2c_dev, clk_divisor, I2C_CLK_DIVISOR);

	if (!i2c_dev->is_dvc && !i2c_dev->hw->is_vi) {
		u32 sl_cfg = i2c_readl(i2c_dev, I2C_SL_CNFG);
		sl_cfg |= I2C_SL_CNFG_NACK | I2C_SL_CNFG_NEWSL;
		i2c_writel(i2c_dev, sl_cfg, I2C_SL_CNFG);
		i2c_writel(i2c_dev, 0xfc, I2C_SL_ADDR1);
		i2c_writel(i2c_dev, 0x00, I2C_SL_ADDR2);

	}

	val = 7 << I2C_FIFO_CONTROL_TX_TRIG_SHIFT |
		((i2c_dev->hw->rx_fifo_trig & 0x7) <<
		 I2C_FIFO_CONTROL_RX_TRIG_SHIFT);
	i2c_writel(i2c_dev, val, I2C_FIFO_CONTROL);

	if (tegra_i2c_flush_fifos(i2c_dev))
		err = -ETIMEDOUT;

	if (i2c_dev->is_multimaster_mode &&
	    i2c_dev->hw->has_slcg_override_reg)
		i2c_writel(i2c_dev, I2C_MST_CORE_CLKEN_OVR, I2C_CLKEN_OVERRIDE);

	err = tegra_i2c_wait_for_config_load(i2c_dev);

	tegra_i2c_clock_disable(i2c_dev);

	if (err)
		return err;

	if (i2c_dev->irq_disabled) {
		i2c_dev->irq_disabled = 0;
		enable_irq(i2c_dev->irq);
	}

	return err;
}

static int tegra_i2c_power_enable(struct tegra_i2c_dev *i2c_dev)
{
	int ret;

	if (i2c_dev->hw->has_regulator) {
		ret = regulator_enable(i2c_dev->reg);
		if (ret)
			return ret;
	}

	if (i2c_dev->hw->has_powergate) {
		ret = tegra_pmc_unpowergate(i2c_dev->hw->powergate_id);
		if (ret)
			goto err_regulator;

		ret = tegra_i2c_init(i2c_dev);
		if (ret)
			goto err_powergate;
	}

	return 0;

err_regulator:
	if (i2c_dev->hw->has_regulator)
		regulator_disable(i2c_dev->reg);
err_powergate:
	if (i2c_dev->hw->has_powergate)
		tegra_pmc_powergate(i2c_dev->hw->powergate_id);

	return ret;
}

static void tegra_i2c_power_disable(struct tegra_i2c_dev *i2c_dev)
{
	if (i2c_dev->hw->has_regulator)
		regulator_disable(i2c_dev->reg);

	if (i2c_dev->hw->has_powergate)
		tegra_pmc_powergate(i2c_dev->hw->powergate_id);
}

static void tegra_i2c_disable_packet_mode(struct tegra_i2c_dev *i2c_dev)
{
	u32 cnfg;

	cnfg = i2c_readl(i2c_dev, I2C_CNFG);
	if (cnfg & I2C_CNFG_PACKET_MODE_EN)
		i2c_writel(i2c_dev, cnfg & (~I2C_CNFG_PACKET_MODE_EN),
			I2C_CNFG);
}

static irqreturn_t tegra_i2c_isr(int irq, void *dev_id)
{
	u32 status;
	const u32 status_err = I2C_INT_NO_ACK | I2C_INT_ARBITRATION_LOST;
	struct tegra_i2c_dev *i2c_dev = dev_id;
	unsigned long flags;
	u32 mask;

	status = i2c_readl(i2c_dev, I2C_INT_STATUS);

	spin_lock_irqsave(&i2c_dev->xfer_lock, flags);
	if (status == 0) {
		dev_warn(i2c_dev->dev, "irq status 0 %08x %08x %08x\n",
			 i2c_readl(i2c_dev, I2C_PACKET_TRANSFER_STATUS),
			 i2c_readl(i2c_dev, I2C_STATUS),
			 i2c_readl(i2c_dev, I2C_CNFG));
		i2c_dev->msg_err |= I2C_ERR_UNKNOWN_INTERRUPT;

		if (!i2c_dev->irq_disabled) {
			disable_irq_nosync(i2c_dev->irq);
			i2c_dev->irq_disabled = 1;
		}
		goto err;
	}

	if (unlikely(status & status_err)) {
		tegra_i2c_disable_packet_mode(i2c_dev);
		if (status & I2C_INT_NO_ACK)
			i2c_dev->msg_err |= I2C_ERR_NO_ACK;
		if (status & I2C_INT_ARBITRATION_LOST)
			i2c_dev->msg_err |= I2C_ERR_ARBITRATION_LOST;
		goto err;
	}

	if (i2c_dev->hw->has_hw_arb_support &&
		(status & I2C_INT_BUS_CLEAR_DONE))
		goto err;

	if (i2c_dev->msg_read && (status & I2C_INT_RX_FIFO_DATA_REQ)) {
		if (i2c_dev->msg_buf_remaining)
			tegra_i2c_empty_rx_fifo(i2c_dev);
		else
			BUG();
	}

	if (!i2c_dev->msg_read && (status & I2C_INT_TX_FIFO_DATA_REQ)) {
		if (i2c_dev->msg_buf_remaining)
			tegra_i2c_fill_tx_fifo(i2c_dev);
		else
			tegra_i2c_mask_irq(i2c_dev, I2C_INT_TX_FIFO_DATA_REQ);
	}

	i2c_writel(i2c_dev, status, I2C_INT_STATUS);
	if (i2c_dev->is_dvc)
		dvc_writel(i2c_dev, DVC_STATUS_I2C_DONE_INTR, DVC_STATUS);

	if (status & I2C_INT_PACKET_XFER_COMPLETE) {
		if (i2c_dev->hw->rx_fifo_trig > 0)
			if (i2c_dev->msg_read && i2c_dev->msg_buf_remaining)
				tegra_i2c_empty_rx_fifo(i2c_dev);
		BUG_ON(i2c_dev->msg_buf_remaining);
		complete(&i2c_dev->msg_complete);
	}
	goto done;

err:
	mask = I2C_INT_NO_ACK | I2C_INT_ARBITRATION_LOST |
		I2C_INT_PACKET_XFER_COMPLETE | I2C_INT_TX_FIFO_DATA_REQ |
		I2C_INT_RX_FIFO_DATA_REQ;

	if (i2c_dev->hw->has_hw_arb_support)
		mask |= I2C_INT_BUS_CLEAR_DONE;

	/* An error occurred, mask all interrupts */
	tegra_i2c_mask_irq(i2c_dev, mask);

	i2c_writel(i2c_dev, status, I2C_INT_STATUS);
	if (i2c_dev->is_dvc)
		dvc_writel(i2c_dev, DVC_STATUS_I2C_DONE_INTR, DVC_STATUS);

	complete(&i2c_dev->msg_complete);
done:
	spin_unlock_irqrestore(&i2c_dev->xfer_lock, flags);
	return IRQ_HANDLED;
}

static int tegra_i2c_bus_clear(struct tegra_i2c_dev *i2c_dev)
{
	int ret = 0;

	if (!i2c_dev->hw->has_hw_arb_support) {
		i2c_recover_bus(&i2c_dev->adapter);
		return -EAGAIN;
	}

	reinit_completion(&i2c_dev->msg_complete);
	i2c_writel(i2c_dev, I2C_BC_ENABLE
			| I2C_BC_SCLK_THRESHOLD
			| I2C_BC_TERMINATE,
			I2C_BUS_CLEAR_CNFG);

	ret = tegra_i2c_wait_for_config_load(i2c_dev);
	if (ret)
		return ret;

	tegra_i2c_unmask_irq(i2c_dev, I2C_INT_BUS_CLEAR_DONE);

	wait_for_completion_timeout(&i2c_dev->msg_complete,
		TEGRA_I2C_TIMEOUT);

	if (!(i2c_readl(i2c_dev, I2C_BUS_CLEAR_STATUS) & I2C_BC_STATUS))
		dev_warn(i2c_dev->dev,
			 "Un-recovered Arbitration lost\n");
	return -EAGAIN;
}

static int tegra_i2c_xfer_msg(struct tegra_i2c_dev *i2c_dev,
	struct i2c_msg *msg, enum msg_end_type end_state)
{
	u32 packet_header[3];
	u32 int_mask;
	unsigned long flags;
	int ret;

	tegra_i2c_flush_fifos(i2c_dev);

	if (msg->len == 0)
		return -EINVAL;

	i2c_dev->msg_buf = msg->buf;
	i2c_dev->msg_buf_remaining = msg->len;
	i2c_dev->msg_err = I2C_ERR_NONE;
	i2c_dev->msg_read = (msg->flags & I2C_M_RD);
	reinit_completion(&i2c_dev->msg_complete);

	spin_lock_irqsave(&i2c_dev->xfer_lock, flags);

	int_mask = I2C_INT_NO_ACK | I2C_INT_ARBITRATION_LOST;
	tegra_i2c_unmask_irq(i2c_dev, int_mask);

	packet_header[0] = (0 << PACKET_HEADER0_HEADER_SIZE_SHIFT) |
			PACKET_HEADER0_PROTOCOL_I2C |
			(i2c_dev->cont_id << PACKET_HEADER0_CONT_ID_SHIFT) |
			(1 << PACKET_HEADER0_PACKET_ID_SHIFT);

	packet_header[1] = msg->len - 1;

	packet_header[2] = I2C_HEADER_IE_ENABLE;
	if (end_state == MSG_END_CONTINUE)
		packet_header[2] |= I2C_HEADER_CONTINUE_XFER;
	else if (end_state == MSG_END_REPEAT_START)
		packet_header[2] |= I2C_HEADER_REPEAT_START;
	if (msg->flags & I2C_M_TEN) {
		packet_header[2] |= msg->addr;
		packet_header[2] |= I2C_HEADER_10BIT_ADDR;
	} else {
		packet_header[2] |= msg->addr << I2C_HEADER_SLAVE_ADDR_SHIFT;
	}
	if (msg->flags & I2C_M_IGNORE_NAK)
		packet_header[2] |= I2C_HEADER_CONT_ON_NAK;
	if (msg->flags & I2C_M_RD)
		packet_header[2] |= I2C_HEADER_READ;

	i2c_writel(i2c_dev, packet_header[0], I2C_TX_FIFO);
	i2c_writel(i2c_dev, packet_header[1], I2C_TX_FIFO);
	i2c_writel(i2c_dev, packet_header[2], I2C_TX_FIFO);

	if (!(msg->flags & I2C_M_RD))
		tegra_i2c_fill_tx_fifo(i2c_dev);

	if (i2c_dev->hw->has_per_pkt_xfer_complete_irq)
		int_mask |= I2C_INT_PACKET_XFER_COMPLETE;
	if (msg->flags & I2C_M_RD)
		int_mask |= I2C_INT_RX_FIFO_DATA_REQ;
	else if (i2c_dev->msg_buf_remaining)
		int_mask |= I2C_INT_TX_FIFO_DATA_REQ;
	tegra_i2c_unmask_irq(i2c_dev, int_mask);

	spin_unlock_irqrestore(&i2c_dev->xfer_lock, flags);

	dev_dbg(i2c_dev->dev, "unmasked irq: %02x\n",
		i2c_readl(i2c_dev, I2C_INT_MASK));

	ret = wait_for_completion_timeout(&i2c_dev->msg_complete, TEGRA_I2C_TIMEOUT);
	tegra_i2c_mask_irq(i2c_dev, int_mask);

	if (ret == 0) {
		dev_err(i2c_dev->dev, "i2c transfer timed out\n");

		tegra_i2c_init(i2c_dev);
		return -ETIMEDOUT;
	}

	dev_dbg(i2c_dev->dev, "transfer complete: %d %d %d\n",
		ret, completion_done(&i2c_dev->msg_complete), i2c_dev->msg_err);

	if (likely(i2c_dev->msg_err == I2C_ERR_NONE))
		return 0;

	/*
	 * NACK interrupt is generated before the I2C controller generates the
	 * STOP condition on the bus. So wait for 2 clock periods before resetting
	 * the controller so that STOP condition has been delivered properly.
	 */
	if (i2c_dev->msg_err & I2C_ERR_NO_ACK)
		udelay(DIV_ROUND_UP(2 * 1000000, i2c_dev->bus_clk_rate));

	ret = tegra_i2c_init(i2c_dev);
	if (WARN_ON(ret))
		return ret;

	/* Arbitration Lost occurs, Start recovery */
	if (i2c_dev->msg_err & I2C_ERR_ARBITRATION_LOST)
		return tegra_i2c_bus_clear(i2c_dev);

	if (i2c_dev->msg_err & I2C_ERR_NO_ACK) {
		if (msg->flags & I2C_M_IGNORE_NAK)
			return 0;
		return -EREMOTEIO;
	}

	return -EIO;
}

static int tegra_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[],
	int num)
{
	struct tegra_i2c_dev *i2c_dev = i2c_get_adapdata(adap);
	int i;
	int ret = 0;

	if (i2c_dev->is_suspended)
		return -EBUSY;

	ret = tegra_i2c_power_enable(i2c_dev);
	if (ret < 0) {
		dev_err(i2c_dev->dev, "Regulator enable failed %d\n", ret);
		return ret;
	}

	ret = tegra_i2c_clock_enable(i2c_dev);
	if (ret < 0) {
		tegra_i2c_power_disable(i2c_dev);
		dev_err(i2c_dev->dev, "Clock enable failed %d\n", ret);
		return ret;
	}

	for (i = 0; i < num; i++) {
		enum msg_end_type end_type = MSG_END_STOP;
		if (i < (num - 1)) {
			if (msgs[i + 1].flags & I2C_M_NOSTART)
				end_type = MSG_END_CONTINUE;
			else
				end_type = MSG_END_REPEAT_START;
		}
		ret = tegra_i2c_xfer_msg(i2c_dev, &msgs[i], end_type);
		if (ret)
			break;
	}
	tegra_i2c_clock_disable(i2c_dev);
	tegra_i2c_power_disable(i2c_dev);
	return ret ?: i;
}

static u32 tegra_i2c_func(struct i2c_adapter *adap)
{
	struct tegra_i2c_dev *i2c_dev = i2c_get_adapdata(adap);
	u32 ret = I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL | I2C_FUNC_10BIT_ADDR |
				I2C_FUNC_PROTOCOL_MANGLING;

	if (i2c_dev->hw->has_continue_xfer_support)
		ret |= I2C_FUNC_NOSTART;
	return ret;
}

static void tegra_i2c_parse_dt(struct tegra_i2c_dev *i2c_dev)
{
	struct device_node *np = i2c_dev->dev->of_node;
	int ret;

	ret = of_property_read_u32(np, "clock-frequency",
			&i2c_dev->bus_clk_rate);
	if (ret)
		i2c_dev->bus_clk_rate = 100000; /* default clock rate */

	i2c_dev->is_multimaster_mode = of_property_read_bool(np,
			"nvidia,multimaster-mode");
	i2c_dev->scl_gpio = of_get_named_gpio(np, "nvidia,scl-gpio", 0);
	i2c_dev->sda_gpio = of_get_named_gpio(np, "nvidia,sda-gpio", 0);
	i2c_dev->needs_cl_dvfs_clock = of_property_read_bool(np,
			"nvidia,require-cldvfs-clock");
}

static const struct i2c_algorithm tegra_i2c_algo = {
	.master_xfer	= tegra_i2c_xfer,
	.functionality	= tegra_i2c_func,
};

static const struct tegra_i2c_hw_feature tegra20_i2c_hw = {
	.has_continue_xfer_support = false,
	.has_per_pkt_xfer_complete_irq = false,
	.has_single_clk_source = false,
	.clk_divisor_hs_mode = 3,
	.clk_divisor_std_fast_mode = 0,
	.clk_divisor_fast_plus_mode = 0,
	.has_config_load_reg = false,
	.has_hw_arb_support = false,
	.has_multi_master_mode = false,
	.has_slcg_override_reg = false,
	.rx_fifo_trig = 0,
};

static const struct tegra_i2c_hw_feature tegra30_i2c_hw = {
	.has_continue_xfer_support = true,
	.has_per_pkt_xfer_complete_irq = false,
	.has_single_clk_source = false,
	.clk_divisor_hs_mode = 3,
	.clk_divisor_std_fast_mode = 0,
	.clk_divisor_fast_plus_mode = 0,
	.has_config_load_reg = false,
	.has_hw_arb_support = false,
	.has_multi_master_mode = false,
	.has_slcg_override_reg = false,
	.rx_fifo_trig = 0,
};

static const struct tegra_i2c_hw_feature tegra114_i2c_hw = {
	.has_continue_xfer_support = true,
	.has_per_pkt_xfer_complete_irq = true,
	.has_single_clk_source = true,
	.clk_divisor_hs_mode = 1,
	.clk_divisor_std_fast_mode = 0x19,
	.clk_divisor_fast_plus_mode = 0x10,
	.has_config_load_reg = false,
	.has_hw_arb_support = true,
	.has_multi_master_mode = false,
	.has_slcg_override_reg = false,
	.rx_fifo_trig = 0,
};

static const struct tegra_i2c_hw_feature tegra124_i2c_hw = {
	.has_continue_xfer_support = true,
	.has_per_pkt_xfer_complete_irq = true,
	.has_single_clk_source = true,
	.clk_divisor_hs_mode = 1,
	.clk_divisor_std_fast_mode = 0x19,
	.clk_divisor_fast_plus_mode = 0x10,
	.has_config_load_reg = true,
	.has_hw_arb_support = true,
	.has_multi_master_mode = false,
	.has_slcg_override_reg = true,
	.rx_fifo_trig = 0,
};

static const struct tegra_i2c_hw_feature tegra210_i2c_hw = {
	.has_continue_xfer_support = true,
	.has_per_pkt_xfer_complete_irq = true,
	.has_single_clk_source = true,
	.clk_divisor_hs_mode = 1,
	.clk_divisor_std_fast_mode = 0x19,
	.clk_divisor_fast_plus_mode = 0x10,
	.has_config_load_reg = true,
	.has_hw_arb_support = true,
	.has_multi_master_mode = true,
	.has_slcg_override_reg = true,
	.rx_fifo_trig = 4,
};

static const struct tegra_i2c_hw_feature tegra210_i2c_vi_hw = {
	.has_continue_xfer_support = true,
	.has_per_pkt_xfer_complete_irq = true,
	.has_single_clk_source = true,
	.clk_divisor_hs_mode = 1,
	.clk_divisor_std_fast_mode = 0x19,
	.clk_divisor_fast_plus_mode = 0,
	.has_config_load_reg = false,
	.has_regulator = true,
	.has_powergate = true,
	.powergate_id = TEGRA_POWERGATE_VENC,
	.is_vi = true,
	.has_hw_arb_support = true,
	.has_multi_master_mode = true,
	.has_slcg_override_reg = true,
	.rx_fifo_trig = 4,
};

static const struct tegra_i2c_hw_feature tegra210_i2c_dpaux_hw = {
	.has_continue_xfer_support = true,
	.has_per_pkt_xfer_complete_irq = true,
	.has_single_clk_source = true,
	.clk_divisor_hs_mode = 1,
	.clk_divisor_std_fast_mode = 0x19,
	.clk_divisor_fast_plus_mode = 0,
	.has_config_load_reg = true,
	.has_powergate = true,
	.powergate_id = TEGRA_POWERGATE_SOR,
	.is_dpaux = true,
	.has_multi_master_mode = true,
	.has_slcg_override_reg = true,
	.rx_fifo_trig = 4,
};

/* Match table for of_platform binding */
static const struct of_device_id tegra_i2c_of_match[] = {
	{ .compatible = "nvidia,tegra210-i2c", .data = &tegra210_i2c_hw, },
	{ .compatible = "nvidia,tegra124-i2c", .data = &tegra124_i2c_hw, },
	{ .compatible = "nvidia,tegra114-i2c", .data = &tegra114_i2c_hw, },
	{ .compatible = "nvidia,tegra30-i2c", .data = &tegra30_i2c_hw, },
	{ .compatible = "nvidia,tegra20-i2c", .data = &tegra20_i2c_hw, },
	{ .compatible = "nvidia,tegra20-i2c-dvc", .data = &tegra20_i2c_hw, },
	{ .compatible = "nvidia,tegra210-i2c-vi",
		.data = &tegra210_i2c_vi_hw, },
	{ .compatible = "nvidia,tegra210-i2c-dpaux",
		.data = &tegra210_i2c_dpaux_hw, },
	{},
};
MODULE_DEVICE_TABLE(of, tegra_i2c_of_match);

static struct i2c_bus_recovery_info tegra_i2c_bus_recovery_info = {
	.recover_bus		= i2c_generic_gpio_recovery,
};

static int tegra_i2c_probe(struct platform_device *pdev)
{
	struct tegra_i2c_dev *i2c_dev;
	struct resource *mem;
	struct resource *res;
	struct clk *div_clk;
	struct clk *fast_clk;
	void __iomem *base;
	int irq;
	int ret = 0;
	int clk_multiplier = I2C_CLK_MULTIPLIER_STD_FAST_MODE;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(base))
		return PTR_ERR(base);

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(&pdev->dev, "no irq resource\n");
		return -EINVAL;
	}
	irq = res->start;

	div_clk = devm_clk_get(&pdev->dev, "div-clk");
	if (IS_ERR(div_clk)) {
		dev_err(&pdev->dev, "missing controller clock");
		return PTR_ERR(div_clk);
	}

	i2c_dev = devm_kzalloc(&pdev->dev, sizeof(*i2c_dev), GFP_KERNEL);
	if (!i2c_dev)
		return -ENOMEM;

	i2c_dev->base = base;
	i2c_dev->div_clk = div_clk;
	i2c_dev->adapter.algo = &tegra_i2c_algo;
	i2c_dev->irq = irq;
	i2c_dev->dev = &pdev->dev;

	i2c_dev->rst = devm_reset_control_get(&pdev->dev, "i2c");
	if (IS_ERR(i2c_dev->rst)) {
		dev_err(&pdev->dev, "missing controller reset");
		return PTR_ERR(i2c_dev->rst);
	}

	tegra_i2c_parse_dt(i2c_dev);

	i2c_dev->hw = &tegra20_i2c_hw;

	if (pdev->dev.of_node) {
		const struct of_device_id *match;
		match = of_match_device(tegra_i2c_of_match, &pdev->dev);
		i2c_dev->hw = match->data;
		i2c_dev->is_dvc = of_device_is_compatible(pdev->dev.of_node,
						"nvidia,tegra20-i2c-dvc");
	} else if (pdev->id == 3) {
		i2c_dev->is_dvc = 1;
	}
	init_completion(&i2c_dev->msg_complete);

	if (!i2c_dev->hw->has_single_clk_source) {
		fast_clk = devm_clk_get(&pdev->dev, "fast-clk");
		if (IS_ERR(fast_clk)) {
			dev_err(&pdev->dev, "missing fast clock");
			return PTR_ERR(fast_clk);
		}
		i2c_dev->fast_clk = fast_clk;
	}

	if (i2c_dev->hw->is_dpaux) {
		i2c_dev->dpaux_clk = devm_clk_get(&pdev->dev, "dpaux-clk");
		if (IS_ERR(i2c_dev->dpaux_clk)) {
			dev_err(&pdev->dev, "missing dpaux clock\n");
			return PTR_ERR(i2c_dev->dpaux_clk);
		}
		i2c_dev->sor_clk = devm_clk_get(&pdev->dev, "sor-clk");
		if (IS_ERR(i2c_dev->sor_clk)) {
			dev_err(&pdev->dev, "missing sor clock\n");
			return PTR_ERR(i2c_dev->sor_clk);
		}

		res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
		if (!res) {
			dev_err(&pdev->dev, "missing dpaux registers\n");
			return -ENODEV;
		}
		i2c_dev->dpaux_base = devm_ioremap(&pdev->dev, res->start,
						   resource_size(res));
		if (IS_ERR(i2c_dev->dpaux_base)) {
			dev_err(&pdev->dev, "failed to map dpaux registers\n");
			return PTR_ERR(i2c_dev->dpaux_base);
		}
	}

	if (i2c_dev->needs_cl_dvfs_clock) {
		i2c_dev->dvfs_ref_clk = devm_clk_get(&pdev->dev,
						     "cl_dvfs_ref");
		if (IS_ERR(i2c_dev->dvfs_ref_clk)) {
			dev_err(&pdev->dev, "missing cl_dvfs_ref clock\n");
			return PTR_ERR(i2c_dev->dvfs_ref_clk);
		}
		i2c_dev->dvfs_soc_clk = devm_clk_get(&pdev->dev,
						     "cl_dvfs_soc");
		if (IS_ERR(i2c_dev->dvfs_soc_clk)) {
			dev_err(&pdev->dev, "missing cl_dvfs_soc clock\n");
			return PTR_ERR(i2c_dev->dvfs_soc_clk);
		}
	}

	if (i2c_dev->hw->is_vi) {
		i2c_dev->slow_clk = devm_clk_get(&pdev->dev, "slow-clk");
		if (IS_ERR(i2c_dev->slow_clk)) {
			dev_err(&pdev->dev, "missing slow clock");
			return PTR_ERR(i2c_dev->slow_clk);
		}
		i2c_dev->host1x_clk = devm_clk_get(&pdev->dev, "host1x-clk");
		if (IS_ERR(i2c_dev->host1x_clk)) {
			dev_err(&pdev->dev, "missing host1x clock");
			return PTR_ERR(i2c_dev->host1x_clk);
		}
	}

	if (i2c_dev->hw->has_regulator) {
		i2c_dev->reg = devm_regulator_get(&pdev->dev, "avdd");
		if (IS_ERR(i2c_dev->reg)) {
			dev_err(&pdev->dev, "could not get regulator");
			return PTR_ERR(i2c_dev->reg);
		}
	}

	platform_set_drvdata(pdev, i2c_dev);

	if (!i2c_dev->hw->has_single_clk_source) {
		ret = clk_prepare(i2c_dev->fast_clk);
		if (ret < 0) {
			dev_err(i2c_dev->dev, "Clock prepare failed %d\n", ret);
			return ret;
		}
	}

	i2c_dev->clk_divisor_non_hs_mode =
			i2c_dev->hw->clk_divisor_std_fast_mode;
	if (i2c_dev->hw->clk_divisor_fast_plus_mode &&
		(i2c_dev->bus_clk_rate == 1000000))
		i2c_dev->clk_divisor_non_hs_mode =
			i2c_dev->hw->clk_divisor_fast_plus_mode;

	clk_multiplier *= (i2c_dev->clk_divisor_non_hs_mode + 1);
	ret = clk_set_rate(i2c_dev->div_clk,
			   i2c_dev->bus_clk_rate * clk_multiplier);
	if (ret) {
		dev_err(i2c_dev->dev, "Clock rate change failed %d\n", ret);
		goto unprepare_fast_clk;
	}

	ret = clk_prepare(i2c_dev->div_clk);
	if (ret < 0) {
		dev_err(i2c_dev->dev, "Clock prepare failed %d\n", ret);
		goto unprepare_fast_clk;
	}

	if (i2c_dev->is_multimaster_mode) {
		ret = clk_enable(i2c_dev->div_clk);
		if (ret < 0) {
			dev_err(i2c_dev->dev, "div_clk enable failed %d\n",
				ret);
			goto unprepare_div_clk;
		}
	}

	if (!i2c_dev->hw->has_powergate) {
		ret = tegra_i2c_init(i2c_dev);
		if (ret) {
			dev_err(&pdev->dev, "Failed to initialize i2c controller");
			goto disable_div_clk;
		}
	}

	ret = devm_request_irq(&pdev->dev, i2c_dev->irq,
			tegra_i2c_isr, 0, dev_name(&pdev->dev), i2c_dev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request irq %i\n", i2c_dev->irq);
		goto disable_div_clk;
	}

	i2c_set_adapdata(&i2c_dev->adapter, i2c_dev);
	i2c_dev->adapter.owner = THIS_MODULE;
	i2c_dev->adapter.class = I2C_CLASS_DEPRECATED;
	strlcpy(i2c_dev->adapter.name, "Tegra I2C adapter",
		sizeof(i2c_dev->adapter.name));
	i2c_dev->adapter.algo = &tegra_i2c_algo;
	i2c_dev->adapter.dev.parent = &pdev->dev;
	i2c_dev->adapter.nr = pdev->id;
	i2c_dev->adapter.dev.of_node = pdev->dev.of_node;

	tegra_i2c_bus_recovery_info.scl_gpio = i2c_dev->scl_gpio;
	tegra_i2c_bus_recovery_info.sda_gpio = i2c_dev->sda_gpio;
	i2c_dev->adapter.bus_recovery_info = &tegra_i2c_bus_recovery_info;

	if (i2c_dev->needs_cl_dvfs_clock) {
		ret = clk_prepare(i2c_dev->dvfs_ref_clk);
		if (ret < 0) {
			dev_err(i2c_dev->dev,
				"Clock dvfs_ref_clk prepare failed %d\n", ret);
			goto disable_div_clk;
		}

		ret = clk_prepare(i2c_dev->dvfs_soc_clk);
		if (ret < 0) {
			dev_err(i2c_dev->dev,
				"Clock dvfs_soc_clk prepare failed %d\n", ret);
			goto unprepare_dvfs_ref_clk;
		}
	}

	ret = i2c_add_numbered_adapter(&i2c_dev->adapter);
	if (ret) {
		dev_err(&pdev->dev, "Failed to add I2C adapter\n");
		goto unprepare_dvfs_soc_clk;
	}
	i2c_dev->cont_id = i2c_dev->adapter.nr & PACKET_HEADER0_CONT_ID_MASK;

	return 0;

unprepare_dvfs_soc_clk:
	if (i2c_dev->needs_cl_dvfs_clock)
		clk_unprepare(i2c_dev->dvfs_soc_clk);

unprepare_dvfs_ref_clk:
	if (i2c_dev->needs_cl_dvfs_clock)
		clk_unprepare(i2c_dev->dvfs_ref_clk);

disable_div_clk:
	if (i2c_dev->is_multimaster_mode)
		clk_disable(i2c_dev->div_clk);

unprepare_div_clk:
	clk_unprepare(i2c_dev->div_clk);

unprepare_fast_clk:
	if (!i2c_dev->hw->has_single_clk_source)
		clk_unprepare(i2c_dev->fast_clk);

	return ret;
}

static int tegra_i2c_remove(struct platform_device *pdev)
{
	struct tegra_i2c_dev *i2c_dev = platform_get_drvdata(pdev);
	i2c_del_adapter(&i2c_dev->adapter);

	if (i2c_dev->is_multimaster_mode)
		clk_disable(i2c_dev->div_clk);

	clk_unprepare(i2c_dev->div_clk);
	if (!i2c_dev->hw->has_single_clk_source)
		clk_unprepare(i2c_dev->fast_clk);

	if (i2c_dev->needs_cl_dvfs_clock) {
		clk_unprepare(i2c_dev->dvfs_ref_clk);
		clk_unprepare(i2c_dev->dvfs_soc_clk);
	}
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int tegra_i2c_suspend(struct device *dev)
{
	struct tegra_i2c_dev *i2c_dev = dev_get_drvdata(dev);

	i2c_lock_adapter(&i2c_dev->adapter);
	i2c_dev->is_suspended = true;
	i2c_unlock_adapter(&i2c_dev->adapter);

	return 0;
}

static int tegra_i2c_resume(struct device *dev)
{
	struct tegra_i2c_dev *i2c_dev = dev_get_drvdata(dev);
	int ret;

	i2c_lock_adapter(&i2c_dev->adapter);

	if (!i2c_dev->hw->has_powergate) {
		ret = tegra_i2c_init(i2c_dev);
		if (ret) {
			i2c_unlock_adapter(&i2c_dev->adapter);
			return ret;
		}
	}

	i2c_dev->is_suspended = false;

	i2c_unlock_adapter(&i2c_dev->adapter);

	return 0;
}

static const struct dev_pm_ops tegra_i2c_pm = {
	.suspend_noirq = tegra_i2c_suspend,
	.resume_noirq = tegra_i2c_resume,
};
#define TEGRA_I2C_PM	(&tegra_i2c_pm)
#else
#define TEGRA_I2C_PM	NULL
#endif

static struct platform_driver tegra_i2c_driver = {
	.probe   = tegra_i2c_probe,
	.remove  = tegra_i2c_remove,
	.driver  = {
		.name  = "tegra-i2c",
		.owner = THIS_MODULE,
		.of_match_table = tegra_i2c_of_match,
		.pm    = TEGRA_I2C_PM,
	},
};

static int __init tegra_i2c_init_driver(void)
{
	return platform_driver_register(&tegra_i2c_driver);
}

static void __exit tegra_i2c_exit_driver(void)
{
	platform_driver_unregister(&tegra_i2c_driver);
}

subsys_initcall(tegra_i2c_init_driver);
module_exit(tegra_i2c_exit_driver);

MODULE_DESCRIPTION("nVidia Tegra2 I2C Bus Controller driver");
MODULE_AUTHOR("Colin Cross");
MODULE_LICENSE("GPL v2");
