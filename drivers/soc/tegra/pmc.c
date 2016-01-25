/*
 * drivers/soc/tegra/pmc.c
 *
 * Copyright (c) 2010 Google, Inc
 *
 * Author:
 *	Colin Cross <ccross@google.com>
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
#include <linux/clk.h>
#include <linux/clk/tegra.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/firmware.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/reset.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/suspend.h>
#include <linux/syscore_ops.h>
#include <linux/wakeup_reason.h>
#include <dt-bindings/soc/tegra-pmc.h>

#include <soc/tegra/common.h>
#include <soc/tegra/fuse.h>
#include <soc/tegra/mc.h>
#include <soc/tegra/pmc.h>

#define UTMIP(port, x, y)		(((port) <= 2) ? (x) : (y))

#define PMC_CNTRL			0x0
#define  PMC_CNTRL_LATCH_WAKEUPS	(1 << 5)
#define  PMC_CNTRL_PWRREQ_POLARITY	(1 << 8)   /* core power req polarity */
#define  PMC_CNTRL_PWRREQ_OE		(1 << 9)   /* core power req enable */
#define  PMC_CNTRL_SYSCLK_POLARITY	(1 << 10)  /* sys clk polarity */
#define  PMC_CNTRL_SYSCLK_OE		(1 << 11)  /* system clock enable */
#define  PMC_CNTRL_SIDE_EFFECT_LP0	(1 << 14)  /* LP0 when CPU pwr gated */
#define  PMC_CNTRL_CPU_PWRREQ_POLARITY	(1 << 15)  /* CPU pwr req polarity */
#define  PMC_CNTRL_CPU_PWRREQ_OE	(1 << 16)  /* CPU pwr req enable */
#define  PMC_CNTRL_INTR_POLARITY	(1 << 17)  /* inverts INTR polarity */

#define PMC_WAKE_MASK			0xc
#define PMC_WAKE_LEVEL			0x10
#define PMC_WAKE_STATUS			0x14
#define PMC_SW_WAKE_STATUS		0x18

#define DPD_SAMPLE			0x020
#define  DPD_SAMPLE_ENABLE		(1 << 0)
#define  DPD_SAMPLE_DISABLE		(0 << 0)

#define PMC_DPD_ENABLE			0x24
#define PMC_DPD_ENABLE_ON		(1 << 0)
#define PMC_DPD_ENABLE_TSC_MULT_ENABLE	(1 << 1)

#define CLAMP_STATUS			0x2c

#define PWRGATE_TOGGLE			0x30
#define  PWRGATE_TOGGLE_START		(1 << 8)

#define REMOVE_CLAMPING			0x34

#define PWRGATE_STATUS			0x38

#define PMC_COREPWRGOOD_TIMER		0x3c

#define PMC_SCRATCH0			0x50
#define  PMC_SCRATCH0_MODE_RECOVERY	(1 << 31)
#define  PMC_SCRATCH0_MODE_BOOTLOADER	(1 << 30)
#define  PMC_SCRATCH0_MODE_RCM		(1 << 1)
#define  PMC_SCRATCH0_MODE_MASK		(PMC_SCRATCH0_MODE_RECOVERY | \
					 PMC_SCRATCH0_MODE_BOOTLOADER | \
					 PMC_SCRATCH0_MODE_RCM)

#define PMC_SCRATCH1			0x54

#define PMC_CPUPWRGOOD_TIMER		0xc8
#define PMC_CPUPWROFF_TIMER		0xcc

#define PMC_WAKE_DELAY			0xe0
#define PMC_COREPWROFF_TIMER		PMC_WAKE_DELAY

#define PMC_USB_DEBOUNCE_DEL		0xec
#define  UTMIP_LINE_DEB_CNT_SHIFT	16
#define  UTMIP_LINE_DEB_CNT_MASK	0xf
#define  DEBOUNCE_VAL_SHIFT		0
#define  DEBOUNCE_VAL_MASK		0xffff

#define PMC_USB_AO			0xf0
#define  DATA_VAL_PD(x)			(1 << (4 * (x) + 13))
#define  STROBE_VAL_PD(x)		(1 << (4 * (x) + 12))
#define  USBON_VAL_PD(x)		UTMIP(x, 1 << (4 * (x) + 1), 1 << 21)
#define  USBOP_VAL_PD(x)		UTMIP(x, 1 << (4 * (x)), 1 << 20)

#define PMC_SCRATCH41			0x140

#define PMC_WAKE2_MASK			0x160
#define PMC_WAKE2_LEVEL			0x164
#define PMC_WAKE2_STATUS		0x168
#define PMC_SW_WAKE2_STATUS		0x16c

#define PMC_OSC_EDPD_OVER		0x1a4
#define  OSC_EDPD_CLK_OK		(0x1 << 23)
#define  OSC_EDPD_OSC_CTRL_SELECT_PMC	(0x1 << 22)
#define  OSC_EDPD_XO_LP0_MODE_MASK	(0x3 << 20)
#define  OSC_EDPD_XO_LP0_MODE_ON	(0x2 << 20)

#define PMC_CLK_OUT_CNTRL		0x1a8
#define  PMC_CLK_OUT_CLK1_SRC_SEL_MASK	(0x3 << 6)
#define  PMC_CLK_OUT_CLK1_SRC_SEL_DIV1	(0x0 << 6)
#define  PMC_CLK_OUT_CLK1_SRC_SEL_DIV2	(0x1 << 6)
#define  PMC_CLK_OUT_CLK1_SRC_SEL_DIV4	(0x1 << 6)
#define  PMC_CLK_OUT_CLK1_SRC_SEL_CAR	(0x3 << 6)
#define  PMC_CLK_OUT_CLK1_FORCE_EN	(0x1 << 2)
#define  PMC_CLK_OUT_CLK1_ACCEPT_REQ	(0x1)
#define  PMC_CLK_OUT_CLK1_IDLE_STATE_MASK	(0x3 << 4)

#define PMC_SENSOR_CTRL			0x1b0
#define PMC_SENSOR_CTRL_SCRATCH_WRITE	(1 << 2)
#define PMC_SENSOR_CTRL_ENABLE_RST	(1 << 1)

#define IO_DPD_REQ			0x1b8
#define  IO_DPD_CSIA			(1 << 0)
#define  IO_DPD_CSIB			(1 << 1)
#define  IO_DPD_DSI			(1 << 2)
#define  IO_DPD_MIPI_BIAS		(1 << 3)
#define  IO_DPD_PEX_BIAS		(1 << 4)
#define  IO_DPD_PEX_CLK1		(1 << 5)
#define  IO_DPD_PEX_CLK2		(1 << 6)
#define  IO_DPD_PEX_CLK3		(1 << 7)
#define  IO_DPD_DAC			(1 << 8)
#define  IO_DPD_USB0			(1 << 9)
#define  IO_DPD_USB1			(1 << 10)
#define  IO_DPD_USB2			(1 << 11)
#define  IO_DPD_USB_BIAS		(1 << 12)
#define  IO_DPD_NAND			(1 << 13)
#define  IO_DPD_UART			(1 << 14)
#define  IO_DPD_BB			(1 << 15)
#define  IO_DPD_VI			(1 << 16)
#define  IO_DPD_AUDIO			(1 << 17)
#define  IO_DPD_LCD			(1 << 18)
#define  IO_DPD_HSIC			(1 << 19)
#define  IO_DPD_REQ_CODE_IDLE		(0 << 30)
#define  IO_DPD_REQ_CODE_OFF		(1 << 30)
#define  IO_DPD_REQ_CODE_ON		(2 << 30)
#define  IO_DPD_REQ_CODE_MASK		(3 << 30)

#define IO_DPD_STATUS			0x1bc

#define IO_DPD2_REQ			0x1c0
#define  IO_DPD2_PEX_CNTRL		(1 << 0)
#define  IO_DPD2_SDMMC1			(1 << 1)
#define  IO_DPD2_SDMMC3			(1 << 2)
#define  IO_DPD2_SDMMC4			(1 << 3)
#define  IO_DPD2_CAM			(1 << 4)
#define  IO_DPD2_RES_RAIL		(1 << 5)
#define  IO_DPD2_HV			(1 << 6)
#define  IO_DPD2_DSIB			(1 << 7)
#define  IO_DPD2_DSIC			(1 << 8)
#define  IO_DPD2_DSID			(1 << 9)
#define  IO_DPD2_CSIC			(1 << 10)
#define  IO_DPD2_CSID			(1 << 11)
#define  IO_DPD2_CSIE			(1 << 12)

#define IO_DPD2_STATUS			0x1c4
#define SEL_DPD_TIM			0x1c8

#define DPD_STATE_CHANGE_DELAY		700

#define PMC_UTMIP_UHSIC_TRIGGERS	0x1ec
#define  UTMIP_CLR_WAKE_ALARM(x)	UTMIP(x, 1 << ((x) + 12), 1 << 19)
#define  UTMIP_CAP_CFG(x)		UTMIP(x, 1 << ((x) + 4), 1 << 17)
#define  UTMIP_CLR_WALK_PTR(x)		UTMIP(x, 1 << (x), 1 << 16)

#define PMC_UTMIP_TERM_PAD_CFG		0x1f8
#define  TEGRA210_TCTRL_VAL_SHIFT	7
#define  TEGRA210_TCTRL_VAL_MASK	0x3f
#define  TCTRL_VAL_SHIFT		5
#define  TCTRL_VAL_MASK			0x1f
#define  PCTRL_VAL_SHIFT		1
#define  PCTRL_VAL_MASK			0x3f
#define  RCTRL_VAL_SHIFT		0
#define  RCTRL_VAL_MASK			0x1f

#define PMC_UTMIP_UHSIC_SLEEP_CFG(x)	UTMIP(x, 0x1fc, 0x4d0)
#define  UTMIP_WAKE_VAL(x, v)		UTMIP(x, \
						((v) & 0xf) << ((8 * (x)) + 4),\
						((v) & 0xf) << 4)
#define  UTMIP_WAKE_VAL_SHIFT(x)	UTMIP(x, 8 * (x) + 4, 4)
#define  UTMIP_WAKE_VAL_MASK		0xf
#define  UTMIP_WAKE_VAL_NONE		0xc
#define  UTMIP_WAKE_VAL_ANY		0xf
#define  UTMIP_TCTRL_USE_PMC(x)		UTMIP(x, 1 << (8 * (x) + 3), 1 << 3)
#define  UTMIP_PCTRL_USE_PMC(x)		UTMIP(x, 1 << (8 * (x) + 2), 1 << 2)
#define  UTMIP_RCTRL_USE_PMC(x)		UTMIP(x, 1 << (8 * (x) + 2), 1 << 2)
#define  UTMIP_FSLS_USE_PMC(x)		UTMIP(x, 1 << (8 * (x) + 1), 1 << 1)
#define  UTMIP_MASTER_ENABLE(x)		UTMIP(x, 1 << (8 * (x)), 1 << 0)

/* for PMC_UTMIP_UHSIC_SLEEP_CFG(3) only */
#define UTMIP_RPU_SWITC_LOW_USE_PMC(x)  (1 << (8 + (x)))
#define UTMIP_RPD_CTRL_USE_PMC(x)	(1 << (16 + (x)))

#define PMC_UTMIP_UHSIC_SLEEPWALK_CFG(x)	UTMIP(x, 0x200, 0x288)
#define  UTMIP_LINEVAL_WALK_EN(x)	UTMIP(x, 1 << (8 * (x) + 7), 1 << 15)

#define PMC_UTMIP_SLEEPWALK_PX(x)	UTMIP(x, 0x204 + (4 * (x)), 0x4e0)
#define  UTMIP_HIGHZ_D			(1 << 30)
#define  UTMIP_AN_D			(1 << 29)
#define  UTMIP_AP_D			(1 << 28)
#define  UTMIP_USBON_RPD_D		(1 << 25)
#define  UTMIP_USBOP_RPD_D		(1 << 24)
#define  UTMIP_HIGHZ_C			(1 << 22)
#define  UTMIP_AN_C			(1 << 21)
#define  UTMIP_AP_C			(1 << 20)
#define  UTMIP_USBON_RPD_C		(1 << 17)
#define  UTMIP_USBOP_RPD_C		(1 << 16)
#define  UTMIP_HIGHZ_B			(1 << 14)
#define  UTMIP_AN_B			(1 << 13)
#define  UTMIP_AP_B			(1 << 12)
#define  UTMIP_USBON_RPD_B		(1 << 9)
#define  UTMIP_USBOP_RPD_B		(1 << 8)
#define  UTMIP_HIGHZ_A			(1 << 6)
#define  UTMIP_AN_A			(1 << 5)
#define  UTMIP_AP_A			(1 << 4)
#define  UTMIP_USBON_RPD_A		(1 << 1)
#define  UTMIP_USBOP_RPD_A		(1 << 0)

#define PMC_UHSIC_SLEEPWALK_P0		0x210

#define PMC_UTMIP_UHSIC_FAKE(x)		UTMIP(x, 0x218, 0x294)
#define  UTMIP_FAKE_USBON_VAL(x)	UTMIP(x, 1 << (4 * (x) + 1), 1 << 9)
#define  UTMIP_FAKE_USBOP_VAL(x)	UTMIP(x, 1 << (4 * (x)), 1 << 8)

#define PMC_SCRATCH54			0x258
#define PMC_SCRATCH54_DATA_SHIFT	8
#define PMC_SCRATCH54_ADDR_SHIFT	0

#define PMC_SCRATCH55			0x25c
#define PMC_SCRATCH55_RESET_TEGRA	(1 << 31)
#define PMC_SCRATCH55_CNTRL_ID_SHIFT	27
#define PMC_SCRATCH55_PINMUX_SHIFT	24
#define PMC_SCRATCH55_16BIT_OP		(1 << 15)
#define PMC_SCRATCH55_CHECKSUM_SHIFT	16
#define PMC_SCRATCH55_I2CSLV1_SHIFT	0

#define PMC_UTMIP_BIAS_MASTER_CNTRL	0x270
#define  UTMIP_BIAS_MASTER_PROG_VAL	(1 << 1)

#define PMC_UTMIP_MASTER_CONFIG		0x274
#define  UTMIP_PWR(x)			UTMIP(x, 1 << (x), 1 << 4)

#define PMC_UTMIP_UHSIC2_TRIGGERS	0x27c
#define PMC_UTMIP_UHSIC2_SLEEP_CFG	0x284
#define PMC_UTMIP_UHSIC2_SLEEPWALK_CFG	0x288
#define PMC_UHSIC_SLEEPWALK_P1		0x28c
#define PMC_UTMIP_UHSIC2_FAKE		0x294
#define PMC_UTMIP_MASTER2_CONFIG	0x29c

#define PMC_UHSIC_TRIGGERS(x)		((x) ? PMC_UTMIP_UHSIC2_TRIGGERS : \
					 PMC_UTMIP_UHSIC_TRIGGERS)
#define  UHSIC_CLR_WAKE_ALARM(x)	((x) ? 3 : 15)
#define  UHSIC_CLR_WALK_PTR(x)		((x) ? 0 : 3)

#define PMC_UHSIC_SLEEP_CFG(x)		((x) ? PMC_UTMIP_UHSIC2_SLEEP_CFG : \
					 PMC_UTMIP_UHSIC_SLEEP_CFG(x))
#define  UHSIC_WAKE_VAL_SHIFT(x)	((x) ? 4 : 28)
#define  UHSIC_WAKE_VAL_MASK		0xf
#define  UHSIC_WAKE_VAL_NONE		0xc
#define  UHSIC_WAKE_VAL_SD10		0x2
#define  UHSIC_MASTER_ENABLE(x)		((x) ? 0 : 24)

#define PMC_UHSIC_SLEEPWALK_CFG(x)	((x) ? PMC_UTMIP_UHSIC2_SLEEPWALK_CFG \
					: PMC_UTMIP_UHSIC_SLEEPWALK_CFG(x))
#define  UHSIC_LINEVAL_WALK_EN(x)	((x) ? 7 : 31)

#define PMC_UHSIC_SLEEPWALK_PX(x)	((x) ? PMC_UHSIC_SLEEPWALK_P1 : \
					 PMC_UHSIC_SLEEPWALK_P0)
#define  UHSIC_DATA_RPU_D		(1 << 27)
#define  UHSIC_STROBE_RPU_D		(1 << 26)
#define  UHSIC_DATA_RPD_D		(1 << 25)
#define  UHSIC_STROBE_RPD_D		(1 << 24)
#define  UHSIC_DATA_RPU_C		(1 << 19)
#define  UHSIC_STROBE_RPU_C		(1 << 18)
#define  UHSIC_DATA_RPD_C		(1 << 17)
#define  UHSIC_STROBE_RPD_C		(1 << 16)
#define  UHSIC_DATA_RPU_B		(1 << 11)
#define  UHSIC_STROBE_RPU_B		(1 << 10)
#define  UHSIC_DATA_RPD_B		(1 << 9)
#define  UHSIC_STROBE_RPD_B		(1 << 8)
#define  UHSIC_DATA_RPU_A		(1 << 3)
#define  UHSIC_STROBE_RPU_A		(1 << 2)
#define  UHSIC_DATA_RPD_A		(1 << 1)
#define  UHSIC_STROBE_RPD_A		(1 << 0)

#define PMC_UHSIC_FAKE(x)		((x) ? PMC_UTMIP_UHSIC2_FAKE : \
					 PMC_UTMIP_UHSIC_FAKE(x))
#define  UHSIC_FAKE_DATA_VAL(x)		((x) ? 1 : 13)
#define  UHSIC_FAKE_STROBE_VAL(x)	((x) ? 0 : 12)

#define PMC_UHSIC_MASTER_CONFIG(x)	((x) ? PMC_UTMIP_MASTER2_CONFIG : \
					 PMC_UTMIP_MASTER_CONFIG)
#define  UHSIC_PWR(x)			((x) ? 0 : 3)

#define GPU_RG_CNTRL			0x2d4

#define PMC_UTMIP_PAD_CFG(x)		(0x4c0 + (4 * (x)))
#define  UTMIP_RPD_CTRL(x)		(((x) & 0x1f) << 22)

#define PMC_FUSE_CTRL			0x450
#define PMC_FUSE_CTRL_PS18_LATCH_SET    (1 << 8)
#define PMC_FUSE_CTRL_PS18_LATCH_CLEAR  (1 << 9)

#define PMC_SCRATCH202				0x848
#define  PMC_SCRATCH202_BOOTREASON_REBOOT	0x1
#define  PMC_SCRATCH202_BOOTREASON_PANIC	0x2
#define  PMC_SCRATCH202_BOOTREASON_WATCHDOG	0x3
#define  PMC_SCRATCH202_BOOTREASON_THERMAL	0x4
#define  PMC_SCRATCH202_BOOTREASON_MASK		0x7

#define PMC_SCRATCH250				0x908
#define  PMC_SCRATCH250_RETRIES_SHIFT		0
#define  PMC_SCRATCH250_TRANSFER_DELAY_SHIFT	3
#define  PMC_SCRATCH250_NUM_CONFIGS_SHIFT	8
#define  PMC_SCRATCH250_BUS_CLEAR_DELAY_SHIFT	11

#define PMC_SCRATCH251				0x90c
#define  PMC_SCRATCH_SLAVE_ADDR_SHIFT		0
#define  PMC_SCRATCH_NUM_CMDS_SHIFT		8
#define  PMC_SCRATCH_16BIT_OP			BIT(15)
#define  PMC_SCRATCH_CHECKSUM_SHIFT		16
#define  PMC_SCRATCH_PINMUX_ID_SHIFT		24
#define  PMC_SCRATCH_CNTRL_ID_SHIFT		27
#define  PMC_SCRATCH_RST_EN			BIT(31)

struct tegra_pmc_soc {
	unsigned int num_powergates;
	const char *const *powergates;
	unsigned int num_cpu_powergates;
	const u8 *cpu_powergates;

	bool has_tsense_reset;
	bool has_gpu_clamps;
	bool has_ps18;
	bool has_bootrom_i2c;

	void (*utmi_sleep_enter)(unsigned int, enum usb_device_speed,
				 struct tegra_utmi_pad_config *);
	void (*utmi_sleep_exit)(unsigned int);
};

struct tegra_powergate_clk {
	struct clk *clk;
	struct list_head list;
};

struct tegra_slcg_clk {
	struct clk *clk;
	struct list_head list;
};

struct tegra_powergate_rst {
	struct reset_control *rst;
	struct list_head list;
};

struct tegra_powergate_flush {
	const struct tegra_mc_flush *flush;
	struct list_head list;
};

struct tegra_powergate {
	const char *name;
	struct list_head clk_list;
	struct list_head slcg_clk_list;
	struct list_head rst_list;
	struct tegra_mc *mc;
	struct list_head flush_list;
	struct raw_notifier_head slcg_notifier;
	unsigned int num_dependencies;
	unsigned int *dependencies;
};

/**
 * struct tegra_pmc - NVIDIA Tegra PMC
 * @dev: pointer to struct device
 * @base: pointer to I/O remapped register region
 * @clk: pointer to pclk clock
 * @rate: currently configured rate of pclk
 * @suspend_mode: lowest suspend mode available
 * @cpu_good_time: CPU power good time (in microseconds)
 * @cpu_off_time: CPU power off time (in microsecends)
 * @core_osc_time: core power good OSC time (in microseconds)
 * @core_pmu_time: core power good PMU time (in microseconds)
 * @core_off_time: core power off time (in microseconds)
 * @corereq_high: core power request is active-high
 * @sysclkreq_high: system clock request is active-high
 * @combined_req: combined power request for CPU & core
 * @cpu_pwr_good_en: CPU power good signal is enabled
 * @pmc_clk1_out_en: keep the PMC CLK1 output available in suspend
 * @pmc_clk1_out_src: the source of the PMC CLK1
 * @lp0_vec_phys: physical base address of the LP0 warm boot code
 * @lp0_vec_size: size of the LP0 warm boot code
 * @powergates_lock: mutex for power gate register access
 * @powergate_count: reference count for each powergate partition
 * @suspend_notifier: PM notifier for suspend events
 */
struct tegra_pmc {
	struct device *dev;
	void __iomem *base;
	struct clk *clk;

	const struct tegra_pmc_soc *soc;

	unsigned long rate;

	enum tegra_suspend_mode suspend_mode;
	u32 cpu_good_time;
	u32 cpu_off_time;
	u32 core_osc_time;
	u32 core_pmu_time;
	u32 core_off_time;
	bool corereq_high;
	bool sysclkreq_high;
	bool combined_req;
	bool cpu_pwr_good_en;
	bool pmc_clk1_out_en;
	u32 lp0_vec_phys;
	u32 lp0_vec_size;
	u32 pmc_clk1_out_src;

	struct mutex powergates_lock;
	unsigned int *powergate_count;
	struct tegra_powergate *powergates;

	struct notifier_block suspend_notifier;
};

#ifdef CONFIG_PM_SLEEP
#define PMC_WAKE_TYPE_INDEX	0
#define PMC_WAKE_MASK_INDEX	1
#define PMC_TRIGGER_TYPE_INDEX	2
#define PMC_OF_ARGS_COUNT	3
struct pmc_wakeup {
	u32 wake_type;
	u32 wake_mask_offset;
	u32 irq_num;
	struct list_head list;
};

struct pmc_lp0_wakeup {
	struct device_node *of_node;
	u64 enable;
	u64 level;
	u64 level_any;
	struct list_head wake_list;
};
static struct pmc_lp0_wakeup tegra_lp0_wakeup;
static u32 io_dpd_reg, io_dpd2_reg;
#endif

static u32 bootrom_i2c_header;

static struct tegra_pmc *pmc = &(struct tegra_pmc) {
	.base = NULL,
	.suspend_mode = TEGRA_SUSPEND_NONE,
};

static u32 tegra_pmc_readl(unsigned long offset)
{
	return readl(pmc->base + offset);
}

static void tegra_pmc_writel(u32 value, unsigned long offset)
{
	writel(value, pmc->base + offset);
}

static int tegra_pmc_readl_poll(unsigned long offset, u32 mask, u32 value)
{
	u32 regval;

	return readl_poll_timeout(pmc->base + offset, regval,
				  (regval & mask) == value, 10, 250);
}

/**
 * __tegra_powergate_set() - set the state of a partition
 * @id: partition ID
 * @new_state: new state of the partition
 */
static int __tegra_powergate_set(int id, bool new_state)
{
	bool status;

	status = tegra_pmc_readl(PWRGATE_STATUS) & (1 << id);

	if (status == new_state)
		return 0;

	tegra_pmc_writel(PWRGATE_TOGGLE_START | id, PWRGATE_TOGGLE);

	return tegra_pmc_readl_poll(PWRGATE_STATUS, BIT(id),
				    new_state ? BIT(id) : 0);
}

/**
 * tegra_powergate_power_on() - power on partition
 * @id: partition ID
 */
static int tegra_powergate_power_on(int id)
{
	int ret = 0;

	if (!pmc->soc || id < 0 || id >= pmc->soc->num_powergates)
		return -EINVAL;

	mutex_lock(&pmc->powergates_lock);
	if (pmc->powergate_count[id]++ == 0)
		ret = __tegra_powergate_set(id, true);
	mutex_unlock(&pmc->powergates_lock);

	return ret;
}

/**
 * tegra_powergate_is_powered() - check if partition is powered
 * @id: partition ID
 */
int tegra_powergate_is_powered(int id)
{
	u32 status;

	if (!pmc->soc || id < 0 || id >= pmc->soc->num_powergates)
		return -EINVAL;

	status = tegra_pmc_readl(PWRGATE_STATUS) & (1 << id);
	return !!status;
}

static int __tegra_powergate_remove_clamping(int id)
{
	u32 mask;

	/*
	 * Tegra 2 has a bug where PCIE and VDE clamping masks are
	 * swapped relatively to the partition ids
	 */
	if (id == TEGRA_POWERGATE_VDEC)
		mask = (1 << TEGRA_POWERGATE_PCIE);
	else if (id == TEGRA_POWERGATE_PCIE)
		mask = (1 << TEGRA_POWERGATE_VDEC);
	else
		mask = (1 << id);

	tegra_pmc_writel(mask, REMOVE_CLAMPING);

	return tegra_pmc_readl_poll(CLAMP_STATUS, mask, 0);
}

/**
 * tegra_powergate_remove_clamping() - remove power clamps for partition
 * @id: partition ID
 */
int tegra_powergate_remove_clamping(int id)
{
	int ret;

	if (!pmc->soc || id < 0 || id >= pmc->soc->num_powergates)
		return -EINVAL;

	mutex_lock(&pmc->powergates_lock);
	ret = __tegra_powergate_remove_clamping(id);
	mutex_unlock(&pmc->powergates_lock);

	return ret;
}
EXPORT_SYMBOL(tegra_powergate_remove_clamping);

/**
 * tegra_powergate_gpu_set_clamping - control GPU-SOC clamps
 *
 * The post-Tegra114 chips have a separate rail gating register to configure
 * clamps.
 *
 * @assert: true to assert clamp, and false to remove
 */
int tegra_powergate_gpu_set_clamping(bool assert)
{
	if (!pmc->soc)
		return -EINVAL;

	if (pmc->soc->has_gpu_clamps) {
		tegra_pmc_writel(assert ? 1 : 0, GPU_RG_CNTRL);
		tegra_pmc_readl(GPU_RG_CNTRL);
		return 0;
	}

	return -EINVAL;
}
EXPORT_SYMBOL(tegra_powergate_gpu_set_clamping);

/**
 * tegra_powergate_sequence_power_up() - power up partition
 * @id: partition ID
 */
static int tegra_powergate_sequence_power_up(int id)
{
	struct tegra_powergate_rst *pg_rst;
	struct tegra_powergate_clk *pg_clk;
	struct tegra_slcg_clk *slcg_clk;
	struct tegra_powergate_flush *pg_flush;
	struct tegra_powergate *pg;
	int ret;

	if (!pmc->soc || id < 0 || id >= pmc->soc->num_powergates)
		return -EINVAL;

	pg = &pmc->powergates[id];

	mutex_lock(&pmc->powergates_lock);
	if (pmc->powergate_count[id]++ == 0) {
		ret = __tegra_powergate_set(id, true);
		if (ret)
			goto err_unref;
		usleep_range(10, 20);

		list_for_each_entry(pg_clk, &pg->clk_list, list) {
			ret = clk_prepare_enable(pg_clk->clk);
			if (ret)
				goto err_pg_clk;
		}

		list_for_each_entry(pg_rst, &pg->rst_list, list)
			reset_control_assert(pg_rst->rst);
		usleep_range(10, 20);

		ret = __tegra_powergate_remove_clamping(id);
		if (ret)
			goto err_clamp;
		usleep_range(10, 20);

		list_for_each_entry(pg_rst, &pg->rst_list, list)
			reset_control_deassert(pg_rst->rst);
		usleep_range(10, 20);

		list_for_each_entry(pg_flush, &pg->flush_list, list)
			tegra_mc_flush(pg->mc, pg_flush->flush, false);
		usleep_range(10, 20);

		list_for_each_entry(slcg_clk, &pg->slcg_clk_list, list)
			clk_prepare_enable(slcg_clk->clk);

		raw_notifier_call_chain(&pg->slcg_notifier, 0, NULL);

		list_for_each_entry(slcg_clk, &pg->slcg_clk_list, list)
			clk_disable_unprepare(slcg_clk->clk);

		list_for_each_entry(pg_clk, &pg->clk_list, list)
			clk_disable_unprepare(pg_clk->clk);
	}
	mutex_unlock(&pmc->powergates_lock);

	return ret;

err_clamp:
	list_for_each_entry_continue_reverse(pg_rst, &pg->rst_list, list)
		reset_control_deassert(pg_rst->rst);
err_pg_clk:
	list_for_each_entry_continue_reverse(pg_clk, &pg->clk_list, list)
		clk_disable_unprepare(pg_clk->clk);
	__tegra_powergate_set(id, false);
err_unref:
	--pmc->powergate_count[id];
	mutex_unlock(&pmc->powergates_lock);
	return ret;
}

/**
 * tegra_powergate_sequence_power_down() - power down partition
 * @id: partition ID
 */
static int tegra_powergate_sequence_power_down(int id)
{
	struct tegra_powergate_rst *pg_rst;
	struct tegra_powergate_clk *pg_clk;
	struct tegra_powergate_flush *pg_flush;
	struct tegra_powergate *pg;
	int ret = 0;

	if (!pmc->soc || id < 0 || id >= pmc->soc->num_powergates)
		return -EINVAL;

	pg = &pmc->powergates[id];

	mutex_lock(&pmc->powergates_lock);
	if (WARN_ON(pmc->powergate_count[id] == 0)) {
		ret = -EINVAL;
	} else if (--pmc->powergate_count[id] == 0) {
		list_for_each_entry(pg_clk, &pg->clk_list, list) {
			ret = clk_prepare_enable(pg_clk->clk);
			if (ret)
				goto err_pg_clk_enable;
		}
		usleep_range(10, 20);

		list_for_each_entry(pg_flush, &pg->flush_list, list)
			tegra_mc_flush(pg->mc, pg_flush->flush, true);
		usleep_range(10, 20);

		list_for_each_entry(pg_rst, &pg->rst_list, list) {
			ret = reset_control_assert(pg_rst->rst);
			if (ret)
				goto err_pg_rst;
		}
		usleep_range(10, 20);

		list_for_each_entry(pg_clk, &pg->clk_list, list)
			clk_disable_unprepare(pg_clk->clk);
		usleep_range(10, 20);

		ret = __tegra_powergate_set(id, false);
		if (ret) {
			goto err_clk;
		}
	}
	mutex_unlock(&pmc->powergates_lock);
	return 0;

err_clk:
	list_for_each_entry(pg_clk, &pg->clk_list, list)
		clk_prepare_enable(pg_clk->clk);

err_pg_rst:
	list_for_each_entry_continue_reverse(pg_rst, &pg->rst_list, list)
		reset_control_deassert(pg_rst->rst);
	usleep_range(10, 20);

	list_for_each_entry(pg_flush, &pg->flush_list, list)
		tegra_mc_flush(pg->mc, pg_flush->flush, false);

err_pg_clk_enable:
	list_for_each_entry_continue_reverse(pg_clk, &pg->clk_list, list)
		clk_disable_unprepare(pg_clk->clk);

	++pmc->powergate_count[id];
	mutex_unlock(&pmc->powergates_lock);
	return ret;
}

/**
 * tegra_pmc_unpowergate() - unpowergate partition with dependency
 * @id: partition ID
 */
int tegra_pmc_unpowergate(int id)
{
	struct tegra_powergate *pg;
	int i, ret = 0;

	if (!pmc->soc || id < 0 || id >= pmc->soc->num_powergates)
		return -EINVAL;

	if (!pmc->powergates)
		return -EPROBE_DEFER;

	pg = &pmc->powergates[id];

	for (i = 0; i < pg->num_dependencies; i++) {
		ret = tegra_powergate_sequence_power_up(pg->dependencies[i]);
		if (ret)
			goto powergate_deps;
	}

	ret = tegra_powergate_sequence_power_up(id);
	if (ret)
		goto powergate_deps;

	return 0;

powergate_deps:
	for (; i > 0; i--)
		tegra_powergate_sequence_power_down(pg->dependencies[i - 1]);
	return ret;
}
EXPORT_SYMBOL(tegra_pmc_unpowergate);

/**
 * tegra_pmc_powergate - powergate partition with dependency
 * @id: partition ID
 */
int tegra_pmc_powergate(int id)
{
	struct tegra_powergate *pg;
	int i, ret = 0;

	if (!pmc->soc || id < 0 || id >= pmc->soc->num_powergates)
		return -EINVAL;

	if (!pmc->powergates)
		return -EPROBE_DEFER;

	pg = &pmc->powergates[id];

	ret = tegra_powergate_sequence_power_down(id);
	if (ret)
		return ret;

	for (i = 0; i < pg->num_dependencies; i++) {
		ret = tegra_powergate_sequence_power_down(pg->dependencies[i]);
		if (ret)
			goto unpowergate_deps;
	}

	return 0;

unpowergate_deps:
	for (; i > 0; i--)
		tegra_powergate_sequence_power_up(pg->dependencies[i - 1]);
	tegra_powergate_sequence_power_up(id);
	return ret;
}
EXPORT_SYMBOL(tegra_pmc_powergate);

#ifdef CONFIG_SMP
/**
 * tegra_get_cpu_powergate_id() - convert from CPU ID to partition ID
 * @cpuid: CPU partition ID
 *
 * Returns the partition ID corresponding to the CPU partition ID or a
 * negative error code on failure.
 */
static int tegra_get_cpu_powergate_id(int cpuid)
{
	if (pmc->soc && cpuid > 0 && cpuid < pmc->soc->num_cpu_powergates)
		return pmc->soc->cpu_powergates[cpuid];

	return -EINVAL;
}

/**
 * tegra_pmc_cpu_is_powered() - check if CPU partition is powered
 * @cpuid: CPU partition ID
 */
bool tegra_pmc_cpu_is_powered(int cpuid)
{
	int id;

	id = tegra_get_cpu_powergate_id(cpuid);
	if (id < 0)
		return false;

	return tegra_powergate_is_powered(id);
}

/**
 * tegra_pmc_cpu_power_on() - power on CPU partition
 * @cpuid: CPU partition ID
 */
int tegra_pmc_cpu_power_on(int cpuid)
{
	int id;

	id = tegra_get_cpu_powergate_id(cpuid);
	if (id < 0)
		return id;

	return tegra_powergate_power_on(id);
}

/**
 * tegra_pmc_cpu_remove_clamping() - remove power clamps for CPU partition
 * @cpuid: CPU partition ID
 */
int tegra_pmc_cpu_remove_clamping(int cpuid)
{
	int id;

	id = tegra_get_cpu_powergate_id(cpuid);
	if (id < 0)
		return id;

	return tegra_powergate_remove_clamping(id);
}
#endif /* CONFIG_SMP */

static int tegra_pmc_restart_notify(struct notifier_block *this,
				    unsigned long mode, void *cmd)
{
	u32 value;
	const char *cmd_str = (const char *) cmd;

	value = tegra_pmc_readl(PMC_SCRATCH0);
	value &= ~PMC_SCRATCH0_MODE_MASK;

	if (cmd_str) {
		if (strcmp(cmd_str, "recovery") == 0)
			value |= PMC_SCRATCH0_MODE_RECOVERY;

		if (strcmp(cmd_str, "bootloader") == 0)
			value |= PMC_SCRATCH0_MODE_BOOTLOADER;

		if (strcmp(cmd_str, "forced-recovery") == 0)
			value |= PMC_SCRATCH0_MODE_RCM;
	}

	tegra_pmc_writel(value, PMC_SCRATCH0);

	value = tegra_pmc_readl(0);
	value |= 0x10;
	tegra_pmc_writel(value, 0);

	return NOTIFY_DONE;
}

static struct notifier_block tegra_pmc_restart_handler = {
	.notifier_call = tegra_pmc_restart_notify,
	.priority = 128,
};

static int powergate_show(struct seq_file *s, void *data)
{
	unsigned int i;

	seq_printf(s, " powergate powered count\n");
	seq_printf(s, "------------------------\n");

	mutex_lock(&pmc->powergates_lock);
	for (i = 0; i < pmc->soc->num_powergates; i++) {
		if (!pmc->soc->powergates[i])
			continue;

		seq_printf(s, " %9s %7s %5u\n", pmc->soc->powergates[i],
			   tegra_powergate_is_powered(i) ? "yes" : "no",
			   pmc->powergate_count[i]);
	}
	mutex_unlock(&pmc->powergates_lock);

	return 0;
}

static int powergate_open(struct inode *inode, struct file *file)
{
	return single_open(file, powergate_show, inode->i_private);
}

static const struct file_operations powergate_fops = {
	.open = powergate_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int tegra_powergate_debugfs_init(void)
{
	struct dentry *d;

	d = debugfs_create_file("powergate", S_IRUGO, NULL, NULL,
				&powergate_fops);
	if (!d)
		return -ENOMEM;

	return 0;
}

static int tegra_io_rail_prepare(int id, unsigned long *request,
				 unsigned long *status, unsigned int *bit)
{
	unsigned long rate, value;
	struct clk *clk;

	*bit = id % 32;

	/*
	 * There are two sets of 30 bits to select IO rails, but bits 30 and
	 * 31 are control bits rather than IO rail selection bits.
	 */
	if (id > 63 || *bit == 30 || *bit == 31)
		return -EINVAL;

	if (id < 32) {
		*status = IO_DPD_STATUS;
		*request = IO_DPD_REQ;
	} else {
		*status = IO_DPD2_STATUS;
		*request = IO_DPD2_REQ;
	}

	clk = clk_get_sys(NULL, "pclk");
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	rate = clk_get_rate(clk);
	clk_put(clk);

	tegra_pmc_writel(DPD_SAMPLE_ENABLE, DPD_SAMPLE);

	/* must be at least 200 ns, in APB (PCLK) clock cycles */
	value = DIV_ROUND_UP(1000000000, rate);
	value = DIV_ROUND_UP(200, value);
	tegra_pmc_writel(value, SEL_DPD_TIM);

	return 0;
}

static void tegra_io_rail_unprepare(void)
{
	tegra_pmc_writel(DPD_SAMPLE_DISABLE, DPD_SAMPLE);
}

int tegra_io_rail_power_on(int id)
{
	unsigned long request, status, value;
	unsigned int bit, mask;
	int err;

	err = tegra_io_rail_prepare(id, &request, &status, &bit);
	if (err < 0)
		return err;

	mask = 1 << bit;

	value = tegra_pmc_readl(request);
	value |= mask;
	value &= ~IO_DPD_REQ_CODE_MASK;
	value |= IO_DPD_REQ_CODE_OFF;
	tegra_pmc_writel(value, request);

	err = tegra_pmc_readl_poll(status, mask, 0);
	if (err < 0)
		return err;

	tegra_io_rail_unprepare();

	return 0;
}
EXPORT_SYMBOL(tegra_io_rail_power_on);

int tegra_io_rail_power_off(int id)
{
	unsigned long request, status, value;
	unsigned int bit, mask;
	int err;

	err = tegra_io_rail_prepare(id, &request, &status, &bit);
	if (err < 0)
		return err;

	mask = 1 << bit;

	value = tegra_pmc_readl(request);
	value |= mask;
	value &= ~IO_DPD_REQ_CODE_MASK;
	value |= IO_DPD_REQ_CODE_ON;
	tegra_pmc_writel(value, request);

	err = tegra_pmc_readl_poll(status, mask, mask);
	if (err < 0)
		return err;

	tegra_io_rail_unprepare();

	return 0;
}
EXPORT_SYMBOL(tegra_io_rail_power_off);

static void
tegra124_utmi_sleep_enter(unsigned int port, enum usb_device_speed speed,
			  struct tegra_utmi_pad_config *config)
{
	u32 val;

	val = tegra_pmc_readl(PMC_UTMIP_UHSIC_SLEEP_CFG(port));
	if (val & UTMIP_MASTER_ENABLE(port))
		return;

	/*
	 * Set PMC MASTER bits to do the following:
	 * a. Take over the UTMI drivers.
	 * b. Take over resume if remote wakeup is detected.
	 * c. Take over suspend-wake detect-drive resume until USB controller
	 *    ready.
	 */

	/* Set UTMIP_PWR for low-power mode. */
	val = tegra_pmc_readl(PMC_UTMIP_MASTER_CONFIG);
	val |= UTMIP_PWR(port);
	tegra_pmc_writel(val, PMC_UTMIP_MASTER_CONFIG);

	/* Configure debouncer. */
	val = tegra_pmc_readl(PMC_USB_DEBOUNCE_DEL);
	val &= ~((UTMIP_LINE_DEB_CNT_MASK << UTMIP_LINE_DEB_CNT_SHIFT) |
		 (DEBOUNCE_VAL_MASK << DEBOUNCE_VAL_SHIFT));
	val |= (0x1 << UTMIP_LINE_DEB_CNT_SHIFT) |
		(0x2 << DEBOUNCE_VAL_SHIFT);
	tegra_pmc_writel(val, PMC_USB_DEBOUNCE_DEL);

	/* Make sure nothing is happening on the line with respect to PMC. */
	val = tegra_pmc_readl(PMC_UTMIP_UHSIC_FAKE(port));
	val &= ~(UTMIP_FAKE_USBOP_VAL(port) | UTMIP_FAKE_USBON_VAL(port));
	tegra_pmc_writel(val, PMC_UTMIP_UHSIC_FAKE(port));

	/* Clear line wake value. */
	val = tegra_pmc_readl(PMC_UTMIP_UHSIC_SLEEPWALK_CFG(port));
	val &= ~UTMIP_LINEVAL_WALK_EN(port);
	tegra_pmc_writel(val, PMC_UTMIP_UHSIC_SLEEPWALK_CFG(port));
	val = tegra_pmc_readl(PMC_UTMIP_UHSIC_SLEEP_CFG(port));
	val &= ~(UTMIP_WAKE_VAL_MASK << UTMIP_WAKE_VAL_SHIFT(port));
	val |= UTMIP_WAKE_VAL_NONE << UTMIP_WAKE_VAL_SHIFT(port);
	tegra_pmc_writel(val, PMC_UTMIP_UHSIC_SLEEP_CFG(port));

	/* Turn off pad detectors. */
	val = tegra_pmc_readl(PMC_USB_AO);
	val |= USBOP_VAL_PD(port) | USBON_VAL_PD(port);
	tegra_pmc_writel(val, PMC_USB_AO);

	/* Remove fake values and make synchronizers work a bit. */
	val = tegra_pmc_readl(PMC_UTMIP_UHSIC_FAKE(port));
	val &= ~(UTMIP_FAKE_USBOP_VAL(port) | UTMIP_FAKE_USBON_VAL(port));
	tegra_pmc_writel(val, PMC_UTMIP_UHSIC_FAKE(port));

	/* Enable walk on USB line value wake. */
	val = tegra_pmc_readl(PMC_UTMIP_UHSIC_SLEEPWALK_CFG(port));
	val |= UTMIP_LINEVAL_WALK_EN(port);
	tegra_pmc_writel(val, PMC_UTMIP_UHSIC_SLEEPWALK_CFG(port));

	/* Capture FS/LS pad configurations. */
	val = tegra_pmc_readl(PMC_UTMIP_UHSIC_TRIGGERS);
	val |= UTMIP_CAP_CFG(port);
	tegra_pmc_writel(val, PMC_UTMIP_UHSIC_TRIGGERS);

	/* Clear BIAS MASTER_ENABLE */
	val = tegra_pmc_readl(PMC_UTMIP_BIAS_MASTER_CNTRL);
	val &= ~UTMIP_BIAS_MASTER_PROG_VAL;
	tegra_pmc_writel(val, PMC_UTMIP_BIAS_MASTER_CNTRL);

	/* Program walk sequence for remote or hotplug wakeup. */
	val = tegra_pmc_readl(PMC_UTMIP_SLEEPWALK_PX(port));
	switch (speed) {
	case USB_SPEED_UNKNOWN:
		/*
		 * Program walk sequence: pull down both DP and DN lines,
		 * tristate lines once a hotplug-in wake event is detected.
		 */
		val &= ~(UTMIP_AP_A | UTMIP_AN_A);
		val |= UTMIP_USBOP_RPD_A | UTMIP_USBON_RPD_A | UTMIP_HIGHZ_A;
		val &= ~(UTMIP_AP_B | UTMIP_AN_B);
		val |= UTMIP_USBOP_RPD_B | UTMIP_USBON_RPD_B | UTMIP_HIGHZ_B;
		val &= ~(UTMIP_AP_C | UTMIP_AN_C);
		val |= UTMIP_USBOP_RPD_C | UTMIP_USBON_RPD_C | UTMIP_HIGHZ_C;
		val &= ~(UTMIP_AP_D | UTMIP_AN_D);
		val |= UTMIP_USBOP_RPD_D | UTMIP_USBON_RPD_D | UTMIP_HIGHZ_D;
		break;
	case USB_SPEED_LOW:
		/*
		 * Program walk sequence: maintain a J, followed by a driven K
		 * to signal a resume once an wake event is detected.
		 */
		val &= ~UTMIP_AP_A;
		val |= UTMIP_USBOP_RPD_A | UTMIP_USBON_RPD_A | UTMIP_AN_A |
			UTMIP_HIGHZ_A;
		val &= ~(UTMIP_AN_B | UTMIP_HIGHZ_B);
		val |= UTMIP_USBOP_RPD_B | UTMIP_USBON_RPD_B | UTMIP_AP_B;
		val &= ~(UTMIP_AN_C | UTMIP_HIGHZ_C);
		val |= UTMIP_USBOP_RPD_C | UTMIP_USBON_RPD_C | UTMIP_AP_C;
		val &= ~(UTMIP_AN_D | UTMIP_HIGHZ_D);
		val |= UTMIP_USBOP_RPD_D | UTMIP_USBON_RPD_D | UTMIP_AP_D;
		break;
	default:
		val &= ~UTMIP_AN_A;
		val |= UTMIP_USBOP_RPD_A | UTMIP_USBON_RPD_A | UTMIP_AP_A |
			UTMIP_HIGHZ_A;
		val &= ~(UTMIP_AP_B | UTMIP_HIGHZ_B);
		val |= UTMIP_USBOP_RPD_B | UTMIP_USBON_RPD_B | UTMIP_AN_B;
		val &= ~(UTMIP_AP_C | UTMIP_HIGHZ_C);
		val |= UTMIP_USBOP_RPD_C | UTMIP_USBON_RPD_C | UTMIP_AN_C;
		val &= ~(UTMIP_AP_D | UTMIP_HIGHZ_D);
		val |= UTMIP_USBOP_RPD_D | UTMIP_USBON_RPD_D | UTMIP_AN_D;
		break;
	}
	tegra_pmc_writel(val, PMC_UTMIP_SLEEPWALK_PX(port));

	/* Turn on pad detectors. */
	val = tegra_pmc_readl(PMC_USB_AO);
	val &= ~(USBOP_VAL_PD(port) | USBON_VAL_PD(port));
	tegra_pmc_writel(val, PMC_USB_AO);

	/* Add small delay before USB detectors provide stable line values. */
	usleep_range(1000, 1100);

	/* Program TCTRL, RCTRL values.*/
	val = tegra_pmc_readl(PMC_UTMIP_TERM_PAD_CFG);
	val = (fls(config->tctrl) << TCTRL_VAL_SHIFT) |
	      (fls(config->rctrl) << RCTRL_VAL_SHIFT);
	tegra_pmc_writel(val, PMC_UTMIP_TERM_PAD_CFG);

	/* Turn over pad configuration to PMC for line wake events. */
	val = tegra_pmc_readl(PMC_UTMIP_UHSIC_SLEEP_CFG(port));
	val &= ~(UTMIP_WAKE_VAL_MASK << UTMIP_WAKE_VAL_SHIFT(port));
	val |= (UTMIP_WAKE_VAL_ANY << UTMIP_WAKE_VAL_SHIFT(port)) |
		UTMIP_TCTRL_USE_PMC(port) | UTMIP_RCTRL_USE_PMC(port) |
		UTMIP_FSLS_USE_PMC(port) | UTMIP_MASTER_ENABLE(port);
	tegra_pmc_writel(val, PMC_UTMIP_UHSIC_SLEEP_CFG(port));
}

static void
tegra210_utmi_sleep_enter(unsigned int port, enum usb_device_speed speed,
			  struct tegra_utmi_pad_config *config)
{
	u32 val;

	val = tegra_pmc_readl(PMC_UTMIP_UHSIC_SLEEP_CFG(port));
	if (val & UTMIP_MASTER_ENABLE(port))
		return;

	/*
	 * Set PMC MASTER bits to do the following:
	 * a. Take over the UTMI drivers.
	 * b. Take over resume if remote wakeup is detected.
	 * c. Take over suspend-wake detect-drive resume until USB controller
	 *    ready.
	 */

	/* Set UTMIP_PWR for low-power mode. */
	val = tegra_pmc_readl(PMC_UTMIP_MASTER_CONFIG);
	val |= UTMIP_PWR(port);
	tegra_pmc_writel(val, PMC_UTMIP_MASTER_CONFIG);

	/* Configure debouncer. */
	val = tegra_pmc_readl(PMC_USB_DEBOUNCE_DEL);
	val &= ~((UTMIP_LINE_DEB_CNT_MASK << UTMIP_LINE_DEB_CNT_SHIFT) |
		 (DEBOUNCE_VAL_MASK << DEBOUNCE_VAL_SHIFT));
	val |= (0x1 << UTMIP_LINE_DEB_CNT_SHIFT) |
	       (0x2 << DEBOUNCE_VAL_SHIFT);
	tegra_pmc_writel(val, PMC_USB_DEBOUNCE_DEL);

	/* Make sure nothing is happening on the line with respect to PMC. */
	val = tegra_pmc_readl(PMC_UTMIP_UHSIC_FAKE(port));
	val &= ~(UTMIP_FAKE_USBOP_VAL(port) | UTMIP_FAKE_USBON_VAL(port));
	tegra_pmc_writel(val, PMC_UTMIP_UHSIC_FAKE(port));

	/* Clear line wake value. */
	val = tegra_pmc_readl(PMC_UTMIP_UHSIC_SLEEPWALK_CFG(port));
	val &= ~UTMIP_LINEVAL_WALK_EN(port);
	tegra_pmc_writel(val, PMC_UTMIP_UHSIC_SLEEPWALK_CFG(port));
	val = tegra_pmc_readl(PMC_UTMIP_UHSIC_SLEEP_CFG(port));
	val &= ~(UTMIP_WAKE_VAL_MASK << UTMIP_WAKE_VAL_SHIFT(port));
	val |= UTMIP_WAKE_VAL_NONE << UTMIP_WAKE_VAL_SHIFT(port);
	tegra_pmc_writel(val, PMC_UTMIP_UHSIC_SLEEP_CFG(port));

	/* Turn off pad detectors. */
	val = tegra_pmc_readl(PMC_USB_AO);
	val |= USBOP_VAL_PD(port) | USBON_VAL_PD(port);
	tegra_pmc_writel(val, PMC_USB_AO);

	/* Remove fake values and make synchronizers work a bit. */
	val = tegra_pmc_readl(PMC_UTMIP_UHSIC_FAKE(port));
	val &= ~(UTMIP_FAKE_USBOP_VAL(port) | UTMIP_FAKE_USBON_VAL(port));
	tegra_pmc_writel(val, PMC_UTMIP_UHSIC_FAKE(port));

	/* Enable walk on USB line value wake. */
	val = tegra_pmc_readl(PMC_UTMIP_UHSIC_SLEEPWALK_CFG(port));
	val |= UTMIP_LINEVAL_WALK_EN(port);
	tegra_pmc_writel(val, PMC_UTMIP_UHSIC_SLEEPWALK_CFG(port));

	/* Capture FS/LS pad configurations. */
	val = tegra_pmc_readl(PMC_UTMIP_UHSIC_TRIGGERS);
	val |= UTMIP_CAP_CFG(port);
	tegra_pmc_writel(val, PMC_UTMIP_UHSIC_TRIGGERS);

	/* Clear BIAS MASTER_ENABLE */
	val = tegra_pmc_readl(PMC_UTMIP_BIAS_MASTER_CNTRL);
	val &= ~UTMIP_BIAS_MASTER_PROG_VAL;
	tegra_pmc_writel(val, PMC_UTMIP_BIAS_MASTER_CNTRL);

	/* Program walk sequence for remote or hotplug wakeup. */
	val = tegra_pmc_readl(PMC_UTMIP_SLEEPWALK_PX(port));
	switch (speed) {
	case USB_SPEED_UNKNOWN:
		/*
		 * Program walk sequence: pull down both DP and DN lines,
		 * tristate lines once a hotplug-in wake event is detected.
		 */
		val &= ~(UTMIP_AP_A | UTMIP_AN_A);
		val |= UTMIP_USBOP_RPD_A | UTMIP_USBON_RPD_A | UTMIP_HIGHZ_A;
		val &= ~(UTMIP_AP_B | UTMIP_AN_B);
		val |= UTMIP_USBOP_RPD_B | UTMIP_USBON_RPD_B | UTMIP_HIGHZ_B;
		val &= ~(UTMIP_AP_C | UTMIP_AN_C);
		val |= UTMIP_USBOP_RPD_C | UTMIP_USBON_RPD_C | UTMIP_HIGHZ_C;
		val &= ~(UTMIP_AP_D | UTMIP_AN_D);
		val |= UTMIP_USBOP_RPD_D | UTMIP_USBON_RPD_D | UTMIP_HIGHZ_D;
		break;
	case USB_SPEED_LOW:
		/*
		 * Program walk sequence: maintain a J, followed by a driven K
		 * to signal a resume once an wake event is detected.
		 */
		val &= ~UTMIP_AP_A;
		val |= UTMIP_USBOP_RPD_A | UTMIP_USBON_RPD_A | UTMIP_AN_A |
			UTMIP_HIGHZ_A;
		val &= ~(UTMIP_AN_B | UTMIP_HIGHZ_B);
		val |= UTMIP_USBOP_RPD_B | UTMIP_USBON_RPD_B | UTMIP_AP_B;
		val &= ~(UTMIP_AN_C | UTMIP_HIGHZ_C);
		val |= UTMIP_USBOP_RPD_C | UTMIP_USBON_RPD_C | UTMIP_AP_C;
		val &= ~(UTMIP_AN_D | UTMIP_HIGHZ_D);
		val |= UTMIP_USBOP_RPD_D | UTMIP_USBON_RPD_D | UTMIP_AP_D;
		break;
	default:
		val &= ~UTMIP_AN_A;
		val |= UTMIP_USBOP_RPD_A | UTMIP_USBON_RPD_A | UTMIP_AN_A |
			UTMIP_HIGHZ_A;
		val &= ~(UTMIP_AP_B | UTMIP_HIGHZ_B);
		val |= UTMIP_USBOP_RPD_B | UTMIP_USBON_RPD_B | UTMIP_AN_B;
		val &= ~(UTMIP_AP_C | UTMIP_HIGHZ_C);
		val |= UTMIP_USBOP_RPD_C | UTMIP_USBON_RPD_C | UTMIP_AN_C;
		val &= ~(UTMIP_AP_D | UTMIP_HIGHZ_D);
		val |= UTMIP_USBOP_RPD_D | UTMIP_USBON_RPD_D | UTMIP_AN_D;
		break;
	}
	tegra_pmc_writel(val, PMC_UTMIP_SLEEPWALK_PX(port));

	/* Turn on pad detectors. */
	val = tegra_pmc_readl(PMC_USB_AO);
	val &= ~(USBOP_VAL_PD(port) | USBON_VAL_PD(port));
	tegra_pmc_writel(val, PMC_USB_AO);

	/* Add small delay before USB detectors provide stable line values. */
	usleep_range(5000, 5100);

	/* Program TCTRL, PCTRL values.*/
	val = tegra_pmc_readl(PMC_UTMIP_TERM_PAD_CFG);
	val |= ((config->tctrl & TEGRA210_TCTRL_VAL_MASK) <<
		TEGRA210_TCTRL_VAL_SHIFT) |
	       ((config->pctrl & PCTRL_VAL_MASK) << PCTRL_VAL_SHIFT);
	tegra_pmc_writel(val, PMC_UTMIP_TERM_PAD_CFG);

	/* Program RPD_CTRL into PMC space */
	val = tegra_pmc_readl(PMC_UTMIP_PAD_CFG(port));
	val &= ~UTMIP_RPD_CTRL(~0);
	val |= UTMIP_RPD_CTRL(config->rpd_ctrl);
	tegra_pmc_writel(val, PMC_UTMIP_PAD_CFG(port));

	/* Turn over pad configuration to PMC for line wake events. */
	val = tegra_pmc_readl(PMC_UTMIP_UHSIC_SLEEP_CFG(port));
	val &= ~(UTMIP_WAKE_VAL_MASK << UTMIP_WAKE_VAL_SHIFT(port));
	val |= (UTMIP_WAKE_VAL_ANY << UTMIP_WAKE_VAL_SHIFT(port)) |
		UTMIP_TCTRL_USE_PMC(port) | UTMIP_PCTRL_USE_PMC(port) |
		UTMIP_FSLS_USE_PMC(port) | UTMIP_MASTER_ENABLE(port);
	tegra_pmc_writel(val, PMC_UTMIP_UHSIC_SLEEP_CFG(port));

	val = tegra_pmc_readl(PMC_UTMIP_UHSIC_SLEEP_CFG(3));
	val |= UTMIP_RPD_CTRL_USE_PMC(port) |
	       UTMIP_RPU_SWITC_LOW_USE_PMC(port);
	tegra_pmc_writel(val, PMC_UTMIP_UHSIC_SLEEP_CFG(3));
}

int tegra_pmc_utmi_sleep_enter(unsigned int port, enum usb_device_speed speed,
			       struct tegra_utmi_pad_config *config)
{
	pr_debug("%s port %u, speed %u\n", __func__, port, speed);

	if (!pmc->soc->utmi_sleep_enter)
		return -ENOTSUPP;

	pmc->soc->utmi_sleep_enter(port, speed, config);
	return 0;
}
EXPORT_SYMBOL(tegra_pmc_utmi_sleep_enter);

static void tegra124_utmi_sleep_exit(unsigned int port)
{
	u32 val;

	val = tegra_pmc_readl(PMC_UTMIP_UHSIC_SLEEP_CFG(port));
	if (!(val & UTMIP_MASTER_ENABLE(port)))
		return;

	/* Clear line wake value. */
	val = tegra_pmc_readl(PMC_UTMIP_UHSIC_SLEEP_CFG(port));
	val &= ~(UTMIP_WAKE_VAL_MASK << UTMIP_WAKE_VAL_SHIFT(port));
	val |= UTMIP_WAKE_VAL_NONE << UTMIP_WAKE_VAL_SHIFT(port);
	tegra_pmc_writel(val, PMC_UTMIP_UHSIC_SLEEP_CFG(port));

	/* Release PMC pad control. */
	val = tegra_pmc_readl(PMC_UTMIP_UHSIC_SLEEP_CFG(port));
	val &= ~(UTMIP_FSLS_USE_PMC(port) | UTMIP_MASTER_ENABLE(port));
	val |= UTMIP_TCTRL_USE_PMC(port) | UTMIP_RCTRL_USE_PMC(port);
	tegra_pmc_writel(val, PMC_UTMIP_UHSIC_SLEEP_CFG(port));

	val = tegra_pmc_readl(PMC_UTMIP_UHSIC_TRIGGERS);
	val &= ~UTMIP_CAP_CFG(port);
	tegra_pmc_writel(val, PMC_UTMIP_UHSIC_TRIGGERS);

	/* Turn off pad detectors. */
	val = tegra_pmc_readl(PMC_USB_AO);
	val |= USBOP_VAL_PD(port) | USBON_VAL_PD(port);
	tegra_pmc_writel(val, PMC_USB_AO);

	val = tegra_pmc_readl(PMC_UTMIP_UHSIC_TRIGGERS);
	val |= UTMIP_CLR_WALK_PTR(port) | UTMIP_CLR_WAKE_ALARM(port);
	tegra_pmc_writel(val, PMC_UTMIP_UHSIC_TRIGGERS);
}

static void tegra210_utmi_sleep_exit(unsigned int port)
{
	u32 val;

	val = tegra_pmc_readl(PMC_UTMIP_UHSIC_SLEEP_CFG(port));
	if (!(val & UTMIP_MASTER_ENABLE(port)))
		return;

	/* Clear line wake value. */
	val = tegra_pmc_readl(PMC_UTMIP_UHSIC_SLEEP_CFG(port));
	val &= ~(UTMIP_WAKE_VAL_MASK << UTMIP_WAKE_VAL_SHIFT(port));
	val |= UTMIP_WAKE_VAL_NONE << UTMIP_WAKE_VAL_SHIFT(port);
	tegra_pmc_writel(val, PMC_UTMIP_UHSIC_SLEEP_CFG(port));

	/* Release PMC pad control. */
	val = tegra_pmc_readl(PMC_UTMIP_UHSIC_SLEEP_CFG(port));
	val &= ~(UTMIP_FSLS_USE_PMC(port) | UTMIP_MASTER_ENABLE(port));
	val |= UTMIP_TCTRL_USE_PMC(port) | UTMIP_PCTRL_USE_PMC(port);
	tegra_pmc_writel(val, PMC_UTMIP_UHSIC_SLEEP_CFG(port));

	val = tegra_pmc_readl(PMC_UTMIP_UHSIC_SLEEP_CFG(3));
	val &= ~(UTMIP_RPD_CTRL_USE_PMC(port) |
		 UTMIP_RPU_SWITC_LOW_USE_PMC(port));
	tegra_pmc_writel(val, PMC_UTMIP_UHSIC_SLEEP_CFG(3));

	val = tegra_pmc_readl(PMC_UTMIP_UHSIC_TRIGGERS);
	val &= ~UTMIP_CAP_CFG(port);
	tegra_pmc_writel(val, PMC_UTMIP_UHSIC_TRIGGERS);

	/* Turn off pad detectors. */
	val = tegra_pmc_readl(PMC_USB_AO);
	val |= USBOP_VAL_PD(port) | USBON_VAL_PD(port);
	tegra_pmc_writel(val, PMC_USB_AO);

	val = tegra_pmc_readl(PMC_UTMIP_UHSIC_TRIGGERS);
	val |= UTMIP_CLR_WALK_PTR(port) | UTMIP_CLR_WAKE_ALARM(port);
	tegra_pmc_writel(val, PMC_UTMIP_UHSIC_TRIGGERS);
}

int tegra_pmc_utmi_sleep_exit(unsigned int port)
{
	pr_debug("%s port %u\n", __func__, port);

	if (!pmc->soc->utmi_sleep_exit)
		return -ENOTSUPP;

	pmc->soc->utmi_sleep_exit(port);
	return 0;
}
EXPORT_SYMBOL(tegra_pmc_utmi_sleep_exit);

int tegra_pmc_hsic_sleep_enter(unsigned int port)
{
	u32 val;

	pr_debug("%s port %u\n", __func__, port);

	val = tegra_pmc_readl(PMC_UHSIC_SLEEP_CFG(port));
	if (val & UHSIC_MASTER_ENABLE(port))
		return 0;

	/*
	 * Set PMC MASTER bits to do the following:
	 * a. Take over the UHSIC drivers.
	 * b. Take over resume if remote wakeup is detected.
	 * c. Take over suspend-wake detect-drive resume until USB controller
	 *    ready.
	 */

	/* Set UHSIC_PWR for low-power mode. */
	val = tegra_pmc_readl(PMC_UHSIC_MASTER_CONFIG(port));
	val |= UHSIC_PWR(port);
	tegra_pmc_writel(val, PMC_UHSIC_MASTER_CONFIG(port));

	/* Configure debouncer. */
	val = tegra_pmc_readl(PMC_USB_DEBOUNCE_DEL);
	val &= ~(DEBOUNCE_VAL_MASK << DEBOUNCE_VAL_SHIFT);
	val |= 0x2 << DEBOUNCE_VAL_SHIFT;
	tegra_pmc_writel(val, PMC_USB_DEBOUNCE_DEL);

	/* Make sure nothing is happening on the line with respect to PMC. */
	val = tegra_pmc_readl(PMC_UTMIP_UHSIC_FAKE(port));
	val &= ~(UHSIC_FAKE_STROBE_VAL(port) | UHSIC_FAKE_DATA_VAL(port));
	tegra_pmc_writel(val, PMC_UTMIP_UHSIC_FAKE(port));

	/* Clear line wake value. */
	val = tegra_pmc_readl(PMC_UHSIC_SLEEPWALK_CFG(port));
	val &= ~UHSIC_LINEVAL_WALK_EN(port);
	tegra_pmc_writel(val, PMC_UHSIC_SLEEPWALK_CFG(port));
	val = tegra_pmc_readl(PMC_UHSIC_SLEEP_CFG(port));
	val &= ~(UHSIC_WAKE_VAL_MASK << UHSIC_WAKE_VAL_SHIFT(port));
	val |= UHSIC_WAKE_VAL_NONE << UHSIC_WAKE_VAL_SHIFT(port);
	tegra_pmc_writel(val, PMC_UHSIC_SLEEP_CFG(port));

	/* Turn on pad detectors. */
	val = tegra_pmc_readl(PMC_USB_AO);
	val &= ~(STROBE_VAL_PD(port) | DATA_VAL_PD(port));
	tegra_pmc_writel(val, PMC_USB_AO);

	/* Add small delay before USB detectors provide stable line values. */
	udelay(1);

	/* Enable walk on USB line value wake. */
	val = tegra_pmc_readl(PMC_UHSIC_SLEEPWALK_CFG(port));
	val |= UHSIC_LINEVAL_WALK_EN(port);
	tegra_pmc_writel(val, PMC_UHSIC_SLEEPWALK_CFG(port));

	/*
	 * Program walk sequence: maintain a J, followed by a driven K
	 * to signal a resume once a wake event is detected.
	 */
	val = tegra_pmc_readl(PMC_UHSIC_SLEEPWALK_PX(port));
	val &= ~(UHSIC_DATA_RPU_A | UHSIC_STROBE_RPD_A);
	val |= UHSIC_DATA_RPD_A | UHSIC_STROBE_RPU_A;
	val &= ~(UHSIC_DATA_RPD_B | UHSIC_STROBE_RPU_B);
	val |= UHSIC_DATA_RPU_B | UHSIC_STROBE_RPD_B;
	val &= ~(UHSIC_DATA_RPD_C | UHSIC_STROBE_RPU_C);
	val |= UHSIC_DATA_RPU_C | UHSIC_STROBE_RPD_C;
	val &= ~(UHSIC_DATA_RPD_D | UHSIC_STROBE_RPU_D);
	val |= UHSIC_DATA_RPU_D | UHSIC_STROBE_RPD_D;
	tegra_pmc_writel(val, PMC_UHSIC_SLEEPWALK_PX(port));

	/* Set wake event. */
	val = tegra_pmc_readl(PMC_UHSIC_SLEEP_CFG(port));
	val &= ~(UHSIC_WAKE_VAL_MASK << UHSIC_WAKE_VAL_SHIFT(port));
	val |= UHSIC_WAKE_VAL_SD10 << UHSIC_WAKE_VAL_SHIFT(port);
	tegra_pmc_writel(val, PMC_UHSIC_SLEEP_CFG(port));

	/* Clear the walk pointers and wake alarm. */
	val = tegra_pmc_readl(PMC_UHSIC_TRIGGERS(port));
	val |= UHSIC_CLR_WAKE_ALARM(port) | UHSIC_CLR_WALK_PTR(port);
	tegra_pmc_writel(val, PMC_UHSIC_TRIGGERS(port));

	/* Turn over pad configuration to PMC for line wake events. */
	val = tegra_pmc_readl(PMC_UHSIC_SLEEP_CFG(port));
	val |= UHSIC_MASTER_ENABLE(port);
	tegra_pmc_writel(val, PMC_UHSIC_SLEEP_CFG(port));

	return 0;
}
EXPORT_SYMBOL(tegra_pmc_hsic_sleep_enter);

int tegra_pmc_hsic_sleep_exit(unsigned int port)
{
	u32 val;

	pr_debug("%s port %u\n", __func__, port);

	val = tegra_pmc_readl(PMC_UHSIC_SLEEP_CFG(port));
	if (!(val & UHSIC_MASTER_ENABLE(port)))
		return 0;

	/* Clear line wake value. */
	val = tegra_pmc_readl(PMC_UHSIC_SLEEP_CFG(port));
	val &= ~(UHSIC_WAKE_VAL_MASK << UHSIC_WAKE_VAL_SHIFT(port));
	val |= UHSIC_WAKE_VAL_NONE << UHSIC_WAKE_VAL_SHIFT(port);
	tegra_pmc_writel(val, PMC_UHSIC_SLEEP_CFG(port));

	/* Release PMC pad control. */
	val = tegra_pmc_readl(PMC_UHSIC_SLEEP_CFG(port));
	val &= ~UHSIC_MASTER_ENABLE(port);
	tegra_pmc_writel(val, PMC_UHSIC_SLEEP_CFG(port));

	/* Turn off pad detectors. */
	val = tegra_pmc_readl(PMC_USB_AO);
	val |= STROBE_VAL_PD(port) | DATA_VAL_PD(port);
	tegra_pmc_writel(val, PMC_USB_AO);

	val = tegra_pmc_readl(PMC_UHSIC_TRIGGERS(port));
	val |= UHSIC_CLR_WAKE_ALARM(port) | UHSIC_CLR_WALK_PTR(port);
	tegra_pmc_writel(val, PMC_UHSIC_TRIGGERS(port));

	return 0;
}
EXPORT_SYMBOL(tegra_pmc_hsic_sleep_exit);

int tegra_fuse_ps18_latch_set(void)
{
	u32 reg;

	if (!pmc->soc->has_ps18)
		return -ENOSYS;

	reg = tegra_pmc_readl(PMC_FUSE_CTRL);
	reg &= ~(PMC_FUSE_CTRL_PS18_LATCH_CLEAR);
	tegra_pmc_writel(reg, PMC_FUSE_CTRL);
	mdelay(1);
	reg |= (PMC_FUSE_CTRL_PS18_LATCH_SET);
	tegra_pmc_writel(reg, PMC_FUSE_CTRL);
	mdelay(1);

	return 0;
}
EXPORT_SYMBOL(tegra_fuse_ps18_latch_set);

int tegra_fuse_ps18_latch_clear(void)
{
	u32 reg;

	if (!pmc->soc->has_ps18)
		return -ENOSYS;

	reg = tegra_pmc_readl(PMC_FUSE_CTRL);
	reg &= ~(PMC_FUSE_CTRL_PS18_LATCH_SET);
	tegra_pmc_writel(reg, PMC_FUSE_CTRL);
	mdelay(1);
	reg |= (PMC_FUSE_CTRL_PS18_LATCH_CLEAR);
	tegra_pmc_writel(reg, PMC_FUSE_CTRL);
	mdelay(1);

	return 0;
}
EXPORT_SYMBOL(tegra_fuse_ps18_latch_clear);

#ifdef CONFIG_PM_SLEEP
void tegra_tsc_suspend(void)
{
	if (IS_ENABLED(CONFIG_ARM_ARCH_TIMER)) {
		u32 reg;

		reg = tegra_pmc_readl(PMC_DPD_ENABLE);
		reg |= PMC_DPD_ENABLE_TSC_MULT_ENABLE;
		tegra_pmc_writel(reg, PMC_DPD_ENABLE);
	}
}

void tegra_tsc_resume(void)
{
	if (IS_ENABLED(CONFIG_ARM_ARCH_TIMER)) {
		u32 reg;

		reg = tegra_pmc_readl(PMC_DPD_ENABLE);
		reg &= ~PMC_DPD_ENABLE_TSC_MULT_ENABLE;
		switch (tegra_get_chip_id()) {
		case TEGRA124:
		case TEGRA132:
			/* WAR to avoid PMC wake status getting cleared */
			reg &= ~PMC_DPD_ENABLE_ON;
			break;
		default:
			break;
		}
		tegra_pmc_writel(reg, PMC_DPD_ENABLE);
	}
}

static void tegra_pmc_remove_dpd_req(void)
{
	/* Clear DPD req */
	tegra_pmc_writel(io_dpd_reg | IO_DPD_REQ_CODE_OFF, IO_DPD_REQ);
	tegra_pmc_readl(IO_DPD_REQ); /* unblock posted write */
	/* delay apb_clk * (SEL_DPD_TIM*5) */
	udelay(DPD_STATE_CHANGE_DELAY);

	tegra_pmc_writel(io_dpd2_reg | IO_DPD_REQ_CODE_OFF, IO_DPD2_REQ);
	tegra_pmc_readl(IO_DPD2_REQ); /* unblock posted write */
	udelay(DPD_STATE_CHANGE_DELAY);
}

void tegra_pmc_lp0_resume(void)
{
	tegra_pmc_remove_dpd_req();
}

static void tegra_pmc_clear_dpd_sample(void)
{
	/* Clear DPD sample */
	tegra_pmc_writel(0x0, DPD_SAMPLE);
}

static void tegra_pmc_add_wakeup_event(struct of_phandle_args *ph_args,
				       struct device *dev,
				       struct device_node *np)
{
	struct platform_device *pdev;
	struct pmc_wakeup *pmc_wake_source;
	struct irq_desc *irqd;
	struct irq_data *irq_data;
	int pmc_wake_type, wake;
	int irq, pmc_trigger_type;

	if (ph_args->np != tegra_lp0_wakeup.of_node)
		return;
	if (ph_args->args_count != PMC_OF_ARGS_COUNT)
		return;

	pdev = to_platform_device(dev);
	irq = platform_get_irq(pdev, 0);
	pmc_wake_type = ph_args->args[PMC_WAKE_TYPE_INDEX];

	switch (pmc_wake_type) {
	case PMC_WAKE_TYPE_GPIO:
		if (irq < 0) {
			int gpio;

			gpio = of_get_named_gpio(np, "gpios", 0);
			irq = gpio_to_irq(gpio);
			if (WARN_ON(irq < 0))
				return;
		}
		irqd = irq_to_desc(irq);
		irq_data = &irqd->irq_data;
		pmc_trigger_type = irqd_get_trigger_type(irq_data);
		break;
	case PMC_WAKE_TYPE_EVENT:
		pmc_trigger_type = ph_args->args[PMC_TRIGGER_TYPE_INDEX];
		break;
	default:
		return;
	}

	pmc_wake_source = kzalloc(sizeof(*pmc_wake_source), GFP_KERNEL);
	if (!pmc_wake_source)
		return;

	pmc_wake_source->wake_type = pmc_wake_type;
	pmc_wake_source->irq_num = irq;
	pmc_wake_source->wake_mask_offset = ph_args->args[PMC_WAKE_MASK_INDEX];
	wake = pmc_wake_source->wake_mask_offset;

	list_add_tail(&pmc_wake_source->list, &tegra_lp0_wakeup.wake_list);

	tegra_lp0_wakeup.enable |= 1ULL << wake;
	switch (pmc_trigger_type) {
	case IRQF_TRIGGER_FALLING:
	case IRQF_TRIGGER_LOW:
		tegra_lp0_wakeup.level &= ~(1ULL << wake);
		tegra_lp0_wakeup.level_any &= ~(1ULL << wake);
		break;
	case IRQF_TRIGGER_HIGH:
	case IRQF_TRIGGER_RISING:
		tegra_lp0_wakeup.level |= (1ULL << wake);
		tegra_lp0_wakeup.level_any &= ~(1ULL << wake);
		break;
	case IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING:
		tegra_lp0_wakeup.level_any |= (1ULL << wake);
		break;
	default:
		break;
	}
}

static void tegra_of_device_add_pmc_wake(struct device *dev)
{
	struct of_phandle_args ph_args;
	struct device_node *np = NULL;
	int child_node_num, i = 0;

	child_node_num = of_get_child_count(dev->of_node);
	if (child_node_num == 0) {
		while (!of_parse_phandle_with_args(dev->of_node,
						   "nvidia,pmc-wakeup",
						   "#nvidia,wake-cells",
						   i++, &ph_args))
			tegra_pmc_add_wakeup_event(&ph_args, dev, dev->of_node);
	} else {
		for_each_child_of_node(dev->of_node, np) {
			i = 0;
			while (!of_parse_phandle_with_args(np,
							   "nvidia,pmc-wakeup",
							   "#nvidia,wake-cells",
							   i++, &ph_args))
				tegra_pmc_add_wakeup_event(&ph_args, dev, np);
		}
	}

	of_node_put(ph_args.np);
}

static int tegra_pmc_wake_notifier_call(struct notifier_block *nb,
				      unsigned long event, void *data)
{
	struct device *dev = data;

	switch (event) {
	case BUS_NOTIFY_BOUND_DRIVER:
		if (dev->of_node)
			tegra_of_device_add_pmc_wake(dev);
		break;
	}
	return NOTIFY_DONE;
}

static struct notifier_block tegra_pmc_wake_notifier = {
	.notifier_call = tegra_pmc_wake_notifier_call,
};

static int __init tegra_pmc_lp0_wakeup_init(void)
{
	if (!soc_is_tegra())
		return 0;

	bus_register_notifier(&platform_bus_type, &tegra_pmc_wake_notifier);
	return 0;
}
arch_initcall(tegra_pmc_lp0_wakeup_init);

static inline void write_pmc_wake_mask(u64 value)
{
	pr_info("PMC wake enable = 0x%llx\n", value);
	tegra_pmc_writel((u32)value, PMC_WAKE_MASK);
	if (tegra_get_chip_id() != TEGRA20)
		tegra_pmc_writel((u32)(value >> 32), PMC_WAKE2_MASK);
}

static inline u64 read_pmc_wake_level(void)
{
	u64 reg;

	reg = tegra_pmc_readl(PMC_WAKE_LEVEL);
	if (tegra_get_chip_id() != TEGRA20)
		reg |= ((u64)tegra_pmc_readl(PMC_WAKE2_LEVEL)) << 32;

	return reg;
}

static inline void write_pmc_wake_level(u64 value)
{
	pr_info("PMC wake level = 0x%llx\n", value);
	tegra_pmc_writel((u32)value, PMC_WAKE_LEVEL);
	if (tegra_get_chip_id() != TEGRA20)
		tegra_pmc_writel((u32)(value >> 32), PMC_WAKE2_LEVEL);
}

static inline u64 read_pmc_wake_status(void)
{
	u64 reg;

	reg = tegra_pmc_readl(PMC_WAKE_STATUS);
	if (tegra_get_chip_id() != TEGRA20)
		reg |= ((u64)tegra_pmc_readl(PMC_WAKE2_STATUS)) << 32;

	return reg;
}

static inline void clear_pmc_wake_status(void)
{
	u32 reg;

	reg = tegra_pmc_readl(PMC_WAKE_STATUS);
	if (reg)
		tegra_pmc_writel(reg, PMC_WAKE_STATUS);
	if (tegra_get_chip_id() != TEGRA20) {
		reg = tegra_pmc_readl(PMC_WAKE2_STATUS);
		if (reg)
			tegra_pmc_writel(reg, PMC_WAKE2_STATUS);
	}
}

static inline u64 read_pmc_sw_wake_status(void)
{
	u64 reg;

	reg = tegra_pmc_readl(PMC_SW_WAKE_STATUS);
	if (tegra_get_chip_id() != TEGRA20)
		reg |= ((u64)tegra_pmc_readl(PMC_SW_WAKE2_STATUS)) << 32;

	return reg;
}

static inline void clear_pmc_sw_wake_status(void)
{
	tegra_pmc_writel(0, PMC_SW_WAKE_STATUS);
	if (tegra_get_chip_id() != TEGRA20)
		tegra_pmc_writel(0, PMC_SW_WAKE2_STATUS);
}

/* translate lp0 wake sources back into irqs to catch edge triggered wakeups */
static void tegra_pmc_wake_syscore_resume(void)
{
	struct pmc_wakeup *wake;
	struct irq_desc *desc;
	u64 wake_status = read_pmc_wake_status();

	pr_info("PMC wake status = 0x%llx\n", wake_status);

	list_for_each_entry(wake, &tegra_lp0_wakeup.wake_list, list) {
		if (!(wake_status & BIT(wake->wake_mask_offset)))
			continue;

		if (wake->irq_num <= 0) {
			pr_info("Resume caused by PMC WAKE%d\n",
				wake->wake_mask_offset);
			continue;
		}

		log_wakeup_reason(wake->irq_num);

		desc = irq_to_desc(wake->irq_num);
		if (!desc || !desc->action || !desc->action->name) {
			pr_info("Resume caused by PMC WAKE%d, irq %d\n",
				wake->wake_mask_offset, wake->irq_num);
			continue;
		}

		pr_info("Resume caused by PMC WAKE%d, %s\n",
			wake->wake_mask_offset, desc->action->name);
		generic_handle_irq(wake->irq_num);
	}
}

static int tegra_pmc_wake_syscore_suspend(void)
{
	u32 reg;
	u64 status;
	u64 lvl;
	u64 wake_level;
	u64 wake_enb;

	clear_pmc_sw_wake_status();

	/* enable PMC wake */
	reg = tegra_pmc_readl(PMC_CNTRL);
	reg |= PMC_CNTRL_LATCH_WAKEUPS;
	tegra_pmc_writel(reg, PMC_CNTRL);
	udelay(120);

	reg &= ~PMC_CNTRL_LATCH_WAKEUPS;
	tegra_pmc_writel(reg, PMC_CNTRL);
	udelay(120);

	status = read_pmc_sw_wake_status();

	lvl = read_pmc_wake_level();

	/*
	 * flip the wakeup trigger for any-edge triggered pads
	 * which are currently asserting as wakeups
	 */
	lvl ^= status;

	lvl &= tegra_lp0_wakeup.level_any;

	wake_level = lvl | tegra_lp0_wakeup.level;
	wake_enb = tegra_lp0_wakeup.enable;

	/* Clear PMC Wake Status registers while going to suspend */
	clear_pmc_wake_status();
	write_pmc_wake_level(wake_level);
	write_pmc_wake_mask(wake_enb);

	return 0;
}

static int tegra_pmc_suspend(void)
{
#ifdef CONFIG_ARM
	tegra_pmc_writel(virt_to_phys(tegra_resume), PMC_SCRATCH41);
#else /* CONFIG_ARM64 */
	enum tegra_suspend_mode mode = tegra_pmc_get_suspend_mode();
	tegra_pmc_enter_suspend_mode(mode);

#endif
	return 0;
}

static void tegra_pmc_resume(void)
{
	tegra_pmc_clear_dpd_sample();
	/* Clear DPD Enable */
	switch (tegra_get_chip_id()) {
	case TEGRA20:
	case TEGRA30:
	case TEGRA114:
		break;
	default:
		tegra_pmc_writel(0x0, PMC_DPD_ENABLE);
		break;
	}

	tegra_pmc_writel(0x0, PMC_SCRATCH41);

	/*
	 * SCRATCH250 (containing the BootROM I2C header) gets cleared
	 * in LP0.  Restore it here.
	 */
	tegra_pmc_writel(bootrom_i2c_header, PMC_SCRATCH250);
}

static void set_core_power_timers(void)
{
	unsigned long osc, pmu, off;

	osc = DIV_ROUND_UP_ULL(pmc->core_osc_time * 8192, 1000000);
	pmu = DIV_ROUND_UP_ULL(pmc->core_pmu_time * 32768, 1000000);
	off = DIV_ROUND_UP_ULL(pmc->core_off_time * 32768, 1000000);

	tegra_pmc_writel(((osc << 8) & 0xff00) | (pmu & 0xff),
			 PMC_COREPWRGOOD_TIMER);
	tegra_pmc_writel(off, PMC_COREPWROFF_TIMER);
}

enum tegra_suspend_mode tegra_pmc_get_suspend_mode(void)
{
	return pmc->suspend_mode;
}

void tegra_pmc_set_suspend_mode(enum tegra_suspend_mode mode)
{
	if (mode < TEGRA_SUSPEND_NONE || mode >= TEGRA_MAX_SUSPEND_MODE)
		return;

	pmc->suspend_mode = mode;
}

void tegra_pmc_enter_suspend_mode(enum tegra_suspend_mode mode)
{
	unsigned long long rate = 0;
	u32 boot_flag, cntrl_value;

	cntrl_value = tegra_pmc_readl(PMC_CNTRL);
	cntrl_value &= ~PMC_CNTRL_SIDE_EFFECT_LP0;
	if (pmc->combined_req)
		cntrl_value &= ~PMC_CNTRL_PWRREQ_OE;
	else
		cntrl_value |= PMC_CNTRL_PWRREQ_OE;
	cntrl_value |= PMC_CNTRL_CPU_PWRREQ_OE;

	switch (mode) {
	case TEGRA_SUSPEND_SC7:
		tegra_pmc_writel(pmc->lp0_vec_phys, PMC_SCRATCH1);
		rate = 32768;
		break;
	case TEGRA_SUSPEND_LP0:
		/*
		 * Enable DPD sample to trigger sampling pads data and direction
		 * in which pad will be driven during LP0 mode.
		 */
		tegra_pmc_writel(0x1, DPD_SAMPLE);

		/*
		 * Power down IO logic
		 */
		switch (tegra_get_chip_id()) {
		case TEGRA114:
		case TEGRA124:
		case TEGRA132:
			io_dpd_reg = IO_DPD_CSIA | IO_DPD_CSIB | IO_DPD_DSI |
				IO_DPD_MIPI_BIAS | IO_DPD_PEX_BIAS |
				IO_DPD_PEX_CLK1 | IO_DPD_PEX_CLK2 |
				IO_DPD_PEX_CLK3 | IO_DPD_DAC | IO_DPD_USB0 |
				IO_DPD_USB1 | IO_DPD_USB2 | IO_DPD_USB_BIAS |
				IO_DPD_UART | IO_DPD_BB | IO_DPD_VI |
				IO_DPD_AUDIO | IO_DPD_LCD | IO_DPD_HSIC;
			io_dpd2_reg = IO_DPD2_PEX_CNTRL | IO_DPD2_SDMMC1 |
				IO_DPD2_SDMMC3 | IO_DPD2_SDMMC4 | IO_DPD2_CAM |
				IO_DPD2_RES_RAIL | IO_DPD2_HV | IO_DPD2_DSIB |
				IO_DPD2_DSIC | IO_DPD2_DSID | IO_DPD2_CSIC |
				IO_DPD2_CSID | IO_DPD2_CSIE;
			break;
		default:
			break;
		}
		tegra_pmc_writel(io_dpd_reg | IO_DPD_REQ_CODE_ON, IO_DPD_REQ);
		tegra_pmc_readl(IO_DPD_REQ); /* unblock posted write */

		/* delay apb_clk * (SEL_DPD_TIM*5) */
		udelay(DPD_STATE_CHANGE_DELAY);

		tegra_pmc_writel(io_dpd2_reg | IO_DPD_REQ_CODE_ON, IO_DPD2_REQ);
		tegra_pmc_readl(IO_DPD2_REQ); /* unblock posted write */
		udelay(DPD_STATE_CHANGE_DELAY);

		/* Set warmboot flag */
		boot_flag = tegra_pmc_readl(PMC_SCRATCH0);
		tegra_pmc_writel(boot_flag | 1, PMC_SCRATCH0);

		tegra_pmc_writel(pmc->lp0_vec_phys, PMC_SCRATCH1);
		cntrl_value |= PMC_CNTRL_SIDE_EFFECT_LP0;
	case TEGRA_SUSPEND_LP1:
		rate = 32768;
		break;

	case TEGRA_SUSPEND_LP2:
		rate = clk_get_rate(pmc->clk);
		break;

	default:
		break;
	}

	if (WARN_ON_ONCE(rate == 0))
		rate = 100000000;

	if (rate != pmc->rate) {
		u64 ticks;

		ticks = pmc->cpu_good_time * rate + USEC_PER_SEC - 1;
		do_div(ticks, USEC_PER_SEC);
		tegra_pmc_writel(ticks, PMC_CPUPWRGOOD_TIMER);

		ticks = pmc->cpu_off_time * rate + USEC_PER_SEC - 1;
		do_div(ticks, USEC_PER_SEC);
		tegra_pmc_writel(ticks, PMC_CPUPWROFF_TIMER);

		wmb();

		pmc->rate = rate;
	}

	tegra_pmc_writel(cntrl_value, PMC_CNTRL);
}

/*
 * When starting to enter LP0 without LP0 boot code, try to request the
 * code with request_firmware, if it can't be loaded, switch to LP1.
 */
static int tegra_pmc_suspend_notifier(struct notifier_block *nb,
				      unsigned long event,
				      void *ptr)
{
	const struct firmware *fw;
	int ret;
	void *fw_buff;
	const char fw_name[] = "tegra_lp0_resume.fw";
	char fw_path[40];

	if (event != PM_SUSPEND_PREPARE)
		return 0;

	if (pmc->suspend_mode < TEGRA_SUSPEND_LP0 || pmc->lp0_vec_size)
		return 0;

	switch (tegra_get_chip_id()) {
	case TEGRA210:
		sprintf(fw_path, "nvidia/tegra210/%s", fw_name);
		break;
	default:
		break;
	}

	ret = request_firmware(&fw, fw_path, pmc->dev);
	if (ret) {
		dev_info(pmc->dev, "Disabling LP0, no resume code found\n");
		pmc->suspend_mode = TEGRA_SUSPEND_LP1;
		return 0;
	}

	fw_buff = (void *)__get_dma_pages(GFP_DMA32, get_order(fw->size));
	if (!fw_buff) {
		pmc->suspend_mode = TEGRA_SUSPEND_LP1;
		goto suspend_check_done;
	}
	dev_info(pmc->dev, "Loaded LP0 firmware with request_firmware.\n");

	memcpy(fw_buff, fw->data, fw->size);
	pmc->lp0_vec_phys = virt_to_phys(fw_buff);
	pmc->lp0_vec_size = fw->size;
suspend_check_done:
	release_firmware(fw);

	return 0;
}
#else
#define tegra_pmc_suspend		NULL
#define tegra_pmc_resume		NULL
#define tegra_pmc_wake_syscore_suspend	NULL
#define tegra_pmc_wake_syscore_resume	NULL
static inline void set_core_power_timers(void) { }
#endif

static struct syscore_ops tegra_pmc_syscore_ops = {
	.suspend = tegra_pmc_suspend,
	.resume = tegra_pmc_resume,
};

static struct syscore_ops tegra_pmc_wake_syscore_ops = {
	.suspend = tegra_pmc_wake_syscore_suspend,
	.resume = tegra_pmc_wake_syscore_resume,
};

static void tegra_pmc_syscore_init(void)
{
	register_syscore_ops(&tegra_pmc_syscore_ops);
}

static void tegra_pmc_wake_syscore_init(void)
{
	register_syscore_ops(&tegra_pmc_wake_syscore_ops);
}

static int tegra_pmc_parse_dt(struct tegra_pmc *pmc, struct device_node *np)
{
	u32 value, values[2];

	if (of_property_read_u32(np, "nvidia,suspend-mode", &value)) {
	} else {
		switch (value) {
		case 0:
			if (tegra_get_chip_id() == TEGRA210)
				pmc->suspend_mode = TEGRA_SUSPEND_SC7;
			else
				pmc->suspend_mode = TEGRA_SUSPEND_LP0;
			break;

		case 1:
			pmc->suspend_mode = TEGRA_SUSPEND_LP1;
			break;

		case 2:
			pmc->suspend_mode = TEGRA_SUSPEND_LP2;
			break;

		default:
			pmc->suspend_mode = TEGRA_SUSPEND_NONE;
			break;
		}
	}

	pmc->suspend_mode = tegra_pm_validate_suspend_mode(pmc->suspend_mode);

	if (of_property_read_u32(np, "nvidia,cpu-pwr-good-time", &value))
		pmc->suspend_mode = TEGRA_SUSPEND_NONE;

	pmc->cpu_good_time = value;

	if (of_property_read_u32(np, "nvidia,cpu-pwr-off-time", &value))
		pmc->suspend_mode = TEGRA_SUSPEND_NONE;

	pmc->cpu_off_time = value;

	if (of_property_read_u32_array(np, "nvidia,core-pwr-good-time",
				       values, ARRAY_SIZE(values)))
		pmc->suspend_mode = TEGRA_SUSPEND_NONE;

	pmc->core_osc_time = values[0];
	pmc->core_pmu_time = values[1];

	if (of_property_read_u32(np, "nvidia,core-pwr-off-time", &value))
		pmc->suspend_mode = TEGRA_SUSPEND_NONE;

	pmc->core_off_time = value;

	pmc->corereq_high = of_property_read_bool(np,
				"nvidia,core-power-req-active-high");

	pmc->sysclkreq_high = of_property_read_bool(np,
				"nvidia,sys-clock-req-active-high");

	pmc->combined_req = of_property_read_bool(np,
				"nvidia,combined-power-req");

	pmc->cpu_pwr_good_en = of_property_read_bool(np,
				"nvidia,cpu-pwr-good-en");

	values[0] = values[1] = 0;
	if (of_property_read_u32_array(np, "nvidia,lp0-vec", values,
				       ARRAY_SIZE(values)))
		if (pmc->suspend_mode == TEGRA_SUSPEND_LP0)
			pmc->suspend_mode = TEGRA_SUSPEND_LP1;

	pmc->lp0_vec_phys = values[0];
	pmc->lp0_vec_size = values[1];

	pmc->pmc_clk1_out_en = of_property_read_bool(np,
			"nvidia,pmc-clk1-out-en");
	if (of_property_read_u32(np, "nvidia,pmc-clk1-out-src", &value) == 0)
		pmc->pmc_clk1_out_src = value;

	return 0;
}

static void tegra_pmc_init(struct tegra_pmc *pmc)
{
	u32 value;

	/* Always enable CPU power request */
	value = tegra_pmc_readl(PMC_CNTRL);
	value |= PMC_CNTRL_CPU_PWRREQ_OE;
	tegra_pmc_writel(value, PMC_CNTRL);

	value = tegra_pmc_readl(PMC_CNTRL);

	if (pmc->sysclkreq_high)
		value &= ~PMC_CNTRL_SYSCLK_POLARITY;
	else
		value |= PMC_CNTRL_SYSCLK_POLARITY;

	if (!pmc->corereq_high)
		value |= PMC_CNTRL_PWRREQ_POLARITY;
	else
		value &= ~PMC_CNTRL_PWRREQ_POLARITY;

	/* configure the output polarity while the request is tristated */
	tegra_pmc_writel(value, PMC_CNTRL);

	/* now enable the request */
	value = tegra_pmc_readl(PMC_CNTRL);
	value |= PMC_CNTRL_SYSCLK_OE;
	tegra_pmc_writel(value, PMC_CNTRL);

	set_core_power_timers();
	tegra_pmc_syscore_init();
	tegra_pmc_wake_syscore_init();
}

void tegra_pmc_init_tsense_reset(struct tegra_pmc *pmc)
{
	static const char disabled[] = "emergency thermal reset disabled";
	u32 pmu_addr, ctrl_id, reg_addr, reg_data, pinmux;
	struct device *dev = pmc->dev;
	struct device_node *np;
	u32 value, checksum;

	if (!pmc->soc->has_tsense_reset)
		goto out;

	np = of_find_node_by_name(pmc->dev->of_node, "i2c-thermtrip");
	if (!np) {
		dev_warn(dev, "i2c-thermtrip node not found, %s.\n", disabled);
		goto out;
	}

	if (of_property_read_u32(np, "nvidia,i2c-controller-id", &ctrl_id)) {
		dev_err(dev, "I2C controller ID missing, %s.\n", disabled);
		goto out;
	}

	if (of_property_read_u32(np, "nvidia,bus-addr", &pmu_addr)) {
		dev_err(dev, "nvidia,bus-addr missing, %s.\n", disabled);
		goto out;
	}

	if (of_property_read_u32(np, "nvidia,reg-addr", &reg_addr)) {
		dev_err(dev, "nvidia,reg-addr missing, %s.\n", disabled);
		goto out;
	}

	if (of_property_read_u32(np, "nvidia,reg-data", &reg_data)) {
		dev_err(dev, "nvidia,reg-data missing, %s.\n", disabled);
		goto out;
	}

	if (of_property_read_u32(np, "nvidia,pinmux-id", &pinmux))
		pinmux = 0;

	value = tegra_pmc_readl(PMC_SENSOR_CTRL);
	value |= PMC_SENSOR_CTRL_SCRATCH_WRITE;
	tegra_pmc_writel(value, PMC_SENSOR_CTRL);

	value = (reg_data << PMC_SCRATCH54_DATA_SHIFT) |
		(reg_addr << PMC_SCRATCH54_ADDR_SHIFT);
	tegra_pmc_writel(value, PMC_SCRATCH54);

	value = PMC_SCRATCH55_RESET_TEGRA;
	value |= ctrl_id << PMC_SCRATCH55_CNTRL_ID_SHIFT;
	value |= pinmux << PMC_SCRATCH55_PINMUX_SHIFT;
	value |= pmu_addr << PMC_SCRATCH55_I2CSLV1_SHIFT;

	/*
	 * Calculate checksum of SCRATCH54, SCRATCH55 fields. Bits 23:16 will
	 * contain the checksum and are currently zero, so they are not added.
	 */
	checksum = reg_addr + reg_data + (value & 0xff) + ((value >> 8) & 0xff)
		+ ((value >> 24) & 0xff);
	checksum &= 0xff;
	checksum = 0x100 - checksum;

	value |= checksum << PMC_SCRATCH55_CHECKSUM_SHIFT;

	tegra_pmc_writel(value, PMC_SCRATCH55);

	value = tegra_pmc_readl(PMC_SENSOR_CTRL);
	value |= PMC_SENSOR_CTRL_ENABLE_RST;
	tegra_pmc_writel(value, PMC_SENSOR_CTRL);

	dev_info(pmc->dev, "emergency thermal reset enabled\n");

out:
	of_node_put(np);
	return;
}

#define PMC_MAX_I2C_COMMAND_BLOCKS 7
#define PMC_MAX_I2C_COMMANDS 63

static int
tegra_pmc_init_i2c_command_block(struct tegra_pmc *pmc, struct device_node *np,
				 unsigned int *offset)
{
	u32 reg_addr[PMC_MAX_I2C_COMMANDS], reg_data[PMC_MAX_I2C_COMMANDS];
	u32 ctrl_id, bus_addr, pinmux, value, header, checksum;
	bool addr_16;
	int cmds, i;

	if (of_property_read_u32(np, "nvidia,i2c-controller-id", &ctrl_id)) {
		dev_err(pmc->dev, "missing i2c controller id\n");
		return -EINVAL;
	}
	if (of_property_read_u32(np, "nvidia,bus-addr", &bus_addr)) {
		dev_err(pmc->dev, "missing i2c slave address\n");
		return -EINVAL;
	}
	if (of_property_read_u32(np, "nvidia,pinmux-id", &pinmux))
		pinmux = 0;
	addr_16 = of_property_read_bool(np, "nvidia,16-bit-addr");

	cmds = of_property_count_elems_of_size(np, "nvidia,reg-addr",
					       sizeof(u32));
	if (cmds <= 0 || cmds >= PMC_MAX_I2C_COMMANDS) {
		dev_err(pmc->dev, "invalid number of i2c register addresses\n");
		return -EINVAL;
	}
	if (of_property_count_elems_of_size(np, "nvidia,reg-data", sizeof(u32))
	    != cmds) {
		dev_err(pmc->dev, "invalid number of i2c register  values\n");
		return -EINVAL;
	}

	of_property_read_u32_array(np, "nvidia,reg-addr", reg_addr, cmds);
	of_property_read_u32_array(np, "nvidia,reg-data", reg_data, cmds);

	/* Set up the command block header. */
	value = (bus_addr << PMC_SCRATCH_SLAVE_ADDR_SHIFT) |
		(cmds << PMC_SCRATCH_NUM_CMDS_SHIFT) |
		(pinmux << PMC_SCRATCH_PINMUX_ID_SHIFT) |
		(ctrl_id << PMC_SCRATCH_CNTRL_ID_SHIFT) |
		PMC_SCRATCH_RST_EN;
	if (addr_16)
		value |= PMC_SCRATCH_16BIT_OP;
	tegra_pmc_writel(value, *offset);
	header = *offset;
	*offset += 0x4;

	/*
	 * Write the commands to the PMC scratch registers.  Two commands may
	 * be packed into a single register if 8-bit address/data is used.
	 */
	for (i = 0; i < cmds; i++) {
		if (addr_16) {
			tegra_pmc_writel(reg_addr[i] | (reg_data[i] << 16),
					 *offset);
			*offset += 0x4;
		} else if (i % 2) {
			value = tegra_pmc_readl(*offset);
			value |= (reg_addr[i] << 16) | (reg_data[i] << 24);
			tegra_pmc_writel(value, *offset);
			*offset += 0x4;
		} else {
			tegra_pmc_writel(reg_addr[i] | (reg_data[i] << 8),
					 *offset);
		}
	}
	if (!addr_16 && (cmds % 2))
		*offset += 0x4;

	/* Compute command block checksum. */
	checksum = 0;
	for (i = header; i < *offset; i += 0x4) {
		int j;

		value = tegra_pmc_readl(i);
		for (j = 0; j < 4; j++)
			checksum += (value >> (j * 8)) & 0xff;
	}
	checksum &= 0xff;
	checksum = 0x100 - checksum;

	value = tegra_pmc_readl(header);
	value |= checksum << PMC_SCRATCH_CHECKSUM_SHIFT;
	tegra_pmc_writel(value, header);

	return 0;
}

static void tegra_pmc_init_bootrom_i2c(struct tegra_pmc *pmc)
{
	u32 value, max_retries, transfer_delay, clear_delay;
	unsigned int blocks, offset, i;
	struct device_node *np;

	if (!pmc->soc->has_bootrom_i2c)
		return;

	np = of_find_node_by_name(pmc->dev->of_node, "bootrom-i2c");
	if (!np) {
		dev_warn(pmc->dev, "bootrom-i2c node not found\n");
		return;
	}
	blocks = of_get_child_count(np);
	if (!blocks) {
		dev_err(pmc->dev, "no bootrom i2c command blocks\n");
		goto out;
	}

	offset = PMC_SCRATCH251;
	for (i = 0; i < blocks && i < PMC_MAX_I2C_COMMAND_BLOCKS; i++) {
		char name[sizeof("command-block-n")];
		struct device_node *block_np;
		int err;

		snprintf(name, sizeof(name), "command-block-%u", i);
		block_np = of_find_node_by_name(np, name);
		if (!block_np) {
			dev_err(pmc->dev, "%s node not found\n", name);
			goto out;
		}

		err = tegra_pmc_init_i2c_command_block(pmc, block_np, &offset);
		if (err < 0) {
			dev_err(pmc->dev, "failed to init command block: %d\n",
				err);
			of_node_put(block_np);
			goto out;
		}

		of_node_put(block_np);
	}

	if (of_property_read_u32(np, "nvidia,transfer-retries", &max_retries))
		max_retries = 1;
	if (of_property_read_u32(np, "nvidia,bus-clear-delay", &clear_delay))
		clear_delay = 500;
	if (of_property_read_u32(np, "nvidia,transfer-delay", &transfer_delay))
		transfer_delay = 500;

	value = (max_retries << PMC_SCRATCH250_RETRIES_SHIFT) |
		(blocks << PMC_SCRATCH250_NUM_CONFIGS_SHIFT) |
		(order_base_2(transfer_delay) <<
		 PMC_SCRATCH250_TRANSFER_DELAY_SHIFT) |
		(order_base_2(clear_delay) <<
		 PMC_SCRATCH250_BUS_CLEAR_DELAY_SHIFT);
	tegra_pmc_writel(value, PMC_SCRATCH250);
	bootrom_i2c_header = value;

	dev_info(pmc->dev, "bootrom i2c command blocks initialized\n");

out:
	of_node_put(np);
}

static int tegra_powergate_get_id(const char *pg_name)
{
	int i;

	for (i = 0; i < pmc->soc->num_powergates; i++) {
		if (!pmc->soc->powergates[i])
			continue;

		if (strcmp(pg_name, pmc->soc->powergates[i]) == 0)
			return i;
	}

	return -EINVAL;
}

static int tegra_powergate_add_clock(struct tegra_powergate *pg,
				     struct clk *clk)
{
	struct tegra_powergate_clk *pg_clk;

	pg_clk = devm_kmalloc(pmc->dev, sizeof(*pg_clk), GFP_KERNEL);
	if (!pg_clk)
		return -ENOMEM;

	pg_clk->clk = clk;
	list_add_tail(&pg_clk->list, &pg->clk_list);

	return 0;
}

static int tegra_powergate_add_slcg_clock(struct tegra_powergate *pg,
				     struct clk *clk)
{
	struct tegra_slcg_clk *slcg_clk;

	slcg_clk = devm_kmalloc(pmc->dev, sizeof(*slcg_clk), GFP_KERNEL);
	if (!slcg_clk)
		return -ENOMEM;

	slcg_clk->clk = clk;
	list_add_tail(&slcg_clk->list, &pg->slcg_clk_list);

	return 0;
}

static int tegra_powergate_add_reset(struct tegra_powergate *pg,
				     struct reset_control *rst)
{
	struct tegra_powergate_rst *pg_rst;

	pg_rst = devm_kmalloc(pmc->dev, sizeof(*pg_rst), GFP_KERNEL);
	if (!pg_rst)
		return -ENOMEM;

	pg_rst->rst = rst;
	list_add_tail(&pg_rst->list, &pg->rst_list);

	return 0;
}

static int tegra_powergate_add_flush(struct tegra_powergate *pg,
				     const struct tegra_mc_flush *flush)
{
	struct tegra_powergate_flush *pg_flush;

	pg_flush = devm_kmalloc(pmc->dev, sizeof(*pg_flush), GFP_KERNEL);
	if (!pg_flush)
		return -ENOMEM;

	pg_flush->flush = flush;
	list_add_tail(&pg_flush->list, &pg->flush_list);

	return 0;
}

static int tegra_powergate_init_one(struct device_node *np,
				struct tegra_powergate *pg)
{
	struct platform_device *pdev;
	struct of_phandle_args args;
	struct device_node *pg_np;
	const char *id;
	int count;
	int i, ret = 0;

	count = of_property_count_strings(np, "clock-names");
	for (i = 0; i < count; ++i) {
		struct clk *clk;

		of_property_read_string_index(np, "clock-names",
						i, &id);
		clk = of_clk_get_by_name(np, id);
		if (IS_ERR(clk)) {
			pr_err("%s: Failed to get '%s' clock\n",
				__func__, id);
			continue;
		}

		if (strncmp(id, "slcg_", 5)) {
			if (tegra_powergate_add_clock(pg, clk))
				continue;
		} else {
			if (tegra_powergate_add_slcg_clock(pg, clk))
				continue;
		}

		pr_debug("%s: Added clock '%s' to powergate '%s'\n",
			__func__, id, pg->name);
	}

	count = of_property_count_strings(np, "reset-names");
	for (i = 0; i < count; ++i) {
		struct reset_control *rst;

		of_property_read_string_index(np, "reset-names",
						i, &id);
		rst = of_reset_control_get(np, id);
		if (IS_ERR(rst)) {
			pr_err("%s: Failed to get '%s' reset\n",
				__func__, id);
			continue;
		}

		ret = tegra_powergate_add_reset(pg, rst);
		if (ret)
			continue;

		pr_debug("%s: Added reset '%s' to powergate '%s'\n",
			__func__, id, pg->name);

	}

	count = of_property_count_strings(np, "swgroup-names");
	for (i = 0; i < count; i++) {
		const struct tegra_mc_flush *flush;

		ret = of_parse_phandle_with_fixed_args(np, "swgroups",
				1, i, &args);
		if (ret)
			continue;

		pdev = of_find_device_by_node(args.np);
		if (!pdev) {
			pr_err("%s: Failed to get mc device\n",
				__func__);
			continue;
		}

		if (!pg->mc) {
			pg->mc = platform_get_drvdata(pdev);
			if (!pg->mc) {
				pr_err("%s: Failed to get mc handler\n",
					__func__);
				return -EPROBE_DEFER;
			}
		}

		flush = tegra_mc_flush_get(pg->mc, args.args[0]);
		if (!flush) {
			pr_err("%s: Failed to get mc_flush for swgroup: %d\n",
				__func__, args.args[0]);
			continue;
		}

		of_property_read_string_index(np, "swgroup-names",
				i, &id);

		ret = tegra_powergate_add_flush(pg, flush);
		if (ret)
			continue;

		pr_debug("%s: Added mc_flush for swgroup '%s' to powergate '%s'\n",
			__func__, id, pg->name);
	}

	count = of_count_phandle_with_args(np, "power-partitions", NULL);
	if (count > 0) {
		pg->dependencies = devm_kcalloc(pmc->dev, count,
						sizeof(*pg->dependencies),
						GFP_KERNEL);
		if (!pg->dependencies) {
			pr_err("%s: Failed to allocate memory for dependency list\n",
				__func__);
			return -ENOMEM;
		}

		for (i = 0; i < count; i++) {
			pg_np = of_parse_phandle(np, "power-partitions", i);
			if (pg_np) {
				ret = tegra_powergate_get_id(pg_np->name);
				if (ret < 0) {
					pr_err("%s: Failed to get powergate ID for %s\n",
					       __func__, pg_np->name);
					of_node_put(pg_np);
					return ret;
				}
				pg->dependencies[i] = ret;
				of_node_put(pg_np);
			}
		}

		pg->num_dependencies = count;
	}

	return 0;
}

static int tegra_powergate_init(struct tegra_pmc *pmc,
				struct device_node *pmc_np)
{
	struct device_node *pgs_np;
	int ret = 0;
	int p;

	pmc->powergates = devm_kcalloc(pmc->dev, pmc->soc->num_powergates,
				       sizeof(*pmc->powergates),
				       GFP_KERNEL);
	if (!pmc->powergates)
		return -ENOMEM;

	for (p = 0; p < pmc->soc->num_powergates; ++p) {
		struct tegra_powergate *pg = &pmc->powergates[p];

		pg->name = pmc->soc->powergates[p];
		INIT_LIST_HEAD(&pg->clk_list);
		INIT_LIST_HEAD(&pg->slcg_clk_list);
		INIT_LIST_HEAD(&pg->rst_list);
		INIT_LIST_HEAD(&pg->flush_list);
	}

	pgs_np = of_get_child_by_name(pmc_np, "powergates");
	if (!pgs_np)
		return 0;

	for (p = 0; p < pmc->soc->num_powergates; ++p) {
		struct tegra_powergate *pg = &pmc->powergates[p];
		struct device_node *np;

		if (!pg->name)
			continue;

		np = of_get_child_by_name(pgs_np, pg->name);
		if (!np)
			continue;

		ret = tegra_powergate_init_one(np, pg);
	}

	return ret;
}

int tegra_slcg_register_notifier(int id, struct notifier_block *nb)
{
	struct tegra_powergate *pg;

	if (!pmc->soc || id < 0 || id >= pmc->soc->num_powergates)
		return -EINVAL;

	if (!pmc->powergates)
		return -EPROBE_DEFER;

	pg = &pmc->powergates[id];

	return raw_notifier_chain_register(&pg->slcg_notifier, nb);
}
EXPORT_SYMBOL(tegra_slcg_register_notifier);

int tegra_slcg_unregister_notifier(int id, struct notifier_block *nb)
{
	struct tegra_powergate *pg;

	if (!pmc->soc || id < 0 || id >= pmc->soc->num_powergates)
		return -EINVAL;

	if (!pmc->powergates)
		return -EPROBE_DEFER;

	pg = &pmc->powergates[id];

	return raw_notifier_chain_unregister(&pg->slcg_notifier, nb);
}
EXPORT_SYMBOL(tegra_slcg_unregister_notifier);

static int tegra_pmc_probe(struct platform_device *pdev)
{
	void __iomem *base = pmc->base;
	struct resource *res;
	int err;

	err = tegra_pmc_parse_dt(pmc, pdev->dev.of_node);
	if (err < 0)
		return err;

	/* take over the memory region from the early initialization */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pmc->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(pmc->base))
		return PTR_ERR(pmc->base);

	iounmap(base);

	pmc->clk = devm_clk_get(&pdev->dev, "pclk");
	if (IS_ERR(pmc->clk)) {
		err = PTR_ERR(pmc->clk);
		dev_err(&pdev->dev, "failed to get pclk: %d\n", err);
		return err;
	}

	pmc->dev = &pdev->dev;

	tegra_pmc_init(pmc);

	tegra_pmc_init_tsense_reset(pmc);

	tegra_pmc_init_bootrom_i2c(pmc);

	err = tegra_powergate_init(pmc, pdev->dev.of_node);
	if (err)
		return err;

	if (IS_ENABLED(CONFIG_DEBUG_FS)) {
		err = tegra_powergate_debugfs_init();
		if (err < 0)
			return err;
	}

	err = register_restart_handler(&tegra_pmc_restart_handler);
	if (err) {
		dev_err(&pdev->dev, "unable to register restart handler, %d\n",
			err);
		return err;
	}

	if (pmc->pmc_clk1_out_en) {
		u32 val;

		val = tegra_pmc_readl(PMC_CLK_OUT_CNTRL);
		val &= ~(PMC_CLK_OUT_CLK1_SRC_SEL_MASK);
		switch (pmc->pmc_clk1_out_src) {
		case PMC_CLK_OUT_SRC_OSC:
			val |= PMC_CLK_OUT_CLK1_SRC_SEL_DIV1;
			break;
		case PMC_CLK_OUT_SRC_OSC_DIV_2:
			val |= PMC_CLK_OUT_CLK1_SRC_SEL_DIV2;
			break;
		case PMC_CLK_OUT_SRC_OSC_DIV_4:
			val |= PMC_CLK_OUT_CLK1_SRC_SEL_DIV4;
			break;
		default:
			break;
		}
		tegra_pmc_writel(val, PMC_CLK_OUT_CNTRL);

		val = tegra_pmc_readl(PMC_CLK_OUT_CNTRL);
		val &= ~(PMC_CLK_OUT_CLK1_IDLE_STATE_MASK);
		tegra_pmc_writel(val, PMC_CLK_OUT_CNTRL);

		val = tegra_pmc_readl(PMC_CLK_OUT_CNTRL);
		val |= PMC_CLK_OUT_CLK1_ACCEPT_REQ;
		tegra_pmc_writel(val, PMC_CLK_OUT_CNTRL);

		val = tegra_pmc_readl(PMC_CLK_OUT_CNTRL);
		val |= PMC_CLK_OUT_CLK1_FORCE_EN;
		tegra_pmc_writel(val, PMC_CLK_OUT_CNTRL);

		val = tegra_pmc_readl(PMC_OSC_EDPD_OVER);
		val &= ~(OSC_EDPD_XO_LP0_MODE_MASK);
		val |= OSC_EDPD_OSC_CTRL_SELECT_PMC |
		       OSC_EDPD_XO_LP0_MODE_ON;
		tegra_pmc_writel(val, PMC_OSC_EDPD_OVER);
	}

#ifdef CONFIG_PM_SLEEP
	pmc->suspend_notifier.notifier_call = tegra_pmc_suspend_notifier;
	register_pm_notifier(&pmc->suspend_notifier);
#endif

	return 0;
}

static int __maybe_unused tegra_pmc_pm_suspend(struct device *dev)
{
	/*
	 * HACK: This works around a hang at resume time on certain boards
	 * for some unknown reason.
	 */
	return tegra_pmc_unpowergate(TEGRA_POWERGATE_DFD);
}

static int __maybe_unused tegra_pmc_pm_resume(struct device *dev)
{
	return tegra_pmc_powergate(TEGRA_POWERGATE_DFD);
}

static SIMPLE_DEV_PM_OPS(tegra_pmc_pm_ops, tegra_pmc_pm_suspend,
			 tegra_pmc_pm_resume);

static const char * const tegra20_powergates[] = {
	[TEGRA_POWERGATE_CPU] = "cpu",
	[TEGRA_POWERGATE_3D] = "3d",
	[TEGRA_POWERGATE_VENC] = "venc",
	[TEGRA_POWERGATE_VDEC] = "vdec",
	[TEGRA_POWERGATE_PCIE] = "pcie",
	[TEGRA_POWERGATE_L2] = "l2",
	[TEGRA_POWERGATE_MPE] = "mpe",
};

static const struct tegra_pmc_soc tegra20_pmc_soc = {
	.num_powergates = ARRAY_SIZE(tegra20_powergates),
	.powergates = tegra20_powergates,
	.num_cpu_powergates = 0,
	.cpu_powergates = NULL,
	.has_tsense_reset = false,
	.has_gpu_clamps = false,
	.has_bootrom_i2c = false,
};

static const char * const tegra30_powergates[] = {
	[TEGRA_POWERGATE_CPU] = "cpu0",
	[TEGRA_POWERGATE_3D] = "3d0",
	[TEGRA_POWERGATE_VENC] = "venc",
	[TEGRA_POWERGATE_VDEC] = "vdec",
	[TEGRA_POWERGATE_PCIE] = "pcie",
	[TEGRA_POWERGATE_L2] = "l2",
	[TEGRA_POWERGATE_MPE] = "mpe",
	[TEGRA_POWERGATE_HEG] = "heg",
	[TEGRA_POWERGATE_SATA] = "sata",
	[TEGRA_POWERGATE_CPU1] = "cpu1",
	[TEGRA_POWERGATE_CPU2] = "cpu2",
	[TEGRA_POWERGATE_CPU3] = "cpu3",
	[TEGRA_POWERGATE_CELP] = "celp",
	[TEGRA_POWERGATE_3D1] = "3d1",
};

static const u8 tegra30_cpu_powergates[] = {
	TEGRA_POWERGATE_CPU,
	TEGRA_POWERGATE_CPU1,
	TEGRA_POWERGATE_CPU2,
	TEGRA_POWERGATE_CPU3,
};

static const struct tegra_pmc_soc tegra30_pmc_soc = {
	.num_powergates = ARRAY_SIZE(tegra30_powergates),
	.powergates = tegra30_powergates,
	.num_cpu_powergates = ARRAY_SIZE(tegra30_cpu_powergates),
	.cpu_powergates = tegra30_cpu_powergates,
	.has_tsense_reset = true,
	.has_gpu_clamps = false,
	.has_bootrom_i2c = false,
};

static const char * const tegra114_powergates[] = {
	[TEGRA_POWERGATE_CPU] = "crail",
	[TEGRA_POWERGATE_3D] = "3d",
	[TEGRA_POWERGATE_VENC] = "venc",
	[TEGRA_POWERGATE_VDEC] = "vdec",
	[TEGRA_POWERGATE_MPE] = "mpe",
	[TEGRA_POWERGATE_HEG] = "heg",
	[TEGRA_POWERGATE_CPU1] = "cpu1",
	[TEGRA_POWERGATE_CPU2] = "cpu2",
	[TEGRA_POWERGATE_CPU3] = "cpu3",
	[TEGRA_POWERGATE_CELP] = "celp",
	[TEGRA_POWERGATE_CPU0] = "cpu0",
	[TEGRA_POWERGATE_C0NC] = "c0nc",
	[TEGRA_POWERGATE_C1NC] = "c1nc",
	[TEGRA_POWERGATE_DIS] = "dis",
	[TEGRA_POWERGATE_DISB] = "disb",
	[TEGRA_POWERGATE_XUSBA] = "xusba",
	[TEGRA_POWERGATE_XUSBB] = "xusbb",
	[TEGRA_POWERGATE_XUSBC] = "xusbc",
};

static const u8 tegra114_cpu_powergates[] = {
	TEGRA_POWERGATE_CPU0,
	TEGRA_POWERGATE_CPU1,
	TEGRA_POWERGATE_CPU2,
	TEGRA_POWERGATE_CPU3,
};

static const struct tegra_pmc_soc tegra114_pmc_soc = {
	.num_powergates = ARRAY_SIZE(tegra114_powergates),
	.powergates = tegra114_powergates,
	.num_cpu_powergates = ARRAY_SIZE(tegra114_cpu_powergates),
	.cpu_powergates = tegra114_cpu_powergates,
	.has_tsense_reset = true,
	.has_gpu_clamps = false,
	.has_bootrom_i2c = false,
};

static const char * const tegra124_powergates[] = {
	[TEGRA_POWERGATE_CPU] = "crail",
	[TEGRA_POWERGATE_3D] = "3d",
	[TEGRA_POWERGATE_VENC] = "venc",
	[TEGRA_POWERGATE_PCIE] = "pcie",
	[TEGRA_POWERGATE_VDEC] = "vdec",
	[TEGRA_POWERGATE_L2] = "l2",
	[TEGRA_POWERGATE_MPE] = "mpe",
	[TEGRA_POWERGATE_HEG] = "heg",
	[TEGRA_POWERGATE_SATA] = "sata",
	[TEGRA_POWERGATE_CPU1] = "cpu1",
	[TEGRA_POWERGATE_CPU2] = "cpu2",
	[TEGRA_POWERGATE_CPU3] = "cpu3",
	[TEGRA_POWERGATE_CELP] = "celp",
	[TEGRA_POWERGATE_CPU0] = "cpu0",
	[TEGRA_POWERGATE_C0NC] = "c0nc",
	[TEGRA_POWERGATE_C1NC] = "c1nc",
	[TEGRA_POWERGATE_SOR] = "sor",
	[TEGRA_POWERGATE_DIS] = "dis",
	[TEGRA_POWERGATE_DISB] = "disb",
	[TEGRA_POWERGATE_XUSBA] = "xusba",
	[TEGRA_POWERGATE_XUSBB] = "xusbb",
	[TEGRA_POWERGATE_XUSBC] = "xusbc",
	[TEGRA_POWERGATE_VIC] = "vic",
	[TEGRA_POWERGATE_IRAM] = "iram",
};

static const u8 tegra124_cpu_powergates[] = {
	TEGRA_POWERGATE_CPU0,
	TEGRA_POWERGATE_CPU1,
	TEGRA_POWERGATE_CPU2,
	TEGRA_POWERGATE_CPU3,
};

static const struct tegra_pmc_soc tegra124_pmc_soc = {
	.num_powergates = ARRAY_SIZE(tegra124_powergates),
	.powergates = tegra124_powergates,
	.num_cpu_powergates = ARRAY_SIZE(tegra124_cpu_powergates),
	.cpu_powergates = tegra124_cpu_powergates,
	.has_tsense_reset = true,
	.has_gpu_clamps = true,
	.has_bootrom_i2c = false,
	.utmi_sleep_enter = tegra124_utmi_sleep_enter,
	.utmi_sleep_exit = tegra124_utmi_sleep_exit,
};

static const char * const tegra210_powergates[] = {
	[TEGRA_POWERGATE_CPU] = "crail",
	[TEGRA_POWERGATE_3D] = "3d",
	[TEGRA_POWERGATE_VENC] = "venc",
	[TEGRA_POWERGATE_PCIE] = "pcie",
	[TEGRA_POWERGATE_L2] = "l2",
	[TEGRA_POWERGATE_MPE] = "mpe",
	[TEGRA_POWERGATE_HEG] = "heg",
	[TEGRA_POWERGATE_SATA] = "sata",
	[TEGRA_POWERGATE_CPU1] = "cpu1",
	[TEGRA_POWERGATE_CPU2] = "cpu2",
	[TEGRA_POWERGATE_CPU3] = "cpu3",
	[TEGRA_POWERGATE_CELP] = "celp",
	[TEGRA_POWERGATE_CPU0] = "cpu0",
	[TEGRA_POWERGATE_C0NC] = "c0nc",
	[TEGRA_POWERGATE_C1NC] = "c1nc",
	[TEGRA_POWERGATE_SOR] = "sor",
	[TEGRA_POWERGATE_DIS] = "dis",
	[TEGRA_POWERGATE_DISB] = "disb",
	[TEGRA_POWERGATE_XUSBA] = "xusba",
	[TEGRA_POWERGATE_XUSBB] = "xusbb",
	[TEGRA_POWERGATE_XUSBC] = "xusbc",
	[TEGRA_POWERGATE_VIC] = "vic",
	[TEGRA_POWERGATE_IRAM] = "iram",
	[TEGRA_POWERGATE_NVDEC] = "nvdec",
	[TEGRA_POWERGATE_NVJPG] = "nvjpg",
	[TEGRA_POWERGATE_AUD] = "aud",
	[TEGRA_POWERGATE_DFD] = "dfd",
	[TEGRA_POWERGATE_VE2] = "ve2",
};

static const u8 tegra210_cpu_powergates[] = {
	TEGRA_POWERGATE_CPU0,
	TEGRA_POWERGATE_CPU1,
	TEGRA_POWERGATE_CPU2,
	TEGRA_POWERGATE_CPU3,
};

static const struct tegra_pmc_soc tegra210_pmc_soc = {
	.num_powergates = ARRAY_SIZE(tegra210_powergates),
	.powergates = tegra210_powergates,
	.num_cpu_powergates = ARRAY_SIZE(tegra210_cpu_powergates),
	.cpu_powergates = tegra210_cpu_powergates,
	.has_tsense_reset = true,
	.has_gpu_clamps = true,
	.has_ps18 = true,
	.has_bootrom_i2c = true,
	.utmi_sleep_enter = tegra210_utmi_sleep_enter,
	.utmi_sleep_exit = tegra210_utmi_sleep_exit,
};

static const struct of_device_id tegra_pmc_match[] = {
	{ .compatible = "nvidia,tegra210-pmc", .data = &tegra210_pmc_soc },
	{ .compatible = "nvidia,tegra124-pmc", .data = &tegra124_pmc_soc },
	{ .compatible = "nvidia,tegra114-pmc", .data = &tegra114_pmc_soc },
	{ .compatible = "nvidia,tegra30-pmc", .data = &tegra30_pmc_soc },
	{ .compatible = "nvidia,tegra20-pmc", .data = &tegra20_pmc_soc },
	{ }
};

static struct platform_driver tegra_pmc_driver = {
	.driver = {
		.name = "tegra-pmc",
		.suppress_bind_attrs = true,
		.of_match_table = tegra_pmc_match,
		.pm = &tegra_pmc_pm_ops,
	},
	.probe = tegra_pmc_probe,
};
module_platform_driver(tegra_pmc_driver);

static int tegra_pmc_reboot_notifier(struct notifier_block *nb,
				     unsigned long action, void *data)
{
	u32 value;

	value = tegra_pmc_readl(PMC_SCRATCH202);
	value &= ~PMC_SCRATCH202_BOOTREASON_MASK;
	value |= PMC_SCRATCH202_BOOTREASON_REBOOT;
	tegra_pmc_writel(value, PMC_SCRATCH202);

	return NOTIFY_OK;
}

static struct notifier_block tegra_pmc_reboot_nb = {
	.notifier_call = tegra_pmc_reboot_notifier,
};

static int tegra_pmc_panic_notifier(struct notifier_block *nb,
				     unsigned long action, void *data)
{
	u32 value;

	value = tegra_pmc_readl(PMC_SCRATCH202);
	value &= ~PMC_SCRATCH202_BOOTREASON_MASK;
	value |= PMC_SCRATCH202_BOOTREASON_PANIC;
	tegra_pmc_writel(value, PMC_SCRATCH202);

	return NOTIFY_OK;
}

static struct notifier_block tegra_pmc_panic_nb = {
	.notifier_call = tegra_pmc_panic_notifier,
};

/*
 * Early initialization to allow access to registers in the very early boot
 * process.
 */
static int __init tegra_pmc_early_init(void)
{
	const struct of_device_id *match;
	struct device_node *np;
	struct resource regs;
	bool invert;
	u32 value;

	if (!soc_is_tegra())
		return 0;

	np = of_find_matching_node_and_match(NULL, tegra_pmc_match, &match);
	if (!np) {
		pr_warn("PMC device node not found, disabling powergating\n");

		regs.start = 0x7000e400;
		regs.end = 0x7000e7ff;
		regs.flags = IORESOURCE_MEM;

		pr_warn("Using memory region %pR\n", &regs);
	} else {
		pmc->soc = match->data;
	}

	if (of_address_to_resource(np, 0, &regs) < 0) {
		pr_err("failed to get PMC registers\n");
		return -ENXIO;
	}

	pmc->base = ioremap_nocache(regs.start, resource_size(&regs));
	if (!pmc->base) {
		pr_err("failed to map PMC registers\n");
		return -ENXIO;
	}

	mutex_init(&pmc->powergates_lock);
	if (pmc->soc) {
		pmc->powergate_count = kcalloc(pmc->soc->num_powergates,
					       sizeof(*pmc->powergate_count),
					       GFP_KERNEL);
		if (!pmc->powergate_count) {
			iounmap(pmc->base);
			return -ENOMEM;
		}
	}

	invert = of_property_read_bool(np, "nvidia,invert-interrupt");

	value = tegra_pmc_readl(PMC_CNTRL);

	if (invert)
		value |= PMC_CNTRL_INTR_POLARITY;
	else
		value &= ~PMC_CNTRL_INTR_POLARITY;

	tegra_pmc_writel(value, PMC_CNTRL);

#ifdef CONFIG_PM_SLEEP
	tegra_lp0_wakeup.of_node = np;
	INIT_LIST_HEAD(&tegra_lp0_wakeup.wake_list);
#endif

	register_reboot_notifier(&tegra_pmc_reboot_nb);
	atomic_notifier_chain_register(&panic_notifier_list,
				       &tegra_pmc_panic_nb);

	return 0;
}
early_initcall(tegra_pmc_early_init);
