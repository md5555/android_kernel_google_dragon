/*
 * arch/arm/mach-tegra/power.h
 *
 * Header for tegra power
 *
 * Copyright (c) 2010, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include "nvos.h"
#include "nvrm_init.h"
#include "nvrm_drf.h"
#include "ap20/arapbpm.h"
#include "nvrm_module.h"
#include "ap20/arflow_ctlr.h"
#include "ap20/arclk_rst.h"
#include "ap20/arapb_misc.h"
#include "ap20/arapbdma.h"
#include "ap20/arapbdmachan.h"
#include "ap20/armc.h"
#include "ap20/arahb_arbc.h"
#include "ap20/aremc.h"
#include "ap15/arictlr.h"
#include "ap15/argpio.h"
#include "nvrm_hardware_access.h"
#include "nvrm_interrupt.h"
#include "nvrm_power.h"
#include "nvrm_power_private.h"
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include "ap20/nvboot_pmc_scratch_map.h"

extern NvRmDeviceHandle s_hRmGlobal;

#define NUM_LOCAL_TIMER_REGISTERS 3
#define WAKE_PAD_MIN_LATCH_TIME_US 130
#define WAKE_PAD_MIN_SAMPLE_TIME_US 70

#define NV_CAR_REGR(pBase, reg)\
		NV_READ32( (((NvUPtr)(pBase)) + CLK_RST_CONTROLLER_##reg##_0))
#define NV_CAR_REGW(pBase, reg, val)\
		NV_WRITE32( (((NvUPtr)(pBase)) + CLK_RST_CONTROLLER_##reg##_0), (val))
#define NV_CAR_REGR_OFFSET(pBase, off)\
		NV_READ32( (((NvUPtr)(pBase)) + off))
#define NV_CAR_REGW_OFFSET(pBase, off, val)\
		NV_WRITE32( (((NvUPtr)(pBase)) + off), (val))

#define NV_ICTLR_REGR(pBase, reg)\
		NV_READ32( (((NvUPtr)(pBase)) + ICTLR_##reg##_0))
#define NV_ICTLR_REGW(pBase, reg, val)\
		NV_WRITE32( (((NvUPtr)(pBase)) + ICTLR_##reg##_0), (val))

#define NV_MISC_REGR(pBase, reg)\
		NV_READ32( (((NvUPtr)(pBase)) + APB_MISC_##reg##_0))
#define NV_MISC_REGW(pBase, reg, val)\
		NV_WRITE32( (((NvUPtr)(pBase)) + APB_MISC_##reg##_0), (val))

#define NV_GPIO_REGR(pBase, reg)\
		NV_READ32( (((NvUPtr)(pBase)) + GPIO_##reg))
#define NV_GPIO_REGW(pBase, reg, val)\
		NV_WRITE32( (((NvUPtr)(pBase)) + GPIO_##reg), (val))

#define NV_APBDMA_REGR(pBase, reg)\
		NV_READ32( (((NvUPtr)(pBase)) + APBDMA_##reg##_0))
#define NV_APBDMA_REGW(pBase, reg, val)\
		NV_WRITE32( (((NvUPtr)(pBase)) + APBDMA_##reg##_0), (val))

#define NV_APBDMACH_REGR(pBase, reg)\
		NV_READ32( (((NvUPtr)(pBase)) + APBDMACHAN_CHANNEL_0_##reg##_0))
#define NV_APBDMACH_REGW(pBase, reg, val)\
		NV_WRITE32( (((NvUPtr)(pBase)) + APBDMACHAN_CHANNEL_0_##reg##_0),(val))

#define NV_MC_REGR(pBase, reg)\
		NV_READ32( (((NvUPtr)(pBase)) + MC_##reg##_0))
#define NV_MC_REGW(pBase, reg, val)\
		NV_WRITE32( (((NvUPtr)(pBase)) + MC_##reg##_0), (val))

#define NV_PMC_REGR(pBase, reg)\
		NV_READ32( (((NvUPtr)(pBase)) + APBDEV_PMC_##reg##_0))
#define NV_PMC_REGW(pBase, reg, val)\
		NV_WRITE32( (((NvUPtr)(pBase)) + APBDEV_PMC_##reg##_0), (val))

#define CAR_CLK_SOURCES_OFFSET_START	CLK_RST_CONTROLLER_CLK_SOURCE_I2S1_0
#define CAR_CLK_SOURCES_OFFSET_END		CLK_RST_CONTROLLER_CLK_SOURCE_OSC_0
#define CAR_CLK_SOURCES_REGISTER_COUNT\
	((CAR_CLK_SOURCES_OFFSET_END - CAR_CLK_SOURCES_OFFSET_START +\
		sizeof(NvU32)) / sizeof(NvU32))

#define NV_DR_REGR(d,r)\
		NV_READ32( ((NvUPtr)(g_p##d)) + d##_##r##_0)


//------------------------------------------------------------------------------
// Boot ROM PMC scratch map name remapping to fix broken names (see bug 542815).
//------------------------------------------------------------------------------

//Correct name \ Broken name from nvboot_pmc_scratch_map.h
#define APBDEV_PMC_SCRATCH3_0_CLK_RST_CONTROLLER_PLLX_BASE_0_PLLX_DIVP_RANGE\
	APBDEV_PMC_SCRATCH3_0_CLK_RST_PLLX_BASE_PLLX_DIVP_RANGE
#define APBDEV_PMC_SCRATCH3_0_CLK_RST_CONTROLLER_PLLX_BASE_0_PLLX_DIVN_RANGE\
	APBDEV_PMC_SCRATCH3_0_CLK_RST_PLLX_BASE_PLLX_DIVN_RANGE
#define APBDEV_PMC_SCRATCH3_0_CLK_RST_CONTROLLER_PLLX_BASE_0_PLLX_DIVM_RANGE\
	APBDEV_PMC_SCRATCH3_0_CLK_RST_PLLX_BASE_PLLX_DIVM_RANGE
#define APBDEV_PMC_SCRATCH3_0_CLK_RST_CONTROLLER_PLLX_MISC_0_PLLX_CPCON_RANGE\
	APBDEV_PMC_SCRATCH3_0_CLK_RST_PLLX_MISC_CPCON_RANGE
#define APBDEV_PMC_SCRATCH3_0_CLK_RST_CONTROLLER_PLLX_MISC_0_PLLX_LFCON_RANGE\
	APBDEV_PMC_SCRATCH3_0_CLK_RST_PLLX_MISC_LFCON_RANGE

/** NV_SDRF_NUM - define a new scratch register value.
	@param s scratch register name (APBDEV_PMC_s)
	@param d register domain (hardware block)
	@param r register name
	@param f register field
	@param n defined value for the field
 */
#define NV_SDRF_NUM(s,d,r,f,n) \
	(((n)& NV_FIELD_MASK(APBDEV_PMC_##s##_0_##d##_##r##_0_##f##_RANGE)) << \
		NV_FIELD_SHIFT(APBDEV_PMC_##s##_0_##d##_##r##_0_##f##_RANGE))

/** NV_FLD_SET_SDRF_NUM - modify a scratch register field.
	@param s scratch register name (APBDEV_PMC_s)
	@param d register domain (hardware block)
	@param r register name
	@param f register field
	@param n numeric field value
 */
#define NV_FLD_SET_SDRF_NUM(s,d,r,f,n) \
	((s & ~NV_FIELD_SHIFTMASK(APBDEV_PMC_##s##_0_##d##_##r##_0_##f##_RANGE)) |\
		NV_SDRF_NUM(s,d,r,f,n))


/** SHADOW_REGS() - Shadowed PMC scratch registers that must be saved/restored
					across low power transitions because they are used by RM
					for other purposes.
	SHADOW_REG(s) - Shadowed PMC scratch register name:
	@param s Scratch register name (APBDEV_PMC_s)
 */
#define SHADOW_REGS()\
		SHADOW_REG(SCRATCH20)   \
		SHADOW_REG(SCRATCH21)   \
		/* End-of-List*/


/** SCRATCH_REGS() - PMC scratch registers (list of SCRATCH_REG() macros).
	SCRATCH_REG(s) - PMC scratch register name:

	@param s Scratch register name (APBDEV_PMC_s)
 */
#define SCRATCH_REGS()			\
		SCRATCH_REG(SCRATCH3)	\
		SCRATCH_REG(SCRATCH5)	\
		SCRATCH_REG(SCRATCH6)	\
		SCRATCH_REG(SCRATCH7)	\
		SCRATCH_REG(SCRATCH8)	\
		SCRATCH_REG(SCRATCH9)	\
		SCRATCH_REG(SCRATCH10)	\
		SCRATCH_REG(SCRATCH11)	\
		SCRATCH_REG(SCRATCH12)	\
		SCRATCH_REG(SCRATCH13)	\
		SCRATCH_REG(SCRATCH14)	\
		SCRATCH_REG(SCRATCH15)	\
		SCRATCH_REG(SCRATCH16)	\
		SCRATCH_REG(SCRATCH17)	\
		SCRATCH_REG(SCRATCH18)	\
		SCRATCH_REG(SCRATCH19)	\
		SCRATCH_REG(SCRATCH20)	\
		SCRATCH_REG(SCRATCH21)	\
		SCRATCH_REG(SCRATCH22)	\
		SCRATCH_REG(SCRATCH23)	\
		SCRATCH_REG(SCRATCH25)	\
		SCRATCH_REG(SCRATCH35)	\
		SCRATCH_REG(SCRATCH36)	\
		SCRATCH_REG(SCRATCH40)	\
		/* End-of-List*/

/** REGS()		- Scratch mapping registers (list of REG() macros).
	REG(s,d,r,f) - Scratch mapping register entry:

	@param s scratch register name (APBDEV_PMC_s)
	@param d register domain (hardware block)
	@param r register name
	@param f register field
 */
#define REGS() \
		/* AHB Group */ \
		REG(SCRATCH25, AHB, ARBITRATION_XBAR_CTRL, POST_DIS) \
		REG(SCRATCH25, AHB, ARBITRATION_XBAR_CTRL, HOLD_DIS) \
		REG(SCRATCH25, AHB, ARBITRATION_XBAR_CTRL, MEM_INIT_DONE) \
		/* CLK_RST Group */ \
		REG(SCRATCH3, CLK_RST_CONTROLLER, PLLX_BASE, PLLX_DIVP) \
		REG(SCRATCH3, CLK_RST_CONTROLLER, PLLX_BASE, PLLX_DIVN) \
		REG(SCRATCH3, CLK_RST_CONTROLLER, PLLX_BASE, PLLX_DIVM) \
		REG(SCRATCH3, CLK_RST_CONTROLLER, PLLX_MISC, PLLX_CPCON) \
		REG(SCRATCH3, CLK_RST_CONTROLLER, PLLX_MISC, PLLX_LFCON) \
		/* EMC Group */ \
		/**/ \
		REG(SCRATCH5, EMC, R2W, R2W) \
		REG(SCRATCH5, EMC, RAS, RAS) \
		REG(SCRATCH5, EMC, RC, RC) \
		REG(SCRATCH5, EMC, RFC, RFC) \
		REG(SCRATCH5, EMC, RP, RP) \
		/**/ \
		REG(SCRATCH6, EMC, R2P, R2P) \
		REG(SCRATCH6, EMC, RD_RCD, RD_RCD) \
		REG(SCRATCH6, EMC, RRD, RRD) \
		REG(SCRATCH6, EMC, W2P, W2P) \
		REG(SCRATCH6, EMC, W2R, W2R) \
		REG(SCRATCH6, EMC, WR_RCD, WR_RCD) \
		/**/ \
		REG(SCRATCH7, EMC, CFG_2, CLKCHANGE_SR_ENABLE) \
		REG(SCRATCH7, EMC, CFG_2, USE_ADDR_CLK) \
		REG(SCRATCH7, EMC, PCHG2PDEN, PCHG2PDEN) \
		REG(SCRATCH7, EMC, QRST, QRST) \
		REG(SCRATCH7, EMC, QSAFE, QSAFE) \
		REG(SCRATCH7, EMC, QUSE, QUSE) \
		REG(SCRATCH7, EMC, RDV, RDV) \
		REG(SCRATCH7, EMC, REXT, REXT) \
		REG(SCRATCH7, EMC, WDV, WDV) \
		/**/ \
		REG(SCRATCH8, EMC, BURST_REFRESH_NUM, BURST_REFRESH_NUM) \
		REG(SCRATCH8, EMC, PDEX2RD, PDEX2RD) \
		REG(SCRATCH8, EMC, PDEX2WR, PDEX2WR) \
		REG(SCRATCH8, EMC, REFRESH, REFRESH_LO) \
		REG(SCRATCH8, EMC, REFRESH, REFRESH) \
		REG(SCRATCH8, EMC, TCLKSTABLE, TCLKSTABLE) \
		/**/ \
		REG(SCRATCH9, EMC, ACT2PDEN, ACT2PDEN) \
		REG(SCRATCH9, EMC, AR2PDEN, AR2PDEN) \
		REG(SCRATCH9, EMC, RW2PDEN, RW2PDEN) \
		REG(SCRATCH9, EMC, TCKE, TCKE) \
		REG(SCRATCH9, EMC, TXSR, TXSR) \
		/**/ \
		REG(SCRATCH10, EMC, DBG, AP_REQ_BUSY_CTRL) \
		REG(SCRATCH10, EMC, DBG, CFG_PRIORITY) \
		REG(SCRATCH10, EMC, DBG, FORCE_UPDATE) \
		REG(SCRATCH10, EMC, DBG, MRS_WAIT) \
		REG(SCRATCH10, EMC, DBG, PERIODIC_QRST) \
		REG(SCRATCH10, EMC, DBG, READ_DQM_CTRL) \
		REG(SCRATCH10, EMC, DBG, READ_MUX) \
		REG(SCRATCH10, EMC, DBG, WRITE_MUX) \
		REG(SCRATCH10, EMC, TCLKSTOP, TCLKSTOP) \
		REG(SCRATCH10, EMC, TREFBW, TREFBW) \
		REG(SCRATCH10, EMC, TRPAB, TRPAB) \
		/**/ \
		REG(SCRATCH11, EMC, FBIO_DQSIB_DLY_MSB, CFG_DQSIB_DLY_MSB_BYTE_0) \
		REG(SCRATCH11, EMC, FBIO_DQSIB_DLY_MSB, CFG_DQSIB_DLY_MSB_BYTE_1) \
		REG(SCRATCH11, EMC, FBIO_DQSIB_DLY_MSB, CFG_DQSIB_DLY_MSB_BYTE_2) \
		REG(SCRATCH11, EMC, FBIO_DQSIB_DLY_MSB, CFG_DQSIB_DLY_MSB_BYTE_3) \
		REG(SCRATCH11, EMC, FBIO_QUSE_DLY_MSB, CFG_QUSE_DLY_MSB_BYTE_0) \
		REG(SCRATCH11, EMC, FBIO_QUSE_DLY_MSB, CFG_QUSE_DLY_MSB_BYTE_1) \
		REG(SCRATCH11, EMC, FBIO_QUSE_DLY_MSB, CFG_QUSE_DLY_MSB_BYTE_2) \
		REG(SCRATCH11, EMC, FBIO_QUSE_DLY_MSB, CFG_QUSE_DLY_MSB_BYTE_3) \
		REG(SCRATCH11, EMC, QUSE_EXTRA, QUSE_EXTRA) \
		REG(SCRATCH11, EMC, TFAW, TFAW) \
		/**/ \
		REG(SCRATCH12, EMC, FBIO_DQSIB_DLY, CFG_DQSIB_DLY_BYTE_0) \
		REG(SCRATCH12, EMC, FBIO_DQSIB_DLY, CFG_DQSIB_DLY_BYTE_1) \
		REG(SCRATCH12, EMC, FBIO_DQSIB_DLY, CFG_DQSIB_DLY_BYTE_2) \
		REG(SCRATCH12, EMC, FBIO_DQSIB_DLY, CFG_DQSIB_DLY_BYTE_3) \
		/**/ \
		REG(SCRATCH13, EMC, FBIO_QUSE_DLY, CFG_QUSE_DLY_BYTE_0) \
		REG(SCRATCH13, EMC, FBIO_QUSE_DLY, CFG_QUSE_DLY_BYTE_1) \
		REG(SCRATCH13, EMC, FBIO_QUSE_DLY, CFG_QUSE_DLY_BYTE_2) \
		REG(SCRATCH13, EMC, FBIO_QUSE_DLY, CFG_QUSE_DLY_BYTE_3) \
		/**/ \
		REG(SCRATCH14, EMC, CFG_CLKTRIM_0, CFG_DATA0_CLKTRIM) \
		REG(SCRATCH14, EMC, CFG_CLKTRIM_0, CFG_DATA1_CLKTRIM) \
		REG(SCRATCH14, EMC, CFG_CLKTRIM_0, CFG_DATA2_CLKTRIM) \
		REG(SCRATCH14, EMC, CFG_CLKTRIM_0, CFG_DATA3_CLKTRIM) \
		REG(SCRATCH14, EMC, CFG_CLKTRIM_0, CFG_MCLK_ADDR_CLKTRIM) \
		/**/ \
		REG(SCRATCH15, EMC, AUTO_CAL_CONFIG, AUTO_CAL_ENABLE) \
		REG(SCRATCH15, EMC, AUTO_CAL_CONFIG, AUTO_CAL_OVERRIDE) \
		REG(SCRATCH15, EMC, AUTO_CAL_CONFIG, AUTO_CAL_PD_OFFSET) \
		REG(SCRATCH15, EMC, AUTO_CAL_CONFIG, AUTO_CAL_PU_OFFSET) \
		REG(SCRATCH15, EMC, AUTO_CAL_CONFIG, AUTO_CAL_STEP) \
		REG(SCRATCH15, EMC, FBIO_CFG1, CFG_DEN_EARLY) \
		REG(SCRATCH15, EMC, FBIO_CFG5, CTT_TERMINATION) \
		REG(SCRATCH15, EMC, FBIO_CFG5, DIFFERENTIAL_DQS) \
		REG(SCRATCH15, EMC, FBIO_CFG5, DQS_PULLD) \
		REG(SCRATCH15, EMC, FBIO_CFG5, DRAM_TYPE) \
		REG(SCRATCH15, EMC, FBIO_CFG5, DRAM_WIDTH) \
		REG(SCRATCH15, EMC, FBIO_CFG6, CFG_QUSE_LATE) \
		/**/ \
		REG(SCRATCH16, EMC, AUTO_CAL_INTERVAL, AUTO_CAL_INTERVAL) \
		REG(SCRATCH16, EMC, CFG_2, CLKCHANGE_PD_ENABLE) \
		REG(SCRATCH16, EMC, CFG_2, CLKCHANGE_REQ_ENABLE) \
		REG(SCRATCH16, EMC, CFG_2, PIN_CONFIG) \
		/**/ \
		REG(SCRATCH17, EMC, ADR_CFG, EMEM_BANKWIDTH) \
		REG(SCRATCH17, EMC, ADR_CFG, EMEM_COLWIDTH) \
		REG(SCRATCH17, EMC, ADR_CFG, EMEM_DEVSIZE) \
		REG(SCRATCH17, EMC, ADR_CFG, EMEM_NUMDEV) \
		REG(SCRATCH17, EMC, CFG, AUTO_PRE_RD) \
		REG(SCRATCH17, EMC, CFG, AUTO_PRE_WR) \
		REG(SCRATCH17, EMC, CFG, CLEAR_AP_PREV_SPREQ) \
		REG(SCRATCH17, EMC, CFG, DRAM_ACPD) \
		REG(SCRATCH17, EMC, CFG, DRAM_CLKSTOP_PDSR_ONLY) \
		REG(SCRATCH17, EMC, CFG, DRAM_CLKSTOP) \
		REG(SCRATCH17, EMC, CFG, PRE_IDLE_CYCLES) \
		REG(SCRATCH17, EMC, CFG, PRE_IDLE_EN) \
		REG(SCRATCH17, EMC, CFG_DIG_DLL, CFG_DLL_LOCK_LIMIT) \
		REG(SCRATCH17, EMC, CFG_DIG_DLL, CFG_DLL_MODE) \
		/**/ \
		REG(SCRATCH18, EMC, ADR_CFG_1, EMEM1_BANKWIDTH) \
		REG(SCRATCH18, EMC, ADR_CFG_1, EMEM1_COLWIDTH) \
		REG(SCRATCH18, EMC, ADR_CFG_1, EMEM1_DEVSIZE) \
		REG(SCRATCH18, EMC, CTT_TERM_CTRL, TERM_DRVUP) \
		/**/ \
		REG(SCRATCH19, EMC, CFG_DIG_DLL, CFG_DLI_TRIMMER_EN) \
		REG(SCRATCH19, EMC, CFG_DIG_DLL, CFG_DLL_EN) \
		REG(SCRATCH19, EMC, CFG_DIG_DLL, CFG_DLL_LOWSPEED) \
		REG(SCRATCH19, EMC, CFG_DIG_DLL, CFG_DLL_OVERRIDE_EN) \
		REG(SCRATCH19, EMC, CFG_DIG_DLL, CFG_DLL_UDSET) \
		REG(SCRATCH19, EMC, CFG_DIG_DLL, CFG_PERBYTE_TRIMMER_OVERRIDE) \
		REG(SCRATCH19, EMC, CFG_DIG_DLL, CFG_USE_SINGLE_DLL) \
		/**/ \
		REG(SCRATCH20, EMC, CFG_CLKTRIM_1, CFG_DQS0_CLKTRIM) \
		REG(SCRATCH20, EMC, CFG_CLKTRIM_1, CFG_DQS1_CLKTRIM) \
		REG(SCRATCH20, EMC, CFG_CLKTRIM_1, CFG_DQS2_CLKTRIM) \
		REG(SCRATCH20, EMC, CFG_CLKTRIM_1, CFG_DQS3_CLKTRIM) \
		REG(SCRATCH20, EMC, CFG_CLKTRIM_1, CFG_MCLK_CLKTRIM) \
		/**/ \
		REG(SCRATCH21, EMC, CFG_CLKTRIM_2, CFG_CMD_CLKTRIM) \
		REG(SCRATCH21, EMC, CFG_CLKTRIM_2, CFG_DQ0_CLKTRIM) \
		REG(SCRATCH21, EMC, CFG_CLKTRIM_2, CFG_DQ1_CLKTRIM) \
		REG(SCRATCH21, EMC, CFG_CLKTRIM_2, CFG_DQ2_CLKTRIM) \
		REG(SCRATCH21, EMC, CFG_CLKTRIM_2, CFG_DQ3_CLKTRIM) \
		/**/ \
		REG(SCRATCH22, EMC, CFG_DIG_DLL, CFG_DLL_OVERRIDE_VAL) \
		REG(SCRATCH22, EMC, DLL_XFORM_DQS, XFORM_DQS_MULT) \
		REG(SCRATCH22, EMC, DLL_XFORM_DQS, XFORM_DQS_OFFS) \
		/**/ \
		REG(SCRATCH23, EMC, DLL_XFORM_QUSE, XFORM_QUSE_MULT) \
		REG(SCRATCH23, EMC, DLL_XFORM_QUSE, XFORM_QUSE_OFFS) \
		REG(SCRATCH23, EMC, ODT_READ, DISABLE_ODT_DURING_READ) \
		REG(SCRATCH23, EMC, ODT_READ, ODT_B4_READ) \
		REG(SCRATCH23, EMC, ODT_READ, ODT_RD_DELAY) \
		REG(SCRATCH23, EMC, ODT_WRITE, ENABLE_ODT_DURING_WRITE) \
		REG(SCRATCH23, EMC, ODT_WRITE, ODT_B4_WRITE) \
		REG(SCRATCH23, EMC, ODT_WRITE, ODT_WR_DELAY) \
		/**/ \
		REG(SCRATCH35, EMC, ZCAL_REF_CNT, ZCAL_REF_INTERVAL) \
		REG(SCRATCH35, EMC, ZCAL_WAIT_CNT, ZCAL_WAIT_CNT) \
		REG(SCRATCH36, EMC, CTT_TERM_CTRL, TERM_DRVDN) \
		REG(SCRATCH36, EMC, CTT_TERM_CTRL, TERM_OFFSET) \
		REG(SCRATCH36, EMC, CTT_TERM_CTRL, TERM_OVERRIDE) \
		REG(SCRATCH36, EMC, CTT_TERM_CTRL, TERM_SLOPE) \
		REG(SCRATCH36, EMC, ZCAL_MRW_CMD, ZQ_MRW_MA) \
		REG(SCRATCH36, EMC, ZCAL_MRW_CMD, ZQ_MRW_OP) \
		/* MC Group */ \
		REG(SCRATCH17, MC, LOWLATENCY_CONFIG, MPCORER_LL_CTRL) \
		REG(SCRATCH17, MC, LOWLATENCY_CONFIG, MPCORER_LL_SEND_BOTH) \
		/**/ \
		REG(SCRATCH19, MC, EMEM_CFG, EMEM_SIZE_KB) \
		/**/ \
		REG(SCRATCH22, MC, LOWLATENCY_CONFIG, LL_DRAM_INTERLEAVE) \
		/* APB_MISC Group */ \
		REG(SCRATCH3, APB_MISC, GP_XM2CFGCPADCTRL2, CFG2TMC_XM2CFGC_VREF_DQ) \
		REG(SCRATCH3, APB_MISC, GP_XM2CFGCPADCTRL, CFG2TMC_XM2CFGC_SCHMT_EN) \
		REG(SCRATCH3, APB_MISC, GP_XM2CLKCFGPADCTRL, CFG2TMC_XM2CLKCFG_PREEMP_EN) \
		/**/ \
		REG(SCRATCH11, APB_MISC, GP_XM2CFGCPADCTRL2, CFG2TMC_XM2CFGC_VREF_DQ_EN) \
		REG(SCRATCH11, APB_MISC, GP_XM2CFGCPADCTRL2, CFG2TMC_XM2CFGC_VREF_DQS) \
		/**/ \
		REG(SCRATCH18, APB_MISC, GP_XM2COMPPADCTRL, CFG2TMC_XM2COMP_VREF_SEL) \
		REG(SCRATCH18, APB_MISC, GP_XM2VTTGENPADCTRL, CFG2TMC_XM2VTTGEN_CAL_DRVDN) \
		REG(SCRATCH18, APB_MISC, GP_XM2VTTGENPADCTRL, CFG2TMC_XM2VTTGEN_CAL_DRVUP) \
		REG(SCRATCH18, APB_MISC, GP_XM2VTTGENPADCTRL, CFG2TMC_XM2VTTGEN_SHORT_PWRGND) \
		REG(SCRATCH18, APB_MISC, GP_XM2VTTGENPADCTRL, CFG2TMC_XM2VTTGEN_SHORT) \
		REG(SCRATCH18, APB_MISC, GP_XM2VTTGENPADCTRL, CFG2TMC_XM2VTTGEN_VAUXP_LEVEL) \
		REG(SCRATCH18, APB_MISC, GP_XM2VTTGENPADCTRL, CFG2TMC_XM2VTTGEN_VCLAMP_LEVEL) \
		/**/ \
		REG(SCRATCH20, APB_MISC, GP_XM2CFGCPADCTRL2, CFG2TMC_XM2CFGC_CTT_HIZ_EN) \
		REG(SCRATCH20, APB_MISC, GP_XM2CFGCPADCTRL2, CFG2TMC_XM2CFGC_VREF_DQS_EN) \
		/**/ \
		REG(SCRATCH21, APB_MISC, GP_XM2CFGCPADCTRL2, CFG2TMC_XM2CFGC_PREEMP_EN) \
		REG(SCRATCH21, APB_MISC, GP_XM2CFGCPADCTRL2, CFG2TMC_XM2CFGC_RX_FT_REC_EN) \
		/**/ \
		REG(SCRATCH40, APB_MISC, GP_XM2CFGDPADCTRL2, CFG2TMC_XM2CFGD0_DLYIN_TRM) \
		REG(SCRATCH40, APB_MISC, GP_XM2CFGDPADCTRL2, CFG2TMC_XM2CFGD1_DLYIN_TRM) \
		REG(SCRATCH40, APB_MISC, GP_XM2CFGDPADCTRL2, CFG2TMC_XM2CFGD2_DLYIN_TRM) \
		REG(SCRATCH40, APB_MISC, GP_XM2CFGDPADCTRL2, CFG2TMC_XM2CFGD3_DLYIN_TRM) \
		REG(SCRATCH40, APB_MISC, GP_XM2CFGDPADCTRL2, CFG2TMC_XM2CFGD_CTT_HIZ_EN) \
		REG(SCRATCH40, APB_MISC, GP_XM2CFGDPADCTRL2, CFG2TMC_XM2CFGD_PREEMP_EN) \
		REG(SCRATCH40, APB_MISC, GP_XM2CFGDPADCTRL2, CFG2TMC_XM2CFGD_RX_FT_REC_EN) \
		/* End-of-List */

#define GPIO_PORT(x)				((x) - 'a')
#define GPIO_PORTS_PER_INSTANCE		(4)
#define GPIO_BITS_PER_PORT			(8)

#define aa							('z'+1) // GPIO port AA
#define ab							('z'+2) // GPIO port AB

#define AVP_CONTEXT_SAVE_AREA_SIZE 	4096

//------------------------------------------------------------------------------
// Wakeup source table macros
//------------------------------------------------------------------------------

/** WAKEUP_INTERNAL(m,i,x) - Internal wakeup module interrupt sources

	@param m Module id
	@param i Module instance
	@param x Module interrupt index
 */
#define WAKEUP_INTERNAL(m,i,x)	{ NVRM_MODULE_ID((m), (i)), (x) }

/** WAKEUP_EXTERNAL(p, b) - External wakeup module interrupt sources

	@param p GPIO port (e.g., 'a', 'b', etc.)
	@param b GPIO port bit
 */
#define WAKEUP_EXTERNAL(p,b)\
	{NVRM_MODULE_ID(NvRmPrivModuleID_Gpio,GPIO_PORT(p)/GPIO_PORTS_PER_INSTANCE),\
		(((GPIO_PORT(p) % GPIO_PORTS_PER_INSTANCE)*GPIO_BITS_PER_PORT) + (b)) }
typedef struct
{
	NvU32 *pBase;
	NvU32 *pContext;
}  power_module_context;

struct power_context
{
	NvU32 context_size_words;
	NvU32 *first_context_location;
	power_module_context interrupt;
	power_module_context misc;
	power_module_context clock_reset;
	power_module_context apb_dma;
	power_module_context apb_dma_chan;
	power_module_context gpio;
	power_module_context vde;
	power_module_context mc;
};

struct wakeup_source
{
	NvU32 Module;
	NvU32 Index;
};

typedef enum
{
	PowerModuleContext_Init,
	PowerModuleContext_Save,
	PowerModuleContext_Restore,
	PowerModuleContext_DisableInterrupt,
	PowerModuleContext_SaveLP1,
	PowerModuleContext_RestoreLP1,
	PowerModuleContext_Force32 = 0x7fffffff
} PowerModuleContext;

typedef enum
{
	PowerPllM = 0x1,  // Memory
	PowerPllC = 0x2,  // CPU
	PowerPllP = 0x4,  // Peripherals
	PowerPllA = 0x8,  // Audio
	PowerPllX = 0x10, // CPU Complex
	PowerPll_Force32 = 0x7fffffff
} PowerPll;

typedef enum
{
	POWER_STATE_LP2,
	POWER_STATE_LP1,
	POWER_STATE_LP0,
} PowerState;

typedef NvU16 NvIrqNumber;
