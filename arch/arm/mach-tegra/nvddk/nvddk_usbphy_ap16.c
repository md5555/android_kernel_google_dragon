/*
 * Copyright (c) 2009 NVIDIA Corporation.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of the NVIDIA Corporation nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */


/**
 * @file
 * @brief <b>NVIDIA Driver Development Kit:
 *           NvDDK USB PHY functions</b>
 *
 * @b Description: Defines USB PHY private functions
 *
 */

#include "nvrm_module.h"
#include "nvrm_drf.h"
#include "ap15/arclk_rst.h"
#include "ap15/arfuse.h"
#include "ap16/arapb_misc.h"
#include "ap20/arusb.h"
#include "nvrm_hardware_access.h"
#include "nvddk_usbphy_priv.h"


#define USB_IF_REG_RD(reg)\
    NV_READ32(pUsbPhy->UsbVirAdr + ((USB2_IF_USB_##reg##_0)/4))

#define USB_IF_REG_WR(reg, data)\
    NV_WRITE32(pUsbPhy->UsbVirAdr + ((USB2_IF_USB_##reg##_0)/4), (data))

/* Defines for MISC register access */
#define USB_MISC_REGR(pUsbPhy, offset) \
    NV_READ32(pUsbPhy->MiscVirAdr + offset/4)

#define USB_MISC_REGW(pUsbPhy, offset, value) \
    NV_WRITE32(pUsbPhy->MiscVirAdr + offset/4, value)


/**
 * Structure defining the fields for USB UTMI clocks delay Parameters.
 */
typedef struct UsbPllDelayParamsRec
{
    // Pll-U Enable Delay Count
    NvU8 EnableDelayCount;
    //PLL-U Stable count
    NvU8 StableCount;
    //Pll-U Active delay count
    NvU8 ActiveDelayCount;
    //PLL-U Xtal frequency count
    NvU8 XtalFreqCount;
} UsbPllDelayParams;

/*
 * Set of oscillator frequencies supported
 */
typedef enum
{
    NvRmClocksOscFreq_13_MHz = 0x0,
    NvRmClocksOscFreq_19_2_MHz,
    NvRmClocksOscFreq_12_MHz,
    NvRmClocksOscFreq_26_MHz,
    NvRmClocksOscFreq_Num,       // dummy to get number of frequencies
    NvRmClocksOscFreq_Force32 = 0x7fffffff
} NvRmClocksOscFreq;

// Possible Oscillator Frequecies in KHz for mapping the index
static NvRmFreqKHz s_RmOscFrequecy [NvRmClocksOscFreq_Num] =
{
    13000, // 13 Mega Hertz
    19200,// 19.2 Mega Hertz
    12000,// 12 Mega Hertz
    26000 // 26 Mega Hertz
};

///////////////////////////////////////////////////////////////////////////////
// USB PLL CONFIGURATION & PARAMETERS: refer to the arapb_misc_utmip.spec file.
///////////////////////////////////////////////////////////////////////////////
// PLL CONFIGURATION & PARAMETERS for different clock generators:
//-----------------------------------------------------------------------------
// Reference frequency     13.0MHz         19.2MHz         12.0MHz     26.0MHz
// ----------------------------------------------------------------------------
// PLLU_ENABLE_DLY_COUNT   02 (02h)        03 (03h)        02 (02h)    04 (04h)
// PLLU_STABLE_COUNT       51 (33h)        75 (4Bh)        47 (2Fh)   102 (66h)
// PLL_ACTIVE_DLY_COUNT    05 (05h)        06 (06h)        04 (04h)    09 (09h)
// XTAL_FREQ_COUNT        127 (7Fh)       187 (BBh)       118 (76h)   254 (FEh)
///////////////////////////////////////////////////////////////////////////////
static const UsbPllDelayParams s_UsbPllDelayParams[NvRmClocksOscFreq_Num] =
{
    //ENABLE_DLY,  STABLE_CNT,  ACTIVE_DLY,  XTAL_FREQ_CNT
    {0x02,         0x33,        0x05,        0x7F}, // For NvRmClocksOscFreq_13_MHz,
    {0x03,         0x4B,        0x06,        0xBB}, // For NvRmClocksOscFreq_19_2_MHz
    {0x02,         0x2F,        0x04,        0x76}, // For NvRmClocksOscFreq_12_MHz
    {0x04,         0x66,        0x09,        0xFE}  // For NvRmClocksOscFreq_26_MHz
};

///////////////////////////////////////////////////////////////////////////////
// USB Debounce values IdDig, Avalid, Bvalid, VbusValid, VbusWakeUp, and SessEnd.
// Each of these signals have their own debouncer and for each of those one out
// of 2 debouncing times can be chosen (BIAS_DEBOUNCE_A or BIAS_DEBOUNCE_B.)
//
// The values of DEBOUNCE_A and DEBOUNCE_B are calculated as follows:
// 0xffff -> No debouncing at all
// <n> ms = <n> *1000 / (1/19.2MHz) / 4
// So to program a 1 ms debounce for BIAS_DEBOUNCE_A, we have:
// BIAS_DEBOUNCE_A[15:0] = 1000 * 19.2 / 4  = 4800 = 0x12c0
// We need to use only DebounceA, We dont need the DebounceB
// values, so we can keep those to default.
///////////////////////////////////////////////////////////////////////////////
static const NvU32 s_UsbBiasDebounceATime[NvRmClocksOscFreq_Num] =
{
    /* Ten milli second delay for BIAS_DEBOUNCE_A */
    0x7EF4,  // For NvRmClocksOscFreq_13_MHz,
    0xBB80,  // For NvRmClocksOscFreq_19_2_MHz
    0x7530,  // For NvRmClocksOscFreq_12_MHz
    0xFDE8   // For NvRmClocksOscFreq_26_MHz
};

///////////////////////////////////////////////////////////////////////////////
// The following arapb_misc_utmip.spec fields need to be programmed to ensure
// correct operation of the UTMIP block:
// Production settings :
//        'HS_SYNC_START_DLY' : 9,
//        'IDLE_WAIT'         : 17,
//        'ELASTIC_LIMIT'     : 16,
// All other fields can use the default reset values.
// Setting the fields above, together with default values of the other fields,
// results in programming the registers below as follows:
//         UTMIP_HSRX_CFG0 = 0x9168c000
//         UTMIP_HSRX_CFG1 = 0x13
///////////////////////////////////////////////////////////////////////////////
//UTMIP Idle Wait Delay
static const NvU8 s_UtmipIdleWaitDelay    = 17;
//UTMIP Elastic limit
static const NvU8 s_UtmipElasticLimit     = 16;
//UTMIP High Speed Sync Start Delay
static const NvU8 s_UtmipHsSyncStartDelay = 9;
//UTMIP Tranciver setup Value, this value must be read from the fuses
static NvU32 s_XcvrSetupValue = 0; 


static NvError
Ap16UsbPhyUtmiConfigure(
    NvDdkUsbPhy *pUsbPhy)
{
    NvU32 RegVal = 0;
    NvRmFreqKHz OscFreqKz = 0;
    NvU32 FreqIndex;

    // Get the Oscillator Frequency
    OscFreqKz = NvRmPowerGetPrimaryFrequency(pUsbPhy->hRmDevice);

    // Get the Oscillator Frequency Index
    for (FreqIndex = 0; FreqIndex < NvRmClocksOscFreq_Num; FreqIndex++)
    {
        if (OscFreqKz == s_RmOscFrequecy[FreqIndex])
        {
            // Bail Out if frequecy matches with the supported frequency
            break;
        }
    }
    // If Index is equal to the maximum supported frequency count
    // There is a mismatch of the frequecy, so returning since the
    // frequency is not supported.
    if (FreqIndex >= NvRmClocksOscFreq_Num)
    {
        return NvError_NotSupported;
    }

    RegVal = USB_MISC_REGR(pUsbPhy, APB_MISC_UTMIP_TX_CFG0_0);
    RegVal = NV_FLD_SET_DRF_NUM( 
                APB_MISC, UTMIP_TX_CFG0, UTMIP_FS_PREAMBLE_J, 0x1, RegVal);
    USB_MISC_REGW(pUsbPhy, APB_MISC_UTMIP_TX_CFG0_0, RegVal);

    // Configure the UTMIP_IDLE_WAIT and UTMIP_ELASTIC_LIMIT
    // Setting these fields, together with default values of the other
    // fields, results in programming the registers below as follows:
    //         UTMIP_HSRX_CFG0 = 0x9168c000
    //         UTMIP_HSRX_CFG1 = 0x13
    RegVal = USB_MISC_REGR(pUsbPhy, APB_MISC_UTMIP_HSRX_CFG0_0);
    RegVal = NV_FLD_SET_DRF_NUM(APB_MISC, UTMIP_HSRX_CFG0,
                UTMIP_IDLE_WAIT, s_UtmipIdleWaitDelay, RegVal);
    RegVal = NV_FLD_SET_DRF_NUM(APB_MISC, UTMIP_HSRX_CFG0, 
                UTMIP_ELASTIC_LIMIT, s_UtmipElasticLimit, RegVal);
    USB_MISC_REGW(pUsbPhy, APB_MISC_UTMIP_HSRX_CFG0_0, RegVal);

    // Configure the UTMIP_HS_SYNC_START_DLY
    RegVal = USB_MISC_REGR(pUsbPhy, APB_MISC_UTMIP_HSRX_CFG1_0);
    RegVal = NV_FLD_SET_DRF_NUM(APB_MISC, UTMIP_HSRX_CFG1,
                UTMIP_HS_SYNC_START_DLY, s_UtmipHsSyncStartDelay, RegVal);
    USB_MISC_REGW(pUsbPhy, APB_MISC_UTMIP_HSRX_CFG1_0, RegVal);

    // Program 1ms Debounce time for VBUS to become valid.
    RegVal = USB_MISC_REGR(pUsbPhy, APB_MISC_UTMIP_DEBOUNCE_CFG0_0);
    RegVal = NV_FLD_SET_DRF_NUM(APB_MISC, UTMIP_DEBOUNCE_CFG0, 
                UTMIP_BIAS_DEBOUNCE_A, s_UsbBiasDebounceATime[FreqIndex], RegVal);
    USB_MISC_REGW(pUsbPhy, APB_MISC_UTMIP_DEBOUNCE_CFG0_0, RegVal);

    // PLL Delay CONFIGURATION settings
    // The following parameters control the bring up of the plls:
    RegVal = USB_MISC_REGR(pUsbPhy, APB_MISC_UTMIP_MISC_CFG0_0);
    RegVal = NV_FLD_SET_DRF_NUM(APB_MISC, UTMIP_MISC_CFG0, 
                UTMIP_SUSPEND_EXIT_ON_EDGE, 0, RegVal);
    USB_MISC_REGW(pUsbPhy, APB_MISC_UTMIP_MISC_CFG0_0, RegVal);

    RegVal = USB_MISC_REGR(pUsbPhy, APB_MISC_UTMIP_MISC_CFG1_0);
    RegVal = NV_FLD_SET_DRF_NUM(APB_MISC, UTMIP_MISC_CFG1, UTMIP_PLLU_STABLE_COUNT,
                s_UsbPllDelayParams[FreqIndex].StableCount, RegVal);
    RegVal = NV_FLD_SET_DRF_NUM(APB_MISC, UTMIP_MISC_CFG1, UTMIP_PLL_ACTIVE_DLY_COUNT,
                s_UsbPllDelayParams[FreqIndex].ActiveDelayCount, RegVal);
    USB_MISC_REGW(pUsbPhy, APB_MISC_UTMIP_MISC_CFG1_0, RegVal);

    // Set PLL enable delay count and Crystal frequency count
    RegVal = USB_MISC_REGR(pUsbPhy, APB_MISC_UTMIP_PLL_CFG1_0);
    RegVal = NV_FLD_SET_DRF_NUM(APB_MISC, UTMIP_PLL_CFG1, UTMIP_PLLU_ENABLE_DLY_COUNT,
                s_UsbPllDelayParams[FreqIndex].EnableDelayCount, RegVal);
    RegVal = NV_FLD_SET_DRF_NUM(APB_MISC, UTMIP_PLL_CFG1, UTMIP_XTAL_FREQ_COUNT,
                s_UsbPllDelayParams[FreqIndex].XtalFreqCount, RegVal);
    USB_MISC_REGW(pUsbPhy, APB_MISC_UTMIP_PLL_CFG1_0, RegVal);

    return NvSuccess;
}

static void
Ap16UsbPhyUtmiPowerControl(
    NvDdkUsbPhy *pUsbPhy, 
    NvBool Enable)
{
    NvU32 RegVal = 0;

    if (Enable)
    {
        // USB Power Up sequence
        // Power Up OTG and Bias circuitry
        RegVal = USB_MISC_REGR(pUsbPhy, APB_MISC_UTMIP_BIAS_CFG0_0);
        RegVal = NV_FLD_SET_DRF_NUM(
                    APB_MISC, UTMIP_BIAS_CFG0, UTMIP_OTGPD, 0, RegVal);
        RegVal = NV_FLD_SET_DRF_NUM(
                    APB_MISC, UTMIP_BIAS_CFG0, UTMIP_BIASPD, 0, RegVal);
        USB_MISC_REGW(pUsbPhy, APB_MISC_UTMIP_BIAS_CFG0_0, RegVal);

        // Turn on power in the tranciver
        RegVal = USB_MISC_REGR(pUsbPhy, APB_MISC_UTMIP_XCVR_CFG0_0);
        RegVal = NV_FLD_SET_DRF_NUM(APB_MISC,
                    UTMIP_XCVR_CFG0, UTMIP_FORCE_PDZI_POWERDOWN, 0, RegVal);
        RegVal = NV_FLD_SET_DRF_NUM(APB_MISC,
                    UTMIP_XCVR_CFG0, UTMIP_FORCE_PD2_POWERDOWN, 0, RegVal);
        RegVal = NV_FLD_SET_DRF_NUM(APB_MISC,
                    UTMIP_XCVR_CFG0, UTMIP_FORCE_PD_POWERDOWN, 0, RegVal);
        RegVal = NV_FLD_SET_DRF_NUM(APB_MISC,
                    UTMIP_XCVR_CFG0, UTMIP_XCVR_SETUP, s_XcvrSetupValue, RegVal);
        USB_MISC_REGW(pUsbPhy, APB_MISC_UTMIP_XCVR_CFG0_0, RegVal);

        // Enable Batery charge enabling bit, set to '0' for enable
        RegVal = USB_MISC_REGR(pUsbPhy, APB_MISC_UTMIP_BAT_CHRG_CFG0_0);
        RegVal = NV_FLD_SET_DRF_NUM(
                    APB_MISC, UTMIP_BAT_CHRG_CFG0, UTMIP_PD_CHRG, 0, RegVal);
        USB_MISC_REGW(pUsbPhy, APB_MISC_UTMIP_BAT_CHRG_CFG0_0, RegVal);
    }
    else
    {
        // USB Power down sequence
        // Power down OTG and Bias circuitry
        RegVal = USB_MISC_REGR(pUsbPhy, APB_MISC_UTMIP_BIAS_CFG0_0);
        // Check if Internal Phy is going to wake up the usb 
        // controller upon cable insertion.
        if (!pUsbPhy->pProperty->UseInternalPhyWakeup)
        {
            /// If not internal Phy then Use PMU interrupt for VBUS detection.
            /// Disable the OTG bias circuitry.
            RegVal = NV_FLD_SET_DRF_NUM(
                        APB_MISC, UTMIP_BIAS_CFG0, UTMIP_OTGPD, 1, RegVal);
        }
        RegVal = NV_FLD_SET_DRF_NUM(
                    APB_MISC, UTMIP_BIAS_CFG0, UTMIP_BIASPD, 1, RegVal);
        USB_MISC_REGW(pUsbPhy, APB_MISC_UTMIP_BIAS_CFG0_0, RegVal);

        // Disable Batery charge enabling bit set to '1' for disable
        RegVal = USB_MISC_REGR(pUsbPhy, APB_MISC_UTMIP_BAT_CHRG_CFG0_0);
        RegVal = NV_FLD_SET_DRF_NUM(
                    APB_MISC, UTMIP_BAT_CHRG_CFG0, UTMIP_PD_CHRG, 1, RegVal);
        USB_MISC_REGW(pUsbPhy, APB_MISC_UTMIP_BAT_CHRG_CFG0_0, RegVal);

        // Turn off power in the tranciver
        RegVal = USB_MISC_REGR(pUsbPhy, APB_MISC_UTMIP_XCVR_CFG0_0);
        RegVal = NV_FLD_SET_DRF_NUM(APB_MISC, 
                    UTMIP_XCVR_CFG0, UTMIP_FORCE_PDZI_POWERDOWN, 1, RegVal);
        RegVal = NV_FLD_SET_DRF_NUM(APB_MISC, 
                    UTMIP_XCVR_CFG0, UTMIP_FORCE_PD2_POWERDOWN, 1, RegVal);
        RegVal = NV_FLD_SET_DRF_NUM(APB_MISC, 
                    UTMIP_XCVR_CFG0, UTMIP_FORCE_PD_POWERDOWN, 1, RegVal);
        USB_MISC_REGW(pUsbPhy, APB_MISC_UTMIP_XCVR_CFG0_0, RegVal);
    }
}


static void 
Ap16UsbPhyUlpiLinkModeConfigure(
    NvDdkUsbPhy *pUsbPhy)
{
    NvU32 RegVal = 0;

    // Enable clock to the USB2 and Bring out of Reset 
    // Then Setup the Link Mode Trimmers.
    // Bypass the Pin Mux on the ULPI outputs and set trimmer values for inputs
    RegVal = USB_MISC_REGR(pUsbPhy, APB_MISC_UTMIP_SPARE_CFG0_0);
    RegVal = RegVal & ( ~( (0x1 << 13) | (0xf << 4) | (0xf << 28) ) );
    //bit 13        : data output pinmux bypass enable: set to 1
    //bit 4         : data input trimmer load enable (toggle)
    //bit [7:5]     : data input trimmer value: set to 3
    //bit 28        : data input trimmer2 load enable (toggle)
    //bit [31:29]   : data input trimmer2 value: set to 3
    RegVal = RegVal | ( (0x1 << 13) | (0x0 << 5) | (0x0 << 4) | 
                        (0x0 << 29) | (0x0 << 28) );
    USB_MISC_REGW(pUsbPhy, APB_MISC_UTMIP_SPARE_CFG0_0, RegVal);

    // wait 10 us
    NvOsWaitUS(10);
    // toggle bits 4 and 28 to latch the trimmer values
    RegVal = RegVal | ( (0x1 << 4) | (0x1 << 28) );
    USB_MISC_REGW(pUsbPhy, APB_MISC_UTMIP_SPARE_CFG0_0, RegVal);
    // wait 10 us
    NvOsWaitUS(10);
}


static void 
Ap16UsbPhyUlpiNullModeConfigure(
    NvDdkUsbPhy *pUsbPhy)
{
    NvU32 RegVal = 0;

    // Enable clock to the USB2 and Bring out of Reset then
    // Clear the trimmers and Setup the ULPI NULL mode PINMUX
    RegVal = USB_MISC_REGR(pUsbPhy, APB_MISC_UTMIP_SPARE_CFG0_0);
    RegVal = RegVal & (0xf);
    RegVal = RegVal | ( (0x0 << 29) | (0x0 << 28) | (0x0 << 27) | (0 << 22) |
                        (0x0 << 21) | (0x0 << 16) | (0x0 << 15) | (0x1 << 14) | 
                        (0x1 << 13) | (0x0 << 12) | (0x1 << 11) | (0x1 << 10) | 
                        (0x1 << 9) | (0x0 << 5) | (0x0 << 4) );
    USB_MISC_REGW(pUsbPhy, APB_MISC_UTMIP_SPARE_CFG0_0, RegVal);

    // wait 10 us
    NvOsWaitUS(10);

    // Enable null phy (bit 0)
    // ULPIS2S_SPARE[3] = ULPIS2S_INT_CLOCK
    // ULPIS2S_SPARE[0] = VBUS active
    RegVal = USB_IF_REG_RD(ULPIS2S_CTRL);
    RegVal = NV_FLD_SET_DRF_DEF(
                    USB2_IF_USB, ULPIS2S_CTRL, ULPIS2S_ENA, ENABLE, RegVal);
    RegVal = NV_FLD_SET_DRF_NUM(
                    USB2_IF_USB, ULPIS2S_CTRL, ULPIS2S_SPARE, 0x9, RegVal);
    USB_IF_REG_WR(ULPIS2S_CTRL, RegVal);

    // wait 10 us
    NvOsWaitUS(10);

    // Set ULPI_CLK_ENA (ULPIS2S_SPARE[2]) to 1 to enable ULPI null clocks
    // ULPIS2S_SPARE[3] = ULPIS2S_INT_CLOCK
    // ULPIS2S_SPARE[2] = 1
    // ULPIS2S_SPARE[0] = VBUS active
    RegVal = USB_IF_REG_RD(ULPIS2S_CTRL);
    RegVal = NV_FLD_SET_DRF_NUM(
                    USB2_IF_USB, ULPIS2S_CTRL, ULPIS2S_SPARE, 0xD, RegVal);
    USB_IF_REG_WR(ULPIS2S_CTRL, RegVal);

    // wait 10 us
    NvOsWaitUS(10);

    // Set trimmers required for the NULL ULPI
    // Configure 60M clock for USB2 - ULPI controller
    // Set up to use PLLU at 60 MHz and keep USB PHY PLL in reset
    // bit 27        : bypass 60 MHz Div5 for PLLU - set to 1
    // bit 9         : nullphy_pll_source - use USB_PHY_PLL output (set to 0) :
    //                    Workaround: set to 1 to use PLLU Output at 12 MHz
    RegVal = USB_MISC_REGR(pUsbPhy, APB_MISC_UTMIP_SPARE_CFG0_0);
    RegVal &= (~( 0x1 << 27) );
    RegVal |= ((0x1 << 9));
    USB_MISC_REGW(pUsbPhy, APB_MISC_UTMIP_SPARE_CFG0_0,
                                        RegVal);

    // Set the trimmers
    // bit 4         : data input trimmer load enable (toggle)
    // bit [7:5]     : data input trimmer value - set to 3
    // bit 28        : data input trimmer2 load enable - (toggle)
    // bit [31:29]   : data input trimmer2 value - set to 3
    RegVal = USB_MISC_REGR(pUsbPhy, APB_MISC_UTMIP_SPARE_CFG0_0);
    RegVal = RegVal | ( (0x7 << 29) | (0x0 << 28) | (0x7 <<  5) | (0x0 <<  4) );
    USB_MISC_REGW(pUsbPhy, APB_MISC_UTMIP_SPARE_CFG0_0, RegVal);

     // wait 10 us
    NvOsWaitUS(10);

    // toggle bits 4 and 28 to latch the trimmer values
    RegVal = RegVal | ( (0x1 << 4) | (0x1 << 28) );
    USB_MISC_REGW(pUsbPhy, APB_MISC_UTMIP_SPARE_CFG0_0, RegVal);

    // wait 10 us
    NvOsWaitUS(10);

    // Set ULPI_CLK_PADOUT_ENA (ULPIS2S_SPARE[1]) to 1 to enable ULPI 
    // null clock going out to ulpi clk pad (gp3_pv[0])
    RegVal = USB_IF_REG_RD(ULPIS2S_CTRL);
    RegVal = NV_FLD_SET_DRF_NUM(
                    USB2_IF_USB, ULPIS2S_CTRL, ULPIS2S_SPARE, 0xF, RegVal);
    USB_IF_REG_WR(ULPIS2S_CTRL, RegVal);
}



static void
Ap16UsbPhyReadXcvrSetupValueFromFuses(
    NvDdkUsbPhy *pUsbPhy)
{
    NvU32 RegVal = 0;

#if NV_USE_FUSE_CLOCK_ENABLE
    // Enable fuse clock
    NvRmPowerModuleClockControl(pUsbPhy->hRmDevice, NvRmModuleID_Fuse, 0, NV_TRUE);
#endif  

    // Enable fuse values to be visible before reading the fuses.
    RegVal = NV_REGR(pUsbPhy->hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
                CLK_RST_CONTROLLER_MISC_CLK_ENB_0);
    RegVal = NV_FLD_SET_DRF_NUM( CLK_RST_CONTROLLER, MISC_CLK_ENB,
                CFG_ALL_VISIBLE, 1, RegVal );
    NV_REGW( pUsbPhy->hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
                CLK_RST_CONTROLLER_MISC_CLK_ENB_0, RegVal );


    // Read the spare register fuses and redundancy fuses for setting up USB
    // UTMIP_XCVR_SETUP value for proper EYE diagram.
    RegVal = NV_REGR( pUsbPhy->hRmDevice, NvRmModuleID_Fuse, 0, FUSE_FUSEDATA21_0);
    s_XcvrSetupValue  = (NV_DRF_VAL(FUSE, FUSEDATA21,
                            FUSEDATA_SPARE_BIT_10__PRI_ALIAS_0, RegVal) |
                          NV_DRF_VAL(FUSE, FUSEDATA21, 
                            FUSEDATA_SPARE_BIT_13__PRI_ALIAS_0, RegVal)) << 0;
    s_XcvrSetupValue |= (NV_DRF_VAL(FUSE, FUSEDATA21, 
                            FUSEDATA_SPARE_BIT_11__PRI_ALIAS_0, RegVal) |
                          NV_DRF_VAL(FUSE, FUSEDATA21, 
                            FUSEDATA_SPARE_BIT_14__PRI_ALIAS_0, RegVal)) << 1;
    s_XcvrSetupValue |= (NV_DRF_VAL(FUSE, FUSEDATA21, 
                            FUSEDATA_SPARE_BIT_12__PRI_ALIAS_0, RegVal) |
                          NV_DRF_VAL(FUSE, FUSEDATA21, 
                            FUSEDATA_SPARE_BIT_15__PRI_ALIAS_0, RegVal)) << 2;
    // Only UTMIP_XCVR_SETUP[3-1] need to be programmed with the fuse vlaue
    // UTMIP_XCVR_SETUP[0] must be equal to 0
    s_XcvrSetupValue = s_XcvrSetupValue << 1;

    // Disable fuse values visibility, we already read the data
    RegVal = NV_REGR( pUsbPhy->hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
                CLK_RST_CONTROLLER_MISC_CLK_ENB_0 );
    RegVal = NV_FLD_SET_DRF_NUM( CLK_RST_CONTROLLER, MISC_CLK_ENB,
                CFG_ALL_VISIBLE, 0, RegVal );
    NV_REGW( pUsbPhy->hRmDevice, NvRmPrivModuleID_ClockAndReset, 0,
            CLK_RST_CONTROLLER_MISC_CLK_ENB_0, RegVal );

#if NV_USE_FUSE_CLOCK_ENABLE
    // Disable fuse clock
    NvRmPowerModuleClockControl(pUsbPhy->hRmDevice, NvRmModuleID_Fuse, 0, NV_FALSE);
#endif
}


static void
Ap16UsbPhyControlClockAndReset(
    NvDdkUsbPhy *pUsbPhy,
    NvBool EnableClock)
{
    NvBool HoldReset = EnableClock ? NV_FALSE : NV_TRUE;
    NvU32 RegVal = 0;
    NvU32 TimeOut = 1000;
    NvU32 PhyClkValid = 0;

    if(!EnableClock && ( pUsbPhy->pProperty->UsbInterfaceType == NvOdmUsbInterfaceType_UlpiExternalPhy))
    {
        // before disabling clock.. put the phy to low power mode..
        // enter low power suspend mode
        RegVal = NV_REGR(pUsbPhy->hRmDevice, NvRmModuleID_Usb2Otg, 1, 
                            USB2_CONTROLLER_1_USB2D_PORTSC1_0);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_PORTSC1, PHCD,
                            ENABLE, RegVal);
        NV_REGW(pUsbPhy->hRmDevice, NvRmModuleID_Usb2Otg, 1, 
                            USB2_CONTROLLER_1_USB2D_PORTSC1_0, RegVal);

        // check for the phy in suspend..
        do
        {
            //Wait for the phy clock to stop or invalid
            RegVal = NV_REGR(pUsbPhy->hRmDevice, NvRmModuleID_Usb2Otg, 1 ,
                                USB2_IF_USB_SUSP_CTRL_0);

            PhyClkValid = NV_DRF_VAL(USB2_IF, USB_SUSP_CTRL, USB_PHY_CLK_VALID,
                                RegVal);

            if (!TimeOut)
            {
                break;
            }
            NvOsWaitUS(1);
            TimeOut--;
        } while (PhyClkValid);
    }

    // Enable/Disable Clock to the controller
    if (pUsbPhy->Caps.CommonClockAndReset)
    {
        RegVal = USB_MISC_REGR(pUsbPhy, APB_MISC_PP_MISC_USB_CLK_RST_CTL_0);
        if (pUsbPhy->Instance == 1)
        {
            RegVal = NV_FLD_SET_DRF_DEF(APB_MISC_PP, MISC_USB_CLK_RST_CTL,
                        MISC_USB2_RST, ENABLE, RegVal);
            RegVal = NV_FLD_SET_DRF_NUM(APB_MISC_PP, MISC_USB_CLK_RST_CTL,
                        MISC_USB2_CE, EnableClock, RegVal);
        }
        else
        {
            RegVal = NV_FLD_SET_DRF_DEF(APB_MISC_PP, MISC_USB_CLK_RST_CTL,
                        MISC_USB_RST, ENABLE, RegVal);
            RegVal = NV_FLD_SET_DRF_NUM(APB_MISC_PP, MISC_USB_CLK_RST_CTL,
                        MISC_USB_CE, EnableClock, RegVal);
        }
        USB_MISC_REGW(pUsbPhy, APB_MISC_PP_MISC_USB_CLK_RST_CTL_0, RegVal);
    }

    // Hold/ Release the RESET to the controller
    if (pUsbPhy->Caps.CommonClockAndReset)
    {
        RegVal = USB_MISC_REGR(pUsbPhy, APB_MISC_PP_MISC_USB_CLK_RST_CTL_0);
        if (pUsbPhy->Instance == 1)
        {
            RegVal = NV_FLD_SET_DRF_NUM(APB_MISC_PP, MISC_USB_CLK_RST_CTL,
                        MISC_USB2_RST, HoldReset, RegVal);
        }
        else
        {
            RegVal = NV_FLD_SET_DRF_NUM(APB_MISC_PP, MISC_USB_CLK_RST_CTL,
                        MISC_USB_RST, HoldReset, RegVal);
        }
        USB_MISC_REGW(pUsbPhy, APB_MISC_PP_MISC_USB_CLK_RST_CTL_0, RegVal);
    }
    else
    {
        NvRmModuleResetWithHold(pUsbPhy->hRmDevice, 
            NVRM_MODULE_ID(NvRmModuleID_Usb2Otg, pUsbPhy->Instance), HoldReset);
    }

    if(EnableClock &&( pUsbPhy->pProperty->UsbInterfaceType == 
                                    NvOdmUsbInterfaceType_UlpiExternalPhy))
    {
        // Wake-up ULPI PHY generate a postive pulse
        //set
        RegVal = NV_REGR(pUsbPhy->hRmDevice, NvRmModuleID_Usb2Otg, 1,
                                USB2_IF_USB_SUSP_CTRL_0);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_IF, USB_SUSP_CTRL, USB_SUSP_CLR, SET,
                                RegVal);
        NV_REGW(pUsbPhy->hRmDevice, NvRmModuleID_Usb2Otg, 1,
                                USB2_IF_USB_SUSP_CTRL_0, RegVal);

        // wait 10 us
        NvOsWaitUS(100);

        // clear
        RegVal = NV_REGR(pUsbPhy->hRmDevice, NvRmModuleID_Usb2Otg, 1,
                                USB2_IF_USB_SUSP_CTRL_0);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_IF, USB_SUSP_CTRL,
                                USB_SUSP_CLR, UNSET, RegVal);
        NV_REGW(pUsbPhy->hRmDevice, NvRmModuleID_Usb2Otg, 1,
                                USB2_IF_USB_SUSP_CTRL_0, RegVal);
    }

}



static NvError 
Ap16UsbPhyIoctlVbusInterrupt(
    NvDdkUsbPhy *pUsbPhy, 
    const void *pInputArgs)
{
    NvDdkUsbPhyIoctl_VBusInterruptInputArgs *pVbusIntr = NULL;
    NvU32 RegVal = 0;

    if (!pInputArgs)
        return NvError_BadParameter;

    pVbusIntr = (NvDdkUsbPhyIoctl_VBusInterruptInputArgs *)pInputArgs;

    if (pUsbPhy->pProperty->UsbInterfaceType == NvOdmUsbInterfaceType_Utmi)
    {
        RegVal = USB_MISC_REGR(pUsbPhy, APB_MISC_PP_USB_PHY_VBUS_SENSORS_0);

        if (pVbusIntr->EnableVBusInterrupt)
        {
            RegVal = NV_FLD_SET_DRF_DEF(APB_MISC_PP, USB_PHY_VBUS_SENSORS, 
                                        A_SESS_VLD_INT_EN, ENABLE, RegVal);
        }
        else
        {
            RegVal = NV_FLD_SET_DRF_DEF(APB_MISC_PP, USB_PHY_VBUS_SENSORS,
                                        A_SESS_VLD_INT_EN, DISABLE, RegVal);
            RegVal = NV_FLD_SET_DRF_DEF(APB_MISC_PP, USB_PHY_VBUS_SENSORS,
                                        A_SESS_VLD_CHG_DET, SET, RegVal);
        }

        USB_MISC_REGW(pUsbPhy, APB_MISC_PP_USB_PHY_VBUS_SENSORS_0, RegVal);
    }
    else
    {
        //TODO:ENable the VBUS interrupt for the ULPI
    }

    return NvSuccess;
}



static NvError 
Ap16UsbPhyIoctlVBusStatus(
    NvDdkUsbPhy *pUsbPhy, 
    void *pOutputArgs)
{
    NvDdkUsbPhyIoctl_VBusStatusOutputArgs *pVBusStatus = NULL;
    NvU32 RegVal = 0;

    if (!pOutputArgs)
        return NvError_BadParameter;

    pVBusStatus = (NvDdkUsbPhyIoctl_VBusStatusOutputArgs *)pOutputArgs;

    if (pUsbPhy->pProperty->UsbInterfaceType == NvOdmUsbInterfaceType_Utmi)
    {
        RegVal = USB_MISC_REGR(pUsbPhy, APB_MISC_PP_USB_PHY_VBUS_SENSORS_0);
        if (NV_DRF_VAL(APB_MISC, PP_USB_PHY_VBUS_SENSORS, A_SESS_VLD_STS, RegVal))
        {
            pVBusStatus->VBusDetected = NV_TRUE;
        }
        else
        {
            pVBusStatus->VBusDetected = NV_FALSE;
        }
    }
    else
    {
        pVBusStatus->VBusDetected = NV_TRUE;
    }

    return NvSuccess;
}



static NvError 
Ap16UsbPhyIoctlIdPinInterrupt(
    NvDdkUsbPhy *pUsbPhy, 
    const void *pInputArgs)
{
    NvDdkUsbPhyIoctl_IdPinInterruptInputArgs *pIdPinIntr = NULL;
    NvU32 RegVal = 0;

    if (!pInputArgs)
        return NvError_BadParameter;

    pIdPinIntr = (NvDdkUsbPhyIoctl_IdPinInterruptInputArgs *)pInputArgs;

    if (pUsbPhy->pProperty->UsbInterfaceType == NvOdmUsbInterfaceType_Utmi)
    {
        RegVal = USB_MISC_REGR(pUsbPhy, APB_MISC_PP_USB_PHY_VBUS_WAKEUP_ID_0);

        if (pIdPinIntr->EnableIdPinInterrupt)
        {
            RegVal = NV_FLD_SET_DRF_DEF(APB_MISC_PP, USB_PHY_VBUS_WAKEUP_ID, 
                                        ID_INT_EN, ENABLE, RegVal);
        }
        else
        {
            RegVal = NV_FLD_SET_DRF_DEF(APB_MISC_PP, USB_PHY_VBUS_WAKEUP_ID,
                                        ID_INT_EN, DISABLE, RegVal);
            RegVal = NV_FLD_SET_DRF_DEF(APB_MISC_PP, USB_PHY_VBUS_WAKEUP_ID,
                                        ID_CHG_DET, SET, RegVal);
        }
        USB_MISC_REGW(pUsbPhy, APB_MISC_PP_USB_PHY_VBUS_WAKEUP_ID_0, RegVal);
    }
    else
    {
        //TODO:ENable the ID pin interrupt for the ULPI
    }

    return NvSuccess;
}


static NvError 
Ap16UsbPhyIoctlIdPinStatus(
    NvDdkUsbPhy *pUsbPhy, 
    void *pOutputArgs)
{
    NvDdkUsbPhyIoctl_IdPinStatusOutputArgs *pIdPinStatus = NULL;
    NvU32 RegVal = 0;

    if (!pOutputArgs)
        return NvError_BadParameter;

    pIdPinStatus = (NvDdkUsbPhyIoctl_IdPinStatusOutputArgs *)pOutputArgs;

    RegVal = USB_MISC_REGR(pUsbPhy, APB_MISC_PP_USB_PHY_VBUS_WAKEUP_ID_0);
    if (!NV_DRF_VAL(APB_MISC_PP, USB_PHY_VBUS_WAKEUP_ID, ID_STS, RegVal))
    {
        pIdPinStatus->IdPinSetToLow = NV_TRUE;
    }
    else
    {
        pIdPinStatus->IdPinSetToLow = NV_FALSE;
    }

    return NvSuccess;
}



static NvError 
Ap16UsbPhyIoctlDedicatedChargerDetection(
    NvDdkUsbPhy *pUsbPhy, 
    const void *pInputArgs)
{
    // These values (in milli second) are taken from the battery charging spec.
    #define TDP_SRC_ON_MS    100
    #define TDPSRC_CON_MS    40
    NvDdkUsbPhyIoctl_DedicatedChargerDetectionInputArgs *pChargerDetection = NULL;
    NvU32 RegVal = 0;

    if (!pInputArgs)
        return NvError_BadParameter;

    pChargerDetection = (NvDdkUsbPhyIoctl_DedicatedChargerDetectionInputArgs *)pInputArgs;

    if (pUsbPhy->pProperty->UsbInterfaceType != NvOdmUsbInterfaceType_Utmi)
    {
        // Charger detection is not there for ULPI
        return NvSuccess;
    }

    if (pChargerDetection->EnableChargerDetection)
    {
        // Enable charger detection logic
        RegVal = USB_MISC_REGR(pUsbPhy, APB_MISC_UTMIP_BAT_CHRG_CFG0_0);
        RegVal = NV_FLD_SET_DRF_NUM(APB_MISC, UTMIP_BAT_CHRG_CFG0,
                                    UTMIP_OP_SRC_EN, 1, RegVal);
        RegVal = NV_FLD_SET_DRF_NUM(APB_MISC, UTMIP_BAT_CHRG_CFG0,
                                   UTMIP_ON_SINK_EN, 1, RegVal);
        USB_MISC_REGW(pUsbPhy, APB_MISC_UTMIP_BAT_CHRG_CFG0_0, RegVal);

        // Source should be on for 100 ms as per USB charging spec
        NvOsSleepMS(TDP_SRC_ON_MS);

        RegVal = USB_MISC_REGR(pUsbPhy, APB_MISC_PP_USB_PHY_VBUS_WAKEUP_ID_0);
        if (pChargerDetection->EnableChargerInterrupt)
        {
            RegVal = NV_FLD_SET_DRF_DEF(APB_MISC, PP_USB_PHY_VBUS_WAKEUP_ID,
                                       VDAT_DET_INT_EN, ENABLE, RegVal);
        }
        else
        {
            // If charger is not connected disable the interrupt
            RegVal = NV_FLD_SET_DRF_DEF(APB_MISC, PP_USB_PHY_VBUS_WAKEUP_ID,
                                       VDAT_DET_INT_EN, DISABLE, RegVal);
            RegVal = NV_FLD_SET_DRF_DEF(APB_MISC, PP_USB_PHY_VBUS_WAKEUP_ID,
                                        VDAT_DET_CHG_DET, SET, RegVal);
        }
        USB_MISC_REGW(pUsbPhy, APB_MISC_PP_USB_PHY_VBUS_WAKEUP_ID_0, RegVal);
    }
    else
    {
        // disable the interrupt
        RegVal = USB_MISC_REGR(pUsbPhy, APB_MISC_PP_USB_PHY_VBUS_WAKEUP_ID_0);
        RegVal = NV_FLD_SET_DRF_DEF(APB_MISC, PP_USB_PHY_VBUS_WAKEUP_ID,
                                    VDAT_DET_INT_EN, DISABLE, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(APB_MISC, PP_USB_PHY_VBUS_WAKEUP_ID,
                                    VDAT_DET_CHG_DET, SET, RegVal);
        USB_MISC_REGW(pUsbPhy, APB_MISC_PP_USB_PHY_VBUS_WAKEUP_ID_0, RegVal);

        // Disable charger detection logic
        RegVal = USB_MISC_REGR(pUsbPhy, APB_MISC_UTMIP_BAT_CHRG_CFG0_0);
        RegVal = NV_FLD_SET_DRF_NUM(APB_MISC, UTMIP_BAT_CHRG_CFG0,
                                    UTMIP_OP_SRC_EN, 0, RegVal);
        RegVal = NV_FLD_SET_DRF_NUM(APB_MISC, UTMIP_BAT_CHRG_CFG0,
                                    UTMIP_ON_SINK_EN, 0, RegVal);
        USB_MISC_REGW(pUsbPhy, APB_MISC_UTMIP_BAT_CHRG_CFG0_0, RegVal);

        // Delay of 40 ms before we pull the D+ as per battery charger spec.
        NvOsSleepMS(TDPSRC_CON_MS);
    }

    return NvSuccess;
}


static NvError 
Ap16UsbPhyIoctlDedicatedChargerStatus(
    NvDdkUsbPhy *pUsbPhy, 
    void *pOutputArgs)
{
    NvDdkUsbPhyIoctl_DedicatedChargerStatusOutputArgs *pChargerStatus = NULL;
    NvU32 RegVal = 0;

    if (!pOutputArgs)
        return NvError_BadParameter;

    pChargerStatus = (NvDdkUsbPhyIoctl_DedicatedChargerStatusOutputArgs *)pOutputArgs;

    if (pUsbPhy->pProperty->UsbInterfaceType != NvOdmUsbInterfaceType_Utmi)
    {
        // Charger detection is not there for ULPI
        // return Charger not available
        pChargerStatus->ChargerDetected = NV_FALSE;
        return NvSuccess;
    }

    RegVal = USB_MISC_REGR(pUsbPhy, APB_MISC_PP_USB_PHY_VBUS_WAKEUP_ID_0);
    if (NV_DRF_VAL(APB_MISC, PP_USB_PHY_VBUS_WAKEUP_ID, VDAT_DET_STS, RegVal))
    {
        pChargerStatus->ChargerDetected = NV_TRUE;
    }
    else
    {
        pChargerStatus->ChargerDetected = NV_FALSE;
    }

    return NvSuccess;
}


static NvError 
Ap16UsbPhyIoctl(
    NvDdkUsbPhy *pUsbPhy,
    NvDdkUsbPhyIoctlType IoctlType,
    const void *pInputArgs,
    void *pOutputArgs)
{
    NvError ErrStatus = NvSuccess;

    switch (IoctlType)
    {
        case NvDdkUsbPhyIoctlType_VBusStatus:
            ErrStatus = Ap16UsbPhyIoctlVBusStatus(pUsbPhy, pOutputArgs);
            break;
        case NvDdkUsbPhyIoctlType_VBusInterrupt:
            ErrStatus = Ap16UsbPhyIoctlVbusInterrupt(pUsbPhy, pInputArgs);
            break;
        case NvDdkUsbPhyIoctlType_IdPinStatus:
            ErrStatus = Ap16UsbPhyIoctlIdPinStatus(pUsbPhy, pOutputArgs);
            break;
        case NvDdkUsbPhyIoctlType_IdPinInterrupt:
            ErrStatus = Ap16UsbPhyIoctlIdPinInterrupt(pUsbPhy, pInputArgs);
            break;
        case NvDdkUsbPhyIoctlType_DedicatedChargerStatus:
            ErrStatus = Ap16UsbPhyIoctlDedicatedChargerStatus(pUsbPhy, pOutputArgs);
            break;
        case NvDdkUsbPhyIoctlType_DedicatedChargerDetection:
            ErrStatus = Ap16UsbPhyIoctlDedicatedChargerDetection(pUsbPhy, pInputArgs);
            break;

        default:
            return NvError_NotSupported;
    }

    return ErrStatus;
}



static NvError
Ap16UsbPhyWaitForStableClock(
    NvDdkUsbPhy *pUsbPhy)
{
    NvU32 TimeOut = USB_PHY_HW_TIMEOUT_US;
    NvU32 PhyClkValid = 0;

    if (pUsbPhy->pProperty->UsbInterfaceType == NvOdmUsbInterfaceType_Utmi)
    {
        // Wait for the phy clock to become valid or hardware timeout
        do {
            PhyClkValid = USB_MISC_REGR(pUsbPhy, APB_MISC_PP_MISC_USB_OTG_0);
            PhyClkValid = NV_DRF_VAL(APB_MISC_PP, 
                                     MISC_USB_OTG, PCLKVLD, PhyClkValid);
            if (!TimeOut)
            {
                return NvError_Timeout;
            }
            NvOsWaitUS(1);
            TimeOut--;
        } while (PhyClkValid != APB_MISC_PP_MISC_USB_OTG_0_PCLKVLD_SET);
    }
    else
    {
        // Wait for the phy clock to become valid or hardware timeout
        do {
            PhyClkValid = USB_IF_REG_RD(SUSP_CTRL);
            PhyClkValid = NV_DRF_VAL(USB2_IF_USB, 
                                     SUSP_CTRL, USB_PHY_CLK_VALID, PhyClkValid);
            if (!TimeOut)
            {
                return NvError_Timeout;
            }
            NvOsWaitUS(1);
            TimeOut--;
        } while (PhyClkValid != USB2_IF_USB_SUSP_CTRL_0_USB_PHY_CLK_VALID_SET);
    }

    return NvSuccess;
}


static NvError 
Ap16UsbPhyPowerUp(
    NvDdkUsbPhy *pUsbPhy)
{
    NvError ErrVal = NvSuccess;

    switch (pUsbPhy->pProperty->UsbInterfaceType)
    {
        case NvOdmUsbInterfaceType_UlpiNullPhy:
            Ap16UsbPhyControlClockAndReset(pUsbPhy, NV_TRUE);
            Ap16UsbPhyUlpiNullModeConfigure(pUsbPhy);
            break;
        case NvOdmUsbInterfaceType_UlpiExternalPhy:
            pUsbPhy->hOdmUlpi = NvOdmUsbUlpiOpen(pUsbPhy->Instance);
            Ap16UsbPhyControlClockAndReset(pUsbPhy, NV_TRUE);
            Ap16UsbPhyUlpiLinkModeConfigure(pUsbPhy);
            break;
        case NvOdmUsbInterfaceType_Utmi:
            default:
            ErrVal = Ap16UsbPhyUtmiConfigure(pUsbPhy);
            if (ErrVal != NvSuccess)
                return ErrVal;
            Ap16UsbPhyUtmiPowerControl(pUsbPhy, NV_TRUE);
            Ap16UsbPhyControlClockAndReset(pUsbPhy, NV_TRUE);
            break;
    }

    ErrVal = Ap16UsbPhyWaitForStableClock(pUsbPhy);

    return ErrVal;
}

static NvError 
Ap16UsbPhyPowerDown(
    NvDdkUsbPhy *pUsbPhy)
{
    switch (pUsbPhy->pProperty->UsbInterfaceType)
    {
        case NvOdmUsbInterfaceType_UlpiExternalPhy:
            if (pUsbPhy->hOdmUlpi)
            {
                NvOdmUsbUlpiClose(pUsbPhy->hOdmUlpi);
                pUsbPhy->hOdmUlpi = NULL;
            }
            Ap16UsbPhyControlClockAndReset(pUsbPhy, NV_FALSE);
            break;
        case NvOdmUsbInterfaceType_UlpiNullPhy:
            Ap16UsbPhyControlClockAndReset(pUsbPhy, NV_FALSE);
            break;
        case NvOdmUsbInterfaceType_Utmi:
            default:
            Ap16UsbPhyUtmiPowerControl(pUsbPhy, NV_FALSE);
            Ap16UsbPhyControlClockAndReset(pUsbPhy, NV_FALSE);
            break;
    }

    return NvSuccess;
}

static void 
Ap16UsbPhyClose(
    NvDdkUsbPhy *pUsbPhy)
{
    // Power down the USB controller 
    NV_ASSERT_SUCCESS(Ap16UsbPhyPowerDown(pUsbPhy));
}

static void
Ap16PhySaveContext(
    NvDdkUsbPhy *pUsbPhy)
{
    // return
}

static void
Ap16PhyRestoreContext(
    NvDdkUsbPhy *pUsbPhy)
{
    // return
}

void
Ap16UsbPhyOpenHwInterface(
    NvDdkUsbPhy *pUsbPhy)
{
    pUsbPhy->PowerUp = Ap16UsbPhyPowerUp;
    pUsbPhy->PowerDown = Ap16UsbPhyPowerDown;
    pUsbPhy->Ioctl = Ap16UsbPhyIoctl;
    pUsbPhy->WaitForStableClock = Ap16UsbPhyWaitForStableClock;
    pUsbPhy->CloseHwInterface = Ap16UsbPhyClose;
    pUsbPhy->SaveContext = Ap16PhySaveContext;
    pUsbPhy->RestoreContext = Ap16PhyRestoreContext;

    Ap16UsbPhyReadXcvrSetupValueFromFuses(pUsbPhy);
}

