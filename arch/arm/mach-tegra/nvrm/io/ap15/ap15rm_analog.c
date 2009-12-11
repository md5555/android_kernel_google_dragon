/*
 * Copyright (c) 2008-2009 NVIDIA Corporation.
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

#include "nvcommon.h"
#include "nvrm_structure.h"
#include "nvrm_analog.h"
#include "nvrm_drf.h"
#include "nvassert.h"
#include "nvrm_hwintf.h"
#include "nvrm_power.h"
#include "ap16/arapb_misc.h"
#include "ap15/arclk_rst.h"
#include "ap15/arfuse.h"
#include "nvodm_query.h"
#include "nvodm_pmu.h"
#include "nvrm_clocks.h"
#include "nvrm_module.h"
#include "ap20/arusb.h"

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
NvRmFreqKHz s_RmOscFrequecy [NvRmClocksOscFreq_Num] =
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
// Tracking Length Time: The tracking circuit of the bias cell consumes a 
// measurable portion of the USB idle power  To curtail this power consumption 
// the bias pad has added a PD_TDK signal to power down the bias cell. It is 
// estimated that after 20microsec of bias cell operation the PD_TRK signal can 
// be turned high to sve power. This can be automated by programming a timing 
// interval as given in the below structure.
static const NvU32 s_UsbBiasTrkLengthTime[NvRmClocksOscFreq_Num] = 
{
    /* 20 micro seconds delay after bias cell operation */
    5,  // For NvBootClocksOscFreq_13,
    7,  // For NvBootClocksOscFreq_19_2
    5,  // For NvBootClocksOscFreq_12
    9   // For NvBootClocksOscFreq_26
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

// Reset USB host controller
static NvBool s_IsUSBResetRequired = NV_TRUE;

static NvError
NvRmPrivTvDcControl( NvRmDeviceHandle hDevice, NvBool enable, NvU32 inst,
    void *Config, NvU32 ConfigLength )
{
    NvRmAnalogTvDacConfig *cfg;
    NvU32 ctrl, source;
    NvU32 src_id;
    NvU32 src_inst;

    NV_ASSERT( ConfigLength == 0 ||
        ConfigLength == sizeof(NvRmAnalogTvDacConfig) );

    if( enable )
    {
        cfg = (NvRmAnalogTvDacConfig *)Config;
        NV_ASSERT( cfg );

        src_id = NVRM_MODULE_ID_MODULE( cfg->Source );
        src_inst = NVRM_MODULE_ID_INSTANCE( cfg->Source );

        ctrl = NV_DRF_DEF( APB_MISC_ASYNC, TVDACCNTL, DAC_IDDQ, DISABLE )
            | NV_DRF_DEF( APB_MISC_ASYNC, TVDACCNTL, DAC_POWERDOWN, DISABLE )
            | NV_DRF_DEF( APB_MISC_ASYNC, TVDACCNTL, DAC_DETECT_EN, ENABLE )
            | NV_DRF_DEF( APB_MISC_ASYNC, TVDACCNTL, DAC_SLEEPR, DISABLE )
            | NV_DRF_DEF( APB_MISC_ASYNC, TVDACCNTL, DAC_SLEEPG, DISABLE )
            | NV_DRF_DEF( APB_MISC_ASYNC, TVDACCNTL, DAC_SLEEPB, DISABLE )
            | NV_DRF_DEF( APB_MISC_ASYNC, TVDACCNTL, DAC_COMPR_EN, ENABLE )
            | NV_DRF_DEF( APB_MISC_ASYNC, TVDACCNTL, DAC_COMPG_EN, ENABLE )
            | NV_DRF_DEF( APB_MISC_ASYNC, TVDACCNTL, DAC_COMPB_EN, ENABLE );

        if( src_id == NvRmModuleID_Tvo )
        {
            source = NV_DRF_DEF( APB_MISC_ASYNC, TVDACDINCONFIG,
                DAC_SOURCE, TVO );
        }
        else
        {
            NV_ASSERT( src_id == NvRmModuleID_Display );
            if( src_inst == 0 )
            {
                source = NV_DRF_DEF( APB_MISC_ASYNC, TVDACDINCONFIG,
                    DAC_SOURCE, DISPLAY );
            }
            else
            {
                source = NV_DRF_DEF( APB_MISC_ASYNC, TVDACDINCONFIG,
                    DAC_SOURCE, DISPLAYB );
            }
        }

        source = NV_FLD_SET_DRF_NUM( APB_MISC_ASYNC, TVDACDINCONFIG, DAC_AMPIN,
            cfg->DacAmplitude, source );
    }
    else
    {
        ctrl = NV_DRF_DEF( APB_MISC_ASYNC, TVDACCNTL, DAC_IDDQ, ENABLE )
            | NV_DRF_DEF( APB_MISC_ASYNC, TVDACCNTL, DAC_POWERDOWN, ENABLE )
            | NV_DRF_DEF( APB_MISC_ASYNC, TVDACCNTL, DAC_DETECT_EN, DISABLE )
            | NV_DRF_DEF( APB_MISC_ASYNC, TVDACCNTL, DAC_SLEEPR, ENABLE )
            | NV_DRF_DEF( APB_MISC_ASYNC, TVDACCNTL, DAC_SLEEPG, ENABLE )
            | NV_DRF_DEF( APB_MISC_ASYNC, TVDACCNTL, DAC_SLEEPB, ENABLE )
            | NV_DRF_DEF( APB_MISC_ASYNC, TVDACCNTL, DAC_COMPR_EN, DISABLE )
            | NV_DRF_DEF( APB_MISC_ASYNC, TVDACCNTL, DAC_COMPG_EN, DISABLE )
            | NV_DRF_DEF( APB_MISC_ASYNC, TVDACCNTL, DAC_COMPB_EN, DISABLE );
        source = NV_DRF_DEF( APB_MISC_ASYNC, TVDACDINCONFIG,
                    DAC_SOURCE, TVDAC_OFF );
    }

    NV_REGW( hDevice, NvRmModuleID_Misc, 0, APB_MISC_ASYNC_TVDACCNTL_0,
        ctrl );
    NV_REGW( hDevice, NvRmModuleID_Misc, 0,
        APB_MISC_ASYNC_TVDACDINCONFIG_0, source );

    return NvSuccess;
}

static NvError
NvRmPrivVideoInputControl( NvRmDeviceHandle hDevice, NvBool enable,
    NvU32 inst, void *Config, NvU32 ConfigLength )
{
    NvU32 val;

    NV_ASSERT(ConfigLength == 0);
    NV_ASSERT(Config == 0);
    NV_ASSERT(inst == 0);

    if( enable )
    {
        val = NV_DRF_DEF( APB_MISC_ASYNC, VCLKCTRL, VCLK_PAD_IE, ENABLE );
    }
    else
    {
        val = NV_DRF_DEF( APB_MISC_ASYNC, VCLKCTRL, VCLK_PAD_IE, DISABLE );
    }

    NV_REGW( hDevice, NvRmModuleID_Misc, 0, APB_MISC_ASYNC_VCLKCTRL_0,
        val );

    return NvSuccess;
}


static void
NvRmPrivUsbfSetUlpiLinkTrimmers(
    NvRmDeviceHandle hDevice,
    NvU32 instance)
{
    NvU32 RegVal = 0;

    // Bypass the Pin Mux on the ULPI outputs and set the trimmer values for inputs to 3
    RegVal = NV_REGR(hDevice, NvRmModuleID_Misc, 0, APB_MISC_UTMIP_SPARE_CFG0_0);
    RegVal = RegVal & ( ~( (0x1 << 13) | (0xf << 4) | (0xf << 28) ) );
    //bit 13        : data output pinmux bypass enable: set to 1
    //bit 4         : data input trimmer load enable (toggle)
    //bit [7:5]     : data input trimmer value: set to 3
    //bit 28        : data input trimmer2 load enable (toggle)
    //bit [31:29]   : data input trimmer2 value: set to 3
    RegVal = RegVal | ( (0x1 << 13) | (0x0 << 5) | (0x0 << 4) | (0x0 << 29) | (0x0 << 28) );
    NV_REGW(hDevice, NvRmModuleID_Misc, 0, APB_MISC_UTMIP_SPARE_CFG0_0, RegVal);

    // wait 10 us
    NvOsWaitUS(10);
    // toggle bits 4 and 28 to latch the trimmer values
    RegVal = RegVal | ( (0x1 << 4) | (0x1 << 28) );
    NV_REGW(hDevice, NvRmModuleID_Misc, 0, APB_MISC_UTMIP_SPARE_CFG0_0, RegVal);
    // wait 10 us
    NvOsWaitUS(10);
}


static void
NvRmPrivUsbfSetUlpiNullTrimmers(
    NvRmDeviceHandle hDevice,
    NvU32 instance)
{
    NvU32 RegVal = 0;
    // Configure 60M clock for USB2 - ULPI controller

     // Set up to use PLLU at 60 MHz and keep USB PHY PLL in reset
    // bit 27        : bypass 60 MHz Div5 for PLLU - set to 1
    // bit 9         : nullphy_pll_source - use USB_PHY_PLL output (set to 0) :
    //                    Workaround: set to 1 to use PLLU Output at 12 MHz

    RegVal = NV_REGR(hDevice, NvRmModuleID_Misc, 0,
                                                    APB_MISC_UTMIP_SPARE_CFG0_0);
    RegVal &= (~( 0x1 << 27) );
    RegVal |= ((0x1 << 9));
    NV_REGW(hDevice, NvRmModuleID_Misc, 0, APB_MISC_UTMIP_SPARE_CFG0_0,
                                        RegVal);

    // Set the trimmers
    // bit 4         : data input trimmer load enable (toggle)
    // bit [7:5]     : data input trimmer value - set to 3
    // bit 28        : data input trimmer2 load enable - (toggle)
    // bit [31:29]   : data input trimmer2 value - set to 3
    RegVal = NV_REGR(hDevice, NvRmModuleID_Misc, 0, APB_MISC_UTMIP_SPARE_CFG0_0);
    RegVal = RegVal | ( (0x7 << 29) | (0x0 << 28) | (0x7 <<  5) | (0x0 <<  4) );
    NV_REGW(hDevice, NvRmModuleID_Misc, 0, APB_MISC_UTMIP_SPARE_CFG0_0, RegVal);

     // wait 10 us
    NvOsWaitUS(10);
    // toggle bits 4 and 28 to latch the trimmer values
    RegVal = RegVal | ( (0x1 << 4) | (0x1 << 28) );
    NV_REGW(hDevice, NvRmModuleID_Misc, 0, APB_MISC_UTMIP_SPARE_CFG0_0, RegVal);
    // wait 10 us
    NvOsWaitUS(10);
}


static void
NvRmPrivUsbfUlpiClockControl(
    NvRmDeviceHandle hDevice,
    NvU32 instance,
    NvBool Enable)
{
    NvU32 RegVal = 0;

    if (Enable)
    {
        RegVal = NV_REGR(hDevice, NvRmModuleID_Misc, 0, 
                                                        APB_MISC_PP_MISC_USB_CLK_RST_CTL_0);
        RegVal = NV_FLD_SET_DRF_DEF(APB_MISC_PP, MISC_USB_CLK_RST_CTL,
                                                                MISC_USB2_RST, DISABLE, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(APB_MISC_PP, MISC_USB_CLK_RST_CTL,
                                                                MISC_USB2_CE, ENABLE, RegVal);
         NV_REGW(hDevice, NvRmModuleID_Misc, 0,
                                                        APB_MISC_PP_MISC_USB_CLK_RST_CTL_0, RegVal);

        NvOsMutexLock(hDevice->CarMutex);
        // Bring Out of reset
        RegVal = NV_REGR(hDevice, NvRmPrivModuleID_ClockAndReset, 0,
                                                        CLK_RST_CONTROLLER_RST_DEVICES_L_0);
        RegVal = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, RST_DEVICES_L,
                                                                SWR_USBD_RST, DISABLE, RegVal);
        NV_REGW(hDevice, NvRmPrivModuleID_ClockAndReset, 0,
                                            CLK_RST_CONTROLLER_RST_DEVICES_L_0, RegVal);

        NvOsMutexUnlock(hDevice->CarMutex);

        RegVal = NV_REGR(hDevice, NvRmModuleID_Misc, 0,
                                                     APB_MISC_UTMIP_SPARE_CFG0_0);
        RegVal = RegVal & (0xf);

        RegVal = RegVal | ( (0x0 << 29) | (0x0 << 28) | (0x0 << 27) | (0 << 22) |
                                     (0x0 << 21) | (0x0 << 16) | (0x0 << 15) | (0x1 << 14) | 
                                     (0x1 << 13) | (0x0 << 12) | (0x1 << 11) | (0x1 << 10) | 
                                     (0x1 << 9) | (0x0 << 5) | (0x0 << 4) );
        NV_REGW(hDevice, NvRmModuleID_Misc, 0, APB_MISC_UTMIP_SPARE_CFG0_0, RegVal);

        // wait 10 us
        NvOsWaitUS(10);
    }
    else
    {
        RegVal = NV_REGR(hDevice, NvRmModuleID_Misc, 0,
                                                        APB_MISC_PP_MISC_USB_CLK_RST_CTL_0);
        RegVal = NV_FLD_SET_DRF_DEF(APB_MISC_PP, MISC_USB_CLK_RST_CTL,
                                                                MISC_USB2_RST, ENABLE, RegVal);
         RegVal = NV_FLD_SET_DRF_DEF(APB_MISC_PP, MISC_USB_CLK_RST_CTL,
                                                                MISC_USB2_CE, DISABLE, RegVal);
        NV_REGW(hDevice, NvRmModuleID_Misc, 0,
                                            APB_MISC_PP_MISC_USB_CLK_RST_CTL_0, RegVal);

        if (!(NV_DRF_VAL(APB_MISC_PP, MISC_USB_CLK_RST_CTL, MISC_USB_CE, RegVal)))
        {
            // Enable reset
            NvOsMutexLock(hDevice->CarMutex);
            RegVal = NV_REGR(hDevice, NvRmPrivModuleID_ClockAndReset, 0,
                                                            CLK_RST_CONTROLLER_RST_DEVICES_L_0);
            RegVal = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, RST_DEVICES_L,
                                                                SWR_USBD_RST, ENABLE, RegVal);
            NV_REGW(hDevice, NvRmPrivModuleID_ClockAndReset, 0,
                                                CLK_RST_CONTROLLER_RST_DEVICES_L_0, RegVal);
            NvOsMutexUnlock(hDevice->CarMutex);
        }
    }
}

static void
NvRmPrivUsbfEnableVbusInterrupt(
    NvRmDeviceHandle hDevice)
{
    NvU32 RegVal = 0;

    //enable VBUS interrupt for cable detection when controller is Off
    RegVal = NV_REGR(hDevice, NvRmModuleID_Misc, 0,
                                                    APB_MISC_PP_USB_PHY_VBUS_SENSORS_0);
    RegVal = NV_FLD_SET_DRF_DEF(APB_MISC_PP, USB_PHY_VBUS_SENSORS,
                                                            A_SESS_VLD_INT_EN, ENABLE, RegVal);
    NV_REGW(hDevice, NvRmModuleID_Misc, 0,
                                        APB_MISC_PP_USB_PHY_VBUS_SENSORS_0, RegVal);
}


static void
NvRmPrivUsbfDisableVbusInterrupt(
    NvRmDeviceHandle hDevice)
{
    NvU32 RegVal = 0;

    //disable the VBUS interrupt, 
    RegVal = NV_REGR(hDevice, NvRmModuleID_Misc, 0,
                                                    APB_MISC_PP_USB_PHY_VBUS_SENSORS_0);
    RegVal = NV_FLD_SET_DRF_DEF(APB_MISC_PP, USB_PHY_VBUS_SENSORS,
                                                            A_SESS_VLD_INT_EN, DISABLE, RegVal);
    RegVal = NV_FLD_SET_DRF_DEF(APB_MISC_PP, USB_PHY_VBUS_SENSORS,
                                                            A_SESS_VLD_CHG_DET, SET, RegVal);
    NV_REGW(hDevice, NvRmModuleID_Misc, 0,
                                        APB_MISC_PP_USB_PHY_VBUS_SENSORS_0, RegVal);
}


static NvBool
NvRmPrivUsbfIsCableConnected(
    NvRmDeviceHandle hDevice)
{
    NvU32 RegVal = 0;
    NvBool CableConnected = NV_FALSE;

    // Check for cable connection
    RegVal = NV_REGR(hDevice, NvRmModuleID_Misc, 0,
                                                    APB_MISC_PP_USB_PHY_VBUS_SENSORS_0);
    if (NV_DRF_VAL(APB_MISC, PP_USB_PHY_VBUS_SENSORS, A_SESS_VLD_STS, RegVal))
    {
        CableConnected = NV_TRUE;
    }
    //disable the interrupt, if we detect the cable connection/dis connection
    NvRmPrivUsbfDisableVbusInterrupt(hDevice);

    if (!CableConnected)
    {
        NvRmPrivUsbfEnableVbusInterrupt(hDevice);
    }

    return CableConnected;
}

static NvError
NvRmPrivUsbfWaitForPhyClock(
    NvRmDeviceHandle hDevice,
    NvBool Enable)
{
    NvU32 TimeOut = 100000;   // 100 milli seconds timeout before H/W gives up;
    NvU32 PhyClockValidStatus = APB_MISC_PP_MISC_USB_OTG_0_PCLKVLD_UNSET;
    NvU32 PhyClkValid = 0;
    NvU32 RegVal = 0;
    // If Enable is true, check for PHY Clock Vailid Set
    // Else check for PHY Clock Unset
    if (Enable)
        PhyClockValidStatus = APB_MISC_PP_MISC_USB_OTG_0_PCLKVLD_SET;
    // Wait for the phy clock to become valid or hardware timeout
    do {
        RegVal = NV_REGR(hDevice, NvRmModuleID_Misc, 0, APB_MISC_PP_MISC_USB_OTG_0);
        PhyClkValid = NV_DRF_VAL(APB_MISC_PP, MISC_USB_OTG, PCLKVLD, RegVal);
        if (!TimeOut)
        {
            return NvError_Timeout;
        }
        NvOsWaitUS(1);
        TimeOut--;
    } while (PhyClkValid != PhyClockValidStatus);

    return NvSuccess;
}

static NvBool
NvRmPrivUsbfIsChargerDetected(
    NvRmDeviceHandle hDevice, NvBool EnableDetection)
{
    NvU32 RegVal = 0;
    NvBool ChargerConnected = NV_FALSE;
    #define TDP_SRC_ON_MS    100

    if (EnableDetection)
    {
        // Enable charger detection logic
        RegVal = NV_REGR(hDevice, NvRmModuleID_Misc, 0,
                                                        APB_MISC_UTMIP_BAT_CHRG_CFG0_0);
        RegVal = NV_FLD_SET_DRF_NUM(APB_MISC, UTMIP_BAT_CHRG_CFG0,
                                                                UTMIP_OP_SRC_EN, 1, RegVal);
        RegVal = NV_FLD_SET_DRF_NUM(APB_MISC, UTMIP_BAT_CHRG_CFG0,
                                                                UTMIP_ON_SINK_EN, 1, RegVal);
        NV_REGW(hDevice, NvRmModuleID_Misc, 0,
                                            APB_MISC_UTMIP_BAT_CHRG_CFG0_0, RegVal);
        // Source should be on for 100 ms as per USB charging spec
        NvOsSleepMS(TDP_SRC_ON_MS);
    }

    RegVal = NV_REGR(hDevice, NvRmModuleID_Misc, 0,
                                                    APB_MISC_PP_USB_PHY_VBUS_WAKEUP_ID_0);
    if (NV_DRF_VAL(APB_MISC, PP_USB_PHY_VBUS_WAKEUP_ID, VDAT_DET_STS, RegVal))
    {
        //disable the interrupt, if we detect the charger
        RegVal = NV_FLD_SET_DRF_DEF(APB_MISC, PP_USB_PHY_VBUS_WAKEUP_ID,
                                                                VDAT_DET_INT_EN, DISABLE, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(APB_MISC, PP_USB_PHY_VBUS_WAKEUP_ID,
                                                                VDAT_DET_CHG_DET, SET, RegVal);
        NV_REGW(hDevice, NvRmModuleID_Misc, 0,
                                            APB_MISC_PP_USB_PHY_VBUS_WAKEUP_ID_0, RegVal);
        ChargerConnected = NV_TRUE;
        // Disable charger detection logic
        RegVal = NV_REGR(hDevice, NvRmModuleID_Misc, 0,
                                                        APB_MISC_UTMIP_BAT_CHRG_CFG0_0);
        RegVal = NV_FLD_SET_DRF_NUM(APB_MISC, UTMIP_BAT_CHRG_CFG0,
                                                                UTMIP_OP_SRC_EN, 0, RegVal);
        RegVal = NV_FLD_SET_DRF_NUM(APB_MISC, UTMIP_BAT_CHRG_CFG0,
                                                                UTMIP_ON_SINK_EN, 0, RegVal);
        NV_REGW(hDevice, NvRmModuleID_Misc, 0,
                                            APB_MISC_UTMIP_BAT_CHRG_CFG0_0, RegVal);

    }

    return ChargerConnected;
}

static void
NvRmPrivUsbfChargerDetection(
    NvRmDeviceHandle hDevice,
    NvBool Enable)
{
    // These values (in milli second) are taken from the battery charging spec.
    #define TDP_SRC_ON_MS    100
    #define TDPSRC_CON_MS    40
    NvU32 RegVal = 0;

    if (Enable)
    {
        // Enable charger detection logic
        RegVal = NV_REGR(hDevice, NvRmModuleID_Misc, 0,
                                                        APB_MISC_UTMIP_BAT_CHRG_CFG0_0);
        RegVal = NV_FLD_SET_DRF_NUM(APB_MISC, UTMIP_BAT_CHRG_CFG0,
                                                                UTMIP_OP_SRC_EN, 1, RegVal);
        RegVal = NV_FLD_SET_DRF_NUM(APB_MISC, UTMIP_BAT_CHRG_CFG0,
                                                                UTMIP_ON_SINK_EN, 1, RegVal);
        NV_REGW(hDevice, NvRmModuleID_Misc, 0,
                                            APB_MISC_UTMIP_BAT_CHRG_CFG0_0, RegVal);
        // Source should be on for 100 ms as per USB charging spec
        NvOsSleepMS(TDP_SRC_ON_MS);
        // Check if charger is connected, enable interrupt to get the event
        RegVal = NV_REGR(hDevice, NvRmModuleID_Misc, 0,
                                                        APB_MISC_PP_USB_PHY_VBUS_WAKEUP_ID_0);
        if (NV_DRF_VAL(APB_MISC, PP_USB_PHY_VBUS_WAKEUP_ID, VDAT_DET_STS, RegVal))
        {
            NV_REGW(hDevice, NvRmModuleID_Misc, 0,
                                                APB_MISC_PP_USB_PHY_VBUS_WAKEUP_ID_0,
                    NV_DRF_DEF(APB_MISC, PP_USB_PHY_VBUS_WAKEUP_ID,
                                            VDAT_DET_INT_EN, ENABLE));
        }
        else
        {
            // If charger is not connected disable the interrupt
            RegVal = NV_FLD_SET_DRF_DEF(APB_MISC, PP_USB_PHY_VBUS_WAKEUP_ID,
                                                                    VDAT_DET_INT_EN, DISABLE, RegVal);
            RegVal = NV_FLD_SET_DRF_DEF(APB_MISC, PP_USB_PHY_VBUS_WAKEUP_ID,
                                                                    VDAT_DET_CHG_DET, SET, RegVal);
            NV_REGW(hDevice, NvRmModuleID_Misc, 0,
                                                APB_MISC_PP_USB_PHY_VBUS_WAKEUP_ID_0, RegVal);
        }
    }
    else
    {
        // Disable charger detection logic
        RegVal = NV_REGR(hDevice, NvRmModuleID_Misc, 0,
                                                        APB_MISC_UTMIP_BAT_CHRG_CFG0_0);
        RegVal = NV_FLD_SET_DRF_NUM(APB_MISC, UTMIP_BAT_CHRG_CFG0,
                                                                UTMIP_OP_SRC_EN, 0, RegVal);
        RegVal = NV_FLD_SET_DRF_NUM(APB_MISC, UTMIP_BAT_CHRG_CFG0,
                                                                UTMIP_ON_SINK_EN, 0, RegVal);
        NV_REGW(hDevice, NvRmModuleID_Misc, 0,
                                            APB_MISC_UTMIP_BAT_CHRG_CFG0_0, RegVal);
        // Delay of 40 ms before we pull the D+ as per battery charger spec.
        NvOsSleepMS(TDPSRC_CON_MS);
    }
}


static NvError
NvRmPrivUsb3ConfigureUtmipPhy(
    NvRmDeviceHandle hDevice)
{
    NvU32 RegVal = 0;
    NvU32 TimeOut = 100000;   // 100 milli seconds timeout before H/W gives up;
    NvU32 PhyClkValid = 0;
    NvRmFreqKHz OscFreqKz = 0;
    NvU32 FreqIndex;

    // Get the Oscillator Frequency
    OscFreqKz = NvRmPowerGetPrimaryFrequency(hDevice);

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


    /**Hold UTMIP3 PHY in reset by writing UTMIP_RESET bit in USB3_IF_USB_SUSP_CTRL 
    register to 1. **/
    RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 2, USB3_IF_USB_SUSP_CTRL_0);
    RegVal = NV_FLD_SET_DRF_DEF(USB3_IF, USB_SUSP_CTRL, UTMIP_RESET, ENABLE, RegVal);
    NV_REGW(hDevice, NvRmModuleID_Usb2Otg, 2, USB3_IF_USB_SUSP_CTRL_0, RegVal);


    /*1.	FORCE_PD_POWERDOWN, FORCE_PD2_POWERDOWN, FORCE_PDZI_POWERDOWN fields in 
    UTMIP_XCVR_CFG0 register.  **/
    RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 2, USB3_UTMIP_XCVR_CFG0_0);
    RegVal = NV_FLD_SET_DRF_NUM(USB3_UTMIP, XCVR_CFG0, UTMIP_FORCE_PD_POWERDOWN,
                    0x0, RegVal);
    RegVal = NV_FLD_SET_DRF_NUM(USB3_UTMIP, XCVR_CFG0, UTMIP_FORCE_PD2_POWERDOWN,
                    0x0, RegVal);
    RegVal = NV_FLD_SET_DRF_NUM(USB3_UTMIP, XCVR_CFG0, UTMIP_FORCE_PDZI_POWERDOWN,
                    0x0, RegVal);
    NV_REGW(hDevice, NvRmModuleID_Usb2Otg, 2, USB3_UTMIP_XCVR_CFG0_0, RegVal);


        // USB Power Up sequence
        // Power Up OTG and Bias circuitry
        RegVal = NV_REGR(hDevice, NvRmModuleID_Misc, 0, APB_MISC_UTMIP_BIAS_CFG0_0);
        RegVal = NV_FLD_SET_DRF_NUM(APB_MISC, UTMIP_BIAS_CFG0,
                            UTMIP_OTGPD, 0, RegVal);
        RegVal = NV_FLD_SET_DRF_NUM(APB_MISC, UTMIP_BIAS_CFG0,
                            UTMIP_BIASPD, 0, RegVal);
        NV_REGW(hDevice, NvRmModuleID_Misc, 0, APB_MISC_UTMIP_BIAS_CFG0_0, RegVal);

    /**OTGOD and BIASPD fields in UTMIP_BIAS_CFG0 register. **/
    RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 2, USB3_UTMIP_BIAS_CFG0_0);
    RegVal = NV_FLD_SET_DRF_NUM(USB3_UTMIP, BIAS_CFG0, UTMIP_OTGPD,
                    0x0, RegVal);
    RegVal = NV_FLD_SET_DRF_NUM(USB3_UTMIP, BIAS_CFG0, UTMIP_BIASPD,
                    0x0, RegVal);
    NV_REGW(hDevice, NvRmModuleID_Usb2Otg, 2, USB3_UTMIP_BIAS_CFG0_0, RegVal);

    /* FORCE_PDDISC_POWERDOWEN , FORCE_PDCHRP_POWERDOWN, FORCE_PDDR_POWERDOWN 
    field in UTMIP_XCVR_CFG1 register.  **/
    RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 2, USB3_UTMIP_XCVR_CFG1_0);
    RegVal = NV_FLD_SET_DRF_NUM(USB3_UTMIP, XCVR_CFG1, UTMIP_FORCE_PDDISC_POWERDOWN,
                    0x0, RegVal);
    RegVal = NV_FLD_SET_DRF_NUM(USB3_UTMIP, XCVR_CFG1, UTMIP_FORCE_PDCHRP_POWERDOWN,
                    0x0, RegVal);
    RegVal = NV_FLD_SET_DRF_NUM(USB3_UTMIP, XCVR_CFG1, UTMIP_FORCE_PDDR_POWERDOWN,
                    0x0, RegVal);
    NV_REGW(hDevice, NvRmModuleID_Usb2Otg, 2, USB3_UTMIP_XCVR_CFG1_0, RegVal);

    /**Enable UTMIP3 interface by setting UTMIP_PHY_ENB in USB3_IF_USB_SUSP_CTRL 
    register to 1.  **/
    RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 2, USB3_IF_USB_SUSP_CTRL_0);
    RegVal = NV_FLD_SET_DRF_DEF(USB3_IF, USB_SUSP_CTRL, UTMIP_PHY_ENB, ENABLE, RegVal);
    NV_REGW(hDevice, NvRmModuleID_Usb2Otg, 2, USB3_IF_USB_SUSP_CTRL_0, RegVal);

    RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 2, USB3_UTMIP_MISC_CFG1_0);
    RegVal = NV_FLD_SET_DRF_NUM(USB3_UTMIP, MISC_CFG1,
                    UTMIP_PLLU_STABLE_COUNT,
                    s_UsbPllDelayParams[FreqIndex].StableCount, RegVal);
    RegVal = NV_FLD_SET_DRF_NUM(USB3_UTMIP, MISC_CFG1,
                UTMIP_PLL_ACTIVE_DLY_COUNT,
                s_UsbPllDelayParams[FreqIndex].ActiveDelayCount, RegVal);
    NV_REGW(hDevice, NvRmModuleID_Usb2Otg, 2, USB3_UTMIP_MISC_CFG1_0, RegVal);


    // Set PLL enable delay count and Crystal frequency count
    RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 2, USB3_UTMIP_PLL_CFG1_0);
    RegVal = NV_FLD_SET_DRF_NUM(USB3_UTMIP, PLL_CFG1,
                UTMIP_PLLU_ENABLE_DLY_COUNT,
                s_UsbPllDelayParams[FreqIndex].EnableDelayCount, RegVal);
    RegVal = NV_FLD_SET_DRF_NUM(USB3_UTMIP,
                PLL_CFG1, UTMIP_XTAL_FREQ_COUNT,
                s_UsbPllDelayParams[FreqIndex].XtalFreqCount, RegVal);
    NV_REGW(hDevice, NvRmModuleID_Usb2Otg, 2, USB3_UTMIP_PLL_CFG1_0, RegVal);

    // Program 1ms Debounce time for VBUS to become valid.
    RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 2, USB3_UTMIP_DEBOUNCE_CFG0_0);
    RegVal = NV_FLD_SET_DRF_NUM(USB3_UTMIP, DEBOUNCE_CFG0, UTMIP_BIAS_DEBOUNCE_A,
                s_UsbBiasDebounceATime[FreqIndex], RegVal);
    NV_REGW(hDevice, NvRmModuleID_Usb2Otg, 2, USB3_UTMIP_DEBOUNCE_CFG0_0, RegVal);

    /** pll_parameters_configured  **/

    // Configure the UTMIP_HS_SYNC_START_DLY
    RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 2, USB3_UTMIP_HSRX_CFG1_0);
    RegVal = NV_FLD_SET_DRF_NUM(USB3_UTMIP, HSRX_CFG1,
                    UTMIP_HS_SYNC_START_DLY,
                    s_UtmipHsSyncStartDelay, RegVal);
    NV_REGW(hDevice, NvRmModuleID_Usb2Otg, 2, USB3_UTMIP_HSRX_CFG1_0, RegVal);

    /* Configure the UTMIP_IDLE_WAIT and UTMIP_ELASTIC_LIMIT
    * Setting these fields, together with default values of the other
    * fields, results in programming the registers below as follows:
    *         UTMIP_HSRX_CFG0 = 0x9168c000
    *         UTMIP_HSRX_CFG1 = 0x13
    */

    // Set PLL enable delay count and Crystal frequency count
    RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 2, USB3_UTMIP_HSRX_CFG0_0);
    RegVal = NV_FLD_SET_DRF_NUM(USB3_UTMIP, HSRX_CFG0,
                    UTMIP_IDLE_WAIT,
                    s_UtmipIdleWaitDelay, RegVal);
    RegVal = NV_FLD_SET_DRF_NUM(USB3_UTMIP,
                    HSRX_CFG0, UTMIP_ELASTIC_LIMIT,
                    s_UtmipElasticLimit, RegVal);
    NV_REGW(hDevice, NvRmModuleID_Usb2Otg, 2, USB3_UTMIP_HSRX_CFG0_0, RegVal);

    /**Release reset to UTMIP3 by writing 0 to UTMIP_RESET field in 
    USB3_IF_USB_SUSP_CTRL register. ***/
    RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 2, USB3_IF_USB_SUSP_CTRL_0);
    RegVal = NV_FLD_SET_DRF_DEF(USB3_IF, USB_SUSP_CTRL, UTMIP_RESET, DISABLE, RegVal);
    NV_REGW(hDevice, NvRmModuleID_Usb2Otg, 2, USB3_IF_USB_SUSP_CTRL_0, RegVal);


    /**Wait until PHY clock comes up by checking for USB_PHY_CLK_VALID bit in 
    USB3_IF_USB_SUSP_CTRL register  **/
    do
    {
        //Wait for the phy clock to become valid
        RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 2, USB3_IF_USB_SUSP_CTRL_0);
        PhyClkValid = NV_DRF_VAL(USB3_IF, USB_SUSP_CTRL, USB_PHY_CLK_VALID, RegVal);

        if (!TimeOut)
        {
            break;
        }
        NvOsWaitUS(1);
        TimeOut--;
    } while (!PhyClkValid);

    /* We can only do this once PHY clock is up. Disable ICUSB interface (it is enabled by default)  */
    RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 2, USB2_CONTROLLER_1_USB2D_ICUSB_CTRL_0);
    RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ICUSB_CTRL, IC_ENB1, DISABLE, RegVal);
    NV_REGW(hDevice, NvRmModuleID_Usb2Otg, 2, USB2_CONTROLLER_1_USB2D_ICUSB_CTRL_0, RegVal);

    /**Program the USB3 controller to use UTMIP3 PHY by setting the PTS field in 
    USB2_CONTROLLER_USB2D_PORTSC1 register to UTMIP (2'b00).  **/
    RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 2, USB2_CONTROLLER_USB2D_PORTSC1_0);

    RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER, USB2D_PORTSC1, PTS, UTMI, RegVal);
    RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER, USB2D_PORTSC1, STS, PARALLEL_IF, RegVal);
    NV_REGW(hDevice, NvRmModuleID_Usb2Otg, 2, USB2_CONTROLLER_USB2D_PORTSC1_0, RegVal);

    return NvSuccess;
}


static NvError
NvRmPrivUsbfConfigureUtmipPhy(
    NvRmDeviceHandle hDevice)
{
    NvU32 RegVal = 0;
    NvRmFreqKHz OscFreqKz = 0;
    NvU32 FreqIndex;

    // Get the Oscillator Frequency
    OscFreqKz = NvRmPowerGetPrimaryFrequency(hDevice);

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

    RegVal = NV_REGR(hDevice, NvRmModuleID_Misc, 0, APB_MISC_UTMIP_TX_CFG0_0);
    RegVal = NV_FLD_SET_DRF_NUM(APB_MISC, UTMIP_TX_CFG0, UTMIP_FS_PREAMBLE_J,
                                                            0x1, RegVal);
    NV_REGW(hDevice, NvRmModuleID_Misc, 0, APB_MISC_UTMIP_TX_CFG0_0, RegVal);

    // Configure the UTMIP_IDLE_WAIT and UTMIP_ELASTIC_LIMIT
    // Setting these fields, together with default values of the other
    // fields, results in programming the registers below as follows:
    //         UTMIP_HSRX_CFG0 = 0x9168c000
    //         UTMIP_HSRX_CFG1 = 0x13
    RegVal = NV_REGR(hDevice, NvRmModuleID_Misc, 0, APB_MISC_UTMIP_HSRX_CFG0_0);
    RegVal = NV_FLD_SET_DRF_NUM(APB_MISC, UTMIP_HSRX_CFG0, UTMIP_IDLE_WAIT,
                                s_UtmipIdleWaitDelay, RegVal);
    RegVal = NV_FLD_SET_DRF_NUM(APB_MISC, UTMIP_HSRX_CFG0, UTMIP_ELASTIC_LIMIT,
                                s_UtmipElasticLimit, RegVal);
    NV_REGW(hDevice, NvRmModuleID_Misc, 0, APB_MISC_UTMIP_HSRX_CFG0_0, RegVal);

    // Configure the UTMIP_HS_SYNC_START_DLY
    RegVal = NV_REGR(hDevice, NvRmModuleID_Misc, 0, APB_MISC_UTMIP_HSRX_CFG1_0);
    RegVal = NV_FLD_SET_DRF_NUM(APB_MISC, UTMIP_HSRX_CFG1,
                                UTMIP_HS_SYNC_START_DLY, s_UtmipHsSyncStartDelay, RegVal);
    NV_REGW(hDevice, NvRmModuleID_Misc, 0, APB_MISC_UTMIP_HSRX_CFG1_0, RegVal);

    // Program 1ms Debounce time for VBUS to become valid.
    RegVal = NV_REGR(hDevice, NvRmModuleID_Misc, 0,
                            APB_MISC_UTMIP_DEBOUNCE_CFG0_0);
    RegVal = NV_FLD_SET_DRF_NUM(APB_MISC, UTMIP_DEBOUNCE_CFG0,
                            UTMIP_BIAS_DEBOUNCE_A,s_UsbBiasDebounceATime[FreqIndex],
                            RegVal);
    NV_REGW(hDevice, NvRmModuleID_Misc, 0,
                            APB_MISC_UTMIP_DEBOUNCE_CFG0_0, RegVal);

    // PLL Delay CONFIGURATION settings
    // The following parameters control the bring up of the plls:
    RegVal = NV_REGR(hDevice, NvRmModuleID_Misc, 0, APB_MISC_UTMIP_MISC_CFG0_0);
    RegVal = NV_FLD_SET_DRF_NUM(APB_MISC, UTMIP_MISC_CFG0,
                            UTMIP_SUSPEND_EXIT_ON_EDGE, 0, RegVal);
    NV_REGW(hDevice, NvRmModuleID_Misc, 0, APB_MISC_UTMIP_MISC_CFG0_0, RegVal);

    RegVal = NV_REGR(hDevice, NvRmModuleID_Misc, 0, APB_MISC_UTMIP_MISC_CFG1_0);
    RegVal = NV_FLD_SET_DRF_NUM(APB_MISC, UTMIP_MISC_CFG1,
                                UTMIP_PLLU_STABLE_COUNT,
                                s_UsbPllDelayParams[FreqIndex].StableCount, RegVal);
    RegVal = NV_FLD_SET_DRF_NUM(APB_MISC, UTMIP_MISC_CFG1,
                                UTMIP_PLL_ACTIVE_DLY_COUNT,
                                s_UsbPllDelayParams[FreqIndex].ActiveDelayCount, RegVal);
    NV_REGW(hDevice, NvRmModuleID_Misc, 0, APB_MISC_UTMIP_MISC_CFG1_0, RegVal);

    // Set PLL enable delay count and Crystal frequency count
    RegVal = NV_REGR(hDevice, NvRmModuleID_Misc, 0, APB_MISC_UTMIP_PLL_CFG1_0);
    RegVal = NV_FLD_SET_DRF_NUM(APB_MISC, UTMIP_PLL_CFG1,
                        UTMIP_PLLU_ENABLE_DLY_COUNT,
                        s_UsbPllDelayParams[FreqIndex].EnableDelayCount, RegVal);
    RegVal = NV_FLD_SET_DRF_NUM(APB_MISC,
                        UTMIP_PLL_CFG1, UTMIP_XTAL_FREQ_COUNT,
                        s_UsbPllDelayParams[FreqIndex].XtalFreqCount, RegVal);
    NV_REGW(hDevice, NvRmModuleID_Misc, 0, APB_MISC_UTMIP_PLL_CFG1_0, RegVal);

    // On AP20 FPGA we do not have VBUS_WAKEUP signal for cable detection.
    // We use A_SESS_VLD that comes from the external UTMIP PHY
    if (NvRmPrivGetExecPlatform(hDevice) == ExecPlatform_Fpga)
    {
        RegVal = NV_REGR(hDevice, NvRmModuleID_Misc, 0, APB_MISC_PP_USB_PHY_PARAM_0);
        RegVal = NV_FLD_SET_DRF_DEF(APB_MISC, PP_USB_PHY_PARAM, VS_CTL, A_SESS_VLD, RegVal);
        NV_REGW(hDevice, NvRmModuleID_Misc, 0, APB_MISC_PP_USB_PHY_PARAM_0, RegVal);
    }
    return NvSuccess;
}

static void
NvRmPrivUsbfPowerControl(
    NvRmDeviceHandle hDevice,
    NvU32 Instance,
    NvBool Enable)
{
    NvU32 RegVal = 0;
    const NvOdmUsbProperty *pUsbProperty = NULL;
    NvU32 TimeOut = 100000;   // 100 milli seconds timeout before H/W gives up;
    static NvU32 s_XcvrSetupValue = 0; 
    static NvBool s_ReadFuseValue = NV_FALSE;

    if (Enable)
    {
        // USB Power Up sequence
        // Power Up OTG and Bias circuitry
        RegVal = NV_REGR(hDevice, NvRmModuleID_Misc, 0, APB_MISC_UTMIP_BIAS_CFG0_0);
        RegVal = NV_FLD_SET_DRF_NUM(APB_MISC, UTMIP_BIAS_CFG0,
                            UTMIP_OTGPD, 0, RegVal);
        RegVal = NV_FLD_SET_DRF_NUM(APB_MISC, UTMIP_BIAS_CFG0,
                            UTMIP_BIASPD, 0, RegVal);
        NV_REGW(hDevice, NvRmModuleID_Misc, 0, APB_MISC_UTMIP_BIAS_CFG0_0, RegVal);

        if (!s_ReadFuseValue)
        { 
#if NV_USE_FUSE_CLOCK_ENABLE
    // Enable fuse clock
    NvRmPowerModuleClockControl(hDevice, NvRmModuleID_Fuse, 0, NV_TRUE);
#endif  
            // Enable fuse values to be visible before reading the fuses.
            RegVal = NV_REGR( hDevice, NvRmPrivModuleID_ClockAndReset, 0,
                CLK_RST_CONTROLLER_MISC_CLK_ENB_0 );
            RegVal = NV_FLD_SET_DRF_NUM( CLK_RST_CONTROLLER, MISC_CLK_ENB,
                CFG_ALL_VISIBLE, 1, RegVal );
            NV_REGW( hDevice, NvRmPrivModuleID_ClockAndReset, 0,
                CLK_RST_CONTROLLER_MISC_CLK_ENB_0, RegVal );

            // Read the spare register fuses and redundancy fuses for setting up USB
            // UTMIP_XCVR_SETUP value for proper EYE diagram.
            RegVal = NV_REGR( hDevice, NvRmModuleID_Fuse, 0, FUSE_FUSEDATA21_0);
    
            s_XcvrSetupValue  = (NV_DRF_VAL(FUSE, FUSEDATA21,FUSEDATA_SPARE_BIT_10__PRI_ALIAS_0, RegVal) |
                               NV_DRF_VAL(FUSE, FUSEDATA21, FUSEDATA_SPARE_BIT_13__PRI_ALIAS_0, RegVal)) << 0;
            s_XcvrSetupValue |= (NV_DRF_VAL(FUSE, FUSEDATA21, FUSEDATA_SPARE_BIT_11__PRI_ALIAS_0, RegVal) |
                               NV_DRF_VAL(FUSE, FUSEDATA21, FUSEDATA_SPARE_BIT_14__PRI_ALIAS_0, RegVal)) << 1;
            s_XcvrSetupValue |= (NV_DRF_VAL(FUSE, FUSEDATA21, FUSEDATA_SPARE_BIT_12__PRI_ALIAS_0, RegVal) |
                               NV_DRF_VAL(FUSE, FUSEDATA21, FUSEDATA_SPARE_BIT_15__PRI_ALIAS_0, RegVal)) << 2;
            // Only UTMIP_XCVR_SETUP[3-1] need to be programmed with the fuse vlaue
            // UTMIP_XCVR_SETUP[0] must be equal to 0
            s_XcvrSetupValue = s_XcvrSetupValue << 1;

            // Disable fuse values visibility, we already read the data
            RegVal = NV_REGR( hDevice, NvRmPrivModuleID_ClockAndReset, 0,
                CLK_RST_CONTROLLER_MISC_CLK_ENB_0 );
            RegVal = NV_FLD_SET_DRF_NUM( CLK_RST_CONTROLLER, MISC_CLK_ENB,
                CFG_ALL_VISIBLE, 0, RegVal );
            NV_REGW( hDevice, NvRmPrivModuleID_ClockAndReset, 0,
                CLK_RST_CONTROLLER_MISC_CLK_ENB_0, RegVal );
            s_ReadFuseValue = NV_TRUE;
#if NV_USE_FUSE_CLOCK_ENABLE
    // Disable fuse clock
    NvRmPowerModuleClockControl(hDevice, NvRmModuleID_Fuse, 0, NV_FALSE);
#endif
        }

        //NvOsDebugPrintf("s_XcvrSetupValue from fuse [0x%x] \n", s_XcvrSetupValue);

        // Turn on power in the tranciver
        RegVal = NV_REGR(hDevice, NvRmModuleID_Misc, 0, APB_MISC_UTMIP_XCVR_CFG0_0);
        RegVal = NV_FLD_SET_DRF_NUM(APB_MISC, UTMIP_XCVR_CFG0,
                            UTMIP_FORCE_PDZI_POWERDOWN, 0, RegVal);
        RegVal = NV_FLD_SET_DRF_NUM(APB_MISC, UTMIP_XCVR_CFG0,
                            UTMIP_FORCE_PD2_POWERDOWN, 0, RegVal);
        RegVal = NV_FLD_SET_DRF_NUM(APB_MISC, UTMIP_XCVR_CFG0,
                            UTMIP_FORCE_PD_POWERDOWN, 0, RegVal);
        RegVal = NV_FLD_SET_DRF_NUM(APB_MISC, UTMIP_XCVR_CFG0,
                            UTMIP_XCVR_SETUP, s_XcvrSetupValue, RegVal);
        NV_REGW(hDevice, NvRmModuleID_Misc, 0, APB_MISC_UTMIP_XCVR_CFG0_0, RegVal);

        // Enable Batery charge enabling bit, set to '0' for enable
        RegVal = NV_REGR(hDevice, NvRmModuleID_Misc, 0,
                            APB_MISC_UTMIP_BAT_CHRG_CFG0_0);
        RegVal = NV_FLD_SET_DRF_NUM(APB_MISC, UTMIP_BAT_CHRG_CFG0,
                            UTMIP_PD_CHRG, 0, RegVal);
        NV_REGW(hDevice, NvRmModuleID_Misc, 0,
                            APB_MISC_UTMIP_BAT_CHRG_CFG0_0, RegVal);

        if (hDevice->ChipId.Id == 0x16)
        {
            if(s_IsUSBResetRequired) 
            {
                NvOsMutexLock(hDevice->CarMutex);
                // Put the controller in  reset
                RegVal = NV_REGR(hDevice, NvRmPrivModuleID_ClockAndReset, 0,
                                                            CLK_RST_CONTROLLER_RST_DEVICES_L_0);
                RegVal = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, RST_DEVICES_L,
                                                                    SWR_USBD_RST, ENABLE, RegVal);
                NV_REGW(hDevice, NvRmPrivModuleID_ClockAndReset, 0,
                                                CLK_RST_CONTROLLER_RST_DEVICES_L_0, RegVal);
                NvOsMutexUnlock(hDevice->CarMutex);
                s_IsUSBResetRequired = NV_FALSE;
            }
        
            RegVal = NV_REGR(hDevice, NvRmModuleID_Misc, 0, 
                                                            APB_MISC_PP_MISC_USB_CLK_RST_CTL_0);
            RegVal = NV_FLD_SET_DRF_DEF(APB_MISC_PP, MISC_USB_CLK_RST_CTL,
                                                                    MISC_USB_CE, ENABLE, RegVal);
            RegVal = NV_FLD_SET_DRF_DEF(APB_MISC_PP, MISC_USB_CLK_RST_CTL,
                                                                    MISC_USB_RST, DISABLE, RegVal);
            NV_REGW(hDevice, NvRmModuleID_Misc, 0,
                                                APB_MISC_PP_MISC_USB_CLK_RST_CTL_0, RegVal);
        }
        NvOsMutexLock(hDevice->CarMutex);
        // Bring Out of reset
        RegVal = NV_REGR(hDevice, NvRmPrivModuleID_ClockAndReset, 0,
                            CLK_RST_CONTROLLER_RST_DEVICES_L_0);
        RegVal = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, RST_DEVICES_L,
                            SWR_USBD_RST, DISABLE, RegVal);
        NV_REGW(hDevice, NvRmPrivModuleID_ClockAndReset, 0,
                            CLK_RST_CONTROLLER_RST_DEVICES_L_0, RegVal);
        NvOsMutexUnlock(hDevice->CarMutex);

    }
    else
    {
        // USB Power down sequence
        // Power down OTG and Bias circuitry
        RegVal = NV_REGR(hDevice, NvRmModuleID_Misc, 0, APB_MISC_UTMIP_BIAS_CFG0_0);
        // Query if Internal Phy is going to wake up the usb controller upon cable insertion.
        pUsbProperty = NvOdmQueryGetUsbProperty(NvOdmIoModule_Usb, Instance);
        if (!pUsbProperty->UseInternalPhyWakeup)
        {
            /// If not internal Phy then Use PMU interrupt for VBUS detection.
            /// Disable the OTG bias circuitry.
            RegVal = NV_FLD_SET_DRF_NUM(APB_MISC, UTMIP_BIAS_CFG0,
                            UTMIP_OTGPD, 1, RegVal);
        }
        RegVal = NV_FLD_SET_DRF_NUM(APB_MISC, UTMIP_BIAS_CFG0,
                            UTMIP_BIASPD, 1, RegVal);
        NV_REGW(hDevice, NvRmModuleID_Misc, 0, APB_MISC_UTMIP_BIAS_CFG0_0, RegVal);

        // Disable Batery charge enabling bit set to '1' for disable
        RegVal = NV_REGR(hDevice, NvRmModuleID_Misc, 0,
                            APB_MISC_UTMIP_BAT_CHRG_CFG0_0);
        RegVal = NV_FLD_SET_DRF_NUM(APB_MISC, UTMIP_BAT_CHRG_CFG0,
                            UTMIP_PD_CHRG, 1, RegVal);
        NV_REGW(hDevice, NvRmModuleID_Misc, 0,
                            APB_MISC_UTMIP_BAT_CHRG_CFG0_0, RegVal);

        // Turn off power in the tranciver
        RegVal = NV_REGR(hDevice, NvRmModuleID_Misc, 0, APB_MISC_UTMIP_XCVR_CFG0_0);
        RegVal = NV_FLD_SET_DRF_NUM(APB_MISC, UTMIP_XCVR_CFG0,
                            UTMIP_FORCE_PDZI_POWERDOWN, 1, RegVal);
        RegVal = NV_FLD_SET_DRF_NUM(APB_MISC, UTMIP_XCVR_CFG0,
                            UTMIP_FORCE_PD2_POWERDOWN, 1, RegVal);
        RegVal = NV_FLD_SET_DRF_NUM(APB_MISC, UTMIP_XCVR_CFG0,
                            UTMIP_FORCE_PD_POWERDOWN, 1, RegVal);
        NV_REGW(hDevice, NvRmModuleID_Misc, 0, APB_MISC_UTMIP_XCVR_CFG0_0, RegVal);

        if (hDevice->ChipId.Id == 0x16)
        {
            RegVal = NV_REGR(hDevice, NvRmModuleID_Misc, 0,
                                                            APB_MISC_PP_MISC_USB_CLK_RST_CTL_0);
            RegVal = NV_FLD_SET_DRF_DEF(APB_MISC_PP, MISC_USB_CLK_RST_CTL,
                                                                    MISC_USB_RST, ENABLE, RegVal);
            RegVal = NV_FLD_SET_DRF_DEF(APB_MISC_PP, MISC_USB_CLK_RST_CTL,
                                                                    MISC_USB_CE, DISABLE, RegVal);
            NV_REGW(hDevice, NvRmModuleID_Misc, 0,
                                                APB_MISC_PP_MISC_USB_CLK_RST_CTL_0, RegVal);
            if (!(NV_DRF_VAL(APB_MISC_PP, MISC_USB_CLK_RST_CTL, MISC_USB2_CE, RegVal)))
            {
                // Enable reset
                NvOsMutexLock(hDevice->CarMutex);
                RegVal = NV_REGR(hDevice, NvRmPrivModuleID_ClockAndReset, 0,
                                                                CLK_RST_CONTROLLER_RST_DEVICES_L_0);
                RegVal = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, RST_DEVICES_L,
                                                                        SWR_USBD_RST, ENABLE, RegVal);
                NV_REGW(hDevice, NvRmPrivModuleID_ClockAndReset, 0,
                                                    CLK_RST_CONTROLLER_RST_DEVICES_L_0, RegVal);
                NvOsMutexUnlock(hDevice->CarMutex);
            }
        }
        else
        {
            // Enable reset
            NvOsMutexLock(hDevice->CarMutex);
            RegVal = NV_REGR(hDevice, NvRmPrivModuleID_ClockAndReset, 0,
                                                            CLK_RST_CONTROLLER_RST_DEVICES_L_0);
            RegVal = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, RST_DEVICES_L,
                                                                    SWR_USBD_RST, ENABLE, RegVal);
            NV_REGW(hDevice, NvRmPrivModuleID_ClockAndReset, 0,
                                                CLK_RST_CONTROLLER_RST_DEVICES_L_0, RegVal);
            NvOsMutexUnlock(hDevice->CarMutex);
        }
        // Wait till B Session end
        do {
            NV_REGW(hDevice, NvRmModuleID_Misc, 0,
                APB_MISC_PP_USB_PHY_VBUS_SENSORS_0,
                NV_DRF_DEF(APB_MISC_PP, USB_PHY_VBUS_SENSORS,
                                            A_SESS_VLD_CHG_DET, SET));
            RegVal = NV_REGR(hDevice, NvRmModuleID_Misc, 0,
                                APB_MISC_PP_USB_PHY_VBUS_SENSORS_0);
            if (NV_DRF_VAL(APB_MISC_PP, USB_PHY_VBUS_SENSORS, B_SESS_END_STS, RegVal))
            {
                // break here once the B Session end apears.
                break;
            }
            NvOsWaitUS(1);
            TimeOut--;
        } while (TimeOut);
    }
}

#if 0
static void
NvRmPrivUsbfDisableChargerInterrupt(
    NvRmDeviceHandle hDevice)
{
    NvU32 RegVal = 0;

    // disable the charger Interrupt
    RegVal = NV_REGR(hDevice, NvRmModuleID_Misc, 0, APB_MISC_PP_USB_PHY_VBUS_WAKEUP_ID_0);
    RegVal = NV_FLD_SET_DRF_DEF(APB_MISC, PP_USB_PHY_VBUS_WAKEUP_ID, VDAT_DET_INT_EN, DISABLE, RegVal);
    RegVal = NV_FLD_SET_DRF_DEF(APB_MISC, PP_USB_PHY_VBUS_WAKEUP_ID, VDAT_DET_CHG_DET, SET, RegVal);
    NV_REGW(hDevice, NvRmModuleID_Misc, 0, APB_MISC_PP_USB_PHY_VBUS_WAKEUP_ID_0, RegVal);
}
#endif

static void NvRmUsbPrivConfigureUsbPhy( 
    NvRmDeviceHandle hDevice,
    NvBool Enable)
 {
    NvU32 RegVal;
    NvU32 TimeOut = 1000;
    NvU32 PhyClkValid = 0;
    NvU32 UlpiRunBit = 1;
    //NvU32   ReadValue = 0;
    //NvU32 i;

    if(Enable)
    {

        if(s_IsUSBResetRequired) 
        {
            // If USB1 is active by this time by KITL, do not do a car reset
            RegVal = NV_REGR(hDevice, NvRmModuleID_Misc, 0,
                                                            APB_MISC_PP_MISC_USB_CLK_RST_CTL_0);
            if (!(NV_DRF_VAL(APB_MISC_PP, MISC_USB_CLK_RST_CTL, MISC_USB_CE, RegVal)))
            {
                NvOsMutexLock(hDevice->CarMutex);
                // Put the controller in  reset
                RegVal = NV_REGR(hDevice, NvRmPrivModuleID_ClockAndReset, 0,
                                                                CLK_RST_CONTROLLER_RST_DEVICES_L_0);
                RegVal = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, RST_DEVICES_L,
                                                                        SWR_USBD_RST, ENABLE, RegVal);
                NV_REGW(hDevice, NvRmPrivModuleID_ClockAndReset, 0,
                                                    CLK_RST_CONTROLLER_RST_DEVICES_L_0, RegVal);
                NvOsMutexUnlock(hDevice->CarMutex);
            }
            s_IsUSBResetRequired = NV_FALSE;
        }
        // Bring controller out of reset
        RegVal = NV_REGR(hDevice, NvRmModuleID_Misc, 0,
                                                        APB_MISC_PP_MISC_USB_CLK_RST_CTL_0);
        RegVal = NV_FLD_SET_DRF_DEF(APB_MISC_PP, MISC_USB_CLK_RST_CTL,
                                                                MISC_USB2_RST, DISABLE, RegVal);
         RegVal = NV_FLD_SET_DRF_DEF(APB_MISC_PP, MISC_USB_CLK_RST_CTL,
                                                                MISC_USB2_CE, ENABLE, RegVal);
        NV_REGW(hDevice, NvRmModuleID_Misc, 0,
                                            APB_MISC_PP_MISC_USB_CLK_RST_CTL_0, RegVal);

        NvOsMutexLock(hDevice->CarMutex);
        // Bring Out of reset
        RegVal = NV_REGR(hDevice, NvRmPrivModuleID_ClockAndReset, 0,
                                                        CLK_RST_CONTROLLER_RST_DEVICES_L_0);
        RegVal = NV_FLD_SET_DRF_DEF(CLK_RST_CONTROLLER, RST_DEVICES_L,
                                                                SWR_USBD_RST, DISABLE, RegVal);
        NV_REGW(hDevice, NvRmPrivModuleID_ClockAndReset, 0,
                                            CLK_RST_CONTROLLER_RST_DEVICES_L_0, RegVal);
        NvOsMutexUnlock(hDevice->CarMutex);
        // Wake-up ULPI PHY generate a postive pulse
        //set
        RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 1,
                                USB2_IF_USB_SUSP_CTRL_0);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_IF, USB_SUSP_CTRL, USB_SUSP_CLR, SET,
                                RegVal);
        NV_REGW(hDevice, NvRmModuleID_Usb2Otg, 1,
                                USB2_IF_USB_SUSP_CTRL_0, RegVal);

        // wait 10 us
        NvOsWaitUS(100);

        // clear
        RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 1,
                                USB2_IF_USB_SUSP_CTRL_0);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_IF, USB_SUSP_CTRL,
                                USB_SUSP_CLR, UNSET, RegVal);
        NV_REGW(hDevice, NvRmModuleID_Usb2Otg, 1,
                                USB2_IF_USB_SUSP_CTRL_0, RegVal);

       //  Set the MISC_USB2_CLK_OVR_ON bit and update PP_MISC_USB_CLK_RST_CTL register.
        RegVal = NV_REGR(hDevice, NvRmModuleID_Misc, 0,
                     APB_MISC_PP_MISC_USB_CLK_RST_CTL_0);
        RegVal = NV_FLD_SET_DRF_DEF(APB_MISC_PP, MISC_USB_CLK_RST_CTL, MISC_USB2_CLK_OVR_ON,
                                ENABLE, RegVal);
        NV_REGW(hDevice, NvRmModuleID_Misc, 0, 
            APB_MISC_PP_MISC_USB_CLK_RST_CTL_0, RegVal);


        // Setting the ULPI register IndicatorPassThru to 1
        RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 1,
                            USB2_CONTROLLER_1_USB2D_ULPI_VIEWPORT_0);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_WAKEUP, CLEAR, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_RUN, SET, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_RD_WR, WRITE, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_PORT, SW_DEFAULT, RegVal);
        RegVal = NV_FLD_SET_DRF_NUM(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_REG_ADDR, 0x8, RegVal);
        RegVal = NV_FLD_SET_DRF_NUM(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_DATA_WR, 0x40, RegVal);
        NV_REGW(hDevice, NvRmModuleID_Usb2Otg, 1, 
                            USB2_CONTROLLER_1_USB2D_ULPI_VIEWPORT_0, RegVal);

        UlpiRunBit = 1;
        do
        {
            // check for run bit being cleared..
            RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 1,
                                USB2_CONTROLLER_1_USB2D_ULPI_VIEWPORT_0);

            UlpiRunBit = NV_DRF_VAL(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT, ULPI_RUN, RegVal);
        } while (UlpiRunBit);

        // Setting ULPI  register UseExternalVbusIndicator  to 1. 
        RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 1,
                            USB2_CONTROLLER_1_USB2D_ULPI_VIEWPORT_0);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_WAKEUP, CLEAR, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_RUN, SET, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_RD_WR, WRITE, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_PORT, SW_DEFAULT, RegVal);
        RegVal = NV_FLD_SET_DRF_NUM(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_REG_ADDR, 0xB, RegVal);
        RegVal = NV_FLD_SET_DRF_NUM(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_DATA_WR, 0x80, RegVal);
        NV_REGW(hDevice, NvRmModuleID_Usb2Otg, 1, 
                            USB2_CONTROLLER_1_USB2D_ULPI_VIEWPORT_0, RegVal);

        UlpiRunBit = 1;
        do
        {
            // check for run bit being cleared..
            RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 1,
                                USB2_CONTROLLER_1_USB2D_ULPI_VIEWPORT_0);

            UlpiRunBit = NV_DRF_VAL(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT, ULPI_RUN, RegVal);
        } while (UlpiRunBit);

    }
    else
    {
        // Programming  the ULPI register functuion control
        RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 1,
                            USB2_CONTROLLER_1_USB2D_ULPI_VIEWPORT_0);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_WAKEUP, CLEAR, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_RUN, SET, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_RD_WR, WRITE, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_PORT, SW_DEFAULT, RegVal);
        RegVal = NV_FLD_SET_DRF_NUM(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_REG_ADDR, 0x4, RegVal);
        RegVal = NV_FLD_SET_DRF_NUM(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_DATA_WR, 0x4d, RegVal);
        NV_REGW(hDevice, NvRmModuleID_Usb2Otg, 1, 
                            USB2_CONTROLLER_1_USB2D_ULPI_VIEWPORT_0, RegVal);

        UlpiRunBit = 1;
        do
        {
            // check for run bit being cleared..
            RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 1,
                                USB2_CONTROLLER_1_USB2D_ULPI_VIEWPORT_0);

            UlpiRunBit = NV_DRF_VAL(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT, ULPI_RUN, RegVal);
        } while (UlpiRunBit);
         // Resetting  the ULPI register IndicatorPassThru
        RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 1,
                            USB2_CONTROLLER_1_USB2D_ULPI_VIEWPORT_0);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_WAKEUP, CLEAR, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_RUN, SET, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_RD_WR, WRITE, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_PORT, SW_DEFAULT, RegVal);
        RegVal = NV_FLD_SET_DRF_NUM(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_REG_ADDR, 0x7, RegVal);
        RegVal = NV_FLD_SET_DRF_NUM(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_DATA_WR, 0x0, RegVal);
        NV_REGW(hDevice, NvRmModuleID_Usb2Otg, 1, 
                            USB2_CONTROLLER_1_USB2D_ULPI_VIEWPORT_0, RegVal);

        UlpiRunBit = 1;
        do
        {
            // check for run bit being cleared..
            RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 1,
                                USB2_CONTROLLER_1_USB2D_ULPI_VIEWPORT_0);

            UlpiRunBit = NV_DRF_VAL(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT, ULPI_RUN, RegVal);
        } while (UlpiRunBit);

        // Resetting ULPI register UseExternalVbusIndicator
        RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 1,
                            USB2_CONTROLLER_1_USB2D_ULPI_VIEWPORT_0);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_WAKEUP, CLEAR, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_RUN, SET, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_RD_WR, WRITE, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_PORT, SW_DEFAULT, RegVal);
        RegVal = NV_FLD_SET_DRF_NUM(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_REG_ADDR, 0xa, RegVal);
        RegVal = NV_FLD_SET_DRF_NUM(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_DATA_WR, 0x86, RegVal);
        NV_REGW(hDevice, NvRmModuleID_Usb2Otg, 1, 
                            USB2_CONTROLLER_1_USB2D_ULPI_VIEWPORT_0, RegVal);

        UlpiRunBit = 1;
        do
        {
            // check for run bit being cleared..
            RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 1,
                                USB2_CONTROLLER_1_USB2D_ULPI_VIEWPORT_0);

            UlpiRunBit = NV_DRF_VAL(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT, ULPI_RUN, RegVal);
        } while (UlpiRunBit);

        // making sure vbus comparator and id are off
        // USB Interrupt Rising
        RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 1,
                            USB2_CONTROLLER_1_USB2D_ULPI_VIEWPORT_0);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_WAKEUP, CLEAR, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_RUN, SET, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_RD_WR, WRITE, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_PORT, SW_DEFAULT, RegVal);
        RegVal = NV_FLD_SET_DRF_NUM(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_REG_ADDR, 0x0d, RegVal);
        RegVal = NV_FLD_SET_DRF_NUM(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_DATA_WR, 0x00, RegVal);
        NV_REGW(hDevice, NvRmModuleID_Usb2Otg, 1, 
                            USB2_CONTROLLER_1_USB2D_ULPI_VIEWPORT_0, RegVal);

        UlpiRunBit = 1;
        do
        {
            // check for run bit being cleared..
            RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 1,
                                USB2_CONTROLLER_1_USB2D_ULPI_VIEWPORT_0);

            UlpiRunBit = NV_DRF_VAL(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT, ULPI_RUN, RegVal);
        } while (UlpiRunBit);

        // USB Interrupt Falling
        RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 1,
                            USB2_CONTROLLER_1_USB2D_ULPI_VIEWPORT_0);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_WAKEUP, CLEAR, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_RUN, SET, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_RD_WR, WRITE, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_PORT, SW_DEFAULT, RegVal);
        RegVal = NV_FLD_SET_DRF_NUM(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_REG_ADDR, 0x10, RegVal);
        RegVal = NV_FLD_SET_DRF_NUM(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_DATA_WR, 0x00, RegVal);
        NV_REGW(hDevice, NvRmModuleID_Usb2Otg, 1, 
                            USB2_CONTROLLER_1_USB2D_ULPI_VIEWPORT_0, RegVal);

        UlpiRunBit = 1;
        do
        {
            // check for run bit being cleared..
            RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 1,
                                USB2_CONTROLLER_1_USB2D_ULPI_VIEWPORT_0);

            UlpiRunBit = NV_DRF_VAL(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT, ULPI_RUN, RegVal);
        } while (UlpiRunBit);


        RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 1,
                            USB2_CONTROLLER_1_USB2D_ULPI_VIEWPORT_0);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_WAKEUP, CLEAR, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_RUN, SET, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_RD_WR, WRITE, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_PORT, SW_DEFAULT, RegVal);
        RegVal = NV_FLD_SET_DRF_NUM(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_REG_ADDR, 0x19, RegVal);
        RegVal = NV_FLD_SET_DRF_NUM(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_DATA_WR, 0x00, RegVal);
        NV_REGW(hDevice, NvRmModuleID_Usb2Otg, 1, 
                            USB2_CONTROLLER_1_USB2D_ULPI_VIEWPORT_0, RegVal);

        UlpiRunBit = 1;
        do
        {
            // check for run bit being cleared..
            RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 1,
                                USB2_CONTROLLER_1_USB2D_ULPI_VIEWPORT_0);

            UlpiRunBit = NV_DRF_VAL(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT, ULPI_RUN, RegVal);
        } while (UlpiRunBit);

        // Disabling ID float Rise/Fall (Carkit Enable)
        RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 1,
                            USB2_CONTROLLER_1_USB2D_ULPI_VIEWPORT_0);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_WAKEUP, CLEAR, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_RUN, SET, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_RD_WR, WRITE, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_PORT, SW_DEFAULT, RegVal);
        RegVal = NV_FLD_SET_DRF_NUM(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_REG_ADDR, 0x1D, RegVal);
        RegVal = NV_FLD_SET_DRF_NUM(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_DATA_WR, 0x00, RegVal);
        NV_REGW(hDevice, NvRmModuleID_Usb2Otg, 1, 
                            USB2_CONTROLLER_1_USB2D_ULPI_VIEWPORT_0, RegVal);

        UlpiRunBit = 1;
        do
        {
            // check for run bit being cleared..
            RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 1,
                                USB2_CONTROLLER_1_USB2D_ULPI_VIEWPORT_0);

            UlpiRunBit = NV_DRF_VAL(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT, ULPI_RUN, RegVal);
        } while (UlpiRunBit);

        RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 1,
                            USB2_CONTROLLER_1_USB2D_ULPI_VIEWPORT_0);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_WAKEUP, CLEAR, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_RUN, SET, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_RD_WR, WRITE, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_PORT, SW_DEFAULT, RegVal);
        RegVal = NV_FLD_SET_DRF_NUM(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_REG_ADDR, 0x39, RegVal);
        RegVal = NV_FLD_SET_DRF_NUM(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_DATA_WR, 0x00, RegVal);
        NV_REGW(hDevice, NvRmModuleID_Usb2Otg, 1, 
                            USB2_CONTROLLER_1_USB2D_ULPI_VIEWPORT_0, RegVal);

        UlpiRunBit = 1;
        do
        {
            // check for run bit being cleared..
            RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 1,
                                USB2_CONTROLLER_1_USB2D_ULPI_VIEWPORT_0);

            UlpiRunBit = NV_DRF_VAL(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT, ULPI_RUN, RegVal);
        } while (UlpiRunBit);
#if 0
        // STARTING register
        // taking the register dump for all ULPI 3317 register
        // staring from FCR
        //Read FCR
        for (i = 0x4;i < 0x14;i+=0x3)
        {
            RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 1,
                                USB2_CONTROLLER_1_USB2D_ULPI_VIEWPORT_0);
            RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                    ULPI_WAKEUP, CLEAR, RegVal);
            RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                    ULPI_RUN, SET, RegVal);
            RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                    ULPI_RD_WR, READ, RegVal);
            RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                    ULPI_PORT, SW_DEFAULT, RegVal);
            RegVal = NV_FLD_SET_DRF_NUM(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                    ULPI_REG_ADDR, i, RegVal);
            NV_REGW(hDevice, NvRmModuleID_Usb2Otg, 1, 
                                USB2_CONTROLLER_1_USB2D_ULPI_VIEWPORT_0, RegVal);

            UlpiRunBit = 1;
            do
            {
                // check for run bit being cleared..
                RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 1,
                                    USB2_CONTROLLER_1_USB2D_ULPI_VIEWPORT_0);

                UlpiRunBit = NV_DRF_VAL(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT, ULPI_RUN, RegVal);
            } while (UlpiRunBit);

            ReadValue = NV_DRF_VAL(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT, ULPI_DATA_RD, RegVal);

            NvOsDebugPrintf("USB ULPI 3317 reg @ 0x%x value %x",i,ReadValue);
        }

        // USB IL
        i = 0x14;
        RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 1,
                            USB2_CONTROLLER_1_USB2D_ULPI_VIEWPORT_0);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_WAKEUP, CLEAR, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_RUN, SET, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_RD_WR, READ, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_PORT, SW_DEFAULT, RegVal);
        RegVal = NV_FLD_SET_DRF_NUM(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_REG_ADDR, i, RegVal);
        NV_REGW(hDevice, NvRmModuleID_Usb2Otg, 1, 
                            USB2_CONTROLLER_1_USB2D_ULPI_VIEWPORT_0, RegVal);

        UlpiRunBit = 1;
        do
        {
            // check for run bit being cleared..
            RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 1,
                                USB2_CONTROLLER_1_USB2D_ULPI_VIEWPORT_0);

            UlpiRunBit = NV_DRF_VAL(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT, ULPI_RUN, RegVal);
        } while (UlpiRunBit);

        ReadValue = NV_DRF_VAL(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT, ULPI_DATA_RD, RegVal);

        NvOsDebugPrintf("USB ULPI 3317 (USB IL)i @ 0x%x value %x",i,ReadValue);


        // USB CARKit
        i = 0x19;
        RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 1,
                            USB2_CONTROLLER_1_USB2D_ULPI_VIEWPORT_0);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_WAKEUP, CLEAR, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_RUN, SET, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_RD_WR, READ, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_PORT, SW_DEFAULT, RegVal);
        RegVal = NV_FLD_SET_DRF_NUM(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_REG_ADDR, i, RegVal);
        NV_REGW(hDevice, NvRmModuleID_Usb2Otg, 1, 
                            USB2_CONTROLLER_1_USB2D_ULPI_VIEWPORT_0, RegVal);

        UlpiRunBit = 1;
        do
        {
            // check for run bit being cleared..
            RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 1,
                                USB2_CONTROLLER_1_USB2D_ULPI_VIEWPORT_0);

            UlpiRunBit = NV_DRF_VAL(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT, ULPI_RUN, RegVal);
        } while (UlpiRunBit);

        ReadValue = NV_DRF_VAL(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT, ULPI_DATA_RD, RegVal);

        NvOsDebugPrintf("USB ULPI 3317 (USB CARKit)i @ 0x%x value %x",i,ReadValue);

        // USB CARKit IE
        i = 0x1D;
        RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 1,
                            USB2_CONTROLLER_1_USB2D_ULPI_VIEWPORT_0);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_WAKEUP, CLEAR, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_RUN, SET, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_RD_WR, READ, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_PORT, SW_DEFAULT, RegVal);
        RegVal = NV_FLD_SET_DRF_NUM(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_REG_ADDR, i, RegVal);
        NV_REGW(hDevice, NvRmModuleID_Usb2Otg, 1, 
                            USB2_CONTROLLER_1_USB2D_ULPI_VIEWPORT_0, RegVal);

        UlpiRunBit = 1;
        do
        {
            // check for run bit being cleared..
            RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 1,
                                USB2_CONTROLLER_1_USB2D_ULPI_VIEWPORT_0);

            UlpiRunBit = NV_DRF_VAL(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT, ULPI_RUN, RegVal);
        } while (UlpiRunBit);

        ReadValue = NV_DRF_VAL(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT, ULPI_DATA_RD, RegVal);

        NvOsDebugPrintf("USB ULPI 3317 (USB CARKit IE)i @ 0x%x value %x",i,ReadValue);

        // USB CARKit IS
        i = 0x20;
        RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 1,
                            USB2_CONTROLLER_1_USB2D_ULPI_VIEWPORT_0);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_WAKEUP, CLEAR, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_RUN, SET, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_RD_WR, READ, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_PORT, SW_DEFAULT, RegVal);
        RegVal = NV_FLD_SET_DRF_NUM(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_REG_ADDR, i, RegVal);
        NV_REGW(hDevice, NvRmModuleID_Usb2Otg, 1, 
                            USB2_CONTROLLER_1_USB2D_ULPI_VIEWPORT_0, RegVal);

        UlpiRunBit = 1;
        do
        {
            // check for run bit being cleared..
            RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 1,
                                USB2_CONTROLLER_1_USB2D_ULPI_VIEWPORT_0);

            UlpiRunBit = NV_DRF_VAL(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT, ULPI_RUN, RegVal);
        } while (UlpiRunBit);

        ReadValue = NV_DRF_VAL(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT, ULPI_DATA_RD, RegVal);

        NvOsDebugPrintf("USB ULPI 3317 (USB CARKit IS)i @ 0x%x value %x",i,ReadValue);

        // USB CARKit IL
        i = 0x21;
        RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 1,
                            USB2_CONTROLLER_1_USB2D_ULPI_VIEWPORT_0);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_WAKEUP, CLEAR, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_RUN, SET, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_RD_WR, READ, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_PORT, SW_DEFAULT, RegVal);
        RegVal = NV_FLD_SET_DRF_NUM(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_REG_ADDR, i, RegVal);
        NV_REGW(hDevice, NvRmModuleID_Usb2Otg, 1, 
                            USB2_CONTROLLER_1_USB2D_ULPI_VIEWPORT_0, RegVal);

        UlpiRunBit = 1;
        do
        {
            // check for run bit being cleared..
            RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 1,
                                USB2_CONTROLLER_1_USB2D_ULPI_VIEWPORT_0);

            UlpiRunBit = NV_DRF_VAL(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT, ULPI_RUN, RegVal);
        } while (UlpiRunBit);

        ReadValue = NV_DRF_VAL(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT, ULPI_DATA_RD, RegVal);

        NvOsDebugPrintf("USB ULPI 3317 (USB CARKit IL)i @ 0x%x value %x ",i,ReadValue);


        // USB I/0
        i = 0x39;
        RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 1,
                            USB2_CONTROLLER_1_USB2D_ULPI_VIEWPORT_0);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_WAKEUP, CLEAR, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_RUN, SET, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_RD_WR, READ, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_PORT, SW_DEFAULT, RegVal);
        RegVal = NV_FLD_SET_DRF_NUM(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT,
                                ULPI_REG_ADDR, i, RegVal);
        NV_REGW(hDevice, NvRmModuleID_Usb2Otg, 1, 
                            USB2_CONTROLLER_1_USB2D_ULPI_VIEWPORT_0, RegVal);

        UlpiRunBit = 1;
        do
        {
            // check for run bit being cleared..
            RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 1,
                                USB2_CONTROLLER_1_USB2D_ULPI_VIEWPORT_0);

            UlpiRunBit = NV_DRF_VAL(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT, ULPI_RUN, RegVal);
        } while (UlpiRunBit);

        ReadValue = NV_DRF_VAL(USB2_CONTROLLER_1, USB2D_ULPI_VIEWPORT, ULPI_DATA_RD, RegVal);

        NvOsDebugPrintf("USB ULPI 3317 (USB I/O)i @ 0x%x value %x",i,ReadValue);

        //// done with the register dump..
        // ENDING register dump programming..
#endif

        // clear WKCN/WKDS/WKOC wake-on events that can cause the USB Controller to 
        // immediately bring the ULPI PHY out of low power mode after setting PHCD 
        RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 1, 
                            USB2_CONTROLLER_1_USB2D_PORTSC1_0);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_USB2D, PORTSC1, WKCN,
                            DISBLE, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_USB2D, PORTSC1, WKDS,
                            DISBLE, RegVal);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_USB2D, PORTSC1, WKOC,
                            DISBLE, RegVal);
        NV_REGW(hDevice, NvRmModuleID_Usb2Otg, 1, 
                            USB2_CONTROLLER_1_USB2D_PORTSC1_0, RegVal);

        // before disabling clock.. put the phy to low power mode..
        // enter low power suspend mode
        RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 1, 
                            USB2_CONTROLLER_1_USB2D_PORTSC1_0);
        RegVal = NV_FLD_SET_DRF_DEF(USB2_CONTROLLER_1, USB2D_PORTSC1, PHCD,
                            ENABLE, RegVal);
        NV_REGW(hDevice, NvRmModuleID_Usb2Otg, 1, 
                            USB2_CONTROLLER_1_USB2D_PORTSC1_0, RegVal);

        // check for the phy in suspend..
        do
        {
            //Wait for the phy clock to stop or invalid
            RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 1 ,
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


        //NvRmAnalogUsbInputParam_ConfigureUsbPhy
        RegVal = NV_REGR(hDevice, NvRmModuleID_Misc, 0,
                                                        APB_MISC_PP_MISC_USB_CLK_RST_CTL_0);
         RegVal = NV_FLD_SET_DRF_DEF(APB_MISC_PP, MISC_USB_CLK_RST_CTL,
                                                                MISC_USB2_CE, DISABLE, RegVal);
        NV_REGW(hDevice, NvRmModuleID_Misc, 0,
                                            APB_MISC_PP_MISC_USB_CLK_RST_CTL_0, RegVal);

        s_IsUSBResetRequired = NV_TRUE;
    }
}


static void
NvRmPrivUsbfEnableIdInterrupt(
    NvRmDeviceHandle hDevice)
{
    NvU32 RegVal = 0;

    //enable ID interrupt for A cable detection 
    RegVal = NV_REGR(hDevice, NvRmModuleID_Misc, 0,
                                                    APB_MISC_PP_USB_PHY_VBUS_WAKEUP_ID_0);
    RegVal = NV_FLD_SET_DRF_DEF(APB_MISC_PP, USB_PHY_VBUS_WAKEUP_ID,
                                                            ID_INT_EN, ENABLE, RegVal);
    NV_REGW(hDevice, NvRmModuleID_Misc, 0,
                                        APB_MISC_PP_USB_PHY_VBUS_WAKEUP_ID_0, RegVal);
}

static void
NvRmPrivUsbfDisableIdInterrupt(
    NvRmDeviceHandle hDevice)
{
    NvU32 RegVal = 0;

    //disable the ID interrupt, 
    RegVal = NV_REGR(hDevice, NvRmModuleID_Misc, 0,
                                                    APB_MISC_PP_USB_PHY_VBUS_WAKEUP_ID_0);
    RegVal = NV_FLD_SET_DRF_DEF(APB_MISC_PP, USB_PHY_VBUS_WAKEUP_ID,
                                                            ID_INT_EN, DISABLE, RegVal);
    RegVal = NV_FLD_SET_DRF_DEF(APB_MISC_PP, USB_PHY_VBUS_WAKEUP_ID,
                                                            ID_CHG_DET, SET, RegVal);
    NV_REGW(hDevice, NvRmModuleID_Misc, 0,
                                        APB_MISC_PP_USB_PHY_VBUS_WAKEUP_ID_0, RegVal);
}

static NvBool
NvRmPrivUsbfIsIdSetToLow(
    NvRmDeviceHandle hDevice)
{
    NvU32 RegVal = 0;
    NvBool IdSetToLow = NV_FALSE;

    // Check for A cable connection
    RegVal = NV_REGR(hDevice, NvRmModuleID_Misc, 0, 
                                                    APB_MISC_PP_USB_PHY_VBUS_WAKEUP_ID_0);
    if (!NV_DRF_VAL(APB_MISC_PP, USB_PHY_VBUS_WAKEUP_ID, ID_STS, RegVal))
    {
        IdSetToLow = NV_TRUE;
    }

    //Ack the ID interrupt, 
    RegVal = NV_FLD_SET_DRF_DEF(APB_MISC_PP, USB_PHY_VBUS_WAKEUP_ID,
                                                            ID_CHG_DET, SET, RegVal);
    NV_REGW(hDevice, NvRmModuleID_Misc, 0,
                                        APB_MISC_PP_USB_PHY_VBUS_WAKEUP_ID_0, RegVal);

    return IdSetToLow;
}
static void NvRmUsbPrivEnableUsb2Clock( NvRmDeviceHandle hDevice )
{
    NvU32 RegVal;

    // fist enable clocks to USB2
    RegVal = NV_REGR(hDevice, NvRmModuleID_Misc, 0,
                                                    APB_MISC_PP_MISC_USB_CLK_RST_CTL_0);
    RegVal = NV_FLD_SET_DRF_DEF(APB_MISC_PP, MISC_USB_CLK_RST_CTL,
                                                            MISC_USB2_CE, ENABLE, RegVal);
    RegVal = NV_FLD_SET_DRF_DEF(APB_MISC_PP, MISC_USB_CLK_RST_CTL,
                                                            MISC_USB_CE, ENABLE, RegVal);
    NV_REGW(hDevice, NvRmModuleID_Misc, 0,
                                        APB_MISC_PP_MISC_USB_CLK_RST_CTL_0, RegVal);

    // wait 10 us
    NvOsWaitUS(10);

    RegVal = NV_REGR(hDevice, NvRmModuleID_Misc, 0,
                                                    APB_MISC_PP_MISC_USB_CLK_RST_CTL_0);
    RegVal = NV_FLD_SET_DRF_DEF(APB_MISC_PP, MISC_USB_CLK_RST_CTL,
                                                            MISC_USB2_RST, DISABLE, RegVal);
    NV_REGW(hDevice, NvRmModuleID_Misc, 0,
                                        APB_MISC_PP_MISC_USB_CLK_RST_CTL_0, RegVal);

    // wait 10 us
    NvOsWaitUS(10);

    RegVal = NV_REGR(hDevice, NvRmModuleID_Misc, 0,
                                                    APB_MISC_PP_MISC_USB_CLK_RST_CTL_0);
    RegVal = NV_FLD_SET_DRF_DEF(APB_MISC_PP, MISC_USB_CLK_RST_CTL,
                                                            MISC_USB2_RST, ENABLE, RegVal);
    NV_REGW(hDevice, NvRmModuleID_Misc, 0,
                                        APB_MISC_PP_MISC_USB_CLK_RST_CTL_0, RegVal);

    // wait 10 us
    NvOsWaitUS(10);
}

static NvError
NvRmPrivUsbfControl(
    NvRmDeviceHandle hDevice,
    NvBool Enable,
    NvU32 inst,
    void *Config,
    NvU32 ConfigLength)
{
    NvError error = NvSuccess;
    NvRmAnalogUsbConfig *pUsbConf = NULL;
    static NvBool s_IsUsb0PhyConfigured = NV_FALSE;
    static NvBool s_IsUsb2PhyConfigured = NV_FALSE;
    NvU32 TimeOut = 100000;   // 100 milli seconds timeout before H/W gives up;
    NvU32 PhyClkValid = 0;
    NvU32 RegVal = 0;


    const NvOdmUsbProperty *pUsbProperty = NULL;

    NV_ASSERT(Config);

    pUsbConf = (NvRmAnalogUsbConfig *)Config;

    switch (pUsbConf->InParam)
    {
    case NvRmAnalogUsbInputParam_CheckCableStatus:
        if (inst == 0)
        {
            pUsbConf->UsbCableDetected = NvRmPrivUsbfIsCableConnected(hDevice);
            // At this time we don't know charger is connected or not
            pUsbConf->UsbChargerDetected = NV_FALSE;
        }
        break;
    case NvRmAnalogUsbInputParam_WaitForPhyClock:
        if (inst == 0)
        {
            // Wait for PHY clock to settle
            error = NvRmPrivUsbfWaitForPhyClock(hDevice, Enable);
        }
        if( inst == 2)
        {
            // wait for USB3 phy clock is settle 
            /**Wait until PHY clock comes up by checking for USB_PHY_CLK_VALID bit in 
            USB3_IF_USB_SUSP_CTRL register  **/

            do {
                //Wait for the phy clock to become valid
                RegVal = NV_REGR(hDevice, NvRmModuleID_Usb2Otg, 2, USB3_IF_USB_SUSP_CTRL_0);

                PhyClkValid = NV_DRF_VAL(USB3_IF, USB_SUSP_CTRL, USB_PHY_CLK_VALID, RegVal);

                if (!TimeOut)
                {
                    break;
                }
                NvOsWaitUS(1);
                TimeOut--;
            } while (!PhyClkValid);
        }
        
        break;
    case NvRmAnalogUsbInputParam_CheckChargerStatus:
        if (inst == 0)
        {
            // Check whether the Dumb charger is detected
            pUsbConf->UsbChargerDetected = NvRmPrivUsbfIsChargerDetected(hDevice, Enable);
        }
        break;
    case NvRmAnalogUsbInputParam_ChargerDetection:
        if (inst == 0)
        {
            // Enable Charger detection logic
            NvRmPrivUsbfChargerDetection(hDevice, Enable);
        }
        break;
    case NvRmAnalogUsbInputParam_ConfigureUsbPhy:
        if (inst == 1)  // for ULPI
        {
            NvRmUsbPrivConfigureUsbPhy(hDevice, Enable);
        }
        else
        {
            // UTMIP settings
            if (Enable)
            {
                if ((inst == 0) && (s_IsUsb0PhyConfigured == NV_TRUE))
                {
                    // If Usb0 Phy is already configure; nothing to do
                    return NvSuccess;
                }
                else if((inst == 2) && (s_IsUsb2PhyConfigured == NV_TRUE))
                {
                    // If Usb2 Phy is already configure; nothing to do
                    return NvSuccess;
                }
                /** For AP20 instance 2 is utmip and we need to select this 
                interface  **/
                pUsbProperty = NvOdmQueryGetUsbProperty(NvOdmIoModule_Usb, inst);
                if ((inst == 2) && (pUsbProperty->UsbInterfaceType == 
                                NvOdmUsbInterfaceType_Utmi))
                {
                    // Select UTMIP3 incase of usb3 and UTMIP
                    error =  NvRmPrivUsb3ConfigureUtmipPhy(hDevice);
                    if (error != NvSuccess)
                    {
                        return error;
                    }
                    s_IsUsb2PhyConfigured = NV_TRUE;
                }
                else
                {
                    // Configure  USB1 UTMIP Phy
                    error = NvRmPrivUsbfConfigureUtmipPhy(hDevice);
                    if (error != NvSuccess)
                    {
                        return error;
                    }
                    // Enable USB circuitry
                    NvRmPrivUsbfPowerControl(hDevice, inst, Enable);
                    s_IsUsb0PhyConfigured = NV_TRUE;
                }
            }
            else
            {
                if (inst == 0)  // for UTMIP1
                {
                    // Disable power to the USB phy
                    NvRmPrivUsbfPowerControl(hDevice, inst, Enable);
                    // Enable VBUS interrupt when USB controller is OFF
                    NvRmPrivUsbfEnableVbusInterrupt(hDevice);
                }
                if (inst == 0)
                    s_IsUsb0PhyConfigured = NV_FALSE;
                else if (inst == 2)
                    s_IsUsb2PhyConfigured = NV_FALSE;
            }
        }
        break;
    case NvRmAnalogUsbInputParam_SetUlpiNullTrimmers:
        if (inst == 1)
        {
            NvRmPrivUsbfSetUlpiNullTrimmers(hDevice, inst);
        }
        break;
    case NvRmAnalogUsbInputParam_SetUlpiLinkTrimmers:
        if (inst == 1)
        {
            NvRmPrivUsbfSetUlpiLinkTrimmers(hDevice, inst);
        }
        break;
    case NvRmAnalogUsbInputParam_VbusInterrupt:
        if (inst == 0)
        {
            if (Enable)
            {
                // enable VBus Interrupt
                NvRmPrivUsbfEnableVbusInterrupt(hDevice);
            }
            else
            {
                // disable VBUS interrupt
                NvRmPrivUsbfDisableVbusInterrupt(hDevice);
            }
        }
        break;
    case NvRmAnalogUsbInputParam_ConfigureUlpiNullClock:
        break;
    case NvRmAnalogUsbInputParam_SetNullUlpiPinMux:
        if(inst == 1)
        {
             NvRmSetModuleTristate(hDevice,
                                                    NVRM_MODULE_ID(NvRmModuleID_Usb2Otg, inst),
                                                    NV_FALSE);

            // enb_usb2_clocks
            NvRmUsbPrivEnableUsb2Clock(hDevice);
            NvRmPrivUsbfUlpiClockControl(hDevice, inst, Enable);
        }
        break;
    case NvRmAnalogUsbInputParam_IdInterrupt:
        if (inst == 0)
        {
            if (Enable)
            {
                // enable ID Interrupt
                NvRmPrivUsbfEnableIdInterrupt(hDevice);
            }
            else
            {
                // disable ID interrupt
                NvRmPrivUsbfDisableIdInterrupt(hDevice);
            }
        }
        break;
    case NvRmAnalogUsbInputParam_CheckIdStatus:
        if (inst == 0)
        {
            pUsbConf->UsbIdDetected = NvRmPrivUsbfIsIdSetToLow(hDevice);
        }
        break;
    default:
        NV_ASSERT(NV_FALSE);
        break;
    }

    return error;
}


NvError
NvRmAnalogInterfaceControl(
    NvRmDeviceHandle hDevice,
    NvRmAnalogInterface Interface,
    NvBool Enable,
    void *Config,
    NvU32 ConfigLength )
{
    NvError err = NvSuccess;
    NvU32 id;
    NvU32 inst;

    NV_ASSERT( hDevice );

    id = NVRM_ANALOG_INTERFACE_ID( Interface );
    inst = NVRM_ANALOG_INTERFACE_INSTANCE( Interface );

    NvOsMutexLock( hDevice->mutex );

    switch( id ) {
    case NvRmAnalogInterface_Dsi:
        break;
    case NvRmAnalogInterface_ExternalMemory:
        break;
    case NvRmAnalogInterface_Hdmi:
        break;
    case NvRmAnalogInterface_Lcd:
        break;
    case NvRmAnalogInterface_Uart:
        break;
    case NvRmAnalogInterface_Usb:
        err = NvRmPrivUsbfControl( hDevice, Enable, inst, Config,
            ConfigLength );
        break;
    case NvRmAnalogInterface_Sdio:
        break;
    case NvRmAnalogInterface_Tv:
        err = NvRmPrivTvDcControl( hDevice, Enable, inst, Config,
            ConfigLength );
        break;
    case NvRmAnalogInterface_VideoInput:
        err = NvRmPrivVideoInputControl( hDevice, Enable, inst, Config,
            ConfigLength);
        break;
    default:
        NV_ASSERT(!"Unknown Analog interface passed. ");
    }

    NvOsMutexUnlock( hDevice->mutex );

    return err;
}

NvBool
NvRmUsbIsConnected(
    NvRmDeviceHandle hDevice)
{
    //Do nothing
    return NV_TRUE;
}

NvU32
NvRmUsbDetectChargerState(
    NvRmDeviceHandle hDevice,
    NvU32 wait)
{
    //Do nothing
    return NvOdmUsbChargerType_UsbHost;
}

NvU8
NvRmAnalogGetTvDacConfiguration(
    NvRmDeviceHandle hDevice,
    NvRmAnalogTvDacType Type)
{
    NvU8 RetVal = 0;
    NvU32 OldRegVal = 0;
    NvU32 NewRegVal = 0;

    NV_ASSERT(hDevice);
    
#if NV_USE_FUSE_CLOCK_ENABLE
    // Enable fuse clock
    NvRmPowerModuleClockControl(hDevice, NvRmModuleID_Fuse, 0, NV_TRUE);
#endif    

    // Enable fuse values to be visible before reading the fuses.
    OldRegVal = NV_REGR(hDevice, NvRmPrivModuleID_ClockAndReset, 0, 
        CLK_RST_CONTROLLER_MISC_CLK_ENB_0);
    NewRegVal = NV_FLD_SET_DRF_NUM(CLK_RST_CONTROLLER, MISC_CLK_ENB, 
        CFG_ALL_VISIBLE, 1, OldRegVal);
    NV_REGW(hDevice, NvRmPrivModuleID_ClockAndReset, 0, 
        CLK_RST_CONTROLLER_MISC_CLK_ENB_0, NewRegVal);

    switch (Type)
    {
        case NvRmAnalogTvDacType_CRT:
            RetVal = NV_REGR(hDevice, NvRmModuleID_Fuse, 0, FUSE_DAC_CRT_CALIB_0);
            break;
        case NvRmAnalogTvDacType_SDTV:
            RetVal = NV_REGR(hDevice, NvRmModuleID_Fuse, 0, FUSE_DAC_SDTV_CALIB_0);
            break;
        case NvRmAnalogTvDacType_HDTV:
            RetVal = NV_REGR(hDevice, NvRmModuleID_Fuse, 0, FUSE_DAC_HDTV_CALIB_0);
            break;
        default:
            NV_ASSERT(!"Unsupported this Dac type");
            break;
    }

    // Disable fuse values visibility
    NV_REGW(hDevice, NvRmPrivModuleID_ClockAndReset, 0, 
        CLK_RST_CONTROLLER_MISC_CLK_ENB_0, OldRegVal);

#if NV_USE_FUSE_CLOCK_ENABLE
    // Disable fuse clock
    NvRmPowerModuleClockControl(hDevice, NvRmModuleID_Fuse, 0, NV_FALSE);
#endif

    return RetVal; 
}
