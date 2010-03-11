/*
 * Copyright (c) 2007-2009 NVIDIA Corporation.
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

#include "nvrm_gpio_private.h"
#include "nvassert.h"
#include "nvos.h"

static NvRmGpioCaps s_ap15_caps = {6, 4, GPIO_PINS_PER_PORT, NVRM_GPIO_CAP_FEAT_EDGE_INTR /* (SEE BUG# 366493) */};

static NvRmModuleCapability s_capsArray[] = {
    /* Major, minor, eco and caps structure */
    { 2, 0, 0, &s_ap15_caps },
};

static NvBool s_GpioIoPowerInitialized = NV_FALSE;

static NvRmGpioIoPowerInfo s_GpioIoPowerTable[] =
{
    {NV_VDD_SYS_ODM_ID, 0},
    {NV_VDD_BB_ODM_ID, 0},
    {NV_VDD_VI_ODM_ID, 0},
    {NV_VDD_SDIO_ODM_ID, 0},
    {NV_VDD_LCD_ODM_ID, 0},
    {NV_VDD_UART_ODM_ID, 0}
};

NvError
NvRmGpioGetCapabilities(
    NvRmDeviceHandle hRm,
    void **Capability )
{
    NvError err = NvSuccess;

    NV_ASSERT(hRm);

    err = NvRmModuleGetCapabilities(hRm, NvRmPrivModuleID_Gpio, s_capsArray,
            NV_ARRAY_SIZE(s_capsArray), Capability);
    if (err)
    {
        /* Default to AP15 caps.
          FIXME: findout why the RM API is returning failure. */
        NV_ASSERT(0);
        *Capability = (void*)&s_ap15_caps;
    }

   ((NvRmGpioCaps *)*Capability)->Instances = 
       NvRmModuleGetNumInstances(hRm, NvRmPrivModuleID_Gpio);
    
    return err;
}

static NvError NvRmGpioIoPowerDiscover(
    NvRmDeviceHandle hRm)
{
    NvU32 i;
    const NvOdmPeripheralConnectivity* pCon = NULL;

    for (i = 0; i < NV_ARRAY_SIZE(s_GpioIoPowerTable); i++)
    {
        pCon = NvOdmPeripheralGetGuid(s_GpioIoPowerTable[i].PowerRailId);
        if (!pCon || !pCon->NumAddress)
            return NvError_NotSupported;
        s_GpioIoPowerTable[i].PmuRailAddress = pCon->AddressList[0].Address;
    }
    return NvSuccess;
}

NvError NvRmGpioIoPowerConfig(
    NvRmDeviceHandle hRm,
    NvU32 port,
    NvU32 pinNumber,
    NvBool Enable)
{
    NvRmPmuVddRailCapabilities RailCaps;
    NvU32 SettlingTime;
    NvRmGpioIoPowerInfo *pGpioIoPower;

    if (!s_GpioIoPowerInitialized)
    {
        NvError err = NvRmGpioIoPowerDiscover(hRm);
        if (err)
            return err;
        s_GpioIoPowerInitialized = NV_TRUE;
    }

    if ((port == GPIO_PORT('s')) ||
        (port == GPIO_PORT('q')) ||
        (port == GPIO_PORT('r')))
    {
        /* NV_VDD_SYS_ODM_ID */
        pGpioIoPower = &s_GpioIoPowerTable[0];
    }
    else if ((port == GPIO_PORT('o')) ||
            ((port == GPIO_PORT('v')) && (pinNumber < 4)))
    {
        /* NV_VDD_BB_ODM_ID */
        pGpioIoPower = &s_GpioIoPowerTable[1];
    }
    else if ((port == GPIO_PORT('l')) ||
            ((port == GPIO_PORT('d')) && (pinNumber > 4)) ||
            ((port == GPIO_PORT('t')) && (pinNumber < 5)))
    {
        /* NV_VDD_VI_ODM_ID */
        pGpioIoPower = &s_GpioIoPowerTable[2];
    }
    else if (((port == GPIO_PORT('d')) && (pinNumber < 5)) ||
             ((port == GPIO_PORT('b')) && (pinNumber > 3)) ||
             ((port == GPIO_PORT('v')) && ((pinNumber > 3) && 
                (pinNumber < 7)))                          ||
             ((port == GPIO_PORT('a')) && ((pinNumber > 5) || 
                (pinNumber == 0))))
    {
        /* NV_VDD_SDIO_ODM_ID */
        pGpioIoPower = &s_GpioIoPowerTable[3];
    }
    else if ((port == GPIO_PORT('e')) ||
             (port == GPIO_PORT('f')) ||
             (port == GPIO_PORT('m')) ||
             ((port == GPIO_PORT('c')) && ((pinNumber == 1) || 
                 (pinNumber == 6)))                         ||
             ((port == GPIO_PORT('w')) && (pinNumber < 2))  ||
             ((port == GPIO_PORT('j')) && ((pinNumber == 1) || 
                     (pinNumber == 3) || (pinNumber == 4))) ||
             ((port == GPIO_PORT('v')) && (pinNumber == 7)) ||
             ((port == GPIO_PORT('n')) && (pinNumber > 3))  ||
             ((port == GPIO_PORT('b')) && ((pinNumber == 2) || 
                  (pinNumber == 3))))
    {
        /* NV_VDD_LCD_ODM_ID */
        pGpioIoPower = &s_GpioIoPowerTable[4];
    }
    else
    {
        /* NV_VDD_UART_ODM_ID */
        pGpioIoPower = &s_GpioIoPowerTable[5];
    }

    if (Enable)
    {
        NvRmPmuGetCapabilities(hRm,
            pGpioIoPower->PmuRailAddress, &RailCaps);
        NvRmPmuSetVoltage(hRm,
            pGpioIoPower->PmuRailAddress,
            RailCaps.requestMilliVolts, &SettlingTime);
    }
    else
    {
        NvRmPmuSetVoltage(hRm,
            pGpioIoPower->PmuRailAddress,
            ODM_VOLTAGE_OFF, &SettlingTime);
    }
    if (SettlingTime)
        NvOsWaitUS(SettlingTime);
    
    return NvSuccess;
}

