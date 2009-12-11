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

#ifndef INCLUDED_NVRM_GPIO_PRIVATE_H
#define INCLUDED_NVRM_GPIO_PRIVATE_H

#include "nvrm_gpio.h"
#include "ap15/argpio.h"
#include "nvrm_structure.h"
#include "ap15/ap15rm_private.h"
#include "nvrm_hwintf.h"
#include "nvodm_query_discovery.h"
#include "nvrm_pmu.h"

#define GPIO_INTR_MAX 32
#define GPIO_PORT(x) ((x) - 'a')
#define GET_PIN(h)      ((((NvU32)(h))) & 0xFF)
#define GET_PORT(h)     ((((NvU32)(h)) >> 8) & 0xFF)
#define GET_INSTANCE(h) ((((NvU32)(h)) >> 16) & 0xFF)

// Size of a port register.
#define NV_GPIO_PORT_REG_SIZE   (GPIO_CNF_1 - GPIO_CNF_0)
#define GPIO_INT_LVL_UNSHADOWED_MASK \
            (GPIO_INT_LVL_0_BIT_7_FIELD | GPIO_INT_LVL_0_BIT_6_FIELD | \
            GPIO_INT_LVL_0_BIT_5_FIELD | GPIO_INT_LVL_0_BIT_4_FIELD | \
            GPIO_INT_LVL_0_BIT_3_FIELD | GPIO_INT_LVL_0_BIT_2_FIELD | \
            GPIO_INT_LVL_0_BIT_1_FIELD | GPIO_INT_LVL_0_BIT_0_FIELD)

#define GPIO_INT_LVL_SHADOWED_MASK (~GPIO_INT_LVL_UNSHADOWED_MASK)

// Gpio register read/write macros

#define GPIO_PINS_PER_PORT  8

#define GPIO_MASKED_WRITE(rm, Instance, Port, Reg, Pin, value) \
    do \
    { \
        NV_REGW((rm), NvRmPrivModuleID_Gpio, (Instance), ((Port) * NV_GPIO_PORT_REG_SIZE) + GPIO_MSK_CNF_0 + \
                                    (GPIO_##Reg##_0), (((1<<((Pin)+ GPIO_PINS_PER_PORT)) | ((value) << (Pin))))); \
    } while (0)


// Gpio register read/write macros
#define GPIO_REGR( rm, Instance, Port, Reg, ReadData) \
    do  \
    {   \
        ReadData = NV_REGR((rm), NvRmPrivModuleID_Gpio, (Instance), ((Port) * NV_GPIO_PORT_REG_SIZE) + \
                                                    (GPIO_##Reg##_0)); \
    } while (0)

#define GPIO_REGW( rm, Instance, Port, Reg, Data2Write ) \
    do \
    { \
        NV_REGW((rm), NvRmPrivModuleID_Gpio, (Instance), ((Port) * NV_GPIO_PORT_REG_SIZE) + \
                                    (GPIO_##Reg##_0), (Data2Write)); \
    } while (0)

/* Bit mask of hardware features present in GPIO controller. */
typedef enum {
    NVRM_GPIO_CAP_FEAT_NONE = 0,
    NVRM_GPIO_CAP_FEAT_EDGE_INTR = 0x000000001
} NvRmGpioCapFeatures;


typedef struct NvRmGpioCapsRec {
    NvU32 Instances;
    NvU32 PortsPerInstances;
    NvU32 PinsPerPort;
    NvU32 Features;
} NvRmGpioCaps;

typedef struct NvRmGpioIoPowerInfoRec
{
    // SoC Power rail GUID 
    NvU64 PowerRailId;

    // PMU Rail Address
    NvU32 PmuRailAddress;

} NvRmGpioIoPowerInfo;

/**
 * GPIO wrapper for NvRmModuleGetCapabilities().
 *
 * @param hRm The RM device handle
 * @param Capability Out parameter: the cap that maches the current hardware
 */
NvError
NvRmGpioGetCapabilities(
    NvRmDeviceHandle hRm,
    void **Capability );

NvError NvRmGpioIoPowerConfig(
    NvRmDeviceHandle hRm,
    NvU32 port,
    NvU32 pinNumber,
    NvBool Enable);

#endif  // INCLUDED_NVRM_GPIO_PRIVATE_H

