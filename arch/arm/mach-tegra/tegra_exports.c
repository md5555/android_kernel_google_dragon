/*
 * tegra_exports.c
 *
 * Export Tegra-specific functions for use by kernel loadable modules.
 *
 * Copyright (c) 2009, NVIDIA Corporation.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include "nvrm_spi.h"
#include "nvodm_services.h"
#include "nvodm_query_discovery.h"
#include "nvos.h"

EXPORT_SYMBOL(NvOdmGpioOpen);
EXPORT_SYMBOL(NvOdmGpioClose);
EXPORT_SYMBOL(NvOdmGpioConfig);
EXPORT_SYMBOL(NvOdmGpioGetState);
EXPORT_SYMBOL(NvOdmGpioSetState);
EXPORT_SYMBOL(NvOdmGpioAcquirePinHandle);
EXPORT_SYMBOL(NvOdmGpioReleasePinHandle);
EXPORT_SYMBOL(NvOdmGpioInterruptRegister);
EXPORT_SYMBOL(NvOdmGpioInterruptUnregister);
EXPORT_SYMBOL(NvOdmGpioInterruptDone);
EXPORT_SYMBOL(NvOdmOsSemaphoreCreate);
EXPORT_SYMBOL(NvOdmOsSemaphoreDestroy);
EXPORT_SYMBOL(NvOdmOsSemaphoreSignal);
EXPORT_SYMBOL(NvOdmOsSemaphoreWait);
EXPORT_SYMBOL(NvOdmOsSleepMS);
EXPORT_SYMBOL(NvOdmPeripheralGetGuid);
EXPORT_SYMBOL(NvOsAlloc);
EXPORT_SYMBOL(NvOsFree);
EXPORT_SYMBOL(NvOsMemset);
EXPORT_SYMBOL(NvOsThreadCreate);
EXPORT_SYMBOL(NvOsThreadJoin);
EXPORT_SYMBOL(NvOsDebugPrintf);
EXPORT_SYMBOL(NvRmOpen);
EXPORT_SYMBOL(NvRmClose);
EXPORT_SYMBOL(NvRmSpiOpen);
EXPORT_SYMBOL(NvRmSpiClose);
EXPORT_SYMBOL(NvRmSpiStartTransaction);
EXPORT_SYMBOL(NvRmSpiGetTransactionData);

