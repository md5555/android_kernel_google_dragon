/*************************************************************************/ /*!
@File
@Title          Services initialisation parameter support
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Services initialisation parameter support functions for
		the Linux kernel.
@License        Dual MIT/GPLv2

The contents of this file are subject to the MIT license as set out below.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

Alternatively, the contents of this file may be used under the terms of
the GNU General Public License Version 2 ("GPL") in which case the provisions
of GPL are applicable instead of those above.

If you wish to allow use of your version of this file only under the terms of
GPL, and not to allow others to use your version of this file under the terms
of the MIT license, indicate your decision by deleting the provisions above
and replace them with the notice and other provisions required by GPL as set
out in the file called "GPL-COPYING" included in this distribution. If you do
not delete the provisions above, a recipient may use your version of this file
under the terms of either the MIT license or GPL.

This License is also included in this distribution in the file called
"MIT-COPYING".

EXCEPT AS OTHERWISE STATED IN A NEGOTIATED AGREEMENT: (A) THE SOFTWARE IS
PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT; AND (B) IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/ /**************************************************************************/

#include <linux/kernel.h>
#include <linux/string.h>

#include "pvr_debug.h"
#include "os_srvinit_param.h"

void 
*SrvInitParamOpen(void)
{
	return NULL;
}

void
SrvInitParamClose(void *pvState)
{
	(void) pvState;
}

void
_SrvInitParamGetBOOL(IMG_BOOL *pbValue, bool bDefault)
{
	*pbValue = bDefault;
}

void
_SrvInitParamGetUINT32(IMG_UINT32 *pui32Value, unsigned int uiDefault)
{
	*pui32Value = uiDefault;
}

bool
_SrvInitParamGetUINT32BitField(IMG_UINT32 *puiValue, unsigned int uiDefault, const char **ppszValues, unsigned int uiNum, const SRV_INIT_PARAM_UINT32_LOOKUP *psLookup, unsigned int uiSize, const char *pszName)
{
	IMG_UINT32 uiValue = uiDefault;
	bool bRet = false;
	unsigned i, j;

	for (i = 0; i < uiNum; i++)
	{
		const char *pszValue = ppszValues[i];

		for (j = 0; j < uiSize; j++)
		{
			if (strcmp(psLookup[j].pszValue, pszValue) == 0)
			{
				uiValue |= psLookup[j].ui32Value;
				bRet = true;
				break;
			}
		}
		if (j ==  uiSize)
		{
			if (strlen(pszValue) == 0)
			{
				PVR_DPF((PVR_DBG_WARNING, "No value set for initialisation parameter %s", pszName));
			}
			else
			{
				PVR_DPF((PVR_DBG_WARNING, "Unrecognised value (%s) for initialisation parameter %s", pszValue, pszName));
			}
		}
	}

	*puiValue = uiValue;

	return bRet;
}

bool
_SrvInitParamGetUINT32List(IMG_UINT32 *puiValue, unsigned int uiDefault, const char *pszValue, const SRV_INIT_PARAM_UINT32_LOOKUP *psLookup, unsigned int uiSize, const char *pszName)
{
	IMG_UINT32 uiValue = uiDefault;
	bool bRet = false;
	unsigned i;

	if (pszValue != NULL)
	{
		bRet = true;

		for (i = 0; i < uiSize; i++)
		{
			if (strcmp(psLookup[i].pszValue, pszValue) == 0)
			{
				uiValue = psLookup[i].ui32Value;
				break;
			}
		}
		if (i ==  uiSize)
		{
			if (strlen(pszValue) == 0)
			{
				PVR_DPF((PVR_DBG_WARNING, "No value set for initialisation parameter %s", pszName));
			}
			else
			{
				PVR_DPF((PVR_DBG_WARNING, "Unrecognised value (%s) for initialisation parameter %s", pszValue, pszName));
			}
		}
	}

	*puiValue = uiValue;

	return bRet;
}
