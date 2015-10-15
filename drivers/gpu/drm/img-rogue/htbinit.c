/*************************************************************************/ /*!
@File
@Title          Host trace buffer initialisation routines
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
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

#include "img_defs.h"
#include "pvr_debug.h"
#include "srvinit.h"
#include "srvinit_param.h"
#include "htbuffer_types.h"
#include "htbuffer_init.h"

/* apphint map of name vs. enable flag */
static SRV_INIT_PARAM_UINT32_LOOKUP asLogGroupTable[] = {
#define X(a, b) { #b, HTB_LOG_GROUP_FLAG_NAME(b) },
	HTB_LOG_SFGROUPLIST
#undef X
};
/* apphint map of arg vs. OpMode */
static SRV_INIT_PARAM_UINT32_LOOKUP asOpModeTable[] = {
	{ "droplatest", HTB_OPMODE_DROPLATEST},
	{ "dropoldest", HTB_OPMODE_DROPOLDEST},
	/* HTB should never be started in HTB_OPMODE_BLOCK
	 * as this can lead to deadlocks
	 */
};
/* apphint map of arg vs. LogMode */
static SRV_INIT_PARAM_UINT32_LOOKUP asLogModeTable[] = {
	{ "all", HTB_LOGMODE_ALLPID},
	{ "restricted", HTB_LOGMODE_RESTRICTEDPID}
};


/* setup apphint root and associated data from the apphint maps */
/* Future improvements: AppHint parsing will need to be reworked to support more than 32 Log Groups */
SrvInitParamInitUINT32BitField( EnableHTBLogGroup,     0,                       asLogGroupTable);
SrvInitParamInitUINT32List(     HTBOperationMode,      HTB_OPMODE_DROPLATEST,   asOpModeTable);
SrvInitParamInitUINT32List(     HTBLogMode,            HTB_LOGMODE_ALLPID,      asLogModeTable);
SrvInitParamInitUINT32(         HTBufferSize,          0x1000 );

/* Future improvements: */
SrvInitParamInitBOOL(           EnableHTBPID,          0);
SrvInitParamInitUINT32(         HTBLogLevel,           0);

IMG_INTERNAL void
_ParseHTBAppHints(SHARED_DEV_CONNECTION hServices)
{
	PVRSRV_ERROR eError;
	void * pvParamState = NULL;
	IMG_UINT32 ui32LogType;
	IMG_BOOL bAnyLogGroupConfigured;
	HTB_LOGMODE_CTRL eLogMode;

	IMG_CHAR * szBufferName = "PVRHTBuffer";
	IMG_UINT32 ui32BufferSize;
	HTB_OPMODE_CTRL eOpMode;

	/* Services initialisation parameters */
	pvParamState = SrvInitParamOpen();

	bAnyLogGroupConfigured = SrvInitParamGetUINT32BitField(pvParamState, EnableHTBLogGroup, ui32LogType);
	SrvInitParamGetUINT32List(pvParamState, HTBOperationMode, eOpMode);
	SrvInitParamGetUINT32(pvParamState, HTBufferSize, ui32BufferSize);
	SrvInitParamGetUINT32List(pvParamState, HTBLogMode, eLogMode);

	/* future improvements:
	 * PID should be enabled against a process name
	 * LogLevel requires SF change
	 */
	{
	IMG_BOOL bEnablePID;
	IMG_UINT32 ui32LogLevel;
	SrvInitParamGetBOOL(pvParamState, EnableHTBPID, bEnablePID);
	SrvInitParamGetUINT32(pvParamState, HTBLogLevel, ui32LogLevel);
	}

	eError = HTBConfigure(hServices, szBufferName, ui32BufferSize);
	PVR_LOGG_IF_ERROR(eError, "PVRSRVHTBConfigure", cleanup);

	if (bAnyLogGroupConfigured)
	{
		eError = HTBControl(hServices, 1, &ui32LogType, 0, 0, eLogMode, eOpMode);
		PVR_LOGG_IF_ERROR(eError, "PVRSRVHTBControl", cleanup);
	}

cleanup:
	SrvInitParamClose(pvParamState);
}

/******************************************************************************
 End of file (htbinit.c)
*****************************************************************************/
