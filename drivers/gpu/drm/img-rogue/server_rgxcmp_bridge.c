/*************************************************************************/ /*!
@File
@Title          Server bridge for rgxcmp
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Implements the server side of the bridge for rgxcmp
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

#include <stddef.h>
#include <asm/uaccess.h>

#include "img_defs.h"

#include "rgxcompute.h"


#include "common_rgxcmp_bridge.h"

#include "allocmem.h"
#include "pvr_debug.h"
#include "connection_server.h"
#include "pvr_bridge.h"
#include "rgx_bridge.h"
#include "srvcore.h"
#include "handle.h"

#include <linux/slab.h>




/* ***************************************************************************
 * Server-side bridge entry points
 */
 
static IMG_INT
PVRSRVBridgeRGXCreateComputeContext(IMG_UINT32 ui32DispatchTableEntry,
					  PVRSRV_BRIDGE_IN_RGXCREATECOMPUTECONTEXT *psRGXCreateComputeContextIN,
					  PVRSRV_BRIDGE_OUT_RGXCREATECOMPUTECONTEXT *psRGXCreateComputeContextOUT,
					 CONNECTION_DATA *psConnection)
{
	IMG_BYTE *psFrameworkCmdInt = NULL;
	IMG_HANDLE hPrivDataInt = NULL;
	RGX_SERVER_COMPUTE_CONTEXT * psComputeContextInt = NULL;




	if (psRGXCreateComputeContextIN->ui32FrameworkCmdize != 0)
	{
		psFrameworkCmdInt = OSAllocMem(psRGXCreateComputeContextIN->ui32FrameworkCmdize * sizeof(IMG_BYTE));
		if (!psFrameworkCmdInt)
		{
			psRGXCreateComputeContextOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;
	
			goto RGXCreateComputeContext_exit;
		}
	}

			/* Copy the data over */
			if ( !OSAccessOK(PVR_VERIFY_READ, (void*) psRGXCreateComputeContextIN->psFrameworkCmd, psRGXCreateComputeContextIN->ui32FrameworkCmdize * sizeof(IMG_BYTE))
				|| (OSCopyFromUser(NULL, psFrameworkCmdInt, psRGXCreateComputeContextIN->psFrameworkCmd,
				psRGXCreateComputeContextIN->ui32FrameworkCmdize * sizeof(IMG_BYTE)) != PVRSRV_OK) )
			{
				psRGXCreateComputeContextOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

				goto RGXCreateComputeContext_exit;
			}

	PMRLock();


				{
					/* Look up the address from the handle */
					psRGXCreateComputeContextOUT->eError =
						PVRSRVLookupHandle(psConnection->psHandleBase,
											(void **) &hPrivDataInt,
											psRGXCreateComputeContextIN->hPrivData,
											PVRSRV_HANDLE_TYPE_DEV_PRIV_DATA);
					if(psRGXCreateComputeContextOUT->eError != PVRSRV_OK)
					{
						PMRUnlock();
						goto RGXCreateComputeContext_exit;
					}
				}


	psRGXCreateComputeContextOUT->eError =
		PVRSRVRGXCreateComputeContextKM(psConnection, OSGetDevData(psConnection),
					psRGXCreateComputeContextIN->ui32Priority,
					psRGXCreateComputeContextIN->sMCUFenceAddr,
					psRGXCreateComputeContextIN->ui32FrameworkCmdize,
					psFrameworkCmdInt,
					hPrivDataInt,
					&psComputeContextInt);
	/* Exit early if bridged call fails */
	if(psRGXCreateComputeContextOUT->eError != PVRSRV_OK)
	{
		PMRUnlock();
		goto RGXCreateComputeContext_exit;
	}
	PMRUnlock();


	psRGXCreateComputeContextOUT->eError = PVRSRVAllocHandle(psConnection->psHandleBase,
							&psRGXCreateComputeContextOUT->hComputeContext,
							(void *) psComputeContextInt,
							PVRSRV_HANDLE_TYPE_RGX_SERVER_COMPUTE_CONTEXT,
							PVRSRV_HANDLE_ALLOC_FLAG_MULTI
							,(PFN_HANDLE_RELEASE)&PVRSRVRGXDestroyComputeContextKM);
	if (psRGXCreateComputeContextOUT->eError != PVRSRV_OK)
	{
		goto RGXCreateComputeContext_exit;
	}




RGXCreateComputeContext_exit:
	if (psRGXCreateComputeContextOUT->eError != PVRSRV_OK)
	{
		if (psComputeContextInt)
		{
			PVRSRVRGXDestroyComputeContextKM(psComputeContextInt);
		}
	}

	if (psFrameworkCmdInt)
		OSFreeMem(psFrameworkCmdInt);

	return 0;
}

static IMG_INT
PVRSRVBridgeRGXDestroyComputeContext(IMG_UINT32 ui32DispatchTableEntry,
					  PVRSRV_BRIDGE_IN_RGXDESTROYCOMPUTECONTEXT *psRGXDestroyComputeContextIN,
					  PVRSRV_BRIDGE_OUT_RGXDESTROYCOMPUTECONTEXT *psRGXDestroyComputeContextOUT,
					 CONNECTION_DATA *psConnection)
{





	PMRLock();




	psRGXDestroyComputeContextOUT->eError =
		PVRSRVReleaseHandle(psConnection->psHandleBase,
					(IMG_HANDLE) psRGXDestroyComputeContextIN->hComputeContext,
					PVRSRV_HANDLE_TYPE_RGX_SERVER_COMPUTE_CONTEXT);
	if ((psRGXDestroyComputeContextOUT->eError != PVRSRV_OK) && (psRGXDestroyComputeContextOUT->eError != PVRSRV_ERROR_RETRY))
	{
		PVR_ASSERT(0);
		PMRUnlock();
		goto RGXDestroyComputeContext_exit;
	}

	PMRUnlock();


RGXDestroyComputeContext_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeRGXKickCDM(IMG_UINT32 ui32DispatchTableEntry,
					  PVRSRV_BRIDGE_IN_RGXKICKCDM *psRGXKickCDMIN,
					  PVRSRV_BRIDGE_OUT_RGXKICKCDM *psRGXKickCDMOUT,
					 CONNECTION_DATA *psConnection)
{
	RGX_SERVER_COMPUTE_CONTEXT * psComputeContextInt = NULL;
	SYNC_PRIMITIVE_BLOCK * *psClientFenceUFOSyncPrimBlockInt = NULL;
	IMG_HANDLE *hClientFenceUFOSyncPrimBlockInt2 = NULL;
	IMG_UINT32 *ui32ClientFenceOffsetInt = NULL;
	IMG_UINT32 *ui32ClientFenceValueInt = NULL;
	SYNC_PRIMITIVE_BLOCK * *psClientUpdateUFOSyncPrimBlockInt = NULL;
	IMG_HANDLE *hClientUpdateUFOSyncPrimBlockInt2 = NULL;
	IMG_UINT32 *ui32ClientUpdateOffsetInt = NULL;
	IMG_UINT32 *ui32ClientUpdateValueInt = NULL;
	IMG_UINT32 *ui32ServerSyncFlagsInt = NULL;
	SERVER_SYNC_PRIMITIVE * *psServerSyncsInt = NULL;
	IMG_HANDLE *hServerSyncsInt2 = NULL;
	IMG_BYTE *psDMCmdInt = NULL;




	if (psRGXKickCDMIN->ui32ClientFenceCount != 0)
	{
		psClientFenceUFOSyncPrimBlockInt = OSAllocMem(psRGXKickCDMIN->ui32ClientFenceCount * sizeof(SYNC_PRIMITIVE_BLOCK *));
		if (!psClientFenceUFOSyncPrimBlockInt)
		{
			psRGXKickCDMOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;
	
			goto RGXKickCDM_exit;
		}
		hClientFenceUFOSyncPrimBlockInt2 = OSAllocMem(psRGXKickCDMIN->ui32ClientFenceCount * sizeof(IMG_HANDLE));
		if (!hClientFenceUFOSyncPrimBlockInt2)
		{
			psRGXKickCDMOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;
	
			goto RGXKickCDM_exit;
		}
	}

			/* Copy the data over */
			if ( !OSAccessOK(PVR_VERIFY_READ, (void*) psRGXKickCDMIN->phClientFenceUFOSyncPrimBlock, psRGXKickCDMIN->ui32ClientFenceCount * sizeof(IMG_HANDLE))
				|| (OSCopyFromUser(NULL, hClientFenceUFOSyncPrimBlockInt2, psRGXKickCDMIN->phClientFenceUFOSyncPrimBlock,
				psRGXKickCDMIN->ui32ClientFenceCount * sizeof(IMG_HANDLE)) != PVRSRV_OK) )
			{
				psRGXKickCDMOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

				goto RGXKickCDM_exit;
			}
	if (psRGXKickCDMIN->ui32ClientFenceCount != 0)
	{
		ui32ClientFenceOffsetInt = OSAllocMem(psRGXKickCDMIN->ui32ClientFenceCount * sizeof(IMG_UINT32));
		if (!ui32ClientFenceOffsetInt)
		{
			psRGXKickCDMOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;
	
			goto RGXKickCDM_exit;
		}
	}

			/* Copy the data over */
			if ( !OSAccessOK(PVR_VERIFY_READ, (void*) psRGXKickCDMIN->pui32ClientFenceOffset, psRGXKickCDMIN->ui32ClientFenceCount * sizeof(IMG_UINT32))
				|| (OSCopyFromUser(NULL, ui32ClientFenceOffsetInt, psRGXKickCDMIN->pui32ClientFenceOffset,
				psRGXKickCDMIN->ui32ClientFenceCount * sizeof(IMG_UINT32)) != PVRSRV_OK) )
			{
				psRGXKickCDMOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

				goto RGXKickCDM_exit;
			}
	if (psRGXKickCDMIN->ui32ClientFenceCount != 0)
	{
		ui32ClientFenceValueInt = OSAllocMem(psRGXKickCDMIN->ui32ClientFenceCount * sizeof(IMG_UINT32));
		if (!ui32ClientFenceValueInt)
		{
			psRGXKickCDMOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;
	
			goto RGXKickCDM_exit;
		}
	}

			/* Copy the data over */
			if ( !OSAccessOK(PVR_VERIFY_READ, (void*) psRGXKickCDMIN->pui32ClientFenceValue, psRGXKickCDMIN->ui32ClientFenceCount * sizeof(IMG_UINT32))
				|| (OSCopyFromUser(NULL, ui32ClientFenceValueInt, psRGXKickCDMIN->pui32ClientFenceValue,
				psRGXKickCDMIN->ui32ClientFenceCount * sizeof(IMG_UINT32)) != PVRSRV_OK) )
			{
				psRGXKickCDMOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

				goto RGXKickCDM_exit;
			}
	if (psRGXKickCDMIN->ui32ClientUpdateCount != 0)
	{
		psClientUpdateUFOSyncPrimBlockInt = OSAllocMem(psRGXKickCDMIN->ui32ClientUpdateCount * sizeof(SYNC_PRIMITIVE_BLOCK *));
		if (!psClientUpdateUFOSyncPrimBlockInt)
		{
			psRGXKickCDMOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;
	
			goto RGXKickCDM_exit;
		}
		hClientUpdateUFOSyncPrimBlockInt2 = OSAllocMem(psRGXKickCDMIN->ui32ClientUpdateCount * sizeof(IMG_HANDLE));
		if (!hClientUpdateUFOSyncPrimBlockInt2)
		{
			psRGXKickCDMOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;
	
			goto RGXKickCDM_exit;
		}
	}

			/* Copy the data over */
			if ( !OSAccessOK(PVR_VERIFY_READ, (void*) psRGXKickCDMIN->phClientUpdateUFOSyncPrimBlock, psRGXKickCDMIN->ui32ClientUpdateCount * sizeof(IMG_HANDLE))
				|| (OSCopyFromUser(NULL, hClientUpdateUFOSyncPrimBlockInt2, psRGXKickCDMIN->phClientUpdateUFOSyncPrimBlock,
				psRGXKickCDMIN->ui32ClientUpdateCount * sizeof(IMG_HANDLE)) != PVRSRV_OK) )
			{
				psRGXKickCDMOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

				goto RGXKickCDM_exit;
			}
	if (psRGXKickCDMIN->ui32ClientUpdateCount != 0)
	{
		ui32ClientUpdateOffsetInt = OSAllocMem(psRGXKickCDMIN->ui32ClientUpdateCount * sizeof(IMG_UINT32));
		if (!ui32ClientUpdateOffsetInt)
		{
			psRGXKickCDMOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;
	
			goto RGXKickCDM_exit;
		}
	}

			/* Copy the data over */
			if ( !OSAccessOK(PVR_VERIFY_READ, (void*) psRGXKickCDMIN->pui32ClientUpdateOffset, psRGXKickCDMIN->ui32ClientUpdateCount * sizeof(IMG_UINT32))
				|| (OSCopyFromUser(NULL, ui32ClientUpdateOffsetInt, psRGXKickCDMIN->pui32ClientUpdateOffset,
				psRGXKickCDMIN->ui32ClientUpdateCount * sizeof(IMG_UINT32)) != PVRSRV_OK) )
			{
				psRGXKickCDMOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

				goto RGXKickCDM_exit;
			}
	if (psRGXKickCDMIN->ui32ClientUpdateCount != 0)
	{
		ui32ClientUpdateValueInt = OSAllocMem(psRGXKickCDMIN->ui32ClientUpdateCount * sizeof(IMG_UINT32));
		if (!ui32ClientUpdateValueInt)
		{
			psRGXKickCDMOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;
	
			goto RGXKickCDM_exit;
		}
	}

			/* Copy the data over */
			if ( !OSAccessOK(PVR_VERIFY_READ, (void*) psRGXKickCDMIN->pui32ClientUpdateValue, psRGXKickCDMIN->ui32ClientUpdateCount * sizeof(IMG_UINT32))
				|| (OSCopyFromUser(NULL, ui32ClientUpdateValueInt, psRGXKickCDMIN->pui32ClientUpdateValue,
				psRGXKickCDMIN->ui32ClientUpdateCount * sizeof(IMG_UINT32)) != PVRSRV_OK) )
			{
				psRGXKickCDMOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

				goto RGXKickCDM_exit;
			}
	if (psRGXKickCDMIN->ui32ServerSyncCount != 0)
	{
		ui32ServerSyncFlagsInt = OSAllocMem(psRGXKickCDMIN->ui32ServerSyncCount * sizeof(IMG_UINT32));
		if (!ui32ServerSyncFlagsInt)
		{
			psRGXKickCDMOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;
	
			goto RGXKickCDM_exit;
		}
	}

			/* Copy the data over */
			if ( !OSAccessOK(PVR_VERIFY_READ, (void*) psRGXKickCDMIN->pui32ServerSyncFlags, psRGXKickCDMIN->ui32ServerSyncCount * sizeof(IMG_UINT32))
				|| (OSCopyFromUser(NULL, ui32ServerSyncFlagsInt, psRGXKickCDMIN->pui32ServerSyncFlags,
				psRGXKickCDMIN->ui32ServerSyncCount * sizeof(IMG_UINT32)) != PVRSRV_OK) )
			{
				psRGXKickCDMOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

				goto RGXKickCDM_exit;
			}
	if (psRGXKickCDMIN->ui32ServerSyncCount != 0)
	{
		psServerSyncsInt = OSAllocMem(psRGXKickCDMIN->ui32ServerSyncCount * sizeof(SERVER_SYNC_PRIMITIVE *));
		if (!psServerSyncsInt)
		{
			psRGXKickCDMOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;
	
			goto RGXKickCDM_exit;
		}
		hServerSyncsInt2 = OSAllocMem(psRGXKickCDMIN->ui32ServerSyncCount * sizeof(IMG_HANDLE));
		if (!hServerSyncsInt2)
		{
			psRGXKickCDMOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;
	
			goto RGXKickCDM_exit;
		}
	}

			/* Copy the data over */
			if ( !OSAccessOK(PVR_VERIFY_READ, (void*) psRGXKickCDMIN->phServerSyncs, psRGXKickCDMIN->ui32ServerSyncCount * sizeof(IMG_HANDLE))
				|| (OSCopyFromUser(NULL, hServerSyncsInt2, psRGXKickCDMIN->phServerSyncs,
				psRGXKickCDMIN->ui32ServerSyncCount * sizeof(IMG_HANDLE)) != PVRSRV_OK) )
			{
				psRGXKickCDMOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

				goto RGXKickCDM_exit;
			}
	if (psRGXKickCDMIN->ui32CmdSize != 0)
	{
		psDMCmdInt = OSAllocMem(psRGXKickCDMIN->ui32CmdSize * sizeof(IMG_BYTE));
		if (!psDMCmdInt)
		{
			psRGXKickCDMOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;
	
			goto RGXKickCDM_exit;
		}
	}

			/* Copy the data over */
			if ( !OSAccessOK(PVR_VERIFY_READ, (void*) psRGXKickCDMIN->psDMCmd, psRGXKickCDMIN->ui32CmdSize * sizeof(IMG_BYTE))
				|| (OSCopyFromUser(NULL, psDMCmdInt, psRGXKickCDMIN->psDMCmd,
				psRGXKickCDMIN->ui32CmdSize * sizeof(IMG_BYTE)) != PVRSRV_OK) )
			{
				psRGXKickCDMOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

				goto RGXKickCDM_exit;
			}

#if defined(PDUMP)
	PMRLock();
#endif


				{
					/* Look up the address from the handle */
					psRGXKickCDMOUT->eError =
						PVRSRVLookupHandle(psConnection->psHandleBase,
											(void **) &psComputeContextInt,
											psRGXKickCDMIN->hComputeContext,
											PVRSRV_HANDLE_TYPE_RGX_SERVER_COMPUTE_CONTEXT);
					if(psRGXKickCDMOUT->eError != PVRSRV_OK)
					{
#if defined(PDUMP)
						PMRUnlock();
#endif
						goto RGXKickCDM_exit;
					}
				}


	{
		IMG_UINT32 i;

		for (i=0;i<psRGXKickCDMIN->ui32ClientFenceCount;i++)
		{
				{
					/* Look up the address from the handle */
					psRGXKickCDMOUT->eError =
						PVRSRVLookupHandle(psConnection->psHandleBase,
											(void **) &psClientFenceUFOSyncPrimBlockInt[i],
											hClientFenceUFOSyncPrimBlockInt2[i],
											PVRSRV_HANDLE_TYPE_SYNC_PRIMITIVE_BLOCK);
					if(psRGXKickCDMOUT->eError != PVRSRV_OK)
					{
#if defined(PDUMP)
						PMRUnlock();
#endif
						goto RGXKickCDM_exit;
					}
				}

		}
	}

	{
		IMG_UINT32 i;

		for (i=0;i<psRGXKickCDMIN->ui32ClientUpdateCount;i++)
		{
				{
					/* Look up the address from the handle */
					psRGXKickCDMOUT->eError =
						PVRSRVLookupHandle(psConnection->psHandleBase,
											(void **) &psClientUpdateUFOSyncPrimBlockInt[i],
											hClientUpdateUFOSyncPrimBlockInt2[i],
											PVRSRV_HANDLE_TYPE_SYNC_PRIMITIVE_BLOCK);
					if(psRGXKickCDMOUT->eError != PVRSRV_OK)
					{
#if defined(PDUMP)
						PMRUnlock();
#endif
						goto RGXKickCDM_exit;
					}
				}

		}
	}

	{
		IMG_UINT32 i;

		for (i=0;i<psRGXKickCDMIN->ui32ServerSyncCount;i++)
		{
				{
					/* Look up the address from the handle */
					psRGXKickCDMOUT->eError =
						PVRSRVLookupHandle(psConnection->psHandleBase,
											(void **) &psServerSyncsInt[i],
											hServerSyncsInt2[i],
											PVRSRV_HANDLE_TYPE_SERVER_SYNC_PRIMITIVE);
					if(psRGXKickCDMOUT->eError != PVRSRV_OK)
					{
#if defined(PDUMP)
						PMRUnlock();
#endif
						goto RGXKickCDM_exit;
					}
				}

		}
	}

	psRGXKickCDMOUT->eError =
		PVRSRVRGXKickCDMKM(
					psComputeContextInt,
					psRGXKickCDMIN->ui32ClientFenceCount,
					psClientFenceUFOSyncPrimBlockInt,
					ui32ClientFenceOffsetInt,
					ui32ClientFenceValueInt,
					psRGXKickCDMIN->ui32ClientUpdateCount,
					psClientUpdateUFOSyncPrimBlockInt,
					ui32ClientUpdateOffsetInt,
					ui32ClientUpdateValueInt,
					psRGXKickCDMIN->ui32ServerSyncCount,
					ui32ServerSyncFlagsInt,
					psServerSyncsInt,
					psRGXKickCDMIN->ui32CmdSize,
					psDMCmdInt,
					psRGXKickCDMIN->bbPDumpContinuous,
					psRGXKickCDMIN->ui32ExtJobRef,
					psRGXKickCDMIN->ui32IntJobRef);
#if defined(PDUMP)
	PMRUnlock();
#endif




RGXKickCDM_exit:
	if (psClientFenceUFOSyncPrimBlockInt)
		OSFreeMem(psClientFenceUFOSyncPrimBlockInt);
	if (hClientFenceUFOSyncPrimBlockInt2)
		OSFreeMem(hClientFenceUFOSyncPrimBlockInt2);
	if (ui32ClientFenceOffsetInt)
		OSFreeMem(ui32ClientFenceOffsetInt);
	if (ui32ClientFenceValueInt)
		OSFreeMem(ui32ClientFenceValueInt);
	if (psClientUpdateUFOSyncPrimBlockInt)
		OSFreeMem(psClientUpdateUFOSyncPrimBlockInt);
	if (hClientUpdateUFOSyncPrimBlockInt2)
		OSFreeMem(hClientUpdateUFOSyncPrimBlockInt2);
	if (ui32ClientUpdateOffsetInt)
		OSFreeMem(ui32ClientUpdateOffsetInt);
	if (ui32ClientUpdateValueInt)
		OSFreeMem(ui32ClientUpdateValueInt);
	if (ui32ServerSyncFlagsInt)
		OSFreeMem(ui32ServerSyncFlagsInt);
	if (psServerSyncsInt)
		OSFreeMem(psServerSyncsInt);
	if (hServerSyncsInt2)
		OSFreeMem(hServerSyncsInt2);
	if (psDMCmdInt)
		OSFreeMem(psDMCmdInt);

	return 0;
}

static IMG_INT
PVRSRVBridgeRGXFlushComputeData(IMG_UINT32 ui32DispatchTableEntry,
					  PVRSRV_BRIDGE_IN_RGXFLUSHCOMPUTEDATA *psRGXFlushComputeDataIN,
					  PVRSRV_BRIDGE_OUT_RGXFLUSHCOMPUTEDATA *psRGXFlushComputeDataOUT,
					 CONNECTION_DATA *psConnection)
{
	RGX_SERVER_COMPUTE_CONTEXT * psComputeContextInt = NULL;





#if defined(PDUMP)
	PMRLock();
#endif


				{
					/* Look up the address from the handle */
					psRGXFlushComputeDataOUT->eError =
						PVRSRVLookupHandle(psConnection->psHandleBase,
											(void **) &psComputeContextInt,
											psRGXFlushComputeDataIN->hComputeContext,
											PVRSRV_HANDLE_TYPE_RGX_SERVER_COMPUTE_CONTEXT);
					if(psRGXFlushComputeDataOUT->eError != PVRSRV_OK)
					{
#if defined(PDUMP)
						PMRUnlock();
#endif
						goto RGXFlushComputeData_exit;
					}
				}


	psRGXFlushComputeDataOUT->eError =
		PVRSRVRGXFlushComputeDataKM(
					psComputeContextInt);
#if defined(PDUMP)
	PMRUnlock();
#endif




RGXFlushComputeData_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeRGXSetComputeContextPriority(IMG_UINT32 ui32DispatchTableEntry,
					  PVRSRV_BRIDGE_IN_RGXSETCOMPUTECONTEXTPRIORITY *psRGXSetComputeContextPriorityIN,
					  PVRSRV_BRIDGE_OUT_RGXSETCOMPUTECONTEXTPRIORITY *psRGXSetComputeContextPriorityOUT,
					 CONNECTION_DATA *psConnection)
{
	RGX_SERVER_COMPUTE_CONTEXT * psComputeContextInt = NULL;







				{
					/* Look up the address from the handle */
					psRGXSetComputeContextPriorityOUT->eError =
						PVRSRVLookupHandle(psConnection->psHandleBase,
											(void **) &psComputeContextInt,
											psRGXSetComputeContextPriorityIN->hComputeContext,
											PVRSRV_HANDLE_TYPE_RGX_SERVER_COMPUTE_CONTEXT);
					if(psRGXSetComputeContextPriorityOUT->eError != PVRSRV_OK)
					{
						goto RGXSetComputeContextPriority_exit;
					}
				}


	psRGXSetComputeContextPriorityOUT->eError =
		PVRSRVRGXSetComputeContextPriorityKM(psConnection, OSGetDevData(psConnection),
					psComputeContextInt,
					psRGXSetComputeContextPriorityIN->ui32Priority);




RGXSetComputeContextPriority_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeRGXGetLastComputeContextResetReason(IMG_UINT32 ui32DispatchTableEntry,
					  PVRSRV_BRIDGE_IN_RGXGETLASTCOMPUTECONTEXTRESETREASON *psRGXGetLastComputeContextResetReasonIN,
					  PVRSRV_BRIDGE_OUT_RGXGETLASTCOMPUTECONTEXTRESETREASON *psRGXGetLastComputeContextResetReasonOUT,
					 CONNECTION_DATA *psConnection)
{
	RGX_SERVER_COMPUTE_CONTEXT * psComputeContextInt = NULL;







				{
					/* Look up the address from the handle */
					psRGXGetLastComputeContextResetReasonOUT->eError =
						PVRSRVLookupHandle(psConnection->psHandleBase,
											(void **) &psComputeContextInt,
											psRGXGetLastComputeContextResetReasonIN->hComputeContext,
											PVRSRV_HANDLE_TYPE_RGX_SERVER_COMPUTE_CONTEXT);
					if(psRGXGetLastComputeContextResetReasonOUT->eError != PVRSRV_OK)
					{
						goto RGXGetLastComputeContextResetReason_exit;
					}
				}


	psRGXGetLastComputeContextResetReasonOUT->eError =
		PVRSRVRGXGetLastComputeContextResetReasonKM(
					psComputeContextInt,
					&psRGXGetLastComputeContextResetReasonOUT->ui32LastResetReason,
					&psRGXGetLastComputeContextResetReasonOUT->ui32LastResetJobRef);




RGXGetLastComputeContextResetReason_exit:

	return 0;
}



/* *************************************************************************** 
 * Server bridge dispatch related glue 
 */

static IMG_BOOL bUseLock = IMG_TRUE;

PVRSRV_ERROR InitRGXCMPBridge(void);
PVRSRV_ERROR DeinitRGXCMPBridge(void);

/*
 * Register all RGXCMP functions with services
 */
PVRSRV_ERROR InitRGXCMPBridge(void)
{

	SetDispatchTableEntry(PVRSRV_BRIDGE_RGXCMP, PVRSRV_BRIDGE_RGXCMP_RGXCREATECOMPUTECONTEXT, PVRSRVBridgeRGXCreateComputeContext,
					NULL, bUseLock);

	SetDispatchTableEntry(PVRSRV_BRIDGE_RGXCMP, PVRSRV_BRIDGE_RGXCMP_RGXDESTROYCOMPUTECONTEXT, PVRSRVBridgeRGXDestroyComputeContext,
					NULL, bUseLock);

	SetDispatchTableEntry(PVRSRV_BRIDGE_RGXCMP, PVRSRV_BRIDGE_RGXCMP_RGXKICKCDM, PVRSRVBridgeRGXKickCDM,
					NULL, bUseLock);

	SetDispatchTableEntry(PVRSRV_BRIDGE_RGXCMP, PVRSRV_BRIDGE_RGXCMP_RGXFLUSHCOMPUTEDATA, PVRSRVBridgeRGXFlushComputeData,
					NULL, bUseLock);

	SetDispatchTableEntry(PVRSRV_BRIDGE_RGXCMP, PVRSRV_BRIDGE_RGXCMP_RGXSETCOMPUTECONTEXTPRIORITY, PVRSRVBridgeRGXSetComputeContextPriority,
					NULL, bUseLock);

	SetDispatchTableEntry(PVRSRV_BRIDGE_RGXCMP, PVRSRV_BRIDGE_RGXCMP_RGXGETLASTCOMPUTECONTEXTRESETREASON, PVRSRVBridgeRGXGetLastComputeContextResetReason,
					NULL, bUseLock);


	return PVRSRV_OK;
}

/*
 * Unregister all rgxcmp functions with services
 */
PVRSRV_ERROR DeinitRGXCMPBridge(void)
{
	return PVRSRV_OK;
}

