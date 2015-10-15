/*************************************************************************/ /*!
@File
@Title          Server bridge for rgxtq
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Implements the server side of the bridge for rgxtq
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

#include "rgxtransfer.h"


#include "common_rgxtq_bridge.h"

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
PVRSRVBridgeRGXCreateTransferContext(IMG_UINT32 ui32DispatchTableEntry,
					  PVRSRV_BRIDGE_IN_RGXCREATETRANSFERCONTEXT *psRGXCreateTransferContextIN,
					  PVRSRV_BRIDGE_OUT_RGXCREATETRANSFERCONTEXT *psRGXCreateTransferContextOUT,
					 CONNECTION_DATA *psConnection)
{
	IMG_BYTE *psFrameworkCmdInt = NULL;
	IMG_HANDLE hPrivDataInt = NULL;
	RGX_SERVER_TQ_CONTEXT * psTransferContextInt = NULL;




	if (psRGXCreateTransferContextIN->ui32FrameworkCmdize != 0)
	{
		psFrameworkCmdInt = OSAllocMem(psRGXCreateTransferContextIN->ui32FrameworkCmdize * sizeof(IMG_BYTE));
		if (!psFrameworkCmdInt)
		{
			psRGXCreateTransferContextOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;
	
			goto RGXCreateTransferContext_exit;
		}
	}

			/* Copy the data over */
			if ( !OSAccessOK(PVR_VERIFY_READ, (void*) psRGXCreateTransferContextIN->psFrameworkCmd, psRGXCreateTransferContextIN->ui32FrameworkCmdize * sizeof(IMG_BYTE))
				|| (OSCopyFromUser(NULL, psFrameworkCmdInt, psRGXCreateTransferContextIN->psFrameworkCmd,
				psRGXCreateTransferContextIN->ui32FrameworkCmdize * sizeof(IMG_BYTE)) != PVRSRV_OK) )
			{
				psRGXCreateTransferContextOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

				goto RGXCreateTransferContext_exit;
			}

	PMRLock();


				{
					/* Look up the address from the handle */
					psRGXCreateTransferContextOUT->eError =
						PVRSRVLookupHandle(psConnection->psHandleBase,
											(void **) &hPrivDataInt,
											psRGXCreateTransferContextIN->hPrivData,
											PVRSRV_HANDLE_TYPE_DEV_PRIV_DATA);
					if(psRGXCreateTransferContextOUT->eError != PVRSRV_OK)
					{
						PMRUnlock();
						goto RGXCreateTransferContext_exit;
					}
				}


	psRGXCreateTransferContextOUT->eError =
		PVRSRVRGXCreateTransferContextKM(psConnection, OSGetDevData(psConnection),
					psRGXCreateTransferContextIN->ui32Priority,
					psRGXCreateTransferContextIN->sMCUFenceAddr,
					psRGXCreateTransferContextIN->ui32FrameworkCmdize,
					psFrameworkCmdInt,
					hPrivDataInt,
					&psTransferContextInt);
	/* Exit early if bridged call fails */
	if(psRGXCreateTransferContextOUT->eError != PVRSRV_OK)
	{
		PMRUnlock();
		goto RGXCreateTransferContext_exit;
	}
	PMRUnlock();


	psRGXCreateTransferContextOUT->eError = PVRSRVAllocHandle(psConnection->psHandleBase,
							&psRGXCreateTransferContextOUT->hTransferContext,
							(void *) psTransferContextInt,
							PVRSRV_HANDLE_TYPE_RGX_SERVER_TQ_CONTEXT,
							PVRSRV_HANDLE_ALLOC_FLAG_MULTI
							,(PFN_HANDLE_RELEASE)&PVRSRVRGXDestroyTransferContextKM);
	if (psRGXCreateTransferContextOUT->eError != PVRSRV_OK)
	{
		goto RGXCreateTransferContext_exit;
	}




RGXCreateTransferContext_exit:
	if (psRGXCreateTransferContextOUT->eError != PVRSRV_OK)
	{
		if (psTransferContextInt)
		{
			PVRSRVRGXDestroyTransferContextKM(psTransferContextInt);
		}
	}

	if (psFrameworkCmdInt)
		OSFreeMem(psFrameworkCmdInt);

	return 0;
}

static IMG_INT
PVRSRVBridgeRGXDestroyTransferContext(IMG_UINT32 ui32DispatchTableEntry,
					  PVRSRV_BRIDGE_IN_RGXDESTROYTRANSFERCONTEXT *psRGXDestroyTransferContextIN,
					  PVRSRV_BRIDGE_OUT_RGXDESTROYTRANSFERCONTEXT *psRGXDestroyTransferContextOUT,
					 CONNECTION_DATA *psConnection)
{





	PMRLock();




	psRGXDestroyTransferContextOUT->eError =
		PVRSRVReleaseHandle(psConnection->psHandleBase,
					(IMG_HANDLE) psRGXDestroyTransferContextIN->hTransferContext,
					PVRSRV_HANDLE_TYPE_RGX_SERVER_TQ_CONTEXT);
	if ((psRGXDestroyTransferContextOUT->eError != PVRSRV_OK) && (psRGXDestroyTransferContextOUT->eError != PVRSRV_ERROR_RETRY))
	{
		PVR_ASSERT(0);
		PMRUnlock();
		goto RGXDestroyTransferContext_exit;
	}

	PMRUnlock();


RGXDestroyTransferContext_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeRGXSubmitTransfer(IMG_UINT32 ui32DispatchTableEntry,
					  PVRSRV_BRIDGE_IN_RGXSUBMITTRANSFER *psRGXSubmitTransferIN,
					  PVRSRV_BRIDGE_OUT_RGXSUBMITTRANSFER *psRGXSubmitTransferOUT,
					 CONNECTION_DATA *psConnection)
{
	RGX_SERVER_TQ_CONTEXT * psTransferContextInt = NULL;
	IMG_UINT32 *ui32ClientFenceCountInt = NULL;
	SYNC_PRIMITIVE_BLOCK * **psFenceUFOSyncPrimBlockInt = NULL;
	IMG_HANDLE **hFenceUFOSyncPrimBlockInt2 = NULL;
	IMG_UINT32 **ui32FenceSyncOffsetInt = NULL;
	IMG_UINT32 **ui32FenceValueInt = NULL;
	IMG_UINT32 *ui32ClientUpdateCountInt = NULL;
	SYNC_PRIMITIVE_BLOCK * **psUpdateUFOSyncPrimBlockInt = NULL;
	IMG_HANDLE **hUpdateUFOSyncPrimBlockInt2 = NULL;
	IMG_UINT32 **ui32UpdateSyncOffsetInt = NULL;
	IMG_UINT32 **ui32UpdateValueInt = NULL;
	IMG_UINT32 *ui32ServerSyncCountInt = NULL;
	IMG_UINT32 **ui32ServerSyncFlagsInt = NULL;
	SERVER_SYNC_PRIMITIVE * **psServerSyncInt = NULL;
	IMG_HANDLE **hServerSyncInt2 = NULL;
	IMG_CHAR *uiUpdateFenceNameInt = NULL;
	IMG_UINT32 *ui32CommandSizeInt = NULL;
	IMG_UINT8 **ui8FWCommandInt = NULL;
	IMG_UINT32 *ui32TQPrepareFlagsInt = NULL;
	IMG_UINT32 *ui32SyncPMRFlagsInt = NULL;
	PMR * *psSyncPMRsInt = NULL;
	IMG_HANDLE *hSyncPMRsInt2 = NULL;




	if (psRGXSubmitTransferIN->ui32PrepareCount != 0)
	{
		ui32ClientFenceCountInt = OSAllocMem(psRGXSubmitTransferIN->ui32PrepareCount * sizeof(IMG_UINT32));
		if (!ui32ClientFenceCountInt)
		{
			psRGXSubmitTransferOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;
	
			goto RGXSubmitTransfer_exit;
		}
	}

			/* Copy the data over */
			if ( !OSAccessOK(PVR_VERIFY_READ, (void*) psRGXSubmitTransferIN->pui32ClientFenceCount, psRGXSubmitTransferIN->ui32PrepareCount * sizeof(IMG_UINT32))
				|| (OSCopyFromUser(NULL, ui32ClientFenceCountInt, psRGXSubmitTransferIN->pui32ClientFenceCount,
				psRGXSubmitTransferIN->ui32PrepareCount * sizeof(IMG_UINT32)) != PVRSRV_OK) )
			{
				psRGXSubmitTransferOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

				goto RGXSubmitTransfer_exit;
			}
	if (psRGXSubmitTransferIN->ui32PrepareCount != 0)
	{
		IMG_UINT32 ui32Pass=0;
		IMG_UINT32 i;
		IMG_UINT32 ui32AllocSize=0;
		IMG_UINT32 ui32Size;
		IMG_UINT8 *pui8Ptr = NULL;
		IMG_UINT32 ui32AllocSize2=0;
		IMG_UINT32 ui32Size2;
		IMG_UINT8 *pui8Ptr2 = NULL;

		/*
			Two pass loop, 1st find out the size and 2nd allocation and set offsets.
			Keeps allocation cost down and simplifies the free path
		*/
		for (ui32Pass=0;ui32Pass<2;ui32Pass++)
		{
			ui32Size = psRGXSubmitTransferIN->ui32PrepareCount * sizeof(SYNC_PRIMITIVE_BLOCK * *);
			ui32Size2 = psRGXSubmitTransferIN->ui32PrepareCount * sizeof(IMG_HANDLE *);
			if (ui32Pass == 0)
			{
				ui32AllocSize += ui32Size;
				ui32AllocSize2 += ui32Size2;
			}
			else
			{
				pui8Ptr = OSAllocMem(ui32AllocSize);
				if (pui8Ptr == NULL)
				{
					psRGXSubmitTransferOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;
					goto RGXSubmitTransfer_exit;
				}
				psFenceUFOSyncPrimBlockInt = (SYNC_PRIMITIVE_BLOCK * **) pui8Ptr;
				pui8Ptr += ui32Size;
				pui8Ptr2 = OSAllocMem(ui32AllocSize2);
				if (pui8Ptr2 == NULL)
				{
					psRGXSubmitTransferOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;
					goto RGXSubmitTransfer_exit;
				}
				hFenceUFOSyncPrimBlockInt2 = (IMG_HANDLE **) pui8Ptr2;
				pui8Ptr2 += ui32Size2;
			}
			
			for (i=0;i<psRGXSubmitTransferIN->ui32PrepareCount;i++)
			{
				ui32Size = ui32ClientFenceCountInt[i] * sizeof(SYNC_PRIMITIVE_BLOCK *);		
				ui32Size2 = ui32ClientFenceCountInt[i] * sizeof(IMG_HANDLE);		
				if (ui32Size)
				{
					if (ui32Pass == 0)
					{
						ui32AllocSize += ui32Size;
						ui32AllocSize2 += ui32Size2;
					}
					else
					{
						psFenceUFOSyncPrimBlockInt[i] = (SYNC_PRIMITIVE_BLOCK * *) pui8Ptr;
						pui8Ptr += ui32Size;
						hFenceUFOSyncPrimBlockInt2[i] = (IMG_HANDLE *) pui8Ptr2;
						pui8Ptr2 += ui32Size2;
					}
				}
			}
		}
	}

	{
		IMG_UINT32 i;
		IMG_HANDLE **psPtr;

		/* Loop over all the pointers in the array copying the data into the kernel */
		for (i=0;i<psRGXSubmitTransferIN->ui32PrepareCount;i++)
		{
			/* Copy the pointer over from the client side */
			if ( !OSAccessOK(PVR_VERIFY_READ, (void*) &psRGXSubmitTransferIN->phFenceUFOSyncPrimBlock[i], sizeof(IMG_HANDLE **))
				|| (OSCopyFromUser(NULL, &psPtr, &psRGXSubmitTransferIN->phFenceUFOSyncPrimBlock[i],
				sizeof(IMG_HANDLE **)) != PVRSRV_OK) )
			{
				psRGXSubmitTransferOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

				goto RGXSubmitTransfer_exit;
			}
			/* Copy the data over */
			if ( !OSAccessOK(PVR_VERIFY_READ, (void*) psPtr, (ui32ClientFenceCountInt[i] * sizeof(IMG_HANDLE)))
				|| (OSCopyFromUser(NULL, (hFenceUFOSyncPrimBlockInt2[i]), psPtr,
				(ui32ClientFenceCountInt[i] * sizeof(IMG_HANDLE))) != PVRSRV_OK) )
			{
				psRGXSubmitTransferOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

				goto RGXSubmitTransfer_exit;
			}
		}
	}
	if (psRGXSubmitTransferIN->ui32PrepareCount != 0)
	{
		IMG_UINT32 ui32Pass=0;
		IMG_UINT32 i;
		IMG_UINT32 ui32AllocSize=0;
		IMG_UINT32 ui32Size;
		IMG_UINT8 *pui8Ptr = NULL;

		/*
			Two pass loop, 1st find out the size and 2nd allocation and set offsets.
			Keeps allocation cost down and simplifies the free path
		*/
		for (ui32Pass=0;ui32Pass<2;ui32Pass++)
		{
			ui32Size = psRGXSubmitTransferIN->ui32PrepareCount * sizeof(IMG_UINT32 *);
			if (ui32Pass == 0)
			{
				ui32AllocSize += ui32Size;
			}
			else
			{
				pui8Ptr = OSAllocMem(ui32AllocSize);
				if (pui8Ptr == NULL)
				{
					psRGXSubmitTransferOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;
					goto RGXSubmitTransfer_exit;
				}
				ui32FenceSyncOffsetInt = (IMG_UINT32 **) pui8Ptr;
				pui8Ptr += ui32Size;
			}
			
			for (i=0;i<psRGXSubmitTransferIN->ui32PrepareCount;i++)
			{
				ui32Size = ui32ClientFenceCountInt[i] * sizeof(IMG_UINT32);		
				if (ui32Size)
				{
					if (ui32Pass == 0)
					{
						ui32AllocSize += ui32Size;
					}
					else
					{
						ui32FenceSyncOffsetInt[i] = (IMG_UINT32 *) pui8Ptr;
						pui8Ptr += ui32Size;
					}
				}
			}
		}
	}

	{
		IMG_UINT32 i;
		IMG_UINT32 **psPtr;

		/* Loop over all the pointers in the array copying the data into the kernel */
		for (i=0;i<psRGXSubmitTransferIN->ui32PrepareCount;i++)
		{
			/* Copy the pointer over from the client side */
			if ( !OSAccessOK(PVR_VERIFY_READ, (void*) &psRGXSubmitTransferIN->pui32FenceSyncOffset[i], sizeof(IMG_UINT32 **))
				|| (OSCopyFromUser(NULL, &psPtr, &psRGXSubmitTransferIN->pui32FenceSyncOffset[i],
				sizeof(IMG_UINT32 **)) != PVRSRV_OK) )
			{
				psRGXSubmitTransferOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

				goto RGXSubmitTransfer_exit;
			}
			/* Copy the data over */
			if ( !OSAccessOK(PVR_VERIFY_READ, (void*) psPtr, (ui32ClientFenceCountInt[i] * sizeof(IMG_UINT32)))
				|| (OSCopyFromUser(NULL, (ui32FenceSyncOffsetInt[i]), psPtr,
				(ui32ClientFenceCountInt[i] * sizeof(IMG_UINT32))) != PVRSRV_OK) )
			{
				psRGXSubmitTransferOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

				goto RGXSubmitTransfer_exit;
			}
		}
	}
	if (psRGXSubmitTransferIN->ui32PrepareCount != 0)
	{
		IMG_UINT32 ui32Pass=0;
		IMG_UINT32 i;
		IMG_UINT32 ui32AllocSize=0;
		IMG_UINT32 ui32Size;
		IMG_UINT8 *pui8Ptr = NULL;

		/*
			Two pass loop, 1st find out the size and 2nd allocation and set offsets.
			Keeps allocation cost down and simplifies the free path
		*/
		for (ui32Pass=0;ui32Pass<2;ui32Pass++)
		{
			ui32Size = psRGXSubmitTransferIN->ui32PrepareCount * sizeof(IMG_UINT32 *);
			if (ui32Pass == 0)
			{
				ui32AllocSize += ui32Size;
			}
			else
			{
				pui8Ptr = OSAllocMem(ui32AllocSize);
				if (pui8Ptr == NULL)
				{
					psRGXSubmitTransferOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;
					goto RGXSubmitTransfer_exit;
				}
				ui32FenceValueInt = (IMG_UINT32 **) pui8Ptr;
				pui8Ptr += ui32Size;
			}
			
			for (i=0;i<psRGXSubmitTransferIN->ui32PrepareCount;i++)
			{
				ui32Size = ui32ClientFenceCountInt[i] * sizeof(IMG_UINT32);		
				if (ui32Size)
				{
					if (ui32Pass == 0)
					{
						ui32AllocSize += ui32Size;
					}
					else
					{
						ui32FenceValueInt[i] = (IMG_UINT32 *) pui8Ptr;
						pui8Ptr += ui32Size;
					}
				}
			}
		}
	}

	{
		IMG_UINT32 i;
		IMG_UINT32 **psPtr;

		/* Loop over all the pointers in the array copying the data into the kernel */
		for (i=0;i<psRGXSubmitTransferIN->ui32PrepareCount;i++)
		{
			/* Copy the pointer over from the client side */
			if ( !OSAccessOK(PVR_VERIFY_READ, (void*) &psRGXSubmitTransferIN->pui32FenceValue[i], sizeof(IMG_UINT32 **))
				|| (OSCopyFromUser(NULL, &psPtr, &psRGXSubmitTransferIN->pui32FenceValue[i],
				sizeof(IMG_UINT32 **)) != PVRSRV_OK) )
			{
				psRGXSubmitTransferOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

				goto RGXSubmitTransfer_exit;
			}
			/* Copy the data over */
			if ( !OSAccessOK(PVR_VERIFY_READ, (void*) psPtr, (ui32ClientFenceCountInt[i] * sizeof(IMG_UINT32)))
				|| (OSCopyFromUser(NULL, (ui32FenceValueInt[i]), psPtr,
				(ui32ClientFenceCountInt[i] * sizeof(IMG_UINT32))) != PVRSRV_OK) )
			{
				psRGXSubmitTransferOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

				goto RGXSubmitTransfer_exit;
			}
		}
	}
	if (psRGXSubmitTransferIN->ui32PrepareCount != 0)
	{
		ui32ClientUpdateCountInt = OSAllocMem(psRGXSubmitTransferIN->ui32PrepareCount * sizeof(IMG_UINT32));
		if (!ui32ClientUpdateCountInt)
		{
			psRGXSubmitTransferOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;
	
			goto RGXSubmitTransfer_exit;
		}
	}

			/* Copy the data over */
			if ( !OSAccessOK(PVR_VERIFY_READ, (void*) psRGXSubmitTransferIN->pui32ClientUpdateCount, psRGXSubmitTransferIN->ui32PrepareCount * sizeof(IMG_UINT32))
				|| (OSCopyFromUser(NULL, ui32ClientUpdateCountInt, psRGXSubmitTransferIN->pui32ClientUpdateCount,
				psRGXSubmitTransferIN->ui32PrepareCount * sizeof(IMG_UINT32)) != PVRSRV_OK) )
			{
				psRGXSubmitTransferOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

				goto RGXSubmitTransfer_exit;
			}
	if (psRGXSubmitTransferIN->ui32PrepareCount != 0)
	{
		IMG_UINT32 ui32Pass=0;
		IMG_UINT32 i;
		IMG_UINT32 ui32AllocSize=0;
		IMG_UINT32 ui32Size;
		IMG_UINT8 *pui8Ptr = NULL;
		IMG_UINT32 ui32AllocSize2=0;
		IMG_UINT32 ui32Size2;
		IMG_UINT8 *pui8Ptr2 = NULL;

		/*
			Two pass loop, 1st find out the size and 2nd allocation and set offsets.
			Keeps allocation cost down and simplifies the free path
		*/
		for (ui32Pass=0;ui32Pass<2;ui32Pass++)
		{
			ui32Size = psRGXSubmitTransferIN->ui32PrepareCount * sizeof(SYNC_PRIMITIVE_BLOCK * *);
			ui32Size2 = psRGXSubmitTransferIN->ui32PrepareCount * sizeof(IMG_HANDLE *);
			if (ui32Pass == 0)
			{
				ui32AllocSize += ui32Size;
				ui32AllocSize2 += ui32Size2;
			}
			else
			{
				pui8Ptr = OSAllocMem(ui32AllocSize);
				if (pui8Ptr == NULL)
				{
					psRGXSubmitTransferOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;
					goto RGXSubmitTransfer_exit;
				}
				psUpdateUFOSyncPrimBlockInt = (SYNC_PRIMITIVE_BLOCK * **) pui8Ptr;
				pui8Ptr += ui32Size;
				pui8Ptr2 = OSAllocMem(ui32AllocSize2);
				if (pui8Ptr2 == NULL)
				{
					psRGXSubmitTransferOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;
					goto RGXSubmitTransfer_exit;
				}
				hUpdateUFOSyncPrimBlockInt2 = (IMG_HANDLE **) pui8Ptr2;
				pui8Ptr2 += ui32Size2;
			}
			
			for (i=0;i<psRGXSubmitTransferIN->ui32PrepareCount;i++)
			{
				ui32Size = ui32ClientUpdateCountInt[i] * sizeof(SYNC_PRIMITIVE_BLOCK *);		
				ui32Size2 = ui32ClientUpdateCountInt[i] * sizeof(IMG_HANDLE);		
				if (ui32Size)
				{
					if (ui32Pass == 0)
					{
						ui32AllocSize += ui32Size;
						ui32AllocSize2 += ui32Size2;
					}
					else
					{
						psUpdateUFOSyncPrimBlockInt[i] = (SYNC_PRIMITIVE_BLOCK * *) pui8Ptr;
						pui8Ptr += ui32Size;
						hUpdateUFOSyncPrimBlockInt2[i] = (IMG_HANDLE *) pui8Ptr2;
						pui8Ptr2 += ui32Size2;
					}
				}
			}
		}
	}

	{
		IMG_UINT32 i;
		IMG_HANDLE **psPtr;

		/* Loop over all the pointers in the array copying the data into the kernel */
		for (i=0;i<psRGXSubmitTransferIN->ui32PrepareCount;i++)
		{
			/* Copy the pointer over from the client side */
			if ( !OSAccessOK(PVR_VERIFY_READ, (void*) &psRGXSubmitTransferIN->phUpdateUFOSyncPrimBlock[i], sizeof(IMG_HANDLE **))
				|| (OSCopyFromUser(NULL, &psPtr, &psRGXSubmitTransferIN->phUpdateUFOSyncPrimBlock[i],
				sizeof(IMG_HANDLE **)) != PVRSRV_OK) )
			{
				psRGXSubmitTransferOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

				goto RGXSubmitTransfer_exit;
			}
			/* Copy the data over */
			if ( !OSAccessOK(PVR_VERIFY_READ, (void*) psPtr, (ui32ClientUpdateCountInt[i] * sizeof(IMG_HANDLE)))
				|| (OSCopyFromUser(NULL, (hUpdateUFOSyncPrimBlockInt2[i]), psPtr,
				(ui32ClientUpdateCountInt[i] * sizeof(IMG_HANDLE))) != PVRSRV_OK) )
			{
				psRGXSubmitTransferOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

				goto RGXSubmitTransfer_exit;
			}
		}
	}
	if (psRGXSubmitTransferIN->ui32PrepareCount != 0)
	{
		IMG_UINT32 ui32Pass=0;
		IMG_UINT32 i;
		IMG_UINT32 ui32AllocSize=0;
		IMG_UINT32 ui32Size;
		IMG_UINT8 *pui8Ptr = NULL;

		/*
			Two pass loop, 1st find out the size and 2nd allocation and set offsets.
			Keeps allocation cost down and simplifies the free path
		*/
		for (ui32Pass=0;ui32Pass<2;ui32Pass++)
		{
			ui32Size = psRGXSubmitTransferIN->ui32PrepareCount * sizeof(IMG_UINT32 *);
			if (ui32Pass == 0)
			{
				ui32AllocSize += ui32Size;
			}
			else
			{
				pui8Ptr = OSAllocMem(ui32AllocSize);
				if (pui8Ptr == NULL)
				{
					psRGXSubmitTransferOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;
					goto RGXSubmitTransfer_exit;
				}
				ui32UpdateSyncOffsetInt = (IMG_UINT32 **) pui8Ptr;
				pui8Ptr += ui32Size;
			}
			
			for (i=0;i<psRGXSubmitTransferIN->ui32PrepareCount;i++)
			{
				ui32Size = ui32ClientUpdateCountInt[i] * sizeof(IMG_UINT32);		
				if (ui32Size)
				{
					if (ui32Pass == 0)
					{
						ui32AllocSize += ui32Size;
					}
					else
					{
						ui32UpdateSyncOffsetInt[i] = (IMG_UINT32 *) pui8Ptr;
						pui8Ptr += ui32Size;
					}
				}
			}
		}
	}

	{
		IMG_UINT32 i;
		IMG_UINT32 **psPtr;

		/* Loop over all the pointers in the array copying the data into the kernel */
		for (i=0;i<psRGXSubmitTransferIN->ui32PrepareCount;i++)
		{
			/* Copy the pointer over from the client side */
			if ( !OSAccessOK(PVR_VERIFY_READ, (void*) &psRGXSubmitTransferIN->pui32UpdateSyncOffset[i], sizeof(IMG_UINT32 **))
				|| (OSCopyFromUser(NULL, &psPtr, &psRGXSubmitTransferIN->pui32UpdateSyncOffset[i],
				sizeof(IMG_UINT32 **)) != PVRSRV_OK) )
			{
				psRGXSubmitTransferOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

				goto RGXSubmitTransfer_exit;
			}
			/* Copy the data over */
			if ( !OSAccessOK(PVR_VERIFY_READ, (void*) psPtr, (ui32ClientUpdateCountInt[i] * sizeof(IMG_UINT32)))
				|| (OSCopyFromUser(NULL, (ui32UpdateSyncOffsetInt[i]), psPtr,
				(ui32ClientUpdateCountInt[i] * sizeof(IMG_UINT32))) != PVRSRV_OK) )
			{
				psRGXSubmitTransferOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

				goto RGXSubmitTransfer_exit;
			}
		}
	}
	if (psRGXSubmitTransferIN->ui32PrepareCount != 0)
	{
		IMG_UINT32 ui32Pass=0;
		IMG_UINT32 i;
		IMG_UINT32 ui32AllocSize=0;
		IMG_UINT32 ui32Size;
		IMG_UINT8 *pui8Ptr = NULL;

		/*
			Two pass loop, 1st find out the size and 2nd allocation and set offsets.
			Keeps allocation cost down and simplifies the free path
		*/
		for (ui32Pass=0;ui32Pass<2;ui32Pass++)
		{
			ui32Size = psRGXSubmitTransferIN->ui32PrepareCount * sizeof(IMG_UINT32 *);
			if (ui32Pass == 0)
			{
				ui32AllocSize += ui32Size;
			}
			else
			{
				pui8Ptr = OSAllocMem(ui32AllocSize);
				if (pui8Ptr == NULL)
				{
					psRGXSubmitTransferOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;
					goto RGXSubmitTransfer_exit;
				}
				ui32UpdateValueInt = (IMG_UINT32 **) pui8Ptr;
				pui8Ptr += ui32Size;
			}
			
			for (i=0;i<psRGXSubmitTransferIN->ui32PrepareCount;i++)
			{
				ui32Size = ui32ClientUpdateCountInt[i] * sizeof(IMG_UINT32);		
				if (ui32Size)
				{
					if (ui32Pass == 0)
					{
						ui32AllocSize += ui32Size;
					}
					else
					{
						ui32UpdateValueInt[i] = (IMG_UINT32 *) pui8Ptr;
						pui8Ptr += ui32Size;
					}
				}
			}
		}
	}

	{
		IMG_UINT32 i;
		IMG_UINT32 **psPtr;

		/* Loop over all the pointers in the array copying the data into the kernel */
		for (i=0;i<psRGXSubmitTransferIN->ui32PrepareCount;i++)
		{
			/* Copy the pointer over from the client side */
			if ( !OSAccessOK(PVR_VERIFY_READ, (void*) &psRGXSubmitTransferIN->pui32UpdateValue[i], sizeof(IMG_UINT32 **))
				|| (OSCopyFromUser(NULL, &psPtr, &psRGXSubmitTransferIN->pui32UpdateValue[i],
				sizeof(IMG_UINT32 **)) != PVRSRV_OK) )
			{
				psRGXSubmitTransferOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

				goto RGXSubmitTransfer_exit;
			}
			/* Copy the data over */
			if ( !OSAccessOK(PVR_VERIFY_READ, (void*) psPtr, (ui32ClientUpdateCountInt[i] * sizeof(IMG_UINT32)))
				|| (OSCopyFromUser(NULL, (ui32UpdateValueInt[i]), psPtr,
				(ui32ClientUpdateCountInt[i] * sizeof(IMG_UINT32))) != PVRSRV_OK) )
			{
				psRGXSubmitTransferOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

				goto RGXSubmitTransfer_exit;
			}
		}
	}
	if (psRGXSubmitTransferIN->ui32PrepareCount != 0)
	{
		ui32ServerSyncCountInt = OSAllocMem(psRGXSubmitTransferIN->ui32PrepareCount * sizeof(IMG_UINT32));
		if (!ui32ServerSyncCountInt)
		{
			psRGXSubmitTransferOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;
	
			goto RGXSubmitTransfer_exit;
		}
	}

			/* Copy the data over */
			if ( !OSAccessOK(PVR_VERIFY_READ, (void*) psRGXSubmitTransferIN->pui32ServerSyncCount, psRGXSubmitTransferIN->ui32PrepareCount * sizeof(IMG_UINT32))
				|| (OSCopyFromUser(NULL, ui32ServerSyncCountInt, psRGXSubmitTransferIN->pui32ServerSyncCount,
				psRGXSubmitTransferIN->ui32PrepareCount * sizeof(IMG_UINT32)) != PVRSRV_OK) )
			{
				psRGXSubmitTransferOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

				goto RGXSubmitTransfer_exit;
			}
	if (psRGXSubmitTransferIN->ui32PrepareCount != 0)
	{
		IMG_UINT32 ui32Pass=0;
		IMG_UINT32 i;
		IMG_UINT32 ui32AllocSize=0;
		IMG_UINT32 ui32Size;
		IMG_UINT8 *pui8Ptr = NULL;

		/*
			Two pass loop, 1st find out the size and 2nd allocation and set offsets.
			Keeps allocation cost down and simplifies the free path
		*/
		for (ui32Pass=0;ui32Pass<2;ui32Pass++)
		{
			ui32Size = psRGXSubmitTransferIN->ui32PrepareCount * sizeof(IMG_UINT32 *);
			if (ui32Pass == 0)
			{
				ui32AllocSize += ui32Size;
			}
			else
			{
				pui8Ptr = OSAllocMem(ui32AllocSize);
				if (pui8Ptr == NULL)
				{
					psRGXSubmitTransferOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;
					goto RGXSubmitTransfer_exit;
				}
				ui32ServerSyncFlagsInt = (IMG_UINT32 **) pui8Ptr;
				pui8Ptr += ui32Size;
			}
			
			for (i=0;i<psRGXSubmitTransferIN->ui32PrepareCount;i++)
			{
				ui32Size = ui32ServerSyncCountInt[i] * sizeof(IMG_UINT32);		
				if (ui32Size)
				{
					if (ui32Pass == 0)
					{
						ui32AllocSize += ui32Size;
					}
					else
					{
						ui32ServerSyncFlagsInt[i] = (IMG_UINT32 *) pui8Ptr;
						pui8Ptr += ui32Size;
					}
				}
			}
		}
	}

	{
		IMG_UINT32 i;
		IMG_UINT32 **psPtr;

		/* Loop over all the pointers in the array copying the data into the kernel */
		for (i=0;i<psRGXSubmitTransferIN->ui32PrepareCount;i++)
		{
			/* Copy the pointer over from the client side */
			if ( !OSAccessOK(PVR_VERIFY_READ, (void*) &psRGXSubmitTransferIN->pui32ServerSyncFlags[i], sizeof(IMG_UINT32 **))
				|| (OSCopyFromUser(NULL, &psPtr, &psRGXSubmitTransferIN->pui32ServerSyncFlags[i],
				sizeof(IMG_UINT32 **)) != PVRSRV_OK) )
			{
				psRGXSubmitTransferOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

				goto RGXSubmitTransfer_exit;
			}
			/* Copy the data over */
			if ( !OSAccessOK(PVR_VERIFY_READ, (void*) psPtr, (ui32ServerSyncCountInt[i] * sizeof(IMG_UINT32)))
				|| (OSCopyFromUser(NULL, (ui32ServerSyncFlagsInt[i]), psPtr,
				(ui32ServerSyncCountInt[i] * sizeof(IMG_UINT32))) != PVRSRV_OK) )
			{
				psRGXSubmitTransferOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

				goto RGXSubmitTransfer_exit;
			}
		}
	}
	if (psRGXSubmitTransferIN->ui32PrepareCount != 0)
	{
		IMG_UINT32 ui32Pass=0;
		IMG_UINT32 i;
		IMG_UINT32 ui32AllocSize=0;
		IMG_UINT32 ui32Size;
		IMG_UINT8 *pui8Ptr = NULL;
		IMG_UINT32 ui32AllocSize2=0;
		IMG_UINT32 ui32Size2;
		IMG_UINT8 *pui8Ptr2 = NULL;

		/*
			Two pass loop, 1st find out the size and 2nd allocation and set offsets.
			Keeps allocation cost down and simplifies the free path
		*/
		for (ui32Pass=0;ui32Pass<2;ui32Pass++)
		{
			ui32Size = psRGXSubmitTransferIN->ui32PrepareCount * sizeof(SERVER_SYNC_PRIMITIVE * *);
			ui32Size2 = psRGXSubmitTransferIN->ui32PrepareCount * sizeof(IMG_HANDLE *);
			if (ui32Pass == 0)
			{
				ui32AllocSize += ui32Size;
				ui32AllocSize2 += ui32Size2;
			}
			else
			{
				pui8Ptr = OSAllocMem(ui32AllocSize);
				if (pui8Ptr == NULL)
				{
					psRGXSubmitTransferOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;
					goto RGXSubmitTransfer_exit;
				}
				psServerSyncInt = (SERVER_SYNC_PRIMITIVE * **) pui8Ptr;
				pui8Ptr += ui32Size;
				pui8Ptr2 = OSAllocMem(ui32AllocSize2);
				if (pui8Ptr2 == NULL)
				{
					psRGXSubmitTransferOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;
					goto RGXSubmitTransfer_exit;
				}
				hServerSyncInt2 = (IMG_HANDLE **) pui8Ptr2;
				pui8Ptr2 += ui32Size2;
			}
			
			for (i=0;i<psRGXSubmitTransferIN->ui32PrepareCount;i++)
			{
				ui32Size = ui32ServerSyncCountInt[i] * sizeof(SERVER_SYNC_PRIMITIVE *);		
				ui32Size2 = ui32ServerSyncCountInt[i] * sizeof(IMG_HANDLE);		
				if (ui32Size)
				{
					if (ui32Pass == 0)
					{
						ui32AllocSize += ui32Size;
						ui32AllocSize2 += ui32Size2;
					}
					else
					{
						psServerSyncInt[i] = (SERVER_SYNC_PRIMITIVE * *) pui8Ptr;
						pui8Ptr += ui32Size;
						hServerSyncInt2[i] = (IMG_HANDLE *) pui8Ptr2;
						pui8Ptr2 += ui32Size2;
					}
				}
			}
		}
	}

	{
		IMG_UINT32 i;
		IMG_HANDLE **psPtr;

		/* Loop over all the pointers in the array copying the data into the kernel */
		for (i=0;i<psRGXSubmitTransferIN->ui32PrepareCount;i++)
		{
			/* Copy the pointer over from the client side */
			if ( !OSAccessOK(PVR_VERIFY_READ, (void*) &psRGXSubmitTransferIN->phServerSync[i], sizeof(IMG_HANDLE **))
				|| (OSCopyFromUser(NULL, &psPtr, &psRGXSubmitTransferIN->phServerSync[i],
				sizeof(IMG_HANDLE **)) != PVRSRV_OK) )
			{
				psRGXSubmitTransferOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

				goto RGXSubmitTransfer_exit;
			}
			/* Copy the data over */
			if ( !OSAccessOK(PVR_VERIFY_READ, (void*) psPtr, (ui32ServerSyncCountInt[i] * sizeof(IMG_HANDLE)))
				|| (OSCopyFromUser(NULL, (hServerSyncInt2[i]), psPtr,
				(ui32ServerSyncCountInt[i] * sizeof(IMG_HANDLE))) != PVRSRV_OK) )
			{
				psRGXSubmitTransferOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

				goto RGXSubmitTransfer_exit;
			}
		}
	}
	
	{
		uiUpdateFenceNameInt = OSAllocMem(32 * sizeof(IMG_CHAR));
		if (!uiUpdateFenceNameInt)
		{
			psRGXSubmitTransferOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;
	
			goto RGXSubmitTransfer_exit;
		}
	}

			/* Copy the data over */
			if ( !OSAccessOK(PVR_VERIFY_READ, (void*) psRGXSubmitTransferIN->puiUpdateFenceName, 32 * sizeof(IMG_CHAR))
				|| (OSCopyFromUser(NULL, uiUpdateFenceNameInt, psRGXSubmitTransferIN->puiUpdateFenceName,
				32 * sizeof(IMG_CHAR)) != PVRSRV_OK) )
			{
				psRGXSubmitTransferOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

				goto RGXSubmitTransfer_exit;
			}
	if (psRGXSubmitTransferIN->ui32PrepareCount != 0)
	{
		ui32CommandSizeInt = OSAllocMem(psRGXSubmitTransferIN->ui32PrepareCount * sizeof(IMG_UINT32));
		if (!ui32CommandSizeInt)
		{
			psRGXSubmitTransferOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;
	
			goto RGXSubmitTransfer_exit;
		}
	}

			/* Copy the data over */
			if ( !OSAccessOK(PVR_VERIFY_READ, (void*) psRGXSubmitTransferIN->pui32CommandSize, psRGXSubmitTransferIN->ui32PrepareCount * sizeof(IMG_UINT32))
				|| (OSCopyFromUser(NULL, ui32CommandSizeInt, psRGXSubmitTransferIN->pui32CommandSize,
				psRGXSubmitTransferIN->ui32PrepareCount * sizeof(IMG_UINT32)) != PVRSRV_OK) )
			{
				psRGXSubmitTransferOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

				goto RGXSubmitTransfer_exit;
			}
	if (psRGXSubmitTransferIN->ui32PrepareCount != 0)
	{
		IMG_UINT32 ui32Pass=0;
		IMG_UINT32 i;
		IMG_UINT32 ui32AllocSize=0;
		IMG_UINT32 ui32Size;
		IMG_UINT8 *pui8Ptr = NULL;

		/*
			Two pass loop, 1st find out the size and 2nd allocation and set offsets.
			Keeps allocation cost down and simplifies the free path
		*/
		for (ui32Pass=0;ui32Pass<2;ui32Pass++)
		{
			ui32Size = psRGXSubmitTransferIN->ui32PrepareCount * sizeof(IMG_UINT8 *);
			if (ui32Pass == 0)
			{
				ui32AllocSize += ui32Size;
			}
			else
			{
				pui8Ptr = OSAllocMem(ui32AllocSize);
				if (pui8Ptr == NULL)
				{
					psRGXSubmitTransferOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;
					goto RGXSubmitTransfer_exit;
				}
				ui8FWCommandInt = (IMG_UINT8 **) pui8Ptr;
				pui8Ptr += ui32Size;
			}
			
			for (i=0;i<psRGXSubmitTransferIN->ui32PrepareCount;i++)
			{
				ui32Size = ui32CommandSizeInt[i] * sizeof(IMG_UINT8);		
				if (ui32Size)
				{
					if (ui32Pass == 0)
					{
						ui32AllocSize += ui32Size;
					}
					else
					{
						ui8FWCommandInt[i] = (IMG_UINT8 *) pui8Ptr;
						pui8Ptr += ui32Size;
					}
				}
			}
		}
	}

	{
		IMG_UINT32 i;
		IMG_UINT8 **psPtr;

		/* Loop over all the pointers in the array copying the data into the kernel */
		for (i=0;i<psRGXSubmitTransferIN->ui32PrepareCount;i++)
		{
			/* Copy the pointer over from the client side */
			if ( !OSAccessOK(PVR_VERIFY_READ, (void*) &psRGXSubmitTransferIN->pui8FWCommand[i], sizeof(IMG_UINT8 **))
				|| (OSCopyFromUser(NULL, &psPtr, &psRGXSubmitTransferIN->pui8FWCommand[i],
				sizeof(IMG_UINT8 **)) != PVRSRV_OK) )
			{
				psRGXSubmitTransferOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

				goto RGXSubmitTransfer_exit;
			}
			/* Copy the data over */
			if ( !OSAccessOK(PVR_VERIFY_READ, (void*) psPtr, (ui32CommandSizeInt[i] * sizeof(IMG_UINT8)))
				|| (OSCopyFromUser(NULL, (ui8FWCommandInt[i]), psPtr,
				(ui32CommandSizeInt[i] * sizeof(IMG_UINT8))) != PVRSRV_OK) )
			{
				psRGXSubmitTransferOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

				goto RGXSubmitTransfer_exit;
			}
		}
	}
	if (psRGXSubmitTransferIN->ui32PrepareCount != 0)
	{
		ui32TQPrepareFlagsInt = OSAllocMem(psRGXSubmitTransferIN->ui32PrepareCount * sizeof(IMG_UINT32));
		if (!ui32TQPrepareFlagsInt)
		{
			psRGXSubmitTransferOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;
	
			goto RGXSubmitTransfer_exit;
		}
	}

			/* Copy the data over */
			if ( !OSAccessOK(PVR_VERIFY_READ, (void*) psRGXSubmitTransferIN->pui32TQPrepareFlags, psRGXSubmitTransferIN->ui32PrepareCount * sizeof(IMG_UINT32))
				|| (OSCopyFromUser(NULL, ui32TQPrepareFlagsInt, psRGXSubmitTransferIN->pui32TQPrepareFlags,
				psRGXSubmitTransferIN->ui32PrepareCount * sizeof(IMG_UINT32)) != PVRSRV_OK) )
			{
				psRGXSubmitTransferOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

				goto RGXSubmitTransfer_exit;
			}
	if (psRGXSubmitTransferIN->ui32SyncPMRCount != 0)
	{
		ui32SyncPMRFlagsInt = OSAllocMem(psRGXSubmitTransferIN->ui32SyncPMRCount * sizeof(IMG_UINT32));
		if (!ui32SyncPMRFlagsInt)
		{
			psRGXSubmitTransferOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;
	
			goto RGXSubmitTransfer_exit;
		}
	}

			/* Copy the data over */
			if ( !OSAccessOK(PVR_VERIFY_READ, (void*) psRGXSubmitTransferIN->pui32SyncPMRFlags, psRGXSubmitTransferIN->ui32SyncPMRCount * sizeof(IMG_UINT32))
				|| (OSCopyFromUser(NULL, ui32SyncPMRFlagsInt, psRGXSubmitTransferIN->pui32SyncPMRFlags,
				psRGXSubmitTransferIN->ui32SyncPMRCount * sizeof(IMG_UINT32)) != PVRSRV_OK) )
			{
				psRGXSubmitTransferOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

				goto RGXSubmitTransfer_exit;
			}
	if (psRGXSubmitTransferIN->ui32SyncPMRCount != 0)
	{
		psSyncPMRsInt = OSAllocMem(psRGXSubmitTransferIN->ui32SyncPMRCount * sizeof(PMR *));
		if (!psSyncPMRsInt)
		{
			psRGXSubmitTransferOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;
	
			goto RGXSubmitTransfer_exit;
		}
		hSyncPMRsInt2 = OSAllocMem(psRGXSubmitTransferIN->ui32SyncPMRCount * sizeof(IMG_HANDLE));
		if (!hSyncPMRsInt2)
		{
			psRGXSubmitTransferOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;
	
			goto RGXSubmitTransfer_exit;
		}
	}

			/* Copy the data over */
			if ( !OSAccessOK(PVR_VERIFY_READ, (void*) psRGXSubmitTransferIN->phSyncPMRs, psRGXSubmitTransferIN->ui32SyncPMRCount * sizeof(IMG_HANDLE))
				|| (OSCopyFromUser(NULL, hSyncPMRsInt2, psRGXSubmitTransferIN->phSyncPMRs,
				psRGXSubmitTransferIN->ui32SyncPMRCount * sizeof(IMG_HANDLE)) != PVRSRV_OK) )
			{
				psRGXSubmitTransferOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

				goto RGXSubmitTransfer_exit;
			}

	PMRLock();


				{
					/* Look up the address from the handle */
					psRGXSubmitTransferOUT->eError =
						PVRSRVLookupHandle(psConnection->psHandleBase,
											(void **) &psTransferContextInt,
											psRGXSubmitTransferIN->hTransferContext,
											PVRSRV_HANDLE_TYPE_RGX_SERVER_TQ_CONTEXT);
					if(psRGXSubmitTransferOUT->eError != PVRSRV_OK)
					{
						PMRUnlock();
						goto RGXSubmitTransfer_exit;
					}
				}


	{
		IMG_UINT32 i;

		for (i=0;i<psRGXSubmitTransferIN->ui32PrepareCount;i++)
		{
			IMG_UINT32 j;
			for (j=0;j<ui32ClientFenceCountInt[i];j++)
			{
				{
					/* Look up the address from the handle */
					psRGXSubmitTransferOUT->eError =
						PVRSRVLookupHandle(psConnection->psHandleBase,
											(void **) &psFenceUFOSyncPrimBlockInt[i][j],
											hFenceUFOSyncPrimBlockInt2[i][j],
											PVRSRV_HANDLE_TYPE_SYNC_PRIMITIVE_BLOCK);
					if(psRGXSubmitTransferOUT->eError != PVRSRV_OK)
					{
						PMRUnlock();
						goto RGXSubmitTransfer_exit;
					}
				}

			}
		}
	}

	{
		IMG_UINT32 i;

		for (i=0;i<psRGXSubmitTransferIN->ui32PrepareCount;i++)
		{
			IMG_UINT32 j;
			for (j=0;j<ui32ClientUpdateCountInt[i];j++)
			{
				{
					/* Look up the address from the handle */
					psRGXSubmitTransferOUT->eError =
						PVRSRVLookupHandle(psConnection->psHandleBase,
											(void **) &psUpdateUFOSyncPrimBlockInt[i][j],
											hUpdateUFOSyncPrimBlockInt2[i][j],
											PVRSRV_HANDLE_TYPE_SYNC_PRIMITIVE_BLOCK);
					if(psRGXSubmitTransferOUT->eError != PVRSRV_OK)
					{
						PMRUnlock();
						goto RGXSubmitTransfer_exit;
					}
				}

			}
		}
	}

	{
		IMG_UINT32 i;

		for (i=0;i<psRGXSubmitTransferIN->ui32PrepareCount;i++)
		{
			IMG_UINT32 j;
			for (j=0;j<ui32ServerSyncCountInt[i];j++)
			{
				{
					/* Look up the address from the handle */
					psRGXSubmitTransferOUT->eError =
						PVRSRVLookupHandle(psConnection->psHandleBase,
											(void **) &psServerSyncInt[i][j],
											hServerSyncInt2[i][j],
											PVRSRV_HANDLE_TYPE_SERVER_SYNC_PRIMITIVE);
					if(psRGXSubmitTransferOUT->eError != PVRSRV_OK)
					{
						PMRUnlock();
						goto RGXSubmitTransfer_exit;
					}
				}

			}
		}
	}

	{
		IMG_UINT32 i;

		for (i=0;i<psRGXSubmitTransferIN->ui32SyncPMRCount;i++)
		{
				{
					/* Look up the address from the handle */
					psRGXSubmitTransferOUT->eError =
						PVRSRVLookupHandle(psConnection->psHandleBase,
											(void **) &psSyncPMRsInt[i],
											hSyncPMRsInt2[i],
											PVRSRV_HANDLE_TYPE_PHYSMEM_PMR);
					if(psRGXSubmitTransferOUT->eError != PVRSRV_OK)
					{
						PMRUnlock();
						goto RGXSubmitTransfer_exit;
					}
				}

		}
	}

	psRGXSubmitTransferOUT->eError =
		PVRSRVRGXSubmitTransferKM(
					psTransferContextInt,
					psRGXSubmitTransferIN->ui32PrepareCount,
					ui32ClientFenceCountInt,
					psFenceUFOSyncPrimBlockInt,
					ui32FenceSyncOffsetInt,
					ui32FenceValueInt,
					ui32ClientUpdateCountInt,
					psUpdateUFOSyncPrimBlockInt,
					ui32UpdateSyncOffsetInt,
					ui32UpdateValueInt,
					ui32ServerSyncCountInt,
					ui32ServerSyncFlagsInt,
					psServerSyncInt,
					psRGXSubmitTransferIN->i32CheckFenceFD,
					psRGXSubmitTransferIN->i32UpdateTimelineFD,
					&psRGXSubmitTransferOUT->i32UpdateFenceFD,
					uiUpdateFenceNameInt,
					ui32CommandSizeInt,
					ui8FWCommandInt,
					ui32TQPrepareFlagsInt,
					psRGXSubmitTransferIN->ui32ExtJobRef,
					psRGXSubmitTransferIN->ui32IntJobRef,
					psRGXSubmitTransferIN->ui32SyncPMRCount,
					ui32SyncPMRFlagsInt,
					psSyncPMRsInt);
	PMRUnlock();




RGXSubmitTransfer_exit:
	if (ui32ClientFenceCountInt)
		OSFreeMem(ui32ClientFenceCountInt);
	if (psFenceUFOSyncPrimBlockInt)
		OSFreeMem(psFenceUFOSyncPrimBlockInt);
	if (hFenceUFOSyncPrimBlockInt2)
		OSFreeMem(hFenceUFOSyncPrimBlockInt2);
	if (ui32FenceSyncOffsetInt)
		OSFreeMem(ui32FenceSyncOffsetInt);
	if (ui32FenceValueInt)
		OSFreeMem(ui32FenceValueInt);
	if (ui32ClientUpdateCountInt)
		OSFreeMem(ui32ClientUpdateCountInt);
	if (psUpdateUFOSyncPrimBlockInt)
		OSFreeMem(psUpdateUFOSyncPrimBlockInt);
	if (hUpdateUFOSyncPrimBlockInt2)
		OSFreeMem(hUpdateUFOSyncPrimBlockInt2);
	if (ui32UpdateSyncOffsetInt)
		OSFreeMem(ui32UpdateSyncOffsetInt);
	if (ui32UpdateValueInt)
		OSFreeMem(ui32UpdateValueInt);
	if (ui32ServerSyncCountInt)
		OSFreeMem(ui32ServerSyncCountInt);
	if (ui32ServerSyncFlagsInt)
		OSFreeMem(ui32ServerSyncFlagsInt);
	if (psServerSyncInt)
		OSFreeMem(psServerSyncInt);
	if (hServerSyncInt2)
		OSFreeMem(hServerSyncInt2);
	if (uiUpdateFenceNameInt)
		OSFreeMem(uiUpdateFenceNameInt);
	if (ui32CommandSizeInt)
		OSFreeMem(ui32CommandSizeInt);
	if (ui8FWCommandInt)
		OSFreeMem(ui8FWCommandInt);
	if (ui32TQPrepareFlagsInt)
		OSFreeMem(ui32TQPrepareFlagsInt);
	if (ui32SyncPMRFlagsInt)
		OSFreeMem(ui32SyncPMRFlagsInt);
	if (psSyncPMRsInt)
		OSFreeMem(psSyncPMRsInt);
	if (hSyncPMRsInt2)
		OSFreeMem(hSyncPMRsInt2);

	return 0;
}

static IMG_INT
PVRSRVBridgeRGXSetTransferContextPriority(IMG_UINT32 ui32DispatchTableEntry,
					  PVRSRV_BRIDGE_IN_RGXSETTRANSFERCONTEXTPRIORITY *psRGXSetTransferContextPriorityIN,
					  PVRSRV_BRIDGE_OUT_RGXSETTRANSFERCONTEXTPRIORITY *psRGXSetTransferContextPriorityOUT,
					 CONNECTION_DATA *psConnection)
{
	RGX_SERVER_TQ_CONTEXT * psTransferContextInt = NULL;





#if defined(PDUMP)
	PMRLock();
#endif


				{
					/* Look up the address from the handle */
					psRGXSetTransferContextPriorityOUT->eError =
						PVRSRVLookupHandle(psConnection->psHandleBase,
											(void **) &psTransferContextInt,
											psRGXSetTransferContextPriorityIN->hTransferContext,
											PVRSRV_HANDLE_TYPE_RGX_SERVER_TQ_CONTEXT);
					if(psRGXSetTransferContextPriorityOUT->eError != PVRSRV_OK)
					{
#if defined(PDUMP)
						PMRUnlock();
#endif
						goto RGXSetTransferContextPriority_exit;
					}
				}


	psRGXSetTransferContextPriorityOUT->eError =
		PVRSRVRGXSetTransferContextPriorityKM(psConnection, OSGetDevData(psConnection),
					psTransferContextInt,
					psRGXSetTransferContextPriorityIN->ui32Priority);
#if defined(PDUMP)
	PMRUnlock();
#endif




RGXSetTransferContextPriority_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeRGXKickSyncTransfer(IMG_UINT32 ui32DispatchTableEntry,
					  PVRSRV_BRIDGE_IN_RGXKICKSYNCTRANSFER *psRGXKickSyncTransferIN,
					  PVRSRV_BRIDGE_OUT_RGXKICKSYNCTRANSFER *psRGXKickSyncTransferOUT,
					 CONNECTION_DATA *psConnection)
{
	RGX_SERVER_TQ_CONTEXT * psTransferContextInt = NULL;
	SYNC_PRIMITIVE_BLOCK * *psClientFenceUFOSyncPrimBlockInt = NULL;
	IMG_HANDLE *hClientFenceUFOSyncPrimBlockInt2 = NULL;
	IMG_UINT32 *ui32ClientFenceSyncOffsetInt = NULL;
	IMG_UINT32 *ui32ClientFenceValueInt = NULL;
	SYNC_PRIMITIVE_BLOCK * *psClientUpdateUFOSyncPrimBlockInt = NULL;
	IMG_HANDLE *hClientUpdateUFOSyncPrimBlockInt2 = NULL;
	IMG_UINT32 *ui32ClientUpdateSyncOffsetInt = NULL;
	IMG_UINT32 *ui32ClientUpdateValueInt = NULL;
	IMG_UINT32 *ui32ServerSyncFlagsInt = NULL;
	SERVER_SYNC_PRIMITIVE * *psServerSyncsInt = NULL;
	IMG_HANDLE *hServerSyncsInt2 = NULL;
	IMG_CHAR *uiUpdateFenceNameInt = NULL;




	if (psRGXKickSyncTransferIN->ui32ClientFenceCount != 0)
	{
		psClientFenceUFOSyncPrimBlockInt = OSAllocMem(psRGXKickSyncTransferIN->ui32ClientFenceCount * sizeof(SYNC_PRIMITIVE_BLOCK *));
		if (!psClientFenceUFOSyncPrimBlockInt)
		{
			psRGXKickSyncTransferOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;
	
			goto RGXKickSyncTransfer_exit;
		}
		hClientFenceUFOSyncPrimBlockInt2 = OSAllocMem(psRGXKickSyncTransferIN->ui32ClientFenceCount * sizeof(IMG_HANDLE));
		if (!hClientFenceUFOSyncPrimBlockInt2)
		{
			psRGXKickSyncTransferOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;
	
			goto RGXKickSyncTransfer_exit;
		}
	}

			/* Copy the data over */
			if ( !OSAccessOK(PVR_VERIFY_READ, (void*) psRGXKickSyncTransferIN->phClientFenceUFOSyncPrimBlock, psRGXKickSyncTransferIN->ui32ClientFenceCount * sizeof(IMG_HANDLE))
				|| (OSCopyFromUser(NULL, hClientFenceUFOSyncPrimBlockInt2, psRGXKickSyncTransferIN->phClientFenceUFOSyncPrimBlock,
				psRGXKickSyncTransferIN->ui32ClientFenceCount * sizeof(IMG_HANDLE)) != PVRSRV_OK) )
			{
				psRGXKickSyncTransferOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

				goto RGXKickSyncTransfer_exit;
			}
	if (psRGXKickSyncTransferIN->ui32ClientFenceCount != 0)
	{
		ui32ClientFenceSyncOffsetInt = OSAllocMem(psRGXKickSyncTransferIN->ui32ClientFenceCount * sizeof(IMG_UINT32));
		if (!ui32ClientFenceSyncOffsetInt)
		{
			psRGXKickSyncTransferOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;
	
			goto RGXKickSyncTransfer_exit;
		}
	}

			/* Copy the data over */
			if ( !OSAccessOK(PVR_VERIFY_READ, (void*) psRGXKickSyncTransferIN->pui32ClientFenceSyncOffset, psRGXKickSyncTransferIN->ui32ClientFenceCount * sizeof(IMG_UINT32))
				|| (OSCopyFromUser(NULL, ui32ClientFenceSyncOffsetInt, psRGXKickSyncTransferIN->pui32ClientFenceSyncOffset,
				psRGXKickSyncTransferIN->ui32ClientFenceCount * sizeof(IMG_UINT32)) != PVRSRV_OK) )
			{
				psRGXKickSyncTransferOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

				goto RGXKickSyncTransfer_exit;
			}
	if (psRGXKickSyncTransferIN->ui32ClientFenceCount != 0)
	{
		ui32ClientFenceValueInt = OSAllocMem(psRGXKickSyncTransferIN->ui32ClientFenceCount * sizeof(IMG_UINT32));
		if (!ui32ClientFenceValueInt)
		{
			psRGXKickSyncTransferOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;
	
			goto RGXKickSyncTransfer_exit;
		}
	}

			/* Copy the data over */
			if ( !OSAccessOK(PVR_VERIFY_READ, (void*) psRGXKickSyncTransferIN->pui32ClientFenceValue, psRGXKickSyncTransferIN->ui32ClientFenceCount * sizeof(IMG_UINT32))
				|| (OSCopyFromUser(NULL, ui32ClientFenceValueInt, psRGXKickSyncTransferIN->pui32ClientFenceValue,
				psRGXKickSyncTransferIN->ui32ClientFenceCount * sizeof(IMG_UINT32)) != PVRSRV_OK) )
			{
				psRGXKickSyncTransferOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

				goto RGXKickSyncTransfer_exit;
			}
	if (psRGXKickSyncTransferIN->ui32ClientUpdateCount != 0)
	{
		psClientUpdateUFOSyncPrimBlockInt = OSAllocMem(psRGXKickSyncTransferIN->ui32ClientUpdateCount * sizeof(SYNC_PRIMITIVE_BLOCK *));
		if (!psClientUpdateUFOSyncPrimBlockInt)
		{
			psRGXKickSyncTransferOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;
	
			goto RGXKickSyncTransfer_exit;
		}
		hClientUpdateUFOSyncPrimBlockInt2 = OSAllocMem(psRGXKickSyncTransferIN->ui32ClientUpdateCount * sizeof(IMG_HANDLE));
		if (!hClientUpdateUFOSyncPrimBlockInt2)
		{
			psRGXKickSyncTransferOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;
	
			goto RGXKickSyncTransfer_exit;
		}
	}

			/* Copy the data over */
			if ( !OSAccessOK(PVR_VERIFY_READ, (void*) psRGXKickSyncTransferIN->phClientUpdateUFOSyncPrimBlock, psRGXKickSyncTransferIN->ui32ClientUpdateCount * sizeof(IMG_HANDLE))
				|| (OSCopyFromUser(NULL, hClientUpdateUFOSyncPrimBlockInt2, psRGXKickSyncTransferIN->phClientUpdateUFOSyncPrimBlock,
				psRGXKickSyncTransferIN->ui32ClientUpdateCount * sizeof(IMG_HANDLE)) != PVRSRV_OK) )
			{
				psRGXKickSyncTransferOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

				goto RGXKickSyncTransfer_exit;
			}
	if (psRGXKickSyncTransferIN->ui32ClientUpdateCount != 0)
	{
		ui32ClientUpdateSyncOffsetInt = OSAllocMem(psRGXKickSyncTransferIN->ui32ClientUpdateCount * sizeof(IMG_UINT32));
		if (!ui32ClientUpdateSyncOffsetInt)
		{
			psRGXKickSyncTransferOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;
	
			goto RGXKickSyncTransfer_exit;
		}
	}

			/* Copy the data over */
			if ( !OSAccessOK(PVR_VERIFY_READ, (void*) psRGXKickSyncTransferIN->pui32ClientUpdateSyncOffset, psRGXKickSyncTransferIN->ui32ClientUpdateCount * sizeof(IMG_UINT32))
				|| (OSCopyFromUser(NULL, ui32ClientUpdateSyncOffsetInt, psRGXKickSyncTransferIN->pui32ClientUpdateSyncOffset,
				psRGXKickSyncTransferIN->ui32ClientUpdateCount * sizeof(IMG_UINT32)) != PVRSRV_OK) )
			{
				psRGXKickSyncTransferOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

				goto RGXKickSyncTransfer_exit;
			}
	if (psRGXKickSyncTransferIN->ui32ClientUpdateCount != 0)
	{
		ui32ClientUpdateValueInt = OSAllocMem(psRGXKickSyncTransferIN->ui32ClientUpdateCount * sizeof(IMG_UINT32));
		if (!ui32ClientUpdateValueInt)
		{
			psRGXKickSyncTransferOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;
	
			goto RGXKickSyncTransfer_exit;
		}
	}

			/* Copy the data over */
			if ( !OSAccessOK(PVR_VERIFY_READ, (void*) psRGXKickSyncTransferIN->pui32ClientUpdateValue, psRGXKickSyncTransferIN->ui32ClientUpdateCount * sizeof(IMG_UINT32))
				|| (OSCopyFromUser(NULL, ui32ClientUpdateValueInt, psRGXKickSyncTransferIN->pui32ClientUpdateValue,
				psRGXKickSyncTransferIN->ui32ClientUpdateCount * sizeof(IMG_UINT32)) != PVRSRV_OK) )
			{
				psRGXKickSyncTransferOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

				goto RGXKickSyncTransfer_exit;
			}
	if (psRGXKickSyncTransferIN->ui32ServerSyncCount != 0)
	{
		ui32ServerSyncFlagsInt = OSAllocMem(psRGXKickSyncTransferIN->ui32ServerSyncCount * sizeof(IMG_UINT32));
		if (!ui32ServerSyncFlagsInt)
		{
			psRGXKickSyncTransferOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;
	
			goto RGXKickSyncTransfer_exit;
		}
	}

			/* Copy the data over */
			if ( !OSAccessOK(PVR_VERIFY_READ, (void*) psRGXKickSyncTransferIN->pui32ServerSyncFlags, psRGXKickSyncTransferIN->ui32ServerSyncCount * sizeof(IMG_UINT32))
				|| (OSCopyFromUser(NULL, ui32ServerSyncFlagsInt, psRGXKickSyncTransferIN->pui32ServerSyncFlags,
				psRGXKickSyncTransferIN->ui32ServerSyncCount * sizeof(IMG_UINT32)) != PVRSRV_OK) )
			{
				psRGXKickSyncTransferOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

				goto RGXKickSyncTransfer_exit;
			}
	if (psRGXKickSyncTransferIN->ui32ServerSyncCount != 0)
	{
		psServerSyncsInt = OSAllocMem(psRGXKickSyncTransferIN->ui32ServerSyncCount * sizeof(SERVER_SYNC_PRIMITIVE *));
		if (!psServerSyncsInt)
		{
			psRGXKickSyncTransferOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;
	
			goto RGXKickSyncTransfer_exit;
		}
		hServerSyncsInt2 = OSAllocMem(psRGXKickSyncTransferIN->ui32ServerSyncCount * sizeof(IMG_HANDLE));
		if (!hServerSyncsInt2)
		{
			psRGXKickSyncTransferOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;
	
			goto RGXKickSyncTransfer_exit;
		}
	}

			/* Copy the data over */
			if ( !OSAccessOK(PVR_VERIFY_READ, (void*) psRGXKickSyncTransferIN->phServerSyncs, psRGXKickSyncTransferIN->ui32ServerSyncCount * sizeof(IMG_HANDLE))
				|| (OSCopyFromUser(NULL, hServerSyncsInt2, psRGXKickSyncTransferIN->phServerSyncs,
				psRGXKickSyncTransferIN->ui32ServerSyncCount * sizeof(IMG_HANDLE)) != PVRSRV_OK) )
			{
				psRGXKickSyncTransferOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

				goto RGXKickSyncTransfer_exit;
			}
	
	{
		uiUpdateFenceNameInt = OSAllocMem(32 * sizeof(IMG_CHAR));
		if (!uiUpdateFenceNameInt)
		{
			psRGXKickSyncTransferOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;
	
			goto RGXKickSyncTransfer_exit;
		}
	}

			/* Copy the data over */
			if ( !OSAccessOK(PVR_VERIFY_READ, (void*) psRGXKickSyncTransferIN->puiUpdateFenceName, 32 * sizeof(IMG_CHAR))
				|| (OSCopyFromUser(NULL, uiUpdateFenceNameInt, psRGXKickSyncTransferIN->puiUpdateFenceName,
				32 * sizeof(IMG_CHAR)) != PVRSRV_OK) )
			{
				psRGXKickSyncTransferOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

				goto RGXKickSyncTransfer_exit;
			}

#if defined(PDUMP)
	PMRLock();
#endif


				{
					/* Look up the address from the handle */
					psRGXKickSyncTransferOUT->eError =
						PVRSRVLookupHandle(psConnection->psHandleBase,
											(void **) &psTransferContextInt,
											psRGXKickSyncTransferIN->hTransferContext,
											PVRSRV_HANDLE_TYPE_RGX_SERVER_TQ_CONTEXT);
					if(psRGXKickSyncTransferOUT->eError != PVRSRV_OK)
					{
#if defined(PDUMP)
						PMRUnlock();
#endif
						goto RGXKickSyncTransfer_exit;
					}
				}


	{
		IMG_UINT32 i;

		for (i=0;i<psRGXKickSyncTransferIN->ui32ClientFenceCount;i++)
		{
				{
					/* Look up the address from the handle */
					psRGXKickSyncTransferOUT->eError =
						PVRSRVLookupHandle(psConnection->psHandleBase,
											(void **) &psClientFenceUFOSyncPrimBlockInt[i],
											hClientFenceUFOSyncPrimBlockInt2[i],
											PVRSRV_HANDLE_TYPE_SYNC_PRIMITIVE_BLOCK);
					if(psRGXKickSyncTransferOUT->eError != PVRSRV_OK)
					{
#if defined(PDUMP)
						PMRUnlock();
#endif
						goto RGXKickSyncTransfer_exit;
					}
				}

		}
	}

	{
		IMG_UINT32 i;

		for (i=0;i<psRGXKickSyncTransferIN->ui32ClientUpdateCount;i++)
		{
				{
					/* Look up the address from the handle */
					psRGXKickSyncTransferOUT->eError =
						PVRSRVLookupHandle(psConnection->psHandleBase,
											(void **) &psClientUpdateUFOSyncPrimBlockInt[i],
											hClientUpdateUFOSyncPrimBlockInt2[i],
											PVRSRV_HANDLE_TYPE_SYNC_PRIMITIVE_BLOCK);
					if(psRGXKickSyncTransferOUT->eError != PVRSRV_OK)
					{
#if defined(PDUMP)
						PMRUnlock();
#endif
						goto RGXKickSyncTransfer_exit;
					}
				}

		}
	}

	{
		IMG_UINT32 i;

		for (i=0;i<psRGXKickSyncTransferIN->ui32ServerSyncCount;i++)
		{
				{
					/* Look up the address from the handle */
					psRGXKickSyncTransferOUT->eError =
						PVRSRVLookupHandle(psConnection->psHandleBase,
											(void **) &psServerSyncsInt[i],
											hServerSyncsInt2[i],
											PVRSRV_HANDLE_TYPE_SERVER_SYNC_PRIMITIVE);
					if(psRGXKickSyncTransferOUT->eError != PVRSRV_OK)
					{
#if defined(PDUMP)
						PMRUnlock();
#endif
						goto RGXKickSyncTransfer_exit;
					}
				}

		}
	}

	psRGXKickSyncTransferOUT->eError =
		PVRSRVRGXKickSyncTransferKM(
					psTransferContextInt,
					psRGXKickSyncTransferIN->ui32ClientFenceCount,
					psClientFenceUFOSyncPrimBlockInt,
					ui32ClientFenceSyncOffsetInt,
					ui32ClientFenceValueInt,
					psRGXKickSyncTransferIN->ui32ClientUpdateCount,
					psClientUpdateUFOSyncPrimBlockInt,
					ui32ClientUpdateSyncOffsetInt,
					ui32ClientUpdateValueInt,
					psRGXKickSyncTransferIN->ui32ServerSyncCount,
					ui32ServerSyncFlagsInt,
					psServerSyncsInt,
					psRGXKickSyncTransferIN->i32CheckFenceFD,
					psRGXKickSyncTransferIN->i32UpdateTimelineFD,
					&psRGXKickSyncTransferOUT->i32UpdateFenceFD,
					uiUpdateFenceNameInt,
					psRGXKickSyncTransferIN->ui32TQPrepareFlags,
					psRGXKickSyncTransferIN->ui32ExtJobRef,
					psRGXKickSyncTransferIN->ui32IntJobRef);
#if defined(PDUMP)
	PMRUnlock();
#endif




RGXKickSyncTransfer_exit:
	if (psClientFenceUFOSyncPrimBlockInt)
		OSFreeMem(psClientFenceUFOSyncPrimBlockInt);
	if (hClientFenceUFOSyncPrimBlockInt2)
		OSFreeMem(hClientFenceUFOSyncPrimBlockInt2);
	if (ui32ClientFenceSyncOffsetInt)
		OSFreeMem(ui32ClientFenceSyncOffsetInt);
	if (ui32ClientFenceValueInt)
		OSFreeMem(ui32ClientFenceValueInt);
	if (psClientUpdateUFOSyncPrimBlockInt)
		OSFreeMem(psClientUpdateUFOSyncPrimBlockInt);
	if (hClientUpdateUFOSyncPrimBlockInt2)
		OSFreeMem(hClientUpdateUFOSyncPrimBlockInt2);
	if (ui32ClientUpdateSyncOffsetInt)
		OSFreeMem(ui32ClientUpdateSyncOffsetInt);
	if (ui32ClientUpdateValueInt)
		OSFreeMem(ui32ClientUpdateValueInt);
	if (ui32ServerSyncFlagsInt)
		OSFreeMem(ui32ServerSyncFlagsInt);
	if (psServerSyncsInt)
		OSFreeMem(psServerSyncsInt);
	if (hServerSyncsInt2)
		OSFreeMem(hServerSyncsInt2);
	if (uiUpdateFenceNameInt)
		OSFreeMem(uiUpdateFenceNameInt);

	return 0;
}



/* *************************************************************************** 
 * Server bridge dispatch related glue 
 */

static IMG_BOOL bUseLock = IMG_TRUE;

PVRSRV_ERROR InitRGXTQBridge(void);
PVRSRV_ERROR DeinitRGXTQBridge(void);

/*
 * Register all RGXTQ functions with services
 */
PVRSRV_ERROR InitRGXTQBridge(void)
{

	SetDispatchTableEntry(PVRSRV_BRIDGE_RGXTQ, PVRSRV_BRIDGE_RGXTQ_RGXCREATETRANSFERCONTEXT, PVRSRVBridgeRGXCreateTransferContext,
					NULL, bUseLock);

	SetDispatchTableEntry(PVRSRV_BRIDGE_RGXTQ, PVRSRV_BRIDGE_RGXTQ_RGXDESTROYTRANSFERCONTEXT, PVRSRVBridgeRGXDestroyTransferContext,
					NULL, bUseLock);

	SetDispatchTableEntry(PVRSRV_BRIDGE_RGXTQ, PVRSRV_BRIDGE_RGXTQ_RGXSUBMITTRANSFER, PVRSRVBridgeRGXSubmitTransfer,
					NULL, bUseLock);

	SetDispatchTableEntry(PVRSRV_BRIDGE_RGXTQ, PVRSRV_BRIDGE_RGXTQ_RGXSETTRANSFERCONTEXTPRIORITY, PVRSRVBridgeRGXSetTransferContextPriority,
					NULL, bUseLock);

	SetDispatchTableEntry(PVRSRV_BRIDGE_RGXTQ, PVRSRV_BRIDGE_RGXTQ_RGXKICKSYNCTRANSFER, PVRSRVBridgeRGXKickSyncTransfer,
					NULL, bUseLock);


	return PVRSRV_OK;
}

/*
 * Unregister all rgxtq functions with services
 */
PVRSRV_ERROR DeinitRGXTQBridge(void)
{
	return PVRSRV_OK;
}

