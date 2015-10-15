/* -*- mode: c; indent-tabs-mode: t; c-basic-offset: 8; tab-width: 8 -*- */
/* vi: set ts=8 sw=8 sts=8: */
/*************************************************************************/ /*!
@File
@Title          PowerVR Linux fence interface
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

#include <linux/kernel.h>
#include <linux/slab.h>

#include "pvr_fence.h"
#include "services_kernel_client.h"

#define PVR_DUMPDEBUG_LOG(pfnDumpDebugPrintf, fmt, ...)			\
	do {								\
		if (pfnDumpDebugPrintf)					\
			pfnDumpDebugPrintf(fmt, ## __VA_ARGS__);	\
		else							\
			pr_err(fmt "\n", ## __VA_ARGS__);		\
	} while (0)

static inline u32
pvr_fence_sync_value_get(struct pvr_fence *pvr_fence)
{
	return *pvr_fence->sync->pui32LinAddr;
}

static inline bool
pvr_fence_sync_value_met(struct pvr_fence *pvr_fence,
			 enum pvr_fence_sync_val value)
{
	return !!(*pvr_fence->sync->pui32LinAddr == value);
}

static inline void
pvr_fence_sync_value_set(struct pvr_fence *pvr_fence,
			 enum pvr_fence_sync_val value)
{
	*pvr_fence->sync->pui32LinAddr = value;
}

static void
pvr_fence_context_check_status(struct work_struct *data)
{
	PVRSRVCheckStatus(NULL);
}

static void
pvr_fence_context_fences_dump(struct pvr_fence_context *fctx)
{
	struct pvr_fence *pvr_fence;
	unsigned long flags;

	spin_lock_irqsave(&fctx->list_lock, flags);
	list_for_each_entry(pvr_fence, &fctx->fence_list, fence_head) {
		PVR_DUMPDEBUG_LOG(g_pfnDumpDebugPrintf,
				  "f %u#%u: (%s%s) Refs = %u, FWAddr = %#08x, Current = %#08x, Next = %#08x, %s %s",
				  pvr_fence->fence->context,
				  pvr_fence->fence->seqno,
				  test_bit(FENCE_FLAG_ENABLE_SIGNAL_BIT, &pvr_fence->fence->flags) ? "+" : "-",
				  test_bit(FENCE_FLAG_SIGNALED_BIT, &pvr_fence->fence->flags) ? "+" : "-",
				  atomic_read(&pvr_fence->fence->refcount.refcount),
				  SyncPrimGetFirmwareAddr(pvr_fence->sync),
				  pvr_fence_sync_value_get(pvr_fence),
				  PVR_FENCE_SYNC_VAL_SIGNALED,
				  pvr_fence->name,
				  (&pvr_fence->base != pvr_fence->fence) ? "(foreign)" : "");
	}
	spin_unlock_irqrestore(&fctx->list_lock, flags);
}

static void
pvr_fence_context_queue_signal_work(void *data)
{
	struct pvr_fence_context *fctx = (struct pvr_fence_context *)data;

	queue_work(fctx->fence_wq, &fctx->signal_work);
}

static inline unsigned
pvr_fence_context_seqno_next(struct pvr_fence_context *fctx)
{
	return atomic_inc_return(&fctx->fence_seqno) - 1;
}

static inline void pvr_fence_context_free_deferred(struct pvr_fence_context *fctx)
{
	struct pvr_fence *pvr_fence, *tmp;
	LIST_HEAD(deferred_free_list);
	unsigned long flags;

	spin_lock_irqsave(&fctx->list_lock, flags);
	list_for_each_entry_safe(pvr_fence, tmp,
				 &fctx->deferred_free_list,
				 fence_head)
		list_move(&pvr_fence->fence_head, &deferred_free_list);
	spin_unlock_irqrestore(&fctx->list_lock, flags);

	list_for_each_entry_safe(pvr_fence, tmp,
				 &deferred_free_list,
				 fence_head) {
		list_del(&pvr_fence->fence_head);
		SyncPrimFree(pvr_fence->sync);
		fence_free(&pvr_fence->base);
	}
}

static void
pvr_fence_context_signal_fences(struct work_struct *data)
{
	struct pvr_fence_context *fctx =
		container_of(data, struct pvr_fence_context, signal_work);
	struct pvr_fence *pvr_fence, *tmp;
	unsigned long flags;
	LIST_HEAD(signal_list);

	/*
	 * We can't call fence_signal while holding the lock as we can end up
	 * in a situation whereby pvr_fence_foreign_signal_sync, which also
	 * takes the list lock, ends up being called as a result of the
	 * fence_signal below, i.e. fence_signal(fence) -> fence->callback()
	 *  -> fence_signal(foreign_fence) -> foreign_fence->callback() where
	 * the foreign_fence callback is pvr_fence_foreign_signal_sync.
	 *
	 * So extract the items we intend to signal and add them to their own
	 * queue.
	 */
	spin_lock_irqsave(&fctx->list_lock, flags);
	list_for_each_entry_safe(pvr_fence, tmp, &fctx->signal_list, signal_head) {
		if (!pvr_fence_sync_value_met(pvr_fence,
					      PVR_FENCE_SYNC_VAL_SIGNALED))
			break;

		list_move(&pvr_fence->signal_head, &signal_list);
	}
	spin_unlock_irqrestore(&fctx->list_lock, flags);

	list_for_each_entry_safe(pvr_fence, tmp, &signal_list, signal_head) {

		PVR_FENCE_TRACE(&pvr_fence->base, "signalled fence (%s)\n",
				pvr_fence->name);
		list_del(&pvr_fence->signal_head);
		fence_signal(pvr_fence->fence);
		fence_put(pvr_fence->fence);
	}

	/*
	 * Take this opportunity to free up any fence objects we
	 * have deferred freeing.
	 */
	pvr_fence_context_free_deferred(fctx);
}

static void
pvr_fence_context_debug_request(void *data, u32 verbosity)
{
	struct pvr_fence_context *fctx = (struct pvr_fence_context *)data;

	if (verbosity == DEBUG_REQUEST_VERBOSITY_HIGH)
		pvr_fence_context_fences_dump(fctx);
}

/**
 * pvr_fence_context_create - creates a PVR fence context
 * @dev_cookie: services device cookie
 * @name: context name (used for debugging)
 *
 * Creates a PVR fence context that can be used to create PVR fences or to
 * create PVR fences from an existing fence.
 *
 * pvr_fence_context_destroy should be called to clean up the fence context.
 *
 * Returns NULL if a context cannot be created.
 */
struct pvr_fence_context *
pvr_fence_context_create(void *dev_cookie,
			 const char *name)
{
	struct pvr_fence_context *fctx;
	PVRSRV_ERROR srv_err;

	fctx = kzalloc(sizeof(*fctx), GFP_KERNEL);
	if (!fctx)
		return NULL;

	spin_lock_init(&fctx->lock);
	atomic_set(&fctx->fence_seqno, 0);
	INIT_WORK(&fctx->check_status_work, pvr_fence_context_check_status);
	INIT_WORK(&fctx->signal_work, pvr_fence_context_signal_fences);
	spin_lock_init(&fctx->list_lock);
	INIT_LIST_HEAD(&fctx->signal_list);
	INIT_LIST_HEAD(&fctx->fence_list);
	INIT_LIST_HEAD(&fctx->deferred_free_list);

	fctx->fence_context = fence_context_alloc(1);
	fctx->name = name;

	fctx->fence_wq =
		create_freezable_workqueue("pvr_fence_sync_workqueue");
	if (!fctx->fence_wq) {
		pr_err("%s: failed to create fence workqueue\n", __func__);
		goto err_free_fctx;
	}

	srv_err = SyncPrimContextCreate(dev_cookie, &fctx->sync_prim_context);
	if (srv_err != PVRSRV_OK) {
		pr_err("%s: failed to create sync prim context (%s)\n",
		       __func__, PVRSRVGetErrorStringKM(srv_err));
		goto err_destroy_workqueue;
	}

	srv_err = PVRSRVRegisterCmdCompleteNotify(&fctx->cmd_complete_handle,
						  pvr_fence_context_queue_signal_work,
						  fctx);
	if (srv_err != PVRSRV_OK) {
		pr_err("%s: failed to register command complete callback (%s)\n",
		       __func__, PVRSRVGetErrorStringKM(srv_err));
		goto err_sync_prim_context_destroy;
	}

	srv_err = PVRSRVRegisterDbgRequestNotify(&fctx->dbg_request_handle,
						 pvr_fence_context_debug_request,
						 DEBUG_REQUEST_LINUXFENCE,
						 fctx);
	if (srv_err != PVRSRV_OK) {
		pr_err("%s: failed to register debug request callback (%s)\n",
		       __func__, PVRSRVGetErrorStringKM(srv_err));
		goto err_unregister_cmd_complete_notify;
	}

	PVR_FENCE_CTX_TRACE(fctx, "created fence context (%s)\n", name);

	return fctx;

err_unregister_cmd_complete_notify:
	PVRSRVUnregisterCmdCompleteNotify(fctx->cmd_complete_handle);
err_sync_prim_context_destroy:
	SyncPrimContextDestroy(fctx->sync_prim_context);
err_destroy_workqueue:
	destroy_workqueue(fctx->fence_wq);
err_free_fctx:
	kfree(fctx);
	return NULL;
}

/**
 * pvr_fence_context_destroy - destroys a context
 * @fctx: PVR fence context to destroy
 *
 * Destroys a PVR fence context with the expectation that all fences have been
 * destroyed.
 */
void
pvr_fence_context_destroy(struct pvr_fence_context *fctx)
{
	PVR_FENCE_CTX_TRACE(fctx, "destroyed fence context (%s)\n", fctx->name);

	pvr_fence_context_free_deferred(fctx);

	if (WARN_ON(!list_empty_careful(&fctx->fence_list)))
		pvr_fence_context_fences_dump(fctx);

	PVRSRVUnregisterDbgRequestNotify(fctx->dbg_request_handle);
	PVRSRVUnregisterCmdCompleteNotify(fctx->cmd_complete_handle);

	destroy_workqueue(fctx->fence_wq);

	SyncPrimContextDestroy(fctx->sync_prim_context);

	kfree(fctx);
}

static const char *
pvr_fence_get_driver_name(struct fence *fence)
{
	return PVR_LDM_DRIVER_REGISTRATION_NAME;
}

static const char *
pvr_fence_get_timeline_name(struct fence *fence)
{
	struct pvr_fence *pvr_fence = to_pvr_fence(fence);

	return pvr_fence->fctx->name;
}

static bool
pvr_fence_enable_signaling(struct fence *fence)
{
	struct pvr_fence *pvr_fence = to_pvr_fence(fence);
	struct pvr_fence_context *fctx = pvr_fence->fctx;
	unsigned long flags;

	WARN_ON_SMP(!spin_is_locked(&fctx->lock));

	if (pvr_fence_sync_value_met(pvr_fence,
				     PVR_FENCE_SYNC_VAL_SIGNALED))
		return false;

	fence_get(&pvr_fence->base);

	spin_lock_irqsave(&fctx->list_lock, flags);
	list_add_tail(&pvr_fence->signal_head, &fctx->signal_list);
	spin_unlock_irqrestore(&fctx->list_lock, flags);

	PVR_FENCE_TRACE(&pvr_fence->base, "signalling enabled (%s)\n",
			pvr_fence->name);

	return true;
}

static bool
pvr_fence_is_signaled(struct fence *fence)
{
	struct pvr_fence *pvr_fence = to_pvr_fence(fence);

	return pvr_fence_sync_value_met(pvr_fence, PVR_FENCE_SYNC_VAL_SIGNALED);
}

static void
pvr_fence_release(struct fence *fence)
{
	struct pvr_fence *pvr_fence = to_pvr_fence(fence);
	struct pvr_fence_context *fctx = pvr_fence->fctx;
	unsigned long flags;

	PVR_FENCE_TRACE(&pvr_fence->base, "released fence (%s)\n",
			pvr_fence->name);

	pvr_fence_sync_value_set(pvr_fence, PVR_FENCE_SYNC_VAL_DONE);

	spin_lock_irqsave(&fctx->list_lock, flags);
	list_move(&pvr_fence->fence_head, &fctx->deferred_free_list);
	spin_unlock_irqrestore(&fctx->list_lock, flags);
}

const struct fence_ops pvr_fence_ops = {
	.get_driver_name = pvr_fence_get_driver_name,
	.get_timeline_name = pvr_fence_get_timeline_name,
	.enable_signaling = pvr_fence_enable_signaling,
	.signaled = pvr_fence_is_signaled,
	.wait = fence_default_wait,
	.release = pvr_fence_release,
};

/**
 * pvr_fence_create - creates a PVR fence
 * @fctx: PVR fence context on which the PVR fence should be created
 * @name: PVR fence name (used for debugging)
 *
 * Creates a PVR fence.
 *
 * Once the fence is finished with pvr_fence_destroy should be called.
 *
 * Returns NULL if a PVR fence cannot be created.
 */
struct pvr_fence *
pvr_fence_create(struct pvr_fence_context *fctx, const char *name)
{
	struct pvr_fence *pvr_fence;
	unsigned seqno;
	unsigned long flags;
	PVRSRV_ERROR srv_err;

	pvr_fence = kzalloc(sizeof(*pvr_fence), GFP_KERNEL);
	if (!pvr_fence)
		return NULL;

	srv_err = SyncPrimAlloc(fctx->sync_prim_context, &pvr_fence->sync,
				name);
	if (srv_err != PVRSRV_OK)
		goto err_free_fence;

	pvr_fence_sync_value_set(pvr_fence, PVR_FENCE_SYNC_VAL_INIT);
	INIT_LIST_HEAD(&pvr_fence->fence_head);
	INIT_LIST_HEAD(&pvr_fence->signal_head);
	pvr_fence->fctx = fctx;
	pvr_fence->name = name;
	pvr_fence->fence = &pvr_fence->base;

	seqno = pvr_fence_context_seqno_next(fctx);
	fence_init(&pvr_fence->base, &pvr_fence_ops, &fctx->lock,
		   fctx->fence_context, seqno);

	spin_lock_irqsave(&fctx->list_lock, flags);
	list_add_tail(&pvr_fence->fence_head, &fctx->fence_list);
	spin_unlock_irqrestore(&fctx->list_lock, flags);

	PVR_FENCE_TRACE(&pvr_fence->base, "created fence (%s)\n", name);

	return pvr_fence;

err_free_fence:
	kfree(pvr_fence);
	return NULL;
}

static const char *
pvr_fence_foreign_get_driver_name(struct fence *fence)
{
	return "unknown";
}

static const char *
pvr_fence_foreign_get_timeline_name(struct fence *fence)
{
	return "unknown";
}

static bool
pvr_fence_foreign_enable_signaling(struct fence *fence)
{
	WARN_ON("cannot enable signalling on foreign fence");
	return false;
}

static signed long
pvr_fence_foreign_wait(struct fence *fence, bool intr, signed long timeout)
{
	WARN_ON("cannot wait on foreign fence");
	return 0;
}

static void
pvr_fence_foreign_release(struct fence *fence)
{
	struct pvr_fence *pvr_fence = to_pvr_fence(fence);
	struct pvr_fence_context *fctx = pvr_fence->fctx;
	unsigned long flags;

	PVR_FENCE_TRACE(&pvr_fence->base,
			"released fence for foreign fence %d#%d (%s)\n",
			pvr_fence->fence->context, pvr_fence->fence->seqno,
			pvr_fence->name);

	pvr_fence_sync_value_set(pvr_fence, PVR_FENCE_SYNC_VAL_DONE);

	spin_lock_irqsave(&fctx->list_lock, flags);
	list_move(&pvr_fence->fence_head, &fctx->deferred_free_list);
	spin_unlock_irqrestore(&fctx->list_lock, flags);
}

const struct fence_ops pvr_fence_foreign_ops = {
	.get_driver_name = pvr_fence_foreign_get_driver_name,
	.get_timeline_name = pvr_fence_foreign_get_timeline_name,
	.enable_signaling = pvr_fence_foreign_enable_signaling,
	.wait = pvr_fence_foreign_wait,
	.release = pvr_fence_foreign_release,
};

static void
pvr_fence_foreign_signal_sync(struct fence *fence, struct fence_cb *cb)
{
	struct pvr_fence *pvr_fence = container_of(cb, struct pvr_fence, cb);
	struct pvr_fence_context *fctx = pvr_fence->fctx;

	if (WARN_ON_ONCE(is_pvr_fence(fence)))
		return;

	pvr_fence_sync_value_set(pvr_fence, PVR_FENCE_SYNC_VAL_SIGNALED);

	queue_work(fctx->fence_wq, &fctx->check_status_work);

	PVR_FENCE_TRACE(&pvr_fence->base, "foreign fence %d#%d signalled (%s)\n",
			pvr_fence->fence->context, pvr_fence->fence->seqno,
			pvr_fence->name);

	/* Drop the reference on the base fence */
	fence_put(&pvr_fence->base);
}

/**
 * pvr_fence_create_from_fence - creates a PVR fence from a fence
 * @fctx: PVR fence context on which the PVR fence should be created
 * @fence: fence from which the PVR fence should be created
 * @name: PVR fence name (used for debugging)
 *
 * Creates a PVR fence from an existing fence. If the fence is a foreign fence,
 * i.e. one that doesn't originate from a PVR fence context, then a new PVR
 * fence will be created. Otherwise, a reference will be taken on the underlying
 * fence and the PVR fence will be returned.
 *
 * Once the fence is finished with pvr_fence_destroy should be called.
 *
 * Returns NULL if a PVR fence cannot be created.
 */
struct pvr_fence *
pvr_fence_create_from_fence(struct pvr_fence_context *fctx,
			    struct fence *fence,
			    const char *name)
{
	struct pvr_fence *pvr_fence = to_pvr_fence(fence);
	unsigned seqno;
	unsigned long flags;
	PVRSRV_ERROR srv_err;
	int err;

	if (pvr_fence) {
		if (WARN_ON(fence->ops == &pvr_fence_foreign_ops))
			return NULL;
		fence_get(fence);

		PVR_FENCE_TRACE(fence, "created fence from PVR fence (%s)\n",
				name);
		return pvr_fence;
	}

	pvr_fence = kzalloc(sizeof(*pvr_fence), GFP_KERNEL);
	if (!pvr_fence)
		return NULL;

	srv_err = SyncPrimAlloc(fctx->sync_prim_context, &pvr_fence->sync,
				name);
	if (srv_err != PVRSRV_OK)
		goto err_free_pvr_fence;

	pvr_fence_sync_value_set(pvr_fence, PVR_FENCE_SYNC_VAL_INIT);
	INIT_LIST_HEAD(&pvr_fence->fence_head);
	INIT_LIST_HEAD(&pvr_fence->signal_head);
	pvr_fence->fctx = fctx;
	pvr_fence->name = name;
	pvr_fence->fence = fence;

	/*
	 * We use the base fence to refcount the PVR fence and to do the
	 * necessary clean up once the refcount drops to 0.
	 */
	seqno = pvr_fence_context_seqno_next(fctx);
	fence_init(&pvr_fence->base, &pvr_fence_foreign_ops, &fctx->lock,
		   fctx->fence_context, seqno);

	/*
	 * Take an extra reference on the base fence that gets dropped when the
	 * foreign fence is signalled.
	 */
	fence_get(&pvr_fence->base);

	spin_lock_irqsave(&fctx->list_lock, flags);
	list_add_tail(&pvr_fence->fence_head, &fctx->fence_list);
	spin_unlock_irqrestore(&fctx->list_lock, flags);

	PVR_FENCE_TRACE(&pvr_fence->base,
			"created fence from foreign fence %d#%d (%s)\n",
			pvr_fence->fence->context, pvr_fence->fence->seqno,
			name);

	err = fence_add_callback(fence, &pvr_fence->cb,
				 pvr_fence_foreign_signal_sync);
	if (err) {
		if (err != -ENOENT)
			goto err_list_del;

		/*
		 * The fence has already signalled so set the sync as signalled.
		 */
		pvr_fence_sync_value_set(pvr_fence,
					 PVR_FENCE_SYNC_VAL_SIGNALED);
		PVR_FENCE_TRACE(&pvr_fence->base, "foreign fence %d#%d already signaled (%s)\n",
				pvr_fence->fence->context, pvr_fence->fence->seqno,
				name);
		fence_put(&pvr_fence->base);
	}


	return pvr_fence;

err_list_del:
	spin_lock_irqsave(&fctx->list_lock, flags);
	list_del(&pvr_fence->fence_head);
	spin_unlock_irqrestore(&fctx->list_lock, flags);
	SyncPrimFree(pvr_fence->sync);
err_free_pvr_fence:
	kfree(pvr_fence);
	return NULL;
}

/**
 * pvr_fence_destroy - destroys a PVR fence
 * @pvr_fence: PVR fence to destroy
 *
 * Destroys a PVR fence. Upon return, the PVR fence may still exist if something
 * else still references the underlying fence, e.g. a reservation object, or if
 * software signalling has been enabled and the fence hasn't yet been signalled.
 */
void
pvr_fence_destroy(struct pvr_fence *pvr_fence)
{
	PVR_FENCE_TRACE(&pvr_fence->base, "destroyed fence (%s)\n",
			pvr_fence->name);

	fence_put(&pvr_fence->base);
}

/**
 * pvr_fence_sync_sw_signal - signals a PVR fence sync
 * @pvr_fence: PVR fence to signal
 *
 * Sets the PVR fence sync value to signalled.
 *
 * Returns -EINVAL if the PVR fence represents a foreign fence.
 */
int
pvr_fence_sync_sw_signal(struct pvr_fence *pvr_fence)
{
	if (!is_our_fence(pvr_fence->fctx, &pvr_fence->base))
		return -EINVAL;

	pvr_fence_sync_value_set(pvr_fence, PVR_FENCE_SYNC_VAL_SIGNALED);

	queue_work(pvr_fence->fctx->fence_wq,
		   &pvr_fence->fctx->check_status_work);

	PVR_FENCE_TRACE(&pvr_fence->base, "sw set fence sync signalled (%s)\n",
			pvr_fence->name);

	return 0;
}
