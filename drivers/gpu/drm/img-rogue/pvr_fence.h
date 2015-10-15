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

#if !defined(__PVR_FENCE_H__)
#define __PVR_FENCE_H__

#include <linux/fence.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

struct SYNC_PRIM_CONTEXT;
struct PVRSRV_CLIENT_SYNC_PRIM;

/**
 * pvr_fence_context - PVR fence context used to create and manage PVR fences
 * @lock: protects the context and fences created on the context
 * @name: fence context name (used for debugging)
 * @dbg_request_handle: handle for callback used to dump debug data
 * @sync_prim_context: sync prim context used to create services client syncs
 * @fence_context: fence context with which to associate fences
 * @fence_seqno: sequence number to use for the next fence
 * @fence_wq: work queue for signalled fence work
 * @check_status_work: work item used to inform services when a foreign fence
 * has signalled
 * @cmd_complete_handle: handle for callback used to signal fences when fence
 * syncs are met
 * @signal_work: work item used to signal fences when fence syncs are met
 * @list_lock: protects the active and active foreign lists
 * @signal_list: list of fences waiting to be signalled
 * @fence_list: list of fences (used for debugging)
 * @deferred_free_list: list of fences that we will free when we are no longer
 * holding spinlocks.  The frees get implemented when an update fence is
 * signalled or the context is freed.
 */
struct pvr_fence_context {
	spinlock_t lock;
	const char *name;
	void *dbg_request_handle;

	struct SYNC_PRIM_CONTEXT *sync_prim_context;
	unsigned fence_context;
	atomic_t fence_seqno;

	struct workqueue_struct *fence_wq;
	struct work_struct check_status_work;

	void *cmd_complete_handle;
	struct work_struct signal_work;

	spinlock_t list_lock;
	struct list_head signal_list;
	struct list_head fence_list;
	struct list_head deferred_free_list;
};

/**
 * pvr_fence - PVR fence that represents both native and foreign fences
 * @base: fence structure
 * @fctx: fence context on which this fence was created
 * @name: fence name (used for debugging)
 * @fence: pointer to base fence structure or foreign fence
 * @sync: services sync primitive used by hardware
 * @fence_head: entry on the context fence and deferred free list
 * @signal_head: entry on the context signal list
 * @cb: foreign fence callback to set the sync to signalled
 */
struct pvr_fence {
	struct fence base;
	struct pvr_fence_context *fctx;
	const char *name;

	struct fence *fence;
	struct PVRSRV_CLIENT_SYNC_PRIM *sync;

	struct list_head fence_head;
	struct list_head signal_head;
	struct fence_cb cb;
};

enum pvr_fence_sync_val {
	PVR_FENCE_SYNC_VAL_INIT = 0x60606060,
	PVR_FENCE_SYNC_VAL_SIGNALED = 0x90909090,
	PVR_FENCE_SYNC_VAL_DONE = 0xDEADDEAD,
};

extern const struct fence_ops pvr_fence_ops;
extern const struct fence_ops pvr_fence_foreign_ops;

static inline bool is_our_fence(struct pvr_fence_context *fctx,
				struct fence *fence)
{
	return (fence->context == fctx->fence_context);
}

static inline bool is_pvr_fence(struct fence *fence)
{
	return ((fence->ops == &pvr_fence_ops) ||
		(fence->ops == &pvr_fence_foreign_ops));
}

static inline struct pvr_fence *to_pvr_fence(struct fence *fence)
{
	if (is_pvr_fence(fence))
		return container_of(fence, struct pvr_fence, base);

	return NULL;
}

struct pvr_fence_context *pvr_fence_context_create(void *dev_cookie,
						   const char *name);
void pvr_fence_context_destroy(struct pvr_fence_context *fctx);

struct pvr_fence *pvr_fence_create(struct pvr_fence_context *fctx,
				   const char *name);
struct pvr_fence *pvr_fence_create_from_fence(struct pvr_fence_context *fctx,
					      struct fence *fence,
					      const char *name);
void pvr_fence_destroy(struct pvr_fence *pvr_fence);
int pvr_fence_sync_sw_signal(struct pvr_fence *pvr_fence);

#if defined(PVR_FENCE_DEBUG)
#define PVR_FENCE_CTX_TRACE(c, fmt, ...)				\
	do {								\
		struct pvr_fence_context *__fctx = (c);			\
		pr_err("c %u: (PVR) " fmt, __fctx->fence_context,	\
		       ## __VA_ARGS__);					\
	} while(0)
#else
#define PVR_FENCE_CTX_TRACE(c, fmt, ...)
#endif

#define PVR_FENCE_CTX_WARN(c, fmt, ...)					\
	do {								\
		struct pvr_fence_context *__fctx = (c);			\
		pr_warn("c %u: (PVR) " fmt, __fctx->fence_context,	\
			## __VA_ARGS__);				\
	} while(0)

#define PVR_FENCE_CTX_ERR(c, fmt, ...)					\
	do {								\
		struct pvr_fence_context *__fctx = (c);			\
		pr_err("c %u: (PVR) " fmt, __fctx->fence_context,	\
		       ## __VA_ARGS__);					\
	} while(0)

#if defined(PVR_FENCE_DEBUG)
#define PVR_FENCE_TRACE(f, fmt, ...)					\
	FENCE_ERR(f, "(PVR) " fmt, ## __VA_ARGS__)
#else
#define PVR_FENCE_TRACE(f, fmt, ...)
#endif

#define PVR_FENCE_WARN(f, fmt, ...)					\
	FENCE_WARN(f, "(PVR) " fmt, ## __VA_ARGS__)

#define PVR_FENCE_ERR(f, fmt, ...)					\
	FENCE_ERR(f, "(PVR) " fmt, ## __VA_ARGS__)

#endif /* !defined(__PVR_FENCE_H__) */
