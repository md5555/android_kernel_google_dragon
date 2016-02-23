/* -*- mode: c; indent-tabs-mode: t; c-basic-offset: 8; tab-width: 8 -*- */
/* vi: set ts=8 sw=8 sts=8: */
/*************************************************************************/ /*!
@File
@Title          Linux buffer sync interface
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

#include <linux/reservation.h>

#include "pvr_buffer_sync.h"
#include "pvr_buffer_sync_shared.h"
#include "pvr_fence.h"
#include "pvrsrv.h"
#include "pvrsrv_error.h"
#include "sync_internal.h"
#include "physmem_dmabuf.h"

struct pvr_buffer_sync_check_data {
	struct fence_cb base;

	u32 nr_fences;
	struct pvr_fence **fences;
};

struct pvr_buffer_sync_append_data {
	bool appended;

	u32 nr_checks;
	PRGXFWIF_UFO_ADDR *check_ufo_addresses;
	u32 *check_values;

	u32 nr_updates;
	PRGXFWIF_UFO_ADDR *update_ufo_addresses;
	u32 *update_values;

	u32 nr_pmrs;
	PMR **pmrs;
	u32 *pmr_flags;

	struct pvr_fence *update_fence;
};

static IMG_HANDLE dev_cookie;
static struct pvr_fence_context *fence_ctx;
static struct ww_acquire_ctx acquire_ctx;


static struct reservation_object *
pmr_reservation_object_get(PMR *pmr)
{
	struct dma_buf *dmabuf;

	dmabuf = PhysmemGetDmaBuf(pmr);
	if (dmabuf)
		return dmabuf->resv;

	return NULL;
}

static int
pvr_buffer_sync_pmrs_lock(u32 nr_pmrs, PMR **pmrs)
{
	struct reservation_object *resv, *cresv = NULL, *lresv = NULL;
	int i, err;

	ww_acquire_init(&acquire_ctx, &reservation_ww_class);
retry:
	for (i = 0; i < nr_pmrs; i++) {
		resv = pmr_reservation_object_get(pmrs[i]);
		if (!resv) {
			pr_err("%s: Failed to get reservation object from pmr %p\n",
			       __func__, pmrs[i]);
			err = -EINVAL;
			goto fail;
		}

		if (resv != lresv) {
			err = ww_mutex_lock_interruptible(&resv->lock,
							  &acquire_ctx);
			if (err) {
				cresv = (err == -EDEADLK) ? resv : NULL;
				goto fail;
			}
		} else {
			lresv = NULL;
		}
	}

	ww_acquire_done(&acquire_ctx);

	return 0;

fail:
	while (i--) {
		resv = pmr_reservation_object_get(pmrs[i]);
		if (WARN_ON_ONCE(!resv))
			continue;
		ww_mutex_unlock(&resv->lock);
	}

	if (lresv)
		ww_mutex_unlock(&lresv->lock);

	if (cresv) {
		err = ww_mutex_lock_slow_interruptible(&cresv->lock,
						       &acquire_ctx);
		if (!err) {
			lresv = cresv;
			cresv = NULL;
			goto retry;
		}
	}

	return err;
}

static void
pvr_buffer_sync_pmrs_unlock(u32 nr_pmrs, PMR **pmrs)
{
	struct reservation_object *resv;
	int i;

	for (i = 0; i < nr_pmrs; i++) {
		resv = pmr_reservation_object_get(pmrs[i]);
		if (WARN_ON_ONCE(!resv))
			continue;
		ww_mutex_unlock(&resv->lock);
	}

	ww_acquire_fini(&acquire_ctx);
}

static u32
pvr_buffer_sync_pmrs_fence_count(u32 nr_pmrs, PMR **pmrs, u32 *pmr_flags)
{
	struct reservation_object *resv;
	struct reservation_object_list *resv_list;
	struct fence *fence;
	u32 fence_count = 0;
	bool exclusive;
	int i;

	for (i = 0; i < nr_pmrs; i++) {
		exclusive = !!(pmr_flags[i] & PVR_BUFFER_FLAG_WRITE);

		resv = pmr_reservation_object_get(pmrs[i]);
		if (WARN_ON_ONCE(!resv))
			continue;

		resv_list = reservation_object_get_list(resv);
		fence = reservation_object_get_excl(resv);

		if (fence &&
		    (!exclusive || !resv_list || !resv_list->shared_count))
			fence_count++;

		if (exclusive && resv_list)
			fence_count += resv_list->shared_count;
	}

	return fence_count;
}

static struct pvr_buffer_sync_check_data *
pvr_buffer_sync_check_fences_create(u32 nr_pmrs, PMR **pmrs, u32 *pmr_flags)
{
	struct pvr_buffer_sync_check_data *data;
	struct reservation_object *resv;
	struct reservation_object_list *resv_list;
	struct fence *fence;
	u32 fence_count;
	bool exclusive;
	int i, j;
	int err;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return NULL;

	fence_count = pvr_buffer_sync_pmrs_fence_count(nr_pmrs, pmrs,
						       pmr_flags);
	if (fence_count) {
		data->fences = kcalloc(fence_count, sizeof(*data->fences),
				       GFP_KERNEL);
		if (!data->fences)
			goto err_check_data_free;
	}

	for (i = 0; i < nr_pmrs; i++) {
		resv = pmr_reservation_object_get(pmrs[i]);
		if (WARN_ON_ONCE(!resv))
			continue;

		exclusive = !!(pmr_flags[i] & PVR_BUFFER_FLAG_WRITE);
		if (!exclusive) {
			err = reservation_object_reserve_shared(resv);
			if (err)
				goto err_destroy_fences;
		}

		resv_list = reservation_object_get_list(resv);
		fence = reservation_object_get_excl(resv);

		if (fence &&
		    (!exclusive || !resv_list || !resv_list->shared_count)) {
			data->fences[data->nr_fences++] =
				pvr_fence_create_from_fence(fence_ctx,
							    fence,
							    "exclusive check fence");
			if (!data->fences[data->nr_fences - 1]) {
				data->nr_fences--;
				PVR_FENCE_TRACE(fence, "waiting on exclusive fence\n");
				WARN_ON(fence_wait(fence, true) <= 0);
			}
		}

		if (exclusive && resv_list) {
			for (j = 0; j < resv_list->shared_count; j++) {
				fence = rcu_dereference_protected(resv_list->shared[j],
								  reservation_object_held(resv));
				data->fences[data->nr_fences++] =
					pvr_fence_create_from_fence(fence_ctx,
								    fence,
								    "check fence");
				if (!data->fences[data->nr_fences - 1]) {
					data->nr_fences--;
					PVR_FENCE_TRACE(fence, "waiting on non-exclusive fence\n");
					WARN_ON(fence_wait(fence, true) <= 0);
				}
			}
		}
	}

	WARN_ON((i != nr_pmrs) || (data->nr_fences != fence_count));

	return data;

err_destroy_fences:
	for (i = 0; i < data->nr_fences; i++)
		pvr_fence_destroy(data->fences[i]);
	kfree(data->fences);
err_check_data_free:
	kfree(data);
	return NULL;
}

static void
pvr_buffer_sync_check_fences_destroy(struct pvr_buffer_sync_check_data *data)
{
	int i;

	for (i = 0; i < data->nr_fences; i++)
		pvr_fence_destroy(data->fences[i]);

	kfree(data->fences);
	kfree(data);
}

static void
pvr_buffer_sync_check_data_cleanup(struct fence *fence, struct fence_cb *cb)
{
	struct pvr_buffer_sync_check_data *data =
		container_of(cb, struct pvr_buffer_sync_check_data, base);
	struct pvr_fence *pvr_fence = to_pvr_fence(fence);

	pvr_buffer_sync_check_fences_destroy(data);

	pvr_fence_destroy(pvr_fence);
}

static int
pvr_buffer_sync_append_fences(u32 nr,
			      PRGXFWIF_UFO_ADDR *ufo_addresses,
			      u32 *values,
			      u32 nr_fences,
			      struct pvr_fence **pvr_fences,
			      u32 *nr_out,
			      PRGXFWIF_UFO_ADDR **ufo_addresses_out,
			      u32 **values_out)
{
	u32 nr_new = nr + nr_fences;
	PRGXFWIF_UFO_ADDR *ufo_addresses_new = NULL;
	u32 *values_new = NULL;
	int i;
	int err;

	if (!nr_new)
		goto finish;

	ufo_addresses_new = kmalloc_array(nr_new, sizeof(*ufo_addresses_new),
					  GFP_KERNEL);
	if (!ufo_addresses_new)
		return -ENOMEM;

	values_new = kmalloc_array(nr_new, sizeof(*values_new), GFP_KERNEL);
	if (!values_new) {
		err = -ENOMEM;
		goto err_free_ufo_addresses;
	}

	/* Copy the original data */
	if (nr) {
		memcpy(ufo_addresses_new, ufo_addresses, sizeof(*ufo_addresses) * nr);
		memcpy(values_new, values, sizeof(*values) * nr);
	}

	/* Append the fence data */
	for (i = 0; i < nr_fences; i++) {
		ufo_addresses_new[i + nr].ui32Addr =
			SyncPrimGetFirmwareAddr(pvr_fences[i]->sync);
		values_new[i + nr] = PVR_FENCE_SYNC_VAL_SIGNALED;
	}

finish:
	*nr_out = nr_new;
	*ufo_addresses_out = ufo_addresses_new;
	*values_out = values_new;

	return 0;

err_free_ufo_addresses:
	kfree(ufo_addresses_new);
	return err;
}

int
pvr_buffer_sync_append_start(u32 nr_pmrs,
			     PMR **pmrs,
			     u32 *pmr_flags,
			     u32 nr_checks,
			     PRGXFWIF_UFO_ADDR *check_ufo_addresses,
			     u32 *check_values,
			     u32 nr_updates,
			     PRGXFWIF_UFO_ADDR *update_ufo_addresses,
			     u32 *update_values,
			     struct pvr_buffer_sync_append_data **data_out)
{
	struct pvr_buffer_sync_append_data *data;
	struct pvr_buffer_sync_check_data *check_data;
	int i;
	int err;

	if ((nr_pmrs && !(pmrs && pmr_flags)) ||
	    (nr_updates && (!update_ufo_addresses || !update_values)) ||
	    (nr_checks && (!check_ufo_addresses || !check_values)))
		return -EINVAL;

	for (i = 0; i < nr_pmrs; i++) {
		if (!(pmr_flags[i] & PVR_BUFFER_FLAG_MASK)) {
			pr_err("%s: Invalid flags %#08x for pmr %p\n",
			       __func__, pmr_flags[i], pmrs[i]);
			return -EINVAL;
		}
	}

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

#if defined(NO_HARDWARE)
	/*
	 * For NO_HARDWARE there's no checking or updating of client sync prims
	 * which means SW waits on our fences will cause a deadlock (since they
	 * will never be signalled). Avoid this by not creating any fences.
	 */
	nr_pmrs = 0;
#endif

	if (!nr_pmrs) {
		data->appended = false;
		data->nr_checks = nr_checks;
		data->check_ufo_addresses = check_ufo_addresses;
		data->check_values = check_values;
		data->nr_updates = nr_updates;
		data->update_ufo_addresses = update_ufo_addresses;
		data->update_values = update_values;
		goto finish;
	}

	data->appended = true;
	data->nr_pmrs = nr_pmrs;
	data->pmrs = pmrs;
	data->pmr_flags = pmr_flags;

	err = pvr_buffer_sync_pmrs_lock(nr_pmrs, pmrs);
	if (err) {
		pr_err("%s: failed to lock pmrs (errno=%d)\n",
		       __func__, err);
		goto err_free_data;
	}

	check_data = pvr_buffer_sync_check_fences_create(nr_pmrs, pmrs,
							 pmr_flags);
	if (!check_data) {
		err = -ENOMEM;
		goto err_pmrs_unlock;
	}

	data->update_fence = pvr_fence_create(fence_ctx, "update fence");
	if (!data->update_fence) {
		err = -ENOMEM;
		goto err_free_check_data;
	}

	err = pvr_buffer_sync_append_fences(nr_checks,
					    check_ufo_addresses,
					    check_values,
					    check_data->nr_fences,
					    check_data->fences,
					    &data->nr_checks,
					    &data->check_ufo_addresses,
					    &data->check_values);
	if (err)
		goto err_cleanup_update_fence;

	err = pvr_buffer_sync_append_fences(nr_updates,
					    update_ufo_addresses,
					    update_values,
					    1,
					    &data->update_fence,
					    &data->nr_updates,
					    &data->update_ufo_addresses,
					    &data->update_values);
	if (err)
		goto err_free_data_checks;

	/*
	 * We need to clean up the fences once the HW has finished with them.
	 * We can do this using fence callbacks. However, instead of adding a
	 * callback to every fence, which would result in more work, we can
	 * simply add one to the update fence since this will be the last fence
	 * to be signalled. This callback can do all the necessary clean up.
	 *
	 * Note: we take an additional reference on the update fence in case
	 * it signals before we can add it to a reservation object.
	 */
	fence_get(&data->update_fence->base);

	err = fence_add_callback(&data->update_fence->base, &check_data->base,
				 pvr_buffer_sync_check_data_cleanup);
	if (err) {
		/*
		 * We should only ever get -ENOENT if the fence has already
		 * signalled, which should *never* be the case as we've not
		 * even inserted the fence into a CCB yet!
		 */
		WARN_ON(err != -ENOENT);
		goto err_free_data_updates;
	}

finish:
	*data_out = data;
	return 0;

err_free_data_updates:
	kfree(data->update_ufo_addresses);
	kfree(data->update_values);
err_free_data_checks:
	kfree(data->check_ufo_addresses);
	kfree(data->check_values);
err_cleanup_update_fence:
	pvr_fence_destroy(data->update_fence);
err_free_check_data:
	pvr_buffer_sync_check_fences_destroy(check_data);
err_pmrs_unlock:
	pvr_buffer_sync_pmrs_unlock(nr_pmrs, pmrs);
err_free_data:
	kfree(data);
	return err;
}

void
pvr_buffer_sync_append_finish(struct pvr_buffer_sync_append_data *data)
{
	struct reservation_object *resv;
	int i;

	if (!data->appended)
		goto finish;

	for (i = 0; i < data->nr_pmrs; i++) {
		resv = pmr_reservation_object_get(data->pmrs[i]);
		if (WARN_ON_ONCE(!resv))
			continue;

		if (data->pmr_flags[i] & PVR_BUFFER_FLAG_WRITE) {
			PVR_FENCE_TRACE(&data->update_fence->base,
					"added exclusive fence (%s) to resv %p\n",
					data->update_fence->name, resv);
			reservation_object_add_excl_fence(resv,
							  &data->update_fence->base);
		} else if (data->pmr_flags[i] & PVR_BUFFER_FLAG_READ) {
			PVR_FENCE_TRACE(&data->update_fence->base,
					"added non-exclusive fence (%s) to resv %p\n",
					data->update_fence->name, resv);
			reservation_object_add_shared_fence(resv,
							    &data->update_fence->base);
		}
	}

	/*
	 * Now that the fence has been added to the necessary reservation
	 * objects we can safely drop the extra reference we took in
	 * pvr_buffer_sync_append_start.
	 */
	fence_put(&data->update_fence->base);

	pvr_buffer_sync_pmrs_unlock(data->nr_pmrs, data->pmrs);

	kfree(data->check_ufo_addresses);
	kfree(data->check_values);
	kfree(data->update_ufo_addresses);
	kfree(data->update_values);

finish:
	kfree(data);
}

void
pvr_buffer_sync_append_abort(struct pvr_buffer_sync_append_data *data)
{
	if (!data)
		return;

	if (!data->appended)
		goto finish;

	/*
	 * Signal the fence to trigger clean-up and drop the additional
	 * reference taken in pvr_buffer_sync_append_start.
	 */
	pvr_fence_sync_sw_signal(data->update_fence);
	fence_put(&data->update_fence->base);
	pvr_buffer_sync_pmrs_unlock(data->nr_pmrs, data->pmrs);

	kfree(data->check_ufo_addresses);
	kfree(data->check_values);
	kfree(data->update_ufo_addresses);
	kfree(data->update_values);

finish:
	kfree(data);
}

void
pvr_buffer_sync_append_checks_get(struct pvr_buffer_sync_append_data *data,
				  u32 *nr_checks_out,
				  PRGXFWIF_UFO_ADDR **check_ufo_addresses_out,
				  u32 **check_values_out)
{
	*nr_checks_out = data->nr_checks;
	*check_ufo_addresses_out = data->check_ufo_addresses;
	*check_values_out = data->check_values;
}

void
pvr_buffer_sync_append_updates_get(struct pvr_buffer_sync_append_data *data,
				   u32 *nr_updates_out,
				   PRGXFWIF_UFO_ADDR **update_ufo_addresses_out,
				   u32 **update_values_out)
{
	*nr_updates_out = data->nr_updates;
	*update_ufo_addresses_out = data->update_ufo_addresses;
	*update_values_out = data->update_values;
}

int
pvr_buffer_sync_wait(PMR *pmr, bool intr, unsigned long timeout)
{
	struct reservation_object *resv;
	struct reservation_object_list *resv_list = NULL;
	struct fence *fence, *wait_fence = NULL;
	unsigned seq;
	int i;
	long lerr;

	if (!timeout)
		return -EINVAL;

	resv = pmr_reservation_object_get(pmr);
	if (!resv)
		return 0;

retry:
	seq = read_seqcount_begin(&resv->seq);
	rcu_read_lock();
	resv_list = rcu_dereference(resv->fence);

	if (read_seqcount_retry(&resv->seq, seq))
		goto unlock_retry;

	if (resv_list) {
		for (i = 0; i < resv_list->shared_count; i++) {
			fence = rcu_dereference(resv_list->shared[i]);
			if (is_our_fence(fence_ctx, fence) &&
			    !test_bit(FENCE_FLAG_SIGNALED_BIT, &fence->flags)) {
				wait_fence = fence_get_rcu(fence);
				if (!wait_fence)
					goto unlock_retry;
				break;
			}
		}
	}

	if (!wait_fence) {
		fence = rcu_dereference(resv->fence_excl);

		if (read_seqcount_retry(&resv->seq, seq))
			goto unlock_retry;

		if (fence &&
		    !test_bit(FENCE_FLAG_SIGNALED_BIT, &fence->flags)) {
			wait_fence = fence_get_rcu(fence);
			if (!wait_fence)
				goto unlock_retry;
		}
	}
	rcu_read_unlock();

	if (wait_fence) {
		if (fence_is_signaled(wait_fence))
			lerr = timeout;
		else
			lerr = fence_wait_timeout(wait_fence, intr, timeout);

		fence_put(wait_fence);
		if (!lerr)
			return -EBUSY;
		else if (lerr < 0)
			return (int)lerr;
	}

	return 0;

unlock_retry:
	rcu_read_unlock();
	goto retry;
}

int
pvr_buffer_sync_init(void)
{
	PVRSRV_ERROR srv_err;
	int err;

	srv_err = PVRSRVAcquireDeviceDataKM(0, PVRSRV_DEVICE_TYPE_RGX,
					    &dev_cookie);
	if (srv_err != PVRSRV_OK) {
		pr_err("%s: Failed to acquire device data (%s)\n",
		       __func__, PVRSRVGetErrorStringKM(srv_err));
		return -ENODEV;
	}

	fence_ctx = pvr_fence_context_create(dev_cookie, "rogue-gpu");
	if (!fence_ctx) {
		err = -ENOMEM;
		goto err_release_dev_data;
	}

	return 0;

err_release_dev_data:
	PVRSRVReleaseDeviceDataKM(dev_cookie);
	return err;
}

void
pvr_buffer_sync_deinit(void)
{
	pvr_fence_context_destroy(fence_ctx);
	PVRSRVReleaseDeviceDataKM(dev_cookie);
}
