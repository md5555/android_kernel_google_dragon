#ifndef __NOUVEAU_CHAN_H__
#define __NOUVEAU_CHAN_H__

#include <linux/wait.h>

#include <nvif/object.h>
struct nvif_device;
struct drm_file;

struct nouveau_channel {
	struct nvif_device *device;
	struct nouveau_drm *drm;

	int chid;

	struct nvif_object vram;
	struct nvif_object gart;
	struct nvif_object nvsw;

	struct {
		struct nouveau_bo *buffer;
		struct nvkm_vma vma;
		struct nvif_object ctxdma;
	} push;

	/* TODO: this will be reworked in the near future */
	bool accel_done;
	void *fence;
	struct {
		int max;
		int free;
		int cur;
		int put;
		int ib_base;
		int ib_max;
		int ib_free;
		int ib_put;
	} dma;
	u32 user_get_hi;
	u32 user_get;
	u32 user_put;

	struct nvif_object *object;

	struct mutex fifo_lock;
	spinlock_t pushbuf_lock;
	struct list_head pushbuf_queue;
	struct task_struct *pushbuf_thread;
	wait_queue_head_t pushbuf_waitqueue;

	struct {
		struct nouveau_bo *buffer;
		u32 offset;
		struct nvif_notify notify;
	} error_notifier;

	bool faulty;
	struct mutex recovery_lock;
};


int  nouveau_channel_new(struct nouveau_drm *, struct nvif_device *,
			 u32 handle, u32 arg0, u32 arg1,
			 struct nouveau_channel **);
void nouveau_channel_del(struct nouveau_channel **);
int  nouveau_channel_idle(struct nouveau_channel *);
int nouveau_channel_init_error_notifier(struct nouveau_channel *chan,
					struct drm_file *file,
					u32 handle, u32 offset);

extern int nouveau_vram_pushbuf;

#endif
