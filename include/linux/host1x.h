/*
 * Copyright (c) 2009-2013, NVIDIA Corporation. All rights reserved.
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

#ifndef __LINUX_HOST1X_H
#define __LINUX_HOST1X_H

#include <linux/device.h>
#include <linux/types.h>
#include <uapi/linux/host1x.h>
#include <linux/devfreq.h>

enum host1x_class {
	HOST1X_CLASS_HOST1X = 0x1,
	HOST1X_CLASS_NVENC = 0x21,
	HOST1X_CLASS_VI = 0x30,
	HOST1X_CLASS_ISPA = 0x32,
	HOST1X_CLASS_ISPB = 0x34,
	HOST1X_CLASS_GR2D = 0x51,
	HOST1X_CLASS_GR2D_SB = 0x52,
	HOST1X_CLASS_VIC = 0x5D,
	HOST1X_CLASS_GR3D = 0x60,
	HOST1X_CLASS_NVJPG = 0xC0,
	HOST1X_CLASS_NVDEC = 0xF0,
};

struct host1x_user_constraint {
	unsigned long rate;
	int type;
};

struct host1x_user {
	struct list_head node;
	struct host1x_user_constraint constraint[HOST1X_CLOCK_INDEX_MAX];
};

struct host1x_client;

struct host1x_client_ops {
	int (*init)(struct host1x_client *client);
	int (*exit)(struct host1x_client *client);
	int (*get_clk_rate)(struct host1x_client *client, u64 *data,
			u32 type);
	int (*set_clk_rate)(struct host1x_client *client, u64 data,
			u32 type);
};

struct host1x_client_clock {
	const char *clk_name;
	struct clk *clk;
	unsigned long default_rate;
	unsigned long valid_constraint_types;
	/*
	 * Only valid if this clock is get/settable via
	 * HOST1X_USER_CONSTRAINT_TYPE_PIXELRATE.
	 */
	int pixels_per_cycle;
};

struct host1x_actmon_data {
	/* Inputs prior to initializing actmon */
	u32 actmon_regs;
	unsigned int actmon_irq;
	const char *devfreq_governor;

	struct host1x_user user;

	struct host1x_actmon *actmon;

	struct clk *clk;

	struct dentry *debugfs;
	struct devfreq *power_manager;

	ktime_t last_event_time;

	struct devfreq_dev_profile devfreq_profile;
	struct devfreq_dev_status dev_stat;
};

struct host1x_client {
	struct list_head list;
	struct device *parent;
	struct device *dev;

	const struct host1x_client_ops *ops;

	enum host1x_class class;
	struct host1x_channel *channel;

	struct host1x_syncpt **syncpts;
	unsigned int num_syncpts;

	/*
	 * This client's clocks, each clock's default rate, and the type of
	 * constraint that makes sense for this particular clock.
	 */
	struct host1x_client_clock clocks[HOST1X_CLOCK_INDEX_MAX];

	/* List of users and rate requests */
	struct list_head user_list;
	struct mutex user_list_lock;

	struct host1x_actmon_data *actmon_data;
};

/*
 * host1x buffer objects
 */

struct host1x_bo;
struct sg_table;

struct host1x_bo_ops {
	struct host1x_bo *(*get)(struct host1x_bo *bo);
	void (*put)(struct host1x_bo *bo);
	int (*pin)(struct host1x_bo *bo, struct sg_table **sgt, dma_addr_t *addr);
	void (*unpin)(struct host1x_bo *bo, struct sg_table *sgt);
	void *(*mmap)(struct host1x_bo *bo);
	void (*munmap)(struct host1x_bo *bo, void *addr);
	void *(*kmap)(struct host1x_bo *bo, unsigned int pagenum);
	void (*kunmap)(struct host1x_bo *bo, unsigned int pagenum, void *addr);
};

struct host1x_bo {
	const struct host1x_bo_ops *ops;
};

static inline void host1x_bo_init(struct host1x_bo *bo,
				  const struct host1x_bo_ops *ops)
{
	bo->ops = ops;
}

static inline struct host1x_bo *host1x_bo_get(struct host1x_bo *bo)
{
	return bo->ops->get(bo);
}

static inline void host1x_bo_put(struct host1x_bo *bo)
{
	bo->ops->put(bo);
}

static inline int host1x_bo_pin(struct host1x_bo *bo,
				       struct sg_table **sgt, dma_addr_t *addr)
{
	return bo->ops->pin(bo, sgt, addr);
}

static inline void host1x_bo_unpin(struct host1x_bo *bo, struct sg_table *sgt)
{
	bo->ops->unpin(bo, sgt);
}

static inline void *host1x_bo_mmap(struct host1x_bo *bo)
{
	return bo->ops->mmap(bo);
}

static inline void host1x_bo_munmap(struct host1x_bo *bo, void *addr)
{
	bo->ops->munmap(bo, addr);
}

static inline void *host1x_bo_kmap(struct host1x_bo *bo, unsigned int pagenum)
{
	return bo->ops->kmap(bo, pagenum);
}

static inline void host1x_bo_kunmap(struct host1x_bo *bo,
				    unsigned int pagenum, void *addr)
{
	bo->ops->kunmap(bo, pagenum, addr);
}

/*
 * host1x syncpoints
 */

#define HOST1X_SYNCPT_CLIENT_MANAGED	(1 << 0)
#define HOST1X_SYNCPT_HAS_BASE		(1 << 1)

struct host1x_syncpt_base;
struct host1x_syncpt;
struct host1x;

struct host1x_syncpt *host1x_syncpt_get(struct host1x *host, u32 id);
u32 host1x_syncpt_id(struct host1x_syncpt *sp);
u32 host1x_syncpt_read_min(struct host1x_syncpt *sp);
u32 host1x_syncpt_read_max(struct host1x_syncpt *sp);
u32 host1x_syncpt_read(struct host1x_syncpt *sp);
int host1x_syncpt_incr(struct host1x_syncpt *sp);
u32 host1x_syncpt_incr_max(struct host1x_syncpt *sp, u32 incrs);
int host1x_syncpt_wait(struct host1x_syncpt *sp, u32 thresh, long timeout,
		       u32 *value);
struct host1x_syncpt *host1x_syncpt_request(struct device *dev,
					    unsigned long flags);
void host1x_syncpt_free(struct host1x_syncpt *sp);

struct host1x_syncpt_base *host1x_syncpt_get_base(struct host1x_syncpt *sp);
u32 host1x_syncpt_base_id(struct host1x_syncpt_base *base);

/*
 * host1x channel
 */

struct host1x_channel;
struct host1x_job;

struct host1x_channel *host1x_channel_request(struct host1x_client *client);
void host1x_channel_free(struct host1x_channel *channel);
struct host1x_channel *host1x_channel_get(struct host1x_channel *channel,
					  struct host1x_user *user);
void host1x_channel_put(struct host1x_channel *channel,
			struct host1x_user *user);
int host1x_job_submit(struct host1x_job *job);

/*
 * host1x job
 */

struct host1x_reloc {
	struct {
		struct host1x_bo *bo;
		unsigned long offset;
	} cmdbuf;
	struct {
		struct host1x_bo *bo;
		unsigned long offset;
	} target;
	unsigned long shift;
};

struct host1x_job_syncpt {
	u32 id;
	u32 incrs;
	u32 end;
};

struct host1x_job {
	/* When refcount goes to zero, job can be freed */
	struct kref ref;

	/* List entry */
	struct list_head list;

	/* Channel where job is submitted to */
	struct host1x_channel *channel;

	u32 client;

	/* Gathers and their memory */
	struct host1x_job_gather *gathers;
	unsigned int num_gathers;

	/* Wait checks to be processed at submit time */
	struct host1x_waitchk *waitchk;
	unsigned int num_waitchk;
	u32 waitchk_mask;

	/* Array of handles to be pinned & unpinned */
	struct host1x_reloc *relocarray;
	unsigned int num_relocs;
	struct host1x_job_unpin_data *unpins;
	unsigned int num_unpins;

	dma_addr_t *addr_phys;
	dma_addr_t *gather_addr_phys;
	dma_addr_t *reloc_addr_phys;

	/* Sync point ids, numbers of increments and ends related to the
	 * submit */
	unsigned int num_syncpts;
	struct host1x_job_syncpt *syncpts;

	/* Maximum time to wait for this job */
	unsigned int timeout;

	/* Index and number of slots used in the push buffer */
	unsigned int first_get;
	unsigned int num_slots;

	/* Copy of gathers */
	size_t gather_copy_size;
	dma_addr_t gather_copy;
	u8 *gather_copy_mapped;

	/* Check if register is marked as an address reg */
	int (*is_addr_reg)(struct device *dev, u32 reg, u32 class);

	/* Function to reset the engine in case timeout occurs */
	void (*reset)(struct device *dev);

	/* Request a SETCLASS to this class */
	u32 class;

	/* Add a channel wait for previous ops to complete */
	bool serialize;
};

struct host1x_job *host1x_job_alloc(struct host1x_channel *ch,
				    u32 num_cmdbufs, u32 num_relocs,
				    u32 num_waitchks, u32 num_syncpts);
void host1x_job_add_gather(struct host1x_job *job, struct host1x_bo *mem_id,
			   u32 words, u32 offset);
struct host1x_job *host1x_job_get(struct host1x_job *job);
void host1x_job_put(struct host1x_job *job);
int host1x_job_pin(struct host1x_job *job, struct device *dev);
void host1x_job_unpin(struct host1x_job *job);
void host1x_job_bo_put(struct host1x_job *job);

/*
 * subdevice probe infrastructure
 */

struct host1x_device;

struct host1x_driver {
	struct device_driver driver;

	const struct of_device_id *subdevs;
	struct list_head list;

	int (*probe)(struct host1x_device *device);
	int (*remove)(struct host1x_device *device);
	void (*shutdown)(struct host1x_device *device);
};

static inline struct host1x_driver *
to_host1x_driver(struct device_driver *driver)
{
	return container_of(driver, struct host1x_driver, driver);
}

int host1x_driver_register_full(struct host1x_driver *driver,
				struct module *owner);
void host1x_driver_unregister(struct host1x_driver *driver);

#define host1x_driver_register(driver) \
	host1x_driver_register_full(driver, THIS_MODULE)

struct host1x_device {
	struct host1x_driver *driver;
	struct list_head list;
	struct device dev;

	struct mutex subdevs_lock;
	struct list_head subdevs;
	struct list_head active;

	struct mutex clients_lock;
	struct list_head clients;

	bool registered;
};

static inline struct host1x_device *to_host1x_device(struct device *dev)
{
	return container_of(dev, struct host1x_device, dev);
}

int host1x_device_init(struct host1x_device *device);
int host1x_device_exit(struct host1x_device *device);

int host1x_client_register(struct host1x_client *client);
int host1x_client_unregister(struct host1x_client *client);

struct tegra_mipi_device;

struct tegra_mipi_device *tegra_mipi_request(struct device *device);
void tegra_mipi_free(struct tegra_mipi_device *device);
int tegra_mipi_calibrate(struct tegra_mipi_device *device);

int host1x_module_get_clocks(struct host1x_client *client);
void host1x_module_put_clocks(struct host1x_client *client);
int host1x_module_enable_clocks(struct host1x_client *client);
void host1x_module_disable_clocks(struct host1x_client *client);
int host1x_module_busy(struct host1x_client *client);
void host1x_module_idle(struct host1x_client *client);
void host1x_module_idle_mult(struct host1x_client *client, int refs);
int host1x_module_get_rate(struct host1x_client *client,
			   struct host1x_user *user,
			   enum host1x_clock_index index,
			   unsigned long *rate,
			   enum host1x_user_constraint_type type);
int host1x_module_set_rate(struct host1x_client *client,
			   struct host1x_user *user,
			   enum host1x_clock_index index,
			   unsigned long rate,
			   enum host1x_user_constraint_type type);
int host1x_actmon_init(struct host1x_client *client,
		       struct host1x_actmon_data *actmon_data);
void host1x_actmon_deinit(struct host1x_client *client);
int host1x_actmon_enable(struct host1x_client *client);
void host1x_actmon_disable(struct host1x_client *client);

#endif
