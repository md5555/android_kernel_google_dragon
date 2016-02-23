/*
 * Copyright (C) 2012 Avionic Design GmbH
 * Copyright (C) 2012-2013 NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef HOST1X_DRM_H
#define HOST1X_DRM_H 1

#include <uapi/drm/tegra_drm.h>
#include <linux/host1x.h>

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_edid.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_fixed.h>

#include <soc/tegra/mc.h>

#include "gem.h"


/**
 * [Copied from intel_drv.h, modified to allow finer grained intervals]
 *
 * _wait_for - magic (register) wait macro
 *
 * Does the right thing for modeset paths when run under kdgb or similar atomic
 * contexts. Note that it's important that we check the condition again after
 * having timed out, since the timeout could be due to preemption or similar and
 * we've never had a chance to check the condition before the timeout.
 */
#define _wait_for(COND, MS, W) ({ \
	unsigned long timeout__ = jiffies + msecs_to_jiffies(MS) + 1;	\
	int ret__ = 0;							\
	while (!(COND)) {						\
		if (time_after(jiffies, timeout__)) {			\
			if (!(COND))					\
				ret__ = -ETIMEDOUT;			\
			break;						\
		}							\
		if ((W) && drm_can_sleep()) {				\
			usleep_range((W), (W)*2);			\
		} else {						\
			cpu_relax();					\
		}							\
	}								\
	ret__;								\
})
#define wait_for(COND, MS) _wait_for(COND, MS, 1000)
#define wait_for_interval(COND, MS, W_US) _wait_for(COND, MS, W_US)
#define wait_for_atomic(COND, MS) _wait_for(COND, MS, 0)

struct reset_control;

struct tegra_fb {
	struct drm_framebuffer base;
	struct tegra_bo **planes;
	unsigned int num_planes;
};

#ifdef CONFIG_DRM_TEGRA_FBDEV
struct tegra_fbdev {
	struct drm_fb_helper base;
	struct tegra_fb *fb;
};
#endif

struct tegra_drm {
	struct drm_device *drm;

	struct iommu_domain *domain;
	struct drm_mm mm;

	struct mutex clients_lock;
	struct list_head clients;

#ifdef CONFIG_DRM_TEGRA_FBDEV
	struct tegra_fbdev *fbdev;
#endif

	unsigned int pitch_align;

	struct {
		struct drm_atomic_state *state;
		struct work_struct work;
		struct mutex lock;
	} commit;
};

struct tegra_drm_client;

struct tegra_drm_context {
	struct tegra_drm_client *client;
	struct host1x_channel *channel;
	struct list_head list;
	struct mutex lock;
	bool keepon;
	struct host1x_user user;
};

struct tegra_drm_client_ops {
	int (*open_channel)(struct tegra_drm_client *client,
			    struct tegra_drm_context *context);
	void (*close_channel)(struct tegra_drm_context *context);
	void (*reset)(struct device *dev);
	int (*is_addr_reg)(struct device *dev, u32 class, u32 offset);
	int (*submit)(struct tegra_drm_context *context,
		      struct drm_tegra_submit *args, struct drm_device *drm,
		      struct drm_file *file);
};

int tegra_drm_submit(struct tegra_drm_context *context,
		     struct drm_tegra_submit *args, struct drm_device *drm,
		     struct drm_file *file);

struct tegra_drm_client {
	struct host1x_client base;
	struct list_head list;

	const struct tegra_drm_client_ops *ops;
};

static inline struct tegra_drm_client *
host1x_to_drm_client(struct host1x_client *client)
{
	return container_of(client, struct tegra_drm_client, base);
}

int tegra_drm_register_client(struct tegra_drm *tegra,
			      struct tegra_drm_client *client);
int tegra_drm_unregister_client(struct tegra_drm *tegra,
				struct tegra_drm_client *client);

int tegra_drm_init(struct tegra_drm *tegra, struct drm_device *drm);
int tegra_drm_exit(struct tegra_drm *tegra);

struct tegra_dc_soc_info;
struct tegra_output;

struct tegra_dc_stats {
	unsigned long frames;
	unsigned long vblank;
	unsigned long underflow;
	unsigned long overflow;
};

struct tegra_dc {
	struct host1x_client client;
	struct host1x_syncpt *syncpt;
	struct device *dev;
	spinlock_t lock;

	struct drm_crtc base;
	int powergate;
	int pipe;

	struct clk *clk;
	struct reset_control *rst;
	struct clk *emc_clk;
	struct clk *emc_la_clk;
	void __iomem *regs;
	int irq;

	struct tegra_output *rgb;

	struct tegra_dc_stats stats;
	struct list_head list;

	struct drm_info_list *debugfs_files;
	struct drm_minor *minor;
	struct dentry *debugfs;

	struct notifier_block slcg_notifier;

	/* page-flip handling */
	struct drm_pending_vblank_event *event;

	const struct tegra_dc_soc_info *soc;

	struct iommu_domain *domain;

	int dpms;
	bool vblank_enabled;
	bool in_modeset;

	struct tegra_mc *mc;
	unsigned int *mc_win_clients;
};

static inline struct tegra_dc *
host1x_client_to_dc(struct host1x_client *client)
{
	return container_of(client, struct tegra_dc, client);
}

static inline struct tegra_dc *to_tegra_dc(struct drm_crtc *crtc)
{
	return crtc ? container_of(crtc, struct tegra_dc, base) : NULL;
}

static inline void tegra_dc_writel(struct tegra_dc *dc, u32 value,
				   unsigned long offset)
{
	writel(value, dc->regs + (offset << 2));
}

static inline u32 tegra_dc_readl(struct tegra_dc *dc, unsigned long offset)
{
	return readl(dc->regs + (offset << 2));
}

struct tegra_dc_window {
	/* Source values are 16.16 fixed point */
	struct {
		u32 x;
		u32 y;
		u32 w;
		u32 h;
	} src;
	struct {
		unsigned int x;
		unsigned int y;
		unsigned int w;
		unsigned int h;
	} dst;
	unsigned int bits_per_pixel;
	unsigned int stride[2];
	unsigned long base[3];
	bool bottom_up;
	bool scan_column;
	bool right_left;

	struct tegra_bo_tiling tiling;
	u32 format;
	u32 swap;
	u8 alpha;
};

/* from dc.c */
u32 tegra_dc_get_vblank_counter(struct tegra_dc *dc);
void tegra_dc_enable_vblank(struct tegra_dc *dc);
void tegra_dc_disable_vblank(struct tegra_dc *dc);
void tegra_dc_cancel_page_flip(struct drm_crtc *crtc, struct drm_file *file);
void tegra_dc_commit(struct tegra_dc *dc);
int tegra_dc_state_setup_clock(struct tegra_dc *dc,
			       struct drm_crtc_state *crtc_state,
			       struct clk *clk, unsigned long pclk,
			       unsigned int div);
void tegra_dc_update_emc_pre_commit(struct drm_atomic_state *old_state);
void tegra_dc_update_emc_post_commit(struct drm_atomic_state *old_state);
int tegra_dc_evaluate_bandwidth(struct drm_atomic_state *state);
void tegra_dc_force_update(struct drm_crtc *crtc);

struct tegra_output {
	struct device_node *of_node;
	struct device *dev;

	struct drm_panel *panel;
	struct i2c_adapter *ddc;
	const struct edid *edid;
	unsigned int hpd_irq;
	int hpd_gpio;

	struct drm_encoder encoder;
	struct drm_connector connector;
};

static inline struct tegra_output *encoder_to_output(struct drm_encoder *e)
{
	return container_of(e, struct tegra_output, encoder);
}

static inline struct tegra_output *connector_to_output(struct drm_connector *c)
{
	return container_of(c, struct tegra_output, connector);
}

/* from rgb.c */
int tegra_dc_rgb_probe(struct tegra_dc *dc);
int tegra_dc_rgb_remove(struct tegra_dc *dc);
int tegra_dc_rgb_init(struct drm_device *drm, struct tegra_dc *dc);
int tegra_dc_rgb_exit(struct tegra_dc *dc);

/* from output.c */
int tegra_output_probe(struct tegra_output *output);
void tegra_output_remove(struct tegra_output *output);
int tegra_output_init(struct drm_device *drm, struct tegra_output *output);
void tegra_output_exit(struct tegra_output *output);

int tegra_output_connector_get_modes(struct drm_connector *connector);
struct drm_encoder *
tegra_output_connector_best_encoder(struct drm_connector *connector);
enum drm_connector_status
tegra_output_connector_detect(struct drm_connector *connector, bool force);
void tegra_output_connector_destroy(struct drm_connector *connector);

void tegra_output_encoder_destroy(struct drm_encoder *encoder);

/* from dpaux.c */
struct tegra_dpaux;
struct drm_dp_link;

struct tegra_dpaux *tegra_dpaux_find_by_of_node(struct device_node *np);
enum drm_connector_status tegra_dpaux_detect(struct tegra_dpaux *dpaux);
int tegra_dpaux_attach(struct tegra_dpaux *dpaux, struct tegra_output *output);
int tegra_dpaux_detach(struct tegra_dpaux *dpaux);
int tegra_dpaux_enable(struct tegra_dpaux *dpaux);
int tegra_dpaux_disable(struct tegra_dpaux *dpaux);
int tegra_dpaux_prepare(struct tegra_dpaux *dpaux, u8 encoding);
int tegra_dpaux_train(struct tegra_dpaux *dpaux, struct drm_dp_link *link,
		      u8 pattern);

/* from fb.c */
struct tegra_bo *tegra_fb_get_plane(struct drm_framebuffer *framebuffer,
				    unsigned int index);
bool tegra_fb_is_bottom_up(struct drm_framebuffer *framebuffer);
int tegra_fb_get_tiling(struct drm_framebuffer *framebuffer,
			struct tegra_bo_tiling *tiling);
struct drm_framebuffer *tegra_fb_create(struct drm_device *drm,
					struct drm_file *file,
					const struct drm_mode_fb_cmd2 *cmd);
int tegra_drm_fb_prepare(struct drm_device *drm);
void tegra_drm_fb_free(struct drm_device *drm);
int tegra_drm_fb_init(struct drm_device *drm);
void tegra_drm_fb_exit(struct drm_device *drm);
#ifdef CONFIG_DRM_TEGRA_FBDEV
void tegra_fbdev_restore_mode(struct tegra_fbdev *fbdev);
void tegra_fb_output_poll_changed(struct drm_device *drm);
#endif

extern struct platform_driver tegra_dc_driver;
extern struct platform_driver tegra_dsi_driver;
extern struct platform_driver tegra_sor_driver;
extern struct platform_driver tegra_hdmi_driver;
extern struct platform_driver tegra_dpaux_driver;
extern struct platform_driver tegra_gr2d_driver;
extern struct platform_driver tegra_gr3d_driver;
extern struct platform_driver tegra_vic_driver;
extern struct platform_driver tegra_nvenc_driver;
extern struct platform_driver tegra_nvdec_driver;
extern struct platform_driver tegra_nvjpg_driver;
extern struct platform_driver tegra_isp_driver;
extern struct platform_driver tegra_vi_driver;

#endif /* HOST1X_DRM_H */
