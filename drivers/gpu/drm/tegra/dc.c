/*
 * Copyright (C) 2012 Avionic Design GmbH
 * Copyright (C) 2012 NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/iommu.h>
#include <linux/of_platform.h>
#include <linux/reset.h>

#include <soc/tegra/pmc.h>
#include <soc/tegra/tegra-dvfs.h>
#include <soc/tegra/tegra_emc.h>

#include "dc.h"
#include "drm.h"
#include "gem.h"

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_plane_helper.h>

#define TEGRA_DC_ENABLE_STANDBY 0

enum la_ptsa_update {
	LA_PTSA_PRE_UPDATE,
	LA_PTSA_POST_UPDATE,
};

struct tegra_dc_window_soc_info {
	bool supports_h_filter;
	bool supports_v_filter;
	bool supports_planar_rotation;
};

struct tegra_dc_scale_limit {
	unsigned int bpp;
	unsigned int limit;
};

struct tegra_dc_soc_info {
	bool supports_border_color;
	bool supports_interlacing;
	bool supports_cursor;
	bool supports_block_linear;
	unsigned int pitch_align;
	bool has_powergate;
	bool supports_v2_blend;
	bool supports_scan_column;
	const struct tegra_dc_window_soc_info *windows;
	unsigned int num_windows;
	unsigned int num_primary_plane_formats;
	const u32 *primary_plane_formats;
	unsigned int num_overlay_plane_formats;
	const u32 *overlay_plane_formats;
	unsigned int num_h_scale_down_limits;
	const struct tegra_dc_scale_limit *h_scale_down_limits;
	unsigned int num_v_scale_down_limits;
	const struct tegra_dc_scale_limit *v_scale_down_limits;
};

struct tegra_plane {
	struct drm_plane base;
	unsigned int index;
};

static inline struct tegra_plane *to_tegra_plane(struct drm_plane *plane)
{
	return container_of(plane, struct tegra_plane, base);
}

struct tegra_dc_state {
	struct drm_crtc_state base;

	struct clk *clk;
	unsigned long pclk;
	unsigned int div;

	bool nc_mode;
	u8 tearing_effect;

	u32 planes;
	unsigned long emc_bandwidth; /* kbps */
	bool update_emc; /* set to false when the emc has been updated */
	bool update_la_ptsa; /* set to true if plane's emc need to updated */
};

static inline struct tegra_dc_state *to_dc_state(struct drm_crtc_state *state)
{
	if (state)
		return container_of(state, struct tegra_dc_state, base);

	return NULL;
}

struct tegra_plane_state {
	struct drm_plane_state base;

	struct tegra_bo_tiling tiling;
	u32 format;
	u32 swap;

	u64 plane_emc_bw;
	enum la_ptsa_update plane_update_la_ptsa;
};

static inline struct tegra_plane_state *
to_tegra_plane_state(struct drm_plane_state *state)
{
	if (state)
		return container_of(state, struct tegra_plane_state, base);

	return NULL;
}

static void tegra_dc_stats_reset(struct tegra_dc_stats *stats)
{
	stats->frames = 0;
	stats->vblank = 0;
	stats->underflow = 0;
	stats->overflow = 0;
}

static int tegra_dc_clk_enable(struct tegra_dc *dc)
{
	int err;

	err = clk_prepare_enable(dc->clk);
	if (err < 0) {
		dev_err(dc->dev,
			"failed to enable dc clock: %d\n", err);
		return err;
	}

	if (dc->emc_clk) {
		err = clk_prepare_enable(dc->emc_clk);
		if (err < 0) {
			dev_err(dc->dev,
				"failed to enable emc clock: %d\n",
				err);
			goto err_emc_clk;
		}
	}


	if (dc->emc_la_clk) {
		err = clk_prepare_enable(dc->emc_la_clk);
		if (err < 0) {
			dev_err(dc->dev,
				"failed to enable emc la clock: %d\n",
				err);
			goto err_emc_la_clk;
		}
	}

	return 0;

err_emc_la_clk:
	clk_disable_unprepare(dc->emc_clk);
err_emc_clk:
	clk_disable_unprepare(dc->clk);

	return err;
}

static void tegra_dc_clk_disable(struct tegra_dc *dc)
{
	clk_disable_unprepare(dc->clk);
	if (dc->emc_clk)
		clk_disable_unprepare(dc->emc_clk);
	if (dc->emc_la_clk)
		clk_disable_unprepare(dc->emc_la_clk);
}

/*
 * Reads the active copy of a register. This takes the dc->lock spinlock to
 * prevent races with the VBLANK processing which also needs access to the
 * active copy of some registers.
 */
static u32 tegra_dc_readl_active(struct tegra_dc *dc, unsigned long offset)
{
	unsigned long flags;
	u32 value;

	spin_lock_irqsave(&dc->lock, flags);

	tegra_dc_writel(dc, READ_MUX, DC_CMD_STATE_ACCESS);
	value = tegra_dc_readl(dc, offset);
	tegra_dc_writel(dc, 0, DC_CMD_STATE_ACCESS);

	spin_unlock_irqrestore(&dc->lock, flags);
	return value;
}

/*
 * Double-buffered registers have two copies: ASSEMBLY and ACTIVE. When the
 * *_ACT_REQ bits are set the ASSEMBLY copy is latched into the ACTIVE copy.
 * Latching happens mmediately if the display controller is in STOP mode or
 * on the next frame boundary otherwise.
 *
 * Triple-buffered registers have three copies: ASSEMBLY, ARM and ACTIVE. The
 * ASSEMBLY copy is latched into the ARM copy immediately after *_UPDATE bits
 * are written. When the *_ACT_REQ bits are written, the ARM copy is latched
 * into the ACTIVE copy, either immediately if the display controller is in
 * STOP mode, or at the next frame boundary otherwise.
 */
void tegra_dc_commit(struct tegra_dc *dc)
{
	tegra_dc_writel(dc, GENERAL_ACT_REQ << 8, DC_CMD_STATE_CONTROL);
	tegra_dc_writel(dc, GENERAL_ACT_REQ, DC_CMD_STATE_CONTROL);
}

static int tegra_dc_format(u32 fourcc, u32 *format, u32 *swap)
{
	/* assume no swapping of fetched data */
	if (swap)
		*swap = BYTE_SWAP_NOSWAP;

	switch (fourcc) {
	case DRM_FORMAT_ABGR8888:
	case DRM_FORMAT_XBGR8888:
		*format = WIN_COLOR_DEPTH_R8G8B8A8;
		break;

	case DRM_FORMAT_RGBA8888:
		if (swap)
			*swap = BYTE_SWAP_SWAP4;
		*format = WIN_COLOR_DEPTH_R8G8B8A8;
		break;

	case DRM_FORMAT_ARGB8888:
	case DRM_FORMAT_XRGB8888:
		*format = WIN_COLOR_DEPTH_B8G8R8A8;
		break;

	case DRM_FORMAT_BGR565:
		*format = WIN_COLOR_DEPTH_B5G6R5;
		break;

	case DRM_FORMAT_RGB565:
		*format = WIN_COLOR_DEPTH_R5G6B5;
		break;

	case DRM_FORMAT_UYVY:
		*format = WIN_COLOR_DEPTH_YCbCr422;
		break;

	case DRM_FORMAT_YUYV:
		if (swap)
			*swap = BYTE_SWAP_SWAP2;

		*format = WIN_COLOR_DEPTH_YCbCr422;
		break;

	case DRM_FORMAT_YUV420:
		*format = WIN_COLOR_DEPTH_YCbCr420P;
		break;

	case DRM_FORMAT_YUV422:
		*format = WIN_COLOR_DEPTH_YCbCr422P;
		break;

	case DRM_FORMAT_NV12:
		*format = WIN_COLOR_DEPTH_YCbCr420SP;
		break;

	case DRM_FORMAT_NV21:
		*format = WIN_COLOR_DEPTH_YCrCb420SP;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static bool tegra_dc_format_is_yuv(unsigned int format, bool *planar)
{
	switch (format) {
	case WIN_COLOR_DEPTH_YCbCr422:
	case WIN_COLOR_DEPTH_YUV422:
		if (planar)
			*planar = false;

		return true;

	case WIN_COLOR_DEPTH_YCbCr420P:
	case WIN_COLOR_DEPTH_YUV420P:
	case WIN_COLOR_DEPTH_YCbCr422P:
	case WIN_COLOR_DEPTH_YUV422P:
	case WIN_COLOR_DEPTH_YCbCr422R:
	case WIN_COLOR_DEPTH_YUV422R:
	case WIN_COLOR_DEPTH_YCbCr422RA:
	case WIN_COLOR_DEPTH_YUV422RA:
	case WIN_COLOR_DEPTH_YCrCb420SP:
	case WIN_COLOR_DEPTH_YCbCr420SP:
		if (planar)
			*planar = true;

		return true;
	}

	if (planar)
		*planar = false;

	return false;
}

static inline u32 dfixed16_div(u32 A, u32 B)
{
	u64 tmp = ((u64)A << 17);

	do_div(tmp, B);
	tmp += 1;
	tmp /= 2;
	return lower_32_bits(tmp);
}

static inline u32 compute_dda_inc(u32 in, unsigned int out_int, bool v,
				  unsigned int bpp)
{
	u32 out = (out_int << 16);
	u32 one = (1 << 16);
	u32 dda_inc;
	int max;

	if (v)
		max = 15;
	else {
		switch (bpp) {
		case 2:
			max = 8;
			break;

		default:
			WARN_ON_ONCE(1);
			/* fallthrough */
		case 4:
			max = 4;
			break;
		}
	}

	out = max_t(u32, out - one, one);
	in -= one;

	dda_inc = dfixed16_div(in, out);
	dda_inc = min_t(u32, dda_inc, (max << 16));

	/* DDA based dc registers take value in 4.12 format */
	return (dda_inc >> 4);
}

static inline u32 compute_initial_dda(u32 in)
{
	/* DDA based dc registers take value in 4.12 format */
	return ((in & ((1 << 16) - 1)) >> 4);
}

/*
 * Initialize horizontal and vertical scaling filter coefficients.
 * Each 32-bit phase register represents 6 filter coefficients and
 * sum of all coefficients for each of these 16 phases should be
 * typically 128.
 */
static void tegra_dc_set_scaling_filter(struct tegra_dc *dc)
{
	unsigned i, v0 = 128, v1 = 0;

	for (i = 0; i < 16; i++) {
		tegra_dc_writel(dc, (v1 << 16) | (v0 << 8),
				DC_WIN_H_FILTER_P(i));

		tegra_dc_writel(dc, v0,
				DC_WIN_V_FILTER_P(i));
		v0 -= 8;
		v1 += 8;
	}
}

static inline bool win_use_h_filter(struct tegra_dc *dc,
				   const struct tegra_dc_window *window,
				   unsigned int index)
{
	return dc->soc->windows[index].supports_h_filter &&
		(window->src.w != (window->dst.w << 16));
}

static inline bool win_use_v_filter(struct tegra_dc *dc,
				   const struct tegra_dc_window *window,
				   unsigned int index)
{
	return dc->soc->windows[index].supports_v_filter &&
		(window->src.h != (window->dst.h << 16));
}

static void tegra_dc_setup_window(struct tegra_dc *dc, unsigned int index,
				  const struct tegra_dc_window *window)
{
	unsigned h_offset, v_offset, h_size, v_size, h_dda, v_dda, bpp;
	unsigned long value, flags;
	bool yuv, planar;

	/*
	 * For YUV planar modes, the number of bytes per pixel takes into
	 * account only the luma component and therefore is 1.
	 */
	yuv = tegra_dc_format_is_yuv(window->format, &planar);
	if (!yuv)
		bpp = window->bits_per_pixel / 8;
	else
		bpp = planar ? 1 : 2;

	spin_lock_irqsave(&dc->lock, flags);

	value = WINDOW_A_SELECT << index;
	tegra_dc_writel(dc, value, DC_CMD_DISPLAY_WINDOW_HEADER);

	tegra_dc_writel(dc, window->format, DC_WIN_COLOR_DEPTH);
	tegra_dc_writel(dc, window->swap, DC_WIN_BYTE_SWAP);

	value = V_POSITION(window->dst.y) | H_POSITION(window->dst.x);
	tegra_dc_writel(dc, value, DC_WIN_POSITION);

	value = V_SIZE(window->dst.h) | H_SIZE(window->dst.w);
	tegra_dc_writel(dc, value, DC_WIN_SIZE);

	if (window->scan_column) {
		h_size = (window->src.h >> 16) * bpp;
		v_size = (window->src.w >> 16);
	} else {
		h_size = (window->src.w >> 16) * bpp;
		v_size = (window->src.h >> 16);
	}

	value = V_PRESCALED_SIZE(v_size) | H_PRESCALED_SIZE(h_size);
	tegra_dc_writel(dc, value, DC_WIN_PRESCALED_SIZE);

	h_offset = (window->src.x >> 16) * bpp;
	if (window->right_left)
		h_offset += (window->src.w >> 16) * bpp - 1;

	v_offset = (window->src.y >> 16);
	if (window->bottom_up)
		v_offset += (window->src.h >> 16) - 1;

	if (yuv && planar) {
		/*
		 * For YUV planar format, h_offset must be even unless window
		 * is flipped horizontally, in which case it needs to be odd;
		 * v_offset must be even unless window is flipped vertically,
		 * in which case it needs to be an odd.
		 */
		h_offset = (h_offset & ~1) | (window->right_left ? 1 : 0);
		v_offset = (v_offset & ~1) | (window->bottom_up ? 1 : 0);
		/*
		 * For DDA computations the number of bytes per pixel for YUV
		 * planar modes needs to take into account all Y, U and V
		 * components.
		 */
		bpp = 2;
	}


	if (window->scan_column) {
		h_dda = compute_dda_inc(window->src.h, window->dst.w, false, bpp);
		v_dda = compute_dda_inc(window->src.w, window->dst.h, true, bpp);
	} else {
		h_dda = compute_dda_inc(window->src.w, window->dst.w, false, bpp);
		v_dda = compute_dda_inc(window->src.h, window->dst.h, true, bpp);
	}

	value = V_DDA_INC(v_dda) | H_DDA_INC(h_dda);
	tegra_dc_writel(dc, value, DC_WIN_DDA_INC);

	if (window->scan_column) {
		h_dda = compute_initial_dda(window->src.y);
		v_dda = compute_initial_dda(window->src.x);
	} else {
		h_dda = compute_initial_dda(window->src.x);
		v_dda = compute_initial_dda(window->src.y);
	}

	tegra_dc_writel(dc, h_dda, DC_WIN_H_INITIAL_DDA);
	tegra_dc_writel(dc, v_dda, DC_WIN_V_INITIAL_DDA);

	tegra_dc_set_scaling_filter(dc);

	tegra_dc_writel(dc, 0, DC_WIN_UV_BUF_STRIDE);
	tegra_dc_writel(dc, 0, DC_WIN_BUF_STRIDE);

	tegra_dc_writel(dc, window->base[0], DC_WINBUF_START_ADDR);

	if (yuv && planar) {
		tegra_dc_writel(dc, window->base[1], DC_WINBUF_START_ADDR_U);
		tegra_dc_writel(dc, window->base[2], DC_WINBUF_START_ADDR_V);
		value = window->stride[1] << 16 | window->stride[0];
		tegra_dc_writel(dc, value, DC_WIN_LINE_STRIDE);
	} else {
		tegra_dc_writel(dc, window->stride[0], DC_WIN_LINE_STRIDE);
	}

	tegra_dc_writel(dc, h_offset, DC_WINBUF_ADDR_H_OFFSET);
	tegra_dc_writel(dc, v_offset, DC_WINBUF_ADDR_V_OFFSET);

	if (dc->soc->supports_block_linear) {
		unsigned long height = window->tiling.value;

		switch (window->tiling.mode) {
		case TEGRA_BO_TILING_MODE_PITCH:
			value = DC_WINBUF_SURFACE_KIND_PITCH;
			break;

		case TEGRA_BO_TILING_MODE_TILED:
			value = DC_WINBUF_SURFACE_KIND_TILED;
			break;

		case TEGRA_BO_TILING_MODE_BLOCK:
			value = DC_WINBUF_SURFACE_KIND_BLOCK_HEIGHT(height) |
				DC_WINBUF_SURFACE_KIND_BLOCK;
			break;
		}

		tegra_dc_writel(dc, value, DC_WINBUF_SURFACE_KIND);
	} else {
		switch (window->tiling.mode) {
		case TEGRA_BO_TILING_MODE_PITCH:
			value = DC_WIN_BUFFER_ADDR_MODE_LINEAR_UV |
				DC_WIN_BUFFER_ADDR_MODE_LINEAR;
			break;

		case TEGRA_BO_TILING_MODE_TILED:
			value = DC_WIN_BUFFER_ADDR_MODE_TILE_UV |
				DC_WIN_BUFFER_ADDR_MODE_TILE;
			break;

		case TEGRA_BO_TILING_MODE_BLOCK:
			/*
			 * No need to handle this here because ->atomic_check
			 * will already have filtered it out.
			 */
			break;
		}

		tegra_dc_writel(dc, value, DC_WIN_BUFFER_ADDR_MODE);
	}

	value = WIN_ENABLE;

	if (yuv) {
		/* setup default colorspace conversion coefficients */
		tegra_dc_writel(dc, 0x00f0, DC_WIN_CSC_YOF);
		tegra_dc_writel(dc, 0x012a, DC_WIN_CSC_KYRGB);
		tegra_dc_writel(dc, 0x0000, DC_WIN_CSC_KUR);
		tegra_dc_writel(dc, 0x0198, DC_WIN_CSC_KVR);
		tegra_dc_writel(dc, 0x039b, DC_WIN_CSC_KUG);
		tegra_dc_writel(dc, 0x032f, DC_WIN_CSC_KVG);
		tegra_dc_writel(dc, 0x0204, DC_WIN_CSC_KUB);
		tegra_dc_writel(dc, 0x0000, DC_WIN_CSC_KVB);

		value |= CSC_ENABLE;
	} else if (window->bits_per_pixel < 24) {
		value |= COLOR_EXPAND;
	}

	if (win_use_h_filter(dc, window, index))
		value |= H_FILTER_ENABLE;

	if (win_use_v_filter(dc, window, index))
		value |= V_FILTER_ENABLE;

	if (window->scan_column)
		value |= WIN_SCAN_COLUMN;

	if (window->right_left)
		value |= H_DIRECTION;

	if (window->bottom_up)
		value |= V_DIRECTION;

	tegra_dc_writel(dc, value, DC_WIN_WIN_OPTIONS);

	if (dc->soc->supports_v2_blend) {
		switch (window->format) {
		case WIN_COLOR_DEPTH_B5G5R5A:
		case WIN_COLOR_DEPTH_B4G4R4A4:
		case WIN_COLOR_DEPTH_AB5G5R5:
		case WIN_COLOR_DEPTH_B8G8R8A8:
		case WIN_COLOR_DEPTH_R8G8B8A8:
			/* Pre-mult alpha blending */
			tegra_dc_writel(dc, K1_VALUE(window->alpha),
					DC_WIN_BLEND_LAYER_CONTROL);
			tegra_dc_writel(dc, 0x3262, DC_WIN_BLEND_MATCH_SELECT);
			tegra_dc_writel(dc, 0x3222, DC_WIN_BLEND_NOMATCH_SELECT);
			break;
		default:
			/* No blending */
			tegra_dc_writel(dc, 0x1000000, DC_WIN_BLEND_LAYER_CONTROL);
			tegra_dc_writel(dc, 0x0, DC_WIN_BLEND_MATCH_SELECT);
			tegra_dc_writel(dc, 0x0, DC_WIN_BLEND_NOMATCH_SELECT);
			break;
		}
	} else {
		/*
		 * Disable blending and assume Window A is the bottom-most
		 * window, Window C is the top-most window and Window B is in
		 * the middle.
		 */
		tegra_dc_writel(dc, 0xffff00, DC_WIN_BLEND_NOKEY);
		tegra_dc_writel(dc, 0xffff00, DC_WIN_BLEND_1WIN);

		switch (index) {
		case 0:
			tegra_dc_writel(dc, 0x000000, DC_WIN_BLEND_2WIN_X);
			tegra_dc_writel(dc, 0x000000, DC_WIN_BLEND_2WIN_Y);
			tegra_dc_writel(dc, 0x000000, DC_WIN_BLEND_3WIN_XY);
			break;

		case 1:
			tegra_dc_writel(dc, 0xffff00, DC_WIN_BLEND_2WIN_X);
			tegra_dc_writel(dc, 0x000000, DC_WIN_BLEND_2WIN_Y);
			tegra_dc_writel(dc, 0x000000, DC_WIN_BLEND_3WIN_XY);
			break;

		case 2:
			tegra_dc_writel(dc, 0xffff00, DC_WIN_BLEND_2WIN_X);
			tegra_dc_writel(dc, 0xffff00, DC_WIN_BLEND_2WIN_Y);
			tegra_dc_writel(dc, 0xffff00, DC_WIN_BLEND_3WIN_XY);
			break;
		}
	}

	spin_unlock_irqrestore(&dc->lock, flags);
}

static void tegra_plane_destroy(struct drm_plane *plane)
{
	struct tegra_plane *p = to_tegra_plane(plane);

	drm_plane_cleanup(plane);
	kfree(p);
}

static void tegra_primary_plane_destroy(struct drm_plane *plane)
{
	tegra_plane_destroy(plane);
}

static void tegra_plane_reset(struct drm_plane *plane)
{
	struct tegra_plane_state *state;

	if (plane->state)
		__drm_atomic_helper_plane_destroy_state(plane, plane->state);

	kfree(plane->state);
	plane->state = NULL;

	state = kzalloc(sizeof(*state), GFP_KERNEL);
	if (state) {
		state->base.alpha = 0xFF; /* Default plane alpha to opaque */
		plane->state = &state->base;
		plane->state->plane = plane;
	}
}

static struct drm_plane_state *tegra_plane_atomic_duplicate_state(struct drm_plane *plane)
{
	struct tegra_plane_state *state = to_tegra_plane_state(plane->state);
	struct tegra_plane_state *copy;

	copy = kmalloc(sizeof(*copy), GFP_KERNEL);
	if (!copy)
		return NULL;

	__drm_atomic_helper_plane_duplicate_state(plane, &copy->base);
	copy->tiling = state->tiling;
	copy->format = state->format;
	copy->swap = state->swap;
	copy->plane_emc_bw = state->plane_emc_bw;

	return &copy->base;
}

static void tegra_plane_atomic_destroy_state(struct drm_plane *plane,
					     struct drm_plane_state *state)
{
	__drm_atomic_helper_plane_destroy_state(plane, state);
	kfree(state);
}

static const struct drm_plane_funcs tegra_primary_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.destroy = tegra_primary_plane_destroy,
	.reset = tegra_plane_reset,
	.atomic_duplicate_state = tegra_plane_atomic_duplicate_state,
	.atomic_destroy_state = tegra_plane_atomic_destroy_state,
};

static int tegra_plane_prepare_fb(struct drm_plane *plane,
				  struct drm_framebuffer *fb,
				  const struct drm_plane_state *new_state)
{
	return 0;
}

static void tegra_plane_cleanup_fb(struct drm_plane *plane,
				   struct drm_framebuffer *fb,
				   const struct drm_plane_state *old_fb)
{
}

static int tegra_plane_state_add(struct tegra_plane *plane,
				 struct drm_plane_state *state)
{
	struct drm_crtc_state *crtc_state;
	struct tegra_dc_state *tegra;

	/* Propagate errors from allocation or locking failures. */
	crtc_state = drm_atomic_get_crtc_state(state->state, state->crtc);
	if (IS_ERR(crtc_state))
		return PTR_ERR(crtc_state);

	/* If crtc is not active, don't add the planes */
	if (!crtc_state->active)
		return 0;

	tegra = to_dc_state(crtc_state);

	tegra->planes |= WIN_A_ACT_REQ << plane->index;
	return 0;
}

static unsigned int tegra_find_scale_limit(struct drm_plane_state *state,
					   unsigned int num_scale_limits,
				const struct tegra_dc_scale_limit *scale_limit)
{
	struct tegra_plane_state *tps = to_tegra_plane_state(state);
	unsigned int bpp;
	int i;

	if (!num_scale_limits)
		return UINT_MAX;

	if (tegra_dc_format_is_yuv(tps->format, NULL))
		bpp = 2;
	else
		bpp = state->fb->bits_per_pixel / 8;

	for (i = 0; i < num_scale_limits; i++) {
		if (!scale_limit[i].bpp || bpp == scale_limit[i].bpp)
			return scale_limit[i].limit;
	}

	return UINT_MAX;
}

/*
 * In order to map rotations to the 3 parameters needed from the dc hardware
 * (scan_column, right_left, bottom_up), it's necessary to reduce the rotation
 * request as much as possible to one of the following:
 * - Plain ole' 90/180/270 rotate
 * - Horizontal or vertical reflection
 * - Some combination of 90 rotate plus one reflection
 *
 * All other combinations can be reduced to one of the aforementioned
 * transforms. That is what this function does.
 */
static unsigned int tegra_plane_simplify_rotation(unsigned int r)
{
	unsigned int rotations = 0;

	/*
	 * If we have more than one rotation, behavior is unspecified. However,
	 * let's just add up the total rotation.
	 */
	if (r & BIT(DRM_ROTATE_90))
		rotations += 1;
	if (r & BIT(DRM_ROTATE_180))
		rotations += 2;
	if (r & BIT(DRM_ROTATE_270))
		rotations += 3;
	rotations %= 4; /* Since DRM_ROTATE_* is 0,1,2,3 */

	/*
	 * Clear out all previous rotations in favor of our new, singular
	 * rotation.
	 */
	r &= ~(BIT(DRM_ROTATE_0) | BIT(DRM_ROTATE_90) | BIT(DRM_ROTATE_180) |
	       BIT(DRM_ROTATE_270));
	if (rotations)
		r |= BIT(rotations);

	/* Handle double reflections */
	if (r & BIT(DRM_REFLECT_X) && r & BIT(DRM_REFLECT_Y)) {
		if (r & BIT(DRM_ROTATE_90))
			return BIT(DRM_ROTATE_270);
		else if (r & BIT(DRM_ROTATE_180))
			return BIT(DRM_ROTATE_0);
		else if (r & BIT(DRM_ROTATE_270))
			return BIT(DRM_ROTATE_90);
		else
			return BIT(DRM_ROTATE_180);
	}

	/* Any reflection with rotation >90 can be simplified */
	if (r == (BIT(DRM_REFLECT_X) | BIT(DRM_ROTATE_180)))
		return BIT(DRM_REFLECT_Y);
	else if (r == (BIT(DRM_REFLECT_Y) | BIT(DRM_ROTATE_180)))
		return BIT(DRM_REFLECT_X);
	else if (r == (BIT(DRM_REFLECT_X) | BIT(DRM_ROTATE_270)))
		return BIT(DRM_REFLECT_Y) | BIT(DRM_ROTATE_90);
	else if (r == (BIT(DRM_REFLECT_Y) | BIT(DRM_ROTATE_270)))
		return BIT(DRM_REFLECT_X) | BIT(DRM_ROTATE_90);

	return r;
}

static int tegra_plane_atomic_check(struct drm_plane *plane,
				    struct drm_plane_state *state)
{
	struct tegra_plane_state *plane_state = to_tegra_plane_state(state);
	struct tegra_bo_tiling *tiling = &plane_state->tiling;
	struct tegra_plane *tegra = to_tegra_plane(plane);
	struct tegra_dc *dc = to_tegra_dc(state->crtc);
	int err;
	unsigned int limit;
	unsigned int rotation = tegra_plane_simplify_rotation(state->rotation);
	uint32_t crtc_w = state->crtc_w, crtc_h = state->crtc_h;
	bool planar, supports_planar_rotation;

	/* no need for further checks if the plane is being disabled */
	if (!state->crtc)
		return 0;

	if ((state->src_w >> 16) == 0 || (state->src_h >> 16) == 0 ||
	    state->crtc_w == 0 || state->crtc_h == 0)
		return -EINVAL;

	err = tegra_dc_format(state->fb->pixel_format, &plane_state->format,
			      &plane_state->swap);
	if (err < 0)
		return err;

	if (tegra_dc_format_is_yuv(plane_state->format, &planar)) {
		supports_planar_rotation =
			dc->soc->windows[tegra->index].supports_planar_rotation;
		if ((planar && rotation) && !supports_planar_rotation)
			return -EINVAL;
	}

	if (rotation & (BIT(DRM_ROTATE_270) | BIT(DRM_ROTATE_90))) {
		crtc_w = state->crtc_h;
		crtc_h = state->crtc_w;
	}

	limit = tegra_find_scale_limit(state, dc->soc->num_h_scale_down_limits,
				       dc->soc->h_scale_down_limits);
	if ((state->src_w >> 16) / crtc_w > limit)
		return -EINVAL;

	limit = tegra_find_scale_limit(state, dc->soc->num_v_scale_down_limits,
				       dc->soc->v_scale_down_limits);
	if ((state->src_h >> 16) / crtc_h > limit)
		return -EINVAL;

	err = tegra_fb_get_tiling(state->fb, tiling);
	if (err < 0)
		return err;

	if (tiling->mode == TEGRA_BO_TILING_MODE_BLOCK &&
	    !dc->soc->supports_block_linear) {
		DRM_ERROR("hardware doesn't support block linear mode\n");
		return -EINVAL;
	}

	/*
	 * Tegra doesn't support different strides for U and V planes so we
	 * error out if the user tries to display a framebuffer with such a
	 * configuration.
	 */
	if (drm_format_num_planes(state->fb->pixel_format) > 2) {
		if (state->fb->pitches[2] != state->fb->pitches[1]) {
			DRM_ERROR("unsupported UV-plane configuration\n");
			return -EINVAL;
		}
	}

	err = tegra_plane_state_add(tegra, state);
	if (err < 0) {
		return err;
	}

	return 0;
}

static void tegra_plane_atomic_update(struct drm_plane *plane,
				      struct drm_plane_state *old_state)
{
	struct tegra_plane_state *state = to_tegra_plane_state(plane->state);
	struct drm_crtc *crtc = plane->state->crtc;
	struct tegra_dc *dc = to_tegra_dc(crtc);
	struct drm_framebuffer *fb = plane->state->fb;
	struct tegra_plane *p = to_tegra_plane(plane);
	struct tegra_dc_window window;
	unsigned int i, rotation;

	/* rien ne va plus */
	if (!crtc || !fb || !crtc->state->active)
		return;

	memset(&window, 0, sizeof(window));
	window.src.x = plane->state->src_x;
	window.src.y = plane->state->src_y;
	window.src.w = plane->state->src_w;
	window.src.h = plane->state->src_h;
	window.dst.x = plane->state->crtc_x;
	window.dst.y = plane->state->crtc_y;
	window.dst.w = plane->state->crtc_w;
	window.dst.h = plane->state->crtc_h;
	window.bits_per_pixel = fb->bits_per_pixel;
	window.alpha = plane->state->alpha;

	/*
	 * Based on the simplified rotation, calculate the scanning parameters
	 * according to the following table (taken from the TRM):
	 * +---------------------------------------------------------+
	 * | REFLECT | ROTATE | SCAN_COLUMN | RIGHT_LEFT | BOTTOM_UP |
	 * +---------------------------------------------------------+
	 * |    -    |    0   |     0       |     0      |     0     |
	 * |    Y    |    0   |     0       |     0      |     1     |
	 * |    X    |    0   |     0       |     1      |     0     |
	 * |    -    |   180  |     0       |     1      |     1     |
	 * |    Y    |   90   |     1       |     0      |     0     |
	 * |    -    |   90   |     1       |     0      |     1     |
	 * |    -    |   270  |     1       |     1      |     0     |
	 * |    X    |   90   |     1       |     1      |     1     |
	 * +---------------------------------------------------------+
	 */
	rotation = tegra_plane_simplify_rotation(plane->state->rotation);
	if (rotation == BIT(DRM_ROTATE_180)) {
		window.scan_column = false;
		window.right_left = true;
		window.bottom_up = true;
	} else if (rotation == BIT(DRM_ROTATE_270)) {
		window.scan_column = dc->soc->supports_scan_column;
		window.right_left = true;
		window.bottom_up = false;
	} else {
		window.scan_column = dc->soc->supports_scan_column &&
				     (rotation & BIT(DRM_ROTATE_90));
		window.right_left = !!(rotation & BIT(DRM_REFLECT_X));
		window.bottom_up = rotation == BIT(DRM_REFLECT_Y) ||
				   rotation == BIT(DRM_ROTATE_90) ||
				   rotation == (BIT(DRM_REFLECT_X) |
						BIT(DRM_ROTATE_90));
	}
	if (tegra_fb_is_bottom_up(fb))
		window.bottom_up = !window.bottom_up;

	/* copy from state */
	window.tiling = state->tiling;
	window.format = state->format;
	window.swap = state->swap;

	for (i = 0; i < drm_format_num_planes(fb->pixel_format); i++) {
		struct tegra_bo *bo = tegra_fb_get_plane(fb, i);

		window.base[i] = bo->paddr + fb->offsets[i];
		if (i < 2)
			window.stride[i] = fb->pitches[i];
		
	}

	/* for semiplanar modes both U and V planes should point to the same
	 * base address.
	 */
	if (drm_format_num_planes(fb->pixel_format) == 2 &&
	    tegra_dc_format_is_yuv(window.format, NULL)) {
	    window.base[2] = window.base[1];
	}

	tegra_dc_setup_window(dc, p->index, &window);
}

static void tegra_plane_atomic_disable(struct drm_plane *plane,
				       struct drm_plane_state *old_state)
{
	struct tegra_plane *p = to_tegra_plane(plane);
	struct tegra_dc *dc;
	unsigned long flags;
	u32 value;

	/* rien ne va plus */
	if (!old_state || !old_state->crtc || !old_state->crtc->state->active)
		return;

	dc = to_tegra_dc(old_state->crtc);

	spin_lock_irqsave(&dc->lock, flags);

	value = WINDOW_A_SELECT << p->index;
	tegra_dc_writel(dc, value, DC_CMD_DISPLAY_WINDOW_HEADER);

	value = tegra_dc_readl(dc, DC_WIN_WIN_OPTIONS);
	value &= ~WIN_ENABLE;
	tegra_dc_writel(dc, value, DC_WIN_WIN_OPTIONS);

	spin_unlock_irqrestore(&dc->lock, flags);
}

static const struct drm_plane_helper_funcs tegra_primary_plane_helper_funcs = {
	.prepare_fb = tegra_plane_prepare_fb,
	.cleanup_fb = tegra_plane_cleanup_fb,
	.atomic_check = tegra_plane_atomic_check,
	.atomic_update = tegra_plane_atomic_update,
	.atomic_disable = tegra_plane_atomic_disable,
};

static void tegra_plane_add_optional_properties(struct drm_device *drm,
		struct tegra_dc *dc, struct tegra_plane *plane)
{
	unsigned int supported_rotations;

	if (!drm->mode_config.rotation_property) {
		supported_rotations = BIT(DRM_ROTATE_0) | BIT(DRM_ROTATE_180) |
			BIT(DRM_REFLECT_X) | BIT(DRM_REFLECT_Y);
		if (dc->soc->supports_scan_column)
			supported_rotations |= BIT(DRM_ROTATE_90) |
				BIT(DRM_ROTATE_270);

		drm->mode_config.rotation_property =
			drm_mode_create_rotation_property(drm,
					supported_rotations);
		if (drm->mode_config.rotation_property == NULL)
			dev_warn(drm->dev,
					"failed to create rotation property\n");
	}
	if (drm->mode_config.rotation_property)
		drm_object_attach_property(&plane->base.base,
				drm->mode_config.rotation_property, 0);

	if (!dc->soc->supports_v2_blend)
		return;

	if (!drm->mode_config.alpha_property) {
		drm->mode_config.alpha_property =
			drm_mode_create_alpha_property(drm);
		if (!drm->mode_config.alpha_property)
			dev_warn(drm->dev, "Failed to create alpha property\n");
	}
	if (drm->mode_config.alpha_property)
		drm_object_attach_property(&plane->base.base,
				drm->mode_config.alpha_property, 0xFF);
}

static struct drm_plane *tegra_dc_primary_plane_create(struct drm_device *drm,
						       struct tegra_dc *dc)
{
	/*
	 * Ideally this would use drm_crtc_mask(), but that would require the
	 * CRTC to already be in the mode_config's list of CRTCs. However, it
	 * will only be added to that list in the drm_crtc_init_with_planes()
	 * (in tegra_dc_init()), which in turn requires registration of these
	 * planes. So we have ourselves a nice little chicken and egg problem
	 * here.
	 *
	 * We work around this by manually creating the mask from the number
	 * of CRTCs that have been registered, and should therefore always be
	 * the same as drm_crtc_index() after registration.
	 */
	unsigned long possible_crtcs = 1 << drm->mode_config.num_crtc;
	struct tegra_plane *plane;
	int err;

	plane = kzalloc(sizeof(*plane), GFP_KERNEL);
	if (!plane)
		return ERR_PTR(-ENOMEM);

	err = drm_universal_plane_init(drm, &plane->base, possible_crtcs,
				       &tegra_primary_plane_funcs,
				       dc->soc->primary_plane_formats,
				       dc->soc->num_primary_plane_formats,
				       DRM_PLANE_TYPE_PRIMARY,
				       NULL);
	if (err < 0) {
		kfree(plane);
		return ERR_PTR(err);
	}

	drm_plane_helper_add(&plane->base, &tegra_primary_plane_helper_funcs);

	tegra_plane_add_optional_properties(drm, dc, plane);

	return &plane->base;
}

static const u32 tegra_cursor_plane_formats[] = {
	DRM_FORMAT_RGBA8888,
};

static int tegra_cursor_atomic_check(struct drm_plane *plane,
				     struct drm_plane_state *state)
{
	struct tegra_plane *tegra = to_tegra_plane(plane);
	int err;

	/* no need for further checks if the plane is being disabled */
	if (!state->crtc)
		return 0;

	/* scaling not supported for cursor */
	if ((state->src_w >> 16 != state->crtc_w) ||
	    (state->src_h >> 16 != state->crtc_h))
		return -EINVAL;

	/* only square cursors supported */
	if (state->src_w != state->src_h)
		return -EINVAL;

	if (state->crtc_w != 32 && state->crtc_w != 64 &&
	    state->crtc_w != 128 && state->crtc_w != 256)
		return -EINVAL;

	err = tegra_plane_state_add(tegra, state);
	if (err < 0)
		return err;

	return 0;
}

static void tegra_cursor_atomic_update(struct drm_plane *plane,
				       struct drm_plane_state *old_state)
{
	struct tegra_bo *bo;
	struct tegra_dc *dc;
	struct drm_plane_state *state = plane->state;
	struct drm_crtc *crtc = plane->state->crtc;
	u32 value = CURSOR_CLIP_DISPLAY;

	if (!plane->state->fb || !crtc || !crtc->state->active)
		return;

	bo = tegra_fb_get_plane(plane->state->fb, 0);
	dc = to_tegra_dc(plane->state->crtc);

	switch (state->crtc_w) {
	case 32:
		value |= CURSOR_SIZE_32x32;
		break;

	case 64:
		value |= CURSOR_SIZE_64x64;
		break;

	case 128:
		value |= CURSOR_SIZE_128x128;
		break;

	case 256:
		value |= CURSOR_SIZE_256x256;
		break;

	default:
		WARN(1, "cursor size %ux%u not supported\n", state->crtc_w,
		     state->crtc_h);
		return;
	}

	value |= (bo->paddr >> 10) & 0x3fffff;
	tegra_dc_writel(dc, value, DC_DISP_CURSOR_START_ADDR);

#ifdef CONFIG_ARCH_DMA_ADDR_T_64BIT
	value = (bo->paddr >> 32) & 0x3;
	tegra_dc_writel(dc, value, DC_DISP_CURSOR_START_ADDR_HI);
#endif

	/* enable cursor and set blend mode */
	value = tegra_dc_readl(dc, DC_DISP_DISP_WIN_OPTIONS);
	value |= CURSOR_ENABLE;
	tegra_dc_writel(dc, value, DC_DISP_DISP_WIN_OPTIONS);

	value = tegra_dc_readl(dc, DC_DISP_BLEND_CURSOR_CONTROL);
	value &= ~CURSOR_DST_BLEND_MASK;
	value &= ~CURSOR_SRC_BLEND_MASK;
	value |= CURSOR_MODE_NORMAL;
	value |= CURSOR_DST_BLEND_NEG_K1_TIMES_SRC;
	value |= CURSOR_SRC_BLEND_K1_TIMES_SRC;
	value |= CURSOR_ALPHA;
	tegra_dc_writel(dc, value, DC_DISP_BLEND_CURSOR_CONTROL);

	/* position the cursor */
	value = (state->crtc_y & 0x3fff) << 16 | (state->crtc_x & 0x3fff);
	tegra_dc_writel(dc, value, DC_DISP_CURSOR_POSITION);

}

static void tegra_cursor_atomic_disable(struct drm_plane *plane,
					struct drm_plane_state *old_state)
{
	struct tegra_dc *dc;
	u32 value;

	/* rien ne va plus */
	if (!old_state || !old_state->crtc || !old_state->crtc->state->active)
		return;

	dc = to_tegra_dc(old_state->crtc);

	value = tegra_dc_readl(dc, DC_DISP_DISP_WIN_OPTIONS);
	value &= ~CURSOR_ENABLE;
	tegra_dc_writel(dc, value, DC_DISP_DISP_WIN_OPTIONS);
}

static const struct drm_plane_funcs tegra_cursor_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.destroy = tegra_plane_destroy,
	.reset = tegra_plane_reset,
	.atomic_duplicate_state = tegra_plane_atomic_duplicate_state,
	.atomic_destroy_state = tegra_plane_atomic_destroy_state,
};

static const struct drm_plane_helper_funcs tegra_cursor_plane_helper_funcs = {
	.prepare_fb = tegra_plane_prepare_fb,
	.cleanup_fb = tegra_plane_cleanup_fb,
	.atomic_check = tegra_cursor_atomic_check,
	.atomic_update = tegra_cursor_atomic_update,
	.atomic_disable = tegra_cursor_atomic_disable,
};

static struct drm_plane *tegra_dc_cursor_plane_create(struct drm_device *drm,
						      struct tegra_dc *dc)
{
	struct tegra_plane *plane;
	unsigned int num_formats;
	const u32 *formats;
	int err;

	plane = kzalloc(sizeof(*plane), GFP_KERNEL);
	if (!plane)
		return ERR_PTR(-ENOMEM);

	/*
	 * We'll treat the cursor as an overlay plane with index 6 here so
	 * that the update and activation request bits in DC_CMD_STATE_CONTROL
	 * match up.
	 */
	plane->index = 6;

	num_formats = ARRAY_SIZE(tegra_cursor_plane_formats);
	formats = tegra_cursor_plane_formats;

	err = drm_universal_plane_init(drm, &plane->base, 1 << dc->pipe,
				       &tegra_cursor_plane_funcs, formats,
				       num_formats, DRM_PLANE_TYPE_CURSOR,
				       NULL);
	if (err < 0) {
		kfree(plane);
		return ERR_PTR(err);
	}

	drm_plane_helper_add(&plane->base, &tegra_cursor_plane_helper_funcs);

	return &plane->base;
}

static void tegra_overlay_plane_destroy(struct drm_plane *plane)
{
	tegra_plane_destroy(plane);
}

static const struct drm_plane_funcs tegra_overlay_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.destroy = tegra_overlay_plane_destroy,
	.reset = tegra_plane_reset,
	.atomic_duplicate_state = tegra_plane_atomic_duplicate_state,
	.atomic_destroy_state = tegra_plane_atomic_destroy_state,
};

static const struct drm_plane_helper_funcs tegra_overlay_plane_helper_funcs = {
	.prepare_fb = tegra_plane_prepare_fb,
	.cleanup_fb = tegra_plane_cleanup_fb,
	.atomic_check = tegra_plane_atomic_check,
	.atomic_update = tegra_plane_atomic_update,
	.atomic_disable = tegra_plane_atomic_disable,
};

static struct drm_plane *tegra_dc_overlay_plane_create(struct drm_device *drm,
						       struct tegra_dc *dc,
						       unsigned int index)
{
	struct tegra_plane *plane;
	int err;

	plane = kzalloc(sizeof(*plane), GFP_KERNEL);
	if (!plane)
		return ERR_PTR(-ENOMEM);

	plane->index = index;

	err = drm_universal_plane_init(drm, &plane->base, 1 << dc->pipe,
				       &tegra_overlay_plane_funcs,
				       dc->soc->overlay_plane_formats,
				       dc->soc->num_overlay_plane_formats,
				       DRM_PLANE_TYPE_OVERLAY,
				       NULL);
	if (err < 0) {
		kfree(plane);
		return ERR_PTR(err);
	}

	drm_plane_helper_add(&plane->base, &tegra_overlay_plane_helper_funcs);

	tegra_plane_add_optional_properties(drm, dc, plane);

	return &plane->base;
}

static int tegra_dc_add_planes(struct drm_device *drm, struct tegra_dc *dc)
{
	struct drm_plane *plane;
	unsigned int i;

	for (i = 0; i < dc->soc->num_windows - 1; i++) {
		plane = tegra_dc_overlay_plane_create(drm, dc, 1 + i);
		if (IS_ERR(plane))
			return PTR_ERR(plane);
	}

	return 0;
}

u32 tegra_dc_get_vblank_counter(struct tegra_dc *dc)
{
	if (dc->syncpt)
		return host1x_syncpt_read(dc->syncpt);

	/* fallback to software emulated VBLANK counter */
	return drm_crtc_vblank_count(&dc->base);
}

static void tegra_dc_dpms(struct tegra_dc *dc, int mode)
{
	u32 value, enable_mask;

	WARN_ON(!spin_is_locked(&dc->lock));

	enable_mask = PW0_ENABLE | PW1_ENABLE | PW2_ENABLE | PW3_ENABLE |
			PW4_ENABLE | PM0_ENABLE | PM1_ENABLE;

	if (dc->dpms == mode)
		return;

	value = tegra_dc_readl(dc, DC_CMD_DISPLAY_POWER_CONTROL);
	switch (mode) {
	case DRM_MODE_DPMS_OFF:
	case DRM_MODE_DPMS_STANDBY:
		value &= ~enable_mask;
		break;
	case DRM_MODE_DPMS_ON:
		value |= enable_mask;
		break;
	case DRM_MODE_DPMS_SUSPEND:
	default:
		WARN(1, "DPMS mode %d unsupported!\n", mode);
		return;
	}
	tegra_dc_writel(dc, value, DC_CMD_DISPLAY_POWER_CONTROL);
	dc->dpms = mode;
}

static void tegra_dc_enable_vblank_impl(struct tegra_dc *dc)
{
	unsigned long value;
	struct tegra_dc_state *state = to_dc_state(dc->base.state);

	WARN_ON(!spin_is_locked(&dc->lock));
#if TEGRA_DC_ENABLE_STANDBY
	if (state->nc_mode && dc->dpms == DRM_MODE_DPMS_STANDBY)
		tegra_dc_dpms(dc, DRM_MODE_DPMS_ON);
#endif

	value = tegra_dc_readl(dc, DC_CMD_INT_MASK);
	if (state->nc_mode) {
		value |= MSF_INT;
		value &= ~VBLANK_INT;
	} else {
		value |= VBLANK_INT;
		value &= ~MSF_INT;
	}
	tegra_dc_writel(dc, value, DC_CMD_INT_MASK);
}

void tegra_dc_enable_vblank(struct tegra_dc *dc)
{
	unsigned long flags;

	spin_lock_irqsave(&dc->lock, flags);

	dc->vblank_enabled = true;
	if (dc->soc->has_powergate && dc->dpms != DRM_MODE_DPMS_OFF)
		tegra_dc_enable_vblank_impl(dc);

	spin_unlock_irqrestore(&dc->lock, flags);
}

void tegra_dc_disable_vblank(struct tegra_dc *dc)
{
	unsigned long value, flags;
	struct tegra_dc_state *state = to_dc_state(dc->base.state);

	spin_lock_irqsave(&dc->lock, flags);

	dc->vblank_enabled = false;
	value = tegra_dc_readl(dc, DC_CMD_INT_MASK);
	if (state->nc_mode)
		value &= ~MSF_INT;
	else
		value &= ~VBLANK_INT;
	tegra_dc_writel(dc, value, DC_CMD_INT_MASK);

#if TEGRA_DC_ENABLE_STANDBY
	if (state->nc_mode && dc->dpms == DRM_MODE_DPMS_ON)
		tegra_dc_dpms(dc, DRM_MODE_DPMS_STANDBY);
#endif

	spin_unlock_irqrestore(&dc->lock, flags);
}

static void tegra_dc_finish_page_flip(struct tegra_dc *dc)
{
	struct drm_device *drm = dc->base.dev;
	struct drm_crtc *crtc = &dc->base;
	struct drm_framebuffer *fb = crtc->primary->state->fb;
	unsigned long flags, base;
	struct tegra_bo *bo;

	spin_lock_irqsave(&drm->event_lock, flags);

	if (!fb || !crtc->state->event)
		goto out;

	bo = tegra_fb_get_plane(fb, 0);

	spin_lock(&dc->lock);

	/* check if new start address has been latched */
	tegra_dc_writel(dc, WINDOW_A_SELECT, DC_CMD_DISPLAY_WINDOW_HEADER);
	tegra_dc_writel(dc, READ_MUX, DC_CMD_STATE_ACCESS);
	base = tegra_dc_readl(dc, DC_WINBUF_START_ADDR);
	tegra_dc_writel(dc, 0, DC_CMD_STATE_ACCESS);

	spin_unlock(&dc->lock);

	if (base == bo->paddr + fb->offsets[0]) {
		if (dc->event) {
			drm_crtc_send_vblank_event(crtc, dc->event);
			dc->event = NULL;
		}
		crtc->state->event = NULL;
	}

out:
	spin_unlock_irqrestore(&drm->event_lock, flags);
}

void tegra_dc_cancel_page_flip(struct drm_crtc *crtc, struct drm_file *file)
{
	struct tegra_dc *dc = to_tegra_dc(crtc);
	struct drm_device *drm = crtc->dev;
	unsigned long flags;

	spin_lock_irqsave(&drm->event_lock, flags);

	if (dc->event && dc->event->base.file_priv == file) {
		dc->event->base.destroy(&dc->event->base);
		dc->event = NULL;
	}
	crtc->state->event = NULL;

	spin_unlock_irqrestore(&drm->event_lock, flags);
}

static void tegra_dc_destroy(struct drm_crtc *crtc)
{
	drm_crtc_cleanup(crtc);
}

static void tegra_crtc_reset(struct drm_crtc *crtc)
{
	struct tegra_dc_state *state;

	if (crtc->state)
		__drm_atomic_helper_crtc_destroy_state(crtc, crtc->state);

	kfree(crtc->state);
	crtc->state = NULL;

	state = kzalloc(sizeof(*state), GFP_KERNEL);
	if (state) {
		crtc->state = &state->base;
		crtc->state->crtc = crtc;
	}
}

static struct drm_crtc_state *
tegra_crtc_atomic_duplicate_state(struct drm_crtc *crtc)
{
	struct tegra_dc_state *state = to_dc_state(crtc->state);
	struct tegra_dc_state *copy;

	copy = kmalloc(sizeof(*copy), GFP_KERNEL);
	if (!copy)
		return NULL;

	__drm_atomic_helper_crtc_duplicate_state(crtc, &copy->base);
	copy->clk = state->clk;
	copy->pclk = state->pclk;
	copy->div = state->div;
	copy->planes = state->planes;
	copy->nc_mode = state->nc_mode;
	copy->tearing_effect = state->tearing_effect;
	copy->emc_bandwidth = state->emc_bandwidth;
	copy->update_emc = true;
	copy->update_la_ptsa = true;

	return &copy->base;
}

static void tegra_crtc_atomic_destroy_state(struct drm_crtc *crtc,
					    struct drm_crtc_state *state)
{
	__drm_atomic_helper_crtc_destroy_state(crtc, state);
	kfree(state);
}

static const struct drm_crtc_funcs tegra_crtc_funcs = {
	.page_flip = drm_atomic_helper_page_flip,
	.set_config = drm_atomic_helper_set_config,
	.destroy = tegra_dc_destroy,
	.reset = tegra_crtc_reset,
	.atomic_duplicate_state = tegra_crtc_atomic_duplicate_state,
	.atomic_destroy_state = tegra_crtc_atomic_destroy_state,
};

static void tegra_dc_stop(struct tegra_dc *dc)
{
	u32 value;

	/* stop the display controller */
	value = tegra_dc_readl(dc, DC_CMD_DISPLAY_COMMAND);
	value &= ~DISP_CTRL_MODE_MASK;
	tegra_dc_writel(dc, value, DC_CMD_DISPLAY_COMMAND);

	tegra_dc_commit(dc);
}

static bool tegra_dc_idle(struct tegra_dc *dc)
{
	u32 value;

	value = tegra_dc_readl_active(dc, DC_CMD_DISPLAY_COMMAND);

	return (value & DISP_CTRL_MODE_MASK) == 0;
}

static int tegra_dc_wait_idle(struct tegra_dc *dc, unsigned long timeout)
{
	return wait_for(tegra_dc_idle(dc), timeout);
}

static void tegra_crtc_disable(struct drm_crtc *crtc)
{
	struct tegra_dc *dc = to_tegra_dc(crtc);
	unsigned long flags;

	if (!tegra_dc_idle(dc)) {
		tegra_dc_stop(dc);

		/*
		 * Ignore the return value, there isn't anything useful to do
		 * in case this fails.
		 */
		tegra_dc_wait_idle(dc, 100);
	}

	spin_lock_irqsave(&dc->lock, flags);
	tegra_dc_dpms(dc, DRM_MODE_DPMS_OFF);
	spin_unlock_irqrestore(&dc->lock, flags);

	drm_crtc_vblank_off(crtc);

	tegra_dc_clk_disable(dc);

	if (dc->soc->has_powergate)
		tegra_pmc_powergate(dc->powergate);
}

static bool tegra_crtc_mode_fixup(struct drm_crtc *crtc,
				  const struct drm_display_mode *mode,
				  struct drm_display_mode *adjusted)
{
	return true;
}

static int tegra_dc_set_timings(struct tegra_dc *dc,
				struct drm_display_mode *mode)
{
	unsigned int h_ref_to_sync = 1;
	unsigned int v_ref_to_sync = 1;
	unsigned long value;

	tegra_dc_writel(dc, 0x0, DC_DISP_DISP_TIMING_OPTIONS);

	value = (v_ref_to_sync << 16) | h_ref_to_sync;
	tegra_dc_writel(dc, value, DC_DISP_REF_TO_SYNC);

	value = ((mode->vsync_end - mode->vsync_start) << 16) |
		((mode->hsync_end - mode->hsync_start) <<  0);
	tegra_dc_writel(dc, value, DC_DISP_SYNC_WIDTH);

	value = ((mode->vtotal - mode->vsync_end) << 16) |
		((mode->htotal - mode->hsync_end) <<  0);
	tegra_dc_writel(dc, value, DC_DISP_BACK_PORCH);

	value = ((mode->vsync_start - mode->vdisplay) << 16) |
		((mode->hsync_start - mode->hdisplay) <<  0);
	tegra_dc_writel(dc, value, DC_DISP_FRONT_PORCH);

	value = (mode->vdisplay << 16) | mode->hdisplay;
	tegra_dc_writel(dc, value, DC_DISP_ACTIVE);

	return 0;
}

/**
 * tegra_dc_state_setup_clock - check clock settings and store them in atomic
 *     state
 * @dc: display controller
 * @crtc_state: CRTC atomic state
 * @clk: parent clock for display controller
 * @pclk: pixel clock
 * @div: shift clock divider
 *
 * Returns:
 * 0 on success or a negative error-code on failure.
 */
int tegra_dc_state_setup_clock(struct tegra_dc *dc,
			       struct drm_crtc_state *crtc_state,
			       struct clk *clk, unsigned long pclk,
			       unsigned int div)
{
	struct tegra_dc_state *state = to_dc_state(crtc_state);

	if (!clk_has_parent(dc->clk, clk))
		return -EINVAL;

	state->clk = clk;
	state->pclk = pclk;
	state->div = div;

	return 0;
}

static void tegra_dc_commit_state(struct tegra_dc *dc,
				  struct tegra_dc_state *state)
{
	struct drm_crtc_state *crtc_state;
	u32 value;
	int err;

	err = clk_set_parent(dc->clk, state->clk);
	if (err < 0)
		dev_err(dc->dev, "failed to set parent clock: %d\n", err);

	/*
	 * Outputs may not want to change the parent clock rate. This is only
	 * relevant to Tegra20 where only a single display PLL is available.
	 * Since that PLL would typically be used for HDMI, an internal LVDS
	 * panel would need to be driven by some other clock such as PLL_P
	 * which is shared with other peripherals. Changing the clock rate
	 * should therefore be avoided.
	 */
	if (state->pclk > 0) {
		err = clk_set_rate(state->clk, state->pclk);
		if (err < 0)
			dev_err(dc->dev,
				"failed to set clock rate to %lu Hz\n",
				state->pclk);
	}

	DRM_DEBUG_KMS("rate: %lu, div: %u\n", clk_get_rate(dc->clk),
		      state->div);
	DRM_DEBUG_KMS("pclk: %lu\n", state->pclk);

	value = SHIFT_CLK_DIVIDER(state->div) | PIXEL_CLK_DIVIDER_PCD1;
	tegra_dc_writel(dc, value, DC_DISP_DISP_CLOCK_CONTROL);

	crtc_state = &state->base;
	if (value != 0) {
		/*
		 * Notify DVFS the pixel clock rate since dc
		 * has internal clock divider.
		 */
		tegra_dvfs_set_rate(dc->clk, crtc_state->adjusted_mode.clock * 1000);
	}
}

static void tegra_dc_init_hw(struct tegra_dc *dc)
{
	u32 value;
	struct tegra_dc_state *state = to_dc_state(dc->base.state);

	/* initialize display controller */
	if (dc->syncpt) {
		u32 syncpt = host1x_syncpt_id(dc->syncpt);

		value = SYNCPT_CNTRL_NO_STALL;
		tegra_dc_writel(dc, value, DC_CMD_GENERAL_INCR_SYNCPT_CNTRL);

		value = SYNCPT_VSYNC_ENABLE | syncpt;
		tegra_dc_writel(dc, value, DC_CMD_CONT_SYNCPT_VSYNC);
	}

	value = WIN_A_UF_INT | WIN_B_UF_INT | WIN_C_UF_INT |
		WIN_A_OF_INT | WIN_B_OF_INT | WIN_C_OF_INT;
	tegra_dc_writel(dc, value, DC_CMD_INT_TYPE);

	value = WIN_A_UF_INT | WIN_B_UF_INT | WIN_C_UF_INT |
		WIN_A_OF_INT | WIN_B_OF_INT | WIN_C_OF_INT;
	tegra_dc_writel(dc, value, DC_CMD_INT_POLARITY);

	/* initialize timer */
	value = CURSOR_THRESHOLD(0) | WINDOW_A_THRESHOLD(0x20) |
		WINDOW_B_THRESHOLD(0x20) | WINDOW_C_THRESHOLD(0x20);
	tegra_dc_writel(dc, value, DC_DISP_DISP_MEM_HIGH_PRIORITY);

	value = CURSOR_THRESHOLD(0) | WINDOW_A_THRESHOLD(1) |
		WINDOW_B_THRESHOLD(1) | WINDOW_C_THRESHOLD(1);
	tegra_dc_writel(dc, value, DC_DISP_DISP_MEM_HIGH_PRIORITY_TIMER);

	value = WIN_A_UF_INT | WIN_B_UF_INT | WIN_C_UF_INT |
		WIN_A_OF_INT | WIN_B_OF_INT | WIN_C_OF_INT;
	if (state->nc_mode)
		value |= MSF_INT;
	else
		value |= VBLANK_INT;
	tegra_dc_writel(dc, value, DC_CMD_INT_ENABLE);

	value = tegra_dc_readl(dc, DC_CMD_INT_MASK);
	value |= WIN_A_UF_INT | WIN_B_UF_INT | WIN_C_UF_INT |
		 WIN_A_OF_INT | WIN_B_OF_INT | WIN_C_OF_INT;
	tegra_dc_writel(dc, value, DC_CMD_INT_MASK);

	if (dc->soc->supports_border_color)
		tegra_dc_writel(dc, 0, DC_DISP_BORDER_COLOR);

	tegra_dc_stats_reset(&dc->stats);
}

static void tegra_crtc_get_display_info(struct drm_crtc *crtc)
{
	struct tegra_dc_state *state = to_dc_state(crtc->state);
	struct drm_mode_config *config = &crtc->dev->mode_config;
	struct drm_connector *connector;

	list_for_each_entry(connector, &config->connector_list, head) {
		if (connector->state->crtc != crtc)
			continue;

		state->nc_mode = connector->display_info.supports_psr;
		state->tearing_effect = connector->display_info.tearing_effect;

		/* We can only support one psr panel config, take the first */
		break;
	}
}

static void tegra_crtc_mode_set_nofb(struct drm_crtc *crtc)
{
	struct drm_display_mode *mode = &crtc->state->adjusted_mode;
	struct tegra_dc_state *state = to_dc_state(crtc->state);
	struct tegra_dc *dc = to_tegra_dc(crtc);
	unsigned long flags;
	u32 value;

	spin_lock_irqsave(&dc->lock, flags);
	dc->in_modeset = true;
	spin_unlock_irqrestore(&dc->lock, flags);

	tegra_crtc_get_display_info(crtc);

	if (dc->dpms == DRM_MODE_DPMS_OFF) {
		if (dc->soc->has_powergate)
			tegra_pmc_unpowergate(dc->powergate);
		tegra_dc_clk_enable(dc);
	}

	/*
	 * If we're already initialized (ie: DPMS_ON || DPMS_STANDBY), don't re-
	 * initialize the hardware. This will disable our vblank interrupt,
	 * making hw out-of-sync with sw.
	 */
	spin_lock_irqsave(&dc->lock, flags);
	if (dc->dpms == DRM_MODE_DPMS_OFF)
		tegra_dc_init_hw(dc);

	tegra_dc_dpms(dc, DRM_MODE_DPMS_ON);

	/*
	 * Note that in addition to enabling vblank that may have been deferred
	 * while we were powergated, this will also switch between using the TE
	 * or VBLANK interrupt depending on the new nc_mode in state.
	 *
	 * For example, if we were previously using VBLANK_INT as vblank
	 * interrupt source and the new mode supports psr, we'll switch to using
	 * MSF_INT here.
	 */
	if (dc->vblank_enabled)
		tegra_dc_enable_vblank_impl(dc);

	spin_unlock_irqrestore(&dc->lock, flags);

	tegra_dc_commit_state(dc, state);

	/* program display mode */
	tegra_dc_set_timings(dc, mode);

	/* interlacing isn't supported yet, so disable it */
	if (dc->soc->supports_interlacing) {
		value = tegra_dc_readl(dc, DC_DISP_INTERLACE_CONTROL);
		value &= ~INTERLACE_ENABLE;
		tegra_dc_writel(dc, value, DC_DISP_INTERLACE_CONTROL);
	}

	if (state->nc_mode) {
		/* TODO: Get MSF source from dt */
		value = MSF_ENABLE;
		if (state->tearing_effect & DRM_TEARING_EFFECT_ACTIVE_LOW)
			value |= MSF_POLARITY_LOW;
		tegra_dc_writel(dc, value, DC_CMD_DISPLAY_COMMAND_OPTION0);
	}

	value = tegra_dc_readl(dc, DC_CMD_DISPLAY_COMMAND);
	value &= ~DISP_CTRL_MODE_MASK;
	if (state->nc_mode)
		value |= DISP_CTRL_MODE_NC_DISPLAY;
	else
		value |= DISP_CTRL_MODE_C_DISPLAY;
	tegra_dc_writel(dc, value, DC_CMD_DISPLAY_COMMAND);

	tegra_dc_commit(dc);

	drm_crtc_vblank_on(crtc);
}

static void tegra_crtc_prepare(struct drm_crtc *crtc)
{
	tegra_crtc_disable(crtc);
}

static void tegra_crtc_commit(struct drm_crtc *crtc)
{
}

static int tegra_crtc_atomic_check(struct drm_crtc *crtc,
				   struct drm_crtc_state *state)
{
	/* If we're transitioning from Off to On, force a full modeset */
	if (state->active && !crtc->state->active)
		state->mode_changed = true;

	return 0;
}

static void tegra_crtc_atomic_begin(struct drm_crtc *crtc,
				    struct drm_crtc_state *old_crtc_state)
{
	struct tegra_dc *dc = to_tegra_dc(crtc);
	struct drm_device *drm = dc->base.dev;
	unsigned long flags;

	spin_lock_irqsave(&drm->event_lock, flags);
	if (crtc->state->event) {
		crtc->state->event->pipe = drm_crtc_index(crtc);
		WARN_ON(dc->event);
		dc->event = crtc->state->event;
	/*
	 * If we are active, enabled, and changing planes, set a fake event to
	 * force a latch wait in tegra_atomic_wait_for_flip_complete
	 */
	} else if (crtc->state->enable && crtc->state->active &&
			crtc->state->planes_changed) {
		crtc->state->event = ERR_PTR(-EINVAL);
	}
	spin_unlock_irqrestore(&drm->event_lock, flags);
}

/*
 * Note about fixed point arithmetic:
 * ----------------------------------
 * calc_disp_params(...) contains fixed point values and arithmetic due to the
 * need to use floating point values. All fixed point values have the "_fp" or
 * "_FP" suffix in their name. Functions/values used to convert between real and
 * fixed point values are listed below:
 *    - FP_FACTOR
 *    - LA_REAL_TO_FP(real_val)
 *    - LA_FP_TO_REAL(fp_val)
 */
static void calc_disp_params(struct drm_crtc *crtc,
				struct tegra_plane *tp,
				struct tegra_plane_state *tps,
				int num_wins,
				int mc_client_id,
				unsigned int emc_freq_hz,
				unsigned int total_active_space_bw,
				unsigned long bw_kbps,
				struct tegra_dc_to_la_params *disp_params)
{
	struct tegra_dc *dc = to_tegra_dc(crtc);
	const struct tegra_disp_client *dci =
		tegra_la_get_disp_client_info(dc->mc,
					      dc->mc_win_clients[tp->index]);
	unsigned int bw_mbps = DIV_ROUND_UP(bw_kbps, 1000);
	unsigned int bw_mbps_fp = LA_REAL_TO_FP(bw_mbps);
	bool win_rotated;
	unsigned int surface_width;
	bool vertical_scaling_enabled;
	bool pitch = (tps->tiling.mode == TEGRA_BO_TILING_MODE_PITCH);
	bool planar, yuv;
	bool packed_yuv422 = ((tps->format == WIN_COLOR_DEPTH_YCbCr422) ||
			      (tps->format == WIN_COLOR_DEPTH_YUV422));
	unsigned int bytes_per_pixel;
	struct drm_display_mode *mode = &crtc->state->mode;
	unsigned int total_h = mode->htotal;
	unsigned int total_v = mode->vtotal;
	unsigned int total_screen_area = total_h * total_v;
	unsigned int total_active_area = mode->hdisplay * mode->vdisplay;
	unsigned int total_blank_area = total_screen_area - total_active_area;
	unsigned int c1_fp, c2, c3;
	unsigned int bpp_for_line_buffer_storage_fp;
	unsigned int latency_buffering_available_in_reqd_buffering_fp;
	unsigned long emc_freq_mhz = emc_freq_hz / 1000000;
	unsigned int bw_disruption_time_usec_fp;
	unsigned int scaled_row_srt_sz_bytes;
	unsigned int static_la_snap_to_row_srt_emcclk;
	unsigned int slow_row_srt_sz_bytes_fp;
	unsigned int effective_row_srt_sz_bytes_fp;
	unsigned int drain_time_usec_fp;
	unsigned int total_latency_usec_fp;
	unsigned int bw_disruption_buffering_bytes_fp;
	unsigned int reqd_lines = 0;
	unsigned int lines_of_latency;
	unsigned int thresh_lwm_bytes_fp;
	unsigned int total_buf_sz_bytes = dci->line_buf_sz_bytes +
					  dci->mccif_size_bytes;
	unsigned int total_vblank_bw;
	unsigned int bw_display_fp, bw_delta_fp, bw_other_wins;
	unsigned int fill_rate_other_wins_fp;
	unsigned int dvfs_time_nsec = tegra_emc_get_clk_latency(emc_freq_hz);
	unsigned int data_shortfall_other_wins_fp;
	unsigned int duration_usec_fp;
	unsigned int spool_up_buffering_adj_bytes = 0;

	bw_disruption_time_usec_fp = LA_BW_DISRUPTION_TIME_EMCCLKS_FP /
				     emc_freq_mhz;

	scaled_row_srt_sz_bytes = min((unsigned long)LA_ROW_SRT_SZ_BYTES,
				      16 * min(emc_freq_mhz + 50, 400ul));

	static_la_snap_to_row_srt_emcclk =
		LA_FP_TO_REAL(LA_STATIC_LA_SNAP_ARB_TO_ROW_SRT_EMCCLKS_FP);

	slow_row_srt_sz_bytes_fp = (LA_MAX_DRAIN_TIME_USEC * emc_freq_mhz -
				    static_la_snap_to_row_srt_emcclk) *
				   2 * LA_DRAM_WIDTH_BITS /
				   8 * LA_CONS_MEM_EFFICIENCY_FP;

	effective_row_srt_sz_bytes_fp =
		min(LA_REAL_TO_FP(scaled_row_srt_sz_bytes),
		    slow_row_srt_sz_bytes_fp);

	drain_time_usec_fp = effective_row_srt_sz_bytes_fp * LA_FP_FACTOR /
			     (emc_freq_mhz * LA_DRAM_WIDTH_BITS / 4 *
			     LA_CONS_MEM_EFFICIENCY_FP) +
			     LA_STATIC_LA_SNAP_ARB_TO_ROW_SRT_EMCCLKS_FP /
			     emc_freq_mhz;

	total_latency_usec_fp = drain_time_usec_fp +
				LA_ST_LA_MINUS_SNAP_ARB_TO_ROW_SRT_EMCCLKS_FP /
				emc_freq_mhz;

	bw_disruption_buffering_bytes_fp = bw_mbps *
					   max(bw_disruption_time_usec_fp,
					       total_latency_usec_fp) +
					   LA_REAL_TO_FP(1) / 2;

	if (tegra_dc_format_is_yuv(tps->format, &planar)) {
		yuv = true;
		bytes_per_pixel = 2;
	} else {
		yuv = false;
		bytes_per_pixel = tps->base.fb->bits_per_pixel / 8;
	}

	disp_params->drain_time_usec_fp = drain_time_usec_fp;

	if (dc->soc->supports_scan_column &&
			((BIT(DRM_ROTATE_90) | BIT(DRM_ROTATE_270)) &
				tps->base.rotation)) {
		win_rotated = true;
		surface_width = tps->base.src_h >> 16;
		vertical_scaling_enabled =
			((tps->base.src_w >> 16) == tps->base.crtc_w) ?
				false : true;
	} else {
		win_rotated = false;
		surface_width = tps->base.src_w >> 16;
		vertical_scaling_enabled =
			((tps->base.src_h >> 16) == tps->base.crtc_h) ?
				false : true;
	}

	if ((dci->line_buf_sz_bytes == 0) || (pitch == true)) {
		reqd_lines = 0;
	} else if (win_rotated && planar) {
		if (vertical_scaling_enabled)
			reqd_lines = 17;
		else
			reqd_lines = 16;
	} else {
		if (win_rotated) {
			if (vertical_scaling_enabled)
				reqd_lines = DIV_ROUND_UP(16, bytes_per_pixel);
			else
				reqd_lines = 16 / bytes_per_pixel;
		} else {
			if (vertical_scaling_enabled)
				reqd_lines = 3;
			else
				reqd_lines = 1;
		}
	}

	if (reqd_lines > 0 && !vertical_scaling_enabled && win_rotated)
		lines_of_latency = 1;
	else
		lines_of_latency = 0;

	if (((tps->format == WIN_COLOR_DEPTH_YCbCr422R) ||
	     (tps->format == WIN_COLOR_DEPTH_YUV422R) ||
	     (tps->format == WIN_COLOR_DEPTH_YCbCr422RA) ||
	     (tps->format == WIN_COLOR_DEPTH_YUV422RA)) &&
	    !win_rotated) {
		c1_fp = LA_REAL_TO_FP(5) / 2;
	} else {
		c1_fp = LA_REAL_TO_FP(1);
	}

	if ((((tps->format == WIN_COLOR_DEPTH_YCbCr420P) ||
	      (tps->format == WIN_COLOR_DEPTH_YUV420P) ||
	      (tps->format == WIN_COLOR_DEPTH_YCrCb420SP) ||
	      (tps->format == WIN_COLOR_DEPTH_YCbCr420SP)) &&
	    !win_rotated) || (yuv && win_rotated)) {
		c2 = 3;
	} else {
		c2 = bytes_per_pixel;
	}

	c3 = (packed_yuv422 && win_rotated) ? 2 : 1;
	latency_buffering_available_in_reqd_buffering_fp = surface_width *
							   lines_of_latency *
							   c1_fp * c2 * c3;

	switch (tps->format) {
	/* YUV 420 case*/
	case WIN_COLOR_DEPTH_YCbCr420P:
	case WIN_COLOR_DEPTH_YUV420P:
	case WIN_COLOR_DEPTH_YCrCb420SP:
	case WIN_COLOR_DEPTH_YCbCr420SP:
		c1_fp = (win_rotated) ?  LA_REAL_TO_FP(2) : LA_REAL_TO_FP(3);
		break;

	/* YUV 422 case */
	case WIN_COLOR_DEPTH_YCbCr422:
	case WIN_COLOR_DEPTH_YUV422:
	case WIN_COLOR_DEPTH_YCbCr422P:
	case WIN_COLOR_DEPTH_YUV422P:
		c1_fp = (win_rotated) ?  LA_REAL_TO_FP(3) : LA_REAL_TO_FP(2);
		break;

	/* YUV 422R case */
	case WIN_COLOR_DEPTH_YCbCr422R:
	case WIN_COLOR_DEPTH_YUV422R:
	case WIN_COLOR_DEPTH_YCbCr422RA:
	case WIN_COLOR_DEPTH_YUV422RA:
		c1_fp = (win_rotated) ?  LA_REAL_TO_FP(2) : LA_REAL_TO_FP(5);
		break;

	default:
		c1_fp = LA_REAL_TO_FP(bytes_per_pixel);
		break;
	}

	c2 = (packed_yuv422 && win_rotated) ? 2 : 1;
	bpp_for_line_buffer_storage_fp = c1_fp * c2;

	thresh_lwm_bytes_fp = surface_width * reqd_lines *
			      bpp_for_line_buffer_storage_fp;

	if (bw_disruption_buffering_bytes_fp >=
	    latency_buffering_available_in_reqd_buffering_fp)
		thresh_lwm_bytes_fp += bw_disruption_buffering_bytes_fp -
			       latency_buffering_available_in_reqd_buffering_fp;

	disp_params->thresh_lwm_bytes = LA_FP_TO_REAL(thresh_lwm_bytes_fp);

	if ((dci->win_type == TEGRA_LA_DISP_WIN_TYPE_FULL) ||
	    (dci->win_type == TEGRA_LA_DISP_WIN_TYPE_FULLA) ||
	    (dci->win_type == TEGRA_LA_DISP_WIN_TYPE_FULLB)) {
		total_vblank_bw = total_buf_sz_bytes / total_blank_area;
	} else {
		total_vblank_bw = 0;
	}

	bw_display_fp = LA_DISP_CATCHUP_FACTOR_FP *
			max(total_active_space_bw, total_vblank_bw);
	bw_delta_fp = bw_mbps_fp - (bw_display_fp / num_wins);

	bw_other_wins = total_active_space_bw - bw_mbps;

	if (num_wins > 0) {
		fill_rate_other_wins_fp = bw_display_fp * (num_wins - 1) /
			       num_wins - LA_REAL_TO_FP(bw_other_wins);
	} else {
		fill_rate_other_wins_fp = 0;
	}

	data_shortfall_other_wins_fp = dvfs_time_nsec * bw_other_wins *
				       LA_FP_FACTOR / 1000;

	duration_usec_fp = (fill_rate_other_wins_fp == 0) ? 0 :
			   data_shortfall_other_wins_fp * LA_FP_FACTOR /
			   fill_rate_other_wins_fp;

	if (bw_delta_fp > 0)
		spool_up_buffering_adj_bytes = bw_delta_fp * duration_usec_fp /
					       LA_FP_FACTOR / LA_FP_FACTOR;

	disp_params->spool_up_buffering_adj_bytes =
						spool_up_buffering_adj_bytes;
}

/*
 * This function helps to select the correct plane emc bandwidth based on
 * the following rule:
 * 1. Use the new emc bandwidth in the following 3 cases
 *    a. If plane's emc bw is increasing, and it is called from the
 *    "tegra_dc_update_emc_pre_commit" function
 *    b. If plane's emc bw is decreasing, and it is called from the
 *    "tegra_dc_update_emc_post_commit" function
 *    c. If plane's emc bw is increasing, and it is called from the
 *    "tegra_dc_update_emc_post_commit" function
 *
 * 2. Use the old emc bandwidth in the following 2 case
 *    a. If plane's emc bw is decreasing, and it is called from the
 *    "tegra_dc_update_emc_pre_commit" function
 */
static u64 tegra_dc_get_plane_emc_bw(struct tegra_plane_state *tps,
					struct tegra_plane_state *old_tps,
					enum la_ptsa_update update)
{
	if (tps->plane_update_la_ptsa == update)
		return tps->plane_emc_bw;

	if (tps->plane_update_la_ptsa == LA_PTSA_PRE_UPDATE)
		return tps->plane_emc_bw;
	else
		return old_tps->plane_emc_bw;
}

static int tegra_dc_set_latency_allowance(struct drm_crtc *crtc,
					  unsigned long emc_freq,
					  struct drm_atomic_state *old_state,
					  enum la_ptsa_update update)
{
	struct tegra_dc *dc = to_tegra_dc(crtc);
	struct tegra_dc_state *dcs = to_dc_state(crtc->state);
	struct drm_plane *plane;
	struct tegra_plane *tp;
	struct drm_plane_state *old_plane_state;
	struct tegra_plane_state *tps;
	struct clk *emc = clk_get_parent(dc->emc_clk);
	struct tegra_dc_to_la_params disp_params = {0};
	unsigned int emc_freq_hz;
	unsigned int emc_la_freq_hz = 0;
	int i, num_wins = 0;
	unsigned int total_active_space_bw = 0;
	u64 plane_emc_bw;
	struct tegra_dc_state *old_dcs;
	struct tegra_plane_state *old_tps;

	if (!dc->emc_la_clk)
		return 0;

	old_dcs = to_dc_state(old_state->crtc_states[drm_crtc_index(crtc)]);

	for_each_plane_in_state(old_state, plane, old_plane_state, i) {
		if (!plane->state->crtc || plane->state->crtc != crtc ||
		    !plane->state->fb)
			continue;

		old_tps = to_tegra_plane_state(old_plane_state);
		tps = to_tegra_plane_state(plane->state);
		num_wins++;

		plane_emc_bw = tegra_dc_get_plane_emc_bw(tps, old_tps, update);

		total_active_space_bw +=
			DIV_ROUND_UP_ULL(plane_emc_bw, 1000);
	}

	for_each_plane_in_state(old_state, plane, old_plane_state, i) {
		if (!plane->state->crtc || plane->state->crtc != crtc ||
		    !plane->state->fb)
			continue;

		old_tps = to_tegra_plane_state(old_plane_state);
		tp = to_tegra_plane(plane);
		tps = to_tegra_plane_state(plane->state);

		plane_emc_bw = tegra_dc_get_plane_emc_bw(tps, old_tps, update);

		emc_freq_hz = tegra_emc_bw_to_freq_req(plane_emc_bw) * 1000;

		if (!emc_freq_hz)
			continue;
		/*
		 * use clk_round_rate on root emc clock instead to
		 * get correct rate.
		 */
		emc_freq_hz = clk_round_rate(emc, emc_freq_hz);

		while (1) {
			int err;
			unsigned long next_freq = 0;
			unsigned long emc_bandwidth;

			calc_disp_params(crtc, tp, tps, num_wins,
					 dc->mc_win_clients[tp->index],
					 emc_freq_hz, total_active_space_bw,
					 plane_emc_bw, &disp_params);

			/*
			 * If called from pre commit, we should use the larger
			 * emc_bandwidth to avoid the early emc rate drop.
			 * If called from post commit, use the new one
			 */
			if (update == LA_PTSA_PRE_UPDATE)
				emc_bandwidth = max(old_dcs->emc_bandwidth,
						    dcs->emc_bandwidth);
			else
				emc_bandwidth = dcs->emc_bandwidth;


			if (dc->pipe)
				disp_params.total_dc1_bw = emc_bandwidth;
			else
				disp_params.total_dc0_bw = emc_bandwidth;

			err = tegra_la_set_disp_la(dc->mc,
						  dc->mc_win_clients[tp->index],
						  emc_freq_hz,
						  plane_emc_bw,
						  disp_params);
			if (!err) {
				if (emc_freq_hz > emc_la_freq_hz) {
					emc_la_freq_hz = emc_freq_hz;
					clk_set_rate(dc->emc_la_clk,
						emc_la_freq_hz);
				}
				break;
			}

			next_freq = clk_round_rate(emc, emc_freq_hz + 1000000);
			if (emc_freq_hz != next_freq)
				emc_freq_hz = next_freq;
			else
				break;
		}
	}

	return 0;
}

static int tegra_dc_program_bandwidth(struct drm_crtc *crtc,
				      struct drm_atomic_state *old_state)
{
	struct tegra_dc *dc;
	struct tegra_dc_state *dc_state;
	unsigned long freq;
	int err;

	dc = to_tegra_dc(crtc);
	dc_state = to_dc_state(crtc->state);
	if (!dc->emc_clk)
		return 0;

	freq = tegra_emc_bw_to_freq_req(dc_state->emc_bandwidth) * 1000;

	err = clk_set_rate(dc->emc_clk, freq);
	if (err) {
		dev_err(dc->dev, "Set DC emc clock to %lu failed: %d\n",
			freq, err);
		return err;
	}

	return 0;
}

static u64 tegra_dc_calculate_bandwidth(int pclk, struct drm_plane_state *state)
{
	struct tegra_dc *dc = to_tegra_dc(state->crtc);
	struct tegra_plane_state *tps = to_tegra_plane_state(state);
	u64 ret;
	unsigned long bpp;
	unsigned in_w, out_w, in_h, out_h;
	bool yuv;

	if (IS_ERR_OR_NULL(state->fb))
		return 0;

	if (dc->soc->supports_scan_column &&
			((BIT(DRM_ROTATE_90) | BIT(DRM_ROTATE_270)) &
				state->rotation)) {
		in_w = state->src_h >> 16;
		out_w = state->crtc_h;
		in_h = state->src_w >> 16;
		out_h = state->crtc_w;
	} else {
		in_w = state->src_w >> 16;
		out_w = state->crtc_w;
		in_h = state->src_h >> 16;
		out_h = state->crtc_h;
	}

	yuv = tegra_dc_format_is_yuv(tps->format, NULL);
	if (!yuv)
		bpp = state->fb->bits_per_pixel / 8;
	else
		bpp = 2;

	ret = pclk * bpp;  /* pclk is KHz */
	ret *= in_w;
	ret = div_u64(ret, out_w);

	if (in_h > out_h) {
		/* vertical downscaling enabled  */
		ret *= in_h;
		ret = div_u64(ret, out_h);
	}

	return ret; /* in KBps */
}

static unsigned long tegra_dc_calculate_overlap(struct drm_crtc *crtc,
					struct drm_atomic_state *state)
{
	struct tegra_plane_state *tps;
	struct drm_plane *pa;
	struct drm_plane_state *pa_state;
	struct drm_plane *pb;
	struct drm_plane_state *pb_state;
	int i, j;
	unsigned long result = 0, bw;

	for_each_plane_in_state(state, pa, pa_state, i) {
		if (pa_state->crtc != crtc)
			continue;

		bw = 0;
		for_each_plane_in_state(state, pb, pb_state, j) {
			if (pb_state->crtc != crtc)
				continue;

			if ((pa_state->crtc_y >= pb_state->crtc_y) &&
			    (pa_state->crtc_y <=
				(pb_state->crtc_y + pb_state->crtc_h - 1))) {
				tps = to_tegra_plane_state(pb_state);
				bw += tps->plane_emc_bw;
			}
		}

		if (result < bw)
			result = bw;
	}

	return result;
}

int tegra_dc_evaluate_bandwidth(struct drm_atomic_state *state)
{
	struct tegra_dc_state *dc_state;
	struct tegra_plane_state *tp_state;
	struct drm_plane *plane;
	struct drm_plane_state *plane_state;
	struct drm_crtc *crtc;
	struct drm_crtc_state *crtc_state;
	int i, j;
	u64 new_plane_emc_bw;
	bool update_la_ptsa = false;

	for_each_plane_in_state(state, plane, plane_state, i) {
		/* Ignore planes which not updating */
		if (!plane_state->crtc)
			continue;

		for_each_crtc_in_state(state, crtc, crtc_state, j) {
			if (crtc != plane_state->crtc)
				continue;
			break;
		}

		tp_state = to_tegra_plane_state(plane_state);

		if (crtc_state->active)
			new_plane_emc_bw = tegra_dc_calculate_bandwidth(
						crtc_state->adjusted_mode.clock,
						plane_state);
		else
			new_plane_emc_bw = 0;

		if (tp_state->plane_emc_bw != new_plane_emc_bw) {
			tp_state->plane_update_la_ptsa =
				(tp_state->plane_emc_bw > new_plane_emc_bw) ?
					LA_PTSA_POST_UPDATE :
						LA_PTSA_PRE_UPDATE;
			tp_state->plane_emc_bw = new_plane_emc_bw;

			/*
			 * Need to update la/ptsa registers if any
			 * plane's emc bandwidth is changed.
			 */
			update_la_ptsa = true;
		} else {
			tp_state->plane_update_la_ptsa = LA_PTSA_PRE_UPDATE;
		}
	}

	for_each_crtc_in_state(state, crtc, crtc_state, i) {
		dc_state = to_dc_state(crtc_state);
		dc_state->emc_bandwidth = tegra_dc_calculate_overlap(crtc,
								state);
		dc_state->update_la_ptsa = update_la_ptsa;
	}

	/* TODO: call MC function to see whether this bandwidth is sane. */

	return 0;
}

void tegra_dc_update_emc_pre_commit(struct drm_atomic_state *old_state)
{
	struct drm_crtc *crtc;
	struct drm_crtc_state *old_crtc_state;
	struct tegra_dc *dc;
	struct tegra_dc_state *old_dc_state;
	struct tegra_dc_state *new_dc_state;
	int i, ret;
	bool update_emc;
	unsigned long freq;

	for_each_crtc_in_state(old_state, crtc, old_crtc_state, i) {
		dc = to_tegra_dc(crtc);
		old_dc_state = to_dc_state(old_crtc_state);
		new_dc_state = to_dc_state(crtc->state);

		/* Don't drop the emc clock early if it's decreasing */
		update_emc = new_dc_state->emc_bandwidth >
			     old_dc_state->emc_bandwidth;

		if (new_dc_state->emc_bandwidth == old_dc_state->emc_bandwidth)
			new_dc_state->update_emc = false;

		/* Program the emc early if the bandwidth is increasing */
		if (update_emc) {
			ret = tegra_dc_program_bandwidth(crtc, old_state);
			if (ret)
				DRM_ERROR("Program emc bandwidth failed %d\n",
					  ret);
			else
				new_dc_state->update_emc = false;
			/*
			 * Forcely update la/ptsa registers as long as the
			 * dc's emc bandwidth is changed, regardless whether
			 * any plane's emc bandwidth is changed
			 */
			new_dc_state->update_la_ptsa = true;
		}

		if (new_dc_state->update_la_ptsa) {
			freq = tegra_emc_bw_to_freq_req(
				new_dc_state->emc_bandwidth) * 1000;

			/* Program MC LA/PTSA registers */
			ret = tegra_dc_set_latency_allowance(
				crtc, freq, old_state, LA_PTSA_PRE_UPDATE);
			if (ret)
				DRM_ERROR("Set DC LA failed: %d\n", ret);
		}
	}
}

void tegra_dc_update_emc_post_commit(struct drm_atomic_state *old_state)
{
	struct drm_crtc *crtc;
	struct drm_crtc_state *old_crtc_state;
	struct tegra_dc *dc;
	struct tegra_dc_state *new_dc_state;
	int i, ret;
	unsigned long freq;

	for_each_crtc_in_state(old_state, crtc, old_crtc_state, i) {
		dc = to_tegra_dc(crtc);
		new_dc_state = to_dc_state(crtc->state);

		if (!new_dc_state)
			continue;

		if (new_dc_state->update_emc) {
			ret = tegra_dc_program_bandwidth(crtc, old_state);
			if (ret)
				DRM_ERROR(
				"Failed to program emc bandwidth %d\n", ret);
			else
				new_dc_state->update_emc = false;

			/*
			 * Forcely update la/ptsa registers as long as the
			 * dc's emc bandwidth is changed, regardless whether
			 * any plane's emc bandwidth is changed
			 */
			new_dc_state->update_la_ptsa = true;
		}

		if (new_dc_state->update_la_ptsa) {
			freq = tegra_emc_bw_to_freq_req(
				new_dc_state->emc_bandwidth) * 1000;

			/* Program MC LA/PTSA registers */
			ret = tegra_dc_set_latency_allowance(
				crtc, freq, old_state, LA_PTSA_POST_UPDATE);
			if (ret)
				DRM_ERROR(
				"Set DC latency allowance failed: %d\n", ret);
		}
	}
}

static void tegra_crtc_atomic_flush(struct drm_crtc *crtc,
				    struct drm_crtc_state *old_crtc_state)
{
	struct tegra_dc_state *state = to_dc_state(crtc->state);
	struct tegra_dc *dc = to_tegra_dc(crtc);
	unsigned long flags;
	u32 value;

	spin_lock_irqsave(&dc->lock, flags);
	if (crtc->state->mode_changed)
		dc->in_modeset = false;

	if (!crtc->state->active)
		goto out;

	tegra_dc_writel(dc, state->planes << 8, DC_CMD_STATE_CONTROL);
	value = state->planes;
	if (state->nc_mode)
		value |= GENERAL_ACT_REQ | NC_HOST_TRIG;
	tegra_dc_writel(dc, value, DC_CMD_STATE_CONTROL);

out:
	spin_unlock_irqrestore(&dc->lock, flags);
}

static const struct drm_crtc_helper_funcs tegra_crtc_helper_funcs = {
	.disable = tegra_crtc_disable,
	.mode_fixup = tegra_crtc_mode_fixup,
	.mode_set_nofb = tegra_crtc_mode_set_nofb,
	.prepare = tegra_crtc_prepare,
	.commit = tegra_crtc_commit,
	.atomic_check = tegra_crtc_atomic_check,
	.atomic_begin = tegra_crtc_atomic_begin,
	.atomic_flush = tegra_crtc_atomic_flush,
};

static irqreturn_t tegra_dc_irq(int irq, void *data)
{
	struct tegra_dc *dc = data;
	unsigned long status;

	status = tegra_dc_readl(dc, DC_CMD_INT_STATUS);
	tegra_dc_writel(dc, status, DC_CMD_INT_STATUS);

	if (status & FRAME_END_INT) {
		/*
		dev_dbg(dc->dev, "%s(): frame end\n", __func__);
		*/
		dc->stats.frames++;
	}

	if ((status & VBLANK_INT) || (status & MSF_INT)) {
		/*
		dev_dbg(dc->dev, "%s(): vertical blank %x\n", __func__, status);
		*/
		drm_crtc_handle_vblank(&dc->base);
		tegra_dc_finish_page_flip(dc);
		dc->stats.vblank++;
	}

	if (status & (WIN_A_UF_INT | WIN_B_UF_INT | WIN_C_UF_INT)) {
		/*
		dev_dbg(dc->dev, "%s(): underflow\n", __func__);
		*/
		dc->stats.underflow++;
	}

	if (status & (WIN_A_OF_INT | WIN_B_OF_INT | WIN_C_OF_INT)) {
		/*
		dev_dbg(dc->dev, "%s(): overflow\n", __func__);
		*/
		dc->stats.overflow++;
	}

	return IRQ_HANDLED;
}

static int tegra_dc_slcg_handler(struct notifier_block *nb,
	unsigned long unused0, void *unused1)
{
	struct tegra_dc *dc = container_of(nb, struct tegra_dc, slcg_notifier);
	u32 val;

	val = tegra_dc_readl(dc, DC_COM_DSC_TOP_CTL);
	val |= DSC_SLCG_OVERRIDE;
	tegra_dc_writel(dc, val, DC_COM_DSC_TOP_CTL);

	/* flush the previous write */
	(void)tegra_dc_readl(dc, DC_CMD_DISPLAY_COMMAND);
	val &= ~DSC_SLCG_OVERRIDE;
	tegra_dc_writel(dc, val, DC_COM_DSC_TOP_CTL);

	return NOTIFY_OK;
}

void tegra_dc_force_update(struct drm_crtc *crtc)
{
	struct tegra_dc *dc = to_tegra_dc(crtc);
	unsigned long flags;
	bool in_modeset;

	/* If we're currently in modeset, let atomic_flush handle updates */
	spin_lock_irqsave(&dc->lock, flags);
	in_modeset = dc->in_modeset;
	spin_unlock_irqrestore(&dc->lock, flags);

	if (dc->dpms == DRM_MODE_DPMS_OFF || in_modeset)
		return;

	WARN_ON(drm_crtc_vblank_get(crtc) != 0);
	tegra_dc_writel(dc, GENERAL_ACT_REQ | NC_HOST_TRIG,
			DC_CMD_STATE_CONTROL);
	drm_crtc_wait_one_vblank(crtc);
	drm_crtc_vblank_put(crtc);
}

static int tegra_dc_show_regs(struct seq_file *s, void *data)
{
	struct drm_info_node *node = s->private;
	struct tegra_dc *dc = node->info_ent->data;
	unsigned long flags;
	u32 win_header_save;
	u32 state_access_save;
	int i;

	if (!tegra_powergate_is_powered(dc->powergate)) {
		DRM_INFO("Can't dump registers as dc is powergated\n");
		return -EPERM;
	}

	spin_lock_irqsave(&dc->lock, flags);
	state_access_save = tegra_dc_readl(dc, DC_CMD_STATE_ACCESS);
	tegra_dc_writel(dc, state_access_save | READ_MUX, DC_CMD_STATE_ACCESS);

#define DUMP_REG(name)						\
	seq_printf(s, "%-40s %#05x %08x\n", #name, name,	\
		   tegra_dc_readl(dc, name))

	DUMP_REG(DC_CMD_GENERAL_INCR_SYNCPT);
	DUMP_REG(DC_CMD_GENERAL_INCR_SYNCPT_CNTRL);
	DUMP_REG(DC_CMD_GENERAL_INCR_SYNCPT_ERROR);
	DUMP_REG(DC_CMD_WIN_A_INCR_SYNCPT);
	DUMP_REG(DC_CMD_WIN_A_INCR_SYNCPT_CNTRL);
	DUMP_REG(DC_CMD_WIN_A_INCR_SYNCPT_ERROR);
	DUMP_REG(DC_CMD_WIN_B_INCR_SYNCPT);
	DUMP_REG(DC_CMD_WIN_B_INCR_SYNCPT_CNTRL);
	DUMP_REG(DC_CMD_WIN_B_INCR_SYNCPT_ERROR);
	DUMP_REG(DC_CMD_WIN_C_INCR_SYNCPT);
	DUMP_REG(DC_CMD_WIN_C_INCR_SYNCPT_CNTRL);
	DUMP_REG(DC_CMD_WIN_C_INCR_SYNCPT_ERROR);
	DUMP_REG(DC_CMD_CONT_SYNCPT_VSYNC);
	DUMP_REG(DC_CMD_DISPLAY_COMMAND_OPTION0);
	DUMP_REG(DC_CMD_DISPLAY_COMMAND);
	DUMP_REG(DC_CMD_SIGNAL_RAISE);
	DUMP_REG(DC_CMD_DISPLAY_POWER_CONTROL);
	DUMP_REG(DC_CMD_INT_STATUS);
	DUMP_REG(DC_CMD_INT_MASK);
	DUMP_REG(DC_CMD_INT_ENABLE);
	DUMP_REG(DC_CMD_INT_TYPE);
	DUMP_REG(DC_CMD_INT_POLARITY);
	DUMP_REG(DC_CMD_SIGNAL_RAISE1);
	DUMP_REG(DC_CMD_SIGNAL_RAISE2);
	DUMP_REG(DC_CMD_SIGNAL_RAISE3);
	DUMP_REG(DC_CMD_STATE_ACCESS);
	DUMP_REG(DC_CMD_STATE_CONTROL);
	DUMP_REG(DC_CMD_DISPLAY_WINDOW_HEADER);
	DUMP_REG(DC_CMD_REG_ACT_CONTROL);
	DUMP_REG(DC_COM_CRC_CONTROL);
	DUMP_REG(DC_COM_CRC_CHECKSUM);
	DUMP_REG(DC_COM_PIN_OUTPUT_ENABLE(0));
	DUMP_REG(DC_COM_PIN_OUTPUT_ENABLE(1));
	DUMP_REG(DC_COM_PIN_OUTPUT_ENABLE(2));
	DUMP_REG(DC_COM_PIN_OUTPUT_ENABLE(3));
	DUMP_REG(DC_COM_PIN_OUTPUT_POLARITY(0));
	DUMP_REG(DC_COM_PIN_OUTPUT_POLARITY(1));
	DUMP_REG(DC_COM_PIN_OUTPUT_POLARITY(2));
	DUMP_REG(DC_COM_PIN_OUTPUT_POLARITY(3));
	DUMP_REG(DC_COM_PIN_OUTPUT_DATA(0));
	DUMP_REG(DC_COM_PIN_OUTPUT_DATA(1));
	DUMP_REG(DC_COM_PIN_OUTPUT_DATA(2));
	DUMP_REG(DC_COM_PIN_OUTPUT_DATA(3));
	DUMP_REG(DC_COM_PIN_INPUT_ENABLE(0));
	DUMP_REG(DC_COM_PIN_INPUT_ENABLE(1));
	DUMP_REG(DC_COM_PIN_INPUT_ENABLE(2));
	DUMP_REG(DC_COM_PIN_INPUT_ENABLE(3));
	DUMP_REG(DC_COM_PIN_INPUT_DATA(0));
	DUMP_REG(DC_COM_PIN_INPUT_DATA(1));
	DUMP_REG(DC_COM_PIN_OUTPUT_SELECT(0));
	DUMP_REG(DC_COM_PIN_OUTPUT_SELECT(1));
	DUMP_REG(DC_COM_PIN_OUTPUT_SELECT(2));
	DUMP_REG(DC_COM_PIN_OUTPUT_SELECT(3));
	DUMP_REG(DC_COM_PIN_OUTPUT_SELECT(4));
	DUMP_REG(DC_COM_PIN_OUTPUT_SELECT(5));
	DUMP_REG(DC_COM_PIN_OUTPUT_SELECT(6));
	DUMP_REG(DC_COM_PIN_MISC_CONTROL);
	DUMP_REG(DC_COM_PIN_PM0_CONTROL);
	DUMP_REG(DC_COM_PIN_PM0_DUTY_CYCLE);
	DUMP_REG(DC_COM_PIN_PM1_CONTROL);
	DUMP_REG(DC_COM_PIN_PM1_DUTY_CYCLE);
	DUMP_REG(DC_COM_SPI_CONTROL);
	DUMP_REG(DC_COM_SPI_START_BYTE);
	DUMP_REG(DC_COM_HSPI_WRITE_DATA_AB);
	DUMP_REG(DC_COM_HSPI_WRITE_DATA_CD);
	DUMP_REG(DC_COM_HSPI_CS_DC);
	DUMP_REG(DC_COM_SCRATCH_REGISTER_A);
	DUMP_REG(DC_COM_SCRATCH_REGISTER_B);
	DUMP_REG(DC_COM_GPIO_CTRL);
	DUMP_REG(DC_COM_GPIO_DEBOUNCE_COUNTER);
	DUMP_REG(DC_COM_CRC_CHECKSUM_LATCHED);
	DUMP_REG(DC_DISP_DISP_SIGNAL_OPTIONS0);
	DUMP_REG(DC_DISP_DISP_SIGNAL_OPTIONS1);
	DUMP_REG(DC_DISP_DISP_WIN_OPTIONS);
	DUMP_REG(DC_DISP_DISP_MEM_HIGH_PRIORITY);
	DUMP_REG(DC_DISP_DISP_MEM_HIGH_PRIORITY_TIMER);
	DUMP_REG(DC_DISP_DISP_TIMING_OPTIONS);
	DUMP_REG(DC_DISP_REF_TO_SYNC);
	DUMP_REG(DC_DISP_SYNC_WIDTH);
	DUMP_REG(DC_DISP_BACK_PORCH);
	DUMP_REG(DC_DISP_ACTIVE);
	DUMP_REG(DC_DISP_FRONT_PORCH);
	DUMP_REG(DC_DISP_H_PULSE0_CONTROL);
	DUMP_REG(DC_DISP_H_PULSE0_POSITION_A);
	DUMP_REG(DC_DISP_H_PULSE0_POSITION_B);
	DUMP_REG(DC_DISP_H_PULSE0_POSITION_C);
	DUMP_REG(DC_DISP_H_PULSE0_POSITION_D);
	DUMP_REG(DC_DISP_H_PULSE1_CONTROL);
	DUMP_REG(DC_DISP_H_PULSE1_POSITION_A);
	DUMP_REG(DC_DISP_H_PULSE1_POSITION_B);
	DUMP_REG(DC_DISP_H_PULSE1_POSITION_C);
	DUMP_REG(DC_DISP_H_PULSE1_POSITION_D);
	DUMP_REG(DC_DISP_H_PULSE2_CONTROL);
	DUMP_REG(DC_DISP_H_PULSE2_POSITION_A);
	DUMP_REG(DC_DISP_H_PULSE2_POSITION_B);
	DUMP_REG(DC_DISP_H_PULSE2_POSITION_C);
	DUMP_REG(DC_DISP_H_PULSE2_POSITION_D);
	DUMP_REG(DC_DISP_V_PULSE0_CONTROL);
	DUMP_REG(DC_DISP_V_PULSE0_POSITION_A);
	DUMP_REG(DC_DISP_V_PULSE0_POSITION_B);
	DUMP_REG(DC_DISP_V_PULSE0_POSITION_C);
	DUMP_REG(DC_DISP_V_PULSE1_CONTROL);
	DUMP_REG(DC_DISP_V_PULSE1_POSITION_A);
	DUMP_REG(DC_DISP_V_PULSE1_POSITION_B);
	DUMP_REG(DC_DISP_V_PULSE1_POSITION_C);
	DUMP_REG(DC_DISP_V_PULSE2_CONTROL);
	DUMP_REG(DC_DISP_V_PULSE2_POSITION_A);
	DUMP_REG(DC_DISP_V_PULSE3_CONTROL);
	DUMP_REG(DC_DISP_V_PULSE3_POSITION_A);
	DUMP_REG(DC_DISP_M0_CONTROL);
	DUMP_REG(DC_DISP_M1_CONTROL);
	DUMP_REG(DC_DISP_DI_CONTROL);
	DUMP_REG(DC_DISP_PP_CONTROL);
	DUMP_REG(DC_DISP_PP_SELECT_A);
	DUMP_REG(DC_DISP_PP_SELECT_B);
	DUMP_REG(DC_DISP_PP_SELECT_C);
	DUMP_REG(DC_DISP_PP_SELECT_D);
	DUMP_REG(DC_DISP_DISP_CLOCK_CONTROL);
	DUMP_REG(DC_DISP_DISP_INTERFACE_CONTROL);
	DUMP_REG(DC_DISP_DISP_COLOR_CONTROL);
	DUMP_REG(DC_DISP_SHIFT_CLOCK_OPTIONS);
	DUMP_REG(DC_DISP_DATA_ENABLE_OPTIONS);
	DUMP_REG(DC_DISP_SERIAL_INTERFACE_OPTIONS);
	DUMP_REG(DC_DISP_LCD_SPI_OPTIONS);
	DUMP_REG(DC_DISP_BORDER_COLOR);
	DUMP_REG(DC_DISP_COLOR_KEY0_LOWER);
	DUMP_REG(DC_DISP_COLOR_KEY0_UPPER);
	DUMP_REG(DC_DISP_COLOR_KEY1_LOWER);
	DUMP_REG(DC_DISP_COLOR_KEY1_UPPER);
	DUMP_REG(DC_DISP_CURSOR_FOREGROUND);
	DUMP_REG(DC_DISP_CURSOR_BACKGROUND);
	DUMP_REG(DC_DISP_CURSOR_START_ADDR);
	DUMP_REG(DC_DISP_CURSOR_START_ADDR_NS);
	DUMP_REG(DC_DISP_CURSOR_POSITION);
	DUMP_REG(DC_DISP_CURSOR_POSITION_NS);
	DUMP_REG(DC_DISP_INIT_SEQ_CONTROL);
	DUMP_REG(DC_DISP_SPI_INIT_SEQ_DATA_A);
	DUMP_REG(DC_DISP_SPI_INIT_SEQ_DATA_B);
	DUMP_REG(DC_DISP_SPI_INIT_SEQ_DATA_C);
	DUMP_REG(DC_DISP_SPI_INIT_SEQ_DATA_D);
	DUMP_REG(DC_DISP_DC_MCCIF_FIFOCTRL);
	DUMP_REG(DC_DISP_MCCIF_DISPLAY0A_HYST);
	DUMP_REG(DC_DISP_MCCIF_DISPLAY0B_HYST);
	DUMP_REG(DC_DISP_MCCIF_DISPLAY1A_HYST);
	DUMP_REG(DC_DISP_MCCIF_DISPLAY1B_HYST);
	DUMP_REG(DC_DISP_DAC_CRT_CTRL);
	DUMP_REG(DC_DISP_DISP_MISC_CONTROL);
	DUMP_REG(DC_DISP_SD_CONTROL);
	DUMP_REG(DC_DISP_SD_CSC_COEFF);
	DUMP_REG(DC_DISP_SD_LUT(0));
	DUMP_REG(DC_DISP_SD_LUT(1));
	DUMP_REG(DC_DISP_SD_LUT(2));
	DUMP_REG(DC_DISP_SD_LUT(3));
	DUMP_REG(DC_DISP_SD_LUT(4));
	DUMP_REG(DC_DISP_SD_LUT(5));
	DUMP_REG(DC_DISP_SD_LUT(6));
	DUMP_REG(DC_DISP_SD_LUT(7));
	DUMP_REG(DC_DISP_SD_LUT(8));
	DUMP_REG(DC_DISP_SD_FLICKER_CONTROL);
	DUMP_REG(DC_DISP_DC_PIXEL_COUNT);
	DUMP_REG(DC_DISP_SD_HISTOGRAM(0));
	DUMP_REG(DC_DISP_SD_HISTOGRAM(1));
	DUMP_REG(DC_DISP_SD_HISTOGRAM(2));
	DUMP_REG(DC_DISP_SD_HISTOGRAM(3));
	DUMP_REG(DC_DISP_SD_HISTOGRAM(4));
	DUMP_REG(DC_DISP_SD_HISTOGRAM(5));
	DUMP_REG(DC_DISP_SD_HISTOGRAM(6));
	DUMP_REG(DC_DISP_SD_HISTOGRAM(7));
	DUMP_REG(DC_DISP_SD_BL_TF(0));
	DUMP_REG(DC_DISP_SD_BL_TF(1));
	DUMP_REG(DC_DISP_SD_BL_TF(2));
	DUMP_REG(DC_DISP_SD_BL_TF(3));
	DUMP_REG(DC_DISP_SD_BL_CONTROL);
	DUMP_REG(DC_DISP_SD_HW_K_VALUES);
	DUMP_REG(DC_DISP_SD_MAN_K_VALUES);
	DUMP_REG(DC_DISP_CURSOR_START_ADDR_HI);
	DUMP_REG(DC_DISP_BLEND_CURSOR_CONTROL);

	win_header_save = tegra_dc_readl(dc, DC_CMD_DISPLAY_WINDOW_HEADER);
	for (i = 0; i < dc->soc->num_windows; i++) {
		switch (i) {
		case 0:
			seq_printf(s, "\nWindow A registers:\n");
			break;
		case 1:
			seq_printf(s, "\nWindow B registers:\n");
			break;
		case 2:
			seq_printf(s, "\nWindow C registers:\n");
			break;
		default:
			/* Currently we don't care about other windows */
			continue;
		}

		tegra_dc_writel(dc, WINDOW_A_SELECT << i,
				DC_CMD_DISPLAY_WINDOW_HEADER);

		DUMP_REG(DC_WIN_WIN_OPTIONS);
		DUMP_REG(DC_WIN_BYTE_SWAP);
		DUMP_REG(DC_WIN_BUFFER_CONTROL);
		DUMP_REG(DC_WIN_COLOR_DEPTH);
		DUMP_REG(DC_WIN_POSITION);
		DUMP_REG(DC_WIN_SIZE);
		DUMP_REG(DC_WIN_PRESCALED_SIZE);
		DUMP_REG(DC_WIN_H_INITIAL_DDA);
		DUMP_REG(DC_WIN_V_INITIAL_DDA);
		DUMP_REG(DC_WIN_DDA_INC);
		DUMP_REG(DC_WIN_LINE_STRIDE);
		DUMP_REG(DC_WIN_BUF_STRIDE);
		DUMP_REG(DC_WIN_UV_BUF_STRIDE);
		DUMP_REG(DC_WIN_BUFFER_ADDR_MODE);
		DUMP_REG(DC_WIN_DV_CONTROL);
		DUMP_REG(DC_WIN_BLEND_NOKEY);
		DUMP_REG(DC_WIN_BLEND_1WIN);
		DUMP_REG(DC_WIN_BLEND_2WIN_X);
		DUMP_REG(DC_WIN_BLEND_2WIN_Y);
		DUMP_REG(DC_WIN_BLEND_3WIN_XY);
		DUMP_REG(DC_WIN_HP_FETCH_CONTROL);
		DUMP_REG(DC_WINBUF_START_ADDR);
		DUMP_REG(DC_WINBUF_START_ADDR_NS);
		DUMP_REG(DC_WINBUF_START_ADDR_U);
		DUMP_REG(DC_WINBUF_START_ADDR_U_NS);
		DUMP_REG(DC_WINBUF_START_ADDR_V);
		DUMP_REG(DC_WINBUF_START_ADDR_V_NS);
		DUMP_REG(DC_WINBUF_ADDR_H_OFFSET);
		DUMP_REG(DC_WINBUF_ADDR_H_OFFSET_NS);
		DUMP_REG(DC_WINBUF_ADDR_V_OFFSET);
		DUMP_REG(DC_WINBUF_ADDR_V_OFFSET_NS);
		DUMP_REG(DC_WINBUF_UFLOW_STATUS);
		DUMP_REG(DC_WINBUF_SURFACE_KIND);
	}

	tegra_dc_writel(dc, win_header_save, DC_CMD_DISPLAY_WINDOW_HEADER);
	tegra_dc_writel(dc, state_access_save, DC_CMD_STATE_ACCESS);

#undef DUMP_REG

	spin_unlock_irqrestore(&dc->lock, flags);
	return 0;
}

static int tegra_dc_show_stats(struct seq_file *s, void *data)
{
	struct drm_info_node *node = s->private;
	struct tegra_dc *dc = node->info_ent->data;

	seq_printf(s, "frames: %lu\n", dc->stats.frames);
	seq_printf(s, "vblank: %lu\n", dc->stats.vblank);
	seq_printf(s, "underflow: %lu\n", dc->stats.underflow);
	seq_printf(s, "overflow: %lu\n", dc->stats.overflow);

	return 0;
}

static struct drm_info_list debugfs_files[] = {
	{ "regs", tegra_dc_show_regs, 0, NULL },
	{ "stats", tegra_dc_show_stats, 0, NULL },
};

static int tegra_dc_debugfs_init(struct tegra_dc *dc, struct drm_minor *minor)
{
	unsigned int i;
	char *name;
	int err;

	name = kasprintf(GFP_KERNEL, "dc.%d", dc->pipe);
	dc->debugfs = debugfs_create_dir(name, minor->debugfs_root);
	kfree(name);

	if (!dc->debugfs)
		return -ENOMEM;

	dc->debugfs_files = kmemdup(debugfs_files, sizeof(debugfs_files),
				    GFP_KERNEL);
	if (!dc->debugfs_files) {
		err = -ENOMEM;
		goto remove;
	}

	for (i = 0; i < ARRAY_SIZE(debugfs_files); i++)
		dc->debugfs_files[i].data = dc;

	err = drm_debugfs_create_files(dc->debugfs_files,
				       ARRAY_SIZE(debugfs_files),
				       dc->debugfs, minor);
	if (err < 0)
		goto free;

	dc->minor = minor;

	return 0;

free:
	kfree(dc->debugfs_files);
	dc->debugfs_files = NULL;
remove:
	debugfs_remove(dc->debugfs);
	dc->debugfs = NULL;

	return err;
}

static int tegra_dc_debugfs_exit(struct tegra_dc *dc)
{
	drm_debugfs_remove_files(dc->debugfs_files, ARRAY_SIZE(debugfs_files),
				 dc->minor);
	dc->minor = NULL;

	kfree(dc->debugfs_files);
	dc->debugfs_files = NULL;

	debugfs_remove(dc->debugfs);
	dc->debugfs = NULL;

	return 0;
}

static void tegra_dc_clk_dvfs_init(struct tegra_dc *dc)
{
	unsigned long *freqs;
	int num_freqs;
	unsigned long rate;
	int ret;

	/*
	 * Set the dc->clk's rate to the maximum rate of the dvfs
	 * table if its default rate is larger than it.
	 */
	ret = tegra_dvfs_get_freqs(dc->clk, &freqs, &num_freqs);
	if (!ret && num_freqs > 0) {
		rate = clk_get_rate(dc->clk);
		if (rate > freqs[num_freqs - 1])
			clk_set_rate(dc->clk, freqs[num_freqs - 1]);
	}
}

static int tegra_dc_init(struct host1x_client *client)
{
	struct drm_device *drm = dev_get_drvdata(client->parent);
	struct tegra_dc *dc = host1x_client_to_dc(client);
	struct tegra_drm *tegra = drm->dev_private;
	struct drm_plane *primary = NULL;
	struct drm_plane *cursor = NULL;
	int err;

	if (tegra->domain) {
		err = iommu_attach_device(tegra->domain, dc->dev);
		if (err < 0) {
			dev_err(dc->dev, "failed to attach to domain: %d\n",
				err);
			return err;
		}

		dc->domain = tegra->domain;
	}

	primary = tegra_dc_primary_plane_create(drm, dc);
	if (IS_ERR(primary)) {
		err = PTR_ERR(primary);
		goto cleanup;
	}

	if (dc->soc->supports_cursor) {
		cursor = tegra_dc_cursor_plane_create(drm, dc);
		if (IS_ERR(cursor)) {
			err = PTR_ERR(cursor);
			goto cleanup;
		}
	}

	err = drm_crtc_init_with_planes(drm, &dc->base, primary, cursor,
					&tegra_crtc_funcs, NULL);
	if (err < 0)
		goto cleanup;

	drm_mode_crtc_set_gamma_size(&dc->base, 256);
	drm_crtc_helper_add(&dc->base, &tegra_crtc_helper_funcs);

	/*
	 * Keep track of the minimum pitch alignment across all display
	 * controllers.
	 */
	if (dc->soc->pitch_align > tegra->pitch_align)
		tegra->pitch_align = dc->soc->pitch_align;

	err = tegra_dc_rgb_init(drm, dc);
	if (err < 0 && err != -ENODEV) {
		dev_err(dc->dev, "failed to initialize RGB output: %d\n", err);
		goto cleanup;
	}

	err = tegra_dc_add_planes(drm, dc);
	if (err < 0)
		goto cleanup;

	if (IS_ENABLED(CONFIG_DEBUG_FS)) {
		err = tegra_dc_debugfs_init(dc, drm->primary);
		if (err < 0)
			dev_err(dc->dev, "debugfs setup failed: %d\n", err);
	}

	err = devm_request_irq(dc->dev, dc->irq, tegra_dc_irq, 0,
			       dev_name(dc->dev), dc);
	if (err < 0) {
		dev_err(dc->dev, "failed to request IRQ#%u: %d\n", dc->irq,
			err);
		goto cleanup;
	}

	/*
	 * Make sure that the default rate of dc->clk is in
	 * the range of the dvfs table. Otherwise, it will cause
	 * fault if any function of setting rate is directly or
	 * indirectly called.
	 */
	tegra_dc_clk_dvfs_init(dc);

	tegra_dc_clk_disable(dc);

	/* Matches the ungate in probe() */
	if (dc->soc->has_powergate)
		tegra_pmc_powergate(dc->powergate);

	return 0;

cleanup:
	if (cursor)
		drm_plane_cleanup(cursor);

	if (primary)
		drm_plane_cleanup(primary);

	if (tegra->domain) {
		iommu_detach_device(tegra->domain, dc->dev);
		dc->domain = NULL;
	}

	tegra_dc_clk_dvfs_init(dc);
	tegra_dc_clk_disable(dc);

	/* Matches the ungate in probe() */
	if (dc->soc->has_powergate)
		tegra_pmc_powergate(dc->powergate);

	return err;
}

static int tegra_dc_exit(struct host1x_client *client)
{
	struct tegra_dc *dc = host1x_client_to_dc(client);
	int err;

	if (dc->soc->has_powergate)
		tegra_pmc_unpowergate(dc->powergate);

	tegra_dc_clk_enable(dc);

	devm_free_irq(dc->dev, dc->irq, dc);

	if (IS_ENABLED(CONFIG_DEBUG_FS)) {
		err = tegra_dc_debugfs_exit(dc);
		if (err < 0)
			dev_err(dc->dev, "debugfs cleanup failed: %d\n", err);
	}

	err = tegra_dc_rgb_exit(dc);
	if (err) {
		dev_err(dc->dev, "failed to shutdown RGB output: %d\n", err);
		return err;
	}

	if (dc->domain) {
		iommu_detach_device(dc->domain, dc->dev);
		dc->domain = NULL;
	}

	tegra_dc_clk_disable(dc);

	if (dc->soc->has_powergate)
		tegra_pmc_powergate(dc->powergate);

	return 0;
}

static const struct host1x_client_ops dc_client_ops = {
	.init = tegra_dc_init,
	.exit = tegra_dc_exit,
};

static const u32 tegra20_primary_plane_formats[] = {
	DRM_FORMAT_ABGR8888,
	DRM_FORMAT_XBGR8888,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_BGR565,
	DRM_FORMAT_RGB565,
};

static const u32 tegra20_overlay_plane_formats[] = {
	DRM_FORMAT_ABGR8888,
	DRM_FORMAT_XBGR8888,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_BGR565,
	DRM_FORMAT_RGB565,
	DRM_FORMAT_UYVY,
	DRM_FORMAT_YUYV,
	DRM_FORMAT_YUV420,
	DRM_FORMAT_YUV422,
};

static const u32 tegra114_overlay_plane_formats[] = {
	DRM_FORMAT_ABGR8888,
	DRM_FORMAT_XBGR8888,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_BGR565,
	DRM_FORMAT_RGB565,
	DRM_FORMAT_UYVY,
	DRM_FORMAT_YUYV,
	DRM_FORMAT_YUV420,
	DRM_FORMAT_YUV422,
	DRM_FORMAT_NV12,
	DRM_FORMAT_NV21,
};

static const u32 tegra124_plane_formats[] = {
	DRM_FORMAT_ABGR8888,
	DRM_FORMAT_XBGR8888,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_BGR565,
	DRM_FORMAT_RGB565,
	DRM_FORMAT_UYVY,
	DRM_FORMAT_YUYV,
	DRM_FORMAT_YUV420,
	DRM_FORMAT_YUV422,
	DRM_FORMAT_NV12,
	DRM_FORMAT_NV21,
};

static const struct tegra_dc_scale_limit tegra114_h_scale_down_limits[] = {
	{ 0, 4 },
};

static const struct tegra_dc_scale_limit tegra114_v_scale_down_limits[] = {
	{ 2, 4 },
	{ 4, 2 },
	{ 0, 2 },
};

static const struct tegra_dc_window_soc_info tegra20_dc_window_soc_info[] = {
	[0] = {
		.supports_v_filter = false,
		.supports_h_filter = false,
		.supports_planar_rotation = false
	},
	[1] = {
		.supports_v_filter = true,
		.supports_h_filter = true,
		.supports_planar_rotation = false
	},
	[2] = {
		.supports_v_filter = false,
		.supports_h_filter = true,
		.supports_planar_rotation = false
	},
};

static const struct tegra_dc_soc_info tegra20_dc_soc_info = {
	.supports_border_color = true,
	.supports_interlacing = false,
	.supports_cursor = false,
	.supports_block_linear = false,
	.pitch_align = 8,
	.has_powergate = false,
	.supports_v2_blend = false,
	.supports_scan_column = false,
	.windows = tegra20_dc_window_soc_info,
	.num_windows = ARRAY_SIZE(tegra20_dc_window_soc_info),
	.num_primary_plane_formats = ARRAY_SIZE(tegra20_primary_plane_formats),
	.primary_plane_formats = tegra20_primary_plane_formats,
	.num_overlay_plane_formats = ARRAY_SIZE(tegra20_overlay_plane_formats),
	.overlay_plane_formats = tegra20_overlay_plane_formats,
};

static const struct tegra_dc_window_soc_info tegra30_dc_window_soc_info[] = {
	[0] = {
		.supports_v_filter = false,
		.supports_h_filter = false,
		.supports_planar_rotation = false
	},
	[1] = {
		.supports_v_filter = true,
		.supports_h_filter = true,
		.supports_planar_rotation = false
	},
	[2] = {
		.supports_v_filter = false,
		.supports_h_filter = true,
		.supports_planar_rotation = false
	},
};

static const struct tegra_dc_soc_info tegra30_dc_soc_info = {
	.supports_border_color = true,
	.supports_interlacing = false,
	.supports_cursor = false,
	.supports_block_linear = false,
	.pitch_align = 8,
	.has_powergate = false,
	.supports_v2_blend = false,
	.supports_scan_column = false,
	.windows = tegra30_dc_window_soc_info,
	.num_windows = ARRAY_SIZE(tegra30_dc_window_soc_info),
	.num_primary_plane_formats = ARRAY_SIZE(tegra20_primary_plane_formats),
	.primary_plane_formats = tegra20_primary_plane_formats,
	.num_overlay_plane_formats = ARRAY_SIZE(tegra20_overlay_plane_formats),
	.overlay_plane_formats = tegra20_overlay_plane_formats,
};

static const struct tegra_dc_window_soc_info tegra114_dc_window_soc_info[] = {
	[0] = {
		.supports_v_filter = true,
		.supports_h_filter = true,
		.supports_planar_rotation = false
	},
	[1] = {
		.supports_v_filter = true,
		.supports_h_filter = true,
		.supports_planar_rotation = false
	},
	[2] = {
		.supports_v_filter = true,
		.supports_h_filter = true,
		.supports_planar_rotation = false
	},
};

static const struct tegra_dc_soc_info tegra114_dc_soc_info = {
	.supports_border_color = true,
	.supports_interlacing = false,
	.supports_cursor = false,
	.supports_block_linear = false,
	.pitch_align = 64,
	.has_powergate = true,
	.supports_v2_blend = false,
	.supports_scan_column = true,
	.windows = tegra114_dc_window_soc_info,
	.num_windows = ARRAY_SIZE(tegra114_dc_window_soc_info),
	.num_primary_plane_formats = ARRAY_SIZE(tegra20_primary_plane_formats),
	.primary_plane_formats = tegra20_primary_plane_formats,
	.num_overlay_plane_formats = ARRAY_SIZE(tegra114_overlay_plane_formats),
	.overlay_plane_formats = tegra114_overlay_plane_formats,
	.num_h_scale_down_limits = ARRAY_SIZE(tegra114_h_scale_down_limits),
	.h_scale_down_limits = tegra114_h_scale_down_limits,
	.num_v_scale_down_limits = ARRAY_SIZE(tegra114_v_scale_down_limits),
	.v_scale_down_limits = tegra114_v_scale_down_limits,
};

static const struct tegra_dc_window_soc_info tegra124_dc_window_soc_info[] = {
	[0] = {
		.supports_v_filter = true,
		.supports_h_filter = true,
		.supports_planar_rotation = true
	},
	[1] = {
		.supports_v_filter = true,
		.supports_h_filter = true,
		.supports_planar_rotation = false
	},
	[2] = {
		.supports_v_filter = true,
		.supports_h_filter = true,
		.supports_planar_rotation = false
	},
};

static const struct tegra_dc_soc_info tegra124_dc_soc_info = {
	.supports_border_color = false,
	.supports_interlacing = true,
	.supports_cursor = true,
	.supports_block_linear = true,
	.pitch_align = 64,
	.has_powergate = true,
	.supports_v2_blend = true,
	.supports_scan_column = true,
	.windows = tegra124_dc_window_soc_info,
	.num_windows = ARRAY_SIZE(tegra124_dc_window_soc_info),
	.num_primary_plane_formats = ARRAY_SIZE(tegra124_plane_formats),
	.primary_plane_formats = tegra124_plane_formats,
	.num_overlay_plane_formats = ARRAY_SIZE(tegra124_plane_formats),
	.overlay_plane_formats = tegra124_plane_formats,
	.num_h_scale_down_limits = ARRAY_SIZE(tegra114_h_scale_down_limits),
	.h_scale_down_limits = tegra114_h_scale_down_limits,
	.num_v_scale_down_limits = ARRAY_SIZE(tegra114_v_scale_down_limits),
	.v_scale_down_limits = tegra114_v_scale_down_limits,
};

static const struct tegra_dc_window_soc_info tegra210_dc_window_soc_info[] = {
	[0] = {
		.supports_v_filter = true,
		.supports_h_filter = true,
		.supports_planar_rotation = true
	},
	[1] = {
		.supports_v_filter = true,
		.supports_h_filter = true,
		.supports_planar_rotation = false
	},
	[2] = {
		.supports_v_filter = true,
		.supports_h_filter = true,
		.supports_planar_rotation = false
	},
};

static const struct tegra_dc_soc_info tegra210_dc_soc_info = {
	.supports_border_color = false,
	.supports_interlacing = true,
	.supports_cursor = true,
	.supports_block_linear = true,
	.pitch_align = 64,
	.has_powergate = true,
	.supports_v2_blend = true,
	.supports_scan_column = true,
	.windows = tegra210_dc_window_soc_info,
	.num_windows = ARRAY_SIZE(tegra210_dc_window_soc_info),
	.num_primary_plane_formats = ARRAY_SIZE(tegra124_plane_formats),
	.primary_plane_formats = tegra124_plane_formats,
	.num_overlay_plane_formats = ARRAY_SIZE(tegra124_plane_formats),
	.overlay_plane_formats = tegra124_plane_formats,
	.num_h_scale_down_limits = ARRAY_SIZE(tegra114_h_scale_down_limits),
	.h_scale_down_limits = tegra114_h_scale_down_limits,
	.num_v_scale_down_limits = ARRAY_SIZE(tegra114_v_scale_down_limits),
	.v_scale_down_limits = tegra114_v_scale_down_limits,
};

static const struct of_device_id tegra_dc_of_match[] = {
	{
		.compatible = "nvidia,tegra210-dc",
		.data = &tegra210_dc_soc_info,
	}, {
		.compatible = "nvidia,tegra124-dc",
		.data = &tegra124_dc_soc_info,
	}, {
		.compatible = "nvidia,tegra114-dc",
		.data = &tegra114_dc_soc_info,
	}, {
		.compatible = "nvidia,tegra30-dc",
		.data = &tegra30_dc_soc_info,
	}, {
		.compatible = "nvidia,tegra20-dc",
		.data = &tegra20_dc_soc_info,
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(of, tegra_dc_of_match);

static int tegra_dc_parse_dt(struct tegra_dc *dc)
{
	struct device_node *np;
	struct of_phandle_args args;
	struct platform_device *pdev;
	u32 value = 0;
	int i, err, count;
	int ret = 0;

	err = of_property_read_u32(dc->dev->of_node, "nvidia,head", &value);
	if (err < 0) {
		dev_err(dc->dev, "missing \"nvidia,head\" property\n");

		/*
		 * If the nvidia,head property isn't present, try to find the
		 * correct head number by looking up the position of this
		 * display controller's node within the device tree. Assuming
		 * that the nodes are ordered properly in the DTS file and
		 * that the translation into a flattened device tree blob
		 * preserves that ordering this will actually yield the right
		 * head number.
		 *
		 * If those assumptions don't hold, this will still work for
		 * cases where only a single display controller is used.
		 */
		for_each_matching_node(np, tegra_dc_of_match) {
			if (np == dc->dev->of_node)
				break;

			value++;
		}
	}

	dc->pipe = value;

	np = of_parse_phandle(dc->dev->of_node, "mc-clients", 0);
	if (!np) {
		dev_err(dc->dev, "get mc device node failed\n");
		return -ENODEV;
	}

	pdev = of_find_device_by_node(np);
	if (!pdev) {
		dev_err(dc->dev, "get mc device failed\n");
		ret = -ENODEV;
		goto out;
	}

	dc->mc = platform_get_drvdata(pdev);
	if (!dc->mc) {
		dev_err(dc->dev, "get mc device drvdata failed\n");
		ret = -EPROBE_DEFER;
		goto out;
	}

	count = of_count_phandle_with_args(dc->dev->of_node, "mc-clients",
					   "#mc-client-cells");
	if (!count) {
		dev_err(dc->dev, "get mc-clients count failed\n");
		ret = -ENODEV;
		goto out;
	}

	WARN_ON(dc->soc->num_windows < count);

	dc->mc_win_clients = devm_kcalloc(dc->dev, dc->soc->num_windows,
					sizeof(int), GFP_KERNEL);
	if (!dc->mc_win_clients) {
		dev_err(dc->dev, "allocate dc mc win clients failed.\n");
		ret = -ENOMEM;
		goto out;
	}

	for (i = 0; i < dc->soc->num_windows; i++) {
		err = of_parse_phandle_with_args(dc->dev->of_node, "mc-clients",
						 "#mc-client-cells", i, &args);
		if (err) {
			dev_err(dc->dev, "parse mc-clients failed: %d\n", err);
			of_node_put(args.np);
			ret = -ENODEV;
			goto out;
		}

		dc->mc_win_clients[i] = args.args[0];

		of_node_put(args.np);
	}

out:
	of_node_put(np);
	return ret;
}

static int tegra_dc_probe(struct platform_device *pdev)
{
	unsigned long flags = HOST1X_SYNCPT_CLIENT_MANAGED;
	const struct of_device_id *id;
	struct resource *regs;
	struct tegra_dc *dc;
	int err;

	dc = devm_kzalloc(&pdev->dev, sizeof(*dc), GFP_KERNEL);
	if (!dc)
		return -ENOMEM;

	id = of_match_node(tegra_dc_of_match, pdev->dev.of_node);
	if (!id)
		return -ENODEV;

	spin_lock_init(&dc->lock);
	INIT_LIST_HEAD(&dc->list);
	dc->dev = &pdev->dev;
	dc->soc = id->data;
	dc->dpms = DRM_MODE_DPMS_OFF;

	err = tegra_dc_parse_dt(dc);
	if (err < 0)
		return err;

	dc->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(dc->clk)) {
		dev_err(&pdev->dev, "failed to get clock\n");
		return PTR_ERR(dc->clk);
	}

	dc->rst = devm_reset_control_get(&pdev->dev, "dc");
	if (IS_ERR(dc->rst)) {
		dev_err(&pdev->dev, "failed to get reset\n");
		return PTR_ERR(dc->rst);
	}

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dc->regs = devm_ioremap_resource(&pdev->dev, regs);
	if (IS_ERR(dc->regs))
		return PTR_ERR(dc->regs);

	if (dc->soc->has_powergate) {
		if (dc->pipe == 0)
			dc->powergate = TEGRA_POWERGATE_DIS;
		else
			dc->powergate = TEGRA_POWERGATE_DISB;

		dc->slcg_notifier.notifier_call = tegra_dc_slcg_handler;
		tegra_slcg_register_notifier(dc->powergate,
			&dc->slcg_notifier);

		err = tegra_pmc_unpowergate(dc->powergate);
		if (err < 0) {
			dev_err(&pdev->dev, "failed to power partition: %d\n",
				err);
			return err;
		}

		err = clk_prepare_enable(dc->clk);
		if (err < 0) {
			tegra_pmc_powergate(dc->powergate);
			dev_err(&pdev->dev, "failed to enable clock: %d\n",
				err);
			return err;
		}
	} else {
		err = clk_prepare_enable(dc->clk);
		if (err < 0) {
			dev_err(&pdev->dev, "failed to enable clock: %d\n",
				err);
			return err;
		}

		err = reset_control_deassert(dc->rst);
		if (err < 0) {
			dev_err(&pdev->dev, "failed to deassert reset: %d\n",
				err);
			return err;
		}
	}

	dc->emc_clk = devm_clk_get(&pdev->dev, "emc");
	if (IS_ERR(dc->emc_clk))
		dc->emc_clk = NULL;
	clk_prepare_enable(dc->emc_clk);

	dc->emc_la_clk = devm_clk_get(&pdev->dev, "emc_la");
	if (IS_ERR(dc->emc_la_clk))
		dc->emc_la_clk = NULL;
	clk_prepare_enable(dc->emc_la_clk);

	dc->irq = platform_get_irq(pdev, 0);
	if (dc->irq < 0) {
		dev_err(&pdev->dev, "failed to get IRQ\n");
		return -ENXIO;
	}

	INIT_LIST_HEAD(&dc->client.list);
	dc->client.ops = &dc_client_ops;
	dc->client.dev = &pdev->dev;

	err = tegra_dc_rgb_probe(dc);
	if (err < 0 && err != -ENODEV) {
		dev_err(&pdev->dev, "failed to probe RGB output: %d\n", err);
		return err;
	}

	err = host1x_client_register(&dc->client);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to register host1x client: %d\n",
			err);
		return err;
	}

	dc->syncpt = host1x_syncpt_request(&pdev->dev, flags);
	if (!dc->syncpt)
		dev_warn(&pdev->dev, "failed to allocate syncpoint\n");

	platform_set_drvdata(pdev, dc);

	return 0;
}

static int tegra_dc_remove(struct platform_device *pdev)
{
	struct tegra_dc *dc = platform_get_drvdata(pdev);
	int err;

	host1x_syncpt_free(dc->syncpt);

	err = host1x_client_unregister(&dc->client);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to unregister host1x client: %d\n",
			err);
		return err;
	}

	err = tegra_dc_rgb_remove(dc);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to remove RGB output: %d\n", err);
		return err;
	}

	reset_control_assert(dc->rst);

	tegra_slcg_unregister_notifier(dc->powergate,
		&dc->slcg_notifier);

	tegra_dc_clk_disable(dc);

	if (dc->soc->has_powergate)
		tegra_pmc_powergate(dc->powergate);

	return 0;
}

struct platform_driver tegra_dc_driver = {
	.driver = {
		.name = "tegra-dc",
		.owner = THIS_MODULE,
		.of_match_table = tegra_dc_of_match,
	},
	.probe = tegra_dc_probe,
	.remove = tegra_dc_remove,
};
