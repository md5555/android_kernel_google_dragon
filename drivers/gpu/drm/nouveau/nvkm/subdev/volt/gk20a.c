/*
 * Copyright (c) 2014, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <core/device.h>
#include <linux/thermal.h>

#include <subdev/volt.h>
#ifdef __KERNEL__
#include <nouveau_platform.h>
#endif

#include "gk20a.h"

static const struct cvb_coef gk20a_cvb_coef[] = {
	/* MHz,        c0,     c1,   c2,    c3,     c4,   c5 */
	/*  72 */ { 1209886, -36468,  515,   417, -13123,  203},
	/* 108 */ { 1130804, -27659,  296,   298, -10834,  221},
	/* 180 */ { 1162871, -27110,  247,   238, -10681,  268},
	/* 252 */ { 1220458, -28654,  247,   179, -10376,  298},
	/* 324 */ { 1280953, -30204,  247,   119,  -9766,  304},
	/* 396 */ { 1344547, -31777,  247,   119,  -8545,  292},
	/* 468 */ { 1420168, -34227,  269,    60,  -7172,  256},
	/* 540 */ { 1490757, -35955,  274,    60,  -5188,  197},
	/* 612 */ { 1599112, -42583,  398,     0,  -1831,  119},
	/* 648 */ { 1366986, -16459, -274,     0,  -3204,   72},
	/* 684 */ { 1391884, -17078, -274,   -60,  -1526,   30},
	/* 708 */ { 1415522, -17497, -274,   -60,   -458,    0},
	/* 756 */ { 1464061, -18331, -274,  -119,   1831,  -72},
	/* 804 */ { 1524225, -20064, -254,  -119,   4272, -155},
	/* 852 */ { 1608418, -21643, -269,     0,    763,  -48},
};

/*
 * The last table entry just mean the temperature is larger than 70C,
 * will not use as thermal trip
 */
static const int gk20a_thermal_table[] = {
	-10, 10, 30, 50, 70, 71
};

/**
 * cvb_mv = ((c2 * speedo / s_scale + c1) * speedo / s_scale + c0)
 */
static inline int
gk20a_volt_get_cvb_voltage(int speedo, int s_scale, const struct cvb_coef *coef)
{
	int mv;

	mv = DIV_ROUND_CLOSEST(coef->c2 * speedo, s_scale);
	mv = DIV_ROUND_CLOSEST((mv + coef->c1) * speedo, s_scale) + coef->c0;
	return mv;
}

/**
 * cvb_t_mv =
 * ((c2 * speedo / s_scale + c1) * speedo / s_scale + c0) +
 * ((c3 * speedo / s_scale + c4 + c5 * T / t_scale) * T / t_scale)
 */
static inline int
gk20a_volt_get_cvb_t_voltage(int speedo, int temp, int s_scale, int t_scale,
			     const struct cvb_coef *coef)
{
	int cvb_mv, mv;

	cvb_mv = gk20a_volt_get_cvb_voltage(speedo, s_scale, coef);

	mv = DIV_ROUND_CLOSEST(coef->c3 * speedo, s_scale) + coef->c4 +
		DIV_ROUND_CLOSEST(coef->c5 * temp, t_scale);
	mv = DIV_ROUND_CLOSEST(mv * temp, t_scale) + cvb_mv;
	return mv;
}

int
gk20a_volt_calc_voltage(struct gk20a_volt_priv *priv,
		const struct cvb_coef *coef, int speedo, int therm_idx)
{
	int mv, lo, hi, lo_temp, hi_temp;

	if (!priv->thermal_table) {
		nv_error(priv, "Thermal table not found\n");
		return -EINVAL;
	}


	if (therm_idx < priv->therm_nr) {
		lo_temp = priv->thermal_table[therm_idx];
		hi_temp = priv->thermal_table[therm_idx + 1];
	} else {
		nv_error(priv, "Exceeded thermal table\n");
		return -EINVAL;
	}

	lo = gk20a_volt_get_cvb_t_voltage(speedo, lo_temp, 100, 10, coef);
	hi = gk20a_volt_get_cvb_t_voltage(speedo, hi_temp, 100, 10, coef);
	mv = max(lo, hi);
	mv = DIV_ROUND_UP(mv, 1000);

	return mv * 1000;
}

int
gk20a_volt_round_voltage(struct gk20a_volt_priv *priv, int uv)
{
	int i, n, ret;

	if (uv <= 0)
		return -EINVAL;

	n = regulator_count_voltages(priv->vdd);
	for (i = 0; i < n; i++) {
		ret = regulator_list_voltage(priv->vdd, i);
		if (ret <= 0)
			continue;
		if (ret >= uv)
			return ret;
	}

	return -EINVAL;
}

int
gk20a_volt_vid_get(struct nvkm_volt *volt)
{
	struct gk20a_volt_priv *priv = (void *)volt;
	int i, uv;

	uv = regulator_get_voltage(priv->vdd);

	for (i = 0; i < volt->vid_nr; i++)
		if (volt->vid[i].uv >= uv)
			return i;

	return -EINVAL;
}

int
gk20a_volt_vid_set(struct nvkm_volt *volt, u8 vid)
{
	struct gk20a_volt_priv *priv = (void *)volt;

	nv_debug(volt, "set voltage as %duv\n", volt->vid[vid].uv);
	return regulator_set_voltage(priv->vdd, volt->vid[vid].uv, 1200000);
}

int
gk20a_volt_set_id(struct nvkm_volt *volt, u8 id, int condition)
{
	struct gk20a_volt_priv *priv = (void *)volt;
	int prev_uv = regulator_get_voltage(priv->vdd);
	int target_uv = volt->vid[id].uv;
	int ret;

	nv_debug(volt, "prev=%d, target=%d, condition=%d\n",
			prev_uv, target_uv, condition);
	if (!condition ||
		(condition < 0 && target_uv < prev_uv) ||
		(condition > 0 && target_uv > prev_uv)) {
		ret = gk20a_volt_vid_set(volt, volt->vid[id].vid);
	} else {
		ret = 0;
	}

	return ret;
}

static int
gk20a_volt_ctor(struct nvkm_object *parent, struct nvkm_object *engine,
		struct nvkm_oclass *oclass, void *data, u32 size,
		struct nvkm_object **pobject)
{
	struct gk20a_volt_priv *priv;
	struct nvkm_volt *volt;
	struct nouveau_platform_device *plat;
	int i, j, ret, uv, speedo_val;

	ret = nvkm_volt_create(parent, engine, oclass, &priv);
	*pobject = nv_object(priv);
	if (ret)
		return ret;

	volt = &priv->base;

	plat = nv_device_to_platform(nv_device(parent));

	uv = regulator_get_voltage(plat->gpu->vdd);
	nv_info(priv, "The default voltage is %duV\n", uv);

	priv->vdd = plat->gpu->vdd;
	priv->base.vid_get = gk20a_volt_vid_get;
	priv->base.vid_set = gk20a_volt_vid_set;
	priv->base.set_id = gk20a_volt_set_id;
	priv->thermal_table = gk20a_thermal_table;

	volt->vid_nr = ARRAY_SIZE(gk20a_cvb_coef);
	nv_debug(priv, "%s - vid_nr = %d\n", __func__, volt->vid_nr);

	priv->therm_nr = ARRAY_SIZE(gk20a_thermal_table) - 1;
	if (priv->therm_nr > MAX_THERMAL_LIMITS) {
		nv_error(priv, "The thermal table is too large\n");
		return -EINVAL;
	}

	speedo_val = plat->gpu_speedo_value;

	priv->therm_idx = 0;

	for (j = 0; j < priv->therm_nr; j++) {
		for (i = 0; i < volt->vid_nr; i++) {
			struct nvkm_voltage *table = &priv->scale_table[j][i];

			ret = gk20a_volt_calc_voltage(priv, &gk20a_cvb_coef[i],
						      speedo_val, j);
			ret = gk20a_volt_round_voltage(priv, ret);
			if (ret < 0)
				return ret;

			table->uv = ret;
			table->vid = i;

			nv_debug(priv, "%2d: therm_idx=%d, vid=%d, uv=%d\n",
					i, j, table->vid, table->uv);
		}
	}

	memcpy(volt->vid, priv->scale_table[priv->therm_idx],
		sizeof(volt->vid));

	return 0;
}

struct nvkm_oclass
gk20a_volt_oclass = {
	.handle = NV_SUBDEV(VOLT, 0xea),
	.ofuncs = &(struct nvkm_ofuncs) {
		.ctor = gk20a_volt_ctor,
		.dtor = _nvkm_volt_dtor,
		.init = _nvkm_volt_init,
		.fini = _nvkm_volt_fini,
	},
};

#ifdef CONFIG_THERMAL
/* Must be called while holding therm_lock */
static void gk20a_volt_dvfs_update_voltage(struct gk20a_volt_priv *priv)
{
	struct nvkm_volt *volt = &priv->base;
	int id, pre_uv, new_uv, ret;

	WARN_ON(!mutex_is_locked(&volt->therm_lock));

	if (volt->vid_get) {
		id = volt->vid_get(volt);
		if (id < 0)
			return;
		pre_uv = volt->vid[id].uv;
	} else {
		return;
	}

	memcpy(volt->vid, priv->scale_table[priv->therm_idx],
		sizeof(volt->vid));
	new_uv = volt->vid[id].uv;

	if ((pre_uv != new_uv) && (volt->vid_set)) {
		ret = volt->vid_set(volt, id);
		nv_debug(volt, "update voltage from %duv to %duv: %d\n",
				pre_uv, new_uv, ret);
	}
}

static int
gk20a_volt_dvfs_get_vts_cdev_max_state(struct thermal_cooling_device *cdev,
				 unsigned long *max_state)
{
	struct gk20a_volt_priv *priv = (struct gk20a_volt_priv *)cdev->devdata;
	*max_state = priv->therm_nr - 1;

	return 0;
}

static int
gk20a_volt_dvfs_get_vts_cdev_cur_state(struct thermal_cooling_device *cdev,
				  unsigned long *cur_state)
{
	struct gk20a_volt_priv *priv = (struct gk20a_volt_priv *)cdev->devdata;
	*cur_state = priv->therm_idx;

	return 0;
}

static int
gk20a_volt_dvfs_set_vts_cdev_state(struct thermal_cooling_device *cdev,
			      unsigned long cur_state)
{
	struct gk20a_volt_priv *priv = (struct gk20a_volt_priv *)cdev->devdata;
	struct nvkm_volt *volt = &priv->base;

	mutex_lock(&volt->therm_lock);

	if (priv->therm_idx == cur_state)
		goto end;

	priv->therm_idx = cur_state;
	gk20a_volt_dvfs_update_voltage(priv);

end:
	mutex_unlock(&volt->therm_lock);
	return 0;
}

static struct thermal_cooling_device_ops gk20a_volt_dvfs_cooling_ops = {
	.get_max_state = gk20a_volt_dvfs_get_vts_cdev_max_state,
	.get_cur_state = gk20a_volt_dvfs_get_vts_cdev_cur_state,
	.set_cur_state = gk20a_volt_dvfs_set_vts_cdev_state,
};

int
gk20a_volt_dvfs_cdev_register(struct gk20a_volt_priv *priv)
{
	struct nvkm_volt *volt = &priv->base;
	struct thermal_cooling_device *tcd;
	struct platform_device *pdev;
	struct device_node *np, *child;

	if (priv->therm_nr == -1)
		return -EINVAL;

	pdev = nv_device(volt)->platformdev;
	if (IS_ERR_OR_NULL(pdev))
		return -EINVAL;
	np = pdev->dev.of_node;
	child = of_get_child_by_name(np, "gpu-scaling-cdev");
	if (!child) {
		nv_error(volt, " No support for gpu_scaling cooling device\n");
		return -EINVAL;
	}
	tcd = thermal_of_cooling_device_register(child,
						"gpu_scaling",
						priv,
						&gk20a_volt_dvfs_cooling_ops);
	of_node_put(child);
	if (IS_ERR_OR_NULL(tcd)) {
		nv_error(volt,
			 "Failed register gpu_scaling cooling device\n");
		return PTR_ERR(tcd);
	}
	priv->cdev = tcd;

	return 0;
}

void gk20a_volt_dvfs_cdev_unregister(struct gk20a_volt_priv *priv)
{
	if (priv->cdev)
		thermal_cooling_device_unregister(priv->cdev);
}
#endif
