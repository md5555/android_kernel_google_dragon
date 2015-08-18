/*
 * Copyright 2013 Red Hat Inc.
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
 * THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Authors: Ben Skeggs <bskeggs@redhat.com>
 */

#include <nvif/os.h>
#include <nvif/class.h>
#include <nvif/ioctl.h>

#include "nouveau_sysfs.h"

MODULE_PARM_DESC(pstate, "enable sysfs pstate file, which will be moved in the future");
int nouveau_pstate = 1;
module_param_named(pstate, nouveau_pstate, int, 0400);

static inline struct drm_device *
drm_device(struct device *d)
{
	return dev_get_drvdata(d);
}

#define snappendf(p,r,f,a...) do {                                             \
	snprintf(p, r, f, ##a);                                                \
	r -= strlen(p);                                                        \
	p += strlen(p);                                                        \
} while(0)

static ssize_t
nouveau_sysfs_pstate_get(struct device *d, struct device_attribute *a, char *b)
{
	struct drm_device *drm_dev = drm_device(d);
	struct device *dev = drm_dev->dev;
	struct nouveau_sysfs *sysfs = nouveau_sysfs(drm_dev);
	struct nvif_control_pstate_info_v0 info = {};
	size_t cnt = PAGE_SIZE;
	char *buf = b;
	int ret, i;

	ret = pm_runtime_get_sync(dev);
	if (ret < 0)
		return ret;

	ret = nvif_mthd(&sysfs->ctrl, NVIF_CONTROL_PSTATE_INFO,
			&info, sizeof(info));
	if (ret)
		goto out;

	for (i = 0; i < info.count + 1; i++) {
		const s32 state = i < info.count ? i :
			NVIF_CONTROL_PSTATE_ATTR_V0_STATE_CURRENT;
		struct nvif_control_pstate_attr_v0 attr = {
			.state = state,
			.index = 0,
		};

		ret = nvif_mthd(&sysfs->ctrl, NVIF_CONTROL_PSTATE_ATTR,
				&attr, sizeof(attr));
		if (ret)
			goto out;

		if (i < info.count)
			snappendf(buf, cnt, "%02x:", attr.state);
		else
			snappendf(buf, cnt, "%s:", info.pwrsrc == 0 ? "DC" :
						   info.pwrsrc == 1 ? "AC" :
						   "--");

		attr.index = 0;
		do {
			attr.state = state;
			ret = nvif_mthd(&sysfs->ctrl,
					NVIF_CONTROL_PSTATE_ATTR,
					&attr, sizeof(attr));
			if (ret)
				goto out;

			snappendf(buf, cnt, " %s %d", attr.name, attr.min);
			if (attr.min != attr.max)
				snappendf(buf, cnt, "-%d", attr.max);
			snappendf(buf, cnt, " %s", attr.unit);
		} while (attr.index);

		if (state >= 0) {
			if (info.ustate.ac.min == state)
				snappendf(buf, cnt, " a");
			if (info.ustate.ac.max == state)
				snappendf(buf, cnt, " A");
			if (info.ustate.dc.min == state)
				snappendf(buf, cnt, " d");
			if (info.ustate.dc.max == state)
				snappendf(buf, cnt, " D");
			if (info.pstate == state)
				snappendf(buf, cnt, " *");
		} else {
			if (info.ustate.ac.min < -1)
				snappendf(buf, cnt, " a");
			if (info.ustate.ac.max < -1)
				snappendf(buf, cnt, " A");
			if (info.ustate.dc.min < -1)
				snappendf(buf, cnt, " d");
			if (info.ustate.dc.max < -1)
				snappendf(buf, cnt, " D");
		}

		snappendf(buf, cnt, "\n");
	}

	ret = strlen(b);

out:
	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);

	return ret;
}

static ssize_t
nouveau_sysfs_pstate_set(struct device *d, struct device_attribute *a,
			 const char *buf, size_t count)
{
	struct drm_device *drm_dev = drm_device(d);
	struct device *dev = drm_dev->dev;
	struct nouveau_sysfs *sysfs = nouveau_sysfs(drm_dev);
	struct nvif_control_pstate_user_v0 args = { .pwrsrc = -EINVAL };
	long ret;
	char *tmp;

	if ((tmp = strchr(buf, '\n')))
		*tmp = '\0';

	if (!strncasecmp(buf, "dc:", 3)) {
		args.pwrsrc = 0;
		buf += 3;
	} else
	if (!strncasecmp(buf, "ac:", 3)) {
		args.pwrsrc = 1;
		buf += 3;
	}

	if (!strcasecmp(buf, "none")) {
		args.ustate.min = NVIF_CONTROL_PSTATE_USER_V0_STATE_UNKNOWN;
		args.ustate.max = NVIF_CONTROL_PSTATE_USER_V0_STATE_UNKNOWN;
	} else
	if (!strcasecmp(buf, "auto")) {
		args.ustate.min = NVIF_CONTROL_PSTATE_USER_V0_STATE_PERFMON;
		args.ustate.max = NVIF_CONTROL_PSTATE_USER_V0_STATE_PERFMON;
	} else {
		if ((tmp = strchr(buf, ','))) {
			long min, max;
			ret = kstrtol(&tmp[1], 16, &max);
			if (ret)
				return ret;

			*tmp = '\0';
			ret = kstrtol(buf, 16, &min);
			if (ret)
				return ret;

			if (min > max)
				return -EINVAL;

			args.ustate.min = min;
			args.ustate.max = max;
		} else {
			long value;
			ret = kstrtol(buf, 16, &value);
			if (ret)
				return ret;
			args.ustate.min = args.ustate.max = value;
		}
	}

	/* make sure the Nouveau is not runtime suspended */
	ret = pm_runtime_get_sync(dev);
	if (ret < 0)
		return ret;


	ret = nvif_mthd(&sysfs->ctrl, NVIF_CONTROL_PSTATE_USER,
			&args, sizeof(args));
	if (ret < 0)
		goto out;

	ret = count;

out:
	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);

	return ret;
}

static DEVICE_ATTR(pstate, S_IRUGO | S_IWUSR,
		   nouveau_sysfs_pstate_get, nouveau_sysfs_pstate_set);

void
nouveau_sysfs_fini(struct drm_device *dev)
{
	struct nouveau_sysfs *sysfs = nouveau_sysfs(dev);
	struct nouveau_drm *drm = nouveau_drm(dev);
	struct nvif_device *device = &drm->device;

	if (sysfs && sysfs->ctrl.priv) {
		device_remove_file(nv_device_base(nvxx_device(device)), &dev_attr_pstate);
		nvif_object_fini(&sysfs->ctrl);
	}

	drm->sysfs = NULL;
	kfree(sysfs);
}

int
nouveau_sysfs_init(struct drm_device *dev)
{
	struct nouveau_drm *drm = nouveau_drm(dev);
	struct nvif_device *device = &drm->device;
	struct nouveau_sysfs *sysfs;
	int ret;

	if (!nouveau_pstate)
		return 0;

	sysfs = drm->sysfs = kzalloc(sizeof(*sysfs), GFP_KERNEL);
	if (!sysfs)
		return -ENOMEM;

	ret = nvif_object_init(nvif_object(device), NULL, NVDRM_CONTROL,
			       NVIF_IOCTL_NEW_V0_CONTROL, NULL, 0,
			      &sysfs->ctrl);
	if (ret == 0)
		device_create_file(nv_device_base(nvxx_device(device)), &dev_attr_pstate);

	return 0;
}
