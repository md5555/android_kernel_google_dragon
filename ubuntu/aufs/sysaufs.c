/*
 * Copyright (C) 2005-2008 Junjiro Okajima
 *
 * This program, aufs is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

/*
 * sysfs interface and lifetime management
 * they are necessary regardless sysfs is disabled.
 *
 * $Id: sysaufs.c,v 1.10 2008/09/15 03:14:55 sfjro Exp $
 */

#include <linux/fs.h>
#include <linux/module.h>
#include <linux/random.h>
#include <linux/sysfs.h>
#include "aufs.h"

/* ---------------------------------------------------------------------- */

unsigned long au_si_mask;

/* ---------------------------------------------------------------------- */

struct kset *au_kset;

#define AuSbiAttr(_name) { \
	.attr   = { .name = __stringify(_name), .mode = 0444 },	\
	.show   = sysaufs_sbi_##_name,				\
}

static struct au_sbi_attr au_sbi_attr_xino = AuSbiAttr(xino);
#ifdef CONFIG_AUFS_EXPORT
static struct au_sbi_attr au_sbi_attr_xigen = AuSbiAttr(xigen);
#endif
struct attribute *au_sbi_attrs[] = {
	&au_sbi_attr_xino.attr,
#ifdef CONFIG_AUFS_EXPORT
	&au_sbi_attr_xigen.attr,
#endif
	NULL,
};

static struct sysfs_ops au_sbi_ops = {
	.show   = sysaufs_sbi_show
};

static struct kobj_type au_sbi_ktype = {
	.release	= au_si_free,
	.sysfs_ops	= &au_sbi_ops,
	.default_attrs	= au_sbi_attrs
};

/* ---------------------------------------------------------------------- */

int sysaufs_si_init(struct au_sbinfo *sbinfo)
{
	int err;

	sbinfo->si_kobj.kset = au_kset;
	/* some people doesn't like to show a pointer in kernel */
	err = kobject_init_and_add(&sbinfo->si_kobj, &au_sbi_ktype,
				   NULL/*&au_kset->kobj*/,
				   SysaufsSb_PREFIX "%lx",
				   au_si_mask ^ (unsigned long)sbinfo);
	AuTraceErr(err);
	return err;
}


/* ---------------------------------------------------------------------- */

void sysaufs_fin(void)
{
	sysfs_remove_group(&au_kset->kobj, au_attr_group);
	kset_unregister(au_kset);
}

int __init sysaufs_init(void)
{
	int err;

	get_random_bytes(&au_si_mask, sizeof(au_si_mask));

	au_kset = kset_create_and_add(AUFS_NAME, NULL, fs_kobj);
	err = PTR_ERR(au_kset);
	if (IS_ERR(au_kset))
		goto out;
	err = sysfs_create_group(&au_kset->kobj, au_attr_group);
	if (unlikely(err))
		kset_unregister(au_kset);

 out:
	AuTraceErr(err);
	return err;
}
