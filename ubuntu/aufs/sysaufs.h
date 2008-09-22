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
 *
 * $Id: sysaufs.h,v 1.11 2008/09/15 03:14:55 sfjro Exp $
 */

#ifndef __SYSAUFS_H__
#define __SYSAUFS_H__

#ifdef __KERNEL__

#include <linux/fs.h>
#include <linux/sysfs.h>
#include "module.h"
#include "super.h"

#define SysaufsSb_PREFIX	"si_"	/* followed by %p */

struct au_sbi_attr {
	struct attribute attr;
	int (*show)(struct seq_file *seq, struct super_block *sb);
};

/* ---------------------------------------------------------------------- */

/* sysaufs.c */
extern unsigned long au_si_mask;
extern struct kset *au_kset;
extern struct attribute *au_sbi_attrs[];
int sysaufs_si_init(struct au_sbinfo *sbinfo);
int __init sysaufs_init(void);
void sysaufs_fin(void);

/* ---------------------------------------------------------------------- */

struct au_branch;
#ifdef CONFIG_SYSFS
/* sysfs.c */
extern struct attribute_group *au_attr_group;
extern struct kobj_type *au_ktype;

int sysaufs_sbi_xino(struct seq_file *seq, struct super_block *sb);
#ifdef CONFIG_AUFS_EXPORT
int sysaufs_sbi_xigen(struct seq_file *seq, struct super_block *sb);
#endif
int sysaufs_sbi_mntpnt1(struct seq_file *seq, struct super_block *sb);
ssize_t sysaufs_sbi_show(struct kobject *kobj, struct attribute *attr,
			 char *buf);

void sysaufs_br_init(struct au_branch *br);
void sysaufs_brs_add(struct super_block *sb, aufs_bindex_t bindex);
void sysaufs_brs_del(struct super_block *sb, aufs_bindex_t bindex);
#else
#define au_attr_group	NULL
#define au_ktype	NULL

static inline
int sysaufs_sbi_xino(struct seq_file *seq, struct super_block *sb)
{
	return 0;
}

#ifdef CONFIG_AUFS_EXPORT
static inline
int sysaufs_sbi_xigen(struct seq_file *seq, struct super_block *sb)
{
	return 0;
}
#endif

static inline
int sysaufs_sbi_mntpnt1(struct seq_file *seq, struct super_block *sb)
{
	return 0;
}

static inline
ssize_t sysaufs_sbi_show(struct kobject *kobj, struct attribute *attr,
			 char *buf)
{
	return 0;
}

static inline void sysaufs_br_init(struct au_branch *br)
{
	/* empty */
}

static inline void sysaufs_brs_add(struct super_block *sb, aufs_bindex_t bindex)
{
	/* nothing */
}

static inline void sysaufs_brs_del(struct super_block *sb, aufs_bindex_t bindex)
{
	/* nothing */
}
#endif /* CONFIG_SYSFS */

#endif /* __KERNEL__ */
#endif /* __SYSAUFS_H__ */
