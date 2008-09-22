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
 * sysfs interface
 *
 * $Id: sysfs.c,v 1.13 2008/09/15 03:14:55 sfjro Exp $
 */

#include <linux/fs.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/sysfs.h>
#include "aufs.h"


#ifdef CONFIG_AUFS_LOCAL
static ssize_t config_show(struct kobject *kobj, struct kobj_attribute *attr,
			   char *buf)
{
#define conf_bool(name)	"CONFIG_AUFS_" #name "=y\n"
	static const char opt[] =
#ifdef CONFIG_AUFS
		"CONFIG_AUFS=y\n"
#else
		"CONFIG_AUFS=m\n"
#endif
#ifdef CONFIG_AUFS_BRANCH_MAX_127
		conf_bool(BRANCH_MAX_127)
#elif defined(CONFIG_AUFS_BRANCH_MAX_511)
		conf_bool(BRANCH_MAX_511)
#elif defined(CONFIG_AUFS_BRANCH_MAX_1023)
		conf_bool(BRANCH_MAX_1023)
#elif defined(CONFIG_AUFS_BRANCH_MAX_32767)
		conf_bool(BRANCH_MAX_32767)
#endif
#ifdef CONFIG_AUFS_HINOTIFY
		conf_bool(HINOTIFY)
#endif
#ifdef CONFIG_AUFS_EXPORT
		conf_bool(EXPORT)
#endif
#ifdef CONFIG_AUFS_ROBR
		conf_bool(ROBR)
#endif
#ifdef CONFIG_AUFS_SHWH
		conf_bool(SHWH)
#endif
#ifdef CONFIG_AUFS_DLGT
		conf_bool(DLGT)
#endif
#ifdef CONFIG_AUFS_HIN_OR_DLGT
		conf_bool(HIN_OR_DLGT)
#endif
#ifdef CONFIG_AUFS_RR_SQUASHFS
		conf_bool(RR_SQUASHFS)
#endif
#ifdef CONFIG_AUFS_SEC_PERM_PATCH
		conf_bool(SEC_PERM_PATCH)
#endif
#ifdef CONFIG_AUFS_SPLICE_PATCH
		conf_bool(SPLICE_PATCH)
#endif
#ifdef CONFIG_AUFS_PUT_FILP_PATCH
		conf_bool(PUT_FILP_PATCH)
#endif
#ifdef CONFIG_AUFS_LHASH_PATCH
		conf_bool(LHASH_PATCH)
#endif
#ifdef CONFIG_AUFS_BR_NFS
		conf_bool(BR_NFS)
#endif
#ifdef CONFIG_AUFS_BR_XFS
		conf_bool(BR_XFS)
#endif
#ifdef CONFIG_AUFS_FSYNC_SUPER_PATCH
		conf_bool(FSYNC_SUPER_PATCH)
#endif
#ifdef CONFIG_AUFS_DENY_WRITE_ACCESS_PATCH
		conf_bool(DENY_WRITE_ACCESS_PATCH)
#endif
#ifdef CONFIG_AUFS_KSIZE_PATCH
		conf_bool(KSIZE_PATCH)
#endif
#ifdef CONFIG_AUFS_WORKAROUND_FUSE
		conf_bool(WORKAROUND_FUSE)
#endif
#ifdef CONFIG_AUFS_HIN_OR_FUSE
		conf_bool(HIN_OR_FUSE)
#endif
#ifdef CONFIG_AUFS_STAT
		conf_bool(STAT)
#endif
#ifdef CONFIG_AUFS_DEBUG
		conf_bool(DEBUG)
#endif
#ifdef CONFIG_AUFS_MAGIC_SYSRQ
		conf_bool(MAGIC_SYSRQ)
#endif
#ifdef CONFIG_AUFS_COMPAT
		conf_bool(COMPAT)
#endif
#ifdef CONFIG_AUFS_UNIONFS22_PATCH
		conf_bool(UNIONFS22_PATCH)
#endif
#ifdef CONFIG_AUFS_UNIONFS23_PATCH
		conf_bool(UNIONFS23_PATCH)
#endif
		;
#undef conf_bool

	char *p = buf;
	const char *end = buf + PAGE_SIZE;

	p += snprintf(p, end - p, "%s", opt);
#ifdef DbgUdbaRace
	if (p < end)
		p += snprintf(p, end - p, "DbgUdbaRace=%d\n", DbgUdbaRace);
#endif
	if (p < end)
		return p - buf;
	else
		return -EFBIG;
}

static struct kobj_attribute au_config_attr = __ATTR_RO(config);
#endif

#ifdef CONFIG_AUFS_STAT
static ssize_t stat_show(struct kobject *kobj, struct kobj_attribute *attr,
			 char *buf)
{
	char *p = buf;
	const char *end = buf + PAGE_SIZE;
	int i;

	p += snprintf(p, end - p, "wkq max_busy:");
	for (i = 0; p < end && i < aufs_nwkq; i++)
		p += snprintf(p, end - p, " %u", au_wkq[i].max_busy);
	if (p < end)
		p += snprintf(p, end - p, ", %u(generic)\n",
			      au_wkq[aufs_nwkq].max_busy);

	if (p < end)
		return p - buf;
	else
		return -EFBIG;
}

static struct kobj_attribute au_stat_attr = __ATTR_RO(stat);
#endif

#ifdef CONFIG_AUFS_DEBUG
static ssize_t debug_show(struct kobject *kobj, struct kobj_attribute *attr,
			  char *buf)
{
	return sprintf(buf, "%d\n", au_debug_test());
}

static ssize_t debug_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t sz)
{
	LKTRTrace("%.*s\n", (unsigned int)sz, buf);

	if (unlikely(!sz || (*buf != '0' && *buf != '1')))
		return -EOPNOTSUPP;

	if (*buf == '0')
		au_debug_off();
	else if (*buf == '1')
		au_debug_on();
	return sz;
}

static struct kobj_attribute au_debug_attr = __ATTR(debug, S_IRUGO | S_IWUSR,
						    debug_show, debug_store);
#endif

static struct attribute *au_attr[] = {
#ifdef CONFIG_AUFS_LOCAL
	&au_config_attr.attr,
#endif
#ifdef CONFIG_AUFS_STAT
	&au_stat_attr.attr,
#endif
#ifdef CONFIG_AUFS_DEBUG
	&au_debug_attr.attr,
#endif
	NULL,	/* need to NULL terminate the list of attributes */
};

static struct attribute_group au_attr_group_body = {
	.attrs = au_attr
};

struct attribute_group *au_attr_group = &au_attr_group_body;

/* ---------------------------------------------------------------------- */

/*
 * they are copied from linux/lib/kobject.c,
 * and will be exported in the future.
 */
static ssize_t au_attr_show(struct kobject *kobj, struct attribute *attr,
			    char *buf)
{
	struct kobj_attribute *kattr;
	ssize_t ret = -EIO;

	kattr = container_of(attr, struct kobj_attribute, attr);
	if (kattr->show)
		ret = kattr->show(kobj, kattr, buf);
	return ret;
}

#ifdef CONFIG_AUFS_DEBUG
static ssize_t au_attr_store(struct kobject *kobj, struct attribute *attr,
			     const char *buf, size_t count)
{
	struct kobj_attribute *kattr;
	ssize_t ret = -EIO;

	kattr = container_of(attr, struct kobj_attribute, attr);
	if (kattr->store)
		ret = kattr->store(kobj, kattr, buf, count);
	return ret;
}
#endif

static struct sysfs_ops sysaufs_ops = {
	.show   = au_attr_show,
#ifdef CONFIG_AUFS_DEBUG
	.store  = au_attr_store
#endif
};

static struct kobj_type au_ktype_body = {
	.sysfs_ops = &sysaufs_ops
};
struct kobj_type *au_ktype = &au_ktype_body;

/* ---------------------------------------------------------------------- */

static int sysaufs_sbi_xi(struct seq_file *seq, struct file *xf, int dlgt,
			  int print_path)
{
	int err;
	struct kstat st;
	struct path path;

	err = vfsub_getattr(xf->f_vfsmnt, xf->f_dentry, &st, dlgt);
	if (!err) {
		seq_printf(seq, "%llux%lu %lld",
			   st.blocks, st.blksize, (long long)st.size);
		if (unlikely(print_path)) {
			path.dentry = xf->f_dentry;
			path.mnt = xf->f_vfsmnt;
			seq_putc(seq, ' ');
			seq_path(seq, &path, au_esc_chars);
		}
		seq_putc(seq, '\n');
	} else
		seq_printf(seq, "err %d\n", err);

	AuTraceErr(err);
	return err;
}

int sysaufs_sbi_xino(struct seq_file *seq, struct super_block *sb)
{
	int err;
	unsigned int mnt_flags;
	aufs_bindex_t bend, bindex;
	unsigned char dlgt, xinodir;
	struct kstat st;
	struct path path;
	struct au_sbinfo *sbinfo;
	struct file *xf;

	AuTraceEnter();

	sbinfo = au_sbi(sb);
	mnt_flags = au_mntflags(sb);
	xinodir = !!au_opt_test(mnt_flags, XINODIR);
	if (unlikely(!au_opt_test_xino(mnt_flags))) {
#ifdef CONFIG_AUFS_DEBUG
		AuDebugOn(sbinfo->si_xib);
		bend = au_sbend(sb);
		for (bindex = 0; bindex <= bend; bindex++)
			AuDebugOn(au_sbr(sb, bindex)->br_xino.xi_file);
#endif
		err = 0;
		goto out; /* success */
	}

	dlgt = !!au_test_dlgt(mnt_flags);
	err = sysaufs_sbi_xi(seq, sbinfo->si_xib, dlgt, xinodir);

	bend = au_sbend(sb);
	for (bindex = 0; !err && bindex <= bend; bindex++) {
		xf = au_sbr(sb, bindex)->br_xino.xi_file;
		if (!xf)
			continue;
		seq_printf(seq, "%d: ", bindex);
		err = vfsub_getattr(xf->f_vfsmnt, xf->f_dentry, &st, dlgt);
		if (!err) {
			seq_printf(seq, "%ld, %llux%lu %lld",
				   (long)file_count(xf), st.blocks, st.blksize,
				   (long long)st.size);
			if (unlikely(xinodir)) {
				path.dentry = xf->f_dentry;
				path.mnt = xf->f_vfsmnt;
				seq_putc(seq, ' ');
				seq_path(seq, &path, au_esc_chars);
			}
			seq_putc(seq, '\n');
		} else
			seq_printf(seq, "err %d\n", err);
	}

 out:
	AuTraceErr(err);
	return err;
}

#ifdef CONFIG_AUFS_EXPORT
int sysaufs_sbi_xigen(struct seq_file *seq, struct super_block *sb)
{
	int err;
	unsigned int mnt_flags;
	struct au_sbinfo *sbinfo;

	AuTraceEnter();

	err = 0;
	sbinfo = au_sbi(sb);
	mnt_flags = au_mntflags(sb);
	if (au_opt_test_xino(mnt_flags))
		err = sysaufs_sbi_xi(seq, sbinfo->si_xigen,
				     !!au_opt_test(mnt_flags, DLGT),
				     !!au_opt_test(mnt_flags, XINODIR));

	AuTraceErr(err);
	return err;
}
#endif

/*
 * the lifetime of branch is independent from the entry under sysfs.
 * sysfs handles the lifetime of the entry, and never call ->show() after it is
 * unlinked.
 */
#define SysaufsBr_PREFIX "br"
static int sysaufs_sbi_br(struct seq_file *seq, struct super_block *sb,
			  aufs_bindex_t bindex)
{
	int err;
	struct dentry *root;
	struct au_branch *br;
	struct path path;

	LKTRTrace("b%d\n", bindex);

	err = -ENOENT;
	if (unlikely(au_sbend(sb) < bindex))
		goto out;

	err = 0;
	root = sb->s_root;
	di_read_lock_parent(root, !AuLock_IR);
	br = au_sbr(sb, bindex);
	path.mnt = br->br_mnt;
	path.dentry = au_h_dptr(root, bindex);
	seq_path(seq, &path, au_esc_chars);
	di_read_unlock(root, !AuLock_IR);
	seq_printf(seq, "=%s\n", au_optstr_br_perm(br->br_perm));

 out:
	AuTraceErr(err);
	return err;
}

/* ---------------------------------------------------------------------- */

static struct seq_file *au_seq(char *p, ssize_t len)
{
	struct seq_file *seq;

	seq = kzalloc(sizeof(*seq), GFP_NOFS);
	if (seq) {
		/* todo: necessary? */
		/* mutex_init(&seq.lock); */
		seq->buf = p;
		seq->size = len;
		return seq; /* success */
	}

	seq = ERR_PTR(-ENOMEM);
	AuTraceErrPtr(seq);
	return seq;
}

/* todo: file size may exceed PAGE_SIZE */
ssize_t sysaufs_sbi_show(struct kobject *kobj, struct attribute *attr,
			 char *buf)
{
	ssize_t err;
	struct au_sbinfo *sbinfo;
	struct super_block *sb;
	struct seq_file *seq;
	char *name;
	struct attribute **cattr;

	LKTRTrace("%s/%s\n", kobject_name(kobj), attr->name);

	sbinfo = container_of(kobj, struct au_sbinfo, si_kobj);
	sb = sbinfo->si_sb;
	si_noflush_read_lock(sb);

	seq = au_seq(buf, PAGE_SIZE);
	err = PTR_ERR(seq);
	if (IS_ERR(seq))
		goto out;

	name = (void *)attr->name;
	cattr = au_sbi_attrs;
	while (*cattr) {
		if (!strcmp(name, (*cattr)->name)) {
			err = container_of(*cattr, struct au_sbi_attr, attr)
				->show(seq, sb);
			goto out_seq;
		}
		cattr++;
	}

	if (!strncmp(name, SysaufsBr_PREFIX, sizeof(SysaufsBr_PREFIX) - 1)) {
		name += sizeof(SysaufsBr_PREFIX) - 1;
		err = sysaufs_sbi_br(seq, sb, simple_strtol(name, NULL, 10));
		goto out_seq;
	}
	BUG();

 out_seq:
	if (!err) {
		err = seq->count;
		/* sysfs limit */
		if (unlikely(err == PAGE_SIZE))
			err = -EFBIG;
	}
	kfree(seq);
 out:
	si_read_unlock(sb);
	AuTraceErr(err);
	return err;
}

/* ---------------------------------------------------------------------- */

void sysaufs_br_init(struct au_branch *br)
{
	br->br_attr.name = br->br_name;
	br->br_attr.mode = S_IRUGO;
	br->br_attr.owner = THIS_MODULE;
}

void sysaufs_brs_del(struct super_block *sb, aufs_bindex_t bindex)
{
	struct au_sbinfo *sbinfo;
	aufs_bindex_t bend;

	LKTRTrace("b%d\n", bindex);

	if (!sysaufs_brs)
		return;

	sbinfo = au_sbi(sb);
	bend = au_sbend(sb);
	for (; bindex <= bend; bindex++)
		sysfs_remove_file(&sbinfo->si_kobj,
				  &au_sbr(sb, bindex)->br_attr);
}

void sysaufs_brs_add(struct super_block *sb, aufs_bindex_t bindex)
{
	int err;
	struct kobject *kobj;
	aufs_bindex_t bend;
	struct au_branch *br;

	LKTRTrace("b%d\n", bindex);

	if (!sysaufs_brs)
		return;

	kobj = &au_sbi(sb)->si_kobj;
	bend = au_sbend(sb);
	for (; bindex <= bend; bindex++) {
		br = au_sbr(sb, bindex);
		snprintf(br->br_name, sizeof(br->br_name),
			 SysaufsBr_PREFIX "%d", bindex);
		err = sysfs_create_file(kobj, &br->br_attr);
		if (unlikely(err))
			AuWarn("failed %s under sysfs(%d)\n", br->br_name, err);
	}
}
