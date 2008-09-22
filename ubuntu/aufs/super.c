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
 * mount and super_block operations
 *
 * $Id: super.c,v 1.16 2008/09/15 03:14:49 sfjro Exp $
 */

#include <linux/module.h>
#include <linux/buffer_head.h>
#include <linux/seq_file.h>
#include <linux/smp_lock.h>
#include <linux/statfs.h>

#include "aufs.h"

/*
 * super_operations
 */
static struct inode *aufs_alloc_inode(struct super_block *sb)
{
	struct aufs_icntnr *c;

	AuTraceEnter();

	c = au_cache_alloc_icntnr();
	if (c) {
		inode_init_once(&c->vfs_inode);
		c->vfs_inode.i_version = 1; /* sigen(sb); */
		c->iinfo.ii_hinode = NULL;
		return &c->vfs_inode;
	}
	return NULL;
}

static void aufs_destroy_inode(struct inode *inode)
{
	int err;

	LKTRTrace("i%lu\n", inode->i_ino);

	if (!inode->i_nlink) {
		struct super_block *sb = inode->i_sb;
		int locked;

		/* in nowait task, sbi is write-locked */
		/* todo: test kernel thread */
		locked = si_noflush_read_trylock(sb);
		err = au_xigen_inc(inode);
		if (unlikely(err))
			AuWarn1("failed resetting i_generation, %d\n", err);
		if (locked)
			si_read_unlock(sb);
	}

	au_iinfo_fin(inode);
	au_cache_free_icntnr(container_of(inode, struct aufs_icntnr,
					  vfs_inode));
}

struct inode *au_iget_locked(struct super_block *sb, ino_t ino)
{
	struct inode *inode;
	int err;

	LKTRTrace("i%lu\n", (unsigned long)ino);

	inode = iget_locked(sb, ino);
	if (unlikely(!inode)) {
		inode = ERR_PTR(-ENOMEM);
		goto out;
	}
	AuDebugOn(IS_ERR(inode));
	if (unlikely(!(inode->i_state & I_NEW)))
		goto out;

	err = au_xigen_new(inode);
	if (!err)
		err = au_iinfo_init(inode);
	if (!err)
		inode->i_version++;
	else {
		iget_failed(inode);
		inode = ERR_PTR(err);
	}

 out:
	/* never return NULL */
	AuDebugOn(!inode);
	AuTraceErrPtr(inode);
	return inode;
}

static int au_show_brs(struct seq_file *seq, struct super_block *sb)
{
	int err;
	aufs_bindex_t bindex, bend;
	struct dentry *root;
	struct path path;

	AuTraceEnter();

	err = 0;
	root = sb->s_root;
	bend = au_sbend(sb);
	for (bindex = 0; !err && bindex <= bend; bindex++) {
		path.mnt = au_sbr_mnt(sb, bindex);
		path.dentry = au_h_dptr(root, bindex);
		err = seq_path(seq, &path, au_esc_chars);
		if (err > 0)
			err = seq_printf
				(seq, "=%s",
				 au_optstr_br_perm(au_sbr_perm(sb, bindex)));
		if (!err && bindex != bend)
			err = seq_putc(seq, ':');
	}

	AuTraceErr(err);
	return err;
}

static void au_show_wbr_create(struct seq_file *m, int v,
			       struct au_sbinfo *sbinfo)
{
	const char *pat;

	AuDebugOn(v == AuWbrCreate_Def);

	seq_printf(m, ",create=");
	pat = au_optstr_wbr_create(v);
	switch (v) {
	case AuWbrCreate_TDP:
	case AuWbrCreate_RR:
	case AuWbrCreate_MFS:
	case AuWbrCreate_PMFS:
		seq_printf(m, pat);
		break;
	case AuWbrCreate_MFSV:
		seq_printf(m, /*pat*/"mfs:%lu",
			   sbinfo->si_wbr_mfs.mfs_expire / HZ);
		break;
	case AuWbrCreate_PMFSV:
		seq_printf(m, /*pat*/"pmfs:%lu",
			   sbinfo->si_wbr_mfs.mfs_expire / HZ);
		break;
	case AuWbrCreate_MFSRR:
		seq_printf(m, /*pat*/"mfsrr:%llu",
			   sbinfo->si_wbr_mfs.mfsrr_watermark);
		break;
	case AuWbrCreate_MFSRRV:
		seq_printf(m, /*pat*/"mfsrr:%llu:%lu",
			   sbinfo->si_wbr_mfs.mfsrr_watermark,
			   sbinfo->si_wbr_mfs.mfs_expire / HZ);
		break;
	}
}

/* seq_file will re-call me in case of too long string */
static int aufs_show_options(struct seq_file *m, struct vfsmount *mnt)
{
	int err, n;
	struct super_block *sb;
	struct au_sbinfo *sbinfo;
	struct dentry *root;
	struct file *xino;
	unsigned int mnt_flags, v;
	struct path path;

	AuTraceEnter();

	sb = mnt->mnt_sb;
	root = sb->s_root;
	if (!sysaufs_brs)
		aufs_read_lock(root, !AuLock_IR);
	else
		si_noflush_read_lock(sb);
	sbinfo = au_sbi(sb);
	seq_printf(m, ",si=%lx", au_si_mask ^ (unsigned long)sbinfo);
	mnt_flags = au_mntflags(sb);
	if (au_opt_test(mnt_flags, XINO)) {
		seq_puts(m, ",xino=");
		xino = sbinfo->si_xib;
		path.mnt = xino->f_vfsmnt;
		path.dentry = xino->f_dentry;
		err = seq_path(m, &path, au_esc_chars);
		if (unlikely(err <= 0))
			goto out;
		err = 0;
#define Deleted "\\040(deleted)"
		m->count -= sizeof(Deleted) - 1;
		AuDebugOn(memcmp(m->buf + m->count, Deleted,
				 sizeof(Deleted) - 1));
#undef Deleted
#ifdef CONFIG_AUFS_EXPORT /* reserved for future use */
	} else if (au_opt_test(mnt_flags, XINODIR)) {
		seq_puts(m, ",xinodir=");
		seq_path(m, &sbinfo->si_xinodir, au_esc_chars);
#endif
	} else
		seq_puts(m, ",noxino");

#define AuBool(name, str) do { \
	v = au_opt_test(mnt_flags, name); \
	if (v != au_opt_test(AuOpt_Def, name)) \
		seq_printf(m, ",%s" #str, v ? "" : "no"); \
} while (0)

#define AuStr(name, str) do { \
	v = mnt_flags & AuOptMask_##name; \
	if (v != (AuOpt_Def & AuOptMask_##name)) \
		seq_printf(m, "," #str "=%s", au_optstr_##str(v)); \
} while (0)

#ifdef CONFIG_AUFS_COMPAT
#define AuStr_BrOpt	"dirs="
#else
#define AuStr_BrOpt	"br:"
#endif

	AuBool(TRUNC_XINO, trunc_xino);
	AuBool(DIRPERM1, dirperm1);
	AuBool(SHWH, shwh);
	AuBool(PLINK, plink);
	AuStr(UDBA, udba);

	v = sbinfo->si_wbr_create;
	if (v != AuWbrCreate_Def)
		au_show_wbr_create(m, v, sbinfo);

	v = sbinfo->si_wbr_copyup;
	if (v != AuWbrCopyup_Def)
		seq_printf(m, ",cpup=%s", au_optstr_wbr_copyup(v));

	v = au_opt_test(mnt_flags, ALWAYS_DIROPQ);
	if (v != au_opt_test(AuOpt_Def, ALWAYS_DIROPQ))
		seq_printf(m, ",diropq=%c", v ? 'a' : 'w');
	AuBool(REFROF, refrof);
	AuBool(DLGT, dlgt);
	AuBool(WARN_PERM, warn_perm);
	AuBool(VERBOSE, verbose);

	n = sbinfo->si_dirwh;
	if (n != AUFS_DIRWH_DEF)
		seq_printf(m, ",dirwh=%d", n);
	n = sbinfo->si_rdcache / HZ;
	if (n != AUFS_RDCACHE_DEF)
		seq_printf(m, ",rdcache=%d", n);

	AuStr(COO, coo);

 out:
	if (!sysaufs_brs) {
		seq_puts(m, "," AuStr_BrOpt);
		au_show_brs(m, sb);
		aufs_read_unlock(root, !AuLock_IR);
	} else
		si_read_unlock(sb);
	return 0;

#undef AuBool
#undef AuStr
#undef AuStr_BrOpt
}

/* todo: in case of round-robin policy, return the sum of all rw branches? */
static int aufs_statfs(struct dentry *dentry, struct kstatfs *buf)
{
	int err;

	AuTraceEnter();

	aufs_read_lock(dentry->d_sb->s_root, 0);
	err = vfsub_statfs(au_h_dptr(dentry->d_sb->s_root, 0), buf,
			   !!au_test_dlgt(au_mntflags(dentry->d_sb)));
	aufs_read_unlock(dentry->d_sb->s_root, 0);
	if (!err) {
		buf->f_type = AUFS_SUPER_MAGIC;
		buf->f_namelen -= AUFS_WH_PFX_LEN;
		memset(&buf->f_fsid, 0, sizeof(buf->f_fsid));
	}
	/* buf->f_bsize = buf->f_blocks = buf->f_bfree = buf->f_bavail = -1; */

	AuTraceErr(err);
	return err;
}

static void au_fsync_br(struct super_block *sb)
{
#ifdef CONFIG_AUFS_FSYNC_SUPER_PATCH
	aufs_bindex_t bend, bindex;
	int brperm;
	struct super_block *h_sb;

	AuTraceEnter();

	si_write_lock(sb);
	bend = au_sbend(sb);
	for (bindex = 0; bindex < bend; bindex++) {
		brperm = au_sbr_perm(sb, bindex);
		if (brperm == AuBrPerm_RR || brperm == AuBrPerm_RRWH)
			continue;
		h_sb = au_sbr_sb(sb, bindex);
		if (bdev_read_only(h_sb->s_bdev))
			continue;

		lockdep_off();
		down_write(&h_sb->s_umount);
		shrink_dcache_sb(h_sb);
		fsync_super(h_sb);
		up_write(&h_sb->s_umount);
		lockdep_on();
	}
	si_write_unlock(sb);
#endif
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 26)
/* this IS NOT for super_operations */
static void aufs_umount_begin(struct super_block *arg)
#define AuUmountBeginSb(arg)	(arg)
#else
/* this IS for super_operations */
static void aufs_umount_begin(struct vfsmount *arg, int flags)
#define AuUmountBeginSb(arg)	(arg)->mnt_sb
#endif
{
	struct super_block *sb = AuUmountBeginSb(arg);
	struct au_sbinfo *sbinfo;

	AuTraceEnter();
	/* dont trust BKL */
	AuDebugOn(!kernel_locked());

	sbinfo = au_sbi(sb);
	if (unlikely(!sbinfo))
		return;

	au_fsync_br(sb);

	si_write_lock(sb);
	if (au_opt_test(au_mntflags(sb), PLINK))
		au_plink_put(sb);
	au_mnt_reset(sbinfo);
#if 0 /* reserved for future use */
	if (sbinfo->si_wbr_create_ops->fin)
		sbinfo->si_wbr_create_ops->fin(sb);
#endif
	si_write_unlock(sb);
}

/* final actions when unmounting a file system */
static void aufs_put_super(struct super_block *sb)
{
	struct au_sbinfo *sbinfo;

	AuTraceEnter();

	sbinfo = au_sbi(sb);
	if (unlikely(!sbinfo))
		return;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 26)
	aufs_umount_begin(sb);
#endif
	kobject_put(&sbinfo->si_kobj);
}

/* ---------------------------------------------------------------------- */

/*
 * refresh dentry and inode at remount time.
 */
static int do_refresh(struct dentry *dentry, mode_t type,
		      unsigned int dir_flags)
{
	int err;
	struct dentry *parent;
	struct inode *inode;

	LKTRTrace("%.*s, 0%o\n", AuDLNPair(dentry), type);
	inode = dentry->d_inode;
	AuDebugOn(!inode);

	di_write_lock_child(dentry);
	parent = dget_parent(dentry);
	di_read_lock_parent(parent, AuLock_IR);
	/* returns a number of positive dentries */
	err = au_refresh_hdentry(dentry, type);
	if (err >= 0) {
		err = au_refresh_hinode(inode, dentry);
		if (!err && type == S_IFDIR)
			au_reset_hinotify(inode, dir_flags);
	}
	if (unlikely(err))
		AuErr("unrecoverable error %d, %.*s\n", err, AuDLNPair(dentry));
	di_read_unlock(parent, AuLock_IR);
	dput(parent);
	di_write_unlock(dentry);

	AuTraceErr(err);
	return err;
}

static int test_dir(struct dentry *dentry, void *arg)
{
	return S_ISDIR(dentry->d_inode->i_mode);
}

/* todo: merge with refresh_nondir()? */
static int refresh_dir(struct dentry *root, au_gen_t sgen)
{
	int err, i, j, ndentry, e;
	struct au_dcsub_pages dpages;
	struct au_dpage *dpage;
	struct dentry **dentries;
	struct inode *inode;
	const unsigned int flags = au_hi_flags(root->d_inode, /*isdir*/1);

	LKTRTrace("sgen %d\n", sgen);
	SiMustWriteLock(root->d_sb);
	/* dont trust BKL */
	AuDebugOn(au_digen(root) != sgen || !kernel_locked());

	err = 0;
	list_for_each_entry(inode, &root->d_sb->s_inodes, i_sb_list)
		if (S_ISDIR(inode->i_mode) && au_iigen(inode) != sgen) {
			ii_write_lock_child(inode);
			e = au_refresh_hinode_self(inode);
			ii_write_unlock(inode);
			if (unlikely(e)) {
				LKTRTrace("e %d, i%lu\n", e, inode->i_ino);
				if (!err)
					err = e;
				/* go on even if err */
			}
		}

	e = au_dpages_init(&dpages, GFP_NOFS);
	if (unlikely(e)) {
		if (!err)
			err = e;
		goto out;
	}
	e = au_dcsub_pages(&dpages, root, test_dir, NULL);
	if (unlikely(e)) {
		if (!err)
			err = e;
		goto out_dpages;
	}

	for (i = 0; !e && i < dpages.ndpage; i++) {
		dpage = dpages.dpages + i;
		dentries = dpage->dentries;
		ndentry = dpage->ndentry;
		for (j = 0; !e && j < ndentry; j++) {
			struct dentry *d;
			d = dentries[j];
#ifdef CONFIG_AUFS_DEBUG
			{
				struct dentry *parent;
				parent = dget_parent(d);
				AuDebugOn(!S_ISDIR(d->d_inode->i_mode)
					  || IS_ROOT(d)
					  || au_digen(parent) != sgen);
				dput(parent);
			}
#endif
			if (au_digen(d) != sgen) {
				e = do_refresh(d, S_IFDIR, flags);
				if (unlikely(e && !err))
					err = e;
				/* break on err */
			}
		}
	}

 out_dpages:
	au_dpages_free(&dpages);
 out:
	AuTraceErr(err);
	return err;
}

static int test_nondir(struct dentry *dentry, void *arg)
{
	return !S_ISDIR(dentry->d_inode->i_mode);
}

static int refresh_nondir(struct dentry *root, au_gen_t sgen, int do_dentry)
{
	int err, i, j, ndentry, e;
	struct au_dcsub_pages dpages;
	struct au_dpage *dpage;
	struct dentry **dentries;
	struct inode *inode;

	LKTRTrace("sgen %d\n", sgen);
	SiMustWriteLock(root->d_sb);
	/* dont trust BKL */
	AuDebugOn(au_digen(root) != sgen || !kernel_locked());

	err = 0;
	list_for_each_entry(inode, &root->d_sb->s_inodes, i_sb_list)
		if (!S_ISDIR(inode->i_mode) && au_iigen(inode) != sgen) {
			ii_write_lock_child(inode);
			e = au_refresh_hinode_self(inode);
			ii_write_unlock(inode);
			if (unlikely(e)) {
				LKTRTrace("e %d, i%lu\n", e, inode->i_ino);
				if (!err)
					err = e;
				/* go on even if err */
			}
		}

	if (!do_dentry)
		goto out;

	e = au_dpages_init(&dpages, GFP_NOFS);
	if (unlikely(e)) {
		if (!err)
			err = e;
		goto out;
	}
	e = au_dcsub_pages(&dpages, root, test_nondir, NULL);
	if (unlikely(e)) {
		if (!err)
			err = e;
		goto out_dpages;
	}

	for (i = 0; i < dpages.ndpage; i++) {
		dpage = dpages.dpages + i;
		dentries = dpage->dentries;
		ndentry = dpage->ndentry;
		for (j = 0; j < ndentry; j++) {
			struct dentry *d;
			d = dentries[j];
#ifdef CONFIG_AUFS_DEBUG
			{
				struct dentry *parent;
				parent = dget_parent(d);
				AuDebugOn(S_ISDIR(d->d_inode->i_mode)
					  || au_digen(parent) != sgen);
				dput(parent);
			}
#endif
			inode = d->d_inode;
			if (inode && au_digen(d) != sgen) {
				e = do_refresh(d, inode->i_mode & S_IFMT, 0);
				if (unlikely(e && !err))
					err = e;
				/* go on even err */
			}
		}
	}

 out_dpages:
	au_dpages_free(&dpages);
 out:
	AuTraceErr(err);
	return err;
}

/* stop extra interpretation of errno in mount(8), and strange error messages */
static int cvt_err(int err)
{
	AuTraceErr(err);

	switch (err) {
	case -ENOENT:
	case -ENOTDIR:
	case -EEXIST:
	case -EIO:
		err = -EINVAL;
	}
	return err;
}

/* protected by s_umount */
static int aufs_remount_fs(struct super_block *sb, int *flags, char *data)
{
	int err, rerr;
	au_gen_t sigen;
	struct dentry *root;
	struct inode *inode;
	struct au_opts opts;
	struct au_sbinfo *sbinfo;
	unsigned char dlgt;

	LKTRTrace("flags 0x%x, data %s, len %lu\n",
		  *flags, data ? data : "NULL",
		  (unsigned long)(data ? strlen(data) : 0));

	au_fsync_br(sb);

	err = 0;
	if (!data || !*data)
		goto out; /* success */

	err = -ENOMEM;
	memset(&opts, 0, sizeof(opts));
	opts.opt = (void *)__get_free_page(GFP_NOFS);
	if (unlikely(!opts.opt))
		goto out;
	opts.max_opt = PAGE_SIZE / sizeof(*opts.opt);
	opts.flags = AuOpts_REMOUNT;

	/* parse it before aufs lock */
	err = au_opts_parse(sb, *flags, data, &opts);
	if (unlikely(err))
		goto out_opts;

	sbinfo = au_sbi(sb);
	root = sb->s_root;
	inode = root->d_inode;
	mutex_lock(&inode->i_mutex);
	aufs_write_lock(root);

	/* au_do_opts() may return an error */
	err = au_opts_remount(sb, &opts);
	au_opts_free(&opts);

	if (au_ftest_opts(opts.flags, REFRESH_DIR)
	    || au_ftest_opts(opts.flags, REFRESH_NONDIR)) {
		dlgt = !!au_opt_test(sbinfo->si_mntflags, DLGT);
		au_opt_clr(sbinfo->si_mntflags, DLGT);
		au_sigen_inc(sb);
		au_reset_hinotify(inode, au_hi_flags(inode, /*isdir*/1));
		sigen = au_sigen(sb);
		au_fclr_si(sbinfo, FAILED_REFRESH_DIRS);

		DiMustNoWaiters(root);
		IiMustNoWaiters(root->d_inode);
		di_write_unlock(root);

		rerr = refresh_dir(root, sigen);
		if (unlikely(rerr)) {
			au_fset_si(sbinfo, FAILED_REFRESH_DIRS);
			AuWarn("Refreshing directories failed, ignores (%d)\n",
			       rerr);
		}

		if (unlikely(au_ftest_opts(opts.flags, REFRESH_NONDIR))) {
			rerr = refresh_nondir(root, sigen, !rerr);
			if (unlikely(rerr))
				AuWarn("Refreshing non-directories failed,"
				       " ignores (%d)\n", rerr);
		}

		/* aufs_write_lock() calls ..._child() */
		di_write_lock_child(root);

		au_cpup_attr_all(inode);
		if (unlikely(dlgt))
			au_opt_set(sbinfo->si_mntflags, DLGT);
	}

	aufs_write_unlock(root);
	mutex_unlock(&inode->i_mutex);

 out_opts:
	free_page((unsigned long)opts.opt);
 out:
	err = cvt_err(err);
	AuTraceErr(err);
	return err;
}

static struct super_operations aufs_sop = {
	.alloc_inode	= aufs_alloc_inode,
	.destroy_inode	= aufs_destroy_inode,
	.drop_inode	= generic_delete_inode,

	.show_options	= aufs_show_options,
	.statfs		= aufs_statfs,

	.put_super	= aufs_put_super,
	.remount_fs	= aufs_remount_fs,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 26)
	.umount_begin	= aufs_umount_begin
#endif
};

/* ---------------------------------------------------------------------- */

static int alloc_root(struct super_block *sb)
{
	int err;
	struct inode *inode;
	struct dentry *root;

	AuTraceEnter();

	err = -ENOMEM;
	inode = au_iget_locked(sb, AUFS_ROOT_INO);
	err = PTR_ERR(inode);
	if (IS_ERR(inode))
		goto out;
	inode->i_op = &aufs_dir_iop;
	inode->i_fop = &aufs_dir_fop;
	inode->i_mode = S_IFDIR;
	unlock_new_inode(inode);
	root = d_alloc_root(inode);
	if (unlikely(!root))
		goto out_iput;
	err = PTR_ERR(root);
	if (IS_ERR(root))
		goto out_iput;

	err = au_alloc_dinfo(root);
	if (!err) {
		sb->s_root = root;
		return 0; /* success */
	}
	dput(root);
	goto out; /* do not iput */

 out_iput:
	iget_failed(inode);
	iput(inode);
 out:
	AuTraceErr(err);
	return err;

}

static int aufs_fill_super(struct super_block *sb, void *raw_data, int silent)
{
	int err;
	struct dentry *root;
	struct inode *inode;
	struct au_opts opts;
	char *arg = raw_data;

	if (unlikely(!arg || !*arg)) {
		err = -EINVAL;
		AuErr("no arg\n");
		goto out;
	}
	LKTRTrace("%s, silent %d\n", arg, silent);

	err = -ENOMEM;
	memset(&opts, 0, sizeof(opts));
	opts.opt = (void *)__get_free_page(GFP_NOFS);
	if (unlikely(!opts.opt))
		goto out;
	opts.max_opt = PAGE_SIZE / sizeof(*opts.opt);

	err = au_si_alloc(sb);
	if (unlikely(err))
		goto out_opts;
	SiMustWriteLock(sb);
	/* all timestamps always follow the ones on the branch */
	sb->s_flags |= MS_NOATIME | MS_NODIRATIME;
	sb->s_op = &aufs_sop;
	sb->s_magic = AUFS_SUPER_MAGIC;
	au_export_init(sb);

	err = alloc_root(sb);
	if (unlikely(err)) {
		AuDebugOn(sb->s_root);
		si_write_unlock(sb);
		goto out_info;
	}
	root = sb->s_root;
	DiMustWriteLock(root);
	inode = root->d_inode;
	inode->i_nlink = 2;

	/*
	 * actually we can parse options regardless aufs lock here.
	 * but at remount time, parsing must be done before aufs lock.
	 * so we follow the same rule.
	 */
	ii_write_lock_parent(inode);
	aufs_write_unlock(root);
	err = au_opts_parse(sb, sb->s_flags, arg, &opts);
	if (unlikely(err))
		goto out_root;

	/* lock vfs_inode first, then aufs. */
	mutex_lock(&inode->i_mutex);
	inode->i_op = &aufs_dir_iop;
	inode->i_fop = &aufs_dir_fop;
	aufs_write_lock(root);

	sb->s_maxbytes = 0;
	err = au_opts_mount(sb, &opts);
	au_opts_free(&opts);
	if (unlikely(err))
		goto out_unlock;
	AuDebugOn(!sb->s_maxbytes);

	aufs_write_unlock(root);
	mutex_unlock(&inode->i_mutex);
	goto out_opts; /* success */

 out_unlock:
	aufs_write_unlock(root);
	mutex_unlock(&inode->i_mutex);
 out_root:
	dput(root);
	sb->s_root = NULL;
 out_info:
	kobject_put(&au_sbi(sb)->si_kobj);
	sb->s_fs_info = NULL;
 out_opts:
	free_page((unsigned long)opts.opt);
 out:
	AuTraceErr(err);
	err = cvt_err(err);
	AuTraceErr(err);
	return err;
}

/* ---------------------------------------------------------------------- */

static int aufs_get_sb(struct file_system_type *fs_type, int flags,
		       const char *dev_name, void *raw_data,
		       struct vfsmount *mnt)
{
	int err;
	struct super_block *sb;

	/* all timestamps always follow the ones on the branch */
	/* mnt->mnt_flags |= MNT_NOATIME | MNT_NODIRATIME; */
	err = get_sb_nodev(fs_type, flags, raw_data, aufs_fill_super, mnt);
	if (!err) {
		sb = mnt->mnt_sb;
		au_mnt_init(au_sbi(sb), mnt);
		si_write_lock(sb);
		sysaufs_brs_add(sb, 0);
		si_write_unlock(sb);
	}
	return err;
}

struct file_system_type aufs_fs_type = {
	.name		= AUFS_FSTYPE,
	.fs_flags	=
		FS_RENAME_DOES_D_MOVE	/* a race between rename and others*/
		| FS_REVAL_DOT,		/* for NFS branch */
	.get_sb		= aufs_get_sb,
	.kill_sb	= generic_shutdown_super,
	/* no need to __module_get() and module_put(). */
	.owner		= THIS_MODULE,
};
