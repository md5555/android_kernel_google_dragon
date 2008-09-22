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
 * directory operations
 *
 * $Id: dir.c,v 1.13 2008/09/22 03:52:19 sfjro Exp $
 */

#include <linux/fs_stack.h>
#include "aufs.h"

static int reopen_dir(struct file *file)
{
	int err;
	struct dentry *dentry, *h_dentry;
	aufs_bindex_t bindex, btail, bstart;
	struct file *h_file;

	dentry = file->f_dentry;
	LKTRTrace("%.*s\n", AuDLNPair(dentry));
	AuDebugOn(!S_ISDIR(dentry->d_inode->i_mode));

	/* open all hidden dirs */
	bstart = au_dbstart(dentry);
#if 1 /* todo: necessary? */
	for (bindex = au_fbstart(file); bindex < bstart; bindex++)
		au_set_h_fptr(file, bindex, NULL);
#endif
	au_set_fbstart(file, bstart);
	btail = au_dbtaildir(dentry);
#if 1 /* todo: necessary? */
	for (bindex = au_fbend(file); btail < bindex; bindex--)
		au_set_h_fptr(file, bindex, NULL);
#endif
	au_set_fbend(file, btail);
	for (bindex = bstart; bindex <= btail; bindex++) {
		h_dentry = au_h_dptr(dentry, bindex);
		if (!h_dentry)
			continue;
		h_file = au_h_fptr(file, bindex);
		if (h_file) {
			AuDebugOn(h_file->f_dentry != h_dentry);
			continue;
		}

		h_file = au_h_open(dentry, bindex, file->f_flags, file);
		err = PTR_ERR(h_file);
		if (IS_ERR(h_file))
			goto out; /* close all? */
		/* cpup_file_flags(h_file, file); */
		au_set_h_fptr(file, bindex, h_file);
	}
	au_update_figen(file);
	/* todo: necessary? */
	/* file->f_ra = h_file->f_ra; */
	err = 0;

 out:
	AuTraceErr(err);
	return err;
}

static int do_open_dir(struct file *file, int flags)
{
	int err;
	aufs_bindex_t bindex, btail;
	struct dentry *dentry, *h_dentry;
	struct file *h_file;

	dentry = file->f_dentry;
	LKTRTrace("%.*s, 0x%x\n", AuDLNPair(dentry), flags);
	AuDebugOn(!dentry->d_inode || !S_ISDIR(dentry->d_inode->i_mode));

	err = 0;
	au_set_fvdir_cache(file, NULL);
	file->f_version = dentry->d_inode->i_version;
	bindex = au_dbstart(dentry);
	au_set_fbstart(file, bindex);
	btail = au_dbtaildir(dentry);
	au_set_fbend(file, btail);
	for (; !err && bindex <= btail; bindex++) {
		h_dentry = au_h_dptr(dentry, bindex);
		if (!h_dentry)
			continue;

		h_file = au_h_open(dentry, bindex, flags, file);
		if (IS_ERR(h_file)) {
			err = PTR_ERR(h_file);
			break;
		}
		au_set_h_fptr(file, bindex, h_file);
	}
	au_update_figen(file);
	/* todo: necessary? */
	/* file->f_ra = h_file->f_ra; */
	if (!err)
		return 0; /* success */

	/* close all */
	for (bindex = au_fbstart(file); bindex <= btail; bindex++)
		au_set_h_fptr(file, bindex, NULL);
	au_set_fbstart(file, -1);
	au_set_fbend(file, -1);
	return err;
}

static int aufs_open_dir(struct inode *inode, struct file *file)
{
	LKTRTrace("i%lu, %.*s\n", inode->i_ino, AuDLNPair(file->f_dentry));

	return au_do_open(inode, file, do_open_dir);
}

static int aufs_release_dir(struct inode *inode, struct file *file)
{
	struct au_vdir *vdir_cache;
	struct super_block *sb;

	LKTRTrace("i%lu, %.*s\n", inode->i_ino, AuDLNPair(file->f_dentry));

	sb = file->f_dentry->d_sb;
	si_noflush_read_lock(sb);
	fi_write_lock(file);
	vdir_cache = au_fvdir_cache(file);
	if (vdir_cache)
		au_vdir_free(vdir_cache);
	fi_write_unlock(file);
	au_finfo_fin(file);
	si_read_unlock(sb);
	return 0;
}

static int fsync_dir(struct dentry *dentry, int datasync)
{
	int err;
	struct inode *inode;
	struct super_block *sb;
	aufs_bindex_t bend, bindex;

	LKTRTrace("%.*s, %d\n", AuDLNPair(dentry), datasync);
	DiMustAnyLock(dentry);
	sb = dentry->d_sb;
	SiMustAnyLock(sb);
	inode = dentry->d_inode;
	IMustLock(inode);
	IiMustAnyLock(inode);

	err = 0;
	bend = au_dbend(dentry);
	for (bindex = au_dbstart(dentry); !err && bindex <= bend; bindex++) {
		struct dentry *h_dentry;
		struct inode *h_inode;
		struct file_operations *fop;

		if (au_test_ro(sb, bindex, inode))
			continue;
		h_dentry = au_h_dptr(dentry, bindex);
		if (!h_dentry)
			continue;
		h_inode = h_dentry->d_inode;
		if (!h_inode)
			continue;

		/* cf. fs/nsfd/vfs.c and fs/nfsd/nfs4recover.c */
		/* todo: inotiry fired? */
		mutex_lock(&h_inode->i_mutex);
		fop = (void *)h_inode->i_fop;
		err = filemap_fdatawrite(h_inode->i_mapping);
		if (!err && fop && fop->fsync)
			err = fop->fsync(NULL, h_dentry, datasync);
		if (!err)
			err = filemap_fdatawrite(h_inode->i_mapping);
		if (!err)
			au_update_fuse_h_inode(NULL, h_dentry); /*ignore*/
		mutex_unlock(&h_inode->i_mutex);
	}

	AuTraceErr(err);
	return err;
}

/*
 * @file may be NULL
 */
static int aufs_fsync_dir(struct file *file, struct dentry *dentry,
			  int datasync)
{
	int err;
	struct inode *inode;
	struct file *h_file;
	struct super_block *sb;
	aufs_bindex_t bend, bindex;

	LKTRTrace("%.*s, %d\n", AuDLNPair(dentry), datasync);
	inode = dentry->d_inode;
	IMustLock(inode);

	err = 0;
	sb = dentry->d_sb;
	si_noflush_read_lock(sb);
	if (file) {
		err = au_reval_and_lock_fdi(file, reopen_dir, /*wlock*/1,
					/*locked*/1);
		if (unlikely(err))
			goto out;
	} else
		di_write_lock_child(dentry);

	if (file) {
		bend = au_fbend(file);
		for (bindex = au_fbstart(file); !err && bindex <= bend;
		     bindex++) {
			h_file = au_h_fptr(file, bindex);
			if (!h_file || au_test_ro(sb, bindex, inode))
				continue;

			err = -EINVAL;
			if (h_file->f_op && h_file->f_op->fsync) {
				/* todo: try do_fsync() in fs/sync.c? */
				mutex_lock(&h_file->f_mapping->host->i_mutex);
				err = h_file->f_op->fsync
					(h_file, h_file->f_dentry, datasync);
				if (!err)
					au_update_fuse_h_inode
						(h_file->f_vfsmnt,
						 h_file->f_dentry);
				/*ignore*/
				mutex_unlock(&h_file->f_mapping->host->i_mutex);
			}
		}
	} else
		err = fsync_dir(dentry, datasync);
	au_cpup_attr_timesizes(inode);
	di_write_unlock(dentry);
	if (file)
		fi_write_unlock(file);

 out:
	si_read_unlock(sb);
	AuTraceErr(err);
	return err;
}

/* ---------------------------------------------------------------------- */

static int aufs_readdir(struct file *file, void *dirent, filldir_t filldir)
{
	int err, iflag;
	struct dentry *dentry;
	struct inode *inode;
	struct super_block *sb;

	dentry = file->f_dentry;
	LKTRTrace("%.*s, pos %lld\n", AuDLNPair(dentry), file->f_pos);
	inode = dentry->d_inode;
	IMustLock(inode);

	au_nfsd_lockdep_off();
	sb = dentry->d_sb;
	si_read_lock(sb, AuLock_FLUSH);
	err = au_reval_and_lock_fdi(file, reopen_dir, /*wlock*/1, /*locked*/1);
	if (unlikely(err))
		goto out;

	err = au_vdir_init(file);
	if (unlikely(err)) {
		di_write_unlock(dentry);
		goto out_unlock;
	}

	/* nfsd filldir calls lookup_one_len(). */
	iflag = AuLock_IW;
	if (unlikely(au_test_nfsd(current)))
		iflag = AuLock_IR;
	di_downgrade_lock(dentry, iflag);
	err = au_vdir_fill_de(file, dirent, filldir);

	fsstack_copy_attr_atime(inode, au_h_iptr(inode, au_ibstart(inode)));
	di_read_unlock(dentry, iflag);

 out_unlock:
	fi_write_unlock(file);
 out:
	si_read_unlock(sb);
	au_nfsd_lockdep_on();
	AuTraceErr(err);
	return err;
}

/* ---------------------------------------------------------------------- */

#define AuTestEmpty_WHONLY	1
#define AuTestEmpty_DLGT	(1 << 1)
#define AuTestEmpty_DIRPERM1	(1 << 2)
#define AuTestEmpty_CALLED	(1 << 3)
#define AuTestEmpty_SHWH	(1 << 4)
#define au_ftest_testempty(flags, name)	((flags) & AuTestEmpty_##name)
#define au_fset_testempty(flags, name)	{ (flags) |= AuTestEmpty_##name; }
#define au_fclr_testempty(flags, name)	{ (flags) &= ~AuTestEmpty_##name; }
#ifndef CONFIG_AUFS_DLGT
#undef AuTestEmpty_DLGT
#define AuTestEmpty_DLGT	0
#undef AuTestEmpty_DIRPERM1
#define AuTestEmpty_DIRPERM1	0
#endif
#ifndef CONFIG_AUFS_SHWH
#undef AuTestEmpty_SHWH
#define AuTestEmpty_SHWH	0
#endif

struct test_empty_arg {
	struct au_nhash *whlist;
	unsigned int flags;
	int err;
	aufs_bindex_t bindex;
};

static int test_empty_cb(void *__arg, const char *__name, int namelen,
			 loff_t offset, u64 ino, unsigned int d_type)
{
	struct test_empty_arg *arg = __arg;
	char *name = (void *)__name;

	LKTRTrace("%.*s\n", namelen, name);

	arg->err = 0;
	au_fset_testempty(arg->flags, CALLED);
	/* smp_mb(); */
	if (name[0] == '.'
	    && (namelen == 1 || (name[1] == '.' && namelen == 2)))
		return 0; /* success */

	if (namelen <= AUFS_WH_PFX_LEN
	    || memcmp(name, AUFS_WH_PFX, AUFS_WH_PFX_LEN)) {
		if (au_ftest_testempty(arg->flags, WHONLY)
		    && !au_nhash_test_known_wh(arg->whlist, name, namelen))
			arg->err = -ENOTEMPTY;
		goto out;
	}

	name += AUFS_WH_PFX_LEN;
	namelen -= AUFS_WH_PFX_LEN;
	if (!au_nhash_test_known_wh(arg->whlist, name, namelen))
		arg->err = au_nhash_append_wh
			(arg->whlist, name, namelen, ino, d_type, arg->bindex,
			 au_ftest_testempty(arg->flags, SHWH));

 out:
	/* smp_mb(); */
	AuTraceErr(arg->err);
	return arg->err;
}

static int do_test_empty(struct dentry *dentry, struct test_empty_arg *arg)
{
	int err, dlgt;
	struct file *h_file;

	LKTRTrace("%.*s, {%p, 0x%x, %d}\n",
		  AuDLNPair(dentry), arg->whlist, arg->flags, arg->bindex);

	h_file = au_h_open(dentry, arg->bindex,
			   O_RDONLY | O_NONBLOCK | O_DIRECTORY | O_LARGEFILE,
			   /*file*/NULL);
	err = PTR_ERR(h_file);
	if (IS_ERR(h_file))
		goto out;
	err = 0;
	if (unlikely(au_opt_test(au_mntflags(dentry->d_sb), UDBA_INOTIFY)
		     && !h_file->f_dentry->d_inode->i_nlink))
		goto out_put;

	dlgt = au_ftest_testempty(arg->flags, DLGT);
	do {
		arg->err = 0;
		au_fclr_testempty(arg->flags, CALLED);
		/* smp_mb(); */
		err = vfsub_readdir(h_file, test_empty_cb, arg, dlgt);
		if (err >= 0)
			err = arg->err;
	} while (!err && au_ftest_testempty(arg->flags, CALLED));

 out_put:
	fput(h_file);
	au_sbr_put(dentry->d_sb, arg->bindex);
 out:
	AuTraceErr(err);
	return err;
}

struct do_test_empty_args {
	int *errp;
	struct dentry *dentry;
	struct test_empty_arg *arg;
};

static void call_do_test_empty(void *args)
{
	struct do_test_empty_args *a = args;
	*a->errp = do_test_empty(a->dentry, a->arg);
}

static int sio_test_empty(struct dentry *dentry, struct test_empty_arg *arg)
{
	int err, wkq_err;
	struct dentry *h_dentry;
	struct inode *h_inode;

	LKTRTrace("%.*s\n", AuDLNPair(dentry));
	h_dentry = au_h_dptr(dentry, arg->bindex);
	AuDebugOn(!h_dentry);
	h_inode = h_dentry->d_inode;
	AuDebugOn(!h_inode);

	mutex_lock_nested(&h_inode->i_mutex, AuLsc_I_CHILD);
	err = au_test_h_perm_sio(h_inode, MAY_EXEC | MAY_READ,
				 au_test_dlgt(au_mntflags(dentry->d_sb)));
	mutex_unlock(&h_inode->i_mutex);
	if (!err)
		err = do_test_empty(dentry, arg);
	else {
		struct do_test_empty_args args = {
			.errp	= &err,
			.dentry	= dentry,
			.arg	= arg
		};
		unsigned int flags = arg->flags;

		au_fclr_testempty(arg->flags, DLGT);
		au_fclr_testempty(arg->flags, DIRPERM1);
		wkq_err = au_wkq_wait(call_do_test_empty, &args, /*dlgt*/0);
		if (unlikely(wkq_err))
			err = wkq_err;
		arg->flags = flags;
	}

	AuTraceErr(err);
	return err;
}

int au_test_empty_lower(struct dentry *dentry)
{
	int err;
	struct inode *inode;
	struct test_empty_arg arg;
	struct au_nhash *whlist;
	aufs_bindex_t bindex, bstart, btail;
	unsigned int mnt_flags;

	LKTRTrace("%.*s\n", AuDLNPair(dentry));
	inode = dentry->d_inode;
	AuDebugOn(!inode || !S_ISDIR(inode->i_mode));

	whlist = au_nhash_new(GFP_NOFS);
	err = PTR_ERR(whlist);
	if (IS_ERR(whlist))
		goto out;

	bstart = au_dbstart(dentry);
	mnt_flags = au_mntflags(dentry->d_sb);
	arg.whlist = whlist;
	arg.flags = 0;
	if (unlikely(au_test_dlgt(mnt_flags)))
		au_fset_testempty(arg.flags, DLGT);
	if (unlikely(au_opt_test(mnt_flags, SHWH)))
		au_fset_testempty(arg.flags, SHWH);
	arg.bindex = bstart;
	err = do_test_empty(dentry, &arg);
	if (unlikely(err))
		goto out_whlist;

	au_fset_testempty(arg.flags, WHONLY);
	if (unlikely(au_test_dirperm1(mnt_flags)))
		au_fset_testempty(arg.flags, DIRPERM1);
	btail = au_dbtaildir(dentry);
	for (bindex = bstart + 1; !err && bindex <= btail; bindex++) {
		struct dentry *h_dentry;
		h_dentry = au_h_dptr(dentry, bindex);
		if (h_dentry && h_dentry->d_inode) {
			arg.bindex = bindex;
			err = do_test_empty(dentry, &arg);
		}
	}

 out_whlist:
	au_nhash_del(whlist);
 out:
	AuTraceErr(err);
	return err;
}

int au_test_empty(struct dentry *dentry, struct au_nhash *whlist)
{
	int err;
	struct inode *inode;
	struct test_empty_arg arg;
	aufs_bindex_t bindex, btail;

	LKTRTrace("%.*s\n", AuDLNPair(dentry));
	inode = dentry->d_inode;
	AuDebugOn(!inode || !S_ISDIR(inode->i_mode));

	err = 0;
	arg.whlist = whlist;
	arg.flags = AuTestEmpty_WHONLY;
	if (unlikely(au_opt_test(au_mntflags(dentry->d_sb), SHWH)))
		au_fset_testempty(arg.flags, SHWH);
	btail = au_dbtaildir(dentry);
	for (bindex = au_dbstart(dentry); !err && bindex <= btail; bindex++) {
		struct dentry *h_dentry;
		h_dentry = au_h_dptr(dentry, bindex);
		if (h_dentry && h_dentry->d_inode) {
			arg.bindex = bindex;
			err = sio_test_empty(dentry, &arg);
		}
	}

	AuTraceErr(err);
	return err;
}

/* ---------------------------------------------------------------------- */

struct file_operations aufs_dir_fop = {
	.read		= generic_read_dir,
	.readdir	= aufs_readdir,
	.open		= aufs_open_dir,
	.release	= aufs_release_dir,
	.flush		= aufs_flush,
	.fsync		= aufs_fsync_dir,
};
