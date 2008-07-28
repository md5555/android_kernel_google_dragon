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
 * handling file/dir, and address_space operation
 *
 * $Id: file.c,v 1.7 2008/06/09 01:11:58 sfjro Exp $
 */

#include <linux/pagemap.h>
#include "aufs.h"

/*
 * a dirty trick for handling FMODE_EXEC and deny_write_access().
 * because FMODE_EXEC flag is not passed to f_op->open(),
 * set it to file->private_data temporary.
 */
#if !defined(CONFIG_AUFS_MODULE) || defined(CONFIG_AUFS_DENY_WRITE_ACCESS_PATCH)
int au_store_fmode_exec(struct nameidata *nd, struct inode *inode)
{
	int err;
	union {
		void *p;
		unsigned long ul;
	} u;

	err = 0;
	if (nd
	    && (nd->flags & LOOKUP_OPEN)
	    && nd->intent.open.file
	    && (nd->intent.open.flags & FMODE_EXEC)
	    && inode
	    && S_ISREG(inode->i_mode)) {
		u.ul = nd->intent.open.flags;
		nd->intent.open.file->private_data = u.p;
		/* smp_mb(); */
		err = 1;
	}

	return err;
}
#endif

/* drop flags for writing */
unsigned int au_file_roflags(unsigned int flags)
{
	flags &= ~(O_WRONLY | O_RDWR | O_APPEND | O_CREAT | O_TRUNC);
	flags |= O_RDONLY | O_NOATIME;
	return flags;
}

/* common functions to regular file and dir */
struct file *au_h_open(struct dentry *dentry, aufs_bindex_t bindex, int flags,
		       struct file *file)
{
	struct file *h_file;
	struct dentry *h_dentry;
	struct inode *h_inode;
	struct super_block *sb;
	struct au_branch *br;
	int hinotify, err;

	LKTRTrace("%.*s, b%d, flags 0%o, f %d\n",
		  AuDLNPair(dentry), bindex, flags, !!file);
	AuDebugOn(!dentry);
	h_dentry = au_h_dptr(dentry, bindex);
	AuDebugOn(!h_dentry);
	h_inode = h_dentry->d_inode;
	/* todo: a race can happen, return ENOENT */
#if 0
	h_file = ERR_PTR(-ENOENT);
	if (unlikely(!h_inode))
		goto out;
#else
	AuDebugOn(!h_inode);
#endif

	sb = dentry->d_sb;
	hinotify = !!au_opt_test(au_mntflags(sb), UDBA_INOTIFY);
	br = au_sbr(sb, bindex);
	au_br_get(br);
	/* drop flags for writing */
	if (au_test_ro(sb, bindex, dentry->d_inode))
		flags = au_file_roflags(flags);
	flags &= ~O_CREAT;

	h_file = NULL;
	if (unlikely(file && au_test_nfs(h_dentry->d_sb)))
		h_file = au_h_intent(dentry, bindex, file);
	if (!h_file)
		h_file = dentry_open(dget(h_dentry), mntget(br->br_mnt), flags);

	/*
	 * a dirty trick for handling FMODE_EXEC and deny_write_access().
	 */
	if (file && (file->f_mode & FMODE_EXEC)) {
		h_file->f_mode |= FMODE_EXEC;
		smp_mb(); /* flush f_mode */
		err = au_deny_write_access(h_file);
		if (unlikely(err)) {
			fput(h_file);
			h_file = ERR_PTR(err);
		}
	}
	if (!IS_ERR(h_file))
		return h_file;

	au_br_put(br);
	AuTraceErrPtr(h_file);
	return h_file;
}

static int do_coo(struct dentry *dentry, aufs_bindex_t bstart)
{
	int err;
	struct dentry *parent, *h_parent, *h_dentry, *gparent;
	aufs_bindex_t bcpup;
	struct inode *h_dir, *h_inode, *dir;
	struct super_block *sb;

	LKTRTrace("%.*s\n", AuDLNPair(dentry));
	AuDebugOn(IS_ROOT(dentry));
	DiMustWriteLock(dentry);

	parent = dget_parent(dentry);
	di_write_lock_parent(parent);
	sb = dentry->d_sb;
	err = AuWbrCopyup(au_sbi(sb), dentry);
	bcpup = err;
	if (err < 0) {
		err = 0; /* stop copyup, it is not an error */
		goto out;
	}
	err = 0;

	h_parent = au_h_dptr(parent, bcpup);
	if (!h_parent) {
		err = au_cpup_dirs(dentry, bcpup, NULL);
		if (unlikely(err))
			goto out;
		h_parent = au_h_dptr(parent, bcpup);
	}

	h_dir = h_parent->d_inode;
	h_dentry = au_h_dptr(dentry, bstart);
	h_inode = h_dentry->d_inode;
	dir = parent->d_inode;
	/* todo: meaningless lock if CONFIG_AUFS_DEBUG is disabled. */
	gparent = NULL;
	if (unlikely(au_opt_test(au_mntflags(sb), UDBA_INOTIFY)
		     && !IS_ROOT(parent))) {
		gparent = dget_parent(parent);
		ii_read_lock_parent2(gparent->d_inode);
	}
	au_hdir_lock(h_dir, dir, bcpup);
	/* todo: test parent-gparent relationship? */
	mutex_lock_nested(&h_inode->i_mutex, AuLsc_I_CHILD);
	AuDebugOn(au_h_dptr(dentry, bcpup));
	err = au_sio_cpup_simple(dentry, bcpup, -1, AuCpup_DTIME);
	AuTraceErr(err);
	mutex_unlock(&h_inode->i_mutex);
	au_hdir_unlock(h_dir, dir, bcpup);
	if (unlikely(gparent)) {
		ii_read_unlock(gparent->d_inode);
		dput(gparent);
	}

 out:
	di_write_unlock(parent);
	dput(parent);
	AuTraceErr(err);
	return err;
}

int au_do_open(struct inode *inode, struct file *file,
	       int (*open)(struct file *file, int flags))
{
	int err, coo;
	struct dentry *dentry;
	struct super_block *sb;
	aufs_bindex_t bstart;

	dentry = file->f_dentry;
	LKTRTrace("i%lu, %.*s\n", inode->i_ino, AuDLNPair(dentry));

	sb = dentry->d_sb;
	si_read_lock(sb, AuLock_FLUSH);
	coo = 0;
	switch (au_mntflags(sb) & AuOptMask_COO) {
	case AuOpt_COO_LEAF:
		coo = !S_ISDIR(inode->i_mode);
		break;
	case AuOpt_COO_ALL:
		coo = 1;
		break;
	}
	err = au_finfo_init(file);
	if (unlikely(err))
		goto out;

	if (!coo) {
		di_read_lock_child(dentry, AuLock_IR);
		/* todo: remove this */
		bstart = au_dbstart(dentry);
	} else {
		di_write_lock_child(dentry);
		bstart = au_dbstart(dentry);
		if (au_test_ro(sb, bstart, dentry->d_inode)) {
			err = do_coo(dentry, bstart);
			if (err) {
				di_write_unlock(dentry);
				goto out_finfo;
			}
			/* todo: remove this */
			bstart = au_dbstart(dentry);
		}
		di_downgrade_lock(dentry, AuLock_IR);
	}

	err = open(file, file->f_flags);
	di_read_unlock(dentry, AuLock_IR);

 out_finfo:
	fi_write_unlock(file);
	if (unlikely(err))
		au_finfo_fin(file);
 out:
	si_read_unlock(sb);
	AuTraceErr(err);
	return err;
}

int au_reopen_nondir(struct file *file)
{
	int err;
	struct dentry *dentry;
	aufs_bindex_t bstart, bindex, bend;
	struct file *h_file, *h_file_tmp;

	dentry = file->f_dentry;
	LKTRTrace("%.*s\n", AuDLNPair(dentry));
	bstart = au_dbstart(dentry);
	AuDebugOn(S_ISDIR(dentry->d_inode->i_mode)
		  || !au_h_dptr(dentry, bstart)->d_inode);

	h_file_tmp = NULL;
	if (au_fbstart(file) == bstart) {
		h_file = au_h_fptr(file, bstart);
		if (file->f_mode == h_file->f_mode)
			return 0; /* success */
		h_file_tmp = h_file;
		get_file(h_file_tmp);
		au_set_h_fptr(file, bstart, NULL);
	}
	AuDebugOn(au_fbstart(file) < bstart
		  || au_fi(file)->fi_hfile[0 + bstart].hf_file);

	h_file = au_h_open(dentry, bstart, file->f_flags & ~O_TRUNC, file);
	err = PTR_ERR(h_file);
	if (IS_ERR(h_file))
		goto out; /* todo: close all? */
	err = 0;
	/* cpup_file_flags(h_file, file); */
	au_set_fbstart(file, bstart);
	au_set_h_fptr(file, bstart, h_file);
	au_update_figen(file);
	/* todo: necessary? */
	/* file->f_ra = h_file->f_ra; */

	/* close lower files */
	bend = au_fbend(file);
	for (bindex = bstart + 1; bindex <= bend; bindex++)
		au_set_h_fptr(file, bindex, NULL);
	au_set_fbend(file, bstart);

 out:
	if (h_file_tmp)
		fput(h_file_tmp);
	AuTraceErr(err);
	return err;
}

/* ---------------------------------------------------------------------- */

static int au_ready_to_write_wh(struct file *file, loff_t len,
				aufs_bindex_t bcpup)
{
	int err;
	struct dentry *dentry, *hi_wh, *old_h_dentry;
	struct au_dinfo *dinfo;
	aufs_bindex_t old_bstart;

	AuTraceEnter();

	dentry = file->f_dentry;
	hi_wh = au_hi_wh(dentry->d_inode, bcpup);
	if (!hi_wh)
		err = au_sio_cpup_wh(dentry, bcpup, len, file);
	else {
		/* already copied-up after unlink */
		dinfo = au_di(dentry);
		old_bstart = dinfo->di_bstart;
		dinfo->di_bstart = bcpup;
		old_h_dentry = dinfo->di_hdentry[0 + bcpup].hd_dentry;
		dinfo->di_hdentry[0 + bcpup].hd_dentry = hi_wh;
		err = au_reopen_nondir(file);
		dinfo->di_hdentry[0 + bcpup].hd_dentry = old_h_dentry;
		dinfo->di_bstart = old_bstart;
	}

	AuTraceErr(err);
	return err;
}

/*
 * prepare the @file for writing.
 */
int au_ready_to_write(struct file *file, loff_t len)
{
	int err;
	struct dentry *dentry, *parent, *h_dentry, *h_parent, *gparent;
	struct inode *h_inode, *h_dir, *inode, *dir;
	struct super_block *sb;
	aufs_bindex_t bstart, bcpup;

	dentry = file->f_dentry;
	LKTRTrace("%.*s, len %lld\n", AuDLNPair(dentry), len);
	FiMustWriteLock(file);

	sb = dentry->d_sb;
	bstart = au_fbstart(file);
	AuDebugOn(au_fbr(file, bstart) != au_sbr(sb, bstart));

	inode = dentry->d_inode;
	AuDebugOn(S_ISDIR(inode->i_mode));
	ii_read_lock_child(inode);
	LKTRTrace("rdonly %d, bstart %d\n",
		  au_test_ro(sb, bstart, inode), bstart);
	err = au_test_ro(sb, bstart, inode);
	ii_read_unlock(inode);
	if (!err && (au_h_fptr(file, bstart)->f_mode & FMODE_WRITE))
		return 0;

	/* need to cpup */
	di_write_lock_child(dentry);
	parent = dget_parent(dentry);
	di_write_lock_parent(parent);
	err = AuWbrCopyup(au_sbi(sb), dentry);
	bcpup = err;
	if (unlikely(err < 0))
		goto out_unlock;
	err = 0;

	h_parent = au_h_dptr(parent, bcpup);
	if (!h_parent) {
		err = au_cpup_dirs(dentry, bcpup, NULL);
		if (unlikely(err))
			goto out_unlock;
		h_parent = au_h_dptr(parent, bcpup);
	}

	/* todo: meaningless lock if CONFIG_AUFS_DEBUG is disabled. */
	gparent = NULL;
	if (unlikely(au_opt_test(au_mntflags(sb), UDBA_INOTIFY)
		     && !IS_ROOT(parent))) {
		gparent = dget_parent(parent);
		ii_read_lock_parent2(gparent->d_inode);
	}
	h_dir = h_parent->d_inode;
	h_dentry = au_h_fptr(file, au_fbstart(file))->f_dentry;
	h_inode = h_dentry->d_inode;
	dir = parent->d_inode;
	au_hdir_lock(h_dir, dir, bcpup);
	/* todo: test parent-gparent relationship? */
	mutex_lock_nested(&h_inode->i_mutex, AuLsc_I_CHILD);
	if (d_unhashed(dentry) /* || d_unhashed(h_dentry) */
	    /* || !h_inode->i_nlink */)
		err = au_ready_to_write_wh(file, len, bcpup);
	else {
		if (!au_h_dptr(dentry, bcpup))
			err = au_sio_cpup_simple(dentry, bcpup, len,
						 AuCpup_DTIME);
		AuTraceErr(err);
		if (!err)
			err = au_reopen_nondir(file);
		AuTraceErr(err);
	}
	mutex_unlock(&h_inode->i_mutex);
	au_hdir_unlock(h_dir, dir, bcpup);
	if (unlikely(gparent)) {
		ii_read_unlock(gparent->d_inode);
		dput(gparent);
	}

 out_unlock:
	di_write_unlock(parent);
	di_write_unlock(dentry);
	dput(parent);
	AuTraceErr(err);
	return err;

}

/* ---------------------------------------------------------------------- */

static int refresh_file_by_inode(struct file *file, int *need_reopen)
{
	int err;
	struct au_finfo *finfo;
	struct dentry *dentry, *parent, *old_h_dentry, *hi_wh;
	struct inode *inode, *dir, *h_dir;
	aufs_bindex_t bstart, new_bstart, old_bstart;
	struct super_block *sb;
	struct au_dinfo *dinfo;

	dentry = file->f_dentry;
	LKTRTrace("%.*s\n", AuDLNPair(dentry));
	FiMustWriteLock(file);

	err = 0;
	finfo = au_fi(file);
	inode = dentry->d_inode;
	sb = dentry->d_sb;
 again:
	bstart = au_ibstart(inode);
	if (bstart == finfo->fi_bstart)
		goto out;

	new_bstart = bstart;
	parent = dget_parent(dentry);
	dir = parent->d_inode;
	if (au_test_ro(sb, bstart, inode)) {
		di_read_lock_parent(parent, !AuLock_IR);
		err = AuWbrCopyup(au_sbi(sb), dentry);
		new_bstart = err;
		di_read_unlock(parent, !AuLock_IR);
		if (unlikely(err < 0))
			goto out_put;
		err = 0;
	}
	di_read_unlock(dentry, AuLock_IR);
	di_write_lock_child(dentry);
	/* someone else might change our inode while we were sleeping */
	/* todo: test more? */
	if (bstart != au_ibstart(inode)) {
		di_downgrade_lock(dentry, AuLock_IR);
		err = 0;
		dput(parent);
		goto again;
	}
	di_read_lock_parent(parent, AuLock_IR);
	bstart = new_bstart;

	hi_wh = au_hi_wh(inode, bstart);
	if (au_opt_test(au_mntflags(sb), PLINK)
	    && au_plink_test(sb, inode)
	    && !d_unhashed(dentry)) {
		err = au_test_and_cpup_dirs(dentry, bstart, NULL);

		/* always superio. */
#if 1 /* reserved for future use */
		h_dir = au_h_dptr(parent, bstart)->d_inode;
		au_hdir_lock(h_dir, dir, bstart);
		err = au_sio_cpup_simple(dentry, bstart, -1, AuCpup_DTIME);
		au_hdir_unlock(h_dir, dir, bstart);
#else
		if (!au_test_wkq(current)) {
			int wkq_err;
			struct cpup_pseudo_link_args args = {
				.errp		= &err,
				.dentry		= dentry,
				.bdst		= bstart,
				.do_lock	= 1
			};
			wkq_err = au_wkq_wait(call_cpup_pseudo_link, &args);
			if (unlikely(wkq_err))
				err = wkq_err;
		} else
			err = cpup_pseudo_link(dentry, bstart, /*do_lock*/1);
#endif
	} else if (hi_wh) {
		/* already copied-up after unlink */
		dinfo = au_di(dentry);
		old_bstart = dinfo->di_bstart;
		dinfo->di_bstart = bstart;
		old_h_dentry = dinfo->di_hdentry[0 + bstart].hd_dentry;
		dinfo->di_hdentry[0 + bstart].hd_dentry = hi_wh;
		err = au_reopen_nondir(file);
		dinfo->di_hdentry[0 + bstart].hd_dentry = old_h_dentry;
		dinfo->di_bstart = old_bstart;
		*need_reopen = 0;
	}
	di_read_unlock(parent, AuLock_IR);
	di_downgrade_lock(dentry, AuLock_IR);

 out_put:
	dput(parent);
 out:
	AuTraceErr(err);
	return err;
}

/*
 * after branch manipulating, refresh the file.
 */
static int refresh_file(struct file *file, int (*reopen)(struct file *file))
{
	int err, new_sz, need_reopen;
	struct dentry *dentry;
	aufs_bindex_t bend, bindex, brid;
	struct au_hfile *p;
	struct au_finfo *finfo;
	struct super_block *sb;
	struct inode *inode;

	dentry = file->f_dentry;
	LKTRTrace("%.*s\n", AuDLNPair(dentry));
	FiMustWriteLock(file);
	DiMustReadLock(dentry);
	inode = dentry->d_inode;
	IiMustReadLock(inode);

	err = -ENOMEM;
	sb = dentry->d_sb;
	finfo = au_fi(file);
	new_sz = sizeof(*finfo->fi_hfile) * (au_sbend(sb) + 1);
	p = au_kzrealloc(finfo->fi_hfile, sizeof(*p) * (finfo->fi_bend + 1),
			 new_sz, GFP_KERNEL);
	if (unlikely(!p))
		goto out;
	finfo->fi_hfile = p;

	p += finfo->fi_bstart;
	brid = p->hf_br->br_id;
	bend = finfo->fi_bend;
	for (bindex = finfo->fi_bstart; bindex <= bend; bindex++, p++) {
		struct au_hfile tmp, *q;
		aufs_bindex_t new_bindex;

		if (!p->hf_file)
			continue;
		new_bindex = au_find_bindex(sb, p->hf_br);
		if (new_bindex == bindex)
			continue;
		/* todo: test more? */
		if (new_bindex < 0) {
			au_set_h_fptr(file, bindex, NULL);
			continue;
		}

		/* swap two hidden inode, and loop again */
		q = finfo->fi_hfile + new_bindex;
		tmp = *q;
		*q = *p;
		*p = tmp;
		if (tmp.hf_file) {
			bindex--;
			p--;
		}
	}
	{
		aufs_bindex_t s = finfo->fi_bstart, e = finfo->fi_bend;
		finfo->fi_bstart = 0;
		finfo->fi_bend = au_sbend(sb);
		finfo->fi_bstart = s;
		finfo->fi_bend = e;
	}

	p = finfo->fi_hfile;
	if (!au_test_mmapped(file) && !d_unhashed(dentry)) {
		bend = au_sbend(sb);
		for (finfo->fi_bstart = 0; finfo->fi_bstart <= bend;
		     finfo->fi_bstart++, p++)
			if (p->hf_file) {
				if (p->hf_file->f_dentry
				    && p->hf_file->f_dentry->d_inode)
					break;
				else
					au_hfput(p);
			}
	} else {
		bend = au_br_index(sb, brid);
		for (finfo->fi_bstart = 0; finfo->fi_bstart < bend;
		     finfo->fi_bstart++, p++)
			if (p->hf_file)
				au_hfput(p);
		bend = au_sbend(sb);
	}

	p = finfo->fi_hfile + bend;
	for (finfo->fi_bend = bend; finfo->fi_bend >= finfo->fi_bstart;
	     finfo->fi_bend--, p--)
		if (p->hf_file) {
			if (p->hf_file->f_dentry
			    && p->hf_file->f_dentry->d_inode)
				break;
			else
				au_hfput(p);
		}
	AuDebugOn(finfo->fi_bend < finfo->fi_bstart);

	err = 0;
	need_reopen = 1;
	if (!au_test_mmapped(file))
		err = refresh_file_by_inode(file, &need_reopen);
	if (!err && need_reopen && !d_unhashed(dentry))
		err = reopen(file);
	if (!err) {
		au_update_figen(file);
		return 0; /* success */
	}

	/* error, close all hidden files */
	bend = au_fbend(file);
	for (bindex = au_fbstart(file); bindex <= bend; bindex++)
		au_set_h_fptr(file, bindex, NULL);

 out:
	AuTraceErr(err);
	return err;
}

/* common function to regular file and dir */
int au_reval_and_lock_finfo(struct file *file, int (*reopen)(struct file *file),
			    int wlock, int locked)
{
	int err, pseudo_link;
	struct dentry *dentry;
	struct super_block *sb;
	aufs_bindex_t bstart;
	au_gen_t sgen, fgen;

	dentry = file->f_dentry;
	LKTRTrace("%.*s, w %d, l %d\n", AuDLNPair(dentry), wlock, locked);
	sb = dentry->d_sb;
	SiMustAnyLock(sb);

	err = 0;
	sgen = au_sigen(sb);
	fi_write_lock(file);
	fgen = au_figen(file);
	di_read_lock_child(dentry, AuLock_IR);
	bstart = au_dbstart(dentry);
	pseudo_link = (bstart != au_ibstart(dentry->d_inode));
	di_read_unlock(dentry, AuLock_IR);
	if (sgen == fgen && !pseudo_link && au_fbstart(file) == bstart) {
		if (!wlock)
			fi_downgrade_lock(file);
		goto out; /* success */
	}

	LKTRTrace("sgen %d, fgen %d\n", sgen, fgen);
	if (unlikely(sgen != au_digen(dentry)
		     || sgen != au_iigen(dentry->d_inode))) {
		/*
		 * d_path() and path_lookup() is a simple and good approach
		 * to revalidate. but si_rwsem in DEBUG_RWSEM will cause a
		 * deadlock. removed the code.
		 */
		di_write_lock_child(dentry);
		err = au_reval_dpath(dentry, sgen);
		di_write_unlock(dentry);
		if (unlikely(err < 0))
			goto out;
		AuDebugOn(au_digen(dentry) != sgen
			  || au_iigen(dentry->d_inode) != sgen);
	}

	di_read_lock_child(dentry, AuLock_IR);
	err = refresh_file(file, reopen
			   /* , au_opt_test(au_mnt_flags(sb), REFROF) */);
	di_read_unlock(dentry, AuLock_IR);
	if (!err) {
		if (!wlock)
			fi_downgrade_lock(file);
	} else
		fi_write_unlock(file);

 out:
	AuTraceErr(err);
	return err;
}

/* ---------------------------------------------------------------------- */

/* cf. aufs_nopage() */
/* for madvise(2) */
static int aufs_readpage(struct file *file, struct page *page)
{
	AuTraceEnter();
	unlock_page(page);
	return 0;
}

/* they will never be called. */
#ifdef CONFIG_AUFS_DEBUG
static int aufs_prepare_write(struct file *file, struct page *page,
			      unsigned from, unsigned to)
{ AuUnsupport(); return 0; }
static int aufs_commit_write(struct file *file, struct page *page,
			     unsigned from, unsigned to)
{ AuUnsupport(); return 0; }
static int aufs_write_begin(struct file *file, struct address_space *mapping,
			    loff_t pos, unsigned len, unsigned flags,
			    struct page **pagep, void **fsdata)
{ AuUnsupport(); return 0; }
static int aufs_write_end(struct file *file, struct address_space *mapping,
			  loff_t pos, unsigned len, unsigned copied,
			  struct page *page, void *fsdata)
{ AuUnsupport(); return 0; }
static int aufs_writepage(struct page *page, struct writeback_control *wbc)
{ AuUnsupport(); return 0; }
static void aufs_sync_page(struct page *page)
{ AuUnsupport(); }

static int aufs_set_page_dirty(struct page *page)
{ AuUnsupport(); return 0; }
static void aufs_invalidatepage(struct page *page, unsigned long offset)
{ AuUnsupport(); }
static int aufs_releasepage(struct page *page, gfp_t gfp)
{ AuUnsupport(); return 0; }
static ssize_t aufs_direct_IO(int rw, struct kiocb *iocb,
			      const struct iovec *iov, loff_t offset,
			      unsigned long nr_segs)
{ AuUnsupport(); return 0; }
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 26)
static struct page *aufs_get_xip_page(struct address_space *mapping,
				      sector_t offset, int create)
{ AuUnsupport(); return NULL; }
#endif
#endif /* CONFIG_AUFS_DEBUG */

struct address_space_operations aufs_aop = {
	.readpage	= aufs_readpage,
#ifdef CONFIG_AUFS_DEBUG
	.writepage	= aufs_writepage,
	.sync_page	= aufs_sync_page,
	.set_page_dirty	= aufs_set_page_dirty,
	.prepare_write	= aufs_prepare_write,
	.commit_write	= aufs_commit_write,
	.write_begin	= aufs_write_begin,
	.write_end	= aufs_write_end,
	.invalidatepage	= aufs_invalidatepage,
	.releasepage	= aufs_releasepage,
	.direct_IO	= aufs_direct_IO,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 26)
	.get_xip_page	= aufs_get_xip_page,
#endif
#endif /* CONFIG_AUFS_DEBUG */
};
