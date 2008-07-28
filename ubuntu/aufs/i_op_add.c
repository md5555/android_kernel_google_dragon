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
 * inode operations (add entry)
 *
 * $Id: i_op_add.c,v 1.6 2008/06/02 02:38:21 sfjro Exp $
 */

#include "aufs.h"

/*
 * final procedure of adding a new entry, except link(2).
 * remove whiteout, instantiate, copyup the parent dir's times and size
 * and update version.
 * if it failed, re-create the removed whiteout.
 */
static int epilog(struct inode *dir, struct dentry *wh_dentry,
		  struct dentry *dentry)
{
	int err, rerr;
	aufs_bindex_t bwh;
	struct inode *inode, *h_dir;
	struct dentry *wh;
	struct au_ndx ndx;
	struct super_block *sb;

	LKTRTrace("wh %p, %.*s\n", wh_dentry, AuDLNPair(dentry));

	bwh = -1;
	if (wh_dentry) {
		h_dir = wh_dentry->d_parent->d_inode; /* dir inode is locked */
		IMustLock(h_dir);
		bwh = au_dbwh(dentry);
		err = au_wh_unlink_dentry(h_dir, wh_dentry, dentry, dir,
					  /*dlgt*/0);
		if (unlikely(err))
			goto out;
	}

	inode = au_new_inode(dentry);
	if (!IS_ERR(inode)) {
		d_instantiate(dentry, inode);
		dir = dentry->d_parent->d_inode; /* dir inode is locked */
		IMustLock(dir);
		/* or always cpup dir mtime? */
		if (au_ibstart(dir) == au_dbstart(dentry))
			au_cpup_attr_timesizes(dir);
		dir->i_version++;
		return 0; /* success */
	}

	err = PTR_ERR(inode);
	if (!wh_dentry)
		goto out;

	/* revert */
	sb = dentry->d_sb;
	ndx.flags = 0;
	if (unlikely(au_test_dlgt(au_mntflags(sb))))
		au_fset_ndx(ndx.flags, DLGT);
	ndx.nfsmnt = au_nfsmnt(sb, bwh);
	ndx.nd = NULL;
	/* ndx.br = NULL; */
	/* dir inode is locked */
	wh = au_wh_create(dir, dentry, bwh, wh_dentry->d_parent, &ndx);
	rerr = PTR_ERR(wh);
	if (!IS_ERR(wh)) {
		dput(wh);
		goto out;
	}
	AuIOErr("%.*s reverting whiteout failed(%d, %d)\n",
		AuDLNPair(dentry), err, rerr);
	err = -EIO;

 out:
	AuTraceErr(err);
	return err;
}

/*
 * simple tests for the adding inode operations.
 * following the checks in vfs, plus the parent-child relationship.
 */
int au_may_add(struct dentry *dentry, aufs_bindex_t bindex,
	       struct dentry *h_parent, int isdir, struct au_ndx *ndx)
{
	int err, exist;
	struct dentry *h_dentry;
	struct inode *h_inode;
	umode_t h_mode;

	LKTRTrace("%.*s/%.*s, b%d, dir %d\n",
		  AuDLNPair(h_parent), AuDLNPair(dentry), bindex, isdir);

	exist = !!dentry->d_inode;
	h_dentry = au_h_dptr(dentry, bindex);
	h_inode = h_dentry->d_inode;
	if (!exist) {
		err = -EEXIST;
		if (unlikely(h_inode))
			goto out;
	} else {
		/* rename(2) case */
		err = -EIO;
		if (unlikely(!h_inode || !h_inode->i_nlink))
			goto out;

		h_mode = h_inode->i_mode;
		if (!isdir) {
			err = -EISDIR;
			if (unlikely(S_ISDIR(h_mode)))
				goto out;
		} else if (unlikely(!S_ISDIR(h_mode))) {
			err = -ENOTDIR;
			goto out;
		}
	}

	err = -EIO;
	/* expected parent dir is locked */
	if (unlikely(h_parent != h_dentry->d_parent))
		goto out;
	err = 0;

	if (unlikely(au_opt_test(au_mntflags(dentry->d_sb), UDBA_INOTIFY))) {
		struct dentry *h_latest;
		struct qstr *qstr = &dentry->d_name;

		err = -EACCES;
		if (unlikely(au_test_h_perm
			     (h_parent->d_inode, MAY_EXEC | MAY_WRITE,
			      au_ftest_ndx(ndx->flags, DLGT))))
			goto out;

		err = -EIO;
		h_latest = au_sio_lkup_one(qstr->name, h_parent, qstr->len,
					   ndx);
		err = PTR_ERR(h_latest);
		if (IS_ERR(h_latest))
			goto out;
		err = -EIO;
		dput(h_latest);
		if (h_latest == h_dentry)
			err = 0;
	}

 out:
	AuTraceErr(err);
	return err;
}

/*
 * initial procedure of adding a new entry.
 * prepare writable branch and the parent dir, lock it,
 * lookup whiteout for the new entry.
 */
static struct dentry *
lock_hdir_lkup_wh(struct dentry *dentry, struct au_dtime *dt,
		  struct dentry *src_dentry, struct au_wr_dir_args *wr_dir_args)
{
	struct dentry *wh_dentry, *parent, *h_parent, *gparent;
	int err;
	aufs_bindex_t bstart, bcpup;
	struct inode *dir, *h_dir, *gdir;
	struct au_ndx ndx;
	struct super_block *sb;
	struct au_hinode *hgdir;
	unsigned int mnt_flags;

	LKTRTrace("%.*s, src %p\n", AuDLNPair(dentry), src_dentry);

	parent = dentry->d_parent; /* dir inode is locked */
	IMustLock(parent->d_inode);
	bstart = au_dbstart(dentry);
	err = au_wr_dir(dentry, src_dentry, wr_dir_args);
	bcpup = err;
	wh_dentry = ERR_PTR(err);
	if (unlikely(err < 0))
		goto out;

	sb = parent->d_sb;
	mnt_flags = au_mntflags(sb);
	/* todo: meaningless lock if CONFIG_AUFS_DEBUG is disabled. */
	hgdir = NULL;
	if (unlikely(dt && au_opt_test(mnt_flags, UDBA_INOTIFY)
		     && !IS_ROOT(parent))) {
		gparent = dget_parent(parent);
		gdir = gparent->d_inode;
		ii_read_lock_parent2(gdir);
		hgdir = au_hi(gdir, bcpup);
		ii_read_unlock(gdir);
		dput(gparent);
	}
	dir = parent->d_inode;
	h_parent = au_h_dptr(parent, bcpup);
	h_dir = h_parent->d_inode;

	AuDbgSleep_UdbaRace();
	au_hdir_lock(h_dir, dir, bcpup);
	/* todo: revalidate the lower dentry? */

	ndx.nfsmnt = au_nfsmnt(sb, bcpup);
	ndx.flags = 0;
	if (unlikely(au_test_dlgt(mnt_flags)))
		au_fset_ndx(ndx.flags, DLGT);
	ndx.nd = NULL;
	/* ndx.br = NULL; */
	/* ndx.nd_file = NULL; */

	if (!au_opt_test(mnt_flags, UDBA_NONE) && au_dbstart(dentry) == bcpup) {
		struct nameidata nd;

		if (unlikely(ndx.nfsmnt)) {
			/* todo: dirty? */
			ndx.nd = &nd;
			ndx.br = au_sbr(sb, bcpup);
			memset(&nd, 0, sizeof(nd));
			nd.flags = LOOKUP_CREATE;
			nd.intent.open.flags = O_EXCL;
		}
		err = au_may_add(dentry, bcpup, h_parent,
				 au_ftest_wrdir(wr_dir_args->flags, ISDIR),
				 &ndx);
		wh_dentry = ERR_PTR(err);
		if (unlikely(err))
			goto out_dir;
		ndx.nd = NULL;
		ndx.br = NULL;
	}

	if (dt)
		au_dtime_store(dt, parent, h_parent, hgdir);
	wh_dentry = NULL;
	if (/* bcpup != bstart || */ bcpup != au_dbwh(dentry))
		goto out; /* success */

	wh_dentry = au_wh_lkup(h_parent, &dentry->d_name, &ndx);

 out_dir:
	if (IS_ERR(wh_dentry))
		au_hdir_unlock(h_dir, dir, bcpup);
 out:
	AuTraceErrPtr(wh_dentry);
	return wh_dentry;
}

/* ---------------------------------------------------------------------- */

enum { Mknod, Symlink, Creat };
struct simple_arg {
	int type;
	union {
		struct {
			int mode;
			struct nameidata *nd;
		} c;
		struct {
			const char *symname;
		} s;
		struct {
			int mode;
			dev_t dev;
		} m;
	} u;
};

static int add_simple(struct inode *dir, struct dentry *dentry,
		      struct simple_arg *arg)
{
	int err, dlgt, created;
	struct dentry *h_dentry, *h_parent, *wh_dentry, *parent;
	struct inode *h_dir;
	struct au_dtime dt;
	struct vfsub_args vargs;
	struct super_block *sb;
	aufs_bindex_t bstart;
	struct au_wr_dir_args wr_dir_args = {
		.force_btgt	= -1,
		.flags		= AuWrDir_ADD_ENTRY
	};

	LKTRTrace("type %d, %.*s\n", arg->type, AuDLNPair(dentry));
	IMustLock(dir);

	aufs_read_lock(dentry, AuLock_DW);
	parent = dentry->d_parent; /* dir inode is locked */
	di_write_lock_parent(parent);
	wh_dentry = lock_hdir_lkup_wh(dentry, &dt, /*src_dentry*/NULL,
				      &wr_dir_args);
	err = PTR_ERR(wh_dentry);
	if (IS_ERR(wh_dentry))
		goto out;

	bstart = au_dbstart(dentry);
	h_dentry = au_h_dptr(dentry, bstart);
	h_parent = h_dentry->d_parent; /* dir inode is locked */
	h_dir = h_parent->d_inode;
	IMustLock(h_dir);
	sb = dir->i_sb;
	dlgt = !!au_test_dlgt(au_mntflags(sb));

	switch (arg->type) {
	case Creat:
		AuDebugOn(au_test_nfs(h_dir->i_sb) && !arg->u.c.nd);
		err = au_h_create(h_dir, h_dentry, arg->u.c.mode, dlgt,
				  arg->u.c.nd, au_nfsmnt(sb, bstart));
		break;
	case Symlink:
		err = vfsub_symlink(h_dir, h_dentry, arg->u.s.symname,
				    S_IALLUGO, dlgt);
		break;
	case Mknod:
		err = vfsub_mknod(h_dir, h_dentry, arg->u.m.mode, arg->u.m.dev,
				  dlgt);
		break;
	default:
		BUG();
	}
	created = !err;
	if (!err)
		err = epilog(dir, wh_dentry, dentry);

	/* revert */
	if (unlikely(created && err && h_dentry->d_inode)) {
		int rerr;
		vfsub_args_init(&vargs, NULL, dlgt, 0);
		rerr = vfsub_unlink(h_dir, h_dentry, &vargs);
		if (rerr) {
			AuIOErr("%.*s revert failure(%d, %d)\n",
				AuDLNPair(dentry), err, rerr);
			err = -EIO;
		}
		/* todo: inotify will be fired to the grand parent dir */
		au_dtime_revert(&dt);
		d_drop(dentry);
	}

	au_hdir_unlock(h_dir, dir, bstart);
	dput(wh_dentry);

 out:
	if (unlikely(err)) {
		au_update_dbstart(dentry);
		d_drop(dentry);
	}
	di_write_unlock(parent);
	aufs_read_unlock(dentry, AuLock_DW);
	AuTraceErr(err);
	return err;
}

int aufs_mknod(struct inode *dir, struct dentry *dentry, int mode, dev_t dev)
{
	struct simple_arg arg = {
		.type = Mknod,
		.u.m = {
			.mode	= mode,
			.dev	= dev
		}
	};
	return add_simple(dir, dentry, &arg);
}

int aufs_symlink(struct inode *dir, struct dentry *dentry, const char *symname)
{
	struct simple_arg arg = {
		.type = Symlink,
		.u.s.symname = symname
	};
	return add_simple(dir, dentry, &arg);
}

int aufs_create(struct inode *dir, struct dentry *dentry, int mode,
		struct nameidata *nd)
{
	struct simple_arg arg = {
		.type = Creat,
		.u.c = {
			.mode	= mode,
			.nd	= nd
		}
	};
	return add_simple(dir, dentry, &arg);
}

/* ---------------------------------------------------------------------- */

struct link_arg {
	aufs_bindex_t bdst, bsrc;
	int issamedir, dlgt;
	struct dentry *src_parent, *parent, *h_dentry;
	struct inode *h_dir, *inode, *dir;
};

static int cpup_before_link(struct dentry *src_dentry, struct inode *dir,
			    struct link_arg *a)
{
	int err;
	struct inode *hi, *h_dir, *src_dir, *gdir;
	struct dentry *gparent;

	AuTraceEnter();

	gparent = NULL;
	gdir = NULL;
	if (unlikely(au_opt_test(au_mntflags(src_dentry->d_sb), UDBA_INOTIFY)
		     && !IS_ROOT(a->src_parent))) {
		gparent = dget_parent(a->src_parent);
		gdir = gparent->d_inode;
		if (gdir == dir) {
			dput(gparent);
			gparent = NULL;
		}
	}
	src_dir = a->src_parent->d_inode;
	h_dir = NULL;

	if (!a->issamedir) {
		/* this temporary unlock/lock is safe */
		au_hdir_unlock(a->h_dir, dir, a->bdst);
		di_read_lock_parent2(a->src_parent, AuLock_IR);
		err = au_test_and_cpup_dirs(src_dentry, a->bdst, a->parent);
		if (unlikely(err)) {
			au_hdir_lock(a->h_dir, dir, a->bdst);
			goto out;
		}

		/* todo: meaningless lock if CONFIG_AUFS_DEBUG is disabled. */
		if (unlikely(gparent))
			ii_read_lock_parent3(gdir);
		h_dir = au_h_iptr(src_dir, a->bdst);
		au_hdir_lock(h_dir, src_dir, a->bdst);
	} else if (unlikely(gparent)) {
		/* this temporary unlock/lock is safe */
		au_hdir_unlock(a->h_dir, dir, a->bdst);
		ii_read_lock_parent3(gdir);
		au_hdir_lock(a->h_dir, dir, a->bdst);
	}
	/* todo: test parent-gparent relationship? */

	AuDebugOn(au_dbstart(src_dentry) != a->bsrc);
	hi = au_h_dptr(src_dentry, a->bsrc)->d_inode;
	mutex_lock_nested(&hi->i_mutex, AuLsc_I_CHILD);
	err = au_sio_cpup_simple(src_dentry, a->bdst, -1, AuCpup_DTIME);
	mutex_unlock(&hi->i_mutex);

	if (unlikely(gparent)) {
		ii_read_unlock(gdir);
		dput(gparent);
	}

 out:
	if (h_dir) {
		au_hdir_unlock(h_dir, src_dir, a->bdst);
		au_hdir_lock(a->h_dir, dir, a->bdst);
	}
	if (!a->issamedir)
		di_read_unlock(a->src_parent, AuLock_IR);

	AuTraceErr(err);
	return err;
}

static int cpup_or_link(struct dentry *src_dentry, struct link_arg *a)
{
	int err;
	struct inode *inode, *h_inode, *h_dst_inode;
	struct dentry *h_dentry;
	aufs_bindex_t bstart;
	struct super_block *sb;

	AuTraceEnter();

	sb = src_dentry->d_sb;
	inode = src_dentry->d_inode;
	AuDebugOn(au_dbstart(src_dentry) != a->bsrc);
	h_dentry = au_h_dptr(src_dentry, a->bsrc);
	h_inode = h_dentry->d_inode;
	bstart = au_ibstart(inode);
	h_dst_inode = NULL;
	if (bstart <= a->bdst)
		h_dst_inode = au_h_iptr(inode, a->bdst);

	if (!h_dst_inode || !h_dst_inode->i_nlink) {
		/* copyup src_dentry as the name of dentry. */
		au_set_dbstart(src_dentry, a->bdst);
		au_set_h_dptr(src_dentry, a->bdst, dget(a->h_dentry));
		mutex_lock_nested(&h_inode->i_mutex, AuLsc_I_CHILD);
		err = au_sio_cpup_single(src_dentry, a->bdst, a->bsrc, -1,
					 !AuCpup_DTIME);
		mutex_unlock(&h_inode->i_mutex);
		au_set_h_dptr(src_dentry, a->bdst, NULL);
		au_set_dbstart(src_dentry, a->bsrc);
	} else {
		/* the inode of src_dentry already exists on a.bdst branch */
		h_dentry = d_find_alias(h_dst_inode);
		if (h_dentry) {
			err = vfsub_link(h_dentry, a->h_dir,
					 a->h_dentry, a->dlgt);
			dput(h_dentry);
		} else {
			AuIOErr("no dentry found for i%lu on b%d\n",
				h_dst_inode->i_ino, a->bdst);
			err = -EIO;
		}
	}

	if (!err)
		au_plink_append(sb, a->inode, a->h_dentry, a->bdst);

	AuTraceErr(err);
	return err;
}

int aufs_link(struct dentry *src_dentry, struct inode *dir,
	      struct dentry *dentry)
{
	int err, rerr;
	struct dentry *h_parent, *wh_dentry, *h_src_dentry;
	struct au_dtime dt;
	struct link_arg a;
	struct super_block *sb;
	unsigned int mnt_flags;
	struct vfsub_args vargs;
	struct au_wr_dir_args wr_dir_args = {
		/* .force_btgt	= -1, */
		.flags		= AuWrDir_ADD_ENTRY
	};

	LKTRTrace("src %.*s, i%lu, dst %.*s\n",
		  AuDLNPair(src_dentry), dir->i_ino, AuDLNPair(dentry));
	IMustLock(dir);
	IMustLock(src_dentry->d_inode);
	AuDebugOn(S_ISDIR(src_dentry->d_inode->i_mode));

	aufs_read_and_write_lock2(dentry, src_dentry, /*flags*/0);
	sb = dentry->d_sb;
	a.dir = dir;
	a.src_parent = dget_parent(src_dentry);
	a.parent = dentry->d_parent; /* dir inode is locked */
	a.issamedir = (a.src_parent == a.parent);
	if (!a.issamedir)
		au_fset_wrdir(wr_dir_args.flags, LOCK_SRCDIR);
	wr_dir_args.force_btgt = au_dbstart(src_dentry);
	di_write_lock_parent(a.parent);
	wr_dir_args.force_btgt = au_wbr(dentry, wr_dir_args.force_btgt);
	wh_dentry = lock_hdir_lkup_wh(dentry, &dt, src_dentry, &wr_dir_args);
	err = PTR_ERR(wh_dentry);
	if (IS_ERR(wh_dentry))
		goto out;

	a.inode = src_dentry->d_inode;
	a.bdst = au_dbstart(dentry);
	a.h_dentry = au_h_dptr(dentry, a.bdst);
	h_parent = a.h_dentry->d_parent; /* dir inode is locked */
	a.h_dir = h_parent->d_inode;
	IMustLock(a.h_dir);

	err = 0;
	mnt_flags = au_mntflags(sb);
	a.dlgt = !!au_test_dlgt(mnt_flags);

	/* todo: minor optimize,
	   their sb may be same while their bindex differs? */
	a.bsrc = au_dbstart(src_dentry);
	h_src_dentry = au_h_dptr(src_dentry, a.bsrc);
	if (unlikely(!au_opt_test(mnt_flags, PLINK))) {
		/*
		 * copyup src_dentry to the branch we process,
		 * and then link(2) to it.
		 * gave up 'pseudo link by cpup' approach,
		 * since nlink may be one and some applications will not work.
		 */
		if (a.bdst < a.bsrc
		    /* && h_src_dentry->d_sb != a.h_dentry->d_sb */)
			err = cpup_before_link(src_dentry, dir, &a);
		if (!err) {
			h_src_dentry = au_h_dptr(src_dentry, a.bdst);
			err = vfsub_link(h_src_dentry, a.h_dir, a.h_dentry,
					 a.dlgt);
		}
	} else {
		if (a.bdst < a.bsrc
		    /* && h_src_dentry->d_sb != a.h_dentry->d_sb */)
			err = cpup_or_link(src_dentry, &a);
		else {
			h_src_dentry = au_h_dptr(src_dentry, a.bdst);
			err = vfsub_link(h_src_dentry, a.h_dir, a.h_dentry,
					 a.dlgt);
		}
	}
	if (unlikely(err))
		goto out_unlock;
	if (wh_dentry) {
		err = au_wh_unlink_dentry(a.h_dir, wh_dentry, dentry,
					  dir, /*dlgt*/0);
		if (unlikely(err))
			goto out_revert;
	}

#if 0 /* cannot support it */
	/* fuse has different memory inode for the same inode number */
	if (unlikely(au_test_fuse(a.h_dentry->d_sb))) {
		LKTRLabel(here);
		d_drop(a.h_dentry);
		/*d_drop(h_src_dentry);
		  d_drop(src_dentry);*/
		inc_nlink(a.inode);
		a.inode->i_ctime = dir->i_ctime;
	}
#endif

	dir->i_version++;
	if (au_ibstart(dir) == au_dbstart(dentry))
		au_cpup_attr_timesizes(dir);
	if (!d_unhashed(a.h_dentry)
	    /* || h_old_inode->i_nlink <= nlink */
	    /* || SB_NFS(h_src_dentry->d_sb) */) {
		dentry->d_inode = igrab(a.inode);
		d_instantiate(dentry, a.inode);
		inc_nlink(a.inode);
		a.inode->i_ctime = dir->i_ctime;
	} else
		/* nfs case (< 2.6.15) */
		d_drop(dentry);
	goto out_unlock; /* success */

 out_revert:
	vfsub_args_init(&vargs, NULL, a.dlgt, 0);
	rerr = vfsub_unlink(a.h_dir, a.h_dentry, &vargs);
	if (!rerr)
		goto out_dt;
	AuIOErr("%.*s reverting failed(%d, %d)\n",
		AuDLNPair(dentry), err, rerr);
	err = -EIO;
 out_dt:
	d_drop(dentry);
	au_dtime_revert(&dt);
 out_unlock:
	au_hdir_unlock(a.h_dir, dir, a.bdst);
	dput(wh_dentry);
 out:
	if (unlikely(err)) {
		au_update_dbstart(dentry);
		d_drop(dentry);
	}
	di_write_unlock(a.parent);
	dput(a.src_parent);
	aufs_read_and_write_unlock2(dentry, src_dentry);
	AuTraceErr(err);
	return err;
}

int aufs_mkdir(struct inode *dir, struct dentry *dentry, int mode)
{
	int err, rerr, diropq, dlgt;
	struct dentry *h_dentry, *h_parent, *wh_dentry, *parent, *opq_dentry;
	struct inode *h_dir, *h_inode;
	struct au_dtime dt;
	aufs_bindex_t bindex;
	struct super_block *sb;
	unsigned int mnt_flags;
	struct vfsub_args vargs;
	struct au_wr_dir_args wr_dir_args = {
		.force_btgt	= -1,
		.flags		= AuWrDir_ADD_ENTRY | AuWrDir_ISDIR
	};

	LKTRTrace("i%lu, %.*s, mode 0%o\n",
		  dir->i_ino, AuDLNPair(dentry), mode);
	IMustLock(dir);

	aufs_read_lock(dentry, AuLock_DW);
	parent = dentry->d_parent; /* dir inode is locked */
	di_write_lock_parent(parent);
	wh_dentry = lock_hdir_lkup_wh(dentry, &dt, /*src_dentry*/NULL,
				      &wr_dir_args);
	err = PTR_ERR(wh_dentry);
	if (IS_ERR(wh_dentry))
		goto out;

	sb = dentry->d_sb;
	bindex = au_dbstart(dentry);
	h_dentry = au_h_dptr(dentry, bindex);
	h_parent = h_dentry->d_parent; /* dir inode is locked */
	h_dir = h_parent->d_inode;
	IMustLock(h_dir);
	mnt_flags = au_mntflags(sb);
	dlgt = !!au_test_dlgt(mnt_flags);

	err = vfsub_mkdir(h_dir, h_dentry, mode, dlgt);
	if (unlikely(err))
		goto out_unlock;
	h_inode = h_dentry->d_inode;

	/* make the dir opaque */
	diropq = 0;
	if (wh_dentry || au_opt_test(mnt_flags, ALWAYS_DIROPQ)) {
		mutex_lock_nested(&h_inode->i_mutex, AuLsc_I_CHILD);
		opq_dentry = au_diropq_create(dentry, bindex, /*dlgt*/0);
		mutex_unlock(&h_inode->i_mutex);
		err = PTR_ERR(opq_dentry);
		if (IS_ERR(opq_dentry))
			goto out_dir;
		dput(opq_dentry);
		diropq = 1;
	}

	err = epilog(dir, wh_dentry, dentry);
	if (!err) {
		inc_nlink(dir);
		goto out_unlock; /* success */
	}

	/* revert */
	if (diropq) {
		LKTRLabel(revert opq);
		mutex_lock_nested(&h_inode->i_mutex, AuLsc_I_CHILD);
		rerr = au_diropq_remove(dentry, bindex, dlgt);
		mutex_unlock(&h_inode->i_mutex);
		if (rerr) {
			AuIOErr("%.*s reverting diropq failed(%d, %d)\n",
				AuDLNPair(dentry), err, rerr);
			err = -EIO;
		}
	}

 out_dir:
	LKTRLabel(revert dir);
	vfsub_args_init(&vargs, NULL, dlgt, 0);
	rerr = vfsub_rmdir(h_dir, h_dentry, &vargs);
	if (rerr) {
		AuIOErr("%.*s reverting dir failed(%d, %d)\n",
			AuDLNPair(dentry), err, rerr);
		err = -EIO;
	}
	d_drop(dentry);
	au_dtime_revert(&dt);
 out_unlock:
	au_hdir_unlock(h_dir, dir, bindex);
	dput(wh_dentry);
 out:
	if (unlikely(err)) {
		au_update_dbstart(dentry);
		d_drop(dentry);
	}
	di_write_unlock(parent);
	aufs_read_unlock(dentry, AuLock_DW);
	AuTraceErr(err);
	return err;
}
