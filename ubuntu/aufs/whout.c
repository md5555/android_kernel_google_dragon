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
 * whiteout for logical deletion and opaque directory
 *
 * $Id: whout.c,v 1.6 2008/06/02 02:38:22 sfjro Exp $
 */

#include <linux/fs.h>
#include <linux/namei.h>
#include "aufs.h"

#define WH_MASK			S_IRUGO

/* If a directory contains this file, then it is opaque.  We start with the
 * .wh. flag so that it is blocked by lookup.
 */
static struct qstr diropq_name = {
	.name = AUFS_WH_DIROPQ,
	.len = sizeof(AUFS_WH_DIROPQ) - 1
};

/*
 * generate whiteout name, which is NOT terminated by NULL.
 * @name: original d_name.name
 * @len: original d_name.len
 * @wh: whiteout qstr
 * returns zero when succeeds, otherwise error.
 * succeeded value as wh->name should be freed by au_wh_name_free().
 */
int au_wh_name_alloc(const char *name, int len, struct qstr *wh)
{
	char *p;

	AuDebugOn(!name || !len || !wh);

	if (unlikely(len > PATH_MAX - AUFS_WH_PFX_LEN))
		return -ENAMETOOLONG;

	wh->len = len + AUFS_WH_PFX_LEN;
	p = kmalloc(wh->len, GFP_KERNEL);
	wh->name = p;
	if (p) {
		memcpy(p, AUFS_WH_PFX, AUFS_WH_PFX_LEN);
		memcpy(p + AUFS_WH_PFX_LEN, name, len);
		/* smp_mb(); */
		return 0;
	}
	return -ENOMEM;
}

void au_wh_name_free(struct qstr *wh)
{
	AuDebugOn(!wh || !wh->name);
	kfree(wh->name);
}

/* ---------------------------------------------------------------------- */

/*
 * test if the @wh_name exists under @h_parent.
 * @try_sio specifies the necessary of super-io.
 */
int au_wh_test(struct dentry *h_parent, struct qstr *wh_name, int try_sio,
	       struct au_ndx *ndx)
{
	int err;
	struct dentry *wh_dentry;
	struct inode *h_dir;
	unsigned int flags;

	LKTRTrace("%.*s/%.*s, ndx{%p, 0x%x}\n", AuDLNPair(h_parent),
		  wh_name->len, wh_name->name, ndx->nfsmnt, ndx->flags);
	h_dir = h_parent->d_inode;
	AuDebugOn(!S_ISDIR(h_dir->i_mode));

	flags = 0;
	if (ndx && ndx->nd) {
		flags = ndx->nd->flags;
		ndx->nd->flags &= ~(LOOKUP_OPEN | LOOKUP_CREATE);
	}

	if (!try_sio)
		wh_dentry = au_lkup_one(wh_name->name, h_parent,
					wh_name->len, ndx);
	else
		wh_dentry = au_sio_lkup_one(wh_name->name, h_parent,
					    wh_name->len, ndx);
	if (flags)
		ndx->nd->flags = flags;
	err = PTR_ERR(wh_dentry);
	if (IS_ERR(wh_dentry))
		goto out;

	err = 0;
	if (!wh_dentry->d_inode)
		goto out_wh; /* success */

	err = 1;
	if (S_ISREG(wh_dentry->d_inode->i_mode))
		goto out_wh; /* success */

	err = -EIO;
	AuIOErr("%.*s Invalid whiteout entry type 0%o.\n",
		AuDLNPair(wh_dentry), wh_dentry->d_inode->i_mode);

 out_wh:
	dput(wh_dentry);
 out:
	AuTraceErr(err);
	return err;
}

/*
 * test if the @h_dentry sets opaque or not.
 */
int au_diropq_test(struct dentry *h_dentry, struct au_ndx *ndx)
{
	int err, try_sio;
	struct inode *h_dir;

	LKTRTrace("dentry %.*s\n", AuDLNPair(h_dentry));
	h_dir = h_dentry->d_inode;
	AuDebugOn(!S_ISDIR(h_dir->i_mode));

	try_sio = au_test_h_perm_sio(h_dir, MAY_EXEC,
				     au_ftest_ndx(ndx->flags, DLGT));
	err = au_wh_test(h_dentry, &diropq_name, try_sio, ndx);
	AuTraceErr(err);
	return err;
}

/*
 * returns a negative dentry whose name is unique and temporary.
 */
struct dentry *au_whtmp_lkup(struct dentry *h_parent, struct qstr *prefix,
			     struct au_ndx *ndx)
{
#define HEX_LEN 4
	struct dentry *dentry;
	int len, i;
	char defname[AUFS_WH_PFX_LEN * 2 + DNAME_INLINE_LEN_MIN + 1
		     + HEX_LEN + 1], *name, *p;
	static unsigned char cnt;

	LKTRTrace("hp %.*s, prefix %.*s\n",
		  AuDLNPair(h_parent), prefix->len, prefix->name);
	AuDebugOn(!h_parent->d_inode);

	name = defname;
	len = sizeof(defname) - DNAME_INLINE_LEN_MIN + prefix->len - 1;
	if (unlikely(prefix->len > DNAME_INLINE_LEN_MIN)) {
		dentry = ERR_PTR(-ENAMETOOLONG);
		if (unlikely(len >= PATH_MAX))
			goto out;
		dentry = ERR_PTR(-ENOMEM);
		name = kmalloc(len + 1, GFP_KERNEL);
		if (unlikely(!name))
			goto out;
	}

	/* doubly whiteout-ed */
	memcpy(name, AUFS_WH_PFX AUFS_WH_PFX, AUFS_WH_PFX_LEN * 2);
	p = name + AUFS_WH_PFX_LEN * 2;
	memcpy(p, prefix->name, prefix->len);
	p += prefix->len;
	*p++ = '.';
	AuDebugOn(name + len + 1 - p <= HEX_LEN);

	for (i = 0; i < 3; i++) {
		sprintf(p, "%.*d", HEX_LEN, cnt++);
		dentry = au_sio_lkup_one(name, h_parent, len, ndx);
		if (IS_ERR(dentry) || !dentry->d_inode)
			goto out_name;
		dput(dentry);
	}
	/* AuWarn("could not get random name\n"); */
	dentry = ERR_PTR(-EEXIST);
	AuDbg("%.*s\n", len, name);
	BUG();

 out_name:
	if (unlikely(name != defname))
		kfree(name);
 out:
	AuTraceErrPtr(dentry);
	return dentry;
#undef HEX_LEN
}

/*
 * rename the @dentry of @bindex to the whiteouted temporary name.
 */
int au_whtmp_ren(struct inode *dir, struct dentry *dentry, aufs_bindex_t bindex,
		 int noself)
{
	int err, dlgt;
	struct inode *h_dir;
	struct dentry *h_dentry, *h_parent, *tmp_dentry;
	struct super_block *sb;
	unsigned int mnt_flags;
	struct au_hin_ignore ign;
	struct vfsub_args vargs;
	struct au_ndx ndx = {
		.flags	= 0,
		.nd	= NULL,
		/* .br	= NULL */
	};

	LKTRTrace("%.*s, b%d\n", AuDLNPair(dentry), bindex);
	h_dentry = au_h_dptr(dentry, bindex);
	AuDebugOn(!h_dentry || !h_dentry->d_inode);
	h_parent = h_dentry->d_parent; /* dir inode is locked */
	h_dir = h_parent->d_inode;
	IMustLock(h_dir);

	sb = dentry->d_sb;
	mnt_flags = au_mntflags(sb);
	dlgt = !!au_test_dlgt(mnt_flags);
	if (unlikely(dlgt))
		au_fset_ndx(ndx.flags, DLGT);
	ndx.nfsmnt = au_nfsmnt(sb, bindex);
	tmp_dentry = au_whtmp_lkup(h_parent, &h_dentry->d_name, &ndx);
	err = PTR_ERR(tmp_dentry);
	if (!IS_ERR(tmp_dentry)) {
		/* under the same dir, no need to lock_rename() */
		vfsub_args_init(&vargs, &ign, dlgt, 0);
		AuDebugOn(!S_ISDIR(dentry->d_inode->i_mode));
		if (unlikely(au_opt_test(mnt_flags, UDBA_INOTIFY) && !noself))
			vfsub_ign_hinode(&vargs, IN_MOVE_SELF,
					 au_hi(dentry->d_inode, bindex));
		err = vfsub_rename(h_dir, h_dentry, h_dir, tmp_dentry, &vargs);
		AuTraceErr(err);
		dput(tmp_dentry);
	}

	AuTraceErr(err);
	return err;
}

/* ---------------------------------------------------------------------- */

static int do_unlink_wh(struct inode *h_dir, struct dentry *wh_dentry,
			struct inode *dir, int dlgt)
{
	struct vfsub_args vargs;

	LKTRTrace("hi%lu, wh %.*s\n", h_dir->i_ino, AuDLNPair(wh_dentry));
	AuDebugOn(!wh_dentry->d_inode || !S_ISREG(wh_dentry->d_inode->i_mode));

	/*
	 * forces superio when the dir has a sticky bit.
	 * this may be a violation of unix fs semantics.
	 */
	vfsub_args_init(&vargs, NULL, dlgt,
			(h_dir->i_mode & S_ISVTX)
			&& wh_dentry->d_inode->i_uid != current->fsuid);
	return vfsub_unlink(h_dir, wh_dentry, &vargs);
}

int au_wh_unlink_dentry(struct inode *h_dir, struct dentry *wh_dentry,
			struct dentry *dentry, struct inode *dir, int dlgt)
{
	int err;

	LKTRTrace("hi%lu, wh %.*s, d %p\n", h_dir->i_ino,
		  AuDLNPair(wh_dentry), dentry);
	AuDebugOn((dentry && au_dbwh(dentry) < 0)
		  || !wh_dentry->d_inode
		  || !S_ISREG(wh_dentry->d_inode->i_mode));

	err = do_unlink_wh(h_dir, wh_dentry, dir, dlgt);
	if (!err && dentry)
		au_set_dbwh(dentry, -1);

	AuTraceErr(err);
	return err;
}

static int unlink_wh_name(struct dentry *h_parent, struct qstr *wh,
			  struct inode *dir, struct au_ndx *ndx)
{
	int err;
	struct inode *h_dir;
	struct dentry *h_dentry;

	LKTRTrace("%.*s/%.*s\n", AuDLNPair(h_parent), AuLNPair(wh));
	h_dir = h_parent->d_inode;

	/* au_test_h_perm() is already done */
	h_dentry = au_lkup_one(wh->name, h_parent, wh->len, ndx);
	if (!IS_ERR(h_dentry)) {
		err = 0;
		if (h_dentry->d_inode && S_ISREG(h_dentry->d_inode->i_mode))
			err = do_unlink_wh(h_dir, h_dentry, dir,
					   au_ftest_ndx(ndx->flags, DLGT));
		dput(h_dentry);
	} else
		err = PTR_ERR(h_dentry);

	AuTraceErr(err);
	return err;
}

/* ---------------------------------------------------------------------- */

static void clean_wh(struct inode *h_dir, struct dentry *wh)
{
	int err;
	struct vfsub_args vargs;

	AuTraceEnter();

	if (wh->d_inode) {
		vfsub_args_init(&vargs, NULL, 0, 0);
		err = vfsub_unlink(h_dir, wh, &vargs);
		if (unlikely(err))
			AuWarn("failed unlink %.*s (%d), ignored.\n",
			       AuDLNPair(wh), err);
	}
}

static void clean_plink(struct inode *h_dir, struct dentry *plink)
{
	int err;
	struct vfsub_args vargs;

	AuTraceEnter();

	if (plink->d_inode) {
		vfsub_args_init(&vargs, NULL, 0, 0);
		err = vfsub_rmdir(h_dir, plink, &vargs);
		if (unlikely(err))
			AuWarn("failed rmdir %.*s (%d), ignored.\n",
			       AuDLNPair(plink), err);
	}
}

static int test_linkable(struct inode *h_dir)
{
	if (h_dir->i_op && h_dir->i_op->link)
		return 0;
	return -ENOSYS;
}

static int plink_dir(struct inode *h_dir, struct dentry *plink)
{
	int err;

	err = -EEXIST;
	if (!plink->d_inode) {
		int mode = S_IRWXU;
		if (unlikely(au_test_nfs(plink->d_sb)))
			mode |= S_IXUGO;
		err = vfsub_mkdir(h_dir, plink, mode, /*dlgt*/0);
	} else if (S_ISDIR(plink->d_inode->i_mode))
		err = 0;
	else
		AuErr("unknown %.*s exists\n", AuDLNPair(plink));

	return err;
}

/*
 * initialize the whiteout base file/dir for @br.
 */
int au_wh_init(struct dentry *h_root, struct au_branch *br,
	       struct vfsmount *nfsmnt, struct super_block *sb)
{
	int err;
	struct dentry *wh, *plink;
	struct inode *h_dir;
	static struct qstr base_name[] = {
		{
			.name	= AUFS_WH_BASENAME,
			.len	= sizeof(AUFS_WH_BASENAME) - 1
		},
		{
			.name	= AUFS_WH_PLINKDIR,
			.len	= sizeof(AUFS_WH_PLINKDIR) - 1
		}
	};
	struct au_ndx ndx = {
		.nfsmnt	= nfsmnt,
		.flags	= 0, /* always no dlgt */
		.nd	= NULL,
		/* .br	= NULL */
	};
	const int do_plink = au_opt_test(au_mntflags(sb), PLINK);

	LKTRTrace("nfsmnt %p\n", nfsmnt);
	BrWhMustWriteLock(br);
	SiMustWriteLock(sb);
	h_dir = h_root->d_inode;

	/* doubly whiteouted */
	wh = au_wh_lkup(h_root, base_name + 0, &ndx);
	err = PTR_ERR(wh);
	if (IS_ERR(wh))
		goto out;
	AuDebugOn(br->br_wh && br->br_wh != wh);

	plink = au_wh_lkup(h_root, base_name + 1, &ndx);
	err = PTR_ERR(plink);
	if (IS_ERR(plink))
		goto out_dput_wh;
	AuDebugOn(br->br_plink && br->br_plink != plink);

	dput(br->br_wh);
	dput(br->br_plink);
	br->br_wh = NULL;
	br->br_plink = NULL;

	err = 0;
	switch (br->br_perm) {
	case AuBr_RR:
	case AuBr_RO:
	case AuBr_RRWH:
	case AuBr_ROWH:
		clean_wh(h_dir, wh);
		clean_plink(h_dir, plink);
		break;

	case AuBr_RWNoLinkWH:
		clean_wh(h_dir, wh);
		if (do_plink) {
			err = test_linkable(h_dir);
			if (unlikely(err))
				goto out_nolink;

			err = plink_dir(h_dir, plink);
			if (unlikely(err))
				goto out_err;
			br->br_plink = dget(plink);
		} else
			clean_plink(h_dir, plink);
		break;

	case AuBr_RW:
		/*
		 * for the moment, aufs supports the branch filesystem
		 * which does not support link(2).
		 * testing on FAT which does not support i_op->setattr() fully
		 * either, copyup failed.
		 * finally, such filesystem will not be used as the writable
		 * branch.
		 */
		err = test_linkable(h_dir);
		if (unlikely(err))
			goto out_nolink;

		err = -EEXIST;
		if (!wh->d_inode)
			err = au_h_create(h_dir, wh, WH_MASK, /*dlgt*/0,
					  /*nd*/NULL, nfsmnt);
		else if (S_ISREG(wh->d_inode->i_mode))
			err = 0;
		else
			AuErr("unknown %.*s/%.*s exists\n",
			      AuDLNPair(h_root), AuDLNPair(wh));
		if (unlikely(err))
			goto out_err;

		if (do_plink) {
			err = plink_dir(h_dir, plink);
			if (unlikely(err))
				goto out_err;
			br->br_plink = dget(plink);
		} else
			clean_plink(h_dir, plink);
		br->br_wh = dget(wh);
		break;

	default:
		BUG();
	}

 out_dput:
	dput(plink);
 out_dput_wh:
	dput(wh);
 out:
	AuTraceErr(err);
	return err;
 out_nolink:
	AuErr("%.*s doesn't support link(2), use noplink and rw+nolwh\n",
	      AuDLNPair(h_root));
	goto out_dput;
 out_err:
	AuErr("an error(%d) on the writable branch %.*s(%s)\n",
	      err, AuDLNPair(h_root), au_sbtype(h_root->d_sb));
	goto out_dput;
}

struct reinit_br_wh {
	struct super_block *sb;
	struct au_branch *br;
};

static void reinit_br_wh(void *arg)
{
	int err;
	struct reinit_br_wh *a = arg;
	struct inode *h_dir, *dir;
	struct dentry *h_root;
	aufs_bindex_t bindex;
	struct vfsub_args vargs;

	AuTraceEnter();
	AuDebugOn(!a->br->br_wh || !a->br->br_wh->d_inode || current->fsuid);

	err = 0;
	/* big aufs lock */
	si_write_lock(a->sb);
	if (unlikely(!au_br_writable(a->br->br_perm)))
		goto out;
	bindex = au_br_index(a->sb, a->br->br_id);
	if (unlikely(bindex < 0))
		goto out;

	dir = a->sb->s_root->d_inode;
	h_root = dget_parent(a->br->br_wh);
	h_dir = h_root->d_inode;
	AuDebugOn(!h_dir->i_op || !h_dir->i_op->link);
	vfsub_args_init(&vargs, NULL, /*dlgt*/0, 0);
	au_hdir_lock(h_dir, dir, bindex);
	/* todo: revalidate h_wh? */
	br_wh_write_lock(a->br);
	err = vfsub_unlink(h_dir, a->br->br_wh, &vargs);
	dput(a->br->br_wh);
	a->br->br_wh = NULL;
	if (!err)
		err = au_wh_init(h_root, a->br, au_do_nfsmnt(a->br->br_mnt),
				 a->sb);
	br_wh_write_unlock(a->br);
	au_hdir_unlock(h_dir, dir, bindex);
	dput(h_root);

 out:
	atomic_dec_return(&a->br->br_wh_running);
	au_br_put(a->br);
	si_write_unlock(a->sb);
	kfree(arg);
	if (unlikely(err))
		AuIOErr("err %d\n", err);
}

static void kick_reinit_br_wh(struct super_block *sb, struct au_branch *br)
{
	int do_dec, wkq_err;
	struct reinit_br_wh *arg;

	do_dec = 1;
	if (atomic_inc_return(&br->br_wh_running) != 1)
		goto out;

	/* ignore ENOMEM */
	arg = kmalloc(sizeof(*arg), GFP_TEMPORARY);
	if (arg) {
		/*
		 * dec(wh_running), kfree(arg) and au_br_put()
		 * in reinit function
		 */
		arg->sb = sb;
		arg->br = br;
		au_br_get(br);
		wkq_err = au_wkq_nowait(reinit_br_wh, arg, sb, /*dlgt*/0);
		if (unlikely(wkq_err)) {
			atomic_dec_return(&br->br_wh_running);
			au_br_put(br);
			kfree(arg);
		}
		do_dec = 0;
	}

 out:
	if (do_dec)
		atomic_dec_return(&br->br_wh_running);
}

/*
 * create the whiteout @wh.
 */
static int link_or_create_wh(struct dentry *wh, struct super_block *sb,
			     aufs_bindex_t bindex, struct inode *dir)
{
	int err, dlgt;
	struct au_branch *br;
	struct dentry *h_parent;
	struct inode *h_dir;

	LKTRTrace("%.*s\n", AuDLNPair(wh));
	SiMustReadLock(sb);
	h_parent = wh->d_parent; /* dir inode is locked */
	h_dir = h_parent->d_inode;
	IMustLock(h_dir);

	dlgt = !!au_test_dlgt(au_mntflags(sb));
	br = au_sbr(sb, bindex);
	br_wh_read_lock(br);
	if (br->br_wh) {
		err = vfsub_link(br->br_wh, h_dir, wh, dlgt);
		if (!err || err != -EMLINK)
			goto out;

		/* link count full. re-initialize br_wh. */
		kick_reinit_br_wh(sb, br);
	}

	/* return this error in this context */
	err = au_h_create(h_dir, wh, WH_MASK, dlgt, /*nd*/NULL,
			  au_do_nfsmnt(br->br_mnt));

 out:
	br_wh_read_unlock(br);
	AuTraceErr(err);
	return err;
}

/* ---------------------------------------------------------------------- */

/*
 * create or remove the diropq.
 */
static struct dentry *do_diropq(struct dentry *dentry, aufs_bindex_t bindex,
				unsigned int flags)
{
	struct dentry *opq_dentry, *h_dentry;
	struct inode *h_dir;
	int err, dlgt;
	struct super_block *sb;
	struct au_ndx ndx = {
		.flags	= 0,
		.nd	= NULL,
		/* .br	= NULL */
	};

	LKTRTrace("%.*s, bindex %d, flags 0x%x\n",
		  AuDLNPair(dentry), bindex, flags);
	h_dentry = au_h_dptr(dentry, bindex);
	AuDebugOn(!h_dentry);
	h_dir = h_dentry->d_inode;
	AuDebugOn(!h_dir || !S_ISDIR(h_dir->i_mode));

	/* already checked by au_test_h_perm(). */
	sb = dentry->d_sb;
	ndx.nfsmnt = au_nfsmnt(sb, bindex);
	dlgt = 0;
	if (unlikely(au_ftest_diropq(flags, DLGT))) {
		dlgt = 1;
		au_fset_ndx(ndx.flags, DLGT);
	}
	opq_dentry = au_lkup_one(diropq_name.name, h_dentry, diropq_name.len,
				 &ndx);
	if (IS_ERR(opq_dentry))
		goto out;

	if (au_ftest_diropq(flags, CREATE)) {
		AuDebugOn(opq_dentry->d_inode);
		err = link_or_create_wh(opq_dentry, sb, bindex,
					dentry->d_inode);
		if (!err) {
			au_set_dbdiropq(dentry, bindex);
			goto out; /* success */
		}
	} else {
		AuDebugOn(/* !S_ISDIR(dentry->d_inode->i_mode)
			   * ||  */!opq_dentry->d_inode);
		err = do_unlink_wh(h_dir, opq_dentry, dentry->d_inode, dlgt);
		if (!err)
			au_set_dbdiropq(dentry, -1);
	}
	dput(opq_dentry);
	opq_dentry = ERR_PTR(err);

 out:
	AuTraceErrPtr(opq_dentry);
	return opq_dentry;
}

struct do_diropq_args {
	struct dentry **errp;
	struct dentry *dentry;
	aufs_bindex_t bindex;
	unsigned int flags;
};

static void call_do_diropq(void *args)
{
	struct do_diropq_args *a = args;
	*a->errp = do_diropq(a->dentry, a->bindex, a->flags);
}

struct dentry *au_diropq_sio(struct dentry *dentry, aufs_bindex_t bindex,
			     unsigned int flags)
{
	struct dentry *diropq, *h_dentry;

	LKTRTrace("%.*s, bindex %d, flags 0x%x\n",
		  AuDLNPair(dentry), bindex, flags);

	h_dentry = au_h_dptr(dentry, bindex);
	if (!au_test_h_perm_sio(h_dentry->d_inode, MAY_EXEC | MAY_WRITE,
				au_ftest_diropq(flags, DLGT)))
		diropq = do_diropq(dentry, bindex, flags);
	else {
		int wkq_err;
		struct do_diropq_args args = {
			.errp		= &diropq,
			.dentry		= dentry,
			.bindex		= bindex,
			.flags		= flags
		};
		wkq_err = au_wkq_wait(call_do_diropq, &args, /*dlgt*/0);
		if (unlikely(wkq_err))
			diropq = ERR_PTR(wkq_err);
	}

	AuTraceErrPtr(diropq);
	return diropq;
}

/* ---------------------------------------------------------------------- */

/*
 * lookup whiteout dentry.
 * @h_parent: hidden parent dentry which must exist and be locked
 * @base_name: name of dentry which will be whiteouted
 * returns dentry for whiteout.
 */
struct dentry *au_wh_lkup(struct dentry *h_parent, struct qstr *base_name,
			  struct au_ndx *ndx)
{
	int err;
	struct qstr wh_name;
	struct dentry *wh_dentry;

	LKTRTrace("%.*s/%.*s\n", AuDLNPair(h_parent), AuLNPair(base_name));

	err = au_wh_name_alloc(base_name->name, base_name->len, &wh_name);
	wh_dentry = ERR_PTR(err);
	if (!err) {
		/* do not superio. */
		wh_dentry = au_lkup_one(wh_name.name, h_parent,
					wh_name.len, ndx);
		au_wh_name_free(&wh_name);
	}
	AuTraceErrPtr(wh_dentry);
	return wh_dentry;
}

/*
 * link/create a whiteout for @dentry on @bindex.
 */
struct dentry *au_wh_create(struct inode *dir, struct dentry *dentry,
			    aufs_bindex_t bindex, struct dentry *h_parent,
			    struct au_ndx *ndx)
{
	struct dentry *wh_dentry;
	int err;
	struct super_block *sb;

	LKTRTrace("%.*s/%.*s on b%d\n", AuDLNPair(h_parent),
		  AuDLNPair(dentry), bindex);

	sb = dentry->d_sb;
	wh_dentry = au_wh_lkup(h_parent, &dentry->d_name, ndx);
	if (!IS_ERR(wh_dentry) && !wh_dentry->d_inode) {
		err = link_or_create_wh(wh_dentry, sb, bindex, dir);
		if (!err)
			au_set_dbwh(dentry, bindex);
		else {
			dput(wh_dentry);
			wh_dentry = ERR_PTR(err);
		}
	}

	AuTraceErrPtr(wh_dentry);
	return wh_dentry;
}

/* ---------------------------------------------------------------------- */

/* Delete all whiteouts in this directory on branch bindex. */
static int del_wh_children(struct au_nhash *whlist, struct dentry *h_parent,
			   aufs_bindex_t bindex, struct inode *inode,
			   struct au_ndx *ndx)
{
	int err, i;
	struct qstr wh_name;
	char *p;
	struct inode *h_dir;
	struct hlist_head *head;
	struct au_vdir_wh *tpos;
	struct hlist_node *pos;
	struct au_vdir_destr *str;

	LKTRTrace("%.*s\n", AuDLNPair(h_parent));
	h_dir = h_parent->d_inode;
	AuDebugOn(IS_RDONLY(h_dir));

	err = -ENOMEM;
	p = __getname();
	wh_name.name = p;
	if (unlikely(!wh_name.name))
		goto out;
	memcpy(p, AUFS_WH_PFX, AUFS_WH_PFX_LEN);
	p += AUFS_WH_PFX_LEN;

	/* already checked by au_test_h_perm(). */
	err = 0;
	for (i = 0; !err && i < AuSize_NHASH; i++) {
		head = whlist->heads + i;
		hlist_for_each_entry(tpos, pos, head, wh_hash) {
			if (tpos->wh_bindex != bindex)
				continue;
			str = &tpos->wh_str;
			if (str->len + AUFS_WH_PFX_LEN <= PATH_MAX) {
				memcpy(p, str->name, str->len);
				wh_name.len = AUFS_WH_PFX_LEN + str->len;
				err = unlink_wh_name(h_parent, &wh_name, inode,
						     ndx);
				if (!err)
					continue;
				break;
			}
			AuIOErr("whiteout name too long %.*s\n",
				str->len, str->name);
			err = -EIO;
			break;
		}
	}
	__putname(wh_name.name);

 out:
	AuTraceErr(err);
	return err;
}

struct del_wh_children_args {
	int *errp;
	struct au_nhash *whlist;
	struct dentry *h_parent;
	aufs_bindex_t bindex;
	struct inode *inode;
	struct au_ndx *ndx;
};

static void call_del_wh_children(void *args)
{
	struct del_wh_children_args *a = args;
	*a->errp = del_wh_children(a->whlist, a->h_parent, a->bindex,
				   a->inode, a->ndx);
}

/* ---------------------------------------------------------------------- */

/*
 * rmdir the whiteouted temporary named dir @h_dentry.
 * @whlist: whiteouted children.
 */
int au_whtmp_rmdir(struct dentry *h_dentry, struct au_nhash *whlist,
		   aufs_bindex_t bindex, struct inode *dir, struct inode *inode,
		   int noself)
{
	int err, dlgt;
	struct inode *h_inode, *h_dir;
	struct super_block *sb;
	unsigned int mnt_flags;
	struct au_hin_ignore ign;
	struct vfsub_args vargs;
	struct au_ndx ndx = {
		.flags	= 0,
		.nd	= NULL,
		/* .br	= NULL */
	};

	LKTRTrace("hd %.*s, b%d, i%lu\n",
		  AuDLNPair(h_dentry), bindex, dir->i_ino);
	IMustLock(dir);
	IiMustAnyLock(dir);
	h_dir = h_dentry->d_parent->d_inode; /* dir inode is locked */
	IMustLock(h_dir);

	sb = inode->i_sb;
	mnt_flags = au_mntflags(sb);
	dlgt = !!au_test_dlgt(mnt_flags);
	if (unlikely(dlgt))
		au_fset_ndx(ndx.flags, DLGT);
	ndx.nfsmnt = au_nfsmnt(sb, bindex);
	h_inode = h_dentry->d_inode;
	AuDebugOn(h_inode != au_h_iptr(inode, bindex));
	au_hdir2_lock(h_inode, inode, bindex);

	/*
	 * someone else might change some whiteouts while we were sleeping.
	 * it means this whlist may have an obsoleted entry.
	 */
	if (!au_test_h_perm_sio(h_inode, MAY_EXEC | MAY_WRITE, dlgt))
		err = del_wh_children(whlist, h_dentry, bindex, inode, &ndx);
	else {
		int wkq_err;
		/* ugly */
		unsigned int flags = ndx.flags;
		struct del_wh_children_args args = {
			.errp		= &err,
			.whlist		= whlist,
			.h_parent	= h_dentry,
			.bindex		= bindex,
			.inode		= inode,
			.ndx		= &ndx
		};

		ndx.flags = 0;
		wkq_err = au_wkq_wait(call_del_wh_children, &args, /*dlgt*/0);
		if (unlikely(wkq_err))
			err = wkq_err;
		ndx.flags = flags;
	}
	au_hdir_unlock(h_inode, inode, bindex);

	if (!err) {
		vfsub_args_init(&vargs, &ign, dlgt, 0);
		if (unlikely(au_opt_test(mnt_flags, UDBA_INOTIFY) && !noself))
			vfsub_ign_hinode(&vargs, IN_DELETE_SELF,
					 au_hi(inode, bindex));
		err = vfsub_rmdir(h_dir, h_dentry, &vargs);
		/* d_drop(h_dentry); */
	}

	if (!err) {
		if (au_ibstart(dir) == bindex) {
			au_cpup_attr_timesizes(dir);
			/* au_cpup_attr_nlink(dir); */
			drop_nlink(dir);
		}
		return 0; /* success */
	}

	AuWarn("failed removing %.*s(%d), ignored\n", AuDLNPair(h_dentry), err);
	return err;
}

static void au_whtmp_rmdir_free_args(struct au_whtmp_rmdir_args *args)
{
	dput(args->h_dentry);
	au_nhash_fin(&args->whlist);
	iput(args->inode);
	mutex_unlock(&args->dir->i_mutex);
	iput(args->dir);
	kfree(args);
}

static void do_rmdir_whtmp(void *args)
{
	int err;
	struct au_whtmp_rmdir_args *a = args;
	struct super_block *sb;

	LKTRTrace("%.*s, b%d, dir i%lu\n",
		  AuDLNPair(a->h_dentry), a->bindex, a->dir->i_ino);

	mutex_lock(&a->dir->i_mutex);
	sb = a->dir->i_sb;
	si_noflush_read_lock(sb);
	err = au_test_ro(sb, a->bindex, NULL);
	if (!err) {
		struct dentry *h_parent = dget_parent(a->h_dentry);
		struct inode *h_dir = h_parent->d_inode;

		ii_write_lock_child(a->inode);
		ii_write_lock_parent(a->dir);
		au_hdir_lock(h_dir, a->dir, a->bindex);
		/* todo: revalidate h_dentry? */
		err = au_whtmp_rmdir(a->h_dentry, &a->whlist, a->bindex,
				     a->dir, a->inode, a->noself);
		au_hdir_unlock(h_dir, a->dir, a->bindex);
		ii_write_unlock(a->dir);
		ii_write_unlock(a->inode);
		dput(h_parent);
	}
	si_read_unlock(sb);
	au_whtmp_rmdir_free_args(a);
	if (unlikely(err))
		AuIOErr("err %d\n", err);
}

void au_whtmp_kick_rmdir(struct dentry *h_dentry, struct au_nhash *whlist,
			 aufs_bindex_t bindex, struct inode *dir,
			 struct inode *inode, int noself,
			 struct au_whtmp_rmdir_args *args)
{
	int wkq_err;

	LKTRTrace("%.*s\n", AuDLNPair(h_dentry));
	IMustLock(dir);

	/* all post-process will be done in do_rmdir_whtmp(). */
	args->h_dentry = dget(h_dentry);
	au_nhash_init(&args->whlist);
	au_nhash_move(&args->whlist, whlist);
	args->bindex = bindex;
	args->dir = igrab(dir);
	args->inode = igrab(inode);
	args->noself = noself;
	wkq_err = au_wkq_nowait(do_rmdir_whtmp, args, dir->i_sb, /*dlgt*/0);
	if (unlikely(wkq_err)) {
		AuWarn("rmdir error %.*s (%d), ignored\n",
		       AuDLNPair(h_dentry), wkq_err);
		au_whtmp_rmdir_free_args(args);
	}
}
