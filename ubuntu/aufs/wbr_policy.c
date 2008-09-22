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
 * policies for selecting one among multiple writable branches
 *
 * $Id: wbr_policy.c,v 1.12 2008/09/01 02:55:35 sfjro Exp $
 */

#include <linux/statfs.h>
#include "aufs.h"

static int au_cpdown_attr(struct au_hinode *hdir, struct dentry *h_dst,
			  struct dentry *h_src)
{
	int err, sbits;
	struct iattr ia;
	struct inode *h_idst, *h_isrc;

	LKTRTrace("%.*s\n", AuDLNPair(h_dst));
	h_idst = h_dst->d_inode;
	/* todo? IMustLock(h_idst); */
	h_isrc = h_src->d_inode;
	/* todo? IMustLock(h_isrc); */

	ia.ia_valid = ATTR_FORCE | ATTR_MODE | ATTR_UID | ATTR_GID;
	ia.ia_mode = h_isrc->i_mode;
	ia.ia_uid = h_isrc->i_uid;
	ia.ia_gid = h_isrc->i_gid;
	sbits = !!(ia.ia_mode & (S_ISUID | S_ISGID));

	err = vfsub_sio_notify_change(hdir, h_dst, &ia);

	/* is this nfs only? */
	if (!err && sbits && au_test_nfs(h_dst->d_sb)) {
		ia.ia_valid = ATTR_FORCE | ATTR_MODE;
		ia.ia_mode = h_isrc->i_mode;
		err = vfsub_sio_notify_change(hdir, h_dst, &ia);
	}

	/* todo: necessary? */
	if (!err)
		h_idst->i_flags = h_isrc->i_flags;

	AuTraceErr(err);
	return err;
}

struct au_cpdown_dir_args {
	struct dentry *parent;
	unsigned int parent_opq; /* bit-flags */
};

static int au_cpdown_dir(struct dentry *dentry, aufs_bindex_t bdst,
			 struct dentry *h_parent, void *arg)
{
	int err, rerr;
	struct au_cpdown_dir_args *args = arg;
	aufs_bindex_t bend, bopq, bstart;
	unsigned char parent_opq, whed, dlgt, do_opq, made_dir, diropq;
	struct dentry *h_dentry, *opq_dentry, *wh_dentry, *parent;
	struct inode *h_dir, *h_inode, *inode, *dir;

	LKTRTrace("%.*s, b%d\n", AuDLNPair(dentry), bdst);
	bstart = au_dbstart(dentry);
	AuDebugOn(bstart <= bdst
		  && bdst <= au_dbend(dentry)
		  && au_h_dptr(dentry, bdst));
	AuDebugOn(!h_parent);
	/* todo: safe? */
	parent = dget_parent(dentry);
	dir = parent->d_inode;
	dput(parent);
	h_dir = h_parent->d_inode;
	AuDebugOn(!h_dir);
	AuDebugOn(h_dir != au_h_iptr(dir, bdst));
	IMustLock(h_dir);

	err = au_lkup_neg(dentry, bdst);
	if (unlikely(err < 0))
		goto out;
	h_dentry = au_h_dptr(dentry, bdst);
	dlgt = !!au_test_dlgt(au_mntflags(dentry->d_sb));
	err = vfsub_sio_mkdir(au_hi(dir, bdst), h_dentry, S_IRWXU | S_IRUGO | S_IXUGO,
			      dlgt);
	if (unlikely(err))
		goto out_put;

	made_dir = 1;
	bend = au_dbend(dentry);
	bopq = au_dbdiropq(dentry);
	whed = (au_dbwh(dentry) == bdst);
	if (!args->parent_opq)
		args->parent_opq |= (bopq <= bdst);
	parent_opq = (args->parent_opq && args->parent == dentry);
	do_opq = 0;
	diropq = 0;
	h_inode = h_dentry->d_inode;
	mutex_lock_nested(&h_inode->i_mutex, AuLsc_I_CHILD);
	if (whed || (parent_opq && do_opq)) {
		opq_dentry = au_diropq_create(dentry, bdst, dlgt);
		err = PTR_ERR(opq_dentry);
		if (IS_ERR(opq_dentry)) {
			mutex_unlock(&h_inode->i_mutex);
			goto out_dir;
		}
		dput(opq_dentry);
		diropq = 1;
	}

	err = au_cpdown_attr(au_hi(dir, bdst), h_dentry, au_h_dptr(dentry, bstart));
	mutex_unlock(&h_inode->i_mutex);
	if (unlikely(err))
		goto out_opq;

	wh_dentry = NULL;
	if (whed) {
		wh_dentry = au_wh_lkup(h_parent, &dentry->d_name, /*ndx*/NULL);
		err = PTR_ERR(wh_dentry);
		if (IS_ERR(wh_dentry))
			goto out_opq;
		err = 0;
		if (wh_dentry->d_inode)
			err = au_wh_unlink_dentry(au_hi(dir, bdst), wh_dentry,
						  dentry, dlgt);
		dput(wh_dentry);
		if (unlikely(err))
			goto out_opq;
	}

	inode = dentry->d_inode;
	if (au_ibend(inode) < bdst)
		au_set_ibend(inode, bdst);
	au_set_h_iptr(inode, bdst, au_igrab(h_inode), au_hi_flags(inode, 1));
	goto out; /* success */

	/* revert */
 out_opq:
	if (diropq) {
		mutex_lock_nested(&h_inode->i_mutex, AuLsc_I_CHILD);
		rerr = au_diropq_remove(dentry, bdst, dlgt);
		mutex_unlock(&h_inode->i_mutex);
		if (unlikely(rerr)) {
			AuIOErr("failed removing diropq for %.*s b%d (%d)\n",
				AuDLNPair(dentry), bdst, rerr);
			err = -EIO;
			goto out;
		}
	}
 out_dir:
	if (made_dir) {
		rerr = vfsub_sio_rmdir(au_hi(dir, bdst), h_dentry, dlgt);
		if (unlikely(rerr)) {
			AuIOErr("failed removing %.*s b%d (%d)\n",
				AuDLNPair(dentry), bdst, rerr);
			err = -EIO;
		}
	}
 out_put:
	au_set_h_dptr(dentry, bdst, NULL);
	if (au_dbend(dentry) == bdst)
		au_update_dbend(dentry);
 out:
	AuTraceErr(err);
	return err;
}

int au_cpdown_dirs(struct dentry *dentry, aufs_bindex_t bdst)
{
	int err;
	struct au_cpdown_dir_args args = {
		.parent		= dget_parent(dentry),
		.parent_opq	= 0
	};

	LKTRTrace("%.*s, b%d\n", AuDLNPair(dentry), bdst);

	err = au_cp_dirs(dentry, bdst, au_cpdown_dir, &args);
	dput(args.parent);

	AuTraceErr(err);
	return err;
}

/* ---------------------------------------------------------------------- */

/* policies for create */

static int au_wbr_bu(struct super_block *sb, aufs_bindex_t bindex)
{
	for (; bindex >= 0; bindex--)
		if (!au_br_rdonly(au_sbr(sb, bindex)))
			return bindex;
	return -EROFS;
}

/* top down parent */
static int au_wbr_create_tdp(struct dentry *dentry, int isdir)
{
	int err;
	struct super_block *sb;
	aufs_bindex_t bstart, bindex;
	unsigned char dirperm1;
	struct dentry *parent, *h_parent;
	struct inode *h_dir;

	LKTRTrace("%.*s, dir %d\n", AuDLNPair(dentry), isdir);

	sb = dentry->d_sb;
	dirperm1 = !!au_test_dirperm1(au_mntflags(sb));
	bstart = au_dbstart(dentry);
	AuDebugOn(bstart < 0);
	err = bstart;
	/* todo: can 'err' be an illegal? */
	if (/* err >= 0 && */ !au_br_rdonly(au_sbr(sb, bstart)))
		goto out;

	err = -EROFS;
	parent = dget_parent(dentry);
	for (bindex = au_dbstart(parent); bindex < bstart; bindex++) {
		h_parent = au_h_dptr(parent, bindex);
		if (!h_parent)
			continue;
		h_dir = h_parent->d_inode;
		if (!h_dir)
			continue;

		if (!au_br_rdonly(au_sbr(sb, bindex))
		    && (!dirperm1
			|| au_test_h_perm(h_dir, MAY_WRITE | MAY_EXEC,
					  /*dlgt*/0))) {
				err = bindex;
				break;
		}
	}
	dput(parent);

	/* bottom up here */
	if (unlikely(err < 0))
		err = au_wbr_bu(sb, bstart - 1);

 out:
	LKTRTrace("b%d\n", err);
	return err;
}

/* ---------------------------------------------------------------------- */

/* an exception for the policy other than tdp */
static int au_wbr_create_exp(struct dentry *dentry)
{
	int err;
	struct dentry *parent;
	aufs_bindex_t bwh, bdiropq;

	LKTRTrace("%.*s\n", AuDLNPair(dentry));

	err = -1;
	bwh = au_dbwh(dentry);
	parent = dget_parent(dentry);
	bdiropq = au_dbdiropq(parent);
	if (bwh >= 0) {
		if (bdiropq >= 0)
			err = min(bdiropq, bwh);
		else
			err = bwh;
		LKTRTrace("%d\n", err);
	} else if (bdiropq >= 0) {
		err = bdiropq;
		LKTRTrace("%d\n", err);
	}
	dput(parent);

	if (err >= 0 && au_br_rdonly(au_sbr(dentry->d_sb, err)))
		err = -1;

	LKTRTrace("%d\n", err);
	return err;
}

/* ---------------------------------------------------------------------- */

/* round robin */
static int au_wbr_create_init_rr(struct super_block *sb)
{
	int err;

	err = au_wbr_bu(sb, au_sbend(sb));
	atomic_set(&au_sbi(sb)->si_wbr_rr_next, -err); /* less important */

	LKTRTrace("b%d\n", err);
	return err;
}

static int au_wbr_create_rr(struct dentry *dentry, int isdir)
{
	int err, nbr;
	struct super_block *sb;
	atomic_t *next;
	unsigned int u;
	aufs_bindex_t bindex, bend;

	LKTRTrace("%.*s, dir %d\n", AuDLNPair(dentry), isdir);

	sb = dentry->d_sb;
	next = NULL;
	err = au_wbr_create_exp(dentry);
	if (err >= 0)
		goto out;

	next = &au_sbi(sb)->si_wbr_rr_next;
	bend = au_sbend(sb);
	nbr = bend + 1;
	for (bindex = 0; bindex <= bend; bindex++) {
		if (!isdir) {
			err = atomic_dec_return(next) + 1;
			/* modulo for 0 is meaningless */
			if (unlikely(!err))
				err = atomic_dec_return(next) + 1;
		} else
			err = atomic_read(next);
		LKTRTrace("%d\n", err);
		u = err;
		err = u % nbr;
		LKTRTrace("%d\n", err);
		if (!au_br_rdonly(au_sbr(sb, err)))
			break;
		err = -EROFS;
	}

 out:
	LKTRTrace("%d\n", err);
	return err;
}

/* ---------------------------------------------------------------------- */

/* most free space */
static void *au_wbr_statfs_arg(struct au_branch *br, struct super_block *sb,
			       aufs_bindex_t bindex)
{
	struct super_block *h_sb;

	h_sb = br->br_mnt->mnt_sb;

	if (!au_test_nfs(h_sb))
		return h_sb->s_root;

	/* sigh,,, why nfs s_root has wrong inode? */
	return au_di(sb->s_root)->di_hdentry[0 + bindex].hd_dentry;
}

static void au_mfs(struct dentry *dentry)
{
	struct super_block *sb;
	aufs_bindex_t bindex, bend;
	unsigned char dlgt;
	int err;
	struct kstatfs st;
	unsigned long long b, bavail;
	void *arg;
	struct au_branch *br;
	struct au_wbr_mfs *mfs;

	LKTRTrace("%.*s\n", AuDLNPair(dentry));

	bavail = 0;
	sb = dentry->d_sb;
	mfs = &au_sbi(sb)->si_wbr_mfs;
	mfs->mfs_bindex = -EROFS;
	mfs->mfsrr_bytes = 0;
	dlgt = !!au_test_dlgt(au_mntflags(sb));
	bend = au_sbend(sb);
	for (bindex = 0; bindex <= bend; bindex++) {
		br = au_sbr(sb, bindex);
		if (au_br_rdonly(br))
			continue;
		AuDebugOn(!br->br_wbr);
		arg = au_wbr_statfs_arg(br, sb, bindex);
		if (!arg)
			continue;

		err = vfsub_statfs(arg, &st, dlgt);
		LKTRTrace("b%d, %d, %llu\n",
			  bindex, err, (unsigned long long)st.f_bavail);
		if (unlikely(err)) {
			AuWarn1("failed statfs, b%d, %d\n", bindex, err);
			continue;
		}

		/* when the available size is equal, select lower one */
		b = st.f_bavail * st.f_bsize;
		br->br_wbr->wbr_bytes = b;
		if (b >= bavail) {
			bavail = b;
			mfs->mfs_bindex = bindex;
			mfs->mfs_jiffy = jiffies;
		}
	}

	mfs->mfsrr_bytes = bavail;
	LKTRTrace("b%d\n", mfs->mfs_bindex);
}

static int au_wbr_create_mfs(struct dentry *dentry, int isdir)
{
	int err;
	struct super_block *sb;
	struct au_wbr_mfs *mfs;

	LKTRTrace("%.*s\n", AuDLNPair(dentry));

	sb = dentry->d_sb;
	err = au_wbr_create_exp(dentry);
	if (err >= 0)
		goto out;

	mfs = &au_sbi(sb)->si_wbr_mfs;
	mutex_lock(&mfs->mfs_lock);
	if (unlikely(time_after(jiffies, mfs->mfs_jiffy + mfs->mfs_expire)
		     || mfs->mfs_bindex < 0
		     || au_br_rdonly(au_sbr(sb, mfs->mfs_bindex))))
		au_mfs(dentry);
	mutex_unlock(&mfs->mfs_lock);
	err = mfs->mfs_bindex;

 out:
	LKTRTrace("b%d\n", err);
	return err;
}

static int au_wbr_create_init_mfs(struct super_block *sb)
{
	struct au_wbr_mfs *mfs;

	mfs = &au_sbi(sb)->si_wbr_mfs;
	LKTRTrace("expire %lu\n", mfs->mfs_expire);

	mutex_init(&mfs->mfs_lock);
	mfs->mfs_jiffy = 0;
	mfs->mfs_bindex = -EROFS;

	return 0;
}

static int au_wbr_create_fin_mfs(struct super_block *sb)
{
	AuTraceEnter();
	mutex_destroy(&au_sbi(sb)->si_wbr_mfs.mfs_lock);
	return 0;
}

/* ---------------------------------------------------------------------- */

/* most free space and then round robin */
static int au_wbr_create_mfsrr(struct dentry *dentry, int isdir)
{
	int err;
	struct au_wbr_mfs *mfs;

	LKTRTrace("%.*s, %d\n", AuDLNPair(dentry), isdir);

	err = au_wbr_create_mfs(dentry, isdir);
	if (err >= 0) {
		mfs = &au_sbi(dentry->d_sb)->si_wbr_mfs;
		LKTRTrace("%llu bytes, %llu wmark\n",
			  mfs->mfsrr_bytes, mfs->mfsrr_watermark);
		if (unlikely(mfs->mfsrr_bytes < mfs->mfsrr_watermark))
			err = au_wbr_create_rr(dentry, isdir);
	}

	LKTRTrace("b%d\n", err);
	return err;
}

static int au_wbr_create_init_mfsrr(struct super_block *sb)
{
	int err;

	au_wbr_create_init_mfs(sb); /* ignore */
	err = au_wbr_create_init_rr(sb);

	AuTraceErr(err);
	return err;
}

/* ---------------------------------------------------------------------- */

/* top down parent and most free space */
static int au_wbr_create_pmfs(struct dentry *dentry, int isdir)
{
	int err, e2;
	struct super_block *sb;
	struct dentry *parent, *h_parent;
	aufs_bindex_t bindex, bstart, bend;
	unsigned char dirperm1;
	struct au_branch *br;
	unsigned long long b;
	struct inode *h_dir;

	LKTRTrace("%.*s, %d\n", AuDLNPair(dentry), isdir);

	err = au_wbr_create_tdp(dentry, isdir);
	if (unlikely(err < 0))
		goto out;
	parent = dget_parent(dentry);
	bstart = au_dbstart(parent);
	bend = au_dbtaildir(parent);
	if (bstart == bend)
		goto out_parent; /* success */

	e2 = au_wbr_create_mfs(dentry, isdir);
	if (e2 < 0)
		goto out_parent; /* success */

	/* when the available size is equal, select upper one */
	sb = dentry->d_sb;
	br = au_sbr(sb, err);
	AuDebugOn(!br->br_wbr);
	dirperm1 = !!au_test_dirperm1(au_mntflags(sb));
	b = br->br_wbr->wbr_bytes;
	LKTRTrace("b%d, %llu\n", err, b);

	if (unlikely(dirperm1)) {
		for (bindex = bstart; bindex <= bend; bindex++) {
			h_parent = au_h_dptr(parent, bindex);
			if (!h_parent)
				continue;
			h_dir = h_parent->d_inode;
			if (!h_dir)
				continue;

			br = au_sbr(sb, bindex);
			if (!au_br_rdonly(br)
			    && au_test_h_perm(h_dir, MAY_WRITE | MAY_EXEC,
					      /*dlgt*/0)
			    && br->br_wbr->wbr_bytes > b) {
				b = br->br_wbr->wbr_bytes;
				err = bindex;
				LKTRTrace("b%d, %llu\n", err, b);
			}
		}
		if (err >= 0)
			goto out_parent;
	}
	for (bindex = bstart; bindex <= bend; bindex++) {
		h_parent = au_h_dptr(parent, bindex);
		if (!h_parent || !h_parent->d_inode)
			continue;

		br = au_sbr(sb, bindex);
		if (!au_br_rdonly(br) && br->br_wbr->wbr_bytes > b) {
			b = br->br_wbr->wbr_bytes;
			err = bindex;
			LKTRTrace("b%d, %llu\n", err, b);
		}
	}

 out_parent:
	dput(parent);
 out:
	LKTRTrace("b%d\n", err);
	return err;
}

/* ---------------------------------------------------------------------- */

/* policies for copyup */

/* top down parent */
static int au_wbr_copyup_tdp(struct dentry *dentry)
{
	return au_wbr_create_tdp(dentry, /*isdir, anything is ok*/0);
}

/* bottom up parent */
static int au_wbr_copyup_bup(struct dentry *dentry)
{
	int err;
	struct dentry *parent, *h_parent;
	aufs_bindex_t bindex, bstart;
	unsigned char dirperm1;
	struct super_block *sb;
	struct inode *h_dir;

	LKTRTrace("%.*s\n", AuDLNPair(dentry));

	err = -EROFS;
	sb = dentry->d_sb;
	dirperm1 = !!au_test_dirperm1(au_mntflags(sb));
	parent = dget_parent(dentry);
	bstart = au_dbstart(parent);
	for (bindex = au_dbstart(dentry); bindex >= bstart; bindex--) {
		h_parent = au_h_dptr(parent, bindex);
		if (!h_parent)
			continue;
		h_dir = h_parent->d_inode;
		if (!h_dir)
			continue;

		if (!au_br_rdonly(au_sbr(sb, bindex))
		    && (!dirperm1
			|| au_test_h_perm(h_dir, MAY_WRITE | MAY_EXEC,
					  /*dlgt*/0))) {
			err = bindex;
			break;
		}
	}
	dput(parent);

	/* bottom up here */
	if (unlikely(err < 0))
		err = au_wbr_bu(sb, bstart - 1);

	LKTRTrace("b%d\n", err);
	return err;
}

/* bottom up */
static int au_wbr_copyup_bu(struct dentry *dentry)
{
	int err;

	LKTRTrace("%.*s\n", AuDLNPair(dentry));

	err = au_wbr_bu(dentry->d_sb, au_dbstart(dentry));

	LKTRTrace("b%d\n", err);
	return err;
}

/* ---------------------------------------------------------------------- */

struct au_wbr_copyup_operations au_wbr_copyup_ops[] = {
	[AuWbrCopyup_TDP] = {
		.copyup	= au_wbr_copyup_tdp
	},
	[AuWbrCopyup_BUP] = {
		.copyup	= au_wbr_copyup_bup
	},
	[AuWbrCopyup_BU] = {
		.copyup	= au_wbr_copyup_bu
	}
};

struct au_wbr_create_operations au_wbr_create_ops[] = {
	[AuWbrCreate_TDP] = {
		.create	= au_wbr_create_tdp
	},
	[AuWbrCreate_RR] = {
		.create	= au_wbr_create_rr,
		.init	= au_wbr_create_init_rr
	},
	[AuWbrCreate_MFS] = {
		.create	= au_wbr_create_mfs,
		.init	= au_wbr_create_init_mfs,
		.fin	= au_wbr_create_fin_mfs
	},
	[AuWbrCreate_MFSV] = {
		.create	= au_wbr_create_mfs,
		.init	= au_wbr_create_init_mfs,
		.fin	= au_wbr_create_fin_mfs
	},
	[AuWbrCreate_MFSRR] = {
		.create	= au_wbr_create_mfsrr,
		.init	= au_wbr_create_init_mfsrr,
		.fin	= au_wbr_create_fin_mfs
	},
	[AuWbrCreate_MFSRRV] = {
		.create	= au_wbr_create_mfsrr,
		.init	= au_wbr_create_init_mfsrr,
		.fin	= au_wbr_create_fin_mfs
	},
	[AuWbrCreate_PMFS] = {
		.create	= au_wbr_create_pmfs,
		.init	= au_wbr_create_init_mfs,
		.fin	= au_wbr_create_fin_mfs
	},
	[AuWbrCreate_PMFSV] = {
		.create	= au_wbr_create_pmfs,
		.init	= au_wbr_create_init_mfs,
		.fin	= au_wbr_create_fin_mfs
	}
};
