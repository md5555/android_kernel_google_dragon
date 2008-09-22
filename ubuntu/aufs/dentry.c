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
 * lookup and dentry operations
 *
 * $Id: dentry.c,v 1.15 2008/09/22 03:52:06 sfjro Exp $
 */

#include "aufs.h"

/* ---------------------------------------------------------------------- */

/*
 * au_lkup_one() is a generic abstract entry function which calls
 * lookup_one_len() or __lookup_hash() finally. it is some condisions that makes
 * lookup complicated, which are nfs branch, open-intent and dlgt mode.
 */

#if defined(CONFIG_AUFS_BR_NFS) || defined(CONFIG_AUFS_DLGT)
/* cf. lookup_one_len() in linux/fs/namei.c */
struct dentry *au_lkup_one(const char *name, struct dentry *parent, int len,
			   struct au_ndx *ndx)
{
	struct dentry *dentry;

	LKTRTrace("%.*s/%.*s, ndx{%d, 0x%x}\n",
		  AuDLNPair(parent), len, name, !!ndx->nfsmnt, ndx->flags);

	ndx->nd_file = NULL;
	if (!ndx->nfsmnt)
		dentry = au_lkup_one_dlgt(name, parent, len, ndx->flags);
	else
		dentry = au_lkup_hash(name, parent, len, ndx);

	AuTraceErrPtr(dentry);
	return dentry;
}
#endif /* CONFIG_AUFS_BR_NFS || CONFIG_AUFS_DLGT */

struct au_lkup_one_args {
	struct dentry **errp;
	const char *name;
	struct dentry *parent;
	int len;
	struct au_ndx *ndx;
};

static void au_call_lkup_one(void *args)
{
	struct au_lkup_one_args *a = args;
	*a->errp = au_lkup_one(a->name, a->parent, a->len, a->ndx);
}

#define AuLkup_ALLOW_NEG	1
#define AuLkup_DLGT		(1 << 1)
#define AuLkup_DIRPERM1		(1 << 2)
#define au_ftest_lkup(flags, name)	((flags) & AuLkup_##name)
#define au_fset_lkup(flags, name)	{ (flags) |= AuLkup_##name; }
#define au_fclr_lkup(flags, name)	{ (flags) &= ~AuLkup_##name; }
#ifndef CONFIG_AUFS_DLGT
#undef AuLkup_DLGT
#define AuLkup_DLGT	0
#undef AuLkup_DIRPERM1
#define AuLkup_DIRPERM1	0
#endif

struct au_do_lookup_args {
	unsigned int		flags;
	mode_t			type;
	struct nameidata	*nd;
};

/*
 * returns positive/negative dentry, NULL or an error.
 * NULL means whiteout-ed or not-found.
 */
static /* noinline_for_stack */
struct dentry *au_do_lookup(struct dentry *h_parent, struct dentry *dentry,
			    aufs_bindex_t bindex, struct qstr *wh_name,
			    struct au_do_lookup_args *args)
{
	struct dentry *h_dentry;
	int err, wh_found, opq;
	unsigned char wh_able;
	struct inode *h_dir, *h_inode, *inode;
	struct qstr *name;
	struct super_block *sb;
	unsigned int nd_flags;
	struct au_ndx ndx = {
		.flags	= 0,
		.nd	= args->nd
	};
	const int allow_neg = au_ftest_lkup(args->flags, ALLOW_NEG);

	LKTRTrace("%.*s/%.*s, b%d, {flags 0x%x, type 0%o, nd %d}\n",
		  AuDLNPair(h_parent), AuDLNPair(dentry), bindex,
		  args->flags, args->type, !!args->nd);
	if (args->nd)
		LKTRTrace("nd{0x%x}\n", args->nd->flags);
	AuDebugOn(IS_ROOT(dentry));
	h_dir = h_parent->d_inode;

	nd_flags = 0;
	wh_found = 0;
	sb = dentry->d_sb;
	ndx.nfsmnt = au_nfsmnt(sb, bindex);
	if (unlikely(au_ftest_lkup(args->flags, DLGT)))
		au_fset_ndx(ndx.flags, DLGT);
	if (unlikely(au_ftest_lkup(args->flags, DIRPERM1)))
		au_fset_ndx(ndx.flags, DIRPERM1);
	LKTRTrace("nfsmnt %p\n", ndx.nfsmnt);
	ndx.br = au_sbr(sb, bindex);
	wh_able = !!au_br_whable(ndx.br->br_perm);
	name = &dentry->d_name;
	if (unlikely(wh_able))
		wh_found = au_test_robr_wh(name, h_parent, wh_name,
					   /*try_sio*/0, &ndx);
	h_dentry = ERR_PTR(wh_found);
	if (!wh_found)
		goto real_lookup;
	if (unlikely(wh_found < 0))
		goto out;

	/* We found a whiteout */
	/* au_set_dbend(dentry, bindex); */
	au_set_dbwh(dentry, bindex);
	if (!allow_neg)
		return NULL; /* success */
	if (unlikely(ndx.nd
		     && au_test_nfs(h_parent->d_sb)
		     && (ndx.nd->flags & LOOKUP_CREATE))) {
		nd_flags = ndx.nd->flags;
		ndx.nd->flags &= ~(LOOKUP_OPEN | LOOKUP_CREATE);
	}

 real_lookup:
	/* do not superio. */
	h_dentry = au_lkup_one(name->name, h_parent, name->len, &ndx);
	if (IS_ERR(h_dentry))
		goto out;
	AuDebugOn(d_unhashed(h_dentry));
	h_inode = h_dentry->d_inode;
	if (!h_inode) {
		if (!allow_neg)
			goto out_neg;
	} else if (wh_found
		   || (args->type && args->type != (h_inode->i_mode & S_IFMT)))
		goto out_neg;

	if (au_dbend(dentry) <= bindex)
		au_set_dbend(dentry, bindex);
	if (au_dbstart(dentry) < 0 || bindex < au_dbstart(dentry))
		au_set_dbstart(dentry, bindex);
	au_set_h_dptr(dentry, bindex, h_dentry);

	err = au_br_nfs_h_intent(ndx.nd_file, dentry, bindex, args->nd);
	if (unlikely(err)) {
		h_dentry = ERR_PTR(err);
		goto out;
	}

	inode = dentry->d_inode;
	if (!h_inode || !S_ISDIR(h_inode->i_mode) || !wh_able
	    || (inode && !S_ISDIR(inode->i_mode)))
		goto out; /* success */

	mutex_lock_nested(&h_inode->i_mutex, AuLsc_I_CHILD);
	opq = au_diropq_test(h_dentry, &ndx);
	mutex_unlock(&h_inode->i_mutex);
	if (opq > 0)
		au_set_dbdiropq(dentry, bindex);
	else if (unlikely(opq < 0)) {
		au_set_h_dptr(dentry, bindex, NULL);
		h_dentry = ERR_PTR(opq);
	}
	goto out;

 out_neg:
	dput(h_dentry);
	h_dentry = NULL;
 out:
	if (unlikely(nd_flags))
		ndx.nd->flags |= (nd_flags & (LOOKUP_OPEN | LOOKUP_CREATE));
	AuTraceErrPtr(h_dentry);
	return h_dentry;
}

/*
 * returns the number of hidden positive dentries,
 * otherwise an error.
 * can be called at unlinking with @type is zero.
 */
int au_lkup_dentry(struct dentry *dentry, aufs_bindex_t bstart, mode_t type,
		   struct nameidata *nd)
{
	int npositive, err;
	unsigned int mnt_flags;
	aufs_bindex_t bindex, btail, bdiropq;
	unsigned char isdir;
	struct qstr whname;
	struct au_do_lookup_args args = {
		.type	= type,
		.nd	= nd
	};
	const struct qstr *name = &dentry->d_name;
	struct dentry *parent;
	struct super_block *sb;
	struct inode *inode;

	LKTRTrace("%.*s, b%d, type 0%o\n", AuLNPair(name), bstart, type);
	AuDebugOn(bstart < 0 || IS_ROOT(dentry));

	/* dir may not be locked */
	parent = dget_parent(dentry);

	err = au_test_robr_shwh(dentry->d_sb, name);
	if (unlikely(err))
		goto out;

	err = au_wh_name_alloc(name->name, name->len, &whname);
	if (unlikely(err))
		goto out;

	sb = dentry->d_sb;
	mnt_flags = au_mntflags(sb);
	inode = dentry->d_inode;
	isdir = !!(inode && S_ISDIR(inode->i_mode));
	args.flags = 0;
	if (unlikely(au_test_dlgt(mnt_flags)))
		au_fset_lkup(args.flags, DLGT);
	if (unlikely(au_test_dirperm1(mnt_flags)))
		au_fset_lkup(args.flags, DIRPERM1);
	if (!type)
		au_fset_lkup(args.flags, ALLOW_NEG);
	npositive = 0;
	btail = au_dbtaildir(parent);
	for (bindex = bstart; bindex <= btail; bindex++) {
		struct dentry *h_parent, *h_dentry;
		struct inode *h_inode, *h_dir;

		h_dentry = au_h_dptr(dentry, bindex);
		if (h_dentry) {
			if (h_dentry->d_inode)
				npositive++;
			if (type != S_IFDIR)
				break;
			continue;
		}
		h_parent = au_h_dptr(parent, bindex);
		if (!h_parent)
			continue;
		h_dir = h_parent->d_inode;
		if (!h_dir || !S_ISDIR(h_dir->i_mode))
			continue;

		mutex_lock_nested(&h_dir->i_mutex, AuLsc_I_PARENT);
		h_dentry = au_do_lookup(h_parent, dentry, bindex, &whname,
					&args);
		mutex_unlock(&h_dir->i_mutex);
		err = PTR_ERR(h_dentry);
		if (IS_ERR(h_dentry))
			goto out_wh;
		au_fclr_lkup(args.flags, ALLOW_NEG);

		if (au_dbwh(dentry) >= 0)
			break;
		if (!h_dentry)
			continue;
		h_inode = h_dentry->d_inode;
		if (!h_inode)
			continue;
		npositive++;
		if (!args.type)
			args.type = h_inode->i_mode & S_IFMT;
		if (args.type != S_IFDIR)
			break;
		else if (isdir) {
			/* the type of lowers may be different */
			bdiropq = au_dbdiropq(dentry);
			if (bdiropq >= 0 && bdiropq <= bindex)
				break;
		}
	}

	if (npositive) {
		LKTRLabel(positive);
		au_update_dbstart(dentry);
	}
	err = npositive;
	if (unlikely(!au_opt_test(mnt_flags, UDBA_NONE)
		     && au_dbstart(dentry) < 0))
		/* both of real entry and whiteout found */
		err = -EIO;

 out_wh:
	au_wh_name_free(&whname);
 out:
	dput(parent);
	AuTraceErr(err);
	return err;
}

struct dentry *au_sio_lkup_one(const char *name, struct dentry *parent, int len,
			       struct au_ndx *ndx)
{
	struct dentry *dentry;
	int wkq_err;

	LKTRTrace("%.*s/%.*s\n", AuDLNPair(parent), len, name);

	if (!au_test_h_perm_sio(parent->d_inode, MAY_EXEC,
				au_ftest_ndx(ndx->flags, DLGT)))
		dentry = au_lkup_one(name, parent, len, ndx);
	else {
		/* todo: ugly? */
		unsigned int flags = ndx->flags;
		struct au_lkup_one_args args = {
			.errp	= &dentry,
			.name	= name,
			.parent	= parent,
			.len	= len,
			.ndx	= ndx
		};

		au_fclr_ndx(ndx->flags, DLGT);
		au_fclr_ndx(ndx->flags, DIRPERM1);
		wkq_err = au_wkq_wait(au_call_lkup_one, &args, /*dlgt*/0);
		if (unlikely(wkq_err))
			dentry = ERR_PTR(wkq_err);
		ndx->flags = flags;
	}

	AuTraceErrPtr(dentry);
	return dentry;
}

/*
 * lookup @dentry on @bindex which should be negative.
 */
int au_lkup_neg(struct dentry *dentry, aufs_bindex_t bindex)
{
	int err;
	struct dentry *parent, *h_parent, *h_dentry;
	struct inode *h_dir;
	struct au_ndx ndx = {
		.flags	= 0,
		.nd	= NULL,
		/* .br	= NULL */
	};
	struct super_block *sb;
	unsigned int mnt_flags;

	LKTRTrace("%.*s, b%d\n", AuDLNPair(dentry), bindex);
	/* dir may not be locked */
	parent = dget_parent(dentry);
	AuDebugOn(!parent || !parent->d_inode
		  || !S_ISDIR(parent->d_inode->i_mode));
	h_parent = au_h_dptr(parent, bindex);
	AuDebugOn(!h_parent);
	h_dir = h_parent->d_inode;
	AuDebugOn(!h_dir || !S_ISDIR(h_dir->i_mode));

	sb = dentry->d_sb;
	mnt_flags = au_mntflags(sb);
	ndx.nfsmnt = au_nfsmnt(sb, bindex);
	if (unlikely(au_test_dlgt(mnt_flags)))
		au_fset_ndx(ndx.flags, DLGT);
	if (unlikely(au_test_dirperm1(mnt_flags)))
		au_fset_ndx(ndx.flags, DIRPERM1);
	h_dentry = au_sio_lkup_one(dentry->d_name.name, h_parent,
				   dentry->d_name.len, &ndx);
	err = PTR_ERR(h_dentry);
	if (IS_ERR(h_dentry))
		goto out;
	if (unlikely(h_dentry->d_inode)) {
		err = -EIO;
		AuIOErr("b%d %.*s should be negative.\n",
			bindex, AuDLNPair(h_dentry));
		dput(h_dentry);
		goto out;
	}

	if (bindex < au_dbstart(dentry))
		au_set_dbstart(dentry, bindex);
	if (au_dbend(dentry) < bindex)
		au_set_dbend(dentry, bindex);
	au_set_h_dptr(dentry, bindex, h_dentry);
	err = 0;

 out:
	dput(parent);
	AuTraceErr(err);
	return err;
}

/*
 * returns the number of found hidden positive dentries,
 * otherwise an error.
 */
int au_refresh_hdentry(struct dentry *dentry, mode_t type)
{
	int npositive, new_sz;
	struct au_dinfo *dinfo;
	struct super_block *sb;
	struct dentry *parent;
	aufs_bindex_t bindex, parent_bend, parent_bstart, bwh, bdiropq, bend;
	struct au_hdentry *p;
	au_gen_t sgen;

	LKTRTrace("%.*s, type 0%o\n", AuDLNPair(dentry), type);
	DiMustWriteLock(dentry);
	sb = dentry->d_sb;
	AuDebugOn(IS_ROOT(dentry));
	sgen = au_sigen(sb);
	parent = dget_parent(dentry);
	AuDebugOn(au_digen(parent) != sgen
		  || au_iigen(parent->d_inode) != sgen);

	npositive = -ENOMEM;
	new_sz = sizeof(*dinfo->di_hdentry) * (au_sbend(sb) + 1);
	dinfo = au_di(dentry);
	p = au_kzrealloc(dinfo->di_hdentry, sizeof(*p) * (dinfo->di_bend + 1),
			 new_sz, GFP_NOFS);
	if (unlikely(!p))
		goto out;
	dinfo->di_hdentry = p;

	bend = dinfo->di_bend;
	bwh = dinfo->di_bwh;
	bdiropq = dinfo->di_bdiropq;
	p += dinfo->di_bstart;
	for (bindex = dinfo->di_bstart; bindex <= bend; bindex++, p++) {
		struct dentry *hd, *hdp;
		struct au_hdentry tmp, *q;
		aufs_bindex_t new_bindex;

		hd = p->hd_dentry;
		if (!hd)
			continue;
		hdp = dget_parent(hd);
		if (hdp == au_h_dptr(parent, bindex)) {
			dput(hdp);
			continue;
		}

		new_bindex = au_find_dbindex(parent, hdp);
		dput(hdp);
		AuDebugOn(new_bindex == bindex);
		if (dinfo->di_bwh == bindex)
			bwh = new_bindex;
		if (dinfo->di_bdiropq == bindex)
			bdiropq = new_bindex;
		/* todo: test more? */
		if (new_bindex < 0) {
			au_hdput(p, /*do_free*/0);
			p->hd_dentry = NULL;
			continue;
		}
		/* swap two hidden dentries, and loop again */
		q = dinfo->di_hdentry + new_bindex;
		tmp = *q;
		*q = *p;
		*p = tmp;
		if (tmp.hd_dentry) {
			bindex--;
			p--;
		}
	}

	/* todo: test more? */
	dinfo->di_bwh = -1;
	if (unlikely(bwh >= 0 && bwh <= au_sbend(sb) && au_sbr_whable(sb, bwh)))
		dinfo->di_bwh = bwh;
	dinfo->di_bdiropq = -1;
	if (unlikely(bdiropq >= 0 && bdiropq <= au_sbend(sb)
		     && au_sbr_whable(sb, bdiropq)))
		dinfo->di_bdiropq = bdiropq;
	parent_bend = au_dbend(parent);
	p = dinfo->di_hdentry;
	for (bindex = 0; bindex <= parent_bend; bindex++, p++)
		if (p->hd_dentry) {
			dinfo->di_bstart = bindex;
			break;
		}
	p = dinfo->di_hdentry + parent_bend;
	for (bindex = parent_bend; bindex >= 0; bindex--, p--)
		if (p->hd_dentry) {
			dinfo->di_bend = bindex;
			break;
		}

	npositive = 0;
	parent_bstart = au_dbstart(parent);
	if (type != S_IFDIR && dinfo->di_bstart == parent_bstart)
		goto out_dgen; /* success */

	npositive = au_lkup_dentry(dentry, parent_bstart, type, /*nd*/NULL);
	if (npositive < 0)
		goto out;
	if (unlikely(dinfo->di_bwh >= 0 && dinfo->di_bwh <= dinfo->di_bstart))
		d_drop(dentry);

 out_dgen:
	au_update_digen(dentry);
 out:
	dput(parent);
	AuTraceErr(npositive);
	return npositive;
}

static int au_lock_nd(struct dentry *dentry, struct nameidata *nd)
{
	int locked = 0;
	if (nd && dentry != nd->path.dentry) {
		di_read_lock_parent(nd->path.dentry, 0);
		locked = 1;
	}
	return locked;
}

static void au_unlock_nd(int locked, struct nameidata *nd)
{
	if (locked)
		di_read_unlock(nd->path.dentry, 0);
}

/* #define TestingFuse */
static noinline_for_stack
int au_do_h_d_reval(struct dentry *dentry, aufs_bindex_t bindex,
		    struct nameidata *nd, struct dentry *h_dentry)
{
	int err, valid, e;
	int (*reval)(struct dentry *, struct nameidata *);
	struct super_block *sb;
	struct nameidata fake_nd, *p;

	LKTRTrace("%.*s, b%d, nd %d\n", AuDLNPair(dentry), bindex, !!nd);

	err = 0;
	reval = NULL;
	if (h_dentry->d_op)
		reval = h_dentry->d_op->d_revalidate;
	if (!reval)
		goto out;

	sb = dentry->d_sb;
	if (nd) {
		memcpy(&fake_nd, nd, sizeof(*nd));
		err = au_fake_intent(&fake_nd, au_sbr_perm(sb, bindex));
		if (unlikely(err)) {
			err = -EINVAL;
			goto out;
		}
	}
	p = au_fake_dm(&fake_nd, nd, sb, bindex);
	AuDebugOn(IS_ERR(p));
	AuDebugOn(nd && p != &fake_nd);
	LKTRTrace("b%d\n", bindex);

	/* it may return tri-state */
#if 1
	valid = reval(h_dentry, p);
#else
	if (p /*&& !IS_ROOT(h_dentry)*/) {
		struct dentry *h_parent = dget_parent(h_dentry);
		struct mutex *h_mtx = &h_parent->d_inode->i_mutex;
#if 0
		lktr_set_pid(current->pid, LktrArrayPid);
		AuDbgDentry(p->path.dentry);
		AuDbgDentry(h_dentry);
		lktr_clear_pid(current->pid, LktrArrayPid);
#endif
		mutex_lock_nested(h_mtx, AuLsc_I_PARENT);
		valid = reval(h_dentry, p);
		mutex_unlock(h_mtx);
		dput(h_parent);
	} else
		valid = reval(h_dentry, p);
#endif
	if (unlikely(valid < 0))
		err = valid;
	else if (!valid)
		err = -EINVAL;
	else
		AuDebugOn(err);

	if (p) {
		AuDebugOn(!nd);
		e = au_hin_after_reval(p, dentry, bindex, nd->intent.open.file);
#ifndef TestingFuse
		au_update_fuse_h_inode(p->path.mnt, h_dentry); /*ignore*/
#endif
		if (unlikely(e && !err))
			err = e;
	}
#ifndef TestingFuse
	else
		au_update_fuse_h_inode(NULL, h_dentry); /*ignore*/
#endif
	au_fake_dm_release(p);

 out:
	AuTraceErr(err);
	return err;
}

static noinline_for_stack
int h_d_revalidate(struct dentry *dentry, struct inode *inode,
		   struct nameidata *nd, int do_udba)
{
	int err;
	aufs_bindex_t bindex, btail, bstart, ibs, ibe;
	unsigned char plus, locked, unhashed, is_root, h_plus;
	struct super_block *sb;
	struct inode *first, *h_inode, *h_cached_inode;
	umode_t mode, h_mode;
	struct dentry *h_dentry;
	struct qstr *name;

	LKTRTrace("%.*s, nd %d\n", AuDLNPair(dentry), !!nd);
	AuDebugOn(inode && au_digen(dentry) != au_iigen(inode));

	err = 0;
	sb = dentry->d_sb;
	plus = 0;
	mode = 0;
	first = NULL;
	ibs = -1;
	ibe = -1;
	unhashed = !!d_unhashed(dentry);
	is_root = !!IS_ROOT(dentry);
	name = &dentry->d_name;

	/*
	 * Theoretically, REVAL test should be unnecessary in case of INOTIFY.
	 * But inotify doesn't fire some necessary events,
	 *	IN_ATTRIB for atime/nlink/pageio
	 *	IN_DELETE for NFS dentry
	 * Let's do REVAL test too.
	 */
	if (do_udba && inode) {
		mode = (inode->i_mode & S_IFMT);
		plus = (inode->i_nlink > 0);
		first = au_h_iptr(inode, au_ibstart(inode));
		ibs = au_ibstart(inode);
		ibe = au_ibend(inode);
	}

	bstart = au_dbstart(dentry);
	btail = bstart;
	if (inode && S_ISDIR(inode->i_mode))
		btail = au_dbtaildir(dentry);
	locked = !!au_lock_nd(dentry, nd);
	for (bindex = bstart; bindex <= btail; bindex++) {
		h_dentry = au_h_dptr(dentry, bindex);
		if (!h_dentry)
			continue;

		LKTRTrace("b%d, %.*s\n", bindex, AuDLNPair(h_dentry));
#ifdef TestingFuse
		/* force re-lookup for fuse, in order to update attributes */
		if (unlikely(au_test_fuse(h_dentry->d_sb)))
			goto err;
#endif

		if (unlikely(do_udba
			     && !is_root
			     && (unhashed != !!d_unhashed(h_dentry)
				 || name->len != h_dentry->d_name.len
				 || memcmp(name->name, h_dentry->d_name.name,
					   name->len)
				     ))) {
			LKTRTrace("unhash 0x%x 0x%x, %.*s %.*s\n",
				  unhashed, d_unhashed(h_dentry),
				  AuDLNPair(dentry), AuDLNPair(h_dentry));
			goto err;
		}

		err = au_do_h_d_reval(dentry, bindex, nd, h_dentry);
		if (unlikely(err))
			/* do not goto err, to keep the errno */
			break;

		/* todo: plink too? */
		if (unlikely(!do_udba))
			continue;

		/* UDBA tests */
		h_inode = h_dentry->d_inode;
		if (unlikely(!!inode != !!h_inode))
			goto err;

		h_plus = plus;
		h_mode = mode;
		h_cached_inode = h_inode;
		if (h_inode) {
			h_mode = (h_inode->i_mode & S_IFMT);
			h_plus = (h_inode->i_nlink > 0);
		}
		if (inode && ibs <= bindex && bindex <= ibe)
			h_cached_inode = au_h_iptr(inode, bindex);

		LKTRTrace("{%d, 0%o, %p}, h{%d, 0%o, %p}\n",
			  plus, mode, h_cached_inode,
			  h_plus, h_mode, h_inode);
		if (unlikely(plus != h_plus
			     || mode != h_mode
			     || h_cached_inode != h_inode))
			goto err;
		continue;

	err:
		err = -EINVAL;
		break;
	}
	au_unlock_nd(locked, nd);

	/*
	 * judging by timestamps is meaningless since some filesystem uses
	 * CURRENT_TIME_SEC instead of CURRENT_TIME.
	 */
	/*
	 * NFS may stop IN_DELETE because of DCACHE_NFSFS_RENAMED.
	 */

	AuTraceErr(err);
	return err;
}

static int simple_reval_dpath(struct dentry *dentry, au_gen_t sgen)
{
	int err;
	mode_t type;
	struct dentry *parent;
	struct inode *inode;

	LKTRTrace("%.*s, sgen %d\n", AuDLNPair(dentry), sgen);
	SiMustAnyLock(dentry->d_sb);
	DiMustWriteLock(dentry);
	inode = dentry->d_inode;
	AuDebugOn(!inode);

	if (au_digen(dentry) == sgen && au_iigen(inode) == sgen)
		return 0;

	parent = dget_parent(dentry);
	di_read_lock_parent(parent, AuLock_IR);
	AuDebugOn(au_digen(parent) != sgen
		  || au_iigen(parent->d_inode) != sgen);
#ifdef CONFIG_AUFS_DEBUG
	{
		int i, j;
		struct au_dcsub_pages dpages;
		struct au_dpage *dpage;
		struct dentry **dentries;

		err = au_dpages_init(&dpages, GFP_NOFS);
		AuDebugOn(err);
		err = au_dcsub_pages_rev(&dpages, parent, /*do_include*/1, NULL,
					 NULL);
		AuDebugOn(err);
		for (i = dpages.ndpage - 1; !err && i >= 0; i--) {
			dpage = dpages.dpages + i;
			dentries = dpage->dentries;
			for (j = dpage->ndentry - 1; !err && j >= 0; j--)
				AuDebugOn(au_digen(dentries[j]) != sgen);
		}
		au_dpages_free(&dpages);
	}
#endif
	type = (inode->i_mode & S_IFMT);
	/* returns a number of positive dentries */
	err = au_refresh_hdentry(dentry, type);
	if (err >= 0)
		err = au_refresh_hinode(inode, dentry);
	di_read_unlock(parent, AuLock_IR);
	dput(parent);
	AuTraceErr(err);
	return err;
}

int au_reval_dpath(struct dentry *dentry, au_gen_t sgen)
{
	int err;
	struct dentry *d, *parent;
	struct inode *inode;

	LKTRTrace("%.*s, sgen %d\n", AuDLNPair(dentry), sgen);
	AuDebugOn(!dentry->d_inode);
	DiMustWriteLock(dentry);

	if (!au_ftest_si(au_sbi(dentry->d_sb), FAILED_REFRESH_DIRS))
		return simple_reval_dpath(dentry, sgen);

	/* slow loop, keep it simple and stupid */
	/* cf: au_cpup_dirs() */
	err = 0;
	parent = NULL;
	while (au_digen(dentry) != sgen || au_iigen(dentry->d_inode) != sgen) {
		d = dentry;
		while (1) {
			dput(parent);
			parent = dget_parent(d);
			if (au_digen(parent) == sgen
			    && au_iigen(parent->d_inode) == sgen)
				break;
			d = parent;
		}

		inode = d->d_inode;
		if (d != dentry)
			di_write_lock_child(d);

		/* someone might update our dentry while we were sleeping */
		if (au_digen(d) != sgen || au_iigen(d->d_inode) != sgen) {
			di_read_lock_parent(parent, AuLock_IR);
			/* returns a number of positive dentries */
			err = au_refresh_hdentry(d, inode->i_mode & S_IFMT);
			if (err >= 0)
				err = au_refresh_hinode(inode, d);
			di_read_unlock(parent, AuLock_IR);
		}

		if (d != dentry)
			di_write_unlock(d);
		dput(parent);
		if (unlikely(err))
			break;
	}

	AuTraceErr(err);
	return err;
}

/*
 * THIS IS A BOOLEAN FUNCTION: returns 1 if valid, 0 otherwise.
 * nfsd passes NULL as nameidata.
 */
static int aufs_d_revalidate(struct dentry *dentry, struct nameidata *nd)
{
	int valid, err;
	au_gen_t sgen;
	unsigned char do_udba;
	struct nameidata tmp_nd, *ndp;
	struct super_block *sb;
	struct inode *inode;

	LKTRTrace("dentry %.*s\n", AuDLNPair(dentry));
	if (nd && nd->path.dentry)
		LKTRTrace("nd{%.*s, 0x%x}\n",
			  AuDLNPair(nd->path.dentry), nd->flags);
	/*
	 * dir case: AuDebugOn(dentry->d_parent != nd->dentry);
	 * remove failure case:AuDebugOn(!IS_ROOT(dentry)
	 * 				 && d_unhashed(dentry));
	*/
	AuDebugOn(!dentry->d_fsdata);

	err = -EINVAL;
#if 0
	if (d_unhashed(dentry))
		goto __out;
#endif
	sb = dentry->d_sb;
#if 1
	aufs_read_lock(dentry, AuLock_FLUSH | AuLock_DW);
	inode = dentry->d_inode;
#else
	si_read_lock(sb, AuLock_FLUSH);
	//lktr_set_pid(current->pid, LktrArrayPid);
	AuDbgDentry(dentry);
	AuDbgInode(dentry->d_inode);
	au_rw_write_lock_nested(&au_di(dentry)->di_rwsem, AuLsc_DI_CHILD);
	LKTRLabel(clean);
	//lktr_clear_pid(current->pid, LktrArrayPid);
	inode = dentry->d_inode;
	if (unlikely(inode && (!inode->i_nlink || IS_DEADDIR(inode)))) {
		AuDbg("here\n");
		au_rw_write_unlock(&au_di(dentry)->di_rwsem);
		si_read_unlock(sb);
		goto __out;
	}
	if (inode) {
		//lktr_set_pid(current->pid, LktrArrayPid);
		AuDbgDentry(dentry);
		AuDbgInode(inode);
		ii_write_lock_child(inode);
		LKTRLabel(clean);
		//lktr_clear_pid(current->pid, LktrArrayPid);
	}
#endif

	sgen = au_sigen(sb);
	if (unlikely(au_digen(dentry) != sgen)) {
		AuDebugOn(IS_ROOT(dentry));
#ifdef ForceInotify
		AuDbg("UDBA or digen, %.*s\n", AuDLNPair(dentry));
#endif
		if (inode)
			err = au_reval_dpath(dentry, sgen);
		if (unlikely(err))
			goto out_dgrade;
		AuDebugOn(au_digen(dentry) != sgen);
	}
	if (unlikely(inode && au_iigen(inode) != sgen)) {
		AuDebugOn(IS_ROOT(dentry));
#ifdef ForceInotify
		AuDbg("UDBA or survived, %.*s\n", AuDLNPair(dentry));
#endif
		err = au_refresh_hinode(inode, dentry);
		if (unlikely(err))
			goto out_dgrade;
		AuDebugOn(au_iigen(inode) != sgen);
	}
	di_downgrade_lock(dentry, AuLock_IR);

#if 0 /* todo: support it? */
	/* parent dir i_nlink is not updated in the case of setattr */
	if (S_ISDIR(inode->i_mode)) {
		mutex_lock(&inode->i_mutex);
		ii_write_lock(inode);
		au_cpup_attr_nlink(inode);
		ii_write_unlock(inode);
		mutex_unlock(&inode->i_mutex);
	}
#endif

	AuDebugOn(au_digen(dentry) != sgen);
	AuDebugOn(inode && au_iigen(inode) != sgen);
	err = -EINVAL;
	do_udba = !au_opt_test(au_mntflags(sb), UDBA_NONE);
	if (do_udba && inode) {
		aufs_bindex_t bstart = au_ibstart(inode);
		if (bstart >= 0
		    && au_test_higen(inode, au_h_iptr(inode, bstart)))
			goto out;
	}
	ndp = au_dup_nd(au_sbi(sb), &tmp_nd, nd);
#if 0
	if (nd) {
		path = nd->path;
		nd->path.dentry = au_h_dptr(nd->path.dentry, bindex);
	if (fake_nd->path.dentry) {
		fake_nd->path.mnt = au_sbr_mnt(sb, bindex);
		AuDebugOn(!fake_nd->path.mnt);
		path_get(&fake_nd->path);
		nd->path.
			}
	}
#endif
	err = h_d_revalidate(dentry, inode, ndp, do_udba);
	if (unlikely(!err && do_udba && au_dbstart(dentry) < 0))
		/* both of real entry and whiteout found */
		err = -EIO;
	goto out;

 out_dgrade:
	di_downgrade_lock(dentry, AuLock_IR);
 out:
	au_store_fmode_exec(nd, inode);

	aufs_read_unlock(dentry, AuLock_IR);
	//__out:
	AuTraceErr(err);
	valid = !err;
	if (!valid)
		LKTRTrace("%.*s invalid\n", AuDLNPair(dentry));
	return valid;
}

static void aufs_d_release(struct dentry *dentry)
{
	struct au_dinfo *dinfo;
	aufs_bindex_t bend, bindex;

	LKTRTrace("%.*s\n", AuDLNPair(dentry));
	AuDebugOn(!d_unhashed(dentry));

	dinfo = dentry->d_fsdata;
	if (unlikely(!dinfo))
		return;

	/* dentry may not be revalidated */
	bindex = dinfo->di_bstart;
	if (bindex >= 0) {
		struct au_hdentry *p;
		bend = dinfo->di_bend;
		AuDebugOn(bend < bindex);
		p = dinfo->di_hdentry + bindex;
		while (bindex++ <= bend) {
			if (p->hd_dentry)
				au_hdput(p, /*do_free*/1);
			p++;
		}
	}
	kfree(dinfo->di_hdentry);
	au_cache_free_dinfo(dinfo);
	au_hin_di_reinit(dentry);
	/* todo: tmp code */
	//dentry->d_fsdata = NULL;
}

struct dentry_operations aufs_dop = {
	.d_revalidate	= aufs_d_revalidate,
	.d_release	= aufs_d_release,
	/* never use d_delete, especially in case of nfs server */
};
