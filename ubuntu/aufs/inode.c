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
 * inode functions
 *
 * $Id: inode.c,v 1.4 2008/05/26 04:04:25 sfjro Exp $
 */

#include "aufs.h"

int au_refresh_hinode_self(struct inode *inode)
{
	int err, new_sz, update;
	struct inode *first;
	struct au_hinode *p, *q, tmp;
	struct super_block *sb;
	struct au_iinfo *iinfo;
	aufs_bindex_t bindex, bend, new_bindex;

	LKTRTrace("i%lu\n", inode->i_ino);
	IiMustWriteLock(inode);

	err = -ENOMEM;
	update = 0;
	sb = inode->i_sb;
	bend = au_sbend(sb);
	new_sz = sizeof(*iinfo->ii_hinode) * (bend + 1);
	iinfo = au_ii(inode);
	p = au_kzrealloc(iinfo->ii_hinode, sizeof(*p) * (iinfo->ii_bend + 1),
			 new_sz, GFP_KERNEL);
	if (unlikely(!p))
		goto out;

	iinfo->ii_hinode = p;
	p = iinfo->ii_hinode + iinfo->ii_bstart;
	first = p->hi_inode;
	err = 0;
	for (bindex = iinfo->ii_bstart; bindex <= iinfo->ii_bend;
	     bindex++, p++) {
		if (!p->hi_inode)
			continue;

		new_bindex = au_br_index(sb, p->hi_id);
		if (new_bindex == bindex)
			continue;
		if (new_bindex < 0) {
			update++;
			au_hiput(p);
			p->hi_inode = NULL;
			continue;
		}

		if (new_bindex < iinfo->ii_bstart)
			iinfo->ii_bstart = new_bindex;
		if (iinfo->ii_bend < new_bindex)
			iinfo->ii_bend = new_bindex;
		/* swap two hidden inode, and loop again */
		q = iinfo->ii_hinode + new_bindex;
		tmp = *q;
		*q = *p;
		*p = tmp;
		if (tmp.hi_inode) {
			bindex--;
			p--;
		}
	}
	au_update_brange(inode, /*do_put_zero*/0);

	if (unlikely(err))
		goto out;

	if (1 || first != au_h_iptr(inode, iinfo->ii_bstart))
		au_cpup_attr_all(inode);
	if (update && S_ISDIR(inode->i_mode))
		inode->i_version++;
	au_update_iigen(inode);

 out:
	AuTraceErr(err);
	return err;
}

int au_refresh_hinode(struct inode *inode, struct dentry *dentry)
{
	int err, update, isdir;
	struct inode *first;
	struct au_hinode *p;
	struct super_block *sb;
	struct au_iinfo *iinfo;
	aufs_bindex_t bindex, bend;
	unsigned int flags;

	LKTRTrace("%.*s\n", AuDLNPair(dentry));
	IiMustWriteLock(inode);

	err = au_refresh_hinode_self(inode);
	if (unlikely(err))
		goto out;

	sb = dentry->d_sb;
	bend = au_sbend(sb);
	iinfo = au_ii(inode);
	update = 0;
	p = iinfo->ii_hinode + iinfo->ii_bstart;
	first = p->hi_inode;
	isdir = S_ISDIR(inode->i_mode);
	flags = au_hi_flags(inode, isdir);
	bend = au_dbend(dentry);
	for (bindex = au_dbstart(dentry); bindex <= bend; bindex++) {
		struct inode *hi;
		struct dentry *hd;

		hd = au_h_dptr(dentry, bindex);
		if (!hd || !hd->d_inode)
			continue;

		if (iinfo->ii_bstart <= bindex && bindex <= iinfo->ii_bend) {
			hi = au_h_iptr(inode, bindex);
			if (hi) {
				if (hi == hd->d_inode)
					continue;
				err = -ESTALE;
				break;
			}
		}
		if (bindex < iinfo->ii_bstart)
			iinfo->ii_bstart = bindex;
		if (iinfo->ii_bend < bindex)
			iinfo->ii_bend = bindex;
		au_set_h_iptr(inode, bindex, igrab(hd->d_inode), flags);
		update++;
	}
	au_update_brange(inode, /*do_put_zero*/0);

	if (unlikely(err))
		goto out;

	if (1 || first != au_h_iptr(inode, iinfo->ii_bstart))
		au_cpup_attr_all(inode);
	if (update && isdir)
		inode->i_version++;
	au_update_iigen(inode);

 out:
	AuTraceErr(err);
	return err;
}

static int set_inode(struct inode *inode, struct dentry *dentry)
{
	int err, isdir;
	struct dentry *h_dentry;
	struct inode *h_inode;
	umode_t mode;
	aufs_bindex_t bindex, bstart, btail;
	struct au_iinfo *iinfo;
	unsigned int flags;

	LKTRTrace("i%lu, %.*s\n", inode->i_ino, AuDLNPair(dentry));
	AuDebugOn(!(inode->i_state & I_NEW));
	IiMustWriteLock(inode);
	bstart = au_dbstart(dentry);
	h_dentry = au_h_dptr(dentry, bstart);
	AuDebugOn(!h_dentry);
	h_inode = h_dentry->d_inode;
	AuDebugOn(!h_inode);

	err = 0;
	isdir = 0;
	mode = h_inode->i_mode;
	switch (mode & S_IFMT) {
	case S_IFREG:
		btail = au_dbtail(dentry);
		break;
	case S_IFDIR:
		isdir = 1;
		btail = au_dbtaildir(dentry);
		inode->i_op = &aufs_dir_iop;
		inode->i_fop = &aufs_dir_fop;
		break;
	case S_IFLNK:
		btail = au_dbtail(dentry);
		inode->i_op = &aufs_symlink_iop;
		break;
	case S_IFBLK:
	case S_IFCHR:
	case S_IFIFO:
	case S_IFSOCK:
		btail = au_dbtail(dentry);
		init_special_inode(inode, mode,
				   au_h_rdev(h_inode, /*h_mnt*/NULL, h_dentry));
		break;
	default:
		AuIOErr("Unknown file type 0%o\n", mode);
		err = -EIO;
		goto out;
	}

	flags = au_hi_flags(inode, isdir);
	iinfo = au_ii(inode);
	iinfo->ii_bstart = bstart;
	iinfo->ii_bend = btail;
	for (bindex = bstart; bindex <= btail; bindex++) {
		h_dentry = au_h_dptr(dentry, bindex);
		if (!h_dentry)
			continue;
		AuDebugOn(!h_dentry->d_inode);
		au_set_h_iptr(inode, bindex, igrab(h_dentry->d_inode), flags);
	}
	au_cpup_attr_all(inode);

 out:
	AuTraceErr(err);
	return err;
}

/* successful returns with iinfo write_locked */
/* todo: return with unlocked? */
static int reval_inode(struct inode *inode, struct dentry *dentry, int *matched)
{
	int err;
	struct inode *h_inode, *h_dinode;
	aufs_bindex_t bindex, bend;

	LKTRTrace("i%lu, %.*s\n", inode->i_ino, AuDLNPair(dentry));

	*matched = 0;

	/*
	 * before this function, if aufs got any iinfo lock, it must be only
	 * one, the parent dir.
	 * it can happen by UDBA and the obsoleted inode number.
	 */
	err = -EIO;
	if (unlikely(inode->i_ino == parent_ino(dentry)))
		goto out;

	err = 0;
	h_dinode = au_h_dptr(dentry, au_dbstart(dentry))->d_inode;
	mutex_lock_nested(&inode->i_mutex, AuLsc_I_CHILD);
	ii_write_lock_new(inode);
	bend = au_ibend(inode);
	for (bindex = au_ibstart(inode); bindex <= bend; bindex++) {
		h_inode = au_h_iptr(inode, bindex);
		if (h_inode && h_inode == h_dinode) {
			/* && (ibs != bstart
			   || !au_test_higen(inode, h_inode))); */
			*matched = 1;
			err = 0;
			if (unlikely(au_iigen(inode) != au_digen(dentry)))
				err = au_refresh_hinode(inode, dentry);
			break;
		}
	}
	if (unlikely(err))
		ii_write_unlock(inode);
	mutex_unlock(&inode->i_mutex);

 out:
	AuTraceErr(err);
	return err;
}

/* successful returns with iinfo write_locked */
/* todo: return with unlocked? */
struct inode *au_new_inode(struct dentry *dentry)
{
	struct inode *inode, *h_inode;
	struct dentry *h_dentry;
	ino_t h_ino;
	struct super_block *sb;
	int err, match;
	aufs_bindex_t bstart;
	struct au_xino_entry xinoe;

	LKTRTrace("%.*s\n", AuDLNPair(dentry));
	sb = dentry->d_sb;
	bstart = au_dbstart(dentry);
	h_dentry = au_h_dptr(dentry, bstart);
	AuDebugOn(!h_dentry);
	h_inode = h_dentry->d_inode;
	AuDebugOn(!h_inode);

	h_ino = h_inode->i_ino;
	err = au_xino_read(sb, bstart, h_ino, &xinoe);
	inode = ERR_PTR(err);
	if (unlikely(err))
		goto out;
 new_ino:
	if (!xinoe.ino) {
		xinoe.ino = au_xino_new_ino(sb);
		if (!xinoe.ino) {
			inode = ERR_PTR(-EIO);
			goto out;
		}
	}

	LKTRTrace("i%lu\n", xinoe.ino);
	inode = au_iget_locked(sb, xinoe.ino);
	err = PTR_ERR(inode);
	if (IS_ERR(inode))
		goto out;

	LKTRTrace("%lx, new %d\n", inode->i_state, !!(inode->i_state & I_NEW));
	if (inode->i_state & I_NEW) {
		ii_write_lock_new(inode);
		err = set_inode(inode, dentry);
		unlock_new_inode(inode);
		if (!err)
			goto out; /* success */
		iget_failed(inode);
		ii_write_unlock(inode);
		goto out_iput;
	} else {
		AuDebugOn(inode->i_state & I_LOCK);
		err = reval_inode(inode, dentry, &match);
		if (!err)
			goto out; /* success */
		else if (match)
			goto out_iput;
	}

	if (unlikely(au_test_unique_ino(h_dentry, h_ino)))
		AuWarn1("Un-notified UDBA or repeatedly renamed dir,"
			" b%d, %s, %.*s, hi%lu, i%lu.\n",
			bstart, au_sbtype(h_dentry->d_sb), AuDLNPair(dentry),
			h_ino, xinoe.ino);
	xinoe.ino = 0;
	err = au_xino_write0(sb, bstart, h_ino, 0);
	if (!err) {
		iput(inode);
		goto new_ino;
	}
	/* force noxino? */

 out_iput:
	iput(inode);
	inode = ERR_PTR(err);
 out:
	AuTraceErrPtr(inode);
	return inode;
}

/* ---------------------------------------------------------------------- */

int au_test_ro(struct super_block *sb, aufs_bindex_t bindex,
	       struct inode *inode)
{
	int err;

	err = au_br_rdonly(au_sbr(sb, bindex));

	/* pseudo-link after flushed may out of bounds */
	if (!err
	    && inode
	    && au_ibstart(inode) <= bindex
	    && bindex <= au_ibend(inode)) {
		/*
		 * permission check is unnecessary since vfsub routine
		 * will be called later
		 */
		struct inode *hi = au_h_iptr(inode, bindex);
		if (hi)
			err = IS_IMMUTABLE(hi) ? -EROFS : 0;
	}

	AuTraceErr(err);
	return err;
}

int au_test_h_perm(struct inode *h_inode, int mask, int dlgt)
{
	if (!current->fsuid)
		return 0;
	/* todo: fake nameidata? */
	return vfsub_permission(h_inode, mask, NULL, dlgt);
}

int au_test_h_perm_sio(struct inode *h_inode, int mask, int dlgt)
{
	if (unlikely(au_test_nfs(h_inode->i_sb)
		     && (mask & MAY_WRITE)
		     && S_ISDIR(h_inode->i_mode)))
		mask |= MAY_READ; /* force permission check */
	return au_test_h_perm(h_inode, mask, dlgt);
}
