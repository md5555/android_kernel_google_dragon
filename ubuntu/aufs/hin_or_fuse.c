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
 * inode attributes on FUSE branch or HINOTIFY
 *
 * $Id: hin_or_fuse.c,v 1.6 2008/09/15 03:14:30 sfjro Exp $
 */

#include "aufs.h"

static struct dentry *
au_h_dget_any(struct dentry *dentry, aufs_bindex_t *bindex)
{
	struct dentry *h_dentry;
	struct inode *inode, *h_inode;
	struct super_block *sb;
	aufs_bindex_t ib, db;

	/* must be positive dentry */
	inode = dentry->d_inode;
	LKTRTrace("%.*s, i%lu\n", AuDLNPair(dentry), inode->i_ino);

	sb = dentry->d_sb;
	db = au_dbstart(dentry);
	ib = au_ibstart(inode);
	if (db == ib) {
		*bindex = db;
		h_dentry = dget(au_h_dptr(dentry, db));
		if (h_dentry)
			goto out; /* success */
	}

	*bindex = ib;
	h_inode = au_h_iptr(inode, ib);
	h_dentry = d_find_alias(h_inode);
	if (h_dentry)
		goto out; /* success */

#if 0
	if (au_opt_test(au_mntflags(sb), PLINK)
	    && au_plink_test(sb, inode)) {
		h_dentry = au_plink_lkup(sb, ib, inode);
		if (IS_ERR(h_dentry))
			goto out;
		AuDebugOn(!h_dentry->d_inode);
		goto out; /* success */
	}
#endif

	h_dentry = dget(au_hi_wh(inode, ib));

 out:
	AuTraceErrPtr(h_dentry);
	return h_dentry;
}

int aufs_getattr(struct vfsmount *mnt, struct dentry *dentry, struct kstat *st)
{
	int err;
	unsigned int mnt_flags;
	aufs_bindex_t bindex;
	struct inode *inode;
	struct dentry *h_dentry;
	struct super_block *sb, *h_sb;

	LKTRTrace("%.*s\n", AuDLNPair(dentry));

	err = 0;
	inode = dentry->d_inode;
	sb = dentry->d_sb;
	aufs_read_lock(dentry, AuLock_FLUSH | AuLock_IR);

	/* todo: nfs branch too? */
	/* todo: test bit inotify option too? */
	mnt_flags = au_mntflags(sb);
	bindex = au_ibstart(inode);
	h_sb = au_sbr_sb(sb, bindex);
	if (!au_test_fuse(h_sb)
	    //&& !au_test_nfs(h_sb) /* todo: fix me */
	    && (au_iigen(inode) == au_sigen(sb)
		|| (au_opt_test(mnt_flags, PLINK) && au_plink_test(sb, inode))))
		goto fill;

	h_dentry = au_h_dget_any(dentry, &bindex);
	err = PTR_ERR(h_dentry);
	if (IS_ERR(h_dentry))
		goto out;

	err = -EIO;
	if (h_dentry && h_dentry->d_inode)
		err = vfsub_getattr(au_sbr_mnt(sb, bindex), h_dentry, st,
				    au_test_dlgt(mnt_flags));
	dput(h_dentry);
	if (!err) {
		au_cpup_attr_all(inode);
	fill:
		generic_fillattr(inode, st);
	}

 out:
	aufs_read_unlock(dentry, AuLock_IR);
	AuTraceErr(err);
	return err;
}
