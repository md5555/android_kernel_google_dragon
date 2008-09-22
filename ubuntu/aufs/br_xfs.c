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
 * special handling inode attributes on XFS branch in linux-2.6.24 and later
 *
 * $Id: br_xfs.c,v 1.3 2008/07/07 01:12:38 sfjro Exp $
 */

#include "aufs.h"

/* h_mnt can be NULL, is it safe? */
dev_t au_h_rdev(struct inode *h_inode, struct vfsmount *h_mnt,
		struct dentry *h_dentry)
{
	dev_t rdev;
	int err;
	struct kstat st;

	LKTRTrace("hi%lu\n", h_inode->i_ino);
	if (h_dentry)
		LKTRTrace("%.*s\n", AuDLNPair(h_dentry));

	rdev = h_inode->i_rdev;
	if (!rdev || !au_test_xfs(h_inode->i_sb))
		goto out;

	rdev = 0;
	if (!h_dentry) {
		err = 0;
		h_dentry = d_find_alias(h_inode);
		if (unlikely(!h_dentry))
			goto failure;
		err = PTR_ERR(h_dentry);
		if (IS_ERR(h_dentry)) {
			h_dentry = NULL;
			goto failure;
		}
		LKTRTrace("%.*s\n", AuDLNPair(h_dentry));
	} else
		dget(h_dentry);

	err = vfsub_getattr(h_mnt, h_dentry, &st, /*dlgt*/0);
	dput(h_dentry);
	if (!err) {
		rdev = st.rdev;
		goto out; /* success */
	}

 failure:
	AuIOErr("failed rdev for XFS inode, hi%lu, %d\n", h_inode->i_ino, err);
 out:
	return rdev;
}
