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
 * special handling for inode attributes on FUSE branch
 *
 * $Id: br_fuse.c,v 1.6 2008/07/27 22:49:28 sfjro Exp $
 */

#include "aufs.h"

/* h_mnt can be NULL, is it safe? */
int au_update_fuse_h_inode(struct vfsmount *h_mnt, struct dentry *h_dentry)
{
	int err;
	struct kstat st;

	LKTRTrace("%.*s\n", AuDLNPair(h_dentry));

	err = 0;
	if (unlikely(h_dentry->d_inode
		     /* && atomic_read(&h_dentry->d_inode->i_count) */
		     && au_test_fuse(h_dentry->d_sb))) {
		err = vfsub_getattr(h_mnt, h_dentry, &st, /*dlgt*/0);
		if (unlikely(err)) {
			AuDbg("err %d\n", err);
			au_debug_on();
			AuDbgDentry(h_dentry);
			au_debug_off();
			WARN_ON(err);
		}
	}
	return err;
}

#if 0 /* temp */
/*
 * This function was born after a discussion with the FUSE developer.
 * The inode attributes on a filesystem who defines i_op->getattr()
 * is unreliable since such fs may not maintain the attributes at lookup.
 * This function doesn't want the result of stat, instead wants the side-effect
 * which refreshes the attributes.
 * Hmm, there seems to be no such filesystem except fuse.
 */
int vfsub_i_attr(struct vfsmount *mnt, struct dentry *dentry, int dlgt)
{
	int err;
	struct inode *inode;
	struct inode_operations *op;
	struct kstat st;

	inode = dentry->d_inode;
	AuDebugOn(!inode);

	err = 0;
	op = inode->i_op;
	if (unlikely(op && op->getattr && !au_test_aufs(dentry->d_sb))) {
		err = security_inode_getattr(mnt, dentry);
		if (!err)
			err = op->getattr(mnt, dentry, &st);
	}
	AuTraceErr(err);
	return err;
}
#endif
