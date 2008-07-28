/*
 * Copyright (c) 2003-2006 Erez Zadok
 * Copyright (c) 2003-2006 Charles P. Wright
 * Copyright (c) 2005-2006 Josef 'Jeff' Sipek
 * Copyright (c) 2005-2006 Junjiro Okajima
 * Copyright (c) 2005      Arun M. Krishnakumar
 * Copyright (c) 2005-2006 David P. Quigley
 * Copyright (c) 2003-2004 Mohammad Nayyer Zubair
 * Copyright (c) 2003      Puja Gupta
 * Copyright (c) 2003      Harikesavan Krishnan
 * Copyright (c) 2003-2006 Stony Brook University
 * Copyright (c) 2003-2006 The Research Foundation of State University of New York
 *
 * For specific licensing information, see the COPYING file distributed with
 * this package.
 *
 * This Copyright notice must be kept intact and distributed with all sources.
 */
/*
 *  $Id: subr.c,v 1.142 2006/09/21 18:19:36 jsipek Exp $
 */

#include "unionfs.h"
#include <linux/security.h>

/* Pass an unionfs dentry and an index.  It will try to create a whiteout
 * for the filename in dentry, and will try in branch 'index'.  On error,
 * it will proceed to a branch to the left.
 */
int create_whiteout(struct dentry *dentry, int start)
{
	int bstart, bend, bindex;
	struct dentry *hidden_dir_dentry;
	struct dentry *hidden_dentry;
	struct dentry *hidden_wh_dentry;
	char *name = NULL;
	int err = -EINVAL;

	print_entry("start = %d", start);

	verify_locked(dentry);

	print_dentry("IN create_whiteout", dentry);
	bstart = dbstart(dentry);
	bend = dbend(dentry);

	/* create dentry's whiteout equivalent */
	name = alloc_whname(dentry->d_name.name, dentry->d_name.len);
	if (IS_ERR(name)) {
		err = PTR_ERR(name);
		goto out;
	}

	for (bindex = start; bindex >= 0; bindex--) {
		hidden_dentry = dtohd_index(dentry, bindex);

		if (!hidden_dentry) {
			/* if hidden dentry is not present, create the entire
			 * hidden dentry directory structure and go ahead.
			 * Since we want to just create whiteout, we only want
			 * the parent dentry, and hence get rid of this dentry.
			 */
			hidden_dentry = create_parents(dentry->d_inode,
						       dentry, bindex);
			if (!hidden_dentry || IS_ERR(hidden_dentry)) {
				dprint(PRINT_DEBUG_WHITEOUT,
				       "create_parents failed for bindex = %d\n",
				       bindex);
				continue;
			}
		}
		hidden_wh_dentry =
		    LOOKUP_ONE_LEN(name, hidden_dentry->d_parent,
				   dentry->d_name.len + WHLEN);
		if (IS_ERR(hidden_wh_dentry))
			continue;

		/* The whiteout already exists. This used to be impossible, but
		 * now is possible because of opaqueness. */
		if (hidden_wh_dentry->d_inode) {
			DPUT(hidden_wh_dentry);
			err = 0;
			goto out;
		}

		hidden_dir_dentry = lock_parent(hidden_wh_dentry);
		if (!(err = is_robranch_super(dentry->d_sb, bindex))) {
			err =
			    vfs_create(hidden_dir_dentry->d_inode,
				       hidden_wh_dentry,
				       ~current->fs->umask & S_IRWXUGO, NULL);

		}
		unlock_dir(hidden_dir_dentry);
		DPUT(hidden_wh_dentry);

		if (!err)
			break;

		if (!IS_COPYUP_ERR(err))
			break;
	}

	/* set dbopaque  so that lookup will not proceed after this branch */
	if (!err)
		set_dbopaque(dentry, bindex);

	print_dentry("OUT create_whiteout", dentry);
      out:
	KFREE(name);
	print_exit_status(err);
	return err;
}

/* This is a helper function for rename, which ends up with hosed over dentries
 * when it needs to revert. */
int unionfs_refresh_hidden_dentry(struct dentry *dentry, int bindex)
{
	struct dentry *hidden_dentry;
	struct dentry *hidden_parent;
	int err = 0;

	print_entry(" bindex = %d", bindex);

	verify_locked(dentry);
	lock_dentry(dentry->d_parent);
	hidden_parent = dtohd_index(dentry->d_parent, bindex);
	unlock_dentry(dentry->d_parent);

	BUG_ON(!S_ISDIR(hidden_parent->d_inode->i_mode));

	hidden_dentry =
	    LOOKUP_ONE_LEN(dentry->d_name.name, hidden_parent,
			   dentry->d_name.len);
	if (IS_ERR(hidden_dentry)) {
		err = PTR_ERR(hidden_dentry);
		goto out;
	}

	if (dtohd_index(dentry, bindex))
		DPUT(dtohd_index(dentry, bindex));
	if (itohi_index(dentry->d_inode, bindex)) {
		IPUT(itohi_index(dentry->d_inode, bindex));
		set_itohi_index(dentry->d_inode, bindex, NULL);
	}
	if (!hidden_dentry->d_inode) {
		DPUT(hidden_dentry);
		set_dtohd_index(dentry, bindex, NULL);
	} else {
		set_dtohd_index(dentry, bindex, hidden_dentry);
		set_itohi_index(dentry->d_inode, bindex,
				IGRAB(hidden_dentry->d_inode));
	}

      out:
	print_exit_status(err);
	return err;
}

int make_dir_opaque(struct dentry *dentry, int bindex)
{
	int err;
	struct dentry *hidden_dentry, *diropq;
	struct inode *hidden_dir;

	hidden_dentry = dtohd_index(dentry, bindex);
	hidden_dir = hidden_dentry->d_inode;
	BUG_ON(!S_ISDIR(dentry->d_inode->i_mode)
	       || !S_ISDIR(hidden_dir->i_mode));

	mutex_lock(&hidden_dir->i_mutex);
	diropq = LOOKUP_ONE_LEN(UNIONFS_DIR_OPAQUE, hidden_dentry,
				sizeof(UNIONFS_DIR_OPAQUE) - 1);
	err = PTR_ERR(diropq);
	if (IS_ERR(diropq))
		goto out;
	err = 0;

	if (!diropq->d_inode)
		err = vfs_create(hidden_dir, diropq, S_IRUGO, NULL);
	DPUT(diropq);
	if (!err)
		set_dbopaque(dentry, bindex);

      out:
	mutex_unlock(&hidden_dir->i_mutex);
	return err;
}

/*
 *
 * vim:shiftwidth=8
 * vim:tabstop=8
 *
 * For Emacs:
 * Local variables:
 * c-basic-offset: 8
 * c-comment-only-line-offset: 0
 * c-offsets-alist: ((statement-block-intro . +) (knr-argdecl-intro . 0)
 *              (substatement-open . 0) (label . 0) (statement-cont . +))
 * indent-tabs-mode: t
 * tab-width: 8
 * End:
 */
