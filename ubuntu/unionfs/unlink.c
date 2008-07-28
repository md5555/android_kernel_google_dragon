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
 *  $Id: unlink.c,v 1.44 2006/08/05 01:28:46 jro Exp $
 */

#include "unionfs.h"

#ifdef UNIONFS_DELETE_ALL
static int unionfs_unlink_all(struct inode *dir, struct dentry *dentry)
{
	struct dentry *hidden_dentry;
	struct dentry *hidden_dir_dentry;
	int bstart, bend, bindex;
	int err = 0;
	int global_err = 0;

	print_entry_location();

	if ((err = unionfs_partial_lookup(dentry)))
		goto out;

	bstart = dbstart(dentry);
	bend = dbend(dentry);

	for (bindex = bend; bindex >= bstart; bindex--) {
		hidden_dentry = dtohd_index(dentry, bindex);
		if (!hidden_dentry)
			continue;

		hidden_dir_dentry = lock_parent(hidden_dentry);

		/* avoid destroying the hidden inode if the file is in use */
		DGET(hidden_dentry);
		if (!(err = is_robranch_super(dentry->d_sb, bindex)))
			err = vfs_unlink(hidden_dir_dentry->d_inode,
					 hidden_dentry, NULL);
		DPUT(hidden_dentry);
		fist_copy_attr_times(dir, hidden_dir_dentry->d_inode);
		unlock_dir(hidden_dir_dentry);

		if (err) {
			/* passup the last error we got */
			if (!IS_COPYUP_ERR(err))
				goto out;
			global_err = err;
		}
	}

	/* check if encountered error in the above loop */
	if (global_err) {
		/* If we failed in the leftmost branch, then err will be set
		 * and we should move one over to create the whiteout.
		 * Otherwise, we should try in the leftmost branch. */
		if (err) {
			if (dbstart(dentry) == 0) {
				goto out;
			}
			err = create_whiteout(dentry, dbstart(dentry) - 1);
		} else {
			err = create_whiteout(dentry, dbstart(dentry));
		}
	} else if (dbopaque(dentry) != -1) {
		/* There is a hidden lower-priority file with the same name. */
		err = create_whiteout(dentry, dbopaque(dentry));
	}
      out:
	/* propagate number of hard-links */
	if (dentry->d_inode->i_nlink != 0) {
		dentry->d_inode->i_nlink = get_nlinks(dentry->d_inode);
		if (!err && global_err)
			dentry->d_inode->i_nlink--;
	}
	/* We don't want to leave negative leftover dentries for revalidate. */
	if (!err && (global_err || dbopaque(dentry) != -1))
		update_bstart(dentry);

	print_exit_status(err);
	return err;
}
#endif
static int unionfs_unlink_whiteout(struct inode *dir, struct dentry *dentry)
{
	struct dentry *hidden_dentry;
	struct dentry *hidden_dir_dentry;
	int bindex;
	int err = 0;

	print_entry_location();

	if ((err = unionfs_partial_lookup(dentry)))
		goto out;

	bindex = dbstart(dentry);

	hidden_dentry = dtohd_index(dentry, bindex);
	if (!hidden_dentry)
		goto out;

	hidden_dir_dentry = lock_parent(hidden_dentry);

	/* avoid destroying the hidden inode if the file is in use */
	DGET(hidden_dentry);
	if (!(err = is_robranch_super(dentry->d_sb, bindex)))
		err = vfs_unlink(hidden_dir_dentry->d_inode, hidden_dentry,
				 NULL);
	DPUT(hidden_dentry);
	fist_copy_attr_times(dir, hidden_dir_dentry->d_inode);
	unlock_dir(hidden_dir_dentry);

	if (err && !IS_COPYUP_ERR(err))
		goto out;

	if (err) {
		if (dbstart(dentry) == 0)
			goto out;

		err = create_whiteout(dentry, dbstart(dentry) - 1);
	} else if (dbopaque(dentry) != -1) {
		/* There is a hidden lower-priority file with the same name. */
		err = create_whiteout(dentry, dbopaque(dentry));
	} else {
		err = create_whiteout(dentry, dbstart(dentry));
	}

      out:
	if (!err)
		dentry->d_inode->i_nlink--;

	/* We don't want to leave negative leftover dentries for revalidate. */
	if (!err && (dbopaque(dentry) != -1))
		update_bstart(dentry);

	print_exit_status(err);
	return err;

}

int unionfs_unlink(struct inode *dir, struct dentry *dentry)
{
	int err = 0;

	print_entry_location();
	lock_dentry(dentry);
	print_dentry("IN unionfs_unlink", dentry);

#ifdef UNIONFS_DELETE_ALL
	if (IS_SET(dir->i_sb, DELETE_ALL))
		err = unionfs_unlink_all(dir, dentry);
	else
#endif
		err = unionfs_unlink_whiteout(dir, dentry);
	/* call d_drop so the system "forgets" about us */
	if (!err)
		d_drop(dentry);

	unlock_dentry(dentry);
	print_exit_status(err);
	return err;
}

static int unionfs_rmdir_first(struct inode *dir, struct dentry *dentry,
			       struct unionfs_dir_state *namelist)
{
	int err;
	struct dentry *hidden_dentry;
	struct dentry *hidden_dir_dentry = NULL;

	print_entry_location();
	print_dentry("IN unionfs_rmdir_first: ", dentry);

	/* Here we need to remove whiteout entries. */
	err = delete_whiteouts(dentry, dbstart(dentry), namelist);
	if (err) {
		goto out;
	}

	hidden_dentry = dtohd(dentry);

	hidden_dir_dentry = lock_parent(hidden_dentry);

	/* avoid destroying the hidden inode if the file is in use */
	DGET(hidden_dentry);
	if (!(err = is_robranch(dentry))) {
		err = vfs_rmdir(hidden_dir_dentry->d_inode, hidden_dentry,
				NULL);
	}
	DPUT(hidden_dentry);

	fist_copy_attr_times(dir, hidden_dir_dentry->d_inode);
	/* propagate number of hard-links */
	dentry->d_inode->i_nlink = get_nlinks(dentry->d_inode);

      out:
	if (hidden_dir_dentry) {
		unlock_dir(hidden_dir_dentry);
	}
	print_dentry("OUT unionfs_rmdir_first: ", dentry);
	print_exit_status(err);
	return err;
}

#ifdef UNIONFS_DELETE_ALL
static int unionfs_rmdir_all(struct inode *dir, struct dentry *dentry,
			     struct unionfs_dir_state *namelist)
{
	struct dentry *hidden_dentry;
	struct dentry *hidden_dir_dentry;
	int bstart, bend, bindex;
	int err = 0;
	int global_err = 0;

	print_entry_location();
	print_dentry("IN unionfs_rmdir_all: ", dentry);

	bstart = dbstart(dentry);
	bend = dbend(dentry);

	for (bindex = bend; bindex >= bstart; bindex--) {
		hidden_dentry = dtohd_index(dentry, bindex);
		if (!hidden_dentry)
			continue;

		hidden_dir_dentry = lock_parent(hidden_dentry);
		if (S_ISDIR(hidden_dentry->d_inode->i_mode)) {
			err = delete_whiteouts(dentry, bindex, namelist);
			if (!err
			    && !(err =
				 is_robranch_super(dentry->d_sb, bindex))) {
				err =
				    vfs_rmdir(hidden_dir_dentry->d_inode,
					      hidden_dentry, NULL);
			}
		} else {
			err = -EISDIR;
		}

		fist_copy_attr_times(dir, hidden_dir_dentry->d_inode);
		unlock_dir(hidden_dir_dentry);
		if (err) {
			int local_err =
			    unionfs_refresh_hidden_dentry(dentry, bindex);
			if (local_err) {
				err = local_err;
				goto out;
			}

			if (!IS_COPYUP_ERR(err) && err != -ENOTEMPTY
			    && err != -EISDIR)
				goto out;

			global_err = err;
		}
	}

	/* check if encountered error in the above loop */
	if (global_err) {
		/* If we failed in the leftmost branch, then err will be set and we should
		 * move one over to create the whiteout.  Otherwise, we should try in the
		 * leftmost branch.
		 */
		if (err) {
			if (dbstart(dentry) == 0) {
				goto out;
			}
			err = create_whiteout(dentry, dbstart(dentry) - 1);
		} else {
			err = create_whiteout(dentry, dbstart(dentry));
		}
	} else {
		err = create_whiteout(dentry, dbstart(dentry));
	}

      out:
	/* propagate number of hard-links */
	dentry->d_inode->i_nlink = get_nlinks(dentry->d_inode);

	print_dentry("OUT unionfs_rmdir_all: ", dentry);
	print_exit_status(err);
	return err;
}
#endif
int unionfs_rmdir(struct inode *dir, struct dentry *dentry)
{
	int err = 0;
	struct unionfs_dir_state *namelist = NULL;

	print_entry_location();
	lock_dentry(dentry);
	print_dentry("IN unionfs_rmdir: ", dentry);

	/* check if this unionfs directory is empty or not */
	err = check_empty(dentry, &namelist);
	if (err) {
#if 0
		/* vfs_rmdir(our caller) unhashed the dentry.  This will recover
		 * the Unionfs inode number for the directory itself, but the
		 * children are already lost.  It seems that tmpfs manages its
		 * way around this by upping the refcount on everything.
		 *
		 * Even if we do this, we still lose the inode numbers of the
		 * children.  The best way to fix this is to fix the VFS (or
		 * use persistent inode maps). */
		if (d_unhashed(dentry))
			d_rehash(dentry);
#endif
		goto out;
	}
#ifdef UNIONFS_DELETE_ALL
	if (IS_SET(dir->i_sb, DELETE_ALL)) {
		/* delete all. */
		err = unionfs_rmdir_all(dir, dentry, namelist);
	} else {		/* Delete the first directory. */
#endif
		err = unionfs_rmdir_first(dir, dentry, namelist);
		/* create whiteout */
		if (!err) {
			err = create_whiteout(dentry, dbstart(dentry));
		} else {
			int new_err;

			if (dbstart(dentry) == 0)
				goto out;

			/* exit if the error returned was NOT -EROFS */
			if (!IS_COPYUP_ERR(err))
				goto out;

			new_err = create_whiteout(dentry, dbstart(dentry) - 1);
			if (new_err != -EEXIST)
				err = new_err;
		}

#ifdef UNIONFS_DELETE_ALL
	}
#endif
      out:
	/* call d_drop so the system "forgets" about us */
	if (!err)
		d_drop(dentry);

	if (namelist)
		free_rdstate(namelist);

	unlock_dentry(dentry);
	print_exit_status(err);
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
