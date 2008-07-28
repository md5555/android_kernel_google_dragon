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
 *  $Id: rename.c,v 1.47 2006/08/05 01:28:46 jro Exp $
 */

#include "unionfs.h"

static int do_rename(struct inode *old_dir, struct dentry *old_dentry,
		     struct inode *new_dir, struct dentry *new_dentry,
		     int bindex, struct dentry **wh_old)
{
	int err = 0;
	struct dentry *hidden_old_dentry;
	struct dentry *hidden_new_dentry;
	struct dentry *hidden_old_dir_dentry;
	struct dentry *hidden_new_dir_dentry;
	struct dentry *hidden_wh_dentry;
	struct dentry *hidden_wh_dir_dentry;
	char *wh_name = NULL;

	print_entry(" bindex=%d", bindex);

	print_dentry("IN: do_rename, old_dentry", old_dentry);
	print_dentry("IN: do_rename, new_dentry", new_dentry);
	dprint(PRINT_DEBUG, "do_rename for bindex = %d\n", bindex);

	hidden_new_dentry = dtohd_index(new_dentry, bindex);
	hidden_old_dentry = dtohd_index(old_dentry, bindex);

	if (!hidden_new_dentry) {
		hidden_new_dentry =
		    create_parents(new_dentry->d_parent->d_inode, new_dentry,
				   bindex);
		if (IS_ERR(hidden_new_dentry)) {
			dprint(PRINT_DEBUG,
				    "error creating directory tree for rename, bindex = %d\n",
				    bindex);
			err = PTR_ERR(hidden_new_dentry);
			goto out;
		}
	}

	wh_name = alloc_whname(new_dentry->d_name.name, new_dentry->d_name.len);
	if (IS_ERR(wh_name)) {
		err = PTR_ERR(wh_name);
		goto out;
	}

	hidden_wh_dentry =
	    LOOKUP_ONE_LEN(wh_name, hidden_new_dentry->d_parent,
			   new_dentry->d_name.len + WHLEN);
	if (IS_ERR(hidden_wh_dentry)) {
		err = PTR_ERR(hidden_wh_dentry);
		goto out;
	}

	if (hidden_wh_dentry->d_inode) {
		/* get rid of the whiteout that is existing */
		if (hidden_new_dentry->d_inode) {
			printk(KERN_WARNING
			       "Both a whiteout and a dentry exist when doing a rename!\n");
			err = -EIO;

			DPUT(hidden_wh_dentry);
			goto out;
		}

		hidden_wh_dir_dentry = lock_parent(hidden_wh_dentry);
		if (!(err = is_robranch_super(old_dentry->d_sb, bindex))) {
			err =
			    vfs_unlink(hidden_wh_dir_dentry->d_inode,
				       hidden_wh_dentry, NULL);
		}
		DPUT(hidden_wh_dentry);
		unlock_dir(hidden_wh_dir_dentry);
		if (err)
			goto out;
	} else
		DPUT(hidden_wh_dentry);

	DGET(hidden_old_dentry);
	hidden_old_dir_dentry = GET_PARENT(hidden_old_dentry);
	hidden_new_dir_dentry = GET_PARENT(hidden_new_dentry);

	lock_rename(hidden_old_dir_dentry, hidden_new_dir_dentry);

	err = is_robranch_super(old_dentry->d_sb, bindex);
	if (err)
		goto out_unlock;

	/* ready to whiteout for old_dentry.
	   caller will create the actual whiteout,
	   and must dput(*wh_old) */
	if (wh_old) {
		char *whname;
		whname = alloc_whname(old_dentry->d_name.name,
				      old_dentry->d_name.len);
		err = PTR_ERR(whname);
		if (IS_ERR(whname))
			goto out_unlock;
		*wh_old = LOOKUP_ONE_LEN(whname, hidden_old_dir_dentry,
					 old_dentry->d_name.len + WHLEN);
		KFREE(whname);
		err = PTR_ERR(*wh_old);
		if (IS_ERR(*wh_old)) {
			*wh_old = NULL;
			goto out_unlock;
		}
	}

	print_dentry("NEWBEF", new_dentry);
	print_dentry("OLDBEF", old_dentry);
	err = vfs_rename(hidden_old_dir_dentry->d_inode, hidden_old_dentry,
			 NULL, hidden_new_dir_dentry->d_inode,
			 hidden_new_dentry, NULL);
	print_dentry("NEWAFT", new_dentry);
	print_dentry("OLDAFT", old_dentry);

      out_unlock:
	unlock_rename(hidden_old_dir_dentry, hidden_new_dir_dentry);

	DPUT(hidden_old_dir_dentry);
	DPUT(hidden_new_dir_dentry);
	DPUT(hidden_old_dentry);

      out:
	if (!err) {
		/* Fixup the newdentry. */
		if (bindex < dbstart(new_dentry))
			set_dbstart(new_dentry, bindex);
		else if (bindex > dbend(new_dentry))
			set_dbend(new_dentry, bindex);
	}

	KFREE(wh_name);

	print_dentry("OUT: do_rename, old_dentry", old_dentry);
	print_dentry("OUT: do_rename, new_dentry", new_dentry);

	print_exit_status(err);
	return err;
}

static int unionfs_rename_whiteout(struct inode *old_dir,
				   struct dentry *old_dentry,
				   struct inode *new_dir,
				   struct dentry *new_dentry)
{
	int err = 0;
	int bindex, bwh_old;
	int old_bstart, old_bend;
	int new_bstart, new_bend;
	int do_copyup = -1;
	struct dentry *parent_dentry;
	int local_err = 0;
	int eio = 0;
	int revert = 0;
	struct dentry *wh_old = NULL;

	print_entry_location();

	old_bstart = dbstart(old_dentry);
	bwh_old = old_bstart;
	old_bend = dbend(old_dentry);
	parent_dentry = old_dentry->d_parent;

	new_bstart = dbstart(new_dentry);
	new_bend = dbend(new_dentry);

	/* Rename source to destination. */
	err = do_rename(old_dir, old_dentry, new_dir, new_dentry, old_bstart,
			&wh_old);
	if (err) {
		if (!IS_COPYUP_ERR(err)) {
			goto out;
		}
		do_copyup = old_bstart - 1;
	} else {
		revert = 1;
	}

	/* Unlink all instances of destination that exist to the left of
	 * bstart of source. On error, revert back, goto out.
	 */
	for (bindex = old_bstart - 1; bindex >= new_bstart; bindex--) {
		struct dentry *unlink_dentry;
		struct dentry *unlink_dir_dentry;

		unlink_dentry = dtohd_index(new_dentry, bindex);
		if (!unlink_dentry) {
			continue;
		}

		unlink_dir_dentry = lock_parent(unlink_dentry);
		if (!(err = is_robranch_super(old_dir->i_sb, bindex))) {
			err =
			    vfs_unlink(unlink_dir_dentry->d_inode,
				       unlink_dentry, NULL);
		}

		fist_copy_attr_times(new_dentry->d_parent->d_inode,
				     unlink_dir_dentry->d_inode);
		/* propagate number of hard-links */
		new_dentry->d_parent->d_inode->i_nlink =
		    get_nlinks(new_dentry->d_parent->d_inode);

		unlock_dir(unlink_dir_dentry);
		if (!err) {
			if (bindex != new_bstart) {
				DPUT(unlink_dentry);
				set_dtohd_index(new_dentry, bindex, NULL);
			}
		} else if (IS_COPYUP_ERR(err)) {
			do_copyup = bindex - 1;
		} else if (revert) {
			DPUT(wh_old);
			goto revert;
		}
	}

	if (do_copyup != -1) {
		for (bindex = do_copyup; bindex >= 0; bindex--) {
			/* copyup the file into some left directory, so that you can rename it */
			err =
			    copyup_dentry(old_dentry->d_parent->d_inode,
					  old_dentry, old_bstart, bindex, NULL,
					  old_dentry->d_inode->i_size);
			if (!err) {
				DPUT(wh_old);
				bwh_old = bindex;
				err =
				    do_rename(old_dir, old_dentry, new_dir,
					      new_dentry, bindex, &wh_old);
				break;
			}
		}
	}

	/* make it opaque */
	if (S_ISDIR(old_dentry->d_inode->i_mode)) {
		err = make_dir_opaque(old_dentry, dbstart(old_dentry));
		if (err)
			goto revert;
	}

	/* Create whiteout for source, only if:
	 * (1) There is more than one underlying instance of source.
	 * (2) We did a copy_up
	 */
	if ((old_bstart != old_bend) || (do_copyup != -1)) {
		struct dentry *hidden_parent;
		BUG_ON(!wh_old || IS_ERR(wh_old) || wh_old->d_inode
		       || bwh_old < 0);
		hidden_parent = lock_parent(wh_old);
		local_err = vfs_create(hidden_parent->d_inode, wh_old, S_IRUGO,
				       NULL);
		unlock_dir(hidden_parent);
		if (!local_err)
			set_dbopaque(old_dentry, bwh_old);
		else {
			/* We can't fix anything now, so we cop-out and use -EIO. */
			printk
			    ("<0>We can't create a whiteout for the source in rename!\n");
			err = -EIO;
		}
	}

      out:
	DPUT(wh_old);
	print_exit_status(err);
	return err;

      revert:
	/* Do revert here. */
	local_err = unionfs_refresh_hidden_dentry(new_dentry, old_bstart);
	if (local_err) {
		printk(KERN_WARNING
		       "Revert failed in rename: the new refresh failed.\n");
		eio = -EIO;
	}

	local_err = unionfs_refresh_hidden_dentry(old_dentry, old_bstart);
	if (local_err) {
		printk(KERN_WARNING
		       "Revert failed in rename: the old refresh failed.\n");
		eio = -EIO;
		goto revert_out;
	}

	if (!dtohd_index(new_dentry, bindex)
	    || !dtohd_index(new_dentry, bindex)->d_inode) {
		printk(KERN_WARNING
		       "Revert failed in rename: the object disappeared from under us!\n");
		eio = -EIO;
		goto revert_out;
	}

	if (dtohd_index(old_dentry, bindex)
	    && dtohd_index(old_dentry, bindex)->d_inode) {
		printk(KERN_WARNING
		       "Revert failed in rename: the object was created underneath us!\n");
		eio = -EIO;
		goto revert_out;
	}

	local_err =
	    do_rename(new_dir, new_dentry, old_dir, old_dentry, old_bstart,
		      NULL);

	/* If we can't fix it, then we cop-out with -EIO. */
	if (local_err) {
		printk(KERN_WARNING "Revert failed in rename!\n");
		eio = -EIO;
	}

	local_err = unionfs_refresh_hidden_dentry(new_dentry, bindex);
	if (local_err)
		eio = -EIO;
	local_err = unionfs_refresh_hidden_dentry(old_dentry, bindex);
	if (local_err)
		eio = -EIO;

      revert_out:
	if (eio)
		err = eio;
	print_exit_status(err);
	return err;
}

/*
 * Unfortunately, we cannot simply call things like dbstart() in different
 * places of the rename code because we move things around. So, we use this
 * structure to pass the necessary information around to all the places that
 * need it.
 */
struct rename_info {
	int do_copyup;
	int do_whiteout;
	int rename_ok;

	int old_bstart;
	int old_bend;
	int new_bstart;
	int new_bend;

	int isdir;		/* Is the source a directory? */
	int clobber;		/* Are we clobbering the destination? */

	int bwh_old;		/* where we create the whiteout */
	struct dentry *wh_old;	/* lookup and set by do_rename() */
};
#ifdef UNIONFS_DELETE_ALL
/*
 * Rename all occurences of source except for the leftmost destination
 */
static int __rename_all(struct inode *old_dir, struct dentry *old_dentry,
			struct inode *new_dir, struct dentry *new_dentry,
			fd_set * success_mask, struct rename_info *info)
{
	int bindex;
	int err = 0;

	print_entry_location();

	/* Loop through all the branches from right to left and rename all
	 * instances of source to destination, except the leftmost destination
	 */
	for (bindex = info->old_bend; bindex >= info->old_bstart; bindex--) {
		/* We don't rename if there is no source. */
		if (dtohd_index(old_dentry, bindex) == NULL)
			continue;

		/* we rename the bstart of destination only at the last of
		 * all operations, so that we don't lose it on error
		 */
		if (info->clobber && (bindex == info->new_bstart))
			continue;

		DPUT(info->wh_old);
		info->bwh_old = bindex;
		/* We shouldn't have a handle on this if there is no inode. */
		err =
		    do_rename(old_dir, old_dentry, new_dir, new_dentry, bindex,
			      &info->wh_old);
		if (!err) {
			/* For reverting. */
			FD_SET(bindex, success_mask);
			/* So we know not to copyup on failures the right */
			info->rename_ok = bindex;
		} else if (IS_COPYUP_ERR(err)) {
			if (info->isdir) {
				err = -EXDEV;
				break;
			}

			/* we need a whiteout... */
			info->do_whiteout = bindex - 1;

			if (bindex == info->old_bstart)
				/* ...and a copyup */
				info->do_copyup = bindex - 1;

			err = 0;	/* reset error */
		} else
			break;	/* error is set by do_rename */
	}

	print_exit_status(err);
	return err;
}

/*
 * Unlink all destinations (if they exist) to the left of the left-most
 * source
 */
static int __rename_all_unlink(struct inode *old_dir, struct dentry *old_dentry,
			       struct inode *new_dir, struct dentry *new_dentry,
			       struct rename_info *info)
{
	int bindex;

	struct dentry *unlink_dentry;
	struct dentry *unlink_dir_dentry;

	int err = 0;

	print_entry_location();

	for (bindex = info->old_bstart - 1; bindex > info->new_bstart; bindex--) {
		unlink_dentry = dtohd_index(new_dentry, bindex);
		if (!unlink_dentry)
			continue;

		/* lock, unlink if possible, copyup times, unlock */
		unlink_dir_dentry = lock_parent(unlink_dentry);
		if (!(err = is_robranch_super(old_dir->i_sb, bindex)))
			err =
			    vfs_unlink(unlink_dir_dentry->d_inode,
				       unlink_dentry, NULL);

		fist_copy_attr_times(new_dentry->d_parent->d_inode,
				     unlink_dir_dentry->d_inode);
		new_dentry->d_parent->d_inode->i_nlink =
		    get_nlinks(new_dentry->d_parent->d_inode);

		unlock_dir(unlink_dir_dentry);

		if (!err) {
			if (bindex != info->new_bstart) {
				DPUT(unlink_dentry);
				set_dtohd_index(new_dentry, bindex, NULL);
			}
		} else if (IS_COPYUP_ERR(err)) {
			if (info->isdir) {
				err = -EXDEV;
				break;
			}
			info->do_copyup = bindex - 1;

			err = 0;	/* reset error */
		} else
			break;	/* err is set by is_ro_branch_super or vfs_unlink */
	}

	print_exit_status(err);
	return err;
}

/*
 * Try to revert everything we have done in __rename_all and __rename_all_unlink
 */
static int __rename_all_revert(struct inode *old_dir, struct dentry *old_dentry,
			       struct inode *new_dir, struct dentry *new_dentry,
			       fd_set * success_mask, struct rename_info *info)
{
	int bindex;

	int err;
	int eio = 0;

	print_entry_location();

	for (bindex = info->old_bstart; bindex <= info->old_bend; bindex++) {
		if (!FD_ISSET(bindex, success_mask))
			continue;

		err = unionfs_refresh_hidden_dentry(new_dentry, bindex);
		if (err) {
			printk(KERN_WARNING "Revert failed in rename: "
			       "the new refresh failed.\n");
			eio = -EIO;
		}

		err = unionfs_refresh_hidden_dentry(old_dentry, bindex);
		if (err) {
			printk(KERN_WARNING "Revert failed in rename: "
			       "the old refresh failed.\n");
			eio = -EIO;
			continue;
		}

		if (!dtohd_index(new_dentry, bindex)
		    || !dtohd_index(new_dentry, bindex)->d_inode) {
			printk(KERN_WARNING "Revert failed in rename: "
			       "the object disappeared from under us!\n");
			eio = -EIO;
			continue;
		}

		if (dtohd_index(old_dentry, bindex)
		    && dtohd_index(old_dentry, bindex)->d_inode) {
			printk(KERN_WARNING "Revert failed in rename: "
			       "the object was created underneath us!\n");
			eio = -EIO;
			continue;
		}

		err =
		    do_rename(new_dir, new_dentry, old_dir, old_dentry, bindex,
			      NULL);
		/* If we can't fix it, then we cop-out with -EIO. */
		if (err) {
			printk(KERN_WARNING "Revert failed in rename!\n");
			eio = -EIO;
		}

		err = unionfs_refresh_hidden_dentry(new_dentry, bindex);
		if (err)
			eio = -EIO;
		err = unionfs_refresh_hidden_dentry(old_dentry, bindex);
		if (err)
			eio = -EIO;
	}

	print_exit_status(eio);
	return eio;
}

/*
 * Finish off the rename, by either over writing the last destination or
 * unlinking the last destination to the left of us
 */
static int __rename_all_clobber(struct inode *old_dir,
				struct dentry *old_dentry,
				struct inode *new_dir,
				struct dentry *new_dentry,
				struct rename_info *info)
{
	int err = 0;

	print_entry_location();

	if (dtohd_index(old_dentry, info->new_bstart)) {
		/* rename the last source, knowing we're overwriting something */
		DPUT(info->wh_old);
		info->bwh_old = info->new_bstart;
		err =
		    do_rename(old_dir, old_dentry, new_dir, new_dentry,
			      info->new_bstart, &info->wh_old);
		if (IS_COPYUP_ERR(err)) {
			if (info->isdir) {
				err = -EXDEV;
				goto out;
			}
			if (info->rename_ok > info->new_bstart) {
				if ((info->do_copyup == -1)
				    || (info->new_bstart - 1 < info->do_copyup))
					info->do_copyup = info->new_bstart - 1;
			}
			if ((info->do_whiteout == -1)
			    || (info->new_bstart - 1 < info->do_whiteout)) {
				info->do_whiteout = info->new_bstart - 1;
			}
			err = 0;	// reset error
		}
	} else if (info->new_bstart < info->old_bstart) {
		/* the newly renamed file would get hidden, let's unlink the
		 * file to the left of it */
		struct dentry *unlink_dentry;
		struct dentry *unlink_dir_dentry;

		unlink_dentry = dtohd_index(new_dentry, info->new_bstart);

		unlink_dir_dentry = lock_parent(unlink_dentry);
		if (!(err = is_robranch_super(old_dir->i_sb, info->new_bstart)))
			err = vfs_unlink(unlink_dir_dentry->d_inode,
					 unlink_dentry, NULL);

		fist_copy_attr_times(new_dentry->d_parent->d_inode,
				     unlink_dir_dentry->d_inode);
		new_dentry->d_parent->d_inode->i_nlink =
		    get_nlinks(new_dentry->d_parent->d_inode);

		unlock_dir(unlink_dir_dentry);

		if (IS_COPYUP_ERR(err)) {
			if (info->isdir) {
				err = -EXDEV;
				goto out;
			}
			if ((info->do_copyup == -1)
			    || (info->new_bstart - 1 < info->do_copyup))
				info->do_copyup = info->new_bstart - 1;

			err = 0;	// reset error
		}
	}

      out:
	print_exit_status(err);
	return err;
}

/*
 * The function is nasty, nasty, nasty, but so is rename. :(
 */
static int unionfs_rename_all(struct inode *old_dir, struct dentry *old_dentry,
			      struct inode *new_dir, struct dentry *new_dentry)
{
	struct dentry *parent_dentry = NULL;
	int err = 0;
	int eio;

	/* These variables control error handling. */
	fd_set success_mask;
	char *name = NULL;

	/* unfortunately, we have to resort to this, because dbstart/dbend would
	   return different things in different place of the rename code */
	struct rename_info info;

	info.rename_ok = FD_SETSIZE;	/* The last rename that is ok. */
	info.do_copyup = -1;	/* Where we should start copyup. */
	info.do_whiteout = -1;	/* Where we should start whiteouts of the source. */
	info.wh_old = NULL;
	info.bwh_old = -1;

	print_entry_location();

	parent_dentry = old_dentry->d_parent;
	name = KMALLOC(old_dentry->d_name.len + 1, GFP_KERNEL);
	if (!name) {
		err = -ENOMEM;
		goto out;
	}
	strncpy(name, old_dentry->d_name.name, old_dentry->d_name.len + 1);

	info.new_bstart = dbstart(new_dentry);
	info.new_bend = dbend(new_dentry);

	info.old_bstart = dbstart(old_dentry);
	info.old_bend = dbend(old_dentry);

	BUG_ON(info.new_bstart < 0);
	BUG_ON(info.old_bstart < 0);

	/* The failure mask only can deal with FD_SETSIZE entries. */
	BUG_ON(info.old_bend > FD_SETSIZE);
	BUG_ON(info.new_bend > FD_SETSIZE);
	FD_ZERO(&success_mask);

	/* Life is simpler if the dentry doesn't exist. */
	info.clobber =
	    (dtohd_index(new_dentry, info.new_bstart)->d_inode) ? 1 : 0;
	info.isdir = S_ISDIR(old_dentry->d_inode->i_mode);

	/* rename everything we can */
	err =
	    __rename_all(old_dir, old_dentry, new_dir, new_dentry,
			 &success_mask, &info);
	if (err)
		goto revert;

	/* unlink destinations even further left */
	err =
	    __rename_all_unlink(old_dir, old_dentry, new_dir, new_dentry,
				&info);
	if (err)
		goto revert;

	if (info.clobber) {
		/* Now we need to handle the leftmost of the destination. */
		err =
		    __rename_all_clobber(old_dir, old_dentry, new_dir,
					 new_dentry, &info);
		if (err)
			goto revert;
	}

	/* Copy up if necessary */
	if (info.do_copyup != -1) {
		int bindex;

		for (bindex = info.do_copyup; bindex >= 0; bindex--) {
			err =
			    copyup_dentry(old_dentry->d_parent->d_inode,
					  old_dentry, info.old_bstart, bindex,
					  NULL, old_dentry->d_inode->i_size);
			if (!err) {
				DPUT(info.wh_old);
				info.bwh_old = bindex;
				err =
				    do_rename(old_dir, old_dentry, new_dir,
					      new_dentry, bindex, &info.wh_old);
				break;
			}
		}
	}

	/* make it opaque */
	if (S_ISDIR(old_dentry->d_inode->i_mode)) {
		err = make_dir_opaque(old_dentry, dbstart(old_dentry));
		if (err)
			goto revert;
	}

	/* Create a whiteout for the source. */
	if (info.do_whiteout != -1) {
		struct dentry *hidden_parent;
		BUG_ON(info.do_whiteout < 0
		       || !info.wh_old || IS_ERR(info.wh_old)
		       || info.wh_old->d_inode || info.bwh_old < 0);
		hidden_parent = lock_parent(info.wh_old);
		err = vfs_create(hidden_parent->d_inode, info.wh_old, S_IRUGO,
				 NULL);
		unlock_dir(hidden_parent);
		if (!err)
			set_dbopaque(old_dentry, info.bwh_old);
		else {
			/* We can't fix anything now, so we -EIO. */
			printk(KERN_WARNING "We can't create a whiteout for the"
			       "source in rename!\n");
			err = -EIO;
			goto out;
		}
	}

	/* We are at the point where reverting doesn't happen. */
	goto out;

      revert:
	/* something bad happened, try to revert */
	eio =
	    __rename_all_revert(old_dir, old_dentry, new_dir, new_dentry,
				&success_mask, &info);
	if (eio)
		err = eio;

      out:
	DPUT(info.wh_old);
	KFREE(name);
	print_exit_status(err);
	return err;
}
#endif

static struct dentry *lookup_whiteout(struct dentry *dentry)
{
	char *whname;
	int bindex = -1, bstart = -1, bend = -1;
	struct dentry *parent, *hidden_parent, *wh_dentry;

	whname = alloc_whname(dentry->d_name.name, dentry->d_name.len);
	if (IS_ERR(whname))
		return (void *)whname;

	parent = GET_PARENT(dentry);
	lock_dentry(parent);
	bstart = dbstart(parent);
	bend = dbend(parent);
	wh_dentry = ERR_PTR(-ENOENT);
	for (bindex = bstart; bindex <= bend; bindex++) {
		hidden_parent = dtohd_index(parent, bindex);
		if (!hidden_parent)
			continue;
		wh_dentry =
		    LOOKUP_ONE_LEN(whname, hidden_parent,
				   dentry->d_name.len + WHLEN);
		if (IS_ERR(wh_dentry))
			continue;
		if (wh_dentry->d_inode)
			break;
		DPUT(wh_dentry);
		wh_dentry = ERR_PTR(-ENOENT);
	}
	unlock_dentry(parent);
	DPUT(parent);
	KFREE(whname);
	return wh_dentry;
}

/* We can't copyup a directory, because it may involve huge
 * numbers of children, etc.  Doing that in the kernel would
 * be bad, so instead we let the userspace recurse and ask us
 * to copy up each file separately
 */
static int may_rename_dir(struct dentry *dentry)
{
	int err, bstart;

	err = check_empty(dentry, NULL);
	if (err == -ENOTEMPTY) {
		if (is_robranch(dentry))
			return -EXDEV;
	} else if (err)
		return err;

	bstart = dbstart(dentry);
	if (dbend(dentry) == bstart || dbopaque(dentry) == bstart)
		return 0;

	set_dbstart(dentry, bstart + 1);
	err = check_empty(dentry, NULL);
	set_dbstart(dentry, bstart);
	if (err == -ENOTEMPTY)
		err = -EXDEV;
	return err;
}

int unionfs_rename(struct inode *old_dir, struct dentry *old_dentry,
		   struct inode *new_dir, struct dentry *new_dentry)
{
	int err = 0;
	struct dentry *wh_dentry;

	print_entry_location();

	double_lock_dentry(old_dentry, new_dentry);

	checkinode(old_dir, "unionfs_rename-old_dir");
	checkinode(new_dir, "unionfs_rename-new_dir");
	print_dentry("IN: unionfs_rename, old_dentry", old_dentry);
	print_dentry("IN: unionfs_rename, new_dentry", new_dentry);

	if (!S_ISDIR(old_dentry->d_inode->i_mode))
		err = unionfs_partial_lookup(old_dentry);
	else
		err = may_rename_dir(old_dentry);

	if (err)
		goto out;

	err = unionfs_partial_lookup(new_dentry);
	if (err)
		goto out;

	/*
	 * if new_dentry is already hidden because of whiteout,
	 * simply override it even if the whiteouted dir is not empty.
	 */
	wh_dentry = lookup_whiteout(new_dentry);
	if (!IS_ERR(wh_dentry))
		DPUT(wh_dentry);
	else if (new_dentry->d_inode) {
		if (S_ISDIR(old_dentry->d_inode->i_mode) !=
		    S_ISDIR(new_dentry->d_inode->i_mode)) {
			err =
			    S_ISDIR(old_dentry->d_inode->
				    i_mode) ? -ENOTDIR : -EISDIR;
			goto out;
		}

		if (S_ISDIR(new_dentry->d_inode->i_mode)) {
			struct unionfs_dir_state *namelist;
			/* check if this unionfs directory is empty or not */
			err = check_empty(new_dentry, &namelist);
			if (err)
				goto out;

			if (!is_robranch(new_dentry))
				err = delete_whiteouts(new_dentry,
						dbstart(new_dentry),
						namelist);

			free_rdstate(namelist);

			if (err)
				goto out;
		}
	}
#ifdef UNIONFS_DELETE_ALL
	if (IS_SET(old_dir->i_sb, DELETE_ALL))
		err = unionfs_rename_all(old_dir, old_dentry, new_dir,
					 new_dentry);
	else
#endif
		err = unionfs_rename_whiteout(old_dir, old_dentry, new_dir,
					      new_dentry);

      out:
	checkinode(new_dir, "post unionfs_rename-new_dir");
	print_dentry("OUT: unionfs_rename, old_dentry", old_dentry);

	if (err) {
		/* clear the new_dentry stuff created */
		d_drop(new_dentry);
	} else {
		/* force re-lookup since the dir on ro branch is not renamed,
		   and hidden dentries still indicate the un-renamed ones. */
		if (S_ISDIR(old_dentry->d_inode->i_mode))
			atomic_dec(&dtopd(old_dentry)->udi_generation);
		print_dentry("OUT: unionfs_rename, new_dentry",
				  new_dentry);
	}

	unlock_dentry(new_dentry);
	unlock_dentry(old_dentry);
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
