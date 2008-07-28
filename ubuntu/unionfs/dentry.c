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
 *  $Id: dentry.c,v 1.77 2006/08/05 01:28:46 jro Exp $
 */

#include "unionfs.h"

/* declarations added for "sparse" */
extern int unionfs_d_revalidate_wrap(struct dentry *dentry,
				     struct nameidata *nd);
extern void unionfs_d_release(struct dentry *dentry);
extern void unionfs_d_iput(struct dentry *dentry, struct inode *inode);

/*
 * THIS IS A BOOLEAN FUNCTION: returns 1 if valid, 0 otherwise.
 */
int unionfs_d_revalidate(struct dentry *dentry, struct nameidata *nd)
{
	int valid = 1;		/* default is valid (1); invalid is 0. */
	struct dentry *hidden_dentry;
	int bindex, bstart, bend;
	int sbgen, dgen;
	int positive = 0;
	int locked = 0;
	int restart = 0;
	int interpose_flag;

	struct nameidata lowernd;

	if(nd)
		memcpy(&lowernd, nd, sizeof(struct nameidata));
	else
		memset(&lowernd, 0, sizeof(struct nameidata));

	print_util_entry_location();

      restart:
	verify_locked(dentry);

	/* if the dentry is unhashed, do NOT revalidate */
	if (d_deleted(dentry)) {
		dprint(PRINT_DEBUG, "unhashed dentry being revalidated: %*s\n",
			    dentry->d_name.len, dentry->d_name.name);
		goto out;
	}

	BUG_ON(dbstart(dentry) == -1);
	if (dentry->d_inode)
		positive = 1;
	dgen = atomic_read(&dtopd(dentry)->udi_generation);
	sbgen = atomic_read(&stopd(dentry->d_sb)->usi_generation);
	/* If we are working on an unconnected dentry, then there is no
	 * revalidation to be done, because this file does not exist within the
	 * namespace, and Unionfs operates on the namespace, not data.
	 */
	if (sbgen != dgen) {
		struct dentry *result;
		int pdgen;

		unionfs_read_lock(dentry->d_sb);
		locked = 1;

		/* The root entry should always be valid */
		BUG_ON(IS_ROOT(dentry));

		/* We can't work correctly if our parent isn't valid. */
		pdgen = atomic_read(&dtopd(dentry->d_parent)->udi_generation);
		if (!restart && (pdgen != sbgen)) {
			unionfs_read_unlock(dentry->d_sb);
			locked = 0;
			/* We must be locked before our parent. */
			if (!
			    (dentry->d_parent->d_op->
			     d_revalidate(dentry->d_parent, nd))) {
				valid = 0;
				goto out;
			}
			restart = 1;
			goto restart;
		}
		BUG_ON(pdgen != sbgen);

		/* Free the pointers for our inodes and this dentry. */
		bstart = dbstart(dentry);
		bend = dbend(dentry);
		if (bstart >= 0) {
			struct dentry *hidden_dentry;
			for (bindex = bstart; bindex <= bend; bindex++) {
				hidden_dentry =
				    dtohd_index_nocheck(dentry, bindex);
				if (!hidden_dentry)
					continue;
				DPUT(hidden_dentry);
			}
		}
		set_dbstart(dentry, -1);
		set_dbend(dentry, -1);

		interpose_flag = INTERPOSE_REVAL_NEG;
		if (positive) {
			interpose_flag = INTERPOSE_REVAL;
			mutex_lock(&dentry->d_inode->i_mutex);
			bstart = ibstart(dentry->d_inode);
			bend = ibend(dentry->d_inode);
			if (bstart >= 0) {
				struct inode *hidden_inode;
				for (bindex = bstart; bindex <= bend; bindex++) {
					hidden_inode =
					    itohi_index(dentry->d_inode,
							bindex);
					if (!hidden_inode)
						continue;
					IPUT(hidden_inode);
				}
			}
			KFREE(itohi_ptr(dentry->d_inode));
			itohi_ptr(dentry->d_inode) = NULL;
			ibstart(dentry->d_inode) = -1;
			ibend(dentry->d_inode) = -1;
			mutex_unlock(&dentry->d_inode->i_mutex);
		}

		result = unionfs_lookup_backend(dentry, &lowernd, interpose_flag);
		if (result) {
			if (IS_ERR(result)) {
				valid = 0;
				goto out;
			}
			/* current unionfs_lookup_backend() doesn't return
			   a valid dentry */
			DPUT(dentry);
			dentry = result;
		}

		if (positive && itopd(dentry->d_inode)->uii_stale) {
			make_stale_inode(dentry->d_inode);
			d_drop(dentry);
			valid = 0;
			goto out;
		}
		goto out;
	}

	/* The revalidation must occur across all branches */
	bstart = dbstart(dentry);
	bend = dbend(dentry);
	BUG_ON(bstart == -1);
	for (bindex = bstart; bindex <= bend; bindex++) {
		hidden_dentry = dtohd_index(dentry, bindex);
		if (!hidden_dentry || !hidden_dentry->d_op
		    || !hidden_dentry->d_op->d_revalidate)
			continue;

		if (!hidden_dentry->d_op->d_revalidate(hidden_dentry, nd))
			valid = 0;
	}

	if (!dentry->d_inode)
		valid = 0;
	if (valid)
		fist_copy_attr_all(dentry->d_inode, itohi(dentry->d_inode));

      out:
	if (locked)
		unionfs_read_unlock(dentry->d_sb);
	print_dentry("revalidate out", dentry);
	print_util_exit_status(valid);
	return valid;
}

int unionfs_d_revalidate_wrap(struct dentry *dentry, struct nameidata *nd)
{
	int err;

	print_entry_location();
	lock_dentry(dentry);

	err = unionfs_d_revalidate(dentry, nd);

	unlock_dentry(dentry);
	print_exit_status(err);
	return err;
}

void unionfs_d_release(struct dentry *dentry)
{
	struct dentry *hidden_dentry;
	int bindex, bstart, bend;

	print_entry_location();
	/* There is no reason to lock the dentry, because we have the only
	 * reference, but the printing functions verify that we have a lock
	 * on the dentry before calling dbstart, etc. */
	lock_dentry(dentry);
	print_dentry_nocheck("unionfs_d_release IN dentry", dentry);

	/* this could be a negative dentry, so check first */
	if (!dtopd(dentry)) {
		dprint(PRINT_DEBUG, "dentry without private data: %*s",
			    dentry->d_name.len, dentry->d_name.name);
		goto out;
	} else if (dbstart(dentry) < 0) {
		/* this is due to a failed lookup */
		/* the failed lookup has a dtohd_ptr set to null,
		   but this is a better check */
		dprint(PRINT_DEBUG, "dentry without hidden dentries : %*s",
			    dentry->d_name.len, dentry->d_name.name);
		goto out_free;
	}

	/* Release all the hidden dentries */
	bstart = dbstart(dentry);
	bend = dbend(dentry);
	for (bindex = bstart; bindex <= bend; bindex++) {
		hidden_dentry = dtohd_index(dentry, bindex);
		DPUT(hidden_dentry);
		set_dtohd_index(dentry, bindex, NULL);
	}
	/* free private data (unionfs_dentry_info) here */
	KFREE(dtohd_ptr(dentry));
	dtohd_ptr(dentry) = NULL;
      out_free:
	/* No need to unlock it, because it is disappeared. */
#ifdef TRACKLOCK
	printk("DESTROYLOCK:%p\n", dentry);
#endif
	free_dentry_private_data(dtopd(dentry));
	dtopd_lhs(dentry) = NULL;	/* just to be safe */
      out:
	print_exit_location();
}

/*
 * we don't really need unionfs_d_iput, because dentry_iput will call iput() if
 * unionfs_d_iput is not defined. We left this implemented for ease of
 * tracing/debugging.
 */
void unionfs_d_iput(struct dentry *dentry, struct inode *inode)
{
	print_entry_location();
	IPUT(inode);
	print_exit_location();
}

struct dentry_operations unionfs_dops = {
	.d_revalidate = unionfs_d_revalidate_wrap,
	.d_release = unionfs_d_release,
	.d_iput = unionfs_d_iput,
};

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
