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
 *  $Id: commonfops.c,v 1.61 2006/08/05 01:28:46 jro Exp $
 */

#include "unionfs.h"

/* We only need this function here, but it could get promoted to unionfs.h, if
 * other things need a generation specific branch putting function. */
static inline void branchput_gen(int generation, struct super_block *sb,
				 int index)
{
	struct putmap *putmap;

	if (generation == atomic_read(&stopd(sb)->usi_generation)) {
		branchput(sb, index);
		return;
	}

	BUG_ON(stopd(sb)->usi_firstputmap > generation);
	BUG_ON(stopd(sb)->usi_lastputmap < generation);

	putmap =
	    stopd(sb)->usi_putmaps[generation - stopd(sb)->usi_firstputmap];
	BUG_ON(index < 0);
	BUG_ON(index > putmap->bend);
	BUG_ON(putmap->map[index] < 0);
	branchput(sb, putmap->map[index]);
	if (atomic_dec_and_test(&putmap->count)) {
		stopd(sb)->usi_putmaps[generation - stopd(sb)->usi_firstputmap]
		    = NULL;
		dprint(PRINT_DEBUG, "Freeing putmap %d.\n", generation);
		KFREE(putmap);
	}
}

static char *get_random_name(int size, unsigned char *name)
{
	int i;
	int j;
	unsigned char *tmpbuf = NULL;

	if (size <= WHLEN)
		return NULL;

	if (!name)
		name = KMALLOC(size + 1, GFP_KERNEL);
	if (!name) {
		name = ERR_PTR(-ENOMEM);
		goto out;
	}
	strncpy(name, WHPFX, WHLEN);

	tmpbuf = KMALLOC(size, GFP_KERNEL);
	if (!tmpbuf) {
		KFREE(name);
		name = ERR_PTR(-ENOMEM);
		goto out;
	}

	get_random_bytes((void *)tmpbuf, (size - 3) / 2);

	j = WHLEN;
	i = 0;
	while ((i < (size - 3) / 2) && (j < size)) {
		/* get characters in the 0-9, A-F range */

		name[j] =
		    (tmpbuf[i] % 16) <
		    10 ? (tmpbuf[i] % 16) + '0' : (tmpbuf[i] % 16) + 'a';
		j++;
		if (j == size)
			break;
		name[j] =
		    (tmpbuf[i] >> 4) <
		    10 ? (tmpbuf[i] >> 4) + '0' : (tmpbuf[i] >> 4) + 'a';
		j++;

		i++;
	}

	name[size] = '\0';

      out:
	KFREE(tmpbuf);
	return (name);

}

static int copyup_deleted_file(struct file *file, struct dentry *dentry,
			       int bstart, int bindex)
{
	int attempts = 0;
	int err;
	int exists = 1;
	char *name = NULL;
	struct dentry *tmp_dentry = NULL;
	struct dentry *hidden_dentry = NULL;
	struct dentry *hidden_dir_dentry = NULL;

	print_entry_location();

	/* Try five times to get a unique file name, fail after that.  Five is
	 * simply a magic number, because we shouldn't try forever.  */
	while (exists) {
		/* The first call allocates, the subsequent ones reuse. */
		name = get_random_name(UNIONFS_TMPNAM_LEN, name);
		err = -ENOMEM;
		if (!name)
			goto out;
		//XXX: Why do we do this every time? bstart never changes?
		hidden_dentry = dtohd_index(dentry, bstart);

		tmp_dentry = LOOKUP_ONE_LEN(name, hidden_dentry->d_parent,
					    UNIONFS_TMPNAM_LEN);
		err = PTR_ERR(tmp_dentry);
		if (IS_ERR(tmp_dentry))
			goto out;
		exists = tmp_dentry->d_inode ? 1 : 0;
		DPUT(tmp_dentry);

		err = -EEXIST;
		if (++attempts > 5)
			goto out;
	}

	err = copyup_named_file(dentry->d_parent->d_inode, file, name, bstart,
				bindex, file->f_dentry->d_inode->i_size);
	if (err)
		goto out;

	/* bring it to the same state as an unlinked file */
	hidden_dentry = dtohd_index(dentry, dbstart(dentry));
	hidden_dir_dentry = lock_parent(hidden_dentry);
	err = vfs_unlink(hidden_dir_dentry->d_inode, hidden_dentry, NULL);
	unlock_dir(hidden_dir_dentry);

      out:
	KFREE(name);
	print_exit_status(err);
	return err;
}

int unionfs_file_revalidate(struct file *file, int willwrite)
{
	struct super_block *sb;
	struct dentry *dentry;
	int sbgen, fgen, dgen;
	int bindex, bstart, bend;
	struct file *hidden_file;
	struct dentry *hidden_dentry;
	int size;

	int err = 0;

	print_entry(" file = %p", file);

	dentry = file->f_dentry;
	lock_dentry(dentry);
	sb = dentry->d_sb;
	unionfs_read_lock(sb);
	if (!unionfs_d_revalidate(dentry, NULL) && !d_deleted(dentry)) {
		err = -ESTALE;
		goto out;
	}
	print_dentry("file revalidate in", dentry);

	sbgen = atomic_read(&stopd(sb)->usi_generation);
	dgen = atomic_read(&dtopd(dentry)->udi_generation);
	fgen = atomic_read(&ftopd(file)->ufi_generation);

	BUG_ON(sbgen > dgen);

	/* There are two cases we are interested in.  The first is if the
	 * generation is lower than the super-block.  The second is if someone
	 * has copied up this file from underneath us, we also need to refresh
	 * things. */
	if (!d_deleted(dentry) &&
	    ((sbgen > fgen) || (dbstart(dentry) != fbstart(file)))) {
		/* First we throw out the existing files. */
		bstart = fbstart(file);
		bend = fbend(file);
		for (bindex = bstart; bindex <= bend; bindex++) {
			if (ftohf_index(file, bindex)) {
				branchput_gen(fgen, dentry->d_sb, bindex);
				fput(ftohf_index(file, bindex));
			}
		}

		if (ftohf_ptr(file)) {
			KFREE(ftohf_ptr(file));
			ftohf_ptr(file) = NULL;
		}

		/* Now we reopen the file(s) as in unionfs_open. */
		bstart = fbstart(file) = dbstart(dentry);
		bend = fbend(file) = dbend(dentry);

		size = sizeof(struct file *) * sbmax(sb);
		ftohf_ptr(file) = KZALLOC(size, GFP_KERNEL);
		if (!ftohf_ptr(file)) {
			err = -ENOMEM;
			goto out;
		}

		if (S_ISDIR(dentry->d_inode->i_mode)) {
			/* We need to open all the files. */
			for (bindex = bstart; bindex <= bend; bindex++) {
				hidden_dentry = dtohd_index(dentry, bindex);
				if (!hidden_dentry)
					continue;

				DGET(hidden_dentry);
				mntget(stohiddenmnt_index(sb, bindex));
				branchget(sb, bindex);

				hidden_file =
				    DENTRY_OPEN(hidden_dentry,
						stohiddenmnt_index(sb, bindex),
						file->f_flags);
				if (IS_ERR(hidden_file)) {
					err = PTR_ERR(hidden_file);
					goto out;
				} else {
					set_ftohf_index(file, bindex,
							hidden_file);
				}
			}
		} else {
			/* We only open the highest priority branch. */
			hidden_dentry = dtohd(dentry);
			if (willwrite && IS_WRITE_FLAG(file->f_flags)
			    && is_robranch(dentry)) {
				for (bindex = bstart - 1; bindex >= 0; bindex--) {

					err = copyup_file(dentry->
							  d_parent->
							  d_inode,
							  file,
							  bstart,
							  bindex,
							  file->
							  f_dentry->
							  d_inode->i_size);

					if (!err)
						break;
					else
						continue;

				}
				atomic_set(&ftopd(file)->ufi_generation,
					   atomic_read(&itopd(dentry->d_inode)->
						       uii_generation));
				goto out;
			}

			DGET(hidden_dentry);
			mntget(stohiddenmnt_index(sb, bstart));
			branchget(sb, bstart);
			hidden_file =
			    DENTRY_OPEN(hidden_dentry,
					stohiddenmnt_index(sb, bstart),
					file->f_flags);
			if (IS_ERR(hidden_file)) {
				err = PTR_ERR(hidden_file);
				goto out;
			}
			set_ftohf(file, hidden_file);
			/* Fix up the position. */
			hidden_file->f_pos = file->f_pos;

			memcpy(&(hidden_file->f_ra), &(file->f_ra),
			       sizeof(struct file_ra_state));
		}
		atomic_set(&ftopd(file)->ufi_generation,
			   atomic_read(&itopd(dentry->d_inode)->
				       uii_generation));
	}

	/* Copyup on the first write to a file on a readonly branch. */
	if (willwrite && IS_WRITE_FLAG(file->f_flags)
	    && !IS_WRITE_FLAG(ftohf(file)->f_flags) && is_robranch(dentry)) {
		dprint(PRINT_DEBUG,
		       "Doing delayed copyup of a read-write file on a read-only branch.\n");
		bstart = fbstart(file);
		bend = fbend(file);

		BUG_ON(!S_ISREG(file->f_dentry->d_inode->i_mode));

		for (bindex = bstart - 1; bindex >= 0; bindex--) {
			if (!d_deleted(file->f_dentry)) {
				err =
				    copyup_file(dentry->d_parent->
						d_inode, file, bstart,
						bindex,
						file->f_dentry->
						d_inode->i_size);
			} else {
				err =
				    copyup_deleted_file(file, dentry, bstart,
							bindex);
			}

			if (!err)
				break;
			else
				continue;

		}
		if (!err && (bstart > fbstart(file))) {
			bend = fbend(file);
			for (bindex = bstart; bindex <= bend; bindex++) {
				if (ftohf_index(file, bindex)) {
					branchput(dentry->d_sb, bindex);
					fput(ftohf_index(file, bindex));
					set_ftohf_index(file, bindex, NULL);
				}
			}
			fbend(file) = bend;
		}
	}

      out:
	print_dentry("file revalidate out", dentry);
	unlock_dentry(dentry);
	unionfs_read_unlock(dentry->d_sb);
	print_exit_status(err);
	return err;
}

int unionfs_open(struct inode *inode, struct file *file)
{
	int err = 0;
	int hidden_flags;
	struct file *hidden_file = NULL;
	struct dentry *hidden_dentry = NULL;
	struct dentry *dentry = NULL;
	int bindex = 0, bstart = 0, bend = 0;
	int locked = 0;
	int size;

	print_entry_location();

	ftopd_lhs(file) = KZALLOC(sizeof(struct unionfs_file_info), GFP_KERNEL);
	if (!ftopd(file)) {
		err = -ENOMEM;
		goto out;
	}
	fbstart(file) = -1;
	fbend(file) = -1;
	atomic_set(&ftopd(file)->ufi_generation,
		   atomic_read(&itopd(inode)->uii_generation));

	size = sizeof(struct file *) * sbmax(inode->i_sb);
	ftohf_ptr(file) = KZALLOC(size, GFP_KERNEL);
	if (!ftohf_ptr(file)) {
		err = -ENOMEM;
		goto out;
	}

	hidden_flags = file->f_flags;

	dentry = file->f_dentry;
	dprint(PRINT_DEBUG, "dentry to open is %p\n", dentry);
	lock_dentry(dentry);
	unionfs_read_lock(inode->i_sb);
	locked = 1;

	bstart = fbstart(file) = dbstart(dentry);
	bend = fbend(file) = dbend(dentry);

	/* increment to show the kind of open, so that we can
	 * flush appropriately
	 */
	atomic_inc(&itopd(dentry->d_inode)->uii_totalopens);

	/* open all directories and make the unionfs file struct point to these hidden file structs */
	if (S_ISDIR(inode->i_mode)) {
		for (bindex = bstart; bindex <= bend; bindex++) {
			hidden_dentry = dtohd_index(dentry, bindex);
			if (!hidden_dentry)
				continue;

			DGET(hidden_dentry);
			mntget(stohiddenmnt_index(inode->i_sb, bindex));
			hidden_file =
			    DENTRY_OPEN(hidden_dentry,
					stohiddenmnt_index(inode->i_sb, bindex),
					hidden_flags);
			if (IS_ERR(hidden_file)) {
				err = PTR_ERR(hidden_file);
				goto out;
			}

			set_ftohf_index(file, bindex, hidden_file);
			/* The branchget goes after the open, because otherwise
			 * we would miss the reference on release. */
			branchget(inode->i_sb, bindex);
		}
	} else {
		/* open a file */
		hidden_dentry = dtohd(dentry);

		/* check for the permission for hidden file.  If the error is COPYUP_ERR,
		 * copyup the file.
		 */
		if (hidden_dentry->d_inode && is_robranch(dentry)) {
			/* if the open will change the file, copy it up otherwise defer it. */
			if (hidden_flags & O_TRUNC) {
				int size = 0;

				err = -EROFS;
				/* copyup the file */
				for (bindex = bstart - 1; bindex >= 0; bindex--) {
					err =
					    copyup_file(dentry->
							d_parent->
							d_inode, file,
							bstart, bindex, size);
					if (!err) {
						break;
					}
				}
				goto out;
			} else {
				hidden_flags &= ~(OPEN_WRITE_FLAGS);
			}
		}

		DGET(hidden_dentry);
		/* dentry_open will decrement mnt refcnt if err.
		 * otherwise fput() will do an mntput() for us upon file close.
		 */
		mntget(stohiddenmnt_index(inode->i_sb, bstart));
		hidden_file = DENTRY_OPEN(hidden_dentry,
					  stohiddenmnt_index(inode->i_sb,
							     bstart),
					  hidden_flags);
		if (IS_ERR(hidden_file)) {
			err = PTR_ERR(hidden_file);
			goto out;
		} else {
			set_ftohf(file, hidden_file);
			branchget(inode->i_sb, bstart);
		}
	}

      out:
	/* freeing the allocated resources, and fput the opened files */
	if (err < 0 && ftopd(file)) {
		if (!locked)
			unionfs_read_lock(file->f_dentry->d_sb);
		for (bindex = bstart; bindex <= bend; bindex++) {
			hidden_file = ftohf_index(file, bindex);
			if (hidden_file) {
				branchput(file->f_dentry->d_sb, bindex);
				/* fput calls dput for hidden_dentry */
				fput(hidden_file);
			}
		}
		if (!locked)
			unionfs_read_unlock(file->f_dentry->d_sb);
		KFREE(ftohf_ptr(file));
		KFREE(ftopd(file));
	}

	print_file("OUT: unionfs_open", file);

	if (locked) {
		unlock_dentry(dentry);
		unionfs_read_unlock(inode->i_sb);
	}
	print_exit_status(err);
	return err;
}

int unionfs_file_release(struct inode *inode, struct file *file)
{
	int err = 0;
	struct file *hidden_file = NULL;
	int bindex, bstart, bend;
	int fgen;

	print_entry_location();

	checkinode(inode, "unionfs_release");

	/* fput all the hidden files */
	fgen = atomic_read(&ftopd(file)->ufi_generation);
	bstart = fbstart(file);
	bend = fbend(file);

	for (bindex = bstart; bindex <= bend; bindex++) {
		hidden_file = ftohf_index(file, bindex);

		if (hidden_file) {
			fput(hidden_file);
			unionfs_read_lock(inode->i_sb);
			branchput_gen(fgen, inode->i_sb, bindex);
			unionfs_read_unlock(inode->i_sb);
		}
	}
	KFREE(ftohf_ptr(file));

	if (ftopd(file)->rdstate) {
		ftopd(file)->rdstate->uds_access = jiffies;
		dprint(PRINT_DEBUG, "Saving rdstate with cookie %u [%d.%lld]\n",
		       ftopd(file)->rdstate->uds_cookie,
		       ftopd(file)->rdstate->uds_bindex,
		       (long long)ftopd(file)->rdstate->uds_dirpos);
		spin_lock(&itopd(inode)->uii_rdlock);
		itopd(inode)->uii_rdcount++;
		list_add_tail(&ftopd(file)->rdstate->uds_cache,
			      &itopd(inode)->uii_readdircache);
		mark_inode_dirty(inode);
		spin_unlock(&itopd(inode)->uii_rdlock);
		ftopd(file)->rdstate = NULL;
	}
	KFREE(ftopd(file));

	checkinode(inode, "post unionfs_release");

	print_exit_status(err);
	return err;
}

long unionfs_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long err = 0;		/* don't fail by default */
	struct file *hidden_file = NULL;
	int val;

	print_entry_location();

	if ((err = unionfs_file_revalidate(file, 1)))
		goto out;

	/* check if asked for local commands */
	switch (cmd) {
	case FIST_IOCTL_GET_DEBUG_VALUE:
		if (!capable(CAP_SYS_ADMIN)) {
			err = -EACCES;
			goto out;
		}
		val = get_debug_mask();
		err = put_user(val, (int __user *)arg);
		break;

	case FIST_IOCTL_SET_DEBUG_VALUE:
		if (!capable(CAP_SYS_ADMIN)) {
			err = -EACCES;
			goto out;
		}
		err = get_user(val, (int __user *)arg);
		if (err)
			break;
		dprint(PRINT_DEBUG, "IOCTL SET: got arg %d\n", val);
		if (val < 0 || val > PRINT_MAX) {
			err = -EINVAL;
			break;
		}
		set_debug_mask(val);
		break;

		/* add non-debugging fist ioctl's here */

	case UNIONFS_IOCTL_BRANCH_COUNT:
		if (!capable(CAP_SYS_ADMIN)) {
			err = -EACCES;
			goto out;
		}
		err = unionfs_ioctl_branchcount(file, cmd, arg);
		break;

	case UNIONFS_IOCTL_INCGEN:
		if (!capable(CAP_SYS_ADMIN)) {
			err = -EACCES;
			goto out;
		}
		err = unionfs_ioctl_incgen(file, cmd, arg);
		break;

	case UNIONFS_IOCTL_ADDBRANCH:
		if (!capable(CAP_SYS_ADMIN)) {
			err = -EACCES;
			goto out;
		}
		err =
		    unionfs_ioctl_addbranch(file->f_dentry->d_inode, cmd, arg);
		break;

	case UNIONFS_IOCTL_RDWRBRANCH:
		if (!capable(CAP_SYS_ADMIN)) {
			err = -EACCES;
			goto out;
		}
		err =
		    unionfs_ioctl_rdwrbranch(file->f_dentry->d_inode, cmd, arg);
		break;

	case UNIONFS_IOCTL_QUERYFILE:
		/* XXX: This should take the file. */
		err = unionfs_ioctl_queryfile(file, cmd, arg);
		break;

	default:
		hidden_file = ftohf(file);

		err = -ENOTTY;
		if (!hidden_file || !hidden_file->f_op)
			goto out;
		if (hidden_file->f_op->unlocked_ioctl) {
			err =
			    hidden_file->f_op->unlocked_ioctl(hidden_file, cmd,
							      arg);
		} else if (hidden_file->f_op->ioctl) {
			lock_kernel();
			err =
			    hidden_file->f_op->ioctl(hidden_file->f_dentry->
						     d_inode, hidden_file, cmd,
						     arg);
			unlock_kernel();
		}
	}			/* end of outer switch statement */

      out:
	print_exit_status((int)err);
	return err;
}

int unionfs_flush(struct file *file, fl_owner_t id)
{
	int err = 0;		/* assume ok (see open.c:close_fp) */
	struct file *hidden_file = NULL;
	int bindex, bstart, bend;

	print_entry_location();

	if ((err = unionfs_file_revalidate(file, 1)))
		goto out;
	if (!atomic_dec_and_test
	    (&itopd(file->f_dentry->d_inode)->uii_totalopens))
		goto out;

	lock_dentry(file->f_dentry);

	bstart = fbstart(file);
	bend = fbend(file);
	for (bindex = bstart; bindex <= bend; bindex++) {
		hidden_file = ftohf_index(file, bindex);

		if (hidden_file && hidden_file->f_op
		    && hidden_file->f_op->flush) {
			err = hidden_file->f_op->flush(hidden_file, id);
			if (err)
				goto out_lock;
			/* This was earlier done in the unlink_all function in unlink.c */
			/* if there are no more references to the dentry, dput it */
			if (d_deleted(file->f_dentry)) {
				DPUT(dtohd_index(file->f_dentry, bindex));
				set_dtohd_index(file->f_dentry, bindex, NULL);
			}
		}

	}

      out_lock:
	unlock_dentry(file->f_dentry);
      out:
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
