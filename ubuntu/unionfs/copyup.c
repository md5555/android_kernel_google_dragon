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
 * Copyright (c) 2003-2006 The Research Foundation of State University of New York*
 *
 * For specific licensing information, see the COPYING file distributed with
 * this package.
 *
 * This Copyright notice must be kept intact and distributed with all sources.
 */
/*
 *  $Id: copyup.c,v 1.78 2006/10/31 18:05:33 yiannos Exp $
 */

#include "unionfs.h"

/*Not Working Yet*/
static int copyup_xattrs(struct dentry *old_hidden_dentry,
			 struct dentry *new_hidden_dentry)
{
	int err = 0;
	ssize_t list_size = -1;
	char *name_list = NULL;
	char *attr_value = NULL;
	char *name_list_orig = NULL;

	print_entry_location();

	list_size = vfs_listxattr(old_hidden_dentry, NULL, NULL, 0, NULL);

	if (list_size <= 0) {
		err = list_size;
		goto out;
	}

	name_list = xattr_alloc(list_size + 1, XATTR_LIST_MAX);
	if (!name_list || IS_ERR(name_list)) {
		err = PTR_ERR(name_list);
		goto out;
	}
	list_size = vfs_listxattr(old_hidden_dentry, NULL, name_list,
				  list_size, NULL);
	attr_value = xattr_alloc(XATTR_SIZE_MAX, XATTR_SIZE_MAX);
	if (!attr_value || IS_ERR(attr_value)) {
		err = PTR_ERR(name_list);
		goto out;
	}
	name_list_orig = name_list;
	while (*name_list) {
		ssize_t size;

		//We need to lock here since vfs_getxattr doesn't lock for us.
		mutex_lock(&old_hidden_dentry->d_inode->i_mutex);
		size = vfs_getxattr(old_hidden_dentry, NULL, name_list,
				    attr_value, XATTR_SIZE_MAX, NULL);
		mutex_unlock(&old_hidden_dentry->d_inode->i_mutex);
		if (size < 0) {
			err = size;
			goto out;
		}

		if (size > XATTR_SIZE_MAX) {
			err = -E2BIG;
			goto out;
		}
		//We don't need to lock here since vfs_setxattr does it for us.
		err = vfs_setxattr(new_hidden_dentry, NULL, name_list,
				   attr_value, size, 0, NULL);

		if (err < 0)
			goto out;
		name_list += strlen(name_list) + 1;
	}
      out:
	name_list = name_list_orig;

	if (name_list)
		xattr_free(name_list, list_size + 1);
	if (attr_value)
		xattr_free(attr_value, XATTR_SIZE_MAX);
	/* It is no big deal if this fails, we just roll with the punches. */
	if (err == -ENOTSUPP || err == -EOPNOTSUPP)
		err = 0;
	return err;
}

/* Determine the mode based on the copyup flags, and the existing dentry. */
static int copyup_permissions(struct super_block *sb,
			      struct dentry *old_hidden_dentry,
			      struct dentry *new_hidden_dentry)
{
	struct iattr newattrs;
	int err;

	print_entry_location();

	newattrs.ia_atime = old_hidden_dentry->d_inode->i_atime;
	newattrs.ia_mtime = old_hidden_dentry->d_inode->i_mtime;
	newattrs.ia_ctime = old_hidden_dentry->d_inode->i_ctime;
	newattrs.ia_valid = ATTR_CTIME | ATTR_ATIME | ATTR_MTIME |
	    ATTR_ATIME_SET | ATTR_MTIME_SET;
	/* original mode of old file */
	newattrs.ia_mode = old_hidden_dentry->d_inode->i_mode;
	newattrs.ia_gid = old_hidden_dentry->d_inode->i_gid;
	newattrs.ia_uid = old_hidden_dentry->d_inode->i_uid;
	newattrs.ia_valid |= ATTR_FORCE | ATTR_GID | ATTR_UID | ATTR_MODE;
	if (newattrs.ia_valid & ATTR_MODE) {
		newattrs.ia_mode =
		    (newattrs.ia_mode & S_IALLUGO) | (old_hidden_dentry->
						      d_inode->
						      i_mode & ~S_IALLUGO);
	}

	err = notify_change(new_hidden_dentry, NULL, &newattrs);

	print_exit_status(err);
	return err;
}

int copyup_dentry(struct inode *dir, struct dentry *dentry,
		  int bstart, int new_bindex,
		  struct file **copyup_file, loff_t len)
{
	return copyup_named_dentry(dir, dentry, bstart, new_bindex,
				   dentry->d_name.name,
				   dentry->d_name.len, copyup_file, len);
}

int copyup_named_dentry(struct inode *dir, struct dentry *dentry,
			int bstart, int new_bindex, const char *name,
			int namelen, struct file **copyup_file, loff_t len)
{
	struct dentry *new_hidden_dentry;
	struct dentry *old_hidden_dentry = NULL;
	struct super_block *sb;
	struct file *input_file = NULL;
	struct file *output_file = NULL;
	struct sioq_args args;
	ssize_t read_bytes, write_bytes;
	mm_segment_t old_fs;
	int err = 0;
	char *buf;
	int old_bindex;
	int got_branch_input = -1;
	int got_branch_output = -1;
	int old_bstart;
	int old_bend;
	int old_mode;
	loff_t size = len;
	struct dentry *new_hidden_parent_dentry = NULL;
	mm_segment_t oldfs;
	char *symbuf = NULL;

	print_entry_location();
	verify_locked(dentry);
	print_dentry("IN: copyup_named_dentry", dentry);

	old_bindex = bstart;
	old_bstart = dbstart(dentry);
	old_bend = dbend(dentry);

	BUG_ON(new_bindex < 0);
	BUG_ON(new_bindex >= old_bindex);

	sb = dir->i_sb;

	unionfs_read_lock(sb);

	if ((err = is_robranch_super(sb, new_bindex)))
		goto out;

	/* Create the directory structure above this dentry. */
	new_hidden_dentry = create_parents_named(dir, dentry, name, new_bindex);
	if (IS_ERR(new_hidden_dentry)) {
		err = PTR_ERR(new_hidden_dentry);
		goto out;
	}

	print_dentry("Copyup Object", new_hidden_dentry);

	/* Now we actually create the object. */
	old_hidden_dentry = dtohd_index(dentry, old_bindex);
	DGET(old_hidden_dentry);
	
	old_mode = old_hidden_dentry->d_inode->i_mode;

	/* For symlinks, we must read the link before we lock the directory. */
	if (S_ISLNK(old_mode)) {

		symbuf = KMALLOC(PATH_MAX, GFP_KERNEL);
		if (!symbuf) {
			err = -ENOMEM;
			goto copyup_readlink_err;
		}

		oldfs = get_fs();
		set_fs(KERNEL_DS);
		err =
		    old_hidden_dentry->d_inode->i_op->
		    readlink(old_hidden_dentry, (char __user *)symbuf,
			     PATH_MAX);
		set_fs(oldfs);
		if (err < 0)
			goto copyup_readlink_err;
		symbuf[err] = '\0';
	}

	/* Now we lock the parent, and create the object in the new branch. */
	new_hidden_parent_dentry = lock_parent(new_hidden_dentry);
	if (S_ISDIR(old_mode)) {
		args.mkdir.parent = new_hidden_parent_dentry->d_inode;
		args.mkdir.dentry = new_hidden_dentry;
		args.mkdir.mode = old_mode; /*S_IRWXU*/
		run_sioq(__unionfs_mkdir, &args);
		err = args.err;
	} else if (S_ISLNK(old_mode)) {
		args.symlink.parent = new_hidden_parent_dentry->d_inode;
		args.symlink.dentry = new_hidden_dentry;
		args.symlink.symbuf = symbuf;
		args.symlink.mode = old_mode;
		run_sioq(__unionfs_symlink, &args);
		err = args.err;
	} else if (S_ISBLK(old_mode)
		   || S_ISCHR(old_mode)
		   || S_ISFIFO(old_mode)
		   || S_ISSOCK(old_mode)) {
		args.mknod.parent = new_hidden_parent_dentry->d_inode;
		args.mknod.dentry = new_hidden_dentry;
		args.mknod.mode = old_mode;
		args.mknod.dev = old_hidden_dentry->d_inode->i_rdev;
		run_sioq(__unionfs_mknod, &args);
		err = args.err;	
	} else if (S_ISREG(old_mode)) {
		args.create.parent = new_hidden_parent_dentry->d_inode;
		args.create.dentry = new_hidden_dentry;
		args.create.mode = old_mode;
		args.create.nd = NULL;
		run_sioq(__unionfs_create, &args);
		err = args.err;
	} else {
		printk(KERN_ERR "Unknown inode type %d\n",
		       old_hidden_dentry->d_inode->i_mode);
		BUG();
	}

      copyup_readlink_err:
	KFREE(symbuf);
	if (err) {
		/* get rid of the hidden dentry and all its traces */
		DPUT(new_hidden_dentry);
		set_dtohd_index(dentry, new_bindex, NULL);
		set_dbstart(dentry, old_bstart);
		set_dbend(dentry, old_bend);
		goto out_dir;
	}
#ifdef UNIONFS_IMAP
	if (stopd(sb)->usi_persistent) {
		err = write_uin(dentry->d_sb, dentry->d_inode->i_ino,
				new_bindex, new_hidden_dentry->d_inode->i_ino);
		if (err)
			goto out_dir;
	}
#endif
	/* We actually copyup the file here. */
	if (S_ISREG(old_hidden_dentry->d_inode->i_mode)) {
		mntget(stohiddenmnt_index(sb, old_bindex));
		branchget(sb, old_bindex);
		got_branch_input = old_bindex;
		input_file =
		    DENTRY_OPEN(old_hidden_dentry,
				stohiddenmnt_index(sb, old_bindex),
				O_RDONLY | O_LARGEFILE);
		if (IS_ERR(input_file)) {
			err = PTR_ERR(input_file);
			goto out_dir;
		}
		if (!input_file->f_op || !input_file->f_op->read) {
			err = -EINVAL;
			goto out_dir;
		}

		/* copy the new file */
		DGET(new_hidden_dentry);
		mntget(stohiddenmnt_index(sb, new_bindex));
		branchget(sb, new_bindex);
		got_branch_output = new_bindex;
		output_file =
		    DENTRY_OPEN(new_hidden_dentry,
				stohiddenmnt_index(sb, new_bindex),
				O_WRONLY | O_LARGEFILE);
		if (IS_ERR(output_file)) {
			err = PTR_ERR(output_file);
			goto out_dir;
		}
		if (!output_file->f_op || !output_file->f_op->write) {
			err = -EINVAL;
			goto out_dir;
		}

		/* allocating a buffer */
		buf = (char *)KMALLOC(PAGE_SIZE, GFP_KERNEL);
		if (!buf) {
			err = -ENOMEM;
			goto out_dir;
		}

		/* now read PAGE_SIZE bytes from offset 0 in a loop */
		old_fs = get_fs();

		input_file->f_pos = 0;
		output_file->f_pos = 0;

		err = 0;	// reset error just in case
		set_fs(KERNEL_DS);
		do {
			if (len >= PAGE_SIZE)
				size = PAGE_SIZE;
			else if ((len < PAGE_SIZE) && (len > 0))
				size = len;

			len -= PAGE_SIZE;

			read_bytes =
			    input_file->f_op->read(input_file,
						   (char __user *)buf, size,
						   &input_file->f_pos);
			if (read_bytes <= 0) {
				err = read_bytes;
				break;
			}

			write_bytes =
			    output_file->f_op->write(output_file,
						     (char __user *)buf,
						     read_bytes,
						     &output_file->f_pos);
			if (write_bytes < 0 || (write_bytes < read_bytes)) {
				err = write_bytes;
				break;
			}
		} while ((read_bytes > 0) && (len > 0));
		set_fs(old_fs);
		KFREE(buf);
#ifdef UNIONFS_MMAP
		/* SP: Now that we copied up the file, have to sync its data
		 * as otherwise when we do a read_cache_page(), we'll possibly
		 * read crap.
		 *
		 * another posisble solution would be in the address op code
		 * would be to check the "lower" page to see if its dirty,
		 * and if it's dirty, use it directl
		 */
		if (!err) {
			err =
			    output_file->f_op->fsync(output_file,
						     new_hidden_dentry, 0);
		}
#endif
		if (err) {
			/* copyup failed, because we ran out of space or quota,
			 * or something else happened so let's unlink; we don't
			 * really care about the return value of vfs_unlink */
			vfs_unlink(new_hidden_parent_dentry->d_inode,
				   new_hidden_dentry, NULL);

			goto out_dir;
		}
	}

	/* Set permissions. */
	if ((err =
	     copyup_permissions(sb, old_hidden_dentry, new_hidden_dentry)))
		goto out_dir;
	/* Selinux uses extended attributes for permissions. */
	if ((err = copyup_xattrs(old_hidden_dentry, new_hidden_dentry)))
		goto out_dir;

	/* do not allow files getting deleted to be reinterposed */
	if (!d_deleted(dentry))
		unionfs_reinterpose(dentry);

      out_dir:
	if (new_hidden_parent_dentry)
		unlock_dir(new_hidden_parent_dentry);

      out:
	if (input_file && !IS_ERR(input_file)) {
		fput(input_file);
	} else {
		/* since input file was not opened, we need to explicitly
		 * dput the old_hidden_dentry
		 */
		DPUT(old_hidden_dentry);
	}

	/* in any case, we have to branchput */
	if (got_branch_input >= 0)
		branchput(sb, got_branch_input);

	if (output_file) {
		if (copyup_file && !err) {
			*copyup_file = output_file;
		} else {
			/* close the file if there was no error, or if we ran
			 * out of space in which case we unlinked the file */
			if (!IS_ERR(output_file))
				fput(output_file);
			branchput(sb, got_branch_output);
		}
	}

	unionfs_read_unlock(sb);

	print_dentry("OUT: copyup_dentry", dentry);
	print_inode("OUT: copyup_dentry", dentry->d_inode);

	print_exit_status(err);
	return err;
}

/* This function creates a copy of a file represented by 'file' which currently
 * resides in branch 'bstart' to branch 'new_bindex.  The copy will be named
 * "name".  */
int copyup_named_file(struct inode *dir, struct file *file, char *name,
		      int bstart, int new_bindex, loff_t len)
{
	int err = 0;
	struct file *output_file = NULL;

	print_entry_location();

	err = copyup_named_dentry(dir, file->f_dentry, bstart,
				  new_bindex, name, strlen(name), &output_file,
				  len);
	if (!err) {
		fbstart(file) = new_bindex;
		set_ftohf_index(file, new_bindex, output_file);
	}

	print_exit_status(err);
	return err;
}

/* This function creates a copy of a file represented by 'file' which currently
 * resides in branch 'bstart' to branch 'new_bindex.
 */
int copyup_file(struct inode *dir, struct file *file, int bstart,
		int new_bindex, loff_t len)
{
	int err = 0;
	struct file *output_file = NULL;

	print_entry_location();

	err = copyup_dentry(dir, file->f_dentry, bstart, new_bindex,
			    &output_file, len);
	if (!err) {
		fbstart(file) = new_bindex;
		set_ftohf_index(file, new_bindex, output_file);
	}

	print_exit_status(err);
	return err;
}

/* This function replicates the directory structure upto given dentry
 * in the bindex branch. Can create directory structure recursively to the right
 * also.
 */
struct dentry *create_parents(struct inode *dir, struct dentry *dentry,
			      int bindex)
{
	struct dentry *hidden_dentry;

	print_entry_location();
	hidden_dentry =
	    create_parents_named(dir, dentry, dentry->d_name.name, bindex);
	print_exit_location();

	return (hidden_dentry);
}

/* This function replicates the directory structure upto given dentry
 * in the bindex branch.  */
struct dentry *create_parents_named(struct inode *dir, struct dentry *dentry,
				    const char *name, int bindex)
{
	int err;
	struct dentry *child_dentry;
	struct dentry *parent_dentry;
	struct dentry *hidden_parent_dentry = NULL;
	struct dentry *hidden_dentry = NULL;
	struct sioq_args args;
	const char *childname;
	unsigned int childnamelen;

	int old_kmalloc_size;
	int kmalloc_size;
	int num_dentry;
	int count;

	int old_bstart;
	int old_bend;
	struct dentry **path = NULL;
	struct dentry **tmp_path;
	struct super_block *sb;
#ifdef UNIONFS_IMAP
	int persistent;
#endif
	print_entry_location();

	verify_locked(dentry);

	/* There is no sense allocating any less than the minimum. */
	kmalloc_size = 128;
	num_dentry = kmalloc_size / sizeof(struct dentry *);

	if ((err = is_robranch_super(dir->i_sb, bindex))) {
		hidden_dentry = ERR_PTR(err);
		goto out;
	}

	print_dentry("IN: create_parents_named", dentry);
	dprint(PRINT_DEBUG, "name = %s\n", name);

	old_bstart = dbstart(dentry);
	old_bend = dbend(dentry);

	hidden_dentry = ERR_PTR(-ENOMEM);
	path = (struct dentry **)KZALLOC(kmalloc_size, GFP_KERNEL);
	if (!path)
		goto out;

	/* assume the negative dentry of unionfs as the parent dentry */
	parent_dentry = dentry;

	count = 0;
	/* This loop finds the first parent that exists in the given branch.
	 * We start building the directory structure from there.  At the end
	 * of the loop, the following should hold:
	 *      child_dentry is the first nonexistent child
	 *      parent_dentry is the first existent parent
	 *      path[0] is the = deepest child
	 *      path[count] is the first child to create
	 */
	do {
		child_dentry = parent_dentry;

		/* find the parent directory dentry in unionfs */
		parent_dentry = child_dentry->d_parent;
		lock_dentry(parent_dentry);

		/* find out the hidden_parent_dentry in the given branch */
		hidden_parent_dentry = dtohd_index(parent_dentry, bindex);

		/* store the child dentry */
		path[count++] = child_dentry;
		if (count == num_dentry) {
			old_kmalloc_size = kmalloc_size;
			kmalloc_size *= 2;
			num_dentry = kmalloc_size / sizeof(struct dentry *);

			tmp_path =
			    (struct dentry **)KZALLOC(kmalloc_size, GFP_KERNEL);
			if (!tmp_path) {
				hidden_dentry = ERR_PTR(-ENOMEM);
				goto out;
			}
			memcpy(tmp_path, path, old_kmalloc_size);
			KFREE(path);
			path = tmp_path;
			tmp_path = NULL;
		}

	} while (!hidden_parent_dentry);
	count--;

	sb = dentry->d_sb;
#ifdef UNIONFS_IMAP
	persistent = stopd(sb)->usi_persistent;
#endif
	/* This is basically while(child_dentry != dentry).  This loop is
	 * horrible to follow and should be replaced with cleaner code. */
	while (1) {
		// get hidden parent dir in the current branch
		hidden_parent_dentry = dtohd_index(parent_dentry, bindex);
		unlock_dentry(parent_dentry);

		// init the values to lookup
		childname = child_dentry->d_name.name;
		childnamelen = child_dentry->d_name.len;

		if (child_dentry != dentry) {
			// lookup child in the underlying file system
			hidden_dentry =
			    LOOKUP_ONE_LEN(childname, hidden_parent_dentry,
					   childnamelen);
			if (IS_ERR(hidden_dentry))
				goto out;
		} else {
			int loop_start;
			int loop_end;
			int new_bstart = -1;
			int new_bend = -1;
			int i;

			/* is the name a whiteout of the childname ? */
			//lookup the whiteout child in the underlying file system
			hidden_dentry =
			    LOOKUP_ONE_LEN(name, hidden_parent_dentry,
					   strlen(name));
			if (IS_ERR(hidden_dentry))
				goto out;

			/* Replace the current dentry (if any) with the new one. */
			DPUT(dtohd_index(dentry, bindex));
			set_dtohd_index(dentry, bindex, hidden_dentry);

			loop_start =
			    (old_bstart < bindex) ? old_bstart : bindex;
			loop_end = (old_bend > bindex) ? old_bend : bindex;

			/* This loop sets the bstart and bend for the new
			 * dentry by traversing from left to right.
			 * It also dputs all negative dentries except
			 * bindex (the newly looked dentry
			 */
			for (i = loop_start; i <= loop_end; i++) {
				if (!dtohd_index(dentry, i))
					continue;

				if (i == bindex) {
					new_bend = i;
					if (new_bstart < 0)
						new_bstart = i;
					continue;
				}

				if (!dtohd_index(dentry, i)->d_inode) {
					DPUT(dtohd_index(dentry, i));
					set_dtohd_index(dentry, i, NULL);
				} else {
					if (new_bstart < 0)
						new_bstart = i;
					new_bend = i;
				}
			}

			if (new_bstart < 0)
				new_bstart = bindex;
			if (new_bend < 0)
				new_bend = bindex;
			set_dbstart(dentry, new_bstart);
			set_dbend(dentry, new_bend);
			break;
		}

		if (hidden_dentry->d_inode) {
			/* since this already exists we dput to avoid
			 * multiple references on the same dentry */
			DPUT(hidden_dentry);
		} else {

			/* its a negative dentry, create a new dir */
			hidden_parent_dentry = lock_parent(hidden_dentry);
			args.mkdir.parent = hidden_parent_dentry->d_inode;
			args.mkdir.dentry = hidden_dentry;
			args.mkdir.mode = child_dentry->d_inode->i_mode;
			run_sioq(__unionfs_mkdir, &args);
			err = args.err;
			if (!err)
				err = copyup_permissions
				    (dir->i_sb, child_dentry, hidden_dentry);
			unlock_dir(hidden_parent_dentry);
			if (err) {
				DPUT(hidden_dentry);
				hidden_dentry = ERR_PTR(err);
				goto out;
			}
#ifdef UNIONFS_IMAP
			if (persistent) {
				err = write_uin
				    (sb, child_dentry->d_inode->i_ino,
				     bindex, hidden_dentry->d_inode->i_ino);
				if (err) {
					DPUT(hidden_dentry);
					hidden_dentry = ERR_PTR(err);
					goto out;
				}
			}
#endif
			set_itohi_index(child_dentry->d_inode, bindex,
					IGRAB(hidden_dentry->d_inode));
			if (ibstart(child_dentry->d_inode) > bindex)
				ibstart(child_dentry->d_inode) = bindex;
			if (ibend(child_dentry->d_inode) < bindex)
				ibend(child_dentry->d_inode) = bindex;

			set_dtohd_index(child_dentry, bindex, hidden_dentry);
			if (dbstart(child_dentry) > bindex)
				set_dbstart(child_dentry, bindex);
			if (dbend(child_dentry) < bindex)
				set_dbend(child_dentry, bindex);
		}

		parent_dentry = child_dentry;
		child_dentry = path[--count];
	}
      out:
	KFREE(path);
	print_dentry("OUT: create_parents_named", dentry);
	print_exit_pointer(hidden_dentry);
	return hidden_dentry;
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
