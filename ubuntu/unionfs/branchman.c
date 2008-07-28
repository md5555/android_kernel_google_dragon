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
 *  $Id: branchman.c,v 1.66 2006/10/31 00:05:22 yiannos Exp $
 */

#include "unionfs.h"

struct dentry **alloc_new_dentries(int objs)
{
	if (!objs)
		return NULL;

	return KZALLOC(sizeof(struct dentry *) * objs, GFP_KERNEL);
}

struct unionfs_usi_data *alloc_new_data(int objs)
{
	if (!objs)
		return NULL;

	return KZALLOC(sizeof(struct unionfs_usi_data) * objs, GFP_KERNEL);
}

static void fixputmaps(struct super_block *sb)
{
	struct unionfs_sb_info *spd;
	struct putmap *cur;
	int gen;
	int i;

	print_entry_location();

	spd = stopd(sb);
	cur = spd->usi_putmaps[spd->usi_lastputmap - spd->usi_firstputmap];

	for (gen = 0; gen < spd->usi_lastputmap - spd->usi_firstputmap; gen++) {
		if (!spd->usi_putmaps[gen])
			continue;
		for (i = 0; i <= spd->usi_putmaps[gen]->bend; i++)
			spd->usi_putmaps[gen]->map[i] =
			    cur->map[spd->usi_putmaps[gen]->map[i]];
	}

	print_exit_location();
}

static int newputmap(struct super_block *sb)
{
	struct unionfs_sb_info *spd;
	struct putmap *newmap;
	int count = 0;
	int i;

	print_entry_location();

	spd = stopd(sb);

	i = sizeof(int) * (sbend(sb) + 1);
	newmap = KMALLOC(sizeof(struct putmap) + i, GFP_KERNEL);
	if (!newmap) {
		print_exit_status(-ENOMEM);
		return -ENOMEM;
	}

	if (!spd->usi_firstputmap) {
		spd->usi_firstputmap = 1;
		spd->usi_lastputmap = 1;

		spd->usi_putmaps = KMALLOC(sizeof(struct putmap *), GFP_KERNEL);
		if (!spd->usi_putmaps) {
			KFREE(newmap);
			print_exit_status(-ENOMEM);
			return -ENOMEM;
		}
	} else {
		struct putmap **newlist;
		int newfirst = spd->usi_firstputmap;

		while (!spd->usi_putmaps[newfirst - spd->usi_firstputmap] &&
		       newfirst <= spd->usi_lastputmap) {
			newfirst++;
		}

		newlist =
		    KMALLOC(sizeof(struct putmap *) *
			    (1 + spd->usi_lastputmap - newfirst), GFP_KERNEL);
		if (!newlist) {
			KFREE(newmap);
			print_exit_status(-ENOMEM);
			return -ENOMEM;
		}

		for (i = newfirst; i <= spd->usi_lastputmap; i++) {
			newlist[i - newfirst] =
			    spd->usi_putmaps[i - spd->usi_firstputmap];
		}

		KFREE(spd->usi_putmaps);
		spd->usi_putmaps = newlist;
		spd->usi_firstputmap = newfirst;
		spd->usi_lastputmap++;
	}

	newmap->bend = sbend(sb);
	for (i = 0; i <= sbend(sb); i++) {
		count += branch_count(sb, i);
		newmap->map[i] = i;
	}
	for (i = spd->usi_firstputmap; i < spd->usi_lastputmap; i++) {
		struct putmap *cur;
		cur = spd->usi_putmaps[i - spd->usi_firstputmap];
		if (!cur)
			continue;
		count -= atomic_read(&cur->count);
	}
	atomic_set(&newmap->count, count);
	spd->usi_putmaps[spd->usi_lastputmap - spd->usi_firstputmap] = newmap;

	print_exit_status(0);
	return 0;
}

/* XXX: this function needs to go. There is no reason for this to be here */
int unionfs_ioctl_branchcount(struct file *file, unsigned int cmd,
			      unsigned long arg)
{
	int err = 0;
	int bstart, bend;
	int i;
	struct super_block *sb = file->f_dentry->d_sb;

	print_entry_location();

	bstart = sbstart(sb);
	bend = sbend(sb);

	err = bend + 1;
	if (!arg)
		goto out;

	for (i = bstart; i <= bend; i++) {
		if (put_user(branch_count(sb, i), ((int __user *)arg) + i)) {
			err = -EFAULT;
			goto out;
		}
	}

      out:
	print_exit_status(err);
	return err;
}

int unionfs_ioctl_incgen(struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	struct super_block *sb;

	print_entry_location();

	sb = file->f_dentry->d_sb;

	unionfs_write_lock(sb);
	if ((err = newputmap(sb)))
		goto out;

	atomic_inc(&stopd(sb)->usi_generation);
	err = atomic_read(&stopd(sb)->usi_generation);

	atomic_set(&dtopd(sb->s_root)->udi_generation, err);
	atomic_set(&itopd(sb->s_root->d_inode)->uii_generation, err);

      out:
	unionfs_write_unlock(sb);
	print_exit_status(err);
	return err;
}

int unionfs_ioctl_addbranch(struct inode *inode, unsigned int cmd,
			    unsigned long arg)
{
	int err;
	struct unionfs_addbranch_args *addargs = NULL;
	struct nameidata nd;
	char *path = NULL;
	int gen;
	int i;

	int pobjects;

	struct unionfs_usi_data *new_data = NULL;
	struct dentry **new_udi_dentry = NULL;
	struct inode **new_uii_inode = NULL;

	struct dentry *root = NULL;
	struct dentry *hidden_root = NULL;

	print_entry_location();

#ifdef UNIONFS_IMAP
	if (stopd(sb)->usi_persistent) {
		printk(KERN_ERR "Cannot manipulate branches if imap is used\n");
		err = -EPERM;
		goto out;
	}
#endif

	err = -ENOMEM;
	addargs = KMALLOC(sizeof(struct unionfs_addbranch_args), GFP_KERNEL);
	if (!addargs)
		goto out;

	err = -EFAULT;
	if (copy_from_user
	    (addargs, (const void __user *)arg,
	     sizeof(struct unionfs_addbranch_args)))
		goto out;

	err = -EINVAL;
	if (addargs->ab_perms & ~(MAY_READ | MAY_WRITE | MAY_NFSRO))
		goto out;
	if (!(addargs->ab_perms & MAY_READ))
		goto out;

	err = -E2BIG;
	if (sbend(inode->i_sb) > FD_SETSIZE)
		goto out;

	err = -ENOMEM;
	if (!(path = getname((const char __user *)addargs->ab_path)))
		goto out;

	err = path_lookup(path, LOOKUP_FOLLOW, &nd);

	RECORD_PATH_LOOKUP(&nd);
	if (err)
		goto out;
	if ((err = check_branch(&nd))) {
		path_put(&nd.path);
		RECORD_PATH_RELEASE(&nd);
		goto out;
	}

	unionfs_write_lock(inode->i_sb);
	lock_dentry(inode->i_sb->s_root);

	root = inode->i_sb->s_root;
	for (i = dbstart(inode->i_sb->s_root); i <= dbend(inode->i_sb->s_root);
	     i++) {
		hidden_root = dtohd_index(root, i);
		if (is_branch_overlap(hidden_root, nd.path.dentry)) {
			err = -EINVAL;
			goto out;
		}
	}

	err = -EINVAL;
	if (addargs->ab_branch < 0
	    || (addargs->ab_branch > (sbend(inode->i_sb) + 1)))
		goto out;

	if ((err = newputmap(inode->i_sb)))
		goto out;

	stopd(inode->i_sb)->b_end++;
	dtopd(inode->i_sb->s_root)->udi_bcount++;
	set_dbend(inode->i_sb->s_root, dbend(inode->i_sb->s_root) + 1);
	itopd(inode->i_sb->s_root->d_inode)->b_end++;

	atomic_inc(&stopd(inode->i_sb)->usi_generation);
	gen = atomic_read(&stopd(inode->i_sb)->usi_generation);

	pobjects = sbend(inode->i_sb) + 1;

	/* Reallocate the dynamic structures. */
	new_data = alloc_new_data(pobjects);
	new_udi_dentry = alloc_new_dentries(pobjects);
	new_uii_inode = KZALLOC(sizeof(struct inode *) * pobjects, GFP_KERNEL);

	if (!new_udi_dentry || !new_uii_inode || !new_data) {
		err = -ENOMEM;
		goto out;
	}

	/* Copy the in-place values to our new structure. */
	for (i = 0; i < addargs->ab_branch; i++) {
		atomic_set(&(new_data[i].sbcount),
			   branch_count(inode->i_sb, i));

		new_data[i].branchperms = branchperms(inode->i_sb, i);
		new_data[i].hidden_mnt = stohiddenmnt_index(inode->i_sb, i);
		new_data[i].sb = stohs_index(inode->i_sb, i);

		new_udi_dentry[i] = dtohd_index(inode->i_sb->s_root, i);
		new_uii_inode[i] = itohi_index(inode->i_sb->s_root->d_inode, i);
	}

	/* Shift the ends to the right (only handle reallocated bits). */
	for (i = sbend(inode->i_sb) - 1; i >= (int)addargs->ab_branch; i--) {
		int j = i + 1;
		int pmindex;

		atomic_set(&new_data[j].sbcount, branch_count(inode->i_sb, i));

		new_data[j].branchperms = branchperms(inode->i_sb, i);
		new_data[j].hidden_mnt = stohiddenmnt_index(inode->i_sb, i);
		new_data[j].sb = stohs_index(inode->i_sb, i);
		new_udi_dentry[j] = dtohd_index(inode->i_sb->s_root, i);
		new_uii_inode[j] = itohi_index(inode->i_sb->s_root->d_inode, i);

		/* Update the newest putmap, so it is correct for later. */
		pmindex = stopd(inode->i_sb)->usi_lastputmap;
		pmindex -= stopd(inode->i_sb)->usi_firstputmap;
		stopd(inode->i_sb)->usi_putmaps[pmindex]->map[i] = j;

	}

	/* Now we can free the old ones. */
	KFREE(dtopd(inode->i_sb->s_root)->udi_dentry);
	KFREE(itopd(inode->i_sb->s_root->d_inode)->uii_inode);
	KFREE(stopd(inode->i_sb)->usi_data);

	/* Update the real pointers. */
	dtohd_ptr(inode->i_sb->s_root) = new_udi_dentry;
	itohi_ptr(inode->i_sb->s_root->d_inode) = new_uii_inode;
	stopd(inode->i_sb)->usi_data = new_data;

	/* Re-NULL the new ones so we don't try to free them. */
	new_data = NULL;
	new_udi_dentry = NULL;
	new_uii_inode = NULL;

	/* Put the new dentry information into it's slot. */
	set_dtohd_index(inode->i_sb->s_root, addargs->ab_branch, nd.path.dentry);
	set_itohi_index(inode->i_sb->s_root->d_inode, addargs->ab_branch,
			IGRAB(nd.path.dentry->d_inode));
	set_branchperms(inode->i_sb, addargs->ab_branch, addargs->ab_perms);
	set_branch_count(inode->i_sb, addargs->ab_branch, 0);
	set_stohiddenmnt_index(inode->i_sb, addargs->ab_branch, nd.path.mnt);
	set_stohs_index(inode->i_sb, addargs->ab_branch, nd.path.dentry->d_sb);

	atomic_set(&dtopd(inode->i_sb->s_root)->udi_generation, gen);
	atomic_set(&itopd(inode->i_sb->s_root->d_inode)->uii_generation, gen);

	fixputmaps(inode->i_sb);

      out:
	unlock_dentry(inode->i_sb->s_root);
	unionfs_write_unlock(inode->i_sb);

	KFREE(new_udi_dentry);
	KFREE(new_uii_inode);
	KFREE(new_data);
	KFREE(addargs);
	if (path)
		putname(path);

	print_exit_status(err);

	return err;
}

/* This must be called with the super block already locked. */
int unionfs_ioctl_delbranch(struct super_block *sb, unsigned long arg)
{
	struct dentry *hidden_dentry;
	struct inode *hidden_inode;
	struct vfsmount *hidden_mnt;
	struct dentry *root_dentry;
	struct inode *root_inode;
	int err = 0;
	int pmindex, i, gen;

	print_entry("branch = %lu ", arg);
	lock_dentry(sb->s_root);

#ifdef UNIONFS_IMAP
	if (stopd(sb)->usi_persistent) {
		printk(KERN_ERR "Cannot manipulate branches if imap is used\n");
		err = -EPERM;
		goto out;
	}
#endif
	err = -EBUSY;
	if (sbmax(sb) == 1)
		goto out;
	err = -EINVAL;
	if (arg < 0 || arg > stopd(sb)->b_end)
		goto out;
	err = -EBUSY;
	if (branch_count(sb, arg))
		goto out;
	if ((err = newputmap(sb)))
		goto out;

	pmindex = stopd(sb)->usi_lastputmap;
	pmindex -= stopd(sb)->usi_firstputmap;

	atomic_inc(&stopd(sb)->usi_generation);
	gen = atomic_read(&stopd(sb)->usi_generation);

	root_dentry = sb->s_root;
	root_inode = sb->s_root->d_inode;

	hidden_dentry = dtohd_index(root_dentry, arg);
	hidden_mnt = stohiddenmnt_index(sb, arg);
	hidden_inode = itohi_index(root_inode, arg);

	DPUT(hidden_dentry);
	IPUT(hidden_inode);
	mntput(hidden_mnt);

	for (i = arg; i <= (sbend(sb) - 1); i++) {
		set_branch_count(sb, i, branch_count(sb, i + 1));
		set_stohiddenmnt_index(sb, i, stohiddenmnt_index(sb, i + 1));
		set_stohs_index(sb, i, stohs_index(sb, i + 1));
		set_branchperms(sb, i, branchperms(sb, i + 1));
		set_dtohd_index(root_dentry, i,
				dtohd_index(root_dentry, i + 1));
		set_itohi_index(root_inode, i, itohi_index(root_inode, i + 1));
		stopd(sb)->usi_putmaps[pmindex]->map[i + 1] = i;
	}

	set_dtohd_index(root_dentry, sbend(sb), NULL);
	set_itohi_index(root_inode, sbend(sb), NULL);
	set_stohiddenmnt_index(sb, sbend(sb), NULL);
	set_stohs_index(sb, sbend(sb), NULL);

	//XXX: Place check for inode maps and removal of branch here

	stopd(sb)->b_end--;
	set_dbend(root_dentry, dbend(root_dentry) - 1);
	dtopd(root_dentry)->udi_bcount--;
	itopd(root_inode)->b_end--;

	atomic_set(&dtopd(root_dentry)->udi_generation, gen);
	atomic_set(&itopd(root_inode)->uii_generation, gen);

	fixputmaps(sb);

	/* This doesn't open a file, so we might have to free the map here. */
	if (atomic_read(&stopd(sb)->usi_putmaps[pmindex]->count) == 0) {
		KFREE(stopd(sb)->usi_putmaps[pmindex]);
		stopd(sb)->usi_putmaps[pmindex] = NULL;
	}

      out:
	unlock_dentry(sb->s_root);
	print_exit_status(err);

	return err;
}

int unionfs_ioctl_rdwrbranch(struct inode *inode, unsigned int cmd,
			     unsigned long arg)
{
	int err;
	struct unionfs_rdwrbranch_args *rdwrargs = NULL;
	int gen;

	print_entry_location();

	unionfs_write_lock(inode->i_sb);
	lock_dentry(inode->i_sb->s_root);

	if ((err = newputmap(inode->i_sb)))
		goto out;

	err = -ENOMEM;
	rdwrargs = KMALLOC(sizeof(struct unionfs_rdwrbranch_args), GFP_KERNEL);
	if (!rdwrargs)
		goto out;

	err = -EFAULT;
	if (copy_from_user
	    (rdwrargs, (const void __user *)arg,
	     sizeof(struct unionfs_rdwrbranch_args)))
		goto out;

	err = -EINVAL;
	if (rdwrargs->rwb_branch < 0
	    || (rdwrargs->rwb_branch > (sbend(inode->i_sb) + 1)))
		goto out;
	if (rdwrargs->rwb_perms & ~(MAY_READ | MAY_WRITE | MAY_NFSRO))
		goto out;
	if (!(rdwrargs->rwb_perms & MAY_READ))
		goto out;

	set_branchperms(inode->i_sb, rdwrargs->rwb_branch, rdwrargs->rwb_perms);

	atomic_inc(&stopd(inode->i_sb)->usi_generation);
	gen = atomic_read(&stopd(inode->i_sb)->usi_generation);
	atomic_set(&dtopd(inode->i_sb->s_root)->udi_generation, gen);
	atomic_set(&itopd(inode->i_sb->s_root->d_inode)->uii_generation, gen);

	err = 0;

      out:
	unlock_dentry(inode->i_sb->s_root);
	unionfs_write_unlock(inode->i_sb);
	KFREE(rdwrargs);

	print_exit_status(err);

	return err;
}

int unionfs_ioctl_queryfile(struct file *file, unsigned int cmd,
			    unsigned long arg)
{
	int err = 0;
	fd_set branchlist;

	int bstart = 0, bend = 0, bindex = 0;
	struct dentry *dentry, *hidden_dentry;

	print_entry_location();

	dentry = file->f_dentry;
	lock_dentry(dentry);
	if ((err = unionfs_partial_lookup(dentry)))
		goto out;
	bstart = dbstart(dentry);
	bend = dbend(dentry);

	FD_ZERO(&branchlist);

	for (bindex = bstart; bindex <= bend; bindex++) {
		hidden_dentry = dtohd_index(dentry, bindex);
		if (!hidden_dentry)
			continue;
		if (hidden_dentry->d_inode)
			FD_SET(bindex, &branchlist);
	}

	err = copy_to_user((void __user *)arg, &branchlist, sizeof(fd_set));
	if (err) {
		err = -EFAULT;
		goto out;
	}

      out:
	unlock_dentry(dentry);
	err = err < 0 ? err : bend;
	print_exit_status(err);
	return (err);
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
