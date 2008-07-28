/*
 * Copyright (c) 2003-2006 Erez Zadok
 * Copyright (c) 2003-2006 Charles P. Wright
 * Copyright (c) 2005-2006 Josef Sipek
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
 *  $Id: persistent_inode.c,v 1.36 2006/07/08 17:58:31 ezk Exp $
 */
#ifdef UNIONFS_IMAP

#include "unionfs.h"

static ssize_t __fread(struct file *filp, void *buf, size_t size, loff_t * pos)
{
	int err;
	mm_segment_t oldfs;
	ssize_t(*func) (struct file *, char __user *, size_t, loff_t *);

	func = do_sync_read;
	if (filp->f_op && filp->f_op->read)
		func = filp->f_op->read;

	oldfs = get_fs();
	set_fs(KERNEL_DS);
	do {
		err = func(filp, (char __user *)buf, size, pos);
	} while (err == -EAGAIN || err == -EINTR);
	set_fs(oldfs);
	return err;
}

static ssize_t __fwrite(struct file *filp, void *buf, size_t size, loff_t * pos)
{
	int err;
	mm_segment_t oldfs;
	unsigned long flim;
	struct rlimit *rl;
	ssize_t(*func) (struct file *, const char __user *, size_t, loff_t *);

	func = do_sync_write;
	if (filp->f_op && filp->f_op->write)
		func = filp->f_op->write;

	/*
	 * it breaks RLIMIT_FSIZE,
	 * but users should be careful to quota.
	 */
	rl = current->signal->rlim + RLIMIT_FSIZE;
	flim = rl->rlim_cur;
	rl->rlim_cur = RLIM_INFINITY;
	oldfs = get_fs();
	set_fs(KERNEL_DS);
	do {
		err = func(filp, (const char __user *)buf, size, pos);
	} while (err == -EAGAIN || err == -EINTR);
	set_fs(oldfs);
	rl->rlim_cur = flim;
	return err;
}

/*
 * verify_forwardmap(super_block *sb)
 * sb: pointer to a superblock containing the forwardmap.
 * returns: 0 on success EINVAL or ENOMEM on failure;
 */
static int verify_forwardmap(struct super_block *sb)
{
	int err = 0, bytesread = 0, bindex = 0, mallocsize = 0;
	loff_t readpos = 0;
	struct file *forwardmap = NULL;
	struct fmaphdr header;
	struct unionfs_sb_info *spd = NULL;
	print_entry_location();

	spd = stopd(sb);
	BUG_ON(!spd);

	forwardmap = spd->usi_forwardmap;
	if (!forwardmap) {
		err = -EINVAL;
		goto out;
	}
	bytesread = __fread(forwardmap, &header, sizeof(struct fmaphdr),
			    &readpos);
	if (bytesread < sizeof(struct fmaphdr)) {
		err = -EINVAL;
		goto out;
	}
	if (header.magic != FORWARDMAP_MAGIC
	    || header.version != FORWARDMAP_VERSION) {
		err = -EINVAL;
		goto out;
	}
	spd->usi_bmap =
	    KMALLOC(sizeof(struct bmapent) * header.usedbranches, GFP_KERNEL);

	if (!spd->usi_bmap) {
		err = -ENOMEM;
		goto out;
	}

	while (bindex < header.usedbranches) {
		bytesread = __fread(forwardmap, &stopd(sb)->usi_bmap[bindex],
				    sizeof(struct bmapent), &readpos);
		if (bytesread < sizeof(struct bmapent)) {
			err = -EINVAL;
			goto out_err;
		}
		bindex++;
	}

	mallocsize = sizeof(int) * header.usedbranches;
	goto out;
      out_err:
	if (spd->usi_bmap)
		KFREE(spd->usi_bmap);
      out:
	print_exit_status(err);
	return err;
}

/*
 * verify_reversemap(struct super_block sb, int rmapindex)
 *
 * sb: The unionfs superblock containing all of the current imap info
 * rmapindex: the index in the usi_reversemaps array that we wish to
 * verify
 *
 * Assumes the reverse maps less than rmapindex are valid.
 *
 * returns: 0 if the opperation succeds
 * 	-EINVAL if the map file does not belong to the forward map
 *
 */
static int verify_reversemap(struct super_block *sb, int rmapindex,
			     struct unionfs_dentry_info *hidden_root_info)
{
	int err = 0, i = 0, bindex = 0, found = 0, bytesread;
	loff_t readpos = 0;
	struct file *forwardmap, *reversemap;
	struct fmaphdr fheader;
	struct rmaphdr rheader;
	struct kstatfs st;
	struct unionfs_sb_info *spd = NULL;

	print_entry_location();

	spd = stopd(sb);
	BUG_ON(!spd);

	forwardmap = spd->usi_forwardmap;
	if (!forwardmap) {
		err = -EINVAL;
		goto out;
	}
	reversemap = spd->usi_reversemaps[rmapindex];
	if (!reversemap) {
		err = -EINVAL;
		goto out;
	}
	bytesread = __fread(forwardmap, &fheader, sizeof(struct fmaphdr),
			    &readpos);
	if (bytesread < sizeof(struct fmaphdr)) {
		err = -EINVAL;
		goto out;
	}
	readpos = 0;
	bytesread = __fread(reversemap, &rheader, sizeof(struct rmaphdr),
			    &readpos);
	if (bytesread < sizeof(struct rmaphdr)) {
		err = -EINVAL;
		goto out;
	}
	if (rheader.magic != REVERSEMAP_MAGIC
	    || rheader.version != REVERSEMAP_VERSION) {
		err = -EINVAL;
		goto out;
	}
	if (memcmp(fheader.uuid, rheader.fwduuid, sizeof(fheader.uuid))) {
		err = -EINVAL;
		goto out;
	}

	/* XXX: Ok so here we take the new map and read the fsid from it. Then
	 * we go through all the branches in the union and see which ones it
	 * matches with*/
	for (i = 0; i < spd->usi_num_bmapents && !found; i++) {
		if (memcmp
		    (rheader.revuuid, spd->usi_bmap[i].uuid,
		     sizeof(rheader.revuuid)))
			continue;

		found = 1;
		for (bindex = 0; bindex <= hidden_root_info->udi_bend; bindex++) {
			struct dentry *d;
			fsid_t fsid;
			dev_t dev;
			memset(&st, 0, sizeof(struct kstatfs));

			d = hidden_root_info->udi_dentry[bindex];

			err = d->d_sb->s_op->statfs(d->d_sb, &st);
			if (err)
				goto out;

			if (st.f_fsid.val[0] || st.f_fsid.val[1]) {
				fsid = st.f_fsid;
			} else {

				dev = d->d_sb->s_dev;
				fsid.val[0] = MAJOR(dev);
				fsid.val[1] = MINOR(dev);
			}

			if (memcmp(&fsid, &rheader.fsid, sizeof(fsid)))
				continue;

			if (spd->usi_bnum_table[bindex] == -1)
				spd->usi_bnum_table[bindex] = i;
			if (spd->usi_map_table[bindex]) {
				printk(KERN_WARNING
				       "Two reverse maps share fsid %u%u!\n",
				       rheader.fsid.val[0],
				       rheader.fsid.val[1]);
				err = -EINVAL;
				goto out;
			} else {
				spd->usi_map_table[bindex] = reversemap;
			}
		}
	}
	if (!found) {
		printk(KERN_WARNING
		       "Could not match the reversemap uuid with an entry in the forwardmap table\n");
		err = -EINVAL;
	}
      out:
	print_exit_status(err);
	return err;
}

int init_imap_data(struct super_block *sb,
		   struct unionfs_dentry_info *hidden_root_info)
{
	int i, err = 0, mallocsize = 0;
	struct unionfs_sb_info *spd;

	print_entry_location();

	spd = stopd(sb);

	spd->usi_forwardmap = NULL;
	spd->usi_reversemaps = NULL;
	spd->usi_bnum_table = NULL;

	mallocsize = sizeof(struct file *) * (hidden_root_info->udi_bend + 1);
	spd->usi_reversemaps = KZALLOC(mallocsize, GFP_KERNEL);
	if (!spd->usi_reversemaps) {
		err = -ENOMEM;
		goto out_error;
	}

	spd->usi_map_table = KZALLOC(mallocsize, GFP_KERNEL);
	if (!spd->usi_map_table) {
		err = -ENOMEM;
		goto out_error;
	}

	mallocsize = sizeof(int) * (hidden_root_info->udi_bend + 1);
	spd->usi_bnum_table = KMALLOC(mallocsize, GFP_KERNEL);
	if (!spd->usi_bnum_table) {
		err = -ENOMEM;
		goto out_error;
	}

	for (i = 0; i <= hidden_root_info->udi_bend; i++) {
		spd->usi_bnum_table[i] = -1;
	}

	if (!err)
		goto out;
      out_error:

	if (spd->usi_reversemaps) {
		KFREE(spd->usi_reversemaps);
		spd->usi_reversemaps = NULL;
	}

	if (spd->usi_map_table) {
		KFREE(spd->usi_map_table);
		spd->usi_map_table = NULL;
	}

	if (spd->usi_bnum_table) {
		KFREE(spd->usi_bnum_table);
		spd->usi_bnum_table = NULL;

	}

      out:
	print_exit_status(err);
	return err;

}

void cleanup_imap_data(struct super_block *sb)
{
	int count = 0;
	struct unionfs_sb_info *spd;

	print_entry_location();

	spd = stopd(sb);

	spd->usi_persistent = 0;
	count = spd->usi_num_bmapents;
	while (count - 1 >= 0) {
		if (spd->usi_reversemaps[count - 1]) {
			filp_close(spd->usi_reversemaps[count - 1], NULL);
			spd->usi_reversemaps[count - 1] = NULL;
		}
		count--;
	}
	if (spd->usi_reversemaps) {
		KFREE(spd->usi_reversemaps);
		spd->usi_reversemaps = NULL;
	}

	if (spd->usi_map_table) {
		KFREE(spd->usi_map_table);
		spd->usi_map_table = NULL;
	}

	if (spd->usi_bnum_table) {
		KFREE(spd->usi_bnum_table);
		spd->usi_bnum_table = NULL;
	}
	if (spd->usi_forwardmap) {
		filp_close(spd->usi_forwardmap, NULL);
		spd->usi_forwardmap = NULL;
	}
	print_exit_location();
}

int parse_imap_option(struct super_block *sb,
		      struct unionfs_dentry_info *hidden_root_info,
		      char *options)
{
	int count = 0, err = 0;
	char *name;
	struct unionfs_sb_info *spd = NULL;

	print_entry_location();
	spd = stopd(sb);
	BUG_ON(!spd);

	err = init_imap_data(sb, hidden_root_info);
	if (err)
		goto out_error;
	while ((name = strsep(&options, ":")) != NULL) {
		if (!*name)
			continue;
		if (!spd->usi_forwardmap) {
			spd->usi_forwardmap = filp_open(name, O_RDWR, 0);
			if (IS_ERR(spd->usi_forwardmap)) {
				err = PTR_ERR(spd->usi_forwardmap);
				spd->usi_forwardmap = NULL;
				goto out_error;
			}
		} else {
			spd->usi_reversemaps[count] =
			    filp_open(name, O_RDWR, 0);
			if (IS_ERR(spd->usi_reversemaps[count])) {
				err = PTR_ERR(spd->usi_reversemaps[count]);
				spd->usi_reversemaps[count] = NULL;
				goto out_error;

			}
			count++;
		}
	}
	if (count <= 0) {
		printk(KERN_WARNING "unionfs: no reverse maps specified.\n");
		err = -EINVAL;
	}
	if (err)
		goto out_error;

	/* Initialize the super block's next_avail field */
	/* Dave, you can't use 64-bit division here because the i386 doesn't
	 * support it natively.  Instead you need to punt if the size is
	 * greater than unsigned long, and then cast it down.  Then you should
	 * be able to assign to this value, without having these problems. */

	if (spd->usi_forwardmap->f_dentry->d_inode->i_size > ULONG_MAX) {
		err = -EFBIG;
		goto out_error;
	}
	spd->usi_next_avail =
	    ((unsigned long)(spd->usi_forwardmap->f_dentry->d_inode->
			     i_size - (sizeof(struct fmaphdr) +
				       sizeof(struct bmapent[256])))
	     / sizeof(struct fmapent));

	if (spd->usi_next_avail < FIRST_VALID_INODE)
		spd->usi_next_avail = FIRST_VALID_INODE;

	spd->usi_num_bmapents = count;
	err = verify_forwardmap(sb);
	if (err)
		goto out_error;
	while (count > 0) {
		err = verify_reversemap(sb, --count, hidden_root_info);
		if (err)
			goto out_error;
	}
	spd->usi_persistent = 1;

	goto out;

      out_error:
	spd->usi_num_bmapents = count;
	cleanup_imap_data(sb);

      out:
	print_exit_status(err);
	return err;
}

 /*
  * get @ino from @hidden_ino.
  */
static int __read_uin(struct unionfs_sb_info *sbi, ino_t hidden_ino, int bindex,
		      ino_t * ino)
{
	int err;
	struct file *rev;
	loff_t pos;
	ssize_t sz;
	uint64_t ino64;
	const int elmnt = sizeof(ino64);

	rev = sbi->usi_map_table[bindex];
	pos = sizeof(struct rmaphdr) + elmnt * hidden_ino;
	*ino = 0;
	err = 0;
	if (pos + elmnt > rev->f_dentry->d_inode->i_size)
		goto out;

	sz = __fread(rev, &ino64, elmnt, &pos);
	err = sz;
	if (err < 0)
		goto out;
	err = 0;
	*ino = -1;
	if (sz != elmnt || ino64 > *ino)
		err = -EIO;
	*ino = ino64;
      out:
	print_exit_status(err);
	return err;
}

/*
 * put unionfs @ino for @hidden_ino on @bindex.
 */
static int __write_uin(struct unionfs_sb_info *sbi, ino_t ino, int bindex,
		       ino_t hidden_ino)
{
	struct file *fwd, *rev;
	struct fmapent ent;
	loff_t pos;
	ssize_t sz;
	int err;
	uint64_t ino64;
	const int fwdhdr = sizeof(struct fmaphdr) + sizeof(struct bmapent[256]);
	const int fwd_elmnt = sizeof(ent);
	const int rev_elmnt = sizeof(ino64);

	err = -ENOSPC;
	if (ino < FIRST_VALID_INODE)
		goto out;

	fwd = sbi->usi_forwardmap;
	ent.fsnum = sbi->usi_bnum_table[bindex];
	ent.inode = hidden_ino;
	pos = fwdhdr + fwd_elmnt * ino;
	sz = __fwrite(fwd, &ent, fwd_elmnt, &pos);
	err = sz;
	if (err < 0)
		goto out;
	err = -EIO;
	if (sz != fwd_elmnt)
		goto out;

	rev = sbi->usi_map_table[bindex];
	pos = sizeof(struct rmaphdr) + rev_elmnt * hidden_ino;
	ino64 = ino;
	sz = __fwrite(rev, &ino64, rev_elmnt, &pos);
	err = sz;
	if (err < 0)
		goto out;
	err = 0;
	if (sz != rev_elmnt)
		err = -EIO;
      out:
	print_exit_status(err);
	return err;
}

/*
 * read_uin(struct super_block *sb, uint8_t branchnum, ino_t inode_number, int flag, ino_t *uino)
 * fsnum: branch to reference when getting the inode number
 * inode_number: lower level inode number use to reference the proper inode.
 * flag: if set to O_CREAT it will creat the entry if it doesent exist
 * 		 otherwise it will return the existing one.
 * returns: the unionfs inode number either created or retrieved based on
 * 			the information.
 */
int read_uin(struct super_block *sb, uint8_t branchnum, ino_t inode_number,
	     int flag, ino_t * uino)
{
	int err = 0;
	struct unionfs_sb_info *spd;

	print_entry_location();

	spd = stopd(sb);
	BUG_ON(!spd);

	/* Find appropriate reverse map and then read from the required position */
	/* get it from the array. */
	err = __read_uin(spd, inode_number, branchnum, uino);
	if (err || *uino)
		goto out;

	err = -EIO;
	if (!(flag & O_CREAT))
		goto out;

	/* If we haven't found an entry and we have the O_CREAT flag set we want to
	 * create a new entry write it out to the file and return its index
	 */
	mutex_lock(&sb->s_lock);
	*uino = spd->usi_next_avail++;
	err = __write_uin(spd, *uino, branchnum, inode_number);
	if (err)
		spd->usi_next_avail--;
	mutex_unlock(&sb->s_lock);
      out:
	print_exit_status(err);
	return err;
}

int write_uin(struct super_block *sb, ino_t ino, int bindex, ino_t hidden_ino)
{
	int err;

	print_entry_location();
	err = __write_uin(stopd(sb), ino, bindex, hidden_ino);
	print_exit_status(err);
	return err;
}

/*
 * get_lin(ino_t inode_number)
 * inode_number : inode number for the unionfs inode
 * returns: the lower level inode# and branch#
 */
/* entry should use a poiner on the stack. should be staticly allocated one
 * level up*/
int get_lin(struct super_block *sb, ino_t inode_number, struct fmapent *entry)
{
	struct file *forwardmap;
	loff_t seek_size;
	mm_segment_t oldfs;
	int err = 0, bytesread = 0;

	print_entry_location();

	if (!entry) {
		entry = ERR_PTR(-ENOMEM);
		goto out;
	}
	forwardmap = stopd(sb)->usi_forwardmap;
	seek_size =
	    sizeof(struct fmaphdr) + sizeof(struct bmapent[256]) +
	    (sizeof(struct fmapent) * inode_number);
	oldfs = get_fs();
	set_fs(KERNEL_DS);
	bytesread = __fread(forwardmap, entry, sizeof(*entry), &seek_size);
	set_fs(oldfs);
	if (bytesread != sizeof(*entry))
		err = -EINVAL;

      out:
	print_exit_location();
	return err;
}

/*
 * remove_map(struct super_block *sb,int bindex)
 *
 * sb: The super block containing all the current imap info
 * bindex: the index of the branch that is being removed.
 *
 * This assumes that end hasen't been decremented yet.
 *
 * Returns: This function really can't fail. The only thing
 * that could possibly happen is that it will oops but that
 * requires unionfs to be in an inconsistant state which
 * shoulden't happen.
 */
int remove_map(struct super_block *sb, int bindex)
{
	int i;
	struct unionfs_sb_info *spd;

	print_entry_location();

	spd = stopd(sb);
	BUG_ON(!spd);

	for (i = bindex; i < sbend(sb); i++) {
		spd->usi_map_table[i] = spd->usi_map_table[i + 1];
		spd->usi_bnum_table[i] = spd->usi_bnum_table[i + 1];
	}
	return 0;
}

#endif
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
