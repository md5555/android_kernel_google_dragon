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
 *  $Id: file.c,v 1.143 2006/08/05 01:28:46 jro Exp $
 */

#include "unionfs.h"

/* declarations for sparse */
extern ssize_t unionfs_read(struct file *, char __user *, size_t, loff_t *);
extern ssize_t unionfs_write(struct file *, const char __user *, size_t,
			     loff_t *);

/*******************
 * File Operations *
 *******************/

#ifndef UNIONFS_MMAP
/* SP: Disable unionfs_llseek, as use generic_file_llseek on upper file */
static loff_t unionfs_llseek(struct file *file, loff_t offset, int origin)
{
	loff_t err;
	struct file *hidden_file = NULL;

	print_entry_location();

	dprint(PRINT_DEBUG, "unionfs_llseek: file=%p, offset=0x%llx, origin=%d\n",
		    file, offset, origin);

	if ((err = unionfs_file_revalidate(file, 0)))
		goto out;

	hidden_file = ftohf(file);
	/* always set hidden position to this one */
	hidden_file->f_pos = file->f_pos;

	memcpy(&(hidden_file->f_ra), &(file->f_ra),
	       sizeof(struct file_ra_state));

	if (hidden_file->f_op && hidden_file->f_op->llseek)
		err = hidden_file->f_op->llseek(hidden_file, offset, origin);
	else
		err = generic_file_llseek(hidden_file, offset, origin);

	if (err < 0)
		goto out;
	if (err != file->f_pos) {
		file->f_pos = err;
		// ION maybe this?
		//      file->f_pos = hidden_file->f_pos;

		file->f_version++;
	}
      out:
	print_exit_status((int)err);
	return err;
}
#endif
ssize_t __unionfs_read(struct file * file, char __user * buf, size_t count,
		       loff_t * ppos)
{
	int err = -EINVAL;
	struct file *hidden_file = NULL;
	loff_t pos = *ppos;

	print_file("entering __unionfs_read()", file);

	hidden_file = ftohf(file);
	if (!hidden_file->f_op || !hidden_file->f_op->read)
		goto out;

	err = hidden_file->f_op->read(hidden_file, buf, count, &pos);
	*ppos = pos;

      out:
	print_file("leaving __unionfs_read()", file);

	print_exit_status(err);
	return err;
}

ssize_t unionfs_read(struct file * file, char __user * buf, size_t count,
		     loff_t * ppos)
{
	int err = -EINVAL;

	print_entry_location();

	if ((err = unionfs_file_revalidate(file, 0)))
		goto out;

#ifdef UNIONFS_MMAP
	err = generic_file_read(file, buf, count, ppos);
	if (err >= 0)
		file_accessed(ftohf(file));
#else
	err = __unionfs_read(file, buf, count, ppos);
#endif

      out:

	print_exit_status(err);
	return err;
}

/* SP: Sendfile code not updated, but should be able to use
 * generic_file_sendfile, as it would use readpage, which we now have */
#ifdef SUPPORT_BROKEN_LOSETUP
static ssize_t unionfs_sendfile(struct file *file, loff_t * ppos,
				size_t count, read_actor_t actor, void *target)
{
	ssize_t err;
	struct file *hidden_file = NULL;

	print_entry_location();

	if ((err = unionfs_file_revalidate(file, 0)))
		goto out;

	hidden_file = ftohf(file);

	err = -EINVAL;
	if (!hidden_file->f_op || !hidden_file->f_op->sendfile)
		goto out;

	err = hidden_file->f_op->sendfile(hidden_file, ppos, count, actor,
					  target);

      out:
	print_exit_status(err);
	return err;
}
#endif
ssize_t __unionfs_write(struct file * file, const char __user * buf,
			size_t count, loff_t * ppos)
{
	int err = -EINVAL;
	struct file *hidden_file = NULL;
	struct inode *inode;
	struct inode *hidden_inode;
	loff_t pos = *ppos;
	int bstart, bend;

	print_entry_location();

	inode = file->f_dentry->d_inode;

	bstart = fbstart(file);
	bend = fbend(file);

	BUG_ON(bstart == -1);

	hidden_file = ftohf(file);
	hidden_inode = hidden_file->f_dentry->d_inode;

	if (!hidden_file->f_op || !hidden_file->f_op->write)
		goto out;

	/* adjust for append -- seek to the end of the file */
	if (file->f_flags & O_APPEND)
		pos = inode->i_size;

	err = hidden_file->f_op->write(hidden_file, buf, count, &pos);

	/*
	 * copy ctime and mtime from lower layer attributes
	 * atime is unchanged for both layers
	 */
	if (err >= 0)
		fist_copy_attr_times(inode, hidden_inode);

	*ppos = pos;

	/* update this inode's size */
	if (pos > inode->i_size)
		inode->i_size = pos;
      out:
	print_exit_status(err);
	return err;
}

ssize_t unionfs_write(struct file * file, const char __user * buf, size_t count,
		      loff_t * ppos)
{
	int err = 0;

	print_entry_location();

	if ((err = unionfs_file_revalidate(file, 1)))
		goto out;
#ifdef UNIONFS_MMAP
	err = generic_file_write(file, buf, count, ppos);
#else
	err = __unionfs_write(file, buf, count, ppos);
#endif
      out:
	print_exit_status(err);
	return err;
}

static int unionfs_file_readdir(struct file *file, void *dirent,
				filldir_t filldir)
{
	int err = -ENOTDIR;
	print_entry_location();
	print_exit_status(err);
	return err;
}

static unsigned int unionfs_poll(struct file *file, poll_table * wait)
{
	unsigned int mask = DEFAULT_POLLMASK;
	struct file *hidden_file = NULL;

	print_entry_location();

	if (unionfs_file_revalidate(file, 0)) {
		/* We should pretend an error happend. */
		mask = POLLERR | POLLIN | POLLOUT;
		goto out;
	}

	hidden_file = ftohf(file);

	if (!hidden_file->f_op || !hidden_file->f_op->poll)
		goto out;

	mask = hidden_file->f_op->poll(hidden_file, wait);

      out:
	print_exit_status(mask);
	return mask;
}

#ifndef UNIONFS_MMAP
static int __do_mmap(struct file *file, struct vm_area_struct *vma)
{
	int err;
	struct file *hidden_file;

	print_entry_location();
	hidden_file = ftohf(file);

	err = -ENODEV;
	if (!hidden_file->f_op || !hidden_file->f_op->mmap)
		goto out;

	vma->vm_file = hidden_file;
	err = hidden_file->f_op->mmap(hidden_file, vma);
	get_file(hidden_file);	/* make sure it doesn't get freed on us */
	fput(file);		/* no need to keep extra ref on ours */
      out:
	print_exit_status(err);
	return err;
}
#endif
/* SP: mmap code now maps upper file
 * like old code, will only copyup at this point, it's possible to copyup
 * in writepage(), but I haven't bothered with that, as only apt-get seem
 * to want to write to a shared/write mapping
 */
static int unionfs_mmap(struct file *file, struct vm_area_struct *vma)
{
	int err = 0;
	int willwrite;

	print_entry_location();

	/* This might could be deferred to mmap's writepage. */
	willwrite = ((vma->vm_flags | VM_SHARED | VM_WRITE) == vma->vm_flags);
	if ((err = unionfs_file_revalidate(file, willwrite)))
		goto out;
#ifdef UNIONFS_MMAP
	err = generic_file_mmap(file, vma);
	if (err) {
		printk("unionfs_mmap: generic_file_mmap failed\n");
	}
#else
	err = __do_mmap(file, vma);
#endif

      out:
	print_exit_status(err);
	return err;
}

/* SP: disabled as use the generic file_fsync */
#ifndef UNIONFS_MMAP
static int unionfs_fsync(struct file *file, struct dentry *dentry, int datasync)
{
	int err;
	struct file *hidden_file = NULL;

	print_entry_location();

	if ((err = unionfs_file_revalidate(file, 1)))
		goto out;

	hidden_file = ftohf(file);

	err = -EINVAL;
	if (!hidden_file->f_op || !hidden_file->f_op->fsync)
		goto out;

	mutex_lock(&hidden_file->f_dentry->d_inode->i_mutex);
	err = hidden_file->f_op->fsync(hidden_file, hidden_file->f_dentry,
				       datasync);
	mutex_unlock(&hidden_file->f_dentry->d_inode->i_mutex);

      out:
	print_exit_status(err);
	return err;
}
#endif

/* SP: disabled as none of the other in kernel fs's seem to use it */
static int unionfs_fasync(int fd, struct file *file, int flag)
{
	int err = 0;
	struct file *hidden_file = NULL;

	print_entry_location();

	if ((err = unionfs_file_revalidate(file, 1)))
		goto out;

	hidden_file = ftohf(file);

	if (hidden_file->f_op && hidden_file->f_op->fasync)
		err = hidden_file->f_op->fasync(fd, hidden_file, flag);

      out:
	print_exit_status(err);
	return err;
}

struct file_operations unionfs_main_fops = {
#ifdef UNIONFS_MMAP
	.llseek = generic_file_llseek,
#else
	.llseek = unionfs_llseek,
#endif
	.read = unionfs_read,
	.write = unionfs_write,
	.readdir = unionfs_file_readdir,
	.poll = unionfs_poll,
	.unlocked_ioctl = unionfs_ioctl,
	.mmap = unionfs_mmap,
	.open = unionfs_open,
	.flush = unionfs_flush,
	.release = unionfs_file_release,
#ifdef UNIONFS_MMAP
	.fsync = file_fsync,
#else
	.fsync = unionfs_fsync,
#endif
	.fasync = unionfs_fasync,
#ifdef SUPPORT_BROKEN_LOSETUP
	.sendfile = unionfs_sendfile,
#endif
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
