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
 * file and vm operations
 *
 * $Id: f_op.c,v 1.11 2008/09/01 02:55:44 sfjro Exp $
 */

#include <linux/fs_stack.h>
#include <linux/poll.h>
#include "aufs.h"

/* common function to regular file and dir */
int aufs_flush(struct file *file, fl_owner_t id)
{
	int err;
	struct dentry *dentry;
	aufs_bindex_t bindex, bend;
	struct file *h_file;

	dentry = file->f_dentry;
	LKTRTrace("%.*s\n", AuDLNPair(dentry));

	si_noflush_read_lock(dentry->d_sb);
	fi_read_lock(file);
	di_read_lock_child(dentry, AuLock_IW);

	err = 0;
	bend = au_fbend(file);
	for (bindex = au_fbstart(file); !err && bindex <= bend; bindex++) {
		h_file = au_h_fptr(file, bindex);
		if (h_file && h_file->f_op && h_file->f_op->flush) {
			err = h_file->f_op->flush(h_file, id);
			if (!err)
				au_update_fuse_h_inode
					(h_file->f_vfsmnt, h_file->f_dentry);
			/*ignore*/
		}
	}
	au_cpup_attr_timesizes(dentry->d_inode);

	di_read_unlock(dentry, AuLock_IW);
	fi_read_unlock(file);
	si_read_unlock(dentry->d_sb);
	AuTraceErr(err);
	return err;
}

/* ---------------------------------------------------------------------- */

static int do_open_nondir(struct file *file, int flags)
{
	int err;
	aufs_bindex_t bindex;
	struct super_block *sb;
	struct file *h_file;
	struct dentry *dentry;
	struct inode *inode;
	struct au_finfo *finfo;

	dentry = file->f_dentry;
	LKTRTrace("%.*s, flags 0%o\n", AuDLNPair(dentry), flags);
	FiMustWriteLock(file);
	inode = dentry->d_inode;
	AuDebugOn(!inode || S_ISDIR(inode->i_mode));

	err = 0;
	finfo = au_fi(file);
	finfo->fi_h_vm_ops = NULL;
	sb = dentry->d_sb;
	bindex = au_dbstart(dentry);
	AuDebugOn(!au_h_dptr(dentry, bindex)->d_inode);
	/* O_TRUNC is processed already */
	BUG_ON(au_test_ro(sb, bindex, inode) && (flags & O_TRUNC));

	h_file = au_h_open(dentry, bindex, flags, file);
	if (IS_ERR(h_file))
		err = PTR_ERR(h_file);
	else {
		au_set_fbstart(file, bindex);
		au_set_fbend(file, bindex);
		au_set_h_fptr(file, bindex, h_file);
		au_update_figen(file);
		/* todo: necessary? */
		/* file->f_ra = h_file->f_ra; */
		err = 0;
	}
	AuTraceErr(err);
	return err;
}

static int aufs_open_nondir(struct inode *inode, struct file *file)
{
	LKTRTrace("i%lu, %.*s\n", inode->i_ino, AuDLNPair(file->f_dentry));

	return au_do_open(inode, file, do_open_nondir);
}

static int aufs_release_nondir(struct inode *inode, struct file *file)
{
	struct super_block *sb = file->f_dentry->d_sb;

	LKTRTrace("i%lu, %.*s\n", inode->i_ino, AuDLNPair(file->f_dentry));

	si_noflush_read_lock(sb);
	au_finfo_fin(file);
	si_read_unlock(sb);
	return 0;
}

/* ---------------------------------------------------------------------- */

static ssize_t aufs_read(struct file *file, char __user *buf, size_t count,
			 loff_t *ppos)
{
	ssize_t err;
	struct dentry *dentry;
	struct file *h_file;
	struct super_block *sb;
	struct inode *h_inode;

	dentry = file->f_dentry;
	LKTRTrace("%.*s, cnt %lu, pos %lld\n",
		  AuDLNPair(dentry), (unsigned long)count, *ppos);

	sb = dentry->d_sb;
	si_read_lock(sb, AuLock_FLUSH);
	err = au_reval_and_lock_fdi(file, au_reopen_nondir, /*wlock*/0,
				    /*locked*/0);
	if (unlikely(err))
		goto out;

	/* support LSM and notify */
	h_file = au_h_fptr(file, au_fbstart(file));
	h_inode = h_file->f_dentry->d_inode;
	err = vfsub_read_u(h_file, buf, count, ppos,
			   au_test_dlgt(au_mntflags(sb)));
	/* todo: necessary? */
	/* file->f_ra = h_file->f_ra; */
	fsstack_copy_attr_atime(dentry->d_inode, h_file->f_dentry->d_inode);

	di_read_unlock(dentry, AuLock_IR);
	fi_read_unlock(file);
 out:
	si_read_unlock(sb);
	AuTraceErr(err);
	return err;
}

static ssize_t aufs_write(struct file *file, const char __user *ubuf,
			  size_t count, loff_t *ppos)
{
	ssize_t err;
	struct dentry *dentry;
	struct inode *inode;
	struct super_block *sb;
	unsigned int mnt_flags;
	struct file *h_file;
	char __user *buf = (char __user *)ubuf;
	struct au_hin_ignore ign;
	struct vfsub_args vargs;
	aufs_bindex_t bstart;
	int hinotify;
	struct au_pin pin;

	dentry = file->f_dentry;
	LKTRTrace("%.*s, cnt %lu, pos %lld\n",
		  AuDLNPair(dentry), (unsigned long)count, *ppos);

	inode = dentry->d_inode;
	mutex_lock(&inode->i_mutex);
	sb = dentry->d_sb;
	si_read_lock(sb, AuLock_FLUSH);
	mnt_flags = au_mntflags(sb);
	hinotify = !!au_opt_test(mnt_flags, UDBA_INOTIFY);
	vfsub_args_init(&vargs, &ign, au_test_dlgt(mnt_flags), 0);

	err = au_reval_and_lock_fdi(file, au_reopen_nondir, /*wlock*/1,
				    /*locked*/1);
	if (unlikely(err))
		goto out;
	err = au_ready_to_write(file, -1, &pin);
	di_downgrade_lock(dentry, AuLock_IR);
	if (unlikely(err))
		goto out_unlock;

	bstart = au_fbstart(file);
	h_file = au_h_fptr(file, bstart);
	if (!hinotify) {
		au_unpin(&pin);
		err = vfsub_write_u(h_file, buf, count, ppos, &vargs);
	} else {
		vfsub_ign_hinode(&vargs, IN_MODIFY,
				 au_pinned_hdir(&pin, bstart));
		err = vfsub_write_u(h_file, buf, count, ppos, &vargs);
		au_unpin(&pin);
	}
	au_cpup_attr_timesizes(inode);

 out_unlock:
	di_read_unlock(dentry, AuLock_IR);
	fi_write_unlock(file);
 out:
	si_read_unlock(sb);
	mutex_unlock(&inode->i_mutex);
	AuTraceErr(err);
	return err;
}

#ifdef CONFIG_AUFS_SPLICE_PATCH
static int au_test_loopback(void)
{
	const char c = current->comm[4];
	/* true if a kernel thread named 'loop[0-9].*' accesses a file */
	const int loopback = (current->mm == NULL
			      && '0' <= c && c <= '9'
			      && strncmp(current->comm, "loop", 4) == 0);
	return loopback;
}

static ssize_t aufs_splice_read(struct file *file, loff_t *ppos,
				struct pipe_inode_info *pipe, size_t len,
				unsigned int flags)
{
	ssize_t err;
	struct file *h_file;
	struct dentry *dentry;
	struct super_block *sb;

	dentry = file->f_dentry;
	LKTRTrace("%.*s, pos %lld, len %lu\n",
		  AuDLNPair(dentry), *ppos, (unsigned long)len);

	sb = dentry->d_sb;
	si_read_lock(sb, AuLock_FLUSH);
	err = au_reval_and_lock_fdi(file, au_reopen_nondir, /*wlock*/0,
				    /*locked*/0);
	if (unlikely(err))
		goto out;

	err = -EINVAL;
	/* support LSM and notify */
	h_file = au_h_fptr(file, au_fbstart(file));
	if (/* unlikely */(au_test_loopback())) {
		file->f_mapping = h_file->f_mapping;
		smp_mb(); /* unnecessary? */
	}
	err = vfsub_splice_to(h_file, ppos, pipe, len, flags,
			      au_test_dlgt(au_mntflags(sb)));
	/* todo: necessasry? */
	/* file->f_ra = h_file->f_ra; */
	fsstack_copy_attr_atime(dentry->d_inode, h_file->f_dentry->d_inode);
	di_read_unlock(dentry, AuLock_IR);
	fi_read_unlock(file);

 out:
	si_read_unlock(sb);
	AuTraceErr(err);
	return err;
}

static ssize_t
aufs_splice_write(struct pipe_inode_info *pipe, struct file *file, loff_t *ppos,
		  size_t len, unsigned int flags)
{
	ssize_t err;
	struct dentry *dentry;
	struct inode *inode, *h_inode;
	struct super_block *sb;
	struct file *h_file;
	/* struct au_hin_ignore ign; */
	struct vfsub_args vargs;
	unsigned int mnt_flags;
	struct au_pin pin;

	dentry = file->f_dentry;
	LKTRTrace("%.*s, len %lu, pos %lld\n",
		  AuDLNPair(dentry), (unsigned long)len, *ppos);

	inode = dentry->d_inode;
	mutex_lock(&inode->i_mutex);
	sb = dentry->d_sb;
	si_read_lock(sb, AuLock_FLUSH);
	mnt_flags = au_mntflags(sb);
	vfsub_args_init(&vargs, /*&ign*/NULL, au_test_dlgt(mnt_flags), 0);

	err = au_reval_and_lock_fdi(file, au_reopen_nondir, /*wlock*/1,
				    /*locked*/1);
	if (unlikely(err))
		goto out;
	err = au_ready_to_write(file, -1, &pin);
	di_downgrade_lock(dentry, AuLock_IR);
	if (unlikely(err))
		goto out_unlock;

	/* support LSM and notify */
	/* current vfs_splice_from() doesn't fire up the inotify event */
	h_file = au_h_fptr(file, au_fbstart(file));
	h_inode = h_file->f_dentry->d_inode;
	if (1 || !au_opt_test(mnt_flags, UDBA_INOTIFY)) {
		au_unpin(&pin);
		err = vfsub_splice_from(pipe, h_file, ppos, len, flags, &vargs);
	}
#if 0 /* reserved for future use */
	else {
		struct dentry *parent = dget_parent(dentry);
		vfsub_ign_hinode(&vargs, IN_MODIFY,
				 au_pinned_hdir(&pin, bstart));
		err = vfsub_splice_from(pipe, h_file, ppos, len, flags, &vargs);
		au_unpin(&pin);
	}
#endif
	au_cpup_attr_timesizes(inode);

 out_unlock:
	di_read_unlock(dentry, AuLock_IR);
	fi_write_unlock(file);
 out:
	si_read_unlock(sb);
	mutex_unlock(&inode->i_mutex);
	AuTraceErr(err);
	return err;
}
#endif /* CONFIG_AUFS_SPLICE_PATCH */

/* ---------------------------------------------------------------------- */

static int aufs_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
	int err;
	struct dentry *dentry;
	struct file *file, *h_file;
	struct inode *inode;
	static DECLARE_WAIT_QUEUE_HEAD(wq);
	struct au_finfo *finfo;

	AuTraceEnter();
	AuDebugOn(!vma || !vma->vm_file);
	/* todo: non-robr mode, user vm_file as it is? */
	wait_event(wq, (file = au_robr_safe_file(vma)));
	AuDebugOn(!au_test_aufs(file->f_dentry->d_sb));
	dentry = file->f_dentry;
	LKTRTrace("%.*s\n", AuDLNPair(dentry));
	inode = dentry->d_inode;
	AuDebugOn(!S_ISREG(inode->i_mode));

	/* do not revalidate, no si lock */
	finfo = au_fi(file);
	h_file = finfo->fi_hfile[0 + finfo->fi_bstart].hf_file;
	AuDebugOn(!h_file || !au_test_mmapped(file));
	fi_write_lock(file);
	vma->vm_file = h_file;
	err = finfo->fi_h_vm_ops->fault(vma, vmf);
	/* todo: necessary? */
	/* file->f_ra = h_file->f_ra; */
	au_robr_reset_file(vma, file);
	fi_write_unlock(file);
#if 0 /* def CONFIG_SMP */
	/* wake_up_nr(&wq, online_cpu - 1); */
	wake_up_all(&wq);
#else
	wake_up(&wq);
#endif

	if (!(err & VM_FAULT_ERROR)) {
#if 0 /* debug */
		struct page *page;
		page = vmf->page;
		AuDbg("%p, %d\n", page, page_mapcount(page));

		page->mapping = file->f_mapping;
		get_page(page);
		file->f_mapping = h_file->f_mapping;
		touch_atime(NULL, dentry);
		inode->i_atime = h_file->f_dentry->d_inode->i_atime;
#endif
	}
	AuTraceErr(err);
	return err;
}

static struct vm_operations_struct aufs_vm_ops = {
	.fault		= aufs_fault,
#if 0 /* reserved for future use */
	.open		= aufs_vmaopen,
	.close		= aufs_vmaclose,
	unsigned long (*nopfn)(struct vm_area_struct *area,
			       unsigned long address);
	page_mkwrite(struct vm_area_struct *vma, struct page *page)
#endif
};

/* ---------------------------------------------------------------------- */

static struct vm_operations_struct *au_vm_ops(struct file *h_file,
					      struct vm_area_struct *vma)
{
	struct vm_operations_struct *vm_ops;
	int err;

	AuTraceEnter();

	au_br_nfs_lockdep_off(h_file->f_vfsmnt->mnt_sb);
	err = h_file->f_op->mmap(h_file, vma);
	au_br_nfs_lockdep_on(h_file->f_vfsmnt->mnt_sb);
	vm_ops = ERR_PTR(err);
	if (unlikely(err))
		goto out;
	vm_ops = vma->vm_ops;
	err = do_munmap(current->mm, vma->vm_start,
			vma->vm_end - vma->vm_start);
	if (unlikely(err)) {
		AuIOErr("failed internal unmapping %.*s, %d\n",
			AuDLNPair(h_file->f_dentry), err);
		vm_ops = ERR_PTR(-EIO);
	}

 out:
	AuTraceErrPtr(vm_ops);
	return vm_ops;
}

static int aufs_mmap(struct file *file, struct vm_area_struct *vma)
{
	int err;
	unsigned char wlock, mmapped;
	struct dentry *dentry;
	struct super_block *sb;
	struct file *h_file;
	struct vm_operations_struct *vm_ops;

	dentry = file->f_dentry;
	LKTRTrace("%.*s, %lx, len %lu\n",
		  AuDLNPair(dentry), vma->vm_start,
		  vma->vm_end - vma->vm_start);
	AuDebugOn(!S_ISREG(dentry->d_inode->i_mode));
	AuDebugOn(down_write_trylock(&vma->vm_mm->mmap_sem));

	mmapped = au_test_mmapped(file); /* can be harmless race condition */
	wlock = !!(file->f_mode & FMODE_WRITE);
	sb = dentry->d_sb;
	si_read_lock(sb, AuLock_FLUSH);
	err = au_reval_and_lock_fdi(file, au_reopen_nondir, wlock | !mmapped,
				/*locked*/0);
	if (unlikely(err))
		goto out;

	if (wlock) {
		struct au_pin pin;

		err = au_ready_to_write(file, -1, &pin);
		di_downgrade_lock(dentry, AuLock_IR);
		if (unlikely(err))
			goto out_unlock;
		au_unpin(&pin);
	} else if (!mmapped)
		di_downgrade_lock(dentry, AuLock_IR);

	h_file = au_h_fptr(file, au_fbstart(file));
	if (unlikely(au_test_fuse(h_file->f_dentry->d_sb))) {
		/*
		 * by this assignment, f_mapping will differs from aufs inode
		 * i_mapping.
		 * if someone else mixes the use of f_dentry->d_inode and
		 * f_mapping->host, then a problem may arise.
		 */
		file->f_mapping = h_file->f_mapping;
	}

	if (0 && h_file->f_op->mmap == generic_file_mmap) {
		err = generic_file_mmap(file, vma); /* instead of h_file */
		if (unlikely(err))
			goto out_unlock;
		au_fi(file)->fi_h_vm_ops = vma->vm_ops;
	} else {
		vm_ops = NULL;
		if (!mmapped) {
			vm_ops = au_vm_ops(h_file, vma);
			err = PTR_ERR(vm_ops);
			if (IS_ERR(vm_ops))
				goto out_unlock;
		}

		err = generic_file_mmap(file, vma);
		if (unlikely(err))
			goto out_unlock;
		vma->vm_ops = &aufs_vm_ops;
		/* test again */
		if (!au_test_mmapped(file)) {
			FiMustWriteLock(file);
			au_fi(file)->fi_h_vm_ops = vm_ops;
		}
	}

	file_accessed(h_file);
	au_update_fuse_h_inode(h_file->f_vfsmnt, h_file->f_dentry); /*ignore*/
	fsstack_copy_attr_atime(dentry->d_inode, h_file->f_dentry->d_inode);

 out_unlock:
	di_read_unlock(dentry, AuLock_IR);
	if (!wlock && mmapped)
		fi_read_unlock(file);
	else
		fi_write_unlock(file);
 out:
	si_read_unlock(sb);
	AuTraceErr(err);
	return err;
}

/* ---------------------------------------------------------------------- */

static unsigned int aufs_poll(struct file *file, poll_table *wait)
{
	unsigned int mask;
	struct file *h_file;
	int err;
	struct dentry *dentry;
	struct super_block *sb;

	dentry = file->f_dentry;
	LKTRTrace("%.*s, wait %p\n", AuDLNPair(dentry), wait);
	AuDebugOn(S_ISDIR(dentry->d_inode->i_mode));

	/* We should pretend an error happened. */
	mask = POLLERR /* | POLLIN | POLLOUT */;
	sb = dentry->d_sb;
	si_read_lock(sb, AuLock_FLUSH);
	err = au_reval_and_lock_fdi(file, au_reopen_nondir, /*wlock*/0,
				    /*locked*/0);
	if (unlikely(err))
		goto out;

	/* it is not an error of hidden_file has no operation */
	mask = DEFAULT_POLLMASK;
	h_file = au_h_fptr(file, au_fbstart(file));
	if (h_file->f_op && h_file->f_op->poll)
		mask = h_file->f_op->poll(h_file, wait);
	di_read_unlock(dentry, AuLock_IR);
	fi_read_unlock(file);

 out:
	si_read_unlock(sb);
	AuTraceErr((int)mask);
	return mask;
}

static int aufs_fsync_nondir(struct file *file, struct dentry *dentry,
			     int datasync)
{
	int err;
	struct inode *inode;
	struct file *h_file;
	struct super_block *sb;
	struct au_pin pin;

	LKTRTrace("%.*s, %d\n", AuDLNPair(dentry), datasync);
	inode = dentry->d_inode;

	IMustLock(file->f_mapping->host);
	if (unlikely(inode != file->f_mapping->host)) {
		mutex_unlock(&file->f_mapping->host->i_mutex);
		mutex_lock(&inode->i_mutex);
	}
	IMustLock(inode);

	sb = dentry->d_sb;
	si_read_lock(sb, AuLock_FLUSH);
	err = 0; /* -EBADF; */ /* posix? */
	if (unlikely(!(file->f_mode & FMODE_WRITE)))
		goto out;
	err = au_reval_and_lock_fdi(file, au_reopen_nondir, /*wlock*/1,
				    /*locked*/1);
	if (unlikely(err))
		goto out;
	err = au_ready_to_write(file, -1, &pin);
	di_downgrade_lock(dentry, AuLock_IR);
	if (unlikely(err))
		goto out_unlock;
	au_unpin(&pin);

	err = -EINVAL;
	h_file = au_h_fptr(file, au_fbstart(file));
	if (h_file->f_op && h_file->f_op->fsync) {
		struct mutex *h_mtx = &h_file->f_dentry->d_inode->i_mutex;

		mutex_lock_nested(h_mtx, AuLsc_I_CHILD);
		err = h_file->f_op->fsync(h_file, h_file->f_dentry, datasync);
		if (!err)
			au_update_fuse_h_inode(h_file->f_vfsmnt,
					       h_file->f_dentry);
		au_cpup_attr_timesizes(inode);
		mutex_unlock(h_mtx);
	}

 out_unlock:
	di_read_unlock(dentry, AuLock_IR);
	fi_write_unlock(file);
 out:
	si_read_unlock(sb);
	if (unlikely(inode != file->f_mapping->host)) {
		mutex_unlock(&inode->i_mutex);
		mutex_lock(&file->f_mapping->host->i_mutex);
	}
	AuTraceErr(err);
	return err;
}

static int aufs_fasync(int fd, struct file *file, int flag)
{
	int err;
	struct file *h_file;
	struct dentry *dentry;
	struct super_block *sb;

	dentry = file->f_dentry;
	LKTRTrace("%.*s, %d\n", AuDLNPair(dentry), flag);

	sb = dentry->d_sb;
	si_read_lock(sb, AuLock_FLUSH);
	err = au_reval_and_lock_fdi(file, au_reopen_nondir, /*wlock*/0,
				    /*locked*/0);
	if (unlikely(err))
		goto out;

	h_file = au_h_fptr(file, au_fbstart(file));
	if (h_file->f_op && h_file->f_op->fasync)
		err = h_file->f_op->fasync(fd, h_file, flag);
	di_read_unlock(dentry, AuLock_IR);
	fi_read_unlock(file);

 out:
	si_read_unlock(sb);
	AuTraceErr(err);
	return err;
}

/* ---------------------------------------------------------------------- */

struct file_operations aufs_file_fop = {
	.read		= aufs_read,
	.write		= aufs_write,
	.poll		= aufs_poll,
	.mmap		= aufs_mmap,
	.open		= aufs_open_nondir,
	.flush		= aufs_flush,
	.release	= aufs_release_nondir,
	.fsync		= aufs_fsync_nondir,
	.fasync		= aufs_fasync,
#ifdef CONFIG_AUFS_SPLICE_PATCH
	.splice_write	= aufs_splice_write,
	.splice_read	= aufs_splice_read,
#endif
};
