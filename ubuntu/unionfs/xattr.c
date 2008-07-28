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
 *  $Id: xattr.c,v 1.32 2006/06/01 03:11:03 jsipek Exp $
 */

#include "unionfs.h"

/* This is lifted from fs/xattr.c */
void *xattr_alloc(size_t size, size_t limit)
{
	void *ptr;

	if (size > limit)
		return ERR_PTR(-E2BIG);

	if (!size)		/* size request, no buffer is needed */
		return NULL;
	else if (size <= PAGE_SIZE)
		ptr = KMALLOC((unsigned long)size, GFP_KERNEL);
	else
		ptr = vmalloc((unsigned long)size);
	if (!ptr)
		return ERR_PTR(-ENOMEM);
	return ptr;
}

void xattr_free(void *ptr, size_t size)
{
	if (!size)		/* size request, no buffer was needed */
		return;
	else if (size <= PAGE_SIZE)
		KFREE(ptr);
	else
		vfree(ptr);
}

/* BKL held by caller.
 * dentry->d_inode->i_mutex locked
 * ssize_t (*getxattr) (struct dentry *, const char *, void *, size_t);
 */
ssize_t unionfs_getxattr(struct dentry * dentry, const char *name, void *value,
			 size_t size)
{
	struct dentry *hidden_dentry = NULL;
	int err = -EOPNOTSUPP;

	print_entry_location();

	dprint(PRINT_DEBUG_XATTR, "getxattr: name=\"%s\", value %lu bytes\n",
			name, size);

	lock_dentry(dentry);

	hidden_dentry = dtohd(dentry);

	err = vfs_getxattr(hidden_dentry, NULL, (char*)name, value, size,
			   NULL);

	unlock_dentry(dentry);
	print_exit_status(err);
	return err;
}

/* BKL held by caller.
 * dentry->d_inode->i_mutex locked
 */
int
unionfs_setxattr(struct dentry *dentry, const char *name, const void *value,
		 size_t size, int flags)
{
	struct dentry *hidden_dentry = NULL;
	int err = -EOPNOTSUPP;

	print_entry_location();

	lock_dentry(dentry);
	hidden_dentry = dtohd(dentry);

	dprint(PRINT_DEBUG_XATTR, "setxattr: name=\"%s\", value %lu bytes,"
			"flags=%x\n", name, (unsigned long)size, flags);

	err =
	    vfs_setxattr(hidden_dentry, NULL, (char *)name, (char *)value,
			 size, flags, NULL);

	unlock_dentry(dentry);
	print_exit_status(err);
	return err;
}

/* BKL held by caller.
 * dentry->d_inode->i_mutex locked
 */
int unionfs_removexattr(struct dentry *dentry, const char *name)
{
	struct dentry *hidden_dentry = NULL;
	int err = -EOPNOTSUPP;

	print_entry_location();

	lock_dentry(dentry);
	hidden_dentry = dtohd(dentry);

	dprint(PRINT_DEBUG_XATTR, "removexattr: name=\"%s\"\n", name);

	err = vfs_removexattr(hidden_dentry, NULL, (char*)name, NULL);

	unlock_dentry(dentry);
	print_exit_status(err);
	return err;
}

/* BKL held by caller.
 * dentry->d_inode->i_mutex locked
 */
ssize_t unionfs_listxattr(struct dentry * dentry, char *list, size_t size)
{
	struct dentry *hidden_dentry = NULL;
	int err = -EOPNOTSUPP;
	char *encoded_list = NULL;

	print_entry_location();
	lock_dentry(dentry);

	hidden_dentry = dtohd(dentry);

	encoded_list = list;
	err = vfs_listxattr(hidden_dentry, NULL, encoded_list, size, NULL);

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
