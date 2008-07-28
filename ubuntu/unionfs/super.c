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
 *  $Id: super.c,v 1.101 2006/11/04 22:27:51 jsipek Exp $
 */

#include "unionfs.h"

/* The inode cache is used with alloc_inode for both our inode info and the
 * vfs inode.  */
static struct kmem_cache *unionfs_inode_cachep;

void unionfs_read_inode(struct inode *inode)
{
#ifdef UNIONFS_MMAP
	/* SP: use real address operations */
	extern struct address_space_operations unionfs_aops;
#else
	static struct address_space_operations unionfs_empty_aops;
#endif
	int size;

	print_entry_location();

	if (!itopd(inode)) {
		printk(KERN_ERR
		       "No kernel memory when allocating inode private data!\n");
		BUG();
	}

	memset(itopd(inode), 0, sizeof(struct unionfs_inode_info));
	itopd(inode)->b_start = -1;
	itopd(inode)->b_end = -1;
	atomic_set(&itopd(inode)->uii_generation,
		   atomic_read(&stopd(inode->i_sb)->usi_generation));
	itopd(inode)->uii_rdlock = SPIN_LOCK_UNLOCKED;
	itopd(inode)->uii_rdcount = 1;
	itopd(inode)->uii_hashsize = -1;
	INIT_LIST_HEAD(&itopd(inode)->uii_readdircache);

	size = sbmax(inode->i_sb) * sizeof(struct inode *);
	itohi_ptr(inode) = KZALLOC(size, GFP_KERNEL);
	if (!itohi_ptr(inode)) {
		printk(KERN_ERR
		       "No kernel memory when allocating lower-pointer array!\n");
		BUG();
	}

	inode->i_version++;
	inode->i_op = &unionfs_main_iops;
	inode->i_fop = &unionfs_main_fops;
#ifdef UNIONFS_MMAP
	inode->i_mapping->a_ops = &unionfs_aops;
#else
	/* I don't think ->a_ops is ever allowed to be NULL */
	inode->i_mapping->a_ops = &unionfs_empty_aops;
	dprint(PRINT_DEBUG, "setting inode 0x%p a_ops to empty (0x%p)\n",
	       inode, inode->i_mapping->a_ops);
#endif

	print_exit_location();
}

#if 0
static void unionfs_put_inode(struct inode *inode)
{
	print_entry_location();

	dprint(PRINT_DEBUG, "%s i_count = %d, i_nlink = %d\n", __FUNCTION__,
	       atomic_read(&inode->i_count), inode->i_nlink);

	/*
	 * This is really funky stuff:
	 * Basically, if i_count == 1, iput will then decrement it and this
	 * inode will be destroyed.  It is currently holding a reference to the
	 * hidden inode.  Therefore, it needs to release that reference by
	 * calling iput on the hidden inode.  iput() _will_ do it for us (by
	 * calling our clear_inode), but _only_ if i_nlink == 0.  The problem
	 * is, NFS keeps i_nlink == 1 for silly_rename'd files.  So we must for
	 * our i_nlink to 0 here to trick iput() into calling our clear_inode.
	 */

	if (atomic_read(&inode->i_count) == 1)
		inode->i_nlink = 0;

	print_exit_location();
}
#endif

/*
 * we now define delete_inode, because there are two VFS paths that may
 * destroy an inode: one of them calls clear inode before doing everything
 * else that's needed, and the other is fine.  This way we truncate the inode
 * size (and its pages) and then clear our own inode, which will do an iput
 * on our and the lower inode.
 */
static void unionfs_delete_inode(struct inode *inode)
{
	print_entry_location();

	checkinode(inode, "unionfs_delete_inode IN");
	inode->i_size = 0;	/* every f/s seems to do that */

#ifdef UNIONFS_MMAP
	/* SP: if you try to clear_inode() when
	 * inode->i_data.nrpages != 0, you'll hit a BUG
	 * this is also what generic_delete_inode does */
	if (inode->i_data.nrpages)
		truncate_inode_pages(&inode->i_data, 0);
#endif
	clear_inode(inode);

	print_exit_location();
}

/* final actions when unmounting a file system */
static void unionfs_put_super(struct super_block *sb)
{
	int bindex, bstart, bend;
	struct unionfs_sb_info *spd;

	print_entry_location();

	if ((spd = stopd(sb))) {
#ifdef UNIONFS_IMAP
		/* XXX: Free persistent inode stuff. */
		cleanup_imap_data(sb);
#endif
		bstart = sbstart(sb);
		bend = sbend(sb);
		for (bindex = bstart; bindex <= bend; bindex++)
			mntput(stohiddenmnt_index(sb, bindex));

		/* Make sure we have no leaks of branchget/branchput. */
		for (bindex = bstart; bindex <= bend; bindex++)
			BUG_ON(branch_count(sb, bindex) != 0);

		KFREE(spd->usi_data);
		KFREE(spd);
		stopd_lhs(sb) = NULL;
	}
	dprint(PRINT_DEBUG, "unionfs: released super\n");

	print_exit_location();
}

/* Since people use this to answer the "How big of a file can I write?"
 * question, we report the size of the highest priority branch as the size of
 * the union.
 */
static int unionfs_statfs(struct dentry *dentry, struct kstatfs *buf)
{
	int err	= 0;
	struct super_block *sb, *hidden_sb;

	sb = dentry->d_sb;

	hidden_sb = stohs_index(sb, sbstart(sb));
	err = vfs_statfs(hidden_sb->s_root, buf);

	buf->f_type = UNIONFS_SUPER_MAGIC;
	buf->f_namelen -= WHLEN;

	memset(&buf->f_fsid, 0, sizeof(__kernel_fsid_t));
	memset(&buf->f_spare, 0, sizeof(buf->f_spare));

	return err;
}

static int do_binary_remount(struct super_block *sb, int *flags, char *data)
{
	unsigned long *uldata = (unsigned long *)data;
	int err;

	uldata++;

	switch (*uldata) {
	case UNIONFS_IOCTL_DELBRANCH:
		err = unionfs_ioctl_delbranch(sb, *(uldata + 1));
		break;
	default:
		err = -ENOTTY;
	}

	return err;
}

/* We don't support a standard text remount, but we do have a magic remount
 * for unionctl.  The idea is that you can remove a branch without opening
 * the union.  Eventually it would be nice to support a full-on remount, so
 * that you can have all of the directories change at once, but that would
 * require some pretty complicated matching code. */
static int unionfs_remount_fs(struct super_block *sb, int *flags, char *data)
{
	if (data && *((unsigned long *)data) == UNIONFS_REMOUNT_MAGIC)
		return do_binary_remount(sb, flags, data);
	printk("Warning! dirs delete and imap options to remount are ignored\n");
	return 0;
}

/*
 * Called by iput() when the inode reference count reached zero
 * and the inode is not hashed anywhere.  Used to clear anything
 * that needs to be, before the inode is completely destroyed and put
 * on the inode free list.
 */
static void unionfs_clear_inode(struct inode *inode)
{
	int bindex, bstart, bend;
	struct inode *hidden_inode;
	struct list_head *pos, *n;
	struct unionfs_dir_state *rdstate;

	print_entry_location();

	checkinode(inode, "unionfs_clear_inode IN");

	list_for_each_safe(pos, n, &itopd(inode)->uii_readdircache) {
		rdstate = list_entry(pos, struct unionfs_dir_state, uds_cache);
		list_del(&rdstate->uds_cache);
		free_rdstate(rdstate);
	}

	/* Decrement a reference to a hidden_inode, which was incremented
	 * by our read_inode when it was created initially.  */
	bstart = ibstart(inode);
	bend = ibend(inode);
	if (bstart >= 0) {
		for (bindex = bstart; bindex <= bend; bindex++) {
			hidden_inode = itohi_index(inode, bindex);
			if (!hidden_inode)
				continue;
			IPUT(hidden_inode);
		}
	}
	// XXX: why this assertion fails?
	// because it doesn't like us
	// BUG_ON((inode->i_state & I_DIRTY) != 0);
	KFREE(itohi_ptr(inode));
	itohi_ptr(inode) = NULL;

	print_exit_location();
}

static struct inode *unionfs_alloc_inode(struct super_block *sb)
{
	struct unionfs_inode_container *c;

	print_entry_location();

	c = (struct unionfs_inode_container *)
	    kmem_cache_alloc(unionfs_inode_cachep, GFP_KERNEL);
	if (!c) {
		print_exit_pointer(NULL);
		return NULL;
	}

	memset(&c->info, 0, sizeof(c->info));

	c->vfs_inode.i_version = 1;
	print_exit_pointer(&c->vfs_inode);
	return &c->vfs_inode;
}

static void unionfs_destroy_inode(struct inode *inode)
{
	print_entry("inode = %p", inode);
	kmem_cache_free(unionfs_inode_cachep, itopd(inode));
	print_exit_location();
}

static void init_once(void *v)
{
	struct unionfs_inode_container *c = (struct unionfs_inode_container *)v;

	print_entry_location();

	inode_init_once(&c->vfs_inode);

	print_exit_location();
}

int init_inode_cache(void)
{
	int err = 0;

	print_entry_location();

	unionfs_inode_cachep =
	    kmem_cache_create("unionfs_inode_cache",
			      sizeof(struct unionfs_inode_container), 0,
			      SLAB_RECLAIM_ACCOUNT, init_once);
	if (!unionfs_inode_cachep)
		err = -ENOMEM;
	print_exit_status(err);
	return err;
}

void destroy_inode_cache(void)
{
	print_entry_location();
	if (!unionfs_inode_cachep)
		goto out;
	kmem_cache_destroy(unionfs_inode_cachep);
      out:
	print_exit_location();
	return;
}

/* Called when we have a dirty inode, right here we only throw out
 * parts of our readdir list that are too old.
 */
static int unionfs_write_inode(struct inode *inode, int sync)
{
	struct list_head *pos, *n;
	struct unionfs_dir_state *rdstate;

	print_entry_location();

	spin_lock(&itopd(inode)->uii_rdlock);
	list_for_each_safe(pos, n, &itopd(inode)->uii_readdircache) {
		rdstate = list_entry(pos, struct unionfs_dir_state, uds_cache);
		/* We keep this list in LRU order. */
		if ((rdstate->uds_access + RDCACHE_JIFFIES) > jiffies)
			break;
		itopd(inode)->uii_rdcount--;
		list_del(&rdstate->uds_cache);
		free_rdstate(rdstate);
	}
	spin_unlock(&itopd(inode)->uii_rdlock);

	print_exit_location();
	return 0;
}

/*
 * Used only in nfs, to kill any pending RPC tasks, so that subsequent
 * code can actually succeed and won't leave tasks that need handling.
 *
 * PS. I wonder if this is somehow useful to undo damage that was
 * left in the kernel after a user level file server (such as amd)
 * dies.
 */
static void unionfs_umount_begin(struct super_block *sb)
{
	struct super_block *hidden_sb;
	int bindex, bstart, bend;

	print_entry_location();
#if 0
	if (!(flags & MNT_FORCE))
		/* we are not being MNT_FORCEd, therefore we should emulate old
		 * behaviour
		 */
		goto out;
#endif
	bstart = sbstart(sb);
	bend = sbend(sb);
	for (bindex = bstart; bindex <= bend; bindex++) {
		hidden_sb  = stohs_index(sb, bindex);

		if (hidden_sb && hidden_sb->s_op &&
		    hidden_sb->s_op->umount_begin)
			hidden_sb->s_op->umount_begin(hidden_sb);
	}

	print_exit_location();
}

static int unionfs_show_options(struct seq_file *m, struct vfsmount *mnt)
{
	struct super_block *sb = mnt->mnt_sb;
	int ret = 0;
	unsigned long tmp = 0;
	char *hidden_path;
	int bindex, bstart, bend;
	int perms;

	lock_dentry(sb->s_root);

	tmp = __get_free_page(GFP_KERNEL);
	if (!tmp) {
		ret = -ENOMEM;
		goto out;
	}

	bindex = bstart = sbstart(sb);
	bend = sbend(sb);

	seq_printf(m, ",dirs=");
	for (bindex = bstart; bindex <= bend; bindex++) {
		struct path tp;
		tp.dentry = dtohd_index(sb->s_root, bindex);
		tp.mnt = stohiddenmnt_index(sb, bindex);

		hidden_path =
		    d_path(&tp, (char *)tmp, PAGE_SIZE);
		perms = branchperms(sb, bindex);
		seq_printf(m, "%s=%s", hidden_path,
			   perms & MAY_WRITE ? "rw" :
			   perms & MAY_NFSRO ? "nfsro" : "ro");
		if (bindex != bend) {
			seq_printf(m, ":");
		}
	}

	seq_printf(m, ",debug=%u", get_debug_mask());

#ifdef UNIONFS_DELETE_ALL
	if (IS_SET(sb, DELETE_ALL))
		seq_printf(m, ",delete=all");
	else
#endif
		seq_printf(m, ",delete=whiteout");
      out:
	if (tmp)
		free_page(tmp);
	unlock_dentry(sb->s_root);
	return ret;
}

#ifdef CONFIG_EXPORTFS
/*
 * export operations.
 * unionfs cannot handle disconnected dentry, since it has no hidden dentries.
 */
/* un-tested 64 bit environment (pointer and inode number) */

#define is_anon(d) ((d)->d_flags & DCACHE_DISCONNECTED)
extern struct export_operations export_op_default;

static void prepend_path(char **path, const char *name, int len)
{
	*path -= len;
	memcpy(*path, name, len);
	(*path)--;
	**path = '/';
}

struct filldir_arg {
	int found, called;
	char *path;
	ino_t ino, parent_ino;
};

static int filldir(void *arg, const char *name, int len, loff_t pos, ino_t ino,
		   unsigned int d_type)
{
	struct filldir_arg *a = arg;

	a->called++;
	if (len == 2 && !strncmp(name, "..", 2)) {
		a->parent_ino = ino;
		a->found++;
	} else if (ino == a->ino) {
		if (len != 1 || *name != '.')
			prepend_path(&a->path, name, len);
		a->found++;
	}
	return (a->found == 2) ? 1 : 0;
}

static struct dentry *get_hidden_parent(struct super_block *hidden_sb,
					ino_t hidden_parent_ino)
{
	__u32 fh[2];

	if (hidden_sb->s_root->d_inode->i_ino == hidden_parent_ino)
		return DGET(hidden_sb->s_root);

	fh[0] = hidden_parent_ino;
	fh[1] = 0;
	return export_op_default.get_dentry(hidden_sb, fh);
}

static struct dentry *do_get_dentry(struct super_block *sb, ino_t ino,
				    __u32 gen, struct dentry *hidden_root,
				    ino_t hidden_ino, ino_t hidden_parent_ino)
{
	struct dentry *dentry, *hidden_parent, *parent;
	char *path, *p;
	struct filldir_arg arg = {
		.ino = hidden_ino,
		.parent_ino = hidden_parent_ino
	};
	int open_flags, err, bindex, bend, found;
	struct file *hidden_file;
	struct super_block *hidden_sb;

	print_entry("hr%p, hi%lu, hpi%lu",
		    hidden_root, hidden_ino, hidden_parent_ino);

	dentry = ERR_PTR(-ENOMEM);
	path = __getname();
	if (!path)
		goto out;
	arg.path = path + PATH_MAX - 1;
	*arg.path = 0;

	open_flags = O_RDONLY | O_DIRECTORY /* | O_NOATIME */ ;
	if (force_o_largefile())
		open_flags |= O_LARGEFILE;

	dentry = ERR_PTR(-ESTALE);
	unionfs_read_lock(sb);
	lock_dentry(sb->s_root);
	bend = dbend(sb->s_root);
	found = -1;
	for (bindex = 0; found == -1 && bindex <= bend; bindex++)
		if (hidden_root == dtohd_index(sb->s_root, bindex))
			found = bindex;
	unlock_dentry(sb->s_root);
	if (found == -1)
		goto out_unlock;

	bindex = found;
	hidden_sb = stohs_index(sb, bindex);
	while (1) {
		hidden_parent = get_hidden_parent(hidden_sb, hidden_parent_ino);
		dentry = hidden_parent;
		if (IS_ERR(hidden_parent))
			goto out_unlock;

		branchget(sb, bindex);
		hidden_file = DENTRY_OPEN(DGET(hidden_parent), NULL,
					  open_flags);
		if (IS_ERR(hidden_file)) {
			dentry = (void *)hidden_file;
			DPUT(hidden_parent);
			branchput(sb, bindex);
			goto out_unlock;
		}

		arg.found = 0;
		while (arg.found != 2) {
			arg.called = 0;
			err = vfs_readdir(hidden_file, filldir, &arg);
			if (!arg.called || err < 0)
				break;
		}
		fput(hidden_file);
		branchput(sb, bindex);
		if (arg.found != 2) {
			dentry = ERR_PTR(-ESTALE);
			DPUT(hidden_parent);
			goto out_unlock;
		}

		DPUT(hidden_parent);
		if (hidden_parent_ino == hidden_root->d_inode->i_ino)
			break;
		arg.ino = hidden_parent_ino;
		hidden_parent_ino = arg.parent_ino;
	}
	BUG_ON(arg.path < path);

	parent = DGET(sb->s_root);
	p = strchr(++arg.path, '/');
	while (p) {
		mutex_lock(&parent->d_inode->i_mutex);
		dentry = LOOKUP_ONE_LEN(arg.path, parent, p - arg.path);
		mutex_unlock(&parent->d_inode->i_mutex);
		DPUT(parent);
		if (IS_ERR(dentry))
			goto out_unlock;
		if (!dentry->d_inode || !S_ISDIR(dentry->d_inode->i_mode)) {
			DPUT(dentry);
			dentry = ERR_PTR(-ESTALE);
			goto out_unlock;
		}
		parent = dentry;
		arg.path = p + 1;
		p = strchr(arg.path, '/');
	}
	mutex_lock(&parent->d_inode->i_mutex);
	dentry = LOOKUP_ONE_LEN(arg.path, parent, strlen(arg.path));
	mutex_unlock(&parent->d_inode->i_mutex);
	DPUT(parent);
	if (!IS_ERR(dentry)
	    && (!dentry->d_inode
		|| dentry->d_inode->i_ino != ino
		|| dentry->d_inode->i_generation != gen)) {
		DPUT(dentry);
		dentry = ERR_PTR(-ESTALE);
	}

      out_unlock:
	unionfs_read_unlock(sb);
	__putname(path);
      out:
	print_exit_pointer(dentry);
	return dentry;
}

enum {
	FhHead = 4, FhHRoot1 = FhHead, FhHRoot2,
	FhHIno1, FhHIno2, FhHPIno1, FhHPIno2,
	FhTail
};

static void do_decode(__u32 * fh, struct dentry **hidden_root,
		      ino_t * hidden_ino, ino_t * hidden_parent_ino)
{
	unsigned long root;
	
	root = fh[FhHRoot2];
	*hidden_ino = fh[FhHIno2];
	*hidden_parent_ino = fh[FhHPIno2];
#if BITS_PER_LONG == 64
	root |= ((unsigned long)fh[FhHRoot1]) << 32;
	*hidden_ino |= ((unsigned long) fh[FhHIno1]) << 32;
	*hidden_parent_ino |= ((unsigned long) fh[FhHPIno1]) << 32;
#elif BITS_PER_LONG == 32
	/* ok */
#else
#error unknown size
#endif

	*hidden_root = (struct dentry*) root;
}

static int unionfs_encode_fh(struct dentry *dentry, __u32 * fh, int *max_len,
			     int connectable)
{
	int type, len, bindex;
	struct super_block *sb;
	struct dentry *h_root;
	ino_t h_ino, hp_ino;
	static int warn;

	print_entry("dentry %p", dentry);
	BUG_ON(is_anon(dentry) || !dentry->d_inode
	       || is_anon(dentry->d_parent));

#ifdef UNIONFS_IMAP
	if (!warn && stopd(dentry->d_sb)->usi_persistent)
		warn++;
#endif
	if (!warn) {
		printk(KERN_WARNING "Exporting Unionfs without imap"
		       " option may stop your NFS server or client");
		warn++;
	}

	sb = dentry->d_sb;
	unionfs_read_lock(sb);
	lock_dentry(dentry);

	len = *max_len;
	type = export_op_default.encode_fh(dentry, fh, max_len, connectable);
	if (type == 255 || *max_len > FhHead || len < FhTail) {
		type = 255;
		goto out;
	}

	*max_len = FhTail;
	bindex = dbstart(dentry);
	lock_dentry(sb->s_root);
	h_root = dtohd_index(sb->s_root, bindex);
	unlock_dentry(sb->s_root);
	h_ino = itohi_index(dentry->d_inode, bindex)->i_ino;
	hp_ino = parent_ino(dtohd(dentry));
	fh[FhHRoot2] = (unsigned long) h_root;
	fh[FhHIno2] = h_ino;
	fh[FhHPIno2] = hp_ino;
#if BITS_PER_LONG == 64
	fh[FhHRoot1] = ((unsigned long) h_root) >> 32;
	fh[FhHIno1] = h_ino >> 32;
	fh[FhHPIno1] = hp_ino >> 32;
#endif

      out:
	unionfs_print(PRINT_MAIN_EXIT, "%d, fh{i%u, g%d, hr%x, hi%u, hpi%u}\n",
		      type, fh[0], fh[1], fh[FhHRoot2], fh[FhHIno2],
		      fh[FhHPIno2]);
	unlock_dentry(dentry);
	unionfs_read_unlock(sb);
	return type;
}

static struct dentry *unionfs_decode_fh(struct super_block *sb, __u32 * fh,
					int fh_len, int fh_type,
					int (*acceptable) (void *context,
							   struct dentry * de),
					void *context)
{
	struct dentry *dentry, *hidden_root;
	ino_t hidden_ino, hidden_parent_ino;

	print_entry("%d, fh{i%u, g%d, hr%x, hi%u, hpi%u}",
		    fh_type, fh[0], fh[1], fh[FhHRoot2], fh[FhHIno2],
		    fh[FhHPIno2]);

	dentry = export_op_default.get_dentry(sb, fh);
	if (!dentry || IS_ERR(dentry) || (dentry->d_inode && !is_anon(dentry)))
		return dentry;

	d_drop(dentry);
	DPUT(dentry);
	do_decode(fh, &hidden_root, &hidden_ino, &hidden_parent_ino);
	dentry = do_get_dentry(sb, fh[0], fh[1], hidden_root, hidden_ino,
			       hidden_parent_ino);
	if (!IS_ERR(dentry)) {
		if (acceptable(context, dentry))
			return dentry;	/* success */
		DPUT(dentry);
		dentry = NULL;
	}
	return dentry;
}

struct export_operations unionfs_export_ops = {
	.decode_fh = unionfs_decode_fh,
	.encode_fh = unionfs_encode_fh
};
#endif

struct super_operations unionfs_sops = {
	//.put_inode = unionfs_put_inode,
	.delete_inode = unionfs_delete_inode,
	.put_super = unionfs_put_super,
	.statfs = unionfs_statfs,
	.remount_fs = unionfs_remount_fs,
	.clear_inode = unionfs_clear_inode,
	.umount_begin = unionfs_umount_begin,
	.show_options = unionfs_show_options,
	.write_inode = unionfs_write_inode,
	.alloc_inode = unionfs_alloc_inode,
	.destroy_inode = unionfs_destroy_inode,
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
