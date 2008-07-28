#ifndef __UNIONFS_H_
#define __UNIONFS_H_

#ifdef __KERNEL__

#include <linux/version.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/string.h>
#include <linux/stat.h>
#include <linux/errno.h>
#include <linux/wait.h>
#include <linux/limits.h>
#include <linux/random.h>
#include <linux/poll.h>
#include <linux/buffer_head.h>
#include <linux/pagemap.h>
#include <linux/namei.h>
#include <linux/module.h>
#include <linux/mount.h>
#include <linux/page-flags.h>
#include <linux/writeback.h>
#include <linux/page-flags.h>
#include <linux/statfs.h>
#include <linux/smp.h>
#include <linux/smp_lock.h>
#include <linux/file.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/poll.h>
#include <linux/list.h>
#include <linux/init.h>
#include <linux/xattr.h>
#include <linux/security.h>
#include <linux/spinlock.h>
#include <linux/compat.h>

#include <linux/swap.h>

#include <asm/system.h>
#include <asm/mman.h>
#include <linux/seq_file.h>
#include <linux/dcache.h>
#include <linux/poll.h>

#ifndef UNIONFS_UNSUPPORTED
#if LINUX_VERSION_CODE != KERNEL_VERSION(SUP_MAJOR,SUP_MINOR,SUP_PATCH)
#warning You are compiling Unionfs on an unsupported kernel version.
#warning To compile Unionfs, you will need to define UNIONFS_UNSUPPORTED.
#warning Try adding: EXTRACFLAGS=-DUNIONFS_UNSUPPORTED to fistdev.mk
#endif
#endif

/* the file system name */
#define UNIONFS_NAME "unionfs"

/* unionfs file systems superblock magic */
#define UNIONFS_SUPER_MAGIC 0xf15f083d

/* unionfs root inode number */
#define UNIONFS_ROOT_INO     1

/* Mount time flags */
#define MOUNT_FLAG(sb)     (stopd(sb)->usi_mount_flag)

/* number of characters while generating unique temporary file names */
#define	UNIONFS_TMPNAM_LEN	12

/* Operations vectors defined in specific files. */
extern struct file_operations unionfs_main_fops;
extern struct file_operations unionfs_dir_fops;
extern struct inode_operations unionfs_main_iops;
extern struct inode_operations unionfs_dir_iops;
extern struct inode_operations unionfs_symlink_iops;
extern struct super_operations unionfs_sops;
extern struct dentry_operations unionfs_dops;
#ifdef CONFIG_EXPORTFS
extern struct export_operations unionfs_export_ops;
#endif

/* How long should an entry be allowed to persist */
#define RDCACHE_JIFFIES 5*HZ

/* file private data. */
struct unionfs_file_info {
	int b_start;
	int b_end;
	atomic_t ufi_generation;

	struct unionfs_dir_state *rdstate;
	struct file **ufi_file;
};

/* unionfs inode data in memory */
struct unionfs_inode_info {
	int b_start;
	int b_end;
	atomic_t uii_generation;
	int uii_stale;
	/* Stuff for readdir over NFS. */
	spinlock_t uii_rdlock;
	struct list_head uii_readdircache;
	int uii_rdcount;
	int uii_hashsize;
	int uii_cookie;
	/* The hidden inodes */
	struct inode **uii_inode;
	/* to keep track of reads/writes for unlinks before closes */
	atomic_t uii_totalopens;
};

struct unionfs_inode_container {
	struct unionfs_inode_info info;
	struct inode vfs_inode;
};

/* unionfs dentry data in memory */
struct unionfs_dentry_info {
	/* The semaphore is used to lock the dentry as soon as we get into a
	 * unionfs function from the VFS.  Our lock ordering is that children
	 * go before their parents. */
	struct semaphore udi_sem;
	int udi_bstart;
	int udi_bend;
	int udi_bopaque;
	int udi_bcount;
	atomic_t udi_generation;
	struct dentry **udi_dentry;
};

/* A putmap is used so that older files can still do branchput correctly. */
struct putmap {
	atomic_t count;
	int bend;
	int map[0];
};

/* These are the pointers to our various objects. */
struct unionfs_usi_data {
	struct super_block *sb;
	struct vfsmount *hidden_mnt;
	atomic_t sbcount;
	int branchperms;
};

/* unionfs super-block data in memory */
struct unionfs_sb_info {
	int b_end;

	atomic_t usi_generation;
	unsigned long usi_mount_flag;
	struct rw_semaphore usi_rwsem;

	struct unionfs_usi_data *usi_data;

	/* These map branch numbers for old generation numbers to the new bindex,
	 * so that branchput will behave properly. */
	int usi_firstputmap;
	int usi_lastputmap;
	struct putmap **usi_putmaps;

#ifdef UNIONFS_IMAP
	int usi_persistent;
	/* These will need a lock. */
	uint64_t usi_next_avail;
	uint8_t usi_num_bmapents;
	struct bmapent *usi_bmap;
	struct file *usi_forwardmap;
	struct file **usi_reversemaps;
	struct file **usi_map_table;
	int *usi_bnum_table;	//This is a table of branches to fsnums.
#endif				/* UNIONFS_IMAP */
};

/*
 * structure for making the linked list of entries by readdir on left branch
 * to compare with entries on right branch
 */
struct filldir_node {
	struct list_head file_list;	// list for directory entries
	char *name;		// name entry
	int hash;		// name hash
	int namelen;		// name len since name is not 0 terminated
	int bindex;		// we can check for duplicate whiteouts and files in the same branch in order to return -EIO.
	int whiteout;		// is this a whiteout entry?
	char iname[DNAME_INLINE_LEN_MIN];	// Inline name, so we don't need to separately kmalloc small ones
};

/* Directory hash table. */
struct unionfs_dir_state {
	unsigned int uds_cookie;	/* The cookie, which is based off of uii_rdversion */
	unsigned int uds_offset;	/* The entry we have returned. */
	int uds_bindex;
	loff_t uds_dirpos;	/* The offset within the lower level directory. */
	int uds_size;		/* How big is the hash table? */
	int uds_hashentries;	/* How many entries have been inserted? */
	unsigned long uds_access;
	/* This cache list is used when the inode keeps us around. */
	struct list_head uds_cache;
	struct list_head uds_list[0];
};

/* privileged io workqueue */
#include "sioq.h"

/* include miscellaneous macros */
#include "unionfs_macros.h"

/* include debug macros */
#include "unionfs_debug.h"

/* include persistent imap code */
#include "unionfs_imap.h"

/* Cache creation/deletion routines. */
void destroy_filldir_cache(void);
int init_filldir_cache(void);
int init_inode_cache(void);
void destroy_inode_cache(void);
int init_dentry_cache(void);
void destroy_dentry_cache(void);

/* Initialize and free readdir-specific  state. */
int init_rdstate(struct file *file);
struct unionfs_dir_state *alloc_rdstate(struct inode *inode, int bindex);
struct unionfs_dir_state *find_rdstate(struct inode *inode, loff_t fpos);
void free_rdstate(struct unionfs_dir_state *state);
int add_filldir_node(struct unionfs_dir_state *rdstate, const char *name,
		     int namelen, int bindex, int whiteout);
struct filldir_node *find_filldir_node(struct unionfs_dir_state *rdstate,
				       const char *name, int namelen);

struct dentry **alloc_new_dentries(int objs);
struct unionfs_usi_data *alloc_new_data(int objs);

#ifdef FIST_MALLOC_DEBUG

extern void *unionfs_kzalloc(size_t size, gfp_t flags, int line,
			     const char *file);
extern void *unionfs_kmalloc(size_t size, gfp_t flags, int line,
			     const char *file);
extern void unionfs_kfree(void *ptr, int line, const char *file);

extern struct dentry *unionfs_dget_parent(struct dentry *child, int line,
					  const char *file);
extern struct dentry *unionfs_dget(struct dentry *ptr, int line,
				   const char *file);
extern void unionfs_dput(struct dentry *ptr, int line, const char *file);
extern struct inode *unionfs_igrab(struct inode *inode, int line, char *file);
extern void unionfs_iput(struct inode *inode, int line, char *file);
extern struct dentry *unionfs_lookup_one_len(const char *name,
					     struct dentry *parent, int len,
					     int line, const char *file);
void record_path_lookup(struct nameidata *nd, int line, const char *file);
void record_path_release(struct nameidata *nd, int line, const char *file);
struct file *unionfs_dentry_open(struct dentry *ptr, struct vfsmount *mnt,
				 int flags, int line, const char *file);
void record_set(struct dentry *upper, int index, struct dentry *ptr,
		struct dentry *old, int line, const char *file);

#define KZALLOC(size,flags) unionfs_kzalloc((size),(flags),__LINE__,__FILE__)
#define KMALLOC(size,flags) unionfs_kmalloc((size),(flags),__LINE__,__FILE__)
#define KFREE(ptr) unionfs_kfree((ptr),__LINE__,__FILE__)
#define DGET(d) unionfs_dget((d),__LINE__,__FILE__)
#define DPUT(d) unionfs_dput((d),__LINE__,__FILE__)
# define IPUT(a)		unionfs_iput((a),__LINE__,__FILE__)
# define IGRAB(a)		unionfs_igrab((a),__LINE__,__FILE__)
#define LOOKUP_ONE_LEN(name,parent,len) unionfs_lookup_one_len((name),(parent),(len),__LINE__,__FILE__)
# define RECORD_PATH_LOOKUP(nd)	record_path_lookup((nd),__LINE__,__FILE__)
# define RECORD_PATH_RELEASE(nd) record_path_release((nd),__LINE__,__FILE__)
/* This has the effect of reducing the reference count sooner or later,
 * if the file is closed.  If it isn't then the mount will be busy and
 * you can't unmount.
 */
# define DENTRY_OPEN(d,m,f) unionfs_dentry_open((d),(m),(f),__LINE__,__FILE__)
# define GET_PARENT(dentry) unionfs_dget_parent((dentry),__LINE__,__FILE__)
#else				/* not FIST_MALLOC_DEBUG */
# define KZALLOC(a,b)		kzalloc((a),(b))
# define KMALLOC(a,b)		kmalloc((a),(b))
# define KFREE(a)		kfree((a))
# define DPUT(a)		dput((a))
# define DGET(a)		dget((a))
# define IPUT(a)		iput((a))
# define IGRAB(a)		igrab((a))
# define LOOKUP_ONE_LEN(a,b,c)	lookup_one_len((a),(b),(c))
# define RECORD_PATH_LOOKUP(a)
# define RECORD_PATH_RELEASE(a)
# define DENTRY_OPEN(d,m,f)	dentry_open((d),(m),(f))
# define GET_PARENT(d)		dget_parent(d)
#endif				/* not FIST_MALLOC_DEBUG */

/* We can only use 32-bits of offset for rdstate --- blech! */
#define DIREOF (0xfffff)
#define RDOFFBITS 20		/* This is the number of bits in DIREOF. */
#define MAXRDCOOKIE (0xfff)
/* Turn an rdstate into an offset. */
static inline off_t rdstate2offset(struct unionfs_dir_state *buf)
{
	off_t tmp;
	tmp =
	    ((buf->uds_cookie & MAXRDCOOKIE) << RDOFFBITS) | (buf->
							      uds_offset &
							      DIREOF);
	return tmp;
}

#define unionfs_read_lock(sb) down_read(&stopd(sb)->usi_rwsem)
#define unionfs_read_unlock(sb) up_read(&stopd(sb)->usi_rwsem)
#define unionfs_write_lock(sb) down_write(&stopd(sb)->usi_rwsem)
#define unionfs_write_unlock(sb) up_write(&stopd(sb)->usi_rwsem)

/* The double lock function needs to go after the debugmacros, so that
 * dtopd is defined.  */
static inline void double_lock_dentry(struct dentry *d1, struct dentry *d2)
{
	if (d2 < d1) {
		struct dentry *tmp = d1;
		d1 = d2;
		d2 = tmp;
	}
	lock_dentry(d1);
	lock_dentry(d2);
}

extern int new_dentry_private_data(struct dentry *dentry);
void free_dentry_private_data(struct unionfs_dentry_info *udi);
void update_bstart(struct dentry *dentry);
#define sbt(sb) ((sb)->s_type->name)

/*
 * EXTERNALS:
 */
/* replicates the directory structure upto given dentry in given branch */
extern struct dentry *create_parents(struct inode *dir, struct dentry *dentry,
				     int bindex);
struct dentry *create_parents_named(struct inode *dir, struct dentry *dentry,
				    const char *name, int bindex);

/* check if two branches overlap */
extern int is_branch_overlap(struct dentry *dent1, struct dentry *dent2);

/* partial lookup */
extern int unionfs_partial_lookup(struct dentry *dentry);

/* Pass an unionfs dentry and an index and it will try to create a whiteout in branch 'index'.
   On error, it will proceed to a branch to the left */
extern int create_whiteout(struct dentry *dentry, int start);
/* copies a file from dbstart to newbindex branch */
extern int copyup_file(struct inode *dir, struct file *file, int bstart,
		       int newbindex, loff_t size);
extern int copyup_named_file(struct inode *dir, struct file *file,
			     char *name, int bstart, int new_bindex,
			     loff_t len);

/* copies a dentry from dbstart to newbindex branch */
extern int copyup_dentry(struct inode *dir, struct dentry *dentry, int bstart,
			 int new_bindex, struct file **copyup_file, loff_t len);
extern int copyup_named_dentry(struct inode *dir, struct dentry *dentry,
			       int bstart, int new_bindex, const char *name,
			       int namelen, struct file **copyup_file,
			       loff_t len);

extern int remove_whiteouts(struct dentry *dentry, struct dentry *hidden_dentry,
			    int bindex);

/* Is this directory empty: 0 if it is empty, -ENOTEMPTY if not. */
extern int check_empty(struct dentry *dentry,
		       struct unionfs_dir_state **namelist);
/* Delete whiteouts from this directory in branch bindex. */
extern int delete_whiteouts(struct dentry *dentry, int bindex,
			    struct unionfs_dir_state *namelist);

/* Re-lookup a hidden dentry. */
extern int unionfs_refresh_hidden_dentry(struct dentry *dentry, int bindex);

extern void unionfs_reinterpose(struct dentry *this_dentry);
extern struct super_block *unionfs_duplicate_super(struct super_block *sb);

/* Locking functions. */
extern int unionfs_setlk(struct file *file, int cmd, struct file_lock *fl);
extern int unionfs_getlk(struct file *file, struct file_lock *fl);

/* Common file operations. */
extern int unionfs_file_revalidate(struct file *file, int willwrite);
extern int unionfs_open(struct inode *inode, struct file *file);
extern int unionfs_file_release(struct inode *inode, struct file *file);
extern int unionfs_flush(struct file *file, fl_owner_t id);
extern long unionfs_ioctl(struct file *file, unsigned int cmd,
			  unsigned long arg);

/* Inode operations */
extern int unionfs_rename(struct inode *old_dir, struct dentry *old_dentry,
			  struct inode *new_dir, struct dentry *new_dentry);
int unionfs_unlink(struct inode *dir, struct dentry *dentry);
int unionfs_rmdir(struct inode *dir, struct dentry *dentry);

int unionfs_d_revalidate(struct dentry *dentry, struct nameidata *nd);

/* The values for unionfs_interpose's flag. */
#define INTERPOSE_DEFAULT	0
#define INTERPOSE_LOOKUP	1
#define INTERPOSE_REVAL		2
#define INTERPOSE_REVAL_NEG	3
#define INTERPOSE_PARTIAL	4

extern int unionfs_interpose(struct dentry *this_dentry, struct super_block *sb,
			     int flag);

/* Branch management ioctls. */
int unionfs_ioctl_branchcount(struct file *file, unsigned int cmd,
			      unsigned long arg);
int unionfs_ioctl_incgen(struct file *file, unsigned int cmd,
			 unsigned long arg);
int unionfs_ioctl_addbranch(struct inode *inode, unsigned int cmd,
			    unsigned long arg);
int unionfs_ioctl_delbranch(struct super_block *sb, unsigned long arg);
int unionfs_ioctl_rdwrbranch(struct inode *inode, unsigned int cmd,
			     unsigned long arg);
int unionfs_ioctl_queryfile(struct file *file, unsigned int cmd,
			    unsigned long arg);

/* Verify that a branch is valid. */
int check_branch(struct nameidata *nd);

/* Extended attribute functions. */
extern void *xattr_alloc(size_t size, size_t limit);
extern void xattr_free(void *ptr, size_t size);

extern ssize_t unionfs_getxattr(struct dentry *dentry, const char *name,
				void *value, size_t size);
extern int unionfs_removexattr(struct dentry *dentry, const char *name);
extern ssize_t unionfs_listxattr(struct dentry *dentry, char *list,
				 size_t size);

int unionfs_setxattr(struct dentry *dentry, const char *name, const void *value,
		     size_t size, int flags);

/* The root directory is unhashed, but isn't deleted. */
static inline int d_deleted(struct dentry *d)
{
	return d_unhashed(d) && (d != d->d_sb->s_root);
}

/* returns the sum of the n_link values of all the underlying inodes of the passed inode */
static inline int get_nlinks(struct inode *inode)
{
	int sum_nlinks = 0;
	int dirs = 0;
	int bindex;
	struct inode *hidden_inode;

	if (!S_ISDIR(inode->i_mode))
		return itohi(inode)->i_nlink;

	for (bindex = ibstart(inode); bindex <= ibend(inode); bindex++) {
		hidden_inode = itohi_index(inode, bindex);
		if (!hidden_inode || !S_ISDIR(hidden_inode->i_mode))
			continue;
		BUG_ON(hidden_inode->i_nlink < 0);

		/* A deleted directory. */
		if (hidden_inode->i_nlink == 0)
			continue;
		dirs++;
		/* A broken directory (e.g., squashfs). */
		if (hidden_inode->i_nlink == 1)
			sum_nlinks += 2;
		else
			sum_nlinks += (hidden_inode->i_nlink - 2);
	}

	if (!dirs)
		return 0;
	return sum_nlinks + 2;
}

static inline void fist_copy_attr_atime(struct inode *dest,
					const struct inode *src)
{
	dest->i_atime = src->i_atime;
}
static inline void fist_copy_attr_times(struct inode *dest,
					const struct inode *src)
{
	dest->i_atime = src->i_atime;
	dest->i_mtime = src->i_mtime;
	dest->i_ctime = src->i_ctime;
}
static inline void fist_copy_attr_timesizes(struct inode *dest,
					    const struct inode *src)
{
	dest->i_atime = src->i_atime;
	dest->i_mtime = src->i_mtime;
	dest->i_ctime = src->i_ctime;
	dest->i_size = src->i_size;
	dest->i_blocks = src->i_blocks;
}
static inline void fist_copy_attr_all(struct inode *dest,
				      const struct inode *src)
{
	print_entry_location();

	dest->i_mode = src->i_mode;
	/* we do not need to copy if the file is a deleted file */
	if (dest->i_nlink > 0)
		dest->i_nlink = get_nlinks(dest);
	dest->i_uid = src->i_uid;
	dest->i_gid = src->i_gid;
	dest->i_rdev = src->i_rdev;
	dest->i_atime = src->i_atime;
	dest->i_mtime = src->i_mtime;
	dest->i_ctime = src->i_ctime;
	dest->i_blkbits = src->i_blkbits;
	dest->i_size = src->i_size;
	dest->i_blocks = src->i_blocks;
	dest->i_flags = src->i_flags;

	print_exit_location();
}

struct dentry *unionfs_lookup_backend(struct dentry *dentry, struct nameidata *nd, int lookupmode);
int is_stale_inode(struct inode *inode);
void make_stale_inode(struct inode *inode);

#define IS_SET(sb, check_flag) (check_flag & MOUNT_FLAG(sb))

/* unionfs_permission, check if we should bypass error to facilitate copyup */
#define IS_COPYUP_ERR(err) (err == -EROFS)

/* unionfs_open, check if we need to copyup the file */
#define OPEN_WRITE_FLAGS (O_WRONLY | O_RDWR | O_APPEND)
#define IS_WRITE_FLAG(flag) (flag & (OPEN_WRITE_FLAGS))

static inline int branchperms(struct super_block *sb, int index)
{
	BUG_ON(index < 0);

	return stopd(sb)->usi_data[index].branchperms;
}
static inline int set_branchperms(struct super_block *sb, int index, int perms)
{
	BUG_ON(index < 0);

	stopd(sb)->usi_data[index].branchperms = perms;

	return perms;
}

/* Is this file on a read-only branch? */
static inline int __is_robranch_super(struct super_block *sb, int index,
				      char *file, const char *function,
				      int line)
{
	int err = 0;

	print_util_entry_location();

	if (!(branchperms(sb, index) & MAY_WRITE))
		err = -EROFS;

	print_util_exit_status(err);
	return err;
}

/* Is this file on a read-only branch? */
static inline int __is_robranch_index(struct dentry *dentry, int index,
				      char *file, const char *function,
				      int line)
{
	int err = 0;
	int perms;

	print_util_entry_location();

	BUG_ON(index < 0);

	perms = stopd(dentry->d_sb)->usi_data[index].branchperms;

	if ((!(perms & MAY_WRITE))
	    || (IS_RDONLY(dtohd_index(dentry, index)->d_inode)))
		err = -EROFS;

	print_util_exit_status(err);

	return err;
}
static inline int __is_robranch(struct dentry *dentry, char *file,
				const char *function, int line)
{
	int index;
	int err;

	print_util_entry_location();

	index = dtopd(dentry)->udi_bstart;
	BUG_ON(index < 0);

	err = __is_robranch_index(dentry, index, file, function, line);

	print_util_exit_status(err);

	return err;
}

#define is_robranch(d) __is_robranch(d, __FILE__, __FUNCTION__, __LINE__)
#define is_robranch_super(s, n) __is_robranch_super(s, n, __FILE__, __FUNCTION__, __LINE__)

/* What do we use for whiteouts. */
#define WHPFX ".wh."
#define WHLEN 4
/* If a directory contains this file, then it is opaque.  We start with the
 * .wh. flag so that it is blocked by loomkup.
 */
#define UNIONFS_DIR_OPAQUE_NAME "__dir_opaque"
#define UNIONFS_DIR_OPAQUE WHPFX UNIONFS_DIR_OPAQUE_NAME

/* construct whiteout filename */
static inline char *alloc_whname(const char *name, int len)
{
	char *buf;

	buf = KMALLOC(len + WHLEN + 1, GFP_KERNEL);
	if (!buf)
		return ERR_PTR(-ENOMEM);

	strcpy(buf, WHPFX);
	strlcat(buf, name, len + WHLEN + 1);

	return buf;
}

/* Definitions for various ways to handle errors.
   Each flag's value is its bit position */

/* 1 = DELETE_ALL, 0 = check for DELETE_WHITEOUT */
#ifdef UNIONFS_DELETE_ALL
#define DELETE_ALL		4
#else
#define DELETE_ALL 		0
#endif

#define VALID_MOUNT_FLAGS (DELETE_ALL)

/*
 * MACROS:
 */

#ifndef SEEK_SET
#define SEEK_SET 0
#endif				/* not SEEK_SET */

#ifndef SEEK_CUR
#define SEEK_CUR 1
#endif				/* not SEEK_CUR */

#ifndef SEEK_END
#define SEEK_END 2
#endif				/* not SEEK_END */

#ifndef DEFAULT_POLLMASK
#define DEFAULT_POLLMASK (POLLIN | POLLOUT | POLLRDNORM | POLLWRNORM)
#endif

/*
 * EXTERNALS:
 */

/* JS: These two functions are here because it is kind of daft to copy and paste the
 * contents of the two functions to 32+ places in unionfs
 */
static inline struct dentry *lock_parent(struct dentry *dentry)
{
	struct dentry *dir = DGET(dentry->d_parent);

	mutex_lock(&dir->d_inode->i_mutex);
	return dir;
}

static inline void unlock_dir(struct dentry *dir)
{
	mutex_unlock(&dir->d_inode->i_mutex);
	DPUT(dir);
}

extern int make_dir_opaque(struct dentry *dir, int bindex);

#endif				/* __KERNEL__ */

/*
 * DEFINITIONS FOR USER AND KERNEL CODE:
 * (Note: ioctl numbers 1--9 are reserved for fistgen, the rest
 *  are auto-generated automatically based on the user's .fist file.)
 */
# define FIST_IOCTL_GET_DEBUG_VALUE	_IOR(0x15, 1, int)
# define FIST_IOCTL_SET_DEBUG_VALUE	_IOW(0x15, 2, int)
# define UNIONFS_IOCTL_BRANCH_COUNT	_IOR(0x15, 10, int)
# define UNIONFS_IOCTL_INCGEN		_IOR(0x15, 11, int)
# define UNIONFS_IOCTL_ADDBRANCH	_IOW(0x15, 12, int)
# define UNIONFS_IOCTL_DELBRANCH	_IOW(0x15, 13, int)
# define UNIONFS_IOCTL_RDWRBRANCH	_IOW(0x15, 14, int)
# define UNIONFS_IOCTL_QUERYFILE	_IOR(0x15, 15, int)

/* We don't support normal remount, but unionctl uses it. */
# define UNIONFS_REMOUNT_MAGIC		0x4a5a4380

/* should be at least LAST_USED_UNIONFS_PERMISSION<<1 */
#define MAY_NFSRO			16

struct unionfs_addbranch_args {
	unsigned int ab_branch;
	char *ab_path;
	unsigned int ab_perms;
};

struct unionfs_rdwrbranch_args {
	unsigned int rwb_branch;
	unsigned int rwb_perms;
};

#endif				/* not __UNIONFS_H_ */
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
