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
 * export via nfs
 *
 * $Id: export.c,v 1.15 2008/09/22 03:52:19 sfjro Exp $
 */

#include <linux/exportfs.h>
#include <linux/mnt_namespace.h>
#include <linux/random.h>
#include "aufs.h"

union conv {
#ifdef CONFIG_AUFS_INO_T_64
	__u32 a[2];
#else
	__u32 a[1];
#endif
	ino_t ino;
};

static ino_t decode_ino(__u32 *a)
{
	union conv u;

	BUILD_BUG_ON(sizeof(u.ino) != sizeof(u.a));
	u.a[0] = a[0];
#ifdef CONFIG_AUFS_INO_T_64
	u.a[1] = a[1];
#endif
	return u.ino;
}

static void encode_ino(__u32 *a, ino_t ino)
{
	union conv u;

	u.ino = ino;
	a[0] = u.a[0];
#ifdef CONFIG_AUFS_INO_T_64
	a[1] = u.a[1];
#endif
}

/* NFS file handle */
enum {
	Fh_br_id,
	Fh_sigen,
#ifdef CONFIG_AUFS_INO_T_64
	/* support 64bit inode number */
	Fh_ino1,
	Fh_ino2,
	Fh_dir_ino1,
	Fh_dir_ino2,
#else
	Fh_ino1,
	Fh_dir_ino1,
#endif
	Fh_igen,
	Fh_h_type,
	Fh_tail,

	Fh_ino = Fh_ino1,
	Fh_dir_ino = Fh_dir_ino1
};

static int au_test_anon(struct dentry *dentry)
{
	return !!(dentry->d_flags & DCACHE_DISCONNECTED);
}

/* ---------------------------------------------------------------------- */
/* inode generation external table */

int au_xigen_inc(struct inode *inode)
{
	int err;
	loff_t pos;
	ssize_t sz;
	__u32 igen;
	struct super_block *sb;
	struct au_sbinfo *sbinfo;

	LKTRTrace("i%lu\n", (unsigned long)inode->i_ino);

	err = 0;
	sb = inode->i_sb;
	if (unlikely(!au_opt_test_xino(au_mntflags(sb))))
		goto out;

	pos = inode->i_ino;
	pos *= sizeof(igen);
	igen = inode->i_generation + 1;
	sbinfo = au_sbi(sb);
	sz = xino_fwrite(sbinfo->si_xwrite, sbinfo->si_xigen, &igen,
			 sizeof(igen), &pos);
	if (sz == sizeof(igen))
		goto out; /* success */

	err = sz;
	if (unlikely(sz >= 0)) {
		err = -EIO;
		AuIOErr("xigen error (%ld)\n", (long)sz);
	}

 out:
	AuTraceErr(err);
	return err;
}

int au_xigen_new(struct inode *inode)
{
	int err;
	loff_t pos;
	ssize_t sz;
	struct super_block *sb;
	struct au_sbinfo *sbinfo;
	struct file *file;

	LKTRTrace("i%lu\n", (unsigned long)inode->i_ino);

	err = 0;
	sb = inode->i_sb;
	if (unlikely(!au_opt_test_xino(au_mntflags(sb))))
		goto out;

	err = -EFBIG;
	pos = inode->i_ino;
	if (unlikely(Au_LOFF_MAX / sizeof(inode->i_generation) - 1 < pos)) {
		AuIOErr1("too large i%lld\n", pos);
		goto out;
	}
	pos *= sizeof(inode->i_generation);

	err = 0;
	sbinfo = au_sbi(sb);
	file = sbinfo->si_xigen;
	/* todo: dirty, at mount time */
	if (unlikely(!file)) {
		if (inode->i_ino == AUFS_ROOT_INO)
			goto out;
		else
			BUG();
	}

	if (i_size_read(file->f_dentry->d_inode)
	    < pos + sizeof(inode->i_generation)) {
		spin_lock(&sbinfo->si_xigen_lock);
		inode->i_generation = sbinfo->si_xigen_next++;
		spin_unlock(&sbinfo->si_xigen_lock);
		sz = xino_fwrite(sbinfo->si_xwrite, file, &inode->i_generation,
				 sizeof(inode->i_generation), &pos);
	} else
		sz = xino_fread(sbinfo->si_xread, file, &inode->i_generation,
				sizeof(inode->i_generation), &pos);
	if (sz == sizeof(inode->i_generation))
		goto out; /* success */

	err = sz;
	if (unlikely(sz >= 0)) {
		err = -EIO;
		AuIOErr("xigen error (%ld)\n", (long)sz);
	}

 out:
	AuTraceErr(err);
	return err;
}

int au_xigen_set(struct super_block *sb, struct file *base)
{
	int err;
	struct au_sbinfo *sbinfo;
	struct file *file;

	LKTRTrace("%.*s\n", AuDLNPair(base->f_dentry));
	SiMustWriteLock(sb);

	sbinfo = au_sbi(sb);
	file = au_xino_create2(sb, base, sbinfo->si_xigen);
	err = PTR_ERR(file);
	if (IS_ERR(file))
		goto out;
	err = 0;
	if (sbinfo->si_xigen)
		fput(sbinfo->si_xigen);
	sbinfo->si_xigen = file;

 out:
	AuTraceErr(err);
	return err;
}

void au_xigen_clr(struct super_block *sb)
{
	struct au_sbinfo *sbinfo;

	sbinfo = au_sbi(sb);
	if (sbinfo->si_xigen) {
		fput(sbinfo->si_xigen);
		sbinfo->si_xigen = NULL;
	}
}

/* ---------------------------------------------------------------------- */

static struct dentry *decode_by_ino(struct super_block *sb, ino_t ino,
				    ino_t dir_ino)
{
	struct dentry *dentry, *d;
	struct inode *inode;

	LKTRTrace("i%lu, diri%lu\n",
		  (unsigned long)ino, (unsigned long)dir_ino);

	dentry = NULL;
	inode = ilookup(sb, ino);
	if (unlikely(!inode))
		goto out;

	dentry = ERR_PTR(-ESTALE);
	if (unlikely(is_bad_inode(inode) || IS_DEADDIR(inode)))
		goto out_iput;
	AuDbgInode(inode);

	dentry = NULL;
	if (!dir_ino || S_ISDIR(inode->i_mode))
 		dentry = d_find_alias(inode);
	else {
		spin_lock(&dcache_lock);
		list_for_each_entry(d, &inode->i_dentry, d_alias)
			if (!au_test_anon(d)
			    && d->d_parent->d_inode->i_ino == dir_ino) {
				dentry = dget_locked(d);
				break;
			}
		spin_unlock(&dcache_lock);
	}
	AuDbgDentry(dentry);

 out_iput:
	iput(inode);
 out:
	AuTraceErrPtr(dentry);
	return dentry;
}

/* ---------------------------------------------------------------------- */

/* todo: dirty? */
/*
 * when you mntput() for the return value of this function,
 * you have to store it to your local var.
 * ie. never mntput si_mntcache directly.
 */
static struct vfsmount *au_do_mnt_get(struct super_block *sb)
{
	struct mnt_namespace *ns;
	struct vfsmount *pos, *mnt;

	AuTraceEnter();

	/* vfsmount_lock is not exported */
	/* no get/put ?? */
	AuDebugOn(!current->nsproxy);
	ns = current->nsproxy->mnt_ns;
	AuDebugOn(!ns);
	mnt = NULL;
	/* the order (reverse) will not be a problem */
	list_for_each_entry(pos, &ns->list, mnt_list)
		if (pos->mnt_sb == sb) {
			mnt = pos;
			break;
		}
	AuDebugOn(!mnt);

	return mntget(mnt);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 26)
static struct vfsmount *au_mnt_get(struct super_block *sb)
{
	struct au_sbinfo *sbinfo;
	struct vfsmount *mnt;

	sbinfo = au_sbi(sb);
	spin_lock(&sbinfo->si_mntcache_lock);
	if (sbinfo->si_mntcache)
		mnt = mntget(sbinfo->si_mntcache);
	else {
		sbinfo->si_mntcache = au_do_mnt_get(sb);
		mnt = sbinfo->si_mntcache;
	}
	spin_unlock(&sbinfo->si_mntcache_lock);
	return mnt;
}
#else
static struct vfsmount *au_mnt_get(struct super_block *sb)
{
	return au_do_mnt_get(sb);
}
#endif

struct find_name_by_ino {
	int called, found;
	ino_t ino;
	char *name;
	int namelen;
};

static int
find_name_by_ino(void *arg, const char *name, int namelen, loff_t offset,
		 u64 ino, unsigned int d_type)
{
	struct find_name_by_ino *a = arg;

	a->called++;
	if (a->ino != ino)
		return 0;

	memcpy(a->name, name, namelen);
	a->namelen = namelen;
	a->found = 1;
	return 1;
}

static struct dentry *au_lkup_by_ino(struct path *path, ino_t ino)
{
	struct dentry *dentry, *parent;
	struct file *file;
	struct inode *dir, *inode;
	struct find_name_by_ino arg;
	int err;

	parent = path->dentry;
	LKTRTrace("%.*s, i%lu\n", AuDLNPair(parent), (unsigned long )ino);

	path_get(path);
	file = dentry_open(parent, path->mnt, au_dir_roflags);
	dentry = (void *)file;
	if (IS_ERR(file))
		goto out;

	dentry = ERR_PTR(-ENOMEM);
	arg.name = __getname();
	if (unlikely(!arg.name))
		goto out_file;
	arg.ino = ino;
	arg.found = 0;
	do {
		arg.called = 0;
		/* smp_mb(); */
		err = vfsub_readdir(file, find_name_by_ino, &arg, /*dlgt*/0);
	} while (!err && !arg.found && arg.called);
	dentry = ERR_PTR(err);
	if (unlikely(err))
		goto out_name;
	dentry = ERR_PTR(-ENOENT);
	if (!arg.found)
		goto out_name;

	/* do not call au_lkup_one(), nor dlgt */
	dir = parent->d_inode;
	mutex_lock(&dir->i_mutex);
	dentry = vfsub_lookup_one_len(arg.name, parent, arg.namelen);
	mutex_unlock(&dir->i_mutex);
	AuTraceErrPtr(dentry);
	if (IS_ERR(dentry))
		goto out_name;
	AuDebugOn(au_test_anon(dentry));
	inode = dentry->d_inode;
	if (unlikely(!inode)) {
		dput(dentry);
		dentry = ERR_PTR(-ENOENT);
	}

 out_name:
	__putname(arg.name);
 out_file:
	fput(file);
 out:
	AuTraceErrPtr(dentry);
	return dentry;
}

static /* noinline_for_stack */
struct dentry *decode_by_dir_ino(struct super_block *sb, ino_t ino,
				 ino_t dir_ino)
{
	struct dentry *dentry, *parent;
	struct path path;

	LKTRTrace("i%lu, diri%lu\n",
		  (unsigned long)ino, (unsigned long)dir_ino);

	parent = sb->s_root;
	if (dir_ino != AUFS_ROOT_INO) {
		parent = decode_by_ino(sb, dir_ino, 0);
		AuDbgDentry(parent);
		dentry = parent;
		if (unlikely(!parent))
			goto out;
		if (IS_ERR(parent))
			goto out;
		AuDebugOn(au_test_anon(parent));
	} else
		dget(parent);

	path.dentry = parent;
	path.mnt = au_mnt_get(sb);
	dentry = au_lkup_by_ino(&path, ino);
	path_put(&path);

 out:
	AuTraceErrPtr(dentry);
	return dentry;
}

/* ---------------------------------------------------------------------- */

static int h_acceptable(void *expv, struct dentry *dentry)
{
	return 1;
}

static char *au_build_path(struct dentry *h_parent, struct path *h_rootpath,
			   char *buf, int len, struct super_block *sb)
{
	char *p;
	int n;
	struct path path;

	AuTraceEnter();

	p = d_path(h_rootpath, buf, len);
	if (IS_ERR(p))
		goto out;
	n = strlen(p);

	path.mnt = h_rootpath->mnt;
	path.dentry = h_parent;
	p = d_path(&path, buf, len);
	if (IS_ERR(p))
		goto out;
	LKTRTrace("%s\n", p);
	if (n != 1)
		p += n;
	LKTRTrace("%p, %s, %ld\n",
		  p, p, (long)(p - buf));

	path.mnt = au_mnt_get(sb);
	path.dentry = sb->s_root;
	p = d_path(&path, buf, len - strlen(p));
	mntput(path.mnt);
	if (IS_ERR(p))
		goto out;
	if (n != 1)
		p[strlen(p)] = '/';
	LKTRTrace("%s\n", p);

 out:
	AuTraceErrPtr(p);
	return p;
}

static noinline_for_stack
struct dentry *decode_by_path(struct super_block *sb, aufs_bindex_t bindex,
			      ino_t ino, __u32 *fh, int fh_len)
{
	struct dentry *dentry, *h_parent, *root;
	struct super_block *h_sb;
	char *pathname, *p;
	struct vfsmount *h_mnt;
	struct au_branch *br;
	int err;
	struct nameidata nd;

	LKTRTrace("b%d\n", bindex);
	SiMustAnyLock(sb);

	br = au_sbr(sb, bindex);
	/* au_br_get(br); */
	h_mnt = br->br_mnt;
	h_sb = h_mnt->mnt_sb;
	LKTRTrace("%s, h_decode_fh\n", au_sbtype(h_sb));
	/* in linux-2.6.24, it takes struct fid * as file handle */
	/* todo: call lower fh_to_dentry()? fh_to_parent()? */
	h_parent = exportfs_decode_fh(h_mnt, (void *)(fh + Fh_tail),
				      fh_len - Fh_tail, fh[Fh_h_type],
				      h_acceptable, /*context*/NULL);
	dentry = h_parent;
	if (unlikely(!h_parent || IS_ERR(h_parent))) {
		AuWarn1("%s decode_fh failed, %ld\n",
			au_sbtype(h_sb), PTR_ERR(h_parent));
		goto out;
	}
	dentry = NULL;
	if (unlikely(au_test_anon(h_parent))) {
		AuWarn1("%s decode_fh returned a disconnected dentry\n",
			au_sbtype(h_sb));
		goto out_h_parent;
	}

	dentry = ERR_PTR(-ENOMEM);
	pathname = (void *)__get_free_page(GFP_NOFS);
	if (unlikely(!pathname))
		goto out_h_parent;

	root = sb->s_root;
	nd.path.mnt = h_mnt;
	di_read_lock_parent(root, !AuLock_IR);
	nd.path.dentry = au_h_dptr(root, bindex);
	di_read_unlock(root, !AuLock_IR);
	p = au_build_path(h_parent, &nd.path, pathname, PAGE_SIZE, sb);
	dentry = (void *)p;
	if (IS_ERR(p))
		goto out_pathname;

	LKTRTrace("%s\n", p);
	err = vfsub_path_lookup(p, LOOKUP_FOLLOW | LOOKUP_DIRECTORY, &nd);
	dentry = ERR_PTR(err);
	if (unlikely(err))
		goto out_pathname;

	dentry = ERR_PTR(-ENOENT);
	AuDebugOn(au_test_anon(nd.path.dentry));
	if (unlikely(!nd.path.dentry->d_inode))
		goto out_nd;

	if (ino != nd.path.dentry->d_inode->i_ino)
		dentry = au_lkup_by_ino(&nd.path, ino);
	else
		dentry = dget(nd.path.dentry);

 out_nd:
	path_put(&nd.path);
 out_pathname:
	free_page((unsigned long)pathname);
 out_h_parent:
	dput(h_parent);
 out:
	/* au_br_put(br); */
	AuTraceErrPtr(dentry);
	return dentry;
}

/* ---------------------------------------------------------------------- */

static struct dentry *
aufs_fh_to_dentry(struct super_block *sb, struct fid *fid, int fh_len,
		  int fh_type)
{
	struct dentry *dentry;
	struct inode *inode;
	__u32 *fh = fid->raw;
	ino_t ino, dir_ino;
	aufs_bindex_t bindex, br_id;
	au_gen_t sigen;

	LKTRTrace("%d, fh{br_id %u, sigen %u, i%u, diri%u, g%u}\n",
		  fh_type, fh[Fh_br_id], fh[Fh_sigen], fh[Fh_ino],
		  fh[Fh_dir_ino], fh[Fh_igen]);
	AuDebugOn(fh_len < Fh_tail);

	si_read_lock(sb, AuLock_FLUSH);
	lockdep_off();

	/* branch id may be wrapped around */
	dentry = ERR_PTR(-ESTALE);
	br_id = fh[Fh_br_id];
	sigen = fh[Fh_sigen];
	bindex = au_br_index(sb, br_id);
	LKTRTrace("b%d\n", bindex);
	if (unlikely(bindex < 0
		     || (0 && sigen != au_sigen(sb))
		     || (1 && sigen + AUFS_BRANCH_MAX <= au_sigen(sb))
		    ))
		goto out;

	/* is this inode still cached? */
	ino = decode_ino(fh + Fh_ino);
	AuDebugOn(ino == AUFS_ROOT_INO);
	dir_ino = decode_ino(fh + Fh_dir_ino);
	dentry = decode_by_ino(sb, ino, dir_ino);
	if (IS_ERR(dentry))
		goto out;
	if (dentry)
		goto accept;

	/* is the parent dir cached? */
	dentry = decode_by_dir_ino(sb, ino, dir_ino);
	if (IS_ERR(dentry))
		goto out;
	if (dentry)
		goto accept;

	/* lookup path */
	dentry = decode_by_path(sb, bindex, ino, fh, fh_len);
	if (IS_ERR(dentry))
		goto out;
	if (unlikely(!dentry))
		goto out;

 accept:
	LKTRLabel(accept);
	inode = dentry->d_inode;
#if 0
	/* support branch manupilation and udba on nfs server */
	sigen = au_sigen(sb);
	if (unlikely(au_digen(dentry) != sigen
		     || au_iigen(inode) != sigen)) {
		int err;

		//lktr_set_pid(current->pid, LktrArrayPid);
		//au_fset_si(au_sbi(dentry->d_sb), FAILED_REFRESH_DIRS);
		di_write_lock_child(dentry);
		err = au_reval_dpath(dentry, sigen);
		di_write_unlock(dentry);
		//lktr_clear_pid(current->pid, LktrArrayPid);
		if (unlikely(err < 0))
			goto out_dput;
	}
#endif

	if (unlikely(inode->i_generation != fh[Fh_igen])) {
		LKTRLabel(stale);
		dput(dentry);
		dentry = ERR_PTR(-ESTALE);
	}

 out:
	LKTRLabel(out);
	lockdep_on();
	si_read_unlock(sb);
	AuTraceErrPtr(dentry);
	return dentry;
}

#if 0 /* reserved for future use */
/* support subtreecheck option */
static struct dentry *aufs_fh_to_parent(struct super_block *sb, struct fid *fid,
					int fh_len, int fh_type)
{
	struct dentry *parent;
	__u32 *fh = fid->raw;
	ino_t dir_ino;

	dir_ino = decode_ino(fh + Fh_dir_ino);
	parent = decode_by_ino(sb, dir_ino, 0);
	if (IS_ERR(parent))
		goto out;
	if (!parent)
		parent = decode_by_path(sb, au_br_index(sb, fh[Fh_br_id]),
					dir_ino, fh, fh_len);

 out:
	AuTraceErrPtr(parent);
	return parent;
}
#endif

/* ---------------------------------------------------------------------- */

static int aufs_encode_fh(struct dentry *dentry, __u32 *fh, int *max_len,
			  int connectable)
{
	int err;
	aufs_bindex_t bindex, bend;
	struct super_block *sb, *h_sb;
	struct inode *inode;
	struct dentry *parent, *h_parent;
	struct au_branch *br;

	LKTRTrace("%.*s, max %d, conn %d\n",
		  AuDLNPair(dentry), *max_len, connectable);
	AuDebugOn(au_test_anon(dentry));

	parent = NULL;
	err = -ENOSPC;
	if (unlikely(*max_len <= Fh_tail)) {
		AuWarn1("NFSv2 client (max_len %d)?\n", *max_len);
		goto out;
	}

	err = FILEID_ROOT;
	inode = dentry->d_inode;
	AuDebugOn(!inode);
	if (inode->i_ino == AUFS_ROOT_INO)
		goto out;

	err = -EIO;
	h_parent = NULL;
	sb = dentry->d_sb;
	parent = dget_parent(dentry);
	aufs_read_lock(parent, AuLock_FLUSH | AuLock_IR);
#ifdef CONFIG_AUFS_DEBUG
	{
		unsigned int mnt_flags = au_mntflags(sb);

		if (unlikely(!au_opt_test_xino(mnt_flags)))
			AuWarn1("NFS-exporting requires xino\n");
		if (unlikely(0 && !au_opt_test(mnt_flags, UDBA_INOTIFY)))
			AuWarn1("udba=inotify is recommended "
				"for NFS-exporting\n");
	}
#endif

	bend = au_dbtaildir(parent);
	for (bindex = au_dbstart(parent); bindex <= bend; bindex++) {
		h_parent = au_h_dptr(parent, bindex);
		if (h_parent) {
			dget(h_parent);
			break;
		}
	}
	if (unlikely(!h_parent))
		goto out_unlock;
	LKTRTrace("b%d\n", bindex);

	err = -EPERM;
	br = au_sbr(sb, bindex);
	h_sb = br->br_mnt->mnt_sb;
	if (unlikely(!h_sb->s_export_op)) {
		AuErr1("%s branch is not exportable\n", au_sbtype(h_sb));
		goto out_dput;
	}

	fh[Fh_br_id] = br->br_id;
	fh[Fh_sigen] = au_sigen(sb);
	encode_ino(fh + Fh_ino, inode->i_ino);
	encode_ino(fh + Fh_dir_ino, parent->d_inode->i_ino);
	fh[Fh_igen] = inode->i_generation;

	*max_len -= Fh_tail;
	/* in linux-2.6.24, it takes struct fid * as file handle */
	fh[Fh_h_type] = exportfs_encode_fh(h_parent, (void *)(fh + Fh_tail),
					   max_len, connectable);
	err = fh[Fh_h_type];
	*max_len += Fh_tail;
	/* todo: macros? */
	if (err != 255)
		err = 99;
	else
		AuWarn1("%s encode_fh failed\n", au_sbtype(h_sb));

 out_dput:
	dput(h_parent);
 out_unlock:
	aufs_read_unlock(parent, AuLock_IR);
	dput(parent);
 out:
	AuTraceErr(err);
	if (unlikely(err < 0))
		err = 255;
	return err;
}

/* ---------------------------------------------------------------------- */

static struct export_operations aufs_export_op = {
	.fh_to_dentry	= aufs_fh_to_dentry,
	//.fh_to_parent	= aufs_fh_to_parent,
	.encode_fh	= aufs_encode_fh
};

void au_export_init(struct super_block *sb)
{
	struct au_sbinfo *sbinfo;

	AuTraceEnter();
	SiMustWriteLock(sb);

	sb->s_export_op = &aufs_export_op;
	sbinfo = au_sbi(sb);
	sbinfo->si_xigen = NULL;
	spin_lock_init(&sbinfo->si_xigen_lock);
	/* todo: meaningless? */
	get_random_bytes(&sbinfo->si_xigen_next, sizeof(sbinfo->si_xigen_next));
	memset(&sbinfo->si_xinodir, 0, sizeof(struct path));
}
