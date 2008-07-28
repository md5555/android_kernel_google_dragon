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
 *  $Id: print.c,v 1.77 2006/07/08 17:58:31 ezk Exp $
 */

/* Print debugging functions */

#include "unionfs.h"

static unsigned int debug_mask = DEFAULT_DEBUG_MASK;

/* get value of debugging variable */
unsigned int get_debug_mask(void)
{
	return debug_mask;
}

/* set debug level variable and return the previous value */
int set_debug_mask(int val)
{
#ifdef UNIONFS_DEBUG
	int prev = debug_mask;

	debug_mask = val;

	printk(KERN_INFO UNIONFS_NAME ": debug mask set to %u\n", debug_mask);

	return prev;
#else /* UNIONFS_DEBUG */
	printk(KERN_WARNING UNIONFS_NAME ": debugging is not enabled\n");
	return -ENOTSUPP;
#endif /* ! UNIONFS_DEBUG */
}

static inline int should_print(const unsigned int req)
{
	return (req & debug_mask);
}

static void unionfs_print_generic_inode(const char *prefix,
		const char *prefix2, const struct inode *inode)
{
	if (!inode) {
		printk(KERN_DEBUG UNIONFS_NAME ": PI:%s%s: NULL INODE PASSED!\n", prefix, prefix2);
		return;
	}

	if (IS_ERR(inode)) {
		printk(KERN_DEBUG UNIONFS_NAME ": PI:%s%s: ERROR INODE PASSED: %ld\n", prefix, prefix2,
		       PTR_ERR(inode));
		return;
	}

	printk(KERN_DEBUG UNIONFS_NAME ": PI:%s%s: i_ino=%lu\n",
			prefix, prefix2, inode->i_ino);
	printk(KERN_DEBUG UNIONFS_NAME ": PI:%s%s: i_count=%u\n",
			prefix, prefix2, atomic_read(&inode->i_count));
	printk(KERN_DEBUG UNIONFS_NAME ": PI:%s%s: i_nlink=%u\n",
			prefix, prefix2, inode->i_nlink);
	printk(KERN_DEBUG UNIONFS_NAME ": PI:%s%s: i_mode=%o\n",
			prefix, prefix2, inode->i_mode);
	printk(KERN_DEBUG UNIONFS_NAME ": PI:%s%s: i_size=%llu\n",
			prefix, prefix2, inode->i_size);
	printk(KERN_DEBUG UNIONFS_NAME ": PI:%s%s: i_op=%p\n",
			prefix, prefix2, inode->i_op);
	printk(KERN_DEBUG UNIONFS_NAME ": PI:%s%s: i_sb=%p (%s)\n",
			prefix, prefix2, inode->i_sb, (inode->i_sb ? sbt(inode->i_sb) : "NullTypeSB"));
}

void unionfs_print_inode(const unsigned int req, const char *prefix, const struct inode *inode)
{
	int bindex;

	if (!should_print(req))
		return;

	if (!inode) {
		printk(KERN_DEBUG UNIONFS_NAME ": PI:%s: NULL INODE PASSED!\n", prefix);
		return;
	}
	if (IS_ERR(inode)) {
		printk(KERN_DEBUG UNIONFS_NAME ": PI:%s: ERROR INODE PASSED: %ld\n", prefix, PTR_ERR(inode));
		return;
	}

	unionfs_print_generic_inode(prefix, "", inode);

	if (strcmp("unionfs", sbt(inode->i_sb))) {
		printk(KERN_DEBUG UNIONFS_NAME ": PI:%s: Not a " UNIONFS_NAME " inode.\n", prefix);
		return;
	}

	if (!itopd(inode))
		return;

	printk(KERN_DEBUG UNIONFS_NAME ": PI:%s: ibstart=%d, ibend=%d\n", prefix, ibstart(inode), ibend(inode));

	if (ibstart(inode) == -1)
		return;

	for (bindex = ibstart(inode); bindex <= ibend(inode); bindex++) {
		struct inode *hidden_inode = itohi_index(inode, bindex);
		char newstr[10];
		if (!hidden_inode) {
			printk(KERN_DEBUG UNIONFS_NAME ": PI:%s: HI#%d: NULL\n", prefix, bindex);
			continue;
		}
		snprintf(newstr, 10, ": HI%d", bindex);
		unionfs_print_generic_inode(prefix, newstr, hidden_inode);
	}
}

static void unionfs_print_generic_file(const char *prefix, const char *prefix2,
				     const struct file *file)
{
	printk(KERN_DEBUG UNIONFS_NAME ": PF:%s%s: f_dentry=0x%p\n", prefix, prefix2, file->f_dentry);

	printk(KERN_DEBUG UNIONFS_NAME ": PF:%s%s: name=%s\n", prefix, prefix2, file->f_dentry->d_name.name);
	if (file->f_dentry->d_inode) {
		printk(KERN_DEBUG UNIONFS_NAME ": PF:%s%s: f_dentry->d_inode->i_ino=%lu\n", prefix, prefix2, file->f_dentry->d_inode->i_ino);
		printk(KERN_DEBUG UNIONFS_NAME ": PF:%s%s: f_dentry->d_inode->i_mode=%o\n", prefix, prefix2, file->f_dentry->d_inode->i_mode);
	}
	printk(KERN_DEBUG UNIONFS_NAME ": PF:%s%s: f_op=0x%p\n", prefix, prefix2, file->f_op);
	printk(KERN_DEBUG UNIONFS_NAME ": PF:%s%s: f_mode=0x%x\n", prefix, prefix2, file->f_mode);
	printk(KERN_DEBUG UNIONFS_NAME ": PF:%s%s: f_pos=0x%llu\n", prefix, prefix2, file->f_pos);
	printk(KERN_DEBUG UNIONFS_NAME ": PF:%s%s: f_count=%u\n", prefix, prefix2, atomic_read(&file->f_count));
	printk(KERN_DEBUG UNIONFS_NAME ": PF:%s%s: f_flags=0x%x\n", prefix, prefix2, file->f_flags);
	printk(KERN_DEBUG UNIONFS_NAME ": PF:%s%s: f_version=%llu\n", prefix, prefix2, file->f_version);
}

void unionfs_print_file(const unsigned int req, const char *prefix, const struct file *file)
{
	struct file *hidden_file;

	if (!should_print(req))
		return;

	if (!file) {
		printk(KERN_DEBUG UNIONFS_NAME ": PF:%s: NULL FILE PASSED!\n", prefix);
		return;
	}

	unionfs_print_generic_file(prefix, "", file);

	if (strcmp("unionfs", sbt(file->f_dentry->d_sb))) {
		printk(KERN_DEBUG UNIONFS_NAME ": PF:%s: Not a " UNIONFS_NAME " file.\n", prefix);
		return;
	}

	if (ftopd(file)) {
		int bindex;

		printk(KERN_DEBUG UNIONFS_NAME ": PF:%s: fbstart=%d, fbend=%d\n", prefix, fbstart(file), fbend(file));

		for (bindex = fbstart(file); bindex <= fbend(file); bindex++) {
			char newstr[10];
			hidden_file = ftohf_index(file, bindex);
			if (!hidden_file) {
				printk(KERN_DEBUG UNIONFS_NAME ": PF:%s: HF#%d is NULL\n", prefix, bindex);
				continue;
			}
			snprintf(newstr, 10, ": HF%d", bindex);
			unionfs_print_generic_file(prefix, newstr, hidden_file);
		}
	}
}

static char mode_to_type(mode_t mode)
{
	if (S_ISDIR(mode))
		return 'd';
	if (S_ISLNK(mode))
		return 'l';
	if (S_ISCHR(mode))
		return 'c';
	if (S_ISBLK(mode))
		return 'b';
	if (S_ISREG(mode))
		return 'f';
	return '?';
}

static void unionfs_print_generic_dentry(const char *prefix, const char *prefix2, const
				 struct dentry *dentry, int check)
{
	if (!dentry) {
		printk(KERN_DEBUG UNIONFS_NAME ": PD:%s%s: NULL DENTRY PASSED!\n", prefix, prefix2);
		return;
	}

	if (IS_ERR(dentry)) {
		printk(KERN_DEBUG UNIONFS_NAME ": PD:%s%s: ERROR DENTRY (%ld)!\n", prefix, prefix2,
			    PTR_ERR(dentry));
		return;
	}

	printk(KERN_DEBUG UNIONFS_NAME ": PD:%s%s: dentry = %p\n", prefix, prefix2, dentry);
	printk(KERN_DEBUG UNIONFS_NAME ": PD:%s%s: d_count=%d\n", prefix, prefix2, atomic_read(&dentry->d_count));
	printk(KERN_DEBUG UNIONFS_NAME ": PD:%s%s: d_flags=%x\n", prefix, prefix2, (int)dentry->d_flags);
	printk(KERN_DEBUG UNIONFS_NAME ": PD:%s%s: d_name.name=\"%s\" (len = %d)\n", prefix, prefix2, dentry->d_name.name, dentry->d_name.len);
	printk(KERN_DEBUG UNIONFS_NAME ": PD:%s%s: d_sb=%p (%s)\n", prefix, prefix2, dentry->d_sb, sbt(dentry->d_sb));
	printk(KERN_DEBUG UNIONFS_NAME ": PD:%s%s: d_inode=%p\n", prefix, prefix2, dentry->d_inode);

	if (dentry->d_inode) {
		printk(KERN_DEBUG UNIONFS_NAME ": PD:%s%s: d_inode->i_ino=%ld (%s)\n", prefix, prefix2,
			    dentry->d_inode->i_ino,
			    sbt(dentry->d_inode->i_sb));
		printk(KERN_DEBUG UNIONFS_NAME ": PD:%s%s: dentry->d_inode->i_mode: %c%o\n", prefix,
			    prefix2, mode_to_type(dentry->d_inode->i_mode),
			    dentry->d_inode->i_mode);
	}

	printk(KERN_DEBUG UNIONFS_NAME ": PD:%s%s: d_parent=%p (%s)\n", prefix, prefix2,
		    dentry->d_parent,
		    (dentry->d_parent ? sbt(dentry->d_parent->d_sb) : "nil"));
	printk(KERN_DEBUG UNIONFS_NAME ": PD:%s%s: d_parent->d_name.name=\"%s\"\n", prefix, prefix2,
		    dentry->d_parent->d_name.name);
	printk(KERN_DEBUG UNIONFS_NAME ": PD:%s%s: d_parent->d_count=%d\n", prefix, prefix2,
		    atomic_read(&dentry->d_parent->d_count));
	printk(KERN_DEBUG UNIONFS_NAME ": PD:%s%s: d_op=%p\n", prefix, prefix2, dentry->d_op);
	printk(KERN_DEBUG UNIONFS_NAME ": PD:%s%s: d_fsdata=%p\n", prefix, prefix2,
		    dentry->d_fsdata);
	printk(KERN_DEBUG UNIONFS_NAME ": PD:%s%s: hlist_unhashed(d_hash)=%d\n", prefix, prefix2,
		    hlist_unhashed(&((struct dentry *)dentry)->d_hash));

	/* After we have printed it, we can assert something about it. */
	if (check)
		BUG_ON(atomic_read(&dentry->d_count) <= 0);
}

static void __unionfs_print_dentry(const char *prefix, const struct dentry *dentry,
			 int check)
{
	if (!dentry) {
		printk(KERN_DEBUG UNIONFS_NAME ": PD:%s: NULL DENTRY PASSED!\n", prefix);
		return;
	}

	if (IS_ERR(dentry)) {
		printk(KERN_DEBUG UNIONFS_NAME ": PD:%s: ERROR DENTRY (%ld)!\n", prefix,
			    PTR_ERR(dentry));
		return;
	}

	unionfs_print_generic_dentry(prefix, "", dentry, check);

	if (strcmp("unionfs", sbt(dentry->d_sb))) {
		printk(KERN_DEBUG UNIONFS_NAME ": PD:%s: Not a " UNIONFS_NAME " dentry.\n", prefix);
		return;
	}

	if (!dtopd(dentry))
		return;

	printk(KERN_DEBUG UNIONFS_NAME ": PD:%s: dbstart=%d, dbend=%d, dbopaque=%d\n",
		    prefix, dbstart(dentry), dbend(dentry), dbopaque(dentry));

	if (dbstart(dentry) != -1) {
		int bindex;
		char newstr[10];
		struct dentry *hidden_dentry;

		for (bindex = dbstart(dentry); bindex <= dbend(dentry);
		     bindex++) {
			hidden_dentry = dtohd_index(dentry, bindex);
			if (!hidden_dentry) {
				printk(KERN_DEBUG UNIONFS_NAME ": PD:%s: HD#%d: NULL\n", prefix, bindex);
				continue;
			}
			snprintf(newstr, 10, ": HD%d", bindex);
			unionfs_print_generic_dentry(prefix, newstr, hidden_dentry, check);
		}
	}
}

void unionfs_print_dentry(const unsigned int req, const char *prefix, const struct dentry *dentry)
{
	if (!should_print(req))
		return;

	__unionfs_print_dentry(prefix, dentry, 1);
}

void unionfs_print_dentry_nocheck(const unsigned int req, const char *prefix, const struct dentry *dentry)
{
	if (!should_print(req))
		return;

	__unionfs_print_dentry(prefix, dentry, 0);
}

void unionfs_checkinode(const unsigned int req, const struct inode *inode, const char *msg)
{
	if (!should_print(req))
		return;

	if (!inode) {
		printk(KERN_DEBUG UNIONFS_NAME ": unionfs_checkinode - inode is NULL! (%s)\n",
		       msg);
		return;
	}

	if (!itopd(inode)) {
		printk(KERN_DEBUG UNIONFS_NAME ": unionfs_checkinode(%ld) - no private data (%s)\n",
			    inode->i_ino, msg);
		return;
	}

	if ((itopd(inode)->b_start < 0) || !itohi(inode)) {
		printk(KERN_DEBUG UNIONFS_NAME
			    "unionfs_checkinode(%ld) - underlying is NULL! (%s)\n",
			    inode->i_ino, msg);
		return;
	}

	if (!inode->i_sb) {
		printk(KERN_DEBUG UNIONFS_NAME
			    ": unionfs_checkinode(%ld) - inode->i_sb is NULL! (%s)\n",
			    inode->i_ino, msg);
		return;
	}

	printk(KERN_DEBUG UNIONFS_NAME ": inode->i_sb->s_type %p\n", inode->i_sb->s_type);
	if (!inode->i_sb->s_type) {
		printk(KERN_DEBUG UNIONFS_NAME
			    ": unionfs_checkinode(%ld) - inode->i_sb->s_type is NULL! (%s)\n",
			    inode->i_ino, msg);
		return;
	}

	printk(KERN_DEBUG UNIONFS_NAME
		    ": CI: %s: inode->i_count = %d, hidden_inode->i_count = %d, inode = %lu, sb = %s, hidden_sb = %s\n",
		    msg, atomic_read(&inode->i_count),
		    itopd(inode)->b_start >=
		    0 ? atomic_read(&itohi(inode)->i_count) : -1, inode->i_ino,
		    inode->i_sb->s_type->name,
		    itopd(inode)->b_start >=
		    0 ? itohi(inode)->i_sb->s_type->name : "(none)");
}

void unionfs_print_sb(const unsigned int req, const char *prefix, const struct super_block *sb)
{
	struct super_block *hidden_superblock;

	if (!should_print(req))
		return;

	if (!sb) {
		printk(KERN_DEBUG UNIONFS_NAME ": PSB:%s: NULL SB PASSED!\n", prefix);
		return;
	}

	printk(KERN_DEBUG UNIONFS_NAME ": PSB:%s: s_blocksize=%lu\n", prefix, sb->s_blocksize);
	printk(KERN_DEBUG UNIONFS_NAME ": PSB:%s: s_blocksize_bits=%u\n", prefix, sb->s_blocksize_bits);
	printk(KERN_DEBUG UNIONFS_NAME ": PSB:%s: s_flags=0x%lx\n", prefix, sb->s_flags);
	printk(KERN_DEBUG UNIONFS_NAME ": PSB:%s: s_magic=0x%lx\n", prefix, sb->s_magic);
	printk(KERN_DEBUG UNIONFS_NAME ": PSB:%s: s_maxbytes=%llu\n", prefix, sb->s_maxbytes);
	printk(KERN_DEBUG UNIONFS_NAME ": PSB:%s: s_count=%d\n", prefix, sb->s_count);
	printk(KERN_DEBUG UNIONFS_NAME ": PSB:%s: s_active=%d\n", prefix, atomic_read(&sb->s_active));

	if (stopd(sb))
		printk(KERN_DEBUG UNIONFS_NAME ": sbstart=%d, sbend=%d\n", sbstart(sb),
			    sbend(sb));

	if (stopd(sb)) {
		int bindex;
		for (bindex = sbstart(sb); bindex <= sbend(sb); bindex++) {
			hidden_superblock = stohs_index(sb, bindex);
			if (!hidden_superblock) {
				printk(KERN_DEBUG UNIONFS_NAME ": PSB:%s: HS#%d is NULL", prefix,
					    bindex);
				continue;
			}

			printk(KERN_DEBUG UNIONFS_NAME ": PSB:%s: HS#%d: s_blocksize=%lu\n", prefix, bindex,
				    hidden_superblock->s_blocksize);
			printk(KERN_DEBUG UNIONFS_NAME ": PSB:%s: HS#%d: s_blocksize_bits=%u\n", prefix, bindex,
				    hidden_superblock->s_blocksize_bits);
			printk(KERN_DEBUG UNIONFS_NAME ": PSB:%s: HS#%d: s_flags=0x%lx\n", prefix, bindex,
				    hidden_superblock->s_flags);
			printk(KERN_DEBUG UNIONFS_NAME ": PSB:%s: HS#%d: s_magic=0x%lx\n", prefix, bindex,
				    hidden_superblock->s_magic);
			printk(KERN_DEBUG UNIONFS_NAME ": PSB:%s: HS#%d: s_maxbytes=%llu\n", prefix, bindex,
				    hidden_superblock->s_maxbytes);
			printk(KERN_DEBUG UNIONFS_NAME ": PSB:%s: HS#%d: s_count=%d\n", prefix, bindex,
				    hidden_superblock->s_count);
			printk(KERN_DEBUG UNIONFS_NAME ": PSB:%s: HS#%d: s_active=%d\n", prefix, bindex,
				    atomic_read(&hidden_superblock->s_active));
		}
	}
}

int unionfs_print(const unsigned int req, const char *fmt, ...)
{
        va_list ap;
	int r;

	if (!should_print(req))
		return 0;

	printk(KERN_DEBUG UNIONFS_NAME ": ");
	va_start(ap, fmt);
	r = vprintk(fmt, ap);
	va_end(ap);

	return r;
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
