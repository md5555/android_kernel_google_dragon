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
 * magic sysrq hanlder
 *
 * $Id: sysrq.c,v 1.10 2008/09/08 02:40:48 sfjro Exp $
 */

#include <linux/fs.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
/* #include <linux/sysrq.h> */
#include "aufs.h"

static void sysrq_sb(struct super_block *sb)
{
	char *plevel;
	struct inode *i;

	plevel = au_plevel;
	au_plevel = KERN_WARNING;
	au_debug_on();

	pr_warning("si=%lx\n", au_si_mask ^ (unsigned long)au_sbi(sb));
	pr_warning(AUFS_NAME ": superblock\n");
	au_dpri_sb(sb);
	pr_warning(AUFS_NAME ": root dentry\n");
	au_dpri_dentry(sb->s_root);
	pr_warning(AUFS_NAME ": root inode\n");
	au_dpri_inode(sb->s_root->d_inode);
	pr_warning(AUFS_NAME ": isolated inode\n");
	list_for_each_entry(i, &sb->s_inodes, i_sb_list)
		if (list_empty(&i->i_dentry))
			au_dpri_inode(i);

	au_plevel = plevel;
	au_debug_off();
}

/* ---------------------------------------------------------------------- */

/* module parameter */
static char *aufs_sysrq_key = "a";
module_param_named(sysrq, aufs_sysrq_key, charp, S_IRUGO);
MODULE_PARM_DESC(sysrq, "MagicSysRq key for " AUFS_NAME);

static void au_sysrq(int key, struct tty_struct *tty)
{
	struct kobject *kobj;
	struct au_sbinfo *sbinfo;

	/* spin_lock(&au_kset->list_lock); */
	list_for_each_entry(kobj, &au_kset->list, entry) {
		sbinfo = container_of(kobj, struct au_sbinfo, si_kobj);
		sysrq_sb(sbinfo->si_sb);
	}
	/* spin_unlock(&au_kset->list_lock); */
}

static struct sysrq_key_op au_sysrq_op = {
	.handler	= au_sysrq,
	.help_msg	= "Aufs",
	.action_msg	= "Aufs",
	/* todo: test mask? */
	.enable_mask	= SYSRQ_ENABLE_DUMP
};

/* ---------------------------------------------------------------------- */

int __init au_sysrq_init(void)
{
	int err;
	char key;

	err = -1;
	key = *aufs_sysrq_key;
	if ('a' <= key && key <= 'z')
		err = register_sysrq_key(key, &au_sysrq_op);
	if (unlikely(err))
		AuErr("err %d, sysrq=%c\n", err, key);
	return err;
}

void au_sysrq_fin(void)
{
	int err;
	err = unregister_sysrq_key(*aufs_sysrq_key, &au_sysrq_op);
	if (unlikely(err))
		AuErr("err %d (ignored)\n", err);
}
