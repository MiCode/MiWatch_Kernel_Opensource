// SPDX-License-Identifier: GPL-2.0

/*
 * This file is part of the QM35 UCI stack for linux.
 *
 * Copyright (c) 2022 Qorvo US, Inc.
 *
 * This software is provided under the GNU General Public License, version 2
 * (GPLv2), as well as under a Qorvo commercial license.
 *
 * You may choose to use this software under the terms of the GPLv2 License,
 * version 2 ("GPLv2"), as published by the Free Software Foundation.
 * You should have received a copy of the GPLv2 along with this program.  If
 * not, see <http://www.gnu.org/licenses/>.
 *
 * This program is distributed under the GPLv2 in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GPLv2 for more
 * details.
 *
 * If you cannot meet the requirements of the GPLv2, you may not use this
 * software for any purpose without first obtaining a commercial license from
 * Qorvo.
 * Please contact Qorvo to inquire about licensing terms.
 *
 * QM35 LOG layer HSSPI Protocol
 */

#include <linux/debugfs.h>
#include <linux/poll.h>
#include <linux/fsnotify.h>

#include <qmrom.h>

#include "qm35.h"
#include "debug.h"

static const struct file_operations debug_enable_fops;
static const struct file_operations debug_log_level_fops;

static void *priv_from_file(const struct file *filp)
{
	return filp->f_path.dentry->d_inode->i_private;
}

static ssize_t debug_enable_write(struct file *filp, const char __user *buff,
				  size_t count, loff_t *off)
{
	struct debug *debug;
	u8 enabled;

	debug = priv_from_file(filp);

	if (kstrtou8_from_user(buff, count, 10, &enabled))
		return -EFAULT;

	if (debug->trace_ops)
		debug->trace_ops->enable_set(debug, enabled == 1 ? 1 : 0);
	else
		return -ENOSYS;

	return count;
}

static ssize_t debug_enable_read(struct file *filp, char __user *buff,
				 size_t count, loff_t *off)
{
	char enabled[2];
	struct debug *debug;

	debug = priv_from_file(filp);

	if (debug->trace_ops)
		enabled[0] = debug->trace_ops->enable_get(debug) + '0';
	else
		return -ENOSYS;

	enabled[1] = '\n';

	return simple_read_from_buffer(buff, count, off, enabled,
				       sizeof(enabled));
}

static ssize_t debug_log_level_write(struct file *filp, const char __user *buff,
				     size_t count, loff_t *off)
{
	u8 log_level = 0;
	struct log_module *log_module;

	log_module = priv_from_file(filp);
	if (kstrtou8_from_user(buff, count, 10, &log_level))
		return -EFAULT;

	if (log_module->debug->trace_ops)
		log_module->debug->trace_ops->level_set(log_module->debug,
							log_module, log_level);
	else
		return -ENOSYS;

	return count;
}

static ssize_t debug_log_level_read(struct file *filp, char __user *buff,
				    size_t count, loff_t *off)
{
	char log_level[2];
	struct log_module *log_module;

	log_module = priv_from_file(filp);

	if (log_module->debug->trace_ops)
		log_level[0] = log_module->debug->trace_ops->level_get(
				       log_module->debug, log_module) +
			       '0';
	else
		return -ENOSYS;

	log_level[1] = '\n';

	return simple_read_from_buffer(buff, count, off, log_level,
				       sizeof(log_level));
}

static ssize_t debug_traces_read(struct file *filp, char __user *buff,
				 size_t count, loff_t *off)
{
	char *entry;
	rb_entry_size_t entry_size;
	struct qm35_ctx *qm35_hdl;
	uint16_t ret;
	struct debug *debug;

	debug = priv_from_file(filp);
	qm35_hdl = container_of(debug, struct qm35_ctx, debug);

	if (!debug->trace_ops)
		return -ENOSYS;

	entry_size = debug->trace_ops->trace_get_next_size(debug);
	if (!entry_size) {
		if (filp->f_flags & O_NONBLOCK)
			return 0;

		ret = wait_event_interruptible(
			debug->wq,
			(entry_size = debug->trace_ops->trace_get_next_size(debug)));
		if (ret)
			return ret;
	}

	if (entry_size > count)
		return -EMSGSIZE;

	entry = debug->trace_ops->trace_get_next(debug, &entry_size);
	if (!entry)
		return 0;

	ret = copy_to_user(buff, entry, entry_size);

	kfree(entry);

	return ret ? -EFAULT : entry_size;
}

static __poll_t debug_traces_poll(struct file *filp,
				  struct poll_table_struct *wait)
{
	struct debug *debug;
	__poll_t mask = 0;

	debug = priv_from_file(filp);

	poll_wait(filp, &debug->wq, wait);

	if (debug->trace_ops && debug->trace_ops->trace_next_avail(debug))
		mask |= POLLIN;

	return mask;
}

static int debug_traces_open(struct inode *inodep, struct file *filep)
{
	struct debug *debug;

	debug = priv_from_file(filep);

	mutex_lock(&debug->pv_filp_lock);
	if (debug->pv_filp) {
		mutex_unlock(&debug->pv_filp_lock);
		return -EBUSY;
	}

	debug->pv_filp = filep;

	if (debug->trace_ops)
		debug->trace_ops->trace_reset(debug);

	mutex_unlock(&debug->pv_filp_lock);

	return 0;
}

static int debug_traces_release(struct inode *inodep, struct file *filep)
{
	struct debug *debug;

	debug = priv_from_file(filep);

	mutex_lock(&debug->pv_filp_lock);
	debug->pv_filp = NULL;
	mutex_unlock(&debug->pv_filp_lock);

	return 0;
}

static ssize_t debug_coredump_read(struct file *filep, char __user *buff,
				   size_t count, loff_t *off)
{
	struct qm35_ctx *qm35_hdl;
	struct debug *debug;
	char *cd;
	size_t cd_len = 0;

	debug = priv_from_file(filep);
	qm35_hdl = container_of(debug, struct qm35_ctx, debug);

	if (!debug->coredump_ops)
		return -ENOSYS;

	cd = debug->coredump_ops->coredump_get(debug, &cd_len);

	return simple_read_from_buffer(buff, count, off, cd, cd_len);
}

static const struct file_operations debug_enable_fops = {
	.owner = THIS_MODULE,
	.write = debug_enable_write,
	.read = debug_enable_read,
};

static const struct file_operations debug_log_level_fops = {
	.owner = THIS_MODULE,
	.write = debug_log_level_write,
	.read = debug_log_level_read
};

static const struct file_operations debug_traces_fops = {
	.owner = THIS_MODULE,
	.open = debug_traces_open,
	.release = debug_traces_release,
	.read = debug_traces_read,
	.poll = debug_traces_poll,
	.llseek = no_llseek,
};

static const struct file_operations debug_coredump_fops = {
	.owner = THIS_MODULE,
	.read = debug_coredump_read,
};

int debug_create_module_entry(struct debug *debug,
			      struct log_module *log_module)
{
	struct dentry *dir;
	struct dentry *file;

	dir = debugfs_create_dir(log_module->name, debug->fw_dir);
	if (!dir) {
		pr_err("qm35: failed to create /sys/kernel/debug/uwb0/%s\n",
		       log_module->name);
		return -1;
	}

	file = debugfs_create_file("log_level", 0666, dir, log_module,
				   &debug_log_level_fops);
	if (!file) {
		pr_err("qm35: failed to create /sys/kernel/debug/uwb0/%s/log_level\n",
		       log_module->name);
		return -1;
	}

	pr_info("qm35 debug: created /sys/kernel/debug/uwb0/%s/log_level\n",
		log_module->name);

	return 0;
}

void debug_new_trace_available(struct debug *debug)
{
	if (debug->pv_filp)
		fsnotify_modify(debug->pv_filp);

	wake_up_interruptible(&debug->wq);
}

static int debug_devid_show(struct seq_file *s, void *unused)
{
	struct debug *debug = (struct debug *)s->private;
	uint8_t soc_id[ROM_SOC_ID_LEN];
	int rc;

	if (debug->trace_ops && debug->trace_ops->get_soc_id) {
		rc = debug->trace_ops->get_soc_id(debug, soc_id);
		if (rc < 0)
			return -EIO;
		seq_printf(s, "%*phN\n", ROM_SOC_ID_LEN, soc_id);
	}
	return 0;
}

DEFINE_SHOW_ATTRIBUTE(debug_devid);

int debug_init(struct debug *debug, struct dentry *root)
{
	struct dentry *file;

	init_waitqueue_head(&debug->wq);
	mutex_init(&debug->pv_filp_lock);
	debug->pv_filp = NULL;

	debug->root_dir = debugfs_create_dir("uwb0", root);
	if (!debug->root_dir) {
		pr_err("qm35: failed to create /sys/kernel/debug/uwb0\n");
		goto unregister;
	}

	debug->fw_dir = debugfs_create_dir("fw", debug->root_dir);
	if (!debug->fw_dir) {
		pr_err("qm35: failed to create /sys/kernel/debug/uwb0/fw\n");
		goto unregister;
	}

	file = debugfs_create_file("enable", 0666, debug->fw_dir, debug,
				   &debug_enable_fops);
	if (!file) {
		pr_err("qm35: failed to create /sys/kernel/debug/uwb0/fw/enable\n");
		goto unregister;
	}

	file = debugfs_create_file("traces", 0444, debug->fw_dir, debug,
				   &debug_traces_fops);
	if (!file) {
		pr_err("qm35: failed to create /sys/kernel/debug/uwb0/fw/traces\n");
		goto unregister;
	}

	file = debugfs_create_file("coredump", 0444, debug->fw_dir, debug,
				   &debug_coredump_fops);
	if (!file) {
		pr_err("qm35: failed to create /sys/kernel/debug/uwb0/fw/coredump\n");
		goto unregister;
	}

	file = debugfs_create_file("devid", S_IRUGO, debug->fw_dir, debug,
				   &debug_devid_fops);
	if (!file) {
		pr_err("qm35: failed to create /sys/kernel/debug/uwb0/fw/devid\n");
		goto unregister;
	}

	return 0;

unregister:
	debug_deinit(debug);
	return -1;
}

void debug_deinit(struct debug *debug)
{
	wake_up_interruptible(&debug->wq);
	debugfs_remove_recursive(debug->root_dir);
}
