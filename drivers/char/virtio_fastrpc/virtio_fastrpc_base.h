/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */
#ifndef __VIRTIO_FASTRPC_BASE_H__
#define __VIRTIO_FASTRPC_BASE_H__

#include <linux/cdev.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/virtio.h>


#define MINOR_NUM_DEV	0
#define MINOR_NUM_SECURE_DEV	1

#define PID_SIZE		10
#define FASTRPC_MSG_MAX	256

struct fastrpc_dsp_capabilities {
	uint32_t is_cached;	/* ! Flag if dsp attributes are cached */
	uint32_t dsp_attributes[FASTRPC_MAX_DSP_ATTRIBUTES];
};

struct fastrpc_channel_ctx {
	int secure;
	bool unsigned_support;
	struct fastrpc_dsp_capabilities dsp_cap_kernel;
};

struct virt_fastrpc_vq {
	/* protects vq */
	spinlock_t vq_lock;
	struct virtqueue *vq;
};

struct virt_fastrpc_msg;

struct fastrpc_apps {
	struct virtio_device *vdev;
	struct virt_fastrpc_vq rvq;
	struct virt_fastrpc_vq svq;
	void *rbufs;
	void *sbufs;
	unsigned int order;
	unsigned int num_bufs;
	unsigned int buf_size;
	unsigned int num_channels;
	int last_sbuf;

	bool has_invoke_attr;
	bool has_invoke_crc;
	bool has_mmap;
	bool has_control;

	struct device *dev;
	struct cdev cdev;
	struct class *class;
	dev_t dev_no;

	struct fastrpc_channel_ctx *channel;
	struct dentry *debugfs_root;
	const struct file_operations *debugfs_fops;
	spinlock_t msglock;
	struct virt_fastrpc_msg *msgtable[FASTRPC_MSG_MAX];
};

#endif /*__VIRTIO_FASTRPC_BASE_H__*/
