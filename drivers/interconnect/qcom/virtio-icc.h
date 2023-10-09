/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef __LINUX_VIRTIO_ICC_H__
#define __LINUX_VIRTIO_ICC_H__

#include <linux/types.h>
#include <linux/virtio_ids.h>
#include <linux/virtio_config.h>
#include <linux/virtio_types.h>

#define MAX_LINKS	16

/*
 * Provider format
 * vdev - The virtIO device specific for the provider
 * vq - The virtqueue used to send/receive messages to/from the device
 * rsp_avail - The completion instance indicating the device has notified the driver
 * lock - The mutex needed for locking access to the virtqueue
 */
struct virtio_icc {
	struct virtio_device	*vdev;
	struct virtqueue	*vq;
	struct completion	rsp_avail;
	struct mutex		lock;
};

/*
 * Request/response message format
 * src_name - The name for the interconnect source/master
 * dst_name - The name for the interconnect destination/slave
 * avg_bw - The instantaneous b/w in bytes per second
 * dst_bw - The peak b/w in bytes per second
 * latency - Latency for the request in nano seconds
 * result - The result returned from the device to the driver
 *
 */
struct virtio_icc_msg {
	u8 src_name[32];
	u8 dst_name[32];
	u64 avg_bw;
	u64 peak_bw;
	u32 latency;
	__virtio32 result;
};

/*
 * Node format
 * name - The name of the ICC node
 * links - The node ID of the links this ICC node is connected to
 * id - The ID for this ICC node
 * num_links - The number of links for this ICC node
 */
struct virtio_icc_node {
	const char *name;
	u16 links[MAX_LINKS];
	u16 id;
	u16 num_links;
};

/* Virtio ID of interconnect */
#define VIRTIO_ID_INTERCONNECT    37

#endif /* __LINUX_VIRTIO_ICC_H__ */
