// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/device.h>
#include <linux/interconnect.h>
#include <linux/interconnect-provider.h>
#include <linux/virtio.h>
#include <linux/scatterlist.h>
#include <linux/delay.h>
#include <linux/completion.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

#include "virtio-icc.h"

#define SLAVE_TCU	632
static struct virtio_icc *vicc;
static struct virtio_icc_node *vnodes[SLAVE_TCU + 1];

static int virtio_icc_aggregate(struct icc_node *node, u32 tag, u32 avg_bw,
		u32 peak_bw, u32 *agg_avg, u32 *agg_peak)
{
	*agg_avg = avg_bw;
	*agg_peak = peak_bw;

	return 0;
}

static int virtio_icc_set(struct icc_node *src, struct icc_node *dst)
{
	struct virtio_icc_msg *req, *rsp;
	struct scatterlist sg[1];
	unsigned int len;
	int ret = 0;

	req = kzalloc(sizeof(struct virtio_icc_msg), GFP_KERNEL);
	if (!req)
		return -ENOMEM;

	strlcpy(req->src_name, src->name, sizeof(req->src_name));
	strlcpy(req->dst_name, dst->name, sizeof(req->dst_name));
	req->avg_bw = dst->avg_bw * 1000;
	req->peak_bw = dst->peak_bw * 1000;
	req->latency = 0;
	sg_init_one(sg, req, sizeof(*req));

	mutex_lock(&vicc->lock);

	ret = virtqueue_add_outbuf(vicc->vq, sg, 1, req, GFP_KERNEL);
	if (ret) {
		pr_err("%s: failed to add output buffer\n", src->name);
		goto out;
	}

	virtqueue_kick(vicc->vq);

	wait_for_completion(&vicc->rsp_avail);

	rsp = virtqueue_get_buf(vicc->vq, &len);
	if (!rsp) {
		pr_err("%s: failed to get virtqueue buffer\n", src->name);
		ret = -EIO;
		goto out;
	}

	ret = virtio32_to_cpu(vicc->vdev, rsp->result);
out:
	mutex_unlock(&vicc->lock);
	kfree(req);

	return ret;
}

static void virtio_icc_isr(struct virtqueue *vq)
{
	struct virtio_icc *vicc = vq->vdev->priv;

	complete(&vicc->rsp_avail);
}

static int virtio_icc_init_vqs(struct virtio_icc *vicc)
{
	struct virtqueue *vqs[1];
	vq_callback_t *cbs[] = { virtio_icc_isr };
	static const char * const names[] = { "interconnect" };
	int ret;

	ret = virtio_find_vqs(vicc->vdev, 1, vqs, cbs, names, NULL);
	if (ret)
		return ret;

	vicc->vq = vqs[0];

	return 0;
}

static int virtio_icc_allocate_nodes(struct virtio_device *vdev)
{
	struct device_node *parent_node, *node;
	int ret = 0;
	u16 id;

	parent_node = vdev->dev.parent->of_node;
	for_each_available_child_of_node(parent_node, node) {
		ret = of_property_read_u16(node, "node-id", &id);
		if (ret)
			return -EINVAL;

		vnodes[id] = devm_kzalloc(&vdev->dev,
				sizeof(struct virtio_icc_node), GFP_KERNEL);
		if (!vnodes[id])
			return -ENOMEM;

		vnodes[id]->id = id;
		ret = of_property_read_string(node, "node-name",
				&(vnodes[id]->name));
		if (ret)
			return -EINVAL;

		ret = of_property_read_u16(node, "node-num-links",
					&(vnodes[id]->num_links));
		if (ret)
			return -EINVAL;

		if (vnodes[id]->num_links > 0) {
			ret = of_property_read_u16_array(node, "node-links",
						vnodes[id]->links,
						vnodes[id]->num_links);
			if (ret)
				return -EINVAL;
		}
	}

	return ret;
}

static int virtio_icc_setup_provider(struct virtio_device *vdev)
{
	struct icc_onecell_data *data;
	struct icc_provider *provider;
	struct icc_node *node;
	size_t num_nodes, i;
	int ret;

	ret = virtio_icc_allocate_nodes(vdev);
	if (ret) {
		dev_err(&vdev->dev, "Failed to create interconnect nodes\n");
		return ret;
	}

	num_nodes = ARRAY_SIZE(vnodes);

	provider = devm_kzalloc(&vdev->dev, sizeof(*provider), GFP_KERNEL);
	if (!provider)
		return -ENOMEM;

	data = devm_kcalloc(&vdev->dev, num_nodes, sizeof(*node), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	provider->dev = vdev->dev.parent;
	provider->set = virtio_icc_set;
	provider->aggregate = virtio_icc_aggregate;
	provider->xlate = of_icc_xlate_onecell;
	INIT_LIST_HEAD(&provider->nodes);
	provider->data = data;

	ret = icc_provider_add(provider);
	if (ret) {
		dev_err(&vdev->dev, "Failed to add an interconnect provider\n");
		goto err;
	}

	for (i = 0; i < num_nodes; i++) {
		size_t j;

		if (!vnodes[i])
			continue;

		node = icc_node_create(vnodes[i]->id);
		if (IS_ERR(node)) {
			dev_err(&vdev->dev, "%s: Creating Node %d failed\n", i);
			ret = PTR_ERR(node);
			goto err;
		}

		node->name = vnodes[i]->name;
		node->data = vnodes[i];
		icc_node_add(node, provider);

		dev_info(&vdev->dev, "registered node %pK %s %d\n", node,
			vnodes[i]->name, node->id);

		for (j = 0; j < vnodes[i]->num_links; j++)
			icc_link_create(node, vnodes[i]->links[j]);

		data->nodes[i] = node;
	}
	data->num_nodes = num_nodes;

	return 0;
err:
	list_for_each_entry(node, &provider->nodes, node_list) {
		icc_node_del(node);
		icc_node_destroy(node->id);
	}

	icc_provider_del(provider);
	return ret;
}

static int virtio_icc_probe(struct virtio_device *vdev)
{
	int ret;

	if (vicc)
		ret = -EPERM;

	vicc = devm_kzalloc(&vdev->dev, sizeof(struct virtio_icc),
			GFP_KERNEL);
	if (!vicc)
		return -ENOMEM;

	vdev->priv = vicc;
	vicc->vdev = vdev;
	mutex_init(&vicc->lock);
	init_completion(&vicc->rsp_avail);

	ret = virtio_icc_init_vqs(vicc);
	if (ret) {
		dev_err(&vdev->dev, "Failed to initialize virtqueue\n");
		return ret;
	}

	virtio_device_ready(vdev);

	ret = virtio_icc_setup_provider(vdev);
	if (ret) {
		dev_err(&vdev->dev, "Failed to setup interconnect provider\n");
		vdev->config->del_vqs(vdev);
		return ret;
	}

	dev_info(&vdev->dev, "Registered VirtIO ICC\n");

	return 0;
}

static void virtio_icc_remove(struct virtio_device *vdev)
{
	struct virtio_icc *vicc = vdev->priv;
	void *buf;

	vdev->config->reset(vdev);
	while ((buf = virtqueue_detach_unused_buf(vicc->vq)) != NULL)
		kfree(buf);
	vdev->config->del_vqs(vdev);
}

static const struct virtio_device_id id_table[] = {
	{ VIRTIO_ID_INTERCONNECT, VIRTIO_DEV_ANY_ID },
	{ 0 },
};

static unsigned int features[] = {
};

static struct virtio_driver virtio_icc_driver = {
	.feature_table			= features,
	.feature_table_size		= ARRAY_SIZE(features),
	.driver.name			= KBUILD_MODNAME,
	.driver.owner			= THIS_MODULE,
	.id_table			= id_table,
	.probe				= virtio_icc_probe,
	.remove				= virtio_icc_remove,
};

static int __init virtio_icc_init(void)
{
	return register_virtio_driver(&virtio_icc_driver);
}

static void __exit virtio_icc_exit(void)
{
	unregister_virtio_driver(&virtio_icc_driver);
}
subsys_initcall(virtio_icc_init);
module_exit(virtio_icc_exit);

MODULE_DEVICE_TABLE(virtio, id_table);
MODULE_DESCRIPTION("VirtIO ICC driver");
MODULE_LICENSE("GPL v2");
MODULE_SOFTDEP("pre: virtio_mmio");
