// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 */

#include <linux/of.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/platform_device.h>
#include <soc/qcom/subsystem_notif.h>
#include <linux/rpmsg/qcom_glink.h>
#include <linux/rpmsg.h>
#include <linux/ipc_logging.h>
#include <soc/qcom/msm_glink_ssr.h>

#define MSM_SSR_LOG_PAGE_CNT 4
static void *ssr_ilc;
static DEFINE_MUTEX(ssr_lock);

#define MSM_SSR_INFO(x, ...) ipc_log_string(ssr_ilc, x, ##__VA_ARGS__)

#define MSM_SSR_ERR(dev, x, ...)				       \
do {								       \
	dev_err(dev, "[%s]: "x, __func__, ##__VA_ARGS__);	       \
	ipc_log_string(ssr_ilc, "[%s]: "x, __func__, ##__VA_ARGS__);   \
} while (0)

#define GLINK_SSR_DO_CLEANUP	0
#define GLINK_SSR_CLEANUP_DONE	1
#define GLINK_SSR_PRIORITY	1
#define GLINK_SSR_REPLY_TIMEOUT	HZ

struct do_cleanup_msg {
	__le32 version;
	__le32 command;
	__le32 seq_num;
	__le32 name_len;
	char name[32];
};

struct cleanup_done_msg {
	__le32 version;
	__le32 response;
	__le32 seq_num;
};

struct glink_ssr_nb {
	struct list_head list;
	struct glink_ssr *ssr;
	void *ssr_register_handle;

	const char *glink_label;
	const char *ssr_label;

	struct notifier_block nb;
};

struct glink_ssr {
	struct device *dev;
	struct rpmsg_endpoint *ept;

	struct list_head notify_list;

	u32 seq_num;
	struct completion completion;
	struct work_struct unreg_work;
	struct kref refcount;
};

static struct rpmsg_endpoint *rpm_ept;
static u32 sequence_num;

static void glink_ssr_release(struct kref *ref)
{
	struct glink_ssr *ssr = container_of(ref, struct glink_ssr,
					     refcount);
	struct glink_ssr_nb *nb, *tmp;

	list_for_each_entry_safe(nb, tmp, &ssr->notify_list, list)
		kfree(nb);

	kfree(ssr);
}

static void glink_ssr_ssr_unreg_work(struct work_struct *work)
{
	struct glink_ssr *ssr = container_of(work, struct glink_ssr,
						unreg_work);
	struct glink_ssr_nb *nb, *tmp;

	list_for_each_entry_safe(nb, tmp, &ssr->notify_list, list) {
		subsys_notif_unregister_notifier(nb->ssr_register_handle,
						&nb->nb);
	}
	kref_put(&ssr->refcount, glink_ssr_release);
}

#if defined(CONFIG_DEEPSLEEP) && defined(CONFIG_RPMSG_QCOM_GLINK_RPM)
void glink_ssr_notify_rpm(void)
{
	struct do_cleanup_msg msg;
	int ret;

	sequence_num = sequence_num + 1;
	memset(&msg, 0, sizeof(msg));
	msg.command = cpu_to_le32(GLINK_SSR_DO_CLEANUP);
	msg.seq_num = cpu_to_le32(sequence_num);
	msg.name_len = cpu_to_le32(strlen("apss"));
	strlcpy(msg.name, "apss", sizeof(msg.name));

	MSM_SSR_INFO("%s: notify of %s Deep Sleep seq_num:%d\n",
		   "rpm", "apss", sequence_num);

	ret = rpmsg_send(rpm_ept, &msg, sizeof(msg));
	if (ret)
		pr_err("fail to send do cleanup to rpm for APSS SSR %d\n",
			ret);
}
EXPORT_SYMBOL(glink_ssr_notify_rpm);
#endif

static int glink_ssr_ssr_cb(struct notifier_block *this,
			    unsigned long code, void *data)
{
	struct glink_ssr_nb *nb = container_of(this, struct glink_ssr_nb, nb);
	struct glink_ssr *ssr = nb->ssr;
	struct device *dev;
	struct do_cleanup_msg msg;
	int ret;
	const char *name;

	kref_get(&ssr->refcount);
	mutex_lock(&ssr_lock);
	dev = ssr->dev;
	if (!dev || !ssr->ept)
		goto out;

	if (code == SUBSYS_AFTER_SHUTDOWN || code == SUBSYS_POWERUP_FAILURE ||
		code == SUBSYS_AFTER_DS_ENTRY || code == SUBSYS_DS_EXIT_FAIL) {

		name = dev->parent->of_node->name;
		if (!strcmp(name, "rpm-glink")) {
			if (ssr->seq_num < sequence_num)
				ssr->seq_num = sequence_num;
			sequence_num = sequence_num + 1;
		}

		ssr->seq_num++;

		reinit_completion(&ssr->completion);

		memset(&msg, 0, sizeof(msg));
		msg.command = cpu_to_le32(GLINK_SSR_DO_CLEANUP);
		msg.seq_num = cpu_to_le32(ssr->seq_num);
		msg.name_len = cpu_to_le32(strlen(nb->glink_label));
		strlcpy(msg.name, nb->glink_label, sizeof(msg.name));

		MSM_SSR_INFO("%s: notify of %s seq_num:%d\n",
			   dev->parent->of_node->name, nb->glink_label,
			   ssr->seq_num);

		ret = rpmsg_send(ssr->ept, &msg, sizeof(msg));
		if (ret) {
			MSM_SSR_ERR(dev, "fail to send do cleanup to %s %d\n",
				  nb->ssr_label, ret);
			goto out;
		}

		ret = wait_for_completion_timeout(&ssr->completion, HZ);
		if (!ret)
			MSM_SSR_ERR(dev, "timeout waiting for cleanup resp\n");
	}
out:
	mutex_unlock(&ssr_lock);
	kref_put(&ssr->refcount, glink_ssr_release);
	return NOTIFY_DONE;
}

static int glink_ssr_callback(struct rpmsg_device *rpdev,
			      void *data, int len, void *priv, u32 addr)
{
	struct cleanup_done_msg *msg = data;
	struct glink_ssr *ssr = dev_get_drvdata(&rpdev->dev);

	if (len < sizeof(*msg)) {
		MSM_SSR_ERR(ssr->dev, "message too short\n");
		return -EINVAL;
	}

	if (le32_to_cpu(msg->version) != 0) {
		MSM_SSR_ERR(ssr->dev, "invalid version\n");
		return -EINVAL;
	}

	if (le32_to_cpu(msg->response) != GLINK_SSR_CLEANUP_DONE)
		return 0;

	if (le32_to_cpu(msg->seq_num) != ssr->seq_num) {
		MSM_SSR_ERR(ssr->dev, "invalid response sequence number %d\n",
			  msg->seq_num);
		return -EINVAL;
	}

	complete(&ssr->completion);

	MSM_SSR_INFO("%s: received seq_num:%d\n",
		    ssr->dev->parent->of_node->name,
		    le32_to_cpu(msg->seq_num));

	return 0;
}

static void glink_ssr_init_notify(struct glink_ssr *ssr)
{
	struct device *dev = ssr->dev;
	struct device_node *node;
	struct glink_ssr_nb *nb;
	void *handle;
	int ret;
	struct of_phandle_iterator it;
	int err;

	of_for_each_phandle(&it, err, dev->of_node, "qcom,notify-edges",
			    NULL, 0) {
		node = it.node;

		nb = kzalloc(sizeof(*nb), GFP_KERNEL);
		if (!nb) {
			of_node_put(node);
			return;
		}

		ret = of_property_read_string(node, "label", &nb->ssr_label);
		if (ret < 0)
			nb->ssr_label = node->name;

		ret = of_property_read_string(node, "qcom,glink-label",
					      &nb->glink_label);
		if (ret < 0) {
			MSM_SSR_ERR(dev, "no qcom,glink-label for %s\n",
				  nb->ssr_label);
			of_node_put(node);
			kfree(nb);
			continue;
		}

		nb->nb.notifier_call = glink_ssr_ssr_cb;
		nb->nb.priority = GLINK_SSR_PRIORITY;
		nb->ssr = ssr;

		handle = subsys_notif_register_notifier(nb->ssr_label, &nb->nb);
		if (IS_ERR_OR_NULL(handle)) {
			MSM_SSR_ERR(dev, "register fail for %s SSR notifier\n",
				  nb->ssr_label);
			of_node_put(node);
			kfree(nb);
			continue;
		}

		nb->ssr_register_handle = handle;
		list_add_tail(&nb->list, &ssr->notify_list);
	}
}

static int glink_ssr_probe(struct rpmsg_device *rpdev)
{
	struct glink_ssr *ssr;
	struct device_node *node;
	struct device *dev;

	ssr = kzalloc(sizeof(*ssr), GFP_KERNEL);
	if (!ssr)
		return -ENOMEM;

	INIT_LIST_HEAD(&ssr->notify_list);
	init_completion(&ssr->completion);
	INIT_WORK(&ssr->unreg_work, glink_ssr_ssr_unreg_work);
	kref_init(&ssr->refcount);

	ssr->dev = &rpdev->dev;
	ssr->ept = rpdev->ept;

	dev = rpdev->dev.parent;
	node = dev->of_node;
	if (!strcmp(node->name, "rpm-glink")) {
		sequence_num = 0;
		rpm_ept = rpdev->ept;
	}

	ssr_ilc = ipc_log_context_create(MSM_SSR_LOG_PAGE_CNT,
				   "glink_ssr", 0);

	glink_ssr_init_notify(ssr);

	dev_set_drvdata(&rpdev->dev, ssr);

	return 0;
}

static void glink_ssr_remove(struct rpmsg_device *rpdev)
{
	struct glink_ssr *ssr = dev_get_drvdata(&rpdev->dev);

	mutex_lock(&ssr_lock);
	ssr->dev = NULL;
	ssr->ept = NULL;
	mutex_unlock(&ssr_lock);

	dev_set_drvdata(&rpdev->dev, NULL);
	schedule_work(&ssr->unreg_work);
}

static const struct rpmsg_device_id glink_ssr_match[] = {
	{ "glink_ssr" },
	{}
};

static struct rpmsg_driver glink_ssr_driver = {
	.probe = glink_ssr_probe,
	.remove = glink_ssr_remove,
	.callback = glink_ssr_callback,
	.id_table = glink_ssr_match,
	.drv = {
		.name = "msm_glink_ssr",
	},
};
module_rpmsg_driver(glink_ssr_driver);

MODULE_ALIAS("rpmsg:msm_glink_ssr");
MODULE_DESCRIPTION("MSM GLINK SSR notifier");
MODULE_LICENSE("GPL v2");
