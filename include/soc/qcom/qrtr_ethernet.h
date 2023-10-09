/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (c) 2020, The Linux Foundation. All rights reserved. */

#ifndef _QRTR_ETHERNET_H
#define _QRTR_ETHERNET_H

#include <linux/skbuff.h>

/**
 * qrtr_ethernet_cb_info - struct to pass transport layer information to qrtr
 * @eth_send: function pointer to send qrtr packets to transport layer
 */
struct qrtr_ethernet_cb_info {
	int (*eth_send)(struct sk_buff *skb);
};

/**
 * eth_adapt_result - struct to pass on buffer from external ap to qrtr
 * buf_addr - address of the buffer that holds the data from external ap
 * bytes_xferd - size of the above buffer
 */
struct eth_adapt_result {
	void *buf_addr;
	size_t bytes_xferd;
};

#if IS_ENABLED(CONFIG_QRTR_ETHERNET) || IS_ENABLED(CONFIG_QTI_QRTR_ETHERNET)
void qcom_ethernet_init_cb(struct qrtr_ethernet_cb_info *cbinfo);
void qcom_ethernet_qrtr_dl_cb(struct eth_adapt_result *eth_res);
void qcom_ethernet_qrtr_status_cb(unsigned int event);
#else
static inline void qcom_ethernet_init_cb(struct qrtr_ethernet_cb_info *cbinfo)
{
}
static inline void qcom_ethernet_qrtr_dl_cb(struct eth_adapt_result *eth_res) {}
static inline void qcom_ethernet_qrtr_status_cb(unsigned int event) {}
#endif /* CONFIG_QRTR_ETHERNET or CONFIG_QTI_QRTR_ETHERNET */

#endif /* _QRTR_ETHERNET_H */
