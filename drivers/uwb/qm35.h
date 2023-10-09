/* SPDX-License-Identifier: GPL-2.0 */

#ifndef __QM35_H___
#define __QM35_H___

#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/spinlock.h>
#include <linux/miscdevice.h>

#include "uci_ioctls.h"
#include "hsspi.h"
#include "hsspi_uci.h"
#include "hsspi_coredump.h"
#include "hsspi_log.h"
#include "debug.h"

extern int cs_gpio;
enum {
	UL_RESERVED,
	UL_BOOT_FLASH,
	UL_UCI_APP,
	UL_COREDUMP,
	UL_LOG,
};

/**
 * struct qm35_ctx - QM35 driver context
 *
 */
struct qm35_ctx {
	unsigned int		state;
	struct miscdevice	uci_dev;
	struct spi_device	*spi;
	struct gpio_desc	*gpio_reset;
	struct gpio_desc	*gpio_ss_rdy;
	struct gpio_desc	*gpio_ss_irq;
	spinlock_t		lock;
	bool			out_data_wait;
	bool			out_active;
	bool			soc_error;
	struct hsspi		hsspi;
	struct uci_layer	uci_layer;
	struct coredump_layer	coredump_layer;
	struct log_layer	log_layer;
	struct debug		debug;
};

static inline unsigned int qm35_get_state(struct qm35_ctx *qm35_hdl)
{
	return qm35_hdl->state;
}

static inline void qm35_set_state(struct qm35_ctx *qm35_hdl, int state)
{
	unsigned long flags;

	spin_lock_irqsave(&qm35_hdl->lock, flags);
	qm35_hdl->state = state;
	spin_unlock_irqrestore(&qm35_hdl->lock, flags);
}

static inline int qm35_reset(struct qm35_ctx *qm35_hdl, int timeout_ms)
{
	if (qm35_hdl->gpio_reset) {
		qm35_set_state(qm35_hdl, QM35_CTRL_STATE_RESET);
		gpiod_set_value(qm35_hdl->gpio_reset, 1);
		usleep_range(timeout_ms * 1000, timeout_ms * 1000);
		gpiod_set_value(qm35_hdl->gpio_reset, 0);
		qm35_set_state(qm35_hdl, QM35_CTRL_STATE_UNKNOWN);
		return 0;
	}

	return -ENODEV;
}

int qm_get_soc_id(struct qm35_ctx *qm35_hdl, uint8_t *soc_id);

#endif /* __QM35_H___ */
