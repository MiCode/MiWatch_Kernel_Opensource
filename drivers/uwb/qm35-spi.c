// SPDX-License-Identifier: GPL-2.0

/*
 * This file is part of the QM35 UCI stack for linux.
 *
 * Copyright (c) 2021 Qorvo US, Inc.
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
 * QM35 UCI over HSSPI protocol
 */

#include <linux/bitfield.h>
#include <linux/interrupt.h>
#include <linux/ioctl.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/spi/spi.h>
#include <linux/spinlock.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>
#include <linux/uaccess.h>
#include <linux/firmware.h>
#include <linux/completion.h>
#ifdef CONFIG_QM35_DEBOUNCE_TIME_US
#include <linux/ktime.h>
#endif
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <qmrom.h>
#include <qmrom_spi.h>
#include <qmrom_log.h>
#include <spi_rom_protocol.h>

#include "qm35.h"
#include "uci_ioctls.h"
#include "hsspi.h"
#include "hsspi_uci.h"

#define QM_RESET_LOW_MS  2
/*
 * value found using a SALAE
 */
#define QM_BOOT_MS 450

static int qm_firmware_load(struct qm35_ctx *qm35_hdl);

static const struct file_operations uci_fops;

static const struct of_device_id qm35_dt_ids[] = {
	{ .compatible = "qorvo,qm35" },
	{},
};
MODULE_DEVICE_TABLE(of, qm35_dt_ids);

static bool use_ss_ready = true;
module_param(use_ss_ready, bool, 0444);
MODULE_PARM_DESC(use_ss_ready, "Enable/disable use of ss_ready gpio");

static bool flash_on_probe = true;
module_param(flash_on_probe, bool, 0444);
MODULE_PARM_DESC(flash_on_probe, "Flash during the module probe");

static int spi_speed_hz;
module_param(spi_speed_hz, int, 0444);
MODULE_PARM_DESC(spi_speed_hz, "SPI speed (if not set use DTS's one)");

static uint8_t qm_soc_id[ROM_SOC_ID_LEN];

/*
 * uci_open() : open operation for uci device
 *
 */
static int uci_open(struct inode *inode, struct file *file)
{
	struct miscdevice *uci_dev = file->private_data;
	struct qm35_ctx *qm35_hdl =
	    container_of(uci_dev, struct qm35_ctx, uci_dev);

	return hsspi_register(&qm35_hdl->hsspi, &qm35_hdl->uci_layer.hlayer);
}

/*
 * uci_ioctl() - ioctl operation for uci device.
 *
 */
static long uci_ioctl(struct file *filp, unsigned int cmd, unsigned long args)
{
	void __user *argp = (void __user *)args;
	struct miscdevice *uci_dev = filp->private_data;
	struct qm35_ctx *qm35_hdl =
	    container_of(uci_dev, struct qm35_ctx, uci_dev);
	int ret;

	switch (cmd) {
	case QM35_CTRL_GET_STATE:
	{
		return copy_to_user(argp, &qm35_hdl->state,
				    sizeof(qm35_hdl->state)) ? -EFAULT : 0;
	}
	case QM35_CTRL_RESET:
	{
		int irq;

		irq = (use_ss_ready && qm35_hdl->gpio_ss_rdy) ?
			gpiod_to_irq(qm35_hdl->gpio_ss_rdy) : -1;

		hsspi_stop(&qm35_hdl->hsspi);

		if (irq != -1) {
			disable_irq_nosync(irq);

			clear_bit(HSSPI_FLAGS_SS_READY, qm35_hdl->hsspi.flags);
		}

		ret = qm35_reset(qm35_hdl, QM_RESET_LOW_MS);

		msleep(QM_BOOT_MS);

		if (irq != -1) {
			enable_irq(irq);
#if 0
			if (gpiod_get_value(qm35_hdl->gpio_ss_rdy))
				hsspi_set_spi_slave_ready(&qm35_hdl->hsspi);
#endif
		}

		hsspi_start(&qm35_hdl->hsspi);

		if (ret)
			return ret;

		return copy_to_user(argp, &qm35_hdl->state,
				    sizeof(qm35_hdl->state)) ? -EFAULT : 0;
	}
	case QM35_CTRL_FW_UPLOAD:
	{
		int irq;

		irq = (use_ss_ready && qm35_hdl->gpio_ss_rdy) ?
			  gpiod_to_irq(qm35_hdl->gpio_ss_rdy) : -1;

		hsspi_stop(&qm35_hdl->hsspi);

		if (irq != -1) {
			disable_irq_nosync(irq);
			clear_bit(HSSPI_FLAGS_SS_READY, qm35_hdl->hsspi.flags);
		}

		ret = qm_firmware_load(qm35_hdl);
		msleep(QM_BOOT_MS);

		if (irq != -1) {
			enable_irq(irq);
			if (gpiod_get_value(qm35_hdl->gpio_ss_rdy))
				hsspi_set_spi_slave_ready(&qm35_hdl->hsspi);
		}

		hsspi_start(&qm35_hdl->hsspi);

		if (ret)
			return ret;

		return copy_to_user(argp, &qm35_hdl->state,
				    sizeof(qm35_hdl->state)) ? -EFAULT : 0;
	}
	default:
		dev_err(&qm35_hdl->spi->dev, "unknown ioctl %x to %s device\n",
			cmd, qm35_hdl->uci_dev.name);
		return -EINVAL;
	}
}

/*
 * uci_release() - release operation for uci device.
 *
 */
static int uci_release(struct inode *inode, struct file *filp)
{
	struct miscdevice *uci_dev = filp->private_data;
	struct qm35_ctx *qm35_hdl =
	    container_of(uci_dev, struct qm35_ctx, uci_dev);

	hsspi_unregister(&qm35_hdl->hsspi, &qm35_hdl->uci_layer.hlayer);
	return 0;
}

static ssize_t uci_read(struct file *filp, char __user *buf, size_t len,
			loff_t *off)
{
	struct miscdevice *uci_dev = filp->private_data;
	struct qm35_ctx *qm35_hdl =
		container_of(uci_dev, struct qm35_ctx, uci_dev);
	struct uci_packet *p;
	int ret;

	p = uci_layer_read(&qm35_hdl->uci_layer, len,
			   filp->f_flags & O_NONBLOCK);
	if (IS_ERR(p))
		return PTR_ERR(p);

	ret = copy_to_user(buf, p->data, p->length);
	if (!ret)
		ret = p->length;

	uci_packet_free(p);
	return ret;
}

static ssize_t uci_write(struct file *filp, const char __user *buf, size_t len,
			 loff_t *off)
{
	struct miscdevice *uci_dev = filp->private_data;
	struct qm35_ctx *qm35_hdl =
		container_of(uci_dev, struct qm35_ctx, uci_dev);
	struct uci_packet *p;
	DECLARE_COMPLETION_ONSTACK(comp);
	int ret;

	p = uci_packet_alloc(len);
	if (!p)
		return -ENOMEM;

	p->write_done = &comp;

	if (copy_from_user(p->data, buf, len)) {
		ret = -EFAULT;
		goto free;
	}

	ret = hsspi_send(&qm35_hdl->hsspi, &qm35_hdl->uci_layer.hlayer,
			 &p->blk);
	if (ret)
		goto free;

	wait_for_completion(&comp);

	ret = p->status ? p->status : len;
free:
	uci_packet_free(p);
	return ret;
}

static __poll_t uci_poll(struct file *filp, struct poll_table_struct *wait)
{
	struct miscdevice *uci_dev = filp->private_data;
	struct qm35_ctx *qm35_ctx =
		container_of(uci_dev, struct qm35_ctx, uci_dev);
	__poll_t mask = 0;

	poll_wait(filp, &qm35_ctx->uci_layer.wq, wait);

	if (uci_layer_has_data_available(&qm35_ctx->uci_layer))
		mask |= EPOLLIN;

	return mask;
}

static const struct file_operations uci_fops = {
	.owner		= THIS_MODULE,
	.open		= uci_open,
	.release	= uci_release,
	.unlocked_ioctl	= uci_ioctl,
	.read		= uci_read,
	.write		= uci_write,
	.poll		= uci_poll,
};

static irqreturn_t qm35_irq_handler(int irq, void *qm35_ctx)
{
	struct qm35_ctx *qm35_hdl = qm35_ctx;

	spin_lock(&qm35_hdl->lock);
	qm35_hdl->state = QM35_CTRL_STATE_READY;
	spin_unlock(&qm35_hdl->lock);

	disable_irq_nosync(irq);

	hsspi_set_output_data_waiting(&qm35_hdl->hsspi);

	return IRQ_HANDLED;
}

static void reenable_ss_irq(struct hsspi *hsspi)
{
	struct qm35_ctx *qm35_hdl =
		container_of(hsspi, struct qm35_ctx, hsspi);
	int irq;

	if (qm35_hdl->gpio_ss_irq)
		irq = gpiod_to_irq(qm35_hdl->gpio_ss_irq);
	else
		irq = qm35_hdl->spi->irq;

	enable_irq(irq);
}

static irqreturn_t qm35_ss_rdy_handler(int irq, void *data)
{
	struct qm35_ctx *qm35_hdl = data;
#ifdef CONFIG_QM35_DEBOUNCE_TIME_US
	static ktime_t old_time;
	ktime_t current_time;

	current_time = ktime_get();

	if (ktime_after(
		    ktime_add(old_time, CONFIG_QM35_DEBOUNCE_TIME_US * 1000),
		    current_time))
		return IRQ_HANDLED;

	old_time = current_time;
#endif
	hsspi_set_spi_slave_ready(&qm35_hdl->hsspi);

	return IRQ_HANDLED;
}

static int qm_firmware_load(struct qm35_ctx *qm35_hdl)
{
	struct spi_device *spi = qm35_hdl->spi;
	unsigned int state = qm35_get_state(qm35_hdl);
	int ret;
	uint8_t uuid[ROM_UUID_LEN];
	uint8_t lcs_state = 0;

	qm35_set_state(qm35_hdl, QM35_CTRL_STATE_FW_DOWNLOADING);

	qmrom_set_log_device(&spi->dev, LOG_WARN);

	dev_info(&spi->dev, "Get device info\n");
	ret = qmrom_get_soc_info(&spi->dev, qmrom_spi_reset_device, qm35_hdl, qm_soc_id, uuid, &lcs_state);
	if (!ret) {
		dev_info(&spi->dev, "    devid:     %*phN\n", ROM_SOC_ID_LEN, qm_soc_id);
		dev_info(&spi->dev, "    uuid:      %*phN\n", ROM_UUID_LEN, uuid);
		dev_info(&spi->dev, "    lcs_state: %hhu\n", lcs_state);
	}

	dev_info(&spi->dev, "Starting device flashing!\n");
	ret = qmrom_download_fw(spi, qmrom_spi_get_firmware, qmrom_spi_release_firmware,
							qmrom_spi_reset_device, qm35_hdl, 0, lcs_state);

	if (ret)
		dev_err(&spi->dev, "Firmware download failed!\n");
	else
		dev_info(&spi->dev, "Device flashing completed!\n");

	qm35_set_state(qm35_hdl, state);

	return ret;
}

int qm_get_soc_id(struct qm35_ctx *qm35_hdl, uint8_t *soc_id)
{
	memcpy(soc_id, qm_soc_id, ROM_SOC_ID_LEN);

	return 0;
}

/**
 * hsspi_irqs_setup() - setup all irqs needed by HSSPI
 * @qm35_ctx: pointer to &struct qm35_ctx
 *
 * SS_IRQ
 * ------
 *
 * If `ss-irq-gpios` exists in the DTS, it is used. If not, it's using
 * the `interrupts` definition from the SPI device.
 *
 * SS_READY
 * --------
 *
 * If `ss-ready-gpios` exists in the DTS and `use_ss_ready` module
 * parameter is true, it is used. Otherwise, the driver will retries
 * transfers without the RDY bit.
 *
 * Return: 0 if no error, -errno otherwise
 */
static int hsspi_irqs_setup(struct qm35_ctx *qm35_ctx)
{
	int ret;
	unsigned int ss_irq;
	unsigned long ss_irqflags;

	/* Get READY GPIO */
	qm35_ctx->gpio_ss_rdy = devm_gpiod_get_optional(
		&qm35_ctx->spi->dev, "ss-ready", GPIOD_IN);

	if (use_ss_ready && qm35_ctx->gpio_ss_rdy) {
		if (IS_ERR(qm35_ctx->gpio_ss_rdy))
			return PTR_ERR(qm35_ctx->gpio_ss_rdy);

		ret = devm_request_irq(&qm35_ctx->spi->dev,
				       gpiod_to_irq(qm35_ctx->gpio_ss_rdy),
				       &qm35_ss_rdy_handler,
				       IRQF_TRIGGER_RISING,
				       "hsspi-ss-rdy", qm35_ctx);
		if (ret)
			return ret;

		/* we can have miss an edge */
		if (gpiod_get_value(qm35_ctx->gpio_ss_rdy))
			hsspi_set_spi_slave_ready(&qm35_ctx->hsspi);
	}

	/* get SS_IRQ GPIO */
	qm35_ctx->gpio_ss_irq = devm_gpiod_get_optional(
		&qm35_ctx->spi->dev, "ss-irq", GPIOD_IN);

	if (qm35_ctx->gpio_ss_irq) {
		if (IS_ERR(qm35_ctx->gpio_ss_irq))
			return PTR_ERR(qm35_ctx->gpio_ss_irq);

		ss_irq = gpiod_to_irq(qm35_ctx->gpio_ss_irq);
		ss_irqflags = IRQF_TRIGGER_HIGH;
	} else {
		ss_irq = qm35_ctx->spi->irq;
		ss_irqflags = irq_get_trigger_type(ss_irq);
	}

	qm35_ctx->hsspi.odw_cleared = reenable_ss_irq;

	ret = devm_request_irq(&qm35_ctx->spi->dev, ss_irq,
			       &qm35_irq_handler, ss_irqflags,
			       "hsspi-ss-irq", qm35_ctx);
	if (ret)
		return ret;

	return 0;
}

static int uwb_power_on(struct device *pdev)
{
	int ret = 0;
	struct regulator *uwb_vio;
	uwb_vio = regulator_get(pdev, "uwbhal_1p8");
	if (IS_ERR(uwb_vio)) {
		ret = -1;
		dev_err(pdev, "Regulator get failed vio ret=%d\n", ret);
		return ret;
	}

	if (regulator_count_voltages(uwb_vio) > 0) {
		ret = regulator_set_voltage(uwb_vio, 1800000, 1800000);
		if (ret) {
 			dev_err(pdev, "Regulator set_vtg failed vio ret=%d\n", ret);
			goto reg_vio_put;
		}
	}

	ret = regulator_enable(uwb_vio);
	if (ret) {
		dev_err(pdev, "Regulator vio enable failed ret=%d\n", ret);
		return ret;
	}
	return ret;

reg_vio_put:
	regulator_put(uwb_vio);
	return ret;
}
static int uwb_power28v_on(struct device *pdev)
{
	int ret = 0;
	struct regulator *uwb_vio;
	uwb_vio = regulator_get(pdev, "uwbhal_2p8");
	if (IS_ERR(uwb_vio)) {
		ret = -1;
		dev_err(pdev, "Regulator get failed vio ret=%d\n", ret);
		return ret;
	}

	if (regulator_count_voltages(uwb_vio) > 0) {
		ret = regulator_set_voltage(uwb_vio, 2800000, 2800000);
		if (ret) {
 			dev_err(pdev, "Regulator set_vtg failed vio ret=%d\n", ret);
			goto reg_vio_put;
		}
	}

	ret = regulator_enable(uwb_vio);
	if (ret) {
		dev_err(pdev, "Regulator vio enable failed ret=%d\n", ret);
		return ret;
	}
	return ret;

reg_vio_put:
	regulator_put(uwb_vio);
	return ret;
}

int cs_gpio;
static int qm35_probe(struct spi_device *spi)
{
	struct qm35_ctx *qm35_ctx;
	struct miscdevice *uci_misc;
	int ret = 0;
	int vdd2_enable;
	int vdd3_enable;

	if (spi_speed_hz) {
		spi->max_speed_hz = spi_speed_hz;

		ret = spi_setup(spi);
		if (ret) {
			dev_err(&spi->dev, "spi_setup: requested spi speed=%d ret=%d\n",
				spi_speed_hz, ret);
			return ret;
		}
	}

	vdd2_enable = of_get_named_gpio(spi->dev.of_node, "vdd2-enable-gpio", 0);
	vdd3_enable = of_get_named_gpio(spi->dev.of_node, "vdd3-enable-gpio", 0);
    ret = gpio_request(vdd2_enable, "VDD2 POWER");
	if (ret){
		pr_err("[UWB]: failed to request VDD2 POWER, ret:%d", ret);
		goto err_exit;
	}
	ret = gpio_direction_output(vdd2_enable, 1);
	if (ret) {
		pr_err("[UWB]: failed set VDD2 POWER as input mode, ret:%d", ret);
		goto err_exit;
	}
	gpio_export(vdd2_enable,true);

   ret = gpio_request(vdd3_enable, "VDD3 POWER");
	if (ret){
		pr_err("[UWB]: failed to request VDD3 POWER, ret:%d", ret);
		goto err_exit;
	}
	ret = gpio_direction_output(vdd3_enable, 1);
	if (ret) {
		pr_err("[UWB]: failed set VDD3 POWER as input mode, ret:%d", ret);
		goto err_exit;
	}
	gpio_export(vdd3_enable,true);

	cs_gpio = of_get_named_gpio(spi->dev.of_node, "cs-gpio", 0);
	ret = gpio_request(cs_gpio, "CS GPIO");
	if (ret){
		pr_err("[UWB]: failed to request CS GPIO, ret:%d", ret);
		goto err_exit;
	}
	ret = gpio_direction_output(cs_gpio, 0);
	if (ret) {
		pr_err("[UWB]: failed set CS GPIO as input mode, ret:%d", ret);
		goto err_exit;
	}
	gpio_export(cs_gpio,true);
	
	uwb_power_on(&spi->dev);
	uwb_power28v_on(&spi->dev);	
	qm35_ctx = devm_kzalloc(&spi->dev, sizeof(struct qm35_ctx),
				   GFP_KERNEL);
	if (!qm35_ctx)
		return -ENOMEM;

	qm35_ctx->gpio_reset = devm_gpiod_get_optional(&spi->dev, "reset",
						       GPIOD_OUT_LOW);
	if (IS_ERR(qm35_ctx->gpio_reset)) {
		ret = PTR_ERR(qm35_ctx->gpio_reset);
		return ret;
	}

	qm35_ctx->spi = spi;
	spin_lock_init(&qm35_ctx->lock);

	uci_misc	 = &qm35_ctx->uci_dev;
	uci_misc->minor	 = MISC_DYNAMIC_MINOR;
	uci_misc->name	 = UCI_DEV_NAME;
	uci_misc->fops	 = &uci_fops;
	uci_misc->parent = &spi->dev;

	ret = misc_register(&qm35_ctx->uci_dev);
	if (ret) {
		dev_err(&spi->dev, "Failed to register uci device\n");
		return ret;
	}

	dev_info(&spi->dev, "Registered: [%s] misc device\n", uci_misc->name);

	qm35_ctx->state = QM35_CTRL_STATE_UNKNOWN;

	dev_set_drvdata(&spi->dev, qm35_ctx);

	if (flash_on_probe) {
		ret = qm_firmware_load(qm35_ctx);
		if (ret)
			goto unregister;
	}

	ret = hsspi_init(&qm35_ctx->hsspi, spi);
	if (ret)
		goto unregister;

	ret = uci_layer_init(&qm35_ctx->uci_layer);
	if (ret)
		goto hsspi_deinit;

	ret = debug_init(&qm35_ctx->debug, NULL);
	if (ret)
		goto uci_layer_deinit;

	ret = coredump_layer_init(&qm35_ctx->coredump_layer, &qm35_ctx->debug);
	if (ret)
		goto unregister;

	ret = log_layer_init(&qm35_ctx->log_layer, &qm35_ctx->debug);
	if (ret)
		goto debug_deinit;

	ret = hsspi_register(&qm35_ctx->hsspi, &qm35_ctx->coredump_layer.hlayer);
	if (ret)
		goto unregister;

	ret = hsspi_register(&qm35_ctx->hsspi, &qm35_ctx->log_layer.hlayer);
	if (ret)
		goto log_layer_deinit;

	msleep(QM_BOOT_MS);

	ret = hsspi_irqs_setup(qm35_ctx);
	if (ret)
		goto hsspi_unregister;

	hsspi_start(&qm35_ctx->hsspi);

	dev_info(&spi->dev, "QM35 spi driver probed\n");
	return 0;

hsspi_unregister:
	hsspi_unregister(&qm35_ctx->hsspi, &qm35_ctx->log_layer.hlayer);
log_layer_deinit:
	log_layer_deinit(&qm35_ctx->log_layer);
debug_deinit:
	debug_deinit(&qm35_ctx->debug);
uci_layer_deinit:
	uci_layer_deinit(&qm35_ctx->uci_layer);
hsspi_deinit:
	hsspi_deinit(&qm35_ctx->hsspi);
unregister:
	misc_deregister(&qm35_ctx->uci_dev);
err_exit:
	return -ENODEV;	
	return ret;
}

static int qm35_remove(struct spi_device *spi)
{
	struct qm35_ctx *qm35_hdl = dev_get_drvdata(&spi->dev);

	if (qm35_hdl) {
		hsspi_stop(&qm35_hdl->hsspi);
		hsspi_unregister(&qm35_hdl->hsspi, &qm35_hdl->log_layer.hlayer);
		hsspi_unregister(&qm35_hdl->hsspi, &qm35_hdl->coredump_layer.hlayer);
		log_layer_deinit(&qm35_hdl->log_layer);
		debug_deinit(&qm35_hdl->debug);
		coredump_layer_deinit(&qm35_hdl->coredump_layer);
		uci_layer_deinit(&qm35_hdl->uci_layer);
		hsspi_deinit(&qm35_hdl->hsspi);
		misc_deregister(&qm35_hdl->uci_dev);
		dev_info(&spi->dev, "Deregistered: [%s] misc device",
			 qm35_hdl->uci_dev.name);
		dev_set_drvdata(&spi->dev, NULL);
	}
	return 0;
}

static struct spi_driver qm35_spi_driver = {
	.driver = {
		.name           = "qm35",
		.of_match_table = of_match_ptr(qm35_dt_ids),
	},
	.probe =	qm35_probe,
	.remove =	qm35_remove,
};
module_spi_driver(qm35_spi_driver);

MODULE_AUTHOR("Qorvo US, Inc.");
MODULE_DESCRIPTION("QM35 SPI device interface");
MODULE_LICENSE("GPL");
