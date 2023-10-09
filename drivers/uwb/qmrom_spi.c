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
 * QM35 FW ROM protocol SPI ops
 */

#include <linux/spi/spi.h>

#include <qmrom_spi.h>
#include <spi_rom_protocol.h>

#include "qm35.h"

int qmrom_spi_transfer(void *handle, char *rbuf, const char *wbuf, size_t size)
{
	struct spi_device *spi = (struct spi_device *)handle;

	struct spi_transfer xfer[] = {
		{
			.tx_buf = wbuf,
			.rx_buf = rbuf,
			.len = size,
			.speed_hz = DEFAULT_SPI_CLOCKRATE,
		},
	};

	return spi_sync_transfer(spi, xfer, ARRAY_SIZE(xfer));
}

int qmrom_spi_set_cs_level(void *handle, int level)
{
	struct spi_device *spi = (struct spi_device *)handle;
	uint8_t dummy = 0;

	struct spi_transfer xfer[] = {
		{
			.tx_buf = &dummy,
			.len = 1,
			.cs_change = !level,
			.speed_hz = DEFAULT_SPI_CLOCKRATE,

		},
	};

	return spi_sync_transfer(spi, xfer, ARRAY_SIZE(xfer));
}

int qmrom_spi_reset_device(void *reset_handle)
{
	struct qm35_ctx *qm35_hdl = (struct qm35_ctx *)reset_handle;

	return qm35_reset(qm35_hdl, SPI_RST_LOW_DELAY_MS);
}

const struct firmware *qmrom_spi_get_firmware(void *handle,
					      enum chip_revision_e revision,
					      int lcs_state)
{
	const struct firmware *fw;
	struct spi_device *spi = handle;
	char fw_name[16]; /* enough room to store "qm35_xx_xxx.bin" */
	int ret;

	if (revision == CHIP_REVISION_A0)
		snprintf(fw_name, sizeof(fw_name), "qm35_%02x.bin", revision);
	else
		snprintf(fw_name, sizeof(fw_name), "qm35_%02x_%.3s.bin",
			 revision,
			 lcs_state == CC_BSV_SECURE_LCS ? "oem" : "icv");
	dev_info(&spi->dev, "Requesting fw %s!\n", fw_name);

	ret = request_firmware(&fw, fw_name, &spi->dev);
	if (ret) {
		release_firmware(fw);
		dev_err(&spi->dev,
			"request_firmware failed (ret=%d) for '%s'\n", ret,
			fw_name);
		return NULL;
	}

	dev_info(&spi->dev, "Firmware size is %zu!\n", fw->size);

	return fw;
}

void qmrom_spi_release_firmware(const struct firmware *fw)
{
	release_firmware(fw);
}

// FIXME: wait for ss ready mock
int qmrom_spi_wait_for_ready_line(void *handle, unsigned int timeout_ms)
{
	usleep_range(timeout_ms * 1000, timeout_ms * 1000);

	return 0;
}
