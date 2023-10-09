/*
 * Copyright 2021 Qorvo US, Inc.
 *
 * SPDX-License-Identifier: GPL-2.0 OR Apache-2.0
 *
 * This file is provided under the Apache License 2.0, or the
 * GNU General Public License v2.0.
 *
 */
#ifndef __QMROM_SPI_H__
#define __QMROM_SPI_H__

#include <stddef.h>
#include <qmrom_error.h>
#include <qmrom.h>

#ifndef __KERNEL__
struct firmware {
	uint32_t size;
	uint8_t data[0];
};
#else
#include <linux/firmware.h>
#endif

#define DEFAULT_SPI_CLOCKRATE 750000
#define DEFAULT_SPI_LATENCY_MS 2

#define SPI_ERR_NOCHAN SPI_ERR_BASE - 1
#define SPI_ERR_INFNOTFOUND SPI_ERR_BASE - 2
#define SPI_ERR_NOMEM SPI_ERR_BASE - 3
#define SPI_ERR_READ_INCOMPLETE SPI_ERR_BASE - 4
#define SPI_ERR_GPIO_WRITE_CMD_INCOMPLETE SPI_ERR_BASE - 5
#define SPI_ERR_GPIO_READ_CMD_INCOMPLETE SPI_ERR_BASE - 6
#define SPI_ERR_READY_LINE_TIMEOUT SPI_ERR_BASE - 7

/*Make sure that the error ranges don't overlap */
#define SPI_ERR_LIB_BASE (SPI_ERR_BASE - 500)
#define SPI_ERR_LIB(rc) (SPI_ERR_LIB_BASE - rc)

void *qmrom_spi_init(int spi_interface_index);
void qmrom_spi_uninit(void *handle);
int qmrom_spi_read(void *handle, char *buffer, size_t size);
int qmrom_spi_write(void *handle, const char *buffer, size_t size);
int qmrom_spi_transfer(void *handle, char *rbuf, const char *wbuf, size_t size);
int qmrom_spi_set_cs_level(void *handle, int level);
int qmrom_spi_reset_device(void *reset_handle);
const struct firmware *qmrom_spi_get_firmware(void *handle, enum chip_revision_e revision, int lcs_state);
void qmrom_spi_release_firmware(const struct firmware *fw);
int qmrom_spi_wait_for_ready_line(void *handle, unsigned int timeout_ms);

#endif /* __QMROM_SPI_H__ */
