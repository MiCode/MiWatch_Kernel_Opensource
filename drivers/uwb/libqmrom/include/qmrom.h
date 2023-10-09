/*
 * Copyright 2021 Qorvo US, Inc.
 *
 * SPDX-License-Identifier: GPL-2.0 OR Apache-2.0
 *
 * This file is provided under the Apache License 2.0, or the
 * GNU General Public License v2.0.
 *
 */
#ifndef __QMROM_H__
#define __QMROM_H__

#ifndef __KERNEL__
#include <stdint.h>
#else
#include <linux/types.h>
#endif

#include <qmrom_error.h>

#define PEG_ERR_TIMEOUT PEG_ERR_BASE - 1
#define PEG_ERR_ROM_NOT_READY PEG_ERR_BASE - 2
#define PEG_ERR_SEND_CERT_WRITE PEG_ERR_BASE - 3
#define PEG_ERR_WRONG_REVISION PEG_ERR_BASE - 4

enum chip_revision_e {
	CHIP_REVISION_A0 = 0xA0,
	CHIP_REVISION_B0 = 0xB0,
	CHIP_REVISION_UNKNOWN = 0xFF
};

#define ROM_VERSION_A0 0x01a0
#define ROM_VERSION_B0 0xb000
#define ROM_SOC_ID_LEN 0x20
#define ROM_UUID_LEN 0x10

/* Life cycle state definitions. */

/*! Defines the CM life-cycle state value. */
#define CC_BSV_CHIP_MANUFACTURE_LCS 0x0
/*! Defines the DM life-cycle state value. */
#define CC_BSV_DEVICE_MANUFACTURE_LCS 0x1
/*! Defines the Secure life-cycle state value. */
#define CC_BSV_SECURE_LCS 0x5
/*! Defines the RMA life-cycle state value. */
#define CC_BSV_RMA_LCS 0x7

/* This function toggles the GPIO driving the RSTn pin of the device
 * It shall be implemented per the user of the library
 */
typedef int (*qmrom_reset_device)(void *reset_handle);

typedef const struct firmware* (*qmrom_get_firmware)(
	void *handle, enum chip_revision_e revision, int lcs_state);
typedef void (*qmrom_release_firmware)(const struct firmware* fw);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"

struct binary_chunk {
	uint32_t size;
	uint8_t data[0];
};

int qmrom_download_fw(void *spi_handle,
		      qmrom_get_firmware get_fw_fn,
		      qmrom_release_firmware release_fw_fn,
		      qmrom_reset_device reset_fn,
		      void *reset_handle,
		      int is_sram,
		      int lcs_state);
int qmrom_erase_dbg_cert(void *spi_handle,
		      qmrom_reset_device reset_fn, void *reset_handle);
int qmrom_flash_dbg_cert(void *spi_handle, struct binary_chunk *dbg_cert,
		      qmrom_reset_device reset_fn, void *reset_handle);
int qmrom_get_soc_info(void *spi_handle,
		      qmrom_reset_device reset_fn, void *reset_handle,
			  uint8_t soc_id[ROM_SOC_ID_LEN],
			  uint8_t uuid[ROM_UUID_LEN],
			  uint8_t *lcs_state);

#endif /* __QMROM_H__ */
