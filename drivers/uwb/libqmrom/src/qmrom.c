/*
 * Copyright 2021 Qorvo US, Inc.
 *
 * SPDX-License-Identifier: GPL-2.0 OR Apache-2.0
 *
 * This file is provided under the Apache License 2.0, or the
 * GNU General Public License v2.0.
 *
 */
#include <qmrom_utils.h>
#include <qmrom_log.h>
#include <qmrom_spi.h>
#include <qmrom.h>
#include <spi_rom_protocol.h>
#include "../../qm35.h"

#ifndef __KERNEL__
#include <errno.h>
#include <stdint.h>
#include <string.h>
#else
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/string.h>
#endif

#define CHUNK_SIZE_A0 1016
#define CHUNK_SIZE_B0 1008

#define HBK_LOC 12
typedef enum {
	HBK_2E_ICV = 0,
	HBK_2E_OEM = 1,
	HBK_1E_ICV_OEM = 2,
} hbk_t;

struct unstitched_firmware {
	struct binary_chunk *fw_img;
	struct binary_chunk *fw_crt;
	struct binary_chunk *key1_crt;
	struct binary_chunk *key2_crt;
};

static int qmrom_chip_revision(void *spi_handle,
			struct stc *sstc,
			struct stc *hstc,
			enum chip_revision_e *revision);
static int qmrom_unstitch_fw(const struct firmware *fw,
			struct unstitched_firmware *unstitched_fw,
			enum chip_revision_e revision);
static int qmrom_start_bootloader_download(void *spi_handle, struct stc *sstc,
			struct stc *hstc, int is_sram,
			enum chip_revision_e revision, uint8_t is_oem);
static int qmrom_get_bootloader_rom_version(void *spi_handle, struct stc *sstc,
            struct stc *hstc, uint16_t *version);
static int qmrom_send_cert(void *spi_handle, struct binary_chunk *fcert,
			uint16_t exp_read, uint16_t read_size,
			uint16_t cert_size, struct stc *sstc,
			struct stc *hstc, enum chip_revision_e revision);
static int qmrom_send_image_size(void *spi_handle, struct binary_chunk *fw,
			struct stc *sstc, struct stc *hstc,
			enum chip_revision_e revision);
static int qmrom_reboot_bootloader(void *spi_handle,
			qmrom_reset_device reset_fn,
			void *reset_handle);
static int qmrom_send_multichunk_bytes(void *spi_handle,
			struct binary_chunk *fw,
			struct stc *sstc, struct stc *hstc,
			enum chip_revision_e revision,
			int rsp_expected, int cmd_code);


static void qmrom_free_stcs(struct stc *hstc, struct stc *sstc)
{
	if (hstc) qmrom_free(hstc);
	if (sstc) qmrom_free(sstc);
}

static int qmrom_allocate_stcs(struct stc **hstc, struct stc **sstc)
{
	int rc = 0;
	uint8_t *tx_buf = NULL, *rx_buf = NULL;

	qmrom_alloc(tx_buf, MAX_STC_FRAME_LEN);
	if (tx_buf == NULL) {
		rc = -ENOMEM;
		goto out;
	}

	qmrom_alloc(rx_buf, MAX_STC_FRAME_LEN);
	if (rx_buf == NULL) {
		qmrom_free(tx_buf);
		rc = -ENOMEM;
		goto out;
	}

	*hstc = (struct stc *)tx_buf;
	*sstc = (struct stc *)rx_buf;
	return rc;
out:
	qmrom_free_stcs((struct stc *)tx_buf, (struct stc *)rx_buf);
	*hstc = NULL;
	*sstc = NULL;
	return rc;
}

static int qmrom_send_dbg_cert_size(void *spi_handle, struct binary_chunk *fw,
				 struct stc *sstc, struct stc *hstc)
{
	int rc;
	memset(hstc->payload, 0, SPI_BOOT_DEBUG_CERT_SIZE_B0);
	rc = spi_proto_prepare_write_cmd(spi_handle, 1,
					     SPI_RSP_WAIT_DBG_CERT_SIZE_B0,
					     SPI_ROM_DBG_CERT_SIZE_SIZE, sstc,
					     hstc, CHIP_REVISION_B0);
	if (rc) {
		LOG_ERR("%s: prepare write failed with %d\n", __func__, rc);
		return rc;
	}

	hstc->all = 0;
	sstc->all = 0;
	hstc->host_flags.write = 1;
	hstc->ul = SPI_UL_ROM_BOOT_PROTO;
	hstc->len = sizeof(uint32_t) + 1;
	hstc->payload[0] = SPI_BOOT_DEBUG_CERT_SIZE_B0;
	hstc->payload[1] = fw->size & 0xff;
	hstc->payload[2] = (fw->size >> 8) & 0xff;
	hstc->payload[3] = (fw->size >> 16) & 0xff;
	hstc->payload[4] = (fw->size >> 24) & 0xff;
	rc = spi_proto_send_stc(spi_handle, sstc, hstc, CHIP_REVISION_B0);
	if (rc) {
		LOG_ERR("%s: write failed with %d\n", __func__, rc);
		return rc;
	}
	dump_raw_hsstc(LOG_DBG, hstc, sstc, "debug_cert_size write\n");
	return 0;
}

int qmrom_flash_dbg_cert(void *spi_handle, struct binary_chunk *dbg_cert,
		      qmrom_reset_device reset_fn, void *reset_handle)
{
	int rc = 0;
	struct stc *hstc, *sstc;

	rc = qmrom_allocate_stcs(&hstc, &sstc);
	if (rc)
		return rc;

	LOG_INFO("Rebooting the board\n");
	rc = qmrom_reboot_bootloader(spi_handle, reset_fn, reset_handle);
	if (rc)
		goto out;

	rc = spi_proto_prepare_write_cmd(spi_handle, 1,
					     0, SPI_ROM_DBG_CERT_SIZE_SIZE, sstc,
					     hstc, CHIP_REVISION_B0);
	if (rc) {
		LOG_ERR("%s: prepare write failed with %d\n", __func__, rc);
		goto out;
	}

	hstc->all = 0;
	sstc->all = 0;
	hstc->host_flags.write = 1;
	hstc->ul = SPI_UL_ROM_BOOT_PROTO;
	hstc->len = 1;
	hstc->payload[0] = SPI_BOOT_DOWNLOAD_DEBUG_CERT_B0;
	rc = spi_proto_send_stc(spi_handle, sstc, hstc, CHIP_REVISION_B0);
	if (rc) {
		LOG_ERR("%s: write failed with %d\n", __func__, rc);
		goto out;
	}
	dump_raw_hsstc(LOG_DBG, hstc, sstc, "download debug cert\n");

	LOG_INFO("Sending the dbg cert size cmd\n");
	rc = qmrom_send_dbg_cert_size(spi_handle, dbg_cert, sstc, hstc);
	if (rc)
		goto out;

	rc = qmrom_send_multichunk_bytes(spi_handle, dbg_cert, sstc, hstc, CHIP_REVISION_B0,
				SPI_RSP_WAIT_DBG_CERT_B0, SPI_BOOT_DOWNLOAD_CERT_DATA_B0);

out:
	qmrom_free_stcs(hstc, sstc);
	return rc;
}

int qmrom_erase_dbg_cert(void *spi_handle,
		      qmrom_reset_device reset_fn, void *reset_handle)
{
	int rc = 0;
	struct stc *hstc, *sstc;

	rc = qmrom_allocate_stcs(&hstc, &sstc);
	if (rc)
		return rc;

	LOG_INFO("Rebooting the board\n");
	rc = qmrom_reboot_bootloader(spi_handle, reset_fn, reset_handle);
	if (rc)
		goto out;

	/* Erasing... */
	hstc->payload[0] = 0;
	rc = spi_proto_prepare_write_cmd(spi_handle, 1,
					     0, 1, sstc, hstc, CHIP_REVISION_B0);
	if (rc) {
		LOG_ERR("%s: prepare write failed with %d\n", __func__, rc);
		goto out;
	}
	/* Send the erase cmd */
	hstc->all = 0;
	sstc->all = 0;
	hstc->host_flags.write = 1;
	hstc->ul = SPI_UL_ROM_BOOT_PROTO;
	hstc->len = 1;
	hstc->payload[0] = SPI_BOOT_ERASE_DEBUG_CERT_B0;
	rc = spi_proto_send_stc(spi_handle, sstc, hstc, CHIP_REVISION_B0);
	if (rc) {
		LOG_ERR("%s: write failed with %d\n", __func__, rc);
		goto out;
	}
	dump_raw_hsstc(LOG_DBG, hstc, sstc, "erase debug cert\n");

out:
	qmrom_free_stcs(hstc, sstc);
	return rc;
}

int qmrom_get_soc_info(void *spi_handle,
		      qmrom_reset_device reset_fn, void *reset_handle,
			  uint8_t soc_id[ROM_SOC_ID_LEN],
			  uint8_t uuid[ROM_UUID_LEN],
			  uint8_t *lcs_state)
{
	int rc = 0;
	struct stc *hstc, *sstc;
	uint8_t *payload;
	int i;

	rc = qmrom_allocate_stcs(&hstc, &sstc);
	if (rc)
		return rc;

	LOG_INFO("Rebooting the board\n");
	rc = qmrom_reboot_bootloader(spi_handle, reset_fn, reset_handle);
	if (rc)
		goto out;

	/* Sending command... */
	hstc->payload[0] = 3;
	rc = spi_proto_prepare_write_cmd(spi_handle, 1,
					     0, 1, sstc, hstc, CHIP_REVISION_B0);
	if (rc) {
		LOG_ERR("%s: prepare write failed with %d\n", __func__, rc);
		goto out;
	}
	/* Send the erase cmd */
	hstc->all = 0;
	sstc->all = 0;
	hstc->host_flags.read = 1;
	hstc->ul = SPI_UL_ROM_BOOT_PROTO;
	hstc->len = 0x32;
	hstc->payload[0] = 0;
	rc = spi_proto_send_stc(spi_handle, sstc, hstc, CHIP_REVISION_B0);
	if (rc) {
		LOG_ERR("%s: write failed with %d\n", __func__, rc);
		goto out;
	}
	dump_raw_hsstc(LOG_DBG, hstc, sstc, "erase debug cert\n");

	hstc->all = 0;
	sstc->all = 0;
	hstc->host_flags.write = 1;
	hstc->ul = SPI_UL_ROM_BOOT_PROTO;
	hstc->len = 1;
	hstc->payload[0] = SPI_BOOT_GET_CHIP_INFO_B0;
	rc = spi_proto_send_stc(spi_handle, sstc, hstc, CHIP_REVISION_B0);
	if (rc) {
		LOG_ERR("%s: write failed with %d\n", __func__, rc);
		return rc;
	}
	hstc->all = 0;
	sstc->all = 0;
	memset(hstc->payload, 0, SPI_ROM_READ_INFO_SIZE_B0);
	rc = spi_proto_prepare_write_cmd(spi_handle, 0,
                                     SPI_RSP_WAIT_DOWNLOAD_MODE,
                                     SPI_ROM_READ_INFO_SIZE_B0, sstc,
                                     hstc, CHIP_REVISION_B0);
	if (rc) {
		LOG_ERR("%s: prepare write failed with %d\n", __func__, rc);
		return rc;
	}
	if (sstc->len != SPI_ROM_READ_INFO_SIZE_B0) {
		LOG_ERR("%s: wrong device info size 0x%x\n", __func__, sstc->len);
		return -1;
	}

	/* skip the first byte */
	payload = &sstc->payload[1];
	for (i = 0 ; i < ROM_SOC_ID_LEN ; i++)
		soc_id[i] = payload[ROM_SOC_ID_LEN - i - 1];
	payload += ROM_SOC_ID_LEN;
	*lcs_state = *payload;
	payload += 1;
	for (i = 0 ; i < ROM_UUID_LEN ; i++)
		uuid[i] = payload[ROM_UUID_LEN - i - 1];

out:
	qmrom_free_stcs(hstc, sstc);
	return rc;
}

int qmrom_download_fw(void *spi_handle,
		      qmrom_get_firmware get_fw_fn,
		      qmrom_release_firmware release_fw_fn,
		      qmrom_reset_device reset_fn,
		      void *reset_handle,
		      int is_sram,
		      int lcs_state)
{
	int rc = 0;
	struct stc *hstc, *sstc;
	uint16_t spi_rom_read_image_cert_size = 0;
	const struct firmware *stitched_firmware;
	struct unstitched_firmware unstitched_fw = { 0 };
	enum chip_revision_e revision;
	int is_oem;

	rc = qmrom_allocate_stcs(&hstc, &sstc);
	if (rc)
		return rc;

	LOG_INFO("Rebooting the board\n");
	rc = qmrom_reboot_bootloader(spi_handle, reset_fn, reset_handle);
	if (rc)
		goto fail;

	LOG_INFO("Getting the board revision\n");
	rc = qmrom_chip_revision(spi_handle, sstc, hstc, &revision);
	if (rc)
		goto fail;

	if (revision == CHIP_REVISION_A0) {
		spi_rom_read_image_cert_size = SPI_ROM_READ_IMAGE_CERT_SIZE_A0;
	} else if (revision == CHIP_REVISION_B0) {
		spi_rom_read_image_cert_size = SPI_ROM_READ_IMAGE_CERT_SIZE_B0;
	} else {
		LOG_ERR("%s: unknown chip: %d\n", __func__, revision);
		goto fail;
	}


	stitched_firmware = get_fw_fn(spi_handle, revision, lcs_state);
	if (stitched_firmware == NULL) {
		LOG_ERR("%s: failed getting the binary firmware\n", __func__);
		rc = -1;
		goto fail;
	}

	LOG_INFO("Unstitching the fw\n");
	rc = qmrom_unstitch_fw(stitched_firmware, &unstitched_fw, revision);
	if (rc)
		goto fail;

	release_fw_fn(stitched_firmware);

	is_oem = unstitched_fw.key1_crt->data[HBK_LOC] != HBK_2E_ICV;
	LOG_INFO("Starting the download fw sequence (%s)\n", is_oem ? "OEM" : "ICV");
	rc = qmrom_start_bootloader_download(spi_handle, sstc, hstc,
					     is_sram, revision, is_oem);
	if (rc)
		goto fail;

	LOG_INFO("Sending the key1 certificate\n");
	rc = qmrom_send_cert(spi_handle, unstitched_fw.key1_crt,
				revision == CHIP_REVISION_A0 ?
					SPI_RSP_WAIT_FOR_KEY1_CERT_A0 :
					SPI_RSP_WAIT_FOR_KEY1_CERT_B0,
				spi_rom_read_image_cert_size,
				unstitched_fw.key1_crt->size,
				sstc, hstc, revision);
	if (rc)
		goto fail;

	LOG_INFO("Sending the key2 certificate\n");
	rc = qmrom_send_cert(spi_handle, unstitched_fw.key2_crt,
				revision == CHIP_REVISION_A0 ?
					SPI_RSP_WAIT_FOR_KEY2_CERT_A0 :
					SPI_RSP_WAIT_FOR_KEY2_CERT_B0,
				spi_rom_read_image_cert_size,
				unstitched_fw.key2_crt->size,
				sstc, hstc, revision);
	if (rc)
		goto fail;

	LOG_INFO("Sending the content certificate\n");
	rc = qmrom_send_cert(spi_handle, unstitched_fw.fw_crt,
				revision == CHIP_REVISION_A0 ?
					SPI_RSP_WAIT_FOR_IMAGE_CERT_A0 :
					SPI_RSP_WAIT_FOR_IMAGE_CERT_B0,
				spi_rom_read_image_cert_size,
				unstitched_fw.fw_crt->size,
				sstc, hstc, revision);
	if (rc)
		goto fail;

	if (revision == CHIP_REVISION_A0) {
		LOG_INFO("Sending the image size\n");
		rc = qmrom_send_image_size(spi_handle, unstitched_fw.fw_img,
					sstc, hstc, revision);
		if (rc)
			goto fail;
	}

	LOG_INFO("Sending the image\n");
	rc = qmrom_send_multichunk_bytes(spi_handle, unstitched_fw.fw_img,
				sstc, hstc, revision,
				SPI_RSP_WAIT_FOR_IMAGE,
				SPI_BOOT_DOWNLOAD_SEC_IMAGE_DATA_B0);
	if (rc)
		goto fail;

fail:
	qmrom_free_stcs(hstc, sstc);
	if (unstitched_fw.fw_img) {
		qmrom_free(unstitched_fw.fw_img);
		qmrom_free(unstitched_fw.fw_crt);
		qmrom_free(unstitched_fw.key1_crt);
		qmrom_free(unstitched_fw.key2_crt);
	}
	return rc;
}

static int qmrom_chip_revision(void *spi_handle,
			       struct stc *sstc,
			       struct stc *hstc,
			       enum chip_revision_e *revision) {
	uint16_t rom_version;
	int rc = 0;

	LOG_INFO("Getting the bootloader ROM version\n");
	rc = qmrom_get_bootloader_rom_version(spi_handle, sstc, hstc,
					      &rom_version);
	if (rc)
		goto out;

	if (rom_version == ROM_VERSION_A0)
		*revision = CHIP_REVISION_A0;
	else if (rom_version == ROM_VERSION_B0)
		*revision = CHIP_REVISION_B0;
	else {
		LOG_ERR("%s: unknown chip revision, received id: 0x%x\n",
			__func__, rom_version);
		*revision = CHIP_REVISION_UNKNOWN;
		rc = 1;
	}

	LOG_INFO("detected board revision: %02x\n", *revision);
out:
	return rc;
}

static int qmrom_unstitch_fw(const struct firmware *fw,
			     struct unstitched_firmware *unstitched_fw,
			     enum chip_revision_e revision) {
	uint32_t tot_len = 0;
	uint32_t fw_img_sz = 0;
	uint32_t fw_crt_sz = 0;
	uint32_t key1_crt_sz = 0;
	uint32_t key2_crt_sz = 0;
	uint8_t *p_key1;
	uint8_t *p_key2;
	uint8_t *p_crt;
	uint8_t *p_fw;
	int ret = 0;

	if (revision == CHIP_REVISION_A0) {
		fw_img_sz = fw->size;
		fw_crt_sz = SPI_ROM_WRITE_IMAGE_CERT_SIZE;
		key1_crt_sz = SPI_ROM_WRITE_KEY_CERT_SIZE;
		key2_crt_sz = SPI_ROM_WRITE_KEY_CERT_SIZE;

		p_fw = (uint8_t *) fw->data;
		p_key1 = NULL;
		p_key2 = NULL;
		p_crt = NULL;
		tot_len = 0;
	} else if (revision == CHIP_REVISION_B0) {
		// key1
		key1_crt_sz = *(uint32_t * ) & fw->data[tot_len];
		tot_len += sizeof(key1_crt_sz);
		p_key1 = (uint8_t * ) & fw->data[tot_len];
		tot_len += key1_crt_sz;
		if (tot_len >= fw->size) {
			LOG_ERR("%s: Invalid key1_len %u -- stitched len %zu\n",
				__func__, key1_crt_sz, (size_t)fw->size);
			ret = -EINVAL;
			goto out;
		}

		// key2
		key2_crt_sz = *(uint32_t * ) & fw->data[tot_len];
		tot_len += sizeof(key2_crt_sz);
		p_key2 = (uint8_t * ) & fw->data[tot_len];
		tot_len += key2_crt_sz;
		if (tot_len >= fw->size) {
			LOG_ERR("%s: Invalid key1_len %u and key2_len %u -- stitched len %zu\n",
				__func__, key1_crt_sz, key2_crt_sz, (size_t)fw->size);
			ret = -EINVAL;
			goto out;
		}

		// cert
		fw_crt_sz = *(uint32_t * ) & fw->data[tot_len];
		tot_len += sizeof(fw_crt_sz);
		p_crt = (uint8_t * ) & fw->data[tot_len];
		tot_len += fw_crt_sz;
		if (tot_len >= fw->size) {
			LOG_ERR("%s: Invalid key1_len %u, key2_len %u and content len %u -- stitched len %zu\n",
				__func__, key1_crt_sz, key2_crt_sz, fw_crt_sz,
				(size_t)fw->size);
			ret = -EINVAL;
			goto out;
		}

		// fw
		fw_img_sz = *(uint32_t * ) & fw->data[tot_len];
		tot_len += sizeof(fw_img_sz);
		p_fw = (uint8_t * ) & fw->data[tot_len];
		tot_len += fw_img_sz;
		if (tot_len != fw->size) {
			LOG_ERR("%s: Invalid key1_len %u, key2_len %u, content len %u and fw_len %u -- stitched len %zu\n",
				__func__, key1_crt_sz, key2_crt_sz, fw_crt_sz,
				fw_img_sz, (size_t)fw->size);
			ret = -EINVAL;
			goto out;
		}
	} else {
		LOG_ERR("%s: unknown chip: %d\n", __func__, revision);
		goto out;
	}

	qmrom_alloc(unstitched_fw->fw_img,
		    fw_img_sz + sizeof(unstitched_fw->fw_img));
	if (unstitched_fw->fw_img == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	qmrom_alloc(unstitched_fw->fw_crt,
		    fw_crt_sz + sizeof(unstitched_fw->fw_crt));
	if (unstitched_fw->fw_crt == NULL) {
		ret = -ENOMEM;
		goto err;
	}

	qmrom_alloc(unstitched_fw->key1_crt,
		    key1_crt_sz + sizeof(unstitched_fw->key1_crt));
	if (unstitched_fw->key1_crt == NULL) {
		ret = -ENOMEM;
		goto err;
	}

	qmrom_alloc(unstitched_fw->key2_crt,
		    key2_crt_sz + sizeof(unstitched_fw->key2_crt));
	if (unstitched_fw->key2_crt == NULL) {
		ret = -ENOMEM;
		goto err;
	}

	unstitched_fw->key1_crt->size = key1_crt_sz;
	unstitched_fw->key2_crt->size = key2_crt_sz;
	unstitched_fw->fw_crt->size = fw_crt_sz;
	unstitched_fw->fw_img->size = fw_img_sz;

	if (revision == CHIP_REVISION_B0) {
		memcpy(unstitched_fw->key1_crt->data, p_key1, key1_crt_sz);
		memcpy(unstitched_fw->key2_crt->data, p_key2, key2_crt_sz);
		memcpy(unstitched_fw->fw_crt->data, p_crt, fw_crt_sz);
	}

	memcpy(unstitched_fw->fw_img->data, p_fw, fw_img_sz);

	return 0;

err:
	qmrom_free(unstitched_fw->fw_img);
	qmrom_free(unstitched_fw->fw_crt);
	qmrom_free(unstitched_fw->key1_crt);
	qmrom_free(unstitched_fw->key2_crt);

out:
	return ret;
}

static int qmrom_start_bootloader_download(void *spi_handle, struct stc *sstc,
					   struct stc *hstc, int is_sram, enum chip_revision_e revision,
					   uint8_t is_oem)
{
	int rc;

	hstc->all = 0;
	sstc->all = 0;
	hstc->host_flags.write = 1;
	hstc->ul = SPI_UL_ROM_BOOT_PROTO;

	if (revision == CHIP_REVISION_A0) {
		hstc->payload[0] = is_sram ? SPI_BOOT_DOWNLOAD_SRAM_CMD :
							SPI_BOOT_DOWNLOAD_RRAM_CMD_A0;
	} else if (revision == CHIP_REVISION_B0) {
		hstc->payload[0] = is_oem ? SPI_BOOT_DOWNLOAD_SEC_OEM_CMD_B0 :
					SPI_BOOT_DOWNLOAD_SEC_ICV_CMD_B0;
	} else {
		LOG_ERR("%s: unknown chip: %d\n", __func__, revision);
		return -1;
	}

	hstc->len = 1;
	rc = spi_proto_send_stc(spi_handle, sstc, hstc, revision);
	if (rc) {
		LOG_ERR("%s: write failed with %d\n", __func__, rc);
		return rc;
	}
	dump_raw_hsstc(LOG_DBG, hstc, sstc, "download write\n");
	return 0;
}

static int qmrom_get_bootloader_rom_version(void *spi_handle, struct stc *sstc,
                                                 struct stc *hstc, uint16_t *version) {
	int rc;
	uint16_t dev_id = 0;

	memset(hstc->payload, 0, SPI_ROM_READ_VERSION_SIZE_A0);
	rc = spi_proto_prepare_write_cmd(spi_handle, 1,
					 SPI_RSP_WAIT_DOWNLOAD_MODE,
					 SPI_ROM_READ_VERSION_SIZE_A0, sstc,
					 hstc, CHIP_REVISION_A0);
	// FIXME: we have to ignore the rc here because for B0, the write cmd
	// function will fail because the SPI_RSP_WAIT_DOWNLOAD_MODE won't match
	// with the expected one as this flag is different between A0 and B0
	// and at this point we don't know the existing revision.
//	if (rc) {
//		LOG_ERR("%s: prepare write failed with %d\n", __func__, rc);
//		return rc;
//	}

	*version = sstc->payload[1] << 8 | sstc->payload[2];
	if (*version == ROM_VERSION_A0) {
		LOG_INFO("%s: ROM bootloader version: 0x%04x, Device ID 0x%04x\n",
			 __func__, *version, dev_id);

		return 0;
	}

	hstc->all = 0;
	sstc->all = 0;
	hstc->host_flags.write = 1;
	hstc->ul = SPI_UL_ROM_BOOT_PROTO;
	hstc->len = 1;
	hstc->payload[0] = SPI_BOOT_GET_CHIP_VER_B0;
	rc = spi_proto_send_stc(spi_handle, sstc, hstc, CHIP_REVISION_B0);
	if (rc) {
		LOG_ERR("%s: write failed with %d\n", __func__, rc);
		return rc;
	}
	hstc->all = 0;
	sstc->all = 0;
	memset(hstc->payload, 0, SPI_ROM_READ_VERSION_SIZE_B0);
	rc = spi_proto_prepare_write_cmd(spi_handle, 0,
                                     SPI_RSP_WAIT_DOWNLOAD_MODE,
                                     SPI_ROM_READ_VERSION_SIZE_B0, sstc,
                                     hstc, CHIP_REVISION_B0);
	if (rc) {
		LOG_ERR("%s: prepare write failed with %d\n", __func__, rc);
		return rc;
	}
	*version = sstc->payload[1] << 8 | sstc->payload[2];
	dev_id = sstc->payload[3] << 8 | sstc->payload[4];
	LOG_INFO("%s: ROM bootloader version: 0x%04x, Device ID 0x%04x\n", __func__, *version, dev_id);

	return 0;
}

static int qmrom_send_cert(void *spi_handle, struct binary_chunk *fcert,
			   uint16_t exp_read, uint16_t read_size,
			   uint16_t cert_size, struct stc *sstc,
			   struct stc *hstc, enum chip_revision_e revision)
{
	int rc;
	memset(hstc->payload, 0, read_size);
	rc = spi_proto_prepare_write_cmd(spi_handle, 1, exp_read, read_size,
					     sstc, hstc, revision);
	if (rc) {
		LOG_ERR("%s: prepare write failed with %d\n", __func__, rc);
		return rc;
	}
	if (fcert->size < cert_size) {
		LOG_ERR("%s: could not read the full certificate (%d bytes asked, %d read)\n",
			__func__, cert_size, fcert->size);
		return PEG_ERR_SEND_CERT_WRITE;
	}

	hstc->all = 0;
	sstc->all = 0;
	hstc->host_flags.write = 1;
	hstc->ul = SPI_UL_ROM_BOOT_PROTO;

	switch (revision) {
		case CHIP_REVISION_A0:
			hstc->len = cert_size;
			memcpy(hstc->payload, fcert->data, cert_size);
			break;
		case CHIP_REVISION_B0:
			hstc->len = cert_size + 1;
			hstc->payload[0] = SPI_BOOT_DOWNLOAD_CERT_DATA_B0;
			memcpy(hstc->payload + 1, fcert->data, cert_size);
			break;
		default:
			LOG_ERR("%s: Wrong revision %d\n", __func__, revision);
			return PEG_ERR_WRONG_REVISION;
	}

	rc = spi_proto_send_stc(spi_handle, sstc, hstc, revision);
	if (rc) {
		LOG_ERR("%s: write failed with %d\n", __func__, rc);
		return rc;
	}
	dump_raw_hsstc(LOG_DBG, hstc, sstc, "certificate write\n");
	return 0;
}

static int qmrom_send_image_size(void *spi_handle, struct binary_chunk *fw,
				 struct stc *sstc, struct stc *hstc, enum chip_revision_e revision)
{
	int rc;
	uint16_t spi_rom_read_image_size_size = 0;

	if (revision == CHIP_REVISION_A0) {
        	spi_rom_read_image_size_size = SPI_ROM_READ_IMAGE_SIZE_SIZE_A0;
	} else if (revision == CHIP_REVISION_B0) {
        	spi_rom_read_image_size_size = SPI_ROM_READ_IMAGE_SIZE_SIZE_B0;
	} else {
		LOG_ERR("%s: unknown chip: %d\n", __func__, revision);
		return -1;
	}

	memset(hstc->payload, 0, spi_rom_read_image_size_size);
	rc = spi_proto_prepare_write_cmd(spi_handle, 1,
					     SPI_RSP_WAIT_IMAGE_SIZE,
					     spi_rom_read_image_size_size, sstc,
					     hstc, revision);

	if (rc) {
		LOG_ERR("%s: prepare write failed with %d\n", __func__, rc);
		return rc;
	}

	hstc->all = 0;
	sstc->all = 0;
	hstc->host_flags.write = 1;
	hstc->ul = SPI_UL_ROM_BOOT_PROTO;
	hstc->len = sizeof(uint32_t);
	hstc->payload[0] = fw->size & 0xff;
	hstc->payload[1] = (fw->size >> 8) & 0xff;
	hstc->payload[2] = (fw->size >> 16) & 0xff;
	hstc->payload[3] = (fw->size >> 24) & 0xff;
	rc = spi_proto_send_stc(spi_handle, sstc, hstc, revision);
	if (rc) {
		LOG_ERR("%s: write failed with %d\n", __func__, rc);
		return rc;
	}
	dump_raw_hsstc(LOG_DBG, hstc, sstc, "image_size write\n");
	return 0;
}

static int qmrom_send_multichunk_bytes(void *spi_handle, struct binary_chunk *fw,
				  struct stc *sstc, struct stc *hstc, enum chip_revision_e revision,
				  int rsp_expected, int cmd_code)
{
	size_t total_sent = 0, current_size;
	uint8_t *payload = fw->data;
	uint16_t chunk_size;

	if (revision == CHIP_REVISION_A0) {
		chunk_size = CHUNK_SIZE_A0;
	} else if (revision == CHIP_REVISION_B0) {
		chunk_size = CHUNK_SIZE_B0;
	} else {
		LOG_ERR("%s: unknown chip: %d\n", __func__, revision);
		return -1;
	}

	do {
		int rc;
		hstc->len = 0;
		hstc->payload[0] = 0;
		rc = spi_proto_prepare_write_cmd(
			spi_handle, 1, rsp_expected, 1, sstc, hstc, revision);
		if (rc) {
			LOG_ERR("%s: spi_proto_prepare_write_cmd failed with %d\n",
				__func__, rc);
			return rc;
		}
		hstc->all = 0;
		sstc->all = 0;
		hstc->host_flags.write = 1;
		hstc->ul = SPI_UL_ROM_BOOT_PROTO;
		current_size = fw->size - total_sent;
		if (current_size > chunk_size)
			current_size = chunk_size;

		if (revision == CHIP_REVISION_A0) {
			memcpy(hstc->payload, payload, current_size);
			hstc->len = current_size;
		} else if (revision == CHIP_REVISION_B0) {
			hstc->payload[0] = cmd_code;
			memcpy(&hstc->payload[1], payload, current_size);
			hstc->len = current_size + 1;
		} else {
			LOG_ERR("%s: unknown chip: %d\n", __func__, revision);
			return -1;
		}

		rc = spi_proto_send_stc(spi_handle, sstc, hstc, revision);
		if (rc) {
			LOG_ERR("%s: write failed with %d\n", __func__, rc);
			return rc;
		}
		dump_raw_hsstc(LOG_DBG, hstc, sstc, "image write\n");

		total_sent += current_size;
		payload += current_size;
	} while (total_sent < fw->size);
	return 0;
}

int qmrom_reboot_bootloader(void *spi_handle,
				   qmrom_reset_device reset_fn,
				   void *reset_handle)
{
	int rc;
	rc = qmrom_spi_set_cs_level(spi_handle, 0);
	gpio_direction_output(cs_gpio, 0);
	if (rc) {
		LOG_ERR("%s: spi_set_cs_level(0) failed with %d\n", __func__,
			rc);
		return rc;
	}
	qmrom_msleep(SPI_RST_LOW_DELAY_MS);

	reset_fn(reset_handle);

	qmrom_msleep(SPI_RST_LOW_DELAY_MS);

	rc = qmrom_spi_set_cs_level(spi_handle, 1);
	gpio_direction_output(cs_gpio, 1);
	if (rc) {
		LOG_ERR("%s: spi_set_cs_level(1) failed with %d\n", __func__,
			rc);
		return rc;
	}
	return 0;
}
