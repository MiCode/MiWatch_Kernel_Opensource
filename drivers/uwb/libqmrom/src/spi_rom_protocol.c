/*
 * Copyright 2021 Qorvo US, Inc.
 *
 * SPDX-License-Identifier: GPL-2.0 OR Apache-2.0
 *
 * This file is provided under the Apache License 2.0, or the
 * GNU General Public License v2.0.
 *
 */
#include <spi_rom_protocol.h>
#include <qmrom_utils.h>
#include <qmrom_log.h>
#include <qmrom_spi.h>
#include <qmrom.h>

#ifdef __KERNEL__
#include <linux/kernel.h>
#define bswap_16 cpu_to_be16
#else
#include <byteswap.h>
#endif

static const char *rom_code_b0_responses_str[] = {
    "res_READY_FOR_CS_LOW_CMD",
    "res_WRONG_CS_LOW_CMD",
    "res_WAITING_FOR_NS_RRAM_FILE_SIZE",
    "res_WAITING_FOR_NS_SRAM_FILE_SIZE",
    "res_WAITING_FOR_NS_RRAM_FILE_DATA",
    "res_WAITING_FOR_NS_SRAM_FILE_DATA", // 5
    "res_WAITING_FOR_SEC_FILE_DATA",
    "res_ERR_NS_SRAM_OR_RRAM_SIZE_CMD",
    "res_ERR_SEC_RRAM_SIZE_CMD",
    "res_ERR_WAITING_FOR_NS_IMAGE_DATA_CMD",
    "res_ERR_WAITING_FOR_SEC_IMAGE_DATA_CMD", // 10
    "res_ERR_IMAGE_SIZE_IS_ZERO",
    "res_ERR_IMAGE_SIZE_TOO_BIG",       /*Got more data than expected size*/
    "res_ERR_IMAGE_IS_NOT_16BYTES_MUL", /*Image must divide in 16 without remainder*/
    "res_ERR_GOT_DATA_MORE_THAN_ALLOWED",
    "res_ERR_RRAM_DATA_REMAINDER_NOT_ALLOWED", // 15 /*Remainder is allowed only for last packet*/
    "res_ERR_WAITING_FOR_CERT_DATA_CMD",
    "res_WAITING_FOR_FIRST_KEY_CERT",
    "res_WAITING_FOR_SECOND_KEY_CERT",
    "res_WAITING_FOR_CONTENT_CERT",
    "res_WAITING_FOR_DEBUG_CERT_DATA", // 20
    "res_ERR_FIRST_KEY_CERT_OR_FW_VER",
    "res_ERR_SECOND_KEY_CERT",
    "res_ERR_CONTENT_CERT_DOWNLOAD_ADDR",
    "res_ERR_TOO_MANY_IMAGES_IN_CONTENT_CERT",/*If the content certificate contains to much images*/
    "res_ERR_ADDRESS_NOT_DIVIDED_BY_8", //25
    "res_ERR_IMAGE_BOUNDARIES",
    "res_ERR_CERT_TYPE", /* Expected ICV type and got OEM */
    "res_ERR_PRODUCT_ID",
    "res_ERR_RRAM_RANGE_OR_WRITE",
    "res_WAITING_TO_DEBUG_CERTIFICATE_SIZE", // 30
    "res_ERR_DEBUG_CERT_SIZE"
};
static const int rom_code_b0_responses_nb =
	sizeof(rom_code_b0_responses_str) / sizeof(char *);
_Static_assert(sizeof(rom_code_b0_responses_str) / sizeof(char *) == 32, "Wrong array size");


int spi_proto_prepare_write_cmd(void *spi_handle, int do_exp_resp,
				uint8_t exp_resp, int read_len,
				struct stc *sstc, struct stc *hstc,
				enum chip_revision_e revision)
{
	uint8_t dev_ready_flag = SPI_DEVICE_READY_FLAGS;
	int rc = spi_proto_wait_for_device_flag(spi_handle,
						dev_ready_flag,
						sstc, hstc,
						SPI_DEVICE_POLL_RETRY);
	if (rc) {
		LOG_ERR("%s: spi_proto_wait_for_device_flag(%x) failed with %d\n",
			__func__, dev_ready_flag, rc);
		return rc;
	}

	hstc->all = 0;
	sstc->all = 0;
	hstc->host_flags.pre_read = 1;
	hstc->ul = SPI_UL_ROM_BOOT_PROTO;
	hstc->len = 0;
	rc = spi_proto_send_stc(spi_handle, sstc, hstc, revision);
	if (rc) {
		LOG_ERR("%s: pre-read failed with %d\n", __func__, rc);
		return rc;
	}
	dump_raw_hsstc(LOG_DBG, hstc, sstc, "pre-read\n");

	hstc->all = 0;
	sstc->all = 0;
	hstc->host_flags.read = 1;
	hstc->ul = SPI_UL_ROM_BOOT_PROTO;
	hstc->len = read_len;
	/* hstc paylaod shall be set by the caller */
	rc = spi_proto_send_stc(spi_handle, sstc, hstc, revision);
	if (rc) {
		LOG_ERR("%s: read failed with %d\n", __func__, rc);
		return rc;
	}
	dump_raw_hsstc(LOG_DBG, hstc, sstc, "read\n");
	if (do_exp_resp && sstc->payload[0] != exp_resp) {
		if ((revision == CHIP_REVISION_B0) &&
			(exp_resp < rom_code_b0_responses_nb) &&
			(sstc->payload[0] < rom_code_b0_responses_nb))
		{
			LOG_ERR("%s: rom code answered %s(0x%02x) vs %s(0x%02x) expected\n",
					__func__, rom_code_b0_responses_str[sstc->payload[0]],
					sstc->payload[0],
					rom_code_b0_responses_str[exp_resp], exp_resp);
		} else {
			LOG_ERR("%s: rom code answered 0x%02x vs 0x%02x expected -- 0x%x 0x%x\n",
					__func__, sstc->payload[0], exp_resp, rom_code_b0_responses_nb, revision);
		}
		return SPI_PROTO_WRONG_RESP;
	}
	return 0;
}

int spi_proto_wait_for_device_flag(void *spi_handle, uint8_t flag_mask,
				   struct stc *sstc, struct stc *hstc,
				   int retries)
{
	uint8_t tx_buffer[16], rx_buffer[16];
	int rc;

	tx_buffer[0] = 0;
	rc = qmrom_spi_wait_for_ready_line(spi_handle, SPI_READY_TIMEOUT_MS);
	if (rc)
		return rc;
	do {
		rc = qmrom_spi_transfer(spi_handle, (char *)rx_buffer,
			(char *)tx_buffer, 1);
		if (rc)
			return rc;
		dump_raw_buffer(LOG_DBG, tx_buffer, rx_buffer, 1, "Device ready\n");
		if ((rx_buffer[0] & flag_mask) == flag_mask)
			return 0;
		qmrom_msleep(SPI_INTERCMD_DELAY_MS);
	} while (--retries > 0);
	return PEG_ERR_TIMEOUT;
}

int spi_proto_send_stc(void *spi_handle, struct stc *sstc,
		       const struct stc *hstc, enum chip_revision_e revision)
{
	int rc = qmrom_spi_transfer(spi_handle, (char *)sstc, (const char *)hstc,
			       sizeof(struct stc) + hstc->len);
	if (rc) {
		LOG_ERR("%s: qmrom_spi_transfer failed: error %d\n", __func__, rc);
		return rc;
	}
	if (revision == CHIP_REVISION_A0) {
		sstc->len = bswap_16(sstc->len);
	}
	return 0;
}
