/*
 * Copyright 2021 Qorvo US, Inc.
 *
 * SPDX-License-Identifier: GPL-2.0 OR Apache-2.0
 *
 * This file is provided under the Apache License 2.0, or the
 * GNU General Public License v2.0.
 *
 */
#ifndef __SPI_ROM_PROTOCOL_H__
#define __SPI_ROM_PROTOCOL_H__

#include <qmrom_error.h>
#include <qmrom.h>

#ifndef __KERNEL__
#include <stdint.h>
#include <unistd.h>
#else
#include <linux/types.h>
#endif

#define SPI_PROTO_WRONG_RESP SPI_PROTO_ERR_BASE - 1

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
struct stc {
	union {
		struct {
			union {
				struct {
					uint8_t reserved : 5;
					uint8_t read : 1; // Read indication: Set to one by the host to tell the SOC SPI driver that this transaction is a read, otherwise it is set to zero.
					uint8_t pre_read : 1; // Pre-read indication: Set to one by the host to tell the SOC SPI driver that the next transaction will be a read, otherwise it is set to zero
					uint8_t write : 1; // Write indication: Set to one by the host when it is doing a write transaction. Set to zero when the host is not doing a write.
				} host_flags;
				struct {
					uint8_t reserved : 4;
					uint8_t err : 1; // Error indication: This is set to one by the SOC SPI driver to tell the HOST that it has detected some error in the SPI protocol.
					uint8_t ready : 1; //
					uint8_t out_active : 1; // Output active indication: Set to one by the SOC SPI driver to tell the HOST that it is outputting data on MISO line and expecting that the host is doing a read transaction at this time. This is set to zero for all other transactions.
					uint8_t out_waiting : 1; // Output data waiting indication: Set to one by the SOC SPI driver to tell the HOST there is data awaiting reading. This is set to zero when there is no data pending output.
				} soc_flags;
			};
			uint8_t ul;
			uint16_t len;
		};
		uint32_t all;
	};
	uint8_t payload[0];
} __attribute__((packed));
#pragma GCC diagnostic pop

/* Host to Soc (HS) masks */
#define SPI_HS_WRITE_CMD_BIT_MASK 0x80
#define SPI_HS_PRD_CMD_BIT_MASK 0x40
#define SPI_HS_RD_CMD_BIT_MASK 0x20

/* Soc to Host (SH) masks */
#define SPI_SH_ODW_BIT_MASK 0x80
#define SPI_SH_ACTIVE_BIT_MASK 0x40
#define SPI_SH_READY_CMD_BIT_MASK 0x20
#define SPI_DEVICE_READY_FLAGS SPI_SH_ODW_BIT_MASK

/* Communication parameters */
#define MAX_STC_FRAME_LEN 1024
#define MAX_STC_PAYLOAD_LEN (MAX_STC_FRAME_LEN - sizeof(struct stc))
#define SPI_NUM_READS_FOR_READY 1
#define SPI_NUM_FAILS_RETRY 4
#define SPI_ET_PROTOCOL 5
#define SPI_RST_LOW_DELAY_MS 20
#define SPI_INTERCMD_DELAY_MS 10
#define SPI_DEVICE_POLL_RETRY 10
#define SPI_READY_TIMEOUT_MS 50
#define SPI_ET_VERSION_LOCATION 0x601f0000

/* ROM boot proto */
#define SPI_ROM_READ_VERSION_SIZE_A0 3
#define SPI_ROM_READ_IMAGE_CERT_SIZE_A0 3
#define SPI_ROM_READ_IMAGE_SIZE_SIZE_A0 3

#define SPI_ROM_READ_VERSION_SIZE_B0 5
#define SPI_ROM_READ_INFO_SIZE_B0 50
#define SPI_ROM_READ_IMAGE_CERT_SIZE_B0 1
#define SPI_ROM_READ_IMAGE_SIZE_SIZE_B0 1

#define SPI_ROM_WRITE_KEY_CERT_SIZE 6
#define SPI_ROM_WRITE_IMAGE_CERT_SIZE 40
#define SPI_ROM_WRITE_IMAGE_SIZE_SIZE 4
#define SPI_ROM_DBG_CERT_SIZE_SIZE 5

_Static_assert(MAX_STC_PAYLOAD_LEN == 1020,
	       "MAX_STC_PAYLOAD_LEN should be 1020...");

enum spi_rsp_e {
	SPI_RSP_WAIT_DOWNLOAD_MODE = 1, /*Waiting for download command*/
	SPI_RSP_WAIT_FOR_KEY1_CERT_A0,
	SPI_RSP_WAIT_FOR_KEY2_CERT_A0,
	SPI_RSP_WAIT_FOR_IMAGE_CERT_A0,
	SPI_RSP_WAIT_IMAGE_SIZE,
	SPI_RSP_WAIT_FOR_IMAGE,
	SPI_RSP_DOWNLOAD_OK,
	SPI_RSP_BOOT_OK,
	SPI_RSP_ERROR_CS, /*Checksum/CRC error*/
	SPI_RSP_ERROR_CERTIFICATE, /*Got error certificate RSA/FW ver/Didn't get all the data before switching to image/...*/
	SPI_RSP_CMD_TOO_SHORT, /*Got command smaller than SPI_HEADER_SIZE. Each command must be at least this size.*/
	SPI_RSP_ERROR_LOADING_IN_DOWNLOAD, /*Error checking certificates or image, going to download mode*/
	SPI_RSP_WAIT_FOR_KEY1_CERT_B0 = 0x11,
	SPI_RSP_WAIT_FOR_KEY2_CERT_B0 = 0x12,
	SPI_RSP_WAIT_FOR_IMAGE_CERT_B0 = 0x13,
	SPI_RSP_WAIT_DBG_CERT_B0 = 0x14,
	SPI_RSP_WAIT_DBG_CERT_SIZE_B0 = 0x1e,
};

enum spi_cmd_dnld_e {
	SPI_BOOT_DOWNLOAD_RRAM_CMD_A0 = 0x40,
	SPI_BOOT_JUMP_RRAM_CMD,
	SPI_BOOT_DOWNLOAD_SRAM_CMD,
	SPI_BOOT_JUMP_SRAM_CMD,

	SPI_BOOT_DOWNLOAD_SEC_ICV_CMD_B0 = 0,
	SPI_BOOT_DOWNLOAD_SEC_OEM_CMD_B0 = 1,
	SPI_BOOT_GET_CHIP_VER_B0 = 2,
	SPI_BOOT_GET_CHIP_INFO_B0 = 3,
	SPI_BOOT_ERASE_DEBUG_CERT_B0 = 4,
	SPI_BOOT_DOWNLOAD_DEBUG_CERT_B0 = 5,
	SPI_BOOT_DOWNLOAD_SEC_IMAGE_DATA_B0 = 0xf,
	SPI_BOOT_DOWNLOAD_CERT_DATA_B0 = 0x10,
	SPI_BOOT_DEBUG_CERT_SIZE_B0 = 0x11,
};

enum spi_ul_e {
	SPI_UL_NO_PROTOCOL = 0,
	SPI_UL_ROM_BOOT_PROTO = 1,
	SPI_UL_EMB_TEST = 5,
};

int spi_proto_prepare_write_cmd(void *spi_handle, int do_exp_resp,
				uint8_t exp_resp, int read_len,
				struct stc *sstc, struct stc *hstc,
				enum chip_revision_e revision);
int spi_proto_send_stc(void *spi_handle, struct stc *sstc,
		       const struct stc *hstc, enum chip_revision_e revision);
int spi_proto_wait_for_device_flag(void *spi_handle, uint8_t flag_mask,
				   struct stc *sstc, struct stc *hstc,
				   int retries);

#define dump_full_hstc(log_lvl, s, ...)                                                             \
	do {                                                                                        \
		log_lvl(__VA_ARGS__);                                                               \
		log_lvl(" # host stc: {.rvd 0x%x, .rd %d, .prd %d, .wr %d, .ul 0x%x, .len %u} -- ", \
			s->host_flags.reserved, s->host_flags.read,                                 \
			s->host_flags.pre_read, s->host_flags.write, s->ul,                         \
			s->len);                                                                    \
		hexdump(log_lvl, (unsigned char *)s,                                                \
			sizeof(struct stc) + s->len);                                               \
	} while (0)

#define dump_full_sstc(log_lvl, s, ...)                                                                          \
	do {                                                                                                     \
		uint16_t l = s->len;                                                                             \
		log_lvl(__VA_ARGS__);                                                                            \
		log_lvl(" # soc stc: {.rvd 0x%x, .err %d, .rdy %d, .oact %d, .owait %d, .ul 0x%x, .len %u} -- ", \
			s->soc_flags.reserved, s->soc_flags.err,                                                 \
			s->soc_flags.ready, s->soc_flags.out_active,                                             \
			s->soc_flags.out_waiting, s->ul, l);                                                     \
		hexdump(log_lvl, (unsigned char *)s, sizeof(struct stc) + l);                                    \
	} while (0)

#define dump_decoded_hstc(log_lvl, s, ...)                                                        \
	do {                                                                                      \
		log_lvl(__VA_ARGS__);                                                             \
		log_lvl(" # host stc: {.rvd 0x%x, .rd %d, .prd %d, .wr %d, .ul 0x%x, .len %u}\n", \
			s->host_flags.reserved, s->host_flags.read,                               \
			s->host_flags.pre_read, s->host_flags.write, s->ul,                       \
			s->len);                                                                  \
	} while (0)

#define dump_decoded_sstc(log_lvl, s, ...)                                                                     \
	do {                                                                                                   \
		log_lvl(__VA_ARGS__);                                                                          \
		log_lvl(" # soc stc: {.rvd 0x%x, .err %d, .rdy %d, .oact %d, .owait %d, .ul 0x%x, .len %u}\n", \
			s->soc_flags.reserved, s->soc_flags.err,                                               \
			s->soc_flags.ready, s->soc_flags.out_active,                                           \
			s->soc_flags.out_waiting, s->ul, s->len);                                              \
	} while (0)

#define dump_raw_hstc(log_lvl, s, ...)                             \
	do {                                                           \
		log_lvl(__VA_ARGS__);                                      \
		log_lvl(" # host stc %d data bytes -- ", s->len);          \
		hexdump(log_lvl, (unsigned char *)s,                       \
			sizeof(struct stc) + s->len);                          \
	} while (0)

#define dump_raw_sstc(log_lvl, s, ...)                             \
	do {                                                           \
		log_lvl(__VA_ARGS__);                                      \
		log_lvl(" # soc stc %d data bytes -- ", s->len);           \
		hexdump(log_lvl, (unsigned char *)s,                       \
			sizeof(struct stc) + s->len);                          \
	} while (0)

#define dump_raw_hsstc(log_lvl, h, s, ...)                         \
	do {                                                           \
		log_lvl(__VA_ARGS__);                                      \
		log_lvl("%04x:", (uint32_t)(h->len + sizeof(struct stc))); \
		hexrawdump(log_lvl, (unsigned char *)h,                    \
			   sizeof(struct stc) + h->len);                       \
		log_lvl(":");                                              \
		hexrawdump(log_lvl, (unsigned char *)s,                    \
			   sizeof(struct stc) + h->len);                       \
		log_lvl("\n");                                             \
	} while (0)

#define dump_raw_buffer(log_lvl, t, r, l, ...)                     \
	do {                                                           \
		log_lvl(__VA_ARGS__);                                      \
		log_lvl("%04x:", l);                                       \
		hexrawdump(log_lvl, t, l);                                 \
		log_lvl(":");                                              \
		hexrawdump(log_lvl, r, l);                                 \
		log_lvl("\n");                                             \
	} while (0)

#endif /* __SPI_ROM_PROTOCOL_H__ */
