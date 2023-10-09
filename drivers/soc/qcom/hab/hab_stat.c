// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2018-2021, The Linux Foundation. All rights reserved.
 */
#include "hab.h"
#include "hab_grantable.h"

#define MAX_LINE_SIZE 128

int hab_stat_init(struct hab_driver *driver)
{
	return hab_stat_init_sub(driver);
}

int hab_stat_deinit(struct hab_driver *driver)
{
	return hab_stat_deinit_sub(driver);
}

/*
 * If all goes well the return value is the formated print and concatenated
 * original dest string.
 */
int hab_stat_buffer_print(char *dest,
		int dest_size, const char *fmt, ...)
{
	va_list args;
	char line[MAX_LINE_SIZE];
	int ret;

	va_start(args, fmt);
	ret = vsnprintf(line, sizeof(line), fmt, args);
	va_end(args);
	if (ret > 0)
		ret = strlcat(dest, line, dest_size);
	return ret;
}

int hab_stat_show_vchan(struct hab_driver *driver,
		char *buf, int size)
{
	int i, ret = 0;

	ret = strlcpy(buf, "", size);
	for (i = 0; i < driver->ndevices; i++) {
		struct hab_device *dev = &driver->devp[i];
		struct physical_channel *pchan;
		struct virtual_channel *vc;

		read_lock_bh(&dev->pchan_lock);
		list_for_each_entry(pchan, &dev->pchannels, node) {
			if (!pchan->vcnt)
				continue;

			ret = hab_stat_buffer_print(buf, size,
				"nm %s r %d lc %d rm %d sq_t %d sq_r %d st 0x%x vn %d:\n",
				pchan->name, pchan->is_be, pchan->vmid_local,
				pchan->vmid_remote, pchan->sequence_tx,
				pchan->sequence_rx, pchan->status, pchan->vcnt);

			read_lock(&pchan->vchans_lock);
			list_for_each_entry(vc, &pchan->vchannels, pnode) {
				ret = hab_stat_buffer_print(buf, size,
					"%08X(%d:%d:%lu:%lu:%d) ", vc->id,
					get_refcnt(vc->refcount),
					vc->otherend_closed,
					(unsigned long)vc->tx_cnt,
					(unsigned long)vc->rx_cnt,
					vc->rx_inflight);
			}
			ret = hab_stat_buffer_print(buf, size, "\n");
			read_unlock(&pchan->vchans_lock);
		}
		read_unlock_bh(&dev->pchan_lock);
	}

	return ret;
}

int hab_stat_show_ctx(struct hab_driver *driver,
		char *buf, int size)
{
	int ret = 0;
	struct uhab_context *ctx;

	ret = strlcpy(buf, "", size);

	spin_lock_bh(&hab_driver.drvlock);
	ret = hab_stat_buffer_print(buf, size,
					"Total contexts %d\n",
					driver->ctx_cnt);
	list_for_each_entry(ctx, &hab_driver.uctx_list, node) {
		ret = hab_stat_buffer_print(buf, size,
		"ctx %d K %d close %d vc %d exp %d imp %d open %d ref %d\n",
			ctx->owner, ctx->kernel, ctx->closing,
			ctx->vcnt, ctx->export_total,
			ctx->import_total, ctx->pending_cnt,
			get_refcnt(ctx->refcount));
	}
	spin_unlock_bh(&hab_driver.drvlock);

	return ret;
}

static int get_pft_tbl_total_size(struct compressed_pfns *pfn_table)
{
	int i, total_size = 0;

	for (i = 0; i < pfn_table->nregions; i++)
		total_size += pfn_table->region[i].size * PAGE_SIZE;

	return total_size;
}

static int print_ctx_total_expimp(struct uhab_context *ctx,
		char *buf, int size)
{
	struct compressed_pfns *pfn_table = NULL;
	int exp_total = 0, imp_total = 0;
	int exp_cnt = 0, imp_cnt = 0;
	struct export_desc *exp = NULL;
	int exim_size = 0;
	int ret = 0;

	read_lock(&ctx->exp_lock);
	list_for_each_entry(exp, &ctx->exp_whse, node) {
		pfn_table =	(struct compressed_pfns *)exp->payload;
		exim_size = get_pft_tbl_total_size(pfn_table);
		exp_total += exim_size;
		exp_cnt++;
	}
	read_unlock(&ctx->exp_lock);

	spin_lock_bh(&ctx->imp_lock);
	list_for_each_entry(exp, &ctx->imp_whse, node) {
		if (habmm_imp_hyp_map_check(ctx->import_ctx, exp)) {
			pfn_table =	(struct compressed_pfns *)exp->payload;
			exim_size = get_pft_tbl_total_size(pfn_table);
			imp_total += exim_size;
			imp_cnt++;
		}
	}
	spin_unlock_bh(&ctx->imp_lock);

	if (exp_cnt || exp_total || imp_cnt || imp_total)
		hab_stat_buffer_print(buf, size,
				"ctx %d exp %d size %d imp %d size %d\n",
				ctx->owner, exp_cnt, exp_total,
				imp_cnt, imp_total);
	else
		return 0;

	read_lock(&ctx->exp_lock);
	hab_stat_buffer_print(buf, size, "export[expid:vcid:size]: ");
	list_for_each_entry(exp, &ctx->exp_whse, node) {
		pfn_table =	(struct compressed_pfns *)exp->payload;
		exim_size = get_pft_tbl_total_size(pfn_table);
		hab_stat_buffer_print(buf, size,
			"[%d:%x:%d] ", exp->export_id,
			exp->vcid_local, exim_size);
	}
	hab_stat_buffer_print(buf, size, "\n");
	read_unlock(&ctx->exp_lock);

	spin_lock_bh(&ctx->imp_lock);
	hab_stat_buffer_print(buf, size, "import[expid:vcid:size]: ");
	list_for_each_entry(exp, &ctx->imp_whse, node) {
		if (habmm_imp_hyp_map_check(ctx->import_ctx, exp)) {
			pfn_table =	(struct compressed_pfns *)exp->payload;
			exim_size = get_pft_tbl_total_size(pfn_table);
			hab_stat_buffer_print(buf, size,
				"[%d:%x:%d] ", exp->export_id,
				exp->vcid_local, exim_size);
		}
	}
	ret = hab_stat_buffer_print(buf, size, "\n");
	spin_unlock_bh(&ctx->imp_lock);

	return ret;
}

int hab_stat_show_expimp(struct hab_driver *driver,
		int pid, char *buf, int size)
{
	struct uhab_context *ctx = NULL;
	int ret = 0;
	struct virtual_channel *vchan = NULL;
	int mmid = 0;
	struct physical_channel *pchans[HABCFG_MMID_NUM];
	int pchan_count = 0;

	(void)driver;

	ret = strlcpy(buf, "", size);

	spin_lock_bh(&hab_driver.drvlock);
	list_for_each_entry(ctx, &hab_driver.uctx_list, node) {
		if (pid == ctx->owner) {
			ret = print_ctx_total_expimp(ctx, buf, size);

			list_for_each_entry(vchan, &ctx->vchannels, node) {
				if (vchan->pchan->habdev->id != mmid) {
					mmid = vchan->pchan->habdev->id;
					pchans[pchan_count++] = vchan->pchan;
					if (pchan_count >= HABCFG_MMID_NUM)
						break;
				}
			}
			break;
		}
	}
	spin_unlock_bh(&hab_driver.drvlock);

	/* print pchannel status, drvlock is not required */
	if (pchan_count > 0)
		ret = hab_stat_log(pchans, pchan_count, buf, size);

	return ret;
}

#define HAB_PIPE_DUMP_FILE_NAME "/sdcard/habpipe-"
#define HAB_PIPE_DUMP_FILE_EXT ".dat"

#define HAB_PIPEDUMP_SIZE (768*1024*4)
static char *filp;
static int pipedump_idx;

int dump_hab_open(void)
{
	int rc = 0;
	char file_path[256];
	char file_time[100];

	rc = dump_hab_get_file_name(file_time, sizeof(file_time));
	strlcpy(file_path, HAB_PIPE_DUMP_FILE_NAME, sizeof(file_path));
	strlcat(file_path, file_time, sizeof(file_path));
	strlcat(file_path, HAB_PIPE_DUMP_FILE_EXT, sizeof(file_path));

	filp = vmalloc(HAB_PIPEDUMP_SIZE);
	if (IS_ERR(filp)) {
		rc = PTR_ERR(filp);
		pr_err("failed to create pipe dump buffer rc %d\n", rc);
		filp = NULL;
	} else {
		pr_info("hab pipe dump buffer opened %s\n", file_path);
		pipedump_idx = 0;
		dump_hab_buf(file_path, strlen(file_path)); /* id first */
	}
	return rc;
}

void dump_hab_close(void)
{
	pr_info("pipe dump content size %d completed\n", pipedump_idx);
	/* transfer buffer ownership to devcoredump */
	filp = NULL;
	pipedump_idx = 0;
}

int dump_hab_buf(void *buf, int size)
{
	if (!buf || !size || size > HAB_PIPEDUMP_SIZE - pipedump_idx) {
		pr_err("wrong parameters buf %pK size %d allowed %d\n",
			 buf, size, HAB_PIPEDUMP_SIZE - pipedump_idx);
		return 0;
	}

	memcpy(&filp[pipedump_idx], buf, size);
	pipedump_idx += size;
	return size;
}

void dump_hab(int mmid)
{
	struct physical_channel *pchan = NULL;
	int i = 0;
	char str[8] = {35, 35, 35, 35, 35, 35, 35, 35}; /* ## */

	dump_hab_open();
	for (i = 0; i < hab_driver.ndevices; i++) {
		struct hab_device *habdev = &hab_driver.devp[i];

		if (habdev->id == mmid) {
			list_for_each_entry(pchan, &habdev->pchannels, node) {
				if (pchan->vcnt > 0) {
					pr_info("***** dump pchan %s vcnt %d *****\n",
						pchan->name, pchan->vcnt);
					hab_pipe_read_dump(pchan);
					break;
				}
			}
			dump_hab_buf(str, 8); /* separator */
		}
	}
	dev_coredumpv(hab_driver.dev, filp, pipedump_idx, GFP_KERNEL);
	dump_hab_close();
}
