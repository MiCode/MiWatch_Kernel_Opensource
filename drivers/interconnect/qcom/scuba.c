// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 *
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/interconnect-provider.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <dt-bindings/interconnect/qcom,scuba.h>

#include <linux/soc/qcom/smd-rpm.h>
#include <soc/qcom/rpm-smd.h>

#include "icc-rpm.h"
#include "qnoc-qos.h"
#include "rpm-ids.h"

static LIST_HEAD(qnoc_probe_list);
static DEFINE_MUTEX(probe_list_lock);

static int probe_count;

static const struct clk_bulk_data bus_clocks[] = {
	{ .id = "bus" },
	{ .id = "bus_a" },
};

static struct qcom_icc_qosbox apps_proc_qos = {
	.regs = icc_bimc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 1,
	.offsets = { 0x8300 },
	.config = &(struct qos_config) {
		.prio = 0,
		.bke_enable = 1,
	 },
};

static struct qcom_icc_node apps_proc = {
	.name = "mas_apps_proc",
	.id = MASTER_AMPSS_M0,
	.channels = 1,
	.buswidth = 16,
	.noc_ops = &qcom_bimc_ops,
	.qosbox = &apps_proc_qos,
	.mas_rpm_id = ICBID_MASTER_APPSS_PROC,
	.slv_rpm_id = -1,
	.num_links = 2,
	.links = { SLAVE_EBI_CH0, BIMC_SNOC_SLV },
};

static struct qcom_icc_qosbox mas_snoc_bimc_rt_qos = {
	.regs = icc_bimc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 1,
	.offsets = { 0x10300 },
	.config = &(struct qos_config) {
		.prio = 0,
		.bke_enable = 0,
	 },
};

static struct qcom_icc_node mas_snoc_bimc_rt = {
	.name = "mas_snoc_bimc_rt",
	.id = MASTER_SNOC_BIMC_RT,
	.channels = 1,
	.buswidth = 16,
	.noc_ops = &qcom_bimc_ops,
	.qosbox = &mas_snoc_bimc_rt_qos,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 1,
	.links = { SLAVE_EBI_CH0 },
};

static struct qcom_icc_qosbox mas_snoc_bimc_nrt_qos = {
	.regs = icc_bimc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 1,
	.offsets = { 0x14300 },
	.config = &(struct qos_config) {
		.prio = 0,
		.bke_enable = 0,
	 },
};

static struct qcom_icc_node mas_snoc_bimc_nrt = {
	.name = "mas_snoc_bimc_nrt",
	.id = MASTER_SNOC_BIMC_NRT,
	.channels = 1,
	.buswidth = 16,
	.noc_ops = &qcom_bimc_ops,
	.qosbox = &mas_snoc_bimc_nrt_qos,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 1,
	.links = { SLAVE_EBI_CH0 },
};

static struct qcom_icc_qosbox mas_snoc_bimc_qos = {
	.regs = icc_bimc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 1,
	.offsets = { 0x20300 },
	.config = &(struct qos_config) {
		.prio = 0,
		.bke_enable = 0,
	 },
};

static struct qcom_icc_node mas_snoc_bimc = {
	.name = "mas_snoc_bimc",
	.id = SNOC_BIMC_MAS,
	.channels = 1,
	.buswidth = 16,
	.noc_ops = &qcom_bimc_ops,
	.qosbox = &mas_snoc_bimc_qos,
	.mas_rpm_id = ICBID_MASTER_SNOC_BIMC,
	.slv_rpm_id = -1,
	.num_links = 1,
	.links = { SLAVE_EBI_CH0 },
};

static struct qcom_icc_qosbox qnm_gpu_qos = {
	.regs = icc_bimc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 1,
	.offsets = { 0xC300 },
	.config = &(struct qos_config) {
		.prio = 0,
		.bke_enable = 0,
	 },
};

static struct qcom_icc_node qnm_gpu = {
	.name = "qnm_gpu",
	.id = MASTER_GRAPHICS_3D,
	.channels = 1,
	.buswidth = 32,
	.mas_rpm_id = -1,
	.noc_ops = &qcom_bimc_ops,
	.qosbox = &qnm_gpu_qos,
	.slv_rpm_id = -1,
	.num_links = 1,
	.links = { SLAVE_EBI_CH0 },
};

static struct qcom_icc_qosbox tcu_0_qos = {
	.regs = icc_bimc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 1,
	.offsets = { 0x18300 },
	.config = &(struct qos_config) {
		.prio = 6,
		.bke_enable = 0,
	 },
};

static struct qcom_icc_node tcu_0 = {
	.name = "tcu_0",
	.id = MASTER_TCU_0,
	.channels = 1,
	.buswidth = 8,
	.noc_ops = &qcom_bimc_ops,
	.qosbox = &tcu_0_qos,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 2,
	.links = { SLAVE_EBI_CH0, BIMC_SNOC_SLV },
};

static struct qcom_icc_node qup0_core_master = {
	.name = "qup0_core_master",
	.id = MASTER_QUP_CORE_0,
	.channels = 1,
	.buswidth = 4,
	.mas_rpm_id = ICBID_MASTER_QUP_CORE_0,
	.slv_rpm_id = -1,
	.num_links = 1,
	.links = { SLAVE_QUP_CORE_0 },
};

static struct qcom_icc_qosbox crypto_c0_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 1,
	.offsets = { 0x2b000, },
	.config = &(struct qos_config) {
		.prio = 2,
		.urg_fwd = 0,
	},
};

static struct qcom_icc_node crypto_c0 = {
	.name = "crypto_c0",
	.id = MASTER_CRYPTO_CORE0,
	.channels = 1,
	.buswidth = 8,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &crypto_c0_qos,
	.mas_rpm_id = ICBID_MASTER_CRYPTO_CORE0,
	.slv_rpm_id = -1,
	.num_links = 1,
	.links = { SLAVE_ANOC_SNOC },
};

static struct qcom_icc_node qnm_snoc_cnoc = {
	.name = "qnm_snoc_cnoc",
	.id = SNOC_CNOC_MAS,
	.channels = 1,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 33,
	.links = { SLAVE_BIMC_CFG, SLAVE_CRYPTO_0_CFG,
		   SLAVE_CAMERA_NRT_THROTTLE_CFG, SLAVE_CAMERA_RT_THROTTLE_CFG,
		   SLAVE_CAMERA_CFG, SLAVE_CLK_CTL,
		   SLAVE_DISPLAY_CFG, SLAVE_DISPLAY_THROTTLE_CFG,
		   SLAVE_GPU_CFG, SLAVE_QUP_0,
		   SLAVE_HWKM, SLAVE_IMEM_CFG,
		   SLAVE_IPA_CFG, SLAVE_LPASS,
		   SLAVE_MESSAGE_RAM, SLAVE_PRNG,
		   SLAVE_PDM, SLAVE_PIMEM_CFG,
		   SLAVE_PKA_WRAPPER_CFG, SLAVE_PMIC_ARB,
		   SLAVE_QDSS_CFG, SLAVE_QM_CFG,
		   SLAVE_QM_MPU_CFG, SLAVE_QPIC,
		   SLAVE_SDCC_1, SLAVE_SDCC_2,
		   SLAVE_SNOC_CFG, SLAVE_TCSR,
		   SLAVE_USB3, SLAVE_VENUS_CFG,
		   SLAVE_VENUS_THROTTLE_CFG, SLAVE_VSENSE_CTRL_CFG,
		   SLAVE_SERVICE_CNOC },
};

static struct qcom_icc_node xm_dap = {
	.name = "xm_dap",
	.id = MASTER_QDSS_DAP,
	.channels = 1,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 33,
	.links = { SLAVE_BIMC_CFG, SLAVE_CRYPTO_0_CFG,
		   SLAVE_CAMERA_NRT_THROTTLE_CFG, SLAVE_CAMERA_RT_THROTTLE_CFG,
		   SLAVE_CAMERA_CFG, SLAVE_CLK_CTL,
		   SLAVE_DISPLAY_CFG, SLAVE_DISPLAY_THROTTLE_CFG,
		   SLAVE_GPU_CFG, SLAVE_QUP_0,
		   SLAVE_HWKM, SLAVE_IMEM_CFG,
		   SLAVE_IPA_CFG, SLAVE_LPASS,
		   SLAVE_MESSAGE_RAM, SLAVE_PRNG,
		   SLAVE_PDM, SLAVE_PIMEM_CFG,
		   SLAVE_PKA_WRAPPER_CFG, SLAVE_PMIC_ARB,
		   SLAVE_QDSS_CFG, SLAVE_QM_CFG,
		   SLAVE_QM_MPU_CFG, SLAVE_QPIC,
		   SLAVE_SDCC_1, SLAVE_SDCC_2,
		   SLAVE_SNOC_CFG, SLAVE_TCSR,
		   SLAVE_USB3, SLAVE_VENUS_CFG,
		   SLAVE_VENUS_THROTTLE_CFG, SLAVE_VSENSE_CTRL_CFG,
		   SLAVE_SERVICE_CNOC },
};

static struct qcom_icc_qosbox qnm_camera_nrt_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 1,
	.offsets = { 0x19000, },
	.config = &(struct qos_config) {
		.prio = 3,
		.urg_fwd = 0,
	},
};

static struct qcom_icc_node qnm_camera_nrt = {
	.name = "qnm_camera_nrt",
	.id = MASTER_CAMNOC_SF,
	.channels = 1,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &qnm_camera_nrt_qos,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 1,
	.links = { SLAVE_SNOC_BIMC_NRT },
};

static struct qcom_icc_qosbox venus0_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 1,
	.offsets = { 0x1e000, },
	.config = &(struct qos_config) {
		.prio = 3,
		.urg_fwd = 1,
	},
};

static struct qcom_icc_node qxm_venus0 = {
	.name = "qxm_venus0",
	.id = MASTER_VIDEO_P0,
	.channels = 1,
	.buswidth = 16,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &venus0_qos,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 1,
	.links = { SLAVE_SNOC_BIMC_NRT },
};

static struct qcom_icc_qosbox venus_cpu_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 1,
	.offsets = { 0x22000, },
	.config = &(struct qos_config) {
		.prio = 4,
		.urg_fwd = 0,
	},
};

static struct qcom_icc_node qxm_venus_cpu = {
	.name = "qxm_venus_cpu",
	.id = MASTER_VIDEO_PROC,
	.channels = 1,
	.buswidth = 8,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &venus_cpu_qos,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 1,
	.links = { SLAVE_SNOC_BIMC_NRT },
};

static struct qcom_icc_qosbox camera_rt_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 1,
	.offsets = { 0x1f000, },
	.config = &(struct qos_config) {
		.prio = 3,
		.urg_fwd = 1,
	},
};

static struct qcom_icc_node qnm_camera_rt = {
	.name = "qnm_camera_rt",
	.id = MASTER_CAMNOC_HF,
	.channels = 1,
	.buswidth = 32,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &camera_rt_qos,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 1,
	.links = { SLAVE_SNOC_BIMC_RT },
};

static struct qcom_icc_qosbox mdp0_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 1,
	.offsets = { 0x1a000, },
	.config = &(struct qos_config) {
		.prio = 3,
		.urg_fwd = 1,
	},
};

static struct qcom_icc_node qxm_mdp0 = {
	.name = "qxm_mdp0",
	.id = MASTER_MDP_PORT0,
	.channels = 1,
	.buswidth = 16,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &mdp0_qos,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 1,
	.links = { SLAVE_SNOC_BIMC_RT },
};

static struct qcom_icc_node qhm_snoc_cfg = {
	.name = "qhm_snoc_cfg",
	.id = MASTER_SNOC_CFG,
	.channels = 1,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 1,
	.links = { SLAVE_SERVICE_SNOC },
};

static struct qcom_icc_qosbox tic_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 1,
	.offsets = { 0x1d000, },
	.config = &(struct qos_config) {
		.prio = 2,
		.urg_fwd = 0,
	},
};

static struct qcom_icc_node qhm_tic = {
	.name = "qhm_tic",
	.id = MASTER_TIC,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &tic_qos,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 7,
	.links = { SLAVE_APPSS, SNOC_CNOC_SLV,
		   SLAVE_OCIMEM, SLAVE_PIMEM,
		   SNOC_BIMC_SLV, SLAVE_QDSS_STM,
		   SLAVE_TCU },
};

static struct qcom_icc_node qnm_anoc_snoc = {
	.name = "qnm_anoc_snoc",
	.id = MASTER_ANOC_SNOC,
	.channels = 1,
	.buswidth = 16,
	.mas_rpm_id = ICBID_MASTER_ANOC_SNOC,
	.slv_rpm_id = -1,
	.num_links = 7,
	.links = { SLAVE_APPSS, SNOC_CNOC_SLV,
		   SLAVE_OCIMEM, SLAVE_PIMEM,
		   SNOC_BIMC_SLV, SLAVE_QDSS_STM,
		   SLAVE_TCU },
};

static struct qcom_icc_node qxm_bimc_snoc = {
	.name = "qxm_bimc_snoc",
	.id = BIMC_SNOC_MAS,
	.channels = 1,
	.buswidth = 8,
	.mas_rpm_id = ICBID_MASTER_BIMC_SNOC,
	.slv_rpm_id = -1,
	.num_links = 6,
	.links = { SLAVE_APPSS, SNOC_CNOC_SLV,
		   SLAVE_OCIMEM, SLAVE_PIMEM,
		   SLAVE_QDSS_STM, SLAVE_TCU },
};

static struct qcom_icc_qosbox pimem_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 1,
	.offsets = { 0x29000, },
	.config = &(struct qos_config) {
		.prio = 2,
		.urg_fwd = 0,
	},
};

static struct qcom_icc_node qxm_pimem = {
	.name = "qxm_pimem",
	.id = MASTER_PIMEM,
	.channels = 1,
	.buswidth = 8,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &pimem_qos,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 2,
	.links = { SLAVE_OCIMEM, SNOC_BIMC_SLV },
};

static struct qcom_icc_qosbox qdss_bam_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 1,
	.offsets = { 0x17000, },
	.config = &(struct qos_config) {
		.prio = 2,
		.urg_fwd = 0,
	},
};

static struct qcom_icc_node qhm_qdss_bam = {
	.name = "qhm_qdss_bam",
	.id = MASTER_QDSS_BAM,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &qdss_bam_qos,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 1,
	.links = { SLAVE_ANOC_SNOC },
};

static struct qcom_icc_qosbox qpic_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 1,
	.offsets = { 0x16000, },
	.config = &(struct qos_config) {
		.prio = 2,
		.urg_fwd = 0,
	},
};

static struct qcom_icc_node qhm_qpic = {
	.name = "qhm_qpic",
	.id = MASTER_QPIC,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &qpic_qos,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 1,
	.links = { SLAVE_ANOC_SNOC },
};

static struct qcom_icc_qosbox qup0_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 1,
	.offsets = { 0x15000, },
	.config = &(struct qos_config) {
		.prio = 2,
		.urg_fwd = 0,
	},
};

static struct qcom_icc_node qhm_qup0 = {
	.name = "qhm_qup0",
	.id = MASTER_QUP_0,
	.channels = 1,
	.buswidth = 4,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &qup0_qos,
	.mas_rpm_id = ICBID_MASTER_QUP_0,
	.slv_rpm_id = -1,
	.num_links = 1,
	.links = { SLAVE_ANOC_SNOC },
};

static struct qcom_icc_qosbox ipa_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 1,
	.offsets = { 0x18000, },
	.config = &(struct qos_config) {
		.prio = 2,
		.urg_fwd = 0,
	},
};

static struct qcom_icc_node qxm_ipa = {
	.name = "qxm_ipa",
	.id = MASTER_IPA,
	.channels = 1,
	.buswidth = 8,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &ipa_qos,
	.mas_rpm_id = ICBID_MASTER_IPA,
	.slv_rpm_id = -1,
	.num_links = 1,
	.links = { SLAVE_ANOC_SNOC },
};

static struct qcom_icc_qosbox qdss_etr_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 1,
	.offsets = { 0x21000, },
	.config = &(struct qos_config) {
		.prio = 2,
		.urg_fwd = 0,
	},
};

static struct qcom_icc_node xm_qdss_etr = {
	.name = "xm_qdss_etr",
	.id = MASTER_QDSS_ETR,
	.channels = 1,
	.buswidth = 8,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &qdss_etr_qos,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 1,
	.links = { SLAVE_ANOC_SNOC },
};

static struct qcom_icc_qosbox sdc1_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 1,
	.offsets = { 0x26000, },
	.config = &(struct qos_config) {
		.prio = 2,
		.urg_fwd = 0,
	},
};

static struct qcom_icc_node xm_sdc1 = {
	.name = "xm_sdc1",
	.id = MASTER_SDCC_1,
	.channels = 1,
	.buswidth = 8,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &sdc1_qos,
	.mas_rpm_id = ICBID_MASTER_SDCC_1,
	.slv_rpm_id = -1,
	.num_links = 1,
	.links = { SLAVE_ANOC_SNOC },
};

static struct qcom_icc_qosbox sdc2_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 1,
	.offsets = { 0x2c000, },
	.config = &(struct qos_config) {
		.prio = 2,
		.urg_fwd = 0,
	},
};

static struct qcom_icc_node xm_sdc2 = {
	.name = "xm_sdc2",
	.id = MASTER_SDCC_2,
	.channels = 1,
	.buswidth = 8,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &sdc2_qos,
	.mas_rpm_id = ICBID_MASTER_SDCC_2,
	.slv_rpm_id = -1,
	.num_links = 1,
	.links = { SLAVE_ANOC_SNOC },
};

static struct qcom_icc_qosbox usb3_0_qos = {
	.regs = icc_qnoc_qos_regs[ICC_QNOC_QOSGEN_TYPE_RPMH],
	.num_ports = 1,
	.offsets = { 0x2d000, },
	.config = &(struct qos_config) {
		.prio = 2,
		.urg_fwd = 0,
	},
};

static struct qcom_icc_node xm_usb3_0 = {
	.name = "xm_usb3_0",
	.id = MASTER_USB3,
	.channels = 1,
	.buswidth = 8,
	.noc_ops = &qcom_qnoc4_ops,
	.qosbox = &usb3_0_qos,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 1,
	.links = { SLAVE_ANOC_SNOC },
};

static struct qcom_icc_node ebi = {
	.name = "ebi",
	.id = SLAVE_EBI_CH0,
	.channels = 1,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = ICBID_SLAVE_EBI1,
	.num_links = 0,
};

static struct qcom_icc_node qxs_bimc_snoc = {
	.name = "qxs_bimc_snoc",
	.id = BIMC_SNOC_SLV,
	.channels = 1,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = ICBID_SLAVE_BIMC_SNOC,
	.num_links = 1,
	.links = { BIMC_SNOC_MAS },
};

static struct qcom_icc_node qup0_core_slave = {
	.name = "qup0_core_slave",
	.id = SLAVE_QUP_CORE_0,
	.channels = 1,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 0,
};

static struct qcom_icc_node qhs_bimc_cfg = {
	.name = "qhs_bimc_cfg",
	.id = SLAVE_BIMC_CFG,
	.channels = 1,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 0,
};

static struct qcom_icc_node qhs_camera_nrt_throttle_cfg = {
	.name = "qhs_camera_nrt_throttle_cfg",
	.id = SLAVE_CAMERA_NRT_THROTTLE_CFG,
	.channels = 1,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 0,
};

static struct qcom_icc_node qhs_camera_rt_throttle_cfg = {
	.name = "qhs_camera_rt_throttle_cfg",
	.id = SLAVE_CAMERA_RT_THROTTLE_CFG,
	.channels = 1,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 0,
};

static struct qcom_icc_node qhs_camera_ss_cfg = {
	.name = "qhs_camera_ss_cfg",
	.id = SLAVE_CAMERA_CFG,
	.channels = 1,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 0,
};

static struct qcom_icc_node qhs_clk_ctl = {
	.name = "qhs_clk_ctl",
	.id = SLAVE_CLK_CTL,
	.channels = 1,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 0,
};

static struct qcom_icc_node qhs_crypto0_cfg = {
	.name = "qhs_crypto0_cfg",
	.id = SLAVE_CRYPTO_0_CFG,
	.channels = 1,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 0,
};

static struct qcom_icc_node qhs_disp_ss_cfg = {
	.name = "qhs_disp_ss_cfg",
	.id = SLAVE_DISPLAY_CFG,
	.channels = 1,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 0,
};

static struct qcom_icc_node qhs_display_throttle_cfg = {
	.name = "qhs_display_throttle_cfg",
	.id = SLAVE_DISPLAY_THROTTLE_CFG,
	.channels = 1,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 0,
};

static struct qcom_icc_node qhs_gpu_cfg = {
	.name = "qhs_gpu_cfg",
	.id = SLAVE_GPU_CFG,
	.channels = 1,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 0,
};

static struct qcom_icc_node qhs_hwkm = {
	.name = "qhs_hwkm",
	.id = SLAVE_HWKM,
	.channels = 1,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 0,
};

static struct qcom_icc_node qhs_imem_cfg = {
	.name = "qhs_imem_cfg",
	.id = SLAVE_IMEM_CFG,
	.channels = 1,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 0,
};

static struct qcom_icc_node qhs_ipa_cfg = {
	.name = "qhs_ipa_cfg",
	.id = SLAVE_IPA_CFG,
	.channels = 1,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 0,
};

static struct qcom_icc_node qhs_lpass = {
	.name = "qhs_lpass",
	.id = SLAVE_LPASS,
	.channels = 1,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 0,
};

static struct qcom_icc_node qhs_mesg_ram = {
	.name = "qhs_mesg_ram",
	.id = SLAVE_MESSAGE_RAM,
	.channels = 1,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 0,
};

static struct qcom_icc_node qhs_pdm = {
	.name = "qhs_pdm",
	.id = SLAVE_PDM,
	.channels = 1,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 0,
};

static struct qcom_icc_node qhs_pimem_cfg = {
	.name = "qhs_pimem_cfg",
	.id = SLAVE_PIMEM_CFG,
	.channels = 1,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 0,
};

static struct qcom_icc_node qhs_pka_wrapper = {
	.name = "qhs_pka_wrapper",
	.id = SLAVE_PKA_WRAPPER_CFG,
	.channels = 1,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 0,
};

static struct qcom_icc_node qhs_pmic_arb = {
	.name = "qhs_pmic_arb",
	.id = SLAVE_PMIC_ARB,
	.channels = 1,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 0,
};

static struct qcom_icc_node qhs_prng = {
	.name = "qhs_prng",
	.id = SLAVE_PRNG,
	.channels = 1,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 0,
};

static struct qcom_icc_node qhs_qdss_cfg = {
	.name = "qhs_qdss_cfg",
	.id = SLAVE_QDSS_CFG,
	.channels = 1,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 0,
};

static struct qcom_icc_node qhs_qm_cfg = {
	.name = "qhs_qm_cfg",
	.id = SLAVE_QM_CFG,
	.channels = 1,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 0,
};

static struct qcom_icc_node qhs_qm_mpu_cfg = {
	.name = "qhs_qm_mpu_cfg",
	.id = SLAVE_QM_MPU_CFG,
	.channels = 1,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 0,
};

static struct qcom_icc_node qhs_qpic = {
	.name = "qhs_qpic",
	.id = SLAVE_QPIC,
	.channels = 1,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 0,
};

static struct qcom_icc_node qhs_qup0 = {
	.name = "qhs_qup0",
	.id = SLAVE_QUP_0,
	.channels = 1,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 0,
};

static struct qcom_icc_node qhs_sdc1 = {
	.name = "qhs_sdc1",
	.id = SLAVE_SDCC_1,
	.channels = 1,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 0,
};

static struct qcom_icc_node qhs_sdc2 = {
	.name = "qhs_sdc2",
	.id = SLAVE_SDCC_2,
	.channels = 1,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 0,
};

static struct qcom_icc_node qhs_snoc_cfg = {
	.name = "qhs_snoc_cfg",
	.id = SLAVE_SNOC_CFG,
	.channels = 1,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 1,
	.links = { MASTER_SNOC_CFG },
};

static struct qcom_icc_node qhs_tcsr = {
	.name = "qhs_tcsr",
	.id = SLAVE_TCSR,
	.channels = 1,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 0,
};

static struct qcom_icc_node qhs_usb3 = {
	.name = "qhs_usb3",
	.id = SLAVE_USB3,
	.channels = 1,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 0,
};

static struct qcom_icc_node qhs_venus_cfg = {
	.name = "qhs_venus_cfg",
	.id = SLAVE_VENUS_CFG,
	.channels = 1,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 0,
};

static struct qcom_icc_node qhs_venus_throttle_cfg = {
	.name = "qhs_venus_throttle_cfg",
	.id = SLAVE_VENUS_THROTTLE_CFG,
	.channels = 1,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 0,
};

static struct qcom_icc_node qhs_vsense_ctrl_cfg = {
	.name = "qhs_vsense_ctrl_cfg",
	.id = SLAVE_VSENSE_CTRL_CFG,
	.channels = 1,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 0,
};

static struct qcom_icc_node srvc_cnoc = {
	.name = "srvc_cnoc",
	.id = SLAVE_SERVICE_CNOC,
	.channels = 1,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 0,
};

static struct qcom_icc_node slv_snoc_bimc_nrt = {
	.name = "slv_snoc_bimc_nrt",
	.id = SLAVE_SNOC_BIMC_NRT,
	.channels = 1,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 1,
	.links = { MASTER_SNOC_BIMC_NRT },
};

static struct qcom_icc_node slv_snoc_bimc_rt = {
	.name = "slv_snoc_bimc_rt",
	.id = SLAVE_SNOC_BIMC_RT,
	.channels = 1,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 1,
	.links = { MASTER_SNOC_BIMC_RT },
};

static struct qcom_icc_node qhs_apss = {
	.name = "qhs_apss",
	.id = SLAVE_APPSS,
	.channels = 1,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 0,
};

static struct qcom_icc_node qns_snoc_cnoc = {
	.name = "qns_snoc_cnoc",
	.id = SNOC_CNOC_SLV,
	.channels = 1,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = ICBID_SLAVE_SNOC_CNOC,
	.num_links = 1,
	.links = { SNOC_CNOC_MAS },
};

static struct qcom_icc_node qxs_imem = {
	.name = "qxs_imem",
	.id = SLAVE_OCIMEM,
	.channels = 1,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = ICBID_SLAVE_IMEM,
	.num_links = 0,
};

static struct qcom_icc_node qxs_pimem = {
	.name = "qxs_pimem",
	.id = SLAVE_PIMEM,
	.channels = 1,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 0,
};

static struct qcom_icc_node qxs_snoc_bimc = {
	.name = "qxs_snoc_bimc",
	.id = SNOC_BIMC_SLV,
	.channels = 1,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = ICBID_SLAVE_SNOC_BIMC,
	.num_links = 1,
};

static struct qcom_icc_node srvc_snoc = {
	.name = "srvc_snoc",
	.id = SLAVE_SERVICE_SNOC,
	.channels = 1,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 0,
};

static struct qcom_icc_node xs_qdss_stm = {
	.name = "xs_qdss_stm",
	.id = SLAVE_QDSS_STM,
	.channels = 1,
	.buswidth = 4,
	.mas_rpm_id = -1,
	.slv_rpm_id = ICBID_SLAVE_QDSS_STM,
	.num_links = 0,
};

static struct qcom_icc_node xs_sys_tcu_cfg = {
	.name = "xs_sys_tcu_cfg",
	.id = SLAVE_TCU,
	.channels = 1,
	.buswidth = 8,
	.mas_rpm_id = -1,
	.slv_rpm_id = -1,
	.num_links = 0,
};

static struct qcom_icc_node qns_anoc_snoc = {
	.name = "qns_anoc_snoc",
	.id = SLAVE_ANOC_SNOC,
	.channels = 1,
	.buswidth = 16,
	.mas_rpm_id = -1,
	.slv_rpm_id = ICBID_SLAVE_ANOC_SNOC,
	.num_links = 1,
	.links = { MASTER_ANOC_SNOC },
};

static struct qcom_icc_node *bimc_nodes[] = {
	[MASTER_AMPSS_M0] = &apps_proc,
	[MASTER_SNOC_BIMC_RT] = &mas_snoc_bimc_rt,
	[MASTER_SNOC_BIMC_NRT] = &mas_snoc_bimc_nrt,
	[SNOC_BIMC_MAS] = &mas_snoc_bimc,
	[MASTER_GRAPHICS_3D] = &qnm_gpu,
	[MASTER_TCU_0] = &tcu_0,
	[SLAVE_EBI_CH0] = &ebi,
	[BIMC_SNOC_SLV] = &qxs_bimc_snoc,
};

static struct qcom_icc_desc scuba_bimc = {
	.nodes = bimc_nodes,
	.num_nodes = ARRAY_SIZE(bimc_nodes),
};

static struct qcom_icc_node *clk_virt_nodes[] = {
	[MASTER_QUP_CORE_0] = &qup0_core_master,
	[SLAVE_QUP_CORE_0] = &qup0_core_slave,
};

static struct qcom_icc_desc scuba_clk_virt = {
	.nodes = clk_virt_nodes,
	.num_nodes = ARRAY_SIZE(clk_virt_nodes),
};

static struct qcom_icc_node *config_noc_nodes[] = {
	[SNOC_CNOC_MAS] = &qnm_snoc_cnoc,
	[MASTER_QDSS_DAP] = &xm_dap,
	[SLAVE_BIMC_CFG] = &qhs_bimc_cfg,
	[SLAVE_CAMERA_NRT_THROTTLE_CFG] = &qhs_camera_nrt_throttle_cfg,
	[SLAVE_CAMERA_RT_THROTTLE_CFG] = &qhs_camera_rt_throttle_cfg,
	[SLAVE_CAMERA_CFG] = &qhs_camera_ss_cfg,
	[SLAVE_CLK_CTL] = &qhs_clk_ctl,
	[SLAVE_CRYPTO_0_CFG] = &qhs_crypto0_cfg,
	[SLAVE_DISPLAY_CFG] = &qhs_disp_ss_cfg,
	[SLAVE_DISPLAY_THROTTLE_CFG] = &qhs_display_throttle_cfg,
	[SLAVE_GPU_CFG] = &qhs_gpu_cfg,
	[SLAVE_HWKM] = &qhs_hwkm,
	[SLAVE_IMEM_CFG] = &qhs_imem_cfg,
	[SLAVE_IPA_CFG] = &qhs_ipa_cfg,
	[SLAVE_LPASS] = &qhs_lpass,
	[SLAVE_MESSAGE_RAM] = &qhs_mesg_ram,
	[SLAVE_PDM] = &qhs_pdm,
	[SLAVE_PIMEM_CFG] = &qhs_pimem_cfg,
	[SLAVE_PKA_WRAPPER_CFG] = &qhs_pka_wrapper,
	[SLAVE_PMIC_ARB] = &qhs_pmic_arb,
	[SLAVE_PRNG] = &qhs_prng,
	[SLAVE_QDSS_CFG] = &qhs_qdss_cfg,
	[SLAVE_QM_CFG] = &qhs_qm_cfg,
	[SLAVE_QM_MPU_CFG] = &qhs_qm_mpu_cfg,
	[SLAVE_QPIC] = &qhs_qpic,
	[SLAVE_QUP_0] = &qhs_qup0,
	[SLAVE_SDCC_1] = &qhs_sdc1,
	[SLAVE_SDCC_2] = &qhs_sdc2,
	[SLAVE_SNOC_CFG] = &qhs_snoc_cfg,
	[SLAVE_TCSR] = &qhs_tcsr,
	[SLAVE_USB3] = &qhs_usb3,
	[SLAVE_VENUS_CFG] = &qhs_venus_cfg,
	[SLAVE_VENUS_THROTTLE_CFG] = &qhs_venus_throttle_cfg,
	[SLAVE_VSENSE_CTRL_CFG] = &qhs_vsense_ctrl_cfg,
	[SLAVE_SERVICE_CNOC] = &srvc_cnoc,
};

static struct qcom_icc_desc scuba_config_noc = {
	.nodes = config_noc_nodes,
	.num_nodes = ARRAY_SIZE(config_noc_nodes),
};

static struct qcom_icc_node *mmnrt_virt_nodes[] = {
	[MASTER_CAMNOC_SF] = &qnm_camera_nrt,
	[MASTER_VIDEO_P0] = &qxm_venus0,
	[MASTER_VIDEO_PROC] = &qxm_venus_cpu,
	[SLAVE_SNOC_BIMC_NRT] = &slv_snoc_bimc_nrt,
};

static struct qcom_icc_desc scuba_mmnrt_virt = {
	.nodes = mmnrt_virt_nodes,
	.num_nodes = ARRAY_SIZE(mmnrt_virt_nodes),
};

static struct qcom_icc_node *mmrt_virt_nodes[] = {
	[MASTER_CAMNOC_HF] = &qnm_camera_rt,
	[MASTER_MDP_PORT0] = &qxm_mdp0,
	[SLAVE_SNOC_BIMC_RT] = &slv_snoc_bimc_rt,
};

static struct qcom_icc_desc scuba_mmrt_virt = {
	.nodes = mmrt_virt_nodes,
	.num_nodes = ARRAY_SIZE(mmrt_virt_nodes),
};

static struct qcom_icc_node *sys_noc_nodes[] = {
	[MASTER_CRYPTO_CORE0] = &crypto_c0,
	[MASTER_SNOC_CFG] = &qhm_snoc_cfg,
	[MASTER_TIC] = &qhm_tic,
	[MASTER_ANOC_SNOC] = &qnm_anoc_snoc,
	[BIMC_SNOC_MAS] = &qxm_bimc_snoc,
	[MASTER_PIMEM] = &qxm_pimem,
	[MASTER_QDSS_BAM] = &qhm_qdss_bam,
	[MASTER_QPIC] = &qhm_qpic,
	[MASTER_QUP_0] = &qhm_qup0,
	[MASTER_IPA] = &qxm_ipa,
	[MASTER_QDSS_ETR] = &xm_qdss_etr,
	[MASTER_SDCC_1] = &xm_sdc1,
	[MASTER_SDCC_2] = &xm_sdc2,
	[MASTER_USB3] = &xm_usb3_0,
	[SLAVE_APPSS] = &qhs_apss,
	[SNOC_CNOC_SLV] = &qns_snoc_cnoc,
	[SLAVE_OCIMEM] = &qxs_imem,
	[SLAVE_PIMEM] = &qxs_pimem,
	[SNOC_BIMC_SLV] = &qxs_snoc_bimc,
	[SLAVE_SERVICE_SNOC] = &srvc_snoc,
	[SLAVE_QDSS_STM] = &xs_qdss_stm,
	[SLAVE_TCU] = &xs_sys_tcu_cfg,
	[SLAVE_ANOC_SNOC] = &qns_anoc_snoc,
};

static struct qcom_icc_desc scuba_sys_noc = {
	.nodes = sys_noc_nodes,
	.num_nodes = ARRAY_SIZE(sys_noc_nodes),
};

static const struct regmap_config icc_regmap_config = {
	.reg_bits       = 32,
	.reg_stride     = 4,
	.val_bits       = 32,
};

static struct regmap *
qcom_icc_map(struct platform_device *pdev, const struct qcom_icc_desc *desc)
{
	void __iomem *base;
	struct resource *res;
	struct device *dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return NULL;

	base = devm_ioremap(dev, res->start, resource_size(res));
	if (IS_ERR(base))
		return ERR_CAST(base);

	return devm_regmap_init_mmio(dev, base, &icc_regmap_config);
}

static int qnoc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct qcom_icc_desc *desc;
	struct icc_onecell_data *data;
	struct icc_provider *provider;
	struct qcom_icc_node **qnodes;
	struct qcom_icc_provider *qp;
	struct icc_node *node, *tmp;
	size_t num_nodes, i;
	int ret;

	desc = of_device_get_match_data(dev);
	if (!desc)
		return -EINVAL;

	qnodes = desc->nodes;
	num_nodes = desc->num_nodes;

	qp = devm_kzalloc(dev, sizeof(*qp), GFP_KERNEL);
	if (!qp)
		return -ENOMEM;

	data = devm_kzalloc(dev, struct_size(data, nodes, num_nodes),
			    GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	qp->bus_clks = devm_kmemdup(dev, bus_clocks, sizeof(bus_clocks),
				    GFP_KERNEL);
	if (!qp->bus_clks)
		return -ENOMEM;

	qp->num_clks = ARRAY_SIZE(bus_clocks);
	ret = devm_clk_bulk_get(dev, qp->num_clks, qp->bus_clks);
	if (ret)
		return ret;

	qp->num_qos_clks = devm_clk_bulk_get_all(dev, &qp->qos_clks);
	if (qp->num_qos_clks < 0)
		return qp->num_qos_clks;

	ret = clk_bulk_prepare_enable(qp->num_clks, qp->bus_clks);
	if (ret)
		return ret;

	provider = &qp->provider;
	provider->dev = dev;
	provider->set = qcom_icc_rpm_set;
	provider->pre_aggregate = qcom_icc_rpm_pre_aggregate;
	provider->aggregate = qcom_icc_rpm_aggregate;
	provider->xlate = of_icc_xlate_onecell;
	INIT_LIST_HEAD(&provider->nodes);
	provider->data = data;
	qp->dev = &pdev->dev;

	qp->init = true;
	qp->keepalive = of_property_read_bool(dev->of_node, "qcom,keepalive");

	if (of_property_read_u32(dev->of_node, "qcom,util-factor",
				 &qp->util_factor))
		qp->util_factor = DEFAULT_UTIL_FACTOR;

	qp->regmap = qcom_icc_map(pdev, desc);
	if (IS_ERR(qp->regmap))
		return PTR_ERR(qp->regmap);

	ret = icc_provider_add(provider);
	if (ret) {
		dev_err(dev, "error adding interconnect provider: %d\n", ret);
		clk_bulk_disable_unprepare(qp->num_clks, qp->bus_clks);
		return ret;
	}

	ret = clk_bulk_prepare_enable(qp->num_qos_clks, qp->qos_clks);
	if (ret)
		goto disable_clks;

	for (i = 0; i < num_nodes; i++) {
		size_t j;

		if (!qnodes[i])
			continue;

		qnodes[i]->regmap = dev_get_regmap(qp->dev, NULL);

		node = icc_node_create(qnodes[i]->id);
		if (IS_ERR(node)) {
			ret = PTR_ERR(node);
			goto err;
		}

		if (qnodes[i]->qosbox) {
			qnodes[i]->noc_ops->set_qos(qnodes[i]);
			qnodes[i]->qosbox->initialized = true;
		}

		node->name = qnodes[i]->name;
		node->data = qnodes[i];
		icc_node_add(node, provider);

		for (j = 0; j < qnodes[i]->num_links; j++)
			icc_link_create(node, qnodes[i]->links[j]);

		data->nodes[i] = node;
	}
	data->num_nodes = num_nodes;

	clk_bulk_disable_unprepare(qp->num_qos_clks, qp->qos_clks);
	platform_set_drvdata(pdev, qp);

	dev_info(dev, "Registered scuba ICC\n");

	mutex_lock(&probe_list_lock);
	list_add_tail(&qp->probe_list, &qnoc_probe_list);
	mutex_unlock(&probe_list_lock);

	return 0;
err:
	list_for_each_entry_safe(node, tmp, &provider->nodes, node_list) {
		icc_node_del(node);
		icc_node_destroy(node->id);
	}
	clk_bulk_disable_unprepare(qp->num_qos_clks, qp->qos_clks);

disable_clks:
	clk_bulk_disable_unprepare(qp->num_clks, qp->bus_clks);
	icc_provider_del(provider);
	return ret;
}

static int qnoc_remove(struct platform_device *pdev)
{
	struct qcom_icc_provider *qp = platform_get_drvdata(pdev);
	struct icc_provider *provider = &qp->provider;
	struct icc_node *n, *tmp;

	list_for_each_entry_safe(n, tmp, &provider->nodes, node_list) {
		icc_node_del(n);
		icc_node_destroy(n->id);
	}

	clk_bulk_disable_unprepare(qp->num_clks, qp->bus_clks);

	return icc_provider_del(provider);
}

static const struct of_device_id qnoc_of_match[] = {
	{ .compatible = "qcom,scuba-bimc",
	  .data = &scuba_bimc},
	{ .compatible = "qcom,scuba-clk_virt",
	  .data = &scuba_clk_virt},
	{ .compatible = "qcom,scuba-config_noc",
	  .data = &scuba_config_noc},
	{ .compatible = "qcom,scuba-mmnrt_virt",
	  .data = &scuba_mmnrt_virt},
	{ .compatible = "qcom,scuba-mmrt_virt",
	  .data = &scuba_mmrt_virt},
	{ .compatible = "qcom,scuba-sys_noc",
	  .data = &scuba_sys_noc},
	{ }
};
MODULE_DEVICE_TABLE(of, qnoc_of_match);

static void qnoc_sync_state(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct qcom_icc_provider *qp = platform_get_drvdata(pdev);
	int ret = 0, i;

	mutex_lock(&probe_list_lock);
	probe_count++;

	if (probe_count < ARRAY_SIZE(qnoc_of_match) - 1) {
		mutex_unlock(&probe_list_lock);
		return;
	}

	list_for_each_entry(qp, &qnoc_probe_list, probe_list) {
		qp->init = false;

		if (!qp->keepalive)
			continue;

		for (i = 0; i < RPM_NUM_CXT; i++) {
			if (i == RPM_ACTIVE_CXT) {
				if (qp->bus_clk_cur_rate[i] == 0)
					ret = clk_set_rate(qp->bus_clks[i].clk,
						RPM_CLK_MIN_LEVEL);
				else
					ret = clk_set_rate(qp->bus_clks[i].clk,
						qp->bus_clk_cur_rate[i]);

				if (ret)
					pr_err("%s clk_set_rate error: %d\n",
						qp->bus_clks[i].id, ret);
			}
		}
	}

	mutex_unlock(&probe_list_lock);

	pr_err("SCUBA ICC Sync State done\n");
}

static struct platform_driver qnoc_driver = {
	.probe = qnoc_probe,
	.remove = qnoc_remove,
	.driver = {
		.name = "qnoc-scuba",
		.of_match_table = qnoc_of_match,
		.sync_state = qnoc_sync_state,
	},
};

static int __init qnoc_driver_init(void)
{
	return platform_driver_register(&qnoc_driver);
}
core_initcall(qnoc_driver_init);

static void __exit qnoc_driver_exit(void)
{
	platform_driver_unregister(&qnoc_driver);
}
module_exit(qnoc_driver_exit);

MODULE_DESCRIPTION("SCUBA NoC driver");
MODULE_LICENSE("GPL v2");
