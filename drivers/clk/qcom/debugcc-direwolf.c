// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 */

#define pr_fmt(fmt) "clk: %s: " fmt, __func__

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include "clk-debug.h"
#include "common.h"

static struct measure_clk_data debug_mux_priv = {
	.ctl_reg = 0x62038,
	.status_reg = 0x6203C,
	.xo_div4_cbcr = 0x62008,
};

static const char *const apss_cc_debug_mux_parent_names[] = {
	"measure_only_apcs_gold_post_acd_clk",
	"measure_only_apcs_goldplus_post_acd_clk",
	"measure_only_apcs_l3_post_acd_clk",
};

static int apss_cc_debug_mux_sels[] = {
	0x21,		/* measure_only_apcs_gold_post_acd_clk */
	0x25,		/* measure_only_apcs_goldplus_post_acd_clk */
	0x41,		/* measure_only_apcs_l3_post_acd_clk */
};

static int apss_cc_debug_mux_pre_divs[] = {
	0x8,		/* measure_only_apcs_gold_post_acd_clk */
	0x8,		/* measure_only_apcs_goldplus_post_acd_clk */
	0x8,		/* measure_only_apcs_l3_post_acd_clk */
};

static struct clk_debug_mux apss_cc_debug_mux = {
	.priv = &debug_mux_priv,
	.debug_offset = 0x18,
	.post_div_offset = 0x18,
	.cbcr_offset = 0x0,
	.src_sel_mask = 0x7F0,
	.src_sel_shift = 4,
	.post_div_mask = 0x7800,
	.post_div_shift = 11,
	.post_div_val = 1,
	.mux_sels = apss_cc_debug_mux_sels,
	.pre_div_vals = apss_cc_debug_mux_pre_divs,
	.hw.init = &(struct clk_init_data){
		.name = "apss_cc_debug_mux",
		.ops = &clk_debug_mux_ops,
		.parent_names = apss_cc_debug_mux_parent_names,
		.num_parents = ARRAY_SIZE(apss_cc_debug_mux_parent_names),
		.flags = CLK_IS_MEASURE,
	},
};

static const char *const cam_cc_debug_mux_parent_names[] = {
	"cam_cc_bps_ahb_clk",
	"cam_cc_bps_areg_clk",
	"cam_cc_bps_axi_clk",
	"cam_cc_bps_clk",
	"cam_cc_camnoc_axi_clk",
	"cam_cc_camnoc_dcd_xo_clk",
	"cam_cc_cci_0_clk",
	"cam_cc_cci_1_clk",
	"cam_cc_cci_2_clk",
	"cam_cc_cci_3_clk",
	"cam_cc_core_ahb_clk",
	"cam_cc_cpas_ahb_clk",
	"cam_cc_csi0phytimer_clk",
	"cam_cc_csi1phytimer_clk",
	"cam_cc_csi2phytimer_clk",
	"cam_cc_csi3phytimer_clk",
	"cam_cc_csiphy0_clk",
	"cam_cc_csiphy1_clk",
	"cam_cc_csiphy2_clk",
	"cam_cc_csiphy3_clk",
	"cam_cc_gdsc_clk",
	"cam_cc_icp_ahb_clk",
	"cam_cc_icp_clk",
	"cam_cc_ife_0_axi_clk",
	"cam_cc_ife_0_clk",
	"cam_cc_ife_0_cphy_rx_clk",
	"cam_cc_ife_0_csid_clk",
	"cam_cc_ife_0_dsp_clk",
	"cam_cc_ife_1_axi_clk",
	"cam_cc_ife_1_clk",
	"cam_cc_ife_1_cphy_rx_clk",
	"cam_cc_ife_1_csid_clk",
	"cam_cc_ife_1_dsp_clk",
	"cam_cc_ife_2_axi_clk",
	"cam_cc_ife_2_clk",
	"cam_cc_ife_2_cphy_rx_clk",
	"cam_cc_ife_2_csid_clk",
	"cam_cc_ife_2_dsp_clk",
	"cam_cc_ife_3_axi_clk",
	"cam_cc_ife_3_clk",
	"cam_cc_ife_3_cphy_rx_clk",
	"cam_cc_ife_3_csid_clk",
	"cam_cc_ife_3_dsp_clk",
	"cam_cc_ife_lite_0_clk",
	"cam_cc_ife_lite_0_cphy_rx_clk",
	"cam_cc_ife_lite_0_csid_clk",
	"cam_cc_ife_lite_1_clk",
	"cam_cc_ife_lite_1_cphy_rx_clk",
	"cam_cc_ife_lite_1_csid_clk",
	"cam_cc_ife_lite_2_clk",
	"cam_cc_ife_lite_2_cphy_rx_clk",
	"cam_cc_ife_lite_2_csid_clk",
	"cam_cc_ife_lite_3_clk",
	"cam_cc_ife_lite_3_cphy_rx_clk",
	"cam_cc_ife_lite_3_csid_clk",
	"cam_cc_ipe_0_ahb_clk",
	"cam_cc_ipe_0_areg_clk",
	"cam_cc_ipe_0_axi_clk",
	"cam_cc_ipe_0_clk",
	"cam_cc_ipe_1_ahb_clk",
	"cam_cc_ipe_1_areg_clk",
	"cam_cc_ipe_1_axi_clk",
	"cam_cc_ipe_1_clk",
	"cam_cc_jpeg_clk",
	"cam_cc_lrme_clk",
	"cam_cc_mclk0_clk",
	"cam_cc_mclk1_clk",
	"cam_cc_mclk2_clk",
	"cam_cc_mclk3_clk",
	"cam_cc_mclk4_clk",
	"cam_cc_mclk5_clk",
	"cam_cc_mclk6_clk",
	"cam_cc_mclk7_clk",
	"cam_cc_sleep_clk",
};

static int cam_cc_debug_mux_sels[] = {
	0xE,		/* cam_cc_bps_ahb_clk */
	0xD,		/* cam_cc_bps_areg_clk */
	0xC,		/* cam_cc_bps_axi_clk */
	0xB,		/* cam_cc_bps_clk */
	0x27,		/* cam_cc_camnoc_axi_clk */
	0x33,		/* cam_cc_camnoc_dcd_xo_clk */
	0x2A,		/* cam_cc_cci_0_clk */
	0x3B,		/* cam_cc_cci_1_clk */
	0x5B,		/* cam_cc_cci_2_clk */
	0x5C,		/* cam_cc_cci_3_clk */
	0x2E,		/* cam_cc_core_ahb_clk */
	0x2C,		/* cam_cc_cpas_ahb_clk */
	0x5,		/* cam_cc_csi0phytimer_clk */
	0x7,		/* cam_cc_csi1phytimer_clk */
	0x9,		/* cam_cc_csi2phytimer_clk */
	0x35,		/* cam_cc_csi3phytimer_clk */
	0x6,		/* cam_cc_csiphy0_clk */
	0x8,		/* cam_cc_csiphy1_clk */
	0xA,		/* cam_cc_csiphy2_clk */
	0x36,		/* cam_cc_csiphy3_clk */
	0x3C,		/* cam_cc_gdsc_clk */
	0x37,		/* cam_cc_icp_ahb_clk */
	0x26,		/* cam_cc_icp_clk */
	0x1B,		/* cam_cc_ife_0_axi_clk */
	0x17,		/* cam_cc_ife_0_clk */
	0x1A,		/* cam_cc_ife_0_cphy_rx_clk */
	0x19,		/* cam_cc_ife_0_csid_clk */
	0x18,		/* cam_cc_ife_0_dsp_clk */
	0x21,		/* cam_cc_ife_1_axi_clk */
	0x1D,		/* cam_cc_ife_1_clk */
	0x20,		/* cam_cc_ife_1_cphy_rx_clk */
	0x1F,		/* cam_cc_ife_1_csid_clk */
	0x1E,		/* cam_cc_ife_1_dsp_clk */
	0x4A,		/* cam_cc_ife_2_axi_clk */
	0x44,		/* cam_cc_ife_2_clk */
	0x49,		/* cam_cc_ife_2_cphy_rx_clk */
	0x47,		/* cam_cc_ife_2_csid_clk */
	0x46,		/* cam_cc_ife_2_dsp_clk */
	0x51,		/* cam_cc_ife_3_axi_clk */
	0x4B,		/* cam_cc_ife_3_clk */
	0x50,		/* cam_cc_ife_3_cphy_rx_clk */
	0x4E,		/* cam_cc_ife_3_csid_clk */
	0x4D,		/* cam_cc_ife_3_dsp_clk */
	0x22,		/* cam_cc_ife_lite_0_clk */
	0x24,		/* cam_cc_ife_lite_0_cphy_rx_clk */
	0x23,		/* cam_cc_ife_lite_0_csid_clk */
	0x38,		/* cam_cc_ife_lite_1_clk */
	0x3A,		/* cam_cc_ife_lite_1_cphy_rx_clk */
	0x39,		/* cam_cc_ife_lite_1_csid_clk */
	0x55,		/* cam_cc_ife_lite_2_clk */
	0x57,		/* cam_cc_ife_lite_2_cphy_rx_clk */
	0x56,		/* cam_cc_ife_lite_2_csid_clk */
	0x58,		/* cam_cc_ife_lite_3_clk */
	0x5A,		/* cam_cc_ife_lite_3_cphy_rx_clk */
	0x59,		/* cam_cc_ife_lite_3_csid_clk */
	0x12,		/* cam_cc_ipe_0_ahb_clk */
	0x11,		/* cam_cc_ipe_0_areg_clk */
	0x10,		/* cam_cc_ipe_0_axi_clk */
	0xF,		/* cam_cc_ipe_0_clk */
	0x16,		/* cam_cc_ipe_1_ahb_clk */
	0x15,		/* cam_cc_ipe_1_areg_clk */
	0x14,		/* cam_cc_ipe_1_axi_clk */
	0x13,		/* cam_cc_ipe_1_clk */
	0x25,		/* cam_cc_jpeg_clk */
	0x2B,		/* cam_cc_lrme_clk */
	0x1,		/* cam_cc_mclk0_clk */
	0x2,		/* cam_cc_mclk1_clk */
	0x3,		/* cam_cc_mclk2_clk */
	0x4,		/* cam_cc_mclk3_clk */
	0x5D,		/* cam_cc_mclk4_clk */
	0x5E,		/* cam_cc_mclk5_clk */
	0x5F,		/* cam_cc_mclk6_clk */
	0x60,		/* cam_cc_mclk7_clk */
	0x3F,		/* cam_cc_sleep_clk */
};

static struct clk_debug_mux cam_cc_debug_mux = {
	.priv = &debug_mux_priv,
	.debug_offset = 0xD100,
	.post_div_offset = 0xD004,
	.cbcr_offset = 0xD008,
	.src_sel_mask = 0xFF,
	.src_sel_shift = 0,
	.post_div_mask = 0xF,
	.post_div_shift = 0,
	.post_div_val = 4,
	.mux_sels = cam_cc_debug_mux_sels,
	.hw.init = &(struct clk_init_data){
		.name = "cam_cc_debug_mux",
		.ops = &clk_debug_mux_ops,
		.parent_names = cam_cc_debug_mux_parent_names,
		.num_parents = ARRAY_SIZE(cam_cc_debug_mux_parent_names),
		.flags = CLK_IS_MEASURE,
	},
};

static const char *const disp_cc_0_debug_mux_parent_names[] = {
	"disp0_cc_mdss_ahb1_clk",
	"disp0_cc_mdss_ahb_clk",
	"disp0_cc_mdss_byte0_clk",
	"disp0_cc_mdss_byte0_intf_clk",
	"disp0_cc_mdss_byte1_clk",
	"disp0_cc_mdss_byte1_intf_clk",
	"disp0_cc_mdss_dptx0_aux_clk",
	"disp0_cc_mdss_dptx0_link_clk",
	"disp0_cc_mdss_dptx0_link_intf_clk",
	"disp0_cc_mdss_dptx0_pixel0_clk",
	"disp0_cc_mdss_dptx0_pixel1_clk",
	"disp0_cc_mdss_dptx0_usb_router_link_intf_clk",
	"disp0_cc_mdss_dptx1_aux_clk",
	"disp0_cc_mdss_dptx1_link_clk",
	"disp0_cc_mdss_dptx1_link_intf_clk",
	"disp0_cc_mdss_dptx1_pixel0_clk",
	"disp0_cc_mdss_dptx1_pixel1_clk",
	"disp0_cc_mdss_dptx1_usb_router_link_intf_clk",
	"disp0_cc_mdss_dptx2_aux_clk",
	"disp0_cc_mdss_dptx2_link_clk",
	"disp0_cc_mdss_dptx2_link_intf_clk",
	"disp0_cc_mdss_dptx2_pixel0_clk",
	"disp0_cc_mdss_dptx2_pixel1_clk",
	"disp0_cc_mdss_dptx3_aux_clk",
	"disp0_cc_mdss_dptx3_link_clk",
	"disp0_cc_mdss_dptx3_link_intf_clk",
	"disp0_cc_mdss_dptx3_pixel0_clk",
	"disp0_cc_mdss_esc0_clk",
	"disp0_cc_mdss_esc1_clk",
	"disp0_cc_mdss_mdp1_clk",
	"disp0_cc_mdss_mdp_clk",
	"disp0_cc_mdss_mdp_lut1_clk",
	"disp0_cc_mdss_mdp_lut_clk",
	"disp0_cc_mdss_non_gdsc_ahb_clk",
	"disp0_cc_mdss_pclk0_clk",
	"disp0_cc_mdss_pclk1_clk",
	"disp0_cc_mdss_rot1_clk",
	"disp0_cc_mdss_rot_clk",
	"disp0_cc_mdss_rscc_ahb_clk",
	"disp0_cc_mdss_rscc_vsync_clk",
	"disp0_cc_mdss_vsync1_clk",
	"disp0_cc_mdss_vsync_clk",
	"disp0_cc_sleep_clk",
	"disp0_cc_xo_clk",
};

static int disp_cc_0_debug_mux_sels[] = {
	0x39,		/* disp0_cc_mdss_ahb1_clk */
	0x38,		/* disp0_cc_mdss_ahb_clk */
	0x19,		/* disp0_cc_mdss_byte0_clk */
	0x1A,		/* disp0_cc_mdss_byte0_intf_clk */
	0x1B,		/* disp0_cc_mdss_byte1_clk */
	0x1C,		/* disp0_cc_mdss_byte1_intf_clk */
	0x23,		/* disp0_cc_mdss_dptx0_aux_clk */
	0x1F,		/* disp0_cc_mdss_dptx0_link_clk */
	0x20,		/* disp0_cc_mdss_dptx0_link_intf_clk */
	0x24,		/* disp0_cc_mdss_dptx0_pixel0_clk */
	0x25,		/* disp0_cc_mdss_dptx0_pixel1_clk */
	0x21,		/* disp0_cc_mdss_dptx0_usb_router_link_intf_clk */
	0x31,		/* disp0_cc_mdss_dptx1_aux_clk */
	0x2A,		/* disp0_cc_mdss_dptx1_link_clk */
	0x2B,		/* disp0_cc_mdss_dptx1_link_intf_clk */
	0x26,		/* disp0_cc_mdss_dptx1_pixel0_clk */
	0x27,		/* disp0_cc_mdss_dptx1_pixel1_clk */
	0x2C,		/* disp0_cc_mdss_dptx1_usb_router_link_intf_clk */
	0x32,		/* disp0_cc_mdss_dptx2_aux_clk */
	0x2D,		/* disp0_cc_mdss_dptx2_link_clk */
	0x2E,		/* disp0_cc_mdss_dptx2_link_intf_clk */
	0x28,		/* disp0_cc_mdss_dptx2_pixel0_clk */
	0x29,		/* disp0_cc_mdss_dptx2_pixel1_clk */
	0x37,		/* disp0_cc_mdss_dptx3_aux_clk */
	0x34,		/* disp0_cc_mdss_dptx3_link_clk */
	0x35,		/* disp0_cc_mdss_dptx3_link_intf_clk */
	0x33,		/* disp0_cc_mdss_dptx3_pixel0_clk */
	0x1D,		/* disp0_cc_mdss_esc0_clk */
	0x1E,		/* disp0_cc_mdss_esc1_clk */
	0x12,		/* disp0_cc_mdss_mdp1_clk */
	0x11,		/* disp0_cc_mdss_mdp_clk */
	0x16,		/* disp0_cc_mdss_mdp_lut1_clk */
	0x15,		/* disp0_cc_mdss_mdp_lut_clk */
	0x3A,		/* disp0_cc_mdss_non_gdsc_ahb_clk */
	0xF,		/* disp0_cc_mdss_pclk0_clk */
	0x10,		/* disp0_cc_mdss_pclk1_clk */
	0x14,		/* disp0_cc_mdss_rot1_clk */
	0x13,		/* disp0_cc_mdss_rot_clk */
	0x3C,		/* disp0_cc_mdss_rscc_ahb_clk */
	0x3B,		/* disp0_cc_mdss_rscc_vsync_clk */
	0x18,		/* disp0_cc_mdss_vsync1_clk */
	0x17,		/* disp0_cc_mdss_vsync_clk */
	0x46,		/* disp0_cc_sleep_clk */
	0x45,		/* disp0_cc_xo_clk */
};

static struct clk_debug_mux disp_cc_0_debug_mux = {
	.priv = &debug_mux_priv,
	.debug_offset = 0x7000,
	.post_div_offset = 0x5008,
	.cbcr_offset = 0x500C,
	.src_sel_mask = 0xFF,
	.src_sel_shift = 0,
	.post_div_mask = 0xF,
	.post_div_shift = 0,
	.post_div_val = 4,
	.mux_sels = disp_cc_0_debug_mux_sels,
	.hw.init = &(struct clk_init_data){
		.name = "disp_cc_0_debug_mux",
		.ops = &clk_debug_mux_ops,
		.parent_names = disp_cc_0_debug_mux_parent_names,
		.num_parents = ARRAY_SIZE(disp_cc_0_debug_mux_parent_names),
		.flags = CLK_IS_MEASURE,
	},
};

static const char *const disp_cc_1_debug_mux_parent_names[] = {
	"disp1_cc_mdss_ahb1_clk",
	"disp1_cc_mdss_ahb_clk",
	"disp1_cc_mdss_byte0_clk",
	"disp1_cc_mdss_byte0_intf_clk",
	"disp1_cc_mdss_byte1_clk",
	"disp1_cc_mdss_byte1_intf_clk",
	"disp1_cc_mdss_dptx0_aux_clk",
	"disp1_cc_mdss_dptx0_link_clk",
	"disp1_cc_mdss_dptx0_link_intf_clk",
	"disp1_cc_mdss_dptx0_pixel0_clk",
	"disp1_cc_mdss_dptx0_pixel1_clk",
	"disp1_cc_mdss_dptx0_usb_router_link_intf_clk",
	"disp1_cc_mdss_dptx1_aux_clk",
	"disp1_cc_mdss_dptx1_link_clk",
	"disp1_cc_mdss_dptx1_link_intf_clk",
	"disp1_cc_mdss_dptx1_pixel0_clk",
	"disp1_cc_mdss_dptx1_pixel1_clk",
	"disp1_cc_mdss_dptx1_usb_router_link_intf_clk",
	"disp1_cc_mdss_dptx2_aux_clk",
	"disp1_cc_mdss_dptx2_link_clk",
	"disp1_cc_mdss_dptx2_link_intf_clk",
	"disp1_cc_mdss_dptx2_pixel0_clk",
	"disp1_cc_mdss_dptx2_pixel1_clk",
	"disp1_cc_mdss_dptx3_aux_clk",
	"disp1_cc_mdss_dptx3_link_clk",
	"disp1_cc_mdss_dptx3_link_intf_clk",
	"disp1_cc_mdss_dptx3_pixel0_clk",
	"disp1_cc_mdss_esc0_clk",
	"disp1_cc_mdss_esc1_clk",
	"disp1_cc_mdss_mdp1_clk",
	"disp1_cc_mdss_mdp_clk",
	"disp1_cc_mdss_mdp_lut1_clk",
	"disp1_cc_mdss_mdp_lut_clk",
	"disp1_cc_mdss_non_gdsc_ahb_clk",
	"disp1_cc_mdss_pclk0_clk",
	"disp1_cc_mdss_pclk1_clk",
	"disp1_cc_mdss_rot1_clk",
	"disp1_cc_mdss_rot_clk",
	"disp1_cc_mdss_rscc_ahb_clk",
	"disp1_cc_mdss_rscc_vsync_clk",
	"disp1_cc_mdss_vsync1_clk",
	"disp1_cc_mdss_vsync_clk",
	"disp1_cc_sleep_clk",
	"disp1_cc_xo_clk",
};

static int disp_cc_1_debug_mux_sels[] = {
	0x39,		/* disp1_cc_mdss_ahb1_clk */
	0x38,		/* disp1_cc_mdss_ahb_clk */
	0x19,		/* disp1_cc_mdss_byte0_clk */
	0x1A,		/* disp1_cc_mdss_byte0_intf_clk */
	0x1B,		/* disp1_cc_mdss_byte1_clk */
	0x1C,		/* disp1_cc_mdss_byte1_intf_clk */
	0x23,		/* disp1_cc_mdss_dptx0_aux_clk */
	0x1F,		/* disp1_cc_mdss_dptx0_link_clk */
	0x20,		/* disp1_cc_mdss_dptx0_link_intf_clk */
	0x24,		/* disp1_cc_mdss_dptx0_pixel0_clk */
	0x25,		/* disp1_cc_mdss_dptx0_pixel1_clk */
	0x21,		/* disp1_cc_mdss_dptx0_usb_router_link_intf_clk */
	0x31,		/* disp1_cc_mdss_dptx1_aux_clk */
	0x2A,		/* disp1_cc_mdss_dptx1_link_clk */
	0x2B,		/* disp1_cc_mdss_dptx1_link_intf_clk */
	0x26,		/* disp1_cc_mdss_dptx1_pixel0_clk */
	0x27,		/* disp1_cc_mdss_dptx1_pixel1_clk */
	0x2C,		/* disp1_cc_mdss_dptx1_usb_router_link_intf_clk */
	0x32,		/* disp1_cc_mdss_dptx2_aux_clk */
	0x2D,		/* disp1_cc_mdss_dptx2_link_clk */
	0x2E,		/* disp1_cc_mdss_dptx2_link_intf_clk */
	0x28,		/* disp1_cc_mdss_dptx2_pixel0_clk */
	0x29,		/* disp1_cc_mdss_dptx2_pixel1_clk */
	0x37,		/* disp1_cc_mdss_dptx3_aux_clk */
	0x34,		/* disp1_cc_mdss_dptx3_link_clk */
	0x35,		/* disp1_cc_mdss_dptx3_link_intf_clk */
	0x33,		/* disp1_cc_mdss_dptx3_pixel0_clk */
	0x1D,		/* disp1_cc_mdss_esc0_clk */
	0x1E,		/* disp1_cc_mdss_esc1_clk */
	0x12,		/* disp1_cc_mdss_mdp1_clk */
	0x11,		/* disp1_cc_mdss_mdp_clk */
	0x16,		/* disp1_cc_mdss_mdp_lut1_clk */
	0x15,		/* disp1_cc_mdss_mdp_lut_clk */
	0x3A,		/* disp1_cc_mdss_non_gdsc_ahb_clk */
	0xF,		/* disp1_cc_mdss_pclk0_clk */
	0x10,		/* disp1_cc_mdss_pclk1_clk */
	0x14,		/* disp1_cc_mdss_rot1_clk */
	0x13,		/* disp1_cc_mdss_rot_clk */
	0x3C,		/* disp1_cc_mdss_rscc_ahb_clk */
	0x3B,		/* disp1_cc_mdss_rscc_vsync_clk */
	0x18,		/* disp1_cc_mdss_vsync1_clk */
	0x17,		/* disp1_cc_mdss_vsync_clk */
	0x46,		/* disp1_cc_sleep_clk */
	0x45,		/* disp1_cc_xo_clk */
};

static struct clk_debug_mux disp_cc_1_debug_mux = {
	.priv = &debug_mux_priv,
	.debug_offset = 0x7000,
	.post_div_offset = 0x5008,
	.cbcr_offset = 0x500C,
	.src_sel_mask = 0xFF,
	.src_sel_shift = 0,
	.post_div_mask = 0xF,
	.post_div_shift = 0,
	.post_div_val = 4,
	.mux_sels = disp_cc_1_debug_mux_sels,
	.hw.init = &(struct clk_init_data){
		.name = "disp_cc_1_debug_mux",
		.ops = &clk_debug_mux_ops,
		.parent_names = disp_cc_1_debug_mux_parent_names,
		.num_parents = ARRAY_SIZE(disp_cc_1_debug_mux_parent_names),
		.flags = CLK_IS_MEASURE,
	},
};

static const char *const gcc_debug_mux_parent_names[] = {
	"apss_cc_debug_mux",
	"cam_cc_debug_mux",
	"disp_cc_0_debug_mux",
	"disp_cc_1_debug_mux",
	"gcc_aggre_noc_pcie0_tunnel_axi_clk",
	"gcc_aggre_noc_pcie1_tunnel_axi_clk",
	"gcc_aggre_noc_pcie_4_axi_clk",
	"gcc_aggre_noc_pcie_south_sf_axi_clk",
	"gcc_aggre_ufs_card_axi_clk",
	"gcc_aggre_ufs_phy_axi_clk",
	"gcc_aggre_usb3_mp_axi_clk",
	"gcc_aggre_usb3_prim_axi_clk",
	"gcc_aggre_usb3_sec_axi_clk",
	"gcc_aggre_usb4_1_axi_clk",
	"gcc_aggre_usb4_axi_clk",
	"gcc_aggre_usb_noc_axi_clk",
	"gcc_aggre_usb_noc_north_axi_clk",
	"gcc_aggre_usb_noc_south_axi_clk",
	"gcc_ahb2phy0_clk",
	"gcc_ahb2phy2_clk",
	"gcc_camera_hf_axi_clk",
	"gcc_camera_sf_axi_clk",
	"gcc_camera_throttle_nrt_axi_clk",
	"gcc_camera_throttle_rt_axi_clk",
	"gcc_camera_throttle_xo_clk",
	"gcc_cfg_noc_usb3_mp_axi_clk",
	"gcc_cfg_noc_usb3_prim_axi_clk",
	"gcc_cfg_noc_usb3_sec_axi_clk",
	"gcc_cnoc_pcie0_tunnel_clk",
	"gcc_cnoc_pcie1_tunnel_clk",
	"gcc_cnoc_pcie4_qx_clk",
	"gcc_ddrss_gpu_axi_clk",
	"gcc_ddrss_pcie_sf_tbu_clk",
	"gcc_disp1_hf_axi_clk",
	"gcc_disp1_sf_axi_clk",
	"gcc_disp1_throttle_nrt_axi_clk",
	"gcc_disp1_throttle_rt_axi_clk",
	"gcc_disp_hf_axi_clk",
	"gcc_disp_sf_axi_clk",
	"gcc_disp_throttle_nrt_axi_clk",
	"gcc_disp_throttle_rt_axi_clk",
	"gcc_emac0_axi_clk",
	"gcc_emac0_ptp_clk",
	"gcc_emac0_rgmii_clk",
	"gcc_emac0_slv_ahb_clk",
	"gcc_emac1_axi_clk",
	"gcc_emac1_ptp_clk",
	"gcc_emac1_rgmii_clk",
	"gcc_emac1_slv_ahb_clk",
	"gcc_gp1_clk",
	"gcc_gp2_clk",
	"gcc_gp3_clk",
	"gcc_gp4_clk",
	"gcc_gp5_clk",
	"gcc_gpu_gpll0_clk_src",
	"gcc_gpu_gpll0_div_clk_src",
	"gcc_gpu_memnoc_gfx_clk",
	"gcc_gpu_snoc_dvm_gfx_clk",
	"gcc_gpu_tcu_throttle_ahb_clk",
	"gcc_gpu_tcu_throttle_clk",
	"gcc_pcie0_phy_rchng_clk",
	"gcc_pcie1_phy_rchng_clk",
	"gcc_pcie2a_phy_rchng_clk",
	"gcc_pcie2b_phy_rchng_clk",
	"gcc_pcie3a_phy_rchng_clk",
	"gcc_pcie3b_phy_rchng_clk",
	"gcc_pcie4_phy_rchng_clk",
	"gcc_pcie_0_aux_clk",
	"gcc_pcie_0_cfg_ahb_clk",
	"gcc_pcie_0_mstr_axi_clk",
	"gcc_pcie_0_pipe_clk",
	"gcc_pcie_0_slv_axi_clk",
	"gcc_pcie_0_slv_q2a_axi_clk",
	"gcc_pcie_1_aux_clk",
	"gcc_pcie_1_cfg_ahb_clk",
	"gcc_pcie_1_mstr_axi_clk",
	"gcc_pcie_1_pipe_clk",
	"gcc_pcie_1_slv_axi_clk",
	"gcc_pcie_1_slv_q2a_axi_clk",
	"gcc_pcie_2a_aux_clk",
	"gcc_pcie_2a_cfg_ahb_clk",
	"gcc_pcie_2a_mstr_axi_clk",
	"gcc_pcie_2a_pipe_clk",
	"gcc_pcie_2a_pipediv2_clk",
	"gcc_pcie_2a_slv_axi_clk",
	"gcc_pcie_2a_slv_q2a_axi_clk",
	"gcc_pcie_2b_aux_clk",
	"gcc_pcie_2b_cfg_ahb_clk",
	"gcc_pcie_2b_mstr_axi_clk",
	"gcc_pcie_2b_pipe_clk",
	"gcc_pcie_2b_pipediv2_clk",
	"gcc_pcie_2b_slv_axi_clk",
	"gcc_pcie_2b_slv_q2a_axi_clk",
	"gcc_pcie_3a_aux_clk",
	"gcc_pcie_3a_cfg_ahb_clk",
	"gcc_pcie_3a_mstr_axi_clk",
	"gcc_pcie_3a_pipe_clk",
	"gcc_pcie_3a_pipediv2_clk",
	"gcc_pcie_3a_slv_axi_clk",
	"gcc_pcie_3a_slv_q2a_axi_clk",
	"gcc_pcie_3b_aux_clk",
	"gcc_pcie_3b_cfg_ahb_clk",
	"gcc_pcie_3b_mstr_axi_clk",
	"gcc_pcie_3b_pipe_clk",
	"gcc_pcie_3b_pipediv2_clk",
	"gcc_pcie_3b_slv_axi_clk",
	"gcc_pcie_3b_slv_q2a_axi_clk",
	"gcc_pcie_4_aux_clk",
	"gcc_pcie_4_cfg_ahb_clk",
	"gcc_pcie_4_mstr_axi_clk",
	"gcc_pcie_4_pipe_clk",
	"gcc_pcie_4_pipediv2_clk",
	"gcc_pcie_4_slv_axi_clk",
	"gcc_pcie_4_slv_q2a_axi_clk",
	"gcc_pcie_rscc_ahb_clk",
	"gcc_pcie_rscc_xo_clk",
	"gcc_pcie_throttle_cfg_clk",
	"gcc_pdm2_clk",
	"gcc_pdm_ahb_clk",
	"gcc_pdm_xo4_clk",
	"gcc_qmip_camera_nrt_ahb_clk",
	"gcc_qmip_camera_rt_ahb_clk",
	"gcc_qmip_disp1_ahb_clk",
	"gcc_qmip_disp1_rot_ahb_clk",
	"gcc_qmip_disp_ahb_clk",
	"gcc_qmip_disp_rot_ahb_clk",
	"gcc_qmip_video_cvp_ahb_clk",
	"gcc_qmip_video_vcodec_ahb_clk",
	"gcc_qupv3_wrap0_core_2x_clk",
	"gcc_qupv3_wrap0_core_clk",
	"gcc_qupv3_wrap0_qspi0_clk",
	"gcc_qupv3_wrap0_s0_clk",
	"gcc_qupv3_wrap0_s1_clk",
	"gcc_qupv3_wrap0_s2_clk",
	"gcc_qupv3_wrap0_s3_clk",
	"gcc_qupv3_wrap0_s4_clk",
	"gcc_qupv3_wrap0_s5_clk",
	"gcc_qupv3_wrap0_s6_clk",
	"gcc_qupv3_wrap0_s7_clk",
	"gcc_qupv3_wrap1_core_2x_clk",
	"gcc_qupv3_wrap1_core_clk",
	"gcc_qupv3_wrap1_qspi0_clk",
	"gcc_qupv3_wrap1_s0_clk",
	"gcc_qupv3_wrap1_s1_clk",
	"gcc_qupv3_wrap1_s2_clk",
	"gcc_qupv3_wrap1_s3_clk",
	"measure_only_gcc_qupv3_wrap1_s4_clk",
	"gcc_qupv3_wrap1_s5_clk",
	"gcc_qupv3_wrap1_s6_clk",
	"gcc_qupv3_wrap1_s7_clk",
	"gcc_qupv3_wrap2_core_2x_clk",
	"gcc_qupv3_wrap2_core_clk",
	"gcc_qupv3_wrap2_qspi0_clk",
	"gcc_qupv3_wrap2_s0_clk",
	"gcc_qupv3_wrap2_s1_clk",
	"gcc_qupv3_wrap2_s2_clk",
	"gcc_qupv3_wrap2_s3_clk",
	"gcc_qupv3_wrap2_s4_clk",
	"gcc_qupv3_wrap2_s5_clk",
	"gcc_qupv3_wrap2_s6_clk",
	"gcc_qupv3_wrap2_s7_clk",
	"gcc_qupv3_wrap_0_m_ahb_clk",
	"gcc_qupv3_wrap_0_s_ahb_clk",
	"measure_only_gcc_qupv3_wrap_1_m_ahb_clk",
	"measure_only_gcc_qupv3_wrap_1_s_ahb_clk",
	"gcc_qupv3_wrap_2_m_ahb_clk",
	"gcc_qupv3_wrap_2_s_ahb_clk",
	"gcc_sdcc2_ahb_clk",
	"gcc_sdcc2_apps_clk",
	"gcc_sdcc4_ahb_clk",
	"gcc_sdcc4_apps_clk",
	"gcc_sys_noc_usb_axi_clk",
	"gcc_ufs_card_ahb_clk",
	"gcc_ufs_card_axi_clk",
	"gcc_ufs_card_ice_core_clk",
	"gcc_ufs_card_phy_aux_clk",
	"gcc_ufs_card_rx_symbol_0_clk",
	"gcc_ufs_card_rx_symbol_1_clk",
	"gcc_ufs_card_tx_symbol_0_clk",
	"gcc_ufs_card_unipro_core_clk",
	"gcc_ufs_phy_ahb_clk",
	"gcc_ufs_phy_axi_clk",
	"gcc_ufs_phy_ice_core_clk",
	"gcc_ufs_phy_phy_aux_clk",
	"gcc_ufs_phy_rx_symbol_0_clk",
	"gcc_ufs_phy_rx_symbol_1_clk",
	"gcc_ufs_phy_tx_symbol_0_clk",
	"gcc_ufs_phy_unipro_core_clk",
	"gcc_usb30_mp_master_clk",
	"gcc_usb30_mp_mock_utmi_clk",
	"gcc_usb30_prim_master_clk",
	"gcc_usb30_prim_mock_utmi_clk",
	"gcc_usb30_sec_master_clk",
	"gcc_usb30_sec_mock_utmi_clk",
	"gcc_usb3_mp_phy_aux_clk",
	"gcc_usb3_mp_phy_com_aux_clk",
	"gcc_usb3_mp_phy_pipe_0_clk",
	"gcc_usb3_mp_phy_pipe_1_clk",
	"gcc_usb3_prim_phy_aux_clk",
	"gcc_usb3_prim_phy_com_aux_clk",
	"gcc_usb3_prim_phy_pipe_clk",
	"gcc_usb3_sec_phy_aux_clk",
	"gcc_usb3_sec_phy_com_aux_clk",
	"gcc_usb3_sec_phy_pipe_clk",
	"gcc_usb4_1_cfg_ahb_clk",
	"gcc_usb4_1_dp_clk",
	"gcc_usb4_1_master_clk",
	"gcc_usb4_1_phy_p2rr2p_pipe_clk",
	"gcc_usb4_1_phy_pcie_pipe_clk",
	"gcc_usb4_1_phy_rx0_clk",
	"gcc_usb4_1_phy_rx1_clk",
	"gcc_usb4_1_phy_usb_pipe_clk",
	"gcc_usb4_1_sb_if_clk",
	"gcc_usb4_1_sys_clk",
	"gcc_usb4_1_tmu_clk",
	"gcc_usb4_cfg_ahb_clk",
	"gcc_usb4_dp_clk",
	"gcc_usb4_master_clk",
	"gcc_usb4_phy_p2rr2p_pipe_clk",
	"gcc_usb4_phy_pcie_pipe_clk",
	"gcc_usb4_phy_rx0_clk",
	"gcc_usb4_phy_rx1_clk",
	"gcc_usb4_phy_usb_pipe_clk",
	"gcc_usb4_sb_if_clk",
	"gcc_usb4_sys_clk",
	"gcc_usb4_tmu_clk",
	"gcc_video_axi0_clk",
	"gcc_video_axi1_clk",
	"gcc_video_cvp_throttle_clk",
	"gcc_video_vcodec_throttle_clk",
	"gpu_cc_debug_mux",
	"measure_only_cnoc_clk",
	"measure_only_gcc_camera_ahb_clk",
	"measure_only_gcc_camera_xo_clk",
	"measure_only_gcc_disp1_ahb_clk",
	"measure_only_gcc_disp1_xo_clk",
	"measure_only_gcc_disp_ahb_clk",
	"measure_only_gcc_disp_xo_clk",
	"measure_only_gcc_gpu_cfg_ahb_clk",
	"measure_only_gcc_video_ahb_clk",
	"measure_only_gcc_video_xo_clk",
	"measure_only_ipa_2x_clk",
	"measure_only_memnoc_clk",
	"measure_only_snoc_clk",
	"mc_cc_debug_mux",
	"video_cc_debug_mux",
};

static int gcc_debug_mux_sels[] = {
	0x17D,		/* apss_cc_debug_mux */
	0x70,		/* cam_cc_debug_mux */
	0x79,		/* disp_cc_0_debug_mux */
	0x82,		/* disp_cc_1_debug_mux */
	0x217,		/* gcc_aggre_noc_pcie0_tunnel_axi_clk */
	0x218,		/* gcc_aggre_noc_pcie1_tunnel_axi_clk */
	0x214,		/* gcc_aggre_noc_pcie_4_axi_clk */
	0x215,		/* gcc_aggre_noc_pcie_south_sf_axi_clk */
	0x222,		/* gcc_aggre_ufs_card_axi_clk */
	0x221,		/* gcc_aggre_ufs_phy_axi_clk */
	0x21B,		/* gcc_aggre_usb3_mp_axi_clk */
	0x219,		/* gcc_aggre_usb3_prim_axi_clk */
	0x21A,		/* gcc_aggre_usb3_sec_axi_clk */
	0x21D,		/* gcc_aggre_usb4_1_axi_clk */
	0x21C,		/* gcc_aggre_usb4_axi_clk */
	0x220,		/* gcc_aggre_usb_noc_axi_clk */
	0x21F,		/* gcc_aggre_usb_noc_north_axi_clk */
	0x21E,		/* gcc_aggre_usb_noc_south_axi_clk */
	0xFE,		/* gcc_ahb2phy0_clk */
	0xFF,		/* gcc_ahb2phy2_clk */
	0x6A,		/* gcc_camera_hf_axi_clk */
	0x6B,		/* gcc_camera_sf_axi_clk */
	0x6D,		/* gcc_camera_throttle_nrt_axi_clk */
	0x6C,		/* gcc_camera_throttle_rt_axi_clk */
	0x6F,		/* gcc_camera_throttle_xo_clk */
	0x2C,		/* gcc_cfg_noc_usb3_mp_axi_clk */
	0x2A,		/* gcc_cfg_noc_usb3_prim_axi_clk */
	0x2B,		/* gcc_cfg_noc_usb3_sec_axi_clk */
	0x1C,		/* gcc_cnoc_pcie0_tunnel_clk */
	0x1D,		/* gcc_cnoc_pcie1_tunnel_clk */
	0x18,		/* gcc_cnoc_pcie4_qx_clk */
	0x155,		/* gcc_ddrss_gpu_axi_clk */
	0x156,		/* gcc_ddrss_pcie_sf_tbu_clk */
	0x7D,		/* gcc_disp1_hf_axi_clk */
	0x7E,		/* gcc_disp1_sf_axi_clk */
	0x80,		/* gcc_disp1_throttle_nrt_axi_clk */
	0x7F,		/* gcc_disp1_throttle_rt_axi_clk */
	0x74,		/* gcc_disp_hf_axi_clk */
	0x75,		/* gcc_disp_sf_axi_clk */
	0x77,		/* gcc_disp_throttle_nrt_axi_clk */
	0x76,		/* gcc_disp_throttle_rt_axi_clk */
	0x246,		/* gcc_emac0_axi_clk */
	0x248,		/* gcc_emac0_ptp_clk */
	0x249,		/* gcc_emac0_rgmii_clk */
	0x247,		/* gcc_emac0_slv_ahb_clk */
	0x24A,		/* gcc_emac1_axi_clk */
	0x24C,		/* gcc_emac1_ptp_clk */
	0x24D,		/* gcc_emac1_rgmii_clk */
	0x24B,		/* gcc_emac1_slv_ahb_clk */
	0x18C,		/* gcc_gp1_clk */
	0x18D,		/* gcc_gp2_clk */
	0x18E,		/* gcc_gp3_clk */
	0x290,		/* gcc_gp4_clk */
	0x291,		/* gcc_gp5_clk */
	0x232,		/* gcc_gpu_gpll0_clk_src */
	0x233,		/* gcc_gpu_gpll0_div_clk_src */
	0x22E,		/* gcc_gpu_memnoc_gfx_clk */
	0x231,		/* gcc_gpu_snoc_dvm_gfx_clk */
	0x22B,		/* gcc_gpu_tcu_throttle_ahb_clk */
	0x22F,		/* gcc_gpu_tcu_throttle_clk */
	0x1D0,		/* gcc_pcie0_phy_rchng_clk */
	0x19F,		/* gcc_pcie1_phy_rchng_clk */
	0x1A9,		/* gcc_pcie2a_phy_rchng_clk */
	0x1B3,		/* gcc_pcie2b_phy_rchng_clk */
	0x1BD,		/* gcc_pcie3a_phy_rchng_clk */
	0x1C7,		/* gcc_pcie3b_phy_rchng_clk */
	0x196,		/* gcc_pcie4_phy_rchng_clk */
	0x1CE,		/* gcc_pcie_0_aux_clk */
	0x1CD,		/* gcc_pcie_0_cfg_ahb_clk */
	0x1CC,		/* gcc_pcie_0_mstr_axi_clk */
	0x1CF,		/* gcc_pcie_0_pipe_clk */
	0x1CB,		/* gcc_pcie_0_slv_axi_clk */
	0x1CA,		/* gcc_pcie_0_slv_q2a_axi_clk */
	0x19D,		/* gcc_pcie_1_aux_clk */
	0x19C,		/* gcc_pcie_1_cfg_ahb_clk */
	0x19B,		/* gcc_pcie_1_mstr_axi_clk */
	0x19E,		/* gcc_pcie_1_pipe_clk */
	0x19A,		/* gcc_pcie_1_slv_axi_clk */
	0x199,		/* gcc_pcie_1_slv_q2a_axi_clk */
	0x1A6,		/* gcc_pcie_2a_aux_clk */
	0x1A5,		/* gcc_pcie_2a_cfg_ahb_clk */
	0x1A4,		/* gcc_pcie_2a_mstr_axi_clk */
	0x1A7,		/* gcc_pcie_2a_pipe_clk */
	0x1A8,		/* gcc_pcie_2a_pipediv2_clk */
	0x1A3,		/* gcc_pcie_2a_slv_axi_clk */
	0x1A2,		/* gcc_pcie_2a_slv_q2a_axi_clk */
	0x1B0,		/* gcc_pcie_2b_aux_clk */
	0x1AF,		/* gcc_pcie_2b_cfg_ahb_clk */
	0x1AE,		/* gcc_pcie_2b_mstr_axi_clk */
	0x1B1,		/* gcc_pcie_2b_pipe_clk */
	0x1B2,		/* gcc_pcie_2b_pipediv2_clk */
	0x1AD,		/* gcc_pcie_2b_slv_axi_clk */
	0x1AC,		/* gcc_pcie_2b_slv_q2a_axi_clk */
	0x1BA,		/* gcc_pcie_3a_aux_clk */
	0x1B9,		/* gcc_pcie_3a_cfg_ahb_clk */
	0x1B8,		/* gcc_pcie_3a_mstr_axi_clk */
	0x1BB,		/* gcc_pcie_3a_pipe_clk */
	0x1BC,		/* gcc_pcie_3a_pipediv2_clk */
	0x1B7,		/* gcc_pcie_3a_slv_axi_clk */
	0x1B6,		/* gcc_pcie_3a_slv_q2a_axi_clk */
	0x1C4,		/* gcc_pcie_3b_aux_clk */
	0x1C3,		/* gcc_pcie_3b_cfg_ahb_clk */
	0x1C2,		/* gcc_pcie_3b_mstr_axi_clk */
	0x1C5,		/* gcc_pcie_3b_pipe_clk */
	0x1C6,		/* gcc_pcie_3b_pipediv2_clk */
	0x1C1,		/* gcc_pcie_3b_slv_axi_clk */
	0x1C0,		/* gcc_pcie_3b_slv_q2a_axi_clk */
	0x193,		/* gcc_pcie_4_aux_clk */
	0x192,		/* gcc_pcie_4_cfg_ahb_clk */
	0x191,		/* gcc_pcie_4_mstr_axi_clk */
	0x194,		/* gcc_pcie_4_pipe_clk */
	0x195,		/* gcc_pcie_4_pipediv2_clk */
	0x190,		/* gcc_pcie_4_slv_axi_clk */
	0x18F,		/* gcc_pcie_4_slv_q2a_axi_clk */
	0x8F,		/* gcc_pcie_rscc_ahb_clk */
	0x8E,		/* gcc_pcie_rscc_xo_clk */
	0x46,		/* gcc_pcie_throttle_cfg_clk */
	0x122,		/* gcc_pdm2_clk */
	0x120,		/* gcc_pdm_ahb_clk */
	0x121,		/* gcc_pdm_xo4_clk */
	0x68,		/* gcc_qmip_camera_nrt_ahb_clk */
	0x69,		/* gcc_qmip_camera_rt_ahb_clk */
	0x7B,		/* gcc_qmip_disp1_ahb_clk */
	0x7C,		/* gcc_qmip_disp1_rot_ahb_clk */
	0x72,		/* gcc_qmip_disp_ahb_clk */
	0x73,		/* gcc_qmip_disp_rot_ahb_clk */
	0x86,		/* gcc_qmip_video_cvp_ahb_clk */
	0x87,		/* gcc_qmip_video_vcodec_ahb_clk */
	0x109,		/* gcc_qupv3_wrap0_core_2x_clk */
	0x108,		/* gcc_qupv3_wrap0_core_clk */
	0x112,		/* gcc_qupv3_wrap0_qspi0_clk */
	0x10A,		/* gcc_qupv3_wrap0_s0_clk */
	0x10B,		/* gcc_qupv3_wrap0_s1_clk */
	0x10C,		/* gcc_qupv3_wrap0_s2_clk */
	0x10D,		/* gcc_qupv3_wrap0_s3_clk */
	0x10E,		/* gcc_qupv3_wrap0_s4_clk */
	0x10F,		/* gcc_qupv3_wrap0_s5_clk */
	0x110,		/* gcc_qupv3_wrap0_s6_clk */
	0x111,		/* gcc_qupv3_wrap0_s7_clk */
	0x116,		/* gcc_qupv3_wrap1_core_2x_clk */
	0x115,		/* gcc_qupv3_wrap1_core_clk */
	0x11F,		/* gcc_qupv3_wrap1_qspi0_clk */
	0x117,		/* gcc_qupv3_wrap1_s0_clk */
	0x118,		/* gcc_qupv3_wrap1_s1_clk */
	0x119,		/* gcc_qupv3_wrap1_s2_clk */
	0x11A,		/* gcc_qupv3_wrap1_s3_clk */
	0x11B,		/* measure_only_gcc_qupv3_wrap1_s4_clk */
	0x11C,		/* gcc_qupv3_wrap1_s5_clk */
	0x11D,		/* gcc_qupv3_wrap1_s6_clk */
	0x11E,		/* gcc_qupv3_wrap1_s7_clk */
	0x251,		/* gcc_qupv3_wrap2_core_2x_clk */
	0x250,		/* gcc_qupv3_wrap2_core_clk */
	0x25A,		/* gcc_qupv3_wrap2_qspi0_clk */
	0x252,		/* gcc_qupv3_wrap2_s0_clk */
	0x253,		/* gcc_qupv3_wrap2_s1_clk */
	0x254,		/* gcc_qupv3_wrap2_s2_clk */
	0x255,		/* gcc_qupv3_wrap2_s3_clk */
	0x256,		/* gcc_qupv3_wrap2_s4_clk */
	0x257,		/* gcc_qupv3_wrap2_s5_clk */
	0x258,		/* gcc_qupv3_wrap2_s6_clk */
	0x259,		/* gcc_qupv3_wrap2_s7_clk */
	0x106,		/* gcc_qupv3_wrap_0_m_ahb_clk */
	0x107,		/* gcc_qupv3_wrap_0_s_ahb_clk */
	0x113,		/* measure_only_gcc_qupv3_wrap_1_m_ahb_clk */
	0x114,		/* measure_only_gcc_qupv3_wrap_1_s_ahb_clk */
	0x24E,		/* gcc_qupv3_wrap_2_m_ahb_clk */
	0x24F,		/* gcc_qupv3_wrap_2_s_ahb_clk */
	0x101,		/* gcc_sdcc2_ahb_clk */
	0x100,		/* gcc_sdcc2_apps_clk */
	0x104,		/* gcc_sdcc4_ahb_clk */
	0x103,		/* gcc_sdcc4_apps_clk */
	0x14,		/* gcc_sys_noc_usb_axi_clk */
	0x1D4,		/* gcc_ufs_card_ahb_clk */
	0x1D3,		/* gcc_ufs_card_axi_clk */
	0x1DA,		/* gcc_ufs_card_ice_core_clk */
	0x1DB,		/* gcc_ufs_card_phy_aux_clk */
	0x1D6,		/* gcc_ufs_card_rx_symbol_0_clk */
	0x1DC,		/* gcc_ufs_card_rx_symbol_1_clk */
	0x1D5,		/* gcc_ufs_card_tx_symbol_0_clk */
	0x1D9,		/* gcc_ufs_card_unipro_core_clk */
	0x1E0,		/* gcc_ufs_phy_ahb_clk */
	0x1DF,		/* gcc_ufs_phy_axi_clk */
	0x1E6,		/* gcc_ufs_phy_ice_core_clk */
	0x1E7,		/* gcc_ufs_phy_phy_aux_clk */
	0x1E2,		/* gcc_ufs_phy_rx_symbol_0_clk */
	0x1E8,		/* gcc_ufs_phy_rx_symbol_1_clk */
	0x1E1,		/* gcc_ufs_phy_tx_symbol_0_clk */
	0x1E5,		/* gcc_ufs_phy_unipro_core_clk */
	0xC5,		/* gcc_usb30_mp_master_clk */
	0xC7,		/* gcc_usb30_mp_mock_utmi_clk */
	0xB6,		/* gcc_usb30_prim_master_clk */
	0xB8,		/* gcc_usb30_prim_mock_utmi_clk */
	0xBF,		/* gcc_usb30_sec_master_clk */
	0xC1,		/* gcc_usb30_sec_mock_utmi_clk */
	0xC8,		/* gcc_usb3_mp_phy_aux_clk */
	0xC9,		/* gcc_usb3_mp_phy_com_aux_clk */
	0xCA,		/* gcc_usb3_mp_phy_pipe_0_clk */
	0xCB,		/* gcc_usb3_mp_phy_pipe_1_clk */
	0xB9,		/* gcc_usb3_prim_phy_aux_clk */
	0xBA,		/* gcc_usb3_prim_phy_com_aux_clk */
	0xBB,		/* gcc_usb3_prim_phy_pipe_clk */
	0xC2,		/* gcc_usb3_sec_phy_aux_clk */
	0xC3,		/* gcc_usb3_sec_phy_com_aux_clk */
	0xC4,		/* gcc_usb3_sec_phy_pipe_clk */
	0xF3,		/* gcc_usb4_1_cfg_ahb_clk */
	0xF0,		/* gcc_usb4_1_dp_clk */
	0xEC,		/* gcc_usb4_1_master_clk */
	0xF8,		/* gcc_usb4_1_phy_p2rr2p_pipe_clk */
	0xEE,		/* gcc_usb4_1_phy_pcie_pipe_clk */
	0xF4,		/* gcc_usb4_1_phy_rx0_clk */
	0xF5,		/* gcc_usb4_1_phy_rx1_clk */
	0xF2,		/* gcc_usb4_1_phy_usb_pipe_clk */
	0xED,		/* gcc_usb4_1_sb_if_clk */
	0xEF,		/* gcc_usb4_1_sys_clk */
	0xF1,		/* gcc_usb4_1_tmu_clk */
	0xE1,		/* gcc_usb4_cfg_ahb_clk */
	0xDE,		/* gcc_usb4_dp_clk */
	0xDA,		/* gcc_usb4_master_clk */
	0xE6,		/* gcc_usb4_phy_p2rr2p_pipe_clk */
	0xDC,		/* gcc_usb4_phy_pcie_pipe_clk */
	0xE2,		/* gcc_usb4_phy_rx0_clk */
	0xE3,		/* gcc_usb4_phy_rx1_clk */
	0xE0,		/* gcc_usb4_phy_usb_pipe_clk */
	0xDB,		/* gcc_usb4_sb_if_clk */
	0xDD,		/* gcc_usb4_sys_clk */
	0xDF,		/* gcc_usb4_tmu_clk */
	0x88,		/* gcc_video_axi0_clk */
	0x89,		/* gcc_video_axi1_clk */
	0x8B,		/* gcc_video_cvp_throttle_clk */
	0x8A,		/* gcc_video_vcodec_throttle_clk */
	0x22D,		/* gpu_cc_debug_mux */
	0x22,		/* measure_only_cnoc_clk */
	0x67,		/* measure_only_gcc_camera_ahb_clk */
	0x6E,		/* measure_only_gcc_camera_xo_clk */
	0x7A,		/* measure_only_gcc_disp1_ahb_clk */
	0x81,		/* measure_only_gcc_disp1_xo_clk */
	0x71,		/* measure_only_gcc_disp_ahb_clk */
	0x78,		/* measure_only_gcc_disp_xo_clk */
	0x22A,		/* measure_only_gcc_gpu_cfg_ahb_clk */
	0x85,		/* measure_only_gcc_video_ahb_clk */
	0x8C,		/* measure_only_gcc_video_xo_clk */
	0x225,		/* measure_only_ipa_2x_clk */
	0x15B,		/* measure_only_memnoc_clk */
	0xC,		/* measure_only_snoc_clk */
	0x15F,		/* mc_cc_debug_mux or ddrss_gcc_debug_clk */
	0x8D,		/* video_cc_debug_mux */
};

static struct clk_debug_mux gcc_debug_mux = {
	.priv = &debug_mux_priv,
	.debug_offset = 0x62024,
	.post_div_offset = 0x62000,
	.cbcr_offset = 0x62004,
	.src_sel_mask = 0x1FFF,
	.src_sel_shift = 0,
	.post_div_mask = 0xF,
	.post_div_shift = 0,
	.post_div_val = 2,
	.mux_sels = gcc_debug_mux_sels,
	.hw.init = &(struct clk_init_data){
		.name = "gcc_debug_mux",
		.ops = &clk_debug_mux_ops,
		.parent_names = gcc_debug_mux_parent_names,
		.num_parents = ARRAY_SIZE(gcc_debug_mux_parent_names),
		.flags = CLK_IS_MEASURE,
	},
};

static const char *const gpu_cc_debug_mux_parent_names[] = {
	"gpu_cc_ahb_clk",
	"gpu_cc_crc_ahb_clk",
	"gpu_cc_cx_gmu_clk",
	"gpu_cc_cx_snoc_dvm_clk",
	"gpu_cc_cxo_aon_clk",
	"gpu_cc_gx_gmu_clk",
	"gpu_cc_hub_aon_clk",
	"gpu_cc_hub_cx_int_clk",
	"gpu_cc_sleep_clk",
	"measure_only_gcc_gpu_cfg_ahb_clk",
	"measure_only_gpu_cc_cx_gfx3d_clk",
	"measure_only_gpu_cc_cx_gfx3d_slv_clk",
	"measure_only_gpu_cc_gx_gfx3d_clk",
};

static int gpu_cc_debug_mux_sels[] = {
	0x12,		/* gpu_cc_ahb_clk */
	0x13,		/* gpu_cc_crc_ahb_clk */
	0x1A,		/* gpu_cc_cx_gmu_clk */
	0x17,		/* gpu_cc_cx_snoc_dvm_clk */
	0xB,		/* gpu_cc_cxo_aon_clk */
	0x11,		/* gpu_cc_gx_gmu_clk */
	0x27,		/* gpu_cc_hub_aon_clk */
	0x1C,		/* gpu_cc_hub_cx_int_clk */
	0x18,		/* gpu_cc_sleep_clk */
	0x1,		/* measure_only_gcc_gpu_cfg_ahb_clk */
	0x1D,		/* measure_only_gpu_cc_cx_gfx3d_clk */
	0x1E,		/* measure_only_gpu_cc_cx_gfx3d_slv_clk */
	0xD,		/* measure_only_gpu_cc_gx_gfx3d_clk */
};

static struct clk_debug_mux gpu_cc_debug_mux = {
	.priv = &debug_mux_priv,
	.debug_offset = 0x1568,
	.post_div_offset = 0x10FC,
	.cbcr_offset = 0x1100,
	.src_sel_mask = 0xFF,
	.src_sel_shift = 0,
	.post_div_mask = 0xF,
	.post_div_shift = 0,
	.post_div_val = 2,
	.mux_sels = gpu_cc_debug_mux_sels,
	.hw.init = &(struct clk_init_data){
		.name = "gpu_cc_debug_mux",
		.ops = &clk_debug_mux_ops,
		.parent_names = gpu_cc_debug_mux_parent_names,
		.num_parents = ARRAY_SIZE(gpu_cc_debug_mux_parent_names),
		.flags = CLK_IS_MEASURE,
	},
};

static const char *const mc_cc_debug_mux_parent_names[] = {
	"measure_only_mccc_clk",
};

static struct clk_debug_mux mc_cc_debug_mux = {
	.period_offset = 0x50,
	.hw.init = &(struct clk_init_data){
		.name = "mc_cc_debug_mux",
		.ops = &clk_debug_mux_ops,
		.parent_names = mc_cc_debug_mux_parent_names,
		.num_parents = ARRAY_SIZE(mc_cc_debug_mux_parent_names),
		.flags = CLK_IS_MEASURE,
	},
};

static const char *const video_cc_debug_mux_parent_names[] = {
	"video_cc_ahb_clk",
	"video_cc_mvs0_clk",
	"video_cc_mvs0c_clk",
	"video_cc_mvs1_clk",
	"video_cc_mvs1_div2_clk",
	"video_cc_mvs1c_clk",
	"video_cc_sleep_clk",
	"video_cc_xo_clk",
};

static int video_cc_debug_mux_sels[] = {
	0x7,		/* video_cc_ahb_clk */
	0x3,		/* video_cc_mvs0_clk */
	0x1,		/* video_cc_mvs0c_clk */
	0x5,		/* video_cc_mvs1_clk */
	0x8,		/* video_cc_mvs1_div2_clk */
	0x9,		/* video_cc_mvs1c_clk */
	0xC,		/* video_cc_sleep_clk */
	0xB,		/* video_cc_xo_clk */
};

static struct clk_debug_mux video_cc_debug_mux = {
	.priv = &debug_mux_priv,
	.debug_offset = 0xA4C,
	.post_div_offset = 0xE9C,
	.cbcr_offset = 0xEBC,
	.src_sel_mask = 0x3F,
	.src_sel_shift = 0,
	.post_div_mask = 0xF,
	.post_div_shift = 0,
	.post_div_val = 3,
	.mux_sels = video_cc_debug_mux_sels,
	.hw.init = &(struct clk_init_data){
		.name = "video_cc_debug_mux",
		.ops = &clk_debug_mux_ops,
		.parent_names = video_cc_debug_mux_parent_names,
		.num_parents = ARRAY_SIZE(video_cc_debug_mux_parent_names),
		.flags = CLK_IS_MEASURE,
	},
};

static struct mux_regmap_names mux_list[] = {
	{ .mux = &apss_cc_debug_mux, .regmap_name = "qcom,apsscc" },
	{ .mux = &cam_cc_debug_mux, .regmap_name = "qcom,camcc" },
	{ .mux = &disp_cc_0_debug_mux, .regmap_name = "qcom,dispcc0" },
	{ .mux = &disp_cc_1_debug_mux, .regmap_name = "qcom,dispcc1" },
	{ .mux = &gcc_debug_mux, .regmap_name = "qcom,gcc" },
	{ .mux = &gpu_cc_debug_mux, .regmap_name = "qcom,gpucc" },
	{ .mux = &mc_cc_debug_mux, .regmap_name = "qcom,mccc" },
	{ .mux = &video_cc_debug_mux, .regmap_name = "qcom,videocc" },
};

static struct clk_dummy measure_only_apcs_gold_post_acd_clk = {
	.rrate = 1000,
	.hw.init = &(struct clk_init_data){
		.name = "measure_only_apcs_gold_post_acd_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_apcs_goldplus_post_acd_clk = {
	.rrate = 1000,
	.hw.init = &(struct clk_init_data){
		.name = "measure_only_apcs_goldplus_post_acd_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_apcs_l3_post_acd_clk = {
	.rrate = 1000,
	.hw.init = &(struct clk_init_data){
		.name = "measure_only_apcs_l3_post_acd_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_cnoc_clk = {
	.rrate = 1000,
	.hw.init = &(struct clk_init_data){
		.name = "measure_only_cnoc_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_gcc_camera_ahb_clk = {
	.rrate = 1000,
	.hw.init = &(struct clk_init_data){
		.name = "measure_only_gcc_camera_ahb_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_gcc_camera_xo_clk = {
	.rrate = 1000,
	.hw.init = &(struct clk_init_data){
		.name = "measure_only_gcc_camera_xo_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_gcc_disp1_ahb_clk = {
	.rrate = 1000,
	.hw.init = &(struct clk_init_data){
		.name = "measure_only_gcc_disp1_ahb_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_gcc_disp1_xo_clk = {
	.rrate = 1000,
	.hw.init = &(struct clk_init_data){
		.name = "measure_only_gcc_disp1_xo_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_gcc_disp_ahb_clk = {
	.rrate = 1000,
	.hw.init = &(struct clk_init_data){
		.name = "measure_only_gcc_disp_ahb_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_gcc_disp_xo_clk = {
	.rrate = 1000,
	.hw.init = &(struct clk_init_data){
		.name = "measure_only_gcc_disp_xo_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_gcc_gpu_cfg_ahb_clk = {
	.rrate = 1000,
	.hw.init = &(struct clk_init_data){
		.name = "measure_only_gcc_gpu_cfg_ahb_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_gcc_qupv3_wrap1_s4_clk = {
	.rrate = 1000,
	.hw.init = &(struct clk_init_data){
		.name = "measure_only_gcc_qupv3_wrap1_s4_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_gcc_qupv3_wrap_1_m_ahb_clk = {
	.rrate = 1000,
	.hw.init = &(struct clk_init_data){
		.name = "measure_only_gcc_qupv3_wrap_1_m_ahb_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_gcc_qupv3_wrap_1_s_ahb_clk = {
	.rrate = 1000,
	.hw.init = &(struct clk_init_data){
		.name = "measure_only_gcc_qupv3_wrap_1_s_ahb_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_gcc_video_ahb_clk = {
	.rrate = 1000,
	.hw.init = &(struct clk_init_data){
		.name = "measure_only_gcc_video_ahb_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_gcc_video_xo_clk = {
	.rrate = 1000,
	.hw.init = &(struct clk_init_data){
		.name = "measure_only_gcc_video_xo_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_gpu_cc_cx_gfx3d_clk = {
	.rrate = 1000,
	.hw.init = &(struct clk_init_data){
		.name = "measure_only_gpu_cc_cx_gfx3d_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_gpu_cc_cx_gfx3d_slv_clk = {
	.rrate = 1000,
	.hw.init = &(struct clk_init_data){
		.name = "measure_only_gpu_cc_cx_gfx3d_slv_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_gpu_cc_gx_gfx3d_clk = {
	.rrate = 1000,
	.hw.init = &(struct clk_init_data){
		.name = "measure_only_gpu_cc_gx_gfx3d_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_ipa_2x_clk = {
	.rrate = 1000,
	.hw.init = &(struct clk_init_data){
		.name = "measure_only_ipa_2x_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_mccc_clk = {
	.rrate = 1000,
	.hw.init = &(struct clk_init_data){
		.name = "measure_only_mccc_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_memnoc_clk = {
	.rrate = 1000,
	.hw.init = &(struct clk_init_data){
		.name = "measure_only_memnoc_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_snoc_clk = {
	.rrate = 1000,
	.hw.init = &(struct clk_init_data){
		.name = "measure_only_snoc_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_hw *debugcc_direwolf_hws[] = {
	&measure_only_apcs_gold_post_acd_clk.hw,
	&measure_only_apcs_goldplus_post_acd_clk.hw,
	&measure_only_apcs_l3_post_acd_clk.hw,
	&measure_only_cnoc_clk.hw,
	&measure_only_gcc_camera_ahb_clk.hw,
	&measure_only_gcc_camera_xo_clk.hw,
	&measure_only_gcc_disp1_ahb_clk.hw,
	&measure_only_gcc_disp1_xo_clk.hw,
	&measure_only_gcc_disp_ahb_clk.hw,
	&measure_only_gcc_disp_xo_clk.hw,
	&measure_only_gcc_gpu_cfg_ahb_clk.hw,
	&measure_only_gcc_qupv3_wrap1_s4_clk.hw,
	&measure_only_gcc_qupv3_wrap_1_m_ahb_clk.hw,
	&measure_only_gcc_qupv3_wrap_1_s_ahb_clk.hw,
	&measure_only_gcc_video_ahb_clk.hw,
	&measure_only_gcc_video_xo_clk.hw,
	&measure_only_gpu_cc_cx_gfx3d_clk.hw,
	&measure_only_gpu_cc_cx_gfx3d_slv_clk.hw,
	&measure_only_gpu_cc_gx_gfx3d_clk.hw,
	&measure_only_mccc_clk.hw,
	&measure_only_ipa_2x_clk.hw,
	&measure_only_memnoc_clk.hw,
	&measure_only_snoc_clk.hw,
};

static const struct of_device_id clk_debug_match_table[] = {
	{ .compatible = "qcom,direwolf-debugcc" },
	{ }
};

static int clk_debug_direwolf_probe(struct platform_device *pdev)
{
	struct clk *clk;
	int ret, i;

	BUILD_BUG_ON(ARRAY_SIZE(apss_cc_debug_mux_parent_names) !=
		ARRAY_SIZE(apss_cc_debug_mux_sels));
	BUILD_BUG_ON(ARRAY_SIZE(cam_cc_debug_mux_parent_names) !=
		ARRAY_SIZE(cam_cc_debug_mux_sels));
	BUILD_BUG_ON(ARRAY_SIZE(disp_cc_0_debug_mux_parent_names) !=
		ARRAY_SIZE(disp_cc_0_debug_mux_sels));
	BUILD_BUG_ON(ARRAY_SIZE(disp_cc_1_debug_mux_parent_names) !=
		ARRAY_SIZE(disp_cc_1_debug_mux_sels));
	BUILD_BUG_ON(ARRAY_SIZE(gcc_debug_mux_parent_names) !=
		ARRAY_SIZE(gcc_debug_mux_sels));
	BUILD_BUG_ON(ARRAY_SIZE(gpu_cc_debug_mux_parent_names) !=
		ARRAY_SIZE(gpu_cc_debug_mux_sels));
	BUILD_BUG_ON(ARRAY_SIZE(video_cc_debug_mux_parent_names) !=
		ARRAY_SIZE(video_cc_debug_mux_sels));

	clk = devm_clk_get(&pdev->dev, "xo_clk_src");
	if (IS_ERR(clk)) {
		if (PTR_ERR(clk) != -EPROBE_DEFER)
			dev_err(&pdev->dev, "Unable to get xo clock\n");
		return PTR_ERR(clk);
	}

	debug_mux_priv.cxo = clk;

	for (i = 0; i < ARRAY_SIZE(mux_list); i++) {
		if (IS_ERR_OR_NULL(mux_list[i].mux->regmap)) {
			ret = map_debug_bases(pdev,
				mux_list[i].regmap_name, mux_list[i].mux);
			if (ret == -EBADR)
				continue;
			else if (ret)
				return ret;
		}
	}

	for (i = 0; i < ARRAY_SIZE(mux_list); i++) {
		if (!mux_list[i].mux->regmap)
			continue;

		clk = devm_clk_register(&pdev->dev, &mux_list[i].mux->hw);
		if (IS_ERR(clk)) {
			dev_err(&pdev->dev, "Unable to register %s, err:(%d)\n",
				clk_hw_get_name(&mux_list[i].mux->hw),
				PTR_ERR(clk));
			return PTR_ERR(clk);
		}
	}

	for (i = 0; i < ARRAY_SIZE(debugcc_direwolf_hws); i++) {
		clk = devm_clk_register(&pdev->dev, debugcc_direwolf_hws[i]);
		if (IS_ERR(clk)) {
			dev_err(&pdev->dev, "Unable to register %s, err:(%d)\n",
				clk_hw_get_name(debugcc_direwolf_hws[i]),
				PTR_ERR(clk));
			return PTR_ERR(clk);
		}
	}

	ret = clk_debug_measure_register(&gcc_debug_mux.hw);
	if (ret) {
		dev_err(&pdev->dev, "Could not register Measure clocks\n");
		return ret;
	}

	dev_info(&pdev->dev, "Registered debug measure clocks\n");

	return ret;
}

static struct platform_driver clk_debug_driver = {
	.probe = clk_debug_direwolf_probe,
	.driver = {
		.name = "direwolf-debugcc",
		.of_match_table = clk_debug_match_table,
	},
};

static int __init clk_debug_direwolf_init(void)
{
	return platform_driver_register(&clk_debug_driver);
}
fs_initcall(clk_debug_direwolf_init);

MODULE_DESCRIPTION("QTI DEBUG CC DIREWOLF Driver");
MODULE_LICENSE("GPL v2");
