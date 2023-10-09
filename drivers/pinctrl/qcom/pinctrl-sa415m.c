// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/pinctrl.h>

#include "pinctrl-msm.h"

#define FUNCTION(fname)			                \
	[msm_mux_##fname] = {		                \
		.name = #fname,				\
		.groups = fname##_groups,               \
		.ngroups = ARRAY_SIZE(fname##_groups),	\
	}

#define REG_BASE 0x0
#define REG_SIZE 0x1000
#define PINGROUP(id, f1, f2, f3, f4, f5, f6, f7, f8, f9, wake_off, bit)	\
{					        \
	.name = "gpio" #id,			\
	.pins = gpio##id##_pins,		\
	.npins = (unsigned int)ARRAY_SIZE(gpio##id##_pins),	\
	.funcs = (int[]){			\
		msm_mux_gpio, /* gpio mode */	\
		msm_mux_##f1,			\
		msm_mux_##f2,			\
		msm_mux_##f3,			\
		msm_mux_##f4,			\
		msm_mux_##f5,			\
		msm_mux_##f6,			\
		msm_mux_##f7,			\
		msm_mux_##f8,			\
		msm_mux_##f9			\
	},				        \
	.nfuncs = 10,				\
	.ctl_reg = REG_BASE + REG_SIZE * id,			\
	.io_reg = REG_BASE + 0x4 + REG_SIZE * id,		\
	.intr_cfg_reg = REG_BASE + 0x8 + REG_SIZE * id,		\
	.intr_status_reg = REG_BASE + 0xc + REG_SIZE * id,	\
	.intr_target_reg = REG_BASE + 0x8 + REG_SIZE * id,	\
	.dir_conn_reg = REG_BASE + 0xab000,	\
	.mux_bit = 2,			\
	.pull_bit = 0,			\
	.drv_bit = 6,			\
	.oe_bit = 9,			\
	.in_bit = 0,			\
	.out_bit = 1,			\
	.intr_enable_bit = 0,		\
	.intr_status_bit = 0,		\
	.intr_target_bit = 5,		\
	.intr_target_kpss_val = 3,	\
	.intr_raw_status_bit = 4,	\
	.intr_polarity_bit = 1,		\
	.intr_detection_bit = 2,	\
	.intr_detection_width = 2,	\
	.dir_conn_en_bit = 8,		\
	.wake_reg = REG_BASE + wake_off,	\
	.wake_bit = bit,		\
}

#define SDC_QDSD_PINGROUP(pg_name, ctl, pull, drv)	\
{					        \
	.name = #pg_name,			\
	.pins = pg_name##_pins,			\
	.npins = (unsigned int)ARRAY_SIZE(pg_name##_pins),	\
	.ctl_reg = ctl,				\
	.io_reg = 0,				\
	.intr_cfg_reg = 0,			\
	.intr_status_reg = 0,			\
	.intr_target_reg = 0,			\
	.mux_bit = -1,				\
	.pull_bit = pull,			\
	.drv_bit = drv,				\
	.oe_bit = -1,				\
	.in_bit = -1,				\
	.out_bit = -1,				\
	.intr_enable_bit = -1,			\
	.intr_status_bit = -1,			\
	.intr_target_bit = -1,			\
	.intr_raw_status_bit = -1,		\
	.intr_polarity_bit = -1,		\
	.intr_detection_bit = -1,		\
	.intr_detection_width = -1,		\
}

#define UFS_RESET(pg_name, offset)				\
{					        \
	.name = #pg_name,			\
	.pins = pg_name##_pins,			\
	.npins = (unsigned int)ARRAY_SIZE(pg_name##_pins),	\
	.ctl_reg = offset,			\
	.io_reg = offset + 0x4,			\
	.intr_cfg_reg = 0,			\
	.intr_status_reg = 0,			\
	.intr_target_reg = 0,			\
	.mux_bit = -1,				\
	.pull_bit = 3,				\
	.drv_bit = 0,				\
	.oe_bit = -1,				\
	.in_bit = -1,				\
	.out_bit = 0,				\
	.intr_enable_bit = -1,			\
	.intr_status_bit = -1,			\
	.intr_target_bit = -1,			\
	.intr_raw_status_bit = -1,		\
	.intr_polarity_bit = -1,		\
	.intr_detection_bit = -1,		\
	.intr_detection_width = -1,		\
}

#define QUP_I3C(qup_mode, qup_offset)			\
{						\
	.mode = qup_mode,			\
	.offset = qup_offset,			\
}


static const struct pinctrl_pin_desc sa415m_pins[] = {
	PINCTRL_PIN(0, "GPIO_0"),
	PINCTRL_PIN(1, "GPIO_1"),
	PINCTRL_PIN(2, "GPIO_2"),
	PINCTRL_PIN(3, "GPIO_3"),
	PINCTRL_PIN(4, "GPIO_4"),
	PINCTRL_PIN(5, "GPIO_5"),
	PINCTRL_PIN(6, "GPIO_6"),
	PINCTRL_PIN(7, "GPIO_7"),
	PINCTRL_PIN(8, "GPIO_8"),
	PINCTRL_PIN(9, "GPIO_9"),
	PINCTRL_PIN(10, "GPIO_10"),
	PINCTRL_PIN(11, "GPIO_11"),
	PINCTRL_PIN(12, "GPIO_12"),
	PINCTRL_PIN(13, "GPIO_13"),
	PINCTRL_PIN(14, "GPIO_14"),
	PINCTRL_PIN(15, "GPIO_15"),
	PINCTRL_PIN(16, "GPIO_16"),
	PINCTRL_PIN(17, "GPIO_17"),
	PINCTRL_PIN(18, "GPIO_18"),
	PINCTRL_PIN(19, "GPIO_19"),
	PINCTRL_PIN(20, "GPIO_20"),
	PINCTRL_PIN(21, "GPIO_21"),
	PINCTRL_PIN(22, "GPIO_22"),
	PINCTRL_PIN(23, "GPIO_23"),
	PINCTRL_PIN(24, "GPIO_24"),
	PINCTRL_PIN(25, "GPIO_25"),
	PINCTRL_PIN(26, "GPIO_26"),
	PINCTRL_PIN(27, "GPIO_27"),
	PINCTRL_PIN(28, "GPIO_28"),
	PINCTRL_PIN(29, "GPIO_29"),
	PINCTRL_PIN(30, "GPIO_30"),
	PINCTRL_PIN(31, "GPIO_31"),
	PINCTRL_PIN(32, "GPIO_32"),
	PINCTRL_PIN(33, "GPIO_33"),
	PINCTRL_PIN(34, "GPIO_34"),
	PINCTRL_PIN(35, "GPIO_35"),
	PINCTRL_PIN(36, "GPIO_36"),
	PINCTRL_PIN(37, "GPIO_37"),
	PINCTRL_PIN(38, "GPIO_38"),
	PINCTRL_PIN(39, "GPIO_39"),
	PINCTRL_PIN(40, "GPIO_40"),
	PINCTRL_PIN(41, "GPIO_41"),
	PINCTRL_PIN(42, "GPIO_42"),
	PINCTRL_PIN(43, "GPIO_43"),
	PINCTRL_PIN(44, "GPIO_44"),
	PINCTRL_PIN(45, "GPIO_45"),
	PINCTRL_PIN(46, "GPIO_46"),
	PINCTRL_PIN(47, "GPIO_47"),
	PINCTRL_PIN(48, "GPIO_48"),
	PINCTRL_PIN(49, "GPIO_49"),
	PINCTRL_PIN(50, "GPIO_50"),
	PINCTRL_PIN(51, "GPIO_51"),
	PINCTRL_PIN(52, "GPIO_52"),
	PINCTRL_PIN(53, "GPIO_53"),
	PINCTRL_PIN(54, "GPIO_54"),
	PINCTRL_PIN(55, "GPIO_55"),
	PINCTRL_PIN(56, "GPIO_56"),
	PINCTRL_PIN(57, "GPIO_57"),
	PINCTRL_PIN(58, "GPIO_58"),
	PINCTRL_PIN(59, "GPIO_59"),
	PINCTRL_PIN(60, "GPIO_60"),
	PINCTRL_PIN(61, "GPIO_61"),
	PINCTRL_PIN(62, "GPIO_62"),
	PINCTRL_PIN(63, "GPIO_63"),
	PINCTRL_PIN(64, "GPIO_64"),
	PINCTRL_PIN(65, "GPIO_65"),
	PINCTRL_PIN(66, "GPIO_66"),
	PINCTRL_PIN(67, "GPIO_67"),
	PINCTRL_PIN(68, "GPIO_68"),
	PINCTRL_PIN(69, "GPIO_69"),
	PINCTRL_PIN(70, "GPIO_70"),
	PINCTRL_PIN(71, "GPIO_71"),
	PINCTRL_PIN(72, "GPIO_72"),
	PINCTRL_PIN(73, "GPIO_73"),
	PINCTRL_PIN(74, "GPIO_74"),
	PINCTRL_PIN(75, "GPIO_75"),
	PINCTRL_PIN(76, "GPIO_76"),
	PINCTRL_PIN(77, "GPIO_77"),
	PINCTRL_PIN(78, "GPIO_78"),
	PINCTRL_PIN(79, "GPIO_79"),
	PINCTRL_PIN(80, "GPIO_80"),
	PINCTRL_PIN(81, "GPIO_81"),
	PINCTRL_PIN(82, "GPIO_82"),
	PINCTRL_PIN(83, "GPIO_83"),
	PINCTRL_PIN(84, "GPIO_84"),
	PINCTRL_PIN(85, "GPIO_85"),
	PINCTRL_PIN(86, "GPIO_86"),
	PINCTRL_PIN(87, "GPIO_87"),
	PINCTRL_PIN(88, "GPIO_88"),
	PINCTRL_PIN(89, "GPIO_89"),
	PINCTRL_PIN(90, "GPIO_90"),
	PINCTRL_PIN(91, "GPIO_91"),
	PINCTRL_PIN(92, "GPIO_92"),
	PINCTRL_PIN(93, "GPIO_93"),
	PINCTRL_PIN(94, "GPIO_94"),
	PINCTRL_PIN(95, "GPIO_95"),
	PINCTRL_PIN(96, "GPIO_96"),
	PINCTRL_PIN(97, "GPIO_97"),
	PINCTRL_PIN(98, "GPIO_98"),
	PINCTRL_PIN(99, "GPIO_99"),
	PINCTRL_PIN(100, "SDC1_RCLK"),
	PINCTRL_PIN(101, "SDC1_CLK"),
	PINCTRL_PIN(102, "SDC1_CMD"),
	PINCTRL_PIN(103, "SDC1_DATA"),
};

#define DECLARE_MSM_GPIO_PINS(pin) \
	static const unsigned int gpio##pin##_pins[] = { pin }
DECLARE_MSM_GPIO_PINS(0);
DECLARE_MSM_GPIO_PINS(1);
DECLARE_MSM_GPIO_PINS(2);
DECLARE_MSM_GPIO_PINS(3);
DECLARE_MSM_GPIO_PINS(4);
DECLARE_MSM_GPIO_PINS(5);
DECLARE_MSM_GPIO_PINS(6);
DECLARE_MSM_GPIO_PINS(7);
DECLARE_MSM_GPIO_PINS(8);
DECLARE_MSM_GPIO_PINS(9);
DECLARE_MSM_GPIO_PINS(10);
DECLARE_MSM_GPIO_PINS(11);
DECLARE_MSM_GPIO_PINS(12);
DECLARE_MSM_GPIO_PINS(13);
DECLARE_MSM_GPIO_PINS(14);
DECLARE_MSM_GPIO_PINS(15);
DECLARE_MSM_GPIO_PINS(16);
DECLARE_MSM_GPIO_PINS(17);
DECLARE_MSM_GPIO_PINS(18);
DECLARE_MSM_GPIO_PINS(19);
DECLARE_MSM_GPIO_PINS(20);
DECLARE_MSM_GPIO_PINS(21);
DECLARE_MSM_GPIO_PINS(22);
DECLARE_MSM_GPIO_PINS(23);
DECLARE_MSM_GPIO_PINS(24);
DECLARE_MSM_GPIO_PINS(25);
DECLARE_MSM_GPIO_PINS(26);
DECLARE_MSM_GPIO_PINS(27);
DECLARE_MSM_GPIO_PINS(28);
DECLARE_MSM_GPIO_PINS(29);
DECLARE_MSM_GPIO_PINS(30);
DECLARE_MSM_GPIO_PINS(31);
DECLARE_MSM_GPIO_PINS(32);
DECLARE_MSM_GPIO_PINS(33);
DECLARE_MSM_GPIO_PINS(34);
DECLARE_MSM_GPIO_PINS(35);
DECLARE_MSM_GPIO_PINS(36);
DECLARE_MSM_GPIO_PINS(37);
DECLARE_MSM_GPIO_PINS(38);
DECLARE_MSM_GPIO_PINS(39);
DECLARE_MSM_GPIO_PINS(40);
DECLARE_MSM_GPIO_PINS(41);
DECLARE_MSM_GPIO_PINS(42);
DECLARE_MSM_GPIO_PINS(43);
DECLARE_MSM_GPIO_PINS(44);
DECLARE_MSM_GPIO_PINS(45);
DECLARE_MSM_GPIO_PINS(46);
DECLARE_MSM_GPIO_PINS(47);
DECLARE_MSM_GPIO_PINS(48);
DECLARE_MSM_GPIO_PINS(49);
DECLARE_MSM_GPIO_PINS(50);
DECLARE_MSM_GPIO_PINS(51);
DECLARE_MSM_GPIO_PINS(52);
DECLARE_MSM_GPIO_PINS(53);
DECLARE_MSM_GPIO_PINS(54);
DECLARE_MSM_GPIO_PINS(55);
DECLARE_MSM_GPIO_PINS(56);
DECLARE_MSM_GPIO_PINS(57);
DECLARE_MSM_GPIO_PINS(58);
DECLARE_MSM_GPIO_PINS(59);
DECLARE_MSM_GPIO_PINS(60);
DECLARE_MSM_GPIO_PINS(61);
DECLARE_MSM_GPIO_PINS(62);
DECLARE_MSM_GPIO_PINS(63);
DECLARE_MSM_GPIO_PINS(64);
DECLARE_MSM_GPIO_PINS(65);
DECLARE_MSM_GPIO_PINS(66);
DECLARE_MSM_GPIO_PINS(67);
DECLARE_MSM_GPIO_PINS(68);
DECLARE_MSM_GPIO_PINS(69);
DECLARE_MSM_GPIO_PINS(70);
DECLARE_MSM_GPIO_PINS(71);
DECLARE_MSM_GPIO_PINS(72);
DECLARE_MSM_GPIO_PINS(73);
DECLARE_MSM_GPIO_PINS(74);
DECLARE_MSM_GPIO_PINS(75);
DECLARE_MSM_GPIO_PINS(76);
DECLARE_MSM_GPIO_PINS(77);
DECLARE_MSM_GPIO_PINS(78);
DECLARE_MSM_GPIO_PINS(79);
DECLARE_MSM_GPIO_PINS(80);
DECLARE_MSM_GPIO_PINS(81);
DECLARE_MSM_GPIO_PINS(82);
DECLARE_MSM_GPIO_PINS(83);
DECLARE_MSM_GPIO_PINS(84);
DECLARE_MSM_GPIO_PINS(85);
DECLARE_MSM_GPIO_PINS(86);
DECLARE_MSM_GPIO_PINS(87);
DECLARE_MSM_GPIO_PINS(88);
DECLARE_MSM_GPIO_PINS(89);
DECLARE_MSM_GPIO_PINS(90);
DECLARE_MSM_GPIO_PINS(91);
DECLARE_MSM_GPIO_PINS(92);
DECLARE_MSM_GPIO_PINS(93);
DECLARE_MSM_GPIO_PINS(94);
DECLARE_MSM_GPIO_PINS(95);
DECLARE_MSM_GPIO_PINS(96);
DECLARE_MSM_GPIO_PINS(97);
DECLARE_MSM_GPIO_PINS(98);
DECLARE_MSM_GPIO_PINS(99);

static const unsigned int sdc1_rclk_pins[] = { 100 };
static const unsigned int sdc1_clk_pins[] = { 101 };
static const unsigned int sdc1_cmd_pins[] = { 102 };
static const unsigned int sdc1_data_pins[] = { 103 };
static const unsigned int sdc2_clk_pins[] = { 104 };
static const unsigned int sdc2_cmd_pins[] = { 105 };
static const unsigned int sdc2_data_pins[] = { 106 };

enum sa415m_functions {
	msm_mux_gpio,
	msm_mux_adsp_ext_vfr,
	msm_mux_atest_char_start,
	msm_mux_atest_char_status0,
	msm_mux_atest_char_status1,
	msm_mux_atest_char_status2,
	msm_mux_atest_char_status3,
	msm_mux_audio_ref_clk,
	msm_mux_bimc_dte_test0,
	msm_mux_bimc_dte_test1,
	msm_mux_blsp1,
	msm_mux_blsp2,
	msm_mux_blsp3,
	msm_mux_blsp4,
	msm_mux_blsp_i2c_scl1,
	msm_mux_blsp_i2c_scl2,
	msm_mux_blsp_i2c_scl4,
	msm_mux_blsp_i2c_sda1,
	msm_mux_blsp_i2c_sda2,
	msm_mux_blsp_i2c_sda4,
	msm_mux_blsp_spi_cs1,
	msm_mux_blsp_spi_cs11,
	msm_mux_blsp_spi_cs12,
	msm_mux_blsp_spi_cs13,
	msm_mux_blsp_spi_cs14,
	msm_mux_blsp_spi_cs2,
	msm_mux_blsp_spi_cs21,
	msm_mux_blsp_spi_cs22,
	msm_mux_blsp_spi_cs23,
	msm_mux_blsp_spi_cs24,
	msm_mux_blsp_spi_cs3,
	msm_mux_blsp_spi_cs31,
	msm_mux_blsp_spi_cs32,
	msm_mux_blsp_spi_cs33,
	msm_mux_blsp_spi_cs34,
	msm_mux_blsp_spi_cs4,
	msm_mux_blsp_uart_cts1,
	msm_mux_blsp_uart_cts2,
	msm_mux_blsp_uart_cts3,
	msm_mux_blsp_uart_cts4,
	msm_mux_blsp_uart_rfr1,
	msm_mux_blsp_uart_rfr2,
	msm_mux_blsp_uart_rfr3,
	msm_mux_blsp_uart_rfr4,
	msm_mux_blsp_uart_rx1,
	msm_mux_blsp_uart_rx2,
	msm_mux_blsp_uart_rx4,
	msm_mux_blsp_uart_tx1,
	msm_mux_blsp_uart_tx2,
	msm_mux_blsp_uart_tx4,
	msm_mux_char_exec_pending,
	msm_mux_char_exec_release,
	msm_mux_coex_uart_rx,
	msm_mux_coex_uart_tx,
	msm_mux_cri_trng_rosc,
	msm_mux_cri_trng_rosc0,
	msm_mux_cri_trng_rosc1,
	msm_mux_dbg_out_clk,
	msm_mux_ddr_bist_complete,
	msm_mux_ddr_bist_fail,
	msm_mux_ddr_bist_start,
	msm_mux_ddr_bist_stop,
	msm_mux_ddr_pxi0_test,
	msm_mux_ebi0_wrcdc_dq2,
	msm_mux_ebi0_wrcdc_dq3,
	msm_mux_ebi2_a_d,
	msm_mux_ebi2_lcd_cs,
	msm_mux_ebi2_lcd_reset,
	msm_mux_ebi2_lcd_te,
	msm_mux_emac_gcc_dll0,
	msm_mux_emac_gcc_dll1,
	msm_mux_emac_pps_o,
	msm_mux_ext_dbg_uart,
	msm_mux_gcc_gp1_clk,
	msm_mux_gcc_gp2_clk,
	msm_mux_gcc_gp3_clk,
	msm_mux_gcc_plltest_bypassnl,
	msm_mux_gcc_plltest_resetn,
	msm_mux_i2s_mclk,
	msm_mux_jitter_bist_ref,
	msm_mux_ldo_en,
	msm_mux_ldo_update,
	msm_mux_m_voc_ext,
	msm_mux_mgpi_clk_req,
	msm_mux_native0,
	msm_mux_native1,
	msm_mux_native2,
	msm_mux_native3,
	msm_mux_native_char_start,
	msm_mux_native_tsens_osc,
	msm_mux_native_tsense_pwm1,
	msm_mux_nav_dr_sync,
	msm_mux_nav_pps_in,
	msm_mux_pa_indicator_1,
	msm_mux_pci_e_rst,
	msm_mux_pcie_clkreq_n,
	msm_mux_pll_bist_sync,
	msm_mux_pll_ref_clk,
	msm_mux_pll_test_se,
	msm_mux_pri_mi2s_data0,
	msm_mux_pri_mi2s_data1,
	msm_mux_pri_mi2s_sck,
	msm_mux_pri_mi2s_ws,
	msm_mux_prng_rosc_test,
	msm_mux_qdss0,
	msm_mux_qdss1,
	msm_mux_qdss10,
	msm_mux_qdss11,
	msm_mux_qdss12,
	msm_mux_qdss13,
	msm_mux_qdss14,
	msm_mux_qdss15,
	msm_mux_qdss16,
	msm_mux_qdss17,
	msm_mux_qdss18,
	msm_mux_qdss19,
	msm_mux_qdss2,
	msm_mux_qdss20,
	msm_mux_qdss21,
	msm_mux_qdss22,
	msm_mux_qdss23,
	msm_mux_qdss24,
	msm_mux_qdss25,
	msm_mux_qdss26,
	msm_mux_qdss27,
	msm_mux_qdss28,
	msm_mux_qdss29,
	msm_mux_qdss3,
	msm_mux_qdss30,
	msm_mux_qdss31,
	msm_mux_qdss4,
	msm_mux_qdss5,
	msm_mux_qdss6,
	msm_mux_qdss7,
	msm_mux_qdss8,
	msm_mux_qdss9,
	msm_mux_qdss_cti_trig0,
	msm_mux_qdss_cti_trig1,
	msm_mux_qdss_traceclk,
	msm_mux_qdss_tracectl,
	msm_mux_qlink_en,
	msm_mux_qlink_req,
	msm_mux_sec_mi2s_data0,
	msm_mux_sec_mi2s_data1,
	msm_mux_sec_mi2s_sck,
	msm_mux_sec_mi2s_ws,
	msm_mux_spmi_vgis_clk,
	msm_mux_spmi_vgis_data,
	msm_mux_tgu_ch0_trigout,
	msm_mux_uim1_clk,
	msm_mux_uim1_data,
	msm_mux_uim1_present,
	msm_mux_uim1_reset,
	msm_mux_uim2_clk,
	msm_mux_uim2_data,
	msm_mux_uim2_present,
	msm_mux_uim2_reset,
	msm_mux_usb2phy_ac_en,
	msm_mux_vsense_trigger_mirnat,
	msm_mux_wmss_reset_n,
	msm_mux_NA,
};

static const char * const gpio_groups[] = {
	"gpio0", "gpio1", "gpio2", "gpio3", "gpio4", "gpio5", "gpio6", "gpio7",
	"gpio8", "gpio9", "gpio10", "gpio11", "gpio12", "gpio13", "gpio14",
	"gpio15", "gpio16", "gpio17", "gpio18", "gpio19", "gpio20", "gpio21",
	"gpio22", "gpio23", "gpio24", "gpio25", "gpio26", "gpio27", "gpio28",
	"gpio29", "gpio30", "gpio31", "gpio32", "gpio33", "gpio34", "gpio35",
	"gpio36", "gpio37", "gpio38", "gpio39", "gpio40", "gpio41", "gpio42",
	"gpio43", "gpio44", "gpio45", "gpio46", "gpio47", "gpio48", "gpio49",
	"gpio50", "gpio51", "gpio52", "gpio52", "gpio53", "gpio53", "gpio54",
	"gpio55", "gpio56", "gpio57", "gpio58", "gpio59", "gpio60", "gpio61",
	"gpio62", "gpio63", "gpio64", "gpio65", "gpio66", "gpio67", "gpio68",
	"gpio69", "gpio70", "gpio71", "gpio72", "gpio73", "gpio74", "gpio75",
	"gpio76", "gpio77", "gpio78", "gpio79", "gpio80", "gpio81", "gpio82",
	"gpio83", "gpio84", "gpio85", "gpio86", "gpio87", "gpio88", "gpio89",
	"gpio90", "gpio91", "gpio92", "gpio93", "gpio94", "gpio95", "gpio96",
	"gpio97", "gpio98", "gpio99",
};
static const char * const adsp_ext_vfr_groups[] = {
	"gpio24", "gpio25",
};
static const char * const atest_char_start_groups[] = {
	"gpio63",
};
static const char * const atest_char_status0_groups[] = {
	"gpio67",
};
static const char * const atest_char_status1_groups[] = {
	"gpio66",
};
static const char * const atest_char_status2_groups[] = {
	"gpio65",
};
static const char * const atest_char_status3_groups[] = {
	"gpio64",
};
static const char * const audio_ref_clk_groups[] = {
	"gpio62",
};
static const char * const bimc_dte_test0_groups[] = {
	"gpio14", "gpio59",
};
static const char * const bimc_dte_test1_groups[] = {
	"gpio15", "gpio60",
};
static const char * const blsp1_groups[] = {
	"gpio72", "gpio73", "gpio75",
};
static const char * const blsp2_groups[] = {
	"gpio4", "gpio5", "gpio7",
};
static const char * const blsp3_groups[] = {
	"gpio8", "gpio8", "gpio9", "gpio9", "gpio10", "gpio11", "gpio11",
};
static const char * const blsp4_groups[] = {
	"gpio16", "gpio17", "gpio19",
};
static const char * const blsp_i2c_scl1_groups[] = {
	"gpio3", "gpio75",
};
static const char * const blsp_i2c_scl2_groups[] = {
	"gpio7", "gpio66",
};
static const char * const blsp_i2c_scl4_groups[] = {
	"gpio19", "gpio77",
};
static const char * const blsp_i2c_sda1_groups[] = {
	"gpio2", "gpio74",
};
static const char * const blsp_i2c_sda2_groups[] = {
	"gpio6", "gpio65",
};
static const char * const blsp_i2c_sda4_groups[] = {
	"gpio18", "gpio76",
};
static const char * const blsp_spi_cs1_groups[] = {
	"gpio74",
};
static const char * const blsp_spi_cs11_groups[] = {
	"gpio71",
};
static const char * const blsp_spi_cs12_groups[] = {
	"gpio71",
};
static const char * const blsp_spi_cs13_groups[] = {
	"gpio71",
};
static const char * const blsp_spi_cs14_groups[] = {
	"gpio71",
};
static const char * const blsp_spi_cs2_groups[] = {
	"gpio6",
};
static const char * const blsp_spi_cs21_groups[] = {
	"gpio52",
};
static const char * const blsp_spi_cs22_groups[] = {
	"gpio52",
};
static const char * const blsp_spi_cs23_groups[] = {
	"gpio52",
};
static const char * const blsp_spi_cs24_groups[] = {
	"gpio52",
};
static const char * const blsp_spi_cs3_groups[] = {
	"gpio10",
};
static const char * const blsp_spi_cs31_groups[] = {
	"gpio62",
};
static const char * const blsp_spi_cs32_groups[] = {
	"gpio62",
};
static const char * const blsp_spi_cs33_groups[] = {
	"gpio62",
};
static const char * const blsp_spi_cs34_groups[] = {
	"gpio62",
};
static const char * const blsp_spi_cs4_groups[] = {
	"gpio18",
};
static const char * const blsp_uart_cts1_groups[] = {
	"gpio2", "gpio22",
};
static const char * const blsp_uart_cts2_groups[] = {
	"gpio6", "gpio65",
};
static const char * const blsp_uart_cts3_groups[] = {
	"gpio10",
};
static const char * const blsp_uart_cts4_groups[] = {
	"gpio18", "gpio22",
};
static const char * const blsp_uart_rfr1_groups[] = {
	"gpio3", "gpio23",
};
static const char * const blsp_uart_rfr2_groups[] = {
	"gpio7", "gpio66",
};
static const char * const blsp_uart_rfr3_groups[] = {
	"gpio11",
};
static const char * const blsp_uart_rfr4_groups[] = {
	"gpio19", "gpio23",
};
static const char * const blsp_uart_rx1_groups[] = {
	"gpio1", "gpio21",
};
static const char * const blsp_uart_rx2_groups[] = {
	"gpio5", "gpio64",
};
static const char * const blsp_uart_rx4_groups[] = {
	"gpio17", "gpio21",
};
static const char * const blsp_uart_tx1_groups[] = {
	"gpio0", "gpio20",
};
static const char * const blsp_uart_tx2_groups[] = {
	"gpio4", "gpio63",
};
static const char * const blsp_uart_tx4_groups[] = {
	"gpio16", "gpio20",
};
static const char * const char_exec_pending_groups[] = {
	"gpio6",
};
static const char * const char_exec_release_groups[] = {
	"gpio7",
};
static const char * const coex_uart_rx_groups[] = {
	"gpio45",
};
static const char * const coex_uart_tx_groups[] = {
	"gpio44",
};
static const char * const cri_trng_rosc_groups[] = {
	"gpio36",
};
static const char * const cri_trng_rosc0_groups[] = {
	"gpio40",
};
static const char * const cri_trng_rosc1_groups[] = {
	"gpio41",
};
static const char * const dbg_out_clk_groups[] = {
	"gpio71",
};
static const char * const ddr_bist_complete_groups[] = {
	"gpio46",
};
static const char * const ddr_bist_fail_groups[] = {
	"gpio47",
};
static const char * const ddr_bist_start_groups[] = {
	"gpio48",
};
static const char * const ddr_bist_stop_groups[] = {
	"gpio49",
};
static const char * const ddr_pxi0_test_groups[] = {
	"gpio45", "gpio46",
};
static const char * const ebi0_wrcdc_dq2_groups[] = {
	"gpio2",
};
static const char * const ebi0_wrcdc_dq3_groups[] = {
	"gpio0",
};
static const char * const ebi2_a_d_groups[] = {
	"gpio20",
};
static const char * const ebi2_lcd_cs_groups[] = {
	"gpio21",
};
static const char * const ebi2_lcd_reset_groups[] = {
	"gpio23",
};
static const char * const ebi2_lcd_te_groups[] = {
	"gpio22",
};
static const char * const emac_gcc_dll0_groups[] = {
	"gpio15",
};
static const char * const emac_gcc_dll1_groups[] = {
	"gpio14",
};
static const char * const emac_pps_o_groups[] = {
	"gpio89",
};
static const char * const ext_dbg_uart_groups[] = {
	"gpio8", "gpio9", "gpio10", "gpio11",
};
static const char * const gcc_gp1_clk_groups[] = {
	"gpio18",
};
static const char * const gcc_gp2_clk_groups[] = {
	"gpio19",
};
static const char * const gcc_gp3_clk_groups[] = {
	"gpio11",
};
static const char * const gcc_plltest_bypassnl_groups[] = {
	"gpio73",
};
static const char * const gcc_plltest_resetn_groups[] = {
	"gpio74",
};
static const char * const i2s_mclk_groups[] = {
	"gpio62",
};
static const char * const jitter_bist_ref_groups[] = {
	"gpio19",
};
static const char * const ldo_en_groups[] = {
	"gpio8",
};
static const char * const ldo_update_groups[] = {
	"gpio62",
};
static const char * const m_voc_ext_groups[] = {
	"gpio25", "gpio46", "gpio59", "gpio61",
};
static const char * const mgpi_clk_req_groups[] = {
	"gpio60", "gpio71",
};
static const char * const native0_groups[] = {
	"gpio33",
};
static const char * const native1_groups[] = {
	"gpio32",
};
static const char * const native2_groups[] = {
	"gpio29",
};
static const char * const native3_groups[] = {
	"gpio28",
};
static const char * const native_char_start_groups[] = {
	"gpio26",
};
static const char * const native_tsens_osc_groups[] = {
	"gpio14",
};
static const char * const native_tsense_pwm1_groups[] = {
	"gpio29",
};
static const char * const nav_dr_sync_groups[] = {
	"gpio29", "gpio42", "gpio62",
};
static const char * const nav_pps_in_groups[] = {
	"gpio29", "gpio42", "gpio62",
};
static const char * const pa_indicator_1_groups[] = {
	"gpio33",
};
static const char * const pci_e_rst_groups[] = {
	"gpio53",
};
static const char * const pcie_clkreq_n_groups[] = {
	"gpio56",
};
static const char * const pll_bist_sync_groups[] = {
	"gpio22",
};
static const char * const pll_ref_clk_groups[] = {
	"gpio42",
};
static const char * const pll_test_se_groups[] = {
	"gpio35",
};
static const char * const pri_mi2s_data0_groups[] = {
	"gpio9", "gpio13",
};
static const char * const pri_mi2s_data1_groups[] = {
	"gpio10", "gpio14",
};
static const char * const pri_mi2s_sck_groups[] = {
	"gpio11", "gpio15",
};
static const char * const pri_mi2s_ws_groups[] = {
	"gpio8", "gpio12",
};
static const char * const prng_rosc_test_groups[] = {
	"gpio38",
};
static const char * const qdss0_groups[] = {
	"gpio7", "gpio23",
};
static const char * const qdss1_groups[] = {
	"gpio6", "gpio22",
};
static const char * const qdss10_groups[] = {
	"gpio18", "gpio45",
};
static const char * const qdss11_groups[] = {
	"gpio19", "gpio44",
};
static const char * const qdss12_groups[] = {
	"gpio12", "gpio60",
};
static const char * const qdss13_groups[] = {
	"gpio13", "gpio53",
};
static const char * const qdss14_groups[] = {
	"gpio14", "gpio52",
};
static const char * const qdss15_groups[] = {
	"gpio15", "gpio57",
};
static const char * const qdss16_groups[] = {
	"gpio15",
};
static const char * const qdss17_groups[] = {
	"gpio14",
};
static const char * const qdss18_groups[] = {
	"gpio13",
};
static const char * const qdss19_groups[] = {
	"gpio12",
};
static const char * const qdss2_groups[] = {
	"gpio5", "gpio21",
};
static const char * const qdss20_groups[] = {
	"gpio7",
};
static const char * const qdss21_groups[] = {
	"gpio6",
};
static const char * const qdss22_groups[] = {
	"gpio5",
};
static const char * const qdss23_groups[] = {
	"gpio4",
};
static const char * const qdss24_groups[] = {
	"gpio19",
};
static const char * const qdss25_groups[] = {
	"gpio18",
};
static const char * const qdss26_groups[] = {
	"gpio17",
};
static const char * const qdss27_groups[] = {
	"gpio16",
};
static const char * const qdss28_groups[] = {
	"gpio3",
};
static const char * const qdss29_groups[] = {
	"gpio2",
};
static const char * const qdss3_groups[] = {
	"gpio4", "gpio20",
};
static const char * const qdss30_groups[] = {
	"gpio1",
};
static const char * const qdss31_groups[] = {
	"gpio0",
};
static const char * const qdss4_groups[] = {
	"gpio63", "gpio66",
};
static const char * const qdss5_groups[] = {
	"gpio64", "gpio65",
};
static const char * const qdss6_groups[] = {
	"gpio64", "gpio65",
};
static const char * const qdss7_groups[] = {
	"gpio63", "gpio66",
};
static const char * const qdss8_groups[] = {
	"gpio16", "gpio62",
};
static const char * const qdss9_groups[] = {
	"gpio17", "gpio56",
};
static const char * const qdss_cti_trig0_groups[] = {
	"gpio16", "gpio17", "gpio54", "gpio55", "gpio59", "gpio61", "gpio88",
	"gpio89",
};
static const char * const qdss_cti_trig1_groups[] = {
	"gpio16", "gpio17", "gpio22", "gpio22", "gpio23", "gpio23", "gpio54",
	"gpio55", "gpio88", "gpio89",
};
static const char * const qdss_traceclk_groups[] = {
	"gpio33",
};
static const char * const qdss_tracectl_groups[] = {
	"gpio60",
};
static const char * const qlink_en_groups[] = {
	"gpio34",
};
static const char * const qlink_req_groups[] = {
	"gpio35",
};
static const char * const sec_mi2s_data0_groups[] = {
	"gpio17", "gpio21",
};
static const char * const sec_mi2s_data1_groups[] = {
	"gpio18", "gpio22",
};
static const char * const sec_mi2s_sck_groups[] = {
	"gpio19", "gpio23",
};
static const char * const sec_mi2s_ws_groups[] = {
	"gpio16", "gpio20",
};
static const char * const spmi_vgis_clk_groups[] = {
	"gpio77",
};
static const char * const spmi_vgis_data_groups[] = {
	"gpio76",
};
static const char * const tgu_ch0_trigout_groups[] = {
	"gpio55",
};
static const char * const uim1_clk_groups[] = {
	"gpio70",
};
static const char * const uim1_data_groups[] = {
	"gpio67",
};
static const char * const uim1_present_groups[] = {
	"gpio68",
};
static const char * const uim1_reset_groups[] = {
	"gpio69",
};
static const char * const uim2_clk_groups[] = {
	"gpio3",
};
static const char * const uim2_data_groups[] = {
	"gpio0",
};
static const char * const uim2_present_groups[] = {
	"gpio1",
};
static const char * const uim2_reset_groups[] = {
	"gpio2",
};
static const char * const usb2phy_ac_en_groups[] = {
	"gpio87",
};
static const char * const vsense_trigger_mirnat_groups[] = {
	"gpio14",
};
static const char * const wmss_reset_n_groups[] = {
	"gpio28",
};

static const struct msm_function sa415m_functions[] = {
	FUNCTION(gpio),
	FUNCTION(adsp_ext_vfr),
	FUNCTION(atest_char_start),
	FUNCTION(atest_char_status0),
	FUNCTION(atest_char_status1),
	FUNCTION(atest_char_status2),
	FUNCTION(atest_char_status3),
	FUNCTION(audio_ref_clk),
	FUNCTION(bimc_dte_test0),
	FUNCTION(bimc_dte_test1),
	FUNCTION(blsp1),
	FUNCTION(blsp2),
	FUNCTION(blsp3),
	FUNCTION(blsp4),
	FUNCTION(blsp_i2c_scl1),
	FUNCTION(blsp_i2c_scl2),
	FUNCTION(blsp_i2c_scl4),
	FUNCTION(blsp_i2c_sda1),
	FUNCTION(blsp_i2c_sda2),
	FUNCTION(blsp_i2c_sda4),
	FUNCTION(blsp_spi_cs1),
	FUNCTION(blsp_spi_cs11),
	FUNCTION(blsp_spi_cs12),
	FUNCTION(blsp_spi_cs13),
	FUNCTION(blsp_spi_cs14),
	FUNCTION(blsp_spi_cs2),
	FUNCTION(blsp_spi_cs21),
	FUNCTION(blsp_spi_cs22),
	FUNCTION(blsp_spi_cs23),
	FUNCTION(blsp_spi_cs24),
	FUNCTION(blsp_spi_cs3),
	FUNCTION(blsp_spi_cs31),
	FUNCTION(blsp_spi_cs32),
	FUNCTION(blsp_spi_cs33),
	FUNCTION(blsp_spi_cs34),
	FUNCTION(blsp_spi_cs4),
	FUNCTION(blsp_uart_cts1),
	FUNCTION(blsp_uart_cts2),
	FUNCTION(blsp_uart_cts3),
	FUNCTION(blsp_uart_cts4),
	FUNCTION(blsp_uart_rfr1),
	FUNCTION(blsp_uart_rfr2),
	FUNCTION(blsp_uart_rfr3),
	FUNCTION(blsp_uart_rfr4),
	FUNCTION(blsp_uart_rx1),
	FUNCTION(blsp_uart_rx2),
	FUNCTION(blsp_uart_rx4),
	FUNCTION(blsp_uart_tx1),
	FUNCTION(blsp_uart_tx2),
	FUNCTION(blsp_uart_tx4),
	FUNCTION(char_exec_pending),
	FUNCTION(char_exec_release),
	FUNCTION(coex_uart_rx),
	FUNCTION(coex_uart_tx),
	FUNCTION(cri_trng_rosc),
	FUNCTION(cri_trng_rosc0),
	FUNCTION(cri_trng_rosc1),
	FUNCTION(dbg_out_clk),
	FUNCTION(ddr_bist_complete),
	FUNCTION(ddr_bist_fail),
	FUNCTION(ddr_bist_start),
	FUNCTION(ddr_bist_stop),
	FUNCTION(ddr_pxi0_test),
	FUNCTION(ebi0_wrcdc_dq2),
	FUNCTION(ebi0_wrcdc_dq3),
	FUNCTION(ebi2_a_d),
	FUNCTION(ebi2_lcd_cs),
	FUNCTION(ebi2_lcd_reset),
	FUNCTION(ebi2_lcd_te),
	FUNCTION(emac_gcc_dll0),
	FUNCTION(emac_gcc_dll1),
	FUNCTION(emac_pps_o),
	FUNCTION(ext_dbg_uart),
	FUNCTION(gcc_gp1_clk),
	FUNCTION(gcc_gp2_clk),
	FUNCTION(gcc_gp3_clk),
	FUNCTION(gcc_plltest_bypassnl),
	FUNCTION(gcc_plltest_resetn),
	FUNCTION(i2s_mclk),
	FUNCTION(jitter_bist_ref),
	FUNCTION(ldo_en),
	FUNCTION(ldo_update),
	FUNCTION(m_voc_ext),
	FUNCTION(mgpi_clk_req),
	FUNCTION(native0),
	FUNCTION(native1),
	FUNCTION(native2),
	FUNCTION(native3),
	FUNCTION(native_char_start),
	FUNCTION(native_tsens_osc),
	FUNCTION(native_tsense_pwm1),
	FUNCTION(nav_dr_sync),
	FUNCTION(nav_pps_in),
	FUNCTION(pa_indicator_1),
	FUNCTION(pci_e_rst),
	FUNCTION(pcie_clkreq_n),
	FUNCTION(pll_bist_sync),
	FUNCTION(pll_ref_clk),
	FUNCTION(pll_test_se),
	FUNCTION(pri_mi2s_data0),
	FUNCTION(pri_mi2s_data1),
	FUNCTION(pri_mi2s_sck),
	FUNCTION(pri_mi2s_ws),
	FUNCTION(prng_rosc_test),
	FUNCTION(qdss0),
	FUNCTION(qdss1),
	FUNCTION(qdss10),
	FUNCTION(qdss11),
	FUNCTION(qdss12),
	FUNCTION(qdss13),
	FUNCTION(qdss14),
	FUNCTION(qdss15),
	FUNCTION(qdss16),
	FUNCTION(qdss17),
	FUNCTION(qdss18),
	FUNCTION(qdss19),
	FUNCTION(qdss2),
	FUNCTION(qdss20),
	FUNCTION(qdss21),
	FUNCTION(qdss22),
	FUNCTION(qdss23),
	FUNCTION(qdss24),
	FUNCTION(qdss25),
	FUNCTION(qdss26),
	FUNCTION(qdss27),
	FUNCTION(qdss28),
	FUNCTION(qdss29),
	FUNCTION(qdss3),
	FUNCTION(qdss30),
	FUNCTION(qdss31),
	FUNCTION(qdss4),
	FUNCTION(qdss5),
	FUNCTION(qdss6),
	FUNCTION(qdss7),
	FUNCTION(qdss8),
	FUNCTION(qdss9),
	FUNCTION(qdss_cti_trig0),
	FUNCTION(qdss_cti_trig1),
	FUNCTION(qdss_traceclk),
	FUNCTION(qdss_tracectl),
	FUNCTION(qlink_en),
	FUNCTION(qlink_req),
	FUNCTION(sec_mi2s_data0),
	FUNCTION(sec_mi2s_data1),
	FUNCTION(sec_mi2s_sck),
	FUNCTION(sec_mi2s_ws),
	FUNCTION(spmi_vgis_clk),
	FUNCTION(spmi_vgis_data),
	FUNCTION(tgu_ch0_trigout),
	FUNCTION(uim1_clk),
	FUNCTION(uim1_data),
	FUNCTION(uim1_present),
	FUNCTION(uim1_reset),
	FUNCTION(uim2_clk),
	FUNCTION(uim2_data),
	FUNCTION(uim2_present),
	FUNCTION(uim2_reset),
	FUNCTION(usb2phy_ac_en),
	FUNCTION(vsense_trigger_mirnat),
	FUNCTION(wmss_reset_n),
};

/* Every pin is maintained as a single group, and missing or non-existing pin
 * would be maintained as dummy group to synchronize pin group index with
 * pin descriptor registered with pinctrl core.
 * Clients would not be able to request these dummy pin groups.
 */
static const struct msm_pingroup sa415m_groups[] = {
	[0] = PINGROUP(0, uim2_data, blsp_uart_tx1, qdss31, ebi0_wrcdc_dq3, NA,
			NA, NA, NA, NA, 0, -1),
	[1] = PINGROUP(1, uim2_present, blsp_uart_rx1, qdss30, NA, NA, NA, NA,
			NA, NA, 0, -1),
	[2] = PINGROUP(2, uim2_reset, blsp_uart_cts1, blsp_i2c_sda1, qdss29,
			ebi0_wrcdc_dq2, NA, NA, NA, NA, 0, -1),
	[3] = PINGROUP(3, uim2_clk, blsp_uart_rfr1, blsp_i2c_scl1, qdss28, NA,
			NA, NA, NA, NA, 0, -1),
	[4] = PINGROUP(4, blsp2, blsp_uart_tx2, NA, qdss23, qdss3, NA, NA, NA,
			NA, 0, -1),
	[5] = PINGROUP(5, blsp2, blsp_uart_rx2, NA, qdss22, qdss2, NA, NA, NA,
			NA, 0, -1),
	[6] = PINGROUP(6, blsp_spi_cs2, blsp_uart_cts2, blsp_i2c_sda2, NA,
			qdss21, qdss1, char_exec_pending, NA, NA, 0, -1),
	[7] = PINGROUP(7, blsp2, blsp_uart_rfr2, blsp_i2c_scl2, NA, qdss20,
			qdss0, char_exec_release, NA, NA, 0, -1),
	[8] = PINGROUP(8, pri_mi2s_ws, blsp3, blsp3, ext_dbg_uart, ldo_en, NA,
			NA, NA, NA, 0, -1),
	[9] = PINGROUP(9, pri_mi2s_data0, blsp3, blsp3, ext_dbg_uart, NA, NA,
			NA, NA, NA, 0, -1),
	[10] = PINGROUP(10, pri_mi2s_data1, blsp_spi_cs3, blsp_uart_cts3,
			blsp3, ext_dbg_uart, NA, NA, NA, NA, 0, -1),
	[11] = PINGROUP(11, pri_mi2s_sck, blsp3, blsp_uart_rfr3, blsp3,
			ext_dbg_uart, gcc_gp3_clk, NA, NA, NA, 0, -1),
	[12] = PINGROUP(12, pri_mi2s_ws, NA, qdss19, qdss12, NA, NA, NA, NA,
			NA, 0, -1),
	[13] = PINGROUP(13, pri_mi2s_data0, NA, qdss18, qdss13, NA, NA, NA, NA,
			NA, 0, -1),
	[14] = PINGROUP(14, pri_mi2s_data1, NA, NA, qdss17, qdss14,
			bimc_dte_test0, native_tsens_osc,
			vsense_trigger_mirnat, NA, 0, -1),
	[15] = PINGROUP(15, pri_mi2s_sck, NA, NA, qdss16, qdss15, NA,
			emac_gcc_dll0, bimc_dte_test1, NA, 0, -1),
	[16] = PINGROUP(16, sec_mi2s_ws, blsp4, blsp_uart_tx4, qdss_cti_trig1,
			qdss_cti_trig0, NA, NA, qdss27, qdss8, 0, -1),
	[17] = PINGROUP(17, sec_mi2s_data0, blsp4, blsp_uart_rx4,
			qdss_cti_trig1, qdss_cti_trig0, NA, qdss26, qdss9, NA, 0, -1),
	[18] = PINGROUP(18, sec_mi2s_data1, blsp_spi_cs4, blsp_uart_cts4,
			blsp_i2c_sda4, gcc_gp1_clk, NA, qdss25, qdss10, NA, 0, -1),
	[19] = PINGROUP(19, sec_mi2s_sck, blsp4, blsp_uart_rfr4, blsp_i2c_scl4,
			jitter_bist_ref, gcc_gp2_clk, NA, qdss24, qdss11, 0, -1),
	[20] = PINGROUP(20, sec_mi2s_ws, ebi2_a_d, blsp_uart_tx1,
			blsp_uart_tx4, NA, qdss3, NA, NA, NA, 0, -1),
	[21] = PINGROUP(21, sec_mi2s_data0, ebi2_lcd_cs, blsp_uart_rx1,
			blsp_uart_rx4, NA, NA, qdss2, NA, NA, 0, -1),
	[22] = PINGROUP(22, sec_mi2s_data1, ebi2_lcd_te, blsp_uart_cts1,
			qdss_cti_trig1, qdss_cti_trig1, blsp_uart_cts4,
			pll_bist_sync, NA, qdss1, 0, -1),
	[23] = PINGROUP(23, sec_mi2s_sck, ebi2_lcd_reset, qdss_cti_trig1,
			qdss_cti_trig1, blsp_uart_rfr1, blsp_uart_rfr4, NA,
			qdss0, NA, 0, -1),
	[24] = PINGROUP(24, adsp_ext_vfr, NA, NA, NA, NA, NA, NA, NA, NA, 0, -1),
	[25] = PINGROUP(25, m_voc_ext, adsp_ext_vfr, NA, NA, NA, NA, NA, NA,
			NA, 0, -1),
	[26] = PINGROUP(26, NA, NA, NA, native_char_start, NA, NA, NA, NA, NA, 0, -1),
	[27] = PINGROUP(27, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0, -1),
	[28] = PINGROUP(28, wmss_reset_n, native3, NA, NA, NA, NA, NA, NA, NA, 0, -1),
	[29] = PINGROUP(29, NA, NA, nav_pps_in, nav_dr_sync, NA, native2,
			native_tsense_pwm1, NA, NA, 0, -1),
	[30] = PINGROUP(30, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0, -1),
	[31] = PINGROUP(31, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0, -1),
	[32] = PINGROUP(32, NA, native1, NA, NA, NA, NA, NA, NA, NA, 0, -1),
	[33] = PINGROUP(33, NA, pa_indicator_1, qdss_traceclk, native0, NA, NA,
			NA, NA, NA, 0, -1),
	[34] = PINGROUP(34, qlink_en, NA, NA, NA, NA, NA, NA, NA, NA, 0, -1),
	[35] = PINGROUP(35, qlink_req, pll_test_se, NA, NA, NA, NA, NA, NA, NA, 0, -1),
	[36] = PINGROUP(36, NA, NA, cri_trng_rosc, NA, NA, NA, NA, NA, NA, 0, -1),
	[37] = PINGROUP(37, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0, -1),
	[38] = PINGROUP(38, NA, NA, prng_rosc_test, NA, NA, NA, NA, NA, NA, 0, -1),
	[39] = PINGROUP(39, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0, -1),
	[40] = PINGROUP(40, NA, NA, cri_trng_rosc0, NA, NA, NA, NA, NA, NA, 0, -1),
	[41] = PINGROUP(41, NA, NA, cri_trng_rosc1, NA, NA, NA, NA, NA, NA, 0, -1),
	[42] = PINGROUP(42, nav_pps_in, NA, nav_dr_sync, pll_ref_clk, NA, NA,
			NA, NA, NA, 0, -1),
	[43] = PINGROUP(43, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0, -1),
	[44] = PINGROUP(44, coex_uart_tx, NA, qdss11, NA, NA, NA, NA, NA, NA, 0, -1),
	[45] = PINGROUP(45, coex_uart_rx, NA, qdss10, ddr_pxi0_test, NA, NA,
			NA, NA, NA, 0, -1),
	[46] = PINGROUP(46, m_voc_ext, ddr_bist_complete, ddr_pxi0_test, NA,
			NA, NA, NA, NA, NA, 0, -1),
	[47] = PINGROUP(47, ddr_bist_fail, NA, NA, NA, NA, NA, NA, NA, NA, 0, -1),
	[48] = PINGROUP(48, ddr_bist_start, NA, NA, NA, NA, NA, NA, NA, NA, 0, -1),
	[49] = PINGROUP(49, ddr_bist_stop, NA, NA, NA, NA, NA, NA, NA, NA, 0, -1),
	[50] = PINGROUP(50, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0, -1),
	[51] = PINGROUP(51, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0, -1),
	[52] = PINGROUP(52, blsp_spi_cs22, blsp_spi_cs21, blsp_spi_cs23,
			blsp_spi_cs24, NA, NA, qdss14, NA, NA, 0, -1),
	[53] = PINGROUP(53, pci_e_rst, NA, NA, qdss13, NA, NA, NA, NA, NA, 0, -1),
	[54] = PINGROUP(54, qdss_cti_trig1, qdss_cti_trig0, NA, NA, NA, NA, NA,
			NA, NA, 0, -1),
	[55] = PINGROUP(55, qdss_cti_trig1, qdss_cti_trig0, tgu_ch0_trigout,
			NA, NA, NA, NA, NA, NA, 0, -1),
	[56] = PINGROUP(56, pcie_clkreq_n, NA, qdss9, NA, NA, NA, NA, NA, NA, 0, -1),
	[57] = PINGROUP(57, NA, qdss15, NA, NA, NA, NA, NA, NA, NA, 0, -1),
	[58] = PINGROUP(58, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0, -1),
	[59] = PINGROUP(59, qdss_cti_trig0, m_voc_ext, bimc_dte_test0, NA, NA,
			NA, NA, NA, NA, 0, -1),
	[60] = PINGROUP(60, mgpi_clk_req, NA, qdss12, qdss_tracectl,
			bimc_dte_test1, NA, NA, NA, NA, 0, -1),
	[61] = PINGROUP(61, qdss_cti_trig0, NA, m_voc_ext, NA, NA, NA, NA, NA,
			NA, 0, -1),
	[62] = PINGROUP(62, i2s_mclk, nav_pps_in, nav_dr_sync, audio_ref_clk,
			blsp_spi_cs31, blsp_spi_cs32, blsp_spi_cs33,
			blsp_spi_cs34, ldo_update, 0, -1),
	[63] = PINGROUP(63, blsp_uart_tx2, NA, qdss7, qdss4, atest_char_start,
			NA, NA, NA, NA, 0, -1),
	[64] = PINGROUP(64, blsp_uart_rx2, NA, qdss6, qdss5,
			atest_char_status3, NA, NA, NA, NA, 0, -1),
	[65] = PINGROUP(65, blsp_uart_cts2, blsp_i2c_sda2, NA, qdss5, qdss6,
			atest_char_status2, NA, NA, NA, 0, -1),
	[66] = PINGROUP(66, blsp_uart_rfr2, blsp_i2c_scl2, NA, qdss4, qdss7,
			atest_char_status1, NA, NA, NA, 0, -1),
	[67] = PINGROUP(67, uim1_data, atest_char_status0, NA, NA, NA, NA, NA,
			NA, NA, 0, -1),
	[68] = PINGROUP(68, uim1_present, NA, NA, NA, NA, NA, NA, NA, NA, 0, -1),
	[69] = PINGROUP(69, uim1_reset, NA, NA, NA, NA, NA, NA, NA, NA, 0, -1),
	[70] = PINGROUP(70, uim1_clk, NA, NA, NA, NA, NA, NA, NA, NA, 0, -1),
	[71] = PINGROUP(71, mgpi_clk_req, blsp_spi_cs11, blsp_spi_cs12,
			blsp_spi_cs13, blsp_spi_cs14, dbg_out_clk, NA, NA, NA, 0, -1),
	[72] = PINGROUP(72, NA, blsp1, NA, NA, NA, NA, NA, NA, NA, 0, -1),
	[73] = PINGROUP(73, NA, blsp1, NA, gcc_plltest_bypassnl, NA, NA, NA,
			NA, NA, 0, -1),
	[74] = PINGROUP(74, NA, blsp_spi_cs1, NA, blsp_i2c_sda1,
			gcc_plltest_resetn, NA, NA, NA, NA, 0, -1),
	[75] = PINGROUP(75, NA, blsp1, NA, blsp_i2c_scl1, NA, NA, NA, NA, NA, 0, -1),
	[76] = PINGROUP(76, blsp_i2c_sda4, spmi_vgis_data, NA, NA, NA, NA, NA,
			NA, NA, 0, -1),
	[77] = PINGROUP(77, blsp_i2c_scl4, spmi_vgis_clk, NA, NA, NA, NA, NA,
			NA, NA, 0, -1),
	[78] = PINGROUP(78, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0, -1),
	[79] = PINGROUP(79, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0, -1),
	[80] = PINGROUP(80, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0, -1),
	[81] = PINGROUP(81, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0, -1),
	[82] = PINGROUP(82, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0, -1),
	[83] = PINGROUP(83, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0, -1),
	[84] = PINGROUP(84, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0, -1),
	[85] = PINGROUP(85, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0, -1),
	[86] = PINGROUP(86, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0, -1),
	[87] = PINGROUP(87, NA, NA, usb2phy_ac_en, NA, NA, NA, NA, NA, NA, 0, -1),
	[88] = PINGROUP(88, qdss_cti_trig1, qdss_cti_trig0, NA, NA, NA, NA, NA,
			NA, NA, 0, -1),
	[89] = PINGROUP(89, qdss_cti_trig1, qdss_cti_trig0, emac_pps_o, NA, NA,
			NA, NA, NA, NA, 0, -1),
	[90] = PINGROUP(90, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0, -1),
	[91] = PINGROUP(91, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0, -1),
	[92] = PINGROUP(92, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0, -1),
	[93] = PINGROUP(93, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0, -1),
	[94] = PINGROUP(94, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0, -1),
	[95] = PINGROUP(95, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0, -1),
	[96] = PINGROUP(96, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0, -1),
	[97] = PINGROUP(97, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0, -1),
	[98] = PINGROUP(98, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0, -1),
	[99] = PINGROUP(99, NA, NA, NA, NA, NA, NA, NA, NA, NA, 0, -1),
	[100] = SDC_QDSD_PINGROUP(sdc1_rclk, 0x399a000, 15, 0),
	[101] = SDC_QDSD_PINGROUP(sdc1_clk, 0x399a000, 13, 6),
	[102] = SDC_QDSD_PINGROUP(sdc1_cmd, 0x399a000, 11, 3),
	[103] = SDC_QDSD_PINGROUP(sdc1_data, 0x399a000, 9, 0),
	[104] = SDC_QDSD_PINGROUP(sdc2_clk, 0x0, 14, 6),
	[105] = SDC_QDSD_PINGROUP(sdc2_cmd, 0x0, 11, 3),
	[106] = SDC_QDSD_PINGROUP(sdc2_data, 0x0, 9, 0),
};
static struct pinctrl_qup sa415m_qup_regs[] = {
};

static const struct msm_gpio_wakeirq_map sa415m_pdc_map[] = {
	{ 42, 20 }, { 53, 21 }, { 56, 22 }, { 57, 23 }, { 78, 24 },
	{ 84, 25 }, { 97, 26 },
};

static struct msm_dir_conn sa415m_dir_conn[] = {
	  {-1, 0}, {-1, 0}, {-1, 0}, {-1, 0}, {-1, 0},
	  {-1, 0}, {-1, 0}, {-1, 0}, {-1, 0}
};

static const struct msm_pinctrl_soc_data sa415m_pinctrl = {
	.pins = sa415m_pins,
	.npins = ARRAY_SIZE(sa415m_pins),
	.functions = sa415m_functions,
	.nfunctions = ARRAY_SIZE(sa415m_functions),
	.groups = sa415m_groups,
	.ngroups = ARRAY_SIZE(sa415m_groups),
	.ngpios = 100,
	.qup_regs = sa415m_qup_regs,
	.nqup_regs = ARRAY_SIZE(sa415m_qup_regs),
	.wakeirq_map = sa415m_pdc_map,
	.nwakeirq_map = ARRAY_SIZE(sa415m_pdc_map),
	.dir_conn = sa415m_dir_conn,
};

static int sa415m_pinctrl_probe(struct platform_device *pdev)
{
	return msm_pinctrl_probe(pdev, &sa415m_pinctrl);
}

static const struct of_device_id sa415m_pinctrl_of_match[] = {
	{ .compatible = "qcom,sa415m-pinctrl", },
	{ },
};

static struct platform_driver sa415m_pinctrl_driver = {
	.driver = {
		.name = "sa415m-pinctrl",
		.of_match_table = sa415m_pinctrl_of_match,
	},
	.probe = sa415m_pinctrl_probe,
	.remove = msm_pinctrl_remove,
};

static int __init sa415m_pinctrl_init(void)
{
	return platform_driver_register(&sa415m_pinctrl_driver);
}
arch_initcall(sa415m_pinctrl_init);

static void __exit sa415m_pinctrl_exit(void)
{
	platform_driver_unregister(&sa415m_pinctrl_driver);
}
module_exit(sa415m_pinctrl_exit);

MODULE_DESCRIPTION("QTI sa415m pinctrl driver");
MODULE_LICENSE("GPL v2");
MODULE_DEVICE_TABLE(of, sa415m_pinctrl_of_match);
