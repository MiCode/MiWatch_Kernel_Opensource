/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020-2021 The Linux Foundation. All rights reserved.
 */

#ifndef __SMBLITE_CHARGER_REG_H
#define __SMBLITE_CHARGER_REG_H

#include <linux/bitops.h>

#define PERPH_TYPE_OFFSET	0x04
#define TYPE_MASK		GENMASK(7, 0)
#define PERPH_SUBTYPE_OFFSET	0x05
#define SUBTYPE_MASK		GENMASK(7, 0)
#define INT_RT_STS_OFFSET	0x10
#define REVID_REVISION4		0x103

/********************************
 *  CHGR Peripheral Registers  *
 ********************************/
#define BATTERY_CHARGER_STATUS_1_REG(base)		(base.chg_base + 0x06)
#define BATTERY_CHARGER_STATUS_MASK		GENMASK(2, 0)
enum {
	INHIBIT_CHARGE = 0,
	TRICKLE_CHARGE,
	PRE_CHARGE,
	FULLON_CHARGE,
	TAPER_CHARGE,
	TERMINATE_CHARGE,
	PAUSE_CHARGE,
	DISABLE_CHARGE,
};

#define CHGR_CHG_EN_STATUS_REG(base)		(base.chg_base + 0x07)
#define CHARGING_DISABLED_FROM_BOOST_BIT	BIT(6)

#define CHARGER_VBAT_STATUS_REG(base)			(base.chg_base + 0x08)
#define BAT_OV_BIT				BIT(7)

#define BATTERY_TEMP_STATUS_REG(base)			(base.batif_base + 0x0C)
#define BAT_TEMP_STATUS_TOO_HOT_AFP_BIT		BIT(5)
#define BAT_TEMP_STATUS_TOO_HOT_BIT		BIT(4)
#define BAT_TEMP_STATUS_HOT_SOFT_BIT		BIT(3)
#define BAT_TEMP_STATUS_COLD_SOFT_BIT		BIT(2)
#define BAT_TEMP_STATUS_TOO_COLD_BIT		BIT(1)
#define BAT_TEMP_STATUS_TOO_COLD_AFP_BIT	BIT(0)

#define CHARGING_ENABLE_CMD_REG(base)			(base.chg_base + 0x46)
#define CHARGING_ENABLE_CMD_BIT			BIT(0)
#define CHARGING_PAUSE_CMD_BIT			BIT(4)

#define CHGR_FAST_CHARGE_CURRENT_CFG_REG(base)	(base.chg_base + 0x54)
#define CHGR_FLOAT_VOLTAGE_CFG_REG(base)		(base.chg_base + 0x58)

#define CHGR_TERM_CFG_REG(base)			(base.chg_base + 0x60)
#define CHGR_ITERM_USE_ANALOG_BIT		BIT(3)

#define CHGR_ADC_ITERM_UP_THD_MSB_REG(base)		(base.chg_base + 0x64)
#define CHGR_ADC_ITERM_UP_THD_LSB_REG(base)		(base.chg_base + 0x65)
#define CHGR_ADC_ITERM_LO_THD_MSB_REG(base)		(base.chg_base + 0x66)
#define CHGR_ADC_ITERM_LO_THD_LSB_REG(base)		(base.chg_base + 0x67)

#define CHGR_RECHG_CFG_REG(base)			(base.chg_base + 0x70)
#define RECHG_MASK				GENMASK(7, 6)
#define VBAT_BASED_RECHG_BIT			BIT(7)
#define SOC_BASED_RECHG_BIT			GENMASK(7, 6)
#define NO_OF_SAMPLE_FOR_RCHG			GENMASK(1, 0)

#define CHGR_ADC_RECHARGE_THRESHOLD_MSB_REG(base)	(base.chg_base + 0x72)

#define CHARGE_RCHG_SOC_THRESHOLD_CFG_REG(base)	(base.chg_base + 0x74)

#define CHGR_INHIBIT_REG(base)			(base.chg_base + 0x78)
#define CHGR_INHIBIT_BIT			BIT(7)

#define CHGR_INHIBIT_THRESHOLD_CFG_REG(base)		(base.chg_base + 0x7A)

#define CHGR_QG_SOC_REG(base)			(base.chg_base + 0x84)

#define CHGR_QG_SOC_UPDATE_REG(base)		(base.chg_base + 0x85)
#define SOC_UPDATE_PCT_BIT			BIT(0)

#define CHGR_FAST_CHARGE_SAFETY_TIMER_CFG_REG(base)	(base.chg_base + 0x90)
#define FAST_CHARGE_SAFETY_TIMER_EN_BIT		BIT(3)
#define FAST_CHARGE_SAFETY_TIMER_MASK		GENMASK(1, 0)
#define FAST_CHARGE_SAFETY_TIMER_192_MIN	0x0
#define FAST_CHARGE_SAFETY_TIMER_384_MIN	0x1
#define FAST_CHARGE_SAFETY_TIMER_768_MIN	0x2
#define FAST_CHARGE_SAFETY_TIMER_1536_MIN	0x3

/********************************
 *  DCDC Peripheral Registers  *
 ********************************/
#define ICL_MAX_STATUS_REG(base)			(base.dcdc_base + 0x06)
#define ICL_STATUS_REG(base)				(base.dcdc_base + 0x09)

#define POWER_PATH_STATUS_REG(base)			(base.dcdc_base + 0x0B)
#define VALID_INPUT_POWER_SOURCE_STS_BIT	BIT(7)
#define USE_USBIN_BIT				BIT(5)
#define USBIN_SUSPEND_STS_BIT			BIT(3)
#define POWER_PATH_MASK				GENMASK(1, 0)

#define DCDC_CMD_OTG_REG(base)			(base.dcdc_base + 0x50)
#define OTG_EN_BIT				BIT(0)

#define DCDC_BST_VREG_SEL(base)			(base.dcdc_base + 0x52)
#define VBOOST_MASK				GENMASK(1, 0)

#define DCDC_OTG_CFG_REG(base)			(base.dcdc_base + 0x56)
#define OTG_EN_SRC_CFG_BIT			BIT(0)

#define DCDC_LDO_CFG_REG(base)			(base.dcdc_base + 0x70)
#define LDO_MODE_BIT				BIT(0)

#define CONCURRENT_MODE_CFG_REG(base)			(base.dcdc_base + 0x50)
#define CONCURRENT_MODE_EN_BIT			BIT(0)

/********************************
 *  BATIF Peripheral Registers  *
 ********************************/
/* BATIF Interrupt Bits	 */
#define BSM_ACTIVE_RT_STS_BIT			BIT(4)
#define BAT_OV_RT_STS_BIT			BIT(3)
#define BAT_LOW_RT_STS_BIT			BIT(2)
#define BAT_THERM_OR_ID_MISSING_RT_STS_BIT      BIT(1)
#define BAT_TEMP_RT_STS_BIT			BIT(0)

#define SHIP_MODE_REG(base)				(base.batif_base + 0x52)
#define SHIP_MODE_EN_BIT			BIT(0)

#define CHGR_JEITA_HOT_THRESHOLD_REG(base)		(base.batif_base + 0x84)
#define CHGR_JEITA_WARM_THRESHOLD_REG(base)		(base.batif_base + 0x86)
#define CHGR_JEITA_COOL_THRESHOLD_REG(base)		(base.batif_base + 0x88)
#define CHGR_JEITA_COLD_THRESHOLD_REG(base)		(base.batif_base + 0x8A)

#define BATIF_PULLDOWN_VPH_CONTROL(base)		(base.batif_base + 0x55)
#define PULLDOWN_VPH_SW_EN_BIT			BIT(1)
#define PULLDOWN_VPH_HW_EN_BIT			BIT(0)
#define BATIF_PULLDOWN_VPH_SEL_MASK		GENMASK(1, 0)

/********************************
 *  USBIN Peripheral Registers  *
 ********************************/
/* USBIN Interrupt Bits */
#define USBIN_SOURCE_CHANGE_RT_STS_BIT		BIT(7)
#define USBIN_ICL_CHANGE_RT_STS_BIT		BIT(6)
#define USBIN_GT_VT_RT_STS_BIT			BIT(4)
#define USBIN_OV_RT_STS_BIT			BIT(3)
#define USBIN_UV_RT_STS_BIT			BIT(2)
#define USBIN_COLLAPSE_RT_STS_BIT		BIT(1)
#define USBIN_PLUGIN_RT_STS_BIT			BIT(0)

#define USBIN_QC23_EN_REG(base)			(base.usbin_base + 0x48)
#define HVDCP_EN_BIT				BIT(2)
#define HVDCP_NO_AUTH_QC3_CFG_BIT		BIT(0)

#define USBIN_ICL_OPTIONS_REG(base)			(base.usbin_base + 0x50)
#define USBIN_MODE_CHG_BIT			BIT(2)
#define USB51_MODE_BIT				BIT(1)
#define CFG_USB3P0_SEL_BIT			BIT(0)

#define CMD_ICL_OVERRIDE_REG(base)			(base.usbin_base + 0x51)
#define ICL_OVERRIDE_BIT			BIT(0)

#define USBIN_CURRENT_LIMIT_CFG_REG(base)		(base.usbin_base + 0x52)

#define USBIN_INPUT_SUSPEND_REG(base)			(base.usbin_base + 0x54)
#define SUSPEND_ON_COLLAPSE_USBIN_BIT		BIT(7)
#define USBIN_SUSPEND_BIT			BIT(0)
#define DISABLE_USBIN_SUSPEND_ON_COLLAPSE	0

#define USBIN_AICL_OPTIONS_CFG_REG(base)		(base.usbin_base + 0x60)
#define USBIN_AICL_EN_BIT			BIT(7)
#define USBIN_AICL_START_AT_MAX			BIT(4)
#define USBIN_AICL_STEP_TIMING_SEL_MASK		GENMASK(3, 2)
#define USBIN_IN_COLLAPSE_GF_SEL_MASK		GENMASK(1, 0)

#define USBIN_LV_AICL_THRESHOLD_REG(base)		(base.usbin_base + 0x63)

#define USB_CMD_PULLDOWN_REG(base)			(base.usbin_base + 0x70)
#define EN_PULLDOWN_USB_IN_BIT			BIT(0)

#define APSD_STATUS_REG(base)				(base.usbin_base + 0x0a)
#define HVDCP_CHECK_TIMEOUT_BIT			BIT(6)
#define SLOW_PLUGIN_TIMEOUT_BIT			BIT(5)
#define ENUMERATION_DONE_BIT			BIT(4)
#define VADP_CHANGE_DONE_AFTER_AUTH_BIT		BIT(3)
#define QC_AUTH_DONE_STATUS_BIT			BIT(2)
#define QC_CHARGER_BIT				BIT(1)
#define APSD_DTC_STATUS_DONE_BIT		BIT(0)

#define APSD_RESULT_STATUS_REG(base)		(base.usbin_base + 0x0b)
#define APSD_RESULT_STATUS_MASK			GENMASK(6, 0)
#define QC_3P0_BIT				BIT(6)
#define FLOAT_CHARGER_BIT			BIT(4)
#define DCP_CHARGER_BIT				BIT(3)
#define CDP_CHARGER_BIT				BIT(2)
#define OCP_CHARGER_BIT				BIT(1)
#define SDP_CHARGER_BIT				BIT(0)

#define CMD_APSD_REG(base)			(base.usbin_base + 0x45)
#define APSD_RERUN_BIT				BIT(0)

#define USBIN_APSD_EN_REG(base)			(base.usbin_base + 0x44)
#define BC1P2_SRC_DETECT_BIT			BIT(7)

#define CMD_HVDCP_REG(base)			(base.usbin_base + 0x4a)
#define SINGLE_INCREMENT_BIT			BIT(5)
#define SINGLE_DECREMENT_BIT			BIT(4)
#define FORCE_5V_BIT				BIT(1)
#define IDLE_BIT				BIT(0)

#define USB_APSD_CFG_REG(base)			(base.usbin_base + 0x46)
#define FLOAT_OPTIONS_MASK			GENMASK(2, 0)
#define FLOAT_DIS_CHGING_CFG_BIT		BIT(2)
#define SUSPEND_FLOAT_CFG_BIT			BIT(1)
#define FORCE_FLOAT_SDP_CFG_BIT			BIT(0)

/********************************
 *  TYPEC Peripheral Registers  *
 ********************************/
#define TYPE_C_SNK_STATUS_REG(base)			(base.typec_base + 0x06)
#define DETECTED_SRC_TYPE_MASK			GENMASK(6, 0)
#define SNK_DAM_500MA_BIT			BIT(6)
#define SNK_DAM_1500MA_BIT			BIT(5)
#define SNK_DAM_3000MA_BIT			BIT(4)
#define SNK_RP_STD_BIT				BIT(3)
#define SNK_RP_1P5_BIT				BIT(2)
#define SNK_RP_3P0_BIT				BIT(1)
#define SNK_RP_SHORT_BIT			BIT(0)

#define TYPE_C_SRC_STATUS_REG(base)			(base.typec_base + 0x08)
#define DETECTED_SNK_TYPE_MASK			GENMASK(4, 0)
#define SRC_HIGH_BATT_BIT			BIT(5)
#define SRC_DEBUG_ACCESS_BIT			BIT(4)
#define SRC_RD_OPEN_BIT				BIT(3)
#define SRC_RA_OPEN_BIT				BIT(1)
#define AUDIO_ACCESS_RA_RA_BIT			BIT(0)

#define TYPE_C_STATE_MACHINE_STATUS_REG(base)		(base.typec_base + 0x09)
#define TYPEC_ATTACH_DETACH_STATE_BIT		BIT(5)

#define TYPE_C_MISC_STATUS_REG(base)			(base.typec_base + 0x0B)
#define SNK_SRC_MODE_BIT			BIT(6)
#define TYPEC_VBUS_ERROR_STATUS_BIT		BIT(4)
#define TYPEC_TCCDEBOUNCE_DONE_STATUS_BIT	BIT(3)
#define CC_ORIENTATION_BIT			BIT(1)
#define CC_ATTACHED_BIT				BIT(0)

#define LEGACY_CABLE_STATUS_REG(base)			(base.typec_base + 0x0D)
#define TYPEC_LEGACY_CABLE_STATUS_BIT		BIT(1)
#define TYPEC_NONCOMP_LEGACY_CABLE_STATUS_BIT	BIT(0)

#define TYPEC_U_USB_STATUS_REG(base)			(base.typec_base + 0x0F)
#define U_USB_GROUND_NOVBUS_BIT			BIT(6)
#define U_USB_GROUND_BIT			BIT(4)
#define U_USB_FLOAT1_BIT			BIT(2)
#define U_USB_FLOAT2_BIT			BIT(0)

#define TYPE_C_MODE_CFG_REG(base)			(base.typec_base + 0x44)
#define TYPEC_TRY_MODE_MASK			GENMASK(4, 3)
#define EN_TRY_SNK_BIT				BIT(4)
#define EN_TRY_SRC_BIT				BIT(3)
#define TYPEC_POWER_ROLE_CMD_MASK		GENMASK(2, 0)
#define EN_SRC_ONLY_BIT				BIT(2)
#define EN_SNK_ONLY_BIT				BIT(1)
#define TYPEC_DISABLE_CMD_BIT			BIT(0)

#define DEBUG_ACCESS_SRC_CFG_REG(base)		(base.typec_base + 0x4C)
#define EN_UNORIENTED_DEBUG_ACCESS_SRC_BIT	BIT(0)

#define TYPE_C_EXIT_STATE_CFG_REG(base)		(base.typec_base + 0x50)
#define BYPASS_VSAFE0V_DURING_ROLE_SWAP_BIT	BIT(3)
#define SEL_SRC_UPPER_REF_BIT			BIT(2)
#define EXIT_SNK_BASED_ON_CC_BIT		BIT(0)

#define TYPE_C_CURRSRC_CFG_REG(base)			(base.typec_base + 0x52)
#define TYPEC_SRC_RP_SEL_MASK			GENMASK(1, 0)
enum {
	TYPEC_SRC_RP_STD,
	TYPEC_SRC_RP_1P5A,
	TYPEC_SRC_RP_3A,
	TYPEC_SRC_RP_3A_DUPLICATE,
	TYPEC_SRC_RP_MAX_ELEMENTS
};

#define TYPE_C_INTERRUPT_EN_CFG_1_REG(base)		(base.typec_base + 0x5E)
#define TYPEC_LEGACY_CABLE_INT_EN_BIT			BIT(7)
#define TYPEC_NONCOMPLIANT_LEGACY_CABLE_INT_EN_BIT	BIT(6)
#define TYPEC_TRYSOURCE_DETECT_INT_EN_BIT		BIT(5)
#define TYPEC_TRYSINK_DETECT_INT_EN_BIT			BIT(4)
#define TYPEC_CCOUT_DETACH_INT_EN_BIT			BIT(3)
#define TYPEC_CCOUT_ATTACH_INT_EN_BIT			BIT(2)
#define TYPEC_VBUS_DEASSERT_INT_EN_BIT			BIT(1)
#define TYPEC_VBUS_ASSERT_INT_EN_BIT			BIT(0)

#define TYPE_C_INTERRUPT_EN_CFG_2_REG(base)		(base.typec_base + 0x60)
#define TYPEC_SRC_BATT_HPWR_INT_EN_BIT		BIT(6)
#define MICRO_USB_STATE_CHANGE_INT_EN_BIT	BIT(5)
#define TYPEC_STATE_MACHINE_CHANGE_INT_EN_BIT	BIT(4)
#define TYPEC_DEBUG_ACCESS_DETECT_INT_EN_BIT	BIT(3)
#define TYPEC_WATER_DETECTION_INT_EN_BIT	BIT(2)
#define TYPEC_VBUS_ERROR_INT_EN_BIT		BIT(1)
#define TYPEC_DEBOUNCE_DONE_INT_EN_BIT		BIT(0)

#define TYPEC_U_USB_CFG_REG(base)			(base.typec_base + 0x70)
#define EN_MICRO_USB_MODE_BIT			BIT(0)

/********************************
 *  MISC Peripheral Registers  *
 ********************************/
#define TEMP_RANGE_STATUS_REG(base)			(base.misc_base + 0x08)
#define THERM_REG_ACTIVE_BIT			BIT(6)
#define TLIM_BIT				BIT(5)
#define TEMP_RANGE_MASK				GENMASK(4, 1)
#define ALERT_LEVEL_BIT				BIT(4)
#define TEMP_ABOVE_RANGE_BIT			BIT(3)
#define TEMP_WITHIN_RANGE_BIT			BIT(2)
#define TEMP_BELOW_RANGE_BIT			BIT(1)
#define THERMREG_DISABLED_BIT			BIT(0)

#define DIE_TEMP_STATUS_REG(base)			(base.misc_base + 0x09)
#define DIE_TEMP_MASK				GENMASK(3, 0)
#define DIE_TEMP_SHDN_BIT			BIT(3)
#define DIE_TEMP_RST_BIT			BIT(2)
#define DIE_TEMP_UB_BIT				BIT(1)
#define DIE_TEMP_LB_BIT				BIT(0)

#define AICL_STATUS_REG(base)				(base.misc_base + 0x06)
#define SOFT_ILIMIT_BIT				BIT(6)
#define AICL_DONE_BIT				BIT(0)

#define AICL_CMD_REG(base)				(base.misc_base + 0x50)
#define RESTART_AICL_BIT			BIT(1)
#define RERUN_AICL_BIT				BIT(0)

#define MISC_SMB_EN_CMD_REG(base)			(base.misc_base + 0x4C)
#define SMB_EN_OVERRIDE_VALUE_BIT		BIT(0)
#define SMB_EN_OVERRIDE_BIT			BIT(1)

#define MISC_AICL_RERUN_CFG_REG(base)			(base.misc_base + 0x54)
#define USBIN_AICL_PERIODIC_RERUN_EN_BIT	BIT(5)
#define USBIN_AICL_RERUN_TIME_MASK		GENMASK(1, 0)
#define AICL_RERUN_TIME_12S_VAL			0x01

#define WD_CFG_REG(base)				(base.misc_base + 0x58)
#define BITE_WDOG_DISABLE_CHARGING_CFG_BIT	BIT(7)
#define BARK_WDOG_INT_EN_BIT			BIT(4)
#define WDOG_TIMER_EN_ON_PLUGIN_BIT		BIT(1)
#define WDOG_TIMER_EN_BIT			BIT(0)

#define SNARL_BARK_BITE_WD_CFG_REG(base)		(base.misc_base + 0x59)
#define SNARL_WDOG_TIMEOUT_MASK                 GENMASK(2, 0)
#define SNARL_WDOG_TMOUT_62P5MS			0x0
#define SNARL_WDOG_TMOUT_1S			0x4
#define SNARL_WDOG_TMOUT_8S			0x7
#define BARK_WDOG_TIMEOUT_MASK			GENMASK(5, 4)
#define BARK_WDOG_TIMEOUT_SHIFT			4
#define BITE_WDOG_TIMEOUT_MASK			GENMASK(7, 6)
#define BITE_WDOG_TIMEOUT_8S			0x3
#define BITE_WDOG_TIMEOUT_SHIFT			6
#define MIN_WD_BARK_TIME			16

#define BARK_BITE_WDOG_PET_REG(base)			(base.misc_base + 0x5A)
#define BARK_BITE_WDOG_PET_BIT			BIT(0)

/********************************
 *  BOOST Peripheral Registers  *
 ********************************/
#define BOOST_BST_STATUS_REG(base)			(base.boost_base + 0x0D)
#define BOOST_SOFTSTART_DONE_BIT			BIT(3)

#define BOOST_BST_EN_REG(base)				(base.boost_base + 0x46)
#define DCIN_BST_EN_BIT					BIT(1)

#endif /* __SMBLITE_CHARGER_REG_H */
