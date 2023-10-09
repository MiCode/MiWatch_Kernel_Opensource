/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (c) 2015, 2021, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef UFSHCD_PLTFRM_H_
#define UFSHCD_PLTFRM_H_

#include "ufshcd.h"

#define UFS_PWM_MODE 1
#define UFS_HS_MODE  2

struct ufs_dev_params {
	u32 pwm_rx_gear;        /* pwm rx gear to work in */
	u32 pwm_tx_gear;        /* pwm tx gear to work in */
	u32 hs_rx_gear;         /* hs rx gear to work in */
	u32 hs_tx_gear;         /* hs tx gear to work in */
	u32 rx_lanes;           /* number of rx lanes */
	u32 tx_lanes;           /* number of tx lanes */
	u32 rx_pwr_pwm;         /* rx pwm working pwr */
	u32 tx_pwr_pwm;         /* tx pwm working pwr */
	u32 rx_pwr_hs;          /* rx hs working pwr */
	u32 tx_pwr_hs;          /* tx hs working pwr */
	u32 hs_rate;            /* rate A/B to work in HS */
	u32 desired_working_mode;
};

int ufshcd_get_pwr_dev_param(struct ufs_dev_params *dev_param,
			     struct ufs_pa_layer_attr *dev_max,
			     struct ufs_pa_layer_attr *agreed_pwr);
int ufshcd_pltfrm_init(struct platform_device *pdev,
		       const struct ufs_hba_variant_ops *vops);
void ufshcd_pltfrm_shutdown(struct platform_device *pdev);

#ifdef CONFIG_PM

int ufshcd_pltfrm_suspend(struct device *dev);
int ufshcd_pltfrm_resume(struct device *dev);
int ufshcd_pltfrm_runtime_suspend(struct device *dev);
int ufshcd_pltfrm_runtime_resume(struct device *dev);
int ufshcd_pltfrm_runtime_idle(struct device *dev);

#ifdef CONFIG_SCSI_UFSHCD_QTI
int ufshcd_pltfrm_freeze(struct device *dev);
int ufshcd_pltfrm_restore(struct device *dev);
int ufshcd_pltfrm_thaw(struct device *dev);
#endif /* CONFIG_SCSI_UFSHCD_QTI */

#else /* !CONFIG_PM */

#define ufshcd_pltfrm_suspend	NULL
#define ufshcd_pltfrm_resume	NULL
#define ufshcd_pltfrm_runtime_suspend	NULL
#define ufshcd_pltfrm_runtime_resume	NULL
#define ufshcd_pltfrm_runtime_idle	NULL

#ifdef CONFIG_SCSI_UFSHCD_QTI
#define ufshcd_pltfrm_restore   NULL
#define ufshcd_pltfrm_freeze   NULL
#define ufshcd_pltfrm_thaw   NULL
#endif /* CONFIG_SCSI_UFSHCD_QTI */

#endif /* CONFIG_PM */

#endif /* UFSHCD_PLTFRM_H_ */
