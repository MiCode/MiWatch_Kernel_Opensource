// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include "board-dt.h"
#include <asm/mach/map.h>
#include <asm/mach/arch.h>

static const char *sa415m_dt_match[] __initconst = {
	"qcom,sa415m",
	NULL
};

static void __init sa415m_init(void)
{
	board_dt_populate(NULL);
}

DT_MACHINE_START(SA415M_DT,
	"Qualcomm Technologies, Inc. SA415M (Flattened Device Tree)")
	.init_machine		= sa415m_init,
	.dt_compat		= sa415m_dt_match,
MACHINE_END
