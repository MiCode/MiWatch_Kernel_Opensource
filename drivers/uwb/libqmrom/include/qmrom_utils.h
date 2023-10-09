/*
 * Copyright 2021 Qorvo US, Inc.
 *
 * SPDX-License-Identifier: GPL-2.0 OR Apache-2.0
 *
 * This file is provided under the Apache License 2.0, or the
 * GNU General Public License v2.0.
 *
 */
#ifndef __QMROM_UTILS_H__
#define __QMROM_UTILS_H__

#ifndef __KERNEL__

#include <unistd.h>
#include <stdlib.h>

#define qmrom_msleep(ms)           \
	do {                       \
		usleep(ms * 1000); \
	} while (0)

#define qmrom_alloc(ptr, size)         \
	do {                           \
		ptr = calloc(1, size); \
	} while (0)

#define qmrom_free free
#else

#include <linux/delay.h>
#include <linux/slab.h>

#define qmrom_msleep(ms)                                   \
	do {                                               \
		usleep_range(ms * 1000, ms * 1000);        \
	} while (0)

#define qmrom_alloc(ptr, size)                   \
	do {                                     \
		ptr = kzalloc(size, GFP_KERNEL); \
	} while (0)

#define qmrom_free kfree
#endif
#endif /* __QMROM_UTILS_H__ */
