// SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0
/**
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE
 * Copyright (c) 2020-2021 Robert Bosch GmbH. All rights reserved.
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * This file is free software licensed under the terms of version 2 
 * of the GNU General Public License, available from the file LICENSE-GPL 
 * in the main directory of this source tree.
 *
 * BSD LICENSE
 * Copyright (c) 2020-2021 Robert Bosch GmbH. All rights reserved.
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 **/

/*! \file smi230_data_sync.c
 * \brief Sensor Driver for SMI230 sensors */

/****************************************************************************/

/**\name        Header files
 ****************************************************************************/
#include "smi230.h"
#include "smi230_data_sync.h"

/****************************************************************************/

/** \name       Macros
 ****************************************************************************/

/****************************************************************************/

/**\name        Local structures
 ****************************************************************************/

/****************************************************************************/

/*! Static Function Declarations
 ****************************************************************************/

/*!
 * @brief This API is used to validate the device structure pointer for
 * null conditions.
 *
 * @param[in] dev : Structure instance of smi230_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
static int8_t null_ptr_check(const struct smi230_dev *dev);

/****************************************************************************/

/**\name        Extern Declarations
 ****************************************************************************/

/****************************************************************************/

/**\name        Globals
 ****************************************************************************/

/****************************************************************************/

/**\name        Function definitions
 ****************************************************************************/

/*!
 *  @brief This API is the entry point for smi230 sensors.
 *  It performs the selection of I2C/SPI read mechanism according to the
 *  selected interface and reads the chip-id of accel & gyro sensors.
 *  Additionally the smi230 config file is loaded onto the device.
 */
int8_t smi230_init(struct smi230_dev *dev)
{
    int8_t rslt;

    /* Initialize smi230 accel sensor */
    rslt = smi230_acc_init(dev);

    if (rslt == SMI230_OK)
    {
        /* Initialize smi230 gyro sensor */
        rslt = smi230_gyro_init(dev);
    }

    return rslt;
}

/*!
 *  @brief This API is used to enable/disable and configure the data synchronization
 *  feature.
 */
int8_t smi230_configure_data_synchronization(struct smi230_data_sync_cfg sync_cfg, struct smi230_dev *dev)
{
    int8_t rslt;
    uint16_t data[SMI230_ACCEL_DATA_SYNC_LEN];

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if (rslt == SMI230_OK)
    {
        /* Change sensor meas config */
        switch (sync_cfg.mode)
        {
            case SMI230_ACCEL_DATA_SYNC_MODE_2000HZ:
                dev->accel_cfg.odr = SMI230_ACCEL_ODR_1600_HZ;
                dev->accel_cfg.bw = SMI230_ACCEL_BW_NORMAL;
                dev->gyro_cfg.odr = SMI230_GYRO_BW_230_ODR_2000_HZ;
                dev->gyro_cfg.bw = SMI230_GYRO_BW_230_ODR_2000_HZ;
                break;
            case SMI230_ACCEL_DATA_SYNC_MODE_1000HZ:
                dev->accel_cfg.odr = SMI230_ACCEL_ODR_800_HZ;
                dev->accel_cfg.bw = SMI230_ACCEL_BW_NORMAL;
                dev->gyro_cfg.odr = SMI230_GYRO_BW_116_ODR_1000_HZ;
                dev->gyro_cfg.bw = SMI230_GYRO_BW_116_ODR_1000_HZ;
                break;
            case SMI230_ACCEL_DATA_SYNC_MODE_400HZ:
                dev->accel_cfg.odr = SMI230_ACCEL_ODR_400_HZ;
                dev->accel_cfg.bw = SMI230_ACCEL_BW_NORMAL;
                dev->gyro_cfg.odr = SMI230_GYRO_BW_47_ODR_400_HZ;
                dev->gyro_cfg.bw = SMI230_GYRO_BW_47_ODR_400_HZ;
                break;
            default:
                break;
        }
        rslt = smi230_acc_set_meas_conf(dev);
        if (rslt != SMI230_OK)
        {
            return rslt;
        }

        rslt = smi230_gyro_set_meas_conf(dev);
        if (rslt != SMI230_OK)
        {
            return rslt;
        }

        /* Enable data synchronization */
        data[0] = (sync_cfg.mode & SMI230_ACCEL_DATA_SYNC_MODE_MASK);
        rslt = smi230_acc_write_feature_config(SMI230_ACCEL_DATA_SYNC_ADR, &data[0], SMI230_ACCEL_DATA_SYNC_LEN, dev);
    }

    return rslt;
}

/*!
 *  @brief This API reads the synchronized accel & gyro data from the sensor,
 *  store it in the smi230_sensor_data structure instance
 *  passed by the user.
 */
int8_t smi230_get_synchronized_data(struct smi230_sensor_data *accel,
                                    struct smi230_sensor_data *gyro,
                                    const struct smi230_dev *dev)
{
    int8_t rslt;
    uint8_t reg_addr, data[6];
    uint8_t lsb, msb;
    uint16_t msblsb;

    /* Check for null pointer in the device structure */
    rslt = null_ptr_check(dev);

    /* Proceed if null check is fine */
    if ((rslt == SMI230_OK) && (accel != NULL) && (gyro != NULL))
    {
        /* Read accel x,y sensor data */
        reg_addr = SMI230_ACCEL_GP_0_REG;
        rslt = smi230_acc_get_regs(reg_addr, &data[0], 4, dev);

        if (rslt == SMI230_OK)
        {
            /* Read accel sensor data */
            reg_addr = SMI230_ACCEL_GP_4_REG;
            rslt = smi230_acc_get_regs(reg_addr, &data[4], 2, dev);

            if (rslt == SMI230_OK)
            {
                lsb = data[0];
                msb = data[1];
                msblsb = (msb << 8) | lsb;
                accel->x = ((int16_t) msblsb); /* Data in X axis */

                lsb = data[2];
                msb = data[3];
                msblsb = (msb << 8) | lsb;
                accel->y = ((int16_t) msblsb); /* Data in Y axis */

                lsb = data[4];
                msb = data[5];
                msblsb = (msb << 8) | lsb;
                accel->z = ((int16_t) msblsb); /* Data in Z axis */

                /* Read gyro sensor data */
                rslt = smi230_gyro_get_data(gyro, dev);
            }
        }

    }
    else
    {
        rslt = SMI230_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API configures the synchronization interrupt
 *  based on the user settings in the smi230_int_cfg
 *  structure instance.
 */
int8_t smi230_set_data_sync_int_config(const struct smi230_int_cfg *int_config, const struct smi230_dev *dev)
{
    int8_t rslt;

    /* Configure accel sync data ready interrupt configuration */
    rslt = smi230_acc_set_int_config(&int_config->accel_int_config_1, dev);
    if (rslt != SMI230_OK)
    {
        return rslt;
    }

    rslt = smi230_acc_set_int_config(&int_config->accel_int_config_2, dev);
    if (rslt != SMI230_OK)
    {
        return rslt;
    }

    /* Configure gyro data ready interrupt configuration */
    rslt = smi230_gyro_set_int_config(&int_config->gyro_int_config_1, dev);
    if (rslt != SMI230_OK)
    {
        return rslt;
    }

    rslt = smi230_gyro_set_int_config(&int_config->gyro_int_config_2, dev);

    return rslt;
}

/*****************************************************************************/
/* Static function definition */

/*!
 * @brief This API is used to validate the device structure pointer for
 * null conditions.
 */
static int8_t null_ptr_check(const struct smi230_dev *dev)
{
    int8_t rslt;

    if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_ms == NULL))
    {
        /* Device structure pointer is not valid */
        rslt = SMI230_E_NULL_PTR;
    }
    else
    {
        /* Device structure is fine */
        rslt = SMI230_OK;
    }

    return rslt;
}

/** @}*/
