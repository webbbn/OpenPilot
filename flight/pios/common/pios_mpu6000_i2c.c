/**
 ******************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup PIOS_MPU6000 MPU6000 I2C Interface Functions
 * @brief Deals with the hardware interface to the 3-axis gyro
 * @{
 *
 * @file       pios_mpu000_i2c.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2013.
 * @brief      MPU6000 6-axis gyro and accel chip
 * @see        The GNU Public License (GPL) Version 3
 *
 ******************************************************************************
 */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

/* Project Includes */
#include "pios.h"

#ifdef PIOS_INCLUDE_MPU6000_I2C

/**
 * @brief Reads one or more bytes into a buffer
 * \param[in] address MPU6000 register address (depends on size)
 * \param[out] buffer destination buffer
 * \param[in] len number of bytes which should be read
 * \return 0 if operation was successful
 * \return <0 if error during I2C transfer
 */
static int32_t PIOS_MPU6000_I2CRead(uint32_t interface_id, uint8_t slave_num, uint8_t address, uint8_t *buffer, uint8_t len)
{
    uint8_t addr_buffer[] = {
        address,
    };

    const struct pios_i2c_txn txn_list[] = {
        {
            .info = __func__,
            .addr = slave_num,
            .rw   = PIOS_I2C_TXN_WRITE,
            .len  = sizeof(addr_buffer),
            .buf  = addr_buffer,
        },
        {
            .info = __func__,
            .addr = slave_num,
            .rw   = PIOS_I2C_TXN_READ,
            .len  = len,
            .buf  = buffer,
        }
    };

    return PIOS_I2C_Transfer(interface_id, txn_list, NELEMENTS(txn_list));
}

/**
 * @brief Change the interface clock speed
 */
void PIOS_MPU6000_SetClockSpeed(__attribute__((unused)) uint32_t interface_id, __attribute__((unused)) SPIPrescalerTypeDef speed)
{}

/**
 * @brief Read data from the MPU6000
 * @param [in] addr The address to transfer from.
 * @param [in] len The number of bytes to transfer.
 * @param [out] data The output buffer to read the data into.
 * @returns The number of bytes read or < 0 on error.
 */
int32_t PIOS_MPU6000_Read(uint32_t interface_id, uint8_t slave_num, uint8_t addr, uint8_t len, uint8_t *data)
{
    if (PIOS_MPU6000_I2CRead(interface_id, slave_num, addr, data, len) < 0) {
        return -1;
    }
    return len;
}

/**
 * @brief Read a register from MPU6000
 * @returns The register value or -1 if failure.
 * @param reg[in] Register address to be read
 */
int32_t PIOS_MPU6000_GetReg(uint32_t interface_id, uint8_t slave_num, uint8_t reg)
{
    uint8_t data;
    int32_t retval = PIOS_MPU6000_I2CRead(interface_id, slave_num, reg, &data, sizeof(data));

    if (retval != 0) {
        return retval;
    } else {
        return data;
    }
}

/**
 * @brief Writes one byte to the MPU6000
 * \param[in] reg Register address
 * \param[in] data Byte to write
 * \return 0 if operation was successful
 * \return < 0 on failure
 */
int32_t PIOS_MPU6000_SetReg(uint32_t interface_id, uint8_t slave_num, uint8_t reg, uint8_t data)
{
    uint8_t i2c_data[] = {
        reg,
        data,
    };

    const struct pios_i2c_txn txn_list[] = {
        {
            .info = __func__,
            .addr = slave_num,
            .rw   = PIOS_I2C_TXN_WRITE,
            .len  = sizeof(i2c_data),
            .buf  = i2c_data,
        },
    };

    return PIOS_I2C_Transfer(interface_id, txn_list, NELEMENTS(txn_list));
}

#endif /* PIOS_INCLUDE_MPU6000_I2C */
