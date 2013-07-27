/**
 ******************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup PIOS_MPU6000 MPU6000 SPI Interface Functions
 * @brief Deals with the hardware interface to the 3-axis gyro
 * @{
 *
 * @file       pios_mpu000_spi.c
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

#ifdef PIOS_INCLUDE_MPU6000_SPI

/**
 * @brief Claim the SPI bus for the accel communications and select this chip
 * @return 0 if successful, -1 for invalid device, -2 if unable to claim bus
 */
int32_t PIOS_MPU6000_ClaimBus(uint32_t interface_id, uint32_t slave_num, bool slow_bus)
{
    if (PIOS_SPI_ClaimBus(interface_id) != 0) {
        return -2;
    }
    if (slow_bus) {
        PIOS_SPI_SetClockSpeed(interface_id, PIOS_SPI_PRESCALER_256);
    }
    PIOS_SPI_RC_PinSet(interface_id, slave_num, 0);
    return 0;
}

/**
 * @brief Release the SPI bus for the accel communications and end the transaction
 * @return 0 if successful
 */
int32_t PIOS_MPU6000_ReleaseBus(uint32_t interface_id, uint8_t slave_num, bool speedup_bus)
{
    PIOS_SPI_RC_PinSet(interface_id, slave_num, 1);
    if (speedup_bus) {
        PIOS_SPI_SetClockSpeed(interface_id, PIOS_SPI_PRESCALER_16);
    }
    PIOS_SPI_SetClockSpeed(interface_id, PIOS_SPI_PRESCALER_16);
    return PIOS_SPI_ReleaseBus(interface_id);
}

/**
 * @brief Read data from the MPU6000
 * @param [in] addr The address to transfer from.
 * @param [in] len The number of bytes to transfer.
 * @param [out] data The output buffer to read the data into.
 * @returns The number of bytes read or < 0 on error.
 */
int32_t PIOS_MPU6000_Read(uint32_t interface_id, uint8_t slave_num, uint8_t addr, uint8_t len, uint8_t *data)
{
    if (PIOS_MPU6000_ClaimBus(interface_id, slave_num, false) != 0) {
        return -1;
    }

    static uint8_t mpu6000_send_buf[1 + sizeof(struct pios_mpu6000_data)] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    static uint8_t mpu6000_rec_buf[1 + sizeof(struct pios_mpu6000_data)];
    mpu6000_send_buf[0] = addr | 0x80;
    if (PIOS_SPI_TransferBlock(interface_id, mpu6000_send_buf, mpu6000_rec_buf, len + 1, NULL) < 0) {
        PIOS_MPU6000_ReleaseBus(interface_id, slave_num, false);
        return -2;
    }
    memcpy(data, mpu6000_rec_buf + 1, len);

    PIOS_MPU6000_ReleaseBus(interface_id, slave_num, false);
    return len;
}

/**
 * @brief Read a register from MPU6000
 * @returns The register value or -1 if failure to get bus
 * @param reg[in] Register address to be read
 */
int32_t PIOS_MPU6000_GetReg(uint32_t interface_id, uint8_t slave_num, uint8_t reg)
{
    uint8_t data;

    if (PIOS_MPU6000_ClaimBus(interface_id, slave_num, false) != 0) {
        return -1;
    }

    PIOS_SPI_TransferByte(interface_id, (0x80 | reg)); // request byte
    data = PIOS_SPI_TransferByte(interface_id, 0); // receive response

    PIOS_MPU6000_ReleaseBus(interface_id, slave_num, false);
    return data;
}

/**
 * @brief Writes one byte to the MPU6000
 * \param[in] reg Register address
 * \param[in] data Byte to write
 * \return 0 if operation was successful
 * \return -1 if unable to claim SPI bus
 * \return -2 if unable to claim i2c device
 */
int32_t PIOS_MPU6000_SetReg(uint32_t interface_id, uint8_t slave_num, uint8_t reg, uint8_t data)
{
    if (PIOS_MPU6000_ClaimBus(interface_id, slave_num, true) != 0) {
        return -1;
    }

    if (PIOS_SPI_TransferByte(interface_id, 0x7f & reg) != 0) {
        PIOS_MPU6000_ReleaseBus(interface_id, slave_num, false);
        return -2;
    }

    if (PIOS_SPI_TransferByte(interface_id, data) != 0) {
        PIOS_MPU6000_ReleaseBus(interface_id, slave_num, false);
        return -3;
    }

    PIOS_MPU6000_ReleaseBus(interface_id, slave_num, true);

    return 0;
}

#endif /* PIOS_INCLUDE_MPU6000_SPI */
