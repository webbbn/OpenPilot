/**
 ******************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup PIOS_MPU6000 MPU6000 Functions
 * @brief Deals with the hardware interface to the 3-axis gyro
 * @{
 *
 * @file       pios_mpu000.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2012-2013.
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

#ifdef PIOS_INCLUDE_MPU6000

enum pios_mpu6000_dev_magic {
    PIOS_MPU6000_DEV_MAGIC = 0x9da9b3ed,
};

#define PIOS_MPU6000_MAX_DOWNSAMPLE 2
struct mpu6000_dev {
    uint32_t interface_id;
    uint32_t slave_num;
    xSemaphoreHandle data_ready_sema;
    const struct pios_mpu6000_cfg *cfg;
    enum pios_mpu6000_range gyro_range;
    enum pios_mpu6000_accel_range accel_range;
    enum pios_mpu6000_filter filter;
    enum pios_mpu6000_dev_magic   magic;
};

/* Global Variables */
static struct mpu6000_dev *dev;
volatile bool mpu6000_configured = false;
static uint32_t mpu6000_irq = 0;

/* Private functions */
static struct mpu6000_dev *PIOS_MPU6000_alloc(void);
static int32_t PIOS_MPU6000_Validate(struct mpu6000_dev *dev);
static void PIOS_MPU6000_Config(struct pios_mpu6000_cfg const *cfg);

/* External interface fuctions */
extern int32_t PIOS_MPU6000_Read(uint32_t interface_id, uint8_t slave_num, uint8_t addr, uint8_t len, uint8_t *data);
extern int32_t PIOS_MPU6000_GetReg(uint32_t interface_id, uint8_t slave_num, uint8_t reg);
extern int32_t PIOS_MPU6000_SetReg(uint32_t interface_id, uint8_t slave_num, uint8_t reg, uint8_t data);

#define GRAV                       9.81f

#ifdef PIOS_MPU6000_ACCEL
#define PIOS_MPU6000_SAMPLES_BYTES 14
#else
#define PIOS_MPU6000_SAMPLES_BYTES  8
#endif

/**
 * @brief Allocate a new device
 */
static struct mpu6000_dev *PIOS_MPU6000_alloc(void)
{
    struct mpu6000_dev *mpu6000_dev;

    mpu6000_dev = (struct mpu6000_dev *)pvPortMalloc(sizeof(*mpu6000_dev));
    if (!mpu6000_dev) {
        return NULL;
    }

    mpu6000_dev->magic = PIOS_MPU6000_DEV_MAGIC;

    mpu6000_dev->data_ready_sema = xSemaphoreCreateMutex();
    if (mpu6000_dev->data_ready_sema == NULL) {
        vPortFree(mpu6000_dev);
        return NULL;
    }

    mpu6000_dev->interface_id = 0;
    mpu6000_dev->slave_num    = 0;

    return mpu6000_dev;
}

/**
 * @brief Validate the handle to the spi device
 * @returns 0 for valid device or -1 otherwise
 */
static int32_t PIOS_MPU6000_Validate(struct mpu6000_dev *vdev)
{
    if (vdev == NULL) {
        return -1;
    }
    if (vdev->magic != PIOS_MPU6000_DEV_MAGIC) {
        return -2;
    }
    if (vdev->interface_id == 0) {
        return -3;
    }
    return 0;
}

/**
 * @brief Initialize the MPU6000 3-axis gyro sensor.
 * @return 0 for success, -1 for failure
 */
int32_t PIOS_MPU6000_Init(uint32_t interface_id, uint32_t slave_num, const struct pios_mpu6000_cfg *cfg)
{
    dev = PIOS_MPU6000_alloc();
    if (dev == NULL) {
        return -1;
    }

    dev->interface_id = interface_id;
    dev->slave_num    = slave_num;
    dev->cfg = cfg;

    /* Configure the MPU6000 Sensor */
    PIOS_MPU6000_Config(cfg);

    /* Set up EXTI line */
    PIOS_EXTI_Init(cfg->exti_cfg);
    return 0;
}

/**
 * @brief Initialize the MPU6000 3-axis gyro sensor
 * \return none
 * \param[in] PIOS_MPU6000_ConfigTypeDef struct to be used to configure sensor.
 *
 */
static void PIOS_MPU6000_Config(struct pios_mpu6000_cfg const *cfg)
{
    PIOS_MPU6000_Test();

    // Reset chip
    while (PIOS_MPU6000_SetReg(dev->interface_id, dev->slave_num, PIOS_MPU6000_PWR_MGMT_REG, PIOS_MPU6000_PWRMGMT_IMU_RST) != 0) {
        ;
    }

    PIOS_DELAY_WaitmS(50);

    // Reset chip and fifo
    while (PIOS_MPU6000_SetReg(dev->interface_id, dev->slave_num, PIOS_MPU6000_USER_CTRL_REG,
                               PIOS_MPU6000_USERCTL_GYRO_RST |
                               PIOS_MPU6000_USERCTL_SIG_COND |
                               PIOS_MPU6000_USERCTL_FIFO_RST) != 0) {
        ;
    }

    // Wait for reset to finish
    while (PIOS_MPU6000_GetReg(dev->interface_id, dev->slave_num, PIOS_MPU6000_USER_CTRL_REG) &
           (PIOS_MPU6000_USERCTL_GYRO_RST |
            PIOS_MPU6000_USERCTL_SIG_COND |
            PIOS_MPU6000_USERCTL_FIFO_RST)) {
        ;
    }
    PIOS_DELAY_WaitmS(10);

    // Power management configuration
    while (PIOS_MPU6000_SetReg(dev->interface_id, dev->slave_num, PIOS_MPU6000_PWR_MGMT_REG, cfg->Pwr_mgmt_clk) != 0) {
        ;
    }

    // FIFO storage
#if defined(PIOS_MPU6000_ACCEL)
    while (PIOS_MPU6000_SetReg(dev->interface_id, dev->slave_num, PIOS_MPU6000_FIFO_EN_REG, cfg->Fifo_store | PIOS_MPU6000_ACCEL_OUT) != 0) {
        ;
    }
#else
    while (PIOS_MPU6000_SetReg(dev->interface_id, dev->slave_num, PIOS_MPU6000_FIFO_EN_REG, cfg->Fifo_store) != 0) {
        ;
    }
#endif
    PIOS_MPU6000_ConfigureRanges(cfg->gyro_range, cfg->accel_range, cfg->filter);

    // User control configuration
    while (PIOS_MPU6000_SetReg(dev->interface_id, dev->slave_num, PIOS_MPU6000_USER_CTRL_REG, cfg->User_ctl) != 0) {
        ;
    }

    // Power managementconfiguration
    while (PIOS_MPU6000_SetReg(dev->interface_id, dev->slave_num, PIOS_MPU6000_PWR_MGMT_REG, cfg->Pwr_mgmt_clk) != 0) {
        ;
    }

    // Interrupt configuration
    while (PIOS_MPU6000_SetReg(dev->interface_id, dev->slave_num, PIOS_MPU6000_INT_CFG_REG, cfg->interrupt_cfg) != 0) {
        ;
    }

    // Interrupt configuration
    while (PIOS_MPU6000_SetReg(dev->interface_id, dev->slave_num, PIOS_MPU6000_INT_EN_REG, cfg->interrupt_en) != 0) {
        ;
    }
    if ((PIOS_MPU6000_GetReg(dev->interface_id, dev->slave_num, PIOS_MPU6000_INT_EN_REG)) != cfg->interrupt_en) {
        return;
    }

    mpu6000_configured = true;
}
/**
 * @brief Configures Gyro, accel and Filter ranges/setings
 * @return 0 if successful, -1 if device has not been initialized
 */
int32_t PIOS_MPU6000_ConfigureRanges(
    enum pios_mpu6000_range gyroRange,
    enum pios_mpu6000_accel_range accelRange,
    enum pios_mpu6000_filter filterSetting)
{
    if (dev == NULL) {
        return -1;
    }

    // update filter settings
    while (PIOS_MPU6000_SetReg(dev->interface_id, dev->slave_num, PIOS_MPU6000_DLPF_CFG_REG, filterSetting) != 0) {
        ;
    }

    // Sample rate divider, chosen upon digital filtering settings
    while (PIOS_MPU6000_SetReg(dev->interface_id, dev->slave_num, PIOS_MPU6000_SMPLRT_DIV_REG,
                               filterSetting == PIOS_MPU6000_LOWPASS_256_HZ ?
                               dev->cfg->Smpl_rate_div_no_dlp : dev->cfg->Smpl_rate_div_dlp) != 0) {
        ;
    }

    dev->filter = filterSetting;

    // Gyro range
    while (PIOS_MPU6000_SetReg(dev->interface_id, dev->slave_num, PIOS_MPU6000_GYRO_CFG_REG, gyroRange) != 0) {
        ;
    }

    dev->gyro_range = gyroRange;
#if defined(PIOS_MPU6000_ACCEL)
    // Set the accel range
    while (PIOS_MPU6000_SetReg(dev->interface_id, dev->slave_num, PIOS_MPU6000_ACCEL_CFG_REG, accelRange) != 0) {
        ;
    }

    dev->accel_range = accelRange;
#endif
    return 0;
}

/*
 * @brief Read the identification bytes from the MPU6000 sensor
 * \return ID read from MPU6000 or -1 if failure
 */
int32_t PIOS_MPU6000_ReadID()
{
    int32_t mpu6000_id = PIOS_MPU6000_GetReg(dev->interface_id, dev->slave_num, PIOS_MPU6000_WHOAMI);

    if (mpu6000_id < 0) {
        return -1;
    }
    return mpu6000_id;
}

/**
 * \brief Returns the data ready semaphore
 * \return Handle to the semaphore or null if invalid device
 */
xSemaphoreHandle PIOS_MPU6000_GetSemaphore()
{
    if (PIOS_MPU6000_Validate(dev) != 0) {
        return (xSemaphoreHandle)NULL;
    }

    return dev->data_ready_sema;
}


float PIOS_MPU6000_GetScale()
{
    switch (dev->gyro_range) {
    case PIOS_MPU6000_SCALE_250_DEG:
        return 1.0f / 131.0f;

    case PIOS_MPU6000_SCALE_500_DEG:
        return 1.0f / 65.5f;

    case PIOS_MPU6000_SCALE_1000_DEG:
        return 1.0f / 32.8f;

    case PIOS_MPU6000_SCALE_2000_DEG:
        return 1.0f / 16.4f;
    }
    return 0;
}

float PIOS_MPU6000_GetAccelScale()
{
    switch (dev->accel_range) {
    case PIOS_MPU6000_ACCEL_2G:
        return GRAV / 16384.0f;

    case PIOS_MPU6000_ACCEL_4G:
        return GRAV / 8192.0f;

    case PIOS_MPU6000_ACCEL_8G:
        return GRAV / 4096.0f;

    case PIOS_MPU6000_ACCEL_16G:
        return GRAV / 2048.0f;
    }
    return 0;
}

/**
 * @brief Run self-test operation.
 * \return 0 if test succeeded
 * \return non-zero value if test succeeded
 */
int32_t PIOS_MPU6000_Test(void)
{
    /* Verify that ID matches (MPU6000 ID is 0x69) */
    int32_t mpu6000_id = PIOS_MPU6000_ReadID();

    if (mpu6000_id < 0) {
        return -1;
    }

    if (mpu6000_id != 0x68) {
        return -2;
    }

    return 0;
}

/**
 * @brief Obtains the number of bytes in the FIFO.
 * @return the number of bytes in the FIFO
 */
static uint16_t PIOS_MPU6000_FifoDepth()
{
    uint8_t buf[2] = { 0, 0 };
    uint8_t len    = PIOS_MPU6000_Read(dev->interface_id, dev->slave_num, PIOS_MPU6000_FIFO_CNT_MSB, 2, buf);

    if (len < 2) {
        return 0;
    }
    return (buf[0] << 8) | buf[1];
}

/**
 * @brief Get the status code.
 * \return the status code if successful
 * \return negative value if failed
 */
static uint8_t PIOS_MPU6000_GetStatus(void)
{
    int32_t reg;

    while ((reg = PIOS_MPU6000_GetReg(dev->interface_id, dev->slave_num, PIOS_MPU6000_INT_STATUS_REG)) < 0) {
        ;
    }
    return reg;
}

/**
 * @brief Determines if the FIFO has overflowed
 * \return true if an overflow has occurred.
 */
static bool PIOS_MPU6000_Overflowed()
{
    uint8_t status = PIOS_MPU6000_GetStatus();

    return status & PIOS_MPU6000_INT_STATUS_OVERFLOW;
}

/**
 * @brief Resets the Fifo (usually used after an overflow).
 */
static void PIOS_MPU6000_FifoReset()
{
    uint8_t reg = PIOS_MPU6000_GetReg(dev->interface_id, dev->slave_num, PIOS_MPU6000_USER_CTRL_REG);

    PIOS_MPU6000_SetReg(dev->interface_id, dev->slave_num, PIOS_MPU6000_USER_CTRL_REG, reg | PIOS_MPU6000_USERCTL_FIFO_RST);
}

/**
 * @brief Read the sensor values out of the data buffer and rotate appropriately.
 * \param[in] buf The input buffer
 * \param[out] data The scaled and rotated values.
 */
static void PIOS_MPU6000_Rotate(struct pios_mpu6000_data *data, const uint8_t *buf)
{
    // Rotate the sensor to OP convention.  The datasheet defines X as towards the right
    // and Y as forward.  OP convention transposes this.  Also the Z is defined negatively
    // to our convention
#if defined(PIOS_MPU6000_ACCEL)
    uint8_t accel_y_msb = buf[0];
    uint8_t accel_y_lsb = buf[1];
    uint8_t accel_x_msb = buf[2];
    uint8_t accel_x_lsb = buf[3];
    uint8_t accel_z_msb = buf[4];
    uint8_t accel_z_lsb = buf[5];
    uint8_t temp_msb    = buf[6];
    uint8_t temp_lsb    = buf[7];
    uint8_t gyro_y_msb  = buf[8];
    uint8_t gyro_y_lsb  = buf[9];
    uint8_t gyro_x_msb  = buf[10];
    uint8_t gyro_x_lsb  = buf[11];
    uint8_t gyro_z_msb  = buf[12];
    uint8_t gyro_z_lsb  = buf[13];

    // Currently we only support rotations on top so switch X/Y accordingly
    switch (dev->cfg->orientation) {
    case PIOS_MPU6000_TOP_0DEG:
        data->accel_y = accel_y_msb << 8 | accel_y_lsb; // chip X
        data->accel_x = accel_x_msb << 8 | accel_x_lsb; // chip Y
        data->gyro_y  = gyro_y_msb << 8 | gyro_y_lsb; // chip X
        data->gyro_x  = gyro_x_msb << 8 | gyro_x_lsb; // chip Y
        break;
    case PIOS_MPU6000_TOP_90DEG:
        // -1 to bring it back to -32768 +32767 range
        data->accel_y = -1 - (accel_x_msb << 8 | accel_x_lsb); // chip Y
        data->accel_x = accel_y_msb << 8 | accel_y_lsb; // chip X
        data->gyro_y  = -1 - (gyro_x_msb << 8 | gyro_x_lsb); // chip Y
        data->gyro_x  = gyro_y_msb << 8 | gyro_y_lsb; // chip X
        break;
    case PIOS_MPU6000_TOP_180DEG:
        data->accel_y = -1 - (accel_y_msb << 8 | accel_y_lsb); // chip X
        data->accel_x = -1 - (accel_x_msb << 8 | accel_x_lsb); // chip Y
        data->gyro_y  = -1 - (gyro_y_msb << 8 | gyro_y_lsb); // chip X
        data->gyro_x  = -1 - (gyro_x_msb << 8 | gyro_x_lsb); // chip Y
        break;
    case PIOS_MPU6000_TOP_270DEG:
        data->accel_y = accel_x_msb << 8 | accel_x_lsb; // chip Y
        data->accel_x = -1 - (accel_y_msb << 8 | accel_y_lsb); // chip X
        data->gyro_y  = gyro_x_msb << 8 | gyro_x_lsb; // chip Y
        data->gyro_x  = -1 - (gyro_y_msb << 8 | gyro_y_lsb); // chip X
        break;
    }
    data->gyro_z      = -1 - (gyro_z_msb << 8 | gyro_z_lsb);
    data->accel_z     = -1 - (accel_z_msb << 8 | accel_z_lsb);
    data->temperature = temp_msb << 8 | temp_lsb;
#else /* if defined(PIOS_MPU6000_ACCEL) */
    uint8_t temp_msb   = buf[0];
    uint8_t temp_lsb   = buf[1];
    uint8_t gyro_y_msb = buf[2];
    uint8_t gyro_y_lsb = buf[3];
    uint8_t gyro_x_msb = buf[4];
    uint8_t gyro_x_lsb = buf[5];
    uint8_t gyro_z_msb = buf[6];
    uint8_t gyro_z_lsb = buf[7];

    data->gyro_x = accel_x_msb << 8 | accel_x_lsb;
    data->gyro_y = accel_z_msb << 8 | accel_z_lsb;
    switch (dev->cfg->orientation) {
    case PIOS_MPU6000_TOP_0DEG:
        data->gyro_y = accel_x_msb << 8 | accel_x_lsb;
        data->gyro_x = accel_z_msb << 8 | accel_z_lsb;
        break;
    case PIOS_MPU6000_TOP_90DEG:
        data->gyro_y = -1 - (accel_z_msb << 8 | accel_z_lsb); // chip Y
        data->gyro_x = accel_x_msb << 8 | accel_x_lsb; // chip X
        break;
    case PIOS_MPU6000_TOP_180DEG:
        data->gyro_y = -1 - (accel_x_msb << 8 | accel_x_lsb);
        data->gyro_x = -1 - (accel_z_msb << 8 | accel_z_lsb);
        break;
    case PIOS_MPU6000_TOP_270DEG:
        data->gyro_y = accel_z_msb << 8 | accel_z_lsb; // chip Y
        data->gyro_x = -1 - (accel_x_msb << 8 | accel_x_lsb); // chip X
        break;
    }
    data->gyro_z = -1 - (temp_msb << 8 | temp_lsb);
    data->temperature = accel_y_msb << 8 | accel_y_lsb;
#endif /* if defined(PIOS_MPU6000_ACCEL) */
}

/**
 * @brief Read current accel, gyro, and temperature values.
 * \param[out] data The scaled and rotated values.
 * \returns true if succesful
 */
bool PIOS_MPU6000_ReadSensors(struct pios_mpu6000_data *data)
{
    uint8_t buf[PIOS_MPU6000_SAMPLES_BYTES];
    uint8_t len = PIOS_MPU6000_Read(dev->interface_id, dev->slave_num, PIOS_MPU6000_ACCEL_X_OUT_MSB, PIOS_MPU6000_SAMPLES_BYTES, buf);

    if (len < PIOS_MPU6000_SAMPLES_BYTES) {
        return false;
    }
    PIOS_MPU6000_Rotate(data, buf);
    return true;
}

/**
 * @brief Read current accel, gyro, and temperature values.
 * \param[out] data The scaled and rotated values.
 * \returns true if succesful
 */
bool PIOS_MPU6000_ReadFifo(struct pios_mpu6000_data *data)
{
    // Did the FIFO overflow?
    if (PIOS_MPU6000_Overflowed()) {
        PIOS_MPU6000_FifoReset();
        return false;
    }

    // Ensure that there's enough data in the FIFO.
    uint16_t depth = PIOS_MPU6000_FifoDepth();
    if (depth < PIOS_MPU6000_SAMPLES_BYTES) {
        return false;
    }

    // Read the data out of the FIFO.
    uint8_t buf[PIOS_MPU6000_SAMPLES_BYTES];
    uint8_t len = PIOS_MPU6000_Read(dev->interface_id, dev->slave_num, PIOS_MPU6000_FIFO_REG, PIOS_MPU6000_SAMPLES_BYTES, buf);
    if (len < PIOS_MPU6000_SAMPLES_BYTES) {
        return false;
    }
    PIOS_MPU6000_Rotate(data, buf);
    return true;
}

/**
 * @brief EXTI IRQ Handler.  Read all the data from onboard buffer
 * @return a boolean to the EXTI IRQ Handler wrapper indicating if a
 *         higher priority task is now eligible to run
 */
bool PIOS_MPU6000_IRQHandler(void)
{
    mpu6000_irq++;

    portBASE_TYPE xHigherPriorityTaskWoken;
    xSemaphoreGiveFromISR(dev->data_ready_sema, &xHigherPriorityTaskWoken);
    return xHigherPriorityTaskWoken == pdTRUE;
}

#endif /* PIOS_INCLUDE_MPU6000 */

/**
 * @}
 * @}
 */
