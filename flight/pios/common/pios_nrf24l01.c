/**
 ******************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup   PIOS_NFR24L01 Radio Functions
 * @brief PIOS interface for for the NFR24L01 radio
 * @{
 *
 * @file       pios_nfr24l01.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2013.
 * @brief      Implements a driver the the nfr24l01 driver
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
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

#include "pios.h"

#ifdef PIOS_INCLUDE_NRF24L01

#include <pios_spi.h>
#include <pios_gpio_priv.h>
#include <pios_nrf24l01_priv.h>

/* NRF24L SPI commands */
#define PIOS_NRF24L01_CMD_R_REG            0x00
#define PIOS_NRF24L01_CMD_W_REG            0x20
#define PIOS_NRF24L01_CMD_R_RX_PAYLOAD     0x61
#define PIOS_NRF24L01_CMD_W_TX_PAYLOAD     0xA0
#define PIOS_NRF24L01_CMD_FLUSH_TX         0xE1
#define PIOS_NRF24L01_CMD_FLUSH_RX         0xE2
#define PIOS_NRF24L01_CMD_REUSE_TX_PL      0xE3
#define PIOS_NRF24L01_CMD_ACTIVATE         0x50
#define PIOS_NRF24L01_CMD_RX_PL_WID        0x60
#define PIOS_NRF24L01_CMD_W_ACK_PAYLOAD(P) (0xA8 | (P & 0x0F))
#define PIOS_NRF24L01_CMD_W_PAYLOAD_NO_ACK 0xD0
#define PIOS_NRF24L01_CMD_NOP              0xFF
#define PIOS_NRF24L01_DUMMY_BYTE           0xA5

/* Registers address definition */
#define PIOS_NRF24L01_REG_CONFIG           0x00
#define PIOS_NRF24L01_REG_EN_AA            0x01
#define PIOS_NRF24L01_REG_EN_RXADDR        0x02
#define PIOS_NRF24L01_REG_SETUP_AW         0x03
#define PIOS_NRF24L01_REG_SETUP_RETR       0x04
#define PIOS_NRF24L01_REG_RF_CH            0x05
#define PIOS_NRF24L01_REG_RF_SETUP         0x06
#define PIOS_NRF24L01_REG_STATUS           0x07
#define PIOS_NRF24L01_REG_OBSERVE_TX       0x08
#define PIOS_NRF24L01_REG_RPD              0x09
#define PIOS_NRF24L01_REG_RX_ADDR_P0       0x0A
#define PIOS_NRF24L01_REG_RX_ADDR_P1       0x0B
#define PIOS_NRF24L01_REG_RX_ADDR_P2       0x0C
#define PIOS_NRF24L01_REG_RX_ADDR_P3       0x0D
#define PIOS_NRF24L01_REG_RX_ADDR_P4       0x0E
#define PIOS_NRF24L01_REG_RX_ADDR_P5       0x0F
#define PIOS_NRF24L01_REG_TX_ADDR          0x10
#define PIOS_NRF24L01_REG_RX_PW_P0         0x11
#define PIOS_NRF24L01_REG_RX_PW_P1         0x12
#define PIOS_NRF24L01_REG_RX_PW_P2         0x13
#define PIOS_NRF24L01_REG_RX_PW_P3         0x14
#define PIOS_NRF24L01_REG_RX_PW_P4         0x15
#define PIOS_NRF24L01_REG_RX_PW_P5         0x16
#define PIOS_NRF24L01_REG_FIFO_STATUS      0x17
#define PIOS_NRF24L01_REG_DYNPD            0x1C
#define PIOS_NRF24L01_REG_FEATURE          0x1D

#define PIOS_NRF24L01_RF_SETUP_250K        0x26
#define PIOS_NRF24L01_RF_SETUP_1M          0x06
#define PIOS_NRF24L01_RF_SETUP_2M          0x0E

#define PIOS_NRF24L01_SETUP_AW_3B          1
#define PIOS_NRF24L01_SETUP_AW_4B          2
#define PIOS_NRF24L01_SETUP_AW_5B          3

/* Task constants */
#define STACK_SIZE_BYTES                   100
#define TASK_PRIORITY                      (tskIDLE_PRIORITY + 2)

/* Defaults */
#define PIOS_NRF24L01_DEFAULT_DATARATE     PIOS_NRF24L01_RF_SETUP_250K
#define PIOS_NRF24L01_DEFAULT_CHANNEL      10
static const uint8_t PIOS_NRF24L01_DEFAULT_RADIO_ADDRESS[5] = { 0xE7, 0xE7, 0xE7, 0xE7, 0xE7 };

/* Structure definitons */
struct pios_nrf24l01_dev {
    const struct pios_nrf24l01_cfg *cfg;
    struct pios_gpio_cfg gpios;
    uint32_t    ce_gpio;
    uint32_t    spi_id;

    // The task handle
    xTaskHandle task_handle;

    // Signaled by the IRQ handler when data is ready to be read.
    xSemaphoreHandle data_ready;

    // The receive buffer.
    uint8_t rx_buffer[32];
};

/* Forward declarations */
static void PIOS_NRF24L01_Task(void *parameters);
static void PIOS_NRF24L01_ChipEnable(struct pios_nrf24l01_dev *dev);
static void PIOS_NRF24L01_ChipDisable(struct pios_nrf24l01_dev *dev);
static void PIOS_NRF24L01_ChipSelect(struct pios_nrf24l01_dev *dev);
static void PIOS_NRF24L01_ChipDeselect(struct pios_nrf24l01_dev *dev);
static uint8_t PIOS_NRF24L01_SendByte(struct pios_nrf24l01_dev *dev, uint8_t byte);
static uint8_t PIOS_NRF24L01_ReceiveByte(struct pios_nrf24l01_dev *dev);
static int32_t PIOS_NRF24L01_ReadReg(struct pios_nrf24l01_dev *dev, uint8_t address, uint8_t *buffer, int len);
static int32_t PIOS_NRF24L01_WriteReg(struct pios_nrf24l01_dev *dev, uint8_t address, const uint8_t *buffer, int len);
static uint8_t PIOS_NRF24L01_Read1Reg(struct pios_nrf24l01_dev *dev, uint8_t address);
static int32_t PIOS_NRF24L01_Write1Reg(struct pios_nrf24l01_dev *dev, uint8_t address, uint8_t byte);
static uint8_t PIOS_NRF24L01_ReadStatus(struct pios_nrf24l01_dev *dev);
static uint8_t PIOS_NRF24L01_FlushRx(struct pios_nrf24l01_dev *dev);
static uint8_t PIOS_NRF24L01_FlushTx(struct pios_nrf24l01_dev *dev);
static uint8_t PIOS_NRF24L01_RxLength(struct pios_nrf24l01_dev *dev);
static uint8_t PIOS_NRF24L01_ReadRX(struct pios_nrf24l01_dev *dev, uint8_t *buffer, uint8_t len);

int32_t PIOS_NRF24L01_Init(uint32_t *nfr24l01_id, uint32_t spi_id, const struct pios_nrf24l01_cfg *cfg)
{
    struct pios_nrf24l01_dev *dev = (struct pios_nrf24l01_dev *)pvPortMalloc(sizeof(struct pios_nrf24l01_dev));

    dev->spi_id          = spi_id;
    dev->cfg             = cfg;
    dev->gpios.gpios     = cfg->gpio_cfg;
    dev->gpios.num_gpios = 1;
    *nfr24l01_id         = (uint32_t)dev;

    /* Disable the chip select */
    PIOS_NRF24L01_ChipDeselect(dev);

    /* Configure the GPIO line */
    PIOS_GPIO_Init(&dev->ce_gpio, &dev->gpios);

    /* Disable the chip enable */
    PIOS_NRF24L01_ChipDisable(dev);

    /* Configure up EXTI line */
    PIOS_EXTI_Init(cfg->exti_cfg);

    /* Default the datarate and channel. */
    PIOS_NRF24L01_SetChannel(*nfr24l01_id, PIOS_NRF24L01_DEFAULT_CHANNEL);
    PIOS_NRF24L01_SetDatarate(*nfr24l01_id, PIOS_NRF24L01_DEFAULT_DATARATE);

    /* Set the radio address */
    PIOS_NRF24L01_SetAddress(*nfr24l01_id, 0, PIOS_NRF24L01_DEFAULT_RADIO_ADDRESS);

    /* Power the radio, Enable the DS interruption, set the radio in PRX mode */
    PIOS_NRF24L01_Write1Reg(dev, PIOS_NRF24L01_REG_CONFIG, 0x3F);
    uint8_t val = PIOS_NRF24L01_Read1Reg(dev, PIOS_NRF24L01_REG_CONFIG);
    if (val != 0x3f) {
        PIOS_Assert(0);
    }

    /* Wait for the chip to be ready */
    vTaskDelay(2 * configTICK_RATE_HZ / 1000);

    /* Enable the dynamic payload size and the ack payload for the pipe 0 */
    PIOS_NRF24L01_Write1Reg(dev, PIOS_NRF24L01_REG_FEATURE, 0x06);
    PIOS_NRF24L01_Write1Reg(dev, PIOS_NRF24L01_REG_DYNPD, 0x01);

    /* Flush RX and Tx */
    for (uint8_t i = 0; i < 3; i++) {
        PIOS_NRF24L01_FlushRx(dev);
    }
    for (uint8_t i = 0; i < 3; i++) {
        PIOS_NRF24L01_FlushTx(dev);
    }

    /* Initialise the semaphore */
    vSemaphoreCreateBinary(dev->data_ready);

    /*
     * Start the driver task.
     * This task controls the radio state machine and removed all of the IO from the IRQ handler.
     */
    xTaskCreate(PIOS_NRF24L01_Task, (signed char *)"PIOS_NRF24L01_Task", STACK_SIZE_BYTES, (void *)dev,
                TASK_PRIORITY, &(dev->task_handle));

    return 0;
}

/**
 * Handle an interrupt on the NRF24L01 device.
 *
 * @param[in] nrf24l01_id  The device ID.
 * @return True if xHigherPriorityTaskWoken == pdTRUE
 */
bool PIOS_NRF24L01_IRQHandler(uint32_t nrf24l01_id)
{
    struct pios_nrf24l01_dev *dev = (struct pios_nrf24l01_dev *)nrf24l01_id;
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    // Unlock radio task
    xSemaphoreGiveFromISR(dev->data_ready, &xHigherPriorityTaskWoken);

    return xHigherPriorityTaskWoken == pdTRUE;
}

/**
 * Set the datarate for the NRF24L01 modem.
 *
 * @param[in] nrf24l01_id  The device ID.
 * @param[in] datarate  The datarate.
 */
void PIOS_NRF24L01_SetDatarate(uint32_t nrf24l01_id, enum pios_nrf24l01_datarate datarate)
{
    struct pios_nrf24l01_dev *dev = (struct pios_nrf24l01_dev *)nrf24l01_id;

    switch (datarate) {
    case PIOS_NRF24L01_RATE_250K:
        PIOS_NRF24L01_Write1Reg(dev, PIOS_NRF24L01_REG_RF_SETUP, PIOS_NRF24L01_RF_SETUP_250K);
        break;
    case PIOS_NRF24L01_RATE_1M:
        PIOS_NRF24L01_Write1Reg(dev, PIOS_NRF24L01_REG_RF_SETUP, PIOS_NRF24L01_RF_SETUP_1M);
        break;
    case PIOS_NRF24L01_RATE_2M:
        PIOS_NRF24L01_Write1Reg(dev, PIOS_NRF24L01_REG_RF_SETUP, PIOS_NRF24L01_RF_SETUP_2M);
        break;
    default:
        // do nothing
        break;
    }
}

/**
 * Set the channel for the NRF24L01 modem.
 *
 * @param[in] nrf24l01_id  The device ID.
 * @param[in] channel  The channel
 */
void PIOS_NRF24L01_SetChannel(uint32_t nrf24l01_id, uint8_t channel)
{
    struct pios_nrf24l01_dev *dev = (struct pios_nrf24l01_dev *)nrf24l01_id;

    if (channel < 126) {
        PIOS_NRF24L01_Write1Reg(dev, PIOS_NRF24L01_REG_RF_CH, channel);
    }
}

/**
 * Set the address for the NRF24L01 modem.
 *
 * @param[in] nrf24l01_id  The device ID.
 * @param[in] pipe  The pipe.
 * @param[in] address  The 5 byte address.
 */
void PIOS_NRF24L01_SetAddress(uint32_t nrf24l01_id, uint8_t pipe, const uint8_t *address)
{
    struct pios_nrf24l01_dev *dev = (struct pios_nrf24l01_dev *)nrf24l01_id;
    uint8_t len = 5;

    if (pipe >= 6) {
        return;
    }
    if (pipe > 1) {
        len = 1;
    }

    PIOS_NRF24L01_WriteReg(dev, PIOS_NRF24L01_REG_RX_ADDR_P0 + pipe, address, len);
}

/**
 * Receive a packet on the NRF24L01 modem.
 *
 * @param[in] nrf24l01_id  The device ID.
 * @param[out] buffer  The output buffer to write into
 * @param[in] len  The length of the output buffer
 * @return The number of bytes read (or 0 of none/error)
 */
uint8_t PIOS_NRF24L01_ReceivePacket(uint32_t nrf24l01_id, uint8_t *buffer, uint8_t len, portTickType max_delay)
{
    struct pios_nrf24l01_dev *dev = (struct pios_nrf24l01_dev *)nrf24l01_id;

    // Enable receive.
    PIOS_NRF24L01_ChipEnable(dev);

    xSemaphoreTake(dev->data_ready, max_delay);

    // Turn on the Rx LED.
    PIOS_LED_On(PIOS_LED_RX);

    // Disable receive.
    PIOS_NRF24L01_ChipDisable(dev);

    // Fetch all the data from the FIFO.
    uint8_t datalen = 0;
    while ((PIOS_NRF24L01_Read1Reg(dev, PIOS_NRF24L01_REG_FIFO_STATUS) & 0x01) == 0) {
        datalen = PIOS_NRF24L01_RxLength(dev);

        // Drop packets that are too long.
        if (datalen > len) {
            PIOS_NRF24L01_FlushRx(dev);
            break;
        }

        // Fetch the data
        if (PIOS_NRF24L01_ReadRX(dev, buffer, datalen) != datalen) {
            datalen = 0;
            PIOS_NRF24L01_FlushRx(dev);
            break;
        }
    }

    // Turn off the Rx LED.
    PIOS_LED_Off(PIOS_LED_RX);

    // Clear the interruptions flags
    PIOS_NRF24L01_Write1Reg(dev, PIOS_NRF24L01_REG_STATUS, 0x70);

    return datalen;
}

/**
 * Send a packet from the NRF24L01 modem.
 *
 * @param[in] nrf24l01_id  The device ID.
 * @param[in] buffer  The output buffer to write into
 * @param[in] len  The length of the send buffer
 * @return The number of bytes send (or 0 of none/error)
 */
uint8_t PIOS_NRF24L01_SendPacket(uint32_t nrf24l01_id, uint8_t *buffer, uint8_t len)
{
    struct pios_nrf24l01_dev *dev = (struct pios_nrf24l01_dev *)nrf24l01_id;

    // Turn on the Tx LED.
    PIOS_LED_On(PIOS_LED_TX);

    // Disable receive.
    PIOS_NRF24L01_ChipDisable(dev);

    // Set the chip select.
    PIOS_NRF24L01_ChipSelect(dev);

    // Send the read command with the address
    PIOS_NRF24L01_SendByte(dev, PIOS_NRF24L01_CMD_W_ACK_PAYLOAD(0));

    // Send the bytes.
    for (uint8_t i = 0; i < len; ++i) {
        PIOS_NRF24L01_SendByte(dev, buffer[i]);
    }

    // Set the chip select.
    PIOS_NRF24L01_ChipDeselect(dev);

    return len;
}

/**
 * Assert the Chip Enable (CE) line.
 *
 * @param[in] dev  The device pointer.
 */
static void PIOS_NRF24L01_Task(void *parameters)
{
    struct pios_nrf24l01_dev *dev = (struct pios_nrf24l01_dev *)parameters;

    while (1) {
        // Read a packet from the radio device.
        int8_t len = PIOS_NRF24L01_ReceivePacket((uint32_t)dev, dev->rx_buffer, 32, portMAX_DELAY);
        if (len == 0) {
            PIOS_NRF24L01_ReadStatus(dev);
        }
    }
}

/**
 * Assert the Chip Enable (CE) line.
 *
 * @param[in] dev  The device pointer.
 */
static void PIOS_NRF24L01_ChipEnable(struct pios_nrf24l01_dev *dev)
{
    PIOS_GPIO_On(dev->ce_gpio, 0);
}

/**
 * Deassert the Chip Enable (CE) line.
 *
 * @param[in] dev  The device pointer.
 */
static void PIOS_NRF24L01_ChipDisable(struct pios_nrf24l01_dev *dev)
{
    PIOS_GPIO_Off(dev->ce_gpio, 0);
}

/**
 * Assert the Chip Select (CS) line.
 *
 * @param[in] dev  The device pointer.
 */
static void PIOS_NRF24L01_ChipSelect(struct pios_nrf24l01_dev *dev)
{
    PIOS_SPI_RC_PinSet(dev->spi_id, 0, 0);
}

/**
 * Deassert the Chip Select (CS) line.
 *
 * @param[in] dev  The device pointer.
 */
static void PIOS_NRF24L01_ChipDeselect(struct pios_nrf24l01_dev *dev)
{
    PIOS_SPI_RC_PinSet(dev->spi_id, 0, 1);
}

/**
 * Send a byte over the SPI bus.
 *
 * @param[in] dev  The device pointer.
 * @param[in] byte  The byte to send
 * @return  The byte to received
 */
static uint8_t PIOS_NRF24L01_SendByte(__attribute__((unused)) struct pios_nrf24l01_dev *dev, uint8_t byte)
{
    /* Loop while DR register in not emplty */
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET) {
        ;
    }

    /* Send byte through the SPI1 peripheral */
    SPI_I2S_SendData(SPI2, byte);

    /* Wait to receive a byte */
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET) {
        ;
    }

    /* Return the byte read from the SPI bus */
    return SPI_I2S_ReceiveData(SPI2);
    // return PIOS_SPI_TransferByte(dev->spi_id, byte);
}

/**
 * Send a byte over the SPI bus.
 *
 * @param[in] dev  The device pointer.
 * @return  The byte to received
 */
static uint8_t PIOS_NRF24L01_ReceiveByte(struct pios_nrf24l01_dev *dev)
{
    return PIOS_NRF24L01_SendByte(dev, PIOS_NRF24L01_DUMMY_BYTE);
}

/**
 * Read len (up to 5) bytes from a register.
 *
 * @param[in] dev  The device pointer.
 * @param[in] address  The register address.
 * @param[out] buffer  The ouput buffer.
 * @param[in] len  The number of bytes to read.
 * @return 0 on success
 */
static int32_t PIOS_NRF24L01_ReadReg(struct pios_nrf24l01_dev *dev, uint8_t address, uint8_t *buffer, int len)
{
    PIOS_NRF24L01_ChipSelect(dev);

    // uint8_t write_buf[6] = { PIOS_NRF24L01_CMD_R_REG | (address & 0x1F), 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    // uint8_t read_buf[6];
    // int32_t status = PIOS_SPI_TransferBlock(dev->spi_id, write_buf, read_buf, len + 1, NULL);
    // if (status == 0) {
    // memcpy(buffer, read_buf, len);
    // }
    uint8_t status = PIOS_NRF24L01_SendByte(dev, PIOS_NRF24L01_CMD_R_REG | (address & 0x1f));
    for (uint8_t i = 0; i < len; ++i) {
        buffer[i] = PIOS_NRF24L01_ReceiveByte(dev);
    }

    PIOS_NRF24L01_ChipDeselect(dev);

    return status;
}


/**
 * Write len (up to 5) bytes to a register.
 *
 * @param[in] dev  The device pointer.
 * @param[in] address  The register address.
 * @param[in] buffer  The ouput buffer.
 * @param[in] len  The number of bytes to read.
 * @return 0 on success
 */
static int32_t PIOS_NRF24L01_WriteReg(struct pios_nrf24l01_dev *dev, uint8_t address, const uint8_t *buffer, int len)
{
    PIOS_NRF24L01_ChipSelect(dev);

    // uint8_t write_buf[6] = { PIOS_NRF24L01_CMD_W_REG | (address & 0x1F), 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    // memcpy(write_buf + 1, buffer, len);
    // int32_t status = PIOS_SPI_TransferBlock(dev->spi_id, write_buf, NULL, len + 1, NULL);
    uint8_t status = PIOS_NRF24L01_SendByte(dev, PIOS_NRF24L01_CMD_W_REG | (address & 0x1f));
    for (uint8_t i = 0; i < len; ++i) {
        PIOS_NRF24L01_SendByte(dev, buffer[i]);
    }

    PIOS_NRF24L01_ChipDeselect(dev);

    return status;
}

/**
 * Read 1 byte from a register.
 *
 * @param[in] dev  The device pointer.
 * @param[in] address  The register address.
 * @return The value.
 */
static uint8_t PIOS_NRF24L01_Read1Reg(struct pios_nrf24l01_dev *dev, uint8_t address)
{
    uint8_t byte;

    PIOS_NRF24L01_ReadReg(dev, address, &byte, 1);
    return byte;
}

/**
 * Write 1 byte to a register.
 *
 * @param[in] dev  The device pointer.
 * @param[in] address  The register address.
 * @param[in] buffer  The ouput buffer.
 * @param[in] len  The number of bytes to read.
 * @return 0 on success
 */
static int32_t PIOS_NRF24L01_Write1Reg(struct pios_nrf24l01_dev *dev, uint8_t address, uint8_t byte)
{
    return PIOS_NRF24L01_WriteReg(dev, address, &byte, 1);
}

/**
 * Read the radio status.
 *
 * @param[in] dev  The device pointer.
 * @return The status byte.
 */
static uint8_t PIOS_NRF24L01_ReadStatus(struct pios_nrf24l01_dev *dev)
{
    PIOS_NRF24L01_ChipSelect(dev);
    uint8_t status = PIOS_NRF24L01_SendByte(dev, PIOS_NRF24L01_CMD_NOP);
    PIOS_NRF24L01_ChipDeselect(dev);
    return status;
}


/**
 * Flush the Rx buffer
 *
 * @param[in] dev  The device pointer.
 * @return The status returned
 */
static uint8_t PIOS_NRF24L01_FlushRx(struct pios_nrf24l01_dev *dev)
{
    PIOS_NRF24L01_ChipSelect(dev);
    uint8_t status = PIOS_NRF24L01_SendByte(dev, PIOS_NRF24L01_CMD_FLUSH_RX);
    PIOS_NRF24L01_ChipDeselect(dev);
    return status;
}

/**
 * Flush the Tx buffer
 *
 * @param[in] dev  The device pointer.
 * @return The status returned
 */
static uint8_t PIOS_NRF24L01_FlushTx(struct pios_nrf24l01_dev *dev)
{
    PIOS_NRF24L01_ChipSelect(dev);
    uint8_t status = PIOS_NRF24L01_SendByte(dev, PIOS_NRF24L01_CMD_FLUSH_TX);
    PIOS_NRF24L01_ChipDeselect(dev);
    return status;
}

/**
 * Read the nmber of bytes in a payload on a pipe.
 *
 * @param[in] dev  The device pointer.
 * @param[in] pipe  The pipe.
 * @return The length is returned
 */
static uint8_t PIOS_NRF24L01_RxLength(struct pios_nrf24l01_dev *dev)
{
    PIOS_NRF24L01_ChipSelect(dev);
    PIOS_NRF24L01_SendByte(dev, PIOS_NRF24L01_CMD_RX_PL_WID);
    uint8_t len = PIOS_NRF24L01_ReceiveByte(dev);
    PIOS_NRF24L01_ChipDeselect(dev);
    return len;
}

/**
 * Read an Rx packet
 *
 * @param[in] dev  The device pointer.
 * @param[out] buffer The buffer to read into
 * @param[in] len  The number of bytes to read.
 * @return The number of bytes read.
 */
static uint8_t PIOS_NRF24L01_ReadRX(struct pios_nrf24l01_dev *dev, uint8_t *buffer, uint8_t len)
{
    PIOS_NRF24L01_ChipSelect(dev);

    /* Send the read command with the address */
    PIOS_NRF24L01_SendByte(dev, PIOS_NRF24L01_CMD_R_RX_PAYLOAD);

    /* Read LEN bytes */
    for (uint8_t i = 0; i < len; i++) {
        buffer[i] = PIOS_NRF24L01_ReceiveByte(dev);
    }

    PIOS_NRF24L01_ChipDeselect(dev);

    return len;
}

#endif /* PIOS_INCLUDE_NRF24L01 */
