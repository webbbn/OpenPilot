/**
 ******************************************************************************
 * @file       board_hw_defs.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2012.
 * @author     PhoenixPilot, http://github.com/PhoenixPilot, Copyright (C) 2012
 * @addtogroup OpenPilotSystem OpenPilot System
 * @{
 * @addtogroup OpenPilotCore OpenPilot Core
 * @{
 * @brief Defines board specific static initializers for hardware for the CopterControl board.
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

#if defined(PIOS_INCLUDE_LED)

#include <pios_led_priv.h>
static const struct pios_gpio pios_leds[] = {
    [PIOS_LED_HEARTBEAT] = {
        .pin                =             {
            .gpio = GPIOB,
            .init =             {
                .GPIO_Pin   = GPIO_Pin_4,
                .GPIO_Mode  = GPIO_Mode_Out_PP,
                .GPIO_Speed = GPIO_Speed_50MHz,
            },
        },
        .remap              = GPIO_Remap_SWJ_JTAGDisable,
        .active_low         = true
    },
    [PIOS_LED_ALARM] =     {
        .pin                =             {
            .gpio = GPIOB,
            .init =             {
                .GPIO_Pin   = GPIO_Pin_5,
                .GPIO_Mode  = GPIO_Mode_Out_PP,
                .GPIO_Speed = GPIO_Speed_50MHz,
            },
        },
        .remap              = GPIO_PartialRemap_TIM3,
        .active_low         = true
    },
};

static const struct pios_gpio_cfg pios_led_cfg = {
    .gpios     = pios_leds,
    .num_gpios = NELEMENTS(pios_leds),
};

const struct pios_gpio_cfg *PIOS_BOARD_HW_DEFS_GetLedCfg(__attribute__((unused)) uint32_t board_revision)
{
    return &pios_led_cfg;
}

#endif /* PIOS_INCLUDE_LED */

#if defined(PIOS_INCLUDE_GPIO)

#include <pios_gpio_priv.h>
static const struct pios_gpio pios_gpios[] = {
    [PIOS_GPIO_USB_ENABLE] = {
        .pin                =               {
            .gpio = GPIOA,
            .init =               {
                .GPIO_Pin   = GPIO_Pin_0,
                .GPIO_Mode  = GPIO_Mode_Out_PP,
                .GPIO_Speed = GPIO_Speed_2MHz,
            },
        },
        .active_low = false
    },
#ifdef PIOS_INCLUDE_BQ24075
    [PIOS_GPIO_PM_PGOOD] = {
        .pin                =               {
            .gpio = GPIOC,
            .init =               {
                .GPIO_Pin   = GPIO_Pin_15,
                .GPIO_Mode  = GPIO_Mode_IN_FLOATING,
                .GPIO_Speed = GPIO_Speed_2MHz,
            },
        },
        .active_low = true
    },
    [PIOS_GPIO_PM_CHG] = {
        .pin                =               {
            .gpio = GPIOB,
            .init =               {
                .GPIO_Pin   = GPIO_Pin_2,
                .GPIO_Mode  = GPIO_Mode_IPU,
                .GPIO_Speed = GPIO_Speed_2MHz,
            },
        },
        .active_low = true
    },
    [PIOS_GPIO_PM_EN1] = {
        .pin                =               {
            .gpio = GPIOC,
            .init =               {
                .GPIO_Pin   = GPIO_Pin_13,
                .GPIO_Mode  = GPIO_Mode_Out_PP,
                .GPIO_Speed = GPIO_Speed_10MHz,
            },
        },
        .active_low = false
    },
    [PIOS_GPIO_PM_EN2] = {
        .pin                =               {
            .gpio = GPIOA,
            .init =               {
                .GPIO_Pin   = GPIO_Pin_2,
                .GPIO_Mode  = GPIO_Mode_Out_PP,
                .GPIO_Speed = GPIO_Speed_10MHz,
            },
        },
        .active_low = false
    },
    [PIOS_GPIO_PM_SYSOFF] = {
        .pin                =               {
            .gpio = GPIOA,
            .init =               {
                .GPIO_Pin   = GPIO_Pin_1,
                .GPIO_Mode  = GPIO_Mode_Out_PP,
                .GPIO_Speed = GPIO_Speed_10MHz,
            },
        },
        .active_low = false
    },
#endif /* PIOS_INCLUDE_BQ24075 */
};

static const struct pios_gpio_cfg pios_gpios_cfg = {
    .gpios     = pios_gpios,
    .num_gpios = NELEMENTS(pios_gpios),
};

#endif /* PIOS_INCLUDE_LED */

#if defined(PIOS_INCLUDE_SPI)

#include <pios_spi_priv.h>


/* NFR24L01 SPI Interface */
void PIOS_SPI_nrf24l01_irq_handler(void);
void DMA1_Channel4_IRQHandler() __attribute__((alias("PIOS_SPI_nrf24l01_irq_handler")));
void DMA1_Channel5_IRQHandler() __attribute__((alias("PIOS_SPI_nrf24l01_irq_handler")));
static const struct pios_spi_cfg pios_spi_nrf24l01_cfg = {
    .regs = SPI2,
    .init = {
        .SPI_Mode              = SPI_Mode_Master,
        .SPI_Direction         = SPI_Direction_2Lines_FullDuplex,
        .SPI_DataSize          = SPI_DataSize_8b,
        .SPI_NSS                                   = SPI_NSS_Soft,
        .SPI_FirstBit          = SPI_FirstBit_MSB,
        .SPI_CRCPolynomial     = 7,
        .SPI_CPOL              = SPI_CPOL_Low,
        .SPI_CPHA              = SPI_CPHA_1Edge,
        .SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8,
    },
    .use_crc = false,
    .dma     = {
        .ahb_clk = RCC_AHBPeriph_DMA1,

        .irq     = {
            .flags = (DMA1_FLAG_TC4 | DMA1_FLAG_TE4 | DMA1_FLAG_HT4 | DMA1_FLAG_GL4),
            .init  = {
                .NVIC_IRQChannel    = DMA1_Channel4_IRQn,
                .NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_HIGH,
                .NVIC_IRQChannelSubPriority        = 0,
                .NVIC_IRQChannelCmd = ENABLE,
            },
        },

        .rx                                        = {
            .channel = DMA1_Channel4,
            .init    = {
                .DMA_PeripheralBaseAddr            = (uint32_t)&(SPI2->DR),
                .DMA_DIR                           = DMA_DIR_PeripheralSRC,
                .DMA_PeripheralInc      = DMA_PeripheralInc_Disable,
                .DMA_MemoryInc          = DMA_MemoryInc_Enable,
                .DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte,
                .DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte,
                .DMA_Mode     = DMA_Mode_Normal,
                .DMA_Priority = DMA_Priority_High,
                .DMA_M2M                           = DMA_M2M_Disable,
            },
        },
        .tx                                        = {
            .channel = DMA1_Channel5,
            .init    = {
                .DMA_PeripheralBaseAddr            = (uint32_t)&(SPI2->DR),
                .DMA_DIR                           = DMA_DIR_PeripheralDST,
                .DMA_PeripheralInc      = DMA_PeripheralInc_Disable,
                .DMA_MemoryInc          = DMA_MemoryInc_Enable,
                .DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte,
                .DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte,
                .DMA_Mode     = DMA_Mode_Normal,
                .DMA_Priority = DMA_Priority_High,
                .DMA_M2M                           = DMA_M2M_Disable,
            },
        },
    },
    .sclk                                          = {
        .gpio = GPIOB,
        .init = {
            .GPIO_Pin   = GPIO_Pin_13,
            .GPIO_Speed = GPIO_Speed_10MHz,
            .GPIO_Mode  = GPIO_Mode_AF_PP,
        },
    },
    .miso                                          = {
        .gpio = GPIOB,
        .init = {
            .GPIO_Pin   = GPIO_Pin_14,
            .GPIO_Speed = GPIO_Speed_10MHz,
            .GPIO_Mode  = GPIO_Mode_IN_FLOATING,
        },
    },
    .mosi                                          = {
        .gpio = GPIOB,
        .init = {
            .GPIO_Pin   = GPIO_Pin_15,
            .GPIO_Speed = GPIO_Speed_10MHz,
            .GPIO_Mode  = GPIO_Mode_AF_PP,
        },
    },
    .slave_count                                   = 1,
    .ssel                                          = {
        {
            .gpio = GPIOB,
            .init = {
                .GPIO_Pin   = GPIO_Pin_12,
                .GPIO_Speed = GPIO_Speed_50MHz,
                .GPIO_Mode  = GPIO_Mode_Out_PP,
            }
        },
    },
};

static uint32_t pios_spi_nrf24l01_id;
void PIOS_SPI_nrf24l01_irq_handler(void)
{
    /* Call into the generic code to handle the IRQ for this specific device */
    PIOS_SPI_IRQ_Handler(pios_spi_nrf24l01_id);
}

#endif /* PIOS_INCLUDE_SPI */

#if defined(PIOS_INCLUDE_NRF24L01)
#include <pios_nrf24l01_priv.h>

static uint32_t pios_nrf24l01_dev;
static bool PIOS_nrf24l01_irq_handler(void)
{
    /* Call into the generic code to handle the IRQ for this specific device */
    return PIOS_NRF24L01_IRQHandler(pios_nrf24l01_dev);
}
static const struct pios_gpio pios_gpio_nrf24l01_ce_cfg = {
    .pin                = {
        .gpio = GPIOA,
        .init = {
            .GPIO_Pin   = GPIO_Pin_10,
            .GPIO_Mode  = GPIO_Mode_Out_PP,
            .GPIO_Speed = GPIO_Speed_50MHz,
        },
    },
};

static const struct pios_exti_cfg pios_exti_nrf24l01_cfg __exti_config = {
    .vector = PIOS_nrf24l01_irq_handler,
    .line   = EXTI_Line9,
    .pin    = {
        .gpio = GPIOA,
        .init = {
            .GPIO_Pin   = GPIO_Pin_9,
            .GPIO_Speed = GPIO_Speed_50MHz,
            .GPIO_Mode  = GPIO_Mode_IN_FLOATING,
        },
    },
    .irq                                       = {
        .init                                  = {
            .NVIC_IRQChannel    = EXTI9_5_IRQn,
            .NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_LOW,
            .NVIC_IRQChannelSubPriority        = 0,
            .NVIC_IRQChannelCmd = ENABLE,
        },
    },
    .exti                                      = {
        .init                                  = {
            .EXTI_Line    = EXTI_Line9,
            .EXTI_Mode    = EXTI_Mode_Interrupt,
            .EXTI_Trigger = EXTI_Trigger_Falling,
            .EXTI_LineCmd = ENABLE,
        },
    },
};
static const struct pios_nrf24l01_cfg pios_nrf24l01_cfg = {
    .gpio_cfg = &pios_gpio_nrf24l01_ce_cfg,
    .exti_cfg = &pios_exti_nrf24l01_cfg
};
#endif /* PIOS_INCLUDE_NRF24L01 */

#if defined(PIOS_INCLUDE_BQ24075)
#include <pios_bq24075_priv.h>
static const struct pios_bq24075_cfg pios_bq24075_cfg = {
    .gpios = pios_gpios,
};
#endif /* PIOS_INCLUDE_BQ24075 */

#if defined(PIOS_INCLUDE_FLASH)
#include "pios_flashfs_logfs_priv.h"
#include "pios_flash_internal_priv.h"

static const struct pios_flash_internal_cfg flash_internal_cfg = {};

static const struct flashfs_logfs_cfg flashfs_internal_cfg = {
    .fs_magic      = 0x99abcfef,
    .total_fs_size = EE_BANK_SIZE, /* 2K bytes (2x1KB sectors) */
    .arena_size    = 0x00002000, /* 4 * slot size = 1K bytes = 1 sector */
    .slot_size     = 0x00000100, /* 256 bytes */

    .start_offset  = EE_BANK_BASE, /* start after the bootloader */
    .sector_size   = 0x00000400, /* 1K bytes */
    .page_size     = 0x00000400, /* 1K bytes */
};

#include "pios_flash.h"

#endif /* PIOS_INCLUDE_FLASH */

#include "pios_tim_priv.h"

static const TIM_TimeBaseInitTypeDef tim_1_2_3_4_time_base = {
    .TIM_Prescaler         = (PIOS_MASTER_CLOCK / 1000000) - 1,
    .TIM_ClockDivision     = TIM_CKD_DIV1,
    .TIM_CounterMode       = TIM_CounterMode_Up,
    .TIM_Period            = ((1000000 / PIOS_SERVO_UPDATE_HZ) - 1),
    .TIM_RepetitionCounter = 0x0000,
};

static const struct pios_tim_clock_cfg tim_1_cfg = {
    .timer = TIM1,
    .time_base_init                            = &tim_1_2_3_4_time_base,
    .irq   = {
        .init                                  = {
            .NVIC_IRQChannel    = TIM1_CC_IRQn,
            .NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_MID,
            .NVIC_IRQChannelSubPriority        = 0,
            .NVIC_IRQChannelCmd = ENABLE,
        },
    },
};

static const struct pios_tim_clock_cfg tim_2_cfg = {
    .timer = TIM2,
    .time_base_init                            = &tim_1_2_3_4_time_base,
    .irq   = {
        .init                                  = {
            .NVIC_IRQChannel    = TIM2_IRQn,
            .NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_MID,
            .NVIC_IRQChannelSubPriority        = 0,
            .NVIC_IRQChannelCmd = ENABLE,
        },
    },
};

static const struct pios_tim_clock_cfg tim_3_cfg = {
    .timer = TIM3,
    .time_base_init                            = &tim_1_2_3_4_time_base,
    .irq   = {
        .init                                  = {
            .NVIC_IRQChannel    = TIM3_IRQn,
            .NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_MID,
            .NVIC_IRQChannelSubPriority        = 0,
            .NVIC_IRQChannelCmd = ENABLE,
        },
    },
};

static const struct pios_tim_clock_cfg tim_4_cfg = {
    .timer = TIM4,
    .time_base_init                            = &tim_1_2_3_4_time_base,
    .irq   = {
        .init                                  = {
            .NVIC_IRQChannel    = TIM4_IRQn,
            .NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_MID,
            .NVIC_IRQChannelSubPriority        = 0,
            .NVIC_IRQChannelCmd = ENABLE,
        },
    },
};

#if defined(PIOS_INCLUDE_USART)

#include "pios_usart_priv.h"

static const struct pios_usart_cfg pios_usart_generic_flexi_cfg = {
    .regs = USART3,
    .init = {
        .USART_BaudRate   = 57600,
        .USART_WordLength = USART_WordLength_8b,
        .USART_Parity     = USART_Parity_No,
        .USART_StopBits   = USART_StopBits_1,
        .USART_HardwareFlowControl             = USART_HardwareFlowControl_None,
        .USART_Mode                            = USART_Mode_Rx | USART_Mode_Tx,
    },
    .irq                                       = {
        .init                                  = {
            .NVIC_IRQChannel    = USART3_IRQn,
            .NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_MID,
            .NVIC_IRQChannelSubPriority        = 0,
            .NVIC_IRQChannelCmd = ENABLE,
        },
    },
    .rx                                        = {
        .gpio = GPIOB,
        .init = {
            .GPIO_Pin   = GPIO_Pin_11,
            .GPIO_Speed = GPIO_Speed_2MHz,
            .GPIO_Mode  = GPIO_Mode_IPU,
        },
    },
    .tx                                        = {
        .gpio = GPIOB,
        .init = {
            .GPIO_Pin   = GPIO_Pin_10,
            .GPIO_Speed = GPIO_Speed_2MHz,
            .GPIO_Mode  = GPIO_Mode_AF_PP,
        },
    },
};

#endif /* PIOS_INCLUDE_USART */

#if defined(PIOS_INCLUDE_COM)

#include "pios_com_priv.h"

#endif /* PIOS_INCLUDE_COM */

#if defined(PIOS_INCLUDE_RTC)
/*
 * Realtime Clock (RTC)
 */
#include <pios_rtc_priv.h>

void PIOS_RTC_IRQ_Handler(void);
void RTC_IRQHandler() __attribute__((alias("PIOS_RTC_IRQ_Handler")));
static const struct pios_rtc_cfg pios_rtc_main_cfg = {
    .clksrc    = RCC_RTCCLKSource_HSE_Div128,
    .prescaler = 100,
    .irq                                       = {
        .init                                  = {
            .NVIC_IRQChannel    = RTC_IRQn,
            .NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_MID,
            .NVIC_IRQChannelSubPriority        = 0,
            .NVIC_IRQChannelCmd = ENABLE,
        },
    },
};

void PIOS_RTC_IRQ_Handler(void)
{
    PIOS_RTC_irq_handler();
}

#endif /* if defined(PIOS_INCLUDE_RTC) */

#if defined(PIOS_INCLUDE_SERVO) && defined(PIOS_INCLUDE_TIM)
/*
 * Servo outputs
 */
#include <pios_servo_priv.h>

static const struct pios_tim_channel pios_tim_servoport_all_pins[] = {
    {
        .timer = TIM3,
        .timer_chan = TIM_Channel_4,
        .pin   = {
            .gpio = GPIOB,
            .init = {
                .GPIO_Pin   = GPIO_Pin_1,
                .GPIO_Mode  = GPIO_Mode_AF_PP,
                .GPIO_Speed = GPIO_Speed_2MHz,
            },
        },
    },
    {
        .timer = TIM3,
        .timer_chan = TIM_Channel_3,
        .pin   = {
            .gpio = GPIOB,
            .init = {
                .GPIO_Pin   = GPIO_Pin_0,
                .GPIO_Mode  = GPIO_Mode_AF_PP,
                .GPIO_Speed = GPIO_Speed_2MHz,
            },
        },
    },
    {
        .timer = TIM4,
        .timer_chan = TIM_Channel_4,
        .pin   = {
            .gpio = GPIOB,
            .init = {
                .GPIO_Pin   = GPIO_Pin_9,
                .GPIO_Mode  = GPIO_Mode_AF_PP,
                .GPIO_Speed = GPIO_Speed_2MHz,
            },
        },
    },
    {
        .timer = TIM4,
        .timer_chan = TIM_Channel_3,
        .pin   = {
            .gpio = GPIOB,
            .init = {
                .GPIO_Pin   = GPIO_Pin_8,
                .GPIO_Mode  = GPIO_Mode_AF_PP,
                .GPIO_Speed = GPIO_Speed_2MHz,
            },
        },
    },
};

const struct pios_servo_cfg pios_servo_cfg = {
    .tim_oc_init          = {
        .TIM_OCMode       = TIM_OCMode_PWM1,
        .TIM_OutputState  = TIM_OutputState_Enable,
        .TIM_OutputNState = TIM_OutputNState_Disable,
        .TIM_Pulse        = PIOS_SERVOS_INITIAL_POSITION,
        .TIM_OCPolarity   = TIM_OCPolarity_High,
        .TIM_OCNPolarity  = TIM_OCPolarity_High,
        .TIM_OCIdleState  = TIM_OCIdleState_Reset,
        .TIM_OCNIdleState = TIM_OCNIdleState_Reset,
    },
    .channels     = pios_tim_servoport_all_pins,
    .num_channels = NELEMENTS(pios_tim_servoport_all_pins),
};

#endif /* PIOS_INCLUDE_SERVO && PIOS_INCLUDE_TIM */

#if defined(PIOS_INCLUDE_I2C)

#include <pios_i2c_priv.h>

/*
 * I2C Adapters
 */

void PIOS_I2C_sensors_ev_irq_handler(void);
void PIOS_I2C_sensors_er_irq_handler(void);
void I2C1_EV_IRQHandler() __attribute__((alias("PIOS_I2C_sensors_ev_irq_handler")));
void I2C1_ER_IRQHandler() __attribute__((alias("PIOS_I2C_sensors_er_irq_handler")));

static const struct pios_i2c_adapter_cfg pios_i2c_sensors_cfg = {
    .regs = I2C1,
    .init = {
        .I2C_Mode = I2C_Mode_I2C,
        .I2C_OwnAddress1                       = 0,
        .I2C_Ack  = I2C_Ack_Enable,
        .I2C_AcknowledgedAddress               = I2C_AcknowledgedAddress_7bit,
        .I2C_DutyCycle                         = I2C_DutyCycle_2,
        .I2C_ClockSpeed                        = 400000,                      /* bits/s */
    },
    .transfer_timeout_ms                       = 50,
    .scl                                       = {
        .gpio = GPIOB,
        .init = {
            .GPIO_Pin   = GPIO_Pin_6,
            .GPIO_Speed = GPIO_Speed_10MHz,
            .GPIO_Mode  = GPIO_Mode_AF_OD,
        },
    },
    .sda                                       = {
        .gpio = GPIOB,
        .init = {
            .GPIO_Pin   = GPIO_Pin_7,
            .GPIO_Speed = GPIO_Speed_10MHz,
            .GPIO_Mode  = GPIO_Mode_AF_OD,
        },
    },
    .event                                     = {
        .flags = 0,           /* FIXME: check this */
        .init  = {
            .NVIC_IRQChannel    = I2C1_EV_IRQn,
            .NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_HIGHEST,
            .NVIC_IRQChannelSubPriority        = 0,
            .NVIC_IRQChannelCmd = ENABLE,
        },
    },
    .error                                     = {
        .flags = 0,           /* FIXME: check this */
        .init  = {
            .NVIC_IRQChannel    = I2C1_ER_IRQn,
            .NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_HIGHEST,
            .NVIC_IRQChannelSubPriority        = 0,
            .NVIC_IRQChannelCmd = ENABLE,
        },
    },
};

uint32_t pios_i2c_sensors_id;
void PIOS_I2C_sensors_ev_irq_handler(void)
{
    /* Call into the generic code to handle the IRQ for this specific device */
    PIOS_I2C_EV_IRQ_Handler(pios_i2c_sensors_id);
}

void PIOS_I2C_sensors_er_irq_handler(void)
{
    /* Call into the generic code to handle the IRQ for this specific device */
    PIOS_I2C_ER_IRQ_Handler(pios_i2c_sensors_id);
}

#ifdef PIOS_INCLUDE_MPU6000
#include "pios_mpu6000.h"
#include "pios_mpu6000_config.h"
static const struct pios_exti_cfg pios_exti_mpu6000_cfg __exti_config = {
    .vector = PIOS_MPU6000_IRQHandler,
    .line   = EXTI_Line14,
    .pin    = {
        .gpio = GPIOC,
        .init = {
            .GPIO_Pin  = GPIO_Pin_14,
            .GPIO_Mode = GPIO_Mode_IN_FLOATING,
        },
    },
    .irq                                       = {
        .init                                  = {
            .NVIC_IRQChannel    = EXTI15_10_IRQn,
            .NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_HIGHEST,
            .NVIC_IRQChannelSubPriority        = 0,
            .NVIC_IRQChannelCmd = ENABLE,
        },
    },
    .exti                                      = {
        .init                                  = {
            .EXTI_Line    = EXTI_Line14,
            .EXTI_Mode    = EXTI_Mode_Interrupt,
            .EXTI_Trigger = EXTI_Trigger_Falling,
            .EXTI_LineCmd = ENABLE,
        },
    },
};

const struct pios_mpu6000_cfg pios_mpu6000_cfg = {
    .exti_cfg   = &pios_exti_mpu6000_cfg,
    .Fifo_store = PIOS_MPU6000_FIFO_TEMP_OUT | PIOS_MPU6000_FIFO_GYRO_X_OUT | PIOS_MPU6000_FIFO_GYRO_Y_OUT | PIOS_MPU6000_FIFO_GYRO_Z_OUT,
    // Clock at 8 khz, downsampled by 16 for 500 Hz
    .Smpl_rate_div_no_dlp = 15,
    // Clock at 1 khz, downsampled by 2 for 500 Hz
    .Smpl_rate_div_dlp    = 1,
    .interrupt_cfg = PIOS_MPU6000_INT_CLR_ANYRD,
    .interrupt_en  = PIOS_MPU6000_INTEN_DATA_RDY,
    .User_ctl             = PIOS_MPU6000_USERCTL_FIFO_EN,
    .Pwr_mgmt_clk  = PIOS_MPU6000_PWRMGMT_PLL_X_CLK,
    .accel_range   = PIOS_MPU6000_ACCEL_8G,
    .gyro_range    = PIOS_MPU6000_SCALE_2000_DEG,
    .filter               = PIOS_MPU6000_LOWPASS_256_HZ,
    .orientation   = PIOS_MPU6000_TOP_180DEG
};

#endif /* PIOS_INCLUDE_MPU6000 */

#endif /* PIOS_INCLUDE_I2C */

#if defined(PIOS_INCLUDE_GCSRCVR)
#include "pios_gcsrcvr_priv.h"
#endif /* PIOS_INCLUDE_GCSRCVR */

#if defined(PIOS_INCLUDE_USB)
#include "pios_usb_priv.h"

static const struct pios_usb_cfg pios_usb_main_cfg = {
    .irq                                       = {
        .init                                  = {
            .NVIC_IRQChannel    = USB_LP_CAN1_RX0_IRQn,
            .NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_LOW,
            .NVIC_IRQChannelSubPriority        = 0,
            .NVIC_IRQChannelCmd = ENABLE,
        },
    },
    .vsense                                    = {
        .gpio = GPIOC,
        .init = {
            .GPIO_Pin   = GPIO_Pin_15,
            .GPIO_Speed = GPIO_Speed_10MHz,
            .GPIO_Mode  = GPIO_Mode_AF_OD,
        },
    },
    .vsense_active_low                         = true
};

#include "pios_usb_board_data_priv.h"
#include "pios_usb_desc_hid_cdc_priv.h"
#include "pios_usb_desc_hid_only_priv.h"

#endif /* PIOS_INCLUDE_USB */

#if defined(PIOS_INCLUDE_COM_MSG)

#include <pios_com_msg_priv.h>

#endif /* PIOS_INCLUDE_COM_MSG */

#if defined(PIOS_INCLUDE_USB_HID)
#include <pios_usb_hid_priv.h>

const struct pios_usb_hid_cfg pios_usb_hid_cfg = {
    .data_if    = 2,
    .data_rx_ep = 1,
    .data_tx_ep = 1,
};

#endif /* PIOS_INCLUDE_USB_HID */

#if defined(PIOS_INCLUDE_USB_CDC)
#include <pios_usb_cdc_priv.h>

const struct pios_usb_cdc_cfg pios_usb_cdc_cfg = {
    .ctrl_if    = 0,
    .ctrl_tx_ep = 2,

    .data_if    = 1,
    .data_rx_ep = 3,
    .data_tx_ep = 3,
};
#endif /* PIOS_INCLUDE_USB_CDC */

/*
 * ADC system
 */
#if defined(PIOS_INCLUDE_ADC)
#include "pios_adc_priv.h"
extern void PIOS_ADC_handler(void);
void DMA1_Channel1_IRQHandler() __attribute__((alias("PIOS_ADC_handler")));
// Remap the ADC DMA handler to this one
static const struct pios_adc_cfg pios_adc_cfg = {
    .dma                                           = {
        .ahb_clk = RCC_AHBPeriph_DMA1,
        .irq     = {
            .flags = (DMA1_FLAG_TC1 | DMA1_FLAG_TE1 | DMA1_FLAG_HT1 | DMA1_FLAG_GL1),
            .init  = {
                .NVIC_IRQChannel    = DMA1_Channel1_IRQn,
                .NVIC_IRQChannelPreemptionPriority = PIOS_IRQ_PRIO_HIGH,
                .NVIC_IRQChannelSubPriority        = 0,
                .NVIC_IRQChannelCmd = ENABLE,
            },
        },
        .rx                                        = {
            .channel = DMA1_Channel1,
            .init    = {
                .DMA_PeripheralBaseAddr            = (uint32_t)&ADC1->DR,
                .DMA_DIR                           = DMA_DIR_PeripheralSRC,
                .DMA_PeripheralInc      = DMA_PeripheralInc_Disable,
                .DMA_MemoryInc          = DMA_MemoryInc_Enable,
                .DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word,
                .DMA_MemoryDataSize     = DMA_MemoryDataSize_Word,
                .DMA_Mode     = DMA_Mode_Circular,
                .DMA_Priority = DMA_Priority_High,
                .DMA_M2M                           = DMA_M2M_Disable,
            },
        }
    },
    .half_flag = DMA1_IT_HT1,
    .full_flag = DMA1_IT_TC1,
};

void PIOS_ADC_handler()
{
    PIOS_ADC_DMA_Handler();
}
#endif /* if defined(PIOS_INCLUDE_ADC) */
