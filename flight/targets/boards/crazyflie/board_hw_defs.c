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
static const struct pios_led pios_leds[] = {
    [PIOS_LED_HEARTBEAT] =  {
        .pin                =              {
            .gpio = GPIOB,
            .init =              {
                .GPIO_Pin   = GPIO_Pin_4,
                .GPIO_Mode  = GPIO_Mode_Out_PP,
                .GPIO_Speed = GPIO_Speed_50MHz,
            },
        },
        .remap              = GPIO_Remap_SWJ_JTAGDisable,
    },
    [PIOS_LED_USB] =        {
        .pin                =              {
            .gpio = GPIOB,
            .init =              {
                .GPIO_Pin   = GPIO_Pin_5,
                .GPIO_Mode  = GPIO_Mode_Out_PP,
                .GPIO_Speed = GPIO_Speed_50MHz,
            },
        },
        .remap              = GPIO_PartialRemap_TIM3,
    },
    [PIOS_LED_USB_ENABLE] = {
        .pin                =              {
            .gpio = GPIOA,
            .init =              {
                .GPIO_Pin   = GPIO_Pin_0,
                .GPIO_Mode  = GPIO_Mode_Out_PP,
                .GPIO_Speed = GPIO_Speed_50MHz,
            },
        },
    },
};

static const struct pios_led_cfg pios_led_cfg = {
    .leds     = pios_leds,
    .num_leds = NELEMENTS(pios_leds),
};

const struct pios_led_cfg *PIOS_BOARD_HW_DEFS_GetLedCfg(__attribute__((unused)) uint32_t board_revision)
{
    return &pios_led_cfg;
}

#endif /* PIOS_INCLUDE_LED */

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

/*
   static const struct pios_mpu6050_cfg pios_mpu6050_cfg = {
    .exti_cfg = &pios_exti_mpu6050_cfg,
    .Fifo_store = PIOS_MPU6050_FIFO_TEMP_OUT | PIOS_MPU6050_FIFO_GYRO_X_OUT | PIOS_MPU6050_FIFO_GYRO_Y_OUT | PIOS_MPU6050_FIFO_GYRO_Z_OUT,
    // Clock at 8 khz, downsampled by 8 for 1khz
    .Smpl_rate_div = 11,
    .interrupt_cfg = PIOS_MPU6050_INT_CLR_ANYRD,
    .interrupt_en = PIOS_MPU6050_INTEN_DATA_RDY,
    .User_ctl = PIOS_MPU6050_USERCTL_FIFO_EN ,
    .Pwr_mgmt_clk = PIOS_MPU6050_PWRMGMT_PLL_X_CLK,
    .accel_range = PIOS_MPU6050_ACCEL_8G,
    .gyro_range = PIOS_MPU6050_SCALE_500_DEG,
    .filter = PIOS_MPU6050_LOWPASS_256_HZ,
   };
 */

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
