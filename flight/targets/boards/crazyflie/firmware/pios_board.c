/**
 *****************************************************************************
 * @file       pios_board.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
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

#include "inc/openpilot.h"
#include <pios_board_info.h>
#include <uavobjectsinit.h>
#include <hwsettings.h>
#include <manualcontrolsettings.h>
#include <gcsreceiver.h>
#include <taskinfo.h>

/*
 * Pull in the board-specific static HW definitions.
 * Including .c files is a bit ugly but this allows all of
 * the HW definitions to be const and static to limit their
 * scope.
 *
 * NOTE: THIS IS THE ONLY PLACE THAT SHOULD EVER INCLUDE THIS FILE
 */
#include "../board_hw_defs.c"

/* One slot per selectable receiver group.
 *  eg. PWM, PPM, GCS, DSMMAINPORT, DSMFLEXIPORT, SBUS
 * NOTE: No slot in this map for NONE.
 */
uint32_t pios_rcvr_group_map[MANUALCONTROLSETTINGS_CHANNELGROUPS_NONE];

#define PIOS_COM_TELEM_RF_RX_BUF_LEN     32
#define PIOS_COM_TELEM_RF_TX_BUF_LEN     12

#define PIOS_COM_TELEM_USB_RX_BUF_LEN    65
#define PIOS_COM_TELEM_USB_TX_BUF_LEN    65

#if defined(PIOS_INCLUDE_DEBUG_CONSOLE)
#define PIOS_COM_DEBUGCONSOLE_TX_BUF_LEN 40
uint32_t pios_com_debug_id;
#endif /* PIOS_INCLUDE_DEBUG_CONSOLE */

uint32_t pios_com_telem_rf_id;
uint32_t pios_com_telem_usb_id;

uintptr_t pios_uavo_settings_fs_id;

uintptr_t pios_user_fs_id = 0;

/**
 * PIOS_Board_Init()
 * initializes all the core subsystems on this specific hardware
 * called from System/openpilot.c
 */
void PIOS_Board_Init(void)
{
    /* Delay system */
    PIOS_DELAY_Init();

    const struct pios_board_info *bdinfo = &pios_board_info_blob;

#if defined(PIOS_INCLUDE_LED)
    const struct pios_led_cfg *led_cfg   = PIOS_BOARD_HW_DEFS_GetLedCfg(bdinfo->board_rev);
    PIOS_Assert(led_cfg);
    PIOS_LED_Init(led_cfg);
#endif /* PIOS_INCLUDE_LED */

#ifdef PIOS_INCLUDE_FLASH_LOGFS_SETTINGS
    uintptr_t flash_id;
    PIOS_Flash_Internal_Init(&flash_id, &flash_internal_cfg);
    PIOS_FLASHFS_Logfs_Init(&pios_uavo_settings_fs_id, &flashfs_internal_cfg, &pios_internal_flash_driver, flash_id);
#endif /* PIOS_INCLUDE_FLASH_LOGFS_SETTINGS */

    /* Initialize UAVObject libraries */
    EventDispatcherInitialize();
    UAVObjInitialize();

#if defined(PIOS_INCLUDE_RTC)
    /* Initialize the real-time clock and its associated tick */
    PIOS_RTC_Init(&pios_rtc_main_cfg);
#endif
    PIOS_IAP_Init();
    HwSettingsInitialize();

#if defined(PIOS_INCLUDE_WDG)
    /* Initialize watchdog as early as possible to catch faults during init */
    PIOS_WDG_Init();
#endif

    /* Initialize the alarms library */
    AlarmsInitialize();

    /* Check for repeated boot failures */
    uint16_t boot_count = PIOS_IAP_ReadBootCount();
    if (boot_count < 3) {
        PIOS_IAP_WriteBootCount(++boot_count);
        AlarmsClear(SYSTEMALARMS_ALARM_BOOTFAULT);
    } else {
        /* Too many failed boot attempts, force hwsettings to defaults */
        HwSettingsSetDefaults(HwSettingsHandle(), 0);
        AlarmsSet(SYSTEMALARMS_ALARM_BOOTFAULT, SYSTEMALARMS_ALARM_CRITICAL);
    }

    /* Initialize the task monitor */
    if (PIOS_TASK_MONITOR_Initialize(TASKINFO_RUNNING_NUMELEM)) {
        PIOS_Assert(0);
    }

    /* Initialize the delayed callback library */
    CallbackSchedulerInitialize();

    /* Set up pulse timers */
    PIOS_TIM_InitClock(&tim_1_cfg);
    PIOS_TIM_InitClock(&tim_2_cfg);
    PIOS_TIM_InitClock(&tim_3_cfg);
    PIOS_TIM_InitClock(&tim_4_cfg);

#if defined(PIOS_INCLUDE_COM)
#if defined(PIOS_INCLUDE_USB)
#if defined(PIOS_INCLUDE_USB_HID)
    /* Initialize board specific USB data */
    PIOS_USB_BOARD_DATA_Init();
    if (PIOS_USB_DESC_HID_ONLY_Init()) {
        PIOS_Assert(0);
    }

    uint32_t pios_usb_id;
    PIOS_USB_Init(&pios_usb_id, &pios_usb_main_cfg);

    /* Configure the usb HID port */
    {
        uint32_t pios_usb_hid_id;
        if (PIOS_USB_HID_Init(&pios_usb_hid_id, &pios_usb_hid_cfg, pios_usb_id)) {
            PIOS_Assert(0);
        }
        uint8_t *rx_buffer = (uint8_t *)pvPortMalloc(PIOS_COM_TELEM_USB_RX_BUF_LEN);
        uint8_t *tx_buffer = (uint8_t *)pvPortMalloc(PIOS_COM_TELEM_USB_TX_BUF_LEN);
        PIOS_Assert(rx_buffer);
        PIOS_Assert(tx_buffer);
        if (PIOS_COM_Init(&pios_com_telem_usb_id, &pios_usb_hid_com_driver, pios_usb_hid_id,
                          rx_buffer, PIOS_COM_TELEM_USB_RX_BUF_LEN,
                          tx_buffer, PIOS_COM_TELEM_USB_TX_BUF_LEN)) {
            PIOS_Assert(0);
        }
    }
#endif /* PIOS_INCLUDE_USB_HID */
#endif /* PIOS_INCLUDE_USB */

#if defined(PIOS_INCLUDE_TELEMETRY_RF)
    {
        uint32_t pios_usart_generic_id;
        if (PIOS_USART_Init(&pios_usart_generic_id, &pios_usart_generic_flexi_cfg)) {
            PIOS_Assert(0);
        }
        uint8_t *rx_buffer = (uint8_t *)pvPortMalloc(PIOS_COM_TELEM_RF_RX_BUF_LEN);
        uint8_t *tx_buffer = (uint8_t *)pvPortMalloc(PIOS_COM_TELEM_RF_TX_BUF_LEN);
        PIOS_Assert(rx_buffer);
        PIOS_Assert(tx_buffer);
        if (PIOS_COM_Init(&pios_com_telem_rf_id, &pios_usart_com_driver, pios_usart_generic_id,
                          rx_buffer, PIOS_COM_TELEM_RF_RX_BUF_LEN,
                          tx_buffer, PIOS_COM_TELEM_RF_TX_BUF_LEN)) {
            PIOS_Assert(0);
        }
    }
#endif /* PIOS_INCLUDE_TELEMETRY_RF */
#endif /* PIOS_INCLUDE_COM */

#if defined(PIOS_INCLUDE_GCSRCVR)
#include <pios_rcvr_priv.h>
    GCSReceiverInitialize();
    uint32_t pios_gcsrcvr_id;
    PIOS_GCSRCVR_Init(&pios_gcsrcvr_id);
    uint32_t pios_gcsrcvr_rcvr_id;
    if (PIOS_RCVR_Init(&pios_gcsrcvr_rcvr_id, &pios_gcsrcvr_rcvr_driver, pios_gcsrcvr_id)) {
        PIOS_Assert(0);
    }
    pios_rcvr_group_map[MANUALCONTROLSETTINGS_CHANNELGROUPS_GCS] = pios_gcsrcvr_rcvr_id;
#endif /* PIOS_INCLUDE_GCSRCVR */

    /* Initialize the motor PWM controls */
    PIOS_Servo_Init(&pios_servo_cfg);

#if defined(PIOS_INCLUDE_I2C)
    /* Configure the sensor I2C bus. */
    if (PIOS_I2C_Init(&pios_i2c_sensors_id, &pios_i2c_sensors_cfg)) {
        PIOS_Assert(0);
    }
#endif /* PIOS_INCLUDE_I2C */

#if defined(PIOS_INCLUDE_MPU6000)
    /* Configure the MPU6000 */
    extern const struct pios_mpu6000_cfg pios_mpu6000_cfg;
    if (PIOS_MPU6000_Init(pios_i2c_sensors_id, 0x69, &pios_mpu6000_cfg) < 0) {
        return;
    }
    PIOS_MPU6000_CONFIG_Configure();
    PIOS_Assert(PIOS_MPU6000_Test() == 0);
#endif /* PIOS_INCLUDE_MPU6000 */

    /* Enable the USB port */
    PIOS_LED_Off(PIOS_LED_USB_ENABLE);

    /* Make sure we have at least one telemetry link configured or else fail initialization */
    PIOS_Assert(pios_com_telem_rf_id || pios_com_telem_usb_id);
}

/**
 * @}
 */
