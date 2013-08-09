/**
 ******************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup   PIOS_BQ24075 Power Management Functions
 * @brief PIOS interface for for the TI BQ24075 Power Management IC
 * @{
 *
 * @file       pios_bq24075.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2013.
 * @brief      Implements a driver the the bq24075 power management unit
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

#ifdef PIOS_INCLUDE_BQ24075

#include <pios_bq24075_priv.h>
#include <pios_gpio.h>

/* Structure definitons */
struct pios_bq24075_dev {
    uint32_t gpio_id;
};

/* Forward declarations */

/**
 * Initialize the BQ24075 device.
 *
 * @param[in] bq24075_id  A pointer to where to store the device id.
 * @param[in] gpio_id  The id for the GPIO devices.
 * @return 0 on success
 */
int32_t PIOS_BQ24075_Init(uint32_t *bq24075_id, uint32_t gpio_id)
{
    struct pios_bq24075_dev *dev = (struct pios_bq24075_dev *)pvPortMalloc(sizeof(struct pios_bq24075_dev));
    *bq24075_id = (uint32_t)dev;

    dev->gpio_id = gpio_id;
    return 0;
}

/**
 * Set the state of the charger.
 *
 * @param[in] bq24075_id  The device id.
 * @param[in] chg_state  The charge state.
 */
void PIOS_BQ24075_SetChargeState(uint32_t bq24075_id, enum pios_bq24075_charge_state chg_state)
{
    struct pios_bq24075_dev *dev = (struct pios_bq24075_dev *)bq24075_id;

    switch (chg_state) {
    case PIOS_BQ24075_CHARGE_100MA:
        PIOS_GPIO_Off(dev->gpio_id, PIOS_GPIO_PM_EN1);
        PIOS_GPIO_Off(dev->gpio_id, PIOS_GPIO_PM_EN2);
        break;
    case PIOS_BQ24075_CHARGE_500MA:
        PIOS_GPIO_On(dev->gpio_id, PIOS_GPIO_PM_EN1);
        PIOS_GPIO_Off(dev->gpio_id, PIOS_GPIO_PM_EN2);
        break;
    case PIOS_BQ24075_CHARGE_MAX:
        PIOS_GPIO_Off(dev->gpio_id, PIOS_GPIO_PM_EN1);
        PIOS_GPIO_On(dev->gpio_id, PIOS_GPIO_PM_EN2);
        break;
    default:
        // Do nothing.
        break;
    }
}

/**
 * Get the charging status.
 *
 * @param[in] bq24075_id  The device id.
 * @return  The charging state.
 */
enum pios_bq24075_charging_state PIOS_BQ24075_GetChargingState(uint32_t bq24075_id)
{
    struct pios_bq24075_dev *dev = (struct pios_bq24075_dev *)bq24075_id;
    bool is_charging = PIOS_GPIO_State(dev->gpio_id, PIOS_GPIO_PM_CHG);
    bool is_pgood = PIOS_GPIO_State(dev->gpio_id, PIOS_GPIO_PM_PGOOD);

    if (is_pgood && !is_charging) {
        return PIOS_BQ24075_CHARGED;
    } else if (is_pgood && is_charging) {
        return PIOS_BQ24075_CHARGING;
    } else if (!is_pgood && !is_charging) {
        return PIOS_BQ24075_LOW_POWER;
    } else {
        return PIOS_BQ24075_BATTERY;
    }
}

#endif /* PIOS_INCLUDE_BQ24075 */
