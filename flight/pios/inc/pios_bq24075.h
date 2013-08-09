/**
 ******************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup   PIOS_BQ24075 Radio Functions
 * @brief PIOS interface for for the BQ24075 power management unit.
 * @{
 *
 * @file       pios_bq24075.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2013.
 * @brief      Implements a driver the the nfr24l01 power management unit.
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

#ifndef PIOS_BQ24075_H
#define PIOS_BQ24075_H

#include <pios_bq24075.h>

/* Global Types */
enum pios_bq24075_charge_state {
    PIOS_BQ24075_CHARGE_100MA,
    PIOS_BQ24075_CHARGE_500MA,
    PIOS_BQ24075_CHARGE_MAX,
};

enum pios_bq24075_charging_state {
    PIOS_BQ24075_CHARGED,
    PIOS_BQ24075_CHARGING,
    PIOS_BQ24075_LOW_POWER,
    PIOS_BQ24075_BATTERY,
};

/* Public Functions */
extern void PIOS_BQ24075_SetChargeState(uint32_t bq24075_id, enum pios_bq24075_charge_state chg_state);
extern enum pios_bq24075_charging_state PIOS_BQ24075_GetChargingState(uint32_t bq24075_id);

#endif /* PIOS_BQ24075_H */

/**
 * @}
 * @}
 */
