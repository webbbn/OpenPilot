/**
 ******************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup   PIOS_NFR24L01 Radio Functions
 * @brief PIOS interface for for the NFR24L01 radio
 * @{
 *
 * @file       pios_nfr24l01_priv.h
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

#ifndef PIOS_NFR24L01_PRIV_H
#define PIOS_NFR24L01_PRIV_H

#include <pios_nrf24l01.h>

/* Global Types */
struct pios_nrf24l01_cfg {
    const struct pios_gpio *gpio_cfg; /* Pointer to the GPIO for the CE line */
    const struct pios_exti_cfg *exti_cfg; /* Pointer to the EXTI configuration */
};

/* Public Functions */
extern int32_t PIOS_NRF24L01_Init(uint32_t *nrf24l01_id, uint32_t spi_id, const struct pios_nrf24l01_cfg *cfg);
extern bool PIOS_NRF24L01_IRQHandler(uint32_t nrf24l01_id);

#endif /* PIOS_NRF24L01_PRIV_H */

/**
 * @}
 * @}
 */
