/**
 ******************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup   PIOS_NFR24L01 Radio Functions
 * @brief PIOS interface for for the NFR24L01 radio
 * @{
 *
 * @file       pios_nfr24l01.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2013.
 * @brief      Implements a driver the the nfr24l01 radio
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

#ifndef PIOS_NRF24L01_H
#define PIOS_NRF24L01_H

/* Global Types */
enum pios_nrf24l01_datarate {
    PIOS_NRF24L01_RATE_250K,
    PIOS_NRF24L01_RATE_1M,
    PIOS_NRF24L01_RATE_2M,
};

/* Public Functions */
extern void PIOS_NRF24L01_SetDatarate(uint32_t nrf24l01_id, enum pios_nrf24l01_datarate datarate);
extern void PIOS_NRF24L01_SetChannel(uint32_t nrf24l01_id, uint8_t channel);
extern void PIOS_NRF24L01_SetAddress(uint32_t nrf24l01_id, uint8_t pipe, const uint8_t *address);
extern uint8_t PIOS_NRF24L01_ReceivePacket(uint32_t nrf24l01_id, uint8_t *buffer, uint8_t len, portTickType max_delay);
extern uint8_t PIOS_NRF24L01_SendPacket(uint32_t nrf24l01_id, uint8_t *buffer, uint8_t len);

#endif /* PIOS_NRF24L01_H */

/**
 * @}
 * @}
 */
