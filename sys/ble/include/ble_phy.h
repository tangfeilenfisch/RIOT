/*
 * Copyright (C) 2014 Johann Fischer <j.fischer_at_fh-bingen_de>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 *
 * ble phy defines and functions.
 */

#ifndef RIOT_BLE_PHY_H
#define RIOT_BLE_PHY_H

#include <stdint.h>
#include <stdbool.h>

/* mapping of rf channels to data and advertising channels */
enum le_channel_idx {
	dch0,
	dch1,
	dch2,
	dch3,
	dch4,
	dch5,
	dch6,
	dch7,
	dch8,
	dch9,
	dch10,
	dch11,
	dch12,
	dch13,
	dch14,
	dch15,
	dch16,
	dch17,
	dch18,
	dch19,
	dch20,
	dch21,
	dch22,
	dch23,
	dch24,
	dch25,
	dch26,
	dch27,
	dch28,
	dch29,
	dch30,
	dch31,
	dch32,
	dch33,
	dch34,
	dch35,
	dch36,
	ach37,
	ach38,
	ach39,
};

#define PHY_CU_ALL				0
#define PHY_CU_T				1
#define PHY_CU_CHMAP				2

struct ll_connection_udata {
	uint32_t aa;
	uint8_t crc_init[3];
	uint8_t win_size;
	uint16_t win_offset;
	uint16_t interval;
	uint16_t latency;
	uint16_t timeout;
	uint8_t ch_map[5];
	uint8_t hop_sca;
} __attribute__((packed));

void phy_reset_fifo(void);
struct ble_pdu* phy_get_next_tx_buf(void);
void phy_set_pending_tx_buf(void);
struct ble_pdu* phy_get_rx_buf(uint8_t pos);
int phy_free_pending_rx_buf(void);
int phy_rx_data_pending(void);

void phy_rng_get_eight(uint8_t *rnd_buf);
uint8_t phy_get_txpower(void);
uint8_t phy_get_max_txpower(void);
int phy_remove_dev_from_white_list(uint32_t b, uint16_t p);
int phy_add_dev_to_white_list(uint32_t b, uint16_t p, uint8_t random);
void phy_clear_white_list(void);
uint8_t phy_read_white_list_size_cmd(void);
void phy_connection_data_update(struct ll_connection_udata *con_data,
				uint16_t instant, uint8_t type);
void phy_prepare_for_adv_state(struct ble_pdu *advpdu,
				struct ble_pdu *scanrpdu,
				uint16_t adv_interval);
int phy_prepare_for_connection(void);
void phy_termitate_connection(void);
void nrf51phy_setup(int pid);

#endif
