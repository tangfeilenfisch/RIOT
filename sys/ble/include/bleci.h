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
 * controller interface funtctions and defines.
 */

#ifndef RIOT_BLECI_H
#define RIOT_BLECI_H

#include <stdint.h>

/* default connection data values */
#define APT_IMAX_DEFAULT			800
#define APT_IMIN_DEFAULT			240
#define APT_RANDOM_ADDR				1
#define APT_PRIVAT_ADDR				0
#define APT_CHMAP_ALL				0x7
#define APT_ADV_IND				0x00
#define APT_ADV_DIRECT_IND_HC			0x01
#define APT_ADV_SCAN_IND			0x02
#define APT_ADV_NONCONN_IND			0x03
#define APT_ADV_DERECT_IND_LC			0x04
#define APT_FP_WHITEL_OFF			0x00
#define APT_FP_WHITEL_SCAN			0x01
#define APT_FP_WHITEL_CON			0x02
#define APT_FP_WHITEL_ALL			0x03

#define LE_FEATURES_LE_ENC			(1 << 0)
#define LE_FEATURES_CPRP			(1 << 1)
#define LE_FEATURES_ERI				(1 << 2)
#define LE_FEATURES_SIFE			(1 << 3)
#define LE_FEATURES_LE_PING			(1 << 4)
#define FEATURES_BREDR_NS			(1 << 37)
#define FEATURES_LE_S				(1 << 38)

/* supported states, slave role only */
#define LE_STATES_SCAN_ADV			(1 << 1)
#define LE_STATES_CON_ADV			(1 << 2)
#define LE_STATES_HC_ADV			(1 << 3)
#define LE_STATES_S_CON				(1 << 7)
#define LE_STATES_LC_ADV			(1 << 29)

struct advparameters {
	uint16_t i_min;
	uint16_t i_max;
	uint8_t adv_type;
	uint8_t dev_addr_type;
	uint8_t dir_addr_type;
	uint32_t dir_addr_b;
	uint16_t dir_addr_p;
	uint8_t adv_ch_map;
	uint8_t adv_policy;
};

struct conparameters {
	uint16_t handle;
	uint16_t i_min;
	uint16_t i_max;
	uint16_t latency;
	uint16_t sv_timeout;
	uint16_t ce_min;
	uint16_t ce_max;
};

struct advparameters* ll_get_adv_parameters(void);
uint8_t* ll_clear_adv_data(void);
int ll_gen_adv_data_flag(uint8_t *data, uint8_t i);
int ll_gen_adv_data_role(uint8_t *data, uint8_t i);
int ll_gen_adv_data_txpwr(uint8_t *data, uint8_t i);

int le_set_random_address_cmd(uint8_t *rnd_buf);
int le_read_adv_channel_tx_power_cmd(int8_t *dmb);
int read_transmit_power_level_cmd(uint8_t param, int8_t *dbm);
int le_read_white_list_size_cmd(uint8_t *size);
int le_clear_white_list_cmd(void);
int le_remove_dev_from_white_list_cmd(uint32_t b, uint16_t p);
int le_add_dev_to_white_list_cmd(uint8_t param, uint32_t b, uint16_t p);
int le_set_adv_enable_cmd(uint8_t param);
int le_set_adv_parameters_cmd(struct advparameters *param);
int le_set_adv_data_cmd(uint8_t param, uint8_t *data);
int le_set_scan_response_data_cmd(uint8_t param, uint8_t *data);
int le_read_remote_used_features_cmd(uint16_t handle);
int le_con_update_cmd(struct conparameters *param);
int le_read_channel_map_cmd(uint16_t handle, uint64_t *ch_map);
int le_read_supported_states_cmd(uint64_t *states);
int le_read_buf_size_cmd(uint16_t *p_length, uint8_t *p_number);
int le_rand_cmd(uint8_t *rnd_buf);
int discon_cmd(uint16_t handle, uint8_t reason);
int read_bd_addr_cmd(uint32_t *b, uint16_t *p);
int le_read_local_supported_features_cmd(uint64_t *le_features);
int read_local_supported_features_cmd(uint64_t *features);
int read_local_supported_commands_cmd(uint64_t *commands);
int reset_cmd(void);
int read_rssi_cmd(uint16_t handle, int8_t *dbm);


/*
//TODO: commands
int le_encrypt_cmd(void);
int le_long_term_key_req_reply_cmd(void);
int le_long_term_key_req_negative_reply_cmd(void);
int le_start_encryption_cmd(void);

int le_transmitter_test_cmd(void);
int le_receiver_test_cmd(void);
int le_test_end_cmd(void);
int read_local_version_information_cmd(void);
int read_remote_version_information_cmd(void);

//TODO: events
int set_event_mask_cmd(void);
int le_set_event_mask_cmd(void);
int cmd_complete_ev(void);
int cmd_status_ev(void);
int discon_complete_ev(void);
int le_adv_report_ev(void);
int le_con_complete_ev(void);
int le_con_update_complete_ev(void);
int le_read_remote_used_features_complete_ev(void);
int number_of_completed_packets_ev(void);
int read_remote_version_information_complete_ev(void);
int encryption_change_ev(void);
int encryption_key_refresh_complete_ev(void);
int le_long_term_key_req_ev(void);

// unsupported commands and events, master role
struct scanparameters {
	uint8_t scan_type;
	uint16_t interval;
	uint16_t window;
	uint8_t dev_addr_type;
	uint8_t scan_policy;
};

int le_set_scan_enable_cmd(uint8_t param);
int le_set_scan_parameters_cmd(struct scanparameters *param);
int le_create_con_cmd(void);
int le_create_con_cancel_cmd(void);
int le_set_host_channel_classification_cmd(void);
*/

#endif
