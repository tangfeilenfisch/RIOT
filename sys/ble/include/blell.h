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
 * ble link layer defines and functions.
 */

#ifndef RIOT_BLELL_H
#define RIOT_BLELL_H

#include <stdint.h>
#include <stdbool.h>
#include "ble_phy.h"

/* advertising PDU types are sent in the advertising state */
#define PDUH_ADV_TYPE_MSK		0xf
#define PDUH_ADV_IND			0x0
#define PDUH_ADV_DIRECT_IND		0x1
#define PDUH_ADV_NONCONN_IND		0x2
#define PDUH_ADV_SCAN_IND		0x6
/* scanning PDU PDUH_SCAN_REQ are received in advertising state */
#define PDUH_SCAN_REQ			0x3
/* scanning PDU PDUH_SCAN_RSP are sent in advertising state */
#define PDUH_SCAN_RSP			0x4
/* initiating PDU type is received in the advertising state */
#define PDUH_CONNECT_REQ		0x5

#define PDUH_ADV_RXADD			(1 << 7)
#define PDUH_ADV_TXADD			(1 << 6)

#define LL_SCA_MASK			0x7
#define LL_HOP_POS			0x3
#define MASTER_SCA_20PPM		0
#define MASTER_SCA_30PPM		1
#define MASTER_SCA_50PPM		2
#define MASTER_SCA_75PPM		3
#define MASTER_SCA_100PPM		4
#define MASTER_SCA_150PPM		5
#define MASTER_SCA_250PPM		6
#define MASTER_SCA_500PPM		7

/* the PDU range = [2 .. 39] octets */
#define PDU_MAX_LENGTH			39
#define ADV_DATA_LENGTH			31

struct ble_pdu {
	uint8_t type;
	uint8_t length;
	uint8_t data[37];
} __attribute__((packed));

struct le_adv_pdu {
	uint8_t type;
	uint8_t length;
	uint32_t addr_base;		/* bit 31..0 */
	uint16_t addr_prefix;		/* bit 47..32 */
	uint8_t data[ADV_DATA_LENGTH];
} __attribute__((packed));

struct le_adv_direct_pdu {
	uint8_t type;
	uint8_t length;
	uint32_t addr_base;		/* bit 31..0 */
	uint16_t addr_prefix;		/* bit 47..32 */
	uint32_t init_addr_base;	/* bit 31..0 */
	uint16_t init_addr_prefix;	/* bit 47..32 */
} __attribute__((packed));

struct le_scan_req_pdu {
	uint8_t type;
	uint8_t length;
	uint32_t scan_base;		/* bit 31..0 */
	uint16_t scan_prefix;		/* bit 47..32 */
	uint32_t addr_base;		/* bit 31..0 */
	uint16_t addr_prefix;		/* bit 47..32 */
} __attribute__((packed));

struct le_scan_rsp_pdu {
	uint8_t type;
	uint8_t length;
	uint32_t addr_base;		/* bit 31..0 */
	uint16_t addr_prefix;		/* bit 47..32 */
	uint8_t data[ADV_DATA_LENGTH];
} __attribute__((packed));

struct le_connect_req_pdu {
	uint8_t type;
	uint8_t length;
	uint32_t init_addr_base;	/* bit 31..0 */
	uint16_t init_addr_prefix;	/* bit 47..32 */
	uint32_t addr_base;		/* bit 31..0 */
	uint16_t addr_prefix;		/* bit 47..32 */
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


#define PDUH_DCH_LLID_MSK	0x3
#define PDUH_LLID_CONT		1
#define PDUH_LLID_START		2
#define PDUH_LLID_CTRL		3
#define PDUH_DCH_NESN		(1 << 2)
#define PDUH_DCH_SN		(1 << 3)
#define PDUH_DCH_MD		(1 << 4)
#define PDUH_DCH_FLAG_MSK	(PDUH_DCH_MD | PDUH_DCH_SN | PDUH_DCH_NESN)

/* link manager */

#define LL_CONNECTION_UPDATE_REQ		0x00
#define LL_CHANNEL_MAP_REQ			0x01
#define LL_TERMINATE_IND			0x02
#define LL_ENC_REQ				0x03
#define LL_ENC_RSP				0x04
#define LL_START_ENC_REQ			0x05
#define LL_START_ENC_RSP			0x06
#define LL_UNKNOWN_RSP			        0x07
#define LL_FEATURE_REQ			        0x08
#define LL_FEATURE_RSP			        0x09
#define LL_PAUSE_ENC_REQ			0x0A
#define LL_PAUSE_ENC_RSP			0x0B
#define LL_VERSION_IND			        0x0C
#define LL_REJECT_IND			        0x0D
#define LL_SLAVE_FEATURE_REQ		        0x0E
#define LL_CONNECTION_PARAM_REQ		        0x0F
#define LL_CONNECTION_PARAM_RSP		        0x10
#define LL_REJECT_IND_EXT			0x11
#define LL_PING_REQ				0x12
#define LL_PING_RSP				0x13

struct cd_update_req {
	uint8_t type;
	uint8_t length;
	uint8_t opcode;
	uint8_t	win_size;
	uint16_t win_offset;
	uint16_t interval;
	uint16_t latency;
	uint16_t timeout;
	uint16_t instant;
} __attribute__((packed));

struct cd_channel_map_req {
	uint8_t type;
	uint8_t length;
	uint8_t opcode;
	uint8_t ch_map[5];
	uint16_t instant;
} __attribute__((packed));

struct cd_terminate_ind {
	uint8_t type;
	uint8_t length;
	uint8_t opcode;
	uint8_t error;
} __attribute__((packed));

struct cd_enc_req {
	uint8_t type;
	uint8_t length;
	uint8_t opcode;
	uint8_t rand[8];
	uint8_t ediv[2];
	uint8_t skd_m[8];
	uint8_t iv_m[4];
} __attribute__((packed));

struct cd_enc_rsp {
	uint8_t type;
	uint8_t length;
	uint8_t opcode;
	uint8_t skd_s[8];
	uint8_t iv_s[4];
} __attribute__((packed));

struct cd_start_enc_req {
	uint8_t type;
	uint8_t length;
	uint8_t opcode;
} __attribute__((packed));

struct cd_start_enc_rsp {
	uint8_t type;
	uint8_t length;
	uint8_t opcode;
} __attribute__((packed));

struct cd_unknown_rsp {
	uint8_t type;
	uint8_t length;
	uint8_t opcode;
	uint8_t u_type;
} __attribute__((packed));

struct cd_feature_req {
	uint8_t type;
	uint8_t length;
	uint8_t opcode;
	uint8_t feature_set[8];
} __attribute__((packed));

struct cd_feature_rsp {
	uint8_t type;
	uint8_t length;
	uint8_t opcode;
	uint8_t feature_set[8];
} __attribute__((packed));

struct cd_pause_enc_req {
	uint8_t type;
	uint8_t length;
	uint8_t opcode;
} __attribute__((packed));

struct cd_pause_enc_rsp {
	uint8_t type;
	uint8_t length;
	uint8_t opcode;
} __attribute__((packed));

struct cd_version_ind {
	uint8_t type;
	uint8_t length;
	uint8_t opcode;
	uint8_t vers_nr;
	uint16_t comp_id;
	uint16_t subvers_nr;
} __attribute__((packed));

struct cd_reject_ind {
	uint8_t type;
	uint8_t length;
	uint8_t opcode;
	uint8_t error;
} __attribute__((packed));

struct cd_slave_feature_req {
	uint8_t type;
	uint8_t length;
	uint8_t opcode;
	uint8_t feature_set[8];
} __attribute__((packed));

struct cd_con_param_req {
	uint8_t type;
	uint8_t length;
	uint8_t opcode;
	uint16_t interval_min;
	uint16_t interval_max;
	uint16_t latency;
	uint16_t timeout;
	uint8_t prfr_period;
	uint16_t rcec;
	uint16_t offset[6];
} __attribute__((packed));

struct cd_con_param_rsp {
	uint8_t type;
	uint8_t length;
	uint8_t opcode;
	uint16_t interval_min;
	uint16_t interval_max;
	uint16_t latency;
	uint16_t timeout;
	uint8_t prfr_period;
	uint16_t rcec;
	uint16_t offset[6];
} __attribute__((packed));

struct cd_reject_ind_ext {
	uint8_t type;
	uint8_t length;
	uint8_t opcode;
	uint8_t r_opcode;
	uint8_t error;
} __attribute__((packed));

struct cd_ping_req {
	uint8_t type;
	uint8_t length;
	uint8_t opcode;
} __attribute__((packed));

struct cd_ping_rsp {
	uint8_t type;
	uint8_t length;
	uint8_t opcode;
} __attribute__((packed));

struct cd_dummy {
	uint8_t type;
	uint8_t length;
	uint8_t opcode;
	uint8_t data[36];
};

struct bmode_fpdu {
	uint8_t type;
	uint8_t length;
	uint16_t f_length;
	uint16_t ch_id;
	uint8_t payload[29];
} __attribute__((packed));

enum linklayer_state {
	link_standby		= 0,
	link_advertising	= 1,
	link_connection		= 2,
};

extern int link_manager_pid;

/* Message types for ble-phy <-> link layer manager communication */
enum blell_message {
	BLE_PHY_RCV_PKT,
	BLE_PHY_CON_ESTBD,
	BLE_PHY_CON_LOST,
	BLE_PHY_CON_TERM,
	BLE_CI_ENABLE_ADV,
	BLE_CI_DISABLE_ADV,
	BLE_L2CAP_PKT_PENDING,
	BLE_L2CAP_PKT_HANDLED,
	BLE_CI_RND_ADR,
	BLE_CI_ADV_PARAM,
	BLE_CI_SCANR_PARAM,
	BLE_CI_TERM_CON,
	BLE_ENOBUFFER,
};

void ble_linklayer_init(void);
int ble_linklayer_start(void);
uint8_t ble_linklayer_register(int pid);

#endif
