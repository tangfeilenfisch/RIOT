/*
 * This file is part of the riot project.
 *
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
 * TODO: implement the necessary functions :-)
 */

#include <string.h>
#include "msg.h"
#include "thread.h"
#include "debug.h"
#include "blell.h"
#include "le_errors.h"
#include "ad_types.h"

#define LINK_LAYER_STACK_SIZE			512
char link_manager_stack [LINK_LAYER_STACK_SIZE] __attribute__ ((aligned (32)));
//char link_manager_stack[LINK_LAYER_STACK_SIZE];	/* hard_fault */

int link_manager_pid = -1; ///< the link manager thread's pid

#define BLELL_PAYLOAD_SIZE			27
#define BLELL_BUFFER_SIZE			48
uint8_t blell_buf[BLELL_BUFFER_SIZE * BLELL_PAYLOAD_SIZE];

#define BLELL_MSG_BUFFER_SIZE			64
msg_t msg_buffer[BLELL_MSG_BUFFER_SIZE];

static struct ll_connection_udata ble_connection_data;

struct ble_linklayer_data {
	bool init_addr_random;
	bool dev_addr_random;
	bool adv_enabled;
	bool whitel_enabled;
	enum linklayer_state ll_state;
	struct ble_pdu advpdu;
	struct ble_pdu scanrpdu;
	int ulayer_pid;
	/* -- */
	uint16_t init_addr_prefix;
	uint32_t init_addr_base;
	uint16_t dev_addr_prefix;
	uint32_t dev_addr_base;
};

static struct ble_linklayer_data blell_data = {
	.dev_addr_random = true,
	.ll_state = link_standby,
};

/*****************************************************************************/
/* link layer manager                                                        */
/*****************************************************************************/

int llcmd_con_update_req(struct cd_dummy *llcmd);
int llcmd_channel_map_req(struct cd_dummy *llcmd);
int llcmd_terminate_ind(struct cd_dummy *llcmd);
int llcmd_enc_req(struct cd_dummy *llcmd);
int llcmd_enc_rsp(struct cd_dummy *llcmd);
int llcmd_start_enc_req(struct cd_dummy *llcmd);
int llcmd_start_enc_rsp(struct cd_dummy *llcmd);
int llcmd_unknown_rsp(struct cd_dummy *llcmd);
int llcmd_feature_req(struct cd_dummy *llcmd);
int llcmd_feature_rsp(struct cd_dummy *llcmd);
int llcmd_pause_enc_req(struct cd_dummy *llcmd);
int llcmd_pause_enc_rsp(struct cd_dummy *llcmd);
int llcmd_version_ind(struct cd_dummy *llcmd);
int llcmd_reject_ind(struct cd_dummy *llcmd);
int llcmd_slave_feature_req(struct cd_dummy *llcmd);
int llcmd_con_param_req(struct cd_dummy *llcmd);
int llcmd_con_param_rsp(struct cd_dummy *llcmd);
int llcmd_reject_ind_ext(struct cd_dummy *llcmd);
int llcmd_ping_req(struct cd_dummy *llcmd);
int llcmd_ping_rsp(struct cd_dummy *llcmd);

/* functions pointer table */
int (*llcmd_func[])(struct cd_dummy *llcmd) = {
	llcmd_con_update_req,
	llcmd_channel_map_req,
	llcmd_terminate_ind,
	llcmd_enc_req,
	llcmd_enc_rsp,
	llcmd_start_enc_req,
	llcmd_start_enc_rsp,
	llcmd_unknown_rsp,
	llcmd_feature_req,
	llcmd_feature_rsp,
	llcmd_pause_enc_req,
	llcmd_pause_enc_rsp,
	llcmd_version_ind,
	llcmd_reject_ind,
	llcmd_slave_feature_req,
	llcmd_con_param_req,
	llcmd_con_param_rsp,
	llcmd_reject_ind_ext,
	llcmd_ping_req,
	llcmd_ping_rsp,
};

struct ll_control_protocol_data {
	uint8_t last_error;
	uint8_t last_rx_opcode;
	uint8_t last_tx_opcode;
	uint8_t reply_pending;
	uint8_t my_features[8];
	uint8_t peer_features[8];
};

static struct ll_control_protocol_data llcp_data = {
	.last_error = 0,
	.last_rx_opcode = 0,
	.last_tx_opcode = 0,
	.reply_pending = 0,
};


int llcmd_con_update_req(struct cd_dummy *llcmd)
{
	/* only master may send this pdu */
	struct cd_update_req *cmd = (struct cd_update_req*)llcmd;

	ble_connection_data.win_size = cmd->win_size;
	ble_connection_data.win_offset = cmd->win_offset;
	ble_connection_data.interval = cmd->interval;
	ble_connection_data.latency = cmd->latency;
	ble_connection_data.timeout = cmd->timeout;

	phy_connection_data_update(&ble_connection_data, cmd->instant,
					PHY_CU_T);
	/* TODO: ctr_data */
	return cmd->length;
}

int llcmd_channel_map_req(struct cd_dummy *llcmd)
{
	struct cd_channel_map_req *cmd = (struct cd_channel_map_req*)llcmd;

	ble_connection_data.ch_map[0] = cmd->ch_map[0];
	ble_connection_data.ch_map[1] = cmd->ch_map[1];
	ble_connection_data.ch_map[2] = cmd->ch_map[2];
	ble_connection_data.ch_map[3] = cmd->ch_map[3];
	ble_connection_data.ch_map[4] = cmd->ch_map[4];
	phy_connection_data_update(&ble_connection_data, cmd->instant,
					PHY_CU_CHMAP);
	/* TODO: ctr_data */
	return cmd->length;
}

int llcmd_terminate_ind(struct cd_dummy *llcmd)
{
	struct cd_terminate_ind *cmd = (struct cd_terminate_ind*)llcmd;

	llcp_data.last_error = cmd->error;
	phy_termitate_connection();
	return 0;
}

int llcmd_enc_req(struct cd_dummy *llcmd)
{
	struct cd_enc_req *cmd = (struct cd_enc_req*)llcmd;
	/* TODO: ctr_data */
	return cmd->length;
}

int llcmd_enc_rsp(struct cd_dummy *llcmd)
{
	struct cd_enc_rsp *cmd = (struct cd_enc_rsp*)llcmd;
	/* TODO: ctr_data */
	return cmd->length;
}

static int llrsp_enc_rsp(struct cd_dummy *llcmd)
{
	struct cd_enc_rsp *cmd = (struct cd_enc_rsp*)llcmd;

	cmd->type = PDUH_LLID_CTRL;
	cmd->length = sizeof(struct cd_enc_rsp) - 2;
	cmd->opcode = LL_ENC_RSP;
	/* TODO: ctr_data */
	return 0;
}

int llcmd_start_enc_req(struct cd_dummy *llcmd)
{
	struct cd_start_enc_req *cmd = (struct cd_start_enc_req*)llcmd;
	/* TODO: ctr_data */
	return cmd->length;
}

int llcmd_start_enc_rsp(struct cd_dummy *llcmd)
{
	struct cd_start_enc_rsp *cmd = (struct cd_start_enc_rsp*)llcmd;
	/* TODO: ctr_data */
	return cmd->length;
}

static int llrsp_start_enc_rsp(struct cd_dummy *llcmd)
{
	struct cd_start_enc_rsp *cmd = (struct cd_start_enc_rsp*)llcmd;

	cmd->type = PDUH_LLID_CTRL;
	cmd->length = sizeof(struct cd_start_enc_rsp) - 2;
	cmd->opcode = LL_START_ENC_RSP;
	/* TODO: ctr_data */
	return 0;
}

int llcmd_unknown_rsp(struct cd_dummy *llcmd)
{
	struct cd_unknown_rsp *cmd = (struct cd_unknown_rsp*)llcmd;
	/* TODO: ctr_data */
	return cmd->length;
}

static int llrsp_unknown_rsp(struct cd_dummy *llcmd)
{
	struct cd_unknown_rsp *cmd = (struct cd_unknown_rsp*)llcmd;

	cmd->type = PDUH_LLID_CTRL;
	cmd->length = sizeof(struct cd_unknown_rsp) - 2;
	cmd->opcode = LL_UNKNOWN_RSP;
	cmd->u_type = llcp_data.last_rx_opcode;
	return 0;
}

int llcmd_feature_req(struct cd_dummy *llcmd)
{
	struct cd_feature_req *cmd = (struct cd_feature_req*)llcmd;
	llcp_data.peer_features[0] = cmd->feature_set[0];
	llcp_data.peer_features[1] = cmd->feature_set[1];
	llcp_data.peer_features[2] = cmd->feature_set[2];
	llcp_data.peer_features[3] = cmd->feature_set[3];
	llcp_data.peer_features[4] = cmd->feature_set[4];
	llcp_data.peer_features[5] = cmd->feature_set[5];
	llcp_data.peer_features[6] = cmd->feature_set[6];
	llcp_data.peer_features[7] = cmd->feature_set[7];
	return 0;
}

int llcmd_slave_feature_req(struct cd_dummy *llcmd)
{
	struct cd_slave_feature_req *cmd = (struct cd_slave_feature_req*)llcmd;
	/* INFO: unnecessary for slave role */
	return cmd->length;
}

int llcmd_feature_rsp(struct cd_dummy *llcmd)
{
	struct cd_feature_rsp *cmd = (struct cd_feature_rsp*)llcmd;
	llcp_data.peer_features[0] = cmd->feature_set[0];
	llcp_data.peer_features[1] = cmd->feature_set[1];
	llcp_data.peer_features[2] = cmd->feature_set[2];
	llcp_data.peer_features[3] = cmd->feature_set[3];
	llcp_data.peer_features[4] = cmd->feature_set[4];
	llcp_data.peer_features[5] = cmd->feature_set[5];
	llcp_data.peer_features[6] = cmd->feature_set[6];
	llcp_data.peer_features[7] = cmd->feature_set[7];
	return 0;
}

static int llrsp_feature_rsp(struct cd_dummy *llcmd)
{
	struct cd_feature_rsp *cmd = (struct cd_feature_rsp*)llcmd;

	cmd->type = PDUH_LLID_CTRL;
	cmd->length = sizeof(struct cd_feature_rsp) - 2;
	cmd->opcode = LL_FEATURE_RSP;
	cmd->feature_set[0] = llcp_data.my_features[0];
	cmd->feature_set[1] = llcp_data.my_features[1];
	cmd->feature_set[2] = llcp_data.my_features[2];
	cmd->feature_set[3] = llcp_data.my_features[3];
	cmd->feature_set[4] = llcp_data.my_features[4];
	cmd->feature_set[5] = llcp_data.my_features[5];
	cmd->feature_set[6] = llcp_data.my_features[6];
	cmd->feature_set[7] = llcp_data.my_features[7];
	return 0;
}

int llcmd_pause_enc_req(struct cd_dummy *llcmd)
{
	/* pdu does not have any data */
	struct cd_pause_enc_req *cmd = (struct cd_pause_enc_req*)llcmd;
	return cmd->length;
}

int llcmd_pause_enc_rsp(struct cd_dummy *llcmd)
{
	/* pdu does not have any data */
	struct cd_pause_enc_rsp *cmd = (struct cd_pause_enc_rsp*)llcmd;
	return cmd->length;
}

static int llrsp_pause_enc_rsp(struct cd_dummy *llcmd)
{
	/* pdu does not have any data */
	struct cd_pause_enc_rsp *cmd = (struct cd_pause_enc_rsp*)llcmd;
	
	cmd->type = PDUH_LLID_CTRL;
	cmd->length = sizeof(struct cd_pause_enc_rsp) - 2;
	cmd->opcode = LL_PAUSE_ENC_RSP;
	return 0;
}

int llcmd_version_ind(struct cd_dummy *llcmd)
{
	struct cd_version_ind *cmd = (struct cd_version_ind*)llcmd;
	return cmd->length;
}

int llcmd_reject_ind(struct cd_dummy *llcmd)
{
	struct cd_reject_ind *cmd = (struct cd_reject_ind*)llcmd;
	llcp_data.last_error = cmd->error;
	return 0;
}

int llcmd_con_param_req(struct cd_dummy *llcmd)
{
	struct cd_con_param_req *cmd = (struct cd_con_param_req*)llcmd;
	/* TODO: ctr_data */
	return cmd->length;
}

int llcmd_con_param_rsp(struct cd_dummy *llcmd)
{
	struct cd_con_param_rsp *cmd = (struct cd_con_param_rsp*)llcmd;
	/* TODO: ctr_data */
	return cmd->length;
}

static int llrsp_con_param_rsp(struct cd_dummy *llcmd)
{
	struct cd_con_param_rsp *cmd = (struct cd_con_param_rsp*)llcmd;

	cmd->type = PDUH_LLID_CTRL;
	cmd->length = sizeof(struct cd_con_param_rsp) - 2;
	cmd->opcode = LL_CONNECTION_PARAM_RSP;
	/* TODO: ctr_data */
	return 0;
}

int llcmd_reject_ind_ext(struct cd_dummy *llcmd)
{
	struct cd_reject_ind_ext *cmd = (struct cd_reject_ind_ext*)llcmd;
	int retval = 0;
	if (cmd->length == 0) {
		retval = -1;
	}

	return retval;
}

int llcmd_ping_req(struct cd_dummy *llcmd)
{
	/* pdu does not have any data */
	struct cd_ping_req *cmd = (struct cd_ping_req*)llcmd;
	return cmd->length;
}

int llcmd_ping_rsp(struct cd_dummy *llcmd)
{
	/* pdu does not have any data */
	struct cd_ping_rsp *cmd = (struct cd_ping_rsp*)llcmd;
	return cmd->length;
}

static int llrsp_ping_rsp(struct cd_dummy *llcmd)
{
	/* pdu does not have any data */
	struct cd_ping_rsp *cmd = (struct cd_ping_rsp*)llcmd;
	cmd->type = PDUH_LLID_CTRL;
	cmd->length = sizeof(struct cd_ping_rsp) - 2;
	cmd->opcode = LL_PING_RSP;
	return 0;
}

static int ll_recv_dch_pdu(void)
{
	int retval = 0;
	msg_t m;
	struct cd_dummy *llcmd = (struct cd_dummy*)phy_get_next_rx_buf();
	static struct cd_dummy *llrsp;

	if (!llcmd)
		return 0;

	switch (llcmd->type & PDUH_DCH_LLID_MSK) {
	case PDUH_LLID_CONT:
		/* rest of the fragment or empty (l2cap) pdu */
		//phy_free_next_rx_buf();
		//break;
	case PDUH_LLID_START:
		/* first or a complete l2cap pdu */

		memcpy(blell_buf, llcmd, sizeof(struct ble_pdu));
		m.type = BLE_L2CAP_RCV_PKT;
		m.content.ptr = (char *)blell_buf;
		msg_send(&m, blell_data.ulayer_pid, false);

		phy_free_next_rx_buf();
		break;
	case PDUH_LLID_CTRL:
		llrsp = (struct cd_dummy*)phy_get_next_tx_buf();
		if (llrsp == 0) {
			/* wait a little */
			llcp_data.reply_pending = true;
			break;
		}
		llcp_data.last_rx_opcode = llcmd->opcode;

		if (llcmd->opcode > LL_PING_RSP) {
			llrsp_unknown_rsp(llrsp);
		}
		else {
			llrsp_unknown_rsp(llrsp);
			//llcmd_func[llcmd->opcode](llcmd);
		}

		llcp_data.reply_pending = false;
		llcp_data.last_tx_opcode = llrsp->opcode;
		phy_free_next_rx_buf();
		break;
	default:
		phy_free_next_rx_buf();
		retval = 0;
	}
	return retval;
}

static inline uint8_t ll_connect_req(struct ble_pdu *rx_pdu)
{
	struct le_connect_req_pdu *pdu;
	pdu = (struct le_connect_req_pdu*)rx_pdu;
	uint8_t i;
	uint8_t *dst = (uint8_t*)(&ble_connection_data);
	uint8_t *src = (uint8_t*)(&pdu->aa);

	/* TODO:
	init_addr_base = con_data->init_addr_base;
	init_addr_prefix = con_data->init_addr_prefix;
	init_addr_random = false;
	*/
	for (i = 0; i < sizeof(ble_connection_data); i++)
		dst[i] = src[i];

	phy_connection_data_update(&ble_connection_data, 0, PHY_CU_ALL);
	return phy_prepare_for_connection();
}

static inline void ll_scan_req_rsp(struct ble_pdu *rx_pdu)
{
	struct le_scan_req_pdu *pdu;
	pdu = (struct le_scan_req_pdu*)rx_pdu;

	blell_data.init_addr_base = pdu->scan_base;
	blell_data.init_addr_prefix = pdu->scan_prefix;
	/* TODO: address privat/random */
	blell_data.init_addr_random = false;
}

/* TODO: add white list handling, with at least one white list record */
static uint8_t ll_recv_ach_pdu(void)
{
	uint8_t retval = 0;
	struct ble_pdu *rx_pdu = phy_get_next_rx_buf();

	if (!rx_pdu)
		return 0;

	switch (rx_pdu->type & PDUH_ADV_TYPE_MSK) {
	case PDUH_SCAN_REQ:
		ll_scan_req_rsp(rx_pdu);
		break;

	case PDUH_CONNECT_REQ:
		retval = ll_connect_req(rx_pdu);
		break;

	/* ignore, not supported */
	case PDUH_SCAN_RSP:
	case PDUH_ADV_IND:
	case PDUH_ADV_DIRECT_IND:
	case PDUH_ADV_NONCONN_IND:
	case PDUH_ADV_SCAN_IND:
	default:
		retval = 0;
	}
	phy_free_next_rx_buf();
	return retval;
}

static inline void ble_link_recv_packet(uint32_t value)
{
	(void)value;

	if (blell_data.ll_state == link_connection) {
		ll_recv_dch_pdu();
	}
	else if (blell_data.ll_state == link_advertising) {
		if (ll_recv_ach_pdu()) {
			blell_data.ll_state = link_connection;
		}
	}
}

void ble_link_manager(void)
{
	msg_t m;
	msg_init_queue(msg_buffer, BLELL_MSG_BUFFER_SIZE);

	while (1) {
		DEBUG("blell manager: waiting for next message\n");
		msg_receive(&m);

		switch (m.type) {
		case BLE_PHY_RCV_PKT:
			ble_link_recv_packet(m.content.value);
			break;
		case BLE_PHY_CON_ESTBD:
			/* TODO: notify host*/
			break;
		case BLE_PHY_CON_LOST:
			/* TODO: notify host*/
		case BLE_PHY_CON_TERM:
			/* TODO: notify host*/
			blell_data.ll_state = link_standby;
			break;
		case BLE_CI_TERM_CON:
			/* TODO: notify peer device */
			if (blell_data.ll_state == link_connection) {
				phy_termitate_connection();
			}
			blell_data.adv_enabled = false;
			msg_reply(&m, &m);
			break;
		case BLE_CI_ENABLE_ADV:
			blell_data.adv_enabled = true;
			blell_data.ll_state = link_advertising;
			phy_prepare_for_adv_state(&blell_data.advpdu,
						&blell_data.scanrpdu,
						(uint16_t)m.content.value);
			msg_reply(&m, &m);
			break;
		case BLE_CI_DISABLE_ADV:
			blell_data.adv_enabled = false;
			if (blell_data.ll_state == link_advertising) {
				blell_data.ll_state = link_standby;
			}
			msg_reply(&m, &m);
			break;
		case BLE_CI_RND_ADR:
			/* TODO: set device random address*/
			msg_reply(&m, &m);
			break;
		case BLE_CI_ADV_PARAM:
        		memcpy(&blell_data.advpdu, m.content.ptr,
					sizeof(struct le_adv_pdu));
			msg_reply(&m, &m);
			break;
		case BLE_CI_SCANR_PARAM:
        		memcpy(&blell_data.scanrpdu, m.content.ptr,
					sizeof(struct le_scan_rsp_pdu));
			msg_reply(&m, &m);
			break;
		default:
			DEBUG("blell manager: unknown message received\n");
			break;
		}
	}

	/* TODO:
	 * notify waiting upper layers this is done non-blocking, 
	 * so packets can get lost
	if (msg_send(&m, blell_data.ulayer_pid, false) && (m.type != ENOBUFFER))
	 */
}

void ble_linklayer_init(void)
{
	/* do not re-initialize an already running link manager */
	if (link_manager_pid >= 0) {
		return;
	}
	/* Initializing transceiver buffer and data buffer */
	memset(blell_buf, 0, BLELL_BUFFER_SIZE * BLELL_PAYLOAD_SIZE);
	blell_data.ulayer_pid = 0;
}

/* start the ble phy thread */
int ble_linklayer_start(void)
{
	link_manager_pid = thread_create(link_manager_stack,
				LINK_LAYER_STACK_SIZE,
				5,
				CREATE_STACKTEST,
				ble_link_manager, "ble-ll");

	if (link_manager_pid < 0) {
		puts("error creating link manager thread");
	}
	else {
		DEBUG("link manager started\n");
		nrf51phy_setup(link_manager_pid);
	}

	return link_manager_pid;
}

uint8_t ble_linklayer_register(int pid)
{
	blell_data.ulayer_pid = pid;
        DEBUG("ble linklayer: thread %i registered", pid);
        return 1;
}
