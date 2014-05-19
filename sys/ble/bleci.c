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
 * controller interface
 *
 * TODO: implement the necessary functions :-)
 */

#include "blell.h"
#include "bleci.h"
#include "le_errors.h"
#include "ad_types.h"

#include "msg.h"

static struct advparameters advparam = {
	.i_min			= APT_IMIN_DEFAULT,
	.i_max			= APT_IMAX_DEFAULT,
	.adv_type		= APT_ADV_IND,
	.dev_addr_type		= APT_RANDOM_ADDR,
	.dir_addr_type		= APT_RANDOM_ADDR,
	.dir_addr_b		= 0,
	.dir_addr_p		= 0,
	.adv_ch_map		= APT_CHMAP_ALL,
	.adv_policy		= APT_FP_WHITEL_OFF,
};

static uint8_t adv_data[ADV_DATA_LENGTH];

static bool advpdu_assembled;
static uint8_t adv_data_length;
static uint8_t scanr_data_length;
static struct ble_pdu advpdu;
static struct ble_pdu scanrpdu;
static bool dev_addr_random;
static uint16_t dev_addr_prefix;
static uint32_t dev_addr_base;
static uint16_t adv_interval;

/*****************************************************************************/
/* controller interface helper functions                                     */
/*****************************************************************************/
struct advparameters* ll_get_adv_parameters(void)
{
	return &advparam;
}

uint8_t* ll_clear_adv_data(void)
{
	return adv_data;
}

int ll_gen_adv_data_flag(uint8_t *data, uint8_t i)
{
	if ((i+2) > ADV_DATA_LENGTH)
		return 0;

	data[i+0] = 2; /* length of this data record */
	data[i+1] = AD_TYPE_FLAGS;
	data[i+2] = FLAG_LE_LIMDISCMODE | FLAG_BREDR_NSUPP;
	return 3;
}

int ll_gen_adv_data_role(uint8_t *data, uint8_t i)
{
	if ((i+2) > ADV_DATA_LENGTH)
		return 0;

	data[i+0] = 2; /* length of this data record */
	data[i+1] = AD_TYPE_LE_ROLE;
	data[i+2] = LE_ROLE_P;
	return 3;
}

int ll_gen_adv_data_txpwr(uint8_t *data, uint8_t i)
{
	if ((i+2) > ADV_DATA_LENGTH)
		return 0;

	data[i+0] = 2; /* length of this data record */
	data[i+1] = AD_TYPE_TX_PWR_LEVEL;
	data[i+2] = phy_get_txpower();
	return 3;
}

/*****************************************************************************/
/* controller interface functions                                            */
/*****************************************************************************/
int le_set_random_address_cmd(uint8_t *rnd_buf)
{
	uint16_t p 	= (rnd_buf[1]<<8) | (rnd_buf[0]);
	uint32_t b 	= (rnd_buf[5]<<24) | (rnd_buf[4]<<16)
			| (rnd_buf[3]<<8) | (rnd_buf[2]);

	if ((b == 0) || (p == 0)) {
		/*
		b = ficr_get_daddr_base();
		p = ficr_get_daddr_prefix();
		*/
		return INVALID_LL_PARAM_ERR;
	}
	dev_addr_prefix = p;
	dev_addr_base = b;
	dev_addr_random = true;

	if (link_manager_pid) {
		msg_t m;
		msg_t r;
		m.type = (uint16_t)BLE_CI_RND_ADR;
		m.content.ptr = (char*)rnd_buf;
		msg_send_receive(&m, &r, link_manager_pid);
	}
	return 0;
}

int le_add_dev_to_white_list_cmd(uint8_t param, uint32_t b, uint16_t p)
{
	return phy_add_dev_to_white_list(b, p, param);
}

int le_clear_white_list_cmd(void)
{
	phy_clear_white_list();
	return 0;
}

int le_read_white_list_size_cmd(uint8_t *size)
{
	*size = phy_read_white_list_size_cmd();
	return 0;
}

int le_remove_dev_from_white_list_cmd(uint32_t b, uint16_t p)
{
	return phy_remove_dev_from_white_list(b, p);
}

int le_read_adv_channel_tx_power_cmd(int8_t *dbm)
{
	*dbm = phy_get_txpower();
	return 0;
}

int read_transmit_power_level_cmd(uint8_t param, int8_t *dbm)
{
	if (!param) {
		*dbm = phy_get_txpower();
	}
	else {
		*dbm = phy_get_max_txpower();
	}
	return 0;
}

int le_set_adv_parameters_cmd(struct advparameters *param)
{
	struct le_adv_pdu *pdu;
	struct le_adv_direct_pdu *dpdu;
	struct le_scan_rsp_pdu *rpdu;
	pdu = (struct le_adv_pdu*)&advpdu;
	dpdu = (struct le_adv_direct_pdu*)(&advpdu);
	rpdu = (struct le_scan_rsp_pdu*)&scanrpdu;

	advpdu_assembled = false;

	if (param->i_min > param->i_max)
		return INVALID_CMD_PARAM_ERR;
	if ((param->i_min < 0x0020) || (param->i_min > 0x4000))
		param->i_min = 0x0800;
	if ((param->i_max < 0x0020) || (param->i_max > 0x4000))
		param->i_max = 0x0800;

	if (!param->dev_addr_type) /* at this stage: use a random address */
		return INVALID_CMD_PARAM_ERR;

	if (param->adv_ch_map != 0x7) /* at this stage: use all advertising channels */
		return INVALID_CMD_PARAM_ERR;

	switch (param->adv_policy) {
	case APT_FP_WHITEL_OFF:
		//whitel_enabled = false;
		break;
	case APT_FP_WHITEL_ALL:
		//whitel_enabled = true;
		break;
	case APT_FP_WHITEL_SCAN:
	case APT_FP_WHITEL_CON:
	default:
		return INVALID_CMD_PARAM_ERR;
	}

	adv_interval = param->i_max;
	/* assembly advertising pdu */
	switch (param->adv_type) {
	case APT_ADV_IND:
		pdu->length 		= adv_data_length + 6;
		pdu->type		= PDUH_ADV_IND;
		pdu->addr_base		= dev_addr_base;
		pdu->addr_prefix	= dev_addr_prefix;

		if (dev_addr_random)
			pdu->type |= PDUH_ADV_TXADD; /* random address */

		rpdu->length 		= scanr_data_length + 6;
		rpdu->type		= PDUH_SCAN_RSP;
		rpdu->addr_base		= dev_addr_base;
		rpdu->addr_prefix	= dev_addr_prefix;

		if (dev_addr_random)
			rpdu->type |= PDUH_ADV_TXADD; /* random address */

		break;
	case APT_ADV_DIRECT_IND_HC:
		adv_interval = param->i_min;
	case APT_ADV_DERECT_IND_LC:
		dpdu->length = sizeof(struct le_adv_direct_pdu) - 2;

		dpdu->type		= PDUH_ADV_DIRECT_IND;
		dpdu->addr_base 	= dev_addr_base;
		dpdu->addr_prefix 	= dev_addr_prefix;
		dpdu->init_addr_base	= param->dir_addr_b;
		dpdu->init_addr_prefix	= param->dir_addr_p;

		if (dev_addr_random)
			dpdu->type |= PDUH_ADV_TXADD; /* random address */
		if (param->dir_addr_type)
			dpdu->type |= PDUH_ADV_RXADD; /* random address */

		break;
	case APT_ADV_SCAN_IND:
	case APT_ADV_NONCONN_IND:
	default:
		return INVALID_CMD_PARAM_ERR;
	}

	advpdu_assembled = true;
	if (link_manager_pid) {
		msg_t m;
		msg_t r;
		m.type = (uint16_t)BLE_CI_ADV_PARAM;
		m.content.ptr = (char*)&advpdu;
		msg_send_receive(&m, &r, link_manager_pid);

		m.type = (uint16_t)BLE_CI_SCANR_PARAM;
		m.content.ptr = (char*)&scanrpdu;
		msg_send_receive(&m, &r, link_manager_pid);
	}
	return 0;
}

int le_set_adv_data_cmd(uint8_t param, uint8_t *data)
{
	struct le_adv_pdu *pdu = (struct le_adv_pdu*)(&advpdu);
	uint8_t i;

	if ((pdu->type & PDUH_ADV_TYPE_MSK) == PDUH_ADV_DIRECT_IND)
		return INVALID_CMD_PARAM_ERR;
	if (param > ADV_DATA_LENGTH)
		return INVALID_CMD_PARAM_ERR;

	advpdu_assembled = false;
	adv_data_length = 0;
	for (i = 0; i < param; i++)
		pdu->data[i] = data[i];

	adv_data_length = param;
	pdu->length = adv_data_length + 6;
	advpdu_assembled = true;
	if (link_manager_pid) {
		msg_t m;
		msg_t r;
		m.type = (uint16_t)BLE_CI_ADV_PARAM;
		m.content.ptr = (char*)&advpdu;
		msg_send_receive(&m, &r, link_manager_pid);
	}
	return 0;
}

int le_set_scan_response_data_cmd(uint8_t param, uint8_t *data)
{
	struct le_adv_pdu *pdu = (struct le_adv_pdu*)(&scanrpdu);
	uint8_t i;
	scanr_data_length = 0;

	if (param > ADV_DATA_LENGTH)
		return INVALID_CMD_PARAM_ERR;

	for (i = 0; i < param; i++)
		pdu->data[i] = data[i];

	scanr_data_length = param;
	pdu->length = scanr_data_length + 6;
	if (link_manager_pid) {
		msg_t m;
		msg_t r;
		m.type = (uint16_t)BLE_CI_SCANR_PARAM;
		m.content.ptr = (char*)&scanrpdu;
		msg_send_receive(&m, &r, link_manager_pid);
	}
	return 0;
}

int le_set_adv_enable_cmd(uint8_t param)
{
	msg_t m;
	msg_t r;

	if ((!advpdu_assembled) || (!link_manager_pid)) {
		return -1;
	}

	if (param) {
		m.type = (uint16_t)BLE_CI_ENABLE_ADV;
	}
	else {
		m.type = (uint16_t)BLE_CI_DISABLE_ADV;
	}
	m.content.value = adv_interval;
	msg_send_receive(&m, &r, link_manager_pid);
	return 0;
}

int le_con_update_cmd(struct conparameters *param)
{
	uint32_t tmp;

	if (param->i_min > param->i_max)
		return -1;
	if (param->ce_min > param->ce_max)
		return -1;
	if ((param->i_min < 0x0006) || (param->i_min > 0x0c80))
		return -1;
	if ((param->i_max < 0x0006) || (param->i_max > 0x0c80))
		return -1;
	if ((param->sv_timeout < 0x000a) || (param->sv_timeout > 0x0c80))
		return -1;

	tmp = (1 + param->latency) * param->ce_max * 2;
	if (param->sv_timeout < tmp)
		return -1;

	/* TODO: to implement */
	return 0;
}

int le_read_channel_map_cmd(uint16_t handle, uint64_t *ch_map)
{
	if (handle != 0)
		return UNK_CON_ID_ERR;
	/* TODO: to implement */
	*ch_map = 0;
	return 0;
}

int le_read_supported_states_cmd(uint64_t *states)
{
	*states 	= LE_STATES_SCAN_ADV
			| LE_STATES_CON_ADV
			| LE_STATES_HC_ADV
			| LE_STATES_S_CON
			| LE_STATES_LC_ADV;
	return 0;
}

int le_read_buf_size_cmd(uint16_t *p_length, uint8_t *p_number)
{
	/* TODO: to implement */
	*p_number = 1;
	*p_length = 255;
	return 0;
}

int le_rand_cmd(uint8_t *rnd_buf)
{
	phy_rng_get_eight(rnd_buf);
	return 0;
}

int discon_cmd(uint16_t handle, uint8_t reason)
{
	if (handle != 0)
		return UNK_CON_ID_ERR;
	if (reason == 0)
		return -1;
	if (link_manager_pid) {
		msg_t m;
		msg_t r;
		m.type = (uint16_t)BLE_CI_TERM_CON;
		msg_send_receive(&m, &r, link_manager_pid);
	}
	return 0;
}

int read_bd_addr_cmd(uint32_t *b, uint16_t *p)
{
	*b = 0x0;
	*p = 0x0;
	return 0;
}

int le_read_remote_used_features_cmd(uint16_t handle)
{
	if (handle != 0)
		return UNK_CON_ID_ERR;
	/* TODO: to implement */
	return 0;
}

int le_read_local_supported_features_cmd(uint64_t *le_features)
{
	/* see spec, vol6, page 81, Feature Support */
	(void)le_features;
	return 0;
}

int read_local_supported_features_cmd(uint64_t *features)
{
	/* see spec, vol2, page 238, Device Features */
	(void)features;
	return 0;
}

int read_local_supported_commands_cmd(uint64_t *commands)
{
	/* TODO: to implement */
	*commands = 0;
	return 0;
}

int read_rssi_cmd(uint16_t handle, int8_t *dbm)
{
	if (handle != 0)
		return UNK_CON_ID_ERR;
	/* TODO: to implement */
	*dbm = 127;
	return 0;
}

int reset_cmd(void)
{
	/* reset link layer */
	return 0;
}


