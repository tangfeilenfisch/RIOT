/*
 * Copyright (C) 19.05.2014 Johann Fischer <j.fischer_at_fh-bingen_de>
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
 */

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/nrf51/rng.h>
#include <libopencm3/nrf51/ficr.h>
#include <libopencm3/nrf51/radio.h>
#include <libopencm3/nrf51/timer.h>
#include <libopencm3/nrf51/ppi.h>
#include <libopencm3/nrf51/gpio.h>

#include "msg.h"
#include "debug.h"
#include "ble_phy.h"
#include "blell.h"

#define PHY_DEBUG			1
#define PIN_DEBUG1			GPIO5
#define PIN_DEBUG2			GPIO6
#define PIN_DEBUG3			GPIO7
#define PIN_DEBUG4			GPIO8
#define PIN_DEBUG5			GPIO9

#define PDU_FIFO_LENGTH			(1 << 3) /* always the power of 2 */

enum ble_phy_state {
	phy_s_ready_rx		= 0,
	phy_s_end_rx		= 1,
	phy_s_ready_tx		= 2,
	phy_s_end_tx		= 3,
	phy_m_ready_tx		= 4,
	phy_m_end_tx		= 5,
	phy_m_ready_rx		= 6,
	phy_m_end_rx		= 7,
	phy_disabled		= 8,
};

struct ble_phy_data {
	enum ble_phy_state state;
	bool md;
	bool established;
	bool terminate;
	uint8_t sn;
	uint8_t nesn;
	uint8_t s_md;
	uint8_t tx_in_pos;
	uint8_t tx_out_pos;
	uint8_t rx_in_pos;
	uint8_t rx_out_pos;
	uint8_t invalid_crc;
	struct ble_pdu tx_buf[PDU_FIFO_LENGTH];
	struct ble_pdu rx_buf[PDU_FIFO_LENGTH];
	uint16_t conn_event_cntr;
	uint16_t denoted_event_cntr;
	uint32_t timeout_cntr;
	uint32_t pdu_cntr;
	/* -- */
	enum le_channel_idx ch_idx;
	uint8_t n_used_ch_idxs;
	uint32_t crc_init;
	uint32_t win_size;
	uint32_t win_offset;
	uint32_t interval;
	uint32_t latency;
	uint32_t timeout;
	uint32_t win_w;
	uint8_t hop;
};

struct ble_phy_data phy_data = {
	.state = phy_disabled,
	.terminate = 0,
	.sn = 0,
	.nesn = 0,
	.s_md = 0,
	.tx_in_pos = 0,
	.tx_out_pos = 0,
	.rx_in_pos = 0,
	.rx_out_pos = 0,
	.pdu_cntr = 0,
	.conn_event_cntr = 0,
};

static struct ble_pdu *assemb_advpdu;
static struct ble_pdu *assemb_scanrpdu;

/*****************************************************************************/
/* BLE PHY                                                                   */
/*****************************************************************************/
/*
 The master and slave alternate sending and receiving packets.
 The connection event is open while both devices continue to send packets.
 The slave shall always acknowledge a packet if it receives a from the master.
*/
/*****************************************************************************/
void nrf51phy_setup(int pid)
{
	link_manager_pid = pid;
	ble_radio_init(nrf51_txpower_0dbm);
	rf_enable_intr(nrf51_rf_ready);
	rf_enable_intr(nrf51_rf_end);
	nvic_enable_irq(NVIC_RADIO_IRQ);

	timer0_init_1us();
	nvic_enable_irq(NVIC_TIMER0_IRQ);

	/* setup ppi channels */
	ppi_setup_for_ble();
}

/*
 * The same channel index shall be used for all pdu's in
 * the connection event. 
 */
static uint8_t used_ch_idxs_tab[37];
static uint8_t last_unmap_ch_idx;

static void phy_ch_selection_algo(void)
{
	uint8_t unmap_ch_idx;
	uint8_t remap_idx;

	unmap_ch_idx = (last_unmap_ch_idx + phy_data.hop) % 37;

	if (used_ch_idxs_tab[unmap_ch_idx] == unmap_ch_idx) {
		/* use it as the data channel index */
		phy_data.ch_idx = unmap_ch_idx;
	}
	else {
		/* re-map to one of the used channel index */
		remap_idx = unmap_ch_idx % phy_data.n_used_ch_idxs;
		phy_data.ch_idx = used_ch_idxs_tab[remap_idx];
	}

	last_unmap_ch_idx = unmap_ch_idx;
}

static inline void nrf51phy_disable_timers_cc(void)
{
	timer0_stop();
	timer0_disable_intr(timer_compare0);
	timer0_disable_intr(timer_compare1);
	ppi_disable_ch(PPI_CH1 | PPI_CH2 | PPI_CH3);
}

/* transmit window starts at 1.25ms + phy_data.win_offset
 * and ends at 1.25ms + phy_data.win_offset + phy_data.win_size.
 * the first recieved packet determines the anchor point
 * for the first connection event and all future timings.
 * the second connection event shall be phy_data.interval after
 * the first connection event anchor point.
 * here we will setup timings for the first connection event.
 */
static inline void nrf51phy_init_timers_con_req(void)
{
	uint32_t tmp;
	phy_data.win_w = 0;

	if (PHY_DEBUG) {
		gpio_toggle(PIN_DEBUG1);
		gpio_toggle(PIN_DEBUG2);
		gpio_toggle(PIN_DEBUG3);
		gpio_toggle(PIN_DEBUG4);
		gpio_toggle(PIN_DEBUG5);
	}
	nrf51phy_disable_timers_cc();
	timer0_clear();

	tmp = 1250 + phy_data.win_offset - BLE_T_TXEN; 
	timer0_set_cc_reg(0, tmp, false);
	timer0_enable_intr(timer_compare0);

	tmp = 1250 + phy_data.win_offset + phy_data.win_size + BLE_T_TXEN; 
	timer0_set_cc_reg(1, tmp, false);
	timer0_enable_intr(timer_compare1);

	timer0_clear_all_events();
	/* enable ppi channels */
	ppi_enable_ch(PPI_CH1);

	timer0_start();
}

/* timer initialisation for advertising state */
static inline void nrf51phy_init_timers_adv_req(void)
{
	uint32_t tmp;
	phy_data.win_w = 0;

	if (PHY_DEBUG) {
		gpio_toggle(PIN_DEBUG1);
		gpio_toggle(PIN_DEBUG2);
		gpio_toggle(PIN_DEBUG3);
	}
	nrf51phy_disable_timers_cc();
	timer0_clear();

	tmp = 1250 - BLE_T_TXEN; 
	timer0_set_cc_reg(0, tmp, false);
	timer0_enable_intr(timer_compare0);

	tmp = 1250 + 625 + BLE_T_TXEN; 
	timer0_set_cc_reg(1, tmp, false);
	timer0_enable_intr(timer_compare1);

	timer0_clear_all_events();
	/* enable ppi channels */
	ppi_enable_ch(PPI_CH3);

	timer0_start();
}

/* timer update at anchor point */
static inline void nrf51phy_init_timers_ap(void)
{
	uint32_t tmp;

	phy_data.win_w = 0;
	phy_data.timeout_cntr = 0;
	phy_data.denoted_event_cntr ++;
	timer0_clear();

	tmp = phy_data.interval - BLE_T_TXEN; 
	timer0_set_cc_reg(0, tmp, false);

	tmp = phy_data.interval + BLE_T_TXEN; 
	timer0_set_cc_reg(1, tmp, false);
}

/* timer update at the end of connection event window,
 * shall be executed if no connection be achieved */
static inline void nrf51phy_set_time_rx_off_eap(void)
{
	uint32_t ap, b;

	ap = timer0_get_cc_reg(1) + phy_data.interval
		- BLE_T_TXEN - phy_data.win_w;

	if ((phy_data.win_w + BLE_T_TXEN) < (phy_data.interval >> 2)) {
		phy_data.win_w += BLE_T_TXEN;
	}

	/* window increase value is not conform to ble spec */
	b = ap - BLE_T_TXEN - phy_data.win_w;
	timer0_set_cc_reg(0, b, false);

	b = ap + BLE_T_TXEN + phy_data.win_w;
	timer0_set_cc_reg(1, b, false);
}

static inline void nrf51phy_timeout_supervisor(void)
{
	msg_t m;
	/* close / terminate connection */
	if (phy_data.established == false) {
		if (phy_data.denoted_event_cntr > 1) {
			phy_data.established = true;
			m.type = (uint16_t)BLE_PHY_CON_ESTBD;
			goto timeout_supervisor_msg;
		}
		if (phy_data.conn_event_cntr >= 6) {
			m.type = (uint16_t)BLE_PHY_CON_LOST;
			goto timeout_supervisor_close;
		}
	}

	if (phy_data.terminate == true) {
		m.type = (uint16_t)BLE_PHY_CON_TERM;
		goto timeout_supervisor_close;
	}

	if (phy_data.timeout_cntr >= phy_data.timeout) {
		m.type = (uint16_t)BLE_PHY_CON_LOST;
		goto timeout_supervisor_close;
	}

	return;

timeout_supervisor_close:
	nrf51phy_disable_timers_cc();
	rf_disable();
	phy_data.established = false;

timeout_supervisor_msg:
	if (link_manager_pid) {
		msg_send_int(&m, link_manager_pid);
	}
	return;
}

/* Timer0: connection event and transmit window timer.
 *
 * all timings values are in us.
 * phy_data.interval [7.5ms .. 4s]
 * phy_data.win_size [1.25ms .. 10ms | interval - 1.25ms]
 * phy_data.win_offset [0s .. interval]
 * phy_data.timeout [0.1s .. 32s]
 *
 * A connection event contains at least one data-pdu sent by the master
 * and can be closed by slave or master device.
 * The events are spaced with phy_data.interval and a 
 * connecton event closes at least T_IFS(150us)
 * before next connection event.
 * (source: Bluetooth LE specification)
 *
*/
void timer0_isr(void)
{
	/* start of a connection interval */
	/* timer shorts will start radio */
	if (timer0_get_intr_source(timer_compare0)) {
		if (PHY_DEBUG) gpio_toggle(PIN_DEBUG1);
		timer0_clear_event(timer_compare0);
		phy_data.conn_event_cntr ++;
		nrf51phy_timeout_supervisor();
		return;
	}

	if (timer0_get_intr_source(timer_compare1)) {
		/* a pdu was not received in a transmit window */
		if (PHY_DEBUG) gpio_toggle(PIN_DEBUG2);
		timer0_clear_event(timer_compare1);
		phy_data.timeout_cntr += phy_data.interval;

		/* shutdown the  radio and prepare for next anchor point */
		nrf51phy_set_time_rx_off_eap();
		phy_ch_selection_algo();
		ble_set_ch(phy_data.ch_idx);
		rf_disable();
		rf_shorts_ready_start();

		if (phy_data.state < phy_m_ready_tx) {
			phy_data.state = phy_s_ready_rx;
		}
		else if (phy_data.state < phy_disabled) {
			rf_set_packetptr((void*)(assemb_advpdu));
			phy_data.state = phy_m_ready_tx;
		}

		return;
	}
}

/*****************************************************************************/

inline void nrf51phy_notify_transceiver_thread(void)
{
	if (link_manager_pid) {
		msg_t m;
		m.type = (uint16_t)BLE_PHY_RCV_PKT;
		m.content.value = phy_data.rx_in_pos;
		msg_send_int(&m, link_manager_pid);
	}
}

inline struct ble_pdu* phy_get_rx_buf(uint8_t pos)
{
	return (phy_data.rx_buf + (pos & (PDU_FIFO_LENGTH - 1)));
}

inline int phy_free_pending_rx_buf(void)
{
	if (phy_data.rx_out_pos == phy_data.rx_in_pos) {
		return 0;
	}
	phy_data.rx_out_pos = (phy_data.rx_out_pos + 1) & (PDU_FIFO_LENGTH - 1);
	return 1;
}

inline int phy_rx_data_pending(void)
{
	if (phy_data.rx_out_pos == phy_data.rx_in_pos) {
		return 0;
	}
	return 1;
}

inline struct ble_pdu* phy_get_next_tx_buf(void)
{
	if ((phy_data.tx_in_pos + 1) == phy_data.tx_out_pos) {
		return 0;
	}
	return (phy_data.tx_buf + phy_data.tx_in_pos);
}

inline void phy_set_pending_tx_buf(void)
{
	phy_data.tx_in_pos = (phy_data.tx_in_pos + 1) & (PDU_FIFO_LENGTH - 1);
}

inline void phy_reset_fifo(void)
{
	phy_data.tx_out_pos = 0;
	phy_data.tx_in_pos = 0;
	phy_data.rx_out_pos = 0;
	phy_data.rx_in_pos = 0;
}

/* 
 * see bluetooth specification: 4.5.9 Acknowledgement and Flow Control
 */
inline static void radio_isr_receiving_data(void)
{
	struct ble_pdu *pdu = phy_data.rx_buf + phy_data.rx_in_pos;

	if (!rf_get_crcstat()) {
		phy_data.invalid_crc ++;
		/* crc fail, unreliable NESN, re-transmit pdu */
		goto do_retranmit_pdu;
	}

	if (pdu->type & PDUH_DCH_MD)
		phy_data.md = true;
	else
		phy_data.md = false;

	if ((pdu->type & PDUH_DCH_SN) ^ (phy_data.nesn << 1)) {
		/* SN and phy_data.nesn are different, old data, ignore */
		goto do_not_change_nesn;
	}
	/* SN and phy_data.nesn are same, rx new data */
	if ((phy_data.rx_in_pos + 1) == phy_data.rx_out_pos) {
		/* fifo is full, nack */
		goto do_not_change_nesn;
	}
	nrf51phy_notify_transceiver_thread();
	phy_data.rx_in_pos = (phy_data.rx_in_pos + 1) & (PDU_FIFO_LENGTH - 1);
	phy_data.nesn ^= PDUH_DCH_NESN;	/* increment phy_data.nesn */

do_not_change_nesn:

	if ((pdu->type & PDUH_DCH_NESN) == (phy_data.sn >> 1)) {
		/* phy_data.sn and NESN are same, nack, re-transmit pdu */
		goto do_retranmit_pdu;
	}
	/* NESN and phy_data.sn are different, tx new data */

	if (phy_data.s_md ) {
		/* ack for tx data pending */
		phy_data.tx_out_pos = (phy_data.tx_out_pos + 1) & (PDU_FIFO_LENGTH - 1);
	}

	/* there should be a pdu in tx-fifo, at least a empty-pdu */
	/* FIXME: fifo is empty, transmit empty pdu (re-transmit last pdu?) */
	if (phy_data.tx_out_pos == phy_data.tx_in_pos) {
		phy_data.s_md = 0;
		goto do_retranmit_pdu;
	}

	phy_data.s_md = PDUH_DCH_MD;	/* continue transmitting */
	phy_data.sn ^= PDUH_DCH_SN;	/* increment phy_data.sn */

do_retranmit_pdu:
	phy_data.tx_buf[phy_data.tx_out_pos].type &= ~PDUH_DCH_FLAG_MSK;
	phy_data.tx_buf[phy_data.tx_out_pos].type |= phy_data.nesn;
	phy_data.tx_buf[phy_data.tx_out_pos].type |= phy_data.sn;
	phy_data.tx_buf[phy_data.tx_out_pos].type |= phy_data.s_md;
	rf_set_packetptr((void*)(&phy_data.tx_buf[phy_data.tx_out_pos]));
}

inline static void radio_isr_receiving_adv_state(void)
{
	struct ble_pdu *pdu;

	if (!rf_get_crcstat()) {
		rf_set_packetptr((void*)(assemb_advpdu));
		phy_data.md = false;
		return;
	}
	pdu = phy_data.rx_buf + phy_data.rx_in_pos;

	/* successfully transmitted and received pdu,
	 * guess it is a acknowledgement, increment tx-fifo out positon.
	 * dirty gimmick: look into pdu header */
	if ((pdu->type & PDUH_ADV_TYPE_MSK) == PDUH_SCAN_REQ) {
		rf_set_packetptr((void*)(assemb_scanrpdu));
		phy_data.md = true;
	}
	else {
		rf_set_packetptr((void*)(assemb_advpdu));
		phy_data.md = false;
	}

	/* there is no any handshake from peer, 
	 * always increment rx in position */
	nrf51phy_notify_transceiver_thread();
	phy_data.rx_in_pos = (phy_data.rx_in_pos + 1) & (PDU_FIFO_LENGTH - 1);
}


void radio_isr(void)
{
	if (PHY_DEBUG) gpio_toggle(PIN_DEBUG3);

	if (rf_get_event_ready()) {
		if (PHY_DEBUG) gpio_toggle(PIN_DEBUG4);

		switch (phy_data.state) {
		case phy_s_ready_rx:
			/* radio can receive a new pdu and may switch
			 * to tx-mode over disabled state */
			rf_shorts_end_disable_txen();
			phy_data.state = phy_s_end_rx;
			break;
		case phy_s_ready_tx:
			/* radio has transmitted a pdu and may
			 * switch to disabled state */
			rf_shorts_end_disable_rxen();
			phy_data.state = phy_s_end_tx;
			break;
		case phy_m_ready_tx:
			/* radio will transmit a pdu and may switch
			 * to rx-mode over disabled state */
			rf_shorts_end_disable_rxen();
			phy_data.state = phy_m_end_tx;
			break;
		case phy_m_ready_rx:
			/* radio can receive a new pdu and may switch
			 * to disabled state */
			rf_shorts_end_disable_txen();
			phy_data.state = phy_m_end_rx;
			break;
		default:
			phy_data.state = phy_disabled;
			rf_disable();
			break;
		}
		rf_clear_event_disabled();
		rf_clear_event_ready();
		if (PHY_DEBUG) gpio_toggle(PIN_DEBUG4);
		return;
	}

	if (rf_get_event_end()) {
		if (PHY_DEBUG) gpio_toggle(PIN_DEBUG5);

		switch (phy_data.state) {
		case phy_s_end_rx:
			/* transit to tx-mode over disabled state,
			 * update the packet pointer
			 */
			if (!phy_data.pdu_cntr) {
				nrf51phy_init_timers_ap();
			}
			phy_data.pdu_cntr ++;

			radio_isr_receiving_data();
			phy_data.state = phy_s_ready_tx;
			rf_shorts_disable_txen_start();
			break;
		case phy_s_end_tx:
			/* transit to disabled state,
			 * if there are more data to transmit (MD-bit), 
			 * re-enable rx-disabled-tx sequence */
			rf_set_packetptr(&phy_data.rx_buf[phy_data.rx_in_pos]);
			if (phy_data.md || phy_data.s_md) {
				if (PHY_DEBUG) gpio_toggle(PIN_DEBUG5);
				rf_shorts_disable_rxen_start();
			}
			else {
				rf_disable();
				rf_shorts_ready_start();
				phy_data.pdu_cntr = 0;
				phy_ch_selection_algo();
				ble_set_ch(phy_data.ch_idx);
			}
			phy_data.state = phy_s_ready_rx;
			break;
		case phy_m_end_tx:
			/* transit to rx-mode over disabled state,
			 * change the packet pointer*/
			rf_set_packetptr(&phy_data.rx_buf[phy_data.rx_in_pos]);
			phy_data.state = phy_m_ready_rx;
			rf_shorts_disable_rxen_start();
			break;
		case phy_m_end_rx:
			/* transit to disabled state,
			 * if there are more data to receive (MD-bit), 
			 * re-enable tx-disabled-rx sequence */
			radio_isr_receiving_adv_state();
			if (phy_data.md) {
				rf_shorts_disable_txen_start();
			}
			else {
				rf_disable();
				rf_shorts_ready_start();
			}
			phy_data.state = phy_m_ready_tx;
			break;
		default:
			phy_data.state = phy_disabled;
			break;
		}
		rf_clear_event_end();
		return;
	}
}

/*****************************************************************************/

/* 
 * link layer shall listen on the same advetising channel for
 * request from scanner or initiator.
 *
 * after CONNECT_REQ, link layer shall exit the advertising
 * state and transition to the connection state (slave role).
 * (source: Bluetooth LE specification)
 */
void phy_prepare_for_adv_state(struct ble_pdu *advpdu,
				struct ble_pdu *scanrpdu,
				uint16_t adv_interval)
{
	used_ch_idxs_tab[0] = ach37;
	used_ch_idxs_tab[1] = ach38;
	used_ch_idxs_tab[2] = ach39;
	phy_data.n_used_ch_idxs = 3;
	last_unmap_ch_idx = 0;
	phy_data.hop = 1;
	phy_ch_selection_algo();
	ble_set_ch(phy_data.ch_idx);

	ble_adv_access_addr_en();
	phy_data.win_offset	= 0;
	phy_data.interval	= (adv_interval + (rng_get_octet() & 0x7))*625;
	phy_data.win_size	= phy_data.interval;
	phy_data.timeout 	= 60000000;
	phy_data.established 	= true;

	assemb_advpdu = advpdu;
	assemb_scanrpdu = scanrpdu;
	rf_clear_event_end();
	rf_clear_event_disabled();
	rf_shorts_ready_start();
	phy_data.state = phy_m_ready_tx;
	rf_set_packetptr(assemb_advpdu);

	nrf51phy_init_timers_adv_req();
}

/*****************************************************************************/
/* ble-phy abstraction */

void phy_connection_data_update(struct ll_connection_udata *con_data,
				uint16_t instant, uint8_t type)
{
	uint8_t i;

	if (type == PHY_CU_ALL) {
		ble_crc_init((con_data->crc_init[2]<<16)
				| (con_data->crc_init[1]<<8)
				| con_data->crc_init[0]);

		ble_access_addr_en(con_data->aa);
	}

	if ((type == PHY_CU_ALL) || (type == PHY_CU_T)) {
		phy_data.win_size	= con_data->win_size * 1250;
		phy_data.win_offset	= con_data->win_offset * 1250;
		phy_data.interval	= con_data->interval * 1250;
		phy_data.latency	= con_data->latency;
		phy_data.timeout	= con_data->timeout * 10000;
		phy_data.hop		= con_data->hop_sca & 0x1F;
		/*phy_data.sca		= con_data->hop_sca >> 5;*/
	}

	if ((type == PHY_CU_ALL) || (type == PHY_CU_CHMAP)) {
		phy_data.n_used_ch_idxs = 0;
		for (i = 0; i < 37; i++) {
			if (con_data->ch_map[i >> 3] & (1 << (i & 0x7))) {
				used_ch_idxs_tab[phy_data.n_used_ch_idxs] = i;
				phy_data.n_used_ch_idxs ++;	
			}
		}
		for (i = phy_data.n_used_ch_idxs; i < 37; i++) {
			used_ch_idxs_tab[i] = 0;
		}
		last_unmap_ch_idx = 0;

		phy_ch_selection_algo();
	}
	/* TODO: evaluate instant value */
	(void) instant;
}

int phy_prepare_for_connection(void)
{
	phy_reset_fifo();
	ble_set_ch(phy_data.ch_idx);
	/* slave role, prepare for receiving */
	rf_clear_event_ready();
	rf_clear_event_end();
	rf_clear_event_disabled();
	rf_set_packetptr(&phy_data.rx_buf[phy_data.rx_in_pos]);
	phy_data.pdu_cntr = 0;
	phy_data.state = phy_s_ready_rx;
	rf_shorts_ready_start();

	nrf51phy_init_timers_con_req();

	phy_data.established = false;
	phy_data.conn_event_cntr = 0;
	phy_data.timeout_cntr = 0;
	phy_data.invalid_crc = 0;

	phy_data.sn = 0;
	phy_data.nesn = 0;
	phy_data.s_md = 0;
	return 1;
}

inline void phy_rng_get_eight(uint8_t *rnd_buf)
{
	rng_get_eight_octets(rnd_buf);
}

inline uint8_t phy_get_txpower(void)
{
	return (uint8_t)rf_get_txpower();
}

inline uint8_t phy_get_max_txpower(void)
{
	return (uint8_t)nrf51_txpower_4dbm;
}

inline int phy_remove_dev_from_white_list(uint32_t b, uint16_t p)
{
	return ble_dab_del(b, p);
}

inline int phy_add_dev_to_white_list(uint32_t b, uint16_t p, uint8_t random)
{
	return ble_dab_add(b, p, random);
}

inline void phy_clear_white_list(void)
{
	ble_dab_clear();
}

inline uint8_t phy_read_white_list_size_cmd(void)
{
	return RADIO_DAB_SIZE;
}

inline void phy_termitate_connection(void)
{
	phy_data.terminate = true;
}
