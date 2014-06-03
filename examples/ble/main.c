/*
 * Copyright (C) 2008, 2009, 2010  Kaspar Schleiser <kaspar@schleiser.de>
 * Copyright (C) 2013 INRIA
 * Copyright (C) 2013 Ludwig Ortmann <ludwig.ortmann@fu-berlin.de>
 * Copyright (C) 2014 Johann Fischer <j.fischer@fh-bingen.de>
 *
 * This file subject to the terms and conditions of the GNU Lesser General
 * Public License. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     examples
 * @{
 *
 * @file
 * @brief       Test application that shows functionality of ble-stack
 * 		on nrf51822
 *
 * @author      Johann Fischer <j.fischer@fh-bingen.de>
 *
 * @}
 */

#include <stdio.h>
#include <string.h>

#include "thread.h"
#include "posix_io.h"
#include "shell.h"
#include "shell_commands.h"
#include "board_uart0.h"
#include "debug.h"

#include "blell.h"
#include "bleci.h"
#include "blel2cap.h"

#include "net_if.h"
#include "sixlowpan.h"
#include "sixlowpan/lowpan.h"

#define RCV_BUFFER_SIZE     (64)
#define L2CAP_STACK_SIZE    (KERNEL_CONF_STACKSIZE_DEFAULT)


char l2cap_test_stack[L2CAP_STACK_SIZE];
msg_t l2cap_msg_q[RCV_BUFFER_SIZE];

static int ul_pid = -1;

static void ble_l2cap_read_frame(char *pkt)
{
	net_if_eui64_t src, dst;

	struct l2cap_fheader *frame = (struct l2cap_fheader *)pkt;
	switch (frame->ch_id) {
	case L2CAP_CHID_6LOWPAN:
		printf("\tlength:\t%u\n", frame->f_length);
		printf("\tch_id:\t%02x\n", frame->ch_id);
		for (uint8_t i = 0; i < (frame->f_length); i++) {
			printf("%02x ", pkt[i + L2CAP_FHEADER_SIZE]);
		}
		puts("\n");
		/*
		TODO:
		*/
            	lowpan_read((pkt + L2CAP_FHEADER_SIZE), frame->f_length, &src, &dst);
		break;

	case L2CAP_CHID_ATT:
		break;

	case L2CAP_CHID_LE_SIGNALING:
		break;

	case L2CAP_CHID_SMP:
		break;

	default:
		break;
	}
}

void l2cap_test(void)
{
	msg_t m;

	msg_init_queue(l2cap_msg_q, RCV_BUFFER_SIZE);

	while (1) {
		msg_receive(&m);

		if (m.type == BLE_L2CAP_PKT_PENDING) {
			ble_l2cap_read_frame(m.content.ptr);
			m.type = BLE_L2CAP_PKT_HANDLED;
			msg_send(&m, m.sender_pid, false);
		}
		else if (m.type == BLE_ENOBUFFER) {
			puts("linklayer buffer full");
		}
		else {
			puts("Unknown packet received");
		}
	}
}

void init_ble(void)
{
	uint8_t i;
	int l2cap_test_pid = thread_create(l2cap_test_stack,
					L2CAP_STACK_SIZE,
					PRIORITY_MAIN - 2,
					CREATE_STACKTEST,
					l2cap_test,
					"l2cap");

	ble_linklayer_init();
	(void) ble_linklayer_start();
	ble_linklayer_register(l2cap_test_pid);

	uint8_t rnd_buf[8] = {0x42,0x42,0x42,0x42,0x42,0x42,0,0};
	uint8_t *adv_data;
	struct advparameters *advparam;

	/* configure phy */
	/* configure link layer */
	/* - setup device address */
	//le_rand_cmd(rnd_buf);
	//le_encrypt_cmd();
	le_set_random_address_cmd(rnd_buf);
	/* - setup white list */
	//le_read_white_list_size_cmd();
	le_clear_white_list_cmd();
	/*
	if (ll_dev_paired) {
		le_add_dev_to_white_list_cmd(1, b, p)
	}
	else
	*/

	/* configure device for undirected advertising */
	/* - get/set default advertising parameters */
	advparam = ll_get_adv_parameters();
	le_set_adv_parameters_cmd(advparam);
	/* - generate advertising data */
	adv_data = ll_clear_adv_data();
	i = ll_gen_adv_data_flag(adv_data, 0);
	le_set_adv_data_cmd(i, adv_data);
	/* - add/generate scan response data */
	i += ll_gen_adv_data_role(adv_data, i);
	i += ll_gen_adv_data_txpwr(adv_data, i);
	le_set_scan_response_data_cmd(i, adv_data);
	le_set_adv_enable_cmd(1);



	//int iface;
        //iface = net_if_init_interface(0, 0x42);

        //net_if_set_src_address_mode(iface, NET_IF_TRANS_ADDR_M_SHORT);
        //net_if_set_hardware_address(iface, 0x42);


        //if (iface >= 0) {
        //    DEBUG("Auto init interface %d\n", iface);
        //}
}

int transceiver_pid = 0;

/* Register an upper layer thread */
uint8_t transceiver_register(uint16_t t, int pid)
{
	(void)t;
	ul_pid = pid;
	DEBUG("l2cap: Thread %i registered\n", pid);
	return 1;
}

static int shell_readc(void)
{
	char c = 0;
	(void) posix_read(uart0_handler_pid, &c, 1);
	return c;
}

static void shell_putchar(int c)
{
	(void) putchar(c);
}

int main(void)
{
	//shell_t shell;
	(void) posix_open(uart0_handler_pid, 0);


	init_ble();

	(void) puts("Welcome to RIOT!");
	while (true) {
		thread_yield();
	}

	//shell_init(&shell, NULL, UART0_BUFSIZE, shell_readc, shell_putchar);

	//shell_run(&shell);
	return 0;
}
