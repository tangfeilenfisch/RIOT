/*
 * Copyright (C) 2008, 2009, 2010  Kaspar Schleiser <kaspar@schleiser.de>
 * Copyright (C) 2013 INRIA
 * Copyright (C) 2013 Ludwig Ortmann <ludwig.ortmann@fu-berlin.de>
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
 * @brief       examle for stm32f0discovery-board
 *
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 * @author      Oliver Hahm <oliver.hahm@inria.fr>
 * @author      Ludwig Ortmann <ludwig.ortmann@fu-berlin.de>
 * @author      Johann Fischer <j.fischer@fh-bingen.de>
 *
 * @}
 */

#include <stdio.h>
#include <string.h>

#include "thread.h"
#include "msg.h"
#include "board_uart0.h"

char second_thread_stack[KERNEL_CONF_STACKSIZE_MAIN];

void second_thread(void)
{
	printf("second_thread starting.\n");
	msg_t m;

	while (1) {
		msg_receive(&m);
		printf("42\n");
		m.content.value++;
		msg_reply(&m, &m);
	}
}


int main(void)
{
	msg_t m;

	(void) puts("Welcome to RIOT!");

	int pid = thread_create(second_thread_stack,
			sizeof(second_thread_stack),
			PRIORITY_MAIN - 1, CREATE_STACKTEST,
			second_thread, "pong");

	m.content.value = 1;

	while (1) {
		msg_send_receive(&m, &m, pid);
		printf("What is the answer to the ultimate question of ");
		printf("life the universe and everything?\n");
	}
	return 0;
}
