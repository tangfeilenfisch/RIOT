/******************************************************************************

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.

You should have received a copy of the GNU General Public License along with
this program.  If not, see http://www.gnu.org/licenses/ .
*******************************************************************************/

/*
 * provides initial serial debug output
 *
 * Copyright (C) 2014  Johann Fischer <j.fischer@fh-bingen.de>
 */

#include <libopencm3/stm32/usart.h>
//#include <libopencm3/cm3/nvic.h>

#include "kernel.h"
#include "board_uart0.h"
#include "uart0.h"

/**
 * @file
 * @ingroup     stm32f0discovery
 *
 * @author      Johann Fischer
 * @version     $Revision$
 *
 * @note    $Id$
 */

//void usart1_isr(void)
//{
//	uint8_t a;
//
//	if (usart_get_intr_source(usart_intr_rxdrdy)) {
//		a = usart1_recv();
//#ifdef MODULE_UART0
//		if (uart0_handler_pid) {
//			uart0_handle_incoming(a);
//			uart0_notify_thread();
//		}
//
//#endif
//		return;
//	}
//}

static inline uint32_t uart0_puts(char *astring, uint32_t length)
{
	uint32_t i = 0;

	for (i = 0; i < length; i++) {
		usart_send_blocking(USART1, astring[i]); 
	}

	return i;
}

void stdio_flush(void)
{
}

int fw_puts(char *astring, int length)
{
	return uart0_puts(astring, length);
}

int bl_uart_init(void)
{
	/* Setup USART2 parameters. */
	usart_set_baudrate(USART1, 115200);
	usart_set_databits(USART1, 8);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_stopbits(USART1, USART_CR2_STOP_1_0BIT);
	usart_set_mode(USART1, USART_MODE_TX);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

	/* Finally enable the USART. */
	usart_enable(USART1);
	return 1;
}
