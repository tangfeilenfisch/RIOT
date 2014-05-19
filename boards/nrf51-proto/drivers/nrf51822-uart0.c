/**
 * CPU specific functions for the RIOT scheduler on NRF51822
 *
 * Copyright (C) 2014 Johann Fischer <j.fischer@fh-bingen.de>
 *
 * This file subject to the terms and conditions of the GNU Lesser General
 * Public License. See the file LICENSE in the top level directory for more
 * details.
 *
 * @file   nrf51822-uart0.c
 * @author Johann Fischer <j.fischer@fh-bingen.de>
 */

#include <libopencm3/nrf51/gpio.h>
#include <libopencm3/nrf51/uart.h>
#include <libopencm3/cm3/nvic.h>

#include "kernel.h"
#include "board_uart0.h"
#include "uart0.h"

void uart_isr(void)
{
	uint8_t a;

	if (uart_get_intr_source(uart_intr_rxdrdy)) {
		uart_clear_event(uart_intr_rxdrdy);
		a = uart_recv();
#ifdef MODULE_UART0
		if (uart0_handler_pid) {
			uart0_handle_incoming(a);
			uart0_notify_thread();
		}

#endif
		return;
	}
	if (uart_get_intr_source(uart_intr_error)) {
		uart_clear_event(uart_intr_error);
		uart_clear_error(UART_ERRORSRC_ALL);
		return;
	}
}

static inline uint32_t uart0_puts(char *astring, uint32_t length)
{
	uint32_t i = 0;

	for (i = 0; i < length; i++) {
		uart_send(astring[i]);
		uart_wait_send_ready();
		uart_clear_event(uart_intr_txdrdy);
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

/* FIXME: it appears to be awkwardly placed */
int bl_uart_init(void)
{
	uart_pmux(UART_PSEL_OFF, PIN_UART_TX, UART_PSEL_OFF, PIN_UART_RX);
	uart_config(baudrate_115200, pbit_off_hwflow_off);
	uart_enable_intr(uart_intr_rxdrdy);
	uart_enable_intr(uart_intr_error);
	nvic_enable_irq(NVIC_UART_IRQ);
	return 1;
}
