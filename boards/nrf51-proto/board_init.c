/**
 * Board specific functions for the RIOT on NRF51822
 *
 * Copyright (C) 2014 Johann Fischer <j.fischer@fh-bingen.de>
 *
 * This file subject to the terms and conditions of the GNU Lesser General
 * Public License. See the file LICENSE in the top level directory for more
 * details.
 *
 * @file   cpu.c
 * @author Johann Fischer <j.fischer@fh-bingen.de>
 */

#include <libopencm3/nrf51/gpio.h>
#include <libopencm3/nrf51/rcc.h>
#include "board.h"


static void gpio_setup(void)
{
	gpio_mode_setup(PIN_DEBUG1 | PIN_DEBUG2 | PIN_DEBUG3
			| PIN_DEBUG4 | PIN_DEBUG5, GPIO_MODE_OUTPUT,
			GPIO_PUPD_NONE, S0S1, GPIO_SENSE_DISABLED);
}

void board_init(void)
{
	extern void bl_uart_init(void);
	rcc_sysclk_config(F_16MHZ, RC, true);
	gpio_setup();
	bl_uart_init();
}
