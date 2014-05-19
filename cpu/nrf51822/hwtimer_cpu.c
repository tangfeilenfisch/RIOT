/**
 * hwtimer specific functions for the RIOT on NRF51822
 *
 * Copyright (C) 2014 Johann Fischer <j.fischer@fh-bingen.de>
 *
 * This file subject to the terms and conditions of the GNU Lesser General
 * Public License. See the file LICENSE in the top level directory for more
 * details.
 *
 * @file   hwtimer_cpu.c
 * @author Johann Fischer <j.fischer@fh-bingen.de>
 */

#include <inttypes.h>
#include <libopencm3/nrf51/rtc.h>
#include <libopencm3/cm3/nvic.h>
#include "hwtimer_arch.h"
#include "hwtimer_cpu.h"

/* High level interrupt handler */
static void (*int_handler)(int);

void rtc0_isr(void)
{
	rtc0_clear_event(rtc_compare0);
	int_handler(0);
}

void hwtimer_arch_enable_interrupt(void)
{
	nvic_enable_irq(NVIC_RTC0_IRQ);
	return;
}

void hwtimer_arch_disable_interrupt(void)
{
	rtc0_disable_intr(rtc_tick);
	return;
}

void hwtimer_arch_unset(short timer)
{
	rtc0_disable_intr(rtc_compare0);
	return;
}

void hwtimer_arch_set(uint32_t offset, short timer)
{
	(void) timer;

	rtc0_stop();
	rtc0_disable_intr(rtc_compare0);
	rtc0_clear_event(rtc_compare0);
	rtc0_set_cc0(rtc0_get_counter() + offset);
	rtc0_enable_intr(rtc_compare0);
	rtc0_start();
	return;
}

void hwtimer_arch_set_absolute(uint32_t value, short timer)
{
	(void) timer;

	rtc0_stop();
	rtc0_disable_intr(rtc_compare0);
	rtc0_clear_event(rtc_compare0);
	rtc0_set_cc0(value);
	rtc0_enable_intr(rtc_compare0);
	rtc0_start();
	return;
}

uint32_t hwtimer_arch_now(void)
{
	return rtc0_get_counter();
}

void hwtimer_arch_init(void (*handler)(int), uint32_t fcpu)
{
	(void) fcpu;

	int_handler = handler;
	rtc0_set_prescaler(HWTIMER_NRF51822_RTC0_PRSCL);
	rtc0_clear();
	return;
}
