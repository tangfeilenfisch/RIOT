/**
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License. See the file LICENSE in the top level directory for more
 * details.
 *
 * @ingroup hwtimer
 * @ingroup stm32f0discovery
 * @{
 * @author  Johann Fischer <j.fischer@fh-bingen.de>
 * @file
 * @}
 */

#include <inttypes.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/rcc.h>
#include "hwtimer_arch.h"
#include "hwtimer_cpu.h"

/* High level interrupt handler */
static void (*int_handler)(int);

/* Called when systick fires */
void sys_tick_handler(void)
{
	int_handler(0);
}

void hwtimer_arch_enable_interrupt(void)
{
	systick_interrupt_enable();
	return;
}

void hwtimer_arch_disable_interrupt(void)
{
	systick_interrupt_disable();
	return;
}

void hwtimer_arch_unset(short timer)
{
	(void) timer;
	return;
}

void hwtimer_arch_set(uint32_t offset, short timer)
{
	(void) timer;
	(void) offset;

	/* FIXME: set reload value */
	systick_set_reload(rcc_core_frequency / 16000);
	//set(get_counter() + offset);
	systick_counter_enable();
	systick_interrupt_enable();
	return;
}

void hwtimer_arch_set_absolute(uint32_t value, short timer)
{
	(void) timer;
	(void) value;
	
	/* FIXME: set reload value */
	systick_set_reload(rcc_core_frequency / 16000);
	systick_counter_enable();
	systick_interrupt_enable();
	return;
}

uint32_t hwtimer_arch_now(void)
{
	/* FIXME: get systick value */
	return 0;
}

void hwtimer_arch_init(void (*handler)(int), uint32_t fcpu)
{
	(void) fcpu;

	int_handler = handler;
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
	STK_CVR=0;
	
	systick_set_reload(rcc_core_frequency / 16000);
	return;
}
