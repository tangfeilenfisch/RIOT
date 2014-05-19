/**
 * CPU specific functions for the RIOT scheduler on NRF51822
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

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/cortex.h>

#include "cpu.h"
#include "kernel.h"
#include "sched.h"
#include "thread.h"

#define STACK_MARKER			0x77777777

extern void sched_task_exit(void);

uint32_t inISR(void)
{
	return cm_is_in_interrupt();
}

uint32_t disableIRQ(void)
{
	uint32_t state = cm_is_masked_interrupts();
	cm_disable_interrupts();
	return state;
}

uint32_t enableIRQ(void)
{
	uint32_t state = cm_is_masked_interrupts();
	cm_enable_interrupts();
	return state;
}

void restoreIRQ(uint32_t state)
{
	cm_mask_interrupts(state);
}

void dINT(void)
{
	cm_disable_interrupts();
}

void eINT(void)
{
	cm_enable_interrupts();
}

inline void thread_yield(void)
{
	asm("svc 0x0\n");
}

void cpu_switch_context_exit(void)
{
	cm_enable_interrupts();
	cm0_switch_to_msp();
	cm0_sign_psp();
	asm("svc 42\n");
	cm_disable_interrupts();
}

void __attribute__ ((naked)) sv_call_handler(void)
{
	cm0_switch_context_psp_use();
}

char* thread_stack_init(void *task_func, void *stack_start, int stack_size )
{
	uint32_t *stk;
	stk = (uint32_t*)((uint32_t)stack_start + stack_size);

	stk--;
	*stk = STACK_MARKER;

	/* xPSR register */
	stk--;
	*stk = (uint32_t)0x01000000;

	/* PC reg, set the entry point */
	stk--;
	*stk = (uint32_t)task_func;

	/* LR reg */
	stk--;
	*stk = (uint32_t)sched_task_exit;

	/* R12 */
	stk--;
	*stk = (uint32_t)0;

	/* R3 .. R0 */
	for (int i = 3; i >= 0; i--) {
		stk--;
		*stk = i;
	}
	/* R7 .. R4 */
	for (int i = 7; i >= 4; i--) {
		stk--;
		*stk = i;
	}

	return (char*)stk;
}

int reboot_arch(int mode)
{
	(void) mode;
	scb_reset_system();
	return -1;
}
