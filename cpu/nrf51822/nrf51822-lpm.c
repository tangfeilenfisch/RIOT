/**
 * LPM for NRF51822
 *
 * Copyright (C) 2014 Johann Fischer <j.fischer@fh-bingen.de>
 *
 * This file subject to the terms and conditions of the GNU Lesser General
 * Public License. See the file LICENSE in the top level directory for more
 * details.
 *
 * @file   nrf51822-lpm.c
 * @author Johann Fischer <j.fischer@fh-bingen.de>
 */

#include <libopencm3/nrf51/pwr.h>
#include "lpm.h"

static enum lpm_mode lpm = LPM_ON;

enum lpm_mode lpm_set(enum lpm_mode target)
{
	enum lpm_mode last_lpm = lpm;

	switch (target) {
	case LPM_ON:
		/* system on, cpu active, constant latency mode */
		pwr_set_constlat_mode();
		break;
	case LPM_IDLE:
		/* system on, cpu sleeps(wfe), constant latency mode */
		pwr_set_constlat_mode();
		asm("wfe");
		break;
	case LPM_SLEEP:
		/* system on, cpu sleeps(wfi), constant latency mode */
		pwr_set_constlat_mode();
		asm("wfi");
		break;
	case LPM_POWERDOWN:
		/* system on, cpu sleeps(wfi), low power mode */
		pwr_set_lowpwr_mode();
		asm("wfi");
		pwr_set_constlat_mode();
		break;
	case LPM_OFF:
		/* system off, ram on */
		pwr_system_off();
		break;
	default:
		break;
	}

	lpm = target;
	return last_lpm;
}

enum lpm_mode lpm_get(void)
{
	return lpm;
}

void lpm_init(void)
{
	lpm = LPM_ON;
}
