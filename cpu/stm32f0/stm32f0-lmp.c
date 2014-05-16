/**
 * LPM for stm32f0discovery
 *
 * This file subject to the terms and conditions of the GNU Lesser General
 * Public License. See the file LICENSE in the top level directory for more
 * details.
 *
 * @ingroup
 * @{
 * @file   
 * @author Johann Fischer <j.fischer@fh-bingen.de>
 * @}
 */

#include <libopencm3/stm32/rcc.h>
#include "lpm.h"

static enum lpm_mode lpm = LPM_ON;

enum lpm_mode lpm_set(enum lpm_mode target)
{
	enum lpm_mode last_lpm = lpm;

	switch (target) {
	case LPM_ON:
		break;
	case LPM_IDLE:
		asm("wfe");
		break;
	case LPM_SLEEP:
		asm("wfi");
		break;
	case LPM_POWERDOWN:
		asm("wfi");
		break;
	case LPM_OFF:
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
