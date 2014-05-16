/**
 * @ingroup stm32f0discovery
 * @{
 */

/**
 * @file
 * @brief       stm32f0discovery startup code
 *
 * @author      Johann Fischer <j.fischer@fh-bingen.de>
 *
 */

#include "kernel_internal.h"
#include "cpu.h"

void libopencm3_os_spec_init(void)
{
	board_init();
	kernel_init();
}
