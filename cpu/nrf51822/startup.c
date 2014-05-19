/**
 * @ingroup nrf51822
 * @{
 */

/**
 * @file
 * @brief       nrf51822 startup code
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
