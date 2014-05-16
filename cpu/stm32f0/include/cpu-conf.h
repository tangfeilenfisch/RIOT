#ifndef CPU_CONF_H
#define CPU_CONF_H

/**
 * @name Kernel configuration
 * @{
 */
#define F_CPU					16000000
#define KERNEL_CONF_STACKSIZE_PRINTF		256

#ifndef KERNEL_CONF_STACKSIZE_DEFAULT
#define KERNEL_CONF_STACKSIZE_DEFAULT		512
#endif

#define KERNEL_CONF_STACKSIZE_IDLE		128

#ifdef UART0_BUFSIZE
#undef UART0_BUFSIZE
#endif
#define UART0_BUFSIZE				128
/** @} */

#endif /* CPU_CONF_H */
