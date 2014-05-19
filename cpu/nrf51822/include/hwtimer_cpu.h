#ifndef HWTIMER_CPU_H_
#define HWTIMER_CPU_H_

#define HWTIMER_MAXTIMERS		1
/* Set HWTIMER_SPEED to number of ticks
 * per second for the current architecture.
 */
#define HWTIMER_SPEED			32768
#define HWTIMER_NRF51822_RTC0_PRSCL	0
#define HWTIMER_MAXTICKS		(0x00FFFFFF)

#endif /* HWTIMER_CPU_H_ */
