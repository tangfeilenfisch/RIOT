#ifndef CPU_H
#define CPU_H

/**
 * @defgroup
 * @ingroup		cpu
 * @{
 */

extern void dINT(void);
extern void eINT(void);

/**
 * @brief Save the thread's context
 */
void save_context(void);

/**
 * @brief Restores the before saved context of a thread
 */
void restore_context(void);

/**
 * @brief Let the thread yield
 */
void thread_yield(void);

/** @} */

#endif /* CPU_H */
