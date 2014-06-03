/**
 * @ingroup pthread
 * @{
 * @file
 * @brief   Singletons features / single-shot execution.
 * @note    Do not include this header file directly, but pthread.h.
 */

#ifndef __SYS__POSIX__PTHREAD_ONCE__H
#define __SYS__POSIX__PTHREAD_ONCE__H

/**
 * @brief           Datatype to supply to pthread_once().
 */
typedef volatile int pthread_once_t;

/**
 * @def             PTHREAD_ONCE_INIT
 * @brief           Initialization for pthread_once_t.
 * @details         A zeroed out pthread_once_t is initialized.
 */
#define PTHREAD_ONCE_INIT 0

/**
 * @brief           Helper function that ensures that `init_routine` is called at once.
 * @details         Calling pthread_once() changes `once_control`.
 *                  For the same `once_control` the supplied `init_routine` won't get called again,
 *                  unless `once_control` is reset to #PTHREAD_ONCE_INIT or zeroed out.
 * @param[in,out]   once_control   Flag to ensure that the `init_routine` is called only once.
 * @param[in]       init_routine   Function to call if `once_control` was not used, yet.
 * @returns         0, this invocation cannot fail.
 */
int pthread_once(pthread_once_t *once_control, void (*init_routine)(void));

#endif

/**
 * @}
 */
