/**
 * Syscall implementation for stm32f0discovery
 *
 * Copyright (C) 2013 Oliver Hahm <oliver.hahm@inria.fr>
 *
 * This file subject to the terms and conditions of the GNU Lesser General
 * Public License. See the file LICENSE in the top level directory for more
 * details.
 *
 * @ingroup stm32f0discovery
 * @{
 * @file   syscalls.c
 * @author Oliver Hahm <oliver.hahm@inria.fr>
 * @author Johann Fischer <j.fischer@fh-bingen.de>
 * @}
 */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/unistd.h>
#include <stdint.h>
#include <sys/time.h>

#include "kernel.h"
#include "irq.h"
#include "io.h"
#if defined MODULE_RTC
#include "rtc.h"
#elif defined MODULE_VTIMER
#include "vtimer.h"
#endif

/**
 * @name Heaps (defined in linker script)
 * @{
 */

#define DEBUG_SYSCALLS			0
#if DEBUG_SYSCALLS
#define	PRINTF(...)			printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

extern uintptr_t __heap_start;		///< start of heap memory space
extern uintptr_t __heap_max;		///< maximum for end of heap memory space

/// current position in heap
static caddr_t heap = {(caddr_t)&__heap_start};
/// maximum position in heap
static const caddr_t heap_max = {(caddr_t)&__heap_max};
// start position in heap
static const caddr_t heap_start = {(caddr_t)&__heap_start};
// current heap in use

/** @} */

/*-----------------------------------------------------------------------------------*/
void heap_stats(void) {
	printf("# heap : %p -- %p -> %p (%li of %li free)\n",
			heap_start, heap, heap_max,
			(uint32_t)heap_max - (uint32_t)heap,
			(uint32_t)heap_max - (uint32_t)heap_start);
}
/*-----------------------------------------------------------------------------------*/
caddr_t _sbrk_r(struct _reent *r, size_t incr)
{
	uint32_t cpsr = disableIRQ();

	caddr_t new_heap = heap + incr;

	/* check the heap for a chunk of the requested size */
	if( new_heap <= heap_max ) {
		caddr_t prev_heap = heap;
		heap = new_heap;

		r->_errno = 0;
		restoreIRQ(cpsr);
		return prev_heap;
	}

	restoreIRQ(cpsr);
	r->_errno = ENOMEM;
	return NULL;
}
/*-----------------------------------------------------------------------------------*/
void __assert_func(const char *file, int line, const char *func, const char *failedexpr)
{
	printf("#! assertion %s failed\n\t%s() in %s:%u\n", failedexpr, func, file, line );
	_exit(3);
}
/*-----------------------------------------------------------------------------------*/
void __assert(const char *file, int line, const char *failedexpr)
{
	__assert_func(file, line, "?", failedexpr);
}
/*---------------------------------------------------------------------------*/
int _isatty_r(struct _reent *r, int fd)
{
	r->_errno = 0;
	if( fd == STDOUT_FILENO || fd == STDERR_FILENO )
		return 1;
	else
		return 0;
}
/*---------------------------------------------------------------------------*/
_off_t _lseek_r(struct _reent *r, int fd, _off_t pos, int whence)
{
	(void) fd;
	(void) pos;
	(void) whence;
	_off_t result = -1;
	PRINTF("lseek [%i] pos %li whence %i\n", fd, pos, whence);

	r->_errno = ENODEV;

	PRINTF("lseek returned %li (0 is success)\n", result);
	return result;
}
/*---------------------------------------------------------------------------*/
int _open_r(struct _reent *r, const char *name, int mode)
{
	(void) name;
	(void) mode;
	int ret = -1;
	PRINTF("open '%s' mode %#x\n", name, mode);

	r->_errno = ENODEV; // no such device

	PRINTF("open [%i] errno %i\n", ret, r->_errno);
	return ret;
}
/*---------------------------------------------------------------------------*/
int _stat_r(struct _reent *r, char *name, struct stat *st)
{
	(void) name;
	(void) st;
	int ret = -1;
	PRINTF("_stat_r '%s' \n", name);
	r->_errno = ENODEV; // no such device
	PRINTF("_stat_r [%i] errno %i\n", ret, r->_errno);
	return ret;
}
/*---------------------------------------------------------------------------*/
int _fstat_r(struct _reent *r, int fd, struct stat * st)
{
	int ret = -1;

	r->_errno = 0;
	memset(st, 0, sizeof(*st));
	if (fd == STDOUT_FILENO || fd == STDERR_FILENO) {
		st->st_mode = S_IFCHR;
		ret = 0;
	}
	else {
		r->_errno = ENODEV;
	}
	return ret;
}
/*---------------------------------------------------------------------------*/
int _write_r(struct _reent *r, int fd, const void *data, unsigned int count)
{
	int result = EOF;
	r->_errno = EBADF;

	switch(fd) {
	case STDOUT_FILENO:
	case STDERR_FILENO:
		result = fw_puts((char *)data, count);
		break;
	default:
		PRINTF("write [%i] data @%p count %i\n", fd, data, count);

		PRINTF("write [%i] returned %i errno %i\n", fd, result, r->_errno);
		break;
	}

	return result;
}
/*---------------------------------------------------------------------------*/
int _read_r(struct _reent *r, int fd, void *buffer, unsigned int count)
{
	(void) fd;
	(void) buffer;
	(void) count;
	int result = -1;
	r->_errno = EBADF;
	PRINTF("read [%i] buffer @%p count %i\n", fd, buffer, count);
	PRINTF("read [%i] returned %i\n", fd, result);
	return result;
}
/*---------------------------------------------------------------------------*/
int _close_r(struct _reent *r, int fd)
{
	(void) fd;
	int ret = -1;
	r->_errno = EBADF;
	PRINTF("close [%i]\n", fd);
	PRINTF("close returned %i errno %i\n", result, errno);
	return ret;
}
/*---------------------------------------------------------------------------*/
int _unlink_r(struct _reent *r, char* path)
{
	(void) path;
	int ret = -1;
	r->_errno = ENODEV;
	PRINTF("unlink '%s'\n", path);
	PRINTF("unlink returned %i errno %i\n", result, errno);
	return ret;
}
/*---------------------------------------------------------------------------*/
void _exit(int n)
{
	printf("#! exit %i: resetting\n", n);
	scb_reset_system();
	while(1);
}
/*---------------------------------------------------------------------------*/
int _getpid(void)
{
	return sched_active_thread->pid;
}
/*---------------------------------------------------------------------------*/
int _kill_r(struct _reent *r, int pid, int sig)
{
	(void) pid;
	(void) sig;
	/* not implemented */
	r->_errno = ESRCH;		// no such process
	return -1;
}
/*---------------------------------------------------------------------------*/
int _gettimeofday(struct timeval *tp, void *restrict tzp)
{
	(void) tzp;
#if defined MODULE_RTC
	rtc_time(tp);
#elif defined MODULE_VTIMER
	vtimer_gettimeofday(tp);
#else
#warning gettimeofday syscall is not implemented without vtimer or rtc module
#endif
	return 0;
}

void _init(void){}
void _fini(void){}
