/*
 * Copyright (c) 2021, Maverick Embedded Technology Ltd
 * All rights reserved.
 *
 * Written for Maverick Embedded Technology Ltd by Steve C. Woodford.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Neither the names of the copyright holders nor the names of their
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/time.h>

#include "console.h"
#include "timer.h"

int _open(const char *, int, int);
int
_open(const char *fname, int flags, int mode)
{

	(void) fname;
	(void) flags;
	(void) mode;

	return -1;
}

int _close(int);
int
_close(int fd)
{

	(void) fd;

	return -1;
}

int _read(int, char *, int);
int
_read(int fd, char *ptr, int len)
{

	(void) fd;
	(void) ptr;
	(void) len;

	return 0;
}

int _write(int, char *, int);
int
_write(int fd, char *ptr, int len)
{
	int rv = len;

	(void) fd;

	console_write_stdout(ptr, len);

	return rv;
}

int _lseek(int, int, int);
int
_lseek(int fd, int ptr, int dir)
{

	(void) fd;
	(void) ptr;
	(void) dir;

	return 0;
}

int _fstat(int, struct stat *);
int
_fstat(int fd, struct stat *st)
{

	(void) fd;

	/*
	 * Return an error for std<in|out|err>. This helps avoid newlib
	 * allocating large buffers.
	 */
	if (fd < 3)
		return -1;

	st->st_mode = S_IFCHR;
	st->st_blksize = 64;
	return 0;
}

int _isatty(int);
int
_isatty(int fd)
{

	(void) fd;

	return 1;
}

int _getpid(void);
int
_getpid(void)
{

	return 1;
}

int _kill(int, int);
int
_kill(int pid, int sig)
{

	(void) pid;
	(void) sig;

	return 0;
}

void _exit(int);
void
_exit(int status)
{

	(void) status;

	for (;;);
}

int _gettimeofday(struct timeval *tv, void *tzp);
int
_gettimeofday(struct timeval *tv, void *tzp)
{

	if (tv != NULL) {
		tv->tv_sec = 0;
		tv->tv_usec = 0;
	}

	if (tzp != NULL) {
		struct timezone *tz = tzp;

		tz->tz_minuteswest = 0;
		tz->tz_dsttime = 0;
	}

	return 0;
}

caddr_t _sbrk(int);
caddr_t
_sbrk(int incr)
{
	extern char _end;
	extern char _ram_end_;
	static char *heap_end;
	char *prev_heap_end;

	if (heap_end == NULL)
		heap_end = &_end;

	prev_heap_end = heap_end;

	if ((heap_end + incr) >= &_ram_end_) {
		write(1, "Out of heap\n", 12);
		abort();
	}

	heap_end += incr;

	return (caddr_t)prev_heap_end;
}
