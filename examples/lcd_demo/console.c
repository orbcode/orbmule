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
#include <stdio.h>

#include "ringbuff.h"
#include "console.h"
#include "timer.h"
#include "soc.h"

/* Define this to use SWO rather than a platform-specific UART. */
#undef USE_SWO
#define	SWO_SPEED	2250000u
#define	SWO_PORT	0

#ifdef USE_SWO
static void
swo_init(void)
{

	/*
	 * "Selected PIN Protocol Register": Select which protocol to use
	 * for trace output (2: SWO NRZ, 1: SWO Manchester encoding)
	 */
	// 0xE00400F0 = 2
	TPI->SPPR = 2;

	/*
	 * Bypass the TPIU formatter
	 */
	// 0xE0040304 = 0
	TPI->FFCR = 0;

	/*
	 * "Async Clock Prescaler Register". Scale the baud rate of the
	 * asynchronous output
	 */
	// 0xE0040010 = 53
	TPI->ACPR = (F_CPU / SWO_SPEED) - 1;

	/* enable trace in core debug */
	// 0xE000EDFC |= 1 << 24
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

	/* DWT_CTRL */
	// 0xE0001000 = 0x400003FE
	DWT->CTRL = 0x400003FE;

	/*
	 * ITM Lock Access Register, C5ACCE55 enables more write access to
	 * Control Register 0xE00 :: 0xFFC
	 */
	// 0xE0000FB0
	ITM->LAR = 0xC5ACCE55;

	/* ITM Trace Control Register. XXX:  Validate with Marple's code */
	// 0xE0000E80 = 0x007f0015
	ITM->TCR = ITM_TCR_TraceBusID_Msk | ITM_TCR_SWOENA_Msk |
	    ITM_TCR_SYNCENA_Msk | ITM_TCR_ITMENA_Msk;

	/* ITM Trace Privilege Register */
	// 0xE0000E40
	ITM->TPR = ITM_TPR_PRIVMASK_Msk;

	/*
	 * ITM Trace Enable Register. Enabled tracing on stimulus ports.
	 * One bit per stimulus port.
	 */
	// 0xE0000E00
	ITM->TER = 1u << SWO_PORT;
}

static __always_inline void
ITM_SendCharX(uint32_t ch)
{

#undef PORT
	if ((ITM->TCR & ITM_TCR_ITMENA_Msk) &&	/* ITM enabled */
	    (ITM->TER & (1UL << SWO_PORT))) {		/* ITM Port #0 enabled */
		while (ITM->PORT[SWO_PORT].u32 == 0)
			;
		ITM->PORT[SWO_PORT].u8 = (uint8_t) ch;
	}
}
#else  /* USE_SWO */

static ringbuff_t console_rb;

static void
cons_out(ringbuff_t rb, char ch)
{

	if (ringbuff_is_full(rb)) {
		/*
		 * If interrupts are enabled, hang around a short time until
		 * some space has opened up in the ring buffer.
		 */
		if (cpu_interrupts_enabled()) {
			timer_timeout_t to;
			timer_timeout_start(&to, 10);
			while (timer_timeout_expired(&to) == 0 &&
			    ringbuff_is_full(rb)) {
				/* Wait */
			}

			/* If still full, drop the character. */
			if (ringbuff_is_full(rb))
				return;
		} else {
			/* Drop the character - the ringbuff won't drain. */
			return;
		}
	}

	ringbuff_produce(rb, (uint8_t)ch);
	ringbuff_produce_done(rb);
}
#endif /* USE_SWO */

static void
cons_putchar(char c)
{

	if (c == '\n')
		(void) cons_putchar('\r');

#ifndef USE_SWO
	cons_out(console_rb, c);
#else
	ITM_SendCharX(c);
#endif
}

void
console_write_stdout(const char *buf, int len)
{

	while (len--)
		cons_putchar(*buf++);
}

void
console_init(ringbuff_t rb)
{

	/*
	 * We want to minimise the size of buffers used by stdio's buffered
	 * I/O, but at the moment newlib's stdio file pointers are pointing
	 * at __sf_fake_std<in|out|err> structures (in Flash!) so we can't set
	 * fp->_flags.__SMBF until the real stdio structures are configured.
	 *
	 * By default stdio will allocate 1 KB per buffer. But by setting the
	 * __SMBF flag *before* performing any actual I/O, we can force the
	 * library routines to use 64-byte buffers. Note that this also relies
	 * on our _fstat() stub in newlib_stubs.c returning -1 for all the
	 * std<in|out|err> file descriptors.
	 *
	 * Fortunately we can get things initialised without causing I/O by
	 * invoking the function version of feof() for each file pointer.
	 *
	 * Note that feof() is a pre-processor macro defined in stdio.h but
	 * there is also a function version available in the library.
	 */
#ifdef feof
#undef feof
#endif
	(void) feof(stdout);
	stdout->_flags |= __SMBF;

	(void) feof(stderr);
	stderr->_flags |= __SMBF;

	(void) feof(stdin);
	stdin->_flags |= __SMBF;

#ifdef USE_SWO
	(void) rb;
	/* XXX: Why does this need to be done twice on some SoCs? */
	swo_init();
	swo_init();
#else
	console_rb = rb;
#endif
}
