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

#ifndef CPU_CORTEX_H
#define CPU_CORTEX_H

#include <stdbool.h>
#include <stdint.h>
#include "soc.h"

typedef void (*cortexm_exception_handler_t)(void);

extern void cpu_install_irq_handler(IRQn_Type, cortexm_exception_handler_t);

static __inline uint32_t
cpu_mask_interrupts(void)
{
	uint32_t old_primask = __get_PRIMASK();
	__disable_irq();
	return old_primask;
}

static __inline void
cpu_restore_interrupts(uint32_t old)
{

	__set_PRIMASK(old);
}

static __inline bool
cpu_interrupts_enabled(void)
{

	return __get_PRIMASK() == 0;
}

static __inline void
cpu_buzz_delay_cycles(uint32_t cycles)
{

	__asm __volatile(
	    "1: subs    %0, %0, #1\n"
	    "   bgt.n   1b\n"
	    : "+l" (cycles) : : "cc");
}

static __inline void
cpu_buzz_delay_us(uint32_t us)
{

	/* Assume best case of 3 cycles per loop. */
	cpu_buzz_delay_cycles(us * (F_CPU / 3000000u));
}

#endif /* CPU_CORTEX_H */
