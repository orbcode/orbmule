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

#include <stdint.h>

#include "timer.h"
#include "soc.h"

static volatile timer_timeout_t timer_tick_counter;

void
timer_timeout_start(timer_timeout_t *tp, uint32_t ms)
{
	timer_timeout_t cur_ticks;
	uint32_t mask;

	mask = cpu_mask_interrupts();
	cur_ticks = timer_tick_counter;
	cpu_restore_interrupts(mask);

	*tp = cur_ticks + ms;
}

int
timer_timeout_expired(const timer_timeout_t *tp)
{
	timer_timeout_t cur_ticks;
	uint32_t mask;

	mask = cpu_mask_interrupts();
	cur_ticks = timer_tick_counter;
	cpu_restore_interrupts(mask);

	return cur_ticks >= *tp;
}

uint32_t
timer_timeout_remaining(const timer_timeout_t *tp)
{
	timer_timeout_t cur_ticks;
	uint32_t mask;

	mask = cpu_mask_interrupts();
	cur_ticks = timer_tick_counter;
	cpu_restore_interrupts(mask);

	if (cur_ticks >= *tp)
		return 0ul;

	return (uint32_t)(*tp - cur_ticks);
}

void
timer_tick(void)
{
	uint32_t mask;

	mask = cpu_mask_interrupts();
	timer_tick_counter++;
	cpu_restore_interrupts(mask);
}
