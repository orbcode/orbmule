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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "soc.h"

static uint32_t
read_rng(void)
{
	static uint32_t previous_value;
	uint32_t rv;

	do {
		while ((RNG->SR & RNG_SR_DRDY) == 0)
			;

		rv = RNG->DR;
	} while (rv == previous_value);

	previous_value = rv;
	return rv;
}

/*
 * Low-level entropy gathering, for libc's arc4random(3) API.
 */
int
getentropy(void *buf, size_t buflen)
{
	uint32_t v;
	size_t c;

	if ((RNG->CR & RNG_CR_RNGEN) == 0) {
		RCC->AHB2ENR |= RCC_AHB2ENR_RNGEN;
		__DSB();
		__ISB();
		(void)RCC->AHB2ENR;
		RNG->CR = RNG_CR_RNGEN;
		__DSB();
		__ISB();
		(void) RNG->CR;
		(void) read_rng();
	}

	while (buflen) {
		v = read_rng();

		if (buflen >= sizeof(v))
			c = sizeof(v);
		else
			c = buflen;

		memcpy(buf, &v, c);
		buf = (void *)((uintptr_t)buf + c);
		buflen -= c;
	}

	return 0;
}
