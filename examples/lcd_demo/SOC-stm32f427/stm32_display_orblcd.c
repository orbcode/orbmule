/*
 * Copyright (c) 2022, Maverick Embedded Technology Ltd
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

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "soc.h"
#include "display_orblcd.h"

static bool
stm32_orblcd_send(void *arg, bool is_cmd, uint32_t v)
{
	uint32_t channel = is_cmd ? LCD_COMMAND_CHANNEL : LCD_DATA_CHANNEL;
	bool rv = false;

	(void)arg;

	if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) != 0 &&
	    (ITM->TCR & ITM_TCR_ITMENA_Msk) != 0 &&
	    (ITM->TER & (1u << channel)) != 0) {
		while (ITM->PORT[channel].u32 == 0)
			;
		ITM->PORT[channel].u32 = v;
		rv = true;
	}

	return rv;
}

void
soc_display_init(void)
{

	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	ITM->TER |= (1u << LCD_DATA_CHANNEL) | (1u << LCD_COMMAND_CHANNEL);

	display_orblcd_attach(stm32_orblcd_send, NULL);
}
