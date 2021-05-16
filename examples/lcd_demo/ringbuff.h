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

#ifndef RINGBUFF_H
#define RINGBUFF_H

#include <stdint.h>

typedef uint32_t ringbuff_len_t;

struct ringbuff;
typedef struct ringbuff *ringbuff_t;
typedef void (*ringbuff_cb_t)(ringbuff_t, void *);

extern ringbuff_t	ringbuff_alloc(ringbuff_len_t);
extern void		ringbuff_free(ringbuff_t);
extern void		ringbuff_init(ringbuff_t);

extern ringbuff_len_t	ringbuff_get_count(ringbuff_t);
extern ringbuff_len_t	ringbuff_get_space(ringbuff_t);
extern uint8_t		ringbuff_is_full(ringbuff_t);
extern uint8_t		ringbuff_is_empty(ringbuff_t);

extern void		ringbuff_producer_init(ringbuff_t, ringbuff_cb_t,
			    void *);
extern void		ringbuff_produce(ringbuff_t, uint8_t);
extern void		ringbuff_produce_done(ringbuff_t);

extern void		ringbuff_consumer_init(ringbuff_t, ringbuff_cb_t,
			    void *);
extern uint8_t		ringbuff_consume(ringbuff_t);
extern void 		ringbuff_consume_done(ringbuff_t);

#endif /* RINGBUFF_H */
