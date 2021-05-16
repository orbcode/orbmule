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

#include <assert.h>
#include <stdint.h>
#include <stdlib.h>

#include "ringbuff.h"
#include "soc.h"

struct ringbuff {
	uint8_t *rb_producer_p;
	ringbuff_cb_t rb_producer_cb;
	void *rb_producer_cookie;

	uint8_t *rb_consumer_p;
	ringbuff_cb_t rb_consumer_cb;
	void *rb_consumer_cookie;

	volatile ringbuff_len_t rb_count;

	uint8_t *rb_buff;
	ringbuff_len_t rb_size;
};

void
ringbuff_init(ringbuff_t rb)
{

	rb->rb_producer_p = rb->rb_buff;
	rb->rb_producer_cb = NULL;
	rb->rb_consumer_p = rb->rb_buff;
	rb->rb_consumer_cb = NULL;
	rb->rb_count = 0;
}

ringbuff_t
ringbuff_alloc(ringbuff_len_t size)
{
	ringbuff_t rb;

	assert(size >= 2);

	if ((rb = malloc(sizeof(*rb))) != NULL) {
		if ((rb->rb_buff = malloc(size)) == NULL) {
			free(rb);
			rb = NULL;
		} else {
			rb->rb_size = size;
			ringbuff_init(rb);
		}
	}

	return rb;
}

void
ringbuff_free(ringbuff_t rb)
{

	free(rb);
}

void
ringbuff_producer_init(ringbuff_t rb, ringbuff_cb_t cb, void *arg)
{

	rb->rb_producer_cookie = arg;
	rb->rb_producer_cb = cb;
}

void
ringbuff_consumer_init(ringbuff_t rb, ringbuff_cb_t cb, void *arg)
{

	rb->rb_consumer_cookie = arg;
	rb->rb_consumer_cb = cb;
}

ringbuff_len_t
ringbuff_get_count(ringbuff_t rb)
{

	return rb->rb_count;
}

ringbuff_len_t
ringbuff_get_space(ringbuff_t rb)
{

	return rb->rb_size - rb->rb_count;
}

uint8_t
ringbuff_is_full(ringbuff_t rb)
{

	return ringbuff_get_count(rb) == rb->rb_size;
}

uint8_t
ringbuff_is_empty(ringbuff_t rb)
{

	return ringbuff_get_count(rb) == 0;
}

void
ringbuff_produce(ringbuff_t rb, uint8_t d)
{
	uint32_t mask;

	if (ringbuff_is_full(rb))
		return;

	*rb->rb_producer_p++ = d;
	if (rb->rb_producer_p == &rb->rb_buff[rb->rb_size])
		rb->rb_producer_p = rb->rb_buff;

	mask = cpu_mask_interrupts();
	rb->rb_count++;
	cpu_restore_interrupts(mask);
}

void
ringbuff_produce_done(ringbuff_t rb)
{

	if (rb->rb_consumer_cb != NULL)
		(void)(rb->rb_consumer_cb)(rb, rb->rb_consumer_cookie);
}

uint8_t
ringbuff_consume(ringbuff_t rb)
{
	uint32_t mask;
	uint8_t rv;

	if (ringbuff_is_empty(rb))
		return 0;

	rv = *rb->rb_consumer_p++;
	if (rb->rb_consumer_p == &rb->rb_buff[rb->rb_size])
		rb->rb_consumer_p = rb->rb_buff;

	mask = cpu_mask_interrupts();
	rb->rb_count--;
	cpu_restore_interrupts(mask);

	return rv;
}

void
ringbuff_consume_done(ringbuff_t rb)
{

	if (rb->rb_producer_cb != NULL)
		(void)(rb->rb_producer_cb)(rb, rb->rb_producer_cookie);
}
