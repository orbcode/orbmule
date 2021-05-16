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
#include <string.h>

#include "display.h"

struct squares_state {
	struct display_gizmo ss_gizmo;
	display_port_t ss_dp;
	uint16_t ss_width;
	uint16_t ss_height;
	display_fp_t ss_loop_acc;
	display_fp_t ss_loop_inc;
	uint16_t ss_loop;
	uint16_t ss_max_loop;
	uint16_t ss_direction;
	uint16_t ss_colour_dir;
	uint16_t ss_reverser;
	display_fp_t ss_red;
	display_fp_t ss_red_inc;
	display_fp_t ss_green;
	display_fp_t ss_green_inc;
	display_fp_t ss_blue;
	display_fp_t ss_blue_inc;
};

static uint8_t
squares_colour(struct squares_state *ss, display_fp_t *deg, display_fp_t inc)
{
	const struct display_sin_cos *dsc;
	display_fp_t f;
	display_fp_t d;
	int rv;

	(void) ss;

	d = *deg;
	dsc = display_sin_cos(display_fp_to_int(d));

	d = display_fp_add(d, inc);
	if (d >= DISPLAY_FP_CONST(360.0))
		d = display_fp_sub(d, DISPLAY_FP_CONST(360.0));
	else
	if (d < DISPLAY_FP_CONST(0.0))
		d = display_fp_add(d, DISPLAY_FP_CONST(360.0));
	*deg = d;

	f = display_fp_mul(DISPLAY_FP_CONST(128.0), dsc->cos);
	rv = display_fp_to_int(f);
	rv += 128;
	if (rv > 255)
		rv = 255;
	else
	if (rv < 0)
		rv = 0;

	return (uint8_t)rv;
}

static void
squares_loop(display_gizmo_t dg)
{
	struct squares_state *ss = (struct squares_state *)dg;
	uint8_t red, green, blue;

	ss->ss_loop_acc = display_fp_add(ss->ss_loop_acc, ss->ss_loop_inc);
	if (ss->ss_loop_acc < DISPLAY_FP_CONST(1.0))
		return;
	ss->ss_loop_acc = display_fp_sub(ss->ss_loop_acc,DISPLAY_FP_CONST(1.0));

	red = squares_colour(ss, &ss->ss_red, ss->ss_red_inc);
	green = squares_colour(ss, &ss->ss_green, ss->ss_green_inc);
	blue = squares_colour(ss, &ss->ss_blue, ss->ss_blue_inc);

	display_set_colour(ss->ss_dp, red, green, blue);

	display_box(ss->ss_dp, ss->ss_loop, ss->ss_loop,
	    ss->ss_width - (ss->ss_loop * 2),
	    ss->ss_height - (ss->ss_loop * 2));

	if (ss->ss_direction) {
		if (ss->ss_loop-- == 0) {
			if (ss->ss_reverser) {
				ss->ss_loop = 1;
				ss->ss_direction = 1 - ss->ss_direction;
			} else {
				ss->ss_loop = ss->ss_max_loop;
			}
		}
	} else
	if (ss->ss_loop++ == ss->ss_max_loop) {
		if (ss->ss_reverser) {
			ss->ss_loop = ss->ss_max_loop - 1;
			ss->ss_direction = 1 - ss->ss_direction;
		} else {
			ss->ss_loop = 0;
		}
	}
}

static void
squares_stop(display_gizmo_t dg)
{
	struct squares_state *ss = (struct squares_state *)dg;

	display_set_colour(ss->ss_dp, 0, 0, 0);

	for (unsigned int l = 0; l <= ss->ss_max_loop; l++) {
		display_box(ss->ss_dp, l, l, ss->ss_width - (l * 2),
		    ss->ss_height - (l * 2));
	}

	free(ss);
}

display_gizmo_t
display_gizmo_squares_start(display_port_t dp, int width, int height)
{
	const struct display_sin_cos *dsc;
	struct squares_state *ss;
	uint16_t deg;

	if ((ss = malloc(sizeof(*ss))) == NULL)
		return NULL;

	ss->ss_dp = dp;
	ss->ss_gizmo.dg_stop = squares_stop;
	ss->ss_gizmo.dg_loop = squares_loop;
	ss->ss_width = width;
	ss->ss_height = height;

	deg = (uint16_t)arc4random_uniform(360);
	ss->ss_reverser = deg & 2;
	ss->ss_direction = deg & 1;
	ss->ss_colour_dir = deg & 3;

	ss->ss_red = display_fp_from_int(deg);
	deg += 120;
	if (deg >= 360)
		deg -= 360;
	ss->ss_green = display_fp_from_int(deg);
	deg += 120;
	if (deg >= 360)
		deg -= 360;
	ss->ss_blue = display_fp_from_int(deg);

	dsc = display_sin_cos(arc4random_uniform(360));
	ss->ss_red_inc = display_fp_mul(dsc->cos, DISPLAY_FP_CONST(2.0));

	dsc = display_sin_cos(arc4random_uniform(360));
	ss->ss_green_inc = display_fp_mul(dsc->cos, DISPLAY_FP_CONST(2.0));

	dsc = display_sin_cos(arc4random_uniform(360));
	ss->ss_blue_inc = display_fp_mul(dsc->cos, DISPLAY_FP_CONST(2.0));

	if (width > height)
		ss->ss_max_loop = (height / 2);
	else
		ss->ss_max_loop = (width / 2);

	ss->ss_loop = arc4random_uniform(ss->ss_max_loop);

	dsc = display_sin_cos(arc4random_uniform(45));
	assert(display_fp_to_int(dsc->cos) >= 0);
	ss->ss_loop_inc = dsc->cos;
	ss->ss_loop_acc = DISPLAY_FP_CONST(0.0);


	return &ss->ss_gizmo;
}
