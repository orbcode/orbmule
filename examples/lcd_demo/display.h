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

#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdio.h>
#include "ringbuff.h"

/* SoC-specific driver interface. */
typedef struct {
	void (*hi_power)(void *, uint32_t);
	void (*hi_reset)(void *, unsigned int);
	void (*hi_cs)(void *, unsigned int);
	void (*hi_dcrs)(void *, unsigned int);
	void (*hi_flush)(void *);
	uint64_t (*hi_stats)(void *);
} display_hw_interface_t;

/* Gizmo interface. */
struct display_gizmo;
typedef struct display_gizmo *display_gizmo_t;
struct display_gizmo {
	void (*dg_loop)(display_gizmo_t);
	void (*dg_stop)(display_gizmo_t);
};

/* Opaque display state for gizmos. */
struct display_port;
typedef struct display_port *display_port_t;

/*
 * The code can use floating point or fixed point arithmetic. We abstract
 * things here to simplify the switch.
 */
#ifdef	DISPLAY_FLOATING_POINT
typedef float display_fp_t;
#define	DISPLAY_FP_CONST(x)	((display_fp_t)(x))
#define	display_fp_from_int(i)	((display_fp_t)(int)(i))
#define	display_fp_to_int(f)	((int)(f))
#define	display_fp_add(a,b)	((a) + (b))
#define	display_fp_sub(a,b)	((a) - (b))
#define	display_fp_mul(a,b)	((a) * (b))
#define	display_fp_div(a,b)	((a) / (b))
#else
#include "fix16.h"
typedef fix16_t display_fp_t;
#define	DISPLAY_FP_CONST(x)	((display_fp_t)F16(x))
#define	display_fp_from_int(i)	fix16_from_int((int)(i))
#define	display_fp_to_int(f)	fix16_to_int(f)
#define	display_fp_add(a,b)	fix16_add(a, b)
#define	display_fp_sub(a,b)	fix16_sub(a, b)
#define	display_fp_mul(a,b)	fix16_mul(a, b)
#define	display_fp_div(a,b)	fix16_div(a, b)
#endif

/* Sine/Cosine support. */
struct display_sin_cos {
	display_fp_t cos;
	display_fp_t sin;
};

/* Exported display API. */
extern ringbuff_t display_attach(display_hw_interface_t *, void *);

extern void display_pixel(display_port_t, int x, int y);
extern void display_line_horizontal(display_port_t, int x, int y, int w);
extern void display_line_vertical(display_port_t, int x, int y, int h);
extern void display_box(display_port_t, int x, int y, int w, int h);
extern void display_set_colour(display_port_t,
			       uint8_t red, uint8_t green, uint8_t blue);
extern const struct display_sin_cos *display_sin_cos(unsigned int deg);
extern void display_load_defaults(void);

/* Supported Gizmos */
extern display_gizmo_t display_gizmo_squares_start(display_port_t, int w,int h);
extern display_gizmo_t display_gizmo_worm_start(display_port_t, int w, int h);

/* Display update. Called from the main loop. */
extern void display_update(void);

#endif /* DISPLAY_H */
