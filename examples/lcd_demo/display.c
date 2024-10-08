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
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "display.h"
#include "fix16.h"
#include "ringbuff.h"
#include "timer.h"

/*
 * This is the maximum number of display ports we support.
 * Reduce on platforms where SRAM is scarce.
 */
#ifndef DISPLAY_MAX_PORTS
#define	DISPLAY_MAX_PORTS	28
#endif

/*
 * How long a gizmo runs before switching to the next.
 */
#define	DISPLAY_GIZMO_PERIOD	(60 * TIMER_HZ)

typedef display_gizmo_t (*display_gizmo_start_t)(display_port_t, int, int);

static const display_gizmo_start_t display_gizmos[] = {
	display_gizmo_worm_start,
	display_gizmo_squares_start,
};
#define	DISPLAY_NGIZMOS	(sizeof(display_gizmos) / sizeof(display_gizmos[0]))

struct display_port {
	struct display_state *dp_state;
	struct display_port *dp_next;
	display_gizmo_t dp_gizmo;
	uint16_t dp_x;
	uint16_t dp_y;
	uint16_t dp_height;
	uint16_t dp_width;
	uint32_t dp_colour;
};

struct display_state {
	struct display_driver ds_drv;
	void *ds_drv_cookie;
	timer_timeout_t ds_timeout;
	unsigned int ds_nports;
	display_port_t ds_ports;
	uint32_t ds_cur_colour;
};
#define	DISPLAY_WIDTH(ds)	((ds)->ds_drv.dd_width)
#define	DISPLAY_HEIGHT(ds)	((ds)->ds_drv.dd_height)

/*
 * Sine/Cosine lookup table. This is ripe for optimisation due to
 * needless duplication, but it's fast...
 */
#define	DCS(cos, sin)	{DISPLAY_FP_CONST(cos), DISPLAY_FP_CONST(sin)}
static const struct display_sin_cos display_sin_cos_lookup[] = {
	DCS(1.00000000, 0.00000000), DCS(0.99984770, 0.01745241),
	DCS(0.99939083, 0.03489950), DCS(0.99862953, 0.05233596),
	DCS(0.99756405, 0.06975647), DCS(0.99619470, 0.08715574),
	DCS(0.99452190, 0.10452846), DCS(0.99254615, 0.12186934),
	DCS(0.99026807, 0.13917310), DCS(0.98768834, 0.15643447),
	DCS(0.98480775, 0.17364818), DCS(0.98162718, 0.19080900),
	DCS(0.97814760, 0.20791169), DCS(0.97437006, 0.22495105),
	DCS(0.97029573, 0.24192190), DCS(0.96592583, 0.25881905),
	DCS(0.96126170, 0.27563736), DCS(0.95630476, 0.29237170),
	DCS(0.95105652, 0.30901699), DCS(0.94551858, 0.32556815),
	DCS(0.93969262, 0.34202014), DCS(0.93358043, 0.35836795),
	DCS(0.92718385, 0.37460659), DCS(0.92050485, 0.39073113),
	DCS(0.91354546, 0.40673664), DCS(0.90630779, 0.42261826),
	DCS(0.89879405, 0.43837115), DCS(0.89100652, 0.45399050),
	DCS(0.88294759, 0.46947156), DCS(0.87461971, 0.48480962),
	DCS(0.86602540, 0.50000000), DCS(0.85716730, 0.51503807),
	DCS(0.84804810, 0.52991926), DCS(0.83867057, 0.54463904),
	DCS(0.82903757, 0.55919290), DCS(0.81915204, 0.57357644),
	DCS(0.80901699, 0.58778525), DCS(0.79863551, 0.60181502),
	DCS(0.78801075, 0.61566148), DCS(0.77714596, 0.62932039),
	DCS(0.76604444, 0.64278761), DCS(0.75470958, 0.65605903),
	DCS(0.74314483, 0.66913061), DCS(0.73135370, 0.68199836),
	DCS(0.71933980, 0.69465837), DCS(0.70710678, 0.70710678),
	DCS(0.69465837, 0.71933980), DCS(0.68199836, 0.73135370),
	DCS(0.66913061, 0.74314483), DCS(0.65605903, 0.75470958),
	DCS(0.64278761, 0.76604444), DCS(0.62932039, 0.77714596),
	DCS(0.61566148, 0.78801075), DCS(0.60181502, 0.79863551),
	DCS(0.58778525, 0.80901699), DCS(0.57357644, 0.81915204),
	DCS(0.55919290, 0.82903757), DCS(0.54463904, 0.83867057),
	DCS(0.52991926, 0.84804810), DCS(0.51503807, 0.85716730),
	DCS(0.50000000, 0.86602540), DCS(0.48480962, 0.87461971),
	DCS(0.46947156, 0.88294759), DCS(0.45399050, 0.89100652),
	DCS(0.43837115, 0.89879405), DCS(0.42261826, 0.90630779),
	DCS(0.40673664, 0.91354546), DCS(0.39073113, 0.92050485),
	DCS(0.37460659, 0.92718385), DCS(0.35836795, 0.93358043),
	DCS(0.34202014, 0.93969262), DCS(0.32556815, 0.94551858),
	DCS(0.30901699, 0.95105652), DCS(0.29237170, 0.95630476),
	DCS(0.27563736, 0.96126170), DCS(0.25881905, 0.96592583),
	DCS(0.24192190, 0.97029573), DCS(0.22495105, 0.97437006),
	DCS(0.20791169, 0.97814760), DCS(0.19080900, 0.98162718),
	DCS(0.17364818, 0.98480775), DCS(0.15643447, 0.98768834),
	DCS(0.13917310, 0.99026807), DCS(0.12186934, 0.99254615),
	DCS(0.10452846, 0.99452190), DCS(0.08715574, 0.99619470),
	DCS(0.06975647, 0.99756405), DCS(0.05233596, 0.99862953),
	DCS(0.03489950, 0.99939083), DCS(0.01745241, 0.99984770),

	DCS(0.00000000, 1.00000000), DCS(-0.01745241, 0.99984770),
	DCS(-0.03489950, 0.99939083), DCS(-0.05233596, 0.99862953),
	DCS(-0.06975647, 0.99756405), DCS(-0.08715574, 0.99619470),
	DCS(-0.10452846, 0.99452190), DCS(-0.12186934, 0.99254615),
	DCS(-0.13917310, 0.99026807), DCS(-0.15643447, 0.98768834),
	DCS(-0.17364818, 0.98480775), DCS(-0.19080900, 0.98162718),
	DCS(-0.20791169, 0.97814760), DCS(-0.22495105, 0.97437006),
	DCS(-0.24192190, 0.97029573), DCS(-0.25881905, 0.96592583),
	DCS(-0.27563736, 0.96126170), DCS(-0.29237170, 0.95630476),
	DCS(-0.30901699, 0.95105652), DCS(-0.32556815, 0.94551858),
	DCS(-0.34202014, 0.93969262), DCS(-0.35836795, 0.93358043),
	DCS(-0.37460659, 0.92718385), DCS(-0.39073113, 0.92050485),
	DCS(-0.40673664, 0.91354546), DCS(-0.42261826, 0.90630779),
	DCS(-0.43837115, 0.89879405), DCS(-0.45399050, 0.89100652),
	DCS(-0.46947156, 0.88294759), DCS(-0.48480962, 0.87461971),
	DCS(-0.50000000, 0.86602540), DCS(-0.51503807, 0.85716730),
	DCS(-0.52991926, 0.84804810), DCS(-0.54463904, 0.83867057),
	DCS(-0.55919290, 0.82903757), DCS(-0.57357644, 0.81915204),
	DCS(-0.58778525, 0.80901699), DCS(-0.60181502, 0.79863551),
	DCS(-0.61566148, 0.78801075), DCS(-0.62932039, 0.77714596),
	DCS(-0.64278761, 0.76604444), DCS(-0.65605903, 0.75470958),
	DCS(-0.66913061, 0.74314483), DCS(-0.68199836, 0.73135370),
	DCS(-0.69465837, 0.71933980), DCS(-0.70710678, 0.70710678),
	DCS(-0.71933980, 0.69465837), DCS(-0.73135370, 0.68199836),
	DCS(-0.74314483, 0.66913061), DCS(-0.75470958, 0.65605903),
	DCS(-0.76604444, 0.64278761), DCS(-0.77714596, 0.62932039),
	DCS(-0.78801075, 0.61566148), DCS(-0.79863551, 0.60181502),
	DCS(-0.80901699, 0.58778525), DCS(-0.81915204, 0.57357644),
	DCS(-0.82903757, 0.55919290), DCS(-0.83867057, 0.54463904),
	DCS(-0.84804810, 0.52991926), DCS(-0.85716730, 0.51503807),
	DCS(-0.86602540, 0.50000000), DCS(-0.87461971, 0.48480962),
	DCS(-0.88294759, 0.46947156), DCS(-0.89100652, 0.45399050),
	DCS(-0.89879405, 0.43837115), DCS(-0.90630779, 0.42261826),
	DCS(-0.91354546, 0.40673664), DCS(-0.92050485, 0.39073113),
	DCS(-0.92718385, 0.37460659), DCS(-0.93358043, 0.35836795),
	DCS(-0.93969262, 0.34202014), DCS(-0.94551858, 0.32556815),
	DCS(-0.95105652, 0.30901699), DCS(-0.95630476, 0.29237170),
	DCS(-0.96126170, 0.27563736), DCS(-0.96592583, 0.25881905),
	DCS(-0.97029573, 0.24192190), DCS(-0.97437006, 0.22495105),
	DCS(-0.97814760, 0.20791169), DCS(-0.98162718, 0.19080900),
	DCS(-0.98480775, 0.17364818), DCS(-0.98768834, 0.15643447),
	DCS(-0.99026807, 0.13917310), DCS(-0.99254615, 0.12186934),
	DCS(-0.99452190, 0.10452846), DCS(-0.99619470, 0.08715574),
	DCS(-0.99756405, 0.06975647), DCS(-0.99862953, 0.05233596),
	DCS(-0.99939083, 0.03489950), DCS(-0.99984770, 0.01745241),

	DCS(-1.00000000, 0.00000000), DCS(-0.99984770, -0.01745241),
	DCS(-0.99939083, -0.03489950), DCS(-0.99862953, -0.05233596),
	DCS(-0.99756405, -0.06975647), DCS(-0.99619470, -0.08715574),
	DCS(-0.99452190, -0.10452846), DCS(-0.99254615, -0.12186934),
	DCS(-0.99026807, -0.13917310), DCS(-0.98768834, -0.15643447),
	DCS(-0.98480775, -0.17364818), DCS(-0.98162718, -0.19080900),
	DCS(-0.97814760, -0.20791169), DCS(-0.97437006, -0.22495105),
	DCS(-0.97029573, -0.24192190), DCS(-0.96592583, -0.25881905),
	DCS(-0.96126170, -0.27563736), DCS(-0.95630476, -0.29237170),
	DCS(-0.95105652, -0.30901699), DCS(-0.94551858, -0.32556815),
	DCS(-0.93969262, -0.34202014), DCS(-0.93358043, -0.35836795),
	DCS(-0.92718385, -0.37460659), DCS(-0.92050485, -0.39073113),
	DCS(-0.91354546, -0.40673664), DCS(-0.90630779, -0.42261826),
	DCS(-0.89879405, -0.43837115), DCS(-0.89100652, -0.45399050),
	DCS(-0.88294759, -0.46947156), DCS(-0.87461971, -0.48480962),
	DCS(-0.86602540, -0.50000000), DCS(-0.85716730, -0.51503807),
	DCS(-0.84804810, -0.52991926), DCS(-0.83867057, -0.54463904),
	DCS(-0.82903757, -0.55919290), DCS(-0.81915204, -0.57357644),
	DCS(-0.80901699, -0.58778525), DCS(-0.79863551, -0.60181502),
	DCS(-0.78801075, -0.61566148), DCS(-0.77714596, -0.62932039),
	DCS(-0.76604444, -0.64278761), DCS(-0.75470958, -0.65605903),
	DCS(-0.74314483, -0.66913061), DCS(-0.73135370, -0.68199836),
	DCS(-0.71933980, -0.69465837), DCS(-0.70710678, -0.70710678),
	DCS(-0.69465837, -0.71933980), DCS(-0.68199836, -0.73135370),
	DCS(-0.66913061, -0.74314483), DCS(-0.65605903, -0.75470958),
	DCS(-0.64278761, -0.76604444), DCS(-0.62932039, -0.77714596),
	DCS(-0.61566148, -0.78801075), DCS(-0.60181502, -0.79863551),
	DCS(-0.58778525, -0.80901699), DCS(-0.57357644, -0.81915204),
	DCS(-0.55919290, -0.82903757), DCS(-0.54463904, -0.83867057),
	DCS(-0.52991926, -0.84804810), DCS(-0.51503807, -0.85716730),
	DCS(-0.50000000, -0.86602540), DCS(-0.48480962, -0.87461971),
	DCS(-0.46947156, -0.88294759), DCS(-0.45399050, -0.89100652),
	DCS(-0.43837115, -0.89879405), DCS(-0.42261826, -0.90630779),
	DCS(-0.40673664, -0.91354546), DCS(-0.39073113, -0.92050485),
	DCS(-0.37460659, -0.92718385), DCS(-0.35836795, -0.93358043),
	DCS(-0.34202014, -0.93969262), DCS(-0.32556815, -0.94551858),
	DCS(-0.30901699, -0.95105652), DCS(-0.29237170, -0.95630476),
	DCS(-0.27563736, -0.96126170), DCS(-0.25881905, -0.96592583),
	DCS(-0.24192190, -0.97029573), DCS(-0.22495105, -0.97437006),
	DCS(-0.20791169, -0.97814760), DCS(-0.19080900, -0.98162718),
	DCS(-0.17364818, -0.98480775), DCS(-0.15643447, -0.98768834),
	DCS(-0.13917310, -0.99026807), DCS(-0.12186934, -0.99254615),
	DCS(-0.10452846, -0.99452190), DCS(-0.08715574, -0.99619470),
	DCS(-0.06975647, -0.99756405), DCS(-0.05233596, -0.99862953),
	DCS(-0.03489950, -0.99939083), DCS(-0.01745241, -0.99984770),

	DCS(0.00000000, -1.00000000), DCS(0.01745241, -0.99984770),
	DCS(0.03489950, -0.99939083), DCS(0.05233596, -0.99862953),
	DCS(0.06975647, -0.99756405), DCS(0.08715574, -0.99619470),
	DCS(0.10452846, -0.99452190), DCS(0.12186934, -0.99254615),
	DCS(0.13917310, -0.99026807), DCS(0.15643447, -0.98768834),
	DCS(0.17364818, -0.98480775), DCS(0.19080900, -0.98162718),
	DCS(0.20791169, -0.97814760), DCS(0.22495105, -0.97437006),
	DCS(0.24192190, -0.97029573), DCS(0.25881905, -0.96592583),
	DCS(0.27563736, -0.96126170), DCS(0.29237170, -0.95630476),
	DCS(0.30901699, -0.95105652), DCS(0.32556815, -0.94551858),
	DCS(0.34202014, -0.93969262), DCS(0.35836795, -0.93358043),
	DCS(0.37460659, -0.92718385), DCS(0.39073113, -0.92050485),
	DCS(0.40673664, -0.91354546), DCS(0.42261826, -0.90630779),
	DCS(0.43837115, -0.89879405), DCS(0.45399050, -0.89100652),
	DCS(0.46947156, -0.88294759), DCS(0.48480962, -0.87461971),
	DCS(0.50000000, -0.86602540), DCS(0.51503807, -0.85716730),
	DCS(0.52991926, -0.84804810), DCS(0.54463904, -0.83867057),
	DCS(0.55919290, -0.82903757), DCS(0.57357644, -0.81915204),
	DCS(0.58778525, -0.80901699), DCS(0.60181502, -0.79863551),
	DCS(0.61566148, -0.78801075), DCS(0.62932039, -0.77714596),
	DCS(0.64278761, -0.76604444), DCS(0.65605903, -0.75470958),
	DCS(0.66913061, -0.74314483), DCS(0.68199836, -0.73135370),
	DCS(0.69465837, -0.71933980), DCS(0.70710678, -0.70710678),
	DCS(0.71933980, -0.69465837), DCS(0.73135370, -0.68199836),
	DCS(0.74314483, -0.66913061), DCS(0.75470958, -0.65605903),
	DCS(0.76604444, -0.64278761), DCS(0.77714596, -0.62932039),
	DCS(0.78801075, -0.61566148), DCS(0.79863551, -0.60181502),
	DCS(0.80901699, -0.58778525), DCS(0.81915204, -0.57357644),
	DCS(0.82903757, -0.55919290), DCS(0.83867057, -0.54463904),
	DCS(0.84804810, -0.52991926), DCS(0.85716730, -0.51503807),
	DCS(0.86602540, -0.50000000), DCS(0.87461971, -0.48480962),
	DCS(0.88294759, -0.46947156), DCS(0.89100652, -0.45399050),
	DCS(0.89879405, -0.43837115), DCS(0.90630779, -0.42261826),
	DCS(0.91354546, -0.40673664), DCS(0.92050485, -0.39073113),
	DCS(0.92718385, -0.37460659), DCS(0.93358043, -0.35836795),
	DCS(0.93969262, -0.34202014), DCS(0.94551858, -0.32556815),
	DCS(0.95105652, -0.30901699), DCS(0.95630476, -0.29237170),
	DCS(0.96126170, -0.27563736), DCS(0.96592583, -0.25881905),
	DCS(0.97029573, -0.24192190), DCS(0.97437006, -0.22495105),
	DCS(0.97814760, -0.20791169), DCS(0.98162718, -0.19080900),
	DCS(0.98480775, -0.17364818), DCS(0.98768834, -0.15643447),
	DCS(0.99026807, -0.13917310), DCS(0.99254615, -0.12186934),
	DCS(0.99452190, -0.10452846), DCS(0.99619470, -0.08715574),
	DCS(0.99756405, -0.06975647), DCS(0.99862953, -0.05233596),
	DCS(0.99939083, -0.03489950), DCS(0.99984770, -0.01745241),
};
#define	DISPLAY_N_SIN_COS	(sizeof(display_sin_cos_lookup) / \
				 sizeof(display_sin_cos_lookup[0]))

static struct display_state display_state;

const struct display_sin_cos *
display_sin_cos(unsigned int heading)
{

	if (heading >= DISPLAY_N_SIN_COS)
		heading %= DISPLAY_N_SIN_COS;

	return &display_sin_cos_lookup[heading];
}

void
display_set_colour(display_port_t dp, uint8_t red, uint8_t green, uint8_t blue)
{
	struct display_state *ds = dp->dp_state;
	uint32_t colour;

	colour = (uint32_t)red << 16;
	colour |= (uint32_t)green << 8;
	colour |= (uint32_t)blue << 0;
	dp->dp_colour = colour;

	if (ds->ds_cur_colour != colour) {
		ds->ds_cur_colour = colour;
		(ds->ds_drv.dd_set_colour)(ds->ds_drv_cookie, red, green, blue);
	}
}

void
display_line_horizontal(display_port_t dp, int x, int y, int w)
{
	struct display_state *ds = dp->dp_state;

	assert(w >= 1);

	assert(x >= 0);
	assert(x < dp->dp_width);
	assert((x + w) <= dp->dp_width);
	x += dp->dp_x;
	assert((x + w) <= DISPLAY_WIDTH(ds));

	assert(y >= 0);
	assert(y < dp->dp_height);
	y += dp->dp_y;
	assert(y < DISPLAY_HEIGHT(ds));

	(ds->ds_drv.dd_draw_hline)(ds->ds_drv_cookie, (uint16_t)x, (uint16_t)y, (uint16_t)w);
}

void
display_line_vertical(display_port_t dp, int x, int y, int h)
{
	struct display_state *ds = dp->dp_state;

	assert(h >= 1);

	assert(x >= 0);
	assert(x < dp->dp_width);
	x += dp->dp_x;
	assert(x < DISPLAY_WIDTH(ds));

	assert(y >= 0);
	assert(y < dp->dp_height);
	assert((y + h) <= dp->dp_height);
	y += dp->dp_y;
	assert((y + h) <= DISPLAY_HEIGHT(ds));

	(ds->ds_drv.dd_draw_vline)(ds->ds_drv_cookie, (uint16_t)x, (uint16_t)y, (uint16_t)h);
}

void
display_pixel(display_port_t dp, int x, int y)
{
	struct display_state *ds = dp->dp_state;

	assert(x >= 0);
	assert(x < dp->dp_width);
	x += dp->dp_x;
	assert(x < DISPLAY_WIDTH(ds));

	assert(y >= 0);
	assert(y < dp->dp_height);
	y += dp->dp_y;
	assert(y < DISPLAY_HEIGHT(ds));

	(ds->ds_drv.dd_draw_pixel)(ds->ds_drv_cookie, (uint16_t)x, (uint16_t)y);
}

void
display_box(display_port_t dp, int x, int y, int width, int height)
{

	if (height <= 1 && width <= 1) {
		display_pixel(dp, x, y);
	} else
	if (height <= 1) {
		display_line_horizontal(dp, x, y, width);
	} else
	if (width <= 1) {
		display_line_vertical(dp, x, y, height);
	} else {
		display_line_horizontal(dp, x, y, width);
		display_line_horizontal(dp, x, (y + height) - 1, width);
		display_line_vertical(dp, x, y, height);
		display_line_vertical(dp, (x + width) - 1, y, height);
	}
}

static struct display_port *
display_port_layout(struct display_state *ds, unsigned int count)
{
	struct display_port *dp, *head, **prev;
	unsigned int rows, r, cols, c, x, y;
	unsigned int remaining_width, remaining_height;
	unsigned int p_width, p_height;

	for (cols = 1; cols < (count / cols); cols++)
		;
	rows = count / cols;
	cols += (count % cols) > 0;

	/*
	 * 'cols' is the number of columns.
	 * 'rows' is the number of rows.
	 * The final row may have fewer columns.
	 */

	/* Ports are always the same height. */
	remaining_height = DISPLAY_HEIGHT(ds);
	p_height = (remaining_height + (rows - 1)) / rows;

	head = dp = NULL;
	prev = &head;
	y = 0;
	for (r = 0; r < rows; r++) {
		/* The final row may have fewer columns. */
		if (count < cols)
			cols = count;

		/* The final row may also be slightly less high. */
		assert(remaining_height > 0);
		if (p_height > remaining_height)
			p_height = remaining_height;
		remaining_height -= p_height;

		/* All ports in a row are the same width. */
		remaining_width = DISPLAY_WIDTH(ds);
		p_width = (remaining_width + (cols - 1)) / cols;

		x = 0;
		for (c = 0; c < cols; c++, count--) {
			*prev = dp = malloc(sizeof(*dp));
			assert(dp != NULL);
			prev = &dp->dp_next;

			/* The final column may be less wide. */
			assert(remaining_width > 0);
			if (p_width > remaining_width)
				p_width = remaining_width;
			remaining_width -= p_width;

			dp->dp_x = x;
			dp->dp_y = y;
			dp->dp_height = p_height;
			dp->dp_width = p_width;

			x += p_width;
		}

		y += p_height;
	}

	dp->dp_next = NULL;

	return head;
}

static display_gizmo_start_t
display_random_gizmo(void)
{
	unsigned int idx = DISPLAY_NGIZMOS;

	if (idx > 1)
		idx = arc4random_uniform(idx) + 1;

	return display_gizmos[idx - 1];
}

static void
display_new_gizmos(struct display_state *ds)
{
	struct display_port *dp, *ndp;
	display_gizmo_start_t gstart;
	unsigned int count;

	do {
		count = arc4random_uniform(DISPLAY_MAX_PORTS) + 1;
	} while (count == ds->ds_nports);
	ds->ds_nports = count;

	printf("New display with %u port%s.\n", count, (count == 1) ? "" : "s");

	ds->ds_cur_colour = 0xff000000u;
	for (dp = ds->ds_ports; dp != NULL; dp = ndp) {
		ndp = dp->dp_next;
		if (dp->dp_gizmo != NULL)
			(dp->dp_gizmo->dg_stop)(dp->dp_gizmo);
		dp->dp_x -= 1;
		dp->dp_y -= 1;
		dp->dp_width += 2;
		dp->dp_height += 2;
		display_set_colour(dp, 0, 0, 0);
		display_box(dp, 0, 0, dp->dp_width, dp->dp_height);
		free(dp);
	}

	ds->ds_ports = display_port_layout(ds, count);

	for (dp = ds->ds_ports; dp != NULL; dp = dp->dp_next) {
		dp->dp_state = ds;

		/* Draw the port's boundary box. */
		display_set_colour(dp, 255, 255, 255);
		display_box(dp, 0, 0, dp->dp_width, dp->dp_height);

		/* Adjust port geometry to account for the box. */
		dp->dp_x += 1;
		dp->dp_y += 1;
		dp->dp_width -= 2;
		dp->dp_height -= 2;

		/* Alocate a random gizmo for the port. */
		gstart = display_random_gizmo();

		/* Start it. */
		dp->dp_gizmo = gstart(dp, dp->dp_width, dp->dp_height);
	}
}

void
display_update(void)
{
	struct display_state *ds = &display_state;
	struct display_port *dp;

	if (timer_timeout_expired(&ds->ds_timeout)) {
		ds->ds_cur_colour = 0;
		display_new_gizmos(ds);
		timer_timeout_start(&ds->ds_timeout, DISPLAY_GIZMO_PERIOD);
	}

	for (dp = ds->ds_ports; dp != NULL; dp = dp->dp_next) {
		if (dp->dp_gizmo != NULL) {
			ds->ds_cur_colour = 0xff000000u;
			dp->dp_gizmo->dg_loop(dp->dp_gizmo);
		}
	}

	if (ds->ds_drv.dd_render != NULL)
		(ds->ds_drv.dd_render)(ds->ds_drv_cookie);
}

void
display_attach(display_driver_t dd, void *cookie)
{
	struct display_state *ds = &display_state;

	ds->ds_drv = *dd;
	ds->ds_drv_cookie = cookie;

	printf("Display geometry is %d by %d pixels\n",
	    DISPLAY_WIDTH(ds), DISPLAY_HEIGHT(ds));

	(ds->ds_drv.dd_clear_screen)(ds->ds_drv_cookie);
}
