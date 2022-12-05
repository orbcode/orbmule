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

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "display.h"
#include "display_orblcd.h"

/* Note: Landscape mode only. */
#define	DISPLAY_ORBLCD_WIDTH	480u
#define	DISPLAY_ORBLCD_HEIGHT	320u

struct display_orblcd_state {
	display_orblcd_send_fn_t os_send;
	void *os_cookie;
	uint32_t os_colour;
	uint16_t os_cur_x;
	uint16_t os_cur_y;
};
static struct display_orblcd_state display_orblcd_state;

static bool
display_orblcd_send(struct display_orblcd_state *os, bool is_cmd, uint32_t v)
{
	bool rv;

	if ((rv = (os->os_send)(os->os_cookie, is_cmd, v)) == false) {
		os->os_cur_x = DISPLAY_ORBLCD_WIDTH;
		os->os_cur_y = DISPLAY_ORBLCD_HEIGHT;
	}

	return rv;
}

static void
display_orblcd_draw_pixel(void *arg, uint16_t x, uint16_t y)
{
	struct display_orblcd_state *os = arg;

	assert(x < DISPLAY_ORBLCD_WIDTH);
	assert(y < DISPLAY_ORBLCD_HEIGHT);

	if (x != os->os_cur_x || y != os->os_cur_y) {
		os->os_cur_x = x;
		os->os_cur_y = y;

		const uint32_t cmd = ORBLCD_ENCODE_C(ORBLCD_CMD_GOTOXY) |
		    ORBLCD_ENCODE_X((uint32_t)x) | ORBLCD_ENCODE_Y((uint32_t)y);

		if (!display_orblcd_send(os, true, cmd))
			return;
	}

	if (display_orblcd_send(os, false, os->os_colour)) {
		if (++os->os_cur_x >= DISPLAY_ORBLCD_WIDTH) {
			os->os_cur_x = 0;

			if (++os->os_cur_y >= DISPLAY_ORBLCD_HEIGHT)
				os->os_cur_y = 0;
		}
	}
}

static void
display_orblcd_clear_screen(void *arg)
{
	struct display_orblcd_state *os = arg;

	display_orblcd_send(os, true, ORBLCD_CLEAR);
}

static void
display_orblcd_set_colour(void *arg, uint8_t red, uint8_t green, uint8_t blue)
{
	struct display_orblcd_state *os = arg;

	os->os_colour = (uint32_t)red |
	    ((uint32_t)green << 8) | ((uint32_t)blue << 16);
}

static void
display_orblcd_draw_hline(void *arg, uint16_t xstart, uint16_t ystart,
    uint16_t length)
{
	struct display_orblcd_state *os = arg;

	while (length--)
		display_orblcd_draw_pixel(os, xstart++, ystart);
}

static void
display_orblcd_draw_vline(void *arg, uint16_t xstart, uint16_t ystart,
    uint16_t length)
{
	struct display_orblcd_state *os = arg;

	while (length--)
		display_orblcd_draw_pixel(os, xstart, ystart++);
}

static void
display_orblcd_render(void *arg)
{
	struct display_orblcd_state *os = arg;

	display_orblcd_send(os, true,
	    ORBLCD_OPEN_SCREEN(DISPLAY_ORBLCD_WIDTH, DISPLAY_ORBLCD_HEIGHT,
	    ORBLCD_DEPTH_24));

	for (int g=0; g<0x90000; g++) asm("nop;");
}

void
display_orblcd_attach(display_orblcd_send_fn_t snd, void *cookie)
{
	struct display_orblcd_state *os = &display_orblcd_state;
	struct display_driver dd;

	os->os_send = snd;
	os->os_cookie = cookie;
	os->os_colour = 0;
	os->os_cur_x = DISPLAY_ORBLCD_WIDTH;
	os->os_cur_y = DISPLAY_ORBLCD_HEIGHT;

	dd.dd_width = DISPLAY_ORBLCD_WIDTH;
	dd.dd_height = DISPLAY_ORBLCD_HEIGHT;
	dd.dd_clear_screen = display_orblcd_clear_screen;
	dd.dd_set_colour = display_orblcd_set_colour;
	dd.dd_draw_hline = display_orblcd_draw_hline;
	dd.dd_draw_vline = display_orblcd_draw_vline;
	dd.dd_draw_pixel = display_orblcd_draw_pixel;
	dd.dd_render = display_orblcd_render;

	/* First call to display_orblcd_render() initialises the host side. */
	display_orblcd_render(os);

	display_attach(&dd, os);
}
