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

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "display.h"
#include "display_ucg.h"
#include "ringbuff.h"
#include "timer.h"
#include "ucg.h"
#include "ucg_local.h"

/*
 * Default chipset is ILI9488. Ucglib does not have a driver for this
 * device, but we have a local version.
 */
#define	DISPLAY_UCG_DEV		ucg_dev_ili9488_18x320x480
#define	DISPLAY_UCG_EXT		ucg_ext_ili9488_18

struct display_ucg_state {
	ucg_t ds_ucg;
	display_ucg_hw_interface_t ds_hw;
	void *ds_hw_cookie;
	ringbuff_t ds_hw_rb;
#define	DISPLAY_RINGBUFF_LEN	128
};
static struct display_ucg_state display_ucg_state;

static void
display_hw_flush(struct display_ucg_state *ds)
{

	while (!ringbuff_is_empty(ds->ds_hw_rb))
		ringbuff_produce_done(ds->ds_hw_rb);

	ds->ds_hw.hi_flush(ds->ds_hw_cookie);
}

static void
display_hw_write_byte(ringbuff_t rb, uint8_t v)
{

	while (ringbuff_is_full(rb))
		ringbuff_produce_done(rb);

	ringbuff_produce(rb, v);
}

static int16_t
display_coms(ucg_t *ucg, int16_t msg, uint16_t arg, uint8_t *data)
{
	struct display_ucg_state *ds = (void *)ucg;
	uint16_t desired_period;
	uint8_t v, v2, v3;

	switch(msg) {
	case UCG_COM_MSG_POWER_UP:
		/*
		 * "data" is a pointer to ucg_com_info_t structure with the
		 * following information:
		 *  - ((ucg_com_info_t *)data)->serial_clk_speed value in
		 *    nanoseconds.
		 *  - ((ucg_com_info_t *)data)->parallel_clk_speed value in
		 *    nanoseconds.
		 * "arg" is not used
		 *
		 * This message is sent once at the uC startup and for
		 * power up, so setup i/o or do any other setup.
		 */
		desired_period = ((ucg_com_info_t *)(uintptr_t)data)->serial_clk_speed;
		ds->ds_hw.hi_power(ds->ds_hw_cookie,
		    1000000000u / desired_period);
		break;

	case UCG_COM_MSG_POWER_DOWN:
		/*
		 * "data" and "arg" are not used.
		 * This message is sent for a power down request
		 */
		ds->ds_hw.hi_power(ds->ds_hw_cookie, 0);
		break;

	case UCG_COM_MSG_DELAY:
		/*
		 * "data" is not used.
		 * "arg" contains the number of microseconds for the delay
		 * By receiving this message, the following code should delay by
		 * "arg" microseconds. One microsecond is 0.000001 second
		 */
		display_hw_flush(ds);
		if (arg < 1000u)
			timer_buzz_delay_us(arg);
		else
			timer_buzz_delay_ms(arg / 1000u);
		break;

	case UCG_COM_MSG_CHANGE_RESET_LINE:
		/*
		 * "data" is not used
		 * "arg" = 1: set the reset output line to 1
		 * "arg" = 0: set the reset output line to 0
		 */
		display_hw_flush(ds);
		ds->ds_hw.hi_reset(ds->ds_hw_cookie, arg);
		break;

	case UCG_COM_MSG_CHANGE_CD_LINE:
		/*
		 * "ucg->com_status"  bit 0 contains the old level for the
		 * CD line.
		 * "data" is not used
		 * "arg" = 1: set the command/data (a0) output line to 1
		 * "arg" = 0: set the command/data (a0) output line to 0
		 */
		display_hw_flush(ds);
		ds->ds_hw.hi_dcrs(ds->ds_hw_cookie, arg);
		break;

	case UCG_COM_MSG_CHANGE_CS_LINE:
		/*
		 * "ucg->com_status"  bit 1 contains the old level for the
		 * CS line.
		 * "data" is not used
		 * "arg" = 1: set the chipselect output line to 1
		 * "arg" = 0: set the chipselect output line to 0
		 */
		display_hw_flush(ds);
		ds->ds_hw.hi_cs(ds->ds_hw_cookie, arg);
		break;

	case UCG_COM_MSG_SEND_BYTE:
		/*
		 * "data" is not used
		 * "arg" contains one byte, which should be sent to the display
		 */
		v = (uint8_t)arg;
		display_hw_write_byte(ds->ds_hw_rb, v);
		ringbuff_produce_done(ds->ds_hw_rb);
		break;

	case UCG_COM_MSG_REPEAT_1_BYTE:
		/*
		 * "data[0]" contains one byte
		 * repeat sending the byte in data[0] "arg" times
		 */
		v = data[0];
		while (arg--)
			display_hw_write_byte(ds->ds_hw_rb, v);
		ringbuff_produce_done(ds->ds_hw_rb);
		break;

	case UCG_COM_MSG_REPEAT_2_BYTES:
		/*
		 * "data[0]" contains first byte
		 * "data[1]" contains second byte
		 * repeat sending the two bytes "arg" times
		 */
		v = data[0];
		v2 = data[1];
		while (arg--) {
			display_hw_write_byte(ds->ds_hw_rb, v);
			display_hw_write_byte(ds->ds_hw_rb, v2);
		}
		ringbuff_produce_done(ds->ds_hw_rb);
		break;

	case UCG_COM_MSG_REPEAT_3_BYTES:
		/*
		 * "data[0]" contains first byte
		 * "data[1]" contains second byte
		 * "data[2]" contains third byte
		 * repeat sending the three bytes "arg" times
		 */
		v = data[0];
		v2 = data[1];
		v3 = data[2];
		while (arg--) {
			display_hw_write_byte(ds->ds_hw_rb, v);
			display_hw_write_byte(ds->ds_hw_rb, v2);
			display_hw_write_byte(ds->ds_hw_rb, v3);
		}
		ringbuff_produce_done(ds->ds_hw_rb);
		break;

	case UCG_COM_MSG_SEND_STR:
		/*
		 * "data" is an array with "arg" bytes
		 * send "arg" bytes to the display
		 */
		while (arg--)
			display_hw_write_byte(ds->ds_hw_rb, *data++);
		ringbuff_produce_done(ds->ds_hw_rb);
		break;

	case UCG_COM_MSG_SEND_CD_DATA_SEQUENCE:
		/*
		 * "data" is a pointer to two bytes, which contain the cd line
		 * status and display data
		 * "arg" contains the number of these two byte tuples which
		 * need to be analysed and sent.
		 */
		while (arg--) {
			switch (*data++) {
			case 0:
				break;
			case 1:
				ringbuff_produce_done(ds->ds_hw_rb);
				display_hw_flush(ds);
				ds->ds_hw.hi_cs(ds->ds_hw_cookie, 0);
				break;
			default:
				ringbuff_produce_done(ds->ds_hw_rb);
				display_hw_flush(ds);
				ds->ds_hw.hi_cs(ds->ds_hw_cookie, 1);
				break;
			}

			display_hw_write_byte(ds->ds_hw_rb, *data++);
		}
		ringbuff_produce_done(ds->ds_hw_rb);
		break;
	}

	return 1;
}

static void
display_ucg_clear_screen(void *arg)
{
	struct display_ucg_state *ds = arg;

	ucg_ClearScreen(&ds->ds_ucg);
}

static void
display_ucg_set_colour(void *arg, uint8_t red, uint8_t green, uint8_t blue)
{
	struct display_ucg_state *ds = arg;

	ucg_SetColor(&ds->ds_ucg, 0, red, green, blue);
}

static void
display_ucg_draw_hline(void *arg, uint16_t xstart, uint16_t ystart, uint16_t length)
{
	struct display_ucg_state *ds = arg;

	ucg_DrawHLine(&ds->ds_ucg, xstart, ystart, length);
}

static void
display_ucg_draw_vline(void *arg, uint16_t xstart, uint16_t ystart, uint16_t length)
{
	struct display_ucg_state *ds = arg;

	ucg_DrawVLine(&ds->ds_ucg, xstart, ystart, length);
}

static void
display_ucg_draw_pixel(void *arg, uint16_t x, uint16_t y)
{
	struct display_ucg_state *ds = arg;

	ucg_DrawPixel(&ds->ds_ucg, x, y);
}

ringbuff_t
display_ucg_attach(const display_ucg_hw_interface_t *hi, void *cookie)
{
	struct display_ucg_state *ds = &display_ucg_state;

	ds->ds_hw = *hi;
	ds->ds_hw_cookie = cookie;
	ds->ds_hw_rb = ringbuff_alloc(DISPLAY_RINGBUFF_LEN);

	return ds->ds_hw_rb;
}

void
display_ucg_start(void)
{
	struct display_ucg_state *ds = &display_ucg_state;
	struct display_driver dd;

	ucg_Init(&ds->ds_ucg, DISPLAY_UCG_DEV, DISPLAY_UCG_EXT, display_coms);

	/*
	 * We want the display in landscape mode. If the display
	 * defaults to portrait mode, rotate it by 90 degrees.
	 */
	if (ds->ds_ucg.dimension.w < ds->ds_ucg.dimension.h) {
		printf("Display defaults to portrait. Rotating...\n");
		ucg_SetRotate90(&ds->ds_ucg);
	}

	dd.dd_width = (uint16_t)ds->ds_ucg.dimension.w;
	dd.dd_height = (uint16_t)ds->ds_ucg.dimension.h;
	dd.dd_clear_screen = display_ucg_clear_screen;
	dd.dd_set_colour = display_ucg_set_colour;
	dd.dd_draw_hline = display_ucg_draw_hline;
	dd.dd_draw_vline = display_ucg_draw_vline;
	dd.dd_draw_pixel = display_ucg_draw_pixel;
	dd.dd_render = NULL;

	display_attach(&dd, ds);
}
