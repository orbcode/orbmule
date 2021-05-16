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
#include <stdlib.h>
#include <string.h>

#include "display.h"
#include "linked-lists.h"

#define	SEGMENT_SIZE_MAX	8
#define	SEGMENTS_PER_WORM_MAX	16

struct worm_segment {
	CIRCLEQ_ENTRY(worm_segment) seg_qent;
	uint16_t seg_x;
	uint16_t seg_y;
	uint8_t seg_red;
	uint8_t seg_green;
	uint8_t seg_blue;
	uint8_t seg_id;
};

struct worm_segment_intersect {
	/* si_line is indexed by Y coord. */
	uint8_t si_line[SEGMENT_SIZE_MAX];
	/* si_pixels is indexed by Y/X coord. */
	uint8_t si_pixels[SEGMENT_SIZE_MAX][SEGMENT_SIZE_MAX];
};

CIRCLEQ_HEAD(ws_qhead, worm_segment);

struct worm {
	display_fp_t w_speed;
	display_fp_t w_dx;
	display_fp_t w_dy;
	uint8_t w_id;
	struct ws_qhead w_segments;
};

struct worm_state {
	struct display_gizmo ws_gizmo;
	display_port_t ws_dp;
	uint16_t ws_segment_size;
	uint16_t ws_segments;
	uint16_t ws_worm_count;
	uint8_t ws_erase_tail;
	uint8_t ws_gregarious;
	display_fp_t ws_x_limit;
	display_fp_t ws_y_limit;
	struct worm *ws_worms;
	struct worm_segment *ws_all_segments;
};

static void
segment_random_colour(struct worm_segment *seg)
{
	uint32_t r;

	while ((r = arc4random()) == 0)
		;

	seg->seg_red = (r >> 3) & 0xffu;
	seg->seg_green = (r >> 7) & 0xffu;
	seg->seg_blue = (r >> 13) & 0xffu;
}

#define	CLIP_COORDS(ws,a,b)	(((int)((a)-(b)) < 0) || \
				 (((a)-(b)) >= ws->ws_segment_size))

static void
segment_find_intersects(struct worm_state *ws,
    struct worm_segment_intersect *si, int x, int y, int px, int py)
{
	unsigned int ix, iy;

	/* Scan each horizontal line of the target segment */
	for (iy = 0; iy < ws->ws_segment_size; iy++) {
		/*
		 * Short-circuit if we already know the
		 * entire line intersects with others.
		 */
		if (si->si_line[iy] == ws->ws_segment_size)
			continue;

		/* Do the Y coordinates intersect? */
		if (CLIP_COORDS(ws, y + iy, py))
			continue;

		/* Check each pixel in the line. */
		for (ix = 0; ix < ws->ws_segment_size; ix++) {
			/*
			 * Short-circuit if the pixel already
			 * intersects with another segment.
			 */
			if (si->si_pixels[iy][ix])
				continue;

			/* X coordinates intersect? */
			if (CLIP_COORDS(ws, x + ix, px))
				continue;

			/* Found a new intersect. Record it. */
			si->si_pixels[iy][ix] = 1;

			/*
			 * Bump the per-line pixel count. If the entire
			 * line is now occluded, we're done.
			 */
			si->si_line[iy]++;
			if (si->si_line[iy] == ws->ws_segment_size)
				break;
		}
	}
}

static int
segment_intersect(struct worm_state *ws, const struct worm_segment *tail,
    struct worm_segment_intersect *si)
{
	const struct worm_segment *seg;
	int rv, x, y, px, py;
	unsigned int total_segs;

	memset(si, 0, sizeof(*si));
	rv = 0;

	x = (int) tail->seg_x;
	y = (int) tail->seg_y;

	total_segs = ws->ws_worm_count * ws->ws_segments;
	seg = ws->ws_all_segments;
	for (unsigned int s = 0; s < total_segs; s++, seg++) {
		/* Ignore this worm's tail segment. */
		if (seg == tail)
			continue;

		/*
		 * If the worms are gregarious, then any segment
		 * can intersect any other.
		 * If the worms are NOT gregarious, then a segment
		 * can only intersect another from the same worm.
		 */
		if (ws->ws_gregarious == 0 && seg->seg_id != tail->seg_id)
			continue;

		px = (int) seg->seg_x;
		py = (int) seg->seg_y;

		/*
		 * Check if this segment overlaps the target segment.
		 */
		if (abs(x - px) < ws->ws_segment_size &&
		    abs(y - py) < ws->ws_segment_size) {
			/*
			 * The segments overlap. Find the overlapping pixels.
			 */
			segment_find_intersects(ws, si, x, y, px, py);
			rv = 1;
		}
	}

	return rv;
}

static void
segment_draw(struct worm_state *ws, int x, int y)
{
	unsigned int i;

	for (i = 0; i < ws->ws_segment_size; i++, y++)
		display_line_horizontal(ws->ws_dp, x, y, ws->ws_segment_size);
}

static void
segment_erase(struct worm_state *ws, const struct worm_segment *tail)
{
	static struct worm_segment_intersect si;
	unsigned int x, y;
	int w, px, py;

	/*
	 * Just return if erase is disabled.
	 */
	if (ws->ws_erase_tail == 0)
		return;

	/* Erase is accomplished by redrawing in black. */
	display_set_colour(ws->ws_dp, 0, 0, 0);

	px = (int) tail->seg_x;
	py = (int) tail->seg_y;

	/*
	 * Check if the target segment overlaps others.
	 */
	if (segment_intersect(ws, tail, &si) == 0) {
		/* No overlap. Erase the entire segment. */
		segment_draw(ws, px, py);
		return;
	}

	/*
	 * At this point, 'si' identifies which pixels overlap with
	 * other segments.
	 */

	/* Erase each horizontal line of the segment. */
	for (y = 0; y < ws->ws_segment_size; y++, py++) {
		if (si.si_line[y] == ws->ws_segment_size) {
			/*
			 * This entire line is occluded by one or
			 * more other segments.
			 */
			continue;
		}

		if (si.si_line[y] == 0) {
			/*
			 * This entire line has no intersecting pixels.
			 * Erase the whole thing in one operation.
			 */
			display_line_horizontal(ws->ws_dp, px, py,
			    ws->ws_segment_size);
			continue;
		}

		/*
		 * At least one pixel in the line is occluded.
		 * Erase those which are not occluded.
		 */
		for (x = 0; x < ws->ws_segment_size; x++) {
			if (si.si_pixels[y][x]) {
				/* Pixel is occluded. Move on. */
				continue;
			}

			/*
			 * Scan until we find the next occluded pixel, or
			 * the end of the line.
			 */
			for (w = x + 1; w < ws->ws_segment_size; w++) {
				if (si.si_pixels[y][w])
					break;
			}

			/* Work out the number of non-occluded pixels. */
			w -= x;

			/* Erase them in one operation. */
			display_line_horizontal(ws->ws_dp, px + x, py, w);
		}
	}
}

static void
worm_new_vector(struct worm_state *ws, struct worm *w)
{
	const struct display_sin_cos *dsc;
	unsigned int heading;
	display_fp_t s, z;
	uint32_t r;

	/*
	 * Maximum speed is proportional to the segment width.
	 */
	r = arc4random_uniform(ws->ws_segment_size * 10);
	s = display_fp_from_int(r);
	s = display_fp_div(s, DISPLAY_FP_CONST(20.0));
	z = display_fp_from_int(ws->ws_segment_size);
	z = display_fp_div(z, DISPLAY_FP_CONST(3.0));
	w->w_speed = s = display_fp_add(s, z);

	/*
	 * Compute the delta X/Y component of the vector.
	 */
	heading = arc4random_uniform(360);
	dsc = display_sin_cos(heading);
	w->w_dx = display_fp_mul(dsc->cos, s);
	w->w_dy = display_fp_mul(dsc->sin, s);
}

static bool
worm_collision(struct worm_state *ws, uint8_t id,
    display_fp_t new_x, display_fp_t new_y)
{
	struct worm_segment *seg;
	unsigned int total_segs;
	int x, y;

	/*
	 * Keep the segment within the bounds of the display port.
	 */
	if (new_x < DISPLAY_FP_CONST(0.0) || new_x > ws->ws_x_limit)
		return true;

	if (new_y < DISPLAY_FP_CONST(0.0) || new_y > ws->ws_y_limit)
		return true;

	/*
	 * Worms in a gregarious colony are happy to crawl over each other.
	 */
	if (ws->ws_gregarious)
		return false;

	/*
	 * Otherwise, worms avoid colliding with each other.
	 */
	x = display_fp_to_int(new_x);
	y = display_fp_to_int(new_y);

	/*
	 * Iterate through all segments of all worms.
	 */
	total_segs = ws->ws_worm_count * ws->ws_segments;
	seg = ws->ws_all_segments;
	while (total_segs--) {
		/*
		 * Worms are happy to overlap their own segments,
		 * but they avoid all others.
		 */
		if (seg->seg_id != id &&
		    abs(x - (int)seg->seg_x) < ws->ws_segment_size &&
		    abs(y - (int)seg->seg_y) < ws->ws_segment_size) {
			return true;
		}

		seg++;
	}

	return false;
}

static void
worm_update(struct worm_state *ws, struct worm *w)
{
	struct worm_segment *new_head, *old_head;
	display_fp_t x, y, old_x, old_y;
	bool collision, collided;
	uint32_t r;

	/*
	 * Remove the tail segment, to be promoted to the new head.
	 */
	old_head = CIRCLEQ_FIRST(&w->w_segments);
	new_head = CIRCLEQ_LAST(&w->w_segments);
	CIRCLEQ_REMOVE(&w->w_segments, new_head, seg_qent);

	/*
	 * Compute the head's new X/Y coords according to the prevailing
	 * vector.
	 */
	collided = false;
	old_x = display_fp_from_int(old_head->seg_x);
	old_y = display_fp_from_int(old_head->seg_y);

	do {
		x = display_fp_add(w->w_dx, old_x);
		y = display_fp_add(w->w_dy, old_y);

		collision = worm_collision(ws, w->w_id, x, y);
		if (collision) {
			worm_new_vector(ws, w);
			collided = true;
		}
	} while (collision);

	/*
	 * If we haven't changed direction due to a collision,
	 * maybe change at random.
	 */
	if (collided == false && ((r = arc4random()) % 128u) < 2) {
		worm_new_vector(ws, w);

		/* Maybe change colour too. */
		if ((r % 8192u) < 4)
			segment_random_colour(new_head);
		else
			goto same_colour;
	} else {
		/*
		 * We're maintaining the same speed and direction. Reuse
		 * the colour values from the original head segment.
		 */
 same_colour:
		new_head->seg_red = old_head->seg_red;
		new_head->seg_green = old_head->seg_green;
		new_head->seg_blue = old_head->seg_blue;
	}

	/* Erase the old tail segment. */
	segment_erase(ws, new_head);

	/* Insert the new head now that the erase has been performed. */
	CIRCLEQ_INSERT_HEAD(&w->w_segments, new_head, seg_qent);

	/* Record the head's coordinates, and draw it. */
	new_head->seg_x = (uint16_t) display_fp_to_int(x);
	new_head->seg_y = (uint16_t) display_fp_to_int(y);

	/* Draw the segment. */
	display_set_colour(ws->ws_dp, new_head->seg_red, new_head->seg_green,
	    new_head->seg_blue);
	segment_draw(ws, (int)new_head->seg_x, (int)new_head->seg_y);
}

static void
worm_loop(display_gizmo_t dg)
{
	struct worm_state *ws = (struct worm_state *)dg;
	struct worm *w;

	w = ws->ws_worms;
	for (unsigned int wc = 0; wc < ws->ws_worm_count; wc++, w++)
		worm_update(ws, w);
}

static void
worm_create_colony(struct worm_state *ws, uint16_t width, uint16_t height)
{
	struct worm_segment *seg, *h = NULL;
	struct worm *w;
	uint16_t x, y;

	seg = ws->ws_all_segments;

	w = ws->ws_worms;
	for (unsigned int wc = 0; wc < ws->ws_worm_count; wc++, w++) {
		w->w_id = (uint8_t)wc;
		worm_new_vector(ws, w);
		CIRCLEQ_INIT(&w->w_segments);

		do {
			x = arc4random_uniform(width -
			    (ws->ws_segment_size * 2)) + ws->ws_segment_size;
			y = arc4random_uniform(height -
			    (ws->ws_segment_size * 2)) + ws->ws_segment_size;
		} while (worm_collision(ws, w->w_id, display_fp_from_int(x),
		    display_fp_from_int(y)));

		h = seg;
		segment_random_colour(h);

		for (unsigned int i = 0; i < ws->ws_segments; i++, seg++) {
			seg->seg_id = (uint8_t)wc;
			seg->seg_x = x;
			seg->seg_y = y;
			seg->seg_red = h->seg_red;
			seg->seg_green = h->seg_green;
			seg->seg_blue = h->seg_blue;

			CIRCLEQ_INSERT_TAIL(&w->w_segments, seg, seg_qent);
		}
	}
}

static void
worm_stop(display_gizmo_t dg)
{
	struct worm_state *ws = (struct worm_state *)dg;
	struct worm_segment *seg;
	unsigned int total_segs;

	total_segs = ws->ws_worm_count * ws->ws_segments;
	seg = ws->ws_all_segments;
	display_set_colour(ws->ws_dp, 0, 0, 0);

	for (unsigned int s = 0; s < total_segs; s++, seg++)
		segment_draw(ws, seg->seg_x, seg->seg_y);

	free(ws->ws_all_segments);
	free(ws->ws_worms);
	free(ws);
}

static unsigned int
worm_segment_size(unsigned int width, unsigned int height)
{
	unsigned int x;

	(void) height;

	x = 500u / width;
	if (x > SEGMENT_SIZE_MAX)
		x = 0;
	else
		x = SEGMENT_SIZE_MAX - x;

	if (x < 2)
		x = 2;

	return x;
}

static unsigned int
worm_segments_per_worm(unsigned int width, unsigned int height)
{
	unsigned int rv;

	(void) width;

	rv = 600u / height;
	if (rv >= SEGMENTS_PER_WORM_MAX)
		rv = 2;
	else {
		rv = SEGMENTS_PER_WORM_MAX - rv;
		if (rv < 2)
			rv = 2;
	}

	return rv;
}

static unsigned int
worm_count(struct worm_state *ws, uint16_t width, uint16_t height)
{

	(void) width;
	(void) height;

	return ((ws->ws_segment_size * ws->ws_segments) / 9) + 1;
}

display_gizmo_t
display_gizmo_worm_start(display_port_t dp, int width, int height)
{
	struct worm_state *ws;
	unsigned int total_segs;
	display_fp_t w, h, s;

	if ((ws = malloc(sizeof(*ws))) == NULL)
		return NULL;

	ws->ws_dp = dp;
	ws->ws_gizmo.dg_stop = worm_stop;
	ws->ws_gizmo.dg_loop = worm_loop;

	ws->ws_segments = worm_segments_per_worm(width, height);
	ws->ws_segment_size = worm_segment_size(width, width);

	ws->ws_worm_count = worm_count(ws, width, height);
	ws->ws_worms = malloc(sizeof(*ws->ws_worms) * ws->ws_worm_count);
	if (ws->ws_worms == NULL) {
		free(ws);
		return NULL;
	}

	total_segs = ws->ws_worm_count * ws->ws_segments;
	ws->ws_all_segments = calloc(total_segs, sizeof(*ws->ws_all_segments));
	if (ws->ws_all_segments == NULL) {
		free(ws->ws_worms);
		free(ws);
		return NULL;
	}

	ws->ws_erase_tail = 1;
	ws->ws_gregarious = arc4random_uniform(128) >= 64;

	w = display_fp_from_int(width);
	h = display_fp_from_int(height);
	s = display_fp_from_int(ws->ws_segment_size);
	ws->ws_x_limit = display_fp_sub(w, s);
	ws->ws_y_limit = display_fp_sub(h, s);

	worm_create_colony(ws, width, height);

	return &ws->ws_gizmo;
}
