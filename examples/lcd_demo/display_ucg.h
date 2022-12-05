/*
 * Copyright 2022 Maverick Embedded Technology Ltd.
 * All Rights Reserved.
 *
 * Written by Steve Woodford.
 *
 * No unauthorised reproduction of this software, in either source code or
 * compiled forms, is permitted without the express written permission of
 * Maverick Embedded Technology Ltd.
 */

#ifndef DISPLAY_UCG_H
#define DISPLAY_UCG_H

#include <stdint.h>
#include "ringbuff.h"

/* SoC-specific driver interface. */
typedef struct {
	void (*hi_power)(void *, uint32_t);
	void (*hi_reset)(void *, unsigned int);
	void (*hi_cs)(void *, unsigned int);
	void (*hi_dcrs)(void *, unsigned int);
	void (*hi_flush)(void *);
	uint64_t (*hi_stats)(void *);
} display_ucg_hw_interface_t;

extern ringbuff_t display_ucg_attach(const display_ucg_hw_interface_t *, void *);
extern void display_ucg_start(void);

#endif /* DISPLAY_UCG_H */
