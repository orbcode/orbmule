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

#ifndef DISPLAY_ORBLCD_H
#define DISPLAY_ORBLCD_H

#include <stdbool.h>
#include <stdint.h>

#include "orblcd_protocol.h"

typedef bool (*display_orblcd_send_fn_t)(void *, bool, uint32_t);

extern void display_orblcd_attach(display_orblcd_send_fn_t, void *);

#endif /* DISPLAY_ORBLCD_H */
