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

/*
 * Simple driver for STM32Fx USARTs.
 */

#include <assert.h>
#include <stdlib.h>
#include <stdint.h>

#include "soc.h"
#include "ringbuff.h"
#include "stm32_usart.h"

/*
 * The following is to allow this code to be shared between STM32Fxxx
 * variants.
 */
#ifndef USART_CR1_M1
#define	RDR				DR
#define	TDR				DR
#define	ISR				SR
#define	USART_ISR_PE			USART_SR_PE
#define	USART_ISR_FE			USART_SR_FE
#define	USART_ISR_NE			USART_SR_NE
#define	USART_ISR_ORE			USART_SR_ORE
#define	USART_ISR_RXNE			USART_SR_RXNE
#define	USART_ISR_TXE			USART_SR_TXE
#define	USART_BRR_DIV_MANTISSA		USART_BRR_DIV_Mantissa
#define	USART_BRR_DIV_MANTISSA_Pos	USART_BRR_DIV_Mantissa_Pos
#define	USART_BRR_DIV_FRACTION		USART_BRR_DIV_Fraction
#define	USART_BRR_DIV_FRACTION_Pos	USART_BRR_DIV_Fraction_Pos
#define	USART_CR1_M0			0
#define	USART_CR1_M1			0
#endif

struct stm32_usart_state {
	USART_TypeDef *us_regs;
	uint32_t us_clock;
	ringbuff_t us_uart2app;
	ringbuff_t us_app2uart;
};

static __always_inline void
stm32_usart_tx_irq_disable(USART_TypeDef *u)
{

	u->CR1 &= ~USART_CR1_TXEIE;
}

static __always_inline void
stm32_usart_tx_irq_enable(USART_TypeDef *u)
{

	u->CR1 |= USART_CR1_TXEIE;
}

static __always_inline void
stm32_usart_rx_irq_disable(USART_TypeDef *u)
{

	u->CR3 &= ~USART_CR3_EIE;
	u->CR1 &= ~(USART_CR1_PEIE | USART_CR1_RXNEIE);
}

static __always_inline void
stm32_usart_rx_irq_enable(USART_TypeDef *u)
{

	u->CR1 |= USART_CR1_PEIE | USART_CR1_RXNEIE;
	u->CR3 |= USART_CR3_EIE;
}

static __always_inline uint8_t
stm32_usart_tx_busy(USART_TypeDef *u)
{

	return (u->ISR & USART_ISR_TXE) == 0;
}

static int
stm32_usart_irq_rx_ready(struct stm32_usart_state *us)
{
	uint32_t data;
	int rv = 0;

	data =  us->us_regs->RDR;

	if (us->us_uart2app == NULL)
		return 0;

	if (!ringbuff_is_full(us->us_uart2app)) {
		rv = 1;
		ringbuff_produce(us->us_uart2app, (uint8_t)(data & 0xffu));
	} else {
		rv = 0;
	}

	return rv;
}

static void
stm32_usart_irq_tx_done(struct stm32_usart_state *us)
{
	uint8_t ch;

	if (us->us_app2uart == NULL || ringbuff_is_empty(us->us_app2uart)) {
		/* No peer, or FIFO empty. Disable Tx Empty irq */
		stm32_usart_tx_irq_disable(us->us_regs);
		return;
	}

	ch = ringbuff_consume(us->us_app2uart);
	us->us_regs->TDR = (uint32_t) ch;
}

static uint32_t
stm32_usart_active_irqs(USART_TypeDef *u)
{
	uint32_t cr1, mask = 0;

#define	RX_ERROR	(USART_ISR_FE | USART_ISR_NE | USART_ISR_PE | USART_ISR_ORE)

	cr1 = u->CR1;

	if (cr1 & USART_CR1_TXEIE)
		mask |= USART_ISR_TXE;

	if (cr1 & (USART_CR1_RXNEIE | USART_CR1_PEIE))
		mask |= USART_ISR_RXNE | RX_ERROR;

	return u->ISR & mask;
}

void
stm32_usart_interrupt(void *arg)
{
	struct stm32_usart_state *us = arg;
	USART_TypeDef *u = us->us_regs;
	int rx_work, tx_work;
	uint32_t st;

	rx_work = tx_work = 0;

	while ((st = stm32_usart_active_irqs(u)) != 0) {
		if ((st & USART_ISR_RXNE) != 0)
			rx_work += stm32_usart_irq_rx_ready(us);

		if ((st & USART_ISR_TXE) != 0) {
			stm32_usart_irq_tx_done(us);
			tx_work = 1;
		}
	}

	if (rx_work && us->us_uart2app)
		ringbuff_produce_done(us->us_uart2app);

	if (tx_work && us->us_app2uart)
		ringbuff_consume_done(us->us_app2uart);
}

/*
 * Invoked whenever data is received from the application and has
 * been loaded into the ring buffer.
 */
static void
stm32_usart_tx_avail_cb(ringbuff_t rb, void *arg)
{
	struct stm32_usart_state *us = arg;

	if (__get_PRIMASK() != 0) {
		/*
		 * Interrupts are disabled. Use polled transmit.
		 */
		while (!ringbuff_is_empty(rb)) {
			uint8_t ch;

			ch = ringbuff_consume(us->us_app2uart);

			while (stm32_usart_tx_busy(us->us_regs))
				;
			us->us_regs->TDR = (uint32_t) ch;
		}
		ringbuff_consume_done(us->us_app2uart);
	} else {
		stm32_usart_tx_irq_enable(us->us_regs);
	}
}

static uint32_t
stm32_usart_actual_speed(struct stm32_usart_state *us, uint32_t brr,
    uint32_t cr1)
{
	uint32_t over, m, f;

	m = (brr & USART_BRR_DIV_MANTISSA) >> USART_BRR_DIV_MANTISSA_Pos;
	f = (brr & USART_BRR_DIV_FRACTION) >> USART_BRR_DIV_FRACTION_Pos;

	if (cr1 & USART_CR1_OVER8) {
		f &= 0x7u;
		f <<= 1;
		over = 8;
	} else {
		over = 16;
	}

	return (us->us_clock * 16u) / (over * ((m * 16u) + f));
}

static uint32_t
stm32_usart_calc_brr(struct stm32_usart_state *us, uint32_t baud, uint32_t *over)
{
	uint32_t i8, f8, m8, brr8, i16, f16, m16, brr16;
	uint32_t baud8, baud16;
	unsigned int err8, err16;
	uint32_t clk;

	clk = us->us_clock * 16u;

	i16 = (clk + ((16u * baud) / 2u)) / (16u * baud);
	m16 = i16 >> 4;
	f16 = i16 & 0x0fu;

	i8 = (clk + ((8u * baud) / 2u)) / (8u * baud);
	m8 = i8 >> 3;
	f8 = (i8 >> 1) & 0x07u;

	if (m16 > (USART_BRR_DIV_MANTISSA >> USART_BRR_DIV_MANTISSA_Pos) ||
	    (m8 > (USART_BRR_DIV_MANTISSA >> USART_BRR_DIV_MANTISSA_Pos))) {
		/* Baudrate too low. */
		return 0;
	}

	/* Both will be zero if the rate is unattainably high */
	if (m16 == 0 && m8 == 0)
		return 0;

	brr8 = (m8 << USART_BRR_DIV_MANTISSA_Pos) |
	    (f8 << USART_BRR_DIV_FRACTION_Pos);
	baud8 = stm32_usart_actual_speed(us, brr8, USART_CR1_OVER8);

	/* If m16 is zero then we have to go with m8 */
	if (m16 == 0) {
		*over |= USART_CR1_OVER8;
		return brr8;
	}

	brr16 = (m16 << USART_BRR_DIV_MANTISSA_Pos) |
	    (f16 << USART_BRR_DIV_FRACTION_Pos);
	baud16 = stm32_usart_actual_speed(us, brr16, 0);

	/*
	 * Go with whichever configuration has the least error, but
	 * prefer x16 if its error is less than about 1.6%.
	 */
	err8 = (unsigned int)abs((int)baud - (int)baud8);
	err16 = (unsigned int)abs((int)baud - (int)baud16);

	if (err16 <= err8 || err16 < ((baud / 128u) * 2u)) {
		*over &= ~USART_CR1_OVER8;
		return brr16;
	}

	*over |= USART_CR1_OVER8;
	return brr8;
}

static uint32_t
stm32_usart_configure(struct stm32_usart_state *us, uint32_t baud)
{
	uint32_t cr1, cr2, brr;

	cr1 = us->us_regs->CR1;
	us->us_regs->CR1 = cr1 & ~USART_CR1_UE;

	/* 1 stop bit. */
	cr2 = 0;

	/* No parity. */
	cr1 &= ~(USART_CR1_PCE | USART_CR1_PS);

	/* 8 data bits. */
	cr1 &= ~(USART_CR1_M0 | USART_CR1_M1);

	brr = stm32_usart_calc_brr(us, baud, &cr1);
	assert(brr != 0);

	us->us_regs->CR1 = cr1;
	us->us_regs->CR2 = cr2;
	us->us_regs->BRR = brr;
	us->us_regs->CR1 = cr1 | USART_CR1_UE;

	return baud;
}

void *
stm32_usart_init(const stm32_usart_attach_args_t *aa)
{
	struct stm32_usart_state *us;

	if ((us = calloc(1, sizeof(*us))) == NULL)
		return NULL;

	us->us_regs = aa->aa_regs;
	us->us_clock = aa->aa_clock;
	us->us_uart2app = aa->aa_rb_rx;
	us->us_app2uart = aa->aa_rb_tx;

	stm32_usart_rx_irq_disable(us->us_regs);
	stm32_usart_tx_irq_disable(us->us_regs);
	stm32_usart_configure(us, aa->aa_baud);

	uint32_t cr = us->us_regs->CR1;
	if (us->us_uart2app != NULL)
		cr |= USART_CR1_RE;
	else
		cr &= ~USART_CR1_RE;

	if (us->us_app2uart != NULL) {
		cr |= USART_CR1_TE;
		ringbuff_consumer_init(us->us_app2uart, stm32_usart_tx_avail_cb,
		    us);
	} else {
		cr &= ~USART_CR1_TE;
	}

	us->us_regs->CR1 = cr;

	if (cr & USART_CR1_RE)
		stm32_usart_rx_irq_enable(us->us_regs);

	return us;
}
