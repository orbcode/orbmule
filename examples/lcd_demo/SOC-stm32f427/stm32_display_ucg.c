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
#include <stdio.h>
#include <unistd.h>

#include "soc.h"
#include "stm32_gpio.h"
#include "stm32_hal.h"
#include "display_ucg.h"

/*
 * TFT Display connectivity using SPI1.
 * Extending this to supported DMA and/or interrupts is left as an exercise
 * for the reader.
 *
 * Note that past experimentation has shown that performance of a non-DMA
 * interrupt mode implementation performs worse than non-DMA polled mode.
 */
#define	SPIDEV	SPI1
SOC_GPIO_PIN(LCD_SCK, A, 5)		/* SPI1 AF5, output */
SOC_GPIO_PIN(LCD_MISO, A, 6)		/* SPI1 AF5, input */
SOC_GPIO_PIN(LCD_MOSI, A, 7)		/* SPI1 AF5, output */
SOC_GPIO_PIN(LCD_CS, A, 4)		/* GPIO, output */
SOC_GPIO_PIN(LCD_DCRS, A, 3)		/* GPIO, output */
SOC_GPIO_PIN(LCD_RESET, A, 2)		/* GPIO, output */

struct soc_display_state {
	SPI_TypeDef *ds_spi;
	ringbuff_t ds_rb;
	uint64_t ds_tx_count;
};
static struct soc_display_state soc_display_state;

static void
spi_clk_to_br_and_drive_strength(uint32_t bus_clock,
    uint32_t desired_clock, uint32_t *br, uint32_t *ds)
{

	*br = 7u << SPI_CR1_BR_Pos;

	for (u_int i = 0; i < 8; i++) {
		if (desired_clock >= (bus_clock / (2 << i))) {
			*br = i << SPI_CR1_BR_Pos;
			break;
		}
	}

	if (ds != NULL) {
		/* Work out GPIO pin drive strength. */
		if (desired_clock >= 100000000u)
			*ds = 3u;
		else
		if (desired_clock >= 50000000u)
			*ds = 3u;
		else
		if (desired_clock >= 25000000u)
			*ds = 2u;
		else
			*ds = 2u;
	}
}

static void
soc_display_power(void *arg, uint32_t clk)
{
	struct soc_display_state *ds = arg;
	SPI_TypeDef *spi = ds->ds_spi;
	uint32_t br, dst;

	if (clk == 0) {
		spi->CR1 = 0;
		return;
	}

	/*
	 * Get as close as possible to the required clock speed,
	 * without exceeding it.
	 */
	spi_clk_to_br_and_drive_strength(CLK_APB2_RATE, clk, &br, &dst);

	/* Configure the GPIO pins */
	SOC_GPIO_LCD_RESET_set();
	SOC_GPIO_LCD_RESET_out();
	SOC_GPIO_LCD_RESET_speed(0);
	SOC_GPIO_LCD_CS_set();
	SOC_GPIO_LCD_CS_out();
	SOC_GPIO_LCD_CS_speed(dst);
	SOC_GPIO_LCD_DCRS_out();
	SOC_GPIO_LCD_DCRS_speed(dst);

	SOC_GPIO_LCD_MOSI_speed(dst);
	SOC_GPIO_LCD_MOSI_function(5);
	SOC_GPIO_LCD_SCK_speed(dst);
	SOC_GPIO_LCD_SCK_function(5);
	SOC_GPIO_LCD_MISO_pullup(1);
	SOC_GPIO_LCD_MISO_function(5);

	spi->CR1 = 0;
	__DSB();
	__ISB();
	spi->CR1 = br | SPI_CR1_MSTR;
#ifdef SPI_CR2_DS_Pos
	spi->CR2 = (7u << SPI_CR2_DS_Pos) | SPI_CR2_SSOE | SPI_CR2_FRXTH;
#else
	spi->CR2 = SPI_CR2_SSOE;
#endif
	__DSB();
	__ISB();
	spi->CR1 |= SPI_CR1_SPE;
}

static void
soc_display_reset(void *arg, unsigned int set)
{

	(void) arg;

	if (set)
		SOC_GPIO_LCD_RESET_set();
	else
		SOC_GPIO_LCD_RESET_clr();
}

static void
soc_display_chip_select(void *arg, unsigned int set)
{

	(void) arg;

	if (set)
		SOC_GPIO_LCD_CS_set();
	else
		SOC_GPIO_LCD_CS_clr();
}

static void
soc_display_dcrs(void *arg, unsigned int set)
{

	(void) arg;

	if (set)
		SOC_GPIO_LCD_DCRS_set();
	else
		SOC_GPIO_LCD_DCRS_clr();
}

static void
soc_display_flush(void *arg)
{
	struct soc_display_state *ds = arg;
	SPI_TypeDef *spi = ds->ds_spi;

#ifndef SPI_SR_FTLVL
#define	SPI_SR_FTLVL	0
#endif

	while ((spi->SR & (SPI_SR_FTLVL | SPI_SR_BSY)) != 0)
		;
}

static uint64_t
soc_display_stats(void *arg)
{
	struct soc_display_state *ds = arg;

	return ds->ds_tx_count;
}

static void
soc_display_irq(void *arg)
{
	struct soc_display_state *ds = arg;
	SPI_TypeDef *spi = ds->ds_spi;
	ringbuff_len_t cnt;
	volatile uint8_t *dr = (volatile uint8_t *)(uintptr_t)&spi->DR;
	unsigned int len = 0;

	cnt = ringbuff_get_count(ds->ds_rb);

	while (cnt && (spi->SR & SPI_SR_TXE) != 0) {
		*dr = ringbuff_consume(ds->ds_rb);
		cnt--;
		len++;
	}

	ds->ds_tx_count += len;
}

static void
soc_display_out(ringbuff_t rb, void *arg)
{
	struct soc_display_state *ds = arg;

	(void) rb;

	soc_display_irq(ds);
}

void
soc_display_init(void)
{
	struct soc_display_state *ds = &soc_display_state;
	display_ucg_hw_interface_t hi;

	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	__DSB();
	__ISB();
	(void)RCC->APB2ENR;
	__DSB();

	ds->ds_spi = SPIDEV;

	hi.hi_power = soc_display_power;
	hi.hi_reset = soc_display_reset;
	hi.hi_cs = soc_display_chip_select;
	hi.hi_dcrs = soc_display_dcrs;
	hi.hi_flush = soc_display_flush;
	hi.hi_stats = soc_display_stats;

	ds->ds_rb = display_ucg_attach(&hi, ds);
	assert(ds->ds_rb != NULL);

	ringbuff_consumer_init(ds->ds_rb, soc_display_out, ds);

	display_ucg_start();
}
