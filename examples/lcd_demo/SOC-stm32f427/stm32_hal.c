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
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "soc.h"
#include "stm32_gpio.h"
#include "stm32_hal.h"
#include "stm32_usart.h"
#include "timer.h"

/*
 * SoC is STM32F427VIT6
 *  - 2MB Flash
 *  - 192KB SRAM
 *  - 64KB CCM
 *
 * For development purposes, we use the ORB mule board.
 *
 * - 12MHz external oscillator from the debug board on HSE.
 * - 32KHz xtal connected to LSE.
 * - LEDs on PD12, PD13, and PD14. Active high.
 * - USART2 is console (PD5=TxD, PD6=RxD).
 */

/*
 * The following values are for use with the 12 MHz MCU clock.
 * Once configured, the following system clocks will be available:
 *
 *  SysClk: 168 MHz
 *     AHB: 168 MHz
 *    APB1: 42 MHz
 *    APB2: 84 MHz
 *     USB: 48 MHz
 *
 * XXX: Support Overdrive so core can run at 180 MHz.
 */
#define	PLL_CFG_PLLM		(12 << RCC_PLLCFGR_PLLM_Pos)
#define	PLL_CFG_PLLN		(336 << RCC_PLLCFGR_PLLN_Pos)
#if 1
#define	PLL_CFG_PLLP		(0 << RCC_PLLCFGR_PLLP_Pos)	/* PLLP == 2 */
#else
#define	PLL_CFG_PLLP		(3 << RCC_PLLCFGR_PLLP_Pos)	/* PLLP == 8 */
#endif
#define	PLL_CFG_PLLQ		(7 << RCC_PLLCFGR_PLLQ_Pos)
#define	PLL_CFG_168M		(RCC_PLLCFGR_PLLSRC_HSE | PLL_CFG_PLLM | \
				 PLL_CFG_PLLN | PLL_CFG_PLLP | PLL_CFG_PLLQ)
#define	CLK_AHB_DIVISOR		RCC_CFGR_HPRE_DIV1
#define	CLK_APB1_DIVISOR	RCC_CFGR_PPRE1_DIV4
#define	CLK_APB2_DIVISOR	RCC_CFGR_PPRE2_DIV2

uint32_t SystemCoreClock;
static void *stm32_console_usart_cookie;

/* GPIO pin configuration. */
SOC_GPIO_PIN(LED_RED, D, 12)	/* Active high. */
SOC_GPIO_PIN(LED_AMBER, D, 13)
SOC_GPIO_PIN(LED_GREEN, D, 14)

/* Console port. USART2. */
SOC_GPIO_PIN(CONSOLE_RX, D, 6)		/* USART2 AF7, input */
SOC_GPIO_PIN(CONSOLE_TX, D, 5)		/* USART2 AF7, output */

/* Console Tx ringbuffer length. */
#define	STM32_CONSOLE_RB_LEN	512u

static void
init_hse(void)
{

	/* Ensure RCC is powered. */
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	(void) RCC->APB1ENR;
	__DSB();
	__ISB();

	/*
	 * If, for whatever reason, the system clock source is already
	 * HSE or PLL, switch back to HSI so we can start afresh.
	 */
	switch (RCC->CFGR & RCC_CFGR_SWS) {
	case RCC_CFGR_SWS_HSE:
	case RCC_CFGR_SWS_PLL:
		/* Ensure HSI is running. */
		RCC->CR |= RCC_CR_HSION;
		while ((RCC->CR & RCC_CR_HSIRDY) == 0)
			;
		/* Select HSI as system clock source. */
		RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSI;
		while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI)
			;
		break;

	default:
		break;
	}

	/* Enable the HSE. */
	RCC->CR |= RCC_CR_HSEON;
	RCC->CR |= RCC_CR_HSEBYP;
	while ((RCC->CR & RCC_CR_HSERDY) == 0)
		;

	/* Ensure PLL is off before we configure it. */
	RCC->CR &= ~RCC_CR_PLLON;
	while ((RCC->CR & RCC_CR_PLLRDY) != 0)
		;

	/* Configure the PLL to generate the System Clock. */
	RCC->PLLCFGR = PLL_CFG_168M;

	/* Enable the PLL. */
	RCC->CR |= RCC_CR_PLLON;
	while ((RCC->CR & RCC_CR_PLLRDY) == 0)
		;

	/* Before switching sysclock to PLL, set Flash wait states. */
	FLASH->ACR = FLASH_ACR_LATENCY_5WS;
	(void)FLASH->ACR;
	__DSB();
	__ISB();

	/* Reset ICache */
	FLASH->ACR = FLASH_ACR_LATENCY_5WS | FLASH_ACR_ICRST;
	(void)FLASH->ACR;
	__DSB();
	__ISB();
	FLASH->ACR = FLASH_ACR_LATENCY_5WS;
	(void)FLASH->ACR;
	__DSB();
	__ISB();

	/* Reset DCache */
	FLASH->ACR = FLASH_ACR_LATENCY_5WS | FLASH_ACR_DCRST;
	(void)FLASH->ACR;
	__DSB();
	__ISB();
	FLASH->ACR = FLASH_ACR_LATENCY_5WS;
	(void)FLASH->ACR;
	__DSB();
	__ISB();

	/* Enable caches */
	FLASH->ACR = FLASH_ACR_LATENCY_5WS | FLASH_ACR_ICEN | FLASH_ACR_DCEN;
	__DSB();
	__ISB();
	(void)FLASH->ACR;

	/* To avoid illegal APB clock speed, set them at the slowest speed. */
	RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2)) |
	    RCC_CFGR_PPRE1_DIV16 | RCC_CFGR_PPRE2_DIV16;

	/* AHB clock. */
	RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_HPRE) | CLK_AHB_DIVISOR;

#if 0
	/* Enable overdrive */
	PWR->CR |= (uint32_t)PWR_CR_ODEN;
	while ((PWR->CSR & PWR_CSR_ODRDY) == 0)
		;
	PWR->CR |= (uint32_t)PWR_CR_ODSWEN;
	while ((PWR->CSR & PWR_CSR_ODSWRDY) == 0)
		;
#endif

	/* SysClk configuration */
	RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
		;

	/* APB clocks. */
	RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2)) |
	    RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;

	SystemCoreClock = CLK_AHB_RATE;
}

static void
console_usart_irq(void)
{

	stm32_usart_interrupt(stm32_console_usart_cookie);
}

static ringbuff_t
attach_console(void)
{
	stm32_usart_attach_args_t aa;

	SOC_GPIO_CONSOLE_TX_out();
	SOC_GPIO_CONSOLE_TX_function(7);
	SOC_GPIO_CONSOLE_RX_in();
	SOC_GPIO_CONSOLE_RX_pullup(1);
	SOC_GPIO_CONSOLE_RX_function(7);

	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	__DSB();
	__ISB();
	(void)RCC->APB1ENR;
	__DSB();

	aa.aa_clock = CLK_APB1_RATE;
	aa.aa_baud = 115200;
	aa.aa_regs = USART2;
	aa.aa_rb_tx = ringbuff_alloc(STM32_CONSOLE_RB_LEN);
	aa.aa_rb_rx = NULL;	/* No Rx for now. */

	stm32_console_usart_cookie = stm32_usart_init(&aa);
	assert(stm32_console_usart_cookie != NULL);

	NVIC_SetPriority(USART2_IRQn, 3);
	cpu_install_irq_handler(USART2_IRQn, console_usart_irq);

	return aa.aa_rb_tx;
}

ringbuff_t
soc_init(void)
{

	init_hse();

	/* Enable GPIO/SYSCFG/PWR clocks. */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN |
	    RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN |
	    RCC_AHB1ENR_GPIOFEN | RCC_AHB1ENR_GPIOGEN | RCC_AHB1ENR_GPIOHEN |
	    RCC_AHB1ENR_GPIOIEN | RCC_AHB1ENR_GPIOJEN | RCC_AHB1ENR_GPIOKEN;
	(void)RCC->AHB1ENR;
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	(void)RCC->AHB2ENR;
	__DSB();
	__ISB();

	SOC_GPIO_LED_RED_out();
	SOC_GPIO_LED_RED_clr();
	SOC_GPIO_LED_AMBER_out();
	SOC_GPIO_LED_AMBER_clr();
	SOC_GPIO_LED_GREEN_out();
	SOC_GPIO_LED_GREEN_clr();

	/* Prime arc4random(3) entropy pool. */
	uint32_t v;
	(void) getentropy(&v, sizeof(v));

	/* Set SysTick priority and start it up */
	SysTick->LOAD = (SystemCoreClock / TIMER_HZ) - 1u;
	SysTick->VAL = (SystemCoreClock / TIMER_HZ) - 1u;
	NVIC_SetPriority(SysTick_IRQn, 1);
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk |
	    SysTick_CTRL_ENABLE_Msk;

	/* Initialise serial console, returning ringbuff reference. */
	return attach_console();
}

/*
 * Poor man's PWM of the onboard LEDs.
 * They're connected to GPIO pins with TIM4_CH[123] listed as alternate
 * functions, so PWM could probably be automated. This is left as an exercise
 * for the reader...
 */
struct led_state {
	int ls_count;
	int ls_duty;
	int ls_dir;
};

static struct led_state led_states[3] = {
	{0, 0, 1},
	{0, 5, 1},
	{0, 9, 1}
};

static bool
led_update(struct led_state *ls)
{
	bool rv;

	rv = (ls->ls_count++ % 10) <= ls->ls_duty;

	if (ls->ls_count == 100) {
		ls->ls_count = 0;
		ls->ls_duty += ls->ls_dir;
		if (ls->ls_duty == 10u || ls->ls_duty < 0) {
			ls->ls_dir = -ls->ls_dir;
			ls->ls_duty += ls->ls_dir;
		}
	}

	return rv;
}

extern void SysTick_Handler(void);
void
SysTick_Handler(void)
{

	if (led_update(&led_states[0]))
		SOC_GPIO_LED_RED_set();
	else
		SOC_GPIO_LED_RED_clr();

	if (led_update(&led_states[1]))
		SOC_GPIO_LED_AMBER_set();
	else
		SOC_GPIO_LED_AMBER_clr();

	if (led_update(&led_states[2]))
		SOC_GPIO_LED_GREEN_set();
	else
		SOC_GPIO_LED_GREEN_clr();

	timer_tick();
}

void
soc_update(void)
{

	/* Invoked from the main loop. Currently does nothing. */
}
