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
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "soc.h"
#include "cpu_cortexm.h"

/* Symbols from linker script. */
extern uint32_t _etext;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;
extern uint32_t _estack;

/* Application and libc externals. */
extern int main(void);
extern void __libc_init_array(void);

/* Cortex-M vector table has strict alignment requirements. */
#define	VECTOR_PO2(c)	(((c) < 128) ? 128 : \
			 (((c) < 256) ? 256 : \
			 (((c) < 512) ? 512 : -1)))
#define	VECTOR_ALIGN	(VECTOR_PO2(SOC_IRQn_Type_COUNT) * sizeof(void *))
static_assert((int)VECTOR_ALIGN > 0, "SOC_IRQn_Type_COUNT out of range");

/* Runtime Vector Table */
typedef struct {
	cortexm_exception_handler_t vt_core[16];
	cortexm_exception_handler_t vt_devs[SOC_IRQn_Type_COUNT];
} cortexm_vector_table_t __attribute__((aligned(VECTOR_ALIGN)));

static cortexm_vector_table_t runtime_vector_table;

void Reset_Handler(void) __attribute__((used, naked));

static void
Default_Exception_Handler(void)
{
	int vec = (int)__get_IPSR() - 16;

	__disable_irq();
	printf("Exception %d\n", vec);
	for (;;);
}

static void
Spurious_Interrupt_Handler(void)
{
	int vec = (int)__get_IPSR() - 16;

	__disable_irq();
	printf("Spurious interrupt, vector %d\n", vec);
	for (;;);
}

#define	VECTOR_FUNC(f)	extern void f(void) \
	__attribute__((weak, alias("Default_Exception_Handler")))

/* Cortex-M core handlers */
VECTOR_FUNC(NonMaskableInt_Handler);
VECTOR_FUNC(HardFault_Handler);
VECTOR_FUNC(MemManagement_Handler);
VECTOR_FUNC(BusFault_Handler);
VECTOR_FUNC(UsageFault_Handler);
VECTOR_FUNC(SVCall_Handler);
VECTOR_FUNC(DebugMonitor_Handler);
VECTOR_FUNC(PendSV_Handler);
VECTOR_FUNC(SysTick_Handler);

static const cortexm_exception_handler_t init_vector_table[16]
    __attribute__((used, section(".vectors"))) = {
        /* Initial Stack Pointer, linker-generated. */
        [0]  = (cortexm_exception_handler_t)(uintptr_t)&_estack,
        [1]  = Reset_Handler,
        [2]  = NonMaskableInt_Handler,
        [3]  = HardFault_Handler,
        [4]  = MemManagement_Handler,
        [5]  = BusFault_Handler,
        [6]  = UsageFault_Handler,
        [7]  = NULL,
        [8]  = NULL,
        [9]  = NULL,
        [10] = NULL,
        [11] = SVCall_Handler,
        [12] = DebugMonitor_Handler,
        [13] = NULL,
        [14] = PendSV_Handler,
        [15] = SysTick_Handler,
};

void
Reset_Handler(void)
{
	uint32_t *src, *dest;

	/* Ensure MSP is initialised. */
	__set_MSP((uint32_t)(uintptr_t)init_vector_table[0]);

	/* Initialise mutable data section */
	src  = &_etext;
	dest = &_sdata;

	if (src != dest) {
		while (dest < &_edata)
			*dest++ = *src++;
	}

	/* Zero the BSS */
	for (dest = &_sbss; dest < &_ebss; dest++)
		*dest = 0;

#if __FPU_USED
	/* Enable FPU */
	SCB->CPACR |= (0xFu << 20);
	__DSB();
	__ISB();
#endif

	/* Initialise the C runtime */
	__libc_init_array();

	/* Configure the runtime vector table. */
	memcpy(runtime_vector_table.vt_core, init_vector_table,
	    sizeof(runtime_vector_table.vt_core));
	for (unsigned int i = 0; i < SOC_IRQn_Type_COUNT; i++)
		runtime_vector_table.vt_devs[i] = Spurious_Interrupt_Handler;
	SCB->VTOR = (uint32_t)(uintptr_t)&runtime_vector_table;

	/* Safe to enable interrupts. */
	__enable_irq();

	/* Invoke main function */
	main();

	/* XXX: Consider resetting device... */
	for (;;);
}

void
cpu_install_irq_handler(IRQn_Type irq, cortexm_exception_handler_t func)
{

	assert(irq >= (IRQn_Type)0 && irq <= (IRQn_Type)SOC_IRQn_Type_COUNT);

	runtime_vector_table.vt_devs[irq] = func;
	__DSB();
	__ISB();
	NVIC_EnableIRQ(irq);
}
