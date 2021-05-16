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
 * This is based on the hal_gpio.h for SAM4S by Alex Taradov
 * (https://github.com/ataradov/mcu-starter-projects). So while this 'port'
 * is all mine, the original author deserves some credit. Hence the copyright
 * below.
 */

/*
 * Copyright (c) 2014-2016, Alex Taradov <alex@taradov.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef SOC_GPIO_H
#define SOC_GPIO_H

#include "soc_header.h"

#define SOC_GPIO_PIN(name, port, pin)						\
	static __always_inline void SOC_GPIO_##name##_set(void)			\
	{									\
		GPIO##port->BSRR = 1u << (pin);					\
	}									\
										\
	static __always_inline void SOC_GPIO_##name##_clr(void)			\
	{									\
		GPIO##port->BSRR = 0x10000u << (pin);				\
	}									\
										\
	static __always_inline void SOC_GPIO_##name##_toggle(void)		\
	{									\
		if (GPIO##port->ODR & (1u << (pin)))				\
			GPIO##port->BSRR = 0x10000u << (pin);			\
		else								\
			GPIO##port->BSRR = 1u << (pin);				\
	}									\
										\
	static __always_inline void SOC_GPIO_##name##_write(int value)		\
	{									\
		if (value)							\
			GPIO##port->BSRR = 1u << (pin);				\
		else								\
			GPIO##port->BSRR = 0x10000u << (pin);			\
	}									\
										\
	static __always_inline void SOC_GPIO_##name##_in(void)			\
	{									\
		GPIO##port->MODER &= ~(3u << ((pin) * 2u));			\
	}									\
										\
	static __always_inline void SOC_GPIO_##name##_out(void)			\
	{									\
		uint32_t x = GPIO##port->MODER & ~(3u << ((pin) * 2u));		\
		GPIO##port->MODER = x | (1u << ((pin) * 2u));			\
	}									\
										\
	static __always_inline void SOC_GPIO_##name##_speed(uint32_t speed)	\
	{									\
		uint32_t x = GPIO##port->OSPEEDR & ~(3u << ((pin) * 2u));	\
		GPIO##port->OSPEEDR = x | (speed << ((pin) * 2u));		\
	}									\
										\
	static __always_inline int SOC_GPIO_##name##_is_output(void)		\
	{									\
		return ((GPIO##port->MODER >> ((pin) * 2u)) & 3u) != 0;		\
	}									\
										\
	static __always_inline void SOC_GPIO_##name##_pullup(int on)		\
	{									\
		uint32_t x = GPIO##port->PUPDR & ~(3u << ((pin) * 2u));		\
		if (on)								\
			x |= 1u << ((pin) * 2u);				\
		GPIO##port->PUPDR = x;						\
	}									\
										\
	static __always_inline void SOC_GPIO_##name##_pulldown(int on)		\
	{									\
		uint32_t x = GPIO##port->PUPDR & ~(3u << ((pin) * 2u));		\
		if (on)								\
			x |= 2u << ((pin) * 2u);				\
		GPIO##port->PUPDR = x;						\
	}									\
										\
	static __always_inline int SOC_GPIO_##name##_read(void)			\
	{									\
		return (GPIO##port->IDR & (1u << (pin))) != 0;			\
	}									\
										\
	static __always_inline int SOC_GPIO_##name##_state(void)		\
	{									\
		return (GPIO##port->ODR & (1u << (pin))) != 0;			\
	}									\
										\
	static __always_inline void SOC_GPIO_##name##_function(unsigned int af)	\
	{									\
		uint32_t x, m = 0xfu << (((pin) & 7u) * 4u);			\
		unsigned int idx = ((pin) & 8u) ? 1u : 0u;			\
		x = GPIO##port->AFR[idx] & ~m;					\
		GPIO##port->AFR[idx] = x | ((af << (((pin) & 7u) * 4u)) & m);	\
		x = GPIO##port->MODER & ~(3u << ((pin) * 2u));			\
		GPIO##port->MODER = x | (2u << ((pin) * 2u));			\
	}									\
  										\
	static __always_inline void SOC_GPIO_##name##_analog(void)		\
	{									\
		uint32_t x = GPIO##port->MODER & ~(3u << ((pin) * 2u));		\
		GPIO##port->MODER = x | (3u << ((pin) * 2u));			\
	}									\

#endif /* SOC_GPIO_H */
