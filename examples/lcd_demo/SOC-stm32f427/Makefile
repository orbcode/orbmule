#
# Copyright (c) 2021, Maverick Embedded Technology Ltd
# All rights reserved.
#
# Written for Maverick Embedded Technology Ltd by Steve C. Woodford.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Neither the names of the copyright holders nor the names of their
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

SOCDEF		:= stm32f427
SOC_CPP		:= STM32F427xx
SOC_DIR		:= ../SOC-$(SOCDEF)
SOC_INC		:= $(SOC_DIR) ../3rdparty/STMicro/STM32F4xx ../3rdparty/CMSIS
SOC_LDSCRIPT	:= $(SOC_DIR)/ldscript-$(SOCDEF).ld

F_CPU		:= 168000000
CPU_TYPE	:= cortex-m4
FPU_TYPE	:= soft
#DEBUG		?= 1

SRCS		+= stm32_entropy.c
SRCS		+= stm32_hal.c
SRCS		+= stm32_usart.c
SRCS		+= ../cpu_cortexm.c
SOC_UCG_SRCS	+= stm32_display_ucg.c
SOC_ORBLCD_SRCS	+= stm32_display_orblcd.c

include ../Makefile.inc
