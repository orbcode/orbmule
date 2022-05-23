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

OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)
SEARCH_DIR(.)

ENTRY(Reset_Handler)

/* Flash and SRAM... */
MEMORY {
	flash	(rx)	: ORIGIN = 0x08000000, LENGTH = 2M
	sram	(rwx)	: ORIGIN = 0x20000000, LENGTH = 192K
	/* CCM is unused. */
}

/* Section Definitions */
SECTIONS {
	.text : {
		FILL(0xff)
		. = ALIGN(4);
		KEEP(*(.vectors .vectors.*))
		*(.text .text.* .gnu.linkonce.t.*)
		*(.glue_7t) *(.glue_7)
		*(.rodata .rodata* .gnu.linkonce.r.*)
		*(.ARM.extab* .gnu.linkonce.armextab.*)

		. = ALIGN(4);
		KEEP(*(.init))
		. = ALIGN(4);
		__preinit_array_start = .;
		KEEP (*(.preinit_array))
		__preinit_array_end = .;

		. = ALIGN(4);
		__init_array_start = .;
		KEEP (*(SORT(.init_array.*)))
		KEEP (*(.init_array))
		__init_array_end = .;

		. = ALIGN(4);
		KEEP (*crtbegin.o(.ctors))
		KEEP (*(EXCLUDE_FILE (*crtend.o) .ctors))
		KEEP (*(SORT(.ctors.*)))
		KEEP (*crtend.o(.ctors))

		. = ALIGN(4);
		KEEP(*(.fini))

		. = ALIGN(4);
		__fini_array_start = .;
		KEEP (*(.fini_array))
		KEEP (*(SORT(.fini_array.*)))
		__fini_array_end = .;

		KEEP (*crtbegin.o(.dtors))
		KEEP (*(EXCLUDE_FILE (*crtend.o) .dtors))
		KEEP (*(SORT(.dtors.*)))
		KEEP (*crtend.o(.dtors))

		. = ALIGN(4);
	} > flash

	PROVIDE_HIDDEN (__exidx_start = .);
	.ARM.exidx : {
		*(.ARM.exidx* .gnu.linkonce.armexidx.*)
	} > flash
	PROVIDE_HIDDEN (__exidx_end = .);

	. = ALIGN(4);
	_etext = .;

	/* Initialised data section. */
	.data : {
		FILL(0xff)
		. = ALIGN(4);
		_sdata = .;
		*(.ramfunc .ramfunc.*);
		*(.data .data.*);
		. = ALIGN(4);
		_edata = .;
	} > sram AT> flash

	/* BSS section */
	.bss (NOLOAD) : {
		. = ALIGN(4);
		_sbss = . ;
		*(.bss .bss.*)
		*(COMMON)
		. = ALIGN(4);
		_ebss = . ;
	} > sram

	/*
	 * Stack section.
	 * Note: Do not be tempted to place this in BSS!
	 */
	.stack (NOLOAD): {
		. = ALIGN(8);
		_sstack = .;
		/* . = . + 0x800;*/	/* 2 KB stack is more than enough */
                . = + 0x20000;
		. = ALIGN(8);
		_estack = .;
	} > sram

	/* Start and end of SRAM heap. */
	. = ALIGN(4);
	_end = . ;
	_ram_end_ = ORIGIN(sram) + LENGTH(sram) - 1 ;
}
