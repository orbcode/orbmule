Simple Test Firmware for Orbuculum "Mule" Board
===============================================

This test/demo firmware draws multi-coloured pulsating squares and worms
in a series of sub-windows on an attached TFT display, while also pulsating
the onboard LEDs.

Hardware Requirements:
 - The Orbuculum mule board based on on STM32F427,
 - A colour TFT display based on the ILI9488 chipset, with a SPI interface
   and a resolution of 480x320 pixels. Many of these will require a wire
   link to be soldered between the VCC and LED pins to provide power to
   the backlight.
   Other SPI displays can be used, so long as they are supported by UCGLIB.
   See DISPLAY_UCG_DEV and DISPLAY_UCG_EXT definitions in display_ucg.c.
 - A debug probe to program the firmware image onto the board.

Software Requirements:
 - GNU make
 - GNU ARM Embedded Toolchain from developer.arm.com
   (https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm)
   This includes a multi-arch build of newlib.
 - All other software dependencies are located under the 3rdparty directory.

Building the firmware:
 - cd SOC-stm32f427 && make
 - Use the debug probe to burn the resulting 'orbdemo.elf' file onto the board.

TFT Display Connections:
  Reset	PA2
  DC/RS	PA3
  /CS	PA4
  SCK	PA5
  MOSI	PA7

Serial Console Connections (8N1 @ 115200 baud):
  TxD	PD5
  RxD	PD6 (currently unused)

	Top View of Orb Mule Board
	==========================
	+--- USB ---+
	|           . TFT Gnd
	.*PWR       . n/c
	.*RED       . TFT 3v3
	.*AMBER     . PA7 TFT MOSI
	.*GREEN     . PA6 n/c
	.           . PA5 TFT SCK
	.           . PA4 TFT /CS
	.           . PA3 TFT DC/RS
	.           . PA2 TFT Reset
	|           |
	.           . PD5 Console TxD
	. DebugHDR  . PD6 Console RxD
	+-----------+

Misc:
The source code uses 8-column hard TABs, and follows NetBSD's style guide
(formerly known as KNF).

