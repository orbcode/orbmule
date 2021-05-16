#
# Copyright (c) 2021, Maverick Embedded Technology Ltd
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# Written for Maverick Embedded Technology Ltd by Steve C. Woodford.
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

NAME		:= orbdemo

CPU_TYPE	?= cortex-m4
FPU_TYPE	?= soft

#
# Select the underlying graphics library. Options are:
#  - UCG:    Small and simple colour graphics library.
#
GRAPHIC_LIBRARY	?= UCG

RELEASE_BUILD	?= 0
DEBUG		?= 0

SRCS		+= ../console.c
SRCS		+= ../display_gizmo_squares.c
SRCS		+= ../display_gizmo_worms.c
SRCS		+= ../main.c
SRCS		+= ../newlib_stubs.c
SRCS		+= ../ringbuff.c
SRCS		+= ../timer.c

##############################################################################

ifeq ($(RELEASE_BUILD),1)
DEFINES		+= -DRELEASE_BUILD=1
DEFINES		+= -DNDEBUG=1
else
ifeq ($(NDEBUG),1)
DEFINES		+= -DNDEBUG=1
endif
endif

EXTRAINCDIRS	= .. $(SOC_INC)

ifeq ($(FPU_TYPE),soft)
LDSPECS		= --specs=nano.specs
endif

#
# Fixed-point math support
#
FIX16_PATH	:= ../3rdparty/libfixmath
SRCS		+= $(FIX16_PATH)/fix16.c
SRCS		+= $(FIX16_PATH)/fix16_str.c
DEFINES		+= -DFIXMATH_NO_OVERFLOW=1
EXTRAINCDIRS	+= $(FIX16_PATH)

#
# ucglib support
#
ifeq ($(GRAPHIC_LIBRARY),UCG)
SRCS		+= ../display_ucg.c
SRCS		+= ../ucg_dev_ic_ili9488.c ../ucg_dev_tft_320x480_ili9488.c
UCG_PATH	:= ../3rdparty/ucglib/csrc
UCGSRCS		= $(UCG_PATH)/ucg_bitmap.c \
		  $(UCG_PATH)/ucg_box.c \
		  $(UCG_PATH)/ucg_ccs.c \
		  $(UCG_PATH)/ucg_circle.c \
		  $(UCG_PATH)/ucg_clip.c \
		  $(UCG_PATH)/ucg_com_msg_api.c \
		  $(UCG_PATH)/ucg_dev_default_cb.c \
		  $(UCG_PATH)/ucg_dev_msg_api.c \
		  $(UCG_PATH)/ucg_font.c \
		  $(UCG_PATH)/ucg_init.c \
		  $(UCG_PATH)/ucg_line.c \
		  $(UCG_PATH)/ucg_pixel.c \
		  $(UCG_PATH)/ucg_pixel_font_data.c \
		  $(UCG_PATH)/ucg_polygon.c \
		  $(UCG_PATH)/ucg_rotate.c \
		  $(UCG_PATH)/ucg_scale.c
SRCS += $(UCGSRCS)
EXTRAINCDIRS += $(UCG_PATH)
endif

#
# No user-servicable parts below
#

DEPDIR	:=	.dep

.PHONY: all clean size

COPT	?= -O3
TOOLCHAIN=arm-none-eabi
CC = $(TOOLCHAIN)-gcc
NM = $(TOOLCHAIN)-nm
OBJCOPY = $(TOOLCHAIN)-objcopy
OBJDUMP = $(TOOLCHAIN)-objdump
SIZE = $(TOOLCHAIN)-size
CTAGS = ctags

COMMON_FLAGS += -mcpu=$(CPU_TYPE) -mfloat-abi=$(FPU_TYPE) -mthumb
COMMON_FLAGS += $(CPU_FLAGS)
ifeq ($(FPU_TYPE),hard)
ifeq ($(CPU_TYPE),cortex-m4)
COMMON_FLAGS += -mfpu=fpv4-sp-d16
else
ifeq ($(CPU_TYPE),cortex-m7)
COMMON_FLAGS += -mfpu=fpv5-d16
endif
endif
endif
COMMON_FLAGS += -ffunction-sections -fdata-sections

CFLAGS += --std=gnu11
ifeq ($(DEBUG),0)
CFLAGS += $(COPT)
else
CFLAGS += -g
LDFLAGS += -g
DEFINES += -DDEBUG
endif
CFLAGS += $(COMMON_FLAGS)
CFLAGS += -MD -MP -MF $(DEPDIR)/$(@F).d
CFLAGS += -Werror -Wall -Wextra -Wpedantic
CFLAGS += -Wstrict-prototypes -Wmissing-prototypes -Wunused
CFLAGS += -Wcast-align -Wpointer-arith
CFLAGS += -Wmissing-declarations
CFLAGS += -Wwrite-strings
CFLAGS += -Wno-expansion-to-defined
CFLAGS += -Wno-attributes
CFLAGS += -Wshadow
CFLAGS += -funsigned-char -funsigned-bitfields
CFLAGS += -Wdouble-promotion -Wformat=2
CFLAGS += -Wmultistatement-macros
# Can't use the following because of ucglib...
#CFLAGS += -Wcast-qual

LDFLAGS += $(COMMON_FLAGS)
LDFLAGS += -Wl,--gc-sections -Wl,--relax $(LDSPECS)
LDFLAGS += -Wl,--script=$(SOC_LDSCRIPT)

INCLUDES += $(patsubst %,-I%,$(EXTRAINCDIRS))

DEFINES += -D$(SOC_CPP) -DDONT_USE_CMSIS_INIT  -DF_CPU=$(F_CPU)
CFLAGS += $(INCLUDES) $(DEFINES)

OBJS = $(SRCS:.c=.o) $(ASRCS:.S=.o)

all: $(NAME).bin $(NAME).lss $(NAME).sym size

%.bin:	%.elf
	$(OBJCOPY) -O binary $< $@

%.lss:	%.elf
	$(OBJDUMP) -h -S $< > $@

%.sym:	%.elf
	$(NM) -n $< > $@

$(NAME).elf: $(OBJS)
	$(CC) $(LDFLAGS) $(OBJS) $(LIBS) -o $@

%.o : %.S
	$(CC) -c $(ASFLAGS) $< -o $@

%.o : %.c
	$(CC) -c $(CFLAGS) $< -o $@

size: $(NAME).elf
	@$(SIZE) -t $^

clean:
	-rm -rf $(NAME).bin $(NAME).elf $(NAME).map $(NAME).sym
	-rm -rf $(NAME).lss $(OBJS) $(DEPDIR) tags

realclean:	clean

-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)
