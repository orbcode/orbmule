VERBOSE?=0                             # Define for verbose builds
DEBUG=1                                # Define for debug and minimal optimisation

PROJECT=mtb

CROSS_COMPILE ?= $(HOME)/bin/armgcc/bin/arm-none-eabi-
GCC_DEFINE=
App_DIR=
System_DIR=system

##########################################################################
# User configuration and firmware specific object files
##########################################################################
# Overall system defines for compilation

CORTEX_TYPE=m0

GCC_DEFINE=

OLOC = ofiles
GCC_DEFINE+= -std=gnu99

ifdef DEBUG
GCC_DEFINE+=-DDEBUG
DEBUG_OPTS = -ggdb
OPT_LEVEL = -Og
else
DEBUG_OPTS =
OPT_LEVEL = -O2
endif

CPU_TYPE = cortex-$(CORTEX_TYPE)

CFILES =
AFILES =
SFILES =
INCLUDE_PATHS =

##########################################################################
# Project-specific files
##########################################################################

# Subdirectory Configuation
# =========================
App_DIR=.

# Main Files
# ==========
App_Inc_DIR=$(App_DIR)/inc
INCLUDE_PATHS += -I$(App_Inc_DIR) -I$(App_DIR)
INCLUDE_PATHS += -I$(OLOC)
CFILES += $(shell find $(App_DIR)/src -name "*.c" -print)

##########################################################################
# Generic multi-project files
##########################################################################

# System Files
# ============
AFILES +=$(System_DIR)/startup_ARMCM0.S
LDFLAGS =-T $(System_DIR)/linker.ld


##########################################################################
# GNU GCC compiler prefix and location
##########################################################################

ASTYLE = astyle
AS = $(CROSS_COMPILE)gcc
CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)gcc
GDB = $(CROSS_COMPILE)gdb
GIT = git
SIZE = $(CROSS_COMPILE)size
OBJCOPY = $(CROSS_COMPILE)objcopy
OBJDUMP = $(CROSS_COMPILE)objdump
MAKE = make
OUTFILE = $(PROJECT)

##########################################################################
# Quietening
##########################################################################

ifeq ('$(strip $(VERBOSE))','1')
cmd = $1
Q :=
else
cmd = @$(if $(value 2),echo "$2";)$1
Q := @
endif

ifdef DEBUG
SPEC=--specs=nosys.specs
HOST= -lgcc -lc -lnosys -lm
else
SPEC=--specs=nano.specs
HOST=-lc -lc -lnosys -lm
endif

##########################################################################
# Compiler settings, parameters and flags
##########################################################################
# filename for embedded git revision
GIT_HASH_FILENAME=git_version_info.h

# Options for specific architecture
ARCH_FLAGS=-mthumb -mcpu=$(CPU_TYPE) 

# Link for code size
GC=-Wl,--gc-sections

# Create map file
MAP=-Wl,-Map=$(OLOC)/$(OUTFILE).map,--cref

CFLAGS =  $(ARCH_FLAGS) $(STARTUP_DEFS) $(OPT_LEVEL) $(DEBUG_OPTS) \
		-ffunction-sections -fdata-sections -fmerge-constants -Wdouble-promotion -Wall \
		-fno-common -c -ffreestanding -fno-builtin  -fmacro-prefix-map="../$(@D)/"=. -fstack-usage \
		$(INCLUDE_PATHS)  $(GCC_DEFINE)

ASFLAGS = -c $(DEBUG_OPTS) $(INCLUDE_PATHS) $(ARCH_FLAGS) $(GCC_DEFINE) -x assembler-with-cpp
LDFLAGS += $(SPEC) $(ARCH_FLAGS) -Wl,--no-wchar-size-warning,--gc-sections -Wl,--sort-section=alignment $(MAP)

OCFLAGS = --strip-unneeded

OBJS     =  $(patsubst %.c,%.o,$(CFILES)) $(patsubst %.S,%.o,$(AFILES))
POBJS    = $(patsubst %,$(OLOC)/%,$(OBJS))

DEPFILES := $(POBJS:.o=.d)

DIRTY := $(shell $(GIT) diff-index --quiet HEAD; echo $$?)

.PHONY: get_version

all : build

$(OLOC)/%.o : %.c
	$(Q)mkdir -p $(basename $@)
	$(call cmd, \$(CC) -MMD -MP -c $(CFLAGS) -o $@ $< ,\
	Compiling $<)

$(OLOC): ; $(Q)mkdir -p $@

$(OLOC)/%.o : %.s
	$(Q)mkdir -p $(basename $@)
	$(call cmd, \$(AS) $(ASFLAGS) -o  $@ $< ,\
	Assembling $<)

$(OLOC)/%.o : %.S
	$(Q)mkdir -p $(basename $@)
	$(call cmd, \$(AS) $(ASFLAGS) -o  $@ $< ,\
	Assembling $<)

build: 	$(POBJS)
	$(Q)$(LD) $(LDFLAGS) $(GC) -o $(OLOC)/$(OUTFILE).elf $(MAP) $(POBJS) $(HOST) $(LDLIBS)
	$(Q)$(SIZE) $(OLOC)/$(OUTFILE).elf
	$(Q)$(OBJCOPY) $(OCFLAGS) -O binary $(OLOC)/$(OUTFILE).elf $(OLOC)/$(OUTFILE).bin
	$(Q)$(OBJCOPY) $(OCFLAGS) -O ihex $(OLOC)/$(OUTFILE).elf $(OLOC)/$(OUTFILE).hex

tags:
	-@etags $(CFILES) 2> /dev/null

clean:
	-$(call cmd, \rm -f $(POBJS) $(OUTFILE).elf $(OUTFILE).bin $(OUTFILE).hex $(OUTFILE).map ,\
	Cleaning )
	$(Q)-rm -rf *~ core
	$(Q)-rm -rf $(OLOC)
	$(Q)-rm -rf TAGS

print-%:
	@echo $* is $($*)

pretty:
	$(Q)$(ASTYLE) --style=gnu -n --quiet --recursive --indent=spaces=4 --indent-classes --indent-switches --indent-preproc-block --indent-col1-comments --break-closing-brackets --add-brackets --convert-tabs --keep-one-line-statements --indent-cases --max-code-length=120 --break-after-logical --convert-tabs "$(App_DIR)/*.h" $(App_DIR)/"*.c"

-include $(DEPFILES)
