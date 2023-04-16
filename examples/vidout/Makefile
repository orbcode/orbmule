#Set to 1 to export LCD information over the ITM channel 
WITH_ORBLCD_MONITOR?=0

# Set to 1 to get busy factor information on a pin
WITH_BUSY_INDICATOR?=1

# Set to 1 to optimise the vidout subsystem irrespective of optimisation for the rest of the app
OPTIMISE_VIDOUT?=1

# Define which board we are building for
BOARD?=MULEF427

# Set to 1 for a verbose build
VERBOSE=0

# Set to 1 for a debug build
DEBUG?=0

##########################################################################
# User configuration and firmware specific object files
##########################################################################

CROSS_COMPILE ?= arm-none-eabi-

ifeq ($(BOARD),MULEF427)
  GCC_DEFINE=-DSTM32F427xI -DSTM32F427xx
  CORTEX_TYPE=m4
  LD_SCRIPT = config/stm32f427vi.ld
else
$(error Target is not defined.)
endif

PROJNAME=vidout

# Overall system defines for compilation
ifeq ($(DEBUG),1)
GCC_DEFINE+=-DDEBUG
OPT_LEVEL = 
VARIANT = "Debug"
else
GCC_DEFINE+= 
OPT_LEVEL = -O2
#fast
VARIANT = "Release"
endif

ifeq ($(WITH_ORBLCD_MONITOR),1)
GCC_DEFINE+=-DMONITOR_OUTPUT
endif

ifeq ($(WITH_BUSY_INDICATOR),1)
GCC_DEFINE+=-DBUSY_MONITOR
endif

ifeq ($(OPTIMISE_VIDOUT),1)
GCC_DEFINE+=-DOPTIMISE_VIDOUT
endif

LDLIBS = 
GCC_DEFINE+= -std=gnu99

# The target, flash and ram of the microprocessor.
CPU_TYPE = cortex-$(CORTEX_TYPE)

CFILES =
SFILES =
OLOC = ofiles
INCLUDE_PATHS = $(OLOC) /home/dmarples/Develop/orb/orbuculum/Inc
DEBUG_OPTS = -gdwarf

##########################################################################
# Generic multi-project files
##########################################################################

# CMSIS Files
# ===========
CMSIS_DIR=thirdparty/CMSIS

ifeq ($(BOARD),MULEF427)
INCLUDE_PATHS += $(CMSIS_DIR)/inc/stm32f427
CFILES += $(CMSIS_DIR)/src/core_cm3.c \
		  $(CMSIS_DIR)/src/system_stm32f4xx.c \
		  $(CMSIS_DIR)/src/sysclock_stm32f427xx.c
SFILES += $(CMSIS_DIR)/src/startup_stm32f427xx.s
endif

# Video Files
# ===========
VIDEO_DIR=vidout
INCLUDE_PATHS += $(VIDEO_DIR)
CFILES += $(VIDEO_DIR)/displayFile.c \
		  $(VIDEO_DIR)/rasterLine.c \
		  $(VIDEO_DIR)/vidout.c

# ITM Files
# =========
ITM_DIR=itm
INCLUDE_PATHS += $(ITM_DIR)
CFILES += $(ITM_DIR)/itm_messages.c

##########################################################################
# Project-specific files
##########################################################################

# Main Files
# ==========
App_DIR=app
App_Inc_DIR=app
INCLUDE_PATHS += $(App_Inc_DIR)

CFILES += $(App_DIR)/main.c $(App_DIR)/io.c

##########################################################################
# GNU GCC compiler prefix and location
##########################################################################

STYLE = clang-format
AS = $(CROSS_COMPILE)gcc
CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)gcc
GDB = $(CROSS_COMPILE)gdb
SIZE = $(CROSS_COMPILE)size
OBJCOPY = $(CROSS_COMPILE)objcopy
OBJDUMP = $(CROSS_COMPILE)objdump
MAKE = make
OUTFILE = firmware

##########################################################################
# Quietening
##########################################################################

ifeq ($(VERBOSE),1)
cmd = $1
Q :=
else
cmd = @$(if $(value 2),echo "$2";)$1
Q := @
endif

HOST=--specs=nano.specs -nostartfiles

##########################################################################
# Compiler settings, parameters and flags
##########################################################################

ARCH_FLAGS=-mcpu=cortex-m4

# Link for code size
GC=-Wl,--gc-sections

# Create map file
MAP=-Wl,-Map=$(OLOC)/$(OUTFILE).map,--cref

CFLAGS =  -D$(BOARD) $(ARCH_FLAGS) $(STARTUP_DEFS) $(OPT_LEVEL) $(DEBUG_OPTS) \
		-ffunction-sections -fdata-sections -Wdouble-promotion -Wall $(GCC_DEFINE)
ASFLAGS = -c $(DEBUG_OPTS) $(ARCH_FLAGS) -x assembler-with-cpp
LDFLAGS = $(CFLAGS) $(ARCH_FLAGS) -Wl,--no-wchar-size-warning,--gc-sections,--no-warn-rwx-segment $(MAP) $(HOST)

OCFLAGS = --strip-unneeded

OBJS =  $(patsubst %.c,%.o,$(CFILES)) $(patsubst %.s,%.o,$(SFILES))
POBJS = $(patsubst %,$(OLOC)/%,$(OBJS))
PDEPS =$(POBJS:.o=.d)

INCLUDE_FLAGS = $(foreach d, $(INCLUDE_PATHS), -I$d)

.SUFFIXES:

all : build

$(OLOC)/%.o : %.c
	$(Q)mkdir -p $(basename $@)
	$(call cmd, \$(CC) -c $(CFLAGS) $(PROFILING) $(INCLUDE_FLAGS) -MMD -o $@ $< ,\
	Compiling $<)

$(OLOC)/%.o : %.s
	$(Q)mkdir -p $(basename $@)
	$(call cmd, \$(AS) $(ASFLAGS) -o  $@ $< ,\
	Assembling $<)

build:  $(POBJS) $(SYS_OBJS) 
	$(Q)$(LD) -g $(LDFLAGS) -T $(LD_SCRIPT) $(MAP) $(POBJS) $(LDLIBS) -o $(OLOC)/$(OUTFILE).elf
	$(Q)$(SIZE) $(OLOC)/$(OUTFILE).elf
	$(Q)$(OBJCOPY) $(OCFLAGS) -O binary $(OLOC)/$(OUTFILE).elf $(OLOC)/$(OUTFILE).bin
	$(Q)$(OBJCOPY) $(OCFLAGS) -O ihex $(OLOC)/$(OUTFILE).elf $(OLOC)/$(OUTFILE).hex
	@echo " Built $(VARIANT) version for $(BOARD)"

clean:
	-$(call cmd, \rm -f $(POBJS) $(LD_TEMP) $(OUTFILE).elf $(OUTFILE).bin $(OUTFILE).hex $(OUTFILE).map $(EXPORT) ,\
	Cleaning )
	$(Q)-rm -rf *~ core
	$(Q)-rm -rf $(OLOC)/*
	$(Q)-rm -rf config/*~
	$(Q)-rm -rf TAGS

flash:	build
	$(Q) $(DEBUGGER)   $(DEBUGSCRIPT) 2> /dev/null &

print-%:
	@echo $* is $($*)

pretty:
	$(Q)-$(STYLE) -i -style=file $(CFILES)

-include $(PDEPS)
