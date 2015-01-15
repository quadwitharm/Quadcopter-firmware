# Toolchain configurations
CROSS_COMPILE ?= arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
AR = $(CROSS_COMPILE)ar
CXX = $(CROSS_COMPILE)g++
LD = $(CROSS_COMPILE)ld
OBJCOPY = $(CROSS_COMPILE)objcopy
OBJDUMP = $(CROSS_COMPILE)objdump
SIZE = $(CROSS_COMPILE)size
GDB = $(CROSS_COMPILE)gdb

# Cortex-M4 implements the ARMv7E-M architecture
CPU = cortex-m4
CFLAGS += -mcpu=$(CPU) -march=armv7e-m -mtune=cortex-m4
CFLAGS += -mlittle-endian -mthumb
CFLAGS += -mfpu=fpv4-sp-d16 -mfloat-abi=hard

# Math library

CFLAGS += -DARM_MATH_CM4 -D__FPU_PRESENT

# Basic configurations
CFLAGS += -g -std=c11
CFLAGS += -Wall #-Werror
CFLAGS += -fno-builtin
CFLAGS += -DUSER_NAME=\"$(USER)\"

# Optimizations
CFLAGS += -O0 -ffast-math
CFLAGS += --lto
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += -fno-common
CFLAGS += --param max-inline-insns-single=1000

# C++ Specific Flags

CXXFLAGS += -fno-exceptions
CXXFLAGS += -fno-rtti
CXXFLAGS += -nostdlib

# TODO: Add __cxa_pure_virtual() for additionally call to a pure virtual function
# (inside extern "C", this function should not be mangled)

# to run from FLASH
CFLAGS += -DVECT_TAB_FLASH

# Linker script
LD_SCRIPT = $(PWD)/port/STM32F429ZI_FLASH.ld
LDFLAGS += -Wl,--gc-sections -Wl,-Map=$(MAP_FILE) -T$(LD_SCRIPT)
