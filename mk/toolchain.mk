# Toolchain configurations
CROSS_COMPILE ?= arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
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
CFLAGS += -mfpu=fpv4-sp-d16 -mfloat-abi=softfp

# Basic configurations
CFLAGS += -g -std=c99
CFLAGS += -Wall
CFLAGS += -DUSER_NAME=\"$(USER)\"

# Optimizations
CFLAGS += -O0 -ffast-math
CFLAGS += --lto
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += -fno-common
CFLAGS += --param max-inline-insns-single=1000

# to run from FLASH
CFLAGS += -DVECT_TAB_FLASH

# Linker script
LD_SCRIPT = $(PWD)/port/STM32F429ZI_FLASH.ld
LDFLAGS += -Wl,--gc-sections -Wl,-Map=$(MAP_FILE) -T$(LD_SCRIPT)
