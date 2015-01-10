PROJECT = main

OUTDIR = build
TOOLDIR = tool

EXECUTABLE = $(OUTDIR)/$(PROJECT).elf
BIN_IMAGE = $(OUTDIR)/$(PROJECT).bin
HEX_IMAGE = $(OUTDIR)/$(PROJECT).hex
MAP_FILE = $(OUTDIR)/$(PROJECT).map
LIST_FILE = $(OUTDIR)/$(PROJECT).lst

TARGET = $(PROJECT)

SRCDIR += \
    port \
    src \
    src/sensor \
    src/controller \
    src/shell \

INCDIR += \
    inc \

SRC += \
    $(wildcard $(addsuffix /*.c,$(SRCDIR))) \
    $(wildcard $(addsuffix /*.s,$(SRCDIR))) \

# Toolchain, Library settings
MAK = $(wildcard mk/*.mk)
include $(MAK)

INCLUDES = $(addprefix -I,$(INCDIR))
OBJS := $(addprefix $(OUTDIR)/,$(patsubst %.s,%.o,$(SRC:.c=.o)))
DEPENDS = $(addsuffix .d,$(OBJS))

all: $(BIN_IMAGE)

$(BIN_IMAGE): $(EXECUTABLE)
	$(OBJCOPY) -O binary $^ $@
	$(OBJCOPY) -O ihex $^ $(HEX_IMAGE)
	$(OBJDUMP) -h -S -D $(EXECUTABLE) > $(LIST_FILE)
	$(SIZE) $(EXECUTABLE)

$(EXECUTABLE): $(OBJS) $(DAT)
	@echo " LD      "$@
	@$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIB_FILE) -Wl,--start-group -lgcc -lg -lc -lm -lnosys -Wl,--end-group

$(OUTDIR)/%.o: %.s
	@mkdir -p $(dir $@)
	@echo " CC      "$@
	@$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

$(OUTDIR)/%.o: %.c
	@mkdir -p $(dir $@)
	@echo " CC      "$@
	@$(CC) $(CFLAGS) $(INCLUDES) -MMD -MF $@.d -c $< -o $@

flash:
	st-flash write $(BIN_IMAGE) 0x8000000

openocd_flash:
	openocd \
	-f board/stm32f429discovery.cfg \
	-c "init" \
	-c "reset init" \
	-c "flash probe 0" \
	-c "flash info 0" \
	-c "flash write_image erase $(BIN_IMAGE)  0x08000000" \
	-c "reset run" -c shutdown

debug: $(EXECUTABLE)
	st-util &
	$(GDB) -x $(TOOLDIR)/gdbscript

openocd_debug: $(EXECUTABLE)
	openocd -f board/stm32f429discovery.cfg >/dev/null & \
	echo $$! > $(OUTDIR)/openocd_pid && \
	$(CROSS_COMPILE)gdb -x $(TOOLDIR)/gdbscript_openocd && \
	cat $(OUTDIR)/openocd_pid |`xargs kill 2>/dev/null || test true` && \
	rm -f $(OUTDIR)/openocd_pid

clang_complete:
	echo "$(INCLUDES) -std=c++11 -DSTM32F429xx -Wdeprecated-register" > .clang_complete_test

clean:
	rm -rf $(EXECUTABLE)
	rm -rf $(BIN_IMAGE)
	rm -rf $(HEX_IMAGE)
	rm -f $(OBJS)
	rm -f $(DEPENDS)
	rm -f $(LIST_FILE)

# -include $(DEPENDS)
.PHONY: all clean flash openocd_flash debug clang_complete
