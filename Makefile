MSP_GNU_ROOT := /Users/timcarlo/ti/msp430-gcc
PREFIX := $(MSP_GNU_ROOT)/bin/msp430-elf-
SUPPORT_FILE_DIRECTORY := $(MSP_GNU_ROOT)/include
MSPFlasherPath := /Users/timcarlo/ti/MSPFlasher_1.3.20

ifndef MSP_GNU_ROOT
define GNU_ERROR_BODY

*******************************************************************************
MSP_GNU_ROOT environment variable is not set. Examples given:
(Linux) export MSP_GNU_ROOT=/path/to/msp430-gcc/

Link to gcc: https://www.ti.com/tool/MSP430-GCC-OPENSOURCE#downloads
*******************************************************************************

endef
$(error $(GNU_ERROR_BODY))
endif

# Directories
OUTPUT_DIR := build
SRC_DIR := src
INCLUDE_DIR := include
#LINKER_SCRIPT := linker.ld
DEVICE := MSP430FR5994

# Source files
CSRC_FILES := $(wildcard $(SRC_DIR)/*.c)
OBJ_FILES := $(notdir $(CSRC_FILES:.c=.o))
OBJ_FILES := $(addprefix $(OUTPUT_DIR)/, $(OBJ_FILES))

# Dependency files
DEP_FILES := $(OBJ_FILES:.o=.d)

# Include paths
INC_FOLDERS := $(SUPPORT_FILE_DIRECTORY) $(INCLUDE_DIR)
INCLUDES := $(INC_FOLDERS:%=-I%)

# Compiler flags
CFLAGS += -O0 -g3 -Wall -mlarge -mmcu=$(DEVICE) $(INCLUDES)
CFLAGS += -MMD -MP  # for dependency generation

# Linker flags
LDFLAGS += $(CFLAGS)
LDFLAGS += -L $(SUPPORT_FILE_DIRECTORY)
LDFLAGS += -Wl,-Map,$(OUTPUT_DIR)/build.map,--gc-sections
#LDFLAGS += -T$(LINKER_SCRIPT)

.PHONY: all clean flash debug

all: | $(OUTPUT_DIR) $(OUTPUT_DIR)/build.hex $(OUTPUT_DIR)/build.elf

$(OUTPUT_DIR):
	mkdir -p $(OUTPUT_DIR)

# Compile C source to .o with dependency generation
$(OUTPUT_DIR)/%.o: $(SRC_DIR)/%.c
	@mkdir -p $(dir $@)
	$(PREFIX)gcc $(CFLAGS) -c $< -o $@
	@echo "CC $<"

# Assemble .S files (optional)
$(OUTPUT_DIR)/%.o: $(SRC_DIR)/%.S
	$(PREFIX)gcc $(CFLAGS) -c $< -o $@
	@echo "AS $<"

# Link
$(OUTPUT_DIR)/build.elf: $(OBJ_FILES)
	$(PREFIX)gcc $(LDFLAGS) $^ -o $@
	@$(PREFIX)size $@

# Convert ELF to HEX
$(OUTPUT_DIR)/build.hex: $(OUTPUT_DIR)/build.elf
	@$(PREFIX)objcopy -O ihex $< $@
	@echo "Preparing $@"

# Flash using MSP430Flasher
flash: $(OUTPUT_DIR)/build.hex
	@echo Flashing with MSP430Flasher...
	DYLD_LIBRARY_PATH=$(MSPFlasherPath) \
	$(MSPFlasherPath)/MSP430Flasher \
		-n $(DEVICE) \
		-w $< \
		-v \
		-g
	@echo "Flashing complete."
	@echo "Press Restart to run the code"

# Clean build artifacts
clear:
	rm -rf $(OUTPUT_DIR)/*

# Debug (adjust GDB if needed)
debug: all
	$(GDB) $(DEVICE).out

# Include generated dependency files (.d)
-include $(DEP_FILES)