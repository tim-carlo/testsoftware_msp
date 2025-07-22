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


OUTPUT_DIR := build
SRC_DIR := src
LINKER_SCRIPT:= linker.ld

DEVICE  = MSP430FR5994

CSRC_FILES += \
  blinky.c \
  printf.c \
  putchar.c \


OBJ_FILES = $(CSRC_FILES:.c=.o)

INC_FOLDERS += \
	$(SUPPORT_FILE_DIRECTORY) \

INCLUDES = $(INC_FOLDERS:%=-I%)

CFLAGS += -O0 -g3
CFLAGS += ${INCLUDES}
CFLAGS += -Wall
CFLAGS += -mlarge
#CFLAGS += -mcode-region=lower
#CFLAGS += -mdata-region=lower
CFLAGS += -mmcu=$(DEVICE)

LDFLAGS += ${CFLAGS}
LDFLAGS += -L $(SUPPORT_FILE_DIRECTORY)
LDFLAGS += -Wl,-Map,$(OUTPUT_DIR)/build.map,--gc-sections
#LDFLAGS += -T$(LINKER_SCRIPT)

.PHONY: clean all debug

all: ${OUTPUT_DIR}/build.hex ${OUTPUT_DIR}/build.elf

${OUTPUT_DIR}/%.o: ${SRC_DIR}/%.c
	${PREFIX}gcc ${CFLAGS} -c $< -o $@
	@echo "CC $<"

${OUTPUT_DIR}/%.o: ${SRC_DIR}/%.S
	${PREFIX}gcc ${CFLAGS} -c $< -o $@
	@echo "AS $<"

${OUTPUT_DIR}/build.elf: $(OBJ_FILES:%=${OUTPUT_DIR}/%)
	${PREFIX}gcc ${LDFLAGS} $^ -o $@
	@${PREFIX}size $@

${OUTPUT_DIR}/build.hex: ${OUTPUT_DIR}/build.elf
	@${PREFIX}objcopy -O ihex $< $@
	@echo "Preparing $@"

flash: ${OUTPUT_DIR}/build.hex
	@echo Flashing with MSP430Flasher...
	DYLD_LIBRARY_PATH=${MSPFlasherPath} \
	${MSPFlasherPath}/MSP430Flasher \
		-n ${DEVICE} \
		-w $< \
		-v \
		-g
	@echo "Flashing complete."
	@echo "Press Restart to run the code"


clean:
	rm -rf build/*

debug: all
	$(GDB) $(DEVICE).out