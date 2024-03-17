# Cross-compilation commands
CC      = arm-none-eabi-gcc
LD      = arm-none-eabi-gcc
AS      = arm-none-eabi-as
OBJCOPY = arm-none-eabi-objcopy
SIZE    = arm-none-eabi-size

# Target information
CPU = cortex-m4
BOARD = board/stm32f4discovery.cfg
MCU = STM32F407xx

# Directories
ROOT_DIR = $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
INC_DIR = $(ROOT_DIR)/inc
SRC_DIR = $(ROOT_DIR)/src
BUILD_DIR = $(ROOT_DIR)/build
STM32CUBEF4_DIR = $(ROOT_DIR)/STM32CubeF4
LINKER_DIR = $(STM32CUBEF4_DIR)/Projects/STM32F4-Discovery/Templates_LL/STM32CubeIDE
STM32F4_HAL_DIR = $(STM32CUBEF4_DIR)/Drivers/STM32F4xx_HAL_Driver
FREERTOS_DIR = $(STM32CUBEF4_DIR)/Middlewares/Third_Party/FreeRTOS/Source
CMSIS_DIR = $(STM32CUBEF4_DIR)/Drivers/CMSIS
STM32F4_DISCOVERY_DIR = $(STM32CUBEF4_DIR)/Projects/STM32F4-Discovery/Templates_LL

# Project information
PROJECT = freertos-shell
ELF = $(BUILD_DIR)/$(PROJECT).elf
BIN = $(BUILD_DIR)/$(PROJECT).bin

# Debugger information
DEBUGGER = openocd
FLASH_TARGET = $(DEBUGGER) -f $(BOARD)
FLASH_CMD = "program $(ELF) verify reset exit"
FLASH = $(FLASH_TARGET) -c $(FLASH_CMD)

# TODO: Use linker and startup.s file from STM32CubeF4
# Linker
LD = $(SRC_DIR)/linker_stm32f407xx.ld

# Startup files
SRC = $(SRC_DIR)/startup_stm32f407xx.c

# Project source files
SRC += $(SRC_DIR)/main.c
SRC += $(CMSIS_DIR)/Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c

# Compilation flags
CFLAGS  = -Wall -O0 -g
CFLAGS += -mcpu=$(CPU) -mthumb -mfloat-abi=soft
CFLAGS +=  --specs=nano.specs -nostdlib

# Defines
CFLAGS += -D$(MCU)

# Include folders
CFLAGS += -I $(INC_DIR)
CFLAGS += -I $(STM32F4_HAL_DIR)/Inc
CFLAGS += -I $(STM32F4_HAL_DIR)/Inc/Legacy
CFLAGS += -I $(FREERTOS_DIR)/include
CFLAGS += -I $(FREERTOS_DIR)/portable/GCC/ARM_CM4F
CFLAGS += -I $(CMSIS_DIR)/Include
CFLAGS += -I $(CMSIS_DIR)/Device/ST/STM32F4xx/Include
CFLAGS += -I $(STM32F4_DISCOVERY_DIR)/Inc
CFLAGS += -T $(LD)

## Rules
default: $(BIN)

$(BIN): $(ELF)
	$(OBJCOPY) -O binary $^ $@

$(ELF): $(SRC) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $^ -o $@

$(BUILD_DIR):
	mkdir $(BUILD_DIR)

flash:
	$(FLASH)

clean:
	rm -rf $(BUILD_DIR)