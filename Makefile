# Cross-compilation commands
CC      = arm-none-eabi-gcc
LD      = arm-none-eabi-gcc
AS      = arm-none-eabi-as
OBJCOPY = arm-none-eabi-objcopy
SIZE    = arm-none-eabi-size

# Target information
CPU = cortex-m4
BOARD = board/stm32f4discovery.cfg

# Directories
ROOT_DIR = $(abspath $(lastword $(MAKEFILE_LIST)))
INC_DIR = $(ROOT_DIR)/inc
SRC_DIR = $(ROOT_DIR)/src
BUILD_DIR = $(ROOT_DIR)/build
LINKER_DIR = $(ROOT_DIR)/STM32CubeF4/Projects/STM32F4-Discovery/Templates_LL/STM32CubeIDE/STM32F407VGTX_FLASH.ld
STM32F4XX_DIR = $(ROOT_DIR)/STM32CubeF4/Drivers/CMSIS/Device/ST/STM32F4xx
STM32F4XX_INC_DIR = $(STM32F4XX_DIR)/Include
STM32F4XX_SRC_DIR = $(STM32F4XX_DIR)/Source/Templates
STM32F4XX_HAL_DIR = $(ROOT_DIR)STM32CubeF4/Drivers/STM32F4xx_HAL_Driver
STM32F4XX_HAL_INC_DIR = $(STM32F4XX_HAL_DIR)/Inc
STM32F4XX_HAL_SRC_DIR = $(STM32F4XX_HAL_DIR)/Src
FREERTOS_DIR = $(ROOT_DIR)/STM32CubeF4/Middlewares/Third_Party/FreeRTOS
FREERTOS_SRC_DIR = $(FREERTOS_DIR)/Source
FREERTOS_INC_DIR = $(FREERTOS_SRC_DIR)/include

# Project information
PROJECT = freertos-shell
ELF = $(BUILD_DIR)/$(PROJECT).elf
BIN = $(BUILD_DIR)/$(PROJECT).bin

# Debugger information
DEBUGGER = openocd
FLASH_TARGET = $(DEBUGGER) -f $(BOARD)
FLASH_CMD = "program $(ELF) verify reset exit"
FLASH = $(FLASH_TARGET) -c $(FLASH_CMD)

# Linker
LD = $(LINKER_DIR)/STM32F407VGTX_FLASH.ld

# Startup files
SRC = $(STM32F4XX_SRC_DIR)/gcc/startup_stm32f407xx.s

# Project source files
SRC += $(SRC_DIR)/main.c
SRC += $(CMSIS_DIR)/cmsis_os.c


# Compilation flags
CFLAGS  = -Wall -O0 -g
CFLAGS += -mcpu=$(CPU) -mthumb -mfloat-abi=soft
CFLAGS +=  --specs=nano.specs -nostdlib

# Include folders
CFLAGS += -I $(INC_DIR)
CFLAGS += -I $(FREERTOS_INC_DIR)
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