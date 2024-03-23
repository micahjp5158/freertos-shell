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
ROOT_DIR = $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
INC_DIR = $(ROOT_DIR)/inc
SRC_DIR = $(ROOT_DIR)/src
BUILD_DIR = $(ROOT_DIR)/build
OBJ_DIR = $(BUILD_DIR)/objects
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

# Linker
LD = $(STM32F4_DISCOVERY_DIR)/STM32CubeIDE/STM32F407VGTX_FLASH.ld

# Assembly source files
ASM_SRC = $(STM32F4_DISCOVERY_DIR)/STM32CubeIDE/Example/Startup/startup_stm32f407vgtx.s

# Project source files
C_SRC  = $(SRC_DIR)/main.c
C_SRC += $(SRC_DIR)/syscalls.c
C_SRC += $(SRC_DIR)/sysmem.c
C_SRC += $(SRC_DIR)/uart_shell.c
C_SRC += $(SRC_DIR)/stm32f4xx_it.c
C_SRC += $(SRC_DIR)/system_stm32f4xx.c
C_SRC += $(STM32F4_HAL_DIR)/Src/stm32f4xx_hal.c
C_SRC += $(STM32F4_HAL_DIR)/Src/stm32f4xx_hal_cortex.c
C_SRC += $(STM32F4_HAL_DIR)/Src/stm32f4xx_hal_dma.c
C_SRC += $(STM32F4_HAL_DIR)/Src/stm32f4xx_hal_gpio.c
C_SRC += $(STM32F4_HAL_DIR)/Src/stm32f4xx_hal_uart.c
C_SRC += $(STM32F4_HAL_DIR)/Src/stm32f4xx_hal_rcc.c

# Object files - generated from SRC lists
ASM_OBJ = $(addprefix $(OBJ_DIR)/,$(notdir $(ASM_SRC:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SRC)))
C_OBJ = $(addprefix $(OBJ_DIR)/,$(notdir $(C_SRC:.c=.o)))
vpath %.c $(sort $(dir $(C_SRC)))

# Compilation flags
CC_FLAGS  = -Wall -O0 -g
CC_FLAGS += -mcpu=$(CPU) -mthumb -mfloat-abi=soft
CC_FLAGS +=  --specs=nano.specs

# Preprocessor Defines
# STM32F4 Discovery board has MCU STM32F407VG
C_DEFS  = -DSTM32F407xx
# STM32F4 Discovery board has 8 MHz OSC for HSE
C_DEFS += -DHSE_VALUE=8000000

# Include folders
INC_FLAGS  = -I $(INC_DIR)
INC_FLAGS += -I $(STM32F4_HAL_DIR)/Inc
INC_FLAGS += -I $(STM32F4_HAL_DIR)/Inc/Legacy
INC_FLAGS += -I $(FREERTOS_DIR)/include
INC_FLAGS += -I $(FREERTOS_DIR)/portable/GCC/ARM_CM4F
INC_FLAGS += -I $(CMSIS_DIR)/Include
INC_FLAGS += -I $(CMSIS_DIR)/Device/ST/STM32F4xx/Include
INC_FLAGS += -I $(STM32F4_DISCOVERY_DIR)/Inc
INC_FLAGS += -T $(LD)

## Rules

# Default: Build project binary
default: $(BIN)

# Build project binary: needs project ELF
$(BIN): $(ELF)
	@echo [OBJCOBY] $@
	@$(OBJCOPY) -O binary $^ $@

# Build ELF: needs all ASM and C objects
$(ELF): $(ASM_OBJ) $(C_OBJ) | $(BUILD_DIR)
	@echo [CC] $@
	@$(CC) $(CC_FLAGS) $(ASM_OBJ) $(C_OBJ) $(INC_FLAGS) -o $@

# Compile all ASM objects from ASM sources
$(OBJ_DIR)/%.o: %.s | $(OBJ_DIR)
	@echo [CC] $(notdir $@): $<
	@$(CC) -c $(CC_FLAGS) $(INC_FLAGS) $(C_DEFS) $< -o $@

# Compile all C objects from C sources
$(OBJ_DIR)/%.o: %.c | $(OBJ_DIR)
	@echo [CC] $(notdir $@): $<
	@$(CC) -c $(CC_FLAGS) $(INC_FLAGS) $(C_DEFS) $< -o $@

# Create object directory: needs build directory
$(OBJ_DIR): | $(BUILD_DIR)
	mkdir -p $@

# Create build directory
$(BUILD_DIR):
	mkdir -p $@

flash:
	$(FLASH)

clean:
	rm -rf $(BUILD_DIR)