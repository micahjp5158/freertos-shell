# Cross-compilation commands
CC      = arm-none-eabi-gcc
LD      = arm-none-eabi-gcc
AS      = arm-none-eabi-as
OBJCOPY = arm-none-eabi-objcopy
SIZE    = arm-none-eabi-size

# Target information
CPU = cortex-m4
FPU = fpv4-sp-d16
BOARD = board/stm32f4discovery.cfg

# Directories
ROOT_DIR = $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
INC_DIR = $(ROOT_DIR)/inc
SRC_DIR = $(ROOT_DIR)/src
SUBMODULES_DIR = $(ROOT_DIR)/submodules
STM32CUBEF4_DIR = $(SUBMODULES_DIR)/STM32CubeF4
CMSIS_DIR = $(STM32CUBEF4_DIR)/Drivers/CMSIS
FREERTOS_DIR = $(STM32CUBEF4_DIR)/Middlewares/Third_Party/FreeRTOS/Source
STM32F4_HAL_DIR = $(STM32CUBEF4_DIR)/Drivers/STM32F4xx_HAL_Driver
BUILD_DIR = $(ROOT_DIR)/build
OBJ_DIR = $(BUILD_DIR)/objects

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
LD = $(SRC_DIR)/STM32F407VGTX_FLASH.ld

# Assembly source files
ASM_SRC = $(SRC_DIR)/startup_stm32f407vgtx.s

# Project source files
C_SRC  = $(SRC_DIR)/main.c
C_SRC += $(SRC_DIR)/led.c
C_SRC += $(SRC_DIR)/stm32f4xx_it.c
C_SRC += $(SRC_DIR)/syscalls.c
C_SRC += $(SRC_DIR)/sysmem.c
C_SRC += $(SRC_DIR)/system_stm32f4xx.c
C_SRC += $(SRC_DIR)/uart_shell.c
C_SRC += $(SUBMODULES_DIR)/ringbuf/src/ringbuf.c
C_SRC += $(FREERTOS_DIR)/croutine.c
C_SRC += $(FREERTOS_DIR)/event_groups.c
C_SRC += $(FREERTOS_DIR)/list.c
C_SRC += $(FREERTOS_DIR)/portable/GCC/ARM_CM4F/port.c
C_SRC += $(FREERTOS_DIR)/portable/MemMang/heap_4.c
C_SRC += $(FREERTOS_DIR)/queue.c
C_SRC += $(FREERTOS_DIR)/stream_buffer.c
C_SRC += $(FREERTOS_DIR)/tasks.c
C_SRC += $(FREERTOS_DIR)/timers.c
C_SRC += $(STM32F4_HAL_DIR)/Src/stm32f4xx_hal.c
C_SRC += $(STM32F4_HAL_DIR)/Src/stm32f4xx_hal_cortex.c
C_SRC += $(STM32F4_HAL_DIR)/Src/stm32f4xx_hal_dma.c
C_SRC += $(STM32F4_HAL_DIR)/Src/stm32f4xx_hal_gpio.c
C_SRC += $(STM32F4_HAL_DIR)/Src/stm32f4xx_hal_uart.c
C_SRC += $(STM32F4_HAL_DIR)/Src/stm32f4xx_hal_rcc.c
C_SRC += $(STM32F4_HAL_DIR)/Src/stm32f4xx_ll_usart.c

# Object files - generated from SRC lists
ASM_OBJ = $(addprefix $(OBJ_DIR)/,$(notdir $(ASM_SRC:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SRC)))
C_OBJ = $(addprefix $(OBJ_DIR)/,$(notdir $(C_SRC:.c=.o)))
vpath %.c $(sort $(dir $(C_SRC)))

# Compilation flags
CC_FLAGS  = -Wall -O0 -g
CC_FLAGS += -mcpu=$(CPU) -mthumb -mfloat-abi=hard -mfpu=$(FPU)
CC_FLAGS +=  --specs=nano.specs

# Preprocessor Defines
# STM32F4 Discovery board has MCU STM32F407VG
C_DEFS  = -DSTM32F407xx
# STM32F4 Discovery board has 8 MHz OSC for HSE
C_DEFS += -DHSE_VALUE=8000000

# Include folders
INC_FLAGS  = -I $(INC_DIR)
INC_FLAGS += -I $(SUBMODULES_DIR)/ringbuf/inc
INC_FLAGS += -I $(STM32F4_HAL_DIR)/Inc
INC_FLAGS += -I $(STM32F4_HAL_DIR)/Inc/Legacy
INC_FLAGS += -I $(FREERTOS_DIR)/include
INC_FLAGS += -I $(FREERTOS_DIR)/portable/GCC/ARM_CM4F
INC_FLAGS += -I $(CMSIS_DIR)/Include
INC_FLAGS += -I $(CMSIS_DIR)/Device/ST/STM32F4xx/Include
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