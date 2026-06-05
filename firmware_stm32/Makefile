# GUI-free hand-written Makefile for the F446RE micro-ROS firmware (no CubeMX).
# Build:        make            -> build/firmware_stm32.elf (+ .hex/.bin)
# print_cflags: used by the micro-ROS Docker static-library builder to ABI-match libmicroros.a.
TARGET    = firmware_stm32
BUILD_DIR = build
OPT       = -Os

CPU       = -mcpu=cortex-m4
FPU       = -mfpu=fpv4-sp-d16
FLOAT-ABI = -mfloat-abi=hard
MCU       = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size

HAL  = vendor/stm32f4xx_hal_driver
RTOS = vendor/FreeRTOS-Kernel
DEV  = vendor/cmsis_device_f4
CORE = vendor/cmsis_core/CMSIS/Core/Include
UROS = micro_ros_stm32cubemx_utils/microros_static_library/libmicroros

C_DEFS = -DUSE_HAL_DRIVER -DSTM32F446xx
C_INCLUDES = -IInc -I$(DEV)/Include -I$(CORE) -I$(HAL)/Inc \
             -I$(RTOS)/include -I$(RTOS)/portable/GCC/ARM_CM4F \
             -I$(UROS)/microros_include

C_SOURCES = \
  Src/main.c Src/microros_glue.c Src/microros_atomic64.c \
  $(DEV)/Source/Templates/system_stm32f4xx.c \
  $(HAL)/Src/stm32f4xx_hal.c $(HAL)/Src/stm32f4xx_hal_rcc.c $(HAL)/Src/stm32f4xx_hal_rcc_ex.c \
  $(HAL)/Src/stm32f4xx_hal_gpio.c $(HAL)/Src/stm32f4xx_hal_cortex.c \
  $(HAL)/Src/stm32f4xx_hal_pwr.c $(HAL)/Src/stm32f4xx_hal_pwr_ex.c \
  $(HAL)/Src/stm32f4xx_hal_flash.c $(HAL)/Src/stm32f4xx_hal_flash_ex.c \
  $(HAL)/Src/stm32f4xx_hal_dma.c $(HAL)/Src/stm32f4xx_hal_uart.c \
  $(RTOS)/tasks.c $(RTOS)/queue.c $(RTOS)/list.c \
  $(RTOS)/portable/GCC/ARM_CM4F/port.c $(RTOS)/portable/MemMang/heap_4.c

ASM_SOURCES = $(DEV)/Source/Templates/gcc/startup_stm32f446xx.s

CFLAGS  = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections -g -gdwarf-2
LDSCRIPT = STM32F446RETX_FLASH.ld
# --- micro-ROS link ---
LIBMICROROS = $(UROS)/libmicroros.a
LDFLAGS = $(MCU) -specs=nano.specs -specs=nosys.specs -T$(LDSCRIPT) $(LIBMICROROS) \
          -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections -lc -lm

OBJECTS  = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

all: $(BUILD_DIR)/$(TARGET).elf

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@
$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@
$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(CP) -O ihex $@ $(BUILD_DIR)/$(TARGET).hex
	$(CP) -O binary -S $@ $(BUILD_DIR)/$(TARGET).bin
	$(SZ) $@
$(BUILD_DIR):
	mkdir $@
clean:
	rm -rf $(BUILD_DIR)
print_cflags:
	@echo $(CFLAGS)
.PHONY: all clean print_cflags
