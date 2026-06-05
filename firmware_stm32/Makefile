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
CC  = $(PREFIX)gcc
CXX = $(PREFIX)g++
AS  = $(PREFIX)gcc -x assembler-with-cpp
CP  = $(PREFIX)objcopy
SZ  = $(PREFIX)size

HAL     = vendor/stm32f4xx_hal_driver
RTOS    = vendor/FreeRTOS-Kernel
DEV     = vendor/cmsis_device_f4
CORE    = vendor/cmsis_core/CMSIS/Core/Include
UROS    = micro_ros_stm32cubemx_utils/microros_static_library/libmicroros
PORTLIB = ../firmware/lib

C_DEFS = -DUSE_HAL_DRIVER -DSTM32F446xx -DUSE_F446RE_CONFIG
C_INCLUDES = -IInc -Iconfig -I$(DEV)/Include -I$(CORE) -I$(HAL)/Inc \
             -I$(RTOS)/include -I$(RTOS)/portable/GCC/ARM_CM4F \
             -I$(UROS)/microros_include \
             -I$(PORTLIB)/kinematics -I$(PORTLIB)/pid \
             -I$(PORTLIB)/odom_integrator -I$(PORTLIB)/motor

C_SOURCES = \
  Src/main.c Src/microros_glue.c Src/microros_atomic64.c Src/sys_freertos.c \
  $(DEV)/Source/Templates/system_stm32f4xx.c \
  $(HAL)/Src/stm32f4xx_hal.c $(HAL)/Src/stm32f4xx_hal_rcc.c $(HAL)/Src/stm32f4xx_hal_rcc_ex.c \
  $(HAL)/Src/stm32f4xx_hal_gpio.c $(HAL)/Src/stm32f4xx_hal_cortex.c \
  $(HAL)/Src/stm32f4xx_hal_pwr.c $(HAL)/Src/stm32f4xx_hal_pwr_ex.c \
  $(HAL)/Src/stm32f4xx_hal_flash.c $(HAL)/Src/stm32f4xx_hal_flash_ex.c \
  $(HAL)/Src/stm32f4xx_hal_dma.c $(HAL)/Src/stm32f4xx_hal_uart.c \
  $(HAL)/Src/stm32f4xx_hal_tim.c $(HAL)/Src/stm32f4xx_hal_tim_ex.c \
  $(RTOS)/tasks.c $(RTOS)/queue.c $(RTOS)/list.c \
  $(RTOS)/portable/GCC/ARM_CM4F/port.c $(RTOS)/portable/MemMang/heap_4.c

# C++ translation units: the native drivers + control loop + reused portable math.
CXX_SOURCES = \
  Src/control_loop.cpp Src/stm32_encoder.cpp Src/stm32_motor.cpp \
  Src/lino_hal.cpp Src/cxx_runtime.cpp \
  $(PORTLIB)/kinematics/kinematics.cpp $(PORTLIB)/pid/pid.cpp

ASM_SOURCES = $(DEV)/Source/Templates/gcc/startup_stm32f446xx.s

CFLAGS   = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections -g -gdwarf-2
# C++: same MCU/ABI flags (hard-VFP must match libmicroros), no exceptions/RTTI/atexit
# so libstdc++ is never pulled in (operator new/__cxa_pure_virtual come from cxx_runtime.cpp).
CXXFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections -g -gdwarf-2 \
           -std=gnu++17 -fno-exceptions -fno-rtti -fno-use-cxa-atexit -fno-threadsafe-statics

LDSCRIPT = STM32F446RETX_FLASH.ld
# --- micro-ROS link --- (link with gcc; g++ is not the linker)
LIBMICROROS = $(UROS)/libmicroros.a
LDFLAGS = $(MCU) -specs=nano.specs -specs=nosys.specs -T$(LDSCRIPT) $(LIBMICROROS) \
          -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections -lc -lm

OBJECTS  = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(CXX_SOURCES:.cpp=.o)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.c   $(sort $(dir $(C_SOURCES)))
vpath %.cpp $(sort $(dir $(CXX_SOURCES)))
vpath %.s   $(sort $(dir $(ASM_SOURCES)))

all: $(BUILD_DIR)/$(TARGET).elf

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@
$(BUILD_DIR)/%.o: %.cpp Makefile | $(BUILD_DIR)
	$(CXX) -c $(CXXFLAGS) $< -o $@
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
