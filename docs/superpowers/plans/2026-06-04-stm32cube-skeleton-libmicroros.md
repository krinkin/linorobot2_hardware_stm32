# STM32 Native Port — Plan 2: F446RE Skeleton + libmicroros ABI-Smoke (Phase F0)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
>
> **GUI-FREE BY DESIGN.** This plan uses **no STM32CubeMX, no GUI, no `.ioc`** — a firmware port needs firmware *code*, not a GUI. The whole project is hand-assembled from open-source sources (CMSIS device pack BSD-3, CMSIS-Core Apache-2.0, `stm32f4xx_hal_driver` BSD-3, `FreeRTOS-Kernel` MIT) with a hand-written `Makefile`, linker script, and config headers. This is fully **reproducible and CI-able with no GUI tool and no myST account** — which is exactly what "universal across any STM32" needs. It runs anywhere with `arm-none-eabi-gcc` + Docker (+ Renode for Plan 3). **Empirically validated (2026-06-05):** a hand-written FreeRTOS firmware (CMSIS startup + hand linker script + `FreeRTOS-Kernel` + hand-written `FreeRTOSConfig.h`) boots and schedules in Renode (task ticked 12 449× / 3 s), and the official `libmicroros.a` builds from a **hand-written** `Makefile` (`make print_cflags` is all the Docker builder needs).

**Goal:** Stand up a real-but-minimal NUCLEO-F446RE firmware (hand-written HAL + FreeRTOS) that **links `libmicroros.a` cleanly** — the ABI/atomics smoke test that de-risks the entire native route (spec `docs/STM32CUBE_PORTING_PLAN.md` §8 Ф0, §9 risks #1–#2).

**Architecture:** Native route (spec §2 Route B): a **hand-assembled** HAL/FreeRTOS project built by a hand-written `Makefile`, the official **`micro_ros_stm32cubemx_utils`** (pinned `jazzy` commit; despite its name it needs **no GUI** — only its `extra_sources/*.c` + the Docker static-library builder) vendored as a submodule, and **`libmicroros.a`** built once by the Docker image `microros/micro_ros_static_library_builder:jazzy`. ABI match is guaranteed *by flag forwarding* — the Docker builder reads the app's CFLAGS via a `print_cflags` Makefile target and compiles the library with the same `-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard`; `colcon.meta` sets `RCUTILS_NO_64_ATOMIC=ON` (covers `rcutils` only; `rcl` still needs an app-provided `__atomic_*_8` shim — Task 5a). There is **no application logic** in F0 — only enough to force the linker to pull in micro-ROS symbols.

**Tech Stack:** GNU Arm Embedded toolchain (`arm-none-eabi-gcc`), GNU Make, Docker, FreeRTOS-Kernel (MIT), STM32 HAL (`stm32f4xx_hal_driver` BSD-3) + CMSIS (`cmsis_device_f4` BSD-3, `cmsis_core` Apache-2.0), micro-ROS (`rcl`/`rclc`/`rmw_microxrcedds`) via `micro_ros_stm32cubemx_utils@a5b2127`. Target: STM32F446RETx (Cortex-M4F). **No STM32CubeMX.**

**Pinned references (verified live against `micro-ROS/micro_ros_stm32cubemx_utils@jazzy`):**
- utils commit: `a5b2127495ae0ab53d7a1360beaf17822309a3cc`
- builder image (multi-arch index digest): `microros/micro_ros_static_library_builder@sha256:1482f3df56184ecc5d4a9d45ad9be0a17a84a91fca947d07f20d1678b23f6243` (amd64 sub-digest `392246e5…`; matches GitHub `ubuntu-latest` runners)

---

## Where this plan sits

Plan 2 of 7 for the native STM32 port (see `docs/STM32CUBE_PORTING_PLAN.md`). Plan 1 (host test tier + odometry refactor) is done and merged (`stm32cube-p1-host-test-tier`). This plan delivers the **Ф0 skeleton**. It does NOT: boot-verify (Plan 3, Renode), wire a working transport/agent round-trip (Plan 4), or implement encoder/PWM/IMU (Plans 5–6).

**F0 is intentionally a "does it LINK" gate, not a "does it RUN" gate.** A clean link proves the float-ABI and 64-bit-atomics issues — the two things that killed the unsupported PlatformIO route (spec §2 Route A) — are resolved. Booting is a separate, runtime concern owned by Plan 3.

**Empirically validated (2026-06-04/05).** The real `libmicroros.a` was built via the official jazzy Docker flow (from a hand-written Makefile) and inspected with the Cortex-M4F toolchain: **float-ABI = PASS** (all 2014 members hard-float `Tag_ABI_VFP_args: VFP registers` — the Route-A VFP wall does NOT occur with this flow), and **64-bit atomics = needs a shim** (`rcl` references `__atomic_*_8` that `arm-none-eabi` cannot satisfy on M4). A PRIMASK critical-section shim (Task 5a) was proven to make the realistic micro-ROS symbol set link cleanly. F0 closed end-to-end on a **real F446 ELF**: hand linker + CMSIS startup + shim + the official `libmicroros.a` → clean link, 73.7 KB Flash / 22.6 KB RAM, hard-float; control proof: the identical link **without** the shim fails on `__atomic_compare_exchange_8`. So F0 is **two** gates — VFP **and** atomics/POSIX — reflected in Task 5a and the Task 6 FAIL signatures. None of this used a GUI.

**Update (2026-06-05): the FULL combined GUI-free firmware** (CMSIS startup + hand linker + HAL `HAL_Init`/RCC/GPIO/USART2 + FreeRTOS + micro-ROS `rclc_support_init`/node + HAL-UART transport + atomic shim + `clock_gettime`/`usleep` glue) was assembled by hand and **links — 49 KB Flash / 73.5 KB RAM, hard-float** — and **boots in Renode** (FreeRTOS scheduler runs, `rclc_support_init` executes, no HardFault; see Plan 3). The native port is thus proven GUI-free **end-to-end** (F0 link + F2 boot) on this machine — zero CubeMX.

---

## File Structure

The project lives in a new top-level `firmware_stm32/` (alongside `firmware/`). Vendored sources come from pinned submodules; everything else is hand-written (no generator).

| Path | Responsibility | Origin |
|---|---|---|
| `firmware_stm32/PINS.md` | Records the exact utils commit + image digest + vendored-source pins | Author (Task 1) |
| `firmware_stm32/vendor/cmsis_device_f4/` | F446 startup + `system_stm32f4xx.c` + device headers | submodule (Task 2) |
| `firmware_stm32/vendor/cmsis_core/` | CMSIS-Core headers (`core_cm4.h`, `cmsis_compiler.h`) | submodule (Task 2) |
| `firmware_stm32/vendor/stm32f4xx_hal_driver/` | STM32 HAL drivers | submodule (Task 2) |
| `firmware_stm32/vendor/FreeRTOS-Kernel/` | FreeRTOS kernel + ARM_CM4F port | submodule (Task 2) |
| `firmware_stm32/Makefile` | Hand-written build (+ micro-ROS addon block) | Author (Task 2, Task 4) |
| `firmware_stm32/STM32F446RETX_FLASH.ld` | Linker script (512K flash @0x08000000 / 128K RAM @0x20000000) | Author (Task 2) |
| `firmware_stm32/Inc/FreeRTOSConfig.h` | FreeRTOS config (CM4F) | Author (Task 2) |
| `firmware_stm32/Inc/stm32f4xx_hal_conf.h` | HAL module enables + config | Author (Task 2) |
| `firmware_stm32/Src/main.c` | `main()` + clock/UART init + the micro-ROS FreeRTOS task | Author (Task 2, Task 5) |
| `firmware_stm32/Src/stm32f4xx_it.c` | IRQ handlers (USART2_IRQHandler; SysTick→HAL tick) | Author (Task 2) |
| `firmware_stm32/Src/syscalls.c` | newlib syscall stubs (`_sbrk`/`_write`/…) | Author (Task 2) |
| `firmware_stm32/Src/microros_atomic64.c` | 64-bit atomic shim — Cortex-M4 (**REQUIRED**) | Author (Task 5a) |
| `firmware_stm32/Src/microros_posix_stubs.c` | `usleep`→`osDelay` (**REQUIRED**) | Author (Task 5a) |
| `firmware_stm32/micro_ros_stm32cubemx_utils/` | Vendored utils (submodule @ pinned commit) | Task 3 |
| `firmware_stm32/.gitignore` | Ignore `build/` + `libmicroros/`, **keep** all hand-written + vendored source | Author (Task 7) |
| `.github/workflows/stm32-f446re-f0.yml` | Dedicated F0 link-smoke CI (separate from `parse_platformio.py`) | Author (Task 7) |

---

### Task 1: Scaffold `firmware_stm32/` and record pinned references

**Files:**
- Create: `firmware_stm32/PINS.md`

- [ ] **Step 1: Confirm the host toolchain is present**
```bash
arm-none-eabi-gcc --version | head -n1
make --version | head -n1
docker --version
```
Expected: each prints a version. If missing (Debian/Ubuntu): `sudo apt-get install -y gcc-arm-none-eabi binutils-arm-none-eabi make docker.io git`. No GUI tool is needed. If `arm-none-eabi-gcc`/`docker` are unavailable, STOP and report BLOCKED.

- [ ] **Step 2: Record the pins** — create `firmware_stm32/PINS.md`:
```markdown
# Pinned references for the native STM32 micro-ROS skeleton (GUI-free)

`jazzy` is a MOVING branch; always check out the commit, not just `-b jazzy`.

- micro_ros_stm32cubemx_utils commit: a5b2127495ae0ab53d7a1360beaf17822309a3cc  (jazzy HEAD as of 2026-06-04)
- builder image (index digest): microros/micro_ros_static_library_builder@sha256:1482f3df56184ecc5d4a9d45ad9be0a17a84a91fca947d07f20d1678b23f6243
  - amd64 sub-digest: sha256:392246e55107aeda92db5b191fce2a7d330428d978e2936a03096c78c8d3ae30 (GitHub ubuntu runners)
- vendored sources (pin each submodule to a release tag/commit): cmsis_device_f4 (BSD-3),
  cmsis_core (Apache-2.0), stm32f4xx_hal_driver (BSD-3), FreeRTOS-Kernel (MIT).

Note: the libmicroros build is NOT bit-reproducible even with these pins — inside the container
the builder clones geometry2@jazzy and vcs-imports extra_packages (moving HEADs) and apt-installs an
unpinned gcc-arm-none-eabi. For strict reproducibility, vendor the produced libmicroros.a (see Task 7).
```

- [ ] **Step 3: Commit**
```bash
git add firmware_stm32/PINS.md
git commit -m "build(stm32): scaffold firmware_stm32 and pin build refs (GUI-free)

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 2: Hand-assemble the F446RE HAL/FreeRTOS project (GUI-free)

Build the project from open-source sources with a hand-written `Makefile` + linker + config headers — **no CubeMX**. Validated building blocks exist already: a hand linker + CMSIS startup links micro-ROS (Option 1, 73.7 KB), and hand-written `FreeRTOS-Kernel` schedules in Renode.

**Files:** see the File Structure table (vendored submodules + hand-written `Makefile`, `.ld`, `Inc/*.h`, `Src/*.c`).

- [ ] **Step 1: Vendor the open-source sources as pinned submodules**
```bash
cd firmware_stm32 && mkdir -p vendor Inc Src
git submodule add https://github.com/STMicroelectronics/cmsis_device_f4    vendor/cmsis_device_f4
git submodule add https://github.com/STMicroelectronics/cmsis_core         vendor/cmsis_core
git submodule add https://github.com/STMicroelectronics/stm32f4xx_hal_driver vendor/stm32f4xx_hal_driver
git submodule add https://github.com/FreeRTOS/FreeRTOS-Kernel              vendor/FreeRTOS-Kernel
# pin each to a known tag/commit (edit to the latest stable you validate), e.g.:
git -C vendor/FreeRTOS-Kernel checkout V11.1.0
```
Verify: `test -f vendor/cmsis_device_f4/Source/Templates/gcc/startup_stm32f446xx.s && test -f vendor/FreeRTOS-Kernel/portable/GCC/ARM_CM4F/port.c && echo OK`.

- [ ] **Step 2: Write the linker script** `firmware_stm32/STM32F446RETX_FLASH.ld` (standard F446RE; validated in Option 1):
```
ENTRY(Reset_Handler)
_estack = ORIGIN(RAM) + LENGTH(RAM);
_Min_Heap_Size = 0x200;
_Min_Stack_Size = 0x800;
MEMORY { RAM (xrw): ORIGIN = 0x20000000, LENGTH = 128K
         FLASH (rx): ORIGIN = 0x08000000, LENGTH = 512K }
SECTIONS {
  .isr_vector : { . = ALIGN(4); KEEP(*(.isr_vector)) . = ALIGN(4); } >FLASH
  .text : { . = ALIGN(4); *(.text) *(.text*) *(.glue_7) *(.glue_7t) *(.eh_frame)
            KEEP(*(.init)) KEEP(*(.fini)) . = ALIGN(4); _etext = .; } >FLASH
  .rodata : { . = ALIGN(4); *(.rodata) *(.rodata*) . = ALIGN(4); } >FLASH
  .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
  .ARM : { __exidx_start = .; *(.ARM.exidx*) __exidx_end = .; } >FLASH
  .preinit_array : { PROVIDE_HIDDEN(__preinit_array_start = .); KEEP(*(.preinit_array*)) PROVIDE_HIDDEN(__preinit_array_end = .); } >FLASH
  .init_array : { PROVIDE_HIDDEN(__init_array_start = .); KEEP(*(SORT(.init_array.*))) KEEP(*(.init_array*)) PROVIDE_HIDDEN(__init_array_end = .); } >FLASH
  .fini_array : { PROVIDE_HIDDEN(__fini_array_start = .); KEEP(*(SORT(.fini_array.*))) KEEP(*(.fini_array*)) PROVIDE_HIDDEN(__fini_array_end = .); } >FLASH
  _sidata = LOADADDR(.data);
  .data : { . = ALIGN(4); _sdata = .; *(.data) *(.data*) . = ALIGN(4); _edata = .; } >RAM AT> FLASH
  .bss : { . = ALIGN(4); _sbss = .; __bss_start__ = _sbss; *(.bss) *(.bss*) *(COMMON) . = ALIGN(4); _ebss = .; __bss_end__ = _ebss; } >RAM
  ._user_heap_stack : { . = ALIGN(8); PROVIDE(end = .); PROVIDE(_end = .); . = . + _Min_Heap_Size; . = . + _Min_Stack_Size; . = ALIGN(8); } >RAM
  .ARM.attributes 0 : { *(.ARM.attributes) }
}
```

- [ ] **Step 3: Write `firmware_stm32/Inc/FreeRTOSConfig.h`** (CM4F; validated in Renode this session). Key: `configCPU_CLOCK_HZ` matches the clock you set in Step 6; map the kernel handlers; `configPRIO_BITS 4`. Set `configTOTAL_HEAP_SIZE` ≥ the micro-ROS need; the micro-ROS task stack (≥24 KB) is set at `xTaskCreate` time (Task 5).
```c
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H
#define configUSE_PREEMPTION 1
#define configSUPPORT_DYNAMIC_ALLOCATION 1
#define configCPU_CLOCK_HZ 180000000
#define configTICK_RATE_HZ 1000
#define configMAX_PRIORITIES 7
#define configMINIMAL_STACK_SIZE 128
#define configTOTAL_HEAP_SIZE (40*1024)
#define configMAX_TASK_NAME_LEN 16
#define configUSE_16_BIT_TICKS 0
#define configUSE_MUTEXES 1
#define configUSE_TIMERS 0
#define configCHECK_FOR_STACK_OVERFLOW 0
#define configUSE_MALLOC_FAILED_HOOK 0
#define configPRIO_BITS 4
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY 15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5
#define configKERNEL_INTERRUPT_PRIORITY (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define INCLUDE_vTaskDelay 1
#define INCLUDE_xTaskGetSchedulerState 1
#define vPortSVCHandler SVC_Handler
#define xPortPendSVHandler PendSV_Handler
#define xPortSysTickHandler SysTick_Handler
#define configASSERT( x ) if((x)==0){ taskDISABLE_INTERRUPTS(); for(;;); }
#endif
```

- [ ] **Step 4: Write `firmware_stm32/Inc/stm32f4xx_hal_conf.h`** — copy `vendor/stm32f4xx_hal_driver/Inc/stm32f4xx_hal_conf_template.h` to `Inc/stm32f4xx_hal_conf.h` and **enable only** the modules used: keep `HAL_MODULE_ENABLED`, `HAL_RCC_MODULE_ENABLED`, `HAL_GPIO_MODULE_ENABLED`, `HAL_CORTEX_MODULE_ENABLED`, `HAL_PWR_MODULE_ENABLED`, `HAL_FLASH_MODULE_ENABLED`, `HAL_DMA_MODULE_ENABLED`, `HAL_UART_MODULE_ENABLED`; comment out the rest. Set `HSE_VALUE 8000000U` (Nucleo MCO bypass).
```bash
cp vendor/stm32f4xx_hal_driver/Inc/stm32f4xx_hal_conf_template.h Inc/stm32f4xx_hal_conf.h
# then edit the #define ..._MODULE_ENABLED list as above
```

- [ ] **Step 5: Write `firmware_stm32/Src/syscalls.c`** — standard newlib stubs so `-specs=nano.specs` links cleanly (`_sbrk` using `end`/`_estack`, plus no-op `_write/_read/_close/_lseek/_fstat/_isatty/_kill/_getpid/_exit`). (Use the well-known ARM newlib `syscalls.c` template; `_sbrk` grows the heap from `end` toward the stack.)

- [ ] **Step 6: Write `firmware_stm32/Src/main.c` platform glue** — `HAL_Init()`, `SystemClock_Config()` (PLL → 180 MHz from HSE-bypass 8 MHz, or stay on HSI for a simpler first bring-up), `MX_GPIO_Init()` + `MX_USART2_UART_Init()` (USART2, **PA2/PA3**, 115200 8N1; `huart2` global), create the FreeRTOS micro-ROS task (Task 5), `vTaskStartScheduler()`. **HAL timebase via DWT** (override `HAL_InitTick`/`HAL_GetTick` to use the DWT cycle counter) so FreeRTOS owns SysTick with no conflict:
```c
HAL_StatusTypeDef HAL_InitTick(uint32_t prio){ (void)prio;
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; DWT->CYCCNT = 0; DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; return HAL_OK; }
uint32_t HAL_GetTick(void){ return DWT->CYCCNT / (SystemCoreClock/1000U); }
```

- [ ] **Step 7: Write `firmware_stm32/Src/stm32f4xx_it.c`** — the IRQ handlers the app needs: `USART2_IRQHandler(){ HAL_UART_IRQHandler(&huart2); }`, the fault handlers (`HardFault_Handler` etc. — simple `for(;;)`), and the core exceptions. (`SVC/PendSV/SysTick` are owned by FreeRTOS via the `FreeRTOSConfig.h` `#define`s — do NOT also define them here.)

- [ ] **Step 8: Write the hand-written `firmware_stm32/Makefile`** — `TARGET=firmware_stm32`, the F4 flags, all sources, and the micro-ROS addon (Task 4). Base skeleton (the micro-ROS addon block is added in Task 4):
```makefile
TARGET = firmware_stm32
BUILD_DIR = build
DEBUG = 1
OPT = -Os
CPU = -mcpu=cortex-m4
FPU = -mfpu=fpv4-sp-d16
FLOAT-ABI = -mfloat-abi=hard
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)
PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
HAL = vendor/stm32f4xx_hal_driver
RTOS = vendor/FreeRTOS-Kernel
DEV = vendor/cmsis_device_f4
CORE = vendor/cmsis_core/CMSIS/Core/Include
C_DEFS = -DUSE_HAL_DRIVER -DSTM32F446xx
C_INCLUDES = -IInc -I$(DEV)/Include -I$(CORE) -I$(HAL)/Inc \
  -I$(RTOS)/include -I$(RTOS)/portable/GCC/ARM_CM4F
C_SOURCES = \
  Src/main.c Src/stm32f4xx_it.c Src/syscalls.c \
  $(DEV)/Source/Templates/system_stm32f4xx.c \
  $(HAL)/Src/stm32f4xx_hal.c $(HAL)/Src/stm32f4xx_hal_rcc.c $(HAL)/Src/stm32f4xx_hal_rcc_ex.c \
  $(HAL)/Src/stm32f4xx_hal_gpio.c $(HAL)/Src/stm32f4xx_hal_cortex.c $(HAL)/Src/stm32f4xx_hal_pwr.c \
  $(HAL)/Src/stm32f4xx_hal_pwr_ex.c $(HAL)/Src/stm32f4xx_hal_flash.c $(HAL)/Src/stm32f4xx_hal_flash_ex.c \
  $(HAL)/Src/stm32f4xx_hal_dma.c $(HAL)/Src/stm32f4xx_hal_uart.c \
  $(RTOS)/tasks.c $(RTOS)/queue.c $(RTOS)/list.c \
  $(RTOS)/portable/GCC/ARM_CM4F/port.c $(RTOS)/portable/MemMang/heap_4.c
ASM_SOURCES = $(DEV)/Source/Templates/gcc/startup_stm32f446xx.s
CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections -g -gdwarf-2
LDSCRIPT = STM32F446RETX_FLASH.ld
LIBS = -lc -lm -lnosys
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o))) $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))
$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@
$(BUILD_DIR)/%.o: %.s | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@
$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(CP) -O ihex $@ $(BUILD_DIR)/$(TARGET).hex
	$(CP) -O binary -S $@ $(BUILD_DIR)/$(TARGET).bin
	$(SZ) $@
$(BUILD_DIR):
	mkdir $@
all: $(BUILD_DIR)/$(TARGET).elf
clean:
	rm -rf $(BUILD_DIR)
.PHONY: all clean
```
> `-specs=nano.specs` (newlib-nano) + your hand `Src/syscalls.c` for `_sbrk` etc. The hard-float triple is explicit here (no `grep`-the-generated-file needed — you wrote it).

- [ ] **Step 9: Sanity-check the base project compiles** (before micro-ROS): `cd firmware_stm32 && make` should build a tiny `build/firmware_stm32.elf` (HAL + FreeRTOS only). (You can boot it in Renode now — that is Plan 3.)

- [ ] **Step 10: Commit**
```bash
git add firmware_stm32/.gitmodules firmware_stm32/vendor firmware_stm32/Makefile \
  firmware_stm32/STM32F446RETX_FLASH.ld firmware_stm32/Inc firmware_stm32/Src
git commit -m "build(stm32): hand-assembled F446RE HAL/FreeRTOS project (GUI-free, no CubeMX)

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 3: Vendor `micro_ros_stm32cubemx_utils` at the pinned commit

The library is named `*_stm32cubemx_utils` but needs **no GUI** — we use only its `extra_sources/*.c` (transport/time/allocators) and its Docker static-library builder.

**Files:** `firmware_stm32/micro_ros_stm32cubemx_utils/` (submodule) + `.gitmodules`

- [ ] **Step 1: Add + pin**
```bash
cd firmware_stm32
git submodule add https://github.com/micro-ROS/micro_ros_stm32cubemx_utils.git micro_ros_stm32cubemx_utils
git -C micro_ros_stm32cubemx_utils checkout a5b2127495ae0ab53d7a1360beaf17822309a3cc
```
- [ ] **Step 2: Verify**
```bash
git -C micro_ros_stm32cubemx_utils rev-parse HEAD   # a5b2127...
test -f micro_ros_stm32cubemx_utils/extra_sources/microros_transports/it_transport.c
test -f micro_ros_stm32cubemx_utils/microros_static_library/library_generation/colcon.meta
test -f micro_ros_stm32cubemx_utils/extra_sources/microros_time.c && echo OK
```
- [ ] **Step 3: Commit**
```bash
cd .. && git add .gitmodules firmware_stm32/micro_ros_stm32cubemx_utils
git commit -m "build(stm32): vendor micro_ros_stm32cubemx_utils @ a5b2127 (jazzy)

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 4: Add the micro-ROS addon block to the hand-written Makefile

**Files:** Modify `firmware_stm32/Makefile`

- [ ] **Step 1: Append the addon block** (just before the build rules). Note `it_transport.c`/`microros_time.c` include `cmsis_os.h` (CMSIS-OS v2) — provide it from FreeRTOS's CMSIS-RTOS-v2 wrapper, OR (GUI-free-simplest) write a thin `cmsis_os.h`/`cmsis_os2.c` shim mapping `osDelay`→`vTaskDelay` and the few symbols used; add that shim to `C_SOURCES`/`C_INCLUDES`.
```makefile
# --- micro-ROS addons ---
LDFLAGS += micro_ros_stm32cubemx_utils/microros_static_library/libmicroros/libmicroros.a
C_INCLUDES += -Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include
C_SOURCES += micro_ros_stm32cubemx_utils/extra_sources/custom_memory_manager.c
C_SOURCES += micro_ros_stm32cubemx_utils/extra_sources/microros_allocators.c
C_SOURCES += micro_ros_stm32cubemx_utils/extra_sources/microros_time.c
C_SOURCES += micro_ros_stm32cubemx_utils/extra_sources/microros_transports/it_transport.c
# REQUIRED on Cortex-M4 (Task 5a): 64-bit-atomics shim + usleep (rcl uses __atomic_*_8
# unconditionally; arm-none-eabi has no baremetal 64-bit atomics; NO_64_ATOMIC covers rcutils only).
C_SOURCES += Src/microros_atomic64.c
C_SOURCES += Src/microros_posix_stubs.c

print_cflags:
	@echo $(CFLAGS)
```
- [ ] **Step 2: TAB check** — the `@echo $(CFLAGS)` recipe line must start with a literal TAB: `grep -nP '^\t@echo \$\(CFLAGS\)' Makefile` → one match. (Space indent → `*** missing separator`.)
- [ ] **Step 3: Confirm hard-float flows out** — `make print_cflags | grep -o -- '-mfloat-abi=hard'` → prints `-mfloat-abi=hard` (the string the Docker builder forwards into `libmicroros.a`).
- [ ] **Step 4: Commit**
```bash
git add firmware_stm32/Makefile
git commit -m "build(stm32): micro-ROS addon block in the hand-written Makefile (print_cflags, link libmicroros)

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 5: Add the minimal micro-ROS task (force-link symbols)

The link only exercises `libmicroros.a` if the app references its symbols. Add a FreeRTOS task that inits the custom-allocator + transport + `rclc_support` — same symbol set as the upstream `sample_main.c`, with `huart2`.

**Files:** Modify `firmware_stm32/Src/main.c`

- [ ] **Step 1: Includes** (top of `main.c`):
```c
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
```
- [ ] **Step 2: Prototypes** (file scope; `cubemx_transport_*` are the upstream `it_transport.c` API names — keep them):
```c
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);
void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);
```
- [ ] **Step 3: The task body** — create the task with a **≥24 KB stack** (`xTaskCreate(microros_task, "uros", 6144, NULL, 24, NULL)` → 6144 words = 24 KB), and in its body:
```c
extern UART_HandleTypeDef huart2;
rmw_uros_set_custom_transport(true, (void*)&huart2,
    cubemx_transport_open, cubemx_transport_close, cubemx_transport_write, cubemx_transport_read);

rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
freeRTOS_allocator.allocate = microros_allocate;
freeRTOS_allocator.deallocate = microros_deallocate;
freeRTOS_allocator.reallocate = microros_reallocate;
freeRTOS_allocator.zero_allocate = microros_zero_allocate;
if (!rcutils_set_default_allocator(&freeRTOS_allocator)) { for(;;){} }

rclc_support_t support;
rcl_allocator_t allocator = rcl_get_default_allocator();
rcl_node_t node;
rclc_support_init(&support, 0, NULL, &allocator);
rclc_node_init_default(&node, "stm32_node", "", &support);
for (;;) { vTaskDelay(pdMS_TO_TICKS(1000)); }
```
- [ ] **Step 4: Commit**
```bash
git add firmware_stm32/Src/main.c
git commit -m "feat(stm32): minimal micro-ROS task to force-link libmicroros (F0)

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 5a: 64-bit atomic shim + `usleep` stub — REQUIRED for the F0 link

**Why (empirically validated 2026-06-04, corroborated by micro_ros_stm32cubemx_utils#112).** The real `libmicroros.a` references `__atomic_{load,store,exchange,compare_exchange}_8` from `librcl-time.c.obj` / `librcl-timer.c.obj` / `librcl-client.c.obj` (+ `librcutils-fault_injection.c.obj`), and `arm-none-eabi` 13.2.1 provides **no** 64-bit atomics for the Cortex-M4 hard multilib (no `LDREXD`/`STREXD`; no baremetal `libatomic`). `RCUTILS_NO_64_ATOMIC=ON` neutralizes only `rcutils`' `atomic_64bits.c`; `rcl`'s `time.c`/`timer.c`/`client.c` use 64-bit atomics **unconditionally**. The maintainer's stated fix: *"provide an implementation of `__atomic_load_8`."* Separately, `rclc_sleep_ms` needs `usleep` (NOT shipped by the utils; `clock_gettime` IS shipped by `microros_time.c` — do not re-stub it). A PRIMASK critical-section shim was proven to link the realistic micro-ROS symbol set cleanly.

**Files:**
- Create: `firmware_stm32/Src/microros_atomic64.c`
- Create: `firmware_stm32/Src/microros_posix_stubs.c`

- [ ] **Step 1: Create the 64-bit atomic shim** `firmware_stm32/Src/microros_atomic64.c`:
```c
/* 64-bit atomics shim for single-core Cortex-M (STM32F446, M4F).
 * Resolves __atomic_*_8 referenced by libmicroros.a (rcl time/timer/client,
 * rcutils fault_injection) — arm-none-eabi has no baremetal 64-bit atomics.
 * SINGLE-CORE ONLY: mutual exclusion vs ISRs via PRIMASK, NOT vs a 2nd core
 * (this would be WRONG on dual-core H7/MP1). Ref: micro_ros_stm32cubemx_utils#112,
 * RIOT core/lib/atomic_c11.c. Memory-order args are irrelevant on single-core. */
#include <stdint.h>
#include <stdbool.h>
#include "cmsis_compiler.h"   /* __get_PRIMASK / __disable_irq / __set_PRIMASK */

static inline uint32_t cs_enter(void) { uint32_t p = __get_PRIMASK(); __disable_irq(); return p; }
static inline void cs_exit(uint32_t p) { __set_PRIMASK(p); }

uint64_t __atomic_load_8(const volatile void *ptr, int mo) {
    (void)mo; uint32_t s = cs_enter(); uint64_t v = *(const volatile uint64_t*)ptr; cs_exit(s); return v; }
void __atomic_store_8(volatile void *ptr, uint64_t val, int mo) {
    (void)mo; uint32_t s = cs_enter(); *(volatile uint64_t*)ptr = val; cs_exit(s); }
uint64_t __atomic_exchange_8(volatile void *ptr, uint64_t val, int mo) {
    (void)mo; uint32_t s = cs_enter(); volatile uint64_t *p = ptr; uint64_t o = *p; *p = val; cs_exit(s); return o; }
bool __atomic_compare_exchange_8(volatile void *ptr, void *exp, uint64_t des,
                                 bool weak, int smo, int fmo) {
    (void)weak;(void)smo;(void)fmo; uint32_t s = cs_enter(); volatile uint64_t *p = ptr; uint64_t *e = exp;
    bool ok = (*p == *e); if (ok) *p = des; else *e = *p; cs_exit(s); return ok; }
uint64_t __atomic_fetch_add_8(volatile void *ptr, uint64_t val, int mo) {
    (void)mo; uint32_t s = cs_enter(); volatile uint64_t *p = ptr; uint64_t o = *p; *p = o + val; cs_exit(s); return o; }
```
> `__atomic_compare_exchange_8` is the **6-arg** form (`bool weak` + two memorder ints). Do not use a naive `LDRD` instead of the critical section (a 64-bit load isn't atomic vs a concurrent CAS on M4).

- [ ] **Step 2: Create the POSIX `usleep` stub** `firmware_stm32/Src/microros_posix_stubs.c`:
```c
/* usleep over FreeRTOS — required by librclc-sleep.c.obj (rclc_sleep_ms).
 * clock_gettime is intentionally NOT here — the utils' microros_time.c provides it. */
#include <unistd.h>
#include "FreeRTOS.h"
#include "task.h"
int usleep(useconds_t usec) { vTaskDelay(pdMS_TO_TICKS(usec / 1000U)); return 0; }
```
- [ ] **Step 3: Confirm both are in `C_SOURCES`** (added in Task 4): `grep -E 'microros_atomic64\.c|microros_posix_stubs\.c' firmware_stm32/Makefile`.
- [ ] **Step 4: Commit**
```bash
git add firmware_stm32/Src/microros_atomic64.c firmware_stm32/Src/microros_posix_stubs.c
git commit -m "feat(stm32): 64-bit atomic shim + usleep stub (Cortex-M4 micro-ROS link fix)

rcl uses __atomic_*_8 unconditionally; arm-none-eabi has no baremetal 64-bit
atomics (RCUTILS_NO_64_ATOMIC covers rcutils only). PRIMASK critical-section
shim per micro_ros_stm32cubemx_utils#112; usleep -> vTaskDelay for rclc_sleep_ms.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 6: Build `libmicroros.a` and link the firmware — THE F0 SMOKE

**Files:** none (produces `firmware_stm32/micro_ros_stm32cubemx_utils/microros_static_library/libmicroros/` and `firmware_stm32/build/`, both gitignored in Task 7).

- [ ] **Step 1: Pull the pinned builder image**
```bash
docker pull microros/micro_ros_static_library_builder@sha256:1482f3df56184ecc5d4a9d45ad9be0a17a84a91fca947d07f20d1678b23f6243
```
- [ ] **Step 2: Build `libmicroros.a` (non-interactive)** — from `firmware_stm32/` (the dir with the hand-written `Makefile`):
```bash
cd firmware_stm32
printf 'y' | docker run --rm -i \
  -v "$(pwd)":/project \
  --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library \
  microros/micro_ros_static_library_builder@sha256:1482f3df56184ecc5d4a9d45ad9be0a17a84a91fca947d07f20d1678b23f6243
```
`printf 'y' | … run --rm -i` (no `-t`) auto-answers the builder's interactive `read -p "...(y/n)"`; without it `set -e` aborts in a no-TTY context. Needs outbound network. The builder runs `make print_cflags` in your hand-written Makefile → ABI-matched library.

- [ ] **Step 3: Assert the library exists**
```bash
test -f micro_ros_stm32cubemx_utils/microros_static_library/libmicroros/libmicroros.a && \
test -d micro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include && echo "libmicroros OK"
```
A `make print_cflags` error here means the Task 4 TAB/flags are wrong.

- [ ] **Step 4: Link the firmware — the F0 acceptance gate**
```bash
make -j"$(nproc)"
```
**PASS** (F0 done): build ends with `objcopy`/`size`, no linker errors; `build/firmware_stm32.elf` exists with a `text` of tens-to-hundreds of KB (micro-ROS linked in). Confirm: `test -f build/firmware_stm32.elf && arm-none-eabi-size build/firmware_stm32.elf`.

If the link FAILS, match the signature (empirically characterized 2026-06-04 against the real `libmicroros.a`):
- **`undefined reference to '__atomic_load_8'`** (and `__atomic_store_8`/`__atomic_exchange_8`/`__atomic_compare_exchange_8`) → the **expected** Cortex-M4 case: the Task 5a `microros_atomic64.c` shim is missing from `C_SOURCES`. From `rcl` (time/timer/client); **not** fixed by `RCUTILS_NO_64_ATOMIC` (rcutils-only). Add the shim (Task 5a / Task 4) — do NOT rebuild `libmicroros.a`.
- **`undefined reference to 'usleep'`** (from `librclc-sleep.c.obj`) → add `microros_posix_stubs.c` (Task 5a).
- **`undefined reference to '__sync_synchronize'`** → benign barrier; if it appears, add `void __sync_synchronize(void){__asm volatile("dmb");}` to the shim. (Arduino/PlatformIO symptom; on the native route the real wall is `__atomic_*_8`.)
- **`...elf uses VFP register arguments, libmicroros.a(...) does not`** → float-ABI mismatch. **Empirically does NOT occur** with this flow (all 2014 members hard-float). If it does, `print_cflags` didn't forward `-mfloat-abi=hard` (TAB missing). Fix Task 4, rebuild (Step 2).
- **`undefined reference to '_sbrk'`** → `Src/syscalls.c` missing from the build (Task 2 Step 5). (`clock_gettime` is provided by `microros_time.c` — do not re-stub it.)

> **Stronger oracle (recommended for CI):** plain `make` uses `--gc-sections`, which can *mask* a latent 64-bit-atomics gap by dead-stripping the atomic-using functions if not reached. Conservatively also do a whole-archive link: `arm-none-eabi-gcc <flags> -Wl,--whole-archive libmicroros.a -Wl,--no-whole-archive -Wl,--no-gc-sections -Wl,--unresolved-symbols=report-all -nostartfiles -e 0 -o /tmp/lp.elf` and require zero `__atomic_*_8` / VFP errors (expected app-provided `clock_gettime`/`usleep`/`_sbrk`/typesupport symbols don't count).

- [ ] **Step 5: Record the result** — capture the `arm-none-eabi-size` line in the task report (evidence F0 passed). No commit (outputs gitignored).

---

### Task 7: Project `.gitignore` + dedicated CI workflow

**Files:**
- Create: `firmware_stm32/.gitignore`
- Create: `.github/workflows/stm32-f446re-f0.yml`

- [ ] **Step 1: `.gitignore`** — keep all hand-written + vendored source; drop build artifacts. Create `firmware_stm32/.gitignore`:
```
build/
micro_ros_stm32cubemx_utils/microros_static_library/libmicroros/
```
(`Makefile`, `STM32F446RETX_FLASH.ld`, `Inc/`, `Src/`, `vendor/` (submodules), `PINS.md` stay tracked.)

- [ ] **Step 2: Verify the hand-written tree is tracked**
```bash
git -C firmware_stm32 ls-files | grep -E 'Src/main.c|Makefile|STM32F446RETX_FLASH.ld' | head
```
Expected: lists `Makefile`, the linker script, and `Src/main.c`.

- [ ] **Step 3: CI workflow** (separate from `.github/parse_platformio.py`, which only enumerates `platformio.ini` envs and can't see a Make project). Create `.github/workflows/stm32-f446re-f0.yml`:
```yaml
name: stm32-f446re-microros-f0
on:
  push:
    paths: ['firmware_stm32/**', '.github/workflows/stm32-f446re-f0.yml']
  pull_request:
    paths: ['firmware_stm32/**', '.github/workflows/stm32-f446re-f0.yml']
jobs:
  f0-link-smoke:
    runs-on: ubuntu-latest          # amd64 — matches pinned image sub-digest 392246e5
    steps:
      - uses: actions/checkout@v4
        with:
          submodules: recursive      # vendored CMSIS/HAL/FreeRTOS + micro_ros utils @ pinned commits
      - name: Install host ARM toolchain + make
        run: sudo apt-get update && sudo apt-get install -y gcc-arm-none-eabi binutils-arm-none-eabi make
      - name: Build libmicroros.a (non-interactive, pinned image)
        working-directory: firmware_stm32
        run: |
          docker pull microros/micro_ros_static_library_builder@sha256:1482f3df56184ecc5d4a9d45ad9be0a17a84a91fca947d07f20d1678b23f6243
          printf 'y' | docker run --rm -i \
            -v "$PWD":/project \
            --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library \
            microros/micro_ros_static_library_builder@sha256:1482f3df56184ecc5d4a9d45ad9be0a17a84a91fca947d07f20d1678b23f6243
      - name: Assert library exists
        working-directory: firmware_stm32
        run: test -f micro_ros_stm32cubemx_utils/microros_static_library/libmicroros/libmicroros.a
      - name: Link firmware (THE F0 test)
        working-directory: firmware_stm32
        run: make -j"$(nproc)"
      - name: Assert ELF + report size
        working-directory: firmware_stm32
        run: |
          test -f build/firmware_stm32.elf
          arm-none-eabi-size build/firmware_stm32.elf
```
- [ ] **Step 4: Commit**
```bash
git add firmware_stm32/.gitignore .github/workflows/stm32-f446re-f0.yml
git commit -m "ci(stm32): F0 link-smoke workflow + project gitignore (GUI-free Make project)

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## On completion (milestone tag)

After all tasks pass and the branch is merged (per superpowers:finishing-a-development-branch):
```bash
git tag -a stm32cube-p2-skeleton-libmicroros -m "STM32 native port — Plan 2 (F0): hand-written F446RE HAL/FreeRTOS skeleton links libmicroros cleanly (GUI-free)"
```

---

## Self-Review (against the spec)

- **Spec coverage (Ф0 / §8):** "project compiles + `libmicroros.a` links (ABI-smoke)" — Tasks 2–6 deliver it; Task 6 Step 4 is the gate, and the FAIL signatures map to spec §9 risks #1 (float-ABI) + the atomics trap in [[stm32cube-microros-gotchas]]. CI (spec §7, separate workflow) = Task 7. FreeRTOS + ≥24 KB task = Task 2/Task 5. UART transport = `it_transport.c` (`dma_transport.c` for Plan 4).
- **GUI-free check:** no STM32CubeMX, no `.ioc`, no `[GUI]` step anywhere. The project is hand-written + vendored OSS submodules, built by a hand-written Makefile. `micro_ros_stm32cubemx_utils` / `cubemx_transport_*` are upstream library/API names (no GUI involved) and are kept.
- **Consistency:** the pinned utils commit (`a5b2127…`) and image digest (`sha256:1482f3df…`) are identical across PINS, the submodule, the Docker build, and CI. `huart2`/USART2/PA2-PA3 consistent across Tasks 2 & 5.
- **Empirically grounded:** linker script, `FreeRTOSConfig.h`, atomic shim, and the libmicroros build were all validated on this machine (Option 1 link 73.7 KB; FreeRTOS scheduled 12 449 ticks/3 s in Renode) — Task 2 assembles those proven pieces. The one thing not yet built here is the *combined* HAL+FreeRTOS+micro-ROS firmware; its pieces are individually proven and gluing them needs no GUI.
