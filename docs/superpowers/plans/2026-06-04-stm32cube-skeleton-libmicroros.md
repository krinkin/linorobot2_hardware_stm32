# STM32Cube Port — Plan 2: F446RE Skeleton + libmicroros ABI-Smoke (Phase F0)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
>
> **⚠️ Execution environment:** This plan CANNOT run in the default sandbox — it needs **STM32CubeMX (GUI)**, a host **`arm-none-eabi-gcc`**, and **Docker**, none of which are present here. Execute it on a workstation that has them. The host-test sandbox (Plan 1) needed none of these; this one does. Where a step is GUI-only it is marked `[GUI]`; where a value depends on what CubeMX emits, the step gives the exact expected value plus a `grep` to verify it (that is verification, not a placeholder).

**Goal:** Stand up a real-but-minimal NUCLEO-F446RE firmware (CubeMX + FreeRTOS) that **links `libmicroros.a` cleanly** — the ABI/atomics smoke test that de-risks the entire native route (spec `docs/STM32CUBE_PORTING_PLAN.md` §8 Ф0, §9 risks #1–#2).

**Architecture:** Official native route (spec §2 Route B): an STM32CubeMX project with **Makefile** toolchain + **FreeRTOS (CMSIS-OS v2)**, the official **`micro_ros_stm32cubemx_utils`** (pinned `jazzy` commit) vendored as a submodule, and **`libmicroros.a`** built once by the Docker image `microros/micro_ros_static_library_builder:jazzy`. ABI match is guaranteed *by flag forwarding* — the Docker builder reads the app's CFLAGS via a `print_cflags` Makefile target and compiles the library with the same `-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard`; `colcon.meta` sets `RCUTILS_NO_64_ATOMIC=ON` (covers `rcutils` only; `rcl` still needs an app-provided `__atomic_*_8` shim — Task 5a). There is **no application logic** in F0 — only enough to force the linker to pull in micro-ROS symbols.

**Tech Stack:** STM32CubeMX, GNU Arm Embedded toolchain (`arm-none-eabi-gcc`), GNU Make, Docker, FreeRTOS/CMSIS-OS v2, micro-ROS (`rcl`/`rclc`/`rmw_microxrcedds`) via `micro_ros_stm32cubemx_utils@a5b2127`. Target: STM32F446RETx (Cortex-M4F).

**Pinned references (verified live against `micro-ROS/micro_ros_stm32cubemx_utils@jazzy`):**
- utils commit: `a5b2127495ae0ab53d7a1360beaf17822309a3cc`
- builder image (multi-arch index digest): `microros/micro_ros_static_library_builder@sha256:1482f3df56184ecc5d4a9d45ad9be0a17a84a91fca947d07f20d1678b23f6243` (amd64 sub-digest `392246e5…`; matches GitHub `ubuntu-latest` runners)

---

## Where this plan sits

Plan 2 of 7 for the native STM32Cube port (see `docs/STM32CUBE_PORTING_PLAN.md`). Plan 1 (host test tier + odometry refactor) is done and merged (`stm32cube-p1-host-test-tier`). This plan delivers the **Ф0 skeleton**. It does NOT: boot-verify (Plan 3, Renode), wire a working transport/agent round-trip (Plan 4), or implement encoder/PWM/IMU (Plans 5–6).

**F0 is intentionally a "does it LINK" gate, not a "does it RUN" gate.** A clean link proves the float-ABI and 64-bit-atomics issues — the two things that killed the unsupported PlatformIO route (spec §2 Route A) — are resolved. Booting is a separate, runtime concern owned by Plan 3.

**Empirically validated (2026-06-04).** The real `libmicroros.a` was built via the official jazzy Docker flow and inspected with the Cortex-M4F toolchain: **float-ABI = PASS** (all 2014 members hard-float `Tag_ABI_VFP_args: VFP registers` — the Route-A VFP wall does NOT occur with this flow), and **64-bit atomics = needs a shim** (`rcl` references `__atomic_*_8` that `arm-none-eabi` cannot satisfy on M4). A PRIMASK critical-section shim (Task 5a) was proven to make the realistic micro-ROS symbol set link cleanly (hard-float, strict `--whole-archive --no-gc-sections`). So F0 is **two** gates — VFP **and** atomics/POSIX — reflected in Task 5a and the Task 6 FAIL signatures.

**F0 closed end-to-end on a real ELF (2026-06-04).** Beyond archive inspection, a real bare-metal F446 image (ST CMSIS startup + `system_stm32f4xx.c` + a standard F446RE linker script + the Task 5a shim + `microros_time.c`/stubs + the official `libmicroros.a`) was linked on a Cortex-M4F toolchain: **clean link, 73.7 KB Flash / 22.6 KB static RAM, hard-float**, with `rcl`'s timer/time/client 64-bit-atomic path linked in. Control proof: the identical link **without** the shim fails on `__atomic_compare_exchange_8`; **with** it, links. This validates the integration (startup + linker + micro-ROS + shim) independent of the CubeMX/FreeRTOS specifics (which remain Plan 3 runtime concerns). The bare-metal harness is a *stand-in* for the GUI-generated CubeMX/FreeRTOS project — Task 2's GUI generation still runs on a workstation; but the load-bearing F0 risk (the ABI/atomics link) is now empirically retired.

---

## File Structure

The CubeMX project lives in a new top-level `firmware_stm32/` (alongside `firmware/`). CubeMX generates most of it; the items you author are marked.

| Path | Responsibility | Origin |
|---|---|---|
| `firmware_stm32/PINS.md` | Records the exact utils commit + image digest + why | Author (Task 1) |
| `firmware_stm32/<proj>.ioc` | CubeMX project definition (F446RETx) | CubeMX `[GUI]` (Task 2) |
| `firmware_stm32/Makefile` | Build script (+ micro-ROS addon block) | CubeMX, then patched (Task 4) |
| `firmware_stm32/Core/` , `Drivers/`, `Middlewares/` | Generated HAL/FreeRTOS/CMSIS sources | CubeMX (Task 2) |
| `firmware_stm32/Core/Src/main.c` (or `freertos.c`) | Minimal micro-ROS task (USER CODE blocks) | Author (Task 5) |
| `firmware_stm32/Core/Src/microros_atomic64.c` | 64-bit atomic shim — Cortex-M4 (**REQUIRED**, empirically validated) | Author (Task 5a) |
| `firmware_stm32/Core/Src/microros_posix_stubs.c` | `usleep`→`osDelay` (**REQUIRED**) | Author (Task 5a) |
| `firmware_stm32/startup_*.s`, `*_FLASH.ld` | Startup + linker script | CubeMX (Task 2) |
| `firmware_stm32/micro_ros_stm32cubemx_utils/` | Vendored utils (submodule @ pinned commit) | Task 3 |
| `firmware_stm32/.gitignore` | Ignore `build/` + `libmicroros/`, **keep** generated sources | Author (Task 7) |
| `.github/workflows/stm32-f446re-f0.yml` | Dedicated F0 link-smoke CI (separate from `parse_platformio.py`) | Author (Task 7) |

---

### Task 1: Scaffold `firmware_stm32/` and record pinned references

**Files:**
- Create: `firmware_stm32/PINS.md`

- [ ] **Step 1: Confirm the host toolchain is present**

Run:
```bash
arm-none-eabi-gcc --version | head -n1
make --version | head -n1
docker --version
```
Expected: each prints a version. If any is missing, install (Debian/Ubuntu):
```bash
sudo apt-get update && sudo apt-get install -y gcc-arm-none-eabi binutils-arm-none-eabi make docker.io git
```
If `arm-none-eabi-gcc`/`docker` cannot be installed in this environment, STOP and report BLOCKED — this plan requires them (see the header note).

- [ ] **Step 2: Record the pins**

Create `firmware_stm32/PINS.md`:
```markdown
# Pinned references for the STM32Cube micro-ROS skeleton

These pins make the libmicroros build inputs explicit. `jazzy` is a MOVING branch;
always check out the commit, not just `-b jazzy`.

- micro_ros_stm32cubemx_utils commit: a5b2127495ae0ab53d7a1360beaf17822309a3cc  (jazzy HEAD as of 2026-06-04)
- builder image (index digest): microros/micro_ros_static_library_builder@sha256:1482f3df56184ecc5d4a9d45ad9be0a17a84a91fca947d07f20d1678b23f6243
  - amd64 sub-digest: sha256:392246e55107aeda92db5b191fce2a7d330428d978e2936a03096c78c8d3ae30 (GitHub ubuntu runners)

Note: the build is NOT bit-reproducible even with these pins — inside the container
the builder clones geometry2@jazzy and vcs-imports extra_packages (moving HEADs) and
apt-installs an unpinned gcc-arm-none-eabi. For strict reproducibility, vendor the
produced libmicroros.a (see Plan 2 Task 7 / §4).
```

- [ ] **Step 3: Commit**
```bash
git add firmware_stm32/PINS.md
git commit -m "build(stm32): scaffold firmware_stm32 and pin micro-ROS build refs

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 2: Generate the CubeMX F446RE project `[GUI]`

Generates the real HAL/FreeRTOS project. **All sub-steps are STM32CubeMX GUI actions** unless noted; there is no FOSS replacement for `.ioc` authoring. The generated tree is firmware *source* and gets committed (Task 7), so this GUI step is one-time — CI builds the committed output.

**Files:**
- Create (generated): `firmware_stm32/<proj>.ioc`, `firmware_stm32/Makefile`, `firmware_stm32/Core/`, `firmware_stm32/Drivers/`, `firmware_stm32/Middlewares/`, `firmware_stm32/startup_stm32f446xx.s`, `firmware_stm32/STM32F446RETX_FLASH.ld`

- [ ] **Step 1: New project for the exact part**
`[GUI]` New Project → select MCU **STM32F446RETx** (LQFP64). Do NOT start from the upstream `sample_project.ioc` — it targets STM32F446**ZE**Tx (LQFP144) with USART3 on PD8/PD9, which is the wrong package and wrong UART for a NUCLEO-F446RE.

- [ ] **Step 2: Toolchain = Makefile (mandatory)**
`[GUI]` Project Manager → Project → **Toolchain/IDE = `Makefile`**. The entire micro-ROS flow keys off the generated `Makefile`.

- [ ] **Step 3: Clock**
`[GUI]` System Core → RCC → **HSE = `BYPASS Clock Source`** (the Nucleo gets 8 MHz from the ST-Link MCO). Clock Configuration tab → target **180 MHz**: `PLLM=4, PLLN=180, PLLP=/2`, APB1 `/4`, APB2 `/2` (CubeMX auto-sets VOS Scale1 + Over-Drive + FLASH latency 5). The exact clock does not affect the F0 *link*; 84/100 MHz is acceptable too.

- [ ] **Step 4: FreeRTOS (required)**
`[GUI]` Middleware → **FREERTOS → Interface = `CMSIS_V2`**. Required: the utils' `microros_time.c` includes `cmsis_os.h`, the allocator is FreeRTOS-derived, and the micro-ROS task needs a large stack.

- [ ] **Step 5: USART2 (the Nucleo VCP)**
`[GUI]` Connectivity → **USART2 → Mode = `Asynchronous`**, 115200 8N1. CubeMX auto-assigns **PA2 (TX) / PA3 (RX)** — the ST-Link Virtual COM Port on NUCLEO-F446RE. (Do not use USART3/PD8-9 from the upstream sample.)

- [ ] **Step 6: USART2 interrupt**
`[GUI]` NVIC → enable **USART2 global interrupt** (needed by the `it_transport.c` chosen in Task 4). *(If you later switch to `dma_transport.c`, also add USART2 DMA Tx + DMA Rx with Rx = Circular; not needed for F0.)*

- [ ] **Step 7: micro-ROS task stack ≥ 24 KB**
`[GUI]` FreeRTOS → Tasks → set `defaultTask` **Stack Size = `6144`**. The CubeMX Tasks dialog field is in **words**, so 6144 words = 24576 bytes (> the 10 KB minimum the utils README requires). You will verify the generated value numerically in Task 5.

- [ ] **Step 8: Ensure syscalls are generated**
`[GUI]` Project Manager → Code Generator → keep defaults (generate `.c/.h` pairs). Confirm `Core/Src/syscalls.c` is produced (provides `_sbrk`/`_write` so newlib links). Keep `-specs=nano.specs` (default); do not switch to `nosys.specs`.

- [ ] **Step 9: Generate**
`[GUI]` **Generate Code**.

- [ ] **Step 10: Verify the generated hard-float flags (NOT GUI — run in shell)**
```bash
cd firmware_stm32
grep -E '^(CPU|FPU|FLOAT-ABI|MCU) ' Makefile
```
Expected:
```
CPU = -mcpu=cortex-m4
FPU = -mfpu=fpv4-sp-d16
FLOAT-ABI = -mfloat-abi=hard
```
If `FLOAT-ABI` is `soft`/absent, the FreeRTOS/FPU options were misconfigured — fix in CubeMX and regenerate (a soft-float app will fail to link against the hard-float library in Task 6).

- [ ] **Step 11: Commit the generated project**
```bash
git add firmware_stm32/
git commit -m "build(stm32): generate CubeMX F446RE project (Makefile + FreeRTOS + USART2)

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 3: Vendor `micro_ros_stm32cubemx_utils` at the pinned commit

**Files:**
- Create: `firmware_stm32/micro_ros_stm32cubemx_utils/` (git submodule) + `.gitmodules`

- [ ] **Step 1: Add the submodule and pin it**
```bash
cd firmware_stm32
git submodule add https://github.com/micro-ROS/micro_ros_stm32cubemx_utils.git micro_ros_stm32cubemx_utils
cd micro_ros_stm32cubemx_utils
git checkout a5b2127495ae0ab53d7a1360beaf17822309a3cc
cd ..
```

- [ ] **Step 2: Verify the pin and required files exist**
```bash
git -C micro_ros_stm32cubemx_utils rev-parse HEAD
test -f micro_ros_stm32cubemx_utils/extra_sources/microros_transports/it_transport.c
test -f micro_ros_stm32cubemx_utils/microros_static_library/library_generation/colcon.meta
test -f micro_ros_stm32cubemx_utils/extra_sources/microros_time.c
echo OK
```
Expected: prints `a5b2127495ae0ab53d7a1360beaf17822309a3cc` then `OK`.

- [ ] **Step 3: Commit**
```bash
cd ..
git add .gitmodules firmware_stm32/micro_ros_stm32cubemx_utils
git commit -m "build(stm32): vendor micro_ros_stm32cubemx_utils @ a5b2127 (jazzy)

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 4: Patch the generated Makefile with the micro-ROS addon block

**Files:**
- Modify: `firmware_stm32/Makefile`

- [ ] **Step 1: Insert the addon block**
Open `firmware_stm32/Makefile`. Immediately **before** the `# build the application` section, insert this block **verbatim**:
```makefile
#######################################
# micro-ROS addons
#######################################
LDFLAGS += micro_ros_stm32cubemx_utils/microros_static_library/libmicroros/libmicroros.a
C_INCLUDES += -Imicro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include

# Add micro-ROS utils
C_SOURCES += micro_ros_stm32cubemx_utils/extra_sources/custom_memory_manager.c
C_SOURCES += micro_ros_stm32cubemx_utils/extra_sources/microros_allocators.c
C_SOURCES += micro_ros_stm32cubemx_utils/extra_sources/microros_time.c

# Custom transport: it_transport is simplest for F0 (USART2 global IT only, no DMA)
C_SOURCES += micro_ros_stm32cubemx_utils/extra_sources/microros_transports/it_transport.c

# REQUIRED on Cortex-M4 (Task 5a, empirically validated): 64-bit-atomics shim + usleep.
# rcl (time/timer/client) calls __atomic_*_8 unconditionally; arm-none-eabi has no
# baremetal 64-bit atomics; RCUTILS_NO_64_ATOMIC=ON covers rcutils only. Without these
# the F0 link fails: "undefined reference to __atomic_load_8" (and to usleep).
C_SOURCES += Core/Src/microros_atomic64.c
C_SOURCES += Core/Src/microros_posix_stubs.c

print_cflags:
	@echo $(CFLAGS)
```

- [ ] **Step 2: Fix the TAB (critical)**
The `@echo $(CFLAGS)` recipe line **must** begin with a literal TAB, not spaces. Verify:
```bash
cd firmware_stm32
grep -nP '^\t@echo \$\(CFLAGS\)' Makefile
```
Expected: one match (line number printed). If no match, the line is space-indented — replace its leading whitespace with a single TAB. (A space-indented recipe yields `Makefile:NN: *** missing separator. Stop.`)

- [ ] **Step 3: Sanity-check `print_cflags` emits the hard-float flags**
```bash
make print_cflags | grep -o -- '-mfloat-abi=hard'
```
Expected: prints `-mfloat-abi=hard`. (This is exactly the string the Docker builder will forward into `libmicroros.a` in Task 6, guaranteeing ABI match.) If empty, fix Task 2 Step 10 first.

- [ ] **Step 4: Commit**
```bash
cd ..
git add firmware_stm32/Makefile
git commit -m "build(stm32): add micro-ROS addon block to Makefile (print_cflags, link libmicroros)

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 5: Add the minimal micro-ROS task (force-link symbols)

The link only exercises `libmicroros.a` if the app references its symbols. This adds a FreeRTOS task that initializes the custom-allocator + transport + `rclc_support` — the same symbol set as the upstream `sample_main.c`, with `huart3 → huart2`.

**Files:**
- Modify: `firmware_stm32/Core/Src/main.c` (or `Core/Src/freertos.c` if `StartDefaultTask` lives there)

- [ ] **Step 1: Add the micro-ROS includes**
In the generated file that defines `StartDefaultTask`, inside `/* USER CODE BEGIN Includes */`:
```c
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
```

- [ ] **Step 2: Declare the transport + allocator prototypes**
Inside `/* USER CODE BEGIN 4 */` (file scope):
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

- [ ] **Step 3: Put the micro-ROS init in the default task body**
Inside the `StartDefaultTask` function's `/* USER CODE BEGIN 5 */` (replace the generated `for(;;) { osDelay(1); }` placeholder):
```c
extern UART_HandleTypeDef huart2;

rmw_uros_set_custom_transport(
    true,
    (void *) &huart2,
    cubemx_transport_open,
    cubemx_transport_close,
    cubemx_transport_write,
    cubemx_transport_read);

rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
freeRTOS_allocator.allocate = microros_allocate;
freeRTOS_allocator.deallocate = microros_deallocate;
freeRTOS_allocator.reallocate = microros_reallocate;
freeRTOS_allocator.zero_allocate = microros_zero_allocate;
if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
    Error_Handler();
}

rclc_support_t support;
rcl_allocator_t allocator = rcl_get_default_allocator();
rcl_node_t node;
rclc_support_init(&support, 0, NULL, &allocator);
rclc_node_init_default(&node, "cubemx_node", "", &support);

for (;;) {
    osDelay(1000);
}
```

- [ ] **Step 4: Confirm the task stack is ≥ 24 KB (bytes)**
Find `defaultTask_attributes` in the generated source and confirm `stack_size` resolves to 24576 bytes. CMSIS-OS v2 `stack_size` is in **bytes**:
```bash
grep -A4 'defaultTask_attributes' firmware_stm32/Core/Src/*.c
```
Expected: `.stack_size = 6144 * 4` (= 24576). If CubeMX wrote a smaller value (e.g. `128 * 4`), fix the Tasks stack in CubeMX (Task 2 Step 7) and regenerate, or edit the attribute to `6144 * 4`.

- [ ] **Step 5: Commit**
```bash
git add firmware_stm32/Core/Src/
git commit -m "feat(stm32): minimal micro-ROS task to force-link libmicroros (F0)

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 5a: 64-bit atomic shim + `usleep` stub — REQUIRED for the F0 link

**Why (empirically validated 2026-06-04, corroborated by micro_ros_stm32cubemx_utils#112).** The real `libmicroros.a` references `__atomic_{load,store,exchange,compare_exchange}_8` from `librcl-time.c.obj` / `librcl-timer.c.obj` / `librcl-client.c.obj` (+ `librcutils-fault_injection.c.obj`), and `arm-none-eabi` 13.2.1 provides **no** 64-bit atomics for the Cortex-M4 hard multilib (no `LDREXD`/`STREXD`; no baremetal `libatomic`). `RCUTILS_NO_64_ATOMIC=ON` neutralizes only `rcutils`' `atomic_64bits.c`; `rcl`'s `time.c`/`timer.c`/`client.c` use 64-bit atomics **unconditionally** — and our firmware drives an `rcl` 50 Hz timer, which reaches that path. The maintainer's stated fix is: *"provide an implementation of `__atomic_load_8`."* Separately, `rclc_sleep_ms` needs `usleep` (NOT shipped by the utils; `clock_gettime` IS shipped by `microros_time.c` — do not re-stub it). A PRIMASK critical-section shim was proven to link the realistic micro-ROS symbol set cleanly (hard-float, strict `--whole-archive --no-gc-sections`).

**Files:**
- Create: `firmware_stm32/Core/Src/microros_atomic64.c`
- Create: `firmware_stm32/Core/Src/microros_posix_stubs.c`

- [ ] **Step 1: Create the 64-bit atomic shim**
Create `firmware_stm32/Core/Src/microros_atomic64.c`:
```c
/* 64-bit atomics shim for single-core Cortex-M (STM32F446, M4F).
 * Resolves __atomic_*_8 referenced by libmicroros.a (rcl time/timer/client,
 * rcutils fault_injection) — arm-none-eabi has no baremetal 64-bit atomics.
 * SINGLE-CORE ONLY: mutual exclusion vs ISRs via PRIMASK, NOT vs a 2nd core
 * (this would be WRONG on dual-core H7/MP1). Ref: micro_ros_stm32cubemx_utils#112,
 * RIOT core/lib/atomic_c11.c. Memory-order args are irrelevant on single-core. */
#include <stdint.h>
#include <stdbool.h>
#include "cmsis_compiler.h"   /* __get_PRIMASK / __disable_irq / __set_PRIMASK / __DMB */

static inline uint32_t cs_enter(void) {
    uint32_t primask = __get_PRIMASK();   /* save */
    __disable_irq();
    return primask;
}
static inline void cs_exit(uint32_t primask) {
    __set_PRIMASK(primask);               /* restore (re-enables only if it was enabled) */
}

uint64_t __atomic_load_8(const volatile void *ptr, int memorder) {
    (void)memorder; uint32_t s = cs_enter();
    uint64_t v = *(const volatile uint64_t *)ptr; cs_exit(s); return v;
}
void __atomic_store_8(volatile void *ptr, uint64_t val, int memorder) {
    (void)memorder; uint32_t s = cs_enter();
    *(volatile uint64_t *)ptr = val; cs_exit(s);
}
uint64_t __atomic_exchange_8(volatile void *ptr, uint64_t val, int memorder) {
    (void)memorder; uint32_t s = cs_enter(); volatile uint64_t *p = ptr;
    uint64_t old = *p; *p = val; cs_exit(s); return old;
}
bool __atomic_compare_exchange_8(volatile void *ptr, void *expected, uint64_t desired,
                                 bool weak, int success_memorder, int failure_memorder) {
    (void)weak; (void)success_memorder; (void)failure_memorder;
    uint32_t s = cs_enter(); volatile uint64_t *p = ptr; uint64_t *exp = expected;
    bool ok = (*p == *exp); if (ok) *p = desired; else *exp = *p; cs_exit(s); return ok;
}
uint64_t __atomic_fetch_add_8(volatile void *ptr, uint64_t val, int memorder) {
    (void)memorder; uint32_t s = cs_enter(); volatile uint64_t *p = ptr;
    uint64_t old = *p; *p = old + val; cs_exit(s); return old;
}
```
> Signature note: `__atomic_compare_exchange_8` is the **6-arg** form (`bool weak` + two memorder ints) — a single-memorder version silently mismatches the call site. Do not use a naive `LDRD` instead of the critical section (a 64-bit load is not atomic vs a concurrent CAS on M4).

- [ ] **Step 2: Create the POSIX `usleep` stub**
Create `firmware_stm32/Core/Src/microros_posix_stubs.c`:
```c
/* usleep over FreeRTOS — required by librclc-sleep.c.obj (rclc_sleep_ms).
 * NOTE: clock_gettime is intentionally NOT here — the utils' microros_time.c
 * already provides it (FreeRTOS-tick based). Do not duplicate it. */
#include <unistd.h>
#include "cmsis_os.h"
int usleep(useconds_t usec) {
    osDelay(usec / 1000U);   /* tick-granular; sub-ms rounds toward 0 */
    return 0;
}
```

- [ ] **Step 3: Confirm both are in the Makefile `C_SOURCES`** (added in Task 4 Step 1)
```bash
grep -E 'microros_atomic64\.c|microros_posix_stubs\.c' firmware_stm32/Makefile
```
Expected: both lines present.

- [ ] **Step 4: Commit**
```bash
git add firmware_stm32/Core/Src/microros_atomic64.c firmware_stm32/Core/Src/microros_posix_stubs.c
git commit -m "feat(stm32): 64-bit atomic shim + usleep stub (Cortex-M4 micro-ROS link fix)

rcl uses __atomic_*_8 unconditionally; arm-none-eabi has no baremetal 64-bit
atomics (RCUTILS_NO_64_ATOMIC covers rcutils only). PRIMASK critical-section
shim per micro_ros_stm32cubemx_utils#112; usleep -> osDelay for rclc_sleep_ms.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 6: Build `libmicroros.a` and link the firmware — THE F0 SMOKE

**Files:** none (produces `firmware_stm32/micro_ros_stm32cubemx_utils/microros_static_library/libmicroros/` and `firmware_stm32/build/`, both gitignored in Task 7).

- [ ] **Step 1: Pull the pinned builder image**
```bash
docker pull microros/micro_ros_static_library_builder@sha256:1482f3df56184ecc5d4a9d45ad9be0a17a84a91fca947d07f20d1678b23f6243
```

- [ ] **Step 2: Build `libmicroros.a` (non-interactive)**
From `firmware_stm32/` (the dir with the patched `Makefile`):
```bash
cd firmware_stm32
printf 'y' | docker run --rm -i \
  -v "$(pwd)":/project \
  --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library \
  microros/micro_ros_static_library_builder@sha256:1482f3df56184ecc5d4a9d45ad9be0a17a84a91fca947d07f20d1678b23f6243
```
The `printf 'y' | … run --rm -i` (no `-t`) auto-answers the script's interactive `read -p "...continue? (y/n)"` prompt; without it, `set -e` aborts the build in a no-TTY context. Requires outbound network (the builder clones/imports packages and apt-installs its toolchain).

- [ ] **Step 3: Assert the library was produced**
```bash
test -f micro_ros_stm32cubemx_utils/microros_static_library/libmicroros/libmicroros.a && \
test -d micro_ros_stm32cubemx_utils/microros_static_library/libmicroros/microros_include && echo "libmicroros OK"
```
Expected: `libmicroros OK`. If the build failed, read the Docker output: a `make print_cflags` error here means the Task 4 TAB/flags are wrong.

- [ ] **Step 4: Link the firmware — the F0 acceptance gate**
```bash
make -j"$(nproc)"
```
**PASS** (F0 done): the build ends with `objcopy`/`size` output and no linker errors, e.g.:
```
arm-none-eabi-objcopy -O ihex build/firmware_stm32.elf build/firmware_stm32.hex
arm-none-eabi-objcopy -O binary -S build/firmware_stm32.elf build/firmware_stm32.bin
arm-none-eabi-size build/firmware_stm32.elf
   text	   data	    bss	    dec	    hex	filename
 1xxxxx	   yyyy	  zzzzz	 dddddd	 hhhhhh	build/firmware_stm32.elf
```
A `text` of tens-to-hundreds of KB means micro-ROS was linked in. Confirm:
```bash
test -f build/*.elf && arm-none-eabi-size build/*.elf
```

If the link FAILS, match the signature (empirically characterized 2026-06-04 against the real `libmicroros.a`):
- **`undefined reference to '__atomic_load_8'`** (and `__atomic_store_8`/`__atomic_exchange_8`/`__atomic_compare_exchange_8`) → the **expected** Cortex-M4 case: the Task 5a `microros_atomic64.c` shim is missing from `C_SOURCES`. These come from `rcl` (time/timer/client) and are **not** fixed by `RCUTILS_NO_64_ATOMIC` (rcutils-only). Add the shim (Task 5a / Task 4 Step 1) — do NOT rebuild `libmicroros.a` for this.
- **`undefined reference to 'usleep'`** (from `librclc-sleep.c.obj`) → add `microros_posix_stubs.c` (Task 5a).
- **`undefined reference to '__sync_synchronize'`** → benign memory barrier; the Task 5a shim defines it. This was the *Arduino/PlatformIO* symptom (from `rcutils-atomic_64bits`); on the native route the real wall is `__atomic_*_8`, above.
- **`<proj>.elf uses VFP register arguments, libmicroros.a(...) does not`** → float-ABI mismatch. **Empirically this does NOT occur** with the official Docker/Makefile flow (all 2014 members verified hard-float). If it does, `print_cflags` didn't forward `-mfloat-abi=hard` (TAB missing → build aborted, or stale lib). Fix Task 4 Step 2/3, rebuild (Step 2).
- **`undefined reference to '_sbrk'`** → `syscalls.c` not in the build; regenerate with syscalls enabled (Task 2 Step 8). (`clock_gettime` is already provided by the utils' `microros_time.c` — do not re-stub it; `_gettimeofday` is a benign newlib warning unless your code calls `gettimeofday()`.)

> **Stronger oracle (recommended for CI):** a plain `make` uses `--gc-sections`, which can *mask* a latent 64-bit-atomics gap by dead-stripping the atomic-using functions if the reached code happens not to call them. To gate conservatively, also do a whole-archive link: `arm-none-eabi-gcc <flags> -Wl,--whole-archive libmicroros.a -Wl,--no-whole-archive -Wl,--no-gc-sections -Wl,--unresolved-symbols=report-all -nostartfiles -e 0 -o /tmp/lp.elf` and require zero `__atomic_*_8` / VFP errors (the expected app-provided `clock_gettime`/`usleep`/`_sbrk`/typesupport symbols don't count).

- [ ] **Step 5: Record the result**
This task makes no commit (its outputs are gitignored). Capture the `arm-none-eabi-size` line in the task report — it is the evidence F0 passed.

---

### Task 7: Project `.gitignore` + dedicated CI workflow

**Files:**
- Create: `firmware_stm32/.gitignore`
- Create: `.github/workflows/stm32-f446re-f0.yml`

- [ ] **Step 1: Add a project `.gitignore` that keeps generated SOURCE but drops build artifacts**
The committed CubeMX output IS the firmware source — do not let the upstream utils `.gitignore` (which ignores `Core`, `Drivers`, `Makefile`…) leak in. Create `firmware_stm32/.gitignore`:
```
build/
micro_ros_stm32cubemx_utils/microros_static_library/libmicroros/
```
(Everything else under `firmware_stm32/` — `Core/`, `Drivers/`, `Middlewares/`, `Makefile`, `*.ioc`, `startup_*.s`, `*.ld` — stays tracked.)

- [ ] **Step 2: Verify the generated tree is actually tracked**
```bash
git -C firmware_stm32 ls-files | grep -E 'Core/Src/main.c|Makefile|\.ioc$' | head
```
Expected: lists `Makefile`, the `.ioc`, and `Core/Src/main.c`. If empty, the tree was wrongly ignored — fix before committing.

- [ ] **Step 3: Create the dedicated CI workflow**
This is SEPARATE from `.github/parse_platformio.py` (which only enumerates `platformio.ini` envs and cannot see a CubeMX/Make project). Create `.github/workflows/stm32-f446re-f0.yml`:
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
          submodules: recursive      # pulls micro_ros_stm32cubemx_utils @ pinned commit
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
          test -f build/*.elf
          arm-none-eabi-size build/*.elf
```

- [ ] **Step 4: Commit**
```bash
git add firmware_stm32/.gitignore .github/workflows/stm32-f446re-f0.yml
git commit -m "ci(stm32): F0 link-smoke workflow + project gitignore

Separate from parse_platformio.py (CubeMX/Make project, no pio env).

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## On completion (milestone tag)

After all tasks pass and the branch is merged (per superpowers:finishing-a-development-branch), create the milestone tag (project rule — always tag significant milestones):
```bash
git tag -a stm32cube-p2-skeleton-libmicroros -m "STM32Cube port — Plan 2 (F0): F446RE CubeMX skeleton links libmicroros cleanly"
```

---

## Self-Review (against the spec)

- **Spec coverage (Ф0 / §8):** "CubeMX project compiles + `libmicroros.a` links (ABI-smoke) without `__sync_synchronize`" — Tasks 2–6 deliver exactly this; Task 6 Step 4 is the acceptance gate, and the FAIL signatures map 1:1 to spec §9 risks #1 (float-ABI) and the atomics trap in [[stm32cube-microros-gotchas]]. CI (spec §7, "separate workflow, not `parse_platformio.py`") = Task 7. FreeRTOS + ≥24 KB task (spec §0, §5.5, risk #3) = Task 2 Step 7 / Task 5 Step 4. UART transport choice (spec §5.4) = `it_transport.c` for F0, with `dma_transport.c` noted for Plan 4.
- **Placeholder scan:** No TBD/TODO. GUI-only steps are marked `[GUI]` with exact settings; values that depend on CubeMX output (FPU flags, stack size) come with an exact expected value + a `grep`/assert to verify — verification, not placeholder.
- **Consistency:** the pinned utils commit (`a5b2127…`) and image digest (`sha256:1482f3df…`) are identical across Task 1 PINS, Task 3 submodule, Task 6 Docker, and Task 7 CI. `huart2`/USART2/PA2-PA3 is consistent across Tasks 2, 5. The Makefile addon paths match the submodule path from Task 3.
- **Out of scope (correctly deferred):** boot/run verification → Plan 3 (Renode); working transport + agent round-trip → Plan 4; encoder/PWM/IMU → Plans 5–6. F0 is link-only by design.
- **Known limitations (cannot be verified in the authoring sandbox):** the literal generated `Makefile` FPU lines, that the link actually succeeds, and the CubeMX-CLI specifics — all require CubeMX + `arm-none-eabi-gcc` + Docker on the executor's machine. Each such point carries an in-step `grep`/assert so the executor verifies it locally.
