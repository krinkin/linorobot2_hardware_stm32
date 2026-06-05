# STM32Cube Port — Plan 3: Renode Boot Smoke (Phase Ф2)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans. Steps use checkbox (`- [ ]`) syntax.
>
> **Prerequisite:** Plan 2 must produce a real firmware ELF (`firmware_stm32/build/*.elf`). Tasks 1–2 of THIS plan (Renode harness + a boot smoke of *any* F4-class ELF) are runnable now; Tasks 3+ need Plan 2's ELF.

**Goal:** Prove the native firmware **boots in emulation** (no hardware): vector table + `Reset_Handler` + clock init run, the FreeRTOS scheduler starts, and `rclc_support_init` executes **without HardFault** — the documented #1 runtime risk (spec §9 risk #3). Headless, CI-gateable.

**Architecture:** Renode **v1.16.1** (pinned) running the firmware ELF on a generic-F4 `.repl`, driven headless by a `.resc` + Robot Framework test (`antmicro/renode-test-action`). No micro-ROS agent is needed for Ф2 (boot + init only); the agent round-trip is Plan 4.

**Tech Stack:** Renode 1.16.1 (portable, mono-based), Robot Framework, the Plan 2 firmware ELF.

**Validated on 2026-06-04 (this is not speculative):** Renode 1.16.1 portable installed; the bundled `platforms/cpus/stm32f4.repl` (Cortex-M4, `flash@0x08000000`, `sram@0x20000000`) booted a real F446-class micro-ROS-linked ELF cleanly — `emulation RunFor "0.1"` → 20 288 instructions, `PC` settled inside `main` (NOT the fault handler at `Default_Handler`/`HardFault_Handler`), `IsHalted=False`. The headless `.resc` pattern below is the one that worked.

**Update (2026-06-05 — Ф2 on the REAL combined GUI-free firmware):** the full hand-written firmware (CMSIS startup + hand linker + HAL `HAL_Init`/USART2 + FreeRTOS + micro-ROS `rclc_support_init`/node + HAL-UART transport + atomic shim + glue) was booted in `stm32f4.repl` (USART2 is modeled, `UART.STM32_UART @ 0x40004400`): **71 469 instructions, FreeRTOS scheduler running, `uros_task` executed `rclc_support_init`, PC in the idle task — NOT in `HardFault_Handler` → the Ф2 no-fault gate PASSED.** **Caveat (now Task 3):** Renode does **not** model the DWT cycle counter (`0xE0001000/4` = non-existing peripheral), so a `HAL_GetTick`-via-DWT timebase is frozen there and UART timeouts won't fire — back the HAL timebase with the **FreeRTOS SysTick/tick** (or a TIM), not DWT, before the Plan-4 agent round-trip.

---

## Where this plan sits

Plan 3 of 7. Plans 1 (host tests) and 2 (skeleton + libmicroros ABI/atomics — empirically closed, tags `stm32cube-f0-abi-validated` / `stm32cube-f0-link-proven`) precede it. Plan 3 covers **Ф2 (boot smoke)**. It does NOT do the agent round-trip / DMA transport (Plan 4), or encoder/PWM/IMU (Plans 5–6). Neither emulator is timing/baud-accurate, so real-time jitter stays a hardware-only item (spec §10).

**Key caveat (discovered empirically):** `clock_gettime` MUST advance. The utils' `microros_time.c` backs it with the FreeRTOS tick — good. But a *frozen* clock (e.g. a `return 0` stub) makes `uxr_create_session` spin forever (its timeout deadline is never reached). For the Ф2 boot smoke we do not connect an agent, so bound `rclc_support_init` (it will retry session-create and **return an error**, not HardFault, once time advances) — that error is a PASS for Ф2 (no fault), and the agent success is Plan 4.

---

## File Structure

| Path | Responsibility | Origin |
|---|---|---|
| `firmware_stm32/renode/f446.repl` | Renode platform (extends bundled `stm32f4.repl`) | Author (Task 1) |
| `firmware_stm32/renode/boot_smoke.resc` | Headless boot script (validated pattern) | Author (Task 1) |
| `firmware_stm32/renode/boot_smoke.robot` | Robot Framework assertions | Author (Task 2) |
| `.github/workflows/stm32-f446re.yml` | Extend with a Renode boot-smoke job | Modify (Task 4) |

---

### Task 1: Renode harness — pinned Renode + platform + boot `.resc`

**Files:**
- Create: `firmware_stm32/renode/f446.repl`
- Create: `firmware_stm32/renode/boot_smoke.resc`

- [ ] **Step 1: Pin Renode** — use `antmicro/renode-test-action@v4` with `renode-version: 1.16.1` in CI (Task 4); for local runs use the portable build `renode_1.16.1` (mono-based; needs system `mono`).

- [ ] **Step 2: Platform `.repl`** — start from the bundled generic F4 (it already maps `flash@0x08000000`, `sram@0x20000000`, NVIC, EXTI, RCC, PWR). Create `firmware_stm32/renode/f446.repl`:
```
// F446RE boot-smoke platform: reuse Renode's generic STM32F4 description.
// (Bundled stm32f4.repl is sufficient for boot; add usart2/tim/i2c in Plans 4-6.)
using "platforms/cpus/stm32f4.repl"

// USART2 (Nucleo VCP, PA2/PA3) — lets the firmware print an "alive" line for the
// Robot assertion. Address per RM0390 (USART2 @ 0x40004400).
usart2: UART.STM32F7_USART @ sysbus 0x40004400
    -> nvic@38
```
> If `UART.STM32F7_USART` doesn't bind cleanly on this base, fall back to the bundled `stm32f4_discovery`'s UART model; the boot smoke does not strictly require UART (PC-vs-fault-handler is the primary gate).

- [ ] **Step 3: Boot `.resc`** (this exact pattern booted a real ELF on 2026-06-04). Create `firmware_stm32/renode/boot_smoke.resc`:
```
:name: F446RE micro-ROS boot smoke
mach create "f446"
machine LoadPlatformDescription @firmware_stm32/renode/f446.repl
$elf ?= @firmware_stm32/build/firmware_stm32.elf
sysbus LoadELF $elf
# fault handlers (Default_Handler/HardFault_Handler) trap into an infinite loop;
# a boot that reaches main never enters them.
showAnalyzer sysbus.usart2
start
```

- [ ] **Step 4: Local sanity (optional, needs an ELF)** — boot any F4-class ELF headless:
```bash
cd /path/to/renode_1.16.1
./renode --console --disable-xwt -e \
  'mach create "f4"; machine LoadPlatformDescription @platforms/cpus/stm32f4.repl; sysbus LoadELF @<ELF>; emulation RunFor "0.1"; cpu PC; cpu ExecutedInstructions; cpu IsHalted; quit'
```
Expected: `ExecutedInstructions` > 0, `IsHalted False`, and `PC` NOT equal to the firmware's `Default_Handler`/`HardFault_Handler` address (from `arm-none-eabi-nm <ELF> | grep -iE 'HardFault_Handler|Default_Handler|main'`).

- [ ] **Step 5: Commit**
```bash
git add firmware_stm32/renode/f446.repl firmware_stm32/renode/boot_smoke.resc
git commit -m "test(stm32): Renode boot-smoke harness (platform + resc)

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 2: Robot Framework boot-smoke assertions

**Files:**
- Create: `firmware_stm32/renode/boot_smoke.robot`

- [ ] **Step 1: Write the test** — gates on (a) the CPU executed instructions and is not halted, (b) PC is not stuck in the fault handler, and (c) optionally a UART "alive" line if the firmware prints one. Create `firmware_stm32/renode/boot_smoke.robot`:
```robotframework
*** Settings ***
Suite Setup       Setup
Suite Teardown    Teardown
Resource          ${RENODEKEYWORDS}

*** Variables ***
${ELF}            ${CURDIR}/../build/firmware_stm32.elf
${FAULT_ADDR}     0x080038b8    # arm-none-eabi-nm <elf> | grep HardFault_Handler — UPDATE per build

*** Test Cases ***
Firmware Boots Without Faulting
    Execute Command           mach create "f446"
    Execute Command           machine LoadPlatformDescription @${CURDIR}/f446.repl
    Execute Command           sysbus LoadELF @${ELF}
    Create Terminal Tester    sysbus.usart2    timeout=5
    Start Emulation
    # run a slice of virtual time, then assert liveness + no fault
    Execute Command           emulation RunFor "0.20"
    ${pc}=                    Execute Command    cpu PC
    Should Not Contain        ${pc}              ${FAULT_ADDR}
    ${halted}=                Execute Command    cpu IsHalted
    Should Contain            ${halted}          False
    ${instr}=                 Execute Command    cpu ExecutedInstructions
    Should Not Contain        ${instr}           0x0000000000000000
```
> Update `${FAULT_ADDR}` from the actual build (`arm-none-eabi-nm build/firmware_stm32.elf | grep -i HardFault_Handler`). If the firmware prints a boot banner on USART2, add `Wait For Line On Uart  <regex>` for a stronger assertion.

- [ ] **Step 2: Run it** (needs Plan 2's ELF + Renode):
```bash
renode-test firmware_stm32/renode/boot_smoke.robot
```
Expected: `1 test, 1 passed`.

- [ ] **Step 3: Commit**
```bash
git add firmware_stm32/renode/boot_smoke.robot
git commit -m "test(stm32): Renode Robot boot-smoke assertions (no-fault gate)

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 3: Runtime `rclc_support_init` no-HardFault gate (the real Ф2)

Needs Plan 2's firmware where `StartDefaultTask` actually calls the init sequence (not just references it). Validates spec §9 risk #3.

- [ ] **Step 1: Ensure the clock advances** — confirm the firmware uses the utils' `microros_time.c` `clock_gettime` (FreeRTOS-tick backed) and that the FreeRTOS tick (SysTick) is modeled/running in Renode. Without an advancing clock, `uxr_create_session` spins forever (frozen-time deadlock) — this is NOT a HardFault but will hang the smoke; bound it with `RunFor`.

- [ ] **Step 2: Boot + run the executor for a bounded slice**, agent absent:
```bash
renode --console --disable-xwt -e \
  'i @firmware_stm32/renode/boot_smoke.resc; emulation RunFor "2.0"; cpu PC; cpu IsHalted; quit'
```
Expected (PASS): `rclc_support_init` runs, retries session-create against the (absent) agent, and **returns an error** without entering the fault handler; `IsHalted=False`, `PC` not in `HardFault_Handler`. (Agent *success* is Plan 4.)
- FAIL modes to diagnose: PC stuck at `HardFault_Handler` → stack/heap too small (raise FreeRTOS task to ≥24 KB / heap) or an unmodeled-peripheral access; emulation hangs with frozen virtual time → the clock isn't advancing (Step 1).

- [ ] **Step 3: Record** the PC/halt/instruction evidence in the task report (no commit — runtime observation).

---

### Task 4: CI — Renode boot-smoke job

**Files:**
- Modify: `.github/workflows/stm32-f446re.yml`

- [ ] **Step 1: Append a boot-smoke job** after the F0 link job (it consumes the linked ELF):
```yaml
  renode-boot-smoke:
    needs: f0-link-smoke
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
        with: { submodules: recursive }
      # (rebuild or download the ELF artifact from the f0-link-smoke job)
      - name: Renode boot smoke
        uses: antmicro/renode-test-action@v4
        with:
          renode-version: '1.16.1'
          tests: firmware_stm32/renode/boot_smoke.robot
```
> Pin `renode-version: 1.16.1` (the version validated here). Share the ELF from the F0 job via `actions/upload-artifact`/`download-artifact` rather than rebuilding.

- [ ] **Step 2: Commit**
```bash
git add .github/workflows/stm32-f446re.yml
git commit -m "ci(stm32): add Renode boot-smoke job (pinned renode 1.16.1)

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## On completion (milestone tag)

After the boot smoke is green on the real Plan 2 firmware and merged:
```bash
git tag -a stm32cube-p3-renode-boot -m "STM32Cube port — Plan 3 (Ф2): firmware boots in Renode, rclc_support_init no HardFault"
```

---

## Self-Review (against the spec)

- **Spec coverage (Ф2 / §9 risk #3):** Tasks 1–3 deliver "boots in Renode, FreeRTOS starts, `rclc_support_init` no HardFault"; Task 4 = CI. The boot-smoke *mechanism* (Tasks 1–2) is empirically validated here; Task 3 needs Plan 2's runtime firmware.
- **Placeholder scan:** the `.resc`/`.robot` are real and runnable; `${FAULT_ADDR}` is explicitly "update from `nm`" (a verification step, not a placeholder); the `.repl` UART line has a documented fallback.
- **Known limitations:** no F446-specific bundled `.repl` (generic `stm32f4.repl` used; peripheral deltas matter only when Plans 4–6 add USART-DMA/TIM/I2C); Renode is not timing-accurate (jitter/baud → hardware, spec §10); the frozen-clock deadlock caveat is called out in Task 3.
