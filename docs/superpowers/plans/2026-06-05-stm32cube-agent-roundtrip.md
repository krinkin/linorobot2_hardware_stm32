# STM32Cube Port — Plan 4: Live micro-ROS Agent Round-Trip (Phase Ф3)

> **For agentic workers:** REQUIRED SUB-SKILL: superpowers:executing-plans. This plan is recorded **as-built** — Ф3 was closed empirically on 2026-06-05. Steps are kept as `- [x]` to document what was proven and how to reproduce it.
>
> **Prerequisite:** Plan 2 (firmware ELF + libmicroros + atomics) and Plan 3 (Ф2 boot smoke) done. Needs `renode`, `docker`, `socat`.

**Goal:** Prove a **real `micro_ros_agent` establishes a live XRCE-DDS session with the firmware** — in pure emulation, no board, no physical serial port. The firmware runs the canonical wait-for-agent loop, then creates a node; the agent must answer the ping, establish the session, and create the participant.

**Result (2026-06-05 — not speculative):** PASS. Bridged a real `micro_ros_agent` (Docker, jazzy) to the Renode firmware's USART2. Agent log:

```
create_client       | create              | client_key: 0x5792F608, session_id: 0x81
establish_session   | session established  | client_key: 0x5792F608
recv_message [==>> SER <<==] ... 73 74 6D 33 32 5F 6E 6F 64 65   # "stm32_node" on the wire
create_participant  | participant created | client_key: 0x5792F608, participant_id: 0x000(1)
```

**Architecture (the bridge):**

```
firmware USART2  ──Renode raw socket (telnet OFF)──▶  socat ──pty──▶  micro_ros_agent
   (in Renode)        CreateServerSocketTerminal P false              (Docker, --network host)
```

Renode exposes USART2 on a **raw** TCP socket terminal; one `--network host` Docker container runs BOTH `socat` (TCP↔pty) and the agent, so the pty is local to that container's devpts (a container cannot open a *host* pty across the devpts namespace). The container reaches Renode over loopback.

---

## Where this plan sits

Plan 4 of 7. Plans 1–3 precede it (host tests; skeleton+libmicroros+atomics; Ф2 boot smoke). Plan 4 = **Ф3 (live round-trip)**. It does NOT add the DMA/IT transport variants, encoder/PWM (Plan 5), or IMU (Plan 6) — the round-trip uses the simple blocking HAL-UART transport, which is sufficient to prove the session. Neither emulator is baud/timing-accurate, so wire-timing fidelity stays a hardware-only item.

---

## Two bugs were the blockers (root-caused, both fixed)

### Bug 1 — FreeRTOS task priority ≥ `configMAX_PRIORITIES` → silent startup hang
`xTaskCreate(uros_task, "uros", 6144, NULL, 24, NULL)` with `configMAX_PRIORITIES 7`. The `24` is **CMSIS-RTOS-v2 `osPriorityNormal`** (which needs `configMAX_PRIORITIES 56`), but this firmware is **raw FreeRTOS**. So `configASSERT(uxPriority < configMAX_PRIORITIES)` fired, and with the usual `configASSERT(){ taskDISABLE_INTERRUPTS(); for(;;); }` the CPU **spun forever inside `xTaskCreate`, before `vTaskStartScheduler`** — `uros_task` never ran, USART2 was dead-silent. Renode `cpu LogFunctionNames` + the disassembly pinned it to a `b.n self` at `xTaskCreate+0x76`.

**Fix:** `xTaskCreate(uros_task, "uros", 6144, NULL, configMAX_PRIORITIES - 2, NULL)` (tie to the config so it can never exceed the bound).

**Boot-smoke false-PASS gap (also fixed):** a configASSERT spin is **not** `HardFault_Handler`, so the old "PC != HardFault" smoke PASSED the hang. The Ф2 smoke now proves the firmware is genuinely alive (below).

### Bug 2 — Renode socket terminal defaults to TELNET → mangles binary XRCE
`emulation CreateServerSocketTerminal <port> "name"` is a **telnet server**: on connect it injects IAC negotiation (`ff fd 00 / ff fb 01 / ff fb 03 / ff fc 22`) and escapes `0xFF`, corrupting the binary micro-XRCE stream both directions → the agent never sees a valid session.

**Fix:** pass the 3rd arg `false` (`emitConfigBytes=false`) for a **raw** socket: `emulation CreateServerSocketTerminal <port> "name" false`.

### Environment wall (worked around, not a firmware issue)
The agent exists only inside a Docker image, and a container **cannot open a host pty** across the devpts namespace (permission denied / ENOENT / EIO). Resolved by running socat + agent **inside one `--network host` container** (pty local to it). The image ENTRYPOINT is `micro_ros_agent` itself, so to also launch socat: `--entrypoint bash` + hand-source ROS (`source /opt/ros/jazzy/setup.bash; source /uros_ws/install/local_setup.bash`).

---

## File Structure

| Path | Responsibility | Status |
|---|---|---|
| `firmware_stm32/Src/main.c` | wait-for-agent loop (`rmw_uros_ping_agent`) + valid task priority | Modified |
| `firmware_stm32/renode/agent_bridge.sh` | Ф3 round-trip harness (Renode raw socket + socat + Docker agent) | Author |
| `firmware_stm32/renode/boot_smoke.sh` | Ф2 upgraded: assert USART2 emits the ping (catches the configASSERT class) | Modified |
| `firmware_stm32/renode/boot_smoke.robot` | Ф2 (CI): assert scheduler ticked (`xTickCount>0`, `uxCurrentNumberOfTasks!=0`) | Modified |
| `Makefile` | `make agent-roundtrip` target | Modified |

---

## Tasks (as-built)

### Task 1 — Wait-for-agent loop + valid priority  ✅
- [x] `main.c`: `rmw_uros_set_custom_transport(...)` then `while (rmw_uros_ping_agent(200,1) != RMW_RET_OK) vTaskDelay(200ms);` before `rclc_support_init` + `rclc_node_init_default("stm32_node", ...)`.
- [x] Task priority `configMAX_PRIORITIES - 2` (Bug 1 fix).

### Task 2 — Ф3 bridge harness  ✅
- [x] `agent_bridge.sh`: Renode `CreateServerSocketTerminal P false` + `connector Connect sysbus.usart2`; socat+agent in one `--network host` container; PASS on `session established` + `participant created`. Auto-builds a socat-augmented agent image on first run.

### Task 3 — Strengthen the Ф2 smoke so it can't false-PASS this class  ✅
- [x] `boot_smoke.sh`: read USART2 over a raw socket; PASS iff bytes were emitted (the ping) — proves scheduler + `uros_task` + transport. On fail, diagnose HardFault vs. a startup spin by resolving the stuck PC.
- [x] `boot_smoke.robot`: `emulation RunFor` (NOT `Start Emulation` + `RunFor` — that errors "already started"), then assert `xTickCount != 0` and `uxCurrentNumberOfTasks != 0` via `GetSymbolAddress`/`ReadDoubleWord`.

### Task 4 — Wire it up  ✅
- [x] `make agent-roundtrip` runs the bridge. `make renode` runs the upgraded Ф2.

---

## Reproduce

```bash
source /opt/ros/jazzy/setup.bash
make build-fw          # ELF with the wait-for-agent loop + valid priority
make agent-roundtrip   # Renode + socat + Docker agent; expect "Ф3 PASS"
make renode            # upgraded Ф2: expect USART2 emits the ping
```

## Carry-forward
- CI runs Ф2 (`boot_smoke.robot` via renode-test-action). Ф3 needs Docker + socat on the runner — left as an opt-in/local gate for now (heavier: builds the agent image, runs Renode + a container).
- The blocking HAL-UART transport is fine for the round-trip; switching to the utils' IT/DMA transport is a performance item for the full control loop (Plan 7), not a correctness blocker.
- Next: Plan 5 (encoder via TIM + PWM motor), Plan 6 (I2Cdev→HAL + IMU), Plan 7 (full loop + CI).
