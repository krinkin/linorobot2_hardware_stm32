# STM32Cube Port — Plan 7: Full Base Node — Topics + Reconnect + CI (Phase Ф6)

> **As-built record.** Designed via a 3-lens workflow panel, implemented, hardened by a
> 4-dimension adversarial review (9 confirmed findings). All tiers green. This completes the
> emulation-provable port (Ф0–Ф6); Ф7 is on-hardware bring-up.
>
> **Prerequisite:** Plans 1–6.

**Goal:** wire the micro-ROS **base-node topic interface** so the firmware is a drop-in linorobot2
low-level controller: subscribe `/cmd_vel`, publish `/odom/unfiltered` + `/imu/data_raw`, with a robust
agent reconnect lifecycle — all proven over a live agent in pure emulation.

**Result (2026-06-05):** Ф6 PASS — all 9 checks: XRCE session + participant; `/cmd_vel` sub +
`/odom/unfiltered` + `/imu/data_raw` pubs present; `/imu/mag` absent (FakeMAG); odom `frame_id: odom` /
`child_frame_id: base_footprint`; imu has angular_velocity + linear_acceleration; `/cmd_vel` published.
All prior tiers stay green (host doctest, F0 link 108.9 KB no-libstdc++, Ф2/Ф4/Ф5).

## Architecture — two-task split (locked)
- **control_task** (pri 4): the 50 Hz encoder→PID→motor→odom→IMU loop, **independent of the agent**, with
  the 200 ms deadman. Runs forever, deadman-safe even when comms are down.
- **uros_task** (pri 5): owns the rclc executor; comms only. A 4-state reconnect machine
  (`WAITING_AGENT → AGENT_AVAILABLE → AGENT_CONNECTED → AGENT_DISCONNECTED`, ported from firmware.ino)
  with `createEntities()`/`destroyEntities()` (full destroy+recreate on agent loss, frame Strings set-once
  to avoid heap leaks). A 50 Hz **publish-only** rcl timer assembles odom+imu from the snapshot surface.
- **Cross-task** data crosses ONLY the plain-C snapshot surface under `taskENTER/EXIT_CRITICAL`:
  `control_set_cmd` (← `cmd_vel` callback), `control_get_odom`, `control_get_imu`. uros_task never touches
  I2C/HAL/the C++ objects.

## Topics
- sub `geometry_msgs/Twist` on `cmd_vel` → `control_set_cmd(lin.x, lin.y, ang.z)`
- pub `nav_msgs/Odometry` on `odom/unfiltered` (frame `odom` / child `base_footprint`, yaw→quat, twist, cov)
- pub `sensor_msgs/Imu` on `imu/data_raw` (frame `imu_link`; orientation marked absent per REP-145)
- `/imu/mag` skipped (FakeMAG). Node name kept `stm32_node` (preserves the Ф3 harness).

## Adversarial review — 9 findings fixed
- **HIGH:** header stamps were board *uptime* (the `rmw_uros_sync_session` offset was discarded) → consume
  `rmw_uros_epoch_nanos()` into a session-scoped offset folded into every stamp (wall-clock time).
- **MED:** the Ф6 `cmd_vel` check only proved the *agent* published → relabeled honestly (firmware delivery
  is implied: the odom/imu pubs prove the executor spins, and the `/cmd_vel` sub exists; the inbound math is
  host-tested by ControlCore).
- **LOW:** odom could publish a torn pose/velocity tuple → pose+velocity captured as one coherent
  writer-side snapshot under a critical section. All-zero orientation quaternion → identity quat +
  `orientation_covariance[0] = -1` (REP-145). Orphaned `g_dbg_stack_overflow` → read + asserted in
  control_smoke. CI soft-gate TODO. Ф6 test robustness: fixed `sleep 16` → discovery poll loop; dead
  `grep -qv` removed; `frame_id` grep anchored.

## CI (.github/workflows/stm32-f446re.yml)
- `stm32-f0-f2` (always-on): host tests already; + libmicroros + build-fw + Ф2 (robot) + **Ф4 `make control`
  + Ф5 `make imu`** (Renode portable + socat installed).
- `stm32-topics` (new, push-gated, `continue-on-error` until proven stable, with a removal TODO): libmicroros
  + build-fw + **Ф6 `make topics`** (Docker micro_ros_agent + `--network host`).

## Deferred (hardware / out-of-scope)
- Real ROS-time path could use `clock_settime` instead of an offset (functionally equivalent here).
- I2C SCL bus-recovery bit-bang (Plan 6), real MAG + MPU9250, the cmd→PWM *value* assertion over the agent
  (needs host-side Renode symbol readback during the docker run). Ф7 = on-hardware NUCLEO-F446RE bring-up.

## Reproduce
```bash
make test-all        # Tier A + libmicroros + build-fw + Ф2 + Ф4 + Ф5
make topics          # Ф6 full base-node topic round-trip (docker agent)
```
