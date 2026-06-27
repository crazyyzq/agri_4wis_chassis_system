# ECU Main Control Framework Reading Guide

This document explains how to read the main control framework from `main()` outward. It is intentionally code-oriented: use it beside the source tree while reviewing each file.

## Top-level layout

```text
ecu/
  apps/
    agri_chassis_control_cpu0/   CPU0 FreeRTOS application entry point.
    agri_chassis_control_cpu1/   CPU1 FreeRTOS application entry point.
  common/                        Shared primitive types and time aliases.
  config/                        Central timing, threshold and priority constants.
  diag/                          Diagnostic code definitions and text.
  protocol/sbus/                 Pure SBUS frame decoding.
  drivers/sbus/                  SBUS UART-facing service state.
  remote/                        Remote-control interpretation FSMs.
  vehicle/                       Command arbitration, safety and execution boundary.
  control/                       Motion and chassis adjustment planning.
  ipc/                           CPU0/CPU1 snapshot contracts.
  os/                            FreeRTOS task orchestration helpers.
```

## Start reading here: CPU0 main

File: `ecu/apps/agri_chassis_control_cpu0/src/main_cpu0.c`

CPU0 is the safety-owning side of the system. It performs board initialization, creates the core FreeRTOS tasks in `main_cpu0.c`, then starts the scheduler.

The rule for CPU0 is simple:

- It may arbitrate commands.
- It may apply safety clamps.
- It may issue actuator commands through the executor boundary.
- It should not hide business logic inside `main()`.

After `main_cpu0.c`, read `ecu/os/src/ecu_tasks_cpu0.c`.

## Task orchestration

Files:

- `ecu/os/include/ecu_tasks.h`
- `ecu/os/src/ecu_tasks_cpu0.c`
- `ecu/os/src/ecu_tasks_cpu1.c`

This layer is the scheduler-facing boundary. The task code wires modules together in deterministic periodic loops:

- Remote input is sampled and converted to a structured request.
- Available command sources are rebuilt every cycle.
- The arbiter selects exactly one active command.
- The safety manager clamps unsafe outputs.
- The executor publishes the final actuator command boundary.
- CPU snapshots are refreshed for debug and inter-core exchange.

Keep this file thin. If logic grows here, it should usually be moved into a named module under `remote/`, `vehicle/`, `control/` or `ipc/`.

CPU0 and CPU1 are intentionally split into separate source files. CPU1 must not include command arbitration, safety manager or executor headers.

## Shared definitions and configuration

Files:

- `ecu/common/include/ecu_types.h`
- `ecu/common/include/ecu_time.h`
- `ecu/config/include/ecu_config.h`
- `ecu/config/src/ecu_config.c`
- `ecu/diag/include/diag_codes.h`
- `ecu/diag/src/diag_codes.c`

These files define project-wide language:

- `ecu_status_t` is the common success/failure result.
- `ecu_millis_t` is the monotonic millisecond timestamp type.
- `ecu_config.h` owns constants such as debounce times, SBUS thresholds, failsafe timeouts and arbitration priorities.
- `diag_codes.h` owns fault and warning identities.

The important rule is that thresholds and timings belong in `config`, not scattered through individual algorithms.

## SBUS input path

Files:

- `ecu/protocol/sbus/include/sbus_decoder.h`
- `ecu/protocol/sbus/src/sbus_decoder.c`
- `ecu/drivers/sbus/include/sbus_service.h`
- `ecu/drivers/sbus/src/sbus_service.c`

The decoder is pure protocol code. It accepts one 25-byte SBUS frame and returns channel values plus frame flags.

The service is the UART-facing state holder. The intended hardware path is:

1. UART receives bytes.
2. UART idle interrupt passes the received block into `sbus_service_feed_bytes()`.
3. The service accepts valid 25-byte frames and updates its snapshot.
4. Other modules read `sbus_service_get_snapshot()`.

This separation keeps interrupt-facing code and protocol decoding testable.

## Remote-control interpretation

Files:

- `ecu/remote/include/remote_types.h`
- `ecu/remote/src/remote_discrete.c`
- `ecu/remote/src/remote_link_fsm.c`
- `ecu/remote/src/remote_arm_fsm.c`
- `ecu/remote/src/remote_estop_fsm.c`
- `ecu/remote/src/remote_gear_fsm.c`
- `ecu/remote/src/remote_mode_fsm.c`
- `ecu/remote/src/remote_adjust_fsm.c`
- `ecu/remote/src/remote_power_fsm.c`
- `ecu/remote/src/remote_authority_fsm.c`
- `ecu/remote/src/remote_event_lifecycle.c`
- `ecu/remote/src/remote_lights_fsm.c`
- `ecu/remote/src/remote_manager.c`

Read `remote_types.h` first. It defines the vocabulary for remote input, remote states and the final `remote_request_t`.

Then read each FSM:

- Link FSM: qualifies SBUS connection and enters failsafe on timeout.
- Arm FSM: only enters ready state through a neutral qualification path.
- E-stop FSM: latches emergency stop source and requires reset plus normal input before clear.
- Gear FSM: tracks requested gear and active gear separately.
- Mode FSM: prevents rapid domain changes through a guard window.
- Adjust FSM: tracks chassis adjustment ownership and commands.
- Power FSM: turns CH4 long-hold actions into high-voltage or orderly shutdown requests.
- Authority FSM: controls manual/automatic authority from CH7 and safety preconditions.
- Event lifecycle: gives operator actions an explicit expiry window so rejected requests are not queued.
- Lights FSM: converts remote light switches into indicator requests.

Finally read `remote_manager.c`. It coordinates those FSMs and rebuilds a complete remote request every update cycle. This is the boundary that the vehicle layer consumes.

CH2 and CH14 are intentionally separate:

- CH2 maps to clearance adjustment.
- CH14 maps to track-width adjustment.
- If both request adjustment at the same time before an owner exists, the request is rejected instead of guessed.

## Vehicle command path

Files:

- `ecu/vehicle/include/vehicle_types.h`
- `ecu/vehicle/src/command_arbiter.c`
- `ecu/vehicle/src/safety_manager.c`
- `ecu/vehicle/src/vehicle_command_executor.c`
- `ecu/vehicle/src/vehicle_state.c`

Read order:

1. `vehicle_types.h`
2. `command_arbiter.c`
3. `safety_manager.c`
4. `vehicle_command_executor.c`
5. `vehicle_state.c`

The arbiter rebuilds command candidates each cycle and selects the highest-priority valid source. The safety manager then clamps the selected command based on remote state, diagnostic severity and platform state. The executor is intentionally a boundary: real CANopen, hydraulic, relay and warning-light drivers attach behind it later.

The current safety rule is conservative:

- Emergency stop latching forces zero speed, brake release disabled, hydraulic disabled and high voltage disabled.
- A-class faults force the same safe command.
- Missing command sources fall back to a safe default.

## Control planning

Files:

- `ecu/control/include/motion_control.h`
- `ecu/control/src/motion_control.c`
- `ecu/control/include/adjust_control.h`
- `ecu/control/src/adjust_control.c`

These modules convert high-level requests into normalized command structures. They should not talk to hardware and should not know SBUS channel numbers.

## CPU0/CPU1 data exchange

Files:

- `ecu/ipc/include/ipc_snapshot.h`
- `ecu/ipc/src/ipc_snapshot.c`
- `ecu/apps/agri_chassis_control_cpu1/src/main_cpu1.c`

CPU1 is the sensing/data aggregation side. It publishes sensor and health snapshots. CPU0 consumes snapshots and owns safety/output decisions.

Do not put actuator commands on CPU1. That is an explicit architecture rule and is guarded by static checks.

## Test and static-check files

Files:

- `tests/python/run_tests.py`
- `tests/python/test_remote_fsm.py`
- `tests/python/test_vehicle_arbiter.py`
- `tools/check_no_forbidden_patterns.py`

These tests are not full embedded integration tests. They are framework-contract checks intended to catch bad architecture early:

- Missing FSM modules.
- Unsafe CPU1 dependencies.
- Scattered magic thresholds.
- Remote logic depending directly on board drivers.
- Generic catch-all files that turn into unmaintainable code.

## How to review changes safely

For each future feature, use this path:

1. Add or update the relevant data structure in `remote_types.h`, `vehicle_types.h` or the specific module header.
2. Put thresholds and timings in `ecu_config.h`.
3. Implement pure logic in the smallest named module.
4. Keep hardware access behind driver/executor boundaries.
5. Add a test or static rule if the behavior is important enough to protect.
6. Run the Python tests and syntax checks before pushing.
