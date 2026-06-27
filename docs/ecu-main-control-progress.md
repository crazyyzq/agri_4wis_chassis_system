# ECU Main Control Framework Progress

Last updated: 2026-06-27

This file records the durable implementation state for the main-branch control framework so the project can be resumed after context compression or a new development session.

## Baseline

- Hardware bring-up and board-level tests for RS485, RS232, CAN1-CAN4, SBUS, digital I/O and analog input have been completed before this framework work.
- Ethernet and CAN-FD are intentionally out of scope for the current software framework.
- The main project target is the HPM6750 ECU based on the MR6750 module and the HPM SDK 1.11 environment.
- The main framework is being written on `main`, not on the previous board-test branch.

## Requirement source

- Primary requirement document: `doc/ECU_Project_Implementation_v1.4.md`
- Current framework plan: `docs/superpowers/plans/2026-06-27-ecu-main-control-framework.md`

## Implemented framework scope

- Shared project primitives:
  - ECU status/result types.
  - Stable millisecond timestamp type.
  - Central configuration constants for timing, SBUS thresholds, failsafe limits and arbitration priorities.
  - Diagnostic code catalog with readable text.
- SBUS layer:
  - Pure SBUS frame decoder.
  - SBUS service state that can be fed from a UART idle-interrupt path.
  - Snapshot API for the rest of the project.
- Remote-control layer:
  - Link qualification and failsafe FSM.
  - Arm/neutral FSM.
  - Emergency-stop latch/reset FSM.
  - Gear FSM.
  - Mode/domain FSM with guard time.
  - Adjustment FSM with owner arbitration.
  - Lights/indicator FSM.
  - Remote manager that rebuilds a full remote request each cycle.
- Vehicle-control layer:
  - Command source arbitration.
  - Safety clamping.
  - Command execution boundary.
  - Vehicle runtime state snapshot.
- Control layer:
  - Motion command planner.
  - Adjustment command planner.
- IPC layer:
  - CPU0 to CPU1 snapshot contract.
  - CPU1 to CPU0 snapshot contract.
- FreeRTOS app skeleton:
  - CPU0 application owns command arbitration, safety and actuator execution.
  - CPU1 application owns sensing/data aggregation snapshots only.
  - CPU0 and CPU1 task implementations are split so CPU1 does not include safety-critical executor code.
- HPM SDK build integration:
  - CPU0 CMake/SES project generation passed.
  - CPU1 CMake/SES project generation passed.
  - CPU0 Ninja build passed and produced `demo.elf`.
  - CPU1 Ninja build passed and produced `demo.elf`.

## Design constraints already encoded in tests

- Remote modules must not include board/HPM peripheral headers directly.
- Raw SBUS thresholds must live in configuration, not scattered through logic files.
- CPU1 code must not issue brake, hydraulic, high-voltage or actuator execution commands.
- Generic dump files such as `common.c`, `misc.c` and `utils.c` are forbidden under `ecu/`.
- Sleep calls should stay in task/application boundaries, not inside pure logic modules.

## Next verification gate

Before this work is considered complete:

1. Run `python tests\python\run_tests.py`.
2. Run the forbidden-pattern static checker.
3. Run RISC-V GCC `-fsyntax-only` over all pure framework C modules.
4. Run SDK/CMake project generation if the local SDK build environment is available.
5. Run CPU0 and CPU1 Ninja builds.
6. Fix every failure before committing.

## Known open items

- Hardware drivers still need to be connected to real HPM peripheral APIs after the framework boundaries are accepted.
- CPU0/CPU1 IPC transport still needs binding to the selected SDK multicore/RPMsg mechanism.
- Actuator command executor currently exposes the safe boundary and command state; actual CANopen, relay, hydraulic and PWM output adapters still need to be attached behind that boundary.
- Generated SES projects currently report that OpenOCD was not located, so debugger configuration must be set manually in SEGGER Embedded Studio unless OpenOCD is added to the SDK/tool PATH.
