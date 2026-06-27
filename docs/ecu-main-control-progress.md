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
- Main framework plan: `docs/superpowers/plans/2026-06-27-ecu-main-control-framework.md`
- Hardware adapter framework plan: `docs/superpowers/plans/2026-06-27-ecu-full-hardware-framework.md`

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
  - Device-adapter fan-out for the final command.
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
- Configurable hardware adapter framework:
  - Guessed CAN bitrates, CANopen node IDs, PDO COB-ID bases, DIO masks, hydraulic valve masks, ADC scaling, Modbus addresses and UART baud rates are centralized behind `ECU_GUESS_*` macros and `ecu_hardware_config_default()`.
  - Pure CANopen frame helpers and Modbus RTU request/CRC helpers are available under `ecu/protocol`.
  - CAN, DIO, ADC and UART service boundaries are available under `ecu/drivers`.
  - Power, motion, lift/hydraulic, local IO and warning-light device adapters are available under `ecu/devices`.
  - `vehicle_command_executor_apply()` now fans out the final command through device adapters instead of keeping the output boundary as state-only.
  - DIO active-low conversion is limited to the configured managed output mask; hydraulic valve outputs are cleared as a group before a new valve target is written.

## Design constraints already encoded in tests

- Remote modules must not include board/HPM peripheral headers directly.
- Raw SBUS thresholds must live in configuration, not scattered through logic files.
- CPU1 code must not issue brake, hydraulic, high-voltage or actuator execution commands.
- Generic dump files such as `common.c`, `misc.c` and `utils.c` are forbidden under `ecu/`.
- Sleep calls should stay in task/application boundaries, not inside pure logic modules.
- Guessed hardware values should stay in `ecu/config`; raw guessed CANopen IDs and similar numbers should not be scattered through device/business code.

## Next verification gate

Before any next framework commit is considered complete:

1. Run `python tests\python\run_tests.py`.
2. Run the forbidden-pattern static checker.
3. Run RISC-V GCC `-fsyntax-only` over all pure framework C modules.
4. Run SDK/CMake project generation if the local SDK build environment is available.
5. Run CPU0 and CPU1 Ninja builds.
6. Run `git diff --check`.
7. Fix every failure before committing.

## Latest verification evidence

Verified on 2026-06-27:

- `python tests\python\run_tests.py`: 22/22 tests passed.
- `python tools\check_no_forbidden_patterns.py`: no forbidden ECU framework patterns found.
- `git diff --check`: no whitespace errors; only Git line-ending warning for `ecu/apps/agri_chassis_control_cpu0/CMakeLists.txt`.
- RISC-V GCC `-fsyntax-only`: passed for framework, protocol, driver and device modules.
- CPU0 CMake/SES generation and Ninja build: passed, `tmp/cmake_cpu0_full/output/demo.elf` produced.
- CPU1 CMake/SES generation and Ninja build: passed, `tmp/cmake_cpu1_full/output/demo.elf` produced.

## Known open items

- Hardware drivers still need to be connected to real HPM peripheral APIs after the framework boundaries are accepted.
- CPU0/CPU1 IPC transport still needs binding to the selected SDK multicore/RPMsg mechanism.
- CANopen PDO/object mapping, Modbus register mapping, relay polarity, hydraulic valve bits and analog scaling are currently guessed configuration values and should be calibrated on the real vehicle.
- Generated SES projects currently report that OpenOCD was not located, so debugger configuration must be set manually in SEGGER Embedded Studio unless OpenOCD is added to the SDK/tool PATH.
