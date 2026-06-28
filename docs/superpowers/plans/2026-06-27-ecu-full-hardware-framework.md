# ECU Full Hardware Framework Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Subagents are disabled for this session by higher-priority instruction, so execute inline with review checkpoints. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extend the main ECU framework from logical control scaffolding to a complete hardware-binding framework with configurable hardware mappings, device adapters, protocol boundaries and executor wiring.

**Architecture:** CPU0 remains the only safety-critical command owner. Remote/control/vehicle modules make decisions in calibrated units; device and driver adapters translate final commands to CANopen, digital outputs, analog input services, hydraulic outputs, warning lights and communication health. Any calibration-open hardware value is centralized in `ecu/config` macros and default config structures so the user can change wiring, node IDs, PDOs, polarity and limits without editing business logic.

**Tech Stack:** C99, HPM SDK 1.11, FreeRTOS, HPM6750, SEGGER Embedded Studio 8.28, Python contract tests, RISC-V GCC/Ninja target builds.

---

## File structure

- `ecu/config/include/ecu_config.h`, `ecu/config/src/ecu_config.c`: project node IDs, CAN bitrates, DO polarity, ADC scaling, hydraulic valve masks, CANopen PDO defaults and safety limits.
- `ecu/protocol/canopen/*`: CANopen frame and object dictionary helper types; no HPM CAN dependency.
- `ecu/protocol/modbus/*`: Modbus RTU request/CRC helpers for ADC/RS485 module integration; no UART dependency.
- `ecu/drivers/can/*`: CAN bus frame queue and backend interface used by CANopen devices.
- `ecu/drivers/dio/*`: digital output state backend with polarity handling.
- `ecu/drivers/adc/*`: analog input conversion to millivolts and sensor validity flags.
- `ecu/drivers/uart/*`: UART-port service boundary for RS232/RS485 diagnostics and Modbus transport.
- `ecu/devices/*`: power, motion, lift, hydraulic, warning light, local IO and communication-health adapters.
- `ecu/vehicle/src/vehicle_command_executor.c`: final command fan-out into devices. This remains the only hardware command exit.
- `ecu/os/src/ecu_tasks_cpu0.c`: task-level data flow from drivers/devices to safety snapshots and final command execution.
- `tests/python/*`: contract tests for config centralization, device boundaries, executor wiring and CPU1 restrictions.
- `docs/ecu-main-control-architecture.md`, `docs/ecu-main-control-progress.md`, `README.md`: updated reading guide and build notes.

## Calibration-open configuration rules

- Every calibration-open node ID, CAN ID, PDO COB-ID, baud rate, ADC scale, DO polarity, relay/MOS mask, hydraulic valve bit and limit is named with `ECU_` or is inside a clearly named config structure.
- Business modules must not contain raw hardware numbers.
- Changing a calibration-open value must require editing only `ecu/config`.
- Hardware adapters must expose status and diagnostics even when the current backend is stateful software-only.

## Execution tasks

### Task 1: Lock architecture gaps with failing tests

- [x] Add tests requiring `ecu/devices`, `ecu/drivers/can`, `ecu/drivers/dio`, `ecu/drivers/adc`, `ecu/drivers/uart`, `ecu/protocol/canopen`, and `ecu/protocol/modbus`.
- [x] Add tests requiring `ECU_` hardware values in config and forbidding raw node IDs/CAN IDs outside config.
- [x] Add tests requiring `vehicle_command_executor` to call device adapters and no remote/control module to include device headers.
- [x] Run tests and confirm they fail before implementation.

### Task 2: Add central hardware configuration

- [x] Add CAN, CANopen node, digital output, analog input, hydraulic, warning light and Modbus config structures.
- [x] Add project default values marked as calibration-open where required.
- [x] Add accessor functions for each config group.
- [x] Run Python tests and RISC-V syntax checks for config.

### Task 3: Add protocol and driver boundaries

- [x] Add pure CANopen frame helpers.
- [x] Add pure Modbus RTU CRC/request helpers.
- [x] Add CAN, DIO, ADC and UART service interfaces with stateful default backends.
- [x] Run tests and syntax checks.

### Task 4: Add device adapters

- [x] Add power device adapter for BMS, DC/DC, inverter and high-voltage request state.
- [x] Add motion device adapter for four drive/steer/brake commands.
- [x] Add lift/hydraulic adapter for clearance and track-width outputs.
- [x] Add local IO/analog adapter for DI/DO/ADC snapshots.
- [x] Add warning light adapter for 485 warning lamp plus local indicators.
- [x] Run tests and syntax checks.

### Task 5: Wire executor and tasks

- [x] Extend `vehicle_executor_state_t` with per-device application results and diagnostics.
- [x] Make `vehicle_command_executor_apply()` call only device adapters for final output fan-out.
- [x] Update CPU0 task context to hold driver/device runtime state and status snapshots.
- [x] Keep CPU1 free of safety-critical device includes.
- [x] Run tests, static scan, syntax checks and SDK builds.

### Task 6: Documentation and final verification

- [x] Update reading guide with the device/driver/protocol path.
- [x] Update progress file with verified commands and remaining real-hardware calibration points.
- [x] Run final verification: Python tests, static scan, `git diff --check`, CPU0 build, CPU1 build.
- [x] Commit with a message describing the full hardware framework.
