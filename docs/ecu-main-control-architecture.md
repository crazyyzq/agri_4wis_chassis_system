# ECU Main Control Framework Reading Guide

This document explains how to read the ECU control code from `main()` outward. Keep it open while reviewing the source tree.

## Current architecture

CPU0 owns all safety-critical decisions and all final actuator output. CPU1 is reserved for non-critical sensing, communication snapshots and diagnostics publication.

The intended dependency direction is:

```text
apps/os
  -> remote/control/vehicle
  -> vehicle_command_executor
  -> devices
  -> drivers
  -> protocol
  -> board/HPM SDK binding later
```

Business logic must not write CAN, GPIO, UART, hydraulic outputs or warning lights directly. It produces structured requests and final actuator commands. The executor fans the final command out through device adapters.

## Top-level layout

```text
ecu/
  apps/                         CPU0/CPU1 application entry points.
  common/                       Shared primitive types and time aliases.
  config/                       Central timing, thresholds, priorities and calibration-open hardware mappings.
  control/                      Motion and chassis-adjustment command shaping.
  devices/                      BMS, DC/DC, drive, steer, lift, hydraulic, local IO and warning-light adapters.
  diag/                         Diagnostic code definitions and text.
  drivers/                      CAN, DIO, ADC, UART and SBUS service boundaries.
  ipc/                          CPU0/CPU1 snapshot contracts.
  os/                           FreeRTOS task orchestration helpers.
  protocol/                     Pure protocol helpers: SBUS, CANopen and Modbus RTU.
  remote/                       Remote-control interpretation FSMs.
  vehicle/                      Command arbitration, safety clamping, executor and vehicle state.
```

## 1. Start from CPU0 main

File: `ecu/apps/agri_chassis_control_cpu0/src/main_cpu0.c`

CPU0 performs board-level startup, creates FreeRTOS tasks, then starts the scheduler. Do not put control policy here. If a future edit adds real behavior inside `main()`, move it into `os`, `remote`, `vehicle`, `control`, `devices` or `drivers`.

Then read:

- `ecu/os/include/ecu_tasks.h`
- `ecu/os/src/ecu_tasks_cpu0.c`
- `ecu/os/src/ecu_tasks_cpu1.c`

The CPU0 task file is the wiring layer. It samples inputs, updates state machines, arbitrates commands, applies safety, calls the executor and refreshes snapshots. CPU0 owns the real CAN2, CAN3, DIO, RS485_1 and RS485_2 service instances and passes them into the executor through `vehicle_executor_io_t`. CPU1 must not include executor, safety manager or hardware command headers.

## 2. Read shared types and configuration

Files:

- `ecu/common/include/ecu_types.h`
- `ecu/common/include/ecu_time.h`
- `ecu/config/include/ecu_config.h`
- `ecu/config/src/ecu_config.c`
- `ecu/diag/include/diag_codes.h`
- `ecu/diag/src/diag_codes.c`

`ecu_types.h` defines small shared result and count types. `ecu_config.h` owns all project-level constants and calibration-open hardware values.

Important rule for this project:

- If a value is a timing threshold, SBUS threshold, priority, limit, node ID, CAN bitrate, Modbus address, DIO mask, relay polarity, ADC scale or valve bit, put it in `ecu/config`.
- If the value is likely to change on the real machine, name it with `ECU_` or place it inside the default hardware config returned by `ecu_hardware_config_default()`.
- Do not write raw hardware numbers inside `remote`, `control`, `vehicle`, `devices` or task logic.

## 3. Read the SBUS input path

Files:

- `ecu/protocol/sbus/include/sbus_decoder.h`
- `ecu/protocol/sbus/src/sbus_decoder.c`
- `ecu/drivers/sbus/include/sbus_service.h`
- `ecu/drivers/sbus/src/sbus_service.c`

The decoder is pure protocol code. It decodes one SBUS frame into 16 channels plus flags.

The service is the UART-facing holder. The intended target path is:

1. UART receives bytes.
2. `ecu/drivers/sbus/src/sbus_uart_hw.c` configures UART1 as 100000 baud 8E2 and feeds bytes from RX data/timeout plus RX line idle interrupts.
3. The service validates 25-byte frames and updates a snapshot.
4. Foreground tasks read the snapshot and never parse UART buffers directly.

## 4. Read remote-control interpretation

Start with:

- `ecu/remote/include/remote_types.h`
- `ecu/remote/src/remote_manager.c`

Then read the individual FSM files:

- `remote_link_fsm.c`: SBUS online, qualifying, failsafe and timeout state.
- `remote_arm_fsm.c`: neutral-qualified arm readiness.
- `remote_estop_fsm.c`: latched remote emergency stop and reset.
- `remote_gear_fsm.c`: requested gear and active gear separation.
- `remote_mode_fsm.c`: motion-domain guard and mode request filtering.
- `remote_adjust_fsm.c`: clearance/track-width adjustment ownership.
- `remote_power_fsm.c`: high-voltage and shutdown request interpretation.
- `remote_authority_fsm.c`: manual/automatic authority.
- `remote_event_lifecycle.c`: expiry of operator action events.
- `remote_lights_fsm.c`: horn, headlight and indicator requests.

Remote code must never include board headers, device headers or executor headers. It only produces a `remote_control_request_t`.

SBUS analog values are kept alongside the discrete channel states. `steer_per_mille`, `throttle_per_mille`, `clearance_per_mille` and `track_per_mille` flow from `ecu_tasks_cpu0.c` through `remote_manager.c` into the final arbiter. R1/R2 mode events require a fresh stable edge after HOME domain selection, so an old switch position cannot change the motion mode after the guard expires.

## 5. Read vehicle arbitration and safety

Files:

- `ecu/vehicle/include/vehicle_types.h`
- `ecu/vehicle/src/command_arbiter.c`
- `ecu/vehicle/src/safety_manager.c`
- `ecu/vehicle/src/vehicle_command_executor.c`
- `ecu/vehicle/src/vehicle_state.c`

Read in that order.

The arbiter rebuilds a complete command from safe defaults every cycle. The safety manager clamps unsafe outputs based on estop, fault severity, remote state and platform state. The executor is the only hardware-command exit from the vehicle layer.

Conservative safety behavior:

- Emergency stop forces zero speed, brake release off, hydraulic off and high voltage off.
- A-class faults force the same safe command.
- Missing command source falls back to a safe default command.

## 6. Read control command-shape modules

Files:

- `ecu/control/include/motion_control.h`
- `ecu/control/src/motion_control.c`
- `ecu/control/include/adjust_control.h`
- `ecu/control/src/adjust_control.c`

These modules convert high-level requests into normalized command structures. They do not know SBUS channel numbers and do not talk to hardware.

## 7. Read protocol, driver and device layers

Protocol helpers:

- `ecu/drivers/canopen/include/canopen_master_service.h`
- `ecu/drivers/canopen/src/canopen_master_service.c`
- HPM SDK `CANopenNode`
- HPM SDK `agile_modbus`

CANopen and Modbus use proven middleware instead of a project-local protocol stack. CPU0 enables HPM SDK `CANopenNode` for both CAN2 and CAN3, and enables HPM SDK `agile_modbus` for RS485_1 analog acquisition and RS485_2 warning-light control. Project code focuses on object indexes, register maps, scaling and high-level control functions.

Driver/service boundaries:

- `ecu/drivers/can/include/can_bus_service.h`
- `ecu/drivers/can/src/can_bus_service.c`
- `ecu/drivers/can/include/can_bus_hw.h`
- `ecu/drivers/can/src/can_bus_hw.c`
- `ecu/drivers/dio/include/dio_hw.h`
- `ecu/drivers/dio/src/dio_hw.c`
- `ecu/drivers/dio/include/dio_service.h`
- `ecu/drivers/dio/src/dio_service.c`
- `ecu/drivers/adc/include/analog_input_service.h`
- `ecu/drivers/adc/src/analog_input_service.c`
- `ecu/drivers/uart/include/uart_comm_service.h`
- `ecu/drivers/uart/src/uart_comm_service.c`
- `ecu/drivers/uart/include/uart_rs485_hw.h`
- `ecu/drivers/uart/src/uart_rs485_hw.c`
- `ecu/drivers/uart/include/modbus_master_service.h`
- `ecu/drivers/uart/src/modbus_master_service.c`
- `ecu/drivers/canopen/include/canopen_master_service.h`
- `ecu/drivers/canopen/src/canopen_master_service.c`

Device adapters:

- `ecu/devices/include/power_device.h`
- `ecu/devices/src/power_device.c`
- `ecu/devices/include/analog_modbus_device.h`
- `ecu/devices/src/analog_modbus_device.c`
- `ecu/devices/include/servo_drive_canopen.h`
- `ecu/devices/src/servo_drive_canopen.c`
- `ecu/devices/include/motion_device.h`
- `ecu/devices/src/motion_device.c`
- `ecu/devices/include/lift_hydraulic_device.h`
- `ecu/devices/src/lift_hydraulic_device.c`
- `ecu/devices/include/local_io_device.h`
- `ecu/devices/src/local_io_device.c`
- `ecu/devices/include/warning_light_device.h`
- `ecu/devices/src/warning_light_device.c`

This layer is where current hardware mapping defaults live. SBUS UART1 is bound to HPM UART hardware. CAN2 is the CANopenNode BC/BC2 motion network, and CAN3 is the CANopenNode lift/hydraulic network. DIO service polarity conversion is connected to the 12 isolated board outputs through `dio_hw`. RS485_1/UART11 is bound to a Modbus master transport service for the 8-channel analog acquisition module; RS485_2/UART12 is bound to the warning-light Agile Modbus direct-control protocol. RS485 direction is GPIO-controlled because HPM6750 SDK 1.11 does not provide automatic DE switching for this UART IP. `servo_drive_canopen` remains the device-level CiA 402 boundary for normal vehicle command fan-out and delegates NMT/SDO requests to `canopen_master_service`. Device adapters should call high-level control functions such as set drive velocity, set steering angle, read ADC module channels or set warning-light mode; they should not assemble protocol-stack internals manually.

## 8. Read CPU0/CPU1 data exchange

Files:

- `ecu/ipc/include/ipc_snapshot.h`
- `ecu/ipc/src/ipc_snapshot.c`
- `ecu/apps/agri_chassis_control_cpu1/src/main_cpu1.c`

CPU1 publishes non-critical sensor and communication snapshots. CPU0 consumes snapshots and remains the only side allowed to decide final outputs.

## 9. Tests and static checks

Files:

- `tests/python/run_tests.py`
- `tests/python/test_remote_fsm.py`
- `tests/python/test_vehicle_arbiter.py`
- `tests/python/test_hardware_framework.py`
- `tools/check_no_forbidden_patterns.py`

These are framework-contract checks. They protect:

- Missing FSM and adapter modules.
- Unsafe CPU1 dependencies.
- Scattered SBUS and CANopen magic values.
- Remote/control logic depending directly on hardware.
- Generic catch-all files that usually become unmaintainable.

## Future hardware binding rule

When adding a real device driver:

1. Put configurable hardware values in `ecu/config`.
2. Keep protocol packing/parsing in `ecu/protocol`.
3. Put HPM peripheral interaction behind `ecu/drivers`.
4. Put equipment-specific command translation in `ecu/devices`.
5. Call devices only from `vehicle_command_executor`.
6. Add a Python static/contract test if the boundary is important.
7. Run Python tests, static checks and CPU0/CPU1 builds before committing.
