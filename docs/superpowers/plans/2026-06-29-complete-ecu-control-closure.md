# Complete ECU Control Closure Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Close the main ECU control gaps so SBUS, CAN1, CAN2, CAN3, RS485_1, RS485_2 warning light, DIO outputs, and the vehicle state machines form one hardware-bound and testable control path.

**Architecture:** CPU0 owns all safety-critical hardware services. The vehicle executor receives explicit service pointers from CPU0 instead of creating private bus or DIO objects. Protocol details stay in `protocol/` and `devices/`; hardware register access stays in `drivers/` and board code.

**Tech Stack:** C11, HPM SDK 1.11, FreeRTOS, HPM CAN driver, HPM UART driver, HPM SDK agile_modbus, existing CANopenNode diagnostic switch, Python contract tests, CMake/Ninja, SEGGER Embedded Studio generated project flow.

---

## Scope and fixed assumptions

- CAN1 remains the 250 kbit/s supplier power bus for BMS, DCDC48, DCDC12 and DCAC.
- CAN2 is the BC/BC2 drive and steer bus at 1 Mbit/s. The default non-CANopenNode build must support transmit and receive through `can_bus_service_t`. The CANopenNode diagnostic build remains available for SDO/NMT bench debugging.
- CAN3 is the lift and hydraulic CAN bus. It must have a hardware TX/RX/ISR binding. Lift servo commands use the existing device-level CANopen PDO adapter.
- RS485_1 remains the external ADC module Modbus master.
- RS485_2 owns the 485 warning light from `doc/警示灯/485报警灯使用说明书V1.1.pdf`: default 9600 baud, 8N1, Modbus RTU function 06, slave `0xFF`, register `0x00C2`, direct command values.
- DIO commands must write physical ECU outputs through board-owned output functions, while keeping `dio_service_t` as the logical mask owner.
- RS232 and CAN4 get safe service boundaries and diagnostics only until their business protocol owners are selected.
- No code or docs may use informal uncertainty markers. Hardware-calibration-open values go to config macros and open-items docs.

## Task 1: Add failing contract tests for the missing closure points

**Files:**
- Modify: `tests/python/test_hardware_framework.py`
- Modify: `tests/python/test_vehicle_arbiter.py`

- [ ] Add tests requiring:
  - `vehicle_command_executor_apply` takes a hardware binding/context argument instead of internally creating CAN2/CAN3/DIO/RS485 services.
  - CAN2 and CAN3 have TX-capable hardware init functions.
  - RS485_2 has its own UART hardware init/send/read/clear API.
  - `dio_service` can call a hardware apply callback.
  - warning light values match the PDF: off `0x0060`, yellow slow flash `0x0022`, red steady buzzer `0x0014`, red fast flash buzzer `0x0034`.
  - remote manual control produces non-zero target speed/steer when D/R plus CH3/CH1 are active.
  - SBUS decode-error and credibility flags are derived instead of hardcoded false.

- [ ] Run `python tests/python/run_tests.py`.
  Expected result before implementation: at least the new tests fail for missing APIs or missing code patterns.

## Task 2: Generalize RS485 UART hardware for RS485_1 and RS485_2

**Files:**
- Modify: `ecu/drivers/uart/include/uart_rs485_hw.h`
- Modify: `ecu/drivers/uart/src/uart_rs485_hw.c`
- Modify: `ecu/drivers/uart/include/modbus_master_service.h`
- Modify: `ecu/drivers/uart/src/modbus_master_service.c`

- [ ] Replace RS485_1-only internals with a port descriptor:
  - UART base, IRQ, clock name, DE GPIO triplet, RX/TX enable levels.
  - Per-port static service pointers for ISR dispatch.
  - Public wrappers for `uart_rs485_1_hw_*` and `uart_rs485_2_hw_*`.
- [ ] Keep ISR work bounded to byte drain, line-error count, and idle flag handling.
- [ ] Make `modbus_master_service_process` use generic UART operation callbacks or generic `uart_rs485_hw_send/read/clear_rx` functions, so it is not hardwired to RS485_1.
- [ ] Run `python tests/python/run_tests.py`.
  Expected result: RS485 structure tests pass; remaining closure tests still fail.

## Task 3: Implement RS485_2 warning light as a real Modbus device path

**Files:**
- Modify: `ecu/config/include/ecu_config.h`
- Modify: `ecu/config/src/ecu_config.c`
- Modify: `ecu/devices/include/warning_light_device.h`
- Modify: `ecu/devices/src/warning_light_device.c`
- Modify: `ecu/os/src/ecu_tasks_cpu0.c`
- Modify: `ecu/diag/include/runtime_monitor.h`
- Modify: `ecu/diag/src/runtime_monitor.c`

- [ ] Add config macros for the PDF-defined warning light defaults:
  - `ECU_MODBUS_WARNING_LIGHT_BAUDRATE` = `9600UL`
  - `ECU_MODBUS_WARNING_LIGHT_SLAVE_ID` = `0xFFU`
  - `ECU_MODBUS_WARNING_LIGHT_REGISTER` = `0x00C2U`
  - `ECU_WARNING_LIGHT_VALUE_OFF` = `0x0060U`
  - `ECU_WARNING_LIGHT_VALUE_YELLOW_SLOW_FLASH` = `0x0022U`
  - `ECU_WARNING_LIGHT_VALUE_RED_STEADY_BUZZER` = `0x0014U`
  - `ECU_WARNING_LIGHT_VALUE_RED_FAST_FLASH_BUZZER` = `0x0034U`
- [ ] Move warning light transport from the executor-private `uart_comm_service_t` to CPU0-owned RS485_2 hardware plus a small Modbus write scheduler.
- [ ] Only transmit when indicator/hazard mode changes or when a configured refresh period elapses.
- [ ] Print RS485_2 warning light counters and last command in the debug monitor.
- [ ] Run `python tests/python/run_tests.py`.

## Task 4: Add TX-capable CAN2/CAN3 hardware bindings

**Files:**
- Modify: `ecu/drivers/can/include/can_bus_hw.h`
- Modify: `ecu/drivers/can/src/can_bus_hw.c`
- Modify: `ecu/os/src/ecu_tasks_cpu0.c`
- Modify: `ecu/diag/include/runtime_monitor.h`
- Modify: `ecu/diag/src/runtime_monitor.c`

- [ ] Add TX backend functions for CAN2 and CAN3.
- [ ] Add `can_bus_hw_init_can2_motion()` and `can_bus_hw_init_can3_lift_hydraulic()` that configure Classic CAN 2.0B normal mode, RX/error IRQ, termination settings, and TX backends.
- [ ] Keep CANopenNode diagnostic build owning CAN2 when `ECU_ENABLE_CANOPENNODE=1`; in that build, normal motion TX remains disabled by design.
- [ ] Add CAN3 foreground RX polling and monitor counters.
- [ ] Run `python tests/python/run_tests.py`.

## Task 5: Hardware-bind DIO outputs without breaking logical masks

**Files:**
- Modify: `ecu/drivers/dio/include/dio_service.h`
- Modify: `ecu/drivers/dio/src/dio_service.c`
- Create: `ecu/drivers/dio/include/dio_hw.h`
- Create: `ecu/drivers/dio/src/dio_hw.c`
- Modify: `ecu/apps/agri_chassis_control_cpu0/CMakeLists.txt`
- Modify: `ecu/os/src/ecu_tasks_cpu0.c`

- [ ] Add `dio_service_apply_backend_t` and backend context support to `dio_service_t`.
- [ ] Implement `dio_hw_apply_output_mask()` using board output APIs, mapping logical bit 0 to ECU output 1.
- [ ] Ensure boot/default state writes all managed outputs off.
- [ ] Keep active-low protection: only managed bits are inverted or applied.
- [ ] Run `python tests/python/run_tests.py`.

## Task 6: Refactor vehicle executor to use CPU0-owned hardware services

**Files:**
- Modify: `ecu/vehicle/include/vehicle_command_executor.h`
- Modify: `ecu/vehicle/src/vehicle_command_executor.c`
- Modify: `ecu/os/src/ecu_tasks_cpu0.c`
- Modify: `tests/python/test_hardware_framework.py`

- [ ] Introduce `vehicle_executor_io_t` containing pointers to:
  - CAN2 motion service
  - CAN3 lift/hydraulic service
  - DIO service
  - RS485_2 warning-light service/scheduler
- [ ] Keep device state inside `vehicle_executor_state_t` or a CPU0-owned executor runtime, but remove private CAN/DIO/UART service creation from the executor.
- [ ] CPU0 passes real hardware services into `vehicle_command_executor_apply`.
- [ ] CAN1 power remains in `task_can1_power`; executor result only reflects the last power result owned by that task.
- [ ] Run `python tests/python/run_tests.py`.

## Task 7: Generate manual motion and adjustment commands from SBUS

**Files:**
- Modify: `ecu/remote/include/remote_types.h`
- Modify: `ecu/os/src/ecu_tasks_cpu0.c`
- Modify: `ecu/vehicle/src/command_arbiter.c`
- Modify: `ecu/control/include/motion_control.h`
- Modify: `ecu/control/src/motion_control.c`
- Modify: `ecu/control/include/adjust_control.h`
- Modify: `ecu/control/src/adjust_control.c`
- Modify: `tests/python/test_vehicle_arbiter.py`

- [ ] Extend `remote_input_snapshot_t` or `remote_control_request_t` with normalized throttle, steer, clearance-rate and track-rate commands.
- [ ] Convert raw SBUS CH3 and CH1 into signed calibrated percentages while preserving discrete low/center/high precondition checks.
- [ ] In remote priority mode, populate `target_speed_kph` and four `target_steer_deg[]` values through `motion_control`.
- [ ] In adjustment mode, populate `height_rate_mm_s`, `track_rate_mm_s`, `hydraulic_enable`, and valve mask through `adjust_control`.
- [ ] Keep safety manager as the final clamp for speed, hydraulic and high voltage.
- [ ] Run `python tests/python/run_tests.py`.

## Task 8: Complete SBUS trust and mode-event behavior

**Files:**
- Modify: `ecu/config/include/ecu_config.h`
- Modify: `ecu/config/src/ecu_config.c`
- Modify: `ecu/os/src/ecu_tasks_cpu0.c`
- Modify: `ecu/remote/src/remote_manager.c`
- Modify: `ecu/remote/src/remote_mode_fsm.c`
- Modify: `tests/python/test_remote_fsm.py`

- [ ] Add config thresholds for consecutive SBUS decode errors and credibility range.
- [ ] Derive `decode_error_limit` from the SBUS service counters instead of hardcoding false.
- [ ] Derive `credibility_error` from invalid channel ranges or mutually impossible command combinations.
- [ ] Use existing `remote_discrete_channel_t.changed` or event lifecycle to require a fresh R1/R2 edge after HOME domain changes, SBUS reconnect, and e-stop reset.
- [ ] Run `python tests/python/run_tests.py`.

## Task 9: Add safe RS232/CAN4 service boundaries and CPU1 status hooks

**Files:**
- Modify: `ecu/drivers/uart/include/uart_comm_service.h`
- Modify: `ecu/drivers/uart/src/uart_comm_service.c`
- Modify: `ecu/drivers/can/include/can_bus_hw.h`
- Modify: `ecu/drivers/can/src/can_bus_hw.c`
- Modify: `ecu/os/src/ecu_tasks_cpu1.c`
- Modify: `ecu/apps/agri_chassis_control_cpu1/CMakeLists.txt`
- Modify: `ecu/diag/include/runtime_monitor.h`
- Modify: `ecu/diag/src/runtime_monitor.c`

- [ ] Add non-commanding status structures for RS232 ports and CAN4.
- [ ] Keep them disabled from actuator control; CPU1 may publish heartbeat/status only.
- [ ] Do not invent device protocols for RS232 or CAN4.
- [ ] Run `python tests/python/run_tests.py`.

## Task 10: Documentation, build verification, and final cleanup

**Files:**
- Modify: `docs/ecu-main-control-progress.md`
- Modify: `docs/ecu-configuration-open-items.md`
- Modify: `docs/ECU_CODE_READING_GUIDE.md` if structure changes require reading-guide updates.

- [ ] Document the completed hardware-bound paths and remaining calibration-only items.
- [ ] Run:
  - `python tests/python/run_tests.py`
  - `python tools/check_no_forbidden_patterns.py`
  - `git diff --check`
  - `ninja -C tmp/cmake_cpu0_power`
  - `ninja -C tmp/cmake_cpu1_power`
  - `ninja -C tmp/cmake_cpu0_power_canopen`
- [ ] Scan project-owned code/docs/tests for informal uncertainty markers.
- [ ] Commit the implementation on `codex/complete-ecu-control-closure`.

