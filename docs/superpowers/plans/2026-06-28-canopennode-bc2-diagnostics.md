# CANopenNode BC2 Diagnostics Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Enable HPM SDK CANopenNode on ECU CAN2 for safe BC2 diagnostic communication with no motion-control output.

**Architecture:** A compile-time switch selects either the existing RX-only CAN2 monitor or a new CANopenNode diagnostic service. Generated DS301 OD files live under `ecu/protocol/canopen/od/ds301`, while the service owns CANopenNode runtime state and exposes a snapshot to CPU0 diagnostics. Python tools support passive CAN analyzer monitoring and safe COM10 Modbus probing.

**Tech Stack:** C11, HPM SDK 1.11 CANopenNode, FreeRTOS, HPM CAN driver, Python ctypes, pyserial, CMake/Ninja, SEGGER Embedded Studio generated project flow.

---

### Task 1: Contract tests

**Files:**
- Modify: `tests/python/test_hardware_framework.py`

- [ ] Add tests requiring DS301 OD files under `ecu/protocol/canopen/od/ds301`.
- [ ] Add tests requiring `ECU_ENABLE_CANOPENNODE` to select CANopenNode CAN2 ownership.
- [ ] Add tests requiring `canopen_master_service` snapshot fields and COM9 monitor output.
- [ ] Add tests forbidding diagnostic code from sending CiA 402 controlword, RPDO motion commands, or NMT start.
- [ ] Run `python tests\python\run_tests.py` and confirm the new tests fail before implementation.

### Task 2: DS301 OD and CANopenNode build switch

**Files:**
- Create: `ecu/protocol/canopen/od/ds301/OD.c`
- Create: `ecu/protocol/canopen/od/ds301/OD.h`
- Modify: `ecu/apps/agri_chassis_control_cpu0/CMakeLists.txt`
- Modify: `ecu/apps/agri_chassis_control_cpu0/src/user_config.h`

- [ ] Copy generated OD files from `doc/ECU/DS301_OD`.
- [ ] Add OD include/source only when `ECU_ENABLE_CANOPENNODE=ON`.
- [ ] Add `CONFIG_CANOPEN_MASTER` only for the diagnostic master build.
- [ ] Keep normal CPU0 build working with `ECU_ENABLE_CANOPENNODE=OFF`.

### Task 3: CANopen diagnostic service

**Files:**
- Create: `ecu/drivers/canopen/include/canopen_master_service.h`
- Create: `ecu/drivers/canopen/src/canopen_master_service.c`
- Modify: `ecu/os/src/ecu_tasks_cpu0.c`
- Modify: `ecu/diag/include/runtime_monitor.h`
- Modify: `ecu/diag/src/runtime_monitor.c`

- [ ] Add a snapshot with init state, bitrate, node ID under test, heartbeat state, SDO success/error counters and last SDO result.
- [ ] Initialize HPM CANopenNode on `BOARD_CAN2_BASE`/`BOARD_CAN2_IRQn`.
- [ ] Process CANopenNode periodically from `ecu_task_can2_motion_step`.
- [ ] Read only safe SDO objects: `0x1000`, `0x1001`, `0x1018`, `0x6041`, `0x6061`.
- [ ] Print snapshot lines on COM9.
- [ ] When CANopenNode is enabled, do not initialize the RX-only CAN2 monitor.

### Task 4: Python hardware tools

**Files:**
- Create: `tools/can/controlcan.py`
- Create: `tools/can/can2_monitor.py`
- Create: `tools/modbus/rtu_probe.py`

- [ ] Wrap `ControlCAN.dll` with `ctypes` structures matching `ControlCAN.h`.
- [ ] Open CAN analyzer channel 2 at 1 Mbit/s and print passive RX frames.
- [ ] Keep transmit disabled by default.
- [ ] Add COM port listing and safe Modbus RTU read helper for COM10.

### Task 5: Build and hardware verification

**Files:**
- No new source files.

- [ ] Run `python tests\python\run_tests.py`.
- [ ] Build CPU0 normal configuration.
- [ ] Configure and build CPU0 CANopenNode diagnostic configuration.
- [ ] Download the CANopenNode diagnostic ELF only after build passes.
- [ ] Observe COM9 for CANopen diagnostic output.
- [ ] Use CAN analyzer Python monitor to confirm CAN2 traffic.
- [ ] Use COM10 Modbus probe only for safe read checks.
