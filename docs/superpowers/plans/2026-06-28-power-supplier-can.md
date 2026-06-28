# Power Supplier CAN Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement the full CAN1 supplier power protocol for BMS, DCDC48, DCDC12 and DCAC, wire it into CPU0 feedback and diagnostics, and submit the result.

**Architecture:** Add a generic classic CAN frame boundary, a pure `power_can_protocol` pack/parse module, and a `power_device` scheduler/snapshot layer. CAN1 hardware provides TX/RX only through `can_bus_hw`; high-level device code never accesses HPM registers.

**Tech Stack:** HPM6750 HPM SDK 1.11, SEGGER Embedded Studio/CMake project files, FreeRTOS, classic CAN 2.0B extended frames, Python source contract tests.

---

### Task 1: Add red tests for the power-bus contract

**Files:**
- Create: `tests/python/test_power_supplier_can.py`

- [ ] **Step 1: Write the failing tests**

```python
"""Power supplier CAN contract tests."""

from __future__ import annotations

import pathlib


def read(root: pathlib.Path, rel: str) -> str:
    path = root / rel
    assert path.exists(), f"missing {rel}"
    return path.read_text(encoding="utf-8")


def test_power_supplier_can_protocol_files_and_ids(root: pathlib.Path) -> None:
    header = read(root, "ecu/protocol/power_can/include/power_can_protocol.h")
    source = read(root, "ecu/protocol/power_can/src/power_can_protocol.c")
    cmake = read(root, "ecu/apps/agri_chassis_control_cpu0/CMakeLists.txt")

    for token in [
        "POWER_CAN_BMS_VCU_COMMAND_ID        (0x1410F41EUL)",
        "POWER_CAN_BMS_STATUS_ID             (0x18111EF4UL)",
        "POWER_CAN_BMS_CELL_VOLTAGE_ID       (0x18141EF4UL)",
        "POWER_CAN_BMS_CELL_TEMPERATURE_ID   (0x18151EF4UL)",
        "POWER_CAN_BMS_SOF_ID                (0x18161EF4UL)",
        "POWER_CAN_BMS_SOP_ID                (0x18171EF4UL)",
        "POWER_CAN_BMS_SOH_STATISTICS_ID     (0x18211EF4UL)",
        "POWER_CAN_BMS_ERROR_STATUS_ID       (0x18221EF4UL)",
        "POWER_CAN_DCDC48_CONTROL_ID         (0x10262B27UL)",
        "POWER_CAN_DCDC48_STATUS_ID          (0x18F8622BUL)",
        "POWER_CAN_DCDC12_CONTROL_ID         (0x18EF3010UL)",
        "POWER_CAN_DCDC12_STATUS_ID          (0x18FF3247UL)",
        "POWER_CAN_DCAC_CONTROL_ID           (0x1806B6A5UL)",
        "POWER_CAN_DCAC_STATUS_ID            (0x18FF50B6UL)",
        "POWER_CAN_DCAC_INPUT_STATUS_ID      (0x18FE50B6UL)",
    ]:
        assert token in header, token

    for token in [
        "power_can_build_bms_command",
        "power_can_build_dcdc48_control",
        "power_can_build_dcdc12_control",
        "power_can_build_dcac_control",
        "power_can_parse_bms_status",
        "power_can_parse_dcdc48_status",
        "power_can_parse_dcdc12_status",
        "power_can_parse_dcac_status",
        "power_can_parse_dcac_input_status",
        "power_can_read_u16_le",
        "power_can_write_u16_be",
    ]:
        assert token in source or token in header, token
    assert "power_can_protocol.c" in cmake


def test_can_service_and_can1_hardware_support_extended_frames(root: pathlib.Path) -> None:
    service_h = read(root, "ecu/drivers/can/include/can_bus_service.h")
    service_c = read(root, "ecu/drivers/can/src/can_bus_service.c")
    hw_h = read(root, "ecu/drivers/can/include/can_bus_hw.h")
    hw_c = read(root, "ecu/drivers/can/src/can_bus_hw.c")

    for token in [
        "ecu_can_frame_t",
        "CAN_BUS_FRAME_MAX_DATA_BYTES",
        "can_bus_service_send_frame",
        "can_bus_service_set_tx_backend",
        "last_tx_frame",
    ]:
        assert token in service_h or token in service_c, token

    for token in [
        "can_bus_hw_init_can1_power",
        "can_bus_hw_send_can1_frame",
        "can_bus_hw_poll_can1_rx",
        "BOARD_CAN1_BASE",
        "BOARD_CAN1_IRQn",
        "can_send_high_priority_message_nonblocking",
        "tx_message.extend_id = frame->extended",
    ]:
        assert token in hw_h or token in hw_c, token


def test_power_device_decodes_feedback_and_schedules_commands(root: pathlib.Path) -> None:
    power_h = read(root, "ecu/devices/include/power_device.h")
    power_c = read(root, "ecu/devices/src/power_device.c")
    config_h = read(root, "ecu/config/include/ecu_config.h")
    config_c = read(root, "ecu/config/src/ecu_config.c")

    for token in [
        "power_device_process_rx",
        "power_device_get_snapshot",
        "power_device_snapshot_t",
        "bms",
        "dcdc48",
        "dcdc12",
        "dcac",
    ]:
        assert token in power_h or token in power_c, token

    for token in [
        "ECU_POWER_CAN_TX_ENABLE",
        "ECU_POWER_BMS_COMMAND_PERIOD_MS",
        "ECU_POWER_DCDC48_COMMAND_PERIOD_MS",
        "ECU_POWER_DCDC12_COMMAND_PERIOD_MS",
        "ECU_POWER_DCAC_COMMAND_PERIOD_MS",
        "ECU_DCDC48_DEFAULT_TERMINAL_VOLTAGE_DV",
        "ECU_DCDC12_DEFAULT_OUTPUT_VOLTAGE_DV",
        "ECU_DCAC_DEFAULT_OUTPUT_VOLTAGE_DV",
        "ECU_POWER_PROTOCOL_SUPPLIER_CAN",
    ]:
        assert token in config_h or token in config_c, token

    for token in [
        "power_can_build_bms_command",
        "power_can_build_dcdc48_control",
        "power_can_build_dcdc12_control",
        "power_can_build_dcac_control",
        "power_can_parse_bms_status",
        "power_can_parse_dcdc48_status",
        "power_can_parse_dcdc12_status",
        "power_can_parse_dcac_status",
        "power_can_parse_dcac_input_status",
    ]:
        assert token in power_c, token


def test_cpu0_monitor_reports_can1_power_snapshot(root: pathlib.Path) -> None:
    tasks_c = read(root, "ecu/os/src/ecu_tasks_cpu0.c")
    monitor_h = read(root, "ecu/diag/include/runtime_monitor.h")
    monitor_c = read(root, "ecu/diag/src/runtime_monitor.c")

    for token in [
        "can_bus_hw_init_can1_power",
        "can_bus_hw_poll_can1_rx",
        "power_device_process_rx",
        "power_device_get_snapshot",
        "refresh_power_feedback",
    ]:
        assert token in tasks_c, token

    assert "power_snapshot" in monitor_h
    assert "[ECU POWER]" in monitor_c
    assert "last_tx_id" in monitor_c
    assert "bms_soc" in monitor_c
```

- [ ] **Step 2: Run the tests and verify they fail because the feature is absent**

Run: `python tests/python/run_tests.py`
Expected: at least one failure from `test_power_supplier_can.py` reporting missing `ecu/protocol/power_can/include/power_can_protocol.h`.

### Task 2: Add generic CAN frame support and CAN1 hardware binding

**Files:**
- Modify: `ecu/drivers/can/include/can_bus_service.h`
- Modify: `ecu/drivers/can/src/can_bus_service.c`
- Modify: `ecu/drivers/can/include/can_bus_hw.h`
- Modify: `ecu/drivers/can/src/can_bus_hw.c`

- [ ] **Step 1: Add `ecu_can_frame_t` and `can_bus_service_send_frame`**

The service records 11-bit and 29-bit classic CAN frames and calls an optional TX backend. `can_bus_service_send_canopen` converts the CANopen frame to a standard `ecu_can_frame_t`.

- [ ] **Step 2: Add CAN1 RX/TX hardware functions**

`can_bus_hw_init_can1_power` initializes `BOARD_CAN1_BASE`, registers the TX backend, enables the CAN1 IRQ, and accepts classic extended frames. `can_bus_hw_send_can1_frame` fills `can_transmit_buf_t` including `extend_id`.

- [ ] **Step 3: Run tests**

Run: `python tests/python/run_tests.py`
Expected: the CAN service/hardware test passes; protocol and power device tests still fail until later tasks.

### Task 3: Add pure power supplier CAN pack/parse module

**Files:**
- Create: `ecu/protocol/power_can/include/power_can_protocol.h`
- Create: `ecu/protocol/power_can/src/power_can_protocol.c`
- Modify: `ecu/apps/agri_chassis_control_cpu0/CMakeLists.txt`

- [ ] **Step 1: Define frame IDs, periods, enums and typed status structs**

Expose constants and structs for BMS, DCDC48, DCDC12 and DCAC. Keep all scaling in this module.

- [ ] **Step 2: Implement builders**

Build these frames: BMS VCU command, DCDC48 control, DCDC12 control and DCAC control.

- [ ] **Step 3: Implement parsers**

Parse BMS status/cell voltage/cell temperature/SOF/SOP/SOH/error status, DCDC48 status, DCDC12 status, DCAC status and DCAC input status.

- [ ] **Step 4: Run tests**

Run: `python tests/python/run_tests.py`
Expected: protocol contract test passes; power device and monitor tests still fail until later tasks.

### Task 4: Wire the protocol into power_device and CPU0

**Files:**
- Modify: `ecu/config/include/ecu_config.h`
- Modify: `ecu/config/src/ecu_config.c`
- Modify: `ecu/devices/include/power_device.h`
- Modify: `ecu/devices/src/power_device.c`
- Modify: `ecu/vehicle/src/vehicle_command_executor.c`
- Modify: `ecu/os/src/ecu_tasks_cpu0.c`

- [ ] **Step 1: Add configuration macros and default protocol selection**

Enable `ECU_POWER_PROTOCOL_SUPPLIER_CAN` by default and centralize all periods, timeouts, and output setpoints.

- [ ] **Step 2: Add power snapshot and RX processing**

`power_device_process_rx` consumes the last CAN1 frame snapshot and updates typed feedback. `power_device_get_snapshot` gives CPU0 and diagnostics a copy.

- [ ] **Step 3: Add periodic command scheduling**

`power_device_apply` sends due command frames through `can_bus_service_send_frame`. Commands are safe-off when high voltage is not requested.

- [ ] **Step 4: Wire CAN1 runtime**

Initialize CAN1 hardware in CPU0 runtime, poll RX in `task_can1_power`, process RX, and derive hardware feedback from the power snapshot.

- [ ] **Step 5: Run tests**

Run: `python tests/python/run_tests.py`
Expected: all new power tests pass.

### Task 5: Add runtime monitor output and documentation updates

**Files:**
- Modify: `ecu/diag/include/runtime_monitor.h`
- Modify: `ecu/diag/src/runtime_monitor.c`
- Modify: `docs/ecu-configuration-open-items.md`

- [ ] **Step 1: Add monitor snapshot fields**

Add `power_device_snapshot_t power_snapshot` and CAN1 frame diagnostics.

- [ ] **Step 2: Print a compact power line**

Print command state, online flags, BMS SOC/voltage/current, DCDC/DCAC status and last CAN1 TX/RX identifiers.

- [ ] **Step 3: Update open items**

Record that protocol IDs are known and that final output voltage/current setpoints remain calibration values.

- [ ] **Step 4: Run tests**

Run: `python tests/python/run_tests.py`
Expected: all source contract tests pass.

### Task 6: Build and submit

**Files:**
- All changed files from previous tasks.

- [ ] **Step 1: Run full source tests**

Run: `python tests/python/run_tests.py`
Expected: all tests pass.

- [ ] **Step 2: Run target build checks**

Run the existing CPU0 and CPU1 build commands from the project build scripts or CMake presets available in the workspace. Expected: both target configurations compile with exit code 0.

- [ ] **Step 3: Review diff**

Run: `git diff --stat` and inspect changed source files for large mixed-responsibility files.

- [ ] **Step 4: Commit**

Run: `git add <changed files>` then `git commit -m "feat: implement supplier power CAN protocol"`.

- [ ] **Step 5: Publish**

Use normal `git push origin main` when reachable. If HTTPS Git transport is unavailable but GitHub API is reachable, publish the exact commit tree through the GitHub API as done earlier in this repository.
