"""Hardware binding framework contract tests."""

from __future__ import annotations

import pathlib
import re


def read(root: pathlib.Path, rel: str) -> str:
    path = root / rel
    assert path.exists(), f"missing {rel}"
    return path.read_text(encoding="utf-8")


def test_guessed_hardware_values_are_centralized(root: pathlib.Path) -> None:
    config_h = read(root, "ecu/config/include/ecu_config.h")
    config_c = read(root, "ecu/config/src/ecu_config.c")
    required = [
        "ECU_GUESS_CAN1_BITRATE",
        "ECU_GUESS_CAN2_BITRATE",
        "ECU_GUESS_CAN3_BITRATE",
        "ECU_GUESS_CAN4_BITRATE",
        "ECU_GUESS_CANOPEN_BMS_NODE_ID",
        "ECU_GUESS_CANOPEN_DRIVE_FL_NODE_ID",
        "ECU_GUESS_CANOPEN_STEER_FL_NODE_ID",
        "ECU_GUESS_DIO_BRAKE_RELEASE_MASK",
        "ECU_GUESS_DIO_HYDRAULIC_ENABLE_MASK",
        "ECU_GUESS_DIO_MANAGED_OUTPUT_MASK",
        "ECU_GUESS_HYD_VALVE_MANAGED_MASK",
        "ECU_GUESS_ADC_EXTERNAL_MV_MAX",
        "ECU_GUESS_MODBUS_ADC_SLAVE_ID",
        "ecu_hardware_config_default",
    ]
    for token in required:
        assert token in config_h or token in config_c, token


def test_protocol_driver_and_device_layers_exist(root: pathlib.Path) -> None:
    required_files = [
        "ecu/protocol/canopen/include/canopen_frame.h",
        "ecu/protocol/canopen/src/canopen_frame.c",
        "ecu/protocol/modbus/include/modbus_rtu.h",
        "ecu/protocol/modbus/src/modbus_rtu.c",
        "ecu/drivers/can/include/can_bus_service.h",
        "ecu/drivers/can/src/can_bus_service.c",
        "ecu/drivers/dio/include/dio_service.h",
        "ecu/drivers/dio/src/dio_service.c",
        "ecu/drivers/adc/include/analog_input_service.h",
        "ecu/drivers/adc/src/analog_input_service.c",
        "ecu/drivers/uart/include/uart_comm_service.h",
        "ecu/drivers/uart/src/uart_comm_service.c",
        "ecu/devices/include/power_device.h",
        "ecu/devices/src/power_device.c",
        "ecu/devices/include/motion_device.h",
        "ecu/devices/src/motion_device.c",
        "ecu/devices/include/lift_hydraulic_device.h",
        "ecu/devices/src/lift_hydraulic_device.c",
        "ecu/devices/include/local_io_device.h",
        "ecu/devices/src/local_io_device.c",
        "ecu/devices/include/warning_light_device.h",
        "ecu/devices/src/warning_light_device.c",
    ]
    for rel in required_files:
        assert (root / rel).exists(), rel


def test_executor_fans_out_only_through_device_adapters(root: pathlib.Path) -> None:
    executor = read(root, "ecu/vehicle/src/vehicle_command_executor.c")
    required_calls = [
        "power_device_apply",
        "motion_device_apply",
        "lift_hydraulic_device_apply",
        "local_io_device_apply",
        "warning_light_device_apply",
    ]
    for token in required_calls:
        assert token in executor, token
    for forbidden in ["hpm_can", "hpm_gpio", "board_", "HPM_CAN", "HPM_GPIO"]:
        assert forbidden not in executor, forbidden


def test_remote_and_control_do_not_depend_on_devices(root: pathlib.Path) -> None:
    forbidden = ["power_device", "motion_device", "lift_hydraulic_device", "local_io_device"]
    for folder in ["ecu/remote", "ecu/control"]:
        for path in (root / folder).rglob("*.[ch]"):
            text = path.read_text(encoding="utf-8")
            for token in forbidden:
                assert token not in text, f"{path}: forbidden {token}"


def test_no_raw_guessed_canopen_values_outside_config(root: pathlib.Path) -> None:
    guessed_number_pattern = re.compile(r"\b(0x18[0-9A-Fa-f]{2}|0x20[0-9A-Fa-f]{2}|0x60[0-9A-Fa-f]{2}|0x70[0-9A-Fa-f]{2})\b")
    for path in (root / "ecu").rglob("*.[ch]"):
        rel = path.relative_to(root).as_posix()
        if rel.startswith("ecu/config/") or rel.startswith("ecu/sdk_env_v1.11.0/"):
            continue
        text = path.read_text(encoding="utf-8")
        match = guessed_number_pattern.search(text)
        assert match is None, f"{rel}: guessed CANopen raw value {match.group(0)} belongs in config"


def test_dio_active_low_is_limited_to_managed_outputs(root: pathlib.Path) -> None:
    dio_h = read(root, "ecu/drivers/dio/include/dio_service.h")
    dio_c = read(root, "ecu/drivers/dio/src/dio_service.c")
    lift_c = read(root, "ecu/devices/src/lift_hydraulic_device.c")

    assert "managed_output_mask" in dio_h
    assert "(~logical) & service->managed_output_mask" in dio_c
    assert "config->hydraulic_managed_valve_mask, false" in lift_c


def test_default_dio_and_hydraulic_masks_do_not_overlap(root: pathlib.Path) -> None:
    config_h = read(root, "ecu/config/include/ecu_config.h")

    def bit_value(name: str) -> int:
        match = re.search(rf"#define\s+{name}\s+\(1UL\s*<<\s*(\d+)\)", config_h)
        assert match, f"missing single-bit macro {name}"
        return 1 << int(match.group(1))

    dio_names = [
        "ECU_GUESS_DIO_BRAKE_RELEASE_MASK",
        "ECU_GUESS_DIO_HYDRAULIC_ENABLE_MASK",
        "ECU_GUESS_DIO_HORN_MASK",
        "ECU_GUESS_DIO_HEADLIGHT_MASK",
        "ECU_GUESS_DIO_LEFT_INDICATOR_MASK",
        "ECU_GUESS_DIO_RIGHT_INDICATOR_MASK",
    ]
    hydraulic_names = [
        "ECU_GUESS_HYD_VALVE_TRACK_EXTEND_MASK",
        "ECU_GUESS_HYD_VALVE_TRACK_RETRACT_MASK",
        "ECU_GUESS_HYD_VALVE_LIFT_UP_MASK",
        "ECU_GUESS_HYD_VALVE_LIFT_DOWN_MASK",
    ]

    dio_mask = 0
    for name in dio_names:
        dio_mask |= bit_value(name)
    hydraulic_mask = 0
    for name in hydraulic_names:
        hydraulic_mask |= bit_value(name)

    assert (dio_mask & hydraulic_mask) == 0


def test_cpu0_runtime_monitor_is_configurable_and_task_owned(root: pathlib.Path) -> None:
    config_h = read(root, "ecu/config/include/ecu_config.h")
    monitor_h = read(root, "ecu/diag/include/runtime_monitor.h")
    monitor_c = read(root, "ecu/diag/src/runtime_monitor.c")
    tasks_c = read(root, "ecu/os/src/ecu_tasks_cpu0.c")
    cmake = read(root, "ecu/apps/agri_chassis_control_cpu0/CMakeLists.txt")

    for token in [
        "ECU_ENABLE_DEBUG_MONITOR",
        "ECU_DEBUG_MONITOR_PERIOD_MS",
        "ECU_DEBUG_MONITOR_VERBOSE",
    ]:
        assert token in config_h, token

    assert "runtime_monitor_snapshot_t" in monitor_h
    assert "runtime_monitor_print_cpu0" in monitor_h
    assert "printf(" in monitor_c
    assert "runtime_monitor_print_cpu0" in tasks_c
    assert "ECU_ENABLE_DEBUG_MONITOR" in tasks_c
    assert "runtime_monitor.c" in cmake


def test_cpu0_startup_and_fatal_hooks_are_visible_on_debug_console(root: pathlib.Path) -> None:
    main_c = read(root, "ecu/apps/agri_chassis_control_cpu0/src/main_cpu0.c")

    assert "ECU CPU0 boot" in main_c
    assert "create_task_or_report" in main_c
    assert "FATAL malloc failed" in main_c
    assert "FATAL stack overflow" in main_c
    assert "\\r\\n" in main_c
