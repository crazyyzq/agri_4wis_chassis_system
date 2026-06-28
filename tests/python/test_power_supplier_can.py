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
        "can_bus_service_pop_rx_frame",
        "CAN_BUS_RX_QUEUE_CAPACITY",
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
