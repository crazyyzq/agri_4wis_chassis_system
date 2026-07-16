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
        "#define ECU_DCDC12_DEFAULT_OUTPUT_VOLTAGE_DV (138U)",
        "#define ECU_DCDC12_DEFAULT_OUTPUT_CURRENT_DA (1000U)",
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


def test_power_accessories_wait_for_high_voltage_feedback(root: pathlib.Path) -> None:
    power_c = read(root, "ecu/devices/src/power_device.c")

    for token in [
        "power_device_high_voltage_feedback_ready",
        "snapshot->bms.positive_contactor_closed",
        "snapshot->bms.negative_contactor_closed",
        "bool accessory_enable =",
        "high_voltage_enable && high_voltage_feedback_ready",
        "power_device_send_bms_if_due(state, can1, high_voltage_enable",
        "power_device_send_dcdc12_if_due(state, can1, accessory_enable",
        "power_device_send_dcac_if_due(state, can1, accessory_enable",
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


def test_power_can_online_uses_current_bus_state_not_cumulative_errors(root: pathlib.Path) -> None:
    power_c = read(root, "ecu/devices/src/power_device.c")

    assert "snapshot->can1_online = can1 != 0 && can1->online;" in power_c
    assert "snapshot->can1_online = can1 != 0 && can1->online && can1->error_count == 0U;" not in power_c


def test_remote_power_request_does_not_require_outputs_that_it_starts(root: pathlib.Path) -> None:
    power_fsm_c = read(root, "ecu/remote/src/remote_power_fsm.c")

    precondition_body = power_fsm_c.split(
        "static uint16_t power_on_precondition_block_mask", 1
    )[1]
    precondition_body = precondition_body.split("static bool power_down_preconditions_ok", 1)[0]

    assert "preconditions->can1_power_online" in precondition_body
    assert "preconditions->power_ready" not in precondition_body
    assert "preconditions->low_voltage_ok" not in precondition_body


def test_remote_power_rejection_exposes_each_startup_interlock(root: pathlib.Path) -> None:
    power_fsm_h = read(root, "ecu/remote/include/remote_power_fsm.h")
    power_fsm_c = read(root, "ecu/remote/src/remote_power_fsm.c")
    monitor_c = read(root, "ecu/diag/src/runtime_monitor.c")

    for token in [
        "REMOTE_POWER_BLOCK_GEAR_NOT_P",
        "REMOTE_POWER_BLOCK_SPEED_NOT_ZERO",
        "REMOTE_POWER_BLOCK_THROTTLE_NOT_LOW",
        "REMOTE_POWER_BLOCK_STEERING_NOT_CENTER",
        "REMOTE_POWER_BLOCK_ARM_NOT_READY",
        "REMOTE_POWER_BLOCK_ESTOP_LATCHED",
        "REMOTE_POWER_BLOCK_A_CLASS_FAULT",
        "REMOTE_POWER_BLOCK_CAN1_POWER_OFFLINE",
    ]:
        assert token in power_fsm_h
        assert token in power_fsm_c

    assert "fsm->power_on_block_mask == 0U" in power_fsm_c
    assert "pwr_blk=0x%02x" in monitor_c
    assert "pwr_ch4=%u" in monitor_c


def test_remote_power_on_latch_does_not_chatter_on_post_hv_tpdo_gap(root: pathlib.Path) -> None:
    power_fsm_c = read(root, "ecu/remote/src/remote_power_fsm.c")

    high_block = power_fsm_c.split("if (input->power == REMOTE_POS_HIGH)", 1)[1]
    high_block = high_block.split("} else {", 1)[0]

    assert "if (fsm->high_voltage_enable_request)" in high_block
    assert "fsm->state = REMOTE_POWER_ON;" in high_block
    assert "return;" in high_block
    assert high_block.index("if (fsm->high_voltage_enable_request)") < high_block.index(
        "fsm->power_on_block_mask == 0U"
    )


def test_estop_preserves_hv_latch_until_explicit_safe_power_down(root: pathlib.Path) -> None:
    power_fsm_c = read(root, "ecu/remote/src/remote_power_fsm.c")
    protect_block = power_fsm_c.split(
        "if (preconditions->estop_latched || preconditions->a_class_fault)", 1
    )[1].split("if (input->power != REMOTE_POS_HIGH", 1)[0]

    assert "REMOTE_POWER_SHUTDOWN_PROTECT" in protect_block
    assert "fsm->high_voltage_disable_request = false;" in protect_block
    assert "fsm->high_voltage_enable_request = false;" not in protect_block


def test_remote_power_request_timing_matches_field_enable_gesture(root: pathlib.Path) -> None:
    config_h = read(root, "ecu/config/include/ecu_config.h")
    config_c = read(root, "ecu/config/src/ecu_config.c")
    power_fsm_h = read(root, "ecu/remote/include/remote_power_fsm.h")

    assert "#define REMOTE_POWER_LONG_PRESS_MS       (350U)" in config_h
    assert "ECU_SBUS_PPM_HIGH_MIN            (1800U)" in config_h
    assert ".power_long_press_ms = REMOTE_POWER_LONG_PRESS_MS" in config_c
    assert "350 ms" in power_fsm_h
    assert "ECU_SBUS_PPM_HIGH_MIN..ECU_SBUS_PPM_HIGH" in power_fsm_h


def test_remote_power_down_waits_for_safe_preconditions_before_clearing_hv(root: pathlib.Path) -> None:
    power_fsm_c = read(root, "ecu/remote/src/remote_power_fsm.c")
    power_fsm_h = read(root, "ecu/remote/include/remote_power_fsm.h")
    remote_manager_c = read(root, "ecu/remote/src/remote_manager.c")

    power_down_fn = power_fsm_c.split("static void request_safe_power_down", 1)[1]
    power_down_fn = power_down_fn.split("void remote_power_fsm_init", 1)[0]
    assert "fsm->high_voltage_enable_request = false;" in power_down_fn
    assert "fsm->high_voltage_disable_request = true;" in power_down_fn
    safe_check = power_down_fn.index("if (power_down_preconditions_ok(preconditions))")
    assert safe_check < power_down_fn.index("fsm->high_voltage_enable_request = false;")
    assert safe_check < power_down_fn.index("fsm->high_voltage_disable_request = true;")
    assert "request_safe_power_down(fsm, preconditions);" in power_fsm_c
    assert "bool high_voltage_disable_request;" in power_fsm_h
    assert "manager->request.high_voltage_disable_request = manager->power.high_voltage_disable_request" in remote_manager_c


def test_remote_power_center_releases_shutdown_protect_after_fault_clears(root: pathlib.Path) -> None:
    power_fsm_c = read(root, "ecu/remote/src/remote_power_fsm.c")

    neutral_block = power_fsm_c.split(
        "if (input->power != REMOTE_POS_HIGH && input->power != REMOTE_POS_LOW)", 1
    )[1].split("if (input->power != fsm->hold_position)", 1)[0]

    assert (
        "fsm->state = fsm->high_voltage_enable_request ? REMOTE_POWER_ON : REMOTE_POWER_OFF;"
        in neutral_block
    )
