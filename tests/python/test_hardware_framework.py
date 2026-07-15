"""Hardware binding framework contract tests."""

from __future__ import annotations

import pathlib
import re


def read(root: pathlib.Path, rel: str) -> str:
    path = root / rel
    assert path.exists(), f"missing {rel}"
    return path.read_text(encoding="utf-8")


def test_hardware_project_defaults_are_centralized(root: pathlib.Path) -> None:
    config_h = read(root, "ecu/config/include/ecu_config.h")
    config_c = read(root, "ecu/config/src/ecu_config.c")
    required = [
        "ECU_CAN1_POWER_BITRATE",
        "ECU_CAN2_MOTION_BITRATE",
        "ECU_CAN3_LIFT_HYDRAULIC_BITRATE",
        "ECU_CAN4_AUXILIARY_BITRATE",
        "ECU_POWER_BMS_COMMAND_PERIOD_MS",
        "ECU_POWER_DCDC48_COMMAND_PERIOD_MS",
        "ECU_POWER_DCDC12_COMMAND_PERIOD_MS",
        "ECU_POWER_DCAC_COMMAND_PERIOD_MS",
        "ECU_CANOPEN_DRIVE_FR_NODE_ID",
        "ECU_CANOPEN_STEER_FR_NODE_ID",
        "ECU_DIO_BRAKE_RELEASE_MASK",
        "ECU_DIO_HYDRAULIC_ENABLE_MASK",
        "ECU_DIO_MANAGED_OUTPUT_MASK",
        "ECU_HYD_VALVE_MANAGED_MASK",
        "ECU_ADC_EXTERNAL_MV_MAX",
        "ECU_MODBUS_ADC_SLAVE_ID",
        "ecu_hardware_config_default",
    ]
    for token in required:
        assert token in config_h or token in config_c, token


def test_modbus_adc_uses_0_to_10v_sensor_scale_and_named_channels(root: pathlib.Path) -> None:
    config_h = read(root, "ecu/config/include/ecu_config.h")
    analog_c = read(root, "ecu/devices/src/analog_modbus_device.c")

    assert "#define ECU_ADC_EXTERNAL_MV_MAX     (10000U)" in config_h
    assert "#define ECU_MODBUS_ADC_RAW_MAX      (65535U)" in config_h
    for token in [
        "ECU_ANALOG_SENSOR_FRONT_SUSPENSION_ANGLE_CH",
        "ECU_ANALOG_SENSOR_REAR_SUSPENSION_ANGLE_CH",
        "ECU_ANALOG_SENSOR_LEG1_TRACK_CYLINDER_CH",
        "ECU_ANALOG_SENSOR_LEG2_TRACK_CYLINDER_CH",
        "ECU_ANALOG_SENSOR_LEG3_TRACK_CYLINDER_CH",
        "ECU_ANALOG_SENSOR_LEG4_TRACK_CYLINDER_CH",
        "ECU_ANALOG_SENSOR_RESERVED6_CH",
        "ECU_ANALOG_SENSOR_RESERVED7_CH",
    ]:
        assert token in config_h
    assert "scale_raw_to_millivolt" in analog_c
    assert "adc_external_mv_max" in analog_c
    assert "modbus_adc_raw_max" in analog_c


def test_steering_remote_follow_uses_latest_target_trajectory_not_raw_jitter(root: pathlib.Path) -> None:
    config_h = read(root, "ecu/config/include/ecu_config.h")
    motion_h = read(root, "ecu/devices/include/motion_device.h")
    motion_c = read(root, "ecu/devices/src/motion_device.c")
    shaper_h = read(root, "ecu/control/include/motion_setpoint_shaper.h")
    shaper_c = read(root, "ecu/control/src/motion_setpoint_shaper.c")

    for token in [
        "steer_latest_target_counts",
        "steer_commanded_target_counts",
        "steer_commanded_velocity_counts_per_sec",
        "steer_group_commanded_speed_counts_per_sec",
        "motion_setpoint_shape_steering_group",
    ]:
        assert token in motion_h or token in motion_c or token in shaper_h or token in shaper_c, token
    for token in [
        "ECU_CANOPEN_STEER_TARGET_ACCEL_FINE_COUNTS_PER_SEC2",
        "ECU_CANOPEN_STEER_TARGET_ACCEL_SMALL_COUNTS_PER_SEC2",
        "ECU_CANOPEN_STEER_TARGET_ACCEL_MEDIUM_COUNTS_PER_SEC2",
        "ECU_CANOPEN_STEER_TARGET_ACCEL_LARGE_COUNTS_PER_SEC2",
        "ECU_CANOPEN_STEER_TARGET_DECEL_COUNTS_PER_SEC2",
        "ECU_CANOPEN_STEER_TARGET_REVERSAL_DECEL_COUNTS_PER_SEC2",
        "ECU_CANOPEN_STEER_TARGET_VELOCITY_SETTLE_COUNTS_PER_SEC",
        "ECU_CANOPEN_STEER_TARGET_RATE_LARGE_COUNTS_PER_SEC         (500000)",
    ]:
        assert token in config_h, token
    cache_fn = motion_c.split("static bool cache_latest_steer_target", 1)[1].split(
        "static bool prepare_steer_axis_once", 1
    )[0]
    assert "state->steer_latest_target_counts[wheel] = position_counts;" in cache_fn
    assert cache_fn.index("state->steer_latest_target_counts[wheel] = position_counts;") < cache_fn.index(
        "state->steer_pending_target[wheel] = true;"
    )
    assert "current_velocity_counts_per_sec" in shaper_c
    assert "output_velocity_counts_per_sec" in shaper_c
    assert "approach_nonnegative" not in shaper_c
    assert "*output_group_speed_counts_per_sec = 0;" not in shaper_c
    assert "bool axis_settled" in shaper_c
    assert "band == MOTION_STEER_FOLLOW_BAND_HOLD" not in shaper_c
    steer_gate_fn = motion_c.split(
        "static bool motion_device_update_steer_safety_gate", 1
    )[1].split("static bool command_changed", 1)[0]
    assert "state->steer_commanded_velocity_counts_per_sec[wheel] = 0" in steer_gate_fn


def test_can2_drive_velocity_does_not_queue_conflicting_group_while_steering_active(root: pathlib.Path) -> None:
    motion_c = read(root, "ecu/devices/src/motion_device.c")

    assert "canopen_pdo_lane_busy_for_other_group" in motion_c


def test_can2_motion_pdo_requires_high_voltage_feedback(root: pathlib.Path) -> None:
    """Unpowered drives must not receive an endless queue/cancel stream."""

    motion_c = read(root, "ecu/devices/src/motion_device.c")
    ready_fn = motion_c.split("static bool can2_motion_high_voltage_ready", 1)[1].split(
        "static bool can2_realtime_motion_state_needs_recovery", 1
    )[0]
    assert "state->last_motion_command.high_voltage_enable" in ready_fn
    assert "state->last_motion_command.high_voltage_feedback_ready" in ready_fn
    assert "!state->last_motion_command.high_voltage_disable_request" in ready_fn
    flush_fn = motion_c.split("ecu_device_apply_result_t motion_device_flush_realtime", 1)[1]
    assert "if (!can2_motion_high_voltage_ready(state))" in flush_fn
    high_voltage_gate = flush_fn.split(
        "if (!can2_motion_high_voltage_ready(state))", 1
    )[1].split("send_can2_feedback_sync_if_due", 1)[0]
    assert "return ECU_DEVICE_APPLY_OK" in high_voltage_gate
    apply_fn = motion_c.split("ecu_device_apply_result_t motion_device_apply", 1)[1].split(
        "ecu_device_apply_result_t motion_device_flush_realtime", 1
    )[0]
    assert (
        "state->last_motion_command.high_voltage_enable =\n"
        "        command->high_voltage_enable"
    ) in apply_fn
    assert (
        "state->last_motion_command.high_voltage_feedback_ready =\n"
        "        command->high_voltage_feedback_ready"
    ) in apply_fn
    drive_flush = motion_c.split("static ecu_device_apply_result_t flush_drive_velocity_realtime", 1)[1].split(
        "static void finish_completed_steer_group", 1
    )[0]
    assert "canopen_pdo_lane_busy_for_other_group(canopen," in drive_flush
    assert "state->drive_next_group_valid = true;" in drive_flush
    assert "return ECU_DEVICE_APPLY_OK;" in drive_flush


def test_unverified_steering_limit_inputs_do_not_block_right_push_by_default(root: pathlib.Path) -> None:
    config_h = read(root, "ecu/config/include/ecu_config.h")
    motion_c = read(root, "ecu/devices/src/motion_device.c")

    assert "#define ECU_CANOPEN_STEER_LIMIT_INPUT_GATING_ENABLED (0)" in config_h
    limit_fn = motion_c.split("static bool steer_limit_blocks_target", 1)[1].split(
        "static int32_t normalize_steer_target_counts", 1
    )[0]
    assert "!ECU_CANOPEN_STEER_LIMIT_INPUT_GATING_ENABLED" in limit_fn
    assert "return false;" in limit_fn


def test_no_informal_uncertainty_marker_in_code_or_tests(root: pathlib.Path) -> None:
    forbidden = "GU" + "ESS"
    lower_forbidden = "gu" + "ess"
    scanned_roots = [root / "ecu", root / "tests"]
    for folder in scanned_roots:
        for path in folder.rglob("*"):
            if not path.is_file():
                continue
            rel = path.relative_to(root).as_posix()
            if rel.startswith("ecu/sdk_env_v1.11.0/"):
                continue
            if rel.startswith("ecu/apps/agri_chassis_control_cpu0/build/"):
                continue
            if path.suffix.lower() not in {".c", ".h", ".py", ".md", ".cmake", ".txt"}:
                continue
            text = path.read_text(encoding="utf-8", errors="ignore")
            assert forbidden not in text, f"{rel}: remove informal uncertainty marker"
            assert lower_forbidden not in text.lower(), f"{rel}: remove informal uncertainty wording"


def test_configuration_open_items_document_exists(root: pathlib.Path) -> None:
    text = read(root, "docs/ecu-configuration-open-items.md")

    assert "CAN1 power bus" in text
    assert "250 kbit/s" in text
    assert "bit time 4 us" in text
    assert "BMS" in text
    assert "CAN2 motion bus" in text


def test_protocol_driver_and_device_layers_exist(root: pathlib.Path) -> None:
    required_files = [
        "ecu/drivers/can/include/can_bus_service.h",
        "ecu/drivers/can/src/can_bus_service.c",
        "ecu/drivers/canopen/include/canopen_master_service.h",
        "ecu/drivers/canopen/src/canopen_master_service.c",
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


def test_communication_stacks_use_sdk_middleware(root: pathlib.Path) -> None:
    cmake = read(root, "ecu/apps/agri_chassis_control_cpu0/CMakeLists.txt")
    user_config_h = read(root, "ecu/apps/agri_chassis_control_cpu0/src/user_config.h")
    sbus_h = read(root, "ecu/protocol/sbus/include/sbus_decoder.h")

    for token in [
        "set(CONFIG_AGILE_MODBUS 1)",
        "set(CONFIG_AGILE_MODBUS_RTU 1)",
        "set(CONFIG_CANOPEN 1)",
    ]:
        assert token in cmake, token

    assert "MAX_CANOPEN_DEVICE (2U)" in user_config_h
    assert "sbus_decode_frame" in sbus_h and "SBUS_CHANNEL_COUNT" in sbus_h


def test_canopen_and_modbus_protocols_are_library_backed(root: pathlib.Path) -> None:
    """Active ECU code must use CANopenNode and Agile Modbus, not local stacks."""

    ignored_prefixes = (
        "ecu/sdk_env_v1.11.0/",
        "ecu/apps/agri_chassis_control_cpu0/build/",
    )
    forbidden_patterns = [
        re.compile("can" + "open_frame"),
        re.compile(r'(?<!agile_)mod' + r'bus_rtu'),
    ]
    scanned_roots = [root / "ecu", root / "tests", root / "docs"]
    for folder in scanned_roots:
        for path in folder.rglob("*"):
            if not path.is_file():
                continue
            rel = path.relative_to(root).as_posix()
            if rel.startswith(ignored_prefixes):
                continue
            if path.suffix.lower() not in {".c", ".h", ".py", ".md", ".cmake", ".txt"}:
                continue
            text = path.read_text(encoding="utf-8", errors="ignore")
            for pattern in forbidden_patterns:
                match = pattern.search(text)
                assert match is None, f"{rel}: remove project-local protocol token {match.group(0)}"

    cmake = read(root, "ecu/apps/agri_chassis_control_cpu0/CMakeLists.txt")
    servo_c = read(root, "ecu/devices/src/servo_drive_canopen.c")
    analog_c = read(root, "ecu/devices/src/analog_modbus_device.c")
    warning_c = read(root, "ecu/devices/src/warning_light_device.c")
    service_c = read(root, "ecu/drivers/canopen/src/canopen_master_service.c")

    assert "sdk_app_src(../../drivers/canopen/src/canopen_master_service.c)" in cmake
    assert "sdk_app_src(../../protocol/canopen/src/" not in cmake
    assert "sdk_app_src(../../protocol/modbus/src/" not in cmake
    assert "canopen_master_service_request_sdo_write" in servo_c
    assert "agile_modbus_serialize_read_input_registers" in analog_c
    assert "agile_modbus_deserialize_read_input_registers" in analog_c
    assert "agile_modbus_serialize_write_register" in warning_c
    assert "CO_SDOclientDownloadInitiate" in service_c


def test_canopennode_ds301_od_and_build_switch(root: pathlib.Path) -> None:
    cmake = read(root, "ecu/apps/agri_chassis_control_cpu0/CMakeLists.txt")
    user_config_h = read(root, "ecu/apps/agri_chassis_control_cpu0/src/user_config.h")
    config_h = read(root, "ecu/config/include/ecu_config.h")
    od_h = read(root, "ecu/protocol/canopen/od/ds301/OD.h")
    od_c = read(root, "ecu/protocol/canopen/od/ds301/OD.c")

    assert "CANopenNode V4" in od_h
    assert "OD_PERSIST_COMM" in od_c
    assert "sdk_inc(../../protocol/canopen/od/ds301)" in cmake
    assert "sdk_app_src(../../protocol/canopen/od/ds301/OD.c)" in cmake
    assert "sdk_compile_definitions(-DECU_ENABLE_CANOPENNODE=1)" in cmake
    assert "sdk_compile_definitions(-DCONFIG_CANOPEN_MASTER=1)" in cmake
    assert 'sdk_compile_options("-Wno-unused-parameter")' in cmake
    assert "-Wno-macro-redefined" not in cmake
    assert 'target_include_directories(${HPM_SDK_LIB_ITF} BEFORE INTERFACE "${CMAKE_CURRENT_LIST_DIR}/src")' in cmake
    assert "sdk_app_src(../../drivers/canopen/src/canopen_master_service.c)" in cmake
    canopen_errno_h = read(root, "ecu/apps/agri_chassis_control_cpu0/src/canopen_errno.h")
    assert "#ifndef EIO" in canopen_errno_h
    assert "#define EIO 5" in canopen_errno_h
    assert "#ifndef EMSGSIZE" in canopen_errno_h
    assert "#define EMSGSIZE 122" in canopen_errno_h
    assert "hpm_sdk_errno.h" not in canopen_errno_h
    assert "without editing the ignored SDK environment" in canopen_errno_h
    assert "MAX_CANOPEN_DEVICE (2U)" in user_config_h
    for token in [
        "ECU_CANOPEN_BC2_DIAG_NODE_ID",
        "ECU_CANOPEN_OBJ_DEVICE_TYPE",
        "ECU_CANOPEN_OBJ_ERROR_REGISTER",
        "ECU_CANOPEN_OBJ_IDENTITY",
        "ECU_CANOPEN_OBJ_STATUSWORD",
        "ECU_CANOPEN_OBJ_MODES_OF_OPERATION_DISPLAY",
    ]:
        assert token in config_h, token


def test_canopen_master_build_disables_unused_local_rpdo_consumers(root: pathlib.Path) -> None:
    """The ECU is a PDO producer for remote drives, not a local RPDO consumer."""

    cmake = read(root, "ecu/apps/agri_chassis_control_cpu0/CMakeLists.txt")
    od_h = read(root, "ecu/protocol/canopen/od/ds301/OD.h")

    assert "#define OD_CNT_RPDO 0" in od_h
    assert "ECU_CANOPEN_MASTER_PDO_CONFIG" in cmake
    assert "CO_CONFIG_RPDO_ENABLE" not in cmake.split(
        "ECU_CANOPEN_MASTER_PDO_CONFIG", 1
    )[1].split("sdk_compile_definitions", 1)[0]


def test_canopennode_hpm_tx_path_is_nonblocking(root: pathlib.Path) -> None:
    cmake = read(root, "ecu/apps/agri_chassis_control_cpu0/CMakeLists.txt")
    wrapper_c = read(root, "ecu/drivers/canopen/src/hpm_can_send_nonblocking_wrap.c")
    sdk_hal_c = read(root, "ecu/sdk_env_v1.11.0/hpm_sdk/middleware/CANopenNode/hal/hpm_canopen_can.c")

    assert "sdk_ld_options(\"-Wl,--wrap=hpm_can_send\")" in cmake
    assert "sdk_app_src(../../drivers/canopen/src/hpm_can_send_nonblocking_wrap.c)" in cmake
    assert "__wrap_hpm_can_send" in wrapper_c
    assert "busy-waits forever" in wrapper_c
    assert "can_send_high_priority_message_nonblocking" in wrapper_c
    assert "can_send_message_nonblocking" not in wrapper_c
    assert "can_is_secondary_transmit_buffer_full" not in wrapper_c
    assert "reported as -EBUSY" in wrapper_c
    assert "printf(" not in wrapper_c
    assert "Transmit failed" not in wrapper_c
    assert "while (!data->has_sent_out)" not in wrapper_c
    assert "while (!data->has_sent_out" not in sdk_hal_c
    assert "CANOPEN_HPM_TX_WAIT_LIMIT" in sdk_hal_c
    assert "CANOPEN_HPM_IRQ_RX_DRAIN_LIMIT" in sdk_hal_c
    assert "rx_drained < CANOPEN_HPM_IRQ_RX_DRAIN_LIMIT" in sdk_hal_c
    assert "CAN_EVENT_TX_SECONDARY_BUF" in sdk_hal_c.split("can_clear_tx_rx_flags(can,", 1)[1]


def test_canopennode_can2_diagnostic_service_is_safe(root: pathlib.Path) -> None:
    service_h = read(root, "ecu/drivers/canopen/include/canopen_master_service.h")
    service_c = read(root, "ecu/drivers/canopen/src/canopen_master_service.c")
    tasks_c = read(root, "ecu/os/src/ecu_tasks_cpu0.c")
    monitor_h = read(root, "ecu/diag/include/runtime_monitor.h")
    monitor_c = read(root, "ecu/diag/src/runtime_monitor.c")

    for token in [
        "canopen_master_service_init",
        "canopen_master_service_process",
        "canopen_master_service_get_snapshot",
        "canopen_master_snapshot_t",
        "last_sdo_index",
        "last_sdo_abort_code",
    ]:
        assert token in service_h, token

    for token in [
        "BOARD_CAN2_BASE",
        "BOARD_CAN2_IRQn",
        "canopen_controller_init",
        "CO_CANinit",
        "CO_CANopenInit",
        "CO_process",
        "CO_SDOclient_setup",
        "CO_SDOclientUploadInitiate",
        "CO_SDOclientUpload",
        "ECU_CANOPEN_OBJ_STATUSWORD",
        "ECU_CANOPEN_OBJ_MODES_OF_OPERATION_DISPLAY",
    ]:
        assert token in service_c, token

    assert "ATTR_PLACE_AT_NONCACHEABLE_BSS" in service_c
    assert "g_canopen_master_debug_control" in service_c
    assert service_c.index("handle_debug_command(service);") < service_c.index("if (service->sdo_download_active)")
    assert "canopen_master_service_init(&s_runtime.can2_motion_canopen" in tasks_c
    assert "canopen_master_service_init(&s_runtime.can3_lift_hydraulic_canopen" in tasks_c
    assert "canopen_master_service_process_realtime_pdo(&s_runtime.can2_motion_canopen" in tasks_c
    assert "canopen_master_service_process_background(&s_runtime.can2_motion_canopen" in tasks_c
    assert "canopen_master_service_process_realtime_pdo(&s_runtime.can3_lift_hydraulic_canopen" in tasks_c
    assert "canopen_master_service_process_background(&s_runtime.can3_lift_hydraulic_canopen" in tasks_c
    assert "can2_canopen_initialized" in monitor_h
    assert "ECU CANopen CAN2" in monitor_c


def test_modbus_virtual_adc_and_rs485_master_are_structured(root: pathlib.Path) -> None:
    virtual_adc = read(root, "tools/modbus/virtual_adc_module.py")
    rtu_codec = read(root, "tools/modbus/rtu_codec.py")
    tasks_c = read(root, "ecu/os/src/ecu_tasks_cpu0.c")
    analog_modbus_c = read(root, "ecu/devices/src/analog_modbus_device.c")
    rs485_c = read(root, "ecu/drivers/uart/src/uart_rs485_hw.c")
    board_h = read(root, "ecu/ecu_isolation/board.h")
    pinmux_c = read(root, "ecu/ecu_isolation/pinmux.c")
    monitor_h = read(root, "ecu/diag/include/runtime_monitor.h")
    monitor_c = read(root, "ecu/diag/src/runtime_monitor.c")
    config_h = read(root, "ecu/config/include/ecu_config.h")
    config_c = read(root, "ecu/config/src/ecu_config.c")

    for token in [
        "Read Input Registers",
        "registers 0..7",
        "--port",
        "--slave",
        "--baudrate",
        "--channels-mv",
        "pop_next_request_frame",
        "verify_crc(frame)",
    ]:
        assert token in virtual_adc, token

    for forbidden in ["write_register", "write_multiple_registers"]:
        assert forbidden not in virtual_adc
        assert forbidden not in rtu_codec

    for rel in [
        "ecu/drivers/uart/include/uart_rs485_hw.h",
        "ecu/drivers/uart/src/uart_rs485_hw.c",
        "ecu/drivers/uart/include/modbus_master_service.h",
        "ecu/drivers/uart/src/modbus_master_service.c",
        "ecu/devices/include/analog_modbus_device.h",
        "ecu/devices/src/analog_modbus_device.c",
    ]:
        assert (root / rel).exists(), rel

    for token in [
        "ECU_MODBUS_ADC_BAUDRATE",
        "ECU_MODBUS_ADC_START_REGISTER",
        "ECU_MODBUS_ADC_REGISTER_COUNT",
        "ECU_MODBUS_ADC_RAW_MAX",
        "ECU_MODBUS_WARNING_LIGHT_REGISTER",
    ]:
        assert token in config_h or token in config_c, token

    assert "analog_modbus_device_process" in tasks_c
    assert "modbus_master_service_process" in analog_modbus_c
    assert "BOARD_RS485_1_UART_BASE" in rs485_c
    assert "#define BOARD_RS485_DE_USING_GPIO 1" in board_h
    assert "IOC_PD18_FUNC_CTL_GPIO_D_18" in pinmux_c
    assert "uart_rs485_1_set_transmit_direction" in rs485_c
    assert "uart_rs485_1_set_receive_direction" in rs485_c
    assert "analog_modbus_adc" in monitor_h
    assert "ECU MODBUS ADC" in monitor_c


def test_analog_modbus_adc_marks_offline_on_master_timeout(root: pathlib.Path) -> None:
    analog_c = read(root, "ecu/devices/src/analog_modbus_device.c")

    assert "if (master_snapshot.timeout_count > previous_timeout_count)" in analog_c
    assert "analog_modbus_note_timeout(state, now_ms);" in analog_c.split(
        "if (master_snapshot.timeout_count > previous_timeout_count)", 1
    )[1]


def test_canopennode_debug_commands_are_sequence_gated(root: pathlib.Path) -> None:
    service_h = read(root, "ecu/drivers/canopen/include/canopen_master_service.h")
    service_c = read(root, "ecu/drivers/canopen/src/canopen_master_service.c")
    monitor_h = read(root, "ecu/diag/include/runtime_monitor.h")
    monitor_c = read(root, "ecu/diag/src/runtime_monitor.c")
    config_h = read(root, "ecu/config/include/ecu_config.h")

    for token in [
        "canopen_master_debug_control_t",
        "command_sequence",
        "CANOPEN_MASTER_DEBUG_COMMAND_NONE",
        "CANOPEN_MASTER_DEBUG_COMMAND_SDO_WRITE_CONTROLWORD",
        "CANOPEN_MASTER_DEBUG_COMMAND_SDO_WRITE_TARGET_VELOCITY",
        "CANOPEN_MASTER_DEBUG_COMMAND_NMT_OPERATIONAL",
    ]:
        assert token in service_h, token

    for token in [
        "ECU_CANOPEN_OBJ_CONTROLWORD",
        "ECU_CANOPEN_OBJ_MODES_OF_OPERATION",
        "ECU_CANOPEN_OBJ_TARGET_POSITION",
        "ECU_CANOPEN_OBJ_TARGET_VELOCITY",
        "ECU_CANOPEN_OBJ_COMMAND_CURRENT",
    ]:
        assert token in config_h, token

    for token in [
        "volatile canopen_master_debug_control_t g_canopen_master_debug_control",
        "CO_SDOclientDownloadInitiate",
        "CO_SDOclientDownload(",
        "CO_NMT_sendCommand",
    ]:
        assert token in service_c, token

    init_body = service_c.split("canopen_master_service_init")[1].split("return true")[0]
    assert "CANOPEN_MASTER_DEBUG_COMMAND_NONE" in init_body
    assert "CO_NMT_sendCommand" not in init_body
    assert "last_command_sequence" in service_h
    assert "canopen_command" in monitor_h
    assert "ECU CANopen CMD" in monitor_c


def test_python_can_analyzer_and_modbus_tools_are_safe_by_default(root: pathlib.Path) -> None:
    controlcan_py = read(root, "tools/can/controlcan.py")
    monitor_py = read(root, "tools/can/can2_monitor.py")
    motion_debug_py = read(root, "tools/canopen_motion_debug/motion8_remote_sim_debug.py")
    steer_zero_py = read(root, "tools/canopen_position_debug/steer4_zero_calibration_debug.py")
    steer_zero_doc = read(root, "doc/ECU/转向零点校准脚本说明.md")
    modbus_py = read(root, "tools/modbus/rtu_probe.py")

    for token in [
        "VCI_USBCAN2",
        "VCI_CAN_OBJ",
        "VCI_INIT_CONFIG",
        "VCI_OpenDevice",
        "VCI_InitCAN",
        "VCI_StartCAN",
        "VCI_Receive",
        "ControlCAN.dll",
    ]:
        assert token in controlcan_py, token

    assert "channel=1" in monitor_py
    assert "default=1000000" in monitor_py
    assert "receive" in monitor_py
    assert ".transmit(" not in monitor_py
    assert "--allow-motion" in motion_debug_py
    assert "--spin-deg" in motion_debug_py
    assert "--smooth-commissioning-scenario" in motion_debug_py
    assert "SERVO_PROFILE_ACCEL_LIMIT_COUNTS_PER_SEC2 = 500_000" in motion_debug_py
    assert "--profile-accel\", type=int, default=SERVO_PROFILE_ACCEL_LIMIT_COUNTS_PER_SEC2" in motion_debug_py
    assert "profile acceleration" in motion_debug_py
    assert "Node1..4 drive wheels" in motion_debug_py
    assert "Node5..8 steering axes" in motion_debug_py
    assert "CONTROL_DISABLE_VOLTAGE" in motion_debug_py
    assert "SYNC_COB_ID" in motion_debug_py
    for token in [
        "--allow-motion",
        "dry-run by default",
        "--stall-current-10ma",
        "--stall-immediate-current-10ma",
        "--stall-dwell-ms",
        "--limit-fast-velocity",
        "--limit-stage2-start-counts",
        "--return-timeout-action",
        "--set-current-position-zero",
        "--zero-object-index",
        "--save-drive-parameters",
        "CANOPEN_STORE_SAVE_SIGNATURE",
        "0x221C",
        "stop_all_axes",
        "best_effort_safe_shutdown",
        "require_fresh_feedback",
        "set_current_position_zero",
        "steer_zero_calibration.json",
        "midpoint_counts_by_node",
        "two_stage_limit_velocity_three_stage_midpoint_velocity_then_6064_zero",
        "0x6064",
    ]:
        assert token in steer_zero_py, token
    assert "--reset-at-midpoint-to-zero" not in steer_zero_py
    assert 'default="fail"' in steer_zero_py
    assert "NMT_COMMAND_RESET_NODE" not in steer_zero_py
    assert "write_home_offsets_607c" not in steer_zero_py
    assert "--write-home-offset-607c" not in steer_zero_py
    assert "requires --set-current-position-zero" in steer_zero_py
    for token in [
        "写 `0x6064:00 = 0`",
        "0x6064:00 = 0",
        "不采用 `0x607C home offset` 作为正式方案",
        "B1 为 CH10",
        "短按 3 次",
        "9A",
        "三段速度",
        "速度模式闭环回到各自中值位置",
        "0x1010:01 = 0x65766173",
    ]:
        assert token in steer_zero_doc, token
    assert "COM10" in modbus_py
    assert "serial.Serial" in modbus_py
    assert "read_holding_registers" in modbus_py
    assert "write_register" not in modbus_py


def test_steering_zero_calibration_uses_two_stage_limit_and_three_stage_return(root: pathlib.Path) -> None:
    config_h = read(root, "ecu/config/include/ecu_config.h")
    remote_h = read(root, "ecu/remote/include/remote_types.h")
    remote_c = read(root, "ecu/remote/src/remote_manager.c")
    remote_mapper_h = read(root, "ecu/remote/include/remote_sbus_mapper.h")
    remote_mapper_c = read(root, "ecu/remote/src/remote_sbus_mapper.c")
    arbiter_c = read(root, "ecu/vehicle/src/command_arbiter.c")
    motion_c = read(root, "ecu/devices/src/motion_device.c")
    motion_h = read(root, "ecu/devices/include/motion_device.h")

    for token in [
        "ECU_STEER_ZERO_SEARCH_FAST_VELOCITY_UNITS             (1600000)",
        "ECU_STEER_ZERO_SEARCH_SLOW_VELOCITY_UNITS             (1120000)",
        "ECU_STEER_ZERO_SEARCH_SLOWDOWN_ABS_COUNTS             (2400000)",
        "ECU_STEER_ZERO_STALL_CURRENT_10MA                     (750)",
        "ECU_STEER_ZERO_PROTECTION_CURRENT_10MA                (900)",
        "ECU_STEER_ZERO_STALL_DWELL_MS                         (150U)",
        "ECU_STEER_ZERO_MID_RETURN_FAST_VELOCITY_UNITS         (1600000)",
        "ECU_STEER_ZERO_MID_RETURN_MEDIUM_VELOCITY_UNITS       (700000)",
        "ECU_STEER_ZERO_MID_RETURN_SLOW_VELOCITY_UNITS         (180000)",
        "ECU_STEER_ZERO_MID_RETURN_MEDIUM_ERROR_COUNTS         (500000)",
        "ECU_STEER_ZERO_MID_RETURN_SLOW_ERROR_COUNTS           (50000)",
        "ECU_STEER_ZERO_MIDPOINT_TOLERANCE_COUNTS              (10000)",
        "ECU_STEER_ZERO_LEG1_LEFT_SIGN                         (1)",
        "ECU_STEER_ZERO_LEG2_LEFT_SIGN                         (-1)",
        "ECU_STEER_ZERO_LEG3_LEFT_SIGN                         (1)",
        "ECU_STEER_ZERO_LEG4_LEFT_SIGN                         (-1)",
        "ECU_REMOTE_B1_ZERO_CALIBRATION_REQUEST_HOLD_MS        (600U)",
        "ECU_STEER_ZERO_VELOCITY_PDO_PERIOD_MS                 (20U)",
        "ECU_STEER_ZERO_PHASE_SETTLE_MS                        (50U)",
        "ECU_STEER_ZERO_SEARCH_TIMEOUT_MS                      (60000U)",
        "ECU_STEER_ZERO_SDO_TIMEOUT_MS                         (1000U)",
        "ECU_STEER_ZERO_SETUP_SDO_MAX_RETRIES                  (3U)",
    ]:
        assert token in config_h, token
    assert "ECU_REMOTE_B1_EDGE_DEBOUNCE_MS    (20U)" in config_h
    assert "remote_discrete_channel_t b1_button;" in remote_mapper_h
    assert "b1_transition_from_channel" in remote_mapper_c
    assert "candidate != REMOTE_POS_LOW && candidate != REMOTE_POS_HIGH" in remote_mapper_c
    assert "ECU_REMOTE_B1_EDGE_DEBOUNCE_MS" in remote_mapper_c
    assert "ECU_STEER_ZERO_STALL_IMMEDIATE_CURRENT_10MA           (1500)" not in config_h

    assert "bool steer_zero_calibration_request;" in remote_h
    assert "bool b1_zero_calibration_raw_request;" in remote_h
    assert "remote_manager_update_b1_zero_calibration_request" in remote_c
    assert "ECU_REMOTE_B1_ZERO_CALIBRATION_PRESS_COUNT" in remote_c
    assert "Count every debounced CH10 state transition as one short press" in remote_c
    assert "manager->b1_zero_calibration_press_latched = input->b1_changed" in remote_c
    assert "b1_zero_calibration_input_initialized" in remote_c
    assert "That synchronization sample" in remote_c
    assert "if (!gate_open)" in remote_c
    assert "steer_zero_calibration_request_hold_active" in remote_c
    assert "ECU_REMOTE_B1_ZERO_CALIBRATION_REQUEST_HOLD_MS" in remote_c
    assert "b1_zero_calibration_press_latched" in remote_c
    zero_gate = remote_c.split(
        "manager->request.steer_zero_calibration_request", 1
    )[0].split("bool steer_zero_calibration_gate_open", 1)[1]
    assert "input->home == REMOTE_POS_CENTER" in zero_gate
    assert "input->home != REMOTE_POS_CENTER" not in remote_c.split(
        "manager->request.steer_zero_calibration_request", 1
    )[1].split("manager->request.orderly_shutdown_request", 1)[0]
    assert "remote->steer_zero_calibration_request" in arbiter_c
    assert "command->steer_zero_calibration_request" in motion_c
    assert "steer_zero_calibration_requested = true" in motion_c
    assert "bool steer_zero_calibration_domain_active;" in read(
        root, "ecu/vehicle/include/vehicle_types.h"
    )
    assert "MOTION_STEER_ZERO_CAL_ABORTING" in motion_h
    assert "steer_zero_begin_abort" in motion_c
    assert "steer_zero_abort_step" in motion_c
    assert "canopen_master_service_request_calibration_position_restore" in motion_c
    assert "steer_profile_setup_reset(state);" in motion_c
    for token in [
        "MOTION_STEER_ZERO_CAL_SETUP",
        "MOTION_STEER_ZERO_CAL_SEARCH_LEFT",
        "MOTION_STEER_ZERO_CAL_SEARCH_RIGHT",
        "MOTION_STEER_ZERO_CAL_RETURN_MID",
        "MOTION_STEER_ZERO_CAL_WRITE_ZERO",
        "MOTION_STEER_ZERO_CAL_VERIFY_ZERO",
        "MOTION_STEER_ZERO_CAL_FAULT",
    ]:
        assert token in motion_h, token
    for token in [
        "steer_zero_calibration_step",
        "ECU_CANOPEN_OBJ_ACTUAL_POSITION",
        "ECU_CANOPEN_OBJ_BC_INTERPOLATION_OPTION",
        "ECU_CANOPEN_OBJ_COMMAND_CURRENT_RAMP",
        "canopen_master_service_request_sdo_write",
        "build_steer_zero_velocity_rpdo_request",
        "ECU_STEER_ZERO_PROTECTION_CURRENT_10MA",
        "ECU_STEER_ZERO_STALL_DWELL_MS",
        "ECU_CANOPEN_OBJ_FAULT_LATCHED",
        "steer_zero_left_direction_sign",
        "steer_zero_prepare_velocity_phase",
        "steer_zero_calibration_return_last_error_counts",
        "steer_zero_verify_zero",
        "steer_zero_calibration_prepare_zero_queued",
        "steer_zero_calibration_verify_sync_sent",
    ]:
        assert token in motion_c, token
    assert "NMT_COMMAND_RESET_NODE" not in motion_c
    assert "0x607C" not in motion_c
    return_mid = motion_c.split("static bool steer_zero_run_return_mid", 1)[1].split(
        "static bool steer_zero_write_zero_step", 1
    )[0]
    assert "Never redefine an arbitrary timeout position" in return_mid
    assert "steer_zero_calibration_midpoint_counts[wheel] =" not in return_mid
    assert "ECU_STEER_ZERO_STALL_ARM_DELAY_MS" in motion_c
    assert "axis_stopped_now" in motion_c
    verify_zero = motion_c.split("static bool steer_zero_verify_zero", 1)[1].split(
        "static ecu_device_apply_result_t steer_zero_calibration_step", 1
    )[0]
    assert "steer_zero_queue_velocity_group" in verify_zero
    write_zero = motion_c.split("static bool steer_zero_write_zero_step", 1)[1].split(
        "static bool steer_zero_verify_zero", 1
    )[0]
    assert "SERVO_DRIVE_CONTROL_FAULT_RESET" not in write_zero

    canopen_service_c = read(
        root, "ecu/drivers/canopen/src/canopen_master_service.c"
    )
    allowed_sdo = canopen_service_c.split(
        "static bool canopen_master_sdo_write_allowed", 1
    )[1].split("static bool canopen_master_sdo_write_requires_order", 1)[0]
    assert "ECU_CANOPEN_OBJ_COMMAND_CURRENT_RAMP" in allowed_sdo
    assert "A failure here is therefore a local" in motion_c
    setup_step = motion_c.split("static bool steer_zero_setup_step", 1)[1].split(
        "static bool steer_zero_queue_velocity_group", 1
    )[0]
    assert "steer_zero_calibration_setup_sdo_retry_count" in setup_step
    assert "ECU_STEER_ZERO_SETUP_SDO_MAX_RETRIES" in setup_step
    assert setup_step.index("download_completed") < setup_step.index(
        "steer_zero_calibration_setup_step++"
    )
    queue_tail = setup_step.split("steer_zero_queue_sdo", 1)[1]
    assert "steer_zero_calibration_setup_step++" not in queue_tail

    tasks_c = read(root, "ecu/os/src/ecu_tasks_cpu0.c")
    can2_task = tasks_c.split("void ecu_task_can2_motion_step", 1)[1].split(
        "void ecu_task_remote_manager_step", 1
    )[0]
    assert "MOTION_STEER_ZERO_CAL_IDLE" in can2_task
    assert "!zero_calibration_active" in can2_task
    assert can2_task.index("canopen_master_service_set_periodic_sdo_enabled") < (
        can2_task.index("canopen_master_service_process_background")
    )


def test_servo_drive_adapter_is_device_level_and_cmake_owned(root: pathlib.Path) -> None:
    servo_h = read(root, "ecu/devices/include/servo_drive_canopen.h")
    servo_c = read(root, "ecu/devices/src/servo_drive_canopen.c")
    motion_c = read(root, "ecu/devices/src/motion_device.c")
    lift_c = read(root, "ecu/devices/src/lift_hydraulic_device.c")
    config_h = read(root, "ecu/config/include/ecu_config.h")
    cmake = read(root, "ecu/apps/agri_chassis_control_cpu0/CMakeLists.txt")

    for token in [
        "servo_drive_canopen_send_control_word",
        "servo_drive_canopen_select_mode",
        "servo_drive_canopen_set_target_position",
        "servo_drive_canopen_set_target_velocity",
        "servo_drive_canopen_set_target_torque",
        "servo_drive_canopen_run_velocity_mode",
        "servo_drive_canopen_run_absolute_position_mode",
        "SERVO_DRIVE_CONTROL_ENABLE_OPERATION",
        "SERVO_DRIVE_MODE_PROFILE_POSITION",
        "SERVO_DRIVE_MODE_PROFILE_VELOCITY",
    ]:
        assert token in servo_h or token in servo_c, token

    assert "servo_drive_canopen.c" in cmake
    assert "canopen_pdo_profile.h" in motion_c
    assert "canopen_pdo_build_position_rpdo1" in motion_c
    assert "servo_drive_canopen_configure_steer_rpdo(canopen" not in motion_c
    assert "servo_drive_canopen_prepare_position_mode(canopen" not in motion_c
    assert "canopen_master_service_queue_pdo" in motion_c
    assert "servo_drive_canopen_run_absolute_position_mode" not in lift_c
    assert "ECU_CANOPEN_RPDO2_BASE" in config_h
    assert "ECU_DRIVE_SPEED_MPS_TO_COUNTS_PER_SEC" in config_h
    assert "ECU_STEER_DEG_TO_COUNTS" in config_h
    assert "ECU_LIFT_MM_TO_COUNTS" in config_h
    assert "#define ECU_LIFT_ENCODER_COUNTS_PER_REV              (131072.0f)" in config_h
    assert "#define ECU_LIFT_MOTOR_REVS_PER_MM                   (2.0f)" in config_h
    assert "#define ECU_REMOTE_MAX_HEIGHT_RATE_MM_S   (6.0f)" in config_h
    assert "ECU_LIFT_ENCODER_COUNTS_PER_REV * ECU_LIFT_MOTOR_REVS_PER_MM" in config_h
    assert "ECU_LIFT_PROFILE_VELOCITY_UNITS_PER_MM_S" not in config_h
    assert "ECU_LIFT_PROFILE_ACCEL_UNITS_PER_RPS2" not in config_h


def test_encoder_count_contracts_are_explicit_per_node_role(root: pathlib.Path) -> None:
    """Node1-8/13 use 10000 counts/rev; lift Node9-12 use 131072."""

    config_h = read(root, "ecu/config/include/ecu_config.h")
    tasks_c = read(root, "ecu/os/src/ecu_tasks_cpu0.c")
    lift_c = read(root, "ecu/devices/src/lift_hydraulic_device.c")

    for token in [
        "#define ECU_CAN2_MOTION_ENCODER_COUNTS_PER_REV          (10000.0f)",
        "#define ECU_HYDRAULIC_PUMP_ENCODER_COUNTS_PER_REV       (10000.0f)",
        "#define ECU_LIFT_ENCODER_COUNTS_PER_REV              (131072.0f)",
        "ECU_CAN2_MOTION_ENCODER_COUNTS_PER_REV * ECU_STEER_GEAR_REDUCTION",
        "ECU_CAN2_MOTION_ENCODER_COUNTS_PER_REV * ECU_DRIVE_GEAR_REDUCTION",
        "ECU_HYDRAULIC_PUMP_VELOCITY_UNITS_PER_RPM",
        "ECU_LIFT_ENCODER_COUNTS_PER_REV * ECU_LIFT_MOTOR_REVS_PER_MM",
    ]:
        assert token in config_h, token

    assert "ECU_BC_SERVO_ENCODER_COUNTS_PER_REV" not in config_h
    assert "ECU_LIFT_POSITION_SPEED_UNITS" not in config_h
    assert "ECU_CAN2_ZERO_SPEED_VELOCITY_UNITS" in tasks_c
    assert "ECU_HYDRAULIC_PUMP_ZERO_SPEED_VELOCITY_UNITS" in lift_c
    assert "ECU_CAN2_ZERO_SPEED_VELOCITY_UNITS" not in lift_c


def test_lift_commissioning_tools_match_installed_mechanical_scale(
    root: pathlib.Path,
) -> None:
    """Dangerous bench tools must not retain the superseded 12-rev ratio."""

    sync_tool = read(root, "tools/canopen_interp_debug/lift4_sync_debug.py")
    velocity_tool = read(
        root, "tools/canopen_interp_debug/lift4_velocity_sync_debug.py"
    )
    profile_tool = read(
        root, "tools/canopen_interp_debug/lift4_profile_position_cycle.py"
    )

    assert "LIFT_MOTOR_REVS_PER_MM = 20.0 / 10.0" in sync_tool
    assert "LIFT_MIN_POSITION_COUNTS = -128_450_560" in sync_tool
    assert "DEFAULT_MOTOR_REVS_PER_MM = 20 / 10" in velocity_tool
    assert 'parser.add_argument("--short-mm", type=float, default=10.0)' in profile_tool
    assert 'parser.add_argument("--safe-min-mm", type=float, default=10.0)' in profile_tool
    assert "12.0 / 10.0" not in sync_tool
    assert "DEFAULT_MOTOR_REVS_PER_MM = 12 / 10" not in velocity_tool


def test_canopen_controlword_sequence_is_not_coalesced(root: pathlib.Path) -> None:
    """CiA 402 control-word transitions are ordered writes, not final-value settings."""

    service_c = read(root, "ecu/drivers/canopen/src/canopen_master_service.c")
    service_h = read(root, "ecu/drivers/canopen/include/canopen_master_service.h")

    assert "preserve_order" in service_h
    assert "request.preserve_order = canopen_master_sdo_write_requires_order" in service_c
    assert "queued->preserve_order || request.preserve_order" in service_c
    assert "never coalesce them" in service_c
    assert "queued->value != request.value" not in service_c
    assert "queued->node_id == node_id" in service_c
    assert "queued->index == index" in service_c


def test_servo_motion_uses_bc_canopen_command_sequences(root: pathlib.Path) -> None:
    """BC/BC2 commands must match the field-proven CANopen examples."""

    servo_h = read(root, "ecu/devices/include/servo_drive_canopen.h")
    servo_c = read(root, "ecu/devices/src/servo_drive_canopen.c")
    motion_c = read(root, "ecu/devices/src/motion_device.c")
    config_h = read(root, "ecu/config/include/ecu_config.h")

    for token in [
        "servo_drive_canopen_run_velocity_mode",
        "servo_drive_canopen_run_absolute_position_mode",
        "servo_drive_canopen_run_current_mode",
        "SERVO_DRIVE_CONTROL_ENABLE_OPERATION",
        "SERVO_DRIVE_CONTROL_TRIGGER_ABSOLUTE_POSITION",
        "ECU_CANOPEN_OBJ_PROFILE_VELOCITY",
        "ECU_CANOPEN_OBJ_COMMAND_CURRENT",
    ]:
        assert token in servo_h or token in servo_c or token in config_h, token

    assert "ECU_CANOPEN_COMMISSIONING_POLICY_MAPPING_VERIFY_ONLY" in config_h
    assert "servo_drive_canopen_configure_steer_rpdo(canopen" not in motion_c
    assert "servo_drive_canopen_prepare_position_mode(canopen" not in motion_c
    assert "canopen_master_service_queue_pdo_batch_with_descriptor(" in motion_c
    position_func = servo_c.split("static bool servo_drive_canopen_run_position_mode", 1)[1]
    assert position_func.index("ECU_CANOPEN_OBJ_TARGET_POSITION") < position_func.index("SERVO_DRIVE_CONTROL_ENABLE_OPERATION")
    assert position_func.index("SERVO_DRIVE_CONTROL_ENABLE_OPERATION") < position_func.index("SERVO_DRIVE_CONTROL_TRIGGER_ABSOLUTE_POSITION")


def test_motion_device_separates_servo_setup_from_realtime_targets(root: pathlib.Path) -> None:
    """Remote motion must not flood the SDO queue with full servo setup every 5 ms."""

    motion_h = read(root, "ecu/devices/include/motion_device.h")
    motion_c = read(root, "ecu/devices/src/motion_device.c")
    servo_h = read(root, "ecu/devices/include/servo_drive_canopen.h")
    servo_c = read(root, "ecu/devices/src/servo_drive_canopen.c")
    config_h = read(root, "ecu/config/include/ecu_config.h")

    for token in [
        "drive_velocity_mode_ready",
        "drive_last_velocity_units",
        "drive_last_target_update_ms",
        "steer_position_mode_ready",
        "steer_last_position_counts",
        "steer_last_target_update_ms",
    ]:
        assert token in motion_h, token

    for token in [
        "ECU_CANOPEN_MOTION_TARGET_MIN_INTERVAL_MS",
        "ECU_CANOPEN_DRIVE_VELOCITY_REFRESH_MS",
        "ECU_CANOPEN_DRIVE_COMMAND_ZERO_DEADBAND_UNITS",
        "ECU_CANOPEN_DRIVE_PDO_CHANGE_THRESHOLD_UNITS",
        "ECU_CANOPEN_STEER_POSITION_DEADBAND_COUNTS",
    ]:
        assert token in config_h, token

    assert "servo_drive_canopen_update_target_velocity" in servo_h
    assert "servo_drive_canopen_update_relative_position" not in servo_h
    assert "servo_drive_canopen_update_relative_position" not in servo_c
    assert "ECU_CANOPEN_OBJ_TARGET_VELOCITY" in servo_c
    assert "build_drive_velocity_rpdo_request" in motion_c
    assert "cache_latest_drive_velocity" in motion_c
    assert "queue_drive_group" in motion_c
    assert "motion_device_flush_realtime" in motion_h
    assert "motion_device_flush_realtime" in motion_c
    assert "state->steer_last_target_update_ms[wheel]" in motion_c
    assert "state->drive_last_target_update_ms[wheel]" in motion_c
    assert "nonzero_velocity_refresh_due" in motion_c
    assert "ECU_CANOPEN_DRIVE_VELOCITY_REFRESH_MS" in motion_c


def test_disable_enable_recovery_is_local_and_does_not_reset_drive_setup(root: pathlib.Path) -> None:
    """Remote disable/enable may clear ECU realtime latches, not drive zero/reference state."""

    motion_c = read(root, "ecu/devices/src/motion_device.c")
    service_h = read(root, "ecu/drivers/canopen/include/canopen_master_service.h")
    service_c = read(root, "ecu/drivers/canopen/src/canopen_master_service.c")

    reset_fn = motion_c.split("static void reset_can2_realtime_motion_state", 1)[1].split(
        "static bool can2_motion_high_voltage_ready", 1
    )[0]

    assert "canopen_master_service_cancel_realtime_pdo" in reset_fn
    assert "ECU-local" in reset_fn
    assert "drive_velocity_mode_ready[wheel] = false" not in reset_fn
    assert "steer_position_mode_ready[wheel] = false" not in reset_fn
    assert "steer_axis_config_state[wheel] = MOTION_STEER_AXIS_UNSEEN" not in reset_fn
    assert "reset_can2_motion_operational_request(state)" not in reset_fn
    assert "CANOPEN_MASTER_DEBUG_COMMAND_NMT_RESET" not in motion_c
    assert "canopen_master_service_cancel_realtime_pdo" in service_h
    assert "canopen_master_service_cancel_realtime_pdo" in service_c
    assert "canopen_master_service_recover_transport" in service_h

    recover_transport_fn = service_c.split(
        "bool canopen_master_service_recover_transport", 1
    )[1].split("void canopen_master_service_note_pdo_safety_inhibit", 1)[0]
    assert "intc_m_disable_irq" in recover_transport_fn
    assert "can_deinit(can)" in recover_transport_fn
    assert "can_init(can," in recover_transport_fn
    assert "data->config" in recover_transport_fn
    assert "intc_m_enable_irq_with_priority" in recover_transport_fn


def test_can2_realtime_transient_pdo_failure_recovers_without_latching(root: pathlib.Path) -> None:
    """Running motion must tolerate a bounded non-trigger PDO fault without freezing output."""

    config_h = read(root, "ecu/config/include/ecu_config.h")
    motion_h = read(root, "ecu/devices/include/motion_device.h")
    motion_c = read(root, "ecu/devices/src/motion_device.c")

    assert "ECU_CANOPEN_REALTIME_TRANSIENT_FAILURE_LIMIT" in config_h
    for token in [
        "can2_realtime_transient_recovery_count",
        "can2_realtime_consecutive_failure_count",
        "can2_realtime_last_recovery_ms",
    ]:
        assert token in motion_h, token
        assert token in read(root, "ecu/vehicle/include/vehicle_types.h"), token
        assert token in read(root, "ecu/diag/include/runtime_monitor.h"), token

    recover_fn = motion_c.split("static bool recover_or_latch_can2_transient_failure", 1)[1].split(
        "static bool fail_active_drive_group", 1
    )[0]
    assert "canopen_master_service_cancel_realtime_pdo(canopen)" in recover_fn
    assert "canopen_master_service_recover_transport(canopen)" in recover_fn
    assert "state->steer_pending_target[wheel] = true" in recover_fn
    assert "state->steer_commanded_velocity_counts_per_sec[wheel] = 0" in recover_fn
    assert "force_can2_drive_safe_stop_intent(state)" in recover_fn
    assert "state->steer_group_degraded = latch_required" in recover_fn

    fail_steer_fn = motion_c.split("static bool fail_active_steer_group", 1)[1].split(
        "#if ECU_CANOPEN_COMMISSIONING_POLICY", 1
    )[0]
    assert "trigger_or_partial_failure" in fail_steer_fn
    assert "CANOPEN_MASTER_PDO_PHASE_STEER_TRIGGER" in fail_steer_fn
    assert "recover_or_latch_can2_transient_failure" in fail_steer_fn
    monitor_c = read(root, "ecu/diag/src/runtime_monitor.c")
    assert "recover=%lu consec_fail=%lu last_recover_ms=%lu" in monitor_c


def test_can2_node_feedback_loss_and_partial_group_failure_self_recover_safely(
    root: pathlib.Path,
) -> None:
    """CAN2 recovery must clear faults without replaying stale traction intent."""

    config_h = read(root, "ecu/config/include/ecu_config.h")
    motion_h = read(root, "ecu/devices/include/motion_device.h")
    motion_c = read(root, "ecu/devices/src/motion_device.c")
    service_c = read(root, "ecu/drivers/canopen/src/canopen_master_service.c")

    assert "ECU_CANOPEN_MOTION_FEEDBACK_STALE_RECOVERY_MS" in config_h
    assert "ECU_CANOPEN_PARTIAL_GROUP_RECOVERY_STABLE_MS" in config_h
    for token in [
        "can2_node_recovery_pending_mask",
        "can2_node_recovery_entry_mask",
        "can2_stale_feedback_mask",
        "can2_partial_group_recovery_active",
        "can2_recovery_steer_sync_pending",
        "can2_recovery_steer_group_sequence",
    ]:
        assert token in motion_h, token

    observe = motion_c.split("static void observe_can2_node_recovery_state", 1)[1].split(
        "static bool can2_all_motion_nodes_operation_enabled_fresh", 1
    )[0]
    assert "feedback.last_tpdo0_ms" in observe
    assert "feedback.last_tpdo1_ms" in observe
    assert "operational_heartbeat_recent" in observe
    assert "state->can2_node_recovery_pending_mask |= bit" in observe
    assert "state->can2_node_recovery_entry_mask |= bit" in observe

    flush = motion_c.split("ecu_device_apply_result_t motion_device_flush_realtime", 1)[1]
    recovery_branch = flush.split(
        "if (state->can2_node_recovery_pending_mask != 0U)", 1
    )[1].split("if (state->can2_partial_group_recovery_active)", 1)[0]
    assert "state->can2_node_recovery_entry_mask != 0U" in recovery_branch
    assert "state->can2_node_recovery_entry_mask = 0U" in recovery_branch
    assert "state->drive_group_active || state->steer_group_active" not in recovery_branch

    recovery = motion_c.split("static void service_can2_node_recovery", 1)[1].split(
        "static void service_can2_partial_group_recovery", 1
    )[0]
    assert "ECU_CANOPEN_OBJ_FAULT_LATCHED" in recovery
    assert "SERVO_DRIVE_CONTROL_FAULT_RESET" in recovery
    assert "SERVO_DRIVE_MODE_PROFILE_VELOCITY" in recovery
    assert "SERVO_DRIVE_MODE_PROFILE_POSITION" in recovery
    assert "CANOPEN_MASTER_DEBUG_COMMAND_NMT_RESET" not in recovery

    partial = motion_c.split("static void service_can2_partial_group_recovery", 1)[1].split(
        "static void reset_can2_realtime_motion_state", 1
    )[0]
    assert "can2_recovery_zero_enable_permitted(state)" in partial
    assert "drive_zero_intent_completed(state, recovery_enable_requested)" in partial
    assert "can2_all_motion_nodes_operation_enabled_fresh(canopen)" in partial
    assert "state->steer_group_degraded = false" in partial
    assert "state->can2_recovery_steer_sync_pending = true" in partial

    apply_fn = motion_c.split("ecu_device_apply_result_t motion_device_apply", 1)[1].split(
        "ecu_device_apply_result_t motion_device_flush_realtime", 1
    )[0]
    assert "can2_recovery_allows_drive" in apply_fn
    assert "!state->can2_recovery_steer_sync_pending" in apply_fn

    finish = motion_c.split("static void finish_completed_steer_group", 1)[1].split(
        "static void clean_cancel_active_steer_group", 1
    )[0]
    assert "completed_group_sequence" in finish
    assert "state->can2_recovery_steer_sync_pending = false" in finish

    assert "ECU_CANOPEN_OBJ_FAULT_LATCHED" in service_c


def test_can2_recovery_holds_zero_speed_operation_enabled_without_replaying_motion(
    root: pathlib.Path,
) -> None:
    """Recovery must not disable the same nodes whose enabled feedback it awaits."""

    motion_c = read(root, "ecu/devices/src/motion_device.c")

    for token in [
        "can2_zero_speed_operation_enable_permitted",
        "can2_recovery_zero_enable_permitted",
        "force_can2_drive_recovery_zero_intent",
        "drive_zero_intent_completed",
    ]:
        assert token in motion_c, token

    permission = motion_c.split(
        "static bool can2_zero_speed_operation_enable_permitted", 1
    )[1].split("static bool", 1)[0]
    assert "command->high_voltage_enable" in permission
    assert "command->high_voltage_feedback_ready" in permission
    assert "!command->high_voltage_disable_request" in permission
    assert "command_source_allows_motion_output(command->source)" in permission

    apply_fn = motion_c.split(
        "ecu_device_apply_result_t motion_device_apply", 1
    )[1].split("ecu_device_apply_result_t motion_device_flush_realtime", 1)[0]
    assert (
        "drive_enable_requested =\n"
        "        can2_zero_speed_operation_enable_permitted(command)"
    ) in apply_fn
    assert "velocity_units = 0" in apply_fn
    assert "can2_recovery_allows_drive" in apply_fn

    flush_fn = motion_c.split(
        "ecu_device_apply_result_t motion_device_flush_realtime", 1
    )[1]
    node_recovery = flush_fn.split(
        "if (state->can2_node_recovery_pending_mask != 0U)", 1
    )[1].split("if (state->can2_partial_group_recovery_active)", 1)[0]
    assert "force_can2_drive_recovery_zero_intent(state)" in node_recovery
    assert "drive_zero_intent_completed" in node_recovery

    partial_recovery = flush_fn.split(
        "if (state->can2_partial_group_recovery_active)", 1
    )[1].split("if (state->can2_recovery_steer_sync_pending", 1)[0]
    assert "force_can2_drive_recovery_zero_intent(state)" in partial_recovery

    drive_flush = motion_c.split(
        "static ecu_device_apply_result_t flush_drive_velocity_realtime", 1
    )[1].split("static void request_can2_motion_nodes_operational", 1)[0]
    recovery_zero = drive_flush.split(
        "if (recovery_zero_intent_active)", 1
    )[1].split("if (!all_drive_axes_realtime_ready", 1)[0]
    assert "int32_t zero_velocity[ECU_WHEEL_COUNT] = {0}" in recovery_zero
    assert "int16_t zero_current[ECU_WHEEL_COUNT] = {0}" in recovery_zero
    assert "queue_drive_group" in recovery_zero
    assert "build_drive_group_targets" not in recovery_zero

    assert "state->last_motion_command.source = command->source" in apply_fn
    assert "state->last_motion_command.diagnostic = command->diagnostic" in apply_fn


def test_can2_steering_startup_enables_position_mode_before_tpdo_gate(root: pathlib.Path) -> None:
    """Node5..8 must not deadlock waiting for TPDO before being mode-set/enabled."""

    motion_c = read(root, "ecu/devices/src/motion_device.c")
    recovery = motion_c.split("static void service_can2_node_recovery", 1)[1].split(
        "static void reset_can2_realtime_motion_state", 1
    )[0]

    for token in [
        "can2_motion_node_is_steer(node_id)",
        "SERVO_DRIVE_MODE_PROFILE_POSITION",
        "SERVO_DRIVE_CONTROL_SHUTDOWN",
        "SERVO_DRIVE_CONTROL_SWITCH_ON",
        "SERVO_DRIVE_CONTROL_ENABLE_OPERATION",
        "Do not wait for TPDO readiness",
        "does not reset the node",
        "actual-position object",
    ]:
        assert token in recovery, token

    assert "CANOPEN_MASTER_DEBUG_COMMAND_NMT_RESET_NODE" not in recovery
    assert "CANOPEN_MASTER_DEBUG_COMMAND_NMT_RESET_COMMUNICATION" not in recovery
    assert "ECU_CANOPEN_OBJ_ACTUAL_POSITION" not in recovery


def test_steering_realtime_coalesces_against_last_command_not_feedback_error(root: pathlib.Path) -> None:
    """A wheel still travelling toward the same target must not retrigger endless zero groups."""

    motion_c = read(root, "ecu/devices/src/motion_device.c")
    build_fn = motion_c.split("static bool build_steer_group_targets", 1)[1].split(
        "static uint32_t next_steer_group_sequence", 1
    )[0]

    assert "steer_last_commanded_position_valid" in build_fn
    assert "steer_last_commanded_position_counts" in build_fn
    # Fresh measured position is allowed only to seed a previously uninitialized
    # trajectory. Ongoing group-change coalescing remains command-based.
    assert "if (!state->steer_commanded_target_valid[wheel])" in build_fn
    assert build_fn.index("steer_last_position_counts") < build_fn.index(
        "if (!state->steer_last_commanded_position_valid[wheel]"
    )
    coalesce = build_fn.split("if (!state->steer_last_commanded_position_valid[wheel]", 1)[1]
    assert "steer_last_position_counts" not in coalesce

    ready_fn = motion_c.split("static bool steer_axis_realtime_ready", 1)[1].split(
        "static bool all_steer_axes_realtime_ready", 1
    )[0]
    assert "bool feedback_valid" in ready_fn
    assert "can2_steer_boot_heartbeat_evidence_ready" in ready_fn
    assert "if (!feedback_valid &&" in ready_fn
    assert "state->steer_realtime_enabled[wheel] = false" in ready_fn
    assert "state->steer_pending_target[wheel] = true" not in ready_fn
    assert "state->steer_realtime_enabled[wheel] = true" not in ready_fn

    gate_fn = motion_c.split("static motion_steer_inhibit_reason_t evaluate_steer_inhibit_reason", 1)[1].split(
        "static bool motion_device_update_steer_safety_gate", 1
    )[0]
    assert "!command->high_voltage_enable" in gate_fn
    assert "!command->high_voltage_feedback_ready" in gate_fn


def test_steering_realtime_uses_pdo_batch_scheduler(root: pathlib.Path) -> None:
    """Four steering axes must be updated by one non-blocking 50 Hz PDO scheduler."""

    motion_h = read(root, "ecu/devices/include/motion_device.h")
    motion_c = read(root, "ecu/devices/src/motion_device.c")
    service_h = read(root, "ecu/drivers/canopen/include/canopen_master_service.h")
    service_c = read(root, "ecu/drivers/canopen/src/canopen_master_service.c")
    servo_h = read(root, "ecu/devices/include/servo_drive_canopen.h")
    config_h = read(root, "ecu/config/include/ecu_config.h")
    executor_h = read(root, "ecu/vehicle/include/vehicle_command_executor.h")
    executor_c = read(root, "ecu/vehicle/src/vehicle_command_executor.c")
    tasks_c = read(root, "ecu/os/src/ecu_tasks_cpu0.c")

    for token in [
        "ECU_CANOPEN_STEER_PDO_PERIOD_MS",
        "ECU_CANOPEN_OBJ_RPDO1_COMM_PARAM",
        "ECU_CANOPEN_OBJ_RPDO1_MAPPING",
        "ECU_CANOPEN_PDO_MAP_CONTROLWORD_16",
        "ECU_CANOPEN_PDO_MAP_TARGET_POSITION_32",
        "ECU_CANOPEN_STEER_POSITION_TRIGGER_THRESHOLD_COUNTS",
        "ECU_CANOPEN_STEER_SETUP_SETTLE_MS",
    ]:
        assert token in config_h, token

    assert "SERVO_DRIVE_CONTROL_ABSOLUTE_UPDATE_ARM" in servo_h
    assert "SERVO_DRIVE_CONTROL_ABSOLUTE_UPDATE_TRIGGER" in servo_h
    assert "canopen_master_service_queue_pdo" in service_h
    assert "canopen_master_service_queue_pdo_batch" in service_h
    assert "canopen_master_service_pdo_group_pending" in service_h
    assert "process_pdo_tx_queue" in service_c
    assert "CANOPEN_MASTER_PDO_QUEUE_CAPACITY" in service_h
    assert "group_sequence" in service_h
    assert "retry_count" in service_h
    assert "hpm_can_send" in service_c
    assert "pdo_tx_count" in service_h

    for token in [
        "steer_pdo_configured",
        "steer_latest_target_counts",
        "steer_realtime_last_flush_ms",
        "steer_realtime_enabled",
        "steer_setup_queued_ms",
        "steer_pdo_tx_error_count",
        "steer_active_group_sequence",
        "steer_active_group_target_counts",
        "steer_next_group_target_counts",
        "steer_next_group_valid",
    ]:
        assert token in motion_h, token

    assert "motion_device_flush_realtime" in motion_c
    assert "steer_axis_realtime_ready" in motion_c
    assert "canopen_master_service_get_node_feedback" in motion_c
    assert "MOTION_STEER_AXIS_READY" in motion_c
    assert "build_steer_rpdo_request" in motion_c
    assert "queue_steer_group" in motion_c
    assert "canopen_master_service_pdo_queue_available(canopen) < ECU_STEER_GROUP_PDO_FRAME_COUNT" in motion_c
    assert "canopen_master_service_pdo_group_pending(canopen, state->steer_active_group_sequence)" in motion_c
    assert "canopen_master_service_queue_pdo_batch_with_descriptor(" in motion_c
    assert "sync_after_arm = true" in motion_c
    assert "sync_after_trigger = true" in motion_c
    assert "SERVO_DRIVE_CONTROL_ABSOLUTE_UPDATE_ARM" in motion_c
    assert "SERVO_DRIVE_CONTROL_ABSOLUTE_UPDATE_TRIGGER" in motion_c
    assert motion_c.index("SERVO_DRIVE_CONTROL_ABSOLUTE_UPDATE_ARM") < motion_c.index(
        "SERVO_DRIVE_CONTROL_ABSOLUTE_UPDATE_TRIGGER"
    )
    assert "canopen_master_service_send_pdo(canopen" not in motion_c
    assert "bool canopen_master_service_send_pdo" not in service_h
    assert "bool canopen_master_service_send_pdo" not in service_c
    assert "servo_drive_canopen_update_absolute_position" not in motion_c
    assert "bit10" not in motion_c.lower()

    assert "vehicle_command_executor_flush_can2_motion" in executor_h
    assert "motion_device_flush_realtime(&s_runtime.motion" in executor_c
    assert "vehicle_command_executor_flush_can2_motion(&s_runtime.executor" in tasks_c


def test_canopen_pdo_scheduler_waits_for_hardware_tx_complete(root: pathlib.Path) -> None:
    """Realtime PDO groups must complete by hardware TX-complete, not mailbox enqueue."""

    service_h = read(root, "ecu/drivers/canopen/include/canopen_master_service.h")
    service_c = read(root, "ecu/drivers/canopen/src/canopen_master_service.c")
    wrap_c = read(root, "ecu/drivers/canopen/src/hpm_can_send_nonblocking_wrap.c")
    monitor_c = read(root, "ecu/diag/src/runtime_monitor.c")

    assert "can_send_high_priority_message_nonblocking" in wrap_c
    assert "can_send_message_nonblocking" not in wrap_c
    assert "can_is_secondary_transmit_buffer_full" not in wrap_c

    for token in [
        "CANOPEN_MASTER_PDO_TX_TIMEOUT_MS",
        "canopen_master_pdo_group_state_t",
        "CANOPEN_MASTER_PDO_GROUP_STATE_QUEUED",
        "CANOPEN_MASTER_PDO_GROUP_STATE_ARM_IN_FLIGHT",
        "CANOPEN_MASTER_PDO_GROUP_STATE_TRIGGER_IN_FLIGHT",
        "CANOPEN_MASTER_PDO_GROUP_STATE_COMPLETE",
        "CANOPEN_MASTER_PDO_GROUP_STATE_FAILED",
        "CANOPEN_MASTER_PDO_GROUP_STATE_CANCELLED",
        "pdo_in_flight",
        "active_pdo_group_state",
        "active_pdo_expected_frames",
        "active_pdo_tx_complete_frames",
        "active_pdo_in_flight_frames",
        "active_pdo_arm_complete_frames",
        "active_pdo_trigger_complete_frames",
        "last_pdo_tx_complete_ms",
        "last_pdo_tx_timeout_ms",
    ]:
        assert token in service_h, token

    for token in [
        "s_canopen_pdo_tx_complete_count",
        "note_canopen_tx_flags_from_isr",
        "CAN_EVENT_TX_PRIMARY_BUF",
        "complete_in_flight_pdo",
        "fail_active_pdo_group",
        "process_pdo_tx_complete_events",
        "cancel_queued_pdo_group",
        "start_next_pdo_frame",
        "CANOPEN_MASTER_PDO_GROUP_STATE_ARM_IN_FLIGHT",
        "CANOPEN_MASTER_PDO_GROUP_STATE_TRIGGER_IN_FLIGHT",
    ]:
        assert token in service_c, token

    process_body = service_c.split("static void process_pdo_tx_queue", 1)[1]
    process_body = process_body.split("static bool start_sdo_upload", 1)[0]
    assert process_body.index("process_pdo_tx_complete_events") < process_body.index(
        "start_next_pdo_frame"
    )
    assert "drop_current_pdo_queue_item(service);" not in process_body.split(
        "complete_in_flight_pdo", 1
    )[0]

    for token in [
        "pdo_group_state",
        "pdo_expected_frames",
        "pdo_tx_complete_frames",
        "pdo_failed_frames",
        "pdo_in_flight_frames",
        "pdo_arm_complete_frames",
        "pdo_trigger_complete_frames",
        "last_pdo_tx_complete_ms",
        "last_pdo_tx_timeout_ms",
    ]:
        assert token in service_h, token
        assert token in monitor_c, token


def test_v4_can2_safety_inhibit_blocks_normal_steering_pdo(root: pathlib.Path) -> None:
    """Safety, park, disarm and unverified axes must not create normal 2F/3F groups."""

    motion_h = read(root, "ecu/devices/include/motion_device.h")
    motion_c = read(root, "ecu/devices/src/motion_device.c")
    service_h = read(root, "ecu/drivers/canopen/include/canopen_master_service.h")
    service_c = read(root, "ecu/drivers/canopen/src/canopen_master_service.c")
    monitor_c = read(root, "ecu/diag/src/runtime_monitor.c")
    config_h = read(root, "ecu/config/include/ecu_config.h")

    for token in [
        "ECU_CAN2_BENCH_PDO_CAPTURE_MODE",
        "#define ECU_CAN2_BENCH_PDO_CAPTURE_MODE (0)",
        "#define ECU_COMMISSIONING_STEER_ONLY_MODE (1U)",
    ]:
        assert token in config_h, token

    for token in [
        "motion_steer_inhibit_reason_t",
        "MOTION_STEER_INHIBIT_ESTOP_LATCHED",
        "MOTION_STEER_INHIBIT_SBUS_OFFLINE",
        "MOTION_STEER_INHIBIT_REMOTE_DISARMED",
        "MOTION_STEER_INHIBIT_GEAR_PARK",
        "MOTION_STEER_INHIBIT_AXIS_NOT_READY",
        "MOTION_STEER_INHIBIT_GROUP_DEGRADED",
        "MOTION_STEER_INHIBIT_COMMAND_SOURCE_NOT_AUTHORIZED",
        "MOTION_STEER_INHIBIT_BENCH_MODE_DISABLED",
        "steer_normal_pdo_allowed",
        "steer_safety_inhibited",
        "steer_inhibit_reason",
        "steer_safety_inhibit_count",
        "steer_last_allowed_to_inhibited_ms",
        "steer_safe_stop_pending",
    ]:
        assert token in motion_h, token

    for token in [
        "motion_device_update_steer_safety_gate",
        "canopen_master_service_note_pdo_safety_inhibit",
        "canopen_master_service_cancel_pdo_group",
        "state->steer_next_group_valid = false;",
        "state->steer_safe_stop_pending = true;",
    ]:
        assert token in motion_c, token

    source_body = motion_c.split("static bool command_source_allows_motion_output", 1)[1]
    source_body = source_body.split("static bool command_changed", 1)[0]
    assert "COMMAND_SOURCE_SAFETY" not in source_body

    queue_call = motion_c.split("queue_steer_group(canopen", 1)[0]
    assert "state->steer_normal_pdo_allowed" in queue_call

    assert "canopen_master_service_note_pdo_safety_inhibit" in service_h
    assert "canopen_master_service_cancel_pdo_group" in service_h
    assert "pdo_safety_inhibit_count" in service_h
    assert "CANOPEN_MASTER_PDO_FAIL_SAFETY_INHIBITED" in service_h
    assert "CANOPEN_MASTER_PDO_GROUP_STATE_CANCELLED" in service_c

    for token in [
        "steer_normal_pdo_allowed",
        "steer_safety_inhibited",
        "steer_inhibit_reason",
        "steer_safety_inhibit_count",
        "steer_safe_stop_pending",
    ]:
        assert token in monitor_c, token


def test_v4_vehicle_motion_command_mailbox_owns_can2_runtime(root: pathlib.Path) -> None:
    """Vehicle task must publish one coherent command; CAN2 task owns motion runtime."""

    executor_c = read(root, "ecu/vehicle/src/vehicle_command_executor.c")
    executor_h = read(root, "ecu/vehicle/include/vehicle_command_executor.h")
    motion_c = read(root, "ecu/devices/src/motion_device.c")

    for token in [
        "vehicle_motion_command_mailbox_t",
        "publish_motion_command_snapshot",
        "read_motion_command_snapshot",
        "publish_sequence",
        "read_sequence_before",
        "read_sequence_after",
        "sequence is even",
    ]:
        assert token in executor_c or token in executor_h, token

    apply_body = executor_c.split("bool vehicle_command_executor_apply", 1)[1]
    apply_body = apply_body.split("bool vehicle_command_executor_flush_can2_motion", 1)[0]
    assert "motion_device_apply(&s_runtime.motion" not in apply_body
    assert "publish_motion_command_snapshot" in apply_body

    flush_body = executor_c.split("bool vehicle_command_executor_flush_can2_motion", 1)[1]
    flush_body = flush_body.split("void vehicle_command_executor_get_state", 1)[0]
    assert "read_motion_command_snapshot" in flush_body
    assert "motion_device_apply(&s_runtime.motion" in flush_body
    assert "motion_device_flush_realtime(&s_runtime.motion" in flush_body

    assert "motion_device_apply(motion_device_state_t *state" in motion_c
    assert "uint32_t command_sequence" in motion_c


def test_v4_unverified_steering_axes_are_not_realtime_ready(root: pathlib.Path) -> None:
    """SDO enqueue or queue-drain must not mark a steering axis realtime-ready."""

    motion_h = read(root, "ecu/devices/include/motion_device.h")
    motion_c = read(root, "ecu/devices/src/motion_device.c")
    service_h = read(root, "ecu/drivers/canopen/include/canopen_master_service.h")
    config_h = read(root, "ecu/config/include/ecu_config.h")

    for token in [
        "motion_steer_axis_config_state_t",
        "MOTION_STEER_AXIS_UNSEEN",
        "MOTION_STEER_AXIS_SDO_PENDING",
        "MOTION_STEER_AXIS_SDO_TIMEOUT",
        "MOTION_STEER_AXIS_SDO_ABORT",
        "MOTION_STEER_AXIS_CONFIG_UNVERIFIED",
        "MOTION_STEER_AXIS_READY",
        "MOTION_STEER_AXIS_FAULT",
        "steer_axis_config_state",
        "steer_axis_remote_verified",
    ]:
        assert token in motion_h, token

    assert "canopen_master_service_has_node_evidence" in service_h
    assert "#define ECU_CAN2_BENCH_PDO_CAPTURE_MODE (0)" in config_h

    ready_body = motion_c.split("static bool steer_axis_realtime_ready", 1)[1]
    ready_body = ready_body.split("static bool all_steer_axes_realtime_ready", 1)[0]
    feedback_helper = motion_c.split("static bool can2_feedback_operation_enabled", 1)[1]
    feedback_helper = feedback_helper.split("static bool steer_axis_calibration_is_valid", 1)[0]
    assert "canopen_master_service_get_node_feedback" in ready_body
    assert "can2_feedback_operation_enabled" in ready_body
    assert "can2_steer_boot_heartbeat_evidence_ready" in ready_body
    assert "ECU_CANOPEN_STEER_BOOT_HEARTBEAT_EVIDENCE_ENABLED" in config_h
    assert "feedback->feedback_fresh" in feedback_helper
    assert "feedback->fault_latched != 0U" in feedback_helper
    assert "steer_axis_remote_verified" in ready_body
    assert "MOTION_STEER_AXIS_READY" in ready_body
    assert "command_queue_count == 0U && !canopen->sdo_download_active" not in ready_body
    assert "state->steer_realtime_enabled[wheel] = true;" not in ready_body

    setup_body = motion_c.split("static bool prepare_steer_axis_once", 1)[1]
    setup_body = setup_body.split("static bool send_steer_command", 1)[0]
    assert "MOTION_STEER_AXIS_CONFIG_UNVERIFIED" in setup_body
    assert "state->steer_pdo_configured[wheel] = true;" not in setup_body
    assert "state->steer_position_mode_ready[wheel] = true;" not in setup_body


def test_v4_pdo_failure_diagnostics_keep_history_and_reasons(root: pathlib.Path) -> None:
    service_h = read(root, "ecu/drivers/canopen/include/canopen_master_service.h")
    service_c = read(root, "ecu/drivers/canopen/src/canopen_master_service.c")
    monitor_c = read(root, "ecu/diag/src/runtime_monitor.c")

    for token in [
        "canopen_master_pdo_fail_reason_t",
        "CANOPEN_MASTER_PDO_FAIL_SUBMIT_BUSY",
        "CANOPEN_MASTER_PDO_FAIL_SUBMIT_ERROR",
        "CANOPEN_MASTER_PDO_FAIL_TX_TIMEOUT",
        "CANOPEN_MASTER_PDO_FAIL_TX_ERROR_EVENT",
        "CANOPEN_MASTER_PDO_FAIL_GROUP_CANCELLED",
        "CANOPEN_MASTER_PDO_FAIL_SAFETY_INHIBITED",
        "CANOPEN_MASTER_PDO_FAIL_GROUP_CONFLICT",
        "CANOPEN_MASTER_PDO_FAIL_QUEUE_FULL",
        "last_pdo_current_error",
        "last_pdo_failed_error",
        "last_pdo_failed_ms",
        "last_pdo_failed_group_id",
        "last_pdo_failed_reason",
        "pdo_queue_full_drop_count",
        "pdo_group_conflict_drop_count",
        "pdo_safety_inhibit_count",
        "pdo_same_target_coalesce_count",
    ]:
        assert token in service_h, token
        assert token in monitor_c or token.startswith("CANOPEN_MASTER_") or token == "canopen_master_pdo_fail_reason_t"

    assert "note_pdo_failure(service, request, error, reason, now_ms)" in service_c
    assert "service->snapshot.last_pdo_failed_error = error;" in service_c
    assert "service->snapshot.last_pdo_current_error = 0;" in service_c
    assert "service->snapshot.last_pdo_failed_error = 0;" not in service_c.split(
        "void canopen_master_service_process", 1
    )[1]


def test_v4_offline_bringup_paths_are_backed_off(root: pathlib.Path) -> None:
    config_h = read(root, "ecu/config/include/ecu_config.h")
    service_h = read(root, "ecu/drivers/canopen/include/canopen_master_service.h")
    service_c = read(root, "ecu/drivers/canopen/src/canopen_master_service.c")
    commissioning_c = read(root, "ecu/diag/src/commissioning_debug.c")
    analog_h = read(root, "ecu/devices/include/analog_modbus_device.h")
    analog_c = read(root, "ecu/devices/src/analog_modbus_device.c")
    power_h = read(root, "ecu/devices/include/power_device.h")

    for token in [
        "ECU_OFFLINE_BACKOFF_MIN_MS",
        "ECU_OFFLINE_BACKOFF_STEP1_MS",
        "ECU_OFFLINE_BACKOFF_STEP2_MS",
        "ECU_OFFLINE_BACKOFF_MAX_MS",
    ]:
        assert token in config_h, token

    for token in [
        "sdo_retry_backoff_ms",
        "sdo_next_retry_ms",
        "sdo_offline_since_ms",
        "canopen_master_service_diagnostic_scan_allowed",
    ]:
        assert token in service_h, token
        assert token in service_c, token

    assert "canopen_master_service_diagnostic_scan_allowed" in commissioning_c
    assert "ECU_CANOPEN_COMMISSIONING_SCAN_PERIOD_MS" in commissioning_c

    for token in [
        "offline_since_ms",
        "retry_backoff_ms",
        "next_retry_ms",
        "analog_modbus_note_timeout",
    ]:
        assert token in analog_h or token in analog_c, token

    for token in [
        "offline_since_ms",
        "retry_backoff_ms",
        "next_retry_ms",
    ]:
        assert token in power_h, token


def test_canopennode_global_stack_access_is_serialized(root: pathlib.Path) -> None:
    """The SDK global co pointer must not be switched concurrently by CAN2/CAN3 tasks."""

    service_c = read(root, "ecu/drivers/canopen/src/canopen_master_service.c")
    service_h = read(root, "ecu/drivers/canopen/include/canopen_master_service.h")

    for token in [
        "xSemaphoreCreateMutex",
        "xSemaphoreTake",
        "xSemaphoreGive",
        "canopen_master_lock",
        "canopen_master_unlock",
        "canopen_master_service_request_nmt_locked",
    ]:
        assert token in service_c, token

    assert "xSemaphoreTakeRecursive" not in service_c
    assert "xSemaphoreCreateRecursiveMutexStatic" not in service_c
    assert "select_stack(service);" in service_c
    process_body = service_c.split("void canopen_master_service_process", 1)[1]
    process_body = process_body.split("void canopen_master_service_get_snapshot", 1)[0]
    assert process_body.index("canopen_master_lock()") < process_body.index("select_stack(service);")
    assert process_body.rindex("canopen_master_unlock()") > process_body.index("CO_process")
    assert "co;" in service_c
    assert "CANOPEN_MASTER_PDO_QUEUE_CAPACITY" in service_h


def test_steer_only_commissioning_uses_direct_steer_targets(root: pathlib.Path) -> None:
    """Steering bring-up should isolate steering drives from Ackermann geometry."""

    config_h = read(root, "ecu/config/include/ecu_config.h")
    command_arbiter_c = read(root, "ecu/vehicle/src/command_arbiter.c")

    assert "#define ECU_COMMISSIONING_STEER_ONLY_MODE (1U)" in config_h
    assert "#define ECU_REMOTE_MAX_STEER_DEG          (50.0f)" in config_h
    assert "#define ECU_STEER_MOTOR_MAX_RPM                      (3000.0f)" in config_h
    assert "#define ECU_STEER_MAX_POSITION_COUNTS_PER_SEC        (500000)" in config_h
    assert "#define ECU_STEER_PROFILE_VELOCITY_UNITS_FROM_RPM    (5000000)" in config_h
    assert "#define ECU_SERVO_MOTION_MAX_RPM                     (3000.0f)" in config_h
    assert "#define ECU_CANOPEN_STEER_TARGET_ACCEL_LARGE_COUNTS_PER_SEC2       (500000)" in config_h
    assert "#define ECU_DRIVE_MOTOR_MAX_RPM                      ECU_SERVO_MOTION_MAX_RPM" in config_h
    assert "ECU_CANOPEN_STEER_TARGET_RATE_LARGE_COUNTS_PER_SEC <= ECU_STEER_MAX_POSITION_COUNTS_PER_SEC" in config_h
    assert "#define ECU_DRIVE_GEAR_REDUCTION                     (86.6f)" in config_h
    assert "#define ECU_STEER_GEAR_REDUCTION                     (490.0f)" in config_h
    assert "apply_commissioning_steer_only_direct_targets" in command_arbiter_c
    assert "out->target_steer_deg[wheel] = steer_deg" in command_arbiter_c


def test_vehicle_canopen_node_mapping_matches_machine_interfaces(root: pathlib.Path) -> None:
    config_h = read(root, "ecu/config/include/ecu_config.h")
    config_c = read(root, "ecu/config/src/ecu_config.c")
    servo_h = read(root, "ecu/devices/include/servo_drive_canopen.h")
    servo_c = read(root, "ecu/devices/src/servo_drive_canopen.c")
    motion_c = read(root, "ecu/devices/src/motion_device.c")
    lift_c = read(root, "ecu/devices/src/lift_hydraulic_device.c")
    requirements = read(root, "doc/ECU_Project_Implementation_v1.4.md")

    expected_nodes = {
        "ECU_CANOPEN_LEG1_DRIVE_NODE_ID": "0x01U",
        "ECU_CANOPEN_LEG2_DRIVE_NODE_ID": "0x02U",
        "ECU_CANOPEN_LEG3_DRIVE_NODE_ID": "0x03U",
        "ECU_CANOPEN_LEG4_DRIVE_NODE_ID": "0x04U",
        "ECU_CANOPEN_LEG1_STEER_NODE_ID": "0x05U",
        "ECU_CANOPEN_LEG2_STEER_NODE_ID": "0x06U",
        "ECU_CANOPEN_LEG3_STEER_NODE_ID": "0x07U",
        "ECU_CANOPEN_LEG4_STEER_NODE_ID": "0x08U",
        "ECU_CANOPEN_LIFT_LEG1_NODE_ID": "0x09U",
        "ECU_CANOPEN_LIFT_LEG2_NODE_ID": "0x0BU",
        "ECU_CANOPEN_LIFT_LEG3_NODE_ID": "0x0CU",
        "ECU_CANOPEN_LIFT_LEG4_NODE_ID": "0x0AU",
        "ECU_CANOPEN_HYDRAULIC_PUMP_NODE_ID": "0x0DU",
    }
    for name, value in expected_nodes.items():
        assert re.search(rf"#define\s+{name}\s+\({value}\)", config_h), name

    expected_position_aliases = {
        "ECU_CANOPEN_DRIVE_FR_NODE_ID": "ECU_CANOPEN_LEG1_DRIVE_NODE_ID",
        "ECU_CANOPEN_DRIVE_FL_NODE_ID": "ECU_CANOPEN_LEG2_DRIVE_NODE_ID",
        "ECU_CANOPEN_DRIVE_RL_NODE_ID": "ECU_CANOPEN_LEG3_DRIVE_NODE_ID",
        "ECU_CANOPEN_DRIVE_RR_NODE_ID": "ECU_CANOPEN_LEG4_DRIVE_NODE_ID",
        "ECU_CANOPEN_STEER_FR_NODE_ID": "ECU_CANOPEN_LEG1_STEER_NODE_ID",
        "ECU_CANOPEN_STEER_FL_NODE_ID": "ECU_CANOPEN_LEG2_STEER_NODE_ID",
        "ECU_CANOPEN_STEER_RL_NODE_ID": "ECU_CANOPEN_LEG3_STEER_NODE_ID",
        "ECU_CANOPEN_STEER_RR_NODE_ID": "ECU_CANOPEN_LEG4_STEER_NODE_ID",
        "ECU_CANOPEN_LIFT_FR_NODE_ID": "ECU_CANOPEN_LIFT_LEG1_NODE_ID",
        "ECU_CANOPEN_LIFT_FL_NODE_ID": "ECU_CANOPEN_LIFT_LEG2_NODE_ID",
        "ECU_CANOPEN_LIFT_RL_NODE_ID": "ECU_CANOPEN_LIFT_LEG3_NODE_ID",
        "ECU_CANOPEN_LIFT_RR_NODE_ID": "ECU_CANOPEN_LIFT_LEG4_NODE_ID",
    }
    for alias, target in expected_position_aliases.items():
        assert re.search(rf"#define\s+{alias}\s+{target}", config_h), alias
    for token in [
        "Leg 1: front-right",
        "Leg 2: front-left",
        "Leg 3: rear-left",
        "Leg 4: rear-right",
        "ECU_WHEEL_LEG1_FRONT_RIGHT",
        "ECU_WHEEL_LEG2_FRONT_LEFT",
        "ECU_WHEEL_LEG3_REAR_LEFT",
        "ECU_WHEEL_LEG4_REAR_RIGHT",
    ]:
        assert token in config_h or token in config_c, token

    drive_block = re.search(r"\.drive_nodes\s*=\s*\{(?P<body>[\s\S]*?)\n\s*\},", config_c)
    steer_block = re.search(r"\.steer_nodes\s*=\s*\{(?P<body>[\s\S]*?)\n\s*\},", config_c)
    lift_block = re.search(r"\.lift_nodes\s*=\s*\{(?P<body>[\s\S]*?)\n\s*\},", config_c)
    assert drive_block and steer_block and lift_block
    assert [
        "ECU_CANOPEN_LEG1_DRIVE_NODE_ID",
        "ECU_CANOPEN_LEG2_DRIVE_NODE_ID",
        "ECU_CANOPEN_LEG3_DRIVE_NODE_ID",
        "ECU_CANOPEN_LEG4_DRIVE_NODE_ID",
    ] == re.findall(r"ECU_CANOPEN_LEG\d_DRIVE_NODE_ID", drive_block.group("body"))
    assert [
        "ECU_CANOPEN_LEG1_STEER_NODE_ID",
        "ECU_CANOPEN_LEG2_STEER_NODE_ID",
        "ECU_CANOPEN_LEG3_STEER_NODE_ID",
        "ECU_CANOPEN_LEG4_STEER_NODE_ID",
    ] == re.findall(r"ECU_CANOPEN_LEG\d_STEER_NODE_ID", steer_block.group("body"))
    assert [
        "ECU_CANOPEN_LIFT_LEG1_NODE_ID",
        "ECU_CANOPEN_LIFT_LEG2_NODE_ID",
        "ECU_CANOPEN_LIFT_LEG3_NODE_ID",
        "ECU_CANOPEN_LIFT_LEG4_NODE_ID",
    ] == re.findall(r"ECU_CANOPEN_LIFT_LEG\d_NODE_ID", lift_block.group("body"))
    assert "node 9, 11, 12 and 10 in vehicle leg order" in config_c

    for token in [
        "ECU_CANOPEN_OBJ_DIGITAL_INPUT_STATES",
        "SERVO_DRIVE_INPUT_IN2_MASK",
        "SERVO_DRIVE_INPUT_IN3_MASK",
        "SERVO_DRIVE_INPUT_IN7_MASK",
        "SERVO_DRIVE_INPUT_IN8_MASK",
        "servo_drive_canopen_read_input_states",
    ]:
        assert token in config_h or token in servo_h or token in servo_c, token

    for forbidden in [
        "ECU_CANOPEN_OBJ_OUTPUT_STATES_PROGRAM_CONTROL",
        "ECU_SERVO_BRAKE_RELEASE_OUTPUT_ACTIVE_LEVEL",
        "ECU_SERVO_BRAKE_RELEASE_CANOPEN_ACTIVE_BIT",
        "SERVO_DRIVE_OUTPUT_OUT1_MASK",
        "SERVO_DRIVE_OUTPUT_OUT4_MASK",
        "servo_drive_canopen_set_output_state",
    ]:
        assert forbidden not in config_h
        assert forbidden not in servo_h
        assert forbidden not in servo_c
        assert forbidden not in motion_c

    assert "servo_drive_canopen_read_input_states" in motion_c
    assert "SERVO_DRIVE_INPUT_IN2_MASK" in motion_c
    assert "SERVO_DRIVE_INPUT_IN3_MASK" in motion_c
    assert "BC2_AXIS_OUTPUT_BRAKE_MASK" not in lift_c
    assert "BC2_AXIS_INPUT_POSITIVE_LIMIT_MASK" not in lift_c
    assert "BC2_AXIS_INPUT_NEGATIVE_LIMIT_MASK" not in lift_c
    assert "SERVO_DRIVE_OUTPUT_OUT4_MASK" not in lift_c
    assert "SERVO_DRIVE_INPUT_IN7_MASK" not in lift_c
    assert "SERVO_DRIVE_INPUT_IN8_MASK" not in lift_c

    for phrase in [
        "CAN2 是控制整车的行走和转向部分",
        "J3 中的引脚 16 OUT1",
        "IN2 是正限位，IN3 是负限位",
        "CAN3 是整车的抬升功能",
        "BC2 的 SW 拨码只设置 A 轴节点号，B 轴节点号等于 A 轴节点号加 1",
        "A 轴电机的抱闸是控制信号 I/O 端子 J3 的引脚 8 OUT1",
        "B 轴电机的抱闸是引脚 17 OUT4",
        "ECU 不得通过 PCB DIO、`0x2194` 或 OUT1 直接控制抱闸",
        "不得写 `0x2194` 或驱动器 OUT 位来控制抱闸",
        "0x2190",
    ]:
        assert phrase in requirements, phrase


def test_servo_brake_release_is_drive_internal_owner(root: pathlib.Path) -> None:
    config_h = read(root, "ecu/config/include/ecu_config.h")
    motion_c = read(root, "ecu/devices/src/motion_device.c")
    servo_h = read(root, "ecu/devices/include/servo_drive_canopen.h")
    servo_c = read(root, "ecu/devices/src/servo_drive_canopen.c")
    lift_c = read(root, "ecu/devices/src/lift_hydraulic_device.c")

    assert "ECU_BRAKE_ACTUATION_OWNER_SERVO_DRIVE_INTERNAL" in config_h
    assert "ECU_ENABLE_MAINTENANCE_SDO_WRITES (0)" in config_h
    for text in [config_h, motion_c, servo_h, servo_c, lift_c]:
        assert "ECU_SERVO_BRAKE_RELEASE_OUTPUT_ACTIVE_LEVEL" not in text
        assert "ECU_SERVO_BRAKE_RELEASE_CANOPEN_ACTIVE_BIT" not in text
        assert "servo_drive_canopen_set_output_state" not in text
    assert "not used to synthesize lift-servo brake release" in lift_c


def test_servo_brakes_are_not_driven_by_pcb_dio(root: pathlib.Path) -> None:
    config_h = read(root, "ecu/config/include/ecu_config.h")
    config_c = read(root, "ecu/config/src/ecu_config.c")
    local_io_c = read(root, "ecu/devices/src/local_io_device.c")
    local_io_h = read(root, "ecu/devices/include/local_io_device.h")

    assert "#define ECU_DIO_BRAKE_RELEASE_MASK       (0UL)" in config_h
    assert "dio_brake_release_mask = ECU_DIO_BRAKE_RELEASE_MASK" in config_c
    assert "dio_brake_release_mask" not in local_io_c
    assert "command->brake_release" not in local_io_c
    assert "Servo brake outputs are intentionally excluded" in local_io_h


def test_sbus_remote_logic_maps_protocol_raw_to_ppm_equivalent(root: pathlib.Path) -> None:
    config_h = read(root, "ecu/config/include/ecu_config.h")
    config_c = read(root, "ecu/config/src/ecu_config.c")
    tasks_c = read(root, "ecu/os/src/ecu_tasks_cpu0.c")
    mapper_c = read(root, "ecu/remote/src/remote_sbus_mapper.c")
    decoder_c = read(root, "ecu/protocol/sbus/src/sbus_decoder.c")
    monitor_h = read(root, "ecu/diag/include/runtime_monitor.h")
    monitor_c = read(root, "ecu/diag/src/runtime_monitor.c")

    expected_defines = {
        "ECU_SBUS_PROTOCOL_RAW_LOW": "282U",
        "ECU_SBUS_PROTOCOL_RAW_CENTER": "1002U",
        "ECU_SBUS_PROTOCOL_RAW_HIGH": "1722U",
        "ECU_SBUS_PPM_LOW": "1050U",
        "ECU_SBUS_PPM_CENTER": "1500U",
        "ECU_SBUS_PPM_HIGH": "1950U",
        "ECU_SBUS_PPM_LOW_MAX": "1200U",
        "ECU_SBUS_PPM_CENTER_MIN": "1400U",
        "ECU_SBUS_PPM_CENTER_MAX": "1600U",
        "ECU_SBUS_PPM_HIGH_MIN": "1800U",
        "ECU_SBUS_PPM_CREDIBLE_MIN": "1000U",
        "ECU_SBUS_PPM_CREDIBLE_MAX": "2000U",
    }
    for name, value in expected_defines.items():
        assert re.search(rf"#define\s+{name}\s+\({value}\)", config_h), name

    for field in [
        ".stick_min = ECU_SBUS_PPM_LOW",
        ".stick_neutral = ECU_SBUS_PPM_CENTER",
        ".stick_max = ECU_SBUS_PPM_HIGH",
        ".throttle_min = ECU_SBUS_PPM_LOW",
        ".throttle_max = ECU_SBUS_PPM_HIGH",
    ]:
        assert field in config_c, field

    assert "0x07FFU" in decoder_c
    assert "sbus_protocol_raw_to_ppm_equivalent" in mapper_c
    assert "remote_sbus_mapper_build_ppm_sample" in mapper_c
    assert "remote_sbus_mapper_build_ppm_sample(&raw_sbus, &remote_sbus)" in tasks_c
    assert tasks_c.index("sbus_service_get_snapshot(&s_runtime.sbus") < tasks_c.index(
        "remote_sbus_mapper_build_ppm_sample(&raw_sbus, &remote_sbus)"
    )
    assert tasks_c.index("remote_sbus_mapper_build_ppm_sample(&raw_sbus, &remote_sbus)") < tasks_c.index(
        "remote_sbus_mapper_build_input(&s_runtime.remote_sbus_mapper"
    )
    assert "sbus_ppm_channels[ECU_SBUS_CHANNEL_COUNT]" in monitor_h
    assert "ECU SBUS RAW" in monitor_c
    assert "ECU SBUS PPM" in monitor_c

    stick_func = re.search(r"static int16_t sbus_per_mille_from_ppm[\s\S]*?\n}", mapper_c)
    assert stick_func is not None
    assert "thresholds->stick_neutral" in stick_func.group(0)
    assert "thresholds->stick_max" in stick_func.group(0)
    assert "thresholds->stick_min" in stick_func.group(0)
    assert "thresholds->high_min" not in stick_func.group(0)
    assert "thresholds->low_max" not in stick_func.group(0)


def test_sbus_channel_roles_match_field_remote_controller(root: pathlib.Path) -> None:
    config_h = read(root, "ecu/config/include/ecu_config.h")
    requirements = read(root, "doc/ECU_Project_Implementation_v1.4.md")

    expected_roles = {
        "ECU_SBUS_CH_STEER": 0,
        "ECU_SBUS_CH_CLEARANCE": 1,
        "ECU_SBUS_CH_THROTTLE": 2,
        "ECU_SBUS_CH_POWER": 3,
        "ECU_SBUS_CH_GEAR": 4,
        "ECU_SBUS_CH_RIGHT_INDICATOR": 5,
        "ECU_SBUS_CH_AUTHORITY": 6,
        "ECU_SBUS_CH_HOME": 7,
        "ECU_SBUS_CH_HAZARD": 8,
        "ECU_SBUS_CH_HORN": 9,
        "ECU_SBUS_CH_HEADLIGHT": 10,
        "ECU_SBUS_CH_LEFT_INDICATOR": 11,
        "ECU_SBUS_CH_ESTOP": 12,
        "ECU_SBUS_CH_TRACK": 13,
        "ECU_SBUS_CH_R1": 14,
        "ECU_SBUS_CH_R2": 15,
    }
    for name, value in expected_roles.items():
        assert re.search(rf"\b{name}\s*=\s*{value}\b", config_h), name

    for text in [
        "| CH8 | HOME 键 | HOME 模式域选择 |",
        "| CH6 | 右肩按键 | 右转向灯请求 |",
        "| CH14 | 右肩波轮 | 变行距输入 |",
    ]:
        assert text in requirements

def test_can3_and_rgb_status_are_enabled_for_whole_machine(root: pathlib.Path) -> None:
    config_h = read(root, "ecu/config/include/ecu_config.h")
    tasks_c = read(root, "ecu/os/src/ecu_tasks_cpu0.c")
    cmake = read(root, "ecu/apps/agri_chassis_control_cpu0/CMakeLists.txt")

    assert re.search(r"#define\s+ECU_ENABLE_CAN3_LIFT_CANOPEN\s+\(1\)", config_h)
    assert "#if ECU_ENABLE_CAN3_LIFT_CANOPEN" not in tasks_c
    assert "canopen_master_service_init(&s_runtime.can3_lift_hydraulic_canopen" in tasks_c
    assert "canopen_master_service_process_realtime_pdo(&s_runtime.can3_lift_hydraulic_canopen" in tasks_c
    assert "canopen_master_service_process_background(&s_runtime.can3_lift_hydraulic_canopen" in tasks_c

    for rel in [
        "ecu/drivers/status_led/include/status_led_service.h",
        "ecu/drivers/status_led/src/status_led_service.c",
    ]:
        assert (root / rel).exists(), rel

    led_h = read(root, "ecu/drivers/status_led/include/status_led_service.h")
    led_c = read(root, "ecu/drivers/status_led/src/status_led_service.c")
    for token in [
        "status_led_service_update",
        "STATUS_LED_PATTERN_BOOT",
        "STATUS_LED_PATTERN_READY",
        "STATUS_LED_PATTERN_WARNING",
        "STATUS_LED_PATTERN_ESTOP",
        "board_rgb_write",
    ]:
        assert token in led_h or token in led_c, token


def test_whole_vehicle_motion_profile_uses_tpdo_feedback_for_zero_speed(root: pathlib.Path) -> None:
    config_h = read(root, "ecu/config/include/ecu_config.h")
    tasks_c = read(root, "ecu/os/src/ecu_tasks_cpu0.c")
    cmake = read(root, "ecu/apps/agri_chassis_control_cpu0/CMakeLists.txt")
    canopen_c = read(root, "ecu/drivers/canopen/src/canopen_master_service.c")
    monitor_c = read(root, "ecu/diag/src/runtime_monitor.c")

    for token in [
        "whole_vehicle_motion",
        "-DECU_BUILD_PROFILE_WHOLE_VEHICLE_MOTION=1",
        "-DECU_CANOPEN_COMMISSIONING_POLICY=3",
        "-DECU_COMMISSIONING_STEER_ONLY_MODE=0",
    ]:
        assert token in cmake, token

    for token in [
        "ECU_CANOPEN_ZERO_SPEED_RPM_TOLERANCE",
        "ECU_CAN2_ZERO_SPEED_VELOCITY_UNITS",
        "ECU_CAN2_MOTION_VELOCITY_UNITS_PER_RPM",
    ]:
        assert token in config_h, token

    for token in [
        "ECU_BUILD_PROFILE_WHOLE_VEHICLE_MOTION",
        "feedback->actual_velocity_units",
        "ECU_CAN2_ZERO_SPEED_VELOCITY_UNITS",
        "pre_hv_stationary_window",
        "!s_runtime.final_command.high_voltage_enable",
        "!s_runtime.power_snapshot.high_voltage_requested",
        "input->throttle == REMOTE_POS_LOW",
        "out->brake_applied = out->zero_speed",
    ]:
        assert token in tasks_c, token

    for token in [
        "bus_tpdo_node_range",
        "CANOPEN_MASTER_BUS_CAN3",
        "ECU_CANOPEN_LIFT_FR_NODE_ID",
        "ECU_CANOPEN_HYDRAULIC_PUMP_NODE_ID",
        "register_bus_tpdo_observers",
    ]:
        assert token in canopen_c, token

    assert "[ECU CAN2 MOTION TPDO]" in monitor_c
    assert "[ECU CAN3 LIFT TPDO]" in monitor_c

    assert "status_led_service_t status_led" in tasks_c
    assert "status_led_service_init(&s_runtime.status_led" in tasks_c
    assert "status_led_service_update(&s_runtime.status_led" in tasks_c
    assert "status_led_service.c" in cmake


def test_status_led_separates_no_remote_warning_active_and_estop(root: pathlib.Path) -> None:
    """The RGB LED must show ECU liveness without turning normal no-remote bench states red."""

    led_h = read(root, "ecu/drivers/status_led/include/status_led_service.h")
    led_c = read(root, "ecu/drivers/status_led/src/status_led_service.c")
    tasks_c = read(root, "ecu/os/src/ecu_tasks_cpu0.c")

    for token in [
        "STATUS_LED_PATTERN_BOOT",
        "STATUS_LED_PATTERN_NO_REMOTE",
        "STATUS_LED_PATTERN_READY",
        "STATUS_LED_PATTERN_ACTIVE",
        "STATUS_LED_PATTERN_WARNING",
        "STATUS_LED_PATTERN_ESTOP",
        "STATUS_LED_PATTERN_FATAL",
    ]:
        assert token in led_h, token

    for description in [
        "BOOT: blue heartbeat while scheduler is starting",
        "NO_REMOTE: blue heartbeat when SBUS/remote is not online",
        "READY: solid green when remote and required buses are healthy",
        "ACTIVE: cyan heartbeat when an actuator/high-voltage command is active",
        "WARNING: yellow heartbeat for missing optional/commissioning peripherals",
        "ESTOP/FATAL: red is reserved for operator emergency stop or fatal faults",
    ]:
        assert description in led_c, description

    selector = re.search(
        r"static status_led_pattern_t select_status_led_pattern\(void\)[\s\S]*?\n}",
        tasks_c,
    )
    assert selector is not None, "missing CPU0 status LED selector"
    selector_body = selector.group(0)
    assert "ECU_ESTOP_SOURCE_CH13" in selector_body
    assert "STATUS_LED_PATTERN_FATAL" in selector_body
    assert "STATUS_LED_PATTERN_NO_REMOTE" in selector_body
    assert "STATUS_LED_PATTERN_ACTIVE" in selector_body
    assert "s_runtime.safety_snapshot.sbus_failsafe" not in selector_body
    assert "s_runtime.safety_snapshot.estop_latched ||" not in selector_body
    assert selector_body.index("STATUS_LED_PATTERN_FATAL") < selector_body.index(
        "STATUS_LED_PATTERN_NO_REMOTE"
    )
    assert selector_body.index("STATUS_LED_PATTERN_NO_REMOTE") < selector_body.index(
        "STATUS_LED_PATTERN_WARNING"
    )


def test_optional_board_peripheral_init_does_not_permanently_block_boot(root: pathlib.Path) -> None:
    """Missing debug/optional peripherals must not hide a running ECU behind a dead boot loop."""

    board_c = read(root, "ecu/ecu_isolation/board.c")

    console = re.search(
        r"void board_init_console\(void\)[\s\S]*?\n}\n\nvoid board_init_clock",
        board_c,
    )
    i2c = re.search(
        r"void board_init_i2c\(I2C_Type \*ptr\)[\s\S]*?\n}\n\nvoid board_init_can",
        board_c,
    )
    assert console is not None, "missing board_init_console()"
    assert i2c is not None, "missing board_init_i2c()"

    assert "while (1)" not in console.group(0)
    assert "while (1)" not in i2c.group(0)
    assert "[ECU BOARD] WARN: console init failed" in console.group(0)
    assert "[ECU BOARD] WARN: I2C init failed" in i2c.group(0)
    assert "if (freq == 0U)" in i2c.group(0)
    assert i2c.group(0).index("if (freq == 0U)") < i2c.group(0).index("i2c_init_master")
    assert "return;" in i2c.group(0)


def test_runtime_monitor_reports_status_led_pattern(root: pathlib.Path) -> None:
    """Serial logs should explicitly report the RGB state selected by CPU0."""

    monitor_h = read(root, "ecu/diag/include/runtime_monitor.h")
    monitor_c = read(root, "ecu/diag/src/runtime_monitor.c")
    tasks_c = read(root, "ecu/os/src/ecu_tasks_cpu0.c")

    assert '#include "status_led_service.h"' in monitor_h
    assert "status_led_pattern_t status_led_pattern" in monitor_h
    assert "status_led_pattern_text" in monitor_c
    assert "led=%s" in monitor_c
    assert "status_led_pattern_text(snapshot->status_led_pattern)" in monitor_c
    assert "out->status_led_pattern = s_runtime.status_led.last_pattern" in tasks_c
    assert tasks_c.index("status_led_service_update(&s_runtime.status_led") < tasks_c.index(
        "build_runtime_monitor_snapshot(now_ms, &monitor_snapshot)"
    )


def test_runtime_monitor_reports_remote_fsm_states(root: pathlib.Path) -> None:
    """Remote-drive debugging needs the FSM state that rejected the command."""

    monitor_h = read(root, "ecu/diag/include/runtime_monitor.h")
    monitor_c = read(root, "ecu/diag/src/runtime_monitor.c")
    tasks_c = read(root, "ecu/os/src/ecu_tasks_cpu0.c")

    for field in [
        "remote_arm_state_t arm_state;",
        "remote_gear_state_t gear_state;",
        "remote_power_state_t power_state;",
        "remote_authority_state_t authority_state;",
        "remote_adjust_state_t adjust_state;",
    ]:
        assert field in monitor_h, field

    for helper in [
        "arm_state_text",
        "gear_state_text",
        "power_state_text",
        "authority_state_text",
        "adjust_state_text",
    ]:
        assert helper in monitor_c, helper

    assert "arm=%s gear_fsm=%s power=%s auth=%s adjust=%s" in monitor_c
    assert "out->arm_state = s_runtime.remote_request.arm_state;" in tasks_c
    assert "out->gear_state = s_runtime.remote_request.gear_state;" in tasks_c
    assert "out->power_state = s_runtime.remote_request.power_state;" in tasks_c
    assert "bool high_voltage_feedback_ready;" in monitor_h
    assert "out->high_voltage_feedback_ready =" in tasks_c
    assert "hv_fb=%s" in monitor_c
    assert "out->authority_state = s_runtime.remote_request.authority_state;" in tasks_c
    assert "out->adjust_state = s_runtime.remote_request.adjust_state;" in tasks_c


def test_motion_and_lift_canopen_outputs_are_command_gated(root: pathlib.Path) -> None:
    motion_h = read(root, "ecu/devices/include/motion_device.h")
    motion_c = read(root, "ecu/devices/src/motion_device.c")
    lift_h = read(root, "ecu/devices/include/lift_hydraulic_device.h")
    lift_c = read(root, "ecu/devices/src/lift_hydraulic_device.c")

    assert "command_source_allows_motion_output" in motion_c
    assert "command_source_allows_lift_output" not in lift_c
    assert "COMMAND_SOURCE_NONE" not in re.search(
        r"command_source_allows_motion_output[\s\S]*?\n}",
        motion_c,
    ).group(0)
    assert "command_changed(state, command)" in motion_c
    assert "can3_actuator_command_changed(state, command)" in lift_c
    assert "last_motion_command_valid" in motion_h
    assert "last_lift_command_valid" in lift_h
    assert "skipped_count" in motion_h
    assert "skipped_lift_canopen_count" in lift_h


def test_motion_command_cache_does_not_memcmp_struct_padding(root: pathlib.Path) -> None:
    motion_c = read(root, "ecu/devices/src/motion_device.c")

    assert "memcmp(&state->last_motion_command" not in motion_c
    for token in [
        "state->last_motion_command.source != command->source",
        "state->last_motion_command.motion_mode != command->motion_mode",
        "state->last_motion_command.active_gear != command->active_gear",
        "state->last_motion_command.target_speed_mps != command->target_speed_mps",
        "state->last_motion_command.brake_release != command->brake_release",
        "state->last_motion_command.target_wheel_speed_mps[wheel]",
        "command->target_wheel_speed_mps[wheel]",
        "state->last_motion_command.target_steer_deg[wheel]",
        "command->target_steer_deg[wheel]",
    ]:
        assert token in motion_c, token


def test_canopen_command_cache_updates_only_after_successful_queueing(root: pathlib.Path) -> None:
    """A transient full SDO queue or offline node must not suppress the next retry."""

    config_h = read(root, "ecu/config/include/ecu_config.h")
    motion_h = read(root, "ecu/devices/include/motion_device.h")
    motion_c = read(root, "ecu/devices/src/motion_device.c")
    lift_h = read(root, "ecu/devices/include/lift_hydraulic_device.h")
    lift_c = read(root, "ecu/devices/src/lift_hydraulic_device.c")
    executor_c = read(root, "ecu/vehicle/src/vehicle_command_executor.c")

    for token in [
        "ECU_CANOPEN_MOTION_COMMAND_REFRESH_MS",
        "ECU_CANOPEN_LIFT_COMMAND_REFRESH_MS",
    ]:
        assert token in config_h, token

    for header_text, field in [
        (motion_h, "last_motion_command_queue_ms"),
        (lift_h, "last_lift_command_queue_ms"),
    ]:
        assert field in header_text, field

    motion_success_block = re.search(
        r"if \(ok\) \{[\s\S]*?state->last_motion_command = \*command;[\s\S]*?"
        r"state->last_motion_command_valid = true;[\s\S]*?"
        r"state->last_motion_command_queue_ms = now_ms;[\s\S]*?\n\s*\}",
        motion_c,
    )
    lift_success_block = re.search(
        r"if \(can3_actuator_command_changed\(state, command\)\) \{[\s\S]*?"
        r"state->last_lift_command = \*command;[\s\S]*?"
        r"state->last_lift_command_valid = true;[\s\S]*?"
        r"state->last_lift_command_queue_ms = now_ms;[\s\S]*?\n\s*\}",
        lift_c,
    )
    assert motion_success_block is not None
    assert lift_success_block is not None
    assert "motion_command_refresh_due(state, now_ms)" in motion_c
    assert "lift_command_refresh_due(state, now_ms)" not in lift_c
    assert "publish_motion_command_snapshot" in executor_c
    assert "read_motion_command_snapshot" in executor_c
    assert "command,\n                                                  command_sequence,\n                                                  now_ms)" in executor_c
    assert "publish_can3_command_snapshot" in executor_c
    assert "read_can3_command_snapshot" in executor_c
    assert "vehicle_command_executor_flush_can3_lift_hydraulic" in executor_c
    assert "lift_hydraulic_device_apply(&s_runtime.lift_hydraulic" in executor_c
    apply_body = re.search(
        r"bool vehicle_command_executor_apply[\s\S]*?"
        r"\n}\n\nbool vehicle_command_executor_flush_can2_motion",
        executor_c,
    )
    assert apply_body is not None
    assert "lift_hydraulic_device_apply(&s_runtime.lift_hydraulic" not in apply_body.group(0)
    assert "command,\n                                    now_ms)" in executor_c

    motion_after_failure = motion_c.split("state->last_result = ok ? ECU_DEVICE_APPLY_OK", 1)[0]
    lift_after_failure = lift_c.split("state->last_result = ok ? ECU_DEVICE_APPLY_OK", 1)[0]
    assert "state->last_motion_command = *command;" not in motion_after_failure.replace(
        motion_success_block.group(0), ""
    )
    assert "state->last_lift_command = *command;" not in lift_after_failure.replace(
        lift_success_block.group(0), ""
    )


def test_lift_can3_uses_standard_pdo_not_legacy_servo_sdo_helpers(root: pathlib.Path) -> None:
    """CAN3 lift/hydraulic motion must use the CAN3-owned PDO service path."""

    lift_c = read(root, "ecu/devices/src/lift_hydraulic_device.c")

    for forbidden in [
        "servo_drive_canopen_run_absolute_position_mode",
        "servo_drive_canopen_run_velocity_mode",
        "servo_drive_canopen_stop_velocity_mode",
        "servo_drive_canopen_send_control_word",
        "servo_drive_canopen_set_output_state",
        "servo_drive_canopen_read_input_states",
    ]:
        assert forbidden not in lift_c, forbidden

    assert "canopen_pdo_build_interpolated_position_rpdo2" in lift_c
    assert "canopen_pdo_build_velocity_rpdo0" in lift_c
    assert "CAN3-owned CANopen service" in lift_c


def test_commissioning_power_debug_can_request_hv_without_motion(root: pathlib.Path) -> None:
    """Bench commissioning can request high voltage without enabling actuators."""

    config_h = read(root, "ecu/config/include/ecu_config.h")
    helper_c = read(root, "ecu/diag/src/commissioning_debug.c")
    tasks_c = read(root, "ecu/os/src/ecu_tasks_cpu0.c")
    monitor_h = read(root, "ecu/diag/include/runtime_monitor.h")
    monitor_c = read(root, "ecu/diag/src/runtime_monitor.c")

    for token in [
        "ECU_ENABLE_COMMISSIONING_POWER_DEBUG",
        "ECU_COMMISSIONING_CONTROL_MAGIC",
        "ECU_COMMISSIONING_HV_REQUEST_TIMEOUT_MS",
        "ecu_commissioning_control_t",
    ]:
        assert token in config_h, token

    assert "volatile ecu_commissioning_control_t g_ecu_commissioning_control" in helper_c
    assert "commissioning_power_debug_is_allowed" in helper_c
    assert "commissioning_debug_apply_power_request" in helper_c
    control_step = re.search(
        r"void ecu_task_vehicle_control_step[\s\S]*?\n}\n\nvoid ecu_task_can1_power_step",
        tasks_c,
    )
    assert control_step, "missing vehicle control task body"
    control_body = control_step.group(0)
    assert control_body.index("safety_manager_apply(&safety_snapshot") < control_body.index(
        "commissioning_debug_apply_power_request"
    )
    assert control_body.index("commissioning_debug_apply_power_request") < control_body.index(
        "vehicle_command_executor_apply"
    )

    safety_block = re.search(r"commissioning_power_debug_is_allowed[\s\S]*?\n}", helper_c)
    assert safety_block, "missing commissioning safety gate"
    for token in [
        "!safety->a_class_fault",
        "!safety->shutdown_protect_active",
    ]:
        assert token in safety_block.group(0), token
    assert "!safety->estop_latched" not in safety_block.group(0)
    assert "remote/SBUS estop latch is allowed during no-remote commissioning" in helper_c

    apply_block = re.search(r"commissioning_debug_apply_power_request[\s\S]*?\n}", helper_c)
    assert apply_block, "missing commissioning apply gate"
    for token in [
        "command->high_voltage_enable = true",
        "command->brake_release = false",
        "command->hydraulic_enable = false",
        "command->hydraulic_valve_mask = 0U",
        "command->target_speed_mps = 0.0f",
        "command->target_wheel_speed_mps[wheel] = 0.0f",
    ]:
        assert token in apply_block.group(0), token

    assert "commissioning_power_debug_active" in monitor_h
    assert "comm_hv=%s" in monitor_c


def test_commissioning_canopen_scan_reads_all_servo_nodes(root: pathlib.Path) -> None:
    """Whole-machine communication checkout must read every configured servo node."""

    config_h = read(root, "ecu/config/include/ecu_config.h")
    helper_c = read(root, "ecu/diag/src/commissioning_debug.c")
    tasks_c = read(root, "ecu/os/src/ecu_tasks_cpu0.c")
    monitor_c = read(root, "ecu/diag/src/runtime_monitor.c")

    assert "ECU_ENABLE_COMMISSIONING_CANOPEN_SCAN" in config_h
    assert "ECU_CANOPEN_COMMISSIONING_SCAN_PERIOD_MS" in config_h
    assert "commissioning_debug_scan_can2" in helper_c
    assert "commissioning_debug_scan_can3" in helper_c
    assert "ECU_CANOPEN_OBJ_STATUSWORD" in helper_c
    can2_step = re.search(
        r"void ecu_task_can2_motion_step[\s\S]*?\n}\n\nvoid ecu_task_remote_manager_step",
        tasks_c,
    )
    can3_step = re.search(
        r"void ecu_task_can3_lift_hydraulic_step[\s\S]*?\n}\n\nvoid ecu_task_io_service_step",
        tasks_c,
    )
    assert can2_step and can3_step
    assert can2_step.group(0).index(
        "canopen_master_service_process_background(&s_runtime.can2_motion_canopen"
    ) < can2_step.group(0).index("commissioning_debug_scan_can2")
    assert can3_step.group(0).index(
        "canopen_master_service_process_background(&s_runtime.can3_lift_hydraulic_canopen"
    ) < can3_step.group(0).index("commissioning_debug_scan_can3")

    can2_func = helper_c[
        helper_c.index("void commissioning_debug_scan_can2"):
        helper_c.index("void commissioning_debug_scan_can3")
    ]
    can3_func = helper_c[
        helper_c.index("void commissioning_debug_scan_can3"):
        helper_c.index("void commissioning_debug_process_can4_physical_test")
    ]
    for token in ["drive_nodes", "steer_nodes"]:
        assert token in can2_func, token
    for token in ["lift_nodes", "hydraulic_pump_node"]:
        assert token in can3_func, token

    assert "last_node=%u" in monitor_c
    assert "abort=0x%08lx" in monitor_c


def test_commissioning_debug_helpers_are_isolated_from_cpu0_task(root: pathlib.Path) -> None:
    """Commissioning-only debug helpers stay out of the CPU0 task orchestrator."""

    helper_h = read(root, "ecu/diag/include/commissioning_debug.h")
    helper_c = read(root, "ecu/diag/src/commissioning_debug.c")
    tasks_c = read(root, "ecu/os/src/ecu_tasks_cpu0.c")
    cmake = read(root, "ecu/apps/agri_chassis_control_cpu0/CMakeLists.txt")

    for token in [
        "commissioning_debug_context_t",
        "commissioning_debug_init",
        "commissioning_debug_apply_power_request",
        "commissioning_debug_scan_can2",
        "commissioning_debug_scan_can3",
        "commissioning_debug_process_can4_physical_test",
        "commissioning_debug_get_can4_snapshot",
    ]:
        assert token in helper_h, token

    for token in [
        "volatile ecu_commissioning_control_t g_ecu_commissioning_control",
        "ECU_COMMISSIONING_CONTROL_MAGIC",
        "ECU_CAN4_PHYSICAL_TEST_FRAME_ID",
        "can_bus_service_send_frame",
        "canopen_master_service_request_sdo_read",
    ]:
        assert token in helper_c, token

    for implementation_detail in [
        "static void send_can4_physical_test_frame",
        "static void queue_can2_commissioning_scan",
        "static void queue_can3_commissioning_scan",
        "static void apply_commissioning_power_debug",
        "static bool commissioning_power_debug_is_allowed",
    ]:
        assert implementation_detail not in tasks_c, implementation_detail

    assert '#include "commissioning_debug.h"' in tasks_c
    assert "commissioning_debug_init(&s_runtime.commissioning_debug" in tasks_c
    assert "commissioning_debug_scan_can2(&s_runtime.commissioning_debug" in tasks_c
    assert "commissioning_debug_scan_can3(&s_runtime.commissioning_debug" in tasks_c
    assert "commissioning_debug_process_can4_physical_test(&s_runtime.commissioning_debug" in tasks_c
    assert "commissioning_debug_get_can4_snapshot(&s_runtime.commissioning_debug" in tasks_c
    assert "commissioning_debug.c" in cmake


def test_executor_fans_out_only_through_device_adapters(root: pathlib.Path) -> None:
    executor = read(root, "ecu/vehicle/src/vehicle_command_executor.c")
    required_calls = [
        "motion_device_apply",
        "lift_hydraulic_device_apply",
        "local_io_device_apply",
        "warning_light_device_apply",
    ]
    for token in required_calls:
        assert token in executor, token
    assert "executor->power_result" in executor
    for forbidden in ["hpm_can", "hpm_gpio", "board_", "HPM_CAN", "HPM_GPIO"]:
        assert forbidden not in executor, forbidden


def test_remote_and_control_do_not_depend_on_devices(root: pathlib.Path) -> None:
    forbidden = ["power_device", "motion_device", "lift_hydraulic_device", "local_io_device"]
    for folder in ["ecu/remote", "ecu/control"]:
        for path in (root / folder).rglob("*.[ch]"):
            text = path.read_text(encoding="utf-8")
            for token in forbidden:
                assert token not in text, f"{path}: forbidden {token}"


def test_no_raw_canopen_project_defaults_outside_config(root: pathlib.Path) -> None:
    canopen_default_pattern = re.compile(r"\b(0x18[0-9A-Fa-f]{2}|0x20[0-9A-Fa-f]{2}|0x60[0-9A-Fa-f]{2}|0x70[0-9A-Fa-f]{2})\b")
    for path in (root / "ecu").rglob("*.[ch]"):
        rel = path.relative_to(root).as_posix()
        if (
            rel.startswith("ecu/config/") or
            rel.startswith("ecu/protocol/canopen/od/") or
            rel.startswith("ecu/sdk_env_v1.11.0/")
        ):
            continue
        text = path.read_text(encoding="utf-8")
        match = canopen_default_pattern.search(text)
        assert match is None, f"{rel}: CANopen raw value {match.group(0)} belongs in config"


def test_dio_polarity_conversion_is_limited_to_managed_outputs(root: pathlib.Path) -> None:
    dio_h = read(root, "ecu/drivers/dio/include/dio_service.h")
    dio_c = read(root, "ecu/drivers/dio/src/dio_service.c")
    lift_c = read(root, "ecu/devices/src/lift_hydraulic_device.c")

    assert "managed_output_mask" in dio_h
    assert "(~logical) & service->managed_output_mask" in dio_c
    assert "dio_service_write_output_mask(dio, output_mask)" in read(
        root, "ecu/vehicle/src/vehicle_command_executor.c"
    )
    assert "dio_service_write_output_mask" not in lift_c


def test_lift_hydraulic_canopen_command_cache_includes_track_and_pump_intent(root: pathlib.Path) -> None:
    lift_c = read(root, "ecu/devices/src/lift_hydraulic_device.c")

    for token in [
        "state->last_lift_command.track_rate_mm_s != command->track_rate_mm_s",
        "state->last_lift_command.height_rate_mm_s != command->height_rate_mm_s",
        "state->last_lift_command.hydraulic_enable != command->hydraulic_enable",
        "state->last_lift_command.hydraulic_valve_mask != command->hydraulic_valve_mask",
    ]:
        assert token in lift_c, token
    assert "send_hydraulic_pump_command" not in lift_c
    assert "servo_drive_canopen_run_velocity_mode" not in lift_c


def test_hydraulic_pump_velocity_is_reverse_only_protected(root: pathlib.Path) -> None:
    """Node13 pump direction is safety-critical: positive motor velocity must never be emitted."""
    config_h = read(root, "ecu/config/include/ecu_config.h")
    lift_h = read(root, "ecu/devices/include/lift_hydraulic_device.h")
    lift_c = read(root, "ecu/devices/src/lift_hydraulic_device.c")

    assert "#define ECU_HYDRAULIC_PUMP_DIRECTION_SIGN (-1)" in config_h
    assert "#define ECU_HYDRAULIC_PUMP_ALLOW_POSITIVE_VELOCITY (0)" in config_h
    assert "hydraulic_pump_safe_velocity_units" in lift_h
    assert "hydraulic_pump_safe_velocity_units" in lift_c
    assert "ECU_HYDRAULIC_PUMP_DIRECTION_SIGN" in lift_c
    assert "ECU_HYDRAULIC_PUMP_ALLOW_POSITIVE_VELOCITY" in lift_c
    assert "ECU_HYDRAULIC_PUMP_MAX_REVERSE_VELOCITY_UNITS" in lift_c
    assert "ECU_HYDRAULIC_PUMP_MIN_WORK_VELOCITY_UNITS" in lift_c
    assert "requested_velocity_units <= 0" in lift_c
    assert "commanded > 0" in lift_c
    assert "return 0;" in lift_c
    assert "last_pump_controlword" in lift_h
    assert "last_pump_velocity_command_valid" in lift_h
    assert "state->last_pump_controlword == controlword" in lift_c
    assert "state->last_pump_controlword = state->pump_active_controlword" in lift_c
    assert "state->pump_active_controlword = controlword" in lift_c
    assert "state->last_pump_velocity_command_valid = true" in lift_c
    assert "#define ECU_HYDRAULIC_PUMP_WORK_RPM                  (1500.0f)" in config_h
    assert "#define ECU_HYDRAULIC_PUMP_TRACK_WIDTH_WORK_RPM      (2400.0f)" in config_h
    assert "#define ECU_HYDRAULIC_PUMP_MAX_REVERSE_RPM           (2400.0f)" in config_h
    assert "#define ECU_HYDRAULIC_PUMP_VALVE_OPEN_MIN_RPM        (800.0f)" in config_h
    assert "ECU_HYDRAULIC_PUMP_TRACK_WIDTH_VELOCITY_UNITS" in lift_c
    assert "track_width_valve_requested" in lift_c
    track_speed_selection = lift_c.split(
        "const bool track_width_valve_requested", 1
    )[1].split("ok = apply_hydraulic_pump_and_valves", 1)[0]
    assert "ECU_HYD_VALVE_TRACK_EXTEND_MASK" in track_speed_selection
    assert "ECU_HYD_VALVE_TRACK_RETRACT_MASK" in track_speed_selection
    assert "ECU_HYDRAULIC_PUMP_TRACK_WIDTH_VELOCITY_UNITS" in track_speed_selection
    assert "ECU_HYDRAULIC_PUMP_ENABLE_VELOCITY_UNITS" in track_speed_selection


def test_hydraulic_valves_wait_for_fresh_node13_reverse_speed_feedback(
    root: pathlib.Path,
) -> None:
    config_h = read(root, "ecu/config/include/ecu_config.h")
    lift_h = read(root, "ecu/devices/include/lift_hydraulic_device.h")
    lift_c = read(root, "ecu/devices/src/lift_hydraulic_device.c")

    for token in [
        "HYDRAULIC_PUMP_STATE_STOPPED",
        "HYDRAULIC_PUMP_STATE_STARTING",
        "HYDRAULIC_PUMP_STATE_VALVE_READY",
        "HYDRAULIC_PUMP_STATE_START_TIMEOUT",
        "pump_actual_velocity_units",
        "pump_feedback_valid",
        "pump_start_timeout_count",
    ]:
        assert token in lift_h, token

    vehicle_types_h = read(root, "ecu/vehicle/include/vehicle_types.h")
    executor_c = read(root, "ecu/vehicle/src/vehicle_command_executor.c")
    monitor_h = read(root, "ecu/diag/include/runtime_monitor.h")
    tasks_c = read(root, "ecu/os/src/ecu_tasks_cpu0.c")
    for token in [
        "hydraulic_pump_state",
        "hydraulic_pump_feedback_valid",
        "hydraulic_pump_actual_velocity_units",
        "hydraulic_pump_start_timeout_count",
    ]:
        assert token in vehicle_types_h, token
        assert token in executor_c, token
        assert token in monitor_h, token
        assert token in tasks_c, token

    for token in [
        "read_hydraulic_pump_feedback",
        "canopen_master_service_get_node_feedback",
        "feedback.tpdo0_valid",
        "feedback.last_tpdo0_ms",
        "feedback.tpdo1_valid",
        "feedback.fault_latched",
        "state->pump_actual_velocity_units <",
        "-ECU_HYDRAULIC_PUMP_VALVE_OPEN_MIN_VELOCITY_UNITS",
        "ECU_HYDRAULIC_PUMP_SPEED_READY_SAMPLES",
        "ECU_HYDRAULIC_PUMP_START_TIMEOUT_MS",
        "ECU_HYDRAULIC_VALVE_CHANGE_DEADTIME_MS",
        "close_hydraulic_valves(state, config)",
    ]:
        assert token in lift_c, token

    pump_feedback = lift_c.split("static bool read_hydraulic_pump_feedback", 1)[1]
    pump_feedback = pump_feedback.split("static void close_hydraulic_valves", 1)[0]
    assert "feedback.fault_latched" in pump_feedback
    assert "feedback.statusword" not in pump_feedback

    assert "ECU_HYDRAULIC_PUMP_VALVE_OPEN_MIN_VELOCITY_UNITS" in config_h
    assert "ECU_HYDRAULIC_PUMP_SPEED_LOSS_CONFIRM_MS" in config_h
    assert "state->pump_last_speed_ready_ms = now_ms" in lift_c
    speed_loss = lift_c.split(
        "ECU_HYDRAULIC_PUMP_SPEED_LOSS_CONFIRM_MS", 1
    )[1].split("const can3_pdo_submit_result_t pump_submit_result", 1)[0]
    assert "close_hydraulic_valves(state, config)" in speed_loss
    assert "state->last_pump_velocity_command_valid = false" in speed_loss


def test_can3_lift_interpolation_uses_feedback_and_sync_rpdo2(root: pathlib.Path) -> None:
    config_h = read(root, "ecu/config/include/ecu_config.h")
    pdo_h = read(root, "ecu/drivers/canopen/include/canopen_pdo_profile.h")
    pdo_c = read(root, "ecu/drivers/canopen/src/canopen_pdo_profile.c")
    service_h = read(root, "ecu/drivers/canopen/include/canopen_master_service.h")
    service_c = read(root, "ecu/drivers/canopen/src/canopen_master_service.c")
    lift_c = read(root, "ecu/devices/src/lift_hydraulic_device.c")
    arbiter_c = read(root, "ecu/vehicle/src/command_arbiter.c")

    for token in [
        "ECU_CANOPEN_LIFT_INTERPOLATION_PERIOD_MS",
        "ECU_LIFT_SHORTEST_POSITION_COUNTS",
        "ECU_LIFT_LONGEST_POSITION_COUNTS",
        "ECU_REMOTE_CLEARANCE_UP_PER_MILLE_MIN",
        "ECU_REMOTE_CLEARANCE_DOWN_PER_MILLE_MAX",
    ]:
        assert token in config_h

    assert "CANOPEN_PDO_MODE_INTERPOLATED_POSITION (7)" in pdo_h
    assert "rpdo2_cob_id" in pdo_h
    assert "canopen_pdo_build_interpolated_position_rpdo2" in pdo_h
    assert "profile->required_mode != CANOPEN_PDO_MODE_INTERPOLATED_POSITION" in pdo_c
    assert "CANOPEN_MASTER_PDO_PHASE_LIFT_INTERPOLATION_POINT" in service_h
    assert "CANOPEN_MASTER_PDO_PHASE_LIFT_INTERPOLATION_POINT" in service_c
    assert (
        "phase == CANOPEN_MASTER_PDO_PHASE_LIFT_INTERPOLATION_TRIGGER ||"
        in service_c
    )

    for token in [
        "refresh_lift_feedback",
        "canopen_master_service_get_node_feedback",
        "ECU_CANOPEN_OBJ_INTERPOLATION_TIME_PERIOD",
        "ECU_CANOPEN_OBJ_INTERPOLATION_MODE",
        "canopen_pdo_build_interpolated_position_rpdo2",
        "canopen_master_service_queue_pdo_batch_with_descriptor",
        ".sync_after_arm = true",
        "lift_target_counts_from_mm",
        "lift_positions_at_target",
        "lift_progress_stalled",
        "recover_lift_transport_state",
        "canopen_master_service_recover_transport",
        "lift_interpolation_recovery_count",
        "command->target_height_mm",
        "command->high_voltage_feedback_ready",
    ]:
        assert token in lift_c

    interpolation_mode_write = lift_c.split(
        "ECU_CANOPEN_OBJ_INTERPOLATION_MODE", 1
    )[1].split(") && node_ok;", 1)[0]
    assert "2U," in interpolation_mode_write

    assert "height_rate = ECU_REMOTE_MAX_HEIGHT_RATE_MM_S" in arbiter_c
    assert "height_rate = -ECU_REMOTE_MAX_HEIGHT_RATE_MM_S" in arbiter_c
    assert "out->target_height_mm = ECU_REMOTE_MAX_HEIGHT_TARGET_MM" in arbiter_c
    assert "out->target_height_mm = ECU_REMOTE_MIN_HEIGHT_TARGET_MM" in arbiter_c
    physical_up_block = arbiter_c.split(
        "remote->clearance_per_mille <= ECU_REMOTE_CLEARANCE_DOWN_PER_MILLE_MAX",
        1,
    )[1].split("} else if", 1)[0]
    physical_down_block = arbiter_c.split(
        "remote->clearance_per_mille >= ECU_REMOTE_CLEARANCE_UP_PER_MILLE_MIN",
        1,
    )[1].split("uint32_t hydraulic_valve_mask", 1)[0]
    assert "right stick up   -> extend legs toward 490 mm" in physical_up_block
    assert "height_rate = ECU_REMOTE_MAX_HEIGHT_RATE_MM_S" in physical_up_block
    assert "out->target_height_mm = ECU_REMOTE_MAX_HEIGHT_TARGET_MM" in physical_up_block
    assert "height_rate = -ECU_REMOTE_MAX_HEIGHT_RATE_MM_S" in physical_down_block
    assert "out->target_height_mm = ECU_REMOTE_MIN_HEIGHT_TARGET_MM" in physical_down_block
    assert "out->hydraulic_enable = hydraulic_valve_mask != 0U;" in arbiter_c
    assert "Ground-clearance lift is electric CAN3 servo motion" in arbiter_c
    assert "ECU_CANOPEN_PUMP_VELOCITY_REFRESH_MS" not in config_h
    pump_queue = lift_c.split("static can3_pdo_submit_result_t queue_hydraulic_pump_velocity", 1)[1]
    pump_queue = pump_queue.split("static bool hydraulic_pump_stop_confirmed", 1)[0]
    unchanged = pump_queue.split(
        "state->last_pump_velocity_command_valid", 1
    )[1].split("if (!canopen_master_service_realtime_pdo_idle", 1)[0]
    assert "state->last_pump_controlword == controlword" in unchanged
    assert "state->last_pump_velocity_units == target_velocity_units" in unchanged
    assert "now_ms - state->last_pump_velocity_ms" not in unchanged


def test_can3_lift_direction_change_clears_buffer_outside_operation_enabled(
    root: pathlib.Path,
) -> None:
    config_h = read(root, "ecu/config/include/ecu_config.h")
    lift_h = read(root, "ecu/devices/include/lift_hydraulic_device.h")
    lift_c = read(root, "ecu/devices/src/lift_hydraulic_device.c")

    for token in [
        "LIFT_INTERPOLATION_STATE_CONFIGURING",
        "LIFT_INTERPOLATION_STATE_PRELOADING",
        "LIFT_INTERPOLATION_STATE_TRIGGERING",
        "LIFT_INTERPOLATION_STATE_RUNNING",
        "LIFT_INTERPOLATION_STATE_STOPPING",
        "LIFT_INTERPOLATION_STATE_FAULT",
    ]:
        assert token in lift_h

    setup = lift_c.split("static bool queue_lift_setup_sdos", 1)[1].split(
        "static bool begin_lift_interpolation_setup", 1
    )[0]
    shutdown_at = setup.index("SERVO_DRIVE_CONTROL_SHUTDOWN")
    vendor_option_at = setup.index("ECU_CANOPEN_OBJ_BC_INTERPOLATION_OPTION")
    mode_select_at = setup.index("ECU_CANOPEN_OBJ_MODES_OF_OPERATION")
    interpolation_mode_at = setup.index("ECU_CANOPEN_OBJ_INTERPOLATION_MODE")
    interpolation_period_at = setup.index("ECU_CANOPEN_OBJ_INTERPOLATION_TIME_PERIOD")
    clear_at = setup.index("ECU_CANOPEN_OBJ_INTERPOLATION_BUFFER_CLEAR")
    assert (
        shutdown_at
        < vendor_option_at
        < mode_select_at
        < interpolation_mode_at
        < interpolation_period_at
        < clear_at
    )
    assert "SERVO_DRIVE_CONTROL_SWITCH_ON" not in setup
    assert "SERVO_DRIVE_CONTROL_ENABLE_OPERATION" not in setup
    assert "LIFT_SETUP_SDO_WRITES_ENABLED_PER_NODE (11U)" in lift_c
    assert "ECU_CANOPEN_OBJ_INTERPOLATION_MODE" in setup
    assert "0U, 2U, ECU_CANOPEN_LIFT_INTERPOLATION_MODE" in setup
    assert "ECU_CANOPEN_OBJ_BC_INTERPOLATION_OPTION" in setup
    assert "ECU_CANOPEN_BC_INTERPOLATION_OPTION_VALUE" in setup
    allowed_sdo = read(
        root, "ecu/drivers/canopen/src/canopen_master_service.c"
    ).split("static bool canopen_master_sdo_write_allowed", 1)[1].split(
        "static bool canopen_master_sdo_write_requires_order", 1
    )[0]
    assert "ECU_CANOPEN_OBJ_BC_INTERPOLATION_OPTION" in allowed_sdo
    for token in [
        "ECU_CANOPEN_OBJ_PROFILE_VELOCITY",
        "ECU_LIFT_PROFILE_VELOCITY_UNITS",
        "ECU_CANOPEN_OBJ_PROFILE_ACCELERATION",
        "ECU_CANOPEN_OBJ_PROFILE_DECELERATION",
        "ECU_LIFT_PROFILE_ACCEL_UNITS",
        "ECU_CANOPEN_OBJ_FOLLOWING_ERROR_WINDOW",
        "ECU_LIFT_FOLLOWING_ERROR_WINDOW_COUNTS",
    ]:
        assert token in setup
    assert "#define ECU_CANOPEN_LIFT_INTERPOLATION_MODE       (0)" in config_h
    assert "ECU_CANOPEN_LIFT_PRELOAD_POINTS" in lift_c
    assert "lift_preload_group_pending" in lift_h
    assert "state->lift_preload_group_pending = true" in lift_c
    assert "state->lift_preload_points_completed++" in lift_c
    assert "lift_setup_nmt_sent_mask" in lift_h
    assert "CANOPEN_MASTER_PDO_PHASE_LIFT_INTERPOLATION_TRIGGER" in lift_c
    assert "SERVO_DRIVE_CONTROL_ENABLE_OPERATION" in lift_c
    assert "state->lift_requested_direction !=" in lift_c
    assert "ECU_LIFT_INTERPOLATION_TARGET_LEAD_COUNTS" in lift_c
    assert "SERVO_DRIVE_CONTROL_DISABLE_VOLTAGE" in lift_c
    assert "state->lift_progress_initialized = false" in lift_c
    assert "ECU_LIFT_TARGET_REACHED_TOLERANCE_COUNTS" in config_h
    assert "ECU_LIFT_STALL_PROGRESS_COUNTS" in config_h
    assert "ECU_LIFT_STALL_TIMEOUT_MS" in config_h
    assert "canopen_master_service_sdo_download_idle" in lift_c

    assert "#define ECU_CANOPEN_LIFT_PRELOAD_POINTS           (3U)" in config_h
    assert "#define ECU_CANOPEN_LIFT_ENABLE_SETTLE_MS         (1500U)" in config_h
    assert "lift_enable_settle_until_ms" in lift_h
    assert "ECU_CANOPEN_LIFT_ENABLE_SETTLE_MS" in lift_c
    assert "refresh_lift_feedback(state, canopen, config, now_ms)" in lift_c
    assert (
        "if (!queue_lift_interpolation_group(\n"
        "                state,\n"
        "                canopen,\n"
        "                config,\n"
        "                LIFT_INTERP_DIRECTION_HOLD,"
    ) in lift_c
    assert "ECU_LIFT_INTERPOLATION_SPEED_COUNTS_PER_SEC \\" in config_h
    assert "(ECU_LIFT_MM_TO_COUNTS * 6.0f)" in config_h
    assert "ECU_LIFT_INTERPOLATION_ACCEL_COUNTS_PER_SEC2" in config_h
    assert "(ECU_LIFT_MM_TO_COUNTS * 8.0f)" in config_h
    assert "#define ECU_LIFT_PROFILE_VELOCITY_UNITS             (24000000)" in config_h
    assert "#define ECU_LIFT_PROFILE_ACCEL_UNITS                (500000)" in config_h
    assert "ECU_LIFT_FOLLOWING_ERROR_WINDOW_COUNTS \\" in config_h
    assert "(ECU_LIFT_MM_TO_COUNTS * 3.0f)" in config_h
    assert "ECU_LIFT_INTERPOLATION_TARGET_LEAD_COUNTS \\" in config_h
    assert "#define ECU_LIFT_INTERPOLATION_TARGET_LEAD_COUNTS \\" in config_h
    assert "    (75000)" in config_h
    assert "ECU_LIFT_FINAL_SPREAD_TOLERANCE_COUNTS \\" in config_h
    assert "(ECU_LIFT_MM_TO_COUNTS * 3.0f)" in config_h
    assert "ECU_LIFT_RUNNING_SPREAD_WARNING_COUNTS \\" in config_h
    assert "(ECU_LIFT_MM_TO_COUNTS * 15.0f)" in config_h
    assert "ECU_LIFT_REMOTE_NEUTRAL_STOP_CONFIRM_MS" in config_h
    assert "lift_remote_neutral_stop_confirmed" in lift_c
    assert "command->source != COMMAND_SOURCE_REMOTE" in lift_c
    assert "lift_remote_neutral_since_ms" in lift_h
    assert "effective_target_height_mm" in lift_c
    assert "ECU_REMOTE_MAX_HEIGHT_TARGET_MM : ECU_REMOTE_MIN_HEIGHT_TARGET_MM" in lift_c
    running_sync = lift_c.split("static int32_t lift_interpolation_delta_counts", 1)[1]
    running_sync = running_sync.split("static bool lift_positions_at_target", 1)[0]
    final_sync = lift_c.split("static bool lift_positions_at_target", 1)[1]
    final_sync = final_sync.split("static bool lift_progress_stalled", 1)[0]
    assert "ECU_LIFT_SYNC_RUNNING_TOLERANCE_COUNTS" not in running_sync
    assert "ECU_LIFT_SYNC_RECOVERY_TOLERANCE_COUNTS" not in running_sync
    assert "ECU_LIFT_FINAL_SPREAD_TOLERANCE_COUNTS" in final_sync
    assert "update_lift_running_spread_diagnostics" in running_sync
    spread_diagnostics = lift_c.split(
        "static void update_lift_running_spread_diagnostics", 1
    )[1].split("static bool lift_positions_at_target", 1)[0]
    assert "ECU_LIFT_RUNNING_SPREAD_WARNING_COUNTS" in spread_diagnostics
    assert "LIFT_INTERPOLATION_STATE_STOPPING" not in spread_diagnostics
    assert "LIFT_INTERPOLATION_STATE_FAULT" not in spread_diagnostics
    assert "recover_lift_" not in spread_diagnostics
    assert "lift_running_spread_counts" in lift_h
    assert "lift_max_running_spread_counts" in lift_h
    assert "lift_running_spread_warning_count" in lift_h
    assert "lift_level_target_from_feedback" in lift_c
    assert "lift_level_target_position_counts" in lift_h
    assert "LIFT_INTERPOLATION_STATE_LEVELING" in lift_c
    assert "LIFT_INTERPOLATION_STATE_READY_TO_SWITCH_ON" in lift_h
    assert "LIFT_INTERPOLATION_STATE_SWITCHING_ON" in lift_h
    assert "LIFT_INTERPOLATION_STATE_ENABLING_OPERATION" in lift_h
    assert "SERVO_DRIVE_CONTROL_SWITCH_ON" in lift_c
    assert "queue_lift_interpolation_trigger" in lift_c
    assert "lift_stream_total_distance" in lift_c
    assert "remaining_counts" in lift_c
    assert "#define ECU_REMOTE_MIN_HEIGHT_TARGET_MM   (10.0f)" in config_h
    assert "#define ECU_REMOTE_MAX_HEIGHT_TARGET_MM   (490.0f)" in config_h


def test_can3_lift_group_sync_does_not_compete_with_periodic_feedback_sync(
    root: pathlib.Path,
) -> None:
    tasks_c = read(root, "ecu/os/src/ecu_tasks_cpu0.c")
    service_h = read(
        root, "ecu/drivers/canopen/include/canopen_master_service.h"
    )
    service_c = read(root, "ecu/drivers/canopen/src/canopen_master_service.c")
    step = re.search(
        r"void ecu_task_can3_lift_hydraulic_step[\s\S]*?\n}\n\n"
        r"void ecu_task_io_service_step",
        tasks_c,
    )
    assert step
    body = step.group(0)
    flush_at = body.index("vehicle_command_executor_flush_can3_lift_hydraulic")
    idle_at = body.index("canopen_master_service_realtime_pdo_idle")
    sync_at = body.index("canopen_master_service_send_feedback_sync", idle_at)
    assert flush_at < idle_at < sync_at
    assert "snapshot.last_sync_tx_ms" in body
    assert "can3_lift_realtime_window_active" in tasks_c
    assert "canopen_master_service_set_periodic_sdo_enabled" in body
    assert "!lift_realtime_window_active" in body
    assert "commissioning_debug_scan_can3" in body
    assert "canopen_master_service_realtime_pdo_idle" in service_h
    assert "canopen_master_service_send_feedback_sync" in service_h
    assert "canopen_master_service_sdo_download_idle" in service_h
    assert "canopen_master_service_set_periodic_sdo_enabled" in service_h
    assert "service->periodic_sdo_enabled = true" in service_c
    assert "service->periodic_sdo_enabled &&" in service_c
    assert "service == NULL || !service->periodic_sdo_enabled" in service_c


def test_can3_stopped_pump_does_not_inject_periodic_sync_into_lift_stream(
    root: pathlib.Path,
) -> None:
    lift_c = read(root, "ecu/devices/src/lift_hydraulic_device.c")

    assert "static bool hydraulic_pump_stop_confirmed" in lift_c
    helper = lift_c.split("static bool hydraulic_pump_stop_confirmed", 1)[1]
    helper = helper.split("static bool ensure_hydraulic_pump_velocity_setup", 1)[0]
    assert "state->last_pump_velocity_command_valid" in helper
    assert "SERVO_DRIVE_CONTROL_DISABLE_VOLTAGE" in helper

    stopped = lift_c.split("if (!pump_request)", 1)[1]
    stopped = stopped.split("if (!ensure_hydraulic_pump_velocity_setup", 1)[0]
    assert "hydraulic_pump_stop_confirmed(state)" in stopped
    assert stopped.index("hydraulic_pump_stop_confirmed(state)") < stopped.index(
        "queue_hydraulic_pump_velocity"
    )


def test_can3_pump_setup_completion_starts_fresh_feedback_timeout_window(
    root: pathlib.Path,
) -> None:
    lift_c = read(root, "ecu/devices/src/lift_hydraulic_device.c")

    setup = lift_c.split("static bool ensure_hydraulic_pump_velocity_setup", 1)[1]
    setup = setup.split("static bool read_hydraulic_pump_feedback", 1)[0]
    completed = setup.split("servo_setup_feedback_allows_ready", 1)[1]
    completed = completed.split("return true;", 1)[0]
    assert "state->pump_state = HYDRAULIC_PUMP_STATE_STARTING" in completed
    assert "state->pump_start_request_ms = now_ms" in completed
    assert "state->pump_speed_ready_samples = 0U" in completed
    assert "state->pump_pressure_ready = false" in completed


def test_can3_lift_setup_uses_sdo_completion_not_sparse_tpdo1_as_hard_gate(
    root: pathlib.Path,
) -> None:
    lift_c = read(root, "ecu/devices/src/lift_hydraulic_device.c")

    helper = lift_c.split("static bool servo_setup_feedback_allows_ready", 1)[1]
    helper = helper.split("static bool lift_axes_setup_feedback_ready", 1)[0]
    assert "lack of a fresh TPDO1 is not by itself" in lift_c
    assert "feedback.tpdo1_valid &&" in helper
    assert "feedback.fault_latched != 0U" in helper
    assert "0x162F" in helper
    assert "feedback.statusword" not in helper
    assert "SERVO_DRIVE_STATUS_OPERATION_ENABLED_MASK" not in lift_c

    lift_ready = lift_c.split("static bool lift_axes_setup_feedback_ready", 1)[1]
    lift_ready = lift_ready.split("static bool queue_lift_setup_sdos", 1)[0]
    pump_setup = lift_c.split("static bool ensure_hydraulic_pump_velocity_setup", 1)[1]
    pump_setup = pump_setup.split("static bool read_hydraulic_pump_feedback", 1)[0]
    assert "servo_setup_feedback_allows_ready(" in lift_ready
    assert "servo_setup_feedback_allows_ready(canopen, node->node_id)" in pump_setup


def test_can3_lift_clears_vendor_fault_and_recovers_without_transport_reset(
    root: pathlib.Path,
) -> None:
    lift_h = read(root, "ecu/devices/include/lift_hydraulic_device.h")
    lift_c = read(root, "ecu/devices/src/lift_hydraulic_device.c")
    vehicle_h = read(root, "ecu/vehicle/include/vehicle_types.h")
    executor_c = read(root, "ecu/vehicle/src/vehicle_command_executor.c")
    monitor_h = read(root, "ecu/diag/include/runtime_monitor.h")
    tasks_c = read(root, "ecu/os/src/ecu_tasks_cpu0.c")
    monitor_c = read(root, "ecu/diag/src/runtime_monitor.c")

    assert "lift_axis_fault_mask" in lift_h
    for source in [vehicle_h, executor_c, monitor_h, tasks_c, monitor_c]:
        assert "lift_axis_fault_mask" in source
    setup = lift_c.split("static bool queue_lift_setup_sdos", 1)[1]
    setup = setup.split("static bool begin_lift_interpolation_setup", 1)[0]
    assert "feedback.fault_latched != 0U" in setup
    assert "ECU_CANOPEN_OBJ_FAULT_LATCHED" in setup
    assert "expected_writes++" in setup

    refresh = lift_c.split("static bool refresh_lift_feedback", 1)[1]
    refresh = refresh.split("static bool servo_setup_feedback_allows_ready", 1)[0]
    assert "state->lift_axis_fault_mask" in refresh
    assert "axis_fault" in refresh

    running = lift_c.split("static bool queue_lift_interpolation_group", 1)[1]
    running = running.split("static bool queue_lift_interpolation_trigger", 1)[0]
    fault_block = running.split("state->lift_axis_fault_mask != 0U", 1)[1]
    fault_block = fault_block.split("if (state->lift_feedback_missing_since_ms", 1)[0]
    assert "SERVO_DRIVE_CONTROL_DISABLE_VOLTAGE" in fault_block
    assert "LIFT_INTERPOLATION_STATE_STOPPING" in fault_block
    assert "recover_lift_transport_state" not in fault_block


def test_can3_lift_setup_failure_retries_without_reinitializing_can_controller(
    root: pathlib.Path,
) -> None:
    lift_h = read(root, "ecu/devices/include/lift_hydraulic_device.h")
    lift_c = read(root, "ecu/devices/src/lift_hydraulic_device.c")

    assert "lift_transport_recovery_required" in lift_h
    group_completion = lift_c.split("static bool lift_group_completed", 1)[1]
    group_completion = group_completion.split(
        "static bool queue_lift_interpolation_trigger", 1
    )[0]
    assert "state->lift_transport_recovery_required = true" in group_completion

    setup_completion = lift_c.split("static bool lift_setup_completed", 1)[1]
    setup_completion = setup_completion.split(
        "static int32_t lift_interpolation_delta_counts", 1
    )[0]
    assert "LIFT_INTERPOLATION_STATE_FAULT" in setup_completion
    assert "lift_transport_recovery_required = true" not in setup_completion

    fault_state = lift_c.rsplit(
        "if (state->lift_interpolation_state == LIFT_INTERPOLATION_STATE_FAULT)", 1
    )[1].split("LIFT_INTERPOLATION_STATE_CONFIGURING", 1)[0]
    assert "state->lift_transport_recovery_required" in fault_state
    assert "recover_lift_transport_state" in fault_state
    assert "recover_lift_control_state" in fault_state


def test_can3_lift_missing_tpdo0_pauses_interpolation_instead_of_disabling_axes(
    root: pathlib.Path,
) -> None:
    lift_h = read(root, "ecu/devices/include/lift_hydraulic_device.h")
    lift_c = read(root, "ecu/devices/src/lift_hydraulic_device.c")

    assert "lift_feedback_missing_since_ms" in lift_h
    running = lift_c.split("static bool queue_lift_interpolation_group", 1)[1].split(
        "static void begin_lift_leveling", 1
    )[0]
    missing_feedback_block = running.split(
        "if (!refresh_lift_feedback(state, canopen, config, now_ms))", 1
    )[1].split("update_lift_position_range_masks(state)", 1)[0]
    assert "state->lift_feedback_missing_since_ms = now_ms" in missing_feedback_block
    assert "recover_lift_transport_state(state, canopen, now_ms)" in missing_feedback_block
    assert "ECU_CANOPEN_LIFT_SETUP_TIMEOUT_MS" in missing_feedback_block
    non_fault_missing = missing_feedback_block.split(
        "if (state->lift_feedback_missing_since_ms == 0U)", 1
    )[1]
    assert "SERVO_DRIVE_CONTROL_DISABLE_VOLTAGE" not in non_fault_missing


def test_can3_lift_progress_stall_soft_recovers_without_axis_disable(
    root: pathlib.Path,
) -> None:
    lift_c = read(root, "ecu/devices/src/lift_hydraulic_device.c")

    stall_block = lift_c.split(
        "if (lift_progress_stalled(state, command_target_position_counts, now_ms))",
        1,
    )[1].split("for (uint32_t leg = 0U; leg < ECU_WHEEL_COUNT; ++leg)", 1)[0]
    assert "visible jerk" in stall_block
    assert "same absolute-position stream" in stall_block
    assert "state->lift_interpolation_reject_count++" in stall_block
    assert "lift_progress_watchdog_rebaseline(state, now_ms)" in stall_block
    assert "lift_stream_reset_trajectory(state, now_ms)" not in stall_block
    assert "SERVO_DRIVE_CONTROL_DISABLE_VOLTAGE" not in stall_block
    assert "LIFT_INTERPOLATION_STATE_STOPPING" not in stall_block
    assert "state->lift_recovery_not_before_ms" not in stall_block


def test_can3_lift_uses_script_equivalent_per_axis_trajectory_without_pretrigger_leveling(
    root: pathlib.Path,
) -> None:
    config_h = read(root, "ecu/config/include/ecu_config.h")
    lift_h = read(root, "ecu/devices/include/lift_hydraulic_device.h")
    lift_c = read(root, "ecu/devices/src/lift_hydraulic_device.c")

    assert "#define ECU_CANOPEN_LIFT_PRELOAD_POINTS           (3U)" in config_h
    assert "(75000)" in config_h.split(
        "#define ECU_LIFT_INTERPOLATION_TARGET_LEAD_COUNTS", 1
    )[1].split("#define ECU_LIFT_SYNC_CORRECTION_GAIN_NUMERATOR", 1)[0]
    assert "ECU_LIFT_SYNC_CORRECTION_GAIN_NUMERATOR" in config_h
    assert "ECU_LIFT_SYNC_CORRECTION_MAX_COUNTS" in config_h
    assert "LIFT_INTERPOLATION_STATE_LEVELING" in lift_h
    assert "lift_stream_total_distance_counts" in lift_h
    assert "const int32_t final_delta = command_target_position_counts - origin" in lift_c
    assert "((int64_t)final_delta *" in lift_c
    assert "ECU_LIFT_SYNC_CORRECTION_GAIN_NUMERATOR" in lift_c
    settling = lift_c.split(
        "LIFT_INTERPOLATION_STATE_SETTLING", 2
    )[2].split("if (state->lift_interpolation_state == LIFT_INTERPOLATION_STATE_STOPPED)", 1)[0]
    assert "LIFT_INTERPOLATION_STATE_PRELOADING" in settling
    assert "state->lift_start_with_leveling" in settling
    preloading = lift_c.split(
        "LIFT_INTERPOLATION_STATE_PRELOADING", 2
    )[2].split("LIFT_INTERPOLATION_STATE_TRIGGERING", 1)[0]
    assert "begin_lift_leveling" not in preloading
    triggering = lift_c.split(
        "LIFT_INTERPOLATION_STATE_TRIGGERING", 2
    )[2].split("LIFT_INTERPOLATION_STATE_RUNNING", 1)[0]
    assert "state->lift_start_with_leveling" in triggering
    assert "begin_lift_leveling(state, requested_direction)" in triggering


def test_can3_lift_neutral_during_setup_runs_disable_setup_before_stopped(
    root: pathlib.Path,
) -> None:
    lift_c = read(root, "ecu/devices/src/lift_hydraulic_device.c")

    neutral_configuring = lift_c.split(
        "if (safe_stop_requested || neutral_level_requested ||", 1
    )[1].split(
        "if (state->lift_interpolation_state ==\n"
        "                LIFT_INTERPOLATION_STATE_STOPPED)",
        1,
    )[0]
    assert "const bool setup_was_enabling" in neutral_configuring
    assert "operator returned the stick to neutral" in neutral_configuring
    assert "LIFT_INTERP_DIRECTION_HOLD,\n                        false" in neutral_configuring
    assert "LIFT_INTERP_DIRECTION_HOLD" in neutral_configuring


def test_can3_lift_waits_for_measured_axis_settle_before_capturing_origin(
    root: pathlib.Path,
) -> None:
    config_h = read(root, "ecu/config/include/ecu_config.h")
    lift_h = read(root, "ecu/devices/include/lift_hydraulic_device.h")
    lift_c = read(root, "ecu/devices/src/lift_hydraulic_device.c")

    for token in [
        "ECU_CANOPEN_LIFT_SETTLE_SAMPLE_MS",
        "ECU_CANOPEN_LIFT_SETTLE_STABLE_MS",
        "ECU_CANOPEN_LIFT_SETTLE_MAX_DRIFT_COUNTS",
        "ECU_CANOPEN_LIFT_SETTLE_TIMEOUT_MS",
    ]:
        assert token in config_h, token
    for token in [
        "lift_settle_reference_position_counts",
        "lift_settle_stable_since_ms",
        "lift_settle_sample_ms",
        "lift_settle_initialized",
    ]:
        assert token in lift_h, token
    assert "lift_axes_settled" in lift_c
    settling = lift_c.split(
        "LIFT_INTERPOLATION_STATE_SETTLING", 2
    )[2].split("if (state->lift_interpolation_state == LIFT_INTERPOLATION_STATE_STOPPED)", 1)[0]
    assert "lift_axes_settled(state, now_ms)" in settling
    assert settling.index("refresh_lift_feedback") < settling.index("lift_axes_settled")


def test_can3_lift_new_reducer_neutral_leveling_and_range_direction_contract(
    root: pathlib.Path,
) -> None:
    config_h = read(root, "ecu/config/include/ecu_config.h")
    vehicle_h = read(root, "ecu/vehicle/include/vehicle_types.h")
    arbiter_c = read(root, "ecu/vehicle/src/command_arbiter.c")
    safety_c = read(root, "ecu/vehicle/src/safety_manager.c")
    lift_h = read(root, "ecu/devices/include/lift_hydraulic_device.h")
    lift_c = read(root, "ecu/devices/src/lift_hydraulic_device.c")
    tasks_c = read(root, "ecu/os/src/ecu_tasks_cpu0.c")

    assert "#define ECU_LIFT_MOTOR_REVS_PER_MM                   (2.0f)" in config_h
    assert "(ECU_LIFT_MM_TO_COUNTS * 6.0f)" in config_h
    assert "(ECU_LIFT_MM_TO_COUNTS * 2.0f)" in config_h
    assert "(ECU_LIFT_MM_TO_COUNTS * 4.0f)" in config_h
    assert "(ECU_LIFT_MM_TO_COUNTS * 15.0f)" in config_h
    assert "ECU_LIFT_LEVELING_STABLE_SAMPLES" in config_h
    assert "ECU_LIFT_ZERO_SPEED_VELOCITY_UNITS" in config_h

    for token in [
        "VEHICLE_LIFT_REQUEST_SAFE_STOP",
        "VEHICLE_LIFT_REQUEST_NEUTRAL_LEVEL",
        "VEHICLE_LIFT_REQUEST_EXTEND",
        "VEHICLE_LIFT_REQUEST_RETRACT",
    ]:
        assert token in vehicle_h
    assert "out->lift_request = VEHICLE_LIFT_REQUEST_NEUTRAL_LEVEL" in arbiter_c
    assert "out->lift_request = VEHICLE_LIFT_REQUEST_EXTEND" in arbiter_c
    assert "out->lift_request = VEHICLE_LIFT_REQUEST_RETRACT" in arbiter_c
    assert "command->lift_request = VEHICLE_LIFT_REQUEST_SAFE_STOP" in safety_c

    for token in [
        "update_lift_position_range_masks",
        "lift_direction_allowed_by_range",
        "lift_below_safe_range_mask",
        "lift_above_safe_range_mask",
        "lift_mechanical_range_invalid_mask",
        "lift_level_target_from_feedback",
        "lift_level_next_target",
        "lift_level_feedback_stable",
        "queue_lift_leveling_group",
        "LIFT_INTERPOLATION_STATE_LEVELING",
    ]:
        assert token in lift_c or token in lift_h, token
    range_gate = lift_c.split("static bool lift_direction_allowed_by_range", 1)[1].split(
        "static int32_t lift_level_target_from_feedback", 1
    )[0]
    assert "LIFT_INTERP_DIRECTION_EXTEND" in range_gate
    assert "LIFT_INTERP_DIRECTION_RETRACT" in range_gate
    assert "return direction == LIFT_INTERP_DIRECTION_EXTEND;" in range_gate
    assert "return direction == LIFT_INTERP_DIRECTION_RETRACT;" in range_gate
    assert "lift_below_safe_range_mask != 0U &&" in range_gate
    assert "lift_above_safe_range_mask != 0U" in range_gate
    planner = lift_c.split("static int32_t lift_level_next_target", 1)[1].split(
        "static bool lift_level_feedback_stable", 1
    )[0]
    assert "state->lift_target_position_counts[leg]" in planner
    assert "approach_zero" in planner
    assert "ECU_LIFT_LEVELING_ACCEL_COUNTS_PER_SEC2" in planner
    assert "ECU_LIFT_LEVELING_SPEED_COUNTS_PER_SEC" in planner
    assert "ECU_LIFT_ZERO_SPEED_VELOCITY_UNITS" in lift_c
    for token in [
        "LIFT_INTERPOLATION_STATE_READY_TO_SWITCH_ON",
        "LIFT_INTERPOLATION_STATE_SWITCHING_ON",
        "LIFT_INTERPOLATION_STATE_ENABLING_OPERATION",
        "LIFT_INTERPOLATION_STATE_LEVELING",
    ]:
        assert token in tasks_c


def test_current_documentation_uses_one_lift_contract_and_index(root: pathlib.Path) -> None:
    current_paths = [
        "AGENTS.md",
        "README.md",
        "doc/README.md",
        "doc/ECU_Project_Implementation_v1.4.md",
        "doc/ECU/CANopen_RPDO_Operation_Manual_Node1-13_V2.md",
        "doc/ECU/遥控操作逻辑说明书.md",
        "docs/ecu-main-control-architecture.md",
        "docs/ecu-configuration-open-items.md",
    ]
    current = "\n".join(read(root, path) for path in current_paths)

    assert "20 motor rev/10 mm" in current or "20 rev/10 mm" in current
    assert "262144 count/mm" in current
    assert "6 mm/s" in current
    assert "12 rev/10 mm" not in current
    assert "normal speed is 5 mm/s" not in current.lower()
    assert "doc/README.md" in read(root, "AGENTS.md")
    assert "tools/README.md" in read(root, "doc/README.md")


def test_can3_lift_requires_stable_final_feedback_before_brake_disable(
    root: pathlib.Path,
) -> None:
    config_h = read(root, "ecu/config/include/ecu_config.h")
    lift_h = read(root, "ecu/devices/include/lift_hydraulic_device.h")
    lift_c = read(root, "ecu/devices/src/lift_hydraulic_device.c")

    assert "#define ECU_CANOPEN_LIFT_TARGET_STABLE_SAMPLES" in config_h
    assert "lift_target_stable_samples" in lift_h
    target_block = lift_c.split(
        "if (lift_positions_at_target(state, command_target_position_counts))", 1
    )[1].split("if (lift_progress_stalled", 1)[0]
    assert "state->lift_target_stable_samples++" in target_block
    assert "ECU_CANOPEN_LIFT_TARGET_STABLE_SAMPLES" in target_block
    assert "SERVO_DRIVE_CONTROL_DISABLE_VOLTAGE" in target_block
    assert "state->lift_target_stable_samples = 0U" in lift_c


def test_default_dio_and_hydraulic_masks_do_not_overlap(root: pathlib.Path) -> None:
    config_h = read(root, "ecu/config/include/ecu_config.h")

    def mask_value(name: str) -> int:
        zero_match = re.search(rf"#define\s+{name}\s+\(0UL\)", config_h)
        if zero_match:
            return 0
        match = re.search(rf"#define\s+{name}\s+\(1UL\s*<<\s*(\d+)\)", config_h)
        if match:
            return 1 << int(match.group(1))
        alias = re.search(rf"#define\s+{name}\s+(ECU_HYD_VALVE\d_MASK)", config_h)
        assert alias, f"missing mask macro {name}"
        return mask_value(alias.group(1))

    assert "#define ECU_DIO_BRAKE_RELEASE_MASK       (0UL)" in config_h

    dio_names = [
        "ECU_DIO_HYDRAULIC_ENABLE_MASK",
        "ECU_DIO_HEADLIGHT_MASK",
        "ECU_DIO_HORN_MASK",
        "ECU_DIO_HIGH_VOLTAGE_RELAY_MASK",
        "ECU_DIO_RESERVED3_MASK",
        "ECU_DIO_LEFT_INDICATOR_MASK",
        "ECU_DIO_RIGHT_INDICATOR_MASK",
    ]
    hydraulic_names = [
        "ECU_HYD_VALVE1_MASK",
        "ECU_HYD_VALVE2_MASK",
        "ECU_HYD_VALVE3_MASK",
        "ECU_HYD_VALVE4_MASK",
        "ECU_HYD_VALVE5_MASK",
        "ECU_HYD_VALVE6_MASK",
        "ECU_HYD_VALVE_TRACK_EXTEND_MASK",
        "ECU_HYD_VALVE_TRACK_RETRACT_MASK",
    ]

    dio_mask = 0
    for name in dio_names:
        dio_mask |= mask_value(name)
    hydraulic_mask = 0
    for name in hydraulic_names:
        hydraulic_mask |= mask_value(name)

    assert (dio_mask & hydraulic_mask) == 0


def test_relay_box_1_5_gpio_high_drives_low_active_relay_inputs(root: pathlib.Path) -> None:
    config_h = read(root, "ecu/config/include/ecu_config.h")
    config_c = read(root, "ecu/config/src/ecu_config.c")
    local_io_h = read(root, "ecu/devices/include/local_io_device.h")
    local_io_c = read(root, "ecu/devices/src/local_io_device.c")

    expected = {
        "ECU_HYD_VALVE1_MASK": 0,
        "ECU_DIO_HEADLIGHT_MASK": 1,
        "ECU_HYD_VALVE2_MASK": 2,
        "ECU_DIO_HORN_MASK": 3,
        "ECU_HYD_VALVE3_MASK": 4,
        "ECU_DIO_HIGH_VOLTAGE_RELAY_MASK": 5,
        "ECU_HYD_VALVE4_MASK": 6,
        "ECU_DIO_RESERVED3_MASK": 7,
        "ECU_HYD_VALVE5_MASK": 8,
        "ECU_DIO_LEFT_INDICATOR_MASK": 9,
        "ECU_HYD_VALVE6_MASK": 10,
        "ECU_DIO_RIGHT_INDICATOR_MASK": 11,
    }
    for name, bit in expected.items():
        assert f"#define {name}" in config_h
        assert re.search(rf"#define\s+{name}\s+\(1UL\s*<<\s*{bit}\)", config_h), name

    assert "#define ECU_DIO_HIGH_VOLTAGE_RELAY_MASK  (1UL << 5)" in config_h
    assert "#define ECU_DIO_HYDRAULIC_ENABLE_MASK    (0UL)" in config_h
    assert ".dio_high_voltage_relay_mask = ECU_DIO_HIGH_VOLTAGE_RELAY_MASK" in config_c
    assert "GPIO high turns the MOS output on" in config_h
    assert "relay control" in config_h and "terminal low" in config_h
    assert ".dio_active_high = true" in config_c
    assert "MOS6 / EX_OUT6 to the battery-key" in local_io_h
    assert "MOS8 / EX_OUT8 is reserved" in local_io_h
    assert "high_voltage_relay_latched" in local_io_h
    assert "config->dio_high_voltage_relay_mask" in local_io_c
    assert "command->high_voltage_enable" in local_io_c
    assert "command->high_voltage_disable_request" in local_io_c
    assert "state->high_voltage_relay_latched = true" in local_io_c
    assert "state->high_voltage_relay_latched = false" in local_io_c
    assert "state->high_voltage_relay_latched" in local_io_c


def test_hydraulic_valve_pairs_are_fail_closed_interlocked(root: pathlib.Path) -> None:
    config_h = read(root, "ecu/config/include/ecu_config.h")
    config_c = read(root, "ecu/config/src/ecu_config.c")
    lift_h = read(root, "ecu/devices/include/lift_hydraulic_device.h")
    lift_c = read(root, "ecu/devices/src/lift_hydraulic_device.c")
    vehicle_types_h = read(root, "ecu/vehicle/include/vehicle_types.h")
    executor_c = read(root, "ecu/vehicle/src/vehicle_command_executor.c")

    for token in [
        "ECU_HYD_VALVE_PAIR12_MASK",
        "ECU_HYD_VALVE_PAIR34_MASK",
        "ECU_HYD_VALVE_PAIR56_MASK",
        ".hydraulic_valve_interlock_pair12_mask = ECU_HYD_VALVE_PAIR12_MASK",
        ".hydraulic_valve_interlock_pair34_mask = ECU_HYD_VALVE_PAIR34_MASK",
        ".hydraulic_valve_interlock_pair56_mask = ECU_HYD_VALVE_PAIR56_MASK",
    ]:
        assert token in config_h or token in config_c, token

    for token in [
        "valve_interlock_reject_count",
        "last_requested_valve_mask",
        "last_interlocked_valve_mask",
    ]:
        assert token in lift_h, token

    for token in [
        "hydraulic_requested_valve_mask",
        "hydraulic_applied_valve_mask",
        "hydraulic_interlocked_valve_mask",
        "hydraulic_valve_interlock_reject_count",
        "update_executor_lift_hydraulic_diagnostics",
    ]:
        assert token in vehicle_types_h or token in executor_c, token

    for token in [
        "queue_lift_interpolation_group",
        "sanitize_hydraulic_valve_mask",
        "clear_conflicting_valve_pair",
        "valve_mask &= ~pair_mask",
        "state->valve_interlock_reject_count++",
        "state->last_interlocked_valve_mask = interlocked_valve_mask",
    ]:
        assert token in lift_c, token


def test_home_center_hydraulic_valve_mapping_uses_confirmed_functions(root: pathlib.Path) -> None:
    config_h = read(root, "ecu/config/include/ecu_config.h")
    arbiter_c = read(root, "ecu/vehicle/src/command_arbiter.c")
    lift_c = read(root, "ecu/devices/src/lift_hydraulic_device.c")

    for token in [
        "#define ECU_HYD_VALVE_TRACK_EXTEND_MASK  ECU_HYD_VALVE4_MASK",
        "#define ECU_HYD_VALVE_TRACK_RETRACT_MASK ECU_HYD_VALVE3_MASK",
        "#define ECU_HYD_VALVE_FRONT_SUSPENSION_RETRACT_MASK ECU_HYD_VALVE1_MASK",
        "#define ECU_HYD_VALVE_FRONT_SUSPENSION_EXTEND_MASK  ECU_HYD_VALVE2_MASK",
        "#define ECU_HYD_VALVE_REAR_SUSPENSION_RETRACT_MASK  ECU_HYD_VALVE5_MASK",
        "#define ECU_HYD_VALVE_REAR_SUSPENSION_EXTEND_MASK   ECU_HYD_VALVE6_MASK",
    ]:
        assert token in config_h, token

    for token in [
        "track_width_valve_mask_from_remote",
        "suspension_valve_mask_from_remote",
        "REMOTE_HYDRAULIC_SUSPENSION_FRONT",
        "remote->adjust_owner != REMOTE_ADJUST_OWNER_HYDRAULIC",
        "remote->requested_gear == ECU_GEAR_REQUEST_P",
        "P is a hard hold command",
        "ECU_HYD_VALVE_FRONT_SUSPENSION_RETRACT_MASK",
        "ECU_HYD_VALVE_REAR_SUSPENSION_RETRACT_MASK",
        "out->hydraulic_enable = hydraulic_valve_mask != 0U",
        "Ground-clearance lift is electric CAN3 servo motion",
    ]:
        assert token in arbiter_c, token

    assert "const bool pump_request =" in lift_c
    assert "command->hydraulic_enable &&" in lift_c
    assert "apply_hydraulic_pump_and_valves(state" in lift_c
    executor_c = read(root, "ecu/vehicle/src/vehicle_command_executor.c")
    for token in [
        "apply_can3_safe_default",
        "vehicle_actuator_command_safe_default(&safe_command)",
        "ECU_CAN3_COMMAND_STALE_TIMEOUT_MS",
        "command_snapshot_timestamp_is_fresh",
    ]:
        assert token in executor_c or token in config_h, token
    assert "ECU_HYDRAULIC_PUMP_MAX_REVERSE_RPM           (2400.0f)" in config_h
    assert "ECU_HYDRAULIC_PUMP_WORK_RPM                  (1500.0f)" in config_h
    assert "ECU_HYDRAULIC_PUMP_VALVE_OPEN_MIN_RPM        (800.0f)" in config_h
    assert (
        "state->pump_actual_velocity_units <\n"
        "            -ECU_HYDRAULIC_PUMP_VALVE_OPEN_MIN_VELOCITY_UNITS"
    ) in lift_c


def test_whole_vehicle_servo_enable_waits_for_bms_high_voltage_feedback(root: pathlib.Path) -> None:
    vehicle_types_h = read(root, "ecu/vehicle/include/vehicle_types.h")
    tasks_c = read(root, "ecu/os/src/ecu_tasks_cpu0.c")
    motion_h = read(root, "ecu/devices/include/motion_device.h")
    motion_c = read(root, "ecu/devices/src/motion_device.c")

    assert "bool high_voltage_feedback_ready;" in vehicle_types_h
    assert "bms_high_voltage_feedback_ready" in tasks_c
    assert "positive_contactor_closed" in tasks_c
    assert "negative_contactor_closed" in tasks_c
    assert "s_runtime.final_command.high_voltage_feedback_ready" in tasks_c

    for token in [
        "can2_motion_operational_nmt_sent_mask",
        "request_can2_motion_nodes_operational",
        "command->high_voltage_feedback_ready",
        "drive_enable_requested",
        "CANOPEN_MASTER_DEBUG_COMMAND_NMT_OPERATIONAL",
    ]:
        assert token in motion_h or token in motion_c, token


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
    assert "high_voltage_relay_latched" in monitor_h
    assert "hv_latch=%s" in monitor_c
    for token in [
        "hydraulic_requested_valve_mask",
        "hydraulic_applied_valve_mask",
        "hydraulic_interlocked_valve_mask",
        "hydraulic_valve_interlock_reject_count",
        "valve_block_cnt=%lu",
    ]:
        assert token in monitor_h or token in monitor_c or token in tasks_c, token
    assert "printf(" in monitor_c
    assert "runtime_monitor_print_cpu0" in tasks_c
    assert "ECU_ENABLE_DEBUG_MONITOR" in tasks_c
    assert "runtime_monitor.c" in cmake


def test_cpu0_segger_project_uses_jlink_debug_connection(root: pathlib.Path) -> None:
    cmake = read(root, "ecu/apps/agri_chassis_control_cpu0/CMakeLists.txt")

    assert 'sdk_ses_opt_debug_connection("J-Link")' in cmake
    assert "sdk_ses_opt_debug_jlink_speed(4000)" in cmake


def test_cpu1_segger_project_uses_jlink_debug_connection(root: pathlib.Path) -> None:
    """CPU1 should regenerate a J-Link SES project without OpenOCD manual setup."""

    cmake = read(root, "ecu/apps/agri_chassis_control_cpu1/CMakeLists.txt")

    for token in [
        "HPM_SDK_BASE",
        "GNURISCV_TOOLCHAIN_PATH",
        'sdk_ses_opt_debug_connection("J-Link")',
        "sdk_ses_opt_debug_jlink_speed(4000)",
    ]:
        assert token in cmake, token


def test_sbus_uart1_idle_interrupt_is_bound_to_service(root: pathlib.Path) -> None:
    sbus_hw_h = read(root, "ecu/drivers/sbus/include/sbus_uart_hw.h")
    sbus_hw_c = read(root, "ecu/drivers/sbus/src/sbus_uart_hw.c")
    sbus_service_h = read(root, "ecu/drivers/sbus/include/sbus_service.h")
    sbus_service_c = read(root, "ecu/drivers/sbus/src/sbus_service.c")
    config_h = read(root, "ecu/config/include/ecu_config.h")
    tasks_c = read(root, "ecu/os/src/ecu_tasks_cpu0.c")
    monitor_h = read(root, "ecu/diag/include/runtime_monitor.h")
    monitor_c = read(root, "ecu/diag/src/runtime_monitor.c")
    cmake = read(root, "ecu/apps/agri_chassis_control_cpu0/CMakeLists.txt")

    for token in [
        "sbus_uart_hw_init",
        "BOARD_SBUS_UART_BASE",
        "BOARD_SBUS_BAUDRATE",
        "BOARD_SBUS_UART_IRQ",
        "uart_intr_rx_line_idle",
        "uart_intr_rx_data_avail_or_timeout",
        "SDK_DECLARE_EXT_ISR_M",
        "xTaskGetTickCountFromISR",
        "sbus_service_feed_byte_from_isr",
        "sbus_service_note_rx_idle_from_isr",
        "parity_even",
        "stop_bits_2",
    ]:
        assert token in sbus_hw_h or token in sbus_hw_c, token

    assert "sbus_service_note_rx_idle_from_isr" in sbus_service_h
    assert "frame_position = 0U" in sbus_service_c
    assert "ECU_SBUS_UART_RX_IDLE_BITS" in config_h
    assert "ECU_SBUS_UART_IRQ_PRIORITY" in config_h
    assert "sbus_uart_hw_init(&s_runtime.sbus)" in tasks_c
    assert "sbus_uart_hw.c" in cmake
    assert "sbus_channels[ECU_SBUS_CHANNEL_COUNT]" in monitor_h
    assert "ECU SBUS" in monitor_c


def test_can2_can3_motion_and_lift_buses_are_tx_capable(root: pathlib.Path) -> None:
    can_hw_h = read(root, "ecu/drivers/can/include/can_bus_hw.h")
    can_hw_c = read(root, "ecu/drivers/can/src/can_bus_hw.c")
    can_service_h = read(root, "ecu/drivers/can/include/can_bus_service.h")
    can_service_c = read(root, "ecu/drivers/can/src/can_bus_service.c")
    config_h = read(root, "ecu/config/include/ecu_config.h")
    tasks_c = read(root, "ecu/os/src/ecu_tasks_cpu0.c")
    monitor_h = read(root, "ecu/diag/include/runtime_monitor.h")
    monitor_c = read(root, "ecu/diag/src/runtime_monitor.c")
    cmake = read(root, "ecu/apps/agri_chassis_control_cpu0/CMakeLists.txt")

    assert re.search(r"#define\s+ECU_CAN1_POWER_BITRATE\s+\(250000UL\)", config_h)
    assert re.search(r"#define\s+ECU_CAN2_MOTION_BITRATE\s+\(1000000UL\)", config_h)
    assert "ECU_CAN3_LIFT_HYDRAULIC_BITRATE" in config_h
    assert "ECU_CAN4_AUXILIARY_BITRATE" in config_h
    assert "ECU_ENABLE_CAN4_PHYSICAL_TEST_TX" in config_h
    assert "#define ECU_ENABLE_CAN4_PHYSICAL_TEST_TX (0)" in config_h
    assert "ECU_CAN4_PHYSICAL_TEST_FRAME_ID" in config_h
    assert "can_bus_hw_init_can2_rx_only" in can_hw_h
    assert "can_bus_hw_init_can2_motion" in can_hw_h
    assert "can_bus_hw_init_can3_lift_hydraulic" in can_hw_h
    assert "can_bus_hw_init_can4_auxiliary" in can_hw_h
    assert "can_bus_hw_poll_can2_rx" in can_hw_h
    assert "can_bus_hw_poll_can3_rx" in can_hw_h
    assert "can_bus_hw_poll_can4_rx" in can_hw_h
    assert "can_bus_hw_send_can2_frame" in can_hw_h
    assert "can_bus_hw_send_can3_frame" in can_hw_h
    assert "can_bus_hw_send_can4_frame" in can_hw_h
    assert "BOARD_CAN2_BASE" in can_hw_c
    assert "BOARD_CAN2_IRQn" in can_hw_c
    assert "BOARD_CAN3_BASE" in can_hw_c
    assert "BOARD_CAN3_IRQn" in can_hw_c
    assert "BOARD_CAN4_BASE" in can_hw_c
    assert "BOARD_CAN4_IRQn" in can_hw_c
    assert "can_get_default_config" in can_hw_c
    assert "can_config.baudrate = bitrate" in can_hw_c
    assert "CAN_EVENT_RECEIVE" in can_hw_c
    assert "can_read_received_message" in can_hw_c
    assert "can_is_data_available_in_receive_buffer" in can_hw_c
    assert "can_get_receive_buffer_status" in can_hw_c
    assert "can_get_receive_error_count" in can_hw_c
    assert "can_get_transmit_error_count" in can_hw_c
    assert "can_get_last_error_kind" in can_hw_c
    assert "SDK_DECLARE_EXT_ISR_M" in can_hw_c
    assert "can_bus_hw_init_can2_rx_only" in can_hw_c
    assert "can_bus_service_set_tx_backend(service, 0)" in can_hw_c
    assert "can_bus_service_set_tx_backend(service, can_bus_hw_send_can2_frame)" in can_hw_c
    assert "can_bus_service_set_tx_backend(service, can_bus_hw_send_can3_frame)" in can_hw_c
    assert "can_bus_service_set_tx_backend(service, can_bus_hw_send_can4_frame)" in can_hw_c
    assert "can_bus_service_note_rx_from_isr" in can_service_h
    assert "can_bus_service_note_error_from_isr" in can_service_h
    assert "can2_motion_canopen" in tasks_c
    assert "can3_lift_hydraulic_canopen" in tasks_c
    assert "commissioning_debug" in tasks_c
    assert "commissioning_debug_process_can4_physical_test" in tasks_c
    assert "can_bus_hw_init_can4_auxiliary" in read(root, "ecu/diag/src/commissioning_debug.c")
    assert "CANOPEN_MASTER_BUS_CAN2" in tasks_c
    assert "CANOPEN_MASTER_BUS_CAN3" in tasks_c
    assert "ECU_ENABLE_CAN3_LIFT_CANOPEN" in config_h
    assert "can3_rpdo=%s" in monitor_c
    assert "#if ECU_ENABLE_CAN3_LIFT_CANOPEN" not in tasks_c
    assert "canopen_master_service_process_realtime_pdo(&s_runtime.can2_motion_canopen" in tasks_c
    assert "canopen_master_service_process_background(&s_runtime.can2_motion_canopen" in tasks_c
    assert "canopen_master_service_process_realtime_pdo(&s_runtime.can3_lift_hydraulic_canopen" in tasks_c
    assert "canopen_master_service_process_background(&s_runtime.can3_lift_hydraulic_canopen" in tasks_c
    assert "can2_rx_count" in monitor_h
    assert "can2_rx_buffer_status" in monitor_h
    assert "can2_receive_error_count" in monitor_h
    assert "can4_test_tx_count" in monitor_h
    assert "can4_test_error_count" in monitor_h
    assert "ECU CAN2" in monitor_c
    assert "ECU CAN4 TEST" in monitor_c
    assert "can_bus_hw.c" in cmake


def test_ecu_can_termination_defaults_enabled_active_high(root: pathlib.Path) -> None:
    config_h = read(root, "ecu/config/include/ecu_config.h")
    board_h = read(root, "ecu/ecu_isolation/board.h")
    board_c = read(root, "ecu/ecu_isolation/board.c")
    pinmux_c = read(root, "ecu/ecu_isolation/pinmux.c")

    for token in [
        "ECU_CAN1_TERMINATION_ENABLE      (1)",
        "ECU_CAN2_TERMINATION_ENABLE      (1)",
        "ECU_CAN3_TERMINATION_ENABLE      (1)",
        "ECU_CAN4_TERMINATION_ENABLE      (1)",
    ]:
        assert token in config_h, token

    assert "BOARD_CAN_TERM_ENABLE_LEVEL  (1U)" in board_h
    assert "BOARD_CAN_TERM_DISABLE_LEVEL (0U)" in board_h
    for token in [
        "board_set_can_termination(1, true)",
        "board_set_can_termination(2, true)",
        "board_set_can_termination(3, true)",
        "board_set_can_termination(4, true)",
    ]:
        assert token in board_c, token
    assert "BOARD_CAN_TERM_ENABLE_LEVEL" in pinmux_c


def test_can_and_sbus_isr_snapshots_are_copied_atomically(root: pathlib.Path) -> None:
    sbus_service_c = read(root, "ecu/drivers/sbus/src/sbus_service.c")
    can_service_h = read(root, "ecu/drivers/can/include/can_bus_service.h")
    can_service_c = read(root, "ecu/drivers/can/src/can_bus_service.c")
    tasks_c = read(root, "ecu/os/src/ecu_tasks_cpu0.c")

    assert "taskENTER_CRITICAL" in sbus_service_c
    assert "taskEXIT_CRITICAL" in sbus_service_c
    assert "can_bus_service_get_snapshot" in can_service_h
    assert "taskENTER_CRITICAL" in can_service_c
    assert "taskEXIT_CRITICAL" in can_service_c
    assert "can_bus_service_get_snapshot(&s_runtime.can1_power" in tasks_c


def test_rs4852_warning_light_protocol_is_hardware_bound(root: pathlib.Path) -> None:
    config_h = read(root, "ecu/config/include/ecu_config.h")
    config_c = read(root, "ecu/config/src/ecu_config.c")
    rs485_h = read(root, "ecu/drivers/uart/include/uart_rs485_hw.h")
    rs485_c = read(root, "ecu/drivers/uart/src/uart_rs485_hw.c")
    modbus_c = read(root, "ecu/drivers/uart/src/modbus_master_service.c")
    warning_h = read(root, "ecu/devices/include/warning_light_device.h")
    warning_c = read(root, "ecu/devices/src/warning_light_device.c")
    tasks_c = read(root, "ecu/os/src/ecu_tasks_cpu0.c")

    for token in [
        "ECU_MODBUS_WARNING_LIGHT_BAUDRATE",
        "ECU_MODBUS_WARNING_LIGHT_REQUEST_PERIOD_MS",
        "ECU_MODBUS_WARNING_LIGHT_RESPONSE_TIMEOUT_MS",
        "ECU_WARNING_LIGHT_VALUE_OFF",
        "ECU_WARNING_LIGHT_VALUE_YELLOW_SLOW_FLASH",
        "ECU_WARNING_LIGHT_VALUE_RED_STEADY_BUZZER",
    ]:
        assert token in config_h or token in config_c, token

    for token in [
        "uart_rs485_hw_send",
        "uart_rs485_hw_read",
        "uart_rs485_hw_clear_rx",
        "uart_rs485_2_hw_init",
        "uart_rs485_2_hw_isr",
        "BOARD_RS485_2_UART_BASE",
        "BOARD_RS485_2_UART_IRQ",
        "BOARD_RS485_2_DE_GPIO_CTRL",
    ]:
        assert token in rs485_h or token in rs485_c, token

    assert "uart_rs485_hw_send(uart" in modbus_c
    assert "uart_rs485_hw_read(uart" in modbus_c
    assert "uart_rs485_hw_clear_rx(uart" in modbus_c
    assert "modbus_master_service_t *master" in warning_h
    assert "modbus_master_service_process" in warning_c
    assert "agile_modbus_serialize_write_register" in warning_c
    assert "config->modbus_warning_light_register" in warning_c
    assert "rs485_2_hw" in tasks_c
    assert "warning_light_modbus_master" in tasks_c
    assert "uart_rs485_2_hw_init(&s_runtime.rs485_2_hw" in tasks_c


def test_executor_uses_cpu0_owned_hardware_services(root: pathlib.Path) -> None:
    executor_h = read(root, "ecu/vehicle/include/vehicle_command_executor.h")
    executor_c = read(root, "ecu/vehicle/src/vehicle_command_executor.c")
    tasks_c = read(root, "ecu/os/src/ecu_tasks_cpu0.c")

    for token in [
        "vehicle_executor_io_t",
        "canopen_master_service_t *can2_motion_canopen",
        "canopen_master_service_t *can3_lift_hydraulic_canopen",
        "dio_service_t *dio",
        "uart_rs485_hw_t *warning_light_uart",
        "modbus_master_service_t *warning_light_modbus",
    ]:
        assert token in executor_h, token

    for forbidden in [
        "can_bus_service_init(&s_runtime.can2_motion",
        "can_bus_service_init(&s_runtime.can3_lift_hydraulic",
        "dio_service_init(&s_runtime.dio",
        "uart_comm_service_init(&s_runtime.rs485",
    ]:
        assert forbidden not in executor_c, forbidden

    assert "vehicle_executor_io_t executor_io" in tasks_c
    assert ".can2_motion_canopen = &s_runtime.can2_motion_canopen" in tasks_c
    assert ".can3_lift_hydraulic_canopen = &s_runtime.can3_lift_hydraulic_canopen" in tasks_c
    assert ".dio = &s_runtime.dio" in tasks_c
    assert ".warning_light_uart = &s_runtime.rs485_2_hw" in tasks_c
    assert "vehicle_command_executor_apply(&s_runtime.executor, &executor_io" in tasks_c


def test_dio_outputs_drive_board_gpio(root: pathlib.Path) -> None:
    dio_h = read(root, "ecu/drivers/dio/include/dio_service.h")
    dio_c = read(root, "ecu/drivers/dio/src/dio_service.c")
    dio_hw_h = read(root, "ecu/drivers/dio/include/dio_hw.h")
    dio_hw_c = read(root, "ecu/drivers/dio/src/dio_hw.c")
    board_c = read(root, "ecu/ecu_isolation/board.c")
    cmake = read(root, "ecu/apps/agri_chassis_control_cpu0/CMakeLists.txt")
    tasks_c = read(root, "ecu/os/src/ecu_tasks_cpu0.c")

    assert "dio_service_apply_backend_t" in dio_h
    assert "dio_service_set_apply_backend" in dio_h
    assert "service->apply_backend" in dio_c
    assert "dio_hw_attach_outputs" in dio_hw_h
    assert "board_ecu_output_write" in dio_hw_c
    assert "BOARD_ECU_OUTPUT_COUNT" in dio_hw_c
    assert "board_ecu_output_write" in board_c
    assert "dio_hw.c" in cmake
    assert "dio_hw_attach_outputs(&s_runtime.dio)" in tasks_c


def test_remote_command_generation_uses_sbus_analog_channels(root: pathlib.Path) -> None:
    remote_h = read(root, "ecu/remote/include/remote_types.h")
    manager_c = read(root, "ecu/remote/src/remote_manager.c")
    mapper_c = read(root, "ecu/remote/src/remote_sbus_mapper.c")
    arbiter_c = read(root, "ecu/vehicle/src/command_arbiter.c")
    config_h = read(root, "ecu/config/include/ecu_config.h")

    for token in [
        "steer_per_mille",
        "throttle_per_mille",
        "clearance_per_mille",
        "track_per_mille",
        "r1_changed",
        "r2_changed",
    ]:
        assert token in remote_h, token

    assert "manager->request.steer_per_mille = input->steer_per_mille" in manager_c
    assert "manager->request.throttle_per_mille = input->throttle_per_mille" in manager_c
    assert "sbus_per_mille_from_ppm" in mapper_c
    assert "decode_error_limit =" in mapper_c
    assert "credibility_error =" in mapper_c
    assert "safety_discrete_positions_valid" in mapper_c
    assert "motion_control_build_candidate" in arbiter_c
    assert "ECU_REMOTE_MAX_SPEED_MPS" in config_h
    assert "ECU_REMOTE_MAX_STEER_DEG" in config_h
    assert "ECU_REMOTE_MAX_HEIGHT_RATE_MM_S" in config_h
    assert "ECU_REMOTE_MAX_TRACK_RATE_MM_S" in config_h


def test_cpu0_runtime_initialization_is_explicit_and_keeps_hardware_init_out_of_critical(root: pathlib.Path) -> None:
    main_c = read(root, "ecu/apps/agri_chassis_control_cpu0/src/main_cpu0.c")
    tasks_h = read(root, "ecu/os/include/ecu_tasks.h")
    tasks_c = read(root, "ecu/os/src/ecu_tasks_cpu0.c")

    assert "ecu_task_runtime_init(0U)" in main_c
    assert "void ecu_task_runtime_init(uint32_t now_ms);" in tasks_h
    assert "void ecu_task_runtime_init(uint32_t now_ms)" in tasks_c
    assert "taskENTER_CRITICAL" not in tasks_c


def test_can1_foreground_poll_has_a_frame_budget(root: pathlib.Path) -> None:
    can_hw_c = read(root, "ecu/drivers/can/src/can_bus_hw.c")

    assert "CAN_BUS_HW_MAX_RX_FRAMES_PER_POLL" in can_hw_c
    assert "frames_drained" in can_hw_c
    assert "frames_drained < CAN_BUS_HW_MAX_RX_FRAMES_PER_POLL" in can_hw_c


def test_cpu0_remote_and_safety_handoffs_are_coherent_and_stale_safe(root: pathlib.Path) -> None:
    tasks_c = read(root, "ecu/os/src/ecu_tasks_cpu0.c")
    mailbox_h = read(root, "ecu/os/include/ecu_control_snapshot_mailbox.h")
    mailbox_c = read(root, "ecu/os/src/ecu_control_snapshot_mailbox.c")
    cmake = read(root, "ecu/apps/agri_chassis_control_cpu0/CMakeLists.txt")
    config_h = read(root, "ecu/config/include/ecu_config.h")

    for token in [
        "remote_request_mailbox_t",
        "safety_snapshot_mailbox_t",
        "remote_request_mailbox_publish",
        "remote_request_mailbox_read",
        "safety_snapshot_mailbox_publish",
        "safety_snapshot_mailbox_read",
        "__ATOMIC_RELEASE",
        "__ATOMIC_ACQUIRE",
        "ECU_REMOTE_REQUEST_STALE_TIMEOUT_MS",
        "ECU_SAFETY_SNAPSHOT_STALE_TIMEOUT_MS",
    ]:
        assert token in tasks_c or token in mailbox_h or token in mailbox_c or token in config_h, token

    assert "remote_control_request_t value[2]" in mailbox_h
    assert "vehicle_safety_snapshot_t value[2]" in mailbox_h
    assert "uint32_t slot = sequence & 1U" in mailbox_c
    assert "ecu_control_snapshot_mailbox.c" in cmake
    assert "remote_request.estop_state = REMOTE_ESTOP_LATCHED" in tasks_c
    assert "safety_snapshot.controlled_stop_active = true" in tasks_c


def test_cpu0_remote_and_safety_mailbox_read_is_atomic_against_task_preemption(
    root: pathlib.Path,
) -> None:
    """A safety-task publication race must not become a false A-class fault."""

    mailbox_c = read(root, "ecu/os/src/ecu_control_snapshot_mailbox.c")

    reader = mailbox_c.split("bool prefix##_read", 1)[1].split(
        "DEFINE_DOUBLE_BUFFER_MAILBOX", 1
    )[0]
    assert '#include "FreeRTOS.h"' in mailbox_c
    assert '#include "task.h"' in mailbox_c
    assert "taskENTER_CRITICAL();" in reader
    assert "taskEXIT_CRITICAL();" in reader
    assert reader.index("taskENTER_CRITICAL();") < reader.index("*value = mailbox->value[slot]")
    assert reader.index("*timestamp_ms = mailbox->timestamp_ms[slot]") < reader.rindex("taskEXIT_CRITICAL();")
    assert "ECU_CONTROL_SNAPSHOT_MAILBOX_READ_ATTEMPTS" not in mailbox_c
    assert "for (uint32_t attempt" not in reader
    assert "return true" in reader
    assert "return false" in reader


def test_cpu0_control_snapshot_freshness_handles_newer_publisher_timestamp(
    root: pathlib.Path,
) -> None:
    """A preempting publisher may timestamp a snapshot after the step's now_ms."""

    tasks_c = read(root, "ecu/os/src/ecu_tasks_cpu0.c")

    assert "control_snapshot_timestamp_is_fresh" in tasks_c
    helper = tasks_c.split("static bool control_snapshot_timestamp_is_fresh", 1)[1].split(
        "\n}", 1
    )[0]
    assert "int32_t signed_age_ms" in helper
    assert "future_skew_ms" in helper

    executor_c = read(root, "ecu/vehicle/src/vehicle_command_executor.c")
    assert "command_snapshot_timestamp_is_fresh" in executor_c
    command_helper = executor_c.split(
        "static bool command_snapshot_timestamp_is_fresh", 1
    )[1].split("\n}", 1)[0]
    assert "int32_t signed_age_ms" in command_helper
    assert "ECU_CONTROL_SNAPSHOT_MAX_FUTURE_SKEW_MS" in command_helper
    assert executor_c.count("command_snapshot_timestamp_is_fresh(") >= 3
    assert "ECU_CONTROL_SNAPSHOT_MAX_FUTURE_SKEW_MS" in tasks_c
    assert tasks_c.count("control_snapshot_timestamp_is_fresh(") >= 5
    assert "(uint32_t)(now_ms - remote_timestamp_ms)" not in tasks_c
    assert "(uint32_t)(now_ms - safety_timestamp_ms)" not in tasks_c


def test_cpu0_preconditions_are_feedback_driven(root: pathlib.Path) -> None:
    tasks_c = read(root, "ecu/os/src/ecu_tasks_cpu0.c")
    config_h = read(root, "ecu/config/include/ecu_config.h")
    monitor_h = read(root, "ecu/diag/include/runtime_monitor.h")
    monitor_c = read(root, "ecu/diag/src/runtime_monitor.c")

    assert "ecu_hardware_feedback_snapshot_t" in config_h
    assert "hardware_feedback" in tasks_c
    assert "out->power_ready = true;" not in tasks_c
    assert "out->low_voltage_ok = true;" not in tasks_c
    assert "out->can1_power_online = true;" not in tasks_c
    assert "out->power_ready = s_runtime.hardware_feedback.power_ready;" in tasks_c
    assert "out->low_voltage_ok = s_runtime.hardware_feedback.low_voltage_ok;" in tasks_c
    assert "out->can1_power_online = s_runtime.hardware_feedback.can1_power_online;" in tasks_c
    assert "hardware_feedback" in monitor_h
    assert "[ECU HW]" in monitor_c


def test_unknown_power_protocol_is_safe_by_default(root: pathlib.Path) -> None:
    config_h = read(root, "ecu/config/include/ecu_config.h")
    power_h = read(root, "ecu/devices/include/power_device.h")
    power_c = read(root, "ecu/devices/src/power_device.c")

    assert "ECU_POWER_PROTOCOL_DISABLED" in config_h
    assert "power_protocol" in config_h
    assert "ECU_DEVICE_APPLY_UNCONFIGURED" in read(root, "ecu/common/include/ecu_types.h")
    assert "power_device_apply" in power_h
    assert "send_power_node" not in power_c
    assert "ECU_DEVICE_APPLY_UNCONFIGURED" in power_c
    assert "high_voltage_enable" in power_c


def test_no_transitional_language_in_active_engineering_files(root: pathlib.Path) -> None:
    forbidden_patterns = [
        "tempo" + "rary",
        "place" + "holder",
        "integration" + " point",
        "st" + "ub",
        "not " + "implemented",
    ]
    scanned_roots = [root / "ecu", root / "tests", root / "docs"]
    ignored = set()
    for folder in scanned_roots:
        for path in folder.rglob("*"):
            if not path.is_file():
                continue
            rel = path.relative_to(root).as_posix()
            if rel.startswith("ecu/sdk_env_v1.11.0/") or rel in ignored:
                continue
            if rel.startswith("ecu/apps/agri_chassis_control_cpu0/build/"):
                continue
            if path.suffix.lower() not in {".c", ".h", ".py", ".md", ".cmake", ".txt"}:
                continue
            text = path.read_text(encoding="utf-8", errors="ignore").lower()
            for pattern in forbidden_patterns:
                assert pattern not in text, f"{rel}: remove transitional wording '{pattern}'"


def test_cpu0_startup_and_fatal_hooks_are_visible_on_debug_console(root: pathlib.Path) -> None:
    main_c = read(root, "ecu/apps/agri_chassis_control_cpu0/src/main_cpu0.c")

    assert "ECU CPU0 boot" in main_c
    assert "create_task_or_report" in main_c
    assert "FATAL malloc failed" in main_c
    assert "FATAL stack overflow" in main_c
    assert "\\r\\n" in main_c


def test_cpu0_periodic_tasks_yield_after_overrun(root: pathlib.Path) -> None:
    main_c = read(root, "ecu/apps/agri_chassis_control_cpu0/src/main_cpu0.c")

    runner = main_c.split("static void run_periodic_task", 1)[1]
    runner = runner.split("typedef struct", 1)[0]

    assert "TickType_t before_step" in runner
    assert "TickType_t after_step" in runner
    assert "if ((after_step - before_step) >= period_ticks)" in runner
    assert "vTaskDelay(1U)" in runner
    assert runner.index("step(") < runner.index("vTaskDelayUntil")


def test_cpu0_realtime_can_tasks_preempt_diagnostics(root: pathlib.Path) -> None:
    tasks_c = read(root, "ecu/os/src/ecu_tasks_cpu0.c")

    # The descriptor table is indexed directly by ecu_cpu0_task_id_t.  The
    # entries must therefore stay in enum order; sorting this list by priority
    # silently assigns the wrong priority/name to each created FreeRTOS task.
    expected = [
        '{ "safety", ECU_CPU0_SAFETY_PERIOD_MS, 31U, 1024U }',
        '{ "can2_motion", ECU_CPU0_CAN2_MOTION_PERIOD_MS, 30U, 1536U }',
        '{ "remote", ECU_CPU0_REMOTE_PERIOD_MS, 28U, 1536U }',
        '{ "vehicle", ECU_CPU0_CONTROL_PERIOD_MS, 27U, 1536U }',
        '{ "can1_power", ECU_CPU0_POWER_PERIOD_MS, 22U, 1024U }',
        '{ "can3_lift", ECU_CPU0_LIFT_HYD_PERIOD_MS, 24U, 1536U }',
        '{ "io", ECU_CPU0_IO_PERIOD_MS, 16U, 1024U }',
        '{ "diag", ECU_CPU0_DIAG_PERIOD_MS, 8U, 2048U }',
    ]

    table_start = tasks_c.index("static const ecu_task_descriptor_t s_cpu0_tasks")
    table_end = tasks_c.index("};", table_start)
    table = tasks_c[table_start:table_end]

    last_position = -1
    for item in expected:
        position = table.find(item)
        assert position >= 0, item
        assert position > last_position, item
        last_position = position

    priorities = {
        name: int(priority)
        for name, priority in re.findall(r'\{ "([^"]+)", ECU_CPU0_[^,]+, ([0-9]+)U, [0-9]+U \}', table)
    }
    assert priorities["can2_motion"] > priorities["diag"]
    assert priorities["can3_lift"] > priorities["diag"]
