"""V8 remote four-wheel steering commissioning source-level contracts.

These tests intentionally verify only firmware structure and safety contracts.
They do not claim CAN analyzer, drive acceptance, or motor motion evidence.
"""

from __future__ import annotations

import pathlib
import re


def read(root: pathlib.Path, rel: str) -> str:
    path = root / rel
    assert path.exists(), f"missing {rel}"
    return path.read_text(encoding="utf-8")


def test_v8_policy_and_authorization_are_fail_closed_by_default(root: pathlib.Path) -> None:
    config_h = read(root, "ecu/config/include/ecu_config.h")
    motion_c = read(root, "ecu/devices/src/motion_device.c")

    assert "ECU_CANOPEN_COMMISSIONING_POLICY_STEER4_REMOTE_COMMISSIONING" in config_h
    assert "ECU_STEER_REMOTE_COMMISSION_AUTH_MAGIC" in config_h
    assert "ecu_steer_commissioning_control_t" in config_h
    assert "steer_remote_commission_enable" in config_h
    assert "enabled_axis_mask" in config_h
    assert "expiry_ms" in config_h

    default_policy = re.search(
        r"#ifndef ECU_CANOPEN_COMMISSIONING_POLICY\s*"
        r"#define ECU_CANOPEN_COMMISSIONING_POLICY\s*\\\s*"
        r"\n\s*ECU_CANOPEN_COMMISSIONING_POLICY_MAPPING_VERIFY_ONLY",
        config_h,
    )
    assert default_policy is not None

    assert "volatile ecu_steer_commissioning_control_t g_ecu_steer_commissioning_control" in motion_c
    assert "g_ecu_steer_commissioning_control.magic == ECU_STEER_REMOTE_COMMISSION_AUTH_MAGIC" in motion_c
    assert "g_ecu_steer_commissioning_control.steer_remote_commission_enable" in motion_c
    assert "g_ecu_steer_commissioning_control.enabled_axis_mask" in motion_c
    assert "g_ecu_steer_commissioning_control.expiry_ms" in motion_c
    assert "clear_steer_commissioning_authorization" in motion_c


def test_v8_axis_calibration_defaults_invalid_and_targets_are_built_by_one_function(root: pathlib.Path) -> None:
    config_h = read(root, "ecu/config/include/ecu_config.h")
    config_c = read(root, "ecu/config/src/ecu_config.c")
    motion_h = read(root, "ecu/devices/include/motion_device.h")
    motion_c = read(root, "ecu/devices/src/motion_device.c")

    for token in [
        "steer_axis_calibration_t",
        "direction_sign",
        "straight_zero_offset_counts",
        "minimum_position_counts",
        "maximum_position_counts",
        "commissioning_max_abs_deg",
        "steer_axis_calibration[ECU_WHEEL_COUNT]",
    ]:
        assert token in config_h, token

    assert config_c.count(".valid = false") >= 4
    assert "steer_commissioning_build_targets" in motion_h
    target_fn = motion_c.split("bool steer_commissioning_build_targets", 1)[1].split(
        "static", 1
    )[0]
    for token in [
        "ECU_STEER_REMOTE_COMMISSION_MAX_DEG",
        "direction_sign",
        "straight_zero_offset_counts",
        "minimum_position_counts",
        "maximum_position_counts",
        "return false",
    ]:
        assert token in target_fn, token


def test_v8_canopen_service_has_tpdo_observer_sync_and_explicit_group_descriptor(root: pathlib.Path) -> None:
    service_h = read(root, "ecu/drivers/canopen/include/canopen_master_service.h")
    service_c = read(root, "ecu/drivers/canopen/src/canopen_master_service.c")

    for token in [
        "canopen_node_feedback_t",
        "tpdo0_valid",
        "tpdo1_valid",
        "feedback_fresh",
        "actual_position_counts",
        "actual_velocity_units",
        "fault_latched",
        "statusword",
        "actual_current_raw",
        "canopen_master_pdo_group_descriptor_t",
        "arm_frame_count",
        "trigger_frame_count",
        "axis_mask",
        "position_group",
        "canopen_master_service_queue_pdo_batch_with_descriptor",
        "canopen_master_service_send_sync",
        "canopen_master_service_get_node_feedback",
    ]:
        assert token in service_h, token

    for token in [
        "CO_CANrxBufferInit",
        "steer_tpdo_rx_callback",
        "CO_CANrxMsg_readIdent",
        "CO_CANrxMsg_readDLC",
        "CANOPEN_MASTER_SYNC_COB_ID",
        "service->active_pdo_group_descriptor.arm_frame_count",
    ]:
        assert token in service_c, token

    assert "required_arms =\n            service->active_pdo_expected_frames == (ECU_WHEEL_COUNT * 2U)" not in service_c


def test_v8_remote_commissioning_uses_selected_axis_0f_1f_and_never_drive_or_can3(root: pathlib.Path) -> None:
    motion_c = read(root, "ecu/devices/src/motion_device.c")
    command_c = read(root, "ecu/vehicle/src/command_arbiter.c")

    assert "commissioning_policy_allows_steer4_remote" in motion_c
    assert "STEER_REMOTE_COMMISSION_ACTIVE" in motion_c
    assert "STEER_REMOTE_COMMISSION_TPDO_MONITOR" in motion_c
    assert "STEER_REMOTE_COMMISSION_AXIS_READY" in motion_c
    assert "selected_axis_mask" in motion_c
    assert "CANOPEN_MASTER_PDO_PHASE_STEER_ARM" in motion_c
    assert "CANOPEN_MASTER_PDO_PHASE_STEER_TRIGGER" in motion_c
    assert "SERVO_DRIVE_CONTROL_ENABLE_OPERATION" in motion_c
    assert "SERVO_DRIVE_CONTROL_TRIGGER_ABSOLUTE_POSITION" in motion_c

    remote_commission_block = motion_c.split("static bool queue_steer4_remote_group", 1)[1].split(
        "static bool queue_steer_group", 1
    )[0]
    assert "SERVO_DRIVE_CONTROL_ABSOLUTE_UPDATE_ARM" not in remote_commission_block
    assert "SERVO_DRIVE_CONTROL_ABSOLUTE_UPDATE_TRIGGER" not in remote_commission_block
    assert "commissioning_policy_allows_drive_rpdo" in motion_c
    assert "return false;" in motion_c.split("commissioning_policy_allows_drive_rpdo", 1)[1].split("}", 1)[0]
    assert "commissioning_policy_allows_can3_rpdo" in motion_c
    assert "return false;" in motion_c.split("commissioning_policy_allows_can3_rpdo", 1)[1].split("}", 1)[0]

    assert "ECU_CANOPEN_COMMISSIONING_POLICY_STEER4_REMOTE_COMMISSIONING" in command_c
    assert "out->active_gear = ECU_GEAR_REQUEST_P" in command_c
    assert "out->target_speed_kph = 0.0f" in command_c
    assert "out->hydraulic_enable = false" in command_c
    assert "remote->throttle_per_mille == 0" in command_c


def test_v9_commit_a_steer_commissioning_uses_explicit_interlock_not_brake_release(root: pathlib.Path) -> None:
    vehicle_h = read(root, "ecu/vehicle/include/vehicle_types.h")
    command_c = read(root, "ecu/vehicle/src/command_arbiter.c")
    motion_c = read(root, "ecu/devices/src/motion_device.c")

    for token in [
        "steer_commission_interlock_ok",
        "steer_commission_steering_neutral",
    ]:
        assert token in vehicle_h, token
        assert token in command_c, token
        assert token in motion_c, token

    interlock_fn = motion_c.split("static bool steer_commissioning_remote_conditions_ok", 1)[1].split(
        "static motion_steer_inhibit_reason_t evaluate_steer_inhibit_reason", 1
    )[0]
    assert "command->steer_commission_interlock_ok" in interlock_fn
    assert "command->brake_release" not in interlock_fn

    steer4_arbiter_block = command_c.split(
        "ECU_CANOPEN_COMMISSIONING_POLICY_STEER4_REMOTE_COMMISSIONING", 1
    )[1].split("#else", 1)[0]
    assert "out->brake_release = false" in steer4_arbiter_block
    assert "out->steer_commission_interlock_ok" in steer4_arbiter_block
    assert "out->steer_commission_steering_neutral" in steer4_arbiter_block


def test_v9_commit_a_active_state_allows_nonzero_steering_after_neutral_entry(root: pathlib.Path) -> None:
    motion_c = read(root, "ecu/devices/src/motion_device.c")

    update_fn = motion_c.split("static void update_steer_remote_commissioning_state", 1)[1].split(
        "static void send_commissioning_sync_if_due", 1
    )[0]
    assert "state->steer_commission_state != STEER_REMOTE_COMMISSION_ACTIVE" in update_fn
    neutral_gate = update_fn.split("state->steer_commission_state != STEER_REMOTE_COMMISSION_ACTIVE", 1)[1].split(
        "request_selected_steer_nodes_operational", 1
    )[0]
    assert "steer_commission_steering_neutral" in neutral_gate
    assert "steering_command_is_neutral" in neutral_gate


def test_v9_commit_a_authorization_clear_resets_session_state(root: pathlib.Path) -> None:
    motion_c = read(root, "ecu/devices/src/motion_device.c")

    clear_fn = motion_c.split("static void clear_steer_commissioning_authorization", 1)[1].split(
        "static bool commissioning_policy_allows_node5_steer_pdo", 1
    )[0]
    for token in [
        "steer_commission_nmt_sent_mask = 0U",
        "steer_commission_neutral_since_ms = 0U",
        "steer_commission_last_sync_ms = 0U",
        "selected_axis_mask = 0U",
        "steer_next_group_valid = false",
        "steer_commission_state = STEER_REMOTE_COMMISSION_WAIT_AUTH",
    ]:
        assert token in clear_fn, token


def test_v9_commit_b_tpdo_observer_registration_is_checked_and_gates_output(root: pathlib.Path) -> None:
    service_h = read(root, "ecu/drivers/canopen/include/canopen_master_service.h")
    service_c = read(root, "ecu/drivers/canopen/src/canopen_master_service.c")
    motion_c = read(root, "ecu/devices/src/motion_device.c")

    for token in [
        "tpdo0_observer_registered_mask",
        "tpdo1_observer_registered_mask",
        "steer_tpdo_observer_ready",
        "tpdo_observer_registration_error_count",
        "canopen_master_service_steer_tpdo_observers_ready",
    ]:
        assert token in service_h, token
        assert token in service_c or token.startswith("canopen_master_service_"), token

    observer_fn = service_c.split("static void register_steer_tpdo_observers", 1)[1].split(
        "static void refresh_tpdo_freshness", 1
    )[0]
    assert "CO_ReturnError_t" in observer_fn
    assert "CO_CANrxBufferInit" in observer_fn
    assert "== CO_ERROR_NO" in observer_fn
    assert "tpdo0_observer_registered_mask" in observer_fn
    assert "tpdo1_observer_registered_mask" in observer_fn
    assert "steer_tpdo_observer_ready" in observer_fn

    feedback_gate = motion_c.split("static bool steer_commissioning_axis_feedback_ready", 1)[1].split(
        "static bool steer_commissioning_axis_calibration_ready", 1
    )[0]
    assert "canopen_master_service_steer_tpdo_observers_ready" in feedback_gate


def test_v9_commit_b_feedback_reader_uses_even_sequence_lock(root: pathlib.Path) -> None:
    service_h = read(root, "ecu/drivers/canopen/include/canopen_master_service.h")
    service_c = read(root, "ecu/drivers/canopen/src/canopen_master_service.c")

    assert "feedback_sequence" in service_h

    callback_fn = service_c.split("static void steer_tpdo_rx_callback", 1)[1].split(
        "static uint16_t find_free_canopen_rx_slot", 1
    )[0]
    for token in [
        "feedback_sequence",
        "begin_feedback_write",
        "end_feedback_write",
    ]:
        assert token in callback_fn or token in service_c, token

    reader_fn = service_c.split("bool canopen_master_service_get_node_feedback", 1)[1].split(
        "void canopen_master_service_get_snapshot", 1
    )[0]
    for token in [
        "sequence_before",
        "sequence_after",
        "sequence_before == sequence_after",
        "(sequence_before & 1U) == 0U",
    ]:
        assert token in reader_fn, token


def test_v9_commit_b_sync_is_gated_and_has_inflight_completion(root: pathlib.Path) -> None:
    service_h = read(root, "ecu/drivers/canopen/include/canopen_master_service.h")
    service_c = read(root, "ecu/drivers/canopen/src/canopen_master_service.c")

    for token in [
        "sync_in_flight",
        "sync_in_flight_submit_ms",
        "sync_tx_complete_count",
        "last_sync_tx_complete_ms",
    ]:
        assert token in service_h, token

    send_sync_fn = service_c.split("bool canopen_master_service_send_sync", 1)[1].split(
        "bool canopen_master_service_get_node_feedback", 1
    )[0]
    for token in [
        "service->pdo_queue_count != 0U",
        "pdo_group_is_active(service)",
        "service->sync_in_flight",
        "service->sync_in_flight_submit_ms = now_ms",
    ]:
        assert token in send_sync_fn, token

    start_next_fn = service_c.split("static bool start_next_pdo_frame", 1)[1].split(
        "static void wait_briefly_for_pdo_tx_complete", 1
    )[0]
    assert "service->sync_in_flight" in start_next_fn

    complete_fn = service_c.split("static void process_pdo_tx_complete_events", 1)[1].split(
        "static bool pdo_in_flight_timed_out", 1
    )[0]
    assert "complete_in_flight_sync" in complete_fn
    assert "sync_tx_complete_count" in service_c


def test_v9_commit_b_position_group_waits_for_post_command_tpdo_window(root: pathlib.Path) -> None:
    motion_h = read(root, "ecu/devices/include/motion_device.h")
    motion_c = read(root, "ecu/devices/src/motion_device.c")
    config_h = read(root, "ecu/config/include/ecu_config.h")

    assert "ECU_STEER_REMOTE_COMMISSION_POST_COMMAND_TPDO_TIMEOUT_MS" in config_h
    assert "STEER_REMOTE_COMMISSION_WAIT_POST_COMMAND_TPDO" in motion_h
    for token in [
        "steer_commission_post_command_tpdo_pending",
        "steer_commission_post_command_axis_mask",
        "steer_commission_tpdo0_count_before",
        "steer_commission_tpdo1_count_before",
        "steer_commission_post_command_timeout_count",
    ]:
        assert token in motion_h, token

    flush_fn = motion_c.split("static ecu_device_apply_result_t flush_steer4_remote_commissioning", 1)[1].split(
        "#endif", 1
    )[0]
    for token in [
        "start_post_command_tpdo_window",
        "post_command_tpdo_window_complete",
        "STEER_REMOTE_COMMISSION_WAIT_POST_COMMAND_TPDO",
        "ECU_STEER_REMOTE_COMMISSION_POST_COMMAND_TPDO_TIMEOUT_MS",
    ]:
        assert token in flush_fn or token in motion_c, token

    post_fn = motion_c.split("static bool post_command_tpdo_window_complete", 1)[1].split(
        "static ecu_device_apply_result_t flush_steer4_remote_commissioning", 1
    )[0]
    assert "feedback.tpdo0_rx_count <= state->steer_commission_tpdo0_count_before[wheel]" in post_fn
    assert "feedback.tpdo1_rx_count <= state->steer_commission_tpdo1_count_before[wheel]" in post_fn
    assert "feedback.fault_latched != 0U" in post_fn


def test_v9_commit_c_cmake_profiles_are_safe_by_default_and_selectable(root: pathlib.Path) -> None:
    cmake = read(root, "ecu/apps/agri_chassis_control_cpu0/CMakeLists.txt")
    config_h = read(root, "ecu/config/include/ecu_config.h")
    monitor_c = read(root, "ecu/diag/src/runtime_monitor.c")

    for token in [
        'set(ECU_COMMISSIONING_PROFILE "safe" CACHE STRING "safe|steer4_remote")',
        "ECU_COMMISSIONING_PROFILE STREQUAL \"steer4_remote\"",
        "-DECU_CANOPEN_COMMISSIONING_POLICY=4",
        "-DECU_COMMISSIONING_STEER_ONLY_MODE=1",
        "-DECU_ENABLE_MAINTENANCE_SDO_WRITES=0",
        "-DECU_ENABLE_COMMISSIONING_POWER_DEBUG=0",
        "-DECU_ENABLE_COMMISSIONING_CANOPEN_SCAN=0",
        "Unsupported ECU_COMMISSIONING_PROFILE",
    ]:
        assert token in cmake, token

    assert "#ifndef ECU_COMMISSIONING_STEER_ONLY_MODE" in config_h
    assert "#ifndef ECU_BUILD_PROFILE_TEXT" in config_h
    assert "build_profile=%s policy=%s drive_rpdo=0 can3_rpdo=0" in monitor_c
    assert "brake_control=none" in monitor_c


def test_v9_commit_c_ram_calibration_override_is_fail_closed_and_not_flash_saved(root: pathlib.Path) -> None:
    config_h = read(root, "ecu/config/include/ecu_config.h")
    motion_h = read(root, "ecu/devices/include/motion_device.h")
    motion_c = read(root, "ecu/devices/src/motion_device.c")
    monitor_c = read(root, "ecu/diag/src/runtime_monitor.c")

    for token in [
        "ECU_STEER_CALIBRATION_OVERRIDE_MAGIC",
        "ecu_steer_calibration_override_t",
        "volatile ecu_steer_calibration_override_t g_ecu_steer_calibration_override",
        "motion_device_get_effective_steer_calibration",
    ]:
        assert token in config_h or token in motion_h or token in motion_c, token

    override_fn = motion_c.split("bool motion_device_get_effective_steer_calibration", 1)[1].split(
        "bool steer_commissioning_build_targets", 1
    )[0]
    for token in [
        "ECU_STEER_CALIBRATION_OVERRIDE_MAGIC",
        "g_ecu_steer_calibration_override.enable",
        "sequence_before != sequence_after",
        "steer_axis_calibration_is_valid",
        "!steer_axis_calibration_is_valid(&out_calibration[wheel])",
        "return false",
    ]:
        assert token in override_fn, token

    assert "ECU_CANOPEN_OBJ_STORE_PARAMETERS" not in override_fn
    assert "ram_override=%s valid=%s seq=%lu" in monitor_c
    assert "steer_effective_calibration" in read(root, "ecu/diag/include/runtime_monitor.h")


def test_v9_commit_c_steer4_output_uses_effective_calibration_not_static_config_only(root: pathlib.Path) -> None:
    motion_c = read(root, "ecu/devices/src/motion_device.c")

    ready_fn = motion_c.split("static bool steer_commissioning_axis_calibration_ready", 1)[1].split(
        "static bool steer_commissioning_remote_conditions_ok", 1
    )[0]
    assert "motion_device_get_effective_steer_calibration" in ready_fn
    assert "steer_commissioning_build_targets(calibration" in ready_fn

    flush_fn = motion_c.split("static ecu_device_apply_result_t flush_steer4_remote_commissioning", 1)[1].split(
        "#endif", 1
    )[0]
    assert "motion_device_get_effective_steer_calibration" in flush_fn
    assert "steer_commissioning_build_targets(calibration" in flush_fn
