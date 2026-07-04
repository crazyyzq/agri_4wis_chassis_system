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
