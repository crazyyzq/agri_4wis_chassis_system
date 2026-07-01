"""Vehicle arbitration, safety and executor boundary contract tests."""

from __future__ import annotations

import pathlib


def read(root: pathlib.Path, rel: str) -> str:
    path = root / rel
    assert path.exists(), f"missing {rel}"
    return path.read_text(encoding="utf-8")


def test_diagnostic_codes_are_stable(root: pathlib.Path) -> None:
    diag = read(root, "ecu/diag/include/diag_codes.h")
    required = [
        "DIAG_OK",
        "DIAG_REMOTE_ESTOP_CH13",
        "DIAG_REMOTE_ESTOP_SBUS_TIMEOUT",
        "DIAG_REMOTE_ESTOP_FAILSAFE",
        "DIAG_REJECT_NOT_PARKED",
        "DIAG_REJECT_THROTTLE_NOT_LOW",
        "DIAG_REJECT_MODE_PRECONDITION",
        "DIAG_REJECT_POWER_PRECONDITION",
        "DIAG_REJECT_AUTHORITY_PRECONDITION",
        "DIAG_REJECT_ADJUST_OWNER_CONFLICT",
        "DIAG_EVENT_EXPIRED",
        "DIAG_TRACK_ADJUST_ABORTED",
        "DIAG_CPU1_IPC_TIMEOUT",
    ]
    for token in required:
        assert token in diag, token


def test_vehicle_final_command_pipeline_exists(root: pathlib.Path) -> None:
    arbiter_h = read(root, "ecu/vehicle/include/command_arbiter.h")
    safety_h = read(root, "ecu/vehicle/include/safety_manager.h")
    executor_h = read(root, "ecu/vehicle/include/vehicle_command_executor.h")
    assert "command_arbiter_update" in arbiter_h
    assert "vehicle_actuator_command_safe_default" in arbiter_h
    assert "safety_manager_apply" in safety_h
    assert "vehicle_command_executor_apply" in executor_h


def test_vehicle_sources_rebuild_complete_command(root: pathlib.Path) -> None:
    text = read(root, "ecu/vehicle/src/command_arbiter.c")
    required = [
        "vehicle_actuator_command_safe_default",
        "COMMAND_SOURCE_REMOTE",
        "COMMAND_SOURCE_AUTO",
        "COMMAND_SOURCE_SAFETY",
        "priority",
        "complete_rebuild_each_cycle",
        "auto_control_allowed",
        "high_voltage_enable_request",
    ]
    for token in required:
        assert token in text, token


def test_auto_motion_command_sets_consistent_gear_and_brake_release(root: pathlib.Path) -> None:
    text = read(root, "ecu/vehicle/src/command_arbiter.c")

    auto_block = text.split("if (remote != 0 && remote->auto_control_allowed", 1)[1]
    for token in [
        "auto_gear_from_speed",
        "auto_requests_brake_release",
        "out->active_gear = auto_gear_from_speed(auto_request->target_speed_kph)",
        "out->brake_release = auto_requests_brake_release(auto_request)",
    ]:
        assert token in text if token.startswith("auto_") else token in auto_block, token


def test_reverse_ackermann_uses_rear_as_driving_forward(root: pathlib.Path) -> None:
    """Reverse Ackermann keeps stick logic in the rear-facing driving frame."""

    arbiter_c = read(root, "ecu/vehicle/src/command_arbiter.c")
    kinematics_c = read(root, "ecu/control/src/four_wheel_kinematics.c")

    for token in [
        "motion_mode_reverses_driving_direction",
        "mode == ECU_MOTION_MODE_REVERSE_ACKERMANN",
        "apply_driving_direction_to_speed",
        "return motion_mode_reverses_driving_direction(mode) ? -speed_kph : speed_kph",
        "remote_speed_command_kph(const remote_control_request_t *remote, ecu_motion_mode_t mode)",
        "apply_driving_direction_to_speed(mode, speed)",
        "auto_speed_command_kph",
        "auto_request->requested_mode",
    ]:
        assert token in arbiter_c, token

    remote_call = arbiter_c.split("motion_control_build_candidate(remote->active_motion_mode", 1)[1]
    assert "remote_speed_command_kph(remote, remote->active_motion_mode)" in remote_call

    auto_block = arbiter_c.split("if (remote != 0 && remote->auto_control_allowed", 1)[1]
    assert "out->active_gear = auto_gear_from_speed(auto_request->target_speed_kph)" in auto_block
    assert "auto_speed_command_kph(auto_request)" in auto_block

    reverse_kinematics = kinematics_c[
        kinematics_c.index("void four_wheel_kinematics_build_reverse_ackermann"):
        kinematics_c.index("void four_wheel_kinematics_build_spin")
    ]
    assert "build_ackermann_from_curvature(speed_kph, steer_input_deg, -1.0f" in reverse_kinematics


def test_remote_arming_gear_requests_release_brake_before_active_drive(root: pathlib.Path) -> None:
    text = read(root, "ecu/vehicle/src/command_arbiter.c")
    required = [
        "remote_requests_brake_release",
        "GEAR_STATE_ARM_D",
        "GEAR_STATE_ARM_R",
        "remote->gear_state",
        "out->brake_release = remote_requests_brake_release(remote)",
    ]
    for token in required:
        assert token in text, token


def test_track_adjust_prepare_requests_brake_release_before_active_adjust(root: pathlib.Path) -> None:
    text = read(root, "ecu/vehicle/src/command_arbiter.c")
    adjust = read(root, "ecu/remote/src/remote_adjust_fsm.c")

    assert "ADJUST_STATE_TRACK_PREPARE" in adjust
    assert "preconditions->brake_release_confirmed" in adjust
    for token in [
        "ADJUST_STATE_TRACK_PREPARE",
        "ADJUST_STATE_TRACK_ACTIVE",
        "remote->adjust_state",
    ]:
        assert token in text, token


def test_track_adjust_configuration_is_parameterized(root: pathlib.Path) -> None:
    text = read(root, "ecu/config/include/ecu_config.h")
    required = [
        "track_adjust_config_t",
        "steer_target_deg",
        "assist_torque_sign",
        "assist_torque_limit_nm",
        "assist_wheel_speed_limit_rpm",
        "ECU_WHEEL_COUNT",
    ]
    for token in required:
        assert token in text, token


def test_motion_control_generates_mode_specific_four_wheel_targets(root: pathlib.Path) -> None:
    """Four steering modes must not collapse into the same four-wheel command."""

    vehicle_h = read(root, "ecu/vehicle/include/vehicle_types.h")
    motion_c = read(root, "ecu/control/src/motion_control.c")
    kin_c = read(root, "ecu/control/src/four_wheel_kinematics.c")
    motion_device_c = read(root, "ecu/devices/src/motion_device.c")
    command_c = read(root, "ecu/vehicle/src/command_arbiter.c")

    assert "float target_wheel_speed_kph[ECU_WHEEL_COUNT]" in vehicle_h
    assert "command->target_wheel_speed_kph[wheel]" in motion_device_c
    assert "motion_control_build_candidate" in command_c
    assert "out->target_wheel_speed_kph[wheel]" in motion_c

    for token in [
        "build_positive_ackermann_targets",
        "build_reverse_ackermann_targets",
        "build_spin_targets",
        "build_crab_targets",
    ]:
        assert token in motion_c, token

    for token in [
        "ECU_WHEEL_LEG1_FRONT_RIGHT",
        "ECU_WHEEL_LEG2_FRONT_LEFT",
        "ECU_WHEEL_LEG3_REAR_LEFT",
        "ECU_WHEEL_LEG4_REAR_RIGHT",
    ]:
        assert token in kin_c, token

    spin_block = motion_c[
        motion_c.index("static void build_spin_targets"):
        motion_c.index("static void build_crab_targets")
    ]
    assert "four_wheel_kinematics_build_spin" in spin_block
    spin_impl = kin_c[
        kin_c.index("void four_wheel_kinematics_build_spin"):
        kin_c.index("void four_wheel_kinematics_build_crab")
    ]
    for token in [
        "out->target_steer_deg[ECU_WHEEL_LEG1_FRONT_RIGHT] = spin_angle",
        "out->target_steer_deg[ECU_WHEEL_LEG2_FRONT_LEFT] = -spin_angle",
        "out->target_steer_deg[ECU_WHEEL_LEG3_REAR_LEFT] = spin_angle",
        "out->target_steer_deg[ECU_WHEEL_LEG4_REAR_RIGHT] = -spin_angle",
        "out->target_wheel_speed_kph[ECU_WHEEL_LEG2_FRONT_LEFT] = -speed_kph",
        "out->target_wheel_speed_kph[ECU_WHEEL_LEG3_REAR_LEFT] = -speed_kph",
    ]:
        assert token in spin_impl, token


def test_four_wheel_ackermann_kinematics_uses_vehicle_geometry(root: pathlib.Path) -> None:
    """Ackermann output must be computed from wheelbase, track width and ICR geometry."""

    config_h = read(root, "ecu/config/include/ecu_config.h")
    motion_h = read(root, "ecu/control/include/motion_control.h")
    motion_c = read(root, "ecu/control/src/motion_control.c")
    kin_h = read(root, "ecu/control/include/four_wheel_kinematics.h")
    kin_c = read(root, "ecu/control/src/four_wheel_kinematics.c")
    arbiter_c = read(root, "ecu/vehicle/src/command_arbiter.c")
    cmake = read(root, "ecu/apps/agri_chassis_control_cpu0/CMakeLists.txt")

    for token in [
        "ECU_VEHICLE_WHEELBASE_MM",
        "ECU_VEHICLE_TRACK_WIDTH_MIN_MM",
        "ECU_VEHICLE_TRACK_WIDTH_DEFAULT_MM",
        "ECU_VEHICLE_TRACK_WIDTH_MAX_MM",
        "ECU_VEHICLE_MIN_TURN_RADIUS_MM",
    ]:
        assert token in config_h, token

    for token in [
        "float wheelbase_mm",
        "float track_width_mm",
    ]:
        assert token in motion_h, token
        assert token in kin_h, token

    assert '#include "four_wheel_kinematics.h"' in motion_c
    assert "four_wheel_kinematics_build_ackermann" in motion_c
    assert "four_wheel_kinematics_build_reverse_ackermann" in motion_c
    assert "four_wheel_kinematics_build_spin" in motion_c
    assert "four_wheel_kinematics_build_crab" in motion_c

    for token in [
        "four_wheel_kinematics_geometry_t",
        "four_wheel_kinematics_output_t",
        "atan2f",
        "sqrtf",
        "tanf",
        "clamp_geometry",
        "turn_radius_mm",
        "wheel_x_mm",
        "wheel_y_mm",
        "linear_velocity_x",
        "linear_velocity_y",
        "ECU_WHEEL_LEG1_FRONT_RIGHT",
        "ECU_WHEEL_LEG2_FRONT_LEFT",
        "ECU_WHEEL_LEG3_REAR_LEFT",
        "ECU_WHEEL_LEG4_REAR_RIGHT",
    ]:
        assert token in kin_c, token

    assert ".wheelbase_mm = ECU_VEHICLE_WHEELBASE_MM" in arbiter_c
    assert ".track_width_mm = ECU_VEHICLE_TRACK_WIDTH_DEFAULT_MM" in arbiter_c
    assert "four_wheel_kinematics.c" in cmake
    assert 'sdk_ld_options("-lm")' in cmake


def test_safety_manager_clamps_dangerous_outputs(root: pathlib.Path) -> None:
    text = read(root, "ecu/vehicle/src/safety_manager.c")
    required = [
        "brake_release_allowed",
        "hydraulic_enable = false",
        "high_voltage_enable = false",
        "target_speed_kph = 0.0f",
        "estop_latched",
        "a_class_fault",
    ]
    for token in required:
        assert token in text, token


def test_cpu1_cannot_include_executor_or_safety_critical_headers(root: pathlib.Path) -> None:
    text = read(root, "ecu/apps/agri_chassis_control_cpu1/src/main_cpu1.c")
    forbidden = [
        "vehicle_command_executor",
        "command_arbiter",
        "safety_manager",
        "can2_motion",
        "brake_release",
        "hydraulic_valve",
        "high_voltage",
    ]
    for token in forbidden:
        assert token not in text, token


def test_config_contains_required_timing_values(root: pathlib.Path) -> None:
    text = read(root, "ecu/config/include/ecu_config.h")
    expected = {
        "REMOTE_DISCRETE_DEBOUNCE_MS": "80",
        "REMOTE_LINK_QUALIFY_MS": "1000",
        "REMOTE_NEUTRAL_QUALIFY_MS": "300",
        "REMOTE_FAILSAFE_TIMEOUT_MS": "100",
        "REMOTE_DOMAIN_EVENT_GUARD_MS": "150",
    }
    for name, value in expected.items():
        assert name in text, name
        assert value in text, f"{name} must include {value}"


def test_static_guard_exists(root: pathlib.Path) -> None:
    text = read(root, "tools/check_no_forbidden_patterns.py")
    assert "remote" in text
    assert "vTaskDelay(" in text
    assert "1050" in text and "1950" in text
