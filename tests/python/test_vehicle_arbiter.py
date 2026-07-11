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
        "high_voltage_disable_request",
    ]
    for token in required:
        assert token in text, token


def test_auto_motion_command_sets_consistent_gear_and_brake_release(root: pathlib.Path) -> None:
    text = read(root, "ecu/vehicle/src/command_arbiter.c")

    auto_block = text.split("if (remote != 0 && remote->auto_control_allowed", 1)[1]
    for token in [
        "auto_gear_from_speed",
        "auto_requests_brake_release",
        "out->active_gear = auto_gear_from_speed(auto_request->target_speed_mps)",
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
        "return motion_mode_reverses_driving_direction(mode) ? -speed_mps : speed_mps",
        "remote_speed_command_mps(const remote_control_request_t *remote, ecu_motion_mode_t mode)",
        "apply_driving_direction_to_speed(mode, speed)",
        "auto_speed_command_mps",
        "auto_request->requested_mode",
    ]:
        assert token in arbiter_c, token

    remote_call = arbiter_c.split("motion_control_build_candidate(remote->active_motion_mode", 1)[1]
    assert "remote_speed_command_mps(remote, remote->active_motion_mode)" in remote_call

    auto_block = arbiter_c.split("if (remote != 0 && remote->auto_control_allowed", 1)[1]
    assert "out->active_gear = auto_gear_from_speed(auto_request->target_speed_mps)" in auto_block
    assert "auto_speed_command_mps(auto_request)" in auto_block

    reverse_kinematics = kinematics_c[
        kinematics_c.index("void four_wheel_kinematics_build_reverse_ackermann"):
        kinematics_c.index("void four_wheel_kinematics_build_spin")
    ]
    assert "build_ackermann_from_curvature(speed_mps, steer_input_deg, -1.0f" in reverse_kinematics


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


def test_home_center_hydraulic_adjust_keeps_drive_parked_and_uses_valve_intent(root: pathlib.Path) -> None:
    text = read(root, "ecu/vehicle/src/command_arbiter.c")
    adjust = read(root, "ecu/remote/src/remote_adjust_fsm.c")
    gear = read(root, "ecu/remote/src/remote_gear_fsm.c")

    for token in [
        "ADJUST_STATE_READY",
        "ADJUST_STATE_HYDRAULIC_ACTIVE",
        "suspension_request_active",
        "hydraulic_suspension_target",
    ]:
        assert token in adjust, token
    for token in [
        "preconditions->adjustment_active",
        "fsm->active_gear = ECU_GEAR_REQUEST_P",
        "HOME-center adjustment uses the physical three-position gear switch",
    ]:
        assert token in gear, token
    for token in [
        "track_width_valve_mask_from_remote",
        "suspension_valve_mask_from_remote",
        "remote->requested_gear == ECU_GEAR_REQUEST_D",
        "remote->requested_gear == ECU_GEAR_REQUEST_R",
        "out->hydraulic_enable = hydraulic_valve_mask != 0U",
        "Ground-clearance lift is electric CAN3 servo motion",
        "remote->adjust_state",
        "inhibit_can2_motion_in_adjust_domain",
    ]:
        assert token in text, token

    adjust_gate = text.split("if (remote_in_adjust_domain(remote))", 1)[1].split(
        "/* CH1 field convention", 1
    )[0]
    assert "inhibit_can2_motion_in_adjust_domain(out);" in adjust_gate
    assert "apply_remote_adjust_command(remote, out);" in adjust_gate
    assert "return;" in adjust_gate


def test_track_adjust_configuration_is_parameterized(root: pathlib.Path) -> None:
    text = read(root, "ecu/config/include/ecu_config.h")
    required = [
        "track_adjust_config_t",
        "steer_target_deg",
        "assist_torque_sign",
        "assist_current_10ma",
        "assist_wheel_speed_limit_rpm",
        "ECU_WHEEL_COUNT",
    ]
    for token in required:
        assert token in text, token


def test_track_width_assist_is_presteered_and_valve_gated(root: pathlib.Path) -> None:
    """Track-width hydraulics must not open or push the wheels before steering is ready."""

    types_h = read(root, "ecu/vehicle/include/vehicle_types.h")
    arbiter_c = read(root, "ecu/vehicle/src/command_arbiter.c")
    executor_c = read(root, "ecu/vehicle/src/vehicle_command_executor.c")
    motion_c = read(root, "ecu/devices/src/motion_device.c")
    remote_adjust_c = read(root, "ecu/remote/src/remote_adjust_fsm.c")
    config_h = read(root, "ecu/config/include/ecu_config.h")

    for token in [
        "ECU_REMOTE_TRACK_ASSIST_CENTER_EXIT_MS",
        "ECU_TRACK_ASSIST_LEG1_CURRENT_10MA",
        "ECU_TRACK_ASSIST_LEG2_CURRENT_10MA",
        "ECU_TRACK_ASSIST_LEG3_CURRENT_10MA",
        "ECU_TRACK_ASSIST_LEG4_CURRENT_10MA",
        "#define ECU_TRACK_ASSIST_LEG1_CURRENT_10MA           (700)",
        "#define ECU_TRACK_ASSIST_LEG2_CURRENT_10MA           (1000)",
        "#define ECU_TRACK_ASSIST_LEG3_CURRENT_10MA           (1000)",
        "#define ECU_TRACK_ASSIST_LEG4_CURRENT_10MA           (700)",
        "ECU_TRACK_ASSIST_STEER_APPROX_TOLERANCE_COUNTS",
    ]:
        assert token in config_h, token

    for token in [
        "track_assist_requested",
        "track_assist_active",
        "track_assist_current_10ma",
    ]:
        assert token in types_h, token

    for token in [
        "track_center_since_ms",
        "ADJUST_STATE_TRACK_PREPARE",
        "ADJUST_STATE_TRACK_ACTIVE",
        "ADJUST_STATE_TRACK_EXITING",
        "ECU_REMOTE_TRACK_ASSIST_CENTER_EXIT_MS",
    ]:
        assert token in remote_adjust_c, token

    assert "ecu_track_adjust_config_default" in arbiter_c
    assert "static void apply_track_adjust_steering_posture" in arbiter_c
    assert "static void apply_track_adjust_drive_assist" in arbiter_c
    adjust_fn = arbiter_c.split("static void apply_remote_adjust_command", 1)[1].split(
        "static void inhibit_can2_motion_in_adjust_domain", 1
    )[0]
    for token in [
        "apply_track_adjust_steering_posture",
        "apply_track_adjust_drive_assist",
        "hydraulic_valve_mask != 0U",
        "remote_adjust_state_keeps_track_posture",
        "remote_adjust_state_allows_clearance",
        "remote_adjust_state_allows_track",
        "remote_adjust_state_allows_suspension",
    ]:
        assert token in adjust_fn, token

    posture_fn = arbiter_c.split("static void apply_track_adjust_steering_posture", 1)[1].split(
        "static void apply_track_adjust_drive_assist", 1
    )[0]
    for token in [
        "track_assist_requested = true",
        "out->motion_mode = ECU_MOTION_MODE_CRAB",
        "out->target_steer_deg[wheel] = track_config->steer_target_deg[wheel]",
    ]:
        assert token in posture_fn, token
    assert "track_assist_current_10ma[wheel]" not in posture_fn

    helper_fn = arbiter_c.split("static void apply_track_adjust_drive_assist", 1)[1].split(
        "static void apply_remote_adjust_command", 1
    )[0]
    for token in [
        "track_assist_current_10ma[wheel]",
        "configured_current_10ma",
        "track_config->assist_current_10ma[wheel]",
    ]:
        assert token in helper_fn, token

    can3_flush = executor_c.split("vehicle_command_executor_flush_can3_lift_hydraulic", 1)[1]
    assert "gate_track_valves_until_presteer_ready" in executor_c
    assert "s_runtime.motion.track_assist_steer_approximately_ready" in executor_c
    assert "s_runtime.motion.presteer_target_reached" in executor_c
    assert "track_gate_request_timestamp_ms" in executor_c
    assert "track_assist_steer_ready_eval_ms" in executor_c
    assert "timestamp_reached(s_runtime.motion.track_assist_steer_ready_eval_ms" in executor_c
    assert "gate_track_valves_until_presteer_ready(&command, command_timestamp_ms)" in can3_flush

    can2_flush = executor_c.split("vehicle_command_executor_flush_can2_motion", 1)[1].split(
        "vehicle_command_executor_flush_can3_lift_hydraulic", 1
    )[0]
    assert "gate_track_assist_current_until_valve_open" in executor_c
    assert "s_runtime.lift_hydraulic.last_valve_mask" in executor_c
    assert "gate_track_assist_current_until_valve_open(&command)" in can2_flush

    for token in [
        "build_drive_current_rpdo_request",
        "canopen_pdo_build_current_rpdo3",
        "MOTION_DRIVE_COMMAND_CURRENT",
        "drive_latest_command_kind",
        "track_assist_steer_approximately_ready",
        "ECU_TRACK_ASSIST_STEER_APPROX_TOLERANCE_COUNTS",
    ]:
        assert token in motion_c, token


def test_motion_control_generates_mode_specific_four_wheel_targets(root: pathlib.Path) -> None:
    """Four steering modes must not collapse into the same four-wheel command."""

    vehicle_h = read(root, "ecu/vehicle/include/vehicle_types.h")
    motion_c = read(root, "ecu/control/src/motion_control.c")
    kin_c = read(root, "ecu/control/src/four_wheel_kinematics.c")
    motion_device_c = read(root, "ecu/devices/src/motion_device.c")
    command_c = read(root, "ecu/vehicle/src/command_arbiter.c")

    assert "float target_wheel_speed_mps[ECU_WHEEL_COUNT]" in vehicle_h
    assert "command->target_wheel_speed_mps[wheel]" in motion_device_c
    assert "motion_control_build_candidate" in command_c
    assert "out->target_wheel_speed_mps[wheel]" in motion_c

    for token in [
        "build_positive_ackermann_targets",
        "build_reverse_ackermann_targets",
        "build_spin_targets",
        "build_crab_targets",
    ]:
        assert token in motion_c, token
    crab_block = motion_c[
        motion_c.index("static void build_crab_targets"):
        motion_c.index("void motion_control_build_candidate")
    ]
    assert "ECU_MOTION_CRAB_STEER_DEG" in crab_block
    assert "four_wheel_kinematics_build_crab" in crab_block
    assert "speed_mps < 0.0f" not in crab_block
    assert "-ECU_MOTION_CRAB_STEER_DEG" not in crab_block
    assert "steer_deg" not in crab_block
    assert "CH1" not in crab_block

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
        "out->target_wheel_speed_mps[ECU_WHEEL_LEG2_FRONT_LEFT] = -speed_mps",
        "out->target_wheel_speed_mps[ECU_WHEEL_LEG3_REAR_LEFT] = -speed_mps",
    ]:
        assert token in spin_impl, token


def test_whole_vehicle_crab_keeps_full_90_degree_steering_range(root: pathlib.Path) -> None:
    """Crab mode needs four absolute +90 deg targets, so device clamping must allow 1225000 counts."""
    config_h = read(root, "ecu/config/include/ecu_config.h")

    max_range_block = config_h[
        config_h.index("#define ECU_CANOPEN_STEER_MAX_POSITION_COUNTS"):
        config_h.index("#define ECU_CANOPEN_STEER_SETUP_SETTLE_MS")
    ]
    assert "(1225000)" in max_range_block
    assert "(612500)" not in max_range_block


def test_fixed_posture_steering_transition_planner_is_isolated(root: pathlib.Path) -> None:
    """Spin/crab posture changes need a focused planner, not more trajectory code in motion_device.c."""
    planner_h = read(root, "ecu/control/include/steering_transition_planner.h")
    planner_c = read(root, "ecu/control/src/steering_transition_planner.c")
    motion_c = read(root, "ecu/devices/src/motion_device.c")
    cmake = read(root, "ecu/apps/agri_chassis_control_cpu0/CMakeLists.txt")

    for token in [
        "steering_transition_planner_t",
        "steering_transition_planner_init",
        "steering_transition_planner_mode_is_fixed_posture",
        "steering_transition_planner_update",
        "feedback_fresh_mask",
        "actual_position_counts",
        "requested_target_counts",
        "output_target_counts",
        "rejected_stale_feedback",
    ]:
        assert token in planner_h, token

    assert "smoothstep" in planner_c
    assert "axis_progress_q15" in planner_c
    assert "ECU_STEER_CRAB_FAST_AXIS_MASK" in planner_c
    assert "#define ECU_STEER_CRAB_FAST_AXIS_MASK" in read(root, "ecu/config/include/ecu_config.h")
    assert "transition_id" in planner_c
    assert "same_target" in planner_c
    assert "planner->completed && same_target" in planner_c
    assert "ECU_STEER_FIXED_TRANSITION_MAX_SPEED_COUNTS_PER_SEC" in planner_c
    assert "steering_transition_planner_update" in motion_c
    assert '#include "steering_transition_planner.h"' in motion_c
    assert "../../control/src/steering_transition_planner.c" in cmake


def test_fixed_posture_planner_uses_feedback_and_ackermann_bypasses_it(root: pathlib.Path) -> None:
    """Fixed posture transitions use TPDO actual position; Ackermann stays realtime-follow."""
    motion_c = read(root, "ecu/devices/src/motion_device.c")
    planner_c = read(root, "ecu/control/src/steering_transition_planner.c")

    assert "collect_steering_feedback_for_planner" in motion_c
    assert "steering_transition_planner_mode_is_fixed_posture(command->motion_mode)" in motion_c
    assert "MOTION_STEER_INHIBIT_AXIS_NOT_READY" in motion_c
    assert "ECU_MOTION_MODE_POSITIVE_ACKERMANN" not in planner_c
    assert "ECU_MOTION_MODE_REVERSE_ACKERMANN" not in planner_c


def test_ackermann_remote_steering_limit_is_50_degrees_not_crab_limit(root: pathlib.Path) -> None:
    """Ackermann follows the remote stick up to ±50 deg; crab keeps its independent fixed 90 deg posture."""
    config_h = read(root, "ecu/config/include/ecu_config.h")

    remote_limit_block = config_h[
        config_h.index("#if ECU_BUILD_PROFILE_STEER4_REMOTE_90"):
        config_h.index("/* Vehicle geometry used by four-wheel kinematics.")
    ]
    assert "#define ECU_REMOTE_MAX_STEER_DEG          (90.0f)" in remote_limit_block
    assert "#define ECU_REMOTE_MAX_STEER_DEG          (50.0f)" in remote_limit_block
    assert "#define ECU_MOTION_CRAB_STEER_DEG         (90.0f)" in config_h


def test_motion_units_are_mps_and_realtime_smoothing_is_discrete(root: pathlib.Path) -> None:
    config_h = read(root, "ecu/config/include/ecu_config.h")
    config_c = read(root, "ecu/config/src/ecu_config.c")
    motion_c = read(root, "ecu/devices/src/motion_device.c")
    debug_py = read(root, "tools/canopen_motion_debug/motion8_remote_sim_debug.py")

    for token in [
        "ECU_DRIVE_SPEED_MPS_TO_COUNTS_PER_SEC",
        "ECU_DRIVE_MAX_SPEED_MPS",
        "ECU_DRIVE_COMMISSIONING_MAX_SPEED_MPS",
        "ECU_REMOTE_MAX_SPEED_MPS",
        "ECU_CANOPEN_STEER_TARGET_RATE_LIMIT_NEAR_COUNTS_PER_SEC",
        "ECU_CANOPEN_STEER_TARGET_RATE_LIMIT_SMALL_COUNTS_PER_SEC",
        "ECU_CANOPEN_STEER_TARGET_RATE_LIMIT_MEDIUM_COUNTS_PER_SEC",
        "ECU_CANOPEN_STEER_TARGET_RATE_LIMIT_LARGE_COUNTS_PER_SEC",
        "ECU_CANOPEN_DRIVE_VELOCITY_RATE_LIMIT_SMALL_UNITS_PER_SEC",
        "ECU_CANOPEN_DRIVE_VELOCITY_RATE_LIMIT_MEDIUM_UNITS_PER_SEC",
        "ECU_CANOPEN_DRIVE_VELOCITY_RATE_LIMIT_LARGE_UNITS_PER_SEC",
        "ECU_CANOPEN_DRIVE_VELOCITY_RATE_LIMIT_REVERSAL_UNITS_PER_SEC",
        "ECU_CANOPEN_LEG1_DRIVE_DIRECTION_SIGN (1)",
        "ECU_CANOPEN_LEG2_DRIVE_DIRECTION_SIGN (-1)",
        "ECU_CANOPEN_LEG3_DRIVE_DIRECTION_SIGN (-1)",
        "ECU_CANOPEN_LEG4_DRIVE_DIRECTION_SIGN (1)",
    ]:
        assert token in config_h, token

    assert "drive_direction_sign[ECU_WHEEL_COUNT]" in config_h
    assert ".drive_direction_sign = {" in config_c
    assert "ECU_CANOPEN_LEG2_DRIVE_DIRECTION_SIGN" in config_c

    for token in [
        "select_steer_rate_limit_counts_per_sec",
        "select_drive_velocity_rate_limit_units_per_sec",
        "requested < 0 && current > 0",
        "requested > 0 && current < 0",
        "drive_direction_sign_is_valid",
        "config->drive_direction_sign[wheel]",
        "config->drive_speed_mps_to_counts_per_sec *",
    ]:
        assert token in motion_c, token

    assert "--speed-mps" in debug_py
    assert "DRIVE_DIRECTION_SIGNS = (1, -1, -1, 1)" in debug_py
    assert "DRIVE_SPEED_MPS_TO_UNITS * DRIVE_DIRECTION_SIGNS[index]" in debug_py
    assert "drive_sign_from_node" in debug_py
    assert "speed_kph" not in debug_py


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


def test_servo_steering_scale_matches_field_calibration(root: pathlib.Path) -> None:
    config_h = read(root, "ecu/config/include/ecu_config.h")
    motion_device_c = read(root, "ecu/devices/src/motion_device.c")

    assert "#define ECU_STEER_GEAR_REDUCTION                     (490.0f)" in config_h
    assert "#define ECU_STEER_DEG_TO_COUNTS" in config_h
    assert "(ECU_STEER_COUNTS_PER_OUTPUT_REV / 360.0f)" in config_h
    assert "send_steer_command(canopen" in motion_device_c
    assert "command->target_steer_deg[wheel]" in motion_device_c
    assert "config->steer_deg_to_counts" in motion_device_c


def test_spin_and_crab_hold_drive_until_steering_feedback_reaches_target(root: pathlib.Path) -> None:
    """Spin/crab must pre-steer from feedback before any drive velocity is enabled."""

    config_h = read(root, "ecu/config/include/ecu_config.h")
    motion_h = read(root, "ecu/devices/include/motion_device.h")
    motion_c = read(root, "ecu/devices/src/motion_device.c")
    executor_c = read(root, "ecu/vehicle/src/vehicle_command_executor.c")
    monitor_c = read(root, "ecu/diag/src/runtime_monitor.c")

    for token in [
        "ECU_CANOPEN_PRESTEER_POSITION_TOLERANCE_COUNTS",
        "ECU_CANOPEN_PRESTEER_TIMEOUT_MS",
        "ECU_CANOPEN_PRESTEER_REQUIRED_AXIS_MASK",
    ]:
        assert token in config_h, token

    for token in [
        "presteer_drive_hold_active",
        "presteer_target_reached",
        "presteer_missing_axis_mask",
        "presteer_timeout_count",
    ]:
        assert token in motion_h, token
        assert token in executor_c, token
        assert token in monitor_c, token

    assert "static bool motion_mode_requires_presteer" in motion_c
    presteer_fn = motion_c.split("static bool motion_mode_requires_presteer", 1)[1].split(
        "static bool steer_limit_blocks_target", 1
    )[0]
    for token in [
        "mode == ECU_MOTION_MODE_SPIN",
        "mode == ECU_MOTION_MODE_CRAB",
        "canopen_master_service_get_node_feedback",
        "feedback.feedback_fresh",
        "feedback.fault_latched == 0U",
        "abs_i32_delta(feedback.actual_position_counts",
        "ECU_CANOPEN_PRESTEER_POSITION_TOLERANCE_COUNTS",
        "return false;",
    ]:
        assert token in presteer_fn, token

    apply_fn = motion_c.split("ecu_device_apply_result_t motion_device_apply", 1)[1].split(
        "ecu_device_apply_result_t motion_device_flush_realtime", 1
    )[0]
    assert "presteer_gate_allows_drive(state" in apply_fn
    assert apply_fn.index("presteer_gate_allows_drive(state") < apply_fn.index("cache_latest_drive_velocity")
    assert "drive_allowed_by_safety &&" in apply_fn


def test_p_gear_allows_visible_steering_but_not_drive_motion(root: pathlib.Path) -> None:
    motion_c = read(root, "ecu/devices/src/motion_device.c")
    arbiter_c = read(root, "ecu/vehicle/src/command_arbiter.c")

    inhibit_fn = motion_c.split("static motion_steer_inhibit_reason_t evaluate_steer_inhibit_reason", 1)[1].split(
        "static bool motion_device_update_steer_safety_gate", 1
    )[0]
    assert "command->active_gear == ECU_GEAR_REQUEST_P" not in inhibit_fn
    assert "command->active_gear != ECU_GEAR_REQUEST_P" in inhibit_fn
    assert "P gear inhibits drive velocity only" in inhibit_fn

    speed_fn = arbiter_c.split("static float remote_speed_command_mps", 1)[1].split(
        "static bool remote_requests_brake_release", 1
    )[0]
    assert "remote->active_gear == ECU_GEAR_REQUEST_P" in speed_fn
    assert "return 0.0f;" in speed_fn


def test_safety_manager_clamps_dangerous_outputs(root: pathlib.Path) -> None:
    text = read(root, "ecu/vehicle/src/safety_manager.c")
    required = [
        "brake_release_allowed",
        "hydraulic_enable = false",
        "high_voltage_enable = false",
        "target_speed_mps = 0.0f",
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
