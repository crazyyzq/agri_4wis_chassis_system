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
