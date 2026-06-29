"""Remote-control framework contract tests."""

from __future__ import annotations

import pathlib


def read(root: pathlib.Path, rel: str) -> str:
    path = root / rel
    assert path.exists(), f"missing {rel}"
    return path.read_text(encoding="utf-8")


def test_remote_state_enums_exist(root: pathlib.Path) -> None:
    text = read(root, "ecu/remote/include/remote_types.h")
    required = [
        "REMOTE_LINK_OFFLINE",
        "REMOTE_LINK_QUALIFYING",
        "REMOTE_LINK_ONLINE",
        "REMOTE_LINK_FAILSAFE",
        "REMOTE_ARM_DISARMED",
        "REMOTE_ARM_WAIT_NEUTRAL",
        "REMOTE_ARM_READY",
        "REMOTE_ESTOP_CLEAR",
        "REMOTE_ESTOP_LATCHED",
        "REMOTE_ESTOP_RESET_REQUESTED",
        "REMOTE_ESTOP_CLEAR_WAIT_NORMAL",
        "GEAR_STATE_PARKED_BRAKED",
        "GEAR_STATE_TRACK_COMPLIANT",
        "ADJUST_STATE_TRACK_PREPARE",
        "INDICATOR_HAZARD_SAFETY",
    ]
    for token in required:
        assert token in text, token


def test_remote_modules_expose_owner_update_api(root: pathlib.Path) -> None:
    modules = [
        "remote_link_fsm",
        "remote_arm_fsm",
        "remote_estop_fsm",
        "remote_gear_fsm",
        "remote_mode_fsm",
        "remote_adjust_fsm",
        "remote_power_fsm",
        "remote_authority_fsm",
        "remote_lights_fsm",
        "remote_manager",
    ]
    for module in modules:
        header = read(root, f"ecu/remote/include/{module}.h")
        assert f"{module}_init" in header or module == "remote_manager", module
        assert "update" in header, module
        assert "get_state" in header or "get_request" in header, module


def test_remote_source_has_required_safety_logic_names(root: pathlib.Path) -> None:
    combined = "\n".join(
        read(root, f"ecu/remote/src/{name}.c")
        for name in [
            "remote_arm_fsm",
            "remote_estop_fsm",
            "remote_mode_fsm",
            "remote_adjust_fsm",
            "remote_power_fsm",
            "remote_authority_fsm",
            "remote_gear_fsm",
            "remote_event_lifecycle",
        ]
    )
    required = [
        "neutral_entry_only",
        "estop_source",
        "domain_guard_until_ms",
        "requested_gear",
        "active_gear",
        "TRACK_COMPLIANT",
        "adjust_owner",
        "high_voltage_enable_request",
        "auto_control_allowed",
        "expires_at_ms",
        "request_rejected",
    ]
    for token in required:
        assert token in combined, token


def test_remote_input_model_keeps_sbus_channels_distinct(root: pathlib.Path) -> None:
    types = read(root, "ecu/remote/include/remote_types.h")
    for token in [
        "clearance",
        "power",
        "authority",
        "track",
        "ch13_estop",
        "r1_changed",
        "r2_changed",
    ]:
        assert token in types, token

    adjust = read(root, "ecu/remote/src/remote_adjust_fsm.c")
    assert "input->clearance" in adjust
    assert "input->track" in adjust
    assert "DIAG_REJECT_ADJUST_OWNER_CONFLICT" in adjust


def test_mode_fsm_requires_fresh_r1_r2_event(root: pathlib.Path) -> None:
    mode_c = read(root, "ecu/remote/src/remote_mode_fsm.c")
    tasks_c = read(root, "ecu/os/src/ecu_tasks_cpu0.c")

    assert "input->r1_changed" in mode_c
    assert "input->r2_changed" in mode_c
    assert "fresh_r1_event" in mode_c
    assert "fresh_r2_event" in mode_c
    assert "old R1/R2 events" not in mode_c
    assert "out->r1_changed = s_runtime.discrete_channels[ECU_SBUS_CH_R1].changed" in tasks_c
    assert "out->r2_changed = s_runtime.discrete_channels[ECU_SBUS_CH_R2].changed" in tasks_c


def test_remote_event_lifetimes_are_configured(root: pathlib.Path) -> None:
    config = read(root, "ecu/config/include/ecu_config.h")
    for token in [
        "REMOTE_EVENT_MODE_REQUEST_TTL_MS",
        "REMOTE_EVENT_POWER_REQUEST_TTL_MS",
        "REMOTE_EVENT_ESTOP_RESET_TTL_MS",
        "REMOTE_EVENT_LIGHT_REQUEST_TTL_MS",
        "REMOTE_POWER_LONG_PRESS_MS",
    ]:
        assert token in config, token

    event = read(root, "ecu/remote/include/remote_event_lifecycle.h")
    assert "remote_event_lifecycle_consume" in event
    assert "not queued" in event


def test_sbus_decoder_and_service_boundaries_exist(root: pathlib.Path) -> None:
    decoder_h = read(root, "ecu/protocol/sbus/include/sbus_decoder.h")
    service_h = read(root, "ecu/drivers/sbus/include/sbus_service.h")
    assert "SBUS_FRAME_LENGTH" in decoder_h
    assert "sbus_decode_frame" in decoder_h
    assert "sbus_service_feed_byte_from_isr" in service_h
    assert "sbus_service_get_snapshot" in service_h
    assert "volatile" not in decoder_h


def test_remote_layer_has_no_hardware_includes(root: pathlib.Path) -> None:
    forbidden = ["board.h", "hpm_can", "hpm_gpio", "vehicle_command_executor"]
    for path in (root / "ecu/remote").rglob("*.[ch]"):
        text = path.read_text(encoding="utf-8")
        for token in forbidden:
            assert token not in text, f"{path}: forbidden {token}"
