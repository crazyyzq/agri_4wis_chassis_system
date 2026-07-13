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


def test_sbus_mapping_is_owned_by_remote_layer_not_cpu0_tasks(root: pathlib.Path) -> None:
    mapper_h = read(root, "ecu/remote/include/remote_sbus_mapper.h")
    mapper_c = read(root, "ecu/remote/src/remote_sbus_mapper.c")
    tasks_c = read(root, "ecu/os/src/ecu_tasks_cpu0.c")
    cmake = read(root, "ecu/apps/agri_chassis_control_cpu0/CMakeLists.txt")

    for token in [
        "remote_sbus_sample_t",
        "remote_sbus_mapper_t",
        "remote_sbus_mapper_init",
        "remote_sbus_mapper_build_ppm_sample",
        "remote_sbus_mapper_build_input",
    ]:
        assert token in mapper_h, token

    for token in [
        "sbus_protocol_raw_to_ppm_equivalent",
        "sbus_per_mille_from_raw",
        "sbus_throttle_per_mille_from_raw",
        "sbus_ppm_channels_are_credible",
        "stable_position_from_channel",
        "remote_discrete_channel_t discrete_channels",
    ]:
        assert token not in tasks_c, token

    for token in [
        "sbus_protocol_raw_to_ppm_equivalent",
        "sbus_per_mille_from_ppm",
        "sbus_throttle_per_mille_from_ppm",
        "sbus_ppm_channels_are_credible",
        "stable_position_from_channel",
    ]:
        assert token in mapper_c, token

    assert "remote_discrete_channel_t discrete_channels" in mapper_h
    assert "remote_sbus_mapper.c" in cmake


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
        "domain_changed_since_ms",
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


def test_gear_fsm_holds_active_drive_before_shift_entry_checks(root: pathlib.Path) -> None:
    gear_c = read(root, "ecu/remote/src/remote_gear_fsm.c")

    required_tokens = [
        "gear_request_matches_active_drive",
        "fsm->active_gear == fsm->requested_gear",
        "GEAR_STATE_DRIVE_D",
        "GEAR_STATE_DRIVE_R",
    ]
    for token in required_tokens:
        assert token in gear_c, token

    hold_active_drive_index = gear_c.index("gear_request_matches_active_drive")
    throttle_entry_check_index = gear_c.index("!preconditions->throttle_low")
    assert hold_active_drive_index < throttle_entry_check_index
    assert "preconditions->brake_release_confirmed" not in gear_c
    assert "preconditions->brake_applied" not in gear_c


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
    assert "suspension_request_active" in adjust
    assert "input->gear == REMOTE_POS_CENTER" in adjust
    assert "input->r1_changed" in adjust
    assert "input->r2_changed" in adjust
    assert "REMOTE_HYDRAULIC_SUSPENSION_FRONT" in types
    assert "REMOTE_HYDRAULIC_SUSPENSION_REAR" in types


def test_track_adjust_hysteresis_and_no_manual_steer_in_adjust_domain(root: pathlib.Path) -> None:
    """Track-width mode must debounce CH14 and ignore manual CH1 steering."""

    adjust = read(root, "ecu/remote/src/remote_adjust_fsm.c")

    for token in [
        "raw_track_direction",
        "update_stable_track_direction",
        "remote_adjust_fsm_get_track_direction",
        "track_owner_center_exit_active",
        "track_request_pending",
        "input->track_per_mille >= ECU_REMOTE_TRACK_EXTEND_PER_MILLE_MIN",
        "input->track_per_mille <= ECU_REMOTE_TRACK_RETRACT_PER_MILLE_MAX",
        "ECU_REMOTE_TRACK_EXTEND_RELEASE_PER_MILLE_MIN",
        "ECU_REMOTE_TRACK_RETRACT_RELEASE_PER_MILLE_MAX",
        "ECU_REMOTE_TRACK_REQUEST_STABLE_MS",
        "Do not clear a pending CH14 track-width request",
        "fsm->state = ADJUST_STATE_TRACK_EXITING",
        "ECU_REMOTE_TRACK_ASSIST_CENTER_EXIT_MS",
    ]:
        assert token in adjust, token

    precondition_block = adjust.split("static bool adjust_preconditions_ok", 1)[1].split(
        "static bool clearance_request_active", 1
    )[0]
    assert "preconditions->steering_neutral" not in precondition_block
    assert "entering_adjust_domain" in precondition_block
    assert "(!entering_adjust_domain || preconditions->zero_speed)" in precondition_block
    assert "CH1" in adjust
    assert "not used as an" in adjust
    assert "would abort the track operation and chatter the hydraulic valve" in adjust
    assert "track_owner_allows_non_neutral_steering" not in adjust

    exit_block = adjust.split("track_owner_center_exit_active", 1)[1].split(
        "if (!clearance_active && !track_active && !suspension_active)", 1
    )[0]
    assert "return;" in exit_block


def test_mode_fsm_requires_fresh_r1_r2_event(root: pathlib.Path) -> None:
    mode_c = read(root, "ecu/remote/src/remote_mode_fsm.c")
    mapper_c = read(root, "ecu/remote/src/remote_sbus_mapper.c")

    assert "input->r1_changed" in mode_c
    assert "input->r2_changed" in mode_c
    assert "fresh_r1_event" in mode_c
    assert "fresh_r2_event" in mode_c
    assert "input->r1 == REMOTE_POS_HIGH && input->r1_changed" not in mode_c
    assert "input->r2 == REMOTE_POS_HIGH && input->r2_changed" not in mode_c
    assert "domain_default_pending" in mode_c
    assert "default_mode_for_domain" in mode_c
    assert "ecu_time_elapsed(input->now_ms, fsm->domain_changed_since_ms, guard_ms)" in mode_c
    assert "domain_guard_until_ms" not in mode_c
    assert "input->home == REMOTE_POS_INVALID" in mode_c
    assert "ECU_MOTION_MODE_SPIN" in mode_c
    assert "ECU_MOTION_MODE_POSITIVE_ACKERMANN" in mode_c
    assert "old R1/R2 events" not in mode_c
    assert "out->r1_changed = mapper->discrete_channels[ECU_SBUS_CH_R1].changed" in mapper_c
    assert "out->r2_changed = mapper->discrete_channels[ECU_SBUS_CH_R2].changed" in mapper_c


def test_ch13_estop_latches_at_either_endpoint_and_clears_only_by_center_hold(root: pathlib.Path) -> None:
    estop_c = read(root, "ecu/remote/src/remote_estop_fsm.c")
    estop_h = read(root, "ecu/remote/include/remote_estop_fsm.h")
    mapper_c = read(root, "ecu/remote/src/remote_sbus_mapper.c")

    for token in [
        "ch13_position_requests_estop",
        "input->ch13_estop == REMOTE_POS_LOW",
        "input->ch13_estop == REMOTE_POS_HIGH",
        "DIAG_REMOTE_ESTOP_CH13",
        "update_center_hold",
        "center_hold_active",
        "center_hold_since_ms",
        "estop_reset_preconditions_met",
        "REMOTE_ESTOP_CENTER_HOLD_MS",
    ]:
        assert token in estop_c, token

    assert "last_ch13_position" not in estop_h
    assert "ch13_position_initialized" not in estop_h
    assert "ch13_estop_changed" not in estop_c
    assert "ch13_estop_changed" not in mapper_c
    assert "ch13_estop_stable_since_ms" not in estop_c
    assert "ch13_estop_stable_since_ms" not in mapper_c
    assert "remote_discrete_position_from_raw(ppm_sbus->channels[ECU_SBUS_CH_ESTOP]" not in mapper_c


def test_remote_event_lifetimes_are_configured(root: pathlib.Path) -> None:
    config = read(root, "ecu/config/include/ecu_config.h")
    for token in [
        "REMOTE_EVENT_MODE_REQUEST_TTL_MS",
        "REMOTE_EVENT_POWER_REQUEST_TTL_MS",
        "REMOTE_EVENT_LIGHT_REQUEST_TTL_MS",
        "REMOTE_POWER_LONG_PRESS_MS",
        "REMOTE_ESTOP_CENTER_HOLD_MS",
    ]:
        assert token in config, token

    assert "REMOTE_EVENT_ESTOP_RESET_TTL_MS" not in config

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
