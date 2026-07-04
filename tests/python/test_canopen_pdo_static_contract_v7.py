"""V7 static CANopen PDO contract tests.

These tests verify the frozen PDO contract and production-firmware cleanup.
They do not claim runtime TPDO monitoring or motor motion.
"""

from __future__ import annotations

import pathlib
import re


def read(root: pathlib.Path, rel: str) -> str:
    path = root / rel
    assert path.exists(), f"missing {rel}"
    return path.read_text(encoding="utf-8")


def test_mapping_record_document_contains_complete_node_contract(root: pathlib.Path) -> None:
    doc = read(root, "doc/CANOPEN_PDO_MAPPING_RECORD_V1.md")

    for token in [
        "RPDO0",
        "RPDO1",
        "TPDO0",
        "TPDO1",
        "1600:01 = 0x60400010",
        "1600:02 = 0x60600008",
        "1600:03 = 0x60FF0020",
        "1601:03 = 0x607A0020",
        "1A00:01 = 0x60640020",
        "1A00:02 = 0x606C0020",
        "1A01:01 = 0x21830020",
        "1A01:02 = 0x60410010",
        "1A01:03 = 0x221C0010",
    ]:
        assert token in doc, token

    for node in range(1, 14):
        assert f"| CAN" in doc and f"| {node} |" in doc, f"missing node {node}"

    for token in [
        "`0x201` RPDO0",
        "`0x305` RPDO1",
        "`0x309` RPDO1",
        "`0x30B` RPDO1",
        "`0x30C` RPDO1",
        "`0x30A` RPDO1",
        "`0x20D` RPDO0",
        "CPU0 production firmware does not configure, verify, or save PDO mapping",
    ]:
        assert token in doc, token


def test_contract_table_contains_node1_to_node13_and_fixed_bus_roles(root: pathlib.Path) -> None:
    header = read(root, "ecu/drivers/canopen/include/canopen_pdo_profile.h")
    source = read(root, "ecu/drivers/canopen/src/canopen_pdo_profile.c")

    for token in [
        "canopen_node_pdo_contract_t",
        "CANOPEN_PDO_CONTRACT_NODE_COUNT (13U)",
        "canopen_pdo_contract_find",
        "canopen_pdo_contract_for_drive_leg",
        "canopen_pdo_contract_for_steer_leg",
        "canopen_pdo_contract_for_lift_leg",
        "canopen_pdo_contract_for_hydraulic_pump",
        "canopen_pdo_contract_table",
    ]:
        assert token in header, token

    assert source.count("CANOPEN_MASTER_BUS_CAN2") >= 8
    assert source.count("CANOPEN_MASTER_BUS_CAN3") >= 5

    for token in [
        "ECU_CANOPEN_DRIVE_FR_NODE_ID",
        "ECU_CANOPEN_DRIVE_FL_NODE_ID",
        "ECU_CANOPEN_DRIVE_RL_NODE_ID",
        "ECU_CANOPEN_DRIVE_RR_NODE_ID",
        "ECU_CANOPEN_STEER_FR_NODE_ID",
        "ECU_CANOPEN_STEER_FL_NODE_ID",
        "ECU_CANOPEN_STEER_RL_NODE_ID",
        "ECU_CANOPEN_STEER_RR_NODE_ID",
        "ECU_CANOPEN_LIFT_FR_NODE_ID",
        "ECU_CANOPEN_LIFT_FL_NODE_ID",
        "ECU_CANOPEN_LIFT_RL_NODE_ID",
        "ECU_CANOPEN_LIFT_RR_NODE_ID",
        "ECU_CANOPEN_HYDRAULIC_PUMP_NODE_ID",
    ]:
        assert token in source, token

    for node_token, leg_index in [
        ("ECU_CANOPEN_LIFT_FR_NODE_ID", "0U"),
        ("ECU_CANOPEN_LIFT_FL_NODE_ID", "1U"),
        ("ECU_CANOPEN_LIFT_RL_NODE_ID", "2U"),
        ("ECU_CANOPEN_LIFT_RR_NODE_ID", "3U"),
    ]:
        pattern = (
            rf"\{{ CANOPEN_MASTER_BUS_CAN3,\s*{node_token},\s*{leg_index},\s*"
            r"CANOPEN_AXIS_ROLE_LIFT_POSITION"
        )
        assert re.search(pattern, source), node_token
    assert "CANOPEN_PDO_CONTRACT_PUMP_LEG_INDEX" in source


def test_contract_builders_use_frozen_standard_wire_format(root: pathlib.Path) -> None:
    source = read(root, "ecu/drivers/canopen/src/canopen_pdo_profile.c")
    motion = read(root, "ecu/devices/src/motion_device.c")

    velocity = source.split("bool canopen_pdo_build_velocity_rpdo0", 1)[1].split(
        "bool canopen_pdo_build_position_rpdo1", 1
    )[0]
    position = source.split("bool canopen_pdo_build_position_rpdo1", 1)[1]

    assert "request->cob_id = profile->rpdo0_cob_id" in velocity
    assert "request->size = 7U" in velocity
    assert "request->data[2] = (uint8_t)CANOPEN_PDO_MODE_PROFILE_VELOCITY" in velocity
    assert "write_le_i32(&request->data[3], target_velocity)" in velocity

    assert "request->cob_id = profile->rpdo1_cob_id" in position
    assert "request->size = 7U" in position
    assert "request->data[2] = (uint8_t)CANOPEN_PDO_MODE_PROFILE_POSITION" in position
    assert "write_le_i32(&request->data[3], target_position)" in position

    steer_builder = motion.split("static bool build_steer_rpdo_request", 1)[1].split(
        "static int32_t rate_limit_target_counts", 1
    )[0]
    assert "canopen_pdo_profile_init" in steer_builder
    assert "CANOPEN_AXIS_ROLE_STEER_POSITION" in steer_builder
    assert "canopen_pdo_build_position_rpdo1" in steer_builder
    assert "request->size = 6U" not in steer_builder
    assert "node->rpdo1_cob_id" not in steer_builder


def test_production_firmware_has_no_runtime_mapping_verifier_or_legacy_mapper(root: pathlib.Path) -> None:
    cmake = read(root, "ecu/apps/agri_chassis_control_cpu0/CMakeLists.txt")
    servo_h = read(root, "ecu/devices/include/servo_drive_canopen.h")
    servo_c = read(root, "ecu/devices/src/servo_drive_canopen.c")
    motion_c = read(root, "ecu/devices/src/motion_device.c")

    assert "canopen_pdo_mapping_verifier.c" not in cmake
    assert not (root / "ecu/drivers/canopen/include/canopen_pdo_mapping_verifier.h").exists()
    assert not (root / "ecu/drivers/canopen/src/canopen_pdo_mapping_verifier.c").exists()

    assert "servo_drive_canopen_configure_steer_rpdo" not in servo_h
    assert "servo_drive_canopen_configure_steer_rpdo" not in servo_c
    assert "servo_drive_canopen_configure_steer_rpdo(canopen" not in motion_c
    assert "servo_drive_canopen_prepare_position_mode(canopen" not in motion_c

    normal_motion = re.search(
        r"ecu_device_apply_result_t motion_device_apply[\s\S]*?"
        r"\n}\n\necu_device_apply_result_t motion_device_flush_realtime",
        motion_c,
    )
    assert normal_motion is not None
    assert "servo_drive_canopen_prepare_velocity_mode(canopen" not in normal_motion.group(0)
    assert "servo_drive_canopen_update_target_velocity(canopen" not in normal_motion.group(0)


def test_default_policy_still_blocks_nmt_sync_and_rpdo_output(root: pathlib.Path) -> None:
    config_h = read(root, "ecu/config/include/ecu_config.h")
    motion_c = read(root, "ecu/devices/src/motion_device.c")
    service_c = read(root, "ecu/drivers/canopen/src/canopen_master_service.c")

    assert "#define ECU_CANOPEN_COMMISSIONING_POLICY \\" in config_h
    assert "ECU_CANOPEN_COMMISSIONING_POLICY_MAPPING_VERIFY_ONLY" in config_h
    assert "ECU_CANOPEN_COMMISSIONING_POLICY_NODE5_STEER_PDO_ONLY" in config_h

    assert (
        "ECU_CANOPEN_COMMISSIONING_POLICY != ECU_CANOPEN_COMMISSIONING_POLICY_PDO_OUTPUT_ENABLED"
        in motion_c
    )
    assert "state->steer_next_group_valid = false;" in motion_c

    init_block = service_c.split("bool canopen_master_service_init", 1)[1].split(
        "bool canopen_master_service_process", 1
    )[0]
    assert "CO_NMT_sendCommand" not in init_block
