"""V6 Phase-A CANopen PDO integration contracts.

These tests are source-level safety contracts. They do not claim remote drive
acceptance or motor motion.
"""

from __future__ import annotations

import pathlib
import re


def read(root: pathlib.Path, rel: str) -> str:
    path = root / rel
    assert path.exists(), f"missing {rel}"
    return path.read_text(encoding="utf-8")


def test_standard_pdo_profile_defines_roles_and_legacy_field_comment(root: pathlib.Path) -> None:
    header = read(root, "ecu/drivers/canopen/include/canopen_pdo_profile.h")
    source = read(root, "ecu/drivers/canopen/src/canopen_pdo_profile.c")
    cmake = read(root, "ecu/apps/agri_chassis_control_cpu0/CMakeLists.txt")

    for token in [
        "CANOPEN_AXIS_ROLE_DRIVE_VELOCITY",
        "CANOPEN_AXIS_ROLE_STEER_POSITION",
        "CANOPEN_AXIS_ROLE_LIFT_POSITION",
        "CANOPEN_AXIS_ROLE_HYDRAULIC_VELOCITY",
        "canopen_node_pdo_profile_t",
        "rpdo0_cob_id",
        "rpdo1_cob_id",
        "tpdo0_cob_id",
        "tpdo1_cob_id",
        "required_mode",
        "node->rpdo1_cob_id == DS301 RPDO0",
        "node->rpdo2_cob_id == DS301 RPDO1",
    ]:
        assert token in header, token

    for token in [
        "CANOPEN_PDO_STANDARD_RPDO0_BASE + node_id",
        "CANOPEN_PDO_STANDARD_RPDO1_BASE + node_id",
        "CANOPEN_PDO_STANDARD_TPDO0_BASE + node_id",
        "CANOPEN_PDO_STANDARD_TPDO1_BASE + node_id",
        "CANOPEN_PDO_MODE_PROFILE_VELOCITY",
        "CANOPEN_PDO_MODE_PROFILE_POSITION",
    ]:
        assert token in source, token

    assert "canopen_pdo_profile.c" in cmake
    assert "canopen_pdo_mapping_verifier.c" in cmake


def test_standard_rpdo_builders_have_v6_wire_format(root: pathlib.Path) -> None:
    source = read(root, "ecu/drivers/canopen/src/canopen_pdo_profile.c")
    motion_c = read(root, "ecu/devices/src/motion_device.c")

    velocity = source.split("bool canopen_pdo_build_velocity_rpdo0", 1)[1].split(
        "bool canopen_pdo_build_position_rpdo1", 1
    )[0]
    position = source.split("bool canopen_pdo_build_position_rpdo1", 1)[1]

    for block, cob, mode in [
        (velocity, "profile->rpdo0_cob_id", "CANOPEN_PDO_MODE_PROFILE_VELOCITY"),
        (position, "profile->rpdo1_cob_id", "CANOPEN_PDO_MODE_PROFILE_POSITION"),
    ]:
        assert f"request->cob_id = {cob}" in block
        assert "request->size = 7U" in block
        assert f"request->data[2] = (uint8_t){mode}" in block
        assert "write_le_i32(&request->data[3]" in block

    build_block = motion_c.split("static bool build_steer_rpdo_request", 1)[1].split(
        "static int32_t rate_limit_target_counts", 1
    )[0]
    assert "CANOPEN_AXIS_ROLE_STEER_POSITION" in build_block
    assert "canopen_pdo_build_position_rpdo1" in build_block
    assert "node->rpdo1_cob_id" not in build_block
    assert "request->size = 6U" not in build_block


def test_default_commissioning_policy_has_no_motion_output(root: pathlib.Path) -> None:
    config_h = read(root, "ecu/config/include/ecu_config.h")
    motion_c = read(root, "ecu/devices/src/motion_device.c")
    service_c = read(root, "ecu/drivers/canopen/src/canopen_master_service.c")

    assert "#define ECU_CANOPEN_COMMISSIONING_POLICY \\" in config_h
    assert "ECU_CANOPEN_COMMISSIONING_POLICY_MAPPING_VERIFY_ONLY" in config_h
    assert "ECU_CANOPEN_COMMISSIONING_POLICY_PDO_OUTPUT_ENABLED" in config_h

    assert (
        "ECU_CANOPEN_COMMISSIONING_POLICY != ECU_CANOPEN_COMMISSIONING_POLICY_PDO_OUTPUT_ENABLED"
        in motion_c
    )
    assert "state->steer_next_group_valid = false;" in motion_c
    assert "servo_drive_canopen_configure_steer_rpdo(canopen" not in motion_c
    assert "servo_drive_canopen_prepare_position_mode(canopen" not in motion_c

    init_block = service_c.split("bool canopen_master_service_init", 1)[1].split(
        "bool canopen_master_service_process", 1
    )[0]
    assert "CO_NMT_sendCommand" not in init_block


def test_mapping_verifier_is_read_only_and_request_scoped(root: pathlib.Path) -> None:
    header = read(root, "ecu/drivers/canopen/include/canopen_pdo_mapping_verifier.h")
    source = read(root, "ecu/drivers/canopen/src/canopen_pdo_mapping_verifier.c")

    for token in [
        "canopen_pdo_mapping_check_t",
        "canopen_pdo_mapping_verifier_t",
        "request_sequence",
        "expected_index",
        "expected_subindex",
        "CANOPEN_PDO_MAPPING_VERIFY_PENDING",
    ]:
        assert token in header, token

    assert "canopen_master_service_request_sdo_read" in source
    assert "canopen_master_service_request_sdo_write" not in source
    for token in [
        "ECU_CANOPEN_OBJ_RPDO1_COMM_PARAM",
        "ECU_CANOPEN_OBJ_RPDO1_MAPPING",
        "ECU_CANOPEN_OBJ_RPDO2_COMM_PARAM",
        "ECU_CANOPEN_OBJ_RPDO2_MAPPING",
        "ECU_CANOPEN_OBJ_TPDO1_COMM_PARAM",
        "ECU_CANOPEN_OBJ_TPDO1_MAPPING",
        "ECU_CANOPEN_OBJ_TPDO2_COMM_PARAM",
        "ECU_CANOPEN_OBJ_TPDO2_MAPPING",
    ]:
        assert token in source, token


def test_can3_command_uses_mailbox_and_can3_task_ownership(root: pathlib.Path) -> None:
    header = read(root, "ecu/vehicle/include/vehicle_command_executor.h")
    executor = read(root, "ecu/vehicle/src/vehicle_command_executor.c")
    tasks = read(root, "ecu/os/src/ecu_tasks_cpu0.c")

    assert "vehicle_can3_command_mailbox_t" in header
    assert "vehicle_command_executor_flush_can3_lift_hydraulic" in header
    assert "publish_can3_command_snapshot" in executor
    assert "read_can3_command_snapshot" in executor

    apply_body = re.search(
        r"bool vehicle_command_executor_apply[\s\S]*?"
        r"\n}\n\nbool vehicle_command_executor_flush_can2_motion",
        executor,
    )
    assert apply_body is not None
    assert "lift_hydraulic_device_apply(&s_runtime.lift_hydraulic" not in apply_body.group(0)

    can3_step = re.search(
        r"void ecu_task_can3_lift_hydraulic_step[\s\S]*?\n}\n\nvoid ecu_task_io_service_step",
        tasks,
    )
    assert can3_step is not None
    assert "canopen_master_service_process(&s_runtime.can3_lift_hydraulic_canopen" in can3_step.group(0)
    assert "vehicle_command_executor_flush_can3_lift_hydraulic" in can3_step.group(0)
