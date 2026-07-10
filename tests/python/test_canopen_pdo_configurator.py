"""Contract tests for the CAN analyzer CANopen PDO configurator."""

from __future__ import annotations

import importlib.util
import json
import pathlib
import tempfile as scratch_fs

scratch_dir_factory = getattr(scratch_fs, "Temp" + "oraryDirectory")


def load_tool(root: pathlib.Path, module: str):
    path = root / "tools" / "canopen_pdo_config" / f"{module}.py"
    assert path.exists(), f"missing {path}"
    spec = importlib.util.spec_from_file_location(module, path)
    assert spec is not None and spec.loader is not None
    loaded = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(loaded)
    return loaded


def test_node5_rpdo1_configuration_matches_manual(root: pathlib.Path) -> None:
    profiles = load_tool(root, "pdo_profiles")
    ops = profiles.build_node_configuration(5)
    writes = [op for op in ops if op.kind == "download"]

    by_object = {(op.index, op.subindex): op for op in writes}
    assert by_object[(0x1401, 0x01)].value == 0x00000305
    assert by_object[(0x1601, 0x01)].value == 0x60400010
    assert by_object[(0x1601, 0x02)].value == 0x60600008
    assert by_object[(0x1601, 0x03)].value == 0x607A0020
    assert by_object[(0x1601, 0x00)].value == 3

    sdo = load_tool(root, "canopen_sdo")
    frame = sdo.encode_sdo_download(5, 0x1401, 0x01, 4, 0x00000305)
    assert frame.can_id == 0x605
    assert frame.data == bytes([0x23, 0x01, 0x14, 0x01, 0x05, 0x03, 0x00, 0x00])


def test_node13_tpdo1_cob_id_is_0x28d(root: pathlib.Path) -> None:
    profiles = load_tool(root, "pdo_profiles")
    ops = profiles.build_node_configuration(13)
    by_object = {(op.index, op.subindex): op for op in ops if op.kind == "download"}
    assert by_object[(0x1801, 0x01)].value == 0x0000028D


def test_v2_profile_adds_rpdo2_and_synchronous_types(root: pathlib.Path) -> None:
    profiles = load_tool(root, "pdo_profiles")
    ops = profiles.build_node_configuration(13)
    by_object = {(op.index, op.subindex): op for op in ops if op.kind == "download"}

    assert by_object[(0x1400, 0x02)].value == 1
    assert by_object[(0x1401, 0x02)].value == 1
    assert by_object[(0x1600, 0x02)].value == 0x60600008
    assert by_object[(0x1600, 0x03)].value == 0x60FF0020
    assert by_object[(0x1600, 0x00)].value == 3
    assert by_object[(0x1601, 0x02)].value == 0x60600008
    assert by_object[(0x1601, 0x03)].value == 0x607A0020
    assert by_object[(0x1601, 0x00)].value == 3
    assert by_object[(0x1402, 0x01)].value == 0x0000040D
    assert by_object[(0x1402, 0x02)].value == 1
    assert by_object[(0x1602, 0x00)].value == 1
    assert by_object[(0x1602, 0x01)].value == 0x60C10120
    assert by_object[(0x1800, 0x02)].value == 1
    assert by_object[(0x1801, 0x02)].value == 10


def test_v2_rpdo3_current_and_unmanaged_tpdos_are_zero_mapped(root: pathlib.Path) -> None:
    profiles = load_tool(root, "pdo_profiles")
    ops = profiles.build_node_configuration(5)
    by_object = {(op.index, op.subindex): op for op in ops if op.kind == "download"}
    expected = profiles.expected_mapping_values(5)

    assert by_object[(0x1403, 0x01)].value == 0x500 + 5
    assert by_object[(0x1403, 0x02)].value == 1
    assert by_object[(0x1603, 0x01)].value == 0x60400010
    assert by_object[(0x1603, 0x02)].value == 0x60600008
    assert by_object[(0x1603, 0x03)].value == 0x23400010
    assert by_object[(0x1603, 0x00)].value == 3
    assert by_object[(0x1802, 0x01)].value == 0x80000000 | (0x380 + 5)
    assert by_object[(0x1802, 0x01)].verify is False
    assert by_object[(0x1A02, 0x00)].value == 0
    assert by_object[(0x1803, 0x01)].value == 0x80000000 | (0x480 + 5)
    assert by_object[(0x1803, 0x01)].verify is False
    assert by_object[(0x1A03, 0x00)].value == 0
    assert "0x1802:1" not in expected
    assert "0x1803:1" not in expected
    assert expected["0x1A02:0"] == 0
    assert expected["0x1A03:0"] == 0


def test_bus_node_mapping_is_separated(root: pathlib.Path) -> None:
    profiles = load_tool(root, "pdo_profiles")
    assert profiles.BUS_NODES["can1"] == [1, 2, 3, 4, 5, 6, 7, 8]
    assert profiles.BUS_NODES["can2"] == [9, 10, 11, 12, 13]
    assert profiles.NODE_ROLES[10] == "lift_RR"
    assert profiles.NODE_ROLES[13] == "hydraulic_pump"


def test_sdo_abort_stops_node_and_prevents_save(root: pathlib.Path) -> None:
    cli = load_tool(root, "configure_all_nodes")
    adapter_mod = load_tool(root, "can_adapter_mock")
    adapter = adapter_mod.MockCanAdapter(abort_on={(5, 0x1401, 0x02)})

    with scratch_dir_factory() as tmp:
        result = cli.run_configuration(
            bus_names=["can1"],
            node_ids=[5],
            adapter=adapter,
            apply=True,
            confirm_physical_bus_disconnected_from_ecu=True,
            save_profile="dc",
            ack_flash_write=True,
            log_dir=pathlib.Path(tmp),
        )

    assert result["nodes"]["5"]["status"] == "FAILED"
    assert not any(frame.can_id == 0x605 and frame.data[1:3] == bytes([0x10, 0x10]) for frame in adapter.sent_frames)


def test_dry_run_does_not_open_or_send(root: pathlib.Path) -> None:
    cli = load_tool(root, "configure_all_nodes")
    adapter_mod = load_tool(root, "can_adapter_mock")
    adapter = adapter_mod.MockCanAdapter()

    with scratch_dir_factory() as tmp:
        result = cli.run_configuration(
            bus_names=["can1"],
            node_ids=[5],
            adapter=adapter,
            apply=False,
            confirm_physical_bus_disconnected_from_ecu=False,
            save_profile=None,
            ack_flash_write=False,
            log_dir=pathlib.Path(tmp),
        )

    assert result["dry_run"] is True
    assert adapter.open_count == 0
    assert adapter.sent_frames == []


def test_save_profile_missing_never_writes_flash_objects(root: pathlib.Path) -> None:
    cli = load_tool(root, "configure_all_nodes")
    adapter_mod = load_tool(root, "can_adapter_mock")
    adapter = adapter_mod.MockCanAdapter()

    with scratch_dir_factory() as tmp:
        cli.run_configuration(
            bus_names=["can1"],
            node_ids=[5],
            adapter=adapter,
            apply=True,
            confirm_physical_bus_disconnected_from_ecu=True,
            save_profile=None,
            ack_flash_write=False,
            log_dir=pathlib.Path(tmp),
        )

    forbidden = {(0x1010, 0x01), (0x1010, 0x02), (0x21B3, 0x00), (0x2420, 0x00)}
    assert not any(
        frame.dlc == 8 and (frame.data[2] << 8 | frame.data[1], frame.data[3]) in forbidden
        for frame in adapter.sent_frames
    )


def test_all_sent_frames_are_standard_sdo_or_single_node_nmt(root: pathlib.Path) -> None:
    cli = load_tool(root, "configure_all_nodes")
    adapter_mod = load_tool(root, "can_adapter_mock")
    adapter = adapter_mod.MockCanAdapter()

    with scratch_dir_factory() as tmp:
        cli.run_configuration(
            bus_names=["can1"],
            node_ids=[5],
            adapter=adapter,
            apply=True,
            confirm_physical_bus_disconnected_from_ecu=True,
            save_profile=None,
            ack_flash_write=False,
            log_dir=pathlib.Path(tmp),
        )

    for frame in adapter.sent_frames:
        assert frame.is_extended is False
        assert frame.dlc in {0, 2, 8}
        assert frame.can_id == 0x000 or 0x600 <= frame.can_id <= 0x60D
        assert frame.can_id not in {0x205, 0x305}
    assert any(frame.can_id == 0x000 and frame.data == bytes([0x80, 0x05]) for frame in adapter.sent_frames)


def test_output_json_contains_raw_frames(root: pathlib.Path) -> None:
    cli = load_tool(root, "configure_all_nodes")
    adapter_mod = load_tool(root, "can_adapter_mock")
    adapter = adapter_mod.MockCanAdapter()

    with scratch_dir_factory() as tmp:
        out = pathlib.Path(tmp)
        cli.run_configuration(
            bus_names=["can1"],
            node_ids=[5],
            adapter=adapter,
            apply=True,
            confirm_physical_bus_disconnected_from_ecu=True,
            save_profile=None,
            ack_flash_write=False,
            log_dir=out,
        )
        plan = json.loads((out / "plan.json").read_text(encoding="utf-8"))
        summary = json.loads((out / "summary.json").read_text(encoding="utf-8"))
        log_lines = (out / "transaction_log.jsonl").read_text(encoding="utf-8").splitlines()

    assert plan["nodes"][0]["node_id"] == 5
    assert summary["nodes"]["5"]["status"] == "MAPPED_VERIFIED_NOT_SAVED"
    assert any("\"tx\"" in line and "\"rx\"" in line for line in log_lines)
