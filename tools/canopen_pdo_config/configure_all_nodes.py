"""Configure standard RPDO/TPDO mappings on BC/BC2 CANopen drives.

Default mode is a dry run.  The script only sends SDO/NMT configuration frames
when --apply and --confirm-physical-bus-disconnected-from-ecu are both present.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any

CURRENT_DIR = Path(__file__).resolve().parent
if str(CURRENT_DIR) not in sys.path:
    sys.path.insert(0, str(CURRENT_DIR))

from can_adapter import CanAdapter, CanFrame
from can_adapter_controlcan import ControlCanAdapter
from can_adapter_mock import MockCanAdapter
from canopen_sdo import (
    SdoAbort,
    SdoError,
    decode_sdo_download_response,
    decode_sdo_upload_response,
    encode_nmt_preop,
    encode_sdo_download,
    encode_sdo_upload,
)
from pdo_profiles import BACKUP_OBJECTS, BUS_LABELS, BUS_NODES, NODE_ROLES, build_node_configuration, expected_mapping_values, select_nodes

SAVE_PROFILES = {
    "dc": [(0x2420, 0x00, 4, 0x0000000A), (0x1010, 0x01, 4, 0x65766173)],
    "de_de2": [(0x21B3, 0x00, 4, 0x00000300), (0x1010, 0x02, 4, 0x65766173)],
}


class TransactionLog:
    def __init__(self, path: Path) -> None:
        self.path = path
        self._file = path.open("w", encoding="utf-8")

    def close(self) -> None:
        self._file.close()

    def write(self, entry: dict[str, Any]) -> None:
        self._file.write(json.dumps(entry, ensure_ascii=False, sort_keys=True) + "\n")
        self._file.flush()


def frame_json(frame: CanFrame | None) -> dict[str, object] | None:
    return None if frame is None else frame.to_json()


def parse_nodes(text: str) -> list[int]:
    nodes: set[int] = set()
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            start, end = part.split("-", 1)
            nodes.update(range(int(start), int(end) + 1))
        else:
            nodes.add(int(part))
    invalid = sorted(node for node in nodes if node < 1 or node > 13)
    if invalid:
        raise ValueError(f"invalid node ids: {invalid}")
    return sorted(nodes)


def parse_buses(text: str) -> list[str]:
    buses = [item.strip() for item in text.split(",") if item.strip()]
    invalid = [bus for bus in buses if bus not in BUS_NODES]
    if invalid:
        raise ValueError(f"invalid buses: {invalid}")
    return buses


def sdo_download(adapter: CanAdapter, log: TransactionLog, bus: str, node_id: int,
                 index: int, subindex: int, size: int, value: int, timeout_ms: int) -> None:
    tx = encode_sdo_download(node_id, index, subindex, size, value)
    adapter.send(bus, tx)
    rx = None
    try:
        rx = adapter.receive(bus, 0x580 + node_id, timeout_ms)
        decode_sdo_download_response(node_id, index, subindex, rx)
        status = "ok"
    except Exception as exc:
        status = "error"
        log.write({
            "bus": bus,
            "node_id": node_id,
            "operation": "sdo_download",
            "object": f"0x{index:04X}:{subindex}",
            "size": size,
            "value": value,
            "tx": frame_json(tx),
            "rx": frame_json(rx),
            "status": status,
            "error": str(exc),
        })
        raise
    log.write({
        "bus": bus,
        "node_id": node_id,
        "operation": "sdo_download",
        "object": f"0x{index:04X}:{subindex}",
        "size": size,
        "value": value,
        "tx": frame_json(tx),
        "rx": frame_json(rx),
        "status": status,
    })


def sdo_upload(adapter: CanAdapter, log: TransactionLog, bus: str, node_id: int,
               index: int, subindex: int, timeout_ms: int) -> tuple[int, int]:
    tx = encode_sdo_upload(node_id, index, subindex)
    adapter.send(bus, tx)
    rx = None
    try:
        rx = adapter.receive(bus, 0x580 + node_id, timeout_ms)
        value = decode_sdo_upload_response(node_id, index, subindex, rx)
        status = "ok"
    except Exception as exc:
        status = "error"
        log.write({
            "bus": bus,
            "node_id": node_id,
            "operation": "sdo_upload",
            "object": f"0x{index:04X}:{subindex}",
            "tx": frame_json(tx),
            "rx": frame_json(rx),
            "status": status,
            "error": str(exc),
        })
        raise
    log.write({
        "bus": bus,
        "node_id": node_id,
        "operation": "sdo_upload",
        "object": f"0x{index:04X}:{subindex}",
        "size": value.size,
        "value": value.value,
        "tx": frame_json(tx),
        "rx": frame_json(rx),
        "status": status,
    })
    return value.size, value.value


def write_json(path: Path, data: Any) -> None:
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2, sort_keys=True), encoding="utf-8")


def read_backup(adapter: CanAdapter, log: TransactionLog, bus: str, node_id: int,
                timeout_ms: int) -> dict[str, dict[str, int | str]]:
    values: dict[str, dict[str, int | str]] = {}
    for index, subindex, default_size in BACKUP_OBJECTS:
        size, value = sdo_upload(adapter, log, bus, node_id, index, subindex, timeout_ms)
        values[f"0x{index:04X}:{subindex}"] = {
            "size": size or default_size,
            "value": value,
            "hex": f"0x{value:08X}",
        }
    return values


def send_preop(adapter: CanAdapter, log: TransactionLog, bus: str, node_id: int) -> None:
    frame = encode_nmt_preop(node_id)
    adapter.send(bus, frame)
    log.write({
        "bus": bus,
        "node_id": node_id,
        "operation": "nmt_pre_operational",
        "tx": frame_json(frame),
        "rx": None,
        "status": "sent",
    })
    time.sleep(0.05)


def configure_node(adapter: CanAdapter, log: TransactionLog, bus: str, node_id: int,
                   timeout_ms: int, save_profile: str | None,
                   ack_flash_write: bool, out_dir: Path) -> dict[str, Any]:
    result: dict[str, Any] = {
        "bus": bus,
        "bus_label": BUS_LABELS[bus],
        "node_id": node_id,
        "role": NODE_ROLES[node_id],
        "status": "STARTED",
        "writes": 0,
        "verified": False,
        "saved": False,
        "error": "",
    }
    try:
        before = read_backup(adapter, log, bus, node_id, timeout_ms)
        write_json(out_dir / f"node_{node_id:02d}_before.json", before)
        send_preop(adapter, log, bus, node_id)
        for op in build_node_configuration(node_id):
            sdo_download(adapter, log, bus, node_id, op.index, op.subindex, op.size, op.value, timeout_ms)
            result["writes"] += 1
        after = read_backup(adapter, log, bus, node_id, timeout_ms)
        write_json(out_dir / f"node_{node_id:02d}_after.json", after)
        expected = expected_mapping_values(node_id)
        mismatches = {
            key: {"expected": value, "actual": after.get(key, {}).get("value")}
            for key, value in expected.items()
            if after.get(key, {}).get("value") != value
        }
        if mismatches:
            result["status"] = "FAILED"
            result["error"] = f"readback mismatches: {mismatches}"
            return result
        result["verified"] = True
        result["status"] = "MAPPED_VERIFIED_NOT_SAVED"
        if save_profile is not None and ack_flash_write:
            for index, subindex, size, value in SAVE_PROFILES[save_profile]:
                sdo_download(adapter, log, bus, node_id, index, subindex, size, value, timeout_ms)
                result["writes"] += 1
            result["saved"] = True
            result["status"] = "MAPPED_SAVED_UNPOWER_CYCLE_UNVERIFIED"
    except (SdoAbort, SdoError, TimeoutError, RuntimeError, KeyError, ValueError) as exc:
        result["status"] = "FAILED"
        result["error"] = str(exc)
    return result


def read_node_only(adapter: CanAdapter, log: TransactionLog, bus: str, node_id: int,
                   timeout_ms: int, out_dir: Path) -> dict[str, Any]:
    result: dict[str, Any] = {
        "bus": bus,
        "bus_label": BUS_LABELS[bus],
        "node_id": node_id,
        "role": NODE_ROLES[node_id],
        "status": "READ_ONLY_BACKUP",
        "writes": 0,
        "verified": False,
        "saved": False,
        "error": "",
    }
    try:
        before = read_backup(adapter, log, bus, node_id, timeout_ms)
        write_json(out_dir / f"node_{node_id:02d}_before.json", before)
    except (SdoAbort, SdoError, TimeoutError, RuntimeError, KeyError, ValueError) as exc:
        result["status"] = "FAILED"
        result["error"] = str(exc)
    return result


def make_plan(bus_names: list[str], node_ids: list[int], bitrate: int, backend: str,
              apply: bool, save_profile: str | None) -> dict[str, Any]:
    selected = select_nodes(bus_names, node_ids)
    nodes = []
    for bus, ids in selected.items():
        for node_id in ids:
            nodes.append({
                "bus": bus,
                "bus_label": BUS_LABELS[bus],
                "node_id": node_id,
                "role": NODE_ROLES[node_id],
                "rpdo0": f"0x{0x200 + node_id:03X}",
                "rpdo1": f"0x{0x300 + node_id:03X}",
                "tpdo0": f"0x{0x180 + node_id:03X}",
                "tpdo1": f"0x{0x280 + node_id:03X}",
            })
    return {
        "dry_run": not apply,
        "backend": backend,
        "bitrate": bitrate,
        "save_profile": save_profile,
        "nodes": nodes,
        "safety": {
            "sends_motion_rpdo": False,
            "broadcast_operational": False,
            "broadcast_reset": False,
        },
    }


def write_summary_md(path: Path, summary: dict[str, Any]) -> None:
    lines = [
        "# CANopen PDO configuration summary",
        "",
        f"- backend: {summary['backend']}",
        f"- bitrate: {summary['bitrate']}",
        f"- dry_run: {summary['dry_run']}",
        f"- physical ECU disconnected confirmation: {summary['confirm_physical_bus_disconnected_from_ecu']}",
        f"- flash save profile: {summary['save_profile'] or 'none'}",
        f"- flash save sent: {summary['flash_save_sent']}",
        f"- power cycle verified: {summary['power_cycle_verified']}",
        "",
        "This script does not send motion RPDO frames and does not prove motor motion.",
        "",
        "| Node | Bus | Role | Status | Writes | Saved | Error |",
        "|---:|---|---|---|---:|---|---|",
    ]
    for node_id, node in summary["nodes"].items():
        lines.append(
            f"| {node_id} | {node['bus_label']} | {node['role']} | {node['status']} | "
            f"{node['writes']} | {node['saved']} | {node['error']} |"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run_configuration(bus_names: list[str], node_ids: list[int], adapter: CanAdapter,
                      apply: bool, confirm_physical_bus_disconnected_from_ecu: bool,
                      save_profile: str | None, ack_flash_write: bool,
                      log_dir: Path, backend: str = "mock", bitrate: int = 1000000,
                      timeout_ms: int = 300, retries: int = 1,
                      read_only: bool = False) -> dict[str, Any]:
    if apply and not confirm_physical_bus_disconnected_from_ecu:
        raise RuntimeError("--apply requires --confirm-physical-bus-disconnected-from-ecu")
    if read_only and save_profile is not None:
        raise RuntimeError("--read-only cannot be combined with --save-profile")
    if save_profile is not None and save_profile not in SAVE_PROFILES:
        raise RuntimeError(f"unsupported save profile {save_profile}")
    if save_profile is not None and not ack_flash_write:
        raise RuntimeError("--save-profile requires --ack-flash-write")
    _ = retries
    log_dir.mkdir(parents=True, exist_ok=True)
    plan = make_plan(bus_names, node_ids, bitrate, backend, apply or read_only, save_profile)
    write_json(log_dir / "plan.json", plan)
    selected = select_nodes(bus_names, node_ids)
    summary: dict[str, Any] = {
        "dry_run": not apply and not read_only,
        "read_only": read_only,
        "backend": backend,
        "bitrate": bitrate,
        "confirm_physical_bus_disconnected_from_ecu": confirm_physical_bus_disconnected_from_ecu,
        "save_profile": save_profile,
        "flash_save_sent": False,
        "power_cycle_verified": False,
        "nodes": {},
    }
    if not apply and not read_only:
        for item in plan["nodes"]:
            summary["nodes"][str(item["node_id"])] = {
                **item,
                "status": "DRY_RUN_ONLY",
                "writes": 0,
                "saved": False,
                "error": "",
            }
        write_json(log_dir / "summary.json", summary)
        write_summary_md(log_dir / "summary.md", summary)
        return summary

    log = TransactionLog(log_dir / "transaction_log.jsonl")
    adapter.open(bus_names, bitrate)
    try:
        for bus in bus_names:
            for node_id in selected[bus]:
                if read_only:
                    node_result = read_node_only(adapter, log, bus, node_id, timeout_ms, log_dir)
                else:
                    node_result = configure_node(
                        adapter,
                        log,
                        bus,
                        node_id,
                        timeout_ms,
                        save_profile,
                        ack_flash_write,
                        log_dir,
                    )
                summary["nodes"][str(node_id)] = node_result
                summary["flash_save_sent"] = summary["flash_save_sent"] or bool(node_result["saved"])
    finally:
        adapter.close()
        log.close()
    write_json(log_dir / "summary.json", summary)
    write_summary_md(log_dir / "summary.md", summary)
    return summary


def build_adapter(args: argparse.Namespace) -> CanAdapter:
    if args.backend == "mock":
        return MockCanAdapter()
    if args.backend == "controlcan":
        return ControlCanAdapter(channel_can1=args.channel_can1, channel_can2=args.channel_can2)
    raise RuntimeError(f"unsupported backend {args.backend}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Configure standard CANopen PDO mappings on Node1..13.")
    parser.add_argument("--dry-run", action="store_true", default=True, help="print/write plan only; default")
    parser.add_argument("--apply", action="store_true", help="open CAN and write SDO configuration")
    parser.add_argument("--confirm-physical-bus-disconnected-from-ecu", action="store_true")
    parser.add_argument("--read-only", action="store_true", help="read current mapping only")
    parser.add_argument("--bus", default="can1,can2", help="can1, can2, or can1,can2")
    parser.add_argument("--nodes", default="1-13", help="node list such as 5 or 1-13")
    parser.add_argument("--backend", default="controlcan", choices=["controlcan", "mock"])
    parser.add_argument("--channel-can1", type=int, default=0)
    parser.add_argument("--channel-can2", type=int, default=1)
    parser.add_argument("--bitrate", type=int, default=1000000)
    parser.add_argument("--timeout-ms", type=int, default=300)
    parser.add_argument("--retries", type=int, default=1)
    parser.add_argument("--log-dir", default="out/canopen_pdo_config")
    parser.add_argument("--save-profile", choices=sorted(SAVE_PROFILES))
    parser.add_argument("--ack-flash-write", action="store_true")
    args = parser.parse_args(argv)

    bus_names = parse_buses(args.bus)
    node_ids = parse_nodes(args.nodes)
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    log_dir = Path(args.log_dir) / timestamp
    adapter = build_adapter(args)
    result = run_configuration(
        bus_names=bus_names,
        node_ids=node_ids,
        adapter=adapter,
        apply=args.apply,
        confirm_physical_bus_disconnected_from_ecu=args.confirm_physical_bus_disconnected_from_ecu,
        save_profile=args.save_profile,
        ack_flash_write=args.ack_flash_write,
        log_dir=log_dir,
        backend=args.backend,
        bitrate=args.bitrate,
        timeout_ms=args.timeout_ms,
        retries=args.retries,
        read_only=args.read_only,
    )
    print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
    print(f"Output written to {log_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
