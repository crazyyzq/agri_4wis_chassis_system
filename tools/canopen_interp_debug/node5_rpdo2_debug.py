"""Node5 RPDO2 interpolation commissioning helper.

This script is intentionally scoped to a single steering drive: Node5 on the
ECU_CAN2 motion bus through the CAN analyzer CAN1 channel.  It is a bench
commissioning tool, not production firmware.

Safety defaults:
  * no NMT reset frames are ever sent;
  * no Flash/NVM save is ever sent;
  * only Node5 is addressed;
  * the default command is read-only precheck.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

THIS_FILE = Path(__file__).resolve()
REPO_ROOT = THIS_FILE.parents[2]
PDO_TOOL_DIR = REPO_ROOT / "tools" / "canopen_pdo_config"
CAN_TOOL_DIR = REPO_ROOT / "tools" / "can"
for path in (PDO_TOOL_DIR, CAN_TOOL_DIR):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from can_adapter import CanFrame  # noqa: E402
from can_adapter_controlcan import ControlCanAdapter  # noqa: E402


NODE_ID = 5
BUS = "can1"
SYNC_COB_ID = 0x080
NMT_COB_ID = 0x000

SDO_RX_ID = 0x600 + NODE_ID
SDO_TX_ID = 0x580 + NODE_ID
RPDO1_ID = 0x300 + NODE_ID
RPDO2_ID = 0x400 + NODE_ID
TPDO0_ID = 0x180 + NODE_ID
TPDO1_ID = 0x280 + NODE_ID
EMCY_ID = 0x080 + NODE_ID


READ_OBJECTS: tuple[tuple[int, int, int, str], ...] = (
    (0x6061, 0x00, 1, "mode_display"),
    (0x6041, 0x00, 2, "statusword"),
    (0x6064, 0x00, 4, "actual_position"),
    (0x606C, 0x00, 4, "actual_velocity"),
    (0x2300, 0x00, 2, "control_source"),
    (0x60C0, 0x00, 1, "interpolation_submode"),
    (0x60C2, 0x01, 1, "interpolation_period_value"),
    (0x60C2, 0x02, 1, "interpolation_period_index"),
    (0x2011, 0x00, 2, "buffer_free_slots"),
    (0x2012, 0x00, 4, "buffer_status"),
    (0x2013, 0x00, 4, "buffer_next_segment"),
    (0x1402, 0x01, 4, "rpdo2_cob_id"),
    (0x1402, 0x02, 1, "rpdo2_transmission_type"),
    (0x1602, 0x00, 1, "rpdo2_map_count"),
    (0x1602, 0x01, 4, "rpdo2_map_1"),
    (0x2180, 0x00, 4, "event_2180"),
    (0x2181, 0x00, 4, "event_2181"),
    (0x2183, 0x00, 4, "latched_fault"),
    (0x60F4, 0x00, 4, "following_error"),
)


@dataclass(frozen=True)
class SdoValue:
    index: int
    subindex: int
    size: int
    value: int
    signed_value: int
    name: str


class Node5Debug:
    def __init__(self, timeout_ms: int, log_dir: Path) -> None:
        self.timeout_ms = timeout_ms
        self.log_dir = log_dir
        self.adapter = ControlCanAdapter()
        self.events: list[dict[str, object]] = []

    def __enter__(self) -> "Node5Debug":
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.adapter.open([BUS], 1_000_000)
        return self

    def __exit__(self, exc_type, exc, tb) -> None:  # type: ignore[no-untyped-def]
        self.write_json("events.json", self.events)
        self.adapter.close()

    def event(self, kind: str, **fields: object) -> None:
        record = {"time_s": time.time(), "kind": kind}
        record.update(fields)
        self.events.append(record)

    def write_json(self, name: str, data: object) -> None:
        (self.log_dir / name).write_text(
            json.dumps(data, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
            encoding="utf-8",
        )

    def send(self, frame: CanFrame, note: str) -> None:
        self.adapter.send(BUS, frame)
        self.event("tx", note=note, can_id=f"0x{frame.can_id:03X}", data=frame.data.hex(" ").upper())

    def receive_expected(self, can_id: int, timeout_ms: int | None = None) -> CanFrame:
        frame = self.adapter.receive(BUS, can_id, timeout_ms or self.timeout_ms)
        self.event("rx", can_id=f"0x{frame.can_id:03X}", data=frame.data.hex(" ").upper())
        return frame

    def collect_frames(self, duration_ms: int) -> list[dict[str, object]]:
        deadline = time.monotonic() + duration_ms / 1000.0
        frames: list[dict[str, object]] = []
        while time.monotonic() < deadline:
            for can_id in (TPDO0_ID, TPDO1_ID, EMCY_ID):
                try:
                    frame = self.adapter.receive(BUS, can_id, 20)
                except TimeoutError:
                    continue
                record = {
                    "can_id": f"0x{frame.can_id:03X}",
                    "data": frame.data.hex(" ").upper(),
                    "decoded": self.decode_feedback(frame),
                }
                frames.append(record)
                self.event("rx_observe", **record)
        return frames

    @staticmethod
    def encode_sdo_download(index: int, subindex: int, size: int, value: int, signed: bool = False) -> CanFrame:
        if size not in (1, 2, 4):
            raise ValueError(f"unsupported SDO size {size}")
        command = {1: 0x2F, 2: 0x2B, 4: 0x23}[size]
        payload = bytearray(8)
        payload[0] = command
        payload[1] = index & 0xFF
        payload[2] = (index >> 8) & 0xFF
        payload[3] = subindex & 0xFF
        payload[4:4 + size] = int(value).to_bytes(size, "little", signed=signed)
        return CanFrame(SDO_RX_ID, bytes(payload))

    @staticmethod
    def encode_sdo_upload(index: int, subindex: int) -> CanFrame:
        return CanFrame(SDO_RX_ID, bytes([0x40, index & 0xFF, (index >> 8) & 0xFF, subindex, 0, 0, 0, 0]))

    def sdo_download(self, index: int, subindex: int, size: int, value: int, name: str, signed: bool = False) -> None:
        self.send(self.encode_sdo_download(index, subindex, size, value, signed=signed), f"SDO write {name}")
        ack = self.receive_expected(SDO_TX_ID)
        if ack.data[0] == 0x80:
            abort_code = int.from_bytes(ack.data[4:8], "little")
            raise RuntimeError(f"SDO abort write {name} 0x{index:04X}:{subindex} abort=0x{abort_code:08X}")
        expected = bytes([0x60, index & 0xFF, (index >> 8) & 0xFF, subindex, 0, 0, 0, 0])
        if ack.data != expected:
            raise RuntimeError(f"unexpected SDO write ack for {name}: {ack.data.hex(' ').upper()}")

    def sdo_upload(self, index: int, subindex: int, size: int, name: str) -> SdoValue:
        self.send(self.encode_sdo_upload(index, subindex), f"SDO read {name}")
        frame = self.receive_expected(SDO_TX_ID)
        if frame.data[0] == 0x80:
            abort_code = int.from_bytes(frame.data[4:8], "little")
            raise RuntimeError(f"SDO abort read {name} 0x{index:04X}:{subindex} abort=0x{abort_code:08X}")
        actual_size_by_cmd = {0x4F: 1, 0x4B: 2, 0x43: 4}
        actual_size = actual_size_by_cmd.get(frame.data[0])
        if actual_size is None:
            raise RuntimeError(f"unsupported SDO read response command=0x{frame.data[0]:02X} for {name}")
        raw = frame.data[4:4 + actual_size]
        value = int.from_bytes(raw, "little", signed=False)
        signed_value = int.from_bytes(raw, "little", signed=True)
        return SdoValue(index, subindex, actual_size, value, signed_value, name)

    def read_precheck(self, filename: str) -> dict[str, dict[str, object]]:
        values: dict[str, dict[str, object]] = {}
        for index, subindex, size, name in READ_OBJECTS:
            key = f"0x{index:04X}:{subindex}"
            try:
                item = self.sdo_upload(index, subindex, size, name)
                values[key] = {
                    "name": name,
                    "status": "ok",
                    "size": item.size,
                    "value": item.value,
                    "signed_value": item.signed_value,
                    "hex": f"0x{item.value:0{item.size * 2}X}",
                }
            except RuntimeError as exc:
                # Some vendor diagnostic objects are firmware-option dependent.
                # A missing optional object is evidence to record, not a reason
                # to discard the rest of the precheck.
                values[key] = {
                    "name": name,
                    "status": "error",
                    "error": str(exc),
                }
        self.write_json(filename, values)
        return values

    def nmt_operational(self) -> None:
        self.send(CanFrame(NMT_COB_ID, bytes([0x01, NODE_ID])), "NMT operational Node5")

    def sync(self) -> None:
        self.send(CanFrame(SYNC_COB_ID, b""), "SYNC")

    @staticmethod
    def rpdo1_payload(controlword: int, target_counts: int) -> bytes:
        return (
            int(controlword).to_bytes(2, "little")
            + bytes([0x01])
            + int(target_counts).to_bytes(4, "little", signed=True)
        )

    @staticmethod
    def rpdo2_payload(point_counts: int, time_value_ms: int | None = None,
                      velocity_counts: int | None = None) -> bytes:
        payload = bytearray(int(point_counts).to_bytes(4, "little", signed=True))
        if time_value_ms is not None:
            payload.append(time_value_ms & 0xFF)
        if velocity_counts is not None:
            payload.extend(int(velocity_counts).to_bytes(4, "little", signed=True))
        return bytes(payload)

    @staticmethod
    def rpdo2_2010_buffer_command(command: int, argument: int = 0) -> bytes:
        return bytes([0x80 | (command & 0x7F), argument & 0xFF, 0, 0, 0, 0, 0, 0])

    @staticmethod
    def rpdo2_2010_linear_abs_segment(counter: int, time_ms: int, position_counts: int) -> bytes:
        payload = bytearray(8)
        payload[0] = ((5 & 0x0F) << 3) | (counter & 0x07)
        payload[1] = time_ms & 0xFF
        payload[2:6] = int(position_counts).to_bytes(4, "little", signed=True)
        return bytes(payload)

    def wait_position(self, target_counts: int, tolerance_counts: int, timeout_ms: int, stable_samples: int = 3) -> bool:
        deadline = time.monotonic() + timeout_ms / 1000.0
        stable = 0
        last: dict[str, object] = {}
        while time.monotonic() < deadline:
            position = self.sdo_upload(0x6064, 0x00, 4, "actual position wait")
            velocity = self.sdo_upload(0x606C, 0x00, 4, "actual velocity wait")
            statusword = self.sdo_upload(0x6041, 0x00, 2, "statusword wait")
            error = position.signed_value - target_counts
            last = {
                "target": target_counts,
                "actual_position": position.signed_value,
                "actual_velocity": velocity.signed_value,
                "statusword": f"0x{statusword.value:04X}",
                "error": error,
            }
            self.event("position_wait_sample", **last)
            if abs(error) <= tolerance_counts and abs(velocity.signed_value) <= 50_000:
                stable += 1
                if stable >= stable_samples:
                    self.event("position_wait_done", result="reached", **last)
                    return True
            else:
                stable = 0
            time.sleep(0.05)
        self.event("position_wait_done", result="timeout", **last)
        return False

    def send_rpdo1_absolute_edge(self, target_counts: int, wait_timeout_ms: int = 12000) -> bool:
        self.send(CanFrame(RPDO1_ID, self.rpdo1_payload(0x002F, target_counts)), f"RPDO1 arm target {target_counts}")
        self.sync()
        self.collect_frames(80)
        self.send(CanFrame(RPDO1_ID, self.rpdo1_payload(0x003F, target_counts)), f"RPDO1 trigger target {target_counts}")
        self.sync()
        self.collect_frames(250)
        return self.wait_position(target_counts, tolerance_counts=5_000, timeout_ms=wait_timeout_ms)

    def setup_common_canopen(self) -> None:
        self.nmt_operational()
        time.sleep(0.05)
        self.sdo_download(0x2300, 0x00, 2, 0x001E, "control source CANopen")
        self.sdo_download(0x6040, 0x00, 2, 0x000F, "enable operation")

    def setup_position_mode(self) -> None:
        self.setup_common_canopen()
        self.sdo_download(0x6060, 0x00, 1, 0x01, "profile position mode")
        mode = self.sdo_upload(0x6061, 0x00, 1, "mode display")
        if mode.signed_value != 1:
            raise RuntimeError(f"mode display is {mode.signed_value}, expected 1")

    def setup_interpolation_mode(self, period_ms: int, submode: int) -> None:
        self.setup_common_canopen()
        self.sdo_download(0x6040, 0x00, 2, 0x000F, "clear interpolation trigger")
        self.sdo_download(0x6060, 0x00, 1, 0x07, "interpolated position mode")
        self.sdo_download(0x60C0, 0x00, 2, submode, "interpolation submode", signed=(submode < 0))
        if submode == 0:
            self.sdo_download(0x60C2, 0x01, 1, period_ms, "interpolation period value")
            self.sdo_download(0x60C2, 0x02, 1, -3, "interpolation period index ms", signed=True)
        mode = self.sdo_upload(0x6061, 0x00, 1, "mode display")
        if mode.signed_value != 7:
            raise RuntimeError(f"mode display is {mode.signed_value}, expected 7")

    def configure_rpdo2_extended_mapping(self, variant: str) -> dict[str, int]:
        original = {
            "0x1402:1": self.sdo_upload(0x1402, 0x01, 4, "original RPDO2 COB-ID").value,
            "0x1402:2": self.sdo_upload(0x1402, 0x02, 1, "original RPDO2 transmission type").value,
            "0x1602:0": self.sdo_upload(0x1602, 0x00, 1, "original RPDO2 map count").value,
            "0x1602:1": self.sdo_upload(0x1602, 0x01, 4, "original RPDO2 map 1").value,
        }
        try:
            original["0x1602:2"] = self.sdo_upload(0x1602, 0x02, 4, "original RPDO2 map 2").value
        except RuntimeError as exc:
            self.event("rpdo2_map2_read_unavailable", error=str(exc))
            original["0x1602:2"] = 0

        cob = 0x400 + NODE_ID
        try:
            self.sdo_download(0x1402, 0x01, 4, 0x80000000 | cob, "disable RPDO2 before remap")
            self.sdo_download(0x1602, 0x00, 1, 0, "clear RPDO2 map")
            self.sdo_download(0x1602, 0x01, 4, 0x60C10120, "RPDO2 map position")
            if variant == "position_time":
                self.sdo_download(0x1602, 0x02, 4, 0x60C10208, "RPDO2 map interpolation time")
                self.sdo_download(0x1602, 0x00, 1, 2, "enable RPDO2 5-byte map")
            elif variant == "position_velocity":
                self.sdo_download(0x1602, 0x02, 4, 0x60C10320, "RPDO2 map interpolation velocity")
                self.sdo_download(0x1602, 0x00, 1, 2, "enable RPDO2 8-byte map")
            elif variant == "pvt2010_linear_abs":
                self.sdo_download(0x1602, 0x01, 4, 0x20100040, "RPDO2 map 2010 IP move segment command")
                self.sdo_download(0x1602, 0x00, 1, 1, "enable RPDO2 2010 8-byte map")
            elif variant == "position_only":
                self.sdo_download(0x1602, 0x00, 1, 1, "enable RPDO2 4-byte map")
            else:
                raise ValueError(f"unsupported RPDO2 mapping variant {variant}")
            self.sdo_download(0x1402, 0x01, 4, cob, "enable RPDO2 after remap")
        except Exception:
            self.restore_rpdo2_mapping(original)
            raise
        return original

    def restore_rpdo2_mapping(self, original: dict[str, int]) -> None:
        cob = original["0x1402:1"] & 0x7FFFFFFF
        self.sdo_download(0x1402, 0x01, 4, 0x80000000 | cob, "disable RPDO2 before restore")
        self.sdo_download(0x1602, 0x00, 1, 0, "clear RPDO2 restore map")
        self.sdo_download(0x1602, 0x01, 4, original["0x1602:1"], "restore RPDO2 map 1")
        if original["0x1602:0"] >= 2:
            self.sdo_download(0x1602, 0x02, 4, original["0x1602:2"], "restore RPDO2 map 2")
        self.sdo_download(0x1602, 0x00, 1, original["0x1602:0"], "restore RPDO2 map count")
        self.sdo_download(0x1402, 0x02, 1, original["0x1402:2"], "restore RPDO2 transmission type")
        self.sdo_download(0x1402, 0x01, 4, original["0x1402:1"], "restore RPDO2 COB-ID")

    def run_rpdo1_baseline(self, amplitude: int) -> None:
        self.setup_position_mode()
        self.read_precheck("rpdo1_before.json")
        if not self.send_rpdo1_absolute_edge(0):
            raise RuntimeError("RPDO1 baseline failed to reach zero before test")
        if not self.send_rpdo1_absolute_edge(amplitude):
            raise RuntimeError(f"RPDO1 baseline failed to reach +{amplitude}")
        if not self.send_rpdo1_absolute_edge(-amplitude):
            raise RuntimeError(f"RPDO1 baseline failed to reach -{amplitude}")
        if not self.send_rpdo1_absolute_edge(0):
            raise RuntimeError("RPDO1 baseline failed to return zero")
        time.sleep(0.2)
        self.read_precheck("rpdo1_after.json")

    def run_rpdo2_short(self, amplitude: int, period_ms: int, preload: int, samples: int,
                        rpdo2_type: int | None, trigger_before_points: bool,
                        no_live_feedback: bool, tail_zero_count: int,
        interpolation_start_controlword: int, rpdo2_mapping_variant: str) -> None:
        rpdo2_mapping_original: dict[str, int] | None = None
        use_2010 = rpdo2_mapping_variant == "pvt2010_linear_abs"
        submode = -1 if rpdo2_mapping_variant == "position_time" else 0
        if rpdo2_mapping_variant != "position_only":
            rpdo2_mapping_original = self.configure_rpdo2_extended_mapping(rpdo2_mapping_variant)
        self.setup_interpolation_mode(period_ms, submode)
        original_type: int | None = None
        if rpdo2_type is not None:
            current_type = self.sdo_upload(0x1402, 0x02, 1, "current RPDO2 transmission type")
            original_type = current_type.value
            if original_type != rpdo2_type:
                self.sdo_download(0x1402, 0x02, 1, rpdo2_type, "temporary RPDO2 transmission type")
        self.read_precheck("rpdo2_before.json")
        try:
            # Start from the known absolute-zero baseline with RPDO1 before
            # switching to interpolation. This is intentionally not repeated
            # during streaming.
            self.sdo_download(0x6040, 0x00, 2, 0x000F, "interpolation trigger low")
            points = self.make_triangle_points(amplitude, samples)
            if tail_zero_count > 0:
                points.extend([0] * tail_zero_count)
            if preload > len(points):
                preload = len(points)
            if use_2010:
                for note, payload in (
                    ("2010 clear buffer", self.rpdo2_2010_buffer_command(0)),
                    ("2010 clear buffer errors", self.rpdo2_2010_buffer_command(2, 0xFF)),
                    ("2010 reset segment id", self.rpdo2_2010_buffer_command(3)),
                ):
                    self.send(CanFrame(RPDO2_ID, payload), note)
                    self.sync()
                    time.sleep(period_ms / 1000.0)
            velocity_step = int((2 * amplitude) / max(1, samples) * (1000 / period_ms))
            def point_velocity(point_index: int) -> int:
                if point_index < (samples // 2):
                    return velocity_step
                if point_index < samples:
                    return -velocity_step
                return 0

            if trigger_before_points:
                self.sdo_download(
                    0x6040,
                    0x00,
                    2,
                    interpolation_start_controlword,
                    "start interpolation trigger before points",
                )
                preload = 0
            else:
                for idx, point in enumerate(points[:preload]):
                    if use_2010:
                        payload = self.rpdo2_2010_linear_abs_segment(idx, period_ms, point)
                    else:
                        payload = self.rpdo2_payload(
                            point,
                            period_ms if rpdo2_mapping_variant == "position_time" else None,
                            point_velocity(idx) if rpdo2_mapping_variant == "position_velocity" else None,
                        )
                    self.send(
                        CanFrame(RPDO2_ID, payload),
                        f"RPDO2 preload {point}",
                    )
                    self.sync()
                    time.sleep(period_ms / 1000.0)
                self.sdo_download(
                    0x6040,
                    0x00,
                    2,
                    interpolation_start_controlword,
                    "start interpolation trigger",
                )

            stream_feedback: list[dict[str, object]] = []
            next_tick = time.monotonic()
            for stream_index, point in enumerate(points[preload:], start=preload):
                next_tick += period_ms / 1000.0
                if use_2010:
                    payload = self.rpdo2_2010_linear_abs_segment(stream_index, period_ms, point)
                else:
                    payload = self.rpdo2_payload(
                        point,
                        period_ms if rpdo2_mapping_variant == "position_time" else None,
                        point_velocity(stream_index) if rpdo2_mapping_variant == "position_velocity" else None,
                    )
                self.send(
                    CanFrame(RPDO2_ID, payload),
                    f"RPDO2 stream {point}",
                )
                self.sync()
                remaining_s = next_tick - time.monotonic()
                if no_live_feedback:
                    if remaining_s > 0:
                        time.sleep(remaining_s)
                else:
                    remaining_ms = int(max(1.0, remaining_s * 1000.0))
                    stream_feedback.extend(self.collect_frames(remaining_ms))
            if no_live_feedback:
                stream_feedback.extend(self.collect_frames(500))
            self.write_json("rpdo2_stream_feedback.json", stream_feedback)
            time.sleep(0.3)
        finally:
            if original_type is not None and original_type != rpdo2_type:
                self.sdo_download(0x1402, 0x02, 1, original_type, "restore RPDO2 transmission type")
            if rpdo2_mapping_original is not None:
                self.restore_rpdo2_mapping(rpdo2_mapping_original)
        self.read_precheck("rpdo2_after.json")

    @staticmethod
    def make_triangle_points(amplitude: int, samples: int) -> list[int]:
        if samples < 4:
            raise ValueError("samples must be >= 4")
        half = samples // 2
        up = [round(amplitude * i / half) for i in range(half + 1)]
        down = [round(amplitude * (half - i) / half) for i in range(1, half + 1)]
        return [int(v) for v in (up + down)]

    @staticmethod
    def decode_feedback(frame: CanFrame) -> dict[str, int | str]:
        if frame.can_id == TPDO0_ID and len(frame.data) == 8:
            return {
                "actual_position": int.from_bytes(frame.data[0:4], "little", signed=True),
                "actual_velocity": int.from_bytes(frame.data[4:8], "little", signed=True),
            }
        if frame.can_id == TPDO1_ID and len(frame.data) == 8:
            return {
                "latched_fault": f"0x{int.from_bytes(frame.data[0:4], 'little'):08X}",
                "statusword": f"0x{int.from_bytes(frame.data[4:6], 'little'):04X}",
                "actual_current": int.from_bytes(frame.data[6:8], "little", signed=True),
            }
        if frame.can_id == EMCY_ID:
            return {"emcy": frame.data.hex(" ").upper()}
        return {}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Node5 RPDO2 interpolation debug helper.")
    parser.add_argument("--mode", choices=["precheck", "rpdo1-baseline", "rpdo2-short"], default="precheck")
    parser.add_argument("--log-dir", default="")
    parser.add_argument("--timeout-ms", type=int, default=900)
    parser.add_argument("--amplitude", type=int, default=20_000)
    parser.add_argument("--period-ms", type=int, default=20)
    parser.add_argument("--preload", type=int, default=8)
    parser.add_argument("--samples", type=int, default=20)
    parser.add_argument("--rpdo2-type", type=int, choices=[1, 4])
    parser.add_argument("--trigger-before-points", action="store_true")
    parser.add_argument("--no-live-feedback", action="store_true")
    parser.add_argument("--tail-zero-count", type=int, default=0)
    parser.add_argument("--interp-start-controlword", type=lambda value: int(value, 0), default=0x003F)
    parser.add_argument(
        "--rpdo2-mapping-variant",
        choices=["position_only", "position_time", "position_velocity", "pvt2010_linear_abs"],
        default="position_only",
    )
    parser.add_argument("--allow-motion", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    log_dir = Path(args.log_dir) if args.log_dir else REPO_ROOT / "out" / f"node5_rpdo2_debug_{args.mode}_{timestamp}"
    try:
        with Node5Debug(args.timeout_ms, log_dir) as debug:
            if args.mode == "precheck":
                debug.read_precheck("precheck.json")
            elif args.mode == "rpdo1-baseline":
                if not args.allow_motion:
                    raise RuntimeError("--allow-motion is required for rpdo1-baseline")
                debug.run_rpdo1_baseline(args.amplitude)
            elif args.mode == "rpdo2-short":
                if not args.allow_motion:
                    raise RuntimeError("--allow-motion is required for rpdo2-short")
                debug.run_rpdo2_short(
                    args.amplitude,
                    args.period_ms,
                    args.preload,
                    args.samples,
                    args.rpdo2_type,
                    args.trigger_before_points,
                    args.no_live_feedback,
                    args.tail_zero_count,
                    args.interp_start_controlword,
                    args.rpdo2_mapping_variant,
                )
        print(f"Output written to {log_dir}")
        return 0
    except Exception as exc:  # noqa: BLE001 - command-line tool should report hardware failures plainly.
        print(f"FAILED: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
