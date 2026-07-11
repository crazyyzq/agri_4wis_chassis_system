"""Four-steering-axis zero calibration helper using the CAN analyzer.

This bench tool validates the zero-search algorithm that ECU firmware can later
run after an operator gesture such as three short B1 presses.  It finds steering
center from measured mechanical limits:

1. Put Node5..8 into CANopen velocity mode.
2. Move toward the left limit until actual current is above the immediate
   protection current once, or above the dwell threshold for the configured
   dwell time.
3. Stop that axis and record actual position from TPDO0.
4. Move toward the right limit with the same current dwell rule.
5. Stop that axis and record actual position.
6. Move all axes to the midpoint between their two measured limits.
7. Optionally write 0 to 0x6064 at the midpoint.  The field verified behavior
   is that this clears the actual position count at the current mechanical
   position, so the midpoint becomes count zero.
8. Save a calibration JSON that ECU firmware can persist/import.

Safety defaults:
  * dry-run by default; --allow-motion is required for real CAN traffic;
  * no NMT reset and no drive reset;
  * no drive Flash save unless --save-drive-parameters is explicitly passed;
  * no position-zero SDO write unless --set-current-position-zero and the
    vendor verified object are explicitly passed;
  * steering nodes only: Node5, Node6, Node7, Node8.

Expected current unit:
  TPDO1 byte6..7 maps vendor object 0x221C actual current, signed int16,
  unit = 10 mA.  Therefore 7.5 A == 750 and 9 A == 900.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
PDO_TOOL_DIR = REPO_ROOT / "tools" / "canopen_pdo_config"
CAN_TOOL_DIR = REPO_ROOT / "tools" / "can"
for path in (PDO_TOOL_DIR, CAN_TOOL_DIR):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from can_adapter import CanFrame  # noqa: E402
from can_adapter_controlcan import ControlCanAdapter  # noqa: E402


STEER_NODES = (5, 6, 7, 8)
NMT_COB_ID = 0x000
SYNC_COB_ID = 0x080
NMT_COMMAND_START_REMOTE_NODE = 0x01

MODE_PROFILE_POSITION = 1
MODE_PROFILE_VELOCITY = 3

CONTROLWORD_ENABLE_OPERATION = 0x000F
CONTROLWORD_POSITION_ARM = 0x000F
CONTROLWORD_POSITION_TRIGGER = 0x001F

CANOPEN_STORE_PARAMETERS_OBJECT = 0x1010
CANOPEN_STORE_APPLICATION_SUBINDEX = 0x01
CANOPEN_STORE_SAVE_SIGNATURE = 0x65766173  # ASCII "save" little-endian


@dataclass
class AxisFeedback:
    position_counts: int | None = None
    velocity_raw: int | None = None
    current_10ma: int | None = None
    statusword: int | None = None
    latched_fault: int | None = None
    last_position_time_s: float = 0.0
    last_current_time_s: float = 0.0


@dataclass
class LimitResult:
    direction_name: str
    direction_sign: int
    position_by_node: dict[int, int] = field(default_factory=dict)
    peak_current_10ma_by_node: dict[int, int] = field(default_factory=dict)
    elapsed_s: float = 0.0


class SteerZeroCalibration:
    def __init__(self, bus: str, timeout_ms: int, log_dir: Path, dry_run: bool) -> None:
        self.bus = bus
        self.timeout_ms = timeout_ms
        self.log_dir = log_dir
        self.dry_run = dry_run
        self.adapter = ControlCanAdapter()
        self.events: list[dict[str, object]] = []
        self.feedback: dict[int, AxisFeedback] = {node: AxisFeedback() for node in STEER_NODES}

    def __enter__(self) -> "SteerZeroCalibration":
        self.log_dir.mkdir(parents=True, exist_ok=True)
        if not self.dry_run:
            self.adapter.open([self.bus], 1_000_000)
        return self

    def __exit__(self, exc_type, exc, tb) -> None:  # type: ignore[no-untyped-def]
        self.write_json("events.json", self.events)
        if not self.dry_run:
            self.adapter.close()

    def event(self, kind: str, **fields: object) -> None:
        record = {"time_s": time.time(), "kind": kind}
        record.update(fields)
        self.events.append(record)

    def write_json(self, name: str, data: object) -> Path:
        path = self.log_dir / name
        path.write_text(
            json.dumps(data, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        return path

    def send(self, frame: CanFrame, note: str) -> None:
        self.event("tx", note=note, can_id=f"0x{frame.can_id:03X}", data=frame.data.hex(" ").upper())
        if not self.dry_run:
            self.adapter.send(self.bus, frame)

    def receive_expected(self, can_id: int, timeout_ms: int | None = None) -> CanFrame:
        if self.dry_run:
            raise RuntimeError("dry-run cannot receive CAN frames")
        frame = self.adapter.receive(self.bus, can_id, timeout_ms or self.timeout_ms)
        self.event("rx", can_id=f"0x{frame.can_id:03X}", data=frame.data.hex(" ").upper())
        return frame

    @staticmethod
    def sdo_download_frame(node: int, index: int, subindex: int, size: int, value: int,
                           signed: bool = False) -> CanFrame:
        command = {1: 0x2F, 2: 0x2B, 4: 0x23}[size]
        payload = bytearray(8)
        payload[0] = command
        payload[1] = index & 0xFF
        payload[2] = (index >> 8) & 0xFF
        payload[3] = subindex & 0xFF
        payload[4:4 + size] = int(value).to_bytes(size, "little", signed=signed)
        return CanFrame(0x600 + node, bytes(payload))

    @staticmethod
    def sdo_upload_frame(node: int, index: int, subindex: int) -> CanFrame:
        return CanFrame(
            0x600 + node,
            bytes([0x40, index & 0xFF, (index >> 8) & 0xFF, subindex, 0, 0, 0, 0]),
        )

    def sdo_download(self, node: int, index: int, subindex: int, size: int, value: int,
                     note: str, signed: bool = False) -> None:
        self.send(self.sdo_download_frame(node, index, subindex, size, value, signed),
                  f"Node{node} SDO write {note}")
        if self.dry_run:
            return
        ack = self.receive_expected(0x580 + node)
        if ack.data[0] == 0x80:
            abort_code = int.from_bytes(ack.data[4:8], "little")
            raise RuntimeError(
                f"Node{node} SDO abort write {note} 0x{index:04X}:{subindex} abort=0x{abort_code:08X}"
            )
        expected = bytes([0x60, index & 0xFF, (index >> 8) & 0xFF, subindex, 0, 0, 0, 0])
        if ack.data != expected:
            raise RuntimeError(f"Node{node} unexpected SDO ack {ack.data.hex(' ').upper()}")

    def sdo_upload_i32(self, node: int, index: int, subindex: int, note: str) -> int:
        self.send(self.sdo_upload_frame(node, index, subindex), f"Node{node} SDO read {note}")
        frame = self.receive_expected(0x580 + node)
        if frame.data[0] == 0x80:
            abort_code = int.from_bytes(frame.data[4:8], "little")
            raise RuntimeError(
                f"Node{node} SDO abort read {note} 0x{index:04X}:{subindex} abort=0x{abort_code:08X}"
            )
        return int.from_bytes(frame.data[4:8], "little", signed=True)

    @staticmethod
    def velocity_rpdo0_payload(controlword: int, velocity_raw: int) -> bytes:
        return (
            int(controlword).to_bytes(2, "little")
            + bytes([MODE_PROFILE_VELOCITY])
            + int(velocity_raw).to_bytes(4, "little", signed=True)
        )

    @staticmethod
    def position_rpdo1_payload(controlword: int, target_counts: int) -> bytes:
        return (
            int(controlword).to_bytes(2, "little")
            + bytes([MODE_PROFILE_POSITION])
            + int(target_counts).to_bytes(4, "little", signed=True)
        )

    def send_velocity(self, node: int, velocity_raw: int) -> None:
        self.send(
            CanFrame(0x200 + node, self.velocity_rpdo0_payload(CONTROLWORD_ENABLE_OPERATION, velocity_raw)),
            f"Node{node} RPDO0 velocity={velocity_raw}",
        )

    def send_position(self, node: int, controlword: int, target_counts: int) -> None:
        self.send(
            CanFrame(0x300 + node, self.position_rpdo1_payload(controlword, target_counts)),
            f"Node{node} RPDO1 cw=0x{controlword:04X} target={target_counts}",
        )

    def sync(self) -> None:
        self.send(CanFrame(SYNC_COB_ID, b""), "SYNC")

    def nmt_operational(self, node: int) -> None:
        self.send(
            CanFrame(NMT_COB_ID, bytes([NMT_COMMAND_START_REMOTE_NODE, node & 0x7F])),
            f"Node{node} NMT operational",
        )

    def configure_axis_for_velocity_limit_search(self, node: int, profile_velocity: int,
                                                 profile_accel: int) -> None:
        self.nmt_operational(node)
        time.sleep(0.02)
        self.sdo_download(node, 0x2300, 0x00, 2, 0x001E, "control source CANopen")
        self.sdo_download(node, 0x6060, 0x00, 1, MODE_PROFILE_VELOCITY, "profile velocity mode")
        self.sdo_download(node, 0x6081, 0x00, 4, profile_velocity, "profile velocity")
        self.sdo_download(node, 0x6083, 0x00, 4, profile_accel, "profile acceleration")
        self.sdo_download(node, 0x6084, 0x00, 4, profile_accel, "profile deceleration")
        self.sdo_download(node, 0x2113, 0x00, 4, 1000, "current ramp 1000 mA/s")
        self.sdo_download(node, 0x6040, 0x00, 2, CONTROLWORD_ENABLE_OPERATION, "enable operation")

    def configure_axis_for_position_return(self, node: int, profile_velocity: int,
                                           profile_accel: int) -> None:
        self.nmt_operational(node)
        time.sleep(0.02)
        self.sdo_download(node, 0x2300, 0x00, 2, 0x001E, "control source CANopen")
        self.sdo_download(node, 0x6060, 0x00, 1, MODE_PROFILE_POSITION, "profile position mode")
        self.sdo_download(node, 0x6081, 0x00, 4, profile_velocity, "profile velocity")
        self.sdo_download(node, 0x6083, 0x00, 4, profile_accel, "profile acceleration")
        self.sdo_download(node, 0x6084, 0x00, 4, profile_accel, "profile deceleration")
        self.sdo_download(node, 0x6040, 0x00, 2, CONTROLWORD_ENABLE_OPERATION, "enable operation")

    def clear_fault_and_reenable_velocity(self, node: int, profile_velocity: int,
                                          profile_accel: int) -> None:
        self.sdo_download(node, 0x6040, 0x00, 2, 0x0080, "fault reset after stall detection")
        time.sleep(0.05)
        self.sdo_download(node, 0x6060, 0x00, 1, MODE_PROFILE_VELOCITY, "profile velocity mode")
        self.sdo_download(node, 0x6081, 0x00, 4, profile_velocity, "profile velocity")
        self.sdo_download(node, 0x6083, 0x00, 4, profile_accel, "profile acceleration")
        self.sdo_download(node, 0x6084, 0x00, 4, profile_accel, "profile deceleration")
        self.sdo_download(node, 0x6040, 0x00, 2, 0x0006, "shutdown before re-enable")
        self.sdo_download(node, 0x6040, 0x00, 2, CONTROLWORD_ENABLE_OPERATION, "re-enable operation")

    def decode_feedback_frame(self, frame: CanFrame) -> None:
        now_s = time.monotonic()
        if 0x180 <= frame.can_id <= 0x18F and len(frame.data) == 8:
            node = frame.can_id - 0x180
            if node in self.feedback:
                item = self.feedback[node]
                item.position_counts = int.from_bytes(frame.data[0:4], "little", signed=True)
                item.velocity_raw = int.from_bytes(frame.data[4:8], "little", signed=True)
                item.last_position_time_s = now_s
        elif 0x280 <= frame.can_id <= 0x28F and len(frame.data) == 8:
            node = frame.can_id - 0x280
            if node in self.feedback:
                item = self.feedback[node]
                item.latched_fault = int.from_bytes(frame.data[0:4], "little")
                item.statusword = int.from_bytes(frame.data[4:6], "little")
                item.current_10ma = int.from_bytes(frame.data[6:8], "little", signed=True)
                item.last_current_time_s = now_s
        elif 0x080 <= frame.can_id <= 0x08F:
            node = frame.can_id - 0x080
            self.event("emcy", node=node, data=frame.data.hex(" ").upper())

    def drain_feedback_nonblocking(self) -> None:
        if self.dry_run:
            return
        wanted = set()
        for node in STEER_NODES:
            wanted.update((0x180 + node, 0x280 + node, 0x080 + node))

        cached = self.adapter._rx_cache[self.bus]  # type: ignore[attr-defined]
        kept: list[CanFrame] = []
        for frame in cached:
            if frame.can_id in wanted:
                self.decode_feedback_frame(frame)
                self.event("rx_observe", can_id=f"0x{frame.can_id:03X}", data=frame.data.hex(" ").upper())
            else:
                kept.append(frame)
        cached[:] = kept

        channel = self.adapter._channels[self.bus]  # type: ignore[attr-defined]
        for raw in self.adapter._device.receive(channel, limit=200, wait_ms=0):  # type: ignore[attr-defined]
            data = bytes(int(raw.Data[i]) for i in range(int(raw.DataLen)))
            frame = CanFrame(int(raw.ID), data, is_extended=bool(raw.ExternFlag), is_remote=bool(raw.RemoteFlag))
            if frame.can_id in wanted:
                self.decode_feedback_frame(frame)
                self.event("rx_observe", can_id=f"0x{frame.can_id:03X}", data=frame.data.hex(" ").upper())
            else:
                cached.append(frame)

    def request_sync_feedback(self) -> None:
        self.sync()
        time.sleep(0.002)
        self.drain_feedback_nonblocking()

    def stop_all_axes(self) -> None:
        for node in STEER_NODES:
            self.send_velocity(node, 0)
        self.sync()
        self.event("safe_stop", nodes=list(STEER_NODES))

    def latest_position_or_sdo(self, node: int) -> int:
        self.request_sync_feedback()
        position = self.feedback[node].position_counts
        if position is not None:
            return position
        if self.dry_run:
            return 0
        return self.sdo_upload_i32(node, 0x6064, 0x00, "actual position fallback")

    def find_limit(self, direction_name: str, direction_sign: int, velocity_raw_abs: int,
                   stall_current_10ma: int, stall_immediate_current_10ma: int,
                   stall_dwell_ms: int, stall_arm_delay_ms: int,
                   max_travel_counts: int, timeout_s: float, profile_velocity: int,
                   profile_accel: int) -> LimitResult:
        direction_velocity = int(direction_sign * abs(velocity_raw_abs))
        start_positions = {node: self.latest_position_or_sdo(node) for node in STEER_NODES}
        start_s = time.monotonic()
        dwell_start: dict[int, float | None] = {node: None for node in STEER_NODES}
        done: set[int] = set()
        result = LimitResult(direction_name=direction_name, direction_sign=direction_sign)
        peak_abs_current: dict[int, int] = {node: 0 for node in STEER_NODES}

        self.event("limit_start", direction=direction_name, velocity=direction_velocity,
                   start_positions=start_positions)
        for node in STEER_NODES:
            self.send_velocity(node, direction_velocity)
        self.sync()

        try:
            while len(done) < len(STEER_NODES):
                now_s = time.monotonic()
                if now_s - start_s > timeout_s:
                    raise TimeoutError(f"{direction_name} limit search timeout; done={sorted(done)}")

                self.request_sync_feedback()
                for node in STEER_NODES:
                    fb = self.feedback[node]
                    if fb.latched_fault not in (None, 0):
                        if peak_abs_current[node] >= stall_current_10ma:
                            limit_position = fb.position_counts
                            if limit_position is None:
                                limit_position = self.sdo_upload_i32(node, 0x6064, 0x00, "fault limit position")
                            result.position_by_node[node] = limit_position
                            result.peak_current_10ma_by_node[node] = peak_abs_current[node]
                            done.add(node)
                            self.send_velocity(node, 0)
                            self.clear_fault_and_reenable_velocity(node, profile_velocity, profile_accel)
                            self.event("limit_axis_done", direction=direction_name, node=node,
                                       position=limit_position, peak_current_10ma=peak_abs_current[node],
                                       fault_at_limit=f"0x{fb.latched_fault:08X}")
                            continue
                        raise RuntimeError(f"Node{node} fault during {direction_name}: 0x{fb.latched_fault:08X}")

                    if fb.position_counts is not None:
                        travel = abs(fb.position_counts - start_positions[node])
                        if travel > max_travel_counts:
                            raise RuntimeError(
                                f"Node{node} exceeded max travel {travel} counts during {direction_name}"
                            )

                    current_abs = abs(fb.current_10ma or 0)
                    peak_abs_current[node] = max(peak_abs_current[node], current_abs)
                    if node in done:
                        continue

                    elapsed_ms = (now_s - start_s) * 1000.0
                    if elapsed_ms < stall_arm_delay_ms:
                        dwell_start[node] = None
                        continue

                    if current_abs >= stall_immediate_current_10ma:
                        limit_position = fb.position_counts
                        if limit_position is None:
                            limit_position = self.sdo_upload_i32(node, 0x6064, 0x00, "instant stall position")
                        result.position_by_node[node] = limit_position
                        result.peak_current_10ma_by_node[node] = peak_abs_current[node]
                        done.add(node)
                        self.send_velocity(node, 0)
                        self.clear_fault_and_reenable_velocity(node, profile_velocity, profile_accel)
                        self.event("limit_axis_done", direction=direction_name, node=node,
                                   position=limit_position, peak_current_10ma=peak_abs_current[node],
                                   instant_over_protection_current=True,
                                   immediate_current_10ma=stall_immediate_current_10ma)
                        continue

                    if current_abs >= stall_current_10ma:
                        if dwell_start[node] is None:
                            dwell_start[node] = now_s
                        dwell_ms = (now_s - dwell_start[node]) * 1000.0
                        if dwell_ms >= stall_dwell_ms:
                            limit_position = fb.position_counts
                            if limit_position is None:
                                limit_position = self.sdo_upload_i32(node, 0x6064, 0x00, "limit position")
                            result.position_by_node[node] = limit_position
                            result.peak_current_10ma_by_node[node] = peak_abs_current[node]
                            done.add(node)
                            self.send_velocity(node, 0)
                            self.clear_fault_and_reenable_velocity(node, profile_velocity, profile_accel)
                            self.event("limit_axis_done", direction=direction_name, node=node,
                                       position=limit_position, peak_current_10ma=peak_abs_current[node])
                    else:
                        dwell_start[node] = None
                time.sleep(0.01)
        finally:
            self.stop_all_axes()

        result.elapsed_s = time.monotonic() - start_s
        return result

    def move_to_midpoints(self, midpoints: dict[int, int], profile_velocity: int,
                          profile_accel: int, settle_timeout_s: float) -> None:
        for node in STEER_NODES:
            self.configure_axis_for_position_return(node, profile_velocity, profile_accel)
        time.sleep(0.05)

        for node in STEER_NODES:
            self.send_position(node, CONTROLWORD_POSITION_ARM, midpoints[node])
        self.sync()
        for node in STEER_NODES:
            self.send_position(node, CONTROLWORD_POSITION_TRIGGER, midpoints[node])
        self.sync()

        deadline = time.monotonic() + settle_timeout_s
        while time.monotonic() < deadline:
            self.request_sync_feedback()
            all_close = True
            errors = {}
            for node in STEER_NODES:
                pos = self.feedback[node].position_counts
                if pos is None:
                    all_close = False
                    continue
                error = pos - midpoints[node]
                errors[node] = error
                if abs(error) > 20_000:
                    all_close = False
            if all_close:
                self.event("midpoint_reached", errors=errors)
                return
            time.sleep(0.05)
        raise TimeoutError("timeout moving steering axes to calculated midpoint")

    def set_current_position_zero(self, index: int, subindex: int, size: int, value: int,
                                  signed: bool) -> None:
        for node in STEER_NODES:
            self.sdo_download(
                node,
                index,
                subindex,
                size,
                value,
                f"set current midpoint position as zero via 0x{index:04X}:{subindex:02X}",
                signed=signed,
            )

    def save_drive_parameters(self) -> None:
        for node in STEER_NODES:
            self.sdo_download(
                node,
                CANOPEN_STORE_PARAMETERS_OBJECT,
                CANOPEN_STORE_APPLICATION_SUBINDEX,
                4,
                CANOPEN_STORE_SAVE_SIGNATURE,
                "store application parameters 0x1010:01",
            )

    def run(self, left_sign: int, velocity_raw_abs: int, profile_velocity: int,
            profile_accel: int, stall_current_10ma: int, stall_immediate_current_10ma: int,
            stall_dwell_ms: int,
            stall_arm_delay_ms: int, max_travel_counts: int, timeout_s: float, settle_timeout_s: float,
            set_current_position_zero: bool, zero_object_index: int | None,
            zero_object_subindex: int, zero_object_size: int, zero_object_value: int,
            zero_object_signed: bool, save_drive_parameters: bool) -> dict[str, object]:
        if save_drive_parameters and not set_current_position_zero:
            raise ValueError("--save-drive-parameters requires --set-current-position-zero")
        if set_current_position_zero and zero_object_index is None:
            raise ValueError("--set-current-position-zero requires --zero-object-index")

        if self.dry_run:
            summary = {
                "dry_run": True,
                "nodes": list(STEER_NODES),
                "left_sign": left_sign,
                "right_sign": -left_sign,
                "stall_current_10ma": stall_current_10ma,
                "stall_immediate_current_10ma": stall_immediate_current_10ma,
                "stall_dwell_ms": stall_dwell_ms,
                "stall_arm_delay_ms": stall_arm_delay_ms,
                "set_current_position_zero": set_current_position_zero,
                "zero_object_index": f"0x{zero_object_index:04X}" if zero_object_index is not None else None,
                "save_drive_parameters": save_drive_parameters,
                "note": "Use --allow-motion to run the real calibration sequence.",
            }
            self.write_json("summary.json", summary)
            return summary

        for node in STEER_NODES:
            self.configure_axis_for_velocity_limit_search(node, profile_velocity, profile_accel)

        left = self.find_limit("left", left_sign, velocity_raw_abs, stall_current_10ma,
                               stall_immediate_current_10ma,
                               stall_dwell_ms, stall_arm_delay_ms, max_travel_counts, timeout_s,
                               profile_velocity, profile_accel)
        time.sleep(0.50)
        right = self.find_limit("right", -left_sign, velocity_raw_abs, stall_current_10ma,
                                stall_immediate_current_10ma,
                                stall_dwell_ms, stall_arm_delay_ms, max_travel_counts, timeout_s,
                                profile_velocity, profile_accel)

        midpoints: dict[int, int] = {}
        spans: dict[int, int] = {}
        for node in STEER_NODES:
            left_pos = left.position_by_node[node]
            right_pos = right.position_by_node[node]
            midpoints[node] = int(round((left_pos + right_pos) / 2.0))
            spans[node] = abs(left_pos - right_pos)

        self.move_to_midpoints(midpoints, profile_velocity, profile_accel, settle_timeout_s)
        midpoint_reached_positions = {node: self.latest_position_or_sdo(node) for node in STEER_NODES}
        zero_write_done = False
        position_after_zero_write: dict[int, int] = {}
        if set_current_position_zero:
            assert zero_object_index is not None
            self.set_current_position_zero(
                zero_object_index,
                zero_object_subindex,
                zero_object_size,
                zero_object_value,
                zero_object_signed,
            )
            zero_write_done = True
            position_after_zero_write = {node: self.latest_position_or_sdo(node) for node in STEER_NODES}
        if save_drive_parameters:
            self.save_drive_parameters()
        final_positions = {node: self.latest_position_or_sdo(node) for node in STEER_NODES}

        calibration = {
            "schema": "agri_4wis_steering_zero_calibration_v3",
            "created_unix_s": int(time.time()),
            "nodes": list(STEER_NODES),
            "method": "stall_current_limits_move_to_midpoint_then_write_6064_zero",
            "midpoint_counts_by_node": midpoints,
            "span_counts_by_node": spans,
            "left_limit_counts_by_node": left.position_by_node,
            "right_limit_counts_by_node": right.position_by_node,
            "midpoint_reached_position_counts_by_node": midpoint_reached_positions,
            "set_current_position_zero_done": zero_write_done,
            "position_counts_after_zero_write_by_node": position_after_zero_write,
            "zero_position_object": {
                "index": f"0x{zero_object_index:04X}" if zero_object_index is not None else None,
                "subindex": zero_object_subindex,
                "size": zero_object_size,
                "value": zero_object_value,
                "signed": zero_object_signed,
            } if zero_write_done else None,
            "stall_current_10ma": stall_current_10ma,
            "stall_immediate_current_10ma": stall_immediate_current_10ma,
            "stall_dwell_ms": stall_dwell_ms,
            "stall_arm_delay_ms": stall_arm_delay_ms,
        }
        self.write_json("steer_zero_calibration.json", calibration)

        summary = {
            "dry_run": False,
            "nodes": list(STEER_NODES),
            "left": left.__dict__,
            "right": right.__dict__,
            "midpoint_counts_by_node": midpoints,
            "span_counts_by_node": spans,
            "midpoint_reached_position_counts_by_node": midpoint_reached_positions,
            "set_current_position_zero_done": zero_write_done,
            "position_counts_after_zero_write_by_node": position_after_zero_write,
            "drive_parameters_saved_by_0x1010_01": save_drive_parameters,
            "final_position_counts_by_node": final_positions,
            "calibration_json": str(self.log_dir / "steer_zero_calibration.json"),
            "ecu_guidance": (
                "Preferred production flow: find both mechanical limits, move to the midpoint, write 0 to "
                "0x6064 so the current midpoint position becomes count zero, then verify 0x6064 readback. "
                "Do not rely on 0x607C home offset for this steering system."
            ),
        }
        self.write_json("summary.json", summary)
        return summary


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Four steering zero calibration by stall-current limit search.")
    parser.add_argument("--bus", choices=["can1", "can2"], default="can1")
    parser.add_argument("--allow-motion", action="store_true")
    parser.add_argument("--left-sign", type=int, choices=[-1, 1], default=1)
    parser.add_argument("--velocity-raw", type=int, default=300_000)
    parser.add_argument("--profile-velocity", type=int, default=800_000)
    parser.add_argument("--profile-accel", type=int, default=500_000)
    parser.add_argument("--stall-current-10ma", type=int, default=750)
    parser.add_argument("--stall-immediate-current-10ma", type=int, default=900,
                        help="immediate stop/protection threshold, unit=10mA; 900 means 9A")
    parser.add_argument("--stall-dwell-ms", type=int, default=100)
    parser.add_argument("--stall-arm-delay-ms", type=int, default=400,
                        help="ignore high current immediately after changing search direction")
    parser.add_argument("--max-travel-counts", type=int, default=1_700_000)
    parser.add_argument("--timeout-s", type=float, default=20.0)
    parser.add_argument("--settle-timeout-s", type=float, default=12.0)
    parser.add_argument("--timeout-ms", type=int, default=900)
    parser.add_argument("--set-current-position-zero", action="store_true",
                        help="after reaching the midpoint, write a vendor verified SDO that defines it as 0")
    parser.add_argument("--zero-object-index", type=lambda text: int(text, 0), default=0x6064,
                        help="vendor verified object index for setting current position as zero, e.g. 0xXXXX")
    parser.add_argument("--zero-object-subindex", type=lambda text: int(text, 0), default=0)
    parser.add_argument("--zero-object-size", type=int, choices=[1, 2, 4], default=4)
    parser.add_argument("--zero-object-value", type=lambda text: int(text, 0), default=0)
    parser.add_argument("--zero-object-signed", action="store_true")
    parser.add_argument("--save-drive-parameters", action="store_true",
                        help="send 0x1010:01='save' after the explicit current-position-zero write")
    parser.add_argument("--log-dir", default="")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    log_dir = Path(args.log_dir) if args.log_dir else REPO_ROOT / "out" / f"steer4_zero_calibration_{timestamp}"
    try:
        with SteerZeroCalibration(args.bus, args.timeout_ms, log_dir, dry_run=not args.allow_motion) as tool:
            summary = tool.run(
                args.left_sign,
                args.velocity_raw,
                args.profile_velocity,
                args.profile_accel,
                args.stall_current_10ma,
                args.stall_immediate_current_10ma,
                args.stall_dwell_ms,
                args.stall_arm_delay_ms,
                args.max_travel_counts,
                args.timeout_s,
                args.settle_timeout_s,
                args.set_current_position_zero,
                args.zero_object_index,
                args.zero_object_subindex,
                args.zero_object_size,
                args.zero_object_value,
                args.zero_object_signed,
                args.save_drive_parameters,
            )
        print(json.dumps({"log_dir": str(log_dir), **summary}, indent=2, ensure_ascii=False))
        if not args.allow_motion:
            print("DRY-RUN ONLY: pass --allow-motion to move steering axes.", file=sys.stderr)
        return 0
    except Exception as exc:  # noqa: BLE001 - hardware CLI should report failures plainly.
        print(f"FAILED: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
