"""Steering zero calibration helper using the CAN analyzer.

This tool is the bench reference for the ECU B1-triple-click steering zero
workflow.  It intentionally uses velocity-mode PDO control for both the limit
search and the return-to-center movement:

* limit search is two-stage velocity control: fast, then 70% fast speed near
  the expected end-stop region;
* return-to-center is three-stage velocity control: fast, medium, and a slow
  final approach only when the axis is already close to target;
* each phase clears faults, re-selects velocity mode, and re-enables operation;
* each axis stops independently once it has found a limit or reached center;
* no drive reset and no NMT reset are used, so the steering count reference is
  not lost unexpectedly;
* writing 0 to 0x6064 is optional and explicit through
  --set-current-position-zero.

Safety defaults:
* dry-run by default; --allow-motion is required for real CAN traffic;
* TPDO1 byte6..7 is expected to expose vendor object 0x221C actual current,
  signed int16, unit = 10 mA.

The installed steering mechanism is mirrored left/right.  Field observation
showed that the physical-left limit direction is:

    Node5 +, Node6 -, Node7 +, Node8 -

Override this with --left-signs if wiring changes.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
for tool_dir in (
    REPO_ROOT / "tools" / "canopen_pdo_config",
    REPO_ROOT / "tools" / "can",
):
    if str(tool_dir) not in sys.path:
        sys.path.insert(0, str(tool_dir))

from can_adapter import CanFrame  # noqa: E402
from can_adapter_controlcan import ControlCanAdapter  # noqa: E402


STEER_NODES = (5, 6, 7, 8)
DEFAULT_LEFT_SIGNS = {5: 1, 6: -1, 7: 1, 8: -1}

NMT_COB_ID = 0x000
SYNC_COB_ID = 0x080
NMT_COMMAND_START_REMOTE_NODE = 0x01

MODE_PROFILE_VELOCITY = 3

CONTROLWORD_FAULT_RESET = 0x0080
CONTROLWORD_SHUTDOWN = 0x0006
CONTROLWORD_SWITCH_ON = 0x0007
CONTROLWORD_ENABLE_OPERATION = 0x000F

CANOPEN_STORE_PARAMETERS_OBJECT = 0x1010
CANOPEN_STORE_APPLICATION_SUBINDEX = 0x01
CANOPEN_STORE_SAVE_SIGNATURE = 0x65766173  # ASCII "save", little-endian.


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
    velocity_sign_by_node: dict[int, int]
    position_by_node: dict[int, int] = field(default_factory=dict)
    peak_current_10ma_by_node: dict[int, int] = field(default_factory=dict)
    elapsed_s: float = 0.0


@dataclass
class AxisReturnState:
    last_error_counts: int | None = None
    stable_samples: int = 0


class SteerZeroCalibration:
    """CAN-analyzer implementation of the steering zero calibration sequence."""

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
        self.event(
            "tx",
            note=note,
            can_id=f"0x{frame.can_id:03X}",
            data=frame.data.hex(" ").upper(),
        )
        if not self.dry_run:
            self.adapter.send(self.bus, frame)

    def receive_expected(self, can_id: int, timeout_ms: int | None = None) -> CanFrame:
        if self.dry_run:
            raise RuntimeError("dry-run cannot receive CAN frames")
        frame = self.adapter.receive(self.bus, can_id, timeout_ms or self.timeout_ms)
        self.event("rx", can_id=f"0x{frame.can_id:03X}", data=frame.data.hex(" ").upper())
        return frame

    @staticmethod
    def sdo_download_frame(
        node: int,
        index: int,
        subindex: int,
        size: int,
        value: int,
        signed: bool = False,
    ) -> CanFrame:
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

    def sdo_download(
        self,
        node: int,
        index: int,
        subindex: int,
        size: int,
        value: int,
        note: str,
        signed: bool = False,
    ) -> None:
        self.send(
            self.sdo_download_frame(node, index, subindex, size, value, signed),
            f"Node{node} SDO write {note}",
        )
        if self.dry_run:
            return

        ack = self.receive_expected(0x580 + node)
        if ack.data[0] == 0x80:
            abort_code = int.from_bytes(ack.data[4:8], "little")
            raise RuntimeError(
                f"Node{node} SDO abort write {note} "
                f"0x{index:04X}:{subindex} abort=0x{abort_code:08X}"
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
                f"Node{node} SDO abort read {note} "
                f"0x{index:04X}:{subindex} abort=0x{abort_code:08X}"
            )
        return int.from_bytes(frame.data[4:8], "little", signed=True)

    @staticmethod
    def velocity_rpdo0_payload(controlword: int, velocity_raw: int) -> bytes:
        return (
            int(controlword).to_bytes(2, "little")
            + bytes([MODE_PROFILE_VELOCITY])
            + int(velocity_raw).to_bytes(4, "little", signed=True)
        )

    def send_velocity(self, node: int, velocity_raw: int) -> None:
        self.send(
            CanFrame(0x200 + node, self.velocity_rpdo0_payload(CONTROLWORD_ENABLE_OPERATION, velocity_raw)),
            f"Node{node} RPDO0 velocity={velocity_raw}",
        )

    def send_velocity_group(self, velocity_by_node: dict[int, int], note: str) -> None:
        for node in STEER_NODES:
            self.send_velocity(node, velocity_by_node.get(node, 0))
        self.sync(note=note)

    def sync(self, note: str = "SYNC") -> None:
        self.send(CanFrame(SYNC_COB_ID, b""), note)

    def nmt_operational(self, node: int) -> None:
        self.send(
            CanFrame(NMT_COB_ID, bytes([NMT_COMMAND_START_REMOTE_NODE, node & 0x7F])),
            f"Node{node} NMT operational",
        )

    def configure_axis_velocity_mode(self, node: int, profile_velocity: int, profile_accel: int) -> None:
        """Clear faults and enter profile-velocity mode with an explicit CiA-402 sequence."""
        self.nmt_operational(node)
        time.sleep(0.02)
        self.sdo_download(node, 0x6040, 0x00, 2, CONTROLWORD_FAULT_RESET, "fault reset")
        time.sleep(0.02)
        self.sdo_download(node, 0x2300, 0x00, 2, 0x001E, "control source CANopen")
        self.sdo_download(node, 0x6060, 0x00, 1, MODE_PROFILE_VELOCITY, "profile velocity mode")
        self.sdo_download(node, 0x6081, 0x00, 4, profile_velocity, "profile velocity")
        self.sdo_download(node, 0x6083, 0x00, 4, profile_accel, "profile acceleration")
        self.sdo_download(node, 0x6084, 0x00, 4, profile_accel, "profile deceleration")
        self.sdo_download(node, 0x2113, 0x00, 4, 1000, "current ramp 1000 mA/s")
        self.sdo_download(node, 0x6040, 0x00, 2, CONTROLWORD_SHUTDOWN, "shutdown")
        self.sdo_download(node, 0x6040, 0x00, 2, CONTROLWORD_SWITCH_ON, "switch on")
        self.sdo_download(node, 0x6040, 0x00, 2, CONTROLWORD_ENABLE_OPERATION, "enable operation")

    def configure_all_velocity_mode(self, profile_velocity: int, profile_accel: int, note: str) -> None:
        self.event("configure_velocity_phase", note=note)
        for node in STEER_NODES:
            self.configure_axis_velocity_mode(node, profile_velocity, profile_accel)
        self.send_velocity_group({node: 0 for node in STEER_NODES}, f"{note}: zero velocity after configure")
        time.sleep(0.05)

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
        for raw in self.adapter._device.receive(channel, limit=300, wait_ms=0):  # type: ignore[attr-defined]
            data = bytes(int(raw.Data[i]) for i in range(int(raw.DataLen)))
            frame = CanFrame(int(raw.ID), data, is_extended=bool(raw.ExternFlag), is_remote=bool(raw.RemoteFlag))
            if frame.can_id in wanted:
                self.decode_feedback_frame(frame)
                self.event("rx_observe", can_id=f"0x{frame.can_id:03X}", data=frame.data.hex(" ").upper())
            else:
                cached.append(frame)

    def request_sync_feedback(self) -> None:
        self.sync("feedback SYNC")
        time.sleep(0.002)
        self.drain_feedback_nonblocking()

    def latest_position_or_sdo(self, node: int) -> int:
        self.request_sync_feedback()
        position = self.feedback[node].position_counts
        if position is not None:
            return position
        if self.dry_run:
            return 0
        return self.sdo_upload_i32(node, 0x6064, 0x00, "actual position fallback")

    def stop_all_axes(self, note: str) -> None:
        self.send_velocity_group({node: 0 for node in STEER_NODES}, note)
        self.event("safe_stop", nodes=list(STEER_NODES), note=note)

    @staticmethod
    def abs_current_10ma(feedback: AxisFeedback) -> int:
        current = feedback.current_10ma or 0
        return -current if current < 0 else current

    @staticmethod
    def signed_velocity_toward(error_counts: int, speed_abs: int) -> int:
        if error_counts == 0:
            return 0
        return speed_abs if error_counts > 0 else -speed_abs

    def find_limit(
        self,
        direction_name: str,
        sign_by_node: dict[int, int],
        limit_fast_velocity: int,
        limit_slow_velocity: int,
        limit_stage2_start_counts: int,
        stall_current_10ma: int,
        stall_immediate_current_10ma: int,
        stall_dwell_ms: int,
        stall_arm_delay_ms: int,
        max_travel_counts: int,
        timeout_s: float,
    ) -> LimitResult:
        start_positions = {node: self.latest_position_or_sdo(node) for node in STEER_NODES}
        result = LimitResult(direction_name=direction_name, velocity_sign_by_node=dict(sign_by_node))
        peak_current = {node: 0 for node in STEER_NODES}
        dwell_start_s: dict[int, float | None] = {node: None for node in STEER_NODES}
        done: set[int] = set()
        start_s = time.monotonic()
        next_tx_s = start_s

        self.event(
            "limit_start",
            direction=direction_name,
            sign_by_node=sign_by_node,
            start_positions=start_positions,
        )

        while len(done) < len(STEER_NODES):
            now_s = time.monotonic()
            if now_s - start_s > timeout_s:
                self.stop_all_axes(f"{direction_name}: timeout stop")
                raise TimeoutError(f"{direction_name} limit search timeout; done={sorted(done)}")

            self.request_sync_feedback()
            velocity_by_node: dict[int, int] = {}
            for node in STEER_NODES:
                if node in done:
                    velocity_by_node[node] = 0
                    continue

                fb = self.feedback[node]
                if fb.position_counts is None:
                    velocity_by_node[node] = 0
                    continue

                travel = abs(fb.position_counts - start_positions[node])
                if travel > max_travel_counts:
                    self.stop_all_axes(f"{direction_name}: max travel stop")
                    raise RuntimeError(
                        f"Node{node} exceeded max travel {travel} counts during {direction_name}"
                    )

                current_abs = self.abs_current_10ma(fb)
                peak_current[node] = max(peak_current[node], current_abs)

                if (time.monotonic() - start_s) * 1000.0 < stall_arm_delay_ms:
                    dwell_start_s[node] = None
                elif current_abs >= stall_immediate_current_10ma:
                    result.position_by_node[node] = fb.position_counts
                    result.peak_current_10ma_by_node[node] = peak_current[node]
                    done.add(node)
                    velocity_by_node[node] = 0
                    self.event(
                        "limit_axis_done",
                        direction=direction_name,
                        node=node,
                        position=fb.position_counts,
                        travel=travel,
                        peak_current_10ma=peak_current[node],
                        immediate=True,
                        fault=f"0x{fb.latched_fault:08X}" if fb.latched_fault else "0x00000000",
                    )
                    continue
                elif current_abs >= stall_current_10ma:
                    if dwell_start_s[node] is None:
                        dwell_start_s[node] = now_s
                    if (now_s - dwell_start_s[node]) * 1000.0 >= stall_dwell_ms:
                        result.position_by_node[node] = fb.position_counts
                        result.peak_current_10ma_by_node[node] = peak_current[node]
                        done.add(node)
                        velocity_by_node[node] = 0
                        self.event(
                            "limit_axis_done",
                            direction=direction_name,
                            node=node,
                            position=fb.position_counts,
                            travel=travel,
                            peak_current_10ma=peak_current[node],
                            dwell_ms=stall_dwell_ms,
                            fault=f"0x{fb.latched_fault:08X}" if fb.latched_fault else "0x00000000",
                        )
                        continue
                else:
                    dwell_start_s[node] = None

                speed = limit_slow_velocity if travel >= limit_stage2_start_counts else limit_fast_velocity
                velocity_by_node[node] = sign_by_node[node] * speed

            if time.monotonic() >= next_tx_s:
                self.send_velocity_group(velocity_by_node, f"{direction_name}: velocity group")
                next_tx_s = time.monotonic() + 0.05

            time.sleep(0.01)

        self.stop_all_axes(f"{direction_name}: all axes done")
        result.elapsed_s = time.monotonic() - start_s
        return result

    def return_to_midpoints(
        self,
        midpoints: dict[int, int],
        return_fast_velocity: int,
        return_medium_velocity: int,
        return_slow_velocity: int,
        return_medium_error_counts: int,
        return_slow_error_counts: int,
        tolerance_counts: int,
        stable_samples: int,
        timeout_s: float,
        timeout_action: str,
    ) -> dict[int, int]:
        axis_state = {node: AxisReturnState() for node in STEER_NODES}
        reached_positions: dict[int, int] = {}
        done: set[int] = set()
        start_s = time.monotonic()
        next_tx_s = start_s

        self.event("return_start", midpoints=midpoints)
        while len(done) < len(STEER_NODES):
            now_s = time.monotonic()
            if now_s - start_s > timeout_s:
                self.stop_all_axes("return timeout stop")
                if timeout_action != "write-current-zero":
                    raise TimeoutError(f"timeout returning to midpoint; done={sorted(done)}")
                for node in STEER_NODES:
                    if node in done:
                        continue
                    pos = self.feedback[node].position_counts
                    if pos is None:
                        pos = self.latest_position_or_sdo(node)
                    reached_positions[node] = pos
                    done.add(node)
                    self.event("return_timeout_accept_current_position", node=node, position=pos)
                break

            self.request_sync_feedback()
            velocity_by_node: dict[int, int] = {}
            for node in STEER_NODES:
                if node in done:
                    velocity_by_node[node] = 0
                    continue

                pos = self.feedback[node].position_counts
                if pos is None:
                    velocity_by_node[node] = 0
                    continue

                error = midpoints[node] - pos
                state = axis_state[node]
                crossed_target = (
                    state.last_error_counts is not None and
                    ((state.last_error_counts > 0 > error) or (state.last_error_counts < 0 < error))
                )
                if abs(error) <= tolerance_counts or crossed_target:
                    state.stable_samples += 1
                else:
                    state.stable_samples = 0

                if state.stable_samples >= stable_samples:
                    reached_positions[node] = pos
                    done.add(node)
                    velocity_by_node[node] = 0
                    self.event("return_axis_done", node=node, position=pos, error=error)
                    continue

                abs_error = abs(error)
                if abs_error <= return_slow_error_counts:
                    speed = return_slow_velocity
                elif abs_error <= return_medium_error_counts:
                    speed = return_medium_velocity
                else:
                    speed = return_fast_velocity
                velocity_by_node[node] = self.signed_velocity_toward(error, speed)
                state.last_error_counts = error

            if time.monotonic() >= next_tx_s:
                self.send_velocity_group(velocity_by_node, "return: velocity group")
                next_tx_s = time.monotonic() + 0.02
            time.sleep(0.005)

        self.stop_all_axes("return complete stop")
        self.event("return_complete", reached_positions=reached_positions)
        return reached_positions

    def set_current_position_zero(
        self,
        index: int,
        subindex: int,
        size: int,
        value: int,
        signed: bool,
    ) -> dict[int, int]:
        for node in STEER_NODES:
            self.sdo_download(
                node,
                index,
                subindex,
                size,
                value,
                f"set current mechanical zero via 0x{index:04X}:{subindex:02X}",
                signed=signed,
            )
        time.sleep(0.05)
        return {node: self.latest_position_or_sdo(node) for node in STEER_NODES}

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

    def run(self, args: argparse.Namespace) -> dict[str, object]:
        left_sign_by_node = parse_left_signs(args.left_signs)
        right_sign_by_node = {node: -sign for node, sign in left_sign_by_node.items()}
        limit_slow_velocity = args.limit_slow_velocity
        if limit_slow_velocity is None:
            limit_slow_velocity = int(round(args.limit_fast_velocity * 0.70))

        if args.save_drive_parameters and not args.set_current_position_zero:
            raise ValueError("--save-drive-parameters requires --set-current-position-zero")

        dry_summary = {
            "dry_run": self.dry_run,
            "nodes": list(STEER_NODES),
            "left_sign_by_node": left_sign_by_node,
            "right_sign_by_node": right_sign_by_node,
            "limit_fast_velocity": args.limit_fast_velocity,
            "limit_slow_velocity": limit_slow_velocity,
            "return_fast_velocity": args.return_fast_velocity,
            "return_medium_velocity": args.return_medium_velocity,
            "return_slow_velocity": args.return_slow_velocity,
            "set_current_position_zero": args.set_current_position_zero,
            "zero_object_index": f"0x{args.zero_object_index:04X}",
            "save_drive_parameters": args.save_drive_parameters,
        }
        if self.dry_run:
            dry_summary["note"] = "Use --allow-motion to run the real calibration sequence."
            self.write_json("summary.json", dry_summary)
            return dry_summary

        self.configure_all_velocity_mode(args.profile_velocity, args.profile_accel, "before left search")
        left = self.find_limit(
            "left",
            left_sign_by_node,
            args.limit_fast_velocity,
            limit_slow_velocity,
            args.limit_stage2_start_counts,
            args.stall_current_10ma,
            args.stall_immediate_current_10ma,
            args.stall_dwell_ms,
            args.stall_arm_delay_ms,
            args.max_travel_counts,
            args.limit_timeout_s,
        )

        self.configure_all_velocity_mode(args.profile_velocity, args.profile_accel, "before right search")
        right = self.find_limit(
            "right",
            right_sign_by_node,
            args.limit_fast_velocity,
            limit_slow_velocity,
            args.limit_stage2_start_counts,
            args.stall_current_10ma,
            args.stall_immediate_current_10ma,
            args.stall_dwell_ms,
            args.stall_arm_delay_ms,
            args.max_travel_counts,
            args.limit_timeout_s,
        )

        midpoints: dict[int, int] = {}
        spans: dict[int, int] = {}
        for node in STEER_NODES:
            left_pos = left.position_by_node[node]
            right_pos = right.position_by_node[node]
            midpoints[node] = int(round((left_pos + right_pos) / 2.0))
            spans[node] = abs(left_pos - right_pos)

        self.configure_all_velocity_mode(args.profile_velocity, args.profile_accel, "before midpoint return")
        midpoint_reached_positions = self.return_to_midpoints(
            midpoints,
            args.return_fast_velocity,
            args.return_medium_velocity,
            args.return_slow_velocity,
            args.return_medium_error_counts,
            args.return_slow_error_counts,
            args.midpoint_tolerance_counts,
            args.midpoint_stable_samples,
            args.return_timeout_s,
            args.return_timeout_action,
        )

        zero_write_done = False
        position_after_zero_write: dict[int, int] = {}
        if args.set_current_position_zero:
            position_after_zero_write = self.set_current_position_zero(
                args.zero_object_index,
                args.zero_object_subindex,
                args.zero_object_size,
                args.zero_object_value,
                args.zero_object_signed,
            )
            zero_write_done = True
        if args.save_drive_parameters:
            self.save_drive_parameters()

        final_positions = {node: self.latest_position_or_sdo(node) for node in STEER_NODES}
        calibration = {
            "schema": "agri_4wis_steering_zero_calibration_v4",
            "created_unix_s": int(time.time()),
            "method": "two_stage_limit_velocity_three_stage_midpoint_velocity_then_6064_zero",
            "nodes": list(STEER_NODES),
            "left_limit_counts_by_node": left.position_by_node,
            "right_limit_counts_by_node": right.position_by_node,
            "midpoint_counts_by_node": midpoints,
            "span_counts_by_node": spans,
            "midpoint_reached_position_counts_by_node": midpoint_reached_positions,
            "set_current_position_zero_done": zero_write_done,
            "position_counts_after_zero_write_by_node": position_after_zero_write,
            "final_position_counts_by_node": final_positions,
            "left_sign_by_node": left_sign_by_node,
            "right_sign_by_node": right_sign_by_node,
            "limit_fast_velocity": args.limit_fast_velocity,
            "limit_slow_velocity": limit_slow_velocity,
            "return_fast_velocity": args.return_fast_velocity,
            "return_medium_velocity": args.return_medium_velocity,
            "return_slow_velocity": args.return_slow_velocity,
            "stall_current_10ma": args.stall_current_10ma,
            "stall_immediate_current_10ma": args.stall_immediate_current_10ma,
            "stall_dwell_ms": args.stall_dwell_ms,
            "return_timeout_action": args.return_timeout_action,
        }
        self.write_json("steer_zero_calibration.json", calibration)

        summary = {
            **dry_summary,
            "left": left.__dict__,
            "right": right.__dict__,
            "midpoint_counts_by_node": midpoints,
            "span_counts_by_node": spans,
            "midpoint_reached_position_counts_by_node": midpoint_reached_positions,
            "set_current_position_zero_done": zero_write_done,
            "position_counts_after_zero_write_by_node": position_after_zero_write,
            "drive_parameters_saved_by_0x1010_01": args.save_drive_parameters,
            "final_position_counts_by_node": final_positions,
            "calibration_json": str(self.log_dir / "steer_zero_calibration.json"),
            "ecu_guidance": (
                "ECU should own CAN2 during zero calibration, use per-axis mirrored "
                "limit-search signs, clear faults before each phase, return with "
                "three-stage velocity control, then write 0 to 0x6064 only after "
                "the accepted zero position is reached or intentionally accepted."
            ),
        }
        self.write_json("summary.json", summary)
        return summary


def parse_left_signs(text: str) -> dict[int, int]:
    signs = dict(DEFAULT_LEFT_SIGNS)
    if not text.strip():
        return signs
    for item in text.split(","):
        if not item.strip():
            continue
        try:
            node_text, sign_text = item.split(":", 1)
            node = int(node_text.strip(), 0)
            sign = int(sign_text.strip(), 0)
        except ValueError as exc:
            raise ValueError(f"invalid --left-signs item '{item}', expected node:sign") from exc
        if node not in STEER_NODES:
            raise ValueError(f"invalid --left-signs node {node}, expected one of {STEER_NODES}")
        if sign not in (-1, 1):
            raise ValueError(f"invalid --left-signs sign {sign} for Node{node}, expected -1 or 1")
        signs[node] = sign
    return signs


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Four steering zero calibration by stall-current limit search.")
    parser.add_argument("--bus", choices=["can1", "can2"], default="can1")
    parser.add_argument("--allow-motion", action="store_true")
    parser.add_argument(
        "--left-signs",
        default="",
        help="per-node physical-left velocity sign, default: 5:1,6:-1,7:1,8:-1",
    )
    parser.add_argument("--profile-velocity", type=int, default=2_000_000)
    parser.add_argument("--profile-accel", type=int, default=1_000_000)
    parser.add_argument("--limit-fast-velocity", type=int, default=1_600_000)
    parser.add_argument(
        "--limit-slow-velocity",
        type=int,
        default=None,
        help="second limit-search speed; default is 70%% of --limit-fast-velocity",
    )
    parser.add_argument("--limit-stage2-start-counts", type=int, default=2_400_000)
    parser.add_argument("--stall-current-10ma", type=int, default=750)
    parser.add_argument(
        "--stall-immediate-current-10ma",
        type=int,
        default=900,
        help="immediate stop/protection threshold, unit=10mA; 900 means 9A",
    )
    parser.add_argument("--stall-dwell-ms", type=int, default=150)
    parser.add_argument("--stall-arm-delay-ms", type=int, default=400)
    parser.add_argument("--max-travel-counts", type=int, default=4_000_000)
    parser.add_argument("--limit-timeout-s", type=float, default=60.0)
    parser.add_argument("--return-fast-velocity", type=int, default=1_600_000)
    parser.add_argument("--return-medium-velocity", type=int, default=700_000)
    parser.add_argument("--return-slow-velocity", type=int, default=180_000)
    parser.add_argument("--return-medium-error-counts", type=int, default=500_000)
    parser.add_argument("--return-slow-error-counts", type=int, default=50_000)
    parser.add_argument("--midpoint-tolerance-counts", type=int, default=10_000)
    parser.add_argument("--midpoint-stable-samples", type=int, default=2)
    parser.add_argument("--return-timeout-s", type=float, default=35.0)
    parser.add_argument(
        "--return-timeout-action",
        choices=["fail", "write-current-zero"],
        default="fail",
        help=(
            "fail keeps calibration strict; write-current-zero accepts current "
            "position if return-to-midpoint stalls, matching field emergency workflow"
        ),
    )
    parser.add_argument("--timeout-ms", type=int, default=900)
    parser.add_argument(
        "--set-current-position-zero",
        action="store_true",
        help="after reaching/accepting the zero position, write a verified SDO that defines it as 0",
    )
    parser.add_argument("--zero-object-index", type=lambda text: int(text, 0), default=0x6064)
    parser.add_argument("--zero-object-subindex", type=lambda text: int(text, 0), default=0)
    parser.add_argument("--zero-object-size", type=int, choices=[1, 2, 4], default=4)
    parser.add_argument("--zero-object-value", type=lambda text: int(text, 0), default=0)
    parser.add_argument("--zero-object-signed", action="store_true")
    parser.add_argument(
        "--save-drive-parameters",
        action="store_true",
        help="send 0x1010:01='save' after the explicit current-position-zero write",
    )
    parser.add_argument("--log-dir", default="")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    log_dir = Path(args.log_dir) if args.log_dir else REPO_ROOT / "out" / f"steer4_zero_calibration_{timestamp}"
    try:
        with SteerZeroCalibration(args.bus, args.timeout_ms, log_dir, dry_run=not args.allow_motion) as tool:
            summary = tool.run(args)
        print(json.dumps({"log_dir": str(log_dir), **summary}, indent=2, ensure_ascii=False))
        if not args.allow_motion:
            print("DRY-RUN ONLY: pass --allow-motion to move steering axes.", file=sys.stderr)
        return 0
    except Exception as exc:  # noqa: BLE001 - hardware CLI should report failures plainly.
        print(f"FAILED: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
