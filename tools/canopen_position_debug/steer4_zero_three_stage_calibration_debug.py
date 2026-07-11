"""Four-steering-axis zero calibration with the field-verified three-stage method.

This script is the CAN-analyzer reference implementation for the ECU steering
zero-calibration maintenance FSM.  It intentionally uses velocity mode for the
whole search and midpoint-return process:

* profile-position return was tested after limit search and can fail to move
  some axes even when statusword/fault feedback looks normal;
* velocity closed-loop return reaches the midpoint repeatably and keeps the
  logic simple enough to port into the CAN2-owned ECU task.

Default behavior is dry-run.  Pass --allow-motion only when the steering
mechanism is safe to move and the operator is ready to stop power.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
for extra in (REPO_ROOT / "tools" / "canopen_pdo_config", REPO_ROOT / "tools" / "can"):
    if str(extra) not in sys.path:
        sys.path.insert(0, str(extra))

from can_adapter import CanFrame  # noqa: E402
from can_adapter_controlcan import ControlCanAdapter  # noqa: E402


STEER_NODES = (5, 6, 7, 8)
NMT_COB_ID = 0x000
SYNC_COB_ID = 0x080
NMT_START_REMOTE_NODE = 0x01
MODE_PROFILE_VELOCITY = 3
CONTROL_ENABLE_OPERATION = 0x000F
STORE_PARAMETERS_INDEX = 0x1010
STORE_PARAMETERS_SUBINDEX = 0x01
STORE_PARAMETERS_SIGNATURE = 0x65766173


@dataclass
class AxisFeedback:
    position_counts: int | None = None
    velocity_raw: int | None = None
    current_10ma: int | None = None
    statusword: int | None = None
    fault_latched: int | None = None


@dataclass
class DirectionResult:
    start: dict[int, int] = field(default_factory=dict)
    hit: dict[int, int] = field(default_factory=dict)
    retreat: dict[int, int] = field(default_factory=dict)
    peak_current_10ma: dict[int, int] = field(default_factory=dict)
    reason: dict[int, str] = field(default_factory=dict)
    elapsed_s: float = 0.0


class ThreeStageSteerZero:
    def __init__(self, bus: str, timeout_ms: int, log_dir: Path, dry_run: bool) -> None:
        self.bus = bus
        self.timeout_ms = timeout_ms
        self.log_dir = log_dir
        self.dry_run = dry_run
        self.adapter = ControlCanAdapter()
        self.feedback = {node: AxisFeedback() for node in STEER_NODES}
        self.events: list[dict[str, object]] = []

    def __enter__(self) -> "ThreeStageSteerZero":
        self.log_dir.mkdir(parents=True, exist_ok=True)
        if not self.dry_run:
            self.adapter.open([self.bus], 1_000_000)
        return self

    def __exit__(self, exc_type, exc, tb) -> None:  # type: ignore[no-untyped-def]
        self.write_json("events.json", self.events)
        if not self.dry_run:
            self.adapter.close()

    def event(self, kind: str, **fields: object) -> None:
        item = {"time_s": time.time(), "kind": kind}
        item.update(fields)
        self.events.append(item)

    def write_json(self, name: str, data: object) -> Path:
        path = self.log_dir / name
        path.write_text(json.dumps(data, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
                        encoding="utf-8")
        return path

    def send(self, frame: CanFrame, note: str) -> None:
        self.event("tx", note=note, can_id=f"0x{frame.can_id:03X}",
                   data=frame.data.hex(" ").upper())
        if not self.dry_run:
            self.adapter.send(self.bus, frame)

    def receive_expected(self, can_id: int) -> CanFrame:
        frame = self.adapter.receive(self.bus, can_id, self.timeout_ms)
        self.event("rx", can_id=f"0x{frame.can_id:03X}", data=frame.data.hex(" ").upper())
        return frame

    @staticmethod
    def sdo_download_frame(node: int, index: int, subindex: int, size: int,
                           value: int, signed: bool = False) -> CanFrame:
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
        return CanFrame(0x600 + node,
                        bytes([0x40, index & 0xFF, (index >> 8) & 0xFF, subindex, 0, 0, 0, 0]))

    def sdo_download(self, node: int, index: int, subindex: int, size: int,
                     value: int, note: str, signed: bool = False) -> None:
        self.send(self.sdo_download_frame(node, index, subindex, size, value, signed),
                  f"Node{node} SDO write {note}")
        if self.dry_run:
            return
        ack = self.receive_expected(0x580 + node)
        if ack.data[0] == 0x80:
            abort_code = int.from_bytes(ack.data[4:8], "little")
            raise RuntimeError(f"Node{node} SDO abort {note}: 0x{abort_code:08X}")
        expected = bytes([0x60, index & 0xFF, (index >> 8) & 0xFF, subindex, 0, 0, 0, 0])
        if ack.data != expected:
            raise RuntimeError(f"Node{node} unexpected SDO ack {ack.data.hex(' ').upper()}")

    def sdo_upload_i32(self, node: int, index: int, subindex: int, note: str) -> int:
        self.send(self.sdo_upload_frame(node, index, subindex), f"Node{node} SDO read {note}")
        frame = self.receive_expected(0x580 + node)
        if frame.data[0] == 0x80:
            abort_code = int.from_bytes(frame.data[4:8], "little")
            raise RuntimeError(f"Node{node} SDO abort read {note}: 0x{abort_code:08X}")
        return int.from_bytes(frame.data[4:8], "little", signed=True)

    @staticmethod
    def velocity_payload(velocity_raw: int) -> bytes:
        return (CONTROL_ENABLE_OPERATION.to_bytes(2, "little")
                + bytes([MODE_PROFILE_VELOCITY])
                + int(velocity_raw).to_bytes(4, "little", signed=True))

    def send_velocity(self, node: int, velocity_raw: int) -> None:
        self.send(CanFrame(0x200 + node, self.velocity_payload(velocity_raw)),
                  f"Node{node} RPDO0 velocity={velocity_raw}")

    def sync(self) -> None:
        self.send(CanFrame(SYNC_COB_ID, b""), "SYNC")

    def nmt_operational(self, node: int) -> None:
        self.send(CanFrame(NMT_COB_ID, bytes([NMT_START_REMOTE_NODE, node & 0x7F])),
                  f"Node{node} NMT operational")

    def configure_velocity_axis(self, node: int, profile_velocity: int, profile_accel: int) -> None:
        self.nmt_operational(node)
        time.sleep(0.02)
        self.sdo_download(node, 0x2300, 0x00, 2, 0x001E, "control source CANopen")
        self.sdo_download(node, 0x6060, 0x00, 1, MODE_PROFILE_VELOCITY, "profile velocity mode")
        self.sdo_download(node, 0x6081, 0x00, 4, profile_velocity, "profile velocity")
        self.sdo_download(node, 0x6083, 0x00, 4, profile_accel, "profile acceleration")
        self.sdo_download(node, 0x6084, 0x00, 4, profile_accel, "profile deceleration")
        self.sdo_download(node, 0x6040, 0x00, 2, CONTROL_ENABLE_OPERATION, "enable operation")

    def decode_feedback(self, frame: CanFrame) -> None:
        if 0x180 <= frame.can_id <= 0x18F and len(frame.data) == 8:
            node = frame.can_id - 0x180
            if node in self.feedback:
                self.feedback[node].position_counts = int.from_bytes(frame.data[0:4], "little", signed=True)
                self.feedback[node].velocity_raw = int.from_bytes(frame.data[4:8], "little", signed=True)
        elif 0x280 <= frame.can_id <= 0x28F and len(frame.data) == 8:
            node = frame.can_id - 0x280
            if node in self.feedback:
                self.feedback[node].fault_latched = int.from_bytes(frame.data[0:4], "little")
                self.feedback[node].statusword = int.from_bytes(frame.data[4:6], "little")
                self.feedback[node].current_10ma = int.from_bytes(frame.data[6:8], "little", signed=True)
        elif 0x080 <= frame.can_id <= 0x08F:
            self.event("emcy", node=frame.can_id - 0x080, data=frame.data.hex(" ").upper())

    def drain_feedback(self) -> None:
        if self.dry_run:
            return
        wanted = {0x180 + n for n in STEER_NODES} | {0x280 + n for n in STEER_NODES} | {
            0x080 + n for n in STEER_NODES
        }
        cached = self.adapter._rx_cache[self.bus]  # type: ignore[attr-defined]
        kept: list[CanFrame] = []
        for frame in cached:
            if frame.can_id in wanted:
                self.decode_feedback(frame)
                self.event("rx_observe", can_id=f"0x{frame.can_id:03X}", data=frame.data.hex(" ").upper())
            else:
                kept.append(frame)
        cached[:] = kept
        channel = self.adapter._channels[self.bus]  # type: ignore[attr-defined]
        for raw in self.adapter._device.receive(channel, limit=200, wait_ms=0):  # type: ignore[attr-defined]
            data = bytes(int(raw.Data[i]) for i in range(int(raw.DataLen)))
            frame = CanFrame(int(raw.ID), data, is_extended=bool(raw.ExternFlag), is_remote=bool(raw.RemoteFlag))
            if frame.can_id in wanted:
                self.decode_feedback(frame)
                self.event("rx_observe", can_id=f"0x{frame.can_id:03X}", data=frame.data.hex(" ").upper())
            else:
                cached.append(frame)

    def request_feedback(self) -> None:
        self.sync()
        time.sleep(0.002)
        self.drain_feedback()

    def stop_all(self) -> None:
        for node in STEER_NODES:
            self.send_velocity(node, 0)
        self.sync()
        self.event("safe_stop", nodes=list(STEER_NODES))

    def latest_position(self, node: int) -> int:
        self.request_feedback()
        position = self.feedback[node].position_counts
        if position is not None:
            return position
        return 0 if self.dry_run else self.sdo_upload_i32(node, 0x6064, 0, "actual position fallback")

    def find_direction(self, name: str, sign: int, args: argparse.Namespace) -> DirectionResult:
        result = DirectionResult()
        result.start = {node: self.latest_position(node) for node in STEER_NODES}
        result.peak_current_10ma = {node: 0 for node in STEER_NODES}
        dwell_start: dict[int, float | None] = {node: None for node in STEER_NODES}
        stage = {node: 0 for node in STEER_NODES}
        done: set[int] = set()
        start_s = time.monotonic()

        for node in STEER_NODES:
            self.send_velocity(node, sign * args.fast_velocity)
        self.sync()

        try:
            while len(done) < len(STEER_NODES):
                now_s = time.monotonic()
                if now_s - start_s > args.direction_timeout_s:
                    raise TimeoutError(f"{name} limit timeout; done={sorted(done)}")
                self.request_feedback()
                for node in STEER_NODES:
                    if node in done:
                        continue
                    fb = self.feedback[node]
                    if fb.position_counts is None:
                        continue
                    current_abs = abs(fb.current_10ma or 0)
                    velocity_abs = abs(fb.velocity_raw or 0)
                    result.peak_current_10ma[node] = max(result.peak_current_10ma[node], current_abs)
                    abs_pos = abs(fb.position_counts)
                    if stage[node] < 1 and abs_pos >= args.slowdown1_abs_counts:
                        self.send_velocity(node, sign * args.medium_velocity)
                        self.sync()
                        stage[node] = 1
                    if stage[node] < 2 and abs_pos >= args.slowdown2_abs_counts:
                        self.send_velocity(node, sign * args.slow_velocity)
                        self.sync()
                        stage[node] = 2

                    if (now_s - start_s) * 1000.0 < args.arm_delay_ms:
                        continue
                    travel = abs(fb.position_counts - result.start[node])
                    hit = False
                    reason = ""
                    if current_abs >= args.immediate_current_10ma:
                        hit = True
                        reason = f"immediate_{args.immediate_current_10ma}"
                    elif travel >= args.min_travel_counts and velocity_abs <= args.zero_velocity_window:
                        if dwell_start[node] is None:
                            dwell_start[node] = now_s
                        if (now_s - dwell_start[node]) * 1000.0 >= args.zero_velocity_dwell_ms:
                            hit = True
                            reason = "travel_zero_velocity"
                    else:
                        dwell_start[node] = None

                    if hit:
                        result.hit[node] = fb.position_counts
                        result.reason[node] = reason
                        done.add(node)
                        self.send_velocity(node, 0)
                        self.sync()
                time.sleep(0.01)
        finally:
            self.stop_all()

        # Retain no axis at a hard stop before reversing direction or returning
        # to midpoint.  This is what made the profile-position return unreliable.
        retreat_sign = -sign
        for node in STEER_NODES:
            self.send_velocity(node, retreat_sign * args.fast_velocity)
        self.sync()
        deadline = time.monotonic() + args.retreat_timeout_s
        while time.monotonic() < deadline:
            self.request_feedback()
            result.retreat = {node: self.feedback[node].position_counts or 0 for node in STEER_NODES}
            if name == "left":
                ready = all(result.retreat[node] <= args.inner_safe_abs_counts for node in STEER_NODES)
            else:
                ready = all(result.retreat[node] >= -args.inner_safe_abs_counts for node in STEER_NODES)
            if ready:
                break
            time.sleep(0.05)
        self.stop_all()
        result.elapsed_s = time.monotonic() - start_s
        return result

    def return_to_midpoint(self, midpoints: dict[int, int], args: argparse.Namespace) -> tuple[bool, dict[int, int]]:
        stable = {node: 0 for node in STEER_NODES}
        done: set[int] = set()
        deadline = time.monotonic() + args.midpoint_timeout_s
        while time.monotonic() < deadline and len(done) < len(STEER_NODES):
            self.request_feedback()
            for node in STEER_NODES:
                if node in done:
                    continue
                position = self.feedback[node].position_counts
                if position is None:
                    continue
                error = midpoints[node] - position
                if abs(error) <= args.midpoint_tolerance_counts:
                    self.send_velocity(node, 0)
                    stable[node] += 1
                    if stable[node] >= args.midpoint_stable_samples:
                        done.add(node)
                else:
                    stable[node] = 0
                    if abs(error) > args.mid_return_medium_error_counts:
                        velocity = args.mid_return_fast_velocity
                    elif abs(error) > args.mid_return_slow_error_counts:
                        velocity = args.mid_return_medium_velocity
                    else:
                        velocity = args.mid_return_slow_velocity
                    self.send_velocity(node, velocity if error > 0 else -velocity)
            self.sync()
            time.sleep(0.04)
        self.stop_all()
        self.request_feedback()
        final = {node: self.feedback[node].position_counts or 0 for node in STEER_NODES}
        return len(done) == len(STEER_NODES), final

    def set_current_position_zero(self, save: bool) -> None:
        for node in STEER_NODES:
            self.sdo_download(node, 0x6064, 0x00, 4, 0, "set current midpoint position as zero", signed=True)
        if save:
            for node in STEER_NODES:
                self.sdo_download(node, STORE_PARAMETERS_INDEX, STORE_PARAMETERS_SUBINDEX, 4,
                                  STORE_PARAMETERS_SIGNATURE, "store parameters")

    def run(self, args: argparse.Namespace) -> dict[str, object]:
        if self.dry_run:
            summary = {"dry_run": True, "nodes": list(STEER_NODES), "method": "three_stage_velocity_zero"}
            self.write_json("summary.json", summary)
            return summary

        for node in STEER_NODES:
            try:
                self.sdo_download(node, 0x6040, 0, 2, 0x0080, "fault reset before calibration")
            except Exception as exc:  # noqa: BLE001 - log and continue to configure.
                self.event("fault_reset_warning", node=node, error=str(exc))
            self.configure_velocity_axis(node, args.profile_velocity, args.profile_accel)

        left = self.find_direction("left", args.left_sign, args)
        time.sleep(0.5)
        right = self.find_direction("right", -args.left_sign, args)
        midpoints = {
            node: int(round((left.hit[node] + right.hit[node]) / 2.0))
            for node in STEER_NODES
        }
        midpoint_ok, final = self.return_to_midpoint(midpoints, args)
        errors = {node: final[node] - midpoints[node] for node in STEER_NODES}
        zero_written = False
        if midpoint_ok and args.set_current_position_zero:
            self.set_current_position_zero(args.save_drive_parameters)
            zero_written = True

        summary: dict[str, object] = {
            "dry_run": False,
            "method": "three_stage_velocity_zero",
            "left": left.__dict__,
            "right": right.__dict__,
            "midpoint_counts_by_node": midpoints,
            "final_position_counts_by_node": final,
            "midpoint_error_counts_by_node": errors,
            "midpoint_ok": midpoint_ok,
            "set_current_position_zero_done": zero_written,
            "drive_parameters_saved_by_0x1010_01": zero_written and args.save_drive_parameters,
            "parameters": vars(args),
        }
        self.write_json("summary.json", summary)
        return summary


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Three-stage steering zero calibration.")
    parser.add_argument("--bus", choices=["can1", "can2"], default="can1")
    parser.add_argument("--allow-motion", action="store_true")
    parser.add_argument("--left-sign", type=int, choices=[-1, 1], default=1)
    parser.add_argument("--fast-velocity", type=int, default=800_000)
    parser.add_argument("--medium-velocity", type=int, default=250_000)
    parser.add_argument("--slow-velocity", type=int, default=60_000)
    parser.add_argument("--slowdown1-abs-counts", type=int, default=1_200_000)
    parser.add_argument("--slowdown2-abs-counts", type=int, default=1_650_000)
    parser.add_argument("--immediate-current-10ma", type=int, default=900)
    parser.add_argument("--zero-velocity-window", type=int, default=30_000)
    parser.add_argument("--zero-velocity-dwell-ms", type=int, default=300)
    parser.add_argument("--min-travel-counts", type=int, default=1_500_000)
    parser.add_argument("--inner-safe-abs-counts", type=int, default=300_000)
    parser.add_argument("--mid-return-fast-velocity", type=int, default=500_000)
    parser.add_argument("--mid-return-medium-velocity", type=int, default=240_000)
    parser.add_argument("--mid-return-slow-velocity", type=int, default=30_000)
    parser.add_argument("--mid-return-medium-error-counts", type=int, default=300_000)
    parser.add_argument("--mid-return-slow-error-counts", type=int, default=30_000)
    parser.add_argument("--midpoint-tolerance-counts", type=int, default=2_000)
    parser.add_argument("--midpoint-stable-samples", type=int, default=5)
    parser.add_argument("--profile-velocity", type=int, default=800_000)
    parser.add_argument("--profile-accel", type=int, default=500_000)
    parser.add_argument("--arm-delay-ms", type=int, default=800)
    parser.add_argument("--direction-timeout-s", type=float, default=120.0)
    parser.add_argument("--retreat-timeout-s", type=float, default=25.0)
    parser.add_argument("--midpoint-timeout-s", type=float, default=40.0)
    parser.add_argument("--timeout-ms", type=int, default=900)
    parser.add_argument("--set-current-position-zero", action="store_true")
    parser.add_argument("--save-drive-parameters", action="store_true")
    parser.add_argument("--log-dir", default="")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    log_dir = Path(args.log_dir) if args.log_dir else REPO_ROOT / "out" / (
        "steer_zero_three_stage_" + time.strftime("%Y%m%d_%H%M%S")
    )
    try:
        with ThreeStageSteerZero(args.bus, args.timeout_ms, log_dir, dry_run=not args.allow_motion) as tool:
            summary = tool.run(args)
        print(json.dumps({"log_dir": str(log_dir), **summary}, indent=2, ensure_ascii=False))
        if not args.allow_motion:
            print("DRY-RUN ONLY: pass --allow-motion to move steering axes.", file=sys.stderr)
        return 0
    except Exception as exc:  # noqa: BLE001 - hardware CLI reports failures plainly.
        print(f"FAILED: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
