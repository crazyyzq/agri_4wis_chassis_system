"""CANopen RPDO whole-chassis suspended-wheel motion debug helper.

This tool drives the CAN analyzer directly, not the ECU firmware.  It validates
the same PDO contract that the ECU uses for remote control:

* Node1..4 drive wheels: RPDO0 = 0x6040 + 0x6060(mode=3) + 0x60FF velocity
* Node5..8 steering axes: RPDO1 = 0x6040 + 0x6060(mode=1) + 0x607A position
* RPDOs are synchronous type 1, so each coherent four-axis group is followed by
  one SYNC.  Steering uses two groups: arm(0x002F) then trigger(0x003F).

Safety defaults assume the vehicle is lifted with all wheels free.  The script
requires --allow-motion, starts with small speeds/angles, stops all drive
targets at the end, and logs every command/feedback frame under out/.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
PDO_TOOL_DIR = REPO_ROOT / "tools" / "canopen_pdo_config"
CAN_TOOL_DIR = REPO_ROOT / "tools" / "can"
for path in (PDO_TOOL_DIR, CAN_TOOL_DIR):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from can_adapter import CanFrame  # noqa: E402
from can_adapter_controlcan import ControlCanAdapter  # noqa: E402


BUS = "can1"  # Analyzer CAN1 is connected to ECU-CAN2 / motion CANopen bus.
SYNC_COB_ID = 0x080
NMT_COB_ID = 0x000

DRIVE_NODES = (1, 2, 3, 4)
STEER_NODES = (5, 6, 7, 8)
ALL_NODES = DRIVE_NODES + STEER_NODES

CONTROL_DISABLE_VOLTAGE = 0x0000
CONTROL_ENABLE_OPERATION = 0x000F
CONTROL_ABSOLUTE_UPDATE_ARM = 0x002F
CONTROL_ABSOLUTE_UPDATE_TRIGGER = 0x003F

MODE_PROFILE_POSITION = 1
MODE_PROFILE_VELOCITY = 3

WHEELBASE_MM = 2200.0
TRACK_WIDTH_MM = 1800.0
MIN_TURN_RADIUS_MM = 2200.0
STEER_DEG_TO_COUNTS = 11111.111
DRIVE_SPEED_KPH_TO_UNITS = (200.0 * (10000.0 / 0.1 / 60.0)) / 6.0

WHEEL_FR = 0
WHEEL_FL = 1
WHEEL_RL = 2
WHEEL_RR = 3


@dataclass(frozen=True)
class WheelCommand:
    steer_deg: tuple[float, float, float, float]
    speed_kph: tuple[float, float, float, float]


class Motion8Debug:
    def __init__(self, timeout_ms: int, log_dir: Path) -> None:
        self.timeout_ms = timeout_ms
        self.log_dir = log_dir
        self.adapter = ControlCanAdapter()
        self.events: list[dict[str, object]] = []
        self.feedback: list[dict[str, object]] = []
        self.command_trace: list[dict[str, object]] = []
        self.timing_trace: list[dict[str, object]] = []

    def __enter__(self) -> "Motion8Debug":
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.adapter.open([BUS], 1_000_000)
        return self

    def __exit__(self, exc_type, exc, tb) -> None:  # type: ignore[no-untyped-def]
        self.write_json("events.json", self.events)
        self.write_json("feedback.json", self.feedback)
        self.write_json("command_trace.json", self.command_trace)
        self.write_json("timing_trace.json", self.timing_trace)
        self.adapter.close()

    def write_json(self, name: str, data: object) -> None:
        (self.log_dir / name).write_text(
            json.dumps(data, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
            encoding="utf-8",
        )

    def event(self, kind: str, **fields: object) -> None:
        record = {"time_s": time.time(), "kind": kind}
        record.update(fields)
        self.events.append(record)

    def send(self, frame: CanFrame, note: str) -> None:
        self.adapter.send(BUS, frame)
        self.event("tx", note=note, can_id=f"0x{frame.can_id:03X}", data=frame.data.hex(" ").upper())

    def receive_expected(self, can_id: int, timeout_ms: int | None = None) -> CanFrame:
        frame = self.adapter.receive(BUS, can_id, timeout_ms or self.timeout_ms)
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
        return CanFrame(0x600 + node, bytes([0x40, index & 0xFF, (index >> 8) & 0xFF, subindex, 0, 0, 0, 0]))

    def sdo_download(self, node: int, index: int, subindex: int, size: int, value: int,
                     note: str, signed: bool = False) -> None:
        self.send(self.sdo_download_frame(node, index, subindex, size, value, signed), f"Node{node} SDO write {note}")
        ack = self.receive_expected(0x580 + node)
        if ack.data[0] == 0x80:
            abort_code = int.from_bytes(ack.data[4:8], "little")
            raise RuntimeError(f"Node{node} SDO abort {note} 0x{index:04X}:{subindex} abort=0x{abort_code:08X}")
        expected = bytes([0x60, index & 0xFF, (index >> 8) & 0xFF, subindex, 0, 0, 0, 0])
        if ack.data != expected:
            raise RuntimeError(f"Node{node} unexpected SDO ack {ack.data.hex(' ').upper()} for {note}")

    def sdo_upload(self, node: int, index: int, subindex: int, size_hint: int, note: str) -> dict[str, object]:
        self.send(self.sdo_upload_frame(node, index, subindex), f"Node{node} SDO read {note}")
        frame = self.receive_expected(0x580 + node)
        if frame.data[0] == 0x80:
            abort_code = int.from_bytes(frame.data[4:8], "little")
            return {"status": "abort", "abort": f"0x{abort_code:08X}", "note": note}
        size_by_command = {0x4F: 1, 0x4B: 2, 0x43: 4}
        size = size_by_command.get(frame.data[0], size_hint)
        raw = frame.data[4:4 + size]
        return {
            "status": "ok",
            "note": note,
            "size": size,
            "value": int.from_bytes(raw, "little", signed=False),
            "signed_value": int.from_bytes(raw, "little", signed=True),
            "hex": f"0x{int.from_bytes(raw, 'little', signed=False):0{size * 2}X}",
        }

    def nmt_operational(self, node: int) -> None:
        self.send(CanFrame(NMT_COB_ID, bytes([0x01, node & 0x7F])), f"Node{node} NMT operational")

    def sync(self) -> None:
        self.send(CanFrame(SYNC_COB_ID, b""), "SYNC")

    @staticmethod
    def rpdo0_velocity_payload(controlword: int, velocity_units: int) -> bytes:
        return (
            int(controlword).to_bytes(2, "little")
            + bytes([MODE_PROFILE_VELOCITY])
            + int(velocity_units).to_bytes(4, "little", signed=True)
        )

    @staticmethod
    def rpdo1_position_payload(controlword: int, target_counts: int) -> bytes:
        return (
            int(controlword).to_bytes(2, "little")
            + bytes([MODE_PROFILE_POSITION])
            + int(target_counts).to_bytes(4, "little", signed=True)
        )

    def send_drive_group(self, velocities: tuple[int, int, int, int], enable: bool) -> None:
        controlword = CONTROL_ENABLE_OPERATION if enable else CONTROL_DISABLE_VOLTAGE
        for offset, node in enumerate(DRIVE_NODES):
            self.send(
                CanFrame(0x200 + node, self.rpdo0_velocity_payload(controlword, velocities[offset])),
                f"Node{node} RPDO0 velocity cw=0x{controlword:04X} velocity={velocities[offset]}",
            )
        self.sync()

    def send_steer_group(self, positions: tuple[int, int, int, int]) -> None:
        for offset, node in enumerate(STEER_NODES):
            self.send(
                CanFrame(0x300 + node, self.rpdo1_position_payload(CONTROL_ABSOLUTE_UPDATE_ARM, positions[offset])),
                f"Node{node} RPDO1 arm target={positions[offset]}",
            )
        self.sync()
        for offset, node in enumerate(STEER_NODES):
            self.send(
                CanFrame(0x300 + node, self.rpdo1_position_payload(CONTROL_ABSOLUTE_UPDATE_TRIGGER, positions[offset])),
                f"Node{node} RPDO1 trigger target={positions[offset]}",
            )
        self.sync()

    @staticmethod
    def node_from_feedback_id(can_id: int) -> int | None:
        if 0x180 <= can_id <= 0x18F:
            return can_id - 0x180
        if 0x280 <= can_id <= 0x28F:
            return can_id - 0x280
        if 0x080 <= can_id <= 0x08F:
            return can_id - 0x080
        return None

    @staticmethod
    def decode_feedback(frame: CanFrame) -> dict[str, int | str]:
        if 0x180 <= frame.can_id <= 0x18F and len(frame.data) == 8:
            return {
                "actual_position": int.from_bytes(frame.data[0:4], "little", signed=True),
                "actual_velocity": int.from_bytes(frame.data[4:8], "little", signed=True),
            }
        if 0x280 <= frame.can_id <= 0x28F and len(frame.data) == 8:
            return {
                "latched_fault": f"0x{int.from_bytes(frame.data[0:4], 'little'):08X}",
                "statusword": f"0x{int.from_bytes(frame.data[4:6], 'little'):04X}",
                "actual_current": int.from_bytes(frame.data[6:8], "little", signed=True),
            }
        if 0x080 <= frame.can_id <= 0x08F:
            return {"emcy": frame.data.hex(" ").upper()}
        return {}

    def drain_feedback_nonblocking(self) -> None:
        wanted = {0x180 + node for node in ALL_NODES}
        wanted.update({0x280 + node for node in ALL_NODES})
        wanted.update({0x080 + node for node in ALL_NODES})

        cached = self.adapter._rx_cache[BUS]  # type: ignore[attr-defined] # Debug tool; nonblocking drain.
        kept_cache: list[CanFrame] = []
        frames: list[CanFrame] = []
        for frame in cached:
            if frame.can_id in wanted:
                frames.append(frame)
            else:
                kept_cache.append(frame)
        cached[:] = kept_cache

        channel = self.adapter._channels[BUS]  # type: ignore[attr-defined]
        for raw in self.adapter._device.receive(channel, limit=400, wait_ms=0):  # type: ignore[attr-defined]
            data = bytes(int(raw.Data[i]) for i in range(int(raw.DataLen)))
            frame = CanFrame(int(raw.ID), data, bool(raw.ExternFlag), bool(raw.RemoteFlag))
            if frame.can_id in wanted:
                frames.append(frame)
            else:
                cached.append(frame)

        for frame in frames:
            record = {
                "time_s": time.time(),
                "can_id": f"0x{frame.can_id:03X}",
                "node": self.node_from_feedback_id(frame.can_id),
                "data": frame.data.hex(" ").upper(),
                "decoded": self.decode_feedback(frame),
            }
            self.feedback.append(record)
            self.event("rx_observe", **record)

    def setup_nodes(self, profile_velocity: int, profile_accel: int, fault_reset: bool) -> None:
        for node in ALL_NODES:
            self.nmt_operational(node)
            time.sleep(0.01)
            self.sdo_download(node, 0x2300, 0x00, 2, 0x001E, "control source CANopen")
            if fault_reset:
                self.sdo_download(node, 0x6040, 0x00, 2, 0x0000, "disable before reset")
                time.sleep(0.02)
                self.sdo_download(node, 0x6040, 0x00, 2, 0x0080, "fault reset")
                time.sleep(0.05)
            if node in DRIVE_NODES:
                self.sdo_download(node, 0x6060, 0x00, 1, MODE_PROFILE_VELOCITY, "profile velocity mode")
            else:
                self.sdo_download(node, 0x6060, 0x00, 1, MODE_PROFILE_POSITION, "profile position mode")
                self.sdo_download(node, 0x6081, 0x00, 4, profile_velocity, "profile velocity")
                self.sdo_download(node, 0x6083, 0x00, 4, profile_accel, "profile acceleration")
                self.sdo_download(node, 0x6084, 0x00, 4, profile_accel, "profile deceleration")
            self.sdo_download(node, 0x6040, 0x00, 2, CONTROL_ENABLE_OPERATION, "enable operation")
            mode = self.sdo_upload(node, 0x6061, 0x00, 1, "mode display")
            expected = MODE_PROFILE_VELOCITY if node in DRIVE_NODES else MODE_PROFILE_POSITION
            if mode.get("signed_value") != expected:
                raise RuntimeError(f"Node{node} mode display {mode}, expected {expected}")

    def read_node_state(self, name: str) -> dict[str, object]:
        objects = (
            (0x6061, 0x00, 1, "mode_display"),
            (0x6041, 0x00, 2, "statusword"),
            (0x6064, 0x00, 4, "actual_position"),
            (0x606C, 0x00, 4, "actual_velocity"),
            (0x2183, 0x00, 4, "latched_fault"),
        )
        result: dict[str, object] = {}
        for node in ALL_NODES:
            result[str(node)] = {
                f"0x{index:04X}:{subindex}": self.sdo_upload(node, index, subindex, size, note)
                for index, subindex, size, note in objects
            }
        self.write_json(name, result)
        return result

    @staticmethod
    def wheel_position(wheel: int) -> tuple[float, float]:
        half_wheelbase = WHEELBASE_MM * 0.5
        half_track = TRACK_WIDTH_MM * 0.5
        if wheel == WHEEL_FR:
            return half_wheelbase, -half_track
        if wheel == WHEEL_FL:
            return half_wheelbase, half_track
        if wheel == WHEEL_RL:
            return -half_wheelbase, half_track
        return -half_wheelbase, -half_track

    @staticmethod
    def signed_speed_magnitude(vx: float, vy: float, center_speed: float) -> float:
        magnitude = math.sqrt(vx * vx + vy * vy)
        return -magnitude if center_speed < 0.0 else magnitude

    @classmethod
    def build_ackermann(cls, speed_kph: float, steer_deg: float, reverse: bool) -> WheelCommand:
        signed_steer = -steer_deg if reverse else steer_deg
        fixed_speed = -speed_kph if reverse else speed_kph
        if abs(signed_steer) < 0.01:
            return WheelCommand((0.0, 0.0, 0.0, 0.0), (fixed_speed, fixed_speed, fixed_speed, fixed_speed))
        radius = WHEELBASE_MM / math.tan(abs(signed_steer) * math.pi / 180.0)
        radius = min(10_000_000.0, max(MIN_TURN_RADIUS_MM, radius))
        if signed_steer < 0.0:
            radius = -radius
        steering_omega = 1.0 / radius
        speed_omega = fixed_speed / radius
        steer: list[float] = []
        speed: list[float] = []
        for wheel in range(4):
            x, y = cls.wheel_position(wheel)
            axis_vx = 1.0 - steering_omega * y
            axis_vy = steering_omega * x
            linear_vx = fixed_speed - speed_omega * y
            linear_vy = speed_omega * x
            steer.append(math.atan2(axis_vy, axis_vx) * 180.0 / math.pi)
            speed.append(cls.signed_speed_magnitude(linear_vx, linear_vy, fixed_speed))
        return WheelCommand(tuple(steer), tuple(speed))  # type: ignore[arg-type]

    @staticmethod
    def build_spin(speed_kph: float, spin_angle_deg: float) -> WheelCommand:
        return WheelCommand(
            (spin_angle_deg, -spin_angle_deg, spin_angle_deg, -spin_angle_deg),
            (speed_kph, -speed_kph, -speed_kph, speed_kph),
        )

    @staticmethod
    def build_crab(speed_kph: float, steer_deg: float) -> WheelCommand:
        return WheelCommand((steer_deg, steer_deg, steer_deg, steer_deg), (speed_kph, speed_kph, speed_kph, speed_kph))

    @staticmethod
    def to_position_counts(steer_deg: tuple[float, float, float, float]) -> tuple[int, int, int, int]:
        return tuple(int(max(-500_000, min(500_000, round(value * STEER_DEG_TO_COUNTS)))) for value in steer_deg)  # type: ignore[return-value]

    @staticmethod
    def to_velocity_units(speed_kph: tuple[float, float, float, float]) -> tuple[int, int, int, int]:
        return tuple(int(round(value * DRIVE_SPEED_KPH_TO_UNITS)) for value in speed_kph)  # type: ignore[return-value]

    @staticmethod
    def interpolate_command(start: WheelCommand, end: WheelCommand, ratio: float) -> WheelCommand:
        ratio = max(0.0, min(1.0, ratio))
        steer = tuple(start.steer_deg[i] + (end.steer_deg[i] - start.steer_deg[i]) * ratio for i in range(4))
        speed = tuple(start.speed_kph[i] + (end.speed_kph[i] - start.speed_kph[i]) * ratio for i in range(4))
        return WheelCommand(steer, speed)  # type: ignore[arg-type]

    def send_command(self, command: WheelCommand, sample: int, mode: str, enable_drive: bool) -> None:
        positions = self.to_position_counts(command.steer_deg)
        velocities = self.to_velocity_units(command.speed_kph)
        started = time.monotonic()
        self.send_steer_group(positions)
        self.send_drive_group(velocities, enable_drive)
        elapsed_ms = (time.monotonic() - started) * 1000.0
        self.command_trace.append({
            "time_s": time.time(),
            "sample": sample,
            "mode": mode,
            "steer_deg": [round(value, 3) for value in command.steer_deg],
            "speed_kph": [round(value, 3) for value in command.speed_kph],
            "positions": positions,
            "velocities": velocities,
            "enable_drive": enable_drive,
        })
        self.timing_trace.append({"sample": sample, "mode": mode, "send_duration_ms": round(elapsed_ms, 3)})

    def stop_all(self) -> None:
        zero = WheelCommand((0.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0))
        for i in range(5):
            self.send_command(zero, -1 - i, "stop", enable_drive=True)
            time.sleep(0.04)
        self.send_drive_group((0, 0, 0, 0), enable=False)

    def run_plan(self, speed_kph: float, steer_deg: float, period_ms: int,
                 samples_per_segment: int, modes: list[str], profile_velocity: int,
                 profile_accel: int, fault_reset: bool) -> dict[str, object]:
        self.setup_nodes(profile_velocity, profile_accel, fault_reset)
        before = self.read_node_state("before.json")
        self.stop_all()
        self.feedback.clear()

        neutral = WheelCommand((0.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0))
        sample = 0
        for mode in modes:
            if mode == "ackermann":
                target = self.build_ackermann(speed_kph, steer_deg, reverse=False)
            elif mode == "reverse_ackermann":
                target = self.build_ackermann(speed_kph, steer_deg, reverse=True)
            elif mode == "spin":
                target = self.build_spin(speed_kph, 25.0)
            elif mode == "crab":
                target = self.build_crab(speed_kph, steer_deg)
            else:
                raise ValueError(f"unsupported mode {mode}")

            phases = (
                (neutral, target, samples_per_segment // 3),
                (target, target, samples_per_segment // 3),
                (target, neutral, samples_per_segment - 2 * (samples_per_segment // 3)),
            )
            for start, end, count in phases:
                for local in range(max(1, count)):
                    ratio = 1.0 if count <= 1 else local / float(count - 1)
                    command = self.interpolate_command(start, end, ratio)
                    tick = time.monotonic()
                    self.send_command(command, sample, mode, enable_drive=True)
                    self.drain_feedback_nonblocking()
                    sample += 1
                    sleep_s = period_ms / 1000.0 - (time.monotonic() - tick)
                    if sleep_s > 0:
                        time.sleep(sleep_s)
            self.stop_all()
            time.sleep(0.5)

        self.stop_all()
        time.sleep(0.5)
        self.drain_feedback_nonblocking()
        after = self.read_node_state("after.json")

        emcy_count = sum(1 for item in self.feedback if isinstance(item.get("decoded"), dict) and "emcy" in item["decoded"])
        by_node: dict[str, dict[str, object]] = {}
        for item in self.feedback:
            node = item.get("node")
            decoded = item.get("decoded")
            if not isinstance(node, int) or not isinstance(decoded, dict):
                continue
            entry = by_node.setdefault(str(node), {"tpdo0": 0, "tpdo1": 0, "max_abs_velocity": 0, "last_statusword": ""})
            if "actual_velocity" in decoded:
                entry["tpdo0"] = int(entry["tpdo0"]) + 1
                entry["max_abs_velocity"] = max(int(entry["max_abs_velocity"]), abs(int(decoded["actual_velocity"])))
            if "statusword" in decoded:
                entry["tpdo1"] = int(entry["tpdo1"]) + 1
                entry["last_statusword"] = decoded["statusword"]

        summary = {
            "speed_kph": speed_kph,
            "steer_deg": steer_deg,
            "period_ms": period_ms,
            "samples_per_segment": samples_per_segment,
            "modes": modes,
            "command_count": len(self.command_trace),
            "feedback_count": len(self.feedback),
            "emcy_count": emcy_count,
            "max_send_duration_ms": max((float(item["send_duration_ms"]) for item in self.timing_trace), default=0.0),
            "feedback_by_node": by_node,
            "before": before,
            "after": after,
        }
        self.write_json("summary.json", summary)
        return summary


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Suspended-wheel CANopen remote-motion simulator for Node1-8.")
    parser.add_argument("--allow-motion", action="store_true", help="required: confirms wheels are lifted and free")
    parser.add_argument("--speed-kph", type=float, default=0.30)
    parser.add_argument("--steer-deg", type=float, default=10.0)
    parser.add_argument("--period-ms", type=int, default=50)
    parser.add_argument("--samples-per-segment", type=int, default=60)
    parser.add_argument("--modes", default="ackermann,reverse_ackermann,crab,spin")
    parser.add_argument("--profile-velocity", type=int, default=1_666_666)
    parser.add_argument("--profile-accel", type=int, default=20_000_000)
    parser.add_argument("--timeout-ms", type=int, default=900)
    parser.add_argument("--fault-reset-before-test", action="store_true")
    parser.add_argument("--log-dir", default="")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.allow_motion:
        print("FAILED: --allow-motion is required", file=sys.stderr)
        return 1
    modes = [item.strip() for item in args.modes.split(",") if item.strip()]
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    log_dir = Path(args.log_dir) if args.log_dir else REPO_ROOT / "out" / f"motion8_remote_sim_{timestamp}"
    try:
        with Motion8Debug(args.timeout_ms, log_dir) as debug:
            summary = debug.run_plan(
                args.speed_kph,
                args.steer_deg,
                args.period_ms,
                args.samples_per_segment,
                modes,
                args.profile_velocity,
                args.profile_accel,
                args.fault_reset_before_test,
            )
        print(json.dumps({"log_dir": str(log_dir), **summary}, indent=2, ensure_ascii=False))
        return 0
    except Exception as exc:  # noqa: BLE001 - hardware CLI should report the exact failure.
        print(f"FAILED: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
