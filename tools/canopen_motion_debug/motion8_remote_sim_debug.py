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
DRIVE_DIRECTION_SIGNS = (1, -1, -1, 1)

CONTROL_DISABLE_VOLTAGE = 0x0000
CONTROL_ENABLE_OPERATION = 0x000F
CONTROL_ABSOLUTE_UPDATE_ARM = 0x002F
CONTROL_ABSOLUTE_UPDATE_TRIGGER = 0x003F

MODE_PROFILE_POSITION = 1
MODE_PROFILE_VELOCITY = 3

WHEELBASE_MM = 2880.0
TRACK_WIDTH_MM = 1980.0
MIN_TURN_RADIUS_MM = 2880.0
ENCODER_COUNTS_PER_MOTOR_REV = 2500.0 * 4.0
DRIVE_GEAR_REDUCTION = 86.6
STEER_GEAR_REDUCTION = 490.0
DRIVE_WHEEL_DIAMETER_M = 0.580
DRIVE_WHEEL_CIRCUMFERENCE_M = math.pi * DRIVE_WHEEL_DIAMETER_M
SERVO_COMMISSIONING_MAX_RPM = 3000.0
DRIVE_MOTOR_MAX_RPM = SERVO_COMMISSIONING_MAX_RPM
SERVO_COMMISSIONING_MAX_ACCEL_RPS2 = 50.0
SERVO_PROFILE_ACCEL_LIMIT_COUNTS_PER_SEC2 = 500_000
DRIVE_SPEED_MPS_TO_UNITS = (
    ENCODER_COUNTS_PER_MOTOR_REV
    * DRIVE_GEAR_REDUCTION
    * 10.0
    / DRIVE_WHEEL_CIRCUMFERENCE_M
)
DRIVE_MAX_SPEED_MPS = (
    DRIVE_MOTOR_MAX_RPM
    / DRIVE_GEAR_REDUCTION
    * DRIVE_WHEEL_CIRCUMFERENCE_M
    / 60.0
)
STEER_DEG_TO_COUNTS = ENCODER_COUNTS_PER_MOTOR_REV * STEER_GEAR_REDUCTION / 360.0
STEER_MAX_COUNTS = int(round(90.0 * STEER_DEG_TO_COUNTS))
SERVO_MAX_VELOCITY_UNITS_FROM_RPM = 5_000_000
DEFAULT_STEER_PROFILE_VELOCITY = 3_000_000
DEFAULT_PRESTEER_TOLERANCE_COUNTS = 35_000
DEFAULT_PRESTEER_TIMEOUT_SEC = 8.0

WHEEL_FR = 0
WHEEL_FL = 1
WHEEL_RL = 2
WHEEL_RR = 3


@dataclass(frozen=True)
class WheelCommand:
    steer_deg: tuple[float, float, float, float]
    speed_mps: tuple[float, float, float, float]


class Motion8Debug:
    def __init__(self,
                 timeout_ms: int,
                 log_dir: Path,
                 active_wheels: tuple[int, ...] = (0, 1, 2, 3),
                 steering_enabled: bool = True) -> None:
        self.timeout_ms = timeout_ms
        self.log_dir = log_dir
        self.active_wheels = active_wheels
        self.steering_enabled = steering_enabled
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
        if exc_type is not None:
            try:
                self.stop_all()
            except Exception as stop_exc:  # noqa: BLE001 - preserve the original hardware failure.
                self.event("emergency_stop_failed", error=str(stop_exc))
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
        for offset, node in enumerate(DRIVE_NODES):
            active = offset in self.active_wheels
            if not active:
                continue
            controlword = CONTROL_ENABLE_OPERATION if enable else CONTROL_DISABLE_VOLTAGE
            velocity = velocities[offset]
            self.send(
                CanFrame(0x200 + node, self.rpdo0_velocity_payload(controlword, velocity)),
                f"Node{node} RPDO0 velocity cw=0x{controlword:04X} velocity={velocity}",
            )
        self.sync()

    def send_steer_group(self, positions: tuple[int, int, int, int]) -> None:
        if not self.steering_enabled:
            return
        for offset, node in enumerate(STEER_NODES):
            if offset not in self.active_wheels:
                continue
            self.send(
                CanFrame(0x300 + node, self.rpdo1_position_payload(CONTROL_ABSOLUTE_UPDATE_ARM, positions[offset])),
                f"Node{node} RPDO1 arm target={positions[offset]}",
            )
        self.sync()
        for offset, node in enumerate(STEER_NODES):
            if offset not in self.active_wheels:
                continue
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
    def drive_sign_from_node(node: int) -> int:
        if node in DRIVE_NODES:
            return DRIVE_DIRECTION_SIGNS[DRIVE_NODES.index(node)]
        return 1

    @staticmethod
    def decode_feedback(frame: CanFrame) -> dict[str, int | str]:
        if 0x180 <= frame.can_id <= 0x18F and len(frame.data) == 8:
            node = frame.can_id - 0x180
            actual_velocity = int.from_bytes(frame.data[4:8], "little", signed=True)
            decoded: dict[str, int | str] = {
                "actual_position": int.from_bytes(frame.data[0:4], "little", signed=True),
                "actual_velocity": actual_velocity,
            }
            if node in DRIVE_NODES:
                vehicle_velocity_mps = Motion8Debug.velocity_units_to_mps(
                    actual_velocity,
                    Motion8Debug.drive_sign_from_node(node),
                )
                decoded["actual_velocity_mps"] = f"{vehicle_velocity_mps:.3f}"
            return decoded
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

    def setup_nodes(self, profile_velocity: int, profile_accel: int, fault_reset: bool,
                    nodes: tuple[int, ...] = ALL_NODES) -> None:
        for node in nodes:
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
                self.sdo_download(node, 0x6083, 0x00, 4, profile_accel, "profile acceleration")
                self.sdo_download(node, 0x6084, 0x00, 4, profile_accel, "profile deceleration")
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

    def read_node_state(self, name: str, nodes: tuple[int, ...] = ALL_NODES) -> dict[str, object]:
        objects = (
            (0x6061, 0x00, 1, "mode_display"),
            (0x6041, 0x00, 2, "statusword"),
            (0x6064, 0x00, 4, "actual_position"),
            (0x606C, 0x00, 4, "actual_velocity"),
            (0x2183, 0x00, 4, "latched_fault"),
        )
        result: dict[str, object] = {}
        for node in nodes:
            result[str(node)] = {
                f"0x{index:04X}:{subindex}": self.sdo_upload(node, index, subindex, size, note)
                for index, subindex, size, note in objects
            }
        self.write_json(name, result)
        return result

    @staticmethod
    def sdo_signed_value(reply: dict[str, object], object_name: str, node: int) -> int:
        if reply.get("status") != "ok" or not isinstance(reply.get("signed_value"), int):
            raise RuntimeError(f"Node{node} failed to read {object_name}: {reply}")
        return int(reply["signed_value"])

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
    def build_ackermann(cls, speed_mps: float, steer_deg: float, reverse: bool) -> WheelCommand:
        signed_steer = -steer_deg if reverse else steer_deg
        fixed_speed = -speed_mps if reverse else speed_mps
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
        return cls.limit_max_wheel_speed(WheelCommand(tuple(steer), tuple(speed)), abs(speed_mps))

    @staticmethod
    def build_spin(speed_mps: float, spin_angle_deg: float) -> WheelCommand:
        return WheelCommand(
            (spin_angle_deg, -spin_angle_deg, spin_angle_deg, -spin_angle_deg),
            (speed_mps, -speed_mps, -speed_mps, speed_mps),
        )

    @staticmethod
    def build_crab(speed_mps: float, steer_deg: float) -> WheelCommand:
        return WheelCommand((steer_deg, steer_deg, steer_deg, steer_deg), (speed_mps, speed_mps, speed_mps, speed_mps))

    @staticmethod
    def limit_max_wheel_speed(command: WheelCommand, max_abs_speed_mps: float) -> WheelCommand:
        max_actual = max((abs(value) for value in command.speed_mps), default=0.0)
        if max_actual <= max_abs_speed_mps or max_actual <= 0.0:
            return command
        scale = max_abs_speed_mps / max_actual
        return WheelCommand(
            command.steer_deg,
            tuple(value * scale for value in command.speed_mps),  # type: ignore[arg-type]
        )

    @staticmethod
    def to_position_counts(steer_deg: tuple[float, float, float, float]) -> tuple[int, int, int, int]:
        return tuple(int(max(-STEER_MAX_COUNTS, min(STEER_MAX_COUNTS, round(value * STEER_DEG_TO_COUNTS)))) for value in steer_deg)  # type: ignore[return-value]

    @staticmethod
    def to_velocity_units(speed_mps: tuple[float, float, float, float]) -> tuple[int, int, int, int]:
        return tuple(
            int(round(value * DRIVE_SPEED_MPS_TO_UNITS * DRIVE_DIRECTION_SIGNS[index]))
            for index, value in enumerate(speed_mps)
        )  # type: ignore[return-value]

    @staticmethod
    def velocity_units_to_mps(velocity_units: int, drive_sign: int = 1) -> float:
        motor_counts_per_s = float(velocity_units) * 0.1
        wheel_rev_per_s = motor_counts_per_s / (ENCODER_COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)
        return wheel_rev_per_s * DRIVE_WHEEL_CIRCUMFERENCE_M * float(drive_sign)

    @staticmethod
    def interpolate_command(start: WheelCommand, end: WheelCommand, ratio: float) -> WheelCommand:
        ratio = max(0.0, min(1.0, ratio))
        steer = tuple(start.steer_deg[i] + (end.steer_deg[i] - start.steer_deg[i]) * ratio for i in range(4))
        speed = tuple(start.speed_mps[i] + (end.speed_mps[i] - start.speed_mps[i]) * ratio for i in range(4))
        return WheelCommand(steer, speed)  # type: ignore[arg-type]

    def send_command(self, command: WheelCommand, sample: int, mode: str, enable_drive: bool) -> None:
        command = self.apply_active_wheel_mask(command)
        positions = self.to_position_counts(command.steer_deg)
        velocities = self.to_velocity_units(command.speed_mps)
        started = time.monotonic()
        self.send_steer_group(positions)
        self.send_drive_group(velocities, enable_drive)
        elapsed_ms = (time.monotonic() - started) * 1000.0
        self.command_trace.append({
            "time_s": time.time(),
            "sample": sample,
            "mode": mode,
            "steer_deg": [round(value, 3) for value in command.steer_deg],
            "speed_mps": [round(value, 3) for value in command.speed_mps],
            "positions": positions,
            "velocities": velocities,
            "enable_drive": enable_drive,
        })
        self.timing_trace.append({"sample": sample, "mode": mode, "send_duration_ms": round(elapsed_ms, 3)})

    def command_steer_only(self, target: WheelCommand, sample: int, mode: str) -> None:
        """Move steering axes while drive RPDO targets remain zero.

        Crab and spin commissioning must not start wheel rotation before the
        steering axes have reached the requested geometry.  This helper sends
        only the steering group and records the same command trace format used
        by normal motion samples.
        """
        command = self.apply_active_wheel_mask(
            WheelCommand(target.steer_deg, (0.0, 0.0, 0.0, 0.0)),
        )
        positions = self.to_position_counts(command.steer_deg)
        started = time.monotonic()
        self.send_steer_group(positions)
        elapsed_ms = (time.monotonic() - started) * 1000.0
        self.command_trace.append({
            "time_s": time.time(),
            "sample": sample,
            "mode": mode,
            "steer_deg": [round(value, 3) for value in command.steer_deg],
            "speed_mps": [0.0, 0.0, 0.0, 0.0],
            "positions": positions,
            "velocities": (0, 0, 0, 0),
            "enable_drive": False,
        })
        self.timing_trace.append({"sample": sample, "mode": mode, "send_duration_ms": round(elapsed_ms, 3)})

    def wait_for_steer_positions(self, target: WheelCommand, tolerance_counts: int,
                                 timeout_sec: float, sample: int, mode: str,
                                 period_ms: int) -> tuple[int, bool]:
        target_counts = self.to_position_counts(target.steer_deg)
        deadline = time.monotonic() + timeout_sec
        ok = False
        while time.monotonic() < deadline:
            max_error = 0
            missing = False
            for offset, node in enumerate(STEER_NODES):
                if offset not in self.active_wheels:
                    continue
                reply = self.sdo_upload(node, 0x6064, 0x00, 4, "presteer actual position")
                actual = self.sdo_signed_value(reply, "0x6064 actual position", node)
                error = abs(actual - target_counts[offset])
                max_error = max(max_error, error)
                if error > tolerance_counts:
                    missing = True
            self.event("presteer_position_check",
                       mode=mode,
                       sample=sample,
                       max_error_counts=max_error,
                       tolerance_counts=tolerance_counts)
            if not missing:
                ok = True
                break
            self.command_steer_only(target, sample, f"{mode}_presteer_hold")
            self.drain_feedback_nonblocking()
            sample += 1
            time.sleep(max(0.02, period_ms / 1000.0))
        if not ok:
            self.event("presteer_position_timeout",
                       mode=mode,
                       sample=sample,
                       tolerance_counts=tolerance_counts)
        return sample, ok

    def apply_active_wheel_mask(self, command: WheelCommand) -> WheelCommand:
        active = set(self.active_wheels)
        return WheelCommand(
            tuple(command.steer_deg[i] if i in active else 0.0 for i in range(4)),  # type: ignore[arg-type]
            tuple(command.speed_mps[i] if i in active else 0.0 for i in range(4)),  # type: ignore[arg-type]
        )

    def stop_all(self) -> None:
        zero = WheelCommand((0.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0))
        for i in range(5):
            self.send_command(zero, -1 - i, "stop", enable_drive=True)
            time.sleep(0.04)
        self.send_drive_group((0, 0, 0, 0), enable=False)

    def run_plan(self, speed_mps: float, steer_deg: float, spin_deg: float,
                 period_ms: int, samples_per_segment: int, modes: list[str],
                 profile_velocity: int, profile_accel: int,
                 fault_reset: bool) -> dict[str, object]:
        self.setup_nodes(profile_velocity, profile_accel, fault_reset)
        before = self.read_node_state("before.json")
        self.stop_all()
        self.feedback.clear()

        neutral = WheelCommand((0.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0))
        sample = 0
        for mode in modes:
            if mode == "ackermann":
                target = self.build_ackermann(speed_mps, steer_deg, reverse=False)
            elif mode == "reverse_ackermann":
                target = self.build_ackermann(speed_mps, steer_deg, reverse=True)
            elif mode == "spin":
                target = self.build_spin(speed_mps, spin_deg)
            elif mode == "crab":
                target = self.build_crab(speed_mps, steer_deg)
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

        summary = self.build_summary(
            speed_mps,
            steer_deg,
            spin_deg,
            period_ms,
            samples_per_segment,
            modes,
            before,
            after,
        )
        self.write_json("summary.json", summary)
        return summary

    def execute_samples(self, start: WheelCommand, end: WheelCommand, count: int,
                        period_ms: int, sample: int, mode: str) -> tuple[int, WheelCommand]:
        count = max(1, count)
        last_command = end
        for local in range(count):
            ratio = 1.0 if count <= 1 else local / float(count - 1)
            command = self.interpolate_command(start, end, ratio)
            tick = time.monotonic()
            self.send_command(command, sample, mode, enable_drive=True)
            self.drain_feedback_nonblocking()
            last_command = command
            sample += 1
            sleep_s = period_ms / 1000.0 - (time.monotonic() - tick)
            if sleep_s > 0:
                time.sleep(sleep_s)
        return sample, last_command

    def run_smooth_commissioning_scenario(self, speed_mps: float, steer_deg: float,
                                          spin_deg: float, period_ms: int,
                                          ramp_sec: float, ackermann_sec: float,
                                          spin_sec: float, crab_sec: float,
                                          profile_velocity: int, profile_accel: int,
                                          fault_reset: bool) -> dict[str, object]:
        self.setup_nodes(profile_velocity, profile_accel, fault_reset)
        before = self.read_node_state("before.json")
        self.stop_all()
        self.feedback.clear()

        neutral = WheelCommand((0.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0))
        segments = (
            ("ackermann_forward", self.build_ackermann(speed_mps, steer_deg, reverse=False), ackermann_sec),
            ("ackermann_reverse", self.build_ackermann(-speed_mps, steer_deg, reverse=False), ackermann_sec),
            ("reverse_ackermann_forward", self.build_ackermann(speed_mps, steer_deg, reverse=True), ackermann_sec),
            ("reverse_ackermann_reverse", self.build_ackermann(-speed_mps, steer_deg, reverse=True), ackermann_sec),
            ("spin_clockwise", self.build_spin(speed_mps, spin_deg), spin_sec),
            ("spin_counter_clockwise", self.build_spin(-speed_mps, spin_deg), spin_sec),
            ("crab_forward", self.build_crab(speed_mps, steer_deg), crab_sec),
            ("crab_reverse", self.build_crab(-speed_mps, steer_deg), crab_sec),
        )
        ramp_samples = max(1, int(round(ramp_sec * 1000.0 / float(period_ms))))
        sample = 0
        current = neutral
        for name, target, hold_sec in segments:
            sample, current = self.execute_samples(current,
                                                   target,
                                                   ramp_samples,
                                                   period_ms,
                                                   sample,
                                                   f"{name}_ramp")
            hold_samples = max(1, int(round(hold_sec * 1000.0 / float(period_ms))))
            sample, current = self.execute_samples(target,
                                                   target,
                                                   hold_samples,
                                                   period_ms,
                                                   sample,
                                                   name)

        sample, current = self.execute_samples(current,
                                               neutral,
                                               ramp_samples,
                                               period_ms,
                                               sample,
                                               "final_ramp_to_stop")
        self.stop_all()
        time.sleep(0.5)
        self.drain_feedback_nonblocking()
        after = self.read_node_state("after.json")

        summary = self.build_summary(
            speed_mps,
            steer_deg,
            spin_deg,
            period_ms,
            sample,
            [item[0] for item in segments],
            before,
            after,
        )
        summary["scenario"] = "smooth_commissioning"
        summary["ramp_sec"] = ramp_sec
        summary["ackermann_sec"] = ackermann_sec
        summary["spin_sec"] = spin_sec
        summary["crab_sec"] = crab_sec
        self.write_json("summary.json", summary)
        return summary

    def run_single_wheel_speed_sweep(self, speed_steps_mps: tuple[float, ...],
                                     period_ms: int, ramp_sec: float, hold_sec: float,
                                     profile_velocity: int, profile_accel: int,
                                     fault_reset: bool) -> dict[str, object]:
        drive_nodes = tuple(DRIVE_NODES[wheel] for wheel in self.active_wheels)
        self.setup_nodes(profile_velocity, profile_accel, fault_reset, nodes=drive_nodes)
        before = self.read_node_state("before.json", nodes=drive_nodes)
        self.stop_all()
        self.feedback.clear()

        neutral = WheelCommand((0.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0))
        current = neutral
        sample = 0
        ramp_samples = max(1, int(round(ramp_sec * 1000.0 / float(period_ms))))
        hold_samples = max(1, int(round(hold_sec * 1000.0 / float(period_ms))))
        settle_samples = max(1, int(round(3.0 * 1000.0 / float(period_ms))))
        aborted_on_emcy = False

        sample, current = self.execute_samples(neutral,
                                               neutral,
                                               settle_samples,
                                               period_ms,
                                               sample,
                                               "brake_release_zero_speed_settle")
        settle_emcy = sum(1 for item in self.feedback
                          if isinstance(item.get("decoded"), dict) and "emcy" in item["decoded"])
        if settle_emcy > 0:
            aborted_on_emcy = True
            self.event("single_wheel_sweep_aborted_during_brake_release_settle",
                       emcy_count=settle_emcy)

        for step in (() if aborted_on_emcy else speed_steps_mps):
            target_speeds = [0.0, 0.0, 0.0, 0.0]
            for wheel in self.active_wheels:
                target_speeds[wheel] = step
            target = WheelCommand((0.0, 0.0, 0.0, 0.0), tuple(target_speeds))  # type: ignore[arg-type]
            emcy_before = sum(1 for item in self.feedback
                              if isinstance(item.get("decoded"), dict) and "emcy" in item["decoded"])
            sample, current = self.execute_samples(current,
                                                   target,
                                                   ramp_samples,
                                                   period_ms,
                                                   sample,
                                                   f"wheel_sweep_ramp_{step:.3f}mps")
            sample, current = self.execute_samples(target,
                                                   target,
                                                   hold_samples,
                                                   period_ms,
                                                   sample,
                                                   f"wheel_sweep_hold_{step:.3f}mps")
            emcy_after = sum(1 for item in self.feedback
                             if isinstance(item.get("decoded"), dict) and "emcy" in item["decoded"])
            if emcy_after > emcy_before:
                aborted_on_emcy = True
                self.event("single_wheel_sweep_aborted_on_emcy",
                           speed_mps=step,
                           emcy_delta=emcy_after - emcy_before)
                break

        sample, current = self.execute_samples(current,
                                               neutral,
                                               ramp_samples,
                                               period_ms,
                                               sample,
                                               "single_wheel_final_ramp_to_stop")
        self.stop_all()
        time.sleep(0.5)
        self.drain_feedback_nonblocking()
        after = self.read_node_state("after.json", nodes=drive_nodes)
        summary = self.build_summary(
            max((abs(value) for value in speed_steps_mps), default=0.0),
            0.0,
            0.0,
            period_ms,
            sample,
            ["single_wheel_speed_sweep"],
            before,
            after,
        )
        summary["scenario"] = "single_wheel_speed_sweep"
        summary["active_wheels"] = [wheel + 1 for wheel in self.active_wheels]
        summary["speed_steps_mps"] = list(speed_steps_mps)
        summary["ramp_sec"] = ramp_sec
        summary["hold_sec"] = hold_sec
        summary["aborted_on_emcy"] = aborted_on_emcy
        self.write_json("summary.json", summary)
        return summary

    def run_ackermann_remote_sweep(self, speed_mps: float, steer_deg: float,
                                   period_ms: int, ramp_sec: float, sweep_sec: float,
                                   sweep_cycles: float, profile_velocity: int,
                                   profile_accel: int, fault_reset: bool) -> dict[str, object]:
        """Simulate a real remote stick that continuously sweeps steering.

        The test keeps only the latest target per control period.  Steering is
        generated from one coherent virtual joystick sample, then converted into
        all four wheel angles and speeds before any RPDO is transmitted.
        """
        self.setup_nodes(profile_velocity, profile_accel, fault_reset)
        before = self.read_node_state("before.json")
        self.stop_all()
        self.feedback.clear()

        neutral = WheelCommand((0.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0))
        straight = self.build_ackermann(speed_mps, 0.0, reverse=False)
        ramp_samples = max(1, int(round(ramp_sec * 1000.0 / float(period_ms))))
        sweep_samples = max(1, int(round(sweep_sec * 1000.0 / float(period_ms))))
        sample = 0
        sample, current = self.execute_samples(neutral,
                                               straight,
                                               ramp_samples,
                                               period_ms,
                                               sample,
                                               "ackermann_remote_speed_ramp")

        for local in range(sweep_samples):
            phase = float(local) / float(max(1, sweep_samples - 1))
            steer = math.sin(phase * 2.0 * math.pi * sweep_cycles) * steer_deg
            command = self.build_ackermann(speed_mps, steer, reverse=False)
            tick = time.monotonic()
            self.send_command(command, sample, "ackermann_remote_sweep", enable_drive=True)
            self.drain_feedback_nonblocking()
            current = command
            sample += 1
            sleep_s = period_ms / 1000.0 - (time.monotonic() - tick)
            if sleep_s > 0:
                time.sleep(sleep_s)

        sample, current = self.execute_samples(current,
                                               neutral,
                                               ramp_samples,
                                               period_ms,
                                               sample,
                                               "ackermann_remote_final_ramp_to_stop")
        self.stop_all()
        time.sleep(0.5)
        self.drain_feedback_nonblocking()
        after = self.read_node_state("after.json")

        summary = self.build_summary(
            speed_mps,
            steer_deg,
            0.0,
            period_ms,
            sample,
            ["ackermann_remote_sweep"],
            before,
            after,
        )
        summary["scenario"] = "ackermann_remote_sweep"
        summary["ramp_sec"] = ramp_sec
        summary["sweep_sec"] = sweep_sec
        summary["sweep_cycles"] = sweep_cycles
        self.write_json("summary.json", summary)
        return summary

    def run_presteer_mode(self, mode: str, speed_mps: float, steer_deg: float,
                          spin_deg: float, period_ms: int, ramp_sec: float,
                          hold_sec: float, profile_velocity: int,
                          profile_accel: int, fault_reset: bool,
                          tolerance_counts: int,
                          timeout_sec: float) -> dict[str, object]:
        """Run crab/spin with correct sequencing: steer first, then drive.

        The sequence is:
        1. Configure drives and steering axes.
        2. Command zero wheel speed.
        3. Move steering axes to the target geometry.
        4. Poll actual steering position until all selected axes are inside the
           tolerance window.
        5. Ramp wheel speed in, hold, ramp wheel speed out.
        6. Return steering to straight after wheel speed is zero.
        """
        if mode == "spin":
            target = self.build_spin(speed_mps, spin_deg)
        elif mode == "crab":
            target = self.build_crab(speed_mps, steer_deg)
        else:
            raise ValueError("--presteer-mode supports only spin or crab")

        self.setup_nodes(profile_velocity, profile_accel, fault_reset)
        before = self.read_node_state("before.json")
        self.stop_all()
        self.feedback.clear()

        neutral = WheelCommand((0.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0))
        presteer_target = WheelCommand(target.steer_deg, (0.0, 0.0, 0.0, 0.0))
        drive_target = target
        sample = 0

        self.command_steer_only(presteer_target, sample, f"{mode}_presteer_command")
        sample += 1
        sample, presteer_ok = self.wait_for_steer_positions(presteer_target,
                                                            tolerance_counts,
                                                            timeout_sec,
                                                            sample,
                                                            mode,
                                                            period_ms)
        if not presteer_ok:
            self.stop_all()
            raise RuntimeError(f"{mode} presteer target was not reached before drive enable")

        ramp_samples = max(1, int(round(ramp_sec * 1000.0 / float(period_ms))))
        hold_samples = max(1, int(round(hold_sec * 1000.0 / float(period_ms))))
        zero_speed_at_angle = WheelCommand(target.steer_deg, (0.0, 0.0, 0.0, 0.0))
        sample, current = self.execute_samples(zero_speed_at_angle,
                                               drive_target,
                                               ramp_samples,
                                               period_ms,
                                               sample,
                                               f"{mode}_drive_ramp_after_presteer")
        sample, current = self.execute_samples(drive_target,
                                               drive_target,
                                               hold_samples,
                                               period_ms,
                                               sample,
                                               f"{mode}_drive_hold")
        sample, current = self.execute_samples(current,
                                               zero_speed_at_angle,
                                               ramp_samples,
                                               period_ms,
                                               sample,
                                               f"{mode}_drive_ramp_to_zero")
        sample, current = self.execute_samples(zero_speed_at_angle,
                                               neutral,
                                               ramp_samples,
                                               period_ms,
                                               sample,
                                               f"{mode}_steer_return_after_stop")
        self.stop_all()
        time.sleep(0.5)
        self.drain_feedback_nonblocking()
        after = self.read_node_state("after.json")

        summary = self.build_summary(
            speed_mps,
            steer_deg,
            spin_deg,
            period_ms,
            sample,
            [f"{mode}_presteer_then_drive"],
            before,
            after,
        )
        summary["scenario"] = "presteer_then_drive"
        summary["presteer_mode"] = mode
        summary["presteer_tolerance_counts"] = tolerance_counts
        summary["presteer_timeout_sec"] = timeout_sec
        summary["ramp_sec"] = ramp_sec
        summary["hold_sec"] = hold_sec
        self.write_json("summary.json", summary)
        return summary

    def build_summary(self, speed_mps: float, steer_deg: float, spin_deg: float,
                      period_ms: int, samples_per_segment: int, modes: list[str],
                      before: dict[str, object], after: dict[str, object]) -> dict[str, object]:
        emcy_count = sum(1 for item in self.feedback if isinstance(item.get("decoded"), dict) and "emcy" in item["decoded"])
        by_node: dict[str, dict[str, object]] = {}
        for item in self.feedback:
            node = item.get("node")
            decoded = item.get("decoded")
            if not isinstance(node, int) or not isinstance(decoded, dict):
                continue
            entry = by_node.setdefault(
                str(node),
                {
                    "tpdo0": 0,
                    "tpdo1": 0,
                    "max_abs_velocity": 0,
                    "max_abs_actual_mps": 0.0 if node in DRIVE_NODES else None,
                    "last_actual_mps": 0.0 if node in DRIVE_NODES else None,
                    "last_statusword": "",
                },
            )
            if "actual_velocity" in decoded:
                entry["tpdo0"] = int(entry["tpdo0"]) + 1
                velocity = int(decoded["actual_velocity"])
                entry["max_abs_velocity"] = max(int(entry["max_abs_velocity"]), abs(velocity))
                if node in DRIVE_NODES:
                    actual_mps = self.velocity_units_to_mps(velocity, self.drive_sign_from_node(node))
                    entry["max_abs_actual_mps"] = round(max(float(entry["max_abs_actual_mps"]), abs(actual_mps)), 3)
                    entry["last_actual_mps"] = round(actual_mps, 3)
            if "statusword" in decoded:
                entry["tpdo1"] = int(entry["tpdo1"]) + 1
                entry["last_statusword"] = decoded["statusword"]

        return {
            "speed_mps": speed_mps,
            "steer_deg": steer_deg,
            "spin_deg": spin_deg,
            "drive_max_speed_mps": round(DRIVE_MAX_SPEED_MPS, 3),
            "drive_velocity_units_per_mps": round(DRIVE_SPEED_MPS_TO_UNITS, 3),
            "steer_counts_per_deg": round(STEER_DEG_TO_COUNTS, 3),
            "steer_90deg_counts": int(round(90.0 * STEER_DEG_TO_COUNTS)),
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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Suspended-wheel CANopen remote-motion simulator for Node1-8.")
    parser.add_argument("--allow-motion", action="store_true", help="required: confirms wheels are lifted and free")
    parser.add_argument("--active-wheels", default="1,2,3,4",
                        help="1-based wheel list to command; e.g. 1 or 1,2,3,4")
    parser.add_argument("--speed-mps", type=float, default=0.50)
    parser.add_argument("--speed-steps-mps", default="0.10,0.20,0.30,0.50")
    parser.add_argument("--steer-deg", type=float, default=10.0)
    parser.add_argument("--spin-deg", type=float, default=45.0)
    parser.add_argument("--period-ms", type=int, default=50)
    parser.add_argument("--samples-per-segment", type=int, default=60)
    parser.add_argument("--modes", default="ackermann,reverse_ackermann,crab,spin")
    parser.add_argument("--single-wheel-speed-sweep", action="store_true")
    parser.add_argument("--smooth-commissioning-scenario", action="store_true")
    parser.add_argument("--ackermann-remote-sweep", action="store_true")
    parser.add_argument("--presteer-mode", choices=("spin", "crab"), default="")
    parser.add_argument("--ramp-sec", type=float, default=3.0)
    parser.add_argument("--hold-sec", type=float, default=8.0)
    parser.add_argument("--ackermann-sec", type=float, default=20.0)
    parser.add_argument("--spin-sec", type=float, default=15.0)
    parser.add_argument("--crab-sec", type=float, default=15.0)
    parser.add_argument("--sweep-sec", type=float, default=30.0)
    parser.add_argument("--sweep-cycles", type=float, default=4.0)
    parser.add_argument("--presteer-tolerance-counts", type=int, default=DEFAULT_PRESTEER_TOLERANCE_COUNTS)
    parser.add_argument("--presteer-timeout-sec", type=float, default=DEFAULT_PRESTEER_TIMEOUT_SEC)
    parser.add_argument("--profile-velocity", type=int, default=DEFAULT_STEER_PROFILE_VELOCITY)
    parser.add_argument("--profile-accel", type=int, default=SERVO_PROFILE_ACCEL_LIMIT_COUNTS_PER_SEC2)
    parser.add_argument("--timeout-ms", type=int, default=900)
    parser.add_argument("--fault-reset-before-test", action="store_true")
    parser.add_argument("--log-dir", default="")
    return parser.parse_args()


def parse_active_wheels(text: str) -> tuple[int, ...]:
    wheels: list[int] = []
    for item in text.split(","):
        stripped = item.strip()
        if not stripped:
            continue
        value = int(stripped)
        if value < 1 or value > 4:
            raise ValueError("--active-wheels entries must be 1..4")
        wheel = value - 1
        if wheel not in wheels:
            wheels.append(wheel)
    if not wheels:
        raise ValueError("--active-wheels must select at least one wheel")
    return tuple(wheels)


def parse_speed_steps_mps(text: str) -> tuple[float, ...]:
    steps: list[float] = []
    for item in text.split(","):
        stripped = item.strip()
        if not stripped:
            continue
        value = float(stripped)
        if value < 0.0:
            raise ValueError("--speed-steps-mps entries must be non-negative")
        steps.append(value)
    if not steps:
        raise ValueError("--speed-steps-mps must contain at least one value")
    return tuple(steps)


def main() -> int:
    args = parse_args()
    if not args.allow_motion:
        print("FAILED: --allow-motion is required", file=sys.stderr)
        return 1
    active_wheels = parse_active_wheels(args.active_wheels)
    speed_steps_mps = parse_speed_steps_mps(args.speed_steps_mps)
    max_requested_speed = max(abs(args.speed_mps),
                              max((abs(value) for value in speed_steps_mps), default=0.0))
    if max_requested_speed > DRIVE_MAX_SPEED_MPS:
        print(
            f"FAILED: requested {max_requested_speed:.3f} m/s exceeds mechanical max "
            f"{DRIVE_MAX_SPEED_MPS:.3f} m/s from 3000 rpm, 86.6:1 and 580 mm wheel",
            file=sys.stderr,
        )
        return 1
    if args.profile_velocity > SERVO_MAX_VELOCITY_UNITS_FROM_RPM:
        print(
            f"FAILED: profile velocity {args.profile_velocity} exceeds 3000 rpm limit "
            f"{SERVO_MAX_VELOCITY_UNITS_FROM_RPM}",
            file=sys.stderr,
        )
        return 1
    if args.profile_accel > SERVO_PROFILE_ACCEL_LIMIT_COUNTS_PER_SEC2:
        print(
            f"FAILED: profile acceleration {args.profile_accel} exceeds "
            f"{SERVO_COMMISSIONING_MAX_ACCEL_RPS2:.0f} rps^2 commissioning limit "
            f"({SERVO_PROFILE_ACCEL_LIMIT_COUNTS_PER_SEC2} count/s^2)",
            file=sys.stderr,
        )
        return 1
    modes = [item.strip() for item in args.modes.split(",") if item.strip()]
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    log_dir = Path(args.log_dir) if args.log_dir else REPO_ROOT / "out" / f"motion8_remote_sim_{timestamp}"
    try:
        with Motion8Debug(args.timeout_ms,
                          log_dir,
                          active_wheels=active_wheels,
                          steering_enabled=not args.single_wheel_speed_sweep) as debug:
            if args.single_wheel_speed_sweep:
                summary = debug.run_single_wheel_speed_sweep(
                    speed_steps_mps,
                    args.period_ms,
                    args.ramp_sec,
                    args.hold_sec,
                    args.profile_velocity,
                    args.profile_accel,
                    args.fault_reset_before_test,
                )
            elif args.smooth_commissioning_scenario:
                summary = debug.run_smooth_commissioning_scenario(
                    args.speed_mps,
                    args.steer_deg,
                    args.spin_deg,
                    args.period_ms,
                    args.ramp_sec,
                    args.ackermann_sec,
                    args.spin_sec,
                    args.crab_sec,
                    args.profile_velocity,
                    args.profile_accel,
                    args.fault_reset_before_test,
                )
            elif args.ackermann_remote_sweep:
                summary = debug.run_ackermann_remote_sweep(
                    args.speed_mps,
                    args.steer_deg,
                    args.period_ms,
                    args.ramp_sec,
                    args.sweep_sec,
                    args.sweep_cycles,
                    args.profile_velocity,
                    args.profile_accel,
                    args.fault_reset_before_test,
                )
            elif args.presteer_mode:
                summary = debug.run_presteer_mode(
                    args.presteer_mode,
                    args.speed_mps,
                    args.steer_deg,
                    args.spin_deg,
                    args.period_ms,
                    args.ramp_sec,
                    args.hold_sec,
                    args.profile_velocity,
                    args.profile_accel,
                    args.fault_reset_before_test,
                    args.presteer_tolerance_counts,
                    args.presteer_timeout_sec,
                )
            else:
                summary = debug.run_plan(
                    args.speed_mps,
                    args.steer_deg,
                    args.spin_deg,
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
