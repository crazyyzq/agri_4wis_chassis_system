"""CAN3 four-lift velocity closed-loop commissioning tool.

Use only while the ECU CAN3 transceiver is disconnected. The CAN analyzer owns
CAN3 and sends one RPDO0 velocity command to Nodes 9, 11, 12 and 10 followed by
one SYNC every 20 ms.

This tool is intentionally not a production controller. It is a field-debug
tool for proving a recoverable four-axis height strategy before the same
state-machine idea is migrated into the ECU CAN3 task.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools" / "can"))
from controlcan import ControlCAN, VCI_CAN_OBJ  # noqa: E402


NODES = (9, 11, 12, 10)
CHANNEL = 1
BITRATE = 1_000_000
DEFAULT_ENCODER_COUNTS_PER_REV = 131_072
DEFAULT_MOTOR_REVS_PER_MM = 12 / 10
COUNTS_PER_MM = DEFAULT_ENCODER_COUNTS_PER_REV * DEFAULT_MOTOR_REVS_PER_MM
TARGET_VELOCITY_OBJECT_SCALE = 10
SYNC_COB_ID = 0x080
MODE_PROFILE_VELOCITY = 3


@dataclass
class Axis:
    position: int = 0
    velocity: int = 0
    current: int = 0
    fault: int = 0
    peak_current: int = 0
    last_command: int = 0


@dataclass
class MoveReport:
    target_mm: float
    start_mm: dict[str, float]
    final_mm: dict[str, float] = field(default_factory=dict)
    max_spread_mm: float = 0.0
    max_error_mm: float = 0.0
    peak_current_10ma: dict[str, int] = field(default_factory=dict)
    recovery_entries: int = 0
    fault_recoveries: int = 0
    node_resets: int = 0
    adaptive_speed_scale: float = 1.0
    duration_s: float = 0.0
    samples: list[dict[str, object]] = field(default_factory=list)


@dataclass(frozen=True)
class LiftScale:
    encoder_counts_per_rev: float
    motor_revs_per_mm: float

    @property
    def counts_per_mm(self) -> float:
        return self.encoder_counts_per_rev * self.motor_revs_per_mm

    @property
    def mm_per_motor_rev(self) -> float:
        return 1.0 / self.motor_revs_per_mm

    def counts_to_mm(self, counts: int) -> float:
        return -counts / self.counts_per_mm

    def mm_to_counts(self, mm: float) -> int:
        return -round(mm * self.counts_per_mm)


def frame(can_id: int, data: bytes) -> VCI_CAN_OBJ:
    item = VCI_CAN_OBJ()
    item.ID = can_id
    item.SendType = 0
    item.RemoteFlag = 0
    item.ExternFlag = 0
    item.DataLen = len(data)
    for index, value in enumerate(data):
        item.Data[index] = value
    return item


def counts_to_mm(counts: int) -> float:
    return -counts / COUNTS_PER_MM


def mm_to_counts(mm: float) -> int:
    return -round(mm * COUNTS_PER_MM)


def clamp(value: int, limit: int) -> int:
    if value > limit:
        return limit
    if value < -limit:
        return -limit
    return value


class LiftVelocitySync:
    def __init__(self, timeout_ms: int, scale: LiftScale | None = None) -> None:
        self.can = ControlCAN()
        self.timeout_ms = timeout_ms
        self.scale = scale or LiftScale(
            DEFAULT_ENCODER_COUNTS_PER_REV,
            DEFAULT_MOTOR_REVS_PER_MM,
        )
        self.axis = {node: Axis() for node in NODES}
        self.emcy: list[str] = []
        self.node_reset_count = 0

    def __enter__(self) -> "LiftVelocitySync":
        self.can.open()
        self.can.init_can(CHANNEL, BITRATE)
        return self

    def __exit__(self, *_: object) -> None:
        try:
            self.stop_disable()
        finally:
            self.can.close()

    def send(self, can_id: int, data: bytes) -> None:
        if self.can.transmit_frames(CHANNEL, [frame(can_id, data)]) != 1:
            raise RuntimeError(f"CAN TX failed 0x{can_id:03X}")

    def read_sdo(self, node: int, index: int, subindex: int = 0) -> int:
        self.send(
            0x600 + node,
            bytes([0x40, index & 0xFF, index >> 8, subindex, 0, 0, 0, 0]),
        )
        end = time.monotonic() + self.timeout_ms / 1000
        while time.monotonic() < end:
            for raw in self.can.receive(CHANNEL, 200, 20):
                if int(raw.ID) != 0x580 + node:
                    self._cache_feedback(raw)
                    continue
                data = bytes(int(raw.Data[i]) for i in range(int(raw.DataLen)))
                if data[0] == 0x80:
                    abort = int.from_bytes(data[4:8], "little")
                    raise RuntimeError(
                        f"SDO abort Node{node} 0x{index:04X}:{subindex} "
                        f"code=0x{abort:08X}"
                    )
                size = {0x4F: 1, 0x4B: 2, 0x43: 4}.get(data[0])
                if size is None:
                    raise RuntimeError(f"bad SDO reply Node{node}")
                return int.from_bytes(data[4:4 + size], "little", signed=True)
        raise TimeoutError(f"SDO timeout Node{node} 0x{index:04X}:{subindex}")

    def write_sdo(
        self,
        node: int,
        index: int,
        size: int,
        value: int,
        subindex: int = 0,
    ) -> None:
        command = {1: 0x2F, 2: 0x2B, 4: 0x23}[size]
        payload = (
            bytes([command, index & 0xFF, index >> 8, subindex])
            + int(value).to_bytes(size, "little", signed=value < 0)
            + bytes(4 - size)
        )
        self.send(0x600 + node, payload)
        end = time.monotonic() + self.timeout_ms / 1000
        while time.monotonic() < end:
            for raw in self.can.receive(CHANNEL, 200, 20):
                if int(raw.ID) != 0x580 + node:
                    self._cache_feedback(raw)
                    continue
                data = bytes(int(raw.Data[i]) for i in range(int(raw.DataLen)))
                if data[0] == 0x60:
                    return
                if data[0] == 0x80:
                    abort = int.from_bytes(data[4:8], "little")
                    raise RuntimeError(
                        f"SDO write abort Node{node} 0x{index:04X}:{subindex} "
                        f"code=0x{abort:08X}"
                    )
                raise RuntimeError(f"SDO write failed Node{node} 0x{index:04X}")
        raise TimeoutError(f"SDO write timeout Node{node} 0x{index:04X}")

    def _cache_feedback(self, raw: object) -> None:
        can_id = int(raw.ID)
        data = bytes(int(raw.Data[i]) for i in range(int(raw.DataLen)))
        node = can_id & 0x7F
        if node not in NODES:
            return
        axis = self.axis[node]
        if can_id == 0x180 + node and len(data) == 8:
            axis.position = int.from_bytes(data[:4], "little", signed=True)
            axis.velocity = int.from_bytes(data[4:], "little", signed=True)
        elif can_id == 0x280 + node and len(data) == 8:
            axis.fault = int.from_bytes(data[:4], "little")
            axis.current = int.from_bytes(data[6:8], "little", signed=True)
            axis.peak_current = max(axis.peak_current, abs(axis.current))
        elif can_id == 0x080 + node and len(data) >= 2:
            code = int.from_bytes(data[:2], "little")
            if code != 0:
                self.emcy.append(f"Node{node} 0x{code:04X}")

    def poll(self, wait_ms: int) -> None:
        for raw in self.can.receive(CHANNEL, 400, wait_ms):
            self._cache_feedback(raw)

    def configure(self) -> None:
        for node in NODES:
            self.configure_node(node)

    def configure_node(self, node: int) -> None:
        self.send(0x000, bytes([0x01, node]))
        time.sleep(0.02)
        fault = self.read_sdo(node, 0x2183)
        if fault != 0:
            self.clear_fault(node, fault)
        self.write_sdo(node, 0x6040, 2, 0x0006)
        self.write_sdo(node, 0x2300, 2, 0x001E)
        self.write_sdo(node, 0x6060, 1, MODE_PROFILE_VELOCITY)
        self.write_sdo(node, 0x6040, 2, 0x0007)
        self.write_sdo(node, 0x6040, 2, 0x000F)
        self.axis[node].position = self.read_sdo(node, 0x6064)
        self.axis[node].fault = self.read_sdo(node, 0x2183)

    def clear_fault(self, node: int, fault: int | None = None) -> None:
        if fault is None:
            fault = self.read_sdo(node, 0x2183)
        if fault != 0:
            self.write_sdo(node, 0x2183, 4, fault)
            time.sleep(0.05)
        self.write_sdo(node, 0x6040, 2, 0x0080)
        time.sleep(0.05)
        self.write_sdo(node, 0x6040, 2, 0x0006)
        self.write_sdo(node, 0x6040, 2, 0x0007)
        self.write_sdo(node, 0x6040, 2, 0x000F)
        self.axis[node].fault = self.read_sdo(node, 0x2183)
        status = self.read_sdo(node, 0x6041) & 0xFFFF
        if self.axis[node].fault != 0 or (status & 0x0008):
            self.reset_node(node)

    def reset_node(self, node: int) -> None:
        # Lift axes use absolute encoders, so a single-node reset is an
        # acceptable last recovery step when a latched drive fault cannot be
        # cleared through 0x2183 and CiA-402 fault reset. Do not reuse this
        # behavior for steering axes, where reset can lose the relative zero.
        position_before = self.read_sdo(node, 0x6064)
        self.send(0x000, bytes([0x81, node]))
        time.sleep(2.0)
        self.configure_node(node)
        position_after = self.read_sdo(node, 0x6064)
        if abs(position_after - position_before) > round(0.5 * self.scale.counts_per_mm):
            raise RuntimeError(
                f"Node{node} reset changed absolute position by "
                f"{(position_after - position_before) / self.scale.counts_per_mm:.3f}mm"
            )
        self.node_reset_count += 1

    def velocity_group(self, values: dict[int, int]) -> None:
        for node in NODES:
            self.axis[node].last_command = values[node]
            payload = (
                (0x000F).to_bytes(2, "little")
                + bytes([MODE_PROFILE_VELOCITY])
                + int(values[node]).to_bytes(4, "little", signed=True)
            )
            self.send(0x200 + node, payload)
        self.send(SYNC_COB_ID, b"")

    def stop_disable(self) -> None:
        try:
            zeros = {node: 0 for node in NODES}
            for _ in range(5):
                self.velocity_group(zeros)
                time.sleep(0.02)
            time.sleep(0.2)
            for node in NODES:
                self.send(
                    0x200 + node,
                    (0).to_bytes(2, "little")
                    + bytes([MODE_PROFILE_VELOCITY, 0, 0, 0, 0]),
                )
            self.send(SYNC_COB_ID, b"")
        except Exception:
            pass

    def recover_velocity_mode(self, reason: str) -> None:
        print(f"[recover] {reason}: zero speed, disable, reconfigure", flush=True)
        self.stop_disable()
        time.sleep(0.2)
        self.configure()

    def snapshot_mm(self) -> dict[str, float]:
        return {
            str(node): round(self.scale.counts_to_mm(self.axis[node].position), 3)
            for node in NODES
        }

    def move_to(
        self,
        target_mm: float,
        speed_mm_s: float,
        accel_mm_s2: float,
        kp_target: float,
        kp_level: float,
        soft_current_10ma: int,
        recovery_speed_mm_s: float,
        max_runtime_s: float,
        print_period_s: float,
        target_tolerance_mm: float,
        sync_tolerance_mm: float,
        stall_time_s: float,
        stall_min_speed_mm_s: float,
        stall_min_progress_mm: float,
        mode_check_period_s: float,
        max_recoveries: int,
    ) -> MoveReport:
        if not 10 <= target_mm <= 490:
            raise ValueError("target must be 10..490 mm")

        for node in NODES:
            self.configure_node(node)

        target = self.scale.mm_to_counts(target_mm)
        period_s = 0.02
        # BC/BC2 profile-velocity target object 0x60FF uses 0.1 count/s units.
        # A requested mechanical speed in mm/s must therefore be converted to
        # physical count/s and then multiplied by ten before transmission.
        max_speed = round(
            speed_mm_s *
            self.scale.counts_per_mm *
            TARGET_VELOCITY_OBJECT_SCALE
        )
        max_step = round(
            accel_mm_s2 * self.scale.counts_per_mm *
            TARGET_VELOCITY_OBJECT_SCALE *
            period_s
        )
        recovery_speed = round(
            recovery_speed_mm_s *
            self.scale.counts_per_mm *
            TARGET_VELOCITY_OBJECT_SCALE
        )
        target_tolerance = round(target_tolerance_mm * self.scale.counts_per_mm)
        sync_tolerance = round(sync_tolerance_mm * self.scale.counts_per_mm)
        recovery_enter = sync_tolerance
        recovery_exit = sync_tolerance
        recovery_abort = round(30.0 * self.scale.counts_per_mm)
        stall_command_threshold = round(
            stall_min_speed_mm_s *
            self.scale.counts_per_mm *
            TARGET_VELOCITY_OBJECT_SCALE
        )
        stall_progress_threshold = round(stall_min_progress_mm * self.scale.counts_per_mm)

        report = MoveReport(target_mm=target_mm, start_mm=self.snapshot_mm())
        stable = 0
        recovery = False
        adaptive_scale = 1.0
        last_print = 0.0
        started = time.monotonic()
        deadline = started + max_runtime_s
        last_progress_time = {node: started for node in NODES}
        last_progress_position = {node: self.axis[node].position for node in NODES}
        last_mode_check = 0.0

        while time.monotonic() < deadline:
            tick_start = time.perf_counter()
            positions = {node: self.axis[node].position for node in NODES}
            spread = max(positions.values()) - min(positions.values())
            report.max_spread_mm = max(
                report.max_spread_mm,
                round(spread / self.scale.counts_per_mm, 3),
            )

            if spread > recovery_abort:
                raise RuntimeError(
                    "unrecoverable sync spread="
                    f"{spread / self.scale.counts_per_mm:.3f}mm"
                )
            if not recovery and spread > recovery_enter:
                recovery = True
                report.recovery_entries += 1
            if recovery and spread <= recovery_exit:
                recovery = False

            values: dict[int, int] = {}
            level_ref = round(sum(positions.values()) / len(NODES))
            for node in NODES:
                axis = self.axis[node]
                main_speed = round(
                    kp_target *
                    (target - axis.position) *
                    TARGET_VELOCITY_OBJECT_SCALE
                )
                level_speed = round(
                    kp_level *
                    (level_ref - axis.position) *
                    TARGET_VELOCITY_OBJECT_SCALE
                )
                if recovery:
                    desired = level_speed
                    limit = recovery_speed
                else:
                    desired = main_speed + level_speed
                    limit = round(max_speed * adaptive_scale)
                desired = clamp(desired, limit)
                command = max(
                    axis.last_command - max_step,
                    min(axis.last_command + max_step, desired),
                )
                if abs(axis.current) >= soft_current_10ma:
                    command = int(command * 0.5)
                values[node] = command

            self.velocity_group(values)
            self.poll(5)

            now = time.monotonic()
            if now - last_mode_check >= mode_check_period_s:
                bad_modes: dict[int, int] = {}
                for node in NODES:
                    mode = self.read_sdo(node, 0x6061)
                    if mode != MODE_PROFILE_VELOCITY:
                        bad_modes[node] = mode
                last_mode_check = now
                if bad_modes:
                    report.fault_recoveries += 1
                    if report.fault_recoveries > max_recoveries:
                        self.stop_disable()
                        raise RuntimeError(
                            f"velocity mode lost repeatedly: {bad_modes}"
                        )
                    self.recover_velocity_mode(f"velocity mode lost {bad_modes}")
                    last_progress_time = {node: time.monotonic() for node in NODES}
                    last_progress_position = {
                        node: self.axis[node].position for node in NODES
                    }
                    stable = 0
                    continue

            recovered_this_tick = False
            for node in NODES:
                axis = self.axis[node]
                if abs(axis.position - last_progress_position[node]) >= stall_progress_threshold:
                    last_progress_position[node] = axis.position
                    last_progress_time[node] = now
                elif (
                    abs(axis.last_command) >= stall_command_threshold and
                    now - last_progress_time[node] >= stall_time_s
                ):
                    report.fault_recoveries += 1
                    if report.fault_recoveries > max_recoveries:
                        self.stop_disable()
                        raise RuntimeError(
                            f"Node{node} stalled repeatedly while command="
                            f"{axis.last_command} target={target_mm:.1f}mm "
                            f"pos={self.scale.counts_to_mm(axis.position):.3f}mm"
                        )
                    self.recover_velocity_mode(
                        f"Node{node} stalled target={target_mm:.1f}mm "
                        f"pos={self.scale.counts_to_mm(axis.position):.3f}mm"
                    )
                    last_progress_time = {node: time.monotonic() for node in NODES}
                    last_progress_position = {
                        node: self.axis[node].position for node in NODES
                    }
                    stable = 0
                    recovered_this_tick = True
                    break
            if recovered_this_tick:
                continue

            if self.emcy or any(axis.fault for axis in self.axis.values()):
                report.fault_recoveries += 1
                fault_snapshot = {
                    node: self.axis[node].fault for node in NODES
                    if self.axis[node].fault != 0
                }
                self.stop_disable()
                adaptive_scale = max(0.35, adaptive_scale * 0.7)
                if fault_snapshot and report.fault_recoveries > max_recoveries:
                    raise RuntimeError(
                        f"drive fault could not recover emcy={self.emcy} "
                        f"fault={fault_snapshot}"
                    )
                self.emcy.clear()
                for node in NODES:
                    if self.axis[node].fault != 0:
                        before_fault = self.axis[node].fault
                        self.clear_fault(node, self.axis[node].fault)
                        if before_fault != 0 and self.axis[node].fault == 0:
                            # Count both normal fault-reset recoveries and
                            # reset-backed recoveries in the same report field.
                            pass
                if not fault_snapshot:
                    # Current-limit EMCY may arrive before a latching 0x2183
                    # fault.  Give the drive one fresh TPDO/SDO observation
                    # cycle before deciding whether a real fault was latched.
                    time.sleep(0.2)
                    self.poll(20)
                report.node_resets = self.node_reset_count
                report.adaptive_speed_scale = round(adaptive_scale, 3)
                self.recover_velocity_mode("fault/emcy recovery")
                last_progress_time = {node: time.monotonic() for node in NODES}
                last_progress_position = {
                    node: self.axis[node].position for node in NODES
                }
                continue

            errors = {
                node: abs(target - self.axis[node].position) for node in NODES
            }
            report.max_error_mm = round(
                max(errors.values()) / self.scale.counts_per_mm,
                3,
            )
            if (
                max(errors.values()) <= target_tolerance and
                spread <= sync_tolerance
            ):
                stable += 1
                if stable >= 10:
                    report.duration_s = round(time.monotonic() - started, 3)
                    report.final_mm = self.snapshot_mm()
                    report.peak_current_10ma = {
                        str(node): self.axis[node].peak_current for node in NODES
                    }
                    report.node_resets = self.node_reset_count
                    report.adaptive_speed_scale = round(adaptive_scale, 3)
                    self.stop_disable()
                    return report
            else:
                stable = 0

            if now - last_print >= print_period_s:
                sample = {
                    "time_s": round(now - started, 2),
                    "state": "leveling" if recovery else "tracking",
                    "target_mm": target_mm,
                    "position_mm": self.snapshot_mm(),
                    "spread_mm": round(spread / self.scale.counts_per_mm, 3),
                    "command_counts_s": {
                        str(node): self.axis[node].last_command for node in NODES
                    },
                    "feedback_velocity_counts_s": {
                        str(node): self.axis[node].velocity for node in NODES
                    },
                    "current_10ma": {
                        str(node): self.axis[node].current for node in NODES
                    },
                    "fault": {
                        str(node): f"0x{self.axis[node].fault:08X}"
                        for node in NODES
                    },
                }
                report.samples.append(sample)
                print(
                    "t={time_s:.1f}s state={state} target={target_mm:.1f}mm "
                    "pos={position_mm} spread={spread_mm:.3f}mm "
                    "cur={current_10ma}".format(**sample),
                    flush=True,
                )
                last_print = now

            remaining = period_s - (time.perf_counter() - tick_start)
            if remaining > 0:
                time.sleep(remaining)

        report.duration_s = round(time.monotonic() - started, 3)
        report.final_mm = self.snapshot_mm()
        report.peak_current_10ma = {
            str(node): self.axis[node].peak_current for node in NODES
        }
        report.node_resets = self.node_reset_count
        report.adaptive_speed_scale = round(adaptive_scale, 3)
        raise TimeoutError(
            f"velocity closed-loop timeout target={target_mm:.1f}mm "
            f"final={report.final_mm} max_spread={report.max_spread_mm:.3f}mm"
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--target-mm", type=float, default=None)
    parser.add_argument("--short-mm", type=float, default=10.0)
    parser.add_argument("--long-mm", type=float, default=490.0)
    parser.add_argument("--cycles", type=int, default=0)
    parser.add_argument("--speed-mm-s", type=float, default=15.0)
    parser.add_argument("--accel-mm-s2", type=float, default=20.0)
    parser.add_argument("--kp-target", type=float, default=2.0)
    parser.add_argument("--kp-level", type=float, default=2.5)
    parser.add_argument("--recovery-speed-mm-s", type=float, default=5.0)
    parser.add_argument("--soft-current-10ma", type=int, default=1300)
    parser.add_argument("--timeout-ms", type=int, default=1000)
    parser.add_argument("--max-runtime-s", type=float, default=240.0)
    parser.add_argument("--print-period-s", type=float, default=1.0)
    parser.add_argument("--target-tolerance-mm", type=float, default=3.0)
    parser.add_argument("--sync-tolerance-mm", type=float, default=3.0)
    parser.add_argument("--stall-time-s", type=float, default=1.5)
    parser.add_argument("--stall-min-speed-mm-s", type=float, default=1.0)
    parser.add_argument("--stall-min-progress-mm", type=float, default=0.25)
    parser.add_argument("--mode-check-period-s", type=float, default=0.5)
    parser.add_argument("--max-recoveries", type=int, default=5)
    parser.add_argument(
        "--encoder-counts-per-rev",
        type=float,
        default=DEFAULT_ENCODER_COUNTS_PER_REV,
    )
    parser.add_argument(
        "--motor-revs-per-mm",
        type=float,
        default=DEFAULT_MOTOR_REVS_PER_MM,
        help="mechanical calibration from motor revolutions to linear travel",
    )
    parser.add_argument(
        "--mm-per-motor-rev",
        type=float,
        default=0.0,
        help="alternative calibration; overrides --motor-revs-per-mm when >0",
    )
    parser.add_argument("--log", default="")
    parser.add_argument("--allow-motion", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.allow_motion:
        print("FAILED: --allow-motion required", file=sys.stderr)
        return 2
    if (
        args.speed_mm_s <= 0
        or args.accel_mm_s2 <= 0
        or args.kp_target <= 0
        or args.kp_level <= 0
        or args.recovery_speed_mm_s <= 0
        or args.soft_current_10ma <= 0
        or args.max_runtime_s <= 0
        or args.target_tolerance_mm <= 0
        or args.sync_tolerance_mm <= 0
        or args.stall_time_s <= 0
        or args.stall_min_speed_mm_s <= 0
        or args.stall_min_progress_mm <= 0
        or args.mode_check_period_s <= 0
        or args.max_recoveries < 0
        or args.cycles < 0
        or args.encoder_counts_per_rev <= 0
        or args.motor_revs_per_mm <= 0
        or args.mm_per_motor_rev < 0
    ):
        print("FAILED: invalid motion parameters", file=sys.stderr)
        return 2
    motor_revs_per_mm = (
        1.0 / args.mm_per_motor_rev
        if args.mm_per_motor_rev > 0
        else args.motor_revs_per_mm
    )
    scale = LiftScale(args.encoder_counts_per_rev, motor_revs_per_mm)

    targets = (
        [args.target_mm]
        if args.target_mm is not None
        else [args.short_mm] + [target for _ in range(args.cycles)
                                for target in (args.long_mm, args.short_mm)]
    )
    if any(target is None or target < 10 or target > 490 for target in targets):
        print("FAILED: targets must be inside 10..490 mm", file=sys.stderr)
        return 2

    log_path = (
        Path(args.log)
        if args.log
        else ROOT / "tmp" /
             f"lift_velocity_sync_{time.strftime('%Y%m%d_%H%M%S')}.json"
    )
    report: dict[str, object] = {
        "targets_mm": targets,
        "speed_mm_s": args.speed_mm_s,
        "accel_mm_s2": args.accel_mm_s2,
        "kp_target": args.kp_target,
        "kp_level": args.kp_level,
        "recovery_speed_mm_s": args.recovery_speed_mm_s,
        "soft_current_10ma": args.soft_current_10ma,
        "target_tolerance_mm": args.target_tolerance_mm,
        "sync_tolerance_mm": args.sync_tolerance_mm,
        "stall_time_s": args.stall_time_s,
        "stall_min_speed_mm_s": args.stall_min_speed_mm_s,
        "stall_min_progress_mm": args.stall_min_progress_mm,
        "mode_check_period_s": args.mode_check_period_s,
        "max_recoveries": args.max_recoveries,
        "encoder_counts_per_rev": scale.encoder_counts_per_rev,
        "motor_revs_per_mm": scale.motor_revs_per_mm,
        "mm_per_motor_rev": scale.mm_per_motor_rev,
        "counts_per_mm": scale.counts_per_mm,
        "moves": [],
    }

    try:
        with LiftVelocitySync(args.timeout_ms, scale) as tool:
            tool.configure()
            for target in targets:
                move = tool.move_to(
                    float(target),
                    args.speed_mm_s,
                    args.accel_mm_s2,
                    args.kp_target,
                    args.kp_level,
                    args.soft_current_10ma,
                    args.recovery_speed_mm_s,
                    args.max_runtime_s,
                    args.print_period_s,
                    args.target_tolerance_mm,
                    args.sync_tolerance_mm,
                    args.stall_time_s,
                    args.stall_min_speed_mm_s,
                    args.stall_min_progress_mm,
                    args.mode_check_period_s,
                    args.max_recoveries,
                )
                report["moves"].append(move.__dict__)
        log_path.parent.mkdir(parents=True, exist_ok=True)
        log_path.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        print(json.dumps({k: v for k, v in report.items() if k != "moves"},
                         ensure_ascii=False, indent=2))
        print(f"Log written to {log_path}")
        print("PASS")
        return 0
    except Exception as exc:  # noqa: BLE001
        report["error"] = str(exc)
        try:
            log_path.parent.mkdir(parents=True, exist_ok=True)
            log_path.write_text(
                json.dumps(report, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            print(f"Log written to {log_path}", file=sys.stderr)
        except Exception:
            pass
        print(f"FAILED: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
