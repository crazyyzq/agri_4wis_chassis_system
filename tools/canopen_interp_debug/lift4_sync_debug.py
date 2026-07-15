"""CAN3 four-lift synchronous interpolation commissioning tool.

The ECU CAN3 transceiver must be physically disconnected before this tool is
used.  The analyzer CAN2 channel owns the bus during the test.  The tool never
resets a drive and never writes nonvolatile memory.

Each realtime cycle sends one absolute 0x60C1:01 RPDO2 point to Node9, Node11,
Node12 and Node10, then sends one SYNC.  Feedback is sampled from TPDO0/TPDO1
after that common SYNC so relative travel and synchronization error can be
measured from one coherent command sequence.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path

THIS_FILE = Path(__file__).resolve()
REPO_ROOT = THIS_FILE.parents[2]
CAN_TOOL_DIR = REPO_ROOT / "tools" / "can"
if str(CAN_TOOL_DIR) not in sys.path:
    sys.path.insert(0, str(CAN_TOOL_DIR))

from controlcan import ControlCAN, VCI_CAN_OBJ  # noqa: E402


CAN_CHANNEL = 1
CAN_BITRATE = 1_000_000
LIFT_NODES = (9, 11, 12, 10)
SYNC_COB_ID = 0x080
NMT_COB_ID = 0x000
RPDO2_BASE = 0x400
RPDO1_BASE = 0x300
TPDO0_BASE = 0x180
TPDO1_BASE = 0x280
EMCY_BASE = 0x080
LIFT_COUNTS_PER_MOTOR_REV = 131_072
LIFT_MOTOR_REVS_PER_MM = 20.0 / 10.0
LIFT_COUNTS_PER_MM = LIFT_COUNTS_PER_MOTOR_REV * LIFT_MOTOR_REVS_PER_MM
# Installed calibration: 490 mm * 20 rev / 10 mm * 131072 count / rev.
# Keep the commissioning limit equal to the configured normal full stroke.
# Callers must not add an overshoot margin beyond this absolute endpoint.
LIFT_MIN_POSITION_COUNTS = -128_450_560
LIFT_MAX_POSITION_COUNTS = 10_000
LIFT_MAX_SYNC_SPREAD_MM = 3.0
LIFT_LIMIT_MARGIN_COUNTS = round(LIFT_MAX_SYNC_SPREAD_MM * LIFT_COUNTS_PER_MM)
# Vendor units are intentionally different:
#   0x6081 profile velocity     = 0.1 count/s per object unit
#   0x6083/0x6084 acceleration = 10 count/s^2 per object unit
# Keeping separate conversion factors prevents a 100x acceleration error.
CANOPEN_PROFILE_VELOCITY_SCALE = 10.0
CANOPEN_PROFILE_ACCELERATION_SCALE = 0.1


@dataclass
class AxisFeedback:
    position: int = 0
    velocity: int = 0
    statusword: int = 0
    fault: int = 0
    current: int = 0
    peak_abs_current: int = 0
    tpdo0_count: int = 0
    tpdo1_count: int = 0


class Lift4SyncDebug:
    def __init__(self, nodes: tuple[int, ...], timeout_ms: int) -> None:
        self.nodes = nodes
        self.timeout_ms = timeout_ms
        self.can = ControlCAN()
        self.rx_cache: list[tuple[int, bytes]] = []
        self.feedback = {node: AxisFeedback() for node in nodes}
        self.buffer_status_after_clear: dict[int, int] = {}
        self.emcy: list[dict[str, object]] = []
        self.emcy_reset_count = 0
        self.current_limit_emcy_count = 0
        self.current_limit_emcy_by_node = {node: 0 for node in nodes}
        self.samples: list[dict[str, object]] = []

    def __enter__(self) -> "Lift4SyncDebug":
        self.can.open()
        self.can.init_can(CAN_CHANNEL, CAN_BITRATE)
        return self

    def __exit__(self, exc_type, exc, tb) -> None:  # type: ignore[no-untyped-def]
        try:
            self.stop_interpolation()
        finally:
            try:
                self.emergency_disable()
            finally:
                self.can.close()

    def send(self, can_id: int, data: bytes) -> None:
        frame = VCI_CAN_OBJ()
        frame.ID = can_id
        frame.SendType = 0
        frame.RemoteFlag = 0
        frame.ExternFlag = 0
        frame.DataLen = len(data)
        for index, value in enumerate(data):
            frame.Data[index] = value
        if self.can.transmit_frames(CAN_CHANNEL, [frame]) != 1:
            raise RuntimeError(f"CAN transmit failed id=0x{can_id:03X}")

    def receive_raw(self, wait_ms: int) -> list[tuple[int, bytes]]:
        result: list[tuple[int, bytes]] = []
        for raw in self.can.receive(CAN_CHANNEL, limit=200, wait_ms=wait_ms):
            result.append(
                (
                    int(raw.ID),
                    bytes(int(raw.Data[i]) for i in range(int(raw.DataLen))),
                )
            )
        return result

    def receive_expected(self, can_id: int) -> bytes:
        for index, (cached_id, data) in enumerate(self.rx_cache):
            if cached_id == can_id:
                self.rx_cache.pop(index)
                return data
        deadline = time.monotonic() + self.timeout_ms / 1000.0
        while time.monotonic() < deadline:
            for received_id, data in self.receive_raw(20):
                if received_id == can_id:
                    return data
                self.rx_cache.append((received_id, data))
        raise TimeoutError(f"timeout waiting for 0x{can_id:03X}")

    @staticmethod
    def sdo_payload(index: int, subindex: int, size: int, value: int,
                    signed: bool = False) -> bytes:
        command = {1: 0x2F, 2: 0x2B, 4: 0x23}[size]
        payload = bytearray(8)
        payload[0:4] = bytes(
            [command, index & 0xFF, (index >> 8) & 0xFF, subindex]
        )
        payload[4:4 + size] = int(value).to_bytes(size, "little", signed=signed)
        return bytes(payload)

    def sdo_write(self, node: int, index: int, subindex: int, size: int,
                  value: int, signed: bool = False) -> None:
        self.send(
            0x600 + node,
            self.sdo_payload(index, subindex, size, value, signed=signed),
        )
        response = self.receive_expected(0x580 + node)
        if len(response) != 8:
            raise RuntimeError(f"short SDO response Node{node}")
        if response[0] == 0x80:
            abort = int.from_bytes(response[4:8], "little")
            raise RuntimeError(
                f"SDO abort Node{node} 0x{index:04X}:{subindex} "
                f"code=0x{abort:08X}"
            )
        expected = bytes(
            [0x60, index & 0xFF, (index >> 8) & 0xFF, subindex, 0, 0, 0, 0]
        )
        if response != expected:
            raise RuntimeError(
                f"unexpected SDO response Node{node}: {response.hex(' ')}"
            )

    def clear_interpolation_buffer(self, node: int) -> None:
        # BC2 exposes the interpolation buffer-clear command through
        # 0x60C4:06 (UNSIGNED8).  Its firmware rejects SDO access to the
        # alternate 0x2010 object even though 0x2011/0x2012 are readable.
        #
        # 0x2012 contains live buffer state plus sticky sequence/underflow
        # history.  Those history bits are not cleared consistently by
        # 0x60C4:06, so they must not be used as proof that stale trajectory
        # points remain.  New EMCY frames, TPDO status and trajectory
        # consistency are checked during the run instead.
        self.sdo_write(node, 0x60C4, 0x06, 1, 0)
        buffer_status, _ = self.sdo_read(node, 0x2012, 0x00)
        self.buffer_status_after_clear[node] = buffer_status

    def sdo_read(self, node: int, index: int, subindex: int) -> tuple[int, int]:
        self.send(
            0x600 + node,
            bytes([0x40, index & 0xFF, (index >> 8) & 0xFF, subindex, 0, 0, 0, 0]),
        )
        response = self.receive_expected(0x580 + node)
        if len(response) != 8:
            raise RuntimeError(f"short SDO response Node{node}")
        if response[0] == 0x80:
            abort = int.from_bytes(response[4:8], "little")
            raise RuntimeError(
                f"SDO abort Node{node} 0x{index:04X}:{subindex} "
                f"code=0x{abort:08X}"
            )
        size_by_command = {0x4F: 1, 0x4B: 2, 0x43: 4}
        size = size_by_command.get(response[0])
        if size is None:
            raise RuntimeError(f"unsupported SDO response 0x{response[0]:02X}")
        return (
            int.from_bytes(response[4:4 + size], "little", signed=False),
            int.from_bytes(response[4:4 + size], "little", signed=True),
        )

    def configure_axes(self, period_ms: int, enable_settle_ms: int,
                       profile_velocity_units: int,
                       profile_acceleration: int,
                       tracking_window_counts: int) -> dict[int, int]:
        start_positions: dict[int, int] = {}
        for node in self.nodes:
            self.send(NMT_COB_ID, bytes([0x01, node]))
            time.sleep(0.01)
            latched_fault, _ = self.sdo_read(node, 0x2183, 0x00)
            if latched_fault != 0:
                self.sdo_write(
                    node, 0x2183, 0x00, 4, latched_fault
                )
                time.sleep(0.1)
            # BC2 only discards every queued segment after leaving Operation
            # Enabled.  Clearing at 0x000F can acknowledge the SDO while stale
            # points remain.  0x0006 preserves the absolute position reference;
            # this is a CiA-402 state transition, not a drive reset.
            self.sdo_write(node, 0x6040, 0x00, 2, 0x0006)
            self.sdo_write(node, 0x2300, 0x00, 2, 0x001E)
            self.sdo_write(node, 0x6060, 0x00, 1, 7)
            # RPDO2 carries only 0x60C1:01 (the four-byte position point).
            # CiA-402 submode -1 requires a per-segment write to 0x60C1:02
            # as well, so it is incompatible with this fixed-size PDO.  Use
            # submode 0: every position point becomes a fixed-period segment
            # using the common 0x60C2:01 interpolation interval below.
            self.sdo_write(node, 0x60C0, 0x00, 2, 0, signed=True)
            self.sdo_write(node, 0x60C2, 0x01, 1, period_ms)
            self.clear_interpolation_buffer(node)
            self.sdo_write(node, 0x6040, 0x00, 2, 0x0007)
            self.sdo_write(node, 0x6040, 0x00, 2, 0x000F)
            self.sdo_write(
                node, 0x6081, 0x00, 4, profile_velocity_units
            )
            self.sdo_write(
                node, 0x6083, 0x00, 4, profile_acceleration
            )
            self.sdo_write(
                node, 0x6084, 0x00, 4, profile_acceleration
            )
            self.sdo_write(
                node, 0x2120, 0x00, 4, tracking_window_counts
            )
            velocity_readback, _ = self.sdo_read(node, 0x6081, 0x00)
            acceleration_readback, _ = self.sdo_read(node, 0x6083, 0x00)
            deceleration_readback, _ = self.sdo_read(node, 0x6084, 0x00)
            tracking_window_readback, _ = self.sdo_read(
                node, 0x2120, 0x00
            )
            if (
                velocity_readback != profile_velocity_units
                or acceleration_readback != profile_acceleration
                or deceleration_readback != profile_acceleration
                or tracking_window_readback != tracking_window_counts
            ):
                raise RuntimeError(
                    f"Node{node} profile limit readback mismatch"
                )
            period_index, signed_period_index = self.sdo_read(
                node, 0x60C2, 0x02
            )
            if period_index == 0 or signed_period_index != -3:
                raise RuntimeError(
                    f"Node{node} interpolation time index "
                    f"{signed_period_index}, expected -3"
                )
            mode, signed_mode = self.sdo_read(node, 0x6061, 0x00)
            if mode != 7 or signed_mode != 7:
                raise RuntimeError(f"Node{node} mode display is {signed_mode}")
            # BC/BC2 field evidence: in interpolated-position mode this drive
            # can report statusword 0x162F while 0x2183 is zero and the axis is
            # enabled/movable.  Treat statusword as diagnostic evidence here;
            # hard-stop the analyzer test on EMCY or vendor fault 0x2183.
            self.sdo_read(node, 0x6041, 0x00)
            _, start_position = self.sdo_read(node, 0x6064, 0x00)
            start_positions[node] = start_position

        # Preserve the CiA-402 bit-4 low state while the first trajectory
        # segments are preloaded.  The caller creates the rising edge only
        # after every axis has the same number of buffered segments.
        for node in self.nodes:
            self.sdo_write(node, 0x6040, 0x00, 2, 0x000F)
        # The installed lift motors use drive-owned holding brakes.  Do not
        # accumulate interpolation error while the brake release delay or stale
        # interpolation-buffer cleanup is still settling.  Re-read the current
        # absolute position after the axes become stable and use that value as
        # the interpolation origin.
        time.sleep(enable_settle_ms / 1000.0)
        start_positions = self.wait_axes_settled()
        return start_positions

    def stop_interpolation(self) -> None:
        for node in self.nodes:
            try:
                self.sdo_write(node, 0x6040, 0x00, 2, 0x000F)
            except Exception:
                pass

    def emergency_disable(self) -> None:
        """Apply the drive-owned holding brakes by disabling all selected axes."""
        for node in self.nodes:
            try:
                self.sdo_write(node, 0x6040, 0x00, 2, 0x0000)
            except Exception:
                pass

    @staticmethod
    def rpdo1_interpolation_trigger_payload(controlword: int,
                                            position_counts: int) -> bytes:
        payload = bytearray(7)
        payload[0:2] = int(controlword).to_bytes(2, "little")
        payload[2] = 7
        payload[3:7] = int(position_counts).to_bytes(4, "little", signed=True)
        return bytes(payload)

    def send_interpolation_trigger_group(self, controlword: int,
                                         positions: dict[int, int]) -> None:
        for node in self.nodes:
            self.send(
                RPDO1_BASE + node,
                self.rpdo1_interpolation_trigger_payload(
                    controlword, positions[node]
                ),
            )
        self.send(SYNC_COB_ID, b"")

    def decode_feedback(self, can_id: int, data: bytes) -> None:
        for node in self.nodes:
            if can_id == TPDO0_BASE + node and len(data) == 8:
                item = self.feedback[node]
                item.position = int.from_bytes(data[0:4], "little", signed=True)
                item.velocity = int.from_bytes(data[4:8], "little", signed=True)
                item.tpdo0_count += 1
                return
            if can_id == TPDO1_BASE + node and len(data) == 8:
                item = self.feedback[node]
                item.fault = int.from_bytes(data[0:4], "little")
                item.statusword = int.from_bytes(data[4:6], "little")
                item.current = int.from_bytes(data[6:8], "little", signed=True)
                item.peak_abs_current = max(
                    item.peak_abs_current,
                    abs(item.current),
                )
                item.tpdo1_count += 1
                return
            if can_id == EMCY_BASE + node:
                error_code = (
                    int.from_bytes(data[0:2], "little") if len(data) >= 2 else -1
                )
                # CANopen EMCY error code 0x0000 is an error-reset/no-current-
                # error notification.  BC2 may retain nonzero diagnostic bytes
                # after the two-byte error code, so classify by the standard
                # error-code field rather than requiring all eight bytes zero.
                if error_code == 0:
                    self.emcy_reset_count += 1
                    return
                # Vendor manual: 0x2310 means "Current Limited".  It is a
                # non-latching limit event on the present BC2 setup and may be
                # used as measured evidence that the requested trajectory is
                # above the sustainable load point.  Record it, but keep the
                # feedback-limited stream alive.  Amplifier errors such as
                # 0x5080 and every other nonzero EMCY remain hard-stop events.
                if error_code == 0x2310:
                    self.current_limit_emcy_count += 1
                    self.current_limit_emcy_by_node[node] += 1
                    return
                self.emcy.append(
                    {
                        "node": node,
                        "error_code": f"0x{error_code:04X}",
                        "data": data.hex(" ").upper(),
                    }
                )
                return

    def drain_feedback(self, duration_ms: int) -> None:
        deadline = time.monotonic() + duration_ms / 1000.0
        while time.monotonic() < deadline:
            for can_id, data in self.receive_raw(1):
                self.decode_feedback(can_id, data)

    def read_positions(self) -> dict[int, int]:
        positions: dict[int, int] = {}
        for node in self.nodes:
            _, position = self.sdo_read(node, 0x6064, 0x00)
            positions[node] = position
        return positions

    def wait_axes_settled(self, timeout_ms: int = 3000,
                          stable_ms: int = 500,
                          max_drift_counts: int = 3000) -> dict[int, int]:
        """Return absolute positions after all selected axes stop drifting."""
        deadline = time.monotonic() + timeout_ms / 1000.0
        stable_since: float | None = None
        previous = self.read_positions()
        while time.monotonic() < deadline:
            time.sleep(0.1)
            current = self.read_positions()
            max_drift = max(abs(current[node] - previous[node])
                            for node in self.nodes)
            if max_drift <= max_drift_counts:
                if stable_since is None:
                    stable_since = time.monotonic()
                if (time.monotonic() - stable_since) * 1000.0 >= stable_ms:
                    return current
            else:
                stable_since = None
            previous = current
        raise RuntimeError("axes did not settle before interpolation start")

    @staticmethod
    def triangle_delta(amplitude: int, index: int, sample_count: int) -> int:
        half = sample_count // 2
        if index <= half:
            return round(amplitude * index / half)
        return round(amplitude * (sample_count - index) / half)

    def run(self, amplitude: int, period_ms: int, sample_count: int,
            warmup_samples: int, hold_samples: int,
            enable_settle_ms: int, profile_velocity_units: int,
            profile_acceleration: int,
            max_following_lead_counts: int,
            one_way: bool,
            cycles: int,
            speed_mm_s: float,
            acceleration_mm_s2: float,
            tracking_window_mm: float,
            sync_correction_gain: float,
            sync_correction_max_counts: int,
            absolute_target_mm: float | None,
            completion_timeout_ms: int,
            completion_tolerance_counts: int,
            completion_stable_samples: int) -> dict[str, object]:
        if sample_count < 20 or (sample_count % 2) != 0:
            raise ValueError("sample-count must be even and at least 20")
        if cycles < 1:
            raise ValueError("cycles must be >= 1")
        if speed_mm_s < 0.0 or acceleration_mm_s2 < 0.0:
            raise ValueError("speed and acceleration must be non-negative")
        if speed_mm_s > 0.0 and acceleration_mm_s2 <= 0.0:
            raise ValueError("positive --speed-mm-s requires --accel-mm-s2")
        if tracking_window_mm <= 0.0 or tracking_window_mm > 3.0:
            raise ValueError("tracking window must be in (0, 3] mm")
        if sync_correction_gain < 0.0 or sync_correction_gain > 1.0:
            raise ValueError("sync correction gain must be in [0, 1]")
        if sync_correction_max_counts < 0:
            raise ValueError("sync correction maximum must be non-negative")
        if completion_timeout_ms <= 0:
            raise ValueError("completion timeout must be positive")
        if completion_tolerance_counts <= 0:
            raise ValueError("completion tolerance must be positive")
        if completion_stable_samples < 1:
            raise ValueError("completion stable samples must be >= 1")
        tracking_window_counts = math.ceil(
            tracking_window_mm * LIFT_COUNTS_PER_MM
        )
        starts = self.configure_axes(
            period_ms,
            enable_settle_ms,
            profile_velocity_units,
            profile_acceleration,
            tracking_window_counts,
        )
        if absolute_target_mm is not None:
            if not one_way:
                raise ValueError("absolute target requires --one-way")
            if absolute_target_mm < 10.0 or absolute_target_mm > 490.0:
                raise ValueError("absolute target must be within 10..490 mm")
            target_count = -round(absolute_target_mm * LIFT_COUNTS_PER_MM)
            final_deltas = {
                node: target_count - starts[node]
                for node in self.nodes
            }
            directions = {
                1 if delta > 0 else -1
                for delta in final_deltas.values()
                if delta != 0
            }
            if len(directions) > 1:
                raise ValueError(
                    "absolute target would require mixed axis directions"
                )
            initial_absolute_spread = max(starts.values()) - min(starts.values())
            absolute_spread_limit = max(
                initial_absolute_spread,
                round(LIFT_MAX_SYNC_SPREAD_MM * LIFT_COUNTS_PER_MM),
            ) + round(0.5 * LIFT_COUNTS_PER_MM)
        else:
            final_deltas = {node: amplitude for node in self.nodes}
            initial_absolute_spread = 0
            absolute_spread_limit = 0
        motion_distance_counts = max(abs(delta) for delta in final_deltas.values())
        if motion_distance_counts == 0:
            raise ValueError("absolute target is already reached on every axis")
        motion_direction = next(
            (1 if delta > 0 else -1 for delta in final_deltas.values() if delta != 0),
            1,
        )
        next_tick = time.perf_counter()
        max_burst_us = 0.0
        max_relative_spread = 0

        if warmup_samples < 3:
            raise ValueError("warmup-samples is the preload depth and must be >= 3")
        trapezoid = None
        if speed_mm_s > 0.0:
            if not one_way:
                raise ValueError("--speed-mm-s currently requires --one-way")
            distance_counts = motion_distance_counts
            speed_counts_s = speed_mm_s * LIFT_COUNTS_PER_MM
            acceleration_counts_s2 = acceleration_mm_s2 * LIFT_COUNTS_PER_MM
            accel_time_s = speed_counts_s / acceleration_counts_s2
            accel_distance_counts = (
                0.5 * acceleration_counts_s2 * accel_time_s * accel_time_s
            )
            if 2.0 * accel_distance_counts >= distance_counts:
                accel_time_s = math.sqrt(
                    distance_counts / acceleration_counts_s2
                )
                peak_speed_counts_s = acceleration_counts_s2 * accel_time_s
                cruise_time_s = 0.0
                accel_distance_counts = distance_counts / 2.0
            else:
                peak_speed_counts_s = speed_counts_s
                cruise_time_s = (
                    distance_counts - 2.0 * accel_distance_counts
                ) / peak_speed_counts_s
            total_motion_time_s = 2.0 * accel_time_s + cruise_time_s
            active_samples = max(
                1,
                math.ceil(total_motion_time_s * 1000.0 / period_ms),
            )
            trapezoid = {
                "distance_counts": distance_counts,
                "acceleration_counts_s2": acceleration_counts_s2,
                "accel_time_s": accel_time_s,
                "accel_distance_counts": accel_distance_counts,
                "peak_speed_counts_s": peak_speed_counts_s,
                "cruise_time_s": cruise_time_s,
                "total_motion_time_s": total_motion_time_s,
            }
            required_profile_velocity = math.ceil(
                peak_speed_counts_s * CANOPEN_PROFILE_VELOCITY_SCALE
            )
            required_profile_acceleration = math.ceil(
                acceleration_counts_s2 * CANOPEN_PROFILE_ACCELERATION_SCALE
            )
            if profile_velocity_units < required_profile_velocity:
                raise ValueError(
                    "profile velocity is below the trajectory requirement: "
                    f"{profile_velocity_units} < {required_profile_velocity}"
                )
            if profile_acceleration < required_profile_acceleration:
                raise ValueError(
                    "profile acceleration is below the trajectory requirement: "
                    f"{profile_acceleration} < {required_profile_acceleration}"
                )
            trapezoid["required_profile_velocity"] = required_profile_velocity
            trapezoid["required_profile_acceleration"] = (
                required_profile_acceleration
            )
        else:
            active_samples = sample_count if one_way else sample_count * cycles
        # The trajectory clock can finish before the drives consume a target
        # lead that was deliberately limited for synchronization.  Continue
        # streaming the final point until all four measured positions reach it
        # stably; a fixed number of trailing samples can otherwise terminate a
        # safe but slow axis short of its requested travel.
        minimum_total_samples = warmup_samples + active_samples + hold_samples + 1
        maximum_total_samples = (
            minimum_total_samples +
            math.ceil(completion_timeout_ms / period_ms)
        )
        final_target_deltas = {
            node: final_deltas[node] if one_way else 0
            for node in self.nodes
        }
        final_tolerance_counts = max(
            completion_tolerance_counts,
            35_000 if absolute_target_mm is not None else 0,
        )
        completion_stable_count = 0
        completed_final_target = False
        next_tick = 0.0
        previous_desired_delta = 0
        previous_target_deltas = {node: 0 for node in self.nodes}
        trajectory_direction = -1
        for sample_index in range(maximum_total_samples):
            if sample_index == warmup_samples:
                # Manual-required start order: fill at least three segments,
                # then create the bit-4 rising edge on every axis through the
                # same synchronous RPDO1 path used by the ECU firmware.
                self.send_interpolation_trigger_group(0x003F, starts)
                next_tick = time.perf_counter()

            active_index = sample_index - warmup_samples
            if active_index < 0:
                # Preload stationary points at the measured current absolute
                # position.  This keeps the interpolation buffer primed without
                # asking the first executed segment to jump away from the
                # physical start position when bit 4 is finally triggered.
                desired_delta = 0
            elif active_index >= active_samples:
                desired_delta = (
                    motion_direction * motion_distance_counts if one_way else 0
                )
            elif trapezoid is not None:
                t_s = min(
                    active_index * period_ms / 1000.0,
                    trapezoid["total_motion_time_s"],
                )
                accel_end_s = trapezoid["accel_time_s"]
                cruise_end_s = accel_end_s + trapezoid["cruise_time_s"]
                if t_s <= accel_end_s:
                    distance = (
                        0.5 * trapezoid["acceleration_counts_s2"] * t_s * t_s
                    )
                elif t_s <= cruise_end_s:
                    distance = (
                        trapezoid["accel_distance_counts"] +
                        trapezoid["peak_speed_counts_s"] *
                        (t_s - accel_end_s)
                    )
                else:
                    remaining_s = trapezoid["total_motion_time_s"] - t_s
                    distance = (
                        trapezoid["distance_counts"] -
                        0.5 * trapezoid["acceleration_counts_s2"] *
                        remaining_s * remaining_s
                    )
                desired_delta = (
                    motion_direction * round(distance)
                )
            elif one_way:
                desired_delta = round(amplitude * active_index / sample_count)
            else:
                desired_delta = self.triangle_delta(
                    amplitude,
                    active_index % sample_count,
                    sample_count,
                )
            progress = min(
                1.0,
                abs(desired_delta) / motion_distance_counts,
            )
            desired_deltas = {
                node: round(final_deltas[node] * progress)
                for node in self.nodes
            }
            actual_relative = {
                node: self.feedback[node].position - starts[node]
                for node in self.nodes
                if self.feedback[node].tpdo0_count > 0
            }
            if desired_delta < previous_desired_delta:
                trajectory_direction = -1
            elif desired_delta > previous_desired_delta:
                trajectory_direction = 1

            # Limit the lead for each axis independently.  A single common
            # clamp based on the slowest axis can pull a faster axis's target
            # backwards, producing a large following error and 0x7390.  Each
            # axis may only advance along the selected trajectory direction;
            # synchronization is checked separately from measured positions.
            target_deltas: dict[int, int] = {}
            sync_corrections: dict[int, int] = {node: 0 for node in self.nodes}
            mean_relative = (
                sum(actual_relative.values()) / len(actual_relative)
                if actual_relative else 0.0
            )
            for node in self.nodes:
                delta = desired_deltas[node]
                if node in actual_relative:
                    measured_delta = actual_relative[node]
                    # A positive error means this axis has moved less far in
                    # the currently requested negative direction.  Increase
                    # its permissible target lead while reducing the lead of
                    # an axis that has moved farther.  The correction never
                    # changes the selected trajectory direction.
                    if absolute_target_mm is not None:
                        expected_position = starts[node] + desired_deltas[node]
                        lag_counts = (
                            expected_position - self.feedback[node].position
                        ) * trajectory_direction
                        correction = int(round(
                            lag_counts * sync_correction_gain
                        ))
                    else:
                        correction = int(round(
                            (measured_delta - mean_relative) * sync_correction_gain
                        ))
                    correction = max(
                        -sync_correction_max_counts,
                        min(sync_correction_max_counts, correction),
                    )
                    sync_corrections[node] = correction
                    if trajectory_direction < 0:
                        lead_counts = max(
                            1,
                            max_following_lead_counts + correction,
                        )
                        delta = max(
                            desired_deltas[node],
                            measured_delta - lead_counts,
                        )
                        delta = min(previous_target_deltas[node], delta)
                    else:
                        lead_counts = max(
                            1,
                            (max_following_lead_counts + correction
                             if absolute_target_mm is not None
                             else max_following_lead_counts - correction),
                        )
                        delta = min(
                            desired_deltas[node],
                            measured_delta + lead_counts,
                        )
                        delta = max(previous_target_deltas[node], delta)
                target_deltas[node] = delta

            previous_desired_delta = desired_delta
            previous_target_deltas = target_deltas.copy()
            targets = {node: starts[node] + target_deltas[node] for node in self.nodes}
            if (min(targets.values()) < (LIFT_MIN_POSITION_COUNTS - LIFT_LIMIT_MARGIN_COUNTS) or
                    max(targets.values()) > (LIFT_MAX_POSITION_COUNTS + LIFT_LIMIT_MARGIN_COUNTS)):
                raise RuntimeError(f"target outside commissioning limits: {targets}")

            burst_start = time.perf_counter()
            for node in self.nodes:
                self.send(
                    RPDO2_BASE + node,
                    int(targets[node]).to_bytes(4, "little", signed=True),
                )
            self.send(SYNC_COB_ID, b"")
            burst_us = (time.perf_counter() - burst_start) * 1_000_000.0
            max_burst_us = max(max_burst_us, burst_us)

            self.drain_feedback(min(5, max(1, period_ms // 3)))
            relatives = [
                self.feedback[node].position - starts[node]
                for node in self.nodes
                if self.feedback[node].tpdo0_count > 0
            ]
            absolute_positions = [
                self.feedback[node].position
                for node in self.nodes
                if self.feedback[node].tpdo0_count > 0
            ]
            relative_spread = (
                max(relatives) - min(relatives) if len(relatives) == len(self.nodes)
                else 0
            )
            absolute_spread = (
                max(absolute_positions) - min(absolute_positions)
                if len(absolute_positions) == len(self.nodes) else 0
            )
            observed_spread = absolute_spread if absolute_target_mm is not None else relative_spread
            max_relative_spread = max(max_relative_spread, observed_spread)
            spread_limit = (
                absolute_spread_limit if absolute_target_mm is not None else
                round(LIFT_MAX_SYNC_SPREAD_MM * LIFT_COUNTS_PER_MM)
            )
            if observed_spread > spread_limit:
                raise RuntimeError(
                    "synchronization error "
                    f"{observed_spread / LIFT_COUNTS_PER_MM:.3f}mm"
                )
            self.samples.append(
                {
                    "index": sample_index,
                    "phase": (
                        "preload"
                        if sample_index < warmup_samples
                        else "hold"
                        if active_index >= active_samples
                        else "stream"
                    ),
                    "active_index": active_index,
                    "desired_delta": desired_delta,
                    "target_deltas": target_deltas,
                    "sync_corrections": sync_corrections,
                    "targets": targets,
                    "burst_us": round(burst_us, 1),
                    "relative_spread": relative_spread,
                    "absolute_spread": absolute_spread,
                    "feedback": {
                        str(node): {
                            "position": self.feedback[node].position,
                            "velocity": self.feedback[node].velocity,
                            "statusword": f"0x{self.feedback[node].statusword:04X}",
                            "fault": f"0x{self.feedback[node].fault:08X}",
                            "current": self.feedback[node].current,
                        }
                        for node in self.nodes
                    },
                }
            )
            if self.emcy:
                raise RuntimeError(
                    "EMCY received during interpolation stream "
                    f"sample={sample_index} active={active_index} "
                    f"desired_delta={desired_delta} target_deltas={target_deltas}: "
                    f"{self.emcy[-1]}"
                )
            for node in self.nodes:
                item = self.feedback[node]
                if item.fault != 0:
                    raise RuntimeError(
                        f"Node{node} fault=0x{item.fault:08X} "
                        f"status=0x{item.statusword:04X}"
                    )

            if active_index >= active_samples:
                final_errors = [
                    abs(self.feedback[node].position -
                        (starts[node] + final_target_deltas[node]))
                    for node in self.nodes
                    if self.feedback[node].tpdo0_count > 0
                ]
                if (len(final_errors) == len(self.nodes) and
                        max(final_errors) <= final_tolerance_counts):
                    completion_stable_count += 1
                else:
                    completion_stable_count = 0

            if sample_index >= warmup_samples:
                next_tick += period_ms / 1000.0
                remaining = next_tick - time.perf_counter()
                if remaining > 0:
                    time.sleep(remaining)

            if (sample_index >= minimum_total_samples and
                    completion_stable_count >= completion_stable_samples):
                completed_final_target = True
                break

        if not completed_final_target:
            raise RuntimeError(
                "final interpolation target was not reached within "
                f"{completion_timeout_ms}ms"
            )

        # The stream is intentionally maintained ahead of execution by the
        # preload depth.  Allow those final buffered hold points to execute
        # before clearing controlword bit 4.
        for _ in range(warmup_samples + 2):
            tick_start = time.perf_counter()
            self.send(SYNC_COB_ID, b"")
            self.drain_feedback(min(5, max(1, period_ms // 3)))
            remaining = period_ms / 1000.0 - (time.perf_counter() - tick_start)
            if remaining > 0:
                time.sleep(remaining)

        self.stop_interpolation()
        time.sleep(0.5)
        final: dict[str, object] = {}
        for node in self.nodes:
            _, position = self.sdo_read(node, 0x6064, 0x00)
            _, velocity = self.sdo_read(node, 0x606C, 0x00)
            status, _ = self.sdo_read(node, 0x6041, 0x00)
            fault, _ = self.sdo_read(node, 0x2183, 0x00)
            final[str(node)] = {
                "start_position": starts[node],
                "position": position,
                "position_error": position - starts[node],
                "velocity": velocity,
                "statusword": f"0x{status:04X}",
                "fault": f"0x{fault:08X}",
                "peak_abs_current": self.feedback[node].peak_abs_current,
                "tpdo0_count": self.feedback[node].tpdo0_count,
                "tpdo1_count": self.feedback[node].tpdo1_count,
            }
        return {
            "nodes": self.nodes,
            "amplitude": amplitude,
            "absolute_target_mm": absolute_target_mm,
            "initial_absolute_spread_mm": round(
                initial_absolute_spread / LIFT_COUNTS_PER_MM, 3
            ),
            "period_ms": period_ms,
            "sample_count": sample_count,
            "cycles": cycles,
            "warmup_samples": warmup_samples,
            "enable_settle_ms": enable_settle_ms,
            "profile_velocity_units": profile_velocity_units,
            "profile_acceleration": profile_acceleration,
            "max_following_lead_counts": max_following_lead_counts,
            "tracking_window_mm": tracking_window_mm,
            "sync_correction_gain": sync_correction_gain,
            "sync_correction_max_counts": sync_correction_max_counts,
            "completion_timeout_ms": completion_timeout_ms,
            "completion_tolerance_counts": completion_tolerance_counts,
            "effective_completion_tolerance_counts": final_tolerance_counts,
            "completion_stable_samples": completion_stable_samples,
            "completed_final_target": completed_final_target,
            "counts_per_mm": LIFT_COUNTS_PER_MM,
            "speed_mm_s": speed_mm_s,
            "acceleration_mm_s2": acceleration_mm_s2,
            "requested_motor_rpm": round(
                speed_mm_s * LIFT_MOTOR_REVS_PER_MM * 60.0,
                3,
            ),
            "trapezoid": trapezoid,
            "max_relative_spread_mm": round(max_relative_spread / LIFT_COUNTS_PER_MM, 3),
            "max_burst_us": round(max_burst_us, 1),
            "max_relative_spread": max_relative_spread,
            "emcy": self.emcy,
            "emcy_reset_count": self.emcy_reset_count,
            "current_limit_emcy_count": self.current_limit_emcy_count,
            "current_limit_emcy_by_node": self.current_limit_emcy_by_node,
            "final": final,
            "samples": self.samples,
        }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="CAN3 Node9/11/12/10 synchronous lift interpolation test"
    )
    parser.add_argument(
        "--nodes",
        default="9,11,12,10",
        help="comma-separated subset in physical leg order",
    )
    parser.add_argument("--amplitude", type=int, default=-20_000)
    parser.add_argument("--period-ms", type=int, default=20)
    parser.add_argument("--sample-count", type=int, default=200)
    # The drive needs at least three buffered points.  Keeping exactly three
    # limits the 20 ms feedback-control horizon to 60 ms instead of allowing
    # a long preloaded trajectory to continue after measured axes diverge.
    parser.add_argument("--warmup-samples", type=int, default=3)
    parser.add_argument("--hold-samples", type=int, default=10)
    parser.add_argument("--enable-settle-ms", type=int, default=1500)
    parser.add_argument("--profile-velocity-units", type=int, default=833_333)
    parser.add_argument("--profile-acceleration", type=int, default=500_000)
    parser.add_argument("--max-following-lead-counts", type=int, default=4_000)
    parser.add_argument("--tracking-window-mm", type=float, default=3.0)
    parser.add_argument(
        "--absolute-target-mm",
        type=float,
        default=None,
        help="common absolute leg-height target in mm; valid range is 10..490",
    )
    parser.add_argument(
        "--sync-correction-gain",
        type=float,
        default=0.25,
        help="bounded proportional correction from relative four-axis position error",
    )
    parser.add_argument(
        "--sync-correction-max-counts",
        type=int,
        default=50_000,
        help="maximum per-axis lead correction applied for synchronization",
    )
    parser.add_argument(
        "--completion-timeout-ms",
        type=int,
        default=30_000,
        help="maximum extra stream time while measured axes close the final lead",
    )
    parser.add_argument(
        "--completion-tolerance-counts",
        type=int,
        default=10_000,
        help="per-axis final-position acceptance tolerance",
    )
    parser.add_argument(
        "--completion-stable-samples",
        type=int,
        default=5,
        help="consecutive TPDO0 samples required at the final target",
    )
    parser.add_argument(
        "--speed-mm-s",
        type=float,
        default=0.0,
        help="trapezoid maximum leg speed; requires --one-way",
    )
    parser.add_argument(
        "--accel-mm-s2",
        type=float,
        default=10.0,
        help="trapezoid leg acceleration/deceleration",
    )
    parser.add_argument(
        "--one-way",
        action="store_true",
        help="ramp once to amplitude instead of performing a triangle return",
    )
    parser.add_argument(
        "--cycles",
        type=int,
        default=1,
        help="number of triangle cycles when --one-way is not used",
    )
    parser.add_argument("--timeout-ms", type=int, default=1000)
    parser.add_argument("--log", default="")
    parser.add_argument("--allow-motion", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.allow_motion:
        print("FAILED: --allow-motion is required", file=sys.stderr)
        return 2
    nodes = tuple(int(item) for item in args.nodes.split(",") if item)
    if not nodes or any(node not in LIFT_NODES for node in nodes):
        print(f"FAILED: nodes must be a subset of {LIFT_NODES}", file=sys.stderr)
        return 2
    log_path = (
        Path(args.log)
        if args.log
        else REPO_ROOT / "tmp" /
             f"lift4_sync_{time.strftime('%Y%m%d_%H%M%S')}.json"
    )
    debug: Lift4SyncDebug | None = None
    try:
        with Lift4SyncDebug(nodes, args.timeout_ms) as debug:
            report = debug.run(
                args.amplitude,
                args.period_ms,
                args.sample_count,
                args.warmup_samples,
                args.hold_samples,
                args.enable_settle_ms,
                args.profile_velocity_units,
                args.profile_acceleration,
                args.max_following_lead_counts,
                args.one_way,
                args.cycles,
                args.speed_mm_s,
                args.accel_mm_s2,
                args.tracking_window_mm,
                args.sync_correction_gain,
                args.sync_correction_max_counts,
                args.absolute_target_mm,
                args.completion_timeout_ms,
                args.completion_tolerance_counts,
                args.completion_stable_samples,
            )
        log_path.parent.mkdir(parents=True, exist_ok=True)
        log_path.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        print(json.dumps({key: value for key, value in report.items()
                          if key != "samples"}, ensure_ascii=False, indent=2))
        print(f"Log written to {log_path}")
        return 0
    except Exception as exc:  # noqa: BLE001 - report and safe-stop hardware errors.
        if debug is not None:
            partial_report = {
                "failed": True,
                "error": str(exc),
                "nodes": debug.nodes,
                "emcy": debug.emcy,
                "current_limit_emcy_count": debug.current_limit_emcy_count,
                "current_limit_emcy_by_node": debug.current_limit_emcy_by_node,
                "buffer_status_after_clear": debug.buffer_status_after_clear,
                "feedback": {
                    str(node): {
                        "position": item.position,
                        "velocity": item.velocity,
                        "statusword": f"0x{item.statusword:04X}",
                        "fault": f"0x{item.fault:08X}",
                        "current": item.current,
                        "peak_abs_current": item.peak_abs_current,
                        "tpdo0_count": item.tpdo0_count,
                        "tpdo1_count": item.tpdo1_count,
                    }
                    for node, item in debug.feedback.items()
                },
                "samples": debug.samples,
            }
            log_path.parent.mkdir(parents=True, exist_ok=True)
            log_path.write_text(
                json.dumps(partial_report, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            print(f"Failure log written to {log_path}", file=sys.stderr)
        print(f"FAILED: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
