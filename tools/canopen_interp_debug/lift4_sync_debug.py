"""CAN3 four-lift synchronous interpolation commissioning tool.

The ECU CAN3 transceiver must be physically disconnected before this tool is
used.  The analyzer CAN2 channel owns the bus during the test.  The tool never
resets a drive and never writes nonvolatile memory.

Only the complete Node9/11/12/10 group may move.  Each realtime cycle sends one
absolute 0x60C1:01 RPDO2 point per axis followed by one SYNC.  Every cycle must
then receive a new TPDO0 from every axis; cached feedback is never accepted as
proof that the current point was executed.  The installed 2026-07 calibration
and trajectory defaults intentionally match ``ecu_config.h``.
"""

from __future__ import annotations

import argparse
import gc
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
PDO_TOOL_DIR = REPO_ROOT / "tools" / "canopen_pdo_config"
if str(PDO_TOOL_DIR) not in sys.path:
    sys.path.insert(0, str(PDO_TOOL_DIR))

from controlcan import ControlCAN, VCI_CAN_OBJ  # noqa: E402
from pdo_profiles import build_node_configuration  # noqa: E402


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
LIFT_SAFE_MIN_HEIGHT_MM = 10.0
LIFT_SAFE_MAX_HEIGHT_MM = 490.0
LIFT_MECHANICAL_MIN_HEIGHT_MM = 0.0
LIFT_MECHANICAL_MAX_HEIGHT_MM = 500.0
LIFT_MECHANICAL_MARGIN_MM = 1.0
# Extension moves the installed absolute-position count in the negative
# direction.  These are normal-control endpoints, not overshoot permissions.
LIFT_MIN_POSITION_COUNTS = -round(
    LIFT_SAFE_MAX_HEIGHT_MM * LIFT_COUNTS_PER_MM
)
LIFT_MAX_POSITION_COUNTS = -round(
    LIFT_SAFE_MIN_HEIGHT_MM * LIFT_COUNTS_PER_MM
)
LIFT_FINAL_SYNC_SPREAD_MM = 3.0
LIFT_RUNNING_SPREAD_WARNING_MM = 15.0
LIFT_ZERO_SPEED_RPM = 3.0
LIFT_ZERO_SPEED_VELOCITY_UNITS = round(
    LIFT_ZERO_SPEED_RPM * LIFT_COUNTS_PER_MOTOR_REV * 10.0 / 60.0
)
# Analyzer-proven defaults mirrored by ecu_config.h.  Keep these named here so
# a review can compare the hazardous bench tool with production configuration.
DEFAULT_PERIOD_MS = 20
DEFAULT_SPEED_MM_S = 20.0
DEFAULT_ACCEL_MM_S2 = 8.0
DEFAULT_PROFILE_VELOCITY_UNITS = 53_000_000
DEFAULT_PROFILE_ACCELERATION_UNITS = 250_000
DEFAULT_PREALIGN_SPEED_MM_S = 2.0
DEFAULT_FEEDBACK_GOVERNOR_MIN_SPEED_MM_S = 2.0
FOLLOWING_ERROR_GUARD_MARGIN_MM = 0.1
# Keep 2.5 mm of common trajectory look-ahead.  The running following-error
# window is deliberately separate from the final 3 mm leveling contract:
# following error measures target lead over the slowest measured axis, while
# four-leg synchronization is the spread between measured axis positions.
# A transient host/USB delay may grow target lead without making the chassis
# unlevel.  The online governor and starvation recovery reduce that lead; only
# final arrival still requires every axis and the measured spread within 3 mm.
DEFAULT_MAX_FOLLOWING_LEAD_COUNTS = round(2.5 * LIFT_COUNTS_PER_MM)
DEFAULT_TRACKING_WINDOW_MM = 10.0
DEFAULT_COMPLETION_TOLERANCE_MM = 3.0
DEFAULT_COMPLETION_STABLE_SAMPLES = 10
DEFAULT_TPDO0_TIMEOUT_MS = 15
DEFAULT_TPDO0_RECOVERY_RETRIES = 2
INTERPOLATION_STARVATION_CONFIRM_SAMPLES = 3
INTERPOLATION_STARVATION_MAX_SPEED_RPM = 10.0
INTERPOLATION_STARVATION_RECOVERY_LIMIT = 3
ACTUAL_CURRENT_AMPS_PER_UNIT = 0.01
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
            # On a normal return, clear the interpolation trigger before the
            # final brake command.  On any exception/interrupt, do not briefly
            # re-enable 0x000F: send the synchronous disable group immediately.
            if exc_type is None:
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

    def clear_drive_fault(self, node: int) -> None:
        """Clear both the vendor latch and the CiA-402 fault state.

        Field failures can leave statusword bit 3 asserted after 0x2183 has
        already been acknowledged.  Looking only at the vendor object then
        skips the required CiA-402 fault-reset edge and every later enable
        command is ignored.  This recovery never resets the CANopen node and
        therefore preserves the established absolute position reference.
        """
        latched_fault, _ = self.sdo_read(node, 0x2183, 0x00)
        statusword, _ = self.sdo_read(node, 0x6041, 0x00)
        cia402_fault_active = (statusword & 0x0008) != 0
        if latched_fault == 0 and not cia402_fault_active:
            return
        if latched_fault != 0:
            self.sdo_write(node, 0x2183, 0x00, 4, latched_fault)
            time.sleep(0.05)
        self.sdo_write(node, 0x6040, 0x00, 2, 0x0000)
        time.sleep(0.03)
        # Hold/reassert the fault-reset request until the drive reports that
        # the CiA-402 fault state has actually cleared.  A fixed 30 ms pulse is
        # too short after an interpolation fault on the installed BC2 drives.
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            # CiA-402 requires a low-to-high transition of bit 7.  Repeating
            # 0x0080 without first clearing the bit does not create a second
            # recovery attempt.
            self.sdo_write(node, 0x6040, 0x00, 2, 0x0000)
            time.sleep(0.03)
            self.sdo_write(node, 0x6040, 0x00, 2, 0x0080)
            time.sleep(0.08)
            remaining_fault, _ = self.sdo_read(node, 0x2183, 0x00)
            remaining_statusword, _ = self.sdo_read(node, 0x6041, 0x00)
            if remaining_fault != 0:
                self.sdo_write(
                    node,
                    0x2183,
                    0x00,
                    4,
                    remaining_fault,
                )
            if (
                remaining_fault == 0
                and (remaining_statusword & 0x0008) == 0
            ):
                self.sdo_write(node, 0x6040, 0x00, 2, 0x0006)
                return
            time.sleep(0.05)
        raise RuntimeError(
            f"Node{node} fault did not clear: "
            f"0x2183=0x{remaining_fault:08X}, "
            f"0x6041=0x{remaining_statusword:04X}"
        )

    @staticmethod
    def position_is_mechanically_plausible(position_counts: int) -> bool:
        height_mm = -position_counts / LIFT_COUNTS_PER_MM
        return (
            LIFT_MECHANICAL_MIN_HEIGHT_MM - LIFT_MECHANICAL_MARGIN_MM
            <= height_mm
            <= LIFT_MECHANICAL_MAX_HEIGHT_MM + LIFT_MECHANICAL_MARGIN_MM
        )

    def verify_sdo_value(
        self,
        node: int,
        index: int,
        subindex: int,
        expected: int,
    ) -> None:
        actual, _ = self.sdo_read(node, index, subindex)
        if actual != expected:
            raise RuntimeError(
                f"Node{node} PDO contract mismatch 0x{index:04X}:"
                f"{subindex:02X}: got 0x{actual:X}, expected 0x{expected:X}"
            )

    def verify_pdo_contract(self, node: int) -> None:
        """Read back the saved current7 + sync1 objects before any enable."""
        required_objects = {
            (0x1401, 0x01), (0x1401, 0x02),
            (0x1601, 0x00), (0x1601, 0x01),
            (0x1601, 0x02), (0x1601, 0x03),
            (0x1402, 0x01), (0x1402, 0x02),
            (0x1602, 0x00), (0x1602, 0x01),
            (0x1800, 0x01), (0x1800, 0x02),
            (0x1A00, 0x00), (0x1A00, 0x01), (0x1A00, 0x02),
            (0x1801, 0x01), (0x1801, 0x02),
            (0x1A01, 0x00), (0x1A01, 0x01),
            (0x1A01, 0x02), (0x1A01, 0x03),
        }
        expected: dict[tuple[int, int], int] = {}
        for operation in build_node_configuration(node):
            key = (operation.index, operation.subindex)
            if operation.kind == "download" and key in required_objects:
                # Mapping construction contains disable/clear and final enable
                # writes.  Retaining the last value yields the saved runtime
                # contract without duplicating mapping constants here.
                expected[key] = operation.value
        if set(expected) != required_objects:
            missing = sorted(required_objects - set(expected))
            raise RuntimeError(
                f"internal PDO profile is incomplete for Node{node}: {missing}"
            )
        for (index, subindex), value in sorted(expected.items()):
            self.verify_sdo_value(node, index, subindex, value)

    def configure_axes(self, period_ms: int, enable_settle_ms: int,
                       profile_velocity_units: int,
                       profile_acceleration: int,
                       tracking_window_counts: int) -> dict[int, int]:
        start_positions: dict[int, int] = {}
        # Put the whole group in a known disabled state before touching the
        # interpolation buffer.  Clearing after 0x0006 can leave queued points
        # from a previous interpolation error active in these drives.
        for node in self.nodes:
            self.send(NMT_COB_ID, bytes([0x01, node]))
            self.sdo_write(node, 0x6040, 0x00, 2, 0x0000)
        self.wait_axes_settled(stable_ms=300)

        for node in self.nodes:
            self.verify_pdo_contract(node)
            self.clear_drive_fault(node)
            # Clear while disabled.  This preserves 0x6064 and the established
            # mechanical reference; it is not a node reset.
            self.clear_interpolation_buffer(node)
            self.sdo_write(node, 0x2300, 0x00, 2, 0x001E)
            self.sdo_write(node, 0x6060, 0x00, 1, 7)
            # RPDO2 carries only 0x60C1:01 (the four-byte position point).
            # CiA-402 submode -1 requires a per-segment write to 0x60C1:02
            # as well, so it is incompatible with this fixed-size PDO.  Use
            # submode 0: every position point becomes a fixed-period segment
            # using the common 0x60C2:01 interpolation interval below.
            self.sdo_write(node, 0x60C0, 0x00, 2, 0, signed=True)
            self.sdo_write(node, 0x60C2, 0x01, 1, period_ms)
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
            self.sdo_write(node, 0x6040, 0x00, 2, 0x0006)
            _, start_position = self.sdo_read(node, 0x6064, 0x00)
            if not self.position_is_mechanically_plausible(start_position):
                raise RuntimeError(
                    f"Node{node} position {start_position} is outside the "
                    "configured mechanical plausibility range"
                )
            start_positions[node] = start_position

        # Apply switch-on and operation-enable as complete four-axis RPDO1
        # groups.  Serial SDO enabling releases the four holding brakes at
        # visibly different times and is not representative of ECU behavior.
        self.send_interpolation_trigger_group(0x0007, start_positions)
        time.sleep(0.05)
        self.send_interpolation_trigger_group(0x000F, start_positions)
        # The installed lift motors use drive-owned holding brakes.  Do not
        # accumulate interpolation error while the brake release delay or stale
        # interpolation-buffer cleanup is still settling.  Re-read the current
        # absolute position after the axes become stable and use that value as
        # the interpolation origin.
        time.sleep(enable_settle_ms / 1000.0)
        start_positions = self.wait_axes_settled()
        for node in self.nodes:
            mode, signed_mode = self.sdo_read(node, 0x6061, 0x00)
            fault, _ = self.sdo_read(node, 0x2183, 0x00)
            # BC/BC2 may report statusword 0x162F while healthy and movable in
            # interpolation mode.  The mapped vendor latch is the hard-fault
            # source for this verified workflow.
            self.sdo_read(node, 0x6041, 0x00)
            if mode != 7 or signed_mode != 7 or fault != 0:
                raise RuntimeError(
                    f"Node{node} enable verification failed: "
                    f"mode={signed_mode}, fault=0x{fault:08X}"
                )
        self.send_interpolation_trigger_group(0x000F, start_positions)
        return start_positions

    def cached_positions(self) -> dict[int, int]:
        return {
            node: self.feedback[node].position
            if self.feedback[node].tpdo0_count > 0 else 0
            for node in self.nodes
        }

    def stop_interpolation(self) -> None:
        try:
            self.send_interpolation_trigger_group(0x000F, self.cached_positions())
        except Exception:
            pass
        for node in self.nodes:
            try:
                self.sdo_write(node, 0x6040, 0x00, 2, 0x000F)
            except Exception:
                pass

    def emergency_disable(self) -> None:
        """Apply the drive-owned holding brakes by disabling all selected axes."""
        try:
            self.send_interpolation_trigger_group(0x0000, self.cached_positions())
        except Exception:
            pass
        for node in self.nodes:
            try:
                self.sdo_write(node, 0x6040, 0x00, 2, 0x0000)
            except Exception:
                pass
        # Once brakes are applied, discard any remaining interpolation points
        # so the next enable cannot replay a trajectory that preceded the
        # failure.  This is best-effort cleanup; the next configure pass still
        # repeats and verifies the disabled-state preparation.
        time.sleep(0.05)
        for node in self.nodes:
            try:
                self.clear_interpolation_buffer(node)
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
        """Apply one startup/controlword group before emitting its SYNC.

        This path is used only while configuring, enabling, triggering, or
        disabling interpolation.  Keep the four RPDO1 submissions separate:
        some ControlCAN adapters return from a batched transmit before every
        frame has left the adapter queue.  A following low-ID SYNC can then
        overtake the controlword frames on the CAN bus and leave an axis armed
        but not executing.  Startup traffic is infrequent, so deterministic
        ordering is more important here than reducing USB calls.
        """
        for node in self.nodes:
            self.send(
                RPDO1_BASE + node,
                self.rpdo1_interpolation_trigger_payload(
                    controlword,
                    positions[node],
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

    def wait_for_fresh_tpdo0(
        self,
        baseline_counts: dict[int, int],
        timeout_ms: int,
    ) -> None:
        """Require one post-SYNC TPDO0 from every lift axis."""
        deadline = time.monotonic() + timeout_ms / 1000.0
        while time.monotonic() < deadline:
            for can_id, data in self.receive_raw(1):
                self.decode_feedback(can_id, data)
            if self.emcy:
                raise RuntimeError(
                    "EMCY received while waiting for post-SYNC TPDO0: "
                    f"{self.emcy[-1]}"
                )
            faulted = [
                node for node in self.nodes
                if self.feedback[node].fault != 0
            ]
            if faulted:
                raise RuntimeError(
                    "drive fault received while waiting for post-SYNC TPDO0: "
                    + ",".join(
                        f"Node{node}=0x{self.feedback[node].fault:08X}"
                        for node in faulted
                    )
                )
            missing = [
                node for node in self.nodes
                if self.feedback[node].tpdo0_count <= baseline_counts[node]
            ]
            if not missing:
                return
        raise TimeoutError(
            "fresh TPDO0 missing after SYNC for nodes "
            + ",".join(str(node) for node in missing)
        )

    def send_stream_point_with_feedback(
        self,
        targets: dict[int, int],
        tpdo0_timeout_ms: int,
    ) -> tuple[float, int]:
        """Submit one coherent point, retrying only a lost feedback cycle.

        A retry repeats the same four-axis absolute point and SYNC.  It never
        advances the target or emits a cached later point, so a transient
        Windows/USB receive gap becomes one longer interpolation segment
        instead of a mechanical jump.  EMCY and mapped drive faults bypass
        this recovery and remain immediate failures.
        """
        maximum_burst_us = 0.0
        for attempt in range(DEFAULT_TPDO0_RECOVERY_RETRIES + 1):
            tpdo0_baseline = {
                node: self.feedback[node].tpdo0_count for node in self.nodes
            }
            burst_start = time.perf_counter()
            for node in self.nodes:
                self.send(
                    RPDO2_BASE + node,
                    int(targets[node]).to_bytes(
                        4,
                        "little",
                        signed=True,
                    ),
                )
            self.send(SYNC_COB_ID, b"")
            maximum_burst_us = max(
                maximum_burst_us,
                (time.perf_counter() - burst_start) * 1_000_000.0,
            )
            try:
                self.wait_for_fresh_tpdo0(
                    tpdo0_baseline,
                    tpdo0_timeout_ms,
                )
                return maximum_burst_us, attempt
            except TimeoutError:
                if attempt >= DEFAULT_TPDO0_RECOVERY_RETRIES:
                    raise
        raise AssertionError("unreachable TPDO0 recovery loop")

    def read_positions(self) -> dict[int, int]:
        positions: dict[int, int] = {}
        for node in self.nodes:
            _, position = self.sdo_read(node, 0x6064, 0x00)
            positions[node] = position
        return positions

    def recover_interpolation_starvation(
        self,
        motion_direction: int,
        period_ms: int,
        warmup_samples: int,
        tpdo0_timeout_ms: int,
    ) -> tuple[int, float, int, dict[str, object]]:
        """Re-prime an empty interpolation stream without disabling an axis.

        A host/USB stall can empty all four BC2 interpolation buffers while
        the axes remain healthy and Operation Enabled.  Continuing to advance
        the target then grows following error but cannot restart execution.
        Clear the trigger edge, preload measured hold points, retrigger all
        axes together, and smoothly converge the small measured spread before
        resuming the original common target.
        """
        measured = {
            node: self.feedback[node].position for node in self.nodes
        }
        if self.emcy or any(
            self.feedback[node].fault != 0 for node in self.nodes
        ):
            raise RuntimeError(
                "interpolation starvation recovery refused because a real "
                "drive fault is present"
            )

        maximum_burst_us = 0.0
        tpdo0_recovery_count = 0
        self.send_interpolation_trigger_group(0x000F, measured)
        time.sleep(period_ms / 1000.0)
        for _ in range(warmup_samples):
            tick_start = time.perf_counter()
            burst_us, retries = self.send_stream_point_with_feedback(
                measured,
                tpdo0_timeout_ms,
            )
            maximum_burst_us = max(maximum_burst_us, burst_us)
            tpdo0_recovery_count += retries
            remaining = period_ms / 1000.0 - (
                time.perf_counter() - tick_start
            )
            if remaining > 0.0:
                time.sleep(remaining)

        trigger_baseline = {
            node: self.feedback[node].tpdo0_count for node in self.nodes
        }
        self.send_interpolation_trigger_group(0x003F, measured)
        self.wait_for_fresh_tpdo0(trigger_baseline, tpdo0_timeout_ms)

        common_position = (
            min(measured.values())
            if motion_direction < 0
            else max(measured.values())
        )
        maximum_alignment_distance = max(
            abs(common_position - measured[node])
            for node in self.nodes
        )
        alignment_samples = 0
        if maximum_alignment_distance > 0:
            alignment_duration_s = (
                1.5 * maximum_alignment_distance /
                (DEFAULT_PREALIGN_SPEED_MM_S * LIFT_COUNTS_PER_MM)
            )
            alignment_samples = max(
                3,
                math.ceil(
                    alignment_duration_s * 1000.0 / period_ms
                ),
            )
        for sample in range(alignment_samples):
            normalized_time = (sample + 1) / alignment_samples
            smooth_progress = (
                3.0 * normalized_time * normalized_time -
                2.0 * normalized_time * normalized_time * normalized_time
            )
            targets = {
                node: round(
                    measured[node] +
                    (common_position - measured[node]) * smooth_progress
                )
                for node in self.nodes
            }
            tick_start = time.perf_counter()
            burst_us, retries = self.send_stream_point_with_feedback(
                targets,
                tpdo0_timeout_ms,
            )
            maximum_burst_us = max(maximum_burst_us, burst_us)
            tpdo0_recovery_count += retries
            remaining = period_ms / 1000.0 - (
                time.perf_counter() - tick_start
            )
            if remaining > 0.0:
                time.sleep(remaining)

        return (
            common_position,
            maximum_burst_us,
            tpdo0_recovery_count,
            {
                "measured_positions": measured,
                "common_position": common_position,
                "alignment_samples": alignment_samples,
            },
        )

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
            absolute_target_mm: float | None,
            completion_timeout_ms: int,
            completion_tolerance_counts: int,
            completion_stable_samples: int,
            tpdo0_timeout_ms: int) -> dict[str, object]:
        if sample_count < 20 or (sample_count % 2) != 0:
            raise ValueError("sample-count must be even and at least 20")
        if period_ms != DEFAULT_PERIOD_MS:
            raise ValueError(
                f"period-ms must remain at the verified {DEFAULT_PERIOD_MS}ms"
            )
        if cycles < 1:
            raise ValueError("cycles must be >= 1")
        if speed_mm_s < 0.0 or acceleration_mm_s2 < 0.0:
            raise ValueError("speed and acceleration must be non-negative")
        if speed_mm_s > 0.0 and acceleration_mm_s2 <= 0.0:
            raise ValueError("positive --speed-mm-s requires --accel-mm-s2")
        if (
            tracking_window_mm <= 0.0
            or tracking_window_mm > LIFT_RUNNING_SPREAD_WARNING_MM
        ):
            raise ValueError(
                "running following-error window must be in "
                f"(0, {LIFT_RUNNING_SPREAD_WARNING_MM:g}] mm"
            )
        if completion_timeout_ms <= 0:
            raise ValueError("completion timeout must be positive")
        if completion_tolerance_counts <= 0:
            raise ValueError("completion tolerance must be positive")
        if completion_stable_samples < 1:
            raise ValueError("completion stable samples must be >= 1")
        if tpdo0_timeout_ms <= 0 or tpdo0_timeout_ms >= period_ms:
            raise ValueError("TPDO0 timeout must be positive and below period-ms")
        tracking_window_counts = math.ceil(
            tracking_window_mm * LIFT_COUNTS_PER_MM
        )
        following_error_guard_counts = (
            tracking_window_counts -
            round(FOLLOWING_ERROR_GUARD_MARGIN_MM * LIFT_COUNTS_PER_MM)
        )
        if max_following_lead_counts >= following_error_guard_counts:
            raise ValueError(
                "maximum following lead must remain below the tracking "
                "window guard"
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
            if (absolute_target_mm < LIFT_SAFE_MIN_HEIGHT_MM or
                    absolute_target_mm > LIFT_SAFE_MAX_HEIGHT_MM):
                raise ValueError(
                    "absolute target must be within "
                    f"{LIFT_SAFE_MIN_HEIGHT_MM:g}.."
                    f"{LIFT_SAFE_MAX_HEIGHT_MM:g} mm"
                )
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
            motion_direction = next(iter(directions), 1)
            initial_absolute_spread = max(starts.values()) - min(starts.values())
            if initial_absolute_spread > round(
                LIFT_FINAL_SYNC_SPREAD_MM * LIFT_COUNTS_PER_MM
            ):
                raise RuntimeError(
                    "axes must be leveled within 3mm before a normal common "
                    f"trajectory; measured spread is "
                    f"{initial_absolute_spread / LIFT_COUNTS_PER_MM:.3f}mm"
                )
            common_start_position = (
                min(starts.values())
                if motion_direction < 0 else max(starts.values())
            )
        else:
            final_deltas = {node: amplitude for node in self.nodes}
            motion_direction = next(
                (
                    1 if delta > 0 else -1
                    for delta in final_deltas.values()
                    if delta != 0
                ),
                1,
            )
            initial_absolute_spread = 0
            common_start_position = round(
                sum(starts.values()) / len(starts)
            )
        motion_distance_counts = (
            abs(target_count - common_start_position)
            if absolute_target_mm is not None
            else max(abs(delta) for delta in final_deltas.values())
        )
        if motion_distance_counts == 0:
            raise ValueError("absolute target is already reached on every axis")
        next_tick = time.perf_counter()
        max_burst_us = 0.0
        max_relative_spread = 0

        if warmup_samples < 3:
            raise ValueError("warmup-samples is the preload depth and must be >= 3")
        prealign_sample_count = 0
        if absolute_target_mm is not None:
            maximum_prealign_distance_counts = max(
                abs(common_start_position - starts[node])
                for node in self.nodes
            )
            if maximum_prealign_distance_counts > 0:
                # Cubic smoothstep has a peak normalized slope of 1.5.  Size
                # the alignment interval from that peak so no axis exceeds
                # the configured 2 mm/s pre-alignment speed.  This stage only
                # removes the initial <=3 mm spread in the requested motion
                # direction; it does not chase feedback during the main run.
                prealign_duration_s = (
                    1.5 * maximum_prealign_distance_counts /
                    (DEFAULT_PREALIGN_SPEED_MM_S * LIFT_COUNTS_PER_MM)
                )
                prealign_sample_count = max(
                    3,
                    math.ceil(prealign_duration_s * 1000.0 / period_ms),
                )
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
        minimum_total_samples = (
            warmup_samples + prealign_sample_count +
            active_samples + hold_samples + 1
        )
        maximum_total_samples = (
            minimum_total_samples +
            math.ceil(completion_timeout_ms / period_ms)
        )
        final_tolerance_counts = completion_tolerance_counts
        completion_stable_count = 0
        completed_final_target = False
        next_tick = 0.0
        maximum_tick_lateness_us = 0.0
        tick_resync_count = 0
        tpdo0_recovery_count = 0
        previous_desired_delta = 0
        trajectory_direction = -1
        running_spread_warning_count = 0
        previous_common_progress = (
            motion_direction * common_start_position
        )
        common_velocity_counts_s = 0.0
        feedback_governor_count = 0
        maximum_following_error_counts = 0
        starvation_candidate_samples = 0
        starvation_recovery_count = 0
        starvation_recovery_events: list[dict[str, object]] = []
        period_s = period_ms / 1000.0
        final_common_progress = (
            motion_direction * target_count
            if absolute_target_mm is not None else previous_common_progress
        )
        # The complete point-by-point evidence is intentionally retained in
        # memory until the move finishes.  CPython's cyclic collector can pause
        # for hundreds of milliseconds once that list becomes large, starving
        # a 20 ms interpolation stream even though no cyclic objects are
        # created in the realtime loop.  Collect once before motion, then keep
        # cyclic GC disabled until the axes have stopped and their brakes hold.
        # Reference counting remains active, and this hazardous tool exits
        # after one leg.
        gc.collect()
        gc_was_enabled = gc.isenabled()
        if gc_was_enabled:
            gc.disable()
        for sample_index in range(maximum_total_samples):
            if sample_index == warmup_samples:
                # Manual-required start order: fill at least three segments,
                # then create the bit-4 rising edge on every axis through the
                # same synchronous RPDO1 path used by the ECU firmware.
                trigger_baseline = {
                    node: self.feedback[node].tpdo0_count
                    for node in self.nodes
                }
                self.send_interpolation_trigger_group(0x003F, starts)
                self.wait_for_fresh_tpdo0(
                    trigger_baseline,
                    tpdo0_timeout_ms,
                )
                next_tick = time.perf_counter() + period_ms / 1000.0

            if sample_index >= warmup_samples:
                now = time.perf_counter()
                remaining = next_tick - now
                if remaining > 0:
                    time.sleep(remaining)
                    tick_start_time = next_tick
                else:
                    # A Windows USB/CAN call can occasionally exceed the 20 ms
                    # period.  Never "catch up" by emitting two trajectory
                    # points back-to-back: that compresses the interpolation
                    # time base and is felt as a mechanical jerk.  Drop only
                    # the elapsed host time and rebuild the next deadline from
                    # the current tick; the absolute target is not skipped.
                    tick_start_time = now
                    lateness_us = -remaining * 1_000_000.0
                    maximum_tick_lateness_us = max(
                        maximum_tick_lateness_us,
                        lateness_us,
                    )
                    if lateness_us >= period_ms * 1000.0:
                        tick_resync_count += 1
                next_tick = tick_start_time + period_s

            active_index = sample_index - warmup_samples
            motion_active_index = active_index - prealign_sample_count
            if active_index < 0:
                # Preload stationary points at the measured current absolute
                # position.  This keeps the interpolation buffer primed without
                # asking the first executed segment to jump away from the
                # physical start position when bit 4 is finally triggered.
                desired_delta = 0
            elif motion_active_index < 0:
                desired_delta = 0
            elif motion_active_index >= active_samples:
                desired_delta = (
                    motion_direction * motion_distance_counts if one_way else 0
                )
            elif trapezoid is not None:
                t_s = min(
                    motion_active_index * period_ms / 1000.0,
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
                desired_delta = round(
                    amplitude * motion_active_index / sample_count
                )
            else:
                desired_delta = self.triangle_delta(
                    amplitude,
                    motion_active_index % sample_count,
                    sample_count,
                )
            if active_index < 0:
                # Preload each measured position so no axis jumps before the
                # common interpolation trigger.
                targets = starts.copy()
                target_deltas = {node: 0 for node in self.nodes}
                slowest_progress = previous_common_progress
                ideal_common_progress = previous_common_progress
            elif motion_active_index < 0:
                # Smoothly converge every axis from its measured start to the
                # direction-leading common start.  Sending a common absolute
                # point immediately would turn an allowed initial spread into
                # a position step and can trigger 0x7390.
                normalized_time = (
                    (active_index + 1) / prealign_sample_count
                )
                smooth_progress = (
                    3.0 * normalized_time * normalized_time -
                    2.0 * normalized_time * normalized_time * normalized_time
                )
                targets = {
                    node: round(
                        starts[node] +
                        (common_start_position - starts[node]) *
                        smooth_progress
                    )
                    for node in self.nodes
                }
                target_deltas = {
                    node: targets[node] - starts[node]
                    for node in self.nodes
                }
                slowest_progress = previous_common_progress
                ideal_common_progress = previous_common_progress
            elif absolute_target_mm is not None:
                # All axes receive exactly the same absolute point.  The
                # common online trajectory accelerates and decelerates from
                # its own last transmitted point.  It therefore cannot finish
                # its time schedule while a feedback-lead clamp is active, and
                # it never accumulates a later catch-up jump.  The slowest
                # measured axis still limits the complete group's lead.
                measured_progress = {
                    node: motion_direction * self.feedback[node].position
                    for node in self.nodes
                }
                slowest_progress = min(measured_progress.values())
                remaining_progress = max(
                    0.0,
                    final_common_progress - previous_common_progress,
                )
                braking_velocity = math.sqrt(
                    max(0.0, 2.0 * acceleration_counts_s2 *
                        remaining_progress)
                )
                requested_velocity = min(
                    speed_counts_s,
                    braking_velocity,
                )
                current_following_lead = max(
                    0.0,
                    previous_common_progress - slowest_progress,
                )
                if current_following_lead > max_following_lead_counts:
                    governor_span = (
                        following_error_guard_counts -
                        max_following_lead_counts
                    )
                    governor_fraction = max(
                        0.0,
                        min(
                            1.0,
                            (
                                following_error_guard_counts -
                                current_following_lead
                            ) / governor_span,
                        ),
                    )
                    minimum_governed_velocity = min(
                        speed_counts_s,
                        DEFAULT_FEEDBACK_GOVERNOR_MIN_SPEED_MM_S *
                        LIFT_COUNTS_PER_MM,
                    )
                    requested_velocity = min(
                        requested_velocity,
                        minimum_governed_velocity +
                        (
                            speed_counts_s -
                            minimum_governed_velocity
                        ) * governor_fraction,
                    )
                    feedback_governor_count += 1
                velocity_delta = acceleration_counts_s2 * period_s
                if common_velocity_counts_s < requested_velocity:
                    common_velocity_counts_s = min(
                        requested_velocity,
                        common_velocity_counts_s + velocity_delta,
                    )
                else:
                    common_velocity_counts_s = max(
                        requested_velocity,
                        common_velocity_counts_s - velocity_delta,
                    )
                requested_step = min(
                    remaining_progress,
                    common_velocity_counts_s * period_s,
                )
                unlimited_progress = (
                    previous_common_progress + requested_step
                )
                # Never freeze a fixed-period interpolation stream at the
                # feedback boundary.  Repeating one point can leave BC/BC2
                # stopped one buffer depth behind it.  The velocity governor
                # above continuously reduces the slope as following error
                # approaches the configured tracking window.
                common_progress = max(
                    previous_common_progress,
                    unlimited_progress,
                )
                common_position = (
                    motion_direction * round(common_progress)
                )
                targets = {node: common_position for node in self.nodes}
                target_deltas = {
                    node: common_position - starts[node]
                    for node in self.nodes
                }
                previous_common_progress = common_progress
                ideal_common_progress = final_common_progress
            else:
                progress = min(
                    1.0,
                    abs(desired_delta) / motion_distance_counts,
                )
                target_deltas = {
                    node: round(final_deltas[node] * progress)
                    for node in self.nodes
                }
                targets = {
                    node: starts[node] + target_deltas[node]
                    for node in self.nodes
                }
                slowest_progress = 0
                ideal_common_progress = 0

            previous_desired_delta = desired_delta
            implausible_targets = {
                node: target for node, target in targets.items()
                if not self.position_is_mechanically_plausible(target)
            }
            if implausible_targets:
                raise RuntimeError(
                    "target outside mechanical plausibility range: "
                    f"{implausible_targets}"
                )

            burst_us, recovery_attempts = (
                self.send_stream_point_with_feedback(
                    targets,
                    tpdo0_timeout_ms,
                )
            )
            max_burst_us = max(max_burst_us, burst_us)
            tpdo0_recovery_count += recovery_attempts
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
            observed_spread = (
                absolute_spread
                if absolute_target_mm is not None else relative_spread
            )
            if absolute_target_mm is not None and motion_active_index >= 0:
                following_errors = [
                    max(
                        0,
                        motion_direction * (
                            targets[node] -
                            self.feedback[node].position
                        ),
                    )
                    for node in self.nodes
                ]
                maximum_following_error_counts = max(
                    maximum_following_error_counts,
                    max(following_errors),
                )
                starvation_speed_limit = round(
                    INTERPOLATION_STARVATION_MAX_SPEED_RPM *
                    LIFT_COUNTS_PER_MOTOR_REV * 10.0 / 60.0
                )
                if (
                    max(following_errors) > max_following_lead_counts
                    and all(
                        abs(self.feedback[node].velocity)
                        <= starvation_speed_limit
                        for node in self.nodes
                    )
                    and not self.emcy
                    and all(
                        self.feedback[node].fault == 0
                        for node in self.nodes
                    )
                ):
                    starvation_candidate_samples += 1
                else:
                    starvation_candidate_samples = 0
                if (
                    starvation_candidate_samples >=
                    INTERPOLATION_STARVATION_CONFIRM_SAMPLES
                ):
                    if (
                        starvation_recovery_count >=
                        INTERPOLATION_STARVATION_RECOVERY_LIMIT
                    ):
                        raise RuntimeError(
                            "interpolation starvation recovery limit "
                            "exceeded"
                        )
                    (
                        recovered_common_position,
                        recovery_burst_us,
                        recovery_feedback_retries,
                        recovery_event,
                    ) = self.recover_interpolation_starvation(
                        motion_direction,
                        period_ms,
                        warmup_samples,
                        tpdo0_timeout_ms,
                    )
                    starvation_recovery_count += 1
                    recovery_event.update(
                        {
                            "sample_index": sample_index,
                            "following_error_mm": round(
                                max(following_errors) /
                                LIFT_COUNTS_PER_MM,
                                3,
                            ),
                        }
                    )
                    starvation_recovery_events.append(recovery_event)
                    max_burst_us = max(max_burst_us, recovery_burst_us)
                    tpdo0_recovery_count += recovery_feedback_retries
                    previous_common_progress = (
                        motion_direction * recovered_common_position
                    )
                    common_velocity_counts_s = 0.0
                    starvation_candidate_samples = 0
                    completion_stable_count = 0
                    next_tick = (
                        time.perf_counter() +
                        period_ms / 1000.0
                    )
                    continue
                if max(following_errors) > tracking_window_counts:
                    raise RuntimeError(
                        "measured following error exceeded configured "
                        f"{tracking_window_mm:.3f}mm tracking window: "
                        f"{max(following_errors) / LIFT_COUNTS_PER_MM:.3f}mm"
                    )
            max_relative_spread = max(max_relative_spread, observed_spread)
            if observed_spread > round(
                LIFT_RUNNING_SPREAD_WARNING_MM * LIFT_COUNTS_PER_MM
            ):
                # Running spread is diagnostic evidence and a correction
                # input, not an automatic disable.  Stopping four loaded legs
                # at unequal heights would preserve the skew and make the next
                # start less stable.  Final acceptance remains strictly 3 mm.
                running_spread_warning_count += 1
            self.samples.append(
                {
                    "index": sample_index,
                    "phase": (
                        "preload"
                        if sample_index < warmup_samples
                        else "align"
                        if motion_active_index < 0
                        else "hold"
                        if motion_active_index >= active_samples
                        else "stream"
                    ),
                    "active_index": active_index,
                    "motion_active_index": motion_active_index,
                    "desired_delta": desired_delta,
                    "target_deltas": target_deltas,
                    "common_target": next(iter(targets.values())),
                    "slowest_progress": slowest_progress,
                    "ideal_common_progress": ideal_common_progress,
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

            if motion_active_index >= active_samples:
                final_errors = [
                    abs(self.feedback[node].position -
                        target_count)
                    for node in self.nodes
                    if self.feedback[node].tpdo0_count > 0
                ]
                final_spread = (
                    max(absolute_positions) - min(absolute_positions)
                    if len(absolute_positions) == len(self.nodes) else
                    2**31 - 1
                )
                zero_speed = all(
                    abs(self.feedback[node].velocity)
                    <= LIFT_ZERO_SPEED_VELOCITY_UNITS
                    for node in self.nodes
                )
                if (len(final_errors) == len(self.nodes) and
                        max(final_errors) <= final_tolerance_counts and
                        final_spread <= round(
                            LIFT_FINAL_SYNC_SPREAD_MM * LIFT_COUNTS_PER_MM
                        ) and zero_speed):
                    completion_stable_count += 1
                else:
                    completion_stable_count = 0

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
            tpdo0_baseline = {
                node: self.feedback[node].tpdo0_count for node in self.nodes
            }
            self.send(SYNC_COB_ID, b"")
            self.wait_for_fresh_tpdo0(tpdo0_baseline, tpdo0_timeout_ms)
            remaining = period_ms / 1000.0 - (time.perf_counter() - tick_start)
            if remaining > 0:
                time.sleep(remaining)

        self.stop_interpolation()
        time.sleep(0.5)
        if gc_was_enabled:
            gc.enable()
        motion_current_stats: dict[str, object] = {}
        for node in self.nodes:
            current_samples = [
                int(sample["feedback"][str(node)]["current"])
                for sample in self.samples
                if sample["phase"] in ("align", "stream")
            ]
            average_absolute_current_units = (
                sum(abs(value) for value in current_samples) /
                len(current_samples)
            )
            maximum_absolute_current_units = max(
                abs(value) for value in current_samples
            )
            motion_current_stats[str(node)] = {
                "sample_count": len(current_samples),
                "average_absolute_current_units": round(
                    average_absolute_current_units,
                    3,
                ),
                "maximum_absolute_current_units": (
                    maximum_absolute_current_units
                ),
                "average_absolute_current_a": round(
                    average_absolute_current_units *
                    ACTUAL_CURRENT_AMPS_PER_UNIT,
                    3,
                ),
                "maximum_absolute_current_a": round(
                    maximum_absolute_current_units *
                    ACTUAL_CURRENT_AMPS_PER_UNIT,
                    3,
                ),
            }
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
            "prealign_sample_count": prealign_sample_count,
            "prealign_speed_mm_s": DEFAULT_PREALIGN_SPEED_MM_S,
            "enable_settle_ms": enable_settle_ms,
            "profile_velocity_units": profile_velocity_units,
            "profile_acceleration": profile_acceleration,
            "max_following_lead_counts": max_following_lead_counts,
            "tracking_window_mm": tracking_window_mm,
            "completion_timeout_ms": completion_timeout_ms,
            "completion_tolerance_counts": completion_tolerance_counts,
            "effective_completion_tolerance_counts": final_tolerance_counts,
            "completion_stable_samples": completion_stable_samples,
            "tpdo0_timeout_ms": tpdo0_timeout_ms,
            "completed_final_target": completed_final_target,
            "counts_per_mm": LIFT_COUNTS_PER_MM,
            "speed_mm_s": speed_mm_s,
            "acceleration_mm_s2": acceleration_mm_s2,
            "requested_motor_rpm": round(
                speed_mm_s * LIFT_MOTOR_REVS_PER_MM * 60.0,
                3,
            ),
            "trapezoid": trapezoid,
            "max_relative_spread_mm": round(
                max_relative_spread / LIFT_COUNTS_PER_MM,
                3,
            ),
            "max_burst_us": round(max_burst_us, 1),
            "maximum_tick_lateness_us": round(
                maximum_tick_lateness_us,
                1,
            ),
            "tick_resync_count": tick_resync_count,
            "tpdo0_recovery_count": tpdo0_recovery_count,
            "tpdo0_recovery_retry_limit": DEFAULT_TPDO0_RECOVERY_RETRIES,
            "feedback_governor_count": feedback_governor_count,
            "starvation_recovery_count": starvation_recovery_count,
            "starvation_recovery_events": starvation_recovery_events,
            "maximum_following_error_counts": maximum_following_error_counts,
            "maximum_following_error_mm": round(
                maximum_following_error_counts / LIFT_COUNTS_PER_MM,
                3,
            ),
            "following_error_guard_mm": round(
                following_error_guard_counts / LIFT_COUNTS_PER_MM,
                3,
            ),
            "minimum_governed_speed_mm_s": (
                DEFAULT_FEEDBACK_GOVERNOR_MIN_SPEED_MM_S
            ),
            "max_relative_spread": max_relative_spread,
            "running_spread_warning_mm": LIFT_RUNNING_SPREAD_WARNING_MM,
            "running_spread_warning_count": running_spread_warning_count,
            "emcy": self.emcy,
            "emcy_reset_count": self.emcy_reset_count,
            "current_limit_emcy_count": self.current_limit_emcy_count,
            "current_limit_emcy_by_node": self.current_limit_emcy_by_node,
            "motion_current_stats": motion_current_stats,
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
        help=(
            "complete four-axis group in physical leg order; "
            "subsets are rejected"
        ),
    )
    parser.add_argument("--period-ms", type=int, default=DEFAULT_PERIOD_MS)
    # The drive needs at least three buffered points.  Keeping exactly three
    # limits the 20 ms feedback-control horizon to 60 ms instead of allowing
    # a long preloaded trajectory to continue after measured axes diverge.
    parser.add_argument("--warmup-samples", type=int, default=3)
    parser.add_argument("--hold-samples", type=int, default=10)
    parser.add_argument("--enable-settle-ms", type=int, default=1500)
    parser.add_argument(
        "--profile-velocity-units",
        type=int,
        default=DEFAULT_PROFILE_VELOCITY_UNITS,
    )
    parser.add_argument(
        "--profile-acceleration",
        type=int,
        default=DEFAULT_PROFILE_ACCELERATION_UNITS,
    )
    parser.add_argument(
        "--max-following-lead-counts",
        type=int,
        default=DEFAULT_MAX_FOLLOWING_LEAD_COUNTS,
    )
    parser.add_argument(
        "--tracking-window-mm",
        type=float,
        default=DEFAULT_TRACKING_WINDOW_MM,
    )
    parser.add_argument(
        "--absolute-target-mm",
        type=float,
        default=None,
        help="required for real motion; common absolute target in 10..490 mm",
    )
    parser.add_argument(
        "--completion-timeout-ms",
        type=int,
        default=30_000,
        help="maximum extra stream time while measured axes close the final lead",
    )
    parser.add_argument(
        "--completion-tolerance-mm",
        type=float,
        default=DEFAULT_COMPLETION_TOLERANCE_MM,
        help=(
            "per-axis final-position tolerance; final four-axis spread uses "
            "the same 3mm contract"
        ),
    )
    parser.add_argument(
        "--completion-stable-samples",
        type=int,
        default=DEFAULT_COMPLETION_STABLE_SAMPLES,
        help="consecutive fresh, zero-speed TPDO0 groups required at the final target",
    )
    parser.add_argument(
        "--speed-mm-s",
        type=float,
        default=DEFAULT_SPEED_MM_S,
        help="trapezoid maximum leg speed",
    )
    parser.add_argument(
        "--accel-mm-s2",
        type=float,
        default=DEFAULT_ACCEL_MM_S2,
        help="trapezoid leg acceleration/deceleration",
    )
    parser.add_argument(
        "--tpdo0-timeout-ms",
        type=int,
        default=DEFAULT_TPDO0_TIMEOUT_MS,
        help="maximum post-SYNC wait for a new TPDO0 from every axis",
    )
    parser.add_argument("--timeout-ms", type=int, default=1000)
    parser.add_argument("--log", default="")
    parser.add_argument("--allow-motion", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        nodes = tuple(int(item) for item in args.nodes.split(",") if item)
    except ValueError:
        print("FAILED: --nodes must contain decimal node IDs", file=sys.stderr)
        return 2
    if nodes != LIFT_NODES:
        print(
            f"FAILED: motion requires the complete ordered group {LIFT_NODES}",
            file=sys.stderr,
        )
        return 2
    if args.absolute_target_mm is not None and not (
        LIFT_SAFE_MIN_HEIGHT_MM
        <= args.absolute_target_mm
        <= LIFT_SAFE_MAX_HEIGHT_MM
    ):
        print("FAILED: target must be within 10..490 mm", file=sys.stderr)
        return 2
    if (
        args.period_ms != DEFAULT_PERIOD_MS
        or args.speed_mm_s <= 0.0
        or args.accel_mm_s2 <= 0.0
        or args.tracking_window_mm <= 0.0
        or args.tracking_window_mm > LIFT_RUNNING_SPREAD_WARNING_MM
        or args.completion_tolerance_mm <= 0.0
        or args.completion_tolerance_mm > LIFT_FINAL_SYNC_SPREAD_MM
        or args.tpdo0_timeout_ms <= 0
        or args.tpdo0_timeout_ms >= args.period_ms
    ):
        print(
            "FAILED: invalid verified trajectory or feedback parameter",
            file=sys.stderr,
        )
        return 2
    required_profile_velocity = math.ceil(
        args.speed_mm_s * LIFT_COUNTS_PER_MM * CANOPEN_PROFILE_VELOCITY_SCALE
    )
    required_profile_acceleration = math.ceil(
        args.accel_mm_s2 * LIFT_COUNTS_PER_MM *
        CANOPEN_PROFILE_ACCELERATION_SCALE
    )
    if (
        args.profile_velocity_units < required_profile_velocity
        or args.profile_acceleration < required_profile_acceleration
    ):
        print(
            "FAILED: drive profile limits are below the requested trajectory",
            file=sys.stderr,
        )
        return 2
    plan = {
        "dry_run": not args.allow_motion,
        "nodes": nodes,
        "target_mm": args.absolute_target_mm,
        "counts_per_motor_rev": LIFT_COUNTS_PER_MOTOR_REV,
        "motor_revs_per_mm": LIFT_MOTOR_REVS_PER_MM,
        "counts_per_mm": LIFT_COUNTS_PER_MM,
        "safe_range_mm": [LIFT_SAFE_MIN_HEIGHT_MM, LIFT_SAFE_MAX_HEIGHT_MM],
        "speed_mm_s": args.speed_mm_s,
        "acceleration_mm_s2": args.accel_mm_s2,
        "period_ms": args.period_ms,
        "profile_velocity_units": args.profile_velocity_units,
        "profile_acceleration_units": args.profile_acceleration,
        "max_following_lead_counts": args.max_following_lead_counts,
        "tracking_window_mm": args.tracking_window_mm,
        "final_spread_mm": LIFT_FINAL_SYNC_SPREAD_MM,
        "running_spread_warning_mm": LIFT_RUNNING_SPREAD_WARNING_MM,
        "tpdo0_timeout_ms": args.tpdo0_timeout_ms,
    }
    if not args.allow_motion:
        print(json.dumps(plan, ensure_ascii=False, indent=2))
        print(
            "DRY-RUN: CAN was not opened; add --absolute-target-mm and "
            "--allow-motion only after the ECU is disconnected."
        )
        return 0
    if args.absolute_target_mm is None:
        print(
            "FAILED: --absolute-target-mm is required with --allow-motion",
            file=sys.stderr,
        )
        return 2
    print(json.dumps(plan, ensure_ascii=False, indent=2), flush=True)
    print(
        "MOTION ENABLED: verify ECU CAN3 is disconnected and all four legs "
        "are mechanically clear.",
        flush=True,
    )
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
                0,
                args.period_ms,
                200,
                args.warmup_samples,
                args.hold_samples,
                args.enable_settle_ms,
                args.profile_velocity_units,
                args.profile_acceleration,
                args.max_following_lead_counts,
                True,
                1,
                args.speed_mm_s,
                args.accel_mm_s2,
                args.tracking_window_mm,
                args.absolute_target_mm,
                args.completion_timeout_ms,
                round(args.completion_tolerance_mm * LIFT_COUNTS_PER_MM),
                args.completion_stable_samples,
                args.tpdo0_timeout_ms,
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
