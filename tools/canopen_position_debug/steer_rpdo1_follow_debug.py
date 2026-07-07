"""Steering RPDO1 realtime profile-position follow debug helper.

This bench tool validates whether BC/BC2 steering axes can follow frequently
updated absolute position targets in CiA-402 profile-position mode.  It avoids
RPDO2 interpolation and uses the already reliable RPDO1 mapping:

    0x6040 controlword + 0x6060 mode + 0x607A target position

For the realtime validation phase the tool can also temporarily remap RPDO1 to
a compact six-byte payload:

    0x6040 controlword + 0x607A target position

The compact mapping leaves 0x6060 fixed at profile-position mode after setup,
which reduces bus bandwidth and avoids rewriting the mode object every cycle.

Safety defaults:
  * no NMT reset frames;
  * no Flash/NVM save;
  * steering nodes only (5-8);
  * no drive-wheel velocity command;
  * all temporary profile parameters are RAM-only.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
PDO_TOOL_DIR = REPO_ROOT / "tools" / "canopen_pdo_config"
CAN_TOOL_DIR = REPO_ROOT / "tools" / "can"
for path in (PDO_TOOL_DIR, CAN_TOOL_DIR):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from can_adapter import CanFrame  # noqa: E402
from can_adapter_controlcan import ControlCanAdapter  # noqa: E402


BUS = "can1"
SYNC_COB_ID = 0x080
NMT_COB_ID = 0x000
STEER_NODES = (5, 6, 7, 8)
RPDO_MAP_CURRENT7 = "current7"
RPDO_MAP_COMPACT6 = "compact6"
RPDO_TX_SYNC1 = "sync1"
RPDO_TX_ASYNC255 = "async255"
WAVEFORM_SINE = "sine"
WAVEFORM_JOYSTICK = "joystick"
WAVEFORM_TRIANGLE = "triangle"
WAVEFORM_STEP = "step"
WAVEFORM_REMOTE_STRESS = "remote_stress"
WAVEFORM_CENTER_NOISE = "center_noise"


class SteerPositionDebug:
    def __init__(self, timeout_ms: int, log_dir: Path) -> None:
        self.timeout_ms = timeout_ms
        self.log_dir = log_dir
        self.adapter = ControlCanAdapter()
        self.events: list[dict[str, object]] = []

    def __enter__(self) -> "SteerPositionDebug":
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

    def drain_feedback_nonblocking(self, nodes: list[int]) -> list[dict[str, object]]:
        """Collect already-available TPDO/EMCY frames without disturbing timing.

        ControlCAN's blocking receive helper is correct for SDO transactions but
        unsuitable inside a 20 ms realtime command loop.  This method drains only
        frames that are already buffered by the adapter or the hardware driver.
        It never waits for a missing frame, so command scheduling remains the
        priority.
        """

        wanted = set()
        for node in nodes:
            wanted.update((0x180 + node, 0x280 + node, 0x080 + node))

        frames: list[CanFrame] = []
        cached = self.adapter._rx_cache[BUS]  # type: ignore[attr-defined] # Debug tool; avoids blocking API.
        kept_cache: list[CanFrame] = []
        for frame in cached:
            if frame.can_id in wanted:
                frames.append(frame)
            else:
                kept_cache.append(frame)
        cached[:] = kept_cache

        channel = self.adapter._channels[BUS]  # type: ignore[attr-defined] # Debug tool; direct nonblocking poll.
        for raw in self.adapter._device.receive(channel, limit=200, wait_ms=0):  # type: ignore[attr-defined]
            data = bytes(int(raw.Data[i]) for i in range(int(raw.DataLen)))
            frame = CanFrame(
                int(raw.ID),
                data,
                is_extended=bool(raw.ExternFlag),
                is_remote=bool(raw.RemoteFlag),
            )
            if frame.can_id in wanted:
                frames.append(frame)
            else:
                cached.append(frame)

        records: list[dict[str, object]] = []
        for frame in frames:
            decoded = self.decode_feedback(frame)
            record = {
                "time_s": time.time(),
                "can_id": f"0x{frame.can_id:03X}",
                "node": self.node_from_feedback_id(frame.can_id),
                "data": frame.data.hex(" ").upper(),
                "decoded": decoded,
            }
            records.append(record)
            self.event("rx_observe", **record)
        return records

    def flush_feedback_buffers(self, nodes: list[int], duration_ms: int) -> None:
        """Discard stale TPDO/EMCY frames before a measured motion segment."""

        deadline = time.monotonic() + duration_ms / 1000.0
        while time.monotonic() < deadline:
            self.drain_feedback_nonblocking(nodes)
            time.sleep(0.002)

    def collect_feedback(self, nodes: list[int], duration_ms: int) -> list[dict[str, object]]:
        expected_ids: list[int] = []
        for node in nodes:
            expected_ids.extend([0x180 + node, 0x280 + node, 0x080 + node])
        deadline = time.monotonic() + duration_ms / 1000.0
        frames: list[dict[str, object]] = []
        while time.monotonic() < deadline:
            made_progress = False
            for can_id in expected_ids:
                try:
                    frame = self.adapter.receive(BUS, can_id, 1)
                except TimeoutError:
                    continue
                decoded = self.decode_feedback(frame)
                record = {
                    "can_id": f"0x{frame.can_id:03X}",
                    "node": self.node_from_feedback_id(frame.can_id),
                    "data": frame.data.hex(" ").upper(),
                    "decoded": decoded,
                }
                frames.append(record)
                self.event("rx_observe", **record)
                made_progress = True
            if not made_progress:
                time.sleep(0.001)
        return frames

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
            raise RuntimeError(f"Node{node} SDO abort write {note} 0x{index:04X}:{subindex} abort=0x{abort_code:08X}")
        expected = bytes([0x60, index & 0xFF, (index >> 8) & 0xFF, subindex, 0, 0, 0, 0])
        if ack.data != expected:
            raise RuntimeError(f"Node{node} unexpected SDO write ack {ack.data.hex(' ').upper()}")

    def sdo_upload(self, node: int, index: int, subindex: int, size_hint: int, note: str) -> dict[str, object]:
        self.send(self.sdo_upload_frame(node, index, subindex), f"Node{node} SDO read {note}")
        frame = self.receive_expected(0x580 + node)
        if frame.data[0] == 0x80:
            abort_code = int.from_bytes(frame.data[4:8], "little")
            return {"status": "abort", "abort": f"0x{abort_code:08X}", "note": note}
        size_by_command = {0x4F: 1, 0x4B: 2, 0x43: 4}
        size = size_by_command.get(frame.data[0], size_hint)
        raw = frame.data[4:4 + size]
        unsigned = int.from_bytes(raw, "little", signed=False)
        signed_value = int.from_bytes(raw, "little", signed=True)
        return {
            "status": "ok",
            "note": note,
            "size": size,
            "value": unsigned,
            "signed_value": signed_value,
            "hex": f"0x{unsigned:0{size * 2}X}",
        }

    def nmt_operational(self, node: int) -> None:
        self.send(CanFrame(NMT_COB_ID, bytes([0x01, node & 0x7F])), f"Node{node} NMT operational")

    def nmt_preoperational(self, node: int) -> None:
        self.send(CanFrame(NMT_COB_ID, bytes([0x80, node & 0x7F])), f"Node{node} NMT pre-operational")

    def sync(self) -> None:
        self.send(CanFrame(SYNC_COB_ID, b""), "SYNC")

    @staticmethod
    def rpdo1_payload(controlword: int, target_counts: int, rpdo_map: str) -> bytes:
        if rpdo_map == RPDO_MAP_COMPACT6:
            return (
                int(controlword).to_bytes(2, "little")
                + int(target_counts).to_bytes(4, "little", signed=True)
            )
        return (
            int(controlword).to_bytes(2, "little")
            + bytes([0x01])
            + int(target_counts).to_bytes(4, "little", signed=True)
        )

    def send_rpdo1(self, node: int, controlword: int, target_counts: int, rpdo_map: str) -> None:
        self.send(
            CanFrame(0x300 + node, self.rpdo1_payload(controlword, target_counts, rpdo_map)),
            f"Node{node} RPDO1 cw=0x{controlword:04X} target={target_counts}",
        )

    def read_node_state(self, nodes: list[int], filename: str) -> dict[str, object]:
        result: dict[str, object] = {}
        objects = (
            (0x2300, 0x00, 2, "control_source"),
            (0x6061, 0x00, 1, "mode_display"),
            (0x6041, 0x00, 2, "statusword"),
            (0x6064, 0x00, 4, "actual_position"),
            (0x606C, 0x00, 4, "actual_velocity"),
            (0x2183, 0x00, 4, "latched_fault"),
            (0x60F4, 0x00, 4, "following_error"),
            (0x6081, 0x00, 4, "profile_velocity"),
            (0x6083, 0x00, 4, "profile_acceleration"),
            (0x6084, 0x00, 4, "profile_deceleration"),
        )
        for node in nodes:
            node_values: dict[str, object] = {}
            for index, subindex, size, note in objects:
                node_values[f"0x{index:04X}:{subindex}"] = self.sdo_upload(node, index, subindex, size, note)
            result[str(node)] = node_values
        self.write_json(filename, result)
        return result

    @staticmethod
    def rpdo_transmission_value(rpdo_transmission: str) -> int:
        if rpdo_transmission == RPDO_TX_SYNC1:
            return 1
        if rpdo_transmission == RPDO_TX_ASYNC255:
            return 255
        raise ValueError(f"unsupported RPDO transmission type: {rpdo_transmission}")

    def configure_rpdo1_mapping(self, nodes: list[int], rpdo_map: str, rpdo_transmission: str) -> None:
        if rpdo_map not in (RPDO_MAP_CURRENT7, RPDO_MAP_COMPACT6):
            raise ValueError(f"unsupported RPDO1 map: {rpdo_map}")
        transmission_type = self.rpdo_transmission_value(rpdo_transmission)

        for node in nodes:
            rpdo1 = 0x300 + node
            self.nmt_preoperational(node)
            time.sleep(0.02)
            self.sdo_download(node, 0x1401, 0x01, 4, 0x80000000 | rpdo1, "disable RPDO1 COB-ID")
            self.sdo_download(node, 0x1401, 0x02, 1, transmission_type, f"RPDO1 transmission {rpdo_transmission}")
            self.sdo_download(node, 0x1601, 0x00, 1, 0, "clear RPDO1 map")
            self.sdo_download(node, 0x1601, 0x01, 4, 0x60400010, "RPDO1 controlword")
            if rpdo_map == RPDO_MAP_CURRENT7:
                self.sdo_download(node, 0x1601, 0x02, 4, 0x60600008, "RPDO1 mode")
                self.sdo_download(node, 0x1601, 0x03, 4, 0x607A0020, "RPDO1 target position")
                self.sdo_download(node, 0x1601, 0x00, 1, 3, "enable current RPDO1 map")
            else:
                self.sdo_download(node, 0x1601, 0x02, 4, 0x607A0020, "RPDO1 target position")
                self.sdo_download(node, 0x1601, 0x00, 1, 2, "enable compact RPDO1 map")
            self.sdo_download(node, 0x1401, 0x01, 4, rpdo1, "enable RPDO1 COB-ID")

    def fault_reset_axis(self, node: int) -> None:
        """Apply a standard CiA-402 controlword reset/re-enable sequence."""

        self.sdo_download(node, 0x6040, 0x00, 2, 0x0000, "disable voltage before fault reset")
        time.sleep(0.05)
        self.sdo_download(node, 0x6040, 0x00, 2, 0x0080, "fault reset")
        time.sleep(0.10)
        self.sdo_download(node, 0x6040, 0x00, 2, 0x0006, "shutdown")
        time.sleep(0.05)
        self.sdo_download(node, 0x6040, 0x00, 2, 0x0007, "switch on")
        time.sleep(0.05)

    def setup_position_mode(self, nodes: list[int], profile_velocity: int, profile_accel: int,
                            rpdo_map: str, rpdo_transmission: str, fault_reset: bool) -> None:
        self.configure_rpdo1_mapping(nodes, rpdo_map, rpdo_transmission)
        for node in nodes:
            self.nmt_operational(node)
            time.sleep(0.02)
            if fault_reset:
                self.fault_reset_axis(node)
            self.sdo_download(node, 0x2300, 0x00, 2, 0x001E, "control source CANopen")
            self.sdo_download(node, 0x6060, 0x00, 1, 0x01, "profile position mode")
            self.sdo_download(node, 0x6081, 0x00, 4, profile_velocity, "profile velocity")
            self.sdo_download(node, 0x6083, 0x00, 4, profile_accel, "profile acceleration")
            self.sdo_download(node, 0x6084, 0x00, 4, profile_accel, "profile deceleration")
            self.sdo_download(node, 0x6040, 0x00, 2, 0x000F, "enable operation")
            mode = self.sdo_upload(node, 0x6061, 0x00, 1, "mode display")
            if mode.get("signed_value") != 1:
                raise RuntimeError(f"Node{node} mode display is {mode}, expected 1")

    def wait_until_stopped_near(self, nodes: list[int], targets: dict[int, int],
                                tolerance_counts: int, timeout_ms: int) -> bool:
        deadline = time.monotonic() + timeout_ms / 1000.0
        stable = 0
        while time.monotonic() < deadline:
            all_ok = True
            for node in nodes:
                pos = self.sdo_upload(node, 0x6064, 0x00, 4, "wait actual position")
                vel = self.sdo_upload(node, 0x606C, 0x00, 4, "wait actual velocity")
                error = int(pos.get("signed_value", 0)) - targets[node]
                if abs(error) > tolerance_counts or abs(int(vel.get("signed_value", 0))) > 50_000:
                    all_ok = False
            if all_ok:
                stable += 1
                if stable >= 3:
                    return True
            else:
                stable = 0
            time.sleep(0.05)
        return False

    def command_group_target(self, nodes: list[int], targets: dict[int, int],
                             rpdo_map: str, rpdo_transmission: str) -> None:
        for node in nodes:
            self.send_rpdo1(node, 0x002F, targets[node], rpdo_map)
        if rpdo_transmission == RPDO_TX_SYNC1:
            self.sync()
        for node in nodes:
            self.send_rpdo1(node, 0x003F, targets[node], rpdo_map)
        if rpdo_transmission == RPDO_TX_SYNC1:
            self.sync()

    @staticmethod
    def waveform_target(sample: int, samples: int, amplitude: int, waveform: str) -> int:
        """Return the joystick-like absolute position target for one cycle.

        `sine` is useful for smooth dynamic tests, but real remote steering is
        closer to a human moving a stick, holding it, then moving it back.  The
        `joystick` waveform intentionally includes holds and ramps so the log
        shows whether the axis follows continuously changing targets and whether
        it settles when the command is held.
        """

        if samples <= 0:
            return 0
        phase = min(1.0, max(0.0, sample / samples))

        if waveform == WAVEFORM_SINE:
            return int(round(amplitude * math.sin(2.0 * math.pi * phase)))

        if waveform == WAVEFORM_TRIANGLE:
            if phase < 0.25:
                value = phase / 0.25
            elif phase < 0.75:
                value = 1.0 - 2.0 * ((phase - 0.25) / 0.5)
            else:
                value = -1.0 + ((phase - 0.75) / 0.25)
            return int(round(amplitude * value))

        if waveform == WAVEFORM_STEP:
            if phase < 0.2:
                return 0
            if phase < 0.45:
                return amplitude
            if phase < 0.7:
                return -amplitude
            return 0

        if waveform == WAVEFORM_JOYSTICK:
            # 0-10%: center hold
            # 10-35%: operator pushes stick to positive steering
            # 35-45%: positive hold
            # 45-75%: operator sweeps through center to negative steering
            # 75-85%: negative hold
            # 85-100%: return to center
            if phase < 0.10:
                value = 0.0
            elif phase < 0.35:
                value = (phase - 0.10) / 0.25
            elif phase < 0.45:
                value = 1.0
            elif phase < 0.75:
                value = 1.0 - 2.0 * ((phase - 0.45) / 0.30)
            elif phase < 0.85:
                value = -1.0
            else:
                value = -1.0 + ((phase - 0.85) / 0.15)
            return int(round(amplitude * value))

        if waveform == WAVEFORM_REMOTE_STRESS:
            # Deterministic "operator is still steering while the wheel is
            # moving" profile.  The command changes every 6-12 control samples
            # at 20 ms (120-240 ms), far faster than a full +/-500000 count
            # steering move can settle.  The motor must therefore discard old
            # targets naturally and chase only the latest profile-position
            # target.
            breakpoints = (
                (0, 0.00),
                (8, 0.35),
                (16, -0.25),
                (26, 0.70),
                (34, -0.80),
                (46, 0.15),
                (54, 1.00),
                (64, -1.00),
                (76, -0.10),
                (86, 0.55),
                (96, 0.00),
            )
            cycle = breakpoints[-1][0]
            local = sample % cycle
            for (x0, y0), (x1, y1) in zip(breakpoints, breakpoints[1:]):
                if x0 <= local <= x1:
                    ratio = 0.0 if x1 == x0 else (local - x0) / (x1 - x0)
                    value = y0 + (y1 - y0) * ratio
                    return int(round(amplitude * value))
            return 0

        if waveform == WAVEFORM_CENTER_NOISE:
            # Deterministic small-signal remote noise around center.  This is
            # intentionally not random so test logs are reproducible.  It helps
            # choose deadband and update-threshold values for the real SBUS
            # joystick path.
            value = (
                0.06 * math.sin(2.0 * math.pi * sample / 11.0)
                + 0.03 * math.sin(2.0 * math.pi * sample / 5.0)
                + 0.015 * math.sin(2.0 * math.pi * sample / 3.0)
            )
            return int(round(amplitude * value))

        raise ValueError(f"unsupported waveform: {waveform}")

    @staticmethod
    def apply_step_limit(previous: int, requested: int, max_step: int) -> int:
        if max_step <= 0:
            return requested
        delta = requested - previous
        if delta > max_step:
            return previous + max_step
        if delta < -max_step:
            return previous - max_step
        return requested

    def run_follow(self, nodes: list[int], amplitude: int, period_ms: int, samples: int,
                   profile_velocity: int, profile_accel: int, live_feedback: bool,
                   rpdo_map: str, rpdo_transmission: str, waveform: str,
                   max_step: int, fault_reset: bool) -> dict[str, object]:
        self.setup_position_mode(nodes, profile_velocity, profile_accel, rpdo_map, rpdo_transmission, fault_reset)
        before = self.read_node_state(nodes, "before.json")

        zero_targets = {node: 0 for node in nodes}
        self.command_group_target(nodes, zero_targets, rpdo_map, rpdo_transmission)
        self.wait_until_stopped_near(nodes, zero_targets, tolerance_counts=5_000, timeout_ms=8000)
        self.flush_feedback_buffers(nodes, 100)

        feedback: list[dict[str, object]] = []
        target_trace: list[dict[str, object]] = []
        timing_trace: list[dict[str, object]] = []
        next_tick = time.monotonic()
        previous_target = 0
        for i in range(samples + 1):
            requested_target = self.waveform_target(i, samples, amplitude, waveform)
            target = self.apply_step_limit(previous_target, requested_target, max_step)
            previous_target = target
            targets = {node: target for node in nodes}
            target_trace.append({
                "sample": i,
                "requested_target": requested_target,
                "target": target,
                "time_s": time.time(),
            })
            sent_at = time.monotonic()
            self.command_group_target(nodes, targets, rpdo_map, rpdo_transmission)
            if live_feedback:
                feedback.extend(self.drain_feedback_nonblocking(nodes))
            next_tick += period_ms / 1000.0
            remaining_s = next_tick - time.monotonic()
            timing_trace.append({
                "sample": i,
                "send_duration_ms": round((time.monotonic() - sent_at) * 1000.0, 3),
                "sleep_ms": round(max(0.0, remaining_s) * 1000.0, 3),
                "late_ms": round(max(0.0, -remaining_s) * 1000.0, 3),
            })
            if remaining_s > 0:
                time.sleep(remaining_s)

        # End at zero and verify the axes stop there.
        for _ in range(3):
            self.command_group_target(nodes, zero_targets, rpdo_map, rpdo_transmission)
            time.sleep(period_ms / 1000.0)
        reached_zero = self.wait_until_stopped_near(nodes, zero_targets, tolerance_counts=8_000, timeout_ms=8000)
        if not live_feedback:
            feedback.extend(self.collect_feedback(nodes, 1000))
        after = self.read_node_state(nodes, "after.json")
        self.write_json("feedback.json", feedback)
        self.write_json("target_trace.json", target_trace)
        self.write_json("timing_trace.json", timing_trace)
        summary = {
            "nodes": nodes,
            "amplitude": amplitude,
            "period_ms": period_ms,
            "samples": samples,
            "profile_velocity": profile_velocity,
            "profile_accel": profile_accel,
            "rpdo_map": rpdo_map,
            "rpdo_transmission": rpdo_transmission,
            "waveform": waveform,
            "reached_zero": reached_zero,
            "before": before,
            "after": after,
            "feedback_count": len(feedback),
            "emcy_count": sum(1 for item in feedback if isinstance(item.get("decoded"), dict) and "emcy" in item["decoded"]),
            "max_late_ms": max((float(item["late_ms"]) for item in timing_trace), default=0.0),
            "max_send_duration_ms": max((float(item["send_duration_ms"]) for item in timing_trace), default=0.0),
            "target_min": min((int(item["target"]) for item in target_trace), default=0),
            "target_max": max((int(item["target"]) for item in target_trace), default=0),
            "requested_target_min": min((int(item["requested_target"]) for item in target_trace), default=0),
            "requested_target_max": max((int(item["requested_target"]) for item in target_trace), default=0),
            "max_step": max_step,
            "fault_reset_before_test": fault_reset,
        }
        self.write_json("summary.json", summary)
        return summary


def parse_nodes(value: str) -> list[int]:
    if value == "steer4":
        return list(STEER_NODES)
    nodes = [int(item) for item in value.split(",") if item.strip()]
    for node in nodes:
        if node not in STEER_NODES:
            raise ValueError(f"only steering nodes 5-8 are allowed, got {node}")
    return nodes


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="RPDO1 steering realtime position-follow debug.")
    parser.add_argument("--nodes", default="5", help="'5' or 'steer4' or comma list within 5,6,7,8")
    parser.add_argument("--amplitude", type=int, default=500_000)
    parser.add_argument("--period-ms", type=int, default=20)
    parser.add_argument("--samples", type=int, default=160)
    parser.add_argument("--profile-velocity", type=int, default=1_666_666)
    parser.add_argument("--profile-accel", type=int, default=20_000_000)
    parser.add_argument("--timeout-ms", type=int, default=900)
    parser.add_argument("--live-feedback", action="store_true")
    parser.add_argument("--rpdo-map", choices=[RPDO_MAP_CURRENT7, RPDO_MAP_COMPACT6], default=RPDO_MAP_CURRENT7)
    parser.add_argument(
        "--rpdo-transmission",
        choices=[RPDO_TX_SYNC1, RPDO_TX_ASYNC255],
        default=RPDO_TX_SYNC1,
    )
    parser.add_argument(
        "--waveform",
        choices=[
            WAVEFORM_SINE,
            WAVEFORM_JOYSTICK,
            WAVEFORM_TRIANGLE,
            WAVEFORM_STEP,
            WAVEFORM_REMOTE_STRESS,
            WAVEFORM_CENTER_NOISE,
        ],
        default=WAVEFORM_JOYSTICK,
    )
    parser.add_argument(
        "--max-step",
        type=int,
        default=0,
        help="optional per-20ms target limiter in counts; 0 disables limiting",
    )
    parser.add_argument(
        "--fault-reset-before-test",
        action="store_true",
        help="apply CiA-402 disable/fault-reset/shutdown/switch-on before enabling each axis",
    )
    parser.add_argument("--allow-motion", action="store_true")
    parser.add_argument("--log-dir", default="")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.allow_motion:
        print("FAILED: --allow-motion is required", file=sys.stderr)
        return 1
    nodes = parse_nodes(args.nodes)
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    suffix = "steer4" if nodes == list(STEER_NODES) else "_".join(str(node) for node in nodes)
    log_dir = Path(args.log_dir) if args.log_dir else REPO_ROOT / "out" / f"steer_rpdo1_follow_{suffix}_{timestamp}"
    try:
        with SteerPositionDebug(args.timeout_ms, log_dir) as debug:
            summary = debug.run_follow(
                nodes,
                args.amplitude,
                args.period_ms,
                args.samples,
                args.profile_velocity,
                args.profile_accel,
                args.live_feedback,
                args.rpdo_map,
                args.rpdo_transmission,
                args.waveform,
                args.max_step,
                args.fault_reset_before_test,
            )
        print(json.dumps({"log_dir": str(log_dir), **summary}, indent=2, ensure_ascii=False))
        return 0
    except Exception as exc:  # noqa: BLE001 - CLI hardware tool reports failures plainly.
        print(f"FAILED: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

