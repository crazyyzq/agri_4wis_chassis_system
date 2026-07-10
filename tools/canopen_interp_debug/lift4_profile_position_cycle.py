"""Four-axis lift profile-position commissioning through the CAN analyzer.

The analyzer CAN2 channel owns ECU CAN3 while this tool is running.  RPDO1 is
the frozen seven-byte mapping: 0x6040 controlword, 0x6060 mode and 0x607A
absolute target position.  All four axes are armed first and then triggered by
one common SYNC so a long lift move is planned inside each drive instead of
streaming thousands of interpolation points from the PC.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

THIS_FILE = Path(__file__).resolve()
REPO_ROOT = THIS_FILE.parents[2]
SCRIPT_DIR = THIS_FILE.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from lift4_sync_debug import (  # noqa: E402
    CAN_CHANNEL,
    LIFT_COUNTS_PER_MM,
    LIFT_MAX_SYNC_SPREAD_MM,
    LIFT_MAX_POSITION_COUNTS,
    LIFT_MIN_POSITION_COUNTS,
    LIFT_MOTOR_REVS_PER_MM,
    Lift4SyncDebug,
    RPDO1_BASE,
    SYNC_COB_ID,
)


LIFT_NODES = (9, 11, 12, 10)
PROFILE_POSITION_MODE = 1
CONTROLWORD_DISABLE_VOLTAGE = 0x0000
CONTROLWORD_SHUTDOWN = 0x0006
CONTROLWORD_SWITCH_ON = 0x0007
CONTROLWORD_ENABLE_OPERATION = 0x000F
CONTROLWORD_TRIGGER_ABSOLUTE = 0x001F
PROFILE_VELOCITY_SCALE = 10.0
PROFILE_ACCELERATION_SCALE = 0.1


def rpdo1_payload(controlword: int, mode: int, target_counts: int) -> bytes:
    return (
        int(controlword).to_bytes(2, "little")
        + int(mode).to_bytes(1, "little", signed=True)
        + int(target_counts).to_bytes(4, "little", signed=True)
    )


class LiftProfilePositionCycle:
    def __init__(
        self,
        nodes: tuple[int, ...],
        timeout_ms: int,
        safe_min_mm: float,
        safe_max_mm: float,
    ) -> None:
        self.debug = Lift4SyncDebug(nodes, timeout_ms)
        self.nodes = nodes
        self.safe_min_mm = safe_min_mm
        self.safe_max_mm = safe_max_mm
        self.samples: list[dict[str, object]] = []
        self.max_absolute_spread_counts = 0

    def __enter__(self) -> "LiftProfilePositionCycle":
        self.debug.__enter__()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:  # type: ignore[no-untyped-def]
        if exc_type is not None:
            # Profile-position motion continues inside the drive after CAN
            # traffic stops.  A fault/timeout must therefore disable voltage;
            # the interpolation helper's normal 0x000F cleanup would leave an
            # active profile move enabled and is unsafe on this path.
            try:
                self.emergency_disable()
            finally:
                self.debug.can.close()
            return
        self.debug.__exit__(exc_type, exc, tb)

    def configure(
        self,
        speed_mm_s: float,
        acceleration_mm_s2: float,
        tracking_window_mm: float,
    ) -> None:
        velocity_units = math.ceil(
            speed_mm_s * LIFT_COUNTS_PER_MM * PROFILE_VELOCITY_SCALE
        )
        acceleration_units = math.ceil(
            acceleration_mm_s2 *
            LIFT_COUNTS_PER_MM *
            PROFILE_ACCELERATION_SCALE
        )
        tracking_window_counts = math.ceil(
            tracking_window_mm * LIFT_COUNTS_PER_MM
        )
        for node in self.nodes:
            self.debug.send(0x000, bytes([0x01, node]))
            time.sleep(0.01)
            _, latched_fault = self.debug.sdo_read(node, 0x2183, 0)
            if latched_fault != 0:
                # The vendor object dictionary defines 0x2183 as
                # read/clear: write a one to every latched bit that must be
                # cleared.  This is not an NMT reset and preserves the
                # absolute-position reference.
                self.debug.sdo_write(node, 0x2183, 0, 4, latched_fault)
                time.sleep(0.1)
            for controlword in (
                CONTROLWORD_DISABLE_VOLTAGE,
                0x0080,
                CONTROLWORD_DISABLE_VOLTAGE,
                CONTROLWORD_SHUTDOWN,
            ):
                self.debug.sdo_write(node, 0x6040, 0, 2, controlword)
                time.sleep(0.03)
            self.debug.sdo_write(node, 0x2300, 0, 2, 0x001E)
            self.debug.sdo_write(node, 0x6060, 0, 1, PROFILE_POSITION_MODE)
            self.debug.sdo_write(node, 0x6081, 0, 4, velocity_units)
            self.debug.sdo_write(node, 0x6083, 0, 4, acceleration_units)
            self.debug.sdo_write(node, 0x6084, 0, 4, acceleration_units)
            self.debug.sdo_write(
                node, 0x2120, 0, 4, tracking_window_counts
            )
            self.debug.sdo_write(node, 0x6040, 0, 2, CONTROLWORD_SWITCH_ON)
            self.debug.sdo_write(
                node, 0x6040, 0, 2, CONTROLWORD_ENABLE_OPERATION
            )
            _, mode_display = self.debug.sdo_read(node, 0x6061, 0)
            fault, _ = self.debug.sdo_read(node, 0x2183, 0)
            window_readback, _ = self.debug.sdo_read(node, 0x2120, 0)
            if (
                mode_display != PROFILE_POSITION_MODE
                or fault != 0
                or window_readback != tracking_window_counts
            ):
                raise RuntimeError(
                    f"Node{node} setup rejected mode={mode_display} "
                    f"fault=0x{fault:08X} "
                    f"tracking_window={window_readback}"
                )
        time.sleep(1.0)

    def prime_feedback(self) -> dict[int, int]:
        for _ in range(5):
            self.debug.send(SYNC_COB_ID, b"")
            self.debug.drain_feedback(5)
            time.sleep(0.02)
        positions = self.debug.read_positions()
        return positions

    def send_group(self, controlword: int, target_counts: int) -> None:
        for node in self.nodes:
            self.debug.send(
                RPDO1_BASE + node,
                rpdo1_payload(
                    controlword,
                    PROFILE_POSITION_MODE,
                    target_counts,
                ),
            )
        self.debug.send(SYNC_COB_ID, b"")

    def emergency_disable(self) -> None:
        for node in self.nodes:
            try:
                self.debug.sdo_write(
                    node, 0x6040, 0, 2, CONTROLWORD_DISABLE_VOLTAGE
                )
            except Exception:
                pass

    def brake_rest(self, duration_s: float) -> None:
        """Apply all holding brakes between high-current lift segments."""
        self.emergency_disable()
        time.sleep(duration_s)
        for node in self.nodes:
            fault, _ = self.debug.sdo_read(node, 0x2183, 0)
            if fault != 0:
                raise RuntimeError(
                    f"Node{node} fault during brake rest: 0x{fault:08X}"
                )
            for controlword in (
                CONTROLWORD_SHUTDOWN,
                CONTROLWORD_SWITCH_ON,
                CONTROLWORD_ENABLE_OPERATION,
            ):
                self.debug.sdo_write(
                    node, 0x6040, 0, 2, controlword
                )
                time.sleep(0.02)
        time.sleep(0.3)

    def move_segmented_to(
        self,
        target_counts: int,
        segment_counts: int,
        speed_mm_s: float,
        rest_s: float,
    ) -> list[dict[str, object]]:
        positions = self.prime_feedback()
        spread = max(positions.values()) - min(positions.values())
        if spread > round(LIFT_MAX_SYNC_SPREAD_MM * LIFT_COUNTS_PER_MM):
            raise RuntimeError(
                "absolute synchronization error before segmented move "
                f"{spread / LIFT_COUNTS_PER_MM:.3f}mm"
            )
        center = round(sum(positions.values()) / len(positions))
        results: list[dict[str, object]] = []
        while center != target_counts:
            delta = target_counts - center
            step = min(abs(delta), segment_counts)
            next_target = center + (step if delta > 0 else -step)
            results.append(self.move_to(next_target, speed_mm_s))
            center = next_target
            if center != target_counts:
                self.brake_rest(rest_s)
        return results

    def move_to(self, target_counts: int, speed_mm_s: float) -> dict[str, object]:
        if not LIFT_MIN_POSITION_COUNTS <= target_counts <= LIFT_MAX_POSITION_COUNTS:
            raise ValueError(f"target outside lift limits: {target_counts}")
        target_mm = -target_counts / LIFT_COUNTS_PER_MM
        if not self.safe_min_mm <= target_mm <= self.safe_max_mm:
            raise ValueError(
                f"target {target_mm:.3f}mm outside active safety range "
                f"{self.safe_min_mm:.3f}..{self.safe_max_mm:.3f}mm"
            )

        starts = self.prime_feedback()
        self.debug.emcy.clear()
        self.debug.current_limit_emcy_count = 0
        for node in self.nodes:
            self.debug.current_limit_emcy_by_node[node] = 0
            self.debug.feedback[node].peak_abs_current = 0

        self.send_group(CONTROLWORD_ENABLE_OPERATION, target_counts)
        self.send_group(CONTROLWORD_TRIGGER_ABSOLUTE, target_counts)

        max_distance_counts = max(
            abs(target_counts - position) for position in starts.values()
        )
        expected_s = (
            max_distance_counts / LIFT_COUNTS_PER_MM / speed_mm_s
        )
        deadline = time.monotonic() + expected_s * 2.0 + 20.0
        stable_samples = 0
        tick = 0
        max_relative_travel_spread = 0
        final_positions = starts.copy()

        while time.monotonic() < deadline:
            tick_start = time.perf_counter()
            self.debug.send(SYNC_COB_ID, b"")
            self.debug.drain_feedback(5)

            if self.debug.emcy:
                raise RuntimeError(f"EMCY received: {self.debug.emcy[-1]}")
            for node in self.nodes:
                feedback = self.debug.feedback[node]
                if feedback.fault != 0:
                    raise RuntimeError(
                        f"Node{node} fault=0x{feedback.fault:08X}"
                    )

            final_positions = {
                node: self.debug.feedback[node].position for node in self.nodes
            }
            absolute_spread = max(final_positions.values()) - min(
                final_positions.values()
            )
            relative_travel = [
                final_positions[node] - starts[node] for node in self.nodes
            ]
            relative_spread = max(relative_travel) - min(relative_travel)
            self.max_absolute_spread_counts = max(
                self.max_absolute_spread_counts, absolute_spread
            )
            max_relative_travel_spread = max(
                max_relative_travel_spread, relative_spread
            )
            if relative_spread > round(3.0 * LIFT_COUNTS_PER_MM):
                raise RuntimeError(
                    f"relative synchronization error "
                    f"{relative_spread / LIFT_COUNTS_PER_MM:.3f}mm"
                )

            max_error = max(
                abs(target_counts - position)
                for position in final_positions.values()
            )
            if max_error <= round(0.25 * LIFT_COUNTS_PER_MM):
                stable_samples += 1
                if stable_samples >= 10:
                    break
            else:
                stable_samples = 0

            if tick % 10 == 0:
                self.samples.append(
                    {
                        "time_ms": tick * 20,
                        "target_counts": target_counts,
                        "positions": final_positions,
                        "absolute_spread_counts": absolute_spread,
                        "relative_spread_counts": relative_spread,
                        "currents": {
                            str(node): self.debug.feedback[node].current
                            for node in self.nodes
                        },
                    }
                )
            tick += 1
            remaining = 0.02 - (time.perf_counter() - tick_start)
            if remaining > 0:
                time.sleep(remaining)
        else:
            raise TimeoutError(f"move timeout target={target_counts}")

        self.send_group(CONTROLWORD_ENABLE_OPERATION, target_counts)
        return {
            "target_counts": target_counts,
            "target_mm": target_mm,
            "start_positions": starts,
            "final_positions": final_positions,
            "duration_s": round(tick * 0.02, 3),
            "max_relative_spread_counts": max_relative_travel_spread,
            "max_relative_spread_mm": round(
                max_relative_travel_spread / LIFT_COUNTS_PER_MM, 3
            ),
            "current_limit_emcy_count": self.debug.current_limit_emcy_count,
            "current_limit_emcy_by_node": self.debug.current_limit_emcy_by_node,
            "peak_abs_current": {
                str(node): self.debug.feedback[node].peak_abs_current
                for node in self.nodes
            },
        }

    def move_to_common_leveling_target(
        self,
        target_counts: int,
        speed_mm_s: float,
        max_initial_spread_mm: float,
    ) -> dict[str, object]:
        """Move all selected axes to one absolute target despite initial skew.

        This mode is only for field recovery from a known height mismatch.  It
        still sends a complete four-axis group.  It may retract the higher
        axes toward the lowest measured leg only after explicit operator
        approval; this is how a skewed machine is recovered before a normal
        upward synchronized move.
        """
        target_mm = -target_counts / LIFT_COUNTS_PER_MM
        if not self.safe_min_mm <= target_mm <= self.safe_max_mm:
            raise ValueError(
                f"target {target_mm:.3f}mm outside active safety range "
                f"{self.safe_min_mm:.3f}..{self.safe_max_mm:.3f}mm"
            )
        starts = self.prime_feedback()
        initial_spread = max(starts.values()) - min(starts.values())
        if initial_spread > round(max_initial_spread_mm * LIFT_COUNTS_PER_MM):
            raise RuntimeError(
                "initial absolute spread too large for leveling move "
                f"{initial_spread / LIFT_COUNTS_PER_MM:.3f}mm"
            )
        self.debug.emcy.clear()
        self.debug.current_limit_emcy_count = 0
        for node in self.nodes:
            self.debug.current_limit_emcy_by_node[node] = 0
            self.debug.feedback[node].peak_abs_current = 0

        self.send_group(CONTROLWORD_ENABLE_OPERATION, target_counts)
        self.send_group(CONTROLWORD_TRIGGER_ABSOLUTE, target_counts)

        max_distance_counts = max(
            abs(target_counts - position) for position in starts.values()
        )
        expected_s = max_distance_counts / LIFT_COUNTS_PER_MM / speed_mm_s
        deadline = time.monotonic() + expected_s * 2.0 + 30.0
        stable_samples = 0
        tick = 0
        max_absolute_spread = initial_spread
        max_current = {str(node): 0 for node in self.nodes}
        final_positions = starts.copy()
        while time.monotonic() < deadline:
            tick_start = time.perf_counter()
            self.debug.send(SYNC_COB_ID, b"")
            self.debug.drain_feedback(5)
            if self.debug.emcy:
                raise RuntimeError(f"EMCY received: {self.debug.emcy[-1]}")
            final_positions = {
                node: self.debug.feedback[node].position for node in self.nodes
            }
            absolute_spread = max(final_positions.values()) - min(
                final_positions.values()
            )
            max_absolute_spread = max(max_absolute_spread, absolute_spread)
            for node in self.nodes:
                feedback = self.debug.feedback[node]
                if feedback.fault != 0:
                    raise RuntimeError(
                        f"Node{node} fault=0x{feedback.fault:08X}"
                    )
                max_current[str(node)] = max(
                    max_current[str(node)],
                    abs(feedback.current),
                )
            if tick % 50 == 0:
                current_mm = {
                    str(node): round(
                        -final_positions[node] / LIFT_COUNTS_PER_MM, 3
                    )
                    for node in self.nodes
                }
                print(
                    f"t={tick * 0.02:.1f}s target={target_mm:.1f}mm "
                    f"pos={current_mm} "
                    f"spread={absolute_spread / LIFT_COUNTS_PER_MM:.3f}mm "
                    f"current={max_current}",
                    flush=True,
                )
            max_error = max(
                abs(target_counts - position)
                for position in final_positions.values()
            )
            if max_error <= round(0.25 * LIFT_COUNTS_PER_MM):
                stable_samples += 1
                if stable_samples >= 10:
                    break
            else:
                stable_samples = 0
            tick += 1
            remaining = 0.02 - (time.perf_counter() - tick_start)
            if remaining > 0:
                time.sleep(remaining)
        else:
            raise TimeoutError(f"move timeout target={target_counts}")

        self.send_group(CONTROLWORD_ENABLE_OPERATION, target_counts)
        return {
            "target_counts": target_counts,
            "target_mm": target_mm,
            "start_positions": starts,
            "final_positions": final_positions,
            "duration_s": round(tick * 0.02, 3),
            "initial_absolute_spread_mm": round(
                initial_spread / LIFT_COUNTS_PER_MM,
                3,
            ),
            "max_absolute_spread_mm": round(
                max_absolute_spread / LIFT_COUNTS_PER_MM,
                3,
            ),
            "current_limit_emcy_count": self.debug.current_limit_emcy_count,
            "current_limit_emcy_by_node": self.debug.current_limit_emcy_by_node,
            "peak_abs_current": {
                str(node): self.debug.feedback[node].peak_abs_current
                for node in self.nodes
            },
        }

    def move_to_common_leveling_target_segmented(
        self,
        target_counts: int,
        segment_counts: int,
        speed_mm_s: float,
        rest_s: float,
        max_initial_spread_mm: float,
    ) -> list[dict[str, object]]:
        """Perform a skew-recovery absolute target as cooled small segments."""
        results: list[dict[str, object]] = []
        while True:
            positions = self.prime_feedback()
            highest_mm = -min(positions.values()) / LIFT_COUNTS_PER_MM
            current_target_counts = min(positions.values()) + segment_counts
            if current_target_counts > target_counts:
                current_target_counts = target_counts
            results.append(
                self.move_to_common_leveling_target(
                    current_target_counts,
                    speed_mm_s,
                    max_initial_spread_mm,
                )
            )
            if current_target_counts == target_counts:
                break
            print(
                f"segment rest {rest_s:.1f}s after highest={highest_mm:.3f}mm",
                flush=True,
            )
            self.brake_rest(rest_s)
        return results


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--nodes",
        default="9,11,12,10",
        help="comma-separated lift node IDs; defaults to the complete group",
    )
    parser.add_argument("--speed-mm-s", type=float, default=1.0)
    parser.add_argument("--accel-mm-s2", type=float, default=0.5)
    parser.add_argument("--tracking-window-mm", type=float, default=1.0)
    parser.add_argument("--short-mm", type=float, default=0.0)
    parser.add_argument("--long-mm", type=float, default=490.0)
    parser.add_argument("--safe-min-mm", type=float, default=0.0)
    parser.add_argument("--safe-max-mm", type=float, default=490.0)
    parser.add_argument("--cycles", type=int, default=2)
    parser.add_argument(
        "--segment-mm",
        type=float,
        default=0.0,
        help="maximum four-axis move per segment; zero disables segmentation",
    )
    parser.add_argument("--segment-rest-s", type=float, default=3.0)
    parser.add_argument("--leveling-target-mm", type=float, default=-1.0)
    parser.add_argument("--max-initial-spread-mm", type=float, default=12.0)
    parser.add_argument("--timeout-ms", type=int, default=1000)
    parser.add_argument("--log", default="")
    parser.add_argument("--allow-motion", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.allow_motion:
        print("FAILED: --allow-motion is required", file=sys.stderr)
        return 2
    if (
        args.speed_mm_s <= 0
        or args.accel_mm_s2 <= 0
        or args.tracking_window_mm <= 0
        or args.tracking_window_mm > 3.0
        or args.segment_mm < 0
        or args.segment_rest_s < 0
        or args.max_initial_spread_mm <= 0
        or args.safe_min_mm < 0
        or args.safe_max_mm <= args.safe_min_mm
        or args.cycles < 0
    ):
        print("FAILED: speed/acceleration must be positive", file=sys.stderr)
        return 2
    nodes = tuple(int(item) for item in args.nodes.split(",") if item)
    if not nodes or any(node not in LIFT_NODES for node in nodes):
        print(f"FAILED: nodes must be a subset of {LIFT_NODES}", file=sys.stderr)
        return 2
    short_target = -round(args.short_mm * LIFT_COUNTS_PER_MM)
    long_target = -round(args.long_mm * LIFT_COUNTS_PER_MM)
    log_path = (
        Path(args.log)
        if args.log
        else REPO_ROOT / "tmp" /
             f"lift_profile_cycle_{time.strftime('%Y%m%d_%H%M%S')}.json"
    )
    report: dict[str, object] = {
        "speed_mm_s": args.speed_mm_s,
        "accel_mm_s2": args.accel_mm_s2,
        "tracking_window_mm": args.tracking_window_mm,
        "short_mm": args.short_mm,
        "long_mm": args.long_mm,
        "safe_min_mm": args.safe_min_mm,
        "safe_max_mm": args.safe_max_mm,
        "cycles": args.cycles,
        "segment_mm": args.segment_mm,
        "segment_rest_s": args.segment_rest_s,
        "leveling_target_mm": args.leveling_target_mm,
        "max_initial_spread_mm": args.max_initial_spread_mm,
        "nodes": nodes,
        "moves": [],
    }
    try:
        with LiftProfilePositionCycle(
            nodes,
            args.timeout_ms,
            args.safe_min_mm,
            args.safe_max_mm,
        ) as test:
            test.configure(
                args.speed_mm_s,
                args.accel_mm_s2,
                args.tracking_window_mm,
            )
            segment_counts = round(args.segment_mm * LIFT_COUNTS_PER_MM)

            def append_move(target: int) -> None:
                if segment_counts > 0:
                    report["moves"].extend(
                        test.move_segmented_to(
                            target,
                            segment_counts,
                            args.speed_mm_s,
                            args.segment_rest_s,
                        )
                    )
                else:
                    report["moves"].append(
                        test.move_to(target, args.speed_mm_s)
                    )

            if args.leveling_target_mm >= 0.0:
                target = -round(args.leveling_target_mm * LIFT_COUNTS_PER_MM)
                if segment_counts > 0:
                    report["moves"].extend(
                        test.move_to_common_leveling_target_segmented(
                            target,
                            segment_counts,
                            args.speed_mm_s,
                            args.segment_rest_s,
                            args.max_initial_spread_mm,
                        )
                    )
                else:
                    report["moves"].append(
                        test.move_to_common_leveling_target(
                            target,
                            args.speed_mm_s,
                            args.max_initial_spread_mm,
                        )
                    )
            else:
                append_move(short_target)
                for _ in range(args.cycles):
                    append_move(long_target)
                    append_move(short_target)
            report["samples"] = test.samples
            report["max_absolute_spread_counts"] = (
                test.max_absolute_spread_counts
            )
            report["max_absolute_spread_mm"] = round(
                test.max_absolute_spread_counts / LIFT_COUNTS_PER_MM,
                3,
            )
        log_path.parent.mkdir(parents=True, exist_ok=True)
        log_path.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        print(json.dumps({k: v for k, v in report.items() if k != "samples"},
                         ensure_ascii=False, indent=2))
        print(f"Log written to {log_path}")
        return 0
    except Exception as exc:  # noqa: BLE001
        print(f"FAILED: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
