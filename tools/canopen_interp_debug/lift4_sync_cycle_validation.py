"""Repeat the verified four-axis interpolation trajectory for endurance evidence.

This wrapper deliberately runs each 10->490 mm or 490->10 mm leg as a fresh
``lift4_sync_debug.py`` process.  Reopening the analyzer and rebuilding the
drive state between legs tests the same disable/enable and stale-buffer
recovery path that production commissioning depends on.  A failed leg stops
the sequence immediately; no automatic retry can turn a failure into a pass.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path


THIS_FILE = Path(__file__).resolve()
REPO_ROOT = THIS_FILE.parents[2]
SYNC_TOOL = THIS_FILE.with_name("lift4_sync_debug.py")
LIFT_NODES = ("9", "11", "12", "10")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="10..490 mm four-lift repeated stability validation"
    )
    parser.add_argument("--cycles", type=int, default=10)
    parser.add_argument("--speed-mm-s", type=float, default=20.0)
    parser.add_argument("--accel-mm-s2", type=float, default=8.0)
    parser.add_argument("--profile-velocity-units", type=int, default=53_000_000)
    parser.add_argument("--profile-acceleration", type=int, default=250_000)
    parser.add_argument("--max-following-lead-counts", type=int, default=655_360)
    parser.add_argument("--completion-timeout-ms", type=int, default=90_000)
    parser.add_argument("--log-dir", default="")
    parser.add_argument("--allow-motion", action="store_true")
    return parser.parse_args()


def current_summary(report: dict[str, object]) -> dict[str, object]:
    stats = report["motion_current_stats"]
    assert isinstance(stats, dict)
    return {
        node: {
            "average_absolute_current_a": stats[node][
                "average_absolute_current_a"
            ],
            "maximum_absolute_current_a": stats[node][
                "maximum_absolute_current_a"
            ],
        }
        for node in LIFT_NODES
    }


def main() -> int:
    args = parse_args()
    if not args.allow_motion:
        print(
            "DRY-RUN: add --allow-motion only after ECU CAN3 is disconnected "
            "and the complete vehicle is mechanically safe."
        )
        return 0
    if (
        args.cycles < 1
        or args.speed_mm_s <= 0.0
        or args.speed_mm_s > 25.0
        or args.accel_mm_s2 <= 0.0
        or args.completion_timeout_ms <= 0
    ):
        print("FAILED: invalid cycle/trajectory parameters", file=sys.stderr)
        return 2

    timestamp = time.strftime("%Y%m%d_%H%M%S")
    log_dir = (
        Path(args.log_dir)
        if args.log_dir
        else REPO_ROOT / "tmp" / f"lift4_endurance_{timestamp}"
    )
    log_dir.mkdir(parents=True, exist_ok=True)
    summary_path = log_dir / "summary.json"
    summary: dict[str, object] = {
        "cycles_requested": args.cycles,
        "cycles_completed": 0,
        "speed_mm_s": args.speed_mm_s,
        "acceleration_mm_s2": args.accel_mm_s2,
        "profile_velocity_units": args.profile_velocity_units,
        "profile_acceleration": args.profile_acceleration,
        "max_following_lead_counts": args.max_following_lead_counts,
        "legs": [],
    }

    for cycle in range(1, args.cycles + 1):
        for direction, target_mm in (("up", 490), ("down", 10)):
            leg_name = f"cycle_{cycle:02d}_{direction}"
            leg_log = log_dir / f"{leg_name}.json"
            command = [
                sys.executable,
                str(SYNC_TOOL),
                "--absolute-target-mm",
                str(target_mm),
                "--speed-mm-s",
                str(args.speed_mm_s),
                "--accel-mm-s2",
                str(args.accel_mm_s2),
                "--profile-velocity-units",
                str(args.profile_velocity_units),
                "--profile-acceleration",
                str(args.profile_acceleration),
                "--max-following-lead-counts",
                str(args.max_following_lead_counts),
                "--completion-timeout-ms",
                str(args.completion_timeout_ms),
                "--log",
                str(leg_log),
                "--allow-motion",
            ]
            print(
                f"[{leg_name}] target={target_mm}mm "
                f"speed={args.speed_mm_s:g}mm/s "
                f"accel={args.accel_mm_s2:g}mm/s^2",
                flush=True,
            )
            completed = subprocess.run(
                command,
                cwd=REPO_ROOT,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                check=False,
            )
            if completed.returncode != 0 or not leg_log.exists():
                failure = {
                    "cycle": cycle,
                    "direction": direction,
                    "target_mm": target_mm,
                    "returncode": completed.returncode,
                    "output": completed.stdout[-4000:],
                }
                summary["failed_leg"] = failure
                summary_path.write_text(
                    json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
                    encoding="utf-8",
                )
                print(completed.stdout, file=sys.stderr)
                print(
                    f"FAILED: {leg_name}; summary written to {summary_path}",
                    file=sys.stderr,
                )
                return 1

            report = json.loads(leg_log.read_text(encoding="utf-8"))
            leg_summary = {
                "cycle": cycle,
                "direction": direction,
                "target_mm": target_mm,
                "completed_final_target": report["completed_final_target"],
                "max_spread_mm": report["max_relative_spread_mm"],
                "emcy_count": len(report["emcy"]),
                "tpdo0_recovery_count": report["tpdo0_recovery_count"],
                "tick_resync_count": report["tick_resync_count"],
                "feedback_governor_count": report[
                    "feedback_governor_count"
                ],
                "starvation_recovery_count": report[
                    "starvation_recovery_count"
                ],
                "maximum_following_error_mm": report[
                    "maximum_following_error_mm"
                ],
                "current": current_summary(report),
                "log": str(leg_log),
            }
            summary["legs"].append(leg_summary)
            if direction == "down":
                summary["cycles_completed"] = cycle
            summary_path.write_text(
                json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            print(
                json.dumps(leg_summary, ensure_ascii=False),
                flush=True,
            )

    print(f"PASS: {args.cycles} cycles completed; {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
