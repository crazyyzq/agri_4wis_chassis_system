"""Passive CAN2 monitor for the USB CAN analyzer.

The vendor API uses zero-based channel indexes, so analyzer CAN2 is
``channel=1``. This script receives only; it does not transmit frames.
"""

from __future__ import annotations

import argparse
import time

from controlcan import ControlCAN


def format_frame(frame) -> str:
    data = " ".join(f"{frame.Data[i]:02X}" for i in range(frame.DataLen))
    frame_type = "ext" if frame.ExternFlag else "std"
    remote = " rtr" if frame.RemoteFlag else ""
    return f"id=0x{frame.ID:03X} {frame_type}{remote} dlc={frame.DataLen} data=[{data}]"


def main() -> int:
    parser = argparse.ArgumentParser(description="Passive CAN analyzer CAN2 monitor.")
    parser.add_argument("--channel", type=int, default=1, help="zero-based analyzer channel; CAN2 is 1")
    parser.add_argument("--bitrate", type=int, default=1000000, help="CAN bitrate in bit/s")
    parser.add_argument("--listen-only", action="store_true", help="request analyzer listen-only mode if supported")
    parser.add_argument(
        "--duration-sec",
        type=float,
        default=0.0,
        help="optional capture time limit; 0 keeps monitoring until Ctrl+C",
    )
    args = parser.parse_args()

    analyzer = ControlCAN()
    analyzer.open()
    try:
        analyzer.init_can(channel=args.channel, bitrate=args.bitrate, listen_only=args.listen_only)
        print(f"CAN analyzer channel={args.channel} bitrate={args.bitrate} passive monitor started")
        started_at = time.monotonic()
        while args.duration_sec <= 0.0 or (time.monotonic() - started_at) < args.duration_sec:
            for frame in analyzer.receive(args.channel, limit=100, wait_ms=100):
                print(format_frame(frame))
            time.sleep(0.01)
    finally:
        analyzer.close()


if __name__ == "__main__":
    raise SystemExit(main())
