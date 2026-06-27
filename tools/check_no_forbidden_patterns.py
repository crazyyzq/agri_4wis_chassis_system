"""Static guard for ECU architecture rules."""

from __future__ import annotations

import pathlib
import sys


ROOT = pathlib.Path(__file__).resolve().parents[1]
EXCLUDED_PATH_PARTS = {
    ".git",
    "build",
    "cmake-build-debug",
    "cmake-build-release",
    "Output",
    "sdk_env_v1.11.0",
}


def iter_sources(rel: str):
    base = ROOT / rel
    if not base.exists():
        return
    for path in base.rglob("*"):
        if any(part in EXCLUDED_PATH_PARTS for part in path.relative_to(ROOT).parts):
            continue
        if path.suffix.lower() in {".c", ".h"}:
            yield path


def fail(errors: list[str], path: pathlib.Path, message: str) -> None:
    errors.append(f"{path.relative_to(ROOT)}: {message}")


def main() -> int:
    errors: list[str] = []

    for path in iter_sources("ecu/remote"):
        text = path.read_text(encoding="utf-8")
        for token in ["board.h", "hpm_can", "hpm_gpio", "vehicle_command_executor"]:
            if token in text:
                fail(errors, path, f"remote layer must not reference {token}")

    for path in iter_sources("ecu/control"):
        text = path.read_text(encoding="utf-8")
        for token in ["remote_link_fsm", "remote_arm_fsm", "remote_estop_fsm"]:
            if token in text:
                fail(errors, path, f"control layer must not depend on remote private FSM {token}")

    for path in iter_sources("ecu"):
        if path.name in {"common.c", "misc.c", "utils.c"}:
            fail(errors, path, "garbage-bucket source file name is forbidden")
        text = path.read_text(encoding="utf-8")
        if "vTaskDelay(" in text and "ecu/apps" not in path.as_posix() and "ecu/os" not in path.as_posix():
            fail(errors, path, "business state machines must use explicit non-blocking waits")
        if "ecu/config" not in path.as_posix():
            for raw in ["1050", "1500", "1900", "1950"]:
                if raw in text:
                    fail(errors, path, f"raw SBUS threshold {raw} belongs in ecu/config only")

    if errors:
        print("Forbidden ECU framework patterns found:")
        for error in errors:
            print(f"- {error}")
        return 1
    print("No forbidden ECU framework patterns found.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
