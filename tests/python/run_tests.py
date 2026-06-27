"""Small no-dependency test runner for ECU framework checks.

The development machine currently has Python and a RISC-V target compiler but
no native C compiler. These tests therefore verify architecture, public API
contracts, and source-level safety rules before target syntax/build checks run.
"""

from __future__ import annotations

import importlib.util
import pathlib
import sys
import traceback


ROOT = pathlib.Path(__file__).resolve().parents[2]
TEST_DIR = pathlib.Path(__file__).resolve().parent


def load_module(path: pathlib.Path):
    spec = importlib.util.spec_from_file_location(path.stem, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main() -> int:
    failures: list[str] = []
    test_files = sorted(TEST_DIR.glob("test_*.py"))
    for test_file in test_files:
        module = load_module(test_file)
        for name in sorted(dir(module)):
            if not name.startswith("test_"):
                continue
            test_func = getattr(module, name)
            if not callable(test_func):
                continue
            try:
                test_func(ROOT)
                print(f"PASS {test_file.name}::{name}")
            except Exception as exc:  # noqa: BLE001 - test runner must report all failures
                failures.append(f"{test_file.name}::{name}: {exc}")
                print(f"FAIL {test_file.name}::{name}")
                traceback.print_exc()
    if failures:
        print("\nFAILED TESTS:")
        for failure in failures:
            print(f"- {failure}")
        return 1
    print(f"\nAll {sum(1 for p in test_files for n in dir(load_module(p)) if n.startswith('test_'))} tests passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
