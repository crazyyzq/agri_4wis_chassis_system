# CAN2 Motion Stability Implementation Plan

> **For agentic workers:** Execute inline with checkpoints. Do not use subagents
> for this shared hardware worktree.

**Goal:** Remove false remote disarming and CAN2 enable/disable recovery loops
while preserving fail-safe motion behavior.

**Architecture:** Keep CPU0 task ownership unchanged. Harden the existing
mailboxes and make the CAN2-owned recovery path publish an explicit zero-speed
enable intent only when the latest coherent safety command permits it.

**Tech Stack:** C, FreeRTOS, CANopenNode, HPM SDK 1.11, Python source-contract
tests, CMake/Ninja, SEGGER/J-Link.

## Global constraints

- No CANopen node reset and no steering zero rewrite.
- No nonzero command replay after recovery.
- Emergency stop, safety source, high-voltage loss and explicit disable still
  produce zero and disabled output.
- No PDO mapping change.

### Task 1: Reproduce both regressions in source-contract tests

**Files:**
- Modify: `tests/python/test_hardware_framework.py`

- Add a test requiring a bounded mailbox retry loop.
- Add a test requiring recovery-safe zero enable and fail-safe disable paths.
- Run `python tests\python\run_tests.py` and confirm the new assertions fail.

### Task 2: Harden remote/safety snapshot reads

**Files:**
- Modify: `ecu/os/src/ecu_control_snapshot_mailbox.c`
- Modify: `ecu/os/include/ecu_control_snapshot_mailbox.h` only if a public
  constant is required.

- Implement four nonblocking attempts inside the existing generated readers.
- Return false only for no publication, invalid arguments, or four conflicting
  publications.
- Run the test runner and confirm the mailbox regression test passes.

### Task 3: Break the CAN2 recovery feedback loop

**Files:**
- Modify: `ecu/devices/src/motion_device.c`
- Modify: `ecu/devices/include/motion_device.h` only if diagnostic state is
  required.

- Add one focused helper that decides whether zero-speed Operation Enabled is
  permitted from the latest coherent command.
- Replace recovery cached intent with zero velocity/current and the helper's
  enable value.
- Keep ordinary safety-stop behavior disabled.
- Run all source-contract tests.

### Task 4: Build and field-verify

- Run `git diff --check`.
- Configure/build CPU0 with `ECU_COMMISSIONING_PROFILE=whole_vehicle_motion`.
- Download the verified ELF with J-Link.
- Verify P-gear steering first, then low-speed traction, using COM diagnostics
  and analyzer CAN2 frames.
- Capture CAN3 setup and RPDO2 traffic during a short lift movement before any
  lift-control change.
