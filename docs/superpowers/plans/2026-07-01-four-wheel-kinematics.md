# Four-Wheel Kinematics Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the simplified four-wheel steering target generation with a configurable kinematic model that computes per-wheel steering angles and wheel speeds.

**Architecture:** Add a focused `four_wheel_kinematics` control module. `motion_control` remains the high-level mode selector and delegates Ackermann, reverse-Ackermann, spin and crab target generation to the kinematic module. Vehicle dimensions are configuration values, and the control API accepts runtime wheelbase and track-width values so later hydraulic/ADC feedback can update them without changing the formulas.

**Tech Stack:** C99 ECU control code, HPM SDK CMake/SEGGER project generation, Python static contract tests, RISC-V GCC/Ninja build.

---

### Task 1: Add regression tests for kinematic behavior

**Files:**
- Modify: `tests/python/test_vehicle_arbiter.py`

- [ ] Add checks that `four_wheel_kinematics.h/.c` exist and are included in the CPU0 build.
- [ ] Add checks that geometry macros live in `ecu_config.h`.
- [ ] Add checks that Ackermann calculation uses wheelbase, track width, turning radius, `atan2f`, `sqrtf`, and clamps geometry.
- [ ] Run `python tests\python\run_tests.py` and confirm the new test fails before implementation.

### Task 2: Implement configurable geometry and kinematic module

**Files:**
- Modify: `ecu/config/include/ecu_config.h`
- Create: `ecu/control/include/four_wheel_kinematics.h`
- Create: `ecu/control/src/four_wheel_kinematics.c`
- Modify: `ecu/control/include/motion_control.h`
- Modify: `ecu/control/src/motion_control.c`
- Modify: `ecu/vehicle/src/command_arbiter.c`
- Modify: `ecu/apps/agri_chassis_control_cpu0/CMakeLists.txt`

- [ ] Add vehicle geometry macros for wheelbase, track-width min/default/max and minimum turn radius.
- [ ] Extend `motion_control_limits_t` with `wheelbase_mm` and `track_width_mm`.
- [ ] Implement Ackermann from ICR: wheel position, angular velocity, per-wheel velocity vector, steering angle and signed wheel speed.
- [ ] Keep spin and crab in the same module with explicit, readable helpers.
- [ ] Add `sdk_ld_options("-lm")` because the kinematic module uses `atan2f`, `sqrtf` and `tanf`.
- [ ] Run `python tests\python\run_tests.py` and confirm all tests pass.

### Task 3: Verify firmware and regenerate SEGGER projects

**Files:**
- Generated: `tmp/cmake_cpu0_canopennode/segger_embedded_studio/agri_chassis_control_cpu0.emProject`
- Generated: `tmp/cmake_cpu1_latest/segger_embedded_studio/agri_chassis_control_cpu1.emProject`

- [ ] Run CPU0 and CPU1 Ninja builds.
- [ ] Run `git diff --check`.
- [ ] Re-run CMake `--fresh` for CPU0 and CPU1 to regenerate SEGGER projects.
- [ ] Build CPU0 and CPU1 again from regenerated directories.
- [ ] If J-Link is available, download `tmp/cmake_cpu0_canopennode/output/demo.elf`.
