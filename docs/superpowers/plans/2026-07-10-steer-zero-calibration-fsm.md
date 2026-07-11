# Steering Zero Calibration FSM Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Port the CAN-analyzer-verified four-steering-axis zero calibration procedure into the CPU0 CAN2 motion owner.

**Architecture:** Remote CH10/B1 only produces a gated request. The vehicle command path passes the request to `motion_device`, and the CAN2-owned motion service runs a bounded maintenance FSM that owns all steering PDO/SDO traffic until completion or fault. Normal drive/steer outputs are inhibited during calibration.

**Tech Stack:** C firmware, FreeRTOS task steps, CANopenNode-backed CANopen service, RPDO0 velocity command, SDO writes, HPM SDK 1.11.

## Global Constraints

- CAN2 motion task owns all CAN2 PDO/SDO motion traffic.
- Do not reset the drive node during zero calibration.
- Do not write 0x607C home offset.
- Use velocity-mode search and velocity-mode midpoint return.
- Final zeroing writes `0x6064:00 = 0` on Node5-8 only after all four axes have returned to midpoint.
- On timeout, fault, or invalid feedback, command all steering axes to zero velocity and leave normal motion inhibited until the operator clears/retries.

---

### Task 1: State and Diagnostics Contract

**Files:**
- Modify: `ecu/devices/include/motion_device.h`
- Modify: `ecu/config/include/ecu_config.h`
- Modify: `tests/python/test_hardware_framework.py`

**Interfaces:**
- Produces: `motion_steer_zero_calibration_state_t`, state fields in `motion_device_state_t`, and compile-time parameters used by the FSM.

- [ ] Add zero-calibration state enum and per-axis fields.
- [ ] Add timeout/velocity/current constants beside existing `ECU_STEER_ZERO_*` constants.
- [ ] Add static tests requiring states, 0x6064 zero write, and no reset/607C usage.

### Task 2: CAN2 Maintenance FSM Helpers

**Files:**
- Modify: `ecu/devices/src/motion_device.c`

**Interfaces:**
- Consumes: CANopen feedback snapshots from `canopen_master_service_get_node_feedback`.
- Consumes: existing `build_drive_velocity_rpdo_request` for RPDO0 velocity payload format.
- Produces: helpers to queue four steering velocity PDOs, queue SDO writes, stop all axes, and read actual position/current/velocity.

- [ ] Add helpers that map wheel index to Node5-8 and feedback.
- [ ] Add a grouped RPDO0 velocity sender for the steering nodes.
- [ ] Add SDO setup helpers for 0x2300, 0x6060, 0x6081, 0x6083, 0x6084, 0x6040, and 0x6064.
- [ ] Ensure helpers return false on queue failure and never block.

### Task 3: Search/Retreat/Return FSM

**Files:**
- Modify: `ecu/devices/src/motion_device.c`

**Interfaces:**
- Consumes: `state->steer_zero_calibration_requested`.
- Produces: one periodic function called from `motion_device_apply_command`.

- [ ] Implement states: idle, setup, search-left, retreat-left, search-right, retreat-right, return-mid, zero-write, complete, fault.
- [ ] Use three-stage search speeds from config based on travel distance from direction start.
- [ ] Accept an end stop by either current >= 9A or min travel plus zero-speed dwell.
- [ ] Compute midpoint per axis from left/right hit positions.
- [ ] Return to midpoint with closed-loop velocity, using fast/medium/slow return bands.
- [ ] Write 0x6064:00 = 0 for all four axes only after midpoint tolerance is satisfied.
- [ ] On any failure, stop all axes and enter fault.

### Task 4: Integration, Monitor, and Validation

**Files:**
- Modify: `ecu/devices/src/motion_device.c`
- Modify: `ecu/diag/include/runtime_monitor.h`
- Modify: `ecu/diag/src/runtime_monitor.c`
- Modify: `ecu/os/src/ecu_tasks_cpu0.c`
- Modify: `tests/python/test_hardware_framework.py`

**Interfaces:**
- Produces: runtime monitor fields for calibration state, phase masks, hit positions, and request count.

- [ ] Call the FSM from CAN2 motion execution before normal steering/drive PDO emission.
- [ ] Inhibit normal motion while FSM is active or faulted.
- [ ] Print compact diagnostics in COM9 monitor.
- [ ] Run static tests, `git diff --check`, CPU0 CMake/Ninja build, J-Link download, then hardware test with B1 triple press.

