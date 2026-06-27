# ECU Main Control Framework Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Subagents are disabled for this session by higher-priority instruction, so execute inline with review checkpoints. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the main-branch ECU application framework required by `doc/ECU_Project_Implementation_v1.4.md`: FreeRTOS/AMP application scaffolding, single-owner state machines, normalized requests, command arbitration, safety override, executor boundary, diagnostics, and regression tests.

**Architecture:** CPU0 owns all safety-critical control. CPU1 only publishes validated non-critical communication snapshots through IPC. Business logic is split into small modules: `remote` converts SBUS snapshots into requests and FSM state, `vehicle` arbitrates and safety-overrides requests into one complete actuator command, `control` holds kinematic/adjustment command shapes, `devices` and `drivers` are executor-facing boundaries, and `app` starts tasks. Hardware-specific commands stay behind `vehicle_command_executor` and are stubbed until CANopen/hydraulic mappings are finalized.

**Tech Stack:** C99, HPM SDK 1.11, SEGGER Embedded Studio 8.28, FreeRTOS, HPM6750 AMP/RPMsg-compatible layout, Python 3.14 for executable behavior tests, RISC-V GCC for target syntax/build checks.

---

## File Structure

Create these paths under `ecu/`:

- `apps/agri_chassis_control_cpu0/`: CPU0 FreeRTOS app, task creation, `FreeRTOSConfig.h`, CMake and SDK app metadata.
- `apps/agri_chassis_control_cpu1/`: CPU1 non-critical communication app scaffold and IPC publication loop.
- `common/include/ecu_types.h`: shared fixed-width enums and small value types used across modules.
- `common/include/ecu_time.h`: monotonic millisecond timestamp helper types; no hardware access.
- `config/include/ecu_config.h`, `config/src/ecu_config.c`: calibration constants, SBUS channel map, debounce windows, timeouts, task periods, mode limits, and safe defaults.
- `diag/include/diag_codes.h`, `diag/src/diag_codes.c`: stable diagnostic code enum and string names.
- `protocol/sbus/include/sbus_decoder.h`, `protocol/sbus/src/sbus_decoder.c`: 25-byte SBUS decoder; ISR-safe parser is not added here.
- `drivers/sbus/include/sbus_service.h`, `drivers/sbus/src/sbus_service.c`: CPU0 SBUS snapshot owner boundary with ISR-style byte feed API and foreground snapshot copy.
- `remote/include/remote_types.h`: remote input snapshot, request, FSM states, event and source enums.
- `remote/include/remote_discrete.h`, `remote/src/remote_discrete.c`: discrete channel thresholding, debounce, changed-once event, and baseline synchronization.
- `remote/include/remote_link_fsm.h`, `remote/src/remote_link_fsm.c`: online/qualifying/failsafe/offline link FSM.
- `remote/include/remote_arm_fsm.h`, `remote/src/remote_arm_fsm.c`: arm readiness FSM; neutral only gates entry, not active driving.
- `remote/include/remote_estop_fsm.h`, `remote/src/remote_estop_fsm.c`: source-tracked remote emergency stop latch and reset FSM.
- `remote/include/remote_gear_fsm.h`, `remote/src/remote_gear_fsm.c`: requested vs active gear, brake-safe D/R entry, track-compliant P state.
- `remote/include/remote_mode_fsm.h`, `remote/src/remote_mode_fsm.c`: HOME domain, old R1/R2 suppression, requested/active motion mode split.
- `remote/include/remote_adjust_fsm.h`, `remote/src/remote_adjust_fsm.c`: clearance/track owner, track prepare/active/exiting/abort states.
- `remote/include/remote_lights_fsm.h`, `remote/src/remote_lights_fsm.c`: indicator, horn and headlight request FSM.
- `remote/include/remote_manager.h`, `remote/src/remote_manager.c`: one-owner remote manager that updates all remote FSMs from an immutable SBUS snapshot.
- `vehicle/include/vehicle_types.h`: safety snapshot, auto request, actuator command, and actuator feedback types.
- `vehicle/include/command_arbiter.h`, `vehicle/src/command_arbiter.c`: rebuild final command each cycle from safe defaults plus winning request.
- `vehicle/include/safety_manager.h`, `vehicle/src/safety_manager.c`: override final command on A faults, estop, failsafe, controlled stop, and shutdown constraints.
- `vehicle/include/vehicle_command_executor.h`, `vehicle/src/vehicle_command_executor.c`: only hardware command exit; first version records/applies stub commands and never touches hardware directly from FSMs.
- `vehicle/include/vehicle_state.h`, `vehicle/src/vehicle_state.c`: CPU0-owned snapshot publication for diagnostics and CPU1.
- `control/include/motion_control.h`, `control/src/motion_control.c`: command shapes for Ackermann/reverse Ackermann/spin/crab; initial implementation clamps and labels targets only.
- `control/include/adjust_control.h`, `control/src/adjust_control.c`: body-height rate integration and track-adjust command shape without per-cylinder fake closed loop.
- `ipc/include/ipc_snapshot.h`, `ipc/src/ipc_snapshot.c`: sequence/timestamp/valid/CRC snapshot buffer used between CPU0 and CPU1.
- `os/include/ecu_tasks.h`, `os/src/ecu_tasks_cpu0.c`, `os/src/ecu_tasks_cpu1.c`: FreeRTOS task declarations and CPU-specific task orchestration.
- `tests/python/test_remote_fsm.py`: executable behavior tests for SBUS baseline, HOME suppression, estop, gear, adjust owner and lights.
- `tests/python/test_vehicle_arbiter.py`: executable behavior tests for final command rebuild, priority, safety override and executor boundary.
- `tests/python/run_tests.py`: no-dependency Python test runner used on this machine.
- `tools/check_no_forbidden_patterns.py`: static guard against direct hardware writes from `remote/` and magic SBUS thresholds in business modules.

Keep existing `ecu/ecu_isolation/` BSP in place during this phase. A later migration may move it to `ecu/boards/ecu_isolation/`, but this plan does not rename board files while untracked hardware documents are present.

---

## Task 1: Commit the requirement document and write progress checkpoint

**Files:**
- Add: `doc/ECU_Project_Implementation_v1.4.md`
- Add: `docs/ecu-main-control-progress.md`

- [ ] **Step 1: Stage only the v1.4 requirement file**

Run:

```powershell
git add doc/ECU_Project_Implementation_v1.4.md
git status --short
```

Expected: only the requirement file is staged; other untracked hardware folders remain untracked.

- [ ] **Step 2: Create progress checkpoint**

Create `docs/ecu-main-control-progress.md` with:

```markdown
# ECU Main Control Progress

## Active implementation basis

- Requirement source: `doc/ECU_Project_Implementation_v1.4.md`
- Branch: `main`
- First framework scope: complete module boundaries, state machines, arbitration, safety override, executor stub, CPU0/CPU1 FreeRTOS app skeleton, Python behavior tests, target syntax/build checks.

## Non-negotiable rules

- Remote/state-machine modules never write CANopen, GPIO, MOS, brake, hydraulic valve or high-voltage outputs.
- CPU1 never commands walking, steering, brake release, high voltage or hydraulic valves.
- Every final actuator command is rebuilt through `command_arbiter`, safety-overridden by `safety_manager`, and emitted only through `vehicle_command_executor`.
- Unsafe mode, gear, high-voltage and adjustment requests are rejected immediately; they are not queued.

## Open hardware details

- CANopen PDO/object mappings are not frozen in this framework commit.
- Hydraulic valve final mapping is not frozen in this framework commit.
- Brake feedback timing is not frozen in this framework commit.
- SBUS exact calibrated thresholds remain configuration values.
```

- [ ] **Step 3: Commit requirement checkpoint**

Run:

```powershell
git add docs/ecu-main-control-progress.md
git commit -m "docs: add main control framework requirements"
```

Expected: one docs commit; no code yet.

---

## Task 2: Add shared types, configuration and diagnostics

**Files:**
- Create: `ecu/common/include/ecu_types.h`
- Create: `ecu/common/include/ecu_time.h`
- Create: `ecu/config/include/ecu_config.h`
- Create: `ecu/config/src/ecu_config.c`
- Create: `ecu/diag/include/diag_codes.h`
- Create: `ecu/diag/src/diag_codes.c`
- Test: `tests/python/test_vehicle_arbiter.py`

- [ ] **Step 1: Write failing Python expectations**

Add a small test that imports generated enum tables from text and verifies these names exist:

```python
REQUIRED_DIAG_NAMES = [
    "DIAG_OK",
    "DIAG_REMOTE_ESTOP_CH13",
    "DIAG_REMOTE_ESTOP_SBUS_TIMEOUT",
    "DIAG_REJECT_NOT_PARKED",
    "DIAG_REJECT_THROTTLE_NOT_LOW",
    "DIAG_REJECT_MODE_PRECONDITION",
    "DIAG_TRACK_ADJUST_ABORTED",
]
```

Run:

```powershell
python tests\python\run_tests.py
```

Expected: FAIL because files do not exist.

- [ ] **Step 2: Implement shared headers**

Define explicit enums for domains, motion modes, gear request/active state, estop source, diagnostic code and command source. Define no anonymous `bool` flags for complex states.

- [ ] **Step 3: Implement configuration**

Provide a single immutable `ecu_config_t` returned by `ecu_config_default()`. Include debounce and timeout values from the requirement: 80 ms discrete debounce, 1000 ms link qualify, 300 ms neutral qualify, 100 ms failsafe timeout, 150 ms domain guard. Include SBUS channel semantic indexes but not raw magic thresholds in business code.

- [ ] **Step 4: Implement diagnostics**

`diag_code_name(diag_code_t code)` must return stable strings and `"DIAG_UNKNOWN"` for invalid codes.

- [ ] **Step 5: Run tests and target syntax check**

Run:

```powershell
python tests\python\run_tests.py
ecu\sdk_env_v1.11.0\toolchains\rv32imac_zicsr_zifencei_multilib_b_ext-win\bin\riscv32-unknown-elf-gcc.exe -std=c99 -Wall -Wextra -Iecu\common\include -Iecu\config\include -Iecu\diag\include -fsyntax-only ecu\config\src\ecu_config.c ecu\diag\src\diag_codes.c
```

Expected: Python tests pass; GCC reports no diagnostics.

---

## Task 3: Add SBUS decoder and SBUS service boundary

**Files:**
- Create: `ecu/protocol/sbus/include/sbus_decoder.h`
- Create: `ecu/protocol/sbus/src/sbus_decoder.c`
- Create: `ecu/drivers/sbus/include/sbus_service.h`
- Create: `ecu/drivers/sbus/src/sbus_service.c`
- Test: `tests/python/test_remote_fsm.py`

- [ ] **Step 1: Add failing decoder and snapshot tests**

Test a valid 25-byte SBUS frame, bad length, bad start/end marker, failsafe bit, frame-lost bit, and snapshot timeout source. Expected initial result: FAIL because decoder/service files do not exist.

- [ ] **Step 2: Implement decoder**

Decode 16 channels, CH17, CH18, frame-lost and failsafe. Do not allocate memory and do not print.

- [ ] **Step 3: Implement service boundary**

Expose `sbus_service_init`, `sbus_service_feed_byte_from_isr`, `sbus_service_poll`, and `sbus_service_get_snapshot`. ISR feed only appends bytes, timestamps and error counters; foreground copy produces immutable snapshots.

- [ ] **Step 4: Verify**

Run Python tests and syntax-check decoder/service with RISC-V GCC. Expected: pass.

---

## Task 4: Add remote discrete channel and link/arm/estop FSMs

**Files:**
- Create: `ecu/remote/include/remote_types.h`
- Create: `ecu/remote/include/remote_discrete.h`
- Create: `ecu/remote/src/remote_discrete.c`
- Create: `ecu/remote/include/remote_link_fsm.h`
- Create: `ecu/remote/src/remote_link_fsm.c`
- Create: `ecu/remote/include/remote_arm_fsm.h`
- Create: `ecu/remote/src/remote_arm_fsm.c`
- Create: `ecu/remote/include/remote_estop_fsm.h`
- Create: `ecu/remote/src/remote_estop_fsm.c`
- Test: `tests/python/test_remote_fsm.py`

- [ ] **Step 1: Write failing behavior tests**

Cover:

- link qualifies only after stable valid frames;
- SBUS timeout/failsafe produce different estop sources;
- arm uses neutral to enter ready but remains ready during valid throttle/steering;
- CH13 high triggers immediately even during baseline/domain guard;
- estop reset requires normal conditions and CH13 returning low.

- [ ] **Step 2: Implement modules**

Each FSM has `init`, `update`, `get_state`. No module writes hardware or includes CAN/board headers.

- [ ] **Step 3: Verify**

Run Python tests, forbidden-pattern check, and RISC-V GCC syntax check. Expected: pass.

---

## Task 5: Add gear, mode, adjustment and lights FSMs

**Files:**
- Create: `ecu/remote/include/remote_gear_fsm.h`
- Create: `ecu/remote/src/remote_gear_fsm.c`
- Create: `ecu/remote/include/remote_mode_fsm.h`
- Create: `ecu/remote/src/remote_mode_fsm.c`
- Create: `ecu/remote/include/remote_adjust_fsm.h`
- Create: `ecu/remote/src/remote_adjust_fsm.c`
- Create: `ecu/remote/include/remote_lights_fsm.h`
- Create: `ecu/remote/src/remote_lights_fsm.c`
- Test: `tests/python/test_remote_fsm.py`

- [ ] **Step 1: Write failing behavior tests**

Cover:

- D/R request keeps active gear P until throttle re-enable and brake release path;
- D↔R while moving is rejected with a diagnostic;
- HOME domain guard suppresses old R1/R2 state;
- mode requests reject immediately when preconditions fail;
- CH2/CH14 owner is exclusive and released only after both return neutral;
- track adjust exit keeps active gear P until exit completes;
- safety hazard overrides user indicators.

- [ ] **Step 2: Implement FSMs**

Use requested/transition/active fields. Do not queue rejected requests. All outputs are requests and diagnostics only.

- [ ] **Step 3: Verify**

Run Python tests, forbidden-pattern check, and RISC-V GCC syntax check. Expected: pass.

---

## Task 6: Add remote manager

**Files:**
- Create: `ecu/remote/include/remote_manager.h`
- Create: `ecu/remote/src/remote_manager.c`
- Test: `tests/python/test_remote_fsm.py`

- [ ] **Step 1: Write failing manager integration tests**

Feed immutable SBUS snapshots and verify one `remote_control_request_t` is produced with link state, arm state, estop state, gear request, mode request, adjustment request, lights and diagnostics.

- [ ] **Step 2: Implement manager**

The manager is the only writer of remote FSM state. It performs baseline sync on first valid frame, reconnect and HOME domain change.

- [ ] **Step 3: Verify**

Run Python tests and syntax check. Expected: pass.

---

## Task 7: Add vehicle command, arbitration, safety manager and executor boundary

**Files:**
- Create: `ecu/vehicle/include/vehicle_types.h`
- Create: `ecu/vehicle/include/command_arbiter.h`
- Create: `ecu/vehicle/src/command_arbiter.c`
- Create: `ecu/vehicle/include/safety_manager.h`
- Create: `ecu/vehicle/src/safety_manager.c`
- Create: `ecu/vehicle/include/vehicle_command_executor.h`
- Create: `ecu/vehicle/src/vehicle_command_executor.c`
- Test: `tests/python/test_vehicle_arbiter.py`

- [ ] **Step 1: Write failing arbitration tests**

Cover complete command rebuild from safe defaults, estop priority over manual/auto, A fault overriding high voltage/hydraulic/brake release, mode transition over normal driving, and executor being the only module with `vehicle_command_executor_apply`.

- [ ] **Step 2: Implement command types**

`vehicle_actuator_command_t` contains full desired outputs every cycle: walking speed, steering mode, steering targets, brake command, high-voltage request, hydraulic pump/valve request, lights, horn, diagnostics and source.

- [ ] **Step 3: Implement arbiter and safety manager**

Arbiter chooses source priority and rebuilds command from `vehicle_actuator_command_safe_default()`. Safety manager clamps dangerous fields to safe values when faults, estop, failsafe or shutdown constraints are active.

- [ ] **Step 4: Implement executor stub**

Executor accepts only final commands, stores the last applied command, increments a sequence number and returns a status. It does not touch CAN/GPIO until device mappings are frozen.

- [ ] **Step 5: Verify**

Run Python tests, forbidden-pattern check and syntax check. Expected: pass.

---

## Task 8: Add control command-shape modules

**Files:**
- Create: `ecu/control/include/motion_control.h`
- Create: `ecu/control/src/motion_control.c`
- Create: `ecu/control/include/adjust_control.h`
- Create: `ecu/control/src/adjust_control.c`
- Test: `tests/python/test_vehicle_arbiter.py`

- [ ] **Step 1: Write failing tests**

Cover clamping of speed/steering commands, spin disabling CH1 steering, crab holding angle, height rate integration with limits, and track adjust producing one global rate plus per-wheel configured posture/assist limits.

- [ ] **Step 2: Implement command-shape functions**

Functions accept calibrated units and config, produce candidates only. They never call executor or hardware.

- [ ] **Step 3: Verify**

Run Python tests and syntax check. Expected: pass.

---

## Task 9: Add IPC snapshot and vehicle state publication

**Files:**
- Create: `ecu/ipc/include/ipc_snapshot.h`
- Create: `ecu/ipc/src/ipc_snapshot.c`
- Create: `ecu/vehicle/include/vehicle_state.h`
- Create: `ecu/vehicle/src/vehicle_state.c`
- Test: `tests/python/test_vehicle_arbiter.py`

- [ ] **Step 1: Write failing tests**

Cover sequence increment, timestamp propagation, validity bit, CRC mismatch rejection and timeout invalidation.

- [ ] **Step 2: Implement double-buffer snapshot**

Use explicit copy functions. `volatile` alone is not used as a synchronization guarantee. Host implementation uses critical-section stubs; target app can wrap these with FreeRTOS/interrupt primitives later.

- [ ] **Step 3: Verify**

Run Python tests and syntax check. Expected: pass.

---

## Task 10: Add CPU0 and CPU1 FreeRTOS app scaffolds

**Files:**
- Create: `ecu/apps/agri_chassis_control_cpu0/CMakeLists.txt`
- Create: `ecu/apps/agri_chassis_control_cpu0/app.yaml`
- Create: `ecu/apps/agri_chassis_control_cpu0/src/main_cpu0.c`
- Create: `ecu/apps/agri_chassis_control_cpu0/src/FreeRTOSConfig.h`
- Create: `ecu/apps/agri_chassis_control_cpu1/CMakeLists.txt`
- Create: `ecu/apps/agri_chassis_control_cpu1/app.yaml`
- Create: `ecu/apps/agri_chassis_control_cpu1/src/main_cpu1.c`
- Create: `ecu/apps/agri_chassis_control_cpu1/src/FreeRTOSConfig.h`
- Create: `ecu/os/include/ecu_tasks.h`
- Create: `ecu/os/src/ecu_tasks_cpu0.c`
- Create: `ecu/os/src/ecu_tasks_cpu1.c`

- [ ] **Step 1: Write static checks**

Add static check that CPU1 app source does not include executor, CAN2 motion, brake, hydraulic valve or high-voltage command headers.

- [ ] **Step 2: Implement CPU0 tasks**

Create task names/periods/priorities matching the requirement:

- `task_safety_supervisor`: 1 ms
- `task_can2_motion`: 1-5 ms placeholder period set to 2 ms
- `task_remote_manager`: 5 ms
- `task_vehicle_control`: 5 ms
- `task_can1_power`: 10 ms
- `task_can3_lift_hydraulic`: 10 ms
- `task_io_service`: 10 ms
- `task_diag_cpu0`: 100 ms

Each task calls a small step function or placeholder hook and uses `vTaskDelayUntil`, never busy waits.

- [ ] **Step 3: Implement CPU1 tasks**

Create non-critical communication and logging task placeholders. CPU1 publishes only IPC snapshots and refuses direct safety-critical command APIs by dependency absence.

- [ ] **Step 4: Verify**

Run static checks and RISC-V GCC syntax checks on app sources. Expected: pass.

---

## Task 11: Add static quality gates and project documentation

**Files:**
- Create: `tools/check_no_forbidden_patterns.py`
- Create: `docs/ecu-main-control-architecture.md`
- Modify: `README.md`

- [ ] **Step 1: Implement static guard**

The script fails when:

- `ecu/remote/**` includes `board.h`, `hpm_can*`, `hpm_gpio*`, or executor headers;
- `ecu/control/**` includes remote private FSM headers;
- raw SBUS values `1050`, `1500`, `1950`, `1900` appear outside `ecu/config/**`;
- `vTaskDelay(` appears in state machine modules;
- file names `common.c`, `misc.c`, or `utils.c` are added under `ecu/`.

- [ ] **Step 2: Document architecture**

Write a short architecture file explaining dependency direction, owner task per state, final execution chain, CPU0/CPU1 split, and how to add the next real device driver without bypassing executor.

- [ ] **Step 3: Verify**

Run:

```powershell
python tools\check_no_forbidden_patterns.py
python tests\python\run_tests.py
git diff --check
```

Expected: all pass.

---

## Task 12: Full verification and commit

**Files:**
- All files above.

- [ ] **Step 1: Run full checks**

Run:

```powershell
python tools\check_no_forbidden_patterns.py
python tests\python\run_tests.py
git diff --check
```

Also run the available RISC-V GCC syntax-check batch for all newly added C files.

- [ ] **Step 2: Review status**

Run:

```powershell
git status --short
git diff --stat
```

Expected: only planned files changed; unrelated untracked hardware documentation stays untracked unless the user explicitly wants it committed.

- [ ] **Step 3: Commit**

Run:

```powershell
git add ecu docs README.md tools tests doc/ECU_Project_Implementation_v1.4.md
git commit -m "feat: add main ecu control framework"
```

Expected: one framework commit on `main`.

---

## Self-Review

- Requirement coverage: the plan covers dependency direction, CPU0/CPU1 split, remote link/arm separation, old event suppression, P/track-compliant split, no queued mode requests, estop source tracking, track adjustment constraints, final executor boundary, no blocking waits, diagnostic codes, and tests/static checks.
- Intentional gaps: real CANopen PDO/object dictionaries, brake feedback timing, hydraulic valve pin/MOS mapping, BMS/DC/DC/inverter sequences, and exact SBUS calibrated thresholds remain configuration/device-detail work because the requirement lists them as still needing real-vehicle confirmation.
- Placeholder scan: no task leaves an undefined future implementation as an instruction; executor and app hooks are deliberate boundaries with safe behavior.
- Type consistency: all modules depend on shared `ecu_types.h`, `remote_types.h`, and `vehicle_types.h`; private state remains module-owned.
