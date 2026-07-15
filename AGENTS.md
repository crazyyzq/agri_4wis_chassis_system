# AGENTS.md — Agricultural 4WIS Chassis ECU

This file applies to the repository root and all descendants unless a nearer
`AGENTS.md` is stricter. The project is safety-relevant embedded vehicle
software; measured hardware evidence outranks assumptions.

## 1. Platform and architecture

- Windows 11 / PowerShell 7
- HPM6750IVM, ZLG MR6750 minimum system
- HPM SDK 1.11.0, FreeRTOS, CMake/Ninja, SEGGER Embedded Studio 8.28
- CPU0 exclusively owns final safety decisions and actuator output. CPU1 may
  publish non-critical snapshots but must not directly command actuators.

Required command path:

```text
SBUS / automatic request
  -> remote manager / producer
  -> command arbiter
  -> safety manager
  -> vehicle command executor
  -> device adapter
  -> CANopen / CAN / DIO / UART service
```

Never send motion, steering, lift, brake, hydraulic, high-voltage or warning
output directly from a parser, UI/debug helper, diagnostic task or unrelated
task.

## 2. Source of truth and repository boundaries

Hardware/configuration truth, highest priority first:

1. `ecu/config/include/ecu_config.h` and `ecu/config/src/ecu_config.c`
2. board configuration and installed wiring
3. vendor manuals in `doc/`
4. saved analyzer/readback evidence
5. explicitly labelled engineering assumptions

Current document entry point: `doc/README.md`.

Project-owned source:

```text
ecu/apps/  ecu/config/  ecu/control/  ecu/devices/  ecu/diag/
ecu/drivers/  ecu/ipc/  ecu/os/  ecu/protocol/  ecu/remote/  ecu/vehicle/
tests/python/  tools/  doc/  docs/
```

Do not modify `ecu/sdk_env_v1.11.0/`, `third_party/`, vendor SDK files or
generated IDE/toolchain files unless the user explicitly requests a toolchain
change. Generated `tmp/` and `out/` are not source of truth.

When code, documentation and measured evidence conflict, preserve safe output,
state the conflict and create a measured verification path. Do not invent a
constant.

## 3. Frozen vehicle topology and units

### CAN buses

```text
CAN1: BMS/power network, 250 kbit/s, J1939-style extended frames
CAN2: motion CANopen, 1 Mbit/s, standard frames
CAN3: lift/hydraulic CANopen, 1 Mbit/s, standard frames
CAN4: auxiliary/physical-test network, 500 kbit/s
```

### Nodes and vehicle positions

```text
CAN2
1 front-right drive     5 front-right steering
2 front-left drive      6 front-left steering
3 rear-left drive       7 rear-left steering
4 rear-right drive      8 rear-right steering

CAN3
9  front-right lift
10 rear-right lift
11 front-left lift
12 rear-left lift
13 hydraulic pump
```

Installed encoder contracts:

```text
Node1..8:  10000 count/motor revolution
Node9..12: 131072 count/motor revolution
Node13:    10000 count/motor revolution
```

Lift geometry is currently `20 motor rev / 10 mm`, therefore Node9..12 use
`262144 count/mm`. Extension moves absolute position negative. Normal remote
range is 10–490 mm; mechanical plausibility range is 0–500 mm with the
configured margin. Do not change or compensate the manually established drive
zero offset in normal control.

Do not swap node order, bus, bitrate, standard/extended format or unit scale in
local arithmetic. Change configuration and add a focused test plus hardware
verification note.

## 4. Current CANopen PDO contract

The only normal runtime mapping is saved Node1–13 `current7 + sync1`:

```text
RPDO0 0x200+node: 6040 + 6060 + 60FF, 7 bytes, type 1
RPDO1 0x300+node: 6040 + 6060 + 607A, 7 bytes, type 1
RPDO2 0x400+node: 60C1:01,             4 bytes, type 1
RPDO3 0x500+node: 6040 + 6060 + 2340, 5 bytes, type 1
TPDO0 0x180+node: 6064 + 606C,         8 bytes, type 1
TPDO1 0x280+node: 2183 + 6041 + 221C, 8 bytes, configured type 10
```

Canonical mapping record: `doc/CANOPEN_PDO_MAPPING_RECORD_V1.md`.

- `6060` stays inside RPDO0/RPDO1/RPDO3. `compact6` and `async255` are archived
  experiments, not production proposals.
- TPDO1 type-10 readback does not prove reduced bandwidth; the installed drives
  may still transmit it on every SYNC. Use analyzer evidence.
- TPDO2/TPDO3 remain zero-mapped.
- Runtime firmware must not remap PDOs or save drive flash. Offline mapping uses
  `tools/canopen_pdo_config/configure_all_nodes.py` with the ECU disconnected.
- SDO is for configuration/readback/diagnosis, not high-rate motion.
- Preserve CiA-402 controlword edges. Local queue or TX-complete success is not
  proof that a remote drive accepted a command; require TPDO/readback/evidence.

### Ordered realtime groups

CAN2 steering update:

```text
Node5..8 arm -> SYNC -> Node5..8 trigger -> SYNC
```

CAN3 lift point:

```text
Node9,11,12,10 RPDO2 points -> one SYNC every 20 ms
```

Use one strict FIFO realtime lane per bus. A group is complete only after every
expected frame completed and nothing remains in flight. Never interleave two
steering groups or inject a second periodic SYNC into the lift window. A partial
steering trigger is a motion safety fault: cancel unsent frames, command zero
traction through the approved safety path, re-establish eight-node feedback,
resynchronize steering and only then accept a newly published traction command.

## 5. Task ownership and concurrency

Expected CPU0 periods:

```text
safety 1 ms, CAN2 2 ms, remote 5 ms, vehicle 5 ms,
CAN1 10 ms, CAN3 10 ms, IO 10 ms, diagnostics 100 ms
```

- CAN2 task exclusively owns CAN2 PDO submission/completion, initialization and
  recovery state.
- CAN3 task owns the equivalent Node9–13 state and traffic.
- Vehicle task publishes one coherent complete command snapshot; it does not
  manipulate CAN queues or in-flight state.
- Cross-task multi-field data requires mailbox/sequence lock/critical snapshot,
  not `volatile` alone. Command snapshots carry a monotonic sequence.
- ISR work is flags/counters/wakeup only. No SDO, logging, blocking work or
  policy in ISR.
- No unbounded lock, busy wait, heap allocation or logging in periodic realtime
  paths. Diagnostics must be rate-limited and lower priority than CAN tasks.

## 6. Safety invariants

- `ECU_COMMISSIONING_STEER_ONLY_MODE` remains enabled by default.
- Never enable real drive, brake release, hydraulic motion, high voltage,
  firmware download or destructive NVM writes merely to test code.
- Estop, SBUS failsafe, A-class fault, stale command, critical CANopen failure
  and loss of required high-voltage feedback must publish explicit safe actuator
  intent. Never leave an old nonzero PDO active by only skipping new output.
- Safety/arbiter output is authoritative. Device adapters may reject unsafe
  output but cannot invent higher-level vehicle decisions.
- Do not NMT-reset drives as routine recovery; steering reset loses reference.
  Clear confirmed drive faults and rebuild the local state machine without
  replaying cached nonzero commands.
- Brake release and relay/MOS polarity are configuration-owned and fail safe.
- A watchdog, timeout, bus-off or recovery path must be deterministic and
  observable in diagnostics.

## 7. Current lift and hydraulic behavior

Ground-clearance lift is electric Node9–12 motion; it does not start Node13.

- Normal trajectory: 6 mm/s, 8 mm/s^2, three stationary preload points, then
  coherent four-axis RPDO2 + SYNC every 20 ms.
- In HOME-center idle, drives may be configured while disabled. Switch-on and
  operation-enable use separate four-axis RPDO1 groups plus SYNC, avoiding
  sequential brake release.
- Operator neutral is distinct from safety stop: smoothly decelerate, converge
  all four legs toward a common measured height at the configured leveling
  profile, confirm position/spread <=3 mm and zero speed for the configured
  stable samples, then disable all axes so drive brakes hold.
- Safety stop/high-voltage loss/stale command disables immediately; it does not
  wait for leveling.
- During motion, spread up to the running warning threshold is corrected rather
  than treated as an automatic stop. Final disable still requires <=3 mm.
- Below 10 mm, only extension toward the safe band is allowed. Above 490 mm,
  only retraction is allowed. Normal bidirectional control returns once all legs
  are inside 10–490 mm. Mixed low/high violations or mechanically implausible
  feedback block ordinary common-direction motion and expose diagnostic masks.
- Healthy BC/BC2 interpolation may report statusword `0x162F`. For this flow,
  mapped vendor latch `0x2183` is the hard drive-fault source; clear the received
  latch value without node reset.
- Node13 pump is reverse-only: suspension uses 1500 rpm, track-width uses
  2400 rpm, valves wait for fresh reverse-speed feedback above the configured
  pressure-build threshold. Pump RPDO is command-on-change, not periodic.
- Valve pairs 1/2, 3/4 and 5/6 are mutually exclusive and fail closed.

## 8. Remote and power essentials

- Remote parsing produces requests, never actuator output.
- Priority is safety > selected command source; remote/automatic/maintenance/
  CPU1 resolution belongs in the arbiter.
- Estop latches on any CH13 endpoint transition and clears only through the
  defined intentional center hold.
- Current channel roles and HOME/R1/R2/B1 behavior are maintained in
  `doc/ECU/遥控操作逻辑说明书.md`; do not duplicate thresholds in FSM source.
- CAN1 BMS/power traffic uses extended frames. High-voltage enable requires
  fresh plausible BMS feedback; stale CAN1 data never authorizes power.
- MOS6 is the battery-key/high-voltage relay output. Output polarity stays in
  hardware configuration.

## 9. Tools and evidence

Tool index: `tools/README.md`.

- PDO mapping tool: offline only, ECU physically disconnected; dry-run default.
- Motion/steering/lift scripts that require `--allow-motion` are hazardous bench
  tools, not production control. Confirm wheels/legs are mechanically safe.
- `steer4_zero_calibration_debug.py` is the reference for the ECU B1 triple-click
  workflow; it writes `0x6064=0` only after validated limit/midpoint completion.
- Preserve raw analyzer logs cited by documents. Historical logs prove only the
  stated test boundary, not current whole-vehicle acceptance.

## 10. Change discipline

- Inspect `git status --short --branch` before editing. Preserve unrelated and
  user-owned changes.
- For a narrow fix, inspect only direct configuration, callers/callees and
  focused tests. Do not combine a P0 fix with unrelated refactoring.
- Put tunable hardware values in configuration with physical units. Check fixed
  width, signedness, bounds, DLC/index validity and conversion overflow.
- Maintain dependency direction. Do not create project-local replacements for
  CANopenNode or Agile Modbus.
- Update current documents only for architecture, wiring, safety, configuration
  or test-procedure changes. Historical records must begin with an archive
  notice linking to `doc/README.md`; do not rewrite raw evidence as if it were a
  current acceptance result.
- Chinese Markdown/text is UTF-8. Read with:

```powershell
Get-Content -LiteralPath '<path>' -Raw -Encoding utf8
```

- Prefer `rg`/`rg --files`. Use `apply_patch` for edits.
- Do not modify/delete user files, use destructive Git commands, or clean the
  workspace. Commit, push, pull/merge/rebase, branch changes and destructive
  file/history operations require explicit user approval.

## 11. Validation

Default static runner:

```powershell
python tests\python\run_tests.py
```

For C/header/CMake/configuration changes also run `git diff --check` and the
CPU0 target build when the bundled SDK exists:

```powershell
$repo = (Get-Location).Path
$env:HPM_SDK_BASE = (Resolve-Path "$repo\ecu\sdk_env_v1.11.0\hpm_sdk").Path
$env:GNURISCV_TOOLCHAIN_PATH = (Resolve-Path "$repo\ecu\sdk_env_v1.11.0\toolchains\rv32imac_zicsr_zifencei_multilib_b_ext-win").Path
$cmake = (Resolve-Path "$repo\ecu\sdk_env_v1.11.0\tools\cmake\bin\cmake.exe").Path
$ninja = (Resolve-Path "$repo\ecu\sdk_env_v1.11.0\tools\ninja\ninja.exe").Path
$py = (Resolve-Path "$repo\ecu\sdk_env_v1.11.0\tools\python3\python.exe").Path
& $cmake -S "$repo\ecu\apps\agri_chassis_control_cpu0" `
  -B "$repo\tmp\cmake_cpu0" -G Ninja -DCMAKE_MAKE_PROGRAM="$ninja" `
  -Dpython_exec="$py" -DBOARD=ecu_isolation -DBOARD_SEARCH_PATH="$repo\ecu"
& $cmake --build "$repo\tmp\cmake_cpu0" --target all
```

Generated SEGGER project:

```text
tmp/cmake_cpu0/segger_embedded_studio/agri_chassis_control_cpu0.emProject
```

Static tests and successful compilation do not prove remote SDO/PDO acceptance,
motor movement, hydraulic behavior or vehicle safety. Say “analyzer-verified” or
“hardware-verified” only when saved measured evidence supports it.

## 12. Final report

Implementation reports state: what changed, files changed, validation actually
run, remaining hardware validation, and risks/assumptions. Review reports list
findings by severity with evidence and concrete fixes. Never claim completion
while a required safety behavior is silently left as a TODO.
