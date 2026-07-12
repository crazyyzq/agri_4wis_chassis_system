# AGENTS.md — Agricultural 4WIS Chassis ECU Project

This file applies to the repository root and all descendants unless a nearer
`AGENTS.md` adds stricter rules. It is the operating contract for Codex in this
repository.

## 1. Project identity and non-negotiable intent

This repository is an embedded ECU project for an agricultural four-wheel-drive,
four-wheel-steering chassis. It is safety-relevant hardware software, not a
generic desktop application.

Primary platform and toolchain:

- Windows 11, PowerShell 7 (`pwsh`)
- HPM6750-class dual-core ECU target
- FreeRTOS, HPM SDK, CMake, Ninja, SEGGER Embedded Studio
- CPU0 is the authoritative real-time control and safety core.
- CPU1 is not permitted to directly own final actuator outputs unless the user
  explicitly changes the architecture.

The normal control flow is:

```text
SBUS / auto request
  -> remote manager / producer
  -> command arbiter
  -> safety manager
  -> vehicle command executor
  -> device adapters
  -> CANopen / CAN / DIO / UART hardware services
```

Never bypass this flow by sending a drive, steering, lift, brake, high-voltage,
or warning-light command directly from a remote parser, UI/debug helper, or
unrelated task.

## 2. Repository boundaries and source-of-truth order

Treat these as project-owned code and documents:

```text
ecu/apps/agri_chassis_control_cpu0/
ecu/config/
ecu/devices/
ecu/diag/
ecu/drivers/
ecu/os/
ecu/remote/
ecu/vehicle/
tests/python/
doc/
tools/
```

Do **not** modify the following unless the user explicitly requests an SDK or
toolchain change:

```text
ecu/sdk_env_v1.11.0/
third_party/
generated toolchain files
vendor SDK files
```

For hardware parameters, do not invent constants. Use this source-of-truth
order:

1. `ecu/config/include/ecu_config.h`
2. hardware configuration structures and board configuration
3. vendor manuals already present in the repository / supplied by the user
4. analyzer capture or measured hardware evidence
5. clearly labeled engineering assumptions

When documentation, code, and analyzer evidence disagree, do not silently pick
one. State the conflict, preserve safety, and ask for or implement a measured
verification path.

## 3. Hardware topology that must not be casually changed

### CAN buses

```text
CAN1: power/BMS network, 250 kbit/s, J1939-style extended frames
CAN2: motion CANopen network, 1 Mbit/s
CAN3: lift/hydraulic CANopen network, 1 Mbit/s
```

### CAN2 node allocation

```text
Node 1: front-right drive
Node 2: front-left drive
Node 3: rear-left drive
Node 4: rear-right drive

Node 5: front-right steering
Node 6: front-left steering
Node 7: rear-left steering
Node 8: rear-right steering
```

### CAN3 node allocation

```text
Node 9 : front-right lift
Node 10: rear-right lift
Node 11: front-left lift
Node 12: rear-left lift
Node 13: pump / hydraulic auxiliary
```

### Encoder position-count contracts

```text
Node 1..8 : 10000 position counts per motor revolution
Node 9..12: 131072 position counts per motor revolution
Node 13   : 10000 position counts per motor revolution
```

These values are tied to installed node roles, not merely to the BC/BC2 family
name. Never use the Node1–8/13 velocity or position conversion for Node9–12.
Keep separate configuration symbols and focused tests for CAN2 motion, CAN3
lift, and the Node13 hydraulic pump.

Do not swap wheel order, node IDs, bus assignments, bitrate, standard versus
extended CAN format, or physical-unit conversions without modifying the
appropriate configuration and adding a focused test or explicit hardware
verification note.

## 4. Task ownership, concurrency, and timing rules

Expected CPU0 ownership:

```text
safety supervisor: 1 ms
CAN2 motion task:  2 ms
remote manager:    5 ms
vehicle control:   5 ms
CAN1 power task:  10 ms
CAN3 lift task:   10 ms
IO task:          10 ms
diagnostics:     100 ms
```

Mandatory ownership rules:

- The CAN2 task owns all realtime CAN2 PDO submission, CAN2 TX-complete
  observation, CAN2 device initialization progress, and CAN2 transmission
  state.
- The CAN3 task owns the equivalent CAN3 lift/hydraulic state and traffic.
- The vehicle task produces a coherent actuator-command snapshot. It must not
  directly manipulate CAN TX queues or CAN2/CAN3 in-flight state.
- Do not allow vehicle and CAN2 tasks to read/write a large
  `motion_device_state_t` concurrently without a mailbox, sequence lock,
  critical-section snapshot, or another explicitly correct synchronization
  mechanism.
- A command mailbox must carry a complete coherent command snapshot and a
  monotonic sequence counter. Do not assemble one four-wheel PDO group from
  mixed old/new wheel targets.
- Keep ISR work short. ISRs may capture flags/counters and wake processing;
  they must not perform blocking work, SDO workflows, logging, or policy logic.
- Do not take `portMAX_DELAY` locks in a high-priority periodic control path
  unless the lock order and bounded worst-case latency are documented.

## 5. Safety rules — always higher priority than convenience

Never weaken or bypass any of these without explicit user instruction and a
documented reason:

- `ECU_COMMISSIONING_STEER_ONLY_MODE` remains enabled by default.
- No real drive motion, brake release, hydraulic motion, high-voltage enable,
  firmware download, or destructive NVM write is allowed merely to “test code.”
- Never change a safety default from fail-safe to fail-open.
- An emergency stop, SBUS failsafe, A-class fault, stale command, or critical
  CANopen failure must produce explicit safe actuator intent. Do not merely
  skip sending the latest command and leave a prior nonzero command active.
- The final command arbiter/safety layer is authoritative. Device adapters may
  reject unsafe output but may not independently make higher-level driving
  decisions.
- A partial steering trigger group is a motion safety fault. If one or more
  trigger PDOs may already have reached drives while the rest failed, block
  fresh steering groups, command drive speed to zero through the approved
  safety path, and enter the defined automatic recovery path. Recovery may not
  replay a cached nonzero drive command: it must re-establish eight-node
  feedback, send one coherent four-steering-axis resynchronization group, and
  only then permit a newly published traction command.
- A watchdog, timeout, fault, or bus-off path must be observable through
  diagnostics and must have deterministic recovery behavior.
- Do not claim a remote device accepted a command only because it was inserted
  into a software queue or accepted by a local CAN peripheral.

## 6. CANopen rules

### General

- Use SDO for setup, mapping, configuration, diagnosis, and infrequent service
  actions. Do not use SDO as a high-rate realtime actuator transport.
- Use PDO for realtime motion only after the required setup sequence is
  successfully verified.
- NMT/SDO/PDO traffic for one bus must go through the bus-owned CANopen service.
  Do not create a second raw CAN path that can overtake, reorder, or bypass the
  service.
- Preserve CiA-402 controlword transition order. Do not coalesce distinct
  controlword edges such as enable/arm/trigger into only the final value.
- Do not treat `hpm_can_send(...) == 0` as physical successful bus delivery.
  It means submission to the local nonblocking TX path only.
- Treat a successful local TX completion flag as local transmitter completion,
  not proof that a remote application accepted the PDO. Remote acceptance
  requires appropriate TPDO/stateword/SDO/readback or hardware evidence.

### Current field PDO baseline

The saved Node1–13 profile is `current7 + sync1` and is the only normal ECU
runtime contract unless a new analyzer capture, readback, focused test and
configuration change are supplied together:

```text
RPDO0  0x200 + node: 6040 + 6060 + 60FF, 7 bytes, type 1
RPDO1  0x300 + node: 6040 + 6060 + 607A, 7 bytes, type 1
RPDO2  0x400 + node: 60C1:01,             4 bytes, type 1
RPDO3  0x500 + node: 6040 + 6060 + 2340, 5 bytes, type 1
TPDO0  0x180 + node: 6064 + 606C,         8 bytes, type 1
TPDO1  0x280 + node: 2183 + 6041 + 221C, 8 bytes, configured type 10
```

- `6060` remains inside RPDO0/RPDO1/RPDO3. Do not revive the historical
  `compact6` production proposal without a complete remap and validation.
- The tested BC/BC2 drives retain `0x1801:02 = 10`, but may still emit TPDO1
  on every SYNC. Do not infer an actual TPDO1 bandwidth reduction merely from
  an SDO readback; use an analyzer capture.
- PDO mapping and flash save belong only to the offline maintenance tool while
  the ECU is disconnected. Normal ECU firmware must not write mapping or save
  objects at boot/runtime.

### CAN2 steering realtime PDO

The current steering direction uses the saved seven-byte `current7` RPDO1
payload:

```text
0x6040 controlword (16-bit) + 0x6060 mode=1 (8-bit) +
0x607A target position (32-bit)
```

The exact COB-IDs, mapping, and controlword constants are configuration/vendor
manual source-of-truth. Do not hardcode alternate mappings in random call sites.

For a four-wheel arm/trigger group:

- Queue a complete group atomically in software before beginning transmission.
- Use one strict FIFO realtime TX lane; do not spill later frames into a second
  hardware mailbox that can overtake an earlier frame.
- Send at most one realtime PDO that is awaiting completion at a time unless a
  verified hardware FIFO/ordering design explicitly proves otherwise.
- Remove a frame from the software queue only after the designated TX-complete
  event is observed.
- Group completion requires all expected frames completed, zero failed frames,
  and zero in-flight frames.
- Do not begin trigger frames until every arm frame in that group completed.
- On a frame retry/timeout/error, cancel all unsent frames in that same group.
- Never interleave two steering groups.
- Keep diagnostics for group sequence, expected/submitted/completed/failed
  frames, phase, in-flight state, timestamps, and last failure.

Expected analyzer pattern for one synchronous steering target update:

```text
Node5 arm, Node6 arm, Node7 arm, Node8 arm,
SYNC,
Node5 trigger, Node6 trigger, Node7 trigger, Node8 trigger
SYNC
```

The analyzer must not show a later group inserted between those frames.

### CANopen initialization

“Queued locally” is not “configured remotely.”

A motion node may be marked realtime-ready only after the required per-node
state machine has confirmed, with SDO success/readback or equivalent verified
feedback:

```text
Pre-operational as required
-> disable/modify PDO mapping safely
-> configure mapping and communication parameters
-> verify mapping/communication values
-> enter Operational
-> select operating mode
-> configure profile limits/velocity/acceleration as required
-> enable operation
-> verify mode display (0x6061), stateword (0x6041), and no relevant fault
-> realtime-ready
```

Do not globally infer Node5–8 readiness from an empty SDO queue or a fixed
delay. Initialization state, timeout, retry count, last SDO result, and
readback must be tracked per node.

### SYNC / synchronous PDO

The current saved field profile uses synchronous RPDO type 1. This is validated
for the exact BC/BC2 network and must be preserved as an ordered transport:

- Steering position: all four arm PDOs -> SYNC -> all four trigger PDOs ->
  SYNC. Do not interleave a later group or a drive group into that sequence.
- Lift interpolation: four RPDO2 points -> one SYNC every 20 ms, after the
  explicit disable/clear-buffer/preload/enable/start sequence.
- A partial trigger, TPDO0/required TPDO1 feedback loss, boot-up, CiA-402 fault
  or heartbeat loss must be observable and recoverable without NMT node reset.
- A successful `0x1801:02 = 10` write is configuration evidence only; it is not
  permission to relax feedback supervision or assume a ten-SYNC TPDO cadence.

### Bench tests without drives attached

When drives are absent:

- It is valid to validate ECU-originated frame ordering with an active CAN
  analyzer that can acknowledge frames.
- It is not valid to claim remote SDO/PDO acceptance, motor enable, mode
  selection, or position motion.
- Avoid noisy periodic commissioning scans during analyzer-only tests. Use a
  bench-specific configuration switch or explicitly disabled scan path; never
  silently change the normal production default.
- A missing ACK / TX timeout during this test is a transport test condition, not
  proof that the motor-side protocol is correct.

## 7. Motion-control scope and future direction

Current commissioning priority:

```text
1. Stable, safe steering PDO ordering and fault handling
2. Per-node steering initialization/readback
3. Hardware validation of steering position and limits
4. Drive velocity PDO design and validation
5. Integrated 4WD + 4WS motion
6. CAN3 lift/hydraulic validation
```

Do not enable drive outputs while steering-only commissioning is active.

For future drive PDO work:

- Use one coherent four-wheel velocity group.
- Define target units, zero-speed watchdog behavior, acceleration/deceleration,
  reversal policy, brake release interlock, and TPDO feedback before enabling
  real movement.
- Do not reuse steering’s target-position trigger protocol for velocity mode.
- Include measured wheel direction/sign conventions and per-wheel inversion in
  configuration, not scattered arithmetic.

For steering:

- Keep per-axis sign, zero offset, mechanical range, software limits, profile
  velocity, acceleration/deceleration, and feedback validity configurable.
- Never use only a software “last commanded position” as if it were measured
  actual position once real hardware feedback is available.
- A calibration/homing workflow must be explicit and safe; do not auto-home
  moving hardware on startup.

## 8. CAN1, CAN3, DIO, remote, and diagnostics constraints

### CAN1 / BMS

- CAN1 BMS frames are extended/J1939-style. Do not treat them as standard
  CANopen frames.
- Preserve BMS source address, scaling, timeout, plausibility, and power-state
  checks.
- No high-voltage enable may be inferred from a stale or unvalidated BMS frame.

### CAN3 lift/hydraulic

- CAN3 must remain independently owned by its 10 ms task.
- Lift/hydraulic commands must flow through the command arbiter and safety
  manager.
- Do not use direct blocking CAN/SDO calls from the vehicle task.
- Define safe behavior for one lift axis fault, pump fault, position mismatch,
  timeout, and loss of CAN3 before enabling real hydraulic hardware.

### DIO / brakes / warning outputs

- Treat DIO mappings and active level as hardware configuration. Do not reverse
  polarity based on guesswork.
- Brake release must be explicitly interlocked with safety state and must
  default safe after reset/failsafe.
- Warning outputs must not delay safety or CAN realtime tasks.

### Remote and vehicle arbitration

- Remote parsing produces requests, not final actuator outputs.
- Keep gear, power, estop, and motion state machines explicit and testable.
- Estop clear must be intentional and edge/condition validated; do not clear it
  merely because one input sample looks healthy.
- Preserve priority: safety overrides all; auto/remote/maintenance/CPU1 must be
  resolved by the command arbiter, not by device adapters.

### Diagnostics

- Diagnostics must expose actionable state, not only counters.
- Include: safety source, selected command source, actuator intent, CAN bus
  state, CANopen group state, SDO failure details, per-node initialization
  status, stale-data ages, fault latches, and recovery state.
- Rate-limit diagnostic text. Do not flood serial logs from 1 ms/2 ms paths.
- Never let logging, printf, dynamic allocation, or filesystem access block
  realtime task execution.

## 9. Coding and change discipline

### Default work mode

- Default to implementing the user’s requested change. Use plan-only mode only
  when the user explicitly asks for a plan/review or when a change crosses
  multiple safety-critical subsystems.
- For a cross-subsystem/safety-critical change, provide a plan of at most
  8 bullets before editing. The plan must name affected modules, hazards, and
  test evidence.
- For a narrow fix, inspect only the named file(s), direct callers/callees,
  relevant headers/configuration, and focused tests. Do not scan or refactor
  the whole repository.
- Do not use subagents by default. Use at most two only for explicitly requested
  independent audit tasks. Do not spend turns narrating subagent setup.

### Scope control

- Do not mix a P0 bug fix with unrelated P1/P2 refactoring.
- Do not rename broad APIs, rewrite formatting, rearrange directories, or
  “modernize” unrelated code unless explicitly requested.
- Preserve existing public API names and test conventions unless the task
  specifically requires an interface change.
- When a wider problem is discovered, finish the requested safe scope and
  report the discovery under `Remaining risks`; do not silently expand scope.
- Do not add placeholder TODO code as a substitute for a required safe behavior.
- Do not fake hardware success, mock a safety response as passing, or weaken
  tests simply to make CI green.

### C / FreeRTOS requirements

- This is C firmware. Match existing C language level and HPM SDK conventions.
- Avoid heap allocation after initialization in periodic control paths.
- Check pointers, array bounds, unit conversion overflow, signedness, and CAN
  DLC/index validity.
- Use fixed-width integer types for protocol payloads.
- Keep critical sections small. Do not call blocking SDK functions, logging, or
  complex work inside a critical section.
- No busy-wait loops in periodic tasks or ISRs for CAN/SDO completion.
- Any timeout must have explicit state transition and recovery/failure behavior.
- All new persistent state must have an initialization path and a diagnostic
  path.
- Do not assume `volatile` alone makes a multi-field cross-task structure safe.

### Configuration and documentation

- Put tunable hardware parameters in configuration, not magic literals in
  device code.
- Document physical units at each API boundary.
- Update existing project documents only when the change affects architecture,
  wiring, safety behavior, configuration, or test procedure.
- Keep production documentation and historical analyzer evidence distinct. The
  current PDO mapping source is `doc/CANOPEN_PDO_MAPPING_RECORD_V1.md`; remote
  operation is `doc/ECU/遥控操作逻辑说明书.md`; current field outcomes are in
  `doc/ECU/整车调试记录_2026-07-07.md`. Any older debug record that recommends a
  superseded mapping, rate, mode, or safety path must carry an explicit archive
  notice and a link to the current source. Do not silently retain an obsolete
  “current/recommended/default” claim.
- Do not delete raw analyzer logs merely because they are old when a project
  document cites them as test evidence. Generated `out/` and `tmp/` contents
  remain untracked; remove them only with a user-requested cleanup scope and
  after preserving any specifically referenced evidence.
- Do not create large review documents unless the user requested a document.
- Chinese task/review documents intended for Windows users should be written as
  UTF-8 with BOM unless the repository explicitly requires otherwise. Source
  files should preserve their existing encoding.

## 10. PowerShell and text encoding

This is a Windows project containing Chinese Markdown/text.

For any non-ASCII text file, always read with:

```powershell
Get-Content -LiteralPath '<path>' -Raw -Encoding utf8
```

Rules:

- Never use bare `Get-Content` to read Chinese task documents.
- Do not spend a separate response reporting an encoding issue. Retry with
  `-Encoding utf8` and continue the task.
- Treat UTF-8 with BOM and UTF-8 without BOM as valid input.
- Use `pwsh` / PowerShell 7 where available.
- Prefer literal paths for Windows paths containing spaces or Chinese characters.
- Do not modify system locale, registry encoding settings, or global shell
  settings unless the user explicitly asks.

## 11. Testing and build policy

### Default test runner

The project’s default Python test entrypoint is:

```powershell
python tests\python\run_tests.py
```

Rules:

- Do **not** run, install, recommend, or spend a separate response explaining
  `pytest` unless the user explicitly asks for pytest.
- `run_tests.py` is a no-dependency source-level contract/safety test runner.
  It does not replace target compilation, CAN analyzer validation, or real
  hardware validation.
- Add or update focused tests when changing architecture contracts, mappings,
  safety state transitions, APIs, configuration, or source-level invariants.
- Do not claim a runtime/hardware property solely because a static Python test
  passed.

### Required validation by change type

```text
Markdown/docs only:
  - read back changed document with UTF-8
  - git diff --check

Python tests only:
  - python tests\python\run_tests.py
  - git diff --check

C / headers / CMake / FreeRTOS / CANopen / configuration:
  - python tests\python\run_tests.py
  - git diff --check
  - CPU0 CMake/Ninja build when the local SDK/toolchain is available

Hardware-facing behavior:
  - all relevant static tests/build checks
  - explicit analyzer / bench / hardware validation checklist
  - do not claim it is verified until measured evidence exists
```

### Canonical CPU0 configure/build procedure

Run from the repository root in PowerShell 7. Do not alter the SDK tree.

```powershell
$repo = (Get-Location).Path

$env:HPM_SDK_BASE = (Resolve-Path "$repo\ecu\sdk_env_v1.11.0\hpm_sdk").Path
$env:GNURISCV_TOOLCHAIN_PATH = (Resolve-Path "$repo\ecu\sdk_env_v1.11.0\toolchains\rv32imac_zicsr_zifencei_multilib_b_ext-win").Path

$cmake = (Resolve-Path "$repo\ecu\sdk_env_v1.11.0\tools\cmake\bin\cmake.exe").Path
$ninja = (Resolve-Path "$repo\ecu\sdk_env_v1.11.0\tools\ninja\ninja.exe").Path
$py = (Resolve-Path "$repo\ecu\sdk_env_v1.11.0\tools\python3\python.exe").Path

& $cmake -S "$repo\ecu\apps\agri_chassis_control_cpu0" `
  -B "$repo\tmp\cmake_cpu0" `
  -G Ninja `
  -DCMAKE_MAKE_PROGRAM="$ninja" `
  -Dpython_exec="$py" `
  -DBOARD=ecu_isolation `
  -DBOARD_SEARCH_PATH="$repo\ecu"

& $cmake --build "$repo\tmp\cmake_cpu0" --target all
```

The SEGGER project is generated at:

```text
tmp\cmake_cpu0\segger_embedded_studio\agri_chassis_control_cpu0.emProject
```

Do not manually edit generated SEGGER project files as the source of truth.
Regenerate them through CMake after relevant CMake/configuration changes.

If the SDK/toolchain is unavailable, report that build was not run and why.
Do not try to install a different compiler, Python package suite, or unrelated
toolchain without explicit approval.

## 12. Git and file safety

Allowed without special approval:

```text
git status --short --branch
git diff
git diff --check
git log --oneline
git show
git grep
```

Require explicit user approval before doing any of the following:

```text
git commit
git push
git pull that can merge/rebase
git switch / checkout another branch
git reset --hard
git clean
git rebase
git merge
git cherry-pick
git tag
force push
delete branch/file/history
```

Additional rules:

- Never use `git clean -fdx` in this repository; it can remove local SDK/tool
  environment assets and generated work the user may need.
- Never overwrite a user’s uncommitted work to make a patch apply.
- Before edits, inspect `git status --short --branch`.
- Before final response, inspect `git diff --check`.
- Do not commit generated build outputs, IDE state, analyzer captures, secrets,
  personal paths, or SDK binaries unless the user explicitly requests them.

## 13. Final response contract

Do not give a long diary of commands, tool limitations, encoding details, or
unavailable pytest. Report only what helps the user make the next decision.

For implementation tasks, final response must contain:

```text
1. What changed
2. Files changed
3. Validation actually run and result
4. Hardware/field validation still required
5. Any blocking risk or assumption
```

For review-only tasks, final response must contain:

```text
1. Findings ordered P0 -> P3
2. Evidence (file/function/config)
3. Concrete recommended fix
4. What was not verified
```

Use honest wording:

- “Compiled successfully” only after the target build actually succeeded.
- “Analyzer-verified” only after a supplied/observed capture supports it.
- “Hardware-verified” only after real node/device evidence exists.
- “Static test passed” means source-level contract only.

Do not claim completion while silently leaving safety-critical behavior as a
future task. Do not create fake certainty from queue success, comments, or
test-token matching.

## 14. Fast pre-flight checklist

Before editing:

```text
[ ] Read task scope and current git status
[ ] Identify the owning task/module and safety impact
[ ] Read direct headers, configuration, callers/callees, and focused tests
[ ] Preserve CPU0/CAN task ownership
[ ] Avoid SDK and unrelated files
```

Before finishing:

```text
[ ] No unsafe direct actuator path introduced
[ ] No mixed multi-wheel snapshot / PDO group interleaving introduced
[ ] Units, node IDs, COB-IDs, and bus assignment remain configuration-driven
[ ] Relevant static test runner executed
[ ] git diff --check executed
[ ] CPU0 build run when relevant and available
[ ] Remaining analyzer/hardware validation clearly stated
```
