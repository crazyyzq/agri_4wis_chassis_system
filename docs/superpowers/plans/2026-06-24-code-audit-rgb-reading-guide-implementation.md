# ECU Code Audit, RGB Status, and Reading Guide Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Audit the complete hand-written ECU board-test firmware, add a non-blocking RGB runtime status indicator, improve maintainability comments, and publish a file-by-file reading guide.

**Architecture:** Keep electrical GPIO ownership in the board package and add an application-level `status_led` state machine. The state machine reads HPM6750 MCHTMR time through an injectable hardware-operations interface, so target self-tests can verify transitions without delays. UART console input becomes polled so the READY heartbeat continues while the shell waits for input; tests temporarily override the status animation and cleanup always restores it.

**Tech Stack:** C11, HPM SDK 1.11.0, HPM6750 MCHTMR/UART/GPIO drivers, CMake/Ninja GNU RISC-V toolchain, SEGGER Embedded Studio 8.28, J-Link, PowerShell, Markdown.

---

## File map

New files:

- `ecu/apps/ecu_board_test/include/status_led.h`: public status states, hardware-operations contract, polling and override API.
- `ecu/apps/ecu_board_test/src/status_led.c`: deterministic LED state machine and MCHTMR-backed target adapter.
- `ecu/apps/ecu_board_test/selftest/selftest_status_led.c`: transition, timing, override and restoration tests.
- `docs/ecu-board-test-code-reading-guide.md`: project tree, call flow, reading order and per-file responsibilities.

Primary modified files:

- `ecu/apps/ecu_board_test/CMakeLists.txt`: compile the status module and its self-test.
- `ecu/apps/ecu_board_test/include/selftest.h`, `selftest/selftest.c`: register status self-test and propagate aggregate result.
- `ecu/apps/ecu_board_test/src/main.c`: BOOTING/FAILED/READY boot sequence.
- `ecu/apps/ecu_board_test/src/operator_io.c`: polled UART line input with animation service.
- `ecu/apps/ecu_board_test/src/app_shell.c`: TESTING/READY/FAILED transitions around test sessions.
- `ecu/apps/ecu_board_test/src/test_runner.c`: service LED while polling abort and preserve cleanup.
- `ecu/apps/ecu_board_test/tests/test_rgb.c`: explicit RGB override ownership and restoration.

Audit/comment scope:

- All hand-written `.c/.h` under `ecu/apps/ecu_board_test`.
- `ecu/boards/ecu_isolation/board.c` and `board.h`; generated `pinmux.c`, `pinmux.h`, and `pinmux.hpmpc` are inspected for integration but not rewritten line-by-line.
- `ecu/scripts/*.ps1`, `ecu/scripts/*.jlink`, three custom linker files, `CMakeLists.txt`, `app.yaml`, board YAML/CFG, and operator fixtures.

### Task 1: Add a testable RGB status state machine

**Files:**

- Create: `ecu/apps/ecu_board_test/include/status_led.h`
- Create: `ecu/apps/ecu_board_test/src/status_led.c`
- Create: `ecu/apps/ecu_board_test/selftest/selftest_status_led.c`
- Modify: `ecu/apps/ecu_board_test/include/selftest.h`
- Modify: `ecu/apps/ecu_board_test/selftest/selftest.c`
- Modify: `ecu/apps/ecu_board_test/CMakeLists.txt`

- [ ] **Step 1: Declare the state-machine contract and write the failing self-test**

Define states and an injectable target adapter in `status_led.h`:

```c
typedef enum {
    STATUS_LED_BOOTING,
    STATUS_LED_READY,
    STATUS_LED_TESTING,
    STATUS_LED_FAILED
} status_led_state_t;

typedef struct {
    uint32_t (*now_ms)(void);
    void (*write)(uint8_t color, bool on);
} status_led_hw_ops_t;

void status_led_init(const status_led_hw_ops_t *ops);
void status_led_init_default(void);
void status_led_set(status_led_state_t state);
status_led_state_t status_led_get(void);
void status_led_poll(void);
void status_led_override_begin(void);
void status_led_override_end(void);
bool status_led_is_overridden(void);
```

In `selftest_status_led.c`, use fake time and three captured color levels. Assert:

```c
status_led_init(&fake_ops);
status_led_set(STATUS_LED_BOOTING);
SELFTEST_ASSERT_TRUE(red && !green && !blue);

status_led_set(STATUS_LED_READY);
fake_now_ms = 499U;
status_led_poll();
SELFTEST_ASSERT_TRUE(green);
fake_now_ms = 500U;
status_led_poll();
SELFTEST_ASSERT_TRUE(!green);

status_led_set(STATUS_LED_TESTING);
fake_now_ms += 125U;
status_led_poll();
SELFTEST_ASSERT_TRUE(blue);

status_led_override_begin();
fake_write(BOARD_RGB_RED, true);
fake_now_ms += 500U;
status_led_poll();
SELFTEST_ASSERT_TRUE(red);
status_led_override_end();
SELFTEST_ASSERT_EQ(STATUS_LED_TESTING, status_led_get());
```

Register `selftest_status_led()` in the self-test table and add both new source files to CMake.

- [ ] **Step 2: Build to prove the new test fails before implementation**

Run:

```powershell
& .\ecu\scripts\build_ecu_test.ps1
```

Expected: compilation or link failure because the declared `status_led_*` functions do not yet exist.

- [ ] **Step 3: Implement the minimal state machine**

Implement these invariants in `status_led.c`:

```c
#define READY_HALF_PERIOD_MS   (500U)
#define ACTIVE_HALF_PERIOD_MS  (125U)

static void write_exclusive(uint8_t color, bool on)
{
    s_ops->write(BOARD_RGB_RED, false);
    s_ops->write(BOARD_RGB_GREEN, false);
    s_ops->write(BOARD_RGB_BLUE, false);
    if (on) {
        s_ops->write(color, true);
    }
}
```

- BOOTING writes red continuously.
- READY toggles green every 500 ms, producing a 1 Hz full cycle.
- TESTING toggles blue every 125 ms.
- FAILED toggles red every 125 ms.
- `status_led_set()` resets the phase and immediately renders the new state.
- `status_led_poll()` performs unsigned elapsed-time comparison and never blocks.
- Override nesting is not supported: a second begin is harmless, and end without begin is harmless.
- Polling while overridden performs no GPIO write; ending override immediately re-renders the saved status.
- The default adapter converts `mchtmr_get_count(HPM_MCHTMR)` using `clock_get_frequency(clock_mchtmr0)`, with a zero-frequency guard that returns zero.

- [ ] **Step 4: Build and observe the self-test compilation pass**

Run:

```powershell
& .\ecu\scripts\build_ecu_test.ps1
```

Expected: GNU build exits 0 and emits `tmp/ecu_board_test_build/output/ecu_board_test.elf`.

- [ ] **Step 5: Commit checkpoint 1**

```powershell
git add ecu/apps/ecu_board_test
git commit -m "feat: add nonblocking RGB status state machine"
```

### Task 2: Integrate status indication without blocking the CLI

**Files:**

- Modify: `ecu/apps/ecu_board_test/src/main.c`
- Modify: `ecu/apps/ecu_board_test/src/operator_io.c`
- Modify: `ecu/apps/ecu_board_test/src/app_shell.c`
- Modify: `ecu/apps/ecu_board_test/src/test_runner.c`
- Modify: `ecu/apps/ecu_board_test/tests/test_rgb.c`
- Modify: `ecu/apps/ecu_board_test/selftest/selftest_cli_runner.c`

- [ ] **Step 1: Extend runner self-tests for status restoration and cleanup**

Add assertions around a fake failing descriptor:

```c
status_led_init(&fake_led_ops);
status_led_set(STATUS_LED_READY);
SELFTEST_ASSERT_EQ(TEST_FAIL, test_runner_execute(&descriptor, &context));
SELFTEST_ASSERT_EQ(1U, cleanup_calls);
SELFTEST_ASSERT_EQ(STATUS_LED_READY, status_led_get());
```

Also add a descriptor with a failing `prepare` and assert `execute` and `cleanup` are not called while `safety_all_off()` remains the final hardware action.

- [ ] **Step 2: Build to confirm the integration expectations fail**

Run:

```powershell
& .\ecu\scripts\build_ecu_test.ps1
```

Expected: new integration assertions fail on the target self-test path or compilation fails until runner hooks exist.

- [ ] **Step 3: Add boot and test lifecycle transitions**

Implement the following sequence in `main.c`:

```c
board_init();
status_led_init_default();
status_led_set(STATUS_LED_BOOTING);
/* reset reporting and safety initialization */
if (selftest_run_all() != 0) {
    safety_init(&real_safety_ops);
    status_led_set(STATUS_LED_FAILED);
} else {
    safety_init(&real_safety_ops);
    status_led_set(STATUS_LED_READY);
}
app_shell_run();
```

Do not prevent CLI access after a self-test failure; the red FAILED animation must remain visible until the operator explicitly runs a successful test/session transition.

In `run_descriptor()` set TESTING immediately before `test_runner_execute()`, then select FAILED for `TEST_FAIL` and READY for PASS/SKIP/BLOCKED. After `run all`, set FAILED if the board summary is FAIL or ABORTED; otherwise set READY for PASS/INCOMPLETE.

- [ ] **Step 4: Replace blocking console input with a polling loop**

Change `operator_read_line()` to read `BOARD_CONSOLE_UART_BASE` with `uart_try_receive_byte()`. On each empty iteration call `status_led_poll()` and `board_delay_ms(1U)`. Preserve CR/LF filtering, backspace echo, printable ASCII filtering, capacity checks and the existing bool return contract.

The loop body must follow this form:

```c
uint8_t byte;
status_led_poll();
if (uart_try_receive_byte(BOARD_CONSOLE_UART_BASE, &byte) != status_success) {
    board_delay_ms(1U);
    continue;
}
int c = (int)byte;
```

In `test_runner_poll_abort()`, call `status_led_poll()` before scanning bytes so long-running test wait loops progress the animation.

- [ ] **Step 5: Give `RGB.ALL` explicit override ownership**

Call `status_led_override_begin()` before the first manual color write. Call `status_led_override_end()` from `test_rgb_cleanup()` after turning all colors off. Because `test_runner_execute()` guarantees cleanup after successful prepare/entry, every RGB exit path restores the prior state.

- [ ] **Step 6: Rebuild and check lifecycle symbols**

Run:

```powershell
& .\ecu\scripts\build_ecu_test.ps1
& "${env:GNURISCV_TOOLCHAIN_PATH}\bin\riscv32-unknown-elf-nm.exe" .\tmp\ecu_board_test_build\output\ecu_board_test.elf | Select-String 'status_led_(set|poll|override_begin|override_end)'
```

Expected: build exits 0; all four public lifecycle symbols occur in the ELF.

- [ ] **Step 7: Commit checkpoint 2**

```powershell
git add ecu/apps/ecu_board_test
git commit -m "feat: integrate RGB status with boot shell and tests"
```

### Task 3: Audit all hand-written firmware and add maintainability comments

**Files:**

- Modify as findings require: `ecu/apps/ecu_board_test/include/*.h`
- Modify as findings require: `ecu/apps/ecu_board_test/src/*.c`
- Modify as findings require: `ecu/apps/ecu_board_test/tests/*.c`, `tests/test_serial_common.h`
- Modify as findings require: `ecu/apps/ecu_board_test/selftest/*.c`
- Modify as findings require: `ecu/boards/ecu_isolation/board.c`, `board.h`
- Modify as findings require: `ecu/scripts/*.ps1`, `ecu/scripts/*.jlink`
- Inspect: generated pinmux files, linker files, YAML/CFG, fixtures

- [ ] **Step 1: Run mechanical audit searches**

Run:

```powershell
rg -n "strcpy|sprintf|gets\(|malloc\(|free\(|while\s*\(1\)|for\s*\(;;\)|memcpy|strncpy|\[[0-9]+\]|0x80[0-9A-Fa-f]{6}" ecu/apps/ecu_board_test ecu/boards/ecu_isolation ecu/scripts
rg -n "board_delay_|uart_try_receive|safety_all_off|cleanup|FLASH|EEPROM|timeout_ms|dependency_id" ecu/apps/ecu_board_test
```

For each hit, verify bounds, timeout/abort behavior, integer width, hardware polarity, destructive address range and cleanup path against the approved design and board definitions.

- [ ] **Step 2: Audit framework and pure algorithms file by file**

Review `cli`, `test_types`, `test_registry`, `test_runner`, `safety_manager`, `result_writer`, `operator_io`, `adc_math`, `memory_patterns`, `comm_pattern`, and `sbus_decoder`. Add a module-purpose comment at each hand-written file top. Add public header comments that state units, accepted ranges, blocking behavior, indexing convention, ownership and side effects.

Required concrete checks:

- Reject null output pointers before assignment in numeric parsing and decoder functions.
- Ensure `run_all()` cannot index its 32-element status array beyond capacity; derive capacity with `sizeof` and block excess registry entries.
- Ensure session counters cannot silently wrap for the current registry maximum.
- Ensure parser copies always terminate and reject truncated semantic arguments.
- Ensure SBUS field extraction cannot read beyond the 25-byte frame.
- Ensure abort matching has no stale partial command across independent runs.
- Ensure every prepared descriptor cleanup runs exactly once and every exit calls `safety_all_off()`.

- [ ] **Step 3: Audit destructive and communication tests**

Review every file under `tests/` with focus on:

- QSPI test writes only the reserved `0x807F0000..0x80800000` range and restores or clearly documents destructive test data.
- EEPROM addresses and page boundaries remain within the selected test area.
- ADC raw-to-voltage units and thresholds match `test_limits.h`.
- DIO outputs are off during prepare failure, abort and cleanup.
- CAN classic-frame lengths stay at or below 8 because CAN-FD is out of scope.
- RS232/RS485 loops have timeout and abort polling; RS485 DE behavior matches hardware-DE pinmux.
- SBUS uses 100000 baud, 8E2/inversion assumptions are documented, and frame length is validated.
- Ethernet remains an explicit SKIP because no PHY/link equipment is attached.

Fix only reproducible defects. For each fix, extend the closest `selftest_*.c` case first, observe failure, implement the fix, and rebuild.

- [ ] **Step 4: Audit board support, build scripts and linkers**

Verify one-based external channel indexes, active-low RGB polarity, active-low CAN termination, safe boot output levels, UART mapping, clock return-zero behavior, SDRAM geometry assumptions and ENET reset safety. Check that GNU, IAR and SES linker files all reserve the same top 64 KiB QSPI test sector. Check every PowerShell script for literal paths, exit-code propagation and noninteractive behavior. Add comments only where hardware rationale or toolchain workaround is not self-evident.

- [ ] **Step 5: Run the full GNU build after audit fixes**

Run:

```powershell
& .\ecu\scripts\build_ecu_test.ps1
```

Expected: exit code 0, no new compiler errors, and the ELF/bin/map artifacts are refreshed.

- [ ] **Step 6: Commit checkpoint 3**

```powershell
git add ecu/apps/ecu_board_test ecu/boards/ecu_isolation ecu/scripts
git commit -m "fix: harden and document ECU board test firmware"
```

### Task 4: Publish a project structure and code-reading guide

**Files:**

- Create: `docs/ecu-board-test-code-reading-guide.md`
- Modify: `README.md`
- Modify: `docs/ecu-test-progress.md`

- [ ] **Step 1: Write the guide with the actual final tree**

The guide must include:

- A compact tree separating hand-written, generated, build-output and vendor SDK content.
- Startup call flow: `main -> board_init -> safety_init -> selftest -> app_shell_run`.
- Test call flow: `cli_parse -> registry -> runner prepare/execute/cleanup -> result_writer`.
- RGB state table including color, cadence, meaning and ownership override.
- Recommended reading batches: board definitions; core framework; safety/status; pure algorithms; hardware tests; build/link/download.
- A table for every hand-written source/header/script/linker/config/fixture containing responsibility, entry point, dependencies and reading focus.
- Explicit notes that generated pinmux is regenerated with the HPM Pinmux Tool and that `tmp/` and `sdk_env_v1.11.0/` are not project source.

- [ ] **Step 2: Document exact build and debug commands**

Include these commands with expected artifacts:

```powershell
& .\ecu\scripts\build_ecu_test.ps1
& .\ecu\scripts\generate_ses_ecu_test.ps1
& 'D:\Program Files\SEGGER\SEGGER Embedded Studio 8.28\bin\emBuild.exe' `
  .\tmp\ecu_board_test_build\segger_embedded_studio\ecu_board_test.emProject `
  'ecu_board_test;Debug' -rebuild
& .\ecu\scripts\flash_ecu_test.ps1
```

Explain that no Windows COM port is currently enumerated, so UART self-test output is not yet hardware-verified.

- [ ] **Step 3: Link the guide and update progress memory**

Add a prominent README link to the guide. Record completed code-audit checkpoints, commit IDs, build evidence, J-Link evidence and remaining HIL limitations in `docs/ecu-test-progress.md` so context compaction cannot lose the state.

- [ ] **Step 4: Validate Markdown paths and source coverage**

Run:

```powershell
$sourceFiles = rg --files ecu/apps/ecu_board_test ecu/boards/ecu_isolation ecu/scripts |
    Where-Object { $_ -match '\.(c|h|ps1|jlink|ld|icf|yaml|cfg)$' -and $_ -notmatch 'pinmux\.(c|h)$' }
$guide = Get-Content -Raw -Encoding UTF8 docs/ecu-board-test-code-reading-guide.md
$missing = $sourceFiles | Where-Object { -not $guide.Contains($_.Replace('\','/')) }
$missing
```

Expected: no missing path output.

- [ ] **Step 5: Commit checkpoint 4**

```powershell
git add README.md docs
git commit -m "docs: add ECU firmware code reading guide"
```

### Task 5: Verify GNU, SES, linker placement, J-Link download and visible status

**Files:**

- Modify if verification reveals a reproducible issue: relevant source/script only
- Modify: `docs/ecu-test-progress.md`

- [ ] **Step 1: Run a clean GNU build**

```powershell
Remove-Item -LiteralPath .\tmp\ecu_board_test_build -Recurse -Force
& .\ecu\scripts\build_ecu_test.ps1
```

Expected: exit code 0 and fresh ELF/bin/map output.

- [ ] **Step 2: Regenerate and build the SES 8.28 project**

```powershell
$project = & .\ecu\scripts\generate_ses_ecu_test.ps1
& 'D:\Program Files\SEGGER\SEGGER Embedded Studio 8.28\bin\emBuild.exe' $project 'ecu_board_test;Debug' -rebuild
```

Expected: XML validation passes and `emBuild` reports Build Complete without errors.

- [ ] **Step 3: Verify reserved flash boundaries and symbols**

Inspect GNU map/objdump and SES map. No loadable application section may overlap `0x807F0000..0x80800000`. Confirm `status_led_*`, `selftest_status_led`, and test registry symbols are linked.

- [ ] **Step 4: Download with J-Link and read the connected target identity**

```powershell
& .\ecu\scripts\flash_ecu_test.ps1
& 'D:\Program Files\SEGGER\JLink\JLink.exe' -CommandFile .\ecu\scripts\read_ecu_info.jlink
```

Expected: download/verify succeeds, JTAG RV32 TAP ID remains `0x1000563D`, and target identity remains consistent with the previously read HPM6750 board.

- [ ] **Step 5: Observe the physical RGB sequence**

Power-cycle or reset the ECU. Expected visible behavior:

- Red steady immediately after entering application startup.
- Green 1 Hz heartbeat after all target self-tests pass.
- Red fast blink if a target self-test fails.
- Blue fast blink during a launched board test.
- `RGB.ALL` displays manual red/green/blue and then returns to the prior status animation.

Record observation as PASS only when physically seen. If remote observation is unavailable, record `NOT OBSERVED`; do not infer it from successful download.

- [ ] **Step 6: Record final evidence and commit checkpoint 5**

Update `docs/ecu-test-progress.md` with exact command outputs, tool versions and unresolved UART/HIL items, then run:

```powershell
git diff --check
git status --short
git add docs/ecu-test-progress.md
git commit -m "test: record RGB firmware verification evidence"
```

Expected: `git diff --check` produces no output and the final worktree is clean after the commit.

