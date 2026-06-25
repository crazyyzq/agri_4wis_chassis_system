# ECU Board-Test Function Comments Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add clear English function contracts to every hand-written ECU board-test API and intent comments to every nontrivial implementation helper without changing executable behavior.

**Architecture:** Public headers are the authoritative contract and use Doxygen-compatible comments. Source files do not duplicate those contracts; they document internal helpers, state transitions, hardware rationale, destructive operations, and cleanup rules. Generated pinmux files remain untouched.

**Tech Stack:** C11, Doxygen-compatible comments, HPM SDK 1.11.0, GNU RISC-V toolchain, SEGGER Embedded Studio 8.28, PowerShell.

---

## Comment format

Use this complete public-function pattern, omitting only tags that do not apply:

```c
/**
 * @brief Convert a 16-bit ADC result to connector voltage.
 *
 * @param raw          ADC code in the inclusive range 0..65535.
 * @param reference_uv ADC reference voltage in microvolts.
 * @return Estimated connector voltage in microvolts.
 *
 * @note Uses the board's 100 kOhm / 56 kOhm divider ratio and 64-bit
 *       intermediate arithmetic.
 */
uint32_t adc_external_uv_from_raw(uint16_t raw, uint32_t reference_uv);
```

Use this internal-helper pattern:

```c
/**
 * @brief Wait for one UART byte while keeping abort and RGB polling active.
 *
 * @return true when a byte was received before the 2 s timeout; false on
 *         timeout, invalid arguments, or an operator abort.
 */
static bool receive_byte_timeout(...)
```

### Task 1: Document pure algorithms and shared data-model APIs

**Files:**

- Modify: `ecu/apps/ecu_board_test/include/adc_math.h`
- Modify: `ecu/apps/ecu_board_test/include/cli.h`
- Modify: `ecu/apps/ecu_board_test/include/comm_pattern.h`
- Modify: `ecu/apps/ecu_board_test/include/memory_patterns.h`
- Modify: `ecu/apps/ecu_board_test/include/sbus_decoder.h`
- Modify: `ecu/apps/ecu_board_test/include/result_writer.h`
- Modify: `ecu/apps/ecu_board_test/include/test_types.h`
- Modify: corresponding `.c` files under `src/`

- [ ] **Step 1: Replace terse header comments with complete contracts**

For each declaration, state exact units, null handling, buffer ownership and return semantics. Required details:

- ADC functions use microvolts and the 100k/56k divider.
- CLI parsing accepts at most `CLI_LINE_MAX` bytes and clears `command` before parsing.
- communication encoding returns zero on invalid arguments/capacity; decoding requires exact frame length and CRC.
- memory patterns overwrite every supplied word and report the first mismatch.
- SBUS requires exactly 25 bytes and expands 16 packed 11-bit channels.
- result JSON returns `0x0104` for argument/capacity failure and always produces one line on success.
- a session can report PASS only when `expected_required_count` is nonzero and exactly matched.

- [ ] **Step 2: Document nontrivial internal helpers without duplicating public contracts**

Add intent comments above `skip_space`, `pattern_value`, `append_escaped`, and any compact CRC/packing logic. Explain why 64-bit intermediate arithmetic, strict length equality, and JSON control-character replacement are required.

- [ ] **Step 3: Verify this batch is comment-only**

Run:

```powershell
git diff --word-diff=porcelain -- ecu/apps/ecu_board_test/include `
  ecu/apps/ecu_board_test/src/adc_math.c `
  ecu/apps/ecu_board_test/src/cli.c `
  ecu/apps/ecu_board_test/src/comm_pattern.c `
  ecu/apps/ecu_board_test/src/memory_patterns.c `
  ecu/apps/ecu_board_test/src/sbus_decoder.c `
  ecu/apps/ecu_board_test/src/result_writer.c `
  ecu/apps/ecu_board_test/src/test_types.c
```

Expected: added/removed words occur only inside comments; declarations and executable tokens are unchanged.

- [ ] **Step 4: Commit batch 1**

```powershell
git add ecu/apps/ecu_board_test/include ecu/apps/ecu_board_test/src
git commit -m "docs: clarify algorithm and result API contracts"
```

### Task 2: Document shell, runner, safety, status, registry and self-test APIs

**Files:**

- Modify: `ecu/apps/ecu_board_test/include/app_shell.h`
- Modify: `ecu/apps/ecu_board_test/include/operator_io.h`
- Modify: `ecu/apps/ecu_board_test/include/safety_manager.h`
- Modify: `ecu/apps/ecu_board_test/include/selftest.h`
- Modify: `ecu/apps/ecu_board_test/include/status_led.h`
- Modify: `ecu/apps/ecu_board_test/include/test_registry.h`
- Modify: `ecu/apps/ecu_board_test/include/test_runner.h`
- Modify: `ecu/apps/ecu_board_test/src/app_shell.c`
- Modify: `ecu/apps/ecu_board_test/src/operator_io.c`
- Modify: `ecu/apps/ecu_board_test/src/safety_manager.c`
- Modify: `ecu/apps/ecu_board_test/src/status_led.c`
- Modify: `ecu/apps/ecu_board_test/src/test_registry.c`
- Modify: `ecu/apps/ecu_board_test/src/test_runner.c`
- Modify: `ecu/apps/ecu_board_test/src/main.c`
- Modify: `ecu/apps/ecu_board_test/selftest/*.c`

- [ ] **Step 1: Document all public framework contracts**

Required statements:

- `app_shell_run()` never returns and services RGB while waiting for UART input.
- `operator_read_line()` blocks the caller, ignores empty lines, and reserves one byte for NUL.
- Safety Manager indexes are one-based; invalid indexes are rejected; `safety_all_off()` is idempotent.
- `safety_backend()` returns a borrowed pointer used only for scoped self-test restoration.
- status states, half-periods, MCHTMR timing, invalid-state fallback and override behavior are explicit.
- registry storage is immutable and statically owned.
- runner cleanup and `safety_all_off()` guarantees are explicit, including prepare-failure behavior.
- abort recognition is exact lowercase `abort`, foreground-polled and scoped to one context.
- each self-test function returns true only when every assertion in its module passes.

- [ ] **Step 2: Document internal state transitions and restoration rules**

Add comments above `run_descriptor`, `run_all`, `render`, `write_exclusive`, the default time adapter, fake self-test backends and `main()`. Explain expected required-count initialization, dependency blocking, fake backend restoration, active-low exclusive writes, unsigned timestamp wrap and why CLI remains available after startup self-test failure.

- [ ] **Step 3: Audit every framework definition against its header comment**

Run:

```powershell
rg -n "^(static )?[A-Za-z_][A-Za-z0-9_ *]*\([^;]*\)$" `
  ecu/apps/ecu_board_test/src ecu/apps/ecu_board_test/selftest
```

Inspect every listed function. Expected: public behavior is documented in its header, and every nontrivial static helper has a nearby intent comment.

- [ ] **Step 4: Commit batch 2**

```powershell
git add ecu/apps/ecu_board_test/include ecu/apps/ecu_board_test/src `
  ecu/apps/ecu_board_test/selftest
git commit -m "docs: clarify shell safety and status function contracts"
```

### Task 3: Document every hardware test function

**Files:**

- Modify: `ecu/apps/ecu_board_test/include/test_cases.h`
- Modify: `ecu/apps/ecu_board_test/include/test_limits.h`
- Modify: `ecu/apps/ecu_board_test/tests/test_adc.c`
- Modify: `ecu/apps/ecu_board_test/tests/test_boot.c`
- Modify: `ecu/apps/ecu_board_test/tests/test_can.c`
- Modify: `ecu/apps/ecu_board_test/tests/test_digital_input.c`
- Modify: `ecu/apps/ecu_board_test/tests/test_digital_output.c`
- Modify: `ecu/apps/ecu_board_test/tests/test_dio_loopback.c`
- Modify: `ecu/apps/ecu_board_test/tests/test_ethernet.c`
- Modify: `ecu/apps/ecu_board_test/tests/test_memory.c`
- Modify: `ecu/apps/ecu_board_test/tests/test_power.c`
- Modify: `ecu/apps/ecu_board_test/tests/test_rgb.c`
- Modify: `ecu/apps/ecu_board_test/tests/test_rs232.c`
- Modify: `ecu/apps/ecu_board_test/tests/test_rs485.c`
- Modify: `ecu/apps/ecu_board_test/tests/test_sbus.c`
- Modify: `ecu/apps/ecu_board_test/tests/test_serial_common.c`
- Modify: `ecu/apps/ecu_board_test/tests/test_serial_common.h`

- [ ] **Step 1: Add one complete Doxygen contract per registered test and cleanup**

Each contract identifies prerequisites, physical units, PASS/FAIL/BLOCKED/SKIP meaning, touched resources, and cleanup ownership. Required safety facts:

- DO uses a 220 Ohm / 2 W load and 500 ms peak-hold pulse.
- QSPI may erase/program only `0x807F0000..0x80800000` and leaves it erased.
- EEPROM saves and restores 16 bytes at `0xF0..0xFF`.
- SDRAM test is destructive over 32 MiB.
- CAN is 500 kbit/s Classic CAN with DLC 0 or 8; no CAN-FD.
- serial echo is 115200 8N1, with hardware DE for RS485.
- SBUS is 100000 8E2 and requires 50 ms verified silence at stop.
- Ethernet deliberately returns SKIP because no fixture is connected.

- [ ] **Step 2: Document internal hardware helpers**

Add comments above ADC averaging, digital-input stability sampling, CAN receive timeout, RAM pattern batches, UART receive timeout, RGB all-off, and SBUS silence detection. State iteration counts, timeout units and abort polling behavior.

- [ ] **Step 3: Check cleanup documentation against registry entries**

Run:

```powershell
Get-Content -Encoding UTF8 ecu/apps/ecu_board_test/src/test_registry.c
rg -n "cleanup|test_.*cleanup|safety_all_off|override_end" `
  ecu/apps/ecu_board_test/include/test_cases.h ecu/apps/ecu_board_test/tests
```

Expected: every non-null cleanup in the registry has a documented declaration and definition; comments do not promise cleanup for descriptors with no cleanup callback.

- [ ] **Step 4: Commit batch 3**

```powershell
git add ecu/apps/ecu_board_test/include/test_cases.h `
  ecu/apps/ecu_board_test/include/test_limits.h ecu/apps/ecu_board_test/tests
git commit -m "docs: clarify hardware test function contracts"
```

### Task 4: Document board-support functions and hardware polarity

**Files:**

- Modify: `ecu/boards/ecu_isolation/board.h`
- Modify: `ecu/boards/ecu_isolation/board.c`
- Inspect only: `ecu/boards/ecu_isolation/pinmux.c`
- Inspect only: `ecu/boards/ecu_isolation/pinmux.h`

- [ ] **Step 1: Add Doxygen comments for every board API declaration**

Cover initialization order, supported peripheral instances, frequency return values, units, one-based indexes, invalid-index behavior, active-low RGB/CAN termination, raw-versus-logical LED APIs, SDRAM assumptions, ENET reset/IRQ behavior and fatal initialization loops.

- [ ] **Step 2: Document board-local helpers and non-obvious implementation choices**

Explain why GPIO output latches are programmed before output-enable, why safe GPIO levels are re-applied after generated pinmux, why UART/ADC/CAN clock helpers return zero for unsupported instances, and why PHY reset remains asserted after pin initialization.

- [ ] **Step 3: Confirm generated files were not changed**

Run:

```powershell
git diff --exit-code HEAD -- ecu/boards/ecu_isolation/pinmux.c `
  ecu/boards/ecu_isolation/pinmux.h
```

Expected: no output and exit code 0.

- [ ] **Step 4: Commit batch 4**

```powershell
git add ecu/boards/ecu_isolation/board.h ecu/boards/ecu_isolation/board.c
git commit -m "docs: clarify board support function contracts"
```

### Task 5: Coverage audit and build verification

**Files:**

- Modify: `docs/ecu-test-progress.md`

- [ ] **Step 1: Audit public declaration comment coverage**

For each declaration returned by:

```powershell
rg -n "^[A-Za-z_][A-Za-z0-9_ *]*\([^;{}]*\);$" `
  ecu/apps/ecu_board_test/include ecu/apps/ecu_board_test/tests/test_serial_common.h `
  ecu/boards/ecu_isolation/board.h
```

confirm the immediately preceding block begins with `/**` and contains `@brief`. Functions with parameters contain matching `@param` names; non-void functions contain `@return`.

- [ ] **Step 2: Confirm source changes are comments only**

Run a token-oriented review:

```powershell
git diff 259c2c4..HEAD --word-diff=porcelain -- `
  'ecu/**/*.c' 'ecu/**/*.h'
```

Expected: no API signatures, constants, statements or control flow changed.

- [ ] **Step 3: Run formatting and both builds**

```powershell
git diff --check
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File .\ecu\scripts\build_ecu_test.ps1
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File .\ecu\scripts\generate_ses_ecu_test.ps1
& 'D:\Program Files\SEGGER\SEGGER Embedded Studio 8.28\bin\emBuild.exe' `
  -config Debug -rebuild `
  .\tmp\ecu_board_test_build\segger_embedded_studio\ecu_board_test.emProject
```

Expected: diff check has no errors; GNU and SES builds exit 0; firmware size remains behaviorally unchanged from the pre-comment build except debug metadata.

- [ ] **Step 4: Record and commit verification**

Add the comment coverage count, comment-only review result and both build results to `docs/ecu-test-progress.md`, then run:

```powershell
git add docs/ecu-test-progress.md
git commit -m "docs: record function comment verification"
```

- [ ] **Step 5: Push the existing PR branch**

```powershell
git push origin codex/ecu-board-test
```

Expected: remote branch advances and draft PR #1 includes all comment commits.
