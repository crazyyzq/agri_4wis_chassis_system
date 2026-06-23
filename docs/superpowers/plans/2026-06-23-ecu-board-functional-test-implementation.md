# ECU Board Functional Test Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a reusable HPM6750 ECU board-test firmware that safely validates every approved onboard interface and emits traceable results for multiple prototype boards.

**Architecture:** A bare-metal Test Runner owns isolated test modules and executes `prepare → execute → cleanup`. A Safety Manager exclusively controls digital outputs, CAN termination, and RS485 DE; a UART CLI emits human-readable and JSONL results.

**Tech Stack:** C11, HPM SDK 1.11.0, HPM6750, CMake/Ninja, SEGGER Embedded Studio 8.28, J-Link, PowerShell, Classic CAN, UART, SBUS, ADC16, I2C, FEMC, ROM XPI NOR API.

---

## Delivery milestones

1. Foundation: layout, build tooling, result model, CLI, runner, Safety Manager.
2. Core resources: boot/reset, RGB, RAM, SDRAM, QSPI Flash, EEPROM.
3. I/O: ADC, DI, DO, loopback fixture.
4. Communications: Classic CAN, RS485, RS232, SBUS.
5. HIL release: full sequence, documentation, reports and release verification.

The workspace is not a Git repository. Commit commands below require explicit user authorization to initialize Git. Without it, update `docs/ecu-test-progress.md` at each checkpoint and skip only the commit command.

## Locked file map

- `ecu/boards/ecu_isolation/`: board, pinmux, YAML and debugger configuration.
- `ecu/apps/ecu_board_test/src/main.c`: boot order and event loop only.
- `ecu/apps/ecu_board_test/include/test_types.h`: stable result and descriptor types.
- `ecu/apps/ecu_board_test/src/test_runner.c`: registry, lifecycle and aggregation.
- `ecu/apps/ecu_board_test/src/safety_manager.c`: dangerous-resource ownership.
- `ecu/apps/ecu_board_test/src/cli.c`: command parser and line input.
- `ecu/apps/ecu_board_test/src/result_writer.c`: text and JSONL output.
- `ecu/apps/ecu_board_test/include/test_limits.h`: all numeric limits.
- `ecu/apps/ecu_board_test/tests/test_*.c`: one hardware subsystem per file.
- `ecu/apps/ecu_board_test/selftest/selftest_*.c`: target-side pure-logic tests.
- `ecu/scripts/`: build, SES generation, J-Link and serial logging tools.
- `test/fixtures/`: wiring and operator procedure.

## Milestone 1 — Foundation

### Task 1: Create the standalone board and application layout

**Files:**
- Move: `ecu/ecu_isolation/*` → `ecu/boards/ecu_isolation/*`
- Create from template: `ecu/boards/ecu_isolation/ecu_isolation.cfg`
- Create: `ecu/apps/ecu_board_test/CMakeLists.txt`
- Create: `ecu/apps/ecu_board_test/app.yaml`
- Create: `ecu/apps/ecu_board_test/src/main.c`
- Copy/modify: template linker files → `ecu/apps/ecu_board_test/linkers/*/ecu_board_test_linker.*`

- [ ] **Step 1: Move the board without regenerating pinmux**

Use `Move-Item -LiteralPath`; verify all seven existing files under the new folder and no source file remains in `ecu/ecu_isolation`.

Copy `ecu/sdk_env_v1.11.0/user_template/user_board/user_board.cfg` to `ecu/boards/ecu_isolation/ecu_isolation.cfg`. Keep it comment-only, but correct its documented Flash size from `0x2000000` to `0x800000` so future activation cannot silently assume 32 MiB.

- [ ] **Step 2: Add a minimal standalone application**

```cmake
cmake_minimum_required(VERSION 3.13)
set(CUSTOM_GCC_LINKER_FILE ${CMAKE_CURRENT_SOURCE_DIR}/linkers/gcc/ecu_board_test_linker.ld)
set(CUSTOM_SES_LINKER_FILE ${CMAKE_CURRENT_SOURCE_DIR}/linkers/segger/ecu_board_test_linker.icf)
set(CUSTOM_IAR_LINKER_FILE ${CMAKE_CURRENT_SOURCE_DIR}/linkers/iar/ecu_board_test_linker.icf)
find_package(hpm-sdk REQUIRED HINTS $ENV{HPM_SDK_BASE})
project(ecu_board_test)
sdk_app_inc(include)
sdk_app_src(src/main.c)
generate_ide_projects()
```

```yaml
dependency:
  - board_gpio_led
```

```c
#include <stdio.h>
#include "board.h"
int main(void)
{
    board_init();
    printf("ECU_TEST READY board=%s sdk=1.11.0\n", BOARD_NAME);
    while (1) { __asm volatile("wfi"); }
}
```

- [ ] **Step 3: Reserve the final 64 KiB of XPI0**

GNU linker definition:

```ld
ECU_TEST_FLASH_SIZE = 64K;
XPI0 (rx) : ORIGIN = 0x80000000, LENGTH = _flash_size - ECU_TEST_FLASH_SIZE
ECU_TEST_FLASH (rx) : ORIGIN = 0x80000000 + _flash_size - ECU_TEST_FLASH_SIZE, LENGTH = ECU_TEST_FLASH_SIZE
__ecu_test_flash_start__ = ORIGIN(ECU_TEST_FLASH);
__ecu_test_flash_end__ = ORIGIN(ECU_TEST_FLASH) + LENGTH(ECU_TEST_FLASH);
```

Add equivalent regions to SES and IAR linker files. For 8 MiB, symbols must resolve to `0x807F0000` and `0x80800000`.

- [ ] **Step 4: Build the standalone skeleton**

Run CMake with `BOARD=ecu_isolation`, `BOARD_SEARCH_PATH=<workspace>/ecu/boards`, and `CMAKE_BUILD_TYPE=flash_sdram_xip`. Expected: `demo.elf` links with no reserved-region overlap.

- [ ] **Step 5: Commit**

```powershell
git add ecu/boards ecu/apps/ecu_board_test
git commit -m "build: create standalone ECU board-test application"
```

### Task 2: Add reproducible build, SES, flash and serial scripts

**Files:**
- Create: `ecu/scripts/build_ecu_test.ps1`
- Create: `ecu/scripts/generate_ses_ecu_test.ps1`
- Create: `ecu/scripts/flash_ecu_test.ps1`
- Create: `ecu/scripts/load_ecu_test.jlink`
- Create: `ecu/scripts/serial_console.ps1`

- [ ] **Step 1: Write the build script**

Resolve every path from `$PSScriptRoot`; set the bundled SDK/toolchain PATH; use `tmp/ecu_board_test_build`.

```powershell
cmake --fresh -GNinja -S $app -B $build `
  '-DBOARD=ecu_isolation' "-DBOARD_SEARCH_PATH=$boards" `
  '-DCMAKE_BUILD_TYPE=flash_sdram_xip'
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
cmake --build $build -j 4
exit $LASTEXITCODE
```

- [ ] **Step 2: Write the SES wrapper**

Call the same configuration and fail unless `tmp/ecu_board_test_build/segger_embedded_studio/ecu_board_test.emProject` exists.

- [ ] **Step 3: Write J-Link loading**

`load_ecu_test.jlink`:

```text
r
h
loadfile D:\agri_4wis_chassis_system\tmp\ecu_board_test_build\output\demo.elf
r
g
exit
```

Wrapper command:

```powershell
& 'C:\Program Files\SEGGER\JLink_V916\JLink.exe' `
  -device HPM6750xVMx -if JTAG -speed 4000 -CommanderScript $commandFile
exit $LASTEXITCODE
```

- [ ] **Step 4: Write serial capture using `System.IO.Ports.SerialPort`**

Require `-Port COMx -LogPath path`; configure 115200/8N1 and `ReadTimeout=100`; mirror received UTF-8 bytes to screen and log, forward keyboard input, and close cleanly on Ctrl-C.

- [ ] **Step 5: Verify tooling**

Run build, SES generation, flash, and serial capture. Expected: exit 0, generated project path, successful J-Link load, and `ECU_TEST READY` in the log.

- [ ] **Step 6: Commit**

```powershell
git add ecu/scripts
git commit -m "build: add reproducible ECU test tooling"
```

### Task 3: Add result types and a target self-test harness

**Files:**
- Create: `include/test_types.h`, `include/selftest.h`
- Create: `src/test_types.c`, `selftest/selftest.c`, `selftest/selftest_result.c`
- Modify: application `CMakeLists.txt`, `src/main.c`

- [ ] **Step 1: Write failing aggregation tests**

```c
SELFTEST_CASE(result_all_required_passes)
{
    test_session_t s;
    test_session_init(&s, "ECU-SELFTEST");
    test_session_add(&s, TEST_REQUIRED, TEST_PASS);
    test_session_add(&s, TEST_OPTIONAL, TEST_SKIP);
    SELFTEST_ASSERT_EQ(TEST_BOARD_PASS, test_session_status(&s));
}
SELFTEST_CASE(result_blocked_is_incomplete)
{
    test_session_t s;
    test_session_init(&s, "ECU-SELFTEST");
    test_session_add(&s, TEST_REQUIRED, TEST_BLOCKED);
    SELFTEST_ASSERT_EQ(TEST_BOARD_INCOMPLETE, test_session_status(&s));
}
```

- [ ] **Step 2: Build and verify the test fails**

Expected: compile failure because the result types/functions do not exist.

- [ ] **Step 3: Implement exact public types**

```c
typedef enum { TEST_PASS, TEST_FAIL, TEST_SKIP, TEST_BLOCKED } test_status_t;
typedef enum { TEST_REQUIRED, TEST_OPTIONAL } test_requirement_t;
typedef enum { TEST_BOARD_PASS, TEST_BOARD_FAIL, TEST_BOARD_INCOMPLETE, TEST_BOARD_ABORTED } test_board_status_t;
typedef struct {
    char board_serial[24];
    uint16_t pass_count, fail_count, skip_count, blocked_count;
    bool aborted;
} test_session_t;
void test_session_init(test_session_t *, const char *serial);
void test_session_add(test_session_t *, test_requirement_t, test_status_t);
test_board_status_t test_session_status(const test_session_t *);
```

Use bounded serial copying and the approved aggregate rules.

- [ ] **Step 4: Run target self-tests**

Expected: two PASS lines and `SELFTEST SUMMARY pass=2 fail=0`.

- [ ] **Step 5: Commit**

```powershell
git add ecu/apps/ecu_board_test
git commit -m "test: add ECU result model and self-test harness"
```

### Task 4: Implement the Safety Manager

**Files:**
- Create: `include/safety_manager.h`, `src/safety_manager.c`
- Create: `selftest/selftest_safety.c`, fake hardware backend
- Modify: `src/main.c`

- [ ] **Step 1: Write failing state tests**

```c
SELFTEST_CASE(safety_boots_all_off)
{
    fake_hw_set_all_unsafe();
    safety_init(&fake_hw_ops);
    SELFTEST_ASSERT_TRUE(fake_hw_all_outputs_off());
    SELFTEST_ASSERT_TRUE(fake_hw_all_can_terms_off());
    SELFTEST_ASSERT_TRUE(fake_hw_all_rs485_rx());
}
SELFTEST_CASE(safety_rejects_two_outputs)
{
    safety_init(&fake_hw_ops);
    SELFTEST_ASSERT_EQ(SAFETY_OK, safety_output_on(1));
    SELFTEST_ASSERT_EQ(SAFETY_BUSY, safety_output_on(2));
    safety_all_off();
}
```

- [ ] **Step 2: Build and confirm undefined-symbol failure**

- [ ] **Step 3: Implement the public API**

```c
void safety_init(const safety_hw_ops_t *ops);
safety_status_t safety_output_on(uint8_t index);
void safety_output_off(uint8_t index);
void safety_can_term_set(uint8_t index, bool enable);
void safety_rs485_receive(uint8_t index);
void safety_all_off(void);
safety_snapshot_t safety_snapshot(void);
```

Reject invalid one-based indexes. Track exactly one active DO. `safety_all_off()` iterates all DO, CAN termination, and RS485 channels and is idempotent.

- [ ] **Step 4: Integrate the real backend immediately after `board_init()`**

Measure that all DO and CAN termination controls are off before the CLI starts.

- [ ] **Step 5: Run self-tests and commit**

```powershell
git add ecu/apps/ecu_board_test
git commit -m "feat: enforce safe ownership of ECU outputs"
```

### Task 5: Implement CLI, runner, registry and JSONL

**Files:**
- Create: `include/test_runner.h`, `src/test_runner.c`
- Create: `include/cli.h`, `src/cli.c`
- Create: `include/result_writer.h`, `src/result_writer.c`
- Create: `selftest/selftest_runner.c`, `selftest/selftest_cli.c`

- [ ] **Step 1: Write failing parser/lifecycle tests**

```c
SELFTEST_ASSERT_EQ(CLI_LIST, cli_parse("list", &cmd));
SELFTEST_ASSERT_EQ(CLI_RUN_ONE, cli_parse("run ADC.EX1", &cmd));
SELFTEST_ASSERT_EQ(CLI_RUN_ALL, cli_parse("run all", &cmd));
SELFTEST_ASSERT_EQ(CLI_BOARD, cli_parse("board ECU-001", &cmd));
SELFTEST_ASSERT_EQ(CLI_INVALID, cli_parse("run", &cmd));
```

Use a fake test whose execute fails; assert cleanup is called exactly once.

- [ ] **Step 2: Build and verify undefined-symbol failure**

- [ ] **Step 3: Implement a fixed-size parser**

Maximum line 96 bytes, no heap, backspace support. UART RX interrupt recognizes `abort` and sets a volatile flag.

- [ ] **Step 4: Implement descriptor lifecycle**

```c
typedef struct test_descriptor {
    const char *id;
    test_requirement_t requirement;
    const char *dependency_id;
    uint32_t timeout_ms;
    test_status_t (*prepare)(test_context_t *);
    test_status_t (*execute)(test_context_t *);
    void (*cleanup)(test_context_t *);
} test_descriptor_t;
```

Execute `dependency → prepare → execute → cleanup → record`. After entering prepare successfully, cleanup runs once for pass, fail, timeout and abort.

- [ ] **Step 5: Implement bounded JSONL**

Serialize into 384 bytes with `snprintf`; escape quotes/backslashes; reject truncation with `0x0104`.

- [ ] **Step 6: Verify CLI**

Run `list`, `board ECU-001`, `run all`, `abort`, `status`, and `report`. Expected: no crash and an empty suite reports INCOMPLETE before hardware tests are registered.

- [ ] **Step 7: Commit**

```powershell
git add ecu/apps/ecu_board_test
git commit -m "feat: add ECU test CLI runner and results"
```

## Milestone 2 — Core board resources

### Task 6: Add limits, boot, reset, power and RGB tests

**Files:**
- Create: `include/test_limits.h`
- Create: `tests/test_boot.c`, `tests/test_power.c`, `tests/test_rgb.c`
- Create: `selftest/selftest_limits.c`

- [ ] **Step 1: Write failing limit tests**

Use integer millivolts. Verify 5 V accepts 4750/5250 and rejects 4749/5251; 3.3 V accepts 3135/3465 and rejects 3134/3466.

- [ ] **Step 2: Implement central limits**

```c
#define TEST_5V_MIN_MV 4750U
#define TEST_5V_MAX_MV 5250U
#define TEST_3V3_MIN_MV 3135U
#define TEST_3V3_MAX_MV 3465U
#define TEST_BOOT_LOG_DEADLINE_MS 3000U
#define TEST_DO_MAX_ON_MS 500U
#define TEST_COMM_FRAME_COUNT 1000U
```

- [ ] **Step 3: Implement BOOT/SAFE cases**

Read reset status before clearing it. Register external, software and watchdog reset cases. Each case verifies `safety_snapshot()` after restart.

- [ ] **Step 4: Implement manual PWR and RGB cases**

Prompt for five rail measurements in millivolts and validate limits. Cycle red/green/blue, require explicit `y/n`, and turn every LED off in cleanup.

- [ ] **Step 5: Verify and commit**

Expected: boundary self-tests PASS; each rail produces JSONL; RGB cleanup leaves all LEDs off.

```powershell
git add ecu/apps/ecu_board_test
git commit -m "feat: test ECU power boot reset and RGB"
```

### Task 7: Implement RAM, SDRAM, Flash and EEPROM

**Files:**
- Create: `tests/test_memory.c`
- Create: `src/memory_patterns.c`, `include/memory_patterns.h`
- Create: `selftest/selftest_memory_patterns.c`

- [ ] **Step 1: Write failing pattern tests**

On a fake 256-word region verify `0`, `FFFFFFFF`, `AAAAAAAA`, `55555555`, address and walking-one patterns. Corrupt one word and assert returned address, expected, and actual.

- [ ] **Step 2: Implement volatile pattern access**

```c
typedef struct {
    uintptr_t address;
    uint32_t expected;
    uint32_t actual;
} memory_mismatch_t;
```

Return the first mismatch; use no heap.

- [ ] **Step 3: Test target RAM and SDRAM**

Use linker-provided unused regions only. Run SDRAM in chunks, servicing abort and watchdog between chunks; execute address-line alias check before full patterns.

- [ ] **Step 4: Test QSPI Flash from the SDK ROM API reference**

Use `samples/rom_api/xpi_nor_api/src/xpi_nor_api_demo.c` and `xpi_util.c`. Validate 8 MiB geometry, linker symbols, JEDEC ID, and firmware CRC. Run erase/program from RAM on `0x807F0000..0x80800000`; leave it erased. Geometry mismatch returns BLOCKED before erase.

- [ ] **Step 5: Test AT24C02 with power-cycle recovery**

Use `samples/drivers/i2c/polling/master/src/i2c.c`. Back up EEPROM `0xF0..0xFF` plus a recovery marker into the reserved QSPI test region, then write and verify the EEPROM pattern. After the requested power cycle, boot code detects the marker, verifies retention, restores the original EEPROM bytes, verifies restoration, and erases the QSPI marker. An interrupted recovery repeats safely on the next boot.

- [ ] **Step 6: Verify and commit**

Expected: exact byte counts; injected mismatch reports address; abort during SDRAM returns safely to CLI.

```powershell
git add ecu/apps/ecu_board_test
git commit -m "feat: add ECU memory tests"
```

## Milestone 3 — Analog and digital I/O

### Task 8: Implement four-channel ADC validation

**Files:**
- Create: `tests/test_adc.c`
- Create: `src/adc_math.c`, `include/adc_math.h`
- Create: `selftest/selftest_adc_math.c`

- [ ] **Step 1: Write failing conversion/tolerance tests**

Use integer microvolts for external 0, 0.5, 2.5 and 5.0 V. Verify the 100/156 divider and ±2% full-scale acceptance boundaries.

- [ ] **Step 2: Implement deterministic conversion**

```c
uint32_t adc_pin_uv_from_external_uv(uint32_t external_uv)
{
    return (uint32_t)(((uint64_t)external_uv * 100U) / 156U);
}
```

Use a 64-bit intermediate for raw-code voltage conversion.

- [ ] **Step 3: Implement ADC3 sampling**

Base setup on `samples/drivers/adc/adc16/src/adc16.c`. Map EX1/CH6, EX2/CH4, EX3/CH5, EX4/CH7. Report mean, min, max and peak-to-peak.

- [ ] **Step 4: Run HIL points**

Apply each voltage only after CLI prompt. Expected: all four channels within ±2% external full scale and strictly monotonic.

- [ ] **Step 5: Commit**

```powershell
git add ecu/apps/ecu_board_test
git commit -m "feat: add ECU analog input test"
```

### Task 9: Implement DI, DO and loopback

**Files:**
- Create: `tests/test_digital_input.c`, `tests/test_digital_output.c`, `tests/test_dio_loopback.c`
- Create: `selftest/selftest_dio_sequence.c`
- Create: `test/fixtures/ecu-board-test-wiring.md`

- [ ] **Step 1: Write failing exclusive-output sequence tests**

With fake operations verify 12 indexes run OFF→ON→OFF, a second active output is rejected, invalid indexes fail, and abort forces all off.

- [ ] **Step 2: Implement DI test**

For each input prompt for 0 V and 12 V, sample repeatedly, require stable inactive/active states, and report logical index plus MCU pin on failure.

- [ ] **Step 3: Implement DO load test**

Require fixture confirmation, enable one output for ≤500 ms, accept measured ON/OFF millivolts, require ON ≤500 mV, and turn off before advancing.

- [ ] **Step 4: Implement DIO loopback**

Drive DO N, require DI N asserted, release DO N, require DI N clear. Any mismatch calls `safety_all_off()` before recording failure.

- [ ] **Step 5: Document exact wiring**

Transcribe connector pins for EX_OUT1..12 and EX_IN1±..12± from schematic page 13. Document DI+ to 12 V, DI− to matching EX_OUT, and separate 220 Ω/2 W load.

- [ ] **Step 6: Verify and commit**

Expected: 12 results, no overlapping activation, all outputs off after pass/fail/abort.

```powershell
git add ecu/apps/ecu_board_test test/fixtures
git commit -m "feat: add isolated ECU digital IO tests"
```

## Milestone 4 — Communications

### Task 10: Implement Classic CAN and termination tests

**Files:** Create `tests/test_can.c`, `src/comm_pattern.c`, `include/comm_pattern.h`, `selftest/selftest_comm_pattern.c`.

- [ ] **Step 1: Write failing frame-pattern tests**

Generate standard/extended IDs, DLC 0/8, and zero, all-ones, alternating and incrementing payloads. Verify deterministic sequence/check values.

- [ ] **Step 2: Implement Classic CAN only**

Use `samples/drivers/can/src/can_demo.c`. Configure 500 kbit/s from the 80 MHz CAN clock. Do not enable CAN-FD.

- [ ] **Step 3: Implement external-adapter exchange**

For CAN1..4 exchange 1000 frames in each direction. Validate ID, DLC, payload, sequence and error counters; record controller status on failure.

- [ ] **Step 4: Implement termination prompts**

Require external bus disconnection. Through Safety Manager, accept 115–125 Ω when enabled and high-resistance confirmation when disabled. Cleanup disables all termination.

- [ ] **Step 5: Verify disconnect/recovery and commit**

Unplug/reconnect once per port. Expected: controlled timeout, successful reinit and no stale termination.

```powershell
git add ecu/apps/ecu_board_test
git commit -m "feat: test four Classic CAN channels"
```

### Task 11: Implement RS485 and RS232

**Files:** Create `tests/test_rs485.c`, `tests/test_rs232.c`, `selftest/selftest_serial_frames.c`; extend `src/comm_pattern.c`.

- [ ] **Step 1: Write failing serial-codec tests**

Frame fields are magic, channel, direction, sequence, length, payload and CRC16. Cover zero/max length, bad magic, truncation and bad CRC.

- [ ] **Step 2: Implement IRQ/ring-buffer UART transport**

Use `samples/drivers/uart/uart_irq/src/uart.c`; configure 115200/8N1; use bounded buffers and explicit overflow errors.

- [ ] **Step 3: Implement RS485 ×3**

Exercise UART11, UART12 and UART10. Exchange 1000 frames each direction. DE is active only during TX and returns to receive on completion, timeout, error and abort.

- [ ] **Step 4: Implement RS232 ×4**

Exercise UART15, UART13, UART5 and UART14. Exchange the same set and prompt for positive/negative TX level confirmation per channel.

- [ ] **Step 5: Verify and commit**

Expected: seven ports pass with zero CRC/data errors; corrupt frames identify channel/sequence; RS485 ends in receive.

```powershell
git add ecu/apps/ecu_board_test
git commit -m "feat: add isolated RS485 and RS232 tests"
```

### Task 12: Implement SBUS

**Files:** Create `tests/test_sbus.c`, `src/sbus_decoder.c`, `include/sbus_decoder.h`, `selftest/selftest_sbus_decoder.c`.

- [ ] **Step 1: Write failing decoder tests**

Use fixed 25-byte frames for min/center/max channels, lost-frame, failsafe, invalid markers and truncation. Assert all 16 channels and flags.

- [ ] **Step 2: Implement pure decoder**

Decode packed 11-bit channels without unaligned casts; return explicit invalid-length/marker errors.

- [ ] **Step 3: Implement UART1 reception**

Configure 100000/8E2. The board transistor supplies inversion. Accumulate frames with a bounded state machine and reset on framing timeout.

- [ ] **Step 4: Run HIL**

Receive at least 100 legal frames, move known channels, check lost/failsafe, then stop frames and verify stream timeout.

- [ ] **Step 5: Commit**

```powershell
git add ecu/apps/ecu_board_test
git commit -m "feat: add isolated SBUS input test"
```

## Milestone 5 — HIL release

### Task 13: Register the suite and document operation

**Files:** Create `src/test_registry.c`, `test/fixtures/ecu-board-test-procedure.md`, `test/reports/.gitignore`; modify `src/main.c`, `docs/ecu-test-progress.md`.

- [ ] **Step 1: Write failing registry tests**

Assert order: precheck, safe boot, power, RGB, memory, ADC, DI, SBUS, RS232, RS485, CAN, DO, DIO loopback, summary/reset. Assert ETH is optional `SKIP_NO_DEVICE`; CAN-FD is absent.

- [ ] **Step 2: Implement const registry**

Power/safe boot gate later tests. Independent failures do not suppress unrelated modules. Final cleanup and summary always execute.

- [ ] **Step 3: Write operator procedure**

Document instruments, limited 12 V startup, cable changes, commands, prompts, measurements, FAIL/BLOCKED handling, report naming and shutdown.

- [ ] **Step 4: Define reports**

```text
test/reports/YYYYMMDD-HHMMSS_<board-serial>_<firmware-version>.jsonl
```

Only JSON objects enter `.jsonl`; raw console text uses `.log`.

- [ ] **Step 5: Update progress and commit**

Record implemented modules and evidence; never mark hardware PASS without a saved log.

```powershell
git add ecu/apps/ecu_board_test docs test/fixtures test/reports/.gitignore
git commit -m "feat: integrate complete ECU board test suite"
```

### Task 14: Execute release verification

**Files:** Produce local `test/reports/<timestamp>_<serial>_<version>.jsonl`; modify source only for a reproduced defect.

- [ ] **Step 1: Clean build and inspect map**

Run `.\ecu\scripts\build_ecu_test.ps1`. Expected: exit 0, reserved Flash begins `0x807F0000`, no overlap.

- [ ] **Step 2: Run `SELFTEST.ALL`**

Expected: all self-tests PASS and fail count zero.

- [ ] **Step 3: Verify startup safety**

Cold start, external reset and watchdog reset. Expected: DO off, CAN termination off, RS485 receive, correct reason, CLI within 3 s.

- [ ] **Step 4: Execute full fixture procedure**

Run `board <serial>` then `run all`. Expected: required tests PASS, ETH `SKIP_NO_DEVICE`, no BLOCKED, board PASS.

- [ ] **Step 5: Audit report**

Verify all required IDs, retained retries, units/limits and valid JSON per line.

- [ ] **Step 6: Reboot and run smoke subset**

Run safe boot, memory summary, one ADC point, one DI/DO loop and one frame on every communication interface. Expected: PASS and no state leakage.

- [ ] **Step 7: Commit verified source only**

```powershell
git add ecu docs test/fixtures test/reports/.gitignore
git commit -m "test: verify ECU board test release"
```

## Plan self-review

- Every required specification subsystem maps to Tasks 6–12.
- Safety cleanup is implemented in Task 4 and exercised throughout hardware tasks.
- CAN-FD is absent; Ethernet is the sole approved optional skip.
- Flash has an explicit reserved region and geometry guard.
- APIs and status names remain consistent across tasks.
- Every code-changing step names concrete files, APIs and verification evidence.
