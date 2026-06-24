# Periodic Communications Transmit Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add independently compile-controlled, automatic 500 ms diagnostic transmission on CAN1..4, RS485 1..3, and RS232 1..4 without blocking the CLI or interfering with registered hardware tests.

**Architecture:** A hardware-independent foreground `periodic_tx` state machine owns scheduling, frame formatting, counters, suspend/resume, and statistics through an injected backend. A separate HPM6750 backend configures Classic CAN and UART hardware. The service starts after boot self-tests, polls from the existing UART0 wait loop, and suspends around registered tests.

**Tech Stack:** C11, HPM SDK 1.11.0 CAN/UART/MCHTMR drivers, CMake cache definitions, PowerShell build scripts, SEGGER Embedded Studio 8.28, J-Link V9.16.

---

## File map

- Create `ecu/apps/ecu_board_test/include/periodic_tx.h`: public configuration, backend, snapshot, and lifecycle contracts.
- Create `ecu/apps/ecu_board_test/src/periodic_tx.c`: hardware-independent scheduler and frame formatter.
- Create `ecu/apps/ecu_board_test/src/periodic_tx_board.c`: HPM6750 CAN/UART/MCHTMR backend and compile-definition defaults.
- Create `ecu/apps/ecu_board_test/selftest/selftest_periodic_tx.c`: deterministic fake-backend regression tests.
- Modify `ecu/apps/ecu_board_test/CMakeLists.txt`: sources and three independent definitions.
- Modify `ecu/apps/ecu_board_test/include/selftest.h` and `selftest/selftest.c`: register the new self-test.
- Modify `ecu/apps/ecu_board_test/src/main.c`: start the default service after startup self-tests.
- Modify `ecu/apps/ecu_board_test/src/operator_io.c`: poll while UART0 waits.
- Modify `ecu/apps/ecu_board_test/src/app_shell.c`: suspend/resume around registered tests and `SELFTEST.ALL`.
- Modify `ecu/scripts/build_ecu_test.ps1` and `generate_ses_ecu_test.ps1`: expose three diagnostic-build switches.
- Modify `docs/ecu-board-test-code-reading-guide.md` and `docs/ecu-test-progress.md`: document ownership, build commands, and evidence.

### Task 1: Add a failing target self-test for the service contract

**Files:**

- Create: `ecu/apps/ecu_board_test/selftest/selftest_periodic_tx.c`
- Modify: `ecu/apps/ecu_board_test/include/selftest.h`
- Modify: `ecu/apps/ecu_board_test/selftest/selftest.c`
- Modify: `ecu/apps/ecu_board_test/CMakeLists.txt`

- [ ] **Step 1: Register the new self-test before the implementation exists**

Add this declaration to `selftest.h`:

```c
/** @brief Verify periodic communication scheduling, formatting, and ownership. @return true only when every assertion passes. */
bool selftest_periodic_tx(void);
```

Add `{ "periodic_tx", selftest_periodic_tx },` to the `cases[]` table in
`selftest.c`, after `status_led`.

Add `sdk_app_src(selftest/selftest_periodic_tx.c)` to `CMakeLists.txt`.

- [ ] **Step 2: Create the fake-backend test with exact assertions**

The test file must include `periodic_tx.h` and implement fake callbacks for time,
four CAN initializations/sends, three RS485 and four RS232 initializations/writes.
Use `periodic_tx_config_t config = { true, true, true };` and assert:

```c
periodic_tx_init(&fake_ops, &config);
SELFTEST_ASSERT_EQ(4U, can_init_calls);
SELFTEST_ASSERT_EQ(3U, rs485_init_calls);
SELFTEST_ASSERT_EQ(4U, rs232_init_calls);

fake_now_ms = 499U;
periodic_tx_poll();
SELFTEST_ASSERT_EQ(0U, can_send_calls);

fake_now_ms = 500U;
periodic_tx_poll();
SELFTEST_ASSERT_EQ(4U, can_send_calls);
SELFTEST_ASSERT_EQ(3U, rs485_write_calls);
SELFTEST_ASSERT_EQ(4U, rs232_write_calls);
SELFTEST_ASSERT_EQ(0x601U, captured_can[0].id);
SELFTEST_ASSERT_EQ(1U, captured_can[0].data[0]);
SELFTEST_ASSERT_EQ(1U, captured_can[0].data[1]);
SELFTEST_ASSERT_TRUE(strcmp(captured_rs485[0], "RS485-1,00000001\r\n") == 0);
SELFTEST_ASSERT_TRUE(strcmp(captured_rs232[0], "RS232-1,00000001\r\n") == 0);
```

Then suspend at 500 ms, advance to 1000 ms, and assert no new sends. Resume,
assert all initialization counters increment again, then assert no send at 1499
ms and the next send at 1500 ms. Initialize at `UINT32_MAX - 249U`, advance to
250 ms, and assert one wrap-safe interval. Make CAN1's fake send return false and
assert CAN2..4 and all serial channels still execute.

- [ ] **Step 3: Run the GNU build and verify the red state**

Run:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\ecu\scripts\build_ecu_test.ps1
```

Expected: compilation fails because `periodic_tx.h` and service symbols do not
exist. The failure must come from the new self-test, not an unrelated source.

### Task 2: Implement the hardware-independent scheduler

**Files:**

- Create: `ecu/apps/ecu_board_test/include/periodic_tx.h`
- Create: `ecu/apps/ecu_board_test/src/periodic_tx.c`
- Modify: `ecu/apps/ecu_board_test/CMakeLists.txt`

- [ ] **Step 1: Define the complete service interface**

Create these public types and declarations in `periodic_tx.h`, with full English
Doxygen contracts:

```c
#define PERIODIC_TX_PERIOD_MS 500U
#define PERIODIC_TX_CAN_COUNT 4U
#define PERIODIC_TX_RS485_COUNT 3U
#define PERIODIC_TX_RS232_COUNT 4U

typedef enum {
    PERIODIC_TX_SERIAL_RS485,
    PERIODIC_TX_SERIAL_RS232
} periodic_tx_serial_kind_t;

typedef struct {
    bool can_enabled;
    bool rs485_enabled;
    bool rs232_enabled;
} periodic_tx_config_t;

typedef struct {
    uint32_t (*now_ms)(void);
    bool (*can_init)(uint8_t channel);
    bool (*can_send)(uint8_t channel, uint16_t id, const uint8_t data[8]);
    bool (*serial_init)(periodic_tx_serial_kind_t kind, uint8_t channel);
    bool (*serial_write)(periodic_tx_serial_kind_t kind, uint8_t channel,
                         const uint8_t *data, size_t length);
} periodic_tx_hw_ops_t;

typedef struct {
    uint32_t can_sequence[PERIODIC_TX_CAN_COUNT];
    uint32_t rs485_sequence[PERIODIC_TX_RS485_COUNT];
    uint32_t rs232_sequence[PERIODIC_TX_RS232_COUNT];
    uint32_t send_success_count;
    uint32_t send_failure_count;
    bool suspended;
} periodic_tx_snapshot_t;

void periodic_tx_init(const periodic_tx_hw_ops_t *ops,
                      const periodic_tx_config_t *config);
void periodic_tx_init_default(void);
void periodic_tx_poll(void);
void periodic_tx_suspend(void);
void periodic_tx_resume(void);
bool periodic_tx_is_suspended(void);
periodic_tx_snapshot_t periodic_tx_snapshot(void);
```

- [ ] **Step 2: Implement initialization and scheduling**

In `periodic_tx.c`, keep borrowed backend/config copies, per-channel readiness,
counters, `last_ms`, and suspension state in one static service object.

`periodic_tx_init()` must clear all state, copy a non-null config, record
`last_ms = now_ms()`, and call the enabled initialization callbacks using
one-based channels. A null/incomplete backend makes the corresponding channels
not ready without performing I/O.

`periodic_tx_poll()` must use:

```c
uint32_t now = s_ops->now_ms();
uint32_t elapsed = now - s_last_ms;
if (s_suspended || elapsed < PERIODIC_TX_PERIOD_MS) return;
s_last_ms += (elapsed / PERIODIC_TX_PERIOD_MS) * PERIODIC_TX_PERIOD_MS;
```

It sends only one burst even if several periods elapsed, preventing an overdue
burst storm after a foreground delay.

- [ ] **Step 3: Implement exact CAN and ASCII formatting**

For each ready CAN channel, increment its sequence and build:

```c
uint8_t data[8] = {
    channel,
    (uint8_t)sequence,
    (uint8_t)(sequence >> 8U),
    (uint8_t)(sequence >> 16U),
    (uint8_t)(sequence >> 24U),
    0x45U, 0x43U, 0x55U
};
bool ok = s_ops->can_send(channel, (uint16_t)(0x600U + channel), data);
```

For serial channels, use a local 32-byte buffer and `snprintf()` with
`"RS485-%u,%08lX\r\n"` or `"RS232-%u,%08lX\r\n"`. Pass the returned byte count
without the NUL terminator to `serial_write()`. Increment success/failure totals
once per physical-channel attempt.

- [ ] **Step 4: Implement safe suspension and resume**

`periodic_tx_suspend()` sets the flag idempotently. `periodic_tx_resume()` does
nothing when already active; otherwise it re-runs enabled channel initialization,
sets `last_ms` from the backend clock, and clears the flag without clearing
sequence/statistic counters.

- [ ] **Step 5: Build and verify the green self-test**

Add `sdk_app_src(src/periodic_tx.c)` to `CMakeLists.txt`, then run the GNU build.

Expected: link succeeds and startup self-test registration count increases from
five to six. No real peripheral backend is required yet because only the fake
self-test calls `periodic_tx_init()`.

- [ ] **Step 6: Commit the core service**

```powershell
git add ecu/apps/ecu_board_test/include/periodic_tx.h `
  ecu/apps/ecu_board_test/src/periodic_tx.c `
  ecu/apps/ecu_board_test/selftest/selftest_periodic_tx.c `
  ecu/apps/ecu_board_test/include/selftest.h `
  ecu/apps/ecu_board_test/selftest/selftest.c `
  ecu/apps/ecu_board_test/CMakeLists.txt
git commit -m "feat: add periodic communications scheduler"
```

### Task 3: Add the HPM6750 backend and independent compile definitions

**Files:**

- Create: `ecu/apps/ecu_board_test/src/periodic_tx_board.c`
- Modify: `ecu/apps/ecu_board_test/CMakeLists.txt`
- Modify: `ecu/scripts/build_ecu_test.ps1`
- Modify: `ecu/scripts/generate_ses_ecu_test.ps1`

- [ ] **Step 1: Add safe default CMake definitions**

Add cache strings that accept only `0` or `1` and default to zero:

```cmake
set(ECU_PERIODIC_CAN_TX 0 CACHE STRING "Enable periodic CAN1..4 transmit")
set(ECU_PERIODIC_RS485_TX 0 CACHE STRING "Enable periodic RS485 1..3 transmit")
set(ECU_PERIODIC_RS232_TX 0 CACHE STRING "Enable periodic RS232 1..4 transmit")
foreach(flag ECU_PERIODIC_CAN_TX ECU_PERIODIC_RS485_TX ECU_PERIODIC_RS232_TX)
    if(NOT "${${flag}}" MATCHES "^[01]$")
        message(FATAL_ERROR "${flag} must be 0 or 1")
    endif()
    sdk_compile_definitions(-D${flag}=${${flag}})
endforeach()
```

Add `sdk_app_src(src/periodic_tx_board.c)`.

- [ ] **Step 2: Implement the default MCHTMR and CAN backend**

Use a static CAN base table `{ BOARD_CAN1_BASE, ... BOARD_CAN4_BASE }`.
`default_now_ms()` uses the same MCHTMR conversion as `status_led.c`.
`default_can_init(channel)` rejects indexes outside 1..4, then initializes:

```c
can_config_t config;
can_get_default_config(&config);
config.baudrate = 500000U;
config.mode = can_mode_normal;
config.enable_canfd = false;
board_init_can(base);
return can_init(base, &config, board_init_can_clock(base)) == status_success;
```

`default_can_send()` creates a zeroed `can_transmit_buf_t`, assigns the standard
ID, `extend_id=false`, `dlc=8`, copies the payload, and calls
`can_send_message_nonblocking()`. It returns immediately on busy/error.

- [ ] **Step 3: Implement the default RS485 and RS232 backend**

Use static UART tables matching `test_rs485.c` and `test_rs232.c`. Initialize
115200, 8 data bits, no parity, one stop bit, FIFO enabled. Set
`config.modem_config.auto_flow_ctrl_en=true` only for RS485 so generated hardware
DE pins own direction.

`default_serial_write()` validates kind/channel and sends exactly `length` bytes
with `uart_send_byte()`, returning false at the first driver error.

`periodic_tx_init_default()` constructs:

```c
static const periodic_tx_hw_ops_t ops = {
    default_now_ms, default_can_init, default_can_send,
    default_serial_init, default_serial_write
};
const periodic_tx_config_t config = {
    ECU_PERIODIC_CAN_TX != 0,
    ECU_PERIODIC_RS485_TX != 0,
    ECU_PERIODIC_RS232_TX != 0
};
periodic_tx_init(&ops, &config);
```

- [ ] **Step 4: Expose diagnostic switches in both PowerShell generators**

Extend `build_ecu_test.ps1` parameters with:

```powershell
[switch]$PeriodicCanTx,
[switch]$PeriodicRs485Tx,
[switch]$PeriodicRs232Tx
```

Convert each switch to `0`/`1` and append these CMake arguments:

```powershell
"-DECU_PERIODIC_CAN_TX=$canTx"
"-DECU_PERIODIC_RS485_TX=$rs485Tx"
"-DECU_PERIODIC_RS232_TX=$rs232Tx"
```

Give `generate_ses_ecu_test.ps1` the same switches and forward them to
`build_ecu_test.ps1 -ConfigureOnly` using explicit `-Switch:$Switch` syntax.

- [ ] **Step 5: Verify disabled and fully enabled builds**

Run the normal default build, then:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File .\ecu\scripts\build_ecu_test.ps1 `
  -PeriodicCanTx -PeriodicRs485Tx -PeriodicRs232Tx
```

Expected: both exit 0. Inspect `compile_commands.json` or Ninja commands and
confirm all three enabled definitions equal `1` only in the diagnostic build.

- [ ] **Step 6: Commit backend and build switches**

```powershell
git add ecu/apps/ecu_board_test/src/periodic_tx_board.c `
  ecu/apps/ecu_board_test/CMakeLists.txt ecu/scripts/build_ecu_test.ps1 `
  ecu/scripts/generate_ses_ecu_test.ps1
git commit -m "feat: add compile-controlled communications backends"
```

### Task 4: Integrate startup, polling, and test ownership

**Files:**

- Modify: `ecu/apps/ecu_board_test/src/main.c`
- Modify: `ecu/apps/ecu_board_test/src/operator_io.c`
- Modify: `ecu/apps/ecu_board_test/src/app_shell.c`

- [ ] **Step 1: Start and poll the service**

In `main.c`, call `periodic_tx_init_default()` after startup self-tests restore
the real Safety Manager and default status LED adapters, and before
`app_shell_run()`.

In `operator_read_line()`, call `periodic_tx_poll()` immediately after
`status_led_poll()` on every foreground wait iteration. UART0 remains separate
from all diagnostic serial ports.

- [ ] **Step 2: Suspend around peripheral-owning operations**

In `run_descriptor()`:

```c
periodic_tx_suspend();
test_status_t status = test_runner_execute(descriptor, &context);
periodic_tx_resume();
```

Wrap `SELFTEST.ALL` with the same suspend/resume bracket. Keep resume after
Safety Manager and status LED restoration so fake self-test backends cannot leak
into periodic operation.

- [ ] **Step 3: Run self-tests and both enabled builds**

Run GNU with all three switches. Generate SES with all three switches and run:

```powershell
& 'D:\Program Files\SEGGER\SEGGER Embedded Studio 8.28\bin\emBuild.exe' `
  -config Debug -rebuild `
  .\tmp\ecu_board_test_build\segger_embedded_studio\ecu_board_test.emProject
```

Expected: both builds exit 0; target self-test summary is designed for
`pass=6 fail=0` after download.

- [ ] **Step 4: Commit integration**

```powershell
git add ecu/apps/ecu_board_test/src/main.c `
  ecu/apps/ecu_board_test/src/operator_io.c `
  ecu/apps/ecu_board_test/src/app_shell.c
git commit -m "feat: integrate automatic periodic communication"
```

### Task 5: Documentation, download, and hardware evidence

**Files:**

- Modify: `docs/ecu-board-test-code-reading-guide.md`
- Modify: `docs/ecu-test-progress.md`

- [ ] **Step 1: Document compile commands and ownership**

Add the service files to the reading guide. Document that defaults are OFF and
that this command enables all groups:

```powershell
.\ecu\scripts\build_ecu_test.ps1 `
  -PeriodicCanTx -PeriodicRs485Tx -PeriodicRs232Tx
```

State that periodic traffic pauses during every registered test and is not proof
of electrical PASS.

- [ ] **Step 2: Run final static and build verification**

```powershell
git diff --check
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File .\ecu\scripts\build_ecu_test.ps1 `
  -PeriodicCanTx -PeriodicRs485Tx -PeriodicRs232Tx
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File .\ecu\scripts\generate_ses_ecu_test.ps1 `
  -PeriodicCanTx -PeriodicRs485Tx -PeriodicRs232Tx
& 'D:\Program Files\SEGGER\SEGGER Embedded Studio 8.28\bin\emBuild.exe' `
  -config Debug -rebuild `
  .\tmp\ecu_board_test_build\segger_embedded_studio\ecu_board_test.emProject
```

Expected: all commands exit 0 and no CAN-FD symbol/test is added.

- [ ] **Step 3: Download the enabled GNU image without GUI automation**

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File .\ecu\scripts\flash_ecu_test.ps1
```

Expected: J-Link transcript contains target power, `Downloading file`, and `O.K.`.

- [ ] **Step 4: Capture UART0 boot evidence**

Read COM9 at 115200 8N1. Expected startup self-test summary is
`pass=6 fail=0`, followed by the CLI prompt. Do not use or automate the Windows
USB-CAN application.

- [ ] **Step 5: Observe external communication one group at a time**

The user operates external analyzer software and wiring. Record only directly
observed evidence:

- CAN1..4 show IDs `0x601..0x604`, DLC 8, about 500 ms period, incrementing LE
  counters and `45 43 55` signature;
- RS485 1..3 show `RS485-n,XXXXXXXX` at 115200 8N1;
- RS232 1..4 show `RS232-n,XXXXXXXX` at 115200 8N1.

Unconnected or unobserved channels remain UNVERIFIED. Do not mark the board PASS.

- [ ] **Step 6: Record evidence and commit**

Update `docs/ecu-test-progress.md` with exact build, J-Link, COM9, and external
analyzer results. Then:

```powershell
git add docs/ecu-board-test-code-reading-guide.md docs/ecu-test-progress.md
git commit -m "docs: record periodic communications verification"
```

- [ ] **Step 7: Push the existing review branch**

```powershell
git push origin codex/ecu-board-test
```

Expected: draft PR #1 advances without creating a duplicate PR.
