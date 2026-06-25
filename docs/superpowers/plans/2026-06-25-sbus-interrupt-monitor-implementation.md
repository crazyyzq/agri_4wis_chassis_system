# SBUS Interrupt Monitor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace slow SBUS polling with UART1 interrupt reception and make the debug monitor read clear state structures.

**Architecture:** `sbus_service` owns UART1/SBUS reception and publishes `g_sbus_debug_state`; `debug_monitor` owns debugger control and text formatting only; `test_sbus` verifies the same service state used by the monitor.

**Tech Stack:** C99, HPMicro SDK 1.11 UART driver, HPM6750 PLIC interrupt declaration, existing ECU board-test selftest framework, SEGGER Embedded Studio 8.28 generated project.

---

## Files

- Create `ecu/apps/ecu_board_test/include/sbus_service.h`: public SBUS runtime state, lifecycle, snapshot, and test-feed API.
- Create `ecu/apps/ecu_board_test/src/sbus_service.c`: UART1 configuration, ISR, byte assembler, timeout logic, and state publishing.
- Create `ecu/apps/ecu_board_test/selftest/selftest_sbus_service.c`: target-side selftests for valid frames, invalid frames, and timeout.
- Modify `ecu/apps/ecu_board_test/CMakeLists.txt`: add new sources.
- Modify `ecu/apps/ecu_board_test/include/selftest.h`: declare `selftest_sbus_service()`.
- Modify `ecu/apps/ecu_board_test/selftest/selftest.c`: register `sbus_service` selftest.
- Modify `ecu/apps/ecu_board_test/src/main.c`: initialize the SBUS service after selftests restore real hardware ownership.
- Modify `ecu/apps/ecu_board_test/src/operator_io.c`: poll SBUS service timeout in the foreground loop.
- Modify `ecu/apps/ecu_board_test/src/app_shell.c`: suspend debug output during tests but leave SBUS receive service available.
- Modify `ecu/apps/ecu_board_test/include/debug_monitor.h`: replace SBUS backend frame callback with service-style state callback.
- Modify `ecu/apps/ecu_board_test/src/debug_monitor.c`: remove direct UART SBUS polling and print service snapshots.
- Modify `ecu/apps/ecu_board_test/selftest/selftest_debug_monitor.c`: update fake backend and expected SBUS text.
- Modify `ecu/apps/ecu_board_test/tests/test_sbus.c`: verify SBUS through `sbus_service` instead of direct UART polling.
- Modify `docs/ecu-test-progress.md`: record the design, verification, and hardware state.
- Modify `docs/ecu-board-test-code-reading-guide.md`: add the new SBUS service to the reading map.

## Task 1: RED selftest for SBUS runtime state

- [ ] Add `sbus_service.h` with the public state type and function declarations only.
- [ ] Add `selftest_sbus_service.c` that feeds a valid SBUS frame, an invalid frame, and a timeout scenario through the desired API.
- [ ] Register the selftest and CMake source.
- [ ] Run `powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\ecu\scripts\build_ecu_test.ps1`.
- [ ] Expected RED: build fails because the new `sbus_service` functions and `g_sbus_debug_state` are not implemented.

## Task 2: GREEN SBUS service core

- [ ] Implement `sbus_service.c` enough for the new selftest to link and pass.
- [ ] Keep hardware init separable from byte ingestion so selftests do not touch UART registers.
- [ ] Use `sbus_service_test_feed_byte(now_ms, byte)` only for selftests.
- [ ] Run the build command again.
- [ ] Expected GREEN: build exits 0 and includes the new selftest.

## Task 3: UART1 interrupt integration

- [ ] Add `sbus_service_init()` hardware configuration for UART1 100000 8E2.
- [ ] Enable RX data/timeout, RX line status, and RX idle interrupts.
- [ ] Add `SDK_DECLARE_EXT_ISR_M(BOARD_SBUS_UART_IRQ, sbus_uart_isr)`.
- [ ] In the ISR, record line errors, drain RX bytes, and clear RX idle flags.
- [ ] Run the build command.
- [ ] Expected: build exits 0.

## Task 4: Debug monitor cleanup

- [ ] Change the SBUS backend from `read_sbus(sbus_frame_t *)` to `read_sbus_state(sbus_debug_state_t *)`.
- [ ] Update default backend to call `sbus_service_get_snapshot()`.
- [ ] Update `print_sbus()` to print connection state, frame count, errors, selected channels, lost, and failsafe.
- [ ] Update `selftest_debug_monitor.c` expected text.
- [ ] Run the build command.
- [ ] Expected: build exits 0.

## Task 5: Formal SBUS test uses the service

- [ ] Replace direct UART polling in `test_sbus.c` with snapshot polling.
- [ ] Wait for `frame_count` to advance to 100 while printing frame 1 and frame 100.
- [ ] Check `connected` becomes 0 only after the operator confirms the transmitter is stopped.
- [ ] Run the build command.
- [ ] Expected: build exits 0.

## Task 6: Documentation and hardware verification

- [ ] Update progress and reading-guide docs.
- [ ] Find the fresh `g_ecu_debug_monitor` and `g_sbus_debug_state` symbols with `riscv32-unknown-elf-nm`.
- [ ] If COM9 and J-Link are available, flash the image, run `SELFTEST.ALL`, enable SBUS debug view, and capture COM9 output.
- [ ] Record exact verification evidence in `docs/ecu-test-progress.md`.
- [ ] Commit the completed changes.

