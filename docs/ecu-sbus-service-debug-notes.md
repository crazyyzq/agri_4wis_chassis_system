# ECU SBUS Service Debug Notes

Last updated: 2026-06-25

## What changed

SBUS reception is now a continuous service instead of a debug-print side effect.

- `ecu/apps/ecu_board_test/src/sbus_service.c`
  - owns UART1 / SBUS hardware configuration;
  - configures 100000 baud, 8 data bits, even parity, 2 stop bits;
  - enables RX data/timeout, RX line-status, and RX idle interrupts;
  - drains RX FIFO in the ISR;
  - assembles standard 25-byte SBUS frames;
  - decodes frames with `sbus_decoder.c`;
  - publishes the latest state in `g_sbus_debug_state`.

- `ecu/apps/ecu_board_test/include/sbus_service.h`
  - exposes `sbus_debug_state_t`;
  - exposes `g_sbus_debug_state` for SEGGER/J-Link live watch;
  - exposes `sbus_service_get_snapshot()` for foreground code;
  - exposes selftest feed functions that do not touch UART hardware.

- `ecu/apps/ecu_board_test/src/debug_monitor.c`
  - no longer configures or reads UART1 for SBUS;
  - reads `sbus_service_get_snapshot()` through the monitor backend;
  - prints `connected`, `frames`, `errors`, selected channels, `lost`, `failsafe`, `ch17`, `ch18`, and `last_ms`.

- `ecu/apps/ecu_board_test/tests/test_sbus.c`
  - no longer reconfigures UART1;
  - waits for `g_sbus_debug_state.frame_count` to advance by 100 frames;
  - checks `connected` becomes 0 after the operator confirms the transmitter has stopped.

## Debugger watch points

Watch these symbols in SEGGER Embedded Studio:

- `g_ecu_debug_monitor`
  - `enable = 1`
  - `view = 1` for SBUS
  - `view = 2` for ADC
  - `view = 3` for DI
  - `view = 4` for DO
  - `view = 5` for all
  - `channel = 0` for all channels, or one-based channel number
  - `period_ms` defaults to 200 and is clamped to at least 50
  - `do_enable = 1` makes DO follow `do_mask`
  - `do_mask` bit 0 maps to EX_OUT1, bit 11 maps to EX_OUT12

- `g_sbus_debug_state`
  - `connected`: 1 when a valid SBUS frame arrived within 100 ms
  - `frame_count`: valid decoded frame counter
  - `decode_error_count`: invalid 25-byte SBUS candidates
  - `uart_error_count`: UART line/FIFO error events
  - `channels[0..15]`: latest 16 SBUS channel values
  - `frame_lost`, `failsafe`, `channel17`, `channel18`: latest SBUS status bits
  - `last_frame_ms`: timestamp of the latest valid frame

## Expected COM9 monitor examples

SBUS connected:

```text
SBUS connected=1 frames=123 errors=0 ch1=1002 ch2=1000 ... ch16=1001 lost=0 failsafe=0 ch17=0 ch18=0 last_ms=45678
```

SBUS disconnected:

```text
SBUS connected=0 frames=123 errors=0 ch1=1002 ... lost=0 failsafe=0 ch17=0 ch18=0 last_ms=45678
```

ADC:

```text
ADC EX1=1200mV EX2=2500mV EX3=0mV EX4=4998mV
```

DI:

```text
DI EX_IN1=0 EX_IN2=1 ... EX_IN12=0
```

DO:

```text
DO mask=0x005 EX_OUT1=1 EX_OUT2=0 EX_OUT3=1 ...
```

## Verification evidence in this implementation session

- TDD RED: build failed at link because `sbus_service_test_feed_byte`, `sbus_service_test_reset`, `sbus_service_get_snapshot`, and `sbus_service_test_poll_at` were intentionally declared but not implemented.
- TDD GREEN: GNU SDK build passed after `sbus_service.c` was added.
- Integration build: GNU SDK build passed after `main.c`, `operator_io.c`, `app_shell.c`, `test_runner.c`, `debug_monitor.c`, and `test_sbus.c` were updated.

## Final bench verification for this image

The latest image in this session was built, flashed, and checked on the connected board.

- GNU build: passed.
- J-Link: connected through JTAG, VTref = 3.277 V, TAP ID `0x1000563D`, ELF download verified with `O.K.`.
- Symbols:
  - `g_ecu_debug_monitor = 0x01080020`
  - `g_sbus_debug_state = 0x01080220`
- COM9 `SELFTEST.ALL`: `SELFTEST SUMMARY pass=8 fail=0`.
- COM9 `run SBUS.RX` receive phase:
  - frame 1 received with `ch1=1002 connected=1 lost=0 failsafe=0`;
  - frame 100 received with `ch1=1002 connected=1 lost=0 failsafe=0`;
  - final result was `BLOCKED` only because the transmitter-stop confirmation was answered `n`.
- COM9 SBUS monitor:
  - printed `SBUS connected=1`;
  - printed all 16 channels;
  - `frame_count` advanced while printing;
  - `lost=0`, `failsafe=0`.
- COM9 ALL monitor:
  - printed SBUS state;
  - printed `ADC EX1=...mV EX2=...mV EX3=...mV EX4=...mV`;
  - printed all 12 DI values as `0/1`;
  - printed all 12 DO values as `0/1`;
  - DO was left safe because `do_enable=0` and `do_mask=0`.

At the end of verification, `g_ecu_debug_monitor.enable`, `do_enable`, and `do_mask` were cleared through J-Link to stop monitor printing and keep DO safe.
