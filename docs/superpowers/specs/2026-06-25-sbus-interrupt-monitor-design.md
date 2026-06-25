# SBUS Interrupt Monitor Design

## Goal

Make the board-test firmware easier to debug by moving SBUS reception out of the slow debug-print path and into a continuously running UART interrupt service. Keep the existing debugger-controlled monitor, but make it read clear state structures instead of owning hardware directly.

## Current Root Cause

The previous SBUS debug path reads UART1 only when `ecu_debug_monitor_poll()` prints, typically every 200 ms. A real SBUS receiver sends frames continuously, so UART1 can overflow before the next print. Direct UART register checks and the formal `SBUS.RX` test showed valid SBUS bytes and decoded frames, while the debug monitor still printed `SBUS no_frame`. The broken boundary is therefore the monitor polling design, not the SBUS wiring or decoder.

## Selected Approach

Add a focused `sbus_service` module:

- Configure UART1 for SBUS at 100000 baud, even parity, 2 stop bits.
- Enable RX data/timeout and RX idle interrupts.
- Drain UART RX FIFO inside the ISR.
- Assemble standard 25-byte SBUS frames.
- Decode frames with the existing `sbus_decoder`.
- Publish the latest status in a debugger-readable global structure.

The debug monitor will read this structure through a small snapshot API and only format text. It will no longer configure or read UART1.

## Public Debug State

Expose a global structure named `g_sbus_debug_state` so SEGGER Embedded Studio can watch it directly:

- `connected`: 1 when a valid frame arrived within the freshness timeout, otherwise 0.
- `frame_count`: total decoded valid frames.
- `decode_error_count`: invalid 25-byte candidate frames.
- `uart_error_count`: UART overrun, parity, framing, break, or FIFO error events observed in ISR.
- `last_frame_ms`: timestamp of the last valid decoded frame.
- `channels[16]`: latest decoded channel values.
- `frame_lost`, `failsafe`, `channel17`, `channel18`: SBUS status bits.

The service also provides `sbus_service_get_snapshot()` so foreground code can copy the state consistently before printing.

## Data Flow

UART1 interrupt receives bytes continuously. The frame assembler ignores bytes until the SBUS start byte `0x0F`, then stores exactly 25 bytes. A valid decoded frame updates `g_sbus_debug_state`. The foreground loop calls `sbus_service_poll()` to refresh `connected` based on time. The debug monitor formats the last snapshot, so print rate no longer affects receive reliability.

## Other Debug-Monitor Cleanup

Keep ADC, DI, and DO debug behavior, but make the monitor code more human-readable:

- SBUS printing includes `connected`, `frames`, `errors`, all selected channels, `lost`, and `failsafe`.
- ADC output remains in millivolts.
- DI output remains direct `0/1`.
- DO remains debugger-controlled level output, not pulse output.
- Comments explain function responsibilities and ISR constraints in English.

## Testing

Use target-side selftests before implementation changes are considered complete:

- SBUS service selftest feeds a known valid frame through the service byte-ingest path and checks channel/state output.
- SBUS service selftest feeds invalid framing and checks `decode_error_count`.
- SBUS service selftest advances time beyond the timeout and checks `connected=0`.
- Debug monitor selftest checks the new SBUS print format reads service-style state and includes connection metadata.

Hardware verification after build:

- Rebuild firmware with SDK 1.11 tooling.
- Flash with J-Link if COM9/J-Link are available.
- Run `SELFTEST.ALL`.
- With the receiver connected, enable SBUS debug view through `g_ecu_debug_monitor` and confirm COM9 prints `connected=1` and 16 channel values.

