# Periodic Communications Transmit Design

## Goal

Add a long-term, compile-time-controlled diagnostic transmitter to the ECU
board-test firmware. When enabled, the firmware automatically transmits on all
configured CAN, RS485, and RS232 channels every 500 ms after startup self-tests.
The operator CLI must remain responsive, and existing hardware tests must retain
exclusive ownership of their peripherals.

## Scope and compile-time control

Three independent compile definitions control the feature:

- `ECU_PERIODIC_CAN_TX`
- `ECU_PERIODIC_RS485_TX`
- `ECU_PERIODIC_RS232_TX`

Each definition accepts `0` or `1` and defaults to `0` for release safety. A
diagnostic build may enable any combination. The initial hardware-verification
build enables all three.

No CAN-FD support is added. Existing `CAN.ALL`, RS485, and RS232 tests keep their
current behavior and registry identifiers.

## Architecture

Create a foreground-polled `periodic_tx` service with these responsibilities:

- initialize only the peripheral groups enabled at compile time;
- obtain wrapping millisecond time from MCHTMR;
- transmit one frame on every enabled channel when 500 ms has elapsed;
- suspend before a registered hardware test acquires peripherals;
- reinitialize changed peripheral configuration and resume after the test;
- never block the operator console indefinitely when a bus or peer is absent.

The service is initialized after board initialization, Safety Manager setup, and
startup self-tests. `periodic_tx_poll()` is called from foreground wait paths,
especially the UART0 line-input loop. It uses unsigned timestamp subtraction so
the schedule remains correct across `uint32_t` millisecond wrap.

`run_descriptor()` suspends periodic transmission before calling the Test Runner
and resumes it after lifecycle cleanup. This gives the existing registered test
exclusive access to CAN and serial peripherals and prevents diagnostic frames
from corrupting deterministic echo traffic.

## CAN behavior

When `ECU_PERIODIC_CAN_TX=1`, initialize external CAN1 through CAN4 as Classic CAN
at 500 kbit/s in normal mode. CAN-FD remains disabled.

Every 500 ms, each port attempts one standard data frame:

| Port | Standard ID |
| --- | --- |
| CAN1 | `0x601` |
| CAN2 | `0x602` |
| CAN3 | `0x603` |
| CAN4 | `0x604` |

All frames use DLC 8. The payload is:

- byte 0: one-based channel number;
- bytes 1..4: little-endian 32-bit per-channel sequence counter;
- bytes 5..7: fixed signature `0x45, 0x43, 0x55` (`ECU`).

Transmission uses the SDK non-blocking API. A busy transmit buffer, missing ACK,
or bus-off condition must not stall UART0 or the RGB status service. The service
records unsuccessful attempts and retries on a later poll; it does not declare a
hardware PASS. Existing CAN termination remains disabled unless explicitly owned
by `CAN.ALL`.

## RS485 and RS232 behavior

When enabled, configure:

- RS485 ports 1..3 as 115200 8N1 with UART automatic hardware DE;
- RS232 ports 1..4 as 115200 8N1 without automatic DE.

Each port emits one ASCII line every 500 ms using an independent counter:

```text
RS485-1,00000001\r\n
RS232-1,00000001\r\n
```

The prefix and one-based port number identify the source. The counter is eight
uppercase hexadecimal digits and wraps naturally at `UINT32_MAX`. Messages are
short enough for foreground byte transmission; calls are bounded by UART FIFO
progress and contain no receive wait.

## Scheduling and state

One 500 ms deadline is shared by all enabled groups so frames appear as a periodic
burst. Each physical channel owns its own sequence counter. Counters start at zero
and the first transmitted frame carries sequence 1. Resume after a hardware test
preserves counters but restarts the 500 ms deadline after peripheral
reinitialization.

The service exposes initialization, polling, suspension, resume, and read-only
diagnostic snapshot APIs. Calling suspend/resume repeatedly is safe. Builds with
all three macros disabled compile to inert service functions with no peripheral
side effects.

## Error handling and safety

- Invalid or unsupported peripheral instances are treated as initialization
  failures for that channel; other channels continue.
- CAN sends are non-blocking and never wait for an ACK in foreground code.
- Serial transmission does not control vehicle outputs or CAN termination.
- Safety Manager cleanup remains authoritative for vehicle-affecting resources.
- Periodic transmission never runs while a registered test is executing.
- Status/reporting distinguishes attempted, successful, and failed transmissions;
  diagnostic traffic alone is never evidence that a physical interface passed.

## Verification

Target-side self-tests use an injectable time and transport backend to verify:

- no frame before 500 ms;
- one frame per enabled channel at 500 ms;
- correct CAN IDs/payloads and serial ASCII format;
- independent sequence counters;
- suspension suppresses traffic;
- resume reinitializes and restarts the deadline;
- timestamp wrap behavior;
- failed CAN attempts do not block later channels or the console.

Build verification covers HPM SDK 1.11.0 GNU and SEGGER Embedded Studio 8.28.
Hardware verification then enables all three macros, downloads with J-Link, and
uses external analyzers to observe CAN1..4, RS485 1..3, and RS232 1..4. Each
interface remains unverified until its external frames are captured.

