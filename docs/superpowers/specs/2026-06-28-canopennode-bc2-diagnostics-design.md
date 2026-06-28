# CANopenNode BC2 Diagnostics Design

## Purpose

Bring up HPM SDK CANopenNode on ECU CAN2 and verify communication with the connected BC2 drive while the drive only has auxiliary power. This phase is diagnostic-only: it may read CANopen data and print status, but it must not command torque, speed, position, enable operation, or NMT operational state.

## Scope

- Use the DS301 object dictionary generated in `doc/ECU/DS301_OD`.
- Enable HPM SDK CANopenNode through an explicit build option.
- Bind CANopenNode diagnostics to external CAN2 (`BOARD_CAN2_BASE`, `BOARD_CAN2_IRQn`) at `1 Mbit/s`.
- Print diagnostic state to the existing COM9 debug console.
- Add Python tooling for the USB CAN analyzer and COM10 Modbus RTU probing.

Out of scope for this phase:

- CiA 402 enable sequence.
- PDO motion control.
- Writing persistent drive parameters.
- Any command that can cause motor motion when main power is later connected.

## Architecture

`ecu/protocol/canopen/od/ds301` owns the generated `OD.c` and `OD.h`. A new `canopen_master_service` owns the CANopenNode/HPM port runtime for CAN2. CPU0 calls the service from the existing `can2_motion` task and reports a compact snapshot through `runtime_monitor`.

CAN2 ownership is selected at compile time:

- `ECU_ENABLE_CANOPENNODE=OFF`: keep the current RX-only `can_bus_hw` monitor.
- `ECU_ENABLE_CANOPENNODE=ON`: CANopenNode owns CAN2; the RX-only monitor is not initialized for CAN2.

The existing `servo_drive_canopen` device adapter remains the high-level boundary for future servo control. During this phase it is not wired to transmit hardware control frames.

## Diagnostic behavior

On startup, the CANopen diagnostics service:

1. Initializes CANopenNode on CAN2 with the DS301 OD.
2. Keeps the local CANopen node in a safe diagnostic state.
3. Processes received heartbeat/EMCY/SDO response traffic.
4. Attempts bounded SDO uploads from configured BC/BC2 node IDs.
5. Stores results in a snapshot for COM9 printing.

Allowed reads:

- `0x1000:00` device type
- `0x1001:00` error register
- `0x1018:01..04` identity object
- `0x6041:00` statusword
- `0x6061:00` mode display

Forbidden in this phase:

- NMT start remote node (`0x000`, command `0x01`)
- controlword writes to `0x6040`
- target position/velocity/torque writes
- RPDO control transmission

## Python tools

`tools/can/controlcan.py` wraps the vendor `ControlCAN.dll` with `ctypes`.

`tools/can/can2_monitor.py` opens the CAN analyzer, initializes channel 2 at 1 Mbit/s, and prints received frames. It is passive unless a future command-line flag explicitly enables transmit.

`tools/modbus/rtu_probe.py` lists COM ports and provides safe Modbus RTU read helpers for COM10. It must not perform write requests by default.

## Verification

- Python contract tests prove the compile-time ownership switch and forbidden motion-control behavior.
- CPU0 builds with CANopenNode disabled and enabled.
- COM9 shows CANopen diagnostic startup and SDO/heartbeat state.
- CAN analyzer confirms ECU CAN2 traffic and BC2 responses.
