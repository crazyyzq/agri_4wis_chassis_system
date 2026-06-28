# Power Supplier CAN Design

## Scope

Implement the CAN1 power-bus protocol for the known BMS, DCDC48, DCDC12 and DCAC supplier documents. The power bus uses classic CAN 2.0B extended frames at 250 kbit/s. CANopen remains limited to BC/BC2 servo devices on CAN2; power devices use their documented supplier CAN frames.

## Architecture

The implementation is split into three small layers:

- `can_bus_service` owns bus-level accounting and exposes a generic classic CAN frame API. CANopen helpers become one consumer of that API.
- `power_can_protocol` owns pure pack/parse logic for BMS, DCDC48, DCDC12 and DCAC frames. It has no hardware or task dependency.
- `power_device` owns command scheduling, receive-frame decoding, timeout handling and the snapshot consumed by CPU0 feedback and the runtime monitor.

CAN1 hardware binding is kept in `can_bus_hw`. It initializes `BOARD_CAN1_BASE`, records RX frames into the service, and registers a transmit backend so device logic does not touch HPM registers.

## Command Behavior

Default firmware sends safe commands on CAN1 when the supplier protocol is enabled:

- BMS command every 20 ms: contactor request disconnect when high voltage is not requested, connect when high voltage is requested; key status is ON.
- DCDC48 control every 100 ms: disable when high voltage is not requested, enable when high voltage is requested; voltage/current defaults are macros.
- DCDC12 control every 200 ms: stop when high voltage is not requested, start when high voltage is requested; voltage/current defaults are macros.
- DCAC control every 500 ms: stop when high voltage is not requested, start when high voltage is requested; output voltage default is a macro.

The command values are centralized in `ecu_config.h` so final vehicle calibration changes do not require editing protocol logic.

## Feedback and Faults

Each received status frame updates a typed snapshot with the decoded engineering units and `last_rx_ms`. Online flags are based on per-device timeout macros. CPU0 hardware feedback uses these snapshots:

- `can1_power_online` is true when CAN1 is online and at least one enabled power device is recently online.
- `low_voltage_ok` is true when enabled DCDC feedback is recent and does not report a communication/fault stop condition.
- `power_ready` is true when BMS feedback is recent, BMS fault level is zero, and the commanded high-voltage state matches relay feedback.

The runtime monitor prints one compact `ECU POWER` line with command state, online flags, key BMS/DCDC/DCAC values and last CAN1 frame.

## Testing

Tests are source-level contract tests because the development machine may not have a native C test compiler. They verify:

- exact supplier CAN IDs, byte order, scale constants and periods;
- separation between power supplier CAN and CANopen;
- CAN1 hardware TX/RX binding exists and uses extended frame fields;
- `power_device` exposes receive processing and a readable snapshot;
- runtime monitor includes CAN1 power diagnostics.

Target build checks still run after implementation to catch syntax and SDK integration issues.
