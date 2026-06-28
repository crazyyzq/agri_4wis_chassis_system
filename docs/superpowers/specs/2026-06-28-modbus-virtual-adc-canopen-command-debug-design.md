# Modbus Virtual ADC and CANopen Command Debug Design

## Goal

Complete the next hardware-debug checkpoint by adding a PC-side virtual Modbus ADC module, wiring ECU RS485_1 into a real Modbus master path, and expanding the verified CANopenNode CAN2 bring-up from read-only diagnostics to controlled BC/BC2 command debugging.

## Scope

This checkpoint covers two communication paths:

1. RS485_1 to a virtual 8-channel analog input module on COM10.
2. CAN2 to the connected BC2 servo drive through HPM SDK CANopenNode.

This checkpoint does not implement autonomous vehicle motion. It verifies device-level commands, protocol framing, response handling and debug observability.

## Safety model

CANopen command output is enabled only through explicit debug controls. ECU boot must remain safe:

- no automatic NMT Operational on boot;
- no automatic CiA 402 enable operation on boot;
- no automatic target position, target velocity or target torque on boot;
- no command repeats unless the debug control structure requests it;
- all command-producing variables are visible and named for debugger/watch use;
- command execution records the last command, result, abort code and object index on COM9.

The first implementation may run while the BC2 drive has only auxiliary power. If main power and motor are later connected, the same debug controls are still required before any state-changing command is transmitted.

## Modbus virtual ADC behavior

The virtual device represents the external 8-channel analog acquisition module described by `doc/模拟信号采集/8路模拟量采集模块说明书.doc`.

Protocol behavior:

- serial default: `9600, 8, N, 1`;
- default slave ID: `1`;
- supported function: `04 Read Input Registers`;
- supported request: start address `0`, quantity `1..8`;
- channel registers: AI1..AI8 at protocol addresses `0..7`;
- returned register value: unsigned 16-bit raw ADC value, `0..65535`;
- PC tool prints each received request and each response.

ECU conversion:

- RS485_1 periodically sends function 04 request for 8 registers;
- valid responses update `analog_input_service`;
- mV conversion uses `adc_external_mv_max / 65535`, default `5000 mV`;
- COM9 prints Modbus counters and ADC mV values.

## RS485_1 ECU transport

RS485_1 maps to board UART11:

- `BOARD_RS485_1_UART_BASE = HPM_UART11`;
- `BOARD_RS485_1_UART_IRQ = IRQn_UART11`;
- DE uses the UART hardware DE pin from current pinmux.

The UART ISR only drains RX bytes into a bounded service buffer and records line errors. It does not parse Modbus, print, block, or call device logic. The CPU0 IO task owns Modbus request scheduling and response parsing.

## CANopen command-debug behavior

The CANopenNode service remains the owner of external CAN2 when `ECU_ENABLE_CANOPENNODE=ON`.

The service is extended from read-only diagnostics to a debug master that supports:

- NMT commands: pre-operational, operational, stopped, reset node, reset communication;
- SDO downloads for CiA 402 objects:
  - controlword;
  - modes of operation;
  - target position;
  - target velocity;
  - target torque;
- SDO uploads for confirmed diagnostics:
  - device type;
  - error register;
  - identity;
  - statusword;
  - modes of operation display.

The initial command path uses SDO downloads because it is explicit, observable and easier to verify safely than RPDO motion output. RPDO mapping can be added later after the BC/BC2 manual mapping is confirmed and main-power motor tests are planned.

## Debug control structure

The service exposes a volatile debug control structure intended for debugger/watch editing. The structure contains:

- target node ID;
- command sequence number;
- command type enum;
- object index/subindex for generic SDO write;
- data size and value;
- typed CiA 402 fields: controlword, mode, target position, target velocity, target torque;
- repeat-period field for deliberate repeated writes;
- cancel/clear command.

The service consumes a command only when the sequence number changes. This prevents random debugger refreshes from retransmitting commands.

## Observability

COM9 debug output must show:

- CANopen init/state/bitrate/node IDs;
- last SDO upload result;
- last SDO download result;
- last NMT command;
- command counters, abort code and last error;
- Modbus TX/RX/timeout/error counters;
- latest ADC raw and mV values.

The PC tools show:

- virtual Modbus ADC requests/responses;
- CAN analyzer passive frame capture.

## Cleanup and architecture rules

- Do not add a project-local CANopen stack.
- Do not add a project-local Modbus stack beyond thin Agile Modbus wrappers and test tools.
- Remove or isolate raw-frame fallback paths from the default command path where CANopenNode owns CAN2.
- Protocol object/register defaults live in `ecu/config`.
- Device adapters expose high-level operations and do not parse UART bytes or CAN frames directly.
- ISR code only buffers data and records errors.

## Verification

Required verification before claiming the checkpoint complete:

- Python tests pass.
- Forbidden-pattern checker passes.
- Default CPU0 build passes.
- CANopen debug CPU0 build passes.
- Python tools compile.
- J-Link downloads the CANopen debug firmware.
- COM9 shows Modbus and CANopen debug fields.
- COM10 virtual ADC receives ECU function 04 requests and returns changing channel values.
- CAN analyzer shows command frames only after a debug command sequence is changed.
