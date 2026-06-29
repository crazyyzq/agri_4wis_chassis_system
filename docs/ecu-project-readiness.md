# ECU Project Readiness

Last updated: 2026-06-28

This document states the current engineering boundary of the ECU main-branch firmware. It separates verified software and bench-debug capability from vehicle items that still require supplier protocol details or live hardware validation.

## Current readiness level

The project is a complete staged ECU control framework for the current bench-validation phase:

- CPU0 owns safety-critical state, remote interpretation, arbitration, safety clamping and final actuator fan-out.
- CPU1 is kept outside the safety-critical output path.
- SBUS decoding, UART idle-interrupt reception, 16-channel runtime reporting and failsafe timing are present.
- RS485_1 Modbus master support for the 8-channel analog module is present and reports raw values plus millivolts.
- CAN2 CANopenNode diagnostic support for BC/BC2 is present, including gated NMT and SDO command-debug control through `g_canopen_master_debug_control`.
- CAN1 supplier power-bus support for BMS, DCDC48, DCDC12 and DCAC is present with 29-bit extended frames at 250 kbit/s.
- Device adapters are separated from remote/control logic. The vehicle executor is the only final command exit.
- Hardware-dependent values are centralized in `ecu/config` and open calibration items are tracked in `docs/ecu-configuration-open-items.md`.
- CPU0 debug UART reporting includes SBUS, CAN2/CANopen, Modbus ADC, hardware feedback and final command state.

## Safety boundary

Unconfigured supplier protocols are not treated as active.

- CAN1 power/BMS/DCDC/DCAC defaults to `ECU_POWER_PROTOCOL_SUPPLIER_CAN`.
- A high-voltage ON request sends BMS contactor connect plus DCDC/DCAC enable commands only after safety clamping sets `final_command.high_voltage_enable`.
- Without high-voltage request, the CAN1 power task continues sending documented disconnect/stop commands.
- CPU0 remote preconditions read from `ecu_hardware_feedback_snapshot_t`; power-ready, low-voltage-ok and CAN1-online are no longer hardcoded true.
- Unknown feedback starts safe-false, except static safe observations such as initial zero-speed and hydraulic-stopped.

## Remaining work before real vehicle operation

- Calibrate final DCDC48, DCDC12 and DCAC setpoints on the real vehicle.
- Validate normal motion output on CAN2 and lift output on CAN3 through the CANopenNode SDO command queue, then optimize selected high-rate objects with confirmed PDO mapping if the BC/BC2 EDS requires it.
- Connect CAN3/CAN4 hardware TX/RX/ISR paths and their target device protocols.
- Bind CPU0/CPU1 IPC to the final SDK multicore transport.
- Calibrate BC/BC2 scaling, steering zero offsets, wheel directions, lift scaling, hydraulic valve bits, relay polarity and analog channel order on the real machine.
- Add live-hardware validation records for each confirmed power, hydraulic and lift protocol before vehicle operation.

## Verification expected before commit

Run these checks before publishing framework changes:

```powershell
python tests\python\run_tests.py
python tools\check_no_forbidden_patterns.py
python -m py_compile tools\can\controlcan.py tools\can\can2_monitor.py tools\modbus\rtu_probe.py tools\modbus\rtu_codec.py tools\modbus\virtual_adc_module.py
```

Build CPU0 with the default CANopenNode-enabled configuration, and build CPU1 if its project files are present. When hardware is connected, download the selected CPU0 image through J-Link and confirm COM9 boot/runtime output.
