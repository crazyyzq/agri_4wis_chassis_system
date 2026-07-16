# ECU Configuration Open Items

This file records project defaults that need final confirmation from vehicle wiring, supplier manuals, or calibration tests. Code symbols stay professional; uncertainty is tracked here.

> Updated 2026-07-15. Items marked “measured” are implemented configuration
> baselines, not blanket vehicle acceptance. Current source-of-truth priority is
> `ecu/config/include/ecu_config.h`, then board configuration, vendor manuals,
> analyzer evidence and finally explicitly labeled calibration items.

## CAN buses

- CAN1 power bus: default `250 kbit/s`.
  - Scope: BMS, DCDC, DCAC/inverter and other power devices on the power bus.
  - Sources: BMS, DCDC48, DCDC12 and DCAC supplier protocol documents under `doc/`.
  - Note: the BMS protocol table lists `bit time 4 us`, which corresponds to 250 kbit/s. The DCDC and DCAC documents also use 250 kbit/s, so CAN1 defaults to 250 kbit/s.
- CAN2 motion bus: default `1 Mbit/s`.
  - Scope: BC/BC2 drive and steering devices.
  - Measured baseline: Node1–8 use the saved `current7 + sync1` PDO profile;
    drive velocity, steering position and TPDO feedback have been exercised on
    the vehicle. The current Node1–8 automatic-recovery implementation still
    requires deliberate live fault-injection validation before it is accepted.
- CAN3 lift/hydraulic bus: project default `1 Mbit/s`.
  - Scope: BC/BC2 lift servos and the hydraulic station servo.
  - Measured baseline: analyzer tests established the four-axis RPDO2/SYNC
    interpolation procedure and Node13 reverse-only pump direction. ECU-side
    full-stroke lift behavior remains a field-validation item.
- CAN4 auxiliary bus: project default `500 kbit/s`.

## Frozen encoder position units

- Node1–8 drive/steering: `10000 counts/rev`.
- Node9–12 lift: `131072 counts/rev`.
- Node13 hydraulic pump: `10000 counts/rev`.

These are implemented role contracts, not open calibration items. The installed
lift reducer is currently `20 motor rev/10 mm` (`262144 count/mm`), with manual
mechanical zero retained by the drives. Loaded-machine behavior, steering zero
repeatability and final sensor calibration remain field items.

## Items requiring vehicle calibration

- Remote manual driving limits:
  - `ECU_REMOTE_MAX_SPEED_KPH`
  - `ECU_REMOTE_MAX_STEER_DEG`
  - `ECU_REMOTE_MIN_HEIGHT_TARGET_MM`
  - `ECU_REMOTE_MAX_HEIGHT_TARGET_MM`
  - `ECU_REMOTE_MAX_HEIGHT_RATE_MM_S`
  - `ECU_REMOTE_MAX_TRACK_RATE_MM_S`
  - These defaults are safe software calibration values and should be matched to the chassis geometry, actuator limits and test-site safety plan.
- DCDC48 command setpoints: current default follows the document example, `14.0 V` and `20.0 A`, controlled by `ECU_DCDC48_DEFAULT_TERMINAL_VOLTAGE_DV` and `ECU_DCDC48_DEFAULT_CURRENT_DA`.
- DCDC12 command setpoints: current default is `13.8 V` and `100.0 A`, controlled by `ECU_DCDC12_DEFAULT_OUTPUT_VOLTAGE_DV` and `ECU_DCDC12_DEFAULT_OUTPUT_CURRENT_DA`; verify against the installed converter rating before enabling load.
- DCAC output voltage: current default is `220.0 V`, controlled by `ECU_DCAC_DEFAULT_OUTPUT_VOLTAGE_DV`.
- BC/BC2 PDO mapping is no longer open: Node1–13 use the saved `current7 + sync1` contract in `doc/CANOPEN_PDO_MAPPING_RECORD_V1.md`. Changes require an offline analyzer procedure, readback and focused tests.
- Drive/steering/lift conversion scales are implemented from measured drivetrain data. Lift normal control is 10–490 mm at 20 mm/s and 8 mm/s²; out-of-band movement is permitted only toward the safe band. The analyzer completed ten 10→490→10 mm cycles, but ECU-owned remote motion still requires loaded-vehicle validation of immediate neutral stop, deterministic same/opposite-direction restart, synchronous enable, starvation recovery, steering zero repeatability and track-width sensor scaling.
- Relay/MOS output polarity is configured from current board wiring; each final relay/MOS mapping still needs an installation-specific electrical continuity check.
- Hydraulic valve mapping and pairwise interlocks are implemented: 1/2 front suspension, 3/4 track width, 5/6 rear suspension. Final cylinder direction and sensor calibration still require loaded-machine validation.
- ADC module channel order and final analog scaling per sensor. Current default follows the 8AI module: slave 1, 9600 baud, function 04, input registers 0..7, raw 0..65535 to 0..5000 mV.
- Warning-light RS485 Modbus address and supported lamp modes for the exact installed model. Current default uses 9600 8N1, slave `0xFF`, function 06, direct-control register `0x00C2` and named values from the supplied manual.
