# ECU Configuration Open Items

This file records project defaults that need final confirmation from vehicle wiring, supplier manuals, or calibration tests. Code symbols stay professional; uncertainty is tracked here.

## CAN buses

- CAN1 power bus: default `250 kbit/s`.
  - Scope: BMS, DCDC, DCAC/inverter and other power devices on the power bus.
  - Sources: BMS, DCDC48, DCDC12 and DCAC supplier protocol documents under `doc/`.
  - Note: the BMS protocol table lists `bit time 4 us`, which corresponds to 250 kbit/s. The DCDC and DCAC documents also use 250 kbit/s, so CAN1 defaults to 250 kbit/s.
- CAN2 motion bus: default `1 Mbit/s`.
  - Scope: BC/BC2 drive and steering devices.
  - Hardware debug: CANopenNode CAN2 command path has verified BC2 SDO downloads and NMT commands with auxiliary power only. Main-power and motor-motion behavior still needs controlled validation.
- CAN3 lift/hydraulic bus: project default `1 Mbit/s`.
  - Scope: BC/BC2 lift servos and the hydraulic station servo.
  - Source: current whole-vehicle wiring uses BC/BC2 CANopen devices on CAN3.
- CAN4 auxiliary bus: project default `500 kbit/s`.

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
- DCDC12 command setpoints: current default follows the document example, `27.5 V` and `10.0 A`, controlled by `ECU_DCDC12_DEFAULT_OUTPUT_VOLTAGE_DV` and `ECU_DCDC12_DEFAULT_OUTPUT_CURRENT_DA`.
- DCAC output voltage: current default is `220.0 V`, controlled by `ECU_DCAC_DEFAULT_OUTPUT_VOLTAGE_DV`.
- BC/BC2 PDO mapping, CiA 402 object dictionary details, scaling and enable sequence. Current CANopen command-debug firmware has verified NMT and SDO writes on a BC2 drive under auxiliary power; main-power and motor-motion behavior still needs controlled validation.
- Drive, steering and lift conversion scales. The four-wheel kinematic model now computes per-wheel angles and wheel speeds from wheelbase, track width and ICR geometry, but the final counts-per-engineering-unit scale factors still require controlled vehicle calibration.
- Lift servo PDO mapping, scaling and enable sequence.
- Relay/MOS digital output polarity and final managed-output bit masks.
- Hydraulic valve bit mapping and electrical interlock requirements.
- ADC module channel order and final analog scaling per sensor. Current default follows the 8AI module: slave 1, 9600 baud, function 04, input registers 0..7, raw 0..65535 to 0..5000 mV.
- Warning-light RS485 Modbus address and supported lamp modes for the exact installed model. Current default uses 9600 8N1, slave `0xFF`, function 06, direct-control register `0x00C2` and named values from the supplied manual.
