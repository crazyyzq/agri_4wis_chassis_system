# ECU Configuration Open Items

This file records project defaults that need final confirmation from vehicle wiring, supplier manuals, or calibration tests. Code symbols stay professional; uncertainty is tracked here.

## CAN buses

- CAN1 power bus: default `250 kbit/s`.
  - Scope: BMS, DCDC, DCAC/inverter and other power devices on the power bus.
  - Source: `doc/电池BMS/科易动力BMS与整车通信协议V1.7-20250804-KPB20.pdf`, page 6.
  - Note: the BMS protocol table lists `bit time 4 us`, which corresponds to 250 kbit/s. The same table also allows the vehicle project to select 250/500 kbit/s, so final vehicle-wide power-bus bitrate still needs confirmation if another power device requires 500 kbit/s.
- CAN2 motion bus: default `1 Mbit/s`.
  - Scope: BC/BC2 drive and steering devices.
  - Hardware debug: CANopenNode CAN2 command path has verified BC2 SDO downloads and NMT commands with auxiliary power only. Main-power and motor-motion behavior still needs controlled validation.
- CAN3 lift/hydraulic bus: project default `500 kbit/s`.
- CAN4 auxiliary bus: project default `500 kbit/s`.

## Items still requiring supplier/manual confirmation

- BMS, DCDC and DCAC CAN node IDs, heartbeat IDs, status words and control words.
- BC/BC2 PDO mapping, CiA 402 object dictionary details, scaling and enable sequence. Current CANopen command-debug firmware has verified NMT and SDO writes on a BC2 drive under auxiliary power; main-power/motor behavior still needs controlled validation.
- Lift servo PDO mapping, scaling and enable sequence.
- Relay/MOS digital output polarity and final managed-output bit masks.
- Hydraulic valve bit mapping and electrical interlock requirements.
- ADC module channel order and final analog scaling per sensor. Current default follows the 8AI module: slave 1, 9600 baud, function 04, input registers 0..7, raw 0..65535 to 0..5000 mV.
- Warning-light RS485 Modbus address and supported lamp modes for the exact installed model. Current default uses slave `0xFF`, direct-control register `0x00C2` and named values from the supplied manual.
