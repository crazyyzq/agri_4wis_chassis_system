# 2026-06-30 Whole Vehicle Communication Test Log

Test time: 2026-06-30 20:22:13 +08:00

## Test boundary

This test verified whole-vehicle communication paths only. No servo enable, no speed command, no steering command, no lift command, no hydraulic command and no brake-release command were sent.

The commissioning high-voltage debug request was enabled only to bring the drive-side communication power into the normal vehicle state. The command path forced:

- speed: 0.00 kph
- steering: 0.00 deg
- brake release: disabled
- hydraulic enable: disabled
- hydraulic valve mask: 0

After the test, the commissioning request was disabled and COM9 confirmed:

```text
[ECU POWER] hv_req=0 ... online[bms=1 dcdc48=1 dcdc12=1 dcac=1] ...
[ECU CMD] ... brake=0 hv=0 comm_hv=0 hyd=0 valve=0x00000000 ...
```

## Hardware connections during this test

- J-Link: connected to ECU CPU0, VTref = 3.270 V.
- COM9: ECU debug UART, 115200 baud.
- CAN analyzer CAN1: connected to ECU CAN1, 250 kbit/s.
- CAN analyzer CAN2: temporarily moved from ECU CAN2 to ECU CAN3, tested at 1 Mbit/s, 500 kbit/s and 250 kbit/s.
- COM12: connected to ECU RS485_1 through USB-RS485. The actual Modbus result was judged from ECU COM9 runtime logs to avoid transmitting from the PC side into the same bus.
- Remote controller / SBUS receiver: not connected in this test.
- Warning light on RS485_2: not connected in this test.

## Firmware and static verification

Firmware branch/worktree state contains the latest commissioning/debug changes for:

- CAN2 CANopen scan: nodes 1, 2, 3, 4, 5, 6, 7, 8.
- CAN3 CANopen scan: nodes 9, 11, 12, 10, 13.
- Lift node mapping:
  - leg 1 lift: node 9
  - leg 2 lift: node 11
  - leg 3 lift: node 12
  - leg 4 lift: node 10
  - hydraulic station servo: node 13
- Commissioning high-voltage request guarded by `ECU_COMMISSIONING_CONTROL_MAGIC`.

Static tests:

```text
python tests\python\run_tests.py
All 59 tests passed.
```

Build check:

```text
D:\agri_4wis_chassis_system\ecu\sdk_env_v1.11.0\tools\cmake\bin\cmake.exe --build tmp\cmake_cpu0_canopennode --target all
ninja: no work to do.
```

No new compiler warning output was produced by this build command.

## Result summary

| Interface | Result | Evidence | Notes |
|---|---:|---|---|
| CAN1 power bus | PASS for communication | Analyzer CAN1 captured 3769 frames in 15 s. COM9 showed BMS, DCDC48, DCDC12 and DCAC online. | DCDC48 still reports `err=152` and `lv_ok=0`; this is a device/power-status issue, not a CAN silence issue. |
| CAN2 drive/steer CANopen | PASS | COM9 showed CANopen normal state, 1 Mbit/s, SDO reads increasing, `sdo_abort=0`, statusword reads from nodes 1..8. | Analyzer was no longer connected to ECU CAN2 during this test, so this judgment is from ECU runtime logs. |
| CAN3 lift/hydraulic CANopen | FAIL | Analyzer CAN2, connected to ECU CAN3, captured 0 frames at 1 Mbit/s, 500 kbit/s and 250 kbit/s. COM9 showed CAN3 CANopen initialized/normal, but all SDO reads timed out with abort `0x05040000`. | Because the analyzer saw 0 physical frames, this is earlier than "node did not reply"; first check ECU CAN3 physical mapping, transceiver enable/power, CANH/CANL, ground and termination. |
| RS485_1 Modbus ADC | PASS | COM9 showed `online=1`, `timeout=0`, `err=0`, continuously increasing tx/rx counters and valid mV data. | Sample mV values were approximately `[4341..4348, 4458..4464, 826, 157, 867, 0, 0, 0]`. |
| SBUS remote | NOT TESTED | COM9 showed `sbus_conn=0`, `frames=0`, `DIAG_REMOTE_ESTOP_SBUS_TIMEOUT`. | Expected because the remote controller / receiver was not connected for this test. |
| RS485_2 warning light | NOT TESTED | Device not connected. | Test after warning light is connected. |

## Key captured evidence

### J-Link

```text
J-Link V9.16
Detected: RV32 core
VTref=3.270V
```

### ECU command safety state during high-voltage communication test

```text
[ECU CMD] src=safety mode=ackermann gear=P speed=0.00kph steer=[0.00,0.00,0.00,0.00]deg brake=0 hv=1 comm_hv=1 hyd=0 valve=0x00000000 res[pwr=ok mot=ok lift=ok io=ok warn=ok]
```

### CAN1 analyzer capture

15 s capture on analyzer channel 0, 250 kbit/s:

```text
CHANNEL_0_TOTAL=3769
CH0 ID=0x1410F41E CNT=715 ext DLC=8 DATA=00 00 01 00 00 00 00 02
CH0 ID=0x18F8622B CNT=450 ext DLC=8 DATA=B8 47 00 00 00 00 98 1A
CH0 ID=0x18111EF4 CNT=300 ext DLC=8 DATA=90 E9 05 36 4E 56 00 03
CH0 ID=0x18161EF4 CNT=300 ext DLC=8 DATA=42 59 58 43 42 59 58 43
CH0 ID=0x18171EF4 CNT=300 ext DLC=8 DATA=FC E7 7A 24 F8 77 D1 07
CH0 ID=0x02002000 CNT=150 ext DLC=8 DATA=00 00 00 00 10 00 20 30
CH0 ID=0x02002001 CNT=150 ext DLC=8 DATA=00 00 00 00 10 00 20 30
CH0 ID=0x02002002 CNT=150 ext DLC=8 DATA=00 00 00 00 10 00 20 30
CH0 ID=0x02003000 CNT=150 ext DLC=8 DATA=00 20 00 01 64 64 5E 47
CH0 ID=0x02003001 CNT=150 ext DLC=8 DATA=00 20 00 01 64 64 56 47
CH0 ID=0x02003002 CNT=150 ext DLC=8 DATA=00 20 00 01 64 64 62 47
```

### CAN2 drive/steer CANopen runtime evidence

COM9 excerpts:

```text
[ECU CANopen CAN2] init=1 state=2 normal=1 bitrate=1000000 local=127 remote=1 ... sdo_ok=11799 sdo_abort=0 last_node=6 last=0x6041:0 size=2 value=0x00000437 abort=0x00000000 err=0
[ECU CANopen CAN2] init=1 state=2 normal=1 bitrate=1000000 local=127 remote=1 ... sdo_ok=11817 sdo_abort=0 last_node=2 last=0x6041:0 size=2 value=0x00000470 abort=0x00000000 err=0
[ECU CANopen CAN2] init=1 state=2 normal=1 bitrate=1000000 local=127 remote=1 ... sdo_ok=12177 sdo_abort=0 last_node=2 last=0x6041:0 size=2 value=0x00000470 abort=0x00000000 err=0
```

### CAN3 lift/hydraulic CANopen evidence

COM9 excerpts:

```text
[ECU CANopen CAN3] init=1 state=2 normal=1 bitrate=1000000 local=127 remote=9 ... sdo_ok=0 sdo_abort=3256 last_node=11 last=0x6041:0 size=0 value=0x00000000 abort=0x05040000 last_err=-9
[ECU CANopen CAN3] init=1 state=2 normal=1 bitrate=1000000 local=127 remote=9 ... sdo_ok=0 sdo_abort=3273 last_node=10 last=0x6041:0 size=0 value=0x00000000 abort=0x05040000 last_err=-9
[ECU CANopen CAN3] init=1 state=2 normal=1 bitrate=1000000 local=127 remote=9 ... sdo_ok=0 sdo_abort=3374 last_node=13 last=0x6041:0 size=0 value=0x00000000 abort=0x05040000 last_err=-9
```

Analyzer CAN2 channel, physically connected to ECU CAN3:

```text
CAN3_ANALYZER_CHANNEL1 bitrate=1000000 total=0
CAN3_ANALYZER_CHANNEL1 bitrate=500000 total=0
CAN3_ANALYZER_CHANNEL1 bitrate=250000 total=0
```

This means the analyzer did not see ECU CAN3 SDO requests such as `0x609`, `0x60A`, `0x60B`, `0x60C`, or `0x60D`.

## CAN3 failure analysis

Current firmware maps the lift/hydraulic CANopen network to:

```text
BOARD_CAN3_BASE = HPM_CAN2
BOARD_CAN3_IRQn = IRQn_CAN2
pinmux: PY11 = CAN2_RXD, PY10 = CAN2_TXD
```

The same pinmux file also configures:

```text
PA29 = CAN3_RXD
PA30 = CAN3_TXD
```

Therefore the immediate hardware check is:

1. Confirm which MCU pins the ECU connector labeled CAN3 actually uses.
   - If ECU CAN3 uses PY10/PY11, the code is targeting the intended peripheral and the next checks are transceiver power/standby, CANH/CANL, ground and termination.
   - If ECU CAN3 uses PA29/PA30, the current lift/hydraulic network is one hardware port off; the code must map this network to `BOARD_CAN4_BASE = HPM_CAN3` instead of `BOARD_CAN3_BASE = HPM_CAN2`.
2. Confirm the analyzer CAN2 channel is connected to the same physical CANH/CANL pair as the lift/hydraulic drives.
3. Confirm the ECU CAN3 transceiver enable/standby pin is not held disabled by hardware.
4. Confirm bus resistance with power off.
   - Approximately 60 ohm if both ends are terminated.
   - Approximately 120 ohm if only one termination is present.
   - Open/high resistance indicates wiring or termination path issue.
5. After physical frames are visible, expected CANopen request/response IDs are:
   - ECU SDO requests: `0x609`, `0x60A`, `0x60B`, `0x60C`, `0x60D`
   - Drive SDO responses: `0x589`, `0x58A`, `0x58B`, `0x58C`, `0x58D`

## Next action

Do not debug this as a CANopen protocol problem yet. The analyzer currently sees 0 frames on ECU CAN3, so the next step is to confirm whether the ECU connector called CAN3 is wired to PY10/PY11 or PA29/PA30.

Once that is confirmed:

- If it is PY10/PY11: continue physical-layer debugging on HPM_CAN2 / CAN2.F.
- If it is PA29/PA30: remap the lift/hydraulic CANopen service from `BOARD_CAN3_BASE` to `BOARD_CAN4_BASE`, rebuild, download and retest.

## 2026-07-01 retest after CAN transceiver rework and firmware re-download

Hardware connection:

- CAN analyzer CAN1: ECU CAN3.
- CAN analyzer CAN2: ECU CAN4.
- J-Link: connected and used to re-download `tmp\cmake_cpu0_canopennode\output\demo.elf`.
- Debug UART: not connected, so this retest used only J-Link and CAN analyzer evidence.

Build/download evidence:

```text
D:\agri_4wis_chassis_system\ecu\sdk_env_v1.11.0\tools\cmake\bin\cmake.exe --build tmp\cmake_cpu0_canopennode --target all
ninja: no work to do.

J-Link V9.16
VTref=3.277V
Downloading file [D:\agri_4wis_chassis_system\tmp\cmake_cpu0_canopennode\output\demo.elf]...
O.K.
```

CAN analyzer result at 1 Mbit/s:

```text
ANALYZER_CAN1_ECU_CAN3_TOTAL=250
ANALYZER_CAN1_ECU_CAN3 ID=0x609 CNT=50 std DLC=8 DATA=40 41 60 00 00 00 00 00
ANALYZER_CAN1_ECU_CAN3 ID=0x60A CNT=50 std DLC=8 DATA=40 41 60 00 00 00 00 00
ANALYZER_CAN1_ECU_CAN3 ID=0x60B CNT=50 std DLC=8 DATA=40 41 60 00 00 00 00 00
ANALYZER_CAN1_ECU_CAN3 ID=0x60C CNT=50 std DLC=8 DATA=40 41 60 00 00 00 00 00
ANALYZER_CAN1_ECU_CAN3 ID=0x60D CNT=50 std DLC=8 DATA=40 41 60 00 00 00 00 00
ANALYZER_CAN2_ECU_CAN4_TOTAL=0
```

Retest conclusion:

- ECU CAN3 physical transmit is working after rework and firmware re-download.
- The observed CAN3 frames are CANopen SDO upload requests for object `0x6041:00` from nodes 9, 10, 11, 12 and 13.
- ECU CAN4 still shows 0 frames, but the current main control firmware does not bind a periodic service to CAN4. This result only means "current firmware is not transmitting on CAN4"; it does not prove CAN4 hardware failure.

## 2026-07-01 CAN4 physical transmit test

Firmware change for this test:

- Added `ECU_ENABLE_CAN4_PHYSICAL_TEST_TX`.
- Added CAN4 hardware binding through `BOARD_CAN4_BASE = HPM_CAN3`.
- CAN4 bitrate: `ECU_CAN4_AUXILIARY_BITRATE = 500000`.
- CAN4 termination macro enabled: `ECU_CAN4_TERMINATION_ENABLE = 1`.
- Test frame:
  - ID: `0x444`
  - Type: standard data frame
  - DLC: 8
  - Period: 500 ms
  - Payload starts with `0xC4`; remaining bytes contain sequence/time marker and `55 AA`.

Static and build verification:

```text
python tests\python\run_tests.py
All 59 tests passed.

D:\agri_4wis_chassis_system\ecu\sdk_env_v1.11.0\tools\cmake\bin\cmake.exe --build tmp\cmake_cpu0_canopennode --target all
Linking C executable output\demo.elf
```

Download evidence:

```text
J-Link V9.16
VTref=3.283V
Downloading file [D:\agri_4wis_chassis_system\tmp\cmake_cpu0_canopennode\output\demo.elf]...
O.K.
```

Analyzer connection:

- Analyzer CAN1: ECU CAN3, 1 Mbit/s.
- Analyzer CAN2: ECU CAN4, 500 kbit/s.

Analyzer result:

```text
ANALYZER_CAN1_ECU_CAN3_1M_TOTAL=250
ANALYZER_CAN1_ECU_CAN3_1M ID=0x609 CNT=50 std DLC=8 DATA=40 41 60 00 00 00 00 00
ANALYZER_CAN1_ECU_CAN3_1M ID=0x60A CNT=50 std DLC=8 DATA=40 41 60 00 00 00 00 00
ANALYZER_CAN1_ECU_CAN3_1M ID=0x60B CNT=50 std DLC=8 DATA=40 41 60 00 00 00 00 00
ANALYZER_CAN1_ECU_CAN3_1M ID=0x60C CNT=50 std DLC=8 DATA=80 41 60 00 00 00 04 05
ANALYZER_CAN1_ECU_CAN3_1M ID=0x60D CNT=50 std DLC=8 DATA=40 41 60 00 00 00 00 00
ANALYZER_CAN2_ECU_CAN4_500K_TOTAL=0
```

J-Link runtime memory evidence for CAN4 service:

```text
CAN4 service bitrate = 0x0007A120 = 500000
CAN4 service online = 1
CAN4 service tx_count = 9
CAN4 service error_count = 0x0000019E
CAN4 last_tx_frame.id = 0x444
```

CAN4 conclusion:

- CPU0 firmware reached the CAN4 transmit path and queued test frame `0x444`.
- Analyzer CAN2, connected to ECU CAN4 at 500 kbit/s, captured 0 frames.
- The CAN4 controller accumulated errors while attempting to transmit.

Current diagnosis:

The CAN4 test fails at the physical bus layer or at the ECU connector/peripheral mapping layer. The failure is not caused by the CPU0 task not running or by the CAN4 test frame not being requested in software.

Recommended next checks:

1. Confirm ECU CAN4 connector pins against `PA29/PA30 = CAN3.B RX/TX` and `BOARD_CAN4_BASE = HPM_CAN3`.
2. Check CAN4 transceiver VCC, VIO, GND and standby/enable level.
3. Check MCU TX pin to transceiver TXD continuity and transceiver RXD to MCU RX pin continuity.
4. Check CAN4 CANH/CANL continuity to the connector.
5. Measure CAN4 bus resistance with power off.
6. If possible, probe CAN4 TXD at the transceiver input while firmware is running; if TXD toggles but CANH/CANL do not, the fault is transceiver-side. If TXD does not toggle, the fault is MCU pinmux/peripheral mapping or PCB routing.

## 2026-07-01 CAN4 retest after CAN4 transceiver rework

After reworking the CAN4 transceiver soldering, the same firmware was rebuilt, re-downloaded and retested.

Verification before download:

```text
python tests\python\run_tests.py
All 59 tests passed.

D:\agri_4wis_chassis_system\ecu\sdk_env_v1.11.0\tools\cmake\bin\cmake.exe --build tmp\cmake_cpu0_canopennode --target all
Linking C executable output\demo.elf
```

Download evidence:

```text
J-Link V9.16
VTref=3.283V
Downloading file [D:\agri_4wis_chassis_system\tmp\cmake_cpu0_canopennode\output\demo.elf]...
O.K.
```

Analyzer connection:

- Analyzer CAN1: ECU CAN3 at 1 Mbit/s.
- Analyzer CAN2: ECU CAN4 at 500 kbit/s.

Analyzer result:

```text
ANALYZER_CAN1_ECU_CAN3_1M_TOTAL=250
ANALYZER_CAN1_ECU_CAN3_1M ID=0x609 CNT=50 std DLC=8 DATA=40 41 60 00 00 00 00 00
ANALYZER_CAN1_ECU_CAN3_1M ID=0x60A CNT=50 std DLC=8 DATA=40 41 60 00 00 00 00 00
ANALYZER_CAN1_ECU_CAN3_1M ID=0x60B CNT=50 std DLC=8 DATA=40 41 60 00 00 00 00 00
ANALYZER_CAN1_ECU_CAN3_1M ID=0x60C CNT=50 std DLC=8 DATA=40 41 60 00 00 00 00 00
ANALYZER_CAN1_ECU_CAN3_1M ID=0x60D CNT=50 std DLC=8 DATA=40 41 60 00 00 00 00 00
ANALYZER_CAN2_ECU_CAN4_500K_TOTAL=30
ANALYZER_CAN2_ECU_CAN4_500K ID=0x444 CNT=30 std DLC=8 DATA=C4 21 91 40 00 00 55 AA
```

Retest conclusion:

- ECU CAN4 physical transmit is now working.
- The captured count matches the configured 500 ms transmit period: 30 frames in 15 s.
- ECU CAN3 remained normal during the same capture.
- Runtime monitor was extended with `[ECU CAN4 TEST]` fields so future serial-console testing can see CAN4 tx/error/last-frame data without reading RAM through J-Link.

## 2026-07-01 commissioning debug code cleanup and regression test

After CAN4 passed, the commissioning-only helpers were moved out of the CPU0 task orchestrator to avoid accumulating debug code in `ecu_tasks_cpu0.c`.

Code structure after cleanup:

- `ecu/diag/include/commissioning_debug.h`
- `ecu/diag/src/commissioning_debug.c`

The new module owns:

- guarded J-Link/Watch high-voltage communication request
- read-only CANopen SDO scan for CAN2 and CAN3 nodes
- CAN4 physical-layer test frame generation
- CAN4 test snapshot for runtime monitor output

`ecu/os/src/ecu_tasks_cpu0.c` now only calls the commissioning module from the relevant task steps. It no longer directly implements:

- CAN4 test frame construction
- CAN2/CAN3 commissioning scan selection
- high-voltage commissioning guard logic

Regression verification:

```text
python tests\python\run_tests.py
All 60 tests passed.

D:\agri_4wis_chassis_system\ecu\sdk_env_v1.11.0\tools\cmake\bin\cmake.exe --build tmp\cmake_cpu0_canopennode --target all
Linking C executable output\demo.elf
```

Download and hardware regression:

```text
J-Link V9.16
VTref=3.283V
Downloading file [D:\agri_4wis_chassis_system\tmp\cmake_cpu0_canopennode\output\demo.elf]...
O.K.

ANALYZER_CAN1_ECU_CAN3_1M_TOTAL=250
ANALYZER_CAN1_ECU_CAN3_1M ID=0x609 CNT=50 std DLC=8 DATA=40 41 60 00 00 00 00 00
ANALYZER_CAN1_ECU_CAN3_1M ID=0x60A CNT=50 std DLC=8 DATA=40 41 60 00 00 00 00 00
ANALYZER_CAN1_ECU_CAN3_1M ID=0x60B CNT=50 std DLC=8 DATA=80 41 60 00 00 00 04 05
ANALYZER_CAN1_ECU_CAN3_1M ID=0x60C CNT=50 std DLC=8 DATA=40 41 60 00 00 00 00 00
ANALYZER_CAN1_ECU_CAN3_1M ID=0x60D CNT=50 std DLC=8 DATA=40 41 60 00 00 00 00 00
ANALYZER_CAN2_ECU_CAN4_500K_TOTAL=30
ANALYZER_CAN2_ECU_CAN4_500K ID=0x444 CNT=30 std DLC=8 DATA=C4 1E AF 3A 00 00 55 AA
```

Cleanup conclusion:

- Refactor preserved CAN3 and CAN4 hardware behavior.
- `ecu_tasks_cpu0.c` reduced to 742 lines.
- Commissioning debug code is isolated in a dedicated module with contract-test coverage.
