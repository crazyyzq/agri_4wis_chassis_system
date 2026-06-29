# ECU Main Control Framework Progress

Last updated: 2026-06-29

This file records the durable implementation state for the main-branch control framework so the project can be resumed after context compression or a new development session.

## Baseline

- Hardware bring-up and board-level tests for RS485, RS232, CAN1-CAN4, SBUS, digital I/O and analog input have been completed before this framework work.
- Ethernet and CAN-FD are intentionally out of scope for the current software framework.
- The main project target is the HPM6750 ECU based on the MR6750 module and the HPM SDK 1.11 environment.
- The main framework is being written on `main`, not on the previous board-test branch.

## Requirement source

- Primary requirement document: `doc/ECU_Project_Implementation_v1.4.md`
- Main framework plan: `docs/superpowers/plans/2026-06-27-ecu-main-control-framework.md`
- Hardware adapter framework plan: `docs/superpowers/plans/2026-06-27-ecu-full-hardware-framework.md`

## Implemented framework scope

- Shared project primitives:
  - ECU status/result types.
  - Stable millisecond timestamp type.
  - Central configuration constants for timing, SBUS thresholds, failsafe limits and arbitration priorities.
  - Diagnostic code catalog with readable text.
- SBUS layer:
  - Pure SBUS frame decoder.
  - SBUS service state that can be fed from a UART idle-interrupt path.
  - Snapshot API for the rest of the project.
  - CPU0 SBUS UART1 hardware binding is now present through `sbus_uart_hw`: 100000 baud, 8E2, RX data/timeout interrupt, RX line idle interrupt, UART error accounting and 16-channel debug monitor output.
- Remote-control layer:
  - Link qualification and failsafe FSM.
  - Arm/neutral FSM.
  - Emergency-stop latch/reset FSM.
  - Gear FSM.
  - Mode/domain FSM with guard time.
  - Adjustment FSM with owner arbitration.
  - Lights/indicator FSM.
  - Remote manager that rebuilds a full remote request each cycle.
- Vehicle-control layer:
  - Command source arbitration.
  - Safety clamping.
  - Command execution boundary.
  - Device-adapter fan-out for the final command.
  - Vehicle runtime state snapshot.
- Control layer:
  - Motion command planner.
  - Adjustment command planner.
- IPC layer:
  - CPU0 to CPU1 snapshot contract.
  - CPU1 to CPU0 snapshot contract.
- FreeRTOS app skeleton:
  - CPU0 application owns command arbitration, safety and actuator execution.
  - CPU1 application owns sensing/data aggregation snapshots only.
  - CPU0 and CPU1 task implementations are split so CPU1 does not include safety-critical executor code.
- HPM SDK build integration:
  - CPU0 CMake/SES project generation passed.
  - CPU1 CMake/SES project generation passed.
  - CPU0 Ninja build passed and produced `demo.elf`.
  - CPU1 Ninja build passed and produced `demo.elf`.
- Configurable hardware adapter framework:
  - CAN bitrates, CANopen node IDs, PDO COB-ID bases, DIO masks, hydraulic valve masks, ADC scaling, Modbus addresses and UART baud rates are centralized behind `ECU_*` macros and `ecu_hardware_config_default()`. Calibration-open values are tracked in `docs/ecu-configuration-open-items.md`.
  - CPU0 enables HPM SDK `CANopenNode` by default for both CAN2 motion and CAN3 lift/hydraulic CANopen networks. The project-generated DS301 minimum OD from `doc/ECU/DS301_OD` is used by the CANopenNode service.
  - CPU0 enables HPM SDK `agile_modbus` RTU. RS485 devices call Agile Modbus serialization and confirmation parsing directly; UART services only own transport, timeout and accounting.
  - RS485_1/UART11 is connected to a foreground Modbus master service for the 8-channel analog acquisition module. The ISR only buffers bytes; the IO task owns request timing, response timeout and ADC snapshot updates.
  - `servo_drive_canopen` now provides the first device-level CiA 402 API for control word, mode selection, target position, target velocity and target torque. `motion_device` and `lift_hydraulic_device` call this API instead of building arbitrary motor payloads directly.
  - CAN, DIO, ADC and UART service boundaries are available under `ecu/drivers`.
  - Power, motion, lift/hydraulic, local IO and warning-light device adapters are available under `ecu/devices`.
  - `vehicle_command_executor_apply()` now fans out the final command through device adapters instead of keeping the output boundary as state-only.
  - DIO active-low conversion is limited to the configured managed output mask; hydraulic valve outputs are cleared as a group before a new valve target is written.
- Control-closure hardware bindings added on 2026-06-29:
  - CPU0 owns CAN2 motion, CAN3 lift/hydraulic, DIO, RS485_1 ADC, and RS485_2 warning-light services. `vehicle_command_executor_apply()` receives these CPU0-owned services through `vehicle_executor_io_t`; it no longer creates private software-only hardware services.
  - CAN2 is TX/RX capable by default for BC/BC2 motion control at 1 Mbit/s. The older CAN2 RX-only binding remains available for passive bring-up.
  - CAN3 now has a TX/RX HPM CAN binding for lift/hydraulic commands.
  - DIO logical masks now drive the 12 isolated board outputs through `dio_hw_attach_outputs()`.
  - RS485_2/UART12 now drives the warning light through Modbus RTU function 06, slave `0xFF`, register `0x00C2`, using the direct-control values from the supplied warning-light manual.
  - SBUS input snapshots now preserve analog per-mille values for steer, throttle, clearance and track commands; R1/R2 mode requests require a fresh stable edge after the HOME domain guard.
  - CAN service TX now rejects requests when no TX backend is bound, so actuator paths cannot report success from a software-only service.
- CPU0 debug observability:
  - `ecu/diag/runtime_monitor` prints periodic CPU0 runtime status through the debug console when `ECU_ENABLE_DEBUG_MONITOR` is enabled.
  - The monitor includes `[ECU MODBUS ADC]` raw/mV values and `[ECU CANopen CMD]` command-debug counters.
  - Debug monitor period and verbosity are controlled by `ECU_DEBUG_MONITOR_PERIOD_MS` and `ECU_DEBUG_MONITOR_VERBOSE`.
  - CPU0 startup logs task creation results; malloc failure, stack overflow and unexpected scheduler return are visible on the debug console.

## Design constraints already encoded in tests

- Remote modules must not include board/HPM peripheral headers directly.
- Raw SBUS thresholds must live in configuration, not scattered through logic files.
- CPU1 code must not issue brake, hydraulic, high-voltage or actuator execution commands.
- Generic dump files such as `common.c`, `misc.c` and `utils.c` are forbidden under `ecu/`.
- Sleep calls should stay in task/application boundaries, not inside pure logic modules.
- Hardware calibration values should stay in `ecu/config`; raw CANopen IDs and similar numbers should not be scattered through device/business code.

## Next verification gate

Before any next framework commit is considered complete:

1. Run `python tests\python\run_tests.py`.
2. Run the forbidden-pattern static checker.
3. Run RISC-V GCC `-fsyntax-only` over all pure framework C modules.
4. Run SDK/CMake project generation if the local SDK build environment is available.
5. Run CPU0 and CPU1 Ninja builds.
6. Run `git diff --check`.
7. Fix every failure before committing.

## Latest verification evidence

Verified on 2026-06-27:

- `python tests\python\run_tests.py`: 22/22 tests passed.
- `python tools\check_no_forbidden_patterns.py`: no forbidden ECU framework patterns found.
- `git diff --check`: no whitespace errors; only Git line-ending warning for `ecu/apps/agri_chassis_control_cpu0/CMakeLists.txt`.
- RISC-V GCC `-fsyntax-only`: passed for framework, protocol, driver and device modules.
- CPU0 CMake/SES generation and Ninja build: passed, `tmp/cmake_cpu0_full/output/demo.elf` produced.
- CPU1 CMake/SES generation and Ninja build: passed, `tmp/cmake_cpu1_full/output/demo.elf` produced.

Verified on 2026-06-28:

- J-Link connection: detected J-Link V13, target reference voltage `VTref=3.277V`, one RV32 core in the HPM6750 JTAG chain.
- CPU0 firmware download: `loadfile tmp/cmake_cpu0_full/output/demo.elf` completed with flash erase/program/verify success.
- Debug UART: `COM9` at 115200 baud printed board clock summary, CPU0 boot banner, all CPU0 task creation lines and periodic `[ECU MON]` / `[ECU CMD]` status lines every 500 ms.
- Runtime state with no SBUS connected is safe: source `safety`, gear `P`, speed `0.00 kph`, brake release `0`, high voltage `0`, hydraulic `0`, and all software device adapters reported `ok`.

Verified later on 2026-06-28:

- `python tests\python\run_tests.py`: 26/26 tests passed.
- `python tools\check_no_forbidden_patterns.py`: no forbidden ECU framework patterns found.
- CPU0 CMake/SES generation and Ninja build: passed, `tmp/cmake_cpu0_sbus/output/demo.elf` produced.
- CPU1 CMake/SES generation and Ninja build: passed, `tmp/cmake_cpu1_sbus/output/demo.elf` produced.
- Communication protocol scope now covered in source:
  - SBUS frame decoder + UART1 interrupt binding.
  - HPM SDK CANopenNode integration path identified; build switch added, but left OFF until real project `OD.h/OD.c` are generated.
  - HPM SDK agile_modbus RTU middleware enabled for CPU0.
  - Device-level servo-drive API added on top of the raw-PDO backend boundary, ready to be backed by CANopenNode after OD generation.

Verified for CANopenNode BC2 diagnostics on 2026-06-28:

- Generated DS301 minimum OD files from `doc/ECU/DS301_OD` were copied into `ecu/protocol/canopen/od/ds301`.
- CPU0 CANopenNode diagnostic build passed with `ECU_ENABLE_CANOPENNODE=ON` and produced `tmp/cmake_cpu0_canopen/output/demo.elf`.
- J-Link download of `tmp/cmake_cpu0_canopen/output/demo.elf` succeeded through `C:\Program Files\SEGGER\JLink_V916\JLink.exe`; flash erase/program/verify completed successfully.
- COM9 debug UART confirmed CANopen diagnostic runtime: `init=1`, `normal=1`, `bitrate=1000000`, `local=127`, `remote=1`, `sdo_abort=0`.
- CAN analyzer CAN2 captured safe SDO uploads only: ECU requests on `0x601`, BC2 responses on `0x581`.
- Captured and decoded BC2 diagnostic objects:
  - `0x1000:0` device type = `0x00060192`.
  - `0x1001:0` error register = `0x50`.
  - `0x1018:1` vendor ID = `0x000000AB`.
  - `0x1018:2` product code = `0x028B`.
  - `0x1018:3` revision number = `0x00010002`.
  - `0x1018:4` serial number = `0x030DB5F8`.
  - `0x6041:0` statusword = `0x0468`.
  - `0x6061:0` mode display = `0`.
- The initial diagnostic service was read-only at this checkpoint: no NMT Operational command, no CiA 402 controlword write, no target write and no RPDO motion command were added.
- COM10 USB-RS485 enumerated as CH340. A safe Modbus RTU read probe to slave 1 received no response, and a 5-second raw receive listen saw no bytes from ECU 485_1. Current conclusion: COM10 is available, but this firmware image is not yet exercising RS485_1 as a Modbus master/slave during the CANopen diagnostic run.
- Final local verification for this checkpoint:
  - `python tests\python\run_tests.py`: 37/37 tests passed.
  - `python tools\check_no_forbidden_patterns.py`: no forbidden ECU framework patterns found.
  - `python -m py_compile tools\can\controlcan.py tools\can\can2_monitor.py tools\modbus\rtu_probe.py`: passed.
  - CPU0 default build `tmp/cmake_cpu0_sbus`: passed.
  - CPU0 CANopen diagnostic build `tmp/cmake_cpu0_canopen`: passed.
  - `git diff --check`: no whitespace errors; only Git line-ending warnings for existing LF/CRLF conversion.

Verified for Modbus ADC fixture and CANopen command-debug implementation on 2026-06-28:

- Added PC-side virtual 8AI Modbus RTU tool:
  - `tools/modbus/rtu_codec.py`
  - `tools/modbus/virtual_adc_module.py`
  - Default serial settings match the 8-channel analog acquisition module: COM10, 9600 baud, slave 1, function 04 Read Input Registers, registers 0..7.
  - The virtual tool intentionally implements the ADC read path only.
- Added ECU RS485_1 Modbus master path:
  - `ecu/drivers/uart/src/uart_rs485_hw.c`: UART11 RX interrupt buffering and TX.
  - `ecu/drivers/uart/src/modbus_master_service.c`: foreground request/response/timeout state machine.
  - `ecu/devices/src/analog_modbus_device.c`: ADC register-map adapter, raw 0..65535 to mV conversion and analog snapshot update.
  - COM9 monitor line: `[ECU MODBUS ADC] init=... tx=... rx=... timeout=... err=... raw=[...] mv=[...]`.
- Added gated CANopenNode command debug path:
  - Debug variable: `volatile canopen_master_debug_control_t g_canopen_master_debug_control`.
  - Commands execute only when `command_sequence` changes.
  - Supported commands: NMT pre-operational, operational, stopped, reset node, reset communication; SDO writes for CiA 402 controlword, mode, target position, target velocity, target torque and generic object writes.
  - COM9 monitor line: `[ECU CANopen CMD] seq=... cmd=... node=... nmt=... dl_ok=... dl_abort=... last=... abort=...`.
  - CANopen build now enables CANopenNode NMT master support through `CO_CONFIG_NMT=0x3002`; no NMT command is sent during initialization.
- Warning-light adapter was moved from the previous draft register to the documented direct-control register `0x00C2`; lamp values are named in configuration.
- Local verification for this checkpoint:
  - `python tests\python\run_tests.py`: 39/39 tests passed.
  - `python tools\check_no_forbidden_patterns.py`: no forbidden ECU framework patterns found.
  - `python -m py_compile tools\can\controlcan.py tools\can\can2_monitor.py tools\modbus\rtu_probe.py tools\modbus\rtu_codec.py tools\modbus\virtual_adc_module.py`: passed.
  - CPU0 default build `tmp/cmake_cpu0_sbus`: passed, `output/demo.elf` produced.
  - CPU0 CANopen build `tmp/cmake_cpu0_canopen`: passed, `output/demo.elf` produced.
  - `git diff --check`: no whitespace errors; only Git line-ending warnings for LF/CRLF conversion.

Hardware debug result on 2026-06-28:

- ECU image downloaded through J-Link with target `HPM6750xVMx`, JTAG, 4000 kHz. J-Link detected one RV32 core and verified flash programming.
- COM9 debug UART and COM10 USB-RS485 were used for live validation.
- RS485_1 Modbus ADC path:
  - Root cause found during bring-up: HPM6750 SDK 1.11 does not expose automatic UART DE direction control for this UART IP, so the previous UART11_DE pinmux left the external RS485 transceiver without a reliable software-owned direction switch.
  - Fix: `BOARD_RS485_DE_USING_GPIO` now defaults to `1`; pinmux drives RS485 DE pins as GPIO outputs in receive state, and `uart_rs485_hw` switches RS485_1 DE to transmit only around a frame.
  - PC virtual ADC tool now performs CRC/address/function based sliding re-synchronization, so it can be started while ECU polling is already active.
  - Verified COM9 sample: `[ECU MODBUS ADC] init=1 ... online=1 ... raw=[0,6554,13107,19660,26214,32768,39321,45874] mv=[0,500,1000,1499,2000,2500,3000,3499]`.
- CANopenNode CAN2 + BC2 result:
  - CAN2 at 1 Mbit/s reads BC2 objects continuously. Periodic uploads showed `sdo_ok` increasing and `sdo_abort=0` during normal operational state.
  - Debug variable address from the CANopen build: `g_canopen_master_debug_control = 0x01080818`; structure size is 36 bytes. GDB `detach` is the reliable way to resume after writing the variable.
  - SDO download commands verified with `dl_abort=0`: mode `0x6060:00=3`, controlword `0x6040:00=0x0006`, `0x0007`, `0x000F`, target velocity `0x60FF:00=100`, target position `0x607A:00=0`, target torque `0x6071:00=0`, and generic write `0x6040:00=0x0006`.
  - NMT commands verified: operational, pre-operational, stopped, reset communication and reset node. `NMT stopped` makes later periodic SDO reads report transient errors until the node is returned to operational/pre-operational; the final recovery command sent was NMT operational.
  - Final observed status after recovery: `[ECU CANopen CMD] seq=19 cmd=2 node=1 nmt=7 dl_ok=8 dl_abort=0 ... last_err=0`, and periodic CANopen SDO uploads resumed.

Engineering closure result on 2026-06-28:

- CPU0 remote preconditions now come from `ecu_hardware_feedback_snapshot_t`; power-ready, low-voltage-ok and CAN1-online are no longer hardcoded true.
- CAN1 power output now uses `ECU_POWER_PROTOCOL_SUPPLIER_CAN` by default. BMS, DCDC48, DCDC12 and DCAC use their documented 29-bit extended supplier CAN frames at 250 kbit/s.
- `task_can1_power` owns CAN1 TX/RX, drains a CAN RX queue, decodes supplier feedback through `power_can_protocol`, and sends safe-off commands unless `final_command.high_voltage_enable` is true.
- CPU0 runtime monitor now prints `[ECU HW]` with power, low-voltage, CAN1/CAN2/CAN3, brake, hydraulic and zero-speed feedback.
- Active `ecu`, `docs` and `tests` files were scanned for informal uncertainty and transitional engineering wording; no matches remain outside historical Git context.
- Local verification for this checkpoint:
  - `python tests\python\run_tests.py`: 41/41 tests passed.
  - `python tools\check_no_forbidden_patterns.py`: no forbidden ECU framework patterns found.
  - `python -m py_compile tools\can\controlcan.py tools\can\can2_monitor.py tools\modbus\rtu_probe.py tools\modbus\rtu_codec.py tools\modbus\virtual_adc_module.py`: passed.
  - CPU0 default build `tmp/cmake_cpu0_sbus`: passed, `output/demo.elf` produced.
  - CPU0 CANopen build `tmp/cmake_cpu0_canopen`: passed, `output/demo.elf` produced.
  - CPU1 build `tmp/cmake_cpu1_sbus`: passed, `output/demo.elf` produced.
  - `git diff --check`: no whitespace errors; Git only reported LF/CRLF conversion warnings for tracked text files.

Engineering closure result on 2026-06-29:

- Implemented the remaining CPU0 control closure for CAN2, CAN3, RS485_2, DIO and SBUS-to-command generation.
- RS485_2 warning-light protocol is now included in the main project. Default parameters are centralized in `ecu/config/include/ecu_config.h`:
  - `ECU_MODBUS_WARNING_LIGHT_BAUDRATE = 9600`
  - `ECU_MODBUS_WARNING_LIGHT_SLAVE_ID = 0xFF`
  - `ECU_MODBUS_WARNING_LIGHT_REGISTER = 0x00C2`
  - `ECU_MODBUS_WARNING_LIGHT_REQUEST_PERIOD_MS = 100`
  - `ECU_MODBUS_WARNING_LIGHT_RESPONSE_TIMEOUT_MS = 100`
- Remote driving limits are centralized as macros for later vehicle calibration:
  - `ECU_REMOTE_MAX_SPEED_KPH`
  - `ECU_REMOTE_MAX_STEER_DEG`
  - `ECU_REMOTE_MIN_HEIGHT_TARGET_MM`
  - `ECU_REMOTE_MAX_HEIGHT_TARGET_MM`
  - `ECU_REMOTE_MAX_HEIGHT_RATE_MM_S`
  - `ECU_REMOTE_MAX_TRACK_RATE_MM_S`
- Local verification for this checkpoint:
  - `python tests\python\run_tests.py`: 50/50 tests passed.
  - `python tools\check_no_forbidden_patterns.py`: no forbidden ECU framework patterns found.
  - CPU0 CANopenNode build is now the default build path and produces `output\demo.elf`.
  - Active `ecu`, `tests` and `docs` files were scanned for informal uncertainty and transitional engineering wording; no matches remain outside generated build output, which was deleted before commit.

## Known open items

- Replace or extend the DS301 minimum OD with the actual BC/BC2 servo-drive EDS/object dictionary if future PDO mapping or vendor-specific objects require local OD entries.
- CAN1 BMS/DCDC/DCAC communication is implemented through the supplier CAN protocol. Final DCDC/DCAC voltage-current setpoints remain calibration values recorded in `docs/ecu-configuration-open-items.md`.
- CAN4 remains an auxiliary service boundary until its real device role is selected.
- RS485_1 is connected to the hardware Modbus master path for the 8AI ADC module. RS485_2 is connected to the warning-light Modbus protocol. RS485_3 and RS232 service boundaries need hardware protocol owners when their devices are selected.
- Device-level control functions still need more supplier-specific detail from the real manuals: BC/BC2 scaling and enable sequence validation under main power, final DCDC/DCAC setpoints and hydraulic relay mapping.
- CPU0/CPU1 IPC transport still needs binding to the selected SDK multicore/RPMsg mechanism.
- CANopen PDO/object mapping, relay polarity, hydraulic valve bits and analog channel order are currently project defaults and should be calibrated on the real vehicle.
- Generated SES projects currently report that OpenOCD was not located, so debugger configuration must be set manually in SEGGER Embedded Studio unless OpenOCD is added to the SDK/tool PATH.
