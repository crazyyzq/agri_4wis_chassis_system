# ECU Main Control Framework Progress

Last updated: 2026-07-01

This file records the durable implementation state for the main-branch control framework so the project can be resumed after context compression or a new development session.

## Baseline

- Hardware bring-up and board-level tests for RS485, RS232, CAN1-CAN4, SBUS, digital I/O and analog input have been completed before this framework work.
- Ethernet and CAN-FD are intentionally out of scope for the current software framework.
- The main project target is the HPM6750 ECU based on the MR6750 module and the HPM SDK 1.11 environment.
- The main framework is being written on `main`, not on the previous board-test branch.

## 2026-07-01 whole-vehicle pre-debug audit

- Fresh Python contract tests passed: `python tests\python\run_tests.py` -> 63/63.
- Fresh CPU0 target build passed through the SDK CMake tool:
  `ecu\sdk_env_v1.11.0\tools\cmake\bin\cmake.exe --build tmp\cmake_cpu0_canopennode --target all`.
- J-Link download to the connected ECU passed with `HPM6750xVMx`, JTAG, 4000 kHz and `VTref=3.283V`; the final run reported the flash content already matched `tmp\cmake_cpu0_canopennode\output\demo.elf`.
- COM5 debug UART is working at 115200 baud. With SBUS and most external devices not connected, the runtime correctly stays in safety state: `DIAG_REMOTE_ESTOP_SBUS_TIMEOUT`, gear `P`, speed `0`, brake release `0`, high voltage `0`, hydraulic `0`.
- Corrected two whole-vehicle blocking logic issues found during audit:
  - Servo brake-release polarity now matches the field wiring note: drive output low releases the brake, so `ECU_SERVO_BRAKE_RELEASE_CANOPEN_ACTIVE_BIT` is `0`.
  - D/R arming now requests brake release while active gear remains `P`, avoiding the previous ARM_D/ARM_R deadlock where the gear FSM waited for brake feedback before the command path released the brake.
- Corrected one hardware-boundary issue:
  - Servo brakes are no longer driven through PCB DIO. PCB local IO now handles horn, headlight and indicators only; BC/BC2 J3 OUT brake outputs are controlled through CANopen object `0x2194`.
- Corrected one CANopen command-cache retry issue:
  - Motion and lift/hydraulic adapters update their cached actuator command only after all SDO requests in the current command set are accepted by the local CANopen queue.
  - Because local queue acceptance is not the same as remote SDO completion, unchanged motion and lift commands are also refreshed at `ECU_CANOPEN_MOTION_COMMAND_REFRESH_MS` / `ECU_CANOPEN_LIFT_COMMAND_REFRESH_MS`. This prevents a later SDO timeout, abort or drive reset from causing an unchanged command to be skipped indefinitely.
- Current commissioning switches intentionally still enabled:
  - `ECU_ENABLE_COMMISSIONING_CANOPEN_SCAN=1` sends read-only `0x6041` statusword SDO scans on CAN2/CAN3.
  - `ECU_ENABLE_CAN4_PHYSICAL_TEST_TX=1` sends standard CAN4 test frame `0x444` every 500 ms. Disable this before assigning CAN4 to a real device protocol.

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

Code-quality and logic audit checkpoint on 2026-07-01:

- `ecu/os/src/ecu_tasks_cpu0.c` was reduced back to orchestration responsibility. SBUS protocol raw-to-PPM conversion, analog channel scaling, credibility checks and discrete switch debouncing now live in `ecu/remote/src/remote_sbus_mapper.c` with the public interface in `ecu/remote/include/remote_sbus_mapper.h`.
- Fixed CAN1 power-bus online logic. `power_device_refresh_online()` now uses the current CAN service online state and no longer treats cumulative `error_count` as a permanent offline latch.
- Fixed CAN3 lift/hydraulic command caching. Track-width adjustment, hydraulic enable and hydraulic valve-mask changes are now part of the CAN3 command cache key, so a pure track-width command still resends the hydraulic-station CANopen command.
- Fixed RS485_1 analog ADC online state. A Modbus master timeout now clears `analog_modbus_device_state_t.online`, preventing stale “online” reporting after the ADC module is unplugged or stops responding.
- Added regression coverage for the three bugs above:
  - `test_sbus_mapping_is_owned_by_remote_layer_not_cpu0_tasks`
  - `test_power_can_online_uses_current_bus_state_not_cumulative_errors`
  - `test_lift_hydraulic_canopen_command_cache_includes_track_and_pump_intent`
  - `test_analog_modbus_adc_marks_offline_on_master_timeout`
- Local verification for this checkpoint:
  - `python tests\python\run_tests.py`: 67/67 tests passed.
  - `cmake --build tmp\cmake_cpu0_canopennode --target all`: passed and linked `output\demo.elf`.
  - `git diff --check`: no whitespace errors; Git only reported LF/CRLF conversion warnings for tracked text files.
- Remaining code-quality note: `remote_event_lifecycle` is initialized but not yet functionally wired into the remote FSM event path. It is not a direct dangerous-output bug, but it should either be connected to mode/power/light events or removed in a later cleanup to avoid unused architecture.

Additional logic audit checkpoint on 2026-07-01:

- Fixed a track-width adjustment deadlock. `remote_adjust_fsm` enters `ADJUST_STATE_TRACK_PREPARE` and waits for `brake_release_confirmed`; `command_arbiter` now requests brake release during `ADJUST_STATE_TRACK_PREPARE` and `ADJUST_STATE_TRACK_ACTIVE`, matching the existing D/R arming handshake.
- Fixed AUTO command consistency. Automatic motion commands now derive D/R/P gear from target speed and request brake release when target speed is non-zero, avoiding contradictory `target_speed_kph != 0` with P gear and brake applied.
- Fixed motion command cache comparison. `motion_device` no longer uses `memcmp()` on `vehicle_actuator_command_t`; it explicitly compares the motion-relevant fields so structure padding cannot cause false command changes.
- Added regression coverage:
  - `test_track_adjust_prepare_requests_brake_release_before_active_adjust`
  - `test_auto_motion_command_sets_consistent_gear_and_brake_release`
  - `test_motion_command_cache_does_not_memcmp_struct_padding`
- Hardware download/check:
  - J-Link V9.16 detected HPM6750xVMx RV32 target at VTref 3.283 V and downloaded `tmp\cmake_cpu0_canopennode\output\demo.elf` successfully.
  - COM5 monitor showed the ECU running in safe state with no SBUS connected: `src=safety`, `gear=P`, `speed=0.00kph`, `brake=0`, `hv=0`, `hyd=0`.
  - CAN2/CAN3 CANopen services initialized and stayed `normal=1`; missing node responses appear as SDO abort/timeouts when devices are not responding.
  - RS485_1 ADC showed `online=0` after timeouts, confirming the stale-online fix.

Hardware-missing and status-indicator audit checkpoint on 2026-07-01:

- Refined ECU RGB status semantics so the PCB LED can distinguish a running safe ECU from a real hard stop:
  - blue heartbeat: booting or no online SBUS/remote;
  - solid green: remote online and required buses healthy;
  - cyan heartbeat: high-voltage, brake-release, hydraulic or motion command active;
  - yellow heartbeat: communication/peripheral warning;
  - red: operator CH13 emergency stop or fatal A-class fault.
- SBUS timeout/failsafe still clamps all actuator outputs through the safety manager, but it no longer forces the LED to solid red. This prevents an ECU on the bench with no transmitter connected from looking like a dead or fatal board.
- `board_init_console()` and `board_init_i2c()` no longer trap forever if the debug console path or optional I2C branch cannot initialize. They report a warning and allow the scheduler, watchdog-visible tasks and RGB status service to continue.
- Core fatal loops remain intentionally limited to clock/SDRAM startup failure and FreeRTOS fault/assert hooks. These are not ordinary unplugged-peripheral cases.
- Local verification for this checkpoint:
  - `python tests\python\run_tests.py`: 72/72 tests passed.
  - `ecu\sdk_env_v1.11.0\tools\ninja\ninja.exe -C tmp\cmake_cpu0_canopennode`: passed and linked `output\demo.elf`.
  - `tmp\cmake_cpu0_canopennode\segger_embedded_studio\agri_chassis_control_cpu0.emProject` includes the modified CPU0 task, board and RGB status LED source files.

Follow-up hardware verification on 2026-07-01:

- Added `led=<pattern>` to the `[ECU MON]` runtime line so the serial log reports the selected PCB RGB state explicitly.
- Added a board-level guard in `board_init_i2c()` so an unbound I2C peripheral base or zero source clock is reported and skipped before the SDK I2C driver is called.
- J-Link download passed after ECU power was restored:
  - target `HPM6750xVMx`, JTAG 4000 kHz, `VTref=3.283V`;
  - flash erase/program/verify completed for `tmp\cmake_cpu0_canopennode\output\demo.elf`.
- COM5 serial monitor confirmed the firmware kept scheduling with disconnected SBUS/power/ADC/drive responses:
  - `[ECU MON] ... led=no_remote ... link=offline estop=latched diag=DIAG_REMOTE_ESTOP_SBUS_TIMEOUT`;
  - `seq` kept increasing, CANopen and Modbus remained in timeout/offline paths, and final command stayed safe: `src=safety`, gear `P`, speed `0`, brake release `0`, high voltage `0`, hydraulic `0`.
- Local verification for this follow-up:
  - `python tests\python\run_tests.py`: 73/73 tests passed.
  - `ecu\sdk_env_v1.11.0\tools\ninja\ninja.exe -C tmp\cmake_cpu0_canopennode`: passed and linked `output\demo.elf`.

SEGGER regeneration and code-quality audit on 2026-07-01:

- Re-generated current SEGGER Embedded Studio projects:
  - CPU0: `tmp\cmake_cpu0_canopennode\segger_embedded_studio\agri_chassis_control_cpu0.emProject`
  - CPU1: `tmp\cmake_cpu1_latest\segger_embedded_studio\agri_chassis_control_cpu1.emProject`
- CPU1 CMake now matches CPU0's self-contained SDK/toolchain setup and explicitly selects J-Link at 4000 kHz. This prevents regenerated CPU1 SES projects from falling back to OpenOCD/manual debugger setup.
- Full code-quality scan result:
  - no project-local uncertainty marker or hand-written CANopen/Modbus protocol stack remains;
  - CANopen communication is routed through HPM SDK CANopenNode and `canopen_master_service`;
  - Modbus communication is routed through HPM SDK Agile Modbus and `modbus_master_service`;
  - remote/control layers do not directly access HPM registers or board GPIO/CAN/UART drivers;
  - remaining infinite loops are limited to FreeRTOS task loops and fatal/assert hooks;
  - UART RX loops have per-ISR byte budgets, RS485 TX wait has a timeout, and CAN RX processing uses finite service queues.
- Local verification for this audit:
  - `python tests\python\run_tests.py`: 74/74 tests passed.
  - CPU0 regenerate + build passed and linked `tmp\cmake_cpu0_canopennode\output\demo.elf`.
  - CPU1 regenerate + build passed and linked `tmp\cmake_cpu1_latest\output\demo.elf`.
  - `git diff --check`: no whitespace errors; only Git LF/CRLF conversion warnings for tracked text files.

Vehicle geometry mapping checkpoint on 2026-07-01:

- Froze the physical wheel/leg convention with the vehicle front as the positive direction:
  - leg 1 = front-right drive, steering and lift;
  - leg 2 = front-left drive, steering and lift;
  - leg 3 = rear-left drive, steering and lift;
  - leg 4 = rear-right drive, steering and lift.
- Updated `ecu/config` so all actuator arrays are explicitly documented as leg order `1, 2, 3, 4`, which means physical order `front-right, front-left, rear-left, rear-right`.
- Corrected the semantic CANopen aliases:
  - `FR` aliases now bind to leg 1 nodes;
  - `FL` aliases now bind to leg 2 nodes;
  - `RL` aliases bind to leg 3 nodes;
  - `RR` aliases bind to leg 4 nodes.
- Updated CPU0 CAN3 CANopen diagnostic default node from the stale front-left alias to the front-right/leg-1 lift node.
- Added regression checks so a future edit cannot silently map leg 1 back to front-left.
- Local verification for this mapping update:
  - `python tests\python\run_tests.py`: 74/74 tests passed.
  - `ecu\sdk_env_v1.11.0\tools\ninja\ninja.exe -C tmp\cmake_cpu0_canopennode`: passed and linked `output\demo.elf`.
  - `ecu\sdk_env_v1.11.0\tools\ninja\ninja.exe -C tmp\cmake_cpu1_latest`: passed and linked `output\demo.elf`.

Motion/control bug-fix checkpoint on 2026-07-01:

- Fixed the motion command model so the final actuator command carries both the
  chassis-level target speed and explicit per-wheel target speeds.  CANopen drive
  nodes now receive `target_wheel_speed_kph[wheel]` instead of the same chassis
  speed for every wheel.
- Replaced the earlier simplified steering output that gave all four wheels the same
  angle.  `motion_control` now builds distinct four-wheel targets for positive
  Ackermann, reverse Ackermann, spin and crab modes using the confirmed leg
  order: front-right, front-left, rear-left, rear-right.
- Fixed CANopen command caching in the motion and lift/hydraulic adapters.  A
  command is cached only after all queued SDO writes for that adapter succeed,
  so a full queue, offline node or rejected write does not suppress the next
  retry.
- Fixed lift limit behavior.  If a lift axis is commanded into an active upper
  or lower limit, the adapter queues quick-stop for that axis and does not
  release that axis brake output.
- Extended commissioning high-voltage debug clamping so it also clears all
  per-wheel speed targets.
- Added regression checks for:
  - mode-specific four-wheel motion targets;
  - per-wheel speed propagation into `motion_device`;
  - CANopen command cache update only on successful queueing;
  - lift limit blocking the corresponding brake release.
- Local verification for this bug-fix checkpoint:
  - `python tests\python\run_tests.py`: 77/77 tests passed.
  - `ecu\sdk_env_v1.11.0\tools\ninja\ninja.exe -C tmp\cmake_cpu0_canopennode`: passed and linked `output\demo.elf`.
  - `ecu\sdk_env_v1.11.0\tools\ninja\ninja.exe -C tmp\cmake_cpu1_latest`: passed and linked `output\demo.elf`.
  - `git diff --check`: no whitespace errors; only Git LF/CRLF conversion warnings for tracked text files.

## Known open items

- Replace or extend the DS301 minimum OD with the actual BC/BC2 servo-drive EDS/object dictionary if future PDO mapping or vendor-specific objects require local OD entries.
- CAN1 BMS/DCDC/DCAC communication is implemented through the supplier CAN protocol. Final DCDC/DCAC voltage-current setpoints remain calibration values recorded in `docs/ecu-configuration-open-items.md`.
- CAN4 remains an auxiliary service boundary until its real device role is selected.
- RS485_1 is connected to the hardware Modbus master path for the 8AI ADC module. RS485_2 is connected to the warning-light Modbus protocol. RS485_3 and RS232 service boundaries need hardware protocol owners when their devices are selected.
- Device-level control functions still need more supplier-specific detail from the real manuals: BC/BC2 scaling and enable sequence validation under main power, final DCDC/DCAC setpoints and hydraulic relay mapping.
- CPU0/CPU1 IPC transport still needs binding to the selected SDK multicore/RPMsg mechanism.
- CANopen PDO/object mapping, relay polarity, hydraulic valve bits and analog channel order are currently project defaults and should be calibrated on the real vehicle.
- Generated CPU0 and CPU1 SEGGER projects currently select J-Link at 4000 kHz.
