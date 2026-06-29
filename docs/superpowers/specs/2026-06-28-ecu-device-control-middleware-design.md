# ECU Device Control Middleware Design

## Goal

Move CANopen and Modbus responsibilities back to proven middleware while keeping project code focused on vehicle-device control functions.

## Scope

This stage covers the first device-control cleanup:

- HPM SDK `agile_modbus` remains the Modbus RTU frame generator/parser.
- HPM SDK `CANopenNode` is the intended CANopen stack.
- Project code provides device-level APIs for servo-drive commands and Modbus devices.
- The earlier raw CAN-frame backend has been replaced by CANopenNode now that the project DS301 object dictionary is available.

This stage does not fake a CANopenNode object dictionary. `ECU_ENABLE_CANOPENNODE` must stay disabled until the actual EDS/object dictionary generation step is done.

## Design

### CANopen servo-drive API

Add a focused servo-drive adapter under `ecu/devices`:

- `servo_drive_canopen_send_control_word()`
- `servo_drive_canopen_select_mode()`
- `servo_drive_canopen_set_target_position()`
- `servo_drive_canopen_set_target_velocity()`
- `servo_drive_canopen_set_target_torque()`

The API names express CiA 402 device behavior. The first implementation uses the EDS default PDO mappings through the raw CAN-frame backend boundary. Later, the same API can be backed by CANopenNode PDO/SDO calls without changing `motion_device`, `lift_hydraulic_device`, `vehicle_command_executor`, or control logic.

The EDS-backed default mappings used by this adapter are:

- RPDO1: `0x6040` control word
- RPDO2: `0x6040` control word + `0x6060` mode of operation
- RPDO3: `0x6040` control word + `0x607A` target position
- RPDO4: `0x6040` control word + `0x60FF` target velocity
- RPDO5: `0x6040` control word + `0x6071` target torque, disabled unless configured

Object indices, COB-ID bases, disabled COB-ID value and unit scaling live in `ecu_config`, not inside device logic.

### Motion and lift usage

`motion_device` should call the servo adapter instead of assembling arbitrary payloads:

- drive nodes select profile velocity mode and send target velocity;
- steering nodes select profile position mode and send target position;
- the brake output remains a local IO responsibility.

`lift_hydraulic_device` should call the servo adapter for lift servo position/rate intent and keep hydraulic valve control through `dio_service`.

### Modbus usage

RS485 devices call Agile Modbus serialization and confirmation parsing directly. UART services own only transport, retry timing and timeout accounting.

Device-specific Modbus behavior belongs in devices such as `warning_light_device` and future ADC module functions. UART transport, idle interrupts, and receive buffers belong under `drivers/uart`.

## Testing

The Python contract tests should enforce:

- servo device API files exist and are included by CPU0 CMake;
- motion/lift devices call servo API functions;
- CANopen SDO/heartbeat/parser functionality is not added to the local shim;
- Modbus wrapper uses Agile Modbus and does not expose local CRC/parser APIs;
- all configurable COB-ID bases and unit scales are centralized in config.

Build verification must include:

- `python tests\python\run_tests.py`
- `python tools\check_no_forbidden_patterns.py`
- CPU0 CMake/Ninja build
- CPU1 CMake/Ninja build
