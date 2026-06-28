# ECU Device Control Middleware Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace ad-hoc device CANopen/Modbus payload construction with device-level control APIs backed by SDK middleware or middleware-compatible backend boundaries.

**Architecture:** Keep protocol stacks out of project device code. Add a servo-drive adapter that exposes CiA 402 control functions and currently emits EDS-defined PDO frames through the raw CAN frame backend boundary. Keep Modbus request generation inside a thin Agile Modbus wrapper.

**Tech Stack:** C11, HPM SDK 1.11, FreeRTOS, HPM SDK CANopenNode staging, HPM SDK agile_modbus, Python contract tests, SEGGER/CMake/Ninja build.

---

### Task 1: Add contract tests for servo device API boundaries

**Files:**
- Modify: `tests/python/test_hardware_framework.py`

- [ ] **Step 1: Write the failing test**

Add this test:

```python
def test_servo_drive_adapter_is_device_level_and_cmake_owned(root: pathlib.Path) -> None:
    servo_h = read(root, "ecu/devices/include/servo_drive_canopen.h")
    servo_c = read(root, "ecu/devices/src/servo_drive_canopen.c")
    motion_c = read(root, "ecu/devices/src/motion_device.c")
    lift_c = read(root, "ecu/devices/src/lift_hydraulic_device.c")
    config_h = read(root, "ecu/config/include/ecu_config.h")
    cmake = read(root, "ecu/apps/agri_chassis_control_cpu0/CMakeLists.txt")

    for token in [
        "servo_drive_canopen_send_control_word",
        "servo_drive_canopen_select_mode",
        "servo_drive_canopen_set_target_position",
        "servo_drive_canopen_set_target_velocity",
        "servo_drive_canopen_set_target_torque",
        "SERVO_DRIVE_MODE_PROFILE_POSITION",
        "SERVO_DRIVE_MODE_PROFILE_VELOCITY",
    ]:
        assert token in servo_h or token in servo_c, token

    assert "servo_drive_canopen.c" in cmake
    assert "servo_drive_canopen_set_target_velocity" in motion_c
    assert "servo_drive_canopen_set_target_position" in motion_c
    assert "servo_drive_canopen_set_target_position" in lift_c
    assert "ECU_CANOPEN_RPDO2_BASE" in config_h
    assert "ECU_DRIVE_SPEED_KPH_TO_COUNTS_PER_SEC" in config_h
    assert "ECU_STEER_DEG_TO_COUNTS" in config_h
    assert "ECU_LIFT_MM_TO_COUNTS" in config_h
```

- [ ] **Step 2: Run test to verify it fails**

Run:

```powershell
python tests\python\run_tests.py
```

Expected: the new test fails because `servo_drive_canopen.h` does not exist.

### Task 2: Centralize PDO bases and unit scales

**Files:**
- Modify: `ecu/config/include/ecu_config.h`
- Modify: `ecu/config/src/ecu_config.c`

- [ ] **Step 1: Add config fields and macros**

Add RPDO2/RPDO3/RPDO4/RPDO5 bases and conversion scales to config:

```c
#define ECU_CANOPEN_RPDO2_BASE     (0x300UL)
#define ECU_CANOPEN_RPDO3_BASE     (0x400UL)
#define ECU_CANOPEN_RPDO4_BASE     (0x500UL)
#define ECU_CANOPEN_RPDO5_BASE     (0x80000000UL)
#define ECU_CANOPEN_COB_ID_DISABLED      (0x80000000UL)

#define ECU_DRIVE_SPEED_KPH_TO_COUNTS_PER_SEC (100.0f)
#define ECU_STEER_DEG_TO_COUNTS               (100.0f)
#define ECU_LIFT_MM_TO_COUNTS                 (100.0f)
```

Extend `ecu_canopen_node_config_t` with `rpdo2_cob_id`, `rpdo3_cob_id`, `rpdo4_cob_id`, and `rpdo5_cob_id`. Extend `ecu_hardware_config_t` with the three scale fields.

- [ ] **Step 2: Update default node configuration**

Change `CANOPEN_NODE_CONFIG(node)` so every node receives all RPDO COB-IDs:

```c
.rpdo2_cob_id = ECU_CANOPEN_RPDO2_BASE + (node), \
.rpdo3_cob_id = ECU_CANOPEN_RPDO3_BASE + (node), \
.rpdo4_cob_id = ECU_CANOPEN_RPDO4_BASE + (node), \
.rpdo5_cob_id = ECU_CANOPEN_RPDO5_BASE, \
```

Set the default scale fields from the new macros.

### Task 3: Add servo-drive CANopen adapter

**Files:**
- Create: `ecu/devices/include/servo_drive_canopen.h`
- Create: `ecu/devices/src/servo_drive_canopen.c`
- Modify: `ecu/apps/agri_chassis_control_cpu0/CMakeLists.txt`

- [ ] **Step 1: Add public API**

Create `servo_drive_canopen.h` with:

```c
typedef enum {
    SERVO_DRIVE_MODE_PROFILE_POSITION = 1,
    SERVO_DRIVE_MODE_PROFILE_VELOCITY = 3,
    SERVO_DRIVE_MODE_PROFILE_TORQUE = 4
} servo_drive_mode_t;

bool servo_drive_canopen_send_control_word(can_bus_service_t *bus,
                                           const ecu_canopen_node_config_t *node,
                                           uint16_t control_word);
bool servo_drive_canopen_select_mode(can_bus_service_t *bus,
                                     const ecu_canopen_node_config_t *node,
                                     uint16_t control_word,
                                     servo_drive_mode_t mode);
bool servo_drive_canopen_set_target_position(can_bus_service_t *bus,
                                             const ecu_canopen_node_config_t *node,
                                             uint16_t control_word,
                                             int32_t target_position_counts);
bool servo_drive_canopen_set_target_velocity(can_bus_service_t *bus,
                                             const ecu_canopen_node_config_t *node,
                                             uint16_t control_word,
                                             int32_t target_velocity_counts_per_sec);
bool servo_drive_canopen_set_target_torque(can_bus_service_t *bus,
                                           const ecu_canopen_node_config_t *node,
                                           uint16_t control_word,
                                           int16_t target_torque_raw);
```

- [ ] **Step 2: Implement minimal PDO serialization**

Implement little-endian packing helpers and send through `canopen_frame_build_pdo()` plus `can_bus_service_send_canopen()`. Return `false` when `bus`, `node`, or required RPDO COB-ID is invalid or disabled.

- [ ] **Step 3: Add CPU0 CMake source**

Add:

```cmake
sdk_app_src(../../devices/src/servo_drive_canopen.c)
```

### Task 4: Refactor motion and lift devices to use servo API

**Files:**
- Modify: `ecu/devices/src/motion_device.c`
- Modify: `ecu/devices/src/lift_hydraulic_device.c`

- [ ] **Step 1: Replace local CANopen payload construction in motion device**

`motion_device_apply()` should call:

```c
servo_drive_canopen_select_mode(can2, node, SERVO_DRIVE_CONTROL_ENABLE_OPERATION, SERVO_DRIVE_MODE_PROFILE_VELOCITY);
servo_drive_canopen_set_target_velocity(can2, node, SERVO_DRIVE_CONTROL_ENABLE_OPERATION, velocity_counts);
servo_drive_canopen_select_mode(can2, node, SERVO_DRIVE_CONTROL_ENABLE_OPERATION, SERVO_DRIVE_MODE_PROFILE_POSITION);
servo_drive_canopen_set_target_position(can2, node, SERVO_DRIVE_CONTROL_ENABLE_OPERATION, position_counts);
```

- [ ] **Step 2: Replace local CANopen payload construction in lift device**

`lift_hydraulic_device_apply()` should call:

```c
servo_drive_canopen_select_mode(can3, node, SERVO_DRIVE_CONTROL_ENABLE_OPERATION, SERVO_DRIVE_MODE_PROFILE_POSITION);
servo_drive_canopen_set_target_position(can3, node, SERVO_DRIVE_CONTROL_ENABLE_OPERATION, lift_counts);
```

Hydraulic valve output remains through `dio_service`.

### Task 5: Verify and document current limits

**Files:**
- Modify: `docs/ecu-main-control-architecture.md`
- Modify: `docs/ecu-main-control-progress.md`

- [ ] **Step 1: Run tests**

Run:

```powershell
python tests\python\run_tests.py
python tools\check_no_forbidden_patterns.py
```

Expected: all tests pass and forbidden-pattern scanner reports no ECU framework violations.

- [ ] **Step 2: Run firmware builds**

Run:

```powershell
& 'D:\agri_4wis_chassis_system\ecu\sdk_env_v1.11.0\tools\cmake\bin\cmake.exe' --build tmp\cmake_cpu0_sbus --clean-first
& 'D:\agri_4wis_chassis_system\ecu\sdk_env_v1.11.0\tools\cmake\bin\cmake.exe' --build tmp\cmake_cpu1_sbus --clean-first
git diff --check
```

Expected: both builds link `output\demo.elf`; `git diff --check` exits with code 0.
