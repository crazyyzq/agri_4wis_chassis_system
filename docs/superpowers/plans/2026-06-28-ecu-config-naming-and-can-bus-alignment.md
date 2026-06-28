# ECU Configuration Naming and CAN Bus Alignment Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove informal uncertainty-marker naming from ECU project code, document calibration-open items separately, and align CAN1/CAN2 defaults with the actual power and BC/BC2 drive buses.

**Architecture:** Project constants keep professional domain names in `ecu/config`; uncertainty is tracked in a markdown open-items file rather than encoded into symbols. CAN1 is the power/BMS bus at 250 kbit/s by default, while CAN2 is the BC/BC2 motion bus at 1 Mbit/s for current drive debugging. The CAN hardware debug adapter is renamed away from CAN1-specific BC2 wording and bound to external CAN2.

**Tech Stack:** C11, HPM SDK 1.11, FreeRTOS, HPM CAN driver, Python static contract tests, CMake/Ninja.

---

### Task 1: Add contract tests before production changes

**Files:**
- Modify: `tests/python/test_hardware_framework.py`

- [x] **Step 1: Add tests that fail on current code**

Add tests that require:
- no informal uncertainty-marker token in `ecu/` production code or active Python tests;
- CAN1 power default bitrate is `250000UL`;
- CAN2 motion/BC debug default bitrate is `1000000UL`;
- BC/BC2 RX-only hardware debug is bound to `BOARD_CAN2_BASE` and `BOARD_CAN2_IRQn`;
- runtime monitor prints CAN2 drive debug fields, not CAN1 BC2 debug fields;
- `docs/ecu-configuration-open-items.md` exists and records BMS bitrate source.

- [x] **Step 2: Run the tests and verify failure**

Run:

```powershell
python tests\python\run_tests.py
```

Expected: new tests fail because current code still has `ECU_*`, CAN1 BC2 debug naming, and CAN1 bitrate at 1 Mbit/s.

### Task 2: Rename configuration symbols and update defaults

**Files:**
- Modify: `ecu/config/include/ecu_config.h`
- Modify: `ecu/config/src/ecu_config.c`
- Modify: users under `ecu/`

- [x] **Step 1: Rename macros**

Rename every marker-prefixed macro to professional domain names, for example:
- CAN1 power bus bitrate -> `ECU_CAN1_POWER_BITRATE`
- CAN2 motion bus bitrate -> `ECU_CAN2_MOTION_BITRATE`
- CANopen defaults -> `ECU_CANOPEN_*`
- digital IO masks -> `ECU_DIO_*`
- hydraulic valve masks -> `ECU_HYD_VALVE_*`
- ADC scaling -> `ECU_ADC_*`
- Modbus defaults -> `ECU_MODBUS_*`
- RS485 baudrate -> `ECU_RS485_BAUDRATE`
- RS232 baudrate -> `ECU_RS232_BAUDRATE`
- SBUS UART settings -> `ECU_SBUS_UART_*`

- [x] **Step 2: Set bus defaults**

Set:

```c
#define ECU_CAN1_POWER_BITRATE          (250000UL)
#define ECU_CAN2_MOTION_BITRATE         (1000000UL)
```

Keep CAN3/CAN4 defaults as configurable project defaults.

### Task 3: Move BC/BC2 RX-only debug from CAN1 to CAN2

**Files:**
- Modify: `ecu/drivers/can/include/can_bus_hw.h`
- Modify: `ecu/drivers/can/src/can_bus_hw.c`
- Modify: `ecu/os/src/ecu_tasks_cpu0.c`
- Modify: `ecu/diag/include/runtime_monitor.h`
- Modify: `ecu/diag/src/runtime_monitor.c`

- [x] **Step 1: Rename API and state**

Rename the former CAN1 BC2 debug API/state to CAN2 motion/drive debug names:
- hardware RX-only init API -> `can_bus_hw_init_can2_rx_only`
- hardware RX poll API -> `can_bus_hw_poll_can2_rx`
- runtime debug service -> `can2_drive_debug`
- monitor fields use `can2_*`
- monitor label is `[ECU CAN2]`

- [x] **Step 2: Bind hardware to CAN2**

Use:

```c
BOARD_CAN2_BASE
BOARD_CAN2_IRQn
board_set_can_termination(2U, ECU_CAN2_TERMINATION_ENABLE != 0)
```

Keep this binding RX-only until explicit motion-control TX is approved.

### Task 4: Document calibration-open items

**Files:**
- Create: `docs/ecu-configuration-open-items.md`
- Modify: `docs/ecu-main-control-architecture.md`
- Modify: `docs/ecu-main-control-progress.md`

- [x] **Step 1: Create open-items document**

Record configurable items that need final hardware/manual confirmation:
- CAN1 BMS/power bus bitrate: default `250 kbit/s`; source: BMS communication protocol page 6, bit time 4 us. Manual also mentions 250/500 kbit/s is vehicle-defined.
- CAN2 BC/BC2 motion bus bitrate: default `1 Mbit/s`.
- BMS/DCDC/DCAC node IDs and CAN IDs.
- BC/BC2 PDO mapping and object dictionary.
- DIO masks, hydraulic valve masks, ADC scaling, Modbus addresses, warning light registers.

- [x] **Step 2: Replace documentation wording**

Use "project default", "calibration item", or "pending hardware confirmation" instead of informal uncertainty wording.

### Task 5: Verify

**Files:**
- No new files.

- [x] **Step 1: Run static tests**

```powershell
python tests\python\run_tests.py
python tools\check_no_forbidden_patterns.py
```

Expected: all tests pass and forbidden scanner reports no violations.

- [x] **Step 2: Build both cores**

```powershell
& 'D:\agri_4wis_chassis_system\ecu\sdk_env_v1.11.0\tools\cmake\bin\cmake.exe' --build tmp\cmake_cpu0_sbus --config debug
& 'D:\agri_4wis_chassis_system\ecu\sdk_env_v1.11.0\tools\cmake\bin\cmake.exe' --build tmp\cmake_cpu1_sbus --config debug
```

Expected: both builds link `output\demo.elf`.

- [x] **Step 3: Check whitespace and remaining naming**

```powershell
git diff --check
python -c "from pathlib import Path; roots=['ecu','tests','docs']; bad=['G'+'U'+'E'+'S'+'S','g'+'u'+'e'+'s'+'s','g'+'u'+'e'+'s'+'s'+'e'+'d']; [print(p) for r in roots for p in Path(r).rglob('*') if p.is_file() and 'ecu/sdk_env_v1.11.0' not in p.as_posix() and p.suffix.lower() in {'.c','.h','.py','.md','.cmake','.txt'} and any(b in p.read_text(encoding='utf-8', errors='ignore') for b in bad)]"
```

Expected: no production/test hits. Historical plan files may be updated or excluded only if they are clearly obsolete.
