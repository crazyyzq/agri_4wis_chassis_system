# Modbus Virtual ADC and CANopen Command Debug Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a PC-side virtual Modbus ADC module, connect ECU RS485_1 to Modbus master polling, and extend the verified CANopenNode CAN2 service to explicit debugger-triggered BC/BC2 command testing.

**Architecture:** CPU0 keeps ownership of safety-critical outputs. RS485_1 UART11 ISR only buffers bytes; the IO task owns Modbus request/response state and updates analog snapshots. CANopenNode owns CAN2 when enabled; command output is only consumed from a volatile debug control structure when its sequence number changes.

**Tech Stack:** C11, HPM SDK 1.11, FreeRTOS, HPM UART/CAN drivers, HPM SDK CANopenNode, HPM SDK agile_modbus, Python pyserial, Python contract tests, CMake/Ninja, J-Link.

---

## File structure

- `tests/python/test_hardware_framework.py`: contract tests for virtual Modbus ADC tooling, RS485_1 Modbus service, CANopen command debug safety gates and cleanup.
- `tools/modbus/virtual_adc_module.py`: PC COM10 virtual ADC slave, function 04 only.
- `tools/modbus/rtu_codec.py`: Python-only Modbus RTU CRC/frame helpers used by test tools.
- `ecu/protocol/modbus/include/modbus_rtu.h`, `ecu/protocol/modbus/src/modbus_rtu.c`: Agile Modbus thin wrappers for request serialization and response extraction.
- `ecu/drivers/uart/include/uart_rs485_hw.h`, `ecu/drivers/uart/src/uart_rs485_hw.c`: RS485_1 UART11 hardware binding and RX byte buffering.
- `ecu/drivers/uart/include/modbus_master_service.h`, `ecu/drivers/uart/src/modbus_master_service.c`: foreground Modbus RTU master state machine.
- `ecu/devices/include/analog_modbus_device.h`, `ecu/devices/src/analog_modbus_device.c`: ADC module register-map adapter.
- `ecu/devices/src/warning_light_device.c`: update warning-light register/value mapping to documented register `0x00C2`.
- `ecu/drivers/canopen/include/canopen_master_service.h`, `ecu/drivers/canopen/src/canopen_master_service.c`: add debugger-triggered NMT/SDO download command path.
- `ecu/diag/include/runtime_monitor.h`, `ecu/diag/src/runtime_monitor.c`: print Modbus and CANopen command-debug snapshots.
- `ecu/os/src/ecu_tasks_cpu0.c`: initialize/process RS485_1 Modbus and include snapshots.
- `ecu/config/include/ecu_config.h`, `ecu/config/src/ecu_config.c`: centralize ADC Modbus and warning-light register defaults.
- `docs/ecu-main-control-progress.md`, `docs/ecu-main-control-architecture.md`: durable progress and structure update.

## Task 1: Add failing contract tests

**Files:**
- Modify: `tests/python/test_hardware_framework.py`

- [ ] **Step 1: Add tests for PC virtual Modbus ADC tool**

Add assertions requiring:

```python
virtual_adc = read(root, "tools/modbus/virtual_adc_module.py")
assert "Read Input Registers" in virtual_adc
assert "registers 0..7" in virtual_adc
assert "write_register" not in virtual_adc
assert "write_multiple_registers" not in virtual_adc
assert "--port" in virtual_adc
assert "--slave" in virtual_adc
assert "--baudrate" in virtual_adc
assert "--channels-mv" in virtual_adc
```

- [ ] **Step 2: Add tests for ECU RS485_1 Modbus master structure**

Add assertions requiring:

```python
assert exists(root, "ecu/drivers/uart/include/uart_rs485_hw.h")
assert exists(root, "ecu/drivers/uart/src/uart_rs485_hw.c")
assert exists(root, "ecu/drivers/uart/include/modbus_master_service.h")
assert exists(root, "ecu/drivers/uart/src/modbus_master_service.c")
assert exists(root, "ecu/devices/include/analog_modbus_device.h")
assert exists(root, "ecu/devices/src/analog_modbus_device.c")
tasks_c = read(root, "ecu/os/src/ecu_tasks_cpu0.c")
assert "modbus_master_service_process" in tasks_c
assert "analog_modbus_device_process" in tasks_c
assert "BOARD_RS485_1_UART_BASE" in read(root, "ecu/drivers/uart/src/uart_rs485_hw.c")
```

- [ ] **Step 3: Add tests for CANopen command-debug safety gates**

Add assertions requiring:

```python
service_h = read(root, "ecu/drivers/canopen/include/canopen_master_service.h")
service_c = read(root, "ecu/drivers/canopen/src/canopen_master_service.c")
assert "canopen_master_debug_control_t" in service_h
assert "volatile canopen_master_debug_control_t g_canopen_master_debug_control" in service_c
assert "command_sequence" in service_h
assert "CANOPEN_MASTER_DEBUG_COMMAND_NONE" in service_h
assert "CANOPEN_MASTER_DEBUG_COMMAND_SDO_WRITE_CONTROLWORD" in service_h
assert "CANOPEN_MASTER_DEBUG_COMMAND_SDO_WRITE_TARGET_VELOCITY" in service_h
assert "CANOPEN_MASTER_DEBUG_COMMAND_NMT_OPERATIONAL" in service_h
assert "CO_SDOclientDownloadInitiate" in service_c
assert "CO_SDOclientDownload(" in service_c
assert "CO_NMT_sendCommand" in service_c
assert "CANOPEN_MASTER_DEBUG_COMMAND_NONE" in service_c.split("canopen_master_service_init")[1].split("return true")[0]
```

- [ ] **Step 4: Verify RED**

Run:

```powershell
python tests\python\run_tests.py
```

Expected: FAIL because the new files and symbols do not exist.

## Task 2: Implement Python virtual ADC module

**Files:**
- Create: `tools/modbus/rtu_codec.py`
- Create: `tools/modbus/virtual_adc_module.py`

- [ ] **Step 1: Implement `rtu_codec.py`**

Provide:

```python
def crc16_modbus(data: bytes) -> int
def append_crc(payload: bytes) -> bytes
def verify_crc(frame: bytes) -> bool
def build_exception(slave: int, function: int, code: int) -> bytes
def build_read_input_registers_response(slave: int, values: list[int]) -> bytes
```

No write helpers in this file.

- [ ] **Step 2: Implement `virtual_adc_module.py`**

Behavior:

- list serial ports with `--list`;
- open `--port COM10` by default;
- default `--baudrate 9600`;
- default `--slave 1`;
- parse `--channels-mv 0,714,1428,2142,2856,3570,4284,5000`;
- convert mV to raw `0..65535` using `--full-scale-mv 5000`;
- respond only to function `04`;
- support start address `0..7`, count `1..8`;
- print `RX`, `TX`, and decoded channel values.

- [ ] **Step 3: Verify GREEN**

Run:

```powershell
python -m py_compile tools\modbus\rtu_codec.py tools\modbus\virtual_adc_module.py
python tests\python\run_tests.py
```

Expected: tests related to tool files pass; remaining ECU tests may still fail until later tasks.

## Task 3: Extend Modbus RTU wrapper and ADC device adapter

**Files:**
- Modify: `ecu/protocol/modbus/include/modbus_rtu.h`
- Modify: `ecu/protocol/modbus/src/modbus_rtu.c`
- Create: `ecu/devices/include/analog_modbus_device.h`
- Create: `ecu/devices/src/analog_modbus_device.c`
- Modify: `ecu/config/include/ecu_config.h`
- Modify: `ecu/config/src/ecu_config.c`

- [ ] **Step 1: Add Modbus response extraction API**

Add:

```c
typedef struct {
    uint16_t registers[ECU_ADC_CHANNEL_COUNT];
    uint8_t register_count;
} modbus_rtu_register_response_t;

bool modbus_rtu_extract_read_input_registers(const uint8_t *adu,
                                             size_t adu_size,
                                             uint8_t expected_slave_id,
                                             uint16_t expected_count,
                                             modbus_rtu_register_response_t *out);
```

Use Agile Modbus helpers where available; keep this wrapper thin.

- [ ] **Step 2: Add ADC module config**

Add central macros/fields:

```c
#define ECU_MODBUS_ADC_BAUDRATE          (9600UL)
#define ECU_MODBUS_ADC_START_REGISTER    (0U)
#define ECU_MODBUS_ADC_REGISTER_COUNT    (8U)
#define ECU_MODBUS_ADC_RAW_MAX           (65535U)
#define ECU_MODBUS_WARNING_LIGHT_REGISTER (0x00C2U)
```

Wire them into `ecu_hardware_config_t`.

- [ ] **Step 3: Add analog Modbus device adapter**

Expose:

```c
typedef struct {
    uint32_t request_count;
    uint32_t response_count;
    uint32_t error_count;
    uint16_t raw[ECU_ADC_CHANNEL_COUNT];
    uint32_t millivolt[ECU_ADC_CHANNEL_COUNT];
} analog_modbus_device_state_t;

bool analog_modbus_device_build_request(const ecu_hardware_config_t *config,
                                        modbus_rtu_frame_t *out);

bool analog_modbus_device_apply_response(analog_modbus_device_state_t *state,
                                         analog_input_service_t *analog_inputs,
                                         const ecu_hardware_config_t *config,
                                         const uint8_t *adu,
                                         size_t adu_size);
```

- [ ] **Step 4: Verify**

Run:

```powershell
python tests\python\run_tests.py
```

Expected: Modbus wrapper and device adapter tests pass; RS485 hardware service tests still fail until Task 4.

## Task 4: Add RS485_1 UART hardware and Modbus master service

**Files:**
- Create: `ecu/drivers/uart/include/uart_rs485_hw.h`
- Create: `ecu/drivers/uart/src/uart_rs485_hw.c`
- Create: `ecu/drivers/uart/include/modbus_master_service.h`
- Create: `ecu/drivers/uart/src/modbus_master_service.c`
- Modify: `ecu/apps/agri_chassis_control_cpu0/CMakeLists.txt`

- [ ] **Step 1: Add UART11 RS485 hardware service**

API:

```c
typedef struct {
    uint32_t rx_count;
    uint32_t tx_count;
    uint32_t line_error_count;
    uint8_t rx_buffer[256];
    size_t rx_size;
    bool initialized;
} uart_rs485_hw_t;

bool uart_rs485_1_hw_init(uart_rs485_hw_t *service, uint32_t baudrate);
bool uart_rs485_1_hw_send(uart_rs485_hw_t *service, const uint8_t *data, size_t size);
size_t uart_rs485_1_hw_read(uart_rs485_hw_t *service, uint8_t *out, size_t max_size);
void uart_rs485_1_hw_isr(void);
```

Implementation uses `BOARD_RS485_1_UART_BASE`, `BOARD_RS485_1_UART_IRQ`, HPM UART interrupt RX data/timeout/line status, and hardware DE pin.

- [ ] **Step 2: Add foreground Modbus master service**

State machine:

- idle until request period expires;
- send request;
- collect response bytes until expected length or timeout;
- call a caller-owned response handler;
- record tx/rx/timeout/crc/error counters.

- [ ] **Step 3: Add CMake sources/includes**

Add new UART and analog Modbus device source files to CPU0 CMake.

- [ ] **Step 4: Verify build**

Run:

```powershell
python tests\python\run_tests.py
cmake --build tmp\cmake_cpu0_sbus --config debug
```

Expected: tests pass or only later CANopen command tests fail; CPU0 default build passes.

## Task 5: Wire ECU IO task to virtual ADC polling and COM9 monitor

**Files:**
- Modify: `ecu/os/src/ecu_tasks_cpu0.c`
- Modify: `ecu/diag/include/runtime_monitor.h`
- Modify: `ecu/diag/src/runtime_monitor.c`

- [ ] **Step 1: Add runtime objects**

Add to CPU0 runtime:

```c
uart_rs485_hw_t rs485_1_hw;
modbus_master_service_t adc_modbus_master;
analog_modbus_device_state_t analog_modbus_adc;
```

- [ ] **Step 2: Initialize RS485_1 and Modbus master**

Use:

```c
uart_rs485_1_hw_init(&s_runtime.rs485_1_hw, ECU_MODBUS_ADC_BAUDRATE);
modbus_master_service_init(&s_runtime.adc_modbus_master, ECU_CPU0_IO_PERIOD_MS);
```

- [ ] **Step 3: Process ADC polling in `ecu_task_io_service_step`**

Build function 04 request from `analog_modbus_device_build_request`, send through Modbus master, and apply valid responses to `analog_input_service`.

- [ ] **Step 4: Print COM9 Modbus line**

Add:

```text
[ECU MODBUS ADC] init=1 tx=... rx=... timeout=... err=... raw=[...] mv=[...]
```

- [ ] **Step 5: Verify**

Run:

```powershell
python tests\python\run_tests.py
cmake --build tmp\cmake_cpu0_sbus --config debug
```

Expected: default build passes and monitor tests pass.

## Task 6: Add CANopen debugger-triggered command path

**Files:**
- Modify: `ecu/drivers/canopen/include/canopen_master_service.h`
- Modify: `ecu/drivers/canopen/src/canopen_master_service.c`
- Modify: `ecu/diag/include/runtime_monitor.h`
- Modify: `ecu/diag/src/runtime_monitor.c`

- [ ] **Step 1: Add command enums and debug control structure**

Add:

```c
typedef enum {
    CANOPEN_MASTER_DEBUG_COMMAND_NONE = 0,
    CANOPEN_MASTER_DEBUG_COMMAND_NMT_PRE_OPERATIONAL,
    CANOPEN_MASTER_DEBUG_COMMAND_NMT_OPERATIONAL,
    CANOPEN_MASTER_DEBUG_COMMAND_NMT_STOPPED,
    CANOPEN_MASTER_DEBUG_COMMAND_NMT_RESET_NODE,
    CANOPEN_MASTER_DEBUG_COMMAND_NMT_RESET_COMMUNICATION,
    CANOPEN_MASTER_DEBUG_COMMAND_SDO_WRITE_CONTROLWORD,
    CANOPEN_MASTER_DEBUG_COMMAND_SDO_WRITE_MODE,
    CANOPEN_MASTER_DEBUG_COMMAND_SDO_WRITE_TARGET_POSITION,
    CANOPEN_MASTER_DEBUG_COMMAND_SDO_WRITE_TARGET_VELOCITY,
    CANOPEN_MASTER_DEBUG_COMMAND_SDO_WRITE_TARGET_TORQUE,
    CANOPEN_MASTER_DEBUG_COMMAND_SDO_WRITE_GENERIC
} canopen_master_debug_command_t;

typedef struct {
    uint32_t command_sequence;
    canopen_master_debug_command_t command;
    uint8_t node_id;
    uint16_t index;
    uint8_t subindex;
    uint8_t size;
    int32_t value;
    uint16_t controlword;
    int8_t mode;
    int32_t target_position;
    int32_t target_velocity;
    int16_t target_torque;
} canopen_master_debug_control_t;
```

Define:

```c
volatile canopen_master_debug_control_t g_canopen_master_debug_control;
```

Default command is `NONE`.

- [ ] **Step 2: Add SDO download state machine**

Use CANopenNode SDO client download APIs. The service must not block waiting for a response. It records:

- last command sequence;
- last command enum;
- last download index/subindex;
- byte size;
- value;
- abort code;
- success/error counters.

- [ ] **Step 3: Add NMT send path**

Use `CO_NMT_sendCommand` only when command sequence changes and command enum requests NMT. Do not send NMT from initialization.

- [ ] **Step 4: Map typed commands to CiA 402 objects**

Use config macros for object indexes:

- controlword;
- modes of operation;
- target position;
- target velocity;
- target torque.

- [ ] **Step 5: COM9 monitor**

Add:

```text
[ECU CANopen CMD] seq=... cmd=... node=... dl_ok=... dl_abort=... last=0x....:.. size=.. value=... abort=... err=...
```

- [ ] **Step 6: Verify**

Run:

```powershell
python tests\python\run_tests.py
cmake --build tmp\cmake_cpu0_canopen --config debug
```

Expected: tests pass and CANopen build passes.

## Task 7: Update warning-light mapping and clean raw protocol fallback paths

**Files:**
- Modify: `ecu/devices/src/warning_light_device.c`
- Modify: `ecu/config/include/ecu_config.h`
- Modify: `ecu/config/src/ecu_config.c`
- Modify: `docs/ecu-main-control-architecture.md`

- [ ] **Step 1: Update warning light register**

Use documented register `ECU_MODBUS_WARNING_LIGHT_REGISTER = 0x00C2`.

- [ ] **Step 2: Keep warning-light values named**

Map logical states to documented values:

- off: `0x0060`;
- left indicator: yellow flash;
- right indicator: yellow flash;
- user hazard: yellow flash;
- safety hazard: red flash plus buzzer if configured.

If a value remains vehicle-policy dependent, name it in config rather than literal in device code.

- [ ] **Step 3: Mark raw CANopen frame fallback as non-default**

Where CANopenNode owns CAN2, business motion output must not use the raw-frame fallback. Keep the high-level servo API but document that real command output for this debug build uses `canopen_master_service`.

- [ ] **Step 4: Verify**

Run:

```powershell
python tools\check_no_forbidden_patterns.py
python tests\python\run_tests.py
git diff --check
```

Expected: no forbidden project patterns and no whitespace errors.

## Task 8: Hardware debug checkpoint

**Files:**
- Modify: `docs/ecu-main-control-progress.md`

- [ ] **Step 1: Start virtual ADC on COM10**

Run:

```powershell
python tools\modbus\virtual_adc_module.py --port COM10 --baudrate 9600 --slave 1 --channels-mv 0,500,1000,1500,2000,2500,3000,3500
```

Expected: tool opens COM10 and waits for ECU function 04 requests.

- [ ] **Step 2: Build and download default firmware if testing Modbus only**

Run default CPU0 build and download `tmp/cmake_cpu0_sbus/output/demo.elf`.

- [ ] **Step 3: Build and download CANopen firmware if testing CAN2 commands**

Run CANopen CPU0 build and download `tmp/cmake_cpu0_canopen/output/demo.elf`.

- [ ] **Step 4: Observe COM9**

Expected:

- `[ECU MODBUS ADC]` counters increase;
- raw/mV values match virtual ADC;
- CANopen read diagnostics still show `sdo_abort=0`.

- [ ] **Step 5: Trigger CANopen debug commands from watch window**

Manually increment `g_canopen_master_debug_control.command_sequence` after setting command fields. Start with:

1. SDO write modes of operation.
2. SDO write controlword shutdown.
3. SDO write controlword switch on.
4. SDO write controlword enable operation.
5. SDO write target velocity with a small value.
6. NMT commands only when explicitly intended.

Expected: COM9 command line reports success or specific abort codes; CAN analyzer shows frames only after sequence changes.

- [ ] **Step 6: Record result**

Update `docs/ecu-main-control-progress.md` with:

- Modbus virtual ADC result;
- CANopen command debug result;
- any abort codes;
- whether main power/motor was connected;
- remaining open items.

## Final verification

Run:

```powershell
python tests\python\run_tests.py
python tools\check_no_forbidden_patterns.py
python -m py_compile tools\can\controlcan.py tools\can\can2_monitor.py tools\modbus\rtu_probe.py tools\modbus\rtu_codec.py tools\modbus\virtual_adc_module.py
cmake --build tmp\cmake_cpu0_sbus --config debug
cmake --build tmp\cmake_cpu0_canopen --config debug
git diff --check
```

Expected: all pass except allowed Git LF/CRLF warnings.
