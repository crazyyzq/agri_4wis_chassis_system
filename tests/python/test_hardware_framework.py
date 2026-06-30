"""Hardware binding framework contract tests."""

from __future__ import annotations

import pathlib
import re


def read(root: pathlib.Path, rel: str) -> str:
    path = root / rel
    assert path.exists(), f"missing {rel}"
    return path.read_text(encoding="utf-8")


def test_hardware_project_defaults_are_centralized(root: pathlib.Path) -> None:
    config_h = read(root, "ecu/config/include/ecu_config.h")
    config_c = read(root, "ecu/config/src/ecu_config.c")
    required = [
        "ECU_CAN1_POWER_BITRATE",
        "ECU_CAN2_MOTION_BITRATE",
        "ECU_CAN3_LIFT_HYDRAULIC_BITRATE",
        "ECU_CAN4_AUXILIARY_BITRATE",
        "ECU_POWER_BMS_COMMAND_PERIOD_MS",
        "ECU_POWER_DCDC48_COMMAND_PERIOD_MS",
        "ECU_POWER_DCDC12_COMMAND_PERIOD_MS",
        "ECU_POWER_DCAC_COMMAND_PERIOD_MS",
        "ECU_CANOPEN_DRIVE_FL_NODE_ID",
        "ECU_CANOPEN_STEER_FL_NODE_ID",
        "ECU_DIO_BRAKE_RELEASE_MASK",
        "ECU_DIO_HYDRAULIC_ENABLE_MASK",
        "ECU_DIO_MANAGED_OUTPUT_MASK",
        "ECU_HYD_VALVE_MANAGED_MASK",
        "ECU_ADC_EXTERNAL_MV_MAX",
        "ECU_MODBUS_ADC_SLAVE_ID",
        "ecu_hardware_config_default",
    ]
    for token in required:
        assert token in config_h or token in config_c, token


def test_no_informal_uncertainty_marker_in_code_or_tests(root: pathlib.Path) -> None:
    forbidden = "GU" + "ESS"
    lower_forbidden = "gu" + "ess"
    scanned_roots = [root / "ecu", root / "tests"]
    for folder in scanned_roots:
        for path in folder.rglob("*"):
            if not path.is_file():
                continue
            rel = path.relative_to(root).as_posix()
            if rel.startswith("ecu/sdk_env_v1.11.0/"):
                continue
            if rel.startswith("ecu/apps/agri_chassis_control_cpu0/build/"):
                continue
            if path.suffix.lower() not in {".c", ".h", ".py", ".md", ".cmake", ".txt"}:
                continue
            text = path.read_text(encoding="utf-8", errors="ignore")
            assert forbidden not in text, f"{rel}: remove informal uncertainty marker"
            assert lower_forbidden not in text.lower(), f"{rel}: remove informal uncertainty wording"


def test_configuration_open_items_document_exists(root: pathlib.Path) -> None:
    text = read(root, "docs/ecu-configuration-open-items.md")

    assert "CAN1 power bus" in text
    assert "250 kbit/s" in text
    assert "bit time 4 us" in text
    assert "BMS" in text
    assert "CAN2 motion bus" in text


def test_protocol_driver_and_device_layers_exist(root: pathlib.Path) -> None:
    required_files = [
        "ecu/drivers/can/include/can_bus_service.h",
        "ecu/drivers/can/src/can_bus_service.c",
        "ecu/drivers/canopen/include/canopen_master_service.h",
        "ecu/drivers/canopen/src/canopen_master_service.c",
        "ecu/drivers/dio/include/dio_service.h",
        "ecu/drivers/dio/src/dio_service.c",
        "ecu/drivers/adc/include/analog_input_service.h",
        "ecu/drivers/adc/src/analog_input_service.c",
        "ecu/drivers/uart/include/uart_comm_service.h",
        "ecu/drivers/uart/src/uart_comm_service.c",
        "ecu/devices/include/power_device.h",
        "ecu/devices/src/power_device.c",
        "ecu/devices/include/motion_device.h",
        "ecu/devices/src/motion_device.c",
        "ecu/devices/include/lift_hydraulic_device.h",
        "ecu/devices/src/lift_hydraulic_device.c",
        "ecu/devices/include/local_io_device.h",
        "ecu/devices/src/local_io_device.c",
        "ecu/devices/include/warning_light_device.h",
        "ecu/devices/src/warning_light_device.c",
    ]
    for rel in required_files:
        assert (root / rel).exists(), rel


def test_communication_stacks_use_sdk_middleware(root: pathlib.Path) -> None:
    cmake = read(root, "ecu/apps/agri_chassis_control_cpu0/CMakeLists.txt")
    user_config_h = read(root, "ecu/apps/agri_chassis_control_cpu0/src/user_config.h")
    sbus_h = read(root, "ecu/protocol/sbus/include/sbus_decoder.h")

    for token in [
        "set(CONFIG_AGILE_MODBUS 1)",
        "set(CONFIG_AGILE_MODBUS_RTU 1)",
        "set(CONFIG_CANOPEN 1)",
    ]:
        assert token in cmake, token

    assert "MAX_CANOPEN_DEVICE (2U)" in user_config_h
    assert "sbus_decode_frame" in sbus_h and "SBUS_CHANNEL_COUNT" in sbus_h


def test_canopen_and_modbus_protocols_are_library_backed(root: pathlib.Path) -> None:
    """Active ECU code must use CANopenNode and Agile Modbus, not local stacks."""

    ignored_prefixes = (
        "ecu/sdk_env_v1.11.0/",
        "ecu/apps/agri_chassis_control_cpu0/build/",
        "docs/superpowers/",
    )
    forbidden_patterns = [
        re.compile("can" + "open_frame"),
        re.compile(r'(?<!agile_)mod' + r'bus_rtu'),
    ]
    scanned_roots = [root / "ecu", root / "tests", root / "docs"]
    for folder in scanned_roots:
        for path in folder.rglob("*"):
            if not path.is_file():
                continue
            rel = path.relative_to(root).as_posix()
            if rel.startswith(ignored_prefixes):
                continue
            if path.suffix.lower() not in {".c", ".h", ".py", ".md", ".cmake", ".txt"}:
                continue
            text = path.read_text(encoding="utf-8", errors="ignore")
            for pattern in forbidden_patterns:
                match = pattern.search(text)
                assert match is None, f"{rel}: remove project-local protocol token {match.group(0)}"

    cmake = read(root, "ecu/apps/agri_chassis_control_cpu0/CMakeLists.txt")
    servo_c = read(root, "ecu/devices/src/servo_drive_canopen.c")
    analog_c = read(root, "ecu/devices/src/analog_modbus_device.c")
    warning_c = read(root, "ecu/devices/src/warning_light_device.c")
    service_c = read(root, "ecu/drivers/canopen/src/canopen_master_service.c")

    assert "sdk_app_src(../../drivers/canopen/src/canopen_master_service.c)" in cmake
    assert "sdk_app_src(../../protocol/canopen/src/" not in cmake
    assert "sdk_app_src(../../protocol/modbus/src/" not in cmake
    assert "canopen_master_service_request_sdo_write" in servo_c
    assert "agile_modbus_serialize_read_input_registers" in analog_c
    assert "agile_modbus_deserialize_read_input_registers" in analog_c
    assert "agile_modbus_serialize_write_register" in warning_c
    assert "CO_SDOclientDownloadInitiate" in service_c


def test_canopennode_ds301_od_and_build_switch(root: pathlib.Path) -> None:
    cmake = read(root, "ecu/apps/agri_chassis_control_cpu0/CMakeLists.txt")
    user_config_h = read(root, "ecu/apps/agri_chassis_control_cpu0/src/user_config.h")
    config_h = read(root, "ecu/config/include/ecu_config.h")
    od_h = read(root, "ecu/protocol/canopen/od/ds301/OD.h")
    od_c = read(root, "ecu/protocol/canopen/od/ds301/OD.c")

    assert "CANopenNode V4" in od_h
    assert "OD_PERSIST_COMM" in od_c
    assert "sdk_inc(../../protocol/canopen/od/ds301)" in cmake
    assert "sdk_app_src(../../protocol/canopen/od/ds301/OD.c)" in cmake
    assert "sdk_compile_definitions(-DECU_ENABLE_CANOPENNODE=1)" in cmake
    assert "sdk_compile_definitions(-DCONFIG_CANOPEN_MASTER=1)" in cmake
    assert 'sdk_compile_options("-Wno-unused-parameter")' in cmake
    assert "-Wno-macro-redefined" not in cmake
    assert "sdk_app_src(../../drivers/canopen/src/canopen_master_service.c)" in cmake
    canopen_errno_h = read(root, "ecu/apps/agri_chassis_control_cpu0/src/canopen_errno.h")
    assert "#ifndef EMSGSIZE" in canopen_errno_h
    assert "#define EMSGSIZE 122" in canopen_errno_h
    assert "without editing the ignored SDK environment" in canopen_errno_h
    assert "MAX_CANOPEN_DEVICE (2U)" in user_config_h
    for token in [
        "ECU_CANOPEN_BC2_DIAG_NODE_ID",
        "ECU_CANOPEN_OBJ_DEVICE_TYPE",
        "ECU_CANOPEN_OBJ_ERROR_REGISTER",
        "ECU_CANOPEN_OBJ_IDENTITY",
        "ECU_CANOPEN_OBJ_STATUSWORD",
        "ECU_CANOPEN_OBJ_MODES_OF_OPERATION_DISPLAY",
    ]:
        assert token in config_h, token


def test_canopennode_hpm_tx_path_is_nonblocking(root: pathlib.Path) -> None:
    cmake = read(root, "ecu/apps/agri_chassis_control_cpu0/CMakeLists.txt")
    wrapper_c = read(root, "ecu/drivers/canopen/src/hpm_can_send_nonblocking_wrap.c")

    assert "sdk_ld_options(\"-Wl,--wrap=hpm_can_send\")" in cmake
    assert "sdk_app_src(../../drivers/canopen/src/hpm_can_send_nonblocking_wrap.c)" in cmake
    assert "__wrap_hpm_can_send" in wrapper_c
    assert "busy-waits forever" in wrapper_c
    assert "can_send_high_priority_message_nonblocking" in wrapper_c
    assert "can_send_message_nonblocking" in wrapper_c
    assert "printf(" not in wrapper_c
    assert "Transmit failed" not in wrapper_c
    assert "while (!data->has_sent_out)" not in wrapper_c


def test_canopennode_can2_diagnostic_service_is_safe(root: pathlib.Path) -> None:
    service_h = read(root, "ecu/drivers/canopen/include/canopen_master_service.h")
    service_c = read(root, "ecu/drivers/canopen/src/canopen_master_service.c")
    tasks_c = read(root, "ecu/os/src/ecu_tasks_cpu0.c")
    monitor_h = read(root, "ecu/diag/include/runtime_monitor.h")
    monitor_c = read(root, "ecu/diag/src/runtime_monitor.c")

    for token in [
        "canopen_master_service_init",
        "canopen_master_service_process",
        "canopen_master_service_get_snapshot",
        "canopen_master_snapshot_t",
        "last_sdo_index",
        "last_sdo_abort_code",
    ]:
        assert token in service_h, token

    for token in [
        "BOARD_CAN2_BASE",
        "BOARD_CAN2_IRQn",
        "canopen_controller_init",
        "CO_CANinit",
        "CO_CANopenInit",
        "CO_process",
        "CO_SDOclient_setup",
        "CO_SDOclientUploadInitiate",
        "CO_SDOclientUpload",
        "ECU_CANOPEN_OBJ_STATUSWORD",
        "ECU_CANOPEN_OBJ_MODES_OF_OPERATION_DISPLAY",
    ]:
        assert token in service_c, token

    assert "ATTR_PLACE_AT_NONCACHEABLE_BSS" in service_c
    assert "g_canopen_master_debug_control" in service_c
    assert service_c.index("handle_debug_command(service);") < service_c.index("if (service->sdo_download_active)")
    assert "canopen_master_service_init(&s_runtime.can2_motion_canopen" in tasks_c
    assert "canopen_master_service_init(&s_runtime.can3_lift_hydraulic_canopen" in tasks_c
    assert "canopen_master_service_process(&s_runtime.can2_motion_canopen" in tasks_c
    assert "canopen_master_service_process(&s_runtime.can3_lift_hydraulic_canopen" in tasks_c
    assert "can2_canopen_initialized" in monitor_h
    assert "ECU CANopen CAN2" in monitor_c


def test_modbus_virtual_adc_and_rs485_master_are_structured(root: pathlib.Path) -> None:
    virtual_adc = read(root, "tools/modbus/virtual_adc_module.py")
    rtu_codec = read(root, "tools/modbus/rtu_codec.py")
    tasks_c = read(root, "ecu/os/src/ecu_tasks_cpu0.c")
    analog_modbus_c = read(root, "ecu/devices/src/analog_modbus_device.c")
    rs485_c = read(root, "ecu/drivers/uart/src/uart_rs485_hw.c")
    board_h = read(root, "ecu/ecu_isolation/board.h")
    pinmux_c = read(root, "ecu/ecu_isolation/pinmux.c")
    monitor_h = read(root, "ecu/diag/include/runtime_monitor.h")
    monitor_c = read(root, "ecu/diag/src/runtime_monitor.c")
    config_h = read(root, "ecu/config/include/ecu_config.h")
    config_c = read(root, "ecu/config/src/ecu_config.c")

    for token in [
        "Read Input Registers",
        "registers 0..7",
        "--port",
        "--slave",
        "--baudrate",
        "--channels-mv",
        "pop_next_request_frame",
        "verify_crc(frame)",
    ]:
        assert token in virtual_adc, token

    for forbidden in ["write_register", "write_multiple_registers"]:
        assert forbidden not in virtual_adc
        assert forbidden not in rtu_codec

    for rel in [
        "ecu/drivers/uart/include/uart_rs485_hw.h",
        "ecu/drivers/uart/src/uart_rs485_hw.c",
        "ecu/drivers/uart/include/modbus_master_service.h",
        "ecu/drivers/uart/src/modbus_master_service.c",
        "ecu/devices/include/analog_modbus_device.h",
        "ecu/devices/src/analog_modbus_device.c",
    ]:
        assert (root / rel).exists(), rel

    for token in [
        "ECU_MODBUS_ADC_BAUDRATE",
        "ECU_MODBUS_ADC_START_REGISTER",
        "ECU_MODBUS_ADC_REGISTER_COUNT",
        "ECU_MODBUS_ADC_RAW_MAX",
        "ECU_MODBUS_WARNING_LIGHT_REGISTER",
    ]:
        assert token in config_h or token in config_c, token

    assert "analog_modbus_device_process" in tasks_c
    assert "modbus_master_service_process" in analog_modbus_c
    assert "BOARD_RS485_1_UART_BASE" in rs485_c
    assert "#define BOARD_RS485_DE_USING_GPIO 1" in board_h
    assert "IOC_PD18_FUNC_CTL_GPIO_D_18" in pinmux_c
    assert "uart_rs485_1_set_transmit_direction" in rs485_c
    assert "uart_rs485_1_set_receive_direction" in rs485_c
    assert "analog_modbus_adc" in monitor_h
    assert "ECU MODBUS ADC" in monitor_c


def test_canopennode_debug_commands_are_sequence_gated(root: pathlib.Path) -> None:
    service_h = read(root, "ecu/drivers/canopen/include/canopen_master_service.h")
    service_c = read(root, "ecu/drivers/canopen/src/canopen_master_service.c")
    monitor_h = read(root, "ecu/diag/include/runtime_monitor.h")
    monitor_c = read(root, "ecu/diag/src/runtime_monitor.c")
    config_h = read(root, "ecu/config/include/ecu_config.h")

    for token in [
        "canopen_master_debug_control_t",
        "command_sequence",
        "CANOPEN_MASTER_DEBUG_COMMAND_NONE",
        "CANOPEN_MASTER_DEBUG_COMMAND_SDO_WRITE_CONTROLWORD",
        "CANOPEN_MASTER_DEBUG_COMMAND_SDO_WRITE_TARGET_VELOCITY",
        "CANOPEN_MASTER_DEBUG_COMMAND_NMT_OPERATIONAL",
    ]:
        assert token in service_h, token

    for token in [
        "ECU_CANOPEN_OBJ_CONTROLWORD",
        "ECU_CANOPEN_OBJ_MODES_OF_OPERATION",
        "ECU_CANOPEN_OBJ_TARGET_POSITION",
        "ECU_CANOPEN_OBJ_TARGET_VELOCITY",
        "ECU_CANOPEN_OBJ_TARGET_TORQUE",
    ]:
        assert token in config_h, token

    for token in [
        "volatile canopen_master_debug_control_t g_canopen_master_debug_control",
        "CO_SDOclientDownloadInitiate",
        "CO_SDOclientDownload(",
        "CO_NMT_sendCommand",
    ]:
        assert token in service_c, token

    init_body = service_c.split("canopen_master_service_init")[1].split("return true")[0]
    assert "CANOPEN_MASTER_DEBUG_COMMAND_NONE" in init_body
    assert "CO_NMT_sendCommand" not in init_body
    assert "last_command_sequence" in service_h
    assert "canopen_command" in monitor_h
    assert "ECU CANopen CMD" in monitor_c


def test_python_can_analyzer_and_modbus_tools_are_safe_by_default(root: pathlib.Path) -> None:
    controlcan_py = read(root, "tools/can/controlcan.py")
    monitor_py = read(root, "tools/can/can2_monitor.py")
    modbus_py = read(root, "tools/modbus/rtu_probe.py")

    for token in [
        "VCI_USBCAN2",
        "VCI_CAN_OBJ",
        "VCI_INIT_CONFIG",
        "VCI_OpenDevice",
        "VCI_InitCAN",
        "VCI_StartCAN",
        "VCI_Receive",
        "ControlCAN.dll",
    ]:
        assert token in controlcan_py, token

    assert "channel=1" in monitor_py
    assert "default=1000000" in monitor_py
    assert "receive" in monitor_py
    assert ".transmit(" not in monitor_py
    assert "COM10" in modbus_py
    assert "serial.Serial" in modbus_py
    assert "read_holding_registers" in modbus_py
    assert "write_register" not in modbus_py


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
        "SERVO_DRIVE_CONTROL_ENABLE_OPERATION",
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


def test_vehicle_canopen_node_mapping_matches_machine_interfaces(root: pathlib.Path) -> None:
    config_h = read(root, "ecu/config/include/ecu_config.h")
    config_c = read(root, "ecu/config/src/ecu_config.c")
    servo_h = read(root, "ecu/devices/include/servo_drive_canopen.h")
    servo_c = read(root, "ecu/devices/src/servo_drive_canopen.c")
    motion_c = read(root, "ecu/devices/src/motion_device.c")
    lift_c = read(root, "ecu/devices/src/lift_hydraulic_device.c")
    requirements = read(root, "doc/ECU_Project_Implementation_v1.4.md")

    expected_nodes = {
        "ECU_CANOPEN_LEG1_DRIVE_NODE_ID": "0x01U",
        "ECU_CANOPEN_LEG2_DRIVE_NODE_ID": "0x02U",
        "ECU_CANOPEN_LEG3_DRIVE_NODE_ID": "0x03U",
        "ECU_CANOPEN_LEG4_DRIVE_NODE_ID": "0x04U",
        "ECU_CANOPEN_LEG1_STEER_NODE_ID": "0x05U",
        "ECU_CANOPEN_LEG2_STEER_NODE_ID": "0x06U",
        "ECU_CANOPEN_LEG3_STEER_NODE_ID": "0x07U",
        "ECU_CANOPEN_LEG4_STEER_NODE_ID": "0x08U",
        "ECU_CANOPEN_LIFT_LEG1_NODE_ID": "0x09U",
        "ECU_CANOPEN_LIFT_LEG2_NODE_ID": "0x0AU",
        "ECU_CANOPEN_LIFT_LEG3_NODE_ID": "0x0BU",
        "ECU_CANOPEN_LIFT_LEG4_NODE_ID": "0x0CU",
        "ECU_CANOPEN_HYDRAULIC_PUMP_NODE_ID": "0x0DU",
    }
    for name, value in expected_nodes.items():
        assert re.search(rf"#define\s+{name}\s+\({value}\)", config_h), name

    drive_block = re.search(r"\.drive_nodes\s*=\s*\{(?P<body>[\s\S]*?)\n\s*\},", config_c)
    steer_block = re.search(r"\.steer_nodes\s*=\s*\{(?P<body>[\s\S]*?)\n\s*\},", config_c)
    lift_block = re.search(r"\.lift_nodes\s*=\s*\{(?P<body>[\s\S]*?)\n\s*\},", config_c)
    assert drive_block and steer_block and lift_block
    assert [
        "ECU_CANOPEN_LEG1_DRIVE_NODE_ID",
        "ECU_CANOPEN_LEG2_DRIVE_NODE_ID",
        "ECU_CANOPEN_LEG3_DRIVE_NODE_ID",
        "ECU_CANOPEN_LEG4_DRIVE_NODE_ID",
    ] == re.findall(r"ECU_CANOPEN_LEG\d_DRIVE_NODE_ID", drive_block.group("body"))
    assert [
        "ECU_CANOPEN_LEG1_STEER_NODE_ID",
        "ECU_CANOPEN_LEG2_STEER_NODE_ID",
        "ECU_CANOPEN_LEG3_STEER_NODE_ID",
        "ECU_CANOPEN_LEG4_STEER_NODE_ID",
    ] == re.findall(r"ECU_CANOPEN_LEG\d_STEER_NODE_ID", steer_block.group("body"))
    assert [
        "ECU_CANOPEN_LIFT_LEG1_NODE_ID",
        "ECU_CANOPEN_LIFT_LEG2_NODE_ID",
        "ECU_CANOPEN_LIFT_LEG3_NODE_ID",
        "ECU_CANOPEN_LIFT_LEG4_NODE_ID",
    ] == re.findall(r"ECU_CANOPEN_LIFT_LEG\d_NODE_ID", lift_block.group("body"))

    for token in [
        "ECU_CANOPEN_OBJ_DIGITAL_INPUT_STATES",
        "ECU_CANOPEN_OBJ_OUTPUT_STATES_PROGRAM_CONTROL",
        "ECU_SERVO_BRAKE_RELEASE_OUTPUT_ACTIVE_LEVEL",
        "SERVO_DRIVE_OUTPUT_OUT1_MASK",
        "SERVO_DRIVE_OUTPUT_OUT4_MASK",
        "SERVO_DRIVE_INPUT_IN2_MASK",
        "SERVO_DRIVE_INPUT_IN3_MASK",
        "SERVO_DRIVE_INPUT_IN7_MASK",
        "SERVO_DRIVE_INPUT_IN8_MASK",
        "servo_drive_canopen_set_output_state",
        "servo_drive_canopen_read_input_states",
    ]:
        assert token in config_h or token in servo_h or token in servo_c, token

    assert "servo_drive_canopen_set_output_state" in motion_c
    assert "SERVO_DRIVE_OUTPUT_OUT1_MASK" in motion_c
    assert "servo_drive_canopen_read_input_states" in motion_c
    assert "SERVO_DRIVE_INPUT_IN2_MASK" in motion_c
    assert "SERVO_DRIVE_INPUT_IN3_MASK" in motion_c
    assert "servo_drive_canopen_set_output_state" in lift_c
    assert "SERVO_DRIVE_OUTPUT_OUT1_MASK" in lift_c
    assert "BC2_AXIS_OUTPUT_BRAKE_MASK" in lift_c
    assert "BC2_AXIS_INPUT_POSITIVE_LIMIT_MASK" in lift_c
    assert "BC2_AXIS_INPUT_NEGATIVE_LIMIT_MASK" in lift_c
    assert "SERVO_DRIVE_OUTPUT_OUT4_MASK" not in lift_c
    assert "SERVO_DRIVE_INPUT_IN7_MASK" not in lift_c
    assert "SERVO_DRIVE_INPUT_IN8_MASK" not in lift_c

    for phrase in [
        "CAN2 是控制整车的行走和转向部分",
        "J3 中的引脚 16 OUT1",
        "IN2 是正限位，IN3 是负限位",
        "CAN3 是整车的抬升功能",
        "BC2 的 SW 拨码只设置 A 轴节点号，B 轴节点号等于 A 轴节点号加 1",
        "A 轴电机的抱闸是控制信号 I/O 端子 J3 的引脚 8 OUT1",
        "B 轴电机的抱闸是引脚 17 OUT4",
        "B 轴节点访问 `0x2194` 时仍写轴内 OUT1 对应的 bit0",
        "0x2194",
        "0x2190",
    ]:
        assert phrase in requirements, phrase


def test_sbus_remote_logic_maps_protocol_raw_to_ppm_equivalent(root: pathlib.Path) -> None:
    config_h = read(root, "ecu/config/include/ecu_config.h")
    config_c = read(root, "ecu/config/src/ecu_config.c")
    tasks_c = read(root, "ecu/os/src/ecu_tasks_cpu0.c")
    decoder_c = read(root, "ecu/protocol/sbus/src/sbus_decoder.c")
    monitor_h = read(root, "ecu/diag/include/runtime_monitor.h")
    monitor_c = read(root, "ecu/diag/src/runtime_monitor.c")

    expected_defines = {
        "ECU_SBUS_PROTOCOL_RAW_LOW": "282U",
        "ECU_SBUS_PROTOCOL_RAW_CENTER": "1002U",
        "ECU_SBUS_PROTOCOL_RAW_HIGH": "1722U",
        "ECU_SBUS_PPM_LOW": "1050U",
        "ECU_SBUS_PPM_CENTER": "1500U",
        "ECU_SBUS_PPM_HIGH": "1950U",
        "ECU_SBUS_PPM_LOW_MAX": "1200U",
        "ECU_SBUS_PPM_CENTER_MIN": "1400U",
        "ECU_SBUS_PPM_CENTER_MAX": "1600U",
        "ECU_SBUS_PPM_HIGH_MIN": "1800U",
        "ECU_SBUS_PPM_CREDIBLE_MIN": "1000U",
        "ECU_SBUS_PPM_CREDIBLE_MAX": "2000U",
    }
    for name, value in expected_defines.items():
        assert re.search(rf"#define\s+{name}\s+\({value}\)", config_h), name

    for field in [
        ".stick_min = ECU_SBUS_PPM_LOW",
        ".stick_neutral = ECU_SBUS_PPM_CENTER",
        ".stick_max = ECU_SBUS_PPM_HIGH",
        ".throttle_min = ECU_SBUS_PPM_LOW",
        ".throttle_max = ECU_SBUS_PPM_HIGH",
    ]:
        assert field in config_c, field

    assert "0x07FFU" in decoder_c
    assert "sbus_protocol_raw_to_ppm_equivalent" in tasks_c
    assert "sbus_build_ppm_snapshot" in tasks_c
    assert "sbus_build_ppm_snapshot(&s_runtime.sbus_snapshot, &remote_sbus)" in tasks_c
    assert tasks_c.index("sbus_service_get_snapshot(&s_runtime.sbus") < tasks_c.index(
        "sbus_build_ppm_snapshot(&s_runtime.sbus_snapshot, &remote_sbus)"
    )
    assert tasks_c.index("sbus_build_ppm_snapshot(&s_runtime.sbus_snapshot, &remote_sbus)") < tasks_c.index(
        "build_remote_input_snapshot(&remote_sbus"
    )
    assert "sbus_ppm_channels[ECU_SBUS_CHANNEL_COUNT]" in monitor_h
    assert "ECU SBUS RAW" in monitor_c
    assert "ECU SBUS PPM" in monitor_c

    stick_func = re.search(r"static int16_t sbus_per_mille_from_raw[\s\S]*?\n}", tasks_c)
    assert stick_func is not None
    assert "thresholds->stick_neutral" in stick_func.group(0)
    assert "thresholds->stick_max" in stick_func.group(0)
    assert "thresholds->stick_min" in stick_func.group(0)
    assert "thresholds->high_min" not in stick_func.group(0)
    assert "thresholds->low_max" not in stick_func.group(0)

def test_can3_and_rgb_status_are_enabled_for_whole_machine(root: pathlib.Path) -> None:
    config_h = read(root, "ecu/config/include/ecu_config.h")
    tasks_c = read(root, "ecu/os/src/ecu_tasks_cpu0.c")
    cmake = read(root, "ecu/apps/agri_chassis_control_cpu0/CMakeLists.txt")

    assert re.search(r"#define\s+ECU_ENABLE_CAN3_LIFT_CANOPEN\s+\(1\)", config_h)
    assert "#if ECU_ENABLE_CAN3_LIFT_CANOPEN" not in tasks_c
    assert "canopen_master_service_init(&s_runtime.can3_lift_hydraulic_canopen" in tasks_c
    assert "canopen_master_service_process(&s_runtime.can3_lift_hydraulic_canopen" in tasks_c

    for rel in [
        "ecu/drivers/status_led/include/status_led_service.h",
        "ecu/drivers/status_led/src/status_led_service.c",
    ]:
        assert (root / rel).exists(), rel

    led_h = read(root, "ecu/drivers/status_led/include/status_led_service.h")
    led_c = read(root, "ecu/drivers/status_led/src/status_led_service.c")
    for token in [
        "status_led_service_update",
        "STATUS_LED_PATTERN_BOOT",
        "STATUS_LED_PATTERN_READY",
        "STATUS_LED_PATTERN_WARNING",
        "STATUS_LED_PATTERN_ESTOP",
        "board_rgb_write",
    ]:
        assert token in led_h or token in led_c, token

    assert "status_led_service_t status_led" in tasks_c
    assert "status_led_service_init(&s_runtime.status_led" in tasks_c
    assert "status_led_service_update(&s_runtime.status_led" in tasks_c
    assert "status_led_service.c" in cmake


def test_motion_and_lift_canopen_outputs_are_command_gated(root: pathlib.Path) -> None:
    motion_h = read(root, "ecu/devices/include/motion_device.h")
    motion_c = read(root, "ecu/devices/src/motion_device.c")
    lift_h = read(root, "ecu/devices/include/lift_hydraulic_device.h")
    lift_c = read(root, "ecu/devices/src/lift_hydraulic_device.c")

    assert "command_source_allows_motion_output" in motion_c
    assert "command_source_allows_lift_output" in lift_c
    assert "COMMAND_SOURCE_NONE" not in re.search(
        r"command_source_allows_motion_output[\s\S]*?\n}",
        motion_c,
    ).group(0)
    assert "COMMAND_SOURCE_NONE" not in re.search(
        r"command_source_allows_lift_output[\s\S]*?\n}",
        lift_c,
    ).group(0)
    assert "command_changed(state, command)" in motion_c
    assert "lift_command_changed(state, command)" in lift_c
    assert "last_motion_command_valid" in motion_h
    assert "last_lift_command_valid" in lift_h
    assert "skipped_count" in motion_h
    assert "skipped_lift_canopen_count" in lift_h


def test_executor_fans_out_only_through_device_adapters(root: pathlib.Path) -> None:
    executor = read(root, "ecu/vehicle/src/vehicle_command_executor.c")
    required_calls = [
        "motion_device_apply",
        "lift_hydraulic_device_apply",
        "local_io_device_apply",
        "warning_light_device_apply",
    ]
    for token in required_calls:
        assert token in executor, token
    assert "executor->power_result" in executor
    for forbidden in ["hpm_can", "hpm_gpio", "board_", "HPM_CAN", "HPM_GPIO"]:
        assert forbidden not in executor, forbidden


def test_remote_and_control_do_not_depend_on_devices(root: pathlib.Path) -> None:
    forbidden = ["power_device", "motion_device", "lift_hydraulic_device", "local_io_device"]
    for folder in ["ecu/remote", "ecu/control"]:
        for path in (root / folder).rglob("*.[ch]"):
            text = path.read_text(encoding="utf-8")
            for token in forbidden:
                assert token not in text, f"{path}: forbidden {token}"


def test_no_raw_canopen_project_defaults_outside_config(root: pathlib.Path) -> None:
    canopen_default_pattern = re.compile(r"\b(0x18[0-9A-Fa-f]{2}|0x20[0-9A-Fa-f]{2}|0x60[0-9A-Fa-f]{2}|0x70[0-9A-Fa-f]{2})\b")
    for path in (root / "ecu").rglob("*.[ch]"):
        rel = path.relative_to(root).as_posix()
        if (
            rel.startswith("ecu/config/") or
            rel.startswith("ecu/protocol/canopen/od/") or
            rel.startswith("ecu/sdk_env_v1.11.0/")
        ):
            continue
        text = path.read_text(encoding="utf-8")
        match = canopen_default_pattern.search(text)
        assert match is None, f"{rel}: CANopen raw value {match.group(0)} belongs in config"


def test_dio_active_low_is_limited_to_managed_outputs(root: pathlib.Path) -> None:
    dio_h = read(root, "ecu/drivers/dio/include/dio_service.h")
    dio_c = read(root, "ecu/drivers/dio/src/dio_service.c")
    lift_c = read(root, "ecu/devices/src/lift_hydraulic_device.c")

    assert "managed_output_mask" in dio_h
    assert "(~logical) & service->managed_output_mask" in dio_c
    assert "config->hydraulic_managed_valve_mask, false" in lift_c


def test_default_dio_and_hydraulic_masks_do_not_overlap(root: pathlib.Path) -> None:
    config_h = read(root, "ecu/config/include/ecu_config.h")

    def bit_value(name: str) -> int:
        match = re.search(rf"#define\s+{name}\s+\(1UL\s*<<\s*(\d+)\)", config_h)
        assert match, f"missing single-bit macro {name}"
        return 1 << int(match.group(1))

    dio_names = [
        "ECU_DIO_BRAKE_RELEASE_MASK",
        "ECU_DIO_HYDRAULIC_ENABLE_MASK",
        "ECU_DIO_HORN_MASK",
        "ECU_DIO_HEADLIGHT_MASK",
        "ECU_DIO_LEFT_INDICATOR_MASK",
        "ECU_DIO_RIGHT_INDICATOR_MASK",
    ]
    hydraulic_names = [
        "ECU_HYD_VALVE_TRACK_EXTEND_MASK",
        "ECU_HYD_VALVE_TRACK_RETRACT_MASK",
        "ECU_HYD_VALVE_LIFT_UP_MASK",
        "ECU_HYD_VALVE_LIFT_DOWN_MASK",
    ]

    dio_mask = 0
    for name in dio_names:
        dio_mask |= bit_value(name)
    hydraulic_mask = 0
    for name in hydraulic_names:
        hydraulic_mask |= bit_value(name)

    assert (dio_mask & hydraulic_mask) == 0


def test_cpu0_runtime_monitor_is_configurable_and_task_owned(root: pathlib.Path) -> None:
    config_h = read(root, "ecu/config/include/ecu_config.h")
    monitor_h = read(root, "ecu/diag/include/runtime_monitor.h")
    monitor_c = read(root, "ecu/diag/src/runtime_monitor.c")
    tasks_c = read(root, "ecu/os/src/ecu_tasks_cpu0.c")
    cmake = read(root, "ecu/apps/agri_chassis_control_cpu0/CMakeLists.txt")

    for token in [
        "ECU_ENABLE_DEBUG_MONITOR",
        "ECU_DEBUG_MONITOR_PERIOD_MS",
        "ECU_DEBUG_MONITOR_VERBOSE",
    ]:
        assert token in config_h, token

    assert "runtime_monitor_snapshot_t" in monitor_h
    assert "runtime_monitor_print_cpu0" in monitor_h
    assert "printf(" in monitor_c
    assert "runtime_monitor_print_cpu0" in tasks_c
    assert "ECU_ENABLE_DEBUG_MONITOR" in tasks_c
    assert "runtime_monitor.c" in cmake


def test_cpu0_segger_project_uses_jlink_debug_connection(root: pathlib.Path) -> None:
    cmake = read(root, "ecu/apps/agri_chassis_control_cpu0/CMakeLists.txt")

    assert 'sdk_ses_opt_debug_connection("J-Link")' in cmake
    assert "sdk_ses_opt_debug_jlink_speed(4000)" in cmake


def test_sbus_uart1_idle_interrupt_is_bound_to_service(root: pathlib.Path) -> None:
    sbus_hw_h = read(root, "ecu/drivers/sbus/include/sbus_uart_hw.h")
    sbus_hw_c = read(root, "ecu/drivers/sbus/src/sbus_uart_hw.c")
    sbus_service_h = read(root, "ecu/drivers/sbus/include/sbus_service.h")
    sbus_service_c = read(root, "ecu/drivers/sbus/src/sbus_service.c")
    config_h = read(root, "ecu/config/include/ecu_config.h")
    tasks_c = read(root, "ecu/os/src/ecu_tasks_cpu0.c")
    monitor_h = read(root, "ecu/diag/include/runtime_monitor.h")
    monitor_c = read(root, "ecu/diag/src/runtime_monitor.c")
    cmake = read(root, "ecu/apps/agri_chassis_control_cpu0/CMakeLists.txt")

    for token in [
        "sbus_uart_hw_init",
        "BOARD_SBUS_UART_BASE",
        "BOARD_SBUS_BAUDRATE",
        "BOARD_SBUS_UART_IRQ",
        "uart_intr_rx_line_idle",
        "uart_intr_rx_data_avail_or_timeout",
        "SDK_DECLARE_EXT_ISR_M",
        "xTaskGetTickCountFromISR",
        "sbus_service_feed_byte_from_isr",
        "sbus_service_note_rx_idle_from_isr",
        "parity_even",
        "stop_bits_2",
    ]:
        assert token in sbus_hw_h or token in sbus_hw_c, token

    assert "sbus_service_note_rx_idle_from_isr" in sbus_service_h
    assert "frame_position = 0U" in sbus_service_c
    assert "ECU_SBUS_UART_RX_IDLE_BITS" in config_h
    assert "ECU_SBUS_UART_IRQ_PRIORITY" in config_h
    assert "sbus_uart_hw_init(&s_runtime.sbus)" in tasks_c
    assert "sbus_uart_hw.c" in cmake
    assert "sbus_channels[ECU_SBUS_CHANNEL_COUNT]" in monitor_h
    assert "ECU SBUS" in monitor_c


def test_can2_can3_motion_and_lift_buses_are_tx_capable(root: pathlib.Path) -> None:
    can_hw_h = read(root, "ecu/drivers/can/include/can_bus_hw.h")
    can_hw_c = read(root, "ecu/drivers/can/src/can_bus_hw.c")
    can_service_h = read(root, "ecu/drivers/can/include/can_bus_service.h")
    can_service_c = read(root, "ecu/drivers/can/src/can_bus_service.c")
    config_h = read(root, "ecu/config/include/ecu_config.h")
    tasks_c = read(root, "ecu/os/src/ecu_tasks_cpu0.c")
    monitor_h = read(root, "ecu/diag/include/runtime_monitor.h")
    monitor_c = read(root, "ecu/diag/src/runtime_monitor.c")
    cmake = read(root, "ecu/apps/agri_chassis_control_cpu0/CMakeLists.txt")

    assert re.search(r"#define\s+ECU_CAN1_POWER_BITRATE\s+\(250000UL\)", config_h)
    assert re.search(r"#define\s+ECU_CAN2_MOTION_BITRATE\s+\(1000000UL\)", config_h)
    assert "ECU_CAN3_LIFT_HYDRAULIC_BITRATE" in config_h
    assert "can_bus_hw_init_can2_rx_only" in can_hw_h
    assert "can_bus_hw_init_can2_motion" in can_hw_h
    assert "can_bus_hw_init_can3_lift_hydraulic" in can_hw_h
    assert "can_bus_hw_poll_can2_rx" in can_hw_h
    assert "can_bus_hw_poll_can3_rx" in can_hw_h
    assert "can_bus_hw_send_can2_frame" in can_hw_h
    assert "can_bus_hw_send_can3_frame" in can_hw_h
    assert "BOARD_CAN2_BASE" in can_hw_c
    assert "BOARD_CAN2_IRQn" in can_hw_c
    assert "BOARD_CAN3_BASE" in can_hw_c
    assert "BOARD_CAN3_IRQn" in can_hw_c
    assert "can_get_default_config" in can_hw_c
    assert "can_config.baudrate = bitrate" in can_hw_c
    assert "CAN_EVENT_RECEIVE" in can_hw_c
    assert "can_read_received_message" in can_hw_c
    assert "can_is_data_available_in_receive_buffer" in can_hw_c
    assert "can_get_receive_buffer_status" in can_hw_c
    assert "can_get_receive_error_count" in can_hw_c
    assert "can_get_transmit_error_count" in can_hw_c
    assert "can_get_last_error_kind" in can_hw_c
    assert "SDK_DECLARE_EXT_ISR_M" in can_hw_c
    assert "can_bus_hw_init_can2_rx_only" in can_hw_c
    assert "can_bus_service_set_tx_backend(service, 0)" in can_hw_c
    assert "can_bus_service_set_tx_backend(service, can_bus_hw_send_can2_frame)" in can_hw_c
    assert "can_bus_service_set_tx_backend(service, can_bus_hw_send_can3_frame)" in can_hw_c
    assert "can_bus_service_note_rx_from_isr" in can_service_h
    assert "can_bus_service_note_error_from_isr" in can_service_h
    assert "can2_motion_canopen" in tasks_c
    assert "can3_lift_hydraulic_canopen" in tasks_c
    assert "CANOPEN_MASTER_BUS_CAN2" in tasks_c
    assert "CANOPEN_MASTER_BUS_CAN3" in tasks_c
    assert "ECU_ENABLE_CAN3_LIFT_CANOPEN" in config_h
    assert "#if ECU_ENABLE_CAN3_LIFT_CANOPEN" not in tasks_c
    assert "canopen_master_service_process(&s_runtime.can2_motion_canopen" in tasks_c
    assert "canopen_master_service_process(&s_runtime.can3_lift_hydraulic_canopen" in tasks_c
    assert "can2_rx_count" in monitor_h
    assert "can2_rx_buffer_status" in monitor_h
    assert "can2_receive_error_count" in monitor_h
    assert "ECU CAN2" in monitor_c
    assert "can_bus_hw.c" in cmake


def test_can_and_sbus_isr_snapshots_are_copied_atomically(root: pathlib.Path) -> None:
    sbus_service_c = read(root, "ecu/drivers/sbus/src/sbus_service.c")
    can_service_h = read(root, "ecu/drivers/can/include/can_bus_service.h")
    can_service_c = read(root, "ecu/drivers/can/src/can_bus_service.c")
    tasks_c = read(root, "ecu/os/src/ecu_tasks_cpu0.c")

    assert "taskENTER_CRITICAL" in sbus_service_c
    assert "taskEXIT_CRITICAL" in sbus_service_c
    assert "can_bus_service_get_snapshot" in can_service_h
    assert "taskENTER_CRITICAL" in can_service_c
    assert "taskEXIT_CRITICAL" in can_service_c
    assert "can_bus_service_get_snapshot(&s_runtime.can1_power" in tasks_c


def test_rs4852_warning_light_protocol_is_hardware_bound(root: pathlib.Path) -> None:
    config_h = read(root, "ecu/config/include/ecu_config.h")
    config_c = read(root, "ecu/config/src/ecu_config.c")
    rs485_h = read(root, "ecu/drivers/uart/include/uart_rs485_hw.h")
    rs485_c = read(root, "ecu/drivers/uart/src/uart_rs485_hw.c")
    modbus_c = read(root, "ecu/drivers/uart/src/modbus_master_service.c")
    warning_h = read(root, "ecu/devices/include/warning_light_device.h")
    warning_c = read(root, "ecu/devices/src/warning_light_device.c")
    tasks_c = read(root, "ecu/os/src/ecu_tasks_cpu0.c")

    for token in [
        "ECU_MODBUS_WARNING_LIGHT_BAUDRATE",
        "ECU_MODBUS_WARNING_LIGHT_REQUEST_PERIOD_MS",
        "ECU_MODBUS_WARNING_LIGHT_RESPONSE_TIMEOUT_MS",
        "ECU_WARNING_LIGHT_VALUE_OFF",
        "ECU_WARNING_LIGHT_VALUE_YELLOW_SLOW_FLASH",
        "ECU_WARNING_LIGHT_VALUE_RED_STEADY_BUZZER",
    ]:
        assert token in config_h or token in config_c, token

    for token in [
        "uart_rs485_hw_send",
        "uart_rs485_hw_read",
        "uart_rs485_hw_clear_rx",
        "uart_rs485_2_hw_init",
        "uart_rs485_2_hw_isr",
        "BOARD_RS485_2_UART_BASE",
        "BOARD_RS485_2_UART_IRQ",
        "BOARD_RS485_2_DE_GPIO_CTRL",
    ]:
        assert token in rs485_h or token in rs485_c, token

    assert "uart_rs485_hw_send(uart" in modbus_c
    assert "uart_rs485_hw_read(uart" in modbus_c
    assert "uart_rs485_hw_clear_rx(uart" in modbus_c
    assert "modbus_master_service_t *master" in warning_h
    assert "modbus_master_service_process" in warning_c
    assert "agile_modbus_serialize_write_register" in warning_c
    assert "config->modbus_warning_light_register" in warning_c
    assert "rs485_2_hw" in tasks_c
    assert "warning_light_modbus_master" in tasks_c
    assert "uart_rs485_2_hw_init(&s_runtime.rs485_2_hw" in tasks_c


def test_executor_uses_cpu0_owned_hardware_services(root: pathlib.Path) -> None:
    executor_h = read(root, "ecu/vehicle/include/vehicle_command_executor.h")
    executor_c = read(root, "ecu/vehicle/src/vehicle_command_executor.c")
    tasks_c = read(root, "ecu/os/src/ecu_tasks_cpu0.c")

    for token in [
        "vehicle_executor_io_t",
        "canopen_master_service_t *can2_motion_canopen",
        "canopen_master_service_t *can3_lift_hydraulic_canopen",
        "dio_service_t *dio",
        "uart_rs485_hw_t *warning_light_uart",
        "modbus_master_service_t *warning_light_modbus",
    ]:
        assert token in executor_h, token

    for forbidden in [
        "can_bus_service_init(&s_runtime.can2_motion",
        "can_bus_service_init(&s_runtime.can3_lift_hydraulic",
        "dio_service_init(&s_runtime.dio",
        "uart_comm_service_init(&s_runtime.rs485",
    ]:
        assert forbidden not in executor_c, forbidden

    assert "vehicle_executor_io_t executor_io" in tasks_c
    assert ".can2_motion_canopen = &s_runtime.can2_motion_canopen" in tasks_c
    assert ".can3_lift_hydraulic_canopen = &s_runtime.can3_lift_hydraulic_canopen" in tasks_c
    assert ".dio = &s_runtime.dio" in tasks_c
    assert ".warning_light_uart = &s_runtime.rs485_2_hw" in tasks_c
    assert "vehicle_command_executor_apply(&s_runtime.executor, &executor_io" in tasks_c


def test_dio_outputs_drive_board_gpio(root: pathlib.Path) -> None:
    dio_h = read(root, "ecu/drivers/dio/include/dio_service.h")
    dio_c = read(root, "ecu/drivers/dio/src/dio_service.c")
    dio_hw_h = read(root, "ecu/drivers/dio/include/dio_hw.h")
    dio_hw_c = read(root, "ecu/drivers/dio/src/dio_hw.c")
    board_c = read(root, "ecu/ecu_isolation/board.c")
    cmake = read(root, "ecu/apps/agri_chassis_control_cpu0/CMakeLists.txt")
    tasks_c = read(root, "ecu/os/src/ecu_tasks_cpu0.c")

    assert "dio_service_apply_backend_t" in dio_h
    assert "dio_service_set_apply_backend" in dio_h
    assert "service->apply_backend" in dio_c
    assert "dio_hw_attach_outputs" in dio_hw_h
    assert "board_ecu_output_write" in dio_hw_c
    assert "BOARD_ECU_OUTPUT_COUNT" in dio_hw_c
    assert "board_ecu_output_write" in board_c
    assert "dio_hw.c" in cmake
    assert "dio_hw_attach_outputs(&s_runtime.dio)" in tasks_c


def test_remote_command_generation_uses_sbus_analog_channels(root: pathlib.Path) -> None:
    remote_h = read(root, "ecu/remote/include/remote_types.h")
    manager_c = read(root, "ecu/remote/src/remote_manager.c")
    tasks_c = read(root, "ecu/os/src/ecu_tasks_cpu0.c")
    arbiter_c = read(root, "ecu/vehicle/src/command_arbiter.c")
    config_h = read(root, "ecu/config/include/ecu_config.h")

    for token in [
        "steer_per_mille",
        "throttle_per_mille",
        "clearance_per_mille",
        "track_per_mille",
        "r1_changed",
        "r2_changed",
    ]:
        assert token in remote_h, token

    assert "manager->request.steer_per_mille = input->steer_per_mille" in manager_c
    assert "manager->request.throttle_per_mille = input->throttle_per_mille" in manager_c
    assert "sbus_per_mille_from_raw" in tasks_c
    assert "decode_error_limit =" in tasks_c
    assert "credibility_error =" in tasks_c
    assert "motion_control_build_candidate" in arbiter_c
    assert "ECU_REMOTE_MAX_SPEED_KPH" in config_h
    assert "ECU_REMOTE_MAX_STEER_DEG" in config_h
    assert "ECU_REMOTE_MAX_HEIGHT_RATE_MM_S" in config_h
    assert "ECU_REMOTE_MAX_TRACK_RATE_MM_S" in config_h


def test_cpu0_runtime_initialization_is_explicit_and_keeps_hardware_init_out_of_critical(root: pathlib.Path) -> None:
    main_c = read(root, "ecu/apps/agri_chassis_control_cpu0/src/main_cpu0.c")
    tasks_h = read(root, "ecu/os/include/ecu_tasks.h")
    tasks_c = read(root, "ecu/os/src/ecu_tasks_cpu0.c")

    assert "ecu_task_runtime_init(0U)" in main_c
    assert "void ecu_task_runtime_init(uint32_t now_ms);" in tasks_h
    assert "void ecu_task_runtime_init(uint32_t now_ms)" in tasks_c
    assert "taskENTER_CRITICAL" not in tasks_c


def test_can1_foreground_poll_has_a_frame_budget(root: pathlib.Path) -> None:
    can_hw_c = read(root, "ecu/drivers/can/src/can_bus_hw.c")

    assert "CAN_BUS_HW_MAX_RX_FRAMES_PER_POLL" in can_hw_c
    assert "frames_drained" in can_hw_c
    assert "frames_drained < CAN_BUS_HW_MAX_RX_FRAMES_PER_POLL" in can_hw_c


def test_cpu0_preconditions_are_feedback_driven(root: pathlib.Path) -> None:
    tasks_c = read(root, "ecu/os/src/ecu_tasks_cpu0.c")
    config_h = read(root, "ecu/config/include/ecu_config.h")
    monitor_h = read(root, "ecu/diag/include/runtime_monitor.h")
    monitor_c = read(root, "ecu/diag/src/runtime_monitor.c")

    assert "ecu_hardware_feedback_snapshot_t" in config_h
    assert "hardware_feedback" in tasks_c
    assert "out->power_ready = true;" not in tasks_c
    assert "out->low_voltage_ok = true;" not in tasks_c
    assert "out->can1_power_online = true;" not in tasks_c
    assert "out->power_ready = s_runtime.hardware_feedback.power_ready;" in tasks_c
    assert "out->low_voltage_ok = s_runtime.hardware_feedback.low_voltage_ok;" in tasks_c
    assert "out->can1_power_online = s_runtime.hardware_feedback.can1_power_online;" in tasks_c
    assert "hardware_feedback" in monitor_h
    assert "[ECU HW]" in monitor_c


def test_unknown_power_protocol_is_safe_by_default(root: pathlib.Path) -> None:
    config_h = read(root, "ecu/config/include/ecu_config.h")
    power_h = read(root, "ecu/devices/include/power_device.h")
    power_c = read(root, "ecu/devices/src/power_device.c")

    assert "ECU_POWER_PROTOCOL_DISABLED" in config_h
    assert "power_protocol" in config_h
    assert "ECU_DEVICE_APPLY_UNCONFIGURED" in read(root, "ecu/common/include/ecu_types.h")
    assert "power_device_apply" in power_h
    assert "send_power_node" not in power_c
    assert "ECU_DEVICE_APPLY_UNCONFIGURED" in power_c
    assert "high_voltage_enable" in power_c


def test_no_transitional_language_in_active_engineering_files(root: pathlib.Path) -> None:
    forbidden_patterns = [
        "tempo" + "rary",
        "place" + "holder",
        "integration" + " point",
        "st" + "ub",
        "not " + "implemented",
    ]
    scanned_roots = [root / "ecu", root / "tests", root / "docs"]
    ignored = {
        "docs/superpowers/plans/2026-06-23-ecu-board-functional-test-implementation.md",
        "docs/superpowers/plans/2026-06-27-ecu-main-control-framework.md",
        "docs/superpowers/plans/2026-06-28-ecu-device-control-middleware.md",
        "docs/superpowers/plans/2026-06-28-modbus-virtual-adc-canopen-command-debug.md",
        "docs/superpowers/specs/2026-06-28-ecu-device-control-middleware-design.md",
        "docs/superpowers/specs/2026-06-28-modbus-virtual-adc-canopen-command-debug-design.md",
    }
    for folder in scanned_roots:
        for path in folder.rglob("*"):
            if not path.is_file():
                continue
            rel = path.relative_to(root).as_posix()
            if rel.startswith("ecu/sdk_env_v1.11.0/") or rel in ignored:
                continue
            if rel.startswith("ecu/apps/agri_chassis_control_cpu0/build/"):
                continue
            if path.suffix.lower() not in {".c", ".h", ".py", ".md", ".cmake", ".txt"}:
                continue
            text = path.read_text(encoding="utf-8", errors="ignore").lower()
            for pattern in forbidden_patterns:
                assert pattern not in text, f"{rel}: remove transitional wording '{pattern}'"


def test_cpu0_startup_and_fatal_hooks_are_visible_on_debug_console(root: pathlib.Path) -> None:
    main_c = read(root, "ecu/apps/agri_chassis_control_cpu0/src/main_cpu0.c")

    assert "ECU CPU0 boot" in main_c
    assert "create_task_or_report" in main_c
    assert "FATAL malloc failed" in main_c
    assert "FATAL stack overflow" in main_c
    assert "\\r\\n" in main_c
