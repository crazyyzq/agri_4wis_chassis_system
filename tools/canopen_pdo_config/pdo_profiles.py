"""Manual-derived PDO mapping profiles for BC/BC2 CANopen drives."""

from __future__ import annotations

from dataclasses import dataclass


BUS_NODES = {
    "can1": [1, 2, 3, 4, 5, 6, 7, 8],
    "can2": [9, 10, 11, 12, 13],
}

NODE_ROLES = {
    1: "drive_FR",
    2: "drive_FL",
    3: "drive_RL",
    4: "drive_RR",
    5: "steer_FR",
    6: "steer_FL",
    7: "steer_RL",
    8: "steer_RR",
    9: "lift_FR",
    10: "lift_RR",
    11: "lift_FL",
    12: "lift_RL",
    13: "hydraulic_pump",
}

BUS_LABELS = {
    "can1": "CAN1 / ECU-CAN2",
    "can2": "CAN2 / ECU-CAN3",
}


@dataclass(frozen=True)
class PdoOperation:
    kind: str
    index: int
    subindex: int
    size: int
    value: int
    description: str
    verify: bool = True


PDO_TRANSMISSION_SYNC1 = 1
PDO_MAP_CONTROLWORD_16 = 0x60400010
PDO_MAP_MODE_OF_OPERATION_8 = 0x60600008
PDO_MAP_TARGET_VELOCITY_32 = 0x60FF0020
PDO_MAP_TARGET_POSITION_32 = 0x607A0020
PDO_MAP_COMMAND_CURRENT_16 = 0x23400010
PDO_MAP_INTERPOLATED_POSITION_32 = 0x60C10120
PDO_MAP_ACTUAL_POSITION_32 = 0x60640020
PDO_MAP_ACTUAL_VELOCITY_32 = 0x606C0020
PDO_MAP_LATCHED_FAULT_32 = 0x21830020
PDO_MAP_STATUSWORD_16 = 0x60410010
PDO_MAP_ACTUAL_CURRENT_16 = 0x221C0010


BACKUP_OBJECTS: tuple[tuple[int, int, int], ...] = (
    (0x1400, 0x01, 4), (0x1400, 0x02, 1),
    (0x1600, 0x00, 1), (0x1600, 0x01, 4), (0x1600, 0x02, 4), (0x1600, 0x03, 4),
    (0x1401, 0x01, 4), (0x1401, 0x02, 1),
    (0x1601, 0x00, 1), (0x1601, 0x01, 4), (0x1601, 0x02, 4), (0x1601, 0x03, 4),
    (0x1402, 0x01, 4), (0x1402, 0x02, 1),
    (0x1602, 0x00, 1), (0x1602, 0x01, 4),
    (0x1403, 0x01, 4), (0x1403, 0x02, 1),
    (0x1603, 0x00, 1), (0x1603, 0x01, 4), (0x1603, 0x02, 4), (0x1603, 0x03, 4),
    (0x1800, 0x01, 4), (0x1800, 0x02, 1),
    (0x1A00, 0x00, 1), (0x1A00, 0x01, 4), (0x1A00, 0x02, 4),
    (0x1801, 0x01, 4), (0x1801, 0x02, 1),
    (0x1A01, 0x00, 1), (0x1A01, 0x01, 4), (0x1A01, 0x02, 4), (0x1A01, 0x03, 4),
    (0x1802, 0x01, 4), (0x1A02, 0x00, 1),
    (0x1803, 0x01, 4), (0x1A03, 0x00, 1),
)


def build_node_configuration(node_id: int) -> list[PdoOperation]:
    rpdo0 = 0x200 + node_id
    rpdo1 = 0x300 + node_id
    rpdo2 = 0x400 + node_id
    rpdo3 = 0x500 + node_id
    tpdo0 = 0x180 + node_id
    tpdo1 = 0x280 + node_id
    tpdo2_reserved = 0x380 + node_id
    tpdo3_reserved = 0x480 + node_id
    return [
        PdoOperation("download", 0x1400, 0x01, 4, 0x80000000 | rpdo0, "disable RPDO0 COB-ID"),
        PdoOperation("download", 0x1400, 0x02, 1, PDO_TRANSMISSION_SYNC1, "RPDO0 synchronous every SYNC"),
        PdoOperation("download", 0x1600, 0x00, 1, 0, "clear RPDO0 map"),
        PdoOperation("download", 0x1600, 0x01, 4, PDO_MAP_CONTROLWORD_16, "RPDO0 controlword"),
        PdoOperation("download", 0x1600, 0x02, 4, PDO_MAP_MODE_OF_OPERATION_8, "RPDO0 mode"),
        PdoOperation("download", 0x1600, 0x03, 4, PDO_MAP_TARGET_VELOCITY_32, "RPDO0 target velocity"),
        PdoOperation("download", 0x1600, 0x00, 1, 3, "enable RPDO0 map entries"),
        PdoOperation("download", 0x1400, 0x01, 4, rpdo0, "enable RPDO0 COB-ID"),
        PdoOperation("download", 0x1401, 0x01, 4, 0x80000000 | rpdo1, "disable RPDO1 COB-ID"),
        PdoOperation("download", 0x1401, 0x02, 1, PDO_TRANSMISSION_SYNC1, "RPDO1 synchronous every SYNC"),
        PdoOperation("download", 0x1601, 0x00, 1, 0, "clear RPDO1 map"),
        PdoOperation("download", 0x1601, 0x01, 4, PDO_MAP_CONTROLWORD_16, "RPDO1 controlword"),
        PdoOperation("download", 0x1601, 0x02, 4, PDO_MAP_MODE_OF_OPERATION_8, "RPDO1 mode"),
        PdoOperation("download", 0x1601, 0x03, 4, PDO_MAP_TARGET_POSITION_32, "RPDO1 target position"),
        PdoOperation("download", 0x1601, 0x00, 1, 3, "enable RPDO1 map entries"),
        PdoOperation("download", 0x1401, 0x01, 4, rpdo1, "enable RPDO1 COB-ID"),
        PdoOperation("download", 0x1402, 0x01, 4, 0x80000000 | rpdo2, "disable RPDO2 COB-ID"),
        PdoOperation("download", 0x1402, 0x02, 1, PDO_TRANSMISSION_SYNC1, "RPDO2 interpolation synchronous every SYNC"),
        PdoOperation("download", 0x1602, 0x00, 1, 0, "clear RPDO2 map"),
        PdoOperation("download", 0x1602, 0x01, 4, PDO_MAP_INTERPOLATED_POSITION_32, "RPDO2 interpolated position point 60C1:01"),
        PdoOperation("download", 0x1602, 0x00, 1, 1, "enable RPDO2 map entries"),
        PdoOperation("download", 0x1402, 0x01, 4, rpdo2, "enable RPDO2 COB-ID"),
        PdoOperation("download", 0x1403, 0x01, 4, 0x80000000 | rpdo3, "disable RPDO3 COB-ID"),
        PdoOperation("download", 0x1403, 0x02, 1, PDO_TRANSMISSION_SYNC1, "RPDO3 torque/current synchronous every SYNC"),
        PdoOperation("download", 0x1603, 0x00, 1, 0, "clear RPDO3 map"),
        PdoOperation("download", 0x1603, 0x01, 4, PDO_MAP_CONTROLWORD_16, "RPDO3 controlword"),
        PdoOperation("download", 0x1603, 0x02, 4, PDO_MAP_MODE_OF_OPERATION_8, "RPDO3 mode"),
        PdoOperation("download", 0x1603, 0x03, 4, PDO_MAP_COMMAND_CURRENT_16, "RPDO3 command current"),
        PdoOperation("download", 0x1603, 0x00, 1, 3, "enable RPDO3 map entries"),
        PdoOperation("download", 0x1403, 0x01, 4, rpdo3, "enable RPDO3 COB-ID"),
        PdoOperation("download", 0x1800, 0x01, 4, 0x80000000 | tpdo0, "disable TPDO0 COB-ID"),
        PdoOperation("download", 0x1A00, 0x00, 1, 0, "clear TPDO0 map"),
        PdoOperation("download", 0x1A00, 0x01, 4, PDO_MAP_ACTUAL_POSITION_32, "TPDO0 actual position"),
        PdoOperation("download", 0x1A00, 0x02, 4, PDO_MAP_ACTUAL_VELOCITY_32, "TPDO0 actual velocity"),
        PdoOperation("download", 0x1800, 0x02, 1, PDO_TRANSMISSION_SYNC1, "TPDO0 synchronous"),
        PdoOperation("download", 0x1A00, 0x00, 1, 2, "enable TPDO0 map entries"),
        PdoOperation("download", 0x1800, 0x01, 4, tpdo0, "enable TPDO0 COB-ID"),
        PdoOperation("download", 0x1801, 0x01, 4, 0x80000000 | tpdo1, "disable TPDO1 COB-ID"),
        PdoOperation("download", 0x1A01, 0x00, 1, 0, "clear TPDO1 map"),
        PdoOperation("download", 0x1A01, 0x01, 4, PDO_MAP_LATCHED_FAULT_32, "TPDO1 latching fault status"),
        PdoOperation("download", 0x1A01, 0x02, 4, PDO_MAP_STATUSWORD_16, "TPDO1 statusword"),
        PdoOperation("download", 0x1A01, 0x03, 4, PDO_MAP_ACTUAL_CURRENT_16, "TPDO1 actual motor current"),
        PdoOperation("download", 0x1801, 0x02, 1, 4, "TPDO1 synchronous every 4th SYNC"),
        PdoOperation("download", 0x1A01, 0x00, 1, 3, "enable TPDO1 map entries"),
        PdoOperation("download", 0x1801, 0x01, 4, tpdo1, "enable TPDO1 COB-ID"),
        PdoOperation(
            "download",
            0x1802,
            0x01,
            4,
            0x80000000 | tpdo2_reserved,
            "transiently disable unmanaged TPDO2 before clearing its map",
            verify=False,
        ),
        PdoOperation("download", 0x1A02, 0x00, 1, 0, "clear unmanaged TPDO2 map"),
        PdoOperation(
            "download",
            0x1803,
            0x01,
            4,
            0x80000000 | tpdo3_reserved,
            "transiently disable unmanaged TPDO3 before clearing its map",
            verify=False,
        ),
        PdoOperation("download", 0x1A03, 0x00, 1, 0, "clear unmanaged TPDO3 map"),
    ]


def expected_mapping_values(node_id: int) -> dict[str, int]:
    values: dict[str, int] = {}
    for op in build_node_configuration(node_id):
        if op.verify:
            values[f"0x{op.index:04X}:{op.subindex}"] = op.value
    return values


def select_nodes(bus_names: list[str], node_ids: list[int]) -> dict[str, list[int]]:
    selected: dict[str, list[int]] = {}
    requested = set(node_ids)
    for bus in bus_names:
        allowed = BUS_NODES[bus]
        selected[bus] = [node for node in allowed if node in requested]
    return selected
