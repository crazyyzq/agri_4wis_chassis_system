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


BACKUP_OBJECTS: tuple[tuple[int, int, int], ...] = (
    (0x1400, 0x01, 4), (0x1400, 0x02, 1),
    (0x1600, 0x00, 1), (0x1600, 0x01, 4), (0x1600, 0x02, 4), (0x1600, 0x03, 4),
    (0x1401, 0x01, 4), (0x1401, 0x02, 1),
    (0x1601, 0x00, 1), (0x1601, 0x01, 4), (0x1601, 0x02, 4), (0x1601, 0x03, 4),
    (0x1800, 0x01, 4), (0x1800, 0x02, 1),
    (0x1A00, 0x00, 1), (0x1A00, 0x01, 4), (0x1A00, 0x02, 4),
    (0x1801, 0x01, 4), (0x1801, 0x02, 1),
    (0x1A01, 0x00, 1), (0x1A01, 0x01, 4), (0x1A01, 0x02, 4), (0x1A01, 0x03, 4),
)


def build_node_configuration(node_id: int) -> list[PdoOperation]:
    rpdo0 = 0x200 + node_id
    rpdo1 = 0x300 + node_id
    tpdo0 = 0x180 + node_id
    tpdo1 = 0x280 + node_id
    return [
        PdoOperation("download", 0x1400, 0x01, 4, 0x80000000 | rpdo0, "disable RPDO0 COB-ID"),
        PdoOperation("download", 0x1400, 0x02, 1, 0xFF, "RPDO0 asynchronous"),
        PdoOperation("download", 0x1600, 0x00, 1, 0, "clear RPDO0 map"),
        PdoOperation("download", 0x1600, 0x01, 4, 0x60400010, "RPDO0 controlword"),
        PdoOperation("download", 0x1600, 0x02, 4, 0x60600008, "RPDO0 mode"),
        PdoOperation("download", 0x1600, 0x03, 4, 0x60FF0020, "RPDO0 target velocity"),
        PdoOperation("download", 0x1600, 0x00, 1, 3, "enable RPDO0 map entries"),
        PdoOperation("download", 0x1400, 0x01, 4, rpdo0, "enable RPDO0 COB-ID"),
        PdoOperation("download", 0x1401, 0x01, 4, 0x80000000 | rpdo1, "disable RPDO1 COB-ID"),
        PdoOperation("download", 0x1401, 0x02, 1, 0xFF, "RPDO1 asynchronous"),
        PdoOperation("download", 0x1601, 0x00, 1, 0, "clear RPDO1 map"),
        PdoOperation("download", 0x1601, 0x01, 4, 0x60400010, "RPDO1 controlword"),
        PdoOperation("download", 0x1601, 0x02, 4, 0x60600008, "RPDO1 mode"),
        PdoOperation("download", 0x1601, 0x03, 4, 0x607A0020, "RPDO1 target position"),
        PdoOperation("download", 0x1601, 0x00, 1, 3, "enable RPDO1 map entries"),
        PdoOperation("download", 0x1401, 0x01, 4, rpdo1, "enable RPDO1 COB-ID"),
        PdoOperation("download", 0x1A00, 0x00, 1, 0, "clear TPDO0 map"),
        PdoOperation("download", 0x1A00, 0x01, 4, 0x60640020, "TPDO0 actual position"),
        PdoOperation("download", 0x1A00, 0x02, 4, 0x606C0020, "TPDO0 actual velocity"),
        PdoOperation("download", 0x1800, 0x01, 4, tpdo0, "TPDO0 COB-ID"),
        PdoOperation("download", 0x1800, 0x02, 1, 1, "TPDO0 synchronous"),
        PdoOperation("download", 0x1A00, 0x00, 1, 2, "enable TPDO0 map entries"),
        PdoOperation("download", 0x1A01, 0x00, 1, 0, "clear TPDO1 map"),
        PdoOperation("download", 0x1A01, 0x01, 4, 0x21830020, "TPDO1 digital inputs"),
        PdoOperation("download", 0x1A01, 0x02, 4, 0x60410010, "TPDO1 statusword"),
        PdoOperation("download", 0x1A01, 0x03, 4, 0x221C0010, "TPDO1 warning/status"),
        PdoOperation("download", 0x1801, 0x01, 4, tpdo1, "TPDO1 COB-ID"),
        PdoOperation("download", 0x1801, 0x02, 1, 1, "TPDO1 synchronous"),
        PdoOperation("download", 0x1A01, 0x00, 1, 3, "enable TPDO1 map entries"),
    ]


def expected_mapping_values(node_id: int) -> dict[str, int]:
    values: dict[str, int] = {}
    for op in build_node_configuration(node_id):
        values[f"0x{op.index:04X}:{op.subindex}"] = op.value
    return values


def select_nodes(bus_names: list[str], node_ids: list[int]) -> dict[str, list[int]]:
    selected: dict[str, list[int]] = {}
    requested = set(node_ids)
    for bus in bus_names:
        allowed = BUS_NODES[bus]
        selected[bus] = [node for node in allowed if node in requested]
    return selected

