"""Small Modbus RTU helpers used by ECU bench tools.

The ECU firmware uses HPM SDK agile_modbus.  This Python module is only for PC
side bench fixtures where we need a deterministic serial peer on COM10.
"""

from __future__ import annotations


def crc16_modbus(data: bytes) -> int:
    """Return the standard Modbus RTU CRC16 value for *data*."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def append_crc(payload: bytes) -> bytes:
    """Append Modbus RTU CRC bytes, low byte first."""
    crc = crc16_modbus(payload)
    return payload + bytes((crc & 0xFF, (crc >> 8) & 0xFF))


def verify_crc(frame: bytes) -> bool:
    """Return True when *frame* has a valid RTU CRC trailer."""
    if len(frame) < 4:
        return False
    received = frame[-2] | (frame[-1] << 8)
    return received == crc16_modbus(frame[:-2])


def build_exception(slave: int, function: int, code: int) -> bytes:
    """Build a Modbus exception response frame."""
    payload = bytes((slave & 0xFF, (function | 0x80) & 0xFF, code & 0xFF))
    return append_crc(payload)


def build_read_input_registers_response(slave: int, values: list[int]) -> bytes:
    """Build a function-04 response containing 16-bit input register values."""
    if not 0 <= len(values) <= 125:
        raise ValueError("invalid register count")

    payload = bytearray((slave & 0xFF, 0x04, len(values) * 2))
    for value in values:
        if not 0 <= value <= 0xFFFF:
            raise ValueError(f"register value out of range: {value}")
        payload.extend(((value >> 8) & 0xFF, value & 0xFF))
    return append_crc(bytes(payload))
