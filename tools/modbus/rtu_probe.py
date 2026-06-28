"""Safe Modbus RTU read probe for COM10 USB-RS485 bring-up."""

from __future__ import annotations

import argparse
import time

import serial
import serial.tools.list_ports


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def add_crc(payload: bytes) -> bytes:
    crc = crc16_modbus(payload)
    return payload + bytes((crc & 0xFF, (crc >> 8) & 0xFF))


def list_ports() -> None:
    for port in serial.tools.list_ports.comports():
        print(f"{port.device}\t{port.description}\t{port.hwid}")


def read_holding_registers(port: str, slave: int, address: int, count: int, baudrate: int) -> bytes:
    request = add_crc(bytes((slave, 0x03, (address >> 8) & 0xFF, address & 0xFF, (count >> 8) & 0xFF, count & 0xFF)))
    expected_len = 5 + count * 2
    with serial.Serial(port=port, baudrate=baudrate, bytesize=8, parity="N", stopbits=1, timeout=0.2) as ser:
        ser.reset_input_buffer()
        ser.write(request)
        ser.flush()
        time.sleep(0.05)
        response = ser.read(expected_len)
    if len(response) < 5:
        raise RuntimeError(f"short response: {response.hex(' ')}")
    rx_crc = response[-2] | (response[-1] << 8)
    calc_crc = crc16_modbus(response[:-2])
    if rx_crc != calc_crc:
        raise RuntimeError(f"bad crc response={response.hex(' ')}")
    return response


def main() -> int:
    parser = argparse.ArgumentParser(description="Safe Modbus RTU read probe for COM10.")
    parser.add_argument("--list", action="store_true", help="list serial ports and exit")
    parser.add_argument("--port", default="COM10")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--slave", type=int, default=2)
    parser.add_argument("--address", type=lambda value: int(value, 0), default=0)
    parser.add_argument("--count", type=int, default=2)
    args = parser.parse_args()

    if args.list:
        list_ports()
        return 0

    response = read_holding_registers(args.port, args.slave, args.address, args.count, args.baudrate)
    print(response.hex(" "))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
