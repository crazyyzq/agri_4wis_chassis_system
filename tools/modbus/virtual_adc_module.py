"""Virtual 8-channel RS485 analog acquisition module.

This bench fixture emulates the INNO-8AI-485 style device used by the ECU:
Modbus RTU, slave 1 by default, function 04 Read Input Registers, registers 0..7.
"""

from __future__ import annotations

import argparse
import pathlib
import sys
import time

import serial
import serial.tools.list_ports

if __package__ in {None, ""}:
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))

from tools.modbus.rtu_codec import (  # noqa: E402
    build_exception,
    build_read_input_registers_response,
    verify_crc,
)


MODBUS_FUNCTION_READ_INPUT_REGISTERS = 0x04
MODBUS_EXCEPTION_ILLEGAL_FUNCTION = 0x01
MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS = 0x02
MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE = 0x03
DEFAULT_CHANNELS_MV = "0,714,1428,2142,2856,3570,4284,5000"


def list_ports() -> None:
    """Print serial ports visible to Python."""
    for port in serial.tools.list_ports.comports():
        print(f"{port.device}\t{port.description}\t{port.hwid}", flush=True)


def parse_channels_mv(text: str) -> list[int]:
    """Parse eight comma-separated channel voltages expressed in mV."""
    values = [int(item.strip(), 0) for item in text.split(",") if item.strip()]
    if len(values) != 8:
        raise argparse.ArgumentTypeError("--channels-mv requires exactly 8 values")
    if any(value < 0 for value in values):
        raise argparse.ArgumentTypeError("channel mV values must be non-negative")
    return values


def millivolts_to_raw(values_mv: list[int], full_scale_mv: int) -> list[int]:
    """Convert millivolts to unsigned 16-bit raw register values."""
    if full_scale_mv <= 0:
        raise ValueError("full scale must be positive")
    raw_values: list[int] = []
    for value_mv in values_mv:
        limited_mv = min(value_mv, full_scale_mv)
        raw_values.append(round(limited_mv * 0xFFFF / full_scale_mv))
    return raw_values


def format_channels(values_mv: list[int], raw_values: list[int]) -> str:
    """Return a compact one-line channel dump for console tracing."""
    return " ".join(f"AI{index + 1}={mv}mV/{raw}" for index, (mv, raw) in enumerate(zip(values_mv, raw_values)))


def handle_request(frame: bytes, slave: int, raw_values: list[int]) -> bytes | None:
    """Decode one 8-byte RTU request and return a response frame."""
    if len(frame) != 8:
        return None
    if not verify_crc(frame):
        print(f"CRC error RX={frame.hex(' ')}", flush=True)
        return None
    if frame[0] != slave:
        return None

    function = frame[1]
    if function != MODBUS_FUNCTION_READ_INPUT_REGISTERS:
        return build_exception(slave, function, MODBUS_EXCEPTION_ILLEGAL_FUNCTION)

    start = (frame[2] << 8) | frame[3]
    count = (frame[4] << 8) | frame[5]
    if count == 0 or count > 8:
        return build_exception(slave, function, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE)
    if start >= 8 or start + count > 8:
        return build_exception(slave, function, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS)

    return build_read_input_registers_response(slave, raw_values[start : start + count])


def pop_next_request_frame(rx: bytearray, slave: int) -> bytes | None:
    """Return the next valid 8-byte request, sliding over stale/noise bytes.

    A PC-side virtual slave can be started while the ECU is already polling.  In
    that case the first bytes read from the USB-RS485 adapter can be the CRC
    tail of the previous request.  A real Modbus RTU receiver re-synchronizes on
    inter-frame timing; this bench tool does the equivalent by sliding one byte
    at a time until slave address, function code and CRC all match.
    """
    while len(rx) >= 8:
        frame = bytes(rx[:8])
        if (
            frame[0] == slave
            and frame[1] == MODBUS_FUNCTION_READ_INPUT_REGISTERS
            and verify_crc(frame)
        ):
            del rx[:8]
            return frame
        del rx[0]
    return None


def serve(args: argparse.Namespace) -> int:
    """Run the virtual ADC until interrupted."""
    channels_mv = parse_channels_mv(args.channels_mv)
    raw_values = millivolts_to_raw(channels_mv, args.full_scale_mv)

    print(
        f"Virtual ADC module on {args.port}, baud={args.baudrate}, slave={args.slave}, "
        f"Read Input Registers registers 0..7",
        flush=True,
    )
    print(format_channels(channels_mv, raw_values), flush=True)

    with serial.Serial(
        port=args.port,
        baudrate=args.baudrate,
        bytesize=8,
        parity="N",
        stopbits=1,
        timeout=0.05,
    ) as ser:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        rx = bytearray()
        while True:
            chunk = ser.read(max(1, 256 - len(rx)))
            if chunk:
                rx.extend(chunk)
            else:
                time.sleep(0.005)

            while True:
                frame = pop_next_request_frame(rx, args.slave)
                if frame is None:
                    break
                print(f"RX {frame.hex(' ')}", flush=True)
                response = handle_request(frame, args.slave, raw_values)
                if response is None:
                    continue
                ser.write(response)
                ser.flush()
                print(f"TX {response.hex(' ')}  {format_channels(channels_mv, raw_values)}",
                      flush=True)


def build_parser() -> argparse.ArgumentParser:
    """Create CLI parser for the bench fixture."""
    parser = argparse.ArgumentParser(description="Virtual COM10 Modbus RTU ADC module.")
    parser.add_argument("--list", action="store_true", help="list serial ports and exit")
    parser.add_argument("--port", default="COM10", help="serial port connected to ECU RS485_1")
    parser.add_argument("--baudrate", type=int, default=9600)
    parser.add_argument("--slave", type=lambda value: int(value, 0), default=1)
    parser.add_argument("--channels-mv", default=DEFAULT_CHANNELS_MV)
    parser.add_argument("--full-scale-mv", type=int, default=5000)
    return parser


def main() -> int:
    """CLI entry point."""
    parser = build_parser()
    args = parser.parse_args()
    if args.list:
        list_ports()
        return 0
    return serve(args)


if __name__ == "__main__":
    raise SystemExit(main())
