"""Minimal CANopen SDO client frame helpers for PDO configuration."""

from __future__ import annotations

from dataclasses import dataclass

from can_adapter import CanFrame


class SdoError(RuntimeError):
    pass


class SdoAbort(SdoError):
    def __init__(self, node_id: int, index: int, subindex: int, abort_code: int) -> None:
        super().__init__(
            f"SDO abort node={node_id} object=0x{index:04X}:{subindex} "
            f"abort=0x{abort_code:08X}"
        )
        self.node_id = node_id
        self.index = index
        self.subindex = subindex
        self.abort_code = abort_code


@dataclass(frozen=True)
class SdoUploadValue:
    node_id: int
    index: int
    subindex: int
    size: int
    value: int
    response: CanFrame


def _write_le(value: int, size: int) -> bytes:
    return int(value).to_bytes(size, byteorder="little", signed=False)


def _read_le(data: bytes) -> int:
    return int.from_bytes(data, byteorder="little", signed=False)


def encode_sdo_download(node_id: int, index: int, subindex: int, size: int, value: int) -> CanFrame:
    if size not in {1, 2, 4}:
        raise ValueError(f"unsupported SDO download size {size}")
    command = {1: 0x2F, 2: 0x2B, 4: 0x23}[size]
    payload = bytearray(8)
    payload[0] = command
    payload[1] = index & 0xFF
    payload[2] = (index >> 8) & 0xFF
    payload[3] = subindex & 0xFF
    payload[4:4 + size] = _write_le(value, size)
    return CanFrame(0x600 + node_id, bytes(payload))


def encode_sdo_upload(node_id: int, index: int, subindex: int) -> CanFrame:
    return CanFrame(
        0x600 + node_id,
        bytes([0x40, index & 0xFF, (index >> 8) & 0xFF, subindex & 0xFF, 0, 0, 0, 0]),
    )


def encode_nmt_preop(node_id: int) -> CanFrame:
    return CanFrame(0x000, bytes([0x80, node_id & 0x7F]))


def decode_sdo_download_response(node_id: int, index: int, subindex: int, frame: CanFrame) -> None:
    expected_id = 0x580 + node_id
    if frame.can_id != expected_id or frame.dlc != 8:
        raise SdoError(f"unexpected SDO response id=0x{frame.can_id:03X} dlc={frame.dlc}")
    if frame.data[0] == 0x80:
        abort_code = _read_le(frame.data[4:8])
        raise SdoAbort(node_id, index, subindex, abort_code)
    expected = bytes([0x60, index & 0xFF, (index >> 8) & 0xFF, subindex & 0xFF, 0, 0, 0, 0])
    if frame.data != expected:
        raise SdoError(f"unexpected SDO download ack data={frame.data.hex(' ').upper()}")


def decode_sdo_upload_response(node_id: int, index: int, subindex: int, frame: CanFrame) -> SdoUploadValue:
    expected_id = 0x580 + node_id
    if frame.can_id != expected_id or frame.dlc != 8:
        raise SdoError(f"unexpected SDO upload response id=0x{frame.can_id:03X} dlc={frame.dlc}")
    if frame.data[0] == 0x80:
        abort_code = _read_le(frame.data[4:8])
        raise SdoAbort(node_id, index, subindex, abort_code)
    size_by_command = {
        0x4F: 1,
        0x4B: 2,
        0x43: 4,
    }
    size = size_by_command.get(frame.data[0])
    if size is None:
        raise SdoError(f"unsupported SDO upload command 0x{frame.data[0]:02X}")
    if frame.data[1] != (index & 0xFF) or frame.data[2] != ((index >> 8) & 0xFF) or frame.data[3] != subindex:
        raise SdoError(f"unexpected SDO upload object data={frame.data.hex(' ').upper()}")
    return SdoUploadValue(node_id, index, subindex, size, _read_le(frame.data[4:4 + size]), frame)


def encode_sdo_upload_response(node_id: int, index: int, subindex: int, size: int, value: int) -> CanFrame:
    command = {1: 0x4F, 2: 0x4B, 4: 0x43}[size]
    payload = bytearray(8)
    payload[0] = command
    payload[1] = index & 0xFF
    payload[2] = (index >> 8) & 0xFF
    payload[3] = subindex & 0xFF
    payload[4:4 + size] = _write_le(value, size)
    return CanFrame(0x580 + node_id, bytes(payload))


def encode_sdo_download_ack(node_id: int, index: int, subindex: int) -> CanFrame:
    return CanFrame(
        0x580 + node_id,
        bytes([0x60, index & 0xFF, (index >> 8) & 0xFF, subindex & 0xFF, 0, 0, 0, 0]),
    )


def encode_sdo_abort(node_id: int, index: int, subindex: int, abort_code: int) -> CanFrame:
    payload = bytearray([0x80, index & 0xFF, (index >> 8) & 0xFF, subindex & 0xFF, 0, 0, 0, 0])
    payload[4:8] = _write_le(abort_code, 4)
    return CanFrame(0x580 + node_id, bytes(payload))

