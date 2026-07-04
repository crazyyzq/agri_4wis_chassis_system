"""Mock CAN adapter for PDO configurator tests and dry-run checks."""

from __future__ import annotations

from can_adapter import CanFrame
from canopen_sdo import encode_sdo_abort, encode_sdo_download_ack, encode_sdo_upload_response


class MockCanAdapter:
    def __init__(self, abort_on: set[tuple[int, int, int]] | None = None) -> None:
        self.abort_on = abort_on or set()
        self.open_count = 0
        self.close_count = 0
        self.sent_frames: list[CanFrame] = []
        self._objects: dict[tuple[int, int, int], tuple[int, int]] = {}
        self._pending_response: dict[str, CanFrame] = {}

    def open(self, buses: list[str], bitrate: int) -> None:
        self.open_count += 1
        self.buses = list(buses)
        self.bitrate = bitrate

    def close(self) -> None:
        self.close_count += 1

    def send(self, bus: str, frame: CanFrame) -> None:
        self.sent_frames.append(frame)
        if frame.can_id == 0x000:
            return
        if not (0x600 <= frame.can_id <= 0x67F) or frame.dlc != 8:
            return
        node_id = frame.can_id - 0x600
        command = frame.data[0]
        index = frame.data[1] | (frame.data[2] << 8)
        subindex = frame.data[3]
        if (node_id, index, subindex) in self.abort_on:
            self._pending_response[bus] = encode_sdo_abort(node_id, index, subindex, 0x06090030)
            return
        if command in {0x23, 0x2B, 0x2F}:
            size = {0x23: 4, 0x2B: 2, 0x2F: 1}[command]
            value = int.from_bytes(frame.data[4:4 + size], "little")
            self._objects[(node_id, index, subindex)] = (size, value)
            self._pending_response[bus] = encode_sdo_download_ack(node_id, index, subindex)
        elif command == 0x40:
            size, value = self._objects.get((node_id, index, subindex), (4, 0))
            self._pending_response[bus] = encode_sdo_upload_response(node_id, index, subindex, size, value)

    def receive(self, bus: str, expected_can_id: int, timeout_ms: int) -> CanFrame:
        frame = self._pending_response.pop(bus)
        if frame.can_id != expected_can_id:
            raise TimeoutError(f"mock expected 0x{expected_can_id:03X}, got 0x{frame.can_id:03X}")
        return frame

