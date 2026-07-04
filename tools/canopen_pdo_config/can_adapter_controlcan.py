"""ControlCAN adapter for the vendor USB-CAN analyzer."""

from __future__ import annotations

import ctypes
import sys
import time
from pathlib import Path

CURRENT_DIR = Path(__file__).resolve().parent
TOOLS_CAN_DIR = CURRENT_DIR.parent / "can"
if str(TOOLS_CAN_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_CAN_DIR))

from can_adapter import CanFrame
from controlcan import ControlCAN, VCI_CAN_OBJ


CHANNEL_INDEX = {
    "can1": 0,
    "can2": 1,
}


class ControlCanAdapter:
    def __init__(self, channel_can1: int = 0, channel_can2: int = 1) -> None:
        self._device = ControlCAN()
        self._channels = {
            "can1": channel_can1,
            "can2": channel_can2,
        }
        self._rx_cache: dict[str, list[CanFrame]] = {
            "can1": [],
            "can2": [],
        }

    def open(self, buses: list[str], bitrate: int) -> None:
        self._device.open()
        for bus in buses:
            self._device.init_can(self._channels[bus], bitrate, listen_only=False)

    def close(self) -> None:
        self._device.close()

    def send(self, bus: str, frame: CanFrame) -> None:
        obj = VCI_CAN_OBJ()
        obj.ID = frame.can_id
        obj.SendType = 0
        obj.RemoteFlag = 1 if frame.is_remote else 0
        obj.ExternFlag = 1 if frame.is_extended else 0
        obj.DataLen = frame.dlc
        for i, value in enumerate(frame.data):
            obj.Data[i] = value
        sent = self._device.transmit_frames(self._channels[bus], [obj])
        if sent != 1:
            raise RuntimeError(f"ControlCAN transmit failed bus={bus} id=0x{frame.can_id:03X}")

    def receive(self, bus: str, expected_can_id: int, timeout_ms: int) -> CanFrame:
        channel = self._channels[bus]
        cached = self._rx_cache[bus]
        for index, frame in enumerate(cached):
            if frame.can_id == expected_can_id:
                return cached.pop(index)

        deadline = time.monotonic() + (max(1, timeout_ms) / 1000.0)
        while time.monotonic() < deadline:
            frames = self._device.receive(channel, limit=100, wait_ms=20)
            for raw in frames:
                data = bytes(int(raw.Data[i]) for i in range(int(raw.DataLen)))
                frame = CanFrame(
                    int(raw.ID),
                    data,
                    is_extended=bool(raw.ExternFlag),
                    is_remote=bool(raw.RemoteFlag),
                )
                if frame.can_id == expected_can_id:
                    return frame
                cached.append(frame)
            time.sleep(0.002)
        raise TimeoutError(f"timeout waiting for 0x{expected_can_id:03X} on {bus}")
