"""Minimal ctypes wrapper for the vendor ControlCAN.dll.

The wrapper is intentionally small and direct. It exposes enough API for
passive CAN bring-up with the USB CAN analyzer while keeping transmit calls out
of the default monitor script.
"""

from __future__ import annotations

import ctypes
from pathlib import Path
from typing import Iterable

VCI_USBCAN1 = 3
VCI_USBCAN2 = 4
VCI_USBCAN2A = 4
VCI_USBCAN_E_U = 20
VCI_USBCAN_2E_U = 21

STATUS_OK = 1


class VCI_CAN_OBJ(ctypes.Structure):
    _fields_ = [
        ("ID", ctypes.c_uint),
        ("TimeStamp", ctypes.c_uint),
        ("TimeFlag", ctypes.c_ubyte),
        ("SendType", ctypes.c_ubyte),
        ("RemoteFlag", ctypes.c_ubyte),
        ("ExternFlag", ctypes.c_ubyte),
        ("DataLen", ctypes.c_ubyte),
        ("Data", ctypes.c_ubyte * 8),
        ("Reserved", ctypes.c_ubyte * 3),
    ]


class VCI_INIT_CONFIG(ctypes.Structure):
    _fields_ = [
        ("AccCode", ctypes.c_uint32),
        ("AccMask", ctypes.c_uint32),
        ("Reserved", ctypes.c_uint32),
        ("Filter", ctypes.c_ubyte),
        ("Timing0", ctypes.c_ubyte),
        ("Timing1", ctypes.c_ubyte),
        ("Mode", ctypes.c_ubyte),
    ]


BITRATE_TIMING = {
    1000000: (0x00, 0x14),
    800000: (0x00, 0x16),
    500000: (0x00, 0x1C),
    250000: (0x01, 0x1C),
    125000: (0x03, 0x1C),
}


def default_dll_path() -> Path:
    return (
        Path(__file__).resolve().parents[2]
        / "doc"
        / "CAN分析仪"
        / "二次开发库文件"
        / "x64(64bit)"
        / "ControlCAN.dll"
    )


class ControlCAN:
    def __init__(self, dll_path: Path | None = None, device_type: int = VCI_USBCAN2, device_index: int = 0) -> None:
        self.dll_path = Path(dll_path) if dll_path is not None else default_dll_path()
        self.device_type = device_type
        self.device_index = device_index
        self._dll = ctypes.WinDLL(str(self.dll_path))
        self._bind_functions()

    def _bind_functions(self) -> None:
        self._dll.VCI_OpenDevice.argtypes = [ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32]
        self._dll.VCI_OpenDevice.restype = ctypes.c_uint32
        self._dll.VCI_CloseDevice.argtypes = [ctypes.c_uint32, ctypes.c_uint32]
        self._dll.VCI_CloseDevice.restype = ctypes.c_uint32
        self._dll.VCI_InitCAN.argtypes = [
            ctypes.c_uint32,
            ctypes.c_uint32,
            ctypes.c_uint32,
            ctypes.POINTER(VCI_INIT_CONFIG),
        ]
        self._dll.VCI_InitCAN.restype = ctypes.c_uint32
        self._dll.VCI_StartCAN.argtypes = [ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32]
        self._dll.VCI_StartCAN.restype = ctypes.c_uint32
        self._dll.VCI_ClearBuffer.argtypes = [ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32]
        self._dll.VCI_ClearBuffer.restype = ctypes.c_uint32
        self._dll.VCI_Receive.argtypes = [
            ctypes.c_uint32,
            ctypes.c_uint32,
            ctypes.c_uint32,
            ctypes.POINTER(VCI_CAN_OBJ),
            ctypes.c_uint32,
            ctypes.c_int,
        ]
        self._dll.VCI_Receive.restype = ctypes.c_uint32
        self._dll.VCI_Transmit.argtypes = [
            ctypes.c_uint32,
            ctypes.c_uint32,
            ctypes.c_uint32,
            ctypes.POINTER(VCI_CAN_OBJ),
            ctypes.c_uint32,
        ]
        self._dll.VCI_Transmit.restype = ctypes.c_uint32

    def open(self) -> None:
        result = self._dll.VCI_OpenDevice(self.device_type, self.device_index, 0)
        if result != STATUS_OK:
            raise RuntimeError(f"VCI_OpenDevice failed: {result}")

    def close(self) -> None:
        self._dll.VCI_CloseDevice(self.device_type, self.device_index)

    def init_can(self, channel: int, bitrate: int, listen_only: bool = False) -> None:
        if bitrate not in BITRATE_TIMING:
            raise ValueError(f"unsupported bitrate {bitrate}; known: {sorted(BITRATE_TIMING)}")
        timing0, timing1 = BITRATE_TIMING[bitrate]
        config = VCI_INIT_CONFIG(
            AccCode=0,
            AccMask=0xFFFFFFFF,
            Reserved=0,
            Filter=1,
            Timing0=timing0,
            Timing1=timing1,
            Mode=1 if listen_only else 0,
        )
        result = self._dll.VCI_InitCAN(self.device_type, self.device_index, channel, ctypes.byref(config))
        if result != STATUS_OK:
            raise RuntimeError(f"VCI_InitCAN channel={channel} failed: {result}")
        self._dll.VCI_ClearBuffer(self.device_type, self.device_index, channel)
        result = self._dll.VCI_StartCAN(self.device_type, self.device_index, channel)
        if result != STATUS_OK:
            raise RuntimeError(f"VCI_StartCAN channel={channel} failed: {result}")

    def receive(self, channel: int, limit: int = 100, wait_ms: int = 100) -> list[VCI_CAN_OBJ]:
        buffer_type = VCI_CAN_OBJ * limit
        buffer = buffer_type()
        count = self._dll.VCI_Receive(
            self.device_type,
            self.device_index,
            channel,
            buffer,
            limit,
            wait_ms,
        )
        return [buffer[i] for i in range(count)]

    def transmit_frames(self, channel: int, frames: Iterable[VCI_CAN_OBJ]) -> int:
        frame_list = list(frames)
        if not frame_list:
            return 0
        buffer_type = VCI_CAN_OBJ * len(frame_list)
        buffer = buffer_type(*frame_list)
        return int(self._dll.VCI_Transmit(self.device_type, self.device_index, channel, buffer, len(frame_list)))
