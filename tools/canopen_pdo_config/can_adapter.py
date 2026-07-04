"""CAN adapter abstractions for the PDO configurator.

The configurator never talks to a concrete CAN device directly.  Production
access goes through a small adapter, while tests use the mock adapter.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol


@dataclass(frozen=True)
class CanFrame:
    can_id: int
    data: bytes
    is_extended: bool = False
    is_remote: bool = False

    @property
    def dlc(self) -> int:
        return len(self.data)

    def to_json(self) -> dict[str, object]:
        return {
            "id": f"0x{self.can_id:03X}",
            "id_int": self.can_id,
            "dlc": self.dlc,
            "data": self.data.hex(" ").upper(),
            "extended": self.is_extended,
            "remote": self.is_remote,
        }


class CanAdapter(Protocol):
    def open(self, buses: list[str], bitrate: int) -> None:
        ...

    def close(self) -> None:
        ...

    def send(self, bus: str, frame: CanFrame) -> None:
        ...

    def receive(self, bus: str, expected_can_id: int, timeout_ms: int) -> CanFrame:
        ...

