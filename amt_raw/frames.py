from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class Touch:
    finger_id: int
    x: int
    y: int
    pressure: int = 0
    major: int = 0
    minor: int = 0
    orientation: int = 0
    contact_status: int = 1


@dataclass(frozen=True)
class RawFrame:
    timestamp: int
    touches: tuple[Touch, ...]
    spurious: bool = False
    end_of_frame: bool = True

    @property
    def finger_count(self) -> int:
        return len(self.touches)


def active_touches(frame: RawFrame) -> list[Touch]:
    return list(frame.touches)


def active_single_touch(frame: RawFrame) -> Optional[Touch]:
    return frame.touches[0] if len(frame.touches) == 1 else None
