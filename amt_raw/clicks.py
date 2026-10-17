from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Optional, Set

from evdev import ecodes

from .frames import RawFrame, Touch
from .source import SourceInfo

LOG = logging.getLogger("amt-raw")


@dataclass
class PhysicalClickPolicy:
    enabled: bool
    button_areas: bool
    area_height_mm: float
    info: SourceInfo
    release_pressure: int = 30
    release_debounce_frames: int = 2
    held_button: Optional[int] = None
    held_finger_id: Optional[int] = None
    _low_pressure_frames: int = 0

    @property
    def area_threshold_y(self) -> float:
        if self.info.y_resolution <= 0:
            return float("inf")
        return self.info.y_max - self.area_height_mm * self.info.y_resolution

    def touch_in_button_area(self, touch: Touch) -> bool:
        return self.button_areas and touch.y >= self.area_threshold_y

    def button_for_touch(self, touch: Touch) -> int:
        """Return the logical button for a contact's horizontal position."""
        width = self.info.x_max - self.info.x_min
        if width <= 0:
            return ecodes.BTN_LEFT
        rel = (touch.x - self.info.x_min) / width
        if rel < 1.0 / 3.0:
            return ecodes.BTN_LEFT
        if rel < 2.0 / 3.0:
            return ecodes.BTN_MIDDLE
        return ecodes.BTN_RIGHT

    def _pick_area_touch(
        self,
        frame: RawFrame,
        allowed_ids: Optional[Set[int]] = None,
    ) -> Optional[Touch]:
        if not frame.touches:
            return None

        if not self.button_areas:
            # Ordinary clickpad behavior: the raw hardware bit is simply left.
            # No contact needs to be latched for the pressure-release fallback.
            return None

        # In enhanced modes button-area contacts may be classified at touch-down
        # and kept isolated from pointer/touchpad motion. allowed_ids restricts a
        # physical click to those sticky button contacts, so a pointer contact
        # that later wanders into the strip cannot suddenly become a button.
        if allowed_ids is None:
            candidates = [t for t in frame.touches if self.touch_in_button_area(t)]
        else:
            candidates = [t for t in frame.touches if t.finger_id in allowed_ids]
        if not candidates:
            return None

        # Favor the contact deepest into the area. This naturally chooses a
        # thumb if another button contact is also present.
        return max(candidates, key=lambda t: t.y)

    def press(
        self,
        frame: RawFrame,
        allowed_ids: Optional[Set[int]] = None,
    ) -> Optional[int]:
        if not self.enabled:
            self.reset()
            return None

        self._low_pressure_frames = 0

        if not self.button_areas:
            self.held_button = ecodes.BTN_LEFT
            self.held_finger_id = None
            return self.held_button

        touch = self._pick_area_touch(frame, allowed_ids=allowed_ids)
        if touch is None:
            self.held_button = None
            self.held_finger_id = None
            LOG.debug(
                "physical click suppressed outside/sticky-excluded bottom %.1f mm button area",
                self.area_height_mm,
            )
            return None

        self.held_button = self.button_for_touch(touch)
        self.held_finger_id = touch.finger_id
        return self.held_button

    def release(self) -> Optional[int]:
        code = self.held_button
        self.held_button = None
        self.held_finger_id = None
        self._low_pressure_frames = 0
        return code

    def maybe_release_from_frame(self, frame: RawFrame) -> Optional[int]:
        """Release a held custom-area click when the thumb pressure is released.

        The Magic Trackpad's raw BTN_LEFT bit is normally authoritative.  During
        a multi-contact click-hold-drag, however, users can release force on the
        button-zone thumb while another finger remains on the surface.  A small
        pressure fallback makes the logical button-up follow that thumb release
        instead of waiting for all touches to disappear.

        This fallback is intentionally restricted to custom button areas, where
        we know which tracking id owns the physical button.  Ordinary clickpad
        mode continues to follow raw BTN_LEFT only.
        """
        if (
            self.held_button is None
            or not self.button_areas
            or self.held_finger_id is None
            or self.release_pressure < 0
        ):
            return None

        touch = next(
            (t for t in frame.touches if t.finger_id == self.held_finger_id),
            None,
        )
        if touch is None:
            LOG.debug("button-area owner contact lifted; forcing logical button release")
            return self.release()

        if touch.pressure <= self.release_pressure:
            self._low_pressure_frames += 1
        else:
            self._low_pressure_frames = 0

        needed = max(1, int(self.release_debounce_frames))
        if self._low_pressure_frames >= needed:
            LOG.debug(
                "button-area pressure release: id=%d pressure=%d <= %d for %d frames",
                touch.finger_id,
                touch.pressure,
                self.release_pressure,
                self._low_pressure_frames,
            )
            return self.release()

        return None

    def reset(self):
        self.held_button = None
        self.held_finger_id = None
        self._low_pressure_frames = 0
