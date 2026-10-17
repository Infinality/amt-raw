from __future__ import annotations

import logging
import math
from collections import deque
from dataclasses import dataclass, field
from typing import Iterable, Optional

from .frames import RawFrame, Touch

LOG = logging.getLogger("amt-raw")


@dataclass
class _ContactState:
    last_pressure: int
    last_x: int
    last_y: int
    fall_frames: int
    fall_start_pressure: int
    frozen: bool = False
    frozen_x: int = 0
    frozen_y: int = 0
    freeze_ts: int = 0
    offset_x: int = 0
    offset_y: int = 0
    motion_history: deque[tuple[int, int, int]] = field(default_factory=deque)
    low_pressure_entry_speed: Optional[float] = None


class LiftOffJitterFilter:
    """Suppress only the final coordinate tail of a contact that is lifting.

    Magic Trackpad contacts commonly remain alive for a few reports while
    pressure collapses toward zero. Coordinates can move during those final
    reports even though the user's intent is to lift at the already-selected
    position. This filter leaves normal motion untouched and intervenes only
    after a conservative release signature is visible:

      * pressure reaches zero after a positive-pressure report; or
      * pressure has fallen monotonically for ``fall_frames`` reports, has
        dropped by at least ``pressure_drop``, and is now at or below
        ``pressure_threshold``.

    A pressure signature may freeze coordinates only when the recent physical
    path speed is in the configured precision/stillness regime. Fast movement
    therefore bypasses suppression even if pressure briefly collapses.

    A release signature is deliberately a *candidate*, not a one-way latch.
    Some real contacts can transiently report very low/zero pressure while the
    finger is still down. While a candidate is frozen, either a clear pressure
    rebound or the contact remaining alive beyond the normal lift-tail window
    proves that the candidate was false. In that case filtering resumes without
    a catch-up jump: a
    per-contact coordinate offset makes the first recovered sample coincide
    with the frozen output position, then subsequent raw deltas pass through
    normally for the rest of that tracking-ID lifetime.

    Pressure, touch size, orientation, contact status, and tracking identity
    always remain physical.
    """

    def __init__(
        self,
        *,
        enabled: bool,
        pressure_threshold: int,
        fall_frames: int,
        pressure_drop: int,
        recovery_pressure: int,
        recovery_timeout_ms: float,
        motion_window_ms: float,
        max_path_speed: float,
    ):
        self.enabled = enabled
        self.pressure_threshold = pressure_threshold
        self.fall_frames = fall_frames
        self.pressure_drop = pressure_drop
        self.recovery_pressure = recovery_pressure
        self.recovery_timeout_ms = recovery_timeout_ms
        self.motion_window_ms = motion_window_ms
        self.max_path_speed = max_path_speed
        self._states: dict[int, _ContactState] = {}

    def reset(self) -> None:
        self._states.clear()

    @staticmethod
    def _elapsed_ms(new_ts: int, old_ts: int) -> float:
        return float((new_ts - old_ts) & 0xFFFF)

    def _record_motion_sample(
        self, state: _ContactState, timestamp: int, x: int, y: int
    ) -> None:
        state.motion_history.append((timestamp, x, y))
        # Keep only the recent physical path used to decide whether this is a
        # precision/stillness release.  Path length, rather than net
        # displacement, is intentional: fast circular motion can return near
        # its starting point while still being unmistakably active motion.
        while len(state.motion_history) > 2:
            age = self._elapsed_ms(timestamp, state.motion_history[0][0])
            if age <= self.motion_window_ms:
                break
            state.motion_history.popleft()

    def _recent_path_speed(self, state: _ContactState) -> Optional[float]:
        history = state.motion_history
        if len(history) < 3:
            return None
        elapsed = self._elapsed_ms(history[-1][0], history[0][0])
        # Two real report intervals are enough to distinguish a tap/precision
        # placement from an actively moving contact without adding latency.
        if elapsed < min(18.0, self.motion_window_ms):
            return None
        path = 0.0
        for (_, x0, y0), (_, x1, y1) in zip(history, list(history)[1:]):
            path += math.hypot(x1 - x0, y1 - y0)
        return path / max(elapsed, 0.001)

    @staticmethod
    def _with_xy(touch: Touch, x: int, y: int) -> Touch:
        return Touch(
            finger_id=touch.finger_id,
            x=x,
            y=y,
            pressure=touch.pressure,
            major=touch.major,
            minor=touch.minor,
            orientation=touch.orientation,
            contact_status=touch.contact_status,
        )

    def _recover(
        self, state: _ContactState, touch: Touch, timestamp: int, *, reason: str
    ) -> Touch:
        # Do not replay the movement accumulated while the release candidate was
        # frozen: that would create the very jump this filter exists to avoid.
        # Instead establish a temporary coordinate offset so this first recovery
        # sample is continuous with the frozen output position. Future *deltas*
        # are then passed through one-for-one until the tracking ID ends.
        state.offset_x = state.frozen_x - touch.x
        state.offset_y = state.frozen_y - touch.y
        state.frozen = False
        state.last_pressure = touch.pressure
        state.fall_frames = 1
        state.fall_start_pressure = touch.pressure
        state.last_x = touch.x + state.offset_x
        state.last_y = touch.y + state.offset_y
        state.motion_history.clear()
        self._record_motion_sample(state, timestamp, touch.x, touch.y)
        state.low_pressure_entry_speed = None
        LOG.debug(
            "lift-off jitter suppression recovered id=%d pressure=%d "
            "raw=(%d,%d) output=(%d,%d) offset=(%d,%d) reason=%s",
            touch.finger_id,
            touch.pressure,
            touch.x,
            touch.y,
            state.last_x,
            state.last_y,
            state.offset_x,
            state.offset_y,
            reason,
        )
        return self._with_xy(touch, state.last_x, state.last_y)

    def filter_frame(
        self,
        frame: RawFrame,
        *,
        ignored_ids: Iterable[int] = (),
    ) -> RawFrame:
        """Return a frame with only release-tail x/y potentially frozen.

        ``ignored_ids`` are passed through untouched and are not tracked. This
        is used for custom button-area contacts so physical click policy always
        sees their completely raw geometry/pressure behavior.
        """
        if not self.enabled:
            return frame

        ignored = set(ignored_ids)
        active_ids = {t.finger_id for t in frame.touches if t.finger_id not in ignored}
        for fid in tuple(self._states):
            if fid not in active_ids:
                self._states.pop(fid, None)

        out: list[Touch] = []
        for touch in frame.touches:
            fid = touch.finger_id
            if fid in ignored:
                self._states.pop(fid, None)
                out.append(touch)
                continue

            state = self._states.get(fid)
            if state is None:
                # Never treat the very first report as release, even if its
                # pressure is zero. Initial contact acquisition can begin at
                # zero/very-low pressure before the pad settles.
                state = _ContactState(
                    last_pressure=touch.pressure,
                    last_x=touch.x,
                    last_y=touch.y,
                    fall_frames=1,
                    fall_start_pressure=touch.pressure,
                )
                self._record_motion_sample(
                    state, frame.timestamp, touch.x, touch.y
                )
                self._states[fid] = state
                out.append(touch)
                continue

            if state.frozen:
                elapsed_ms = (frame.timestamp - state.freeze_ts) & 0xFFFF
                pressure_recovered = touch.pressure >= self.recovery_pressure
                timeout_recovered = elapsed_ms >= self.recovery_timeout_ms

                if pressure_recovered or timeout_recovered:
                    reason = (
                        "pressure-rebound"
                        if pressure_recovered
                        else f"candidate-timeout-{elapsed_ms:.1f}ms"
                    )
                    out.append(
                        self._recover(state, touch, frame.timestamp, reason=reason)
                    )
                    continue

                state.last_pressure = touch.pressure
                out.append(self._with_xy(touch, state.frozen_x, state.frozen_y))
                continue

            # Evaluate motion *before* adding this report.  That keeps the
            # release-tail displacement itself from deciding whether the user
            # had been moving intentionally.
            approach_speed = self._recent_path_speed(state)
            previous_pressure = state.last_pressure
            if touch.pressure < previous_pressure:
                if state.fall_frames <= 1:
                    state.fall_start_pressure = previous_pressure
                    state.fall_frames = 2
                else:
                    state.fall_frames += 1
            else:
                state.fall_frames = 1
                state.fall_start_pressure = touch.pressure

            # Capture intent at entry into the low-pressure release region,
            # rather than at the first pressure decline. Pressure often starts
            # falling naturally very early in a short tap, before enough motion
            # history exists to classify it.
            if touch.pressure <= self.pressure_threshold:
                if previous_pressure > self.pressure_threshold:
                    state.low_pressure_entry_speed = approach_speed
            else:
                state.low_pressure_entry_speed = None

            total_drop = state.fall_start_pressure - touch.pressure
            zero_release = touch.pressure == 0 and previous_pressure > 0
            trend_release = (
                touch.pressure <= self.pressure_threshold
                and state.fall_frames >= self.fall_frames
                and total_drop >= self.pressure_drop
            )
            candidate_speed = (
                state.low_pressure_entry_speed
                if state.low_pressure_entry_speed is not None
                else approach_speed
            )
            precision_release = (
                candidate_speed is not None
                and candidate_speed <= self.max_path_speed
            )

            if (zero_release or trend_release) and precision_release:
                state.frozen = True
                state.frozen_x = state.last_x
                state.frozen_y = state.last_y
                state.freeze_ts = frame.timestamp
                LOG.debug(
                    "lift-off jitter suppression engaged id=%d pressure=%d "
                    "previous=%d fall_frames=%d drop=%d path_speed=%.3f "
                    "anchor=(%d,%d)%s",
                    fid,
                    touch.pressure,
                    previous_pressure,
                    state.fall_frames,
                    total_drop,
                    candidate_speed,
                    state.frozen_x,
                    state.frozen_y,
                    " zero-pressure" if zero_release else "",
                )
                state.last_pressure = touch.pressure
                out.append(self._with_xy(touch, state.frozen_x, state.frozen_y))
                continue

            if (zero_release or trend_release) and not precision_release:
                LOG.debug(
                    "lift-off jitter suppression bypassed id=%d pressure=%d "
                    "path_speed=%s threshold=%.3f (active motion)",
                    fid,
                    touch.pressure,
                    "insufficient-history"
                    if candidate_speed is None
                    else f"{candidate_speed:.3f}",
                    self.max_path_speed,
                )

            self._record_motion_sample(
                state, frame.timestamp, touch.x, touch.y
            )
            output_x = touch.x + state.offset_x
            output_y = touch.y + state.offset_y
            state.last_pressure = touch.pressure
            state.last_x = output_x
            state.last_y = output_y
            out.append(self._with_xy(touch, output_x, output_y))

        return RawFrame(
            timestamp=frame.timestamp,
            touches=tuple(out),
            spurious=frame.spurious,
            end_of_frame=frame.end_of_frame,
        )
