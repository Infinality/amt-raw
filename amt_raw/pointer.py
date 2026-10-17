from __future__ import annotations

import logging
import math
import threading
import time
from collections import deque
from typing import Optional

from evdev import UInput, ecodes

from .frames import Touch, RawFrame, active_touches
from .clicks import PhysicalClickPolicy
from .source import SourceInfo
from .liftoff import LiftOffJitterFilter
from .resample import PointerMotionResampler

LOG = logging.getLogger("amt-raw")


def make_pointer_uinput(name: str) -> UInput:
    capabilities = {
        ecodes.EV_REL: [
            ecodes.REL_X, ecodes.REL_Y,
            ecodes.REL_WHEEL, ecodes.REL_HWHEEL,
            ecodes.REL_WHEEL_HI_RES, ecodes.REL_HWHEEL_HI_RES,
        ],
        ecodes.EV_KEY: [ecodes.BTN_LEFT, ecodes.BTN_MIDDLE, ecodes.BTN_RIGHT],
    }
    return UInput(
        capabilities, name=name, vendor=0x05AC, product=0xA324,
        version=1, bustype=ecodes.BUS_USB,
    )


class RawPointerEngine:
    """Pointer, tap, and scroll processing for absolute multitouch frames.

    Pointer motion uses a base linear gain.  At very low raw velocity an
    optional precision multiplier reduces that gain, then smoothly returns to
    exactly 1.0 above precision_transition.  KDE/libinput can therefore remain
    Flat/neutral.

    Short grace windows around one-finger <-> two-finger transitions prevent
    asynchronous finger landing/lifting from being misinterpreted as pointer
    motion.  During those windows the pointer anchor follows the remaining
    finger but no REL_X/REL_Y is emitted.
    """

    def __init__(
        self,
        gain_x: float,
        gain_y: float,
        invert_x: bool,
        invert_y: bool,
        max_raw_jump: int,
        pointer_hysteresis_units: float,
        scroll_hysteresis_units: float,
        ui: Optional[UInput],
        print_raw: bool,
        timestamp_unit_ms: float,
        precision_gain: float,
        precision_speed: float,
        precision_transition: float,
        precision_filter_ms: float,
        tap_enabled: bool,
        two_finger_tap_enabled: bool,
        three_finger_tap_enabled: bool,
        tap_max_ms: float,
        two_finger_tap_grace_ms: float,
        tap_move_units: float,
        tap_landing_move_units: float,
        scroll_enabled: bool,
        scroll_start_units: float,
        scroll_units_per_detent: float,
        scroll_invert: bool,
        horizontal_scroll: bool,
        hscroll_units_per_detent: float,
        hscroll_invert: bool,
        multitouch_entry_grace_ms: float,
        multitouch_exit_grace_ms: float,
        tap_drag_window_ms: float,
        tap_drag_activation_units: float,
        scroll_axis_lock: bool,
        scroll_axis_lock_ratio: float,
        kinetic_scroll: bool,
        kinetic_history_ms: float,
        kinetic_start_velocity: float,
        kinetic_stop_velocity: float,
        kinetic_decay_ms: float,
        kinetic_max_ms: float,
        tap_drag_hold_ms: float = 120.0,
        resampling_rate_hz: float = 120.0,
        resampling_predictor: str = "interpolated",
        kinetic_ui: Optional[UInput] = None,
    ):
        self.gain_x = gain_x
        self.gain_y = gain_y
        self.x_sign = -1.0 if invert_x else 1.0
        self.y_sign = -1.0 if invert_y else 1.0
        self.max_raw_jump = max_raw_jump
        self.pointer_hysteresis_units = pointer_hysteresis_units
        self.scroll_hysteresis_units = scroll_hysteresis_units
        self.ui = ui
        # Optional dedicated output device for post-release kinetic scrolling.
        # Pointer mode leaves this unset and therefore keeps the historical
        # single-device behavior. Touchpad mode can route coast events through
        # a separate companion pointer while live scrolling remains on scroll_ui.
        self.kinetic_ui = kinetic_ui
        self.print_raw = print_raw
        self.timestamp_unit_ms = timestamp_unit_ms

        self.precision_gain = precision_gain
        self.precision_speed = precision_speed
        self.precision_transition = precision_transition
        self.precision_filter_ms = precision_filter_ms

        self.tap_enabled = tap_enabled
        self.two_finger_tap_enabled = two_finger_tap_enabled
        self.three_finger_tap_enabled = three_finger_tap_enabled
        self.tap_max_ms = tap_max_ms
        self.two_finger_tap_grace_ms = two_finger_tap_grace_ms
        self.tap_move_units = tap_move_units
        self.tap_landing_move_units = tap_landing_move_units

        self.scroll_enabled = scroll_enabled
        self.scroll_start_units = scroll_start_units
        self.scroll_units_per_detent = scroll_units_per_detent
        self.scroll_invert = scroll_invert
        self.horizontal_scroll = horizontal_scroll
        self.hscroll_units_per_detent = hscroll_units_per_detent
        self.hscroll_invert = hscroll_invert

        self.multitouch_entry_grace_ms = multitouch_entry_grace_ms
        self.multitouch_exit_grace_ms = multitouch_exit_grace_ms

        # Match libinput's tap-and-drag model: a valid tap presses BTN_LEFT
        # when the tap completes, but the release is held briefly.  If a
        # second finger lands inside this window, the already-held button can
        # become a drag without generating a second BTN_LEFT press (and thus
        # without triggering a desktop double-click action first).  If no
        # second touch arrives, the held press is released by a timer and the
        # ordinary single click completes.
        self.tap_drag_window_ms = tap_drag_window_ms

        # On a second touch after a tap, suppress only this small amount of
        # motion while deciding double-click vs. drag.  The button is already
        # logically held from the first tap.  Release inside the threshold
        # completes a normal second click; crossing it arms drag motion.
        self.tap_drag_activation_units = tap_drag_activation_units

        # A deliberately held second touch should not remain motion-frozen
        # indefinitely while waiting to cross the displacement threshold.
        # Short retouches can still complete a double tap; a retouch that
        # remains down this long is promoted to drag even with small motion.
        # Zero disables time-based promotion and restores distance-only logic.
        self.tap_drag_hold_ms = tap_drag_hold_ms

        # Axis locking only matters when horizontal scrolling is enabled.
        # A sufficiently dominant initial axis stays locked for the remainder
        # of that two-finger scroll gesture.
        self.scroll_axis_lock = scroll_axis_lock
        self.scroll_axis_lock_ratio = scroll_axis_lock_ratio

        # Stock K400 firmware performs post-release wheel coasting itself:
        # full wheel detents continue after both fingers leave, with event
        # intervals increasing as velocity decays.  Recreate that behavior
        # from the raw centroid velocity.
        self.kinetic_scroll = kinetic_scroll
        self.kinetic_history_ms = kinetic_history_ms
        self.kinetic_start_velocity = kinetic_start_velocity
        self.kinetic_stop_velocity = kinetic_stop_velocity
        self.kinetic_decay_ms = kinetic_decay_ms
        self.kinetic_max_ms = kinetic_max_ms
        self._scroll_velocity_history = deque()
        self._kinetic_lock = threading.Lock()
        self._kinetic_cancel: Optional[threading.Event] = None
        self._kinetic_thread: Optional[threading.Thread] = None

        self._emit_lock = threading.Lock()

        # Optional fixed-rate pointer-motion presentation layer.  Every tap,
        # gesture, lift-off, gain, and precision decision remains on the
        # physical-report clock; only the already interpreted post-gain
        # REL_X/REL_Y trajectory is resampled.
        self.resampler = PointerMotionResampler(
            rate_hz=resampling_rate_hz,
            emit_relative=self._emit_resampled_relative,
            predictor=resampling_predictor,
        )

        # Tap-and-drag state.  A valid tap leaves BTN_LEFT logically held for
        # a short window, mirroring libinput's TAP_STATE_*TAPPED behavior.
        # A second touch either turns that held press into a drag or, if it is
        # released without meaningful motion, completes a second click.
        self._drag_source_deadline_ns = 0
        self._drag_source_x = 0
        self._drag_source_y = 0
        self._tap_drag_candidate = False
        self._tap_drag_active = False
        self._tap_hold_active = False
        self._tap_hold_timer: Optional[threading.Timer] = None
        self._tap_hold_generation = 0
        self._tap_state_lock = threading.RLock()

        # One-finger pointer state.
        self.prev_x: Optional[float] = None
        self.prev_y: Optional[float] = None
        self.prev_finger_id: Optional[int] = None
        self.prev_hid_ts: Optional[int] = None
        # Pointer-only absolute-coordinate hysteresis.  The Magic Trackpad
        # reports tiny coordinate changes under an apparently stationary
        # finger; keep a small deadband around the last accepted position so
        # those samples do not become REL jitter.  Deliberate slow motion is
        # preserved once it crosses the margin.
        self.hysteresis_x: Optional[float] = None
        self.hysteresis_y: Optional[float] = None
        self.frac_x = 0.0
        self.frac_y = 0.0
        self.filtered_speed: Optional[float] = None

        # One-finger tap candidate.
        self.single_active = False
        self.single_start_ns = 0
        # Physical frame timestamp at contact start.  Tap duration is a
        # property of the hardware contact, so qualification uses this
        # millisecond timestamp rather than host-side processing latency.
        self.single_start_hid_ts = 0
        self.single_last_hid_ts = 0
        self.single_start_x = 0
        self.single_start_y = 0
        self.single_max_move = 0.0
        self.single_landing_max_move = 0.0
        self.single_tap_candidate = False
        # Fresh one-finger contacts wait briefly before pointer motion is
        # allowed, giving a slightly delayed second finger time to arrive.
        self.single_motion_grace_until_ns = 0

        # Multi-touch state.
        self.multitouch_session = False
        self.two_active = False
        self.two_start_ns = 0
        self.two_start_centroid: Optional[tuple[float, float]] = None
        self.two_last_centroid: Optional[tuple[float, float]] = None
        # When the second finger first lands, both contacts can shift several
        # raw units merely from settling onto the glass.  Reuse the existing
        # multitouch entry grace to absorb that landing transient before
        # deciding tap vs. scroll.  This is gesture arbitration only; once the
        # grace expires the normal fine-scroll path is unchanged.
        self.two_motion_grace_until_ns = 0
        # Two-finger scroll hysteresis is applied to each contact *before*
        # averaging the pair.  This mirrors the successful one-finger pointer
        # deadband and suppresses independent per-finger sensor wobble without
        # batching or quantizing coherent scroll motion.  Once a finger escapes
        # the margin, one-unit raw steps remain one-unit accepted steps.
        self.scroll_touch_hysteresis: dict[int, tuple[float, float]] = {}
        # Optional pre-scroll threshold.  Filtered
        # motion is accumulated and replayed when this threshold is crossed, so
        # slow motion is never discarded.  The default is zero: the per-contact
        # hysteresis itself provides the stationary-noise gate.
        self.scroll_pending_x = 0.0
        self.scroll_pending_y = 0.0
        self.two_start_positions: dict[int, tuple[int, int]] = {}
        self.two_max_finger_move = 0.0
        self.two_tap_candidate = False
        self.two_scrolling = False
        self.scroll_axis: Optional[str] = None
        # High-resolution wheel conversion state.
        #
        # *_v120_frac preserves sub-v120 fractions from the raw->wheel
        # conversion.  *_legacy_v120 accumulates integer v120 output until a
        # full +/-120 detent is reached, at which point the corresponding
        # legacy REL_WHEEL/REL_HWHEEL event is emitted for compatibility.
        self.scroll_v120_frac = 0.0
        self.hscroll_v120_frac = 0.0
        self.scroll_legacy_v120 = 0
        self.hscroll_legacy_v120 = 0

        # When a scrolling gesture temporarily falls from two fingers to one,
        # wait briefly before handing the remaining finger to pointer motion.
        self.scroll_exit_pending = False
        self.scroll_exit_until_ns = 0

        # Once 3+ contacts are seen, suppress pointer/scroll interpretation
        # until every contact is lifted.  This keeps three/four-finger
        # gestures available to an external recognizer without partial
        # pointer/scroll output while fingers land or lift asynchronously.
        self.external_gesture_active = False
        self.external_gesture_from_scroll = False

        # Three-finger tap candidate.  Three/four-finger swipes remain reserved
        # for the external gesture recognizer; we only emit BTN_MIDDLE after a
        # complete exactly-three-finger contact session stays within the normal
        # tap duration/movement limits.
        self.three_start_ns = 0
        self.three_start_positions: dict[int, tuple[int, int]] = {}
        self.three_max_finger_move = 0.0
        self.three_tap_candidate = False

    def _emit_resampled_relative(self, out_x: int, out_y: int) -> None:
        if self.ui is None or (out_x == 0 and out_y == 0):
            return
        with self._emit_lock:
            if out_x:
                self.ui.write(ecodes.EV_REL, ecodes.REL_X, out_x)
            if out_y:
                self.ui.write(ecodes.EV_REL, ecodes.REL_Y, out_y)
            self.ui.syn()

    def _reset_pointer(self, touch: Optional[Touch] = None, hid_ts: Optional[int] = None,
                       reset_fraction: bool = True):
        if touch is None:
            self.prev_x = None
            self.prev_y = None
            self.prev_finger_id = None
            self.prev_hid_ts = None
            self.hysteresis_x = None
            self.hysteresis_y = None
            self.filtered_speed = None
        else:
            self.prev_x = float(touch.x)
            self.prev_y = float(touch.y)
            self.hysteresis_x = float(touch.x)
            self.hysteresis_y = float(touch.y)
            self.prev_finger_id = touch.finger_id
            self.prev_hid_ts = hid_ts

        if reset_fraction:
            self.frac_x = 0.0
            self.frac_y = 0.0
            self.resampler.reset("pointer anchor/state reset")

    def reset_runtime(self):
        """Cancel all in-progress pointer/gesture state without emitting events.

        Used after a device reconnect/power-cycle so stale absolute coordinates
        can never turn into a large relative jump.
        """
        self._cancel_kinetic_scroll()
        self._cancel_tap_sequence("runtime reset", release_button=True)
        self._drag_source_deadline_ns = 0
        self._tap_drag_candidate = False
        self._reset_pointer()
        self.single_active = False
        self.single_tap_candidate = False
        self.single_start_ns = 0
        self.single_start_hid_ts = 0
        self.single_last_hid_ts = 0
        self.single_max_move = 0.0
        self.single_landing_max_move = 0.0
        self.single_motion_grace_until_ns = 0

        self.multitouch_session = False
        self.two_active = False
        self.two_tap_candidate = False
        self.two_scrolling = False
        self.scroll_axis = None
        self.two_start_ns = 0
        self.two_start_centroid = None
        self.two_last_centroid = None
        self.two_motion_grace_until_ns = 0
        self.scroll_touch_hysteresis.clear()
        self.scroll_pending_x = 0.0
        self.scroll_pending_y = 0.0
        self.two_start_positions = {}
        self.two_max_finger_move = 0.0
        self.scroll_v120_frac = 0.0
        self.hscroll_v120_frac = 0.0
        self.scroll_legacy_v120 = 0
        self.hscroll_legacy_v120 = 0
        self.scroll_exit_pending = False
        self.scroll_exit_until_ns = 0
        self._scroll_velocity_history.clear()
        self.external_gesture_active = False
        self.external_gesture_from_scroll = False
        self.three_start_ns = 0
        self.three_start_positions = {}
        self.three_max_finger_move = 0.0
        self.three_tap_candidate = False

    def _precision_multiplier(self, raw_speed: float, dt_ms: float) -> float:
        if self.precision_gain >= 0.999999:
            return 1.0

        if self.precision_filter_ms <= 0.0:
            self.filtered_speed = raw_speed
        elif self.filtered_speed is None:
            self.filtered_speed = raw_speed
        else:
            alpha = 1.0 - math.exp(-max(dt_ms, 0.001) / self.precision_filter_ms)
            self.filtered_speed += alpha * (raw_speed - self.filtered_speed)

        speed = raw_speed if self.filtered_speed is None else self.filtered_speed

        if speed <= self.precision_speed:
            return self.precision_gain
        if speed >= self.precision_transition:
            return 1.0

        # Smoothstep keeps both ends of the transition slope-free.
        t = (speed - self.precision_speed) / (
            self.precision_transition - self.precision_speed
        )
        t = max(0.0, min(1.0, t))
        smooth = t * t * (3.0 - 2.0 * t)
        return self.precision_gain + (1.0 - self.precision_gain) * smooth

    def _emit_click(self, code: int, label: str):
        if self.ui is None:
            LOG.debug("%s recognized (no uinput; not emitted)", label)
            return

        # Serialize complete uinput click sequences so button events cannot
        # interleave with motion/scroll writes from another helper thread.
        with self._emit_lock:
            self.ui.write(ecodes.EV_KEY, code, 1)
            self.ui.syn()
            self.ui.write(ecodes.EV_KEY, code, 0)
            self.ui.syn()
        LOG.debug("%s emitted", label)








    def _emit_button_state(self, code: int, pressed: bool, label: str):
        if self.ui is None:
            LOG.debug("%s recognized (no uinput; not emitted)", label)
            return

        with self._emit_lock:
            self.ui.write(ecodes.EV_KEY, code, 1 if pressed else 0)
            self.ui.syn()
        LOG.debug("%s emitted", label)


    def _cancel_tap_hold_timer_locked(self):
        timer = self._tap_hold_timer
        self._tap_hold_timer = None
        self._tap_hold_generation += 1
        if timer is not None:
            timer.cancel()

    def _tap_hold_timeout(self, generation: int):
        """Complete a normal single tap if no retouch arrived in time."""
        with self._tap_state_lock:
            if generation != self._tap_hold_generation:
                return
            self._tap_hold_timer = None
            if not self._tap_hold_active:
                return
            if self._tap_drag_candidate or self._tap_drag_active:
                return

            self._tap_hold_active = False
            self._drag_source_deadline_ns = 0
            self._emit_button_state(
                ecodes.BTN_LEFT,
                False,
                "tap hold timeout -> BTN_LEFT up",
            )

    def _arm_tap_hold(self, release_ns: int, x: int, y: int):
        """Press BTN_LEFT now and defer only its release for drag detection."""
        if self.tap_drag_window_ms <= 0:
            self._emit_click(ecodes.BTN_LEFT, "one-finger tap -> BTN_LEFT")
            self._drag_source_deadline_ns = 0
            return

        with self._tap_state_lock:
            self._cancel_tap_hold_timer_locked()

            # Defensive cleanup: this should never normally be active here,
            # but never stack a new synthetic press on top of an old one.
            if self._tap_hold_active:
                self._tap_hold_active = False
                self._emit_button_state(
                    ecodes.BTN_LEFT,
                    False,
                    "stale tap hold -> BTN_LEFT up",
                )

            self._tap_hold_active = True
            self._tap_drag_candidate = False
            self._tap_drag_active = False
            self._drag_source_deadline_ns = (
                release_ns + int(self.tap_drag_window_ms * 1_000_000.0)
            )
            self._drag_source_x = x
            self._drag_source_y = y

            self._emit_button_state(
                ecodes.BTN_LEFT,
                True,
                "one-finger tap -> BTN_LEFT down (release deferred for tap-and-drag)",
            )

            self._tap_hold_generation += 1
            generation = self._tap_hold_generation
            timer = threading.Timer(
                self.tap_drag_window_ms / 1000.0,
                self._tap_hold_timeout,
                args=(generation,),
            )
            timer.daemon = True
            self._tap_hold_timer = timer
            timer.start()

    def _begin_possible_tap_drag(self, host_ns: int, x: int, y: int) -> bool:
        self._tap_drag_candidate = False

        with self._tap_state_lock:
            if not self._tap_hold_active:
                return False
            if not self._drag_source_deadline_ns:
                return False
            if host_ns > self._drag_source_deadline_ns:
                # The timer should normally have completed the click already,
                # but handle scheduler delay deterministically here too.
                self._cancel_tap_hold_timer_locked()
                self._tap_hold_active = False
                self._drag_source_deadline_ns = 0
                self._emit_button_state(
                    ecodes.BTN_LEFT,
                    False,
                    "expired tap hold -> BTN_LEFT up",
                )
                return False

            # Keep the original BTN_LEFT press held.  Do not emit a second
            # press here: desktops commonly recognize double-click on that
            # press, which would trigger e.g. titlebar maximize before drag.
            self._cancel_tap_hold_timer_locked()
            self._tap_drag_candidate = True
            LOG.debug(
                "new touch joined held tap: dt remaining=%.1f ms "
                "drag activation threshold=%.1f raw units",
                (self._drag_source_deadline_ns - host_ns) / 1_000_000.0,
                self.tap_drag_activation_units,
            )
            return True

    def _start_tap_drag(self):
        """Convert the held first-tap press into active drag motion."""
        with self._tap_state_lock:
            if self._tap_drag_active or not self._tap_drag_candidate:
                return False
            if not self._tap_hold_active:
                return False

            self._tap_drag_candidate = False
            self._drag_source_deadline_ns = 0
            self.single_tap_candidate = False
            self.single_motion_grace_until_ns = 0
            self._tap_drag_active = True
            LOG.debug("tap-and-drag armed using already-held BTN_LEFT")
            return True

    def _release_tap_hold(self, reason: str):
        with self._tap_state_lock:
            self._cancel_tap_hold_timer_locked()
            if not self._tap_hold_active:
                return
            self._tap_hold_active = False
            self._drag_source_deadline_ns = 0
            self._emit_button_state(
                ecodes.BTN_LEFT,
                False,
                f"{reason} -> BTN_LEFT up",
            )

    def _cancel_tap_sequence(self, reason: str, release_button: bool = True):
        with self._tap_state_lock:
            self._cancel_tap_hold_timer_locked()
            self._tap_drag_candidate = False
            self._tap_drag_active = False
            self._drag_source_deadline_ns = 0
            if release_button and self._tap_hold_active:
                self._tap_hold_active = False
                self._emit_button_state(
                    ecodes.BTN_LEFT,
                    False,
                    f"tap sequence cancelled ({reason}) -> BTN_LEFT up",
                )
            elif not release_button:
                self._tap_hold_active = False

    def _end_tap_drag(self, reason: str):
        if not self._tap_drag_active:
            return

        self._tap_drag_active = False
        self._tap_drag_candidate = False
        self._release_tap_hold(f"tap-and-drag end ({reason})")

    def _emit_hires_scroll(
        self,
        wheel_v120: float = 0.0,
        hwheel_v120: float = 0.0,
    ):
        """Emit fine-grained high-resolution wheel motion.

        Linux defines +/-120 REL_*WHEEL_HI_RES units as one logical wheel
        detent.  Pointer mode therefore maps raw touch displacement directly
        into v120 units and emits those small increments at the raw report
        cadence instead of waiting for a whole detent.

        Legacy REL_WHEEL / REL_HWHEEL compatibility events are generated only
        when the running high-resolution total crosses a full +/-120 boundary.
        High-resolution-aware consumers see the fine increments; legacy-only
        consumers retain ordinary detent semantics.
        """
        if self.ui is None or (wheel_v120 == 0.0 and hwheel_v120 == 0.0):
            return (0, 0)

        with self._emit_lock:
            # Preserve fractional v120 conversion error between frames.
            self.scroll_v120_frac += wheel_v120
            wheel_hi = int(self.scroll_v120_frac)
            self.scroll_v120_frac -= wheel_hi

            self.hscroll_v120_frac += hwheel_v120
            hwheel_hi = int(self.hscroll_v120_frac)
            self.hscroll_v120_frac -= hwheel_hi

            wheel_legacy = 0
            hwheel_legacy = 0

            if wheel_hi:
                self.scroll_legacy_v120 += wheel_hi
                wheel_legacy = int(self.scroll_legacy_v120 / 120)
                self.scroll_legacy_v120 -= wheel_legacy * 120

            if hwheel_hi:
                self.hscroll_legacy_v120 += hwheel_hi
                hwheel_legacy = int(self.hscroll_legacy_v120 / 120)
                self.hscroll_legacy_v120 -= hwheel_legacy * 120

            if not (wheel_hi or hwheel_hi or wheel_legacy or hwheel_legacy):
                return (0, 0)

            # Put the high-resolution values and any corresponding legacy
            # boundary crossings in the same SYN_REPORT.
            if wheel_hi:
                self.ui.write(
                    ecodes.EV_REL,
                    ecodes.REL_WHEEL_HI_RES,
                    wheel_hi,
                )
            if wheel_legacy:
                self.ui.write(
                    ecodes.EV_REL,
                    ecodes.REL_WHEEL,
                    wheel_legacy,
                )

            if hwheel_hi:
                self.ui.write(
                    ecodes.EV_REL,
                    ecodes.REL_HWHEEL_HI_RES,
                    hwheel_hi,
                )
            if hwheel_legacy:
                self.ui.write(
                    ecodes.EV_REL,
                    ecodes.REL_HWHEEL,
                    hwheel_legacy,
                )

            self.ui.syn()
            return (wheel_hi, hwheel_hi)

    def _record_scroll_velocity(self, host_ns: int, wheel_raw_delta: float):
        """Record signed raw scroll displacement for release-velocity estimate."""
        self._scroll_velocity_history.append((host_ns, wheel_raw_delta))

        cutoff_ns = host_ns - int(self.kinetic_history_ms * 1_000_000.0)
        while (
            len(self._scroll_velocity_history) > 1
            and self._scroll_velocity_history[0][0] < cutoff_ns
        ):
            self._scroll_velocity_history.popleft()

    def _estimate_release_scroll_velocity(self, host_ns: int) -> float:
        """Return signed raw-units/ms velocity immediately before release.

        Build the history window backwards from the last *motion* sample rather
        than from the final lift frame.  The Magic Trackpad can spend several
        reports flattening/lifting after the last meaningful scroll delta;
        anchoring the cutoff to the lift timestamp can therefore erase an
        otherwise valid flick.  A separate staleness guard still prevents a
        finger that truly stopped and rested before lift from launching coast.
        """
        if len(self._scroll_velocity_history) < 2:
            LOG.debug("kinetic release estimate unavailable: fewer than 2 motion samples")
            return 0.0

        last_motion_ns = self._scroll_velocity_history[-1][0]
        release_lag_ms = max(0.0, (host_ns - last_motion_ns) / 1_000_000.0)
        # Allow one history window for motion estimation plus roughly one more
        # for asynchronous finger-lift reporting.  This is intentionally far
        # shorter than a deliberate planted-finger pause.
        release_grace_ms = max(
            self.kinetic_history_ms * 2.0,
            self.kinetic_history_ms + self.multitouch_exit_grace_ms,
        )
        if release_lag_ms > release_grace_ms:
            LOG.debug(
                "kinetic release estimate stale: last motion %.1f ms before lift "
                "(grace %.1f ms)",
                release_lag_ms,
                release_grace_ms,
            )
            return 0.0

        cutoff_ns = last_motion_ns - int(self.kinetic_history_ms * 1_000_000.0)
        samples = [
            (ts, delta)
            for ts, delta in self._scroll_velocity_history
            if ts >= cutoff_ns
        ]
        if len(samples) < 2:
            LOG.debug("kinetic release estimate unavailable: fewer than 2 samples in window")
            return 0.0

        elapsed_ms = (samples[-1][0] - samples[0][0]) / 1_000_000.0
        if elapsed_ms <= 0.0:
            return 0.0

        # The first delta belongs partly to the interval preceding its sample;
        # omit it so numerator and denominator describe the same span.
        displacement = sum(delta for _, delta in samples[1:])
        velocity = displacement / elapsed_ms
        LOG.debug(
            "kinetic release estimate: %.3f raw units/ms from %d samples "
            "over %.1f ms (last motion %.1f ms before lift)",
            velocity,
            len(samples),
            elapsed_ms,
            release_lag_ms,
        )
        return velocity

    def _cancel_kinetic_scroll(self):
        with self._kinetic_lock:
            cancel = self._kinetic_cancel
            self._kinetic_cancel = None
            self._kinetic_thread = None
        if cancel is not None:
            cancel.set()

    def _start_kinetic_scroll(self, release_velocity: float):
        """Continue the existing exponential coast using fine v120 events.

        The decay model is unchanged:

            v(t) = v0 * exp(-t / tau)

        To preserve the old pointer backend's total coast distance, first
        compute how many complete raw scroll detents that model would have
        produced before the configured stop-velocity/max-duration limit.
        The high-resolution worker then emits exactly that many logical
        detents worth of v120 motion, but subdivides the same displacement at
        an 8 ms cadence instead of emitting one +/-120 jump at a time.

        Thus:
          * release threshold is unchanged;
          * tau/stop velocity/max duration are unchanged;
          * total logical coast distance is unchanged;
          * only event granularity changes.
        """
        self._cancel_kinetic_scroll()

        if not self.kinetic_scroll:
            return

        speed = abs(release_velocity)
        if speed < self.kinetic_start_velocity:
            LOG.debug(
                "kinetic scroll not started: release velocity %.3f < %.3f units/ms",
                speed,
                self.kinetic_start_velocity,
            )
            return

        direction = 1 if release_velocity > 0 else -1
        tau = self.kinetic_decay_ms
        step_raw = self.scroll_units_per_detent

        # Determine the same continuous-decay horizon used by the old
        # full-detent implementation.
        if self.kinetic_stop_velocity > 0.0:
            if speed <= self.kinetic_stop_velocity:
                return
            stop_horizon_ms = tau * math.log(
                speed / self.kinetic_stop_velocity
            )
            horizon_ms = min(self.kinetic_max_ms, stop_horizon_ms)
        else:
            # A zero stop velocity means decay only by the max-duration cap.
            horizon_ms = self.kinetic_max_ms
        if horizon_ms <= 0.0:
            return

        total_decay_raw = speed * tau * (
            1.0 - math.exp(-horizon_ms / tau)
        )

        # The legacy full-detent implementation emitted only complete detents.  Keep
        # exactly that total coast distance so the hi-res implementation does
        # not make a flick travel farther.
        detent_count = int(total_decay_raw / step_raw)
        if detent_count <= 0:
            LOG.debug(
                "kinetic scroll not started: decay contains less than one "
                "complete scroll detent (%.1f < %.1f raw units)",
                total_decay_raw,
                step_raw,
            )
            return

        target_raw = detent_count * step_raw
        cancel = threading.Event()

        # Touchpad mode may provide a dedicated coast-only companion. Keep its
        # v120/legacy accounting independent from the live-scroll device, while
        # pointer mode intentionally stays on the existing
        # self.ui path with the existing accumulators.
        dedicated_kinetic_ui = (
            self.kinetic_ui
            if self.kinetic_ui is not None and self.kinetic_ui is not self.ui
            else None
        )
        kinetic_v120_frac = 0.0
        kinetic_legacy_v120 = 0

        def emit_kinetic_v120(v120: float) -> int:
            nonlocal kinetic_v120_frac, kinetic_legacy_v120
            if dedicated_kinetic_ui is None:
                wheel_hi, _ = self._emit_hires_scroll(v120, 0.0)
                return wheel_hi

            with self._emit_lock:
                kinetic_v120_frac += v120
                wheel_hi = int(kinetic_v120_frac)
                kinetic_v120_frac -= wheel_hi
                if not wheel_hi:
                    return 0

                kinetic_legacy_v120 += wheel_hi
                wheel_legacy = int(kinetic_legacy_v120 / 120)
                kinetic_legacy_v120 -= wheel_legacy * 120

                dedicated_kinetic_ui.write(
                    ecodes.EV_REL, ecodes.REL_WHEEL_HI_RES, wheel_hi
                )
                if wheel_legacy:
                    dedicated_kinetic_ui.write(
                        ecodes.EV_REL, ecodes.REL_WHEEL, wheel_legacy
                    )
                dedicated_kinetic_ui.syn()
                return wheel_hi

        def worker():
            # Match roughly the K400 raw-report cadence.  This is intentionally
            # a time slice, not a fixed v120 quantum: fast coast motion gets
            # larger events while slow tail motion naturally becomes very fine.
            interval_ms = 8.0
            v = speed
            emitted_raw = 0.0
            target_v120 = detent_count * 120
            emitted_v120 = 0
            start_time = time.monotonic()
            last_time = start_time
            events = 0

            LOG.debug(
                "kinetic scroll started (hi-res): release=%.3f units/ms "
                "direction=%+d tau=%.0f ms target=%d detent(s) interval=%.1f ms",
                speed,
                direction,
                tau,
                detent_count,
                interval_ms,
            )

            try:
                while not cancel.is_set() and emitted_raw < target_raw:
                    elapsed_total_ms = (time.monotonic() - start_time) * 1000.0
                    if elapsed_total_ms >= horizon_ms:
                        break

                    remaining_horizon_ms = horizon_ms - elapsed_total_ms
                    wait_ms = min(interval_ms, remaining_horizon_ms)
                    if wait_ms <= 0.0:
                        break

                    if cancel.wait(wait_ms / 1000.0):
                        break

                    now = time.monotonic()
                    actual_dt_ms = (now - last_time) * 1000.0
                    last_time = now

                    # Do not integrate past the configured decay horizon even
                    # if the Python thread was scheduled late.
                    elapsed_before_ms = elapsed_total_ms
                    allowed_dt_ms = min(
                        actual_dt_ms,
                        max(0.0, horizon_ms - elapsed_before_ms),
                    )
                    if allowed_dt_ms <= 0.0:
                        break

                    decay = math.exp(-allowed_dt_ms / tau)
                    raw_displacement = v * tau * (1.0 - decay)
                    v *= decay

                    remaining_raw = target_raw - emitted_raw
                    raw_displacement = min(raw_displacement, remaining_raw)
                    if raw_displacement <= 0.0:
                        break

                    emitted_raw += raw_displacement
                    v120 = (
                        direction
                        * raw_displacement
                        * 120.0
                        / step_raw
                    )
                    wheel_hi = emit_kinetic_v120(v120)
                    emitted_v120 += abs(wheel_hi)
                    if wheel_hi:
                        events += 1

                # Floating-point subdivision can otherwise leave a final
                # sub-unit residual (e.g. 479.999... -> 479).  The old
                # implementation's coast distance was an integer number of
                # detents, so flush exactly the remaining integer v120 units.
                remaining_v120 = target_v120 - emitted_v120
                if remaining_v120 > 0 and not cancel.is_set():
                    wheel_hi = emit_kinetic_v120(
                        direction * remaining_v120
                    )
                    emitted_v120 += abs(wheel_hi)
                    if wheel_hi:
                        events += 1

                LOG.debug(
                    "kinetic scroll ended (hi-res): events=%d "
                    "logical=%.2f detent(s) target=%d duration=%.0f ms",
                    events,
                    emitted_v120 / 120.0,
                    detent_count,
                    (time.monotonic() - start_time) * 1000.0,
                )
            finally:
                with self._kinetic_lock:
                    if self._kinetic_cancel is cancel:
                        self._kinetic_cancel = None
                        self._kinetic_thread = None

        thread = threading.Thread(
            target=worker,
            name="amt-kinetic-scroll",
            daemon=True,
        )
        with self._kinetic_lock:
            self._kinetic_cancel = cancel
            self._kinetic_thread = thread
        thread.start()

    def _start_single(self, touch: Touch, host_ns: int, hid_ts: int):
        self._cancel_kinetic_scroll()
        self._begin_possible_tap_drag(host_ns, touch.x, touch.y)

        self.single_active = True
        self.single_start_ns = host_ns
        self.single_start_hid_ts = hid_ts
        self.single_last_hid_ts = hid_ts
        self.single_start_x = touch.x
        self.single_start_y = touch.y
        self.single_max_move = 0.0
        self.single_landing_max_move = 0.0
        self.single_tap_candidate = self.tap_enabled
        self.single_motion_grace_until_ns = (
            host_ns + int(self.multitouch_entry_grace_ms * 1_000_000.0)
        )
        self._reset_pointer(touch, hid_ts)

        # A valid tap followed by a retouch is ambiguous until the second
        # contact either moves far enough to become a drag or is released as
        # the second click.  BTN_LEFT is already held from the first tap, so
        # there is deliberately no new button event here.  While unresolved,
        # process() suppresses only the small activation slop.
        if self._tap_drag_candidate:
            self.single_tap_candidate = False
            self.single_motion_grace_until_ns = 0

    def _update_single_tap(self, touch: Touch) -> bool:
        """Update post-landing tap slop."""
        if self._tap_drag_active or not self.single_tap_candidate:
            return False

        dist = math.hypot(
            touch.x - self.single_start_x,
            touch.y - self.single_start_y,
        )
        self.single_max_move = max(self.single_max_move, dist)

        if self.single_max_move <= self.tap_move_units:
            return False

        LOG.debug(
            "one-finger tap candidate cancelled by settled movement: "
            "max=%.1f raw units threshold=%.1f",
            self.single_max_move,
            self.tap_move_units,
        )
        self.single_tap_candidate = False
        self._tap_drag_candidate = False
        return False

    def _update_single_landing_tap(self, touch: Touch) -> None:
        """Track raw center travel while the fingertip is landing.

        The desktop pointer is intentionally re-anchored during the one-finger
        entry grace, so raw center settling in this interval is not visible to
        the user.  Do not permanently cancel a tap merely because that hidden
        settling is large: if the contact survives until grace expiry we can
        re-center at the settled position and let the normal tap_move_units
        threshold govern subsequent motion.

        tap_landing_move_units remains useful for a contact that *ends before*
        grace ever expires/re-centers; _finish_single() applies it there so a
        very short fast swipe is not converted into a tap.
        """
        if self._tap_drag_active or not self.single_tap_candidate:
            return

        dist = math.hypot(
            touch.x - self.single_start_x,
            touch.y - self.single_start_y,
        )
        self.single_landing_max_move = max(self.single_landing_max_move, dist)

    @staticmethod
    def _hid_elapsed_ms(start_ts: int, end_ts: int) -> float:
        """Return modular elapsed milliseconds for the 16-bit frame clock."""
        return float((end_ts - start_ts) & 0xFFFF)

    def _single_tap_durations(self, host_ns: int, release_hid_ts: int) -> tuple[float, float, float]:
        # Qualify against the span for which the contact was actually present.
        # The TRACKING_ID=-1 report arrives one hardware frame later and would
        # otherwise add up to a full report interval to a borderline tap.
        physical_active_ms = self._hid_elapsed_ms(
            self.single_start_hid_ts, self.single_last_hid_ts
        )
        release_report_gap_ms = self._hid_elapsed_ms(
            self.single_last_hid_ts, release_hid_ts
        )
        host_ms = (host_ns - self.single_start_ns) / 1_000_000.0
        return physical_active_ms, release_report_gap_ms, host_ms

    def _finish_single(self, host_ns: int, hid_ts: int):
        if self._tap_drag_active:
            self._end_tap_drag("finger released")
            self.single_active = False
            self.single_tap_candidate = False
            self.single_motion_grace_until_ns = 0
            return

        if self._tap_drag_candidate:
            # The second contact was released without crossing the drag slop.
            # The first tap's BTN_LEFT is still held.  Complete that first
            # click now, then emit the second click only if the retouch itself
            # still qualifies as a tap.  Tap qualification uses the physical
            # frame clock; host monotonic time remains authoritative for the
            # tap-drag deadline itself.
            physical_ms, release_gap_ms, host_ms = self._single_tap_durations(host_ns, hid_ts)
            self._tap_drag_candidate = False
            self._release_tap_hold("tap retouch release completes first click")

            if physical_ms <= self.tap_max_ms:
                self._emit_click(
                    ecodes.BTN_LEFT,
                    "tap retouch released -> second BTN_LEFT click",
                )
            else:
                LOG.debug(
                    "tap retouch rejected by duration: physical_active=%.1f ms "
                    "release_report_gap=%.1f ms host=%.1f ms threshold=%.1f ms "
                    "settled_move=%.1f landing_move=%.1f raw units; "
                    "second click suppressed",
                    physical_ms,
                    release_gap_ms,
                    host_ms,
                    self.tap_max_ms,
                    self.single_max_move,
                    self.single_landing_max_move,
                )

            self.single_active = False
            self.single_tap_candidate = False
            self.single_motion_grace_until_ns = 0
            return

        valid_tap = False
        if self.single_active and self.single_tap_candidate:
            # If no active report ever survived long enough to expire entry
            # grace, there was no settled coordinate at which to re-center.
            # Retain the larger landing-slop guard for that short-contact case.
            # Once grace has expired (single_motion_grace_until_ns == 0), raw
            # landing travel is historical only and must not veto the tap.
            if (
                self.single_motion_grace_until_ns
                and self.single_landing_max_move > self.tap_landing_move_units
            ):
                LOG.debug(
                    "one-finger tap rejected by pre-settle landing movement: "
                    "landing_move=%.1f raw units threshold=%.1f; "
                    "contact ended before grace re-anchor",
                    self.single_landing_max_move,
                    self.tap_landing_move_units,
                )
            else:
                physical_ms, release_gap_ms, host_ms = self._single_tap_durations(host_ns, hid_ts)
                valid_tap = physical_ms <= self.tap_max_ms
                if not valid_tap:
                    LOG.debug(
                        "one-finger tap rejected by duration: physical_active=%.1f ms "
                        "release_report_gap=%.1f ms host=%.1f ms threshold=%.1f ms "
                        "settled_move=%.1f landing_move=%.1f raw units",
                        physical_ms,
                        release_gap_ms,
                        host_ms,
                        self.tap_max_ms,
                        self.single_max_move,
                        self.single_landing_max_move,
                    )

        if valid_tap:
            # Mirror libinput's tap-and-drag state machine: emit the tap press
            # now but defer only its release for a short drag-detection window.
            # This is intentionally different from the old implementation that
            # deferred the entire first click.
            self._arm_tap_hold(
                host_ns,
                self.single_start_x,
                self.single_start_y,
            )
        else:
            self._tap_drag_candidate = False

        self.single_active = False
        self.single_tap_candidate = False
        self.single_motion_grace_until_ns = 0

    def _start_two(self, touches: list[Touch], host_ns: int):
        self._cancel_kinetic_scroll()
        self._cancel_tap_sequence("multi-touch began", release_button=True)
        self._scroll_velocity_history.clear()
        touches = touches[:2]
        cx = sum(t.x for t in touches) / 2.0
        cy = sum(t.y for t in touches) / 2.0

        self.multitouch_session = True
        self.two_active = True
        self.two_start_ns = host_ns
        self.two_start_centroid = (cx, cy)
        self.two_last_centroid = (cx, cy)
        # Two-finger tap arbitration needs a slightly longer landing window
        # than the generic one<->two transition grace.  Real AMT taps can keep
        # reporting finger-center motion for ~100 ms while the pads flatten and
        # lift.  During this window we suppress scroll output, but still track
        # raw displacement from the original landing points so a decisive
        # swipe can cancel the tap early instead of being mistaken for a click.
        two_grace_ms = self.multitouch_entry_grace_ms
        if self.two_finger_tap_enabled:
            two_grace_ms = max(two_grace_ms, self.two_finger_tap_grace_ms)
        self.two_motion_grace_until_ns = (
            host_ns + int(two_grace_ms * 1_000_000.0)
        )
        self.scroll_touch_hysteresis = {
            t.finger_id: (float(t.x), float(t.y)) for t in touches
        }
        self.scroll_pending_x = 0.0
        self.scroll_pending_y = 0.0
        self.two_start_positions = {t.finger_id: (t.x, t.y) for t in touches}
        self.two_max_finger_move = 0.0
        self.two_tap_candidate = self.two_finger_tap_enabled
        self.two_scrolling = False
        self.scroll_axis = None
        self.scroll_v120_frac = 0.0
        self.hscroll_v120_frac = 0.0
        self.scroll_exit_pending = False
        self.scroll_exit_until_ns = 0

        # A two-finger gesture cancels the single-finger tap candidate.
        self.single_tap_candidate = False
        self._reset_pointer()

    def _resume_two_after_exit_gap(self, touches: list[Touch]):
        """Resume an existing scroll after a brief 2->1->2 contact gap.

        Re-anchor the centroid so the temporary missing finger cannot turn
        into a large wheel delta when the second contact returns.
        """
        touches = touches[:2]
        cx = sum(t.x for t in touches) / 2.0
        cy = sum(t.y for t in touches) / 2.0
        self.two_last_centroid = (cx, cy)
        self.scroll_touch_hysteresis = {
            t.finger_id: (float(t.x), float(t.y)) for t in touches
        }
        self.scroll_pending_x = 0.0
        self.scroll_pending_y = 0.0
        self.two_motion_grace_until_ns = 0
        self._scroll_velocity_history.clear()
        self.scroll_exit_pending = False
        self.scroll_exit_until_ns = 0
        LOG.debug("two-finger scroll resumed inside exit grace; centroid re-anchored")

    def _filtered_two_centroid(self, touches: list[Touch]) -> tuple[float, float]:
        """Return a centroid after per-contact positional hysteresis.

        Filtering each contact before averaging matters here.  The two Magic
        Trackpad contacts can wobble independently while physically stationary;
        filtering only their already-averaged centroid allows those independent
        errors to form a random walk.  Per-contact deadbands stop that at the
        source while preserving sustained one-unit motion after the initial
        margin is crossed.
        """
        active_ids = {t.finger_id for t in touches}
        for finger_id in tuple(self.scroll_touch_hysteresis):
            if finger_id not in active_ids:
                self.scroll_touch_hysteresis.pop(finger_id, None)

        filtered = []
        for t in touches:
            accepted = self.scroll_touch_hysteresis.get(t.finger_id)
            if accepted is None:
                fx, fy = float(t.x), float(t.y)
            else:
                fx = self._hysteresis_axis(
                    float(t.x), accepted[0], self.scroll_hysteresis_units
                )
                fy = self._hysteresis_axis(
                    float(t.y), accepted[1], self.scroll_hysteresis_units
                )
            self.scroll_touch_hysteresis[t.finger_id] = (fx, fy)
            filtered.append((fx, fy))

        return (
            sum(x for x, _ in filtered) / len(filtered),
            sum(y for _, y in filtered) / len(filtered),
        )

    def _process_two(self, touches: list[Touch], host_ns: int):
        if len(touches) < 2:
            return

        touches = touches[:2]
        if not self.two_active:
            self._start_two(touches, host_ns)
            return

        ids = {t.finger_id for t in touches}
        if ids != set(self.two_start_positions):
            self.two_tap_candidate = False

        # Two-finger tap arbitration.  Real taps on the Magic Trackpad keep
        # moving their reported finger centers while the fingertips flatten and
        # lift.  Hold scroll output during the initial tap window, but measure
        # each contact against its *original* landing point.  A deliberate
        # movement that exceeds tap slop cancels the tap and ends the grace
        # early; ordinary landing motion remains a tap candidate.
        if self.two_motion_grace_until_ns:
            raw_max_move = self.two_max_finger_move
            for t in touches:
                start = self.two_start_positions.get(t.finger_id)
                if start is not None:
                    raw_max_move = max(
                        raw_max_move,
                        math.hypot(t.x - start[0], t.y - start[1]),
                    )
            self.two_max_finger_move = raw_max_move

            decisive_motion = self.two_max_finger_move > self.tap_move_units
            if decisive_motion:
                if self.two_tap_candidate:
                    LOG.debug(
                        "two-finger tap candidate cancelled during landing "
                        "grace: max=%.1f raw units threshold=%.1f",
                        self.two_max_finger_move,
                        self.tap_move_units,
                    )
                self.two_tap_candidate = False

            # Follow current raw contacts only for scroll anchoring.  Do not
            # overwrite two_start_positions/two_max_finger_move: those retain
            # the original landing geometry for tap classification.
            cx = sum(t.x for t in touches) / 2.0
            cy = sum(t.y for t in touches) / 2.0
            self.two_start_centroid = (cx, cy)
            self.two_last_centroid = (cx, cy)
            self.scroll_touch_hysteresis = {
                t.finger_id: (float(t.x), float(t.y)) for t in touches
            }
            self.scroll_pending_x = 0.0
            self.scroll_pending_y = 0.0

            if host_ns < self.two_motion_grace_until_ns and not decisive_motion:
                return

            self.two_motion_grace_until_ns = 0
            LOG.debug(
                "two-finger tap grace ended%s; scroll motion armed",
                " by decisive motion" if decisive_motion else " by timeout",
            )
            # Current report becomes the scroll anchor.  The next report
            # supplies the first delta, avoiding a catch-up jump from landing
            # motion that was intentionally filtered.
            return

        cx, cy = self._filtered_two_centroid(touches)

        # Tap slop is evaluated on the same per-contact *filtered* coordinates
        # used for scrolling, not on raw landing jitter.  Opposing finger motion
        # (for example a pinch-like move with little centroid displacement) can
        # still invalidate a tap, while stationary sensor wobble inside the
        # hysteresis band cannot.
        for finger_id, (fx, fy) in self.scroll_touch_hysteresis.items():
            start = self.two_start_positions.get(finger_id)
            if start is not None:
                self.two_max_finger_move = max(
                    self.two_max_finger_move,
                    math.hypot(fx - start[0], fy - start[1]),
                )

        if self.two_max_finger_move > self.tap_move_units:
            if self.two_tap_candidate:
                LOG.debug(
                    "two-finger tap candidate cancelled by filtered finger "
                    "travel: max=%.1f raw units threshold=%.1f",
                    self.two_max_finger_move,
                    self.tap_move_units,
                )
            self.two_tap_candidate = False

        assert self.two_start_centroid is not None
        assert self.two_last_centroid is not None

        dx = (cx - self.two_last_centroid[0]) * self.x_sign
        dy = (cy - self.two_last_centroid[1]) * self.y_sign
        self.two_last_centroid = (cx, cy)

        if not self.two_scrolling:
            # Keep every filtered pre-scroll delta.  A non-zero optional start
            # threshold delays recognition only; it never discards slow motion.
            self.scroll_pending_x += dx
            self.scroll_pending_y += dy

            threshold = self.scroll_start_units
            if threshold <= 0.0:
                start_by_motion = (dx != 0.0 or dy != 0.0)
            else:
                start_by_motion = (
                    abs(self.scroll_pending_y) >= threshold
                    or (
                        self.horizontal_scroll
                        and abs(self.scroll_pending_x) >= threshold
                    )
                )

            if not start_by_motion:
                return

            self.two_scrolling = True
            # Scroll commitment and tap commitment are mutually exclusive.
            # The entry grace above removes the Magic Trackpad landing transient
            # first, so the first post-grace filtered motion can safely make
            # this decision without turning normal landing motion into false scrolling.
            self.two_tap_candidate = False
            dx = self.scroll_pending_x
            dy = self.scroll_pending_y
            self.scroll_pending_x = 0.0
            self.scroll_pending_y = 0.0

            if not self.horizontal_scroll:
                self.scroll_axis = "vertical"
            elif not self.scroll_axis_lock:
                self.scroll_axis = "free"
            else:
                ax = abs(dx)
                ay = abs(dy)
                if ay >= ax * self.scroll_axis_lock_ratio:
                    self.scroll_axis = "vertical"
                elif ax >= ay * self.scroll_axis_lock_ratio:
                    self.scroll_axis = "horizontal"
                else:
                    self.scroll_axis = "free"

            LOG.debug(
                "two-finger scroll started (axis=%s pending_x=%.3f "
                "pending_y=%.3f threshold=%.3f hysteresis=%.3f)",
                self.scroll_axis,
                dx,
                dy,
                threshold,
                self.scroll_hysteresis_units,
            )

        if not self.scroll_enabled:
            return

        if self.scroll_axis == "vertical":
            dx = 0.0
        elif self.scroll_axis == "horizontal":
            dy = 0.0

        # Release velocity is derived from the same filtered physical centroid
        # delta that is emitted for live scrolling.
        raw_wheel_delta = -dy
        if self.scroll_invert:
            raw_wheel_delta = -raw_wheel_delta
        if raw_wheel_delta:
            self._record_scroll_velocity(host_ns, raw_wheel_delta)

        wheel_delta = -dy
        if self.scroll_invert:
            wheel_delta = -wheel_delta

        wheel_v120 = (
            wheel_delta * 120.0 / self.scroll_units_per_detent
            if wheel_delta
            else 0.0
        )

        hwheel_v120 = 0.0
        if self.horizontal_scroll:
            hdelta = dx
            if self.hscroll_invert:
                hdelta = -hdelta
            if hdelta:
                hwheel_v120 = (
                    hdelta * 120.0 / self.hscroll_units_per_detent
                )

        if wheel_v120 or hwheel_v120:
            self._emit_hires_scroll(wheel_v120, hwheel_v120)

    def _finish_two(
        self, host_ns: int, allow_kinetic: bool = True, allow_tap: bool = True
    ):
        release_velocity = 0.0
        valid_two_tap = False

        if (
            self.two_active
            and self.two_tap_candidate
            and not self.two_scrolling
        ):
            duration_ms = (host_ns - self.two_start_ns) / 1_000_000.0
            if duration_ms <= self.tap_max_ms:
                if allow_tap:
                    valid_two_tap = True
                    self._emit_click(ecodes.BTN_RIGHT, "two-finger tap -> BTN_RIGHT")
                else:
                    LOG.debug(
                        "two-finger tap suppressed by 3+ contact transition "
                        "(duration=%.1f ms move=%.1f raw units)",
                        duration_ms,
                        self.two_max_finger_move,
                    )

        if self.two_scrolling:
            if allow_kinetic and not valid_two_tap:
                release_velocity = self._estimate_release_scroll_velocity(host_ns)
            LOG.debug(
                "two-finger scroll ended%s%s",
                (
                    "; classified as tap, kinetic suppressed"
                    if valid_two_tap
                    else (
                        "; release velocity="
                        if allow_kinetic
                        else "; kinetic suppressed for one-finger handoff"
                    )
                ),
                (
                    f"{release_velocity:.3f} raw units/ms"
                    if allow_kinetic and not valid_two_tap
                    else ""
                ),
            )

        self.two_active = False
        self.two_tap_candidate = False
        was_scrolling = self.two_scrolling
        self.two_scrolling = False
        self.scroll_axis = None
        self.two_start_centroid = None
        self.two_last_centroid = None
        self.two_motion_grace_until_ns = 0
        self.scroll_touch_hysteresis.clear()
        self.scroll_pending_x = 0.0
        self.scroll_pending_y = 0.0
        self.two_start_positions = {}
        self.scroll_v120_frac = 0.0
        self.hscroll_v120_frac = 0.0
        self.scroll_exit_pending = False
        self.scroll_exit_until_ns = 0
        self._scroll_velocity_history.clear()

        if was_scrolling and allow_kinetic and not valid_two_tap:
            self._start_kinetic_scroll(release_velocity)

    def _hysteresis_axis(
        self,
        raw: float,
        accepted: Optional[float],
        margin: Optional[float] = None,
    ) -> float:
        """Apply a small stationary deadband without quantizing sustained motion.

        Unlike a per-frame delta threshold, this holds an accepted coordinate
        while raw samples remain inside +/-margin.  Once the raw position
        escapes the band, the accepted point follows at the band edge.  Thus a
        slow sequence of one-unit moves is delayed only by the initial margin,
        not discarded forever.
        """
        if margin is None:
            margin = self.pointer_hysteresis_units
        if margin <= 0.0 or accepted is None:
            return raw
        if raw > accepted + margin:
            return raw - margin
        if raw < accepted - margin:
            return raw + margin
        return accepted

    def _process_pointer_motion(self, touch: Touch, frame: RawFrame, host_ns: int):
        if (
            self.prev_x is None
            or self.prev_y is None
            or self.prev_finger_id != touch.finger_id
            or self.prev_hid_ts is None
        ):
            self._reset_pointer(touch, frame.timestamp)
            return

        filtered_x = self._hysteresis_axis(float(touch.x), self.hysteresis_x)
        filtered_y = self._hysteresis_axis(float(touch.y), self.hysteresis_y)
        sensor_dx = filtered_x - self.prev_x
        sensor_dy = filtered_y - self.prev_y
        hid_dt = (frame.timestamp - self.prev_hid_ts) & 0xFFFF
        dt_ms = max(hid_dt * self.timestamp_unit_ms, 0.001)

        self.hysteresis_x = filtered_x
        self.hysteresis_y = filtered_y
        self.prev_x = filtered_x
        self.prev_y = filtered_y
        self.prev_finger_id = touch.finger_id
        self.prev_hid_ts = frame.timestamp

        if self.max_raw_jump > 0 and (
            abs(sensor_dx) > self.max_raw_jump or abs(sensor_dy) > self.max_raw_jump
        ):
            LOG.warning(
                "Ignoring/re-anchoring implausible raw jump dx=%d dy=%d",
                sensor_dx, sensor_dy,
            )
            self.frac_x = 0.0
            self.frac_y = 0.0
            self.resampler.reset("implausible raw jump")
            self.filtered_speed = None
            self.single_tap_candidate = False
            return

        raw_speed = math.hypot(sensor_dx, sensor_dy) / dt_ms
        precision_mult = self._precision_multiplier(raw_speed, dt_ms)

        dx = sensor_dx * self.x_sign
        dy = sensor_dy * self.y_sign

        logical_dx = dx * self.gain_x * precision_mult
        logical_dy = dy * self.gain_y * precision_mult

        if self.resampler.enabled:
            # Zero-motion hardware samples are submitted too: they are the
            # causal stop signal that prevents extrapolation from coasting.
            self.resampler.submit(
                logical_dx,
                logical_dy,
                sample_ns=host_ns,
                sample_dt_ms=dt_ms,
            )
            LOG.debug(
                "filtered dx=%.2f dy=%.2f dt=%.3f speed=%.4f "
                "filtered_speed=%.4f precision=%.4f -> resampler logical=(%.4f,%.4f)",
                sensor_dx, sensor_dy, dt_ms, raw_speed,
                self.filtered_speed if self.filtered_speed is not None else raw_speed,
                precision_mult, logical_dx, logical_dy,
            )
            return

        self.frac_x += logical_dx
        self.frac_y += logical_dy

        out_x = int(self.frac_x)
        out_y = int(self.frac_y)
        self.frac_x -= out_x
        self.frac_y -= out_y

        if out_x == 0 and out_y == 0:
            return

        self._emit_resampled_relative(out_x, out_y)

        LOG.debug(
            "filtered dx=%.2f dy=%.2f dt=%.3f speed=%.4f "
            "filtered_speed=%.4f precision=%.4f -> rel dx=%d dy=%d "
            "rem=(%.4f,%.4f)",
            sensor_dx, sensor_dy, dt_ms, raw_speed,
            self.filtered_speed if self.filtered_speed is not None else raw_speed,
            precision_mult, out_x, out_y, self.frac_x, self.frac_y,
        )

    def _start_three_finger_gesture(self, touches: list[Touch], host_ns: int):
        self.three_start_ns = host_ns
        self.three_start_positions = {
            t.finger_id: (t.x, t.y) for t in touches[:3]
        }
        self.three_max_finger_move = 0.0
        self.three_tap_candidate = (
            self.three_finger_tap_enabled and len(touches) == 3
        )
        if self.three_tap_candidate:
            LOG.debug(
                "three-finger tap candidate started (max %.1f ms, move %.1f units)",
                self.tap_max_ms,
                self.tap_move_units,
            )

    def _update_three_finger_gesture(self, touches: list[Touch], host_ns: int):
        if not self.three_tap_candidate:
            return

        if len(touches) > 3:
            self.three_tap_candidate = False
            LOG.debug("three-finger tap candidate cancelled: fourth contact seen")
            return

        duration_ms = (host_ns - self.three_start_ns) / 1_000_000.0
        if duration_ms > self.tap_max_ms:
            self.three_tap_candidate = False
            LOG.debug(
                "three-finger tap candidate cancelled by duration: %.1f ms > %.1f ms",
                duration_ms,
                self.tap_max_ms,
            )
            return

        for t in touches:
            start = self.three_start_positions.get(t.finger_id)
            if start is None:
                self.three_tap_candidate = False
                LOG.debug(
                    "three-finger tap candidate cancelled: replacement contact id=%d",
                    t.finger_id,
                )
                return
            dist = math.hypot(t.x - start[0], t.y - start[1])
            self.three_max_finger_move = max(self.three_max_finger_move, dist)

        if self.three_max_finger_move > self.tap_move_units:
            self.three_tap_candidate = False
            LOG.debug(
                "three-finger tap candidate cancelled by movement: max=%.1f raw units threshold=%.1f",
                self.three_max_finger_move,
                self.tap_move_units,
            )

    def _finish_three_finger_gesture(self, host_ns: int):
        valid = False
        duration_ms = 0.0
        if self.three_start_ns:
            duration_ms = (host_ns - self.three_start_ns) / 1_000_000.0
        if (
            self.three_tap_candidate
            and duration_ms <= self.tap_max_ms
            and self.three_max_finger_move <= self.tap_move_units
        ):
            valid = True
            self._emit_click(ecodes.BTN_MIDDLE, "three-finger tap -> BTN_MIDDLE")

        LOG.debug(
            "external 3+ finger gesture ended%s",
            (
                f"; three-finger tap emitted (duration={duration_ms:.1f} ms "
                f"move={self.three_max_finger_move:.1f})"
                if valid
                else ""
            ),
        )
        self.three_start_ns = 0
        self.three_start_positions = {}
        self.three_max_finger_move = 0.0
        self.three_tap_candidate = False

    def emit_button_click(self, code: int, label: str):
        self._emit_click(code, label)

    def physical_click_started(self):
        """Cancel tap candidates when a real clickpad press occurs."""
        self.single_tap_candidate = False
        self.two_tap_candidate = False
        self.three_tap_candidate = False
        self._tap_drag_candidate = False
        self._cancel_tap_sequence("physical click", release_button=True)

    def emit_physical_button(self, code: int, value: int):
        """Emit a physical clickpad button through the pointer uinput."""
        if self.ui is None:
            return
        with self._emit_lock:
            self.ui.write(ecodes.EV_KEY, code, value)
            self.ui.syn()

    def process(self, frame: RawFrame, host_ns: int):
        if self.print_raw:
            LOG.debug(
                "raw ts=%d count=%d touches=%s",
                frame.timestamp,
                frame.finger_count,
                [(t.finger_id, t.x, t.y, t.pressure) for t in frame.touches],
            )

        if frame.spurious:
            return

        touches = active_touches(frame)
        count = len(touches)

        if count > 0:
            self._cancel_kinetic_scroll()

        if self.external_gesture_active:
            if count == 0:
                self._finish_three_finger_gesture(host_ns)
                self.external_gesture_active = False
                self.external_gesture_from_scroll = False
                self.multitouch_session = False
                self.two_active = False
                self.two_scrolling = False
                self.scroll_exit_pending = False
                self.single_active = False
                self._reset_pointer()
                self._scroll_velocity_history.clear()
            elif count == 2:
                # A brief accidental third-finger brush must not latch the
                # pointer backend out of scrolling until every finger lifts.
                # Cancel any possible three-finger tap and re-enter the normal
                # two-finger state with fresh anchors.  If the 3+ contact
                # interrupted an active scroll, skip the landing/tap grace so
                # the next real two-finger delta can resume immediately.
                resume_scroll = self.external_gesture_from_scroll
                # Normal fingers lift asynchronously.  Let a genuine
                # three-finger tap complete on the first 3->2 transition
                # instead of cancelling it merely because one finger left a
                # report before the others.  Sessions that interrupted an
                # already-active two-finger scroll disable the three-tap
                # candidate when 3+ contact begins, so they cannot middle-click.
                self._finish_three_finger_gesture(host_ns)
                self.external_gesture_active = False
                self.external_gesture_from_scroll = False
                self.multitouch_session = False
                self.two_active = False
                self.two_scrolling = False
                self.scroll_exit_pending = False
                self._scroll_velocity_history.clear()
                self._start_two(touches, host_ns)
                # Once a 3+ contact has existed in this contact session, never
                # reinterpret the remaining two fingers as a fresh two-finger
                # tap.  Otherwise normal asynchronous 3->2 lift-off can emit a
                # ghost BTN_RIGHT after a three-finger tap or external swipe.
                self.two_tap_candidate = False
                if resume_scroll:
                    self.two_motion_grace_until_ns = 0
                LOG.debug(
                    "3+ gesture -> two-finger %s; contacts re-anchored; "
                    "two-finger tap suppressed",
                    "scroll resume" if resume_scroll else "arbitration",
                )
            else:
                self._update_three_finger_gesture(touches, host_ns)
            return

        if count >= 3:
            # End any in-progress one/two-finger interpretation without
            # kinetic output.  Remember whether an active two-finger scroll
            # was interrupted so dropping back to exactly two contacts can
            # resume immediately instead of remaining blocked until all lift.
            interrupted_scroll = bool(
                self.multitouch_session and self.two_scrolling
            )
            if self.multitouch_session:
                self._finish_two(
                    host_ns, allow_kinetic=False, allow_tap=False
                )
                self.multitouch_session = False
            else:
                self.single_tap_candidate = False
                self.single_active = False
            self._cancel_tap_sequence("3+ finger gesture", release_button=True)
            self._reset_pointer()
            self.external_gesture_active = True
            self.external_gesture_from_scroll = interrupted_scroll
            self._start_three_finger_gesture(touches, host_ns)
            if interrupted_scroll and self.three_tap_candidate:
                self.three_tap_candidate = False
                LOG.debug(
                    "three-finger tap candidate suppressed: 3+ contact "
                    "interrupted an active two-finger scroll"
                )
            LOG.debug(
                "3+ contacts detected; suppressing pointer/scroll "
                "(interrupted_scroll=%s)",
                interrupted_scroll,
            )
            return

        if count >= 2:
            self._tap_drag_candidate = False

            if self.multitouch_session and self.scroll_exit_pending:
                # The second finger came back before the exit grace expired:
                # this is still the same scroll stroke. Re-anchor the current
                # centroid and continue without generating a wheel jump.
                self._resume_two_after_exit_gap(touches)

            if not self.multitouch_session:
                # A second finger means the original one-finger contact was
                # the beginning of a multi-touch gesture, not a completed tap.
                self.single_tap_candidate = False
                self.single_active = False
                self.single_motion_grace_until_ns = 0
                self._reset_pointer()

            self._process_two(touches, host_ns)
            return

        if count == 1:
            touch = touches[0]

            if self.single_active and not self.multitouch_session:
                self.single_last_hid_ts = frame.timestamp

            if self.multitouch_session:
                if self.two_scrolling:
                    # Do not immediately hand a lone remaining finger to the
                    # pointer. Fingers commonly leave a touchpad a few reports
                    # apart, and that last finger may still be moving quickly
                    # as the hand lifts. Follow it as an anchor, but suppress
                    # REL motion until the short exit grace expires.
                    if not self.scroll_exit_pending:
                        self.scroll_exit_pending = True
                        self.scroll_exit_until_ns = (
                            host_ns
                            + int(self.multitouch_exit_grace_ms * 1_000_000.0)
                        )
                        LOG.debug(
                            "two-finger scroll entered one-finger exit grace (%.1f ms)",
                            self.multitouch_exit_grace_ms,
                        )

                    self._reset_pointer(
                        touch,
                        frame.timestamp,
                        reset_fraction=True,
                    )

                    if host_ns < self.scroll_exit_until_ns:
                        return

                    # One finger has genuinely remained down past the grace
                    # period. End the scroll and hand off at the *current*
                    # coordinate. This frame emits no pointer motion; the next
                    # frame supplies the first relative delta.
                    self._finish_two(host_ns, allow_kinetic=False)
                    self.multitouch_session = False

                    self.single_active = True
                    self.single_start_ns = host_ns
                    self.single_start_hid_ts = frame.timestamp
                    self.single_last_hid_ts = frame.timestamp
                    self.single_start_x = touch.x
                    self.single_start_y = touch.y
                    self.single_max_move = 0.0
                    self.single_tap_candidate = False
                    self.single_motion_grace_until_ns = 0

                    self._reset_pointer(
                        touch,
                        frame.timestamp,
                        reset_fraction=True,
                    )
                    LOG.debug(
                        "two-finger scroll -> one-finger pointer handoff "
                        "after %.1f ms exit grace (finger id=%d x=%d y=%d)",
                        self.multitouch_exit_grace_ms,
                        touch.finger_id,
                        touch.x,
                        touch.y,
                    )
                    return

                # Non-scrolling two-finger contact dropping to one finger:
                # use the same exit grace as scrolling. This preserves a
                # normal two-finger tap when the fingers lift a few reports
                # apart, but if one finger genuinely remains down, hand it
                # cleanly back to pointer motion.
                if not self.scroll_exit_pending:
                    self.scroll_exit_pending = True
                    self.scroll_exit_until_ns = (
                        host_ns
                        + int(self.multitouch_exit_grace_ms * 1_000_000.0)
                    )
                    LOG.debug(
                        "two-finger contact entered one-finger exit grace (%.1f ms)",
                        self.multitouch_exit_grace_ms,
                    )

                self._reset_pointer(
                    touch,
                    frame.timestamp,
                    reset_fraction=True,
                )

                if host_ns < self.scroll_exit_until_ns:
                    return

                # One finger genuinely remained after the grace period, so
                # this is no longer a two-finger tap. Cancel that candidate
                # before finishing the two-finger state.
                self.two_tap_candidate = False
                self._finish_two(host_ns, allow_kinetic=False)
                self.multitouch_session = False

                self.single_active = True
                self.single_start_ns = host_ns
                self.single_start_hid_ts = frame.timestamp
                self.single_last_hid_ts = frame.timestamp
                self.single_start_x = touch.x
                self.single_start_y = touch.y
                self.single_max_move = 0.0
                self.single_tap_candidate = False
                self.single_motion_grace_until_ns = 0

                self._reset_pointer(
                    touch,
                    frame.timestamp,
                    reset_fraction=True,
                )

                LOG.debug(
                    "two-finger contact -> one-finger pointer handoff "
                    "after %.1f ms exit grace (finger id=%d x=%d y=%d)",
                    self.multitouch_exit_grace_ms,
                    touch.finger_id,
                    touch.x,
                    touch.y,
                )
                return

            if not self.single_active:
                self._start_single(touch, host_ns, frame.timestamp)
                return

            if self._tap_drag_candidate:
                # The second contact after a tap is unresolved. BTN_LEFT is
                # already held from the first tap. Keep the pointer anchored
                # until it either:
                #   * releases inside the slop -> complete first click + second click
                #   * crosses the slop         -> continue held-button drag
                #
                # No second BTN_LEFT press occurs on finger-down or drag start,
                # so a desktop cannot fire its double-click action before drag.
                drag_dist = math.hypot(
                    touch.x - self.single_start_x,
                    touch.y - self.single_start_y,
                )
                self.single_max_move = max(self.single_max_move, drag_dist)
                held_ms = (host_ns - self.single_start_ns) / 1_000_000.0
                distance_ready = drag_dist > self.tap_drag_activation_units
                hold_ready = (
                    self.tap_drag_hold_ms > 0
                    and held_ms >= self.tap_drag_hold_ms
                )
                if not distance_ready and not hold_ready:
                    self._reset_pointer(
                        touch,
                        frame.timestamp,
                        reset_fraction=True,
                    )
                    return

                if self._start_tap_drag():
                    self._reset_pointer(
                        touch,
                        frame.timestamp,
                        reset_fraction=True,
                    )
                    if distance_ready:
                        LOG.debug(
                            "tap-and-drag motion armed after %.1f raw units "
                            "(threshold=%.1f margin=+%.1f held=%.1f ms); "
                            "BTN_LEFT remained held",
                            drag_dist,
                            self.tap_drag_activation_units,
                            drag_dist - self.tap_drag_activation_units,
                            held_ms,
                        )
                    else:
                        LOG.debug(
                            "tap-and-drag hold armed after %.1f ms "
                            "(threshold=%.1f ms movement=%.1f raw units); "
                            "BTN_LEFT remained held",
                            held_ms,
                            self.tap_drag_hold_ms,
                            drag_dist,
                        )
                    return

            # Fresh one-finger contacts are ambiguous for a few reports: this
            # may be the first half of an intended two-finger gesture. During
            # the entry grace we continuously re-anchor instead of accumulating
            # motion.  Tap classification uses a separate, larger landing slop
            # here because the pointer itself is not exposing this fingertip-
            # center settling to the desktop.
            if self.single_motion_grace_until_ns:
                self._update_single_landing_tap(touch)
                if host_ns < self.single_motion_grace_until_ns:
                    self._reset_pointer(
                        touch,
                        frame.timestamp,
                        reset_fraction=True,
                    )
                    return

                # Landing grace is over.  Re-center the normal tap anchor at
                # the same coordinate used to arm visible pointer motion, then
                # apply the original, tighter tap_move_units threshold from the
                # next report onward.  Duration still starts at first contact.
                self.single_motion_grace_until_ns = 0
                self.single_start_x = touch.x
                self.single_start_y = touch.y
                self.single_max_move = 0.0
                self._reset_pointer(
                    touch,
                    frame.timestamp,
                    reset_fraction=True,
                )
                LOG.debug(
                    "one-finger entry grace expired; pointer motion armed; "
                    "tap anchor re-centered (landing max=%.1f quick-release threshold=%.1f)",
                    self.single_landing_max_move,
                    self.tap_landing_move_units,
                )
                return

            self._update_single_tap(touch)
            self._process_pointer_motion(touch, frame, host_ns)
            return

        # Zero active fingers: finish whichever gesture was in progress.
        was_multitouch_session = self.multitouch_session
        if was_multitouch_session:
            self._finish_two(host_ns)
            self.multitouch_session = False
        else:
            self._finish_single(host_ns, frame.timestamp)

        self.single_active = False
        self.single_motion_grace_until_ns = 0
        self.scroll_exit_pending = False
        self.scroll_exit_until_ns = 0
        if (
            not was_multitouch_session
            and self.resampler.enabled
            and self.resampler.predictor == "interpolated"
        ):
            # A clean one-finger lift may drain only already-measured delayed
            # motion. Gesture/mode transitions still use reset() immediately.
            self.resampler.finish("clean one-finger lift")
            self._reset_pointer(reset_fraction=False)
        else:
            self._reset_pointer()

    def close(self):
        self.reset_runtime()
        self.resampler.close()



class PointerBackend:
    mode = "pointer"

    def __init__(self, args):
        self.args = args
        self.ui = None
        self.engine = None
        self.clicks = None
        self._button_contacts = {}
        self._button_contact_lock = threading.RLock()
        # A light touch in a synthetic button zone remains a quick click when
        # released promptly, but becomes a real held button after this short
        # delay (or immediately when another pointer finger appears). This
        # avoids relying on an immediate button-down at the instant the thumb
        # first lands while preserving click-hold-drag semantics.
        self._touch_button_hold_ms = 120.0
        # Light button-area contacts are true held buttons when raw physical
        # clicks are disabled. Reference counts make overlapping contacts on
        # the same logical button safe.
        self._touch_button_counts = {
            ecodes.BTN_LEFT: 0,
            ecodes.BTN_MIDDLE: 0,
            ecodes.BTN_RIGHT: 0,
        }
        self._previous_touch_ids = set()
        self.liftoff_filter = LiftOffJitterFilter(
            enabled=self.args.liftoff_jitter_suppression,
            pressure_threshold=self.args.liftoff_pressure_threshold,
            fall_frames=self.args.liftoff_fall_frames,
            pressure_drop=self.args.liftoff_pressure_drop,
            recovery_pressure=self.args.liftoff_recovery_pressure,
            recovery_timeout_ms=self.args.liftoff_recovery_timeout_ms,
            motion_window_ms=self.args.liftoff_motion_window_ms,
            max_path_speed=self.args.liftoff_max_path_speed,
        )
        self._logged_config = False

    def configure(self, source, info: SourceInfo):
        invert_x = self.args.invert_x
        invert_y = self.args.invert_y
        if self.ui is None and not self.args.no_uinput:
            self.ui = make_pointer_uinput(self.args.pointer_name)
            path = self.ui.device.path if self.ui.device is not None else self.ui.devnode
            LOG.info("Created persistent virtual pointer: %s (%s)", self.args.pointer_name, path)
            time.sleep(0.25)

        timestamp_unit_ms = 1.0

        gain_x = self.args.pointer_gain if self.args.pointer_gain_x is None else self.args.pointer_gain_x
        gain_y = self.args.pointer_gain if self.args.pointer_gain_y is None else self.args.pointer_gain_y

        if self.engine is not None:
            self.engine.close()

        self.engine = RawPointerEngine(
            gain_x=gain_x, gain_y=gain_y,
            invert_x=invert_x, invert_y=invert_y,
            max_raw_jump=self.args.pointer_max_raw_jump,
            pointer_hysteresis_units=self.args.pointer_hysteresis_units,
            scroll_hysteresis_units=self.args.pointer_scroll_hysteresis_units,
            ui=self.ui, print_raw=self.args.print_raw,
            timestamp_unit_ms=timestamp_unit_ms,
            precision_gain=self.args.pointer_precision_gain,
            precision_speed=self.args.pointer_precision_speed,
            precision_transition=self.args.pointer_precision_transition,
            precision_filter_ms=self.args.pointer_precision_filter_ms,
            tap_enabled=self.args.pointer_tap,
            two_finger_tap_enabled=self.args.pointer_two_finger_tap,
            three_finger_tap_enabled=self.args.pointer_three_finger_tap,
            tap_max_ms=self.args.pointer_tap_max_ms,
            two_finger_tap_grace_ms=self.args.pointer_two_finger_tap_grace_ms,
            tap_move_units=self.args.pointer_tap_move_units,
            tap_landing_move_units=self.args.pointer_tap_landing_move_units,
            scroll_enabled=self.args.pointer_scroll,
            scroll_start_units=self.args.pointer_scroll_start_units,
            scroll_units_per_detent=self.args.pointer_scroll_units_per_detent,
            scroll_invert=self.args.pointer_scroll_invert,
            horizontal_scroll=self.args.pointer_horizontal_scroll,
            hscroll_units_per_detent=self.args.pointer_hscroll_units_per_detent,
            hscroll_invert=self.args.pointer_hscroll_invert,
            multitouch_entry_grace_ms=self.args.pointer_multitouch_entry_grace_ms,
            multitouch_exit_grace_ms=self.args.pointer_multitouch_exit_grace_ms,
            tap_drag_window_ms=self.args.pointer_tap_drag_window_ms,
            tap_drag_activation_units=self.args.pointer_tap_drag_activation_units,
            scroll_axis_lock=self.args.pointer_scroll_axis_lock == "on",
            scroll_axis_lock_ratio=self.args.pointer_scroll_axis_lock_ratio,
            kinetic_scroll=self.args.pointer_kinetic_scroll,
            kinetic_history_ms=self.args.pointer_kinetic_history_ms,
            kinetic_start_velocity=self.args.pointer_kinetic_start_velocity,
            kinetic_stop_velocity=self.args.pointer_kinetic_stop_velocity,
            kinetic_decay_ms=self.args.pointer_kinetic_decay_ms,
            kinetic_max_ms=self.args.pointer_kinetic_max_ms,
            tap_drag_hold_ms=self.args.pointer_tap_drag_hold_ms,
            resampling_rate_hz=self.args.resampling_rate_hz,
            resampling_predictor=self.args.resampling_predictor,
        )

        self.clicks = PhysicalClickPolicy(
            enabled=self.args.physical_click,
            button_areas=self.args.button_areas,
            area_height_mm=self.args.button_area_height_mm,
            info=info,
            release_pressure=self.args.physical_click_release_pressure,
            release_debounce_frames=self.args.physical_click_release_debounce_frames,
        )

        if not self._logged_config:
            LOG.info(
                "Pointer mode: normal gain X=%.3f Y=%.3f; low-speed effective gain X=%.3f Y=%.3f <= %.3f -> normal at %.3f units/ms; pointer hysteresis %.2f units; per-finger scroll hysteresis %.2f units; extra scroll-start %.2f units",
                gain_x, gain_y,
                gain_x * self.args.pointer_precision_gain,
                gain_y * self.args.pointer_precision_gain,
                self.args.pointer_precision_speed, self.args.pointer_precision_transition,
                self.args.pointer_hysteresis_units,
                self.args.pointer_scroll_hysteresis_units,
                self.args.pointer_scroll_start_units,
            )
            if self.args.resampling_rate_hz > 0:
                LOG.info(
                    "Pointer motion resampling: %.3f Hz independent output clock predictor=%s (pointer mode only)",
                    self.args.resampling_rate_hz,
                    self.args.resampling_predictor,
                )
            else:
                LOG.info("Pointer motion resampling: disabled (hardware-driven REL output)")
            LOG.info(
                "Pointer tap press is immediate; release is held up to %.0f ms for tap-and-drag; retouch release becomes second click; drag arms at %.1f raw units or %.0f ms hold",
                self.args.pointer_tap_drag_window_ms,
                self.args.pointer_tap_drag_activation_units,
                self.args.pointer_tap_drag_hold_ms,
            )
            self._logged_config = True

    def _activate_touch_button(self, fid: int, reason: str):
        if self.args.physical_click or self.engine is None:
            return
        with self._button_contact_lock:
            state = self._button_contacts.get(fid)
            if state is None or state.get("synthetic_down"):
                return
            timer = state.get("hold_timer")
            if timer is not None:
                timer.cancel()
                state["hold_timer"] = None
            button = state["button"]
            count = self._touch_button_counts.get(button, 0)
            self._touch_button_counts[button] = count + 1
            if count == 0:
                self.engine.emit_physical_button(button, 1)
            state["synthetic_down"] = True
            LOG.debug(
                "button-area hold activated id=%d button=%d (%s)",
                fid, button, reason,
            )

    def _touch_button_hold_timeout(self, fid: int):
        self._activate_touch_button(fid, "hold timeout")

    def _classify_button_contacts(self, frame: RawFrame, host_ns: int):
        """Keep contacts that *start* in the button strip isolated.

        Classification is sticky for the lifetime of a tracking id. A pointer
        finger that later wanders into the bottom strip remains a pointer; a
        thumb that started in the strip remains a button contact even if it
        slips slightly while pressing.
        """
        current = {t.finger_id: t for t in frame.touches}
        current_ids = set(current)

        if self.args.button_areas and self.clicks is not None:
            for fid in current_ids - self._previous_touch_ids:
                touch = current[fid]
                if self.clicks.touch_in_button_area(touch):
                    button = self.clicks.button_for_touch(touch)
                    self._button_contacts[fid] = {
                        "start_ns": host_ns,
                        "start_x": touch.x,
                        "start_y": touch.y,
                        "max_move": 0.0,
                        "button": button,
                        "physical_seen": False,
                        "synthetic_down": False,
                        "hold_timer": None,
                    }

                    # With raw force-clicks disabled, a quick touch/release
                    # is emitted as a normal click. A hold is promoted to a
                    # real button-down after a short delay, and is promoted
                    # immediately if another active-area finger appears so
                    # click-hold-drag begins before pointer motion.
                    if not self.args.physical_click and self.engine is not None:
                        timer = threading.Timer(
                            self._touch_button_hold_ms / 1000.0,
                            self._touch_button_hold_timeout,
                            args=(fid,),
                        )
                        timer.daemon = True
                        self._button_contacts[fid]["hold_timer"] = timer
                        timer.start()

                    LOG.debug(
                        "button-area contact started id=%d x=%d y=%d button=%d synthetic_down=%s",
                        fid, touch.x, touch.y, button,
                        self._button_contacts[fid]["synthetic_down"],
                    )

            for fid, state in list(self._button_contacts.items()):
                touch = current.get(fid)
                if touch is None:
                    continue
                dist = math.hypot(
                    touch.x - state["start_x"],
                    touch.y - state["start_y"],
                )
                state["max_move"] = max(state["max_move"], dist)

        removed = [
            (fid, state) for fid, state in self._button_contacts.items()
            if fid not in current_ids
        ]
        reserved_ids = set(self._button_contacts)

        # Button-area classification is authoritative for the lifetime of a
        # tracking id. A thumb that starts in the strip is a button contact,
        # not a pointer/scroll/gesture finger -- even when two or more active-
        # area fingers are present. This lets a resting thumb act as an
        # orienting pivot without turning a two-finger scroll into a 3-finger
        # gesture or cancelling an in-progress scroll. A genuine 3+ finger
        # gesture still works when those contacts all start in the active area.
        if not self.args.button_areas:
            engine_frame = frame
        else:
            engine_frame = RawFrame(
                timestamp=frame.timestamp,
                touches=tuple(
                    t for t in frame.touches
                    if t.finger_id not in reserved_ids
                ),
                spurious=frame.spurious,
                end_of_frame=frame.end_of_frame,
            )

        return engine_frame, removed, reserved_ids, current_ids

    def _finish_button_contacts(self, removed, host_ns: int):
        for fid, state in removed:
            with self._button_contact_lock:
                timer = state.get("hold_timer")
                if timer is not None:
                    timer.cancel()
                    state["hold_timer"] = None

                if state.get("synthetic_down") and self.engine is not None:
                    button = state["button"]
                    count = max(0, self._touch_button_counts.get(button, 0) - 1)
                    self._touch_button_counts[button] = count
                    if count == 0:
                        self.engine.emit_physical_button(button, 0)
                    LOG.debug(
                        "button-area held touch released id=%d button=%d",
                        fid, button,
                    )
                elif (
                    not self.args.physical_click
                    and self.engine is not None
                    and state.get("max_move", 0.0) <= self.args.pointer_tap_move_units
                ):
                    button = state["button"]
                    self.engine.emit_button_click(
                        button,
                        f"button-area quick tap id={fid} -> button {button}",
                    )

                self._button_contacts.pop(fid, None)

    def process_report(self, frame: RawFrame, events, host_ns: int):
        if self.engine is None:
            return

        engine_frame, removed, reserved_ids, current_ids = (
            self._classify_button_contacts(frame, host_ns)
        )

        # A pending synthetic button should become a held button before an
        # active-area finger is allowed to move the pointer. This guarantees
        # click-hold-drag ordering: button-down first, REL motion second.
        if (
            not self.args.physical_click
            and engine_frame.touches
            and self._button_contacts
        ):
            for fid in list(reserved_ids & current_ids):
                self._activate_touch_button(fid, "active pointer finger joined")

        # Process the raw physical button before pointer/tap state for this
        # frame. That prevents a force-click frame from first being interpreted
        # as an ordinary tap frame.
        if self.clicks is not None:
            for ev in events:
                if ev.type != ecodes.EV_KEY or ev.code != ecodes.BTN_LEFT:
                    continue
                if ev.value:
                    self.engine.physical_click_started()
                    for fid in reserved_ids & current_ids:
                        state = self._button_contacts.get(fid)
                        if state is not None:
                            state["physical_seen"] = True
                    allowed_ids = reserved_ids if self.args.button_areas else None
                    code = self.clicks.press(frame, allowed_ids=allowed_ids)
                    if code is not None:
                        self.engine.emit_physical_button(code, 1)
                else:
                    code = self.clicks.release()
                    if code is not None:
                        self.engine.emit_physical_button(code, 0)

            # In custom button-area mode, also let the latched button-zone
            # contact's pressure terminate the click. This makes button-up
            # follow thumb force release even while another finger remains on
            # the surface moving the pointer. Raw BTN_LEFT release remains the
            # normal/first path when the hardware reports it promptly.
            code = self.clicks.maybe_release_from_frame(frame)
            if code is not None:
                self.engine.emit_physical_button(code, 0)

        motion_frame = self.liftoff_filter.filter_frame(engine_frame)
        self.engine.process(motion_frame, host_ns)
        self._finish_button_contacts(removed, host_ns)
        self._previous_touch_ids = current_ids

    def reset_runtime(self):
        with self._button_contact_lock:
            for state in self._button_contacts.values():
                timer = state.get("hold_timer")
                if timer is not None:
                    timer.cancel()
            if self.engine is not None:
                for button, count in self._touch_button_counts.items():
                    if count > 0:
                        self.engine.emit_physical_button(button, 0)
            for button in self._touch_button_counts:
                self._touch_button_counts[button] = 0
            self._button_contacts.clear()
        self._previous_touch_ids.clear()
        self.liftoff_filter.reset()
        if self.clicks is not None:
            code = self.clicks.release()
            if code is not None and self.engine is not None:
                self.engine.emit_physical_button(code, 0)
        if self.engine is not None:
            self.engine.reset_runtime()

    def close(self):
        self.reset_runtime()
        if self.engine is not None:
            self.engine.close()
            self.engine = None
        if self.ui is not None:
            self.ui.close()
            self.ui = None
