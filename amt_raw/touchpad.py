from __future__ import annotations

import logging
import threading
import time
from typing import Optional

from evdev import UInput, ecodes

from .clicks import PhysicalClickPolicy
from .frames import RawFrame, Touch, active_touches
from .pointer import RawPointerEngine
from .source import SourceInfo
from .liftoff import LiftOffJitterFilter

LOG = logging.getLogger("amt-raw")

SCROLL_DEVICE_NAME = "AMT Raw Touchpad Scroll"
KINETIC_DEVICE_NAME = "AMT Raw Touchpad Kinetic"


def make_touchpad_scroll_uinput() -> UInput:
    """Relative companion used for touchpad-mode live scrolling.

    KWin disables libinput's native two-finger/edge scroll methods on the
    virtual AMT touchpad.  The touchpad itself therefore remains a genuine
    ABS_MT device for one-finger motion, taps, click handling and gestures,
    while this companion carries the same high-resolution live-scroll stream
    used by pointer mode. Post-release coast uses the separate Kinetic device.
    """
    capabilities = {
        ecodes.EV_REL: [
            ecodes.REL_X,
            ecodes.REL_Y,
            ecodes.REL_WHEEL,
            ecodes.REL_WHEEL_HI_RES,
            ecodes.REL_HWHEEL,
            ecodes.REL_HWHEEL_HI_RES,
        ],
        ecodes.EV_KEY: [ecodes.BTN_LEFT],
    }
    return UInput(
        capabilities,
        name=SCROLL_DEVICE_NAME,
        vendor=0x05AC,
        product=0xA326,
        version=1,
        bustype=ecodes.BUS_USB,
        phys="amt-raw/touchpad-scroll",
    )


def make_touchpad_kinetic_uinput() -> UInput:
    """Coast-only relative companion for post-release kinetic scrolling."""
    capabilities = {
        ecodes.EV_REL: [
            ecodes.REL_X,
            ecodes.REL_Y,
            ecodes.REL_WHEEL,
            ecodes.REL_WHEEL_HI_RES,
            ecodes.REL_HWHEEL,
            ecodes.REL_HWHEEL_HI_RES,
        ],
        ecodes.EV_KEY: [ecodes.BTN_LEFT],
    }
    return UInput(
        capabilities,
        name=KINETIC_DEVICE_NAME,
        vendor=0x05AC,
        product=0xA326,
        version=1,
        bustype=ecodes.BUS_USB,
        phys="amt-raw/touchpad-kinetic",
    )


class TouchpadScrollEngine(RawPointerEngine):
    """Use the pointer backend's proven two-finger scroll state machine only.

    The virtual touchpad continues to own taps and pointer motion through
    libinput.  This adapter deliberately invokes only RawPointerEngine's
    two-finger scroll/tap-arbitration path and suppresses its synthetic tap
    clicks. Live scroll keeps the pointer engine's hysteresis/arbitration and
    post-release kinetic uses the same decay math, with coast routed through a
    separate companion device so live and post-release scroll remain distinct.
    """

    def _emit_click(self, code: int, label: str):
        # libinput on AMT Raw Touchpad still owns tap-to-click.  We retain the
        # pointer engine's two-finger tap state solely to arbitrate tap vs scroll
        # and to suppress kinetic output for a tap.
        LOG.debug("touchpad scroll engine suppressed synthetic tap click: %s", label)

    def process_scroll_frame(self, frame: RawFrame, host_ns: int):
        touches = active_touches(frame)
        count = len(touches)

        if count > 0:
            self._cancel_kinetic_scroll()

        # Match pointer mode's 3+ finger policy: once a larger gesture appears,
        # end scroll interpretation without kinetic output and remain quiet
        # until all contacts lift.  The real/virtual touchpad stream remains
        # available to libinput and external gesture software.
        blocked = getattr(self, "_touchpad_gesture_blocked", False)
        if blocked:
            if count == 0:
                self._touchpad_gesture_blocked = False
                self._touchpad_gesture_from_scroll = False
                self.multitouch_session = False
                self.two_active = False
                self.two_scrolling = False
                self.scroll_exit_pending = False
                self._scroll_velocity_history.clear()
            elif count == 2:
                resume_scroll = getattr(self, "_touchpad_gesture_from_scroll", False)
                self._touchpad_gesture_blocked = False
                self._touchpad_gesture_from_scroll = False
                self.multitouch_session = False
                self.two_active = False
                self.two_scrolling = False
                self.scroll_exit_pending = False
                self._scroll_velocity_history.clear()
                self._start_two(touches, host_ns)
                if resume_scroll:
                    self.two_tap_candidate = False
                    self.two_motion_grace_until_ns = 0
                LOG.debug(
                    "touchpad 3+ gesture -> two-finger %s; contacts re-anchored",
                    "scroll resume" if resume_scroll else "arbitration",
                )
            return

        if count >= 3:
            interrupted_scroll = bool(
                self.multitouch_session and self.two_scrolling
            )
            if self.multitouch_session:
                self.two_tap_candidate = False
                self._finish_two(host_ns, allow_kinetic=False)
                self.multitouch_session = False
            self._touchpad_gesture_blocked = True
            self._touchpad_gesture_from_scroll = interrupted_scroll
            return

        if count >= 2:
            if self.multitouch_session and self.scroll_exit_pending:
                self._resume_two_after_exit_gap(touches)
            self._process_two(touches, host_ns)
            return

        if count == 1:
            if not self.multitouch_session:
                return

            # Preserve pointer mode's asynchronous 2->1 release grace.  A
            # second finger returning inside this window resumes the same
            # scroll; one finger remaining past it ends the scroll without
            # kinetic coast.
            if not self.scroll_exit_pending:
                self.scroll_exit_pending = True
                self.scroll_exit_until_ns = (
                    host_ns + int(self.multitouch_exit_grace_ms * 1_000_000.0)
                )
            if host_ns < self.scroll_exit_until_ns:
                return

            self._finish_two(host_ns, allow_kinetic=False)
            self.multitouch_session = False
            return

        # Both fingers are gone.  Finish the exact pointer-mode scroll state,
        # including release-velocity estimation and optional kinetic coast.
        if self.multitouch_session:
            self._finish_two(host_ns, allow_kinetic=True)
            self.multitouch_session = False


class TouchpadBackend:
    mode = "touchpad"

    def __init__(self, args):
        self.args = args
        self.ui: Optional[UInput] = None
        self.scroll_ui: Optional[UInput] = None
        self.kinetic_ui: Optional[UInput] = None
        self.scroll_engine: Optional[TouchpadScrollEngine] = None
        self.clicks: Optional[PhysicalClickPolicy] = None
        self.info: Optional[SourceInfo] = None

        # Virtual Type-B contact state.  We synthesize the MT stream from the
        # assembled RawFrame rather than forwarding the physical event packet
        # verbatim.  That lets custom bottom-button contacts be excluded from
        # libinput completely while an active-area finger continues to point.
        self._finger_to_slot: dict[int, int] = {}
        self._accepted: dict[int, list[Optional[float]]] = {}
        self._button_contacts: set[int] = set()
        self._previous_physical_ids: set[int] = set()
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
        self._emit_lock = threading.RLock()

    @staticmethod
    def _hysteresis_axis(raw: float, accepted: Optional[float], margin: float) -> float:
        if accepted is None or margin <= 0.0:
            return raw
        if raw > accepted + margin:
            return raw - margin
        if raw < accepted - margin:
            return raw + margin
        return accepted

    def _make_scroll_engine(self) -> TouchpadScrollEngine:
        assert self.scroll_ui is not None
        return TouchpadScrollEngine(
            gain_x=1.0,
            gain_y=1.0,
            invert_x=False,
            invert_y=False,
            max_raw_jump=self.args.pointer_max_raw_jump,
            pointer_hysteresis_units=self.args.pointer_hysteresis_units,
            scroll_hysteresis_units=self.args.pointer_scroll_hysteresis_units,
            ui=self.scroll_ui,
            print_raw=False,
            timestamp_unit_ms=1.0,
            precision_gain=1.0,
            precision_speed=1.0,
            precision_transition=1.0,
            precision_filter_ms=0.0,
            tap_enabled=False,
            # Keep the exact pointer-mode landing/tap arbitration, but the
            # overridden _emit_click leaves actual touchpad taps to libinput.
            two_finger_tap_enabled=self.args.pointer_two_finger_tap,
            # This adapter owns only two-finger scroll arbitration. Three or
            # more contacts are handled by the real touchpad/gesture path and
            # are blocked in process_scroll_frame(), so do not enable the
            # pointer engine's synthetic three-finger tap recognizer here.
            three_finger_tap_enabled=False,
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
            kinetic_scroll=self.args.touchpad_kinetic_scroll,
            kinetic_history_ms=self.args.kinetic_history_ms,
            kinetic_start_velocity=self.args.kinetic_start_velocity,
            kinetic_stop_velocity=self.args.kinetic_stop_velocity,
            kinetic_decay_ms=self.args.kinetic_decay_ms,
            kinetic_max_ms=self.args.kinetic_max_ms,
            kinetic_ui=self.kinetic_ui,
        )

    def configure(self, source, info: SourceInfo):
        self.info = info
        if self.ui is None:
            caps = source.capabilities(absinfo=True)
            caps.pop(ecodes.EV_SYN, None)
            caps.pop(ecodes.EV_FF, None)

            keys = list(caps.get(ecodes.EV_KEY, []))
            for code in (ecodes.BTN_LEFT, ecodes.BTN_MIDDLE, ecodes.BTN_RIGHT):
                if code not in keys:
                    keys.append(code)
            caps[ecodes.EV_KEY] = keys
            caps.pop(ecodes.EV_REL, None)

            input_props = list(source.input_props())
            effective_button_areas = self.args.button_areas and self.args.physical_click
            if self.args.button_areas and not self.args.physical_click:
                LOG.info(
                    "AMT_BUTTON_AREAS ignored in touchpad mode because "
                    "AMT_PHYSICAL_CLICK is off; tap-zone remapping is not enabled"
                )
            if effective_button_areas:
                input_props = [
                    prop for prop in input_props
                    if prop != ecodes.INPUT_PROP_BUTTONPAD
                ]
                LOG.info(
                    "Custom button areas enabled; bottom contacts are excluded "
                    "from virtual MT motion and exposed as explicit L/M/R buttons"
                )

            # Preserve the real USB-C Magic Trackpad identity so libinput keeps
            # the same Apple model/pressure/touch-size quirks for pointer, tap,
            # palm/thumb and gesture handling.  Source autodetection rejects
            # AMT Raw device names, preventing a virtual-device feedback loop.
            self.ui = UInput(
                caps,
                name=self.args.touchpad_name,
                vendor=info.vendor,
                product=info.product,
                version=info.version,
                bustype=source.info.bustype,
                phys="amt-raw/touchpad",
                input_props=input_props,
            )
            path = self.ui.device.path if self.ui.device is not None else self.ui.devnode
            LOG.info(
                "Created virtual touchpad: %s (%s) identity=%04x:%04x",
                self.args.touchpad_name,
                path,
                info.vendor,
                info.product,
            )
            LOG.info(
                "Touchpad coordinate hysteresis %.2f raw units",
                self.args.touchpad_hysteresis_units,
            )
            time.sleep(0.25)

        # The companion is now required for live touchpad-mode scrolling even
        # when kinetic coast itself is disabled.
        if self.scroll_ui is None:
            self.scroll_ui = make_touchpad_scroll_uinput()
            path = (
                self.scroll_ui.device.path
                if self.scroll_ui.device is not None
                else self.scroll_ui.devnode
            )
            LOG.info("Created touchpad scroll companion: %s (%s)", SCROLL_DEVICE_NAME, path)
            time.sleep(0.1)

        if self.args.touchpad_kinetic_scroll and self.kinetic_ui is None:
            self.kinetic_ui = make_touchpad_kinetic_uinput()
            path = (
                self.kinetic_ui.device.path
                if self.kinetic_ui.device is not None
                else self.kinetic_ui.devnode
            )
            LOG.info(
                "Created touchpad kinetic companion: %s (%s)",
                KINETIC_DEVICE_NAME,
                path,
            )
            time.sleep(0.1)

        self.scroll_engine = self._make_scroll_engine()
        LOG.info(
            "Touchpad live scroll uses pointer engine: hysteresis=%.2f start=%.2f "
            "units/detent=%.2f kinetic=%s",
            self.args.pointer_scroll_hysteresis_units,
            self.args.pointer_scroll_start_units,
            self.args.pointer_scroll_units_per_detent,
            "on" if self.args.touchpad_kinetic_scroll else "off",
        )

        self.clicks = PhysicalClickPolicy(
            enabled=self.args.physical_click,
            button_areas=(self.args.button_areas and self.args.physical_click),
            area_height_mm=self.args.button_area_height_mm,
            info=info,
            release_pressure=self.args.physical_click_release_pressure,
            release_debounce_frames=self.args.physical_click_release_debounce_frames,
        )

    def _classify_button_contacts(self, frame: RawFrame) -> set[int]:
        current_ids = {t.finger_id for t in frame.touches}
        effective = (
            self.args.button_areas
            and self.args.physical_click
            and self.clicks is not None
        )
        if effective:
            for t in frame.touches:
                if t.finger_id not in self._previous_physical_ids:
                    if self.clicks.touch_in_button_area(t):
                        self._button_contacts.add(t.finger_id)
                        LOG.debug(
                            "touchpad button-area contact reserved id=%d x=%d y=%d",
                            t.finger_id,
                            t.x,
                            t.y,
                        )
        else:
            self._button_contacts.clear()

        self._button_contacts.intersection_update(current_ids)
        self._previous_physical_ids = current_ids
        return set(self._button_contacts)

    def _virtual_touches(self, frame: RawFrame, reserved: set[int]) -> list[Touch]:
        # Match pointer-mode button isolation: a contact that started in the
        # custom button strip remains a button for its whole tracking-id
        # lifetime. Do not put a resting thumb back into libinput merely
        # because 3+ physical contacts are present; otherwise two active-area
        # scrolling fingers plus the thumb become a competing 3-finger gesture.
        raw = [t for t in frame.touches if t.finger_id not in reserved]

        active_ids = {t.finger_id for t in raw}
        for fid in tuple(self._accepted):
            if fid not in active_ids:
                self._accepted.pop(fid, None)

        out = []
        for t in raw:
            accepted = self._accepted.setdefault(t.finger_id, [None, None])
            fx = self._hysteresis_axis(
                float(t.x), accepted[0], self.args.touchpad_hysteresis_units
            )
            fy = self._hysteresis_axis(
                float(t.y), accepted[1], self.args.touchpad_hysteresis_units
            )
            accepted[0] = fx
            accepted[1] = fy
            out.append(
                Touch(
                    finger_id=t.finger_id,
                    x=int(round(fx)),
                    y=int(round(fy)),
                    pressure=t.pressure,
                    major=t.major,
                    minor=t.minor,
                    orientation=t.orientation,
                    contact_status=t.contact_status,
                )
            )
        return out

    def _scroll_touches(self, frame: RawFrame, reserved: set[int]) -> list[Touch]:
        # Button-area contacts are semantically buttons, not gesture fingers.
        # Unlike the virtual touchpad presentation, there is no reason to put a
        # reserved thumb back into the scroll engine merely because 3+ physical
        # contacts happen to exist.
        return [t for t in frame.touches if t.finger_id not in reserved]

    def _allocate_slot(self, fid: int) -> int:
        if fid in self._finger_to_slot:
            return self._finger_to_slot[fid]
        max_slots = self.info.max_slots if self.info is not None else 16
        used = set(self._finger_to_slot.values())
        for slot in range(max_slots):
            if slot not in used:
                self._finger_to_slot[fid] = slot
                return slot
        raise RuntimeError("virtual touchpad ran out of MT slots")

    def _emit_virtual_frame(self, touches: list[Touch], button_events: list[tuple[int, int]]):
        assert self.ui is not None

        current_ids = {t.finger_id for t in touches}
        removed = [fid for fid in self._finger_to_slot if fid not in current_ids]

        with self._emit_lock:
            for fid in removed:
                slot = self._finger_to_slot.pop(fid)
                self.ui.write(ecodes.EV_ABS, ecodes.ABS_MT_SLOT, slot)
                self.ui.write(ecodes.EV_ABS, ecodes.ABS_MT_TRACKING_ID, -1)

            for t in touches:
                is_new = t.finger_id not in self._finger_to_slot
                slot = self._allocate_slot(t.finger_id)
                self.ui.write(ecodes.EV_ABS, ecodes.ABS_MT_SLOT, slot)
                if is_new:
                    self.ui.write(ecodes.EV_ABS, ecodes.ABS_MT_TRACKING_ID, t.finger_id)
                self.ui.write(ecodes.EV_ABS, ecodes.ABS_MT_POSITION_X, t.x)
                self.ui.write(ecodes.EV_ABS, ecodes.ABS_MT_POSITION_Y, t.y)
                self.ui.write(ecodes.EV_ABS, ecodes.ABS_MT_PRESSURE, t.pressure)
                self.ui.write(ecodes.EV_ABS, ecodes.ABS_MT_TOUCH_MAJOR, t.major)
                self.ui.write(ecodes.EV_ABS, ecodes.ABS_MT_TOUCH_MINOR, t.minor)
                self.ui.write(ecodes.EV_ABS, ecodes.ABS_MT_ORIENTATION, t.orientation)

            count = len(touches)
            self.ui.write(ecodes.EV_KEY, ecodes.BTN_TOUCH, 1 if count else 0)
            self.ui.write(ecodes.EV_KEY, ecodes.BTN_TOOL_FINGER, 1 if count == 1 else 0)
            self.ui.write(ecodes.EV_KEY, ecodes.BTN_TOOL_DOUBLETAP, 1 if count == 2 else 0)
            self.ui.write(ecodes.EV_KEY, ecodes.BTN_TOOL_TRIPLETAP, 1 if count == 3 else 0)
            self.ui.write(ecodes.EV_KEY, ecodes.BTN_TOOL_QUADTAP, 1 if count == 4 else 0)
            if hasattr(ecodes, "BTN_TOOL_QUINTTAP"):
                self.ui.write(
                    ecodes.EV_KEY,
                    ecodes.BTN_TOOL_QUINTTAP,
                    1 if count >= 5 else 0,
                )

            if touches:
                primary = min(touches, key=lambda t: self._finger_to_slot[t.finger_id])
                self.ui.write(ecodes.EV_ABS, ecodes.ABS_X, primary.x)
                self.ui.write(ecodes.EV_ABS, ecodes.ABS_Y, primary.y)
                self.ui.write(ecodes.EV_ABS, ecodes.ABS_PRESSURE, primary.pressure)
            else:
                self.ui.write(ecodes.EV_ABS, ecodes.ABS_PRESSURE, 0)

            for code, value in button_events:
                self.ui.write(ecodes.EV_KEY, code, value)

            self.ui.syn()

    def process_report(self, frame: RawFrame, events, host_ns: int):
        if self.ui is None or self.clicks is None:
            return

        reserved = self._classify_button_contacts(frame)
        motion_frame = self.liftoff_filter.filter_frame(
            frame, ignored_ids=reserved
        )
        virtual_touches = self._virtual_touches(motion_frame, reserved)

        button_events: list[tuple[int, int]] = []
        for ev in events:
            if ev.type != ecodes.EV_KEY or ev.code != ecodes.BTN_LEFT:
                continue
            if ev.value:
                allowed_ids = reserved if self.clicks.button_areas else None
                code = self.clicks.press(frame, allowed_ids=allowed_ids)
                if code is not None:
                    button_events.append((code, 1))
            else:
                code = self.clicks.release()
                if code is not None:
                    button_events.append((code, 0))

        # Pressure is a fallback for custom button areas only. Raw BTN_LEFT
        # remains authoritative when it arrives normally.
        code = self.clicks.maybe_release_from_frame(frame)
        if code is not None:
            button_events.append((code, 0))

        self._emit_virtual_frame(virtual_touches, button_events)

        if self.scroll_engine is not None:
            scroll_frame = RawFrame(
                timestamp=motion_frame.timestamp,
                touches=tuple(self._scroll_touches(motion_frame, reserved)),
                spurious=motion_frame.spurious,
                end_of_frame=motion_frame.end_of_frame,
            )
            self.scroll_engine.process_scroll_frame(scroll_frame, host_ns)

    def reset_runtime(self):
        if self.scroll_engine is not None:
            self.scroll_engine.reset_runtime()

        self._accepted.clear()
        self.liftoff_filter.reset()
        self._button_contacts.clear()
        self._previous_physical_ids.clear()

        if self.ui is not None:
            try:
                self._emit_virtual_frame([], [])
            except Exception:
                LOG.debug("Unable to flush virtual touch contacts", exc_info=True)
        self._finger_to_slot.clear()

        if self.clicks is not None:
            code = self.clicks.release()
            if code is not None and self.ui is not None:
                with self._emit_lock:
                    self.ui.write(ecodes.EV_KEY, code, 0)
                    self.ui.syn()

    def close(self):
        self.reset_runtime()
        self.scroll_engine = None
        if self.kinetic_ui is not None:
            self.kinetic_ui.close()
            self.kinetic_ui = None
        if self.scroll_ui is not None:
            self.scroll_ui.close()
            self.scroll_ui = None
        if self.ui is not None:
            self.ui.close()
            self.ui = None
