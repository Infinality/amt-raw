#!/usr/bin/env python3
from __future__ import annotations

import argparse
import logging
import math
import signal
import threading

from amt_raw.native import NativeBackend
from amt_raw.pointer import PointerBackend
from amt_raw.runner import run
from amt_raw.touchpad import TouchpadBackend

LOG = logging.getLogger("amt-raw")


def build_parser():
    p = argparse.ArgumentParser(
        description="Apple Magic Trackpad userspace bridge (custom pointer, enhanced touchpad, or native baseline)."
    )
    p.add_argument("--mode", choices=("pointer", "touchpad", "native"), default="pointer")
    p.add_argument("--device", default="auto", help="Explicit physical evdev path, or 'auto' (default) for USB-C Magic Trackpad 05ac:0324 interface 01")
    p.add_argument("--reconnect-delay-seconds", type=float, default=2.0)
    p.add_argument("--hardware-report-rate-hz", type=float, default=120.0,
                   help="Request the USB-C Magic Trackpad hardware report rate via feature report 0x99/0xf2 (0 disables; 120 tested)")
    p.add_argument("--resampling-rate-hz", type=float, default=120.0,
                   help="Pointer-mode fixed-rate REL_X/REL_Y output clock (default 120; 0 disables; e.g. 60, 120, 144, 240)")
    p.add_argument("--resampling-predictor", choices=("latest", "adaptive", "interpolated"), default="interpolated",
                   help="Pointer resampler mode: interpolated is the preferred delayed known-trajectory renderer; latest is the low-latency causal CV reference; adaptive adds conservative causal acceleration prediction")
    p.add_argument("--invert-x", action="store_true")
    p.add_argument("--invert-y", action="store_true")
    p.add_argument("--print-raw", action="store_true")
    p.add_argument("--log-level", choices=("warning", "info", "debug"), default="warning")
    p.add_argument("-v", "--verbose", action="count", default=0)

    p.add_argument("--physical-click", choices=("on", "off"), default="on",
                   help="Forward the Magic Trackpad's hardware/force BTN_LEFT click bit")
    p.add_argument("--button-areas", choices=("on", "off"), default="off",
                   help="Reserve a bottom L/M/R button strip; pointer-mode contacts that start there are isolated from cursor motion")
    p.add_argument("--button-area-height-mm", type=float, default=20.0)
    p.add_argument("--physical-click-release-pressure", type=int, default=30,
                   help="Custom button-area pressure fallback for logical button-up (-1 disables)")
    p.add_argument("--physical-click-release-debounce-frames", type=int, default=2,
                   help="Consecutive low-pressure reports required by the physical-click release fallback")

    p.add_argument("--liftoff-jitter-suppression", choices=("on", "off"), default="on",
                   help="Freeze final low/falling-pressure contact coordinates to suppress lift-off cursor/scroll jitter")
    p.add_argument("--liftoff-pressure-threshold", type=int, default=8,
                   help="Low-pressure ceiling for trend-based lift-off recognition")
    p.add_argument("--liftoff-fall-frames", type=int, default=3,
                   help="Consecutive monotonically falling pressure reports required for trend-based lift-off recognition")
    p.add_argument("--liftoff-pressure-drop", type=int, default=4,
                   help="Minimum pressure drop across the falling sequence before lift-off coordinates are frozen")
    p.add_argument("--liftoff-recovery-pressure", type=int, default=10,
                   help="Pressure at/above which a frozen release candidate is treated as an active contact again")
    p.add_argument("--liftoff-recovery-timeout-ms", type=float, default=80.0,
                   help="Maximum time a still-present tracking ID may remain frozen before the release candidate is cancelled")
    p.add_argument("--liftoff-motion-window-ms", type=float, default=45.0,
                   help="Recent raw-motion window used to distinguish precision release from active motion")
    p.add_argument("--liftoff-max-path-speed", type=float, default=0.8,
                   help="Maximum recent raw path speed (units/ms) at which lift-off suppression may engage")

    p.add_argument("--touchpad-name", default="AMT Raw Touchpad")
    p.add_argument("--touchpad-hysteresis-units", type=float, default=2.0,
                   help="Per-contact MT coordinate deadband in touchpad mode (0 disables)")

    p.add_argument("--pointer-name", default="AMT Raw Pointer")
    p.add_argument("--pointer-gain", type=float, default=1.35)
    p.add_argument("--pointer-gain-x", type=float)
    p.add_argument("--pointer-gain-y", type=float)
    p.add_argument("--pointer-hysteresis-units", type=float, default=2.0,
                   help="One-finger absolute-coordinate deadband for stationary jitter (0 disables)")
    p.add_argument("--pointer-scroll-hysteresis-units", type=float, default=2.0,
                   help="Per-finger absolute-coordinate deadband for two-finger scrolling (0 disables)")
    p.add_argument("--pointer-precision-gain", type=float, default=0.55)
    p.add_argument("--pointer-precision-speed", type=float, default=0.8)
    p.add_argument("--pointer-precision-transition", type=float, default=1.5)
    p.add_argument("--pointer-precision-filter-ms", type=float, default=35.0)
    p.add_argument("--pointer-tap", action=argparse.BooleanOptionalAction, default=True)
    p.add_argument("--pointer-two-finger-tap", action=argparse.BooleanOptionalAction, default=True)
    p.add_argument("--pointer-three-finger-tap", choices=("on", "off"), default="on",
                   help="Three-finger tap emits middle click in pointer mode; 3+ finger motion remains reserved for external gestures")
    p.add_argument("--pointer-tap-max-ms", type=float, default=300.0)
    p.add_argument("--pointer-two-finger-tap-grace-ms", type=float, default=120.0,
                   help="Initial two-finger tap arbitration window; scroll motion is held while small landing motion settles")
    p.add_argument("--pointer-tap-move-units", type=float, default=83.0)
    p.add_argument("--pointer-tap-landing-move-units", type=float, default=2000.0,
                   help="Larger movement slop allowed only during the initial one-finger landing grace before the tap anchor is re-centered")
    p.add_argument("--pointer-tap-drag-window-ms", type=float, default=180.0)
    p.add_argument("--pointer-tap-drag-activation-units", type=float, default=60.0)
    p.add_argument("--pointer-tap-drag-hold-ms", type=float, default=120.0,
                   help="Retouch hold duration that promotes a tap-drag candidate even before displacement crosses the activation threshold (0 disables)")
    p.add_argument("--pointer-scroll", action=argparse.BooleanOptionalAction, default=True)
    p.add_argument("--pointer-scroll-start-units", type=float, default=0.0,
                   help="Optional additional filtered-centroid travel before scrolling starts; accumulated motion is replayed (0 recommended)")
    p.add_argument("--pointer-scroll-units-per-detent", type=float, default=66.0)
    p.add_argument("--pointer-scroll-invert", action="store_true")
    p.add_argument("--pointer-horizontal-scroll", action=argparse.BooleanOptionalAction, default=True)
    p.add_argument("--pointer-hscroll-units-per-detent", type=float, default=66.0)
    p.add_argument("--pointer-hscroll-invert", action="store_true")
    p.add_argument("--pointer-scroll-axis-lock", choices=("on", "off"), default="off")
    p.add_argument("--pointer-scroll-axis-lock-ratio", type=float, default=1.5)
    p.add_argument("--pointer-multitouch-entry-grace-ms", type=float, default=50.0)
    p.add_argument("--pointer-multitouch-exit-grace-ms", type=float, default=80.0)
    # Kinetic parameters are raw-coordinate physics and are shared by pointer
    # and touchpad modes.  Touchpad mode also reuses the pointer two-finger
    # live-scroll state machine through its relative scroll companion.
    p.add_argument("--pointer-kinetic-scroll", choices=("on", "off"), default="on")
    p.add_argument("--touchpad-kinetic-scroll", choices=("on", "off"), default="on")
    p.add_argument("--kinetic-history-ms", type=float, default=80.0)
    p.add_argument("--kinetic-start-velocity", type=float, default=0.58)
    p.add_argument("--kinetic-stop-velocity", type=float, default=0.05)
    p.add_argument("--kinetic-decay-ms", type=float, default=500.0)
    p.add_argument("--kinetic-max-ms", type=float, default=1600.0)
    p.add_argument("--pointer-max-raw-jump", type=int, default=500)
    p.add_argument("--no-uinput", action="store_true", help="Pointer-mode diagnostic only")
    return p


def validate(p, a):
    positive = [
        (a.reconnect_delay_seconds, "--reconnect-delay-seconds"),
        (a.button_area_height_mm, "--button-area-height-mm"),
        (a.pointer_gain, "--pointer-gain"),
        (a.pointer_precision_gain, "--pointer-precision-gain"),
        (a.pointer_tap_max_ms, "--pointer-tap-max-ms"),
        (a.pointer_tap_move_units, "--pointer-tap-move-units"),
        (a.pointer_tap_landing_move_units, "--pointer-tap-landing-move-units"),
        (a.pointer_scroll_units_per_detent, "--pointer-scroll-units-per-detent"),
        (a.pointer_hscroll_units_per_detent, "--pointer-hscroll-units-per-detent"),
        (a.kinetic_history_ms, "--kinetic-history-ms"),
        (a.kinetic_start_velocity, "--kinetic-start-velocity"),
        (a.kinetic_stop_velocity, "--kinetic-stop-velocity"),
        (a.kinetic_decay_ms, "--kinetic-decay-ms"),
        (a.kinetic_max_ms, "--kinetic-max-ms"),
    ]
    for value, name in positive:
        if not math.isfinite(value) or value <= 0:
            p.error(f"{name} must be positive")
    if not math.isfinite(a.hardware_report_rate_hz) or a.hardware_report_rate_hz < 0:
        p.error("--hardware-report-rate-hz must be finite and >=0")
    if not math.isfinite(a.resampling_rate_hz) or a.resampling_rate_hz < 0:
        p.error("--resampling-rate-hz must be finite and >=0")
    if a.resampling_rate_hz > 1000:
        p.error("--resampling-rate-hz must be <=1000 Hz")
    if a.hardware_report_rate_hz and not (3850.0 / 255.0 <= a.hardware_report_rate_hz <= 3850.0):
        p.error("--hardware-report-rate-hz must be 0 (disabled) or between about 15.1 and 3850")
    for value, name in ((a.pointer_gain_x, "--pointer-gain-x"), (a.pointer_gain_y, "--pointer-gain-y")):
        if value is not None and (not math.isfinite(value) or value <= 0):
            p.error(f"{name} must be positive")
    if not (0 < a.pointer_precision_gain <= 1):
        p.error("--pointer-precision-gain must be >0 and <=1")
    if a.pointer_precision_speed < 0:
        p.error("--pointer-precision-speed must be >=0")
    if a.pointer_precision_transition <= a.pointer_precision_speed:
        p.error("--pointer-precision-transition must exceed --pointer-precision-speed")
    if a.pointer_precision_filter_ms < 0:
        p.error("--pointer-precision-filter-ms must be >=0")
    if a.physical_click_release_pressure < -1:
        p.error("--physical-click-release-pressure must be >= -1")
    if a.physical_click_release_debounce_frames < 1:
        p.error("--physical-click-release-debounce-frames must be >=1")
    if a.liftoff_pressure_threshold < 0:
        p.error("--liftoff-pressure-threshold must be >=0")
    if a.liftoff_fall_frames < 2:
        p.error("--liftoff-fall-frames must be >=2")
    if a.liftoff_pressure_drop < 0:
        p.error("--liftoff-pressure-drop must be >=0")
    if a.liftoff_recovery_pressure < 0:
        p.error("--liftoff-recovery-pressure must be >=0")
    if not math.isfinite(a.liftoff_recovery_timeout_ms) or a.liftoff_recovery_timeout_ms <= 0:
        p.error("--liftoff-recovery-timeout-ms must be finite and >0")
    if not math.isfinite(a.liftoff_motion_window_ms) or a.liftoff_motion_window_ms <= 0:
        p.error("--liftoff-motion-window-ms must be finite and >0")
    if not math.isfinite(a.liftoff_max_path_speed) or a.liftoff_max_path_speed < 0:
        p.error("--liftoff-max-path-speed must be finite and >=0")
    if not math.isfinite(a.touchpad_hysteresis_units) or a.touchpad_hysteresis_units < 0:
        p.error("--touchpad-hysteresis-units must be finite and >=0")
    if not math.isfinite(a.pointer_hysteresis_units) or a.pointer_hysteresis_units < 0:
        p.error("--pointer-hysteresis-units must be finite and >=0")
    if not math.isfinite(a.pointer_scroll_hysteresis_units) or a.pointer_scroll_hysteresis_units < 0:
        p.error("--pointer-scroll-hysteresis-units must be finite and >=0")
    if a.pointer_two_finger_tap_grace_ms < 0:
        p.error("--pointer-two-finger-tap-grace-ms must be >=0")
    if (a.pointer_tap_drag_window_ms < 0
            or a.pointer_tap_drag_activation_units < 0
            or a.pointer_tap_drag_hold_ms < 0):
        p.error("tap-drag values must be >=0")
    if a.pointer_scroll_start_units < 0:
        p.error("--pointer-scroll-start-units must be >=0")
    if a.pointer_scroll_axis_lock_ratio <= 1:
        p.error("--pointer-scroll-axis-lock-ratio must be >1")
    if a.pointer_multitouch_entry_grace_ms < 0 or a.pointer_multitouch_exit_grace_ms < 0:
        p.error("multitouch grace values must be >=0")
    if a.kinetic_stop_velocity >= a.kinetic_start_velocity:
        p.error("kinetic stop velocity must be below start velocity")


def main():
    p = build_parser()
    a = p.parse_args()
    validate(p, a)
    a.physical_click = a.physical_click == "on"
    a.button_areas = a.button_areas == "on"
    a.pointer_kinetic_scroll = a.pointer_kinetic_scroll == "on"
    a.touchpad_kinetic_scroll = a.touchpad_kinetic_scroll == "on"
    a.pointer_three_finger_tap = a.pointer_three_finger_tap == "on"
    a.liftoff_jitter_suppression = a.liftoff_jitter_suppression == "on"

    # Pointer engine retains its historical numeric attribute names internally;
    # map the shared kinetic physics onto them so pointer behavior remains
    # equivalent apart from the public configuration rename.
    a.pointer_kinetic_history_ms = a.kinetic_history_ms
    a.pointer_kinetic_start_velocity = a.kinetic_start_velocity
    a.pointer_kinetic_stop_velocity = a.kinetic_stop_velocity
    a.pointer_kinetic_decay_ms = a.kinetic_decay_ms
    a.pointer_kinetic_max_ms = a.kinetic_max_ms

    level = {"warning": logging.WARNING, "info": logging.INFO, "debug": logging.DEBUG}[a.log_level]
    if a.verbose == 1:
        level = logging.INFO
    elif a.verbose >= 2:
        level = logging.DEBUG
    logging.basicConfig(level=logging.WARNING, format="%(asctime)s %(levelname)-8s %(name)s: %(message)s")
    LOG.setLevel(level)

    if a.mode != "pointer" and a.resampling_rate_hz > 0:
        LOG.warning(
            "AMT_RESAMPLING_RATE_HZ/--resampling-rate-hz is currently pointer-mode only; ignoring %.3f Hz in %s mode",
            a.resampling_rate_hz,
            a.mode,
        )
        a.resampling_rate_hz = 0.0

    if a.mode == "native":
        backend = NativeBackend(a)
    elif a.mode == "touchpad":
        backend = TouchpadBackend(a)
    else:
        backend = PointerBackend(a)

    stop = threading.Event()

    def sig(signum, _frame):
        LOG.info("Received signal %s; stopping", signum)
        stop.set()

    signal.signal(signal.SIGINT, sig)
    signal.signal(signal.SIGTERM, sig)
    try:
        return run(a, backend, stop)
    finally:
        backend.close()


if __name__ == "__main__":
    raise SystemExit(main())
