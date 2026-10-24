#!/usr/bin/python3
"""Capture synchronized amt-raw lift-off/tap calibration traces.

This is a side utility for the Apple Magic Trackpad USB-C over USB or
Bluetooth. It does
not grab either evdev device and is intended to run while amt-raw itself is
active in pointer mode.

It records:
  * every physical Type-B multitouch SYN_REPORT frame;
  * events emitted by the live ``AMT Raw Pointer`` uinput device;
  * selected amt-raw INFO/DEBUG log messages from journald, including resampler cadence/prediction statistics, live resampling-mode selection, dual-shadow predictor statistics, and interpolated-mode delay/curve counters;
  * per-contact and per-attempt summaries, including offline simulations of
    several lift-off pressure thresholds using the alpha31-alpha35 lift-off algorithm;
  * optional libinput debug-events output for the live virtual pointer.

Typical use:

    sudo ./amt-liftoff-capture.py --scenario position-release
    sudo ./amt-liftoff-capture.py --scenario position-tap
    sudo ./amt-liftoff-capture.py --scenario position-double-tap

Perform repeated attempts naturally.  Keep the contacts that belong to one
attempt less than --attempt-gap-ms apart, then pause longer than that between
attempts.  Ctrl-C ends capture and writes the final summary.
"""

from __future__ import annotations

import argparse
import datetime as _dt
import errno
import glob
import json
import math
import os
import re
import statistics
import select
import shutil
import subprocess
import sys
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Optional

try:
    from evdev import InputDevice, ecodes, list_devices
except ImportError as exc:  # pragma: no cover - user-facing dependency check
    raise SystemExit(
        "python3-evdev is required (Fedora: sudo dnf install python3-evdev)"
    ) from exc

SCRIPT_VERSION = "2.5"
APPLE_USB_VENDOR = 0x05AC
APPLE_BLUETOOTH_VENDOR = 0x004C
MAGIC_TRACKPAD_USBC = 0x0324
POINTER_NAME = "AMT Raw Pointer"

RELEVANT_JOURNAL = re.compile(
    r"(?:"
    r"lift-off jitter|"
    r"one-finger tap|"
    r"tap candidate|"
    r"tap sequence|"
    r"new touch joined held tap|"
    r"tap-and-drag|"
    r"tap hold|"
    r"tap retouch|"
    r"entry grace|"
    r"3\+ contacts|"
    r"external 3\+|"
    r"two-finger tap suppressed|"
    r"two-finger tap grace ended|"
    r"two-finger scroll|"
    r"kinetic scroll|"
    r"3\+ gesture -> two-finger|"
    r"two-finger .*exit grace|"
    r"button-area|"
    r"physical click|"
    r"resampler"
    r")",
    re.IGNORECASE,
)

DEFAULT_SETTINGS = {
    "AMT_LIFTOFF_PRESSURE_THRESHOLD": "8",
    "AMT_LIFTOFF_FALL_FRAMES": "3",
    "AMT_LIFTOFF_PRESSURE_DROP": "4",
    "AMT_LIFTOFF_RECOVERY_PRESSURE": "10",
    "AMT_LIFTOFF_RECOVERY_TIMEOUT_MS": "80",
    "AMT_LIFTOFF_MOTION_WINDOW_MS": "45",
    "AMT_LIFTOFF_MAX_PATH_SPEED": "0.8",
    "AMT_POINTER_TAP_MAX_MS": "300",
    "AMT_POINTER_TAP_MOVE_UNITS": "83",
    "AMT_POINTER_TAP_DRAG_HOLD_MS": "120",
    "AMT_POINTER_TAP_LANDING_MOVE_UNITS": "2000",
    "AMT_RESAMPLING_RATE_HZ": "120",
    "AMT_RESAMPLING_PREDICTOR": "interpolated",
    "AMT_POINTER_MULTITOUCH_ENTRY_GRACE_MS": "50",
    "AMT_BUTTON_AREAS": "off",
    "AMT_BUTTON_AREA_HEIGHT_MM": "20",
    "AMT_PHYSICAL_CLICK": "on",
}


@dataclass
class SlotState:
    tracking_id: int = -1
    x: int = 0
    y: int = 0
    pressure: int = 0
    major: int = 0
    minor: int = 0
    orientation: int = 0

    @property
    def active(self) -> bool:
        return self.tracking_id >= 0


@dataclass
class ContactTrace:
    finger_id: int
    slot: int
    attempt_id: int
    samples: list[dict[str, Any]] = field(default_factory=list)
    lifted_ns: Optional[int] = None
    lifted_evdev_s: Optional[float] = None


@dataclass
class Attempt:
    attempt_id: int
    scenario: str
    start_ns: int
    last_activity_ns: int
    idle_since_ns: Optional[int] = None
    contacts: list[ContactTrace] = field(default_factory=list)
    pointer_rel_x: int = 0
    pointer_rel_y: int = 0
    pointer_wheel_hi_res: int = 0
    pointer_hwheel_hi_res: int = 0
    pointer_pos_x: int = 0
    pointer_pos_y: int = 0
    pointer_button_events: list[dict[str, Any]] = field(default_factory=list)
    pointer_frames: int = 0
    pointer_motion_times_ns: list[int] = field(default_factory=list)
    journal_lines: list[str] = field(default_factory=list)
    libinput_lines: list[str] = field(default_factory=list)
    ended_ns: Optional[int] = None


class PhysicalAssembler:
    def __init__(self, dev: InputDevice):
        slot_info = dev.absinfo(ecodes.ABS_MT_SLOT)
        self.slot_min = slot_info.min
        self.slot_max = slot_info.max
        self.current_slot = self.slot_min
        self.slots = {
            i: SlotState() for i in range(self.slot_min, self.slot_max + 1)
        }
        self.button_left = 0
        self.frame_seq = 0

    def update(self, event: Any, host_ns: int) -> Optional[dict[str, Any]]:
        if event.type == ecodes.EV_ABS:
            c, v = event.code, event.value
            if c == ecodes.ABS_MT_SLOT:
                if self.slot_min <= v <= self.slot_max:
                    self.current_slot = v
                return None

            slot = self.slots[self.current_slot]
            if c == ecodes.ABS_MT_TRACKING_ID:
                if v < 0:
                    self.slots[self.current_slot] = SlotState()
                else:
                    slot.tracking_id = v
            elif c == ecodes.ABS_MT_POSITION_X:
                slot.x = v
            elif c == ecodes.ABS_MT_POSITION_Y:
                slot.y = v
            elif c == ecodes.ABS_MT_PRESSURE:
                slot.pressure = v
            elif c == ecodes.ABS_MT_TOUCH_MAJOR:
                slot.major = v
            elif c == ecodes.ABS_MT_TOUCH_MINOR:
                slot.minor = v
            elif c == ecodes.ABS_MT_ORIENTATION:
                slot.orientation = v
            return None

        if event.type == ecodes.EV_KEY and event.code == ecodes.BTN_LEFT:
            self.button_left = event.value
            return None

        if event.type == ecodes.EV_SYN and event.code == ecodes.SYN_REPORT:
            self.frame_seq += 1
            touches = []
            for slot_no, slot in self.slots.items():
                if not slot.active:
                    continue
                touches.append(
                    {
                        "slot": slot_no,
                        "id": slot.tracking_id,
                        "x": slot.x,
                        "y": slot.y,
                        "pressure": slot.pressure,
                        "major": slot.major,
                        "minor": slot.minor,
                        "orientation": slot.orientation,
                    }
                )
            return {
                "type": "physical_frame",
                "host_ns": host_ns,
                "evdev_s": event.timestamp(),
                "frame_seq": self.frame_seq,
                "button_left": self.button_left,
                "touches": touches,
            }

        return None


class PointerAssembler:
    def __init__(self):
        self.rel: dict[int, int] = {}
        self.keys: list[dict[str, Any]] = []
        self.misc: list[dict[str, int]] = []
        self.frame_seq = 0

    def update(self, event: Any, host_ns: int) -> Optional[dict[str, Any]]:
        if event.type == ecodes.EV_REL:
            self.rel[event.code] = self.rel.get(event.code, 0) + event.value
            return None
        if event.type == ecodes.EV_KEY:
            self.keys.append({"code": event.code, "value": event.value})
            return None
        if event.type == ecodes.EV_SYN and event.code == ecodes.SYN_REPORT:
            self.frame_seq += 1
            record = {
                "type": "pointer_frame",
                "host_ns": host_ns,
                "evdev_s": event.timestamp(),
                "frame_seq": self.frame_seq,
                "rel": {
                    "x": self.rel.get(ecodes.REL_X, 0),
                    "y": self.rel.get(ecodes.REL_Y, 0),
                    "wheel_hi_res": self.rel.get(
                        getattr(ecodes, "REL_WHEEL_HI_RES", -9999), 0
                    ),
                    "hwheel_hi_res": self.rel.get(
                        getattr(ecodes, "REL_HWHEEL_HI_RES", -9998), 0
                    ),
                },
                "keys": list(self.keys),
            }
            self.rel.clear()
            self.keys.clear()
            self.misc.clear()
            return record
        return None


def _magic_trackpad_probe(dev: InputDevice) -> tuple[bool, str]:
    name = dev.name or ""
    if name.startswith("AMT Raw "):
        return False, f"virtual replacement name={name!r}"

    vendor = int(dev.info.vendor)
    product = int(dev.info.product)
    if (
        vendor not in (APPLE_USB_VENDOR, APPLE_BLUETOOTH_VENDOR)
        or product != MAGIC_TRACKPAD_USBC
    ):
        return (
            False,
            f"VID:PID={vendor:04x}:{product:04x} name={name!r} "
            f"(expected USB {APPLE_USB_VENDOR:04x}:{MAGIC_TRACKPAD_USBC:04x} "
            f"or Bluetooth {APPLE_BLUETOOTH_VENDOR:04x}:{MAGIC_TRACKPAD_USBC:04x})",
        )

    try:
        caps = dev.capabilities(absinfo=False)
    except OSError as exc:
        return False, f"VID:PID matched but capabilities() failed: {exc}"

    abs_codes = set(caps.get(ecodes.EV_ABS, []))
    needed = {
        ecodes.ABS_MT_SLOT,
        ecodes.ABS_MT_TRACKING_ID,
        ecodes.ABS_MT_POSITION_X,
        ecodes.ABS_MT_POSITION_Y,
    }
    missing = sorted(needed - abs_codes)
    if missing:
        names = [ecodes.ABS.get(code, str(code)) for code in missing]
        return (
            False,
            f"VID:PID matched name={name!r} but MT capabilities are missing "
            + ", ".join(names),
        )
    return True, f"VID:PID={vendor:04x}:{product:04x} name={name!r} MT capabilities OK"


def _is_magic_trackpad(dev: InputDevice) -> bool:
    ok, _ = _magic_trackpad_probe(dev)
    return ok


def _interface_score(path: str) -> int:
    base = os.path.basename(path)
    target = os.path.realpath(f"/sys/class/input/{base}/device")
    return 10 if ":1.1/" in target else 0


def find_physical(explicit: Optional[str]) -> InputDevice:
    diagnostics: list[str] = []

    if explicit and explicit != "auto":
        try:
            dev = InputDevice(explicit)
        except OSError as exc:
            raise RuntimeError(f"cannot open explicit physical device {explicit}: {exc}") from exc
        ok, why = _magic_trackpad_probe(dev)
        if not ok:
            dev.close()
            raise RuntimeError(f"{explicit} rejected: {why}")
        return dev

    by_id = sorted(
        glob.glob("/dev/input/by-id/*Apple*Magic*Trackpad*-if01-event-mouse")
    )
    if not by_id:
        diagnostics.append("no matching *-if01-event-mouse by-id symlink exists")
    for path in by_id:
        try:
            dev = InputDevice(path)
        except OSError as exc:
            diagnostics.append(f"by-id {path}: open failed: {exc}")
            continue
        ok, why = _magic_trackpad_probe(dev)
        if ok:
            return dev
        diagnostics.append(
            f"by-id {path} -> {os.path.realpath(path)}: rejected: {why}"
        )
        dev.close()

    candidates: list[tuple[int, str, InputDevice]] = []
    for path in list_devices():
        try:
            dev = InputDevice(path)
        except OSError as exc:
            diagnostics.append(f"event {path}: open failed: {exc}")
            continue
        ok, why = _magic_trackpad_probe(dev)
        if ok:
            candidates.append((_interface_score(dev.path), dev.path, dev))
        else:
            # Only include potentially relevant Apple/AMT rejection details to
            # avoid burying the useful diagnostics in every keyboard/mouse.
            vendor = int(dev.info.vendor)
            if (
                vendor in (APPLE_USB_VENDOR, APPLE_BLUETOOTH_VENDOR)
                or (dev.name or "").startswith("AMT Raw ")
            ):
                diagnostics.append(f"event {path}: rejected: {why}")
            dev.close()
    if not candidates:
        py = sys.executable or "unknown"
        try:
            import evdev as _evdev
            evdev_path = getattr(_evdev, "__file__", "unknown")
            evdev_version = getattr(_evdev, "__version__", "unknown")
        except Exception:
            evdev_path = "unknown"
            evdev_version = "unknown"
        detail = "; ".join(diagnostics) if diagnostics else "no candidate diagnostics available"
        raise RuntimeError(
            "Apple Magic Trackpad USB-C USB/Bluetooth MT event device not found. "
            f"python={py}; evdev={evdev_version} ({evdev_path}); probes: {detail}"
        )
    candidates.sort(key=lambda x: (x[0], x[1]), reverse=True)
    chosen = candidates[0][2]
    for _, _, dev in candidates[1:]:
        dev.close()
    return chosen


def find_pointer(explicit: Optional[str]) -> Optional[InputDevice]:
    if explicit:
        return InputDevice(explicit)
    for path in list_devices():
        try:
            dev = InputDevice(path)
        except OSError:
            continue
        if dev.name == POINTER_NAME:
            return dev
        dev.close()
    return None


def parse_sysconfig(path: str) -> dict[str, str]:
    values = dict(DEFAULT_SETTINGS)
    try:
        text = Path(path).read_text(encoding="utf-8", errors="replace")
    except OSError:
        return values
    for raw in text.splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        key = key.strip()
        value = value.strip().strip('"').strip("'")
        if key.startswith("AMT_"):
            values[key] = value
    return values


def _setting_float(settings: dict[str, str], key: str, fallback: float) -> float:
    try:
        return float(settings.get(key, fallback))
    except (TypeError, ValueError):
        return fallback


def _setting_int(settings: dict[str, str], key: str, fallback: int) -> int:
    try:
        return int(float(settings.get(key, fallback)))
    except (TypeError, ValueError):
        return fallback


def _setting_on(settings: dict[str, str], key: str, fallback: bool) -> bool:
    value = settings.get(key)
    if value is None:
        return fallback
    return value.strip().lower() in {"1", "yes", "true", "on"}


def parse_thresholds(text: str) -> list[int]:
    out: list[int] = []
    for item in text.split(","):
        item = item.strip()
        if not item:
            continue
        value = int(item)
        if value < 0:
            raise ValueError("thresholds must be >= 0")
        if value not in out:
            out.append(value)
    if not out:
        raise ValueError("at least one threshold is required")
    return out


def path_length(samples: Iterable[dict[str, Any]]) -> float:
    seq = list(samples)
    return sum(
        math.hypot(b["x"] - a["x"], b["y"] - a["y"])
        for a, b in zip(seq, seq[1:])
    )


def max_displacement(samples: list[dict[str, Any]]) -> float:
    if not samples:
        return 0.0
    x0, y0 = samples[0]["x"], samples[0]["y"]
    return max(math.hypot(s["x"] - x0, s["y"] - y0) for s in samples)


def recent_path_speed(
    history: deque[tuple[float, int, int]], motion_window_ms: float
) -> Optional[float]:
    if len(history) < 3:
        return None
    elapsed = history[-1][0] - history[0][0]
    if elapsed < min(18.0, motion_window_ms):
        return None
    path = 0.0
    points = list(history)
    for (_, x0, y0), (_, x1, y1) in zip(points, points[1:]):
        path += math.hypot(x1 - x0, y1 - y0)
    return path / max(elapsed, 0.001)


def simulate_alpha31(
    samples: list[dict[str, Any]],
    *,
    pressure_threshold: int,
    recovery_pressure: int,
    fall_frames_required: int,
    pressure_drop: int,
    recovery_timeout_ms: float,
    motion_window_ms: float,
    max_path_speed: float,
) -> dict[str, Any]:
    """Replay one raw contact through the alpha31 lift-off state logic."""
    if len(samples) < 2:
        return {"engages": [], "bypasses": [], "recoveries": []}

    t0 = samples[0]["host_ns"] / 1_000_000.0
    history: deque[tuple[float, int, int]] = deque()

    def record(s: dict[str, Any]) -> None:
        t = s["host_ns"] / 1_000_000.0 - t0
        history.append((t, s["x"], s["y"]))
        while len(history) > 2 and t - history[0][0] > motion_window_ms:
            history.popleft()

    first = samples[0]
    last_pressure = first["pressure"]
    last_x, last_y = first["x"], first["y"]
    fall_frames = 1
    fall_start_pressure = first["pressure"]
    frozen = False
    frozen_x = frozen_y = 0
    freeze_t = 0.0
    low_pressure_entry_speed: Optional[float] = None
    offset_x = offset_y = 0
    record(first)
    output_points: list[tuple[int, int]] = [(first["x"], first["y"])]

    engages: list[dict[str, Any]] = []
    bypasses: list[dict[str, Any]] = []
    recoveries: list[dict[str, Any]] = []

    for idx, s in enumerate(samples[1:], 1):
        t = s["host_ns"] / 1_000_000.0 - t0
        p, x, y = s["pressure"], s["x"], s["y"]

        if frozen:
            elapsed = t - freeze_t
            pressure_recovered = p >= recovery_pressure
            timeout_recovered = elapsed >= recovery_timeout_ms
            if pressure_recovered or timeout_recovered:
                offset_x = frozen_x - x
                offset_y = frozen_y - y
                frozen = False
                last_pressure = p
                fall_frames = 1
                fall_start_pressure = p
                last_x = x + offset_x
                last_y = y + offset_y
                history.clear()
                record(s)
                low_pressure_entry_speed = None
                output_points.append((last_x, last_y))
                recoveries.append(
                    {
                        "sample": idx,
                        "t_ms": round(t, 3),
                        "pressure": p,
                        "reason": "pressure-rebound"
                        if pressure_recovered
                        else "candidate-timeout",
                        "elapsed_ms": round(elapsed, 3),
                    }
                )
                continue
            last_pressure = p
            output_points.append((frozen_x, frozen_y))
            continue

        approach_speed = recent_path_speed(history, motion_window_ms)
        previous_pressure = last_pressure
        if p < previous_pressure:
            if fall_frames <= 1:
                fall_start_pressure = previous_pressure
                fall_frames = 2
            else:
                fall_frames += 1
        else:
            fall_frames = 1
            fall_start_pressure = p

        if p <= pressure_threshold:
            if previous_pressure > pressure_threshold:
                low_pressure_entry_speed = approach_speed
        else:
            low_pressure_entry_speed = None

        total_drop = fall_start_pressure - p
        zero_release = p == 0 and previous_pressure > 0
        trend_release = (
            p <= pressure_threshold
            and fall_frames >= fall_frames_required
            and total_drop >= pressure_drop
        )
        candidate_speed = (
            low_pressure_entry_speed
            if low_pressure_entry_speed is not None
            else approach_speed
        )
        precision_release = (
            candidate_speed is not None and candidate_speed <= max_path_speed
        )

        if (zero_release or trend_release) and precision_release:
            frozen = True
            frozen_x, frozen_y = last_x, last_y
            freeze_t = t
            engages.append(
                {
                    "sample": idx,
                    "t_ms": round(t, 3),
                    "pressure": p,
                    "previous_pressure": previous_pressure,
                    "fall_frames": fall_frames,
                    "drop": total_drop,
                    "path_speed": round(candidate_speed, 4),
                    "anchor_x": frozen_x,
                    "anchor_y": frozen_y,
                    "zero_pressure": bool(zero_release),
                }
            )
            last_pressure = p
            output_points.append((frozen_x, frozen_y))
            continue

        if (zero_release or trend_release) and not precision_release:
            bypasses.append(
                {
                    "sample": idx,
                    "t_ms": round(t, 3),
                    "pressure": p,
                    "path_speed": None
                    if candidate_speed is None
                    else round(candidate_speed, 4),
                }
            )

        record(s)
        last_x = x + offset_x
        last_y = y + offset_y
        last_pressure = p
        output_points.append((last_x, last_y))

    out_x0, out_y0 = output_points[0]
    filtered_max_displacement = max(
        math.hypot(x - out_x0, y - out_y0) for x, y in output_points
    )
    filtered_path = sum(
        math.hypot(x1 - x0, y1 - y0)
        for (x0, y0), (x1, y1) in zip(output_points, output_points[1:])
    )

    if engages:
        first_engage = engages[0]
        i = int(first_engage["sample"])
        anchor_x = first_engage["anchor_x"]
        anchor_y = first_engage["anchor_y"]
        tail = samples[i:]
        first_engage["raw_tail_path"] = round(path_length(tail), 3)
        first_engage["raw_tail_net_from_anchor"] = round(
            math.hypot(samples[-1]["x"] - anchor_x, samples[-1]["y"] - anchor_y),
            3,
        )
        first_engage["ms_to_last_sample"] = round(
            (samples[-1]["host_ns"] - samples[i]["host_ns"]) / 1_000_000.0,
            3,
        )

    return {
        "engages": engages,
        "bypasses": bypasses,
        "recoveries": recoveries,
        "ends_frozen": frozen,
        "filtered_max_displacement_units": round(filtered_max_displacement, 3),
        "filtered_path_units": round(filtered_path, 3),
    }


def contact_summary(
    trace: ContactTrace,
    *,
    settings: dict[str, str],
    thresholds: list[int],
    y_max: int,
    y_resolution: int,
) -> dict[str, Any]:
    samples = trace.samples
    if not samples:
        return {"id": trace.finger_id, "slot": trace.slot, "samples": 0}

    first, last = samples[0], samples[-1]
    duration_ms = (last["host_ns"] - first["host_ns"]) / 1_000_000.0
    physical_active_duration_ms = (last["evdev_s"] - first["evdev_s"]) * 1000.0
    release_report_gap_ms = (
        None
        if trace.lifted_evdev_s is None
        else max(0.0, (trace.lifted_evdev_s - last["evdev_s"]) * 1000.0)
    )
    max_move = max_displacement(samples)
    p_values = [s["pressure"] for s in samples]
    tap_max_ms = _setting_float(settings, "AMT_POINTER_TAP_MAX_MS", 300.0)
    tap_move = _setting_float(settings, "AMT_POINTER_TAP_MOVE_UNITS", 83.0)
    grace_ms = _setting_float(
        settings, "AMT_POINTER_MULTITOUCH_ENTRY_GRACE_MS", 50.0
    )

    grace_samples = [
        s
        for s in samples
        if (s["host_ns"] - first["host_ns"]) / 1_000_000.0 <= grace_ms
    ]

    # Model the alpha35 one-finger tap landing policy.  Pointer motion remains
    # suppressed until entry grace expires.  If an active report survives to
    # that boundary, alpha35 re-centers the tap anchor there and only later
    # movement counts against AMT_POINTER_TAP_MOVE_UNITS.  If the contact ends
    # before any such active report, AMT_POINTER_TAP_LANDING_MOVE_UNITS is the
    # quick-release guard.
    landing_move = _setting_float(
        settings, "AMT_POINTER_TAP_LANDING_MOVE_UNITS", 180.0
    )
    reanchor_index = None
    for i, sample in enumerate(samples):
        if (sample["host_ns"] - first["host_ns"]) / 1_000_000.0 >= grace_ms:
            reanchor_index = i
            break

    alpha35_pre_settle = reanchor_index is None
    alpha35_post_settle_move = 0.0
    if reanchor_index is not None:
        anchor = samples[reanchor_index]
        later = samples[reanchor_index + 1 :]
        if later:
            alpha35_post_settle_move = max(
                math.hypot(s["x"] - anchor["x"], s["y"] - anchor["y"])
                for s in later
            )

    alpha35_tap_candidate = bool(
        physical_active_duration_ms <= tap_max_ms
        and (
            (alpha35_pre_settle and max_move <= landing_move)
            or (not alpha35_pre_settle and alpha35_post_settle_move <= tap_move)
        )
    )
    if physical_active_duration_ms > tap_max_ms:
        alpha35_tap_reason = "duration"
    elif alpha35_pre_settle and max_move > landing_move:
        alpha35_tap_reason = "pre-settle-landing"
    elif not alpha35_pre_settle and alpha35_post_settle_move > tap_move:
        alpha35_tap_reason = "settled-move"
    else:
        alpha35_tap_reason = "candidate"

    def post_time_move(anchor_ms: float):
        idx = None
        for i, sample in enumerate(samples):
            if (sample["host_ns"] - first["host_ns"]) / 1_000_000.0 >= anchor_ms:
                idx = i
                break
        if idx is None:
            return None
        anchor = samples[idx]
        tail = samples[idx + 1 :]
        if not tail:
            return 0.0
        return max(
            math.hypot(s["x"] - anchor["x"], s["y"] - anchor["y"])
            for s in tail
        )

    post18 = post_time_move(18.0)
    post27 = post_time_move(27.0)
    post36 = post_time_move(36.0)

    area_height_mm = _setting_float(settings, "AMT_BUTTON_AREA_HEIGHT_MM", 20.0)
    button_areas = _setting_on(settings, "AMT_BUTTON_AREAS", False)
    area_threshold_y = (
        y_max - area_height_mm * y_resolution if y_resolution > 0 else float("inf")
    )
    starts_in_button_area = bool(button_areas and first["y"] >= area_threshold_y)

    fall_frames = _setting_int(settings, "AMT_LIFTOFF_FALL_FRAMES", 3)
    pressure_drop = _setting_int(settings, "AMT_LIFTOFF_PRESSURE_DROP", 4)
    recovery_timeout = _setting_float(
        settings, "AMT_LIFTOFF_RECOVERY_TIMEOUT_MS", 80.0
    )
    motion_window = _setting_float(settings, "AMT_LIFTOFF_MOTION_WINDOW_MS", 45.0)
    max_speed = _setting_float(settings, "AMT_LIFTOFF_MAX_PATH_SPEED", 0.8)

    sims: dict[str, Any] = {}
    for threshold in thresholds:
        sims[str(threshold)] = simulate_alpha31(
            samples,
            pressure_threshold=threshold,
            recovery_pressure=threshold + 2,
            fall_frames_required=fall_frames,
            pressure_drop=pressure_drop,
            recovery_timeout_ms=recovery_timeout,
            motion_window_ms=motion_window,
            max_path_speed=max_speed,
        )

    return {
        "id": trace.finger_id,
        "slot": trace.slot,
        "samples": len(samples),
        "duration_ms": round(duration_ms, 3),
        "physical_active_duration_ms": round(physical_active_duration_ms, 3),
        "release_report_gap_ms": (
            None if release_report_gap_ms is None else round(release_report_gap_ms, 3)
        ),
        "start": {"x": first["x"], "y": first["y"], "pressure": first["pressure"]},
        "end": {"x": last["x"], "y": last["y"], "pressure": last["pressure"]},
        "pressure_min": min(p_values),
        "pressure_max": max(p_values),
        "path_units": round(path_length(samples), 3),
        "max_displacement_units": round(max_move, 3),
        "first_grace_max_displacement_units": round(
            max_displacement(grace_samples), 3
        ),
        "raw_tap_candidate_by_duration_and_slop": bool(
            physical_active_duration_ms <= tap_max_ms and max_move <= tap_move
        ),
        "alpha35_tap_candidate": alpha35_tap_candidate,
        "alpha35_tap_reason": alpha35_tap_reason,
        "alpha35_pre_settle": alpha35_pre_settle,
        "alpha35_post_settle_move_units": round(alpha35_post_settle_move, 3),
        "post18_move_units": None if post18 is None else round(post18, 3),
        "post27_move_units": None if post27 is None else round(post27, 3),
        "post36_move_units": None if post36 is None else round(post36, 3),
        "starts_in_button_area": starts_in_button_area,
        "offline_liftoff": sims,
    }


def short_contact_line(summary: dict[str, Any], thresholds: list[int]) -> str:
    parts = [
        f"id={summary.get('id')}",
        f"slot={summary.get('slot')}",
        f"dur={summary.get('physical_active_duration_ms', summary.get('duration_ms', 0)):.1f}ms",
        (
            "releaseGap=?"
            if summary.get("release_report_gap_ms") is None
            else f"releaseGap={summary.get('release_report_gap_ms', 0):.1f}ms"
        ),
        f"move={summary.get('max_displacement_units', 0):.1f}",
        f"grace50={summary.get('first_grace_max_displacement_units', 0):.1f}",
        f"p={summary.get('pressure_min', 0)}..{summary.get('pressure_max', 0)}",
        "tapRaw=" + ("yes" if summary.get("raw_tap_candidate_by_duration_and_slop") else "no"),
        "tapA35=" + ("yes" if summary.get("alpha35_tap_candidate") else "no")
        + "/" + str(summary.get("alpha35_tap_reason", "?")),
        (
            "post18/27/36="
            + "/".join(
                "-" if summary.get(k) is None else f"{summary.get(k):.1f}"
                for k in ("post18_move_units", "post27_move_units", "post36_move_units")
            )
        ),
    ]
    if summary.get("starts_in_button_area"):
        parts.append("BUTTON-AREA")
    simbits = []
    sims = summary.get("offline_liftoff", {})
    for t in thresholds:
        sim = sims.get(str(t), {})
        if sim.get("engages"):
            e = sim["engages"][0]
            tag = f"{t}:E@p{e['pressure']}/v{e['path_speed']}"
            if sim.get("recoveries"):
                tag += "/REC"
            simbits.append(tag)
        elif sim.get("bypasses"):
            b = sim["bypasses"][0]
            simbits.append(f"{t}:B@p{b['pressure']}/v{b['path_speed']}")
        else:
            simbits.append(f"{t}:-")
    parts.append("lift[" + " ".join(simbits) + "]")
    return "  " + " ".join(parts)



def _shadow_stats_summary(lines: list[str]) -> Optional[str]:
    """Return a compact summary of the newest cumulative alpha40 dual-shadow stats."""
    latest = None
    for line in lines:
        if "resampler stats" in line and "shadow25_changed=" in line and "shadow125_changed=" in line:
            latest = line
    if latest is None:
        return None
    pattern = re.compile(
        r"shadow_samples=(?P<samples>\d+).*?"
        r"shadow_err_latest med=(?P<lm>[0-9.]+) p95=(?P<lp>[0-9.]+).*?"
        r"shadow25_win_pct=(?P<w25>[0-9.]+).*?"
        r"shadow25_changed=(?P<c25>\d+).*?"
        r"shadow25_changed_win_pct=(?P<cw25>[0-9.]+).*?"
        r"shadow25_err_adapt med=(?P<am25>[0-9.]+) p95=(?P<ap25>[0-9.]+).*?"
        r"shadow25_changed_ratio med=(?P<crm25>[0-9.]+) p95=(?P<crp25>[0-9.]+).*?"
        r"shadow125_win_pct=(?P<w125>[0-9.]+).*?"
        r"shadow125_changed=(?P<c125>\d+).*?"
        r"shadow125_changed_win_pct=(?P<cw125>[0-9.]+).*?"
        r"shadow125_err_adapt med=(?P<am125>[0-9.]+) p95=(?P<ap125>[0-9.]+).*?"
        r"shadow125_changed_ratio med=(?P<crm125>[0-9.]+) p95=(?P<crp125>[0-9.]+)"
    )
    m = pattern.search(latest)
    if not m:
        return None
    g = m.groupdict()
    return (
        "shadow predictors (cumulative): "
        f"samples={g['samples']} latestErr={g['lm']}/{g['lp']} | "
        f"25% changed={g['c25']} changedWin={g['cw25']}% "
        f"adaptErr={g['am25']}/{g['ap25']} changedRatio={g['crm25']}/{g['crp25']} | "
        f"12.5% changed={g['c125']} changedWin={g['cw125']}% "
        f"adaptErr={g['am125']}/{g['ap125']} changedRatio={g['crm125']}/{g['crp125']} "
        "(med/p95; ratio<1 better)"
    )

class Capture:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.settings = parse_sysconfig(args.sysconfig)
        self.thresholds = parse_thresholds(args.thresholds)
        self.physical = find_physical(args.device)
        self.pointer = find_pointer(args.pointer_device)
        self.phys_assembler = PhysicalAssembler(self.physical)
        self.pointer_assembler = PointerAssembler() if self.pointer else None

        xi = self.physical.absinfo(ecodes.ABS_MT_POSITION_X)
        yi = self.physical.absinfo(ecodes.ABS_MT_POSITION_Y)
        self.geometry = {
            "x_min": xi.min,
            "x_max": xi.max,
            "x_resolution": xi.resolution,
            "y_min": yi.min,
            "y_max": yi.max,
            "y_resolution": yi.resolution,
        }

        stamp = _dt.datetime.now().strftime("%Y%m%d-%H%M%S")
        slug = re.sub(r"[^A-Za-z0-9_.-]+", "-", args.scenario).strip("-") or "capture"
        prefix = args.prefix or f"amt-liftoff-{slug}-{stamp}"
        outdir = Path(args.output_dir)
        outdir.mkdir(parents=True, exist_ok=True)
        self.json_path = outdir / f"{prefix}.jsonl"
        self.summary_path = outdir / f"{prefix}-summary.txt"
        self.json_f = self.json_path.open("w", encoding="utf-8", buffering=1)
        self.summary_f = self.summary_path.open("w", encoding="utf-8", buffering=1)

        self.attempt_gap_ns = int(args.attempt_gap_ms * 1_000_000.0)
        self.attempt_seq = 0
        self.attempt: Optional[Attempt] = None
        self.active_traces: dict[int, ContactTrace] = {}
        self.last_physical_ids: set[int] = set()
        self.stop = False

        self.journal: Optional[subprocess.Popen[str]] = None
        if not args.no_journal:
            try:
                self.journal = subprocess.Popen(
                    [
                        "journalctl",
                        "-u",
                        "amt-raw.service",
                        "-f",
                        "-n",
                        "0",
                        "-o",
                        "cat",
                    ],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.DEVNULL,
                    text=False,
                    bufsize=0,
                )
            except OSError as exc:
                print(f"warning: could not start journalctl: {exc}", file=sys.stderr)

        self._journal_buffer = b""

        # Optional second-stage observation: a separate libinput context over
        # AMT Raw Pointer.  This does not grab the device and lets a trace show
        # whether the kernel/uinput BTN events are also turned into libinput
        # POINTER_BUTTON events.  libinput debug-events is intentionally
        # treated as human-readable diagnostic output, not a stable parser API.
        self.libinput: Optional[subprocess.Popen[bytes]] = None
        self.libinput_command: Optional[list[str]] = None
        self._libinput_buffer = b""
        self._next_pointer_retry_ns = 0
        self._start_libinput()
        self._write_meta()

    def _stop_libinput(self) -> None:
        proc = self.libinput
        self.libinput = None
        self.libinput_command = None
        self._libinput_buffer = b""
        if proc is None:
            return
        try:
            proc.terminate()
            proc.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            proc.kill()
        except OSError:
            pass

    def _start_libinput(self) -> None:
        self._stop_libinput()
        if self.pointer is None or self.args.no_libinput:
            return
        libinput_bin = shutil.which("libinput")
        if libinput_bin is None:
            return
        command = [libinput_bin, "debug-events", "--verbose", "--device", self.pointer.path]
        stdbuf_bin = shutil.which("stdbuf")
        if stdbuf_bin is not None:
            command = [stdbuf_bin, "-oL", "-eL"] + command
        try:
            self.libinput = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=False,
                bufsize=0,
            )
            self.libinput_command = command
        except OSError as exc:
            print(f"warning: could not start libinput debug-events: {exc}", file=sys.stderr)

    def _drop_pointer(self, reason: str) -> None:
        old_path = self.pointer.path if self.pointer is not None else None
        self._stop_libinput()
        if self.pointer is not None:
            try:
                self.pointer.close()
            except Exception:
                pass
        self.pointer = None
        self.pointer_assembler = None
        self._next_pointer_retry_ns = time.monotonic_ns() + 250_000_000
        self._write_json({
            "type": "pointer_device_lost",
            "host_ns": time.monotonic_ns(),
            "path": old_path,
            "reason": reason,
        })
        self._write_summary(f"warning: AMT Raw Pointer disappeared ({reason}); continuing physical capture and waiting for replacement")

    def _try_reconnect_pointer(self, now_ns: int) -> None:
        if self.pointer is not None or now_ns < self._next_pointer_retry_ns:
            return
        try:
            # A virtual uinput device often returns under a different eventN after
            # daemon restart/reconnect.  Rediscover by name rather than pinning the
            # vanished event path unless the user explicitly requested a path.
            explicit = self.args.pointer_device
            candidate = find_pointer(explicit) if explicit else find_pointer(None)
        except OSError:
            candidate = None
        if candidate is None:
            self._next_pointer_retry_ns = now_ns + 250_000_000
            return
        self.pointer = candidate
        self.pointer_assembler = PointerAssembler()
        self._write_json({
            "type": "pointer_device_reconnected",
            "host_ns": now_ns,
            "path": candidate.path,
            "name": candidate.name,
        })
        self._write_summary(f"pointer: reconnected {candidate.path} ({candidate.name})")
        self._start_libinput()

    def _write_json(self, obj: dict[str, Any]) -> None:
        self.json_f.write(json.dumps(obj, separators=(",", ":")) + "\n")

    def _write_summary(self, line: str = "") -> None:
        self.summary_f.write(line + "\n")
        print(line)

    def _write_meta(self) -> None:
        meta = {
            "type": "meta",
            "script_version": SCRIPT_VERSION,
            "scenario": self.args.scenario,
            "started_local": _dt.datetime.now().astimezone().isoformat(),
            "physical": {
                "path": self.physical.path,
                "name": self.physical.name,
                "vendor": self.physical.info.vendor,
                "product": self.physical.info.product,
            },
            "pointer": None
            if self.pointer is None
            else {"path": self.pointer.path, "name": self.pointer.name},
            "geometry": self.geometry,
            "settings": self.settings,
            "offline_thresholds": self.thresholds,
            "attempt_gap_ms": self.args.attempt_gap_ms,
            "libinput_debug_events": self.libinput_command,
        }
        self._write_json(meta)
        self._write_summary(f"amt-liftoff-capture {SCRIPT_VERSION}")
        self._write_summary(f"scenario: {self.args.scenario}")
        self._write_summary(f"physical: {self.physical.path} ({self.physical.name})")
        if self.pointer:
            self._write_summary(f"pointer:  {self.pointer.path} ({self.pointer.name})")
        else:
            self._write_summary(
                "pointer:  NOT FOUND (raw capture will continue; synthetic click/motion evidence unavailable)"
            )
        if self.libinput is not None:
            self._write_summary("libinput: monitoring AMT Raw Pointer with debug-events --verbose")
        elif not self.args.no_libinput:
            self._write_summary("libinput: unavailable (capture continues without libinput channel)")
        else:
            self._write_summary("libinput: disabled by --no-libinput")
        self._write_summary(
            "offline pressure thresholds: " + ", ".join(map(str, self.thresholds))
        )
        self._write_summary(
            f"attempt grouping gap: {self.args.attempt_gap_ms:.0f} ms; pause longer than this between attempts"
        )
        log_level = self.settings.get("AMT_LOG_LEVEL", "info").strip().lower()
        if _setting_float(self.settings, "AMT_RESAMPLING_RATE_HZ", 120.0) > 0.0:
            predictor = self.settings.get("AMT_RESAMPLING_PREDICTOR", "interpolated").strip().lower() or "interpolated"
            self._write_summary(f"resampling predictor: {predictor}")
            if log_level in {"debug", "info"}:
                self._write_summary(
                    f"dual-shadow stats: AMT_LOG_LEVEL={log_level} (periodic INFO summaries enabled)"
                )
            else:
                self._write_summary(
                    f"dual-shadow stats: AMT_LOG_LEVEL={log_level}; set AMT_LOG_LEVEL=info or debug to capture periodic dual-shadow summaries"
                )
        self._write_summary(f"raw log:   {self.json_path}")
        self._write_summary(f"summary:   {self.summary_path}")
        self._write_summary("Ctrl-C to finish.\n")

    def _start_attempt(self, host_ns: int) -> None:
        self.attempt_seq += 1
        self.attempt = Attempt(
            attempt_id=self.attempt_seq,
            scenario=self.args.scenario,
            start_ns=host_ns,
            last_activity_ns=host_ns,
        )
        self._write_json(
            {
                "type": "attempt_start",
                "attempt": self.attempt_seq,
                "host_ns": host_ns,
                "scenario": self.args.scenario,
            }
        )
        self._write_summary(f"Attempt {self.attempt_seq} started")

    def _finalize_attempt(self, host_ns: int, reason: str) -> None:
        a = self.attempt
        if a is None:
            return
        a.ended_ns = host_ns
        # Any still-active contacts are retained, but ordinary idle-gap endings
        # will have finalized all of them already.
        for fid, trace in list(self.active_traces.items()):
            if trace.attempt_id == a.attempt_id:
                trace.lifted_ns = host_ns
                a.contacts.append(trace)
                self.active_traces.pop(fid, None)

        contact_summaries = [
            contact_summary(
                c,
                settings=self.settings,
                thresholds=self.thresholds,
                y_max=self.geometry["y_max"],
                y_resolution=self.geometry["y_resolution"],
            )
            for c in a.contacts
        ]
        downs = sum(
            1
            for ev in a.pointer_button_events
            if ev["code"] == ecodes.BTN_LEFT and ev["value"] == 1
        )
        ups = sum(
            1
            for ev in a.pointer_button_events
            if ev["code"] == ecodes.BTN_LEFT and ev["value"] == 0
        )
        left_presses = [
            ev for ev in a.pointer_button_events
            if ev["code"] == ecodes.BTN_LEFT and ev["value"] == 1
        ]
        press_gaps_ms = [
            round((b["host_ns"] - aa["host_ns"]) / 1_000_000.0, 3)
            for aa, b in zip(left_presses, left_presses[1:])
        ]
        press_position_deltas = [
            {
                "dx": b["rel_pos_x"] - aa["rel_pos_x"],
                "dy": b["rel_pos_y"] - aa["rel_pos_y"],
                "distance": round(
                    math.hypot(
                        b["rel_pos_x"] - aa["rel_pos_x"],
                        b["rel_pos_y"] - aa["rel_pos_y"],
                    ),
                    3,
                ),
            }
            for aa, b in zip(left_presses, left_presses[1:])
        ]
        motion_intervals_ms = [
            (b - aa) / 1_000_000.0
            for aa, b in zip(a.pointer_motion_times_ns, a.pointer_motion_times_ns[1:])
        ]
        if motion_intervals_ms:
            ordered_motion_intervals = sorted(motion_intervals_ms)
            motion_median_ms = statistics.median(ordered_motion_intervals)
            motion_p95_ms = ordered_motion_intervals[
                min(
                    len(ordered_motion_intervals) - 1,
                    max(0, math.ceil(0.95 * len(ordered_motion_intervals)) - 1),
                )
            ]
            motion_rate_hz = 1000.0 / motion_median_ms if motion_median_ms > 0 else 0.0
        else:
            motion_median_ms = None
            motion_p95_ms = None
            motion_rate_hz = None

        summary = {
            "type": "attempt_end",
            "attempt": a.attempt_id,
            "scenario": a.scenario,
            "host_ns": host_ns,
            "reason": reason,
            "duration_ms": round((host_ns - a.start_ns) / 1_000_000.0, 3),
            "pointer": {
                "rel_x": a.pointer_rel_x,
                "rel_y": a.pointer_rel_y,
                "wheel_hi_res": a.pointer_wheel_hi_res,
                "hwheel_hi_res": a.pointer_hwheel_hi_res,
                "btn_left_down": downs,
                "btn_left_up": ups,
                "frames": a.pointer_frames,
                "motion_frames": len(a.pointer_motion_times_ns),
                "motion_interval_median_ms": (
                    None if motion_median_ms is None else round(motion_median_ms, 4)
                ),
                "motion_interval_p95_ms": (
                    None if motion_p95_ms is None else round(motion_p95_ms, 4)
                ),
                "motion_rate_from_median_hz": (
                    None if motion_rate_hz is None else round(motion_rate_hz, 3)
                ),
                "button_events": a.pointer_button_events,
                "left_press_gaps_ms": press_gaps_ms,
                "left_press_position_deltas": press_position_deltas,
            },
            "contacts": contact_summaries,
            "journal": a.journal_lines,
            "libinput_pointer_buttons": [
                line for line in a.libinput_lines if "POINTER_BUTTON" in line
            ],
            "libinput_pointer_scroll": [
                line for line in a.libinput_lines
                if "POINTER_SCROLL" in line or "POINTER_AXIS" in line
            ],
        }
        self._write_json(summary)
        extra = ""
        if press_gaps_ms:
            extra += " pressGap=" + ",".join(f"{x:.1f}ms" for x in press_gaps_ms)
        if press_position_deltas:
            extra += " pressMove=" + ",".join(
                f"({d['dx']},{d['dy']};{d['distance']:.1f}px)"
                for d in press_position_deltas
            )
        self._write_summary(
            f"Attempt {a.attempt_id} ended: contacts={len(contact_summaries)} "
            f"REL=({a.pointer_rel_x},{a.pointer_rel_y}) "
            f"WHEEL_HI=(v={a.pointer_wheel_hi_res},h={a.pointer_hwheel_hi_res}) "
            f"BTN_LEFT down/up={downs}/{ups}"
            f"{extra} reason={reason}"
        )
        if motion_median_ms is not None:
            self._write_summary(
                f"  REL cadence: motionFrames={len(a.pointer_motion_times_ns)} "
                f"median={motion_median_ms:.3f}ms p95={motion_p95_ms:.3f}ms "
                f"rate~{motion_rate_hz:.2f}Hz"
            )
        for cs in contact_summaries:
            self._write_summary(short_contact_line(cs, self.thresholds))
        shadow_summary = _shadow_stats_summary(a.journal_lines)
        if shadow_summary is not None:
            self._write_summary("  " + shadow_summary)
        for line in a.journal_lines:
            self._write_summary("  log: " + line)
        for line in a.libinput_lines:
            if (
                "POINTER_BUTTON" in line
                or "POINTER_SCROLL" in line
                or "POINTER_AXIS" in line
            ):
                self._write_summary("  libinput: " + line)
        self._write_summary()
        self.attempt = None
        self.last_physical_ids.clear()

    def _maybe_roll_attempt_before_frame(self, frame: dict[str, Any]) -> None:
        host_ns = frame["host_ns"]
        touches = frame["touches"]
        if self.attempt is not None and self.attempt.idle_since_ns is not None and touches:
            if host_ns - self.attempt.idle_since_ns >= self.attempt_gap_ns:
                self._finalize_attempt(self.attempt.idle_since_ns + self.attempt_gap_ns, "idle-gap")
        if self.attempt is None and touches:
            self._start_attempt(host_ns)

    def handle_physical_frame(self, frame: dict[str, Any]) -> None:
        self._maybe_roll_attempt_before_frame(frame)
        a = self.attempt
        frame["attempt"] = None if a is None else a.attempt_id
        self._write_json(frame)

        current = {t["id"]: t for t in frame["touches"]}
        current_ids = set(current)
        removed_ids = self.last_physical_ids - current_ids

        if a is not None:
            a.last_activity_ns = frame["host_ns"]
            if current_ids:
                a.idle_since_ns = None

            for fid in current_ids:
                touch = current[fid]
                trace = self.active_traces.get(fid)
                if trace is None:
                    trace = ContactTrace(
                        finger_id=fid,
                        slot=touch["slot"],
                        attempt_id=a.attempt_id,
                    )
                    self.active_traces[fid] = trace
                sample = dict(touch)
                sample["host_ns"] = frame["host_ns"]
                sample["evdev_s"] = frame["evdev_s"]
                sample["button_left"] = frame["button_left"]
                trace.samples.append(sample)

            for fid in removed_ids:
                trace = self.active_traces.pop(fid, None)
                if trace is not None:
                    trace.lifted_ns = frame["host_ns"]
                    trace.lifted_evdev_s = frame["evdev_s"]
                    a.contacts.append(trace)

            if not current_ids and self.last_physical_ids:
                a.idle_since_ns = frame["host_ns"]

        self.last_physical_ids = current_ids

    def handle_pointer_frame(self, record: dict[str, Any]) -> None:
        a = self.attempt
        record["attempt"] = None if a is None else a.attempt_id
        self._write_json(record)
        if a is None:
            return
        a.pointer_frames += 1
        dx = int(record["rel"]["x"])
        dy = int(record["rel"]["y"])
        a.pointer_rel_x += dx
        a.pointer_rel_y += dy
        if dx or dy:
            a.pointer_motion_times_ns.append(int(record["host_ns"]))
        a.pointer_wheel_hi_res += int(record["rel"].get("wheel_hi_res", 0))
        a.pointer_hwheel_hi_res += int(record["rel"].get("hwheel_hi_res", 0))
        a.pointer_pos_x += dx
        a.pointer_pos_y += dy
        for ev in record["keys"]:
            if ev["code"] in {
                ecodes.BTN_LEFT,
                ecodes.BTN_RIGHT,
                ecodes.BTN_MIDDLE,
            }:
                item = {
                    "host_ns": record["host_ns"],
                    "code": ev["code"],
                    "value": ev["value"],
                    "rel_pos_x": a.pointer_pos_x,
                    "rel_pos_y": a.pointer_pos_y,
                }
                a.pointer_button_events.append(item)

    def handle_journal_line(self, line: str, host_ns: int) -> None:
        line = line.rstrip("\n")
        if not line or not RELEVANT_JOURNAL.search(line):
            return
        a = self.attempt
        record = {
            "type": "journal",
            "host_ns": host_ns,
            "attempt": None if a is None else a.attempt_id,
            "line": line,
        }
        self._write_json(record)
        if a is not None:
            a.journal_lines.append(line)

    def handle_libinput_line(self, line: str, host_ns: int) -> None:
        line = line.rstrip("\n")
        if not line:
            return
        a = self.attempt
        record = {
            "type": "libinput",
            "host_ns": host_ns,
            "attempt": None if a is None else a.attempt_id,
            "line": line,
        }
        self._write_json(record)
        if a is not None:
            a.libinput_lines.append(line)

    def check_idle(self, now_ns: int) -> None:
        if (
            self.attempt is not None
            and self.attempt.idle_since_ns is not None
            and now_ns - self.attempt.idle_since_ns >= self.attempt_gap_ns
        ):
            self._finalize_attempt(
                self.attempt.idle_since_ns + self.attempt_gap_ns,
                "idle-gap",
            )

    def run(self) -> None:
        physical_fd = self.physical.fd
        journal_fd = (
            self.journal.stdout.fileno()
            if self.journal is not None and self.journal.stdout is not None
            else None
        )

        try:
            while not self.stop:
                now_ns = time.monotonic_ns()
                self._try_reconnect_pointer(now_ns)
                pointer_fd = self.pointer.fd if self.pointer is not None else None
                libinput_fd = (
                    self.libinput.stdout.fileno()
                    if self.libinput is not None and self.libinput.stdout is not None
                    else None
                )

                fds = [physical_fd]
                if pointer_fd is not None:
                    fds.append(pointer_fd)
                if journal_fd is not None:
                    fds.append(journal_fd)
                if libinput_fd is not None:
                    fds.append(libinput_fd)

                try:
                    readable, _, _ = select.select(fds, [], [], 0.050)
                except OSError as exc:
                    if exc.errno == errno.ENODEV and self.pointer is not None:
                        self._drop_pointer("select ENODEV")
                        continue
                    raise
                now_ns = time.monotonic_ns()

                for fd in readable:
                    if fd == physical_fd:
                        try:
                            events = self.physical.read()
                        except BlockingIOError:
                            events = []
                        for event in events:
                            host_ns = time.monotonic_ns()
                            frame = self.phys_assembler.update(event, host_ns)
                            if frame is not None:
                                self.handle_physical_frame(frame)
                    elif pointer_fd is not None and fd == pointer_fd:
                        try:
                            events = self.pointer.read() if self.pointer else []
                        except BlockingIOError:
                            events = []
                        except OSError as exc:
                            if exc.errno == errno.ENODEV:
                                self._drop_pointer(f"read ENODEV on {getattr(self.pointer, 'path', '?')}")
                                events = []
                            else:
                                raise
                        for event in events:
                            host_ns = time.monotonic_ns()
                            rec = self.pointer_assembler.update(event, host_ns) if self.pointer_assembler else None
                            if rec is not None:
                                self.handle_pointer_frame(rec)
                    elif journal_fd is not None and fd == journal_fd:
                        assert self.journal is not None and self.journal.stdout is not None
                        chunk = os.read(journal_fd, 65536)
                        if chunk:
                            self._journal_buffer += chunk
                            while b"\n" in self._journal_buffer:
                                raw_line, self._journal_buffer = self._journal_buffer.split(b"\n", 1)
                                line = raw_line.decode("utf-8", errors="replace")
                                self.handle_journal_line(line, time.monotonic_ns())
                    elif libinput_fd is not None and fd == libinput_fd:
                        assert self.libinput is not None and self.libinput.stdout is not None
                        chunk = os.read(libinput_fd, 65536)
                        if chunk:
                            self._libinput_buffer += chunk
                            while b"\n" in self._libinput_buffer:
                                raw_line, self._libinput_buffer = self._libinput_buffer.split(b"\n", 1)
                                line = raw_line.decode("utf-8", errors="replace")
                                self.handle_libinput_line(line, time.monotonic_ns())
                        else:
                            # libinput normally exits when its --device node is
                            # destroyed.  The pointer reconnect path will relaunch
                            # it for the replacement event node.
                            self._stop_libinput()

                self.check_idle(now_ns)
        except KeyboardInterrupt:
            pass
        finally:
            now_ns = time.monotonic_ns()
            if self.attempt is not None:
                self._finalize_attempt(now_ns, "capture-ended")
            self.close()

    def close(self) -> None:
        self._stop_libinput()
        if self.journal is not None:
            self.journal.terminate()
            try:
                self.journal.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self.journal.kill()
        try:
            self.physical.close()
        except Exception:
            pass
        if self.pointer is not None:
            try:
                self.pointer.close()
            except Exception:
                pass
        self.json_f.close()
        self.summary_f.close()

        # Captures are normally run with sudo because /dev/input/event* may not
        # be user-readable.  Hand ownership back to the invoking user so the
        # files can be inspected/uploaded without sudo.
        sudo_uid = os.environ.get("SUDO_UID")
        sudo_gid = os.environ.get("SUDO_GID")
        if sudo_uid is not None and sudo_gid is not None:
            try:
                uid, gid = int(sudo_uid), int(sudo_gid)
                os.chown(self.json_path, uid, gid)
                os.chown(self.summary_path, uid, gid)
            except (OSError, ValueError):
                pass

        print(f"\nWrote {self.json_path}")
        print(f"Wrote {self.summary_path}")


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Capture synchronized raw AMT + amt-raw pointer traces for lift-off/tap calibration"
    )
    p.add_argument(
        "--scenario",
        required=True,
        help="label for this run, e.g. position-release, position-tap, position-double-tap",
    )
    p.add_argument("--device", default="auto", help="physical event node or auto (default)")
    p.add_argument(
        "--pointer-device",
        default=None,
        help="AMT Raw Pointer event node; default finds it by name",
    )
    p.add_argument(
        "--sysconfig",
        default="/etc/sysconfig/amt-raw",
        help="amt-raw sysconfig to snapshot (default: /etc/sysconfig/amt-raw)",
    )
    p.add_argument(
        "--thresholds",
        default="6,8,10,12,14,16",
        help="offline lift-off pressure thresholds to simulate (default: 6,8,10,12,14,16)",
    )
    p.add_argument(
        "--attempt-gap-ms",
        type=float,
        default=1200.0,
        help="all-contacts-up gap that ends one attempt (default: 1200 ms)",
    )
    p.add_argument("--output-dir", default=".", help="capture output directory")
    p.add_argument("--prefix", default=None, help="explicit output filename prefix")
    p.add_argument(
        "--no-journal",
        action="store_true",
        help="do not collect selected amt-raw.service debug messages",
    )
    p.add_argument(
        "--no-libinput",
        action="store_true",
        help="do not run libinput debug-events on AMT Raw Pointer",
    )
    return p


def main() -> int:
    args = build_parser().parse_args()
    if args.attempt_gap_ms <= 0:
        raise SystemExit("--attempt-gap-ms must be > 0")
    try:
        cap = Capture(args)
    except (OSError, RuntimeError, ValueError) as exc:
        raise SystemExit(f"capture setup failed: {exc}") from exc
    cap.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
