#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import logging
import os
import shutil
import signal
import subprocess
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

LOG = logging.getLogger("amt-raw-cinnamon-sync")

SCHEMA = "org.cinnamon.desktop.peripherals.touchpad"
SEND_EVENTS_KEY = "send-events"
VALID_SEND_EVENTS = {
    "enabled",
    "disabled",
    "disabled-on-external-mouse",
}

PHYSICAL_NAME = "Apple Inc. Magic Trackpad"
TOUCHPAD_NAME = "AMT Raw Touchpad"
POINTER_NAME = "AMT Raw Pointer"
SCROLL_NAME = "AMT Raw Touchpad Scroll"
KINETIC_NAME = "AMT Raw Touchpad Kinetic"


def _desktop_tokens() -> tuple[str, ...]:
    values = []
    for key in ("XDG_CURRENT_DESKTOP", "XDG_SESSION_DESKTOP", "DESKTOP_SESSION"):
        value = os.environ.get(key, "")
        if value:
            values.extend(part.strip().upper() for part in value.replace(":", ";").split(";") if part.strip())
    return tuple(values)


def is_cinnamon_session() -> bool:
    return any("CINNAMON" in token for token in _desktop_tokens())


def find_gsettings(command: Optional[str] = None) -> str:
    if command:
        resolved = shutil.which(command) if os.sep not in command else command
        if resolved and os.path.isfile(resolved) and os.access(resolved, os.X_OK):
            return resolved
        raise RuntimeError(f"configured gsettings executable is not usable: {command}")

    for path in ("/usr/bin/gsettings", "/bin/gsettings"):
        if os.path.isfile(path) and os.access(path, os.X_OK):
            return path
    resolved = shutil.which("gsettings")
    if resolved:
        return resolved
    raise RuntimeError(
        "Cinnamon routing requires the gsettings command but it was not found. "
        "On Fedora it is provided by GLib (glib2)."
    )


class GSettingsTouchpad:
    def __init__(self, command: Optional[str] = None):
        self.command = find_gsettings(command)
        LOG.info("Using gsettings executable: %s", self.command)

    def _run(self, *args: str) -> str:
        proc = subprocess.run(
            [self.command, *args],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
        if proc.returncode != 0:
            detail = proc.stderr.strip() or proc.stdout.strip() or f"exit {proc.returncode}"
            raise RuntimeError(detail)
        return proc.stdout.strip()

    def get_send_events(self) -> str:
        out = self._run("get", SCHEMA, SEND_EVENTS_KEY).strip()
        # Enum values are normally printed as 'enabled'. Be tolerant of raw
        # unquoted output from test shims or future gsettings formatting.
        value = out.strip().strip("'\"")
        if value not in VALID_SEND_EVENTS:
            raise RuntimeError(
                f"unexpected {SCHEMA} {SEND_EVENTS_KEY} value: {out!r}"
            )
        return value

    def set_send_events(self, value: str) -> None:
        if value not in VALID_SEND_EVENTS:
            raise ValueError(f"invalid Cinnamon touchpad send-events value: {value}")
        self._run("set", SCHEMA, SEND_EVENTS_KEY, value)
        LOG.info("Cinnamon touchpad send-events -> %s", value)


@dataclass(frozen=True)
class Topology:
    physical_count: int
    pointer_count: int
    touchpad_count: int
    scroll_count: int
    kinetic_count: int
    extra_touchpad_names: tuple[str, ...]

    @property
    def mode(self) -> str:
        if self.pointer_count and not self.touchpad_count:
            return "pointer" if self.physical_count else "pointer-no-source"
        if self.touchpad_count and not self.pointer_count:
            return "touchpad" if self.physical_count else "touchpad-no-source"
        if not self.pointer_count and not self.touchpad_count:
            return "native"
        return "conflict"


def _read_text(path: Path) -> Optional[str]:
    try:
        return path.read_text(errors="replace").strip()
    except OSError:
        return None


def scan_topology(event_root: Path) -> Topology:
    names: list[str] = []
    for event in sorted(event_root.glob("event*")):
        # /sys/class/input/eventN/device/name in production. Tests may point
        # event_root at a synthetic tree with the same shape.
        name = _read_text(event / "device" / "name")
        if name:
            names.append(name)

    known = {
        PHYSICAL_NAME,
        TOUCHPAD_NAME,
        POINTER_NAME,
        SCROLL_NAME,
        KINETIC_NAME,
    }
    extras = sorted(
        {
            name
            for name in names
            if name not in known
            and ("touchpad" in name.lower() or "trackpad" in name.lower())
        }
    )

    return Topology(
        physical_count=sum(1 for n in names if n == PHYSICAL_NAME),
        pointer_count=sum(1 for n in names if n == POINTER_NAME),
        touchpad_count=sum(1 for n in names if n == TOUCHPAD_NAME),
        scroll_count=sum(1 for n in names if n == SCROLL_NAME),
        kinetic_count=sum(1 for n in names if n == KINETIC_NAME),
        extra_touchpad_names=tuple(extras),
    )


class SavedState:
    def __init__(self, path: Path):
        self.path = path

    def load(self) -> Optional[str]:
        try:
            data = json.loads(self.path.read_text())
        except FileNotFoundError:
            return None
        except (OSError, json.JSONDecodeError) as exc:
            LOG.warning("Unable to read Cinnamon routing state %s: %s", self.path, exc)
            return None

        value = data.get("send_events") if isinstance(data, dict) else None
        if value not in VALID_SEND_EVENTS:
            LOG.warning("Ignoring invalid Cinnamon routing state in %s", self.path)
            return None
        return value

    def save(self, value: str) -> None:
        if value not in VALID_SEND_EVENTS:
            raise ValueError(value)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        payload = json.dumps({"send_events": value}, sort_keys=True) + "\n"
        fd, tmp_name = tempfile.mkstemp(
            prefix=self.path.name + ".",
            dir=str(self.path.parent),
            text=True,
        )
        try:
            with os.fdopen(fd, "w") as f:
                f.write(payload)
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp_name, self.path)
        finally:
            try:
                os.unlink(tmp_name)
            except FileNotFoundError:
                pass

    def clear(self) -> None:
        try:
            self.path.unlink()
        except FileNotFoundError:
            pass


class Synchronizer:
    def __init__(self, settings: GSettingsTouchpad, saved: SavedState):
        self.settings = settings
        self.saved = saved
        self._last_summary: Optional[tuple] = None
        self._warned_extra: Optional[tuple[str, ...]] = None

    def _engage_pointer_routing(self) -> None:
        baseline = self.saved.load()
        if baseline is None:
            baseline = self.settings.get_send_events()
            # Save before changing GSettings so a crash between the two leaves
            # enough information to restore the user's setting next time.
            self.saved.save(baseline)
            LOG.info("Saved pre-amt-raw Cinnamon touchpad send-events=%s", baseline)

        current = self.settings.get_send_events()
        if current != "disabled":
            self.settings.set_send_events("disabled")

    def restore(self) -> None:
        baseline = self.saved.load()
        if baseline is None:
            return
        try:
            current = self.settings.get_send_events()
            if current != baseline:
                self.settings.set_send_events(baseline)
            LOG.info("Restored pre-amt-raw Cinnamon touchpad send-events=%s", baseline)
        except RuntimeError:
            # Keep the state file so the next successful helper start can
            # restore the user's value.
            raise
        else:
            self.saved.clear()

    def sync_once(self, topology: Topology) -> None:
        mode = topology.mode
        summary = (
            mode,
            topology.physical_count,
            topology.pointer_count,
            topology.touchpad_count,
            topology.scroll_count,
            topology.kinetic_count,
            topology.extra_touchpad_names,
        )

        if topology.extra_touchpad_names and topology.extra_touchpad_names != self._warned_extra:
            LOG.warning(
                "Cinnamon exposes touchpad send-events globally; pointer mode will also disable other touchpads in Cinnamon while amt-raw is active: %s",
                ", ".join(topology.extra_touchpad_names),
            )
            self._warned_extra = topology.extra_touchpad_names

        if mode == "pointer":
            self._engage_pointer_routing()
        else:
            # Cinnamon's public setting is global to the touchpad class. Restoring
            # it is therefore the only safe choice outside pointer mode.
            self.restore()

        if summary == self._last_summary:
            return

        if mode == "pointer":
            LOG.info(
                "running mode=pointer; AMT Raw Pointer present and physical Magic Trackpad hidden from Cinnamon via Cinnamon touchpad send-events=disabled"
            )
        elif mode == "native":
            LOG.info("running mode=native/failsafe; Cinnamon touchpad setting restored")
        elif mode == "touchpad":
            LOG.error(
                "running mode=touchpad is not routable through Cinnamon's public settings: send-events is global and would disable AMT Raw Touchpad together with the physical device; restored the user's setting instead"
            )
        elif mode == "pointer-no-source":
            LOG.warning(
                "AMT Raw Pointer is present but the physical Magic Trackpad is absent; restored the user's Cinnamon touchpad setting until the source returns"
            )
        elif mode == "touchpad-no-source":
            LOG.warning(
                "AMT Raw Touchpad is present but the physical Magic Trackpad is absent; restored the user's Cinnamon touchpad setting"
            )
        else:
            LOG.warning(
                "both AMT Raw Pointer and AMT Raw Touchpad are live; Cinnamon routing left at the user's original touchpad setting"
            )

        self._last_summary = summary


def default_state_path() -> Path:
    state_home = os.environ.get("XDG_STATE_HOME")
    if state_home:
        return Path(state_home) / "amt-raw" / "cinnamon-sync.json"
    return Path.home() / ".local" / "state" / "amt-raw" / "cinnamon-sync.json"


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description=(
            "Synchronize Cinnamon touchpad routing with the actually running amt-raw backend. "
            "Pointer mode is supported; Cinnamon's public touchpad setting cannot safely route amt-raw touchpad mode per device."
        )
    )
    p.add_argument("--poll-seconds", type=float, default=0.10,
                   help="evdev/sysfs topology polling interval (default: 0.10)")
    p.add_argument("--reconcile-seconds", type=float, default=1.0,
                   help="while pointer mode is active, re-check Cinnamon send-events at this interval (default: 1.0)")
    p.add_argument("--gsettings", default=None,
                   help="override gsettings executable (primarily for testing)")
    p.add_argument("--event-root", type=Path, default=Path("/sys/class/input"),
                   help=argparse.SUPPRESS)
    p.add_argument("--state-path", type=Path, default=None,
                   help=argparse.SUPPRESS)
    p.add_argument("--force", action="store_true",
                   help="run even when the current desktop session does not identify as Cinnamon")
    p.add_argument("--once", action="store_true", help="run one reconciliation pass and exit")
    p.add_argument("--log-level", choices=("warning", "info", "debug"), default="info")
    return p


def main() -> int:
    args = build_parser().parse_args()
    if args.poll_seconds <= 0:
        raise SystemExit("--poll-seconds must be positive")
    if args.reconcile_seconds <= 0:
        raise SystemExit("--reconcile-seconds must be positive")

    logging.basicConfig(
        level={"warning": logging.WARNING, "info": logging.INFO, "debug": logging.DEBUG}[args.log_level],
        format="%(asctime)s %(levelname)-8s %(name)s: %(message)s",
    )

    if not args.force and not is_cinnamon_session():
        LOG.info(
            "Cinnamon session not detected (desktop=%s); exiting without changing input settings",
            ":".join(_desktop_tokens()) or "unknown",
        )
        return 0

    try:
        settings = GSettingsTouchpad(args.gsettings)
        # Fail early with a useful schema/session-bus error rather than after
        # the first topology transition.
        settings.get_send_events()
    except RuntimeError as exc:
        LOG.error("Cinnamon GSettings routing unavailable: %s", exc)
        return 1

    state_path = args.state_path or default_state_path()
    sync = Synchronizer(settings, SavedState(state_path))

    # Recover from a previous crash/logout that left the persistent setting
    # changed, unless a live pointer backend still needs that routing now.
    topology = scan_topology(args.event_root)
    try:
        sync.sync_once(topology)
    except RuntimeError as exc:
        LOG.error("Cinnamon input routing failed: %s", exc)
        return 1

    if args.once:
        return 0

    stop = False

    def handle_signal(_signum, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    last_topology = topology
    last_error: Optional[str] = None
    next_reconcile = time.monotonic() + args.reconcile_seconds
    while not stop:
        time.sleep(args.poll_seconds)
        topology = scan_topology(args.event_root)
        now = time.monotonic()
        topology_changed = topology != last_topology
        periodic_pointer_check = topology.mode == "pointer" and now >= next_reconcile
        if not topology_changed and not periodic_pointer_check:
            continue
        last_topology = topology
        if periodic_pointer_check or topology_changed:
            next_reconcile = now + args.reconcile_seconds
        try:
            sync.sync_once(topology)
            last_error = None
        except RuntimeError as exc:
            msg = str(exc)
            if msg != last_error:
                LOG.warning("Cinnamon input routing unavailable: %s", msg)
                last_error = msg

    try:
        sync.restore()
    except RuntimeError as exc:
        LOG.warning(
            "Unable to restore Cinnamon touchpad setting during helper shutdown: %s; saved state retained at %s",
            exc,
            state_path,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
