#!/usr/bin/env python3
from __future__ import annotations

import argparse
import logging
import os
import shutil
import signal
import subprocess
import time
from pathlib import Path
from typing import Dict, Iterable, Optional

LOG = logging.getLogger("amt-raw-kde-sync")

KWIN_SERVICE = "org.kde.KWin"
INPUT_IFACE = "org.kde.KWin.InputDevice"
PROPS_IFACE = "org.freedesktop.DBus.Properties"
INPUT_PREFIX = "/org/kde/KWin/InputDevice/event"

PHYSICAL_NAME = "Apple Inc. Magic Trackpad"
TOUCHPAD_NAME = "AMT Raw Touchpad"
POINTER_NAME = "AMT Raw Pointer"
SCROLL_NAME = "AMT Raw Touchpad Scroll"
KINETIC_NAME = "AMT Raw Touchpad Kinetic"

QDBUS_COMMANDS = ("qdbus-qt6", "qdbus6", "qdbus")
QDBUS_COMMON_PATHS = (
    "/usr/bin/qdbus-qt6",
    "/usr/bin/qdbus6",
    "/usr/bin/qdbus",
    "/usr/lib64/qt6/bin/qdbus",
    "/usr/lib/qt6/bin/qdbus",
)


def find_qdbus(command: Optional[str] = None) -> str:
    if command:
        resolved = shutil.which(command) if os.sep not in command else command
        if resolved and os.path.isfile(resolved) and os.access(resolved, os.X_OK):
            return resolved
        raise RuntimeError(f"configured qdbus executable is not usable: {command}")

    for path in QDBUS_COMMON_PATHS:
        if os.path.isfile(path) and os.access(path, os.X_OK):
            return path
    for name in QDBUS_COMMANDS:
        resolved = shutil.which(name)
        if resolved:
            return resolved

    raise RuntimeError(
        "Qt 6 qdbus is required for KDE/Plasma device routing but was not found. "
        "On Fedora install qt6-qttools (provides qdbus-qt6), then restart "
        "systemctl --user restart amt-raw-kde-sync.service."
    )


class QDbusKWin:
    def __init__(self, command: Optional[str] = None):
        self.command = find_qdbus(command)
        LOG.info("Using qdbus executable: %s", self.command)
        self._names: Dict[str, str] = {}

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

    def paths(self) -> list[str]:
        out = self._run(KWIN_SERVICE)
        paths = sorted(
            line.strip()
            for line in out.splitlines()
            if line.strip().startswith(INPUT_PREFIX)
        )
        current = set(paths)
        self._names = {p: n for p, n in self._names.items() if p in current}
        return paths

    def get(self, path: str, prop: str) -> str:
        return self._run(
            KWIN_SERVICE,
            path,
            f"{PROPS_IFACE}.Get",
            INPUT_IFACE,
            prop,
        ).strip()

    def name(self, path: str) -> str:
        if path not in self._names:
            self._names[path] = self.get(path, "name")
        return self._names[path]

    def set_bool_property(self, path: str, prop: str, value: bool) -> None:
        self._run(
            KWIN_SERVICE,
            path,
            f"{PROPS_IFACE}.Set",
            INPUT_IFACE,
            prop,
            "true" if value else "false",
        )

    def set_enabled(self, path: str, value: bool) -> None:
        self.set_bool_property(path, "enabled", value)
        LOG.info("KWin %s: %s", "enabled" if value else "disabled", self.name(path))


class Synchronizer:
    def __init__(self, qdbus: QDbusKWin):
        self.qdbus = qdbus
        self._last_summary: Optional[tuple] = None

    @staticmethod
    def _event_node(path: str) -> Optional[Path]:
        leaf = path.rsplit("/", 1)[-1]
        if not leaf.startswith("event"):
            return None
        return Path("/dev/input") / leaf

    @classmethod
    def _path_is_live(cls, path: str) -> bool:
        """Ignore a KWin object whose backing /dev/input/eventN has vanished."""
        node = cls._event_node(path)
        return node is not None and node.exists()

    def _named_paths(self) -> Dict[str, list[str]]:
        groups: Dict[str, list[str]] = {
            PHYSICAL_NAME: [],
            TOUCHPAD_NAME: [],
            POINTER_NAME: [],
            SCROLL_NAME: [],
            KINETIC_NAME: [],
        }
        for path in self.qdbus.paths():
            if not self._path_is_live(path):
                LOG.debug("Ignoring stale KWin input object with no live event node: %s", path)
                continue
            try:
                name = self.qdbus.name(path)
            except RuntimeError as exc:
                LOG.debug("Unable to read KWin device name for %s: %s", path, exc)
                continue
            if name in groups:
                groups[name].append(path)
        return groups

    @staticmethod
    def _set_many(qdbus: QDbusKWin, paths: Iterable[str], value: bool) -> None:
        for path in paths:
            try:
                qdbus.set_enabled(path, value)
            except RuntimeError as exc:
                LOG.warning("Unable to set KWin device %s enabled=%s: %s", path, value, exc)


    def _ensure_touchpad_native_scroll_disabled(self, paths: Iterable[str]) -> None:
        """Keep KWin/libinput from emitting a duplicate touchpad scroll stream."""
        for path in paths:
            for prop in ("scrollTwoFinger", "scrollEdge"):
                try:
                    current = self.qdbus.get(path, prop).strip().lower()
                    if current not in ("false", "0"):
                        self.qdbus.set_bool_property(path, prop, False)
                        LOG.info("KWin %s disabled: %s", prop, self.qdbus.name(path))
                except RuntimeError as exc:
                    LOG.warning("Unable to reconcile KWin %s on %s: %s", prop, path, exc)

    def sync_once(self) -> None:
        groups = self._named_paths()
        physical = groups[PHYSICAL_NAME]
        touchpads = groups[TOUCHPAD_NAME]
        pointers = groups[POINTER_NAME]
        scroll = groups[SCROLL_NAME]
        kinetic = groups[KINETIC_NAME]

        # The helper intentionally follows the *running* daemon state, not the
        # desired AMT_MODE written in /etc/sysconfig/amt-raw.  systemd only
        # applies EnvironmentFile changes when amt-raw.service is restarted,
        # and KWin should not switch early merely because the file was edited.
        #
        # Presence of the corresponding live uinput device is the authoritative
        # signal that an enhanced backend is actually running.  No replacement
        # means native mode or a stopped/restarting/crashed daemon; both safely
        # require the physical Magic Trackpad to remain enabled.
        if touchpads and not pointers:
            mode = "touchpad"
            replacement = touchpads
            other = pointers
        elif pointers and not touchpads:
            mode = "pointer"
            replacement = pointers
            other = touchpads
        elif not touchpads and not pointers:
            mode = "native"
            replacement = []
            other = []
        else:
            mode = "conflict"
            replacement = []
            other = touchpads + pointers

        # Native touchpad scrolling must stay off for as long as touchpad mode
        # is active; System Settings may otherwise re-enable it while the daemon
        # is running and produce duplicate scroll streams.
        if mode == "touchpad":
            self._ensure_touchpad_native_scroll_disabled(touchpads)

        summary = (
            mode,
            tuple(physical),
            tuple(touchpads),
            tuple(pointers),
            tuple(scroll),
            tuple(kinetic),
        )
        if summary == self._last_summary:
            return

        if mode == "native":
            self._set_many(self.qdbus, scroll, False)
            self._set_many(self.qdbus, kinetic, False)
            self._set_many(self.qdbus, physical, True)
            LOG.info("running mode=native/failsafe; physical Magic Trackpad enabled in KWin")
        elif mode in ("touchpad", "pointer"):
            # Fail-safe ordering: make the running replacement usable first,
            # then remove the physical device only from KWin's libinput context.
            self._set_many(self.qdbus, other, False)
            self._set_many(self.qdbus, replacement, True)

            self._set_many(self.qdbus, scroll, mode == "touchpad")
            self._set_many(self.qdbus, kinetic, mode == "touchpad")
            self._set_many(self.qdbus, physical, False)
            LOG.info(
                "running mode=%s; replacement present, physical Magic Trackpad disabled in KWin",
                mode,
            )
        else:
            # Seeing both live replacement backends should not happen during a
            # normal systemd restart. Prefer stock input until the ambiguity
            # clears rather than risk duplicate or conflicting synthetic input.
            self._set_many(self.qdbus, touchpads, False)
            self._set_many(self.qdbus, pointers, False)
            self._set_many(self.qdbus, scroll, False)
            self._set_many(self.qdbus, kinetic, False)
            self._set_many(self.qdbus, physical, True)
            LOG.warning(
                "both AMT Raw Touchpad and AMT Raw Pointer are live; disabled both replacements and kept physical input enabled"
            )

        self._last_summary = summary

    def invalidate(self) -> None:
        self._last_summary = None
        self.qdbus._names.clear()

    def restore_safe(self) -> None:
        """Best-effort safe state if the session helper is deliberately stopped."""
        try:
            groups = self._named_paths()
        except RuntimeError:
            return
        self._set_many(self.qdbus, groups[PHYSICAL_NAME], True)
        self._set_many(self.qdbus, groups[TOUCHPAD_NAME], False)
        self._set_many(self.qdbus, groups[POINTER_NAME], False)
        self._set_many(self.qdbus, groups[SCROLL_NAME], False)
        self._set_many(self.qdbus, groups[KINETIC_NAME], False)


def input_event_signature() -> tuple[tuple[str, int, int], ...]:
    """Cheaply detect evdev topology changes without polling KWin over D-Bus."""
    items: list[tuple[str, int, int]] = []
    for path in Path("/dev/input").glob("event*"):
        try:
            st = path.stat()
        except OSError:
            continue
        items.append((path.name, int(st.st_ino), int(st.st_rdev)))
    return tuple(sorted(items))


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Synchronize KWin with the actually running amt-raw virtual input backend"
    )
    p.add_argument("--poll-seconds", type=float, default=1.0,
                   help="Steady-state KWin reconciliation interval")
    p.add_argument("--fast-poll-seconds", type=float, default=0.10,
                   help="KWin reconciliation interval during the short window after evdev topology changes")
    p.add_argument("--fast-window-seconds", type=float, default=2.0,
                   help="Duration of fast reconciliation after an evdev topology change")
    p.add_argument("--qdbus", default=None, help="Override qdbus executable (primarily for testing)")
    p.add_argument("--once", action="store_true", help="Run one reconciliation pass and exit")
    p.add_argument("--log-level", choices=("warning", "info", "debug"), default="info")
    return p


def main() -> int:
    args = build_parser().parse_args()
    if args.poll_seconds <= 0:
        raise SystemExit("--poll-seconds must be positive")
    if args.fast_poll_seconds <= 0:
        raise SystemExit("--fast-poll-seconds must be positive")
    if args.fast_window_seconds < 0:
        raise SystemExit("--fast-window-seconds must be >= 0")

    logging.basicConfig(
        level={"warning": logging.WARNING, "info": logging.INFO, "debug": logging.DEBUG}[args.log_level],
        format="%(asctime)s %(levelname)-8s %(name)s: %(message)s",
    )

    try:
        qdbus = QDbusKWin(args.qdbus)
    except RuntimeError as exc:
        LOG.error("%s", exc)
        return 1

    sync = Synchronizer(qdbus)
    if args.once:
        try:
            sync.sync_once()
            return 0
        except RuntimeError as exc:
            LOG.error("KWin D-Bus synchronization failed: %s", exc)
            return 1

    stop = False

    def handle_signal(_signum, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    last_error: Optional[str] = None
    last_input_sig = input_event_signature()
    now = time.monotonic()
    next_regular = now
    fast_until = 0.0
    next_fast = float("inf")

    while not stop:
        now = time.monotonic()
        current_sig = input_event_signature()
        if current_sig != last_input_sig:
            last_input_sig = current_sig
            sync.invalidate()
            fast_until = now + args.fast_window_seconds
            next_fast = now
            LOG.debug(
                "evdev topology changed; fast KWin reconciliation for %.2f s",
                args.fast_window_seconds,
            )

        due_regular = now >= next_regular
        due_fast = now < fast_until and now >= next_fast
        if due_regular or due_fast:
            try:
                sync.sync_once()
                last_error = None
            except RuntimeError as exc:
                sync.invalidate()
                msg = str(exc)
                if msg != last_error:
                    LOG.warning("KWin D-Bus unavailable: %s", msg)
                    last_error = msg

            now = time.monotonic()
            if due_regular:
                next_regular = now + args.poll_seconds
            if now < fast_until:
                next_fast = now + args.fast_poll_seconds
            else:
                next_fast = float("inf")

        now = time.monotonic()
        sleep_for = min(args.fast_poll_seconds, max(0.0, next_regular - now))
        if now < fast_until:
            sleep_for = min(sleep_for, max(0.0, next_fast - now))
        time.sleep(max(0.01, sleep_for))

    sync.restore_safe()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
