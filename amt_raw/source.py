from __future__ import annotations

import fcntl
import glob
import logging
import os
import re
from dataclasses import dataclass
from typing import Optional

from evdev import InputDevice, ecodes, list_devices

from .frames import RawFrame, Touch

LOG = logging.getLogger("amt-raw")

APPLE_USB_VENDOR = 0x05AC
APPLE_BLUETOOTH_VENDOR = 0x004C
MAGIC_TRACKPAD_USBC = 0x0324
BUS_BLUETOOTH = getattr(ecodes, "BUS_BLUETOOTH", 0x05)


# Undocumented USB-C Magic Trackpad hardware report-rate feature discovered
# empirically by the Linux Magic Trackpad community.  hidapitester's
# ``--send-feature 153,242,X`` is just HIDIOCSFEATURE(3) on the matching
# hidraw node.  X is approximately 3850 / requested_hz.
_REPORT_RATE_FEATURE_ID = 0x99
_REPORT_RATE_FEATURE_TAG = 0xF2
_REPORT_RATE_CLOCK = 3850.0
_HID_SYSFS_RE = re.compile(r"^[0-9A-Fa-f]{4}:[0-9A-Fa-f]{4}:[0-9A-Fa-f]{4}\.[0-9A-Fa-f]+$")

# Linux asm-generic ioctl encoding used by Fedora x86_64/aarch64 and the
# architectures on which hidraw is normally exposed.  This produces the same
# request as HIDIOCSFEATURE(len) from <linux/hidraw.h>.
_IOC_NRBITS = 8
_IOC_TYPEBITS = 8
_IOC_SIZEBITS = 14
_IOC_NRSHIFT = 0
_IOC_TYPESHIFT = _IOC_NRSHIFT + _IOC_NRBITS
_IOC_SIZESHIFT = _IOC_TYPESHIFT + _IOC_TYPEBITS
_IOC_DIRSHIFT = _IOC_SIZESHIFT + _IOC_SIZEBITS
_IOC_WRITE = 1
_IOC_READ = 2


def _ioc(direction: int, type_char: str, nr: int, size: int) -> int:
    return (
        (direction << _IOC_DIRSHIFT)
        | (ord(type_char) << _IOC_TYPESHIFT)
        | (nr << _IOC_NRSHIFT)
        | (size << _IOC_SIZESHIFT)
    )


def _hidio_csfeature(length: int) -> int:
    return _ioc(_IOC_WRITE | _IOC_READ, "H", 0x06, length)


def _event_sysfs_path(event_path: str) -> str:
    event_node = os.path.basename(os.path.realpath(event_path))
    return os.path.realpath(f"/sys/class/input/{event_node}/device")


def _hid_sysfs_parent_for_event(event_path: str) -> Optional[str]:
    """Return the HID-bus sysfs node that owns an evdev event node."""
    current = _event_sysfs_path(event_path)
    while current and current != "/":
        if _HID_SYSFS_RE.match(os.path.basename(current)):
            return current
        current = os.path.dirname(current)
    return None


def _usb_sysfs_parent_for_event(event_path: str) -> Optional[str]:
    """Return the composite USB-device sysfs node owning event_path."""
    current = _event_sysfs_path(event_path)
    while current and current != "/":
        base = os.path.basename(current)
        # USB interface names look like ``1-7:1.1``.  Its parent is the
        # composite USB device (``1-7``), shared by all three 0324 HID
        # interfaces.
        if re.search(r":\d+\.\d+$", base):
            return os.path.dirname(current)
        current = os.path.dirname(current)
    return None


def _candidate_hidraws(event_path: str) -> list[str]:
    """Return likely 05ac:0324 hidraw nodes, source interface first.

    hidapitester selects a HID collection through HIDAPI/udev.  amt-raw can
    avoid that dependency because it already knows the exact evdev source and
    therefore the owning HID and composite USB sysfs paths.
    """
    hid_parent = _hid_sysfs_parent_for_event(event_path)
    usb_parent = _usb_sysfs_parent_for_event(event_path)
    exact: list[str] = []
    siblings: list[str] = []

    for sys_node in sorted(glob.glob("/sys/class/hidraw/hidraw*/device")):
        resolved = os.path.realpath(sys_node)
        hid_component = next(
            (part for part in resolved.split(os.sep) if _HID_SYSFS_RE.match(part)),
            None,
        )
        if hid_component is None or ":05AC:0324." not in hid_component.upper():
            continue

        devnode = "/dev/" + os.path.basename(os.path.dirname(sys_node))
        if hid_parent and (resolved == hid_parent or resolved.startswith(hid_parent + os.sep)):
            exact.append(devnode)
        elif usb_parent and resolved.startswith(usb_parent + os.sep):
            siblings.append(devnode)

    # De-duplicate while preserving the source-interface preference.
    return list(dict.fromkeys(exact + siblings))


def configure_hardware_report_rate(
    event_path: str, requested_hz: float, bustype: Optional[int] = None
) -> bool:
    """Request the USB Magic Trackpad hardware report rate through hidraw.

    Bluetooth intentionally skips this USB-specific feature. A value <= 0
    disables the feature. Failure is non-fatal so amt-raw can
    continue to operate if a kernel/firmware revision does not expose the
    undocumented feature report.
    """
    if requested_hz <= 0:
        LOG.info("Hardware report-rate override disabled")
        return False

    # Feature 0x99/0xf2 was discovered and validated on the USB transport.
    # Bluetooth exposes the same normalized evdev MT stream through
    # hid-magicmouse, but uses different HID reports. Do not send an
    # undocumented USB feature over Bluetooth; resampling uses measured
    # report cadence and therefore does not depend on this override.
    if bustype == BUS_BLUETOOTH:
        LOG.info(
            "Hardware report-rate override is USB-only; leaving Bluetooth "
            "trackpad cadence unchanged (resampling uses measured input timing)"
        )
        return False

    interval = int(round(_REPORT_RATE_CLOCK / requested_hz))
    if not 1 <= interval <= 255:
        raise ValueError(
            f"hardware report rate {requested_hz:g} Hz maps to invalid interval {interval}; "
            "choose a rate between about 15.1 and 3850 Hz"
        )

    candidates = _candidate_hidraws(event_path)
    if not candidates:
        LOG.warning(
            "Could not locate hidraw interface for %s; leaving hardware report rate unchanged",
            event_path,
        )
        return False

    payload = bytearray((_REPORT_RATE_FEATURE_ID, _REPORT_RATE_FEATURE_TAG, interval))
    nominal_hz = _REPORT_RATE_CLOCK / interval
    failures = []
    for hidraw in candidates:
        try:
            fd = os.open(hidraw, os.O_RDWR | os.O_CLOEXEC)
            try:
                fcntl.ioctl(fd, _hidio_csfeature(len(payload)), payload, True)
            finally:
                os.close(fd)
        except OSError as exc:
            failures.append(f"{hidraw}: {exc}")
            continue

        LOG.info(
            "Requested Magic Trackpad hardware report rate %.1f Hz via %s "
            "(feature 99 f2 %02x; nominal %.1f Hz)",
            requested_hz, hidraw, interval, nominal_hz,
        )
        return True

    LOG.warning(
        "Magic Trackpad hardware report-rate feature failed on candidate hidraw nodes (%s)",
        "; ".join(failures),
    )
    return False


@dataclass(frozen=True)
class SourceInfo:
    path: str
    name: str
    vendor: int
    product: int
    bustype: int
    version: int
    x_min: int
    x_max: int
    y_min: int
    y_max: int
    x_resolution: int
    y_resolution: int
    max_slots: int

    @property
    def width_mm(self) -> float:
        return (self.x_max - self.x_min) / self.x_resolution if self.x_resolution else 0.0

    @property
    def height_mm(self) -> float:
        return (self.y_max - self.y_min) / self.y_resolution if self.y_resolution else 0.0


@dataclass
class _Slot:
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


class MagicTrackpadState:
    """Assemble Linux Type-B MT changes into complete contact frames."""

    def __init__(self, info: SourceInfo):
        self.info = info
        self.current_slot = 0
        self.slots = [_Slot() for _ in range(info.max_slots)]
        self.button_left = 0

    def update(self, event) -> Optional[RawFrame]:
        if event.type == ecodes.EV_ABS:
            c, v = event.code, event.value
            if c == ecodes.ABS_MT_SLOT:
                if 0 <= v < len(self.slots):
                    self.current_slot = v
                return None

            slot = self.slots[self.current_slot]
            if c == ecodes.ABS_MT_TRACKING_ID:
                if v < 0:
                    self.slots[self.current_slot] = _Slot()
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
            touches = tuple(
                Touch(
                    finger_id=s.tracking_id,
                    x=s.x,
                    y=s.y,
                    pressure=s.pressure,
                    major=s.major,
                    minor=s.minor,
                    orientation=s.orientation,
                )
                for s in self.slots
                if s.active
            )
            ts_ms = int(event.timestamp() * 1000.0) & 0xFFFF
            return RawFrame(timestamp=ts_ms, touches=touches)

        return None


def _is_magic_trackpad(dev: InputDevice) -> bool:
    # The virtual touchpad uses the real 05ac:0324 identity so
    # libinput applies the same Apple hardware quirks as native mode. Never
    # allow source autodetection to consume our own replacement device.
    if (dev.name or "").startswith("AMT Raw "):
        return False
    if (
        dev.info.vendor not in (APPLE_USB_VENDOR, APPLE_BLUETOOTH_VENDOR)
        or dev.info.product != MAGIC_TRACKPAD_USBC
    ):
        return False
    caps = dev.capabilities(absinfo=False)
    abs_codes = set(caps.get(ecodes.EV_ABS, []))
    needed = {
        ecodes.ABS_MT_SLOT,
        ecodes.ABS_MT_TRACKING_ID,
        ecodes.ABS_MT_POSITION_X,
        ecodes.ABS_MT_POSITION_Y,
    }
    return needed.issubset(abs_codes)


def _interface_score(path: str) -> int:
    """Prefer USB interface 01, the active MT stream on the tested 05ac:0324."""
    base = os.path.basename(path)
    syslink = f"/sys/class/input/{base}/device"
    try:
        target = os.path.realpath(syslink)
    except OSError:
        return 0
    return 10 if ":1.1/" in target else 0


def find_device(explicit_path: Optional[str] = None) -> InputDevice:
    if explicit_path == "auto":
        explicit_path = None
    if explicit_path:
        dev = InputDevice(explicit_path)
        if not _is_magic_trackpad(dev):
            dev.close()
            raise RuntimeError(f"{explicit_path} is not the expected Apple Magic Trackpad MT device")
        return dev

    # The stable by-id interface-01 name is the strongest signal when USB-connected.
    by_id = sorted(glob.glob("/dev/input/by-id/*Apple*Magic*Trackpad*-if01-event-mouse"))
    for path in by_id:
        try:
            dev = InputDevice(path)
            if _is_magic_trackpad(dev):
                return dev
            dev.close()
        except OSError:
            pass

    candidates = []
    for path in list_devices():
        try:
            dev = InputDevice(path)
        except OSError:
            continue
        if _is_magic_trackpad(dev):
            candidates.append((_interface_score(dev.path), dev.path, dev))
        else:
            dev.close()

    if not candidates:
        raise RuntimeError(
            "Apple Magic Trackpad USB-C (USB 05ac:0324 or Bluetooth 004c:0324) "
            "multitouch event device not found"
        )

    candidates.sort(key=lambda item: (item[0], item[1]), reverse=True)
    chosen = candidates[0][2]
    for _, _, dev in candidates[1:]:
        dev.close()
    return chosen


def source_info(dev: InputDevice) -> SourceInfo:
    xi = dev.absinfo(ecodes.ABS_MT_POSITION_X)
    yi = dev.absinfo(ecodes.ABS_MT_POSITION_Y)
    si = dev.absinfo(ecodes.ABS_MT_SLOT)
    return SourceInfo(
        path=dev.path,
        name=dev.name,
        vendor=dev.info.vendor,
        product=dev.info.product,
        bustype=dev.info.bustype,
        version=dev.info.version,
        x_min=xi.min,
        x_max=xi.max,
        y_min=yi.min,
        y_max=yi.max,
        x_resolution=xi.resolution,
        y_resolution=yi.resolution,
        max_slots=si.max - si.min + 1,
    )
