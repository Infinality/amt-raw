from __future__ import annotations

import logging
import select
import time

from .source import MagicTrackpadState, configure_hardware_report_rate, find_device, source_info

LOG = logging.getLogger("amt-raw")


def run(args, backend, stop_event):
    session = 0
    while not stop_event.is_set():
        dev = None
        try:
            dev = find_device(args.device)
            info = source_info(dev)
            configure_hardware_report_rate(dev.path, args.hardware_report_rate_hz)
            session += 1
            LOG.info(
                "Opened Magic Trackpad session #%d: %s (%s), %dx%d slots=%d, %.1fx%.1f mm",
                session,
                dev.name,
                dev.path,
                info.vendor,
                info.product,
                info.max_slots,
                info.width_mm,
                info.height_mm,
            )
            LOG.info(
                "ABS range X=%d..%d @ %d units/mm; Y=%d..%d @ %d units/mm",
                info.x_min, info.x_max, info.x_resolution,
                info.y_min, info.y_max, info.y_resolution,
            )

            backend.configure(dev, info)
            state = MagicTrackpadState(info)
            pending = []

            while not stop_event.is_set():
                readable, _, _ = select.select([dev], [], [], 0.5)
                if not readable:
                    continue

                for ev in dev.read():
                    pending.append(ev)
                    frame = state.update(ev)
                    if frame is None:
                        continue

                    host_ns = time.monotonic_ns()
                    if args.print_raw:
                        LOG.debug(
                            "frame touches=%s raw_button=%d",
                            [(t.finger_id, t.x, t.y, t.pressure) for t in frame.touches],
                            state.button_left,
                        )

                    if backend.mode in ("pointer", "touchpad"):
                        backend.process_report(frame, pending, host_ns)
                    else:
                        backend.process_report(frame, pending)
                    pending = []

        except (OSError, IOError) as exc:
            if not stop_event.is_set():
                LOG.warning(
                    "Magic Trackpad event stream lost (%s); retrying in %.1f s",
                    exc,
                    args.reconnect_delay_seconds,
                )
        except Exception as exc:
            if not stop_event.is_set():
                LOG.warning(
                    "Magic Trackpad session failed (%s); retrying in %.1f s",
                    exc,
                    args.reconnect_delay_seconds,
                    exc_info=LOG.isEnabledFor(logging.DEBUG),
                )
        finally:
            try:
                backend.reset_runtime()
            except Exception:
                LOG.debug("Error resetting backend runtime", exc_info=True)
            if dev is not None:
                try:
                    dev.close()
                except Exception:
                    pass

        if not stop_event.is_set():
            stop_event.wait(args.reconnect_delay_seconds)

    return 0
