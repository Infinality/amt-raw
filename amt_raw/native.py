from __future__ import annotations

import logging

LOG = logging.getLogger("amt-raw")


class NativeBackend:
    mode = "native"

    def __init__(self, args):
        self.args = args
        self._logged = False

    def configure(self, source, info):
        if not self._logged:
            LOG.info("Native mode: monitoring only; physical Magic Trackpad remains authoritative")
            self._logged = True

    def process_report(self, frame, events, host_ns=None):
        return

    def reset_runtime(self):
        return

    def close(self):
        return
