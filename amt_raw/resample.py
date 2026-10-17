from __future__ import annotations

import logging
import math
import statistics
import threading
import time
from collections import deque
from typing import Callable, Optional

LOG = logging.getLogger("amt-raw")


class PointerMotionResampler:
    """Fixed-rate pointer-motion resampler.

    The input side supplies already interpreted, post-gain logical pointer
    deltas at physical-trackpad report times.  The output side owns an
    independent monotonic clock and emits REL-ready integer deltas at the
    requested rate.

    Three presentation strategies are available. ``interpolated`` is the
    preferred delayed known-trajectory renderer. ``latest`` uses newest-segment
    constant-velocity extrapolation, while ``adaptive`` adds a conservative
    short-history acceleration estimate. The two causal modes are capped to at
    most one recent physical report interval and an absolute 20 ms ceiling.

    ``interpolated`` intentionally trades approximately one measured physical
    interval of latency for a known-trajectory reconstruction.  It keeps a
    short history of measured post-gain logical positions, renders a delayed
    timestamp on the independent output clock, and reconstructs each fully known
    segment with a position-anchored linear-velocity (constant-acceleration)
    curve derived from the previous and current interval-average velocities.
    The curve is constrained to land exactly on every measured logical position;
    stops/reversals that would create a local backwards tangent fall back to
    ordinary linear position interpolation.  It never predicts beyond the newest
    measured position.

    The 25% and 12.5% acceleration candidates continue to run in shadow for
    diagnostics in every live mode. Shadow scoring remains isolated from
    presentation state and REL output; only the explicitly selected live
    predictor can supply the extrapolation velocity used by _predict_locked().

    This layer knows nothing about taps, gestures, pressure, scrolling, or
    lift-off.  Those decisions must happen before submit().
    """

    def __init__(
        self,
        rate_hz: float,
        emit_relative: Callable[[int, int], None],
        *,
        predictor: str = "latest",
        max_prediction_ms: float = 20.0,
        stats_interval_s: float = 5.0,
    ):
        self.rate_hz = float(rate_hz)
        self.enabled = self.rate_hz > 0.0
        self.predictor = str(predictor).strip().lower()
        if self.predictor not in {"latest", "adaptive", "interpolated"}:
            raise ValueError(f"unsupported pointer resampling predictor: {predictor!r}")
        self._emit_relative = emit_relative
        self.max_prediction_ns = max(0, int(max_prediction_ms * 1_000_000.0))
        self.stats_interval_ns = max(1, int(stats_interval_s * 1_000_000_000.0))

        self._cond = threading.Condition(threading.RLock())
        self._closed = False
        self._active = False
        self._generation = 0

        self._period_ns = (
            max(1, int(round(1_000_000_000.0 / self.rate_hz)))
            if self.enabled
            else 0
        )
        self._next_tick_ns: Optional[int] = None

        # Continuous logical coordinate system for one pointer-motion session.
        self._measured_x = 0.0
        self._measured_y = 0.0
        self._presentation_offset_x = 0.0
        self._presentation_offset_y = 0.0
        self._velocity_x_per_ns = 0.0
        self._velocity_y_per_ns = 0.0
        self._sample_ns: Optional[int] = None
        self._sample_interval_ns = 0

        # Last logical position sampled by the fixed-rate output clock.
        self._last_target_x = 0.0
        self._last_target_y = 0.0
        self._frac_x = 0.0
        self._frac_y = 0.0

        # Diagnostics.  Keep bounded recent windows rather than logging every
        # tick; 240 Hz debug logging would itself perturb scheduler timing.
        self._logical_ticks = 0
        self._nonzero_emits = 0
        self._skipped_ticks = 0
        self._input_samples = 0
        self._rebases = 0
        self._last_wake_ns: Optional[int] = None
        self._last_stats_log_ns = time.monotonic_ns()
        self._tick_intervals_ms: deque[float] = deque(maxlen=2048)
        self._lateness_ms: deque[float] = deque(maxlen=2048)
        self._input_intervals_ms: deque[float] = deque(maxlen=2048)
        self._prediction_age_ms: deque[float] = deque(maxlen=2048)
        self._prediction_used_ms: deque[float] = deque(maxlen=2048)
        self._rebase_distance: deque[float] = deque(maxlen=2048)

        # Delayed interpolation state. Samples are
        # (host_sample_ns, cumulative_x, cumulative_y, physical_interval_ns).
        # The first physical delta gets a synthetic zero-position anchor one
        # measured interval earlier, so every rendered segment is position
        # anchored from its first output tick.
        self._interp_samples: deque[tuple[int, float, float, int]] = deque(maxlen=32)
        self._interp_interval_history_ns: deque[int] = deque(maxlen=8)
        self._interp_delay_ns = 0
        self._interp_last_query_ns: Optional[int] = None
        self._interp_finishing = False
        self._interp_hold_ticks = 0
        self._interp_curve_ticks = 0
        self._interp_linear_ticks = 0

        # Dual shadow predictors. This state is intentionally isolated
        # from every presentation/output variable above.  Both candidates use
        # the same three-segment consistency gates and differ only in how much
        # of the newest acceleration they carry forward (25% versus 12.5%).
        # Nothing here is consulted by _predict_locked() or _run().
        self._shadow_initialized = False
        self._shadow_latest_vx_per_ns = 0.0
        self._shadow_latest_vy_per_ns = 0.0
        self._shadow25_vx_per_ns = 0.0
        self._shadow25_vy_per_ns = 0.0
        self._shadow125_vx_per_ns = 0.0
        self._shadow125_vy_per_ns = 0.0
        self._shadow_prevprev_vx_per_ns: Optional[float] = None
        self._shadow_prevprev_vy_per_ns: Optional[float] = None
        self._shadow_prediction_samples = 0
        self._shadow_latest_error: deque[float] = deque(maxlen=2048)
        self._shadow_latest_long_abs: deque[float] = deque(maxlen=2048)
        self._shadow_latest_lat_abs: deque[float] = deque(maxlen=2048)
        self._shadow_mode_counts = {
            "predictive": 0,
            "history": 0,
            "speedchange": 0,
            "turn": 0,
            "accelturn": 0,
            "accelmag": 0,
            "accelzero": 0,
            "start": 0,
            "stop": 0,
        }
        self._shadow_candidates = {
            "25": {
                "carry": 0.25,
                "wins": 0,
                "ties": 0,
                "changed": 0,
                "changed_wins": 0,
                "adaptive_error": deque(maxlen=2048),
                "error_ratio": deque(maxlen=2048),
                "changed_ratio": deque(maxlen=2048),
                "long_abs": deque(maxlen=2048),
                "lat_abs": deque(maxlen=2048),
            },
            "125": {
                "carry": 0.125,
                "wins": 0,
                "ties": 0,
                "changed": 0,
                "changed_wins": 0,
                "adaptive_error": deque(maxlen=2048),
                "error_ratio": deque(maxlen=2048),
                "changed_ratio": deque(maxlen=2048),
                "long_abs": deque(maxlen=2048),
                "lat_abs": deque(maxlen=2048),
            },
        }

        self._thread: Optional[threading.Thread] = None
        if self.enabled:
            self._thread = threading.Thread(
                target=self._run,
                name="amt-raw-resampler",
                daemon=True,
            )
            self._thread.start()
            LOG.info(
                "Pointer resampler enabled: target=%.3f Hz period=%.3f ms predictor=%s max-lookahead=%.1f ms shadows=consistent-acceleration-v2(25%%,12.5%%)",
                self.rate_hz,
                self._period_ns / 1_000_000.0,
                self.predictor,
                self.max_prediction_ns / 1_000_000.0,
            )

    @staticmethod
    def _p95(values: deque[float]) -> float:
        if not values:
            return 0.0
        seq = sorted(values)
        idx = min(len(seq) - 1, max(0, math.ceil(0.95 * len(seq)) - 1))
        return seq[idx]

    @staticmethod
    def _median(values: deque[float]) -> float:
        return statistics.median(values) if values else 0.0

    def _prediction_horizon_ns_locked(self) -> int:
        if self._sample_interval_ns <= 0:
            return self.max_prediction_ns
        return min(self.max_prediction_ns, self._sample_interval_ns)

    def _interpolated_delay_locked(self) -> int:
        """Return the measured delay used by the known-trajectory renderer.

        The delay is based only on observed physical cadence.  The configured
        hardware-rate request is deliberately irrelevant after device setup.
        A short rolling maximum absorbs the normal 9/10 ms or 16/17 ms cadence
        quantization without requiring a nominal Hz assumption.
        """
        if self._interp_interval_history_ns:
            return max(self._interp_interval_history_ns)
        if self._sample_interval_ns > 0:
            return self._sample_interval_ns
        return 0

    def _interpolated_predict_locked(
        self, when_ns: int
    ) -> tuple[float, float, int, int]:
        if not self._active or not self._interp_samples:
            return self._last_target_x, self._last_target_y, 0, 0

        newest_ns = self._interp_samples[-1][0]
        age_ns = max(0, when_ns - newest_ns)
        delay_ns = self._interpolated_delay_locked()
        self._interp_delay_ns = delay_ns
        query_ns = when_ns - delay_ns

        # A cadence estimate can grow slightly (for example 16 -> 17 ms).  Do
        # not let that adjustment move presentation time backwards; hold until
        # the delayed clock catches up instead.
        if self._interp_last_query_ns is not None and query_ns < self._interp_last_query_ns:
            query_ns = self._interp_last_query_ns
        self._interp_last_query_ns = query_ns

        samples = list(self._interp_samples)
        if query_ns <= samples[0][0]:
            self._interp_hold_ticks += 1
            return samples[0][1], samples[0][2], 0, 0
        if query_ns >= samples[-1][0]:
            self._interp_hold_ticks += 1
            return samples[-1][1], samples[-1][2], 0, 0

        # History is tiny (<=32 samples); a linear scan avoids maintaining a
        # parallel timestamp index under the input/output lock.
        seg = 0
        for idx in range(len(samples) - 1):
            if samples[idx][0] <= query_ns <= samples[idx + 1][0]:
                seg = idx
                break

        t0, x0, y0, _dt0 = samples[seg]
        t1, x1, y1, cur_physical_dt_ns = samples[seg + 1]
        host_dt_ns = max(1, t1 - t0)
        u = max(0.0, min(1.0, (query_ns - t0) / host_dt_ns))
        dx = x1 - x0
        dy = y1 - y0

        # The first known segment has no preceding interval-average velocity.
        # Stops, starts, reversals, or any curve whose endpoint tangent would
        # point backwards use plain position interpolation.  This remains
        # exactly position anchored and cannot create prediction debt.
        use_curve = seg >= 1 and cur_physical_dt_ns > 0
        corr_x = 0.0
        corr_y = 0.0
        if use_curve:
            tp, xp, yp, prev_physical_dt_ns = samples[seg - 1]
            if prev_physical_dt_ns <= 0:
                use_curve = False
            else:
                prev_dx = x0 - xp
                prev_dy = y0 - yp
                prev_vx = prev_dx / prev_physical_dt_ns
                prev_vy = prev_dy / prev_physical_dt_ns
                cur_vx = dx / cur_physical_dt_ns
                cur_vy = dy / cur_physical_dt_ns
                prev_speed = math.hypot(prev_vx, prev_vy)
                cur_speed = math.hypot(cur_vx, cur_vy)
                if (
                    prev_speed <= 1.0e-12
                    or cur_speed <= 1.0e-12
                    or (prev_vx * cur_vx + prev_vy * cur_vy) <= 0.0
                ):
                    use_curve = False
                else:
                    midpoint_dt_ns = max(
                        1.0,
                        0.5 * (prev_physical_dt_ns + cur_physical_dt_ns),
                    )
                    ax = (cur_vx - prev_vx) / midpoint_dt_ns
                    ay = (cur_vy - prev_vy) / midpoint_dt_ns
                    dvx_half = 0.5 * ax * cur_physical_dt_ns
                    dvy_half = 0.5 * ay * cur_physical_dt_ns
                    start_vx = cur_vx - dvx_half
                    start_vy = cur_vy - dvy_half
                    end_vx = cur_vx + dvx_half
                    end_vy = cur_vy + dvy_half
                    if (
                        start_vx * cur_vx + start_vy * cur_vy <= 0.0
                        or end_vx * cur_vx + end_vy * cur_vy <= 0.0
                    ):
                        use_curve = False
                    else:
                        # Integrating a linearly changing velocity gives this
                        # zero-at-both-ends correction to ordinary position
                        # lerp.  Therefore u=0 is exactly P0 and u=1 exactly P1.
                        a_dt2_x = ax * cur_physical_dt_ns * cur_physical_dt_ns
                        a_dt2_y = ay * cur_physical_dt_ns * cur_physical_dt_ns
                        shape = 0.5 * (u * u - u)
                        corr_x = a_dt2_x * shape
                        corr_y = a_dt2_y * shape

        if use_curve:
            self._interp_curve_ticks += 1
        else:
            self._interp_linear_ticks += 1

        return (
            x0 + dx * u + corr_x,
            y0 + dy * u + corr_y,
            0,
            0,
        )

    def _predict_locked(self, when_ns: int) -> tuple[float, float, int, int]:
        if self.predictor == "interpolated":
            return self._interpolated_predict_locked(when_ns)
        if not self._active or self._sample_ns is None:
            return self._last_target_x, self._last_target_y, 0, 0

        age_ns = max(0, when_ns - self._sample_ns)
        used_ns = min(age_ns, self._prediction_horizon_ns_locked())
        x = (
            self._measured_x
            + self._presentation_offset_x
            + self._velocity_x_per_ns * used_ns
        )
        y = (
            self._measured_y
            + self._presentation_offset_y
            + self._velocity_y_per_ns * used_ns
        )
        return x, y, age_ns, used_ns

    def _shadow_start_locked(self, dx: float, dy: float, dt_ns: int) -> None:
        vx = float(dx) / dt_ns
        vy = float(dy) / dt_ns
        self._shadow_initialized = True
        self._shadow_latest_vx_per_ns = vx
        self._shadow_latest_vy_per_ns = vy
        self._shadow25_vx_per_ns = vx
        self._shadow25_vy_per_ns = vy
        self._shadow125_vx_per_ns = vx
        self._shadow125_vy_per_ns = vy
        # ``prevprev`` becomes available after one further physical segment.
        # Require three segment velocities before any
        # acceleration extrapolation is allowed.
        self._shadow_prevprev_vx_per_ns = None
        self._shadow_prevprev_vy_per_ns = None

    @staticmethod
    def _shadow_cosine(
        ax: float, ay: float, bx: float, by: float, *, eps: float = 1.0e-12
    ) -> Optional[float]:
        amag = math.hypot(ax, ay)
        bmag = math.hypot(bx, by)
        if amag <= eps or bmag <= eps:
            return None
        return max(-1.0, min(1.0, (ax * bx + ay * by) / (amag * bmag)))

    def _shadow_score_candidate_locked(
        self,
        candidate: dict,
        pred_dx: float,
        pred_dy: float,
        pred_latest_dx: float,
        pred_latest_dy: float,
        actual_dx: float,
        actual_dy: float,
        latest_err: float,
        actual_mag: float,
        actual_ux: float,
        actual_uy: float,
    ) -> None:
        adapt_ex = pred_dx - actual_dx
        adapt_ey = pred_dy - actual_dy
        adapt_err = math.hypot(adapt_ex, adapt_ey)
        changed = math.hypot(
            pred_dx - pred_latest_dx, pred_dy - pred_latest_dy
        ) >= 0.05

        candidate["adaptive_error"].append(adapt_err)
        if latest_err >= 0.5:
            ratio = adapt_err / latest_err
            candidate["error_ratio"].append(ratio)
            if changed:
                candidate["changed_ratio"].append(ratio)
        if changed:
            candidate["changed"] += 1
        if adapt_err + 0.05 < latest_err:
            candidate["wins"] += 1
            if changed:
                candidate["changed_wins"] += 1
        elif abs(adapt_err - latest_err) <= 0.05:
            candidate["ties"] += 1

        if actual_mag >= 0.5:
            adapt_long = adapt_ex * actual_ux + adapt_ey * actual_uy
            adapt_lat = -adapt_ex * actual_uy + adapt_ey * actual_ux
            candidate["long_abs"].append(abs(adapt_long))
            candidate["lat_abs"].append(abs(adapt_lat))

    def _shadow_score_and_update_locked(self, dx: float, dy: float, dt_ns: int) -> None:
        """Score constant velocity against two diagnostics-only acceleration candidates.

        Both shadow candidates use the same conservative gates:

          * newest speed change versus the previous segment is <= 25%;
          * newest trajectory turn is < 35 degrees;
          * the two newest acceleration vectors differ by < 35 degrees;
          * their magnitudes are within 1.5x of one another.

        The only experimental variable is acceleration carry.  Candidate ``25``
        applies 25% of the newest acceleration; candidate ``125`` applies
        12.5%.  The acceleration vector remains capped at 40% of newest speed.
        Any failed gate falls straight back to newest-segment velocity.
        This method is diagnostics-only and must never mutate live
        presentation state or output.
        """
        if not self._shadow_initialized:
            self._shadow_start_locked(dx, dy, dt_ns)
            return

        actual_dx = float(dx)
        actual_dy = float(dy)
        pred_latest_dx = self._shadow_latest_vx_per_ns * dt_ns
        pred_latest_dy = self._shadow_latest_vy_per_ns * dt_ns
        latest_ex = pred_latest_dx - actual_dx
        latest_ey = pred_latest_dy - actual_dy
        latest_err = math.hypot(latest_ex, latest_ey)

        actual_mag = math.hypot(actual_dx, actual_dy)
        if actual_mag >= 0.5:
            actual_ux = actual_dx / actual_mag
            actual_uy = actual_dy / actual_mag
            latest_long = latest_ex * actual_ux + latest_ey * actual_uy
            latest_lat = -latest_ex * actual_uy + latest_ey * actual_ux
            self._shadow_latest_long_abs.append(abs(latest_long))
            self._shadow_latest_lat_abs.append(abs(latest_lat))
        else:
            actual_ux = 0.0
            actual_uy = 0.0

        self._shadow_prediction_samples += 1
        self._shadow_latest_error.append(latest_err)
        self._shadow_score_candidate_locked(
            self._shadow_candidates["25"],
            self._shadow25_vx_per_ns * dt_ns,
            self._shadow25_vy_per_ns * dt_ns,
            pred_latest_dx,
            pred_latest_dy,
            actual_dx,
            actual_dy,
            latest_err,
            actual_mag,
            actual_ux,
            actual_uy,
        )
        self._shadow_score_candidate_locked(
            self._shadow_candidates["125"],
            self._shadow125_vx_per_ns * dt_ns,
            self._shadow125_vy_per_ns * dt_ns,
            pred_latest_dx,
            pred_latest_dy,
            actual_dx,
            actual_dy,
            latest_err,
            actual_mag,
            actual_ux,
            actual_uy,
        )

        newest_vx = actual_dx / dt_ns
        newest_vy = actual_dy / dt_ns
        newest_speed = math.hypot(newest_vx, newest_vy)
        prev_vx = self._shadow_latest_vx_per_ns
        prev_vy = self._shadow_latest_vy_per_ns
        prev_speed = math.hypot(prev_vx, prev_vy)
        prevprev_vx = self._shadow_prevprev_vx_per_ns
        prevprev_vy = self._shadow_prevprev_vy_per_ns

        eps = 1.0e-12
        cos35 = math.cos(math.radians(35.0))
        accel_x = 0.0
        accel_y = 0.0
        if newest_speed <= eps:
            mode = "stop"
        elif prev_speed <= eps:
            mode = "start"
        elif prevprev_vx is None or prevprev_vy is None:
            mode = "history"
        else:
            turn_cos = self._shadow_cosine(newest_vx, newest_vy, prev_vx, prev_vy)
            rel_speed_change = abs(newest_speed - prev_speed) / max(
                newest_speed, prev_speed, eps
            )
            if turn_cos is None or turn_cos < cos35:
                mode = "turn"
            elif rel_speed_change > 0.25:
                mode = "speedchange"
            else:
                accel1_x = prev_vx - prevprev_vx
                accel1_y = prev_vy - prevprev_vy
                accel2_x = newest_vx - prev_vx
                accel2_y = newest_vy - prev_vy
                accel1_mag = math.hypot(accel1_x, accel1_y)
                accel2_mag = math.hypot(accel2_x, accel2_y)
                if accel1_mag <= eps or accel2_mag <= eps:
                    mode = "accelzero"
                else:
                    accel_cos = self._shadow_cosine(
                        accel1_x, accel1_y, accel2_x, accel2_y
                    )
                    accel_mag_ratio = max(accel1_mag, accel2_mag) / min(
                        accel1_mag, accel2_mag
                    )
                    if accel_cos is None or accel_cos < cos35:
                        mode = "accelturn"
                    elif accel_mag_ratio > 1.5:
                        mode = "accelmag"
                    else:
                        accel_x = accel2_x
                        accel_y = accel2_y
                        accel_cap = 0.40 * newest_speed
                        if accel2_mag > accel_cap and accel2_mag > eps:
                            scale = accel_cap / accel2_mag
                            accel_x *= scale
                            accel_y *= scale
                        mode = "predictive"

        if mode == "predictive":
            self._shadow25_vx_per_ns = newest_vx + 0.25 * accel_x
            self._shadow25_vy_per_ns = newest_vy + 0.25 * accel_y
            self._shadow125_vx_per_ns = newest_vx + 0.125 * accel_x
            self._shadow125_vy_per_ns = newest_vy + 0.125 * accel_y
        else:
            self._shadow25_vx_per_ns = newest_vx
            self._shadow25_vy_per_ns = newest_vy
            self._shadow125_vx_per_ns = newest_vx
            self._shadow125_vy_per_ns = newest_vy

        self._shadow_mode_counts[mode] += 1
        self._shadow_prevprev_vx_per_ns = prev_vx
        self._shadow_prevprev_vy_per_ns = prev_vy
        self._shadow_latest_vx_per_ns = newest_vx
        self._shadow_latest_vy_per_ns = newest_vy

    def submit(self, dx: float, dy: float, sample_ns: int, sample_dt_ms: float) -> None:
        """Submit one post-gain hardware motion sample.

        Zero-motion samples are significant: they immediately set the latest
        velocity to zero and therefore stop further extrapolation.
        """
        if not self.enabled:
            return

        dt_ns = max(1, int(round(max(sample_dt_ms, 0.001) * 1_000_000.0)))
        vx = float(dx) / dt_ns
        vy = float(dy) / dt_ns

        with self._cond:
            if self._closed:
                return

            if not self._active or self._sample_ns is None:
                self._active = True
                self._generation += 1
                self._measured_x = float(dx)
                self._measured_y = float(dy)
                self._presentation_offset_x = 0.0
                self._presentation_offset_y = 0.0
                # Both predictors are identical on the first physical segment.
                self._velocity_x_per_ns = vx
                self._velocity_y_per_ns = vy
                self._sample_ns = int(sample_ns)
                self._sample_interval_ns = dt_ns
                if self.predictor == "interpolated":
                    self._interp_samples.clear()
                    self._interp_interval_history_ns.clear()
                    self._interp_samples.append(
                        (int(sample_ns) - dt_ns, 0.0, 0.0, dt_ns)
                    )
                    self._interp_samples.append(
                        (int(sample_ns), float(dx), float(dy), dt_ns)
                    )
                    self._interp_interval_history_ns.append(dt_ns)
                    self._interp_delay_ns = dt_ns
                    self._interp_last_query_ns = None
                    self._interp_finishing = False
                self._last_target_x = 0.0
                self._last_target_y = 0.0
                self._frac_x = 0.0
                self._frac_y = 0.0
                # First motion sample is eligible immediately; every following
                # output tick remains on this absolute cadence.
                self._next_tick_ns = int(sample_ns)
                self._last_wake_ns = None
                self._input_samples += 1
                self._input_intervals_ms.append(dt_ns / 1_000_000.0)
                self._shadow_start_locked(dx, dy, dt_ns)
                self._cond.notify_all()
                return

            # Score/update the shadow predictors before touching any
            # live presentation variables. This also computes the next
            # 12.5% adaptive velocity candidate used when explicitly
            # selected; shadow statistics themselves remain diagnostics-only.
            self._shadow_score_and_update_locked(dx, dy, dt_ns)

            if self.predictor == "interpolated":
                self._measured_x += float(dx)
                self._measured_y += float(dy)
                self._interp_samples.append(
                    (
                        int(sample_ns),
                        self._measured_x,
                        self._measured_y,
                        dt_ns,
                    )
                )
                # The delayed presentation clock is based on the physical HID
                # timestamp cadence, never on AMT_HARDWARE_REPORT_RATE_HZ.  Host
                # scheduling stalls do not redefine the device's sample rate; if
                # userspace is late, the renderer simply holds the newest known
                # position until another measured sample is available.
                self._interp_interval_history_ns.append(dt_ns)
                self._interp_delay_ns = self._interpolated_delay_locked()
                self._sample_ns = int(sample_ns)
                self._sample_interval_ns = dt_ns
                self._input_samples += 1
                self._input_intervals_ms.append(dt_ns / 1_000_000.0)
                self._cond.notify_all()
                return

            # Map the newly measured absolute position through the previous
            # presentation offset.  If reaching that mapped measurement would
            # move *with* the newly observed velocity, preserve the real motion
            # that occurred since the last output tick.  If it would require a
            # correction opposite the new motion (or the new velocity is zero),
            # hold the last presented point instead: prediction debt is then
            # repaid by future real motion rather than by a visible snap-back.
            old_offset_x = self._presentation_offset_x
            old_offset_y = self._presentation_offset_y
            presented_x = self._last_target_x
            presented_y = self._last_target_y

            self._measured_x += float(dx)
            self._measured_y += float(dy)
            mapped_x = self._measured_x + old_offset_x
            mapped_y = self._measured_y + old_offset_y
            to_measured_x = mapped_x - presented_x
            to_measured_y = mapped_y - presented_y
            speed2 = vx * vx + vy * vy
            if speed2 > 1.0e-24 and (to_measured_x * vx + to_measured_y * vy) >= 0.0:
                base_x = mapped_x
                base_y = mapped_y
            else:
                base_x = presented_x
                base_y = presented_y

            self._presentation_offset_x = base_x - self._measured_x
            self._presentation_offset_y = base_y - self._measured_y

            # Adaptive mode changes only the extrapolation velocity. Rebase safety
            # above intentionally continues to use the newly observed physical
            # velocity (vx/vy). In adaptive mode the
            # already-updated 12.5% shadow candidate becomes the live velocity
            # for the interval until the next real physical report.
            if self.predictor == "adaptive":
                self._velocity_x_per_ns = self._shadow125_vx_per_ns
                self._velocity_y_per_ns = self._shadow125_vy_per_ns
            else:
                self._velocity_x_per_ns = vx
                self._velocity_y_per_ns = vy
            self._sample_ns = int(sample_ns)
            self._sample_interval_ns = dt_ns

            self._input_samples += 1
            self._rebases += 1
            self._input_intervals_ms.append(dt_ns / 1_000_000.0)
            self._rebase_distance.append(
                math.hypot(self._presentation_offset_x, self._presentation_offset_y)
            )
            self._cond.notify_all()

    def finish(self, reason: str = "pointer motion finished") -> None:
        """Finish a clean interpolated one-finger stroke by draining known data.

        Causal predictors stop immediately on the physical state transition.
        The interpolated mode intentionally presents already-measured motion
        about one source interval late, so a clean finger lift is allowed to
        drain only that known buffer.  Gesture/mode transitions still call
        reset() and cancel it immediately.
        """
        if not self.enabled:
            return
        if self.predictor != "interpolated":
            self.reset(reason)
            return
        with self._cond:
            if self._closed or not self._active or not self._interp_samples:
                return
            self._interp_finishing = True
            self._cond.notify_all()
        LOG.debug("pointer interpolated resampler draining known tail (%s)", reason)

    def reset(self, reason: str = "pointer state reset") -> None:
        """Stop extrapolation and start the next pointer session from zero."""
        if not self.enabled:
            return
        with self._cond:
            if self._closed:
                return
            had_state = self._active or self._sample_ns is not None
            self._generation += 1
            self._active = False
            self._next_tick_ns = None
            self._last_wake_ns = None
            self._measured_x = 0.0
            self._measured_y = 0.0
            self._presentation_offset_x = 0.0
            self._presentation_offset_y = 0.0
            self._velocity_x_per_ns = 0.0
            self._velocity_y_per_ns = 0.0
            self._sample_ns = None
            self._sample_interval_ns = 0
            self._interp_samples.clear()
            self._interp_interval_history_ns.clear()
            self._interp_delay_ns = 0
            self._interp_last_query_ns = None
            self._interp_finishing = False
            self._last_target_x = 0.0
            self._last_target_y = 0.0
            self._frac_x = 0.0
            self._frac_y = 0.0
            self._shadow_initialized = False
            self._shadow_latest_vx_per_ns = 0.0
            self._shadow_latest_vy_per_ns = 0.0
            self._shadow25_vx_per_ns = 0.0
            self._shadow25_vy_per_ns = 0.0
            self._shadow125_vx_per_ns = 0.0
            self._shadow125_vy_per_ns = 0.0
            self._shadow_prevprev_vx_per_ns = None
            self._shadow_prevprev_vy_per_ns = None
            self._cond.notify_all()
        if had_state:
            LOG.debug("pointer resampler reset (%s)", reason)

    def close(self) -> None:
        if not self.enabled:
            return
        with self._cond:
            self._closed = True
            self._active = False
            self._next_tick_ns = None
            self._cond.notify_all()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self._thread = None

    def _maybe_log_stats_locked(self, now_ns: int) -> None:
        if now_ns - self._last_stats_log_ns < self.stats_interval_ns:
            return
        self._last_stats_log_ns = now_ns
        if self._logical_ticks == 0 and self._input_samples == 0:
            return

        shadow_total = self._shadow_prediction_samples
        c25 = self._shadow_candidates["25"]
        c125 = self._shadow_candidates["125"]

        def pct(num: int, den: int) -> float:
            return (100.0 * num / den) if den else 0.0

        LOG.info(
            "resampler stats target=%.3fHz predictor=%s ticks=%d emits=%d skipped=%d inputs=%d rebases=%d "
            "tick_ms med=%.3f p95=%.3f lateness_ms med=%.3f p95=%.3f "
            "input_ms med=%.3f p95=%.3f prediction_age_ms med=%.3f p95=%.3f "
            "prediction_used_ms med=%.3f p95=%.3f rebase_units med=%.3f p95=%.3f max=%.3f "
            "shadow_samples=%d shadow_err_latest med=%.3f p95=%.3f "
            "shadow25_win_pct=%.1f shadow25_ties=%d shadow25_changed=%d shadow25_changed_win_pct=%.1f "
            "shadow25_err_adapt med=%.3f p95=%.3f shadow25_err_ratio med=%.3f p95=%.3f "
            "shadow25_changed_ratio med=%.3f p95=%.3f shadow25_long_abs med=%.3f p95=%.3f "
            "shadow25_lat_abs med=%.3f p95=%.3f "
            "shadow125_win_pct=%.1f shadow125_ties=%d shadow125_changed=%d shadow125_changed_win_pct=%.1f "
            "shadow125_err_adapt med=%.3f p95=%.3f shadow125_err_ratio med=%.3f p95=%.3f "
            "shadow125_changed_ratio med=%.3f p95=%.3f shadow125_long_abs med=%.3f p95=%.3f "
            "shadow125_lat_abs med=%.3f p95=%.3f "
            "shadow_long_abs_latest med=%.3f p95=%.3f shadow_lat_abs_latest med=%.3f p95=%.3f "
            "shadow_modes predictive=%d history=%d speedchange=%d turn=%d "
            "accelturn=%d accelmag=%d accelzero=%d start=%d stop=%d "
            "interp_delay_ms=%.3f interp_holds=%d interp_curve_ticks=%d interp_linear_ticks=%d",
            self.rate_hz,
            self.predictor,
            self._logical_ticks,
            self._nonzero_emits,
            self._skipped_ticks,
            self._input_samples,
            self._rebases,
            self._median(self._tick_intervals_ms),
            self._p95(self._tick_intervals_ms),
            self._median(self._lateness_ms),
            self._p95(self._lateness_ms),
            self._median(self._input_intervals_ms),
            self._p95(self._input_intervals_ms),
            self._median(self._prediction_age_ms),
            self._p95(self._prediction_age_ms),
            self._median(self._prediction_used_ms),
            self._p95(self._prediction_used_ms),
            self._median(self._rebase_distance),
            self._p95(self._rebase_distance),
            max(self._rebase_distance) if self._rebase_distance else 0.0,
            shadow_total,
            self._median(self._shadow_latest_error),
            self._p95(self._shadow_latest_error),
            pct(c25["wins"], shadow_total),
            c25["ties"],
            c25["changed"],
            pct(c25["changed_wins"], c25["changed"]),
            self._median(c25["adaptive_error"]),
            self._p95(c25["adaptive_error"]),
            self._median(c25["error_ratio"]),
            self._p95(c25["error_ratio"]),
            self._median(c25["changed_ratio"]),
            self._p95(c25["changed_ratio"]),
            self._median(c25["long_abs"]),
            self._p95(c25["long_abs"]),
            self._median(c25["lat_abs"]),
            self._p95(c25["lat_abs"]),
            pct(c125["wins"], shadow_total),
            c125["ties"],
            c125["changed"],
            pct(c125["changed_wins"], c125["changed"]),
            self._median(c125["adaptive_error"]),
            self._p95(c125["adaptive_error"]),
            self._median(c125["error_ratio"]),
            self._p95(c125["error_ratio"]),
            self._median(c125["changed_ratio"]),
            self._p95(c125["changed_ratio"]),
            self._median(c125["long_abs"]),
            self._p95(c125["long_abs"]),
            self._median(c125["lat_abs"]),
            self._p95(c125["lat_abs"]),
            self._median(self._shadow_latest_long_abs),
            self._p95(self._shadow_latest_long_abs),
            self._median(self._shadow_latest_lat_abs),
            self._p95(self._shadow_latest_lat_abs),
            self._shadow_mode_counts["predictive"],
            self._shadow_mode_counts["history"],
            self._shadow_mode_counts["speedchange"],
            self._shadow_mode_counts["turn"],
            self._shadow_mode_counts["accelturn"],
            self._shadow_mode_counts["accelmag"],
            self._shadow_mode_counts["accelzero"],
            self._shadow_mode_counts["start"],
            self._shadow_mode_counts["stop"],
            self._interp_delay_ns / 1_000_000.0,
            self._interp_hold_ticks,
            self._interp_curve_ticks,
            self._interp_linear_ticks,
        )

    def _run(self) -> None:
        while True:
            out_x = 0
            out_y = 0
            with self._cond:
                while not self._closed and (
                    not self._active or self._next_tick_ns is None
                ):
                    self._cond.wait(timeout=0.5)
                    self._maybe_log_stats_locked(time.monotonic_ns())

                if self._closed:
                    return

                now_ns = time.monotonic_ns()
                assert self._next_tick_ns is not None
                if now_ns < self._next_tick_ns:
                    self._cond.wait((self._next_tick_ns - now_ns) / 1_000_000_000.0)
                    continue

                # If userspace scheduling missed one or more logical samples,
                # do not burst stale REL events.  Skip directly to the newest
                # scheduled timestamp at/before now, preserving phase for the
                # next deadline.
                overdue_ns = now_ns - self._next_tick_ns
                missed = overdue_ns // self._period_ns
                if missed:
                    self._skipped_ticks += int(missed)
                    self._next_tick_ns += int(missed) * self._period_ns

                tick_ns = self._next_tick_ns
                self._next_tick_ns += self._period_ns

                wake_ns = time.monotonic_ns()
                if self._last_wake_ns is not None:
                    self._tick_intervals_ms.append(
                        (wake_ns - self._last_wake_ns) / 1_000_000.0
                    )
                self._last_wake_ns = wake_ns
                self._lateness_ms.append(max(0, wake_ns - tick_ns) / 1_000_000.0)

                target_x, target_y, age_ns, used_ns = self._predict_locked(tick_ns)
                self._prediction_age_ms.append(age_ns / 1_000_000.0)
                self._prediction_used_ms.append(used_ns / 1_000_000.0)

                logical_dx = target_x - self._last_target_x
                logical_dy = target_y - self._last_target_y
                self._last_target_x = target_x
                self._last_target_y = target_y

                self._frac_x += logical_dx
                self._frac_y += logical_dy
                out_x = int(self._frac_x)
                out_y = int(self._frac_y)
                self._frac_x -= out_x
                self._frac_y -= out_y

                self._logical_ticks += 1
                finish_interpolated_after_tick = bool(
                    self.predictor == "interpolated"
                    and self._interp_finishing
                    and self._interp_samples
                    and (tick_ns - self._interp_delay_ns) >= self._interp_samples[-1][0]
                )
                if out_x or out_y:
                    self._nonzero_emits += 1
                    # Emit while state is locked so a concurrent gesture/reset
                    # cannot let one stale predicted motion event escape after
                    # the state machine has suppressed pointer motion.
                    try:
                        self._emit_relative(out_x, out_y)
                    except Exception:
                        LOG.exception("pointer resampler output failed")

                if finish_interpolated_after_tick:
                    self._active = False
                    self._next_tick_ns = None
                    self._interp_finishing = False

                self._maybe_log_stats_locked(wake_ns)
