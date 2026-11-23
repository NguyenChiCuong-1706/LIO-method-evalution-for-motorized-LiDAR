"""Pure, provisional speed and angle analysis for MF9025-family telemetry."""

from __future__ import annotations

from dataclasses import dataclass
import math


# Retain the existing experimental hard stop. One 5.83 RPM sample does not
# establish enough evidence to change the physical safety limit.
EXPERIMENT_HARD_SPEED_RPM = 5.5
TRACKING_BAND_DPS = 1  # One whole 0x9C feedback count, V2.35 interpretation.
STARTUP_LABEL_WINDOW_S = 0.4  # Logging label only; never suppresses a hard stop.


@dataclass(frozen=True)
class TimedRawSample:
    raw: int
    request_started_mono: float
    received_mono: float

    @property
    def midpoint_mono(self) -> float:
        return (self.request_started_mono + self.received_mono) / 2

    @property
    def round_trip_s(self) -> float:
        return self.received_mono - self.request_started_mono

    def validate(self) -> None:
        if (not math.isfinite(self.request_started_mono) or
                not math.isfinite(self.received_mono) or
                self.received_mono < self.request_started_mono):
            raise ValueError("invalid monotonic sample timestamps")


def classify_speed(raw_dps: int, requested_rpm: float, elapsed_s: float,
                   consecutive_deviations: int = 0, *, experimental: bool = True,
                   hard_limit_rpm: float | None = None) -> dict:
    """Separate hard stops from tracking labels; do not relax existing limits."""
    if not math.isfinite(requested_rpm) or not math.isfinite(elapsed_s) or elapsed_s < 0:
        raise ValueError("invalid speed request or monotonic elapsed time")
    if consecutive_deviations < 0:
        raise ValueError("negative deviation count")
    reported_rpm = raw_dps / 6.0
    error_rpm = reported_rpm - requested_rpm
    requested_dps = requested_rpm * 6
    if hard_limit_rpm is not None and (not math.isfinite(hard_limit_rpm) or hard_limit_rpm <= 0):
        raise ValueError("hard speed limit must be positive and finite")
    hard_limit = (hard_limit_rpm if hard_limit_rpm is not None else
                  EXPERIMENT_HARD_SPEED_RPM if experimental else 10.5)
    # Keep the existing reverse-direction condition unchanged.
    reverse = raw_dps * requested_rpm < -2 * abs(requested_rpm)
    hard_overspeed = abs(reported_rpm) > hard_limit
    deviates = abs(raw_dps - requested_dps) > TRACKING_BAND_DPS
    count = consecutive_deviations + 1 if deviates else 0
    if hard_overspeed or reverse:
        category = "critical_failure"
        reason = "hard_overspeed" if hard_overspeed else "unexpected_reverse_speed"
    elif not deviates:
        category, reason = "normal_tracking", "within_one_feedback_count"
    elif elapsed_s <= STARTUP_LABEL_WINDOW_S and count < 2:
        category, reason = "startup_transient_candidate", "single_tracking_deviation"
    else:
        category, reason = "tracking_deviation", "sustained_or_late_deviation"
    return {
        "category": category, "reason": reason,
        "requested_rpm": requested_rpm, "requested_dps": requested_dps,
        "reported_speed_raw_dps": raw_dps, "reported_rpm_provisional": reported_rpm,
        "signed_tracking_error_rpm": error_rpm,
        "absolute_tracking_error_rpm": abs(error_rpm),
        "relative_tracking_error_fraction": abs(error_rpm) / abs(requested_rpm)
        if requested_rpm else None,
        "hard_speed_limit_rpm": hard_limit,
        "tracking_band_dps": TRACKING_BAND_DPS,
        "startup_label_window_s": STARTUP_LABEL_WINDOW_S,
        "consecutive_tracking_deviations": count,
        "startup_or_steady": "startup" if elapsed_s <= STARTUP_LABEL_WINDOW_S else "later_sample",
    }


def compare_speed_with_angle(before: TimedRawSample, speed: TimedRawSample,
                             after: TimedRawSample) -> dict:
    """Compare speed with bracketing 0x92 angles using monotonic midpoints.

    The two values come from the same controller and are not physical proof.
    The estimate averages an interval; it is not simultaneous with 0x9C.
    """
    for sample in (before, speed, after):
        sample.validate()
    interval = after.midpoint_mono - before.midpoint_mono
    result = {
        "angle_before_raw": before.raw, "angle_after_raw": after.raw,
        "speed_raw_dps": speed.raw,
        "angle_delta_raw": after.raw - before.raw,
        "angle_interval_s": interval,
        "speed_sample_midpoint_mono": speed.midpoint_mono,
        "speed_sample_rtt_s": speed.round_trip_s,
        "angle_before_rtt_s": before.round_trip_s,
        "angle_after_rtt_s": after.round_trip_s,
        "speed_reported_rpm_provisional": speed.raw / 6,
        "source": "0x9C and 0x92 from the same controller; not independent physical proof",
    }
    timing_uncertainty = (before.round_trip_s + after.round_trip_s) / 2
    result["angle_interval_uncertainty_s"] = timing_uncertainty
    result["angle_scale"] = "0.01 deg/count per V2.35; unverified for MHF9025-V3"
    if (interval < 0.05 or interval > 2 or
            not before.midpoint_mono <= speed.midpoint_mono <= after.midpoint_mono or
            max(sample.round_trip_s for sample in (before, speed, after)) > 0.1 or
            timing_uncertainty > interval / 4):
        result.update(quality="unusable_timing", speed_from_angle_rpm_provisional=None,
                      speed_from_angle_timing_bounds_rpm=None,
                      discrepancy_rpm=None, consistency="not_evaluated")
        return result
    angle_rpm = ((after.raw - before.raw) / 100) / interval / 6
    signed_delta_per_minute = ((after.raw - before.raw) / 100) / 6
    timing_bounds = sorted((signed_delta_per_minute / (interval - timing_uncertainty),
                            signed_delta_per_minute / (interval + timing_uncertainty)))
    discrepancy = (speed.raw / 6) - angle_rpm
    result.update(quality="timing_acceptable_provisional",
                  speed_from_angle_rpm_provisional=angle_rpm,
                  speed_from_angle_timing_bounds_rpm=timing_bounds,
                  discrepancy_rpm=discrepancy,
                  consistency="difference_above_one_feedback_count_unresolved"
                  if abs(discrepancy) > 1 / 6 else "within_one_feedback_count_provisional")
    return result
