# Phase 6: experimental speed monitoring review

## Evidence and root cause

The local record `experiment_logs/experimental_20261030T072632630471Z.log`
contains one +5 RPM, 1-second request. The controller acknowledged
`A2 00 00 00 B8 0B 00 00`; it then returned `0x9C` speeds of 5 deg/s at
about 0.01 s and 35 deg/s at about 0.21 s. `motor_can.speed_payload()` encodes
RPM as `round(RPM * 600)`, so +5 RPM becomes signed int32 3000, representing
30 deg/s at 0.01 deg/s per command count. `decode_feedback()` reads bytes 4–5
as signed int16 whole deg/s and divides by 6 for RPM. Thus 35 counts is
5.833 RPM under the V2.35 interpretation.

The old experimental hard limit was an **absolute 5.5 RPM**, not a percentage
limit. A single `0x9C` sample above it triggered immediate cleanup. The
35-count sample exceeds the equivalent 33-count limit by 2 counts. The reverse
direction check independently stops on raw speed below -2 deg/s during a
positive request. The code slept up to 0.2 s between polls; the two observed
response times differ by about 0.203 s, including roughly 3 ms per CAN
request/response. The old code did not distinguish startup samples or require
multiple samples for the hard limit. It stopped after the second sample.

Feedback resolution of 1 deg/s is about 0.1667 RPM per count. Ideal rounding
uncertainty is at most 0.5 count, about 0.0833 RPM, before controller filtering
and calibration uncertainty. Raw 29, 30, 31, and 35 correspond to about 4.83,
5.00, 5.17, and 5.83 RPM. The 5-count difference between request and observed
raw speed cannot be explained by simple one-count quantization alone. It may
be a startup overshoot, filtering/timing artifact, controller tuning issue, or
an MHF9025-V3 scaling difference. Two samples do not establish steady-state
tracking. The 0x9C encoder moved 1041 raw counts between these samples, but
its scaling to mechanical output angle is unverified.

The earlier manual `0x92` readings changed from +6.16 to -404.46 degrees
under the V2.35 scale, a -410.62 degree change while the user reported about
one manual turn. This suggests the value responds to motion, but is not a
calibration of angle scale or direction. Both `0x9C` and `0x92` are produced by
the same controller; agreement is not independent physical proof.

## Monitoring policy

The **5.5 RPM experimental hard stop remains unchanged**. One sample above it,
unexpected reverse speed, invalid feedback, or missing communication still
aborts and attempts STOP/DISABLE. No available evidence establishes a safe
higher absolute ceiling, so changing this threshold solely to pass the latest
run would be unjustified. The normal `positive`/`negative` UNKNOWN-protection
gate remains unchanged.

For diagnosis only, a one-count (1 deg/s) band labels normal tracking, a
single deviation in the first 0.4 s is a startup transient *candidate*, and
two consecutive deviations or a later deviation are labeled tracking
deviation. These labels do not suppress the hard stop or extend the test.
They are provisional because the V3 scaling is unverified. A sustained tracking
deviation is reported but is not by itself a verified dangerous overspeed.

Each motion sample now logs a `monitor_sample_json` line with raw CAN data,
requested and reported speed, absolute and relative error, policy limits,
startup label, monotonic request/response times, round-trip time, polling
interval, encoder raw count, and protection UNKNOWN. `monitor_policy_json`
records the raw `0xA2` command. `stop_trigger_json` gives the stop reason.
The existing terminal lines still show STOP/DISABLE acknowledgments and
zero-speed telemetry. After acknowledged cleanup and confirmed near-zero
feedback, `0x92` and `0x9D` are read once for post-stop context; they are not
inserted into the timed motion loop. Failure of either optional read is logged
without retry or weakening cleanup.

The per-exchange timeout for the speed command and monitoring polls is now
capped by the experiment's remaining duration. The host still cannot guarantee
physical stop; USB/OS scheduling and CAN/controller latency remain present.

## Read-only angle comparison

`speed-angle-diagnostic` reads `0x92`, waits 0.2 s, reads `0x9C`, waits 0.2 s,
and reads `0x92` again. It sends no enable, speed, torque, position, stop, or
configuration command. The reusable `compare_speed_with_angle()` uses
monotonic request/response midpoints, signed raw 56-bit angle values, the
actual angle sample interval, and observed round-trip times. It reports the
interval-average angle-derived speed and its discrepancy from the bracketed
speed sample, or marks timing unusable if sample order, delay, or interval is
poor. A difference above one speed-feedback count is flagged as unresolved;
it may reflect an outlier, changing speed during the interval, or an incorrect
V3 scale. Raw values are preserved and no discrepancy is labeled a motor fault.

`_encoder_delta()` separately handles wraparound of the 16-bit raw encoder
counter. Neither encoder counts nor angle estimates are assumed to equal
physical output-shaft degrees without independent calibration.

## Next decision

This evidence does not justify another +5 RPM / 1-second trial with an
increased hard limit. The exact blocker is the lack of a verified safe
absolute speed ceiling and independently calibrated output-shaft speed/angle
measurement during startup. The existing experimental mode remains guarded
and will stop again if a sample exceeds 5.5 RPM. An external tachometer or
angle reference synchronized with CAN telemetry, plus more startup samples
from a safely bounded setup, would support a future threshold review.

The local copy of `LK-TECH motor control protocol(CAN) V2.35.pdf` was not
present. The V2.35 family command scales were cross-checked against a
[manufacturer-authored MF9025-family V2.35 protocol copy](https://www.scribd.com/document/826568712/%E7%93%B4%E6%8E%A7MF9025).
They remain unverified for this MHF9025-V3 controller.
