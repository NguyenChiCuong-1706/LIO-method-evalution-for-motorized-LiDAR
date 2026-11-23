# Phase 7: output-shaft speed validation plan

## What the existing evidence establishes

The +5 RPM command was encoded as signed int32 `3000` in `0xA2`, following the
V2.35 command scale of 0.01 deg/s per count. The old log contains two in-motion
`0x9C` speed samples: 5 deg/s at 0.01 s and 35 deg/s at 0.21 s. Under the
V2.35 feedback scale of 1 deg/s per count, these are 0.83 and 5.83 RPM. The
second sample exceeded the unchanged 5.5 RPM hard stop (33 deg/s), so one
sample triggered STOP/DISABLE. The controller acknowledged both commands;
subsequent CAN feedback reached zero. That is controller telemetry, not
independent physical-stop verification. The two startup samples do not measure
steady-state speed or establish whether the excess was a transient, tuning
issue, or a V3 scaling discrepancy.

The `0x9C` encoder changed from raw 58419 to 59460 between those samples.
This confirms a controller-reported counter change, but its scale to the
output shaft is unknown. There are **no timed `0x92` samples during this motion
interval**, so angle-derived speed for the event is unavailable. Prior manual
rotation changed a provisional `0x92` angle by -410.62 degrees for roughly one
reported turn; this supports responsiveness, not calibrated scale or direction.
Both `0x92` and `0x9C` originate in the same controller and cannot serve as
independent physical verification.

`offline_speed_report.py` parses the saved log and prints these limits in JSON.
`speed-angle-diagnostic` is a separate **read-only** tool that brackets one
`0x9C` speed sample between two `0x92` angles using monotonic timestamps. It
reports the actual interval, CAN round-trip delays, a timing-only speed range,
and an unresolved discrepancy label. The 0.01 degree/count angle scale remains
provisional for MHF9025-V3, so the timing bounds are not full physical-speed
bounds. The diagnostic does not command motor movement.

## Protocol limits and read access

The available [MF9025-family CAN V2.35 protocol copy](https://www.scribd.com/document/826568712/%E7%93%B4%E6%8E%A7MF9025)
says `0xA2` speed is limited by **Max Speed** and acceleration by **Max
Acceleration** in the motor tool; it also mentions **Max Torque Current** for
the MF/MH/MG families. The copy describes `0x33` as a read of acceleration,
`0x30` as a read of PID gains, and `0x9A` as a status/error read. The motor
answered `0x90`, `0x92`, and `0x9D` in a previous read-only diagnostic, but
`0x30` and `0x33` timed out. Therefore this project has no measured
acceleration setting, PID values, Max Speed, or Max Torque Current from this
MHF9025-V3. The V2.35 copy does not establish a verified V3 read command for
Max Speed or Max Torque Current. Another V2.35 variant describes a generic
parameter read, but its compatibility with this controller is unverified;
this project does not send it. Verified V3 documentation or a read-only
inspection in LK Motor Tool is needed before treating those limits as known.

The existing `0x9A` voltage value of about 28.64 V is supported by one
independent supply measurement. Its V3 protection bit mapping remains
**UNKNOWN**. Byte 7 equal to zero does not establish fault-free status. The
software blocks legacy fault-candidate bits conservatively; it does not clear
errors or change motor parameters.

## Independent output-shaft video procedure for a later authorized test

1. With motor power disconnected and the shaft fully stopped, place a small,
   flush, high-contrast mark on the **rotating output shaft or flange**. Add a
   fixed reference mark on the nonrotating housing. Keep marks away from gaps,
   wiring, and any place they could catch. Do not attach a protruding pointer.
2. Secure the motor mechanically. Place the camera on a tripod outside the
   rotating area, aimed squarely at the output face. Keep the shaft and the
   terminal display in the same frame if possible; zoom optically or crop the
   recording afterward. The operator and camera must remain clear of the
   shaft. Keep the independent power cutoff within reach.
3. Record the original video frame rate and retain the unedited recording.
   Higher frame rate and clear marks reduce timing and angle uncertainty, but
   do not substitute for a known safe motion envelope. Record the frame where
   the terminal first visibly prints `TX 0x141: A2` if it is in view. That
   screen event occurs after host transmission; display and CAN/controller
   delays remain uncertain. If they cannot be bounded, omit CAN synchronization.
4. For each selected video frame, record its integer frame number and the
   **unwrapped** output-shaft angle in degrees relative to the fixed mark.
   Continue angles past 360 degrees for further turns; use negative values for
   the opposite direction. Record an honest angle-reading uncertainty for each
   endpoint. Do not infer angle from the controller encoder field.
5. Between two frames, compute
   `average_RPM = (angle_2 - angle_1) / 360 * 60 / ((frame_2 - frame_1) / fps)`.
   The offline script reports conservative bounds using ±1 frame at each
   endpoint plus the supplied angle uncertainty. These are **interval-average**
   output-shaft speeds; a `0x9C` value is a controller-reported sample and may
   differ during acceleration. Only compare times when the synchronization
   uncertainty is bounded and both measurements cover a relevant interval.

At 5 RPM the shaft turns only about 6.3 degrees in 0.21 s. A 5.83 RPM shaft
would turn about 7.35 degrees in the same period. That roughly 1-degree
difference is difficult to resolve with uncertain marker angles or frame
timing; a short video alone may not discriminate these speeds. No new physical
movement is requested by this plan.

## Offline commands

```bash
cd /home/cuong/lm_calibr/src/motor
. .venv/bin/activate
python offline_speed_report.py --log experiment_logs/experimental_20261030T072632630471Z.log
python waveshare_motor.py --mode speed-angle-diagnostic --device /dev/ttyUSB0 --can-bitrate 500000 --motor-id 1 --timeout 0.5
```

The first command only reads a file. The second sends only `0x92`, `0x9C`,
and `0x92` read requests. To prepare a future video measurement file, copy
`video_angles_template.csv` and fill it with **observed** frame numbers and
unwrapped output-shaft angles. Then run, substituting actual camera values:

```bash
python offline_speed_report.py --log experiment_logs/experimental_20261030T072632630471Z.log --video-csv video_angles.csv --fps FPS --angle-uncertainty-deg ANGLE_ERROR
```

If the video also contains the terminal A2 line and its timing uncertainty can
be bounded, add `--a2-frame FRAME --sync-uncertainty-s SECONDS`. Otherwise the
report computes physical average RPM without claiming time alignment to CAN.
No example motor measurements are supplied in the template.

## Decision before another supervised motion test

Keep the **5.5 RPM hard stop unchanged**. The current monitor distinguishes
one-sample critical overspeed, unexpected reverse speed, missing/invalid
telemetry, a small startup deviation, and later or repeated tracking errors.
Startup labels never suppress a hard stop. The exact blockers to reviewing
the limit or interpreting closed-loop performance are: no independent
output-shaft speed during the startup event; no verified V3 scaling and
protection definition; and no measured Max Speed, Max Acceleration, or Max
Torque Current settings. A further motion trial is not recommended until a
supervised plan addresses those points and the independent cutoff is ready.
