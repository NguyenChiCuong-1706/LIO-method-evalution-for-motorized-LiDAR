# Phase 2 — Persistent absolute HOME software implementation

## Scope

`absolute_home_calibration.py` is a separate calibration tool. No GUI, PID,
motor-control, STOP/DISABLE, or existing safety-limit code was changed. Phase 2
software was tested with mocked transport only; command `0x19` was not sent to
physical hardware.

The default `inspect` mode and `prepare` mode are read-only. A prepared JSON
session is required before `write` can reach the sole `0x19` call site. The
session file is exclusively locked during write and verification.

## Interlocks before a write

1. Read fresh `0x90`, corrected V3 `0x94`, and `0x92` values.
2. Read five fresh `0x9C` samples at approximately 0.1 s intervals.
3. Require all reported speeds within ±4°/s and all encoders within ±2 counts.
4. Require speed-feedback age no greater than 0.25 s.
5. Display `19 00 00 00 00 00 00 00` and the ROM-write warning.
6. Require exact interactive confirmations:
   - `MECHANICAL_HOME_ALIGNED`
   - `ROTATING_ASSEMBLY_SAFE`
   - `NO_ACTIVE_MOTION_COMMAND`
   - `WRITE_ABSOLUTE_HOME`
7. Repeat the five-sample stationary check after the human interaction.
8. Commit the one-shot write guard to disk before making one `0x19` exchange.

`0x9C` has no verified MHF9025-V3 field that proves another controller has no
queued movement command. The tool therefore combines observed stationarity,
exclusive ownership of its session, no motion commands of its own, and an
explicit operator confirmation. Other CAN controllers must be disconnected or
otherwise prevented from commanding motion.

## Write result handling

The proposed standard CAN frame is ID `0x141`, DLC 8:

```text
19 00 00 00 00 00 00 00
```

The provisional V2.35 acknowledgement layout is command `0x19`, five zero
bytes, and a uint16 encoder offset in bytes 6–7. A valid reply produces
`WRITE_ACKNOWLEDGED_UNVERIFIED`. Timeout, transport error, or malformed reply
produces `WRITE_RESULT_UNKNOWN`. Neither state permits another write from the
same session. No automatic retry exists.

## Session log

Each JSON file in `absolute_home_logs/` contains:

- UUID calibration session identifier and UTC timestamps
- device, bitrate, standard CAN ID and proposed frame
- raw TX/RX plus decoded `0x90`, `0x94`, `0x92`, and every `0x9C` sample
- communication and observed motor state
- speed-feedback age and stationary-check evidence
- write-attempt timestamp, exact TX/RX, acknowledgement classification and error
- state-transition history
- post-power-cycle raw measurements, tolerance and verification result
- an explicit flag that physical HOME alignment was not independently measured

Supported states are:

```text
NOT_CALIBRATED
READY_TO_WRITE
WRITE_SENT_UNVERIFIED
WRITE_ACKNOWLEDGED_UNVERIFIED
WRITE_RESULT_UNKNOWN
VERIFIED_AFTER_POWER_CYCLE
FAILED
```

`VERIFIED_AFTER_POWER_CYCLE` requires the operator's exact power-cycle
attestation plus fresh stationary feedback and both `0x94` circular-zero error
and absolute `0x92` error within the selected tolerance. This verifies the
controller readings after the reported power cycle; it is not independent
mechanical metrology.

## Manual commands

Read the current encoder and angles without preparing a write:

```bash
cd /home/cuong/lm_calibr/src/motor
. .venv/bin/activate
python absolute_encoder_diagnostic.py --device /dev/ttyUSB0 --can-bitrate 500000 --motor-id 1 --timeout 0.5
```

After physically aligning the LiDAR to mechanical HOME, prepare and inspect the
operation without writing:

```bash
python absolute_home_calibration.py --mode prepare --device /dev/ttyUSB0 --can-bitrate 500000 --motor-id 1 --timeout 0.5
```

Copy the printed `SESSION_FILE` path. Only during a separately authorized
physical experiment, run:

```bash
python absolute_home_calibration.py --mode write --session /absolute/path/from/SESSION_FILE
```

The program requests all four phrases interactively. Afterward, make a read-only
record before power cycling:

```bash
python absolute_encoder_diagnostic.py --device /dev/ttyUSB0 --can-bitrate 500000 --motor-id 1 --timeout 0.5 --output-dir absolute_encoder_logs/after_home_write_before_power_cycle
```

For later verification, turn motor power off, do not move the marked shaft,
restore power, then run:

```bash
python absolute_home_calibration.py --mode verify --session /absolute/path/from/SESSION_FILE --verification-tolerance-deg 1.0
```

This implementation does not send `0x91`, `0x95`, ENABLE, speed, torque,
position, PID, STOP, or DISABLE commands.
