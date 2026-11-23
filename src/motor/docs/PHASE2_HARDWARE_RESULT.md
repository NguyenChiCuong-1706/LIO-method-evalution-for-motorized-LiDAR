# Phase 2 recorded hardware result

## Evidence reviewed

- Session: `absolute_home_logs/home_calibration_20261101T034928496478Z_0d659fe5.json`
- Baseline: `absolute_encoder_logs/absolute_encoder_20261101T034909404764Z.csv`
- Post-write, before power cycle: `absolute_encoder_logs/after_home_write_before_power_cycle/absolute_encoder_20261101T035100092781Z.csv`

## Measurements

| Stage | 0x94 single-turn | 0x92 multi-turn | 0x9C speed | 0x9C encoder |
|---|---:|---:|---:|---:|
| Before write | 19.39 deg | 19.39 deg | 0 deg/s | 3530 |
| After write, before power cycle | 0.00 deg | 0.00 deg | 0 deg/s | 0 |
| After operator-attested power cycle | 0.00 deg | 0.00 deg | 0 deg/s | 0 |

The initial and final pre-write stationary checks each contained five fresh
0x9C samples. All reported zero speed and stable encoder counts. The
power-cycle verification also contained five zero-speed, stable-encoder
samples.

## 0x19 response

- TX: `19 00 00 00 00 00 00 00`
- RX: `19 00 00 00 CA 0D 00 00`

The response does not use the documented V2.35 candidate layout expected by
the initial implementation. Bytes 4-5 contain `0x0DCA` (3530), matching the
pre-write 0x90 original encoder value. This is recorded as an observed
MHF9025-V3 layout; the precise field meaning remains unverified.

## Assessment

The controller-side evidence supports that the 0x19 operation established a
persistent zero: both angle commands changed from 19.39 degrees to zero and
remained zero after the operator-attested power cycle. The acknowledgement
alone is not proof of persistence; the post-power-cycle readings provide that
evidence.

The logs do not independently prove that the LiDAR was physically aligned to
the intended mechanical HOME or that the shaft did not move during the power
cycle. Those facts remain operator observations. Absolute angular accuracy
against an external reference also remains unmeasured.

No second 0x19 write is required or justified by these logs.
