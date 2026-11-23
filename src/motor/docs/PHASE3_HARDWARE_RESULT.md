# Phase 3 recorded hardware result

## Test B — Absolute Position Recovery

Evidence:

- `absolute_encoder_logs/phase3_test_b/absolute_recovery_20261101T040252145409Z.csv`
- `absolute_encoder_logs/phase3_test_b/absolute_recovery_20261101T040252145409Z.md`

| Source | Before power-off rotation | After power restoration | Change |
|---|---:|---:|---:|
| 0x94 single-turn | 0.00 deg | 269.61 deg | -90.39 deg wrapped |
| 0x92 accumulated | 0.00 deg | -90.37 deg | -90.37 deg |
| 0x90 original raw | 0 | 49082 | -16454 counts wrapped |

Assuming 65536 raw encoder counts per turn only for this comparison, the
wrapped 0x90 change is -90.3845 degrees. It agrees with 0x92 to approximately
0.0145 degrees and with 0x94 to approximately 0.0055 degrees. These values all
originate in the motor controller and are not independent measurements.

The operator requested an approximate physical +90-degree movement. The
controller-negative result matches the direction convention observed in Phase
1. The magnitude error relative to the approximate physical request is 0.39
degrees, but this is not an encoder accuracy measurement because the physical
rotation was not measured independently.

Assessment: Test B passes. The stored HOME remained available after power was
removed, the shaft was moved, and power was restored. The motor reported the
new absolute position relative to HOME with internally consistent 0x94, 0x92,
and raw encoder values.

## Test D — Multi-turn Angle Tracking

Evidence:

- `absolute_encoder_logs/phase3_test_d/multi_turn_tracking_20261101T041032849269Z.csv`
- `absolute_encoder_logs/phase3_test_d/multi_turn_tracking_20261101T041032849269Z.md`
- `absolute_encoder_logs/phase3_test_d/multi_turn_tracking_20261101T041032849269Z.svg`

The recording contains 271 complete sample sets and 813 successful CAN
responses. No timeout, malformed response, or incomplete sample was recorded.
The mean complete-sample interval was 0.110706 seconds.

| Metric | Result |
|---|---:|
| Duration | 29.890 s |
| Initial 0x92 | -1624.03 deg |
| Final 0x92 | -4680.20 deg |
| Net 0x92 displacement | -3056.17 deg |
| Net revolution count | -8.4894 turns |
| Unwrapped 0x94 displacement | -3056.17 deg |
| End-to-end 0x92/0x94 disagreement | approximately 0.00 deg |
| Maximum instantaneous comparison difference | 1.31 deg |
| Median instantaneous comparison difference | 0.03 deg |
| Maximum 0x92/0x94 timestamp skew | 3.743 ms |
| Provisional peak motion during that skew | 1.374 deg |
| Largest sampled angle step | 39.24 deg |

The maximum instantaneous disagreement is smaller than the estimated motion
during the worst timestamp skew. The two commands were queried sequentially,
so this difference does not demonstrate encoder disagreement.

The accumulated path length was approximately 3098.19 degrees (8.6061 turns),
including about 21.01 degrees of brief positive-direction movement. Therefore
the 13 detected 0x94 zero-boundary crossings are crossing events, not a direct
revolution count. The net displacement comes from 0x92 and the separately
unwrapped 0x94 sequence.

No single sampled step approached 180 degrees, so the 0x94 unwrap was not
ambiguous at the observed sampling rate. The angle remained fixed at -4680.20
degrees for the final 10.62 seconds, while 0x9C speed ranged from -5 to +1
degrees/s; this is consistent with the previously observed low-speed estimator
noise, not independent proof of physical stopping.

Assessment: Test D passes. The recorded 0x92 signal tracked continuously well
beyond 360 and 720 degrees, 0x94 wrapped at the single-turn boundary, and the
unwrapped 0x94 progression agreed with the 0x92 displacement within the timing
uncertainty. Both signals originate from the same controller and do not
independently establish mechanical angle accuracy.

No Test A, Test C, motor command, or LM-Calibr integration was performed.
