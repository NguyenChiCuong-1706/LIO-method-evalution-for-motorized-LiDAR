# Phase 3 — Test B and Test D preparation

## Scope

Only these read-only tests are implemented:

- Test B: absolute position recovery after power-off rotation
- Test D: powered, disabled, manual multi-turn tracking

The implementation does not contain Test A or Test C workflows. It does not
send ENABLE, DISABLE, STOP, motion, encoder-write, PID, firmware, `0x19`, or
`0x91` commands. The only allowed test requests are `0x90`, `0x94`, `0x92`,
and `0x9C`, each with seven zero data bytes.

## Test B behavior

The tool reads `0x90`, `0x94`, and `0x92`, closes the serial connection, and
waits while the operator disconnects motor power and performs the approximate
physical rotation. It reconnects only after the operator confirms power was
restored. It reports:

- the signed shortest wrapped `0x94` difference;
- the `0x92` difference without assuming that its multi-turn state persisted;
- consistency with the Phase 1 convention, where physical positive rotation
  produced a controller-negative angle change;
- controller evidence separately from the approximate physical observation.

Exact command:

```bash
cd /home/cuong/lm_calibr/src/motor
. .venv/bin/activate
python absolute_encoder_diagnostic.py \
  --mode absolute-recovery \
  --device /dev/ttyUSB0 \
  --can-bitrate 500000 \
  --motor-id 1 \
  --timeout 0.5 \
  --output-dir absolute_encoder_logs/phase3_test_b
```

## Test D behavior

The tool requires the operator to attest that the motor is disabled, powered,
safe to rotate, and mechanically suitable for manual rotation. It does not
disable the motor itself because this test is restricted to read commands.
After confirmation, manually rotate at least two revolutions without forcing
the mechanism.

Each complete sample reads fresh `0x92`, `0x94`, and `0x9C` frames. Request and
response times are recorded separately. Analysis unwraps `0x94`, compares it
with relative `0x92`, reports angle-pair timestamp skew, detects possible
sampling aliasing, and retains partial data on a communication error or Ctrl+C.

Exact 30-second command:

```bash
cd /home/cuong/lm_calibr/src/motor
. .venv/bin/activate
python absolute_encoder_diagnostic.py \
  --mode multi-turn-tracking \
  --device /dev/ttyUSB0 \
  --can-bitrate 500000 \
  --motor-id 1 \
  --timeout 0.5 \
  --duration 30 \
  --sample-interval 0.1 \
  --output-dir absolute_encoder_logs/phase3_test_d
```

If the shaft cannot be rotated safely while powered and disabled, do not run
Test D. A separate controlled-motion test would require explicit authorization.

## Outputs

Test B creates a raw-frame CSV and Markdown analysis report. Test D creates a
raw-frame CSV, Markdown analysis report, and standalone SVG angle-versus-time
plot. Both `0x92` and `0x94` originate in the same controller and therefore do
not independently establish physical angular accuracy. Zero `0x9C` speed is
also not treated as proof of physical stopping.

## Status

Software preparation and mocked validation are complete. No Phase 3 hardware
test was run by the implementation process. Actual Test B and Test D results
remain pending operator execution and review. LM-Calibr integration is outside
this phase.
