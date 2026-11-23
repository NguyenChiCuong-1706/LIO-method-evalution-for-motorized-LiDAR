# Persistent Absolute HOME GUI — software validation

## Implementation

- `motor_control_gui.py` adds the Absolute HOME Calibration panel, guarded
  dialogs, direction sign, tolerance, angle displays, theta1, and worker signals.
- `motor_gui_core.py` keeps every calibration transaction on the existing GUI
  worker and transport. SET HOME is blocked during motion and until DISABLE is
  acknowledged.
- `absolute_home_calibration.py` remains the owner of session states, five-sample
  speed checks, the one-shot 0x19 write, V3 acknowledgement decoder, and
  power-cycle verification. It now also owns the five-sample 0x94 stability
  check and optional physical-mark verification confirmation.

## Interlocks

- Motor ID 1 / CAN 0x141 and 500000 bit/s are required.
- Known active fault candidates, active GUI motion, missing DISABLE
  acknowledgement, stale speed data, unstable encoder angle, or an incorrect
  phrase block the write.
- The session is marked `WRITE_SENT_UNVERIFIED` and flushed before 0x19.
- Timeout or malformed acknowledgement becomes `WRITE_RESULT_UNKNOWN`; there
  is no retry.
- Post-power-cycle verification uses 0x94 tolerance and explicit physical mark
  attestation. 0x92 is recorded but is not treated as persistent.
- No 0x91, PID write, ENABLE, speed, torque, or position command was added to
  the HOME workflow.

## Mocked coverage

Tests cover successful preparation and write acknowledgement, active motion,
missing DISABLE, stale telemetry, unstable 0x94, missing confirmation,
malformed acknowledgement, timeout, exactly one write attempt, power-cycle
verification, physical mark confirmation, nonpersistent 0x92, theta1 sign,
GUI status wording, worker serialization, and existing motor-control behavior.

No software-validation command opens `/dev/ttyUSB0` or accesses the physical
motor. A real 0x19 operation remains an explicit operator action in the GUI.

## Result

Python syntax checks passed. The complete software-only suite ran 150 tests in
the offscreen Qt environment; all 150 passed. This includes existing motion,
overspeed, telemetry, STOP/DISABLE, PID read-only, GUI shutdown, and diagnostic
regression tests.
