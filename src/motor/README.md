# MHF9025-V3 CAN motor smoke test

## Phase 2 persistent HOME calibration tool

`absolute_home_calibration.py` implements a separately gated, one-shot `0x19`
ROM-write workflow. Its default `inspect` and explicit `prepare` modes are
read-only. Writing requires a prepared session JSON, multiple fresh stationary
samples, four exact interactive confirmations, and a second stationary check.
The session is marked attempted before transmission and can never retry `0x19`.
A reply remains unverified until a later operator-confirmed power cycle and
read-only angle check. See [PHASE2_ABSOLUTE_HOME.md](docs/PHASE2_ABSOLUTE_HOME.md).

Phase 2 software tests do not access `/dev/ttyUSB0` and do not send a physical
ROM write.

## Phase 1 absolute-encoder diagnostic

`absolute_encoder_diagnostic.py` is an independent read-only tool for commands
`0x90`, `0x94`, `0x92`, and `0x9C`. It uses the existing Waveshare serial
transport on `/dev/ttyUSB0`, standard CAN ID `0x141`, and 500000 bit/s. Each
command is sent once with a bounded timeout, raw request/response frames are
printed, and a timestamped CSV is written under `absolute_encoder_logs/`.
V2.35 decoding is explicitly provisional for MHF9025-V3. See
[`PHASE1_ABSOLUTE_ENCODER.md`](docs/PHASE1_ABSOLUTE_ENCODER.md) for the decoder table and manual test record.

```bash
cd /home/cuong/lm_calibr/src/motor
. .venv/bin/activate
python absolute_encoder_diagnostic.py --device /dev/ttyUSB0 --can-bitrate 500000 --motor-id 1 --timeout 0.5
```

This utility contains no encoder-offset/HOME write, PID write, ENABLE, motion,
STOP, or DISABLE sequence. Phase 1 does not establish persistent HOME.

## PySide6 motor-control dashboard

`motor_control_gui.py` is a desktop engineering dashboard for the Waveshare
serial-CAN adapter. It has one worker thread and one serial owner. Connecting
does not send ENABLE or a motion command. The GUI polls `0x9C`, `0x92`, and
`0x9A`, displays provisional V2.35 speed/angle interpretations, maintains
bounded plots, and records session telemetry under `motor_logs/`.

The speed panel separates fresh `0x9C` raw speed, a timestamp-aware low-pass
display value, and average speed derived from successive fresh `0x92` angles.
The display filter uses a 0.25 s time constant and can be hidden. It does not
feed the overspeed, direction, communication, or stop logic. CSV rows identify
their fresh source (`0x9A`, `0x9C`, or `0x92`), so cached values carried into a
different response row are not counted as new observations. New logs also
record request/response time, the angle interval, host timing uncertainty, and
structured STOP/DISABLE/controller-zero feedback results. Controller feedback
does not independently prove physical shaft motion or physical stopping.

The PID panel is read-only. It can try V2.35 command `0x30` and display its six
raw bytes, but earlier MHF9025-V3 requests timed out. The GUI has no `0x31` RAM
write or `0x32` ROM write path. PID meanings, scaling, and compatibility remain
unverified for this motor.

### Persistent Absolute HOME panel

The GUI now reuses `absolute_home_calibration.py` for guarded persistent HOME
calibration. All HOME transactions run on the existing worker thread and the
same serial transport owner. The panel displays fresh `0x94` single-turn angle,
`0x92` accumulated angle, and LM-Calibr `theta1`. The default mounting uses
`theta1 = +1 * motor_angle` for the operator-observed mounting: a positive speed
request decreased both the accumulated encoder angle and physical `theta1`.
`direction_sign` can be changed to `-1` for a different mounting. The GUI
uses `0x94` as the startup/single-turn absolute reference and `0x92` only as the
continuous angle while running. The single-turn angle is shown as a signed
equivalent for `theta1` (for example, 270 degrees becomes -90 degrees). It does
not assume that `0x92` persists across power cycles. GO HOME first measures
the sign of fresh encoder-angle change at 0.5 RPM, then calculates the travel
to zero using that observed sign; a missing direction or angle moving away
aborts with STOP/DISABLE cleanup.

To set HOME:

1. Connect using `/dev/ttyUSB0`, 500000 bit/s, motor ID 1 (`0x141`).
2. With motor power off, manually align the LiDAR to the desired
   mechanical HOME. Restore power, reconnect the GUI, then press
   **SET HOME (ROM WRITE)**.
3. The GUI sends STOP/DISABLE and requires their
   acknowledgments and settled speed/encoder feedback before continuing.
4. The backend reads status, five fresh `0x9C` samples, encoder and angles, and
   five fresh stable `0x94` samples. Review the displayed old angle and exact
   `19 00 00 00 00 00 00 00` frame.
5. Type exactly `WRITE_ABSOLUTE_HOME`. The GUI commits the session log before
   sending `0x19` once. It never retries an unknown result.
6. A valid response remains `WRITE_ACKNOWLEDGED_UNVERIFIED`. Disconnect, power
   cycle the motor without moving the marked shaft, restore power, and reconnect.
7. Check that the physical mark is still at HOME, then use the separate
   `absolute_home_calibration.py --mode verify --session PATH_TO_SESSION_JSON`
   command if a recorded power-cycle verification is needed. This command is
   read-only; it requires operator confirmation.

The GUI does not initiate HOME verification. It displays a previously verified
session only when that verification is recorded, and never treats a zero CAN
angle by itself as proof of mechanical alignment. `0x92` and `0x90` are logged
during the separate verification but do not prove mechanical alignment. Calibration JSON logs are
stored under `absolute_home_logs/` and include the previous angle, stationary
samples, exact TX/RX frames, confirmation result, session ID, write state, and
verification evidence. No HOME write occurs at startup or from **READ ANGLE**.
On reconnect the GUI restores the newest saved HOME write session for CAN
`0x141`. **READ ANGLE** refreshes the displayed angles without changing the
saved verification result. The separate calibration tool records power-cycle
verification. No new `0x19` write is sent during angle reading or verification.

### GO HOME button

**GO HOME** in the bounded motor control and Absolute HOME panels moves to the already stored
absolute encoder zero without starting the target speed and duration run. It
uses the HOME approach speed and verification tolerance shown in the GUI,
performs the same stationary preflight and `OK` confirmation as START, then
uses the existing bounded auto HOME routine. The button remains available when
**Auto HOME before START** is unchecked. Session frames and feedback are saved
under `motor_logs/`. The STOP button interrupts the operation, and failures
attempt STOP/DISABLE; a successful GO HOME that moved the shaft ends with STOP
only and leaves the controller enabled. **GO HOME** never writes a new encoder zero; use
**SET HOME (ROM WRITE)** only for an explicitly authorized calibration. It
confirms STOP/DISABLE before presenting the ROM-write confirmation.

Launch the hardware GUI:

The **CAN bitrate** selector offers 500000 and 1000000 bit/s. The current
motor answered read-only status requests at 1000000 bit/s on 2026-11-05, so
the GUI selects that rate by default. Choose the rate that matches the motor;
changing this selector configures the Waveshare adapter, not motor firmware.

```bash
cd /home/cuong/lm_calibr/src/motor
. .venv/bin/activate
python motor_control_gui.py
```

### Continuous ROS 2 rotation through successive 360° turns

`./motor.sh` now launches `rotate_360.launch.py`. It first returns to the
nearest mechanical HOME turn, checks positive joint direction at 0.5 RPM,
then keeps rotating in the same direction at the default 50 RPM. The node
publishes fresh `0x92` position and signed velocity to `/joint_states` and
reports each completed 360° turn. There is no default runtime limit; Ctrl+C,
CAN failure, overspeed, angle discontinuity, reverse rotation, or
stalled progress; cleanup attempts STOP and DISABLE. The 80 RPM speed feedback
threshold requires five consecutive fresh `0x9C` samples
above 80 RPM in continuous mode. A reading above 90 RPM still stops
immediately. At the default 25 ms poll period, five samples span roughly
0.1 seconds; serial or scheduling delays can make this longer. Homing and the
older ±45° mode still stop at their original single-sample limits. Protection
UNKNOWN handling remains in effect.
Continuous-mode angle travel is checked against the actual elapsed time
between fresh `0x92` replies, so a short ROS bag recording delay does not
look like an impossible fixed-period angle jump. Missing telemetry and the
separate `0x9C` speed checks still stop motion.
Motor motion begins automatically after HOME; secure the assembly and keep
an independent power cutoff ready. Only this node may own `/dev/ttyUSB0`.

```bash
cd /home/cuong/lm_calibr
source /opt/ros/kilted/setup.bash
colcon build --packages-select mhf9025_joint_state --symlink-install
source install/setup.bash
./motor.sh
```

`motor.sh` logs CAN frames to `src/motor/experiment_logs/rotate_360_*.log`.
Use `./motor.sh sweep_rpm:=50 max_runtime_s:=0` to specify the current
defaults explicitly. Set `max_runtime_s` to 1–600 for a bounded run. The
`oscillate_45.launch.py` launch remains available
for the older ±45° behavior.

### Experimental ROS 2 ±45° oscillation

`oscillating_motor_node` opens the Waveshare adapter itself, reads the stored
single-turn HOME reference (`0x94`), returns to the nearest HOME turn using
fresh `0x92` feedback, and then moves `theta1` between +45° and -45° while
publishing fresh `0x92` position and
signed velocity on `/joint_states`. HOME still uses slow 1°/s `0xA4` position
commands. The ±45° sweeps use the GUI's `0xA2` speed command, up to 20 RPM.
Each leg begins at 0.5 RPM to check direction, commands 20 RPM while the
reported joint angle is between -40° and +40°, and commands 10 RPM in the
outer 5° at either end. Fresh feedback is polled every 25 ms; a STOP is
requested within 1.5° of the endpoint to allow for command and feedback delay.
The actual transition and stopping angles depend on controller response.
The node sends STOP near the endpoint,
requires five settled speed samples, and only then enables the reverse leg.
It stops if reported sweep speed exceeds 24 RPM at the default 20 RPM request.
Negative `0xA2` direction during reverse sweeps remains experimental on this
installation and needs supervised physical verification.
Default maximum runtime is 300 s;
then it sends STOP and DISABLE. A CAN error, overspeed, angle jump, stalled
progress, or Ctrl+C also attempts STOP and DISABLE. Controller acknowledgements
do not prove physical stopping. The MHF9025-V3 `0xA4` travel behavior is
experimental, and protection decoding remains UNKNOWN.

Secure the motor and LiDAR, verify the marked HOME reference physically, clear the
rotating area, and keep an independent power cutoff ready before launch.
Close the GUI and the separate read-only joint-state node first: this node
must be the only owner of `/dev/ttyUSB0`. The launch default is 500000 bit/s,
matching the current motor configuration.

```bash
cd /home/cuong/lm_calibr
source /opt/ros/kilted/setup.bash
colcon build --packages-select mhf9025_joint_state
source install/setup.bash
ros2 launch mhf9025_joint_state oscillate_45.launch.py
```

The ±45° launch prints its CAN TX/RX frames to the console.

This launch may start physical motion immediately to return HOME. The 300-second limit covers
both homing and oscillation; if it expires during homing, STOP/DISABLE is sent.
Press Ctrl+C to request STOP followed by DISABLE at any point. To override the
bitrate, pass `can_bitrate:=<value>`. `sweep_rpm:=20` is the maximum allowed
by this node; `direction_sign:=-1` changes the joint convention if
the physical mounting differs. Only one motor controller may run at a time.

### Read-only ROS 2 joint feedback

Run one publisher at a time. The GUI can optionally publish `/joint_states`
using its existing USB-CAN connection. The GUI worker owns `/dev/ttyUSB0`; its ROS publisher uses
the worker's fresh `0x92` telemetry signals, not the GUI's coalesced display
rows. On connection, the same worker reads `0x94` and `0x92` to align the
single-turn HOME reference with the live multi-turn count. The first fresh
`0x92` frame anchors the velocity calculation; each later usable frame produces
one `sensor_msgs/msg/JointState` with `name[0] = lidar_joint`, `position[0]`
equal to `theta1` in radians, and signed `velocity[0]` in radians per second.
The GUI `direction_sign` is fixed for the connection while publishing.

Each message retains the CAN sample's UTC time, adjusted to the midpoint of
its request/response interval. The motor does not provide an independent
acquisition timestamp. Check this clock against LiDAR header timestamps in a
recorded bag before calibration. Missing, delayed, malformed, or implausible
angle updates are omitted rather than republished as fresh data. After a motor
power cycle, disconnect and reconnect the GUI so it reads a new `0x94` reference.

To publish without opening the GUI, close the GUI first and start the read-only
standalone ROS 2 node. It owns `/dev/ttyUSB0` for its lifetime and uses the
same `JointStateEstimator` and V3 angle decoders as the GUI. On startup it
reads `0x94` and `0x92`; afterward it polls fresh `0x92` every 0.125 s.
It sends no ENABLE, speed, torque, position, HOME-write, STOP, or DISABLE
commands. A missing or malformed response exits without automatic retry.
The shared Waveshare transport takes an exclusive advisory lock so a second
process using this code cannot own the device simultaneously.

Build the ROS 2 package once, then run its installed node. Use a terminal
without the motor GUI's virtual environment activated:

```bash
cd /home/cuong/lm_calibr
source /opt/ros/kilted/setup.bash
colcon build --packages-select mhf9025_joint_state --symlink-install
source install/setup.bash
ros2 run mhf9025_joint_state joint_state_node --device /dev/ttyUSB0 --can-bitrate 500000 --motor-id 1
```

The same installed node is available through its launch file:

```bash
source /opt/ros/kilted/setup.bash
cd /home/cuong/lm_calibr
source install/setup.bash
ros2 launch mhf9025_joint_state joint_states.launch.py
```

Stop with Ctrl+C. The default `direction_sign=+1` matches the observed
mounting; `--direction-sign -1` is available for another mounting. Check
LiDAR and joint header clock alignment before calibration. This node provides
feedback only; it does not move the motor.

Launch the GUI with publishing enabled (ROS 2 Kilted installed locally):

```bash
source /opt/ros/kilted/setup.bash
cd /home/cuong/lm_calibr/src/motor
. .venv/bin/activate
export PYTHONPATH="/usr/lib/python3/dist-packages:${PYTHONPATH}"
python motor_control_gui.py --publish-joint-states
```

The `PYTHONPATH` line makes the Ubuntu-installed PyYAML visible to this venv;
it can be omitted once PyYAML is installed in `.venv`. Install the other GUI
dependencies with `python -m pip install -r requirements.txt` if needed.

Test the same path without motor hardware:

```bash
python motor_control_gui.py --simulate --publish-joint-states
```

From another ROS terminal, inspect the topic without opening the CAN adapter:

```bash
source /opt/ros/kilted/setup.bash
ros2 topic echo /joint_states
```

For a separately authorized, supervised calibration recording, capture the
LiDAR and joint topics together with `ros2 bag record -o BAG_PATH /livox/lidar
/joint_states`. Do not infer physical angle accuracy from CAN feedback alone.

Analyze the recorded 90 RPM session without opening the adapter:

```bash
python telemetry_analysis.py motor_logs/motor_session_20261030T083514061588Z.csv
```

Hardware motion remains restricted to the positive protocol direction, at
more than 0 and at most **100 RPM**, for at most **20 seconds**. Negative values
are represented by the input widget but rejected for hardware motion until
direction is independently validated. The GUI emergency hard stop is
**100 RPM**. Each START performs status and stationary
preflight first, then asks for the exact operator confirmation immediately
before ENABLE; type `OK` in that dialog. With **Auto HOME before START** enabled
(the default), START first reads fresh `0x94`, approaches the stored encoder zero
using a positive speed command. It first observes the actual encoder-angle
direction at 0.5 RPM, then uses the configured 0.5–5 RPM coarse speed and
returns to 0.5 RPM for the final 10 degrees. Missing direction feedback aborts
within 1.5 seconds; the subsequent distance-aware timeout is at most 130
seconds. The active approach uses an internal capture band of 40% of the GUI
tolerance, capped at 0.4 degrees. The configured GUI tolerance remains the
post-STOP acceptance limit. This margin accounts for the measured approximately
0.2 degree position change while the controller settles after STOP.
It then sends STOP only, requires five fresh final angle samples inside
the HOME tolerance, and repeats the stationary preflight before beginning the
selected speed and duration. Normal Auto HOME uses `0x88`, `0xA2`, `0x81`, and
read commands only; it never sends the persistent HOME-write command `0x19`.
After STOP, the controller waits for three consecutive settled `0x9C`
speed and encoder samples within a bounded ten-sample window. This accommodates
the measured short deceleration interval without accepting a continuously
moving shaft.
The controller-reported zero does not independently verify physical mechanical
alignment. Protection remains **UNKNOWN**. Normal STOP and duration expiry
send `0x81` without `0x80`; the GUI reports `STOPPED_ENABLED`. A failed STOP
confirmation attempts STOP/DISABLE and reports failure. Communication failure,
overspeed, DISABLE, disconnect, and application shutdown attempt STOP followed
by DISABLE. STOP feedback does not prove physical stopping or position holding;
software cannot replace the independent physical power cutoff.

These 100 RPM / 20 second limits apply to the desktop GUI. The older
`waveshare_motor.py --mode experimental-motion` CLI retains its separate
historical 5 RPM / 2 second validation rules.

### Read-only PID panel

The GUI exposes the V2.35 `0x30` readback fields as six raw uint8 values:
Angle Kp/Ki, Speed Kp/Ki, and Torque Kp/Ki. Their scaling and compatibility
remain unverified for MHF9025-V3. Press **READ PID (0x30)** while connected and
idle. The fields cannot be edited. The GUI implements no PID write command.
If `0x30` times out, it reports the timeout and does not infer parameter values.
PID changes are deferred until V3 compatibility, existing values, and an
independently measured control problem are established.

### Live torque-current feedback

During a connected session the GUI shows each fresh `0x9C` signed torque-current
sample (bytes 2–3) and its age. LK V2.35 identifies this field as torque
current, but its sections give inconsistent current ranges for the motor
families, and the scale has not been verified on MHF9025-V3. The raw count is
saved in the telemetry CSV only for rows with a
fresh `0x9C` response. After 0.5 seconds without a new sample the value is
hidden. These are controller reports, not an independent torque measurement;
there is no validated conversion to N·m.

The bounded speed session now ends with STOP (`0x81`) only on normal completion;
the motor remains enabled until the operator presses DISABLE, disconnects, or
closes the GUI. Without the experimental hold option below, this does not
establish verified continuous position holding.
Validated torque adjustment and continuous holding require verified V3 command
semantics, current and thermal limits, and a measured LiDAR load. The GUI sends
no torque command.

### Experimental position hold

The **Experimental position hold after GO HOME / START** checkbox is off by
default. With explicit `OK` motion confirmation, GO HOME uses fresh matching
`0x94` and `0x92` readings to target the nearest encoder HOME with provisional
V2.36 `0xA4` directly from speed mode, without an intermediate STOP. At normal
duration expiry, the GUI first commands 0.5 RPM with `0xA2`, requires fresh
speed and angle samples to settle, then sends `0xA4` at the fresh `0x92` angle.
The low-speed check accepts the observed 0/5 dps quantization of `0x9C` near
0.5 RPM only when timestamped `0x92` angle changes also remain below 6 dps
for eight consecutive samples. It still rejects reverse travel and excessive
speed, and the hold monitors angle drift and speed after the transition.
This low-speed transition may extend the time before the motor is stationary
by at most about 1.5 seconds. The `0xA4` speed limit is 1 degree/s; no current
limit or PID setting is changed. Five fresh angle/speed pairs are checked
before reporting `POSITION_HOLD`; idle polling then watches drift, speed,
voltage and temperature. Operator STOP and every failed transition still use
the immediate STOP path, with DISABLE on fault. DISABLE, disconnect and GUI
close end the hold. V3 position-command compatibility and actual holding torque
have **not** been tested under load. Acknowledgement and CAN angle stability do
not independently prove physical holding. The checkbox does not command a
large correction after a homing failure; existing HOME tolerance failures
still disable the motor. An independent power cutoff and a mechanically secured
load are required for any supervised first hardware test.

Install and launch:

```bash
cd /home/cuong/lm_calibr/src/motor
. .venv/bin/activate
sudo apt install -y libxcb-cursor0
python -m pip install -r requirements.txt
python motor_control_gui.py
```

`libxcb-cursor0` supplies `libxcb-cursor.so.0`, which Qt 6 needs for the X11
`xcb` display plugin on Ubuntu. It is an operating-system package and cannot
be installed by `pip`.

Run the complete GUI with deterministic software feedback and no serial/CAN
device access:

```bash
python motor_control_gui.py --simulate
```

Run all software tests in an offscreen Qt session:

```bash
QT_QPA_PLATFORM=offscreen python -m unittest discover -s tests -v
```

If the GUI does not own `/dev/ttyUSB0`, send the existing independent
STOP/DISABLE sequence from another terminal:

```bash
cd /home/cuong/lm_calibr/src/motor
. .venv/bin/activate
python waveshare_motor.py --mode stop --device /dev/ttyUSB0 --can-bitrate 500000 --motor-id 1
```

Do not run that second process while the GUI owns the serial port. Inspect a
recorded CSV with, for example:

```bash
ls -lt motor_logs/*.csv
head -n 20 motor_logs/motor_session_TIMESTAMP.csv
```

### Supervised hardware use

1. Mechanically secure the motor, clear the shaft area, and prepare an
   independent physical power cutoff. Connect motor CAN H, CAN L, common
   ground, motor power, and the Waveshare adapter before starting the GUI.
2. Launch the GUI, retain `/dev/ttyUSB0`, 500000 bit/s, and motor ID 1, then
   press **CONNECT**. Confirm the CAN ID is `0x141`. Connection alone does not
   enable the controller.
3. Enter a positive target no greater than 100 RPM and a duration no greater
   than 20 seconds. Press **START**. Wait for the read-only status and stationary
   preflight to complete.
4. Read the UNKNOWN-protection warning. Only when the physical setup is ready,
   type the exact requested confirmation. ENABLE and `0xA2` are sent only after
   this confirmation.
5. Watch actual speed, accumulated angle, raw fields, sample rates, plots, and
   the event log. These values use provisional V2.35 scaling and are not an
   independent measurement of shaft motion.
6. Press **STOP** for an immediate stop request. The worker cancels further
   nonessential polling and sends `0x81`; after confirmation the motor remains
   enabled. **DISABLE** sends STOP before DISABLE. Use the physical cutoff if
   acknowledgements or zero speed cannot be confirmed.
7. An overspeed, unexpected direction, missing telemetry, or timeout is shown
   in the event log with the stop reason. GUI feedback above 100 RPM causes an
   immediate STOP/DISABLE sequence.
8. Use **EXPORT CSV…** after a session, then DISCONNECT. Disconnecting or
   closing the application attempts STOP/DISABLE before releasing the serial
   port, including after a confirmed STOP-only session.

## Phase 7 output-shaft validation

See [PHASE7_ANALYSIS.md](docs/PHASE7_ANALYSIS.md) for the offline log findings,
V2.35 limit review, and a future video-marker procedure for independently
measuring output-shaft speed. `offline_speed_report.py` only reads files. Its
video input requires measured frame numbers and unwrapped shaft angles; if
video-to-CAN timing is unknown, it does not claim a synchronized comparison.

```bash
cd /home/cuong/lm_calibr/src/motor
. .venv/bin/activate
python offline_speed_report.py --log experiment_logs/experimental_20261030T072632630471Z.log
```

The existing 5.5 RPM hard stop and protection `UNKNOWN` remain unchanged.
No further motor motion is recommended on the current evidence.

## Phase 6 speed analysis

See [PHASE6_ANALYSIS.md](docs/PHASE6_ANALYSIS.md) for the cause of the 0.21-second
abort and the unchanged experimental hard stop. The new read-only
`speed-angle-diagnostic` mode compares a bracketed `0x9C` speed sample with
two timed `0x92` angle samples. It sends no motion or configuration command.

```bash
cd /home/cuong/lm_calibr/src/motor
. .venv/bin/activate
python waveshare_motor.py --mode speed-angle-diagnostic --device /dev/ttyUSB0 --can-bitrate 500000 --motor-id 1 --timeout 0.5
```

The comparison uses V2.35 scales provisionally. It is not independent proof
of shaft speed. Experimental motion now logs machine-readable speed errors,
sample timing, stop reasons, and optional post-stop angle/current reads. The
experimental duration default is 1 second; its 5.5 RPM hard stop is unchanged.

Standalone Python 3 motor diagnostics for the documented MF9025-family protocol
V2.35. `test_motor.py` uses `python-can` with a CANdo Pro (`gs_usb`) on SocketCAN;
the currently connected Waveshare adapter instead uses the CH340 serial bridge
and `waveshare_probe.py` / `waveshare_motor.py`. Exact MHF9025-V3 motion
compatibility remains provisional. The `dry-run` modes never open the bus.

## Waveshare USB-CAN-A adapter

The Waveshare USB-CAN-A appears as CH340 `/dev/ttyUSB0`, not SocketCAN `can0`.
Its host serial link defaults to 2 Mbps; the CAN bitrate is configured separately.
`waveshare_probe.py` uses Waveshare's documented serial bridge format and sends
only one read-only motor status request per invocation. `waveshare_motor.py`
prepares bounded speed tests through this adapter. Normal modes default to
**5 RPM for 2 seconds**, with hard limits of 10 RPM and 5 seconds; the separate
experimental mode defaults to **+5 RPM for 1 second**. It checks both status
replies and a three-state protection assessment before enabling. The only
status commands `0x9A` and `0x9C` do not provide a verified MHF9025-V3
fault-field definition. Consequently
the protection assessment is **UNKNOWN** and the test cannot enter `READY`.

```bash
cd /home/cuong/lm_calibr/src/motor
. .venv/bin/activate
python waveshare_probe.py --dry-run --can-bitrate 500000
python waveshare_probe.py --can-bitrate 500000 --status 1 --timeout 1
python waveshare_probe.py --can-bitrate 500000 --status 2 --timeout 1
python waveshare_motor.py --mode fault-diagnostic --timeout 0.5
```

For the five additional MF9025-family V2.35 read commands, use:

```bash
python waveshare_motor.py --mode extended-read-diagnostic --device /dev/ttyUSB0 --can-bitrate 500000 --motor-id 1 --timeout 0.5
```

This mode sends exactly one standard `0x141` CAN frame with DLC 8 for each
command, in order: `0x30` PID, `0x33` acceleration, `0x90` encoder, `0x92`
multi-turn angle, `0x9D` three-phase current. Each payload is the command byte
followed by seven zero bytes. It uses the existing Waveshare serial bridge at
2 Mbps host baud. TX/RX frames are printed raw, followed by a JSON result for
each command. Each read has a bounded timeout and no retry; a timed-out read is
reported before continuing to the next command. The process exits nonzero if
any read times out. It sends no motion, enable, stop, fault-clear, or parameter
write command.

Decoded PID gains are raw byte values without asserted physical units. The
[MF9025-family CAN V2.35 reference](https://www.scribd.com/document/826568712/%E7%93%B4%E6%8E%A7MF9025)
defines `0x33` acceleration at 1 degree/s² per signed 32-bit
LSB; `0x90` returns three unsigned 16-bit encoder fields; `0x92` carries the
low seven bytes of a signed multi-turn angle at 0.01 degree/LSB; and `0x9D`
returns signed 16-bit phase currents at 1/64 A per LSB. These field layouts,
scales, and any direction interpretation are **unverified for MHF9025-V3**.
The `0x90` values must not be treated as its full encoder resolution, and
none of these reads verifies the motor's protection state or physical shaft
motion. The mode does not infer a normal controller state from the responses.

### LED state test without motion commands

`led-state-test` compares the motor LED after STOP/DISABLE and after ENABLE.
It checks `0x9A` and `0x9C` first, requires reported speed exactly zero,
and blocks ENABLE for a verified active protection fault or legacy V2.35
fault-candidate bits in `0x9A` byte 7. The MHF9025-V3 protection state stays
**UNKNOWN**; zero candidate bits do not prove a fault-free controller.
The user must confirm that the motor is mechanically secured and an independent
power cutoff is ready before the serial adapter is opened. After recording the
disabled LED pattern, the user must explicitly confirm ENABLE. The program
then sends only `0x88` and allows up to 20 seconds to record the enabled LED
pattern. It automatically attempts STOP/DISABLE before exit, including after
a timeout, communication failure, or interrupt. Software cannot guarantee
physical stopping. All CAN TX/RX frames, confirmations, and observations are
saved in `led_logs/led_state_*.log`.

```bash
cd /home/cuong/lm_calibr/src/motor
. .venv/bin/activate
python waveshare_motor.py --mode led-state-test --device /dev/ttyUSB0 --can-bitrate 500000 --motor-id 1 --timeout 0.5
```

The LED response must be observed by the user; this test does not verify motor
motion or establish that the controller is safe to command motion.

With motor power on, both `0x9A` and `0x9C` replied at 500 kbps on CAN ID
`0x141`. A captured `0x9A` payload is `9A 21 32 0B 00 00 30 00`. The supply
was independently measured at **28.66 V**. The MHF9025-V3-specific decoder
uses bytes 2–3 little-endian at 0.01 V/LSB: `0x0B32 / 100 = 28.66 V`.
This is experimentally supported by one measured point, not manufacturer
verified. The older MF9025-family decoder uses bytes 3–4 at 0.1 V/LSB and
still exists for other models; it is not used for Waveshare MHF9025-V3 status.
The meaning of byte 6 and the protection-field mapping are unknown. Earlier
read-only captures showed `0x30`; later captures showed `0x20` with the same
28.66 V decode. This change does not identify whether byte 6 represents motor
state, protection bits, or another field.
Byte 7 being zero does not establish a fault-free state. The `0x9C` response
`9C 21 FD FF 00 00 34 32` has zero raw speed while the shaft is stationary;
later read-only samples included raw speed values 1 and 3. These small values
do not by themselves establish shaft rotation, and nonzero speed scaling remains
untested. Normal motion remains blocked until the protection state can be
verified independently. The separately authorized experimental mode below
retains `UNKNOWN` and does not establish normal operation.

`--mode fault-diagnostic` sends exactly one `0x9A` and one `0x9C` request and
prints a structured JSON report with all eight bytes of each response, the
8-bit byte-7 candidate (`0x00`), the 16-bit little-endian bytes-6–7 candidate
(`0x0030` in earlier captures, `0x0020` in later captures), and
`protection.state=UNKNOWN`. These are numerical interpretations,
not assigned fault meanings:

| Hypothesis | Support | Limitation | Assessment |
|---|---|---|---|
| Byte 7 is an 8-bit error field | Older MF9025 V2.35 places errors there | Its voltage layout conflicts with this motor; byte 6 remains unexplained | UNKNOWN |
| Bytes 6–7 are a 16-bit error field | A related RMD-X V3.x reference uses that layout | Compatibility and bit mapping for MHF9025-V3 are unverified | UNKNOWN |
| Byte 6 is operating-state data | Read-only captures contain both `0x30` and `0x20` at the same decoded voltage | Variation could also reflect faults or other feedback; no field definition | UNKNOWN |
| Full layout differs from both references | Voltage uses bytes 2–3 here | Other fields are not mapped | PLAUSIBLE |

No other read-only fault command with an independently verified MHF9025-V3
definition is present in this project. The smallest missing input is the
manufacturer-specific definition of the MHF9025-V3 protection field (or a
verified compatible read-only fault command). Do not infer a fault name from
the `0x0030` or `0x0020` bit patterns.

`waveshare_probe.py` prints timestamp, standard/extended type, CAN ID, DLC,
complete payload, individual bytes, and candidate 16-bit interpretations.
The CH340 bridge does not expose SocketCAN controller counters to `ip`, so
before/after CAN packet and error statistics are unavailable. The host
timestamp is the serial receipt time, not a controller timestamp.

For the first 5 RPM, 2-second trials, inspect the software-only frame plans:

```bash
python waveshare_motor.py --mode dry-run --dry-run-direction positive --speed 5 --duration 2
python waveshare_motor.py --mode dry-run --dry-run-direction negative --speed 5 --duration 2
```

Only after the fault mapping and protection state are independently verified,
the guarded physical test commands are:

```bash
python waveshare_motor.py --mode positive --speed 5 --duration 2 --allow-motion --ack-protocol-compatibility
# Run the negative test separately, only after the positive test succeeds.
python waveshare_motor.py --mode negative --speed 5 --duration 2 --allow-motion --ack-protocol-compatibility
```

With the observed `0x9A` reply, status mode reports 28.66 V and the evidence
for `protection=UNKNOWN`. A normal `positive` or `negative` motion attempt exits
**before enable or speed transmission**. The state machine logs each transition;
on future authorized motion it stops after the bounded interval, requires two
consecutive near-zero speed samples, and attempts stop and disable in cleanup.
Positive protocol speed is not confirmed as physical
CW from any particular viewing side. To use `positive` or `negative` as CW,
first establish the direction mapping from the intended viewing side and verify
the MHF9025-V3 status layout and fault flags. The emergency software stop command
for this adapter is `python waveshare_motor.py --mode stop`; motor power must
also be independently disconnectable.

### Separate experimental +5 RPM test

`experimental-motion` is a manually authorized trial. It accepts an unverified
protection field as `UNKNOWN` only in this mode, and blocks a verified fault,
missing replies, nonstationary feedback, and values outside the local
test limits of 24–32 V and 0–70 C. These limits are chosen around the measured
28.66 V supply; they are **not** a verified manufacturer rating. It accepts
only positive speed up to 5 RPM and duration up to 2 seconds. Its defaults are
+5 RPM and 1 second. The motor must be secured,
the shaft clear, and an independent power disconnect within reach. This mode
requires `--allow-experimental-motion` and a separate exact-phrase terminal
confirmation after the status reads, immediately before `0x88` ENABLE.
Because stationary captures have included isolated raw `0x9C` readings of
3 and -5 deg/s, this mode checks five read-only speed samples over roughly
0.4 seconds before ENABLE. The first reading may be an outlier; the next four
must each be within ±3 deg/s, and the raw encoder must stay within ±2 counts
of the first sample throughout. A persistent speed reading or larger encoder
change blocks motion. Normal `positive`/`negative` mode retains its existing
stationary check.

```bash
cd /home/cuong/lm_calibr/src/motor
. .venv/bin/activate
ls -l /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
python waveshare_motor.py --mode status --can-bitrate 500000
python waveshare_motor.py --mode dry-run --dry-run-direction positive --speed 5 --duration 1
# PHYSICAL MOTION: run only when the secured motor and independent power disconnect are ready.
python waveshare_motor.py --mode experimental-motion --can-bitrate 500000 --motor-id 1 --speed 5 --duration 1 --allow-experimental-motion
# STOP and DISABLE may be sent independently at any time.
python waveshare_motor.py --mode stop --can-bitrate 500000 --motor-id 1
ls -t experiment_logs/experimental_*.log | head -1
tail -n 100 "$(ls -t experiment_logs/experimental_*.log | head -1)"
```

The provisional frames on CAN ID `0x141` are ENABLE
`88 00 00 00 00 00 00 00`, +5 RPM
`A2 00 00 00 B8 0B 00 00`, STOP `81 00 00 00 00 00 00 00`, and DISABLE
`80 00 00 00 00 00 00 00`. Their exact movement behavior on MHF9025-V3
has not been verified. Positive RPM may not mean physical CW; commanded and
actual RPM may differ. The mode monitors `0x9C` feedback and attempts STOP and
DISABLE after a monotonic deadline or on error. To avoid counting observed
stationary noise as rotation, the experimental report requires at least
15 deg/s raw feedback at a 5 RPM request. **Motor-reported speed and encoder
movement do not verify physical output-shaft motion.** The motor's output was
visually stationary in the first hardware trials despite changing telemetry.
After experimental STOP, three near-zero speed readings with a stable raw
encoder confirm only that feedback settled; command acknowledgment and
telemetry do not independently prove the shaft stopped. The project log contains timestamped raw TX/RX frames, status,
feedback, command acknowledgments, and cleanup results. After an experimental
abort, the program attempts STOP and DISABLE first, then makes bounded
read-only speed and `0x9A` status checks; a failed check is reported as
unconfirmed. If USB or CAN fails,
software cannot guarantee a physical stop: disconnect motor power independently.
If a run aborts on abnormal speed or the motor/controller LED pattern changes,
inspect the hardware and obtain a controller-specific LED/fault definition
before another motion attempt. Do not increase the speed limit to hide the abort.

Adapter protocol references: [Waveshare USB-CAN-A](https://www.waveshare.com/wiki/USB-CAN-A)
and [serial-to-CAN frame definition](https://files.waveshare.com/wiki/USB-CAN-A/Demo/USB%20(Serial%20port)%20to%20CAN%20protocol%20defines.pdf).

## Hardware and preparation

Connect CAN H, CAN L, common reference ground, measured 28.66 V motor power,
and correct termination at the two ends of the bus. Secure the motor and keep
the shaft clear. Provide an independent motor-power disconnect; use a separately
functioning emergency stop and a current-limited power supply if available.
Software stop cannot guarantee a stop after adapter, laptop, or CAN failure.

Run from a terminal:

```bash
cd /home/cuong/lm_calibr/src/motor
python3 -m venv .venv
. .venv/bin/activate
python -m pip install -r requirements.txt
```

### Historical CANdo Pro SocketCAN instructions

The following `can0` commands apply only when the older CANdo Pro is connected.
They do not configure the current Waveshare USB-serial adapter.

The adapter has been identified as USB `1d50:606f`, driver `gs_usb`, interface
`can0`. Bring it up at the initial 1 Mbps rate (these commands change the live
interface only):

```bash
ip -details link show dev can0
sudo ip link set dev can0 down
sudo ip link set dev can0 type can bitrate 1000000
sudo ip link set dev can0 up
ip -details -statistics link show dev can0
```

Do not add `restart-ms`; this adapter does not support automatic bus-off restart.
Check for `state UP`, `bitrate 1000000`, and no `BUS-OFF` or growing error counters.
The program requires this link check before status or motion. It accepts
`--channel`, `--motor-id` (1–32), and `--bitrate` for the expected live
bitrate; node 1 uses standard ID `0x141`. `--bitrate` does not reconfigure
SocketCAN. The program refuses to transmit while `can0` is in listen-only mode.

## CAN communication diagnosis

`can_diagnostic.py` inspects Linux CAN state, listens without transmitting,
performs one explicitly selected read-only `0x9A` probe, compares counters, and
prints a manual listen-only bitrate plan. It never sends enable, speed, torque,
position, stop, or configuration commands. The probe refuses to transmit unless
the controller is `ERROR-ACTIVE`.

Use this sequence when communication is not established:

```bash
cd /home/cuong/lm_calibr/src/motor
. .venv/bin/activate

# Interface, driver, bitrate, packet totals, and CAN error counters.
python can_diagnostic.py --mode inspect
python can_diagnostic.py --mode analyze

# Passive receive only. Output is capped while all frames are counted.
python can_diagnostic.py --mode monitor --timeout 10 --max-display 100

# Exactly one read-only status request, only while ERROR-ACTIVE.
# This prints complete before/after counters.
python can_diagnostic.py --mode probe --motor-id 1 --timeout 1
```

A successful SocketCAN send means that Linux queued the frame. It does not mean
another CAN node acknowledged or received it. Repeated kernel CAN error
notifications, a transition to `ERROR-PASSIVE`, or increasing warning/passive
counters indicate a CAN-layer failure. Application-level motor ID or command
selection cannot cause these CAN-layer errors. The utility decodes Linux error
classes, including `bit stuffing error` (`CAN_ERR_PROT_STUFF`) and an explicit
`no ACK` indication (`CAN_ERR_ACK`) when the driver provides them. A protocol
error is not automatically evidence of missing ACK.

After repairing power or bus wiring, clear a stuck retransmission or recover
from `ERROR-PASSIVE`/`BUS-OFF` by manually cycling the interface:

```bash
sudo ip link set dev can0 down
sudo ip link set dev can0 type can bitrate 1000000
sudo ip link set dev can0 up
python can_diagnostic.py --mode inspect
```

Do not probe again unless it reports `ERROR-ACTIVE`. For manual bitrate
investigation, print commands that configure this adapter in listen-only mode:

```bash
python can_diagnostic.py --mode bitrates
```

Run only one printed bitrate block at a time. Passive silence cannot determine
the bitrate of a motor that does not transmit autonomously. Do not automate
probing across bitrates or motor IDs.

If the motor's configured bitrate is independently confirmed as 500 kbps,
disable listen-only mode and make a single read-only status check before any
motion test:

```bash
sudo ip link set dev can0 down
sudo ip link set dev can0 type can bitrate 500000 listen-only off
sudo ip link set dev can0 up
python test_motor.py --mode status --bitrate 500000 --verbose
```

Only a coherent `0x9A` and `0x9C` reply with `error=0x00` establishes the
software prerequisite for a motion test. If the status check instead causes
CAN errors or no response, bring `can0` down and repair the bus before retrying.

## Run order

The first three program commands below are read-only or software-only. A status
reply with plausible temperature and voltage, the right CAN ID and command
byte, and `error=0x00` is needed before attempting motion. A valid read-only
reply does not prove that motion commands have identical behavior on this motor.
Verify an independent power disconnect and a clear mechanical area first.

```bash
# 1. Inspect the CAN interface (software-only).
ip -details -statistics link show dev can0

# 2. Check motor communication (read-only CAN traffic).
python test_motor.py --mode status --verbose

# 3. Read motor status again (read-only CAN traffic).
python test_motor.py --mode status

# 4. Print the complete intended frame sequence (no CAN bus opened).
python test_motor.py --mode dry-run
```

After those checks pass and the physical safeguards are ready, the following
commands **physically command rotation**. The second flag records that you
understand protocol compatibility is provisional. Positive and negative are
protocol signs; their CW/CCW mapping depends on viewing orientation.

```bash
# 5. Positive 5 RPM for 2 seconds.
python test_motor.py --mode positive --speed 5 --duration 2 --allow-motion --ack-protocol-compatibility

# 6. Negative 5 RPM for 2 seconds.
python test_motor.py --mode negative --speed 5 --duration 2 --allow-motion --ack-protocol-compatibility

# 7. Both directions, with a stop check and 2-second pause between them.
python test_motor.py --mode both --speed 5 --duration 2 --allow-motion --ack-protocol-compatibility

# 8. Send stop and disable without first requiring status (actuating safety commands).
python test_motor.py --mode stop
```

`--dry-run` can be added to any mode to print frames without opening CAN.
Speed magnitude is limited to 10 RPM and duration to 5 seconds. Ctrl+C and
SIGTERM trigger stop and disable cleanup during motion. A missing stop response
or missing zero-speed feedback is **not** proof the shaft has stopped. Disconnect
motor power independently if behavior is uncertain.

## Protocol and expected output

Frames are eight-byte classic CAN 2.0A, ID `0x140 + motor_id`, little-endian.
`0x9A` returns temperature, voltage, and an error byte. Bit 0 means
undervoltage; bit 3 means overtemperature. Any other set bit is displayed as
unknown. Any nonzero error byte blocks motion. `0x9C` and the `0xA2` response
return temperature, raw torque-current feedback, speed in whole degrees per
second, and raw uint16 encoder feedback. The encoder field is **not assumed**
to contain the complete 19-bit encoder value. Command speed uses signed int32
at 0.01 degree/second per LSB, so 5 RPM is raw 3000. Feedback uses signed int16
at 1 degree/second per LSB, so divide by 6 for RPM. Feedback is quantized and
may transiently differ from the commanded speed.

Representative output (numbers depend on the actual motor):

```text
SocketCAN can0: UP, 1000000 bit/s
Status1: temperature=25 C voltage=24.0 V error=0x00 flags=none
Status2: temperature=25 C torque_current_raw=0 speed=0 deg/s (0.00 RPM) encoder_raw=4660
Communication: OK (read-only 0x9A and 0x9C replies)
enable: acknowledged
Speed command +5 RPM response: temperature=25 C torque_current_raw=0 speed=0 deg/s (0.00 RPM) encoder_raw=4660
Motion feedback: ... speed=30 deg/s (5.00 RPM) ...
Motion feedback: direction observed
stop: acknowledged
Stopped: confirmed by speed feedback
disable: acknowledged
```

The speed command response alone does not establish rotation; the program
requires later speed feedback with the requested sign. `--verbose` prints
timestamped TX/RX IDs and bytes. Enable, stop, and disable acknowledge with
their same eight-byte command frames. Disabling may reset accumulated rotation
information.

## Troubleshooting

- No response: confirm motor 24 V, shared ground, CAN H/L wiring, termination,
  `can0` UP, and the adapter's error counters. Do not send arbitrary commands.
- Wrong motor ID: use the known configured ID with `--motor-id N`; the initial
  assumption is 1. This program does not sweep IDs or bitrates.
- Wrong bitrate: verify `bitrate 1000000` on the interface and the controller's
  configured bitrate; change the live interface only after establishing the
  correct value by a trusted configuration source.
- Bus-off: inspect `ip -details -statistics link show dev can0`, repair wiring
  or termination, then manually cycle the interface. No automatic restart is used.
- Missing feedback: check RX wiring and ID, inspect verbose frames, and use
  independent power disconnect if motion may be continuing.
- Wrong direction: positive and negative are protocol signs, not guaranteed
  CW/CCW. Verify from a defined viewing side at low speed before labeling them.
- Enabled but not rotating: check protection bits, power-supply current limit,
  mechanical load, actual speed feedback, and protocol compatibility.

## Software-only validation

```bash
python -m compileall -q motor_can.py test_motor.py tests
python -m unittest discover -s tests -v
python test_motor.py --mode dry-run
```
