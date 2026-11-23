# Phase 1 — MHF9025-V3 read-only absolute encoder diagnostic

## Scope and software status

`absolute_encoder_diagnostic.py` sends exactly one read request for each of
`0x90`, `0x94`, `0x92`, and `0x9C` on standard CAN ID `0x141`. Every request is
the command byte followed by seven zero bytes. Each exchange has one bounded
timeout and no retry. The tool never sends `0x19`, `0x91`, encoder writes,
ENABLE, motion, STOP/DISABLE, PID writes, or other configuration commands.

The decoders implement these provisional LK-TECH CAN V2.35 family layouts:

| Command | V2.35 decode | MHF9025-V3 status |
|---|---|---|
| `0x90` | Bytes 2–3 position, 4–5 original position, 6–7 offset; unsigned 16-bit LE | Prior motor response observed; field meaning and resolution remain unverified |
| `0x94` | Older V2.35: bytes 6–7 uint16; observed V3: bytes 4–7 uint32; 0.01°/LSB; 0–359.99° | Supported; V3 byte layout confirmed internally against `0x92` and encoder changes |
| `0x92` | Bytes 1–7 accumulated angle; signed 56-bit LE; 0.01°/LSB | Prior motor response observed; V2.35 calls the value signed but the CAN frame carries only seven angle bytes |
| `0x9C` | Temperature s8, current s16, speed s16 at 1°/s, encoder u16 | Prior motor response observed; scaling and state meaning remain provisional |

Successful replies establish communication and reproducible controller values.
They do not establish encoder accuracy, output-shaft coupling, direction, or a
persistent mechanical HOME.

### Historical captures already present in the project

These were collected before this Phase 1 tool and are context only. They are
not results from Tests A–C:

| Command | Raw response | Provisional decode |
|---|---|---|
| `0x90` | `90 00 00 00 9F 50 50 50` | position=0, original=20639, offset=20560 |
| `0x92` | `92 49 2C 00 00 00 00 00` | signed raw=11337, accumulated angle=113.37° |
| `0x9C` | `9C 1F FC FF 00 00 9E 50` | 31°C, current raw=-4, speed=0°/s, encoder=20638 |
| `0x94` | earlier: no capture | Phase 1 captures below establish a V3-specific bytes 4–7 layout |

The matching `0x90` original value and `0x9C` encoder value are suggestive, but
one pair differing by one count does not establish field identity or accuracy.

## Hardware measurements — 2026-11-01

The user manually performed Tests A–C. The following values were recalculated
from the preserved raw frames. The original Test CSV decoded `0x94` as the older
bytes 6–7 layout and displayed 0.00°; that stored interpretation is superseded
by the raw-frame analysis here.

### Test A — stationary

Both `0x92` and corrected `0x94` reported 6.54° in both recordings. The `0x90`
original field changed from 1191 to 1192 counts and `0x9C` encoder was 1192 in
both samples. The second `0x9C` reported an isolated 4°/s although the encoder
was unchanged, consistent with previously observed low-speed estimator noise.

### Test B — manual approximately 90° rotation

| Source | Before | After | Signed shortest change |
|---|---:|---:|---:|
| Corrected `0x94` | 6.54° | 273.94° | -92.60° |
| `0x92` | 6.54° | -86.05° | -92.59° |
| `0x9C`/`0x90` raw encoder | 1192 | 49871 | -16857 counts = -92.6038° if 65536 counts/turn |

The three controller fields agree to about 0.014°. They originate from the same
controller, so this is strong internal consistency rather than independent
metrology. The physical direction sign remains tied to the user's viewing side.

### Test C — power cycle without intentional movement

| Source | Before power-off | After power-on | Change |
|---|---:|---:|---:|
| Corrected `0x94` | 11.66° | 11.62° | -0.04° |
| `0x92` | 11.66° | 11.62° | -0.04° |
| `0x9C`/`0x90` raw encoder | 2123 | 2116 | -7 counts = -0.0385° if 65536 counts/turn |
| `0x90` third field | 2056 | 2056 | 0 counts |

This one power cycle preserved the marked-position reading to approximately
0.04°. The result supports proceeding to design a persistent software HOME
reference based on the read-only absolute angle. Before relying on it for
calibration accuracy, repeat the power-cycle test several times and at positions
on both sides of the 0°/360° wrap. Phase 1 does not write or define HOME.

## Manual test record

| Test | CSV before | CSV after | Physical observation | Result |
|---|---|---|---|---|
| A: stationary repeatability | `test_a_1` | `test_a_2` | shaft stationary | pass for angle/encoder; one noisy speed sample |
| B: manual approximately 90° rotation | `test_b_before` | `test_b_after` | approximately 90° | pass, measured -92.6° protocol sign |
| C: power cycle without movement | `test_c_before` | `test_c_after` | mark unchanged | initial pass, -0.04° change |
