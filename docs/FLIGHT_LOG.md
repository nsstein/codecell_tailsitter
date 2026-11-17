# Flight log

The flight controller records a 50 Hz onboard log to the CodeCell's flash while armed. No SD card, no telemetry and no USB cable are needed in flight. After landing, connect USB and dump the log as CSV.

## Requirements

- Partition scheme: **Default 4MB with spiffs**. Without it, the log is disabled and the serial monitor prints `Logging unavailable (no partition)`.
- `LOG_ENABLE 1` (the default).
- Serial Monitor or terminal at **115200**.
- Commands work **only while disarmed**.

## What is recorded

- One **session** per arm. Disarm, then arm again, and a new session number starts.
- 64-byte records at **50 Hz**.
- Event records for arm, disarm reason, failsafe and link restore.
- Roughly **10 minutes** of data on the default partition. With `DEBUG 1`, the exact free time is printed at boot.
- Data survives power-off.
- When the partition is full, recording stops. The LED blinks **yellow** while disarmed at 95 % or more.

Each record carries a magic number (`0xA55B`) and an XOR checksum. Records that fail the check are skipped on dump, so a partial or interrupted write does not corrupt the rest of the log.

## Commands

| Command | Effect |
|---|---|
| `list` | One line per session: start time, duration, samples, last event. |
| `dump` | All sessions as CSV. |
| `dump <n>` | Only session `n`. |
| `erase` | Erase the whole log. Takes a few seconds; LED fast yellow. |
| `help` | List the commands. |
| `cal` / `cal status` / `cal force` / `cal erase` | Calibration commands, see [CALIBRATION.md](CALIBRATION.md). |

Example `list`:

```text
LOG: 412 KB used of 1536 KB (27%), 3 session(s), ~9.4 min free at 50 Hz
session  start_ms    duration_s  samples  last_event
      1      18234          42.6     2130  DISARM_SWITCH
      2      68110          18.2      910  DISARM_CRASH
      3     132900          61.4     3070  DISARM_SWITCH
```

Dumping prints:

```text
### LOG BEGIN
session,t_ms,type,event,state,...
...
### LOG END (6110 rows)
```

The `### LOG BEGIN` / `### LOG END` markers make it easy to cut the CSV out of a terminal capture.

## CSV columns

### Identity and timing

| Column | Meaning |
|---|---|
| `session` | Arm session number. |
| `t_ms` | Milliseconds since boot. |
| `type` | `DATA` or `EVENT`. |
| `event` | `ARM`, `DISARM_SWITCH`, `DISARM_FAILSAFE_TIMEOUT`, `DISARM_CRASH`, `DISARM_IMU_LOST`, `FAILSAFE`, `LINK_RESTORED`. |
| `state` | `DISARMED`, `HOVER`, `TO_FWD`, `FORWARD`, `TO_HOVER`, `FAILSAFE`. |
| `armed` | 0 or 1. |
| `link` | 1 if the RC link was valid at that sample. |

### Flight state

| Column | Meaning |
|---|---|
| `tau` | Transition variable, 0 = hover, 1 = forward. |
| `lq` | Uplink link quality, percent. `-1` if unavailable. |
| `loop_hz` | Control loop rate, rounded to 2 Hz. |
| `nose_deg` | Nose above horizon, degrees. |
| `bank_deg` | Bank angle, degrees. |
| `hdg_deg` | Heading, degrees. Relative, no magnetometer. |

### Pilot inputs

| Column | Meaning |
|---|---|
| `stk_roll` | Roll stick, −1..+1 after expo. |
| `stk_pitch` | Pitch stick, −1..+1 after expo. |
| `stk_yaw` | Yaw stick, −1..+1 after expo. |
| `thr` | Throttle **after** transition floors and failsafe. |
| `thr_stick` | Raw throttle stick, 0..1. |
| `ch5_us` | Raw CH5 pulse, µs. |
| `ch6_us` | Raw CH6 pulse, µs. |
| `arm_sw` | CH5 above the arm threshold. |
| `fwd_sw` | CH6 above the mode threshold. |
| `landed` | Throttle below `LANDED_THR` in hover (integrators frozen). |
| `rx_fs` | Receiver-reported failsafe. |

### Control

| Column | Meaning |
|---|---|
| `err_x`, `err_y`, `err_z` | Attitude error, degrees, body X/Y/Z. |
| `out_x`, `out_y`, `out_z` | PID output, −1..+1. |
| `gyro_x`, `gyro_y`, `gyro_z` | Body rate, °/s. |
| `rate_sp_x`, `rate_sp_y`, `rate_sp_z` | Rate setpoint, °/s. |
| `i_x`, `i_y`, `i_z` | PID integrator, −0.40..+0.40. |

Axis mapping:

| Axis | Hover | Forward |
|---|---|---|
| X | Yaw | Roll |
| Y | Pitch | Pitch |
| Z | Side lean | Yaw |

### Outputs and diagnostics

| Column | Meaning |
|---|---|
| `servo_l`, `servo_r` | Elevon pulse, µs. |
| `motor_l_cmd`, `motor_r_cmd` | DShot 0–2047, or PWM/OneShot/Multishot 1000–2000 µs. |
| `imu_glitches` | Low byte of the IMU glitch counter. |
| `ahrs_trust` | Accelerometer trust, 0..1. |
| `acc_g` | Accelerometer magnitude, g. |

## Reading a log

### Health checks

- `loop_hz` should sit near **200**. A sustained drop points to an I2C stall, a flash write stall or a busy serial port.
- `ahrs_trust` should be near **1** in calm flight. It dips during acceleration, which is expected; a long low-trust period means the attitude estimate was poor.
- `acc_g` should be near **1** in steady flight. Large excursions are normal in manoeuvres; a stuck value means the accelerometer is not updating.
- `imu_glitches` should stay small. A rising counter points to I2C noise or a loose connection.
- `link` and `lq` show the radio link. A dropout should produce a `FAILSAFE` event and a `LINK_RESTORED` event.

### Hover

- `tau` should be **0**.
- `nose_deg` should be near **+90°** and `bank_deg` near **0°**.
- `thr` is your hover throttle. Compare it with `FS_THR` 0.48: the failsafe descent should be just below hover.
- With `thr ≥ 0.15`, `motor_l_cmd` and `motor_r_cmd` should both stay above the mixer floor. If one is at the floor while the other is high, the mixer is clipping differential as designed.
- `rate_sp_x/y/z` and `gyro_x/y/z` should track. A large, persistent gap means the axis is saturated or the gain is wrong.
- `i_x/y/z` should stay small. A large, slowly growing integrator suggests a mechanical trim problem.

### Transition

- `tau` should ramp **0 → 1 over about 2.0 s**, and **1 → 0 over about 1.5 s**.
- `nose_deg` should move from about **+90°** to about **+3°**.
- `thr` should sit at or above **0.65** during the transition, even if `thr_stick` is lower.
- `state` should be `TO_FWD`, then `FORWARD`, and `TO_HOVER`, then `HOVER`.

### Forward flight

- `tau` should be **1**.
- `nose_deg` should be near `FWD_PITCH_DEG` (3°) with the pitch stick centred.
- `bank_deg` should follow the roll stick up to 45°.
- Yaw is differential thrust: `out_z` and the motor commands should move in opposite directions.
- There is no heading hold, so `hdg_deg` may drift with wind.

### Events

| Event | Meaning |
|---|---|
| `ARM` | Armed. |
| `DISARM_SWITCH` | Pilot moved CH5 low. |
| `DISARM_FAILSAFE_TIMEOUT` | 15 s in failsafe. |
| `DISARM_CRASH` | Crash detector fired. |
| `DISARM_IMU_LOST` | No fresh IMU sample for 200 ms. |
| `FAILSAFE` | Link lost while armed. |
| `LINK_RESTORED` | Link returned before the timeout. |

## Analysing the CSV

The dump is plain CSV. A quick plot with Python:

```python
import pandas as pd
import matplotlib.pyplot as plt

rows = []
capture = False
for line in open("flight.csv"):
    if line.startswith("### LOG BEGIN"):
        capture = True
        continue
    if line.startswith("### LOG END"):
        break
    if capture:
        rows.append(line)

from io import StringIO
df = pd.read_csv(StringIO("".join(rows)))
df = df[df["type"] == "DATA"]

fig, ax = plt.subplots(3, 1, sharex=True)
ax[0].plot(df["t_ms"], df["nose_deg"], label="nose")
ax[0].plot(df["t_ms"], df["tau"] * 90, label="tau x 90")
ax[1].plot(df["t_ms"], df["rate_sp_y"], label="rate sp Y")
ax[1].plot(df["t_ms"], df["gyro_y"], label="gyro Y")
ax[2].plot(df["t_ms"], df["motor_l_cmd"], label="motor L")
ax[2].plot(df["t_ms"], df["motor_r_cmd"], label="motor R")
for a in ax:
    a.legend()
    a.grid(True)
plt.show()
```

Or use the CSV directly in a spreadsheet. The `session` column separates flights, and `type` separates data from events.

## Log lifecycle

- At boot, `logBegin()` scans for the first empty record and the highest session number.
- If the tail of the partition contains non-erased data, it repairs the log by erasing from the next sector.
- While disarmed, the log is pre-erased so in-flight writes do not need an erase.
- Each arm increments the session.
- `erase` clears the whole partition and resets the session counter.
- Upgrading from v19 or earlier erases the old log on first boot because the record format changed. Dump it before upgrading.

## Known limitation

The log is written from the same loop that flies the aircraft. A flash write can occasionally take long enough to create a control-loop gap. The firmware detects a gap longer than two control periods, resets the PIDs and clamps `dt`, but this is a mitigation, not a real-time guarantee. If you see repeated `loop_hz` dips, reduce logging or investigate the flash. See [SAFETY.md](SAFETY.md).

---

Next: **[SAFETY.md](SAFETY.md)** · **[TUNING.md](TUNING.md)** · **[ARCHITECTURE.md](ARCHITECTURE.md)**
