# Tuning

Tune the airframe **before** the gains. Most “bad PID” behaviour in a tail-sitter is really a mechanical problem: a soft mount, a mis-set servo, a thrust-line asymmetry or a battery that cannot hold voltage.

> [!WARNING]
> Props off for every direction, mixer and gain check on the bench. Fit props only for a flight test.

## Tune in this order

1. **Mechanical:** CG, thrust line, servo centring, hinge slop, prop direction, BEC capacity.
2. **Directions:** elevons and motors move the right way.
3. **Mixer authority:** roll, pitch and differential thrust.
4. **Hover gains.**
5. **Transition throttle floors.**
6. **Forward gains.**

Change **one parameter at a time**, and keep a note of the old value. Re-run the bench checks after any change that affects directions or the mixer.

## Servo directions and limits

| Constant | Default | Meaning |
|---|---|---|
| `SERVO_REV_L` | `false` | Invert the left elevon |
| `SERVO_REV_R` | `true` | Invert the right elevon |
| `SERVO_CENTER_L` | 1500 | Left elevon centre, µs |
| `SERVO_CENTER_R` | 1500 | Right elevon centre, µs |
| `SERVO_THROW_US` | 400 | Full-scale throw from centre |
| `SERVO_MIN_US` | 1000 | Lower clamp |
| `SERVO_MAX_US` | 2000 | Upper clamp |

### Direction check

With `HW_TEST 1`, props off:

1. Move the roll stick right. The elevons move in **opposite** directions — one up, one down — so the aircraft rolls right.
2. Move the pitch stick back (nose up). Both elevons move **up**.
3. If a surface is reversed, flip its `SERVO_REV_*` constant.

Do not use radio reversing to fix an elevon. Fix it in the sketch so the flight controller and the disarmed stick-follow mode agree.

### Centring

- Centre the servo arms mechanically first.
- Use `SERVO_CENTER_L/R` only for fine trim.
- Keep the linkage symmetric. A large `SERVO_CENTER` offset reduces throw on one side.
- If the surfaces buzz at the endpoints, reduce `SERVO_THROW_US` or increase `SERVO_MIN_US` / decrease `SERVO_MAX_US`.

## Elevon mix

```text
outL = PITCH_MIX * pitch - ROLL_MIX * roll
outR = PITCH_MIX * pitch + ROLL_MIX * roll
```

| Constant | Default | Effect |
|---|---|---|
| `ROLL_MIX` | 0.6 | Elevon differential authority (forward roll / hover yaw) |
| `PITCH_MIX` | 0.8 | Elevon collective authority (forward pitch / hover pitch) |

- If roll is sluggish but pitch is fine, raise `ROLL_MIX` in steps of 0.05.
- If pitch is too strong or the servo stalls, lower `PITCH_MIX`.
- The sum can exceed 1; `elevonUs()` clamps the final command to ±1, so a large combined command saturates rather than over-driving the servo.
- A tail-sitter needs more pitch authority in hover than a conventional wing because the elevons are also the pitch control while the wing is vertical.

## Differential thrust

| Constant | Default | Effect |
|---|---|---|
| `DIFF_THRUST_MAX` | 0.30 | Maximum ± differential as a fraction of throttle |
| `AIRMODE_MIN_THR` | 0.15 | Low-motor floor for the mixer |

The mixer clips the differential to the room between the collective and both rails. At or above the floor, both motors stay in `[AIRMODE_MIN_THR, 1]`. Below the floor, the differential is removed and the motors stay equal.

- More yaw authority in hover or forward: raise `DIFF_THRUST_MAX` toward 0.40, in steps of 0.05.
- Do **not** exceed 0.5; the sketch has a `static_assert` because a larger value no longer leaves meaningful room.
- Raising `AIRMODE_MIN_THR` keeps the low motor spinning faster but reduces differential room near low throttle.
- If one motor stutters at low throttle, raise `AIRMODE_MIN_THR` slightly or fix the ESC idle / motor timing.

## Attitude limits

| Constant | Default | Meaning |
|---|---|---|
| `HOVER_MAX_TILT_DEG` | 15 | Maximum hover lean from the stick |
| `HOVER_YAW_RATE_DPS` | 90 | Hover yaw rate at full stick |
| `HOVER_HDG_SLACK_DEG` | 25 | How far the heading target may lead the actual heading |
| `FWD_PITCH_DEG` | 3 | Forward-flight nose attitude with the pitch stick centred |
| `FWD_MAX_BANK_DEG` | 45 | Maximum forward bank |
| `FWD_MAX_PITCH_DEG` | 25 | Forward pitch stick authority around the trim |
| `FWD_YAW_RATE_DPS` | 60 | Forward yaw rate at full stick |

- A small, light wing may feel twitchy at 15° hover lean. Lower it to 10–12° for the first flights.
- `HOVER_HDG_SLACK_DEG` prevents the heading target from winding up. Do not remove it; a large slack lets a sustained yaw command build a target the aircraft cannot reach, and the controller fights it when the stick returns to centre.
- `FWD_PITCH_DEG` is the trim attitude for forward flight. If the aircraft climbs or sinks with the pitch stick centred, adjust this in 0.5° steps rather than adding radio trim.
- `FWD_MAX_BANK_DEG` above 45° is aerobatic territory; the angle loop and the fixed transition throttle are not designed for it.

## Gains

Axis order is always `[0] = X`, `[1] = Y`, `[2] = Z`.

| Axis | Hover role | Forward role |
|---|---|---|
| **X** | Yaw (elevon differential) | Roll (elevon differential) |
| **Y** | Pitch (elevon collective) | Pitch (elevon collective) |
| **Z** | Side lean (differential thrust) | Yaw (differential thrust) |

### Hover gains

```c
Gains G_HOVER = {
  {2.0f, 4.5f, 3.0f},       // angP
  {0.20f, 0.25f, 0.20f},    // rP
  {0.10f, 0.20f, 0.15f},    // rI
  {0.003f, 0.006f, 0.004f}, // rD
  {1.0f, 3.0f, 3.0f}        // maxRate
};
```

### Forward gains

```c
Gains G_FWD = {
  {3.5f, 3.5f, 0.0f},       // angP
  {0.08f, 0.10f, 0.06f},    // rP
  {0.08f, 0.10f, 0.02f},    // rI
  {0.002f, 0.003f, 0.0f},   // rD
  {3.5f, 2.5f, 1.5f}        // maxRate
};
```

| Term | What it does | If too high | If too low |
|---|---|---|---|
| `angP` | Angle error → rate setpoint | Fast oscillation, overshoot | Sluggish, cannot hold attitude |
| `rP` | Rate error → output | High-frequency shake | Soft, slow rate response |
| `rI` | Removes steady rate error | Slow oscillation, wind-up | Persistent drift in a steady manoeuvre |
| `rD` | Damps rate change | Noise amplification, hot motors | Overshoot, bounce |
| `maxRate` | Caps the rate setpoint | Limits authority | Allows excessive rate demand |

Global limits:

- `I_LIMIT` 0.40 clamps each integrator.
- `D_LPF` 0.25 low-pass filters the derivative. Lower it (e.g. 0.15) if the D term is noisy; raise it (e.g. 0.35) if the response feels soft.

### Hover tuning workflow

Props on, low wind, safe altitude:

1. Hover and note the behaviour: slow drift, fast shake, or bounce.
2. **Fast oscillation** (a buzz or visible shake): reduce `rP` by 20 %, then `rD` if it persists.
3. **Slow wobble**: reduce `angP` by 20 %.
4. **Bounce on a stop**: increase `rD` slightly, or reduce `rP`.
5. **Persistent lean in one direction**: fix the mechanical trim or the thrust line first. Do not use the integrator to hide a bent airframe.
6. **Yaw hunts**: reduce hover X `angP` (currently 2.0) or increase X `rP`.
7. **Side-lean drift**: check the differential-thrust mixer floor and the motor symmetry before changing Z gains.

Make one change, fly, land, and compare the log. A 10–20 % change is enough; large jumps make the aircraft unflyable and the log hard to interpret.

### Forward tuning workflow

1. Transition at a safe altitude with clear space.
2. Check pitch trim first: with the stick centred, the aircraft should neither climb hard nor dive. Adjust `FWD_PITCH_DEG` in 0.5° steps.
3. Check roll: if it rocks in a bank, reduce forward X `rP` (0.08) or `rD` (0.002).
4. Check yaw: differential thrust is soft. If yaw is weak, raise `FWD_YAW_RATE_DPS` or `DIFF_THRUST_MAX` before raising the Z gains.
5. Do not tune forward gains in turbulence. You will chase noise.

## Transition constants

| Constant | Default | Meaning |
|---|---|---|
| `TRANS_FWD_SEC` | 2.0 | Hover → forward pitch-down time |
| `TRANS_BACK_SEC` | 1.5 | Forward → hover pitch-up time |
| `TRANS_START_MIN_THR` | 0.30 | Minimum throttle to start a forward transition |
| `TRANS_START_HOLD_MS` | 500 | Throttle must stay above the floor this long |
| `TRANS_ARM_DELAY_MS` | 1000 | No forward transition this soon after arming |
| `TRANS_FWD_THR_MIN` | 0.65 | Throttle floor during hover → forward |
| `TRANS_BACK_THR_MIN` | 0.65 | Throttle floor during forward → hover |

- The **timing** constants define the flight model. Change them only if you understand the effect on the pitch-down trajectory. A faster forward transition needs more altitude and more thrust.
- The **throttle floors** should be tuned to your aircraft. `0.65` is a safe starting point for a small wing; a heavier wing may need more, a very light one may need less.
- Do not lower the floors until you have flown the transition and watched the altitude loss in the log. A transition that sags is safer with more throttle, not less.
- `TRANS_START_MIN_THR` and `TRANS_START_HOLD_MS` are safety interlocks. Do not lower them to make the switch feel more responsive.

## Failsafe constants

| Constant | Default | Meaning |
|---|---|---|
| `FS_THR` | 0.48 | Descent throttle in hover after a link loss |
| `FS_DISARM_MS` | 15000 | Disarm delay in failsafe |

`FS_THR` should sit **just below** your actual hover throttle. If hover is around 0.55, 0.48 gives a gentle descent. Too high and the aircraft climbs away; too low and it descends fast or loses attitude authority.

Test the failsafe with props off first, then at altitude with a spotter. Confirm the receiver failsafe is **no pulses**.

## Arming constants

| Constant | Default | Do not weaken |
|---|---|---|
| `ARM_MAX_THR` | 0.05 | Throttle must be low |
| `ARM_MAX_TILT` | 30° | Must be near upright |
| `ARM_MAX_RATE_DPS` | 20 °/s | Must be still |
| `ARM_STILL_MS` | 100 ms | Stillness time |
| `ARM_MIN_TRUST` | 0.5 | Attitude must be trusted |

These are safety interlocks, not tuning knobs. If arming is inconvenient, fix the cause (a vibrating mount, a moving aircraft, a bad calibration) rather than relaxing the check.

## Calibration constants

| Constant | Default | Meaning |
|---|---|---|
| `CAL_STILL_MS` | 1500 | Still time per pose |
| `CAL_STILL_DPS` | 8 °/s | Gyro limit for “still” |
| `CAL_STILL_DEG` | 3° | Max change of “up” while holding |
| `CAL_MIN_ANGLE_DEG` | 80° | Minimum angle between the two poses |
| `CAL_WARN_ANGLE_DEG` | 85° | Warn below this angle |

`CAL_MIN_ANGLE_DEG` was raised from 60° to 80° in v23 because two similar poses produce a poorly conditioned mount. Do not lower it to make calibration easier; stand the aircraft up properly instead.

## Using the log to tune

After each flight, dump the log and compare:

| Column | What to look for |
|---|---|
| `rate_sp_x/y/z` vs `gyro_x/y/z` | The gyro should follow the setpoint without a large lag or overshoot. |
| `out_x/y/z` | Sustained saturation means the axis lacks authority or the gain is too high. |
| `i_x/y/z` | A large, slowly growing integrator suggests a mechanical trim problem or wind-up. |
| `motor_l_cmd` / `motor_r_cmd` | Look for the mixer floor and for asymmetry. |
| `tau` | Confirm the transition timing. |
| `nose_deg`, `bank_deg` | Confirm the attitude actually reaches the target. |
| `ahrs_trust`, `acc_g` | Low trust or an accel far from 1 g means the estimate was poor during that manoeuvre. |
| `imu_glitches`, `loop_hz` | Glitches or a low loop rate point to an I2C or flash stall. |

See [FLIGHT_LOG.md](FLIGHT_LOG.md) for the full column list.

## After tuning

- Re-run `HW_TEST` for directions and motor order.
- Re-check the arm and transition interlocks on the bench.
- Re-check the failsafe.
- Update this document if you settle on values that differ from the defaults, and say what airframe they suit.

---

Next: **[FLIGHT_LOG.md](FLIGHT_LOG.md)** · **[SAFETY.md](SAFETY.md)** · **[ARCHITECTURE.md](ARCHITECTURE.md)**
