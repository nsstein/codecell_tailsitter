# Flight

This is the pilot's guide: radio setup, preflight, arming, hover, the forward transition, landing, failsafe and the LED meanings.

> [!WARNING]
> Props on only when the aircraft is ready to fly and the area is clear. All bench work happens with props off.

## Radio setup

### Channels

| Channel | Function | Hover | Forward |
|---|---|---|---|
| **CH1** | Roll | Lean left / right | Bank |
| **CH2** | Pitch | Lean forward / back | Pitch |
| **CH3** | Throttle | Collective thrust | Thrust |
| **CH4** | Yaw | Rotate about vertical | Differential-thrust yaw |
| **CH5** | Arm / disarm | High = arm, low = disarm | Same |
| **CH6** | Flight mode | Low = vertical / tail-sitter | High = forward / plane |

### Model setup

- Use a **simple 4-channel + 2 aux** model.
- **Do not add elevon, delta or V-tail mixing in the radio.** The flight controller does all mixing. Radio mixing would double-mix the elevons.
- Set endpoints to approximately **1000–2000 µs**, centre **1500 µs**.
- Set all dual rates to **100 %** and radio expo to **0**. The sketch applies its own expo (`RC_EXPO 0.35`) and its own rate limits.
- Set the throttle trim low enough that the stick reads below `ARM_MAX_THR` (0.05) when fully down. A throttle cut switch is fine, but the arm check only looks at the channel value.
- CH5 must be a two-position switch that goes **low → high**. Arming is one attempt per flick; if it is refused, release CH5 and flick it again.
- CH6 must be **low** to arm. Low is vertical/tail-sitter; high is forward/plane.
- Verify the channel directions in `HW_TEST` before the first flight. `RC_REV_PITCH` is `true` by default because EdgeTX/OpenTX mode-2 radios send a low value when the elevator stick is pulled back.

### Receiver failsafe

Configure the ExpressLRS receiver failsafe to **no pulses**. The built-in CRSF parser cannot see a “hold” failsafe bit, so a receiver that keeps sending stale channel values can defeat the link-loss detector. With no pulses, the controller sees the link drop and enters failsafe.

Also confirm:

- Receiver Serial Protocol = **CRSF**.
- Baud **420000** (ELRS default).
- The receiver is bound and linked. An unlinked ELRS receiver sends nothing.
- The radio is on before the aircraft.

## Preflight

With the aircraft on the ground, props fitted, and the area clear:

1. **Radio on**, correct model selected, throttle low, CH5 low, CH6 low.
2. **Battery charged**, correct cell count, secured.
3. **BEC and wiring** secure. One BEC only, or the red wire cut on the second ESC.
4. **CG** at the airframe designer's point. Battery not loose.
5. **Props** tight, correct rotation, counter-rotating pair, no cracks.
6. **Control surfaces** free, hinges secure, servo arms tight.
7. **Antenna** clear of the ESCs and battery leads.
8. **Flight log** not full. A **yellow** LED while disarmed means the log is at least 95 % full. Dump and erase it before flying.
9. **Calibration** loaded. No cyan/magenta sequence at boot.
10. **Wind** within the aircraft's capability. Hover is the most wind-sensitive mode.
11. **People** clear of the aircraft and the flight path.

Power-up order: **radio first, then aircraft**. Power-down order: **aircraft first, then radio**.

## Arming

Stand the aircraft on its tail, wings vertical, nose up.

1. Confirm CH6 is **low** (vertical).
2. Confirm the throttle stick is **fully down**.
3. Hold the aircraft **still**.
4. Flick CH5 **low → high**.

The controller arms only if **all** of these are true:

| Check | Limit |
|---|---|
| Throttle low | ≤ `ARM_MAX_THR` 0.05 |
| CH6 vertical | Low |
| Upright | Within `ARM_MAX_TILT` 30° of vertical |
| Still | Gyro below `ARM_MAX_RATE_DPS` 20 °/s for `ARM_STILL_MS` 100 ms |
| Attitude trusted | `ahrsTrust` ≥ `ARM_MIN_TRUST` 0.5 |

If a check fails, the arm is refused. With `DEBUG 1` the serial monitor prints the reason:

```text
ARM refused: throttle not low
ARM refused: CH6 must be VERTICAL
ARM refused: not upright (42 deg)
ARM refused: aircraft moving (35 dps)
ARM refused: attitude estimate untrusted (0.20)
```

Release CH5 and flick it again after fixing the cause. A refusal does not retry by itself.

When armed:

- The LED blinks **orange**.
- Both motors are commanded to idle (`MOTOR_IDLE_US` 1080, or DShot idle 100). **The props may turn.**
- The elevons are active.
- The flight log starts a new session.

To disarm at any time: CH5 **low**. The motors stop and the elevons return to direct stick control.

## Hover

The aircraft is self-levelling in hover. The right stick commands a lean, not a rate.

| Control | Authority |
|---|---|
| Roll / pitch lean | Up to `HOVER_MAX_TILT_DEG` 15° from vertical |
| Yaw rate | Up to `HOVER_YAW_RATE_DPS` 90 °/s |
| Heading hold | Yes, with a 25° slack limit |
| Differential thrust | Yaw and side-lean, clipped by the mixer floor |

Takeoff:

1. Arm as above.
2. Raise the throttle **slowly**. The aircraft should lift straight up.
3. Climb to a safe altitude — at least a few metres — before making inputs.
4. Check roll, pitch and yaw response. Small inputs first.
5. Confirm the aircraft holds heading and does not drift persistently.

The hover throttle is whatever holds altitude. For a small wing it is often around **0.55**, but it depends on weight, props and battery. The failsafe descent throttle (`FS_THR` 0.48) is deliberately set just below that typical point; check your own hover throttle in the log and adjust if needed. See [TUNING.md](TUNING.md).

Keep the first flight in hover. Do not transition until the hover is stable and the controls are confirmed.

## Transition to forward flight

1. Establish a stable hover at a safe altitude, with clear space ahead.
2. Raise the throttle to at least **0.30** and hold it.
3. Flip **CH6 high**.

The transition starts only when:

- At least **1 s** has passed since arming (`TRANS_ARM_DELAY_MS`).
- The throttle has stayed at or above **0.30** for **0.5 s** (`TRANS_START_HOLD_MS`).

Then `tau` ramps from 0 to 1 over **2.0 s** (`TRANS_FWD_SEC`). The nose pitches down, the aircraft accelerates, and the throttle floor is **0.65** (`TRANS_FWD_THR_MIN`). Once `tau > 0`, the transition continues even if the throttle later drops.

If you flip CH6 high with the throttle low, nothing happens. Raise the throttle and hold it; the transition begins after the hold time. If you flip CH6 back to low before it starts, the pending start is cancelled.

During the transition:

- Keep the throttle up.
- Do not fight the pitch-down; let the aircraft accelerate.
- Keep the wings level with small roll inputs.

## Forward flight

Forward flight is angle mode:

| Control | Authority |
|---|---|
| Bank | Up to `FWD_MAX_BANK_DEG` 45° |
| Pitch stick | ±`FWD_MAX_PITCH_DEG` 25° around the 3° trim |
| Yaw | Differential thrust, up to `FWD_YAW_RATE_DPS` 60 °/s |
| Heading hold | **None** |

- The wings generate lift; the motors provide thrust.
- Bank to turn. Add a little pitch to hold altitude in a bank.
- Yaw is differential thrust, so it is softer than a rudder. Use bank for turns.
- There is no heading hold in forward flight. The nose will drift in wind; correct it with bank or yaw.
- There is no altitude hold and no airspeed hold.

## Transition back to hover

1. Point the aircraft into wind if possible.
2. Reduce throttle to a comfortable level, but keep flying.
3. Flip **CH6 low**.

`tau` ramps from 1 to 0 over **1.5 s** (`TRANS_BACK_SEC`). The nose pitches up, the throttle floor is **0.65** (`TRANS_BACK_THR_MIN`), and the aircraft settles into hover.

Once `tau` reaches 0, heading hold resumes. Be ready to catch a small heading change as the controller re-establishes the target.

## Landing

1. Transition back to hover at a safe altitude.
2. Descend gently with small throttle changes.
3. Keep the aircraft upright and into wind if there is any.
4. Touch down on the tail skids.
5. **Disarm with CH5 low** as soon as the aircraft is down.

The firmware has no landing detector. `LANDED_THR` 0.30 only freezes the PID integrators below that throttle in hover; it does not stop the motors. **You must disarm.**

If the aircraft tips over, disarm immediately. The hover crash detector disarms after 1 s above 70° tilt, but do not rely on it.

## Failsafe

If the radio link is lost while armed:

1. The state machine enters **FAILSAFE**.
2. `tau` is driven back toward hover over 1.5 s.
3. Throttle is held at **0.65** while `tau > 0`, then drops to **0.48** (`FS_THR`) in hover.
4. The elevons level the wings and pitch the aircraft back toward hover.
5. After **15 s** (`FS_DISARM_MS`) the controller disarms and the motors stop.

If the link returns before the timeout:

- The state returns to **HOVER**.
- The pilot regains control.
- The forward-transition hold is cleared, so you must raise the throttle and hold it again before CH6 high will start a transition.

Failsafe behaviour is only as good as the receiver configuration. **Set the receiver failsafe to no pulses.** A receiver that holds stale values may keep the controller thinking the link is alive.

There is no altitude, airspeed or GPS sensing. The 15 s timeout is a deliberate trade-off: long enough to recover a brief link dropout, short enough to stop the motors after a real loss. See [SAFETY.md](SAFETY.md).

## Crash detection and automatic disarm

The controller disarms automatically when:

| Condition | Threshold | Time |
|---|---|---|
| Hover tilt from vertical | > 70° (`CRASH_TILT_DEG`) | 1 s |
| Transition / forward attitude error | > 60° (`CRASH_ATT_ERR_DEG`) | 1 s |
| IMU data loss while armed | No fresh sample for 200 ms (`IMU_LOSS_MS`) | immediate |

A disarm stops the motors and returns the elevons to direct stick control. It does not deploy a parachute, and it does not level the aircraft. If the aircraft is airborne when it disarms, it will fall.

## LED meanings

| LED | Meaning |
|---|---|
| Fast cyan | Calibration: waiting for the flat-on-belly pose |
| Solid cyan | Flat pose captured; rotate to vertical |
| Fast magenta | Calibration: waiting for the on-tail, nose-up pose |
| Solid magenta | Vertical pose captured; saving |
| Fast red | IMU not found (halted), or ESC calibration waiting for a link |
| Green blink | Ready, disarmed |
| Orange blink | Armed |
| Yellow blink | Disarmed, flight log at least 95 % full — dump and erase |
| Fast yellow | Erasing or preparing the flight log |
| Fast white | ESC calibration passthrough active |

## Flight limits

- **Visual line of sight only.** No GPS, no return-to-home, no geofence.
- **No battery voltage sensing.** The firmware cannot warn about a low battery or cut off the motors. Use a timer, telemetry or a voltage alarm, and land with margin.
- **No magnetometer.** Heading is relative and may drift slowly. Heading hold is a convenience, not a compass.
- **No altitude or airspeed sensing.** Failsafe and transition throttles are fixed fractions, not closed-loop height control.
- **One IMU.** A failed IMU disarms the motors after 200 ms.
- **Hover is the hard mode.** Wind, ground effect and a tired battery all show up there first.
- **The log holds about 10 minutes** at 50 Hz. When it is full, recording stops and the LED warns at 95 %. Dump and erase between flying sessions.

## First-flight protocol

Do this once, with props fitted, in a large open area, in low wind:

1. Bench-test everything with props off: `RX_TEST`, `HW_TEST`, `IMU_TEST`, calibration, arm checks, failsafe.
2. Fit the props. Confirm rotation and tightness.
3. Arm and hover at low height for 10–15 seconds. Land and disarm. Check for heat, loose parts and vibration.
4. Hover again, climb to a safe altitude, and check roll, pitch, yaw and heading hold.
5. Land and disarm. Dump the log and check `loop_hz`, `ahrs_trust`, `acc_g`, `imu_glitches` and the PID outputs.
6. Only then try a transition at a safe altitude, with clear space ahead and a finger ready to flip CH6 back.
7. Transition back to hover, land, disarm, and review the log.

If anything is uncertain, stop and fix it on the bench. A tail-sitter has no glide-back option in hover.

---

Next: **[SAFETY.md](SAFETY.md)** · **[TUNING.md](TUNING.md)** · **[FLIGHT_LOG.md](FLIGHT_LOG.md)**
