# Safety

This is a hobby flight controller for a small tail-sitting flying wing. It is not a certified aviation system. It has one IMU, no redundancy, no altitude or airspeed sensing, and no way to recover from a hardware failure. Treat it accordingly.

> [!WARNING]
> **Remove both propellers for all bench testing.** The sketch commands two ESCs and two servos. Treat the aircraft as loaded whenever the battery is connected.

## Hard rules

1. **Props off for all bench work.** Uploads, calibration, `RX_TEST`, `HW_TEST`, `IMU_TEST`, `ESC_CAL`, gain changes and mixer checks all happen with the propellers removed.
2. **Props on only when the aircraft is ready to fly**, the area is clear, and you are behind it or beside it — never in front of the props.
3. **Power the radio first, then the aircraft.** Power the aircraft down first.
4. **One BEC, or cut the red wire on the second ESC.** Parallel BECs can fight and overheat.
5. **All grounds common.** A missing ground is a classic “works on the bench, fails in the air” fault.
6. **Do not use the CodeCell battery connector in the aircraft.** Power the CodeCell from the 5 V pin.
7. **Configure the receiver failsafe to no pulses.** A receiver that holds stale channel values can defeat the link-loss detector.
8. **Calibrate with the nose up.** The two-pose calibration cannot tell nose-up from nose-down.
9. **Arm only when the aircraft is still and upright.** The interlocks enforce this; do not try to defeat them.
10. **Disarm before touching the aircraft.** CH5 low, every time.

## Safety interlocks in the firmware

### Arming

| Check | Limit | Why |
|---|---|---|
| Throttle low | ≤ 0.05 | Prevents an arm at power. |
| CH6 vertical | Low | Prevents arming in forward mode. |
| Upright | Within 30° of vertical | The hover controller assumes a tail-sitter attitude. |
| Still | Gyro < 20 °/s for 100 ms | Prevents arming while the aircraft is being handled or is vibrating. |
| Attitude trusted | `ahrsTrust` ≥ 0.5 | Prevents arming on a bad accelerometer estimate. |
| Switch cycle | CH5 low → high | One attempt per flick; no auto-retry. |

A refusal leaves the arm switch latched low. Release CH5 and flick it again after fixing the cause.

### Forward transition

| Check | Limit | Why |
|---|---|---|
| Time since arm | ≥ 1 s | Lets the hover settle before a transition. |
| Throttle hold | ≥ 0.30 for 0.5 s | Prevents a transition from a throttle bump or on the ground. |
| CH6 | High | Pilot command. |
| Throttle floor | 0.65 during the transition | Keeps the wing flying while it pitches down. |

A pending transition is cancelled if CH6 returns to low, the throttle hold is broken before the transition starts, the aircraft disarms, or it enters failsafe.

### Automatic disarm

| Condition | Threshold | Time |
|---|---|---|
| Hover tilt | > 70° from vertical | 1 s |
| Transition / forward attitude error | > 60° | 1 s |
| IMU loss | No fresh sample for 200 ms | Immediate |
| Failsafe timeout | 15 s | 15 s |

A disarm stops the motors. It does **not** level the aircraft, deploy a parachute or bring it home. If it happens in the air, the aircraft falls.

## Failsafe

On link loss while armed:

1. Enter **FAILSAFE**.
2. Pitch back toward hover over 1.5 s.
3. Hold throttle at **0.65** while transitioning, then **0.48** in hover.
4. Level the elevons.
5. Disarm after **15 s**.

If the link returns before the timeout, the state returns to **HOVER** and the pilot regains control.

### Why “no pulses”

The built-in CRSF parser cannot see a receiver “hold” failsafe bit. If the receiver keeps sending the last channel values, the controller still sees a valid frame and never enters failsafe. Configure the ExpressLRS receiver to **no pulses** so the link loss is visible.

### The 15 s timeout is a trade-off

- Too short: a brief dropout disarms the motors while the aircraft is still airborne.
- Too long: a real loss leaves the motors running for 15 s.

The current value is a compromise. It is not adaptive and it does not know the altitude. Do not treat it as a recovery system.

## Residual risks

These are known and **not** fully mitigated by the current firmware.

| Risk | Effect | Mitigation |
|---|---|---|
| Two-pose calibration cannot distinguish nose-up from nose-down | A mirrored mount produces a mirrored attitude reference | Strict nose-up procedure, 80° minimum pose angle, nose-angle check, recalibrate after mount changes |
| No magnetometer | Heading drifts slowly; no absolute yaw | Heading hold is relative; do not rely on it for navigation |
| No altitude, airspeed or GPS | Failsafe and transition throttles are fixed fractions | Fly visual line of sight; keep altitude margin; test failsafe at altitude |
| No battery voltage sensing | No low-voltage warning or cutoff | Timer, telemetry or voltage alarm; land with margin |
| Single IMU, single board | An IMU or board failure disarms the motors | Preflight checks; secure mounting; accept the risk |
| I2C bus stalls | Control-loop gap or IMU loss | Bus timeout and recovery; dt guard resets PIDs; IMU-loss disarm |
| Flash writes in the control loop | Occasional loop-rate dip | dt guard, PID reset; keep the log partition healthy |
| BEC brownout under servo load | ESP32 reset in flight | Size the BEC for two servos plus RX; add a capacitor; use one BEC |
| ESC or motor failure | Asymmetric thrust, loss of control | Preflight motor test; matched hardware; accept the risk |
| RF link loss or interference | Failsafe | ELRS, antenna placement, range check, no-pulses failsafe |
| Wind and ground effect | Hover instability | Fly in low wind; keep altitude; tune in calm air |
| No geofence or return-to-home | Fly-away | Visual line of sight; conservative range; pilot judgement |
| No companion interface | No external failsafe or autonomy | The C3 remains the only controller; any future companion needs a watchdog and a pilot enable |

### The calibration ambiguity in detail

The vertical pose gives the accelerometer an “up” vector along the aircraft's axis. It does not say which end is the nose. If the aircraft is stood on its tail **nose-down**, the learned nose axis is reversed. The controller will then believe the aircraft is pitched the wrong way.

The firmware requires at least 80° between the flat and vertical poses and warns below 85°, which rejects a lazy hold. It also prints the estimated nose angle, which should be about **+90°** at the vertical pose. These checks catch a bad hold, but they do not resolve the nose-up/nose-down ambiguity itself. A third pose or a magnetometer would. Until then, the procedure is the mitigation.

## Preflight checklist

With props fitted, before every flight:

- [ ] Radio on, correct model, throttle low, CH5 low, CH6 low.
- [ ] Battery charged and secured; correct cell count.
- [ ] One BEC only, or red wire cut on the second ESC.
- [ ] All grounds common; no loose connectors.
- [ ] Props tight, undamaged, correct rotation, counter-rotating pair.
- [ ] CG at the recommended point.
- [ ] Elevons free, hinges secure, servo arms tight.
- [ ] Antenna clear of ESCs, battery leads and carbon.
- [ ] Calibration loaded; no cyan/magenta sequence at boot.
- [ ] Log not full; LED not yellow.
- [ ] Wind within limits; people and property clear.
- [ ] You are standing behind or beside the aircraft, never in front of the props.

## Arm checklist

- [ ] Aircraft on its tail, wings vertical, nose up.
- [ ] CH6 low.
- [ ] Throttle fully down.
- [ ] Aircraft still.
- [ ] CH5 low → high.
- [ ] LED orange; motors at idle.
- [ ] Control response checked before lifting off.

If the arm is refused, **fix the cause**. Do not relax the interlock.

## First-flight protocol

1. Complete every bench test with props off: `RX_TEST`, `HW_TEST`, `IMU_TEST`, calibration, arm refusals, failsafe.
2. Fit the props. Confirm rotation and tightness.
3. Hover low for 10–15 seconds. Land, disarm, check for heat, loose parts and vibration.
4. Hover at a safe altitude. Check roll, pitch, yaw and heading hold.
5. Land, disarm, dump the log. Check `loop_hz`, `ahrs_trust`, `acc_g`, `imu_glitches` and the PID outputs.
6. Only then try a transition at altitude, with clear space ahead and a finger ready to flip CH6 back.
7. Transition back, land, disarm, review the log.

If anything is uncertain, stop. A tail-sitter in hover has no glide-back option.

## Emergency procedures

### Link loss

- The aircraft enters failsafe and pitches back to hover.
- If the link returns, take control and land.
- If it does not, the motors stop after 15 s. Keep people clear.

### Fly-away or loss of orientation

- If you still have control, reduce throttle and fly a predictable descent.
- Disarming stops the motors immediately. The aircraft will fall. Only do this if the alternative — a powered fly-away — is more dangerous.
- Do not chase the aircraft into people, traffic or restricted airspace.

### Crash

1. **Disarm** (CH5 low) or power off if the aircraft is still powered.
2. Keep people away. A LiPo can ignite after a crash.
3. Wait for the battery to cool before handling it. Move it to a safe, non-flammable place if it is damaged.
4. Inspect the airframe, props, wiring and mounts before any further flight.
5. Recalibrate if the CodeCell or its mount moved.

### Fire

- Use a LiPo-safe bag or a metal container if available.
- Do not use water on a lithium fire.
- Evacuate and call emergency services if the fire spreads.

## Legal and insurance

- Fly within your local rules: line of sight, altitude limits, no flying over people, no flying near airports or restricted airspace.
- Check whether your insurance covers experimental/home-built aircraft.
- This software is provided under the MIT license, **without warranty**. You are responsible for the aircraft and its operation.

## If you add a companion or autonomy

The current sketch has no companion interface. If you add one, follow the engineering review's guidance:

- Keep the C3 as the flight controller.
- Use a dedicated pilot-enable switch (for example CH7).
- Use a strict command watchdog and a timeout.
- Validate CRC, length and bounds.
- Fall back to RC on any fault.
- Never let a companion command motors directly.

See [TailSitter_REVIEW.md](../TailSitter_REVIEW.md) §11–§13 for the design options.

---

Next: **[TROUBLESHOOTING.md](TROUBLESHOOTING.md)** · **[FLIGHT.md](FLIGHT.md)** · **[CALIBRATION.md](CALIBRATION.md)**
