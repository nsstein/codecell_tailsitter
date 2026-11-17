# Troubleshooting

Start here:

1. **Props off.**
2. Open Serial Monitor at **115200**.
3. Set `DEBUG 1` if you need status lines and refusal reasons.
4. Check the LED pattern first — it tells you which phase the board is in.

## Quick reference

| Symptom | Most likely cause | Go to |
|---|---|---|
| No LED, no serial | Power, USB cable, boot mode | [Power and boot](#power-and-boot) |
| Fast red LED, nothing else | IMU not found | [IMU](#imu) |
| Green blink but no receiver frames | Wiring, baud, receiver protocol, unlinked ELRS | [Receiver](#receiver) |
| Servos do not move | No link, wrong pins, servo power | [Servos](#servos) |
| Servo moves the wrong way | `SERVO_REV_*` | [Servos](#servos) |
| Motors do not spin | Not armed, ESC calibration, protocol mismatch | [Motors and ESCs](#motors-and-escs) |
| Motor spins the wrong way | Swap two motor wires or reverse in the ESC | [Motors and ESCs](#motors-and-escs) |
| Arm refused | One of the five interlocks | [Arming](#arming) |
| Transition will not start | CH6, throttle hold, arm delay, link | [Transition](#transition) |
| Aircraft drifts or oscillates | Trim, vibration, gains | [Flight behaviour](#flight-behaviour) |
| Failsafe does not trigger | Receiver failsafe is “hold”, not “no pulses” | [Failsafe](#failsafe) |
| Log unavailable | Wrong partition scheme | [Flight log](#flight-log) |
| Calibration will not finish | Movement, pose angle, IMU data | [Calibration](#calibration) |

## Power and boot

### No LED, no serial

- Use a **data** USB-C cable, not a charge-only cable.
- Check the CodeCell is powered. On the bench, USB alone is enough; in the aircraft, the 5 V BEC must be present.
- Try another USB port or hub.
- If the port does not appear, hold **BOOT** while plugging in, then upload.
- Confirm **USB CDC On Boot: Enabled** in the Arduino IDE.

### Board resets when the motors spin

This is almost always a **BEC brownout**.

- Power the servos and receiver from the BEC, not from a GPIO.
- Size the BEC for two servos plus the receiver. A small BEC that sags under servo load resets the ESP32-C3.
- Use **one** BEC, or cut the red wire on the second ESC.
- Check all grounds are common.
- Add a low-ESR capacitor across the 5 V bus near the ESCs.
- Keep the battery leads short and away from the receiver antenna.

### Board boots, then halts with fast red

The BNO085 was not found. See [IMU](#imu).

## IMU

### Fast red LED, motors off

The sketch halts when `imuBegin()` fails. It prints:

```text
FATAL: BNO085 not found at 0x4A - check IMU_DRIVER / library.
```

- Confirm you are using the **CodeCell** library.
- Set `IMU_DRIVER 1` (the default).
- Do not install the SparkFun BNO08x library; it conflicts with the CodeCell library.
- Check the board is a CodeCell C3 with the onboard BNO085.
- Power-cycle. If it persists, run `IMU_TEST 1` for raw diagnostics.

### IMU rates are low

- With `IMU_DRIVER 1`, the gyro should be near **200 Hz** and the accel near **100 Hz**.
- `IMU_DRIVER 0` uses the CodeCell library path and measures only about 30 Hz of new gyro data. It is not recommended for hover.
- A low rate with `IMU_DRIVER 1` points to I2C trouble: a bad connection, bus noise, or a stalled bus. The firmware re-initialises the bus after an error burst and increments `imuBusRecoveries`.

### Attitude drifts or jumps

- The aircraft must be **still at boot** for the attitude to initialise cleanly.
- A loose CodeCell mount adds gyro noise. Foam-tape the board and keep it away from ESC heat and prop wash.
- Recalibrate after moving the board. See [CALIBRATION.md](CALIBRATION.md).
- Check `imu_glitches` in the log. A rising counter points to I2C noise.

## Receiver

### No frames at all

1. Run `RX_TEST 1` with the radio on and the receiver bound.
2. Confirm the receiver is **powered** and its LED shows a link. An unlinked ELRS receiver sends nothing.
3. Confirm the receiver **TX** goes to CodeCell **IO1**.
4. Confirm **Serial Protocol = CRSF** in the receiver.
5. Confirm **420 kbaud** (ELRS default). `RX_TEST` sweeps 420000, 416666, 400000, 115200 and 921600, plus inverted polarity.
6. Confirm a **common ground** between the receiver and the CodeCell.

`RX_TEST` verdicts:

| Verdict | Meaning |
|---|---|
| `CRSF OK on IO1 at 420000 baud` | Wiring and protocol are correct. |
| `data but no valid CRSF frames` | Set the receiver Serial Protocol to CRSF. |
| `no data on ANY pin` | Receiver not powered, not bound, or not linked. |
| `pin toggles but UART reads nothing` | The wire is on the wrong pad (TX vs RX). |

### Frames arrive but the link is “lost”

`rcLinkOk()` requires all of:

- At least one valid frame ever received.
- No receiver-reported failsafe.
- A frame within **250 ms**.
- Link quality not equal to **0**.

If the receiver is sending frames with LQ 0, the controller treats it as a link loss. That is intentional. Check the radio link and the receiver failsafe configuration.

### `CRSF_LIB 1` receives nothing

The built-in parser (`CRSF_LIB 0`) is the verified default on this board. The library path received zero frames in testing. Set `CRSF_LIB 0` unless you have a specific reason to use the library.

## Servos

### Servos do not move

- Confirm the receiver link is good (`RX_TEST`, then `HW_TEST`).
- Confirm the servos are on **IO6** (left) and **IO7** (right), with 5 V and ground from the BEC.
- Confirm the servos are powered. A GPIO cannot power a servo.
- While disarmed, the elevons follow the sticks. If they do not move in `HW_TEST`, the problem is wiring or power, not the flight code.

### A servo moves the wrong way

Flip its direction constant:

```c
#define SERVO_REV_L  false
#define SERVO_REV_R  true
```

Do this in the sketch, not in the radio. Radio reversing would make the disarmed stick-follow mode disagree with the flight controller.

### Servos buzz or jitter at the endpoints

- Reduce `SERVO_THROW_US` (default 400).
- Tighten `SERVO_MIN_US` / `SERVO_MAX_US` (default 1000/2000).
- Check for binding hinges or a servo arm hitting a limit.
- Check the BEC voltage under load; a sagging BEC causes jitter.

### One elevon does not centre

- Centre the servo arm mechanically first.
- Use `SERVO_CENTER_L/R` only for fine trim.
- Check for a stripped servo, a loose arm or a binding hinge.

## Motors and ESCs

### Motors do not spin when armed

- The aircraft must be **armed** (orange LED). In `HW_TEST`, the motor test needs throttle low, then CH5 high.
- The throttle must be above idle. At arm, motors run at idle (`MOTOR_IDLE_US` 1080 or DShot 100), which may be too low for some ESCs to start. Raise the throttle slightly.
- Confirm `ESC_PROTOCOL` matches the ESCs. The default is **3 (OneShot125)**. A PWM ESC on OneShot may not arm.
- For analog protocols, run the `ESC_CAL 1` procedure. DShot needs no calibration.
- Check the ESC is powered and the signal wire is on the right pin: **IO5 left**, **IO3 right**.

### A motor spins the wrong way

- Swap any two of the three motor wires, **or**
- Reverse the motor direction in the ESC configurator (BLHeli / Bluejay / AM32).

Do not try to reverse a motor in the flight code. The mixer assumes the props are counter-rotating and the thrust directions are correct.

### One motor starts later or runs faster

- Re-run the ESC throttle calibration for analog protocols.
- Check both ESCs have the same firmware and settings (timing, PWM frequency, brake).
- Check the motors are the same kV and the props are the correct pair.
- Check for a damaged prop, a tight bearing or a bad solder joint.
- Some asymmetry at idle is normal; a large asymmetry in hover is not.

### Motors stutter at low throttle

- Raise `AIRMODE_MIN_THR` slightly (default 0.15) so the low motor keeps more RPM.
- Check the ESC idle setting and motor timing.
- Check the battery can supply the current; a sagging pack causes stutter.
- Check the mixer is not clipping differential at the floor. See [ARCHITECTURE.md](ARCHITECTURE.md).

### ESC calibration does not work

- `ESC_CAL 1` only works with analog protocols: `ESC_PROTOCOL` `0`, `3`, `4` or `5`.
- DShot (`1`, `2`) is digital and needs no calibration.
- `ESC_CAL 1` cannot be combined with `RX_TEST`, `HW_TEST` or `IMU_TEST`.
- Follow the sequence exactly: throttle **full up** before power-on, wait for the max tones, then **full down** within about 2 seconds.
- The LED is **fast red** until the receiver links, then **fast white**. If it stays red, the receiver is not linking.

## Arming

The arm is refused unless **all** checks pass. With `DEBUG 1`, the reason is printed.

| Message | Cause | Fix |
|---|---|---|
| `throttle not low` | CH3 above 0.05 | Lower the stick and the trim. |
| `CH6 must be VERTICAL` | CH6 is high | Flip CH6 low. |
| `not upright (xx deg)` | More than 30° from vertical | Stand the aircraft on its tail, nose up. |
| `aircraft moving (xx dps)` | Gyro above 20 °/s | Hold it still. Fix vibration or a loose mount. |
| `attitude estimate untrusted (x.xx)` | `ahrsTrust` below 0.5 | Let it settle; check the accel and calibration. |

Other reasons the arm does nothing:

- CH5 was not cycled **low → high**. The firmware allows one attempt per flick.
- No RC link. Failsafe or no frames prevents arming.
- The arm switch endpoint is below `ARM_SWITCH_US` (1700). Check the channel value in `HW_TEST`.

Do not weaken the interlocks to make arming easier. Fix the cause.

## Transition

### CH6 high does nothing

The forward transition needs:

- Armed.
- CH6 high.
- At least **1 s** since arming.
- Throttle at or above **0.30** for **0.5 s**.
- A valid RC link.

If you raise the throttle and lower it before the hold time, the pending start is cancelled. Raise it and hold.

### The transition starts immediately

- Check the CH6 endpoint. If CH6 reads above 1500 µs at “low”, the firmware thinks it is in forward mode.
- Check the radio has no mix or offset on CH6.
- Check the throttle is genuinely low. A throttle curve or trim can keep it above 0.30.

### The aircraft sags or pitches too aggressively

- Raise `TRANS_FWD_THR_MIN` (default 0.65) in small steps.
- Raise `TRANS_START_MIN_THR` so the transition starts with more energy.
- Check the battery, props and AUW. A tired pack or a heavy wing sags.
- Do not shorten `TRANS_FWD_SEC` until the aircraft has enough thrust margin.

### The aircraft will not pitch back to hover

- Check CH6 actually goes low.
- Check the throttle is not so low that the wing stalls. `TRANS_BACK_THR_MIN` is 0.65 during the pitch-up.
- Check for a crash event or a disarm in the log.

## Flight behaviour

### Persistent drift in hover

- Fix the mechanical trim first: CG, thrust line, motor mounts, a bent airframe.
- Check both motors and props are matched.
- Check the calibration mount. Recalibrate if the board moved.
- Do not use the integrator to hide a mechanical problem.

### Fast oscillation or shake

- Reduce `rP` by 20 % on the affected axis.
- If it persists, reduce `rD`.
- Check for vibration: a loose CodeCell, a bent prop, a bad motor bearing.
- Check `loop_hz` and `imu_glitches` in the log.

### Slow wobble

- Reduce `angP` by 20 % on the affected axis.
- Check the CG. A tail-heavy wing is hard to hover.
- Check the hover tilt limit; 15° may be too much for a small, light wing.

### “Toilet bowl” circling

- Check the heading hold and the yaw axis. The hover yaw gain (`angP` X = 2.0) may be too high or too low.
- Check for a bent motor mount or a thrust-line offset.
- Check the calibration. A mirrored or skewed mount produces a heading-dependent lean.
- Reduce `HOVER_MAX_TILT_DEG` for the test.

### Pitch direction feels reversed

- Verify in `HW_TEST`: stick back should move both elevons up.
- `RC_REV_PITCH` is `true` by default for EdgeTX/OpenTX mode-2 radios.
- Check the radio channel direction and the servo direction constants.

### Yaw is weak in forward flight

- Differential thrust is inherently softer than a rudder.
- Raise `FWD_YAW_RATE_DPS` (default 60) or `DIFF_THRUST_MAX` (default 0.30) in small steps.
- Check both motors and props are matched and counter-rotating.
- Use bank to turn; yaw is a trim, not the primary turn control.

### Heading drifts

There is no magnetometer. Heading is relative and will drift slowly, especially after aggressive yaw or a long flight. This is expected. Re-arm or re-establish hover to reset the heading target.

### The crash detector disarms in normal flight

- Hover: tilt above 70° for 1 s. Check for a real upset or a bad attitude estimate.
- Transition/forward: attitude error above 60° for 1 s. A very aggressive manoeuvre or a large wind gust can trigger it.
- Check `ahrs_trust` and `acc_g` in the log. A poor estimate can cause a false error.
- Do not raise `CRASH_TILT_DEG` or `CRASH_ATT_ERR_DEG` without understanding the trade-off.

## Failsafe

### Failsafe does not trigger when the radio is switched off

- The receiver failsafe is probably set to **hold**, not **no pulses**. The built-in parser cannot see a hold failsafe bit. Configure **no pulses**.
- Check `rx_fs`, `link` and `lq` in the log.
- Confirm the receiver is actually losing the link, not just the radio display.

### Failsafe triggers randomly

- Check the antenna placement. Keep it away from ESCs, battery leads and carbon.
- Check the receiver power and ground.
- Check the radio link quality (`lq`) in the log.
- Check for a loose connector or a damaged antenna.
- Range-check the system before flying.

### Failsafe disarms after 15 s even though the link returned

- The link must return **before** `FS_DISARM_MS` (15 s).
- Check whether the receiver re-linked but the controller still saw LQ 0 or a failsafe flag.
- Check the log for `FAILSAFE` and `LINK_RESTORED` events.

## Flight log

### `Logging unavailable (no partition)`

Set the partition scheme to **Default 4MB with spiffs** and re-upload. The log writes to the `spiffs` data partition, not a file system.

### `list` is empty after a flight

- The aircraft must have been **armed**. The log records only while armed.
- Check the log was not erased.
- Check the partition is present.
- Check the arm actually happened; a refused arm records nothing.

### `dump` is truncated or garbled

- Use a terminal that can capture the full output, or redirect to a file.
- Keep the USB cable connected and the board powered.
- The dump uses a longer USB timeout; a slow terminal can still drop data.
- Check for `### LOG END`; if it is missing, the dump was interrupted.

### The log fills too quickly

The log records 50 samples per second and holds roughly 10 minutes. Dump and `erase` between sessions. A yellow LED while disarmed means 95 % full. When full, recording stops.

### Upgrading erased my old log

The record format changed in v20 (64 bytes). The firmware erases an incompatible log on first boot. Dump old logs before upgrading.

## Calibration

See [CALIBRATION.md](CALIBRATION.md) for the full procedure. Quick checks:

| Symptom | Fix |
|---|---|
| Stuck on fast cyan | Hold still on a solid surface; check `IMU_TEST`. |
| Never reaches fast magenta | Rotate at least 20° away from flat. |
| Stuck on fast magenta | Stand fully on the tail, nose up, within 80° of flat, and hold still. |
| Nose reads −90° when upright | Vertical pose was captured nose-down. Recalibrate. |
| Calibration not saved | Check the serial warning; NVS may have failed. It still applies to this power-up. |
| Attitude drifts after boot | Boot stationary; recalibrate if it persists. |

## Serial commands

### Commands are ignored

- Commands work **only while disarmed**.
- `LOG_ENABLE` must be `1`.
- Use **115200** baud.
- Send a newline (`\n`) after the command. The parser ignores `\r`.
- The command buffer is 23 characters; keep commands short.

### Nothing prints at all

- `DEBUG 0` and `LOG_ENABLE 0` silence the port.
- Set `DEBUG 1` for status lines.
- With `LOG_ENABLE 1`, the port starts even with `DEBUG 0`; it prints log and calibration messages, but not the debug status lines.

## Compile errors

| Error | Fix |
|---|---|
| `multiple definition of ... bno08x` | Uninstall the SparkFun BNO08x library. |
| `ESC_CAL 1 cannot be combined with ...` | Set `RX_TEST`, `HW_TEST`, `IMU_TEST` to `0`. |
| `ESC_CAL 1 needs an analog protocol` | Use `ESC_PROTOCOL` 0, 3, 4 or 5, or set `ESC_CAL 0`. |
| `ESC_PROTOCOL has no value` | The define line is empty; give it a number. |
| `DSHOT_SPEED must be 150, 300 or 600` | Fix the value. |
| `LogRec must be 64 bytes` | The record struct changed; restore the packed layout. |
| `AIRMODE_MIN_THR must leave room ...` | Keep the floor between 0 and 0.5. |

## Still stuck?

1. Set `DEBUG 1`, reproduce the problem, and capture the serial output.
2. Dump the flight log if the problem happened in flight.
3. Check [TailSitter_REVIEW.md](../TailSitter_REVIEW.md) for the known residual risks.
4. Change one thing at a time. A tail-sitter has many coupled behaviours; changing three parameters at once makes the cause unknowable.

---

Next: **[SAFETY.md](SAFETY.md)** · **[BUILD.md](BUILD.md)** · **[FLIGHT.md](FLIGHT.md)**
