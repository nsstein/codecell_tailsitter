# TailSitter.ino — Flight-Controller Review

**File reviewed:** `/Users/steinne/personal/Radio_Control/TailSitter/TailSitter.ino` (v24, persistent boot calibration)
**Method:** full static read of the sketch; patch batches compiled for the target board. Hardware testing is still required.
**Target:** Microbots CodeCell C3 (ESP32-C3 + BNO085), ExpressLRS/CRSF, twin-motor tail-sitter.

Severity used below:

| Level | Meaning |
|---|---|
| **CRITICAL** | Can directly cause loss of control or a crash; fix before next flight. |
| **HIGH** | Can cause a crash in a plausible operating condition or make a failure worse. |
| **MEDIUM** | Robustness / failure-handling gap; may cause loss of control after another fault. |
| **LOW** | Efficiency, cleanliness, diagnostics, or data-loss issue. |

---

> **Update (v26, 2026-11-16):** Arming and forward-transition interlocks added,
> plus an elevon non-finite guard; see §20.
> **Update (v24, 2026-11-16):** Persistent boot calibration added. The two-pose
> result is stored in NVS and reused on later boots; see §18. Patch batch 1
> (v23) remains applied; see §17.

## 1. Top findings (read this first)

| # | Sev | Finding | Where |
|---|---|---|---|
| 1 | **CRITICAL** | Differential-thrust mixer can drive one motor to **zero** at low hover throttle. Losing one motor in hover removes yaw/side-lean authority. The `AIRMODE_MIN_THR` logic does not actually keep a minimum on the low motor. | 2219–2226 |
| 2 | **HIGH** | Failsafe pitch-back uses `FS_THR` (0.48), which is **below** the normal transition-back floor `TRANS_BACK_THR_MIN` (0.65). The aircraft can pitch up from forward flight while under-throttled. | 2130–2137, 2192–2194 |
| 3 | **HIGH** | Boot calibration accepts a vertical pose only **60°** from flat and does not reject roll error. A sloppy hold produces a skewed mount matrix → wrong attitude → wrong control. | 266, 1329–1376 |
| 4 | **HIGH** | Crash detection runs **only when `tau == 0`** (pure hover). A tip-over during transition or forward flight leaves the motors running. `crashStartMs` is also never reset on arm/disarm. | 2151–2155, 1758–1764 |
| 5 | **HIGH** | `landed` is true whenever `tau == 0 && throttle < 0.30`. A normal low-throttle descent is treated as "landed": integrators reset and **differential thrust is set to zero**. | 2196, 2206, 2222 |
| 6 | **MEDIUM** | I2C has no timeout and no bus recovery. A stuck BNO085 bus can block the control loop for the Wire default timeout, then trigger IMU-loss disarm. | 1080–1081, 980–1008 |
| 7 | **MEDIUM** | Flight-log flash writes happen **inside the control loop** at 50 Hz. Each write stalls the CPU/cache for a few hundred microseconds, adding jitter to IMU, CRSF and DShot timing. | 2232–2234, 1548–1556 |
| 8 | **MEDIUM** | PID derivative is computed with a `dt` clamped to 50 ms. A longer stall (I2C, flash, USB) divides a large gyro change by a too-small `dt` → derivative spike. | 2085–2089, 1382–1389 |
| 9 | **MEDIUM** | The built-in CRSF parser never sets `rcFailsafeFlag`. Failsafe is detected only by frame timeout or `LQ == 0`. A receiver configured to "hold" last values can keep the aircraft flying on stale inputs. | 507, 548–623, 1725–1727 |
| 10 | **MEDIUM** | Forward transition can start on the ground: `TRANS_START_MIN_THR` is a **stick** threshold, not an airborne/altitude check. | 2143–2149 |

> **Status (v25, 2026-11-16):** CRITICAL #1 is **closed**. The v23 mixer fix is
> verified by an exhaustive sweep and hardened in v25; see §19. The HIGH items
> were patched in batch 1 (§17) and will be verified and closed in the next pass.

---

## 2. Arming / disarming

**What is done well**
- CH5 must be cycled low → high before arming (2111, 2035).
- Throttle must be low, CH6 must be VERTICAL, and tilt must be within 30° of vertical (2113–2115).
- `disarm()` resets `tau`, state, PIDs and commands motor stop (1758–1764).
- Arming is refused if the RC link is not OK.

**Gaps / recommendations**
1. **No gyro/stillness check at arm.** A moving aircraft can pass the tilt check. Add `if (vNorm3(gyro) > ARM_MAX_RATE_DPS) refuse` (e.g. 20–30 dps). This is error handling, not a new feature.
2. **No AHRS-confidence check at arm.** `ahrsTrust` and `imuGlitches` are available. Refuse to arm if `ahrsTrust < 0.5` or glitches increased recently.
3. **`crashStartMs` is not cleared in `disarm()`** (1758–1764) or on arm. After a crash disarm, the next tip-over can trigger an immediate disarm because `millis() - crashStartMs` is already > `CRASH_TIME_MS`. Add `crashStartMs = 0;` in `disarm()` and at the arm transition.
4. **`crashStartMs == 0` is used as a sentinel.** This fails only if `millis()` is exactly 0, so it is not a practical bug, but a separate `bool crashTimerActive` is cleaner and avoids the reset bug above.
5. **Arming is allowed with a stale/held receiver failsafe.** Because the built-in parser never sets `rcFailsafeFlag` (finding 9), a receiver in "hold" failsafe can look like valid sticks. Configure ELRS failsafe to **no pulses** (or a preset that drops LQ to 0) and bench-test with the radio off, props removed.

---

## 3. Failsafe / RC link

**Current behaviour**
- `rcLinkOk()` = ever-valid && no receiver failsafe flag && frame age < 250 ms && `linkLQ != 0` (1725–1727).
- On link loss: state = `ST_FAILSAFE`, `tau` ramps to 0 over `TRANS_BACK_SEC`, sticks centred, throttle = `FS_THR`, disarm after 15 s (2130–2137, 2170, 2192).

**Issues**
1. **Failsafe throttle is lower than the normal transition-back floor.** Normal `ST_TO_HOVER` uses `max(thr, 0.65)` (2194), but failsafe uses 0.48 (2192). During the pitch-up from forward flight the aircraft may not have enough thrust/airflow. Recommended one-line fix:
   ```cpp
   thr = fs ? (tau > 0.0f ? TRANS_BACK_THR_MIN : FS_THR) : thrStk;
   ```
   or raise `FS_THR` to at least `TRANS_BACK_THR_MIN`.
2. **15 s failsafe timeout can disarm while still airborne.** This is a deliberate fly-away trade-off, but document it. If the aircraft is still descending at 15 s, cutting motors guarantees a fall. Consider a longer timeout or a "motors idle, attitude hold" state if you can estimate landing (no barometer/GPS today).
3. **Built-in parser failsafe flag is dead code.** `rcFailsafeFlag` is only written in the CRSFforArduino callbacks (528–540). With the default `CRSF_LIB 0` it is always false. The RC-channels frame has no failsafe bit, so this is expected; the timeout is the real detector. Make the receiver configuration part of the pre-flight checklist.
4. **`linkLQ != 0` can cause a false failsafe.** If link statistics are lost but RC frames are still valid, `linkLQ` may stay at 0 or -1. With the built-in parser, `linkLQ` starts at -1 (OK). If a link-stats frame with LQ 0 arrives while RC frames are still good, the aircraft enters failsafe. This is conservative and acceptable, but note it.
5. **No distinction between "receiver unplugged" and "radio off".** The UART watchdog restarts the UART only while disarmed (597–602). While armed, silence → failsafe. Good.

---

## 4. Attitude estimation and boot calibration

**What is done well**
- Quaternion attitude, no gimbal lock at 90° (416, 879–912).
- Mahony complementary filter with accel-trust weighting (1292–1310).
- Two-pose mount calibration removes the need for a fixed board orientation (1329–1376).
- Glitch rejection on impossible accel/gyro values (941–942, 1033–1051).

**Issues**
1. **Calibration pose tolerance is too loose.** `CAL_MIN_ANGLE_DEG` is 60° (266). As long as the vertical pose is tilted only in the pitch plane, the orthogonalisation recovers the nose direction; but **roll error during either pose directly skews the mount matrix**. A 30° roll during the "vertical" pose produces a mount that is 30° off, and every attitude target is then wrong. Raise the requirement to 80–85° and add a warning if the captured angle is < 85°.
2. **No nose-up/nose-down sign check.** The code assumes the vertical pose has the nose pointing **up**: `bx = upV - (upV·bz)bz` (1344). If the user stands the aircraft on its nose (nose down), `upV = -nose`, and the body X axis is reversed. The controller would then command pitch in the wrong direction. The only safeguard is the written procedure. A robust fix is to integrate the gyro during the flat→vertical rotation and verify the rotation sign about the wing axis; a simpler fix is a third pose or a magnetometer reference.
3. **No flat-pose wings-level check.** `bz = -upF` (1342). If the aircraft is rolled on the bench during the flat pose, `bz` tilts, and the vertical pose is orthogonalised against the wrong belly axis. The procedure says "wings level"; make that visually explicit and consider a roll-error estimate from the BNO085's gravity vector.
4. **Gyro bias is captured only at boot.** The BNO085 calibrated-gyro report already has internal bias correction; subtracting an additional boot bias is reasonable, but if the sensor re-calibrates in flight the extra bias can become wrong. It is small, but worth a comment.
5. **No magnetometer / heading reference.** Heading is relative and will drift slowly. `HOVER_HDG_SLACK_DEG` (25°) limits the target runaway, which is good. For long hover flights, enable the BNO085 magnetometer or its rotation-vector report to bound drift (later improvement, not now).
6. **`ahrsTrust` is not updated when `|accel| <= 0.1`.** It retains the previous value. Cosmetic for logging; initialize it to 0 and set it explicitly.
7. **Attitude is re-derived several times per loop.** `qToMat(qAtt)` is called in `ahrsUpdate`, `noseAboveHorizonDeg`, `bankDeg`, `headingRad`, `tiltFromVerticalDeg`, and logging. Compute `Mat3 R` once per control tick and pass it to the readouts. This is a clean efficiency win.

---

## 5. Control loop, PID and mixing

**What is done well**
- Body-frame quaternion error → rate setpoint → rate PID → mixer (2186–2226). Conventions are internally consistent: body X = elevon differential, body Y = elevon collective, body Z = differential thrust.
- Gains and rate limits are blended by `tau` (2200–2211).
- Integrator clamp and D-term low-pass (398–399, 1382–1389).
- `maxRate` limits the attitude loop output.

**Issues**
1. **CRITICAL — mixer can stop a motor. — CLOSED (v23, verified and hardened v25; see §19).** Original v22 code (2219–2226), kept for history:
   ```cpp
   float d = u[2] * DIFF_THRUST_MAX;
   float base = thr;
   if (landed) d = 0;
   if (base + fabsf(d) > 1.0f) base = 1.0f - fabsf(d);
   if (base > AIRMODE_MIN_THR && base - fabsf(d) < 0) base = fabsf(d);
   mL = motorCmd(base + d);
   mR = motorCmd(base - d);
   ```
   At `base = 0.3`, `|d| = 0.3`, the second `if` sets `base = 0.3`, giving `mR = 0`. At `base < 0.15`, the condition is skipped and `mR` goes negative → clamped to motor stop. Either way one motor can stop in hover, which is exactly where differential thrust is the yaw/side-lean control. Replace with a mixer that **scales `d` to keep both motors inside a floor/ceiling**, e.g.:
   ```cpp
   float d = u[2] * DIFF_THRUST_MAX;
   float base = thr;
   if (landed) d = 0;
   const float floor = 0.02f;               // or a configurable minimum
   float room = fminf(1.0f - base, base - floor);
   if (room < 0.0f) room = 0.0f;
   d = constrain(d, -room, room);
   mL = motorCmd(base + d);
   mR = motorCmd(base - d);
   ```
   This trades yaw authority for keeping both motors alive; that is the safer failure mode. If you want to preserve authority, raise `base` toward `|d| + floor` **and** cap at 1, but never let the low motor hit zero in hover.

   **Status:** the current code uses exactly this scaling (`room = min(1 - base, base - AIRMODE_MIN_THR)`). The v25 sweep confirms the low motor never goes below the floor when the collective is at/above it, and that the differential is removed below it. See §19.
2. **HIGH — `landed` zeroes differential thrust during a descent.** `landed = (tau == 0 && thr < LANDED_THR)` (2196). Any low-throttle hover descent is "landed": `d = 0` (2222) and PIDs reset (2206). Keep yaw authority, or debounce `landed` over ~1 s and require the attitude to be near vertical. At minimum, do not zero `d`; just freeze the integrators.
3. **MEDIUM — `dt` clamp hides long stalls.** `if (dt > 0.05f) dt = 1/IMU_RATE_HZ` (2088). If the real gap is 200 ms (I2C stall), the D term uses 5 ms and a large gyro delta → spike. Add:
   ```cpp
   if (dt > 2.0f / IMU_RATE_HZ) { for (int i=0;i<3;i++) pid[i].reset(); dt = 1.0f / IMU_RATE_HZ; }
   ```
   This is error handling and does not change normal tuning.
4. **MEDIUM — no transition airborne interlock.** `thrStk >= 0.30` (2144) can be true on the ground. Add a simple interlock: require armed + hover + throttle above the threshold for N ms, and optionally a minimum climb/tilt condition. Without an altitude sensor, a time/throttle interlock is the practical minimum.
5. **MEDIUM — no crash detection in `ST_TO_FWD`, `ST_FORWARD`, `ST_TO_HOVER`.** Extend the detector: in hover use tilt > 70°; in forward use a large attitude error vs target (e.g. > 60° for 1 s) or a gyro/accel signature. At minimum run the hover check while `tau` is decreasing (`ST_TO_HOVER`, `ST_FAILSAFE`).
6. **LOW — no throttle slew limit.** Abrupt stick changes can cause pitch/roll transients, especially at the hover↔forward blend. A first-order slew on `thr` (e.g. 2–4 per second) is cheap and smooths the transition.
7. **LOW — no motor/prop direction verification in code.** `MOTOR TEST` exists (HW_TEST), but the flight code cannot detect reversed motor or prop direction. Keep the MOTOR TEST procedure in the pre-flight checklist.

---

## 6. IMU driver / I2C

**What is done well**
- Direct SHTP/SH-2 driver avoids the slow `myCodeCell.Run()` path (944–1125).
- 200 Hz calibrated gyro, 100 Hz accel; control loop runs on fresh gyro reports.
- Report sanity checks (`IMU_MAX_DPS`, `IMU_MAX_ACC`) and a glitch counter (941–942, 1033–1051).
- Sensor-reset detection re-enables reports (1115–1120).
- IMU-loss while armed disarms after 200 ms (2079–2084).

**Issues**
1. **MEDIUM — no I2C timeout.** `Wire.begin()` / `Wire.setClock(400000)` only (1080–1081). The ESP32 Arduino Wire default timeout is typically 50 ms; a stuck bus blocks the loop. Add `Wire.setTimeOut(10);` (or 5 ms) after `begin`, and add a bus-recovery counter:
   - If `bnoI2cErrors` rises rapidly, `Wire.end(); Wire.begin(...);` and re-enable reports.
   - This is exactly the kind of error handling you asked for.
2. **MEDIUM — I2C polling is expensive.** `BNO_POLL_US` is 400 µs (956), i.e. 2500 I2C header reads/s. A 4-byte read at 400 kHz costs ~90 µs, so roughly 20% of the CPU is spent polling. The control loop only needs 200 Hz; 1000 µs polling still catches every report with ≤1 ms latency. Raise `BNO_POLL_US` to 1000 (or use the BNO085 INT pin if the CodeCell exposes it).
3. **MEDIUM — `bnoParse()` stops on an unknown report ID.** `default: return;` (1029) discards the rest of the packet. Only gyro/accel/timestamps are enabled today, so this is unlikely, but a firmware change or an unexpected report silently loses data. Skip the rest of the packet only after logging the ID, or use a safe fallback length if the ID is known.
4. **LOW — `shtpRx` silently truncates above 512 bytes.** Sensor reports are small; control responses are not parsed. Fine today, but document the limit.
5. **LOW — `bnoDrain()` can starve ESC/radio servicing** while packets are flowing (1074–1077). Add `idleService()` every N packets, not only on the empty path.
6. **LOW — no product-ID / WHOAMI check.** A different device at 0x4A would be treated as a BNO085. Optional robustness.
7. **LOW — soft reset every boot discards the BNO085's dynamic calibration.** The boot gyro bias compensates, but the accel/gyro calibration is restarted each power-up. If the BNO085 supports saving calibration, consider it later; not a code-cleanliness issue.
8. **LOW — `IMU_DRIVER 0` calls `myCodeCell.Init()` again** (1131) and `setup()` calls `outputsBegin()` twice (2010, 2024). It works, but the first output start is wasted and can glitch servos/ESCs. Reorder so outputs start once, after the final `Init()`.

---

## 7. ESC / servo outputs

**What is done well**
- Analog protocols use the LEDC hardware, which keeps generating pulses even if the CPU stalls.
- DShot packet/CRC/bit order look correct (708–713, 718–767).
- RMT DShot has a software fallback (728–746, 831–844).
- ESC signal pins are held LOW until the receiver links in ESC_CAL mode (1940–1943).
- Compile-time checks catch empty/invalid protocol values (283–322).

**Issues**
1. **MEDIUM — no check on `ledcAttach` / `ledcDetach` return values.** On core 3.x these return `bool`. A failed attach leaves a servo or ESC silent. Check and log/LED-fault.
2. **LOW — `escBegin()` is not idempotent for analog.** `escAnalogBegin()` re-attaches the motor pins without detaching first (805–812). In `IMU_DRIVER 0` it is called twice. Detach first, as `outputsBegin()` does for servos.
3. **LOW — software DShot disables interrupts for ~53 µs per frame** at DShot300 (749–769). At 200 Hz that is ~1% of the time with IRQs off, and it can delay CRSF/I2C. Prefer RMT (default for `ESC_PROTOCOL 1`) and treat software DShot as a bench/debug mode.
4. **LOW — `escModeName()` returns a pointer to a static buffer** (772–788). Safe for one call per `printf`, but not reentrant. Return a `const char*` literal or pass a buffer.
5. **LOW — no ESC arming/ready feedback.** DShot telemetry bit is always 0 (710). Fine for now; note it if you later want RPM/voltage.
6. **LOW — analog ESC range assumes 1000–2000 µs endpoints.** The header documents recalibration; keep that in the checklist.

---

## 8. Flight log

**What is done well**
- Raw partition logging, no filesystem, pre-erased free space, checksummed 64-byte records (1422–1603).
- Session numbering, event records, `list`/`dump`/`erase` commands.
- Records are packed and `static_assert`ed to 64 bytes (454).

**Issues**
1. **MEDIUM — flash write inside the control loop.** `logSample()` is called at the end of `loop()` (2232–2234) and `logWrite()` calls `esp_partition_write()` (1548–1556). On the C3 the cache is disabled during the write, so the CPU stalls for a few hundred microseconds. This is usually tolerable, but it adds jitter to the 200 Hz loop. Options, in order of effort:
   - Measure it: wrap the write in `micros()` and record the max in debug.
   - Buffer 4–8 records in RAM and write 256–512 bytes at once (fewer transactions).
   - Lower `LOG_HZ` to 25 while armed.
   - Do not remove logging; the data is valuable.
2. **LOW — first-record corruption erases the whole log.** `logBegin()` formats the partition if record 0 is neither valid nor empty (1500–1506). A power loss during the first write loses every previous session. Scan for any valid record in the first few KB before deciding it is "foreign data", or add a separate log header sector.
3. **LOW — `logFill()` returns 1.0 when there is no partition** (1470). With `LOG_ENABLE 1` and no SPIFFS partition, the LED shows slow yellow "log full" forever (2075) even though logging is unavailable. Return 0.0 or gate on `logPart`.
4. **LOW — three separate 4 KB static buffers.** `logBegin()` (1499), `logList()` (1617) and `logDump()` (1653) each declare `static uint8_t buf[LOG_REC_SIZE * 64]`. Share one global buffer to save 8 KB RAM.
5. **LOW — `esp_partition_read/erase_range` return values are not checked.** `esp_partition_write` is checked. Check erase/read and disable logging after repeated failures.
6. **LOW — stale comment.** Line 1432 says "Records 48-byte binary samples"; the record is 64 bytes (line 118 correctly says 64). Update.
7. **LOW — `logEvent(EV_ARM)` increments the session even if the log is not ready/full.** Harmless, but session numbers can advance without records.

---

## 9. LED, debug and status

**What is done well**
- Clear LED state machine for calibration, armed, disarmed, log full, ESC cal (419–489).
- Debug output is rate-limited and change-filtered (2041–2068).
- Receiver diagnostic (`RX_TEST`) and hardware test (`HW_TEST`) modes are genuinely useful.

**Issues**
1. **LOW — `LED_LOG_BUSY` is not in the slow-pattern list** (486), so it blinks fast yellow while erasing. The header says fast yellow, so this is intentional; just confirm.
2. **LOW — no LED/status indication for an ESC-attach or DShot-RMT fallback failure.** A failed RMT allocation only prints when `DEBUG 1` (833). Consider a distinct LED fault pattern.
3. **LOW — `debugReport()` calls the attitude readouts again**, each doing its own `qToMat()` (2051). Cache the matrix per tick (see §4.7).
4. **LOW — `stateName[state]` / `logEventName[event]` are indexed without bounds checks** in debug paths (2064, 1628). The values are enum-controlled today; a guard costs nothing.

---

## 10. Efficiency and cleanliness (no behaviour change)

1. Make `G_HOVER` and `G_FWD` `static const` (383–397) so they live in flash, not RAM.
2. Make `stateName` and `logEventName` `static const char* const` (1418, 1446).
3. Cache `Mat3 R = qToMat(qAtt)` once per control tick and pass it to `noseAboveHorizonDeg()`, `bankDeg()`, `headingRad()` and `tiltFromVerticalDeg()`. This removes 4–8 matrix builds per loop.
4. Share the 4 KB log buffer between `logBegin`, `logList` and `logDump`.
5. Guard the `memcmp(rcPrevUs, ...)` / `rcChangedFrames` bookkeeping with `#if DEBUG` (533, 576) — it is only used by the debug/status paths.
6. Raise `BNO_POLL_US` from 400 to 1000 (see §6.2).
7. Factor the armed control block out of `loop()` into `controlUpdate()`; `loop()` is ~170 lines and mixes state machine, attitude, mixing and logging.
8. Factor the log partition scan into one helper used by `logBegin`, `logList` and `logDump`; the three scans are similar.
9. Add a short comment to the `#if (1 - X - 1) == 2` empty-value checks (285–293, 311–313); the trick is clever but obscure.
10. Add compile-time checks that only one of `RX_TEST` / `HW_TEST` / `IMU_TEST` is 1. Today the first `for(;;)` wins silently.
11. Keep the single-file sketch; the section structure is good. Splitting into `.h/.cpp` would make the Arduino build more fragile without a clear benefit.

---

## 11. Companion-board connectivity — current state

**Short answer:** the code leaves a **partial** opening, but it is **not ready** for a companion board today. There is no companion protocol, no arbitration, no watchdog and no telemetry stream in the flight code. The structure is clean enough to add one with modest work.

**What exists now**

| Interface | Pins | Current use | Free for companion? |
|---|---|---|---|
| CRSF UART | IO1 RX, IO2 TX | Receiver in. **IO2 is never transmitted on** by this code. | IO2 is effectively free (one pin). |
| I2C | IO8 SDA, IO9 SCL | BNO085 at 0x4A. The CodeCell exposes I2C. | **Yes, shared bus** — companion can be an I2C slave. |
| USB-C / `Serial` | USB CDC | Debug + `list`/`dump`/`erase` text commands, **disarmed only**. | Yes, but USB is device-only and the cable is a flight-reliability risk. |
| 6 GPIO | IO1,2,3,5,6,7 | 1=CRSF RX, 2=CRSF TX (unused), 3=motor R, 5=motor L, 6=servo L, 7=servo R | All allocated; only IO2 can be repurposed. |
| Wi-Fi / BLE | internal | Unused. | Possible, but 2.4 GHz coexistence and single-core jitter are concerns. |

**What is missing for a safe companion link**
- A binary command/telemetry protocol with magic, version, sequence, CRC and timestamp.
- A command watchdog (e.g. 100 ms) and a fallback to RC when the companion is silent.
- Authority arbitration: the companion must not arm/disarm and must not command raw motors/servos.
- Bounds checks on every setpoint (NaN/inf, max tilt/bank/pitch/yaw rate, throttle range).
- A pilot-controlled enable switch (CH7 is available in `rcUs`; `MAX_CH` is 16).
- Logging of companion commands, rejections and timeouts.
- I2C bus-sharing rules with the BNO085 (the IMU is safety-critical).

---

## 12. Companion-board options

### Option A — I2C companion on the existing bus (recommended)

- **Hardware:** a companion MCU (ESP32-S3, RP2040, STM32, Pico, etc.) as an **I2C slave** at a distinct address (e.g. 0x42), sharing SDA/SCL with the BNO085. The CodeCell C3 remains the I2C **master** (it already is for the IMU).
- **Pros:** no pin changes; low latency; simple wiring; the C3 stays in control; the companion can be powered from a separate BEC with a common ground.
- **Cons:** shares the bus with the IMU. A companion that stretches the clock or holds SDA can disturb the IMU. Mitigate with `Wire.setTimeOut()`, a bus-recovery counter, and a companion that is a well-behaved slave.
- **Data flow:** C3 polls the companion at 50–100 Hz for a fixed-size command struct; C3 writes a telemetry struct back. Keep transfers small (≤ 32 bytes each way).
- **Best use:** the companion is an **outer-loop advisor** (heading, forward speed, altitude/vertical-speed, mode request). The C3's inner attitude/rate loop and failsafe remain authoritative.

### Option B — UART on IO2 (CRSF TX pin)

- IO2 is free because the code never sends telemetry to the receiver.
- **Problem:** you need both TX and RX for a bidirectional companion link, and there is no second free pin. Options:
  - Half-duplex single-wire UART on IO2 with external diode/resistor (RS-485-style). The Arduino `HardwareSerial` API does not expose this cleanly; you would use the ESP-IDF UART driver.
  - Use IO2 only for C3→companion telemetry and use I2C for companion→C3 commands (hybrid).
  - Give up CRSF telemetry and use IO2 as the companion RX, with the companion sending commands only (no C3 telemetry). The companion would be blind unless it also reads I2C.
- **Verdict:** feasible but more work and less clean than I2C.

### Option C — USB CDC companion

- The C3's USB is **device-only**; the companion must be a USB host (Linux, ESP32-S3 host, etc.).
- The existing `logCommandPoll()` text protocol is disarmed-only (1702). You would add a binary protocol, multiplexed with the text commands, with the same watchdog/authority rules.
- **Pros:** no pin changes; high bandwidth; easy development on a Linux companion.
- **Cons:** USB cable in flight (vibration/connector reliability), shared with programming/logging, not ideal for safety-critical control. Use for development/telemetry, not primary command.

### Option D — Companion in-line with the CRSF receiver

- Receiver TX → companion RX; companion TX → C3 CRSF RX. The companion parses CRSF and forwards (optionally modified) channels.
- **Pros:** zero changes to the C3 flight code; the C3 sees normal RC.
- **Cons:** the companion becomes a single point of failure; if it hangs, the C3 loses RC. Only acceptable with a hardware bypass/mux and a robust companion. Not recommended for a first version.

### Option E — Wi-Fi / BLE / ESP-NOW

- The C3 has Wi-Fi 4 + BLE 5. A companion ESP32 could use ESP-NOW, or a ground station could use Wi-Fi/BLE.
- **Pros:** no wires.
- **Cons:** single-core C3, Wi-Fi stack preemption, latency, power, and **2.4 GHz coexistence with ELRS** if your ELRS link is 2.4 GHz. Not recommended for the inner loop or safety-critical commands. Could be used for low-rate telemetry/parameters with the C3 still flying on RC.

### Comparison

| Option | Pins needed | Latency | Safety | Effort | Recommendation |
|---|---|---|---|---|---|
| A. I2C slave | 2 (shared I2C) | ~1 ms | High if watchdog/bounds added | Low–medium | **Recommended** |
| B. UART on IO2 | 1–2 | ~1 ms | High, but pin-limited | Medium–high | Only with half-duplex or hybrid |
| C. USB CDC | 0 | ~1 ms | Medium (connector) | Medium | Development/telemetry |
| D. CRSF in-line | 0 (rewire) | ~1 ms | Low (single point of failure) | Low | Not for first version |
| E. Wi-Fi/BLE | 0 | 10–100 ms | Low (RF/jitter) | Medium | Telemetry only |

---

## 13. What it would take to add Option A (concrete)

**Hardware**
1. Confirm the CodeCell's I2C connector exposes SDA (IO8), SCL (IO9), 3.3 V and GND. The product spec lists "6 GPIO + I2C"; verify the connector pinout before soldering.
2. Use a 3.3 V companion (or level-shift). Power it from a separate BEC; tie grounds together.
3. Keep the BNO085 at 0x4A; give the companion a different 7-bit address (e.g. 0x42).

**Firmware structure**
1. Add a compile-time flag, default off:
   ```cpp
   #define COMPANION 0        // 1 = enable I2C companion link
   #define COMP_ADDR 0x42
   #define COMP_TIMEOUT_MS 100
   #define CH_COMPANION 6     // CH7 = companion enable (pilot kill switch)
   ```
2. Define packed, versioned structs with a CRC:
   ```cpp
   struct __attribute__((packed)) CompCmd {
     uint16_t magic; uint8_t ver, seq;
     uint32_t tMs;
     float vN, vE, vD;      // desired velocity, m/s (or 0 if unused)
     float headingRad;      // desired heading
     float altM;            // desired altitude (if a sensor exists)
     uint8_t mode;          // 0=hold, 1=hover, 2=forward
     uint8_t flags;
     uint16_t crc;
   };
   struct __attribute__((packed)) CompTelem {
     uint16_t magic; uint8_t ver, seq;
     uint32_t tMs;
     float qw, qx, qy, qz;
     float rateX, rateY, rateZ;
     float noseDeg, bankDeg, hdgDeg;
     float thr, tau;
     uint8_t state, armed, linkOk;
     int8_t lq;
     uint16_t crc;
   };
   ```
3. Add `companionBegin()` (nothing beyond `Wire.setTimeOut(10)`) and `companionPoll()` called at 50–100 Hz from `loop()` — **not** every 200 Hz tick, to keep bus load low.
4. `companionPoll()`:
   - Read `CompCmd` with `Wire.beginTransmission(COMP_ADDR); Wire.write(0); Wire.endTransmission(false); Wire.requestFrom(COMP_ADDR, sizeof(CompCmd));`.
   - Validate magic, version, CRC, `seq` monotonicity and timestamp freshness.
   - Reject NaN/inf and clamp every setpoint to the same limits used for the sticks (`HOVER_MAX_TILT_DEG`, `FWD_MAX_BANK_DEG`, `FWD_MAX_PITCH_DEG`, `FWD_YAW_RATE_DPS`, throttle range).
   - On success: `compLastMs = millis(); compOk = true;`.
   - Write `CompTelem` back to the companion.
5. `companionOk()` = `COMPANION && compOk && (millis() - compLastMs < COMP_TIMEOUT_MS) && rcUs[CH_COMPANION] > ARM_SWITCH_US`.
6. Integrate:
   - If `armed && companionOk()`: blend the companion's high-level setpoints into the target attitude/throttle, **within the same clamps as the sticks**. Do not bypass the inner loop.
   - If `!companionOk()`: fall back to RC immediately; log a `COMP_TIMEOUT` event once.
   - The companion can request a mode change, but the C3 applies the same preconditions as the CH6 switch (armed, airborne interlock, attitude).
   - The companion **cannot** arm, disarm, or write motor/servo commands.
7. Add `COMPANION` and `compOk` to the flight log and debug output.
8. Keep the companion code behind `#if COMPANION` so the default build is byte-for-byte the current behaviour.

**Safety rules (non-negotiable)**
- RC CH5 remains the only arm/disarm authority.
- CH7 (or another spare channel) is a pilot kill switch for companion authority.
- Companion commands are **setpoints**, never actuator commands.
- A silent, invalid, out-of-range or stale companion is ignored, not obeyed.
- The C3's failsafe, crash detection, IMU-loss disarm and motor mixer stay in the C3.
- Log every accepted command, every rejection reason, and every timeout.

**Testing checklist**
- Companion absent at boot → normal flight, no errors.
- Companion disconnected in flight → immediate fallback to RC/hover, one log event.
- Companion sends bad CRC/garbage → ignored, counter increments.
- Companion sends out-of-range/NaN → clamped/rejected, no attitude jump.
- Companion sends commands while disarmed → no arming, no motor output.
- CH7 low → companion commands ignored even if valid.
- Measure the control-loop rate and IMU rate with companion polling enabled; compare with `COMPANION 0`.
- Power-cycle/brownout the companion in flight; the C3 must be unaffected.
- I2C bus contention: run the IMU test and confirm 200 Hz gyro / 100 Hz accel with the companion connected.

---

## 14. Suggested patch order

**Before the next flight (small, low-risk error handling)**
1. Fix the differential-thrust mixer so neither motor stops in hover (§5.1). **Done (v23; verified and hardened v25 — §19).**
2. Use `TRANS_BACK_THR_MIN` during the failsafe pitch-back, then `FS_THR` (§3.1).
3. Reset `crashStartMs` in `disarm()` and on arm (§2.3); extend crash detection to `ST_TO_HOVER`/`ST_FAILSAFE` (§5.5).
4. Do not zero differential thrust when `landed`; only freeze integrators (§5.2).
5. Add `Wire.setTimeOut(10)` and a bus-recovery counter (§6.1).
6. Reset PIDs when `dt` is far above nominal (§5.3).
7. Raise `CAL_MIN_ANGLE_DEG` to 80–85 and add a calibration warning (§4.1).
8. Configure and bench-test the ELRS failsafe as "no pulses" (§2.5, §3.3).

**Next (efficiency / cleanliness)**
9. Cache `qToMat` per tick; make gains/name tables `const`; share the log buffer (§10).
10. Raise `BNO_POLL_US` to 1000 (§6.2).
11. Check `ledcAttach`/`ledcDetach` returns; make `escBegin()` idempotent (§7.1–7.2).
12. Fix `logFill()` when there is no partition; fix the first-record format decision; update stale comments (§8).
13. Factor `controlUpdate()` and the log scan helper (§10.7–10.8).

**Later (features, not part of this pass)**
14. Companion link (Option A) behind `#if COMPANION`.
15. Magnetometer/rotation-vector heading reference.
16. Battery-voltage monitoring and low-voltage failsafe (if the CodeCell exposes it).
17. Barometer/rangefinder for a true airborne interlock and landing detection.

---

## 15. Bench-test checklist (props off)

1. `RX_TEST 1`: confirm CRSF frames and channel directions.
2. `HW_TEST 1`: confirm elevon directions, motor directions, and MOTOR TEST behaviour.
3. `ESC_CAL 1` (analog only): confirm throttle endpoints; then set back to 0.
4. Boot calibration (first boot or `cal force`): verify the LED sequence, the printed mount vectors, and that the estimated nose is ~90° at the vertical pose. Then power-cycle and confirm the saved calibration is loaded (no CYAN/MAGENTA sequence) and `cal status` shows the same mount.
5. Arm on the bench: verify the CH5 low→high cycle, throttle-low refusal, CH6-vertical refusal, tilt refusal, **moving-aircraft refusal** (flick CH5 while rotating the airframe), and **untrusted-attitude refusal** (if reproducible).
6. Failsafe test: configure the ELRS receiver failsafe to **no pulses** first, then radio off with props off; verify the aircraft enters failsafe, `tau` returns to hover, and the 15 s timeout disarms.
7. Log test: arm briefly, disarm, `list`, `dump`, `erase`; confirm records and checksums.
8. With the companion connected (if implemented): repeat the disconnect/garbage/out-of-range tests.
9. Forward-transition interlock: props off, CH6 vertical, arm, then flip CH6 forward with the throttle low and confirm `tau` stays 0. Raise the throttle above 0.30 and hold; confirm the transition starts only after ~0.5 s. Flip CH6 back and confirm a pending start is cancelled. Confirm no transition can start within ~1 s of arming.

---

## 16. Bottom line

The core architecture is sound: body-frame quaternion control, blended hover/forward gains, a custom 200 Hz IMU path, CRSF input, analog/DShot outputs, and a useful flight log. The main risks are not in the math but in the **edge cases**: the differential-thrust mixer at low throttle, the failsafe pitch-back throttle, calibration tolerance, crash detection outside hover, and I2C/flash stalls. Those are all fixable without changing the flight model.

The code does **not** currently provide a companion-board interface. The best path is an **I2C slave companion** on the existing bus, with the C3 remaining the flight controller, a CH7 pilot enable switch, a strict command watchdog, CRC/bounds checks, and a fallback to RC. IO2 is the only free GPIO if you prefer UART, but I2C is the cleaner fit for this hardware.

---

## 17. Patch batch 1 applied (v23, 2026-11-09)

The following low-risk fixes from §14 were applied to `TailSitter.ino`. The
flight model, gains, transition timing and companion interface were not changed.
The sketch compiles for `esp32:esp32:esp32c3` with `DEBUG 0`, `DEBUG 1`, and
`IMU_DRIVER 0/1`.

| # | Fix | Where |
|---|---|---|
| 1 | Mixer now scales differential thrust so neither motor drops below `AIRMODE_MIN_THR`; average thrust is unchanged. | `loop()` motor mixer |
| 2 | Failsafe pitch-back uses `TRANS_BACK_THR_MIN` while `tau > 0`, then `FS_THR` in hover. | `loop()` throttle |
| 3 | `crashStartMs` is reset on arm and disarm; crash detection now covers transition/forward/failsafe via attitude error (`CRASH_ATT_ERR_DEG`), while hover keeps the 70° tilt check. | `disarm()`, arm block, state machine |
| 4 | `landed` no longer zeroes differential thrust or resets the PIDs; it only freezes integrators. | `loop()` PID/mixer |
| 5 | `Wire.setTimeOut(10)` added; a burst of I2C errors re-initialises the bus and increments `imuBusRecoveries`. | `imuBegin()`, `imuService()`, `imuBusRecover()` |
| 6 | PID reset when `dt > 2 / IMU_RATE_HZ` to prevent a derivative spike after a stall. | `loop()` dt handling |
| 7 | `CAL_MIN_ANGLE_DEG` raised 60° → 80°; a warning is printed if the captured vertical pose is < 85°. | boot calibration |
| 8 | ELRS failsafe is documented in the sketch as **no pulses**; the receiver configuration remains a pre-flight checklist item. | CRSF defines, §15.6 |

Still open from the review (not part of this batch): the forward-transition
airborne interlock, CRSF `rcFailsafeFlag` support, log-flash writes inside the
control loop, `qToMat` caching, and the companion link. The companion design in
§13 is unchanged.

---

## 18. Persistent boot calibration (v24, 2026-11-16)

The two-pose calibration result is now stored in NVS and reused on later boots.
This removes the field requirement for a known flat and known vertical pose.

**What is stored**
- Sensor→body mount matrix `MNT` (nose, right wing, belly rows).
- Body-frame gyro bias.
- A versioned, CRC-checked `CalBlob` in NVS namespace `tailcal`, key `cal`.
  The record also carries a size field and a right-handed orthonormal-basis
  sanity check, so a truncated, old or corrupt write is rejected.

**Boot behaviour**
1. `CAL_FORCE 1` or a one-shot `cal force` flag → run the two poses, save,
   clear the one-shot flag.
2. Valid saved blob → load `MNT` + gyro bias, initialise `qAtt` from gravity
   (`qFromTwoVectors`), then refine with the AHRS for ~0.5 s. The aircraft may
   be held in **any** attitude; no flat/vertical reference is needed.
3. No valid blob → run the two poses, save.
   If the NVS save fails, the calibration still applies for that power-up and
   a warning is printed.

**Serial commands (disarmed; serial commands require `LOG_ENABLE 1`)**
- `cal` — run the two-pose calibration immediately and save.
- `cal status` — show the saved mount vectors and gyro bias.
- `cal force` — set the one-shot flag; the next boot calibrates.
- `cal erase` — delete the saved calibration and force flag; the next boot calibrates.

**Safety notes**
- Recalibrate after changing the IMU, its mounting, or the airframe.
- The saved gyro bias is a bench-temperature value; the BNO085's internal
  calibration handles normal temperature drift.
- Field boot still requires the aircraft to be stationary while `qAtt` is
  initialised from gravity, but it may be held in any attitude.
- Props off for `cal`, `cal force`, and all bench testing.

Compiles for `esp32:esp32:esp32c3` with `DEBUG 0/1`, `IMU_DRIVER 0/1`,
`LOG_ENABLE 0/1`, and `CAL_FORCE 0/1`.

---

## 19. CRITICAL item closure — differential-thrust mixer (v25, 2026-11-16)

**Finding (§1.1, §5.1):** the mixer could drive one motor to zero at low hover
throttle, removing yaw / side-lean authority in exactly the regime where it is
needed.

**Root cause (v22):** two independent `if` guards:

```cpp
if (base + fabsf(d) > 1.0f) base = 1.0f - fabsf(d);
if (base > AIRMODE_MIN_THR && base - fabsf(d) < 0) base = fabsf(d);
mL = motorCmd(base + d);   // one of these could be 0
mR = motorCmd(base - d);
```

At `base = 0.30`, `|d| = 0.30` the low motor was commanded to `0.00` — armed
idle in command space — giving a `0.60 / 0.00` split. Below `AIRMODE_MIN_THR`
the guard was skipped, the low motor went negative and `motorCmd()` clamped it
to the same idle value. Either way the differential collapsed and one motor sat
at idle while the other carried the collective.

**Fix (v23, retained in v25):** clip the differential to the room between the
collective and both rails:

```cpp
float room = fminf(1.0f - base, base - AIRMODE_MIN_THR);
if (room < 0.0f) room = 0.0f;
d = constrain(d, -room, room);
mL = motorCmd(base + d);
mR = motorCmd(base - d);
```

Result:
- `base >= AIRMODE_MIN_THR` → both motors stay in `[AIRMODE_MIN_THR, 1]`; the
  low motor cannot fall below the floor.
- `base < AIRMODE_MIN_THR` → `room = 0`, so `d = 0`; the motors stay equal and
  the differential is removed rather than one motor dropping toward idle.
- The average thrust is unchanged (the collective is not raised).

**Supporting fixes already in place (from batch 1):**
- HIGH §5.2 — `landed` no longer zeroes `d`; it only freezes the integrators.
  This removes the other path that silently deleted yaw authority in a descent.
- MEDIUM §5.3 — a long `dt` resets the PIDs instead of producing a derivative
  spike that could saturate `u[2]` and demand a large differential.

**Hardening added in v25:**
- `motorCmd()` rejects non-finite values and commands idle, so no call site can
  send a NaN/Inf-derived value to an ESC.
- The mixer rejects non-finite `base` / `d` and removes the differential.
- Compile-time checks that `AIRMODE_MIN_THR` and `DIFF_THRUST_MAX` are sane.

**Verification:** a host-side sweep mirrored the exact mixer and both
`motorCmd` mappings over `base ∈ [0,1]` and `dReq ∈ [-1,1]` (4,004,001 pairs):

| Case | Pairs | Result |
|---|---|---|
| `base >= floor` | 3,403,701 | low-motor minimum = **0.150000** (the floor); failures = 0 |
| `base < floor` | 600,300 | differential exactly 0 (max asymmetry = 0.000000) |
| all pairs | 4,004,001 | no command outside `[0,1]`; high-motor max = 1.0 |
| v22 mixer, same sweep | — | low motor reached **-0.300000** (clamped to idle by `motorCmd`) |
| review case `base=0.30, dReq=1.0` | — | v22 = `(0.60, 0.00)`; v25 = `(0.45, 0.15)` |
| NaN base / NaN dReq | — | `(0.00, 0.00)` / `(0.50, 0.50)` — no asymmetry |

Command-space check: analog `[1080, 2000] µs`, DShot `[100, 2047]`; no value
outside the armed idle-to-max range.

Compiles for `esp32:esp32:esp32c3` with `DEBUG 0/1`, `IMU_DRIVER 0/1`,
`LOG_ENABLE 0/1`, `CAL_FORCE 0/1`, and `ESC_PROTOCOL 3/1` (analog and DShot),
no sketch warnings.

**Residual trade-off (accepted):** below the collective floor, yaw authority is
removed rather than raising collective thrust. Raising the collective would
cause an unintended climb or hop, which is worse. The mixer comment documents
this, and the behaviour is symmetric, so it cannot produce a single-motor
asymmetry.

**Bench check (props off):** in `DEBUG 1`, arm the aircraft with the props
removed (throttle low, CH6 vertical), then move the yaw stick at low throttle
and watch `mL`/`mR` in the debug line; confirm the low motor never reaches the
idle value while the collective is at/above the floor, and that the two motors
stay equal below the floor. Repeat in `HW_TEST` before flight.

---

## 20. Arming and transition interlocks (v26, 2026-11-16)

These changes reduce the chance of a ground or bench mistake and of a
forward transition starting before the aircraft is ready. They do not change
the flight model, gains, or transition timing.

### 20.1 Arm interlock

The previous arm checks were throttle, CH6 position, and tilt only. A moving
aircraft, or one with a poor attitude estimate, could still arm.

Added to the arm block in `loop()`:

| Check | Constant | Behaviour |
|---|---|---|
| Gyro stillness | `ARM_MAX_RATE_DPS` 20 °/s, `ARM_STILL_MS` 100 | Arm is refused unless the gyro has stayed below 20 °/s continuously for 100 ms. `armStillSec` resets on any faster motion. |
| Accel trust | `ARM_MIN_TRUST` 0.5 | Arm is refused while `ahrsTrust < 0.5` (accel magnitude far from 1 g, i.e. the attitude estimate is not trustworthy). |

Both refusals print a reason in `DEBUG 1` and leave the arm switch latched low,
so the pilot must release and re-flick CH5. `armStillSec` is capped at 1 s.

The stillness timer is updated right after `ahrsUpdate()` from the
bias-corrected body rate (`vNorm3(gyro) * R2D`), so it uses the same gyro data
as the controller.

### 20.2 Forward-transition interlock

The previous start condition was `tau > 0 || thrStk >= TRANS_START_MIN_THR`,
which could begin the pitch-down immediately on a throttle bump, including on
the ground.

New start logic (`loop()`, CH6 forward branch):

1. A settle delay after arming: `TRANS_ARM_DELAY_MS` (1 s). No transition can
   start within 1 s of arming.
2. The throttle must stay at/above `TRANS_START_MIN_THR` (0.30) for
   `TRANS_START_HOLD_MS` (0.5 s). `fwdStartMs` is the hold timer; it is cleared
   when the hold is broken before the transition starts, when CH6 returns to
   vertical, on disarm, and on entering failsafe.
3. Once `tau > 0`, the transition continues even if the throttle later drops,
   preserving the existing behaviour and the `TRANS_FWD_THR_MIN` floor.

The `tau` rate and the `TRANS_FWD_SEC` / `TRANS_BACK_SEC` timings are unchanged.

### 20.3 Elevon non-finite guard

`elevonUs()` now rejects a non-finite command and substitutes 0 before the
`constrain()` and servo mapping, matching the `motorCmd()` guard added in v25.
A NaN/Inf can no longer reach a servo output.

### 20.4 Verification

Compiles for `esp32:esp32:esp32c3` with `DEBUG 0/1`, `IMU_DRIVER 0/1`,
`LOG_ENABLE 0/1`, `CAL_FORCE 0/1`, and `ESC_PROTOCOL 3/1` (analog and DShot),
no sketch warnings.

Bench checks are in §15 items 5 and 9 (props off).

### 20.5 Residual risk (unchanged by this section)

The two-pose calibration still cannot distinguish a nose-up from a nose-down
vertical pose (see §4, issue 2). Persistent calibration can therefore save and
reuse a mirrored mount. Until a third pose or the magnetometer is added, the
mitigation is the strict calibration procedure and recalibration after any
mount change. This is called out here so it is not lost behind the new
interlocks.

