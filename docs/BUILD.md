# Build and flash

This page takes a stock Microbots CodeCell C3 from "new in the bag" to "running the flight controller", including the bench-test modes and the ESC throttle calibration.

> [!WARNING]
> Remove both propellers before any upload, bench test, calibration or ESC procedure. The sketch can spin motors as soon as it is armed.

## What you need

| Item | Notes |
|---|---|
| Arduino IDE 2.x | The sketch is a single `.ino` file. |
| ESP32 board support | **esp32 by Espressif**, core **3.x** tested. |
| CodeCell library | Library Manager: **CodeCell** by Microbots. |
| USB-C cable | Data cable, not charge-only. |
| Optional: `CRSFforArduino` | Only if you set `CRSF_LIB 1`. The default built-in parser is recommended. |

Do **not** install the SparkFun BNO08x library. The CodeCell library already contains the BNO085 driver files, and adding SparkFun's library causes `multiple definition` link errors.

## Board settings

In **Tools**:

| Setting | Value |
|---|---|
| Board | **ESP32C3 Dev Module** |
| CPU frequency | **160 MHz** |
| Flash size | **4 MB** |
| Partition scheme | **Default 4MB with spiffs** |
| USB CDC On Boot | **Enabled** |
| Core debug level | **None** |
| Upload speed | 921600 (or 460800 if uploads are unreliable) |

The partition scheme is not optional. The flight log writes raw records to the `spiffs` data partition; without it the log reports `Logging unavailable (no partition)`.

## Install the CodeCell library

1. Open **Tools → Manage Libraries**.
2. Search for **CodeCell**.
3. Install the Microbots **CodeCell** library.
4. If you previously installed **SparkFun BNO08x Cortex Based IMU**, uninstall it (or at least do not use it with this sketch).

The sketch uses the CodeCell library for the LED and the I2C bus, and drives the BNO085 directly through its own SHTP/SH-2 driver (`IMU_DRIVER 1`). That direct path gives a 200 Hz gyro loop and 100 Hz accelerometer updates.

## Configuration flags

All flags are at the top of `TailSitter.ino`. Change them, re-upload, and change them back when the test is done.

| Flag | Default | What it does |
|---|---|---|
| `DEBUG` | `0` | `1` prints a status line while disarmed and arm-refusal reasons. Serial 115200. |
| `CRSF_LIB` | `0` | `0` uses the built-in CRSF parser. `1` uses the CRSFforArduino library. |
| `RX_TEST` | `0` | `1` runs a receiver wiring/baud diagnostic only. No flight code, no outputs. |
| `HW_TEST` | `0` | `1` runs receiver + servo + ESC checks only. No calibration, no flight code. |
| `ESC_CAL` | `0` | `1` runs throttle passthrough for analog ESC calibration. Never returns. |
| `IMU_DRIVER` | `1` | `1` built-in direct BNO085 driver. `0` CodeCell library path (about 30 Hz, not recommended). |
| `IMU_TEST` | `0` | `1` prints raw accel/gyro and update rates. No flight code. |
| `LOG_ENABLE` | `1` | `1` enables the onboard flight log and the USB commands. |
| `CAL_FORCE` | `0` | `1` forces the two-pose calibration at every boot. |

Test modes are mutually exclusive by design. `ESC_CAL 1` cannot be combined with `RX_TEST`, `HW_TEST` or `IMU_TEST`; the sketch stops with a clear `#error`. Set the other flags back to `0` first.

## Compile and upload

1. Connect the CodeCell by USB.
2. Select the correct serial port under **Tools → Port**. On macOS it looks like `/dev/cu.usbmodem*`.
3. Click **Verify** first, then **Upload**.
4. Open **Serial Monitor** at **115200** if you set `DEBUG 1` or `LOG_ENABLE 1`.

With the default `DEBUG 0`, the board does not print the per-second status lines. It still prints calibration and log messages when `LOG_ENABLE 1`. That is intentional: the serial transmitter never stalls the control loop when no PC is attached.

### Command-line compile check

If you have `arduino-cli`, a basic compile check is:

```sh
arduino-cli compile --fqbn esp32:esp32:esp32c3 TailSitter.ino
```

That verifies the code compiles. Set the partition scheme and USB CDC options in the Arduino IDE (or through your core's board options) before uploading; the option names vary between ESP32 core versions.

### Verified compile matrix

The sketch has been compiled for `esp32:esp32:esp32c3` with:

| Variation | Result |
|---|---|
| Default | clean |
| `DEBUG 1` | clean |
| `IMU_DRIVER 0` | clean |
| `LOG_ENABLE 0` | clean |
| `CAL_FORCE 1` | clean |
| `ESC_PROTOCOL 1` (DShot) | clean |
| `ESC_PROTOCOL 3` (OneShot125, default) | clean |

## First boot

On the first boot, or after `cal erase`, the board runs the two-pose calibration. The LED is the primary guide:

| LED | Meaning |
|---|---|
| Fast cyan | Waiting for the **flat on belly** pose |
| Solid cyan | Flat pose captured; rotate to vertical |
| Fast magenta | Waiting for the **on tail, nose up** pose |
| Solid magenta | Vertical pose captured; calibration is being saved |
| Green blink | Ready, disarmed |
| Fast red | IMU not found, or ESC calibration waiting for a link |

The full procedure is in [CALIBRATION.md](CALIBRATION.md).

On later boots, the saved calibration is loaded and the aircraft only needs to be **stationary**; it can be held in any attitude. The board initialises attitude from gravity.

## Bench-test modes

Do these with **props off**, in this order.

### 1. `RX_TEST 1` — receiver wiring and baud

Set `RX_TEST 1`, upload, open Serial Monitor at 115200.

The test scans IO1, IO2, IO3, IO5, IO6 and IO7 for activity, then sweeps baud rates and polarity on the pin that shows data. It prints a verdict:

- `CRSF OK on IO1 at 420000 baud` plus CH1–CH6 values: wiring and receiver protocol are correct.
- `data but no valid CRSF frames`: set the receiver's Serial Protocol to **CRSF**.
- `no data on ANY pin`: the receiver is not powered, not bound, or not linked. The radio must be on.

No ESC or servo output is produced in this mode.

### 2. `HW_TEST 1` — servos, motors, directions

Set `HW_TEST 1`, upload, open Serial Monitor.

- Move the sticks: the elevons should follow. Check that roll and pitch move the correct surfaces in the correct direction.
- Check the printed channel values and frame rate.
- **Motor test:** throttle low, then flip CH5 high. Both motors follow the throttle stick directly, with no stabilisation. CH5 low stops them. This is the only place to judge whether the motors start smoothly and in the right direction.

No IMU, no calibration and no flight code runs in this mode.

### 3. `IMU_TEST 1` — raw IMU check

Set `IMU_TEST 1`, upload. The monitor prints accel, gyro and update rates at 5 Hz.

- Gyro should read near zero when still.
- Accel magnitude should be about 1 g.
- The gyro rate should be near 200 Hz and the accel rate near 100 Hz with `IMU_DRIVER 1`.

If the rates are far below that, check `IMU_DRIVER` and the I2C connection.

### 4. `ESC_CAL 1` — analog ESC throttle range

Only for analog protocols: `ESC_PROTOCOL` `0`, `3`, `4` or `5`. DShot (`1`, `2`) is digital and needs no calibration.

> [!WARNING]
> **PROPS OFF.** This mode passes CH3 straight to both ESCs. There is no arming, no stabilisation, no IMU and no servo output.

1. Set `ESC_CAL 1` and the `ESC_PROTOCOL` you will fly with. Upload.
2. Power off. Turn the radio on with the **throttle stick full up**.
3. Power the aircraft. The LED is **fast red** until the receiver links, then **fast white**.
4. The ESCs play their start-up tones, then the maximum-throttle tones.
5. Within about 2 seconds, move the throttle stick **full down**. The ESCs play the minimum-throttle tones.
6. Confirmation tones mean the range is calibrated. Check that the motors follow the stick and start just above the bottom.
7. Power off. Set `ESC_CAL 0` and upload the flight code again.

The ESC signal pins are held low (no pulses) until the receiver has had a stable link for `ESC_CAL_LINK_MS` (500 ms). Most ESCs wait silently without a signal, so the first pulse they see is the real stick position.

Radio endpoints at ±100 % (about 988–2012 µs) are clamped to 1000/2000 µs. No link means minimum output.

### 5. Normal flight firmware

Set every test flag back to `0`:

```c
#define RX_TEST   0
#define HW_TEST   0
#define ESC_CAL   0
#define IMU_TEST  0
```

Keep `DEBUG` and `LOG_ENABLE` as you prefer. Upload. The board now boots into the flight controller.

## Build-flag recipes

| Goal | Flags |
|---|---|
| Normal flight, no serial noise | `DEBUG 0`, `LOG_ENABLE 1` |
| Normal flight with debug lines | `DEBUG 1`, `LOG_ENABLE 1` |
| Receiver wiring check | `RX_TEST 1`, everything else `0` |
| Servo/motor direction check | `HW_TEST 1`, everything else `0` |
| ESC throttle calibration | `ESC_CAL 1`, analog protocol, everything else `0` |
| IMU health check | `IMU_TEST 1`, everything else `0` |
| Force recalibration at boot | `CAL_FORCE 1` (or `cal force` + reboot) |

## Common build problems

| Symptom | Cause / fix |
|---|---|
| `multiple definition of ... bno08x` | SparkFun BNO08x library is installed. Uninstall it; the CodeCell library already has the driver. |
| `ESC_CAL 1 cannot be combined with ...` | Set `RX_TEST`, `HW_TEST`, `IMU_TEST` to `0`. |
| `ESC_CAL 1 needs an analog protocol` | Set `ESC_PROTOCOL` to `0`, `3`, `4` or `5`, or leave `ESC_CAL 0`. |
| `Logging unavailable (no partition)` | Set Partition Scheme to **Default 4MB with spiffs**. |
| Upload fails or port disappears | Use a data USB-C cable, hold BOOT while plugging in, or lower the upload speed. |
| Serial Monitor shows nothing | `DEBUG 0` and `LOG_ENABLE 0` silence the port. Set `DEBUG 1` or type a command with `LOG_ENABLE 1`. |

---

Next: **[CALIBRATION.md](CALIBRATION.md)** · **[FLIGHT.md](FLIGHT.md)** · **[TROUBLESHOOTING.md](TROUBLESHOOTING.md)**
