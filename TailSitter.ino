/*
  ============================================================================
   CodeCell C3 - Twin-Motor TAIL-SITTER Flying Wing  -  ExpressLRS / CRSF  (v26)
  ============================================================================
   Board    : Microbots CodeCell C3 (ESP32-C3 + BNO085 IMU)
   Airframe : Flying wing, 2 motors (left/right), 2 elevons, sits on its tail
   Input    : ExpressLRS receiver, CRSF @ 420000 baud

   v26: arming and transition interlocks. Arming is refused while the gyro is
   moving or the attitude estimate is untrusted; the forward transition needs
   the throttle held above its floor and a short settle delay after arming.
   Elevon commands reject non-finite values. See TailSitter_REVIEW.md §20.
   v25: CRITICAL mixer finding closed and hardened. The v23 mixer already
   scales the differential so the low motor cannot be driven below the mixer
   floor; v25 adds a non-finite guard (mixer + motorCmd) and compile-time
   floor/limit checks. See TailSitter_REVIEW.md §19.
   v24: persistent boot calibration. The two-pose result (sensor->body mount
   and gyro bias) is stored in NVS and reused on later boots, so the aircraft
   can be powered up in the field without a known flat/vertical reference.
   `CAL_FORCE 1` or the serial command `cal force` re-runs the two-pose
   calibration at the next boot; `cal` runs it immediately while disarmed.
   (v23: mixer motor floor, failsafe pitch-back throttle, transition crash
   detection, I2C timeout/recovery, PID dt guard, stricter calibration angle.)

   CHANNELS (1-based, as shown on the radio)
     CH1  Roll      hover: lean left/right       forward: bank
     CH2  Pitch     hover: lean fwd/back         forward: pitch
     CH3  Throttle  hover: collective thrust     forward: thrust
     CH4  Yaw       hover: rotate about vertical forward: diff-thrust yaw
     CH5  ARM / DISARM   (high = arm, low = disarm - ELRS AUX1)
     CH6  FLIGHT MODE    (low = VERTICAL / tail-sitter, high = FORWARD / plane)

   CONTROL CONCEPT
     All control is done in the aircraft (plane) body frame with quaternions,
     so there is no gimbal lock at 90 deg nose-up:
       body X (nose)      -> elevons differential  (plane roll / hover yaw)
       body Y (wing span) -> elevons collective    (plane pitch / hover pitch)
       body Z (belly)     -> differential thrust   (plane yaw / hover side-lean)
     A transition variable tau (0 = hover, 1 = forward) blends the target
     attitude, stick mapping and PID gains.

   STATES
     DISARMED  -> motors off, elevons follow sticks (for pre-flight checks)
     HOVER     -> self-levelling vertical flight, heading hold
     TO_FWD    -> nose pitched down over TRANS_FWD_SEC, throttle floor applied
     FORWARD   -> self-levelling plane flight (angle mode)
     TO_HOVER  -> nose pitched back up over TRANS_BACK_SEC, throttle floor
     FAILSAFE  -> forced back to hover (transition floor, then FS_THR descent), disarm

   BOOT CALIBRATION  - the board may be mounted in ANY orientation.
   The first boot (or after `cal erase`) learns the mount with two poses:
     Step 1  Fast CYAN     : lay the aircraft FLAT on its BELLY, wings level,
                             hands off. Captured after 1.5 s still.
             Solid CYAN    : flat captured - now rotate it to vertical.
     Step 2  Fast MAGENTA  : (starts as soon as you rotate) stand it on its
                             TAIL, nose straight up, wings vertical, hands off.
             Solid MAGENTA : vertical captured (1.5 s) - orientation, gyro
                             bias and attitude are set.
   The result is saved in NVS. Later boots load it and initialise attitude
   from gravity, so NO flat/vertical reference is needed in the field.
   To recalibrate: set CAL_FORCE 1, or type `cal force` and reboot, or type
   `cal` while disarmed (runs the same two-pose process immediately).
   `cal status` shows the saved mount and bias; `cal erase` forgets them.
   Serial calibration commands require LOG_ENABLE 1 (the default).
     Then    GREEN blink   : ready / DISARMED.
   Servos follow the radio sticks whenever DISARMED (also during calibration).

   ONBOARD LED
     Fast/solid CYAN, MAGENTA - boot calibration steps (above)
     Fast RED    - IMU not found (halted, motors off)
     GREEN ~1 s  - calibrated, DISARMED
     ORANGE ~1 s - ARMED
     YELLOW ~1 s - DISARMED and flight log >= 95% full (dump + erase it)
     Fast YELLOW - erasing / preparing the flight log (wait)

   RECEIVER  - CRSF for Arduino library (ZZ-Cat), CRSF_LIB 1
     Wiring : receiver TX -> CodeCell IO1 (CRSF_RX_PIN)
              receiver RX -> CodeCell IO2 (CRSF_TX_PIN, optional)
              receiver 5V/GND from the BEC, common ground with the CodeCell
     Receiver: ExpressLRS, bound and linked, Serial Protocol = CRSF,
               baud 420000 (ELRS default). An unlinked ELRS receiver sends
               nothing, so the radio must be ON to see frames.
     DEBUG 1 prints a receiver status line every second while disarmed.

   RECEIVER TEST  - set RX_TEST 1, upload, open Serial Monitor 115200.
     Pin activity scan + baud/polarity sweep + verdict. No ESC/servo output.

   OUTPUTS  - servos and ESCs are started AFTER myCodeCell.Init().
   HW_TEST 1  - receiver + servo + ESC check without calibration or
     flight code .

   ESC OUTPUT  - set ESC_PROTOCOL to match the ESC:
       0 = PWM            1000-2000 us  @ ESC_PWM_HZ (default 400 Hz; 50 for old ESCs)
       1 = DSHOT  (RMT hardware, speed DSHOT_SPEED = 150 / 300 / 600)
       2 = DSHOT  (software timing, same speeds)
       3 = ONESHOT125      125-250 us   @ 2 kHz
       4 = ONESHOT42        42-84 us    @ 8 kHz
       5 = MULTISHOT         5-25 us    @ 32 kHz
     Analog protocols (0,3,4,5) use the PWM hardware; the throttle range is
     the same 1000-2000 "us" scale, scaled to each protocol's pulse width.
     The ESC receives "motor stop" (DShot 0 / minimum pulse) continuously from
     start-up so it can complete its arming tones. Analog ESCs use the 1000-2000
     range as their end points - recalibrate the ESC throttle range if the
     motors do not start at the bottom of the stick. Motor direction is set in the ESC
     configurator (BLHeli/Bluejay/AM32) or by swapping two motor wires.

   MOTOR TEST  - PROPS OFF
     Throttle LOW, then CH5 HIGH: both motors follow the THROTTLE stick
     directly (no stabilisation). CH5 LOW = stop. Use it to judge whether
     the motors spin up smoothly before trying the armed flight code.

   PITCH DIRECTION 
     RC_REV_PITCH is now true: on EdgeTX/OpenTX radios (mode 2) pulling the
     elevator stick BACK sends a LOW value, which v11-v16 read as nose-down.

   ESC CALIBRATION  - ESC_CAL 1.  !! PROPS OFF !!
     Throttle passthrough ONLY: CH3 is sent straight to BOTH ESCs on the
     1000-2000 scale of the selected analog protocol (ESC_PROTOCOL 0, 3, 4
     or 5). Nothing else runs: no IMU, no boot calibration, no arming, no
     stabilisation, no logging, no servo output.
     v22: the ESC signal pins are held LOW (no pulses) after power-up until
     the receiver has a STABLE link (ESC_CAL_LINK_MS). Most ESCs wait silently
     without a signal, so the first pulse they see is the real stick position.
       LED fast RED   = waiting for receiver link (ESCs get no signal)
       LED fast WHITE = passthrough active
     Radio endpoints at +/-100 % (988-2012 us) are clamped to 1000/2000.
     No radio link = minimum (1000) output.
     Procedure (typical BLHeli_S / BLHeli_32 / SimonK - check the ESC manual):
       1. Set ESC_CAL 1 and the ESC_PROTOCOL you will fly with. Upload.
       2. Power OFF. Radio ON, throttle stick FULL UP.
       3. Power the plane. LED fast RED until the receiver links, then WHITE;
          the ESCs then play start-up tones, then the "max" tones.
       4. Within ~2 s move the throttle stick FULL DOWN -> "min" tones.
       5. Confirmation tones = calibrated. Check: motors follow the stick,
          start just above the bottom. Power OFF.
       6. Set ESC_CAL 0 and upload the flight code.
     Serial Monitor (115200) shows CH3 and the output 5x per second.
     Not used for DShot (1, 2): DShot is digital and needs no calibration.

   FLIGHT LOG - record format 64 bytes (was 48). New columns:
     ch5_us, ch6_us (raw arm / mode switch), thr_stick (raw throttle stick
     before transition floors / failsafe), arm_sw, fwd_sw, landed, rx_fs,
     ahrs_trust (accel used by the attitude estimator), acc_g (|accel| in g),
     rate_sp_x/y/z (rate setpoints deg/s), i_x/y/z (PID integrators).
     Upgrading from v19 ERASES the old log on first boot - dump it first.
     ~7.5 min of recording at 50 Hz.
   FLIGHT LOG  - no USB cable needed in flight
     While ARMED the sketch records 50 samples/s to the CodeCell flash
     (~10 min total, kept through power-off). Each arm = new session.
     After landing: DISARM, connect USB, open a terminal at 115200 and type
       list            sessions with duration and last event
       dump            all sessions as CSV  (dump <n> = one session)
       erase           clear the log (takes a few seconds)
       cal             run the two-pose calibration now (props off)
       cal status      show the saved mount and gyro bias
       cal force       force the two-pose calibration at the next boot
       cal erase       forget the saved calibration
     Commands work only while disarmed. Requires Partition Scheme
     "Default 4MB with spiffs".

   IMU DATA 
     IMU_DRIVER 1 (default): the BNO085 is read DIRECTLY by a small built-in
       SHTP/SH-2 driver in this sketch (no extra library - the CodeCell library
       already contains the BNO085 driver files, so adding SparkFun's library
       causes "multiple definition" link errors).
       Calibrated gyro at 200 Hz + accelerometer at 100 Hz.
       The control loop runs on every new gyro sample (~200 Hz).
       myCodeCell.Run() is NOT used, so CodeCell power management cannot
       sleep the board in flight. The CodeCell library is still used for the
       LED and I2C setup.
     IMU_DRIVER 0: CodeCell library via myCodeCell.Run()  behaviour,
       measured only ~30 Hz of new gyro data - not recommended for hover).
     Power the CodeCell from the 5V pin (ESC BEC) - do NOT use the CodeCell
     battery connector in the aircraft.

   ARDUINO IDE (tested target: esp32 core 3.x): Board "ESP32C3 Dev Module", CPU 160 MHz, Flash 4MB,
     Partition Scheme "Default 4MB with spiffs" (REQUIRED for the flight log), USB CDC On Boot: Enabled, Core Debug Level: None
   Libraries (Library Manager): "CodeCell". "CRSFforArduino" only if CRSF_LIB 1.
     If "SparkFun BNO08x Cortex Based IMU" is installed it is not used -
     uninstall it if you prefer (it is not needed).

   !! SAFETY: REMOVE BOTH PROPELLERS FOR ALL BENCH TESTING !!
  ============================================================================
*/

#include <Arduino.h>
#include <CodeCell.h>
#include <Wire.h>
#include <Preferences.h>        // NVS: persistent boot calibration
#include <stddef.h>             // offsetof
#include <esp_partition.h>      // raw flash access for the flight log
#if __has_include(<esp_arduino_version.h>)
  #include <esp_arduino_version.h>
#endif
#ifndef ESP_ARDUINO_VERSION_MAJOR
  #define ESP_ARDUINO_VERSION_MAJOR 2
#endif
#include <soc/soc.h>            // REG_WRITE
#include <soc/gpio_reg.h>       // GPIO_OUT_W1TS_REG / W1TC (DShot bit output)
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  #include <esp_cpu.h>
  #define CYC_NOW()  esp_cpu_get_cycle_count()
#else
  #define CYC_NOW()  ESP.getCycleCount()
#endif

// ============================ DEBUG =========================================
#define DEBUG             0       // 1 = Serial debug, 0 = Serial completely off
#define CRSF_LIB          0       // 0 = built-in CRSF parser (verified by RX_TEST on this board)
                                  // 1 = CRSFforArduino library (received 0 frames on this board)
#define RX_TEST           0       // 1 = receiver wiring/baud diagnostic only (no flight code)
#define HW_TEST           0       // 1 = receiver + servo + ESC check only (no calibration / flight)
#define ESC_CAL           0       // 1 = ESC throttle-range calibration: CH3 passthrough ONLY.
                                  //     Analog protocols (ESC_PROTOCOL 0/3/4/5). PROPS OFF.
#define ESC_CAL_LINK_MS   500     // ESC_CAL: link must be stable this long before ESC output starts
#define RC_STATUS_MS      1000    // DEBUG: receiver status line interval while disarmed
#define RX_RESTART_MS     2000    // disarmed + no receiver bytes this long -> restart UART
#if (CRSF_LIB + 0)
  #include "CRSFforArduino.hpp"     // Library Manager: "CRSFforArduino" by ZZ-Cat
#endif
#define IMU_DRIVER        1       // 1 = built-in direct BNO085 driver (fast), 0 = CodeCell library
#define IMU_TEST          0       // 1 = raw IMU monitor only (no flight code runs).
                                  //     Prints accel/gyro + update rates at 5 Hz.
#define DEBUG_MAX_HZ      20
#define DEBUG_ANGLE_STEP  0.5f
#define DEBUG_US_STEP     3
#define LOG_ENABLE        1       // 1 = onboard flight log + USB log commands
#define LOG_HZ            50      // log samples per second while armed

// Serial is started if DEBUG or LOG_ENABLE is on. With DEBUG 0 nothing is
// printed unless you type a log command (list / dump / erase / help).
#if (DEBUG + 0) || (LOG_ENABLE + 0)
  #if ARDUINO_USB_CDC_ON_BOOT
    // TX timeout 0: printing never stalls the loop when no PC is attached
    #define DBG_BEGIN()    do { Serial.setTxBufferSize(1024); Serial.begin(115200); \
                                Serial.setTxTimeoutMs(0); delay(300); } while (0)
  #else
    #define DBG_BEGIN()    do { Serial.begin(115200); delay(300); } while (0)
  #endif
  #define LOG_PRINTF(...)  Serial.printf(__VA_ARGS__)
#else
  #define DBG_BEGIN()      do {} while (0)
  #define LOG_PRINTF(...)  do {} while (0)
#endif
#if (DEBUG + 0)
  #define DBG_PRINTF(...)  Serial.printf(__VA_ARGS__)
#else
  #define DBG_PRINTF(...)  do {} while (0)
#endif
// Calibration messages are useful even when DEBUG is off (the serial `cal`
// command); print them whenever the serial port is running for logging.
#if (DEBUG + 0) || (LOG_ENABLE + 0)
  #define CAL_PRINTF(...)  Serial.printf(__VA_ARGS__)
#else
  #define CAL_PRINTF(...)  do {} while (0)
#endif

// ============================ PINS ==========================================
// CodeCell C3 has 6 GPIOs. Do NOT use SDA/SCL (onboard sensors).
#define CRSF_RX_PIN    1     // <- receiver TX   (CRSF data in)
#define CRSF_TX_PIN    2     // -> receiver RX   (optional; a real pin is required so
                             //    the UART never falls back to the USB pins)
#define MOTOR_L_PIN    5     // Left ESC
#define MOTOR_R_PIN    3     // Right ESC
#define SERVO_L_PIN    6     // Left elevon
#define SERVO_R_PIN    7     // Right elevon

// ============================ CRSF / RC =====================================
#define CRSF_BAUD       420000
#define MAX_CH          16
#define CH_ROLL         0    // CH1
#define CH_PITCH        1    // CH2
#define CH_THR          2    // CH3
#define CH_YAW          3    // CH4
#define CH_ARM          4    // CH5
#define CH_MODE         5    // CH6
#define RC_REV_ROLL     false   // right stick must read +
#define RC_REV_PITCH    true    // PULL BACK must read + (nose up / lean back).
                                // EdgeTX/OpenTX: stick back = LOW value -> true
#define RC_REV_YAW      false   // right stick must read +
#define RC_DEADBAND     0.03f
#define RC_EXPO 0.35f           // 0 = linear, 0.3-0.5 = softer around centre
#define RC_TIMEOUT_MS   250
// IMPORTANT: configure the ELRS receiver failsafe as "no pulses" (or a preset
// that drops LQ to 0). The built-in CRSF parser cannot see a "hold" failsafe
// bit, so a receiver that holds stale channel values can defeat failsafe.
#define ARM_SWITCH_US   1700   // CH5 above = armed
#define MODE_SWITCH_US  1500   // CH6 above = forward flight
#define ARM_MAX_THR     0.05f  // throttle must be this low to arm
#define ARM_MAX_TILT    30.0f  // deg from vertical allowed when arming
#define ARM_MAX_RATE_DPS 20.0f // gyro rate above this counts as "moving"
#define ARM_STILL_MS    100    // must be still this long before arming
#define ARM_MIN_TRUST   0.5f   // accel trust required to arm (0..1)

// ============================ BOOT CALIBRATION ========================
// Body frame used by the controller: X = nose, Y = right wing, Z = belly (down).
// The two-pose calibration learns the sensor-to-body rotation and the gyro bias.
// The result is stored in NVS so it survives power-off and can be reused in the
// field, where a known-flat / known-vertical reference is not available.
//   CAL_FORCE 1  : always run the two-pose calibration at boot (bench setup).
//   Serial       : cal | cal status | cal force | cal erase
//   cal force    : one-shot NVS flag; the next boot runs calibration and clears it.
#define CAL_FORCE             0       // 1 = force the two-pose calibration at every boot
#define CAL_STILL_MS          1500    // each pose must be held still this long
#define CAL_STILL_DPS         8.0f    // gyro limit (deg/s) for "still"
#define CAL_STILL_DEG         3.0f    // max change of "up" direction while holding
#define CAL_SPIKE_ALLOW       3       // consecutive out-of-limit samples ignored
#define CAL_G_MIN             0.75f   // accepted |accel| range (g); samples
#define CAL_G_MAX             1.25f   //   outside are skipped, not a reset
#define CAL_MOVE_AWAY_DEG     20.0f   // LED switches to MAGENTA once rotated this far
#define CAL_MIN_ANGLE_DEG     80.0f   // vertical pose must differ from flat by >= this
#define CAL_WARN_ANGLE_DEG    85.0f   // warn if the captured pose is worse than this
#define CAL_DONE_SOLID_MS     1500    // solid MAGENTA time after vertical capture

// ---------------------------- Calibration storage (NVS) ---------------------
#define CAL_NVS_NAMESPACE     "tailcal"
#define CAL_NVS_KEY           "cal"
#define CAL_NVS_FORCE_KEY     "force"
#define CAL_MAGIC             0x54414331UL   // "TAC1"
#define CAL_VERSION           1

// ============================ ATTITUDE ESTIMATOR ============================
#define AHRS_KP              1.0f    // accel correction gain (in flight)
#define AHRS_KP_INIT         10.0f   // fast convergence during calibration
#define AHRS_ACC_TRUST_BAND  0.25f   // fraction of 1 g where accel is trusted

// ============================ OUTPUTS =======================================
#define ESC_PROTOCOL      3       // 0 PWM | 1 DSHOT (RMT) | 2 DSHOT (software)
                                  // 3 ONESHOT125 | 4 ONESHOT42 | 5 MULTISHOT
#define DSHOT_SPEED       150     // DShot speed: 150, 300 or 600
#define ESC_PWM_HZ        400     // ESC_PROTOCOL 0 only (50 for old/standard ESCs)
#define DSHOT_RATE_HZ     500     // DShot keep-alive frame rate (control ticks also send)
#define DSHOT_IDLE        100     // armed idle command (48..2047; 48 = lowest throttle)
#define DSHOT_MAX         2047    // full throttle
#define SERVO_HZ          50
// ---- setting checks: an empty value (e.g. "#define ESC_PROTOCOL  // 0 PWM")
//      gives a clear message instead of "operator '==' has no left operand"
#if (1 - ESC_PROTOCOL - 1) == 2
  #error "ESC_PROTOCOL has no value - the line must read e.g.  #define ESC_PROTOCOL  0"
#endif
#if (1 - DSHOT_SPEED - 1) == 2
  #error "DSHOT_SPEED has no value - use 150, 300 or 600"
#endif
#if (1 - ESC_PWM_HZ - 1) == 2
  #error "ESC_PWM_HZ has no value - use e.g. 400 or 50"
#endif
#define ESC_IS_DSHOT      (ESC_PROTOCOL == 1 || ESC_PROTOCOL == 2)
#if   ESC_PROTOCOL == 3
  #define ESC_HZ          2000    // OneShot125
  #define ESC_RES_BITS    14
#elif ESC_PROTOCOL == 4
  #define ESC_HZ          8000    // OneShot42
  #define ESC_RES_BITS    13
#elif ESC_PROTOCOL == 5
  #define ESC_HZ          32000   // Multishot
  #define ESC_RES_BITS    11
#else
  #define ESC_HZ          ESC_PWM_HZ
  #define ESC_RES_BITS    14
#endif
#if !(ESC_PROTOCOL >= 0 && ESC_PROTOCOL <= 5)
  #error "ESC_PROTOCOL must be 0..5"
#endif
#if (1 - ESC_CAL - 1) == 2
  #error "ESC_CAL has no value - use 0 (normal) or 1 (ESC calibration)"
#endif
#if (ESC_CAL + 0) && ESC_IS_DSHOT
  #error "ESC_CAL 1 needs an analog protocol (ESC_PROTOCOL 0, 3, 4 or 5). DShot needs no calibration."
#endif
#if (ESC_CAL + 0) && ((RX_TEST + 0) || (HW_TEST + 0) || (IMU_TEST + 0))
  #error "ESC_CAL 1 cannot be combined with RX_TEST, HW_TEST or IMU_TEST - set those to 0"
#endif
#if ESC_IS_DSHOT && !(DSHOT_SPEED == 150 || DSHOT_SPEED == 300 || DSHOT_SPEED == 600)
  #error "DSHOT_SPEED must be 150, 300 or 600"
#endif
#define PWM_RES_BITS      14
#define SERVO_CENTER_L    1500
#define SERVO_CENTER_R    1500
#define SERVO_THROW_US    400
#define SERVO_MIN_US      1000
#define SERVO_MAX_US      2000
#define SERVO_REV_L       false   // +command must move LEFT elevon UP
#define SERVO_REV_R       true    // +command must move RIGHT elevon UP
#define ROLL_MIX          0.6f
#define PITCH_MIX         0.8f

#define MOTOR_OFF_US      1000    // analog (PWM/OneShot/Multishot): disarmed, 1000-2000 scale
#define MOTOR_IDLE_US     1080    // analog: armed idle (props spin slowly)
#define MOTOR_MAX_US      2000    // analog: full throttle
#if ESC_IS_DSHOT
  #define MOTOR_OFF_CMD   0               // DShot 0 = motor stop
#else
  #define MOTOR_OFF_CMD   MOTOR_OFF_US
#endif
#define DIFF_THRUST_MAX   0.30f   // max +/- differential thrust (fraction)
#define AIRMODE_MIN_THR   0.15f   // low-motor floor for differential mixing
// Mixer invariant: when the collective is at/above AIRMODE_MIN_THR both motors
// stay in [AIRMODE_MIN_THR, 1]; below it the differential is removed so the
// motors stay equal. These bounds keep that guarantee meaningful.
static_assert(AIRMODE_MIN_THR > 0.0f && AIRMODE_MIN_THR < 0.5f,
              "AIRMODE_MIN_THR must leave room for differential thrust");
static_assert(DIFF_THRUST_MAX > 0.0f && DIFF_THRUST_MAX <= 0.5f,
              "DIFF_THRUST_MAX must be a small fraction of throttle");

// ============================ FLIGHT / TRANSITION ===========================
#define IMU_RUN_HZ            100     // IMU_DRIVER 0: myCodeCell.Run() rate
#define BNO_ADDR              0x4A    // CodeCell onboard BNO085 I2C address
#define BNO_SDA               8       // CodeCell I2C (shared with onboard sensors)
#define BNO_SCL               9
#define BNO_GYRO_MS           5       // IMU_DRIVER 1: gyro report interval (200 Hz)
#define BNO_ACCEL_MS          10      // IMU_DRIVER 1: accel report interval (100 Hz)
#if IMU_DRIVER == 1
  #define IMU_RATE_HZ         (1000 / BNO_GYRO_MS)
#else
  #define IMU_RATE_HZ         IMU_RUN_HZ
#endif
#define IMU_LOSS_MS           200     // no fresh IMU sample this long while armed -> disarm
#define HOVER_MAX_TILT_DEG    15.0f   // hover lean from sticks
#define HOVER_YAW_RATE_DPS    90.0f   // hover heading rate at full yaw stick
#define HOVER_HDG_SLACK_DEG   25.0f     // heading target never trails the real heading by more than this
#define FWD_PITCH_DEG         3.0f    // forward-flight nose attitude, sticks centred
#define FWD_MAX_BANK_DEG      45.0f
#define FWD_MAX_PITCH_DEG     25.0f
#define FWD_YAW_RATE_DPS      60.0f   // forward yaw-rate at full yaw stick

#define TRANS_FWD_SEC         2.0f    // hover -> forward pitch-down time
#define TRANS_BACK_SEC        1.5f    // forward -> hover pitch-up time
#define TRANS_START_MIN_THR   0.30f   // must be airborne (throttle) to start fwd transition
#define TRANS_START_HOLD_MS   500     // throttle must stay above the floor this long to start
#define TRANS_ARM_DELAY_MS    1000    // no forward transition this soon after arming
#define TRANS_FWD_THR_MIN     0.65f   // throttle floor during hover->forward
#define TRANS_BACK_THR_MIN    0.65f // was 0.55 - margin above hover when pitching backup

#define FS_THR                0.48f // was 0.40 - just below the ~0.55 hover
#define FS_DISARM_MS          15000   // disarm after this long in failsafe
#define CRASH_TILT_DEG        70.0f   // hover tilt that counts as a crash
#define CRASH_ATT_ERR_DEG     60.0f   // transition/forward attitude error that counts as a crash
#define CRASH_TIME_MS         1000
#define LANDED_THR            0.30f   // below this in hover: reset integrators

// ============================ GAINS =========================================
// Angle loop: rate setpoint (rad/s) = ANG_P * angle error (rad)
// Rate loop : output (-1..1)      = P*err + I*int(err) - D*d(gyro)/dt
// Axis order: [0]=X (elevon diff), [1]=Y (elevon collective), [2]=Z (diff thrust)
struct Gains { float angP[3]; float rP[3]; float rI[3]; float rD[3]; float maxRate[3]; };
Gains G_HOVER = {
  {2.0f, 4.5f, 3.0f}, // angP: heading 4.5->2.0, side lean 4.5->3.0
  {0.20f, 0.25f, 0.20f}, // rP: heading 0.12->0.20, pitch 0.15->0.25
  {0.10f, 0.20f, 0.15f}, // rI: pitch 0.12->0.20
  {0.003f, 0.006f, 0.004f}, // rD: pitch 0.004->0.006
  {1.0f, 3.0f, 3.0f} // maxRate: heading 2.0->1.0
};

Gains G_FWD = {
  {3.5f, 3.5f, 0.0f},           // no heading hold in forward flight
  {0.08f, 0.10f, 0.06f},
  {0.08f, 0.10f, 0.02f},
  {0.002f, 0.003f, 0.0f},
  {3.5f, 2.5f, 1.5f}
};
#define I_LIMIT   0.40f
#define D_LPF     0.25f

// ============================ LED ===========================================
#define LED_FAST_MS       60
#define LED_SLOW_ON_MS    120
#define LED_SLOW_PERIOD   1000

// ============================================================================
CodeCell myCodeCell;
const float D2R = 0.01745329252f, R2D = 57.2957795f;
const float GRAV = 9.80665f;

// ============================================================================
// TYPES - must be declared BEFORE any function. The Arduino IDE auto-generates
// function prototypes and inserts them above the first function in the sketch;
// any type used in a function signature has to exist at that point.
// ============================================================================
struct Quat { float w, x, y, z; };            // q rotates BODY -> WORLD (NED)
struct Mat3 { float m[3][3]; };               // rotation matrix
struct PwmOut { int pin; int ch; int hz; };   // LEDC output (pin, v2.x channel, Hz)

// Persistent calibration record stored in NVS. `size` and the CRC make an
// interrupted/old/corrupt write easy to reject instead of using bad offsets.
struct CalBlob {
  uint32_t magic;          // CAL_MAGIC
  uint16_t version;        // CAL_VERSION
  uint16_t size;           // sizeof(CalBlob)
  float    mnt[3][3];      // rows: body X (nose), Y (right), Z (belly) in sensor axes
  float    gyroBias[3];    // body frame, rad/s
  uint32_t crc;            // CRC32 over everything above
};

enum LedPattern  { LED_CAL_FLAT, LED_CAL_FLAT_OK, LED_CAL_VERT, LED_CAL_VERT_OK,
                   LED_CAL_FAILED, LED_DISARMED, LED_ARMED,
                   LED_LOG_BUSY, LED_LOG_FULL, LED_ESC_CAL };
#define LOG_REC_SIZE     64
enum LogType  { LOGT_DATA = 0, LOGT_EVENT = 1 };
enum LogEvent { EV_NONE = 0, EV_ARM, EV_DISARM_SWITCH, EV_DISARM_FS_TIMEOUT,
                EV_DISARM_CRASH, EV_DISARM_IMU_LOST, EV_FAILSAFE, EV_LINK_RESTORED };

struct __attribute__((packed)) LogRec {
  uint16_t magic;          // LOG_MAGIC
  uint8_t  flags;          // b0-2 state, b3 armed, b4 link ok, b5-6 type
  uint8_t  tau;            // tau * 250
  uint32_t tMs;            // millis()
  int8_t   lq;             // link quality %
  uint8_t  loopHz2;        // control loop Hz / 2
  int16_t  nose, bank, hdg;            // deg * 10
  int8_t   stk[3];                     // roll/pitch/yaw stick * 100
  uint8_t  thr;                        // throttle * 250 (after floors / failsafe)
  int16_t  err[3];                     // attitude error deg * 10 (body X,Y,Z)
  int8_t   out[3];                     // PID outputs * 100
  int16_t  gyr[3];                     // body rates deg/s * 10
  uint16_t servoL, servoR, motorL, motorR;   // servo us, motor cmd (DShot 0-2047 or PWM us)
  uint8_t  glitches;                   // IMU glitch counter (low byte)
  uint16_t session;
  uint8_t  event;                      // LogEvent
  // ---- v20 additions ----
  uint16_t ch5, ch6;                   // raw arm / mode switch (us)
  uint8_t  thrStk;                     // raw throttle stick * 250
  uint8_t  flags2;                     // b0 armSw, b1 fwdSw, b2 landed, b3 rx failsafe,
                                       // b4-7 AHRS accel trust * 15
  uint8_t  accG;                       // |accel| in g * 50 (0..5.1 g)
  int16_t  rateSp[3];                  // rate setpoints deg/s * 10
  int8_t   iTerm[3];                   // PID integrators * 100
  uint8_t  csum;                       // XOR of bytes 0..62
};
static_assert(sizeof(LogRec) == LOG_REC_SIZE, "LogRec must be 64 bytes");

enum FlightState { ST_DISARMED, ST_HOVER, ST_TO_FWD, ST_FORWARD, ST_TO_HOVER, ST_FAILSAFE };

// v2.x core: channels 0/1 (timer 0) = servos @ SERVO_HZ, 2/3 (timer 1) = ESCs
PwmOut outSL = {SERVO_L_PIN, 0, SERVO_HZ}, outSR = {SERVO_R_PIN, 1, SERVO_HZ};
PwmOut outML = {MOTOR_L_PIN, 2, ESC_HZ},   outMR = {MOTOR_R_PIN, 3, ESC_HZ};   // analog ESC modes only

// ---------------------------- LED -------------------------------------------
LedPattern ledPattern = LED_CAL_FLAT;
int8_t ledState = -1;

void ledSet(bool on) {
  if ((int8_t)on == ledState) return;
  ledState = on;
  if (!on) { myCodeCell.LED(0, 0, 0); return; }
  switch (ledPattern) {
    case LED_CAL_FLAT:
    case LED_CAL_FLAT_OK: myCodeCell.LED(0, 255, 255);   break;   // cyan
    case LED_CAL_VERT:
    case LED_CAL_VERT_OK: myCodeCell.LED(255, 0, 255);   break;   // magenta
    case LED_CAL_FAILED:  myCodeCell.LED(255, 0, 0);     break;
    case LED_DISARMED:    myCodeCell.LED(0, 255, 0);     break;
    case LED_ARMED:       myCodeCell.LED(255, 120, 0);   break;
    case LED_LOG_BUSY:    myCodeCell.LED(255, 200, 0); break;
    case LED_LOG_FULL:    myCodeCell.LED(255, 200, 0); break;
    case LED_ESC_CAL:     myCodeCell.LED(255, 255, 255); break;   // white
  }
}
void ledUpdate() {
  uint32_t t = millis();
  if (ledPattern == LED_CAL_FLAT_OK || ledPattern == LED_CAL_VERT_OK) { ledSet(true); return; }
  bool slow = (ledPattern == LED_DISARMED || ledPattern == LED_ARMED || ledPattern == LED_LOG_FULL);
  ledSet(slow ? (t % LED_SLOW_PERIOD) < LED_SLOW_ON_MS : ((t / LED_FAST_MS) & 1));
}
void ledSetPattern(LedPattern p) { if (p != ledPattern) { ledPattern = p; ledState = -1; } }
bool imuService();   // defined in the IMU section
void idleService();  // defined before setup(): radio + ESC keep-alive + disarmed servos
// Delay that keeps the IMU, radio, ESCs, servos and LED alive
void ledDelay(uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) { imuService(); idleService(); ledUpdate(); delay(1); }
}
// Delay that keeps radio, ESCs and servos alive (no IMU polling)
void svcDelay(uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) { idleService(); ledUpdate(); delay(1); }
}

// ---------------------------- CRSF ------------------------------------------
uint16_t rcUs[MAX_CH];
uint32_t rcLastFrameMs = 0;
bool     rcEverValid   = false;
bool     rcFailsafeFlag = false;     // receiver-reported failsafe
int16_t  linkLQ        = -1;
uint32_t rcFrames      = 0;          // valid RC frames received
uint32_t escFrames = 0;              // ESC frames sent (declared early for status lines)
uint32_t rcChangedFrames = 0;        // RC frames whose CH1-CH8 differ from the previous frame
uint16_t rcPrevUs[8]   = {0};
uint32_t rcBytes       = 0;          // CRSF_LIB 0: raw bytes received
uint32_t rcCrcErrors   = 0;          // CRSF_LIB 0: frames with bad CRC

#if (CRSF_LIB + 0)
CRSFforArduino* crsf = nullptr;
bool crsfStarted = false;

bool crsfBegin() {
  Serial1.setRxBufferSize(1024);               // must be set before the UART starts
  crsf = new CRSFforArduino(&Serial1, CRSF_RX_PIN, CRSF_TX_PIN);
  if (!crsf->begin(CRSF_BAUD)) {
    crsf->end(); delete crsf; crsf = nullptr;
    return false;
  }
  // Callbacks are captureless lambdas (no auto-generated prototypes needed)
  crsf->setRcChannelsCallback([](serialReceiverLayer::rcChannels_t* rc) {
    if (rc->failsafe) { rcFailsafeFlag = true; return; }
    rcFailsafeFlag = false;
    for (uint8_t i = 0; i < MAX_CH; i++)
      rcUs[i] = crsf->rcToUs(crsf->getChannel(i + 1));    // channels are 1-based
    if (memcmp(rcPrevUs, rcUs, sizeof(rcPrevUs)) != 0) { rcChangedFrames++; memcpy(rcPrevUs, rcUs, sizeof(rcPrevUs)); }
    rcLastFrameMs = millis();
    rcEverValid = true;
    rcFrames++;
  });
  crsf->setLinkStatisticsCallback([](serialReceiverLayer::link_statistics_t ls) {
    linkLQ = (int16_t)ls.lqi;                              // uplink link quality %
  });
  crsfStarted = true;
  return true;
}

void crsfPoll() { if (crsf) crsf->update(); }
void crsfWatchdog(bool) { }

#else   // ---------- built-in parser (CRSF_LIB 0) ----------
#define CRSF_TYPE_RC_CHANNELS  0x16
#define CRSF_TYPE_LINK_STATS   0x14
#define CRSF_MAX_FRAME         64

uint8_t crc8_dvb_s2(const uint8_t* p, uint8_t len) {
  uint8_t crc = 0;
  while (len--) {
    crc ^= *p++;
    for (uint8_t i = 0; i < 8; i++)
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
  }
  return crc;
}

void crsfHandleFrame(const uint8_t* f) {
  uint8_t len = f[1], type = f[2];
  if (crc8_dvb_s2(&f[2], len - 1) != f[len + 1]) { rcCrcErrors++; return; }
  if (type == CRSF_TYPE_RC_CHANNELS && len == 24) {
    const uint8_t* p = &f[3];
    uint32_t bits = 0; uint8_t nBits = 0, ch = 0;
    for (uint8_t i = 0; i < 22 && ch < MAX_CH; i++) {
      bits |= (uint32_t)p[i] << nBits; nBits += 8;
      while (nBits >= 11 && ch < MAX_CH) {
        uint16_t v = bits & 0x7FF; bits >>= 11; nBits -= 11;
        rcUs[ch++] = (uint16_t)(1500 + ((int32_t)v - 992) * 5 / 8);
      }
    }
    if (memcmp(rcPrevUs, rcUs, sizeof(rcPrevUs)) != 0) { rcChangedFrames++; memcpy(rcPrevUs, rcUs, sizeof(rcPrevUs)); }
    rcLastFrameMs = millis();
    rcEverValid = true;
    rcFrames++;
  } else if (type == CRSF_TYPE_LINK_STATS && len >= 12) {
    linkLQ = f[5];
  }
}

uint32_t rcLastByteMs = 0, rcUartRestarts = 0;

bool crsfBegin() {
  Serial1.end();
  delay(5);
  Serial1.setRxBufferSize(1024);
  Serial1.begin(CRSF_BAUD, SERIAL_8N1, CRSF_RX_PIN, CRSF_TX_PIN, false);
  rcLastByteMs = millis();
  return true;
}

// Disarmed only: if the UART has been silent for RX_RESTART_MS, restart it
void crsfWatchdog(bool isArmed) {
  if (isArmed || millis() - rcLastByteMs < RX_RESTART_MS) return;
  rcUartRestarts++;
  DBG_PRINTF("RX: no bytes for %d ms - restarting UART (#%lu)\n", RX_RESTART_MS, (unsigned long)rcUartRestarts);
  crsfBegin();
}

void crsfPoll() {
  static uint8_t buf[CRSF_MAX_FRAME];
  static uint8_t pos = 0;
  while (Serial1.available()) {
    uint8_t b = Serial1.read();
    rcBytes++;
    rcLastByteMs = millis();
    if (pos == 0) {                   // address byte: FC (0xC8), broadcast (0x00),
      if (b == 0xC8 || b == 0xEE || b == 0xEA || b == 0x00) buf[pos++] = b;   // TX/RX module
      continue;
    }
    if (pos == 1) {
      if (b < 2 || b > CRSF_MAX_FRAME - 2) { pos = 0; continue; }
      buf[pos++] = b; continue;
    }
    buf[pos++] = b;
    if (pos >= buf[1] + 2) { crsfHandleFrame(buf); pos = 0; }
  }
}
#endif

// DEBUG: one receiver status line per RC_STATUS_MS while disarmed
void rcStatusReport() {
#if (DEBUG + 0)
  static uint32_t last = 0, lastFrames = 0, lastBytes = 0;
  if (millis() - last < RC_STATUS_MS) return;
  uint32_t dt = millis() - last; last = millis();
  uint32_t fps = (rcFrames - lastFrames) * 1000UL / (dt ? dt : 1);
  lastFrames = rcFrames;
  bool ok = rcEverValid && (millis() - rcLastFrameMs < RC_TIMEOUT_MS);
  static uint32_t lastChg = 0, lastEsc = 0;
  uint32_t cps = (rcChangedFrames - lastChg) * 1000UL / (dt ? dt : 1); lastChg = rcChangedFrames;
  uint32_t eps = (escFrames - lastEsc) * 1000UL / (dt ? dt : 1);       lastEsc = escFrames;
#if (CRSF_LIB + 0)
  (void)lastBytes;
  DBG_PRINTF("RX[lib %s]: frames %lu (%lu/s, changed %lu/s) %s LQ %d | CH1 %4u CH2 %4u CH3 %4u CH4 %4u CH5 %4u CH6 %4u%s | ESC %lu fr/s\n",
             crsfStarted ? "ok" : "FAILED", (unsigned long)rcFrames, (unsigned long)fps, (unsigned long)cps,
             ok ? "LINK" : "NO LINK", linkLQ,
             rcUs[0], rcUs[1], rcUs[2], rcUs[3], rcUs[4], rcUs[5], rcFailsafeFlag ? " FAILSAFE" : "", (unsigned long)eps);
  if (!rcFrames) DBG_PRINTF("   no frames: if RX_TEST 1 passes, set CRSF_LIB 0 (built-in parser)\n");
#else
  uint32_t bps = (rcBytes - lastBytes) * 1000UL / (dt ? dt : 1);
  lastBytes = rcBytes;
  DBG_PRINTF("RX[built-in]: bytes %lu/s frames %lu (%lu/s, changed %lu/s) crcErr %lu %s LQ %d | CH1 %4u CH2 %4u CH3 %4u CH4 %4u CH5 %4u CH6 %4u | ESC %lu fr/s\n",
             (unsigned long)bps, (unsigned long)rcFrames, (unsigned long)fps, (unsigned long)cps, (unsigned long)rcCrcErrors,
             ok ? "LINK" : "NO LINK", linkLQ, rcUs[0], rcUs[1], rcUs[2], rcUs[3], rcUs[4], rcUs[5], (unsigned long)eps);
  if (!bps) DBG_PRINTF("   no bytes on IO%d (UART restarts %lu): run RX_TEST 1 to confirm the receiver is sending\n",
                       CRSF_RX_PIN, (unsigned long)rcUartRestarts);
  else if (!fps) DBG_PRINTF("   bytes but no frames: wrong baud or receiver not in CRSF mode\n");
#endif
#endif
}

// ---------------------------- PWM (LEDC) ------------------------------------

void pwmBegin(const PwmOut& o) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(o.pin, o.hz, PWM_RES_BITS);
#else
  ledcSetup(o.ch, o.hz, PWM_RES_BITS);
  ledcAttachPin(o.pin, o.ch);
#endif
}
void pwmWriteUs(const PwmOut& o, int us) {
  const uint32_t periodUs = 1000000UL / o.hz;
  const uint32_t maxDuty  = (1UL << PWM_RES_BITS) - 1;
  uint32_t duty = ((uint32_t)us * maxDuty) / periodUs;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(o.pin, duty);
#else
  ledcWrite(o.ch, duty);
#endif
}

// ---------------------------- ESC output (DShot / OneShot / Multishot / PWM)
// DShot300: 16-bit frame = 11-bit throttle, 1 telemetry bit, 4-bit CRC.
//   bit period 3.333 us, "1" high 2.50 us, "0" high 1.25 us, MSB first.
// ESC_PROTOCOL 1: RMT hardware, 20 MHz tick: bit 67, "1" high 50, "0" high 25.
//   Each motor has its own RMT TX channel; frames are queued asynchronously.
// ESC_PROTOCOL 2 (or RMT fallback): cycle-timed software loop in IRAM.
uint16_t escValL = MOTOR_OFF_CMD, escValR = MOTOR_OFF_CMD;   // current commands
uint32_t escLastSendUs = 0;
uint32_t escSkipped = 0;                                     // RMT busy, frame skipped
bool     dshotRmt = false;                                   // true = RMT hardware in use

#if ESC_IS_DSHOT
#define DSHOT_RMT_HZ   20000000                  // 50 ns RMT tick
#if   DSHOT_SPEED == 150
  #define DSHOT_T_BIT  133                       // 6.67 us
  #define DSHOT_T1H    100                       // 5.00 us
  #define DSHOT_T0H    50                        // 2.50 us
#elif DSHOT_SPEED == 600
  #define DSHOT_T_BIT  33                        // 1.65 us
  #define DSHOT_T1H    25                        // 1.25 us
  #define DSHOT_T0H    12                        // 0.60 us
#else
  #define DSHOT_T_BIT  67                        // 3.33 us
  #define DSHOT_T1H    50                        // 2.50 us
  #define DSHOT_T0H    25                        // 1.25 us
#endif

portMUX_TYPE dshotMux = portMUX_INITIALIZER_UNLOCKED;
uint32_t dshotCycBit = 533, dshotCycOne = 400, dshotCycZero = 200;  // @160 MHz

uint16_t dshotPacket(uint16_t value) {
  if (value > 2047) value = 2047;
  uint16_t p = (uint16_t)(value << 1);                 // telemetry bit = 0
  uint16_t crc = (p ^ (p >> 4) ^ (p >> 8)) & 0x0F;
  return (uint16_t)((p << 4) | crc);
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
rmt_data_t dshotSymL[16], dshotSymR[16];

void dshotBuild(rmt_data_t* sym, uint16_t pkt) {
  for (int i = 0; i < 16; i++) {
    bool one = pkt & (0x8000 >> i);
    sym[i].level0    = 1;
    sym[i].duration0 = one ? DSHOT_T1H : DSHOT_T0H;
    sym[i].level1    = 0;
    sym[i].duration1 = DSHOT_T_BIT - sym[i].duration0;
  }
}

bool dshotRmtBegin() {
  if (!rmtInit(MOTOR_L_PIN, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, DSHOT_RMT_HZ)) return false;
  if (!rmtInit(MOTOR_R_PIN, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, DSHOT_RMT_HZ)) {
    rmtDeinit(MOTOR_L_PIN);
    return false;
  }
  return true;
}

// Queue one frame per motor; skip a motor whose previous frame is still running
bool dshotSentL = false, dshotSentR = false;
void dshotRmtSend(uint16_t pa, uint16_t pb) {
  if (!dshotSentL || rmtTransmitCompleted(MOTOR_L_PIN)) {
    dshotBuild(dshotSymL, pa); dshotSentL = rmtWriteAsync(MOTOR_L_PIN, dshotSymL, 16);
  } else escSkipped++;
  if (!dshotSentR || rmtTransmitCompleted(MOTOR_R_PIN)) {
    dshotBuild(dshotSymR, pb); dshotSentR = rmtWriteAsync(MOTOR_R_PIN, dshotSymR, 16);
  } else escSkipped++;
}
#endif

void IRAM_ATTR dshotSend2(uint16_t pa, uint16_t pb) {
  const uint32_t ma = 1UL << MOTOR_L_PIN, mb = 1UL << MOTOR_R_PIN;
  const uint32_t cBit = dshotCycBit, c1 = dshotCycOne, c0 = dshotCycZero;
  portENTER_CRITICAL(&dshotMux);
  uint32_t t = CYC_NOW() + 64;
  for (int i = 15; i >= 0; i--) {
    uint32_t ha = ((pa >> i) & 1) ? c1 : c0;
    uint32_t hb = ((pb >> i) & 1) ? c1 : c0;
    uint32_t hMin = ha < hb ? ha : hb, hMax = ha < hb ? hb : ha;
    uint32_t mFirst = (ha == hMin ? ma : 0) | (hb == hMin ? mb : 0);
    uint32_t g = 0;                                    // guards: never hang with IRQs off
    while ((int32_t)(CYC_NOW() - t) < 0 && ++g < 20000) { }
    REG_WRITE(GPIO_OUT_W1TS_REG, ma | mb);
    while ((uint32_t)(CYC_NOW() - t) < hMin && ++g < 20000) { }
    REG_WRITE(GPIO_OUT_W1TC_REG, mFirst);
    while ((uint32_t)(CYC_NOW() - t) < hMax && ++g < 20000) { }
    REG_WRITE(GPIO_OUT_W1TC_REG, ma | mb);
    t += cBit;
  }
  portEXIT_CRITICAL(&dshotMux);
}
#endif

const char* escModeName() {
#if ESC_IS_DSHOT
  static char n[40];
  snprintf(n, sizeof(n), "DSHOT%d (%s)", DSHOT_SPEED, dshotRmt ? "RMT hardware" : "software");
  return n;
#elif ESC_PROTOCOL == 3
  return "ONESHOT125 (2 kHz)";
#elif ESC_PROTOCOL == 4
  return "ONESHOT42 (8 kHz)";
#elif ESC_PROTOCOL == 5
  return "MULTISHOT (32 kHz)";
#else
  static char n[24];
  snprintf(n, sizeof(n), "PWM (%d Hz)", ESC_PWM_HZ);
  return n;
#endif
}

#if !ESC_IS_DSHOT
// Analog ESC output: cmd on the 1000-2000 scale -> protocol pulse width
uint32_t escPulseNs(int cmd) {
  cmd = constrain(cmd, 1000, 2000);
#if   ESC_PROTOCOL == 3
  return (uint32_t)cmd * 125;                       // 125-250 us
#elif ESC_PROTOCOL == 4
  return (uint32_t)cmd * 42;                        // 42-84 us
#elif ESC_PROTOCOL == 5
  return 5000 + (uint32_t)(cmd - 1000) * 20;        // 5-25 us
#else
  return (uint32_t)cmd * 1000;                      // 1000-2000 us
#endif
}

void escAnalogBegin(const PwmOut& o) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(o.pin, ESC_HZ, ESC_RES_BITS);
#else
  ledcSetup(o.ch, ESC_HZ, ESC_RES_BITS);
  ledcAttachPin(o.pin, o.ch);
#endif
}

void escAnalogWrite(const PwmOut& o, int cmd) {
  uint64_t duty = (uint64_t)escPulseNs(cmd) * ESC_HZ * (1ULL << ESC_RES_BITS) / 1000000000ULL;
  uint32_t maxDuty = (1UL << ESC_RES_BITS) - 1;
  if (duty > maxDuty) duty = maxDuty;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(o.pin, (uint32_t)duty);
#else
  ledcWrite(o.ch, (uint32_t)duty);
#endif
}
#endif

void escBegin() {
#if ESC_IS_DSHOT
  static bool started = false;
  if (!started) {                  // RMT channels are allocated only once
    started = true;
  #if ESC_PROTOCOL == 1 && ESP_ARDUINO_VERSION_MAJOR >= 3
    dshotRmt = dshotRmtBegin();
    if (!dshotRmt) DBG_PRINTF("ESC: RMT allocation failed - using software DShot\n");
  #endif
  }
  if (!dshotRmt) {
    uint32_t mhz = getCpuFrequencyMhz();
    uint32_t bitNs = 1000000UL / DSHOT_SPEED;      // 6667 / 3333 / 1667 ns
    dshotCycBit  = mhz * bitNs / 1000;
    dshotCycOne  = dshotCycBit * 3 / 4;            // "1" high 75 %
    dshotCycZero = dshotCycBit * 3 / 8;            // "0" high 37.5 %
    pinMode(MOTOR_L_PIN, OUTPUT); digitalWrite(MOTOR_L_PIN, LOW);
    pinMode(MOTOR_R_PIN, OUTPUT); digitalWrite(MOTOR_R_PIN, LOW);
  }
#else
  escAnalogBegin(outML); escAnalogBegin(outMR);
#endif
  escValL = escValR = MOTOR_OFF_CMD;
  escLastSendUs = micros() - 1000000UL;
}

// Sends the current motor commands. force = send now (control tick),
// otherwise only as a keep-alive at DSHOT_RATE_HZ.
void escService(bool force) {
  uint32_t now = micros();
#if ESC_IS_DSHOT
  uint32_t gap = force ? 100 : (1000000UL / DSHOT_RATE_HZ);
  if (now - escLastSendUs < gap) return;
  escLastSendUs = now;
  uint16_t pa = dshotPacket(escValL), pb = dshotPacket(escValR);
  #if ESP_ARDUINO_VERSION_MAJOR >= 3
  if (dshotRmt) dshotRmtSend(pa, pb); else dshotSend2(pa, pb);
  #else
  dshotSend2(pa, pb);
  #endif
  escFrames++;
#else
  if (!force && now - escLastSendUs < 20000) return;   // hardware repeats the pulse
  escLastSendUs = now;
  escAnalogWrite(outML, escValL); escAnalogWrite(outMR, escValR);
  escFrames++;
#endif
}

void escWrite(int l, int r) { escValL = (uint16_t)l; escValR = (uint16_t)r; escService(true); }

// ---------------------------- Quaternion math -------------------------------

Quat qMul(const Quat& a, const Quat& b) {
  return { a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
           a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
           a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
           a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w };
}
Quat qConj(const Quat& q) { return { q.w, -q.x, -q.y, -q.z }; }
Quat qNorm(Quat q) {
  float n = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
  if (n < 1e-6f) return {1, 0, 0, 0};
  return { q.w/n, q.x/n, q.y/n, q.z/n };
}
Quat qAxis(int axis, float ang) {            // rotation about body axis 0/1/2
  float s = sinf(ang * 0.5f), c = cosf(ang * 0.5f);
  return { c, axis == 0 ? s : 0, axis == 1 ? s : 0, axis == 2 ? s : 0 };
}
// Quaternion that rotates unit vector a onto unit vector b (shortest arc).
Quat qFromTwoVectors(const float a[3], const float b[3]) {
  float an = sqrtf(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
  float bn = sqrtf(b[0]*b[0] + b[1]*b[1] + b[2]*b[2]);
  if (an < 1e-6f || bn < 1e-6f) return {1, 0, 0, 0};
  float ax = a[0]/an, ay = a[1]/an, az = a[2]/an;
  float bx = b[0]/bn, by = b[1]/bn, bz = b[2]/bn;
  float d = ax*bx + ay*by + az*bz;
  if (d >= 0.999999f) return {1, 0, 0, 0};
  if (d <= -0.999999f) {                     // 180 deg: pick any perpendicular axis
    float px = 1, py = 0, pz = 0;
    if (fabsf(ax) > 0.9f) { px = 0; py = 1; pz = 0; }
    float cx = ay*pz - az*py;
    float cy = az*px - ax*pz;
    float cz = ax*py - ay*px;
    float cn = sqrtf(cx*cx + cy*cy + cz*cz);
    if (cn < 1e-6f) return {1, 0, 0, 0};
    return { 0, cx/cn, cy/cn, cz/cn };
  }
  float cx = ay*bz - az*by;
  float cy = az*bx - ax*bz;
  float cz = ax*by - ay*bx;
  return qNorm({ 1.0f + d, cx, cy, cz });
}
Mat3 qToMat(const Quat& q) {
  Mat3 R;
  float w=q.w, x=q.x, y=q.y, z=q.z;
  R.m[0][0]=1-2*(y*y+z*z); R.m[0][1]=2*(x*y-w*z);   R.m[0][2]=2*(x*z+w*y);
  R.m[1][0]=2*(x*y+w*z);   R.m[1][1]=1-2*(x*x+z*z); R.m[1][2]=2*(y*z-w*x);
  R.m[2][0]=2*(x*z-w*y);   R.m[2][1]=2*(y*z+w*x);   R.m[2][2]=1-2*(x*x+y*y);
  return R;
}

// Body-frame rotation vector (rad) that takes the current attitude to target
void attitudeError(const Quat& q, const Quat& qd, float e[3]) {
  Quat qe = qMul(qConj(q), qd);
  if (qe.w < 0) { qe.w = -qe.w; qe.x = -qe.x; qe.y = -qe.y; qe.z = -qe.z; }
  float s = sqrtf(qe.x*qe.x + qe.y*qe.y + qe.z*qe.z);
  if (s < 1e-6f) { e[0] = 2*qe.x; e[1] = 2*qe.y; e[2] = 2*qe.z; return; }
  float ang = 2.0f * atan2f(s, qe.w);
  e[0] = qe.x / s * ang; e[1] = qe.y / s * ang; e[2] = qe.z / s * ang;
}

float wrapPi(float a) {
  while (a >  PI) a -= 2 * PI;
  while (a < -PI) a += 2 * PI;
  return a;
}

// ---------------------------- IMU + AHRS ------------------------------------
Quat  qAtt = {0.70710678f, 0, 0.70710678f, 0};   // start: nose up
float gyroBias[3] = {0, 0, 0};                   // body frame, rad/s
float gyro[3]  = {0, 0, 0};                      // body rad/s (bias removed)
float accel[3] = {0, 0, 0};                      // body m/s^2

// Sensor -> body rotation. Rows = body X (nose), Y (right wing), Z (belly)
// expressed in sensor axes. Default = board flat, sensor +Y to nose; replaced
// by the learned mount at boot.
float MNT[3][3] = { {0, 1, 0}, {1, 0, 0}, {0, 0, -1} };

// IMU sampling.
//  IMU_DRIVER 1: built-in SHTP driver reads the BNO085 reports directly. Every
//    report is a real new measurement; a new GYRO report = one control tick.
//  IMU_DRIVER 0: CodeCell library; data only refreshes inside myCodeCell.Run().
float    imuA[3] = {0,0,0}, imuG[3] = {0,0,0};    // latest sample, sensor axes (m/s^2, deg/s)
uint32_t imuTicks = 0;                            // control ticks
uint32_t imuAccUpdates = 0, imuGyroUpdates = 0;   // new accel / gyro samples
uint32_t imuLastTickUs = 0, imuLastTickMs = 0;
uint32_t imuResets = 0;
uint32_t imuGlitches = 0;                         // physically impossible reports dropped
#define IMU_MAX_DPS  2100.0f                      // BNO085 gyro range is +/-2000 dps
#define IMU_MAX_ACC  80.0f                        // BNO085 accel range is +/-8 g

#if IMU_DRIVER == 1
// ---------------------------------------------------------------------------
// Minimal SHTP / SH-2 driver for the onboard BNO085 (I2C, polled, no INT pin)
//   SHTP header: [len LSB][len MSB (bit15 = continuation)][channel][seq]
//   Every I2C read returns a fresh 4-byte header followed by the next cargo.
// ---------------------------------------------------------------------------
#define SHTP_CH_COMMAND   0
#define SHTP_CH_EXEC      1
#define SHTP_CH_CONTROL   2
#define SHTP_CH_REPORTS   3
#define SHTP_MAX_RX       512
#define SHTP_I2C_CHUNK    128          // ESP32 Wire buffer
#define BNO_POLL_US       400          // min. time between I2C polls
#define BNO_BUS_ERR_WINDOW_MS  100     // I2C error window for bus recovery
#define BNO_BUS_ERR_LIMIT      8       // errors in one window -> re-init the bus
#define BNO_RID_ACCEL     0x01         // accelerometer (m/s^2, Q8)
#define BNO_RID_GYRO_CAL  0x02         // calibrated gyroscope (rad/s, Q9)

uint8_t  shtpSeq[6] = {0, 0, 0, 0, 0, 0};
uint8_t  shtpRx[SHTP_MAX_RX];
uint16_t shtpRxLen  = 0;
uint8_t  shtpRxChan = 0;
uint32_t bnoLastPollUs = 0;
bool     bnoResetSeen = false, bnoGotGyro = false;
uint32_t bnoI2cErrors = 0;
uint32_t bnoI2cErrMark = 0, bnoI2cErrWindowMs = 0;
uint32_t imuBusRecoveries = 0;

bool shtpSend(uint8_t chan, const uint8_t* data, uint8_t len) {
  uint16_t total = len + 4;
  Wire.beginTransmission(BNO_ADDR);
  Wire.write((uint8_t)(total & 0xFF));
  Wire.write((uint8_t)(total >> 8));
  Wire.write(chan);
  Wire.write(shtpSeq[chan]++);
  for (uint8_t i = 0; i < len; i++) Wire.write(data[i]);
  return Wire.endTransmission() == 0;
}

// Read one SHTP packet into shtpRx. true = packet with cargo received.
bool shtpReceive() {
  if (Wire.requestFrom((int)BNO_ADDR, 4) != 4) {
    while (Wire.available()) Wire.read();
    bnoI2cErrors++;
    return false;
  }
  uint8_t h[4];
  for (int i = 0; i < 4; i++) h[i] = Wire.read();
  uint16_t total = (((uint16_t)h[1] << 8) | h[0]) & 0x7FFF;
  if (total <= 4 || total == 0x7FFF) return false;          // nothing waiting
  uint16_t remaining = total - 4;
  shtpRxChan = h[2];
  shtpRxLen  = 0;
  while (remaining > 0) {
    uint16_t n = remaining < (SHTP_I2C_CHUNK - 4) ? remaining : (SHTP_I2C_CHUNK - 4);
    if (Wire.requestFrom((int)BNO_ADDR, (int)(n + 4)) != (int)(n + 4)) {
      while (Wire.available()) Wire.read();
      bnoI2cErrors++;
      return false;
    }
    for (int i = 0; i < 4; i++) Wire.read();                 // repeated header
    for (uint16_t i = 0; i < n; i++) {
      uint8_t b = Wire.read();
      if (shtpRxLen < SHTP_MAX_RX) shtpRx[shtpRxLen++] = b;
    }
    remaining -= n;
  }
  return true;
}

int16_t rdI16(const uint8_t* p) { return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8)); }

// Decode every sensor report in the packet (several may be batched together)
void bnoParse() {
  if (shtpRxChan == SHTP_CH_EXEC) {
    if (shtpRxLen >= 1 && shtpRx[0] == 0x01) bnoResetSeen = true;   // reset complete
    return;
  }
  if (shtpRxChan != SHTP_CH_REPORTS) return;
  uint16_t i = 0;
  while (i < shtpRxLen) {
    uint8_t id = shtpRx[i], sz;
    switch (id) {
      case 0xFB: case 0xFA:                         sz = 5;  break;   // timestamps
      case 0x01: case 0x02: case 0x03: case 0x04:
      case 0x06:                                    sz = 10; break;
      case 0x08:                                    sz = 12; break;
      case 0x05: case 0x09:                         sz = 14; break;
      case 0x07: case 0x0F:                         sz = 16; break;
      default: return;                                              // unknown: stop
    }
    if (i + sz > shtpRxLen) return;
    const uint8_t* r = &shtpRx[i];
    if (id == BNO_RID_ACCEL) {
      float ax = rdI16(r + 4) / 256.0f, ay = rdI16(r + 6) / 256.0f, az = rdI16(r + 8) / 256.0f;
      if (fabsf(ax) > IMU_MAX_ACC || fabsf(ay) > IMU_MAX_ACC || fabsf(az) > IMU_MAX_ACC) {
        imuGlitches++;                                // corrupt report - drop it
      } else {
        imuA[0] = ax; imuA[1] = ay; imuA[2] = az;
        imuAccUpdates++;
      }
    } else if (id == BNO_RID_GYRO_CAL) {
      float gx = rdI16(r + 4) / 512.0f * R2D;       // rad/s -> deg/s
      float gy = rdI16(r + 6) / 512.0f * R2D;
      float gz = rdI16(r + 8) / 512.0f * R2D;
      if (fabsf(gx) > IMU_MAX_DPS || fabsf(gy) > IMU_MAX_DPS || fabsf(gz) > IMU_MAX_DPS) {
        imuGlitches++;                                // corrupt report - drop it
      } else {
        imuG[0] = gx; imuG[1] = gy; imuG[2] = gz;
        imuGyroUpdates++;
        bnoGotGyro = true;
      }
    }
    i += sz;
  }
}

bool bnoSetFeature(uint8_t reportId, uint32_t intervalUs) {
  uint8_t c[17] = {0};
  c[0] = 0xFD;                                       // Set Feature command
  c[1] = reportId;
  c[5] = intervalUs & 0xFF;  c[6] = (intervalUs >> 8) & 0xFF;
  c[7] = (intervalUs >> 16) & 0xFF; c[8] = (intervalUs >> 24) & 0xFF;
  return shtpSend(SHTP_CH_CONTROL, c, sizeof(c));
}

bool bnoEnableReports() {
  bool ok = bnoSetFeature(BNO_RID_GYRO_CAL, BNO_GYRO_MS * 1000UL);
  delay(10);
  ok &= bnoSetFeature(BNO_RID_ACCEL, BNO_ACCEL_MS * 1000UL);
  delay(10);
  return ok;
}

void bnoDrain(uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) { if (shtpReceive()) bnoParse(); else svcDelay(2); }
}

// Re-initialise the I2C peripheral after a burst of bus errors. The BNO085
// keeps its report configuration across an ESP32-side bus reset; if it did
// reset, the existing bnoResetSeen path re-enables reports.
void imuBusRecover() {
  Wire.end();
  Wire.begin(BNO_SDA, BNO_SCL);
  Wire.setClock(400000);
  Wire.setTimeOut(10);
  bnoLastPollUs = micros();
  imuBusRecoveries++;
  DBG_PRINTF("I2C bus recovery #%lu (%lu errors)\n",
             (unsigned long)imuBusRecoveries, (unsigned long)bnoI2cErrors);
}

bool imuBegin() {
  Wire.begin(BNO_SDA, BNO_SCL);
  Wire.setClock(400000);
  Wire.setTimeOut(10);                 // bound a stuck transaction; never block the loop
  bnoI2cErrMark = bnoI2cErrors;
  bnoI2cErrWindowMs = millis();
  for (int attempt = 1; attempt <= 5; attempt++) {
    Wire.beginTransmission(BNO_ADDR);
    if (Wire.endTransmission() != 0) {
      DBG_PRINTF("BNO085 not responding at 0x%02X (attempt %d)\n", BNO_ADDR, attempt);
      svcDelay(200);
      continue;
    }
    uint8_t rst = 1;
    shtpSend(SHTP_CH_EXEC, &rst, 1);                 // soft reset
    svcDelay(100);
    bnoDrain(600);                                   // adverts + reset-complete
    bnoResetSeen = false;
    if (!bnoEnableReports()) { svcDelay(200); continue; }
    uint32_t g0 = imuGyroUpdates, a0 = imuAccUpdates, t0 = millis();
    while (millis() - t0 < 1000 && (imuGyroUpdates - g0 < 20 || imuAccUpdates - a0 < 10)) {
      if (shtpReceive()) bnoParse(); else { idleService(); delayMicroseconds(300); }
    }
    if (imuGyroUpdates - g0 >= 20 && imuAccUpdates - a0 >= 10) return true;
    DBG_PRINTF("BNO085 reports not arriving (gyro %lu, accel %lu) - retrying\n",
               (unsigned long)(imuGyroUpdates - g0), (unsigned long)(imuAccUpdates - a0));
  }
  return false;
}

bool imuService() {
  uint32_t now = micros();
  if (now - bnoLastPollUs < BNO_POLL_US) return false;
  bnoLastPollUs = now;
  // A burst of I2C errors means the bus is wedged or a wire is loose; re-init
  // the peripheral instead of letting the control loop time out.
  if (millis() - bnoI2cErrWindowMs >= BNO_BUS_ERR_WINDOW_MS) {
    uint32_t errs = bnoI2cErrors - bnoI2cErrMark;
    bnoI2cErrMark = bnoI2cErrors;
    bnoI2cErrWindowMs = millis();
    if (errs >= BNO_BUS_ERR_LIMIT) imuBusRecover();
  }
  bnoGotGyro = false;
  for (int i = 0; i < 4; i++) {                      // drain a few queued packets
    if (!shtpReceive()) break;
    bnoParse();
  }
  if (bnoResetSeen) {                                // sensor rebooted itself
    bnoResetSeen = false;
    imuResets++;
    DBG_PRINTF("BNO085 reset detected (#%lu) - re-enabling reports\n", (unsigned long)imuResets);
    bnoEnableReports();
  }
  if (!bnoGotGyro) return false;
  imuTicks++;
  imuLastTickUs = micros(); imuLastTickMs = millis();
  return true;
}

#else   // IMU_DRIVER 0 - CodeCell library
float imuLastA[3] = {0,0,0}, imuLastG[3] = {0,0,0};

bool imuBegin() {
  myCodeCell.Init(MOTION_ACCELEROMETER + MOTION_GYRO);
  return true;
}

bool imuService() {
  if (!myCodeCell.Run(IMU_RUN_HZ)) return false;
  myCodeCell.Motion_AccelerometerRead(imuA[0], imuA[1], imuA[2]);   // m/s^2
  myCodeCell.Motion_GyroRead(imuG[0], imuG[1], imuG[2]);            // deg/s
  imuTicks++;
  imuLastTickUs = micros(); imuLastTickMs = millis();
  if (imuA[0] != imuLastA[0] || imuA[1] != imuLastA[1] || imuA[2] != imuLastA[2]) {
    imuAccUpdates++; for (int k = 0; k < 3; k++) imuLastA[k] = imuA[k];
  }
  if (imuG[0] != imuLastG[0] || imuG[1] != imuLastG[1] || imuG[2] != imuLastG[2]) {
    imuGyroUpdates++; for (int k = 0; k < 3; k++) imuLastG[k] = imuG[k];
  }
  ledState = -1;            // Run() may drive the LED itself - re-assert ours
  ledUpdate();
  return true;
}
#endif

// Block until the next fresh IMU sample (keeps LED alive). false on timeout.
bool imuWaitSample(uint32_t timeoutMs) {
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    idleService();
    if (imuService()) return true;
    ledUpdate();
    delayMicroseconds(200);
  }
  return false;
}

void readImuSensor(float g[3], float a[3]) {
  for (int k = 0; k < 3; k++) { g[k] = imuG[k]; a[k] = imuA[k]; }
}

void readImuBody(float g[3], float a[3]) {
  float s[3], t[3];
  readImuSensor(s, t);
  for (int i = 0; i < 3; i++) {
    g[i] = (MNT[i][0] * s[0] + MNT[i][1] * s[1] + MNT[i][2] * s[2]) * D2R;
    a[i] =  MNT[i][0] * t[0] + MNT[i][1] * t[1] + MNT[i][2] * t[2];
  }
}

// ---------------------------- Mount setup -----------------------------------
float vNorm3(const float v[3]) { return sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]); }
float vAngleDeg(const float a[3], const float b[3]) {
  float d = (a[0]*b[0] + a[1]*b[1] + a[2]*b[2]) / (vNorm3(a) * vNorm3(b) + 1e-9f);
  return acosf(constrain(d, -1.0f, 1.0f)) * R2D;
}

// ---------------------------- Calibration storage ---------------------------
// CRC32 (IEEE) over the blob; catches truncated, old or corrupt writes.
uint32_t calCrc32(const void* data, size_t len) {
  const uint8_t* p = (const uint8_t*)data;
  uint32_t crc = 0xFFFFFFFFUL;
  for (size_t i = 0; i < len; i++) {
    crc ^= p[i];
    for (int k = 0; k < 8; k++)
      crc = (crc >> 1) ^ (0xEDB88320UL & (uint32_t)(-(int32_t)(crc & 1)));
  }
  return ~crc;
}

bool calBlobValid(const CalBlob& b) {
  if (b.magic != CAL_MAGIC || b.version != CAL_VERSION || b.size != sizeof(CalBlob))
    return false;
  if (calCrc32(&b, offsetof(CalBlob, crc)) != b.crc) return false;

  // MNT must be a right-handed orthonormal basis (rows = body axes).
  for (int i = 0; i < 3; i++) {
    float n = vNorm3(b.mnt[i]);
    if (!isfinite(n) || fabsf(n - 1.0f) > 0.02f) return false;
  }
  for (int i = 0; i < 3; i++)
    for (int j = i + 1; j < 3; j++) {
      float d = b.mnt[i][0]*b.mnt[j][0] + b.mnt[i][1]*b.mnt[j][1] + b.mnt[i][2]*b.mnt[j][2];
      if (fabsf(d) > 0.02f) return false;
    }
  float det = b.mnt[0][0] * (b.mnt[1][1]*b.mnt[2][2] - b.mnt[1][2]*b.mnt[2][1])
            - b.mnt[0][1] * (b.mnt[1][0]*b.mnt[2][2] - b.mnt[1][2]*b.mnt[2][0])
            + b.mnt[0][2] * (b.mnt[1][0]*b.mnt[2][1] - b.mnt[1][1]*b.mnt[2][0]);
  if (det < 0.98f) return false;

  for (int i = 0; i < 3; i++)
    if (!isfinite(b.gyroBias[i]) || fabsf(b.gyroBias[i]) > 0.5f) return false;  // > ~29 dps
  return true;
}

void calApply(const CalBlob& b) {
  memcpy(MNT, b.mnt, sizeof(MNT));
  memcpy(gyroBias, b.gyroBias, sizeof(gyroBias));
}

bool calLoad(CalBlob& b) {
  Preferences p;
  if (!p.begin(CAL_NVS_NAMESPACE, true)) return false;
  size_t len = p.getBytesLength(CAL_NVS_KEY);
  if (len != sizeof(CalBlob)) { p.end(); return false; }
  size_t got = p.getBytes(CAL_NVS_KEY, &b, sizeof(b));
  p.end();
  if (got != sizeof(b)) return false;
  return calBlobValid(b);
}

bool calSave() {
  CalBlob b;
  memset(&b, 0, sizeof(b));
  b.magic = CAL_MAGIC;
  b.version = CAL_VERSION;
  b.size = sizeof(CalBlob);
  memcpy(b.mnt, MNT, sizeof(b.mnt));
  memcpy(b.gyroBias, gyroBias, sizeof(b.gyroBias));
  b.crc = calCrc32(&b, offsetof(CalBlob, crc));

  Preferences p;
  if (!p.begin(CAL_NVS_NAMESPACE, false)) return false;
  size_t n = p.putBytes(CAL_NVS_KEY, &b, sizeof(b));
  p.end();
  return n == sizeof(b);
}

bool calErase() {
  Preferences p;
  if (!p.begin(CAL_NVS_NAMESPACE, false)) return false;
  p.remove(CAL_NVS_KEY);
  p.remove(CAL_NVS_FORCE_KEY);
  p.end();
  return true;
}

bool calForceRequested() {
  Preferences p;
  if (!p.begin(CAL_NVS_NAMESPACE, true)) return false;
  bool f = p.getBool(CAL_NVS_FORCE_KEY, false);
  p.end();
  return f;
}

void calSetForce(bool on) {
  Preferences p;
  if (!p.begin(CAL_NVS_NAMESPACE, false)) return;
  if (on) p.putBool(CAL_NVS_FORCE_KEY, true);
  else    p.remove(CAL_NVS_FORCE_KEY);
  p.end();
}

void calStatus() {
  CalBlob b;
  bool forced = calForceRequested();
  (void)forced;                     // used only when LOG_PRINTF is enabled
  if (!calLoad(b)) {
    LOG_PRINTF("Calibration: none saved%s.\n", forced ? " (forced at next boot)" : "");
    return;
  }
  LOG_PRINTF("Calibration: saved (version %u)%s\n", (unsigned)b.version,
             forced ? " - FORCED at next boot" : "");
  LOG_PRINTF("  nose  = sensor %+.3f %+.3f %+.3f\n", b.mnt[0][0], b.mnt[0][1], b.mnt[0][2]);
  LOG_PRINTF("  right = sensor %+.3f %+.3f %+.3f\n", b.mnt[1][0], b.mnt[1][1], b.mnt[1][2]);
  LOG_PRINTF("  belly = sensor %+.3f %+.3f %+.3f\n", b.mnt[2][0], b.mnt[2][1], b.mnt[2][2]);
  LOG_PRINTF("  gyro bias %+.2f %+.2f %+.2f dps\n",
             b.gyroBias[0]*R2D, b.gyroBias[1]*R2D, b.gyroBias[2]*R2D);
}

// Wait until the aircraft is held still for CAL_STILL_MS. Returns the averaged
// "up" vector (accel, SENSOR axes) and averaged gyro (deg/s, SENSOR axes).
// If awayFrom != nullptr the pose must differ from it by >= CAL_MIN_ANGLE_DEG;
// the LED switches from solid CYAN to fast MAGENTA once the aircraft has been
// rotated CAL_MOVE_AWAY_DEG away from the flat pose.
// Stale / out-of-range / brief spike samples are skipped; real movement
// restarts the still timer. Never fails - it waits.
void waitStillPose(float upOut[3], float gyroOut[3], const float* awayFrom) {
  float ref[3] = {0,0,0}, sumA[3] = {0,0,0}, sumG[3] = {0,0,0};
  int n = 0, spikes = 0;
  bool movedAway = (awayFrom == nullptr);
  uint32_t t0 = 0, lastMsg = millis(), tickAtMsg = imuTicks, accAtMsg = imuAccUpdates;
  const char* why = "waiting";
  for (;;) {
    float g[3], a[3];
    uint32_t accBefore = imuAccUpdates;
    bool tick = imuWaitSample(50);
    readImuSensor(g, a);
    bool fresh = tick && imuAccUpdates != accBefore;
    float an = vNorm3(a), gn = vNorm3(g);
    float ang = awayFrom ? vAngleDeg(a, awayFrom) : 999.0f;

    if (awayFrom && !movedAway && fresh && ang >= CAL_MOVE_AWAY_DEG) {
      movedAway = true;
      ledSetPattern(LED_CAL_VERT);                       // fast MAGENTA
    }

    if (!tick) {
      why = "no IMU data";
    } else if (!fresh) {
      // gyro-only tick (accel runs at half rate) - skip
    } else if (an < CAL_G_MIN * GRAV || an > CAL_G_MAX * GRAV) {
      why = "accel out of range (skipped)";
    } else {
      bool moving = gn > CAL_STILL_DPS || (n > 0 && vAngleDeg(a, ref) > CAL_STILL_DEG);
      if (awayFrom && ang < CAL_MIN_ANGLE_DEG) {
        why = movedAway ? "keep rotating - stand it on its TAIL, nose up"
                        : "flat captured - rotate to VERTICAL";
        n = 0; spikes = 0;
      } else if (moving) {
        if (n > 0 && ++spikes <= CAL_SPIKE_ALLOW) {
          why = "holding (spike ignored)";
        } else {
          why = (gn > CAL_STILL_DPS) ? "rotating" : "moving";
          n = 0; spikes = 0;
        }
      } else {
        spikes = 0;
        if (n == 0) {
          for (int k = 0; k < 3; k++) { ref[k] = a[k]; sumA[k] = 0; sumG[k] = 0; }
          t0 = millis();
        }
        for (int k = 0; k < 3; k++) { sumA[k] += a[k]; sumG[k] += g[k]; }
        n++;
        why = "holding";
        if (millis() - t0 >= CAL_STILL_MS && n >= 20) {
          for (int k = 0; k < 3; k++) { upOut[k] = sumA[k] / n; gyroOut[k] = sumG[k] / n; }
          return;
        }
      }
    }

    if (millis() - lastMsg >= 1000) {
      uint32_t dt = millis() - lastMsg;
      lastMsg = millis();
      DBG_PRINTF("  up %+.2f %+.2f %+.2f |a| %.2f gyro %.1f dps | IMU %lu Hz | held %.1f/%.1f s | %s\n",
                 a[0]/(an+1e-9f), a[1]/(an+1e-9f), a[2]/(an+1e-9f), an, gn,
                 (unsigned long)((imuTicks - tickAtMsg) * 1000UL / dt),
                 n ? (millis() - t0) / 1000.0f : 0.0f, CAL_STILL_MS / 1000.0f, why);
      if (awayFrom) DBG_PRINTF("     angle from flat: %.0f deg (need >= %.0f)\n", ang, CAL_MIN_ANGLE_DEG);
      (void)dt;                                   // used only when DEBUG = 1
      tickAtMsg = imuTicks; accAtMsg = imuAccUpdates;
    }
    (void)why; (void)tickAtMsg; (void)accAtMsg;     // unused when DEBUG = 0
  }
}

#if (IMU_TEST + 0)
// Raw IMU monitor: shows whether the library is delivering fresh data.
void imuTestLoop() {
  DBG_PRINTF("\n=== IMU TEST MODE ===  Move/rotate the board and watch the values.\n"
             "Expect |a| ~9.8 at rest. IMU_DRIVER 1: new gyro ~200 Hz, new acc ~100 Hz.\n");
  uint32_t lastMsg = millis(), t0 = imuTicks, acc0 = imuAccUpdates, gyr0 = imuGyroUpdates;
  for (;;) {
    imuService();
    idleService();
    ledUpdate();
    if (millis() - lastMsg >= 200) {
      float g[3], a[3];
      readImuSensor(g, a);
      uint32_t dt = millis() - lastMsg; lastMsg = millis();
      DBG_PRINTF("acc %+7.2f %+7.2f %+7.2f |a| %5.2f  gyro %+7.1f %+7.1f %+7.1f  "
                 "IMU %3lu Hz  new acc %3lu Hz  new gyro %3lu Hz  glitches %lu\n",
                 a[0], a[1], a[2], vNorm3(a), g[0], g[1], g[2],
                 (unsigned long)((imuTicks - t0) * 1000UL / dt),
                 (unsigned long)((imuAccUpdates - acc0) * 1000UL / dt),
                 (unsigned long)((imuGyroUpdates - gyr0) * 1000UL / dt),
                 (unsigned long)imuGlitches);
      t0 = imuTicks; acc0 = imuAccUpdates; gyr0 = imuGyroUpdates;
    }
    delay(1);
  }
}
#endif

float ahrsTrust = 0;                              // last accel trust (0..1), logged
// Mahony complementary filter (gyro + accel). No magnetometer: heading is
// relative and may drift slowly - fine for heading-hold and transitions.
void ahrsUpdate(const float g[3], const float a[3], float dt, float kp) {
  float w[3] = { g[0], g[1], g[2] };
  float an = sqrtf(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
  if (an > 0.1f) {
    float trust = 1.0f - fabsf(an - GRAV) / (AHRS_ACC_TRUST_BAND * GRAV);
    ahrsTrust = trust > 0 ? trust : 0;
    if (trust > 0) {
      Mat3 R = qToMat(qAtt);
      float v[3] = { -R.m[2][0], -R.m[2][1], -R.m[2][2] };  // estimated "up" in body
      float m[3] = { a[0]/an, a[1]/an, a[2]/an };          // measured "up" in body
      float e[3] = { m[1]*v[2] - m[2]*v[1], m[2]*v[0] - m[0]*v[2], m[0]*v[1] - m[1]*v[0] };
      for (int i = 0; i < 3; i++) w[i] += kp * trust * e[i];
    }
  }
  Quat dq = qMul(qAtt, Quat{0, w[0], w[1], w[2]});
  qAtt.w += 0.5f * dq.w * dt; qAtt.x += 0.5f * dq.x * dt;
  qAtt.y += 0.5f * dq.y * dt; qAtt.z += 0.5f * dq.z * dt;
  qAtt = qNorm(qAtt);
}

// Initialise qAtt from gravity using the learned mount. This is what lets a
// field boot skip the two-pose calibration: the aircraft may be in any
// attitude, and the accelerometer supplies the initial tilt.
void initAttitudeFromAccel() {
  float g[3], a[3];
  if (!imuWaitSample(500)) {
    CAL_PRINTF("WARNING: no IMU sample for attitude init - using default attitude.\n");
    return;
  }
  readImuBody(g, a);
  float n = vNorm3(a);
  if (n > 0.5f * GRAV && n < 1.5f * GRAV) {
    float m[3] = { a[0]/n, a[1]/n, a[2]/n };   // measured "up" in body
    float up[3] = { 0, 0, -1 };                // world up (NED)
    qAtt = qFromTwoVectors(m, up);
  } else {
    CAL_PRINTF("WARNING: accel %.2f g at boot - attitude init skipped.\n", n / GRAV);
  }
  // Refine while the aircraft is still; AHRS_KP_INIT converges quickly.
  for (int i = 0; i < 100; i++) {
    if (!imuWaitSample(200)) break;
    readImuBody(g, a);
    for (int k = 0; k < 3; k++) g[k] -= gyroBias[k];
    ahrsUpdate(g, a, 1.0f / IMU_RATE_HZ, AHRS_KP_INIT);
  }
}

// Attitude readouts (singularity-free, degrees)
float noseAboveHorizonDeg() { Mat3 R = qToMat(qAtt); return asinf(constrain(-R.m[2][0], -1.0f, 1.0f)) * R2D; }
float bankDeg()             { Mat3 R = qToMat(qAtt); return asinf(constrain( R.m[2][1], -1.0f, 1.0f)) * R2D; }
float tiltFromVerticalDeg() { Mat3 R = qToMat(qAtt); return acosf(constrain(-R.m[2][0], -1.0f, 1.0f)) * R2D; }
// Heading of the flight direction: belly direction in hover, nose in forward
float headingRad(float tau) {
  Mat3 R = qToMat(qAtt);
  float hx = tau * R.m[0][0] + (1 - tau) * R.m[0][2];
  float hy = tau * R.m[1][0] + (1 - tau) * R.m[1][2];
  return atan2f(hy, hx);
}

// Two-pose calibration:
//   Step 1 FLAT on belly  -> up = body -Z      (fast CYAN, solid when captured)
//   Step 2 VERTICAL       -> up = body +X nose (fast MAGENTA, solid when captured)
// From the two poses: sensor->body rotation (nose + overall orientation),
// gyro bias (measured while still on the tail) and initial attitude.
// calibrateAndSave() stores the result in NVS for future field boots.
void bootCalibrate() {
  float upF[3], gF[3], upV[3], gV[3];
  CAL_PRINTF("\n=== CALIBRATION ===\n"
             "Step 1: lay the aircraft FLAT on its BELLY, wings level, hands off. (fast CYAN)\n");
  ledSetPattern(LED_CAL_FLAT);
  waitStillPose(upF, gF, nullptr);
  ledSetPattern(LED_CAL_FLAT_OK);                       // solid CYAN
  CAL_PRINTF("Step 1 captured (solid CYAN).\n"
             "Step 2: stand the aircraft VERTICAL on its TAIL, nose up, hands off. (fast MAGENTA)\n");
  waitStillPose(upV, gV, upF);
  float poseAngle = vAngleDeg(upF, upV);
  if (poseAngle < CAL_WARN_ANGLE_DEG) {
    CAL_PRINTF("WARNING: vertical pose captured at %.0f deg (target 90, minimum %.0f).\n"
               "  A rolled or leaning hold skews the mount matrix - redo calibration if the\n"
               "  nose was not straight up or the wings were not vertical.\n",
               poseAngle, CAL_MIN_ANGLE_DEG);
  }

  float bx[3], by[3], bz[3];
  float nf = vNorm3(upF);
  for (int k = 0; k < 3; k++) bz[k] = -upF[k] / nf;              // belly = -up(flat)
  float d = upV[0]*bz[0] + upV[1]*bz[1] + upV[2]*bz[2];
  for (int k = 0; k < 3; k++) bx[k] = upV[k] - d * bz[k];        // nose = up(vertical), orthogonalised
  float nx = vNorm3(bx);
  for (int k = 0; k < 3; k++) bx[k] /= nx;
  by[0] = bz[1]*bx[2] - bz[2]*bx[1];                             // right wing = Z x X
  by[1] = bz[2]*bx[0] - bz[0]*bx[2];
  by[2] = bz[0]*bx[1] - bz[1]*bx[0];
  for (int k = 0; k < 3; k++) { MNT[0][k] = bx[k]; MNT[1][k] = by[k]; MNT[2][k] = bz[k]; }

  for (int i = 0; i < 3; i++)
    gyroBias[i] = (MNT[i][0] * gV[0] + MNT[i][1] * gV[1] + MNT[i][2] * gV[2]) * D2R;

  ledSetPattern(LED_CAL_VERT_OK);                       // solid MAGENTA
  uint32_t tDone = millis();

  initAttitudeFromAccel();                              // qAtt from gravity, then refine
  CAL_PRINTF("Step 2 captured (solid MAGENTA). Angle flat->vertical %.0f deg\n"
             "  nose  = sensor %+.2f %+.2f %+.2f\n"
             "  right = sensor %+.2f %+.2f %+.2f\n"
             "  belly = sensor %+.2f %+.2f %+.2f\n"
             "  gyro bias %.2f %.2f %.2f dps, nose %.1f deg above horizon, %lu IMU glitches\n",
             poseAngle, bx[0], bx[1], bx[2], by[0], by[1], by[2], bz[0], bz[1], bz[2],
             gyroBias[0]*R2D, gyroBias[1]*R2D, gyroBias[2]*R2D, noseAboveHorizonDeg(),
             (unsigned long)imuGlitches);
  uint32_t el = millis() - tDone;
  if (el < CAL_DONE_SOLID_MS) ledDelay(CAL_DONE_SOLID_MS - el);
}

// Run the two-pose calibration and persist it. If NVS fails the calibration
// still applies to this power-up, so the aircraft is never left uncalibrated.
void calibrateAndSave() {
  bootCalibrate();
  if (calSave()) CAL_PRINTF("Calibration saved to NVS.\n");
  else          CAL_PRINTF("WARNING: could not save calibration - it applies to this power-up only.\n");
}

// ---------------------------- Rate PID --------------------------------------
struct RatePID {
  float integ = 0, dFilt = 0, prevMeas = 0;
  bool  first = true;
  float update(float sp, float meas, float kp, float ki, float kd, float dt, bool freezeI) {
    float err = sp - meas;
    if (!freezeI) integ = constrain(integ + ki * err * dt, -I_LIMIT, I_LIMIT);
    if (first) { prevMeas = meas; dFilt = 0; first = false; }
    float d = (meas - prevMeas) / dt; prevMeas = meas;
    dFilt += D_LPF * (d - dFilt);
    return constrain(kp * err + integ - kd * dFilt, -1.0f, 1.0f);
  }
  void reset() { integ = 0; first = true; }
};
RatePID pid[3];

// ---------------------------- Helpers ---------------------------------------
float stickNorm(uint16_t us, bool rev) {
  float v = constrain(((float)us - 1500.0f) / 500.0f, -1.0f, 1.0f);
  if (fabsf(v) < RC_DEADBAND) v = 0;
  v = v * (1.0f - RC_EXPO) + v * v * v * RC_EXPO;
  return rev ? -v : v;
}
float throttleNorm(uint16_t us) { return constrain(((float)us - 1000.0f) / 1000.0f, 0.0f, 1.0f); }
float lerpf(float a, float b, float t) { return a + (b - a) * t; }
int elevonUs(float cmd, int center, bool rev) {
  if (!isfinite(cmd)) cmd = 0.0f;   // never send an indeterminate value to a servo
  cmd = constrain(cmd, -1.0f, 1.0f);
  return constrain(center + (int)((rev ? -cmd : cmd) * SERVO_THROW_US), SERVO_MIN_US, SERVO_MAX_US);
}
// Armed motor command for throttle fraction m (0..1): DShot value or PWM us
int motorCmd(float m) {
  if (!isfinite(m)) m = 0.0f;      // never send an indeterminate value to an ESC
  m = constrain(m, 0.0f, 1.0f);
#if ESC_IS_DSHOT
  return constrain((int)(DSHOT_IDLE + m * (DSHOT_MAX - DSHOT_IDLE)), DSHOT_IDLE, DSHOT_MAX);
#else
  return constrain((int)(MOTOR_IDLE_US + m * (MOTOR_MAX_US - MOTOR_IDLE_US)), MOTOR_IDLE_US, MOTOR_MAX_US);
#endif
}

// ---------------------------- Flight state ----------------------------------
const char* stateName[] = {"DISARMED", "HOVER", "TO_FWD", "FORWARD", "TO_HOVER", "FAILSAFE"};

FlightState state = ST_DISARMED;
bool  armed = false;
bool  armSwitchWasLow = false;
float tau = 0;                 // 0 = hover, 1 = forward
float headingTarget = 0;       // rad
uint32_t failsafeStartMs = 0, crashStartMs = 0;
uint32_t armMs = 0, fwdStartMs = 0;   // arm time / forward-transition throttle-hold timer
float armStillSec = 0;                // seconds the gyro has been below ARM_MAX_RATE_DPS
uint32_t lastLoopUs = 0;
float loopHz = 0;

// Magnitude (deg) of the attitude error to the current flight target. Mirrors
// the target construction in loop() and is used only by crash detection.
float attitudeErrorDeg(float tauNow, float rStk, float pStk) {
  float pitchDeg = lerpf(90.0f, FWD_PITCH_DEG, tauNow) +
                   pStk * lerpf(HOVER_MAX_TILT_DEG, FWD_MAX_PITCH_DEG, tauNow);
  float bank = rStk * FWD_MAX_BANK_DEG * tauNow;
  float lean = rStk * HOVER_MAX_TILT_DEG * (1.0f - tauNow);
  Quat qd = qMul(qMul(qAxis(2, headingTarget), qAxis(1, pitchDeg * D2R)),
                 qMul(qAxis(0, bank * D2R), qAxis(2, lean * D2R)));
  float e[3];
  attitudeError(qAtt, qd, e);
  return sqrtf(e[0]*e[0] + e[1]*e[1] + e[2]*e[2]) * R2D;
}

// ============================================================================
// ONBOARD FLIGHT LOG
//   Records 48-byte binary samples at LOG_HZ to the "spiffs" data partition
//   (raw flash, no file system) whenever the aircraft is ARMED. A new session
//   number starts at every arm. Flash is pre-erased while disarmed, so in
//   flight each sample is one small flash write (no erase stalls).
//   Commands (Serial Monitor / terminal, 115200, while DISARMED):
//     help | list | dump | dump <session> | erase
// ============================================================================
// Extra per-tick values captured in loop() for the log
struct LogExtra { float thrStk; bool armSw, fwdSw, landed; float rateSp[3]; };
LogExtra logX = {0, false, false, false, {0, 0, 0}};

#define LOG_MAGIC        0xA55B           // v20 (64-byte records)
#define LOG_FULL_WARN    0.95f            // LED warns above this fill level
#define LOG_DIV          ((IMU_RATE_HZ / LOG_HZ) > 0 ? (IMU_RATE_HZ / LOG_HZ) : 1)

const char* logEventName[] = { "", "ARM", "DISARM_SWITCH", "DISARM_FAILSAFE_TIMEOUT",
                               "DISARM_CRASH", "DISARM_IMU_LOST", "FAILSAFE", "LINK_RESTORED" };


const esp_partition_t* logPart = nullptr;
uint32_t logWritePos = 0;          // byte offset of next free record
uint32_t logCapacity = 0;          // usable bytes (multiple of record size)
uint16_t logSession  = 0;          // current / last session number
bool     logReady    = false;      // partition found and erased ahead
bool     logFull     = false;
uint32_t logWriteErrors = 0;
uint8_t  logDecim = 0;

uint8_t logChecksum(const LogRec& r) {
  const uint8_t* p = (const uint8_t*)&r;
  uint8_t c = 0;
  for (int i = 0; i < LOG_REC_SIZE - 1; i++) c ^= p[i];
  return c;
}
bool logRecValid(const LogRec& r) { return r.magic == LOG_MAGIC && r.csum == logChecksum(r); }
bool logRecEmpty(const uint8_t* p) {
  for (int i = 0; i < LOG_REC_SIZE; i++) if (p[i] != 0xFF) return false;
  return true;
}
float logFill() { return logCapacity ? (float)logWritePos / logCapacity : 1.0f; }

// Erase in 64 KB chunks, keeping ESC keep-alive / servos / LED running between
void logEraseRange(uint32_t off, uint32_t len) {
  ledSetPattern(LED_LOG_BUSY);
  while (len > 0) {
    uint32_t n = len > 65536UL ? 65536UL : len;
    esp_partition_erase_range(logPart, off, n);
    off += n; len -= n;
    ledUpdate(); idleService();
  }
}

void logEraseAll() {
  if (!logPart) return;
  logEraseRange(0, logPart->size);
  logWritePos = 0; logFull = false; logReady = true;
}

// Find the end of the log, the highest session number, and make sure the
// space after the end is erased (repairs a log cut by a power loss).
void logBegin() {
  logPart = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, NULL);
  if (!logPart) {
    DBG_PRINTF("LOG: no SPIFFS partition - select Partition Scheme \"Default 4MB with spiffs\". Logging OFF.\n");
    return;
  }
  logCapacity = (logPart->size / LOG_REC_SIZE) * LOG_REC_SIZE;

  static uint8_t buf[LOG_REC_SIZE * 64];          // 4096 bytes
  LogRec first;
  esp_partition_read(logPart, 0, &first, LOG_REC_SIZE);
  if (!logRecValid(first) && !logRecEmpty((uint8_t*)&first)) {
    DBG_PRINTF("LOG: partition holds foreign data - formatting once.\n");
    logEraseAll();
    return;
  }

  // 1) scan for the first empty record
  uint32_t pos = 0;
  bool found = false;
  while (pos < logCapacity && !found) {
    uint32_t n = min((uint32_t)sizeof(buf), logCapacity - pos);
    esp_partition_read(logPart, pos, buf, n);
    for (uint32_t i = 0; i + LOG_REC_SIZE <= n; i += LOG_REC_SIZE) {
      if (logRecEmpty(buf + i)) { pos += i; found = true; break; }
      LogRec r; memcpy(&r, buf + i, LOG_REC_SIZE);
      if (logRecValid(r) && r.session > logSession) logSession = r.session;
    }
    if (!found) pos += n - (n % LOG_REC_SIZE);
  }
  logWritePos = pos;

  // 2) verify everything after the end is erased (0xFF)
  bool clean = true;
  for (uint32_t p = pos; p < logPart->size && clean; ) {
    uint32_t n = min((uint32_t)sizeof(buf), logPart->size - p);
    esp_partition_read(logPart, p, buf, n);
    for (uint32_t i = 0; i < n; i++) if (buf[i] != 0xFF) { clean = false; break; }
    p += n;
  }
  if (!clean) {
    // jump to the next 4 KB sector and erase from there to the end
    uint32_t sec = (pos + 4095) & ~4095UL;
    DBG_PRINTF("LOG: repairing unfinished log - erasing from %lu KB... ", (unsigned long)(sec / 1024));
    if (sec < logPart->size) logEraseRange(sec, logPart->size - sec);
    // first record boundary at/after the sector start
    logWritePos = ((sec + LOG_REC_SIZE - 1) / LOG_REC_SIZE) * LOG_REC_SIZE;
    DBG_PRINTF("done.\n");
  }
  logFull  = logWritePos + LOG_REC_SIZE > logCapacity;
  logReady = true;
  DBG_PRINTF("LOG: %lu KB used of %lu KB (%.0f%%), %u session(s), ~%.1f min free at %d Hz\n",
             (unsigned long)(logWritePos / 1024), (unsigned long)(logCapacity / 1024),
             logFill() * 100.0f, logSession,
             (logCapacity - logWritePos) / (float)LOG_REC_SIZE / LOG_HZ / 60.0f, LOG_HZ);
}

void logWrite(LogRec& r) {
  if (!logReady || logFull) return;
  if (logWritePos + LOG_REC_SIZE > logCapacity) { logFull = true; return; }
  r.magic = LOG_MAGIC;
  r.session = logSession;
  r.csum = logChecksum(r);
  if (esp_partition_write(logPart, logWritePos, &r, LOG_REC_SIZE) != ESP_OK) logWriteErrors++;
  logWritePos += LOG_REC_SIZE;
}

int16_t q10(float v) { return (int16_t)constrain(lroundf(v * 10.0f), -32767L, 32767L); }
int8_t  q100(float v) { return (int8_t)constrain(lroundf(v * 100.0f), -127L, 127L); }

void logBuild(LogRec& r, uint8_t type, uint8_t ev, const float e[3], const float u[3],
             float sR, float sP, float sY, float thr, bool rcOk, int sL, int sRu, int mL, int mR) {
  memset(&r, 0, sizeof(r));
  r.flags = (uint8_t)(state & 0x07) | (armed ? 0x08 : 0) | (rcOk ? 0x10 : 0) | ((type & 0x03) << 5);
  r.tau = (uint8_t)lroundf(constrain(tau, 0.0f, 1.0f) * 250.0f);
  r.tMs = millis();
  r.lq = (int8_t)constrain((int)linkLQ, -1, 127);
  r.loopHz2 = (uint8_t)constrain((int)(loopHz / 2), 0, 255);
  r.nose = q10(noseAboveHorizonDeg()); r.bank = q10(bankDeg()); r.hdg = q10(headingRad(tau) * R2D);
  r.stk[0] = q100(sR); r.stk[1] = q100(sP); r.stk[2] = q100(sY);
  r.thr = (uint8_t)lroundf(constrain(thr, 0.0f, 1.0f) * 250.0f);
  for (int k = 0; k < 3; k++) { r.err[k] = q10(e[k] * R2D); r.out[k] = q100(u[k]); r.gyr[k] = q10(gyro[k] * R2D); }
  r.servoL = sL; r.servoR = sRu; r.motorL = mL; r.motorR = mR;
  r.glitches = (uint8_t)(imuGlitches & 0xFF);
  r.ch5 = rcUs[CH_ARM]; r.ch6 = rcUs[CH_MODE];
  r.thrStk = (uint8_t)lroundf(constrain(logX.thrStk, 0.0f, 1.0f) * 250.0f);
  r.flags2 = (logX.armSw ? 1 : 0) | (logX.fwdSw ? 2 : 0) | (logX.landed ? 4 : 0) | (rcFailsafeFlag ? 8 : 0)
           | ((uint8_t)lroundf(constrain(ahrsTrust, 0.0f, 1.0f) * 15.0f) << 4);
  { float an = sqrtf(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]) / GRAV;
    r.accG = (uint8_t)lroundf(constrain(an, 0.0f, 5.1f) * 50.0f); }
  for (int k = 0; k < 3; k++) { r.rateSp[k] = q10(logX.rateSp[k] * R2D); r.iTerm[k] = q100(pid[k].integ); }
  r.event = ev;
}

// Event record (arm, disarm reason, failsafe ...) - outputs not available here
void logEvent(uint8_t ev) {
  if (ev == EV_ARM) { logSession++; logDecim = 0; }
  LogRec r;
  const float z[3] = {0, 0, 0};
  logBuild(r, LOGT_EVENT, ev, z, z, 0, 0, 0, 0, true, 0, 0, 0, 0);
  logWrite(r);
}

// Called every control tick; writes a data record at LOG_HZ while armed
void logSample(const float e[3], const float u[3], float sR, float sP, float sY, float thr,
               bool rcOk, int sL, int sRu, int mL, int mR) {
  if (!armed) return;
  if (++logDecim < LOG_DIV) return;
  logDecim = 0;
  LogRec r;
  logBuild(r, LOGT_DATA, EV_NONE, e, u, sR, sP, sY, thr, rcOk, sL, sRu, mL, mR);
  logWrite(r);
}

// ---------------------------- Log commands ----------------------------------
const char* logStateName(uint8_t s) {
  static const char* n[] = {"DISARMED", "HOVER", "TO_FWD", "FORWARD", "TO_HOVER", "FAILSAFE", "?", "?"};
  return n[s & 7];
}

void logList() {
  if (!logPart) { LOG_PRINTF("Logging unavailable (no partition).\n"); return; }
  LOG_PRINTF("Flight log: %lu KB used of %lu KB (%.0f%%)%s\n",
             (unsigned long)(logWritePos / 1024), (unsigned long)(logCapacity / 1024),
             logFill() * 100.0f, logFull ? "  ** FULL **" : "");
  LOG_PRINTF("session  start_ms    duration_s  samples  last_event\n");
  static uint8_t buf[LOG_REC_SIZE * 64];
  uint16_t cur = 0; uint32_t t0 = 0, t1 = 0, cnt = 0; uint8_t lastEv = 0;
  uint32_t bad = 0;
  for (uint32_t pos = 0; pos < logWritePos; ) {
    uint32_t n = min((uint32_t)sizeof(buf), logWritePos - pos);
    esp_partition_read(logPart, pos, buf, n);
    for (uint32_t i = 0; i + LOG_REC_SIZE <= n; i += LOG_REC_SIZE) {
      LogRec r; memcpy(&r, buf + i, LOG_REC_SIZE);
      if (!logRecValid(r)) { bad++; continue; }
      if (r.session != cur) {
        if (cnt) LOG_PRINTF("%7u  %9lu  %10.1f  %7lu  %s\n", cur, (unsigned long)t0,
                            (t1 - t0) / 1000.0f, (unsigned long)cnt, logEventName[lastEv < 8 ? lastEv : 0]);
        cur = r.session; t0 = r.tMs; cnt = 0; lastEv = 0;
      }
      t1 = r.tMs; cnt++;
      if (r.event) lastEv = r.event;
    }
    pos += n - (n % LOG_REC_SIZE);
  }
  if (cnt) LOG_PRINTF("%7u  %9lu  %10.1f  %7lu  %s\n", cur, (unsigned long)t0,
                      (t1 - t0) / 1000.0f, (unsigned long)cnt, logEventName[lastEv < 8 ? lastEv : 0]);
  if (bad) LOG_PRINTF("(%lu damaged records skipped)\n", (unsigned long)bad);
  if (logWriteErrors) LOG_PRINTF("(%lu flash write errors this power-up)\n", (unsigned long)logWriteErrors);
  (void)t0; (void)t1; (void)lastEv;   // used only when LOG_PRINTF is enabled
}

void logDump(int onlySession) {
  if (!logPart) { LOG_PRINTF("Logging unavailable (no partition).\n"); return; }
#if ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(250);                 // dump must not drop data
#endif
  LOG_PRINTF("### LOG BEGIN\n");
  LOG_PRINTF("session,t_ms,type,event,state,armed,link,tau,lq,loop_hz,nose_deg,bank_deg,hdg_deg,"
             "stk_roll,stk_pitch,stk_yaw,thr,err_x,err_y,err_z,out_x,out_y,out_z,"
             "gyro_x,gyro_y,gyro_z,servo_l,servo_r,motor_l_cmd,motor_r_cmd,imu_glitches,"
             "ch5_us,ch6_us,thr_stick,arm_sw,fwd_sw,landed,rx_fs,ahrs_trust,acc_g,"
             "rate_sp_x,rate_sp_y,rate_sp_z,i_x,i_y,i_z\n");
  static uint8_t buf[LOG_REC_SIZE * 64];
  uint32_t rows = 0;
  for (uint32_t pos = 0; pos < logWritePos; ) {
    uint32_t n = min((uint32_t)sizeof(buf), logWritePos - pos);
    esp_partition_read(logPart, pos, buf, n);
    for (uint32_t i = 0; i + LOG_REC_SIZE <= n; i += LOG_REC_SIZE) {
      LogRec r; memcpy(&r, buf + i, LOG_REC_SIZE);
      if (!logRecValid(r)) continue;
      if (onlySession >= 0 && r.session != onlySession) continue;
      uint8_t type = (r.flags >> 5) & 3;
      LOG_PRINTF("%u,%lu,%s,%s,%s,%d,%d,%.3f,%d,%d,%.1f,%.1f,%.1f,%.2f,%.2f,%.2f,%.3f,"
                 "%.1f,%.1f,%.1f,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%u,%u,%u,%u,%u,"
                 "%u,%u,%.3f,%d,%d,%d,%d,%.2f,%.2f,%.1f,%.1f,%.1f,%.2f,%.2f,%.2f\n",
                 r.session, (unsigned long)r.tMs, type == LOGT_EVENT ? "EVENT" : "DATA",
                 logEventName[r.event < 8 ? r.event : 0], logStateName(r.flags & 7),
                 (r.flags >> 3) & 1, (r.flags >> 4) & 1, r.tau / 250.0f, r.lq, r.loopHz2 * 2,
                 r.nose / 10.0f, r.bank / 10.0f, r.hdg / 10.0f,
                 r.stk[0] / 100.0f, r.stk[1] / 100.0f, r.stk[2] / 100.0f, r.thr / 250.0f,
                 r.err[0] / 10.0f, r.err[1] / 10.0f, r.err[2] / 10.0f,
                 r.out[0] / 100.0f, r.out[1] / 100.0f, r.out[2] / 100.0f,
                 r.gyr[0] / 10.0f, r.gyr[1] / 10.0f, r.gyr[2] / 10.0f,
                 r.servoL, r.servoR, r.motorL, r.motorR, r.glitches,
                 r.ch5, r.ch6, r.thrStk / 250.0f, r.flags2 & 1, (r.flags2 >> 1) & 1,
                 (r.flags2 >> 2) & 1, (r.flags2 >> 3) & 1, (r.flags2 >> 4) / 15.0f, r.accG / 50.0f,
                 r.rateSp[0] / 10.0f, r.rateSp[1] / 10.0f, r.rateSp[2] / 10.0f,
                 r.iTerm[0] / 100.0f, r.iTerm[1] / 100.0f, r.iTerm[2] / 100.0f);
      (void)type;                   // used only when LOG_PRINTF is enabled
      rows++;
      idleService();
      if ((rows & 63) == 0) ledUpdate();
    }
    pos += n - (n % LOG_REC_SIZE);
  }
  LOG_PRINTF("### LOG END (%lu rows)\n", (unsigned long)rows);
  Serial.flush();
#if ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(0);
#endif
}

// Reads a command line from USB Serial. Only acts while DISARMED.
void logCommandPoll() {
  static char line[24];
  static uint8_t len = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c != '\n') { if (len < sizeof(line) - 1) line[len++] = c; continue; }
    line[len] = 0; len = 0;
    if (line[0] == 0) continue;
    if (armed) { LOG_PRINTF("Ignored '%s' - commands only work while DISARMED.\n", line); continue; }
    if (!strcmp(line, "help")) {
      LOG_PRINTF("Commands: list | dump | dump <session> | erase | help\n"
                 "          cal | cal status | cal force | cal erase\n");
    } else if (!strcmp(line, "cal")) {
      LOG_PRINTF("=== MANUAL CALIBRATION ===\n");
      calibrateAndSave();
      calSetForce(false);
      lastLoopUs = imuLastTickUs;
      ledSetPattern(LED_DISARMED);
      LOG_PRINTF("Calibration complete.\n");
    } else if (!strcmp(line, "cal status")) {
      calStatus();
    } else if (!strcmp(line, "cal force")) {
      calSetForce(true);
      LOG_PRINTF("Forced calibration armed - reset or power-cycle to run it.\n");
    } else if (!strcmp(line, "cal erase")) {
      if (calErase()) LOG_PRINTF("Saved calibration erased - the next boot will calibrate.\n");
      else            LOG_PRINTF("Calibration erase FAILED.\n");
    } else if (!strcmp(line, "list")) {
      logList();
    } else if (!strcmp(line, "dump")) {
      logDump(-1);
    } else if (!strncmp(line, "dump ", 5)) {
      logDump(atoi(line + 5));
    } else if (!strcmp(line, "erase")) {
      if (!logPart) { LOG_PRINTF("Logging unavailable (no partition).\n"); continue; }
      LOG_PRINTF("Erasing flight log (%lu KB)... ", (unsigned long)(logPart->size / 1024));
      Serial.flush();
      logEraseAll(); logSession = 0;
      ledSetPattern(LED_DISARMED);
      LOG_PRINTF("done.\n");
    } else {
      LOG_PRINTF("Unknown command '%s' - type help\n", line);
    }
  }
}

// ---------------------------- Disarmed servos / idle service ----------------
bool rcLinkOk() {
  return rcEverValid && !rcFailsafeFlag && (millis() - rcLastFrameMs < RC_TIMEOUT_MS) && linkLQ != 0;
}

// Elevons follow the radio sticks (same mixing/directions as flight).
// No radio link -> elevons centred.
void servosFromSticks(int& sL, int& sR) {
  bool ok = rcLinkOk();
  float r = ok ? stickNorm(rcUs[CH_ROLL],  RC_REV_ROLL)  : 0;
  float p = ok ? stickNorm(rcUs[CH_PITCH], RC_REV_PITCH) : 0;
  sL = elevonUs(PITCH_MIX * p - ROLL_MIX * r, SERVO_CENTER_L, SERVO_REV_L);
  sR = elevonUs(PITCH_MIX * p + ROLL_MIX * r, SERVO_CENTER_R, SERVO_REV_R);
}

// Called from every wait loop (boot, calibration, log erase/dump) and the
// main loop: radio parsing, ESC keep-alive and - while DISARMED - servos
// following the sticks.
void idleService() {
  crsfPoll();
  crsfWatchdog(armed);
  escService(false);
  if (!armed) {
    rcStatusReport();               // DEBUG: receiver status once per second
    static uint32_t lastServoMs = 0;
    if (millis() - lastServoMs >= 20) {
      lastServoMs = millis();
      int sL, sR;
      servosFromSticks(sL, sR);
      pwmWriteUs(outSL, sL); pwmWriteUs(outSR, sR);
    }
  }
}

void disarm(const char* why, uint8_t ev) {
  if (armed) { DBG_PRINTF("DISARMED: %s\n", why); if (LOG_ENABLE) logEvent(ev); }
  armed = false; tau = 0; state = ST_DISARMED;
  crashStartMs = 0;                  // do not carry a crash timer into the next flight
  fwdStartMs = 0;                    // no throttle-hold carried into the next flight
  for (int i = 0; i < 3; i++) pid[i].reset();
  escWrite(MOTOR_OFF_CMD, MOTOR_OFF_CMD);
  ledSetPattern(LED_DISARMED);
}

// ============================================================================
// ---------------------------- Receiver diagnostic (RX_TEST 1) -------------
#if (RX_TEST + 0)
volatile uint32_t rxtEdges[8];
void IRAM_ATTR rxtEdgeIsr(void* arg) { (*(volatile uint32_t*)arg)++; }

uint8_t rxtCrc8(const uint8_t* p, uint8_t len) {
  uint8_t crc = 0;
  while (len--) {
    crc ^= *p++;
    for (uint8_t i = 0; i < 8; i++)
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
  }
  return crc;
}

void rxtUart(int rx, int tx, uint32_t baud, bool inv, uint32_t ms,
             uint32_t& bytes, uint32_t& frames, uint32_t& rcFr, uint16_t ch[6]) {
  Serial1.end(); delay(20);
  Serial1.setRxBufferSize(1024);
  Serial1.begin(baud, SERIAL_8N1, rx, tx, inv);
  delay(20);
  while (Serial1.available()) Serial1.read();
  bytes = frames = rcFr = 0;
  uint8_t buf[64]; uint8_t pos = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    while (Serial1.available()) {
      uint8_t b = Serial1.read(); bytes++;
      if (pos == 0) { if (b == 0xC8 || b == 0xEE || b == 0xEA || b == 0x00) buf[pos++] = b; continue; }
      if (pos == 1) { if (b < 2 || b > 62) { pos = 0; continue; } buf[pos++] = b; continue; }
      buf[pos++] = b;
      if (pos >= buf[1] + 2) {
        if (rxtCrc8(&buf[2], buf[1] - 1) == buf[buf[1] + 1]) {
          frames++;
          if (buf[2] == 0x16 && buf[1] == 24) {
            rcFr++;
            uint32_t bits = 0; uint8_t nb = 0, c = 0;
            for (int i = 0; i < 22 && c < 6; i++) {
              bits |= (uint32_t)buf[3 + i] << nb; nb += 8;
              while (nb >= 11 && c < 6) { ch[c++] = (uint16_t)(1500 + ((int32_t)(bits & 0x7FF) - 992) * 5 / 8); bits >>= 11; nb -= 11; }
            }
          }
        }
        pos = 0;
      }
    }
    delay(1);
  }
  Serial1.end();
}

void rxTestLoop() {
  Serial.begin(115200);
  delay(2000);
  const int pins[6] = {1, 2, 3, 5, 6, 7};
  bool ccInit = false;
  for (int round = 1; ; round++) {
    Serial.printf("\n=== RX TEST round %d %s===  radio ON + bound, receiver LED solid\n",
                  round, ccInit ? "(CodeCell library initialised) " : "");
    int best = -1; uint32_t bestE = 0;
    for (int i = 0; i < 6; i++) {
      int p = pins[i];
      pinMode(p, INPUT_PULLDOWN); delay(5); int lo = digitalRead(p);
      pinMode(p, INPUT_PULLUP);   delay(5); int hi = digitalRead(p);
      pinMode(p, INPUT);
      rxtEdges[i] = 0;
      attachInterruptArg(digitalPinToInterrupt(p), rxtEdgeIsr, (void*)&rxtEdges[i], CHANGE);
      delay(300);
      detachInterrupt(digitalPinToInterrupt(p));
      uint32_t e = rxtEdges[i];
      const char* st = e > 50 ? "DATA ACTIVITY" :
                       (lo == 0 && hi == 1) ? "floating (nothing connected / driving)" :
                       (lo == 1 && hi == 1) ? "held HIGH (UART idle level, no data)" :
                       (lo == 0 && hi == 0) ? "held LOW" : "unstable";
      Serial.printf("  IO%d: %6lu edges / 0.3 s   %s%s\n", p, (unsigned long)e, st,
                    p == CRSF_RX_PIN ? "   <- CRSF_RX_PIN" : "");
      if (e > bestE) { bestE = e; best = p; }
    }
    if (best < 0 || bestE <= 50) {
      Serial.printf("  VERDICT: no data on ANY pin. Receiver powered + LINKED? TX->IO%d?\n", CRSF_RX_PIN);
    } else {
      int tx = (best == CRSF_TX_PIN) ? CRSF_RX_PIN : CRSF_TX_PIN;
      const uint32_t bauds[5] = {420000, 416666, 400000, 115200, 921600};
      uint32_t bestRc = 0, bestBaud = 0, anyBytes = 0; bool bestInv = false;
      uint16_t bestCh[6] = {0, 0, 0, 0, 0, 0};
      for (int b = 0; b < 5; b++) {
        for (int inv = 0; inv < 2; inv++) {
          uint32_t by, fr, rcf; uint16_t ch[6] = {0, 0, 0, 0, 0, 0};
          rxtUart(best, tx, bauds[b], inv, 400, by, fr, rcf, ch);
          Serial.printf("  IO%d %7lu baud %-8s: bytes %5lu  CRSF frames %4lu  RC frames %4lu\n",
                        best, (unsigned long)bauds[b], inv ? "inverted" : "normal",
                        (unsigned long)by, (unsigned long)fr, (unsigned long)rcf);
          anyBytes += by;
          if (rcf > bestRc) { bestRc = rcf; bestBaud = bauds[b]; bestInv = inv; memcpy(bestCh, ch, sizeof(ch)); }
        }
      }
      if (bestRc > 0)
        Serial.printf("  VERDICT: CRSF OK on IO%d at %lu baud %s.  CH1 %u CH2 %u CH3 %u CH4 %u CH5 %u CH6 %u\n",
                      best, (unsigned long)bestBaud, bestInv ? "INVERTED" : "normal",
                      bestCh[0], bestCh[1], bestCh[2], bestCh[3], bestCh[4], bestCh[5]);
      else if (anyBytes > 0)
        Serial.printf("  VERDICT: data but no valid CRSF frames - set Serial Protocol = CRSF in the receiver.\n");
      else
        Serial.printf("  VERDICT: pin toggles but UART reads nothing - check the wire is on the receiver TX pad.\n");
    }
    if (!ccInit) { myCodeCell.Init(LIGHT); ccInit = true; }
    delay(3000);
  }
}
#endif

// ---------------------------- Output start ---------------------------
// Must run AFTER myCodeCell.Init(): Init re-configures pins set up before it.
void outputsBegin() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcDetach(SERVO_L_PIN); ledcDetach(SERVO_R_PIN);       // clean (re)attach
#endif
  pwmBegin(outSL); pwmBegin(outSR);
  pwmWriteUs(outSL, SERVO_CENTER_L); pwmWriteUs(outSR, SERVO_CENTER_R);
  escBegin();
  escService(true);
}

#if (HW_TEST + 0)
void hwTestLoop() {
  Serial.begin(115200);
  delay(1500);
  for (int i = 0; i < MAX_CH; i++) rcUs[i] = 1500;
  rcUs[CH_THR] = 1000; rcUs[CH_ARM] = 1000; rcUs[CH_MODE] = 1000;
  myCodeCell.Init(LIGHT);
  outputsBegin();
  crsfBegin();
  Serial.printf("\n=== HW TEST ===  PROPS OFF.  Radio ON.\n"
                "Sticks move the elevons. ESC: %s on L IO%d / R IO%d.\n"
                "MOTOR TEST: throttle LOW, then CH5 HIGH -> both motors follow the throttle stick.\n"
                "            CH5 LOW = stop.\n\n", escModeName(), MOTOR_L_PIN, MOTOR_R_PIN);
  uint32_t last = millis(), f0 = 0, c0 = 0, e0 = 0;
  bool motorTest = false, armWasLow = false;
  for (;;) {
    idleService();                                 // receiver, ESC keep-alive, servos from sticks
    bool ok = rcLinkOk();
    bool armSw = ok && rcUs[CH_ARM] > ARM_SWITCH_US;
    float thr = ok ? throttleNorm(rcUs[CH_THR]) : 0;
    if (!armSw) { armWasLow = true; motorTest = false; }
    else if (!motorTest && armWasLow && thr <= ARM_MAX_THR) { motorTest = true; armWasLow = false; }
    int cmd = motorTest ? motorCmd(thr) : MOTOR_OFF_CMD;
    if (cmd != escValL || cmd != escValR) escWrite(cmd, cmd);
    ledSetPattern(motorTest ? LED_ARMED : (ok ? LED_DISARMED : LED_CAL_FAILED));
    ledUpdate();
    if (millis() - last >= 200) {
      uint32_t dt = millis() - last; last = millis();
      int sL, sR; servosFromSticks(sL, sR);
      Serial.printf("%s fr %3lu/s chg %3lu/s | CH1 %4u CH2 %4u CH3 %4u CH4 %4u CH5 %4u CH6 %4u"
                    " | servo L %4d R %4d | %s motor cmd %4d | ESC %4lu fr/s skip %lu\n",
                    ok ? "LINK" : "NOLINK",
                    (unsigned long)((rcFrames - f0) * 1000UL / dt), (unsigned long)((rcChangedFrames - c0) * 1000UL / dt),
                    rcUs[0], rcUs[1], rcUs[2], rcUs[3], rcUs[4], rcUs[5], sL, sR,
                    motorTest ? "MOTOR TEST" : "stopped   ", cmd,
                    (unsigned long)((escFrames - e0) * 1000UL / dt), (unsigned long)escSkipped);
      f0 = rcFrames; c0 = rcChangedFrames; e0 = escFrames;
    }
    delayMicroseconds(200);
  }
}
#endif

// ---------------------------- ESC calibration (ESC_CAL 1) -----------------
// Throttle passthrough only. Uses the receiver and the ESC outputs - nothing
// else is started (no IMU, servos, arming, stabilisation or logging).
#if (ESC_CAL + 0)
void escCalLoop() {
  Serial.begin(115200);
  myCodeCell.Init(LIGHT);                 // LED only (re-configures pins - run first)
  // Hold ESC signal pins LOW - no pulses - until the receiver is linked.
  pinMode(MOTOR_L_PIN, OUTPUT); digitalWrite(MOTOR_L_PIN, LOW);
  pinMode(MOTOR_R_PIN, OUTPUT); digitalWrite(MOTOR_R_PIN, LOW);
  ledSetPattern(LED_CAL_FAILED);          // fast RED = waiting for link
  for (int i = 0; i < MAX_CH; i++) rcUs[i] = 1500;
  rcUs[CH_THR] = 1000;
  crsfBegin();
  Serial.printf("\n=== ESC CALIBRATION ===  PROPS OFF!  ESCs get NO signal until the receiver links.\n");
  uint32_t linkSince = 0, lastMsg = 0;
  for (;;) {                              // wait for a stable receiver link
    crsfPoll();
    crsfWatchdog(false);
    if (rcLinkOk()) { if (!linkSince) linkSince = millis(); }
    else linkSince = 0;
    if (linkSince && millis() - linkSince >= ESC_CAL_LINK_MS) break;
    ledUpdate();
    if (millis() - lastMsg >= 500) {
      lastMsg = millis();
      Serial.printf("  waiting for receiver link... (ESC outputs LOW)  frames %lu\n", (unsigned long)rcFrames);
    }
    delay(1);
  }
  ledSetPattern(LED_ESC_CAL);             // fast WHITE = passthrough
  escBegin();                             // ESC pins only - servo pins are not driven
  escWrite(constrain((int)rcUs[CH_THR], 1000, 2000), constrain((int)rcUs[CH_THR], 1000, 2000));
  Serial.printf("Receiver linked - ESC output started at CH3 = %u us.\n", rcUs[CH_THR]);
  Serial.printf("\n=== ESC CALIBRATION (passthrough) ===  PROPS OFF!\n"
                "ESC: %s on L IO%d / R IO%d.  CH3 -> both ESCs, 1000-2000 scale.\n"
                "Typical: throttle FULL UP before power-on, wait for max tones,\n"
                "then FULL DOWN for min tones. Set ESC_CAL 0 when finished.\n\n",
                escModeName(), MOTOR_L_PIN, MOTOR_R_PIN);
  uint32_t last = 0;
  for (;;) {
    crsfPoll();
    crsfWatchdog(false);
    bool ok = rcLinkOk();
    int cmd = ok ? constrain((int)rcUs[CH_THR], 1000, 2000) : MOTOR_OFF_US;
    if (cmd != escValL || cmd != escValR) escWrite(cmd, cmd);
    else escService(false);
    ledUpdate();
    if (millis() - last >= 200) {
      last = millis();
      Serial.printf("%s  CH3 %4u us -> ESC %4d (%5.1f us pulse)\n",
                    ok ? "LINK  " : "NOLINK", rcUs[CH_THR], cmd,
                    escPulseNs(cmd) / 1000.0f);
    }
    delayMicroseconds(200);
  }
}
#endif

void setup() {
#if (ESC_CAL + 0)
  escCalLoop();                     // ESC calibration passthrough only - never returns
#endif
#if (RX_TEST + 0)
  rxTestLoop();                     // receiver diagnostic only - never returns
#endif
#if (HW_TEST + 0)
  hwTestLoop();                     // receiver + servo + ESC check - never returns
#endif
  DBG_BEGIN();
  for (int i = 0; i < MAX_CH; i++) rcUs[i] = 1500;
  rcUs[CH_THR] = 1000; rcUs[CH_ARM] = 1000; rcUs[CH_MODE] = 1000;

#if IMU_DRIVER == 1
  myCodeCell.Init(LIGHT);           // LED + I2C bus; BNO085 is driven directly below
#endif
  // Servos + ESCs + receiver AFTER myCodeCell.Init() (Init re-configures
  // pins set up before it). ESCs receive "motor stop" from here on.
  outputsBegin();
  if (crsfBegin()) DBG_PRINTF("CRSF receiver started (%s) RX IO%d TX IO%d @ %d baud\n",
                              CRSF_LIB ? "CRSFforArduino" : "built-in", CRSF_RX_PIN, CRSF_TX_PIN, CRSF_BAUD);
  else             DBG_PRINTF("CRSF receiver FAILED to start - check the CRSFforArduino library\n");
  ledSetPattern(LED_DISARMED);      // green until calibration starts (if needed)
#if (LOG_ENABLE + 0)
  logBegin();                       // find end of log, pre-erase free space
#endif
  if (!imuBegin()) {
    DBG_PRINTF("FATAL: BNO085 not found at 0x%02X - check IMU_DRIVER / library.\n", BNO_ADDR);
    ledSetPattern(LED_CAL_FAILED);
    for (;;) { ledUpdate(); idleService(); delay(1); }   // halt: fast red, motors stopped
  }
#if IMU_DRIVER == 0
  outputsBegin();                   // imuBegin() ran myCodeCell.Init() again
#endif
  DBG_PRINTF("IMU driver %d started (%d Hz control rate), ESC %s\n", IMU_DRIVER, IMU_RATE_HZ,
             escModeName());
#if (IMU_TEST + 0)
  imuTestLoop();                    // never returns
#endif

  // ---- Boot calibration: reuse saved data, or learn it once ----
  // A saved calibration lets a field boot skip the two poses: the aircraft
  // may be held in any attitude, and gravity supplies the initial attitude.
  CalBlob cal;
  bool forceCal = (CAL_FORCE != 0) || calForceRequested();
  bool haveCal = false;
  if (!forceCal) {
    haveCal = calLoad(cal);
    if (haveCal) calApply(cal);
  }
  if (haveCal) {
    CAL_PRINTF("Saved calibration loaded - skipping the two-pose calibration.\n");
    initAttitudeFromAccel();
  } else {
    if (forceCal) CAL_PRINTF("Boot calibration FORCED%s.\n",
                             (CAL_FORCE != 0) ? " (CAL_FORCE)" : " (cal force flag)");
    else          CAL_PRINTF("No saved calibration - running the two-pose calibration.\n");
    calibrateAndSave();
    calSetForce(false);             // one-shot flag consumed
  }
  ledSetPattern(LED_DISARMED);

  armSwitchWasLow = false;          // CH5 must be cycled low -> high to arm
  lastLoopUs = imuLastTickUs;       // first control dt starts from the last calibration sample
  DBG_PRINTF("Tail-sitter ready. CH5 = arm, CH6 = vertical/forward. Servos follow sticks while disarmed.\n");
}

// ============================================================================
#if (DEBUG + 0)
void debugReport(const float e[3], const float u[3], int sL, int sR, int mL, int mR,
                 float sRoll, float sPitch, float sYaw, float thr, bool rcOk) {
  static uint32_t lastPrint = 0;
  static float pN = 1e9, pB = 1e9, pH = 1e9;
  static int pSL = -1, pSR = -1, pML = -1, pMR = -1;
  static FlightState pSt = (FlightState)-1;
  static bool pLink = false;
  if (millis() - lastPrint < 1000 / DEBUG_MAX_HZ) return;

  float nose = noseAboveHorizonDeg(), bank = bankDeg(), hdg = headingRad(tau) * R2D;
  bool changed = fabsf(nose - pN) >= DEBUG_ANGLE_STEP || fabsf(bank - pB) >= DEBUG_ANGLE_STEP ||
                 fabsf(wrapPi((hdg - pH) * D2R)) * R2D >= DEBUG_ANGLE_STEP ||
                 abs(sL - pSL) >= DEBUG_US_STEP || abs(sR - pSR) >= DEBUG_US_STEP ||
                 abs(mL - pML) >= DEBUG_US_STEP || abs(mR - pMR) >= DEBUG_US_STEP ||
                 state != pSt || rcOk != pLink;
  if (!changed) return;
  lastPrint = millis();
  pN = nose; pB = bank; pH = hdg; pSL = sL; pSR = sR; pML = mL; pMR = mR; pSt = state; pLink = rcOk;

  Serial.printf("%-8s tau%.2f %s LQ%3d %3.0fHz | nose %6.1f bank %6.1f hdg %6.1f | "
                "CH6 %4u | stk R%+.2f P%+.2f Y%+.2f T%.2f | err %+5.1f %+5.1f %+5.1f | "
                "out %+.2f %+.2f %+.2f | elev L%4d R%4d | mot L%4d R%4d\n",
                stateName[state], tau, rcOk ? "link" : "LOST", linkLQ, loopHz,
                nose, bank, hdg, rcUs[CH_MODE], sRoll, sPitch, sYaw, thr,
                e[0]*R2D, e[1]*R2D, e[2]*R2D, u[0], u[1], u[2], sL, sR, mL, mR);
}
#endif

void loop() {
  idleService();                    // radio, ESC keep-alive, disarmed servos
  ledUpdate();
#if (LOG_ENABLE + 0)
  logCommandPoll();                 // commands run only while disarmed
  if (!armed) ledSetPattern(logFill() >= LOG_FULL_WARN ? LED_LOG_FULL : LED_DISARMED);
#endif

  // Control loop runs once per fresh IMU sample from myCodeCell.Run()
  if (!imuService()) {
    if (armed && millis() - imuLastTickMs > IMU_LOSS_MS) {
      disarm("IMU data lost", EV_DISARM_IMU_LOST);
    }
    return;
  }
  uint32_t nowUs = imuLastTickUs;
  float dt = (nowUs - lastLoopUs) * 1e-6f;
  lastLoopUs = nowUs;
  // A gap much longer than one control period (I2C/flash/USB stall) makes the
  // D term divide a large gyro change by a tiny dt. Reset the PIDs instead.
  if (dt <= 0.0f || dt > 2.0f / IMU_RATE_HZ) {
    for (int i = 0; i < 3; i++) pid[i].reset();
    dt = 1.0f / IMU_RATE_HZ;
  }
  loopHz += 0.05f * (1.0f / dt - loopHz);

  // ---- Sensors / attitude ----
  readImuBody(gyro, accel);
  for (int k = 0; k < 3; k++) gyro[k] -= gyroBias[k];
  ahrsUpdate(gyro, accel, dt, AHRS_KP);
  // Stillness for the arm interlock: only accumulate time while the gyro is
  // quiet, and reset immediately on any motion.
  if (vNorm3(gyro) * R2D < ARM_MAX_RATE_DPS) {
    armStillSec += dt;
    if (armStillSec > 1.0f) armStillSec = 1.0f;
  } else {
    armStillSec = 0;
  }

  // ---- RC ----
  bool rcOk = rcLinkOk();
  float sRoll  = rcOk ? stickNorm(rcUs[CH_ROLL],  RC_REV_ROLL)  : 0;
  float sPitch = rcOk ? stickNorm(rcUs[CH_PITCH], RC_REV_PITCH) : 0;
  float sYaw   = rcOk ? stickNorm(rcUs[CH_YAW],   RC_REV_YAW)   : 0;
  float thrStk = rcOk ? throttleNorm(rcUs[CH_THR]) : 0;
  bool  armSw  = rcOk && rcUs[CH_ARM] > ARM_SWITCH_US;
  bool  fwdSw  = rcOk && rcUs[CH_MODE] > MODE_SWITCH_US;
  logX.thrStk = thrStk; logX.armSw = armSw; logX.fwdSw = fwdSw;
  logX.landed = false; logX.rateSp[0] = logX.rateSp[1] = logX.rateSp[2] = 0;

  // ---- Arm / disarm (CH5) ----
  if (rcOk && !armSw) armSwitchWasLow = true;
  if (!armed) {
    if (armSw && armSwitchWasLow) {
      armSwitchWasLow = false;                         // one attempt per flick
      float tilt = tiltFromVerticalDeg();
      if (thrStk > ARM_MAX_THR)        DBG_PRINTF("ARM refused: throttle not low\n");
      else if (fwdSw)                  DBG_PRINTF("ARM refused: CH6 must be VERTICAL\n");
      else if (tilt > ARM_MAX_TILT)    DBG_PRINTF("ARM refused: not upright (%.0f deg)\n", tilt);
      else if (armStillSec < ARM_STILL_MS * 0.001f)
        DBG_PRINTF("ARM refused: aircraft moving (%.0f dps)\n", vNorm3(gyro) * R2D);
      else if (ahrsTrust < ARM_MIN_TRUST)
        DBG_PRINTF("ARM refused: attitude estimate untrusted (%.2f)\n", ahrsTrust);
      else {
        armed = true; tau = 0; state = ST_HOVER;
        crashStartMs = 0;                // start each flight with a clean crash timer
        armMs = millis();                // forward transition is blocked for a moment
        fwdStartMs = 0;                  // no throttle-hold carried over
        headingTarget = headingRad(0);
        for (int i = 0; i < 3; i++) pid[i].reset();
        ledSetPattern(LED_ARMED);
        if (LOG_ENABLE) logEvent(EV_ARM);
        DBG_PRINTF("ARMED (log session %u)\n", logSession);
      }
    }
  } else if (rcOk && !armSw) {
    disarm("CH5 switch", EV_DISARM_SWITCH);
  }

  // ---- State machine / transition ----
  if (armed) {
    if (!rcOk) {
      if (state != ST_FAILSAFE) {
        state = ST_FAILSAFE; failsafeStartMs = millis(); DBG_PRINTF("FAILSAFE\n");
        fwdStartMs = 0;                  // re-require the throttle hold after recovery
        if (LOG_ENABLE) logEvent(EV_FAILSAFE);
      }
      tau = max(0.0f, tau - dt / TRANS_BACK_SEC);
      if (millis() - failsafeStartMs > FS_DISARM_MS) disarm("failsafe timeout", EV_DISARM_FS_TIMEOUT);
    } else {
      if (state == ST_FAILSAFE) {
        state = ST_HOVER; DBG_PRINTF("Link restored\n");
        if (LOG_ENABLE) logEvent(EV_LINK_RESTORED);
      }
      if (fwdSw) {
        bool armSettled = (millis() - armMs) >= TRANS_ARM_DELAY_MS;
        bool thrHigh    = thrStk >= TRANS_START_MIN_THR;
        // Start only after the throttle has stayed above the floor for the hold
        // time and the aircraft has settled after arming. Once the transition
        // has begun (tau > 0) it continues even if the throttle later drops.
        if (armSettled && thrHigh) {
          if (fwdStartMs == 0) fwdStartMs = millis();
        } else if (tau <= 0.0f) {
          fwdStartMs = 0;                // hold broken before the transition began
        }
        bool startReady = (fwdStartMs != 0) &&
                          (millis() - fwdStartMs >= TRANS_START_HOLD_MS);
        if (tau > 0.0f || startReady) tau = min(1.0f, tau + dt / TRANS_FWD_SEC);
        state = (tau >= 1.0f) ? ST_FORWARD : (tau > 0 ? ST_TO_FWD : ST_HOVER);
      } else {
        fwdStartMs = 0;                  // CH6 back to vertical cancels a pending start
        tau = max(0.0f, tau - dt / TRANS_BACK_SEC);
        state = (tau <= 0.0f) ? ST_HOVER : ST_TO_HOVER;
      }
    }
    // Crash detection: a large, sustained attitude error from the intended
    // attitude. Hover keeps the original tilt check; transition/forward uses
    // the attitude error so a normal pitch-up is not mistaken for a crash.
    bool crashCond;
    if (tau <= 0.0f) {
      crashCond = tiltFromVerticalDeg() > CRASH_TILT_DEG;
    } else {
      bool fsNow = (state == ST_FAILSAFE);
      crashCond = attitudeErrorDeg(tau, fsNow ? 0 : sRoll, fsNow ? 0 : sPitch) > CRASH_ATT_ERR_DEG;
    }
    if (armed && crashCond) {
      if (crashStartMs == 0) crashStartMs = millis();
      if (millis() - crashStartMs > CRASH_TIME_MS) disarm("crash / loss of attitude", EV_DISARM_CRASH);
    } else crashStartMs = 0;
  }

  float e[3] = {0, 0, 0}, u[3] = {0, 0, 0};
  int sL, sR, mL, mR;
  float thr = 0;

  if (!armed) {
    // ---- DISARMED: elevons follow sticks for pre-flight checks, motors stopped ----
    servosFromSticks(sL, sR);
    mL = mR = MOTOR_OFF_CMD;
  } else {
    // ---- Target attitude ----
    // qd = Yaw(heading) * Pitch(nose) * Roll(bank, forward) * BodyZ(side-lean, hover)
    bool fs = (state == ST_FAILSAFE);
    float rStk = fs ? 0 : sRoll, pStk = fs ? 0 : sPitch, yStk = fs ? 0 : sYaw;

 if (tau <= 0.0f) {
  headingTarget = wrapPi(headingTarget + yStk * HOVER_YAW_RATE_DPS * D2R * dt);
  float hNow = headingRad(0.0f);
  float dh = wrapPi(headingTarget - hNow);
  float lim = HOVER_HDG_SLACK_DEG * D2R;
  if (dh > lim) headingTarget = wrapPi(hNow + lim);
  if (dh < -lim) headingTarget = wrapPi(hNow - lim);
  } else {
  headingTarget = headingRad(tau); // no heading hold once moving
  }
    float pitchDeg = lerpf(90.0f, FWD_PITCH_DEG, tau) + pStk * lerpf(HOVER_MAX_TILT_DEG, FWD_MAX_PITCH_DEG, tau);
    float bank     = rStk * FWD_MAX_BANK_DEG * tau;
    float lean     = rStk * HOVER_MAX_TILT_DEG * (1.0f - tau);

    Quat qd = qMul(qMul(qAxis(2, headingTarget), qAxis(1, pitchDeg * D2R)),
                   qMul(qAxis(0, bank * D2R), qAxis(2, lean * D2R)));
    attitudeError(qAtt, qd, e);
    e[2] *= (1.0f - tau);          // body-Z angle hold only while hovering

    // ---- Throttle ----
    // During the failsafe pitch-back, keep the transition floor; only drop to
    // FS_THR once the aircraft is back in hover.
    thr = fs ? (tau > 0.0f ? TRANS_BACK_THR_MIN : FS_THR) : thrStk;
    if (state == ST_TO_FWD)   thr = max(thr, TRANS_FWD_THR_MIN);
    if (state == ST_TO_HOVER) thr = max(thr, TRANS_BACK_THR_MIN);

    bool landed = (tau == 0 && thr < LANDED_THR);
    logX.landed = landed;

    // ---- Angle -> rate -> output (gains blended hover <-> forward) ----
    for (int k = 0; k < 3; k++) {
      float angP = lerpf(G_HOVER.angP[k], G_FWD.angP[k], tau);
      float maxR = lerpf(G_HOVER.maxRate[k], G_FWD.maxRate[k], tau);
      float rateSp = constrain(angP * e[k], -maxR, maxR);
      if (k == 2) rateSp += yStk * FWD_YAW_RATE_DPS * D2R * tau;   // rudder in forward flight
      logX.rateSp[k] = rateSp;
      // 'landed' only freezes the integrators (passed below); it must not
      // reset the D filter or zero differential thrust during a descent.
      u[k] = pid[k].update(rateSp, gyro[k],
                           lerpf(G_HOVER.rP[k], G_FWD.rP[k], tau),
                           lerpf(G_HOVER.rI[k], G_FWD.rI[k], tau),
                           lerpf(G_HOVER.rD[k], G_FWD.rD[k], tau), dt, landed);
    }

    // ---- Elevons: X -> differential, Y -> collective (+ = elevon UP) ----
    float outL = PITCH_MIX * u[1] - ROLL_MIX * u[0];
    float outR = PITCH_MIX * u[1] + ROLL_MIX * u[0];
    sL = elevonUs(outL, SERVO_CENTER_L, SERVO_REV_L);
    sR = elevonUs(outR, SERVO_CENTER_R, SERVO_REV_R);

    // ---- Motors: Z -> differential thrust (+Z = left motor faster) ----
    // Scale the differential so neither motor drops below AIRMODE_MIN_THR.
    // This trades some yaw/side-lean authority for keeping both motors alive;
    // it never raises collective throttle, so average thrust is unchanged.
    // When the collective is below the floor the differential is removed, so
    // the motors stay equal instead of one dropping toward stop.
    float d = u[2] * DIFF_THRUST_MAX;
    float base = thr;
    // A non-finite value bypasses the comparisons in constrain() and could
    // command one motor to an indeterminate output. Fail safe to equal motors.
    if (!isfinite(base)) base = 0.0f;
    if (!isfinite(d))    d = 0.0f;
    float room = fminf(1.0f - base, base - AIRMODE_MIN_THR);
    if (room < 0.0f) room = 0.0f;
    d = constrain(d, -room, room);
    mL = motorCmd(base + d);
    mR = motorCmd(base - d);
  }

  pwmWriteUs(outSL, sL); pwmWriteUs(outSR, sR);
  escWrite(mL, mR);

#if (LOG_ENABLE + 0)
  logSample(e, u, sRoll, sPitch, sYaw, thr, rcOk, sL, sR, mL, mR);
#endif

#if (DEBUG + 0)
  debugReport(e, u, sL, sR, mL, mR, sRoll, sPitch, sYaw, thr, rcOk);
#endif
}
