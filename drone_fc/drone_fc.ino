/*
  VTOL / Quad Flight Controller - classic ESP32 (RECEIVER)
  4-axis radio control + MPU6050 PID stabilization.

  Stabilization:
    - MPU6050 accel + gyro fused with a complementary filter -> clean roll/pitch.
    - Roll & pitch use ANGLE-mode PID (stick = lean angle, IMU holds it level).
    - Yaw uses RATE-mode PID (stick = turn speed).
    - Gyro bias + mounting tilt are auto-calibrated at boot (keep it still/level).
    - D-term on the gyro rate (no derivative kick); integral anti-windup + reset.
    - Correct quad-X mixing (the old code mixed pitch/yaw wrong).
    - Missing IMU -> falls back to manual stick passthrough so it still flies.
    - Signal-loss failsafe drops the motors to off and dumps the integrators.

  Motors (X layout):  esc1 = Left Front, esc2 = Left Rear,
                      esc3 = Right Front, esc4 = Right Rear

  >> TUNE PID gains pidRoll/pidPitch/pidYaw below. Always test with props OFF. <<
*/

#include <Wire.h>
#include <MPU6050.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <ESP32Servo.h>
#include <math.h>

// ---------------- nRF24 ----------------
RF24 radio(4, 5);                  // CE=GPIO4, CSN=GPIO5
const byte address[6] = "00001";

struct DataPacket {
  uint16_t throttle;               // 0..1023 (throttle stick)
  int16_t  yaw;                    // -512..511 (centered)
  int16_t  pitch;                  // -512..511 (centered)
  int16_t  roll;                   // -512..511 (centered)
};
DataPacket receivedData;

unsigned long lastPacketMs = 0;
const unsigned long FAILSAFE_MS = 1000;

// ---------------- MPU6050 ----------------
MPU6050 mpu;
int16_t ax, ay, az, gx, gy, gz;       // raw accel + gyro
float roll = 0, pitch = 0;            // fused angles (deg) from the complementary filter
float rollRate = 0, pitchRate = 0, yawRate = 0;   // gyro rates (deg/s)
bool mpuOk = false;

// MPU6050 scale factors (default ranges): accel +/-2g, gyro +/-250 deg/s
const float ACC_LSB_PER_G   = 16384.0;
const float GYRO_LSB_PER_DPS = 131.0;

// Gyro zero-rate bias captured at boot, and the resting (mount) angle trim.
float gxBias = 0, gyBias = 0, gzBias = 0;
float rollTrim = 0, pitchTrim = 0;

// Complementary-filter weight (how much we trust the gyro vs the accel).
const float COMP_ALPHA = 0.98;
unsigned long lastLoopUs = 0;          // for the real dt used by the filter + PID

// ---------------- ESCs ----------------
Servo esc1, esc2, esc3, esc4;
const int escPin1 = 14;            // Left Front
const int escPin2 = 27;            // Left Rear
const int escPin3 = 26;            // Right Front
const int escPin4 = 25;            // Right Rear

// ---------------- Limits ----------------
const int minThrottle = 1100;
const int maxThrottle = 1800;

// Live commands from the radio
int throttleCmd = 0;                 // 0..1023 (throttle stick)
int yawCmd = 0, pitchCmd = 0, rollCmd = 0;   // -512..511 (centered sticks)

// ---------------- Stabilization: PID (TUNE THESE) ----------------
// Roll & pitch run in ANGLE mode (stick = desired lean angle, the IMU holds it).
// Yaw runs in RATE mode (stick = desired rotation speed). Outputs are in ESC
// microseconds. Start gentle; raise kp until it holds firmly, add kd to stop
// the wobble, then a little ki to kill slow drift.
struct PID { float kp, ki, kd; };
PID pidRoll  = { 4.0, 2.0, 0.8 };
PID pidPitch = { 4.0, 2.0, 0.8 };
PID pidYaw   = { 3.0, 0.0, 0.0 };     // yaw rate hold (P-only is usually enough)

float iRoll = 0, iPitch = 0, iYaw = 0;   // integral accumulators (anti-windup clamped)
const float I_LIMIT = 150.0;             // max |integral| contribution (us)

const float MAX_ANGLE    = 30.0;      // deg of lean at full roll/pitch stick
const float MAX_YAW_RATE = 150.0;     // deg/s at full yaw stick
const int   CORR_LIMIT   = 400;       // max us correction per axis

// Manual fallback gain used only if the MPU is missing (direct stick -> motors).
const float kManual = 0.40;
const float kYaw    = 0.40;

// ---------------- Throttle ratchet / peak-hold ----------------
// Position-based throttle (starts at 0 = motors off):
//   - raise the stick      -> motors rise and HOLD the highest level reached
//   - bring it to the middle-> motors drop to mid speed
//   - bring it fully down   -> motors turn off
const int THR_MID     = 512;         // middle of the 0..1023 throttle range
const int THR_MIDBAND = 60;          // +/- window that counts as "middle"
const int THR_OFF     = 60;          // at/below this = "full below" -> off
int heldThrottle = 0;                // level we hold (0 = off at power-up)

// Calibrate the IMU at power-up: average the gyro to find its zero-rate bias and
// the accelerometer to find the resting (mounting) tilt. Keep the drone STILL and
// LEVEL during this. Skipped gracefully if the MPU isn't detected.
void autoCalibrate() {
  if (!mpuOk) { Serial.println("Auto-calibrate skipped (MPU not detected)."); return; }
  Serial.println("Calibrating IMU - keep the drone still & level...");

  const int N = 600;
  double sgx = 0, sgy = 0, sgz = 0;     // gyro sums (for bias)
  double sr = 0, sp = 0;                // accel-angle sums (for level trim)
  for (int i = 0; i < N; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sgx += gx; sgy += gy; sgz += gz;
    sr += atan2((float)ay, (float)az) * 180.0 / PI;
    sp += atan2(-(float)ax, sqrt((float)ay * ay + (float)az * az)) * 180.0 / PI;
    delay(2);
  }
  gxBias = sgx / N; gyBias = sgy / N; gzBias = sgz / N;
  rollTrim  = sr / N;
  pitchTrim = sp / N;

  // Seed the fused angles so they start at "level" instead of drifting in.
  roll = 0; pitch = 0;
  lastLoopUs = micros();

  Serial.printf("Gyro bias: gx=%.1f gy=%.1f gz=%.1f | Level trim: roll0=%.1f pitch0=%.1f\n",
    gxBias, gyBias, gzBias, rollTrim, pitchTrim);
}

void setup() {
  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(400000);
  Wire.setTimeOut(10);             // a dead IMU must not stall the loop

  mpu.initialize();
  mpuOk = mpu.testConnection();
  Serial.print("MPU6050 connection: ");
  Serial.println(mpuOk ? "OK" : "FAILED <-- check SDA=21/SCL=22 wiring, pull-ups, 3.3V, GND");

  esc1.attach(escPin1, 1000, 2000);
  esc2.attach(escPin2, 1000, 2000);
  esc3.attach(escPin3, 1000, 2000);
  esc4.attach(escPin4, 1000, 2000);
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);
  delay(4000);                     // let the ESCs arm

  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(76);
  radio.setPALevel(RF24_PA_MIN);
  radio.openReadingPipe(0, address);
  radio.startListening();
  Serial.print("nRF24 chip connected (SPI ok?): ");
  Serial.println(radio.isChipConnected() ? "YES" : "NO  <-- check CE/CSN/SCK/MOSI/MISO + 3.3V");

  autoCalibrate();                   // gyro bias + level trim at power-up, no reset needed
  lastLoopUs = micros();             // start the PID/filter clock
  Serial.println("=== DRONE RECEIVER STARTED ===");
}

void loop() {
  // ===== RECEIVE 4 AXES =====
  if (radio.available()) {
    radio.read(&receivedData, sizeof(receivedData));
    throttleCmd = constrain((int)receivedData.throttle, 0, 1023);
    yawCmd   = receivedData.yaw;
    pitchCmd = receivedData.pitch;
    rollCmd  = receivedData.roll;
    lastPacketMs = millis();
  }

  bool signalOk = (millis() - lastPacketMs) < FAILSAFE_MS;

  // ===== FAILSAFE: no link -> motors off and cut the sticks =====
  if (!signalOk) {
    throttleCmd = 0; yawCmd = pitchCmd = rollCmd = 0;
    heldThrottle = 0;
    iRoll = iPitch = iYaw = 0;            // dump PID integrals so it can't wind up
  }

  // ===== THROTTLE: ratchet peak-hold with middle / off detents =====
  // Rising stick raises the held level and keeps the highest. Lowering does
  // nothing until the stick reaches the middle (-> mid speed) or the very
  // bottom (-> off). So small dips don't cut thrust, but you can always step
  // down to mid or stop by bringing the stick to those positions.
  if (throttleCmd <= THR_OFF) {
    heldThrottle = 0;                              // stick fully down -> off
  } else if (throttleCmd > heldThrottle) {
    heldThrottle = throttleCmd;                    // rising -> hold the new peak
  } else if (abs(throttleCmd - THR_MID) <= THR_MIDBAND) {
    heldThrottle = THR_MID;                        // brought to middle -> mid speed
  }
  // otherwise: keep the last highest level (hold)
  int effThrottle = heldThrottle;

  // ===== THROTTLE -> motor base (spins the instant you raise it) =====
  int baseThrottle;
  if (effThrottle < 20) baseThrottle = 1000;                            // off
  else baseThrottle = map(effThrottle, 20, 1023, minThrottle, maxThrottle);
  bool flying = baseThrottle > minThrottle;   // only stabilize once actually spun up

  // ===== IMU: read accel + gyro, fuse into clean angles =====
  // dt is the real elapsed time so the filter and PID are frame-rate independent.
  unsigned long nowUs = micros();
  float dt = (nowUs - lastLoopUs) * 1e-6f;
  lastLoopUs = nowUs;
  if (dt <= 0 || dt > 0.05f) dt = 0.005f;     // guard against a bad first/!long step

  if (mpuOk) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    // Gyro -> deg/s (bias removed). gx=roll, gy=pitch, gz=yaw for a flat mount.
    rollRate  = (gx - gxBias) / GYRO_LSB_PER_DPS;
    pitchRate = (gy - gyBias) / GYRO_LSB_PER_DPS;
    yawRate   = (gz - gzBias) / GYRO_LSB_PER_DPS;
    // Accel -> absolute angles (deg), with the mounting trim removed.
    float rollAcc  = atan2((float)ay, (float)az) * 180.0 / PI - rollTrim;
    float pitchAcc = atan2(-(float)ax, sqrt((float)ay * ay + (float)az * az)) * 180.0 / PI - pitchTrim;
    // Complementary filter: gyro for fast/smooth, accel to stop long-term drift.
    roll  = COMP_ALPHA * (roll  + rollRate  * dt) + (1.0f - COMP_ALPHA) * rollAcc;
    pitch = COMP_ALPHA * (pitch + pitchRate * dt) + (1.0f - COMP_ALPHA) * pitchAcc;
  } else {
    roll = pitch = 0; rollRate = pitchRate = yawRate = 0;
  }

  // ===== PID STABILIZATION =====
  int rollCorr, pitchCorr, yawCorr;
  if (mpuOk) {
    // Sticks set the TARGET: angle for roll/pitch, rotation rate for yaw.
    float rollSet  = rollCmd  * (MAX_ANGLE / 512.0);
    float pitchSet = pitchCmd * (MAX_ANGLE / 512.0);
    float yawSet   = yawCmd   * (MAX_YAW_RATE / 512.0);

    float rollErr  = rollSet  - roll;
    float pitchErr = pitchSet - pitch;
    float yawErr   = yawSet   - yawRate;

    // Integrate only while flying (stops windup on the bench), with clamping.
    if (flying) {
      iRoll  = constrain(iRoll  + pidRoll.ki  * rollErr  * dt, -I_LIMIT, I_LIMIT);
      iPitch = constrain(iPitch + pidPitch.ki * pitchErr * dt, -I_LIMIT, I_LIMIT);
      iYaw   = constrain(iYaw   + pidYaw.ki   * yawErr   * dt, -I_LIMIT, I_LIMIT);
    } else {
      iRoll = iPitch = iYaw = 0;
    }

    // D uses the gyro rate directly (measurement derivative = no derivative kick).
    float rollOut  = pidRoll.kp  * rollErr  + iRoll  - pidRoll.kd  * rollRate;
    float pitchOut = pidPitch.kp * pitchErr + iPitch - pidPitch.kd * pitchRate;
    float yawOut   = pidYaw.kp   * yawErr   + iYaw;

    rollCorr  = constrain((int)rollOut,  -CORR_LIMIT, CORR_LIMIT);
    pitchCorr = constrain((int)pitchOut, -CORR_LIMIT, CORR_LIMIT);
    yawCorr   = constrain((int)yawOut,   -CORR_LIMIT, CORR_LIMIT);
  } else {
    // No IMU -> manual passthrough so it's still flyable (no auto-leveling).
    rollCorr  = constrain((int)(rollCmd  * kManual), -CORR_LIMIT, CORR_LIMIT);
    pitchCorr = constrain((int)(pitchCmd * kManual), -CORR_LIMIT, CORR_LIMIT);
    yawCorr   = constrain((int)(yawCmd   * kYaw),    -CORR_LIMIT, CORR_LIMIT);
  }

  // ===== MIX (quad-X). esc1=LeftFront esc2=LeftRear esc3=RightFront esc4=RightRear.
  // pitch: +front/-rear ; roll: +left/-right ; yaw: + on one diagonal (LF,RR).
  // If an axis fights you, flip that term's sign for ALL four motors (props OFF!).
  int motorLeftFront  = baseThrottle + pitchCorr + rollCorr + yawCorr;
  int motorLeftRear   = baseThrottle - pitchCorr + rollCorr - yawCorr;
  int motorRightFront = baseThrottle + pitchCorr - rollCorr - yawCorr;
  int motorRightRear  = baseThrottle - pitchCorr - rollCorr + yawCorr;

  // Throttle effectively off -> motors fully off (no idle creep).
  if (baseThrottle <= 1000) {
    motorLeftFront = motorLeftRear = motorRightFront = motorRightRear = 1000;
  } else {
    motorLeftFront  = constrain(motorLeftFront,  minThrottle, maxThrottle);
    motorLeftRear   = constrain(motorLeftRear,   minThrottle, maxThrottle);
    motorRightFront = constrain(motorRightFront, minThrottle, maxThrottle);
    motorRightRear  = constrain(motorRightRear,  minThrottle, maxThrottle);
  }

  // ===== ESC OUTPUT =====
  esc1.writeMicroseconds(motorLeftFront);
  esc2.writeMicroseconds(motorLeftRear);
  esc3.writeMicroseconds(motorRightFront);
  esc4.writeMicroseconds(motorRightRear);

  // ===== Auto-recover the radio if it goes silent (no reset needed) =====
  static unsigned long lastRadioKick = 0;
  if (!signalOk && millis() - lastRadioKick > 2000) {
    lastRadioKick = millis();
    radio.startListening();
  }

  // ===== STATUS (every 500 ms) =====
  static unsigned long dbg = 0;
  if (millis() - dbg > 500) {
    dbg = millis();
    Serial.printf("link:%d mpu:%d held:%d | T:%d Y:%d P:%d R:%d | ang r=%.1f p=%.1f | corr R:%d P:%d Y:%d | M:%d/%d/%d/%d\n",
      signalOk, mpuOk, heldThrottle, throttleCmd, yawCmd, pitchCmd, rollCmd,
      roll, pitch, rollCorr, pitchCorr, yawCorr,
      motorLeftFront, motorLeftRear, motorRightFront, motorRightRear);
  }

  delay(2);   // ~ fast loop; the PID uses the measured dt regardless
}
