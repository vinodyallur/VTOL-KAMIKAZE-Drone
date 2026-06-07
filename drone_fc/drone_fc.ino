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

  Throttle (hover): direct proportional + smoothed. Hold the stick at the hover
  point and the drone holds that thrust while the PID keeps it level -> it hovers.
  (No barometer, so this holds THRUST, not a fixed altitude; trim the throttle by
  hand as the battery sags.)

  Mounting the IMU: flat & parallel to the frame, CHIP-SIDE UP (az ~ +16384 when
  level), X axis pointing to the FRONT, near the CG, on a little foam to damp
  motor vibration. Boot prints an orientation check - heed its warnings.

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

// ---------------- Throttle (direct proportional, smoothed) ----------------
// Stick position maps straight to motor thrust, so the drone HOLDS whatever
// thrust you set: raise the stick to the hover point and leave it there, and the
// PID keeps it level -> it hovers. (No barometer, so this holds THRUST, not a
// fixed altitude; nudge the throttle as the battery sags.)
const int   THR_MIN_ON        = 30;     // stick below this (~3%) = motors fully off
const float THR_SLEW_US_PER_S = 1200;   // max thrust ramp -> smooth, no surge
float baseSmooth = 1000;                // smoothed motor base (1000 = off)

// Calibrate the IMU at power-up: average the gyro to find its zero-rate bias and
// the accelerometer to find the resting (mounting) tilt. Keep the drone STILL and
// LEVEL during this. Skipped gracefully if the MPU isn't detected.
void autoCalibrate() {
  if (!mpuOk) { Serial.println("Auto-calibrate skipped (MPU not detected)."); return; }
  Serial.println("Calibrating IMU - keep the drone still & level...");

  const int N = 600;
  double sgx = 0, sgy = 0, sgz = 0;     // gyro sums (for bias)
  double sr = 0, sp = 0;                // accel-angle sums (for level trim)
  double sax = 0, say = 0, saz = 0;     // accel sums (for orientation check)
  for (int i = 0; i < N; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sgx += gx; sgy += gy; sgz += gz;
    sax += ax; say += ay; saz += az;
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

  // --- Orientation sanity check: when level, gravity must sit on +Z (az large &
  // positive). Catches an upside-down or sideways mount BEFORE you take off. ---
  double mAx = sax / N, mAy = say / N, mAz = saz / N;
  if (mAz < 0)
    Serial.println("WARNING: MPU UPSIDE DOWN (az<0) -> roll/pitch inverted = FLIP RISK. "
                   "Mount it chip-side UP.");
  else if (fabs(mAx) > fabs(mAz) || fabs(mAy) > fabs(mAz))
    Serial.println("WARNING: MPU not flat (a side axis sees gravity) -> mount it parallel "
                   "to the frame.");
  else
    Serial.println("Orientation OK: gravity on +Z (flat, right-side up).");
}

void setup() {
  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(400000);
  Wire.setTimeOut(10);             // a dead IMU must not stall the loop

  // --- I2C bus scan: tells us exactly what (if anything) is wired up ---
  Serial.println("Scanning I2C bus...");
  uint8_t mpuAddr = 0;
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      found++;
      Serial.printf("  found I2C device @ 0x%02X\n", a);
      if (a == 0x68 || a == 0x69) mpuAddr = a;   // MPU6050 lives at 0x68 (AD0=GND) or 0x69 (AD0=VCC)
    }
  }
  if (found == 0)
    Serial.println("  no I2C devices! -> check SDA=21, SCL=22 (not swapped), VCC=3.3V, GND, pull-ups");

  // --- MPU init at the detected address (fall back to 0x68) ---
  if (mpuAddr == 0) mpuAddr = 0x68;

  // Read raw WHO_AM_I (reg 0x75) BEFORE init so we know what chip this is.
  // Many "MPU6050" modules are actually MPU6500/MPU9250 (WHO_AM_I 0x70/0x71).
  // They share the same accel/gyro data registers and scaling, so they work
  // fine for us even though the MPU6050 library's testConnection() rejects them.
  uint8_t whoami = 0xFF;
  Wire.beginTransmission(mpuAddr);
  Wire.write(0x75);
  if (Wire.endTransmission(false) == 0 && Wire.requestFrom((int)mpuAddr, 1) == 1)
    whoami = Wire.read();

  mpu = MPU6050(mpuAddr);
  mpu.initialize();
  // Force the chip out of sleep (PWR_MGMT_1=0x6B -> 0x00) in case a clone's
  // initialize() left the sleep bit set.
  Wire.beginTransmission(mpuAddr);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  bool answered = (whoami != 0x00 && whoami != 0xFF);
  bool knownId   = (whoami == 0x68 || whoami == 0x70 || whoami == 0x71 ||
                    whoami == 0x73 || whoami == 0x75 || whoami == 0x98);
  mpuOk = mpu.testConnection() || (answered && knownId);

  const char* chip = "unknown";
  if (whoami == 0x68) chip = "MPU6050";
  else if (whoami == 0x70) chip = "MPU6500";
  else if (whoami == 0x71) chip = "MPU9250";
  else if (whoami == 0x73) chip = "MPU9255";
  else if (whoami == 0x98) chip = "MPU6886";
  Serial.printf("MPU @ 0x%02X  WHO_AM_I=0x%02X (%s)  -> %s\n",
                mpuAddr, whoami, chip, mpuOk ? "OK" : "FAILED");
  if (mpuOk) {
    int16_t tax, tay, taz, tgx, tgy, tgz;
    mpu.getMotion6(&tax, &tay, &taz, &tgx, &tgy, &tgz);
    Serial.printf("  sample ax=%d ay=%d az=%d gx=%d gy=%d gz=%d (az~16384 when level)\n",
                  tax, tay, taz, tgx, tgy, tgz);
  } else if (whoami == 0xFF) {
    Serial.println("  chip not answering -> wiring/power (SDA=21, SCL=22, 3.3V, GND, pull-ups)");
  } else {
    Serial.println("  unrecognized WHO_AM_I -> tell me this value so I can add support");
  }

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
    baseSmooth = 1000;                    // cut thrust immediately (no ramp)
    iRoll = iPitch = iYaw = 0;            // dump PID integrals so it can't wind up
  }

  // ===== TIMING: real elapsed dt (shared by throttle smoothing, filter, PID) =====
  unsigned long nowUs = micros();
  float dt = (nowUs - lastLoopUs) * 1e-6f;
  lastLoopUs = nowUs;
  if (dt <= 0 || dt > 0.05f) dt = 0.005f;     // guard against a bad first / long step

  // ===== THROTTLE -> motor base (direct proportional, smoothed) =====
  // Stick position = thrust. Leave the stick at the hover point and the drone
  // holds that thrust while the PID keeps it level -> it hovers. The slew limit
  // eases big stick moves so it settles instead of surging.
  int targetBase;
  if (throttleCmd < THR_MIN_ON) targetBase = 1000;                       // off
  else targetBase = map(throttleCmd, THR_MIN_ON, 1023, minThrottle, maxThrottle);

  float maxStep = THR_SLEW_US_PER_S * dt;     // most the thrust may change this frame
  if      (targetBase > baseSmooth + maxStep) baseSmooth += maxStep;
  else if (targetBase < baseSmooth - maxStep) baseSmooth -= maxStep;
  else                                        baseSmooth  = targetBase;
  int baseThrottle = (int)baseSmooth;
  bool flying = baseThrottle > minThrottle;   // only stabilize once actually spun up

  // ===== IMU: read accel + gyro, fuse into clean angles =====
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
    Serial.printf("link:%d mpu:%d base:%d | T:%d Y:%d P:%d R:%d | ang r=%.1f p=%.1f | corr R:%d P:%d Y:%d | M:%d/%d/%d/%d\n",
      signalOk, mpuOk, baseThrottle, throttleCmd, yawCmd, pitchCmd, rollCmd,
      roll, pitch, rollCorr, pitchCorr, yawCorr,
      motorLeftFront, motorLeftRear, motorRightFront, motorRightRear);
  }

  delay(2);   // ~ fast loop; the PID uses the measured dt regardless
}
