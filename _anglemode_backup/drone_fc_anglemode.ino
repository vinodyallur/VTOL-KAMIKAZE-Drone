/*
  ESP32 Quadcopter Flight Controller - ANGLE (self-level) mode
  MPU6050 (accel+gyro) + complementary filter + PID + nRF24 + 4 ESCs

  X-configuration (top view):
            FRONT
        FL(CW)   FR(CCW)
            \     /
             \   /
              \ /
              / \
             /   \
        RL(CCW)  RR(CW)
            REAR
*/

#include <Wire.h>
#include <MPU6050.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <ESP32Servo.h>
#include <math.h>

// ---------------- Radio ----------------
RF24 radio(4, 5);                       // CE=GPIO4, CSN=GPIO5
const byte address[6] = "00001";

struct DataPacket {
  uint16_t throttle;   // 1000..2000
  int16_t  roll;       // -500..500
  int16_t  pitch;      // -500..500
  int16_t  yaw;        // -500..500
};
DataPacket rx;
unsigned long lastRadioMs = 0;
const unsigned long FAILSAFE_MS = 800;

// ---------------- IMU ----------------
MPU6050 mpu;
int16_t axr, ayr, azr, gxr, gyr, gzr;
float gxOff = 0, gyOff = 0, gzOff = 0;
float angleRoll = 0, anglePitch = 0;

// ---------------- ESCs ----------------
Servo escFL, escFR, escRL, escRR;
const int PIN_FL = 14;
const int PIN_FR = 27;
const int PIN_RL = 26;
const int PIN_RR = 25;

// ---------------- Limits ----------------
const int   MOTOR_MIN       = 1000;   // motors stopped
const int   MOTOR_IDLE      = 1100;   // armed, props spinning slowly
const int   MOTOR_MAX       = 2000;
const int   THROTTLE_CEIL   = 1800;   // headroom for corrections
const float MAX_ANGLE       = 30.0;   // max commanded tilt (deg)
const float MAX_YAW_RATE    = 150.0;  // deg/s
const float I_LIMIT         = 150.0;
const float PID_LIMIT       = 400.0;

// ---------------- PID gains (TUNE) ----------------
float Kp_roll = 1.8, Ki_roll = 0.04, Kd_roll = 0.45;
float Kp_pitch= 1.8, Ki_pitch= 0.04, Kd_pitch= 0.45;
float Kp_yaw  = 3.0, Ki_yaw  = 0.02;                 // yaw = rate control, no D

float iRoll = 0, iPitch = 0, iYaw = 0;

// ---------------- Loop timing ----------------
const unsigned long LOOP_US = 4000;   // 250 Hz
unsigned long loopTimer = 0;
float dt = 0.004;

// ---------------- State ----------------
bool armed = false;
uint16_t throttleCmd = 1000;

void resetPID() { iRoll = iPitch = iYaw = 0; }

void calibrateGyro() {
  long sx = 0, sy = 0, sz = 0;
  const int N = 2000;
  for (int i = 0; i < N; i++) {
    mpu.getMotion6(&axr, &ayr, &azr, &gxr, &gyr, &gzr);
    sx += gxr; sy += gyr; sz += gzr;
    delay(2);
  }
  gxOff = sx / (float)N;
  gyOff = sy / (float)N;
  gzOff = sz / (float)N;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  Wire.setTimeOut(10);              // don't let a flaky I2C read stall the 250 Hz loop

  mpu.initialize();                 // defaults: +/-2g, +/-250 deg/s
  bool mpuOk = mpu.testConnection();
  Serial.print("MPU6050 connection: ");
  Serial.println(mpuOk ? "OK" : "FAILED <-- check SDA=21/SCL=22 wiring, pull-ups, 3.3V, GND");

  escFL.attach(PIN_FL, 1000, 2000);
  escFR.attach(PIN_FR, 1000, 2000);
  escRL.attach(PIN_RL, 1000, 2000);
  escRR.attach(PIN_RR, 1000, 2000);
  escFL.writeMicroseconds(1000);
  escFR.writeMicroseconds(1000);
  escRL.writeMicroseconds(1000);
  escRR.writeMicroseconds(1000);
  delay(3000);                      // let ESCs arm

  Serial.println("Calibrating gyro - keep drone still...");
  calibrateGyro();

  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MIN);
  radio.setChannel(76);
  radio.openReadingPipe(0, address);
  radio.startListening();
  Serial.print("nRF24 chip connected (SPI ok?): ");
  Serial.println(radio.isChipConnected() ? "YES" : "NO  <-- check CE/CSN/SCK/MOSI/MISO + 3.3V");

  // seed angle from accelerometer
  mpu.getMotion6(&axr, &ayr, &azr, &gxr, &gyr, &gzr);
  angleRoll  = atan2(ayr, azr) * 180.0 / PI;
  anglePitch = atan2(-axr, sqrt((float)ayr * ayr + (float)azr * azr)) * 180.0 / PI;

  loopTimer = micros();
  Serial.println("=== FC READY (DISARMED) ===");
}

void loop() {
  // ----- fixed 250 Hz loop -----
  while (micros() - loopTimer < LOOP_US);
  dt = (micros() - loopTimer) / 1000000.0;
  loopTimer = micros();

  static unsigned long loopCtr = 0;
  loopCtr++;

  // ----- radio -----
  static unsigned long rxCount = 0;
  if (radio.available()) {
    radio.read(&rx, sizeof(rx));
    lastRadioMs = millis();
    rxCount++;
  }
  bool signalOk = (millis() - lastRadioMs) < FAILSAFE_MS;

  // ----- setpoints from sticks -----
  float rollSet, pitchSet, yawRateSet;
  if (signalOk) {
    throttleCmd = constrain(rx.throttle, 1000, THROTTLE_CEIL);
    rollSet    = (rx.roll  / 500.0) * MAX_ANGLE;
    pitchSet   = (rx.pitch / 500.0) * MAX_ANGLE;
    yawRateSet = (rx.yaw   / 500.0) * MAX_YAW_RATE;
  } else {
    // FAILSAFE: level out + slow descent
    rollSet = 0; pitchSet = 0; yawRateSet = 0;
    if (throttleCmd > MOTOR_IDLE) throttleCmd -= 1;   // ~250 us/s ramp down
  }

  // ----- IMU + complementary filter -----
  mpu.getMotion6(&axr, &ayr, &azr, &gxr, &gyr, &gzr);
  float gRoll  = (gxr - gxOff) / 131.0;   // deg/s
  float gPitch = (gyr - gyOff) / 131.0;
  float gYaw   = (gzr - gzOff) / 131.0;

  float accRoll  = atan2(ayr, azr) * 180.0 / PI;
  float accPitch = atan2(-axr, sqrt((float)ayr * ayr + (float)azr * azr)) * 180.0 / PI;

  angleRoll  = 0.98 * (angleRoll  + gRoll  * dt) + 0.02 * accRoll;
  anglePitch = 0.98 * (anglePitch + gPitch * dt) + 0.02 * accPitch;

  // ----- arming (stick combo, only at low throttle) -----
  static unsigned long armTimer = 0, disarmTimer = 0;
  bool thrLow = signalOk && (rx.throttle < 1050);

  if (thrLow && rx.yaw > 400) {                 // hold throttle down + yaw RIGHT 1s
    if (millis() - armTimer > 1000) { armed = true; resetPID(); }
  } else armTimer = millis();

  if (thrLow && rx.yaw < -400) {                // hold throttle down + yaw LEFT 1s
    if (millis() - disarmTimer > 1000) armed = false;
  } else disarmTimer = millis();

  if (!signalOk && throttleCmd <= MOTOR_IDLE) armed = false;  // disarm after failsafe landing

  // ----- PID -----
  // Roll (angle), derivative on gyro to avoid kick/noise
  float errRoll = rollSet - angleRoll;
  iRoll = constrain(iRoll + Ki_roll * errRoll * dt, -I_LIMIT, I_LIMIT);
  float pidRoll = constrain(Kp_roll * errRoll + iRoll - Kd_roll * gRoll, -PID_LIMIT, PID_LIMIT);

  // Pitch (angle)
  float errPitch = pitchSet - anglePitch;
  iPitch = constrain(iPitch + Ki_pitch * errPitch * dt, -I_LIMIT, I_LIMIT);
  float pidPitch = constrain(Kp_pitch * errPitch + iPitch - Kd_pitch * gPitch, -PID_LIMIT, PID_LIMIT);

  // Yaw (rate)
  float errYaw = yawRateSet - gYaw;
  iYaw = constrain(iYaw + Ki_yaw * errYaw * dt, -I_LIMIT, I_LIMIT);
  float pidYaw = constrain(Kp_yaw * errYaw + iYaw, -PID_LIMIT, PID_LIMIT);

  // ----- motor mixing + output -----
  if (armed && throttleCmd > MOTOR_IDLE) {
    int mFL = throttleCmd - pidRoll + pidPitch - pidYaw;
    int mFR = throttleCmd + pidRoll + pidPitch + pidYaw;
    int mRL = throttleCmd - pidRoll - pidPitch + pidYaw;
    int mRR = throttleCmd + pidRoll - pidPitch - pidYaw;

    escFL.writeMicroseconds(constrain(mFL, MOTOR_IDLE, MOTOR_MAX));
    escFR.writeMicroseconds(constrain(mFR, MOTOR_IDLE, MOTOR_MAX));
    escRL.writeMicroseconds(constrain(mRL, MOTOR_IDLE, MOTOR_MAX));
    escRR.writeMicroseconds(constrain(mRR, MOTOR_IDLE, MOTOR_MAX));
  } else {
    resetPID();                                   // no windup on the ground
    int out = armed ? MOTOR_IDLE : MOTOR_MIN;
    escFL.writeMicroseconds(out);
    escFR.writeMicroseconds(out);
    escRL.writeMicroseconds(out);
    escRR.writeMicroseconds(out);
  }

  // ----- debug (every ~100 ms) -----
  static unsigned long dbg = 0;
  if (millis() - dbg > 500) {
    unsigned long elapsed = millis() - dbg;
    unsigned long hz = (loopCtr * 1000) / (elapsed ? elapsed : 1);
    dbg = millis();
    loopCtr = 0;
    Serial.printf("%s link:%d rx#:%lu hz:%lu chip:%d | T:%d R:%d P:%d Y:%d | ang R:%.1f P:%.1f\n",
      armed ? "ARMED " : "DISARM", signalOk, rxCount, hz, radio.isChipConnected(),
      rx.throttle, rx.roll, rx.pitch, rx.yaw, angleRoll, anglePitch);
  }
}
