/*
  VTOL / Quad Flight Controller - classic ESP32 (RECEIVER)
  Throttle + Yaw over nRF24, MPU6050 self-leveling on roll & pitch.

  This is the known-working simple design, restored and lightly hardened
  for the hardware measured on COM9:
    - MPU6050 health is reported at boot (it read FAILED -> fix the wiring).
    - Wire timeout keeps a missing IMU from stalling the control loop.
    - Radio is pinned to 250 kbps / channel 76 so it matches the C3 transmitter
      (a data-rate mismatch is what produced rx#:0 / no packets).
    - A signal-loss failsafe drops the motors to off; the old code had none.

  Motors (X layout):  esc1 = Left Front, esc2 = Left Rear,
                      esc3 = Right Front, esc4 = Right Rear
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
int16_t ax, ay, az;
float roll, pitch;
bool mpuOk = false;

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

// ---------------- Control gains (TUNE) ----------------
const float kManual = 0.40;          // stick authority (us per stick count)
const float kYaw    = 0.40;
const float kLevel  = 3.00;          // MPU self-leveling gain
const int   CORR_LIMIT = 300;        // max us correction per axis

// Level reference captured during auto-calibration
float rollTrim = 0, pitchTrim = 0;

// ---------------- Throttle ratchet / peak-hold ----------------
// Position-based throttle (starts at 0 = motors off):
//   - raise the stick      -> motors rise and HOLD the highest level reached
//   - bring it to the middle-> motors drop to mid speed
//   - bring it fully down   -> motors turn off
const int THR_MID     = 512;         // middle of the 0..1023 throttle range
const int THR_MIDBAND = 60;          // +/- window that counts as "middle"
const int THR_OFF     = 60;          // at/below this = "full below" -> off
int heldThrottle = 0;                // level we hold (0 = off at power-up)

// Average the resting accel angles so "level" reads as zero. Only meaningful
// when the MPU is connected; skipped gracefully otherwise.
void autoCalibrate() {
  if (!mpuOk) { Serial.println("Auto-calibrate skipped (MPU not detected)."); return; }
  Serial.println("Auto-calibrating level - keep the drone still & level...");
  float r = 0, p = 0; const int N = 200;
  for (int i = 0; i < N; i++) {
    mpu.getAcceleration(&ax, &ay, &az);
    r += atan2(ay, az) * 180.0 / PI;
    p += atan2(ax, az) * 180.0 / PI;
    delay(3);
  }
  rollTrim  = r / N;
  pitchTrim = p / N;
  Serial.printf("Level trim captured: roll0=%.1f pitch0=%.1f\n", rollTrim, pitchTrim);
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

  autoCalibrate();                   // auto-level at power-up, no reset needed
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

  // ===== MPU SELF-LEVELING (only when the IMU is present) =====
  if (mpuOk) {
    mpu.getAcceleration(&ax, &ay, &az);
    roll  = atan2(ay, az) * 180.0 / PI - rollTrim;
    pitch = atan2(ax, az) * 180.0 / PI - pitchTrim;
  } else {
    roll = 0; pitch = 0;                          // no IMU -> no auto-correction
  }

  // ===== MIX: manual sticks + leveling =====
  int rollCorr  = constrain((int)(rollCmd  * kManual + (mpuOk ? roll  * kLevel : 0)), -CORR_LIMIT, CORR_LIMIT);
  int pitchCorr = constrain((int)(pitchCmd * kManual + (mpuOk ? pitch * kLevel : 0)), -CORR_LIMIT, CORR_LIMIT);
  int yawCorr   = constrain((int)(yawCmd   * kYaw), -CORR_LIMIT, CORR_LIMIT);

  int motorLeftFront  = baseThrottle + rollCorr + pitchCorr + yawCorr;
  int motorLeftRear   = baseThrottle + rollCorr + pitchCorr + yawCorr;
  int motorRightFront = baseThrottle - rollCorr + pitchCorr - yawCorr;
  int motorRightRear  = baseThrottle - rollCorr + pitchCorr - yawCorr;

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
    Serial.printf("link:%d mpu:%d held:%d | T:%d Y:%d P:%d R:%d | base:%d | M:%d/%d/%d/%d\n",
      signalOk, mpuOk, heldThrottle, throttleCmd, yawCmd, pitchCmd, rollCmd,
      baseThrottle, motorLeftFront, motorLeftRear, motorRightFront, motorRightRear);
  }

  delay(5);
}
