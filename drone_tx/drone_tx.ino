/*
  VTOL / Quad Transmitter - ESP32-C3 Supermini
  Throttle + Yaw -> nRF24 -> flight controller.

  This is the known-working simple transmitter, re-pinned for the C3 Supermini
  that is the actual transmitter on COM36. The old code used a classic ESP32's
  GPIO34/35 and the default SPI bus - neither of which exists on the C3 - so the
  joystick inputs and the radio SPI are moved to valid C3 pins here. The packet
  format and radio settings are unchanged so it talks to the same receiver.
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// ---- nRF24L01 on the C3 Supermini (custom SPI pins) ----
#define NRF_SCK   5    // GPIO5  -> nRF24 SCK
#define NRF_MISO  6    // GPIO6  -> nRF24 MISO
#define NRF_MOSI  7    // GPIO7  -> nRF24 MOSI
#define NRF_CE    10   // GPIO10 -> nRF24 CE
#define NRF_CSN   20   // GPIO20 -> nRF24 CSN

RF24 radio(NRF_CE, NRF_CSN);
const byte address[6] = "00001";

struct DataPacket {
  uint16_t throttle;   // 0..1023 (not centered)
  int16_t  yaw;        // -512..511 (centered)
  int16_t  pitch;      // -512..511 (centered)
  int16_t  roll;       // -512..511 (centered)
};
DataPacket data;

// Joystick wipers on ADC1 pins (power the pots from 3.3V, NOT 5V).
//
//  Throttle is on GPIO4 (a safe ADC pin). It used to be on GPIO2, but GPIO2 is
//  a C3 strapping pin: if the throttle pot pulled it low at reset, the board
//  could fail to boot and calibration never ran. GPIO4 has no such restriction,
//  so the board boots and calibrates on every reset.
const int PIN_THR   = 4;   // GPIO4  ADC1_CH4  throttle  (left stick - vertical)
const int PIN_YAW   = 3;   // GPIO3  ADC1_CH3  yaw       (left stick - horizontal)
const int PIN_PITCH = 1;   // GPIO1  ADC1_CH1  pitch     (right stick - vertical)
const int PIN_ROLL  = 0;   // GPIO0  ADC1_CH0  roll      (right stick - horizontal)

// Onboard LED (GPIO8, active LOW on the C3 Supermini). Used as a visual
// calibration heartbeat because the C3's USB serial is hard to read live.
#define LED_PIN 8

// Captured resting positions of the centered sticks (yaw/pitch/roll).
int cYaw = 2048, cPitch = 2048, cRoll = 2048;

// Throttle mapping is FIXED to the measured hardware: the stick rests at the
// HIGH end (raw ~4095) and pushing it up drops the voltage toward 0. We map it
// reversed so idle = 0 throttle (motors off) and full-up = 1023. This does NOT
// depend on stick position at reset, so every boot behaves identically.
//   If your throttle ever reads backwards (full at idle, off at top), just swap
//   these two numbers.
const int THR_IDLE_RAW = 4095;   // raw at idle (stick down) -> 0
const int THR_FULL_RAW = 0;      // raw at full (stick up)   -> 1023

// Map a centered stick around its calibrated middle -> -512..511 + deadband.
int16_t axisCentered(int pin, int center) {
  int raw = analogRead(pin);
  long v;
  if (raw >= center) { int s = 4095 - center; if (s < 1) s = 1; v = (long)(raw - center) * 512 / s; }
  else               { int s = center;        if (s < 1) s = 1; v = (long)(raw - center) * 512 / s; }
  if (v > -25 && v < 25) v = 0;                 // deadband around center
  return (int16_t)constrain(v, -512L, 511L);
}

// Learn the resting position of the centered sticks at power-up.
void calibrateSticks() {
  const int N = 64; long y = 0, p = 0, r = 0;
  for (int i = 0; i < N; i++) {
    y += analogRead(PIN_YAW); p += analogRead(PIN_PITCH); r += analogRead(PIN_ROLL);
    delay(2);
  }
  cYaw = y / N; cPitch = p / N; cRoll = r / N;
}

void initRadio() {
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(76);
  radio.setPALevel(RF24_PA_MIN);
  radio.openWritingPipe(address);
  radio.stopListening();
}

// Blink the onboard LED: LOW = on, HIGH = off (active-low LED).
void blinkLed(int times, int onMs, int offMs) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, LOW);  delay(onMs);
    digitalWrite(LED_PIN, HIGH); delay(offMs);
  }
}

void setup() {
  Serial.begin(115200);

  // LED on solid while we boot + calibrate (so a reset is visible).
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Pull the throttle pin up: idle is the HIGH end, so a loose/disconnected
  // throttle wire reads idle (=0) and the motors stay OFF instead of floating.
  pinMode(PIN_THR, INPUT_PULLUP);

  // Route SPI to the C3 pins first, then start the radio on that bus.
  SPI.begin(NRF_SCK, NRF_MISO, NRF_MOSI, NRF_CSN);
  initRadio();
  Serial.print("nRF24 chip connected (SPI ok?): ");
  Serial.println(radio.isChipConnected() ? "YES" : "NO  <-- check CE/CSN/SCK/MOSI/MISO + 3.3V");

  // Auto-calibrate the centered sticks: keep yaw/pitch/roll centered at boot.
  calibrateSticks();
  Serial.printf("Stick centers: yaw=%d pitch=%d roll=%d | throttle idle_raw=%d full_raw=%d\n",
    cYaw, cPitch, cRoll, THR_IDLE_RAW, THR_FULL_RAW);

  // Three quick blinks = calibration finished, transmitter is live.
  blinkLed(3, 120, 120);
}

void loop() {
  // Throttle: idle (raw ~THR_IDLE_RAW) = 0, full (raw ~THR_FULL_RAW) = 1023.
  int rawThr = analogRead(PIN_THR);
  int thr = map(rawThr, THR_IDLE_RAW, THR_FULL_RAW, 0, 1023);
  thr = constrain(thr, 0, 1023);
  if (thr < 30) thr = 0;                 // idle deadband so the motors stay off
  data.throttle = thr;
  data.yaw      = axisCentered(PIN_YAW,   cYaw);
  data.pitch    = axisCentered(PIN_PITCH, cPitch);
  data.roll     = axisCentered(PIN_ROLL,  cRoll);

  bool ok = radio.write(&data, sizeof(data));   // true only when the FC ACKs

  // Auto-reconnect: if the link stays dead, re-init the radio (no reset needed).
  static unsigned long okCount = 0, failCount = 0;
  static unsigned int  consecFail = 0;
  if (ok) { okCount++; consecFail = 0; }
  else    { failCount++; if (++consecFail > 250) { initRadio(); consecFail = 0; } }

  static unsigned long dbg = 0;
  if (millis() - dbg > 200) {
    dbg = millis();
    Serial.printf("rawT:%d T:%d Y:%d P:%d R:%d | chip:%d ackOK:%lu fail:%lu\n",
      rawThr, data.throttle, data.yaw, data.pitch, data.roll,
      radio.isChipConnected(), okCount, failCount);
  }

  delay(20);   // 50 Hz
}
