/*
  ESP32-C3 Supermini Quadcopter Transmitter
  4-axis stick input -> nRF24 -> flight controller

  NOTE: The C3 only has GPIO0..GPIO5 as ADC pins, and GPIO2/8/9 are
  strapping pins (avoid for sticks). The nRF24 runs on custom SPI pins so
  all four pots can stay on safe ADC1 channels.
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
  uint16_t throttle;   // 1000..2000
  int16_t  roll;       // -500..500
  int16_t  pitch;      // -500..500
  int16_t  yaw;        // -500..500
};
DataPacket tx;

// Joystick wipers -> ADC1 pins (power the pots from 3.3V, NOT 5V).
// All on ADC1 and clear of strapping pins so the board always boots.
const int PIN_THR   = 0;    // GPIO0  ADC1_CH0  left stick  - vertical
const int PIN_YAW   = 1;    // GPIO1  ADC1_CH1  left stick  - horizontal
const int PIN_PITCH = 3;    // GPIO3  ADC1_CH3  right stick - vertical
const int PIN_ROLL  = 4;    // GPIO4  ADC1_CH4  right stick - horizontal

// Resting (center) ADC reading per axis, captured at boot. Sticks MUST be
// centered when the board powers up. Throttle is NOT centered.
int rawCenterRoll  = 2048;
int rawCenterPitch = 2048;
int rawCenterYaw   = 2048;

// Map a stick around its captured center -> -500..500 with a deadband.
// Per-side scaling so an off-center resting point still gives full range.
int16_t centeredCal(int pin, int center) {
  int  raw = analogRead(pin);
  long v;
  if (raw >= center) {
    int span = 4095 - center; if (span < 1) span = 1;
    v = (long)(raw - center) * 500 / span;
  } else {
    int span = center; if (span < 1) span = 1;
    v = (long)(raw - center) * 500 / span;
  }
  if (v < 25 && v > -25) v = 0;
  return (int16_t)constrain(v, -500L, 500L);
}

// Average several reads of each stick at rest to learn its center.
void captureCenters() {
  const int N = 64;
  long r = 0, p = 0, y = 0;
  for (int i = 0; i < N; i++) {
    r += analogRead(PIN_ROLL);
    p += analogRead(PIN_PITCH);
    y += analogRead(PIN_YAW);
    delay(2);
  }
  rawCenterRoll  = r / N;
  rawCenterPitch = p / N;
  rawCenterYaw   = y / N;
}

void setup() {
  Serial.begin(115200);

  // Route the SPI bus to the C3 Supermini pins, THEN start the radio.
  // (RF24.begin() reuses an already-initialised SPI bus, so these pins stick.)
  SPI.begin(NRF_SCK, NRF_MISO, NRF_MOSI, NRF_CSN);

  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MIN);
  radio.setChannel(76);
  radio.openWritingPipe(address);
  radio.stopListening();
  Serial.print("nRF24 chip connected (SPI ok?): ");
  Serial.println(radio.isChipConnected() ? "YES" : "NO  <-- check CE/CSN/SCK/MOSI/MISO + 3.3V");

  // Learn each stick's resting position (keep R/P/Y sticks centered at boot).
  captureCenters();
  Serial.printf("Stick centers (raw ADC): roll=%d pitch=%d yaw=%d\n",
    rawCenterRoll, rawCenterPitch, rawCenterYaw);
}

void loop() {
  // If pushing the throttle UP gives a LOW number, swap 1000<->2000 here.
  tx.throttle = map(analogRead(PIN_THR), 0, 4095, 1000, 2000);
  tx.roll  = centeredCal(PIN_ROLL,  rawCenterRoll);
  tx.pitch = centeredCal(PIN_PITCH, rawCenterPitch);
  tx.yaw   = centeredCal(PIN_YAW,   rawCenterYaw);

  bool ok = radio.write(&tx, sizeof(tx));   // returns true only if FC ACKs

  static unsigned long okCount = 0, failCount = 0;
  if (ok) okCount++; else failCount++;

  static unsigned long dbg = 0;
  if (millis() - dbg > 200) {
    dbg = millis();
    Serial.printf("T:%d R:%d P:%d Y:%d | chip:%d ackOK:%lu fail:%lu\n",
      tx.throttle, tx.roll, tx.pitch, tx.yaw,
      radio.isChipConnected(), okCount, failCount);
  }
  delay(20);   // 50 Hz
}
