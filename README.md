# VTOL KAMIKAZE Drone

A two-board ESP32 quadcopter / VTOL build:

- **Transmitter (TX):** ESP32-C3 Supermini — reads four joystick pots and sends
  4-axis commands over an nRF24L01 radio.
- **Flight Controller (FC):** classic ESP32 — receives the commands, self-levels
  with an MPU6050 IMU, and drives four ESCs/motors.

> ⚠️ **Always test with the propellers OFF.** Treat every power-up as live.

## Repository layout

| Path | Purpose |
|------|---------|
| `drone_fc/drone_fc.ino` | Flight-controller (receiver) firmware — classic ESP32 |
| `drone_tx/drone_tx.ino` | Transmitter firmware — ESP32-C3 Supermini |
| `flash.ps1` | Flash a compiled sketch to the correct board (auto-detects by USB VID) |
| `tools/read_serial.ps1` | Serial-monitor helper |
| `_anglemode_backup/` | Earlier advanced angle-mode PID sketches (reference) |

Build outputs and the `arduino-cli.exe` binary are intentionally **not** checked
in (see `.gitignore`).

## Hardware

### Boards
| Role | Board | USB bridge | Identify by |
|------|-------|-----------|-------------|
| FC | Classic ESP32 | CP210x (VID `0x10C4`) | USB VID, **not** COM-port order |
| TX | ESP32-C3 Supermini | native USB-Serial/JTAG (VID `0x303A`) | USB VID |

### Flight controller (classic ESP32) pins
- **nRF24L01:** CE = GPIO4, CSN = GPIO5 (default VSPI for SCK/MISO/MOSI)
- **MPU6050 (I2C):** SDA = GPIO21, SCL = GPIO22, 3.3 V, GND (needs pull-ups)
- **ESCs:** GPIO14 = Left-Front, GPIO27 = Left-Rear, GPIO26 = Right-Front,
  GPIO25 = Right-Rear

### Transmitter (ESP32-C3 Supermini) pins
- **nRF24L01:** SCK = GPIO5, MISO = GPIO6, MOSI = GPIO7, CE = GPIO10, CSN = GPIO20
- **Joysticks (ADC1, powered from 3.3 V — *not* 5 V):**
  throttle = GPIO4, yaw = GPIO3, pitch = GPIO1, roll = GPIO0

### Radio settings (must match on BOTH boards)
- Data rate `RF24_250KBPS`, channel `76`, power `RF24_PA_MIN`, address `"00001"`

## Control behavior

- **8-byte packet (identical on both MCUs):**
  `{ uint16 throttle (0..1023); int16 yaw; int16 pitch; int16 roll; }`
  yaw/pitch/roll are centered (−512..511).
- **Throttle (ratchet / peak-hold):** starts at 0 (motors off). Raising the stick
  increases the motors and *holds* the highest level reached; bringing the stick
  to the middle drops to mid speed; bringing it fully down turns the motors off.
- **Throttle mapping** is fixed so idle = 0 (motors off at every power-up), with a
  pull-up so a disconnected throttle wire reads idle instead of floating.
- **Self-leveling:** the FC uses the MPU6050 to correct roll & pitch. With the IMU
  absent, those corrections are forced to 0 (manual stick control still works).
- **Failsafe:** if the radio link drops, the FC cuts throttle and sticks to off.
- **Auto-reconnect:** the TX re-inits the radio after repeated write failures —
  no reset required.

## Build & flash (Windows / PowerShell)

These scripts expect `arduino-cli.exe` in `tools/` (download it from
<https://arduino.github.io/arduino-cli/> and drop it in `tools/`). Required cores
& libraries: esp32 core 3.x, ESP32Servo, MPU6050, RF24.

```powershell
# Compile
./tools/arduino-cli.exe compile --fqbn esp32:esp32:esp32   --output-dir build_fc --clean drone_fc
./tools/arduino-cli.exe compile --fqbn esp32:esp32:esp32c3 --output-dir build_tx --clean drone_tx

# Flash (auto-detects the right COM port by USB VID)
./flash.ps1 fc
./flash.ps1 tx
```

> The classic ESP32 can corrupt uploads at 921600 baud; `flash.ps1` uses
> `UploadSpeed=115200` for the FC. The C3 flashes fine at 921600 over native USB.

## Known notes / troubleshooting

- **MPU6050 reads `FAILED`** → check SDA = 21 / SCL = 22 wiring, pull-ups, 3.3 V,
  GND. Without it, self-leveling is disabled but manual control still works.
- The C3's native USB resets when the serial port is opened; verify TX behavior
  indirectly via the FC's received values (`link:1`, `T:` …) rather than reading
  the C3 directly.
- If pitch/roll feel swapped, exchange `PIN_PITCH` / `PIN_ROLL` in
  `drone_tx/drone_tx.ino`.
