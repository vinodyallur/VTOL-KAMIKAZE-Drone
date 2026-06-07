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
| `docs/WIRING.md` | **Full wiring & connection diagrams** (both boards + power) |
| `flash.ps1` | Flash a compiled sketch to the correct board (auto-detects by USB VID) |
| `tools/read_serial.ps1` | Serial-monitor helper |
| `_anglemode_backup/` | Earlier advanced angle-mode PID sketches (reference) |

Build outputs and the `arduino-cli.exe` binary are intentionally **not** checked
in (see `.gitignore`).

## Hardware

> 📐 **Full wiring tables + connection diagrams (with power distribution) are in
> [`docs/WIRING.md`](docs/WIRING.md).** The quick reference below is a summary.

### Boards
| Role | Board | USB bridge | Identify by |
|------|-------|-----------|-------------|
| FC | Classic ESP32 | CP210x (VID `0x10C4`) | USB VID, **not** COM-port order |
| TX | ESP32-C3 Supermini | native USB-Serial/JTAG (VID `0x303A`) | USB VID |

### Flight controller (classic ESP32) pins
- **nRF24L01:** CE = GPIO4, CSN = GPIO5 (default VSPI: SCK = GPIO18, MISO = GPIO19, MOSI = GPIO23)
- **MPU6050 (I2C):** SDA = GPIO21, SCL = GPIO22, AD0 = GND, 3.3 V, GND (needs pull-ups)
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
- **Throttle (hover — direct proportional):** stick position maps straight to
  motor thrust, smoothed by a slew limit so big moves don't surge. Park the stick
  at the hover point and the drone holds that thrust while the PID keeps it level
  → it hovers. Below ~3 % stick the motors are fully off. (No barometer, so this
  holds **thrust**, not a fixed altitude — trim by hand as the battery sags.)
- **Throttle mapping** is fixed on the TX so idle = 0 (motors off at every
  power-up), with a pull-up so a disconnected throttle wire reads idle.
- **Stabilization (PID + MPU6050):** the FC fuses the accelerometer and gyro with
  a complementary filter for clean roll/pitch angles, then runs **angle-mode PID**
  on roll & pitch (stick = desired lean angle, the IMU holds it) and **rate-mode
  PID** on yaw. Gyro bias and mounting tilt are auto-calibrated at boot (keep the
  drone still & level). Tune `pidRoll` / `pidPitch` / `pidYaw` in
  `drone_fc/drone_fc.ino`. With the IMU absent it falls back to manual stick
  passthrough so it still flies (no auto-leveling).
- **Failsafe:** if the radio link drops, the FC cuts throttle and sticks to off
  and dumps the PID integrators.
- **Auto-reconnect:** the TX re-inits the radio after repeated write failures —
  no reset required.

### PID tuning (props OFF!)
1. Start with the defaults. With props off, tilt the drone by hand and watch the
   serial `ang r= p=` and `corr R: P: Y:` values — the corrections should oppose
   the tilt.
2. Raise `kp` until it holds firmly / fights tilt strongly; if it oscillates,
   back off ~20%.
3. Add `kd` to damp the wobble/overshoot.
4. Add a little `ki` to remove slow drift. The integral is clamped and reset on
   failsafe to prevent windup.

## Known notes / troubleshooting

### Upload fails: "chip stopped responding" / "No serial data received"
The classic ESP32's auto-reset into download mode can fail (especially with extra
load on the 3.3 V rail). If `flash.ps1 fc` keeps failing to connect:
- **Hold the BOOT (IO0) button** on the FC, start the flash, and release BOOT
  once you see writing %. This is the reliable fix.
- Make sure **nothing is wired to GPIO0 or GPIO2** (strapping pins) — a stray
  wire there blocks download mode.
- If it flashed fine before and stopped after adding a peripheral, temporarily
  **unplug that peripheral's VCC**, flash, then reconnect.

### MPU6050 reads `FAILED` (`mpu:0`)
The firmware runs an **I2C bus scan at boot** — open the serial monitor right
after reset to see it. Common cases:
- **No I2C devices found** → wiring/power: **SDA = GPIO21, SCL = GPIO22,
  VCC = 3.3V, GND = GND**, tie **AD0 = GND** (address `0x68`), and confirm SDA &
  SCL **pull-ups** to 3.3V (most GY-521 breakouts have them). SDA/SCL not swapped.
- **Device found at 0x68 but `WHO_AM_I` ≠ 0x68** → your "MPU6050" is really an
  **MPU6500 (0x70)**, MPU9250 (0x71), etc. These are register-compatible, so the
  firmware now **auto-accepts** WHO_AM_I `0x68/0x70/0x71/0x73/0x75/0x98` and reads
  them normally. If you see an unrecognized value, report it and it can be added.
- Without a working IMU the firmware still flies in manual passthrough, but **PID
  stabilization is disabled**.

> **Pre-flight (props OFF!):** after `mpu:1`, tilt the drone by hand and watch the
> `M:` motor values — the motors on the **low** side must speed up to push it back
> level. If they speed up on the *high* side instead, the correction sign is
> inverted (you'd flip on takeoff) — swap the affected axis in the mixing.

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

## More notes

- The C3's native USB resets when the serial port is opened; verify TX behavior
  indirectly via the FC's received values (`link:1`, `T:` …) rather than reading
  the C3 directly.
- If pitch/roll feel swapped, exchange `PIN_PITCH` / `PIN_ROLL` in
  `drone_tx/drone_tx.ino`.
