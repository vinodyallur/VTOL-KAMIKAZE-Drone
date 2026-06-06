# flash.ps1 - Flash a compiled ESP32 sketch (.bin) to the board.
#
# Boards: fc = flight controller (classic ESP32),
#         tx = remote/transmitter (ESP32-C3 Supermini).
#
# Usage examples (run from the 'drone' folder):
#   .\flash.ps1 fc                 # auto-detect COM port, flash flight controller
#   .\flash.ps1 tx                 # auto-detect COM port, flash transmitter (C3)
#   .\flash.ps1 fc -Port COM5      # flash flight controller on COM5
#
# This uploads the ALREADY-COMPILED binaries in build_fc\ or build_tx\
# (no recompile) using arduino-cli's bundled esptool.

param(
    [Parameter(Mandatory = $true)]
    [ValidateSet('fc', 'tx')]
    [string]$Target,

    [string]$Port,

    [string]$Fqbn,

    # Classic-ESP32 uploads over the CP210x can corrupt at the 921600 default
    # ("Serial data stream stopped"). 115200 is slow but reliable. The C3 (tx)
    # uses native USB and ignores this.
    [int]$UploadSpeed = 115200
)

$ErrorActionPreference = 'Stop'
$cli = Join-Path $PSScriptRoot 'tools\arduino-cli.exe'
if (-not (Test-Path $cli)) { throw "arduino-cli not found at $cli" }

# Pick the right board automatically: the flight controller is a classic
# ESP32, the transmitter (remote) is an ESP32-C3 Supermini.
if (-not $Fqbn) {
    if ($Target -eq 'tx') { $Fqbn = 'esp32:esp32:esp32c3' }
    else                  { $Fqbn = 'esp32:esp32:esp32' }
}

# For the classic ESP32 (fc), force a reliable upload baud unless the caller
# already pinned one in a custom -Fqbn. The C3 flashes over native USB, so we
# leave its FQBN untouched.
if ($Target -eq 'fc' -and $Fqbn -eq 'esp32:esp32:esp32') {
    $Fqbn = "$Fqbn`:UploadSpeed=$UploadSpeed"
}

$buildDir = Join-Path $PSScriptRoot ("build_" + $Target)
if (-not (Test-Path $buildDir)) { throw "Build folder '$buildDir' not found. Compile first." }

# Auto-detect the COM port if not supplied.
#
# With BOTH boards plugged in we must pick the right one per target, not just
# the first serial port. They are told apart by their USB vendor ID:
#   tx = ESP32-C3 Supermini -> native USB        -> VID 0x303A (Espressif)
#   fc = classic ESP32      -> CP210x USB-UART   -> VID 0x10C4 (Silicon Labs)
if (-not $Port) {
    $found = & $cli board list --format json | ConvertFrom-Json
    $serial = @($found.detected_ports | Where-Object { $_.port.protocol -eq 'serial' })
    if ($serial.Count -eq 0) { throw "No serial port detected. Plug in the ESP32 or pass -Port COMx." }

    $wantVid = if ($Target -eq 'tx') { '0x303A' } else { '0x10C4' }
    $match = $serial | Where-Object { $_.port.properties.vid -ieq $wantVid } | Select-Object -First 1

    if (-not $match) {
        if ($serial.Count -eq 1) {
            $match = $serial[0]
            Write-Host "No USB VID match for '$Target'; only one port present, using it." -ForegroundColor Yellow
        }
        else {
            $list = ($serial | ForEach-Object { "$($_.port.address) (vid $($_.port.properties.vid))" }) -join ', '
            throw "Could not pick a port for '$Target' by USB VID ($wantVid). Ports seen: $list. Pass -Port COMx explicitly."
        }
    }
    $Port = $match.port.address
    Write-Host "Auto-detected port for '$Target': $Port"
}

Write-Host "Flashing '$Target' to $Port ..." -ForegroundColor Cyan
& $cli upload -p $Port --fqbn $Fqbn --input-dir $buildDir
Write-Host "Done. Open the serial monitor at 115200 baud to view output." -ForegroundColor Green
