param(
    [Parameter(Mandatory = $true)][string]$Port,
    [int]$Baud = 115200,
    [int]$Seconds = 6,
    # ESP32-C3 native USB-Serial/JTAG resets when DTR/RTS are asserted, which
    # drops the handle on re-enumeration. Use -NoReset to read it passively.
    [switch]$NoReset
)

$p = New-Object System.IO.Ports.SerialPort($Port, $Baud)
$p.ReadTimeout = 1500
$p.NewLine = "`n"
if (-not $NoReset) {
    $p.DtrEnable = $true
    $p.RtsEnable = $true
}
try {
    $p.Open()
}
catch {
    Write-Host "OPEN FAILED on $Port : $($_.Exception.Message)"
    return
}
Start-Sleep -Milliseconds 300
Write-Host "==== $Port @ $Baud baud, capturing $Seconds s ===="
try { $p.DiscardInBuffer() } catch { }   # drop stale/partial bytes
$end = (Get-Date).AddSeconds($Seconds)
while ((Get-Date) -lt $end) {
    try {
        $line = $p.ReadLine()
        if ($line.Trim().Length -gt 0) { Write-Host ($line.TrimEnd("`r")) }
    } catch { }
}
$p.Close()
Write-Host "==== $Port capture done ===="
