# flash-fw.ps1 - Flash a built firmware onto the device from the HOST using esptool (open-source,
# pure Python - a tiny, auditable trust surface). Build first with build-fw.ps1.
#
#   .\flash-fw.ps1 -Chip esp32c3                 # auto-detects the COM port if exactly one
#   .\flash-fw.ps1 -Chip esp32c3 -Port COM4      # or specify it
#   .\flash-fw.ps1 -Chip esp32c6 -Serial usb -Port COM16   # flash the USB/JTAG (GND-over-USB) image
#   .\flash-fw.ps1 -Chip esp32c3 -Role air -Port COM7      # provision a role-baked JONOKR unit
#   .\flash-fw.ps1 -Chip esp32c6 -Port COM7 -Erase   # also wipe saved config/NVS (factory reset)
#
# -Role air|gnd|beacon flashes the role image from build\<chip>-<role>\ (build with
# build-fw.ps1 -Role ...) and ALWAYS erases first: the role's boot defaults only take
# effect on a clean NVS — that's the whole point of provisioning.
#
# Requires on host:  pip install "esptool<5"
# (No-Python option: use the browser flasher .\webflasher.ps1 instead - nothing to install.)
#requires -Version 5
[CmdletBinding()]
param(
    [Parameter(Mandatory)][ValidateSet('esp32c3', 'esp32c6', 'esp32s3')][string]$Chip,
    [ValidateSet('uart', 'usb')][string]$Serial = 'uart',
    [ValidateSet('air', 'gnd', 'beacon')][string]$Role,
    [string]$Port,
    [int]$Baud = 460800,
    [switch]$Erase
)

if ($Role) { $Erase = $true }   # role provisioning requires factory-fresh NVS

function Fail($msg) { Write-Host "ERROR: $msg" -ForegroundColor Red; exit 1 }

$repo     = $PSScriptRoot
# Role images live in build\<chip>-<role>\, USB/JTAG-serial images in build\<chip>-usb\.
$variant  = if ($Role) { "$Chip-$Role" } elseif ($Serial -eq 'usb') { "$Chip-usb" } else { $Chip }
$buildDir = Join-Path $repo "build\$variant"
if (-not (Test-Path (Join-Path $buildDir 'flash_args'))) {
    $hint = if ($Role) { ".\build-fw.ps1 -Chips $Chip -Role $Role" }
            elseif ($Serial -eq 'usb') { ".\build-fw.ps1 -Chips $Chip -Serial usb" }
            else { ".\build-fw.ps1 -Chips $Chip" }
    Fail "No build at $buildDir. Run:  $hint"
}

# esptool available on the host?
if (-not (Get-Command python -ErrorAction SilentlyContinue)) {
    Fail "Python not found on PATH. Install Python 3, then:  pip install `"esptool<5`""
}
python -m esptool version 2>&1 | Out-Null
if ($LASTEXITCODE -ne 0) { Fail "esptool is not installed for this Python. Run:  pip install `"esptool<5`"" }

# resolve the COM port
if (-not $Port) {
    $ports = @([System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object -Unique)
    if     ($ports.Count -eq 0) { Fail "No serial ports found. Plug the board in via USB (install its USB-serial driver if needed)." }
    elseif ($ports.Count -eq 1) { $Port = $ports[0]; Write-Host "Using serial port $Port" -ForegroundColor Cyan }
    else   { Fail "Multiple serial ports found ($($ports -join ', ')). Re-run with -Port <COMx>." }
}

Push-Location $buildDir
try {
    if ($Erase) {
        Write-Host "==> [$Chip] erasing flash (wipes saved WiFi/config in NVS) ..." -ForegroundColor Yellow
        python -m esptool --chip $Chip -p $Port erase_flash
        if ($LASTEXITCODE -ne 0) { Pop-Location; Fail "erase_flash failed (exit $LASTEXITCODE)." }
    }
    Write-Host "==> [$Chip] writing firmware on $Port ..." -ForegroundColor Cyan
    # "@flash_args" must be QUOTED so PowerShell passes it literally to esptool (an unquoted
    # @name is PowerShell splatting). esptool reads it: flash params + each .bin at its offset.
    python -m esptool --chip $Chip -p $Port -b $Baud `
        --before default_reset --after hard_reset write_flash "@flash_args"
    $code = $LASTEXITCODE
}
finally { Pop-Location }

if ($code -ne 0) {
    Write-Host "`nFlash failed. If it hung at 'Connecting....', put the chip in download mode:" -ForegroundColor Red
    Write-Host "  hold BOOT (IO0), tap/replug RST, keep holding BOOT a moment, then re-run." -ForegroundColor Red
    exit 1
}
if ($Role) {
    $roleMsg = switch ($Role) {
        'air'    { "AIR unit ready - wire the UART to the flight controller; it joins the ESP-NOW net automatically." }
        'gnd'    { "GND unit ready - plug it into the GCS computer over USB-C; it appears as a COM port." }
        'beacon' { "Beacon ready - wire the UART to the u-blox GPS (115200); position streams to the GCS automatically." }
    }
    Write-Host "`n==> [$Chip/$Role] flashed on $Port. $roleMsg" -ForegroundColor Green
} else {
    Write-Host "`n==> [$Chip] flashed on $Port. Connect to WiFi 'DroneBridge ESP32' (pwd: dronebridge) -> http://192.168.2.1/" -ForegroundColor Green
}
