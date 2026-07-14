# build-fw.ps1 - Build DroneBridge JONOKR firmware for ESP32-C3 / C6 / S3 inside an isolated
# ESP-IDF + Node.js Docker container. No Espressif toolchain is installed on the host - the
# toolchain lives only inside the disposable container. Each chip builds into build\<chip>\ and
# (best-effort) gets a merged.bin used by the browser flasher (webflasher.ps1).
#
#   .\build-fw.ps1                  # build all three: esp32c3, esp32c6, esp32s3
#   .\build-fw.ps1 -Chips esp32c3   # build one
#   .\build-fw.ps1 -Chips esp32c6 -Serial usb   # GND image: MAVLink data over the USB-C port
#   .\build-fw.ps1 -Chips esp32c3 -Role air     # role-baked JONOKR image (see below)
#   .\build-fw.ps1 -Clean           # fullclean rebuild of the selected chips
#
# -Role air|gnd|beacon : JONOKR provisioning image — bakes the role's boot defaults into the
#                        firmware (ESP-NOW AIR / ESP-NOW GND / GPS-beacon; see main/CMakeLists.txt),
#                        so a freshly-erased unit needs zero web-UI configuration. gnd implies the
#                        USB/JTAG data port (plugs into the GCS over USB-C); air/beacon use the GPIO
#                        UART (FC / GPS). Builds into build\<chip>-<role>\.
#
# -Serial uart (default): data on the GPIO UART - use for AIR units wired to a flight controller.
# -Serial usb           : data on the native USB/JTAG port, so a GND unit plugs straight into the
#                         GCS computer with no UART-to-USB adapter. Builds into build\<chip>-usb\ so
#                         it never clobbers the UART image. NOTE: no console logs over USB in this
#                         mode (that port now carries MAVLink data instead).
#                         (Ignored when -Role is given — the role decides the data port.)
#
# Requires: Docker Desktop running. First run builds the image 'jonokr-idf' (a few minutes).
#requires -Version 5
[CmdletBinding()]
param(
    [ValidateSet('esp32c3', 'esp32c6', 'esp32s3')][string[]]$Chips = @('esp32c3', 'esp32c6', 'esp32s3'),
    [ValidateSet('uart', 'usb')][string]$Serial = 'uart',
    [ValidateSet('air', 'gnd', 'beacon')][string]$Role,
    [switch]$Clean
)

# The role decides the data port: GND talks to the GCS over USB-C, AIR/beacon use the UART.
if ($Role) { $Serial = if ($Role -eq 'gnd') { 'usb' } else { 'uart' } }

function Fail($msg) { Write-Host "ERROR: $msg" -ForegroundColor Red; exit 1 }

$repo  = $PSScriptRoot
$image = 'jonokr-idf'                       # = espressif/idf:v5.4.4 + Node.js
$dfDir = Join-Path $repo 'flash-tool'

# --- preflight: docker present + engine running ---
if (-not (Get-Command docker -ErrorAction SilentlyContinue)) {
    Fail "Docker not found on PATH. Install Docker Desktop (see FLASHING.md)."
}
docker info 2>&1 | Out-Null      # exits non-zero if the engine isn't running
if ($LASTEXITCODE -ne 0) {
    Fail "Docker engine isn't running. Start Docker Desktop, wait for 'Engine running', then retry."
}

# --- build the IDF+Node image once (cached afterwards) ---
if (-not (docker images -q $image)) {
    Write-Host "==> building image '$image' (ESP-IDF 5.4 + Node.js); one-time, a few minutes ..." -ForegroundColor Cyan
    docker build -t $image $dfDir
    if ($LASTEXITCODE -ne 0) { Fail "docker build of '$image' failed." }
}

foreach ($chip in $Chips) {
    # Role images build into build\<chip>-<role>\, USB/JTAG-serial images into build\<chip>-usb\ —
    # each variant keeps its own build dir so images never overwrite each other.
    $variant = if ($Role) { "$chip-$Role" } elseif ($Serial -eq 'usb') { "$chip-usb" } else { $chip }
    $roleLabel = if ($Role) { ", role $Role" } else { '' }
    Write-Host "`n==> [$variant] building ($Serial serial$roleLabel) ..." -ForegroundColor Cyan
    $bdir = "build/$variant"                                 # forward slashes for the Linux container
    $cfg  = "-B $bdir -D SDKCONFIG=$bdir/sdkconfig"
    if ($Serial -eq 'usb') {
        # Layer on the ready-made USB/JTAG overlay (flips data serial to USB/JTAG and frees the
        # secondary USB console, as that Kconfig option requires). It is self-contained, so it
        # fully replaces sdkconfig.defaults - a single file, no semicolon list / shell-quoting needed.
        $cfg += " -D SDKCONFIG_DEFAULTS=config_defaults/sdkconfig.defaults.USBSerial"
    }
    if ($Role) {
        # Bake the role's boot defaults into the image (main/CMakeLists.txt maps this to
        # DB_BUILD_DEFAULT_* compile definitions). Uppercase to match the CMake comparisons.
        $cfg += " -D DB_ROLE=$($Role.ToUpper())"
    }
    $configured = Test-Path (Join-Path $repo "build\$variant\sdkconfig")

    if     ($Clean -and $configured) { $action = "fullclean set-target $chip build" }
    elseif ($configured)             { $action = "build" }
    else                             { $action = "set-target $chip build" }

    docker run --rm -v "${repo}:/project" -w /project $image bash -c "idf.py $cfg $action"
    if ($LASTEXITCODE -ne 0) { Fail "[$chip] firmware build failed (exit $LASTEXITCODE)." }

    # Merged single-image for the browser flasher (best-effort; CLI flashing does not need it).
    # merge-bin runs INSIDE the build dir (-B), so --output is relative to it -> build/<chip>/merged.bin.
    docker run --rm -v "${repo}:/project" -w /project $image bash -c "idf.py $cfg merge-bin --output merged.bin"
    if ($LASTEXITCODE -ne 0) { Write-Warning "[$variant] merge-bin failed; browser flasher won't have this chip (CLI flash still works)." }

    Write-Host "==> [$variant] done -> build\$variant" -ForegroundColor Green
}

$flashArgsHint = if ($Role) { " -Role $Role" } elseif ($Serial -eq 'usb') { ' -Serial usb' } else { '' }
Write-Host "`nFlash it:" -ForegroundColor Yellow
Write-Host "  USB cable :  .\flash-fw.ps1 -Chip $($Chips[0])$flashArgsHint -Port COM16"
Write-Host "  GUI       :  python flash-gui.py   (or double-click Flasher.cmd)"
Write-Host "  Browser   :  .\webflasher.ps1   (then open http://localhost:8000 in Chrome/Edge)"

# The firmware build succeeded by this point; merge-bin is best-effort. Exit 0 explicitly so a
# non-fatal merge-bin exit code can't leak out as a script-level failure.
exit 0
