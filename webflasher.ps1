# webflasher.ps1 - Generate & serve a browser-based flasher (ESP Web Tools) from the merged
# binaries produced by build-fw.ps1. Flash from Chrome/Edge over USB with ZERO host install
# (uses the browser's Web Serial API). Build first:  .\build-fw.ps1
#
#   .\webflasher.ps1                  # (re)generate flasher\ and serve http://localhost:8000
#   .\webflasher.ps1 -ServerPort 9000
#   .\webflasher.ps1 -NoServe         # just (re)generate flasher\, don't start a server
#requires -Version 5
[CmdletBinding()]
param([int]$ServerPort = 8000, [switch]$NoServe)

function Fail($msg) { Write-Host "ERROR: $msg" -ForegroundColor Red; exit 1 }

$repo    = $PSScriptRoot
$flasher = Join-Path $repo 'flasher'
$builds  = Join-Path $flasher 'builds'

# UTF-8 WITHOUT BOM - a BOM can break JSON.parse / fetch().json() in the browser.
$utf8NoBom = New-Object System.Text.UTF8Encoding($false)
# chipFamily strings must match ESP Web Tools' accepted values exactly.
$families = [ordered]@{ esp32c3 = 'ESP32-C3'; esp32c6 = 'ESP32-C6'; esp32s3 = 'ESP32-S3' }

New-Item -ItemType Directory -Force -Path $builds -ErrorAction Stop | Out-Null

$options = New-Object System.Collections.Generic.List[string]
foreach ($chip in $families.Keys) {
    $merged = Join-Path $repo "build\$chip\merged.bin"
    if (-not (Test-Path $merged)) { continue }

    $dest = Join-Path $builds $chip
    New-Item -ItemType Directory -Force -Path $dest -ErrorAction Stop | Out-Null
    Copy-Item $merged (Join-Path $dest 'merged.bin') -Force -ErrorAction Stop

    $manifest = @"
{
  "name": "DroneBridge JONOKR",
  "version": "2.2.1",
  "new_install_prompt_erase": true,
  "builds": [
    { "chipFamily": "$($families[$chip])", "parts": [ { "path": "merged.bin", "offset": 0 } ] }
  ]
}
"@
    [System.IO.File]::WriteAllText((Join-Path $dest 'manifest.json'), $manifest, $utf8NoBom)
    $options.Add("        <option value=`"builds/$chip/manifest.json`">$($families[$chip])</option>")
    Write-Host "  + $chip  ($($families[$chip]))" -ForegroundColor Green
}

if ($options.Count -eq 0) {
    Fail "No merged.bin found under build\<chip>\. Run .\build-fw.ps1 first (it produces merged.bin)."
}

$indexHtml = @"
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>DroneBridge JONOKR - Web Flasher</title>
  <script type="module" src="https://unpkg.com/esp-web-tools@10/dist/web/install-button.js?module"></script>
  <style>
    body { font-family: system-ui, sans-serif; max-width: 40rem; margin: 3rem auto; padding: 0 1rem; line-height: 1.5; }
    select, button { font-size: 1rem; padding: .5rem; }
    .row { margin: 1.25rem 0; }
  </style>
</head>
<body>
  <h1>DroneBridge JONOKR - Web Flasher</h1>
  <p>Plug the ESP32 in via USB, pick your board, click Flash. Desktop <b>Chrome</b> or <b>Edge</b> only.</p>
  <div class="row">
    <label for="variant">Board:</label>
    <select id="variant">
$($options -join "`n")
    </select>
  </div>
  <div class="row">
    <esp-web-install-button id="installer">
      <button slot="activate">Connect &amp; Flash</button>
      <span slot="unsupported">This browser can't flash. Use desktop Chrome or Edge.</span>
      <span slot="not-allowed">Must be served over HTTPS or http://localhost.</span>
    </esp-web-install-button>
  </div>
  <script>
    const sel = document.getElementById("variant");
    const btn = document.getElementById("installer");
    const apply = () => btn.setAttribute("manifest", sel.value);
    sel.addEventListener("change", apply);
    apply();
  </script>
</body>
</html>
"@
[System.IO.File]::WriteAllText((Join-Path $flasher 'index.html'), $indexHtml, $utf8NoBom)
Write-Host "`nGenerated $flasher" -ForegroundColor Green

if ($NoServe) { return }

if (-not (Get-Command python -ErrorAction SilentlyContinue)) {
    Write-Warning "Python not found - can't auto-serve. Host the 'flasher' folder on any HTTPS server and open it in Chrome."
    return
}
Write-Host "Serving http://localhost:$ServerPort   (Ctrl+C to stop). Open it in Chrome/Edge." -ForegroundColor Cyan
python -m http.server $ServerPort --directory $flasher
