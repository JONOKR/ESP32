# Flashing DroneBridge JONOKR

A small, security-conscious toolkit to build your modified firmware and flash it onto
ESP32-C3 / C6 / S3 boards. The Espressif toolchain runs **only inside a disposable Docker
container** — nothing Espressif-native is installed on your Windows host.

## What's here
| File | Purpose |
|------|---------|
| `flash-tool/Dockerfile` | Build image = `espressif/idf:v5.4.4` + Node.js. (Node is required because the build inlines the web UI into `www.bin`.) |
| `build-fw.ps1` | Builds firmware for each chip inside the container → `build\<chip>\`. |
| `flash-fw.ps1` | Flashes a build onto the device from the host with `esptool` (USB cable). |
| `webflasher.ps1` | Optional: generates & serves a browser flasher — flash from Chrome/Edge, zero host install. |
| `flash-gui.py` | **Visual desktop app** (Tkinter): chip + port dropdowns, Build/Flash buttons, live log. Wraps the scripts above — no new dependencies. Launch: `python flash-gui.py` or double-click `Flasher.cmd`. |

## Prerequisites
- **Docker Desktop**, running (`winget install Docker.DockerDesktop`; needs WSL2 via `wsl --install`).
- To flash with `flash-fw.ps1`: **Python 3** + `pip install "esptool<5"`.
  - …or skip Python and use the **browser flasher** (`webflasher.ps1`) — nothing to install.
- You do **not** need ESP-IDF or Node.js on the host — both live inside the container.

## Easiest: the visual app
```powershell
python flash-gui.py            # or just double-click Flasher.cmd
```
Opens a window with **chip** + **COM-port** dropdowns, **Build** and **Flash** buttons, an
optional *Erase* checkbox, and a live log. It simply runs the scripts below underneath, so the
same Docker build + host-esptool flash happen — just with buttons instead of typing.

## Build
```powershell
.\build-fw.ps1                 # all three: esp32c3, esp32c6, esp32s3
.\build-fw.ps1 -Chips esp32c3  # just one
.\build-fw.ps1 -Clean          # fullclean rebuild
```
The first run builds the `jonokr-idf` image (a few minutes), then compiles. Re-runs are
incremental: edit code → re-run `build-fw.ps1` → re-flash.

## Flash — option A: USB cable + esptool (host)
```powershell
.\flash-fw.ps1 -Chip esp32c3                    # auto-detects the COM port if exactly one
.\flash-fw.ps1 -Chip esp32c3 -Port COM4         # or specify it
.\flash-fw.ps1 -Chip esp32c6 -Port COM7 -Erase  # also factory-wipe NVS (WiFi/config)
```
Find the COM port in Device Manager → *Ports (COM & LPT)*.

## Flash — option B: browser (zero host install)
```powershell
.\webflasher.ps1               # serves http://localhost:8000
```
Open `http://localhost:8000` in **Chrome or Edge**, pick the board, click **Connect & Flash**.
(Web Serial works only in Chromium browsers, over `localhost`/HTTPS.)

## Security model
- The cross-compiler, ESP-IDF SDK and Node toolchain are confined to the **container** built from a
  pinned base image; nothing Espressif-native touches your host. Pin the base by digest in the
  Dockerfile for extra assurance.
- Host-side you run only **`esptool`** (auditable, pure-Python) or the **browser** (`esptool-js`,
  open-source JS) — a tiny trust surface.
- The on-device WiFi/BT radio blobs are closed (true of any WiFi SoC) but run on the air-gapped
  device — its own WiFi AP — which you can traffic-monitor. The firmware source (DroneBridge) is open.

## Notes & troubleshooting
- **Download mode**: official DB boards and native-USB chips auto-reset. If flashing hangs at
  `Connecting....`, hold **BOOT/IO0**, tap **RST** (or replug), keep holding BOOT, then re-run.
- **Official board pins/antenna**: the build defaults to the *generic* board. To configure the
  official C3/C6 board, run menuconfig in the container:
  ```powershell
  docker run --rm -it -v "${PWD}:/project" -w /project jonokr-idf `
    idf.py -B build/esp32c3 -D SDKCONFIG=build/esp32c3/sdkconfig menuconfig
  ```
- `build\` and `flasher\` are generated output — safe to delete; they are git-ignored.
- esptool v5 renamed some CLI commands; pin **`"esptool<5"`** so the host flasher matches the
  flash arguments produced by the IDF 5.4 build.
