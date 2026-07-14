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
| `flash-gui.py` | **Visual desktop app** (Tkinter): pick the **unit type** (air / ground / beacon), pick the COM port, click **Flash & configure** — chip is auto-detected, the image is built on first use, the flash is erased, and the unit boots straight into its role with zero web-UI configuration. Launch: `python flash-gui.py` or double-click `Flasher.cmd`. |

## Prerequisites
- **Docker Desktop**, running (`winget install Docker.DockerDesktop`; needs WSL2 via `wsl --install`).
- To flash with `flash-fw.ps1`: **Python 3** + `pip install "esptool<5"`.
  - …or skip Python and use the **browser flasher** (`webflasher.ps1`) — nothing to install.
- You do **not** need ESP-IDF or Node.js on the host — both live inside the container.

## Easiest: the visual app (one-click provisioning)
```powershell
python flash-gui.py            # or just double-click Flasher.cmd
```
Pick the **unit type**, plug the board in, click **Flash & configure**:

| Unit type | What you get |
|---|---|
| **air**    | ESP-NOW AIR unit — wire the UART to the flight controller. |
| **ground** | ESP-NOW GND station — plugs into the GCS computer over USB-C, shows up as a COM port. |
| **beacon** | GPS beacon / armband — wire the UART to a u-blox GPS (MicoAir M10, 115200 baud); it streams its position to the GCS automatically. |

The tool auto-detects the chip (C3/C6/S3), builds the role image on first use
(Docker, a few minutes once), **always erases the flash**, and writes the image.
The role's settings are baked into the firmware, so the unit needs **no web-UI
configuration** — it boots straight into its job. There is nothing else to set.

## Build (CLI)
```powershell
.\build-fw.ps1                          # stock images, all three chips
.\build-fw.ps1 -Chips esp32c3 -Role air # role-baked JONOKR image (air | gnd | beacon)
.\build-fw.ps1 -Clean                   # fullclean rebuild
```
The first run builds the `jonokr-idf` image (a few minutes), then compiles. Re-runs are
incremental: edit code → re-run `build-fw.ps1` → re-flash. Role images build into
`build\<chip>-<role>\` and never clobber the stock images.

## Flash — option A: USB cable + esptool (host)
```powershell
.\flash-fw.ps1 -Chip esp32c3 -Role air -Port COM4   # provision a role unit (always erases)
.\flash-fw.ps1 -Chip esp32c3                        # stock image; auto-detects the port if exactly one
.\flash-fw.ps1 -Chip esp32c6 -Port COM7 -Erase      # stock + factory-wipe NVS (WiFi/config)
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
