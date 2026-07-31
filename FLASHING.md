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
| `requirements.txt` | Host-side Python deps (`esptool`, `pyserial`) for `flash-fw.ps1` / `flash-gui.py`. Not needed for `build-fw.ps1` — that runs entirely in Docker. |
| `serial-probe.py` | **Diagnostic**: read-only listen on a COM port and decode whatever MAVLink comes out (which sysid/compid nodes, which messages). Use it when GroundControl shows no drones — it tells you whether the ESP32 is transmitting at all. `python serial-probe.py -p COM7` |

## Prerequisites
- **Docker Desktop**, running (`winget install Docker.DockerDesktop`; needs WSL2 via `wsl --install`).
- To flash with `flash-fw.ps1` **or** `flash-gui.py`: **Python 3**, then from this folder:
  ```powershell
  pip install -r requirements.txt
  ```
  `flash-gui.py` checks for this at launch and announces clearly (in its log
  pane) if it's missing, instead of failing silently mid-flash.
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
| **beacon** | GPS beacon / armband — wire the u-blox GPS (MicoAir M10 Ultra, factory 230400 baud) to ESP **RX=GPIO20 / TX=GPIO21**; it streams its position to the GCS automatically. |

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
- **Role unit boots as a WiFi AP instead of ESP-NOW** (you see a "DroneBridge for ESP32" WiFi
  network): the flash you wrote predates the role-baked defaults, or the flasher GUI/`-Role`
  skipped rebuilding because a `build\<chip>-<role>\` folder from an earlier attempt already
  existed (`build-fw.ps1 -Role` now always does a fresh `fullclean` reconfigure to prevent this,
  but an *already-built* image sitting in that folder needs a rebuild to pick up the fix). Fix:
  in the GUI click **Rebuild firmware** (or delete `build\<chip>-<role>\` and re-run
  `build-fw.ps1 -Role ...`), then **Flash & configure** again. To confirm:
  - **air / beacon** (data on the GPIO UART, console stays on the board's own USB port): open any
    serial terminal on the COM port at 115200 baud right after boot — main.c logs a line like
    `Boot config: radio_mode=4 serial_proto=4 baud=230400 chan=6 ext_ant=1` (`radio_mode` 4 = AIR
    ESP-NOW; `serial_proto` 4 = MAVLink, 6 = the beacon's UBX-GPS protocol; 1 = stock AP mode —
    if you see that, the rebuild didn't take).
  - **gnd** (data rides the same USB port, so there's no free console to watch): simplest check
    is the one that surfaced this bug — the "DroneBridge for ESP32" WiFi network should be
    **gone**, and GroundControl's SITL/TCP-style COM-port connect should see a drone.
- **Nothing at all appears in GroundControl** (not even the ESP32's own node): isolate the
  firmware from the app with the read-only probe — close GroundControl first, then:
  ```powershell
  python serial-probe.py -p COM7 -s 10
  ```
  * *"the port is silent"* → the ESP32 isn't transmitting: wrong COM port, charge-only USB
    cable, or the unit isn't running the role image.
  * *nodes listed* (e.g. `sysid=1 compid=68 … ONBOARD_CONTROLLER`) → the firmware and link are
    fine and the problem is on the GroundControl side.
- **Console pins collide with the data UART (ESP32-C3)**: the IDF default console is UART0 =
  **GPIO20/21 on the C3** — the very pins the beacon uses for its u-blox GPS. `air` and `beacon`
  role builds therefore use the `noUARTConsole` overlay, which moves the console to USB. Side
  benefit: their boot logs are readable on the USB port. If you build those roles by hand, pass
  `-D SDKCONFIG_DEFAULTS=config_defaults/sdkconfig.defaults.noUARTConsole` or you will get a
  silent, garbled GPS link.
- **Official board pins/antenna**: the build defaults to the *generic* board. To configure the
  official C3/C6 board, run menuconfig in the container:
  ```powershell
  docker run --rm -it -v "${PWD}:/project" -w /project jonokr-idf `
    idf.py -B build/esp32c3 -D SDKCONFIG=build/esp32c3/sdkconfig menuconfig
  ```
- `build\` and `flasher\` are generated output — safe to delete; they are git-ignored.
- esptool v5 renamed some CLI commands; pin **`"esptool<5"`** so the host flasher matches the
  flash arguments produced by the IDF 5.4 build.
