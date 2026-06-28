#!/usr/bin/env python3
"""
flash-gui.py - Visual build/flash tool for DroneBridge JONOKR (ESP32-C3/C6/S3).

A thin Tkinter front-end over the audited build-fw.ps1 / flash-fw.ps1 scripts:
  * Build  -> runs build-fw.ps1  (Docker-isolated ESP-IDF build)
  * Flash  -> runs flash-fw.ps1  (host esptool over USB)

Pure Python standard library (tkinter) + pyserial (already installed with esptool)
for COM-port listing. No new binaries are introduced - all real work happens in the
reviewed PowerShell scripts.

Launch:  python flash-gui.py     (or double-click Flasher.cmd)
"""
import os
import queue
import re
import subprocess
import threading
import tkinter as tk
from tkinter import ttk
from tkinter.scrolledtext import ScrolledText

REPO = os.path.dirname(os.path.abspath(__file__))
BUILD_PS1 = os.path.join(REPO, "build-fw.ps1")
FLASH_PS1 = os.path.join(REPO, "flash-fw.ps1")
CHIPS = ("esp32c3", "esp32c6", "esp32s3")
SERIALS = ("uart", "usb")  # data interface: uart = GPIO UART (AIR -> flight controller); usb = USB/JTAG (GND over USB-C)
PCT_RE = re.compile(r"\((\d+)\s*%\)")
NO_WINDOW = getattr(subprocess, "CREATE_NO_WINDOW", 0)  # hide the child PowerShell console


def list_serial_ports():
    """Return display labels like 'COM4  (USB JTAG/serial debug unit)'. The COM name is always the
    first whitespace-delimited token. pyserial first (adds descriptions), winreg fallback (names)."""
    try:
        from serial.tools.list_ports import comports
        labels = []
        for p in sorted(comports(), key=lambda x: x.device):
            desc = (p.description or "").strip()
            labels.append(f"{p.device}  ({desc})" if desc and desc.lower() != "n/a" else p.device)
        return labels
    except Exception:
        pass
    try:
        import winreg
        names = []
        with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE,
                            r"HARDWARE\DEVICEMAP\SERIALCOMM") as k:
            i = 0
            while True:
                try:
                    names.append(winreg.EnumValue(k, i)[1])
                    i += 1
                except OSError:
                    break
        return sorted(names)
    except Exception:
        return []


class FlasherApp:
    def __init__(self, root):
        self.root = root
        self.proc = None
        self.q = queue.Queue()
        self.running = False

        root.title("DroneBridge JONOKR Flasher")
        root.minsize(660, 480)

        top = ttk.Frame(root, padding=8)
        top.pack(fill="x")
        ttk.Label(top, text="Chip:").grid(row=0, column=0, sticky="w")
        self.chip = ttk.Combobox(top, values=CHIPS, state="readonly", width=10)
        self.chip.set(CHIPS[0])
        self.chip.grid(row=0, column=1, padx=(4, 16))
        ttk.Label(top, text="Port:").grid(row=0, column=2, sticky="w")
        self.port = ttk.Combobox(top, state="readonly", width=14)
        self.port.grid(row=0, column=3, padx=4)
        self.refresh_btn = ttk.Button(top, text="Refresh", width=8, command=self.refresh_ports)
        self.refresh_btn.grid(row=0, column=4, padx=4)
        self.erase = tk.BooleanVar(value=False)
        self.erase_chk = ttk.Checkbutton(top, text="Erase first (factory wipe)", variable=self.erase)
        self.erase_chk.grid(row=0, column=5, padx=(16, 0))

        ttk.Label(top, text="Serial:").grid(row=1, column=0, sticky="w", pady=(6, 0))
        self.serial = ttk.Combobox(top, values=SERIALS, state="readonly", width=10)
        self.serial.set(SERIALS[0])
        self.serial.grid(row=1, column=1, padx=(4, 16), pady=(6, 0), sticky="w")
        ttk.Label(top, text="usb = GND plugged into the GCS over USB-C   |   uart = AIR wired to a flight controller"
                  ).grid(row=1, column=2, columnspan=4, sticky="w", pady=(6, 0))

        btns = ttk.Frame(root, padding=(8, 0))
        btns.pack(fill="x")
        self.build_btn = ttk.Button(btns, text="Build", command=self.do_build)
        self.build_btn.pack(side="left", padx=4, ipadx=12, ipady=3)
        self.flash_btn = ttk.Button(btns, text="Flash", command=self.do_flash)
        self.flash_btn.pack(side="left", padx=4, ipadx=12, ipady=3)
        self.stop_btn = ttk.Button(btns, text="Stop", command=self.stop, state="disabled")
        self.stop_btn.pack(side="left", padx=4, ipadx=6, ipady=3)

        prog = ttk.Frame(root, padding=8)
        prog.pack(fill="x")
        self.bar = ttk.Progressbar(prog, mode="indeterminate")
        self.bar.pack(side="left", fill="x", expand=True)
        self.status = ttk.Label(prog, text="Ready", width=26, anchor="w")
        self.status.pack(side="left", padx=(8, 0))

        self.log = ScrolledText(root, height=18, bg="#111111", fg="#dddddd",
                                insertbackground="#dddddd", font=("Consolas", 9))
        self.log.pack(fill="both", expand=True, padx=8, pady=(0, 8))
        self.log.configure(state="disabled")

        self.append("DroneBridge JONOKR visual flasher\n"
                    f"Repo: {REPO}\n"
                    "Build runs in Docker; Flash uses host esptool. Both wrap the .ps1 scripts.\n")
        self._check_scripts()
        self.refresh_ports()
        self.root.after(80, self._drain)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---------- helpers ----------
    def _check_scripts(self):
        missing = [os.path.basename(p) for p in (BUILD_PS1, FLASH_PS1) if not os.path.isfile(p)]
        if missing:
            self.append("ERROR: missing " + ", ".join(missing) +
                        f" - run this from the repo folder.\n")
            self.build_btn.config(state="disabled")
            self.flash_btn.config(state="disabled")

    def refresh_ports(self):
        ports = list_serial_ports()
        self.port["values"] = ports
        if ports and self.port.get() not in ports:
            self.port.set(ports[0])
        if not ports:
            self.port.set("")
        self.append("[ports] " + (", ".join(ports) if ports else "none found - plug in via USB") + "\n")

    def append(self, text):
        self.log.configure(state="normal")
        self.log.insert("end", text)
        self.log.see("end")
        self.log.configure(state="disabled")

    def set_busy(self, busy, status):
        self.running = busy
        for w in (self.build_btn, self.flash_btn, self.refresh_btn, self.erase_chk):
            w.config(state="disabled" if busy else "normal")
        for combo in (self.chip, self.serial, self.port):
            combo.config(state="disabled" if busy else "readonly")
        self.stop_btn.config(state="normal" if busy else "disabled")
        self.status.config(text=status)
        if busy:
            self.bar.start(12)
        else:
            self.bar.stop()

    # ---------- run a powershell script ----------
    def run_script(self, args, label):
        if self.running:
            return
        self.set_busy(True, label + " ...")
        self.append(f"\n===== {label} =====\n> {os.path.basename(args[0])} {' '.join(args[1:])}\n")
        cmd = ["powershell.exe", "-NoProfile", "-ExecutionPolicy", "Bypass", "-File"] + args
        threading.Thread(target=self._worker, args=(cmd, label), daemon=True).start()

    def _worker(self, cmd, label):
        try:
            self.proc = subprocess.Popen(
                cmd, cwd=REPO, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, encoding="utf-8", errors="replace", bufsize=1,
                creationflags=NO_WINDOW)
        except Exception as e:
            self.q.put(("line", f"failed to start: {e}\n"))
            self.q.put(("done", (label, 1)))
            return
        for line in self.proc.stdout:
            self.q.put(("line", line))
            m = PCT_RE.search(line)
            if m:
                self.q.put(("status", f"{label} ... {m.group(1)}%"))
        code = self.proc.wait()
        self.proc = None
        self.q.put(("done", (label, code)))

    def _drain(self):
        try:
            while True:
                kind, payload = self.q.get_nowait()
                if kind == "line":
                    self.append(payload)
                elif kind == "status":
                    self.status.config(text=payload)
                elif kind == "done":
                    label, code = payload
                    ok = (code == 0)
                    result = "done" if ok else f"failed (exit {code})"
                    self.set_busy(False, ("OK - " if ok else "FAIL - ") + f"{label} {result}")
                    self.append(f"----- {label}: {result.upper()} -----\n")
        except queue.Empty:
            pass
        self.root.after(80, self._drain)

    # ---------- button actions ----------
    def do_build(self):
        chip = self.chip.get()
        serial = self.serial.get()
        self.run_script([BUILD_PS1, "-Chips", chip, "-Serial", serial], f"Build {chip} ({serial})")

    def do_flash(self):
        chip = self.chip.get()
        serial = self.serial.get()
        args = [FLASH_PS1, "-Chip", chip, "-Serial", serial]
        sel = self.port.get().strip()
        port = sel.split()[0] if sel else ""   # take the COMx token, drop the description
        if port:
            args += ["-Port", port]
        if self.erase.get():
            args += ["-Erase"]
        self.append("(tip: if flashing stalls at 'Connecting....', hold BOOT/IO0, tap RST, then Flash again)\n")
        self.run_script(args, f"Flash {chip} ({serial})")

    def stop(self):
        if self.proc:
            self.append("\n[stopping - best effort; a running docker build may keep going]\n")
            try:
                self.proc.terminate()
            except Exception:
                pass

    def _on_close(self):
        if self.proc:
            try:
                self.proc.terminate()
            except Exception:
                pass
        self.root.destroy()


def main():
    root = tk.Tk()
    try:
        ttk.Style().theme_use("vista")
    except Exception:
        pass
    FlasherApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
