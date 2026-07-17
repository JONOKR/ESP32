#!/usr/bin/env python3
"""
flash-gui.py - One-click provisioning tool for DroneBridge JONOKR units.

Pick the UNIT TYPE (air / ground / beacon), plug the board in, click
"Flash & configure". The tool then:
  1. auto-detects the ESP32 chip on the selected COM port (esptool),
  2. builds the role-baked firmware image if it isn't built yet
     (build-fw.ps1 -Role ... -> Docker-isolated ESP-IDF build),
  3. erases the flash and writes the image (flash-fw.ps1 -Role ...).

The role image boots straight into its job with ZERO further configuration
(no web UI): the role's boot defaults are baked into the firmware and the
erase guarantees they take effect.

  air    - ESP-NOW AIR unit: UART wired to the flight controller.
  ground - ESP-NOW GND station: plugs into the GCS computer over USB-C.
  beacon - GPS beacon/armband: UART wired to a u-blox GPS (115200 baud);
           streams its position to the GCS automatically.

Pure Python standard library (tkinter) + pyserial (installed with esptool)
for COM-port listing. All real work happens in the reviewed PowerShell
scripts + esptool.

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

# GUI label -> (script role value, one-line description)
ROLES = {
    "air":    ("air",    "wired to the flight controller (UART) — joins the ESP-NOW net"),
    "ground": ("gnd",    "plugs into the GCS computer over USB-C — appears as a COM port"),
    "beacon": ("beacon", "u-blox GPS on RX=GPIO20/TX=GPIO21 (230400 baud) — streams its position to the GCS"),
}
SUPPORTED_CHIPS = ("esp32c3", "esp32c6", "esp32s3")
PCT_RE = re.compile(r"\((\d+)\s*%\)")
CHIP_RE = re.compile(r"(?:Detecting chip type\.+|Chip is)\s+(ESP32[-A-Za-z0-9]*)")
NO_WINDOW = getattr(subprocess, "CREATE_NO_WINDOW", 0)  # hide child consoles


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
        self.stop_requested = False

        root.title("DroneBridge JONOKR Flasher")
        root.minsize(680, 480)

        top = ttk.Frame(root, padding=8)
        top.pack(fill="x")
        ttk.Label(top, text="Unit type:").grid(row=0, column=0, sticky="w")
        self.role = ttk.Combobox(top, values=list(ROLES), state="readonly", width=10)
        self.role.set("air")
        self.role.grid(row=0, column=1, padx=(4, 16))
        self.role.bind("<<ComboboxSelected>>", lambda _e: self._show_role_hint())
        ttk.Label(top, text="Port:").grid(row=0, column=2, sticky="w")
        self.port = ttk.Combobox(top, state="readonly", width=14)
        self.port.grid(row=0, column=3, padx=4)
        self.refresh_btn = ttk.Button(top, text="Refresh", width=8, command=self.refresh_ports)
        self.refresh_btn.grid(row=0, column=4, padx=4)

        self.role_hint = ttk.Label(top, text="", foreground="#666666")
        self.role_hint.grid(row=1, column=0, columnspan=5, sticky="w", pady=(6, 0))

        btns = ttk.Frame(root, padding=(8, 4))
        btns.pack(fill="x")
        self.flash_btn = ttk.Button(btns, text="Flash && configure", command=self.do_flash)
        self.flash_btn.pack(side="left", padx=4, ipadx=14, ipady=4)
        self.build_btn = ttk.Button(btns, text="Rebuild firmware", command=self.do_rebuild)
        self.build_btn.pack(side="left", padx=4, ipadx=8, ipady=4)
        self.stop_btn = ttk.Button(btns, text="Stop", command=self.stop, state="disabled")
        self.stop_btn.pack(side="left", padx=4, ipadx=6, ipady=4)

        prog = ttk.Frame(root, padding=8)
        prog.pack(fill="x")
        self.bar = ttk.Progressbar(prog, mode="indeterminate")
        self.bar.pack(side="left", fill="x", expand=True)
        self.status = ttk.Label(prog, text="Ready", width=30, anchor="w")
        self.status.pack(side="left", padx=(8, 0))

        self.log = ScrolledText(root, height=18, bg="#111111", fg="#dddddd",
                                insertbackground="#dddddd", font=("Consolas", 9))
        self.log.pack(fill="both", expand=True, padx=8, pady=(0, 8))
        self.log.configure(state="disabled")

        self.append("DroneBridge JONOKR provisioning flasher\n"
                    f"Repo: {REPO}\n"
                    "Pick the unit type, plug the board in, click 'Flash & configure'.\n"
                    "Chip type is auto-detected; the flash is always erased so the role\n"
                    "defaults take effect — the unit needs no further configuration.\n")
        self._check_scripts()
        self._show_role_hint()
        self.refresh_ports()
        self.root.after(80, self._drain)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---------- helpers ----------
    def _check_scripts(self):
        missing = [os.path.basename(p) for p in (BUILD_PS1, FLASH_PS1) if not os.path.isfile(p)]
        if missing:
            self.append("ERROR: missing " + ", ".join(missing) + " - run this from the repo folder.\n")
            self.build_btn.config(state="disabled")
            self.flash_btn.config(state="disabled")

    def _show_role_hint(self):
        _script_role, hint = ROLES[self.role.get()]
        self.role_hint.config(text=f"{self.role.get()}: {hint}")

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
        for w in (self.build_btn, self.flash_btn, self.refresh_btn):
            w.config(state="disabled" if busy else "normal")
        for combo in (self.role, self.port):
            combo.config(state="disabled" if busy else "readonly")
        self.stop_btn.config(state="normal" if busy else "disabled")
        self.status.config(text=status)
        if busy:
            self.bar.start(12)
        else:
            self.bar.stop()

    def _selected_port(self):
        sel = self.port.get().strip()
        return sel.split()[0] if sel else ""   # COMx token, drop the description

    # ---------- worker plumbing ----------
    def _start_job(self, label, job):
        if self.running:
            return
        self.stop_requested = False
        self.set_busy(True, label + " ...")
        self.append(f"\n===== {label} =====\n")
        threading.Thread(target=self._job_wrapper, args=(label, job), daemon=True).start()

    def _job_wrapper(self, label, job):
        try:
            ok = job()
        except Exception as e:
            self.q.put(("line", f"unexpected error: {e}\n"))
            ok = False
        self.q.put(("done", (label, 0 if ok else 1)))

    def _run_stream(self, label, cmd):
        """Run a command streaming output to the log. Returns True on exit 0."""
        self.q.put(("line", "> " + " ".join(os.path.basename(str(c)) for c in cmd[:2])
                    + " " + " ".join(str(c) for c in cmd[2:]) + "\n"))
        try:
            self.proc = subprocess.Popen(
                cmd, cwd=REPO, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, encoding="utf-8", errors="replace", bufsize=1,
                creationflags=NO_WINDOW)
        except Exception as e:
            self.q.put(("line", f"failed to start: {e}\n"))
            return False
        for line in self.proc.stdout:
            self.q.put(("line", line))
            m = PCT_RE.search(line)
            if m:
                self.q.put(("status", f"{label} ... {m.group(1)}%"))
        code = self.proc.wait()
        self.proc = None
        return code == 0 and not self.stop_requested

    def _run_ps1(self, label, script, args):
        return self._run_stream(label, ["powershell.exe", "-NoProfile", "-ExecutionPolicy",
                                        "Bypass", "-File", script] + args)

    def _detect_chip(self, port):
        """Ask esptool which chip is on the port. Returns 'esp32c3' etc. or None."""
        self.q.put(("status", "Detecting chip ..."))
        self.q.put(("line", f"[detect] querying {port} ...\n"))
        try:
            out = subprocess.run(
                ["python", "-m", "esptool", "--port", port, "chip_id"],
                cwd=REPO, capture_output=True, text=True, encoding="utf-8",
                errors="replace", timeout=30, creationflags=NO_WINDOW,
            ).stdout
        except FileNotFoundError:
            self.q.put(("line", "python/esptool not found - install Python 3 and:  pip install \"esptool<5\"\n"))
            return None
        except subprocess.TimeoutExpired:
            self.q.put(("line", "chip detection timed out - hold BOOT/IO0, tap RST, try again\n"))
            return None
        m = CHIP_RE.search(out or "")
        if not m:
            tail = "\n".join((out or "").splitlines()[-4:])
            self.q.put(("line", f"could not detect the chip:\n{tail}\n"))
            return None
        chip = m.group(1).lower().replace("-", "")
        if chip not in SUPPORTED_CHIPS:
            self.q.put(("line", f"unsupported chip '{m.group(1)}' (supported: {', '.join(SUPPORTED_CHIPS)})\n"))
            return None
        self.q.put(("line", f"[detect] {m.group(1)}\n"))
        return chip

    def _build_exists(self, chip, script_role):
        return os.path.isfile(os.path.join(REPO, "build", f"{chip}-{script_role}", "flash_args"))

    # ---------- button actions ----------
    def do_flash(self):
        role_label = self.role.get()
        script_role, _hint = ROLES[role_label]
        port = self._selected_port()
        if not port:
            self.append("no COM port selected - plug the unit in and hit Refresh\n")
            return

        def job():
            chip = self._detect_chip(port)
            if chip is None or self.stop_requested:
                return False
            if not self._build_exists(chip, script_role):
                self.q.put(("line", f"[build] no {role_label} image for {chip} yet - building (first time takes a few minutes) ...\n"))
                self.q.put(("status", f"Building {chip} {role_label} ..."))
                if not self._run_ps1(f"Build {chip} ({role_label})", BUILD_PS1,
                                     ["-Chips", chip, "-Role", script_role]):
                    return False
            if self.stop_requested:
                return False
            self.q.put(("status", f"Flashing {chip} {role_label} ..."))
            self.q.put(("line", "(erase + write; if it stalls at 'Connecting....', hold BOOT/IO0, tap RST, retry)\n"))
            return self._run_ps1(f"Flash {chip} ({role_label})", FLASH_PS1,
                                 ["-Chip", chip, "-Role", script_role, "-Port", port])

        self._start_job(f"Flash & configure — {role_label} unit", job)

    def do_rebuild(self):
        role_label = self.role.get()
        script_role, _hint = ROLES[role_label]
        port = self._selected_port()
        if not port:
            self.append("no COM port selected - plug the unit in and hit Refresh\n")
            return

        def job():
            chip = self._detect_chip(port)
            if chip is None or self.stop_requested:
                return False
            return self._run_ps1(f"Build {chip} ({role_label})", BUILD_PS1,
                                 ["-Chips", chip, "-Role", script_role])

        self._start_job(f"Rebuild firmware — {role_label}", job)

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

    def stop(self):
        self.stop_requested = True
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
