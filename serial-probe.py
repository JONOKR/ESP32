#!/usr/bin/env python3
"""
serial-probe.py - Is the ESP32 actually saying anything on this COM port?

Answers, without involving GroundControl at all:
  * are ANY bytes arriving on the port?
  * are they valid-looking MAVLink frames?
  * which nodes (sysid/compid) are transmitting, and what are they?

Use it on the GND unit's USB port when GroundControl shows no drones. It is
read-only - it never writes to the port, so it can't disturb the device.

  python serial-probe.py                 # auto-pick the port if there's only one
  python serial-probe.py -p COM7         # or name it
  python serial-probe.py -p COM7 -s 20   # listen for 20 s (default 10)
  python serial-probe.py -p COM7 --hex   # also hex-dump the first raw bytes

Requires:  pip install -r requirements.txt   (pyserial)
"""
import argparse
import collections
import sys
import time

# MAVLink message ids we care about naming. Anything else prints as its number.
MSG_NAMES = {
    0: "HEARTBEAT", 1: "SYS_STATUS", 24: "GPS_RAW_INT", 30: "ATTITUDE",
    33: "GLOBAL_POSITION_INT", 32: "LOCAL_POSITION_NED", 74: "VFR_HUD",
    77: "COMMAND_ACK", 109: "RADIO_STATUS", 147: "BATTERY_STATUS",
    253: "STATUSTEXT", 385: "TUNNEL",
}
# HEARTBEAT.type -> what kind of node this is (the subset this project emits).
MAV_TYPE = {
    0: "GENERIC (beacon, if autopilot=INVALID)", 1: "FIXED_WING", 2: "QUADROTOR",
    6: "GCS", 18: "ONBOARD_CONTROLLER (DroneBridge ESP32 itself)",
}
MAV_AUTOPILOT = {0: "GENERIC", 3: "ARDUPILOTMEGA", 8: "INVALID"}


def pick_port(explicit):
    """Choose the port and ALWAYS report its description.

    The description matters as much as the data: it says whether you're on the
    chip's *native* USB (`USB JTAG/serial debug unit`) or on a USB-UART bridge
    chip (`CP210x`, `CH340`, `USB-Enhanced-SERIAL`, ...). A `gnd` role image
    sends MAVLink to the NATIVE USB peripheral, so if your only port is a
    bridge chip, the data is going somewhere that port can't see.
    """
    from serial.tools.list_ports import comports
    ports = sorted(comports(), key=lambda p: p.device)
    print("Serial ports present:")
    for p in ports:
        print(f"   {p.device:<8} {p.description}")
    if not ports:
        print("   (none)")
    print()

    if explicit:
        hit = next((p for p in ports if p.device.upper() == explicit.upper()), None)
        if hit is None:
            print(f"WARNING: {explicit} is not in the list above - is the unit plugged in?")
        else:
            print(f"Using {hit.device}  ({hit.description})")
        return explicit
    if not ports:
        sys.exit("No serial ports found - plug the unit in over USB.")
    if len(ports) > 1:
        sys.exit("Multiple ports found - pick one with -p COMx.")
    print(f"Using {ports[0].device}  ({ports[0].description})")
    return ports[0].device


def scan_frames(buf):
    """Yield (sysid, compid, msgid, payload) for MAVLink v1/v2 frames in buf.

    Deliberately does NOT check CRCs: we only want to know who is talking and
    roughly what they're sending. Returns leftover bytes for the next round.
    """
    i = 0
    while i < len(buf):
        b = buf[i]
        if b == 0xFD:                      # MAVLink v2
            if len(buf) - i < 10:
                break                      # header incomplete - wait for more
            plen = buf[i + 1]
            incompat = buf[i + 2]
            total = 12 + plen + (13 if incompat & 0x01 else 0)
            if len(buf) - i < total:
                break
            sysid, compid = buf[i + 5], buf[i + 6]
            msgid = buf[i + 7] | (buf[i + 8] << 8) | (buf[i + 9] << 16)
            yield sysid, compid, msgid, buf[i + 10:i + 10 + plen]
            i += total
        elif b == 0xFE:                    # MAVLink v1
            if len(buf) - i < 8:
                break
            plen = buf[i + 1]
            total = 8 + plen
            if len(buf) - i < total:
                break
            sysid, compid, msgid = buf[i + 3], buf[i + 4], buf[i + 5]
            yield sysid, compid, msgid, buf[i + 6:i + 6 + plen]
            i += total
        else:
            i += 1                         # not a frame start - resync
    del buf[:i]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--port")
    ap.add_argument("-b", "--baud", type=int, default=115200,
                    help="ignored for native-USB (JTAG) units; matters only for FTDI/UART links")
    ap.add_argument("-s", "--seconds", type=float, default=10.0)
    ap.add_argument("--hex", action="store_true", help="hex-dump the first 64 raw bytes")
    args = ap.parse_args()

    try:
        import serial
    except ImportError:
        sys.exit('pyserial missing. Run:  pip install -r requirements.txt')

    port = pick_port(args.port)
    try:
        ser = serial.Serial(port, args.baud, timeout=0.2)
    except Exception as exc:
        sys.exit(f"Could not open {port}: {exc}\n"
                 "(Close GroundControl / any serial monitor holding the port first.)")

    print(f"Listening on {port} for {args.seconds:g}s ... (read-only)\n")
    buf = bytearray()
    raw_total = 0
    first_raw = b""
    nodes = collections.defaultdict(collections.Counter)
    heartbeats = {}

    end = time.monotonic() + args.seconds
    with ser:
        while time.monotonic() < end:
            chunk = ser.read(4096)
            if not chunk:
                continue
            raw_total += len(chunk)
            if not first_raw:
                first_raw = bytes(chunk[:64])
            buf.extend(chunk)
            for sysid, compid, msgid, payload in scan_frames(buf):
                nodes[(sysid, compid)][msgid] += 1
                if msgid == 0 and len(payload) >= 8 and (sysid, compid) not in heartbeats:
                    # HEARTBEAT: custom_mode(4) type(1) autopilot(1) base_mode(1) status(1)
                    heartbeats[(sysid, compid)] = (payload[4], payload[5])

    print(f"Raw bytes received: {raw_total}")
    if args.hex and first_raw:
        print("First bytes:", " ".join(f"{b:02x}" for b in first_raw))
    print()

    if raw_total == 0:
        print("VERDICT: the port is silent - nothing is coming out of the ESP32.")
        print("  * Wrong COM port? (unplug/replug and see which one disappears)")
        print("  * GND unit: is it actually running the 'gnd' role image?")
        print("  * Try a different USB cable - charge-only cables are a classic.")
        return 1

    if not nodes:
        print("VERDICT: bytes ARE arriving, but none of them parse as MAVLink.")
        print("  Likely the unit is in transparent/passthrough mode, is sending")
        print("  console log text on this port, or the baud rate is wrong.")
        return 1

    print("MAVLink nodes seen:")
    for (sysid, compid), counter in sorted(nodes.items()):
        hb = heartbeats.get((sysid, compid))
        who = ""
        if hb:
            t, ap_ = hb
            who = (f"  [HEARTBEAT type={MAV_TYPE.get(t, t)}"
                   f" autopilot={MAV_AUTOPILOT.get(ap_, ap_)}]")
        print(f"  sysid={sysid:<4} compid={compid:<4}{who}")
        for msgid, n in counter.most_common(8):
            print(f"       {MSG_NAMES.get(msgid, f'msg {msgid}'):<24} x{n}")
    print("\nVERDICT: the link works. If GroundControl still shows nothing, the")
    print("problem is on the GroundControl side, not the firmware.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
