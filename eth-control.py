#!/usr/bin/env python3
"""Listen to T-962 status and write the CUSTOM #1 reflow profile.

The oven is 192.168.0.62 and speaks UDP port 1962. Put the host on
192.168.0.0/24. Status is broadcast once a second. A profile is 48
temperatures in degrees C, each 0 or 30..300, one point every 10 seconds.

Wiring on the controller (3.3 V logic):
  P0.17 SCK, P0.6 MOSI, P0.5 MISO, P0.10 CS, P0.30 RESET

Examples:
  python3 eth-control.py listen
  python3 eth-control.py write --file profile.txt
  python3 eth-control.py write 50 50 60 70 ...
"""

import argparse
import re
import socket
import struct
import sys

OVEN_IP = "192.168.0.62"
UDP_PORT = 1962
MAGIC = b"T962"
TYPE_STATUS = 1
TYPE_WRITE = 2
TYPE_ACK = 3
PROFILE_POINTS = 48

MODES = {
    0: "INITIAL",
    1: "STANDBY",
    2: "BAKE",
    3: "REFLOW",
    4: "STANDBYFAN",
}

STATUS_FMT = "<4sBBBBBBhH20s"
ACK_FMT = "<4sBB"


def parse_status(payload):
    if len(payload) < struct.calcsize(STATUS_FMT):
        return None
    magic, kind, mode, profile, fan, heat, _reserved, temp_x10, setpoint, name = struct.unpack(
        STATUS_FMT, payload[: struct.calcsize(STATUS_FMT)]
    )
    if magic != MAGIC or kind != TYPE_STATUS:
        return None
    text = name.split(b"\x00", 1)[0].decode("ascii", "replace")
    return {
        "mode": MODES.get(mode, str(mode)),
        "profile_idx": profile,
        "profile": text,
        "fan": fan,
        "heat": heat,
        "temp_c": temp_x10 / 10.0,
        "setpoint": setpoint,
    }


def format_status(status):
    return (
        "%6.1fC  set %3u  fan %3u  heat %3u  profile %u %s  %s"
        % (
            status["temp_c"],
            status["setpoint"],
            status["fan"],
            status["heat"],
            status["profile_idx"],
            status["profile"],
            status["mode"],
        )
    )


def listen(port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.bind(("", port))
    print("Listening for T-962 status on UDP %u" % port)
    while True:
        data, addr = sock.recvfrom(512)
        status = parse_status(data)
        if status is None:
            continue
        print("%s  %s" % (addr[0], format_status(status)))


def load_temps(text):
    values = [int(item) for item in re.findall(r"-?\d+", text)]
    if len(values) != PROFILE_POINTS:
        raise SystemExit("Need %u temperatures, got %u" % (PROFILE_POINTS, len(values)))
    for value in values:
        if value != 0 and (value < 30 or value > 300):
            raise SystemExit("Temperature %u is outside 0 or 30..300" % value)
    return values


def write_profile(host, port, temps):
    payload = struct.pack("<4sBB%dH" % PROFILE_POINTS, MAGIC, TYPE_WRITE, PROFILE_POINTS, *temps)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(2.0)
    sock.sendto(payload, (host, port))
    try:
        data, _addr = sock.recvfrom(64)
    except socket.timeout:
        raise SystemExit("No reply from %s:%u" % (host, port))
    if len(data) < struct.calcsize(ACK_FMT):
        raise SystemExit("Short reply from the oven")
    magic, kind, status = struct.unpack(ACK_FMT, data[: struct.calcsize(ACK_FMT)])
    if magic != MAGIC or kind != TYPE_ACK:
        raise SystemExit("Unexpected reply from the oven")
    if status == 0:
        print("CUSTOM #1 stored on %s" % host)
        return
    if status == 1:
        raise SystemExit("Oven rejected the profile")
    raise SystemExit("Oven failed while writing EEPROM (status %u)" % status)


def main():
    parser = argparse.ArgumentParser(description="T-962 Ethernet status and CUSTOM #1 writer")
    parser.add_argument("--host", default=OVEN_IP, help="Oven IPv4 address")
    parser.add_argument("--port", type=int, default=UDP_PORT, help="UDP port")
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("listen", help="Print status broadcasts until interrupted")

    write = sub.add_parser("write", help="Store 48 temperatures in CUSTOM #1")
    write.add_argument("--file", help="Text file of 48 temperatures")
    write.add_argument("temps", nargs="*", help="48 temperatures, separated by spaces or commas")

    args = parser.parse_args()
    if args.command == "listen":
        listen(args.port)
        return
    if args.file:
        with open(args.file, "r", encoding="utf-8") as handle:
            text = handle.read()
    else:
        text = " ".join(args.temps)
    write_profile(args.host, args.port, load_temps(text))


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
