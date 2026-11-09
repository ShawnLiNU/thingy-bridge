#!/usr/bin/env python3
"""Read the Thingy Bridge's USB stream in Python.

The bridge sends one JSON object per line (see docs/PROTOCOL.md). This module
finds the bridge's serial port, reads those lines, and hands you each message.

Use it as a module:

    from thingy_bridge import Bridge
    with Bridge() as bridge:
        for msg in bridge.messages():
            if msg["t"] == "m":
                print(msg["lr"], msg["ud"])      # tilt angles in degrees
            elif msg["t"] == "key":
                print("gesture:", msg["k"])

Or run it to watch the stream:

    python3 thingy_bridge.py            # finds the port by itself
    python3 thingy_bridge.py --port /dev/cu.usbserial-0001

Requires pyserial:  pip install pyserial
"""
import argparse
import json
import sys

import serial
from serial.tools import list_ports

# Names that USB serial adapters on ESP32 boards usually get on macOS and Linux.
PORT_HINTS = ("usbserial", "wchusbserial", "SLAB_USBtoUART", "ttyUSB", "ttyACM")


def find_port():
    """Return the first serial port that looks like an ESP32 board, or None."""
    for p in list_ports.comports():
        if any(hint in p.device for hint in PORT_HINTS):
            return p.device
    return None


class Bridge:
    """A connection to the Thingy Bridge over USB."""

    def __init__(self, port=None, baud=115200, timeout=1.0):
        self.port = port or find_port()
        if not self.port:
            raise RuntimeError("No ESP32 serial port found. Plug in the bridge, or pass port=...")
        self.serial = serial.Serial(self.port, baud, timeout=timeout)
        # Keep the board running: some adapters reset it when these lines change.
        self.serial.dtr = False
        self.serial.rts = False

    def messages(self, include_text=False):
        """Yield each JSON message as a dict. Plain text lines are skipped unless include_text."""
        while True:
            raw = self.serial.readline()
            if not raw:
                continue  # timeout, keep waiting
            line = raw.decode("utf-8", "replace").strip()
            if not line:
                continue
            if line.startswith("{"):
                try:
                    yield json.loads(line)
                    continue
                except ValueError:
                    pass
            if include_text:
                yield {"t": "text", "text": line}

    def send(self, command):
        """Send a one-letter command: l r u d o b p (keys), c (learn flat), i (report info)."""
        self.serial.write(command.encode("ascii"))

    def close(self):
        self.serial.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


def main():
    ap = argparse.ArgumentParser(description="Print the Thingy Bridge stream")
    ap.add_argument("--port", help="serial port (found automatically if left out)")
    ap.add_argument("--raw", action="store_true", help="print every line as received")
    args = ap.parse_args()

    try:
        bridge = Bridge(args.port)
    except (RuntimeError, serial.SerialException) as e:
        sys.exit(str(e))
    print(f"Reading {bridge.port}. Press Ctrl+C to stop.")
    bridge.send("i")
    last_state = None
    try:
        for msg in bridge.messages(include_text=True):
            if args.raw:
                print(msg)
            elif msg["t"] == "m":
                print(f"\rtilt left/right {msg['lr']:6.1f}   back/forward {msg['ud']:6.1f}", end="", flush=True)
            elif msg["t"] == "key":
                print(f"\ngesture: {msg['k']}" + ("" if msg.get("sent") else " (no Bluetooth host)"))
            elif msg["t"] == "state":
                state = (msg["thingy"], msg["host"], msg.get("cal"))
                if state == last_state:
                    continue  # the bridge repeats its state every 2 s; print changes only
                last_state = state
                print(f"\nThingy {'connected' if msg['thingy'] else 'not connected'}, "
                      f"keyboard {'paired' if msg['host'] else 'not paired'}"
                      + (", learning flat" if msg.get("cal") else ""))
            elif msg["t"] == "text":
                print(f"\n{msg['text']}")
    except KeyboardInterrupt:
        print()
    finally:
        bridge.close()


if __name__ == "__main__":
    main()
