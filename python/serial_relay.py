#!/usr/bin/env python3
"""Share the Thingy Bridge's USB port over the network, for a virtual machine.

Run this on the computer the bridge is plugged into (for example a Mac). A program in
a virtual machine can then use the address socket://HOST:5555 instead of a serial port,
and everything passes through in both directions: the stream one way, commands the other.

    python3 serial_relay.py                         # UTM's shared network: the VM reaches 192.168.64.1
    python3 serial_relay.py --host 0.0.0.0          # any network (anyone on it can connect)
    python3 serial_relay.py --port /dev/cu.usbserial-0001
    python3 serial_relay.py --connect 192.168.64.5:5555   # connect out to the VM instead (see below)

--connect is for computers that block incoming connections (for example a managed work laptop):
the VM listens instead, with  socat TCP-LISTEN:5555,reuseaddr PTY,link=/tmp/thingy,raw,echo=0
and programs in the VM use the port /tmp/thingy.

Uses only the Python standard library. Works on macOS and Linux.
"""
import argparse
import glob
import os
import select
import socket
import sys
import termios
import time
import tty

PORT_PATTERNS = ["/dev/cu.usbserial*", "/dev/cu.wchusbserial*", "/dev/cu.SLAB_USBtoUART*",
                 "/dev/ttyUSB*", "/dev/ttyACM*"]


def find_port():
    ports = sorted({p for pattern in PORT_PATTERNS for p in glob.glob(pattern)})
    if len(ports) == 1:
        return ports[0]
    if not ports:
        sys.exit("No ESP32 serial port found. Plug in the bridge, or pass --port.")
    sys.exit("More than one possible port (" + ", ".join(ports) + "). Pass --port.")


def open_serial(path):
    """Open the port raw at 115200 baud, without waiting for modem signals."""
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    tty.setraw(fd)
    attrs = termios.tcgetattr(fd)
    attrs[2] |= termios.CLOCAL | termios.CREAD  # ignore modem control lines
    attrs[4] = attrs[5] = termios.B115200       # input and output speed
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return fd


def pump(fd, sock):
    """Copy data both ways until either side closes. Returns 'serial' or 'socket' for the side that closed."""
    while True:
        ready, _, _ = select.select([fd, sock], [], [], 1.0)
        if fd in ready:
            try:
                data = os.read(fd, 4096)
            except OSError:
                data = b""
            if not data:
                return "serial"
            try:
                sock.sendall(data)
            except OSError:
                return "socket"
        if sock in ready:
            data = sock.recv(1024)
            if not data:
                return "socket"
            os.write(fd, data)


def connect_out(path, address):
    """Keep a connection open to address, reconnecting to the bridge or the address as needed."""
    host, _, port = address.rpartition(":")
    print(f"Sharing {path} by connecting to {address}. Press Ctrl+C to stop.", flush=True)
    fd = sock = None
    try:
        while True:
            if fd is None:
                try:
                    fd = open_serial(path)
                    print(f"Bridge connected on {path}", flush=True)
                except OSError:
                    time.sleep(2)
                    continue
            if sock is None:
                try:
                    sock = socket.create_connection((host, int(port)), timeout=5)
                    sock.settimeout(None)
                    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    print(f"Connected to {address}", flush=True)
                except OSError as e:
                    print(f"Can't reach {address} yet ({e}). Trying again...", flush=True)
                    time.sleep(3)
                    continue
            closed = pump(fd, sock)
            if closed == "serial":
                print("Bridge disconnected, waiting for it...", flush=True)
                os.close(fd)
                fd = None
            else:
                print(f"{address} closed the connection. Reconnecting...", flush=True)
                sock.close()
                sock = None
                time.sleep(1)
    except KeyboardInterrupt:
        print()
    finally:
        if sock:
            sock.close()
        if fd is not None:
            os.close(fd)


def main():
    ap = argparse.ArgumentParser(description="Share the Thingy Bridge's USB port over TCP")
    ap.add_argument("--port", help="serial port (found automatically if left out)")
    ap.add_argument("--host", default="192.168.64.1",
                    help="address to listen on (default: the Mac's address on UTM's shared network)")
    ap.add_argument("--tcp-port", type=int, default=5555)
    ap.add_argument("--connect", metavar="HOST:PORT", help="connect out to this address instead of listening")
    args = ap.parse_args()
    path = args.port or find_port()
    if args.connect:
        connect_out(path, args.connect)
        return

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        server.bind((args.host, args.tcp_port))
    except OSError as e:
        sys.exit(f"Can't listen on {args.host}:{args.tcp_port} ({e}). "
                 "Start the virtual machine first, or pass --host.")
    server.listen(1)
    print(f"Sharing {path} at socket://{args.host}:{args.tcp_port}. Press Ctrl+C to stop.", flush=True)

    fd, client = None, None
    try:
        while True:
            if fd is None:  # (re)open the bridge, for example after it was unplugged
                try:
                    fd = open_serial(path)
                    print(f"Bridge connected on {path}", flush=True)
                except OSError:
                    time.sleep(2)
                    continue
            watch = [server, fd] + ([client] if client else [])
            ready, _, _ = select.select(watch, [], [], 1.0)
            if server in ready:
                new, addr = server.accept()
                if client:
                    client.close()  # one program at a time: the newest wins
                client = new
                client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                print(f"Program connected from {addr[0]}", flush=True)
            if fd in ready:
                try:
                    data = os.read(fd, 4096)
                except OSError:
                    data = b""
                if not data:
                    print("Bridge disconnected, waiting for it...", flush=True)
                    os.close(fd)
                    fd = None
                    continue
                if client:
                    try:
                        client.sendall(data)
                    except OSError:
                        client.close()
                        client = None
            if client and client in ready:
                data = client.recv(1024)
                if data:
                    os.write(fd, data)  # commands such as "i" or "c" go to the bridge
                else:
                    print("Program disconnected", flush=True)
                    client.close()
                    client = None
    except KeyboardInterrupt:
        print()
    finally:
        if client:
            client.close()
        if fd is not None:
            os.close(fd)
        server.close()


if __name__ == "__main__":
    main()
