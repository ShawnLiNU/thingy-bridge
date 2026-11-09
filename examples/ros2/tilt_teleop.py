#!/usr/bin/env python3
"""Drive a mobile robot (for example a TurtleBot 4) by tilting the Thingy:52.

Reads the Thingy Bridge's USB stream and publishes velocity commands:
    tilt forward / back   drive forward / backward
    tilt left / right     turn left / right
    double tap            arm or disarm (starts disarmed). Arming also makes the way you
                          hold the Thingy at that moment its "flat", so hold it still briefly.
    flip face down        disarm and stop
    put it down           stops while the Thingy lies still, and drives again when picked up

Safety: the robot only moves while armed, speeds are capped, and it stops
whenever data is late (bridge unplugged, Thingy asleep) or the node exits.
In a simulator it switches to simulation time by itself, so the robot's own
command timeout works (otherwise a robot can keep driving if commands stop).

Run on the computer the bridge is plugged into, in a terminal with ROS 2 sourced:
    python3 tilt_teleop.py
    python3 tilt_teleop.py --ros-args -p port:=/dev/ttyUSB0 -p max_linear:=0.2
From a virtual machine, with python/serial_relay.py running on the host:
    python3 tilt_teleop.py --ros-args -p port:=socket://192.168.64.1:5555
For TwistStamped (TurtleBot 4 on ROS 2 Jazzy, including its simulator):
    python3 tilt_teleop.py --ros-args -p stamped:=true

Requires pyserial (pip install pyserial) and a ROS 2 installation (rclpy, geometry_msgs).
"""
import collections
import glob
import json
import math
import os
import threading
import time

import rclpy
import serial
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from serial.tools import list_ports

PORT_HINTS = ("usbserial", "wchusbserial", "SLAB_USBtoUART", "ttyUSB", "ttyACM")
PREFERRED_PORT = "/dev/thingy_bridge"  # a fixed name, set up as in docs/TURTLEBOT4.md


def find_port():
    """Return the bridge's serial port.

    Uses /dev/thingy_bridge if it exists. Otherwise looks for ESP32-style ports, skipping any
    port a lidar already uses (the TurtleBot 4's lidar is also a USB serial device). Raises
    RuntimeError if there is no port, or more than one possible port.
    """
    if os.path.exists(PREFERRED_PORT):
        return PREFERRED_PORT
    taken = {os.path.realpath(p) for p in glob.glob("/dev/*")
             if os.path.islink(p) and "lidar" in os.path.basename(p).lower()}
    ports = [p.device for p in list_ports.comports()
             if any(hint in p.device for hint in PORT_HINTS) and os.path.realpath(p.device) not in taken]
    if not ports:
        raise RuntimeError("No ESP32 serial port found. Plug in the bridge, or give the port.")
    if len(ports) > 1:
        raise RuntimeError("More than one possible port (" + ", ".join(ports) + "). Give the port to use.")
    return ports[0]


def open_port(port, timeout):
    """Open a serial port, or a network address such as socket://192.168.64.1:5555 (see serial_relay.py)."""
    link = serial.serial_for_url(port, 115200, timeout=timeout)
    try:
        link.dtr = False  # keep the board running: some adapters reset it when these lines change
        link.rts = False
    except (OSError, ValueError, serial.SerialException):
        pass  # network connections have no such lines
    return link


def scale(deg, deadzone, full):
    """Map a tilt angle to -1..1: zero inside the dead zone, 1 at full tilt."""
    mag = abs(deg)
    if mag <= deadzone:
        return 0.0
    value = min(1.0, (mag - deadzone) / (full - deadzone))
    return value if deg > 0 else -value


class TiltTeleop(Node):
    def __init__(self):
        super().__init__("thingy_tilt_teleop")
        self.declare_parameter("port", "")
        self.declare_parameter("topic", "/cmd_vel")
        self.declare_parameter("stamped", False)
        self.declare_parameter("max_linear", 0.15)   # m/s, keep low indoors
        self.declare_parameter("max_angular", 0.8)   # rad/s
        self.declare_parameter("deadzone_deg", 8.0)  # small tilts do nothing
        self.declare_parameter("full_tilt_deg", 35.0)
        self.declare_parameter("timeout_s", 0.3)     # stop if no data for this long
        self.declare_parameter("invert_forward", False)
        self.declare_parameter("invert_turn", False)
        self.declare_parameter("start_armed", False)  # for simulators only: skip the double tap
        self.declare_parameter("flat_on_arm", True)   # learn "flat" from how you hold it when arming
        self.declare_parameter("stop_when_still", True)  # stop while the Thingy lies still on a surface
        self.declare_parameter("still_deg", 0.6)      # moving less than this counts as lying still...
        self.declare_parameter("still_s", 1.5)        # ...for this long

        p = lambda name: self.get_parameter(name).value
        port = p("port") or find_port()  # set -p port:=... if more than one port is found
        self.stamped = p("stamped")
        msg_type = TwistStamped if self.stamped else Twist
        self.pub = self.create_publisher(msg_type, p("topic"), 10)
        self.serial = open_port(port, 0.2)

        self.lock = threading.Lock()
        self.lr = self.ud = 0.0
        self.last_data = 0.0
        self.calibrating = False
        self.cal_sent = 0.0
        self.recent = collections.deque()  # (time, gravity direction) for the stillness check
        self.still = False
        self.armed = bool(p("start_armed"))
        self.running = True
        threading.Thread(target=self.read_loop, daemon=True).start()
        self.create_timer(0.05, self.publish)  # 20 Hz
        self.sim_check = self.create_timer(2.0, self.check_for_simulator)
        self.get_logger().info(f"Reading {port}, publishing on {p('topic')}. "
                               + ("Armed: tilt to drive." if self.armed else "Double tap the Thingy to arm."))

    def read_loop(self):
        while self.running:
            try:
                line = self.serial.readline().decode("utf-8", "replace").strip()
            except (serial.SerialException, OSError) as e:
                self.get_logger().error(f"Bridge disconnected: {e}. Stopped and disarmed.")
                with self.lock:
                    self.armed = False
                return
            if not line.startswith("{"):
                continue
            try:
                msg = json.loads(line)
            except ValueError:
                continue
            with self.lock:
                if msg.get("t") == "m":
                    self.lr, self.ud, self.last_data = msg["lr"], msg["ud"], time.monotonic()
                    if "g" in msg:
                        self.update_stillness(msg["g"])
                elif msg.get("t") == "key" and msg.get("k") == "Play/Pause":
                    self.armed = not self.armed
                    if self.armed and self.get_parameter("flat_on_arm").value:
                        self.calibrating = True  # hold still until the bridge reports it's done
                        self.cal_sent = time.monotonic()
                        self.serial.write(b"c")
                        self.get_logger().info("Armed. Hold the Thingy the way you like for a moment...")
                    else:
                        self.get_logger().info("Armed: tilt to drive" if self.armed else "Disarmed")
                elif msg.get("t") == "state" and "cal" in msg:
                    if msg["cal"]:
                        self.calibrating = True
                    elif self.calibrating and time.monotonic() - self.cal_sent > 0.5:  # skip a report sent before "c" arrived
                        self.calibrating = False
                        if self.armed:
                            self.get_logger().info("Ready: tilt to drive")
                elif msg.get("t") == "key" and msg.get("k") == "Back" and self.armed:
                    self.armed = False
                    self.get_logger().info("Disarmed (flipped)")
                if msg.get("t") == "state" and not msg.get("thingy") and self.armed:
                    self.armed = False
                    self.get_logger().warn("Thingy disconnected, disarmed")

    def check_for_simulator(self):
        """A simulator publishes /clock. Then stamp commands with simulation time, as the robot expects:
        with wall-clock stamps, a simulated robot may never time out an old command."""
        self.sim_check.cancel()
        if self.count_publishers("/clock") > 0 and not self.get_parameter("use_sim_time").value:
            self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
            self.get_logger().info("Simulator found: using simulation time")

    def update_stillness(self, g):
        """Lying on a table, the Thingy barely moves; in a hand it always shakes a little."""
        norm = math.sqrt(sum(v * v for v in g)) or 1.0
        unit = tuple(v / norm for v in g)
        now = time.monotonic()
        self.recent.append((now, unit))
        window = self.get_parameter("still_s").value
        while self.recent and now - self.recent[0][0] > window:
            self.recent.popleft()
        if not self.recent or now - self.recent[0][0] < window * 0.8:
            return  # not enough history yet
        spread = max(math.degrees(math.acos(max(-1.0, min(1.0, sum(a * b for a, b in zip(u, unit))))))
                     for _, u in self.recent)
        still = spread < self.get_parameter("still_deg").value
        if still != self.still and self.armed:
            self.get_logger().info("Thingy is lying still: stopped" if still else "Picked up: driving again")
        self.still = still

    def publish(self):
        p = lambda name: self.get_parameter(name).value
        with self.lock:
            fresh = time.monotonic() - self.last_data < p("timeout_s")
            if self.calibrating and time.monotonic() - self.cal_sent > 3.0:
                self.calibrating = False  # the bridge never answered; don't stay stuck
            resting = self.still and p("stop_when_still")
            go = self.armed and fresh and not self.calibrating and not resting
            lr, ud = self.lr, self.ud
        twist = Twist()
        if go:
            fwd = scale(ud, p("deadzone_deg"), p("full_tilt_deg"))
            turn = scale(lr, p("deadzone_deg"), p("full_tilt_deg"))
            if p("invert_forward"):
                fwd = -fwd
            if not p("invert_turn"):
                turn = -turn  # tilting right turns right, which is a negative z rotation in ROS
            twist.linear.x = fwd * p("max_linear")
            twist.angular.z = turn * p("max_angular")
        self.send(twist)

    def send(self, twist):
        if self.stamped:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.twist = twist
            self.pub.publish(msg)
        else:
            self.pub.publish(twist)

    def stop(self):
        self.running = False
        for _ in range(3):
            self.send(Twist())  # make sure the robot ends stopped
            time.sleep(0.05)
        self.serial.close()


def main():
    # Handle Ctrl+C ourselves, so ROS is still running when the final stop command is sent.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = TiltTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
