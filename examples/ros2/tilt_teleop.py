#!/usr/bin/env python3
"""Drive a mobile robot (for example a TurtleBot 4) by tilting the Thingy:52.

Reads the Thingy Bridge's USB stream and publishes velocity commands:
    tilt forward / back   drive forward / backward
    tilt left / right     turn left / right
    double tap            arm or disarm (starts disarmed)
    flip face down        disarm and stop

Safety: the robot only moves while armed, speeds are capped, and it stops
whenever data is late (bridge unplugged, Thingy asleep) or the node exits.

Run on the computer the bridge is plugged into, in a terminal with ROS 2 sourced:
    python3 tilt_teleop.py
    python3 tilt_teleop.py --ros-args -p port:=/dev/ttyUSB0 -p max_linear:=0.2
For TwistStamped (newer TurtleBot 4 and ROS 2 Jazzy setups):
    python3 tilt_teleop.py --ros-args -p stamped:=true

Requires pyserial (pip install pyserial) and a ROS 2 installation (rclpy, geometry_msgs).
"""
import json
import threading
import time

import rclpy
import serial
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from serial.tools import list_ports

PORT_HINTS = ("usbserial", "wchusbserial", "SLAB_USBtoUART", "ttyUSB", "ttyACM")


def find_port():
    for p in list_ports.comports():
        if any(hint in p.device for hint in PORT_HINTS):
            return p.device
    return None


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

        p = lambda name: self.get_parameter(name).value
        port = p("port") or find_port()
        if not port:
            raise RuntimeError("No ESP32 serial port found. Plug in the bridge or set -p port:=...")
        self.stamped = p("stamped")
        msg_type = TwistStamped if self.stamped else Twist
        self.pub = self.create_publisher(msg_type, p("topic"), 10)
        self.serial = serial.Serial(port, 115200, timeout=0.2)
        self.serial.dtr = False
        self.serial.rts = False

        self.lock = threading.Lock()
        self.lr = self.ud = 0.0
        self.last_data = 0.0
        self.armed = False
        self.running = True
        threading.Thread(target=self.read_loop, daemon=True).start()
        self.create_timer(0.05, self.publish)  # 20 Hz
        self.get_logger().info(f"Reading {port}, publishing on {p('topic')}. Double tap the Thingy to arm.")

    def read_loop(self):
        while self.running:
            try:
                line = self.serial.readline().decode("utf-8", "replace").strip()
            except serial.SerialException as e:
                self.get_logger().error(f"Bridge disconnected: {e}")
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
                elif msg.get("t") == "key" and msg.get("k") == "Play/Pause":
                    self.armed = not self.armed
                    self.get_logger().info("Armed: tilt to drive" if self.armed else "Disarmed")
                elif msg.get("t") == "key" and msg.get("k") == "Back" and self.armed:
                    self.armed = False
                    self.get_logger().info("Disarmed (flipped)")
                elif msg.get("t") == "state" and not msg.get("thingy") and self.armed:
                    self.armed = False
                    self.get_logger().warn("Thingy disconnected, disarmed")

    def publish(self):
        p = lambda name: self.get_parameter(name).value
        with self.lock:
            fresh = time.monotonic() - self.last_data < p("timeout_s")
            go = self.armed and fresh
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
    rclpy.init()
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
