#!/usr/bin/env python3
"""Publish everything the Thingy Bridge sends as ROS 2 topics, and map the readings.

Run on the robot (for example a TurtleBot 4) with the bridge plugged into it:
    python3 thingy_ros.py
    python3 thingy_ros.py --ros-args -p map_value:=temperature -p csv:=~/thingy_log.csv

Topics (all under /thingy):
    imu            sensor_msgs/Imu               orientation and gravity
    tilt           geometry_msgs/Vector3         x: left/right, y: back/forward, in degrees
    gesture        std_msgs/String               Left, Right, Up, Down, Enter, Back, Play/Pause
    temperature    sensor_msgs/Temperature       degrees C
    humidity       sensor_msgs/RelativeHumidity  0 to 1
    pressure       sensor_msgs/FluidPressure     pascals
    eco2           std_msgs/Float32              estimated CO2 in ppm (calculated from VOCs)
    tvoc           std_msgs/Float32              volatile organic compounds in ppb
    light          std_msgs/Float32              raw brightness from the color sensor (not lux)
    battery        sensor_msgs/BatteryState      Thingy battery, 0 to 1
    reading_map    visualization_msgs/Marker     a colored dot for each reading, where the robot was

The map needs the robot's position: a "map" frame from SLAM or localization, and the robot's
"base_link" frame. Without them, everything except reading_map still works.

Requires pyserial (pip install pyserial) and a ROS 2 installation.
"""
import csv
import glob
import json
import math
import os
import queue
import threading
import time

import rclpy
import serial
from geometry_msgs.msg import Point, TransformStamped, Vector3
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import BatteryState, FluidPressure, Imu, RelativeHumidity, Temperature
from serial.tools import list_ports
from std_msgs.msg import ColorRGBA, Float32, String
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener
from visualization_msgs.msg import Marker

PORT_HINTS = ("usbserial", "wchusbserial", "SLAB_USBtoUART", "ttyUSB", "ttyACM")
PREFERRED_PORT = "/dev/thingy_bridge"  # a fixed name, set up as in docs/TURTLEBOT4.md

# Which stream field each mappable reading comes from
MAP_FIELDS = {
    "temperature": lambda e: e.get("temp"),
    "humidity": lambda e: e.get("hum"),
    "pressure": lambda e: e.get("press"),
    "eco2": lambda e: e.get("eco2"),
    "tvoc": lambda e: e.get("tvoc"),
    "light": lambda e: e["rgbc"][3] if "rgbc" in e else None,
}
CSV_FIELDS = ["time", "x", "y", "temp", "hum", "press", "eco2", "tvoc", "light", "bat"]


def find_port():
    """Use /dev/thingy_bridge if it exists, otherwise the one ESP32-style port that no lidar uses."""
    if os.path.exists(PREFERRED_PORT):
        return PREFERRED_PORT
    taken = {os.path.realpath(p) for p in glob.glob("/dev/*")
             if os.path.islink(p) and "lidar" in os.path.basename(p).lower()}
    ports = [p.device for p in list_ports.comports()
             if any(hint in p.device for hint in PORT_HINTS) and os.path.realpath(p.device) not in taken]
    if not ports:
        raise RuntimeError("No ESP32 serial port found. Plug in the bridge, or set -p port:=...")
    if len(ports) > 1:
        raise RuntimeError("More than one possible port (" + ", ".join(ports) + "). Set -p port:=...")
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


def color_for(t):
    """Blue (low) to green, yellow, and red (high) for t from 0 to 1."""
    stops = [(0.0, (0.15, 0.35, 1.0)), (0.35, (0.1, 0.8, 0.5)), (0.65, (1.0, 0.85, 0.1)), (1.0, (0.95, 0.2, 0.15))]
    t = min(1.0, max(0.0, t))
    for (t0, c0), (t1, c1) in zip(stops, stops[1:]):
        if t <= t1:
            f = (t - t0) / (t1 - t0)
            return tuple(a + (b - a) * f for a, b in zip(c0, c1))
    return stops[-1][1]


class ThingyRos(Node):
    def __init__(self):
        super().__init__("thingy_bridge")
        params = {
            "port": "",
            "frame_id": "thingy_link",        # frame for the Thingy's own messages
            "parent_frame": "base_link",      # where the Thingy is mounted
            "mount_xyz": [0.0, 0.0, 0.25],    # Thingy position on the robot, in meters
            "map_frame": "map",
            "base_frame": "base_link",
            "map_value": "tvoc",              # temperature, humidity, pressure, eco2, tvoc, or light
            "map_min": float("nan"),          # color range; leave unset to fit the readings so far
            "map_max": float("nan"),
            "marker_spacing": 0.25,           # meters between dots on the map
            "max_markers": 5000,
            "csv": "",                        # file to log every reading with the robot position
        }
        for name, default in params.items():
            self.declare_parameter(name, default)
        p = lambda name: self.get_parameter(name).value
        self.p = p
        if p("map_value") not in MAP_FIELDS:
            raise ValueError("map_value must be one of: " + ", ".join(MAP_FIELDS))

        # Publishers
        topic = lambda name: f"/thingy/{name}"
        self.pub = {
            "imu": self.create_publisher(Imu, topic("imu"), 10),
            "tilt": self.create_publisher(Vector3, topic("tilt"), 10),
            "gesture": self.create_publisher(String, topic("gesture"), 10),
            "temperature": self.create_publisher(Temperature, topic("temperature"), 10),
            "humidity": self.create_publisher(RelativeHumidity, topic("humidity"), 10),
            "pressure": self.create_publisher(FluidPressure, topic("pressure"), 10),
            "eco2": self.create_publisher(Float32, topic("eco2"), 10),
            "tvoc": self.create_publisher(Float32, topic("tvoc"), 10),
            "light": self.create_publisher(Float32, topic("light"), 10),
            "battery": self.create_publisher(BatteryState, topic("battery"), 10),
            "map": self.create_publisher(Marker, topic("reading_map"), 10),
        }

        # Where the Thingy sits on the robot, so its messages have a place in the 3D view
        self.static_tf = StaticTransformBroadcaster(self)
        mount = TransformStamped()
        mount.header.stamp = self.get_clock().now().to_msg()
        mount.header.frame_id = p("parent_frame")
        mount.child_frame_id = p("frame_id")
        mount.transform.translation.x, mount.transform.translation.y, mount.transform.translation.z = p("mount_xyz")
        mount.transform.rotation.w = 1.0
        self.static_tf.sendTransform(mount)

        # Robot position, for the map of readings
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.points, self.values = [], []
        self.last_point = None

        # Optional CSV log
        self.csv_file = self.csv_writer = None
        if p("csv"):
            path = os.path.expanduser(p("csv"))
            new = not os.path.exists(path)
            self.csv_file = open(path, "a", newline="")
            self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=CSV_FIELDS)
            if new:
                self.csv_writer.writeheader()

        # Serial input runs in its own thread; messages are handled in the ROS timer below.
        port = p("port") or find_port()
        self.serial = open_port(port, 0.2)
        self.inbox = queue.Queue(maxsize=500)
        self.running = True
        threading.Thread(target=self.read_loop, daemon=True).start()
        self.create_timer(0.02, self.handle_messages)   # 50 Hz
        self.create_timer(1.0, self.publish_map)
        self.get_logger().info(f"Reading {port}. Mapping {p('map_value')} in the '{p('map_frame')}' frame.")

    # ---------- input ----------
    def read_loop(self):
        while self.running:
            try:
                line = self.serial.readline().decode("utf-8", "replace").strip()
            except serial.SerialException as e:
                self.get_logger().error(f"Bridge disconnected: {e}")
                return
            if line.startswith("{"):
                try:
                    self.inbox.put_nowait(json.loads(line))
                except (ValueError, queue.Full):
                    pass

    def handle_messages(self):
        while True:
            try:
                msg = self.inbox.get_nowait()
            except queue.Empty:
                return
            kind = msg.get("t")
            if kind == "m":
                self.on_motion(msg)
            elif kind == "env":
                self.on_environment(msg)
            elif kind == "key":
                self.pub["gesture"].publish(String(data=msg.get("k", "")))

    # ---------- motion ----------
    def on_motion(self, m):
        stamp = self.get_clock().now().to_msg()
        imu = Imu()
        imu.header.stamp, imu.header.frame_id = stamp, self.p("frame_id")
        if "q" in m:
            w, x, y, z = m["q"]  # the Thingy sends w first, ROS puts it last
            imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w = x, y, z, w
        else:
            imu.orientation_covariance[0] = -1.0  # no orientation
        imu.angular_velocity_covariance[0] = -1.0  # not provided
        if "g" in m:
            imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = m["g"]
        self.pub["imu"].publish(imu)
        self.pub["tilt"].publish(Vector3(x=float(m.get("lr", 0.0)), y=float(m.get("ud", 0.0)), z=0.0))

    # ---------- climate ----------
    def on_environment(self, e):
        stamp = self.get_clock().now().to_msg()
        frame = self.p("frame_id")
        if "temp" in e:
            msg = Temperature(temperature=float(e["temp"]), variance=0.0)
            msg.header.stamp, msg.header.frame_id = stamp, frame
            self.pub["temperature"].publish(msg)
        if "hum" in e:
            msg = RelativeHumidity(relative_humidity=e["hum"] / 100.0, variance=0.0)
            msg.header.stamp, msg.header.frame_id = stamp, frame
            self.pub["humidity"].publish(msg)
        if "press" in e:
            msg = FluidPressure(fluid_pressure=e["press"] * 100.0, variance=0.0)  # hPa to Pa
            msg.header.stamp, msg.header.frame_id = stamp, frame
            self.pub["pressure"].publish(msg)
        if "eco2" in e:
            self.pub["eco2"].publish(Float32(data=float(e["eco2"])))
            self.pub["tvoc"].publish(Float32(data=float(e["tvoc"])))
        if "rgbc" in e:
            self.pub["light"].publish(Float32(data=float(e["rgbc"][3])))
        if "bat" in e:
            msg = BatteryState(percentage=e["bat"] / 100.0, present=True)
            msg.header.stamp, msg.header.frame_id = stamp, frame
            self.pub["battery"].publish(msg)

        pose = self.robot_position()
        value = MAP_FIELDS[self.p("map_value")](e)
        if pose and value is not None:
            if self.last_point is None or math.dist(pose, self.last_point) >= self.p("marker_spacing"):
                self.points.append(pose)
                self.values.append(float(value))
                self.last_point = pose
                if len(self.points) > self.p("max_markers"):
                    self.points.pop(0)
                    self.values.pop(0)
        if self.csv_writer:
            row = {"time": round(time.time(), 2), "x": pose[0] if pose else "", "y": pose[1] if pose else "",
                   "light": e["rgbc"][3] if "rgbc" in e else ""}
            row.update({k: e.get(k, "") for k in ("temp", "hum", "press", "eco2", "tvoc", "bat")})
            self.csv_writer.writerow(row)
            self.csv_file.flush()

    def robot_position(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.p("map_frame"), self.p("base_frame"), Time())
        except Exception:  # no map yet, or the robot isn't localized
            self.get_logger().warn("No robot position yet, so readings aren't placed on the map",
                                   throttle_duration_sec=15.0)
            return None
        t = tf.transform.translation
        return (t.x, t.y)

    # ---------- map of readings ----------
    def publish_map(self):
        if not self.points:
            return
        lo, hi = self.p("map_min"), self.p("map_max")
        if math.isnan(lo):
            lo = min(self.values)
        if math.isnan(hi):
            hi = max(self.values)
        span = (hi - lo) or 1.0
        m = Marker()
        m.header.frame_id = self.p("map_frame")
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id = "thingy_readings", 0
        m.type, m.action = Marker.SPHERE_LIST, Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.18
        for (x, y), v in zip(self.points, self.values):
            m.points.append(Point(x=x, y=y, z=0.05))
            r, g, b = color_for((v - lo) / span)
            m.colors.append(ColorRGBA(r=r, g=g, b=b, a=0.9))
        self.pub["map"].publish(m)

    def stop(self):
        self.running = False
        self.serial.close()
        if self.csv_file:
            self.csv_file.close()


def main():
    rclpy.init()
    node = ThingyRos()
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
