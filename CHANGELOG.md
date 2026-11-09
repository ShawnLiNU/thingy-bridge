# Changelog

## 1.2 (firmware 1.1.3)

- Firmware reads the Thingy's temperature, humidity, pressure, air quality, light, and battery, and sends them once a second
- The motion demo shows the room readings live
- `examples/ros2/thingy_ros.py`: every reading as a ROS 2 topic, a colored map of readings, and CSV logging (`docs/TURTLEBOT4.md`)
- `python/serial_relay.py`: shares the bridge's USB port over the network, for virtual machines, in either direction
- All scripts accept `socket://HOST:PORT` in place of a serial port, and skip a TurtleBot 4's lidar port
- `tilt_teleop.py`: learns "flat" when you arm, stops while the Thingy lies still, uses simulation time in simulators, and always sends a final stop
- Simulator guide for a Mac with UTM, tested with the TurtleBot3 World (`docs/SIMULATOR.md`)
- Firmware: `s` command lists nearby Bluetooth devices, recognizes a renamed Thingy, and skips a Thingy in update mode ("ThingyDfu")

## 1.0 (firmware 1.0.0)

- First version: Bluetooth keyboard gestures, the USB motion stream, the 3D motion demo, a keyboard-controlled web page, a Python reader, and tilt-to-drive for ROS 2
