# Thingy Bridge

Tilt, tap, and flip a Nordic Thingy:52 to control TVs, web pages, and robots.

An ESP32 reads the Thingy's sensors over Bluetooth and does two things at once: it acts as a **Bluetooth keyboard**, so anything that accepts a keyboard can be controlled with no extra software, and it **streams the data over USB**: orientation, gestures, temperature, humidity, pressure, air quality, light, and battery. No wiring and no changes to the Thingy's firmware.

## Demo

https://github.com/user-attachments/assets/0d968c52-f987-4dff-b26a-3f040a129814

Tilting the Thingy moves the 3D view in the motion demo and steps through a web page in real time.

## One controller, many machines

```mermaid
flowchart LR
    thingy["Thingy:52<br/>tilt, tap, flip"] -- Bluetooth --> bridge["ESP32 bridge"]

    bridge -- "Bluetooth keyboard" --> keys
    bridge -- "USB data stream" --> stream

    subgraph keys [" "]
        direction TB
        laptop["Web pages and slides<br/>on a laptop"]
        tv["TV photo app"]
        phone["Phone or tablet"]
    end

    subgraph stream [" "]
        direction TB
        demo["3D motion demo"]
        python["Python scripts"]
        ros["ROS 2"]
    end

    ros --> turtlebot["TurtleBot 4"]
    ros --> sim["TurtleBot simulators<br/>in Gazebo"]
    ros --> climate["Climate map<br/>of a building"]
    ros --> arm["UR12e arm"]
    ros --> dog["Unitree robot dog"]

    classDef planned stroke-dasharray: 5 5
    class arm,dog planned
```

Dashed boxes are planned.

| Application | Output | Status |
|---|---|---|
| Web pages, slides, and keyboard-driven apps on a laptop | Keyboard | Tested on macOS |
| [3D motion demo](web/motion-demo.html) | USB stream | Tested |
| [Example web page](examples/keyboard-webpage/index.html) | Keyboard | Tested |
| TV photo app (LG webOS) | Keyboard | Ready to try |
| [Python scripts](python/thingy_bridge.py) | USB stream | Included |
| [TurtleBot 4 driving with ROS 2](examples/ros2/tilt_teleop.py) | USB stream | Example, not yet run on a robot |
| [TurtleBot3 simulator on a Mac (UTM and Gazebo)](docs/SIMULATOR.md) | USB stream, shared over the network | Tested |
| [TurtleBot 4 simulator on a Mac](docs/SIMULATOR.md#the-turtlebot-4-simulator) | USB stream, shared over the network | Tested, slow in a virtual machine |
| [Climate map on a TurtleBot 4, viewed live in Lichtblick](docs/TURTLEBOT4.md) | USB stream | Example, not yet run on a robot |
| UR12e arm | USB stream | Planned |
| Unitree robot dog | USB stream | Planned |

## Gestures

Hold the Thingy flat, logo up.

| Gesture | Key | In the stream |
|---|---|---|
| Tilt left / right | Left / Right | `lr` angle and a `key` event |
| Tilt forward / back | Up / Down | `ud` angle and a `key` event |
| Single tap | Enter | `key` event |
| Double tap | Play/Pause | `key` event |
| Flip face down for about 1 second | Escape | `key` event |
| Triple tap | Learn "flat" again | |

Holding a tilt repeats the key, faster the further you tilt, so a small tilt steps slowly and a big tilt scrolls quickly. The stream also carries the full orientation (a quaternion), the gravity vector, and the room readings once a second. See [docs/PROTOCOL.md](docs/PROTOCOL.md).

## Quick start

### You need

| Item | Explain |
|---|---|
| Nordic Thingy:52 | Charged, with its stock firmware |
| ESP32 board | Tested with ESP32-WROOM-32E. Other ESP32 boards with Bluetooth Low Energy should work. |
| USB data cable | Some cables only charge. Use one that transfers data. |
| Arduino IDE 2 | Free, from arduino.cc |

### Set up the firmware

1. In **Arduino IDE > Settings**, add this to "Additional boards manager URLs":
   `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
2. In the **Boards Manager**, install **esp32 by Espressif Systems**.
3. In the **Library Manager**, install **NimBLE-Arduino by h2zero** (version 2.x).
4. Open `firmware/ThingyBridge/ThingyBridge.ino`, choose **Tools > Board > esp32 > ESP32 Dev Module** and the board's port, then **Upload**.

### Try it

1. Pair **Thingy Bridge** in your computer's Bluetooth settings.
2. Close the Arduino Serial Monitor, then open `web/motion-demo.html` in Chrome or Edge and choose **Connect bridge**.
3. Press the Thingy's button to wake it, and hold it still for a second.
4. Tilt it. The 3D view follows, and each gesture shows up as a key marked **Received** once your computer gets it as a keyboard key. The room readings appear below, updated once a second.

To see the demo page without hardware, open it with `?demo=1` at the end of the address. If something doesn't work, see [docs/TUNING.md](docs/TUNING.md).

### Read the stream in Python

```bash
pip install pyserial
python3 python/thingy_bridge.py
```

```python
from thingy_bridge import Bridge

with Bridge() as bridge:            # finds the port by itself
    for msg in bridge.messages():
        if msg["t"] == "key":
            print("gesture:", msg["k"])
```

### Drive a TurtleBot 4 (ROS 2)

On the computer the bridge is plugged into, with ROS 2 sourced:

```bash
python3 examples/ros2/tilt_teleop.py
```

Double tap to arm (it learns how you're holding it as "flat"), tilt forward and sideways to drive, and flip face down to stop. Putting it down on a table also stops the robot until you pick it up. Speeds are capped at 0.15 m/s and 0.8 rad/s by default. Use `--ros-args -p stamped:=true` if your robot expects `TwistStamped`, as the TurtleBot 4 does on ROS 2 Jazzy.

No robot handy? [docs/SIMULATOR.md](docs/SIMULATOR.md) drives a simulated TurtleBot in Gazebo, in an Ubuntu virtual machine on a Mac, with the lidar beams shown as it moves. `python/serial_relay.py` shares the bridge's USB port over the network, so any script here can use `socket://HOST:5555` in place of a serial port.

### Map a building's climate with a TurtleBot 4

With the Thingy on the robot and the bridge plugged into the robot's Raspberry Pi, `examples/ros2/thingy_ros.py` publishes every reading as a ROS 2 topic and drops a colored dot on the robot's map for each one. Lichtblick shows the map, the dots, and live charts on your laptop. Setup is in [docs/TURTLEBOT4.md](docs/TURTLEBOT4.md).

## Repository layout

```
firmware/ThingyBridge/     ESP32 firmware (Arduino)
web/motion-demo.html       live 3D view and key check, over USB (Web Serial)
python/thingy_bridge.py    Python reader for the USB stream
python/serial_relay.py     shares the bridge's USB port with a virtual machine
examples/keyboard-webpage/ a web page controlled only by keys
examples/ros2/             tilt to drive a mobile robot, and all readings as ROS 2 topics
docs/PROTOCOL.md           keys, stream format, and commands
docs/TUNING.md             settings and troubleshooting
docs/TURTLEBOT4.md         the bridge on a TurtleBot 4, with Lichtblick
docs/SIMULATOR.md          drive a simulated TurtleBot on a Mac
CHANGELOG.md               what changed in each version
```

## Safety with robots

The stream gives a program everything it needs to move a machine, so treat the bridge like any teleoperation device:

- Start disarmed, and require a deliberate gesture to arm.
- Cap speeds, and stop whenever data is late or the Thingy disconnects.
- Test in simulation first, keep the robot's emergency stop within reach, and keep people clear of arms.

`examples/ros2/tilt_teleop.py` follows these rules and is a starting point for other robots.

## Roadmap

- UR12e arm: Cartesian jogging through MoveIt Servo, with a hold-to-move gesture
- Unitree robot dog: walking and turning through its SDK
- Wireless link to robots (ESP-NOW or Wi-Fi), so the bridge doesn't need a USB cable to a computer
- Learned gestures such as circles and shakes, trained on the quaternion stream
- Key maps that can be changed without reflashing

## Built with

- [Nordic Thingy:52](https://www.nordicsemi.com/Products/Development-hardware/Nordic-Thingy-52) and its stock firmware
- [NimBLE-Arduino](https://github.com/h2zero/NimBLE-Arduino) by h2zero
- [Arduino core for the ESP32](https://github.com/espressif/arduino-esp32) by Espressif

## License

MIT. See [LICENSE](LICENSE).
