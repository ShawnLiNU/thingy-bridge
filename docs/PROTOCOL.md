# Protocol

The bridge has two outputs. Use the keyboard when the target already understands keys, and the USB stream when a program needs the motion itself.

## Bluetooth keyboard

The bridge pairs as a standard Bluetooth Low Energy keyboard named **Thingy Bridge**. Any host that accepts a Bluetooth keyboard works without drivers.

| Gesture | Key | USB HID usage |
|---|---|---|
| Tilt left / right | Left / Right arrow | 0x50 / 0x4F |
| Tilt forward / back | Up / Down arrow | 0x52 / 0x51 |
| Single tap | Enter | 0x28 |
| Double tap | Play/Pause (media key) | Consumer 0x00CD |
| Flip face down, about 1 s | Escape (Backspace if changed in the firmware) | 0x29 / 0x2A |
| Triple tap | Learn "flat" again (no key) | |

Tilts repeat while held: about once a second just past 25 degrees, up to about 6 times a second at 55 degrees and beyond. A tilt held longer than 8 seconds stops repeating until the Thingy returns to flat.

## USB stream

The bridge appears as a USB serial port at **115200 baud**. Each line is one JSON object, and lines that don't start with `{` are plain status text for people.

### Motion, 20 times a second

```json
{"t":"m","q":[0.9981,0.0123,-0.0567,0.0210],"g":[1.70,-0.85,9.61],"lr":10.0,"ud":-5.0}
```

| Field | Meaning |
|---|---|
| `q` | Orientation quaternion `w, x, y, z` from the Thingy's sensor fusion. Missing if the Thingy doesn't provide it. |
| `g` | Gravity vector in m/s², in the Thingy's axes (x right, y forward, z up) |
| `lr` | Left/right tilt in degrees relative to "flat". Positive is right. |
| `ud` | Back/forward tilt in degrees relative to "flat". Positive is the direction that sends Up. |

### Room readings, once a second

```json
{"t":"env","temp":23.45,"hum":41,"press":1013.25,"eco2":412,"tvoc":12,"rgbc":[10,20,30,300],"bat":87}
```

Only sensors that have reported appear, so early messages may have fewer fields.

| Field | Meaning |
|---|---|
| `temp` | Temperature in °C |
| `hum` | Relative humidity in % |
| `press` | Air pressure in hPa |
| `eco2` | Estimated CO2 in ppm. The Thingy calculates it from VOCs, so use it for comparing places, not as a true CO2 measurement. |
| `tvoc` | Volatile organic compounds in ppb |
| `rgbc` | Raw red, green, blue, and clear light readings. Clear is overall brightness, but not in lux. |
| `bat` | Thingy battery in % |

The gas sensor needs about 20 minutes to warm up each time, and a longer break-in on its first use. Temperature and humidity respond slowly, over seconds to tens of seconds.

### Gesture events

```json
{"t":"key","k":"Right","sent":true}
```

`k` is one of `Left`, `Right`, `Up`, `Down`, `Enter`, `Back`, `Play/Pause`. `sent` is `false` when no Bluetooth host is connected, so the key went only to the stream.

### State, on every change and every 2 seconds

```json
{"t":"state","host":true,"thingy":true,"cal":false}
```

`host` is a connected Bluetooth host, `thingy` a connected Thingy, and `cal` is true while the bridge is learning "flat" (hold still).

### Settings, after the `i` command and at startup

```json
{"t":"cfg","on":25.0,"off":12.0,"fast":55.0,"name":"Thingy Bridge","version":"1.0.0"}
```

`on` and `off` are the tilt angles where a key presses and releases, and `fast` is where repeats are fastest.

### Commands to the bridge

Send single characters over the same serial port:

| Command | Action |
|---|---|
| `l` `r` `u` `d` | Send Left, Right, Up, Down |
| `o` `b` `p` | Send Enter, Back, Play/Pause |
| `c` | Learn "flat" again |
| `s` | List the Bluetooth devices the bridge can see, to check that it finds the Thingy |
| `i` | Report settings and state |

To use the stream from another computer or a virtual machine, run `python/serial_relay.py` where the bridge is plugged in. It passes everything through in both directions at `socket://HOST:5555`, and the Python scripts accept that address wherever they take a port.

Opening the port can reset some ESP32 boards. To avoid that, set DTR and RTS to off right after opening, as `python/thingy_bridge.py` and the web demo do.
