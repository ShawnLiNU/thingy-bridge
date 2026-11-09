# Tuning and troubleshooting

## Settings

At the top of `firmware/ThingyBridge/ThingyBridge.ino`:

| Item | Explain |
|---|---|
| `DEVICE_NAME` | The keyboard name hosts see when pairing |
| `TILT_ON_DEG`, `TILT_OFF_DEG` | Tilt angles where a key presses and releases (25 and 12 degrees) |
| `TILT_FAST_DEG` | Tilt where repeats are fastest (55 degrees) |
| `REPEAT_SLOW_MS`, `REPEAT_FAST_MS` | Repeat interval at a small and a big tilt |
| `REPEAT_LIMIT_MS` | Stop repeating after a tilt is held this long, for example when left on a shelf |
| `SWAP_AXES`, `INVERT_LR`, `INVERT_UD` | Fix swapped or reversed directions |
| `BACK_KEY` | Escape by default. Some TVs need `0x2A` (Backspace). |
| `STREAM`, `STREAM_MS` | The USB stream and its update interval |
| `LED_PIN` | Status light: fast blink waits for a host, slow blink looks for the Thingy, steady means ready |

## Troubleshooting

| Item | Explain |
|---|---|
| No port for the board | Use a USB cable that transfers data, and try without a hub. If the Mac lists the board's USB chip but no port, install the free driver for it: CP210x (Silicon Labs) or CH340/CH9102 (WCH). |
| Compile error about the library | Install NimBLE-Arduino 2.x and choose the ESP32 Dev Module board |
| The host doesn't list Thingy Bridge | Press the board's EN button to restart it, and remove any old pairing from the host first |
| The Thingy never connects | Press its button to wake it, and disconnect it from the Thingy phone app, since it accepts one connection at a time |
| A second host can't connect | The bridge pairs with one host at a time. Turn off Bluetooth on the first one, or remove the pairing there. |
| Gestures do nothing, but test keys work | Watch the gauges in the motion demo. If they stay near 0, choose Set as flat or triple tap. |
| The demo says "Not received" | The bridge sent the key but the computer didn't get it. Check the Bluetooth connection, and click the page so it has focus. |
| Double tap starts the Music app on a Mac | That is the Play/Pause key working |
