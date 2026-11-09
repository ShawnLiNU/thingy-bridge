/*
  Thingy Bridge: a Nordic Thingy:52 as a tilt-and-tap controller for TVs, computers, and robots.
  https://github.com/ShawnLiNU/thingy-bridge

  The ESP32 does two jobs at once:
    1. Reads the Thingy:52's motion sensor over Bluetooth (stock Thingy firmware).
    2. Turns gestures into Bluetooth keyboard keys for any host (TV, laptop, tablet),
       and streams the motion as JSON lines over USB for programs (web pages, Python, ROS 2).

  Gestures (hold the Thingy flat, logo up):
    Tilt left / right    Left / Right arrow. Hold the tilt to repeat; tilt further to repeat faster.
    Tilt forward / back  Up / Down arrow (repeats the same way)
    Single tap           Enter (OK)
    Double tap           Play/Pause media key
    Triple tap           Re-learn "flat" (hold it the way you like, then triple tap)
    Flip face down       Back (hold upside down for about 1 second)

  Board: ESP32-WROOM-32E ("ESP32 Dev Module" in the Arduino IDE).
  Library: NimBLE-Arduino by h2zero, version 2.x (Library Manager).

  The USB stream format is described in docs/PROTOCOL.md.
  Serial commands (Serial Monitor, the demo page, or your own program):
    l r u d o b p   send Left, Right, Up, Down, OK, Back, Play/Pause by hand
    c               learn "flat" again
    i               report settings and connection state
    s               list the Bluetooth devices the bridge can see (for finding the Thingy)
*/

#include <NimBLEDevice.h>
#include <NimBLEHIDDevice.h>
#include <math.h>

// Defined here, before any function: the Arduino IDE adds declarations of every function
// near the top of the file, and handleAxis() needs this type to already exist there.
struct Axis { bool on; int dir; uint32_t since; uint32_t last; };  // one tilt direction's state
typedef void (*NotifyFn)(NimBLERemoteCharacteristic*, uint8_t*, size_t, bool);  // a Thingy data handler

// ---------------- Settings you may want to change ----------------
static const char*    FIRMWARE_VERSION = "1.1.3";
static const char*    DEVICE_NAME     = "Thingy Bridge"; // Bluetooth keyboard name shown on the host
static const int      LED_PIN         = 2;     // onboard LED on most WROOM-32E boards, -1 for none
static const float    TILT_ON_DEG     = 25;    // tilt past this to press a key
static const float    TILT_OFF_DEG    = 12;    // come back under this to let go
static const float    TILT_FAST_DEG   = 55;    // at this tilt, repeats are fastest
static const uint32_t REPEAT_SLOW_MS  = 900;   // repeat interval just past TILT_ON_DEG
static const uint32_t REPEAT_FAST_MS  = 150;   // repeat interval at TILT_FAST_DEG and beyond
static const uint32_t REPEAT_LIMIT_MS = 8000;  // stop repeating after this long (left tilted on a shelf)
static const uint32_t FLIP_HOLD_MS    = 900;   // how long to hold it upside down for Back
static const uint32_t TAP_WINDOW_MS   = 450;   // time to wait for a second or third tap
static const bool     SWAP_AXES       = false; // set true if left/right and up/down are swapped
static const bool     INVERT_LR       = false; // set true if left and right are reversed
static const bool     INVERT_UD       = false; // set true if up and down are reversed
static const uint8_t  BACK_KEY        = 0x29;  // Escape. If the TV ignores it, try 0x2A (Backspace)
static const bool     STREAM          = true;  // send motion as JSON lines over USB for motion-demo.html
static const uint32_t STREAM_MS       = 50;    // 20 updates per second
static const bool     ENVIRONMENT     = true;  // also read the Thingy's climate, air, light, and battery sensors
static const uint32_t ENV_MS          = 1000;  // send climate readings once a second

// ---------------- Keyboard and media key codes (USB HID) ----------------
static const uint8_t  KEY_RIGHT = 0x4F, KEY_LEFT = 0x50, KEY_DOWN = 0x51, KEY_UP = 0x52, KEY_ENTER = 0x28;
static const uint16_t MEDIA_PLAY_PAUSE = 0x00CD;
static const uint16_t APPEARANCE_KEYBOARD = 0x03C1;

// ---------------- Thingy:52 services (stock firmware) ----------------
static NimBLEUUID THINGY_SERVICE("ef680100-9b35-4933-9b10-52ffa9740042");  // advertised by the Thingy
static NimBLEUUID MOTION_SERVICE("ef680400-9b35-4933-9b10-52ffa9740042");
static NimBLEUUID MOTION_CONFIG ("ef680401-9b35-4933-9b10-52ffa9740042");
static NimBLEUUID TAP_CHAR      ("ef680402-9b35-4933-9b10-52ffa9740042");
static NimBLEUUID QUAT_CHAR     ("ef680404-9b35-4933-9b10-52ffa9740042");  // orientation, for the 3D demo
static NimBLEUUID GRAVITY_CHAR  ("ef68040a-9b35-4933-9b10-52ffa9740042");
static NimBLEUUID ENV_SERVICE   ("ef680200-9b35-4933-9b10-52ffa9740042");  // climate, air, and light
static NimBLEUUID TEMP_CHAR     ("ef680201-9b35-4933-9b10-52ffa9740042");
static NimBLEUUID PRESSURE_CHAR ("ef680202-9b35-4933-9b10-52ffa9740042");
static NimBLEUUID HUMIDITY_CHAR ("ef680203-9b35-4933-9b10-52ffa9740042");
static NimBLEUUID GAS_CHAR      ("ef680204-9b35-4933-9b10-52ffa9740042");
static NimBLEUUID COLOR_CHAR    ("ef680205-9b35-4933-9b10-52ffa9740042");
static NimBLEUUID BATTERY_SERVICE((uint16_t)0x180F);  // standard Bluetooth battery service
static NimBLEUUID BATTERY_LEVEL  ((uint16_t)0x2A19);

// HID report map: a keyboard (report 1) and media keys (report 2).
static const uint8_t REPORT_MAP[] = {
  0x05, 0x01, 0x09, 0x06, 0xA1, 0x01, 0x85, 0x01,              // Keyboard, report ID 1
  0x05, 0x07, 0x19, 0xE0, 0x29, 0xE7, 0x15, 0x00, 0x25, 0x01,  //   modifier keys
  0x75, 0x01, 0x95, 0x08, 0x81, 0x02,
  0x95, 0x01, 0x75, 0x08, 0x81, 0x01,                          //   reserved byte
  0x95, 0x06, 0x75, 0x08, 0x15, 0x00, 0x25, 0x65,              //   up to 6 keys
  0x05, 0x07, 0x19, 0x00, 0x29, 0x65, 0x81, 0x00,
  0xC0,
  0x05, 0x0C, 0x09, 0x01, 0xA1, 0x01, 0x85, 0x02,              // Consumer control, report ID 2
  0x15, 0x00, 0x26, 0xFF, 0x03, 0x19, 0x00, 0x2A, 0xFF, 0x03,  //   one 16-bit media key
  0x75, 0x10, 0x95, 0x01, 0x81, 0x00,
  0xC0
};

// ---------------- Keyboard side (TV) ----------------
static NimBLEServer*         server     = nullptr;
static NimBLEHIDDevice*      hid        = nullptr;
static NimBLECharacteristic* keyInput   = nullptr;
static NimBLECharacteristic* mediaInput = nullptr;
static volatile bool         tvConnected = false;
static uint32_t              lastKeyAt   = 0;

class TvCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* s, NimBLEConnInfo& info) override {
    tvConnected = true;
    Serial.println("Host connected (TV or computer)");
  }
  void onDisconnect(NimBLEServer* s, NimBLEConnInfo& info, int reason) override {
    tvConnected = false;
    Serial.printf("Host disconnected (reason %d), waiting for it again\n", reason);
    NimBLEDevice::startAdvertising();
  }
  void onAuthenticationComplete(NimBLEConnInfo& info) override {
    Serial.println(info.isBonded() ? "Paired with the host" : "Connected to the host without pairing");
  }
};

static void sendKey(uint8_t key) {
  if (!tvConnected) return;
  uint8_t report[8] = {0, 0, key, 0, 0, 0, 0, 0};
  keyInput->setValue(report, sizeof(report));
  keyInput->notify();
  delay(12);
  memset(report, 0, sizeof(report));  // release
  keyInput->setValue(report, sizeof(report));
  keyInput->notify();
  lastKeyAt = millis();
}

static void sendMedia(uint16_t usage) {
  if (!tvConnected) return;
  uint8_t report[2] = {(uint8_t)(usage & 0xFF), (uint8_t)(usage >> 8)};
  mediaInput->setValue(report, sizeof(report));
  mediaInput->notify();
  delay(12);
  report[0] = report[1] = 0;  // release
  mediaInput->setValue(report, sizeof(report));
  mediaInput->notify();
  lastKeyAt = millis();
}

static const char* keyName(uint8_t key) {
  switch (key) {
    case KEY_LEFT: return "Left";   case KEY_RIGHT: return "Right";
    case KEY_UP:   return "Up";     case KEY_DOWN:  return "Down";
    case KEY_ENTER: return "Enter"; default:        return "Back";
  }
}

// Every key goes to the host, and to the USB stream as an event for the demo page.
static void announce(const char* name) {
  if (STREAM) Serial.printf("{\"t\":\"key\",\"k\":\"%s\",\"sent\":%s}\n", name, tvConnected ? "true" : "false");
  else Serial.printf("Key: %s%s\n", name, tvConnected ? "" : " (not sent, no host connected)");
}

static void press(uint8_t key) {
  announce(keyName(key));
  sendKey(key);
}

static void pressPlayPause() {
  announce("Play/Pause");
  sendMedia(MEDIA_PLAY_PAUSE);
}

// ---------------- Thingy side ----------------
static NimBLEClient*                 thingy      = nullptr;
static const NimBLEAdvertisedDevice* thingyFound = nullptr;
static volatile bool                 doConnect   = false;
static volatile bool                 thingyConnected = false;

// Data from the Thingy arrives on the Bluetooth task; loop() reads it under this lock.
static portMUX_TYPE dataLock = portMUX_INITIALIZER_UNLOCKED;
static float    gravity[3]  = {0, 0, 0};
static uint32_t gravityAt   = 0;
static uint8_t  tapCount    = 0;
static bool     tapPending  = false;
static float    quat[4]     = {1, 0, 0, 0};  // w, x, y, z
static bool     haveQuat    = false;

// Latest climate readings. Each "have" flag turns on once that sensor has reported.
struct Env {
  float tempC = 0, pressureHpa = 0;
  uint8_t humidity = 0, battery = 0;
  uint16_t eco2 = 0, tvoc = 0, rgbc[4] = {0, 0, 0, 0};
  bool haveTemp = false, havePressure = false, haveHumidity = false, haveGas = false, haveColor = false, haveBattery = false;
};
static Env env;

static void onGravity(NimBLERemoteCharacteristic*, uint8_t* data, size_t len, bool) {
  if (len < 12) return;
  float v[3];
  memcpy(v, data, 12);  // three little-endian floats: x, y, z in m/s^2
  portENTER_CRITICAL(&dataLock);
  gravity[0] = v[0]; gravity[1] = v[1]; gravity[2] = v[2];
  gravityAt = millis();
  portEXIT_CRITICAL(&dataLock);
}

static void onQuat(NimBLERemoteCharacteristic*, uint8_t* data, size_t len, bool) {
  if (len < 16) return;
  int32_t v[4];
  memcpy(v, data, 16);  // w, x, y, z as fixed point numbers with 30 fraction bits
  portENTER_CRITICAL(&dataLock);
  for (int i = 0; i < 4; i++) quat[i] = v[i] / 1073741824.0f;
  haveQuat = true;
  portEXIT_CRITICAL(&dataLock);
}

// Thingy environment formats (little-endian):
//   temperature: int8 degrees + uint8 hundredths    pressure: int32 hPa + uint8 hundredths
//   humidity: uint8 %    gas: uint16 eCO2 ppm + uint16 TVOC ppb    color: uint16 red, green, blue, clear
static void onTemp(NimBLERemoteCharacteristic*, uint8_t* d, size_t len, bool) {
  if (len < 2) return;
  portENTER_CRITICAL(&dataLock);
  env.tempC = (int8_t)d[0] + (d[0] & 0x80 ? -d[1] : d[1]) / 100.0f; env.haveTemp = true;
  portEXIT_CRITICAL(&dataLock);
}
static void onPressure(NimBLERemoteCharacteristic*, uint8_t* d, size_t len, bool) {
  if (len < 5) return;
  int32_t whole; memcpy(&whole, d, 4);
  portENTER_CRITICAL(&dataLock);
  env.pressureHpa = whole + d[4] / 100.0f; env.havePressure = true;
  portEXIT_CRITICAL(&dataLock);
}
static void onHumidity(NimBLERemoteCharacteristic*, uint8_t* d, size_t len, bool) {
  if (len < 1) return;
  portENTER_CRITICAL(&dataLock);
  env.humidity = d[0]; env.haveHumidity = true;
  portEXIT_CRITICAL(&dataLock);
}
static void onGas(NimBLERemoteCharacteristic*, uint8_t* d, size_t len, bool) {
  if (len < 4) return;
  portENTER_CRITICAL(&dataLock);
  env.eco2 = d[0] | (d[1] << 8); env.tvoc = d[2] | (d[3] << 8); env.haveGas = true;
  portEXIT_CRITICAL(&dataLock);
}
static void onColor(NimBLERemoteCharacteristic*, uint8_t* d, size_t len, bool) {
  if (len < 8) return;
  portENTER_CRITICAL(&dataLock);
  for (int i = 0; i < 4; i++) env.rgbc[i] = d[2 * i] | (d[2 * i + 1] << 8);
  env.haveColor = true;
  portEXIT_CRITICAL(&dataLock);
}
static void onBattery(NimBLERemoteCharacteristic*, uint8_t* d, size_t len, bool) {
  if (len < 1) return;
  portENTER_CRITICAL(&dataLock);
  env.battery = d[0]; env.haveBattery = true;
  portEXIT_CRITICAL(&dataLock);
}

static void onTap(NimBLERemoteCharacteristic*, uint8_t* data, size_t len, bool) {
  if (len < 2) return;  // byte 0: direction, byte 1: tap count
  portENTER_CRITICAL(&dataLock);
  if (data[1] > tapCount) tapCount = data[1];
  tapPending = true;
  portEXIT_CRITICAL(&dataLock);
}

class ThingyCallbacks : public NimBLEClientCallbacks {
  void onDisconnect(NimBLEClient* c, int reason) override {
    thingyConnected = false;
    portENTER_CRITICAL(&dataLock);
    env = Env();  // don't report old readings after a reconnect
    haveQuat = false;
    portEXIT_CRITICAL(&dataLock);
    Serial.printf("Thingy disconnected (reason %d). Press its button to wake it.\n", reason);
  }
};

// A Thingy advertises its configuration service. A renamed one may not, so also accept any name with "thingy" in it.
static std::string lowerName(const NimBLEAdvertisedDevice* d) {
  std::string name = d->getName();
  for (auto& ch : name) ch = tolower(ch);
  return name;
}
// "ThingyDfu" is a Thingy in firmware-update mode: it has no sensors to read, so leave it alone.
static bool inUpdateMode(const NimBLEAdvertisedDevice* d) { return lowerName(d).find("dfu") != std::string::npos; }
static bool looksLikeThingy(const NimBLEAdvertisedDevice* d) {
  if (inUpdateMode(d)) return false;
  if (d->isAdvertisingService(THINGY_SERVICE)) return true;
  return lowerName(d).find("thingy") != std::string::npos;
}
static uint32_t lastDfuNote = 0;

static volatile bool listing = false;  // the "s" command: print every device seen instead of connecting
static uint32_t      listUntil = 0;

class ScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* d) override {
    const bool thingy = looksLikeThingy(d);
    if (listing) {
      const std::string name = d->getName();
      Serial.printf("  %s  %4d dBm  %s%s\n", d->getAddress().toString().c_str(), d->getRSSI(),
                    name.empty() ? "(no name)" : name.c_str(),
                    thingy ? "   <- Thingy" : (inUpdateMode(d) ? "   <- Thingy in update mode" : ""));
      return;
    }
    if (inUpdateMode(d) && (lastDfuNote == 0 || millis() - lastDfuNote > 30000)) {
      lastDfuNote = millis();
      Serial.println("Found a Thingy in firmware-update mode (ThingyDfu). Restart it, or reinstall its firmware: see docs/TUNING.md");
    }
    if (thingy) {
      Serial.printf("Found Thingy at %s\n", d->getAddress().toString().c_str());
      thingyFound = d;
      doConnect = true;
      NimBLEDevice::getScan()->stop();
    }
  }
};

static void startListing() {
  if (thingyConnected) { Serial.println("The Thingy is already connected"); return; }
  NimBLEDevice::getScan()->stop();
  listing = true;
  listUntil = millis() + 6000;
  Serial.println("Bluetooth devices the bridge can see (6 s):");
  NimBLEDevice::getScan()->start(6000, false);
}

// Neutral ("flat") direction of gravity, learned after connecting or with a triple tap.
static float    neutral[3]  = {0, 0, 1};
static bool     calibrating = false;
static uint32_t calStart    = 0;
static float    calSum[3]   = {0, 0, 0};
static int      calCount    = 0;

static void startCalibration() {
  calibrating = true; calStart = millis();
  calSum[0] = calSum[1] = calSum[2] = 0; calCount = 0;
  Serial.println("Hold the Thingy still the way you like to hold it...");
}

// Climate, air, light, and battery: each one is optional, so a missing sensor never blocks the rest.
static void subscribeOptional(NimBLERemoteService* svc, const NimBLEUUID& uuid, NotifyFn cb, const char* name) {
  NimBLERemoteCharacteristic* c = svc ? svc->getCharacteristic(uuid) : nullptr;
  if (!c || !c->canNotify() || !c->subscribe(true, cb)) { Serial.printf("%s not available\n", name); return; }
  if (c->canRead()) {  // get a first value now instead of waiting for the next update
    NimBLEAttValue v = c->readValue();
    if (v.size()) cb(c, (uint8_t*)v.data(), v.size(), false);
  }
}

static void subscribeEnvironment() {
  NimBLERemoteService* e = thingy->getService(ENV_SERVICE);
  if (!e) { Serial.println("Thingy has no environment service"); }
  else {
    subscribeOptional(e, TEMP_CHAR, onTemp, "Temperature");
    subscribeOptional(e, PRESSURE_CHAR, onPressure, "Pressure");
    subscribeOptional(e, HUMIDITY_CHAR, onHumidity, "Humidity");
    subscribeOptional(e, GAS_CHAR, onGas, "Air quality");
    subscribeOptional(e, COLOR_CHAR, onColor, "Light");
  }
  subscribeOptional(thingy->getService(BATTERY_SERVICE), BATTERY_LEVEL, onBattery, "Battery level");
}

static bool connectThingy() {
  if (!thingy) {
    thingy = NimBLEDevice::createClient();
    thingy->setClientCallbacks(new ThingyCallbacks(), false);
  }
  if (!thingy->connect(thingyFound)) { Serial.println("Could not connect to the Thingy"); return false; }

  NimBLERemoteService* motion = thingy->getService(MOTION_SERVICE);
  if (!motion) { Serial.println("Thingy has no motion service"); thingy->disconnect(); return false; }

  // Motion settings: step counter, temperature and compass intervals (ms), sensor rate (Hz), wake on motion.
  NimBLERemoteCharacteristic* cfg = motion->getCharacteristic(MOTION_CONFIG);
  if (cfg && cfg->canWrite()) {
    const uint16_t stepMs = 1000, tempMs = 10000, compassMs = 1000, rateHz = 30;
    uint8_t c[9] = {
      (uint8_t)(stepMs & 0xFF), (uint8_t)(stepMs >> 8), (uint8_t)(tempMs & 0xFF), (uint8_t)(tempMs >> 8),
      (uint8_t)(compassMs & 0xFF), (uint8_t)(compassMs >> 8), (uint8_t)(rateHz & 0xFF), (uint8_t)(rateHz >> 8), 1};
    if (!cfg->writeValue(c, sizeof(c), true)) Serial.println("Could not set the motion rate, using the Thingy's default");
  }

  NimBLERemoteCharacteristic* g = motion->getCharacteristic(GRAVITY_CHAR);
  if (!g || !g->canNotify() || !g->subscribe(true, onGravity)) {
    Serial.println("Could not read the Thingy's gravity data"); thingy->disconnect(); return false;
  }
  NimBLERemoteCharacteristic* t = motion->getCharacteristic(TAP_CHAR);
  if (!t || !t->canNotify() || !t->subscribe(true, onTap)) Serial.println("Taps not available");
  NimBLERemoteCharacteristic* q = motion->getCharacteristic(QUAT_CHAR);
  if (STREAM && (!q || !q->canNotify() || !q->subscribe(true, onQuat))) Serial.println("Orientation not available, the demo shows tilt only");
  if (STREAM && ENVIRONMENT) subscribeEnvironment();

  Serial.println("Thingy connected");
  startCalibration();
  return true;
}

// ---------------- Turning motion into keys ----------------
static Axis lr = {false, 0, 0, 0}, ud = {false, 0, 0, 0};

static void releaseAxes() { lr.on = false; ud.on = false; }

// One tilt axis: press when tilted past TILT_ON_DEG, repeat while held, release under TILT_OFF_DEG.
static void handleAxis(Axis& a, float deg, uint8_t keyPositive, uint8_t keyNegative) {
  const uint32_t now = millis();
  const float mag = fabsf(deg);
  const int dir = deg > 0 ? 1 : -1;
  if (!a.on) {
    if (mag > TILT_ON_DEG) {
      a.on = true; a.dir = dir; a.since = now; a.last = now;
      press(dir > 0 ? keyPositive : keyNegative);
    }
    return;
  }
  if (mag < TILT_OFF_DEG || dir != a.dir) { a.on = false; return; }
  if (now - a.since > REPEAT_LIMIT_MS) return;  // held too long: stop until it comes back to flat
  float t = (mag - TILT_ON_DEG) / (TILT_FAST_DEG - TILT_ON_DEG);
  t = t < 0 ? 0 : (t > 1 ? 1 : t);
  const uint32_t interval = REPEAT_SLOW_MS - (uint32_t)(t * (REPEAT_SLOW_MS - REPEAT_FAST_MS));
  if (now - a.last >= interval) { a.last = now; press(a.dir > 0 ? keyPositive : keyNegative); }
}

static uint32_t flipStart = 0;
static bool     flipSent  = false;
static uint32_t firstTapAt = 0;
static float    tiltLR = 0, tiltUD = 0;  // latest tilt angles, for the stream

static float asinDeg(float v) { v = v > 1 ? 1 : (v < -1 ? -1 : v); return asinf(v) * 57.29578f; }

static void processMotion() {
  float g[3]; uint32_t at; uint8_t taps; bool newTap;
  portENTER_CRITICAL(&dataLock);
  g[0] = gravity[0]; g[1] = gravity[1]; g[2] = gravity[2]; at = gravityAt;
  taps = tapCount; newTap = tapPending; tapPending = false;
  portEXIT_CRITICAL(&dataLock);

  const uint32_t now = millis();
  const float len = sqrtf(g[0] * g[0] + g[1] * g[1] + g[2] * g[2]);
  if (at == 0 || now - at > 1000 || len < 1.0f) { releaseAxes(); return; }  // no fresh data
  const float u[3] = {g[0] / len, g[1] / len, g[2] / len};

  if (calibrating) {  // average for 0.8 s
    tiltLR = tiltUD = 0;  // report no tilt while learning, never an old value
    calSum[0] += u[0]; calSum[1] += u[1]; calSum[2] += u[2]; calCount++;
    if (now - calStart > 800 && calCount > 3) {
      const float n = sqrtf(calSum[0] * calSum[0] + calSum[1] * calSum[1] + calSum[2] * calSum[2]);
      for (int i = 0; i < 3; i++) neutral[i] = calSum[i] / n;
      calibrating = false;
      Serial.printf("Learned flat: %.2f %.2f %.2f. Ready.\n", neutral[0], neutral[1], neutral[2]);
    }
    return;
  }

  // Taps: wait a moment to tell single, double, and triple taps apart.
  if (newTap && firstTapAt == 0) firstTapAt = now;
  if (firstTapAt && now - firstTapAt > TAP_WINDOW_MS) {
    portENTER_CRITICAL(&dataLock); taps = tapCount; tapCount = 0; portEXIT_CRITICAL(&dataLock);
    firstTapAt = 0;
    if (taps >= 3) { startCalibration(); return; }
    if (taps == 2) pressPlayPause();
    else press(KEY_ENTER);
  }

  // Flip face down and hold: Back.
  const float facing = u[0] * neutral[0] + u[1] * neutral[1] + u[2] * neutral[2];
  if (facing < -0.5f) {
    releaseAxes();
    tiltLR = tiltUD = 0;
    if (!flipStart) flipStart = now;
    if (!flipSent && now - flipStart > FLIP_HOLD_MS) { press(BACK_KEY); flipSent = true; }
    return;
  }
  flipStart = 0;
  if (facing > 0.7f) flipSent = false;

  // Tilt angles relative to "flat".
  float a = asinDeg(u[0]) - asinDeg(neutral[0]);  // left / right
  float b = asinDeg(u[1]) - asinDeg(neutral[1]);  // forward / back
  if (SWAP_AXES) { const float tmp = a; a = b; b = tmp; }
  if (INVERT_LR) a = -a;
  if (INVERT_UD) b = -b;

  tiltLR = a; tiltUD = b;

  // Only the stronger tilt counts, so a diagonal tilt doesn't press two keys.
  if (fabsf(a) >= fabsf(b)) { handleAxis(lr, a, KEY_RIGHT, KEY_LEFT); handleAxis(ud, 0, KEY_UP, KEY_DOWN); }
  else                      { handleAxis(ud, b, KEY_UP, KEY_DOWN);   handleAxis(lr, 0, KEY_RIGHT, KEY_LEFT); }
}

// ---------------- USB stream for motion-demo.html ----------------
static void reportInfo() {
  Serial.printf("{\"t\":\"cfg\",\"on\":%.1f,\"off\":%.1f,\"fast\":%.1f,\"name\":\"%s\",\"version\":\"%s\"}\n",
                TILT_ON_DEG, TILT_OFF_DEG, TILT_FAST_DEG, DEVICE_NAME, FIRMWARE_VERSION);
  Serial.printf("{\"t\":\"state\",\"host\":%s,\"thingy\":%s,\"cal\":%s}\n",
                tvConnected ? "true" : "false", thingyConnected ? "true" : "false", calibrating ? "true" : "false");
}

static uint32_t lastStream = 0, lastState = 0;
static bool     sentHost = false, sentThingy = false, sentCal = false;

static void streamMotion() {
  if (!STREAM) return;
  const uint32_t now = millis();
  const bool host = tvConnected, th = thingyConnected;
  if (host != sentHost || th != sentThingy || calibrating != sentCal || now - lastState > 2000) {
    sentHost = host; sentThingy = th; sentCal = calibrating; lastState = now;
    Serial.printf("{\"t\":\"state\",\"host\":%s,\"thingy\":%s,\"cal\":%s}\n",
                  host ? "true" : "false", th ? "true" : "false", calibrating ? "true" : "false");
  }
  if (!th || now - lastStream < STREAM_MS) return;
  lastStream = now;
  float q[4], g[3]; bool hq;
  portENTER_CRITICAL(&dataLock);
  for (int i = 0; i < 4; i++) q[i] = quat[i];
  for (int i = 0; i < 3; i++) g[i] = gravity[i];
  hq = haveQuat;
  portEXIT_CRITICAL(&dataLock);
  if (hq) Serial.printf("{\"t\":\"m\",\"q\":[%.4f,%.4f,%.4f,%.4f],\"g\":[%.2f,%.2f,%.2f],\"lr\":%.1f,\"ud\":%.1f}\n",
                        q[0], q[1], q[2], q[3], g[0], g[1], g[2], tiltLR, tiltUD);
  else    Serial.printf("{\"t\":\"m\",\"g\":[%.2f,%.2f,%.2f],\"lr\":%.1f,\"ud\":%.1f}\n",
                        g[0], g[1], g[2], tiltLR, tiltUD);
}

// Climate readings, once a second, with only the sensors that have reported so far.
static uint32_t lastEnv = 0;
static void streamEnvironment() {
  if (!STREAM || !ENVIRONMENT || !thingyConnected) return;
  const uint32_t now = millis();
  if (now - lastEnv < ENV_MS) return;
  lastEnv = now;
  Env e;
  portENTER_CRITICAL(&dataLock);
  e = env;
  portEXIT_CRITICAL(&dataLock);
  if (!(e.haveTemp || e.havePressure || e.haveHumidity || e.haveGas || e.haveColor || e.haveBattery)) return;
  char line[256];
  int n = snprintf(line, sizeof(line), "{\"t\":\"env\"");
  if (e.haveTemp)     n += snprintf(line + n, sizeof(line) - n, ",\"temp\":%.2f", e.tempC);
  if (e.haveHumidity) n += snprintf(line + n, sizeof(line) - n, ",\"hum\":%u", e.humidity);
  if (e.havePressure) n += snprintf(line + n, sizeof(line) - n, ",\"press\":%.2f", e.pressureHpa);
  if (e.haveGas)      n += snprintf(line + n, sizeof(line) - n, ",\"eco2\":%u,\"tvoc\":%u", e.eco2, e.tvoc);
  if (e.haveColor)    n += snprintf(line + n, sizeof(line) - n, ",\"rgbc\":[%u,%u,%u,%u]", e.rgbc[0], e.rgbc[1], e.rgbc[2], e.rgbc[3]);
  if (e.haveBattery)  n += snprintf(line + n, sizeof(line) - n, ",\"bat\":%u", e.battery);
  snprintf(line + n, sizeof(line) - n, "}");
  Serial.println(line);
}

// ---------------- Status light ----------------
// Fast blink: waiting for the TV. Slow blink: TV connected, looking for the Thingy. On: ready.
static void updateLed() {
  if (LED_PIN < 0) return;
  const uint32_t now = millis();
  bool on;
  if (!tvConnected) on = (now / 150) % 2;
  else if (!thingyConnected) on = (now / 700) % 2;
  else on = now - lastKeyAt > 80;  // brief flicker on each key
  digitalWrite(LED_PIN, on ? HIGH : LOW);
}

// ---------------- Typing keys in the Serial Monitor (testing) ----------------
static void handleSerial() {
  while (Serial.available()) {
    switch (Serial.read()) {
      case 'l': press(KEY_LEFT); break;   case 'r': press(KEY_RIGHT); break;
      case 'u': press(KEY_UP); break;     case 'd': press(KEY_DOWN); break;
      case 'o': press(KEY_ENTER); break;  case 'b': press(BACK_KEY); break;
      case 'p': pressPlayPause(); break;
      case 'c': startCalibration(); break;
      case 'i': reportInfo(); break;
      case 's': startListing(); break;
      default: break;
    }
  }
}

void setup() {
  Serial.begin(115200);
  if (LED_PIN >= 0) pinMode(LED_PIN, OUTPUT);
  delay(300);
  Serial.printf("\nThingy Bridge %s starting\n", FIRMWARE_VERSION);

  NimBLEDevice::init(DEVICE_NAME);
  NimBLEDevice::setSecurityAuth(true, false, true);  // pair and remember the TV, no PIN needed
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);

  // Keyboard for the TV
  server = NimBLEDevice::createServer();
  server->setCallbacks(new TvCallbacks());
  hid = new NimBLEHIDDevice(server);
  keyInput = hid->getInputReport(1);
  mediaInput = hid->getInputReport(2);
  hid->setManufacturer("DIY");
  hid->setPnp(0x02, 0xE502, 0xA111, 0x0210);
  hid->setHidInfo(0x00, 0x01);
  hid->setReportMap((uint8_t*)REPORT_MAP, sizeof(REPORT_MAP));
  hid->startServices();
  hid->setBatteryLevel(100);

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->setAppearance(APPEARANCE_KEYBOARD);
  adv->addServiceUUID(hid->getHidService()->getUUID());
  adv->setName(DEVICE_NAME);
  adv->enableScanResponse(true);
  adv->start();
  Serial.printf("Pair \"%s\" from the Bluetooth settings of your TV or computer\n", DEVICE_NAME);
  reportInfo();

  // Searching for the Thingy
  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(new ScanCallbacks(), false);
  scan->setActiveScan(true);  // the Thingy sends its name in the scan response
}

void loop() {
  updateLed();
  handleSerial();

  if (doConnect) {
    doConnect = false;
    thingyConnected = connectThingy();
  }
  if (listing && millis() > listUntil) {
    listing = false;
    Serial.println("End of list. If the Thingy isn't in it, it's asleep, out of range, or connected to another device.");
  }
  static uint32_t lastSearchNote = 0;
  if (!thingyConnected && !doConnect && !NimBLEDevice::getScan()->isScanning()) {
    if (!listing && (lastSearchNote == 0 || millis() - lastSearchNote > 30000)) {
      lastSearchNote = millis();
      Serial.println("Looking for the Thingy...");
    }
    NimBLEDevice::getScan()->start(5000, false);  // look for the Thingy for 5 s, then again
  }
  if (thingyConnected) processMotion();
  streamMotion();
  streamEnvironment();
  delay(10);
}
