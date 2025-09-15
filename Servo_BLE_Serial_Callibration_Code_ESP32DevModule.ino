// ESP32 4x SG90 Bluetooth Controller (50 Hz, fixed 4 channels)
// Library: ESP32Servo (Library Manager)
// Board: ESP32 Dev Module
//
// Command format over Bluetooth (newline-terminated):
//   By INDEX (0..3):
//     "0,+45"   -> servo[0] +45° relative
//     "1,=120"  -> servo[1] to 120° absolute
//   By PIN (matches configured pins below):
//     "13,+10"  -> move servo on GPIO 13 by +10°
//
// Batch multiple ops with ';' (all moved in one synchronized glide):
//   "0,=150;1,-20;26,+30"
//
// Notes:
// - Angles are clamped 0..180
// - SG90 pulse window typical: 500..2400 µs
// - Change SERVO_PINS[] to match your wiring.

#include <BluetoothSerial.h>
#include <ESP32Servo.h>

BluetoothSerial SerialBT;

constexpr int SERVO_MIN_US = 500;   // SG90 typical
constexpr int SERVO_MAX_US = 2400;  // SG90 typical
constexpr int NUM_SERVOS   = 4;

// >>>> EDIT THESE PINS TO MATCH YOUR WIRING <<<<
const int SERVO_PINS[NUM_SERVOS] = {13, 14, 26, 27}; // good PWM-capable GPIOs

struct ServoSlot {
  Servo servo;
  int   pin     = -1;
  int   angle   = 90;
  int   channel = -1; // returned by attach()
};

ServoSlot s[NUM_SERVOS];
String rxLine;

// --- Utilities -------------------------------------------------------------

int indexFromPin(int pin) {
  for (int i = 0; i < NUM_SERVOS; ++i) if (SERVO_PINS[i] == pin) return i;
  return -1;
}

// Accept either an index "0..3" or a pin number matching SERVO_PINS[]
// Returns servo index or -1 on error.
int parseTargetId(const String& id) {
  String t = id; t.trim();
  // Try as integer
  bool isNum = true;
  for (size_t i = 0; i < t.length(); ++i) if (!isDigit(t[i]) && !(i == 0 && t[i] == '-')) { isNum = false; break; }
  if (isNum) {
    long v = t.toInt();
    // If it matches a PIN, map to index; else if 0..NUM_SERVOS-1, treat as index.
    int pinIdx = indexFromPin((int)v);
    if (pinIdx >= 0) return pinIdx;
    if (v >= 0 && v < NUM_SERVOS) return (int)v;
  }
  return -1;
}

static inline int clampAngle(long a) {
  if (a < 0)   return 0;
  if (a > 180) return 180;
  return (int)a;
}

// Non-blocking-ish synchronized glide: step all active servos together
void glideMany(const int targets[NUM_SERVOS], int stepDelayMs = 5) {
  bool anyMoving = true;
  while (anyMoving) {
    anyMoving = false;
    for (int i = 0; i < NUM_SERVOS; ++i) {
      int cur = s[i].angle;
      int tgt = targets[i];
      if (cur == tgt) continue;
      anyMoving = true;
      int step = (tgt > cur) ? 1 : -1;
      cur += step;
      s[i].angle = cur;
      s[i].servo.write(cur);
    }
    if (anyMoving) delay(stepDelayMs);
  }
}

// --- Command parsing -------------------------------------------------------
// One op: "<id>,<+/-delta>" or "<id>,=<abs>"
// where <id> is either servo index (0..3) or a configured GPIO pin (e.g., 13).
bool parseOp(const String& op, int& idxOut, int& targetAngleOut, bool& ok) {
  ok = false;
  int comma = op.indexOf(',');
  if (comma < 0) return false;

  String idStr = op.substring(0, comma); idStr.trim();
  String valStr = op.substring(comma + 1); valStr.trim();

  int idx = parseTargetId(idStr);
  if (idx < 0) return false;

  int target = s[idx].angle;
  if (valStr.length() == 0) return false;

  if (valStr[0] == '=') {
    long absA = valStr.substring(1).toInt();
    target = clampAngle(absA);
  } else {
    long delta = valStr.toInt(); // supports +/-N
    target = clampAngle((long)s[idx].angle + delta);
  }

  idxOut = idx;
  targetAngleOut = target;
  ok = true;
  return true;
}

void handleLine(const String& line) {
  // First pass: parse all ops and build per-servo targets
  int targets[NUM_SERVOS];
  for (int i = 0; i < NUM_SERVOS; ++i) targets[i] = s[i].angle;

  int start = 0;
  bool any = false;

  while (start < (int)line.length()) {
    int sep = line.indexOf(';', start);
    String op = (sep < 0) ? line.substring(start) : line.substring(start, sep);
    op.trim();
    if (op.length() > 0) {
      int idx, tgt; bool ok;
      if (!parseOp(op, idx, tgt, ok) || !ok) {
        SerialBT.printf("ERR: Bad op '%s'. Use index 0..%d or pin in list.\n", op.c_str(), NUM_SERVOS-1);
      } else {
        targets[idx] = tgt;
        any = true;
      }
    }
    if (sep < 0) break;
    start = sep + 1;
  }

  if (!any) return;

  // Second pass: synchronized glide to all targets
  glideMany(targets);

  // Report results
  for (int i = 0; i < NUM_SERVOS; ++i) {
    SerialBT.printf("OK idx=%d pin=%d ch=%d angle=%d\n", i, s[i].pin, s[i].channel, s[i].angle);
  }
}

// --- Arduino lifecycle -----------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(150);

  // Bluetooth
  SerialBT.begin("MjakaMwise");
  Serial.println("Bluetooth device started as 'MjakaMwise'");
  SerialBT.println("Ready. Use '<id>,±delta' or '<id>,=abs' ; batch with ';'");
  SerialBT.print("Pins: ");
  for (int i = 0; i < NUM_SERVOS; ++i) {
    SerialBT.printf("%d%s", SERVO_PINS[i], (i == NUM_SERVOS-1 ? "\n" : ", "));
  }

  // Attach 4 servos once at boot (prevents channel exhaustion)
  for (int i = 0; i < NUM_SERVOS; ++i) {
    s[i].pin = SERVO_PINS[i];
    s[i].servo.setPeriodHertz(50); // 50 Hz for SG90
    int ch = s[i].servo.attach(s[i].pin, SERVO_MIN_US, SERVO_MAX_US);
    if (ch == 0) {
      SerialBT.printf("ERR: attach failed on pin %d (no free channel)\n", s[i].pin);
    }
    s[i].channel = ch;
    s[i].angle   = 90;
    s[i].servo.write(s[i].angle);
    delay(150);
  }
}

void loop() {
  while (SerialBT.available()) {
    char c = (char)SerialBT.read();
    if (c == '\n' || c == '\r') {
      if (rxLine.length() > 0) { handleLine(rxLine); rxLine = ""; }
    } else {
      rxLine += c;
      if (rxLine.length() > 200) rxLine = ""; // safety
    }
  }
}
