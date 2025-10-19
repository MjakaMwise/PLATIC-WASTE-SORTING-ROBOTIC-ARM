// ESP32 3-DOF Arm + Gripper with Inverse Kinematics
// Library: ESP32Servo (Library Manager)
// Board: ESP32 Dev Module
//
// Command formats:
//   DIRECT CONTROL (original):
//     "0,+45"   -> servo[0] +45° relative
//     "1,=120"  -> servo[1] to 120° absolute
//     "0,=150;1,-20;2,+30" -> batch operations
//
//   INVERSE KINEMATICS (new):
//     "IK,X=10,Y=5,Z=8"     -> Move end effector to (10,5,8) in cm
//     "IK,X=10,Y=5,Z=8,G=45" -> Same + set gripper to 45°
//
//   UTILITIES:
//     "HOME"    -> Move to home position (all 90°)
//     "STATUS"  -> Print current angles and position
//
// Coordinate system:
//   Origin at base center
//   X: forward, Y: left, Z: up
//   All distances in cm

#include <BluetoothSerial.h>
#include <ESP32Servo.h>
#include <math.h>

BluetoothSerial SerialBT;

constexpr int SERVO_MIN_US = 500;
constexpr int SERVO_MAX_US = 2400;
constexpr int NUM_SERVOS   = 4;

// >>>> EDIT THESE PINS TO MATCH YOUR WIRING <<<<
const int SERVO_PINS[NUM_SERVOS] = {13, 14, 26, 27}; // Base, Shoulder, Elbow, Gripper

// Arm dimensions (in cm)
constexpr float BASE_HEIGHT = 5.0;   // Height from base to shoulder
constexpr float UPPER_ARM   = 5.0;   // Shoulder to elbow
constexpr float FOREARM     = 5.0;   // Elbow to end effector

struct ServoSlot {
  Servo servo;
  int   pin     = -1;
  int   angle   = 90;
  int   channel = -1;
};

ServoSlot s[NUM_SERVOS];
String rxLine;

// --- Math Utilities --------------------------------------------------------

float radToDeg(float rad) { return rad * 180.0 / PI; }
float degToRad(float deg) { return deg * PI / 180.0; }

static inline int clampAngle(long a) {
  if (a < 0)   return 0;
  if (a > 180) return 180;
  return (int)a;
}

// --- Forward Kinematics ----------------------------------------------------
// Calculate end effector position from current servo angles
void forwardKinematics(float& x, float& y, float& z) {
  float baseAngle = degToRad(s[0].angle);      // Base rotation
  float shoulderAngle = degToRad(s[1].angle);  // Shoulder pitch
  float elbowAngle = degToRad(s[2].angle);     // Elbow pitch
  
  // Calculate in 2D plane first (ignoring base rotation)
  float shoulderZ = BASE_HEIGHT;
  float elbowZ = shoulderZ + UPPER_ARM * sin(shoulderAngle - PI/2);
  float elbowR = UPPER_ARM * cos(shoulderAngle - PI/2);
  
  float endZ = elbowZ + FOREARM * sin(shoulderAngle + elbowAngle - PI);
  float endR = elbowR + FOREARM * cos(shoulderAngle + elbowAngle - PI);
  
  // Apply base rotation to get 3D coordinates
  x = endR * cos(baseAngle);
  y = endR * sin(baseAngle);
  z = endZ;
}

// --- Inverse Kinematics ----------------------------------------------------
// Calculate servo angles needed to reach target position
// Returns true if solution found, false if unreachable
bool inverseKinematics(float targetX, float targetY, float targetZ, 
                       int& baseAngle, int& shoulderAngle, int& elbowAngle) {
  
  // Step 1: Calculate base angle (rotation around Z-axis)
  float baseRad = atan2(targetY, targetX);
  baseAngle = clampAngle((long)radToDeg(baseRad));
  
  // Step 2: Work in 2D plane (cylindrical coordinates)
  float targetR = sqrt(targetX * targetX + targetY * targetY); // Horizontal distance
  float targetH = targetZ - BASE_HEIGHT;  // Height relative to shoulder
  
  // Distance from shoulder to target
  float distance = sqrt(targetR * targetR + targetH * targetH);
  
  // Check if target is reachable
  float maxReach = UPPER_ARM + FOREARM;
  float minReach = fabs(UPPER_ARM - FOREARM);
  
  if (distance > maxReach || distance < minReach) {
    SerialBT.printf("IK: Target unreachable. Distance=%.2fcm (min=%.2f, max=%.2f)\n", 
                    distance, minReach, maxReach);
    return false;
  }
  
  // Step 3: Calculate elbow angle using law of cosines
  // cos(elbow) = (upper² + forearm² - distance²) / (2 * upper * forearm)
  float cosElbow = (UPPER_ARM * UPPER_ARM + FOREARM * FOREARM - distance * distance) 
                   / (2.0 * UPPER_ARM * FOREARM);
  cosElbow = constrain(cosElbow, -1.0, 1.0);
  
  float elbowRad = acos(cosElbow);
  // For "elbow down" configuration (more natural), use PI - elbowRad
  elbowRad = PI - elbowRad;
  
  // Step 4: Calculate shoulder angle
  float alpha = atan2(targetH, targetR);  // Angle to target from shoulder
  float beta = acos((UPPER_ARM * UPPER_ARM + distance * distance - FOREARM * FOREARM) 
                    / (2.0 * UPPER_ARM * distance));
  
  float shoulderRad = alpha + beta;
  
  // Convert to servo angles (90° = horizontal)
  shoulderAngle = clampAngle((long)radToDeg(shoulderRad + PI/2));
  elbowAngle = clampAngle((long)radToDeg(elbowRad));
  
  return true;
}

// --- Servo Control ---------------------------------------------------------

int indexFromPin(int pin) {
  for (int i = 0; i < NUM_SERVOS; ++i) if (SERVO_PINS[i] == pin) return i;
  return -1;
}

int parseTargetId(const String& id) {
  String t = id; t.trim();
  bool isNum = true;
  for (size_t i = 0; i < t.length(); ++i) 
    if (!isDigit(t[i]) && !(i == 0 && t[i] == '-')) { isNum = false; break; }
  
  if (isNum) {
    long v = t.toInt();
    int pinIdx = indexFromPin((int)v);
    if (pinIdx >= 0) return pinIdx;
    if (v >= 0 && v < NUM_SERVOS) return (int)v;
  }
  return -1;
}

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

// --- Command Parsing -------------------------------------------------------

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
    long delta = valStr.toInt();
    target = clampAngle((long)s[idx].angle + delta);
  }

  idxOut = idx;
  targetAngleOut = target;
  ok = true;
  return true;
}

// Parse IK command: "IK,X=10,Y=5,Z=8" or "IK,X=10,Y=5,Z=8,G=45"
bool parseIKCommand(const String& cmd, float& x, float& y, float& z, int& gripperAngle, bool& hasGripper) {
  x = y = z = 0;
  gripperAngle = -1;
  hasGripper = false;
  
  int start = 3; // Skip "IK,"
  while (start < (int)cmd.length()) {
    int comma = cmd.indexOf(',', start);
    String pair = (comma < 0) ? cmd.substring(start) : cmd.substring(start, comma);
    pair.trim();
    
    int eq = pair.indexOf('=');
    if (eq > 0) {
      String key = pair.substring(0, eq);
      String val = pair.substring(eq + 1);
      key.trim(); key.toUpperCase();
      val.trim();
      
      if (key == "X") x = val.toFloat();
      else if (key == "Y") y = val.toFloat();
      else if (key == "Z") z = val.toFloat();
      else if (key == "G") {
        gripperAngle = clampAngle(val.toInt());
        hasGripper = true;
      }
    }
    
    if (comma < 0) break;
    start = comma + 1;
  }
  
  return true;
}

void handleLine(const String& line) {
  String cmd = line;
  cmd.trim();
  cmd.toUpperCase();
  
  // HOME command
  if (cmd == "HOME") {
    int targets[NUM_SERVOS] = {90, 90, 90, 90};
    glideMany(targets);
    SerialBT.println("OK: Moved to HOME position");
    for (int i = 0; i < NUM_SERVOS; ++i) {
      SerialBT.printf("  Servo %d: %d°\n", i, s[i].angle);
    }
    return;
  }
  
  // STATUS command
  if (cmd == "STATUS") {
    float x, y, z;
    forwardKinematics(x, y, z);
    SerialBT.println("=== ARM STATUS ===");
    SerialBT.printf("Servo 0 (Base):     %d°\n", s[0].angle);
    SerialBT.printf("Servo 1 (Shoulder): %d°\n", s[1].angle);
    SerialBT.printf("Servo 2 (Elbow):    %d°\n", s[2].angle);
    SerialBT.printf("Servo 3 (Gripper):  %d°\n", s[3].angle);
    SerialBT.printf("Position: X=%.2f Y=%.2f Z=%.2f cm\n", x, y, z);
    return;
  }
  
  // IK command
  if (cmd.startsWith("IK,")) {
    float x, y, z;
    int gripperAngle;
    bool hasGripper;
    
    if (!parseIKCommand(line, x, y, z, gripperAngle, hasGripper)) {
      SerialBT.println("ERR: IK parse failed. Use: IK,X=10,Y=5,Z=8[,G=45]");
      return;
    }
    
    int baseA, shoulderA, elbowA;
    if (inverseKinematics(x, y, z, baseA, shoulderA, elbowA)) {
      int targets[NUM_SERVOS];
      targets[0] = baseA;
      targets[1] = shoulderA;
      targets[2] = elbowA;
      targets[3] = hasGripper ? gripperAngle : s[3].angle;
      
      SerialBT.printf("IK: Target (%.2f, %.2f, %.2f) -> Base=%d° Shoulder=%d° Elbow=%d°\n",
                      x, y, z, baseA, shoulderA, elbowA);
      
      glideMany(targets);
      
      // Verify with forward kinematics
      float actualX, actualY, actualZ;
      forwardKinematics(actualX, actualY, actualZ);
      SerialBT.printf("OK: Reached (%.2f, %.2f, %.2f)\n", actualX, actualY, actualZ);
    }
    return;
  }
  
  // Direct servo control (original functionality)
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
        SerialBT.printf("ERR: Bad op '%s'. Use index 0..%d or pin.\n", op.c_str(), NUM_SERVOS-1);
      } else {
        targets[idx] = tgt;
        any = true;
      }
    }
    if (sep < 0) break;
    start = sep + 1;
  }

  if (!any) return;

  glideMany(targets);

  for (int i = 0; i < NUM_SERVOS; ++i) {
    SerialBT.printf("OK idx=%d angle=%d\n", i, s[i].angle);
  }
}

// --- Arduino Lifecycle -----------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(150);

  SerialBT.begin("MjakaMwise");
  Serial.println("Bluetooth started as 'MjakaMwise'");
  SerialBT.println("=== 3-DOF ARM WITH IK ===");
  SerialBT.println("Commands:");
  SerialBT.println("  IK,X=10,Y=5,Z=8[,G=45]  - Inverse kinematics");
  SerialBT.println("  0,=120 or 1,+45         - Direct servo control");
  SerialBT.println("  HOME                    - Move to home position");
  SerialBT.println("  STATUS                  - Show current state");
  SerialBT.print("Pins: ");
  for (int i = 0; i < NUM_SERVOS; ++i) {
    SerialBT.printf("%d%s", SERVO_PINS[i], (i == NUM_SERVOS-1 ? "\n" : ", "));
  }

  for (int i = 0; i < NUM_SERVOS; ++i) {
    s[i].pin = SERVO_PINS[i];
    s[i].servo.setPeriodHertz(50);
    int ch = s[i].servo.attach(s[i].pin, SERVO_MIN_US, SERVO_MAX_US);
    s[i].channel = ch;
    s[i].angle   = 90;
    s[i].servo.write(s[i].angle);
    delay(150);
  }
  
  SerialBT.println("Ready!");
}

void loop() {
  while (SerialBT.available()) {
    char c = (char)SerialBT.read();
    if (c == '\n' || c == '\r') {
      if (rxLine.length() > 0) { handleLine(rxLine); rxLine = ""; }
    } else {
      rxLine += c;
      if (rxLine.length() > 200) rxLine = "";
    }
  }
}
