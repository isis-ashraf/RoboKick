#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define SERVOMIN   100
#define SERVOMAX   560
#define SERVO_FREQ  50

#define L_HIP    0
#define L_KNEE   1
#define L_ANKLE  2
#define R_HIP    3
#define R_KNEE   4
#define R_ANKLE  5
#define NUM_SERVOS 6

float neutralPos[NUM_SERVOS] = { 90, 50, 35, 90, 50, 85 };

float currentPos[NUM_SERVOS];
float targetPos[NUM_SERVOS];

const float STEP_DEG   = 2.0;
const int   LOOP_DELAY = 10;

String serialBuffer  = "";
bool   walkedOnce    = false;
bool   kickInProgress = false;

uint16_t degreesToPulse(float deg) {
  deg = constrain(deg, 0, 180);
  return map((long)deg, 0, 180, SERVOMIN, SERVOMAX);
}

void setServo(uint8_t ch, float deg) {
  deg = constrain(deg, 0, 180);
  pwm.setPWM(ch, 0, degreesToPulse(deg));
  currentPos[ch] = deg;
}

float applyIKOffset(uint8_t ch, float ikAngle) {
  return neutralPos[ch] + ikAngle;
}

bool allServosAtTarget() {
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (abs(currentPos[i] - targetPos[i]) > 0.5) return false;
  }
  return true;
}

// Interpolation
void stepServos() {
  for (int i = 0; i < NUM_SERVOS; i++) {
    float diff = targetPos[i] - currentPos[i];
    if (abs(diff) < 0.5) {
      if (abs(currentPos[i] - targetPos[i]) > 0.01) {
        setServo(i, targetPos[i]);
      }
    } else {
      float step = (diff > 0) ? STEP_DEG : -STEP_DEG;
      setServo(i, currentPos[i] + step);
    }
  }
}

// Angles from ROS
void applyAngles(float angles[NUM_SERVOS]) {
  for (int i = 0; i < NUM_SERVOS; i++) {
    targetPos[i] = constrain(applyIKOffset(i, angles[i]), 0, 180);
  }
}

void goNeutral() {
  for (int i = 0; i < NUM_SERVOS; i++) {
    targetPos[i] = neutralPos[i];
  }
}

// Walking sequence 
void simpleWalk() {
  unsigned long start = millis();

  while (millis() - start < 3000) {
    targetPos[L_HIP]  = 110; targetPos[L_KNEE] = 60;
    targetPos[R_HIP]  = 70;  targetPos[R_KNEE] = 25;

    unsigned long t = millis();
    while (millis() - t < 400) { stepServos(); delay(LOOP_DELAY); }

    targetPos[L_HIP]  = 70;  targetPos[L_KNEE] = 20;
    targetPos[R_HIP]  = 110; targetPos[R_KNEE] = 65;

    t = millis();
    while (millis() - t < 400) { stepServos(); delay(LOOP_DELAY); }
  }

  goNeutral();
  for (int i = 0; i < NUM_SERVOS; i++) setServo(i, neutralPos[i]);
}

// Parsing 
void parseCommand(String cmd) {
  cmd.trim();

  if (cmd == "NEUTRAL") {
    goNeutral();
    kickInProgress = false;
    Serial.println("OK NEUTRAL");
    return;
  }

  if (cmd == "STATUS") {
    Serial.print("POS:");
    for (int i = 0; i < NUM_SERVOS; i++) {
      Serial.print(currentPos[i]);
      if (i < NUM_SERVOS - 1) Serial.print(",");
    }
    Serial.println();
    return;
  }

  if (cmd.startsWith("ANGLES:")) {
    String data = cmd.substring(7);
    float  angles[NUM_SERVOS];
    int    idx = 0, start = 0;

    for (int i = 0; i <= (int)data.length() && idx < NUM_SERVOS; i++) {
      if (i == (int)data.length() || data[i] == ',') {
        angles[idx++] = data.substring(start, i).toFloat();
        start = i + 1;
      }
    }

    if (idx == NUM_SERVOS) {
      applyAngles(angles);
      kickInProgress = true;
      Serial.println("OK");
    } else {
      Serial.println("ERR: need 6 angles");
    }
    return;
  }

  if (cmd.startsWith("SET:")) {
    String data  = cmd.substring(4);
    int    comma = data.indexOf(',');
    if (comma > 0) {
      int   ch  = data.substring(0, comma).toInt();
      float deg = data.substring(comma + 1).toFloat();
      if (ch >= 0 && ch < NUM_SERVOS) {
        targetPos[ch] = constrain(deg, 0, 180);
        Serial.printf("SET:ch%d=%.1f\n", ch, deg);
      }
    }
    return;
  }

  if (cmd.length() > 0) {
    Serial.println("ERR: unknown command");
  }
}

// Setup
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(200);

  for (int i = 0; i < NUM_SERVOS; i++) {
    currentPos[i] = neutralPos[i];
    targetPos[i]  = neutralPos[i];
    setServo(i, neutralPos[i]);
  }
  delay(500);

  Serial.println("ESP32 READY: ANGLES | NEUTRAL | STATUS | SET");
}

// Loop
void loop() {
  if (!walkedOnce) {
    simpleWalk();
    walkedOnce = true;

    goNeutral();
    unsigned long t = millis();
    while (millis() - t < 2000) { stepServos(); delay(LOOP_DELAY); }
  }

  // Serial Input
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      if (serialBuffer.length() > 0) {
        parseCommand(serialBuffer);
        serialBuffer = "";
      }
    } else {
      serialBuffer += c;
    }
  }

  stepServos();

  if (kickInProgress && allServosAtTarget()) {
    delay(2000);
    goNeutral();
    kickInProgress = false;
    Serial.println("KICK DONE: returning to neutral");
  }

  delay(LOOP_DELAY);
}