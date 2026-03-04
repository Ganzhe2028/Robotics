#include <Servo.h>

// ===== Pin Map =====
#define PIN_SENSOR_L2 10
#define PIN_SENSOR_L1 11
#define PIN_SENSOR_R1 9
#define PIN_SENSOR_R2 8

#define PIN_SERVO_LEFT 5
#define PIN_SERVO_RIGHT 4
#define PIN_SERVO_DUMP 7

#define PIN_BTN_SW 6

// ===== Logic Level =====
#define BLACK LOW
#define WHITE HIGH
#define BTN_PRESSED LOW

// ===== Tunables =====
const unsigned long LOOP_DELAY_MS = 10;
const unsigned long LINE_END_CONFIRM_MS = 250;
const unsigned long TURN_FORWARD_MS = 500;
const unsigned long TURN_FIND_LINE_TIMEOUT_MS = 1200;
const unsigned long UTURN_TIMEOUT_MS = 1800;
const unsigned long BTN_DEBOUNCE_MS = 30;

const int DUMP_HOME_ANGLE = 0;
const int DUMP_RELEASE_ANGLE = 50;

Servo leftServo;
Servo rightServo;
Servo dumpServo;

bool straightLineArmed = false;
bool dumpServoAttached = false;

struct Sensors {
  int l2;
  int l1;
  int r1;
  int r2;
};

Sensors readSensors() {
  Sensors s;
  s.l2 = digitalRead(PIN_SENSOR_L2);
  s.l1 = digitalRead(PIN_SENSOR_L1);
  s.r1 = digitalRead(PIN_SENSOR_R1);
  s.r2 = digitalRead(PIN_SENSOR_R2);
  return s;
}

bool isAllWhite(const Sensors &s) {
  return s.l2 == WHITE && s.l1 == WHITE && s.r1 == WHITE && s.r2 == WHITE;
}

bool isCenterLine(const Sensors &s) {
  return s.l2 == WHITE && s.l1 == BLACK && s.r1 == BLACK && s.r2 == WHITE; // 1001
}

bool isLeftTurnSignal(const Sensors &s) {
  return s.l2 == BLACK && s.r2 == WHITE;
}

bool isRightTurnSignal(const Sensors &s) {
  return s.l2 == WHITE && s.r2 == BLACK;
}

void stopMoving() {
  leftServo.write(90);
  rightServo.write(90);
}

void moveForward() {
  leftServo.write(180);
  rightServo.write(0);
}

void turnLeftSoft() {
  leftServo.write(90);
  rightServo.write(0);
}

void turnRightSoft() {
  leftServo.write(180);
  rightServo.write(90);
}

void turnLeftHard() {
  leftServo.write(0);
  rightServo.write(0);
}

void turnRightHard() {
  leftServo.write(180);
  rightServo.write(180);
}

void rotateUntilLineFound(bool turnLeft) {
  unsigned long startTime = millis();
  while (millis() - startTime < TURN_FIND_LINE_TIMEOUT_MS) {
    if (turnLeft) {
      turnLeftHard();
    } else {
      turnRightHard();
    }

    Sensors s = readSensors();
    if (s.l1 == BLACK || s.r1 == BLACK) {
      Serial.println(F("[TURN] line found, exit rotate"));
      break;
    }
    delay(1);
  }
}

void handleRightAngleTurn(bool turnLeft) {
  Serial.println(turnLeft ? F("[TURN] left signal") : F("[TURN] right signal"));
  moveForward();
  delay(TURN_FORWARD_MS);

  Sensors s = readSensors();
  if (isAllWhite(s)) {
    Serial.println(F("[TURN] crossed corner, rotate to find line"));
    rotateUntilLineFound(turnLeft);
  }
}

bool confirmLineEnd() {
  moveForward();
  delay(LINE_END_CONFIRM_MS);
  Sensors s = readSensors();
  return isAllWhite(s);
}

void attachDumpServoIfNeeded() {
  if (!dumpServoAttached) {
    dumpServo.attach(PIN_SERVO_DUMP);
    dumpServoAttached = true;
    Serial.println(F("[DUMP] attach top servo"));
    dumpServo.write(DUMP_HOME_ANGLE);
    delay(120);
  }
}

void unloadAndUTurn() {
  Serial.println(F("[DUMP] line end confirmed"));
  stopMoving();

  attachDumpServoIfNeeded();

  for (int pos = DUMP_HOME_ANGLE; pos <= DUMP_RELEASE_ANGLE; pos++) {
    dumpServo.write(pos);
    delay(10);
  }

  delay(500);

  for (int pos = DUMP_RELEASE_ANGLE; pos >= DUMP_HOME_ANGLE; pos--) {
    dumpServo.write(pos);
    delay(10);
  }

  delay(250);

  // Release PWM after unload to reduce interference/jitter.
  dumpServo.detach();
  dumpServoAttached = false;
  Serial.println(F("[DUMP] detach top servo"));

  turnLeftHard();
  unsigned long startTime = millis();
  while (millis() - startTime < UTURN_TIMEOUT_MS) {
    Sensors s = readSensors();
    if (s.l1 == BLACK || s.r1 == BLACK) {
      Serial.println(F("[UTURN] line found, exit"));
      break;
    }
    delay(1);
  }
}

bool isPressedStable(unsigned long stableMs) {
  if (digitalRead(PIN_BTN_SW) != BTN_PRESSED) {
    return false;
  }
  unsigned long start = millis();
  while (millis() - start < stableMs) {
    if (digitalRead(PIN_BTN_SW) != BTN_PRESSED) {
      return false;
    }
    delay(1);
  }
  return true;
}

void waitForStartButton() {
  Serial.println(F("[BOOT] waiting button release..."));
  while (digitalRead(PIN_BTN_SW) == BTN_PRESSED) {
    delay(5);
  }

  Serial.println(F("[BOOT] waiting button press..."));
  while (!isPressedStable(BTN_DEBOUNCE_MS)) {
    delay(5);
  }

  while (digitalRead(PIN_BTN_SW) == BTN_PRESSED) {
    delay(5);
  }

  Serial.println(F("[BOOT] button confirmed, start"));
}

void setup() {
  Serial.begin(9600);
  Serial.println(F("[BOOT] init"));

  pinMode(PIN_SENSOR_L2, INPUT);
  pinMode(PIN_SENSOR_L1, INPUT);
  pinMode(PIN_SENSOR_R1, INPUT);
  pinMode(PIN_SENSOR_R2, INPUT);

  pinMode(PIN_BTN_SW, INPUT_PULLUP);

  leftServo.attach(PIN_SERVO_LEFT);
  rightServo.attach(PIN_SERVO_RIGHT);
  stopMoving();

  // Keep dump servo detached on boot to avoid startup jump to 90 deg.
  dumpServo.detach();
  dumpServoAttached = false;
  Serial.println(F("[BOOT] dump servo detached"));

  waitForStartButton();
}

void loop() {
  Sensors s = readSensors();

  // Debug print: sensors + armed state for first-run issue tracing.
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 80) {
    lastPrint = millis();
    Serial.print(F("[S] "));
    Serial.print(s.l2); Serial.print(' ');
    Serial.print(s.l1); Serial.print(' ');
    Serial.print(s.r1); Serial.print(' ');
    Serial.print(s.r2); Serial.print(F(" | armed="));
    Serial.println(straightLineArmed ? 1 : 0);
  }

  if (isCenterLine(s)) {
    straightLineArmed = true;
  }

  // Split after center-line state: turn or line-end unload.
  if (straightLineArmed && isLeftTurnSignal(s)) {
    straightLineArmed = false;
    handleRightAngleTurn(true);
    delay(LOOP_DELAY_MS);
    return;
  }
  if (straightLineArmed && isRightTurnSignal(s)) {
    straightLineArmed = false;
    handleRightAngleTurn(false);
    delay(LOOP_DELAY_MS);
    return;
  }
  if (straightLineArmed && isAllWhite(s) && confirmLineEnd()) {
    straightLineArmed = false;
    unloadAndUTurn();
    delay(LOOP_DELAY_MS);
    return;
  }

  // Normal line tracking.
  if (isLeftTurnSignal(s)) {
    handleRightAngleTurn(true);
  } else if (isRightTurnSignal(s)) {
    handleRightAngleTurn(false);
  } else if (s.l2 == WHITE && s.r2 == WHITE) {
    if (s.l1 == BLACK && s.r1 == BLACK) {
      moveForward();
    } else if (s.l1 == BLACK && s.r1 == WHITE) {
      turnLeftSoft();
    } else if (s.l1 == WHITE && s.r1 == BLACK) {
      turnRightSoft();
    } else if (isAllWhite(s)) {
      moveForward();
    } else {
      stopMoving();
    }
  } else {
    stopMoving();
  }

  delay(LOOP_DELAY_MS);
}
