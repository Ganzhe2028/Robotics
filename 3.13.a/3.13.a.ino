#include <Servo.h>

// ===== Pin Map =====
#define PIN_SENSOR_L2 10
#define PIN_SENSOR_L1 11
#define PIN_SENSOR_R1 9
#define PIN_SENSOR_R2 8

#define PIN_SERVO_LEFT 5
#define PIN_SERVO_RIGHT 4
#define PIN_SERVO_DUMP 7

#define PIN_BTN_START 6

// ===== Logic Level =====
#define BTN_RELEASED HIGH
#define BLACK HIGH
#define WHITE LOW
#define BTN_PRESSED LOW

// ===== Tunables =====
const unsigned long LOOP_DELAY_MS = 10;
const unsigned long BUTTON_DEBOUNCE_MS = 30;
const unsigned long START_HOLD_MS = 700;
const unsigned long TARGET_SELECTION_RESET_MS = 1500;
const unsigned long SENSOR_EVENT_BLOCK_MS = 100;
const unsigned long END_CONFIRM_MS = 300;
const unsigned long STATUS_PRINT_MS = 800;
const unsigned long TURN_FORWARD_MS = 500;
const unsigned long SEEK_LINE_LOG_MS = 1000;
const unsigned long DROP_PAYLOAD_MS = 500;
const unsigned long DROP_RETURN_SETTLE_MS = 250;
const unsigned long DROP_EXIT_MS = 150;
const unsigned long STUCK_LOW_WARN_MS = 1500;

const int DUMP_HOME_ANGLE = 0;
const int DUMP_RELEASE_ANGLE = 80;
const int MAX_TARGET_STATION = 3;

enum RobotState {
  STATE_SETTING_TARGET,
  STATE_RUNNING,
  STATE_DROPPING,
  STATE_TURNING
};

enum GapMode {
  MODE_ADD,
  MODE_SUB
};

struct Sensors {
  int l2;
  int l1;
  int r1;
  int r2;
};

struct ButtonState {
  uint8_t pin;
  const char *name;
  bool stableLevel;
  bool lastReading;
  bool pressedArmed;
  bool longPressHandled;
  bool stuckLowWarned;
  unsigned long stableSinceTime;
  unsigned long lastChangeTime;
};

enum ButtonEvent {
  BUTTON_EVENT_NONE,
  BUTTON_EVENT_SHORT_RELEASE,
  BUTTON_EVENT_LONG_PRESS
};

Servo leftServo;
Servo rightServo;
Servo dumpServo;

ButtonState startButton = {
  PIN_BTN_START, "START", BTN_RELEASED, BTN_RELEASED, false, false, false, 0, 0
};

RobotState robotState = STATE_SETTING_TARGET;
GapMode mode = MODE_ADD;

int targetStation = 0;
int stationNum = 0;
int boxNum = 0;
unsigned long lastTargetSelectionTime = 0;

bool straightLineArmed = false;
bool dumpServoAttached = false;
bool stationLatched = false;
bool gapLatched = false;
bool isTurning = false;
bool isDropping = false;

int readNormalizedSensor(uint8_t pin) {
  int rawValue = digitalRead(pin);
  return rawValue == HIGH ? WHITE : BLACK;
}

Sensors readSensors() {
  Sensors s;
  s.l2 = readNormalizedSensor(PIN_SENSOR_L2);
  s.l1 = readNormalizedSensor(PIN_SENSOR_L1);
  s.r1 = readNormalizedSensor(PIN_SENSOR_R1);
  s.r2 = readNormalizedSensor(PIN_SENSOR_R2);
  return s;
}

bool isAllWhite(const Sensors &s) {
  return s.l2 == WHITE && s.l1 == WHITE && s.r1 == WHITE && s.r2 == WHITE;
}

bool isAllBlack(const Sensors &s) {
  return s.l2 == BLACK && s.l1 == BLACK && s.r1 == BLACK && s.r2 == BLACK;
}

bool isCenterLine(const Sensors &s) {
  return s.l2 == WHITE && s.l1 == BLACK && s.r1 == BLACK && s.r2 == WHITE;
}

bool isGapMark(const Sensors &s) {
  return isAllWhite(s);
}

bool isStationMark(const Sensors &s) {
  return isAllBlack(s);
}

bool isLeftTurnSignal(const Sensors &s) {
  return s.l2 == BLACK && s.r2 == WHITE && !isStationMark(s);
}

bool isRightTurnSignal(const Sensors &s) {
  return s.l2 == WHITE && s.r2 == BLACK && !isStationMark(s);
}

const char *stateName(RobotState state) {
  switch (state) {
    case STATE_SETTING_TARGET:
      return "SETTING_TARGET";
    case STATE_RUNNING:
      return "RUNNING";
    case STATE_DROPPING:
      return "DROPPING";
    case STATE_TURNING:
      return "TURNING";
    default:
      return "UNKNOWN";
  }
}

const char *modeName(GapMode currentMode) {
  return currentMode == MODE_ADD ? "ADD" : "SUB";
}

const char *levelName(bool level) {
  return level == BTN_PRESSED ? "LOW/PRESSED" : "HIGH/RELEASED";
}

char stationLabel(int station) {
  if (station < 1 || station > MAX_TARGET_STATION) {
    return '-';
  }

  return 'A' + station - 1;
}

void printPrefix() {
  Serial.print('[');
  Serial.print(millis());
  Serial.print(F(" ms] "));
}

void setupButton(ButtonState &button) {
  pinMode(button.pin, INPUT_PULLUP);
  button.stableLevel = digitalRead(button.pin);
  button.lastReading = button.stableLevel;
  button.pressedArmed = false;
  button.longPressHandled = false;
  button.stuckLowWarned = false;
  button.stableSinceTime = millis();
  button.lastChangeTime = millis();
}

void printButtonBootState(const ButtonState &button) {
  Serial.print(F("[BOOT] "));
  Serial.print(button.name);
  Serial.print(F(" initial state = "));
  Serial.println(levelName(button.stableLevel));
}

void printSensorPattern(const Sensors &s) {
  Serial.print(s.l2);
  Serial.print(s.l1);
  Serial.print(s.r1);
  Serial.print(s.r2);
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

void attachDumpServoIfNeeded() {
  if (dumpServoAttached) {
    return;
  }

  dumpServo.attach(PIN_SERVO_DUMP);
  dumpServoAttached = true;
  dumpServo.write(DUMP_HOME_ANGLE);
  delay(120);
}

void detachDumpServoIfNeeded() {
  if (!dumpServoAttached) {
    return;
  }

  dumpServo.detach();
  dumpServoAttached = false;
}

void resetTripState() {
  stationNum = 0;
  boxNum = 0;
  mode = MODE_ADD;
  straightLineArmed = false;
  stationLatched = false;
  gapLatched = false;
}

ButtonEvent pollButtonEvent(ButtonState &button, unsigned long longPressMs) {
  bool reading = digitalRead(button.pin);

  if (reading != button.lastReading) {
    button.lastReading = reading;
    button.lastChangeTime = millis();
  }

  if ((millis() - button.lastChangeTime) < BUTTON_DEBOUNCE_MS) {
    return BUTTON_EVENT_NONE;
  }

  if (reading != button.stableLevel) {
    button.stableLevel = reading;
    button.stableSinceTime = millis();
    button.stuckLowWarned = false;

    if (button.stableLevel == BTN_PRESSED) {
      button.pressedArmed = true;
      button.longPressHandled = false;
      return BUTTON_EVENT_NONE;
    }

    if (button.pressedArmed && !button.longPressHandled) {
      button.pressedArmed = false;
      return BUTTON_EVENT_SHORT_RELEASE;
    }

    button.pressedArmed = false;
    button.longPressHandled = false;
    return BUTTON_EVENT_NONE;
  }

  if (button.stableLevel != BTN_PRESSED) {
    return BUTTON_EVENT_NONE;
  }

  if (!button.pressedArmed || button.longPressHandled) {
    return BUTTON_EVENT_NONE;
  }

  if (millis() - button.stableSinceTime < longPressMs) {
    return BUTTON_EVENT_NONE;
  }

  button.longPressHandled = true;
  button.pressedArmed = false;
  return BUTTON_EVENT_LONG_PRESS;
}

void warnIfButtonStuck(ButtonState &button) {
  if (button.stableLevel != BTN_PRESSED) {
    return;
  }

  if (button.pressedArmed) {
    return;
  }

  if (button.longPressHandled) {
    return;
  }

  if (button.stuckLowWarned) {
    return;
  }

  if (millis() - button.stableSinceTime < STUCK_LOW_WARN_MS) {
    return;
  }

  button.stuckLowWarned = true;

  printPrefix();
  Serial.print(F("[WARN] "));
  Serial.print(button.name);
  Serial.println(F(" held LOW without an edge; check wiring, switch, or pin choice"));
}

void rotateUntilLineFound(bool turnLeft) {
  unsigned long lastLogTime = millis();

  while (true) {
    if (turnLeft) {
      turnLeftHard();
    } else {
      turnRightHard();
    }

    Sensors s = readSensors();
    if (s.l1 == BLACK || s.r1 == BLACK) {
      Serial.println(F("[SEEK] line found"));
      return;
    }

    if (millis() - lastLogTime >= SEEK_LINE_LOG_MS) {
      lastLogTime = millis();
      Serial.println(F("[SEEK] still searching"));
    }

    delay(1);
  }
}

void handleRightAngleTurn(bool turnLeft) {
  Serial.println(turnLeft ? F("[TURN] L") : F("[TURN] R"));
  moveForward();
  delay(TURN_FORWARD_MS);

  Sensors s = readSensors();
  if (isAllWhite(s)) {
    rotateUntilLineFound(turnLeft);
  }
}

void completeStationIfNeeded() {
  if (mode != MODE_SUB) {
    return;
  }

  if (boxNum > 0) {
    return;
  }

  boxNum = 0;
  mode = MODE_ADD;
  Serial.println(F("[MODE] A"));
}

bool shouldCheckLineEnd() {
  return stationNum == 3 && boxNum == 0 && mode == MODE_ADD;
}

bool confirmLineEndCandidate() {
  unsigned long startTime = millis();
  Serial.println(F("[END] ?"));

  while (true) {
    moveForward();

    Sensors s = readSensors();
    if (!isGapMark(s)) {
      Serial.println(F("[END] gap"));
      return false;
    }

    if (millis() - startTime >= END_CONFIRM_MS) {
      Serial.println(F("[END] yes"));
      return true;
    }

    delay(1);
  }
}

void performDropoff() {
  robotState = STATE_DROPPING;
  isDropping = true;
  Serial.println(F("[DROP] start"));

  stopMoving();
  attachDumpServoIfNeeded();

  for (int pos = DUMP_HOME_ANGLE; pos <= DUMP_RELEASE_ANGLE; pos++) {
    dumpServo.write(pos);
    delay(10);
  }

  delay(DROP_PAYLOAD_MS);

  for (int pos = DUMP_RELEASE_ANGLE; pos >= DUMP_HOME_ANGLE; pos--) {
    dumpServo.write(pos);
    delay(10);
  }

  delay(DROP_RETURN_SETTLE_MS);
  detachDumpServoIfNeeded();

  moveForward();
  delay(DROP_EXIT_MS);

  isDropping = false;
  robotState = STATE_RUNNING;
  Serial.println(F("[DROP] done"));
}

void performTurnaround() {
  robotState = STATE_TURNING;
  isTurning = true;
  Serial.println(F("[TURN] U"));

  stopMoving();
  delay(120);

  rotateUntilLineFound(true);
  moveForward();
  delay(DROP_EXIT_MS);

  resetTripState();
  robotState = STATE_RUNNING;
  isTurning = false;
  Serial.println(F("[TURN] done"));
}

void handleGapEvent() {
  straightLineArmed = false;

  if (shouldCheckLineEnd() && confirmLineEndCandidate()) {
    performTurnaround();
    return;
  }

  if (mode == MODE_ADD) {
    boxNum++;
  } else if (boxNum > 0) {
    boxNum--;
  }

  Serial.print(F("[GAP] "));
  Serial.print(mode == MODE_ADD ? 'A' : 'S');
  Serial.print(F(" b="));
  Serial.println(boxNum);

  completeStationIfNeeded();
  delay(SENSOR_EVENT_BLOCK_MS);
}

void handleStationEvent() {
  int currentStation = boxNum + 1;
  stationNum++;
  straightLineArmed = false;

  Serial.print(F("[ST] cur="));
  Serial.print(currentStation);
  Serial.print(F(" tgt="));
  Serial.print(targetStation);
  Serial.print(F(" s="));
  Serial.println(stationNum);

  if (currentStation == targetStation) {
    performDropoff();
  }

  mode = MODE_SUB;
  completeStationIfNeeded();
  delay(SENSOR_EVENT_BLOCK_MS);
}

void beginRunning() {
  resetTripState();
  robotState = STATE_RUNNING;
  Serial.print(F("[RUN] start t="));
  Serial.print(stationLabel(targetStation));
  Serial.print(F(" ("));
  Serial.print(targetStation);
  Serial.println(F(")"));
}

void handleSettingTarget() {
  stopMoving();

  ButtonEvent startEvent = pollButtonEvent(startButton, START_HOLD_MS);

  if (startEvent == BUTTON_EVENT_SHORT_RELEASE) {
    unsigned long now = millis();

    if ((targetStation > 0) &&
        ((now - lastTargetSelectionTime) > TARGET_SELECTION_RESET_MS)) {
      targetStation = 0;
    }

    if (targetStation < MAX_TARGET_STATION) {
      targetStation++;
    } else {
      targetStation = 1;
    }

    lastTargetSelectionTime = now;
    Serial.print(F("[SET] target="));
    Serial.print(stationLabel(targetStation));
    Serial.print(F(" taps="));
    Serial.println(targetStation);
    return;
  }

  if (startEvent != BUTTON_EVENT_LONG_PRESS) {
    return;
  }

  if (targetStation < 1 || targetStation > MAX_TARGET_STATION) {
    Serial.println(F("[READY] tap START 1=A 2=B 3=C, then hold START to run"));
    return;
  }

  Sensors s = readSensors();
  if (!isCenterLine(s)) {
    Serial.print(F("[READY] need 0110 got "));
    printSensorPattern(s);
    Serial.println();
    return;
  }

  beginRunning();
}

void followLine(const Sensors &s) {
  if (isCenterLine(s)) {
    straightLineArmed = true;
  }

  if (straightLineArmed && isLeftTurnSignal(s)) {
    straightLineArmed = false;
    handleRightAngleTurn(true);
    return;
  }

  if (straightLineArmed && isRightTurnSignal(s)) {
    straightLineArmed = false;
    handleRightAngleTurn(false);
    return;
  }

  if (s.l2 == WHITE && s.r2 == WHITE) {
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
    return;
  }

  stopMoving();
}

void handleRunning() {
  Sensors s = readSensors();

  if (!isStationMark(s)) {
    stationLatched = false;
  }
  if (!isGapMark(s)) {
    gapLatched = false;
  }

  if (!stationLatched && isStationMark(s)) {
    stationLatched = true;
    handleStationEvent();
    return;
  }

  if (!gapLatched && isGapMark(s)) {
    gapLatched = true;
    handleGapEvent();
    return;
  }

  followLine(s);
}

void printStatus() {
  static unsigned long lastPrintTime = 0;

  if (millis() - lastPrintTime < STATUS_PRINT_MS) {
    return;
  }

  lastPrintTime = millis();
  Sensors s = readSensors();

  if (robotState == STATE_SETTING_TARGET) {
    Serial.print(F("[READY] t="));
    if (targetStation > 0) {
      Serial.print(stationLabel(targetStation));
      Serial.print(F(" ("));
      Serial.print(targetStation);
      Serial.print(F(" tap"));
      if (targetStation > 1) {
        Serial.print('s');
      }
      Serial.print(F(")"));
    } else {
      Serial.print(F("none"));
    }
    Serial.print(F(" s="));
    printSensorPattern(s);
    if (isCenterLine(s)) {
      Serial.println(F(" ok, tap 1=A 2=B 3=C then hold START"));
    } else {
      Serial.println(F(" wait"));
    }
    return;
  }

  if (robotState != STATE_RUNNING) {
    return;
  }

  Serial.print(F("[RUN] t="));
  Serial.print(targetStation);
  Serial.print(F(" s="));
  Serial.print(stationNum);
  Serial.print(F(" b="));
  Serial.print(boxNum);
  Serial.print(F(" m="));
  Serial.print(mode == MODE_ADD ? 'A' : 'S');
  Serial.print(F(" x="));
  printSensorPattern(s);
  Serial.println();
}

void setup() {
  Serial.begin(9600);
  Serial.println(F("[BOOT] init"));

  pinMode(PIN_SENSOR_L2, INPUT);
  pinMode(PIN_SENSOR_L1, INPUT);
  pinMode(PIN_SENSOR_R1, INPUT);
  pinMode(PIN_SENSOR_R2, INPUT);

  setupButton(startButton);

  leftServo.attach(PIN_SERVO_LEFT);
  rightServo.attach(PIN_SERVO_RIGHT);
  stopMoving();

  dumpServo.detach();
  dumpServoAttached = false;

  resetTripState();

  Serial.print(F("[BOOT] btn start="));
  Serial.println(PIN_BTN_START);
  Serial.println(F("[BOOT] sensor raw active-low -> normalized to black=1 white=0"));
  Serial.println(F("[BOOT] input: tap START 1=A 2=B 3=C, hold START to run"));
  printButtonBootState(startButton);
  Serial.println(F("[BOOT] target=none"));
}

void loop() {
  printStatus();

  if (robotState == STATE_SETTING_TARGET) {
    warnIfButtonStuck(startButton);
    handleSettingTarget();
  } else if (robotState == STATE_RUNNING) {
    handleRunning();
  }

  delay(LOOP_DELAY_MS);
}
