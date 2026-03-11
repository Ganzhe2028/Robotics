#include <Servo.h>

// ===== Pin Map =====
#define PIN_SENSOR_L2 10
#define PIN_SENSOR_L1 11
#define PIN_SENSOR_R1 9
#define PIN_SENSOR_R2 8

#define PIN_SERVO_LEFT 5
#define PIN_SERVO_RIGHT 4
#define PIN_SERVO_DUMP 7

#define PIN_BTN_SELECT 0
#define PIN_BTN_START 6

// ===== Logic Level =====
#define BLACK HIGH
#define WHITE LOW
#define BTN_PRESSED LOW

// ===== Tunables =====
const unsigned long LOOP_DELAY_MS = 10;
const unsigned long BUTTON_DEBOUNCE_MS = 30;
const unsigned long SENSOR_EVENT_BLOCK_MS = 100;
const unsigned long END_CONFIRM_MS = 300;
const unsigned long STATUS_PRINT_MS = 120;
const unsigned long TURN_FORWARD_MS = 500;
const unsigned long SEEK_LINE_LOG_MS = 1000;
const unsigned long DROP_PAYLOAD_MS = 500;
const unsigned long DROP_RETURN_SETTLE_MS = 250;
const unsigned long DROP_EXIT_MS = 150;

const int DUMP_HOME_ANGLE = 0;
const int DUMP_RELEASE_ANGLE = 80;

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
  bool stableLevel;
  bool lastReading;
  unsigned long lastChangeTime;
};

Servo leftServo;
Servo rightServo;
Servo dumpServo;

ButtonState selectButton = {PIN_BTN_SELECT, HIGH, HIGH, 0};
ButtonState startButton = {PIN_BTN_START, HIGH, HIGH, 0};

RobotState robotState = STATE_SETTING_TARGET;
GapMode mode = MODE_ADD;

int targetStation = 1;
int stationNum = 0;
int boxNum = 0;

bool straightLineArmed = false;
bool dumpServoAttached = false;
bool stationLatched = false;
bool gapLatched = false;
bool isTurning = false;
bool isDropping = false;

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

bool pollButtonPress(ButtonState &button) {
  bool reading = digitalRead(button.pin);

  if (reading != button.lastReading) {
    button.lastReading = reading;
    button.lastChangeTime = millis();
  }

  if ((millis() - button.lastChangeTime) < BUTTON_DEBOUNCE_MS) {
    return false;
  }

  if (reading == button.stableLevel) {
    return false;
  }

  button.stableLevel = reading;
  return button.stableLevel == BTN_PRESSED;
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
  Serial.println(turnLeft ? F("[TURN] left signal") : F("[TURN] right signal"));
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
  Serial.println(F("[MODE] station complete -> ADD"));
}

bool shouldCheckLineEnd() {
  return stationNum == 3 && boxNum == 0 && mode == MODE_ADD;
}

bool confirmLineEndCandidate() {
  unsigned long startTime = millis();
  Serial.println(F("[END] candidate"));

  while (true) {
    moveForward();

    Sensors s = readSensors();
    if (!isGapMark(s)) {
      Serial.println(F("[END] released -> gap"));
      return false;
    }

    if (millis() - startTime >= END_CONFIRM_MS) {
      Serial.println(F("[END] confirmed"));
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
  Serial.println(F("[TURN] start u-turn"));

  stopMoving();
  delay(120);

  rotateUntilLineFound(true);
  moveForward();
  delay(DROP_EXIT_MS);

  resetTripState();
  robotState = STATE_RUNNING;
  isTurning = false;
  Serial.println(F("[TURN] new trip"));
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

  Serial.print(F("[GAP] mode="));
  Serial.print(modeName(mode));
  Serial.print(F(" b="));
  Serial.println(boxNum);

  completeStationIfNeeded();
  delay(SENSOR_EVENT_BLOCK_MS);
}

void handleStationEvent() {
  int currentStation = boxNum + 1;
  stationNum++;
  straightLineArmed = false;

  Serial.print(F("[STATION] current="));
  Serial.print(currentStation);
  Serial.print(F(" target="));
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
  Serial.print(F("[BOOT] start target="));
  Serial.println(targetStation);
}

void handleSettingTarget() {
  stopMoving();

  if (pollButtonPress(selectButton)) {
    if (targetStation < 3) {
      targetStation++;
    }

    Serial.print(F("[SET] targetStation="));
    Serial.println(targetStation);
  }

  if (!pollButtonPress(startButton)) {
    return;
  }

  Sensors s = readSensors();
  if (!isCenterLine(s)) {
    Serial.print(F("[BOOT] place robot on 0110, got "));
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

  Serial.print(F("[STATUS] state="));
  Serial.print(stateName(robotState));
  Serial.print(F(" target="));
  Serial.print(targetStation);
  Serial.print(F(" s="));
  Serial.print(stationNum);
  Serial.print(F(" b="));
  Serial.print(boxNum);
  Serial.print(F(" mode="));
  Serial.print(modeName(mode));
  Serial.print(F(" turning="));
  Serial.print(isTurning ? 1 : 0);
  Serial.print(F(" dropping="));
  Serial.print(isDropping ? 1 : 0);
  Serial.print(F(" sensors="));
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

  pinMode(PIN_BTN_SELECT, INPUT_PULLUP);
  pinMode(PIN_BTN_START, INPUT_PULLUP);

  selectButton.stableLevel = digitalRead(PIN_BTN_SELECT);
  selectButton.lastReading = selectButton.stableLevel;
  selectButton.lastChangeTime = millis();

  startButton.stableLevel = digitalRead(PIN_BTN_START);
  startButton.lastReading = startButton.stableLevel;
  startButton.lastChangeTime = millis();

  leftServo.attach(PIN_SERVO_LEFT);
  rightServo.attach(PIN_SERVO_RIGHT);
  stopMoving();

  dumpServo.detach();
  dumpServoAttached = false;

  resetTripState();

  Serial.println(F("[BOOT] select target with D12, start with D6"));
  Serial.println(F("[BOOT] default targetStation=1"));
}

void loop() {
  printStatus();

  if (robotState == STATE_SETTING_TARGET) {
    handleSettingTarget();
  } else if (robotState == STATE_RUNNING) {
    handleRunning();
  }

  delay(LOOP_DELAY_MS);
}
