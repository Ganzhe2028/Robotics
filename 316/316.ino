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
const unsigned long STATUS_PRINT_MS = 250;
const unsigned long TURN_FORWARD_MS = 500;
const unsigned long TURN_SIGNAL_CONFIRM_MS = 50;
const unsigned long SEEK_LINE_LOG_MS = 1000;
const unsigned long DROP_PAYLOAD_MS = 500;
const unsigned long DROP_RETURN_SETTLE_MS = 250;
const unsigned long DROP_EXIT_MS = 150;
const unsigned long STUCK_LOW_WARN_MS = 1500;
const unsigned long GAP_EVENT_COOLDOWN_MS = 150;
const unsigned long STATION_EVENT_COOLDOWN_MS = 600;

const int LEFT_STOP_ANGLE = 90;
const int RIGHT_STOP_ANGLE = 90;
const int LEFT_FORWARD_ANGLE = 180;
const int RIGHT_FORWARD_ANGLE = 0;
const int LEFT_SOFT_LEFT_ANGLE = 90;
const int RIGHT_SOFT_LEFT_ANGLE = 0;
const int LEFT_SOFT_RIGHT_ANGLE = 180;
const int RIGHT_SOFT_RIGHT_ANGLE = 90;
const int LEFT_HARD_LEFT_ANGLE = 0;
const int RIGHT_HARD_LEFT_ANGLE = 0;
const int LEFT_HARD_RIGHT_ANGLE = 180;
const int RIGHT_HARD_RIGHT_ANGLE = 180;

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

enum TravelDirection {
  DIR_UNKNOWN,
  DIR_TO_A,
  DIR_TO_C
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
GapMode gapMode = MODE_ADD;
TravelDirection travelDirection = DIR_UNKNOWN;

int targetStation = 0;
int passedStationCount = 0;
int boxOffset = 0;
int lastConfirmedStation = 0;
unsigned long lastTargetSelectionTime = 0;

bool straightLineArmed = false;
bool dumpServoAttached = false;
bool stationLatched = false;
bool gapLatched = false;
bool stationArmed = true;
bool gapArmed = true;
bool gapCandidateActive = false;
bool gapCandidateLong = false;
unsigned long lastGapEventTime = 0;
unsigned long lastStationEventTime = 0;
unsigned long gapCandidateStartTime = 0;

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

bool areOuterSensorsWhite(const Sensors &s) {
  return s.l2 == WHITE && s.r2 == WHITE;
}

bool isTrackRecoveredPattern(const Sensors &s) {
  return areOuterSensorsWhite(s) && (s.l1 == BLACK || s.r1 == BLACK);
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

char stationLabel(int station) {
  if (station < 1 || station > MAX_TARGET_STATION) {
    return '-';
  }

  return 'A' + station - 1;
}

const char *modeName(GapMode mode) {
  return mode == MODE_ADD ? "ADD" : "SUB";
}

const char *directionName(TravelDirection direction) {
  if (direction == DIR_TO_A) {
    return "TO_A";
  }

  if (direction == DIR_TO_C) {
    return "TO_C";
  }

  return "UNKNOWN";
}

const char *levelName(bool level) {
  return level == BTN_PRESSED ? "LOW/PRESSED" : "HIGH/RELEASED";
}

bool isValidStationNumber(int station) {
  return station >= 1 && station <= MAX_TARGET_STATION;
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

void setDrive(int leftAngle, int rightAngle) {
  leftServo.write(leftAngle);
  rightServo.write(rightAngle);
}

void stopMoving() {
  setDrive(LEFT_STOP_ANGLE, RIGHT_STOP_ANGLE);
}

void moveForward() {
  setDrive(LEFT_FORWARD_ANGLE, RIGHT_FORWARD_ANGLE);
}

void turnLeftSoft() {
  setDrive(LEFT_SOFT_LEFT_ANGLE, RIGHT_SOFT_LEFT_ANGLE);
}

void turnRightSoft() {
  setDrive(LEFT_SOFT_RIGHT_ANGLE, RIGHT_SOFT_RIGHT_ANGLE);
}

void turnLeftHard() {
  setDrive(LEFT_HARD_LEFT_ANGLE, RIGHT_HARD_LEFT_ANGLE);
}

void turnRightHard() {
  setDrive(LEFT_HARD_RIGHT_ANGLE, RIGHT_HARD_RIGHT_ANGLE);
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
  gapMode = MODE_ADD;
  passedStationCount = 0;
  boxOffset = 0;
  straightLineArmed = false;
  stationLatched = false;
  gapLatched = false;
  stationArmed = true;
  gapArmed = true;
  gapCandidateActive = false;
  gapCandidateLong = false;
  lastGapEventTime = 0;
  lastStationEventTime = 0;
  gapCandidateStartTime = 0;
}

void resetNavigationState() {
  travelDirection = DIR_UNKNOWN;
  lastConfirmedStation = 0;
}

void updateTravelDirection(int currentStation) {
  if (!isValidStationNumber(currentStation)) {
    return;
  }

  if (isValidStationNumber(lastConfirmedStation)) {
    if (currentStation > lastConfirmedStation) {
      travelDirection = DIR_TO_C;
    } else if (currentStation < lastConfirmedStation) {
      travelDirection = DIR_TO_A;
    }
  }

  lastConfirmedStation = currentStation;
}

bool shouldAllowTurnaround() {
  if (travelDirection == DIR_TO_A && lastConfirmedStation == 1) {
    return true;
  }

  if (travelDirection == DIR_TO_C && lastConfirmedStation == MAX_TARGET_STATION) {
    return true;
  }

  return false;
}

void updateDirectionAfterTurnaround() {
  if (lastConfirmedStation == 1) {
    travelDirection = DIR_TO_C;
    return;
  }

  if (lastConfirmedStation == MAX_TARGET_STATION) {
    travelDirection = DIR_TO_A;
    return;
  }

  if (travelDirection == DIR_TO_A) {
    travelDirection = DIR_TO_C;
  } else if (travelDirection == DIR_TO_C) {
    travelDirection = DIR_TO_A;
  }
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
  Serial.println(turnLeft ? F("[TURN] left signal") : F("[TURN] right signal"));
  moveForward();
  delay(TURN_FORWARD_MS);

  Sensors s = readSensors();
  if (isAllWhite(s)) {
    rotateUntilLineFound(turnLeft);
  }
}

void finishSubModeIfNeeded() {
  if (gapMode != MODE_SUB) {
    return;
  }

  if (boxOffset > 0) {
    return;
  }

  boxOffset = 0;
  gapMode = MODE_ADD;

  printPrefix();
  Serial.println(F("[MODE] add"));
}

void startGapCandidate() {
  gapCandidateActive = true;
  gapCandidateLong = false;
  gapCandidateStartTime = millis();
  straightLineArmed = false;
  gapArmed = false;

  printPrefix();
  Serial.println(F("[GAP] candidate"));
}

void performDropoff() {
  robotState = STATE_DROPPING;
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

  robotState = STATE_RUNNING;
  Serial.println(F("[DROP] done"));
}

void performTurnaround() {
  robotState = STATE_TURNING;
  Serial.println(F("[TURN] turnaround"));

  stopMoving();
  delay(120);

  rotateUntilLineFound(true);
  moveForward();
  delay(DROP_EXIT_MS);

  updateDirectionAfterTurnaround();
  resetTripState();
  robotState = STATE_RUNNING;
  Serial.println(F("[TURN] done"));
}

void handleGapEvent() {
  straightLineArmed = false;
  lastGapEventTime = millis();
  gapArmed = false;
  gapCandidateActive = false;
  gapCandidateLong = false;

  if (gapMode == MODE_ADD) {
    boxOffset++;
  } else if (boxOffset > 0) {
    boxOffset--;
  }

  printPrefix();
  Serial.print(F("[GAP] "));
  Serial.print(gapMode == MODE_ADD ? F("add") : F("sub"));
  Serial.print(F(" box="));
  Serial.print(boxOffset);
  Serial.print(F(" x=0000"));
  Serial.println();

  finishSubModeIfNeeded();
  delay(SENSOR_EVENT_BLOCK_MS);
}

void handleStationEvent() {
  int currentStation = boxOffset + 1;
  passedStationCount++;
  straightLineArmed = false;
  lastStationEventTime = millis();
  stationArmed = false;
  updateTravelDirection(currentStation);

  printPrefix();
  Serial.print(F("[ST] station detected cur="));
  Serial.print(stationLabel(currentStation));
  Serial.print(F(" tgt="));
  Serial.print(stationLabel(targetStation));
  Serial.print(F(" pass="));
  Serial.print(passedStationCount);
  Serial.print(F(" box="));
  Serial.print(boxOffset);
  Serial.print(F(" x=1111"));
  Serial.println();

  if (currentStation == targetStation) {
    printPrefix();
    Serial.println(F("[ST] station match -> drop"));
    performDropoff();
  } else {
    printPrefix();
    Serial.println(F("[ST] station pass"));
  }

  gapMode = MODE_SUB;
  finishSubModeIfNeeded();
  delay(SENSOR_EVENT_BLOCK_MS);
}

void beginRunning() {
  resetTripState();
  resetNavigationState();
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

void applyLineFollowingMotion(const Sensors &s) {
  if (s.l2 == BLACK && s.r2 == WHITE) {
    turnLeftHard();
    return;
  }

  if (s.l2 == WHITE && s.r2 == BLACK) {
    turnRightHard();
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

bool confirmTurnSignal(bool turnLeft) {
  delay(TURN_SIGNAL_CONFIRM_MS);

  Sensors confirmSensors = readSensors();
  if (isStationMark(confirmSensors)) {
    printPrefix();
    Serial.print(F("[TURN] canceled by station x="));
    printSensorPattern(confirmSensors);
    Serial.println();
    return false;
  }

  if (turnLeft ? isLeftTurnSignal(confirmSensors) : isRightTurnSignal(confirmSensors)) {
    return true;
  }

  printPrefix();
  Serial.print(F("[TURN] canceled x="));
  printSensorPattern(confirmSensors);
  Serial.println();
  return false;
}

bool handleGapCandidate(const Sensors &s) {
  if (!gapCandidateActive) {
    return false;
  }

  if (isGapMark(s)) {
    moveForward();

    if (!gapCandidateLong &&
        millis() - gapCandidateStartTime >= END_CONFIRM_MS) {
      if (shouldAllowTurnaround()) {
        printPrefix();
        Serial.print(F("[END] confirmed last="));
        Serial.print(stationLabel(lastConfirmedStation));
        Serial.print(F(" dir="));
        Serial.println(directionName(travelDirection));
        gapCandidateActive = false;
        performTurnaround();
      } else {
        gapCandidateLong = true;
        printPrefix();
        Serial.print(F("[END] blocked last="));
        Serial.print(stationLabel(lastConfirmedStation));
        Serial.print(F(" dir="));
        Serial.println(directionName(travelDirection));
      }
    }

    return true;
  }

  if (gapCandidateLong) {
    gapCandidateActive = false;
    gapCandidateLong = false;
    printPrefix();
    Serial.print(F("[GAP] long candidate canceled x="));
    printSensorPattern(s);
    Serial.println();
    return false;
  }

  if (!isTrackRecoveredPattern(s) && !isStationMark(s)) {
    applyLineFollowingMotion(s);
    return true;
  }

  handleGapEvent();
  return true;
}

void followLine(const Sensors &s) {
  if (isCenterLine(s)) {
    straightLineArmed = true;
  }

  if (straightLineArmed && isLeftTurnSignal(s)) {
    if (confirmTurnSignal(true)) {
      straightLineArmed = false;
      handleRightAngleTurn(true);
      return;
    }
  }

  if (straightLineArmed && isRightTurnSignal(s)) {
    if (confirmTurnSignal(false)) {
      straightLineArmed = false;
      handleRightAngleTurn(false);
      return;
    }
  }

  applyLineFollowingMotion(s);
}

void releaseMarkerLatchesIfNeeded(const Sensors &s) {
  if (isTrackRecoveredPattern(s)) {
    stationLatched = false;
    gapLatched = false;
    stationArmed = true;
    gapArmed = true;
    return;
  }

  if (stationLatched && (areOuterSensorsWhite(s) || isGapMark(s))) {
    stationLatched = false;
  }

  if (!isGapMark(s)) {
    gapLatched = false;
  }
}

bool handleMarkersIfDetected(const Sensors &s) {
  if (!gapCandidateActive &&
      gapArmed &&
      !gapLatched &&
      isGapMark(s) &&
      (lastGapEventTime == 0 ||
       millis() - lastGapEventTime >= GAP_EVENT_COOLDOWN_MS)) {
    gapLatched = true;
    startGapCandidate();
    return true;
  }

  if (!stationLatched &&
      stationArmed &&
      isStationMark(s) &&
      (lastStationEventTime == 0 ||
       millis() - lastStationEventTime >= STATION_EVENT_COOLDOWN_MS)) {
    stationLatched = true;
    handleStationEvent();
    return true;
  }

  return false;
}

void handleRunning() {
  Sensors s = readSensors();

  releaseMarkerLatchesIfNeeded(s);

  if (handleGapCandidate(s)) {
    return;
  }

  if (handleMarkersIfDetected(s)) {
    return;
  }

  if (stationLatched && isStationMark(s)) {
    moveForward();
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
  Serial.print(stationLabel(targetStation));
  Serial.print(F(" mode="));
  Serial.print(modeName(gapMode));
  Serial.print(F(" box="));
  Serial.print(boxOffset);
  Serial.print(F(" pass="));
  Serial.print(passedStationCount);
  Serial.print(F(" last="));
  Serial.print(stationLabel(lastConfirmedStation));
  Serial.print(F(" dir="));
  Serial.print(directionName(travelDirection));
  Serial.print(F(" arm="));
  Serial.print(straightLineArmed ? 1 : 0);
  if (gapCandidateActive) {
    Serial.print(F(" gapMs="));
    Serial.print(millis() - gapCandidateStartTime);
  }
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
  Serial.print(F("[BOOT] end confirm >= "));
  Serial.print(END_CONFIRM_MS);
  Serial.println(F(" ms"));
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
