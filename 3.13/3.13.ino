#include <Servo.h>

// ===== Hardware Pins =====
// L2/L1/R1/R2 are kept in the same physical order as the original wiring:
// left outer, left inner, right inner, right outer.
#define PIN_SENSOR_LEFT_OUTER 10
#define PIN_SENSOR_LEFT_INNER 11
#define PIN_SENSOR_RIGHT_INNER 9
#define PIN_SENSOR_RIGHT_OUTER 8

#define PIN_LEFT_SERVO 5
#define PIN_RIGHT_SERVO 4
#define PIN_DUMP_SERVO 7

#define PIN_BUTTON_SELECT 13
#define PIN_BUTTON_START 6

// ===== Logic Levels =====
#define BUTTON_RELEASED HIGH
#define BUTTON_PRESSED LOW

#define LINE_BLACK HIGH
#define LINE_WHITE LOW

// ===== Timing and Motion Constants =====
const unsigned long LOOP_DELAY_MS = 10;
const unsigned long BUTTON_DEBOUNCE_MS = 30;
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

// ===== Robot States =====
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

// ===== Data Structures =====
struct LineSensors {
  int leftOuter;
  int leftInner;
  int rightInner;
  int rightOuter;
};

struct ButtonState {
  uint8_t pin;
  const char *name;
  bool stableLevel;
  bool lastReading;
  bool releaseArmed;
  bool stuckLowWarned;
  unsigned long stableSinceTime;
  unsigned long lastChangeTime;
};

// ===== Hardware Objects =====
Servo leftServo;
Servo rightServo;
Servo dumpServo;

ButtonState selectButton = {
  PIN_BUTTON_SELECT, "SELECT", BUTTON_RELEASED, BUTTON_RELEASED, false, false, 0, 0
};
ButtonState startButton = {
  PIN_BUTTON_START, "START", BUTTON_RELEASED, BUTTON_RELEASED, false, false, 0, 0
};

// ===== Runtime State =====
RobotState robotState = STATE_SETTING_TARGET;
GapMode gapMode = MODE_ADD;

int targetStation = 1;
int passedStationCount = 0;
int boxOffset = 0;

// A turn signal is only meaningful after the robot has just seen the center line.
bool turnSignalArmed = false;
bool dumpServoAttached = false;

// These latches prevent the same solid marker from being counted on every loop.
bool stationMarkerLatched = false;
bool gapMarkerLatched = false;

// ===== Debug Print Helpers =====
const char *levelName(bool level) {
  return level == BUTTON_PRESSED ? "LOW/PRESSED" : "HIGH/RELEASED";
}

void printPrefix() {
  Serial.print('[');
  Serial.print(millis());
  Serial.print(F(" ms] "));
}

void printSensorPattern(const LineSensors &sensors) {
  Serial.print(sensors.leftOuter);
  Serial.print(sensors.leftInner);
  Serial.print(sensors.rightInner);
  Serial.print(sensors.rightOuter);
}

void printButtonBootState(const ButtonState &button) {
  Serial.print(F("[BOOT] "));
  Serial.print(button.name);
  Serial.print(F(" initial state = "));
  Serial.println(levelName(button.stableLevel));
}

// ===== Sensor Reading and Pattern Checks =====
// The sensors are wired active-low, so raw HIGH means white floor and raw LOW
// means black tape. The rest of the program works with the normalized values.
int readNormalizedLineSensor(uint8_t pin) {
  int rawValue = digitalRead(pin);
  return rawValue == HIGH ? LINE_WHITE : LINE_BLACK;
}

LineSensors readLineSensors() {
  LineSensors sensors;
  sensors.leftOuter = readNormalizedLineSensor(PIN_SENSOR_LEFT_OUTER);
  sensors.leftInner = readNormalizedLineSensor(PIN_SENSOR_LEFT_INNER);
  sensors.rightInner = readNormalizedLineSensor(PIN_SENSOR_RIGHT_INNER);
  sensors.rightOuter = readNormalizedLineSensor(PIN_SENSOR_RIGHT_OUTER);
  return sensors;
}

bool isAllWhite(const LineSensors &sensors) {
  return sensors.leftOuter == LINE_WHITE &&
         sensors.leftInner == LINE_WHITE &&
         sensors.rightInner == LINE_WHITE &&
         sensors.rightOuter == LINE_WHITE;
}

bool isAllBlack(const LineSensors &sensors) {
  return sensors.leftOuter == LINE_BLACK &&
         sensors.leftInner == LINE_BLACK &&
         sensors.rightInner == LINE_BLACK &&
         sensors.rightOuter == LINE_BLACK;
}

bool isCenterLine(const LineSensors &sensors) {
  return sensors.leftOuter == LINE_WHITE &&
         sensors.leftInner == LINE_BLACK &&
         sensors.rightInner == LINE_BLACK &&
         sensors.rightOuter == LINE_WHITE;
}

bool isGapMark(const LineSensors &sensors) {
  return isAllWhite(sensors);
}

bool isStationMark(const LineSensors &sensors) {
  return isAllBlack(sensors);
}

bool isLeftTurnSignal(const LineSensors &sensors) {
  return sensors.leftOuter == LINE_BLACK &&
         sensors.rightOuter == LINE_WHITE &&
         !isStationMark(sensors);
}

bool isRightTurnSignal(const LineSensors &sensors) {
  return sensors.leftOuter == LINE_WHITE &&
         sensors.rightOuter == LINE_BLACK &&
         !isStationMark(sensors);
}

bool areOuterSensorsWhite(const LineSensors &sensors) {
  return sensors.leftOuter == LINE_WHITE && sensors.rightOuter == LINE_WHITE;
}

// ===== Drive Helpers =====
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

  dumpServo.attach(PIN_DUMP_SERVO);
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

// ===== Button Helpers =====
void setupButton(ButtonState &button) {
  pinMode(button.pin, INPUT_PULLUP);
  button.stableLevel = digitalRead(button.pin);
  button.lastReading = button.stableLevel;
  button.releaseArmed = false;
  button.stuckLowWarned = false;
  button.stableSinceTime = millis();
  button.lastChangeTime = millis();
}

// A button event happens on a stable release, not on the initial press.
// This avoids repeated triggers while the button is still being held down.
bool pollButtonReleaseEvent(ButtonState &button) {
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
  button.stableSinceTime = millis();
  button.stuckLowWarned = false;

  if (button.stableLevel == BUTTON_PRESSED) {
    button.releaseArmed = true;
    return false;
  }

  if (!button.releaseArmed) {
    return false;
  }

  button.releaseArmed = false;
  return true;
}

void warnIfButtonStuck(ButtonState &button) {
  if (button.stableLevel != BUTTON_PRESSED) {
    return;
  }

  if (button.releaseArmed) {
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

// ===== Mission State Helpers =====
void resetTripState() {
  passedStationCount = 0;
  boxOffset = 0;
  gapMode = MODE_ADD;
  turnSignalArmed = false;
  stationMarkerLatched = false;
  gapMarkerLatched = false;
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
  Serial.println(F("[MODE] A"));
}

bool shouldCheckLineEnd() {
  return passedStationCount == 3 && boxOffset == 0 && gapMode == MODE_ADD;
}

// The robot only treats a long all-white region as the end of the line after
// reaching the final station and returning to the first box lane.
bool confirmLineEndCandidate() {
  unsigned long startTime = millis();
  Serial.println(F("[END] ?"));

  while (true) {
    moveForward();

    LineSensors sensors = readLineSensors();
    if (!isGapMark(sensors)) {
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

void rotateUntilLineFound(bool turnLeft) {
  unsigned long lastLogTime = millis();

  while (true) {
    if (turnLeft) {
      turnLeftHard();
    } else {
      turnRightHard();
    }

    LineSensors sensors = readLineSensors();
    if (sensors.leftInner == LINE_BLACK || sensors.rightInner == LINE_BLACK) {
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

  LineSensors sensors = readLineSensors();
  if (isAllWhite(sensors)) {
    rotateUntilLineFound(turnLeft);
  }
}

void performDropoff() {
  robotState = STATE_DROPPING;
  Serial.println(F("[DROP] start"));

  stopMoving();
  attachDumpServoIfNeeded();

  for (int position = DUMP_HOME_ANGLE; position <= DUMP_RELEASE_ANGLE; position++) {
    dumpServo.write(position);
    delay(10);
  }

  delay(DROP_PAYLOAD_MS);

  for (int position = DUMP_RELEASE_ANGLE; position >= DUMP_HOME_ANGLE; position--) {
    dumpServo.write(position);
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
  Serial.println(F("[TURN] U"));

  stopMoving();
  delay(120);

  rotateUntilLineFound(true);
  moveForward();
  delay(DROP_EXIT_MS);

  resetTripState();
  robotState = STATE_RUNNING;
  Serial.println(F("[TURN] done"));
}

void handleGapEvent() {
  turnSignalArmed = false;

  if (shouldCheckLineEnd() && confirmLineEndCandidate()) {
    performTurnaround();
    return;
  }

  if (gapMode == MODE_ADD) {
    boxOffset++;
  } else if (boxOffset > 0) {
    boxOffset--;
  }

  Serial.print(F("[GAP] "));
  Serial.print(gapMode == MODE_ADD ? 'A' : 'S');
  Serial.print(F(" b="));
  Serial.println(boxOffset);

  finishSubModeIfNeeded();
  delay(SENSOR_EVENT_BLOCK_MS);
}

void handleStationEvent() {
  int currentStation = boxOffset + 1;
  passedStationCount++;
  turnSignalArmed = false;

  Serial.print(F("[ST] cur="));
  Serial.print(currentStation);
  Serial.print(F(" tgt="));
  Serial.print(targetStation);
  Serial.print(F(" s="));
  Serial.println(passedStationCount);

  if (currentStation == targetStation) {
    performDropoff();
  }

  gapMode = MODE_SUB;
  finishSubModeIfNeeded();
  delay(SENSOR_EVENT_BLOCK_MS);
}

void beginRunning() {
  resetTripState();
  robotState = STATE_RUNNING;
  Serial.print(F("[RUN] start t="));
  Serial.println(targetStation);
}

// ===== Running Logic =====
void updateTurnSignalArm(const LineSensors &sensors) {
  if (isCenterLine(sensors)) {
    turnSignalArmed = true;
  }
}

bool handleTurnSignal(const LineSensors &sensors) {
  if (!turnSignalArmed) {
    return false;
  }

  if (isLeftTurnSignal(sensors)) {
    turnSignalArmed = false;
    handleRightAngleTurn(true);
    return true;
  }

  if (isRightTurnSignal(sensors)) {
    turnSignalArmed = false;
    handleRightAngleTurn(false);
    return true;
  }

  return false;
}

void applyLineFollowingMotion(const LineSensors &sensors) {
  if (!areOuterSensorsWhite(sensors)) {
    stopMoving();
    return;
  }

  if (sensors.leftInner == LINE_BLACK && sensors.rightInner == LINE_BLACK) {
    moveForward();
  } else if (sensors.leftInner == LINE_BLACK && sensors.rightInner == LINE_WHITE) {
    turnLeftSoft();
  } else if (sensors.leftInner == LINE_WHITE && sensors.rightInner == LINE_BLACK) {
    turnRightSoft();
  } else if (isAllWhite(sensors)) {
    moveForward();
  } else {
    stopMoving();
  }
}

void followLine(const LineSensors &sensors) {
  updateTurnSignalArm(sensors);

  if (handleTurnSignal(sensors)) {
    return;
  }

  applyLineFollowingMotion(sensors);
}

void releaseMarkerLatchesIfNeeded(const LineSensors &sensors) {
  if (!isStationMark(sensors)) {
    stationMarkerLatched = false;
  }

  if (!isGapMark(sensors)) {
    gapMarkerLatched = false;
  }
}

bool handleMarkersIfDetected(const LineSensors &sensors) {
  if (!stationMarkerLatched && isStationMark(sensors)) {
    stationMarkerLatched = true;
    handleStationEvent();
    return true;
  }

  if (!gapMarkerLatched && isGapMark(sensors)) {
    gapMarkerLatched = true;
    handleGapEvent();
    return true;
  }

  return false;
}

void handleRunning() {
  LineSensors sensors = readLineSensors();

  releaseMarkerLatchesIfNeeded(sensors);

  if (handleMarkersIfDetected(sensors)) {
    return;
  }

  followLine(sensors);
}

// ===== Target Selection State =====
void handleSettingTarget() {
  stopMoving();

  if (pollButtonReleaseEvent(selectButton)) {
    if (targetStation < 3) {
      targetStation++;
    }

    Serial.print(F("[SET] targetStation="));
    Serial.println(targetStation);
  }

  if (!pollButtonReleaseEvent(startButton)) {
    return;
  }

  LineSensors sensors = readLineSensors();
  if (!isCenterLine(sensors)) {
    Serial.print(F("[READY] need 0110 got "));
    printSensorPattern(sensors);
    Serial.println();
    return;
  }

  beginRunning();
}

// ===== Status Printing =====
void printStatus() {
  static unsigned long lastPrintTime = 0;

  if (millis() - lastPrintTime < STATUS_PRINT_MS) {
    return;
  }

  lastPrintTime = millis();
  LineSensors sensors = readLineSensors();

  if (robotState == STATE_SETTING_TARGET) {
    Serial.print(F("[READY] t="));
    Serial.print(targetStation);
    Serial.print(F(" s="));
    printSensorPattern(sensors);
    Serial.println(isCenterLine(sensors) ? F(" ok") : F(" wait"));
    return;
  }

  if (robotState != STATE_RUNNING) {
    return;
  }

  Serial.print(F("[RUN] t="));
  Serial.print(targetStation);
  Serial.print(F(" s="));
  Serial.print(passedStationCount);
  Serial.print(F(" b="));
  Serial.print(boxOffset);
  Serial.print(F(" m="));
  Serial.print(gapMode == MODE_ADD ? 'A' : 'S');
  Serial.print(F(" x="));
  printSensorPattern(sensors);
  Serial.println();
}

// ===== Setup Helpers =====
void setupSensorPins() {
  pinMode(PIN_SENSOR_LEFT_OUTER, INPUT);
  pinMode(PIN_SENSOR_LEFT_INNER, INPUT);
  pinMode(PIN_SENSOR_RIGHT_INNER, INPUT);
  pinMode(PIN_SENSOR_RIGHT_OUTER, INPUT);
}

void setupButtons() {
  setupButton(selectButton);
  setupButton(startButton);
}

void setupDriveServos() {
  leftServo.attach(PIN_LEFT_SERVO);
  rightServo.attach(PIN_RIGHT_SERVO);
  stopMoving();
}

void setupDumpServo() {
  dumpServo.detach();
  dumpServoAttached = false;
}

void printBootMessages() {
  Serial.print(F("[BOOT] btn start="));
  Serial.print(PIN_BUTTON_START);
  Serial.print(F(" select="));
  Serial.println(PIN_BUTTON_SELECT);
  Serial.println(F("[BOOT] sensor raw active-low -> normalized to black=1 white=0"));
  Serial.println(F("[BOOT] button event = stable release"));
  printButtonBootState(startButton);
  printButtonBootState(selectButton);
  Serial.println(F("[BOOT] target=1"));
}

// ===== Arduino Entry Points =====
void setup() {
  Serial.begin(9600);
  Serial.println(F("[BOOT] init"));

  setupSensorPins();
  setupButtons();
  setupDriveServos();
  setupDumpServo();
  resetTripState();
  printBootMessages();
}

void loop() {
  printStatus();

  if (robotState == STATE_SETTING_TARGET) {
    warnIfButtonStuck(startButton);
    warnIfButtonStuck(selectButton);
    handleSettingTarget();
  } else if (robotState == STATE_RUNNING) {
    handleRunning();
  }

  delay(LOOP_DELAY_MS);
}
