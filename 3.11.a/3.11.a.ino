#include <Servo.h>

Servo leftServo;
Servo rightServo;
Servo actionServo;

const int PIN_SENSOR_L2 = 10;
const int PIN_SENSOR_L1 = 11;
const int PIN_SENSOR_R1 = 9;
const int PIN_SENSOR_R2 = 8;

const int PIN_SERVO_LEFT  = 5;
const int PIN_SERVO_RIGHT = 4;

const int PIN_ACTION_SERVO = 7;

const int PIN_BUTTON = 6;

const int BTN_PRESSED = LOW;
const unsigned long BTN_DEBOUNCE_MS = 30;

const int BLACK = LOW;
const int WHITE = HIGH;

const int TURN_POWER = 65;
const int RAMP_STEPS   = 10;
const int RAMP_STEP_MS = 8;
const int TURN_90_MS  = 1000;
const int TURN_180_MS = 2000;
const int PRE_TURN_FORWARD_MS = 300;
const int AFTER_TURN_FORWARD_MS = 120;
const int SEARCH_LEFT_SPEED  = 120;
const int SEARCH_RIGHT_SPEED = 60;
const unsigned long WHITE_MARKER_SCAN_MS = 220;
const unsigned long TURN_DECISION_MEMORY_MS = 450;

// 在这里填写要卸货的站点编号，例如 {1, 4, 5}
// 站点编号定义：
// 1 = 左尽头，2/3/4 = 中间站，5 = 右尽头
const int UNLOAD_STATIONS[] = {2,4};
const int UNLOAD_STATION_COUNT = sizeof(UNLOAD_STATIONS) / sizeof(UNLOAD_STATIONS[0]);

int actionStartAngle = 20;      
int actionTargetAngle = 150;    
int actionStepDelay = 5;        
int actionStayTime = 1000;      

int lastInnerError = 0;
int lastPatternCode = -1;
bool stationHandled[6] = {false, false, false, false, false, false};
int lastConfirmedStation = 0;

enum TravelDir { DIR_UNKNOWN, DIR_TO_LEFT, DIR_TO_RIGHT };
TravelDir travelDir = DIR_UNKNOWN;

enum MarkerEvent { MARKER_NONE, MARKER_WHITE, MARKER_BLACK };
const int MARKER_HISTORY_SIZE = 5;
MarkerEvent markerHistory[MARKER_HISTORY_SIZE];
int markerHistoryCount = 0;

enum TurnDir { NONE, LEFT, RIGHT };
TurnDir pendingTurn = NONE;
unsigned long pendingTurnSeenAt = 0;

struct SensorState {
  int l2;
  int l1;
  int r1;
  int r2;
};

bool turnLockedUntilMiddleBlack = false;
bool systemStarted = false;

bool isPressedStable(unsigned long stableMs);
void waitForStartButton();
void handleButtonToggle();

SensorState readSensors();
bool middleSeesBlack(const SensorState& sensors);
bool allSensorsWhite(const SensorState& sensors);
bool allSensorsBlack(const SensorState& sensors);
bool hasRecentPendingTurn();
TurnDir detectTurnDirection(const SensorState& sensors);
int sensorPatternCode(const SensorState& sensors);
void trackStationMarkers(const SensorState& sensors);
void pushMarkerEvent(MarkerEvent event);
void handleStationDetected(int stationId);
bool shouldUnloadAtStation(int stationId);
bool wasStationHandled(int stationId);
bool isMiddleStation(int stationId);
void updateTravelDirection(int stationId);
int inferTerminalStation();
const char* travelDirText(TravelDir dir);
void printCurrentDirection();
bool tryPassWhiteMarker();
void resetMarkerTracking();
void resetStationHandling();
void beginNewLeg(int startStation);
void waitForButtonRelease();
void resetTrackingState();
void setDrive(int leftAngle, int rightAngle);
void rampToAngles(int targetLeft, int targetRight);

void moveForward();
void moveForwardSlow();
void stopMoving();
void turnLeftSoft();
void turnRightSoft();

void runStraight(int sL1, int sR1);
void smoothTurnInPlace(int turnMs, TurnDir dir);
TurnDir chooseUTurnDir();
void triggerActionServo();

void setup() {
  Serial.begin(9600);

  pinMode(PIN_SENSOR_L2, INPUT);
  pinMode(PIN_SENSOR_L1, INPUT);
  pinMode(PIN_SENSOR_R1, INPUT);
  pinMode(PIN_SENSOR_R2, INPUT);

  pinMode(PIN_BUTTON, INPUT_PULLUP);

  leftServo.attach(PIN_SERVO_LEFT);
  rightServo.attach(PIN_SERVO_RIGHT);
  actionServo.attach(PIN_ACTION_SERVO);

  stopMoving();
  actionServo.write(actionStartAngle);

  delay(1000);
  Serial.println("[BOOT] init");

  waitForStartButton();
  resetStationHandling();
  systemStarted = true;
  Serial.println("System started");
  printCurrentDirection();
}

void loop() {
  handleButtonToggle();

  if (!systemStarted) {
    stopMoving();
    delay(5);
    return;
  }

  SensorState sensors = readSensors();
  trackStationMarkers(sensors);
  bool middleBlack = middleSeesBlack(sensors);
  bool allWhite = allSensorsWhite(sensors);

  if (turnLockedUntilMiddleBlack) {
    if (middleBlack) {
      turnLockedUntilMiddleBlack = false;
      resetTrackingState();
      Serial.println("Back to main line-following");
    } else {
      moveForwardSlow();
      delay(2);
      return;
    }
  }

  if (sensors.l2 == BLACK) {
    pendingTurn = LEFT;
    pendingTurnSeenAt = millis();
  }
  if (sensors.r2 == BLACK) {
    pendingTurn = RIGHT;
    pendingTurnSeenAt = millis();
  }

  if (allWhite) {
    if (!hasRecentPendingTurn() && tryPassWhiteMarker()) {
      delay(2);
      return;
    }

    if (PRE_TURN_FORWARD_MS > 0) {
      moveForward();
      delay(PRE_TURN_FORWARD_MS);
    }

    sensors = readSensors();
    trackStationMarkers(sensors);
    middleBlack = middleSeesBlack(sensors);

    if (middleBlack) {
      runStraight(sensors.l1, sensors.r1);
      delay(2);
      return;
    }

    TurnDir currentTurn = detectTurnDirection(sensors);

    if (currentTurn != NONE) {
      pendingTurn = currentTurn;
      Serial.println("TURN 90");
      smoothTurnInPlace(TURN_90_MS, currentTurn);
    } else {
      TurnDir udir = chooseUTurnDir();
      int terminalStation = inferTerminalStation();
      Serial.print("检测到尽头，判定为站点: ");
      Serial.println(terminalStation);
      handleStationDetected(terminalStation);
      Serial.println("UTURN");

      stopMoving();
      delay(100);
      smoothTurnInPlace(TURN_180_MS, udir);
      travelDir = (terminalStation == 5) ? DIR_TO_LEFT : DIR_TO_RIGHT;
      lastConfirmedStation = terminalStation;
      beginNewLeg(terminalStation);
      Serial.println("掉头完成");
      printCurrentDirection();
    }

    turnLockedUntilMiddleBlack = true;

    moveForwardSlow();
    if (AFTER_TURN_FORWARD_MS > 0) {
      delay(AFTER_TURN_FORWARD_MS);
    }

    delay(2);
    return;
  }

  runStraight(sensors.l1, sensors.r1);
  delay(2);
}

SensorState readSensors() {
  SensorState sensors = {
    digitalRead(PIN_SENSOR_L2),
    digitalRead(PIN_SENSOR_L1),
    digitalRead(PIN_SENSOR_R1),
    digitalRead(PIN_SENSOR_R2)
  };
  return sensors;
}

bool middleSeesBlack(const SensorState& sensors) {
  return sensors.l1 == BLACK || sensors.r1 == BLACK;
}

bool allSensorsWhite(const SensorState& sensors) {
  return sensors.l2 == WHITE &&
         sensors.l1 == WHITE &&
         sensors.r1 == WHITE &&
         sensors.r2 == WHITE;
}

bool allSensorsBlack(const SensorState& sensors) {
  return sensors.l2 == BLACK &&
         sensors.l1 == BLACK &&
         sensors.r1 == BLACK &&
         sensors.r2 == BLACK;
}

bool hasRecentPendingTurn() {
  return pendingTurn != NONE &&
         millis() - pendingTurnSeenAt <= TURN_DECISION_MEMORY_MS;
}

TurnDir detectTurnDirection(const SensorState& sensors) {
  if (sensors.l2 == BLACK && sensors.r2 != BLACK) {
    return LEFT;
  }

  if (sensors.r2 == BLACK && sensors.l2 != BLACK) {
    return RIGHT;
  }

  if (sensors.l2 == BLACK && sensors.r2 == BLACK && hasRecentPendingTurn()) {
    return pendingTurn;
  }

  if (hasRecentPendingTurn()) {
    return pendingTurn;
  }

  return NONE;
}

int sensorPatternCode(const SensorState& sensors) {
  int code = 0;
  if (sensors.l2 == BLACK) code |= 0b1000;
  if (sensors.l1 == BLACK) code |= 0b0100;
  if (sensors.r1 == BLACK) code |= 0b0010;
  if (sensors.r2 == BLACK) code |= 0b0001;
  return code;
}

void pushMarkerEvent(MarkerEvent event) {
  if (markerHistoryCount < MARKER_HISTORY_SIZE) {
    markerHistory[markerHistoryCount++] = event;
    return;
  }

  for (int i = 1; i < MARKER_HISTORY_SIZE; i++) {
    markerHistory[i - 1] = markerHistory[i];
  }
  markerHistory[MARKER_HISTORY_SIZE - 1] = event;
}

bool shouldUnloadAtStation(int stationId) {
  for (int i = 0; i < UNLOAD_STATION_COUNT; i++) {
    if (UNLOAD_STATIONS[i] == stationId) {
      return true;
    }
  }
  return false;
}

bool wasStationHandled(int stationId) {
  if (stationId < 1 || stationId > 5) {
    return true;
  }

  return stationHandled[stationId];
}

bool isMiddleStation(int stationId) {
  return stationId >= 2 && stationId <= 4;
}

void updateTravelDirection(int stationId) {
  if (!isMiddleStation(stationId)) {
    return;
  }

  if (isMiddleStation(lastConfirmedStation)) {
    if (stationId > lastConfirmedStation) {
      travelDir = DIR_TO_RIGHT;
    } else if (stationId < lastConfirmedStation) {
      travelDir = DIR_TO_LEFT;
    }
  } else if (lastConfirmedStation == 1) {
    travelDir = DIR_TO_RIGHT;
  } else if (lastConfirmedStation == 5) {
    travelDir = DIR_TO_LEFT;
  }

  lastConfirmedStation = stationId;
}

int inferTerminalStation() {
  if (travelDir == DIR_TO_RIGHT) {
    return 5;
  }

  if (travelDir == DIR_TO_LEFT) {
    return 1;
  }

  if (lastConfirmedStation == 4) {
    return 5;
  }

  if (lastConfirmedStation == 2) {
    return 1;
  }

  if (lastConfirmedStation == 5) {
    return 1;
  }

  return 5;
}

const char* travelDirText(TravelDir dir) {
  if (dir == DIR_TO_LEFT) {
    return "向左";
  }

  if (dir == DIR_TO_RIGHT) {
    return "向右";
  }

  return "未知";
}

void printCurrentDirection() {
  Serial.print("当前方向: ");
  Serial.println(travelDirText(travelDir));
}

void handleStationDetected(int stationId) {
  if (wasStationHandled(stationId)) {
    return;
  }

  stationHandled[stationId] = true;
  updateTravelDirection(stationId);

  Serial.print("刚识别到站点: ");
  Serial.println(stationId);
  printCurrentDirection();

  if (!shouldUnloadAtStation(stationId)) {
    return;
  }

  Serial.print("Unload at station ");
  Serial.println(stationId);
  triggerActionServo();
}

void trackStationMarkers(const SensorState& sensors) {
  int patternCode = sensorPatternCode(sensors);
  if (patternCode == lastPatternCode) {
    return;
  }

  lastPatternCode = patternCode;

  MarkerEvent event = MARKER_NONE;
  if (patternCode == 0b0000) {
    event = MARKER_WHITE;
  } else if (patternCode == 0b1111) {
    event = MARKER_BLACK;
  }

  if (event == MARKER_NONE) {
    return;
  }

  pushMarkerEvent(event);

  if (markerHistoryCount >= 5 &&
      markerHistory[markerHistoryCount - 5] == MARKER_WHITE &&
      markerHistory[markerHistoryCount - 4] == MARKER_WHITE &&
      markerHistory[markerHistoryCount - 3] == MARKER_BLACK &&
      markerHistory[markerHistoryCount - 2] == MARKER_WHITE &&
      markerHistory[markerHistoryCount - 1] == MARKER_WHITE) {
    handleStationDetected(4);
    return;
  }

  if (markerHistoryCount >= 3 &&
      markerHistory[markerHistoryCount - 3] == MARKER_WHITE &&
      markerHistory[markerHistoryCount - 2] == MARKER_BLACK &&
      markerHistory[markerHistoryCount - 1] == MARKER_WHITE) {
    handleStationDetected(3);
    return;
  }

  if (event == MARKER_BLACK) {
    MarkerEvent prevEvent = (markerHistoryCount >= 2)
      ? markerHistory[markerHistoryCount - 2]
      : MARKER_NONE;

    if (prevEvent != MARKER_WHITE) {
      handleStationDetected(2);
    }
  }
}

bool tryPassWhiteMarker() {
  unsigned long start = millis();
  moveForward();

  while (millis() - start < WHITE_MARKER_SCAN_MS) {
    SensorState scanSensors = readSensors();
    trackStationMarkers(scanSensors);

    if (!allSensorsWhite(scanSensors) &&
        (middleSeesBlack(scanSensors) || allSensorsBlack(scanSensors))) {
      runStraight(scanSensors.l1, scanSensors.r1);
      return true;
    }

    delay(2);
  }

  return false;
}

bool isPressedStable(unsigned long stableMs) {
  if (digitalRead(PIN_BUTTON) != BTN_PRESSED) {
    return false;
  }

  unsigned long start = millis();
  while (millis() - start < stableMs) {
    if (digitalRead(PIN_BUTTON) != BTN_PRESSED) {
      return false;
    }
    delay(1);
  }
  return true;
}

void waitForButtonRelease() {
  while (digitalRead(PIN_BUTTON) == BTN_PRESSED) {
    delay(5);
  }
}

void waitForStartButton() {
  Serial.println("[BOOT] waiting button release...");
  waitForButtonRelease();

  Serial.println("[BOOT] waiting button press...");
  while (!isPressedStable(BTN_DEBOUNCE_MS)) {
    delay(5);
  }

  waitForButtonRelease();

  Serial.println("[BOOT] button confirmed, start");
}

void handleButtonToggle() {
  if (!isPressedStable(BTN_DEBOUNCE_MS)) {
    return;
  }

  waitForButtonRelease();

  systemStarted = !systemStarted;

  if (systemStarted) {
    Serial.println("System started");
    resetStationHandling();
    printCurrentDirection();
  } else {
    Serial.println("System stopped");
    stopMoving();
  }
  resetTrackingState();

  delay(100);
}

void resetTrackingState() {
  pendingTurn = NONE;
  pendingTurnSeenAt = 0;
  turnLockedUntilMiddleBlack = false;
  lastInnerError = 0;
  resetMarkerTracking();
}

void resetMarkerTracking() {
  lastPatternCode = -1;
  markerHistoryCount = 0;
}

void resetStationHandling() {
  for (int i = 0; i <= 5; i++) {
    stationHandled[i] = false;
  }
  lastConfirmedStation = 0;
  travelDir = DIR_UNKNOWN;
}

void beginNewLeg(int startStation) {
  for (int i = 0; i <= 5; i++) {
    stationHandled[i] = false;
  }

  if (startStation >= 1 && startStation <= 5) {
    stationHandled[startStation] = true;
  }

  Serial.print("进入新轮次，当前起点站: ");
  Serial.println(startStation);
}

void runStraight(int sL1, int sR1) {
  if (sL1 == sR1) {
    moveForward();
    lastInnerError = 0;
    return;
  }

  if (sL1 == BLACK) {
    turnLeftSoft();
    lastInnerError = -1;
    return;
  }

  turnRightSoft();
  lastInnerError = 1;
}

void smoothTurnInPlace(int turnMs, TurnDir dir) {
  int target = (dir == LEFT) ? 90 - TURN_POWER : 90 + TURN_POWER;

  rampToAngles(target, target);

  delay(turnMs);

  rampToAngles(90, 90);

  stopMoving();
  delay(50);
}

void rampToAngles(int targetLeft, int targetRight) {
  int currentLeft = leftServo.read();
  int currentRight = rightServo.read();

  for (int i = 1; i <= RAMP_STEPS; i++) {
    int nextLeft = currentLeft + (targetLeft - currentLeft) * i / RAMP_STEPS;
    int nextRight = currentRight + (targetRight - currentRight) * i / RAMP_STEPS;
    setDrive(nextLeft, nextRight);
    delay(RAMP_STEP_MS);
  }
}

TurnDir chooseUTurnDir() {
  if (lastInnerError < 0) return LEFT;
  if (lastInnerError > 0) return RIGHT;
  return RIGHT;
}

void triggerActionServo() {
  Serial.println("Action servo triggered!");

  stopMoving();

  for (int pos = actionStartAngle; pos <= actionTargetAngle; pos++) {
    actionServo.write(pos);
    delay(actionStepDelay);
  }

  delay(actionStayTime);

  for (int pos = actionTargetAngle; pos >= actionStartAngle; pos--) {
    actionServo.write(pos);
    delay(actionStepDelay);
  }

  stopMoving();
}

void setDrive(int leftAngle, int rightAngle) {
  leftServo.write(leftAngle);
  rightServo.write(rightAngle);
}

void stopMoving() {
  setDrive(90, 90);
}

void moveForward() {
  setDrive(180, 0);
}

void moveForwardSlow() {
  setDrive(SEARCH_LEFT_SPEED, SEARCH_RIGHT_SPEED);
}

void turnLeftSoft() {
  setDrive(90, 0);
}

void turnRightSoft() {
  setDrive(180, 90);
}