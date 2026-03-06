#include <Servo.h>

// ===== Pin Map =====
// Sensor pins (L2, L1 are left; R1, R2 are right)
#define PIN_SENSOR_L2 10
#define PIN_SENSOR_L1 11
#define PIN_SENSOR_R1 9
#define PIN_SENSOR_R2 8

// Servo motor pins
#define PIN_SERVO_LEFT 5
#define PIN_SERVO_RIGHT 4
#define PIN_SERVO_DUMP 7 // Servo for the dumping mechanism

// Button pin for starting the robot
#define PIN_BTN_SW 6

// ===== Logic Level =====
// Logical definitions for sensor readings and button state
#define BLACK LOW
#define WHITE HIGH
#define BTN_PRESSED LOW

// ===== Tunables =====
// Timing constants for various operations (in milliseconds)
const unsigned long LOOP_DELAY_MS = 10;                     // Delay at the end of each main loop iteration
const unsigned long LINE_END_CONFIRM_MS = 250;              // Time to move forward to confirm line end
const unsigned long TURN_FORWARD_MS = 500;                  // Time to move forward before turning at a right angle
const unsigned long TURN_FIND_LINE_TIMEOUT_MS = 1200;       // Max time to rotate while searching for a line
const unsigned long UTURN_TIMEOUT_MS = 1800;                // Max time allowed for a U-turn operation
const unsigned long BTN_DEBOUNCE_MS = 30;                   // Debounce period to ensure stable button press readings

// delay(200); // WARNING: This is in global scope and may cause compilation errors in standard C++/Arduino

// Servo angles for the dumping mechanism
const int DUMP_HOME_ANGLE = 0;                              // Resting/home position angle for dump servo
const int DUMP_RELEASE_ANGLE = 50;                          // Angle required to release/dump the load

// Servo control objects
Servo leftServo;
Servo rightServo;
Servo dumpServo;

// State flags
bool straightLineArmed = false;                             // Flag to indicate if the robot is currently tracking a straight center line
bool dumpServoAttached = false;                             // Flag to track whether the dump servo is currently attached to save power/reduce jitter

// Structure to hold readings from the four line tracking sensors
struct Sensors {
  int l2; // Far left
  int l1; // Inner left
  int r1; // Inner right
  int r2; // Far right
};

// Reads all four line sensors and returns their states
Sensors readSensors() {
  Sensors s;
  s.l2 = digitalRead(PIN_SENSOR_L2);
  s.l1 = digitalRead(PIN_SENSOR_L1);
  s.r1 = digitalRead(PIN_SENSOR_R1);
  s.r2 = digitalRead(PIN_SENSOR_R2);
  return s;
}

// Checks if all sensors detect white (e.g., crossing a horizontal line or end of path)
bool isAllWhite(const Sensors &s) {
  return s.l2 == WHITE && s.l1 == WHITE && s.r1 == WHITE && s.r2 == WHITE;
}

// Checks if the robot is perfectly centered on the line (inner sensors black, outer white)
bool isCenterLine(const Sensors &s) {
  // 1001 binary representation concept: outer sensors see white (1), inner see black (0)
  return s.l2 == WHITE && s.l1 == BLACK && s.r1 == BLACK && s.r2 == WHITE; 
}

// Checks for a left turn marker/signal (left outer sensor detecting black, right outer white)
bool isLeftTurnSignal(const Sensors &s) {
  return s.l2 == BLACK && s.r2 == WHITE;
}

// Checks for a right turn marker/signal (right outer sensor detecting black, left outer white)
bool isRightTurnSignal(const Sensors &s) {
  return s.l2 == WHITE && s.r2 == BLACK;
}

// ===== Movement Control Functions =====
// Since continuous rotation servos are used, 90 is stop, 0 is full reverse, 180 is full forward.
// Note: right servo is mounted opposite to the left servo, so writing 0 to right servo means forward.

void stopMoving() {
  leftServo.write(90);
  rightServo.write(90);
}

void moveForward() {
  leftServo.write(180);
  rightServo.write(0);
}

// Soft turns: One wheel stops while the other moves forward
void turnLeftSoft() {
  leftServo.write(90);
  rightServo.write(0);
}

void turnRightSoft() {
  leftServo.write(180);
  rightServo.write(90);
}

// Hard turns: Wheels move in opposite directions to rotate on the spot
void turnLeftHard() {
  leftServo.write(0);
  rightServo.write(0);
}

void turnRightHard() {
  leftServo.write(180);
  rightServo.write(180);
}

// Rotates the robot in place until one of the inner sensors detects the line or a timeout occurs
void rotateUntilLineFound(bool turnLeft) {
  unsigned long startTime = millis();
  while (millis() - startTime < TURN_FIND_LINE_TIMEOUT_MS) {
    if (turnLeft) {
      turnLeftHard();
    } else {
      turnRightHard();
    }

    Sensors s = readSensors();
    // Stop rotating as soon as an inner sensor detects the black line
    if (s.l1 == BLACK || s.r1 == BLACK) {
      Serial.println(F("[TURN] line found, exit rotate"));
      break;
    }
    delay(1);
  }
}

// Handles right-angle turns upon detecting a turn signal marker
void handleRightAngleTurn(bool turnLeft) {
  Serial.println(turnLeft ? F("[TURN] left signal") : F("[TURN] right signal"));
  
  // Move forward briefly to align the rotation center with the intersecting line
  moveForward();
  delay(TURN_FORWARD_MS);

  Sensors s = readSensors();
  // If moving forward put us entirely in a white zone, find the line again
  if (isAllWhite(s)) {
    Serial.println(F("[TURN] crossed corner, rotate to find line"));
    rotateUntilLineFound(turnLeft);
  }
}

// Checks to confirm we have reached the genuine end of the line (all white after moving forward)
bool confirmLineEnd() {
  moveForward();
  delay(LINE_END_CONFIRM_MS);
  Sensors s = readSensors();
  return isAllWhite(s);
}

// Attaches the dump servo to its pin only when needed, to save power and prevent jitter
void attachDumpServoIfNeeded() {
  if (!dumpServoAttached) {
    dumpServo.attach(PIN_SERVO_DUMP);
    dumpServoAttached = true;
    Serial.println(F("[DUMP] attach top servo"));
    dumpServo.write(DUMP_HOME_ANGLE);
    delay(120);
  }
}

// Executes the dumping sequence to drop a load and then performs a U-turn to return
void unloadAndUTurn() {
  Serial.println(F("[DUMP] line end confirmed"));
  stopMoving();

  attachDumpServoIfNeeded();

  // Smoothly move the dump servo to the release angle
  for (int pos = DUMP_HOME_ANGLE; pos <= DUMP_RELEASE_ANGLE; pos++) {
    dumpServo.write(pos);
    delay(10);
  }

  delay(500); // Allow time for payload to drop

  // Smoothly return the dump servo to the home angle
  for (int pos = DUMP_RELEASE_ANGLE; pos >= DUMP_HOME_ANGLE; pos--) {
    dumpServo.write(pos);
    delay(10);
  }

  delay(250);

  // Release PWM after unload to reduce interference/jitter.
  dumpServo.detach();
  dumpServoAttached = false;
  Serial.println(F("[DUMP] detach top servo"));

  // Perform a 180-degree hard U-turn until line is found again
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

// Reads the start button and uses a software debounce to ensure the press is stable
bool isPressedStable(unsigned long stableMs) {
  if (digitalRead(PIN_BTN_SW) != BTN_PRESSED) {
    return false;
  }
  unsigned long start = millis();
  while (millis() - start < stableMs) {
    // If the button bounces high again, return false
    if (digitalRead(PIN_BTN_SW) != BTN_PRESSED) {
      return false;
    }
    delay(1);
  }
  return true; // Press held consistently for the entire stableMs duration
}

// Blocks execution until the start button is safely pressed and released
void waitForStartButton() {
  Serial.println(F("[BOOT] waiting button release..."));
  // Ensure the button is fully released before we start waiting for a press
  while (digitalRead(PIN_BTN_SW) == BTN_PRESSED) {
    delay(5);
  }

  Serial.println(F("[BOOT] waiting button press..."));
  // Wait indefinitely for a stable button press event
  while (!isPressedStable(BTN_DEBOUNCE_MS)) {
    delay(5);
  }

  // Wait for the final release of the button
  while (digitalRead(PIN_BTN_SW) == BTN_PRESSED) {
    delay(5);
  }

  Serial.println(F("[BOOT] button confirmed, start"));
}


// ===== Core System Loops =====

void setup() {
  Serial.begin(9600);
  Serial.println(F("[BOOT] init"));

  // Initialize line sensor pins as inputs
  pinMode(PIN_SENSOR_L2, INPUT);
  pinMode(PIN_SENSOR_L1, INPUT);
  pinMode(PIN_SENSOR_R1, INPUT);
  pinMode(PIN_SENSOR_R2, INPUT);

  // Initialize start button pin with internal pullup resistor
  pinMode(PIN_BTN_SW, INPUT_PULLUP);

  // Attach movement servos and command them to stop immediately
  leftServo.attach(PIN_SERVO_LEFT);
  rightServo.attach(PIN_SERVO_RIGHT);
  stopMoving();

  // Keep dump servo detached on boot to avoid startup jump to 90 deg.
  dumpServo.detach();
  dumpServoAttached = false;
  Serial.println(F("[BOOT] dump servo detached"));

  // Wait for the human operator to press the start button
  waitForStartButton();
}

void loop() {
  // Read current line sensor states
  Sensors s = readSensors();

  // Debug print: output sensors + armed state periodically for first-run issue tracing.
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 80) { // Print every ~80ms to avoid serial buffer locking
    lastPrint = millis();
    Serial.print(F("[S] "));
    Serial.print(s.l2); Serial.print(' ');
    Serial.print(s.l1); Serial.print(' ');
    Serial.print(s.r1); Serial.print(' ');
    Serial.print(s.r2); Serial.print(F(" | armed="));
    Serial.println(straightLineArmed ? 1 : 0);
  }

  // Arm straight line tracking if center line format is perfectly matched
  if (isCenterLine(s)) {
    straightLineArmed = true;
  }

  // Split after center-line state: Handle corner turning or line-end sequence.
  // We only execute these commands if we previously confirmed stable straight tracking (armed).
  if (straightLineArmed && isLeftTurnSignal(s)) {
    straightLineArmed = false; // Disarm to prevent re-triggering during turn maneuver
    handleRightAngleTurn(true); // Process the left right-angle turn
    delay(LOOP_DELAY_MS);
    return;
  }
  
  if (straightLineArmed && isRightTurnSignal(s)) {
    straightLineArmed = false; // Disarm
    handleRightAngleTurn(false); // Process the right right-angle turn
    delay(LOOP_DELAY_MS);
    return;
  }
  
  if (straightLineArmed && isAllWhite(s) && confirmLineEnd()) {
    straightLineArmed = false; // Disarm
    unloadAndUTurn(); // Trigger dumping container sequence then flip 180 degrees
    delay(LOOP_DELAY_MS);
    return;
  }

  // Normal proportional-like line tracking logic
  if (isLeftTurnSignal(s)) {
    handleRightAngleTurn(true);
  } else if (isRightTurnSignal(s)) {
    handleRightAngleTurn(false);
  } else if (s.l2 == WHITE && s.r2 == WHITE) { 
    // Outer sensors are safely over white background, adjust heading based on inner sensors
    if (s.l1 == BLACK && s.r1 == BLACK) {
      // Both inner sensors see black: go dead straight
      moveForward();
    } else if (s.l1 == BLACK && s.r1 == WHITE) {
      // Line is biased left, softly correct course to the left
      turnLeftSoft();
    } else if (s.l1 == WHITE && s.r1 == BLACK) {
      // Line is biased right, softly correct course to the right
      turnRightSoft();
    } else if (isAllWhite(s)) {
      // Lost the line completely: cautiously inch forward to see if we can catch it
      moveForward();
    } else {
      // Unknown edge-case state, stop to prevent runaway
      stopMoving();
    }
  } else {
    // If outer sensors trigger unexpectedly outside designated intersections, stop
    stopMoving();
  }

  // End of loop cycle delay
  delay(LOOP_DELAY_MS);
}
