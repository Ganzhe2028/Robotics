#include <Servo.h>

Servo leftServo;
Servo rightServo;

const int S1 = 11;
const int S2 = 10;
const int S3 = 9;
const int S4 = 8;

// const int goForward = 80;
// const int turnLeft = 70;
// const int turnRight = 90;
// const int stopMoving = 90;

const int BLACK = LOW;
const int WHITE = HIGH;


// -------------------------------------------------------------------

// lestServo: 0-前进
// rightServo: 180-前进


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);

  leftServo.attach(4);
  rightServo.attach(5);

  stopMoving();
  delay(500);
}


// ------------------------------------------------------

// 0: 白（地面）
// 1: 黑（线）

void loop() {
  // put your main code here, to run repeatedly:
  int s1 = digitalRead(S1);
  int s2 = digitalRead(S2);
  int s3 = digitalRead(S3);
  int s4 = digitalRead(S4);

  // condition making
  // Priority 1: Center - Go Forward
  if (s2 == 1 && s3 == 1) {
    goForward();
  } 
  // Priority 2: Slight Deviations (Line under one middle sensor)
  else if (s2 == 1) {
    // Line on S2 (Left of center) -> Turn Left
    turnLeftSlow();
  } else if (s3 == 1) {
    // Line on S3 (Right of center) -> Turn Right
    turnRightSlow();
  } 
  // Priority 3: Sharp Deviations (Line under outer sensors)
  else if (s1 == 1) {
    turnLeftSlow(); // Or spinLeft90 for sharper turn
  } else if (s4 == 1) {
    turnRightSlow(); // Or spinRight90 for sharper turn
  } 
  // Priority 4: Lost Line / Intersection (0000)
  else {
    handle90DegreeTurn();
  }
}


// ------------------------------------------------

void handle90DegreeTurn() {
  leftServo.write(90);
  rightServo.write(90);
  delay(200);

  int s1 = digitalRead(S1);
  int s2 = digitalRead(S2);
  int s3 = digitalRead(S3);
  int s4 = digitalRead(S4);

  if (s1 == 1 || s2 == 1) {

    spinLeft90();
  } else if (s3 == 1 || s4 == 1) {
    spinRight90();
  } else {
    spinRight90();
  }
}


// -------


void goForward() {
  leftServo.write(0);
  rightServo.write(180);
}

void turnLeftSlow() {
  leftServo.write(90);
  rightServo.write(110);
}

void turnRightSlow() {
  leftServo.write(70);
  rightServo.write(90);
}

void spinLeft90() {
  leftServo.write(90);
  rightServo.write(110);
  delay(500);
  leftServo.write(90);
  rightServo.write(90);
  delay(100);
}

void spinRight90() {
  leftServo.write(70);
  rightServo.write(90);
  delay(500);
  leftServo.write(90);
  rightServo.write(90);
  delay(100);
}

void stopMoving() {
  leftServo.write(90);
  rightServo.write(90);
}