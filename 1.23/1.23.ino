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

// lestServo: 0: 前进 180: 后退
// rightServo: 

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


void loop() {
  int s1 = digitalRead(S1);
  int s2 = digitalRead(S2);
  int s3 = digitalRead(S3);
  int s4 = digitalRead(S4);

  // 理想状态：中间两个传感器在线上 → 直行
  if (s2 == 1 && s3 == 1) {
    goForward();
  }
  // 偏右严重：最左传感器检测到线 → 左转
  else if (s1 == 1) {
    turnLeftSlow();
  }
  // 偏右轻微：仅左中传感器在线 → 左转
  else if (s2 == 1 && s3 == 0) {
    turnLeftSlow();
  }
  // 偏左轻微：仅右中传感器在线 → 右转
  else if (s2 == 0 && s3 == 1) {
    turnRightSlow();
  }
  // 偏左严重：最右传感器检测到线 → 右转
  else if (s4 == 1) {
    turnRightSlow();
  }
  // 所有传感器都未检测到线 → 处理90度转弯或丢线
  else {
    handle90DegreeTurn();
  }
}


// ------------------------------------------------

void handle90DegreeTurn(){
  leftServo.write(90);
  rightServo.write(90);
  delay(200);

  int s1 = digitalRead(S1);
  int s2 = digitalRead(S2);
  int s3 = digitalRead(S3);
  int s4 = digitalRead(S4);

  if (s1==1 || s2==1){
    
    spinLeft90();
  }
  else if (s3==1 || s4==1){
    spinRight90();
  }
  else {
    spinRight90();
  }
}


// -------


void goForward(){
    leftServo.write(0);
    rightServo.write(180);
  }

void turnLeftSlow(){
  leftServo.write(80);
  rightServo.write(110);
}

void turnRightSlow(){
  leftServo.write(70);   // 左轮快速前进 (<90)
  rightServo.write(100); // 右轮慢速前进 (>90但接近90)
}

void spinLeft90(){
  leftServo.write(90);
  rightServo.write(110);
  delay(500);
  leftServo.write(90);
  rightServo.write(90);
  delay(100);
}

void spinRight90(){
  leftServo.write(70);
  rightServo.write(90);
  delay(500);
  leftServo.write(90);
  rightServo.write(90);
  delay(100);
}

void stopMoving(){
  leftServo.write(90);
  rightServo.write(90);
}