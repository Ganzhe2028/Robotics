#include <Servo.h>

Servo myservo;  // create Servo object to control a servo
// twelve Servo objects can be created on most boards

int pos = 0;

void setup() {
  // put your setup code here, to run once:
  myservo.attach(9);

  int btnsw = 6;

  Serial.begin(9600);
  pinMode(btnsw, INPUT);

  // wait for pressing
  while (digitalRead(btnsw) == 0){}
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("Btn stats: ");
  Serial.println();
}
