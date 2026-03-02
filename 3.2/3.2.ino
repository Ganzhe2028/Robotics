#include <Servo.h>

Servo myservo;  // create Servo object to control a servo
// twelve Servo objects can be created on most boards

int pos = 0;

void setup() {
  // put your setup code here, to run once:
  myservo.attach(7);

  int btnsw = 6;

  Serial.begin(9600);
  pinMode(btnsw, INPUT);

  // wait for pressing
  while (digitalRead(btnsw) == 1){}
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Btn triggered!");
  // Serial.print();
  // Serial.println();

  while (true) {


    for (pos = 0; pos <= 50; pos += 1) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      myservo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(10);                       // waits 15 ms for the servo to reach the position
    }

    delay(500);

    for (pos = 50; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
      myservo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(10);                       // waits 15 ms for the servo to reach the position
    }

    delay(1000);

  }
}
