#include <Servo.h>

Servo liftArm;
 
void setup() {
  // put your setup code here, to run once:
  liftArm.attach(9);

  liftArm.write(0);
}

void loop() {
  // put your main code here, to run repeatedly:
  for (int i = 0; i < 100; i++) {
    liftArm.write(i);
    delay(25);
  }

  for (int i = 99; i >= 0; i--) {
    liftArm.write(i);
    delay(25);
  }

  // liftArm.write(90);
  // delay(2000);
  // liftArm.write(0);
  // delay(2000);
}
