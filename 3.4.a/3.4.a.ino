#include <Servo.h>

Servo myservo;  // create Servo object to control a servo
// twelve Servo objects can be created on most boards

int pos = 0;
const int BTN_SW_PIN = 6;
const int BTN_PRESSED_LEVEL = LOW;  // most 3-wire button modules: pressed = LOW



void setup() {
  // put your setup code here, to run once:
  myservo.attach(7);

  Serial.begin(9600);
  pinMode(BTN_SW_PIN, INPUT);

  while (digitalRead(BTN_SW_PIN) != BTN_PRESSED_LEVEL) {}
  delay(20);  // debounce
  while (digitalRead(BTN_SW_PIN) == BTN_PRESSED_LEVEL) {}

  Serial.println("Btn triggered!");
}

void loop() {
  
  // put your main code here, to run repeatedly:
  myservo.write(pos);              // tell servo to go to position in variable 'pos'
  delay(500);

  pos = 50;

  myservo.write(pos);              // tell servo to go to position in variable 'pos'
  delay(5000);
}
