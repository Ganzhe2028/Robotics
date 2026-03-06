//v1.3

#include <Servo.h>
Servo leftServo;
Servo rightServo;
Servo topServo;

//Pin
const int S1_PIN = 10;//left most
const int S2_PIN = 11;//middle left
const int S3_PIN = 9;//middle right
const int S4_PIN = 8;//right most
const int SERVO_LEFT_PIN = 4;//left wheel
const int SERVO_RIGHT_PIN = 5;//right wheel
const int BUTTON_PIN = 6;//start buttom
const int SERVO_TOP_PIN = 7;//top servo (for unload)
//Sensor Value
int S1_value;
int S2_value;
int S3_value;
int S4_value;
String SensorValue = "";

void setup() {
  //Motor
  leftServo.attach(SERVO_LEFT_PIN);
  rightServo.attach(SERVO_RIGHT_PIN);
  stop();
  //IR sensor
  pinMode(S1_PIN,INPUT);
  pinMode(S2_PIN,INPUT);
  pinMode(S3_PIN,INPUT);
  pinMode(S4_PIN,INPUT);
  //Serial
  Serial.begin(9600);
  //unload servo
  topServo.attach(SERVO_TOP_PIN);
  topServo.write(0);//change the degree
  //Start buttom
  while (digitalRead(BUTTON_PIN)==1){}//wait for the button to be pressed
}

void loop() {
  read_sensor();
  Serial.print(SensorValue);
  if (S1_value == 1 && S4_value == 1){
    //following a straight line
    if (S2_value == 1 && S3_value == 1){
      //all white! 1111
      backward();
    }
    else if (S2_value == 0 && S3_value == 0){
      //on the straight black line 1001
      forward();
    }
    else if (S2_value == 1){//1101
      arc_right();
    }
    else if (S3_value == 1){//1011
      arc_left();
    }
  }
  else{
    //have a 90 turn
    if (S1_value == 0 && S4_value == 0){
      //0000, all black
      stop();
      delay(500);
      unload();
      delay(500);
      turn_round();
      delay(500);
    }
    else if (S1_value == 0){
      //0001
      arc_left();
      delay(150);
      while (SensorValue != "1001"){
        read_sensor();
      }
    }
    else if (S4_value == 0){
      //1000
      arc_right();
      delay(150);
      while (SensorValue != "1001"){
        read_sensor();
      }
    }
  }
 
  Serial.println();
  delay(100);
}

void unload(){
  for (int i = 0; i < 90; i++){
    topServo.write(i);
    delay(20);
  }
  delay(2000);
  for (int i = 90; i > 0; i--){
    topServo.write(i);
    delay(20);
  }
}

void read_sensor(){
  S1_value = digitalRead(S1_PIN);
  S2_value = digitalRead(S2_PIN);
  S3_value = digitalRead(S3_PIN);
  S4_value = digitalRead(S4_PIN);
  SensorValue = "";
  SensorValue += S1_value;
  SensorValue += S2_value;
  SensorValue += S3_value;
  SensorValue += S4_value;
}

void stop() {
  leftServo.write(90);
  rightServo.write(90);
}
void forward(){
  leftServo.write(0);
  rightServo.write(180);
}
void backward(){
  leftServo.write(180);
  rightServo.write(0);
}
void arc_left(){
  leftServo.write(0);
  rightServo.write(90);
}
void arc_right(){
  leftServo.write(90);
  rightServo.write(180);
}
void turn_round(){
  leftServo.write(0);
  rightServo.write(0);
  delay(1000);
  while (SensorValue != "1001"){
    read_sensor();
  }
  stop();
}