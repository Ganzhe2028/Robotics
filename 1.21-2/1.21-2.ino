/*
 * 4-Sensor Line Following Robot
 * 基于提供的伪代码文档编写
 * * 硬件连接 (Pin Assignments):
 * - S1 (最左侧传感器): Pin 2
 * - S2 (左中传感器):   Pin 3
 * - S3 (右中传感器):   Pin 4
 * - S4 (最右侧传感器): Pin 5
 * - Left Servo:       Pin 9
 * - Right Servo:      Pin 10
 */

#include <Servo.h>

// ==========================
// 1. 常量定义 (CONSTANTS)
// ==========================

// 传感器引脚
const int PIN_S1 = 8;
const int PIN_S2 = 9;
const int PIN_S3 = 10;
const int PIN_S4 = 11;

// 舵机引脚
const int PIN_LEFT_SERVO = 5;
const int PIN_RIGHT_SERVO = 4;

// 电机速度值 (Motor Speeds from Image 1)
// 注意：基于文档，0=全速顺时针, 90=停止, 180=全速逆时针
const int SPEED_GO_FORWARD = 80;  // 前进速度值
const int SPEED_TURN_LEFT  = 70;  // 文档定义的"turnLeft"数值 (用于大转弯时的快速侧)
const int SPEED_TURN_RIGHT = 90;  // 文档定义的"turnRight"数值
const int SPEED_STOP       = 90;  // 停止

// 创建舵机对象
Servo leftServo;
Servo rightServo;

// ==========================
// 2. 初始化 (SETUP)
// ==========================
void setup() {
  // 设置传感器为输入模式
  pinMode(PIN_S1, INPUT);
  pinMode(PIN_S2, INPUT);
  pinMode(PIN_S3, INPUT);
  pinMode(PIN_S4, INPUT);

  // 连接舵机
  leftServo.attach(PIN_LEFT_SERVO);
  rightServo.attach(PIN_RIGHT_SERVO);

  // 初始状态停止电机
  stopMotors();

  // 启动前等待 2 秒 (Wait 2 seconds before starting)
  delay(2000);
}

// ==========================
// 3. 主循环 (MAIN LOOP)
// ==========================
void loop() {
  // STEP 1: 读取所有传感器 (Read all four sensors)
  int s1 = digitalRead(PIN_S1);
  int s2 = digitalRead(PIN_S2);
  int s3 = digitalRead(PIN_S3);
  int s4 = digitalRead(PIN_S4);

  // STEP 2: 基于传感器读数的决策逻辑 (Decision making)

  // CONDITION 1: 线在中间 -> 直行
  // 逻辑: (S2, S3 都在线上) OR (S2 在线, S3 离线) OR (S2 离线, S3 在线)
  if ( (s2 == 1 && s3 == 1) || (s2 == 1 && s3 == 0) || (s2 == 0 && s3 == 1) ) {
    goStraight();
  }
  // CONDITION 2: 线在左侧 -> 小左转
  // 逻辑: 只要 S1 触发
  else if (s1 == 1) {
    turnLeftSlow();
  }
  // CONDITION 3: 线在右侧 -> 小右转
  // 逻辑: 只要 S4 触发
  else if (s4 == 1) {
    turnRightSlow();
  }
  // CONDITION 4: 没有传感器看到线 -> 处理 90 度急转弯
  // 逻辑: 所有传感器都为 0 (ELSE 情况)
  else {
    handle90DegreeTurn();
  }
}

// ==========================
// 4. 运动控制函数 (FUNCTIONS)
// ==========================

// 直行 (goStraight)
void goStraight() {
  leftServo.write(SPEED_GO_FORWARD);  // 80
  rightServo.write(SPEED_GO_FORWARD); // 80
}

// 停止电机 (Helper function)
void stopMotors() {
  leftServo.write(SPEED_STOP);
  rightServo.write(SPEED_STOP);
}

// 缓慢左转 (turnLeftSlow)
// 文档逻辑: 左轮停 (90), 右轮走 (80)
void turnLeftSlow() {
  leftServo.write(SPEED_STOP);        // 90
  rightServo.write(SPEED_GO_FORWARD); // 80
}

// 缓慢右转 (turnRightSlow)
// 文档逻辑: 左轮走 (80), 右轮停 (90)
void turnRightSlow() {
  leftServo.write(SPEED_GO_FORWARD);  // 80
  rightServo.write(SPEED_STOP);       // 90
}

// 左旋 90 度 (spinLeft90)
// 文档 Image 4: Left=70, Right=90, Wait 500ms
void spinLeft90() {
  leftServo.write(SPEED_TURN_LEFT);   // 70
  rightServo.write(SPEED_TURN_RIGHT); // 90
  delay(500);
  
  stopMotors();
  delay(100);
}

// 右旋 90 度 (spinRight90)
// 文档 Image 5: Left=90, Right=70, Wait 500ms
void spinRight90() {
  leftServo.write(SPEED_TURN_RIGHT);  // 90
  rightServo.write(SPEED_TURN_LEFT);  // 70
  delay(500);

  stopMotors();
  delay(100);
}

// ==========================
// 5. 复杂逻辑处理
// ==========================

// 处理 90 度转弯逻辑 (handle90DegreeTurn)
void handle90DegreeTurn() {
  // 1. 停止电机并等待 200ms
  stopMotors();
  delay(200);

  // 2. 再次读取所有传感器
  int s1 = digitalRead(PIN_S1);
  int s2 = digitalRead(PIN_S2);
  int s3 = digitalRead(PIN_S3);
  int s4 = digitalRead(PIN_S4);

  // 3. 判断转弯方向
  // 逻辑: 如果左侧传感器 (S1 或 S2) 看到线 -> 左旋
  if (s1 == 1 || s2 == 1) {
    spinLeft90();
  }
  // 逻辑: 否则如果右侧传感器 (S3 或 S4) 看到线 -> 右旋
  else if (s3 == 1 || s4 == 1) {
    spinRight90();
  }
  // 默认动作 (ELSE) -> 右旋
  else {
    spinRight90();
  }
}