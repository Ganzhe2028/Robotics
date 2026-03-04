#include <Servo.h>

// ================= 硬件定义 =================


// 舵机对象
Servo leftServo;
Servo rightServo;
Servo myservo;

// 传感器引脚 (根据你的图片和上一段代码)
// 假设从左到右依次连接到 8, 9, 10, 11
const int PIN_SENSOR_L2 = 10;   // 最左 (大幅度偏离)
const int PIN_SENSOR_L1 = 11;   // 左中 (用于微调)
const int PIN_SENSOR_R1 = 9;  // 右中 (用于微调)
const int PIN_SENSOR_R2 = 8;  // 最右 (大幅度偏离)

// 舵机引脚
const int PIN_SERVO_LEFT = 5;
const int PIN_SERVO_RIGHT = 4;

// 逻辑常量 (根据实际测试结果)
const int BLACK = LOW;   // 检测到黑线输出 0
const int WHITE = HIGH;  // 检测到白色地面输出 1

int pos = 0; //顶部舵机角度
const int BTN_SW_PIN = 6;

const unsigned long LINE_END_CONFIRM_MS = 250;
const unsigned long TURN_FORWARD_MS = 500;
const unsigned long TURN_FIND_LINE_TIMEOUT_MS = 1200;
const unsigned long UTURN_TIMEOUT_MS = 1800;

// 只有先进入直线中心状态(1001)，后续才允许判定“尽头卸货”或“直角转弯”
bool straightLineArmed = false;

bool isAllWhitePattern(int sL2, int sL1, int sR1, int sR2);
bool confirmLineEnd();
void performUnloadAndUTurn();
void rotateUntilLineFound(bool turnLeft);






// ================= SETUP =================
void setup() {
  Serial.begin(9600);  // 用于调试

  // 配置传感器
  pinMode(PIN_SENSOR_L2, INPUT);
  pinMode(PIN_SENSOR_L1, INPUT);
  pinMode(PIN_SENSOR_R1, INPUT);
  pinMode(PIN_SENSOR_R2, INPUT);

  // 配置舵机
  leftServo.attach(PIN_SERVO_LEFT);
  rightServo.attach(PIN_SERVO_RIGHT);

  // 启动时先停一下，给你1秒钟反应时间
  // stopMoving();
  // delay(1000);

  myservo.attach(7);

  pinMode(BTN_SW_PIN, INPUT);

  // wait for pressing
  while (digitalRead(BTN_SW_PIN) == 1){}
}






// ================= LOOP =================

void loop() {
  // 1. 读取传感器
  int sL2 = digitalRead(PIN_SENSOR_L2);  // 最左
  int sL1 = digitalRead(PIN_SENSOR_L1);  // 左中
  int sR1 = digitalRead(PIN_SENSOR_R1);  // 右中
  int sR2 = digitalRead(PIN_SENSOR_R2);  // 最右

  // 调试输出 - 打印传感器状态
  Serial.print("  ");
  Serial.print(sL2);
  Serial.print("  ");
  Serial.print(sL1);
  Serial.print("  ");
  Serial.print(sR1);
  Serial.print("  ");
  Serial.print(sR2);
  Serial.println();

  bool isAllWhite = isAllWhitePattern(sL2, sL1, sR1, sR2);
  bool isCenterLine = (sL2 == WHITE && sL1 == BLACK && sR1 == BLACK && sR2 == WHITE); // 1001
  bool isLeftTurnSignal = (sL2 == BLACK && sR2 == WHITE);
  bool isRightTurnSignal = (sL2 == WHITE && sR2 == BLACK);
  bool isNormalTrackRange = (sL2 == WHITE && sR2 == WHITE);

  if (isCenterLine) {
    straightLineArmed = true;
  }

  // 先从直线状态分流：要么进直角转弯，要么进尽头卸货
  if (straightLineArmed && isLeftTurnSignal) {
    straightLineArmed = false;
    checkIntersectionAndTurnLeft();
    delay(10);
    return;
  }

  if (straightLineArmed && isRightTurnSignal) {
    straightLineArmed = false;
    checkIntersectionAndTurnRight();
    delay(10);
    return;
  }

  if (straightLineArmed && isAllWhite) {
    if (confirmLineEnd()) {
      straightLineArmed = false;
      performUnloadAndUTurn();
      delay(10);
      return;
    }
  }

  // 常规循迹
  if (isLeftTurnSignal) {
    checkIntersectionAndTurnLeft();
  } else if (isRightTurnSignal) {
    checkIntersectionAndTurnRight();
  } else if (isNormalTrackRange) {
    if (sL1 == BLACK && sR1 == BLACK) {
      moveForward();
    } else if (sL1 == BLACK && sR1 == WHITE) {
      turnLeftSoft();
    } else if (sL1 == WHITE && sR1 == BLACK) {
      turnRightSoft();
    } else if (isAllWhite) {
      // 未命中“直线->全白”卸货条件时，全白只做短时前探，不触发卸货
      moveForward();
    } else {
      stopMoving();
    }
  } else {
    stopMoving();
  }

  // 小延时增加稳定性
  delay(10);
}

// ================= 动作控制函数 =================
// 基于你的测试数据：左180=前，右0=前

void stopMoving() {
  leftServo.write(90);
  rightServo.write(90);
}

void moveForward() {
  leftServo.write(180);
  rightServo.write(0);
}

// 左转 - 柔和 (单轮转动)
// 左轮停，右轮走 -> 比较平滑的转弯
void turnLeftSoft() {
  leftServo.write(90);  // 左轮停
  rightServo.write(0);  // 右轮前
}

// 右转 - 柔和 (单轮转动)
// 左轮走，右轮停
void turnRightSoft() {
  leftServo.write(180);  // 左轮前
  rightServo.write(90);  // 右轮停
}

// 左转 - 急转 (原地旋转)
// 对应你原来的 "righting" 逻辑 (左退右进)
void turnLeftHard() {
  leftServo.write(0);   // 左轮后退
  rightServo.write(0);  // 右轮前进
}

// 右转 - 急转 (原地旋转)
// 对应你原来的 "lefting" 逻辑 (左进右退)
void turnRightHard() {
  leftServo.write(180);   // 左轮前进
  rightServo.write(180);  // 右轮后退
}

bool isAllWhitePattern(int sL2, int sL1, int sR1, int sR2) {
  return (sL2 == WHITE && sL1 == WHITE && sR1 == WHITE && sR2 == WHITE);
}

bool confirmLineEnd() {
  // 从直线中心状态冲出去一小段，再二次确认是否仍全白
  moveForward();
  delay(LINE_END_CONFIRM_MS);

  int sL2 = digitalRead(PIN_SENSOR_L2);
  int sL1 = digitalRead(PIN_SENSOR_L1);
  int sR1 = digitalRead(PIN_SENSOR_R1);
  int sR2 = digitalRead(PIN_SENSOR_R2);

  return isAllWhitePattern(sL2, sL1, sR1, sR2);
}

void performUnloadAndUTurn() {
  stopMoving();

  for (pos = 0; pos <= 50; pos += 1) {
    myservo.write(pos);
    delay(10);
  }

  delay(500);

  for (pos = 50; pos >= 0; pos -= 1) {
    myservo.write(pos);
    delay(10);
  }

  delay(250);

  // 卸货后左转掉头，直到重新压到黑线或超时
  turnLeftHard();
  unsigned long startTime = millis();
  while (millis() - startTime < UTURN_TIMEOUT_MS) {
    int checkL1 = digitalRead(PIN_SENSOR_L1);
    int checkR1 = digitalRead(PIN_SENSOR_R1);
    if (checkL1 == BLACK || checkR1 == BLACK) {
      break;
    }
    delay(1);
  }
}

void rotateUntilLineFound(bool turnLeft) {
  unsigned long startTime = millis();
  while (millis() - startTime < TURN_FIND_LINE_TIMEOUT_MS) {
    if (turnLeft) {
      turnLeftHard();
    } else {
      turnRightHard();
    }

    int sL1 = digitalRead(PIN_SENSOR_L1);
    int sR1 = digitalRead(PIN_SENSOR_R1);
    if (sL1 == BLACK || sR1 == BLACK) {
      break;
    }
    delay(1);
  }
}

// ================= 新增：路口检测与转弯逻辑 =================

// 检测到左转信号后：前行0.5秒 -> 检测是否过线 -> 确实过线则左急转
void checkIntersectionAndTurnLeft() {
  moveForward();
  delay(TURN_FORWARD_MS);
  
  // 重新读取传感器
  int sL2 = digitalRead(PIN_SENSOR_L2);
  int sL1 = digitalRead(PIN_SENSOR_L1);
  int sR1 = digitalRead(PIN_SENSOR_R1);
  int sR2 = digitalRead(PIN_SENSOR_R2);
  
  // 如果全白，说明已冲过拐角，开始急转直到重新找到黑线
  if (isAllWhitePattern(sL2, sL1, sR1, sR2)) {
    rotateUntilLineFound(true);
  }
}

// 检测到右转信号后：前行0.5秒 -> 检测是否过线 -> 确实过线则右急转
void checkIntersectionAndTurnRight() {
  moveForward();
  delay(TURN_FORWARD_MS);
  
  // 重新读取传感器
  int sL2 = digitalRead(PIN_SENSOR_L2);
  int sL1 = digitalRead(PIN_SENSOR_L1);
  int sR1 = digitalRead(PIN_SENSOR_R1);
  int sR2 = digitalRead(PIN_SENSOR_R2);
  
  // 如果全白，说明已冲过拐角，开始急转直到重新找到黑线
  if (isAllWhitePattern(sL2, sL1, sR1, sR2)) {
    rotateUntilLineFound(false);
  }
}
