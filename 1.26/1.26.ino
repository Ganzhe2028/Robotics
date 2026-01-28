#include <Servo.h>

// ================= 硬件定义 =================


// 舵机对象
Servo leftServo;
Servo rightServo;

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

  // 2. 巡线逻辑决策
  // 传感器状态: {sL2, sL1, sR1, sR2} = {最左, 中左, 中右, 最右}
  // 优先级：先判断急转(最外侧)，再判断直行，最后判断微调

  // ========== 优先级 1: 直角与急转 ==========
  
  // 1.1 直角/锐角逻辑 (0111 / 1110) -> 调用封装好的函数
  if (sL2 == BLACK && sL1 == WHITE && sR1 == WHITE && sR2 == WHITE) {
     // {0,1,1,1} -> 左转检测
     checkIntersectionAndTurnLeft();
  }

  else if (sL2 == WHITE && sL1 == WHITE && sR1 == WHITE && sR2 == BLACK) {
     // {1,1,1,0} -> 右转检测
     checkIntersectionAndTurnRight();
  }

  
  // 1.2 其他急转情况 (如 0011, 0001) -> 原地旋转 (Hard Turn)
  // 恢复这部分逻辑以处理大幅度偏离

  else if (sL2 == BLACK && sR2 == WHITE) {
    // 左侧大幅度偏离或左侧多传感器检测到黑线
    checkIntersectionAndTurnLeft();
  }
  else if (sL2 == WHITE && sR2 == BLACK) {
    // 右侧大幅度偏离或右侧多传感器检测到黑线
    checkIntersectionAndTurnRight();
  }

  // ========== 优先级 2: 正常轨道范围 (两个最外侧都是白色) ==========
  // {1,X,X,1} 说明在正常行驶区域，根据中间两个传感器决定动作
  
  else if (sL2 == WHITE && sR2 == WHITE) {
    
    if (sL1 == BLACK && sR1 == BLACK) {
      // {1,0,0,1} 中间两个都在黑线上 -> 完美居中，直行
      moveForward();
    }
    else if (sL1 == BLACK && sR1 == WHITE) {
      // {1,0,1,1} 中左=黑, 中右=白 -> 稍微偏右，向左微调
      turnLeftSoft();
      
    }
    else if (sL1 == WHITE && sR1 == BLACK) {
      // {1,1,0,1} 中左=白, 中右=黑 -> 稍微偏左，向右微调
      turnRightSoft();
  
    }
    else {
      // {1,1,1,1} 全白 -> 走到尽头，执行掉头逻辑
      // 保持之前的状态 (通常是直行) 继续走 250ms，确保完全脱离
      delay(250);
      
      // 左转掉头 (U-turn) 带传感器检测中断
      turnLeftHard();
      
      unsigned long startTime = millis();
      // 执行掉头，最长 1800ms，检测到中间传感器压线立刻停止
      while (millis() - startTime < 1800) {
        // 读取中间传感器
        int checkL1 = digitalRead(PIN_SENSOR_L1);
        int checkR1 = digitalRead(PIN_SENSOR_R1);
        
        // 如果中间两个传感器检测到黑线，说明已经转正，跳出循环
        if (checkL1 == BLACK || checkR1 == BLACK) {
           break;
        }
        delay(1); // 避免死循环占用过多资源
      }
    }
  }
  
  // ========== 优先级 3: 特殊情况 ==========
  
  // else if (sL2 == BLACK && sR2 == BLACK) {
  //   // {0,X,X,0} 两个最外侧都是黑色
  //   // 可能情况：1) 交叉路口 2) 特殊标记 3) 宽黑线
  //   // 暂时保持直行，你可以根据实际情况调整
  //   moveForward();
  // }
  
  else {
    // 其他未预期情况，停车
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

// ================= 新增：路口检测与转弯逻辑 =================

// 检测到左转信号后：前行0.5秒 -> 检测是否过线 -> 确实过线则左急转
void checkIntersectionAndTurnLeft() {
  moveForward();
  delay(500); // 按要求改为 0.5 秒
  
  // 重新读取传感器
  int sL2 = digitalRead(PIN_SENSOR_L2);
  int sL1 = digitalRead(PIN_SENSOR_L1);
  int sR1 = digitalRead(PIN_SENSOR_R1);
  int sR2 = digitalRead(PIN_SENSOR_R2);
  
  // 如果全是白色，说明已经过了黑线交叉口，执行急转
  if (sL2 == WHITE && sL1 == WHITE && sR1 == WHITE && sR2 == WHITE) {
    turnLeftHard();
    delay(800);
  }
}

// 检测到右转信号后：前行0.5秒 -> 检测是否过线 -> 确实过线则右急转
void checkIntersectionAndTurnRight() {
  moveForward();
  delay(500); // 按要求改为 0.5 秒
  
  // 重新读取传感器
  int sL2 = digitalRead(PIN_SENSOR_L2);
  int sL1 = digitalRead(PIN_SENSOR_L1);
  int sR1 = digitalRead(PIN_SENSOR_R1);
  int sR2 = digitalRead(PIN_SENSOR_R2);
  
  // 如果全是白色，说明已经过了黑线交叉口，执行急转
  if (sL2 == WHITE && sL1 == WHITE && sR1 == WHITE && sR2 == WHITE) {
    turnRightHard();
    delay(800);
  }
}
