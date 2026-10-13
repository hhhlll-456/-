/*
 * 麦轮「占空比 → 轮速」密集标定程序
 * 功能：每个轮正反方向自动扫描多个占空比，计算实际平均轮速 (rad/s)
 * 输出格式可直接复制到 Excel / 查表
 */

#include <Arduino.h>

// ===================== 引脚定义 =====================
const int MOTOR_PINS[4][2] = {
  {13, 15},  // 0: FL
  {12, 14},  // 1: FR
  {27, 26},  // 2: RL
  {25, 33}   // 3: RR
};

const int ENC_PINS[4] = {32, 34, 35, 36};

const char* WHEEL_NAME[4] = {"FL", "FR", "RL", "RR"};

// 单圈计数（你实测值）
const float C1REV[4] = {1276.0f, 1270.0f, 1276.0f, 1274.0f};

// ===================== 编码器 =====================
volatile long encoderCount[4] = {0};

void IRAM_ATTR encISR0() { encoderCount[0]++; }
void IRAM_ATTR encISR1() { encoderCount[1]++; }
void IRAM_ATTR encISR2() { encoderCount[2]++; }
void IRAM_ATTR encISR3() { encoderCount[3]++; }

void setupEncoders() {
  for (int i = 0; i < 4; i++) {
    pinMode(ENC_PINS[i], INPUT_PULLUP);
  }
  attachInterrupt(digitalPinToInterrupt(ENC_PINS[0]), encISR0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_PINS[1]), encISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_PINS[2]), encISR2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_PINS[3]), encISR3, CHANGE);
}

// ===================== 电机控制 =====================
const int PWM_FREQ = 1000;
const int PWM_RES  = 8;

void setupMotors() {
  for (int i = 0; i < 4; i++) {
    ledcSetup(i*2,     PWM_FREQ, PWM_RES);
    ledcSetup(i*2 + 1, PWM_FREQ, PWM_RES);
    ledcAttachPin(MOTOR_PINS[i][0], i*2);
    ledcAttachPin(MOTOR_PINS[i][1], i*2 + 1);
  }
}

void setMotor(int wheel, int duty) {
  duty = constrain(duty, -255, 255);
  if (duty >= 0) {
    ledcWrite(wheel*2,     duty);
    ledcWrite(wheel*2 + 1, 0);
  } else {
    ledcWrite(wheel*2,     0);
    ledcWrite(wheel*2 + 1, -duty);
  }
}

void stopAll() {
  for (int i = 0; i < 4; i++) setMotor(i, 0);
}

// ===================== 工具 =====================
void clearEncoder(int w) {
  noInterrupts();
  encoderCount[w] = 0;
  interrupts();
}

long getEncoder(int w) {
  noInterrupts();
  long c = encoderCount[w];
  interrupts();
  return c;
}

// 计算平均轮速 (rad/s)
// 采样时间 sample_ms，返回平均角速度
float measureOmega(int wheel, int duty, int sample_ms = 1500) {
  clearEncoder(wheel);
  setMotor(wheel, duty);
  delay(300);               // 先让速度稳定
  clearEncoder(wheel);      // 稳定后再开始正式计

  unsigned long t0 = millis();
  while (millis() - t0 < (unsigned long)sample_ms) {
    delay(10);
  }
  long counts = getEncoder(wheel);
  setMotor(wheel, 0);

  // 脉冲数 → 弧度
  // 注意：单通道 CHANGE 触发，一圈约 C1REV 个边沿
  float revolutions = (float)counts / C1REV[wheel];
  float omega = revolutions * 2.0f * PI * (1000.0f / sample_ms);  // rad/s
  return omega;
}

// ===================== 主测试流程 =====================
void calibrateWheel(int wheel) {
  Serial.printf("\n\n==============================\n");
  Serial.printf("开始标定轮子 %s\n", WHEEL_NAME[wheel]);
  Serial.printf("==============================\n");

  // 推荐测量点（可根据死区微调）
  int duties[] = {50, 60, 80, 100, 120, 140, 160, 180, 200, 220, 240};
  int n = sizeof(duties) / sizeof(duties[0]);

  // ----- 正向 -----
  Serial.println("\n【正向】duty → ω (rad/s)");
  Serial.println("duty, omega");
  for (int i = 0; i < n; i++) {
    int d = duties[i];
    float w = measureOmega(wheel, d, 1500);
    Serial.printf("%d, %.2f\n", d, w);
    delay(400);  // 冷却一下
  }

  // ----- 反向 -----
  Serial.println("\n【反向】duty → ω (rad/s)");
  Serial.println("duty, omega");
  for (int i = 0; i < n; i++) {
    int d = -duties[i];
    float w = measureOmega(wheel, d, 1500);
    // 反向时速度取绝对值方便记录，符号后面自己加
    Serial.printf("%d, %.2f\n", d, w);
    delay(400);
  }

  Serial.printf("\n%s 标定完成！请把上面两段数据复制保存。\n", WHEEL_NAME[wheel]);
}

void printMenu() {
  Serial.println("\n========== 占空比-轮速标定菜单 ==========");
  Serial.println("输入数字选择轮子：");
  Serial.println("  0 = FL");
  Serial.println("  1 = FR");
  Serial.println("  2 = RL");
  Serial.println("  3 = RR");
  Serial.println("  a = 自动依次标定全部四个轮（推荐）");
  Serial.println("  s = 全部停止");
  Serial.println("=========================================");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  setupMotors();
  setupEncoders();
  stopAll();
  Serial.println("占空比 → 轮速 密集标定程序已启动");
  Serial.println("重要：请把轮子悬空！");
  printMenu();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    while (Serial.available()) Serial.read();

    if (c >= '0' && c <= '3') {
      calibrateWheel(c - '0');
      printMenu();
    }
    else if (c == 'a' || c == 'A') {
      for (int i = 0; i < 4; i++) {
        calibrateWheel(i);
        delay(1000);
      }
      Serial.println("\n\n***** 全部四轮标定完成 *****");
      printMenu();
    }
    else if (c == 's' || c == 'S') {
      stopAll();
      Serial.println("已全部停止");
    }
  }
}
