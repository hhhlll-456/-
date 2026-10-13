/*
 * 麦轮小车 —— K_PWM 标定 (轮速 -> PWM 比例系数)
 *
 * 模型：施加占空比 = K_PWM * ω + 死区    =>  K_PWM = (duty - DZ) / ω
 *
 * 方法：对每个轮、每个方向，在若干固定占空比下让轮子空转，
 *       待转速稳定后，用编码器在固定时间窗内计脉冲，算出 ω (rad/s)。
 *       输出 (duty, ω) 数据点，供线性拟合 duty = K*ω + DZ。
 *
 * 已知（实测）：
 *   编码器仅 A 相，CHANGE 双边沿：C_1rev = FL1276 FR1270 DL1276 DR1274
 *   死区：DZ_POS = 40,48,56,44   DZ_NEG = 52,48,68,60
 *   前进方向符号：FL/DL = -1, FR/DR = +1
 *
 * 前提：小车已悬空（轮子可自由转），电机电源已接。
 * 复位后 5 秒开始（留时间给串口抓取）。
 */

#include <Arduino.h>

static const char* WNAME[4] = {"FL", "FR", "DL", "DR"};

static const int MOTOR_PINS[4][2] = {
  {13, 15},  // FL
  {12, 14},  // FR
  {27, 26},  // DL
  {25, 33},  // DR
};
static const int ENC_PINS[4] = {32, 34, 35, 36};
static const int FWD_SIGN[4] = {-1, +1, -1, +1};

static const float C1REV[4] = {1276, 1270, 1276, 1274};

static const int PWM_FREQ = 1000;
static const int PWM_RES  = 8;

// 测试点：占空比（都明显高于最大死区 68）
static const int DUTY[6] = {80, 110, 140, 170, 200, 230};
static const int ND = 6;

static const int SPINUP_MS = 700;   // 换挡后等转速稳定
static const int MEAS_MS   = 1000;  // 计脉冲窗口

// ---------------- 编码器 ----------------
volatile long enc[4] = {0, 0, 0, 0};
void IRAM_ATTR isr0() { enc[0]++; }
void IRAM_ATTR isr1() { enc[1]++; }
void IRAM_ATTR isr2() { enc[2]++; }
void IRAM_ATTR isr3() { enc[3]++; }
static void (*ISRFN[4])() = {isr0, isr1, isr2, isr3};

static long readEnc(int w)  { noInterrupts(); long v = enc[w]; interrupts(); return v; }
static void clearEnc(int w) { noInterrupts(); enc[w] = 0;  interrupts(); }

// ---------------- 电机 ----------------
static void setupMotors() {
  for (int i = 0; i < 4; i++) {
    ledcSetup(i * 2,     PWM_FREQ, PWM_RES);
    ledcSetup(i * 2 + 1, PWM_FREQ, PWM_RES);
    ledcAttachPin(MOTOR_PINS[i][0], i * 2);
    ledcAttachPin(MOTOR_PINS[i][1], i * 2 + 1);
    ledcWrite(i * 2, 0);
    ledcWrite(i * 2 + 1, 0);
  }
}
static void setMotor(int w, int duty) {
  duty = constrain(duty, -255, 255);
  if (duty >= 0) { ledcWrite(w * 2, duty); ledcWrite(w * 2 + 1, 0);     }
  else           { ledcWrite(w * 2, 0);    ledcWrite(w * 2 + 1, -duty); }
}
static void stopAll() { for (int i = 0; i < 4; i++) setMotor(i, 0); }

// ---------------- 标定 ----------------
static void runTest() {
  Serial.println("\n\n########## K_PWM 标定 ##########");
  Serial.println("DATA 轮 方向 duty counts dtMs omega(rad/s)");

  for (int w = 0; w < 4; w++) {
    for (int s = 0; s < 2; s++) {
      int dir = (s == 0) ? +1 : -1;         // +1=前进, -1=后退
      for (int k = 0; k < ND; k++) {
        int d = DUTY[k];
        setMotor(w, dir * FWD_SIGN[w] * d);
        delay(SPINUP_MS);

        clearEnc(w);
        unsigned long t0 = millis();
        delay(MEAS_MS);
        long c = readEnc(w);
        unsigned long dt = millis() - t0;
        setMotor(w, 0);

        float rev   = (float)c / C1REV[w];
        float omega = rev * 2.0f * PI / (dt / 1000.0f);

        Serial.printf("DATA %s %s %3d %6ld %4lu %.4f\n",
                      WNAME[w], dir > 0 ? "F" : "R", d, c, dt, omega);
        delay(150);
      }
      stopAll();
      delay(250);
    }
  }
  stopAll();
  Serial.println("########## DONE ##########");
}

void setup() {
  Serial.begin(115200);
  setupMotors();               // 先把电机脚驱动为 0
  stopAll();
  for (int i = 0; i < 4; i++) {
    pinMode(ENC_PINS[i], INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_PINS[i]), ISRFN[i], CHANGE);
  }
  delay(500);
  Serial.println("\n\n=== K_PWM 标定程序 ===");
  Serial.println("5 秒后开始，请勿触碰小车（轮子需空转）...");
  delay(5000);
  runTest();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    while (Serial.available()) Serial.read();
    if (c == 'r' || c == 'R') runTest();
    else if (c == 's' || c == 'S') { stopAll(); Serial.println("[stop]"); }
  }
}
