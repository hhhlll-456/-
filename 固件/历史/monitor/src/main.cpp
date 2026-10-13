/*
 * 麦轮小车 —— 编码器手动转动计数监视 (ESP32)
 *
 * 用途：测“单圈计数 C_1rev”。电机全停，人手动把某个轮子精确转 N 圈，
 *       读该轮编码器累计计数，C_1rev = 计数 / N。
 *
 * 编码器仅 A 相：FL=32 FR=34 DL=35 DR=36
 * 无内部上拉(34/35/36)：若读数静止乱跳，需外接 ~10k 上拉。
 *
 * 每 250ms 打印一次四轮累计计数，格式：
 *   t=12345  FL=0  FR=0  DL=0  DR=0
 * 命令： z=清零四轮计数   h=帮助
 */

#include <Arduino.h>

static const char* WNAME[4] = {"FL", "FR", "DL", "DR"};
static const int ENC_PINS[4] = {32, 34, 35, 36};

// 电机脚：必须初始化并驱动为 0，否则 H 桥输入悬空 -> 电机会自己乱转
static const int MOTOR_PINS[4][2] = {
  {13, 15},  // FL
  {12, 14},  // FR
  {27, 26},  // DL
  {25, 33},  // DR
};
static const int PWM_FREQ = 1000;
static const int PWM_RES  = 8;

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

volatile long enc[4] = {0, 0, 0, 0};

void IRAM_ATTR isr0() { enc[0]++; }
void IRAM_ATTR isr1() { enc[1]++; }
void IRAM_ATTR isr2() { enc[2]++; }
void IRAM_ATTR isr3() { enc[3]++; }
static void (*ISRFN[4])() = {isr0, isr1, isr2, isr3};

static long readEnc(int w) { noInterrupts(); long v = enc[w]; interrupts(); return v; }
static void clearAll()     { noInterrupts(); for (int i = 0; i < 4; i++) enc[i] = 0; interrupts(); }

void setup() {
  Serial.begin(115200);
  setupMotors();                 // 先把电机脚驱动为 0，防止悬空自转
  for (int i = 0; i < 4; i++) {
    pinMode(ENC_PINS[i], INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_PINS[i]), ISRFN[i], CHANGE);
  }
  delay(500);
  Serial.println("\n\n=== 编码器计数监视 (手动转动) ===");
  Serial.println("电机已停。手动把轮子转 N 整圈，读计数 -> C_1rev = 计数/N");
  Serial.println("命令: z=清零  h=帮助");
  Serial.println("格式: t=毫秒 FL= FR= DL= DR=");
}

void loop() {
  // 命令
  if (Serial.available()) {
    char c = Serial.read();
    while (Serial.available()) Serial.read();
    if (c == 'z' || c == 'Z') { clearAll(); Serial.println("[zero] 计数已清零"); }
    else if (c == 'h' || c == 'H')
      Serial.println("命令: z=清零四轮计数");
  }

  static uint32_t t0 = 0;
  if (millis() - t0 < 250) return;
  t0 = millis();
  Serial.printf("t=%-7lu FL=%-7ld FR=%-7ld DL=%-7ld DR=%-7ld\n",
                (unsigned long)millis(),
                readEnc(0), readEnc(1), readEnc(2), readEnc(3));
}
