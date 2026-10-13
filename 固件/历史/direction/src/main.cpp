/*
 * 麦轮小车 —— 轮到方向检查程序 (ESP32)
 *
 * 目的：只确认“给正占空比时，每个轮子实际是向前还是向后转”。
 * 不读编码器、不跑 PID。
 *
 * 行为：烧录/复位后 3 秒，自动依次给
 *       FL -> FR -> DL -> DR  +130 占空比、各 2 秒。
 *       请盯住小车，记住每轮方向。
 *       之后可在串口输入命令重复。
 *
 * 命令：a=再跑一遍全部；1~4=只转对应轮(+130,2s)；s=停止；h=帮助
 *
 * 接线（用户提供）：
 *   FL 电机 13,15   编码器A 32
 *   FR 电机 12,14   编码器A 34
 *   DL 电机 27,26   编码器A 35
 *   DR 电机 25,33   编码器A 36
 */

#include <Arduino.h>

static const char* WNAME[4] = {"FL", "FR", "DL", "DR"};

// 每个轮两个方向脚：(pin0, pin1)
static const int MOTOR_PINS[4][2] = {
  {13, 15},  // FL
  {12, 14},  // FR
  {27, 26},  // DL
  {25, 33},  // DR
};

static const int PWM_FREQ = 1000;  // 1 kHz
static const int PWM_RES  = 8;     // 8-bit -> 0..255

// 方向检查用的占空比
static const int CHECK_DUTY = 130;
static const int CHECK_MS   = 2000;

static void setupMotors() {
  for (int i = 0; i < 4; i++) {
    ledcSetup(i * 2,     PWM_FREQ, PWM_RES);
    ledcSetup(i * 2 + 1, PWM_FREQ, PWM_RES);
    ledcAttachPin(MOTOR_PINS[i][0], i * 2);
    ledcAttachPin(MOTOR_PINS[i][1], i * 2 + 1);
  }
}

// duty: -255..255  （正=pin0 输出 PWM；负=pin1 输出 PWM）
static void setMotor(int w, int duty) {
  duty = constrain(duty, -255, 255);
  if (duty >= 0) { ledcWrite(w * 2, duty); ledcWrite(w * 2 + 1, 0);      }
  else           { ledcWrite(w * 2, 0);    ledcWrite(w * 2 + 1, -duty);  }
}

static void stopAll() { for (int i = 0; i < 4; i++) setMotor(i, 0); }

static void spinOne(int w, int duty, int ms) {
  Serial.printf(">>> %s 轮   占空比 %+d   持续 %d ms <<<\n", WNAME[w], duty, ms);
  setMotor(w, duty);
  delay(ms);
  setMotor(w, 0);
}

static void directionRoutine() {
  Serial.println("\n===== 方向检查：每轮依次 +130，各 2 秒 =====");
  Serial.println("请盯住小车，记住每个轮是【向前】还是【向后】。\n");
  for (int i = 0; i < 4; i++) {
    spinOne(i, CHECK_DUTY, CHECK_MS);
    delay(1500);
  }
  Serial.println("\n===== 完成。请回报每个轮的方向（向前/向后）。=====\n");
}

static void help() {
  Serial.println("\n----- 命令 -----");
  Serial.println("  a : 再跑一遍完整方向检查 (FL,FR,DL,DR 依次 +130)");
  Serial.println("  1 / 2 / 3 / 4 : 只让对应轮正转 (+130) 2 秒");
  Serial.println("  s : 全部停止");
  Serial.println("  h : 显示本帮助");
  Serial.println("----------------\n");
}

void setup() {
  Serial.begin(115200);
  setupMotors();
  stopAll();
  delay(500);
  Serial.println("\n\n=== 麦轮方向检查程序 ===");
  Serial.printf("引脚: FL(13,15) FR(12,14) DL(27,26) DR(25,33)  PWM %dHz %dbit\n",
                PWM_FREQ, PWM_RES);
  help();
  Serial.println("3 秒后自动开始方向检查，请看向小车 ...");
  delay(3000);
  directionRoutine();
  Serial.println("(要看第二遍：串口输入 a 回车)");
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();
  while (Serial.available()) Serial.read();  // 清空

  if (c == 'a' || c == 'A') {
    directionRoutine();
  } else if (c >= '1' && c <= '4') {
    spinOne(c - '1', CHECK_DUTY, CHECK_MS);
  } else if (c == 's' || c == 'S') {
    stopAll();
    Serial.println("[stop] 全部停止");
  } else if (c == 'h' || c == 'H') {
    help();
  }
}
