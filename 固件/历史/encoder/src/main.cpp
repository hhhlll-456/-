/*
 * 麦轮小车 —— 编码器(仅A相) + 死区 自动测试 (ESP32)
 *
 * 前提（用户提供）：
 *   编码器只接了 A 相：FL=32, FR=34, DL=35, DR=36
 *   电机脚：FL(13,15) FR(12,14) DL(27,26) DR(25,33)
 *
 * 注意：34/35/36 无内部上拉。若编码器是开漏输出，需外接 ~10k 上拉到 3.3V。
 *       本程序第一步就是“静止噪声检查”，用来自动判断上拉够不够。
 *
 * 本程序自动跑：
 *   [1] 静止 3 秒，看每个轮编码器计数是否乱跳   -> 判断接线/上拉
 *   [2] 每个轮 正向死区扫描 (duty 从 0 递增，找最小能转起来的)
 *   [3] 每个轮 反向死区扫描
 * 仅 A 相 -> 只能得到“计数大小”，得不到方向增减符号。
 *
 * 复位后 5 秒开始（留时间给串口抓取）。
 * 完成后等待命令：r=重跑一遍  s=停止  h=帮助
 */

#include <Arduino.h>

static const char* WNAME[4] = {"FL", "FR", "DL", "DR"};

static const int MOTOR_PINS[4][2] = {
  {13, 15},  // FL
  {12, 14},  // FR
  {27, 26},  // DL
  {25, 33},  // DR
};

// 编码器 A 相
static const int ENC_PINS[4] = {32, 34, 35, 36};

// 让该轮“向前”转所需的 duty 符号（由方向检查得出：左− 右+）
static const int FWD_SIGN[4] = {-1, +1, -1, +1};

static const int PWM_FREQ = 1000;
static const int PWM_RES  = 8;

static const int NOISE_MS      = 3000;  // 静止观察时长
static const long NOISE_LIMIT  = 8;     // 超过此计数变化判为“噪声/悬空”
static const int DZ_MAX        = 84;    // 死区扫描上限
static const int DZ_STEP       = 4;
static const int DZ_WIN_MS     = 400;   // 每个 duty 观察窗口
static const long DZ_MOVE_CNT  = 3;     // 窗口内计数变化 >= 此值算“动了”

// ---------------- 编码器 ----------------
volatile long enc[4] = {0, 0, 0, 0};

void IRAM_ATTR isr0() { enc[0]++; }
void IRAM_ATTR isr1() { enc[1]++; }
void IRAM_ATTR isr2() { enc[2]++; }
void IRAM_ATTR isr3() { enc[3]++; }
static void (*ISRFN[4])() = {isr0, isr1, isr2, isr3};

static void setupEncoders() {
  for (int i = 0; i < 4; i++) {
    pinMode(ENC_PINS[i], INPUT_PULLUP);   // 34/35/36 上此项无效
    attachInterrupt(digitalPinToInterrupt(ENC_PINS[i]), ISRFN[i], CHANGE);
  }
}

static long readEnc(int w)  { noInterrupts(); long v = enc[w]; interrupts(); return v; }
static void clearEnc(int w) { noInterrupts(); enc[w] = 0;  interrupts(); }

// ---------------- 电机 ----------------
static void setupMotors() {
  for (int i = 0; i < 4; i++) {
    ledcSetup(i * 2,     PWM_FREQ, PWM_RES);
    ledcSetup(i * 2 + 1, PWM_FREQ, PWM_RES);
    ledcAttachPin(MOTOR_PINS[i][0], i * 2);
    ledcAttachPin(MOTOR_PINS[i][1], i * 2 + 1);
  }
}

static void setMotor(int w, int duty) {
  duty = constrain(duty, -255, 255);
  if (duty >= 0) { ledcWrite(w * 2, duty); ledcWrite(w * 2 + 1, 0);     }
  else           { ledcWrite(w * 2, 0);    ledcWrite(w * 2 + 1, -duty); }
}

static void stopAll() { for (int i = 0; i < 4; i++) setMotor(i, 0); }

// ---------------- 测试 ----------------
static bool noiseCheck() {
  Serial.println("\n[1] 静止噪声检查（3 秒，电机全停）...");
  stopAll();
  delay(600);
  for (int i = 0; i < 4; i++) clearEnc(i);
  delay(NOISE_MS);

  bool noisy = false;
  for (int i = 0; i < 4; i++) {
    long c = readEnc(i);
    bool bad = labs(c) > NOISE_LIMIT;
    if (bad) noisy = true;
    Serial.printf("    %s: 静止计数变化 = %4ld  %s\n",
                  WNAME[i], c, bad ? "<- 异常(悬空/缺上拉?)" : "ok");
  }
  if (noisy) {
    Serial.println("    >> 有轮子静止时乱跳。34/35/36 无内部上拉，");
    Serial.println("       请给这三个脚各加 ~10k 上拉到 3.3V，或改用带内部上拉的 GPIO。");
  }
  return !noisy;
}

// dir: +1=前进, -1=后退 ; 返回找到的最小 |duty|（0 = 未找到）
static int findDeadzone(int w, int dir) {
  for (int d = DZ_STEP; d <= DZ_MAX; d += DZ_STEP) {
    clearEnc(w);
    setMotor(w, dir * FWD_SIGN[w] * d);
    delay(DZ_WIN_MS);
    long c = readEnc(w);
    setMotor(w, 0);
    Serial.printf("    %s %s |duty|=%3d -> 计数 %4ld%s\n",
                  WNAME[w], dir > 0 ? "正向" : "反向", d, c,
                  labs(c) >= DZ_MOVE_CNT ? "  <== 动了" : "");
    if (labs(c) >= DZ_MOVE_CNT) return d;
    delay(120);
  }
  return 0;
}

static int dzF[4], dzR[4];

static void runAll() {
  Serial.println("\n\n########## 编码器 / 死区 自动测试 ##########");

  bool ok = noiseCheck();

  if (!ok) {
    Serial.println("\n[2][3] 静止噪声过大，死区结果会不可信 -> 跳过。");
    Serial.println("      请先加上拉电阻，再重跑（输入 r）。");
    return;
  }

  Serial.println("\n[2] 正向死区扫描（每个轮，duty 递增找最小起转值）");
  for (int i = 0; i < 4; i++) dzF[i] = findDeadzone(i, +1);

  Serial.println("\n[3] 反向死区扫描");
  for (int i = 0; i < 4; i++) dzR[i] = findDeadzone(i, -1);

  Serial.println("\n########## 结果汇总 ##########");
  Serial.println("轮   正向死区   反向死区");
  for (int i = 0; i < 4; i++) {
    Serial.printf("%s   %6d   %6d%s\n", WNAME[i], dzF[i], dzR[i],
                  (dzF[i] == 0 || dzR[i] == 0) ? "   (0=未找到)" : "");
  }
  Serial.println("##############################");
}

static void help() {
  Serial.println("\n命令: r=重跑全部测试  s=停止  h=帮助\n");
}

void setup() {
  Serial.begin(115200);
  setupMotors();
  setupEncoders();
  stopAll();
  Serial.println("\n\n=== 编码器/死区测试程序 ===");
  Serial.printf("编码器A: FL=%d FR=%d DL=%d DR=%d  (仅A相)\n",
                ENC_PINS[0], ENC_PINS[1], ENC_PINS[2], ENC_PINS[3]);
  help();
  Serial.println("5 秒后自动开始，请勿触碰小车 ...");
  delay(5000);
  runAll();
  Serial.println("\n== 测试完成。r=重跑 ==\n");
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();
  while (Serial.available()) Serial.read();
  if (c == 'r' || c == 'R') runAll();
  else if (c == 's' || c == 'S') { stopAll(); Serial.println("[stop]"); }
  else if (c == 'h' || c == 'H') help();
}
